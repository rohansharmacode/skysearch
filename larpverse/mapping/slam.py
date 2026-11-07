"""LiDAR SLAM over a pile orbit: KISS-ICP registration with odometry priors -> site cloud -> heightmap.

Pipeline (``build_pile_map``):
  1. ``raycast_lidar.simulate_orbit``: spinning-LiDAR sweeps (sensor frame) + drifting odometry.
  2. KISS-ICP 1.x (MIT): each sweep is registered to KISS-ICP's local voxel map; the initial guess
     is ``last_pose @ odom_delta`` (the odometry increment replaces KISS-ICP's constant-velocity
     model).  The trajectory is anchored at the take-off pose.  If KISS-ICP is missing or diverges
     (estimate > ``DIVERGE_M`` from odometry), fall back to "GT pose + noise" mapping and say so.
  3. Sweeps are transformed by the estimated poses and voxel-downsampled (Open3D if importable,
     else numpy voxel hashing) into the site cloud, then rasterised with
     ``sim.world.heightmap_from_points`` on a pile-centred window aligned to the site grid.
  4. ``sim.landing.detect_landing_sites`` runs on the SLAM heightmap.

GROUND-TRUTH BOUNDARY: steps 2-4 only see sensor-frame sweeps and odometry.  Ground-truth poses
and the true heightmap are used afterwards, only for the evaluation metrics (ATE, map error,
landing-cell agreement).  ``Scenario.truth`` (victims, sounds) is never read here.
"""
from __future__ import annotations

import os
import time

import numpy as np

from sim.landing import (LIM_CLEAR, LIM_ROUGH, LIM_SLOPE, LIM_STEP, SEARCH_RADIUS,
                         _clearance_step_approach, detect_landing_sites, patch_fields)
from sim.scenario import SITE_BOUNDS, Scenario
from sim.world import Grid, heightmap_from_points

from .raycast_lidar import LidarConfig, simulate_orbit

MAP_VOXEL = 0.10        # m, output cloud
ICP_VOXEL = 0.20        # m, KISS-ICP local map voxel
HM_RES = 0.5            # m, same as the site heightmap
HALF_WINDOW = 16.0      # m, heightmap window around the pile
DIVERGE_M = 5.0         # m, |SLAM - odometry| that counts as divergence


# --------------------------------------------------------------------------- utilities
def voxel_down_sample(pts: np.ndarray, voxel: float) -> np.ndarray:
    """Centroid per voxel. Open3D when available (MAPPING_NO_O3D=1 forces numpy)."""
    if len(pts) == 0:
        return pts.astype(np.float32)
    if not os.environ.get("MAPPING_NO_O3D"):
        try:
            import open3d as o3d
            pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts.astype(np.float64)))
            return np.asarray(pc.voxel_down_sample(voxel).points, dtype=np.float32)
        except ImportError:
            pass
    key = np.floor(pts / voxel).astype(np.int64)
    _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    inv = inv.ravel()
    out = np.zeros((len(cnt), 3))
    np.add.at(out, inv, pts)
    return (out / cnt[:, None]).astype(np.float32)


def transform(T, pts):
    return pts @ T[:3, :3].T + T[:3, 3]


def rasterise(cloud, origin, w, h, res=HM_RES):
    """heightmap_from_points on the window only (it clamps outside points onto the edge cells)."""
    x1, y1 = origin[0] + w * res, origin[1] + h * res
    inside = (cloud[:, 0] >= origin[0]) & (cloud[:, 0] < x1) & (cloud[:, 1] >= origin[1]) & (cloud[:, 1] < y1)
    return heightmap_from_points(cloud[inside], origin, res, w, h)


def ate(est, gt):
    e = np.linalg.norm(est[:, :3, 3] - gt[:, :3, 3], axis=1)
    return float(np.sqrt((e ** 2).mean())), float(e.max()), float(e[-1])


def rot_err_deg(est, gt):
    R = np.einsum("nji,njk->nik", est[:, :3, :3], gt[:, :3, :3])
    c = np.clip((np.trace(R, axis1=1, axis2=2) - 1) / 2, -1, 1)
    return float(np.degrees(np.arccos(c)).mean())


def pile_window(centre, half=HALF_WINDOW, res=HM_RES):
    """(origin, w, h) of a square window around ``centre`` snapped to the site heightmap grid."""
    x0 = SITE_BOUNDS[0] + np.floor((centre[0] - half - SITE_BOUNDS[0]) / res) * res
    y0 = SITE_BOUNDS[1] + np.floor((centre[1] - half - SITE_BOUNDS[1]) / res) * res
    n = int(round(2 * half / res))
    return (float(x0), float(y0)), n, n


# --------------------------------------------------------------------------- registration
def run_kiss_icp(scans, odom_poses, max_range=40.0, min_range=0.5, voxel=ICP_VOXEL, threads=2,
                 max_points_per_voxel=5, max_iterations=20):
    """KISS-ICP with odometry priors. Returns site-frame 4x4 poses anchored at odom_poses[0]."""
    from kiss_icp.config import KISSConfig
    from kiss_icp.kiss_icp import KissICP

    cfg = KISSConfig()
    cfg.data.max_range = max_range
    cfg.data.min_range = min_range
    cfg.data.deskew = False                   # each sweep is captured at a single pose
    cfg.mapping.voxel_size = voxel
    cfg.registration.max_num_threads = threads
    cfg.registration.max_num_iterations = max_iterations
    cfg.mapping.max_points_per_voxel = max_points_per_voxel
    icp = KissICP(cfg)
    rel = []
    for k, scan in enumerate(scans):
        if k > 0:   # odometry increment as the prior (replaces the constant-velocity model)
            icp.last_delta = np.linalg.inv(odom_poses[k - 1]) @ odom_poses[k]
        icp.register_frame(np.ascontiguousarray(scan, dtype=np.float64), np.zeros(len(scan)))
        rel.append(icp.last_pose.copy())
    return np.einsum("ij,njk->nik", odom_poses[0], np.array(rel))


def gt_plus_noise(gt_poses, seed=0, sigma_t=0.03, sigma_r_deg=0.2):
    """Fallback poses: ground truth with independent per-pose noise (labelled as such)."""
    from .raycast_lidar import _exp_so3
    rng = np.random.default_rng(seed)
    out = gt_poses.copy()
    for T in out:
        T[:3, 3] += rng.normal(0, sigma_t, 3)
        T[:3, :3] = T[:3, :3] @ _exp_so3(rng.normal(0, np.radians(sigma_r_deg), 3))
    return out


def accumulate(scans, poses, voxel=MAP_VOXEL, chunk=25):
    acc = np.zeros((0, 3), np.float32)
    buf = []
    for k, (s, T) in enumerate(zip(scans, poses)):
        buf.append(transform(T, s).astype(np.float32))
        if len(buf) == chunk or k == len(scans) - 1:
            acc = voxel_down_sample(np.concatenate([acc] + buf), voxel)
            buf = []
    return acc


# --------------------------------------------------------------------------- evaluation
def _feasible_mask(g: Grid, centre):
    slope, rough = patch_fields(g)[:2]
    X, Y = g.mesh()
    return (np.hypot(X - centre[0], Y - centre[1]) <= SEARCH_RADIUS) & (slope <= LIM_SLOPE) & (rough <= LIM_ROUGH)


def _valid_on(g: Grid, xy):
    """Would a landing at xy pass every landing.py check on grid g? Returns (ok, reason)."""
    slope, rough, a, b, mean = patch_fields(g)
    i, j = (int(v) for v in g.index(xy[0], xy[1]))
    pad = 14
    zpad = np.pad(g.z.astype(np.float64), pad, mode="edge")
    clear, step, app = _clearance_step_approach(zpad, i, j, pad, a[j, i], b[j, i], mean[j, i], g.res)
    for bad, why in ((slope[j, i] > LIM_SLOPE, "slope"), (rough[j, i] > LIM_ROUGH, "roughness"),
                     (step > LIM_STEP, "step"), (clear < LIM_CLEAR, "clearance"), (not app, "approach")):
        if bad:
            return False, why
    return True, None


def landing_agreement(slam_grid: Grid, truth_grid: Grid, centre):
    s_sites = detect_landing_sites(slam_grid, centre)
    t_sites = detect_landing_sites(truth_grid, centre)
    ms, mt = _feasible_mask(slam_grid, centre), _feasible_mask(truth_grid, centre)
    inter, union = int((ms & mt).sum()), int((ms | mt).sum())
    s_c = [s for s in s_sites if s["status"] == "candidate"]
    t_c = [s for s in t_sites if s["status"] == "candidate"]
    checks = []
    for s in s_c:
        ok, why = _valid_on(truth_grid, s["xy"])
        d = min((np.hypot(s["xy"][0] - t["xy"][0], s["xy"][1] - t["xy"][1]) for t in t_c), default=np.inf)
        checks.append({"id": s["id"], "xy": s["xy"], "valid_on_truth": ok, "truth_reason": why,
                       "nearest_truth_candidate_m": round(float(d), 2)})
    n_valid = sum(c["valid_on_truth"] for c in checks)
    return {
        "slam_sites": s_sites, "truth_sites": t_sites, "slam_site_checks": checks,
        "summary": {
            "slam_candidates": len(s_c), "truth_candidates": len(t_c),
            "slam_candidates_valid_on_truth": n_valid,
            "slam_candidate_precision": round(n_valid / max(1, len(s_c)), 3),
            "slam_candidates_within_1.5m_of_truth": sum(c["nearest_truth_candidate_m"] <= 1.5 for c in checks),
            "feasible_cells_slam": int(ms.sum()), "feasible_cells_truth": int(mt.sum()),
            "feasible_cell_iou": round(inter / max(1, union), 3),
            "feasible_cell_recall": round(inter / max(1, int(mt.sum())), 3),
            "feasible_cell_precision": round(inter / max(1, int(ms.sum())), 3),
        },
    }


def height_error(g: Grid, truth: Grid, observed, centre, radius=SEARCH_RADIUS):
    X, Y = g.mesh()
    m = observed & (np.hypot(X - centre[0], Y - centre[1]) <= radius)
    e = (g.z - truth.z)[m]
    if not len(e):
        return {}
    a = np.abs(e)
    return {"cells": int(m.sum()), "coverage": round(float(m.sum() / max(1, (np.hypot(X - centre[0], Y - centre[1]) <= radius).sum())), 3),
            "mae_m": round(float(a.mean()), 4), "rmse_m": round(float(np.sqrt((e ** 2).mean())), 4),
            "bias_m": round(float(e.mean()), 4), "p95_abs_m": round(float(np.percentile(a, 95)), 4),
            "frac_within_10cm": round(float((a <= 0.10).mean()), 3)}


# --------------------------------------------------------------------------- main entry
def run_pile_slam(scenario: Scenario, pile_name: str, seed: int = 0, n_scans: int = 300,
                  method: str = "kiss", cfg: LidarConfig | None = None, threads: int = 2,
                  evaluate: bool = True, revs: float = 1.0):
    """Full run; returns a dict with cloud, grids, poses, landing sites and metrics.

    ``revs`` = orbit revolutions flown during the ``n_scans`` sweeps (1.0: one loop in 30 s at 300)."""
    timing = {}
    t0 = time.perf_counter()
    sim = simulate_orbit(scenario, pile_name, seed=seed, n_scans=n_scans, cfg=cfg, revs=revs)
    timing["raycast_s"] = time.perf_counter() - t0
    scans, gt, odom = sim["scans"], sim["gt_poses"], sim["odom_poses"]
    centre = sim["centre"]

    t1 = time.perf_counter()
    note = None
    est = None
    if method == "kiss":
        try:
            est = run_kiss_icp(scans, odom, max_range=sim["cfg"].max_range,
                               min_range=sim["cfg"].min_range, threads=threads)
            dev = np.linalg.norm(est[:, :3, 3] - odom[:, :3, 3], axis=1).max()
            if not np.isfinite(est).all() or dev > DIVERGE_M:
                note = f"KISS-ICP diverged (max |SLAM-odom| {dev:.1f} m); fell back to GT pose + noise"
                est = None
        except ImportError as exc:
            note = f"KISS-ICP unavailable ({exc}); fell back to GT pose + noise"
    if est is None:
        est = gt_plus_noise(gt, seed=seed + 3)
        method_used = "gt_plus_noise"
    else:
        method_used = "kiss_icp"
    timing["registration_s"] = time.perf_counter() - t1

    t2 = time.perf_counter()
    cloud = accumulate(scans, est)
    origin, w, h = pile_window(centre)
    grid = rasterise(cloud, origin, w, h)
    cnt = np.zeros((h, w), int)
    ix = ((cloud[:, 0] - origin[0]) / HM_RES).astype(int)
    iy = ((cloud[:, 1] - origin[1]) / HM_RES).astype(int)
    ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    np.add.at(cnt, (iy[ok], ix[ok]), 1)
    grid.observed = cnt > 0
    timing["mapping_s"] = time.perf_counter() - t2
    timing["total_s"] = time.perf_counter() - t0

    out = {"cloud": cloud, "grid": grid, "centre": centre, "gt_poses": gt, "odom_poses": odom,
           "est_poses": est, "method": method_used, "note": note}
    metrics = {"pile": pile_name, "seed": seed, "method": method_used, "note": note,
               "n_scans": len(scans), "points_per_scan": int(np.mean([len(s) for s in scans])),
               "map_points": int(len(cloud)), "map_voxel_m": MAP_VOXEL,
               "runtime": {k: round(v, 2) for k, v in timing.items()}}
    if evaluate:
        t3 = time.perf_counter()
        truth = scenario.heightmap(HM_RES, bounds=(origin[0], origin[1], origin[0] + w * HM_RES, origin[1] + h * HM_RES))
        a_rmse, a_max, a_end = ate(est, gt)
        o_rmse, o_max, o_end = ate(odom, gt)
        sub = slice(None, None, max(1, len(scans) // 30))     # point-level error on a subset of sweeps
        pe = np.concatenate([np.linalg.norm(transform(T, s) - tw, axis=1)
                             for s, T, tw in zip(scans[sub], est[sub], sim["truth_world"][sub])])
        gt_grid = rasterise(accumulate(scans, gt), origin, w, h)
        odom_cloud = accumulate(scans, odom)
        odom_grid = rasterise(odom_cloud, origin, w, h)
        land = landing_agreement(grid, truth, centre)
        land_odom = landing_agreement(odom_grid, truth, centre)["summary"]
        metrics.update({
            "ate_rmse_m": round(a_rmse, 4), "ate_max_m": round(a_max, 4), "final_drift_m": round(a_end, 4),
            "rot_err_mean_deg": round(rot_err_deg(est, gt), 3),
            "odom_ate_rmse_m": round(o_rmse, 4), "odom_final_drift_m": round(o_end, 4),
            "odom_rot_err_mean_deg": round(rot_err_deg(odom, gt), 3),
            "point_err_mean_m": round(float(pe.mean()), 4), "point_err_p95_m": round(float(np.percentile(pe, 95)), 4),
            "height_error": height_error(grid, truth, grid.observed, centre),
            "height_error_odom_only": height_error(odom_grid, truth, grid.observed, centre),
            # same LiDAR sweeps placed with GT poses: isolates the SLAM pose error from
            # rasterisation / occlusion effects that a perfect-pose LiDAR map also has
            "height_error_vs_gt_pose_lidar": height_error(grid, gt_grid, grid.observed, centre),
            "height_error_gt_pose_lidar_vs_truth": height_error(gt_grid, truth, grid.observed, centre),
            "landing": land["summary"], "landing_odom_only": land_odom,
        })
        metrics["runtime"]["evaluation_s"] = round(time.perf_counter() - t3, 2)
        out.update(truth_grid=truth, odom_grid=odom_grid, odom_cloud=odom_cloud, landing=land)
    out["metrics"] = metrics
    return out


def build_pile_map(scenario: Scenario, pile_name: str, seed: int = 0, n_scans: int = 300,
                   method: str = "kiss", evaluate: bool = True, **kw):
    """Orbit ``pile_name``, run LiDAR SLAM, return (points Nx3 float32 site frame, Grid, metrics).

    ``Grid`` covers a 32 m window around the pile at 0.5 m, aligned to the site heightmap grid,
    with an extra ``grid.observed`` (h, w) bool mask of cells that received LiDAR points.
    """
    r = run_pile_slam(scenario, pile_name, seed=seed, n_scans=n_scans, method=method, evaluate=evaluate, **kw)
    return r["cloud"], r["grid"], r["metrics"]
