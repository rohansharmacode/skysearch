"""Numpy ray-casting LiDAR against the true scenario geometry (SYNTHETIC sensor).

Geometry: bare terrain ``Scenario.ground_height`` (ray-marched on a fine pre-sampled grid, then
linearly refined) plus every box ``Piece`` of the piles (vectorised ray-OBB slab test with a
bounding-sphere prefilter).  The nearest hit wins; range noise is Gaussian along the beam.

Sensor: a spinning 32-channel unit tilted to look down (elevations -18 to -80 deg below the body
horizon, 0.5 deg azimuth step, 10 Hz, max range 40 m), i.e. ~23k beams per sweep.  Each sweep is
taken at one pose (no intra-sweep motion), so KISS-ICP deskewing is switched off.

Trajectory: one orbit around a pile at 9.5 m radius, 11 m altitude, scans at 10 Hz, with a small
altitude bob and roll/pitch wobble.  Ground-truth poses are known; ``noisy_odometry`` integrates
perturbed increments (scale error, yaw-rate bias, random walk) to produce a drifting VIO-like prior.

Frames: site frame (x east, y north, z up).  Sensor/body frame: x forward, y left, z up.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from sim.scenario import Scenario, rot_matrix

MAX_RANGE = 40.0
MIN_RANGE = 0.5
RANGE_SIGMA = 0.02


@dataclass
class LidarConfig:
    channels: int = 32
    el_min_deg: float = -80.0
    el_max_deg: float = -18.0
    az_step_deg: float = 0.5
    max_range: float = MAX_RANGE
    min_range: float = MIN_RANGE
    range_sigma: float = RANGE_SIGMA

    def directions(self):
        """Unit beam directions in the sensor frame, (channels * n_az, 3)."""
        el = np.radians(np.linspace(self.el_min_deg, self.el_max_deg, self.channels))
        az = np.radians(np.arange(0.0, 360.0, self.az_step_deg))
        E, A = np.meshgrid(el, az, indexing="ij")
        return np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], -1).reshape(-1, 3)


# --------------------------------------------------------------------------- poses
def pose_matrix(xyz, rpy_deg):
    T = np.eye(4)
    T[:3, :3] = rot_matrix(rpy_deg)
    T[:3, 3] = xyz
    return T


def orbit_poses(centre_xy, n_scans=300, radius=9.5, altitude=11.0, rate_hz=10.0, revs=1.0,
                phase0=-np.pi / 2, seed=0):
    """Ground-truth 4x4 poses (site frame) of a drone orbiting ``centre_xy`` counter-clockwise."""
    rng = np.random.default_rng(seed)
    t = np.arange(n_scans) / rate_hz
    T_orbit = n_scans / rate_hz / revs
    th = phase0 + 2 * np.pi * t / T_orbit
    ph = rng.uniform(0, 2 * np.pi, 3)
    poses = []
    for k, a in enumerate(th):
        x = centre_xy[0] + radius * np.cos(a)
        y = centre_xy[1] + radius * np.sin(a)
        z = altitude + 0.3 * np.sin(2 * np.pi * t[k] / 7.0 + ph[0])
        yaw = np.degrees(a) + 90.0                      # heading along the orbit tangent
        roll = 3.0 * np.sin(2 * np.pi * t[k] / 3.1 + ph[1])
        pitch = 3.0 * np.sin(2 * np.pi * t[k] / 4.3 + ph[2])
        poses.append(pose_matrix((x, y, z), (roll, pitch, yaw)))
    return np.array(poses), t


def _exp_so3(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def noisy_odometry(gt_poses, seed=0, scale_err=0.03, yaw_bias_deg_s=0.5, rate_hz=10.0,
                   trans_sigma=0.01, rot_sigma_deg=0.05):
    """Drifting odometry (VIO-like): compounded GT increments with scale error, yaw-rate bias and
    per-step noise.  Anchored at the first GT pose (take-off pose known from GNSS / the pad)."""
    rng = np.random.default_rng(seed)
    out = [gt_poses[0].copy()]
    bias = np.radians(yaw_bias_deg_s) / rate_hz
    for k in range(1, len(gt_poses)):
        d = np.linalg.inv(gt_poses[k - 1]) @ gt_poses[k]
        dn = np.eye(4)
        dn[:3, 3] = d[:3, 3] * (1 + scale_err) + rng.normal(0, trans_sigma, 3)
        w = rng.normal(0, np.radians(rot_sigma_deg), 3)
        w[2] += bias
        dn[:3, :3] = d[:3, :3] @ _exp_so3(w)
        out.append(out[-1] @ dn)
    return np.array(out)


# --------------------------------------------------------------------------- geometry
class SceneGeometry:
    """True geometry around a pile, prepared for fast ray casting."""

    def __init__(self, scenario: Scenario, centre_xy, half_extent=46.0, terrain_res=0.1):
        self.sc = scenario
        cx, cy = centre_xy
        n = int(round(2 * half_extent / terrain_res)) + 1
        self.t_origin = (cx - half_extent, cy - half_extent)
        self.t_res = terrain_res
        xs = self.t_origin[0] + np.arange(n) * terrain_res
        ys = self.t_origin[1] + np.arange(n) * terrain_res
        X, Y = np.meshgrid(xs, ys)
        self.terrain = scenario.ground_height(X, Y).astype(np.float32)   # (ny, nx), samples at nodes
        self.t_n = n
        self.zmin, self.zmax = float(self.terrain.min()), float(self.terrain.max())
        self._tflat = self.terrain.ravel()

        C, H, Rs = [], [], []
        for p in scenario.piles:
            if np.hypot(p.x - cx, p.y - cy) > half_extent + 10:
                continue
            for pc in p.pieces:
                C.append(pc.center)
                H.append(np.asarray(pc.size) / 2)
                Rs.append(rot_matrix(pc.rpy_deg))
        self.box_c = np.asarray(C, float).reshape(-1, 3)
        self.box_h = np.asarray(H, float).reshape(-1, 3)
        self.box_R = np.asarray(Rs, float).reshape(-1, 3, 3)
        self.box_r = np.linalg.norm(self.box_h, axis=1)

    def terrain_z(self, x, y):
        fx = np.clip((x - self.t_origin[0]) / self.t_res, 0, self.t_n - 1.000001)
        fy = np.clip((y - self.t_origin[1]) / self.t_res, 0, self.t_n - 1.000001)
        i0 = fx.astype(np.int64)
        j0 = fy.astype(np.int64)
        tx = fx - i0
        ty = fy - j0
        z, n = self._tflat, self.t_n
        k = j0 * n + i0
        return ((z[k] * (1 - tx) + z[k + 1] * tx) * (1 - ty)
                + (z[k + n] * (1 - tx) + z[k + n + 1] * tx) * ty)

    def cast_terrain(self, o, D, max_range, n_samples=24, refine=2):
        """First terrain hit distance per ray (inf if none): coarse march between the terrain's
        z-range planes, then regula-falsi refinement inside the bracketing step."""
        t = np.full(len(D), np.inf)
        dz = D[:, 2]
        live = (dz < -1e-3) & ((o[2] - self.zmax - 0.05) < -dz * max_range)
        if not live.any():
            return t
        idx = np.nonzero(live)[0]
        Dd = D[idx]
        t0 = np.clip((o[2] - self.zmax - 0.05) / -Dd[:, 2], 0, max_range)
        t1 = np.clip((o[2] - self.zmin + 0.05) / -Dd[:, 2], 0, max_range)
        s = np.linspace(0.0, 1.0, n_samples)[None, :]
        T = t0[:, None] + (t1 - t0)[:, None] * s                      # (n, K)

        def f(tt, rows=None):
            d = Dd if rows is None else Dd[rows]
            if tt.ndim == 2:
                return o[2] + d[:, 2:3] * tt - self.terrain_z(o[0] + d[:, 0:1] * tt, o[1] + d[:, 1:2] * tt)
            return o[2] + d[:, 2] * tt - self.terrain_z(o[0] + d[:, 0] * tt, o[1] + d[:, 1] * tt)

        F = f(T)                                                      # >0 above ground
        below = F <= 0
        has = below.any(1) & (F[:, 0] > 0)
        k = np.argmax(below, axis=1)[has]
        rows = np.nonzero(has)[0]
        ta, tb = T[rows, k - 1], T[rows, k]
        fa, fb = F[rows, k - 1], F[rows, k]
        for _ in range(refine):
            tm = ta + fa / (fa - fb + 1e-12) * (tb - ta)
            fm = f(tm, rows)
            up = fm > 0
            ta, fa = np.where(up, tm, ta), np.where(up, fm, fa)
            tb, fb = np.where(up, tb, tm), np.where(up, fb, fm)
        th = ta + fa / (fa - fb + 1e-12) * (tb - ta)
        t[idx[rows]] = th
        return t

    def cast_boxes(self, o, D, max_range, grid=None):
        """Nearest ray-OBB hit distance per ray (inf if none), vectorised slab test.

        ``grid`` = (R_sensor, cfg) enables the fast path: candidate (ray, box) pairs come from each
        box's bounding-sphere footprint in the sensor's (elevation, azimuth) beam grid instead of a
        dense rays x boxes test."""
        t = np.full(len(D), np.inf)
        if not len(self.box_c):
            return t
        v = self.box_c - o
        dist = np.linalg.norm(v, axis=1)
        near = np.nonzero((dist < max_range + self.box_r) & (dist > self.box_r))[0]
        if not len(near):
            return t
        if grid is not None:
            ri, bj = self._beam_pairs(v[near], dist[near], self.box_r[near], *grid)
            bj = near[bj]
        else:
            r = self.box_r[near]
            proj = D.astype(np.float32) @ v[near].T.astype(np.float32)
            thr = np.sqrt(dist[near] ** 2 - r ** 2).astype(np.float32)
            ri, bj = np.nonzero(proj >= thr[None, :])
            bj = near[bj]
        if not len(ri):
            return t
        Rb = self.box_R[bj]                                          # (P, 3, 3)
        o_l = np.einsum("pji,pj->pi", Rb, -v[bj])                     # R^T (o - c)
        d_l = np.einsum("pji,pj->pi", Rb, D[ri])
        d_l = np.where(np.abs(d_l) < 1e-12, 1e-12, d_l)
        h = self.box_h[bj]
        t1 = (-h - o_l) / d_l
        t2 = (h - o_l) / d_l
        tmin = np.minimum(t1, t2).max(1)
        tmax = np.maximum(t1, t2).min(1)
        hit = (tmax >= tmin) & (tmin > 0)
        np.minimum.at(t, ri[hit], tmin[hit])
        return t

    @staticmethod
    def _beam_pairs(v, dist, r, R, cfg):
        """(ray, box) index pairs whose beam may hit the box's bounding sphere (conservative)."""
        vs = v @ R                                                    # sensor frame: R^T v
        el = np.arcsin(np.clip(vs[:, 2] / dist, -1, 1))
        az = np.arctan2(vs[:, 1], vs[:, 0])
        rho = np.arcsin(np.clip(r / dist, 0, 1))
        C = cfg.channels
        n_az = int(round(360.0 / cfg.az_step_deg))
        e0, de = np.radians(cfg.el_min_deg), np.radians(cfg.el_max_deg - cfg.el_min_deg) / (C - 1)
        da = np.radians(cfg.az_step_deg)
        c0 = np.clip(np.floor((el - rho - e0) / de), 0, C).astype(int)
        c1 = np.clip(np.ceil((el + rho - e0) / de), -1, C - 1).astype(int)
        el_ext = np.minimum(np.maximum(np.abs(el - rho), np.abs(el + rho)), np.radians(89.9))
        sr = np.sin(rho) / np.cos(el_ext)
        full = sr >= 1
        raz = np.arcsin(np.clip(sr, 0, 1))
        a0 = np.where(full, 0, np.floor((az - raz) / da)).astype(int)
        a1 = np.where(full, n_az - 1, np.ceil((az + raz) / da)).astype(int)
        a1 = np.minimum(a1, a0 + n_az - 1)
        nc = np.maximum(c1 - c0 + 1, 0)
        na = a1 - a0 + 1
        cnt = nc * na
        tot = int(cnt.sum())
        if tot == 0:
            return np.zeros(0, int), np.zeros(0, int)
        bj = np.repeat(np.arange(len(v)), cnt)
        off = np.arange(tot) - np.repeat(np.cumsum(cnt) - cnt, cnt)
        ch = c0[bj] + off // na[bj]
        a = (a0[bj] + off % na[bj]) % n_az
        return ch * n_az + a, bj


def simulate_scan(geom: SceneGeometry, pose, cfg: LidarConfig, rng, dirs=None):
    """One sweep. Returns (points_sensor (n,3) float64, points_world_truth (n,3)) of valid returns."""
    dirs = cfg.directions() if dirs is None else dirs
    R, o = pose[:3, :3], pose[:3, 3]
    D = dirs @ R.T
    t = np.minimum(geom.cast_terrain(o, D, cfg.max_range), geom.cast_boxes(o, D, cfg.max_range, grid=(R, cfg)))
    ok = np.isfinite(t) & (t <= cfg.max_range) & (t >= cfg.min_range)
    rn = t[ok] + rng.normal(0, cfg.range_sigma, ok.sum())
    local = dirs[ok] * rn[:, None]
    world = o + D[ok] * rn[:, None]
    return local, world


def simulate_orbit(scenario: Scenario, pile_name: str, seed=0, n_scans=300, cfg: LidarConfig | None = None,
                   radius=9.5, altitude=11.0, revs=1.0, rate_hz=10.0):
    """Fly one orbit around a pile and scan. Returns dict with scans, gt/odom poses, timestamps."""
    cfg = cfg or LidarConfig()
    p = scenario.pile(pile_name)
    geom = SceneGeometry(scenario, (p.x, p.y))
    gt, ts = orbit_poses((p.x, p.y), n_scans, radius, altitude, rate_hz, revs, seed=seed)
    odom = noisy_odometry(gt, seed=seed + 1, rate_hz=rate_hz)
    rng = np.random.default_rng(seed + 2)
    dirs = cfg.directions()
    scans, truth_world = [], []
    for T in gt:
        loc, wld = simulate_scan(geom, T, cfg, rng, dirs)
        scans.append(loc)
        truth_world.append(wld.astype(np.float32))
    return {"scans": scans, "truth_world": truth_world, "gt_poses": gt, "odom_poses": odom,
            "timestamps": ts, "centre": (p.x, p.y), "cfg": cfg}
