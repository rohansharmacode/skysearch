"""CLI: orbit a pile, run LiDAR SLAM, write the map + landing cells + metrics.

    python -m mapping.run --seed 7 --pile B2 --out dashboard/data/map_B2 [--plot docs/slam.png]

Writes into --out:
  pointcloud.bin      float32 x,y,z triples, site frame (cropped to --crop m around the pile)
  heightmap.json      Grid fields as in state.json "heightmap" (+ "observed" 0/1 mask, "source")
  landing_cells.json  landing sites detected on the SLAM vs the true heightmap, and their agreement
  metrics.json        ATE, map-to-truth height error, landing agreement, runtime
"""
from __future__ import annotations

import argparse
import json
import os

import numpy as np

from sim.scenario import make_scenario

from .slam import run_pile_slam


def _atomic_json(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f, separators=(",", ":"))
    os.replace(tmp, path)


def write_outputs(r, out, crop=20.0):
    os.makedirs(out, exist_ok=True)
    cloud, g, c = r["cloud"], r["grid"], r["centre"]
    if crop:
        cloud = cloud[np.hypot(cloud[:, 0] - c[0], cloud[:, 1] - c[1]) <= crop]
    cloud.astype("<f4").tofile(os.path.join(out, "pointcloud.bin"))
    _atomic_json(os.path.join(out, "heightmap.json"), {
        "origin": [round(g.origin[0], 3), round(g.origin[1], 3)], "res": g.res, "w": g.w, "h": g.h,
        "z": [round(float(v), 3) for v in g.z.ravel()],
        "observed": g.observed.astype(int).ravel().tolist(), "source": r["method"],
        "pointcloud": {"url": "pointcloud.bin", "count": int(len(cloud))}})
    m = r["metrics"]
    if "landing" in r:
        L = r["landing"]
        _atomic_json(os.path.join(out, "landing_cells.json"), {
            "pile": m["pile"], "centre": [float(c[0]), float(c[1])], "summary": L["summary"],
            "odom_only_summary": m.get("landing_odom_only"), "slam_sites": L["slam_sites"],
            "truth_sites": L["truth_sites"], "slam_site_checks": L["slam_site_checks"]})
    _atomic_json(os.path.join(out, "metrics.json"), m)
    return len(cloud)


def summary(m):
    L, he = m.get("landing", {}), m.get("height_error", {})
    lines = [f"[mapping] pile {m['pile']} seed {m['seed']}: method {m['method']}"
             + (f"  NOTE: {m['note']}" if m["note"] else ""),
             f"  sweeps {m['n_scans']} x ~{m['points_per_scan']} pts -> map {m['map_points']:,} pts @ {m['map_voxel_m']} m voxels"]
    if "ate_rmse_m" in m:
        lines += [
            f"  ATE RMSE {m['ate_rmse_m'] * 100:.1f} cm (max {m['ate_max_m'] * 100:.1f}), rot {m['rot_err_mean_deg']:.2f} deg"
            f" | odometry only {m['odom_ate_rmse_m']:.2f} m, final drift {m['odom_final_drift_m']:.2f} m",
            f"  point error mean {m['point_err_mean_m'] * 100:.1f} cm, p95 {m['point_err_p95_m'] * 100:.1f} cm",
            f"  heightmap vs truth (r<=14 m, {he['cells']} cells): MAE {he['mae_m'] * 100:.1f} cm, RMSE {he['rmse_m'] * 100:.1f} cm,"
            f" {he['frac_within_10cm'] * 100:.0f}% within 10 cm | vs GT-pose LiDAR map: MAE "
            f"{m['height_error_vs_gt_pose_lidar']['mae_m'] * 100:.1f} cm",
            f"  landing cells: IoU {L['feasible_cell_iou']:.3f} (recall {L['feasible_cell_recall']:.3f}, precision "
            f"{L['feasible_cell_precision']:.3f}); SLAM sites {L['slam_candidates']}, valid on truth "
            f"{L['slam_candidates_valid_on_truth']}, truth sites {L['truth_candidates']}"]
    rt = m["runtime"]
    lines.append("  runtime " + ", ".join(f"{k} {v:.1f}s" for k, v in rt.items()))
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--pile", default="B2")
    ap.add_argument("--scans", type=int, default=300, help="sweeps at 10 Hz over one orbit (<= 300)")
    ap.add_argument("--method", choices=["kiss", "gt_noise"], default="kiss")
    ap.add_argument("--threads", type=int, default=2)
    ap.add_argument("--crop", type=float, default=20.0, help="radius (m) of the written cloud, 0 = all")
    ap.add_argument("--out", default="dashboard/data/map_B2")
    ap.add_argument("--plot", help="write the summary figure (e.g. docs/slam.png)")
    a = ap.parse_args(argv)
    sc = make_scenario(a.seed)
    r = run_pile_slam(sc, a.pile, seed=a.seed, n_scans=a.scans, method=a.method, threads=a.threads)
    write_outputs(r, a.out, a.crop)
    if a.plot:
        from .plot import plot_slam
        plot_slam(r, a.plot)
    print(summary(r["metrics"]))
    print(f"  wrote {a.out}/{{pointcloud.bin,heightmap.json,landing_cells.json,metrics.json}}"
          + (f" and {a.plot}" if a.plot else ""))


if __name__ == "__main__":
    main()
