"""Fast tests for mapping/ (ray-cast LiDAR, KISS-ICP SLAM, heightmap, landing agreement)."""
import json

import numpy as np
import pytest

from mapping.raycast_lidar import LidarConfig, SceneGeometry, noisy_odometry, orbit_poses, simulate_orbit
from mapping.slam import build_pile_map, run_pile_slam
from sim.scenario import make_scenario

SMALL = LidarConfig(channels=16, az_step_deg=1.0)      # ~5.8k beams per sweep


@pytest.fixture(scope="module")
def sc():
    return make_scenario(7)


def test_raycast_hits_true_geometry(sc):
    s = simulate_orbit(sc, "B2", seed=1, n_scans=3, cfg=SMALL, revs=0.05)
    w = np.concatenate(s["truth_world"])
    assert 4000 < len(w) / 3 <= SMALL.channels * 360
    above = w[:, 2] - sc.ground_height(w[:, 0], w[:, 1])
    assert (above > -0.1).mean() > 0.99          # nothing sees through the ground
    assert (np.abs(above) < 0.08).mean() > 0.6   # most returns are terrain (2 cm range noise)
    assert (above > 0.15).mean() > 0.03          # some are rubble pieces on top
    r = np.linalg.norm(w - s["gt_poses"][0][:3, 3], axis=1)
    assert r.max() <= SMALL.max_range + 0.1


def test_box_fast_path_matches_dense(sc):
    p = sc.pile("B2")
    g = SceneGeometry(sc, (p.x, p.y))
    T = orbit_poses((p.x, p.y), 10)[0][4]
    d = SMALL.directions(np.random.default_rng(0))
    D = d @ T[:3, :3].T
    fast = g.cast_boxes(T[:3, 3], D, 40.0, grid=(T[:3, :3], SMALL))
    dense = g.cast_boxes(T[:3, 3], D, 40.0)
    assert np.isfinite(dense).sum() > 50
    assert np.array_equal(np.isfinite(fast), np.isfinite(dense))
    m = np.isfinite(fast)
    assert np.allclose(fast[m], dense[m])


def test_odometry_drifts():
    gt, _ = orbit_poses((0, 0), 100)
    od = noisy_odometry(gt, seed=0)
    assert np.allclose(od[0], gt[0])
    assert np.linalg.norm(od[-1][:3, 3] - gt[-1][:3, 3]) > 0.3


def test_kiss_icp_beats_odometry(sc):
    pytest.importorskip("kiss_icp")
    r = run_pile_slam(sc, "B2", seed=3, n_scans=40, cfg=SMALL, revs=0.15)
    m = r["metrics"]
    assert m["method"] == "kiss_icp", m["note"]
    assert m["ate_rmse_m"] < 0.15
    assert m["ate_rmse_m"] < m["odom_ate_rmse_m"]
    assert m["height_error"]["mae_m"] < 0.15
    assert m["landing"]["feasible_cell_iou"] > 0.8
    assert m["landing"]["slam_candidate_precision"] >= 0.75


def test_build_pile_map_contract_and_outputs(sc, tmp_path):
    from mapping.run import write_outputs

    pts, grid, metrics = build_pile_map(sc, "A1", seed=2, n_scans=6, method="gt_noise", cfg=SMALL, revs=0.05)
    assert pts.dtype == np.float32 and pts.shape[1] == 3 and len(pts) > 1000
    assert grid.z.shape == (grid.h, grid.w) == (64, 64)
    assert grid.observed.shape == grid.z.shape and grid.observed.mean() > 0.5
    p = sc.pile("A1")
    assert grid.bounds[0] < p.x < grid.bounds[2] and grid.bounds[1] < p.y < grid.bounds[3]
    assert metrics["method"] == "gt_plus_noise" and "ate_rmse_m" in metrics

    r = run_pile_slam(sc, "A1", seed=2, n_scans=6, method="gt_noise", cfg=SMALL, revs=0.05)
    n = write_outputs(r, str(tmp_path))
    assert (tmp_path / "pointcloud.bin").stat().st_size == 12 * n
    hm = json.loads((tmp_path / "heightmap.json").read_text())
    assert len(hm["z"]) == hm["w"] * hm["h"] and hm["res"] == 0.5
    lc = json.loads((tmp_path / "landing_cells.json").read_text())
    assert {"summary", "slam_sites", "truth_sites"} <= lc.keys()
    assert json.loads((tmp_path / "metrics.json").read_text())["pile"] == "A1"
