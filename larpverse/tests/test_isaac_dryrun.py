"""Isaac adapter --dry-run: the whole render loop minus omni/pxr (numpy heightmap LiDAR, null drawing)."""
import json
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)


def test_isaac_dryrun(tmp_path):
    from isaac.run_mission import main

    out = tmp_path / "data"
    s = main(["--dry-run", "--max-steps", "60", "--out", str(out), "--quiet"])

    assert s["steps"] == 60
    assert all(v > 5.0 for v in s["drone_moved_m"].values()), s["drone_moved_m"]   # drones flew
    assert s["lidar_points"] > 1000, s                                              # LiDAR produced points
    assert s["overlay_updates"] >= 60 and s["camera_updates"] > 60
    state = json.loads((out / "state.json").read_text())                             # state was written
    assert state["t"] >= 59 and len(state["drones"]) == 3
    if s["mission"] == "GridMission":                                                # points went back in
        assert sum(s["ingested"].values()) > 0
    assert not any(m == "pxr" or m.startswith(("pxr.", "omni.")) or m == "omni" for m in sys.modules)


def test_isaac_selftest_dryrun(tmp_path):
    """--selftest plumbing without Isaac: Isaac-only steps SKIP, the rest must PASS (or WARN if scipy is missing)."""
    from isaac.run_mission import main

    res = main(["--selftest", "--dry-run", "--out", str(tmp_path), "--quiet"])
    status = {r["step"]: r["status"] for r in res["rows"]}
    assert not res["counts"].get("FAIL"), res["rows"]
    for step in ("app_boot", "stage_build", "physics_parse", "raycast", "camera_switch", "debug_draw"):
        assert status[step] == "SKIP", (step, status)
    for step in ("mission_import", "lidar_sweep", "overlay_usd", "render_frame"):
        assert status[step] in ("PASS", "WARN"), (step, status)
    png = (tmp_path / "selftest_frame.png").read_bytes()
    assert png[:8] == b"\x89PNG\r\n\x1a\n" and len(png) > 100
    assert (tmp_path / "selftest.json").exists()
    assert not any(m == "pxr" or m.startswith(("pxr.", "omni.", "isaaclab")) or m in ("omni", "isaaclab")
                   for m in sys.modules)


def test_look_geometry_and_quat():
    """Pure-numpy helpers used on the GPU path: chipped rubble twins and the camera quaternion."""
    import numpy as np

    from isaac.camera import look_at
    from isaac.compat import mat_to_quat_wxyz
    from isaac.look import chipped_box, merge_pieces

    V, T = chipped_box(np.random.default_rng(0))
    assert V.shape == (24, 3) and T.shape == (44, 3)
    assert np.all(np.abs(V) <= 0.56)                                   # stays inside the collider box (+jitter)
    P, N, C = merge_pieces([((1, 2, 3), np.eye(3), (2.0, 1.0, 0.5), (0.5, 0.4, 0.3))])
    assert len(P) == 44 * 3 and np.allclose(np.linalg.norm(N, axis=1), 1, atol=1e-4)
    c = P.reshape(-1, 3, 3).mean(1) - np.array([1, 2, 3])                # outward normals
    assert (np.sum(c * N[::3], 1) > 0).all()
    eye, tgt = np.array([10.0, -20.0, 15.0]), np.array([0.0, 0.0, 1.0])
    M = look_at(eye, tgt)
    w, x, y, z = mat_to_quat_wxyz(M[:3, :3].T)
    # rotate the camera's -Z by the quaternion: must point at the target
    v = np.array([0.0, 0.0, -1.0])
    q = np.array([x, y, z])
    r = v + 2 * np.cross(q, np.cross(q, v) + w * v)
    d = (tgt - eye) / np.linalg.norm(tgt - eye)
    assert np.allclose(r, d, atol=1e-6)
