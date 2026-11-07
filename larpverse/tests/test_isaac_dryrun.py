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


def test_flags_merged_once():
    """Our flags + the Isaac teammate's photoreal/recording flags: every option registered exactly once."""
    from isaac.compat import add_kit_args
    from isaac.run_mission import _base_parser, parse_args

    ap = _base_parser()
    add_kit_args(ap)
    opts = [o for act in ap._actions for o in act.option_strings]
    assert len(opts) == len(set(opts))
    lost = [act.dest for act in ap._actions if not act.option_strings and act.nargs != 0 and act.dest != "help"]
    assert not lost, lost                                  # conflict_handler="resolve" would silently drop these
    for flag in ("--photoreal", "--record", "--rec-cams", "--rec-res", "--rec-fps", "--rec-speed", "--rec-warp",
                 "--rec-stop-t", "--rec-windows", "--rec-every", "--rtx", "--spp", "--hdri", "--sky", "--sun",
                 "--sun-elev", "--veg", "--operator", "--assets", "--launcher", "--selftest", "--look", "--flow",
                 "--fabric", "--dry-run", "--radar"):
        assert flag in opts, flag
    a = parse_args(["--dry-run"])
    assert (a.photoreal, a.sun, a.sky, a.radar, a.rec_res) == (0, 1800.0, 350.0, "dsp", "1920x1080")
    a = parse_args(["--dry-run", "--look", "pretty"])          # --look pretty is an alias for --photoreal 1
    assert (a.photoreal, a.sun, a.sky, a.hdri) == (1, 2200.0, 900.0, "evening_road_01_4k.hdr")
    a = parse_args(["--dry-run", "--photoreal", "1", "--sun", "3000"])
    assert (a.sun, a.sky) == (3000.0, 900.0)


def test_isaac_dryrun_photoreal_record_flags(tmp_path):
    """Their recording flags on the dry run: virtual clock at --rec-fps, --rec-speed, --rec-warp, --rec-stop-t,
    photoreal LiDAR draw radius. No recorder without Isaac; nothing Isaac-only is imported."""
    from isaac.run_mission import main

    s = main(["--dry-run", "--photoreal", "1", "--record", str(tmp_path / "rec"), "--rec-fps", "10",
              "--rec-speed", "6", "--rec-warp", "4", "--rec-stop-t", "30", "--out", str(tmp_path / "d"), "--quiet"])
    assert 28 <= s["t"] <= 40 and s["recorded_frames"] == 0, s
    assert s["lidar_points"] > 0
    assert not any(m == "pxr" or m.startswith(("pxr.", "omni.")) or m == "omni" for m in sys.modules)


def test_isaac_dryrun_weiguan(tmp_path):
    """--site weiguan on the dry run: scenario-owned bounds/staging reach the state; drones fly; LiDAR hits."""
    from isaac.run_mission import main

    out = tmp_path / "data"
    s = main(["--dry-run", "--site", "weiguan", "--max-steps", "40", "--out", str(out), "--quiet"])
    assert s["site"] == "weiguan" and s["steps"] == 40
    assert s["lidar_points"] > 500, s
    state = json.loads((out / "state.json").read_text())
    assert state["site"]["bounds"] == [-32.0, -64.0, 68.0, 42.0]
    assert state["staging"]["xy"] == [31.0, -55.0]
    assert {p["id"] for p in state.get("piles", [])} <= set("ABCDEFGHI")
