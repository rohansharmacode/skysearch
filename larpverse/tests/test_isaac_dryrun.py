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
