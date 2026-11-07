"""Fast sanity tests for the grid Bayesian filter (no ground-truth module involved)."""
import numpy as np

from sim.localise import Localiser, predicted_range_field
from sim.world import Grid


def flat_grid():
    n = 100
    return Grid((-25.0, -25.0), 0.5, n, n, np.full((n, n), 1.0, np.float32))


def obs_for(grid, loc, sid, sensor_xy, truth_xy, quality=0.9, noise=0.0):
    pose = [sensor_xy[0], sensor_xy[1], 1.3, 0.0]
    r, _ = predicted_range_field(grid, pose, loc.cfg.depth_nom)
    ix, iy = grid.index(*truth_xy)
    rng_true = float(r[iy, ix]) + noise
    return {"scan_id": sid, "drone_id": "d1", "sensor_pose": pose, "timestamp": 0.0,
            "range_candidates": [rng_true], "confidence": [0.9], "quality": quality,
            "breathing_hz": 0.26, "doppler_moving": False}


def test_trilateration_converges_with_three_good_ranges():
    g = flat_grid()
    loc = Localiser(g)
    truth = (2.0, -1.5)
    for k, s in enumerate([(-9, 4), (3, 9), (7, -8)]):
        loc.update_radar(obs_for(g, loc, f"s{k}", s, truth))
    res = loc.extract()
    assert res["status"] == "candidate", res["reason"]
    m = res["modes"][0]
    assert np.hypot(m["xy"][0] - truth[0], m["xy"][1] - truth[1]) < 1.0
    assert m["p_mass"] > 0.7


def test_two_ranges_give_mirror_ambiguity():
    g = flat_grid()
    loc = Localiser(g)
    truth = (3.0, 3.0)
    for k, s in enumerate([(-10, -2), (10, -2)]):  # sensors on the line y = -2
        loc.update_radar(obs_for(g, loc, f"s{k}", s, truth))
    res = loc.extract()
    sig = [m for m in res["modes"] if m["p_mass"] >= loc.cfg.significant_min]
    assert len(sig) == 2, [m["p_mass"] for m in res["modes"]]
    assert res["status"] == "insufficient_evidence"
    # the two modes mirror each other across the sensor-sensor line y = -2
    (x1, y1), (x2, y2) = sig[0]["xy"], sig[1]["xy"]
    assert abs(x1 - x2) < 1.5 and abs((y1 + y2) / 2 + 2) < 1.5 and abs(y1 - y2) > 5


def test_moving_observations_do_not_touch_stationary_belief():
    g = flat_grid()
    loc = Localiser(g)
    before = loc.belief().copy()
    o = obs_for(g, loc, "m1", (0, -10), (12, 3))
    o["doppler_moving"] = True
    assert loc.update_radar(o) == "moving"
    assert np.allclose(loc.belief(), before)


def test_empty_scan_is_not_evidence_of_absence():
    g = flat_grid()
    loc = Localiser(g)
    before = loc.belief().copy()
    o = obs_for(g, loc, "s0", (0, -10), (1, 1))
    o["range_candidates"], o["confidence"] = [], []
    assert loc.update_radar(o) == "empty"
    assert np.allclose(loc.belief(), before)
