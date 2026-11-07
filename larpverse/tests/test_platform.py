"""Fast tests for sim/platform.py (heavy-lift presets), the dense LiDAR config, isaac/lidar.py profiles and the
heavy drone model parts list.  No pxr / omni, no SLAM; well under 10 s."""
import argparse
import math

import numpy as np
import pytest

from sim import landing
from sim.platform import DEFAULT, PRESETS, get_platform, radar_rx_advantage_db, summary


def test_presets_realistic_and_sourced():
    assert DEFAULT == "heavy_sar" and get_platform().name == "heavy_sar"
    assert get_platform("heavy") is get_platform("heavy_sar") and get_platform("small").name == "legacy_small"
    h = get_platform("heavy_sar")
    assert 2.5 <= h.span_m <= 3.2 and h.n_rotors == 8 and h.layout == "coaxial-X8"
    assert 20 <= h.endurance_min <= 40 and 8 <= h.cruise_mps <= 12
    assert h.takeoff_mass_kg <= h.mtow_kg
    assert h.lidar.channels >= 32 and 3e5 <= h.lidar.pts_per_s <= 3e6
    assert h.radar.band_ghz == (0.4, 2.0) and h.mics.n_mics == 8 and 0.6 <= h.mics.aperture_m <= 1.0
    assert h.speaker is not None and h.speaker.spl_db_1m >= 120
    for name in ("heavy_sar", "industrial_sar"):
        p = PRESETS[name]
        assert any(v.startswith("https://") for v in p.sources.values()) and p.estimates
    with pytest.raises(KeyError):
        get_platform("fpv")
    s = summary(h)
    assert s["usable_min_25pct_reserve"] == pytest.approx(0.75 * h.endurance_min, abs=0.1)


def test_legacy_reproduces_sim_constants():
    lg = get_platform("legacy_small")
    assert lg.fly_rate == pytest.approx(0.0016, rel=1e-3)          # coordinator.KinematicFlight
    assert lg.idle_rate == pytest.approx(0.0005, rel=1e-3)
    fk = lg.flight_kwargs()
    assert (fk["speed"], fk["climb"], fk["descend"], fk["cruise_alt"]) == (4.0, 2.0, 1.2, 8.0)
    L = lg.landing_limits()
    assert L["clear_m"] == pytest.approx(landing.LIM_CLEAR) and L["step_m"] == landing.LIM_STEP
    assert L["slope_deg"] == landing.LIM_SLOPE and L["rough_m"] == landing.LIM_ROUGH
    assert L["perch_slope_deg"] == landing.PERCH_SLOPE and L["patch_cells"] == 3


def test_heavy_needs_bigger_stricter_landing_and_lasts_longer():
    h, lg = get_platform("heavy_sar"), get_platform("legacy_small")
    H = h.landing_limits()
    assert H["patch_m"] >= 3.0 and H["patch_cells"] % 2 == 1
    assert H["clear_m"] >= h.span_m / 2
    assert H["slope_deg"] < landing.LIM_SLOPE and H["perch_slope_deg"] < landing.PERCH_SLOPE
    assert "edge" in H["prefer"]
    assert h.fly_rate < lg.fly_rate / 2                              # >2x endurance
    assert h.static_load_n() > 15 * lg.static_load_n()               # the structural concern
    assert 4 < h.downwash_mps() < 12


def test_sensor_advantages():
    h, ind, lg = (get_platform(n) for n in ("heavy_sar", "industrial_sar", "legacy_small"))
    assert radar_rx_advantage_db(h) > radar_rx_advantage_db(ind) > 0 == radar_rx_advantage_db(lg)
    assert h.radar.rx_gain_dbi(1.2) > h.radar.rx_gain_dbi(0.6)
    assert h.radar.beamwidth_deg(1.2) < lg.radar.beamwidth_deg(1.2)
    assert h.mics.beamwidth_deg(1000) < lg.mics.beamwidth_deg(1000) / 2
    assert h.speaker.spl_at(20.0) == pytest.approx(h.speaker.spl_db_1m - 20 * math.log10(20))


def test_lidar_config_from_spec_and_legacy_default():
    from mapping.raycast_lidar import LidarConfig
    legacy = LidarConfig()
    assert legacy.n_beams == 23040 and legacy.tilt_deg == 0 and np.allclose(legacy.mount(), np.eye(3))
    os1 = LidarConfig.from_spec(get_platform("heavy_sar"))
    assert os1.channels == 128 and os1.n_beams == 128 * 1024 and os1.pts_per_s == pytest.approx(1310720)
    assert np.allclose(os1.mount() @ [0, 0, 1], [2 ** -0.5, 0, 2 ** -0.5])   # spin axis pitched 45 deg forward
    q = LidarConfig.from_spec(get_platform("heavy_sar"), sim_fraction=0.1)
    sub = q.beam_subset(np.random.default_rng(0))
    assert len(sub) == q.sim_beams and np.all(np.diff(sub) > 0)
    assert q.directions(np.random.default_rng(0), sub).shape == (len(sub), 3)


@pytest.fixture(scope="module")
def geom():
    from mapping.raycast_lidar import SceneGeometry
    from sim.scenario import make_scenario
    sc = make_scenario(7)
    p = sc.pile("B2")
    return sc, SceneGeometry(sc, (p.x, p.y), half_extent=30.0)


def test_dense_tilted_subset_scan(geom):
    from mapping.raycast_lidar import LidarConfig, orbit_poses, simulate_scan
    sc, g = geom
    p = sc.pile("B2")
    T = orbit_poses((p.x, p.y), 10)[0][3]
    cfg = LidarConfig.from_spec(get_platform("heavy_sar"), sim_fraction=0.05)
    loc, w = simulate_scan(g, T, cfg, np.random.default_rng(1))
    assert 0.2 * cfg.sim_beams < len(loc) <= cfg.sim_beams          # ~half the tilted fan looks at the sky
    assert np.allclose(loc @ T[:3, :3].T + T[:3, 3], w, atol=1e-6)  # points are body-frame
    above = w[:, 2] - sc.ground_height(w[:, 0], w[:, 1])
    assert (above > -0.15).mean() > 0.99


def test_box_fast_path_with_tilt_and_subset(geom):
    from mapping.raycast_lidar import LidarConfig, orbit_poses
    sc, g = geom
    p = sc.pile("B2")
    T = orbit_poses((p.x, p.y), 10)[0][5]
    cfg = LidarConfig.from_spec(get_platform("industrial_sar"), sim_fraction=0.3)
    rng = np.random.default_rng(2)
    sub = cfg.beam_subset(rng)
    M = cfg.mount()
    D = cfg.directions(rng, sub) @ M.T @ T[:3, :3].T
    fast = g.cast_boxes(T[:3, 3], D, 40.0, grid=(T[:3, :3] @ M, cfg, sub))
    dense = g.cast_boxes(T[:3, 3], D, 40.0)
    assert np.isfinite(dense).sum() > 50
    assert np.array_equal(np.isfinite(fast), np.isfinite(dense))
    m = np.isfinite(fast)
    assert np.allclose(fast[m], dense[m])


# ------------------------------------------------------------------------------------------- isaac side
def _floor_cast(o, d, r):
    if d[2] >= -1e-6:
        return None
    t = -o[2] / d[2]
    return None if t > r else (o[0] + d[0] * t, o[1] + d[1] * t, 0.0)


def test_lidar_profiles_and_budget():
    from isaac.lidar import COST_US_ASSUMED, PROFILES, add_lidar_args, make_lidar, profile
    a = add_lidar_args(argparse.ArgumentParser()).parse_args([])
    assert a.lidar_profile == "heavy" and a.lidar_budget_ms is None
    small = make_lidar("physx", profile("small"), cast=_floor_cast)
    assert small.n_rays == 96 and small.budget_ms is None
    heavy = make_lidar("physx", profile("heavy"), cast=_floor_cast)
    assert heavy.n_rays == int(PROFILES["heavy"]["budget_ms"] * 1000 / COST_US_ASSUMED)   # assumed cost first
    pts = heavy.sweep((0.0, 0.0, 10.0), 0.0, 0)
    assert heavy.cost_measured and len(pts) > 0 and np.all(np.abs(pts[:, 2]) < 0.5)
    heavy.cost_us = 1000.0                                           # pretend a slow box: shrinks to min
    assert heavy._rays_for_budget() == PROFILES["heavy"]["min_rays"]
    pinned = make_lidar("physx", profile("heavy", rays=300), cast=_floor_cast)
    pinned.sweep((0.0, 0.0, 10.0))
    assert pinned.n_rays == 300                                      # explicit --rays wins over the budget
    assert "OS1_REV7_128ch10hz1024res" in PROFILES["heavy"]["rtx_configs"]


def test_point_accumulator_vectorised_matches_voxel_set():
    from isaac.lidar import PointAccumulator
    rng = np.random.default_rng(0)
    P = rng.uniform(-3, 3, (6000, 3)).astype(np.float32)
    acc = PointAccumulator(0.15, 10 ** 6)
    n = sum(len(acc.add("A", c)) for c in np.array_split(P, 4))
    want = len(set(map(tuple, np.floor(P / 0.15).astype(int))))
    assert n == acc.count("A") == want
    capped = PointAccumulator(0.15, 500)
    for c in np.array_split(P, 3):
        capped.add("A", c)
    assert capped.count("A") == 500 and len(capped.add("A", P)) == 0


def test_heavy_drone_parts_from_spec():
    from isaac.drone_viz import MODELS, NullDrone, heavy_parts, model_extent
    assert MODELS[0] == "heavy"
    spec = get_platform("heavy_sar")
    parts = heavy_parts(spec, "d2")
    names = {p[1] for p in parts}
    assert sum(p[0] == "Rotor" for p in parts) == 8
    assert {"LidarPuck", "RadarPanel", "SpeakerHorn", "Thermal", "Skid0", "Skid2"} <= names
    assert sum(n.startswith("Mic") for n in names) == spec.mics.n_mics
    ex, ey, z0, z1 = model_extent(parts)
    assert abs(max(ex, ey) - spec.span_m) < 0.3 and z0 == pytest.approx(0.0) and 1.0 < z1 < 1.6
    NullDrone(None, "/World/Drones/d1", model="heavy").set_pose(0, 0, 1, 0)   # dry-run interface unchanged
