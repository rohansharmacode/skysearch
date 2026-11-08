"""Tests for the SYNTHETIC IF generator + radar DSP chain (sim/radar_dsp.py)."""
import numpy as np
import pytest

from sim import radar_dsp as R

P = R.RadarParams()
SCHEMA = {"scan_id", "drone_id", "sensor_pose", "timestamp", "range_candidates", "confidence", "quality",
          "breathing_hz", "doppler_moving", "max_reliable_depth_m", "vitals"}


def make(seed, range_m=8.0, path=0.5, q=0.95, hz=0.27, breather=True, fan=False, mover=False):
    rng = np.random.default_rng(seed)
    sc = R.Scene(quality=q)
    sc.clutter = R.default_clutter(rng)
    if breather:
        sc.breathers = [R.Breather(range_m, R.reflection_amplitude(range_m, path), hz, float(rng.uniform(4, 8)))]
    if fan:
        sc.vibrators = [R.Vibrator(11.0, R.reflection_amplitude(11.0, 0.0, 24.0), 1.2, 3.0)]
    if mover:
        sc.movers = [R.Mover(17.0 - 0.5 * P.t_slow, R.reflection_amplitude(17.0, 0.0, 8.0))]
    return R.process_if(R.simulate_if(sc, P, rng), P)


def test_radar_parameters():
    assert (P.f_start, P.f_stop) == (0.4e9, 2.0e9)
    assert P.range_res == pytest.approx(0.0937, abs=1e-3)
    assert P.n_slow == 400 and P.fs_slow == 20.0
    assert 19 < P.max_range < 50


@pytest.mark.parametrize("seed,range_m,hz", [(0, 8.0, 0.27), (1, 6.0, 0.22), (2, 10.0, 0.31), (3, 7.3, 0.25)])
def test_good_snr_range_and_rate(seed, range_m, hz):
    res = make(seed, range_m=range_m, hz=hz)
    assert res.range_candidates, "no detection at good SNR"
    assert abs(res.range_candidates[0] - range_m) < 0.15
    assert abs(res.breathing_hz - hz) < 0.03
    assert res.confidence[0] > 0.8
    assert not res.doppler_moving


def test_low_quality_lowers_confidence_and_quality():
    def conf_at(q):
        out = []
        for s in range(6):
            r = make(s, range_m=6.0, path=1.0, q=q)
            hit = [c for rg, c in zip(r.range_candidates, r.confidence) if abs(rg - 6.0) < 0.3]
            out.append((max(hit) if hit else 0.0, r.quality))
        return np.mean([o[0] for o in out]), np.mean([o[1] for o in out])

    c_hi, q_hi = conf_at(0.95)
    c_lo, q_lo = conf_at(0.1)
    assert c_lo < c_hi
    assert q_lo < q_hi - 0.3


def test_moving_target_sets_doppler_moving():
    res = make(0, mover=True)
    assert res.doppler_moving
    assert res.moving["speed_mps"] == pytest.approx(-0.5, abs=0.1)
    assert abs(res.moving["range_m"] - 7.0) < 1.0           # range at the end of the dwell
    obs = res.observations("s1", "d1", [0, 0, 0.4, 0], 20.0)
    assert [o["doppler_moving"] for o in obs] == [False, True]
    assert abs(obs[0]["range_candidates"][0] - 8.0) < 0.15   # the stationary breather is still found
    assert not make(0).doppler_moving


def test_fan_1p2hz_is_not_respiration():
    res = make(0, breather=False, fan=True)
    assert res.range_candidates == [] and res.breathing_hz is None
    assert any(abs(n["range_m"] - 11.0) < 0.3 and abs(n["freq_hz"] - 1.2) < 0.1 for n in res.periodic_nonhuman)
    # with a real breather present, only the breather is reported
    both = make(1, fan=True)
    assert len(both.range_candidates) >= 1
    assert abs(both.range_candidates[0] - 8.0) < 0.15
    assert all(abs(r - 11.0) > 0.5 for r in both.range_candidates)


def test_observation_schema_exact():
    res = make(0)
    obs = res.observations("s7", "d3", [1.0, 2.0, 0.4, 135.0], 41.2)[0]
    assert set(obs) == SCHEMA
    assert len(obs["range_candidates"]) == len(obs["confidence"])
    assert all(0.0 <= c <= 1.0 for c in obs["confidence"]) and 0.0 <= obs["quality"] <= 1.0
    assert obs["doppler_moving"] is False


def test_no_breather_no_confident_detection():
    for s in range(5):
        res = make(s, breather=False)
        assert all(c < 0.5 for c in res.confidence)



# ------------------------------------------------------------------ through-rubble penetration limit
@pytest.fixture
def literature():
    old = R.penetration_model()
    R.use_penetration("literature")
    yield
    R.use_penetration(old)


def test_penetration_physics_numbers(literature):
    # FHWA rule of thumb: ~10 dB/m one-way below 1 GHz, ~20 dB/m above; voids dilute it
    assert R.concrete_db_per_m(0.6e9) == pytest.approx(10.0) and R.concrete_db_per_m(2.5e9) == pytest.approx(20.0)
    losses = [R.two_way_loss_db(L) for L in (0.0, 1.0, 2.0, 3.0, 10.0)]
    assert losses[0] == pytest.approx(0.0, abs=1e-6) and all(b > a for a, b in zip(losses, losses[1:]))
    assert losses[-1] > 120                                 # 10 m of rubble: hopeless at any range
    # recalibrated with the physio chest waveform (docs/RADAR_PENETRATION.md): small legacy radar, 20 s dwell ~1.9 m;
    # heavy_sar pod (0.48 m face, +7.6 dB) at the 45 s pod dwell ~2.9 m. Literature: ~0.5-3 m typical.
    beneath = R.reach_path_m(R.REACH_REF_RANGE_M, 0.95)
    assert 1.7 <= beneath <= 2.2
    old_gain = R._PEN["rx_gain_db"]
    try:
        R.use_rx_gain_db(7.63)
        assert 2.7 <= R.reach_path_m(R.REACH_REF_RANGE_M, 0.95, dwell=45.0) <= 3.1
    finally:
        R.use_rx_gain_db(old_gain)
    assert R.reach_path_m(11.0, 0.95) < beneath - 1.0       # spreading eats reach at a stand-off
    assert R.reach_path_m(2.0, 0.3) <= beneath


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_breather_beyond_reach_gives_no_detection(seed, literature):
    deep = make(seed, range_m=6.0, path=6.0)
    assert not any(abs(r - 6.0) < 0.3 for r in deep.range_candidates)
    near = make(seed, range_m=6.0, path=0.3)
    assert any(abs(r - 6.0) < 0.3 for r in near.range_candidates)
    obs = near.observations("s1", "d1", [0, 0, 0.4, 0], 20.0)[0]
    assert 0.0 <= obs["max_reliable_depth_m"] <= 5.0


def test_legacy_preset_is_the_optimistic_flat_loss():
    old = R.penetration_model()
    try:
        R.use_penetration("legacy")
        assert R.two_way_loss_db(10.0) == pytest.approx(14.0)          # 10 m costs only 14 dB: not credible
        assert R.reach_path_m(2.0, 0.95) > 10
    finally:
        R.use_penetration(old)
    with pytest.raises(ValueError):
        R.use_penetration("nope")
