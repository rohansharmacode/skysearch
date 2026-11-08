"""Fast tests for sim/physio.py (synthetic chest motion) and sim/vitals.py (pattern features + triage aid)."""
import json

import numpy as np
import pytest

from sim import physio, radar_dsp as R, vitals

FS = 20.0
CONTRACT = {"rate_bpm", "rate_ci", "cv", "depth_rel", "depth_rel_conf", "apnea_events_per_min", "movement",
            "pattern", "pattern_conf", "condition", "condition_text", "triage_hint", "caveats"}


def _scan(state, seed, dur=60.0, noise_m=0.1e-3):
    rng = np.random.default_rng([seed, 42])
    ph = physio.sample_physio(state, rng)
    t = np.arange(0, dur, 1 / FS)
    x, ev = physio.chest_displacement(t, ph, rng, return_events=True)
    return x + rng.normal(0, noise_m, t.size), ph, ev


def test_physio_waveform_shape():
    t = np.arange(0, 60, 1 / FS)
    ph = {"state": "calm", "rate_bpm": 12.0, "depth_rel": 1.0, "cv": 0.0, "apnea_per_min": 0.0}
    x, ev = physio.chest_displacement(t, ph, np.random.default_rng(0), cardiac=False, movements=False, return_events=True)
    assert 10 <= len(ev["breaths"]) <= 14
    assert 3e-3 < np.ptp(x) < 1.2e-2                      # ~5 mm peak to peak (sighs allowed)
    # asymmetric: rising (inhale) faster than falling (exhale)
    d = np.diff(x)
    assert (d > 0).mean() < 0.5


@pytest.mark.parametrize("state", list(physio.PATTERN_FOR_STATE))
def test_each_state_classified_at_good_snr(state):
    want = physio.PATTERN_FOR_STATE[state]
    hits = 0
    for seed in range(6):
        x, ph, _ = _scan(state, seed)
        v = vitals.extract(x, FS, kind="displacement")
        hits += v["pattern"] == want
    assert hits >= 5, f"{state}: {hits}/6 classified as {want}"


def test_rate_within_1p5_bpm_at_good_snr():
    for state in ("calm", "distressed_tachypnea", "unconscious_slow"):
        for seed in range(4):
            x, ph, ev = _scan(state, seed)
            true = 60.0 / np.median(np.diff(ev["breaths"]))     # realised rate in this window
            v = vitals.extract(x, FS, kind="displacement")
            assert abs(v["rate_bpm"] - true) <= 1.5, (state, seed, v["rate_bpm"], true)
            assert v["rate_ci"][0] <= v["rate_bpm"] <= v["rate_ci"][1]


def test_apnea_detected_and_not_hallucinated():
    t = np.arange(0, 60, 1 / FS)
    calm = {"state": "calm", "rate_bpm": 14.0, "depth_rel": 1.0, "cv": 0.05, "apnea_per_min": 0.0}
    x = physio.chest_displacement(t, calm, np.random.default_rng(3), movements=False)
    x[int(22 * FS):int(38 * FS)] = x[int(22 * FS)]          # a 16 s pause
    x = x + np.random.default_rng(4).normal(0, 0.05e-3, t.size)
    v = vitals.extract(x, FS, kind="displacement", keep_arrays=True)
    assert v["apnea_events"] == 1
    (a, b), = v["_apneas_s"]
    assert 20 <= a <= 27 and 33 <= b <= 40
    assert v["pattern"] == "irregular_with_pauses" and v["triage_hint"] == "immediate (check)"
    for seed in range(4):                                    # regular breathers: no pauses invented
        x, _, _ = _scan("calm", seed)
        assert vitals.extract(x, FS, kind="displacement")["apnea_events"] == 0


def test_unclear_at_low_quality():
    rng = np.random.default_rng(9)
    v = vitals.extract(rng.normal(0, 1e-3, int(60 * FS)), FS, kind="displacement")
    assert v["pattern"] == "unclear" and v["condition"] == "unknown" and v["pattern_conf"] < 0.95
    assert v["triage_hint"] in ("assess", "reject") and v["source"] != "human_breathing"
    x, _, _ = _scan("calm", 1)
    v = vitals.extract(x + rng.normal(0, 4e-3, x.size), FS, kind="displacement")   # breathing buried in noise
    assert v["pattern"] == "unclear" or v["pattern_conf"] < 0.4


def test_condition_only_no_age():
    assert set(vitals.PATTERNS) == {"regular_calm", "rapid_shallow", "slow_regular", "irregular_with_pauses", "unclear"}
    assert set(vitals.CONDITIONS) == set(physio.PATTERN_FOR_STATE) | {"unknown"}
    for state in physio.PATTERN_FOR_STATE:
        v = vitals.extract(_scan(state, 0)[0], FS, kind="displacement")
        text = json.dumps(v).lower()
        assert "child" not in text and "age" not in set(v)
        assert v["condition"] == state and "->" in v["condition_text"]


def test_slow_regular_is_not_immediate():
    """A slow (6-8/min), regular, deep, pause-free breather is 'urgent: check responsiveness', not 'immediate'."""
    t = np.arange(0, 60, 1 / FS)
    for k, rate in enumerate((6.5, 8.0, 9.5)):
        ph = {"state": "unconscious_slow", "rate_bpm": rate, "depth_rel": 1.3, "cv": 0.06, "apnea_per_min": 0.0}
        x = physio.chest_displacement(t, ph, np.random.default_rng(k)) + np.random.default_rng(k + 9).normal(0, 0.1e-3, t.size)
        v = vitals.extract(x, FS, kind="displacement")
        assert v["pattern"] == "slow_regular" and v["triage_hint"] == "urgent", (rate, v["pattern"], v["pattern_reason"])
        assert "cannot tell" in v["condition_text"] and v["condition"] == "unconscious_slow"


def test_contract_fields_json_and_priority():
    x, _, _ = _scan("distressed_tachypnea", 0)
    v = vitals.extract(x, FS, kind="displacement")
    assert CONTRACT <= set(v)
    json.dumps(v)
    assert v["triage_hint"] in vitals.TRIAGE_RANK and v["depth_rel_conf"] <= 0.3
    calm = vitals.extract(_scan("calm", 0)[0], FS, kind="displacement")
    assert vitals.priority_weight(v) > vitals.priority_weight(calm)
    assert vitals.priority_weight({"triage_hint": "immediate", "pattern_conf": 0.0}) == vitals.TRIAGE_RANK["assess"]


def test_end_to_end_through_dsp_and_extract_from_obs():
    """Synthetic IF cube with a physio chest -> process_if candidate bin -> extract_from_obs."""
    rng = np.random.default_rng(5)
    P = R.RadarParams(dwell=60.0, resp_band=(0.1, 0.75), oob_band=(0.85, 3.0))
    ph = {"state": "calm", "rate_bpm": 14.0, "depth_rel": 1.0, "cv": 0.05, "apnea_per_min": 0.0}
    sc = R.Scene(quality=0.95)
    sc.clutter = R.default_clutter(rng, max_range=20.0)
    sc.breathers.append(R.Breather(2.0, R.reflection_amplitude(2.0, 0.5, R.CHEST_RCS_DB, P), 14 / 60))
    t = P.t_slow
    disp = physio.chest_displacement(t, ph, rng, movements=False)
    orig = R._breath_displacement
    R._breath_displacement = lambda b, p, g: disp            # stand-in for the requested generator hook
    try:
        iq = R.simulate_if(sc, P, rng)
    finally:
        R._breath_displacement = orig
    res = R.process_if(iq, P, keep_debug=True)
    c = min(res.candidates, key=lambda c: abs(c["range_m"] - 2.0))
    X = R.range_fft(iq)
    d_ref, _ = R._estimate_platform_motion(X, P, R.DspConfig())
    Xc = X * np.exp(-1j * 4 * np.pi * d_ref / P.wavelength)[:, None]
    v = vitals.extract_from_obs({"slow_time_iq": Xc[:, c["bin"]], "fs": P.fs_slow, "wavelength": P.wavelength,
                                 "range_m": c["range_m"], "rubble_path_m": 0.5, "quality": res.quality,
                                 "snr_db": c["snr_db"], "n_fast": P.n_fast})
    assert v["pattern"] == "regular_calm" and abs(v["rate_bpm"] - 14.0) <= 1.5
    assert v["depth_rel"] is not None and 0.3 < v["depth_rel"] < 3.0
    v2 = vitals.extract_from_obs(res.debug)                   # fallback: the existing debug dict
    assert v2["depth_rel"] is None and abs(v2["rate_bpm"] - v["rate_bpm"]) <= 2.0


# ----------------------------------------------------------------------------- confounder rejector
def _t(dur=60.0):
    return np.arange(0, dur, 1 / FS)


def test_rejector_machinery_tone():
    t = _t()
    x = 0.8e-3 * np.sin(2 * np.pi * 2.3 * t) + 0.25e-3 * np.sin(2 * np.pi * 4.6 * t)
    v = vitals.extract(x + np.random.default_rng(1).normal(0, 0.02e-3, t.size), FS, kind="displacement")
    assert v["source"] == "machinery" and v["condition"] == "unknown" and v["triage_hint"] == "reject"


def test_rejector_debris_step():
    from scipy.special import expit
    t = _t()
    x = 8e-3 * expit((t - 31.0) / 0.3) + np.random.default_rng(2).normal(0, 0.05e-3, t.size)
    v = vitals.extract(x, FS, kind="displacement")
    assert v["source"] == "debris_shift", (v["source"], v["source_reason"], v["signature"])


def test_rejector_knocking_person():
    t = _t()
    rng = np.random.default_rng(3)
    x = physio.chest_displacement(t, physio.sample_physio("calm", rng), rng, movements=False)
    for tb in (5.0, 17.0, 29.0, 41.0, 52.0):
        for k in range(3):
            x = x + 10e-3 * np.exp(-0.5 * ((t - tb - k / 3.0) / 0.05) ** 2)
    v = vitals.extract(x, FS, kind="displacement")
    assert v["source"] == "moving_person" and v["triage_hint"] == "urgent"


def test_rejector_small_animal_and_human_kept():
    t = _t()
    rng = np.random.default_rng(4)
    dog = {"state": "animal", "rate_bpm": 30.0, "depth_rel": 0.12, "cv": 0.08, "apnea_per_min": 0.0}
    x = physio.chest_displacement(t, dog, rng, cardiac=False, movements=False) + rng.normal(0, 0.01e-3, t.size)
    assert vitals.extract(x, FS, kind="displacement")["source"] == "small_animal"
    for state in physio.PATTERN_FOR_STATE:                    # real breathing patterns are not rejected
        v = vitals.extract(_scan(state, 0)[0], FS, kind="displacement")
        assert v["source"] == "human_breathing", (state, v["source"], v["source_reason"])


def test_identify_through_dsp_signature_library():
    """One synthetic pod scan per class through the real chain: humans keep a condition, confounders are not
    called a breathing person (rotor / machinery / empty may come back as no_signal: that is still a reject)."""
    from sim import radar_signatures as RS
    for cls, seed in (("calm", 1002), ("empty", 1000), ("machinery", 1000), ("moving_person", 1000)):
        sc = RS.render(cls, seed, 45.0)
        o = vitals.identify(sc["iq"], sc["params"])
        if cls in RS.HUMAN:
            assert o["source"] == "human_breathing" and o["condition"] == cls, (cls, o["source"], o["reason"])
        elif cls == "moving_person":
            assert o["source"] in ("moving_person", "human_breathing"), (cls, o["source"])
        else:
            assert o["source"] != "human_breathing", (cls, o["source"], o["reason"])
