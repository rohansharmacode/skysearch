"""SYNTHETIC chest-wall displacement for the radar generator (truth side only).

*** THIS MODULE TURNS HIDDEN TRUTH (a victim's ``physio`` dict) INTO A WAVEFORM. ***
Only observation generators (sim/radar_dsp.simulate_if via sim/radar_model.py) and evaluators may
call it. The planner side never sees ``physio``; it sees what sim/vitals.py measures from the radar.

``physio`` (from sim/scenario.py, PHYSIO_STATES): {state, rate_bpm, depth_rel, cv, apnea_per_min}
  rate_bpm       mean breaths per minute while breathing
  depth_rel      chest excursion relative to a calm adult; 1.0 = DEPTH_REF_M peak-to-peak
  cv             breath-to-breath interval variability (std / mean)
  apnea_per_min  pauses > 10 s per minute (Poisson)

Model (per breath, not a sinusoid):
  * inhale: raised-cosine rise over Ti; exhale: passive exponential-like fall over Te; an
    end-expiratory pause for slow breathing. Ti / Ttot depends on the state (0.33-0.45; adults at rest
    have I:E about 1:1.5-1:2, fast breathing pushes it towards 1:1).
  * interval and amplitude jitter per breath (log-normal, from ``cv``); occasional sighs (2x breath).
  * apnea pauses: Poisson events, 10-20 s of flat end-expiratory chest, with smaller breaths just
    before and after (a crescendo-decrescendo hint, as in periodic breathing).
  * optional small body movements (0.5-2 s, 1-6 mm steps) for conscious states.
  * optional cardiac component, ~0.1 mm at 60-110 bpm. At the 0.4-2 GHz rubble radar's wavelength
    (~25 cm) that is ~0.005 rad of phase: NOT reliably resolvable through rubble; it is included for
    realism only and sim/vitals.py does not try to measure it.

Numbers are textbook-plausible, not fitted to patients: adult tidal chest-wall excursion of a few mm
(radar literature typically 4-12 mm anterior chest), START/SALT respiratory cut-offs at 10 / 30 bpm.
"""
from __future__ import annotations

import numpy as np

DEPTH_REF_M = 5e-3                 # depth_rel 1.0 = 5 mm peak-to-peak chest excursion (calm adult)
CARDIAC_M = 1e-4                   # ~0.1 mm heartbeat displacement (not resolvable through rubble)

# state -> waveform shape: inhale fraction of the cycle, end-expiratory pause fraction,
# sigh probability per breath, body movements per minute, heart-rate range (bpm)
SHAPE = {
    "calm":                 dict(ti=0.38, pause=0.10, sigh=0.03, moves=0.3, hr=(60, 85)),
    "distressed_tachypnea": dict(ti=0.45, pause=0.00, sigh=0.00, moves=0.6, hr=(95, 110)),
    "unconscious_slow":     dict(ti=0.33, pause=0.25, sigh=0.00, moves=0.0, hr=(55, 70)),
    "irregular_apneic":     dict(ti=0.38, pause=0.10, sigh=0.05, moves=0.1, hr=(60, 100)),
    "child_calm":           dict(ti=0.40, pause=0.05, sigh=0.03, moves=0.4, hr=(90, 110)),
}
DEFAULT_SHAPE = SHAPE["calm"]

# evaluator mapping: hidden state -> the pattern sim/vitals.py should report (truth side only)
PATTERN_FOR_STATE = {
    "calm": "regular_calm",
    "distressed_tachypnea": "rapid_shallow",
    "unconscious_slow": "slow_deep",
    "irregular_apneic": "irregular_with_pauses",
    "child_calm": "fast_small",
}


def physio_from_breathing_hz(hz: float, state: str = "calm") -> dict:
    """Legacy targets (only ``breathing_hz``): a calm, regular breather at that rate."""
    return {"state": state, "rate_bpm": 60.0 * float(hz), "depth_rel": 1.0, "cv": 0.05, "apnea_per_min": 0.0}


def sample_physio(state: str, rng) -> dict:
    """Draw a physio dict for ``state`` from sim/scenario.PHYSIO_STATES ranges (evaluators / tests)."""
    from .scenario import PHYSIO_STATES
    return {"state": state, **{k: float(rng.uniform(*r)) for k, r in PHYSIO_STATES[state].items()}}


def _breath(n_i: int, n_e: int, n_p: int) -> np.ndarray:
    """One breath, 0 -> 1 -> 0 (+ flat pause): raised-cosine inhale, passive exponential exhale."""
    up = 0.5 - 0.5 * np.cos(np.pi * np.arange(n_i) / max(n_i, 1))
    s = np.arange(n_e) / max(n_e, 1)
    k = 3.5                                                   # exhale time constant ~ Te / 3.5
    down = (np.exp(-k * s) - np.exp(-k)) / (1 - np.exp(-k))
    down = 0.5 * down + 0.5 * (0.5 + 0.5 * np.cos(np.pi * s))  # blend: smooth start, passive tail
    return np.concatenate([up, down, np.zeros(n_p)])


def chest_displacement(t, physio: dict, rng, *, cardiac: bool = True, movements: bool = True,
                       return_events: bool = False):
    """Chest-wall displacement (metres, towards the radar positive) at times ``t`` (s, uniform).

    Deterministic given ``rng``. With ``return_events`` also returns
    {"breaths": [t_peak...], "apneas": [(t0, t1)...], "moves": [(t0, t1)...]} (truth, evaluators only).
    """
    t = np.asarray(t, float)
    fs = 1.0 / float(np.median(np.diff(t))) if t.size > 1 else 20.0
    dur = float(t[-1] - t[0]) + 1.0 / fs if t.size else 0.0
    shp = SHAPE.get(physio.get("state", "calm"), DEFAULT_SHAPE)
    rate = float(np.clip(physio.get("rate_bpm", 14.0), 2.0, 80.0))
    depth = DEPTH_REF_M * float(max(physio.get("depth_rel", 1.0), 0.0))
    cv = float(np.clip(physio.get("cv", 0.05), 0.0, 0.8))
    apnea_rate = float(max(physio.get("apnea_per_min", 0.0), 0.0))
    T0 = 60.0 / rate

    # apnea onsets: Poisson in time (start a little before t0 so a pause can already be running)
    pre = 1.5 * T0 + rng.uniform(0, 15.0)
    span = dur + pre
    n_ap = rng.poisson(apnea_rate * span / 60.0) if apnea_rate > 0 else 0
    ap_starts = np.sort(rng.uniform(0, span, n_ap))
    ap_len = rng.uniform(10.5, 20.0, n_ap)

    n_tot = int(np.ceil(span * fs)) + 1
    x = np.zeros(n_tot)
    events = {"breaths": [], "apneas": [], "moves": []}
    pos = 0.0                                               # seconds since the (virtual) start
    ai = 0
    sigma_log = np.sqrt(np.log1p(cv ** 2))
    near_ap = lambda p: any(abs(p - s) < 2.5 * T0 or abs(p - (s + l)) < 2.5 * T0
                            for s, l in zip(ap_starts, ap_len))
    while pos < span:
        if ai < n_ap and pos >= ap_starts[ai]:
            events["apneas"].append((pos - pre, pos + ap_len[ai] - pre))
            pos += ap_len[ai]                                # flat chest (end-expiratory)
            ai += 1
            continue
        T = T0 * float(np.exp(rng.normal(-0.5 * sigma_log ** 2, sigma_log)))
        T = float(np.clip(T, 0.35 * T0, 2.5 * T0))
        a = depth * float(np.exp(rng.normal(0, 0.6 * sigma_log + 0.05)))
        if near_ap(pos):
            a *= rng.uniform(0.35, 0.7)                       # shallow breaths around a pause
        if rng.uniform() < shp["sigh"]:
            a *= 2.0
            T *= 1.4
        n = max(int(round(T * fs)), 4)
        n_p = int(round(shp["pause"] * n * rng.uniform(0.6, 1.4)))
        n_act = n - n_p
        n_i = max(int(round(shp["ti"] / max(1 - shp["pause"], 1e-6) * n_act * rng.uniform(0.9, 1.1))), 2)
        n_e = max(n_act - n_i, 2)
        b = a * _breath(n_i, n_e, max(n - n_i - n_e, 0))
        i0 = int(round(pos * fs))
        seg = slice(i0, min(i0 + b.size, n_tot))
        x[seg] += b[:seg.stop - seg.start]
        events["breaths"].append(pos + n_i / fs - pre)
        pos += b.size / fs

    i_pre = int(round(pre * fs))
    x = x[i_pre:i_pre + t.size]
    if x.size < t.size:
        x = np.pad(x, (0, t.size - x.size))
    x = x - x.mean()

    tt = t - t[0]
    if movements and shp["moves"] > 0:
        for _ in range(rng.poisson(shp["moves"] * dur / 60.0)):
            t0, L = rng.uniform(0, dur), rng.uniform(0.5, 2.0)
            step = rng.choice([-1, 1]) * rng.uniform(1e-3, 6e-3)
            ramp = np.clip((tt - t0) / L, 0, 1)
            back = rng.uniform() < 0.5                         # shift and return, or a lasting step
            x += step * (np.sin(np.pi * ramp) if back else 0.5 - 0.5 * np.cos(np.pi * ramp))
            events["moves"].append((t0, t0 + L))
    if cardiac:
        hr = rng.uniform(*shp["hr"]) / 60.0
        hr_t = hr * (1 + 0.03 * np.sin(2 * np.pi * rate / 60.0 * tt))   # respiratory sinus arrhythmia
        x += CARDIAC_M * np.sin(2 * np.pi * np.cumsum(hr_t) / fs + rng.uniform(0, 2 * np.pi))
    events["breaths"] = [b for b in events["breaths"] if 0 <= b <= dur]
    events["apneas"] = [(max(a, 0.0), min(b, dur)) for a, b in events["apneas"] if b > 0 and a < dur]
    return (x, events) if return_events else x
