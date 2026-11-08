"""SYNTHETIC radar signature library: breathing conditions + non-human confounders, seen from a sensor pod.

*** TRUTH SIDE. THIS MODULE FABRICATES RADAR SCENES (like sim/radar_model.py and sim/physio.py). ***
Each class renders a reflector motion into sim/radar_dsp.simulate_if (the synthetic FMCW IF generator);
everything downstream (range FFT, motion compensation, clutter removal, phase demodulation, CFAR,
sim/vitals.py) is the real processing chain and never sees the class.

Geometry: the winch pod rests on the rubble surface and looks down. The reflector sits 1.2-3.0 m from
the antenna, and the whole path is rubble (rubble_path = range). The DSP ignores < 1 m (antenna
leakage, ground bounce). Pod scans are steady: quality 0.75-0.97.

    from sim import radar_signatures as RS
    sc = RS.render("irregular_apneic", seed=3, dwell=60.0)   # dict: iq, params, truth (evaluator only)

Classes (docs/RADAR_SIGNATURES.md has the physics and the literature anchors):
  human breathing   calm | distressed_tachypnea | unconscious_slow | irregular_apneic (sim/physio.py)
  confounders       empty | debris | small_animal | machinery | rotor | moving_person
"""
from __future__ import annotations

from contextlib import contextmanager

import numpy as np
from scipy.special import expit

from . import physio
from . import radar_dsp as R

HUMAN = ("calm", "distressed_tachypnea", "unconscious_slow", "irregular_apneic")
CONFOUNDERS = ("empty", "debris", "small_animal", "machinery", "rotor", "moving_person")
CLASSES = HUMAN + CONFOUNDERS

# plain-words label + the expected sim/vitals.py verdict (source, condition) for each class
LABEL = {
    "calm": "calm, regular breathing (stable for now)",
    "distressed_tachypnea": "rapid shallow breathing (possible injury or distress)",
    "unconscious_slow": "slow, regular breathing (deep rest or reduced consciousness)",
    "irregular_apneic": "irregular breathing with pauses (check first)",
    "empty": "empty rubble: noise only",
    "debris": "settling debris: step / creep, no rhythm",
    "small_animal": "small animal breathing (fast, tiny)",
    "machinery": "generator / machinery vibration (pure tones)",
    "rotor": "drone rotor reflection (aliased blade flashes)",
    "moving_person": "person moving or knocking (bursts)",
}
EXPECTED = {**{c: ("human_breathing", c) for c in HUMAN},
            "empty": ("no_signal", "unknown"), "debris": ("debris_shift", "unknown"),
            "small_animal": ("small_animal", "unknown"), "machinery": ("machinery", "unknown"),
            "rotor": ("rotor", "unknown"), "moving_person": ("moving_person", "unknown")}


# ----------------------------------------------------------------------------- motion models (metres)
def _debris(t, rng):
    """1-3 settling events (sigmoid steps of 2-15 mm over 0.3-3 s) plus slow creep (0-4 mm/min)."""
    x = np.zeros_like(t)
    for _ in range(int(rng.integers(1, 4))):
        t0, L, a = rng.uniform(0.1, 0.9) * t[-1], rng.uniform(0.3, 3.0), rng.choice([-1, 1]) * rng.uniform(2e-3, 15e-3)
        x += a * expit((t - t0) / (L / 6))
    return x + rng.uniform(-4e-3, 4e-3) / 60.0 * t


def _machinery(t, rng):
    """Structure-borne vibration from a generator / pump: a pure tone and 1-3 harmonics. Engine shafts turn
    at 25-30 Hz (1500 / 1800 rpm); at the 20 Hz frame rate those alias, so the slow-time fundamental is
    drawn in 0.8-6 Hz. Amplitude 0.1-1.5 mm, frequency stable to < 0.5%."""
    f0, a0 = rng.uniform(0.8, 6.0), rng.uniform(0.1e-3, 1.5e-3)
    x = a0 * np.sin(2 * np.pi * f0 * (1 + 0.003 * rng.normal()) * t + rng.uniform(0, 2 * np.pi))
    for k in range(2, int(rng.integers(2, 5))):
        fk = (k * f0 + 10.0) % 20.0 - 10.0                   # alias into +-fs/2
        x += a0 * rng.uniform(0.15, 0.5) / k * np.sin(2 * np.pi * abs(fk) * t + rng.uniform(0, 2 * np.pi))
    return x


def _rotor(t, rng, wavelength):
    """A hovering drone seen through the backlobe / multipath. Two parts:
    body: hover sway (2-10 mm, 0.2-1.5 Hz);
    blades: tips move tens of cm between 20 Hz frames (blade-pass ~50-200 Hz), so their return has a new,
    effectively random phase every frame: aliased micro-Doppler that fills the whole slow-time band."""
    sway = sum(rng.uniform(1e-3, 5e-3) * np.sin(2 * np.pi * rng.uniform(0.2, 1.5) * t + rng.uniform(0, 2 * np.pi))
               for _ in range(2))
    blades = sway + wavelength / (4 * np.pi) * rng.uniform(0, 2 * np.pi, t.size)
    return sway, blades


def _moving(t, rng):
    """A trapped person knocking or shifting: bursts of 3-5 knocks (2-4 Hz, 4-20 mm hand / shoulder motion)
    every 4-12 s, occasional body shifts, and calm breathing underneath."""
    x = physio.chest_displacement(t, physio.sample_physio("calm", rng), rng, movements=False)
    tb = rng.uniform(0, 4)
    while tb < t[-1]:
        n, f, a = int(rng.integers(3, 6)), rng.uniform(2.0, 4.0), rng.uniform(4e-3, 20e-3)
        for k in range(n):
            tk = tb + k / f
            x += a * np.exp(-0.5 * ((t - tk) / 0.05) ** 2)
        if rng.uniform() < 0.4:                               # shift the body after knocking
            x += rng.choice([-1, 1]) * rng.uniform(3e-3, 15e-3) * expit((t - tb - n / f - 0.5) / 0.3)
        tb += n / f + rng.uniform(4.0, 12.0)
    return x


def _animal(t, rng):
    """Small animal (dog / cat) at rest: 20-40 breaths/min, 0.5-1.5 mm chest excursion; 30% of instances
    also pant (3-6 Hz, 0.2-0.6 mm) for part of the window."""
    ph = {"state": "animal", "rate_bpm": rng.uniform(20, 40), "depth_rel": rng.uniform(0.1, 0.3),
          "cv": rng.uniform(0.05, 0.15), "apnea_per_min": 0.0}
    x = physio.chest_displacement(t, ph, rng, cardiac=False, movements=False)
    if rng.uniform() < 0.3:
        t0, t1 = sorted(rng.uniform(0, t[-1], 2))
        x += np.where((t > t0) & (t < t1), rng.uniform(0.2e-3, 0.6e-3), 0.0) * np.sin(2 * np.pi * rng.uniform(3, 6) * t)
    return x, ph


@contextmanager
def _custom_motion():
    """Stand-in hook: a Breather may carry ``disp`` (metres per frame); simulate_if then renders that motion.
    (Patched here, not in radar_dsp, so the shared generator stays as the mission builder left it.)"""
    orig = R._breath_displacement
    R._breath_displacement = lambda b, params, rng: getattr(b, "disp", None) if getattr(b, "disp", None) is not None \
        else orig(b, params, rng)
    try:
        yield
    finally:
        R._breath_displacement = orig


def render(cls: str, seed: int = 0, dwell: float = 60.0, quality: float | None = None, range_m: float | None = None):
    """Render one synthetic pod scan of ``cls``. Returns {"iq", "params", "truth"} (truth: evaluator / display only)."""
    rng = np.random.default_rng([seed, CLASSES.index(cls), 77])
    P = R.RadarParams(dwell=float(dwell), resp_band=(0.1, 0.75), oob_band=(0.85, 3.0))
    t = P.t_slow
    q = float(rng.uniform(0.75, 0.97)) if quality is None else float(quality)
    r = float(rng.uniform(1.2, 3.0)) if range_m is None else float(range_m)
    scene = R.Scene(quality=q)
    scene.clutter = R.default_clutter(rng, max_range=min(22.0, P.max_range - 2.0))
    truth = {"class": cls, "label": LABEL[cls], "range_m": r, "rubble_path_m": r, "quality": q, "dwell_s": float(dwell)}
    disp, rcs = None, R.CHEST_RCS_DB + float(rng.normal(0, 3.0))
    if cls in HUMAN:
        ph = physio.sample_physio(cls, rng)
        disp, ev = physio.chest_displacement(t, ph, rng, return_events=True)
        truth.update(physio=ph, pauses=[(a, b) for a, b in ev["apneas"] if b - a >= 10])
    elif cls == "small_animal":
        disp, ph = _animal(t, rng)
        rcs = float(rng.uniform(-14.0, -8.0))               # a dog's chest: ~10 dB below an adult's
        truth.update(physio=ph)
    elif cls == "debris":
        disp, rcs = _debris(t, rng), float(rng.uniform(5.0, 15.0))
    elif cls == "machinery":
        disp, rcs = _machinery(t, rng), float(rng.uniform(5.0, 15.0))
    elif cls == "rotor":
        r = float(rng.uniform(4.0, 9.0))                     # the drone above the pod, via backlobe / multipath
        disp, blades = _rotor(t, rng, P.wavelength)
        rcs = float(rng.uniform(-10.0, 0.0))                 # 0-10 dB body RCS minus ~20 dB backlobe
        truth.update(range_m=r, rubble_path_m=0.0)
        bb = R.Breather(r, R.reflection_amplitude(r, 0.0, rcs + float(rng.uniform(-3.0, 3.0)), P), 0.25)
        bb.disp = blades                                     # blade flashes: specular, as strong as the body (+-3 dB)
        scene.breathers.append(bb)
    elif cls == "moving_person":
        disp, rcs = _moving(t, rng), R.CHEST_RCS_DB + float(rng.uniform(0.0, 6.0))
    if disp is not None:
        b = R.Breather(r, R.reflection_amplitude(r, truth["rubble_path_m"], rcs, P), 0.25)
        b.disp = disp
        scene.breathers.append(b)
        truth["displacement_m"] = disp
    truth["rcs_db"] = rcs
    with _custom_motion():
        iq = R.simulate_if(scene, P, rng)
    return {"iq": iq, "params": P, "truth": truth}


def front_end(iq, params, cfg=None):
    """The motion-compensated range-slow-time cube Xc exactly as radar_dsp.process_if builds it."""
    cfg = cfg or R.DspConfig()
    X = R.range_fft(iq)
    d_ref, _ = R._estimate_platform_motion(X, params, cfg)
    Xc = X * np.exp(-1j * 4 * np.pi * d_ref / params.wavelength)[:, None]
    A = np.column_stack([d_ref, np.linspace(-1, 1, X.shape[0])])
    coef, *_ = np.linalg.lstsq(A, Xc - Xc.mean(axis=0, keepdims=True), rcond=None)
    return Xc - A @ coef
