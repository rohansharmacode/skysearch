"""Breathing-pattern features and a transparent triage AID from the radar slow-time signal.

Planner side: this module never sees ground truth. Its input is what the DSP chain recovers at a
candidate range bin (sim/radar_dsp.process_if): the complex slow-time samples of that bin, or a
phase / displacement series derived from them. On real radar (scripts/validate_vitals.py) the same
code runs on 24 GHz CW I/Q recordings.

    from sim import vitals
    v = vitals.extract(iq_bin, fs=20.0, wavelength=0.25, range_m=2.3, rubble_path_m=1.4, quality=0.8)
    v = vitals.extract_from_obs(obs_debug)            # dict form, see extract_from_obs

Pipeline
  1. demodulation (complex input): circle-fit centre + arctangent when the I/Q arc is wide enough to
     fit (>= MIN_ARC_RAD), else principal-axis linear demodulation scaled by an amplitude estimate
     (path-loss model from range + rubble path, sim/radar_dsp.reflection_amplitude); -> metres.
  2. movement / artefact frames: 1 s frames whose 0.9-3.5 Hz energy is far above the scan's median.
  3. rate: spectral peak (0.1-0.75 Hz = 6-45 bpm) AND time-domain breath peaks; time domain wins when
     there are >= 3 clean intervals (it is robust to waveform asymmetry and pauses).
  4. breath-interval CV, shorter/longer half-cycle ratio (I:E proxy), per-breath depth.
  5. apnea: moving-RMS envelope below APNEA_FRAC of the breathing level for > APNEA_MIN_S seconds.
  6. signal quality index (SQI) from band SNR, movement fraction, number of breaths, scan quality.
  7. rule-based pattern classifier (thresholds below) -> pattern, pattern_conf, triage_hint.

The triage hint follows START / SALT RESPIRATORY criteria only (rate > 30 -> immediate, irregular or
pauses -> immediate (check), 10-20 regular -> delayed). It is an aid for rescuers ordering the dig,
never a diagnosis: START also needs perfusion and mental status, which a radar cannot assess.
"""
from __future__ import annotations

import numpy as np
from scipy import signal

VERSION = "vitals-v1"

# ----------------------------------------------------------------------------- thresholds (documented in docs/VITALS.md)
DEPTH_REF_M = 5e-3          # depth_rel 1.0 = 5 mm peak-to-peak chest excursion (same as sim/physio.py)
RATE_BAND_HZ = (0.1, 0.75)  # 6-45 breaths/min searched for the rate
WAVE_BAND_HZ = (0.05, 1.2)  # band kept for the breathing waveform (keeps the 2nd harmonic of fast breathing)
MOVE_BAND_HZ = (0.9, 3.5)   # energy here (beyond breathing + its harmonics) marks movement / artefact
MOVE_K = 5.0                # frame is "movement" if its MOVE_BAND rms > MOVE_K x the scan's median frame rms
APNEA_MIN_S = 10.0          # clinical: a pause > 10 s
APNEA_FRAC = 0.35          # envelope below 35% of the breathing level counts as "no breathing motion"
APNEA_WIN_S = 3.0           # moving-RMS window for the envelope
APNEA_BAND_HZ = (0.1, 0.8)  # envelope band: breathing fundamentals only (drops post-hold drift and heartbeat)
MIN_ARC_RAD = 0.6           # I/Q arc needed for a trustworthy circle fit (arctangent demodulation)
MAX_RING_RESID = 0.6        # std(|y| - r) / r: a centred noise blob gives ~0.5-0.6, real 24 GHz records 0.15-0.5
MIN_BEND = 0.25             # sqrt(minor / major I/Q variance): below this the arc is too short to fit a circle
MAX_PHASE_STEP = 0.15       # median |phase step| per sample (rad): real records ~0.05, noise blobs >= 0.25
SQI_MIN = 0.35              # below this the pattern is "unclear"
SNR_MID_DB = 8.0            # band SNR at which the SNR term of the SQI is 0.5

RATE_FAST = 30.0            # START: respiratory rate > 30/min -> IMMEDIATE
RATE_ELEVATED = 19.5        # adult resting normal is about 12-20/min
RATE_SLOW = 11.0            # below this: bradypnoea (NEWS scores <= 8 as critical; we use 11 as the split)
CV_IRREGULAR = 0.20         # robust breath-interval CV above this = irregular (at scan quality 1) ...
CV_Q_SLOPE = 0.25           # ... raised by this x (1 - quality): platform motion jitters breath timing
RATE_RAPID = 27.0           # 27-30/min: "rapid_shallow" pattern (hint "immediate (check)"); > 30 is START "immediate"
CV_VERY_REGULAR = 0.08      # 27-30/min but this regular (robust CV) -> still "fast_small" (children breathe up to ~30)
CV_FAST_IRREGULAR = 0.30    # 19.5-27/min with robust CV above this -> rapid_shallow (radar timing jitter inflates
                            # CV of fast breathing, so this is set high; measured child CV reads 0.1-0.3 at low quality)
CHILD_DEPTH_MAX = 1.0       # depth_rel above this argues against "small chest" (only when depth is measured; synthetic depth reads ~1.5x high)
CHILD_CONF_CAP = 0.45       # age inference from breathing is weak: never more confident than this

PATTERNS = ("regular_calm", "rapid_shallow", "slow_deep", "irregular_with_pauses", "fast_small", "unclear")
# rescuer-facing hint and its dig-order rank (higher = dig first); "assess" sits in the middle on purpose
TRIAGE_RANK = {"immediate": 1.0, "immediate (check)": 0.85, "assess": 0.55, "delayed (re-check)": 0.45, "delayed": 0.35}

CAVEATS_ALWAYS = [
    "aid for rescuers, not a diagnosis: radar sees chest-wall motion only (no perfusion, mental status or airway)",
    "pattern is inferred from one scan window; re-scan to confirm before acting on it",
]


# ----------------------------------------------------------------------------- helpers
def _circle_fit(x):
    a = np.column_stack([2 * x.real, 2 * x.imag, np.ones(x.size)])
    b = np.abs(x) ** 2
    sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    c = complex(sol[0], sol[1])
    r = float(np.sqrt(max(sol[2] + abs(c) ** 2, 0.0)))
    return c, r


def _bp(x, fs, band, order=3):
    hi = min(band[1], 0.45 * fs)
    sos = signal.butter(order, (band[0], hi), btype="bandpass", fs=fs, output="sos")
    return signal.sosfiltfilt(sos, x)


def _lp(x, fs, fc, order=3):
    sos = signal.butter(order, min(fc, 0.45 * fs), btype="lowpass", fs=fs, output="sos")
    return signal.sosfiltfilt(sos, x)


def _runs(mask):
    """[(start, stop)] index runs where mask is True."""
    d = np.diff(np.concatenate([[0], mask.astype(int), [0]]))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def expected_bin_amplitude(range_m, rubble_path_m=0.0, params=None):
    """Model-predicted complex amplitude of a chest return in its range bin (IF units x range-FFT gain).

    Uses the SAME public propagation model the planner uses (radar_dsp.reflection_amplitude). Real chest
    RCS varies by +-10 dB with aspect / posture and real rubble loss is not this scalar, so any depth
    derived from it is low-confidence by construction."""
    from . import radar_dsp as R
    P = params or R.RadarParams()
    gain = float(np.hanning(P.n_fast + 2)[1:-1].sum())
    return R.reflection_amplitude(float(range_m), float(rubble_path_m or 0.0), R.CHEST_RCS_DB, P) * gain


def demodulate(iq, wavelength=None, expected_amp=None):
    """Complex slow-time bin -> (displacement, units, method). units 'm' when calibrated, else 'rel'."""
    x = np.asarray(iq, complex)
    c, r = _circle_fit(x)
    y = x - c
    ang = np.unwrap(np.angle(y))
    arc = float(np.percentile(ang, 98) - np.percentile(ang, 2))
    resid = float(np.std(np.abs(y) - r) / max(r, 1e-12))
    k = wavelength / (4 * np.pi) if wavelength else None
    # arctangent only when the points trace a real arc: wide enough, on a ring (not a noise blob), and the
    # phase moves smoothly sample to sample (a noise blob around the fitted centre gives random phase steps)
    step = float(np.median(np.abs(np.diff(ang))))
    z = x - x.mean()
    ev, v = np.linalg.eigh(np.cov(np.vstack([z.real, z.imag])))
    bend = float(np.sqrt(max(ev[0], 0.0) / max(ev[1], 1e-300)))   # ~0 for a short (straight) arc, ~1 for a full circle
    if r > 0 and arc >= MIN_ARC_RAD and bend >= MIN_BEND and resid < MAX_RING_RESID and step < MAX_PHASE_STEP:
        return (ang - ang.mean()) * (k or 1.0), ("m" if k else "rad"), "arctan"
    # short arc: principal-axis linear demodulation, phase ~= projection / amplitude
    proj = z.real * v[0, -1] + z.imag * v[1, -1]
    if expected_amp and k:
        return proj / float(expected_amp) * k, "m", "linear+pathloss"
    return proj / max(float(np.std(proj)), 1e-30), "rel", "linear"


# ----------------------------------------------------------------------------- features
def features(d, fs, units="m"):
    """Feature dict from a displacement series ``d`` (metres if units == 'm', else arbitrary)."""
    d = signal.detrend(np.asarray(d, float), type="linear")
    n = d.size
    dur = n / fs
    out = {"duration_s": dur}

    # --- movement / artefact frames (1 s) on out-of-breathing-band energy
    hf = _bp(d, fs, MOVE_BAND_HZ) if fs > 2 * MOVE_BAND_HZ[1] else np.zeros(n)
    fl = max(int(fs), 1)
    nf = n // fl
    frms = np.sqrt(np.mean(hf[:nf * fl].reshape(nf, fl) ** 2, axis=1)) if nf else np.zeros(0)
    w = _bp(d, fs, WAVE_BAND_HZ)
    wrms = float(np.std(w))
    moving = np.zeros(n, bool)
    if nf:
        thr = MOVE_K * max(float(np.median(frms)), 1e-30)
        thr = max(thr, 0.15 * wrms)            # small high-band wiggles relative to breathing are harmless
        fm = frms > thr
        fm = np.convolve(fm.astype(int), [1, 1, 1], "same") > 0      # dilate by one frame each side
        moving[:nf * fl] = np.repeat(fm, fl)
    out["movement"] = float(moving.mean())

    # --- band SNR: breathing-band power vs a high-frequency noise floor
    f, p = signal.periodogram(d * np.hanning(n), fs, nfft=max(8192, n))
    hi = (f >= 2.0) & (f <= min(5.0, 0.48 * fs))
    if hi.sum() >= 5:
        noise_psd = float(np.median(p[hi]))
        band = (f >= WAVE_BAND_HZ[0]) & (f <= WAVE_BAND_HZ[1])
        sig_pow = max(float(p[band].sum()) - noise_psd * band.sum(), 1e-30)
        out["snr_db"] = float(10 * np.log10(sig_pow / max(noise_psd * band.sum(), 1e-30)))
    else:
        out["snr_db"] = None

    # --- spectral rate on the waveform
    fw, pw = signal.periodogram(w * np.hanning(n), fs, nfft=max(16384, 4 * n))
    rb = (fw >= RATE_BAND_HZ[0]) & (fw <= RATE_BAND_HZ[1])
    k = np.flatnonzero(rb)[np.argmax(pw[rb])]
    if 0 < k < len(pw) - 1:
        a_, b_, c_ = np.log(pw[k - 1:k + 2] + 1e-300)
        den = a_ - 2 * b_ + c_
        off = 0.5 * (a_ - c_) / den if den != 0 else 0.0
    else:
        off = 0.0
    f_spec = float(fw[k] + np.clip(off, -0.5, 0.5) * (fw[1] - fw[0]))
    near = rb & (np.abs(fw - f_spec) <= 0.04)
    out["rate_spectral_bpm"] = 60 * f_spec
    out["peakiness"] = float(pw[near].sum() / max(pw[rb].sum(), 1e-300))

    # --- apnea: moving-RMS envelope of the waveform
    m = max(int(APNEA_WIN_S * fs), 3)
    wa = _bp(d, fs, APNEA_BAND_HZ)
    env = np.sqrt(np.convolve(wa ** 2, np.ones(m) / m, "same"))
    level = float(np.percentile(env[~moving] if (~moving).sum() > m else env, 80))
    low = (env < APNEA_FRAC * level) & ~moving
    apneas = [(a / fs, b / fs) for a, b in _runs(low) if (b - a) / fs >= APNEA_MIN_S]
    out["apneas"] = apneas
    out["apnea_events"] = len(apneas)
    out["apnea_events_per_min"] = 60.0 * len(apneas) / dur
    out["apnea_longest_s"] = max([b - a for a, b in apneas], default=0.0)

    # --- time-domain breaths: peaks of the smoothed waveform
    ws = _lp(w, fs, min(1.2, max(0.5, 2.5 * f_spec)))
    span = float(np.percentile(ws[~moving] if (~moving).sum() > fs else ws, 95)
                 - np.percentile(ws[~moving] if (~moving).sum() > fs else ws, 5))
    dist = int(np.clip(0.35 / max(f_spec, 0.05), 1.0, 2.0) * fs)    # >= 1 s between breath peaks
    best = None
    for sgn in (1.0, -1.0):                    # polarity of chest motion is unknown through rubble
        pk, _ = signal.find_peaks(sgn * ws, prominence=0.3 * span, distance=dist)
        tr, _ = signal.find_peaks(-sgn * ws, prominence=0.3 * span, distance=dist)
        if best is None or len(pk) > len(best[0]):
            best = (pk, tr, sgn)
    pk, tr, sgn = best
    pk = np.array([i for i in pk if not moving[i]], int)
    t_pk = pk / fs
    iv = np.diff(t_pk)
    ap_gap = max(APNEA_MIN_S, 2.2 * (np.median(iv) if iv.size else 0))
    # intervals spanning an apnea or a movement artefact are not breath intervals
    ok = np.array([g < ap_gap and not any(a < t1 and b > t0 for a, b in apneas)
                   and not moving[int(t0 * fs):int(t1 * fs)].any()
                   for t0, t1, g in zip(t_pk[:-1], t_pk[1:], iv)], bool) if iv.size else np.zeros(0, bool)
    ivc = iv[ok]
    out["n_breaths"] = int(pk.size)
    out["n_intervals"] = int(ivc.size)
    out["intervals_s"] = ivc
    out["breath_times_s"] = t_pk
    out["rate_td_bpm"] = float(60.0 / np.median(ivc)) if ivc.size else None
    # robust CV (1.4826 MAD / median): one sigh or one missed peak should not make a regular breather "irregular"
    out["cv"] = float(1.4826 * np.median(np.abs(ivc - np.median(ivc))) / np.median(ivc)) if ivc.size >= 3 else None
    out["cv_classic"] = float(np.std(ivc, ddof=1) / np.mean(ivc)) if ivc.size >= 3 else None

    # half-cycle asymmetry and per-breath depth from peak / preceding trough pairs
    halves, depths = [], []
    for i in pk:
        before = tr[tr < i]
        after = tr[tr > i]
        if before.size and after.size:
            t_up, t_dn = (i - before[-1]) / fs, (after[0] - i) / fs
            if 0 < t_up < 10 and 0 < t_dn < 12:
                halves.append(min(t_up, t_dn) / max(t_up, t_dn))
        if before.size:
            depths.append(sgn * (ws[i] - ws[before[-1]]))
    out["ie_ratio"] = float(np.median(halves)) if halves else None
    out["depth_pp"] = float(np.median(depths)) if depths else None
    out["units"] = units
    return out


# ----------------------------------------------------------------------------- classifier
def classify(fe: dict, sqi: float, depth_rel=None, depth_conf=0.0, quality=None):
    """Rule-based pattern from features. Returns (pattern, conf, reason)."""
    rate = fe.get("rate_bpm")
    cv = fe.get("cv")
    ci = fe.get("rate_ci") or (rate, rate)
    half = 0.5 * (ci[1] - ci[0]) if rate else 0.0
    if rate is None or sqi < SQI_MIN or (fe["n_breaths"] < 2 and not fe["apnea_events"]):
        return "unclear", 0.0, f"signal quality {sqi:.2f} below {SQI_MIN} or too few breaths"

    def margin(x, thr, scale):
        return float(np.clip(0.5 + abs(x - thr) / (2 * scale), 0.5, 1.0))
    s = max(half, 1.0)
    evidence = float(np.clip(fe["n_intervals"] / 5.0, 0.4, 1.0))
    if fe["apnea_events"]:
        conf = sqi * float(np.clip(fe["apnea_longest_s"] / 15.0, 0.6, 1.0))
        return "irregular_with_pauses", conf, f"{fe['apnea_events']} pause(s) > {APNEA_MIN_S:.0f} s, longest {fe['apnea_longest_s']:.0f} s"
    if rate > RATE_FAST:
        return "rapid_shallow", sqi * margin(rate, RATE_FAST, s), f"rate {rate:.0f}/min > {RATE_FAST:.0f}"
    cv_thr = CV_IRREGULAR + CV_Q_SLOPE * (1.0 - (1.0 if quality is None else float(np.clip(quality, 0, 1))))
    if cv is not None and cv >= cv_thr and rate < RATE_ELEVATED:
        return "irregular_with_pauses", sqi * evidence * margin(cv, cv_thr, 0.1), \
            f"breath-to-breath variability {cv:.2f} >= {cv_thr:.2f}"
    if rate < RATE_SLOW:
        return "slow_deep", sqi * margin(rate, RATE_SLOW, s), f"rate {rate:.0f}/min < {RATE_SLOW:.0f}"
    if rate >= RATE_ELEVATED:
        if rate >= RATE_RAPID and not (cv is not None and cv < CV_VERY_REGULAR):
            return "rapid_shallow", sqi * margin(rate, RATE_RAPID, s), f"rate {rate:.0f}/min >= {RATE_RAPID:.0f}"
        if cv is not None and cv >= CV_FAST_IRREGULAR:
            return "rapid_shallow", sqi * evidence * margin(cv, CV_FAST_IRREGULAR, 0.1), \
                f"rate {rate:.0f}/min, irregular (variability {cv:.2f})"
        conf = min(CHILD_CONF_CAP, sqi * evidence * margin(rate, RATE_ELEVATED, s))
        if depth_rel is not None and depth_conf > 0 and depth_rel > CHILD_DEPTH_MAX:
            conf *= 0.5                     # adult-like excursion argues against a child, but depth is unreliable
        return "fast_small", conf, f"fast ({rate:.0f}/min) and regular: possible child (weak inference)"
    conf = sqi * evidence * margin(rate, RATE_SLOW, s) * margin(rate, RATE_ELEVATED, s)
    return "regular_calm", conf, f"regular, rate {rate:.0f}/min within {RATE_SLOW:.0f}-{RATE_ELEVATED:.0f}"


def triage(pattern: str, rate=None):
    """START / SALT-style respiratory hint for the dig order (an aid, not a diagnosis)."""
    if pattern == "rapid_shallow":
        if rate is not None and rate > RATE_FAST:
            return "immediate", f"breathing over {RATE_FAST:.0f}/min (START respiratory criterion for IMMEDIATE)"
        return "immediate (check)", "fast, irregular breathing: possible distress or shock"
    if pattern == "irregular_with_pauses":
        return "immediate (check)", "irregular breathing or pauses: possible airway problem or reduced consciousness"
    if pattern == "slow_deep":
        return "immediate (check)", "slow breathing (< 11/min): possible reduced consciousness"
    if pattern == "regular_calm":
        return "delayed", "regular breathing in the normal range (respiration only; perfusion and responsiveness unknown)"
    if pattern == "fast_small":
        return "delayed (re-check)", "fast regular breathing; normal for a child, elevated for an adult: re-check"
    return "assess", "breathing pattern unclear from this scan: needs a longer or better scan"


def priority_weight(v: dict) -> float:
    """Dig-order weight in [0.35, 1]: the triage rank shrunk towards 'assess' when pattern_conf is low.
    The decider multiplies this by localisation certainty and accessibility (see contracts)."""
    r = TRIAGE_RANK.get(v.get("triage_hint", "assess"), TRIAGE_RANK["assess"])
    c = float(np.clip(v.get("pattern_conf", 0.0), 0.0, 1.0))
    return round(TRIAGE_RANK["assess"] + (r - TRIAGE_RANK["assess"]) * c, 3)


def with_rubble_path(v: dict, range_m: float, rubble_path_m: float, params=None) -> dict:
    """Re-scale a path-loss depth estimate once the planner has a rubble-path estimate (e.g. from depth_m).

    process_if does not know the rubble path, so vitals attached there assume none; call this when the
    ping's depth is known. Only touches depth_rel for the 'linear+pathloss' demodulation."""
    if v.get("demod") != "linear+pathloss" or v.get("depth_rel") is None:
        return v
    a0 = expected_bin_amplitude(range_m, 0.0, params)
    a1 = expected_bin_amplitude(range_m, rubble_path_m, params)
    out = dict(v)
    out["depth_rel"] = round(float(v["depth_rel"]) * a0 / max(a1, 1e-30), 2)
    return out


def dig_score(v: dict | None, p_mass: float, depth_m: float | None = None) -> float:
    """Suggested dig-order score: triage weight x localisation certainty x accessibility.

    accessibility = clip(1 - depth_m / 4, 0.2, 1) (shallower is faster to reach); no vitals -> 'assess' weight."""
    w = priority_weight(v) if v else TRIAGE_RANK["assess"]
    acc = 1.0 if depth_m is None else float(np.clip(1.0 - float(depth_m) / 4.0, 0.2, 1.0))
    return round(w * float(np.clip(p_mass, 0.0, 1.0)) * acc, 3)


# ----------------------------------------------------------------------------- public API
def extract(x, fs: float = 20.0, *, kind: str = "auto", wavelength=None, range_m=None, rubble_path_m=None,
            expected_amp=None, quality=None, snr_db=None, params=None, keep_arrays: bool = False) -> dict:
    """Vital-sign pattern from one slow-time series.

    x     complex I/Q of the candidate's range bin (kind 'iq'), phase in radians ('phase'), displacement
          in metres ('displacement') or any displacement-proportional series ('relative').
    fs    slow-time sample rate (Hz). Use >= 30 s for apnea / variability claims; 20 s works for rate.
    wavelength  carrier wavelength (m) to convert phase to metres (RadarParams.wavelength).
    range_m, rubble_path_m  candidate range and the planner's estimate of the rubble along the path,
          used only for the low-confidence depth estimate when the I/Q arc is too small to self-calibrate.
    quality, snr_db  scan quality (0-1) and the DSP's candidate SNR, folded into the SQI.
    Returns the contract dict (contracts/INTERFACES.md, "vitals"), JSON-safe.
    """
    x = np.asarray(x)
    if kind == "auto":
        kind = "iq" if np.iscomplexobj(x) else ("phase" if wavelength else "relative")
    method = kind
    if kind == "iq":
        if expected_amp is None and range_m is not None:
            expected_amp = expected_bin_amplitude(range_m, rubble_path_m, params)
        d, units, method = demodulate(x, wavelength, expected_amp)
    elif kind == "phase":
        d, units = np.unwrap(np.asarray(x, float)) * (wavelength / (4 * np.pi) if wavelength else 1.0), \
            ("m" if wavelength else "rad")
    elif kind == "displacement":
        d, units = np.asarray(x, float), "m"
    else:
        d, units = np.asarray(x, float), "rel"
    fe = features(d, fs, units)

    # rate + CI
    n_int = fe["n_intervals"]
    if n_int >= 3 and fe["rate_td_bpm"]:
        rate = fe["rate_td_bpm"]
        iv = fe["intervals_s"]
        se = float(np.std(iv, ddof=1) / np.sqrt(iv.size))
        lo, hi = 60.0 / (np.median(iv) + 1.96 * se), 60.0 / max(np.median(iv) - 1.96 * se, 0.3)
        rate_src = "breath count"
    else:
        rate = fe["rate_spectral_bpm"]
        hw = 60.0 * 0.5 / fe["duration_s"]
        lo, hi = rate - hw, rate + hw
        rate_src = "spectrum"
    lo, hi = min(lo, rate - 0.5), max(hi, rate + 0.5)
    fe["rate_bpm"], fe["rate_ci"] = rate, (lo, hi)

    # signal quality index
    snr_term = 1.0 / (1.0 + np.exp(-((fe["snr_db"] if fe["snr_db"] is not None else SNR_MID_DB) - SNR_MID_DB) / 2.5))
    if snr_db is not None:                                          # the DSP's own candidate SNR (dB over CFAR floor)
        snr_term *= float(np.clip(0.5 + (snr_db - 6.0) / 12.0, 0.3, 1.0))
    sqi = snr_term * (1.0 - fe["movement"]) ** 1.5
    sqi *= float(np.clip((fe["n_breaths"] + 2 * fe["apnea_events"]) / 3.0, 0.2, 1.0)) ** 0.5
    if quality is not None:
        sqi *= 0.4 + 0.6 * float(np.clip(quality, 0, 1))
    sqi = float(np.clip(sqi, 0.0, 1.0))

    # depth (LOW confidence by construction)
    depth_rel, depth_conf = None, 0.0
    if units == "m" and fe["depth_pp"]:
        depth_rel = fe["depth_pp"] / DEPTH_REF_M
        depth_conf = (0.3 if method in ("arctan", "displacement", "phase") else 0.15) * sqi

    pattern, conf, reason = classify(fe, sqi, depth_rel, depth_conf, quality)
    hint, hint_why = triage(pattern, rate)
    caveats = list(CAVEATS_ALWAYS)
    caveats.append(f"rate_bpm: from {rate_src} over {fe['duration_s']:.0f} s; through rubble the SNR is far lower than in the lab")
    caveats.append("depth_rel: LOW confidence, chest-wall amplitude depends on aspect, posture, range and the rubble in the path"
                   + ("; here scaled by a path-loss MODEL, not measured" if method == "linear+pathloss" else ""))
    if fe["duration_s"] < 30:
        caveats.append(f"cv / apnea: a {fe['duration_s']:.0f} s window holds few breaths and cannot contain many > 10 s pauses; use a >= 45 s vitals dwell")
    if pattern == "fast_small":
        caveats.append("possible child: fast, small, regular breathing is a weak hint of age; a distressed adult can look the same")
    if fe["movement"] > 0.2:
        caveats.append(f"movement / artefacts in {100 * fe['movement']:.0f}% of the window (person moving, rubble shifting or platform)")
    caveats.append("cardiac motion (~0.1 mm) is not reliably resolvable through rubble and is not reported")

    r2 = lambda v, n=2: None if v is None else round(float(v), n)
    out = {
        "version": VERSION,
        "rate_bpm": r2(rate, 1), "rate_ci": [r2(lo, 1), r2(hi, 1)],
        "rate_spectral_bpm": r2(fe["rate_spectral_bpm"], 1), "rate_count_bpm": r2(fe["rate_td_bpm"], 1),
        "cv": r2(fe["cv"]), "ie_ratio": r2(fe["ie_ratio"]),
        "depth_rel": r2(depth_rel), "depth_rel_conf": r2(depth_conf),
        "apnea_events": int(fe["apnea_events"]), "apnea_events_per_min": r2(fe["apnea_events_per_min"]),
        "apnea_longest_s": r2(fe["apnea_longest_s"], 1),
        "movement": r2(fe["movement"]), "snr_db": r2(fe["snr_db"], 1), "sqi": r2(sqi),
        "n_breaths": int(fe["n_breaths"]), "duration_s": r2(fe["duration_s"], 1), "demod": method,
        "pattern": pattern, "pattern_conf": r2(conf), "pattern_reason": reason,
        "triage_hint": hint, "triage_reason": hint_why,
        "caveats": caveats,
    }
    out["priority_weight"] = priority_weight(out)
    if keep_arrays:
        out["_displacement"] = d
        out["_apneas_s"] = fe["apneas"]
        out["_breath_times_s"] = fe["breath_times_s"]
    return out


def extract_from_obs(obs_debug: dict) -> dict:
    """Dict front end for the mission / radar model.

    Preferred input (one candidate):
        {"slow_time_iq": complex (n_slow,)  -- Xc[:, k] of process_if (motion-compensated, clutter NOT removed)
                         or [[re, im], ...] (JSON),
         "fs": 20.0, "wavelength": RadarParams.wavelength, "range_m": candidate range,
         "rubble_path_m": planner's estimate (optional), "quality": scan quality (optional),
         "snr_db": candidate snr_db (optional), "n_fast": RadarParams.n_fast (optional, default 512)}
    Fallback: the process_if debug dict ({"slow_time_phase": PCA displacement, "t": ...}) -> uncalibrated
    (depth_rel None), same features otherwise.
    """
    o = obs_debug
    fs = float(o.get("fs", 0) or (1.0 / float(np.median(np.diff(o["t"]))) if "t" in o else 20.0))
    params = None
    if o.get("n_fast"):
        from . import radar_dsp as R
        params = R.RadarParams(n_fast=int(o["n_fast"]))
    if "slow_time_iq" in o:
        iq = np.asarray(o["slow_time_iq"])
        if not np.iscomplexobj(iq) and iq.ndim == 2 and iq.shape[1] == 2:
            iq = iq[:, 0] + 1j * iq[:, 1]
        return extract(iq, fs, kind="iq", wavelength=o.get("wavelength"), range_m=o.get("range_m"),
                       rubble_path_m=o.get("rubble_path_m"), quality=o.get("quality"), snr_db=o.get("snr_db"),
                       params=params)
    if "slow_time_phase" in o:
        return extract(np.asarray(o["slow_time_phase"], float), fs, kind="relative", quality=o.get("quality"))
    raise KeyError("extract_from_obs needs 'slow_time_iq' (preferred) or 'slow_time_phase'")
