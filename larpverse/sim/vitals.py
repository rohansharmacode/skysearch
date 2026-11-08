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

It reads the survivor's CONDITION from the breathing only (no age / child inference). Patterns:
regular_calm, rapid_shallow, slow_regular, irregular_with_pauses, unclear. The triage hint follows START /
SALT RESPIRATORY criteria (rate > 30 -> immediate, irregular or pauses -> immediate (check), slow but regular
-> urgent, 11-20 regular -> delayed). It is an aid for rescuers ordering the dig, never a diagnosis: START
also needs perfusion and mental status, which a radar cannot assess.
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
APNEA_FRAC = 0.45           # envelope below 45% of the breathing level counts as "no breathing motion"
GAP_ENV_FRAC = 0.8          # a breath-free gap >= 10 s (and >= 2.2 median intervals) counts as a pause if its
                            # envelope stays below 60% of the breathing level
APNEA_WIN_PERIODS = 0.6     # envelope window = 0.6 breath periods, clipped to [APNEA_WIN_S, APNEA_WIN_MAX_S]
APNEA_WIN_MAX_S = 6.0
APNEA_WIN_S = 3.0           # moving-RMS window for the envelope
APNEA_BAND_HZ = (0.08, 0.5) # envelope band (upper edge raised to 1.6 x the breathing rate for fast breathers)
MIN_ARC_RAD = 0.6           # I/Q arc needed for a trustworthy circle fit (arctangent demodulation)
MAX_RING_RESID = 0.6        # std(|y| - r) / r: a centred noise blob gives ~0.5-0.6, real 24 GHz records 0.15-0.5
MIN_BEND = 0.25             # sqrt(minor / major I/Q variance): below this the arc is too short to fit a circle
MAX_PHASE_STEP = 0.15       # median |phase step| per sample (rad): real records ~0.05, noise blobs >= 0.25
SQI_MIN = 0.35              # below this the pattern is "unclear"
SNR_MID_DB = 8.0            # band SNR at which the SNR term of the SQI is 0.5

RATE_FAST = 30.0            # START: respiratory rate > 30/min -> IMMEDIATE
RATE_FAST_CHECK = 25.0      # 25-30/min: rapid shallow, "immediate (check)"; 20-25: "urgent"
RATE_ELEVATED = 20.0        # adult resting normal is about 12-20/min
RATE_SLOW = 11.0            # below this: slow breathing (bradypnoea)
CV_IRREGULAR = 0.30         # robust breath-interval CV above this = irregular (at scan quality 1); real resting adults read 0.1-0.28
CV_Q_SLOPE = 0.25           # ... raised by this x (1 - quality): platform motion jitters breath timing
AMP_CV_IRREGULAR = 0.35     # robust breath-to-breath DEPTH variability above this (with CV >= AMP_CV_WITH x the CV
AMP_CV_WITH = 0.5           # threshold) also reads as irregular: waxing / waning breaths around pauses

# Condition patterns (no age / child class: the radar reads the breathing, not the person)
PATTERNS = ("regular_calm", "rapid_shallow", "slow_regular", "irregular_with_pauses", "unclear")
# contract v1.3.1 condition names (vitals.condition); vitals.pattern stays as a legacy alias
CONDITION_OF = {"regular_calm": "calm", "rapid_shallow": "distressed_tachypnea", "slow_regular": "unconscious_slow",
                "irregular_with_pauses": "irregular_apneic", "unclear": "unknown"}
CONDITIONS = ("calm", "distressed_tachypnea", "unconscious_slow", "irregular_apneic", "unknown")
# rescuer-facing hint and its dig-order rank (higher = dig first); "assess" sits in the middle on purpose
TRIAGE_RANK = {"immediate": 1.0, "immediate (check)": 0.85, "urgent": 0.7, "assess": 0.55, "delayed": 0.35,
               "reject": 0.15}       # reject: the rejector says this return is not human breathing

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
    d_raw = np.asarray(d, float)
    d = signal.detrend(d_raw, type="linear")
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
    # envelope window scales with the breath period so a slow breather's end-expiratory pause is not a "gap"
    m = max(int(float(np.clip(APNEA_WIN_PERIODS / max(f_spec, 0.05), APNEA_WIN_S, APNEA_WIN_MAX_S)) * fs), 3)
    wa = _bp(d, fs, (APNEA_BAND_HZ[0], float(np.clip(1.6 * f_spec, APNEA_BAND_HZ[1], 0.8))))
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
    dp = np.abs(np.array(depths, float))
    # robust breath-to-breath depth variability (shape of the breathing, NOT absolute amplitude)
    out["amp_cv"] = float(1.4826 * np.median(np.abs(dp - np.median(dp))) / max(np.median(dp), 1e-30)) if dp.size >= 4 else None
    # long gaps between detected breaths with a low envelope are pauses too (the envelope alone misses
    # pauses that platform / rubble motion keeps above APNEA_FRAC)
    for t0, t1, g in zip(t_pk[:-1], t_pk[1:], iv):
        if g >= ap_gap and not moving[int(t0 * fs):int(t1 * fs)].any() \
                and not any(a < t1 and b > t0 for a, b in apneas):
            inner = env[int((t0 + 1.5) * fs):int((t1 - 1.5) * fs)]
            if inner.size and float(np.median(inner)) < GAP_ENV_FRAC * level:
                apneas.append((float(t0 + 1.0), float(t1 - 1.0)))
    apneas.sort()
    out["apneas"] = apneas
    out["apnea_events"] = len(apneas)
    out["apnea_events_per_min"] = 60.0 * len(apneas) / dur
    out["apnea_longest_s"] = max([b - a for a, b in apneas], default=0.0)
    out["depth_pp"] = float(np.median(depths)) if depths else None
    out["units"] = units

    # --- confounder features (sim/vitals.source_of): slow steps / creep, spectral lines, broadband motion
    e = max(int(2 * fs), 1)
    slow = _lp(d_raw - d_raw[0], fs, 0.04) if n > 4 * e else d_raw - d_raw[0]
    out["slow_pp"] = float(np.ptp(slow[e:-e])) if n > 4 * e else float(np.ptp(slow))
    out["noise_rms"] = float(1.4826 * np.median(np.abs(np.diff(d) - np.median(np.diff(d)))) / np.sqrt(2))
    fa, pa = signal.periodogram(d * np.hanning(n), fs, nfft=max(16384, 4 * n))
    band = (fa >= 0.08) & (fa <= 0.48 * fs)
    floor = float(np.median(pa[band]))
    pex = np.maximum(pa - floor, 0.0) * band
    kf = int(np.argmax(pex))
    f_line = float(fa[kf])
    lines = np.zeros_like(band)
    for h in range(1, 5):                                     # fundamental + harmonics (aliased into 0..fs/2)
        fh = abs(((h * f_line + fs / 2) % fs) - fs / 2)
        lines |= np.abs(fa - fh) <= 0.05
    out["line_hz"] = f_line
    out["line_db"] = float(10 * np.log10(max(pa[kf], 1e-300) / max(floor, 1e-300)))
    out["line_frac"] = float(pex[lines].sum() / max(pex.sum(), 1e-300))
    hi_b = band & (fa >= 1.5)
    out["hf_frac"] = float(pa[hi_b].sum() / max(pa[band].sum(), 1e-300))
    out["hf_rms"] = float(np.sqrt(2 * pa[hi_b].sum() * (fa[1] - fa[0]))) if hi_b.any() else 0.0
    # level shift: largest difference between the medians of adjacent 10 s stretches (debris steps persist;
    # breathing and knocks return to the same level)
    wl = int(STEP_WIN_S * fs)
    lp1 = _lp(d_raw, fs, 1.0) if n > 4 * wl // 3 else d_raw
    out["step"] = float(max((abs(np.median(lp1[i - wl:i]) - np.median(lp1[i:i + wl]))
                             for i in range(wl, n - wl + 1, max(int(fs), 1))), default=0.0))
    # bursts: 1 s frames whose sample-to-sample motion is far above the scan's median frame (knocks, steps)
    dd = np.abs(np.diff(d_raw))
    nb = dd.size // fl
    if nb >= 3:
        fr = np.sqrt(np.mean(dd[:nb * fl].reshape(nb, fl) ** 2, axis=1))
        med = max(float(np.median(fr)), 1e-30)
        out["burst"] = float(np.max(fr) / med)
        out["n_bursts"] = int(len(_runs(fr > BURST_FRAME_X * med)))
    else:
        out["burst"], out["n_bursts"] = 1.0, 0
    return out


# ----------------------------------------------------------------------------- classifier
def classify(fe: dict, sqi: float, quality=None):
    """Rule-based condition pattern from features. Returns (pattern, conf, reason)."""
    rate = fe.get("rate_bpm")
    cv, acv = fe.get("cv"), fe.get("amp_cv")
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
    if rate >= RATE_ELEVATED:
        return "rapid_shallow", sqi * margin(rate, RATE_ELEVATED, s), f"rate {rate:.0f}/min >= {RATE_ELEVATED:.0f}"
    cv_thr = CV_IRREGULAR + CV_Q_SLOPE * (1.0 - (1.0 if quality is None else float(np.clip(quality, 0, 1))))
    if cv is not None and cv >= cv_thr:
        return "irregular_with_pauses", sqi * evidence * margin(cv, cv_thr, 0.1), \
            f"breath-to-breath timing variability {cv:.2f} >= {cv_thr:.2f}"
    if cv is not None and acv is not None and acv >= AMP_CV_IRREGULAR and cv >= AMP_CV_WITH * cv_thr:
        return "irregular_with_pauses", 0.8 * sqi * evidence * margin(acv, AMP_CV_IRREGULAR, 0.2), \
            f"breath depth varies {acv:.2f} breath to breath, timing variability {cv:.2f}"
    if rate < RATE_SLOW:
        return "slow_regular", sqi * evidence * margin(rate, RATE_SLOW, s), f"rate {rate:.0f}/min < {RATE_SLOW:.0f}, regular, no pauses"
    conf = sqi * evidence * margin(rate, RATE_SLOW, s) * margin(rate, RATE_ELEVATED, s)
    return "regular_calm", conf, f"regular, rate {rate:.0f}/min within {RATE_SLOW:.0f}-{RATE_ELEVATED:.0f}"


def triage(pattern: str, rate=None, fe: dict | None = None):
    """START / SALT-style respiratory hint + plain-words condition. Returns (hint, condition, criterion).

    The condition sentence says what the breathing looks like and what it MAY mean; never a diagnosis."""
    r = f"{rate:.0f}/min" if rate is not None else "rate unknown"
    fe = fe or {}
    if pattern == "rapid_shallow":
        cond = f"rapid shallow breathing, {r} -> possible injury, pain or distress, check"
        if rate is not None and rate > RATE_FAST:
            return "immediate", cond, f"breathing over {RATE_FAST:.0f}/min (START respiratory criterion for IMMEDIATE)"
        if rate is not None and rate >= RATE_FAST_CHECK:
            return "immediate (check)", cond, f"breathing {RATE_FAST_CHECK:.0f}-{RATE_FAST:.0f}/min: close to the START cut-off"
        return "urgent", cond, f"breathing faster than the adult resting range ({RATE_ELEVATED:.0f}/min)"
    if pattern == "irregular_with_pauses":
        n = fe.get("apnea_events", 0)
        what = f"irregular breathing with {n} pause(s) over {APNEA_MIN_S:.0f} s" if n else "irregular breathing"
        return "immediate (check)", f"{what}, {r} -> possible airway problem or reduced consciousness, check", \
            "irregular breathing or pauses (SALT: breathing present but abnormal)"
    if pattern == "slow_regular":
        return "urgent", (f"slow, regular breathing, {r} -> could be deep rest or reduced consciousness; the radar "
                          f"cannot tell which, so call out and check responsiveness"), \
            f"slow breathing below {RATE_SLOW:.0f}/min, regular and without pauses"
    if pattern == "regular_calm":
        return "delayed", f"calm, regular breathing, {r} -> stable for now (breathing only), re-check later", \
            "regular breathing in the normal range (perfusion and responsiveness unknown)"
    return "assess", "breathing pattern unclear -> rescan longer or from a better landing spot", \
        "no usable breathing pattern from this scan"


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
            expected_amp=None, quality=None, snr_db=None, params=None, keep_arrays: bool = False,
            context: dict | None = None) -> dict:
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

    pattern, conf, reason = classify(fe, sqi, quality)
    hint, condition, hint_why = triage(pattern, rate, fe)
    caveats = list(CAVEATS_ALWAYS)
    caveats.append(f"rate_bpm: from {rate_src} over {fe['duration_s']:.0f} s; through rubble the SNR is far lower than in the lab")
    caveats.append("depth_rel: LOW confidence, chest-wall amplitude depends on aspect, posture, range and the rubble in the path"
                   + ("; here scaled by a path-loss MODEL, not measured" if method == "linear+pathloss" else ""))
    if fe["duration_s"] < 30:
        caveats.append(f"cv / apnea: a {fe['duration_s']:.0f} s window holds few breaths and cannot contain many > 10 s pauses; use a >= 45 s vitals dwell")
    if pattern == "slow_regular":
        caveats.append("slow regular breathing looks the same in deep rest and in reduced consciousness; only a voice / responsiveness check separates them")
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
        "amp_cv": r2(fe.get("amp_cv")),
        "pattern": pattern, "pattern_conf": r2(conf), "pattern_reason": reason,
        "condition": CONDITION_OF[pattern], "condition_text": condition,
        "triage_hint": hint, "triage_reason": hint_why,
        "caveats": caveats,
    }
    k_mm = 1e3 if units == "m" else 1.0
    out["signature"] = {"slow_pp_mm": r2(k_mm * fe["slow_pp"], 3), "step_mm": r2(k_mm * fe["step"], 3),
                        "line_db": r2(fe["line_db"], 1), "noise_mm": r2(k_mm * fe["noise_rms"], 4),
                        "line_hz": r2(fe["line_hz"], 3), "line_frac": r2(fe["line_frac"]), "hf_frac": r2(fe["hf_frac"]),
                        "hf_rms_mm": r2(k_mm * fe["hf_rms"], 3), "burst": r2(fe["burst"], 1), "n_bursts": int(fe["n_bursts"]),
                        "depth_pp_mm": r2(k_mm * fe["depth_pp"], 3) if fe["depth_pp"] else None,
                        "peakiness": r2(fe["peakiness"]), "units": "mm" if units == "m" else units}
    apply_source(out, source_of(out, context))
    out["priority_weight"] = priority_weight(out)
    if keep_arrays:
        out["_displacement"] = d
        out["_apneas_s"] = fe["apneas"]
        out["_breath_times_s"] = fe["breath_times_s"]
    return out


# ----------------------------------------------------------------------------- confounder rejector
SOURCES = ("human_breathing", "no_signal", "debris_shift", "small_animal", "machinery", "rotor", "moving_person")
LINE_MIN_HZ = 0.8           # a dominant spectral line above this (outside breathing) with ...
LINE_FRAC_MIN = 0.25        # ... this share of the excess power in the line + harmonics = machinery
TONE_CV_MAX = 0.02          # in-band but too exact for a body: breath CV and depth variability below these ...
TONE_AMPCV_MAX = 0.03       # ... with a strong line = machinery
ROTOR_FLAT_MIN = 0.30       # complex-bin PSD density 2-9 Hz / 0.1-0.75 Hz above this (white, like aliased blade
ROTOR_HF_DB = 15.0          # flashes) and 1.5-9 Hz power this far above the scan's thermal floor = rotor
STEP_WIN_S = 10.0           # level-shift detector: medians of adjacent 10 s stretches
DEBRIS_STEP_BREATH_X = 1.5  # a persistent level shift >= 1.5 x the breath size (and >= DEBRIS_MM) = settling debris
LINE_DB_MIN = 15.0          # a spectral line this far above the median spectrum ...
BURST_FRAME_X = 4.5         # a 1 s frame whose sample-to-sample motion is 6x the median frame is a burst
MOVING_MIN_BURSTS = 3       # >= 3 separate bursts in the window = person moving / knocking
REFLECTOR_DB_MIN_LIN = 1000.0  # a bin "holds a reflector" if its static power is 30 dB over the thermal noise
ANIMAL_DEPTH_MAX = 0.25     # human-like rhythm but chest excursion below 0.25 x an adult's (path-loss scaled) ...
ANIMAL_RATE_MIN = 18.0      # ... at 18+/min = possible small animal (LOW confidence: amplitude is unreliable)
DEBRIS_OVER_HUMAN_X = 3.0   # a level shift must be 3x the breath size to overrule a breathing rhythm
DEBRIS_MM = 1.0             # minimum level shift (mm, when calibrated) for "settling debris"

SOURCE_TEXT = {
    "no_signal": "no breathing-like motion here (noise only) -> nothing to dig for from this scan",
    "debris_shift": "rubble shifting or settling (a step, no rhythm) -> not a breathing person; watch for collapse",
    "small_animal": "fast, tiny breathing -> possibly a small animal; a person cannot be ruled out, re-check",
    "machinery": "pure tone with harmonics -> machinery or generator vibration, not a person",
    "rotor": "fast broadband flicker -> drone rotor reflection, not a person",
    "moving_person": "large bursts of movement or knocking -> a person may be moving; breathing can't be read while they move, so listen and call out",
}


def source_of(v: dict, context: dict | None = None):
    """Is this return human breathing? Returns (source, conf, reason). Transparent rules, thresholds above.

    ``context`` (from ``identify``): {"dsp_candidate": the DSP's CFAR found a breathing-band candidate here,
    "flat": complex-bin PSD density 2-9 Hz / 0.1-0.75 Hz, "hf_excess_db": bin power 1.5-9 Hz over the scan's
    thermal floor, "dsp_nonhuman", "dsp_moving"}. Without context (a bare extract) the return is assumed to
    be a DSP candidate and the rotor rule is skipped."""
    sg, ctx = v.get("signature") or {}, context or {}
    calib = sg.get("units") == "mm"
    detected = ctx.get("dsp_candidate", True)
    # a 45-60 s window holds >= 3 breaths even at 4/min; with a pause, 2 breaths around it are enough
    min_breaths = 2 if v.get("apnea_events") or (v.get("duration_s") or 0) < 40 else 3
    human_like = detected and v.get("pattern") not in (None, "unclear") and (v.get("n_breaths") or 0) >= min_breaths
    nb = int(sg.get("n_bursts") or 0)
    if nb >= MOVING_MIN_BURSTS:
        return "moving_person", float(np.clip(nb / 8.0, 0.5, 0.9)), f"{nb} separate bursts of movement / knocking"
    if ctx.get("dsp_nonhuman") and not detected:
        return "machinery", 0.8, "the DSP flagged a strong periodic return outside the breathing band"
    lf, lfrac, ldb = sg.get("line_hz") or 0.0, sg.get("line_frac") or 0.0, sg.get("line_db") or 0.0
    if lf >= LINE_MIN_HZ and ldb >= LINE_DB_MIN and lfrac >= LINE_FRAC_MIN:
        return "machinery", float(np.clip(lfrac + 0.3, 0.5, 0.9)), \
            f"pure tone at {lf:.2f} Hz ({60 * lf:.0f}/min), {ldb:.0f} dB above the rest of the spectrum"
    flat, hf_db = ctx.get("flat"), ctx.get("hf_excess_db")
    if flat is not None and hf_db is not None and flat >= ROTOR_FLAT_MIN and hf_db >= ROTOR_HF_DB \
            and ctx.get("beyond_rubble", True):           # a rotor is above the pod, never inside the rubble volume
        return "rotor", float(np.clip(flat, 0.5, 0.9)), f"white, fast flicker {hf_db:.0f} dB over the noise floor (aliased blades)"
    cv, acv = v.get("cv"), v.get("amp_cv")
    if human_like and cv is not None and acv is not None and cv < TONE_CV_MAX and acv < TONE_AMPCV_MAX and lfrac >= 0.8:
        return "machinery", 0.5, f"in-band rhythm too exact for a body (variability {cv:.3f})"
    step = sg.get("step_mm") or 0.0
    breath = abs(sg.get("depth_pp_mm") or 0.0) if human_like else 0.0      # no rhythm: no breath size to beat
    debrisish = step >= DEBRIS_STEP_BREATH_X * max(breath, 3 * (sg.get("noise_mm") or 0.0)) and \
        (not calib or step >= DEBRIS_MM)
    # a real breathing rhythm is only overruled by a much larger level shift, and never when it has pauses
    # (a breath hold after a deep breath is itself a level shift: real Apnea recordings showed this)
    overrule = debrisish and step >= DEBRIS_OVER_HUMAN_X * breath and not v.get("apnea_events")
    if human_like and not overrule:
        d, r = v.get("depth_rel"), v.get("rate_bpm") or 0.0
        if d is not None and (v.get("depth_rel_conf") or 0) > 0 and d < ANIMAL_DEPTH_MAX and r >= ANIMAL_RATE_MIN:
            return "small_animal", 0.35, f"rhythm {r:.0f}/min but chest motion only {d:.2f}x an adult's (amplitude is unreliable)"
        return "human_breathing", float(v.get("pattern_conf") or 0.0), "breathing-like rhythm in the human range"
    if debrisish:
        return "debris_shift", float(np.clip(step / 5.0, 0.4, 0.85)), f"step / creep of {step:.1f} mm with no breathing rhythm"
    if ctx.get("dsp_moving"):
        return "moving_person", 0.5, "the DSP saw bulk motion (one-sided Doppler)"
    return "no_signal", float(np.clip(1.0 - (v.get("sqi") or 0.0) if detected else 0.8, 0.3, 0.9)), \
        "no breathing rhythm, tone, step or burst" + ("" if detected else " (no DSP detection)")


def apply_source(v: dict, src):
    """Write source fields; a non-human source turns condition to 'unknown' and the hint to 'reject' / 'urgent'."""
    source, conf, why = src
    v["source"], v["source_conf"], v["source_reason"] = source, round(float(conf), 2), why
    if source == "human_breathing":
        return v
    v["condition"], v["condition_text"] = "unknown", SOURCE_TEXT[source]
    if source == "moving_person":
        v["triage_hint"], v["triage_reason"] = "urgent", "movement or knocking: possibly a responsive person; listen and call out"
    else:
        v["triage_hint"], v["triage_reason"] = "reject", f"not human breathing ({why})"
    v["pattern_conf"] = round(float(conf), 2)
    return v


def motion_compensated(iq, params=None, cfg=None):
    """Range-slow-time cube Xc (motion-compensated, clutter NOT removed), exactly as radar_dsp.process_if builds it."""
    from . import radar_dsp as R
    P, cfg = params or R.RadarParams(), cfg or R.DspConfig()
    X = R.range_fft(iq)
    d_ref, _ = R._estimate_platform_motion(X, P, cfg)
    Xc = X * np.exp(-1j * 4 * np.pi * d_ref / P.wavelength)[:, None]
    A = np.column_stack([d_ref, np.linspace(-1, 1, X.shape[0])])
    coef, *_ = np.linalg.lstsq(A, Xc - Xc.mean(axis=0, keepdims=True), rcond=None)
    return Xc - A @ coef


def identify(iq, params=None, *, rubble_path_m="pod", cfg=None, res=None, Xc=None, look_m=(1.2, 10.0),
             look_near_m: float = 3.5) -> dict:
    """Whole-scan verdict: which return is it, and is it human breathing?

    Runs the real DSP chain (radar_dsp.process_if) unless ``res`` is given, picks the strongest breathing
    candidate (else the DSP's periodic / moving flags, else the strongest respiration-band bin), runs
    ``extract`` on it with the scan context, and returns
    {"source", "condition", "confidence", "reason", "range_m", "vitals", "dsp": {...}}.
    ``rubble_path_m="pod"``: the pod looks straight down through rubble, so the path = the range."""
    from . import radar_dsp as R
    P = params or R.RadarParams()
    res = res if res is not None else R.process_if(iq, P, cfg, keep_debug=True)
    Xc = Xc if Xc is not None else motion_compensated(iq, P, cfg)
    fs = P.fs_slow
    r0 = int(np.ceil(P.min_range / P.range_res))
    # thermal floor: median over range bins of the 5-9 Hz slow-time power (breathing and clutter live lower)
    Y = Xc - Xc.mean(axis=0, keepdims=True)
    S = np.abs(np.fft.fft(Y * np.hanning(Y.shape[0])[:, None], axis=0)) ** 2
    f = np.abs(np.fft.fftfreq(Y.shape[0], 1 / fs))
    hb = (f >= 1.5) & (f <= 0.45 * fs)
    floor_bin = np.median(S[(f >= 5.0) & (f <= 0.45 * fs)][:, r0:], axis=0)
    floor = float(np.median(floor_bin))
    lo_b, hi_b = int(np.ceil(look_m[0] / P.range_res)), int(np.floor(look_m[1] / P.range_res))
    cands = [c for c in res.candidates if look_m[0] <= c["range_m"] <= look_m[1]]
    nonh = [n for n in res.periodic_nonhuman if look_m[0] <= n["range_m"] <= look_m[1]]
    if cands:
        c = cands[0]
        k, rng_m, snr = int(c["bin"]), float(c["range_m"]), float(c["snr_db"])
    elif nonh:
        k = int(round(nonh[0]["range_m"] / P.range_res)); rng_m, snr = nonh[0]["range_m"], None
    elif res.doppler_moving and res.moving and look_m[0] <= res.moving["range_m"] <= look_m[1]:
        k = int(round(res.moving["range_m"] / P.range_res)); rng_m, snr = res.moving["range_m"], None
    else:
        # no detection: (1) a rotor-like bin anywhere in the look window, else (2) in the near (rubble) window the
        # reflector with the largest modulation index var / |mean|^2 (motion beyond the common rubble jitter)
        W = np.arange(lo_b, hi_b + 1)
        rb_ = (f >= 0.1) & (f <= 0.75)
        flat_w = S[hb][:, W].mean(axis=0) / np.maximum(S[rb_][:, W].mean(axis=0), 1e-30)
        hfdb_w = 10 * np.log10(np.maximum(S[hb][:, W].mean(axis=0), 1e-30) / max(floor, 1e-30))
        p_mean = np.abs(Xc[:, W].mean(axis=0)) ** 2
        noise_var = floor / float((np.hanning(Y.shape[0]) ** 2).sum())          # thermal variance per sample
        has_refl = p_mean >= REFLECTOR_DB_MIN_LIN * noise_var
        # a rotor sits above the pod (beyond the rubble look volume): search for it only past look_near_m
        rot = (flat_w >= ROTOR_FLAT_MIN) & (hfdb_w >= ROTOR_HF_DB) & has_refl & (W * P.range_res > look_near_m)
        near = has_refl & (W * P.range_res <= look_near_m)
        if rot.any():
            k = int(W[rot][np.argmax(hfdb_w[rot])])
        elif near.any():
            mi = np.maximum(np.var(Y[:, W], axis=0) - noise_var, 0.0) / np.maximum(p_mean, 1e-30)
            k = int(W[near][np.argmax(mi[near])])
        else:
            k = lo_b + int(np.argmax(np.var(Y[:, W], axis=0)))
        rng_m, snr = float(k * P.range_res), None
    hf_db = float(10 * np.log10(max(S[hb, k].mean(), 1e-30) / max(floor, 1e-30)))
    rb = (f >= 0.1) & (f <= 0.75)
    flat = float(S[hb, k].mean() / max(S[rb, k].mean(), 1e-30))
    ctx = {"hf_excess_db": hf_db, "flat": flat, "dsp_candidate": bool(cands), "dsp_nonhuman": bool(nonh),
           "beyond_rubble": bool(rng_m > look_near_m),
           "dsp_moving": bool(res.doppler_moving)}
    path = rng_m if rubble_path_m == "pod" else rubble_path_m
    v = extract(Xc[:, k], fs, kind="iq", wavelength=P.wavelength, range_m=rng_m, rubble_path_m=path,
                quality=res.quality, snr_db=snr, params=P, context=ctx)
    return {"source": v["source"], "condition": v["condition"], "confidence": v["source_conf"]
            if v["source"] != "human_breathing" else v["pattern_conf"], "reason": v["source_reason"]
            if v["source"] != "human_breathing" else v["pattern_reason"], "range_m": round(float(rng_m), 2), "bin": k,
            "vitals": v, "dsp": {"candidates": len(res.candidates), "nonhuman": len(res.periodic_nonhuman),
                                 "moving": bool(res.doppler_moving), "quality": round(float(res.quality), 2),
                                 "hf_excess_db": round(hf_db, 1), "flat": round(flat, 3)}}


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
