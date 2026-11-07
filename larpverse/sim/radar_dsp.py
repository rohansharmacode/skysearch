"""Radar signal-processing chain: SYNTHETIC raw-signal generator + a standard open DSP chain.

*** PART 1 (``simulate_if`` and the Scene classes) IS A SYNTHETIC SIGNAL GENERATOR. ***
It fabricates the complex intermediate-frequency (IF) samples a low-frequency UWB / FMCW radar
would deliver after dechirping: a 0.4-2.0 GHz sweep (1.6 GHz bandwidth -> 9.4 cm range resolution;
the sub-1 GHz part is what penetrates concrete), 512 complex fast-time samples per sweep (unambiguous
range 48 m), 20 Hz slow-time frame rate and a 20 s dwell.  The scene it renders contains
  * strong static clutter (rubble layers, ground, antenna leakage),
  * a breathing chest (4-8 mm at 0.2-0.33 Hz, with slight rate / amplitude irregularity),
  * two-way attenuation through rubble: frequency-dependent concrete loss (~10 dB/m below 1 GHz,
    ~20 dB/m above, FHWA rule of thumb) x solid fraction, power-averaged over the sweep
    (``two_way_loss_db``; preset ``literature``) or the old flat 0.7 dB/m (preset ``legacy``),
  * thermal noise,
  * platform-motion artefacts scaled by (1 - quality): slow drift + band-limited jitter,
  * optionally a walking person (bulk range migration -> doppler_moving) and a periodic
    non-human source (e.g. a 1.2 Hz fan / machinery vibration).
Real through-rubble propagation is NOT simulated (scattering, rebar, moisture, layering are folded into
a per-metre loss along the heightmap path); the slides must say so. docs/RADAR_PENETRATION.md.  The generator is told where its reflectors are by the caller (sim/radar_model.py,
the only module allowed to read hidden truth).  This file never imports the world.

*** PART 2 (``process_if`` and friends) IS THE REAL DSP, KNOWN TO THE PLANNER SIDE. ***
It sees only the IF cube and the radar parameters:
  range FFT (Hann) -> platform-motion estimate from a strong static bin and common-mode
  compensation -> static-clutter removal (slow-time mean) -> respiration-band power per range
  bin -> OS-CFAR across range -> per-candidate sub-bin range refinement -> phase / displacement
  extraction (PCA linear demodulation or DC-compensated arctangent + unwrap) -> Butterworth
  band-pass 0.1-0.5 Hz (sosfiltfilt) -> spectrum (zero-padded FFT, Welch for long records) ->
  peak frequency, band-power ratio, peakiness -> confidence.  Bulk motion is detected from short
  range-Doppler segments and must migrate in range; periodic energy at 0.7-3 Hz is flagged as
  "periodic but out of band" (not human respiration) and never reported as a candidate.

The same extractor (``extract_displacement`` + ``estimate_breathing``) is reused by
scripts/validate_real_breathing.py on real 24 GHz CW recordings.

Only numpy + scipy.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy import signal

C0 = 299_792_458.0


# =============================================================================== parameters
@dataclass
class RadarParams:
    f_start: float = 0.4e9          # sweep start (Hz): low band so part of the sweep penetrates concrete
    f_stop: float = 2.0e9           # sweep stop (Hz)
    n_fast: int = 512               # complex IF samples per sweep (= range bins)
    fs_slow: float = 20.0           # frame (slow-time) rate, Hz
    dwell: float = 20.0             # s
    # --- processing
    resp_band: tuple = (0.1, 0.5)   # Hz, human respiration
    oob_band: tuple = (0.7, 3.0)    # Hz, periodic-but-not-respiration region
    min_range: float = 1.0          # m, ignore antenna leakage / ground bounce
    nfft_slow: int = 2048           # zero padded slow-time FFT

    @property
    def bandwidth(self):
        return self.f_stop - self.f_start

    @property
    def f_c(self):
        return 0.5 * (self.f_start + self.f_stop)

    @property
    def wavelength(self):
        return C0 / self.f_c

    @property
    def range_res(self):
        return C0 / (2.0 * self.bandwidth)          # 9.4 cm for 1.6 GHz

    @property
    def max_range(self):
        return self.n_fast * self.range_res         # complex IF: N bins

    @property
    def n_slow(self):
        return int(round(self.dwell * self.fs_slow))

    @property
    def range_axis(self):
        return np.arange(self.n_fast) * self.range_res

    @property
    def t_slow(self):
        return np.arange(self.n_slow) / self.fs_slow


# =============================================================================== generator (SYNTHETIC)
# Per-sample amplitude of a 0 dB reflector at 1 m (unit complex noise). With A90_DB below this puts the
# reach directly beneath a good scan (2 m slant) at ~3.0 m of rubble (docs/RADAR_PENETRATION.md).
AMP_1M = 70.0
CHEST_RCS_DB = 0.0
PERSON_RCS_DB = 8.0
CLUTTER_RCS_DB = (12.0, 28.0)

# ---- through-rubble propagation (SYNTHETIC physics; sources and numbers in docs/RADAR_PENETRATION.md)
CONCRETE_DB_PER_M_LOW = 10.0     # one-way loss in solid concrete below 1 GHz (FHWA GPR rule of thumb)
CONCRETE_DB_PER_M_HIGH = 20.0    # ... and above ~2 GHz; linear in between
SOLID_FRACTION = 0.65            # share of a slant path through a collapse pile that is solid (rest: voids)
REBAR_DB_PER_M = 0.0             # extra one-way loss per metre for dense rebar mesh (0: plain / mixed rubble)
PERCH_ROUGH_FACTOR = 0.6         # adaptive legs settle on a slab: effective roughness of a perch
LEVEL_DEG = 12.0                 # self-levelling radar mount: perch tilt up to this does not tilt the beam
REACH_BENEATH_M = 3.0            # design point: rubble a good scan sees through at 2 m slant range
REACH_REF_RANGE_M = 2.0
LEGACY_DB_PER_M = 0.7            # pre-review model: one flat one-way loss for any rubble (optimistic)

# Penetration presets. "literature": frequency-dependent concrete loss x solid fraction, ~3 m reach beneath
# a good scan, < 1 m at a 11 m stand-off, 10 m of rubble is hopeless. "legacy": the original flat 0.7 dB/m
# (kept so the pre-review seed-7 demo can still be reproduced; it lets the radar see through 10+ m).
PENETRATION_MODELS = ("literature", "legacy")
_PEN = {"model": "legacy"}       # process default; GridMission / CLI select explicitly


def use_penetration(model: str):
    """Select the propagation preset for this process (generators, planner, coverage use the same one)."""
    if model not in PENETRATION_MODELS:
        raise ValueError(f"penetration model {model!r}; choose {PENETRATION_MODELS}")
    _PEN["model"] = model


def penetration_model() -> str:
    return _PEN["model"]


def concrete_db_per_m(f_hz):
    """One-way attenuation of solid concrete (dB/m) vs frequency: ~10 dB/m below 1 GHz, ~20 dB/m at 2 GHz+."""
    f = np.asarray(f_hz, float) / 1e9
    return np.clip(CONCRETE_DB_PER_M_LOW + (CONCRETE_DB_PER_M_HIGH - CONCRETE_DB_PER_M_LOW) * (f - 1.0),
                   CONCRETE_DB_PER_M_LOW, CONCRETE_DB_PER_M_HIGH)


def two_way_loss_db(rubble_path_m, params: RadarParams | None = None):
    """Two-way rubble loss (dB) of the swept band through ``rubble_path_m`` metres of pile.

    Power is averaged over the sweep, so deep returns are carried by the low (sub-1 GHz) part of the band:
    loss(L) = -10 log10( mean_f 10^(-2 L (SOLID_FRACTION alpha_c(f) + REBAR) / 10) ).
    """
    P = params or RadarParams()
    L = np.atleast_1d(np.asarray(rubble_path_m, float))
    if _PEN["model"] == "legacy":
        out = 2.0 * LEGACY_DB_PER_M * L
        return out if np.ndim(rubble_path_m) else float(out[0])
    f = np.linspace(P.f_start, P.f_stop, 33)
    a = SOLID_FRACTION * concrete_db_per_m(f) + REBAR_DB_PER_M                     # (F,) one-way dB/m
    lin = 10 ** (-2.0 * L[:, None] * a[None, :] / 10.0)
    out = -10 * np.log10(lin.mean(axis=1))
    return out if np.ndim(rubble_path_m) else float(out[0])


def reflection_amplitude(range_m, rubble_path_m=0.0, rcs_db=0.0, params: RadarParams | None = None):
    """SYNTHETIC radar-equation amplitude: 1/R^2 spreading (amplitude) and the two-way rubble loss."""
    r = max(float(range_m), 1.0)
    loss_db = two_way_loss_db(float(rubble_path_m), params) if rubble_path_m else 0.0
    return AMP_1M * 10 ** ((rcs_db - loss_db) / 20.0) / r ** 2


# detection threshold of process_if: chest amplitude (dB re unit noise per IF sample) at which ~90% of scans
# give a strong candidate, per landing quality (measured by scripts in docs/RADAR_PENETRATION.md)
A90_DB = {0.95: -17.0, 0.6: -15.0, 0.3: -11.0}   # 12 scans per 3 dB step at 6 m, seeds 0-11


def detect_threshold_db(quality):
    qs = sorted(k for k, v in A90_DB.items() if v is not None)
    if not qs:
        return 20 * np.log10(AMP_1M / REACH_REF_RANGE_M ** 2) - two_way_loss_db(REACH_BENEATH_M)
    return float(np.interp(quality, qs, [A90_DB[q] for q in qs]))


def reach_path_m(range_m, quality=0.9, params: RadarParams | None = None):
    """Rubble path (m) through which a breathing chest at ``range_m`` still clears the detection threshold
    in ~90% of scans of the given quality (0 when even free space is out of reach). Vectorised over range.
    Not ground truth: a property of the radar and the propagation model (planner, coverage map, scans)."""
    r = np.maximum(np.asarray(range_m, float), 1.0)
    margin = 20 * np.log10(AMP_1M / r ** 2) + CHEST_RCS_DB - detect_threshold_db(quality)
    Ls = np.linspace(0.0, 8.0 if _PEN["model"] == "literature" else 60.0, 241)
    out = np.where(margin > 0, np.interp(margin, two_way_loss_db(Ls, params), Ls), 0.0)
    return out if np.ndim(range_m) else float(out)


def reach_prob(range_m, path_m, quality=0.9, params: RadarParams | None = None, soft_m=0.25):
    """Probability the breathing return clears the threshold: logistic in (reach - path), 90% at the reach."""
    x = reach_path_m(range_m, quality, params) - np.asarray(path_m, float)
    out = 1.0 / (1.0 + np.exp(np.clip(-x / soft_m, -50, 50)) / 9.0)
    return out if np.ndim(out) else float(out)


@dataclass
class Breather:
    range_m: float
    amp: float
    rate_hz: float = 0.25
    chest_mm: float = 6.0
    irregularity: float = 0.03      # fractional wander of rate / amplitude


@dataclass
class Mover:
    """Walking person: ``range_track`` is the range (m) at every slow-time frame."""
    range_track: np.ndarray
    amp: float


@dataclass
class Vibrator:
    """Periodic non-human source (fan, generator): a fixed-range reflector vibrating at freq_hz."""
    range_m: float
    amp: float
    freq_hz: float = 1.2
    disp_mm: float = 3.0


@dataclass
class Scene:
    clutter: list = field(default_factory=list)      # [(range_m, amp)]
    breathers: list = field(default_factory=list)
    movers: list = field(default_factory=list)
    vibrators: list = field(default_factory=list)
    quality: float = 0.9                              # landing quality: scales platform motion
    noise_sigma: float = 1.0                          # complex noise std per IF sample
    tilt_std: float = 0.5                             # fraction of platform motion NOT common to all ranges
    diff_jitter_mm: float = 4.0                       # independent per-reflector wobble (mm rms) at quality 0


def default_clutter(rng, max_range=22.0, leakage=True):
    """SYNTHETIC rubble layers + ground bounce + antenna leakage (strong static returns)."""
    cl = []
    if leakage:
        cl.append((0.30, 4.0e3))                      # antenna leakage / skid
        cl.append((0.55 + 0.1 * rng.uniform(), 2.0e3))  # ground bounce
    n = int(rng.integers(3, 7))
    for r in np.sort(rng.uniform(1.0, max_range, n)):
        rcs = rng.uniform(*CLUTTER_RCS_DB)
        cl.append((float(r), reflection_amplitude(r, 0.0, rcs)))
    return cl


def platform_motion(quality, params: RadarParams, rng):
    """SYNTHETIC platform displacement (m): slow drift + band-limited jitter, scaled by (1 - quality)."""
    s = float(np.clip(1.0 - quality, 0.0, 1.0))
    t = params.t_slow
    m = t.size
    drift = np.zeros(m)
    for _ in range(2):
        drift += 4.0e-3 * s * np.sin(2 * np.pi * rng.uniform(0.01, 0.04) * t + rng.uniform(0, 2 * np.pi))
    drift += 3.0e-3 * s * rng.normal() * (t / params.dwell)
    w = rng.normal(size=m)
    spec = np.fft.rfft(w)
    f = np.fft.rfftfreq(m, 1 / params.fs_slow)
    spec[(f < 0.08) | (f > 2.5)] = 0.0                 # touchdown wobble, rotor spin-down, wind
    jit = np.fft.irfft(spec, m)
    jit *= 5.0e-3 * s / max(jit.std(), 1e-12)
    return drift + jit


def _diff_jitter(scene, params, rng):
    """SYNTHETIC per-reflector displacement not common to the platform (rubble shifting, antenna wobble)."""
    s = float(np.clip(1.0 - scene.quality, 0.0, 1.0))
    m = params.n_slow
    if s <= 0:
        return np.zeros(m)
    spec = np.fft.rfft(rng.normal(size=m))
    f = np.fft.rfftfreq(m, 1 / params.fs_slow)
    spec[(f < 0.08) | (f > 2.5)] = 0.0
    j = np.fft.irfft(spec, m)
    return 1e-3 * scene.diff_jitter_mm * s * j / max(j.std(), 1e-12)


def _breath_displacement(b: Breather, params: RadarParams, rng):
    t = params.t_slow
    wander = np.cumsum(rng.normal(size=t.size))
    wander = wander / max(np.abs(wander).max(), 1e-9)                   # random walk in [-1, 1]
    inst_f = b.rate_hz * (1.0 + b.irregularity * wander)
    phase = 2 * np.pi * np.cumsum(inst_f) / params.fs_slow + rng.uniform(0, 2 * np.pi)
    am = 1.0 + 0.5 * b.irregularity * np.convolve(wander[::-1], np.ones(20) / 20, "same")
    # a hint of the 2nd harmonic (chest wall is not a pure sinusoid)
    return 1e-3 * b.chest_mm * am * (np.sin(phase) + 0.1 * np.sin(2 * phase + 0.7))


def simulate_if(scene: Scene, params: RadarParams, rng) -> np.ndarray:
    """SYNTHETIC IF cube, complex (n_slow, n_fast).  Slow time on axis 0, fast time on axis 1."""
    m, n = params.n_slow, params.n_fast
    t_idx = np.arange(n)
    lam = params.wavelength
    dp = platform_motion(scene.quality, params, rng)
    x = np.zeros((m, n), complex)

    def add(r_t, amp, tilt=True, phi0=None):
        r_t = np.asarray(r_t, float)
        g = 1.0 + (scene.tilt_std * rng.normal() if tilt else 0.0)
        r_t = r_t + g * dp + _diff_jitter(scene, params, rng)
        if r_t.max() >= params.max_range - 0.5 or r_t.min() < 0.0:
            return
        phi = (rng.uniform(0, 2 * np.pi) if phi0 is None else phi0) + 4 * np.pi * r_t / lam
        x[:] += amp * np.exp(1j * phi)[:, None] * np.exp(2j * np.pi * (r_t / params.range_res)[:, None] * t_idx[None, :] / n)

    for r, a in scene.clutter:
        add(np.full(m, r), a)
    for b in scene.breathers:
        add(b.range_m + _breath_displacement(b, params, rng), b.amp)
    for v in scene.vibrators:
        d = 1e-3 * v.disp_mm * np.sin(2 * np.pi * v.freq_hz * params.t_slow + rng.uniform(0, 2 * np.pi))
        add(v.range_m + d, v.amp)
    for mv in scene.movers:
        add(mv.range_track, mv.amp, tilt=False)
    x += scene.noise_sigma / np.sqrt(2) * (rng.normal(size=(m, n)) + 1j * rng.normal(size=(m, n)))
    return x


# =============================================================================== DSP helpers
def range_fft(iq: np.ndarray) -> np.ndarray:
    """Hann-windowed range FFT along fast time -> (n_slow, n_range) complex."""
    w = np.hanning(iq.shape[1] + 2)[1:-1]
    return np.fft.fft(iq * w[None, :], axis=1)


def _parabolic(y, i):
    """Sub-sample peak offset in (-0.5, 0.5) from three samples around index i."""
    if i <= 0 or i >= len(y) - 1:
        return 0.0
    a, b, c = y[i - 1], y[i], y[i + 1]
    den = a - 2 * b + c
    return 0.0 if abs(den) < 1e-12 else float(np.clip(0.5 * (a - c) / den, -0.5, 0.5))


def os_cfar_floor(p: np.ndarray, n_train: int = 16, n_guard: int = 2, quantile: float = 0.75) -> np.ndarray:
    """Ordered-statistic CFAR noise-floor estimate for every cell of a 1-D power profile."""
    half = n_train + n_guard
    pad = np.pad(p, half, mode="edge")
    win = np.lib.stride_tricks.sliding_window_view(pad, 2 * half + 1)
    mask = np.ones(2 * half + 1, bool)
    mask[n_train: n_train + 2 * n_guard + 1] = False
    return np.quantile(win[:, mask], quantile, axis=1)


def circle_fit_center(x: np.ndarray) -> complex:
    """Algebraic (Kasa) circle fit of I/Q samples: centre = static clutter vector."""
    a = np.column_stack([2 * x.real, 2 * x.imag, np.ones(x.size)])
    b = np.abs(x) ** 2
    sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    return complex(sol[0], sol[1])


def extract_displacement(x: np.ndarray, method: str = "pca") -> np.ndarray:
    """Displacement-proportional waveform from one complex slow-time bin (radians-like units).

    "pca"    : subtract the static vector (mean), project on the principal I/Q axis (small-angle
               linear demodulation; robust to a static clutter vector in the same bin).
    "arctan" : subtract the circle-fit centre, then unwrap(angle(.)) (arctangent demodulation, the
               standard for CW radar with a DC offset; used for real 24 GHz recordings).
    """
    x = np.asarray(x, complex)
    if method == "arctan":
        y = x - circle_fit_center(x)
        return np.unwrap(np.angle(y))
    y = x - x.mean()
    cov = np.cov(np.vstack([y.real, y.imag]))
    w, v = np.linalg.eigh(cov)
    u = v[:, -1]
    proj = y.real * u[0] + y.imag * u[1]
    # positive sign convention: align with the unwrapped phase of the mean-removed signal slope
    if np.corrcoef(proj, np.unwrap(np.angle(x))[:proj.size])[0, 1] < 0:
        proj = -proj
    return proj / max(np.abs(x.mean()), 1e-12)


def bandpass(x: np.ndarray, fs: float, band=(0.1, 0.5), order: int = 4) -> np.ndarray:
    sos = signal.butter(order, band, btype="bandpass", fs=fs, output="sos")
    return signal.sosfiltfilt(sos, x)


def spectrum(x: np.ndarray, fs: float, nfft: int = 4096, welch_min_s: float = 90.0):
    """One-sided power spectrum.  Windowed zero-padded periodogram; Welch for long records."""
    x = np.asarray(x, float) - np.mean(x)
    if x.size / fs >= welch_min_s:
        seg = int(60 * fs)
        f, p = signal.welch(x, fs, nperseg=seg, noverlap=seg // 2, nfft=max(nfft, 2 * seg))
        return f, p
    w = np.hanning(x.size)
    sp = np.abs(np.fft.rfft(x * w, nfft)) ** 2 / max((w ** 2).sum(), 1e-12)
    return np.fft.rfftfreq(nfft, 1 / fs), sp


def estimate_breathing(disp: np.ndarray, fs: float, band=(0.1, 0.5), oob=(0.7, 3.0), peak_halfwidth=0.08):
    """Band-pass -> spectrum -> peak.  Returns dict(rate_hz, peakiness, band_ratio, filtered, f, p)."""
    hp = bandpass(disp, fs, band)
    f, p = spectrum(hp, fs)
    inb = (f >= band[0]) & (f <= band[1])
    if not inb.any() or p[inb].sum() <= 0:
        return {"rate_hz": None, "peakiness": 0.0, "band_ratio": 0.0, "filtered": hp, "f": f, "p": p}
    ib = np.flatnonzero(inb)
    k = ib[np.argmax(p[ib])]
    off = _parabolic(np.log(p + 1e-30), k)
    rate = float(f[k] + off * (f[1] - f[0]))
    near = inb & (np.abs(f - rate) <= peak_halfwidth)
    peakiness = float(p[near].sum() / p[inb].sum())
    # band-power ratio on the *unfiltered* (detrended) displacement: respiration vs everything 0.1-3 Hz
    f2, p2 = spectrum(signal.detrend(disp, type="linear"), fs)
    tot = (f2 >= band[0]) & (f2 <= oob[1])
    ratio = float(p2[(f2 >= band[0]) & (f2 <= band[1])].sum() / max(p2[tot].sum(), 1e-30))
    return {"rate_hz": rate, "peakiness": peakiness, "band_ratio": ratio, "filtered": hp, "f": f, "p": p}


# =============================================================================== processing chain
@dataclass
class DspConfig:
    cfar_alpha: float = 4.0          # threshold = alpha * floor (respiration band power)
    cfar_alpha_oob: float = 6.0
    n_train: int = 16
    n_guard: int = 3
    max_candidates: int = 4
    min_conf: float = 0.2
    phase_method: str = "pca"
    motion_comp: bool = True
    motion_rms_ref_mm: float = 7.0   # estimated platform motion rms (mm) that maps to quality ~ 0
    seg_len: int = 32                # range-Doppler segment (frames) for bulk-motion detection
    seg_hop: int = 16
    dop_hi_hz: float = 2.0           # "bulk motion" = Doppler above this
    dop_alpha: float = 8.0
    min_track_segs: int = 5
    persist_frac: float = 0.4
    track_gap_m: float = 1.2
    min_speed_mps: float = 0.15
    max_track_resid_m: float = 0.5


@dataclass
class ScanResult:
    range_candidates: list
    confidence: list
    quality: float
    breathing_hz: float | None
    doppler_moving: bool
    moving: dict | None                  # {"range_m", "confidence", "speed_mps"}
    periodic_nonhuman: list              # [{"range_m", "freq_hz", "snr_db"}]
    candidates: list                     # richer per-candidate dicts (range, conf, hz, snr, peakiness, ratio)
    debug: dict = field(default_factory=dict)

    def observations(self, scan_id: str, drone_id: str, sensor_pose, timestamp: float):
        """Observation dicts exactly matching the radar schema in contracts/INTERFACES.md.

        [stationary observation] plus, if bulk motion was found, a separate ``doppler_moving``
        observation (same convention as sim/radar_model.py).
        """
        pose = [round(float(v), 2) for v in sensor_pose]
        out = [{
            "scan_id": scan_id, "drone_id": drone_id, "sensor_pose": pose,
            "timestamp": round(float(timestamp), 1),
            "range_candidates": [round(float(r), 2) for r in self.range_candidates],
            "confidence": [round(float(c), 2) for c in self.confidence],
            "quality": round(float(self.quality), 2),
            "breathing_hz": None if self.breathing_hz is None else round(float(self.breathing_hz), 3),
            "doppler_moving": False,
            # rubble this scan can see through directly beneath (2 m slant), from its measured quality
            "max_reliable_depth_m": round(reach_path_m(REACH_REF_RANGE_M, float(self.quality)), 2),
        }]
        if self.doppler_moving and self.moving:
            out.append({
                "scan_id": f"{scan_id}m", "drone_id": drone_id, "sensor_pose": pose,
                "timestamp": out[0]["timestamp"],
                "range_candidates": [round(float(self.moving["range_m"]), 2)],
                "confidence": [round(float(self.moving["confidence"]), 2)],
                "quality": out[0]["quality"], "breathing_hz": None, "doppler_moving": True,
            })
        return out


def _estimate_platform_motion(X, params, cfg):
    """Common-mode platform displacement (m) from the strongest static bin beyond min_range."""
    r0 = int(np.ceil(params.min_range / params.range_res))
    mag = np.abs(X[:, r0:]).mean(axis=0)
    ref = r0 + int(np.argmax(mag))
    ph = np.unwrap(np.angle(X[:, ref]))
    d = ph * params.wavelength / (4 * np.pi)
    return d - d.mean(), ref


def _motion_quality(d, params, cfg):
    hp = bandpass(d, params.fs_slow, (0.05, 3.0), order=2)
    rms_mm = 1e3 * float(np.std(hp))
    q = 1.0 - rms_mm / cfg.motion_rms_ref_mm
    q *= min(1.0, (1 - np.exp(-params.dwell / 8.0)) / (1 - np.exp(-20.0 / 8.0)))
    return float(np.clip(q, 0.03, 0.97)), rms_mm


def _detect_motion(X, params, cfg):
    """Bulk-motion (walking person) detection from short range-Doppler segments + range-migration track."""
    m, nr = X.shape
    L, hop = cfg.seg_len, cfg.seg_hop
    starts = list(range(0, m - L + 1, hop))
    w = np.hanning(L)[None, :, None]
    segs = np.stack([X[s:s + L] for s in starts])                  # (S, L, R)
    segs = segs - segs.mean(axis=1, keepdims=True)
    S = np.abs(np.fft.fft(segs * w, axis=1)) ** 2
    f = np.fft.fftfreq(L, 1 / params.fs_slow)
    H = S[:, np.abs(f) >= cfg.dop_hi_hz, :].sum(axis=1)            # (S, R) high-Doppler energy
    r0 = int(np.ceil(params.min_range / params.range_res))
    flags = np.zeros_like(H, bool)
    snr = np.zeros_like(H)
    for si in range(H.shape[0]):
        fl = os_cfar_floor(H[si], cfg.n_train, cfg.n_guard + 4)
        fl = np.maximum(fl, np.median(H[si]))
        snr[si] = H[si] / np.maximum(fl, 1e-12)
        flags[si] = snr[si] > cfg.dop_alpha
    flags[:, :r0] = False
    persist = flags.mean(axis=0) >= cfg.persist_frac               # fixed-range = platform jitter / vibration
    flags[:, persist] = False
    # per-segment strongest remaining cell
    det = []
    for si in range(H.shape[0]):
        if flags[si].any():
            idx = np.flatnonzero(flags[si])
            k = idx[np.argmax(snr[si, idx])]
            lo, hi = max(k - 1, 0), min(k + 2, nr)
            r = float((params.range_axis[lo:hi] * H[si, lo:hi]).sum() / H[si, lo:hi].sum())
            det.append((si, r, float(snr[si, k])))
    # longest chain of range-continuous detections (allow 1-segment gaps)
    best = []
    for i0 in range(len(det)):
        chain = [det[i0]]
        for d in det[i0 + 1:]:
            if d[0] - chain[-1][0] <= 2 and abs(d[1] - chain[-1][1]) <= cfg.track_gap_m:
                chain.append(d)
        if len(chain) > len(best):
            best = chain
    dbg = {"H": H, "flags": flags, "seg_times": (np.array(starts) + L / 2) / params.fs_slow, "track": best}
    if len(best) < cfg.min_track_segs:
        return None, dbg
    tt = np.array([(starts[d[0]] + L / 2) / params.fs_slow for d in best])
    rr = np.array([d[1] for d in best])
    slope, icpt = np.polyfit(tt, rr, 1)
    resid = rr - (slope * tt + icpt)
    # bulk motion = coherent range migration; platform jitter / vibration stays at fixed ranges
    if abs(slope) < cfg.min_speed_mps or np.std(resid) > cfg.max_track_resid_m:
        return None, dbg
    speed = float(slope)
    last = best[-3:]
    t_last = float(np.mean([(starts[d[0]] + L / 2) / params.fs_slow for d in last]))
    rng_end = float(np.median([d[1] for d in last])) + slope * (params.dwell - t_last)   # extrapolate to scan end
    conf = float(np.clip((np.median([d[2] for d in best]) - cfg.dop_alpha) / 25.0 + 0.5, 0.4, 0.9))
    return {"range_m": rng_end, "confidence": conf, "speed_mps": speed}, dbg


def process_if(iq: np.ndarray, params: RadarParams | None = None, cfg: DspConfig | None = None,
               keep_debug: bool = True) -> ScanResult:
    """Run the full chain on an IF cube (n_slow, n_fast).  Never sees ground truth."""
    params = params or RadarParams()
    cfg = cfg or DspConfig()
    fs = params.fs_slow
    X = range_fft(iq)
    m, nr = X.shape
    ra = params.range_axis
    r0 = int(np.ceil(params.min_range / params.range_res))

    # --- platform motion: estimate from strongest static bin, quality, common-mode compensation
    d_ref, ref_bin = _estimate_platform_motion(X, params, cfg)
    quality, motion_rms_mm = _motion_quality(d_ref, params, cfg)
    Xc = X
    if cfg.motion_comp:
        Xc = X * np.exp(-1j * 4 * np.pi * d_ref / params.wavelength)[:, None]

    # --- decorrelate each range bin from the platform motion (tilt: the part of the motion
    #     the common-mode rotation does not remove), complex least squares on [d_ref, linear trend]
    if cfg.motion_comp:
        tl = np.linspace(-1, 1, m)
        A = np.column_stack([d_ref, tl])
        Z = Xc - Xc.mean(axis=0, keepdims=True)
        coef, *_ = np.linalg.lstsq(A, Z, rcond=None)
        Xc = Xc - A @ coef

    # --- bulk motion (before clutter removal windows, on the compensated cube)
    moving, mdbg = _detect_motion(Xc, params, cfg)

    # --- static clutter removal + respiration-band / out-of-band power per range bin
    Y = Xc - Xc.mean(axis=0, keepdims=True)
    w = np.hanning(m)[:, None]
    S = np.abs(np.fft.fft(Y * w, params.nfft_slow, axis=0)) ** 2 / (w ** 2).sum()
    f = np.fft.fftfreq(params.nfft_slow, 1 / fs)
    af = np.abs(f)
    df = fs / params.nfft_slow
    inb = (af >= params.resp_band[0]) & (af <= params.resp_band[1])
    ooB = (af >= params.oob_band[0]) & (af <= params.oob_band[1])
    p_resp = S[inb].sum(axis=0) * df
    p_oob = S[ooB].sum(axis=0) * df
    p_resp[:r0] = 0.0
    p_oob[:r0] = 0.0
    floor = np.maximum(os_cfar_floor(p_resp, cfg.n_train, cfg.n_guard), np.median(p_resp[r0:]))
    floor_o = np.maximum(os_cfar_floor(p_oob, cfg.n_train, cfg.n_guard), np.median(p_oob[r0:]))
    snr_lin = p_resp / np.maximum(floor, 1e-12)
    snr_oob = p_oob / np.maximum(floor_o, 1e-12)

    # --- candidate peaks
    det = snr_lin > cfg.cfar_alpha
    peaks = [k for k in np.flatnonzero(det)
             if snr_lin[k] >= snr_lin[max(k - 3, 0):k + 4].max() and k >= r0]
    cands = []
    for k in peaks:
        lo, hi = max(k - 1, 0), min(k + 2, nr)
        r_ref = float(ra[k] + _parabolic(np.log(p_resp + 1e-30), k) * params.range_res)
        x_bin = Xc[:, k]
        disp = extract_displacement(x_bin, cfg.phase_method)
        est = estimate_breathing(disp, fs, params.resp_band, params.oob_band)
        if est["rate_hz"] is None:
            continue
        snr_db = 10 * np.log10(max(snr_lin[k], 1e-9))
        # breathing = phase modulation -> symmetric +/- Doppler sidebands; a slowly walking person is one-sided
        pp = S[(f > 0) & inb, k].sum()
        pn = S[(f < 0) & inb, k].sum()
        asym = float(abs(pp - pn) / max(pp + pn, 1e-30))
        xr = X[:, k]
        static_frac = float(abs(xr.mean()) ** 2 / max(np.mean(np.abs(xr) ** 2), 1e-30))   # ~1 for clutter, low for a walker
        c_snr = 1.0 / (1.0 + np.exp(-(snr_db - 9.0) / 3.0))
        c_pk = float(np.clip((est["peakiness"] - 0.35) / 0.4, 0.0, 1.0))
        c_rt = float(np.clip((est["band_ratio"] - 0.25) / 0.5, 0.0, 1.0))
        conf = float(np.clip(c_snr * (0.25 + 0.75 * c_pk) * (0.15 + 0.85 * c_rt), 0.0, 0.98))
        edge = est["rate_hz"] < params.resp_band[0] + 0.02 or est["rate_hz"] > params.resp_band[1] - 0.02
        if edge:
            conf *= 0.5
        conf *= float(np.clip(1.0 - (asym - 0.25) / 0.35, 0.0, 1.0))      # one-sided Doppler = bulk motion
        cands.append({"bin": int(k), "range_m": r_ref, "confidence": conf, "breathing_hz": est["rate_hz"],
                      "snr_db": float(snr_db), "asym": asym, "static_frac": static_frac, "peakiness": est["peakiness"], "band_ratio": est["band_ratio"],
                      "_disp": disp, "_filt": est["filtered"], "_f": est["f"], "_p": est["p"]})
    if moving is None:                  # slow walker: strong, one-sided Doppler inside the respiration band
        slow = [c for c in cands if c["asym"] > 0.5 and c["snr_db"] > 30.0 and c["static_frac"] < 0.3]
        if slow:
            moving = {"range_m": slow[0]["range_m"], "confidence": 0.5, "speed_mps": float("nan")}
    cands = [c for c in cands if c["confidence"] >= cfg.min_conf]
    cands.sort(key=lambda c: -c["confidence"])
    cands = cands[:cfg.max_candidates]

    # --- periodic but out of band (fans, machinery): flag, never report as respiration
    nonhuman = []
    for k in np.flatnonzero(snr_oob > cfg.cfar_alpha_oob):
        if k < r0 or snr_oob[k] < snr_oob[max(k - 3, 0):k + 4].max():
            continue
        disp = extract_displacement(Xc[:, k], cfg.phase_method)
        f2, p2 = spectrum(signal.detrend(disp), fs)
        band = (f2 >= params.oob_band[0]) & (f2 <= params.oob_band[1])
        kk = np.flatnonzero(band)[np.argmax(p2[band])]
        peaky = p2[band & (np.abs(f2 - f2[kk]) <= 0.15)].sum() / max(p2[band].sum(), 1e-30)
        if peaky > 0.5 and p2[band].sum() > 2 * p2[(f2 >= params.resp_band[0]) & (f2 <= params.resp_band[1])].sum():
            nonhuman.append({"range_m": float(ra[k]), "freq_hz": float(f2[kk]),
                             "snr_db": float(10 * np.log10(snr_oob[k]))})
    # a strong respiration candidate in a bin that is mostly out-of-band energy is a distractor, not a person
    nh_bins = {int(round(n["range_m"] / params.range_res)) for n in nonhuman}
    cands = [c for c in cands if not any(abs(c["bin"] - b) <= 2 for b in nh_bins)]

    top_hz = cands[0]["breathing_hz"] if cands else None
    dbg = {}
    if keep_debug:
        k0 = cands[0]["bin"] if cands else (r0 + int(np.argmax(p_resp[r0:])))
        disp0 = extract_displacement(Xc[:, k0], cfg.phase_method)
        est0 = estimate_breathing(disp0, fs, params.resp_band, params.oob_band)
        dbg = {"range_axis": ra, "t": params.t_slow,
               "range_profile_db": 20 * np.log10(np.abs(X).mean(axis=0) + 1e-12),
               "range_slow_map_db": 20 * np.log10(np.abs(Y) + 1e-12),
               "p_resp": p_resp, "p_oob": p_oob, "threshold_resp": cfg.cfar_alpha * floor,
               "snr_resp_db": 10 * np.log10(np.maximum(snr_lin, 1e-9)),
               "platform_motion_mm": 1e3 * d_ref, "ref_bin": ref_bin, "motion_rms_mm": motion_rms_mm,
               "bin": k0, "slow_time_phase": disp0, "breathing_waveform": est0["filtered"],
               "spec_f": est0["f"], "spec_p": est0["p"], "spec_peak_hz": est0["rate_hz"],
               "motion": mdbg}
    for c in cands:
        for key in ("_disp", "_filt", "_f", "_p"):
            c.pop(key, None)
    return ScanResult(
        range_candidates=[c["range_m"] for c in cands], confidence=[c["confidence"] for c in cands],
        quality=quality, breathing_hz=top_hz, doppler_moving=moving is not None, moving=moving,
        periodic_nonhuman=nonhuman, candidates=cands, debug=dbg)


def radar_observations(iq, scan_id, drone_id, sensor_pose, timestamp, params=None, cfg=None, debug=False):
    """IF cube -> list of radar observation dicts (INTERFACES.md schema); optionally also the debug arrays."""
    res = process_if(iq, params, cfg, keep_debug=debug)
    obs = res.observations(scan_id, drone_id, sensor_pose, timestamp)
    return (obs, res) if debug else obs
