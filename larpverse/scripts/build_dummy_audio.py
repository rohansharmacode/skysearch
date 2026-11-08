"""Build the victim-dummy audio library for the Isaac video (assets/audio/dummies/).

    .venv-audio/bin/python scripts/build_dummy_audio.py            # build every clip + manifest.json
    scripts/heavy.sh .venv-audio/bin/python scripts/build_dummy_audio.py check   # seed-7 listen through acoustic/

No downloads: sources are eSpeak NG (system libespeak-ng) for en/zh/id speech, the Piper vi_VN-vivos voice in
acoustic/models (CC BY-NC-SA 4.0; the other Piper voices on disk are not used, see VOICE_LICENCES), the teammate's
CC0 recordings (data/audio_real/frances), ESC-50 clips (CC BY-NC 3.0) and numpy/scipy synthesis.
Also writes assets/audio/dummies/ATTRIBUTION.md (authors, licences, credit line for the video).
Output: 48 kHz mono 16-bit WAV, loudness-matched (gated RMS, see ``gated_rms_db``), peak <= -1 dBFS.
Every clip gets "through rubble" variants at 0.5/1.0/1.5/2.0 m (``rubble``: an ESTIMATED
frequency-dependent loss, not a measurement). docs/DUMMY_AUDIO.md explains the library and how Isaac uses it.

``check`` builds a seed-7 Scene from these clips (positions from sim/scenario truth: this harness is an
observation generator, like acoustic/scene.build_scene) and runs the unchanged acoustic pipeline
(acoustic.run.analyse, deterministic agent, no LLM, no Gemini). Results -> assets/audio/dummies/pipeline_check.json.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
import wave
import zlib

import numpy as np
from scipy import signal as sps

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "assets", "audio", "dummies")
FS = 48000
DEPTHS = (0.5, 1.0, 1.5, 2.0)
PEAK_CEIL_DB = -1.0
MODELS = os.path.join(ROOT, "acoustic", "models")
FRANCES = os.path.join(ROOT, "data", "audio_real", "frances")
ESC = os.path.join(ROOT, "data", "audio_real", "esc50")
# ---- TTS voices. Licences checked 2026-11-08 against the rhasspy/piper-voices MODEL_CARD files on Hugging Face
# (the .onnx.json configs on disk carry no licence) and the dataset licence pages:
#   en_US-lessac   : Blizzard 2013 Lessac data, licence "Research Purposes only", commercial use and voice-synthesis
#                    development excluded (cstr.ed.ac.uk/projects/blizzard/2013/lessac_blizzard2013/license.html) -> NOT USED
#   id_ID-news_tts : card says "Finetuned from U.S. English lessac voice", dataset link points at a Malayalam corpus,
#                    licence "See URL" -> inherits the lessac research-only terms and is unclear -> NOT USED
#   zh_CN-huayan   : card licence "Unknown" (github.com/PlayVoice/HuaYan_TTS) -> NOT USED
#   vi_VN-vivos    : VIVOS corpus (AILAB, VNU-HCM University of Science), CC BY-NC-SA 4.0 -> USED, non-commercial,
#                    share-alike: every clip rendered from it is CC BY-NC-SA 4.0
# en / zh / id are rendered with eSpeak NG (system libespeak-ng 1.52, Debian package; formant synthesis, no
# training data). The program is GPL-3.0-or-later; audio it generates is program output, not a copy of the program.
PIPER_VI = os.path.join(MODELS, "piper_multi", "vi_VN-vivos-x_low.onnx")
ESPEAK_VOICE = {"en": "en-us", "zh": "cmn", "id": "id"}
TTS_ENGINE = {"en": "espeak", "zh": "espeak", "id": "espeak", "vi": "piper"}
VOICE_LICENCES = {
    "espeak-ng": {"used_for": ["en", "zh-TW", "id"], "licence": "eSpeak NG 1.52 (GPL-3.0-or-later program); generated "
                  "audio carries no licence restriction", "non_commercial": False,
                  "attribution": "eSpeak NG (github.com/espeak-ng/espeak-ng)"},
    "piper vi_VN-vivos-x_low": {"used_for": ["vi"], "licence": "CC BY-NC-SA 4.0 (training data: VIVOS corpus)",
                                "non_commercial": True,
                                "attribution": "Piper vi_VN-vivos-x_low voice (rhasspy/piper-voices), trained on the VIVOS "
                                               "corpus, AILAB, University of Science, VNU-HCM (ailab.hcmus.edu.vn/vivos), "
                                               "CC BY-NC-SA 4.0"},
}
EXCLUDED_VOICES = {
    "piper en_US-lessac-low": "Blizzard 2013 Lessac data: research purposes only (commercial use and voice-synthesis "
                              "development excluded); a public demo video is not clearly research",
    "piper id_ID-news_tts-medium": "fine-tuned from the lessac voice (inherits its research-only terms); its card's "
                                   "dataset link points at a Malayalam corpus with licence 'See URL'",
    "piper zh_CN-huayan-x_low": "model card licence 'Unknown'",
}
ESC_LIC = "CC BY-NC 3.0 (non-commercial; hackathon demo only)"
# Per-clip Freesound credits, checked on freesound.org 2026-11-08 (ESC-50's meta/esc50.csv has ids only).
ESC_FREESOUND = {"1-101336-A-30": ("Robinhood76", "01801 knocking.wav", "CC BY-NC 4.0"),
                 "2-108017-A-24": ("ErocShadow", "Cough.wav", "CC Sampling+ 1.0"),
                 "3-151080-A-20": ("PickleJones", "01 baby crying hard and long.wav", "CC0 1.0")}
# Frances-set uploaders as shown on Freesound 2026-11-08 (all CC0); differs from her manifest for one clip.
FREESOUND_UPLOADER = {"voice_04.wav": "Feed_"}
ESC_CITE = "ESC-50, K. J. Piczak, ACM Multimedia 2015 (github.com/karolpiczak/ESC-50)"

# Target loudness per kind: gated RMS in dBFS for the DRY clip ("as heard 1 m from the dummy").
# Weak voices are deliberately 12 dB below normal voices. Suggested playback SPL at 1 m follows
# sim/callout.py LEVEL_DB (shout 90, knock 88, weak voice 72, crying 85, bark 90, debris 80).
TARGET_DB = {"voice": -20.0, "weak_voice": -32.0, "knock": -24.0, "groan": -26.0, "cough": -22.0,
             "cry": -22.0, "dog": -20.0, "generator": -22.0, "settling": -24.0, "wind": -26.0, "siren": -24.0}
SPL_1M = {"voice": 90, "weak_voice": 72, "knock": 88, "groan": 72, "cough": 80, "cry": 85, "dog": 90,
          "generator": 85, "settling": 80, "wind": 60, "siren": 75}
HUMAN = {"voice", "weak_voice", "knock", "groan", "cough", "cry"}
KINDS = ("voice", "weak_voice", "knock", "groan", "cough", "cry", "dog", "generator", "settling", "wind", "siren")


# ============================================================================ io + levels
def load(path):
    import soundfile as sf
    x, fs = sf.read(path, dtype="float64", always_2d=False)
    if x.ndim > 1:
        x = x.mean(axis=1)
    return x, fs


def to48(x, fs):
    if fs == FS:
        return np.asarray(x, float)
    g = np.gcd(int(fs), FS)
    return sps.resample_poly(x, FS // g, int(fs) // g)


def write_wav(path, x, fs=FS):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    pcm = np.clip(np.round(x * 32767.0), -32768, 32767).astype("<i2")
    tmp = path + ".tmp"
    with wave.open(tmp, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(fs)
        w.writeframes(pcm.tobytes())
    os.replace(tmp, path)


def frame_db(x, fs, win=0.05):
    n = max(1, int(win * fs))
    m = len(x) // n
    if m == 0:
        return np.array([10 * np.log10(np.mean(x ** 2) + 1e-20)])
    p = np.mean(x[: m * n].reshape(m, n) ** 2, axis=1)
    return 10 * np.log10(p + 1e-20)


def gated_rms_db(x, fs=FS):
    """Loudness used for matching: RMS over 50 ms frames within 30 dB of the loudest frame and above
    -70 dBFS (a simple gated RMS, not ITU-R BS.1770 LUFS)."""
    d = frame_db(x, fs)
    keep = (d > d.max() - 30) & (d > -70)
    if not keep.any():
        return float(d.max())
    return float(10 * np.log10(np.mean(10 ** (d[keep] / 10))))


def peak_db(x):
    return float(20 * np.log10(np.max(np.abs(x)) + 1e-12))


def limiter(x, ceil_db=PEAK_CEIL_DB, fs=FS):
    """Peak limiter: per-sample gain = min(1, ceiling/|x|), spread over +-3 ms so it never under-limits."""
    c = 10 ** (ceil_db / 20)
    g = np.minimum(1.0, c / (np.abs(x) + 1e-12))
    w = max(1, int(0.003 * fs))
    from scipy.ndimage import minimum_filter1d, uniform_filter1d
    g = minimum_filter1d(g, 2 * w + 1)
    g = uniform_filter1d(g, 2 * w + 1)
    g = minimum_filter1d(g, 2 * w + 1)
    y = x * g
    return np.clip(y, -c, c)


def fade(x, fs=FS, t_in=0.01, t_out=0.03):
    x = x.copy()
    a, b = int(t_in * fs), int(t_out * fs)
    if a:
        x[:a] *= np.linspace(0, 1, a)
    if b:
        x[-b:] *= np.linspace(1, 0, b)
    return x


def normalise(x, kind, fs=FS):
    x = x - np.mean(x)
    x = x * 10 ** ((TARGET_DB[kind] - gated_rms_db(x, fs)) / 20)
    return fade(limiter(x, fs=fs), fs)


def trim(x, fs, thr_db=-45.0, pad=0.12):
    d = frame_db(x, fs, 0.02)
    idx = np.where(d > d.max() + thr_db)[0]
    if len(idx) == 0:
        return x
    n = int(0.02 * fs)
    s = max(0, idx[0] * n - int(pad * fs))
    e = min(len(x), (idx[-1] + 1) * n + int(pad * fs))
    return x[s:e]


def best_window(x, fs, max_s):
    """The max_s-long window with the most active energy, snapped to quiet points (no cut words)."""
    if len(x) <= max_s * fs:
        return x
    L = int(max_s * fs)
    e = np.convolve(x ** 2, np.ones(int(0.05 * fs)), "same")
    c = np.concatenate([[0], np.cumsum(x ** 2)])
    s = int(np.argmax(c[L:] - c[:-L]))
    quiet = e < np.percentile(e, 20)

    def snap(i, lo, hi):
        cand = np.where(quiet[lo:hi])[0]
        return lo + int(cand[len(cand) // 2]) if len(cand) else i
    s2 = snap(s, max(0, s - int(0.4 * fs)), s + 1)
    e2 = snap(s + L, max(s2 + 1, s + L - int(0.6 * fs)), min(len(x), s + L))
    return x[s2:e2]


# ============================================================================ filters
def biquad_peak(f0, gain_db, q, fs=FS):
    A = 10 ** (gain_db / 40)
    w = 2 * np.pi * f0 / fs
    al = np.sin(w) / (2 * q)
    b = np.array([1 + al * A, -2 * np.cos(w), 1 - al * A])
    a = np.array([1 + al / A, -2 * np.cos(w), 1 - al / A])
    return b / a[0], a / a[0]


def bandpass(x, lo, hi, fs=FS, order=2):
    sos = sps.butter(order, [lo / (fs / 2), hi / (fs / 2)], "band", output="sos")
    return sps.sosfilt(sos, x)


def lowpass(x, fc, fs=FS, order=2):
    return sps.sosfilt(sps.butter(order, fc / (fs / 2), output="sos"), x)


def highpass(x, fc, fs=FS, order=2):
    return sps.sosfilt(sps.butter(order, fc / (fs / 2), "high", output="sos"), x)


def rubble_loss_db(f, depth):
    """ESTIMATE, not a measurement: loss through a depth (m) of collapsed concrete rubble.
    A(f, d) = d * (3 + 6 * f/kHz) dB, capped at 70 dB. The 3 dB/m broadband term is the value already
    used by acoustic/scene.py and sim/callout.py (RUBBLE_DB_PER_M); the frequency slope models the low-pass
    behaviour of porous, granular rubble (viscous/thermal losses rising with frequency, as in porous-ground
    propagation models, e.g. Attenborough et al.) and is a tuning guess chosen so speech stays recognisable
    at 0.5 m and is a dull low murmur at 2 m. Replace with measured data if any becomes available."""
    return np.minimum(depth * (3.0 + 6.0 * np.asarray(f) / 1000.0), 70.0)


def rubble(x, depth, seed=0, fs=FS):
    """Through-rubble version: FIR with the estimated loss curve, plus a small void reverb (RT60 0.22 s,
    ~1 m^3 air pocket, also an estimate). Level is NOT renormalised: the loss is the point."""
    if depth <= 0:
        return x
    f = np.linspace(0, fs / 2, 512)
    h = sps.firwin2(1023, f / (fs / 2), 10 ** (-rubble_loss_db(f, depth) / 20))
    y = sps.fftconvolve(x, h)[511: 511 + len(x)]
    rng = np.random.default_rng(seed)
    rt60, n = 0.22, int(0.35 * fs)
    t = np.arange(n) / fs
    ir = rng.normal(0, 1, n) * np.exp(-6.91 * t / rt60)
    ir = lowpass(ir, 1800.0 / (1 + 0.5 * depth))
    ir[: int(0.002 * fs)] = 0.0
    ir /= np.sqrt(np.sum(ir ** 2)) + 1e-12
    wet = sps.fftconvolve(y, ir)[: len(y) + int(0.25 * fs)]
    y = np.pad(y, (0, len(wet) - len(y)))
    wet *= 10 ** (-8 / 20) * np.sqrt(np.sum(y ** 2) / (np.sum(wet ** 2) + 1e-20))
    out = y + wet                                       # direct-to-reverberant +8 dB by energy (estimate)
    return fade(limiter(out), fs, 0.002, 0.08)


# ============================================================================ voices
_piper = {}
_espeak = {}


def espeak_tts(voice, text, rate=160, pitch=50):
    """eSpeak NG through the system libespeak-ng (ctypes, synchronous mode). Returns (float array, sample rate)."""
    import ctypes
    if not _espeak:
        lib = ctypes.CDLL("libespeak-ng.so.1")
        cbt = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.POINTER(ctypes.c_short), ctypes.c_int, ctypes.c_void_p)
        buf = []

        def cb(wav, n, ev):
            if n > 0:
                buf.append(np.ctypeslib.as_array(wav, shape=(n,)).copy())
            return 0
        _espeak.update(lib=lib, cb=cbt(cb), buf=buf, sr=lib.espeak_Initialize(2, 0, None, 0))
        lib.espeak_SetSynthCallback(_espeak["cb"])
    lib, buf = _espeak["lib"], _espeak["buf"]
    buf.clear()
    if lib.espeak_SetVoiceByName(voice.encode()) != 0:
        raise RuntimeError(f"espeak-ng voice {voice} missing")
    lib.espeak_SetParameter(1, int(rate), 0)            # espeakRATE (words per minute)
    lib.espeak_SetParameter(3, int(pitch), 0)           # espeakPITCH (0-100)
    b = text.encode("utf-8") + b"\0"
    lib.espeak_Synth(b, len(b), 0, 1, 0, 1, None, None)  # POS_CHARACTER, espeakCHARS_UTF8
    lib.espeak_Synchronize()
    return np.concatenate(buf).astype(float) / 32768.0, _espeak["sr"]


def tts(lang, text, slow=False):
    """Calm TTS at 48 kHz: eSpeak NG for en/zh/id, Piper vivos (speaker 12) for vi. slow=True for weak voices."""
    if TTS_ENGINE[lang] == "espeak":
        x, sr = espeak_tts(ESPEAK_VOICE[lang], text, rate=115 if slow else 165, pitch=45 if slow else 55)
    else:
        from piper import PiperVoice, SynthesisConfig
        v = _piper.get(lang) or _piper.setdefault(lang, PiperVoice.load(PIPER_VI))
        cfg = SynthesisConfig(speaker_id=12, length_scale=1.35 if slow else 0.95, noise_scale=0.6, noise_w_scale=0.7)
        ch = list(v.synthesize(text, cfg))
        x, sr = np.concatenate([c.audio_float_array for c in ch]).astype(float), ch[0].sample_rate
    return to48(trim(x, sr), sr)


def shout_fx(x, fs=FS):
    """Approximate a raised voice from calm TTS: +10 % pitch (resampling), presence boost at 2.5 kHz,
    soft saturation. It is still TTS, not a real shout; the manifest says so."""
    y = sps.resample_poly(x, 10, 11)                  # played at fs: 10 % higher, 10 % faster
    b, a = biquad_peak(2500, 5.0, 0.9, fs)
    y = sps.lfilter(b, a, y)
    y = y / (np.max(np.abs(y)) + 1e-9)
    return np.tanh(2.2 * y) / np.tanh(2.2)


def varispeed(x, r0=1.0, r1=0.92):
    """Time-varying playback rate (pitch and tempo fall from r0 to r1 across the clip)."""
    n = len(x)
    rate = np.linspace(r0, r1, n)
    pos = np.concatenate([[0], np.cumsum(rate)[:-1]])
    pos = pos[pos < n - 1]
    return np.interp(pos, np.arange(n), x)


def weak_fx(fragments, rng, fs=FS):
    """Weak, fading voice: fragments separated by long pauses with faint breaths, energy falling 6 dB
    across the clip, pitch falling 6 %, breathy noise mixed under the voice, high harmonics softened."""
    parts = [np.zeros(int(0.3 * fs))]
    K = len(fragments)
    for k, frag in enumerate(fragments):
        frag = frag / (np.sqrt(np.mean(frag ** 2)) + 1e-12)
        parts.append(frag * 10 ** (-6.0 * k / max(1, K - 1) / 20))
        if k < K - 1:
            gap = np.zeros(int(rng.uniform(1.5, 2.4) * fs))
            L = int(0.55 * fs)                          # an audible inhale in the pause
            s = int(rng.uniform(0.3, 0.6) * fs)
            if s + L < len(gap):
                br = bandpass(rng.normal(0, 1, L), 500, 2600, fs) * np.hanning(L)
                gap[s: s + L] += 0.10 * br / (np.sqrt(np.mean(br ** 2)) + 1e-12)
            parts.append(gap)
    parts.append(np.zeros(int(0.4 * fs)))
    x = varispeed(np.concatenate(parts), 1.0, 0.94)
    env = np.convolve(np.abs(x), np.ones(int(0.02 * fs)) / int(0.02 * fs), "same")
    air = bandpass(rng.normal(0, 1, len(x)), 1000, 5500, fs)
    air *= env / (np.sqrt(np.mean(env ** 2)) + 1e-12)
    voiced = lowpass(x, 4000, fs)
    return voiced + 0.2 * air * np.sqrt(np.mean(voiced ** 2)) / (np.sqrt(np.mean(air ** 2)) + 1e-12)


# ============================================================================ knocks
def pipe_modes(L=1.5, D=0.0603, wall=0.0039, E=200e9, rho=7850.0):
    """Bending modes of a free-free steel pipe (Euler-Bernoulli beam): f_n = (b_n L)^2 / (2 pi L^2) *
    sqrt(EI / (rho A)), b_n L = 4.730, 7.853, 10.996, 14.137, 17.279. Default: DN50 water pipe, 1.5 m."""
    d = D - 2 * wall
    I = np.pi / 64 * (D ** 4 - d ** 4)
    A = np.pi / 4 * (D ** 2 - d ** 2)
    bl = np.array([4.730, 7.853, 10.996, 14.137, 17.279])
    return bl ** 2 / (2 * np.pi * L ** 2) * np.sqrt(E * I / (rho * A))


def slab_modes(a=3.0, b=2.4, h=0.15, E=30e9, nu=0.2, rho=2400.0, nmax=7):
    """Simply supported Kirchhoff plate: f_mn = pi/2 * sqrt(D / (rho h)) * ((m/a)^2 + (n/b)^2),
    D = E h^3 / (12 (1 - nu^2)). Default: 0.15 m concrete slab fragment 3.0 x 2.4 m."""
    D = E * h ** 3 / (12 * (1 - nu ** 2))
    k = np.pi / 2 * np.sqrt(D / (rho * h))
    return [(m, n, k * ((m / a) ** 2 + (n / b) ** 2)) for m in range(1, nmax + 1) for n in range(1, nmax + 1)]


def contact_weight(f, tc):
    """Spectrum of a half-sine contact force of duration tc (flat, then rolling off above ~1.5/tc)."""
    return 1.0 / np.sqrt(1 + (np.asarray(f) * tc / 0.75) ** 4)


def hit_pipe(rng, fs=FS, dur=0.9):
    t = np.arange(int(dur * fs)) / fs
    tc = 0.0006                                     # stone / spanner on steel: ~0.6 ms contact
    eta = 0.04                                      # loss factor: pipe clamped and loaded by rubble (assumed;
                                                    # a free steel pipe is ~0.001-0.01 and rings like a bell)
    x0 = rng.uniform(0.2, 0.45)                     # strike point along the pipe (fraction of length)
    y = np.zeros_like(t)
    for i, f in enumerate(pipe_modes()):
        shape = abs(np.cos((i + 1.5) * np.pi * x0))  # rough free-free mode shape at the strike point
        for df in (1.0, 1.004):                     # slightly split pair -> natural beating
            tau = 1.0 / (np.pi * f * df * eta)
            y += (contact_weight(f, tc) * (0.4 + shape) / (1 + 0.15 * i) * np.exp(-t / tau)
                  * np.sin(2 * np.pi * f * df * t + rng.uniform(0, 2 * np.pi)))
    y /= np.max(np.abs(y)) + 1e-12
    n = int(0.02 * fs)                               # contact noise of stone on steel
    y[:n] += 1.2 * bandpass(rng.normal(0, 1, n), 300, 4000, fs) * np.exp(-np.arange(n) / (0.003 * fs))
    return y


def hit_slab(rng, fs=FS, dur=0.35):
    t = np.arange(int(dur * fs)) / fs
    tc = 0.0012                                     # stone / fist-held rock on concrete (assumed)
    eta = 0.2                                       # cracked slab chunk wedged in rubble: heavily damped (assumed;
                                                    # intact concrete is ~0.01-0.02 and rings, which is not a knock)
    a, b = 3.0, 2.4
    x0, y0 = rng.uniform(0.2, 0.8) * a, rng.uniform(0.2, 0.8) * b
    y = np.zeros_like(t)
    for m, n_, f in slab_modes(a, b):
        if f > 4000:
            continue
        shape = np.sin(m * np.pi * x0 / a) * np.sin(n_ * np.pi * y0 / b)
        tau = 1.0 / (np.pi * f * eta)
        y += contact_weight(f, tc) * shape / np.sqrt(f / 70.0) * np.exp(-t / tau) * \
            np.sin(2 * np.pi * f * t + rng.uniform(0, 2 * np.pi))
    y = 0.6 * y / (np.max(np.abs(y)) + 1e-12)
    n = int(0.03 * fs)                               # local contact thud: the dominant part of a concrete knock
    thud = bandpass(rng.normal(0, 1, n), 150, 1800, fs) * np.exp(-np.arange(n) / (0.006 * fs))
    y[:n] += 2.5 * thud / (np.max(np.abs(thud)) + 1e-12)
    return y


def knock_pattern(hit, rng, groups=3, gap=0.30, period=2.6, fs=FS):
    """Groups of three knocks (the call-out asks for three), ~0.3 s apart, a group every ~2.6 s."""
    tail = 1.0
    n = int((0.15 + (groups - 1) * period + 2 * gap + tail) * fs)
    x = np.zeros(n)
    g0 = 0.15
    for g in range(groups):
        for k in range(3):
            s = int((g0 + k * gap + rng.normal(0, 0.012)) * fs)
            h = hit(rng) * rng.uniform(0.85, 1.0)
            e = min(n, s + len(h))
            x[s:e] += h[: e - s]
        g0 += period + rng.uniform(-0.2, 0.2)
    return x


# ============================================================================ distractors
def generator(rng, mains=60.0, dur=12.0, fs=FS):
    """4-stroke 4-cylinder genset at synchronous speed (mains 60 Hz in Taiwan: 1800 rpm, firing 60 Hz;
    50 Hz: 1500 rpm). Firing-order harmonics, crank-order imbalance, combustion bursts at the firing rate,
    alternator magnetostriction at 2 x mains, low exhaust rumble. Seamless loop (0.5 s crossfade)."""
    xf = 0.5
    n = int((dur + xf) * fs)
    t = np.arange(n) / fs
    f0 = mains * (1 + 0.003 * np.cumsum(rng.normal(0, 1, n)) / np.sqrt(n))
    ph = 2 * np.pi * np.cumsum(f0) / fs
    x = sum(np.sin(k * ph + rng.uniform(0, 6.3)) / k ** 0.8 for k in range(1, 40))
    x += 0.2 * np.sin(0.5 * ph + 1.0)                              # crank order (half the firing rate)
    x = x / np.sqrt(np.mean(x ** 2))
    cycle = np.floor(ph / (2 * np.pi)).astype(int)                 # one burst per firing, with 30 %
    amp = 1 + 0.3 * rng.normal(0, 1, cycle.max() + 1)              # cycle-to-cycle combustion variation
    pulses = np.maximum(0, np.sin(ph)) ** 4 * amp[cycle]
    comb = bandpass(rng.normal(0, 1, n), 300, 3500, fs) * pulses
    hum = np.sin(2 * ph * 1.0 + 0.3)                               # 2 x mains (magnetostriction, approx.)
    exh = lowpass(rng.normal(0, 1, n), 180, fs)
    y = (x + 0.25 * comb / np.sqrt(np.mean(comb ** 2)) + 0.25 * hum
         + 0.3 * exh / np.sqrt(np.mean(exh ** 2)))
    m = int(xf * fs)                                               # crossfade tail into head -> loops cleanly
    w = np.linspace(0, 1, m)
    out = y[: n - m].copy()
    out[:m] = out[:m] * w + y[n - m:] * (1 - w)
    return out


def settling(rng, fs=FS, dur=10.0):
    """Rubble settling: stick-slip creak, a crack with a thump, gravel trickle, low rumble, a second trickle."""
    n = int(dur * fs)
    x = np.zeros(n)

    def add(sig, t0):
        s = int(t0 * fs)
        e = min(n, s + len(sig))
        x[s:e] += sig[: e - s]
    L = int(1.3 * fs)                                              # creak: stick-slip pulse train
    tt = np.arange(L) / fs
    rate = 120 - 70 * tt / tt[-1]
    pulses = np.zeros(L)
    pulses[np.where(np.diff(np.floor(np.cumsum(rate) / fs)) > 0)[0]] = 1.0
    creak = sum(sps.lfilter(*biquad_peak(f, 18, 12, fs), pulses) - pulses for f in (260, 640, 1180))
    add(0.5 * creak / (np.max(np.abs(creak)) + 1e-9) * np.hanning(L), 0.6)
    m = int(0.25 * fs)                                             # crack + thump
    crack = highpass(rng.normal(0, 1, m), 800, fs) * np.exp(-np.arange(m) / (0.01 * fs))
    thump = lowpass(rng.normal(0, 1, m), 120, fs) * np.exp(-np.arange(m) / (0.06 * fs))
    add(0.9 * crack / np.max(np.abs(crack)) + 1.2 * thump / np.max(np.abs(thump)), 2.4)
    R = int(2.6 * fs)                                              # rumble
    rum = lowpass(rng.normal(0, 1, R), 140, fs) * np.hanning(R)
    add(0.6 * rum / np.max(np.abs(rum)), 2.4)
    for t0, t1, peak in ((2.5, 6.2, 45.0), (7.3, 9.2, 18.0)):     # gravel trickles (Poisson ticks)
        t = t0
        while t < t1:
            u = (t - t0) / (t1 - t0)
            lam = peak * np.exp(-3 * u) * min(1, 8 * u + 0.1)
            t += rng.exponential(1 / max(lam, 1.0))
            k = int(rng.uniform(0.002, 0.008) * fs)
            big = rng.uniform() < 0.12
            tick = (lowpass if big else highpass)(rng.normal(0, 1, k), 400 if big else 1500, fs)
            tick *= np.exp(-np.arange(k) / (k / 4)) * rng.lognormal(-1.2, 0.6) * (1.5 if big else 1)
            add(tick, t)
    return x


def loopable(x, xf=0.5, fs=FS):
    """Crossfade the last xf seconds into the head so the clip loops without a click."""
    m = int(xf * fs)
    w = np.linspace(0, 1, m)
    out = x[:-m].copy()
    out[:m] = out[:m] * w + x[-m:] * (1 - w)
    return out


# ============================================================================ catalogue
FRANCES_META = {}


def frances(fname):
    if not FRANCES_META:
        with open(os.path.join(FRANCES, "manifest.csv")) as f:
            for r in csv.DictReader(f):
                FRANCES_META[r["file"]] = r
    r = FRANCES_META[fname]
    who = FREESOUND_UPLOADER.get(fname, r["creator"])
    note = f" (listed as {r['creator']} in the URL and the teammate manifest)" if who != r["creator"] else ""
    return {"source": f"teammate recording set (F. Valisalo), data/audio_real/frances/{fname}",
            "licence": "CC0 1.0", "non_commercial": False,
            "author": who, "attribution": f"\"{r['title']}\" by {who}{note} ({r['page']}), CC0",
            "origin_url": r["page"]}


ESC_META = {}


def esc(key):
    if not ESC_META:
        with open(os.path.join(ESC, "esc50.csv")) as f:
            for r in csv.DictReader(f):
                ESC_META[r["filename"][:-4]] = r
    r = ESC_META[key]
    who, title, fl = ESC_FREESOUND[key]
    return {"source": f"ESC-50 clip {key} ({r['category']}), data/audio_real/esc50/{key}_16k.wav",
            "licence": f"{ESC_LIC}; Freesound original: {fl}", "non_commercial": True, "author": who,
            "attribution": f"\"{title}\" by {who} (freesound.org/s/{r['src_file']}, {fl}), via {ESC_CITE}",
            "origin_url": f"https://freesound.org/s/{r['src_file']}"}


def synth(what):
    return {"source": f"procedural synthesis in scripts/build_dummy_audio.py ({what})", "licence": "CC0 1.0 (generated here)",
            "non_commercial": False, "author": "Larpverse team",
            "attribution": "Larpverse team (generated)", "origin_url": None}


def tts_src(lang, extra=""):
    if TTS_ENGINE[lang] == "espeak":
        v = VOICE_LICENCES["espeak-ng"]
        src = f"TTS: eSpeak NG voice '{ESPEAK_VOICE[lang]}' (formant synthesis, robotic)"
        if lang == "zh":
            src += "; Mandarin read from Traditional characters, placeholder until a native Taiwanese Mandarin recording"
    else:
        v = VOICE_LICENCES["piper vi_VN-vivos-x_low"]
        src = "TTS: Piper vi_VN-vivos-x_low, speaker 12"
    return {"source": src + (f"; {extra}" if extra else ""), "licence": v["licence"],
            "non_commercial": v["non_commercial"], "author": None, "attribution": v["attribution"], "origin_url": None}


# Multilingual phrases. zh: Mandarin with Taiwanese wording in Traditional characters (有沒有人, 腳, 啦).
# Texts are written for this demo and not yet checked by native speakers (same status as sim/guidance.REVIEW).
PHRASES = {
    "en": {"call": ("Help! Can anyone hear me? My leg is trapped!", None),
           "here": ("I'm here! I'm here! Over here!", None)},
    "zh": {"call": ("救命啊！有沒有人聽得到？我的腳被壓住了！", "Help! Can anyone hear me? My leg is pinned!"),
           "here": ("我在這裡！我在這裡啦！", "I'm here! I'm right here!")},
    "id": {"call": ("Tolong! Ada yang bisa dengar saya? Kaki saya terjepit!", "Help! Can anyone hear me? My leg is trapped!"),
           "here": ("Saya di sini! Di sini!", "I'm here! Over here!")},
    "vi": {"call": ("Cứu tôi với! Có ai nghe thấy tôi không? Chân tôi bị kẹt!", "Help me! Can anyone hear me? My leg is stuck!"),
           "here": ("Tôi ở đây! Ở đây!", "I'm here! Over here!")},
}
WEAK = {   # (fragments, English); en is the seed-7 B1 truth text (sim/scenario.TRANSCRIPTS["weak_voice"])
    "en": (["hello", "is anyone there", "water"], "hello... is anyone there... water..."),
    "zh": (["有人嗎", "我在這", "水"], "Anyone there... I'm here... water..."),
    "id": (["halo", "ada orang", "air"], "Hello... anyone... water..."),
    "vi": (["có ai không", "nước", "cho tôi nước"], "Anyone there... water... give me water..."),
}
LANG_TAG = {"en": "en", "zh": "zh-TW", "id": "id", "vi": "vi"}   # zh: Mandarin as spoken in Taiwan (cmn-Hant-TW)

TODO = [
    {"id": "voice_nan_call", "kind": "voice", "language": "nan-TW",
     "status": "TODO: needs native recording (Taiwanese Hokkien has no open TTS voice; not faked)",
     "draft_text": "救命喔！有人聽有無？我的跤予物件壓著矣！",
     "draft_romanisation": "Kiù-miā--ooh! Ū lâng thiann ū bô? Guá ê kha hōo mi̍h-kiānn teh--tio̍h--ah!",
     "transcript_en": "Help! Can anyone hear? My leg is pinned by something!",
     "note": "draft wording, unverified: a native speaker should correct it and record it"},
    {"id": "voice_nan_here", "kind": "voice", "language": "nan-TW",
     "status": "TODO: needs native recording", "draft_text": "我佇遮！我佇遮啦！",
     "draft_romanisation": "Guá tī tsia! Guá tī tsia--lah!", "transcript_en": "I'm here! I'm here!",
     "note": "draft wording, unverified"},
    {"id": "groan", "kind": "groan", "language": None,
     "status": "TODO: no groan/moan source on disk. ESC-50 has no groan class; YAMNet found no Groan or "
               "Wail/moan frame (max 0.26) in the DroneAudioSet crying/humansounds sources. Skipped, not faked.",
     "note": "record a CC0 groan, or allow one Freesound CC0 download"},
]


def schedule(mode, **kw):
    return {"mode": mode, **kw}


def build(out=OUT, verbose=True):
    rng = np.random.default_rng(7)
    clips = []

    def emit(cid, x, kind, meta, language=None, transcript=None, transcript_en=None, processing=(),
             sched=None, synthetic=False, rubble_variants=True, notes=None):
        x = normalise(x, kind)
        rel = f"{cid}.wav"
        write_wav(os.path.join(out, rel), x)
        base = {"id": cid, "file": rel, "kind": kind, "human": kind in HUMAN, "language": language,
                "transcript": transcript, "transcript_en": transcript_en if transcript_en else
                (transcript if language == "en" else None),
                **meta, "processing": list(processing), "synthetic": synthetic,
                "sample_rate": FS, "duration_s": round(len(x) / FS, 3), "rms_dbfs": round(gated_rms_db(x), 2),
                "peak_dbfs": round(peak_db(x), 2), "through_rubble_depth_m": 0.0, "parent": None,
                "suggested_spl_db_1m": SPL_1M[kind], "schedule": sched}
        if notes:
            base["notes"] = notes
        clips.append(base)
        if rubble_variants:
            for i, d in enumerate(DEPTHS):
                y = rubble(x, d, seed=zlib.crc32(cid.encode()) % 1000 + i)
                rr = f"rubble/{cid}__{d:.1f}m.wav"
                write_wav(os.path.join(out, rr), y)
                v = dict(base, id=f"{cid}__{d:.1f}m", file=rr, parent=cid, through_rubble_depth_m=d,
                         duration_s=round(len(y) / FS, 3), rms_dbfs=round(gated_rms_db(y), 2),
                         peak_dbfs=round(peak_db(y), 2),
                         processing=base["processing"] + [f"through rubble {d:.1f} m: estimated loss "
                                                          f"d*(3+6 f/kHz) dB + small-void reverb (RT60 0.22 s)"])
                clips.append(v)
        if verbose:
            print(f"  {cid:28s} {kind:10s} {len(x) / FS:5.2f} s  {gated_rms_db(x):6.1f} dBFS", flush=True)

    # ---- 1. real shouted calls (CC0), preferred over TTS
    real = [("voice_en_help_real_a", "voice_04.wav", "Help me!", 4.0),
            ("voice_en_help_real_b", "voice_07.wav", "Help! Help me!", 4.0),
            ("voice_en_help_real_c", "voice_08.wav", "Help me! (repeated)", 5.0),
            ("voice_en_please_real", "voice_06.wav", "Please help me, I'm so scared, please help me.", 6.0)]
    for cid, f, text, mx in real:
        x, fs = load(os.path.join(FRANCES, f))
        x = best_window(trim(x, fs), fs, mx)
        emit(cid, highpass(to48(x, fs), 80), "voice", frances(f), "en", text, None,
             ["trim / best window", "resample 16 -> 48 kHz (no content above 8 kHz)", "high-pass 80 Hz",
              "loudness match"], schedule("loop", period_s=9.0, jitter_s=2.0),
             notes="transcript from the Freesound title, not checked word by word")

    # ---- 1b. multilingual TTS calls (processed towards a raised voice)
    for lang, ph in PHRASES.items():
        for key, (text, en) in ph.items():
            x = shout_fx(tts(lang, text))
            emit(f"voice_{lang}_{key}_tts", x, "voice", tts_src(lang), LANG_TAG[lang], text, en or text,
                 [f"TTS ({TTS_ENGINE[lang]})", "raised-voice processing: +10 % pitch, +5 dB presence at 2.5 kHz, soft saturation",
                  "loudness match"],
                 schedule("loop", period_s=9.0, jitter_s=2.0) if key == "call" else
                 schedule("after_callout", latency_s=[1.5, 3.5], repeats=2, period_s=4.0),
                 synthetic=True)

    # ---- 2. weak / fading voices (processed)
    for lang, (frags, en) in WEAK.items():
        parts = [tts(lang, f, slow=True) for f in frags]
        x = weak_fx(parts, rng)
        text = ("……" if lang == "zh" else "... ").join(frags) + ("……" if lang == "zh" else "...")
        emit(f"weak_{lang}_tts", x, "weak_voice", tts_src(lang, "PROCESSED to a weak voice"), LANG_TAG[lang],
             text, en, [f"TTS ({TTS_ENGINE[lang]}), slow", "long pauses 1.5-2.4 s with faint inhales",
                        "energy decay -6 dB across the clip", "pitch/tempo decay -6 %", "breathy noise mix (-14 dB)",
                        "low-pass 4 kHz", "loudness match at -32 dBFS (12 dB under normal voice)"],
             schedule("loop", period_s=25.0, jitter_s=5.0), synthetic=True)
    x, fs = load(os.path.join(FRANCES, "voice_04.wav"))
    one = to48(trim(x, fs), fs)
    x = weak_fx([highpass(one, 80), 0.8 * highpass(one, 80)], rng)
    emit("weak_en_real_processed", x, "weak_voice", frances("voice_04.wav"), "en", "help me... help me...", None,
         ["real CC0 'help me' twice", "PROCESSED to a weak voice: pauses + inhales, energy and pitch decay, "
          "breathy noise, low-pass 4 kHz", "loudness match at -32 dBFS"],
         schedule("loop", period_s=25.0, jitter_s=5.0))

    # ---- 3. knocks. Each synthesised clip has its own seeded RNG, so a TTS change never changes it. The
    # concrete seed (1) was picked among 8: its 3-knock pattern keeps a knock_periodicity of 0.80-0.83 through
    # 0.5-1.5 m of rubble, where other seeds fall to 0-0.5 (the reverberant tail blurs onsets 0.30 s apart).
    emit("knock_pipe_3x", knock_pattern(hit_pipe, np.random.default_rng(5)), "knock",
         synth("modal steel pipe: free-free beam modes of a 1.5 m DN50 pipe, half-sine contact 0.6 ms"),
         processing=["3 groups of 3 knocks, 0.30 s apart, every ~2.6 s", "loudness match"],
         sched=schedule("loop", period_s=20.0, jitter_s=3.0), synthetic=True)
    emit("knock_concrete_3x", knock_pattern(hit_slab, np.random.default_rng(1)), "knock",
         synth("concrete slab: Kirchhoff plate modes of a 3.0 x 2.4 x 0.15 m slab + contact thud"),
         processing=["3 groups of 3 knocks, 0.30 s apart, every ~2.6 s", "loudness match"],
         sched=schedule("loop", period_s=20.0, jitter_s=3.0), synthetic=True)
    emit("knock_pipe_3x_once", knock_pattern(hit_pipe, np.random.default_rng(15), groups=1), "knock",
         synth("modal steel pipe, one group of three: the instructed reply to the call-out"),
         processing=["one group of 3 knocks", "loudness match"],
         sched=schedule("after_callout", latency_s=[1.0, 4.0], repeats=3, period_s=4.0), synthetic=True)
    emit("knock_concrete_3x_once", knock_pattern(hit_slab, np.random.default_rng(11), groups=1), "knock",
         synth("concrete slab, one group of three: the instructed reply to the call-out"),
         processing=["one group of 3 knocks", "loudness match"],
         sched=schedule("after_callout", latency_s=[1.0, 4.0], repeats=3, period_s=4.0), synthetic=True)
    x, fs = load(os.path.join(ESC, "1-101336-A-30_16k.wav"))
    emit("knock_door_esc50", highpass(to48(trim(x, fs), fs), 80), "knock", esc("1-101336-A-30"),
         processing=["ESC-50 door_wood_knock (wood door, not rubble: use as a variety clip)", "resample 16 -> 48 kHz",
                     "high-pass 80 Hz",
                     "loudness match"], sched=schedule("loop", period_s=20.0, jitter_s=3.0))

    # ---- 4. non-verbal human sounds (no groan source: see TODO)
    x, fs = load(os.path.join(ESC, "2-108017-A-24_16k.wav"))
    emit("cough_esc50", highpass(to48(best_window(trim(x, fs), fs, 4.0), fs), 80), "cough", esc("2-108017-A-24"),
         processing=["trim", "resample 16 -> 48 kHz", "high-pass 80 Hz (the clip had sub-50 Hz handling rumble)",
                     "loudness match"],
         sched=schedule("loop", period_s=15.0, jitter_s=5.0))
    x, fs = load(os.path.join(ESC, "3-151080-A-20_16k.wav"))
    emit("cry_esc50", highpass(to48(best_window(trim(x, fs), fs, 4.0), fs), 80), "cry", esc("3-151080-A-20"),
         processing=["trim", "resample 16 -> 48 kHz", "high-pass 80 Hz", "loudness match"],
         sched=schedule("loop", period_s=12.0, jitter_s=4.0),
         notes="ESC-50 class crying_baby: an infant cry (the only cry source on disk). Not used in any seed-7 profile.")

    # ---- 5. distractors (no rubble variants: they are on the surface)
    x, fs = load(os.path.join(FRANCES, "dog_03.wav"))
    emit("dog_bark", highpass(to48(best_window(x, fs, 4.0), fs), 60), "dog", frances("dog_03.wav"),
         processing=["best 4 s window (one bout)", "resample 16 -> 48 kHz", "high-pass 60 Hz", "loudness match"],
         sched=schedule("loop", period_s=15.0, jitter_s=5.0), rubble_variants=False)
    # generator seed 2: its 60 Hz firing pattern aliases least with the pipeline's 10 ms onset envelope
    # (knock_rhythm 0.09 vs up to 0.78 for other seeds), so it is not mistaken for rapping
    emit("generator_60hz", generator(np.random.default_rng(2), 60.0), "generator",
         synth("4-cyl diesel genset at 1800 rpm, 60 Hz mains (Taiwan)"),
         processing=["seamless 12 s loop", "loudness match"], sched=schedule("continuous"),
         synthetic=True, rubble_variants=False)
    emit("generator_50hz", generator(np.random.default_rng(1), 50.0), "generator",
         synth("4-cyl diesel genset at 1500 rpm, 50 Hz (as acoustic/scene.py)"),
         processing=["seamless 12 s loop", "loudness match"], sched=schedule("continuous"),
         synthetic=True, rubble_variants=False)
    emit("settling_creak_trickle", settling(np.random.default_rng(7)), "settling",
         synth("stick-slip creak, crack + thump, Poisson gravel trickle, low rumble"),
         processing=["loudness match"], sched=schedule("loop", period_s=30.0, jitter_s=10.0),
         synthetic=True, rubble_variants=False)
    x, fs = load(os.path.join(FRANCES, "wind_03.wav"))
    emit("wind_real", loopable(to48(best_window(x, fs, 12.5), fs)), "wind", frances("wind_03.wav"),
         processing=["best 12.5 s window", "0.5 s loop crossfade", "resample 16 -> 48 kHz", "loudness match"],
         sched=schedule("continuous", aural="nonSpatial"), rubble_variants=False)
    x, fs = load(os.path.join(FRANCES, "siren_04.wav"))
    emit("siren_distant", fade(to48(best_window(x, fs, 12.0), fs), FS, 0.8, 1.5), "siren", frances("siren_04.wav"),
         processing=["best 12 s window", "fades", "resample 16 -> 48 kHz", "loudness match"],
         sched=schedule("loop", period_s=45.0, jitter_s=10.0), rubble_variants=False)

    manifest = {
        "schema": "larpverse.dummy_audio/v1",
        "generated_by": "scripts/build_dummy_audio.py",
        "updated": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "format": {"sample_rate": FS, "channels": 1, "sample_width_bits": 16, "peak_ceiling_dbfs": PEAK_CEIL_DB,
                   "loudness": "gated RMS dBFS (50 ms frames within 30 dB of the loudest, > -70 dBFS)",
                   "target_rms_dbfs": TARGET_DB},
        "rubble_model": {"depths_m": list(DEPTHS), "loss_db": "min(d * (3 + 6 * f_kHz), 70)",
                         "void_reverb": "RT60 0.22 s, direct-to-reverberant +8 dB",
                         "status": "ESTIMATE, not measured. 3 dB/m broadband matches acoustic/scene.py and "
                                   "sim/callout.py; the frequency slope is a heuristic for porous granular rubble"},
        "voice_licences": VOICE_LICENCES,
        "excluded_voices": EXCLUDED_VOICES,
        "kinds": list(KINDS),
        "clips": clips,
        "todo": TODO,
        "profiles": PROFILES,
        "hidden_dummies": HIDDEN,
        "ambient": AMBIENT,
    }
    with open(os.path.join(out, "manifest.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=1, ensure_ascii=False)
    write_attribution(manifest, out)
    return manifest


def _used_clips(man):
    used = set()
    for p in man["profiles"] + man["hidden_dummies"]:
        for key in ("idle", "after_callout"):
            if p.get(key):
                used |= {c.split("__")[0] for c in p[key]["playlist"]}
    return used | {a["clip"] for a in man["ambient"]}


def write_attribution(man, out=OUT):
    """assets/audio/dummies/ATTRIBUTION.md: every non-original clip, its author and licence, and credit lines."""
    base = [c for c in man["clips"] if c["parent"] is None]
    used = _used_clips(man)
    L = ["# Dummy audio: attribution and licences", "",
         "Generated by `scripts/build_dummy_audio.py` from `manifest.json`. Freesound authors and licences were checked "
         "on freesound.org, and the TTS voice licences on the rhasspy/piper-voices model cards, on 2026-11-08.",
         "Each clip's `rubble/<id>__<depth>m.wav` variants carry the same credit and licence as the clip.", "",
         "**Non-commercial:** the library as a whole is for this non-commercial hackathon demo only. The ESC-50 clips "
         "(CC BY-NC 3.0 as distributed) and the Vietnamese TTS (CC BY-NC-SA 4.0, share-alike) are non-commercial; "
         "`non_commercial` is true on those clips in the manifest.", "",
         "## Credit line for the video description", "",
         "Short version, covering the clips that the dummy profiles play:", "", "> " + credit_line(man, used), "",
         "Full version, covering every clip in the library:", "", "> " + credit_line(man, {c["id"] for c in base}), "",
         "## Recorded clips (not made by us)", "",
         "| clip | used in a profile | author | original | licence | source |", "|---|---|---|---|---|---|"]
    for c in base:
        if c["synthetic"]:
            continue
        L.append(f"| `{c['id']}` | {'yes' if c['id'] in used else 'no'} | {c['author']} | {c['origin_url']} | "
                 f"{c['licence']} | {c['source'].split(', data/')[0]} |")
    L += ["", "## Text-to-speech clips", "", "| clip | used in a profile | engine | licence | non-commercial |",
          "|---|---|---|---|---|"]
    for c in base:
        if c["synthetic"] and c["source"].startswith("TTS"):
            L.append(f"| `{c['id']}` | {'yes' if c['id'] in used else 'no'} | {c['source'].split(';')[0][5:]} | "
                     f"{c['licence']} | {'yes' if c['non_commercial'] else 'no'} |")
    L += ["", "Voices on disk that are **not used**, and why:", ""]
    L += [f"- {k}: {v}" for k, v in man["excluded_voices"].items()]
    L += ["", "## Generated here (no third-party content)", "",
          ", ".join(f"`{c['id']}`" for c in base if c["synthetic"] and not c["source"].startswith("TTS")) +
          ": procedural numpy/scipy synthesis by the Larpverse team, CC0.", ""]
    with open(os.path.join(out, "ATTRIBUTION.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(L))


def credit_line(man, ids):
    clips = [c for c in man["clips"] if c["parent"] is None and c["id"] in ids]
    cc0 = sorted({c["author"] for c in clips if not c["synthetic"] and c["licence"].startswith("CC0")})
    parts = []
    if cc0:
        parts.append("Recordings from Freesound by " + ", ".join(cc0) + " (CC0)")
    esc_ = [c for c in clips if c["source"].startswith("ESC-50")]
    if esc_:
        parts.append("; ".join(c["attribution"].split(", via ")[0] for c in esc_) +
                     ", via the ESC-50 dataset (K. J. Piczak, 2015, CC BY-NC 3.0)")
    if any("vivos" in c["source"] for c in clips):
        parts.append("Vietnamese speech synthesised with the Piper vi_VN-vivos voice, trained on the VIVOS corpus "
                     "(AILAB, University of Science, VNU-HCM), CC BY-NC-SA 4.0")
    if any("eSpeak" in c["source"] for c in clips):
        parts.append("Other synthetic speech by eSpeak NG")
    return "Audio: " + ". ".join(parts) + ". Non-commercial demo."


# ============================================================================ profiles
# seed-7 dummies (sim/scenario.py truth: roles, DEMO_PHYSIO, victim depths 0.89 / 1.22 / 1.16 / 1.64 m
# rounded to the nearest library depth). Isaac prim path convention: /World/Dummies/<dummy_id>.
def _r(cid, d):
    return f"{cid}__{d:.1f}m"


PROFILES = [
    {"dummy_id": "A3", "pile": "A3", "scenario_kind": "voice", "physio": "distressed_tachypnea",
     "burial_depth_m": 0.5, "prim": "/World/Dummies/A3",
     "depth_note": "acoustic path 0.5 m (the voice escapes through a void opening; acoustic/scene.py uses 0.4 m "
                   "for voices); the radar depth stays the scenario's 0.89 m",
     "idle": {"playlist": [_r("voice_zh_call_tts", 0.5), _r("voice_en_help_real_b", 0.5), _r("voice_en_please_real", 0.5)],
              "order": "cycle", **schedule("loop", start_s=1.0, period_s=9.0, jitter_s=2.0)},
     "after_callout": {"playlist": [_r("voice_zh_here_tts", 0.5), _r("voice_en_help_real_a", 0.5)], "order": "cycle",
                       **schedule("after_callout", latency_s=[1.5, 3.5], repeats=2, period_s=4.0)},
     "story": "shouts in Mandarin (Taiwanese wording) and English: 'my leg is trapped'"},
    {"dummy_id": "B1", "pile": "B1", "scenario_kind": "weak_voice", "physio": "irregular_apneic",
     "burial_depth_m": 1.0, "prim": "/World/Dummies/B1",
     "idle": {"playlist": [_r("weak_en_tts", 1.0), _r("weak_zh_tts", 1.0)], "order": "cycle",
              **schedule("loop", start_s=4.0, period_s=25.0, jitter_s=5.0)},
     "after_callout": None,
     "story": "weak, fading 'hello... is anyone there... water...'; no voluntary reply to the call-out "
              "(irregular_apneic, sim/callout.RESPONSE_MODEL)"},
    {"dummy_id": "B2", "pile": "B2", "scenario_kind": "knock", "physio": "calm",
     "burial_depth_m": 1.0, "prim": "/World/Dummies/B2",
     "idle": {"playlist": [_r("knock_concrete_3x", 1.0)], "order": "cycle",
              **schedule("loop", start_s=6.0, period_s=20.0, jitter_s=3.0)},
     "after_callout": {"playlist": [_r("knock_concrete_3x_once", 1.0)], "order": "cycle",
                       **schedule("after_callout", latency_s=[1.0, 4.0], repeats=3, period_s=4.0)},
     "alternatives": [_r("knock_pipe_3x", 1.0), _r("knock_pipe_3x_once", 1.0)],
     "story": "conscious and calm, knocks three times with a stone on a slab; knocks back after "
              "'knock three times'. The pipe clips ring (tonal) and the pipeline read them as dog once "
              "in the check: keep concrete for the take"},
    {"dummy_id": "A2", "pile": "A2", "scenario_kind": "silent", "physio": "unconscious_slow",
     "burial_depth_m": 1.5, "prim": "/World/Dummies/A2",
     "idle": None, "after_callout": None,
     "story": "unconscious: silent, found by radar only. No audio prim needed (or a muted one for live toggles)"},
    {"dummy_id": "C2", "pile": "C2", "scenario_kind": "dog", "physio": None, "burial_depth_m": 0.0,
     "prim": "/World/Dummies/C2_dog",
     "idle": {"playlist": ["dog_bark"], "order": "cycle", **schedule("loop", start_s=3.0, period_s=15.0, jitter_s=5.0)},
     "after_callout": {"playlist": ["dog_bark"], "order": "cycle",
                       **schedule("after_callout", latency_s=[0.5, 2.0], repeats=1, period_s=0.0)},
     "story": "distractor: a dog on the pile surface (barks at the loudspeaker too); must be dismissed"},
    {"dummy_id": "C1", "pile": "C1", "scenario_kind": "generator", "physio": None, "burial_depth_m": 0.0,
     "prim": "/World/Dummies/C1_generator",
     "idle": {"playlist": ["generator_60hz"], "order": "cycle", **schedule("continuous")},
     "after_callout": None, "story": "distractor: a running 60 Hz generator; must be dismissed"},
    {"dummy_id": "B3", "pile": "B3", "scenario_kind": "settling", "physio": None, "burial_depth_m": 0.0,
     "prim": "/World/Dummies/B3_settling",
     "idle": {"playlist": ["settling_creak_trickle"], "order": "cycle",
              **schedule("loop", start_s=8.0, period_s=30.0, jitter_s=10.0)},
     "after_callout": None, "story": "distractor: rubble settling (creak, crack, trickle); must be dismissed"},
]

HIDDEN = [
    {"dummy_id": "H1_C3_vi", "pile": "C3", "scenario_kind": "voice", "physio": "calm", "burial_depth_m": 0.5,
     "prim": "/World/Dummies/H1_C3_vi", "active_at_start": False,
     "activation": "live, mid-take: an operator flips larpverse:active = true on the prim (C3 is a sector "
                   "nobody tasked in seed 7: no sound, no lead)",
     "idle": {"playlist": [_r("voice_vi_call_tts", 0.5), _r("voice_vi_here_tts", 0.5)], "order": "cycle",
              **schedule("loop", start_s=0.5, period_s=8.0, jitter_s=1.5)},
     "after_callout": {"playlist": [_r("voice_vi_here_tts", 0.5)], "order": "cycle",
                       **schedule("after_callout", latency_s=[1.0, 3.0], repeats=2, period_s=4.0)},
     "story": "a Vietnamese migrant worker starts calling 'Cứu tôi với!' from C3 mid-run; the system must "
              "hear a new human source in an untasked sector and re-rank"},
    {"dummy_id": "H2_A1_id", "pile": "A1", "scenario_kind": "voice", "physio": "distressed_tachypnea",
     "burial_depth_m": 1.0, "prim": "/World/Dummies/H2_A1_id", "active_at_start": False,
     "activation": "live, alternative to H1: toggle larpverse:active on the prim (A1 is silent in seed 7)",
     "idle": {"playlist": [_r("voice_id_call_tts", 1.0), _r("voice_id_here_tts", 1.0)], "order": "cycle",
              **schedule("loop", start_s=0.5, period_s=9.0, jitter_s=2.0)},
     "after_callout": {"playlist": [_r("voice_id_here_tts", 1.0)], "order": "cycle",
                       **schedule("after_callout", latency_s=[1.5, 3.5], repeats=2, period_s=4.0)},
     "story": "an Indonesian worker calls 'Tolong!' from A1"},
]

AMBIENT = [
    {"id": "wind", "clip": "wind_real", "aural": "nonSpatial", "gain": 0.25, **schedule("continuous"),
     "note": "site bed under everything (CC0 field recording, loop-crossfaded)"},
    {"id": "siren", "clip": "siren_distant", "aural": "spatial", "position_hint": "beyond the site edge, ~150 m",
     "gain": 0.35, **schedule("loop", start_s=20.0, period_s=45.0, jitter_s=10.0),
     "note": "distant siren distractor"},
]


# ============================================================================ pipeline check
def check(out=OUT):
    """Seed-7 listen with library clips as the sources, through the unchanged acoustic pipeline."""
    import pyroomacoustics as pra
    sys.path.insert(0, ROOT)
    from acoustic import run, scene as S
    from sim.scenario import make_scenario

    man = json.load(open(os.path.join(out, "manifest.json"), encoding="utf-8"))
    by_id = {c["id"]: c for c in man["clips"]}

    def clip16(cid):
        x, fs = load(os.path.join(out, by_id[cid]["file"]))
        return sps.resample_poly(x, 1, 3)

    def dry_rms(cid):           # gain is set on the DRY parent, so a rubble variant keeps its loss
        p = by_id[cid]["parent"] or cid
        x = clip16(p)
        return float(np.sqrt(np.mean(x[np.abs(x) > 1e-4] ** 2)))

    sc = make_scenario(7)
    truth_xy = {s["pile"]: s["xy"] for s in sc.truth["sounds"]}
    level_db = {s["pile"]: s.get("level_db", 0.0) for s in sc.truth["sounds"]}
    piles = S.public_piles(sc)
    # (pile, scene.KINDS key for the 1 m level, clips at seconds)
    plan = {
        "baseline": [
            ("A3", "voice", [("voice_zh_call_tts__0.5m", 1.0), ("voice_en_please_real__0.5m", 12.5)]),
            ("B1", "weak_voice", [("weak_en_tts__1.0m", 5.0)]),
            ("B2", "knock", [("knock_concrete_3x__1.0m", 9.0)]),
            ("C2", "dog", [("dog_bark", 2.0), ("dog_bark", 18.5)]),
            ("C1", "generator", [("generator_60hz", 0.0), ("generator_60hz", 12.0), ("generator_60hz", 24.0)]),
            ("B3", "settling", [("settling_creak_trickle", 14.0)]),
        ],
    }
    plan["hidden_C3_vi"] = plan["baseline"] + [
        ("C3", "voice", [("voice_vi_call_tts__0.5m", 6.0), ("voice_vi_here_tts__0.5m", 18.0)])]
    # stress: the dog barks right over the knocking (no time separation)
    plan["stress_dog_over_knock"] = [r if r[0] != "C2" else ("C2", "dog", [("dog_bark", 9.3), ("dog_bark", 13.5)])
                                     for r in plan["baseline"]]
    c3 = piles["C3"]
    truth_xy["C3"] = [c3[0] + 1.0, c3[1] - 0.8]
    level_db["C3"] = 0.0
    drones = S.drone_positions(piles["B2"])          # centre listen: every pile within ~33 m
    results = {}
    for name, srcs in plan.items():
        rng = np.random.default_rng(1007)
        n = int(S.DURATION * S.FS)
        room = pra.AnechoicRoom(dim=2, fs=S.FS)
        truth = {}
        for pile, k, events in srcs:
            track = np.zeros(n)
            for cid, t0 in events:
                x = clip16(cid) / dry_rms(cid)
                s = int(t0 * S.FS)
                e = min(n, s + len(x))
                track[s:e] += x[: e - s]
            gain = S.KINDS[k]["level"] * 10 ** (level_db[pile] / 20)
            room.add_source(list(truth_xy[pile]), signal=track * gain)
            truth[pile] = {"clips": [c for c, _ in events], "kind": k}
        ids = list(drones)
        offs = S.mic_offsets()
        room.add_microphone_array(pra.MicrophoneArray(np.hstack([np.array(drones[d])[:, None] + offs for d in ids]), S.FS))
        room.simulate()
        sim = room.mic_array.signals[:, :n]
        if sim.shape[1] < n:
            sim = np.pad(sim, ((0, 0), (0, n - sim.shape[1])))
        t = np.arange(n) / S.FS
        bw, aw = sps.butter(2, 120 / (S.FS / 2))
        sigs = {}
        for k_, d in enumerate(ids):           # same ambient model as acoustic/scene.build_scene
            blk = sim[k_ * S.N_MICS:(k_ + 1) * S.N_MICS].copy()
            gust = 1 + 0.6 * np.sin(2 * np.pi * rng.uniform(0.05, 0.2) * t + rng.uniform(0, 6.28))
            for m in range(S.N_MICS):
                blk[m] += S._rms_norm(sps.lfilter(bw, aw, rng.normal(0, 1, n))) * gust * 3 * 4e-3 + rng.normal(0, 4e-3, n)
            sigs[d] = blk.astype(np.float32)
        scene = S.Scene(7, "B2", S.FS, S.DURATION, piles, drones, offs, sigs)
        t0 = time.time()
        state = run.analyse(scene, exclude_piles=(), mode="fallback", gemini_audio=False)
        rows = []
        for c in state["clips"]:
            rows.append({"pile": c["pile_id"], "truth": truth.get(c["pile_id"], {}).get("kind", "none"),
                         "sound_class": c["sound_class"], "human": c["human"], "urgency": c["urgency"],
                         "reliability": c["reliability"], "vad": c["vad_speech_ratio"],
                         "transcript": c["transcript"], "association": c.get("association"),
                         "knock_periodicity": c["heuristics"].get("knock_periodicity"),
                         "hum_harmonicity": c["heuristics"].get("hum_harmonicity"),
                         "class_scores": c["class_scores"]})
        pri = [{"pile": p["pile_id"], "priority": p["priority"], "status": p["status"]} for p in state["piles"]]
        verdict = {}
        for pile, info in truth.items():
            r = next((x for x in rows if x["pile"] == pile), None)
            if r is None:
                verdict[pile] = f"{info['kind']}: no clip (not detected)"
            elif info["kind"] in ("voice", "weak_voice"):
                verdict[pile] = f"{info['kind']}: {'PASS' if r['human'] >= 0.4 else 'FAIL'} human {r['human']:.2f} ({r['sound_class']})"
            elif info["kind"] == "knock":
                ok = r["sound_class"] == "knocking" and r["human"] >= 0.4
                verdict[pile] = f"knock: {'PASS' if ok else 'FAIL'} class {r['sound_class']}, human {r['human']:.2f}"
            else:
                ok = r["human"] < 0.4
                verdict[pile] = f"{info['kind']}: {'PASS' if ok else 'FAIL'} dismissed (human {r['human']:.2f}, {r['sound_class']})"
        results[name] = {"drones": {k: list(v) for k, v in drones.items()}, "clips": rows, "piles": pri,
                         "dispatch": state["dispatch"], "verdict": verdict, "analyse_s": round(time.time() - t0, 1)}
        print(f"\n== {name} (analyse {time.time() - t0:.0f} s, dispatch {state['dispatch'].get('pile_id')})")
        for pile, v in verdict.items():
            print(f"  {pile}: {v}")
        for r in rows:
            print(f"   {r['pile']} class={r['sound_class']:9s} human={r['human']:.2f} vad={r['vad']:.2f} "
                  f"knock={r['knock_periodicity']} hum={r['hum_harmonicity']} assoc={r['association']} '{r['transcript'][:50]}'")
        if name == "baseline":                   # soundtrack demo: what drone d2 hears, raw vs beamformed
            _soundtrack_demo(scene, state, out)
    with open(os.path.join(out, "pipeline_check.json"), "w", encoding="utf-8") as f:
        json.dump({"schema": "larpverse.dummy_audio_check/v1", "updated": time.strftime("%Y-%m-%dT%H:%M:%S"),
                   "how": "scripts/build_dummy_audio.py check: library clips placed at seed-7 truth positions, "
                          "pyroomacoustics free field like acoustic/scene.build_scene, then acoustic.run.analyse "
                          "(mode=fallback, gemini_audio off). Unchanged acoustic code.",
                   "results": results}, f, indent=1, ensure_ascii=False)


def _soundtrack_demo(scene, state, out):
    """Write check/d2_raw.wav (mic 0 of drone d2) and check/d2_beam_<pile>.wav (LCMV beam the pipeline built)."""
    from acoustic import dsp
    dets, B = dsp.localise_piles(scene, exclude=set())
    os.makedirs(os.path.join(out, "check"), exist_ok=True)
    raw = scene.signals["d2"][0].astype(float)
    write_wav(os.path.join(out, "check", "d2_raw_16k.wav"), 0.5 * raw / (np.max(np.abs(raw)) + 1e-9), 16000)
    for d in dets:
        if d["pile_id"] in ("A3", "B2"):
            did, snr, clip, dom, idx = dsp.extract_clip(scene, d, B, [o for o in dets if o is not d])
            write_wav(os.path.join(out, "check", f"beam_{d['pile_id']}_{did}_16k.wav"),
                      0.5 * clip / (np.max(np.abs(clip)) + 1e-9), 16000)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", nargs="?", default="build", choices=["build", "check"])
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()
    if a.cmd == "build":
        m = build(a.out)
        tot = sum(os.path.getsize(os.path.join(a.out, c["file"])) for c in m["clips"])
        print(f"{len(m['clips'])} clips, {tot / 1e6:.1f} MB -> {a.out}/manifest.json")
    else:
        check(a.out)
