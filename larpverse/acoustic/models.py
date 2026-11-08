"""Per-clip recognisers. All CPU, no torch/tensorflow:
  * Silero VAD v5 (ONNX via onnxruntime, MIT)
  * YAMNet (TFLite via ai-edge-litert, Apache-2.0), 521 AudioSet classes grouped into ours
  * faster-whisper (CTranslate2 int8, MIT), base.en by default
  * two documented heuristics: knock periodicity (onset autocorrelation) and mains/engine
    harmonicity (power at 50/60 Hz harmonics). They only boost YAMNet's group score.
"""
from __future__ import annotations

import csv
import os
import re

import numpy as np
from scipy import signal as sps

from .scene import FS, MODEL_DIR

CLASSES = ["voice", "shout", "knocking", "dog", "engine", "debris", "noise"]
# YAMNet class indices (yamnet_class_map.csv) per group
GROUPS = {
    "voice": [0, 1, 2, 3, 5, 19, 21, 36, 65],
    "shout": [6, 9, 10, 11],
    "knocking": [353, 354, 168, 431, 454],
    "dog": [69, 70, 71, 72, 73, 74, 75, 117],
    "engine": [337, 338, 342, 343, 346, 490, 510, 406, 300],
    "debris": [487, 434, 293, 130, 460, 437, 463, 464, 469, 473, 420, 426, 428, 430, 433, 472, 483, 281],
    "noise": [494, 277, 279, 507, 508, 509, 514, 515],
}
_cache = {}
STAGE = {}                 # seconds per pipeline stage since the last reset_stages() (listen timings)


def reset_stages():
    STAGE.clear()


def timed(name):
    """Decorator: add the wall time of each call to STAGE[name]."""
    import functools
    import time as _t

    def deco(fn):
        @functools.wraps(fn)
        def wrap(*a, **k):
            t0 = _t.perf_counter()
            try:
                return fn(*a, **k)
            finally:
                STAGE[name] = STAGE.get(name, 0.0) + _t.perf_counter() - t0
        return wrap
    return deco
# well-known Whisper outputs on non-speech audio
HALLUCINATIONS = {"thank you", "thanks for watching", "you", "bye", "thank you for watching"}


def _norm(x, target_rms=0.05):
    x = np.asarray(x, np.float32)
    r = float(np.sqrt(np.mean(x ** 2))) + 1e-9
    return np.clip(x * (target_rms / r), -1, 1).astype(np.float32)


# --------------------------------------------------------------------------- VAD
@timed("vad")
def vad_probs(x) -> np.ndarray:
    """Silero speech probability per 512-sample (32 ms) frame."""
    import onnxruntime as ort
    if "vad" not in _cache:
        so = ort.SessionOptions()
        so.intra_op_num_threads = 1
        so.inter_op_num_threads = 1
        _cache["vad"] = ort.InferenceSession(os.path.join(MODEL_DIR, "silero_vad.onnx"), so,
                                             providers=["CPUExecutionProvider"])
    s = _cache["vad"]
    x = _norm(x)
    state = np.zeros((2, 1, 128), np.float32)
    ctx = np.zeros((1, 64), np.float32)
    sr = np.array(FS, np.int64)
    probs = []
    for i in range(0, len(x) - 512 + 1, 512):
        inp = np.concatenate([ctx, x[None, i:i + 512]], axis=1)
        out, state = s.run(None, {"input": inp, "state": state, "sr": sr})
        ctx = inp[:, -64:]
        probs.append(float(out[0, 0]))
    return np.array(probs)


def vad_speech_ratio(x, thr=0.5) -> float:
    p = vad_probs(x)
    return round(float(np.mean(p > thr)) if len(p) else 0.0, 3)


def block_speech(x, blk: int, thr=0.5) -> np.ndarray:
    """Speech-frame share per consecutive block of `blk` samples (the clip's 0.5 s blocks)."""
    p = vad_probs(x) > thr
    nb = int(np.ceil(len(x) / blk))
    centre = (np.arange(len(p)) * 512 + 256) // blk
    return np.array([float(p[centre == j].mean()) if np.any(centre == j) else 0.0 for j in range(nb)])


# --------------------------------------------------------------------------- YAMNet
def _yamnet_labels():
    with open(os.path.join(MODEL_DIR, "yamnet_class_map.csv")) as f:
        return [r["display_name"] for r in csv.DictReader(f)]


@timed("yamnet")
def yamnet_scores(x):
    """Returns (frame scores (T, 521), labels)."""
    if "yam" not in _cache:
        from ai_edge_litert.interpreter import Interpreter
        it = Interpreter(os.path.join(MODEL_DIR, "yamnet.tflite"), num_threads=2)
        it.allocate_tensors()
        _cache["yam"] = (it, _yamnet_labels())
    it, labels = _cache["yam"]
    x = _norm(x, 0.1)
    W, H = 15600, 7800
    if len(x) < W:
        x = np.pad(x, (0, W - len(x)))
    i_in = it.get_input_details()[0]["index"]
    i_out = it.get_output_details()[0]["index"]
    frames = []
    for s in range(0, len(x) - W + 1, H):
        it.set_tensor(i_in, x[s:s + W])
        it.invoke()
        frames.append(it.get_tensor(i_out)[0].copy())
    return np.array(frames), labels


# --------------------------------------------------------------------------- heuristics
def onset_times(x):
    """Impulsive onsets (s): peaks of a 10 ms envelope (>150 Hz) rising 4x above its median, >= 80 ms
    apart. None for clips shorter than 0.6 s."""
    from scipy.signal import find_peaks
    b, a = sps.butter(2, 150 / (FS / 2), "high")
    e = np.abs(sps.lfilter(b, a, np.asarray(x, float)))
    hop = FS // 100
    env = e[: len(e) // hop * hop].reshape(-1, hop).max(axis=1)
    if len(env) < 60:
        return None
    pk, _ = find_peaks(env, height=4 * np.median(env), distance=8)
    return pk / 100.0


def _knock_runs(t):
    """Evenly spaced runs of onsets (knock groups) inside 0.5 s-gap groups of onset times t (s).

    A run is >= 3 consecutive onsets (or a whole 2-onset group) whose spacings are 0.12-0.6 s and
    within a factor 1.4 of each other. A run is `dense` when its group also holds ticks < 0.1 s apart
    or more than 5 onsets (a debris burst). Returns ([(spacings, dense)], onsets in multi-onset groups)."""
    groups = np.split(t, np.where(np.diff(t) > 0.5)[0] + 1)
    runs, n_multi = [], 0
    for g in groups:
        if len(g) < 2:
            continue                     # stray single onsets are neutral
        n_multi += len(g)
        d = np.diff(g)
        dense = bool(d.min() < 0.1 or len(g) > 5)
        i = 0
        while i < len(d):
            j = i
            while (j + 1 < len(d) and 0.12 <= d[j + 1] <= 0.6 and 0.12 <= d[i] <= 0.6
                   and max(d[i:j + 2]) <= 1.4 * min(d[i:j + 2])):
                j += 1
            if 0.12 <= d[i] <= 0.6 and (j > i or len(g) == 2):
                runs.append((list(d[i:j + 1]), dense))
            i = j + 1
    return runs, n_multi


def knock_rhythm(x, lo=0.12, hi=0.45) -> float:
    """Rapping-rhythm score: peak of the normalised autocorrelation of the onset-strength curve
    (log envelope >150 Hz, 10 ms hop, half-wave rectified flux) at 0.12-0.45 s lags. Door knocks
    (ESC-50: 9/12 clips > 0.25) repeat at a steady rate; debris, fire, rain, creaks and footsteps
    do not (0-2/12 > 0.25), and the synthetic rubble debris stays < 0.15."""
    b, a = sps.butter(2, 150 / (FS / 2), "high")
    e = np.abs(sps.lfilter(b, a, np.asarray(x, float)))
    hop = FS // 100
    env = e[: len(e) // hop * hop].reshape(-1, hop).max(axis=1)
    if len(env) < 60:
        return 0.0
    flux = np.maximum(np.diff(np.log(env + 1e-9 * env.max() + 1e-12)), 0)
    flux = flux - flux.mean()
    r = np.correlate(flux, flux, "full")[len(flux) - 1:]
    r = r / (r[0] + 1e-12)
    lags = np.arange(len(r)) / 100.0
    w = (lags >= lo) & (lags <= hi)
    return round(float(max(0.0, r[w].max())), 3) if w.any() else 0.0


@timed("heuristics")
def knock_periodicity(x, rhythm_ok=True) -> float:
    """Patterned-knock score in [0, 1]: the larger of a group score and a rhythm score.

    Group score (deliberate signalling, e.g. "knock three times"): onsets (onset_times) form short
    groups of evenly spaced knocks repeated with the same spacing (here 3 knocks 0.22 s apart every
    ~2 s). At least one group must stand clear of any debris burst; it sets the spacing. 0 below
    two groups within +-15 % of that spacing, 0.7 for two, 1.0 for three or more, times the share
    of grouped onsets in those groups. Rhythm score (continuous rapping, real door knocks):
    knock_rhythm mapped 0.2 -> 0, 0.4 -> 1. Settling debris (dense bursts of irregular ticks)
    scores low on both. rhythm_ok=False (an engine-like clip) uses the group score only: idling
    engines and rotors are rhythmic too."""
    rhythm = float(np.clip((knock_rhythm(x) - 0.2) / 0.2, 0, 1)) if rhythm_ok else 0.0
    t = onset_times(x)
    if t is None or len(t) < 4:
        return round(rhythm, 3)
    runs, n_multi = _knock_runs(t)
    clean = [r for r, dense in runs if not dense]
    group = 0.0
    if clean:                            # knock groups must show up at least once on their own
        med = float(np.median([np.median(r) for r in clean]))
        runs = [r for r, _ in runs if abs(np.median(r) - med) <= 0.15 * med]
        if len(runs) >= 2:
            share = sum(len(r) + 1 for r in runs) / max(n_multi, 1)
            group = (0.7 if len(runs) == 2 else 1.0) * min(1.0, share)
    return round(float(max(group, rhythm)), 3)


@timed("heuristics")
def loudness_steadiness(x, frame_s=0.05) -> float:
    """p10/p90 of 50 ms band (150-4000 Hz) RMS. Rotors, engines, rain and wind are steady (ESC-50
    helicopter median 0.47, engine 0.73); speech, cries, coughs, knocks and debris have gaps
    (<= 0.08 on every ESC-50 human class)."""
    b, a = sps.butter(2, [150 / (FS / 2), 4000 / (FS / 2)], "band")
    y = sps.lfilter(b, a, np.asarray(x, float))
    n = int(frame_s * FS)
    if len(y) < 4 * n:
        return 0.0
    e = np.sqrt(np.mean(y[: len(y) // n * n].reshape(-1, n) ** 2, axis=1)) + 1e-12
    return round(float(np.percentile(e, 10) / np.percentile(e, 90)), 3)


@timed("heuristics")
def blade_pass(x, fmin=40.0, fmax=400.0, band=(80.0, 2500.0)) -> float:
    """Blade-pass comb: share of 80-2500 Hz power in stationary spectral lines (>= 4x the local
    median, whole-clip Welch, 2 Hz bins) that sit on >= 3 harmonics of one f0 in 40-400 Hz.
    A rotor's BPF comb is steady over the clip; speech pitch moves and smears out."""
    f, P = sps.welch(np.asarray(x, float), FS, nperseg=8192)
    sel = (f >= band[0]) & (f <= band[1])
    fb, Pb = f[sel], P[sel]
    if len(Pb) < 20:
        return 0.0
    df = f[1] - f[0]
    k = int(40 / df)
    pad = np.pad(Pb, k, mode="edge")
    med = np.array([np.median(pad[i:i + 2 * k + 1]) for i in range(len(Pb))])
    line = Pb > 4 * med
    if not line.any():
        return 0.0
    tot = Pb.sum() + 1e-20
    best = 0.0
    for f0 in np.arange(fmin, fmax, 1.0):
        h = np.arange(np.ceil(band[0] / f0), np.floor(band[1] / f0) + 1) * f0
        idx = np.clip(np.round((h - fb[0]) / df).astype(int), 0, len(fb) - 1)
        on = np.zeros(len(fb), bool)
        nh = 0
        for i in idx:                    # +-1 bin around each harmonic
            w = slice(max(i - 1, 0), i + 2)
            if line[w].any():
                nh += 1
                on[w] |= line[w]
        if nh >= 3:
            best = max(best, float(Pb[on].sum() / tot))
    return round(best, 3)


@timed("heuristics")
def hum_harmonicity(x) -> float:
    """Fraction of 90-1000 Hz power that sits on 50 or 60 Hz harmonics (+-2 Hz); wind lives below."""
    f, P = sps.welch(x, FS, nperseg=8192)
    band = (f >= 90) & (f <= 1000)
    tot = P[band].sum() + 1e-15
    best = 0.0
    for f0 in (50.0, 60.0):
        k = np.arange(1, int(1000 // f0) + 1) * f0
        on = np.zeros_like(f, bool)
        for h in k:
            on |= np.abs(f - h) <= 2.0
        best = max(best, P[band & on].sum() / tot)
    return round(float(best), 3)


# Teammate (F. Valisalo, branch acoustic-voice-detection) human-minus-animal YAMNet score: per
# 0.975 s frame max(human classes) - max(animal classes), max over frames. On real audio under rotor
# noise it ranks speech better than the VAD-scaled evidence (docs/ACOUSTIC_VALIDATION.md).
HMA_HUMAN = {"Speech", "Child speech, kid speaking", "Conversation", "Narration, monologue", "Babbling", "Shout",
             "Bellow", "Whoop", "Yell", "Children shouting", "Screaming", "Crying, sobbing",
             "Baby cry, infant cry", "Whimper", "Wail, moan", "Groan", "Hubbub, speech noise, speech babble"}
HMA_ANIMAL = {"Animal", "Domestic animals, pets", "Dog", "Bark", "Yip", "Howl", "Bow-wow", "Growling",
              "Whimper (dog)", "Livestock, farm animals, working animals", "Wild animals"}
HMA_MIN = 0.2


def human_minus_animal(fr, labels) -> float:
    if "hma_idx" not in _cache:
        _cache["hma_idx"] = ([i for i, n in enumerate(labels) if n in HMA_HUMAN],
                             [i for i, n in enumerate(labels) if n in HMA_ANIMAL])
    h, a = _cache["hma_idx"]
    if not len(fr):
        return 0.0
    return round(float(np.max(fr[:, h].max(axis=1) - fr[:, a].max(axis=1))), 3)


def classify(x, vad=None):
    fr, labels = yamnet_scores(x)
    agg = 0.5 * fr.mean(axis=0) + 0.5 * fr.max(axis=0)
    g = {c: float(agg[idx].max()) for c, idx in GROUPS.items()}
    yam_engine = g["engine"]
    hh = hum_harmonicity(x)
    heur = {"knock_periodicity": knock_periodicity(x, rhythm_ok=yam_engine < 0.3 and hh < 0.3),
            "hum_harmonicity": hh, "human_minus_animal": human_minus_animal(fr, labels),
            "steadiness": loudness_steadiness(x), "blade_pass": blade_pass(x)}
    g["knocking"] = max(g["knocking"], 0.8 * heur["knock_periodicity"])
    g["engine"] = max(g["engine"], float(np.clip((heur["hum_harmonicity"] - 0.15) / 0.35, 0, 1)))
    if heur["knock_periodicity"] >= 0.5 and heur["hum_harmonicity"] < 0.35 and yam_engine < 0.5:
        # a clear knock pattern without mains/engine harmonics: YAMNet's broadband "engine" guess
        # (knocks through rubble are low-passed thuds) must not override it
        g["engine"] = min(g["engine"], 0.5 * g["knocking"])
    tot = sum(g.values()) + 1e-9
    scores = {c: round(g[c] / tot, 3) for c in CLASSES}
    top = [(labels[i], round(float(agg[i]), 3)) for i in np.argsort(-agg)[:3]]
    fg = {c: v for c, v in scores.items() if c != "noise"}       # background only if nothing else
    if vad is not None and vad < 0.1 and (heur["human_minus_animal"] < HMA_MIN or heur["knock_periodicity"] >= 0.5):
        # no voice activity and no human-over-animal YAMNet frame -> never a "voice" headline (R2-015);
        # shouts that Silero misses (VAD ~0) keep it when YAMNet hears a human clearly, unless the
        # clip carries a clear knock pattern (a knocker under a distant talker's leak)
        fg = {c: v for c, v in fg.items() if c not in ("voice", "shout")}
    cls = max(fg, key=fg.get) if max(fg.values()) >= 0.15 else "noise"
    return cls, scores, heur, top


# --------------------------------------------------------------------------- ASR
ASR_MAX_S = 12.0          # Whisper sees at most the loudest 12 s of a clip (latency)


def _asr_window(x, max_s=ASR_MAX_S):
    n = int(max_s * FS)
    if len(x) <= n:
        return x
    e = np.convolve(np.asarray(x, float) ** 2, np.ones(FS // 2), "valid")[::FS // 4]   # 0.5 s energy, 0.25 s hop
    w = n // (FS // 4)
    s = int(np.argmax(np.convolve(e, np.ones(max(w - 1, 1)), "valid"))) * (FS // 4)
    return x[s:s + n]


@timed("whisper")
def transcribe(x, model="base.en") -> str:
    key = "asr:" + model
    if key not in _cache:
        from faster_whisper import WhisperModel
        _cache[key] = WhisperModel(model, device="cpu", compute_type="int8", cpu_threads=2,
                                   download_root=os.path.join(MODEL_DIR, "whisper"))
    segs, _ = _cache[key].transcribe(_norm(_asr_window(x), 0.1), language="en", beam_size=1, temperature=0.0,
                                     condition_on_previous_text=False, vad_filter=False)
    # a segment must look like speech to Whisper itself: low no-speech probability AND a
    # reasonable average log-probability (hallucinations on barks/knocks fail one or the other)
    words = [s.text.strip() for s in segs if s.no_speech_prob < 0.6 and s.avg_logprob > -1.0]
    text = re.sub(r"\s+", " ", " ".join(words)).strip()
    if text.lower().strip(" .!") in HALLUCINATIONS:
        return ""
    return text
