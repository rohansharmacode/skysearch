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
# well-known Whisper outputs on non-speech audio
HALLUCINATIONS = {"thank you", "thanks for watching", "you", "bye", "thank you for watching"}


def _norm(x, target_rms=0.05):
    x = np.asarray(x, np.float32)
    r = float(np.sqrt(np.mean(x ** 2))) + 1e-9
    return np.clip(x * (target_rms / r), -1, 1).astype(np.float32)


# --------------------------------------------------------------------------- VAD
def vad_speech_ratio(x, thr=0.5) -> float:
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
    return round(float(np.mean(np.array(probs) > thr)) if probs else 0.0, 3)


# --------------------------------------------------------------------------- YAMNet
def _yamnet_labels():
    with open(os.path.join(MODEL_DIR, "yamnet_class_map.csv")) as f:
        return [r["display_name"] for r in csv.DictReader(f)]


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
def knock_periodicity(x) -> float:
    """Patterned-knock score in [0, 1]: impulsive onsets whose inter-onset intervals repeat.

    Onsets are peaks of a 10 ms envelope (>150 Hz) rising 4x above the median. The score
    counts intervals within +-10 % of the most common interval: it needs >= 3 such repeats
    to rise above 0 and saturates at 5, scaled by their share of all intervals. Random
    debris ticks or speech give low scores."""
    from scipy.signal import find_peaks
    b, a = sps.butter(2, 150 / (FS / 2), "high")
    e = np.abs(sps.lfilter(b, a, x))
    hop = FS // 100
    env = e[: len(e) // hop * hop].reshape(-1, hop).max(axis=1)
    if len(env) < 60:
        return 0.0
    pk, _ = find_peaks(env, height=4 * np.median(env), distance=8)
    if len(pk) < 4:
        return 0.0
    ioi = np.diff(pk) / 100.0
    ioi = ioi[(ioi > 0.1) & (ioi < 3.0)]
    if len(ioi) < 3:
        return 0.0
    count = max(int(np.sum(np.abs(ioi - c) <= 0.1 * c)) for c in ioi)
    return round(float(np.clip((count - 2) / 3, 0, 1) * count / len(ioi)), 3)


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


def classify(x, vad=None):
    fr, labels = yamnet_scores(x)
    agg = 0.5 * fr.mean(axis=0) + 0.5 * fr.max(axis=0)
    g = {c: float(agg[idx].max()) for c, idx in GROUPS.items()}
    heur = {"knock_periodicity": knock_periodicity(x), "hum_harmonicity": hum_harmonicity(x)}
    g["knocking"] = max(g["knocking"], 0.8 * heur["knock_periodicity"])
    g["engine"] = max(g["engine"], float(np.clip((heur["hum_harmonicity"] - 0.15) / 0.35, 0, 1)))
    tot = sum(g.values()) + 1e-9
    scores = {c: round(g[c] / tot, 3) for c in CLASSES}
    top = [(labels[i], round(float(agg[i]), 3)) for i in np.argsort(-agg)[:3]]
    fg = {c: v for c, v in scores.items() if c != "noise"}       # background only if nothing else
    if vad is not None and vad < 0.1:      # VAD gate: YAMNet "speech" with no voice activity is weak
        alt = {c: v for c, v in fg.items() if c not in ("voice", "shout")}
        if max(alt.values()) >= 0.15:
            fg = alt
    cls = max(fg, key=fg.get) if max(fg.values()) >= 0.15 else "noise"
    return cls, scores, heur, top


# --------------------------------------------------------------------------- ASR
def transcribe(x, model="base.en") -> str:
    key = "asr:" + model
    if key not in _cache:
        from faster_whisper import WhisperModel
        _cache[key] = WhisperModel(model, device="cpu", compute_type="int8", cpu_threads=2,
                                   download_root=os.path.join(MODEL_DIR, "whisper"))
    segs, _ = _cache[key].transcribe(_norm(x, 0.1), language="en", beam_size=1, temperature=0.0,
                                     condition_on_previous_text=False, vad_filter=False)
    words = [s.text.strip() for s in segs if not (s.no_speech_prob > 0.6 and s.avg_logprob < -1.0)]
    text = re.sub(r"\s+", " ", " ".join(words)).strip()
    if text.lower().strip(" .!") in HALLUCINATIONS:
        return ""
    return text
