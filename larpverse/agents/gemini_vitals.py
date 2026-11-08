"""Gemini reads the radar BREATHING WAVEFORM and gives one combined priority (breathing + acoustic distress).

    from agents.gemini_vitals import analyse_breathing
    g = analyse_breathing(displacement_mm, fs, dsp_vitals=v, audio=clip, mode="auto")
    # {"pattern", "rate_bpm", "confidence", "priority", "reason", "by", ...}

The slow-time chest displacement is block-averaged to ~4 Hz, rounded and capped at 240 samples (60 s), and
sent as a numeric series with the DSP's own rate estimate as a hint and, if given, the acoustic summary for
the same pile (class, distress cues, language, transcript snippet). Gemini answers in JSON mode:
pattern in PATTERNS, rate_bpm, confidence 0-1, priority 0-100, reason (one plain sentence that says "check").

Model: GEMINI_VITALS_MODEL, else GEMINI_MODEL (llm default). One attempt, short timeout
(GEMINI_VITALS_TIMEOUT_S, default 15 s), through agents.llm.call_json (record/replay cache; LLM_REPLAY=1 never touches the
network). Any failure or invalid answer (pattern outside the enum, priority out of range, forbidden wording)
-> the deterministic fallback, marked ``by: "fallback"``: the DSP pattern mapped to the enum and
priority = 100 x clip(dig_score x (1 + 0.5 x audio urgency), 0, 1).

Only measured values go into the prompt: the waveform, the DSP rate and SQI, and the audio summary fields named
above (whitelisted). Never truth, labels or scenario fields. An aid for rescuers, never a diagnosis.
"""
from __future__ import annotations

import math
import os
import re
import threading

import numpy as np

from . import llm

PROMPT_VERSION = "gemini-vitals-v1"
PATTERNS = ("deep_slow", "shallow_rapid", "regular_calm", "irregular_pauses", "unclear")
# sim/vitals.py pattern (legacy alias) / condition -> this enum
DSP_TO_PATTERN = {"slow_regular": "deep_slow", "rapid_shallow": "shallow_rapid", "regular_calm": "regular_calm",
                  "irregular_with_pauses": "irregular_pauses", "unclear": "unclear",
                  "unconscious_slow": "deep_slow", "distressed_tachypnea": "shallow_rapid", "calm": "regular_calm",
                  "irregular_apneic": "irregular_pauses", "unknown": "unclear"}
FALLBACK_REASON = {
    "deep_slow": "slow, deep breathing; could be deep rest or reduced consciousness, call out and check responsiveness",
    "shallow_rapid": "fast, shallow breathing; possible pain or distress, check first",
    "regular_calm": "calm, regular breathing; stable for now, re-check later",
    "irregular_pauses": "irregular breathing with pauses; possible airway problem, check first",
    "unclear": "breathing pattern unclear from this scan; rescan or check on arrival",
}
TARGET_HZ, MAX_SAMPLES = 4.0, 240
TIMEOUT_S = float(os.environ.get("GEMINI_VITALS_TIMEOUT_S", "15"))
AUDIO_FIELDS = ("class", "distress", "language", "transcript", "urgency", "human")

SYSTEM = """You support a search-and-rescue team. A radar on a drone measured the chest-wall motion of a
possible person buried under rubble. You get the slow-time displacement series (mm, about 4 samples per
second), the radar DSP's own breathing-rate estimate (a hint, may be wrong) and, when available, what the
microphones heard from the same rubble pile. Read the breathing waveform yourself.

Answer with JSON only:
- pattern: one of deep_slow (slow, large regular breaths, under about 11/min), shallow_rapid (fast, small
  breaths, over about 24/min), regular_calm (regular, about 11-22/min), irregular_pauses (irregular timing or
  pauses over 10 s), unclear (no readable breathing).
- rate_bpm: breaths per minute you count in the series (null if unclear).
- confidence: 0-1, how sure you are of the pattern.
- priority: 0-100, how urgently rescuers should reach this person first, combining the breathing pattern and
  the audio distress (shallow_rapid or irregular_pauses with distress cues = highest; regular_calm with no
  distress = lowest; deep_slow = high because the person may be unresponsive).
- reason: ONE short sentence in plain words for the rescue operator. It must say "check". Describe what the
  breathing looks like and what it MAY mean. Never say "diagnosis" or "confirmed", never state an age, never
  state an injury as fact (say "possible")."""

SCHEMA = {"type": "object", "properties": {
    "pattern": {"type": "string", "enum": list(PATTERNS)},
    "rate_bpm": {"type": "number", "nullable": True},
    "confidence": {"type": "number"},
    "priority": {"type": "number"},
    "reason": {"type": "string"}},
    "required": ["pattern", "rate_bpm", "confidence", "priority", "reason"], "additionalProperties": False}

_BANNED = re.compile(r"diagnos|confirm|\byears?[ -]old\b|\belderly\b|\binfant\b|\bbaby\b|\btoddler\b|\bchild\b|"
                     r"\bkid\b|\bteen|\baged?\b", re.I)
_INJURY = re.compile(r"\binjur|\bbleed|\bfractur|\bbroken\b|\bcrush", re.I)
_HEDGE = re.compile(r"\bpossibl|\bmay\b|\bmight\b|\bcould\b|\blikely\b|\bif\b|\bcheck", re.I)
_env_lock = threading.Lock()


# --------------------------------------------------------------------------- waveform
def downsample(displacement, fs, target_hz=TARGET_HZ, max_samples=MAX_SAMPLES, decimals=3):
    """Block-average a slow-time series to ~target_hz, remove its mean, round, keep the LAST max_samples.
    Returns (list of floats, actual sample rate). Non-finite samples are dropped first."""
    x = np.asarray(displacement, float).ravel()
    x = x[np.isfinite(x)]
    fs = float(fs) if fs and fs > 0 else target_hz
    if x.size == 0:
        return [], fs
    step = max(1, int(round(fs / target_hz)))
    n = x.size // step
    y = x[:n * step].reshape(n, step).mean(axis=1) if n else x[:1]
    y = y[-max_samples:]
    y = y - y.mean()
    return [round(float(v), decimals) for v in y], fs / step


# --------------------------------------------------------------------------- audio summary
def audio_summary(audio):
    """Whitelisted, short audio summary for the prompt (no ids, no truth, no positions)."""
    if not isinstance(audio, dict):
        return None
    out = {}
    cls = audio.get("class") or audio.get("sound_class") or audio.get("sound_type")
    if cls:
        out["class"] = str(cls)[:40]
    dist = audio.get("distress")
    if isinstance(dist, (list, tuple)) and dist:
        out["distress_cues"] = [str(d)[:40] for d in dist[:6]]
    lang = audio.get("language") or audio.get("lang")
    if isinstance(lang, dict):
        lang = lang.get("name") or lang.get("code")
    if lang:
        out["language"] = str(lang)[:30]
    tr = audio.get("translation_en") or audio.get("transcript")
    if tr:
        out["transcript_snippet"] = str(tr).strip()[:160]
    sc = audio.get("scores") if isinstance(audio.get("scores"), dict) else audio
    for k in ("urgency", "human"):
        v = sc.get(k)
        if isinstance(v, (int, float)) and math.isfinite(v):
            out[k] = round(float(v), 2)
    return out or None


def audio_urgency(audio) -> float:
    if not isinstance(audio, dict):
        return 0.0
    sc = audio.get("scores") if isinstance(audio.get("scores"), dict) else audio
    u = sc.get("urgency")
    return float(min(1.0, max(0.0, u))) if isinstance(u, (int, float)) and math.isfinite(u) else 0.0


# --------------------------------------------------------------------------- prompt
def build_prompt(series, fs_ds, dsp_vitals=None, audio=None):
    v = dsp_vitals if isinstance(dsp_vitals, dict) else {}
    lines = [f"[{PROMPT_VERSION}]",
             f"Chest displacement series: {len(series)} samples at {fs_ds:.2f} Hz "
             f"({len(series) / fs_ds:.0f} s), mm, mean removed:",
             "[" + ", ".join(f"{s:g}" for s in series) + "]"]
    rate = v.get("rate_bpm")
    lines.append(f"Radar DSP rate estimate (hint): {rate}/min" if isinstance(rate, (int, float)) else
                 "Radar DSP rate estimate: none")
    sqi = v.get("sqi")
    if isinstance(sqi, (int, float)):
        lines.append(f"Radar signal quality 0-1: {sqi}")
    a = audio_summary(audio)
    lines.append("Audio from the same pile: " + (", ".join(f"{k}: {a[k]}" for k in a) if a else "none heard"))
    return "\n".join(lines)


# --------------------------------------------------------------------------- validation
def validate(o) -> bool:
    if not isinstance(o, dict) or o.get("pattern") not in PATTERNS:
        return False
    p = o.get("priority")
    if isinstance(p, bool) or not isinstance(p, (int, float)) or not math.isfinite(p) or not 0 <= p <= 100:
        return False
    c = o.get("confidence")
    if isinstance(c, bool) or not isinstance(c, (int, float)) or not 0 <= c <= 1:
        return False
    r = o.get("rate_bpm")
    if r is not None and (isinstance(r, bool) or not isinstance(r, (int, float)) or not 0 <= r <= 90):
        return False
    reason = o.get("reason")
    if not isinstance(reason, str) or not reason.strip() or len(reason) > 300:
        return False
    if _BANNED.search(reason):
        return False
    if _INJURY.search(reason) and not _HEDGE.search(reason):
        return False
    return True


def _clean(o):
    reason = " ".join(o["reason"].split())
    if not re.search(r"\bcheck", reason, re.I):
        reason = reason.rstrip(". ") + "; check on arrival."
    return {"pattern": o["pattern"], "rate_bpm": None if o.get("rate_bpm") is None else round(float(o["rate_bpm"]), 1),
            "confidence": round(float(o["confidence"]), 2), "priority": int(round(float(o["priority"]))),
            "reason": reason}


# --------------------------------------------------------------------------- fallback
def fallback(dsp_vitals=None, audio=None, dig=None, error=None):
    """Deterministic: DSP pattern -> enum; priority = 100 x clip(dig_score x (1 + 0.5 x audio urgency), 0, 1).
    dig defaults to sim.vitals.dig_score(v, 1.0) (the triage weight), else 0.55 ('assess')."""
    v = dsp_vitals if isinstance(dsp_vitals, dict) else {}
    nonhuman = v.get("source") not in (None, "human_breathing") or v.get("triage_hint") == "reject"
    pattern = "unclear" if nonhuman else DSP_TO_PATTERN.get(v.get("condition") or v.get("pattern"), "unclear")
    if dig is None:
        try:
            from sim.vitals import dig_score
            dig = dig_score(v or None, 1.0)
        except Exception:                                   # noqa: BLE001
            dig = 0.55
    u = audio_urgency(audio)
    pri = int(round(100 * min(1.0, max(0.0, float(dig) * (1.0 + 0.5 * u)))))
    reason = FALLBACK_REASON[pattern]
    if u >= 0.5:
        reason = reason.replace("; ", "; distress heard nearby; ", 1)
    rate = v.get("rate_bpm") if pattern != "unclear" else None
    return {"pattern": pattern, "rate_bpm": rate if isinstance(rate, (int, float)) else None,
            "confidence": round(float(v.get("pattern_conf") or 0.0), 2), "priority": pri,
            "reason": reason[0].upper() + reason[1:] + ".", "by": "fallback", "model": None,
            "prompt": PROMPT_VERSION, "error": error, "audio_used": bool(audio_summary(audio))}


# --------------------------------------------------------------------------- public
def analyse_breathing(displacement_mm, fs, dsp_vitals=None, audio=None, mode="auto", *, dig=None,
                      cache_path=None, timeout=None, meta=None) -> dict:
    """Combined breathing + audio priority for one ping. Never raises.

    displacement_mm  slow-time chest displacement (mm) at ``fs`` Hz (any length; downsampled to ~4 Hz, last 60 s)
    dsp_vitals       the sim/vitals.py dict for the same return (rate hint + fallback pattern), optional
    audio            the acoustic clip / analysis for the same pile (class, distress, language, transcript,
                     scores.urgency), optional
    mode             "auto" (Gemini if available, else fallback), "gemini" (same; reports the error),
                     "fallback" (never calls the network)
    dig              dig_score to use in the fallback priority (default: the DSP triage weight)
    Returns {pattern, rate_bpm, confidence, priority, reason, by: "gemini"|"fallback", model, prompt, error,
    audio_used, n_samples}."""
    try:
        series, fs_ds = downsample(displacement_mm, fs)
    except Exception:                                       # noqa: BLE001
        series, fs_ds = [], TARGET_HZ
    m = meta if meta is not None else {}
    if mode == "fallback" or len(series) < 20:
        out = fallback(dsp_vitals, audio, dig, None if mode == "fallback" else "waveform too short")
        out["n_samples"] = len(series)
        return out
    if mode == "auto" and not llm.available():
        out = fallback(dsp_vitals, audio, dig, "no LLM available")
        out["n_samples"] = len(series)
        return out
    prompt = build_prompt(series, fs_ds, dsp_vitals, audio)
    with _env_lock:                    # one attempt: no 429/5xx retries, no backup models (llm.GEMINI_FALLBACK_MODELS)
        saved = {k: os.environ.get(k) for k in ("LLM_RETRIES", "GEMINI_FALLBACK_MODELS")}
        os.environ.update(LLM_RETRIES="0", GEMINI_FALLBACK_MODELS="")
        try:
            o = llm.call_json(prompt, SCHEMA, system=SYSTEM, provider="gemini",
                              timeout=timeout or TIMEOUT_S, max_tokens=4096, temperature=0.1,
                              cache_path=cache_path, meta=m, validate=validate,
                              model=os.environ.get("GEMINI_VITALS_MODEL") or None)
        finally:
            for k, v in saved.items():
                if v is None:
                    os.environ.pop(k, None)
                else:
                    os.environ[k] = v
    if o is None or not validate(o):
        err = m.get("error") or ("invalid answer" if o is not None else "no answer")
        out = fallback(dsp_vitals, audio, dig, err)
    else:
        out = dict(_clean(o), by="gemini", model=m.get("model"), prompt=PROMPT_VERSION, error=None,
                   audio_used=bool(audio_summary(audio)), cached=bool(m.get("cached")))
    out["n_samples"] = len(series)
    return out


def priority_order_key(c):
    """Sort key for dig order: combined priority (0-100) when present, else 100 x dig_score."""
    p = c.get("priority")
    if isinstance(p, (int, float)) and not isinstance(p, bool):
        return float(p)
    s = c.get("dig_score")
    return 100.0 * float(s) if isinstance(s, (int, float)) else 0.0
