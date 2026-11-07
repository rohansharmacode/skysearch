"""Stage 2 (Gemini audio understanding) and stage 3 (per-sector prioritisation).

Stage 2: each clip that stage 1 (acoustic/novelty.py) forwarded is sent to Gemini as denoised mono 16 kHz
WAV (<= 15 s, base64 inline_data, audio/wav) through agents.llm.call_json_audio. Gemini returns: human vocal
activity, language(s) (BCP-47 + name), verbatim transcript, English translation, sound type, distress
indicators, people count, location hints, confidence and a one-line rationale. The answer is validated;
on failure or without a key the fallback is local Whisper (multilingual model if one is on disk, else
base.en) + multilingual distress keywords + the stage-1 classes, marked ``by: "fallback"``.

Stage 3: per sector (rubble pile) priority = human likelihood x urgency x direction reliability, with the
distress indicators raising urgency (can't breathe, injured, trapped, child, several people ...).
It never says a person is confirmed: every output is a candidate lead.

Integration (acoustic/run.py, owned by the pipeline builder):
    from acoustic import gemini_audio
    res = gemini_audio.analyse_sector_audio(items, mode="auto")
    gemini_audio.merge_into_state(state, res)          # adds fields to clips/piles + state["sector_tasks"]
"""
from __future__ import annotations

import io
import json
import os
import re
import sys
import time
import wave

import numpy as np

try:
    from agents import llm
except ImportError:                       # run from acoustic/ without the repo root on sys.path
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from agents import llm

from . import novelty

FS = 16000
MAX_CLIP_S = 15.0
PROMPT_VERSION = "gemini-audio-v4"
HERE = os.path.dirname(os.path.abspath(__file__))
WHISPER_DIR = os.path.join(HERE, "models", "whisper")
MULTI_WHISPER = ("small", "base", "tiny")         # preferred multilingual faster-whisper models, if on disk

SOUND_TYPES = ["speech", "call_for_help", "scream", "crying", "moaning", "coughing", "knocking_tapping",
               "animal", "machinery", "other"]
DISTRESS = ["call_for_help", "trapped", "injured", "cant_breathe", "child", "multiple_people", "elderly",
            "water_request", "weak_or_fading", "location_hint"]

SYSTEM = """You are the acoustic analyst for an urban search-and-rescue team at a collapsed building.
A drone landed on the rubble with its rotors off and recorded this clip with a microphone array. The clip
was denoised and was flagged by an anomaly detector as different from the background. It may contain a
buried person, an animal, machinery, debris or just noise. Speech can be in any language (residents,
migrant workers, caregivers and visitors) and is often faint, muffled or partial.

Report only what is audible:
- human_vocal: true only if a human voice or human body sound (speech, shouting, screaming, crying,
  moaning, coughing, breathing) is audible.
- sound_type: the dominant sound. knocking_tapping = deliberate knocks or taps (note any repeating pattern).
- languages: every language heard, as BCP-47 code and English name (e.g. cmn Mandarin, nan Taiwanese
  Hokkien, yue Cantonese, en English, id Indonesian, vi Vietnamese, fil Filipino, th Thai, ja Japanese,
  ko Korean). Identify the language from the sounds themselves; if the transcript is written in a
  language's own script or spelling (e.g. Vietnamese diacritics), the code must match it. Empty if there
  is no intelligible speech.
- transcript: verbatim, in the original language and its usual script (never a translation). Mark unclear words with [...]. Never invent
  words; "" if no intelligible speech.
- translation_en: faithful English translation of the transcript ("" if none).
- distress: only indicators supported by the audio: call_for_help, trapped, injured, cant_breathe, child,
  multiple_people, elderly, water_request, weak_or_fading, location_hint.
- people_count: number of distinct people you can hear (0 if none or unclear).
- location_hints: short English phrases about where the person is ("under the stairs", "third floor").
- confidence: 0-1 probability that a living person made the sound.
- rationale: one line, at most 25 words, citing what you heard.
Most flagged clips are NOT people: wind, debris, animals, machinery, radio or noise. Do not let the
rescue context make you hear calls for help: transcribe only words you can actually make out, leave the
transcript empty and the confidence low when the clip is short, muffled or unclear, and never fill in
plausible distress phrases. Detector notes below are hints from cheap models and may be wrong.
This is a candidate lead for responders, never a confirmation: do not say a person is confirmed."""

PROMPT = ("Analyse this rubble-field audio clip ({dur:.1f} s, 16 kHz mono, denoised). Stage-1 detector "
          "notes (cheap open models, may be wrong): {hint}. Return the JSON object.")

SCHEMA = {
    "type": "object",
    "properties": {
        "human_vocal": {"type": "boolean"},
        "sound_type": {"type": "string", "enum": SOUND_TYPES},
        "languages": {"type": "array", "items": {
            "type": "object",
            "properties": {"code": {"type": "string"}, "name": {"type": "string"}},
            "required": ["code", "name"]}},
        "transcript": {"type": "string"},
        "translation_en": {"type": "string"},
        "distress": {"type": "array", "items": {"type": "string", "enum": DISTRESS}},
        "people_count": {"type": "integer"},
        "location_hints": {"type": "array", "items": {"type": "string"}},
        "confidence": {"type": "number"},
        "rationale": {"type": "string"},
    },
    "required": ["human_vocal", "sound_type", "languages", "transcript", "translation_en", "distress",
                 "people_count", "location_hints", "confidence", "rationale"],
    "additionalProperties": False,
}

BANNED = re.compile(r"(?<!not )(?<!un)\b(confirmed|survivor found|definitely a person)\b", re.I)

# Multilingual distress keywords for the fallback (Mandarin trad/simplified, Hokkien in Han characters,
# Indonesian, Vietnamese, Tagalog, English). Matched on transcript and Whisper translation.
KEYWORDS = {
    "call_for_help": r"\bhelp\b|anyone|somebody|someone|hear me|救命|救人|救救|來人|来人|有人嗎|有人吗|有人無|"
                     r"\btolong\b|cứu|\bsaklolo\b|\btulong\b",
    "trapped": r"trapped|stuck|pinned|can'?t (get out|move)|cannot move|困住|卡住|壓住|压住|出不去|動不了|动不了|"
               r"terjebak|terjepit|tidak bisa (keluar|bergerak)|mắc kẹt|bị kẹt|\bkẹt\b|naipit",
    "injured": r"hurt|injur|bleed|blood|pain|broken|受傷|受伤|流血|好痛|骨折|\bluka\b|berdarah|\bsakit\b|"
               r"bị thương|chảy máu|\bđau\b|sugatan",
    "cant_breathe": r"can'?t breathe|cannot breathe|hard to breathe|no air|不能呼吸|無法呼吸|无法呼吸|喘不過|喘不过|"
                    r"透不過氣|透不过气|sesak|tidak bisa bernapas|không thở được|khó thở",
    "child": r"\bchild|\bkid|\bbaby|my (son|daughter)|孩子|小孩|寶寶|宝宝|囡仔|兒子|儿子|女兒|女儿|\banak\b|\bbayi\b|"
             r"em bé|con tôi|đứa trẻ|\bbata\b",
    "multiple_people": r"\bwe\b|\bus\b|two of us|three of us|my (wife|husband|mother|father|family)|我們|我们|"
                       r"兩個人|两个人|三個人|三个人|\bkami\b|dua orang|chúng tôi|\bkami\b",
    "elderly": r"grandm|grandf|old (man|woman)|阿嬤|阿公|奶奶|爺爺|爷爷|老人|nenek|kakek|\bbà\b|\bông\b",
    "water_request": r"\bwater\b|thirst|喝水|口渴|\bair minum\b|\bhaus\b|\bnước\b|khát|\btubig\b",
    "location_hint": r"under (the|a)|near (the|a)|stair|bathroom|kitchen|bedroom|floor|basement|樓梯|楼梯|浴室|"
                     r"廚房|厨房|房間|房间|樓|楼下|tangga|kamar|dapur|lantai|cầu thang|phòng|tầng",
}
SOUND_URGENCY = {"call_for_help": 0.75, "scream": 0.75, "crying": 0.6, "moaning": 0.65, "knocking_tapping": 0.55,
                 "coughing": 0.5, "speech": 0.4, "other": 0.2, "animal": 0.05, "machinery": 0.05}
DISTRESS_BOOST = {"cant_breathe": 0.3, "injured": 0.2, "trapped": 0.15, "child": 0.2, "multiple_people": 0.15,
                  "elderly": 0.1, "weak_or_fading": 0.15, "call_for_help": 0.1, "water_request": 0.05,
                  "location_hint": 0.05}
HUMAN_LEAD = 0.4


# --------------------------------------------------------------------------- audio encoding
def wav_bytes(x, fs=FS, max_s=MAX_CLIP_S) -> bytes:
    """Mono 16-bit PCM WAV. Longer clips keep their loudest ``max_s`` window. Peak-normalised to -1 dBFS."""
    x = np.asarray(x, float).ravel()
    L = int(max_s * fs)
    if len(x) > L:
        e = np.convolve(x ** 2, np.ones(fs // 10), "same")
        c = np.cumsum(np.r_[0.0, e])
        s = int(np.argmax(c[L:] - c[:-L])) if len(c) > L else 0
        x = x[s: s + L]
    pk = float(np.abs(x).max()) if len(x) else 0.0
    if pk > 0:
        x = x / pk * 0.89
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(fs)
        w.writeframes((np.clip(x, -1, 1) * 32767).astype("<i2").tobytes())
    return buf.getvalue()


# --------------------------------------------------------------------------- validation
def validate_analysis(o) -> dict | None:
    """Normalise a Gemini answer; None if it is malformed or claims a confirmed person."""
    try:
        conf = float(o["confidence"])
        if conf != conf or not (0.0 <= conf <= 1.0):
            return None
        st = str(o["sound_type"]).strip().lower()
        if st not in SOUND_TYPES:
            return None
        langs = []
        for lg in o.get("languages") or []:
            code, name = str(lg.get("code", "")).strip(), str(lg.get("name", "")).strip()
            if code and re.fullmatch(r"[A-Za-z]{2,3}(-[A-Za-z0-9]{2,8})*", code):
                langs.append({"code": code, "name": name or code})
        dist = []
        for d in o.get("distress") or []:
            d = str(d).strip().lower()
            if d in DISTRESS and d not in dist:
                dist.append(d)
        rat = str(o["rationale"]).strip()
        tr, tl = str(o.get("transcript") or "").strip(), str(o.get("translation_en") or "").strip()
        if not rat or any(BANNED.search(t) for t in (rat, tl)):
            return None
        pc = int(o.get("people_count") or 0)
        return {"human_vocal": bool(o["human_vocal"]), "sound_type": st, "languages": langs[:4],
                "transcript": tr[:600], "translation_en": tl[:600], "distress": dist,
                "people_count": max(0, min(pc, 20)),
                "location_hints": [str(h).strip()[:80] for h in (o.get("location_hints") or []) if str(h).strip()][:4],
                "confidence": round(conf, 2), "rationale": rat[:240]}
    except (KeyError, TypeError, ValueError, AttributeError):
        return None


# --------------------------------------------------------------------------- fallback (local)
def multilingual_whisper() -> str | None:
    """Name of a multilingual faster-whisper model already on disk (never downloads), else None."""
    want = os.environ.get("WHISPER_MULTI")
    names = [want] if want else list(MULTI_WHISPER)
    for n in names:
        if n and os.path.isdir(os.path.join(WHISPER_DIR, f"models--Systran--faster-whisper-{n}")):
            return n
    return None


_wcache = {}


def _whisper(name):
    if name not in _wcache:
        from faster_whisper import WhisperModel
        _wcache[name] = WhisperModel(name, device="cpu", compute_type="int8", cpu_threads=2,
                                     download_root=WHISPER_DIR)
    return _wcache[name]


def _segments_text(segs):
    words = [s.text.strip() for s in segs if s.no_speech_prob < 0.6 and s.avg_logprob > -1.0]
    return re.sub(r"\s+", " ", " ".join(words)).strip()


LANG_NAMES = {"zh": "Chinese", "en": "English", "id": "Indonesian", "vi": "Vietnamese", "tl": "Tagalog",
              "th": "Thai", "ja": "Japanese", "ms": "Malay", "ko": "Korean", "jw": "Javanese"}


def local_asr(x, model=None):
    """(transcript, [lang], lang_prob, english, model_used). Multilingual Whisper if on disk, else base.en."""
    from . import models
    name = model or multilingual_whisper()
    if name and not name.endswith(".en"):
        wm = _whisper(name)
        xn = models._norm(x, 0.1)
        segs, info = wm.transcribe(xn, beam_size=1, temperature=0.0, condition_on_previous_text=False,
                                   vad_filter=False)
        text = _segments_text(segs)
        lang = info.language if text else None
        en = text
        if text and lang != "en":
            segs2, _ = wm.transcribe(xn, task="translate", beam_size=1, temperature=0.0,
                                     condition_on_previous_text=False, vad_filter=False)
            en = _segments_text(segs2)
        return text, ([lang] if lang else []), float(info.language_probability or 0.0), en, name
    text = models.transcribe(x, "base.en")
    return text, (["en"] if text else []), (0.5 if text else 0.0), text, "base.en"


def keyword_distress(*texts):
    t = " ".join(x or "" for x in texts).lower()
    return [k for k, pat in KEYWORDS.items() if re.search(pat, t)]


def fallback_analysis(clip, cand=None, whisper=None, asr=True) -> dict:
    """Deterministic local answer in the stage-2 format. ``cand``: the stage-1 candidate (classes, VAD)."""
    c = cand or {}
    vad = float(c.get("vad_top", 0.0))
    st = c.get("sound_type") or "other"
    text, langs, lp, en, used = "", [], 0.0, "", None
    if asr and (vad >= 0.3 or c.get("label") == "likely_human" and st in ("speech", "call_for_help")):
        try:
            text, langs, lp, en, used = local_asr(np.asarray(clip, np.float32), whisper)
        except Exception as e:                                          # noqa: BLE001 (models missing)
            used = f"asr failed: {type(e).__name__}"
    dist = keyword_distress(text, en)
    human = float(c.get("human_score", 0.0))
    if text and vad >= 0.3:
        human = max(human, 0.7)
        if st in ("other", "speech"):
            st = "call_for_help" if "call_for_help" in dist else "speech"
    if st == "knocking" or (c.get("human_class") in ("Knock", "Tap", "Wood block", "Thump, thud") and not text):
        st = "knocking_tapping"
    hv = human >= 0.35 and st not in ("knocking_tapping", "animal", "machinery")
    bits = [c.get("reason") or "no stage-1 classes"]
    if text:
        bits.append(f"Whisper {used}: {len(text.split())} words" + (f", lang {langs[0]} p={lp:.2f}" if langs else ""))
    if dist:
        bits.append("keywords: " + ", ".join(dist))
    return {"human_vocal": bool(hv), "sound_type": st if st in SOUND_TYPES else "other",
            "languages": [{"code": lg, "name": LANG_NAMES.get(lg, lg)} for lg in langs],
            "transcript": text, "translation_en": en if langs and langs[0] != "en" else (text if text else ""),
            "distress": dist, "people_count": 2 if "multiple_people" in dist else (1 if hv else 0),
            "location_hints": [], "confidence": round(min(1.0, human), 2),
            "rationale": "; ".join(bits)[:240], "asr_model": used}


# --------------------------------------------------------------------------- stage 2
def _hint(cand):
    if not cand:
        return "none"
    top = ", ".join(f"{n} {s:.2f}" for n, s in cand.get("yamnet_top", [])[:3])
    return (f"label {cand.get('label')}, YAMNet top [{top}], speech-VAD {cand.get('vad_top', 0):.2f}, "
            f"novelty {cand.get('novelty_peak', 0):.1f}")


def cross_check(a, cand):
    """Code verifies the audio model: words where stage-1 VAD heard no speech, or a long transcript from a
    very short clip, cap confidence at 0.6 and are noted (hallucination guard). In place."""
    if not cand:
        return a
    notes = []
    words = len(re.findall(r"\w+", a["transcript"])) if a["transcript"] else 0
    if words and float(cand.get("vad_top", 1.0)) < 0.2:
        notes.append(f"stage-1 VAD {cand.get('vad_top', 0):.2f} heard no speech")
    dur = float(cand.get("t1", 0) - cand.get("t0", 0)) if "t1" in cand else None
    if dur is not None and dur < 1.5 and len(a["transcript"]) > 12:
        notes.append(f"long transcript from a {dur:.1f} s clip")
    if notes:
        a["confidence"] = min(a["confidence"], 0.6)
        a["cross_check"] = "; ".join(notes)
    return a


_breaker = {"fails": 0, "until": 0.0}
BREAKER_S = 600.0            # after 2 consecutive 429s (quota), skip Gemini for 10 min: fallback at once


def _breaker_open():
    return time.time() < _breaker["until"]


def _breaker_note(err):
    if err and "HTTP 429" in err:
        _breaker["fails"] += 1
        if _breaker["fails"] >= 2:
            _breaker["until"] = time.time() + BREAKER_S
    elif err is None:
        _breaker["fails"], _breaker["until"] = 0, 0.0


def audio_model():
    """GEMINI_AUDIO_MODEL overrides the model for audio calls only (e.g. gemini-3.5-flash-lite); else
    agents.llm's GEMINI_MODEL / gemini-3.5-flash. Recorded answers replay only for the same model."""
    return os.environ.get("GEMINI_AUDIO_MODEL") or None


def _thinking():
    """GEMINI_AUDIO_THINKING = low (default: ~2-3 s/clip, no thinking tokens) | high | off (model default)."""
    lv = os.environ.get("GEMINI_AUDIO_THINKING", "low").strip().lower()
    return None if lv in ("", "off", "default") else {"thinkingLevel": lv}


def analyse_clip(clip, *, cand=None, mode="auto", whisper=None, cache_path=None, fs=FS) -> dict:
    """Stage 2 for one forwarded clip -> analysis dict (stage-2 fields + by, latency_ms, error, usage).

    mode: auto (Gemini if a key or replay, else fallback) | live (Gemini; fallback only on failure) |
          replay (recorded Gemini answers only) | fallback (never call Gemini)."""
    clip = np.asarray(clip, float)
    if fs != FS:
        from scipy.signal import resample_poly
        clip = resample_poly(clip, FS, fs)
    meta, err, busy = {}, None, False
    t0 = time.time()
    use = mode in ("live", "replay") or (mode == "auto" and llm.resolve()[0] == "gemini" and llm.available())
    if use and mode != "replay" and not llm.replay_only() and _breaker_open():
        use, err, busy = False, "gemini HTTP 429: circuit open for 10 min", True
    if use:
        prev = os.environ.get("LLM_REPLAY")
        if mode == "replay":
            os.environ["LLM_REPLAY"] = "1"
        try:
            raw = llm.call_json_audio(PROMPT.format(dur=min(len(clip) / FS, MAX_CLIP_S), hint=_hint(cand)),
                                      wav_bytes(clip), SCHEMA, system=SYSTEM + f"\n[{PROMPT_VERSION}]",
                                      provider="gemini" if mode in ("live", "replay") else None,
                                      cache_path=cache_path, meta=meta, validate=validate_analysis,
                                      thinking=_thinking(), model=audio_model())
        finally:
            if mode == "replay" and prev is None:
                os.environ.pop("LLM_REPLAY", None)
            elif mode == "replay":
                os.environ["LLM_REPLAY"] = prev
        out = validate_analysis(raw) if raw is not None else None
        if not meta.get("cached"):
            _breaker_note(meta.get("error") if out is None else None)
        if out is not None:
            cross_check(out, cand)
            out.update(by="gemini", model=meta.get("model"), cached=meta.get("cached", False),
                       latency_ms=meta.get("latency_ms", 0.0), throttled_ms=meta.get("throttled_ms", 0.0), usage=meta.get("usage") or {},
                       audio_s=round(min(len(clip) / FS, MAX_CLIP_S), 2), error=None)
            return out
        err = meta.get("error") or ("invalid Gemini answer" if raw is not None else "Gemini call failed")
        busy = bool(meta.get("busy"))
    out = fallback_analysis(clip, cand, whisper)
    note = None
    if busy or (err and "429" in err):
        note = "Gemini busy — using fallback"
    elif err and use:
        note = "Gemini failed — using fallback"
    out.update(by="fallback", by_note=note, model=None, cached=False,
               latency_ms=round((time.time() - t0) * 1000, 1), usage={}, audio_s=round(len(clip) / FS, 2), error=err)
    if note:
        out["rationale"] = f"[{note}] " + out["rationale"]
    return out


# --------------------------------------------------------------------------- stage 3
def urgency_of(a) -> float:
    if not a:
        return 0.0
    u = SOUND_URGENCY.get(a["sound_type"], 0.2) + sum(DISTRESS_BOOST.get(d, 0.0) for d in a["distress"])
    if a.get("people_count", 0) >= 2 and "multiple_people" not in a["distress"]:
        u += DISTRESS_BOOST["multiple_people"]
    if a.get("location_hints"):
        u += 0.05
    return round(min(1.0, u), 2)


def human_of(a, cand) -> float:
    """Human likelihood of one forwarded event: Gemini/fallback confidence, nudged by stage 1."""
    if not a:
        return 0.0
    h = float(a["confidence"])
    if not a["human_vocal"] and a["sound_type"] in ("animal", "machinery"):
        h = min(h, 0.2)
    if a["by"] == "fallback":                   # local evidence only: blend with stage-1 human score
        h = max(h, float(cand.get("human_score", 0.0)) * 0.9)
    return round(min(1.0, h), 2)


def sector_priority(human, urgency, reliability) -> int:
    """Same shape as acoustic.agent.clip_priority so the scales agree."""
    return int(round(100 * human * (0.35 + 0.65 * urgency) * (0.45 + 0.55 * reliability)))


def _rank_sector(pid, events, reliability, association):
    fw = [e for e in events if e["forwarded"] and e.get("analysis")]
    dis = [e for e in events if not e["forwarded"]]
    rel = float(reliability)
    if association and association != "confident":
        rel = min(rel, 0.45)
    if not fw:
        status = "dismissed_nonhuman" if dis else "no_signal_not_cleared"
        tags = sorted({e["dismissed"] for e in dis})
        why = (f"Only dismissed sounds ({', '.join('dismissed: ' + t for t in tags)}); not cleared." if dis
               else "No novel sound against the background; silence is not absence: not cleared.")
        return {"pile_id": pid, "priority": 0 if dis else 8, "status": status, "human": 0.0, "urgency": 0.0,
                "reliability": round(rel, 2), "top_event": None, "languages": [], "distress": [],
                "reasons": [why], "events": len(events), "forwarded": 0,
                "dismissed": [f"dismissed: {t}" for t in tags], "by": None}
    scored = []
    for e in fw:
        a = e["analysis"]
        h, u = human_of(a, e), urgency_of(a)
        scored.append((sector_priority(h, u, rel), h, u, e))
    scored.sort(key=lambda s: -s[0])
    pr, h, u, top = scored[0]
    a = top["analysis"]
    dist = sorted({d for _, _, _, e in scored for d in e["analysis"]["distress"]})
    people = max(e["analysis"].get("people_count", 0) for _, _, _, e in scored)
    if people >= 2 and "multiple_people" not in dist:
        dist.append("multiple_people")
    n_h = sum(1 for s in scored if s[1] >= HUMAN_LEAD)
    pr = min(100, pr + 3 * max(0, n_h - 1))                     # repeated human events add a little
    langs = []
    for _, _, _, e in scored:
        for lg in e["analysis"]["languages"]:
            if lg["code"] not in [x["code"] for x in langs]:
                langs.append(lg)
    reasons = [f"{a['sound_type'].replace('_', ' ')} at {top['t0']:.1f}-{top['t1']:.1f} s "
               f"(novelty {top['novelty_peak']:.1f}, {a['by']}, confidence {a['confidence']:.2f})"]
    if a.get("translation_en"):
        reasons.append(f"says \"{_short(a['translation_en'])}\"" +
                       (f" [{', '.join(l['name'] for l in a['languages'])}]" if a["languages"] else ""))
    if dist:
        reasons.append("distress: " + ", ".join(d.replace("_", " ") for d in dist))
    if a.get("location_hints"):
        reasons.append("location: " + "; ".join(a["location_hints"]))
    reasons.append(f"direction reliability {rel:.2f}" + ("" if association in (None, "confident")
                                                          else f" ({association} direction)"))
    if n_h > 1:
        reasons.append(f"{n_h} human-like events")
    if dis:
        reasons.append("also " + ", ".join(sorted({'dismissed: ' + e['dismissed'] for e in dis})))
    status = "lead" if h >= HUMAN_LEAD else "uncertain_sound"
    return {"pile_id": pid, "priority": pr, "status": status, "human": h, "urgency": u, "reliability": round(rel, 2),
            "top_event": top["cand_id"], "languages": langs, "distress": dist, "reasons": reasons,
            "events": len(events), "forwarded": len(fw), "dismissed": sorted({f"dismissed: {e['dismissed']}" for e in dis}),
            "by": a["by"]}


def _short(t, n=60):
    t = (t or "").strip()
    return t if len(t) <= n else t[: n - 1].rsplit(" ", 1)[0] + "…"


def analyse_sector_audio(items, *, mode="auto", whisper=None, max_forward=3, cache_path=None,
                         classifier=None, denoise_method="auto", rationale=True) -> dict:
    """Stage 1 -> 2 -> 3 for the separated clips of one listen.

    items: [{"clip_id": "ac-C1", "pile_id": "C1", "drone_id": "d1",
             "audio": np.ndarray (16 kHz mono beam/clip for that sector),
             "background": np.ndarray | None (same drone's channel over the whole listen; profile source),
             "reliability": float 0-1 (direction reliability after multi-drone association),
             "association": "confident" | "ambiguous" | None}]
    Returns {"clips": {clip_id: fields to merge}, "sector_tasks": [ranked], "stage1": summary,
             "stage2": {calls, cached, fallback, errors, latency_ms, audio_s, usage}}.
    Reads no ground truth. At most ``max_forward`` events per clip go to stage 2 (highest novelty first)."""
    clips_out, by_pile, st2 = {}, {}, {"calls": 0, "cached": 0, "fallback": 0, "errors": [], "latency_ms": 0.0,
                                      "audio_s": 0.0, "usage": {}}
    s1 = {"candidates": 0, "forwarded": 0, "dismissed": {}}
    for it in items:
        r = novelty.detect(it["audio"], background=it.get("background"), drone_id=it.get("drone_id"),
                           classifier=classifier, denoise_method=denoise_method)
        evs = r["candidates"]
        groups = sorted(r["groups"], key=lambda g: -g["novelty_peak"])
        for g in groups[max_forward:]:
            g["forwarded"], g["dismissed"] = False, "over stage-2 budget"
            for e in evs:
                if e.get("group") == g["cand_id"]:
                    e["forwarded"], e["dismissed"] = False, "over stage-2 budget"
        groups = groups[:max_forward]
        for g in groups:
            a = analyse_clip(g["clip"], cand=g, mode=mode, whisper=whisper, cache_path=cache_path)
            g["analysis"] = a
            if a["by"] == "gemini":
                st2["calls"] += 0 if a["cached"] else 1
                st2["cached"] += 1 if a["cached"] else 0
            else:
                st2["fallback"] += 1
            if a.get("error"):
                st2["errors"].append(a["error"])
            st2["latency_ms"] += a["latency_ms"]
            st2["audio_s"] += a["audio_s"]
            for k, v in (a.get("usage") or {}).items():
                if isinstance(v, (int, float)):
                    st2["usage"][k] = st2["usage"].get(k, 0) + v
        s1["candidates"] += len(evs)
        s1["forwarded"] += sum(e["forwarded"] for e in evs)
        for e in evs:
            if not e["forwarded"]:
                s1["dismissed"][e["dismissed"]] = s1["dismissed"].get(e["dismissed"], 0) + 1
        units = groups + [e for e in evs if not e["forwarded"]]
        by_pile.setdefault(it["pile_id"], []).append((it, units))
        clips_out[it["clip_id"]] = _clip_fields(units, evs, r)
    tasks = []
    for pid, lst in by_pile.items():
        evs = [e for _, es in lst for e in es]
        rel = max(float(it.get("reliability", 0.5)) for it, _ in lst)
        assoc = next((it.get("association") for it, _ in lst if it.get("association")), None)
        tasks.append(_rank_sector(pid, evs, rel, assoc))
    tasks.sort(key=lambda t: (-t["priority"], t["pile_id"]))
    for i, t in enumerate(tasks):
        t["rank"] = i + 1
    rat = sector_rationale(tasks, mode, cache_path) if rationale else {"by": "template", "error": None}
    st2["rationale"] = rat
    st2["latency_ms"] = round(st2["latency_ms"], 1)
    st2["audio_s"] = round(st2["audio_s"], 2)
    return {"prompt": PROMPT_VERSION, "mode": mode, "clips": clips_out, "sector_tasks": tasks, "stage1": s1,
            "stage2": st2}


RATIONALE_SYSTEM = """You write the per-sector rationale for an urban search-and-rescue acoustic triage
board (collapsed building, Tainan 2016 case study). Code has already scored and ranked each rubble sector
from landed-drone audio: human likelihood x urgency x direction reliability. For each sector write ONE
sentence (max 30 words) a rescue team leader can act on: what was heard (in English, quoting the
translated words if any), the distress cues, how reliable the direction is, and what to do next (send
the radar drone, re-listen, or keep in queue). Use only the facts given; do not change the ranking or
the numbers; never say a person is confirmed, found or alive: these are candidate leads."""

RATIONALE_SCHEMA = {
    "type": "object",
    "properties": {"sectors": {"type": "array", "items": {
        "type": "object",
        "properties": {"pile_id": {"type": "string"}, "rationale": {"type": "string"}},
        "required": ["pile_id", "rationale"], "additionalProperties": False}}},
    "required": ["sectors"], "additionalProperties": False,
}


def sector_rationale(tasks, mode="auto", cache_path=None):
    """One-sentence rationale per sector task, written by Gemini (text call via agents.llm, recorded for
    replay) and checked by code: known pile ids only, no other sector named, no banned claims, <= 300
    chars. Missing/invalid -> template from the reasons. Sets t["rationale"], t["rationale_by"] in place."""
    for t in tasks:
        t["rationale"] = "; ".join(t["reasons"]) + "."
        t["rationale_by"] = "template"
    use = [t for t in tasks if t["status"] in ("lead", "uncertain_sound")]
    if not use or mode == "fallback":
        return {"by": "template", "error": None}
    if mode == "auto" and not (llm.resolve()[0] != "none" and llm.available()):
        return {"by": "template", "error": "no LLM"}
    facts = [{k: t[k] for k in ("rank", "pile_id", "priority", "status", "human", "urgency", "reliability",
                                "distress", "reasons")} | {"languages": [lg["name"] for lg in t["languages"]]}
             for t in use]
    meta = {}
    prev = os.environ.get("LLM_REPLAY")
    if mode == "replay":
        os.environ["LLM_REPLAY"] = "1"
    try:
        out = llm.call_json(json.dumps({"sectors": facts}, ensure_ascii=False, sort_keys=True), RATIONALE_SCHEMA,
                            system=RATIONALE_SYSTEM, cache_path=cache_path, meta=meta, timeout=40,
                            max_tokens=2048, model=audio_model() if llm.resolve()[0] == "gemini" else None,
                            validate=lambda o: bool(_valid_rationales(o, use)))
    finally:
        if mode == "replay" and prev is None:
            os.environ.pop("LLM_REPLAY", None)
    got = _valid_rationales(out, use) if out else {}
    for t in use:
        if t["pile_id"] in got:
            t["rationale"], t["rationale_by"] = got[t["pile_id"]], "gemini"
    return {"by": "gemini" if got else "template", "error": meta.get("error"), "cached": meta.get("cached"),
            "latency_ms": meta.get("latency_ms"), "model": meta.get("model")}


def _valid_rationales(o, tasks):
    ids = {t["pile_id"] for t in tasks}
    out = {}
    for r in (o or {}).get("sectors", []) if isinstance(o, dict) else []:
        try:
            pid, txt = str(r["pile_id"]).strip().upper(), str(r["rationale"]).strip()
        except (KeyError, TypeError):
            continue
        named = set(re.findall(r"\b[A-C][1-3]\b", txt))
        if pid in ids and txt and len(txt) <= 300 and not BANNED.search(txt) and named <= {pid} \
                and not re.search(r"\b(found|alive|located)\b", txt, re.I):
            out[pid] = txt
    return out


def _clip_fields(units, evs, r):
    """Fields added to one larpverse.acoustic/v1 clip. units: analysed groups + dismissed candidates."""
    fw = [e for e in units if e.get("analysis")]
    best = max(fw, key=lambda e: (e["analysis"]["confidence"], e["novelty_peak"])) if fw else None
    a = best["analysis"] if best else None
    langs = a["languages"] if a else []
    return {
        "novelty": max((e["novelty_peak"] for e in evs), default=0.0),
        "forwarded": bool(fw),
        "dismissed": sorted({f"dismissed: {e['dismissed']}" for e in units if not e["forwarded"]}),
        "language": langs[0]["code"] if langs else None,
        "languages": langs,
        "transcript_original": a["transcript"] if a else "",
        "translation": a["translation_en"] if a else "",
        "distress": a["distress"] if a else [],
        "audio_analysis": ({**{k: a.get(k) for k in ("by", "by_note", "sound_type", "human_vocal", "confidence", "people_count",
                                                 "location_hints", "rationale", "model", "error")},
                            "cross_check": a.get("cross_check"), "t0": best["t0"], "t1": best["t1"]} if a else None),
        "stage1": {"denoise": r["denoise"], "profile": r["profile"],
                   "events": [{**{k: e[k] for k in ("cand_id", "t0", "t1", "novelty_peak", "label", "reason",
                                                    "forwarded", "dismissed")}, "group": e.get("group")} for e in evs],
                   "stage2_clips": [{k: g[k] for k in ("cand_id", "t0", "t1", "members")}
                                    for g in units if g.get("members") and g.get("analysis")]},
    }


def merge_into_state(state, res, reprioritise=True):
    """Add stage-1/2/3 fields to a larpverse.acoustic/v1 dict (in place) and return it.

    clips[]  += novelty, forwarded, dismissed[], language, languages[], transcript_original, translation,
                distress[], audio_analysis{}, stage1{}
    piles[]  += audio_priority, distress[], languages[] (and, with reprioritise, priority/why/status from
                stage 3 when it ranks the pile higher; dispatch re-picked from the top lead)
    state    += sector_tasks[], gemini_audio{prompt, mode, stage1, stage2}"""
    for c in state.get("clips", []):
        f = res["clips"].get(c["clip_id"])
        if f:
            c.update(f)
    task_by = {t["pile_id"]: t for t in res["sector_tasks"]}
    for p in state.get("piles", []):
        t = task_by.get(p["pile_id"])
        if not t:
            continue
        p["audio_priority"] = t["priority"]
        p["distress"] = t["distress"]
        p["languages"] = [lg["code"] for lg in t["languages"]]
        if reprioritise and t["status"] == "lead" and t["priority"] > p["priority"]:
            p["priority"] = t["priority"]
            p["why"] = "Audio analysis: " + t.get("rationale", "; ".join(t["reasons"]) + ".")
            if p["status"] in ("no_signal_not_cleared", "dismissed_nonhuman"):
                p["status"] = "lead"
    if reprioritise and state.get("piles"):
        state["piles"].sort(key=lambda p: (-p["priority"], p["pile_id"]))
        leads = [p for p in state["piles"] if p["status"] in ("lead", "radar_ping")]
        for p in leads:
            p["status"] = "lead"
        if leads:
            top = leads[0]
            top["status"] = "radar_ping"
            t = task_by.get(top["pile_id"])
            if t and t["status"] == "lead" and t["priority"] >= top.get("priority", 0):
                state["dispatch"] = {"pile_id": top["pile_id"],
                                     "reason": f"Send the radar drone to {top['pile_id']}: highest priority "
                                               f"({top['priority']}); " + "; ".join(t["reasons"][:3]) + "."}
            elif not state.get("dispatch", {}).get("pile_id"):
                state["dispatch"] = {"pile_id": top["pile_id"], "reason": f"Send the radar drone to {top['pile_id']}: "
                                     f"highest priority ({top['priority']})."}
    state["sector_tasks"] = res["sector_tasks"]
    state["gemini_audio"] = {"prompt": res["prompt"], "mode": res["mode"], "stage1": res["stage1"],
                             "stage2": res["stage2"]}
    return state
