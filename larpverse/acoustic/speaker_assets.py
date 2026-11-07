"""Render the loudspeaker messages (sim/guidance.py templates) to WAV for the demo. Run in .venv-audio.

    .venv-audio/bin/python -m acoustic.speaker_assets                     # fixed messages (callout, reassure)
    .venv-audio/bin/python -m acoustic.speaker_assets --state dashboard/data/state.json   # + every broadcast in a run
    .venv-audio/bin/python -m acoustic.speaker_assets --engine gemini --langs nan-TW      # try Gemini TTS (opt-in)

Engines, per language:
  * Piper (open, local, MIT; voices already in acoustic/models): en_US-lessac-low, zh_CN-huayan-x_low (reads
    the Traditional-character zh-TW text with Mandarin pronunciation; low quality, tones partly lost),
    id_ID-news_tts-medium, vi_VN-vivos-x_low.
  * espeak-ng, if installed, for any language without a Piper voice (not installed on this machine).
  * Gemini TTS (opt-in, ``--engine gemini``, model GEMINI_TTS_MODEL, default gemini-2.5-flash-preview-tts) via the
    same REST style as agents/llm.py (key from .env, token bucket, backoff). Taiwanese Hokkien is not an
    officially supported TTS language: a Gemini rendering is marked ``unvetted`` and should be replaced by a
    native-speaker recording before any real use.
  * Taiwanese Hokkien (nan-TW): no open TTS voice exists here, so by default it is listed as
    ``missing: record a native speaker`` (Tai-lo script in sim.guidance.NAN_TAILO) and the drone skips it.

Output: acoustic/data/speaker/<sha1(lang, text)[:12]>.<lang>.wav (gitignored) and manifest.json
{items: {key: {lang, text, engine, path, duration_s, sample_rate, status}}}. sim/callout.Speaker reads the
manifest for real segment durations and ``audio_url`` (served by scripts/serve.py from the repo root).
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import os
import shutil
import subprocess
import sys
import time
import wave

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from sim.guidance import LANGS, MESSAGES, audio_key  # noqa: E402

OUT = os.path.join(ROOT, "acoustic", "data", "speaker")
MODELS = os.path.join(ROOT, "acoustic", "models")
PIPER = {"en": os.path.join(MODELS, "en_US-lessac-low.onnx"),
         "zh-TW": os.path.join(MODELS, "piper_multi", "zh_CN-huayan-x_low.onnx"),
         "id": os.path.join(MODELS, "piper_multi", "id_ID-news_tts-medium.onnx"),
         "vi": os.path.join(MODELS, "piper_multi", "vi_VN-vivos-x_low.onnx")}
ESPEAK = {"en": "en", "zh-TW": "cmn", "id": "id", "vi": "vi"}
GEMINI_TTS_MODEL = "gemini-2.5-flash-preview-tts"
GEMINI_LANG_HINT = {"zh-TW": "Mandarin Chinese (Taiwan)", "nan-TW": "Taiwanese Hokkien (Tâi-gí)",
                    "en": "English", "id": "Indonesian", "vi": "Vietnamese"}


def _write_wav(path, pcm16: bytes, sr: int):
    tmp = path + ".tmp"
    with wave.open(tmp, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(pcm16)
    os.replace(tmp, path)


_voices = {}


def piper_tts(lang, text):
    import numpy as np
    from piper import PiperVoice
    if lang not in PIPER or not os.path.exists(PIPER[lang]):
        return None
    v = _voices.get(lang) or _voices.setdefault(lang, PiperVoice.load(PIPER[lang]))
    chunks = list(v.synthesize(text))
    x = np.concatenate([c.audio_float_array for c in chunks]).astype(np.float32)
    sr = chunks[0].sample_rate
    x = np.concatenate([np.zeros(int(0.15 * sr), np.float32), x, np.zeros(int(0.25 * sr), np.float32)])
    x = 0.9 * x / (np.max(np.abs(x)) + 1e-9)                   # loudspeaker: normalise to near full scale
    return (x * 32767).astype("<i2").tobytes(), sr


def espeak_tts(lang, text):
    exe = shutil.which("espeak-ng") or shutil.which("espeak")
    if not exe or lang not in ESPEAK:
        return None
    r = subprocess.run([exe, "-v", ESPEAK[lang], "--stdout", text], capture_output=True, timeout=60)
    if r.returncode != 0 or not r.stdout:
        return None
    with wave.open(io.BytesIO(r.stdout)) as w:
        return w.readframes(w.getnframes()), w.getframerate()


def gemini_tts(lang, text):
    """Gemini TTS through REST (agents.llm transport: .env key, token bucket, retry/backoff). Returns
    (pcm16, 24000) or None. Not cached in agents/cache (audio); the WAV file is the cache."""
    from agents import llm
    llm.load_dotenv()
    key = os.environ.get("GEMINI_API_KEY", "").strip()
    if not key:
        print("[speaker] no GEMINI_API_KEY", file=sys.stderr)
        return None
    model = os.environ.get("GEMINI_TTS_MODEL", GEMINI_TTS_MODEL)
    body = {"contents": [{"parts": [{"text": f"Say clearly and calmly, as a rescue team loudspeaker announcement, "
                                              f"in {GEMINI_LANG_HINT[lang]}: {text}"}]}],
            "generationConfig": {"responseModalities": ["AUDIO"],
                                 "speechConfig": {"voiceConfig": {"prebuiltVoiceConfig": {"voiceName": "Kore"}}}}}
    try:
        resp = llm._post_retry(llm.GEMINI_URL.format(model=model), {"x-goog-api-key": key}, body, 90)
        part = resp["candidates"][0]["content"]["parts"][0]["inlineData"]
        pcm = base64.b64decode(part["data"])
        sr = 24000
        mt = part.get("mimeType", "")
        if "rate=" in mt:
            sr = int(mt.split("rate=")[1].split(";")[0])
        return pcm, sr
    except Exception as e:                                       # noqa: BLE001
        print(f"[speaker] gemini TTS failed for {lang}: {type(e).__name__}: {str(e)[:160]}", file=sys.stderr)
        return None


def render(lang, text, engine="auto", manifest=None, force=False):
    """Render one (lang, text); returns the manifest item."""
    key = audio_key(lang, text)
    items = manifest if manifest is not None else {}
    it = items.get(key)
    if it and it.get("path") and os.path.exists(os.path.join(OUT, os.path.basename(it["path"]))) and not force:
        return it
    order = {"auto": ["piper", "espeak"], "piper": ["piper"], "espeak": ["espeak"], "gemini": ["gemini"]}[engine]
    res, used = None, None
    for eng in order:
        res = {"piper": piper_tts, "espeak": espeak_tts, "gemini": gemini_tts}[eng](lang, text)
        if res:
            used = eng
            break
    if not res:
        it = {"lang": lang, "text": text, "engine": None, "path": None, "duration_s": None,
              "status": "missing: record a native speaker" if lang == "nan-TW" else "missing: no TTS voice"}
    else:
        pcm, sr = res
        os.makedirs(OUT, exist_ok=True)
        name = f"{key}.{lang}.wav"
        _write_wav(os.path.join(OUT, name), pcm, sr)
        status = "unvetted (Gemini TTS, Hokkien not officially supported)" if used == "gemini" and lang == "nan-TW" \
            else ("rendered (Mandarin voice zh_CN, low quality)" if lang == "zh-TW" and used == "piper" else "rendered")
        it = {"lang": lang, "text": text, "engine": used, "path": name, "sample_rate": sr,
              "duration_s": round(len(pcm) / 2 / sr, 2), "status": status}
    items[key] = it
    return it


def load(path=None):
    path = path or os.path.join(OUT, "manifest.json")
    try:
        with open(path) as f:
            return json.load(f).get("items", {})
    except (OSError, ValueError):
        return {}


def save(items):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, "manifest.json")
    with open(p + ".tmp", "w") as f:
        json.dump({"schema": "larpverse.speaker_assets/v1", "updated": time.strftime("%Y-%m-%dT%H:%M:%S"),
                   "items": items}, f, ensure_ascii=False, indent=1)
    os.replace(p + ".tmp", p)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--langs", default=",".join(LANGS))
    ap.add_argument("--engine", choices=["auto", "piper", "espeak", "gemini"], default="auto")
    ap.add_argument("--state", help="also render every broadcast text found in this state.json")
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args(argv)
    langs = [x for x in a.langs.split(",") if x]
    items = load()
    todo = [(lang, MESSAGES[k][lang]) for k in MESSAGES for lang in langs]
    if a.state:
        with open(a.state) as f:
            st = json.load(f)
        todo += [(b["lang"], b["text"]) for b in st.get("broadcasts") or [] if b.get("lang") in langs
                 and (b.get("source") != "llm_translation" or b.get("approved_by"))]
    for lang, text in dict.fromkeys(todo):
        it = render(lang, text, a.engine, items, a.force)
        print(f"{lang:7s} {it['status']:<45s} {it.get('duration_s') or '-':>6} s  {text[:60]}")
    save(items)


if __name__ == "__main__":
    main()
