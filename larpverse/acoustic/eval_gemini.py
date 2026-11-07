"""Evaluate the novelty -> Gemini audio -> sector-priority pipeline (evaluator: may read truth/labels).

    .venv-audio/bin/python -m acoustic.eval_gemini fetch                 # FLEURS clips + Piper voices (~120 MB)
    scripts/heavy.sh .venv-audio/bin/python -m acoustic.eval_gemini stage1 --bed ambient   # also: rotor, generator
    scripts/heavy.sh .venv-audio/bin/python -m acoustic.eval_gemini stage2               # fallback (no key)
    scripts/heavy.sh .venv-audio/bin/python -m acoustic.eval_gemini stage2 --live        # Gemini, once the key is set
    scripts/heavy.sh .venv-audio/bin/python -m acoustic.eval_gemini scene --seeds 3
    .venv-audio/bin/python -m acoustic.eval_gemini report

Stage 1: real open audio events (FLEURS speech in 4 languages, ESC-50 crying/cough/breathing/knock vs
dog/engine/chainsaw/helicopter/glass/fire/creak/rain/wind, scene.py knocks/debris, Piper distress phrases)
mixed 8 s into a 20 s bed at +10/0/-5 dB SNR (event active RMS vs bed RMS). Beds: ambient = scene.py's
rotors-off recipe (gusty wind + sensor noise), rotor = real DroneAudioSet drone-only recording (another
drone hovering), generator = ambient + scene.py's diesel generator. The profile is learned from the
first 6 s. Recall = a forwarded candidate overlaps a human event; false-forward = a forwarded candidate
overlaps a non-human event, or any forwarded candidate on a background-only bed.
Stage 2: FLEURS + distress phrases, clean and mixed, through stage 1 then analyse_clip: language ID,
transcript CER, translation token-F1 vs the parallel English FLEURS sentence, distress-tag recall,
latency, tokens and cost. Results: eval/acoustic_gemini/*.json (gitignored).
"""
from __future__ import annotations

import os

os.environ.setdefault("OMP_NUM_THREADS", "2")

import argparse  # noqa: E402
import csv  # noqa: E402
import glob  # noqa: E402
import io  # noqa: E402
import json  # noqa: E402
import re  # noqa: E402
import sys  # noqa: E402
import tarfile  # noqa: E402
import time  # noqa: E402
import urllib.request  # noqa: E402

import numpy as np  # noqa: E402
from scipy import signal as sps  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

FS = 16000
REAL = os.path.join(ROOT, "data", "audio_real")
FLEURS_DIR = os.path.join(REAL, "fleurs")
ESC_DIR = os.path.join(REAL, "esc50")
DAS_DIR = os.path.join(REAL, "das")
OUT_DIR = os.path.join(ROOT, "eval", "acoustic_gemini")
VOICE_DIR = os.path.join(HERE, "models", "piper_multi")
LANGS = {"cmn_hans_cn": "zh", "en_us": "en", "id_id": "id", "vi_vn": "vi"}
HF = "https://huggingface.co/datasets/google/fleurs/resolve/main/data/{lang}/{path}"
PIPER = "https://huggingface.co/rhasspy/piper-voices/resolve/main/{p}"
VOICES = {"zh": "zh/zh_CN/huayan/x_low/zh_CN-huayan-x_low", "vi": "vi/vi_VN/vivos/x_low/vi_VN-vivos-x_low",
          "id": "id/id_ID/news_tts/medium/id_ID-news_tts-medium"}
# distress phrases (written by us) and the tags a good analyser should return
PHRASES = [
    ("en", "Help! I'm trapped under the stairs and my leg is hurt.", ["call_for_help", "trapped", "injured", "location_hint"]),
    ("en", "Can anyone hear me? There are two of us. My child can't breathe.", ["call_for_help", "multiple_people", "child", "cant_breathe"]),
    ("zh", "救命！我被困在楼梯下面，我的腿受伤了。", ["call_for_help", "trapped", "injured", "location_hint"]),
    ("zh", "有人吗？我们这里有两个人，孩子不能呼吸。", ["call_for_help", "multiple_people", "child", "cant_breathe"]),
    ("vi", "Cứu tôi với! Tôi bị kẹt dưới cầu thang, chân tôi bị thương.", ["call_for_help", "trapped", "injured", "location_hint"]),
    ("vi", "Có ai không? Chúng tôi có hai người, con tôi khó thở.", ["call_for_help", "multiple_people", "child", "cant_breathe"]),
    ("id", "Tolong! Saya terjebak di bawah tangga, kaki saya terluka.", ["call_for_help", "trapped", "injured", "location_hint"]),
    ("id", "Ada orang? Kami berdua di sini, anak saya tidak bisa bernapas.", ["call_for_help", "multiple_people", "child", "cant_breathe"]),
]
ESC_HUMAN = {20: "crying", 24: "cough", 23: "breathing", 30: "knock"}
ESC_OTHER = {0: "dog", 44: "engine", 41: "chainsaw", 40: "helicopter", 39: "glass", 12: "fire", 33: "creak",
             10: "rain", 16: "wind"}


# --------------------------------------------------------------------------- fetch
def _get(url, timeout=60):
    return urllib.request.urlopen(urllib.request.Request(url, headers={"User-Agent": "larpverse-eval"}), timeout=timeout)


def _tsv(lang, split="dev"):
    with _get(HF.format(lang=lang, path=f"{split}.tsv")) as r:
        rows = list(csv.reader(io.StringIO(r.read().decode("utf-8")), delimiter="\t", quoting=csv.QUOTE_NONE))
    return [{"id": r[0], "file": r[1], "raw": r[2], "norm": r[3], "n": int(r[5]), "gender": r[6]} for r in rows if len(r) >= 7]


def fetch_fleurs(n=3, split="dev", max_s=11.0):
    """Stream the first matching wavs out of each language's dev tarball (stops early: a few MB each)."""
    os.makedirs(FLEURS_DIR, exist_ok=True)
    en = {r["id"]: r["raw"] for r in _tsv("en_us", split)}
    index, total = [], 0
    for lang in LANGS:
        rows = {r["file"]: r for r in _tsv(lang, split) if r["n"] <= max_s * FS and r["id"] in en}
        d = os.path.join(FLEURS_DIR, lang)
        os.makedirs(d, exist_ok=True)
        got, seen_ids = 0, set()
        with _get(HF.format(lang=lang, path=f"audio/{split}.tar.gz"), timeout=120) as resp:
            tf = tarfile.open(fileobj=resp, mode="r|gz")
            for m in tf:
                base = os.path.basename(m.name)
                if not m.isfile() or base not in rows or not re.fullmatch(r"\d+\.wav", base):
                    continue
                r = rows[base]
                if r["id"] in seen_ids:
                    continue
                data = tf.extractfile(m).read()          # bytes only; never tar.extract()
                with open(os.path.join(d, base), "wb") as fh:
                    fh.write(data)
                total += len(data)
                seen_ids.add(r["id"])
                index.append({"lang": lang, "code": LANGS[lang], "wav": f"{lang}/{base}", "sentence_id": r["id"],
                              "transcript": r["raw"], "en_reference": en[r["id"]], "gender": r["gender"],
                              "duration_s": round(r["n"] / FS, 2)})
                got += 1
                if got >= n:
                    break
        print(f"{lang}: {got} clips")
    with open(os.path.join(FLEURS_DIR, "index.json"), "w") as fh:
        json.dump(index, fh, indent=1, ensure_ascii=False)
    with open(os.path.join(FLEURS_DIR, "SOURCE.md"), "w") as fh:
        fh.write(SOURCE_MD.format(n=n, split=split, mb=total / 1e6, langs=", ".join(LANGS)))
    print(f"FLEURS: {len(index)} clips, {total / 1e6:.1f} MB written to {FLEURS_DIR}")


def fetch_voices():
    os.makedirs(VOICE_DIR, exist_ok=True)
    for code, p in VOICES.items():
        for ext in (".onnx", ".onnx.json"):
            dst = os.path.join(VOICE_DIR, os.path.basename(p) + ext)
            if not os.path.exists(dst):
                with _get(PIPER.format(p=p + ext), timeout=300) as r, open(dst + ".part", "wb") as fh:
                    fh.write(r.read())
                os.replace(dst + ".part", dst)
        print(f"piper voice {code}: {os.path.basename(p)}")


SOURCE_MD = """# FLEURS clips (evaluation only)

- Source: Google FLEURS, https://huggingface.co/datasets/google/fleurs (Conneau et al. 2022,
  "FLEURS: Few-shot Learning Evaluation of Universal Representations of Speech", arXiv:2205.12446).
- Licence: CC BY 4.0. Attribution: Google; read FLoRes-101 sentences.
- What is here: the first {n} {split}-split clips (<= 11 s) per language ({langs}) streamed from each
  language's `audio/{split}.tar.gz` (download stopped after those members, {mb:.1f} MB written), plus
  `index.json`: sentence id, original transcript, and the parallel English sentence (FLEURS is n-way
  parallel, so the en_us transcript of the same sentence id is the reference translation).
- Fetched by `python -m acoustic.eval_gemini fetch`. Read news-style speech, not distress calls; the
  distress phrases in the evaluation are our own sentences voiced by Piper TTS (MIT; voices
  zh_CN-huayan-x_low, vi_VN-vivos-x_low, id_ID-news_tts-medium, en_US-lessac-low; see each voice's
  MODEL_CARD on huggingface.co/rhasspy/piper-voices for its dataset licence).
"""


# --------------------------------------------------------------------------- audio helpers
def _read(path):
    import soundfile as sf
    x, fs = sf.read(path, dtype="float64", always_2d=False)
    if x.ndim > 1:
        x = x.mean(axis=1)
    if fs != FS:
        x = sps.resample_poly(x, FS, fs)
    return x


def _trim(x, rel=0.05, pad=0.15):
    e = np.convolve(np.abs(x), np.ones(400) / 400, "same")
    idx = np.where(e > rel * e.max())[0] if e.max() > 0 else []
    if len(idx) == 0:
        return x
    p = int(pad * FS)
    return x[max(0, idx[0] - p): idx[-1] + p]


def _active_rms(x):
    e = np.convolve(x ** 2, np.ones(320) / 320, "same")
    m = e > 0.05 * e.max()
    return float(np.sqrt(np.mean(x[m] ** 2))) if m.any() else float(np.sqrt(np.mean(x ** 2)) + 1e-12)


def tts_multi(code, text):
    """Piper TTS in zh/vi/id (voices from fetch) or en (scene.py's lessac voice). Cached wav, None if missing."""
    import hashlib
    import soundfile as sf
    from acoustic import scene
    if code == "en":
        return scene.tts(text).astype(float)
    d = os.path.join(scene.DATA_DIR, "tts")
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, f"{code}-" + hashlib.sha1(text.encode()).hexdigest()[:12] + ".wav")
    if os.path.exists(path):
        return _read(path)
    onnx = os.path.join(VOICE_DIR, os.path.basename(VOICES[code]) + ".onnx")
    if not os.path.exists(onnx):
        return None
    from piper import PiperVoice
    v = PiperVoice.load(onnx)
    ch = list(v.synthesize(text))
    x = np.concatenate([c.audio_float_array for c in ch]).astype(np.float64)
    sr = ch[0].sample_rate
    sf.write(path, x, sr)
    return _read(path)


def beds(kind, n, rng):
    """A background bed of n samples: ambient | rotor | generator."""
    from acoustic import scene
    t = np.arange(n) / FS
    bw, aw = sps.butter(2, 120 / (FS / 2))
    gust = 1 + 0.6 * np.sin(2 * np.pi * rng.uniform(0.05, 0.2) * t + rng.uniform(0, 6.28))
    amb = scene._rms_norm(sps.lfilter(bw, aw, rng.normal(0, 1, n))) * gust * 3 * 4e-3 + rng.normal(0, 4e-3, n)
    if kind == "ambient":
        return amb
    if kind == "generator":
        return amb + scene._rms_norm(scene.generator_hum(rng, n / FS)) * 8e-3
    if kind == "rotor":
        f = sorted(glob.glob(os.path.join(DAS_DIR, "drone-only-recordings__*throttle-low*File1.wav")))[0]
        x = _read(f)
        s = int(rng.integers(0, len(x) - n))
        return x[s:s + n] / (np.sqrt(np.mean(x[s:s + n] ** 2)) + 1e-12) * 0.02
    raise ValueError(kind)


def mix(bed, ev, snr_db, at_s=8.0):
    x = bed.copy()
    s = int(at_s * FS)
    ev = ev[: len(x) - s - FS]
    g = np.sqrt(np.mean(bed ** 2)) * 10 ** (snr_db / 20) / _active_rms(ev)
    x[s:s + len(ev)] += g * ev
    return x, (at_s, at_s + len(ev) / FS)


def events(include_tts=True):
    """[(name, group, fine, signal)] group: human | other."""
    from acoustic import scene
    rng = np.random.default_rng(11)
    out = []
    idx = _fleurs_index()
    for r in idx:
        out.append((f"fleurs:{r['wav']}", "human", f"speech_{r['code']}", _read(os.path.join(FLEURS_DIR, r["wav"]))[: 8 * FS]))
    files = sorted(glob.glob(os.path.join(ESC_DIR, "*_16k.wav")))
    for tgt, fine in list(ESC_HUMAN.items()) + list(ESC_OTHER.items()):
        sel = [f for f in files if re.search(rf"-{tgt}_16k\.wav$", f)][:3 if tgt in (0, 20, 23, 24, 30, 44) else 2]
        grp = "human" if tgt in ESC_HUMAN else "other"
        for f in sel:
            out.append((f"esc:{os.path.basename(f)}", grp, fine, _trim(_read(f))))
    out += [(f"scene:knocking{i}", "human", "knock_pattern", scene.knocking(rng)) for i in range(2)]
    out += [(f"scene:debris{i}", "other", "debris", scene.debris(rng)) for i in range(2)]
    if include_tts:
        for i, (code, text, _) in enumerate(PHRASES):
            x = tts_multi(code, text)
            if x is not None:
                out.append((f"tts:{code}{i}", "human", f"distress_{code}", x))
    return out


def _fleurs_index():
    p = os.path.join(FLEURS_DIR, "index.json")
    return json.load(open(p)) if os.path.exists(p) else []


# --------------------------------------------------------------------------- stage 1
def _overlap(c, win, slack=0.3):
    return c["t1"] > win[0] - slack and c["t0"] < win[1] + slack


def run_stage1(bed_kind, snrs, profile="first6", limit=None):
    from acoustic import novelty
    rng = np.random.default_rng({"ambient": 1, "rotor": 2, "generator": 3}[bed_kind])
    evs = events()[: limit or None]
    rows = []
    t_all = time.time()
    n = 20 * FS
    for name, grp, fine, sig in evs:
        for snr in snrs:
            x, win = mix(beds(bed_kind, n, rng), sig, snr)
            t0 = time.time()
            r = novelty.detect(x, bg_seconds=6.0 if profile == "first6" else None)
            hits = [c for c in r["candidates"] if _overlap(c, win)]
            rows.append({"item": name, "group": grp, "fine": fine, "bed": bed_kind, "snr_db": snr,
                         "detected": bool(hits), "forwarded": any(c["forwarded"] for c in hits),
                         "labels": [c["label"] for c in hits], "dismissed": [c["dismissed"] for c in hits if c["dismissed"]],
                         "reasons": [c["reason"] for c in hits][:2],
                         "spurious_forwarded": sum(c["forwarded"] for c in r["candidates"] if not _overlap(c, win)),
                         "novelty_peak": max([c["novelty_peak"] for c in hits], default=0.0),
                         "seconds": round(time.time() - t0, 2)})
    for k in range(6):                                     # background-only beds
        x = beds(bed_kind, n, rng)
        r = novelty.detect(x, bg_seconds=6.0 if profile == "first6" else None)
        rows.append({"item": f"bed{k}", "group": "background", "fine": "background", "bed": bed_kind, "snr_db": None,
                     "detected": bool(r["candidates"]), "forwarded": any(c["forwarded"] for c in r["candidates"]),
                     "labels": [c["label"] for c in r["candidates"]],
                     "dismissed": [c["dismissed"] for c in r["candidates"] if c["dismissed"]],
                     "reasons": [c["reason"] for c in r["candidates"]][:2], "spurious_forwarded": 0,
                     "novelty_peak": max([c["novelty_peak"] for c in r["candidates"]], default=0.0), "seconds": 0})
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, f"stage1_{bed_kind}_{profile}.json")
    json.dump(rows, open(path, "w"), indent=1)
    print(f"stage1 {bed_kind}/{profile}: {len(rows)} rows in {time.time() - t_all:.0f} s -> {path}")
    report_stage1([path])


def report_stage1(paths):
    rows = [r for p in paths for r in json.load(open(p))]
    if not rows:
        return
    print(f"\n{'bed':10} {'snr':>4}  {'human recall':>13} {'detected':>9}  {'false-fwd other':>15}  {'bg false-fwd':>12}")
    keys = sorted({(r["bed"], r["snr_db"]) for r in rows if r["snr_db"] is not None}, key=lambda k: (k[0], -k[1]))
    for bed, snr in keys:
        h = [r for r in rows if r["bed"] == bed and r["snr_db"] == snr and r["group"] == "human"]
        o = [r for r in rows if r["bed"] == bed and r["snr_db"] == snr and r["group"] == "other"]
        b = [r for r in rows if r["bed"] == bed and r["group"] == "background"]
        print(f"{bed:10} {snr:>+4}  {_pct(h, 'forwarded'):>13} {_pct(h, 'detected'):>9}  {_pct(o, 'forwarded'):>15}  "
              f"{_pct(b, 'forwarded'):>12}")
    print("\nper fine class (all beds/SNRs): forwarded / detected")
    for fine in sorted({r["fine"] for r in rows}):
        rr = [r for r in rows if r["fine"] == fine]
        print(f"  {fine:16} {rr[0]['group']:10} fwd {_pct(rr, 'forwarded'):>12}  det {_pct(rr, 'detected'):>12}")


def _pct(rows, k):
    if not rows:
        return "-"
    n = sum(bool(r[k]) for r in rows)
    return f"{n}/{len(rows)} {100 * n / len(rows):.0f}%"


# --------------------------------------------------------------------------- stage 2
def _norm_text(t, code):
    t = (t or "").lower()
    t = re.sub(r"[\s\W_]+", " " if code != "zh" else "", t, flags=re.UNICODE).strip()
    return t


def cer(hyp, ref, code):
    h, r = _norm_text(hyp, code), _norm_text(ref, code)
    if not r:
        return None
    d = np.arange(len(h) + 1)
    for i, rc in enumerate(r, 1):
        prev, d[0] = d.copy(), i
        for j, hc in enumerate(h, 1):
            d[j] = min(prev[j] + 1, d[j - 1] + 1, prev[j - 1] + (rc != hc))
    return round(float(d[len(h)]) / len(r), 3)


def token_f1(hyp, ref):
    h = re.findall(r"[a-z0-9']+", (hyp or "").lower())
    r = re.findall(r"[a-z0-9']+", (ref or "").lower())
    if not h or not r:
        return 0.0
    from collections import Counter
    c = sum((Counter(h) & Counter(r)).values())
    if c == 0:
        return 0.0
    p, q = c / len(h), c / len(r)
    return round(2 * p * q / (p + q), 3)


def _lang_ok(langs, code):
    if not langs:
        return False
    c = langs[0]["code"].lower().split("-")[0]
    want = {"zh": ("zh", "cmn", "yue", "nan"), "en": ("en",), "id": ("id", "ind", "ms"), "vi": ("vi", "vie")}[code]
    return c in want


def run_stage2(mode, snrs, limit=None, whisper=None, start=0):
    from acoustic import gemini_audio as ga
    from acoustic import novelty
    if mode == "live":
        os.environ.setdefault("GEMINI_RPM", "6")       # leave headroom for a demo on the same key
    items = []
    for r in _fleurs_index():
        items.append({"item": f"fleurs:{r['wav']}", "code": r["code"], "ref": r["transcript"], "en_ref": r["en_reference"],
                      "tags": None, "sig": _read(os.path.join(FLEURS_DIR, r["wav"]))})
    for i, (code, text, tags) in enumerate(PHRASES):
        x = tts_multi(code, text)
        if x is not None:
            items.append({"item": f"tts:{code}{i}", "code": code, "ref": text,
                          "en_ref": PHRASES[i % 2][1] if code != "en" else None, "tags": tags, "sig": x})
    rng = np.random.default_rng(5)
    beds_ = {id(it): beds("ambient", 20 * FS, rng) for it in items}       # same bed per item whatever the slice
    items = items[start: (start + limit) if limit else None]
    rows = []
    for it in items:
        for snr in snrs:
            if snr == "clean":
                clip, cand, fwd = it["sig"], {"label": "likely_human", "vad_top": 1.0, "human_score": 0.9,
                                               "sound_type": "speech", "reason": "clean clip (stage 1 skipped)"}, True
            else:
                x, win = mix(beds_[id(it)], it["sig"][: 11 * FS], float(snr))
                r = novelty.detect(x, bg_seconds=6.0)
                hits = [g for g in r["groups"] if _overlap(g, win)]
                fwd = bool(hits)
                if not fwd:
                    rows.append({"item": it["item"], "code": it["code"], "snr": snr, "forwarded": False})
                    continue
                cand = max(hits, key=lambda g: min(g["t1"], win[1]) - max(g["t0"], win[0]))
                clip = cand["clip"]
            a = ga.analyse_clip(clip, cand=cand, mode=mode, whisper=whisper)
            row = {"item": it["item"], "code": it["code"], "snr": snr, "forwarded": fwd, "by": a["by"],
                   "error": a.get("error"), "lang_pred": [lg["code"] for lg in a["languages"]],
                   "lang_ok": _lang_ok(a["languages"], it["code"]), "transcript": a["transcript"],
                   "translation": a["translation_en"], "cer": cer(a["transcript"], it["ref"], it["code"]),
                   "sound_type": a["sound_type"], "human_vocal": a["human_vocal"], "confidence": a["confidence"],
                   "distress": a["distress"], "latency_ms": a["latency_ms"], "cached": a.get("cached", False),
                   "cross_check": a.get("cross_check"),
                   "usage": a.get("usage") or {}, "audio_s": a["audio_s"]}
            ref_en = it["en_ref"] or (it["ref"] if it["code"] == "en" else None)
            if it["tags"] is not None:
                row["tags_expected"] = it["tags"]
                row["tag_recall"] = round(len(set(a["distress"]) & set(it["tags"])) / len(it["tags"]), 2)
            if ref_en:
                row["translation_f1"] = token_f1(a["translation_en"], ref_en)
            rows.append(row)
            print(f"{it['item']:34} {str(snr):>5} {a['by']:8} lang {row['lang_pred']} ok={row['lang_ok']} "
                  f"cer={row['cer']} f1={row.get('translation_f1')} tags={a['distress']} "
                  f"{a['latency_ms']:.0f} ms" + (f" err={a['error']}" if a.get("error") else ""), flush=True)
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, f"stage2_{mode}_{'_'.join(str(s) for s in snrs)}.json")
    if start:                                                    # a later slice appends to the same file
        old = json.load(open(path)) if os.path.exists(path) else []
        done = {(r["item"], str(r["snr"])) for r in rows}
        rows = [r for r in old if (r["item"], str(r["snr"])) not in done] + rows
    json.dump(rows, open(path, "w"), indent=1, ensure_ascii=False)
    print(f"-> {path}")
    report_stage2(path)


PRICE_IN = float(os.environ.get("GEMINI_PRICE_AUDIO_IN_PER_M", "1.00"))     # USD per 1M audio input tokens (assumed)
PRICE_OUT = float(os.environ.get("GEMINI_PRICE_OUT_PER_M", "2.50"))         # USD per 1M output+thinking tokens (assumed)


def report_stage2(path):
    rows = json.load(open(path))
    done = [r for r in rows if r.get("by")]
    print(f"\nstage 2 ({os.path.basename(path)}): {len(done)}/{len(rows)} clips analysed "
          f"({sum(not r['forwarded'] for r in rows)} not forwarded by stage 1)")
    for code in ("zh", "en", "id", "vi"):
        rr = [r for r in done if r["code"] == code]
        if not rr:
            continue
        cers = [r["cer"] for r in rr if r["cer"] is not None]
        f1 = [r["translation_f1"] for r in rr if "translation_f1" in r]
        print(f"  {code}: lang-ID {sum(r['lang_ok'] for r in rr)}/{len(rr)}  median CER "
              f"{np.median(cers) if cers else float('nan'):.2f}  translation F1 {np.mean(f1) if f1 else float('nan'):.2f}")
    tags = [r["tag_recall"] for r in done if "tag_recall" in r]
    if tags:
        print(f"  distress-tag recall (TTS phrases): {np.mean(tags):.2f} over {len(tags)} clips")
    by = {}
    for r in done:
        by.setdefault(r["by"], []).append(r)
    for b, rr in by.items():
        lat = [r["latency_ms"] for r in rr if not r.get("cached")]
        ain = sum((r["usage"].get("promptTokenCount") or 0) for r in rr)
        out = sum((r["usage"].get("candidatesTokenCount") or 0) + (r["usage"].get("thoughtsTokenCount") or 0) for r in rr)
        cost = (ain * PRICE_IN + out * PRICE_OUT) / 1e6
        print(f"  by={b}: {len(rr)} clips, median latency {np.median(lat) if lat else 0:.0f} ms, "
              f"tokens in {ain} out {out}, est. cost ${cost:.4f} (${cost / max(1, len(rr)):.5f}/clip at "
              f"${PRICE_IN}/M in, ${PRICE_OUT}/M out: assumed prices, override GEMINI_PRICE_*)")
    errs = sorted({r["error"] for r in done if r.get("error")})
    if errs:
        print("  errors: " + "; ".join(errs[:4]))


# --------------------------------------------------------------------------- scene
def run_scene(seeds):
    """Stage 1 on the raw first mic of each landed drone in scene.py's synthetic listen (truth: evaluator only)."""
    from acoustic import novelty
    from acoustic.scene import build_scene
    from sim.scenario import make_scenario
    rows = []
    for seed in seeds:
        sc = make_scenario(seed, None)
        scene, truth = build_scene(sc, None, seed)
        for did, sig in scene.signals.items():
            x = sig[0].astype(float)
            r = novelty.detect(x, background=x, drone_id=did)
            for pile, v in truth.items():
                dist = float(np.hypot(*(np.array(v["xy"]) - np.array(scene.drones[did]))))
                for t in v["events_s"]:
                    if v["kind"] == "generator":
                        continue
                    win = (t + dist / 343.0, t + dist / 343.0 + 1.5)
                    hits = [c for c in r["candidates"] if _overlap(c, win, 0.2)]
                    rows.append({"seed": seed, "drone": did, "pile": pile, "kind": v["kind"], "t": t,
                                 "dist_m": round(dist, 1), "detected": bool(hits),
                                 "forwarded": any(c["forwarded"] for c in hits),
                                 "labels": sorted({c["label"] for c in hits}),
                                 "dismissed": sorted({c["dismissed"] for c in hits if c["dismissed"]})})
            print(f"seed {seed} {did}: {len(r['candidates'])} candidates, "
                  + ", ".join(f"{c['t0']:.1f}-{c['t1']:.1f} {c['label']}" for c in r["candidates"]))
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, "scene.json")
    json.dump(rows, open(path, "w"), indent=1)
    for kind in sorted({r["kind"] for r in rows}):
        rr = [r for r in rows if r["kind"] == kind]
        print(f"  {kind:11} events x drones {len(rr):3}  detected {_pct(rr, 'detected'):>10}  forwarded {_pct(rr, 'forwarded'):>10}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("fetch")
    f.add_argument("--n", type=int, default=3)
    f.add_argument("--no-voices", action="store_true")
    s1 = sub.add_parser("stage1")
    s1.add_argument("--bed", default="ambient", choices=["ambient", "rotor", "generator"])
    s1.add_argument("--snrs", default="10,0,-5")
    s1.add_argument("--profile", default="first6", choices=["first6", "minstat"])
    s1.add_argument("--limit", type=int, default=None)
    s2 = sub.add_parser("stage2")
    g = s2.add_mutually_exclusive_group()
    g.add_argument("--live", action="store_true", help="call Gemini (needs GEMINI_API_KEY in .env)")
    g.add_argument("--replay", action="store_true", help="recorded Gemini answers only")
    s2.add_argument("--snrs", default="clean,10,0")
    s2.add_argument("--limit", type=int, default=None)
    s2.add_argument("--start", type=int, default=0, help="skip the first N items (append to the results file)")
    s2.add_argument("--whisper", default=None, help="force a faster-whisper model name")
    sc = sub.add_parser("scene")
    sc.add_argument("--seeds", default="3")
    sub.add_parser("report")
    a = ap.parse_args(argv)
    if a.cmd == "fetch":
        fetch_fleurs(a.n)
        if not a.no_voices:
            fetch_voices()
    elif a.cmd == "stage1":
        run_stage1(a.bed, [float(s) for s in a.snrs.split(",")], a.profile, a.limit)
    elif a.cmd == "stage2":
        mode = "live" if a.live else "replay" if a.replay else "fallback"
        snrs = [s if s == "clean" else float(s) for s in a.snrs.split(",")]
        run_stage2(mode, snrs, a.limit, a.whisper, a.start)
    elif a.cmd == "scene":
        run_scene([int(s) for s in a.seeds.split(",")])
    elif a.cmd == "report":
        ps = sorted(glob.glob(os.path.join(OUT_DIR, "stage1_*.json")))
        report_stage1(ps)
        for p in sorted(glob.glob(os.path.join(OUT_DIR, "stage2_*.json"))):
            report_stage2(p)


if __name__ == "__main__":
    main()
