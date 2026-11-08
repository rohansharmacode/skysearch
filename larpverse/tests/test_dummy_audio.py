"""Victim-dummy audio library (assets/audio/dummies, built by scripts/build_dummy_audio.py):
manifest schema, files on disk, format, durations, levels, no clipping, profiles, size budget."""
import json
import os
import wave

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DIR = os.path.join(ROOT, "assets", "audio", "dummies")
MAN = os.path.join(DIR, "manifest.json")
KINDS = {"voice", "weak_voice", "knock", "groan", "cough", "cry", "dog", "generator", "settling", "wind", "siren"}
HUMAN = {"voice", "weak_voice", "knock", "groan", "cough", "cry"}
DEPTHS = [0.5, 1.0, 1.5, 2.0]
REQUIRED = {"id", "file", "kind", "human", "language", "transcript", "transcript_en", "source", "licence",
            "attribution", "processing", "synthetic", "sample_rate", "duration_s", "rms_dbfs", "peak_dbfs",
            "through_rubble_depth_m", "parent", "suggested_spl_db_1m", "schedule", "non_commercial", "author"}

pytestmark = pytest.mark.skipif(not os.path.exists(MAN), reason="library not built")


@pytest.fixture(scope="module")
def man():
    with open(MAN, encoding="utf-8") as f:
        return json.load(f)


@pytest.fixture(scope="module")
def clips(man):
    return {c["id"]: c for c in man["clips"]}


def read(path):
    with wave.open(path) as w:
        assert w.getnchannels() == 1 and w.getsampwidth() == 2
        fs = w.getframerate()
        pcm = np.frombuffer(w.readframes(w.getnframes()), "<i2")
    return pcm, fs


def gated_rms_db(x, fs):
    n = int(0.05 * fs)
    m = len(x) // n
    d = 10 * np.log10(np.mean(x[: m * n].reshape(m, n) ** 2, axis=1) + 1e-20)
    keep = (d > d.max() - 30) & (d > -70)
    return 10 * np.log10(np.mean(10 ** (d[keep] / 10)))


def test_schema(man, clips):
    assert man["schema"] == "larpverse.dummy_audio/v1"
    assert len(clips) == len(man["clips"]), "duplicate ids"
    for c in man["clips"]:
        assert REQUIRED <= set(c), (c["id"], REQUIRED - set(c))
        assert c["kind"] in KINDS
        assert c["human"] == (c["kind"] in HUMAN)
        assert c["source"] and c["licence"] and c["attribution"]
        if "ESC-50" in c["source"]:
            assert "BY-NC" in c["licence"] and "Piczak" in c["attribution"]
        if c["kind"] in ("voice", "weak_voice"):
            assert c["language"] in ("en", "zh-TW", "id", "vi") and c["transcript"] and c["transcript_en"]
        assert c["sample_rate"] == 48000
        s = c["schedule"]
        assert s and s["mode"] in ("loop", "after_callout", "continuous")


def test_files_format_duration_no_clipping(man):
    for c in man["clips"]:
        path = os.path.join(DIR, c["file"])
        assert os.path.exists(path), c["file"]
        pcm, fs = read(path)
        assert fs == 48000
        dur = len(pcm) / fs
        assert abs(dur - c["duration_s"]) < 0.01, c["id"]
        assert 0.5 <= dur <= 15.0, (c["id"], dur)
        assert np.abs(pcm.astype(int)).max() < 32767, f"{c['id']} clips"
        x = pcm / 32768.0
        peak = 20 * np.log10(np.abs(x).max())
        assert peak <= -0.9 and abs(peak - c["peak_dbfs"]) < 0.1, c["id"]
        assert abs(gated_rms_db(x, fs) - c["rms_dbfs"]) < 0.3, c["id"]


def test_loudness_matched(man):
    tgt = man["format"]["target_rms_dbfs"]
    for c in man["clips"]:
        if c["parent"] is None:
            assert abs(c["rms_dbfs"] - tgt[c["kind"]]) < 1.0, (c["id"], c["rms_dbfs"])   # limiter may pull peaks
    assert tgt["weak_voice"] <= tgt["voice"] - 10


def test_rubble_variants(man, clips):
    for c in man["clips"]:
        if c["parent"] is not None or not c["human"]:
            continue
        kids = sorted((k for k in man["clips"] if k["parent"] == c["id"]), key=lambda k: k["through_rubble_depth_m"])
        assert [k["through_rubble_depth_m"] for k in kids] == DEPTHS, c["id"]
        energy = []
        for k in [c] + kids:
            x = read(os.path.join(DIR, k["file"]))[0] / 32768.0
            energy.append(float(np.sum(x ** 2)))
            assert k["kind"] == c["kind"] and k["transcript"] == c["transcript"]
        assert all(b < a for a, b in zip(energy, energy[1:])), (c["id"], energy)   # deeper = quieter


def test_coverage_and_todo(man, clips):
    langs = {c["language"] for c in man["clips"] if c["kind"] == "voice"}
    assert {"en", "zh-TW", "id", "vi"} <= langs
    assert any(not c["synthetic"] and c["kind"] == "voice" for c in man["clips"]), "real CC0 voices"
    for k in ("weak_voice", "knock", "cough", "cry", "dog", "generator", "settling", "wind", "siren"):
        assert any(c["kind"] == k for c in man["clips"]), k
    todo = {t["id"]: t for t in man["todo"]}
    assert todo["voice_nan_call"]["language"] == "nan-TW" and "native" in todo["voice_nan_call"]["status"]
    assert "groan" in todo and not any(c["kind"] == "groan" for c in man["clips"])
    assert all("processed" in " ".join(c["processing"]).lower() or "weak" in c["source"].lower()
               for c in man["clips"] if c["kind"] == "weak_voice")


def _profile_clips(p):
    out = []
    for key in ("idle", "after_callout"):
        if p.get(key):
            out += p[key]["playlist"]
    return out + p.get("alternatives", [])


def test_profiles(man, clips):
    prof = {p["dummy_id"]: p for p in man["profiles"]}
    assert set(prof) == {"A3", "B1", "B2", "A2", "C2", "C1", "B3"}
    for p in man["profiles"] + man["hidden_dummies"]:
        for cid in _profile_clips(p):
            assert cid in clips, (p["dummy_id"], cid)
        assert p["prim"].startswith("/World/Dummies/")
    assert prof["A2"]["idle"] is None and prof["A2"]["after_callout"] is None
    a3 = {clips[c]["language"] for c in prof["A3"]["idle"]["playlist"]}
    assert {"zh-TW", "en"} <= a3
    assert all(clips[c]["kind"] == "weak_voice" for c in prof["B1"]["idle"]["playlist"])
    assert all(clips[c]["kind"] == "knock" for c in prof["B2"]["after_callout"]["playlist"])
    assert prof["B2"]["after_callout"]["mode"] == "after_callout"
    assert [clips[c]["kind"] for c in prof["C2"]["idle"]["playlist"]] == ["dog"]
    assert [clips[c]["kind"] for c in prof["C1"]["idle"]["playlist"]] == ["generator"]
    assert [clips[c]["kind"] for c in prof["B3"]["idle"]["playlist"]] == ["settling"]
    for c in ("A3", "B1", "B2"):     # buried dummies play the through-rubble version of their depth
        for cid in prof[c]["idle"]["playlist"]:
            assert clips[cid]["through_rubble_depth_m"] == prof[c]["burial_depth_m"]
    for a in man["ambient"]:
        assert a["clip"] in clips


def test_hidden_dummies_in_silent_sectors(man, clips):
    """Hidden dummies sit in seed-7 sectors with no sound (an evaluator-side truth read)."""
    import sys
    sys.path.insert(0, ROOT)
    from sim.scenario import make_scenario
    roles = make_scenario(7).truth["roles"]
    assert 1 <= len(man["hidden_dummies"]) <= 2
    for h in man["hidden_dummies"]:
        assert roles.get(h["pile"]) == "silent", h["pile"]
        assert h["active_at_start"] is False
        langs = {clips[c]["language"] for c in h["idle"]["playlist"]}
        assert langs & {"vi", "id"}


def test_size_budget(man):
    tot = sum(os.path.getsize(os.path.join(dp, f)) for dp, _, fs in os.walk(DIR) for f in fs if f.endswith(".wav"))
    assert tot <= 60e6, tot / 1e6


def test_licences(man):
    """Review pass 3: no voice with an unknown or research-only licence; non-commercial sources are flagged."""
    assert set(man["voice_licences"]) == {"espeak-ng", "piper vi_VN-vivos-x_low"}
    for c in man["clips"]:
        lic = c["licence"].lower()
        assert "unknown" not in lic and "model card" not in lic and "research" not in lic, c["id"]
        assert isinstance(c["non_commercial"], bool)
        if c["source"].startswith("ESC-50") or "vivos" in c["source"]:
            assert c["non_commercial"], c["id"]
        if "vivos" in c["source"]:
            assert "BY-NC-SA" in c["licence"]
        if c["source"].startswith("TTS"):
            assert c["language"] == "vi" or "eSpeak" in c["source"], c["id"]
        if not c["synthetic"]:
            assert c["author"] and c["origin_url"], c["id"]


def test_attribution_file(man):
    path = os.path.join(DIR, "ATTRIBUTION.md")
    assert os.path.exists(path)
    text = open(path, encoding="utf-8").read()
    for c in man["clips"]:
        if c["parent"] is None and not c["synthetic"]:
            assert c["origin_url"] in text and c["author"] in text, c["id"]
    for who in ("Robinhood76", "ErocShadow", "PickleJones", "VIVOS", "CC BY-NC-SA 4.0", "eSpeak NG"):
        assert who in text, who
    assert "Credit line for the video description" in text
