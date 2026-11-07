"""Stage 2/3 + agents.llm audio/throttling tests. Fake HTTP only, no network.
    .venv-audio/bin/python -m pytest -q tests/test_gemini_audio.py"""
import base64
import io
import json
import wave

import numpy as np
import pytest

from acoustic import gemini_audio as ga
from acoustic import novelty as nv
from agents import llm

FS = 16000


@pytest.fixture(autouse=True)
def clean_env(monkeypatch, tmp_path):
    for k in ("GEMINI_API_KEY", "ANTHROPIC_API_KEY", "LLM_PROVIDER", "LLM_REPLAY", "GEMINI_MODEL",
              "GEMINI_AUDIO_MODEL", "GEMINI_AUDIO_THINKING", "GEMINI_RPM", "LLM_RETRIES"):
        monkeypatch.delenv(k, raising=False)
    monkeypatch.setattr(llm, "_env_loaded", True)                     # ignore any real .env
    monkeypatch.setattr(llm, "CACHE_PATH", str(tmp_path / "llm.jsonl"))
    monkeypatch.setattr(llm, "_http_post", _no_network)
    ga._breaker.update(fails=0, until=0.0)


def _no_network(*a, **k):
    raise AssertionError("network call without a fake")


ANSWER = {"human_vocal": True, "sound_type": "call_for_help", "languages": [{"code": "cmn-TW", "name": "Mandarin"}],
          "transcript": "救命！我被困在樓梯下面，孩子不能呼吸。", "translation_en": "Help! I'm trapped under the stairs, the child can't breathe.",
          "distress": ["call_for_help", "trapped", "child", "cant_breathe", "location_hint", "bogus_tag"],
          "people_count": 2, "location_hints": ["under the stairs"], "confidence": 0.9,
          "rationale": "Clear Mandarin call for help mentioning a child who cannot breathe."}


class FakeGemini:
    def __init__(self, answer=ANSWER, status=None, usage=True):
        self.answer, self.status, self.calls, self.usage = answer, status, [], usage

    def __call__(self, url, headers, body, timeout):
        self.calls.append({"url": url, "headers": headers, "body": body})
        if self.status:
            raise llm.HTTPError(self.status, "busy")
        text = self.answer if isinstance(self.answer, str) else json.dumps(self.answer, ensure_ascii=False)
        r = {"candidates": [{"finishReason": "STOP", "content": {"parts": [{"text": text}]}}]}
        if self.usage:
            r["usageMetadata"] = {"promptTokenCount": 700, "candidatesTokenCount": 120, "totalTokenCount": 820,
                                  "promptTokensDetails": [{"modality": "AUDIO", "tokenCount": 160},
                                                          {"modality": "TEXT", "tokenCount": 540}]}
        return r


def _key(monkeypatch):
    monkeypatch.setenv("GEMINI_API_KEY", "test-key-not-real")


def _speech(seconds=3.0, f=220.0):
    t = np.arange(int(seconds * FS)) / FS
    return 0.2 * np.sin(2 * np.pi * f * t) * (1 + 0.5 * np.sin(2 * np.pi * 3 * t))


CAND = {"label": "likely_human", "vad_top": 0.9, "human_score": 0.85, "yamnet_top": [["Speech", 0.9]],
        "novelty_peak": 9.0, "sound_type": "speech", "reason": "Speech 0.85", "t0": 8.0, "t1": 11.0}


# --------------------------------------------------------------------------- llm.call_json_audio
def test_wav_bytes_is_mono_16k_and_capped():
    b = ga.wav_bytes(np.random.default_rng(0).normal(0, 0.1, 20 * FS))
    with wave.open(io.BytesIO(b)) as w:
        assert (w.getnchannels(), w.getframerate(), w.getsampwidth()) == (1, FS, 2)
        assert w.getnframes() == 15 * FS


def test_audio_request_shape_cache_and_replay(monkeypatch):
    _key(monkeypatch)
    fake = FakeGemini()
    monkeypatch.setattr(llm, "_http_post", fake)
    wb = ga.wav_bytes(_speech())
    meta = {}
    out = llm.call_json_audio("analyse", wb, ga.SCHEMA, system="sys", meta=meta, thinking={"thinkingLevel": "low"})
    assert out["sound_type"] == "call_for_help" and meta["usage"]["audioTokenCount"] == 160
    body = fake.calls[0]["body"]
    parts = body["contents"][0]["parts"]
    assert parts[0]["inline_data"]["mime_type"] == "audio/wav"
    assert base64.b64decode(parts[0]["inline_data"]["data"]) == wb
    assert parts[1]["text"].startswith("sys") and "analyse" in parts[1]["text"]
    gen = body["generationConfig"]
    assert gen["responseSchema"]["type"] == "OBJECT" and gen["thinkingConfig"] == {"thinkingLevel": "low"}
    assert "additionalProperties" not in json.dumps(gen["responseSchema"])
    assert fake.calls[0]["url"].endswith("/models/gemini-3.5-flash:generateContent")
    # same audio -> cache; different audio -> new call
    monkeypatch.setattr(llm, "_http_post", _no_network)
    m2 = {}
    assert llm.call_json_audio("analyse", wb, ga.SCHEMA, system="sys", meta=m2) == out and m2["cached"]
    m3 = {}                                                   # new audio reaches HTTP (here: the no-network guard)
    assert llm.call_json_audio("analyse", ga.wav_bytes(_speech(f=330.0)), ga.SCHEMA, system="sys", meta=m3) is None
    assert "AssertionError" in m3["error"] and not m3["cached"]
    # replay-only, no key: hit for the recorded clip, None otherwise
    monkeypatch.delenv("GEMINI_API_KEY")
    monkeypatch.setenv("LLM_REPLAY", "1")
    assert llm.call_json_audio("analyse", wb, ga.SCHEMA, system="sys") == out
    assert llm.call_json_audio("analyse", ga.wav_bytes(_speech(f=440.0)), ga.SCHEMA, system="sys") is None


def test_audio_needs_gemini(monkeypatch):
    meta = {}
    assert llm.call_json_audio("x", b"RIFF", meta=meta) is None and "no LLM" in meta["error"]
    monkeypatch.setenv("ANTHROPIC_API_KEY", "x")
    meta = {}
    assert llm.call_json_audio("x", b"RIFF", meta=meta) is None and "needs gemini" in meta["error"]
    _key(monkeypatch)
    monkeypatch.setenv("GEMINI_AUDIO_MODEL", "gemini-3.5-flash-lite")
    fake = FakeGemini()
    monkeypatch.setattr(llm, "_http_post", fake)
    llm.call_json_audio("x", b"RIFFdata", ga.SCHEMA, model=ga.audio_model())
    assert "gemini-3.5-flash-lite:generateContent" in fake.calls[0]["url"]


# --------------------------------------------------------------------------- throttling + backoff
def test_token_bucket_throttles(monkeypatch):
    slept = []
    monkeypatch.setattr(llm.time, "sleep", lambda s: slept.append(s))
    b = llm.TokenBucket(rpm=60, burst=1)
    assert b.acquire() == 0.0
    w = b.acquire()
    assert 0.9 <= w <= 1.01 and slept and b.waited_s > 0


def test_backoff_honours_retry_after_and_daily_quota(monkeypatch):
    slept, n = [], {"i": 0}
    monkeypatch.setattr(llm.time, "sleep", lambda s: slept.append(s))

    def flaky(url, headers, body, timeout):
        n["i"] += 1
        if n["i"] < 3:
            raise llm.HTTPError(429, '{"retryDelay": "4s"}', retry_after=4.0)
        return {"ok": 1}
    monkeypatch.setattr(llm, "_http_post", flaky)
    assert llm._post_retry("u", {}, {}, 5) == {"ok": 1}
    assert len(slept) == 2 and all(3.0 <= s <= 5.0 for s in slept)

    def daily(url, headers, body, timeout):
        raise llm.HTTPError(429, "Generate content requests per day limit exceeded on Free Tier")
    monkeypatch.setattr(llm, "_http_post", daily)
    slept.clear()
    with pytest.raises(llm.HTTPError) as e:
        llm._post_retry("u", {}, {}, 5)
    assert e.value.daily_quota and slept == []
    assert llm._retry_after(None, '"retryDelay": "12.5s"') == 12.5


# --------------------------------------------------------------------------- stage 2
def test_validate_analysis_normalises_and_rejects():
    v = ga.validate_analysis(ANSWER)
    assert v["distress"] == ["call_for_help", "trapped", "child", "cant_breathe", "location_hint"]
    assert v["languages"][0]["code"] == "cmn-TW" and v["confidence"] == 0.9
    assert ga.validate_analysis(dict(ANSWER, rationale="Survivor confirmed under the stairs.")) is None
    assert ga.validate_analysis(dict(ANSWER, rationale="Not confirmed; a candidate lead.")) is not None
    assert ga.validate_analysis(dict(ANSWER, sound_type="opera")) is None
    assert ga.validate_analysis(dict(ANSWER, confidence=1.7)) is None
    assert ga.validate_analysis({"confidence": 0.5}) is None


def test_analyse_clip_gemini(monkeypatch):
    _key(monkeypatch)
    fake = FakeGemini()
    monkeypatch.setattr(llm, "_http_post", fake)
    a = ga.analyse_clip(_speech(), cand=CAND)
    assert a["by"] == "gemini" and a["translation_en"].startswith("Help") and a["usage"]["promptTokenCount"] == 700
    prompt = fake.calls[0]["body"]["contents"][0]["parts"][1]["text"]
    assert "Speech 0.90" in prompt and "never" in prompt.lower()
    assert fake.calls[0]["body"]["generationConfig"]["thinkingConfig"] == {"thinkingLevel": "low"}


def test_cross_check_caps_hallucination(monkeypatch):
    _key(monkeypatch)
    monkeypatch.setattr(llm, "_http_post", FakeGemini())
    a = ga.analyse_clip(_speech(1.0), cand=dict(CAND, vad_top=0.05, t0=8.0, t1=9.0))
    assert a["confidence"] <= 0.6 and "VAD" in a["cross_check"] and "1.0 s clip" in a["cross_check"]


def test_distress_needs_a_second_transcription(monkeypatch):
    _key(monkeypatch)
    calls = []

    def fake(url, headers, body, timeout):
        txt = body["contents"][0]["parts"][-1]["text"]
        calls.append(txt)
        out = {"transcript": "Por favor ayúdame", "language": "es"} if txt.startswith("Transcribe exactly") else ANSWER
        return {"candidates": [{"finishReason": "STOP", "content": {"parts": [{"text": json.dumps(out)}]}}]}
    monkeypatch.setattr(llm, "_http_post", fake)
    a = ga.analyse_clip(_speech(), cand=CAND)
    assert len(calls) == 2 and a["verify"]["ok"] is False and a["distress"] == []
    assert "trapped" in a["distress_unverified"] and a["confidence"] <= 0.5 and a["sound_type"] == "speech"
    assert "second transcription" in a["cross_check"]
    monkeypatch.setenv("GEMINI_AUDIO_VERIFY", "0")                        # can be switched off
    calls.clear()
    b = ga.analyse_clip(_speech(f=260.0), cand=CAND)
    assert len(calls) == 1 and b["distress"] and "verify" not in b


def test_429_falls_back_with_busy_note_and_breaker(monkeypatch):
    _key(monkeypatch)
    monkeypatch.setenv("LLM_RETRIES", "0")
    fake = FakeGemini(status=429)
    monkeypatch.setattr(llm, "_http_post", fake)
    quiet = dict(CAND, vad_top=0.0, label="uncertain")            # no ASR in the fallback
    a = ga.analyse_clip(_speech(), cand=quiet)
    assert a["by"] == "fallback" and a["by_note"] == "Gemini busy — using fallback" and "429" in a["error"]
    assert a["rationale"].startswith("[Gemini busy")
    ga.analyse_clip(_speech(f=300.0), cand=quiet)
    n = len(fake.calls)
    b = ga.analyse_clip(_speech(f=350.0), cand=quiet)              # circuit open: no HTTP at all
    assert len(fake.calls) == n and b["by"] == "fallback" and "circuit" in b["error"]


def test_invalid_answer_falls_back(monkeypatch):
    _key(monkeypatch)
    monkeypatch.setattr(llm, "_http_post", FakeGemini(dict(ANSWER, rationale="Person confirmed.")))
    a = ga.analyse_clip(_speech(), cand=dict(CAND, vad_top=0.0, label="uncertain"))
    assert a["by"] == "fallback" and a["error"]


def test_fallback_keywords_multilingual():
    assert set(ga.keyword_distress("救命！我被困住了，孩子不能呼吸")) >= {"call_for_help", "trapped", "child", "cant_breathe"}
    assert set(ga.keyword_distress("Tolong! Saya terjebak")) >= {"call_for_help", "trapped"}
    assert set(ga.keyword_distress("Cứu tôi với! Tôi bị kẹt")) >= {"call_for_help", "trapped"}


# --------------------------------------------------------------------------- stage 3
def _clf_for(kind):
    def clf(clip):
        base = {"reason": kind, "background_score": 0.0, "vad_top": 0.9, "vad_frac": 0.6, "knock_periodicity": 0.0,
                "yamnet_top": [["Speech", 0.9]], "sound_type": "speech", "human_class": "Speech", "other_class": "Dog"}
        if kind == "dog":
            return dict(base, label="likely_other", human_score=0.02, other_score=0.8, vad_top=0.0)
        return dict(base, label="likely_human", human_score=0.9, other_score=0.0)
    return clf


def _clip_with_event(seed):
    rng = np.random.default_rng(seed)
    x = rng.normal(0, 4e-3, 16 * FS)
    t = np.arange(2 * FS) / FS
    x[8 * FS:10 * FS] += 0.05 * np.sin(2 * np.pi * 500 * t) * np.hanning(len(t))
    return x


def test_sector_ranking_distress_and_association(monkeypatch):
    _key(monkeypatch)
    calm = dict(ANSWER, sound_type="speech", distress=[], people_count=1, location_hints=[],
                transcript="今天天氣很好", translation_en="The weather is nice today", rationale="Calm speech.")
    answers = iter([ANSWER, calm])
    calls = []

    def fake(url, headers, body, timeout):
        calls.append(body)
        txt = body["contents"][0]["parts"][-1]["text"]
        if "inline_data" not in body["contents"][0]["parts"][0]:          # rationale (text) call
            return {"candidates": [{"finishReason": "STOP", "content": {"parts": [{"text": json.dumps({"sectors": [
                {"pile_id": "A1", "rationale": "Mandarin call for help; child can't breathe; send the radar drone to A1."},
                {"pile_id": "B2", "rationale": "Calm speech; survivor confirmed at B2."}]})}]}}]}
        assert "truth" not in txt
        if txt.startswith("Transcribe exactly"):                          # distress verification pass
            v = {"transcript": ANSWER["transcript"], "language": "cmn"}
            return {"candidates": [{"finishReason": "STOP", "content": {"parts": [{"text": json.dumps(v)}]}}]}
        return {"candidates": [{"finishReason": "STOP", "content": {"parts": [{"text": json.dumps(next(answers))}]}}]}
    monkeypatch.setattr(llm, "_http_post", fake)
    clf = _clf_for("speech")
    items = [{"clip_id": "ac-A1", "pile_id": "A1", "drone_id": "d1", "audio": _clip_with_event(1), "reliability": 0.8,
              "association": "confident"},
             {"clip_id": "ac-B2", "pile_id": "B2", "drone_id": "d2", "audio": _clip_with_event(2), "reliability": 0.9,
              "association": "ambiguous"}]
    res = ga.analyse_sector_audio(items, classifier=clf, denoise_method="wiener")
    t = {x["pile_id"]: x for x in res["sector_tasks"]}
    assert res["sector_tasks"][0]["pile_id"] == "A1" and t["A1"]["rank"] == 1
    assert t["A1"]["urgency"] > t["B2"]["urgency"] and "cant_breathe" in t["A1"]["distress"]
    assert "multiple_people" in t["A1"]["distress"]                       # people_count 2
    assert t["B2"]["reliability"] <= 0.45                                 # ambiguous direction capped
    assert t["A1"]["rationale_by"] == "gemini" and t["B2"]["rationale_by"] == "template"   # "confirmed" rejected
    assert res["clips"]["ac-A1"]["language"] == "cmn-TW" and res["clips"]["ac-A1"]["forwarded"]
    assert res["clips"]["ac-A1"]["translation"].startswith("Help")


def test_dismissed_sector_and_merge(monkeypatch):
    items = [{"clip_id": "ac-C3", "pile_id": "C3", "drone_id": "d3", "audio": _clip_with_event(3), "reliability": 0.7}]
    res = ga.analyse_sector_audio(items, classifier=_clf_for("dog"), denoise_method="wiener", mode="fallback")
    t = res["sector_tasks"][0]
    assert t["status"] == "dismissed_nonhuman" and t["dismissed"] == ["dismissed: dog"]
    assert res["clips"]["ac-C3"]["dismissed"] == ["dismissed: dog"] and not res["clips"]["ac-C3"]["forwarded"]
    state = {"schema": "larpverse.acoustic/v1", "clips": [{"clip_id": "ac-C3", "pile_id": "C3"}],
             "piles": [{"pile_id": "C3", "priority": 5, "status": "dismissed_nonhuman", "why": "dog", "top_clip": "ac-C3"}],
             "dispatch": {"pile_id": None, "reason": "none"}}
    ga.merge_into_state(state, res)
    assert state["clips"][0]["novelty"] > 0 and state["sector_tasks"][0]["pile_id"] == "C3"
    assert state["piles"][0]["audio_priority"] == 0 and state["piles"][0]["status"] == "dismissed_nonhuman"
    assert state["gemini_audio"]["stage1"]["dismissed"] == {"dog": 1}
    assert "confirmed" not in json.dumps(state).replace("not confirmed", "")


def test_merge_reprioritises_and_dispatches(monkeypatch):
    res = {"prompt": "p", "mode": "auto", "stage1": {}, "stage2": {}, "clips": {},
           "sector_tasks": [{"pile_id": "B1", "priority": 70, "status": "lead", "distress": ["trapped"], "languages": [],
                             "reasons": ["call for help"], "rationale": "Call for help; send radar to B1."}]}
    state = {"clips": [], "piles": [{"pile_id": "A1", "priority": 40, "status": "radar_ping", "why": "x"},
                                    {"pile_id": "B1", "priority": 8, "status": "no_signal_not_cleared", "why": "y"}],
             "dispatch": {"pile_id": "A1", "reason": "x"}}
    ga.merge_into_state(state, res)
    assert [p["pile_id"] for p in state["piles"]] == ["B1", "A1"]
    assert state["piles"][0]["status"] == "radar_ping" and state["piles"][1]["status"] == "lead"
    assert state["dispatch"]["pile_id"] == "B1"


def test_ground_truth_never_reaches_stage2(monkeypatch):
    _key(monkeypatch)
    seen = []

    def fake(url, headers, body, timeout):
        seen.append(json.dumps(body))
        return FakeGemini()(url, headers, body, timeout)
    monkeypatch.setattr(llm, "_http_post", fake)
    items = [{"clip_id": "ac-A2", "pile_id": "A2", "drone_id": "d1", "audio": _clip_with_event(4), "reliability": 0.8}]
    ga.analyse_sector_audio(items, classifier=_clf_for("speech"), denoise_method="wiener", rationale=False)
    assert seen and not any(w in s for s in seen for w in ("truth", "targets", "events_s"))
    assert nv.public_candidate({"clip": 1, "a": 2}) == {"a": 2}
