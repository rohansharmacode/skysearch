"""agents/gemini_vitals: fallback structure, pattern mapping, output validation, no truth in the prompt.
No network: llm._http_post is replaced by a fake; the cache goes to tmp_path."""
import numpy as np
import pytest

from agents import gemini_decider, gemini_vitals as gv, llm

KEYS = {"pattern", "rate_bpm", "confidence", "priority", "reason", "by"}


def wave(rate_bpm=8.0, amp_mm=4.0, fs=20.0, dur=60.0):
    t = np.arange(0, dur, 1 / fs)
    return amp_mm * np.sin(2 * np.pi * rate_bpm / 60.0 * t)


DSP = {"pattern": "slow_regular", "condition": "unconscious_slow", "rate_bpm": 8.2, "sqi": 0.8,
       "pattern_conf": 0.7, "triage_hint": "urgent", "source": "human_breathing"}


@pytest.fixture
def fake_gemini(monkeypatch, tmp_path):
    """Fake Gemini HTTP: returns `answer[0]` as the model's JSON text; records each request body."""
    import json
    monkeypatch.setenv("GEMINI_API_KEY", "test-key")
    monkeypatch.setenv("LLM_PROVIDER", "gemini")
    monkeypatch.delenv("LLM_REPLAY", raising=False)
    calls, answer = [], [None]

    def post(url, headers, body, timeout):
        calls.append(body)
        if isinstance(answer[0], Exception):
            raise answer[0]
        return {"candidates": [{"finishReason": "STOP",
                                "content": {"parts": [{"text": json.dumps(answer[0])}]}}]}

    monkeypatch.setattr(llm, "_http_post", post)
    return calls, answer, str(tmp_path / "llm.jsonl")


def test_fallback_structure_and_mapping():
    for dsp_pat, cond, want in [("slow_regular", "unconscious_slow", "deep_slow"),
                                ("rapid_shallow", "distressed_tachypnea", "shallow_rapid"),
                                ("regular_calm", "calm", "regular_calm"),
                                ("irregular_with_pauses", "irregular_apneic", "irregular_pauses"),
                                ("unclear", "unknown", "unclear")]:
        v = dict(DSP, pattern=dsp_pat, condition=cond)
        g = gv.analyse_breathing(wave(), 20.0, dsp_vitals=v, mode="fallback")
        assert KEYS <= set(g) and g["by"] == "fallback" and g["pattern"] == want
        assert 0 <= g["priority"] <= 100 and "check" in g["reason"].lower()
        assert gv.validate(g)
    # non-human source -> unclear
    g = gv.fallback(dict(DSP, source="machinery", triage_hint="reject"))
    assert g["pattern"] == "unclear"


def test_fallback_priority_uses_dig_score_and_audio_urgency():
    quiet = gv.fallback(DSP, None, dig=0.4)
    loud = gv.fallback(DSP, {"class": "shout", "scores": {"urgency": 1.0, "human": 0.9}}, dig=0.4)
    assert quiet["priority"] == 40 and loud["priority"] == 60
    assert gv.fallback(DSP, {"scores": {"urgency": 1.0}}, dig=0.9)["priority"] == 100        # clipped


def test_no_key_auto_falls_back(monkeypatch):
    monkeypatch.setattr(llm, "available", lambda provider=None: False)
    g = gv.analyse_breathing(wave(), 20.0, dsp_vitals=DSP, mode="auto")
    assert g["by"] == "fallback" and g["pattern"] == "deep_slow" and g["n_samples"] == 240


def test_downsample_caps_at_240_samples_4hz():
    s, fs = gv.downsample(wave(dur=120.0), 20.0)
    assert len(s) == 240 and fs == pytest.approx(4.0) and abs(np.mean(s)) < 1e-2


def test_gemini_answer_used(fake_gemini):
    calls, answer, cache = fake_gemini
    answer[0] = {"pattern": "deep_slow", "rate_bpm": 8, "confidence": 0.8, "priority": 82,
                 "reason": "Slow deep breaths, may be unresponsive; check responsiveness first."}
    g = gv.analyse_breathing(wave(), 20.0, dsp_vitals=DSP, mode="auto", cache_path=cache)
    assert g["by"] == "gemini" and g["priority"] == 82 and g["pattern"] == "deep_slow" and len(calls) == 1
    # recorded: a replay-only second call is answered from the cache, no network
    import os
    os.environ["LLM_REPLAY"] = "1"
    try:
        g2 = gv.analyse_breathing(wave(), 20.0, dsp_vitals=DSP, mode="auto", cache_path=cache)
    finally:
        del os.environ["LLM_REPLAY"]
    assert g2["by"] == "gemini" and len(calls) == 1


@pytest.mark.parametrize("bad", [
    {"pattern": "agonal", "rate_bpm": 8, "confidence": 0.8, "priority": 80, "reason": "check"},
    {"pattern": "deep_slow", "rate_bpm": 8, "confidence": 0.8, "priority": 140, "reason": "check"},
    {"pattern": "deep_slow", "rate_bpm": 8, "confidence": 0.8, "priority": -1, "reason": "check"},
    {"pattern": "deep_slow", "rate_bpm": 8, "confidence": 1.7, "priority": 50, "reason": "check"},
    {"pattern": "deep_slow", "rate_bpm": 8, "confidence": 0.8, "priority": 80, "reason": "Diagnosis: coma, check."},
    {"pattern": "deep_slow", "rate_bpm": 8, "confidence": 0.8, "priority": 80, "reason": "Confirmed person, check."},
    {"pattern": "deep_slow", "rate_bpm": 8, "confidence": 0.8, "priority": 80, "reason": "A child about 6 years old, check."},
    {"pattern": "shallow_rapid", "rate_bpm": 30, "confidence": 0.8, "priority": 80, "reason": "He is injured."},
    {"pattern": "deep_slow", "rate_bpm": 8, "confidence": 0.8, "priority": 80},
])
def test_validation_rejects_bad_output(fake_gemini, bad):
    calls, answer, cache = fake_gemini
    answer[0] = bad
    assert not gv.validate(bad)
    g = gv.analyse_breathing(wave(), 20.0, dsp_vitals=DSP, mode="auto", cache_path=cache)
    assert g["by"] == "fallback" and g["pattern"] == "deep_slow" and g["error"]


def test_http_429_falls_back_with_one_attempt(fake_gemini):
    calls, answer, cache = fake_gemini
    answer[0] = llm.HTTPError(429, "quota")
    g = gv.analyse_breathing(wave(), 20.0, dsp_vitals=DSP, mode="gemini", cache_path=cache)
    assert g["by"] == "fallback" and "429" in g["error"] and len(calls) == 1


def test_prompt_has_no_truth(fake_gemini):
    calls, answer, cache = fake_gemini
    answer[0] = {"pattern": "shallow_rapid", "rate_bpm": 32, "confidence": 0.7, "priority": 90,
                 "reason": "Fast shallow breathing, possible distress; check first."}
    v = dict(DSP, truth={"condition": "SECRET_TRUTH_COND"}, label="SECRET_LABEL",
             condition_text="SECRET_DSP_TEXT", target_id="SECRET_TARGET")
    audio = {"class": "shout", "transcript": "help I am stuck", "language": "en",
             "scores": {"urgency": 0.8, "human": 0.9}, "truth": "SECRET_AUDIO_TRUTH", "pile_id": "SECRET_PILE",
             "target": {"id": "SECRET_T"}}
    gv.analyse_breathing(wave(rate_bpm=32, amp_mm=1.0), 20.0, dsp_vitals=v, audio=audio, cache_path=cache)
    text = calls[0]["contents"][0]["parts"][0]["text"]
    assert "SECRET" not in text and "truth" not in text.lower()
    assert "help I am stuck" in text and "8.2/min" in text and "shout" in text
    assert gv.SYSTEM.split("\n")[0] in text


def test_dig_order_sorts_by_priority_when_present():
    rows = gemini_decider.dig_order([
        {"id": "A-R1", "status": "candidate", "p_mass": 0.9, "dig_score": 0.6, "priority": 40, "priority_by": "gemini"},
        {"id": "B-R1", "status": "candidate", "p_mass": 0.8, "dig_score": 0.3, "priority": 85, "priority_by": "gemini"}])
    assert [r["ping"] for r in rows] == ["B-R1", "A-R1"] and rows[0]["priority"] == 85
