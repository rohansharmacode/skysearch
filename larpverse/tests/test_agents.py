"""Agent layer tests: fake HTTP only, no network.  .venv/bin/python -m pytest -q tests/test_agents.py"""
import json

import pytest

from agents import brief, gemini_decider, intake, llm

FORBIDDEN = ("truth", "victim", "target")


@pytest.fixture(autouse=True)
def clean_env(monkeypatch, tmp_path):
    for k in ("GEMINI_API_KEY", "ANTHROPIC_API_KEY", "LLM_PROVIDER", "LLM_REPLAY", "GEMINI_MODEL", "OPS_INTEL"):
        monkeypatch.delenv(k, raising=False)
    monkeypatch.setattr(llm, "_env_loaded", True)                     # ignore any real .env
    monkeypatch.setattr(llm, "CACHE_PATH", str(tmp_path / "llm.jsonl"))
    monkeypatch.setattr(intake, "INTEL_PATH", str(tmp_path / "intel.txt"))
    monkeypatch.setattr(llm, "_http_post", _no_network)


def _no_network(*a, **k):
    raise AssertionError("network call without a fake")


class FakeGemini:
    """Records requests; answers with a Gemini generateContent body wrapping ``answer``."""

    def __init__(self, answer, status=None):
        self.answer, self.status, self.calls = answer, status, []

    def __call__(self, url, headers, body, timeout):
        self.calls.append({"url": url, "headers": headers, "body": body})
        if self.status:
            raise llm.HTTPError(self.status, "boom")
        text = self.answer if isinstance(self.answer, str) else json.dumps(self.answer)
        return {"candidates": [{"finishReason": "STOP", "content": {"role": "model", "parts": [
            {"text": "thinking about it...", "thought": True}, {"text": text}]}}]}

    def prompt(self, i=-1):
        return self.calls[i]["body"]["contents"][0]["parts"][0]["text"]


def _ctx():
    return {"t": 122.0, "current_pile": "A2", "visited": ["A2"], "pile_budget": 3, "budget_left": 2,
            "priority_min": 20,
            "pings": [{"id": "A2-R1", "pile_id": "A2", "status": "candidate", "p_mass": 0.97}],
            "piles": [{"id": "A3", "status": "lead", "priority": 95, "why": "Voice saying help"},
                      {"id": "C3", "status": "lead", "priority": 48, "why": "Voice asking for water"}],
            "fleet": [{"id": "d1", "battery": 0.86, "state": "landed"}],
            # a leaked field the decider must strip before prompting
            "truth": {"victims": [[1, 2]]}, "targets": [[3, 4]]}


def _opts():
    return [
        {"id": "o1", "action": "dispatch", "pile_id": "A3", "priority": 95, "travel_s": 15.0,
         "battery_after_min": 0.71, "evidence": "voice \"Help, my leg is trapped\" (human 0.99)", "flags": []},
        {"id": "o2", "action": "dispatch", "pile_id": "C3", "priority": 48, "travel_s": 21.3,
         "battery_after_min": 0.70, "evidence": "voice \"anyone there, water\" (human 0.9)", "flags": []},
        {"id": "o3", "action": "dispatch", "pile_id": "B3", "priority": 42, "travel_s": 16.9,
         "battery_after_min": 0.10, "evidence": "knocking (human 0.8)", "flags": []},
        {"id": "o4", "action": "stop", "pile_id": None, "priority": None, "travel_s": None,
         "battery_after_min": 0.86, "evidence": "return to the staging area", "flags": []},
    ]


def _with_key(monkeypatch):
    monkeypatch.setenv("GEMINI_API_KEY", "test-key-not-real")


# --------------------------------------------------------------------------- llm layer
def test_provider_resolution(monkeypatch):
    assert llm.resolve() == ("none", None)
    monkeypatch.setenv("ANTHROPIC_API_KEY", "x")
    assert llm.resolve()[0] == "anthropic"
    _with_key(monkeypatch)
    assert llm.resolve() == ("gemini", "gemini-3.5-flash")
    monkeypatch.setenv("LLM_PROVIDER", "none")
    assert llm.resolve()[0] == "none"


def test_gemini_request_shape_and_parse(monkeypatch):
    _with_key(monkeypatch)
    fake = FakeGemini({"choice": "o1", "rationale": "ok"})
    monkeypatch.setattr(llm, "_http_post", fake)
    meta = {}
    out = llm.call_json("hello", gemini_decider.SCHEMA, system="sys", meta=meta)
    assert out == {"choice": "o1", "rationale": "ok"} and meta["provider"] == "gemini"
    c = fake.calls[0]
    assert c["url"].endswith("/models/gemini-3.5-flash:generateContent")
    assert c["headers"]["x-goog-api-key"] == "test-key-not-real"
    gen = c["body"]["generationConfig"]
    assert gen["responseMimeType"] == "application/json" and gen["temperature"] == 0.2
    assert gen["responseSchema"]["type"] == "OBJECT" and "additionalProperties" not in gen["responseSchema"]


def test_llm_failures_return_none(monkeypatch):
    assert llm.call_json("x", {}) is None                                  # no provider
    monkeypatch.setenv("LLM_PROVIDER", "gemini")
    meta = {}
    assert llm.call_json("x", {}, meta=meta) is None and "key" in meta["error"]
    _with_key(monkeypatch)
    monkeypatch.setattr(llm, "_http_post", FakeGemini("not json at all"))
    assert llm.call_json("x", {}) is None
    monkeypatch.setattr(llm, "_http_post", FakeGemini({}, status=403))
    meta = {}
    assert llm.call_json("y", {}, meta=meta) is None and meta["error"] == "gemini HTTP 403"
    assert "test-key" not in json.dumps(meta)


def test_replay_cache(monkeypatch):
    _with_key(monkeypatch)
    fake = FakeGemini({"choice": "o2", "rationale": "recorded"})
    monkeypatch.setattr(llm, "_http_post", fake)
    a = llm.call_json("same prompt", gemini_decider.SCHEMA)
    monkeypatch.setattr(llm, "_http_post", _no_network)                    # second call must not hit HTTP
    meta = {}
    b = llm.call_json("same prompt", gemini_decider.SCHEMA, meta=meta)
    assert a == b and meta["cached"] and len(fake.calls) == 1
    bad = FakeGemini({"choice": "nope"})                                   # rejected answers are not recorded
    monkeypatch.setattr(llm, "_http_post", bad)
    for _ in range(2):
        llm.call_json("p2", gemini_decider.SCHEMA, validate=lambda o: "rationale" in o)
    assert len(bad.calls) == 2
    monkeypatch.delenv("GEMINI_API_KEY")
    monkeypatch.setenv("LLM_REPLAY", "1")                                  # offline demo, no key
    assert llm.call_json("same prompt", gemini_decider.SCHEMA) == a
    assert llm.call_json("other prompt", gemini_decider.SCHEMA) is None    # replay miss, still no network


def test_dotenv_parser(tmp_path, monkeypatch):
    p = tmp_path / ".env"
    for k in ("GEMINI_API_KEY", "GEMINI_MODEL"):           # register so monkeypatch removes them afterwards
        monkeypatch.setenv(k, "")
        monkeypatch.delenv(k)
    p.write_text("# comment\nexport GEMINI_API_KEY=\"abc\"\nGEMINI_MODEL=m1 # trailing\n")
    llm.load_dotenv(str(p))
    import os
    assert os.environ["GEMINI_API_KEY"] == "abc" and os.environ["GEMINI_MODEL"] == "m1"


# --------------------------------------------------------------------------- decider
def test_decider_accepts_valid_choice(monkeypatch):
    _with_key(monkeypatch)
    fake = FakeGemini({"choice": "o2", "rationale": "C3 asks for water. Second sentence. Third dropped."})
    monkeypatch.setattr(llm, "_http_post", fake)
    d = gemini_decider.make_decider(constraints=intake.empty())
    out = d(_ctx(), _opts())
    assert out["choice"] == "o2" and out["by"] == "gemini"
    assert out["rationale"] == "C3 asks for water. Second sentence."
    assert "latency_ms" in out


@pytest.mark.parametrize("answer,why", [
    ({"choice": "o9", "rationale": "made up"}, "not a listed option"),
    ({"choice": "o3", "rationale": "B3 knocking"}, "hard limit"),                 # below battery reserve
    ({"choice": "o1", "rationale": "Survivor found at A3, confirmed."}, "banned"),
    ("{broken", "gemini"),
])
def test_decider_invalid_falls_back(monkeypatch, answer, why):
    _with_key(monkeypatch)
    monkeypatch.setattr(llm, "_http_post", FakeGemini(answer))
    out = gemini_decider.make_decider(constraints=intake.empty())(_ctx(), _opts())
    assert out["by"] == "fallback" and out["choice"] == "o1" and why in out["fallback_reason"]


def test_decider_missing_key_falls_back():
    out = gemini_decider.make_decider(constraints=intake.empty())(_ctx(), _opts())
    assert out["by"] == "fallback" and out["choice"] == "o1"


def test_fallback_respects_operator_constraints():
    c = intake.clamp({"pile_priors": {"C3": 3.0}, "no_go_piles": ["A3"], "ignore_sound_piles": []}, "n")
    out = gemini_decider.make_decider(constraints=c)(_ctx(), _opts())
    assert out["choice"] == "o2"                                        # A3 no-go, C3 boosted
    c = intake.clamp({"pile_priors": {}, "no_go_piles": [], "ignore_sound_piles": ["A3", "C3"]}, "n")
    out = gemini_decider.make_decider(constraints=c)(_ctx(), _opts())
    assert out["choice"] == "o4"                                        # B3 below reserve -> stop


def test_no_ground_truth_in_prompts(monkeypatch, tmp_path):
    _with_key(monkeypatch)
    fake = FakeGemini({"choice": "o1", "rationale": "A3 has the strongest call for help."})
    monkeypatch.setattr(llm, "_http_post", fake)
    (tmp_path / "intel.txt").write_text("Bedrooms were on the east side of A3.")
    d = gemini_decider.make_decider()                                   # intake prompt + decider prompt
    d(_ctx(), _opts())
    fake.answer = {"brief": "Dig at A2-R1."}
    brief.brief({"t": 10.0, "mission": {"visited": ["A2"]}, "piles": [{"id": "A2", "xy": [0, 0]}],
                 "radar_candidates": [{"id": "A2-R1", "pile_id": "A2", "xy": [1, 0], "status": "candidate",
                                       "z_est": 1.0, "cov": [[0.1, 0], [0, 0.1]], "p_mass": 0.9}],
                 "truth": {"targets": [[1, 0]]}})
    assert len(fake.calls) >= 3
    for i in range(len(fake.calls)):
        text = fake.prompt(i).lower()
        for w in FORBIDDEN:
            assert w not in text, (w, text[:200])


# --------------------------------------------------------------------------- intake + brief
def test_intake_llm_clamped_and_keyword_fallback(monkeypatch):
    notes = "Bedrooms were on the east side of A3. Our crew is digging at B2, ignore noise there. C1 unstable."
    kw = intake.intake(notes)
    assert kw["by"] == "keyword" and kw["pile_priors"] == {"A3": 2.0}
    assert kw["ignore_sound_piles"] == ["B2"] and kw["no_go_piles"] == ["C1"]
    _with_key(monkeypatch)
    monkeypatch.setattr(llm, "_http_post", FakeGemini({
        "pile_priors": [{"pile": "a3", "multiplier": 9, "reason": "bedrooms"},
                        {"pile": "Z9", "multiplier": 2, "reason": "bogus"}],
        "ignore_sound_piles": ["B2", "Q1"], "no_go_piles": ["C1"]}))
    out = intake.intake(notes)
    assert out["by"] == "gemini" and out["pile_priors"] == {"A3": 3.0}
    assert out["ignore_sound_piles"] == ["B2"] and out["no_go_piles"] == ["C1"]


def _state():
    return {"t": 492.0, "mission": {"visited": ["A2", "B2"]},
            "piles": [{"id": "A2", "xy": [0, 0], "status": "ping"}, {"id": "B2", "xy": [10, 0], "status": "lead"},
                      {"id": "A1", "xy": [0, 10], "status": "no_signal_not_cleared"},
                      {"id": "C3", "xy": [20, -20], "status": "lead", "priority": 46, "why": "Voice"}],
            "radar_candidates": [{"id": "A2-R1", "pile_id": "A2", "xy": [2, 0], "z_est": 1.0,
                                  "cov": [[0.2, 0], [0, 0.1]], "p_mass": 0.97, "breathing_hz": 0.3,
                                  "status": "candidate", "evidence": ["s1", "s2"]}],
            "heightmap": {"origin": [-5, -5], "res": 1.0, "w": 30, "h": 30, "z": [2.5] * 900}}


def test_brief_template_and_llm_guard(monkeypatch):
    r = brief.brief(_state())
    assert r["by"] == "template"
    t = r["brief"]
    assert "A2-R1" in t and "east flank" in t and "~1.5 m below" in t and "A1" in t and "C3" in t
    assert not brief.BANNED.search(t)
    _with_key(monkeypatch)
    monkeypatch.setattr(llm, "_http_post", FakeGemini({"brief": "Survivor found at A2-R1, confirmed."}))
    assert brief.brief(_state())["by"] == "template"
    monkeypatch.setattr(llm, "_http_post", FakeGemini({"brief": "Dig A2-R1 on A2; also check C2."}))
    assert brief.brief(_state())["by"] == "template"                    # unknown pile id
    monkeypatch.setattr(llm, "_http_post", FakeGemini({"brief": "Dig at A2-R1 on A2, 1.5 m down. A1 not cleared."}))
    assert brief.brief(_state())["by"] == "gemini"


# --------------------------------------------------------------------------- acoustic agent via agents.llm
def test_acoustic_agent_uses_llm_layer(monkeypatch, tmp_path):
    from acoustic import agent
    f = {"clip_id": "a", "pile_id": "B3", "drone_id": "d1", "duration_s": 4.0, "continuous": False,
         "bearing_deg": 270.0, "width_deg": 25.0, "snr_db": 12.0, "vad_speech_ratio": 0.5, "transcript": "help",
         "sound_class": "voice", "class_scores": {"voice": 0.9}, "heuristics": {}, "yamnet_top": [],
         "doa": {"drones_agreeing": 3, "drones_listening": 3, "tri_err_m": 1.0, "assigned": True}}
    cache = tmp_path / "decisions.jsonl"
    j, meta = agent.judge_clips([f], mode="auto", cache_path=str(cache))
    assert j["a"]["by"] == "fallback" and meta["llm_calls"] == 0 and meta["provider"] == "none"
    _with_key(monkeypatch)
    fake = FakeGemini({"judgments": [{"clip_id": "a", "human": 0.8, "urgency": 0.7, "reliability": 0.6,
                                      "rationale": "voice says help"}]})
    monkeypatch.setattr(llm, "_http_post", fake)
    j, meta = agent.judge_clips([f], mode="auto", cache_path=str(cache))
    assert j["a"]["by"] == "llm" and j["a"]["human"] == 0.8 and meta["provider"] == "gemini"
    assert "acoustic triage analyst" in fake.prompt()
    monkeypatch.setattr(llm, "_http_post", _no_network)
    j, meta = agent.judge_clips([f], mode="auto", cache_path=str(cache))   # replayed from its own cache
    assert j["a"]["by"] == "llm" and meta["cache_hits"] == 1
