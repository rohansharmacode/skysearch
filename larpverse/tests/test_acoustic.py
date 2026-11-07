"""Fast acoustic tests. Run with the audio venv:  .venv-audio/bin/python -m pytest tests/test_acoustic.py"""
import json

import numpy as np
import pytest

from acoustic import agent


def _feat(cid, transcript, cls, scores, vad, snr=12.0, width=25.0, n=3):
    base = {c: 0.0 for c in ("voice", "shout", "knocking", "dog", "engine", "debris", "noise")}
    base.update(scores)
    return {"clip_id": cid, "pile_id": "B3", "drone_id": "d1", "duration_s": 4.0, "continuous": False,
            "bearing_deg": 270.0, "width_deg": width, "snr_db": snr, "vad_speech_ratio": vad,
            "transcript": transcript, "sound_class": cls, "class_scores": base,
            "heuristics": {"knock_periodicity": 0.0, "hum_harmonicity": 0.1}, "yamnet_top": [],
            "doa": {"drones_agreeing": n, "drones_listening": 3, "tri_err_m": 1.0, "tri_resid_m": 0.3,
                    "assigned": True}}


@pytest.mark.parametrize("src", [(20.0, 10.0), (-15.0, -12.0), (3.0, 24.0)])
def test_srp_phat_doa_clean_source(src):
    pytest.importorskip("pyroomacoustics")
    from acoustic import dsp
    from acoustic.scene import single_source_scene
    sc = single_source_scene(src, (0.0, 0.0), duration=2.0)
    _, zs = dsp.srp_blocks(sc.signals["d1"], sc.mic_offsets, sc.fs)
    est = dsp.circ_peaks(zs)[0][0]
    truth = np.degrees(np.arctan2(src[1], src[0])) % 360
    assert abs(dsp.wrap180(est - truth)) <= 10.0


def test_fallback_ranks_trapped_call_above_dog():
    help_ = _feat("a", "Help! I'm trapped, I can't move.", "voice", {"voice": 0.8, "noise": 0.2}, vad=0.6)
    dog = _feat("b", "", "dog", {"dog": 0.85, "voice": 0.1, "noise": 0.05}, vad=0.0, snr=18.0)
    jh, jd = agent.fallback_judge(help_), agent.fallback_judge(dog)
    assert jh["human"] > jd["human"] and jh["urgency"] > jd["urgency"]
    assert agent.clip_priority(jh) > agent.clip_priority(jd) + 30


def test_validation_and_replay_cache(tmp_path, monkeypatch):
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    assert agent.validate_judgment({"human": 1.4, "urgency": 0.5, "reliability": 0.5, "rationale": "x"}) is None
    assert agent.validate_judgment({"human": 0.4, "urgency": 0.5, "rationale": "x"}) is None
    f = _feat("a", "help", "voice", {"voice": 0.9}, vad=0.5)
    cache = tmp_path / "decisions.jsonl"
    j, meta = agent.judge_clips([f], mode="auto", cache_path=str(cache))
    assert j["a"]["by"] == "fallback" and meta["llm_calls"] == 0
    rec = {"key": agent.feature_key(f), "clip_id": "a",
           "judgment": {"human": 0.7, "urgency": 0.6, "reliability": 0.5, "rationale": "recorded"}}
    cache.write_text(json.dumps(rec) + "\n")
    j, meta = agent.judge_clips([f], mode="replay", cache_path=str(cache))
    assert j["a"]["by"] == "llm" and j["a"]["rationale"] == "recorded" and meta["cache_hits"] == 1
