"""Acoustic priority list: trapped vs safe people on the surface vs noise (acoustic/priority.py + stage 2 + stub)."""
import numpy as np

from acoustic import gemini_audio as ga
from acoustic import priority as tiers
from acoustic.scene import FS, footsteps, through_rubble
from sim.acoustic_grid import listen_and_rank
from sim.scenario import SURFACE_KINDS, make_scenario

CALM = "Okay, team two is heading to the next block, all clear here."


def _voiceish(seed=0, dur=3.0):
    """Broadband voiced+fricative stand-in for speech (crisp highs, like an unmuffled talker)."""
    rng = np.random.default_rng(seed)
    t = np.arange(int(dur * FS)) / FS
    ph = 2 * np.pi * np.cumsum(120 + 20 * np.sin(2 * np.pi * 0.7 * t)) / FS
    x = sum(np.sin(k * ph) / k ** 0.8 for k in range(1, 60)) + 0.6 * rng.normal(0, 1, len(t))
    return 0.1 * x * (np.sin(2 * np.pi * 2.5 * t) > 0)


def _fallback(clip, text, monkeypatch, sound_type="speech"):
    monkeypatch.setattr(ga, "local_asr", lambda x, model=None: (text, ["en"], 0.9, text, "base.en"))
    cand = {"label": "likely_human", "vad_top": 0.8, "human_score": 0.8, "sound_type": sound_type,
            "yamnet_top": [["Speech", 0.8]], "novelty_peak": 8.0, "reason": "Speech 0.80", "t0": 0, "t1": 3}
    return ga.analyse_clip(clip, cand=cand, mode="fallback")


def test_muffling_feature_open_air_vs_buried():
    x = _voiceish()
    assert tiers.muffling(x)["setting"] == "open_air"
    assert tiers.muffling(through_rubble(x, 1.5))["setting"] == "under_rubble"
    assert tiers.muffling(footsteps(np.random.default_rng(1)))["setting"] == "open_air"


def test_surface_voice_fallback_is_safe_with_zero_priority(monkeypatch):
    a = _fallback(_voiceish(), CALM, monkeypatch)
    assert a["by"] == "fallback" and a["setting"] == "open_air" and a["setting_by"] == "dsp"
    assert a["tier"] == "safe_surface" and "safe" in a["tier_why"] and len(a["tier_why"].split()) <= 12
    t = ga._rank_sector("B3", [{"forwarded": True, "analysis": a, "cand_id": "e1", "t0": 0, "t1": 3,
                                "novelty_peak": 8.0, "human_score": 0.8}], 0.9, "confident")
    assert t["priority"] == 0 and t["status"] != "lead" and t["tier"] == "safe_surface"


def test_buried_voice_is_trapped(monkeypatch):
    a = _fallback(through_rubble(_voiceish(), 1.5), "I am down here under the slab", monkeypatch)
    assert a["setting"] == "under_rubble" and a["tier"] == "trapped"


def test_distress_words_in_open_air_stay_trapped(monkeypatch):
    a = _fallback(_voiceish(), "Help! I'm trapped, my leg is stuck", monkeypatch)
    assert a["setting"] == "open_air" and a["tier"] == "trapped"
    assert tiers.classify("speech", "open_air", ["call_for_help"], 6, 0.9)[0] == "trapped"


def test_classify_table():
    assert tiers.classify("footsteps", "open_air")[0] == "safe_surface"
    assert tiers.classify("knocking", "under_rubble", human=0.7)[0] == "trapped"
    assert tiers.classify("knocking", "open_air", human=0.7)[0] == "check"
    assert tiers.classify("speech", "open_air", words=1, human=0.8)[0] == "check"      # words not understood
    assert tiers.classify("speech", "unclear", words=8, human=0.8)[0] == "check"
    for s in ("dog", "machinery", "debris", "noise"):
        assert tiers.classify(s, "open_air", human=0.05)[0] == "ignore"
    assert [t["tier"] for t in tiers.PRIORITY_LIST] == list(tiers.TIERS)


def test_scenario_surface_people_are_not_victims(monkeypatch):
    monkeypatch.setenv("LARPVERSE_PEOPLE", "3")
    for site, seed in (("grid", 7), ("weiguan", 7), ("grid", 11)):
        t = make_scenario(seed, site=site).truth
        surf = t["surface_sounds"]
        assert [s["kind"] for s in surf] == list(SURFACE_KINDS)
        assert len({s["pile"] for s in surf}) == 1 and surf[0]["pile"] not in {v["pile"] for v in t["victims"]}
        assert surf[0]["pile"] != t["start_pile"] and len(t["victims"]) == 3
        assert not any(s["kind"] in SURFACE_KINDS for s in t["sounds"])


def test_stub_listen_never_leads_on_surface_people(monkeypatch):
    monkeypatch.setenv("LARPVERSE_PEOPLE", "3")
    sc = make_scenario(7)
    p = sc.pile("B2")
    r = listen_and_rank(sc, {"d1": (p.x, p.y + 8.6), "d2": (p.x - 7.4, p.y - 4.3), "d3": (p.x + 7.4, p.y - 4.3)},
                        ["B2"], 1)
    surf = [c for c in r["clips"] if c["clip_id"].endswith(SURFACE_KINDS)]
    assert surf and all(c["tier"] == "safe_surface" and c["priority"] == 0 and c["human"] <= 0.1 for c in surf)
    trapped = [c for c in r["clips"] if c["pile_id"] in ("A3", "B1")]
    assert trapped and all(c["tier"] in ("trapped", "check") and c["priority"] > 20 for c in trapped)
    b3 = next(q for q in r["piles"] if q["pile_id"] == "B3")
    assert b3["status"] != "lead" and b3["priority"] <= 8 and r["dispatch"]["pile_id"] != "B3"
    assert r["priority_list"] == tiers.PRIORITY_LIST
