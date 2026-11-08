"""Seed-7 demo story on the committed real-pipeline acoustic fixture (R1-014 / R2-011).

The fixture (tests/fixtures/acoustic_seed7.json, assoc-v11) holds the acoustic/ pipeline's listen results for the
demo's listen sequence; it is replayed by excluded-pile set, so the radar side (realistic penetration, winch pod)
runs live. Drone positions differ slightly from the recorded ones; association results are kept as recorded.
"""
import json
import os

import pytest

from sim.grid_mission import GridMission

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIXTURE = os.path.join(ROOT, "tests", "fixtures", "acoustic_seed7.json")


def replay_fn():
    with open(FIXTURE) as f:
        fx = json.load(f)
    calls = []

    def fn(scenario, drones, exclude, seed):
        ex = set(exclude)
        usable = [L for L in fx["listens"] if set(L["exclude"]) <= ex]
        L = max(usable, key=lambda L: len(L["exclude"]))
        r = dict(L["state"])
        r["clips"] = [c for c in r["clips"] if c["pile_id"] not in ex]
        r["piles"] = [p for p in r["piles"] if p["pile_id"] not in ex]
        if (r.get("dispatch") or {}).get("pile_id") in ex:
            r["dispatch"] = {"pile_id": None, "reason": "recorded dispatch already visited"}
        calls.append({"exclude": sorted(ex), "replayed": sorted(L["exclude"])})
        return r
    return fn, calls


@pytest.fixture(scope="module")
def story():
    if not os.path.exists(FIXTURE):
        pytest.skip("acoustic fixture missing")
    fn, calls = replay_fn()
    m = GridMission(seed=7, verbose=False, acoustic_fn=fn, tasking="default").run()
    return m, calls


def test_story_visits_and_pings(story):
    m, _ = story
    assert m.delivery == "pod" and m.penetration == "literature" and m.radar == "dsp"
    first = list(dict.fromkeys(m.visited))
    assert first[:2] == ["A2", "A3"] and {"B1", "B2"} <= set(first)
    firm = {c["pile_id"]: c for c in m.done_cands if c["status"] == "candidate"}
    # Realistic physics (physio chest motion, ~2.9 m reach beneath a 45 s pod scan, smooth flight timing): which
    # victims ping depends on how the pods seat (marginal SNR). The start pile always pings; every ping's vitals
    # condition matches the hidden state; A3's shallow rapid breather (~2 mm, 33/min) stays an open lead.
    assert "A2" in firm
    truth = {"A2": "unconscious_slow", "A3": "distressed_tachypnea", "B1": "irregular_apneic", "B2": "calm"}
    for p, c in firm.items():
        v = c.get("vitals")
        assert v and v["condition"] == truth[p] and c["dig_score"] > 0, (p, v)
    assert all(c["t_found"] < 3600 for c in firm.values())


def test_story_acoustic_ranking(story):
    m, calls = story
    s = m.build_state()
    piles = {p["id"]: p for p in s["piles"]}
    assert (piles["B1"]["priority"] or 0) >= m.priority_min
    assert piles["C3"]["status"] != "lead"
    for c in s["acoustic_clips"]:
        if c["class"] == "dog":
            assert (c["scores"]["human"] or 0) < 0.4
