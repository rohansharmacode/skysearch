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
    # physio chest motion + 45 s pod dwell: A2, B1, B2 ping; A3's shallow rapid breathing (~2 mm, 33/min) stays an
    # open acoustic lead (its closest seats rock: q 0.03-0.6). Honest result, see docs/RADAR_PENETRATION.md.
    assert {"A2", "B1", "B2"} <= set(firm)
    assert all(firm[p]["t_found"] < 3000 for p in firm)
    for p, cond in (("A2", "unconscious_slow"), ("B1", "irregular_apneic"), ("B2", "calm")):
        v = firm[p].get("vitals")
        assert v and v["condition"] == cond and firm[p]["dig_score"] > 0, (p, v)


def test_story_acoustic_ranking(story):
    m, calls = story
    s = m.build_state()
    piles = {p["id"]: p for p in s["piles"]}
    assert (piles["B1"]["priority"] or 0) >= m.priority_min
    assert piles["C3"]["status"] != "lead"
    for c in s["acoustic_clips"]:
        if c["class"] == "dog":
            assert (c["scores"]["human"] or 0) < 0.4
