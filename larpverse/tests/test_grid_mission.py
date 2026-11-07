"""Fast checks for the multi-pile grid mission (sim/grid_mission.py), synthetic acoustic stub only."""
import json
import os
import re

import numpy as np
import pytest

from sim import acoustic_grid
from sim.grid_mission import GridMission
from sim.scenario import SITE_BOUNDS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PILE_STATUSES = {"unvisited", "surveying", "scanning", "ping", "lead", "dismissed_nonhuman", "no_signal_not_cleared"}


@pytest.fixture(scope="module")
def run(tmp_path_factory):
    out = tmp_path_factory.mktemp("grid")
    calls = []

    def recording_stub(sc, drones, exclude, seed):
        r = acoustic_grid.listen_and_rank(sc, drones, exclude, seed)
        calls.append({"exclude": list(exclude), "result": r})
        return r

    m = GridMission(seed=7, out_dir=str(out), verbose=False, n_points=20000, acoustic_fn=recording_stub)
    m.run()
    with open(out / "state.json") as f:
        state = json.load(f)
    return m, calls, state, out


def test_visits_multiple_piles_in_acoustic_priority_order(run):
    m, calls, state, _ = run
    assert m.done and len(m.visited) >= 2
    assert m.visited[0] == m.start_pile
    first = calls[0]["result"]
    ranked = sorted((p for p in first["piles"] if p["pile_id"] not in calls[0]["exclude"]),
                    key=lambda p: (-p["priority"], p["pile_id"]))
    assert m.visited[1] == ranked[0]["pile_id"]
    assert state["dispatch"][0]["from_pile"] == m.visited[0] and state["dispatch"][0]["to_pile"] == m.visited[1]
    # every dispatch goes to a pile not visited before it
    for k, d in enumerate(state["dispatch"]):
        if d["to_pile"]:
            assert d["to_pile"] not in m.visited[:k + 1]
    assert any(c["status"] == "candidate" for c in state["radar_candidates"]), "expected at least one radar ping"


def test_brain_never_reads_truth():
    for name in ("localise.py", "planner.py", "landing.py", "coordinator.py", "grid_mission.py"):
        src = open(os.path.join(ROOT, "sim", name)).read()
        code = "\n".join(line.split("#")[0] for line in src.splitlines())
        assert not re.search(r"\.truth\b|\.targets\b|truth_json|\[\s*['\"]victims['\"]", code), name
    for name in ("localise.py", "planner.py", "landing.py"):
        src = open(os.path.join(ROOT, "sim", name)).read()
        assert not re.search(r"^\s*(from|import)\s+.*(scenario|grid_world|radar_model|acoustic)", src, re.M), name


def test_state_json_grid_extensions(run):
    m, _, s, out = run
    assert s["schema"] == "larpverse.state/v1"
    for k in ("t", "phase", "site", "heightmap", "pointcloud", "drones", "landing_sites", "scans",
              "radar_candidates", "acoustic", "belief", "next_scan", "log"):
        assert k in s
    hm = s["heightmap"]
    assert len(hm["z"]) == hm["w"] * hm["h"]
    assert s["site"]["bounds"] == list(SITE_BOUNDS)
    assert os.path.getsize(out / "pointcloud.bin") == 12 * s["pointcloud"]["count"]
    assert len(s["piles"]) == 9
    for p in s["piles"]:
        assert set(p) >= {"id", "xy", "status", "priority", "radar_candidate_ids", "why"}
        assert p["status"] in PILE_STATUSES
        assert p["priority"] is None or (isinstance(p["priority"], int) and 0 <= p["priority"] <= 100)
    ids = {c["id"] for c in s["radar_candidates"]}
    assert len(ids) == len(s["radar_candidates"])
    for c in s["radar_candidates"]:
        assert c["pile_id"] in m.visited and c["id"].startswith(c["pile_id"] + "-")
    for p in s["piles"]:
        assert set(p["radar_candidate_ids"]) <= ids
    assert all("pile_id" in o for o in s["scans"])
    assert s["acoustic_clips"]
    for c in s["acoustic_clips"]:
        assert set(c) >= {"id", "t", "pile_id", "drone_id", "origin", "bearing_deg", "class", "transcript",
                          "scores", "priority", "rationale", "by"}
        assert set(c["scores"]) == {"human", "urgency", "reliability"}
    for d in s["dispatch"]:
        assert set(d) == {"t", "from_pile", "to_pile", "reason"} and d["reason"]
    assert all(d["state"] == "docked" for d in s["drones"])           # mission ends back at the pads
    assert s["mission"]["visited"] == m.visited
    # build_state() is the same document (Isaac adapter API)
    b = m.build_state()
    assert b["piles"] == s["piles"] and b["heightmap"]["z"][:50] == s["heightmap"]["z"][:50]


def test_step_api_and_listening_state():
    m = GridMission(seed=7, verbose=False, acoustic="stub")
    seen = set()
    while not m.done and m.t < 400:
        m.step()
        st = m.build_state()
        seen.update(d["state"] for d in st["drones"])
        assert all(len(d["pose"]) == 4 and np.all(np.isfinite(d["pose"])) for d in st["drones"])
    assert {"flying", "scanning", "listening"} <= seen


def test_decider_hook_choice_and_fallback():
    seen = []

    def stop_now(ctx, options):
        seen.append((ctx, options))
        assert ctx["current_pile"] and options[-1]["action"] == "stop"
        assert all({"id", "action", "pile_id", "priority", "travel_s", "battery_after_min", "evidence", "flags"} <= set(o)
                   for o in options)
        return {"choice": options[-1]["id"], "rationale": "test: stop", "by": "test"}

    m = GridMission(seed=7, verbose=False, acoustic="stub", decider=stop_now).run()
    assert m.visited == [m.start_pile] and seen
    d = m.build_state()["decisions"]
    assert d[0]["by"] == "test" and d[0]["chosen"] == seen[0][1][-1]["id"] and d[0]["gate"] == "dispatch"

    bad = GridMission(seed=7, verbose=False, acoustic="stub", decider=lambda c, o: {"choice": "nope"}).run()
    ref = GridMission(seed=7, verbose=False, acoustic="stub").run()
    assert bad.visited == ref.visited
    assert all(x["by"] == "fallback" for x in bad.decisions if len(x["options"]) > 1)
    assert all(x["by"] == "argmax" for x in ref.decisions)
