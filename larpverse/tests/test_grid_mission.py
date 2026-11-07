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
    # every dispatch goes to a pile not visited before it, unless it is an explicit revisit of an open lead
    for k, d in enumerate(state["dispatch"]):
        if d["to_pile"] and not d["reason"].startswith("revisit"):
            assert d["to_pile"] not in m.visited[:k + 1]
    assert any(c["status"] == "candidate" for c in state["radar_candidates"]), "expected at least one radar ping"


def test_seed7_dsp_default_pings_every_voice_pile(run):
    """BL-8 / R1-005: with the default dsp radar the demo pings A2, A3 (voice) and B2 (knock)."""
    m, _, state, _ = run
    assert m.radar == "dsp" and state["mission"]["radar"] == "dsp"
    firm = {c["pile_id"] for c in state["radar_candidates"] if c["status"] == "candidate"}
    assert {"A2", "A3", "B2"} <= firm
    for c in state["radar_candidates"]:
        assert c["depth_source"] in ("radar_estimate", "nominal") and c["p_kind"] == "share_of_pile_belief"
        assert isinstance(c["support"], int) and c["support_needed"] == 3
        assert c["depth_sigma"] is None or c["depth_sigma"] > 0
        if c["status"] == "candidate":
            assert c["support"] >= 3 and c["t_found"] > 0
    assert "MISSION SUMMARY" in state["log"][-1]["msg"] and state["mission"]["summary"]


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


def test_slam_hook_ingests_pile_map(tmp_path):
    pytest.importorskip("mapping.slam")
    from mapping.raycast_lidar import LidarConfig
    kw = {"cfg": LidarConfig(channels=16, az_step_deg=1.0), "revs": 0.05, "method": "gt_noise"}
    m = GridMission(seed=7, verbose=False, acoustic="stub", pile_budget=1, slam=True, slam_scans=6, slam_kw=kw)
    m.run()
    s = m.build_state()
    assert s["slam"] and s["slam"][0]["pile"] == m.start_pile
    assert s["slam"][0]["map_points"] > 1000 and s["slam"][0]["ate_rmse_m"] < 1.0
    assert any("LiDAR SLAM" in e["msg"] for e in s["log"])
    assert any("Heightmap window rebuilt" in e["msg"] for e in s["log"])



# ------------------------------------------------------------------ autonomy loop (stub acoustic, fast)
def test_open_lead_without_ping_is_revisited_and_pinged():
    """First radar pass at A3 hears nothing (jammed scans); the autonomous loop comes back later and pings it."""
    from sim.radar_model import make_generator
    made = {}

    class Deaf:
        def __init__(self, inner):
            self.inner = inner

        def radar_scan(self, *a, **kw):
            obs = self.inner.radar_scan(*a, **kw)
            for o in obs:
                o["range_candidates"], o["confidence"], o["breathing_hz"] = [], [], None
            return obs[:1]

    def factory(pw, seed):
        g = make_generator(pw, seed, "model")
        made[pw.name] = made.get(pw.name, 0) + 1
        return Deaf(g) if pw.name == "A3" and made[pw.name] == 1 else g

    m = GridMission(seed=7, verbose=False, acoustic="stub", radar="model", sensor_factory=factory,
                    penetration="legacy").run()
    assert m.visited.count("A3") == 2 and m.visited.index("A3") < len(m.visited) - 1
    a3 = [c for c in m.done_cands if c["pile_id"] == "A3"]
    assert len(a3) == 1 and a3[0]["status"] == "candidate" and a3[0]["visit"] == 2
    assert any(e["msg"].startswith("Revisit d1-d3 -> pile A3") for e in m.log)
    assert any("Evidence:" in e["msg"] and "heard nothing" in e["msg"] for e in m.log)   # plain-words log
    assert m.scans_used <= m.mission_scans


def test_literature_penetration_reports_reach_and_coverage():
    m = GridMission(seed=7, verbose=False, acoustic="stub", pile_budget=1, penetration="literature",
                    mission_scans=6).run()
    try:
        s = m.build_state()
        assert s["mission"]["radar_penetration"] == "literature"
        assert 2.0 <= s["mission"]["radar_reach_beneath_m"] <= 4.0
        p = next(p for p in s["piles"] if p["id"] == m.start_pile)
        assert 0.0 <= p["radar_coverage"] <= 1.0 and 0.0 <= p["radar_coverage_shallow"] <= 1.0
        assert all("max_reliable_depth_m" in o for o in s["scans"])
        assert s["site"].get("geo") is None or {"lat", "lon", "bearing_deg"} <= set(s["site"]["geo"])
    finally:
        from sim import radar_dsp
        radar_dsp.use_penetration("legacy")


def test_battery_swap_then_redeploy():
    m = GridMission(seed=7, verbose=False, acoustic="stub", reserve=0.9, pile_budget=2, swap_s=30)
    phases, low = set(), None
    while not m.done:
        m.step()
        phases.add(m.phase)
        if m.stage == "swap" and low is None:
            low = min(d["battery"] for d in m.flight.poses())
    assert m.n_swaps == 1 and low is not None and low < 0.9
    assert {"return", "battery_swap"} <= phases and len(m.visited) == 2
    swap_msgs = [e for e in m.log if "battery swap" in e["msg"]]
    assert swap_msgs and "reserve" in swap_msgs[0]["msg"]
    assert m.build_state()["mission"]["battery_swaps"] == 1


def test_scan_budget_and_pile_cap_stop_the_loop():
    m = GridMission(seed=7, verbose=False, acoustic="stub", mission_scans=6).run()
    assert m.scans_used <= 6 and m.dispatch[-1]["to_pile"] is None
    assert "scan budget" in m.dispatch[-1]["reason"]
    m2 = GridMission(seed=7, verbose=False, acoustic="stub", pile_budget=2).run()
    assert len(m2.visited) == 2 and "pile budget" in m2.dispatch[-1]["reason"]
    legacy = GridMission(seed=7, verbose=False, acoustic="stub", autonomous=False, radar="model").run()
    assert legacy.pile_budget == 3 and len(legacy.visited) == 3 and legacy.n_swaps == 0


def test_baseline_policies_and_summary():
    r = GridMission(seed=7, verbose=False, acoustic="stub", dispatch_policy="raster", mission_scans=12).run()
    from sim.grid_mission import RASTER
    rest = [p for p in RASTER if p != r.start_pile]
    assert r.visited[1:] == rest[:len(r.visited) - 1] and not r.clips          # baselines do not listen
    n = GridMission(seed=7, verbose=False, acoustic="stub", dispatch_policy="nearest", mission_scans=10, pile_budget=2).run()
    sx, sy = n.world.pile_xy[n.start_pile]
    d = {p: np.hypot(x - sx, y - sy) for p, (x, y) in n.world.pile_xy.items() if p != n.start_pile}
    assert d[n.visited[1]] <= min(d.values()) + 1.0
    txt = r.final_summary()
    assert txt.startswith("MISSION SUMMARY") and "Open leads" in txt and "Not cleared" in txt


def test_stub_acoustic_is_labelled_from_truth():
    m = GridMission(seed=7, verbose=False, acoustic="stub", pile_budget=1).run()
    assert m.acoustic_source == "stub-from-truth"
    assert any(e["msg"].startswith("WARNING: acoustic clips are SYNTHETIC") for e in m.log)


def test_evaluate_grid_report(tmp_path):
    from sim import evaluate_grid as eg
    rows = str(tmp_path / "rows")
    eg.main(["--seeds", "7", "--scans", "6", "--rows-dir", rows])
    eg.main(["--seeds", "7", "--scans", "6", "--rows-dir", rows, "--start", "independent", "--policies", "acoustic"])
    out, md = tmp_path / "r.json", tmp_path / "e.md"
    eg.main(["--seeds", "7", "--scans", "6", "--rows-dir", rows, "--report", "--out", str(out), "--md", str(md)])
    res = json.loads(out.read_text())["legacy:reported"]
    assert set(res["summary"]) == set(eg.POLICIES)
    for p, s in res["summary"].items():
        assert 1 <= s["pinged_total"] <= s["victims_total"]     # the start-pile victim is always pinged
    text = md.read_text()
    assert "| metric | acoustic | nearest | raster |" in text and "from the hidden truth" in text
    assert "legacy` propagation, reported start" in text and "independent start" not in text   # incomplete: skipped
    assert eg.independent_start(7) in eg.PILES


def test_outvoted_trusted_scan_is_flagged_not_a_veto():
    """R2-002: one early trusted scan that disagrees no longer blocks a fix that >= 2 later strong ranges confirm."""
    from sim.coordinator import Mission
    from sim.localise import Localiser, predicted_range_field
    from sim.world import Grid
    n = 100
    g = Grid((-25.0, -25.0), 0.5, n, n, np.full((n, n), 1.0, np.float32))
    loc = Localiser(g)
    truth = (2.0, -1.5)

    def obs(sid, sxy, off=0.0):
        pose = [sxy[0], sxy[1], 1.3, 0.0]
        r, _ = predicted_range_field(g, pose, loc.cfg.depth_nom)
        ix, iy = g.index(*truth)
        return {"scan_id": sid, "drone_id": "d1", "sensor_pose": pose, "timestamp": 0.0,
                "range_candidates": [float(r[iy, ix]) + off], "confidence": [0.95], "quality": 0.9,
                "breathing_hz": 0.25, "doppler_moving": False}

    m = Mission.__new__(Mission)
    m.loc, m.scans, m.outliers = loc, [], []
    for o in [obs("s1", (-9, 4)), obs("s2", (8, 8), off=4.0), obs("s3", (3, 9)), obs("s4", (7, -8)), obs("s5", (-6, -9))]:
        loc.update_radar(o)
        m.scans.append(o)
    res = loc.extract()
    uw = loc.used_weights(res)
    for o in m.scans:
        o["used_weight"] = uw[o["scan_id"]]
    assert res["status"] == "candidate"
    assert m._rescan_reason(res) is None and m.outliers == ["s2"]
    # the same disagreement with fewer than 2 later confirmations still asks for a rescan
    m2 = Mission.__new__(Mission)
    m2.loc, m2.scans = Localiser(g), []
    for o in [obs("s1", (-9, 4)), obs("s3", (3, 9)), obs("s4", (7, -8)), obs("s2", (8, 8), off=4.0)]:
        m2.loc.update_radar(o)
        m2.scans.append(dict(o, used_weight=0.0 if o["scan_id"] == "s2" else 1.0))
    assert m2._rescan_reason(m2.loc.extract()) is not None and not m2.outliers   # not enough later looks yet


def test_acoustic_cache_is_versioned(tmp_path, monkeypatch):
    """R2-003: cached clips from another pipeline version are never replayed; no legacy served-folder cache."""
    from sim.scenario import make_scenario
    monkeypatch.setattr(acoustic_grid, "CACHE_DIR", str(tmp_path))
    sc = make_scenario(7)
    drones = {"d1": (0.0, 0.0), "d2": (1.0, 0.0), "d3": (2.0, 0.0)}
    path = acoustic_grid._cache_path(sc, drones, ["A2"], 3)
    r = acoustic_grid.listen_and_rank(sc, drones, ["A2"], 3)
    with open(path, "w") as f:
        json.dump(dict(r, _cache_version="old"), f)
    assert acoustic_grid._cache_lookup(sc, drones, ["A2"], 3) is None
    with open(path, "w") as f:
        json.dump(dict(r, _cache_version=acoustic_grid.ACOUSTIC_VERSION), f)
    assert acoustic_grid._cache_lookup(sc, drones, ["A2"], 3) is not None
    assert not hasattr(acoustic_grid, "LEGACY_CACHE_DIR")
