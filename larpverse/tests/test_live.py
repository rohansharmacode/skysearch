"""Live single take (isaac/live.py, isaac/hud.py, run_mission.py --live) without omni / pxr / a GPU.

Scripted triggers appear here (tests only): in the take the change is a live human action, and
run_mission.py refuses --test-trigger outside --dry-run."""
import json
import math
import os
import sys

import numpy as np
import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)


def _no_omni():
    return not any(m == "pxr" or m.startswith(("pxr.", "omni.", "carb", "isaaclab")) or m in ("omni", "isaaclab")
                   for m in sys.modules)


def _mission(tmp_path, tasking=True):
    from isaac.live import write_default_tasking
    from sim.grid_mission import GridMission
    tk = write_default_tasking(str(tmp_path)) if tasking else None
    return GridMission(seed=7, verbose=False, acoustic="stub", tasking=tk, out_dir=None)


def _live(m):
    from isaac.live import ListSource, LiveController, LiveWorld, dummies_from_scenario
    sc = m.scenario
    hm = sc.heightmap(0.5)
    w = LiveWorld(sc, ListSource(dummies_from_scenario(sc, hm.height_at)), surface=hm.height_at, log=lambda s: None)
    c = LiveController(m, w, acoustic="stub", log=lambda s: None, clock=lambda: m.t)
    return w, c


def test_untouched_stage_is_the_scenario(tmp_path):
    """Dummies where the scenario put them reproduce its truth exactly (same story as the CPU run), so listens
    may use the cache; the hidden dummies are inactive."""
    m = _mission(tmp_path, tasking=False)
    base = m.scenario
    w, c = _live(m)
    assert c.hook in ("stub", "native")
    assert not w.changed
    t = w.truth()
    for v in base.truth["victims"]:
        lv = next(x for x in t["victims"] if x["id"] == v["id"])
        assert all(lv[k] == v[k] for k in ("pile", "depth", "breathing_hz", "sound")), (v, lv)
    for s in base.truth["sounds"]:
        assert any(all(x.get(k) == s[k] for k in s) for x in t["sounds"]), s
    hid = w.source.hidden_ids()
    assert len(hid) >= 1 and not any(v["id"] in hid for v in t["victims"])


def test_world_override_changes_acoustic_leads(tmp_path):
    """A hidden dummy switched on: the next listen (fresh, uncached synthesis) hears a new human lead in its
    sector, and the radar generator gets it as a target; switching it off again restores the scenario."""
    m = _mission(tmp_path, tasking=False)
    w, c = _live(m)
    hid = w.source.hidden_ids()
    did = hid[-1]                                   # grid seed 7: H2_A1_id (A1, next to the start pile A2)
    row = next(d for d in w.source.read() if d["id"] == did)
    sector = w.locate(*row["xyz"][:2])
    drones = {"d1": (-18.0, 6.0), "d2": (-26.0, 4.0), "d3": (-20.0, -3.0)}    # landed around A2
    before = m.acoustic_fn(m.scenario, drones, ["A2"], 101)
    assert before["live"]["fresh_uncached"] is True  # stub mode is always synthesised
    lead0 = {p["pile_id"]: p for p in before["piles"]}[sector]
    assert lead0["status"] != "lead"
    assert not w.targets_for(sector)

    assert w.set_active(did, True, "test")
    after = m.acoustic_fn(m.scenario, drones, ["A2"], 101)
    assert w.changed and after["live"]["world"] != after["live"]["scenario_world"]
    lead1 = {p["pile_id"]: p for p in after["piles"]}[sector]
    assert lead1["status"] == "lead" and lead1["priority"] >= 20, lead1
    assert [t["id"] for t in w.targets_for(sector)] == [did]
    assert w.events and w.events[-1]["what"] == "activated" and w.events[-1]["id"] == did

    w.set_active(did, False, "test")
    w.refresh(0.0)
    assert not w.changed


def test_moved_dummy_reaches_the_radar(tmp_path):
    """Dragging a dummy (viewport) moves the radar target from the next scan on; a dragged slab over it (column
    probe higher than the map) deepens it."""
    m = _mission(tmp_path, tasking=False)
    w, c = _live(m)
    v = w.targets_for("A2")[0]
    new = [v["xy"][0] + 3.0, v["xy"][1] - 2.0, 0.6]
    w.move(next(d["id"] for d in w.source.read() if d.get("victim") == v["id"] or d["id"] == v["id"]), new, "test drag")
    w.refresh(1.0)
    t2 = w.targets_for("A2")[0]
    assert math.dist(t2["xy"], new[:2]) < 1e-6 and t2["z"] == pytest.approx(0.6)
    assert w.events[-1]["what"] == "moved" and w.changed
    w.probe = lambda x, y: float(w.surface(x, y)) + 1.0          # a 1 m slab dragged on top
    w.refresh(2.0)
    assert w.targets_for("A2")[0]["depth"] > t2["depth"] + 0.9


def test_tasking_revision_redirects(tmp_path):
    """Operator draws a new zone mid-run (POST /api/tasking writes a new revision): the controller's 1 s poll
    merges it, the radar dwell under way finishes, and the fleet is dispatched to the new zone."""
    m = _mission(tmp_path)
    w, c = _live(m)
    while not (m.stage == "pile" and m.pm.scans):
        c.before_step()
        m.step()
        c.after_step(m.build_state())
        assert m.t < 200
    path = os.path.join(str(tmp_path), "tasking.json")
    doc = json.load(open(path))
    x, y = m.world.pile_xy["C3"]
    doc["zones"].append({"id": "Z_NEW", "name": "operator circle", "status": "tasked",
                         "polygon": [[x - 8, y - 8], [x + 8, y - 8], [x + 8, y + 8], [x - 8, y + 8]]})
    doc["revision"] = int(doc.get("revision") or 1) + 1
    doc["source"] = "operator"
    with open(path, "w") as f:
        json.dump(doc, f)
    os.utime(path, None)
    t0 = m.t
    while m.t < t0 + 60 and "C3" not in m.visited:
        c.before_step()
        m.step()
        c.after_step(m.build_state())
    assert m.visited[-1] == "C3", (m.visited, c.replans)
    assert c.replans and "Z_NEW" in c.replans[0]["why"]
    assert m.dispatch[-1]["to_pile"] == "C3"
    assert m.t - t0 < 60


def test_hud_noop_import():
    from isaac.hud import Hud, HudModel, NullHud, make_hud
    assert isinstance(make_hud("auto"), NullHud) and isinstance(make_hud("off"), NullHud)
    state = {"t": 300.0, "phase": "acoustic", "mission": {"active_pile": "A2"},
             "site": {"bounds": [-36, -54, 36, 36]}, "piles": [{"id": "A2", "xy": [-21.6, 0.7]}],
             "acoustic_clips": [{"listen": 1, "pile_id": "A3", "drone_id": "d1", "bearing_deg": -64, "width_deg": 24,
                                 "class": "voice", "transcript": "Help!", "priority": 77,
                                 "scores": {"human": 0.9}}],
             "pings": [{"id": "A2-R1", "t_found": 275.0, "dig_score": 0.41,
                        "vitals": {"condition": "irregular_apneic", "rate_bpm": 9.0, "triage_hint": "immediate (check)"}}],
             "dispatch": [{"t": 381.0, "to_pile": "B1", "reason": "weak voice + radar coverage gap"}]}
    h = Hud("auto", bounds=state["site"]["bounds"], piles=state["piles"])
    h.add_points(np.array([[-21.6, 0.7, 2.0], [-20.0, 1.0, 2.0]]))
    p = h.update(state, None, 1000, now=1.0, force=True)
    assert "irregular breathing with pauses - check first" in p["vitals"]["text"]
    assert "not a diagnosis" in p["vitals"]["text"]
    assert "B1" in p["decision"]["text"] and "A2-R1" in p["decision"]["text"] and "[by: argmax]" in p["decision"]["text"]
    st2 = dict(state, pings=[{"id": "A2-R1", "t_found": 275.0, "dig_score": 0.4, "vitals": {"condition": "unconscious_slow"}}],
               acoustic_clips=[{"listen": 2, "pile_id": "A1", "drone_id": "d2", "bearing_deg": 90, "width_deg": 20,
                                "class": "voice", "transcript": "", "priority": 56, "scores": {"human": 0.8}}],
               decisions=[{"t": 381.0, "by": "gemini"}])
    p2 = HudModel(bounds=state["site"]["bounds"]).update(st2)
    assert "deep rest or reduced consciousness" in p2["vitals"]["text"] and "possibly unconscious" not in p2["vitals"]["text"]
    assert "voice-like, language not identified" in p2["listen"]["text"]                 # nothing promised
    assert "[by: gemini]" in p2["decision"]["text"]
    assert p["listen"]["cones"][0]["drone"] == "d1" and p["lidar"]["coverage"] > 0
    assert HudModel(speed=8).update(state)["header"]["text"].count("sped up") == 1
    assert _no_omni()


def test_hud_radar_verdict_gated():
    """Radar panel: 'analysing breathing... (n s)' until 45 s of dwell, then the whole-dwell verdict; with no live
    scan it plays the synthetic library's reference stream, labelled NOT this scan."""
    from isaac.hud import HudModel, load_signature
    if load_signature("irregular_apneic") is None:
        pytest.skip("assets/radar_signatures not present")
    st = {"t": 280.0, "pings": [{"id": "A2-R1", "t_found": 275.0, "vitals": {"condition": "irregular_apneic"}}]}
    h = HudModel()
    early = h.update(st, None, 0, now=0.0, sim_t=280.0)["radar"]
    assert early["source"] == "library" and early["verdict"] is None and "analysing breathing... (5 s)" in early["text"]
    assert "NOT this scan" in early["text"] and early["wave"]
    late = h.update(st, None, 0, now=1.0, sim_t=275.0 + 50.0)["radar"]
    assert late["verdict"]["condition"] == "irregular_apneic" and "check first" in late["text"]
    live = {"last_scan": {"t": 100.0, "drone_id": "d2", "scan_id": "s7", "pile": "A2", "quality": 0.9,
                          "sig": {"fps": 10.0, "dwell_s": 20.0, "range_m": 2.1, "wave_mm": list(np.sin(np.arange(200) / 6.0)),
                                  "final": {"source": "machinery", "condition": "unknown", "conf": 0.9,
                                            "text": "pure tone with harmonics", "rate_bpm": None}}}}
    r = h.update(st, live, 0, now=2.0, sim_t=110.0)["radar"]
    assert r["source"] == "live" and r["verdict"] is None and len(r["wave"]) == 101
    r = h.update(st, live, 0, now=3.0, sim_t=125.0)["radar"]       # a 20 s dwell: its verdict once it is all in
    assert r["verdict"]["source"] == "machinery" and "not dug for" in r["text"]


def test_pose_track_is_smooth():
    """Render-time poses: continuous velocity across step boundaries, no dip below a landing pad."""
    from isaac.run_mission import PoseTrack
    tr = PoseTrack(use_motion=False)
    zs = [9.0, 9.0, 6.0, 3.0, 0.2, 0.2, 0.2]
    xs = [0.0, 4.0, 8.0, 10.0, 10.0, 10.0, 10.0]
    for k, (x, z) in enumerate(zip(xs, zs)):
        tr.push({"t": float(k), "drones": [{"id": "d1", "pose": [x, 0.0, z, 90.0], "state": "flying"}]})
    ts = np.linspace(0.0, 6.0, 601)
    P = np.array([tr.poses(t)["d1"][:3] for t in ts])
    V = np.array([tr.poses(t)["d1"][5] for t in ts])
    assert P[:, 2].min() >= 0.2 - 1e-9                       # monotone: never below the pad
    assert np.abs(np.diff(V)).max() < 0.2                    # no velocity jumps at the step boundaries
    assert np.abs(np.diff(P[:, 0])).max() < 0.06


def test_take_camera_never_cuts():
    from isaac.live import TakeCamera
    pile_xy = {"A2": (-21.6, 0.7), "C3": (22.0, -22.0)}
    cam = TakeCamera(None, pile_xy, base_xy=(0.0, -47.0), pads=[(-2.6, -49.0), (0.0, -49.0), (2.6, -49.0)],
                     van=(10.0, -49.0, 0.0, -90.0))
    poses = {"d1": (-21.0, 1.0, 9.0, 0.0, "hovering", 0.0, 0.0)}
    seq = [({"t": 0.0, "phase": "deploy"}, {"d1": (0.0, -49.0, 0.2, 0.0, "docked", 0.0, 0.0)}, None)] * 60 + \
          [({"t": 50.0, "phase": "survey"}, poses, "A2")] * 90 + \
          [({"t": 120.0, "phase": "radar", "pods": []}, poses, "A2")] * 60
    eyes = []
    for k, (st, ps, act) in enumerate(seq):
        if k == 200:
            cam.on_event((22.0, -22.0, 1.0), "H1 activated")
        cam.observe(st, ps, act)
        cam.update(1 / 30)
        eyes.append(cam.eye.copy())
    E = np.array(eyes)
    step = np.linalg.norm(np.diff(E, axis=0), axis=1)
    assert step.max() < 1.5                                   # < 45 m/s at 30 fps: a move, never a cut
    acc = np.linalg.norm(np.diff(E, 2, axis=0), axis=1)
    assert acc.max() < 0.05                                   # continuous velocity (spring-followed arcs)
    names = [s for _, s in cam.shots]
    assert names[0] == "van" and "survey" in names and "event" in names


def test_live_dryrun_with_trigger(tmp_path):
    """run_mission --dry-run --live with a scripted trigger (tests only): the hidden dummy switched on at t 90 is
    heard by the pod-mic quick listens (needs an earlier listen that heard A1 silent: the first one is at the pods' first
    dwell), the A2 dwell finishes and the fleet redirects to its sector; live.json and
    hud.json are written; no omni / pxr imported."""
    from isaac.live import write_default_tasking
    from isaac.run_mission import main
    out = tmp_path / "data"
    write_default_tasking(str(out))
    s = main(["--dry-run", "--live", "--camera", "director-take", "--acoustic", "stub", "--max-steps", "175",
              "--out", str(out), "--test-trigger", "activate:hidden2@90", "--quiet"])
    lv = s["live"]
    assert lv["world_changed"] and lv["events"] and "activated" in lv["events"][0]
    assert lv["new_evidence"] and lv["replans"]
    assert lv["visited"][:2] == ["A2", "A1"], lv["visited"]
    t_event = float(lv["events"][0].split()[0][2:])
    t_redirect = next(d[0] for d in lv["dispatch"] if d[2] == "A1")
    assert t_redirect - t_event < 60                          # sim s; ~3-4 s of video at the take pace
    live = json.loads((out / "live.json").read_text())
    assert live["schema"] == "larpverse.live/v1" and live["world"]["changed"]
    hud = json.loads((out / "hud.json").read_text())
    assert {"lidar", "listen", "radar", "vitals", "decision", "event"} <= set(hud["panels"])
    rad = hud["panels"]["radar"]
    assert rad["wave"] and rad["source"] == "live"            # the pod scan's own slow-time signal
    assert rad["verdict"] is not None or "analysing breathing" in rad["text"]
    sig = live["last_scan"]["sig"]
    assert sig["final"]["source"] and len(sig["wave_mm"]) >= 10 * 20
    assert s["camera_shots"][0][1] == "van"
    # judging: perception-driven action, adaptation, autonomy - all visible
    assert live["badge"].startswith(("LIVE INPUT", "AUTONOMOUS"))
    assert live["badge"] != "AUTONOMOUS \u00b7 no operator input"            # the live input was registered
    assert live["banner"] and live["banner"]["text"].startswith("PLAN CHANGED: A2 \u2192 A1")
    assert any("BECAUSE" in c["text"] for c in live["causal"])
    assert {"causal", "banner", "badge"} <= set(hud["panels"])
    st = json.loads((out / "state.json").read_text())                        # speaker audio muted for the take
    assert st.get("broadcasts") is not None and not any(b.get("audio_url") for b in st["broadcasts"])
    assert _no_omni()


def test_test_trigger_refused_outside_dry_run():
    from isaac.run_mission import parse_args
    with pytest.raises(SystemExit):
        parse_args(["--live", "--test-trigger", "activate:hidden1@10", "--launcher", "kit"])


def test_real_acoustic_preflight_fails_loudly(tmp_path):
    """No .venv-audio -> the service refuses to start (the take never silently falls back to the stub); the live
    sources go to the pipeline as the dummies' own WAVs."""
    from isaac.live import AcousticService, ListSource, LiveWorld, _sources_payload, dummies_from_scenario
    from sim.scenario import make_scenario
    sc = make_scenario(7)
    with pytest.raises(RuntimeError, match="setup.sh"):
        AcousticService(sc, python=str(tmp_path / "no-python")).start()
    hm = sc.heightmap(0.5)
    w = LiveWorld(sc, ListSource(dummies_from_scenario(sc, hm.height_at)), surface=hm.height_at, log=lambda s: None)
    src = _sources_payload(w.truth())
    if src is None:
        pytest.skip("dummy audio assets not present")
    assert all(os.path.exists(x["clip"]) and x["clip"].endswith(".wav") for x in src)
    assert {x["kind"] for x in src} >= {"voice", "weak_voice", "knock"}


def test_causal_lines_and_rejection():
    from isaac.hud import HudModel, causal_lines
    st = {"t": 500.0,
          "dispatch": [{"t": 494.0, "from_pile": "A2", "to_pile": "A1", "reason": "x", "by": "argmax"}],
          "acoustic_clips": [{"listen": 2, "pile_id": "A1", "class": "voice", "doa_agree": "3/3", "priority": 84}],
          "pings": [{"id": "A2-R1", "t_found": 364.0, "support": 3, "evidence": ["s4", "s5", "s6", "s7", "s9"],
                     "vitals": {"rate_bpm": 9.0}}],
          "piles": [{"id": "C3", "radar_note": "radar return rejected: level shift, settling debris (not a person)"}],
          "pods": [{"drone_id": "d2", "xy": [-21.0, 3.0], "status": "scanning"}],
          "landing_sites": [{"id": "A1-P3", "xy": [-21.2, 3.1], "pod": True, "slope_deg": 6.2, "roughness": 0.012,
                             "winch_m": 6.8}]}
    lines = [c["text"] for c in causal_lines(st, n=10)]
    assert any(l.startswith("d1-d3 \u2192 A1 BECAUSE voice heard, 3/3 drones agree on bearing") for l in lines), lines
    assert any(l.startswith("PING A2-R1 BECAUSE 3 agreeing radar ranges from 5 scans, breathing 9/min") for l in lines)
    assert any("pod d2 placed at A1-P3 BECAUSE best-scored flat seat on the LiDAR map" in l for l in lines)
    assert any(l == "C3: not a person: settling debris \u2192 no dig" for l in lines)
    lj = {"last_scan": {"t": 501.0, "scan_id": "s12", "sig": {"final": {"source": "machinery"}}},
          "badge": "AUTONOMOUS \u00b7 no operator input", "banner": {"text": "PLAN CHANGED: B2 \u2192 A1", "since": 0.0}}
    p = HudModel().update(st, lj, now=3.0)
    assert "radar s12: not a person: machinery / generator vibration \u2192 no dig" in p["causal"]["text"]
    assert p["banner"]["active"] and HudModel().update(st, lj, now=9.0)["banner"]["text"] == ""   # ~5 s on screen
    assert not p["badge"]["live"]
