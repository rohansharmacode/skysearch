"""Loudspeaker call-out, reassurance and guidance (sim/callout.py, sim/guidance.py). Fast, numpy only."""
import math

import numpy as np
import pytest

from sim import callout as co
from sim import guidance as gd
from sim.scenario import BASE_XY, make_scenario
from sim.world import Grid


def ring(cx, cy, r=10.0):
    return {f"d{i + 1}": (cx + r * math.cos(a), cy + r * math.sin(a)) for i, a in enumerate([0.3, 2.4, 4.5])}


class FakeScenario:
    """One victim per pile, no other sounds: hidden truth for the generator only."""

    bystanders = False

    def __init__(self, state, depth=0.8, seed=1):
        self.seed = seed
        self.piles = [type("P", (), {"name": "B2", "x": 0.0, "y": 0.0})(),
                      type("P", (), {"name": "C2", "x": 22.0, "y": 0.0})()]
        self.truth = {"victims": [{"id": "V1", "pile": "B2", "xy": [1.0, 0.5], "depth": depth, "sound": None,
                                   "physio": {"state": state}}], "sounds": []}


def run_state(state, seed, depth=0.8):
    sc = FakeScenario(state, depth, seed)
    sectors = {"B2": (0.0, 0.0), "C2": (22.0, 0.0)}
    plan = co.plan_callout(100.0, ring(0, 0), {"type": "sector", "id": "B2", "xy": (0.0, 0.0)})
    obs = co.simulate_callout(sc, plan, seed)
    return co.analyse_callout(plan, obs["detections"], sectors)["sectors"]["B2"]


@pytest.mark.parametrize("state", ["unconscious_slow", "irregular_apneic"])
def test_unconscious_never_responds(state):
    for seed in range(40):
        r = run_state(state, seed)
        assert r["status"] == "no_response", (seed, r)
        assert "does not rule out an unconscious person" in r["text"]


def test_calm_usually_responds():
    st = [run_state("calm", s)["status"] for s in range(40)]
    assert st.count("responded") >= 28, st


def test_distressed_responds_less_than_calm_but_sometimes():
    calm = sum(run_state("calm", s)["status"] == "responded" for s in range(30))
    dis = sum(run_state("distressed_tachypnea", s)["status"] == "responded" for s in range(30))
    assert 0 < dis < calm


def test_empty_sectors_never_marked_responded_seed_sweep():
    """Evaluator-side check over real scenarios: a sector with no person never reads 'responded'."""
    for seed in range(1, 11):
        sc = make_scenario(seed)
        sectors = {p.name: (p.x, p.y) for p in sc.piles}
        occupied = {v["pile"] for v in sc.truth["victims"]}
        for name in sectors:
            if name in occupied:
                continue
            plan = co.plan_callout(100.0, ring(*sectors[name]), {"type": "sector", "id": name, "xy": sectors[name]})
            r = co.analyse_callout(plan, co.simulate_callout(sc, plan, seed)["detections"], sectors)
            assert r["sectors"][name]["status"] != "responded", (seed, name, r["sectors"][name])


def test_analysis_never_sees_truth():
    """analyse_callout gets detections + public geometry only; its inputs carry no truth keys."""
    import inspect
    src = inspect.getsource(co.analyse_callout) + inspect.getsource(co._group) + inspect.getsource(co._associate)
    assert "truth" not in src and "scenario" not in src
    plan = co.plan_callout(0.0, ring(0, 0), {"type": "sector", "id": "B2", "xy": (0, 0)})
    out = co.analyse_callout(plan, [], {"B2": (0, 0)})
    assert out["sectors"]["B2"]["status"] == "no_response"


def test_knock_x3_is_recognised_and_time_locked():
    plan = co.plan_callout(100.0, ring(0, 0), {"type": "sector", "id": "B2", "xy": (0, 0)})
    t = plan["segments"][0]["t1"] + 1.5
    dets = []
    for k in range(3):
        for did, (x, y) in plan["drones"].items():
            dets.append({"drone_id": did, "t0": t + 4 * k, "t1": t + 4 * k + 0.7, "class": "knocking",
                         "class_conf": 0.8, "n_pulses": 3, "pulse_period_s": 0.3,
                         "bearing_deg": math.degrees(math.atan2(0.5 - y, 1.0 - x)) % 360, "snr_db": 15.0})
    r = co.analyse_callout(plan, dets, {"B2": (0, 0), "C2": (22, 0)})["sectors"]["B2"]
    assert r["status"] == "responded" and r["kind"] == "knock_x3" and r["avpu"] == "V"
    assert "knock pattern x3" in r["text"] and r["events"][0]["latency_s"] == pytest.approx(1.5, abs=0.01)


def test_triage_fusion_uses_vitals_conditions_and_ranks():
    from sim.vitals import TRIAGE_RANK
    no = {"status": "no_response", "message_reached": True, "avpu": "not V"}
    yes = {"status": "responded", "confidence": 0.8, "sector": "B2", "avpu": "V", "kind": "knock_x3"}
    slow = {"condition": "unconscious_slow", "triage_hint": "urgent", "condition_text": "slow, regular breathing"}
    f = co.fuse_triage(slow, no)                                # the AVPU check the radar asks for
    assert f["triage_hint"] == "immediate" and f["changed_from"] == "urgent" and f["rank"] == TRIAGE_RANK["immediate"]
    assert co.fuse_triage(slow, yes)["triage_hint"] == "urgent"   # a response never lowers an abnormal condition
    f = co.fuse_triage({"condition": "irregular_apneic", "triage_hint": "immediate (check)"}, no)
    assert f["triage_hint"] == "immediate"
    assert co.fuse_triage({"condition": "calm", "triage_hint": "delayed"}, yes)["triage_hint"] == "delayed"
    assert co.fuse_triage({"condition": "calm", "triage_hint": "delayed"}, no)["triage_hint"] == "immediate (check)"
    assert co.fuse_triage({"condition": "unknown", "triage_hint": "assess"}, no)["triage_hint"] == "assess"
    assert co.fuse_triage({"triage_hint": "assess"}, {"status": "possible_response"})["changed_from"] is None
    assert co.fuse_triage(slow, dict(no, message_reached=False))["triage_hint"] == "urgent"
    for cond in ("calm", "distressed_tachypnea", "unconscious_slow", "irregular_apneic", "unknown"):
        for hint in TRIAGE_RANK:
            for r in (no, yes, None):
                out = co.fuse_triage({"condition": cond, "triage_hint": hint}, r)
                assert out["triage_hint"] in TRIAGE_RANK and TRIAGE_RANK[out["triage_hint"]] == out["rank"]
                if r is no:
                    assert out["rank"] >= TRIAGE_RANK[hint]          # no response never lowers priority
    import inspect
    assert "child" not in inspect.getsource(co).lower()


def test_association_boost():
    yes = {"status": "responded", "confidence": 0.8, "sector": "B2", "avpu": "V", "kind": "knock_x3"}
    r, why = co.association_boost(0.45, yes)
    assert r > 0.45 and "call-and-response" in why
    assert co.association_boost(0.45, {"status": "no_response"}) == (0.45, None)
    assert co.response_priority(yes) >= 20


# ----------------------------------------------------------------------------- templates
def test_all_templates_present_for_five_languages():
    assert set(gd.LANGS) == {"zh-TW", "nan-TW", "en", "id", "vi"}
    for key in ("callout", "reassure"):
        for lang in gd.LANGS:
            assert gd.MESSAGES[key][lang].strip()
    for lang in gd.LANGS:
        g = gd.GUIDANCE[lang]
        assert len(g["dirs"]) == 8 and g["dests"]["tent"]
        for k in ("intro", "first", "next", "dest", "close"):
            assert k in g
        assert "{dir}" in g["first"] and "{dist}" in g["first"] and "{dest}" in g["dest"]
    legs = [{"bearing_deg": 0.0, "dist_m": 48.0}, {"bearing_deg": 270.0, "dist_m": 31.0}]
    t = gd.guidance_texts(legs)
    assert t["en"].startswith("Keep clear of the rubble. Walk east about 50 metres, then south about 30 metres")
    assert "往東走大約50公尺" in t["zh-TW"] and "東爿" in t["nan-TW"] and "timur" in t["id"] and "đông" in t["vi"]


def test_compass_follows_contract_bearings():
    assert gd.compass_index(0) == 0 and gd.GUIDANCE["en"]["dirs"][gd.compass_index(90)] == "north"
    assert gd.GUIDANCE["en"]["dirs"][gd.compass_index(225)] == "south-west"


# ----------------------------------------------------------------------------- route
def mound_grid():
    """Flat 60 x 60 m site with a 4 m mound in the middle (rubble)."""
    res = 0.5
    n = 120
    g = Grid((-30.0, -30.0), res, n, n, np.zeros((n, n), np.float32))
    X, Y = g.mesh()
    g.z = (4.0 * np.exp(-((X ** 2 + Y ** 2) / 40.0))).astype(np.float32)
    return g


def test_guidance_path_avoids_rubble_and_zones():
    g = mound_grid()
    zone = [[-25, 5], [-5, 5], [-5, 25], [-25, 25]]
    cost, origin, r, rubble, zmask = gd.cost_map(g, [zone])
    route = gd.guidance_route(g, (-20.0, 0.0), (20.0, 0.0), zones=[zone])
    assert route["ok"] and route["crosses_blocked_m"] == 0
    for x, y in route["path"]:
        i, j = int((x - origin[0]) / r), int((y - origin[1]) / r)
        i, j = min(max(i, 0), cost.shape[1] - 1), min(max(j, 0), cost.shape[0] - 1)
        assert not rubble[j, i] and not zmask[j, i]
    assert 1 <= len(route["legs"]) <= 3 and route["heading_deg"] is not None
    straight = gd.guidance_route(mound_grid(), (-20.0, -20.0), (20.0, -20.0))
    assert straight["legs"][0]["compass"] == "east" and abs(straight["length_m"] - 40) < 2


def test_route_on_seed7_site_reaches_staging():
    sc = make_scenario(7)
    route = gd.guidance_route(sc.heightmap(0.5), (11.0, 31.4), BASE_XY)
    assert route["ok"] and route["crosses_blocked_m"] == 0 and route["path"][-1] == [BASE_XY[0], BASE_XY[1]]


# ----------------------------------------------------------------------------- broadcast safety
def test_no_llm_text_broadcast_without_confirmation():
    log = gd.BroadcastLog()

    def fake_llm(prompt, schema, system=None):
        return {"translations": [{"lang": "zh-TW", "text": "請往南走50公尺。"},
                                 {"lang": "id", "text": "Berjalanlah ke selatan 50 meter."}]}

    made, tr = gd.operator_drafts(log, t=10.0, drone_id="d1", text="Walk south 50 metres.", langs=("en", "zh-TW", "id"),
                                  operator="op1", llm_call=fake_llm)
    by_lang = {b["lang"]: b for b in made}
    assert by_lang["en"]["status"] == "scheduled" and by_lang["en"]["source"] == "operator"
    assert by_lang["zh-TW"]["status"] == "pending_approval" and by_lang["id"]["status"] == "pending_approval"
    assert {b["lang"] for b in log.playable()} == {"en"}
    assert all(b["status"] != "played" for b in log.state(1e9) if b["source"] == "llm_translation")
    with pytest.raises(gd.BroadcastError):
        log.add(t=0, drone_id="d1", kind="operator", lang="vi", text="xin chao", text_en="hi",
                source="llm_translation", status="scheduled")
    log.approve(by_lang["zh-TW"]["id"], "op1")
    assert {b["lang"] for b in log.playable()} == {"en", "zh-TW"}
    with pytest.raises(gd.BroadcastError):
        log.approve(by_lang["zh-TW"]["id"], "op1")             # already approved


def test_bad_translation_is_dropped_and_templates_are_checked():
    assert gd.validate_translation("Walk 50 metres", "id", "Berjalan 60 meter") == ["numbers differ from the source"]
    assert "banned claim" in gd.validate_translation("Stay calm", "en", "Stay calm, you are confirmed found")
    log = gd.BroadcastLog()
    made, _ = gd.operator_drafts(log, t=0, drone_id="d1", text="Walk 50 metres.", langs=("en", "vi"), operator="op",
                                 llm_call=lambda *a, **k: {"translations": [{"lang": "vi", "text": "Đi 60 mét."}]})
    assert [b["lang"] for b in made] == ["en"]
    with pytest.raises(gd.BroadcastError):
        log.add(t=0, drone_id="d1", kind="callout", lang="en", text="Something else", text_en="x", template="callout")
    made, tr = gd.operator_drafts(gd.BroadcastLog(), t=0, drone_id="d1", text="Hello", langs=("en", "vi"),
                                  operator="op", llm_call=lambda *a, **k: None)
    assert [b["lang"] for b in made] == ["en"] and tr["vi"]["by"] == "none"


def test_speaker_facade_seed7_logs_broadcasts_and_reveals_by_time():
    sc = make_scenario(7)
    sectors = {p.name: (p.x, p.y) for p in sc.piles}
    spk = co.Speaker(sectors, sc.heightmap(0.5), BASE_XY, seed=7, asset_dir="/nonexistent")
    cx, cy = sectors["B1"]
    off, rec = spk.begin_listen(sc, 200.0, ring(cx, cy), {"type": "ping", "id": "B1-R1", "xy": (cx, cy)},
                                pings=[{"id": "B1-R1", "xy": [cx, cy], "pile_id": "B1"}])
    assert rec["listen_ends"] - 200.0 > 40                     # 5 languages + 20 s listen
    assert all(b["t"] + b["duration_s"] <= 200.0 + off + 1e-6 for b in spk.log.items)   # all played while landed
    kinds = [b["kind"] for b in spk.log.items]
    assert kinds[:5] == ["callout"] * 5 and "reassure" in kinds
    assert all(set(b) >= {"t", "drone_id", "kind", "lang", "text", "text_en", "audio_url", "target"} for b in spk.log.items)
    early = spk.state(205.0)
    assert len(early["broadcasts"]) == 1 and early["responsiveness"] == {}
    late = spk.state(200.0 + off + 300)
    assert "B1" in late["responsiveness"] and late["callouts"][0]["id"] == "C1"
    g = [b for b in spk.log.items if b["kind"] == "guidance"]
    if g:                                                    # W1 (synthetic bystander) heard on open ground
        assert g[0]["heading_deg"] is not None and g[0]["target"]["type"] == "track"


def test_operator_queue_roundtrip(tmp_path):
    q = str(tmp_path / "broadcast_queue.json")
    llm = lambda *a, **k: {"translations": [{"lang": "id", "text": "Tetap di tempat."}]}   # noqa: E731
    out = gd.queue_post(q, {"drone_id": "d2", "kind": "operator", "text": "Stay where you are.",
                            "langs": ["en", "id"], "operator": "op1"}, llm_call=llm)
    ids = {b["lang"]: b["id"] for b in out["items"]}
    with pytest.raises(gd.BroadcastError):
        gd.queue_post(q, {"drone_id": "d2", "text": "x"})                       # no operator name
    sc = make_scenario(7)
    spk = co.Speaker({p.name: (p.x, p.y) for p in sc.piles}, sc.heightmap(0.5), BASE_XY, asset_dir="/nonexistent")
    assert spk.ingest_operator(q, 50.0, {"d1": (0, 0), "d2": (1, 1)}) == 1   # only the operator's own text
    assert [b["lang"] for b in spk.log.playable()] == ["en"]
    gd.queue_decide(q, ids["id"], "op1")
    assert spk.ingest_operator(q, 60.0, {"d1": (0, 0), "d2": (1, 1)}) == 1
    assert {b["lang"] for b in spk.log.playable()} == {"en", "id"} and spk.log.playable()[1]["t"] == 60.0
    out = gd.queue_post(q, {"zone": "Z1", "kind": "callout", "operator": "op1", "langs": ["vi"]})
    assert out["items"][0]["text"] == gd.MESSAGES["callout"]["vi"] and out["items"][0]["status"] == "scheduled"
