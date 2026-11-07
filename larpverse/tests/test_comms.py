"""Rescue hotspot and comms relay (sim/comms.py, contract v1.4). Offline: no LLM key needed."""
import json
import math
import os
import re

import numpy as np
import pytest

import sim.comms as C
from sim.scenario import BASE_XY, make_scenario
from sim.grid_world import GridWorld

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAC_RE = re.compile(r"\b[0-9a-fA-F]{2}([:-])[0-9a-fA-F]{2}(\1[0-9a-fA-F]{2}){4}\b")


@pytest.fixture(scope="module")
def site():
    sc = make_scenario(7)
    gw = GridWorld(sc)
    return sc, gw


@pytest.fixture(autouse=True)
def _offline(monkeypatch):
    monkeypatch.setenv("LLM_PROVIDER", "none")      # the fallback path must work with no key / no quota


def _ring_drones(g, px, py, k, n=3):
    out = []
    for i in range(n):
        a = 2 * math.pi * i / n + k * 0.08
        r = 9 if k < 25 else 4
        x, y = px + r * math.cos(a), py + r * math.sin(a)
        z = g.height_at(x, y) + (10 if k < 25 else 0.6)
        out.append({"id": f"d{i + 1}", "pose": [x, y, z], "state": "flying" if k < 25 else "landed", "battery": 0.8})
    return out


def _run(sc, gw, steps_per_pile=50, piles=("A3", "B2", "A2"), seed=7):
    comms = C.Comms(sc, gw.grid, BASE_XY, sectors={k: {"xy": list(v)} for k, v in gw.pile_xy.items()}, seed=seed)
    events, t = [], 0.0
    for name in piles:
        px, py = gw.pile_xy[name]
        for k in range(steps_per_pile):
            events += comms.step(t, _ring_drones(gw.grid, px, py, k))
            t += 2.0
    return comms, events


# ----------------------------------------------------------------------------- link model
def test_link_model_monotone():
    d = np.linspace(1, 300, 400)
    pl = C.path_loss_db(d)
    assert np.all(np.diff(pl) >= 0)
    assert C.path_loss_db(10.0) == pytest.approx(20 * math.log10(10) + 20 * math.log10(C.FREQ_MHZ) - 27.55)
    down, up = C.link_margins(d, 0.0)
    assert np.all(np.diff(down) <= 0) and np.all(np.diff(up) <= 0)
    rub = np.linspace(0, 80, 50)
    dn2, up2 = C.link_margins(15.0, rub)
    assert np.all(np.diff(dn2) < 0) and np.all(np.diff(up2) < 0)
    assert np.all(np.diff(C.uplink_rssi(d, 10.0)) <= 0)
    assert C.associates(5.0, 0.0) and not C.associates(5000.0, 0.0)
    assert C.associates(8.0, 20.0) and not C.associates(8.0, 120.0)
    radii = [C.coverage_radius_m(8.0, r) for r in (0, 10, 20, 40, 80)]
    assert all(a > b for a, b in zip(radii, radii[1:]))
    assert 30 < C.coverage_radius_m(8.0) < 80        # 1 m of rubble: tens of metres, not hundreds


def test_mesh_multihop_relays_when_direct_link_fails():
    gcs = [0.0, 0.0, 3.0]
    far = {"d1": [1000.0, 0.0, 30.0], "d2": [500.0, 0.0, 30.0]}
    r = C.backhaul_routes(far, gcs)
    assert C.mesh_link(far["d1"], gcs)[1] == 0.0     # beyond direct range
    assert r["d1"]["next"] == "d2" and r["d1"]["hops"] == 2 and r["d2"]["next"] == "gcs"
    assert 0 < r["d1"]["e2e_mbps"] <= min(r["d1"]["mbps"], r["d2"]["mbps"])
    alone = C.backhaul_routes({"d1": [5000.0, 0.0, 30.0]}, gcs)
    assert alone["d1"]["next"] is None and alone["d1"]["mbps"] == 0.0
    near = C.backhaul_routes({"d1": [40.0, 0.0, 10.0]}, gcs)
    assert near["d1"]["next"] == "gcs" and near["d1"]["mbps"] == C.MESH_MAX_MBPS
    assert r["d1"]["latency_ms"] > r["d2"]["latency_ms"] > 0          # every hop adds latency
    via = C.backhaul_routes({"d1": [1000.0, 0.0, 30.0], "relay": [500.0, 0.0, 2.0]}, gcs)   # drop node relays
    assert via["d1"]["next"] == "relay" and via["relay"]["next"] == "gcs"


def test_store_and_forward_when_mesh_is_lost(site):
    sc, gw = site
    comms = C.Comms(sc, gw.grid, BASE_XY, seed=7, gcs_uplink="lte")
    near = [{"id": "d1", "pose": [0.0, -40.0, 15.0], "state": "flying", "battery": 0.9}]
    far = [{"id": "d1", "pose": [4000.0, 0.0, 15.0], "state": "flying", "battery": 0.9}]
    comms.step(0.0, near)
    ev = comms.step(10.0, far)
    assert ev[0]["type"] == "link_lost" and comms.drone_link("d1")["link"] == "stored"
    comms.step(20.0, far)
    q = comms.drone_link("d1")["queued_kb"]
    assert q == pytest.approx(2 * 10 * C.DRONE_DATA_KBPS) and comms.drone_link("d1")["last_link_t"] == 0.0
    ev = comms.step(30.0, near)
    assert ev[0]["type"] == "link_restored" and comms.drone_link("d1")["queued_kb"] < q
    with pytest.raises(C.CommsError):
        C.Comms(sc, gw.grid, BASE_XY, gcs_uplink="satellite")
    assert C.Comms(sc, gw.grid, BASE_XY, gcs_uplink=None).state()["backhaul"]["internet"] is False


# ----------------------------------------------------------------------------- localisation
def _trial(g, piles, i, buried=True, traj="ring"):
    rng = np.random.default_rng([i, 5])
    _name, (px, py) = piles[i % len(piles)]
    a = rng.uniform(0, 2 * np.pi)
    if buried:
        r = rng.uniform(0.5, 3.0)
        xy, depth = [px + r * np.cos(a), py + r * np.sin(a)], rng.uniform(0.8, 1.9)
    else:
        xy, depth = [px + 15 * np.cos(a), py + 15 * np.sin(a)], 0.0
    ph = {"person": "X", "kind": "buried" if buried else "surface", "xy": xy, "depth": depth, "z": None,
          "has_phone": True, "works": True, "aware": True, "battery_pct": 50, "drain_pct_h": 5,
          "rubble_db_per_m": float(np.clip(20 * np.exp(rng.normal(0, .45)), 6, 60)), "lang": "en", "react_s": 0,
          "tap_s": 1, "msg_s": 1, "shares_battery": False, "sends_msg": False, "msg_idx": 0, "token": "ph-x"}
    rs = C.RadioSim([ph], g, seed=i)
    reads = []
    for k in range(40):
        for d in range(3):
            if traj == "ring":
                p = _ring_drones(g, px, py, k)[d]["pose"]
            else:                                    # lawnmower survey at 10 m AGL
                u, row = (k % 20) / 19.0, k // 20
                x = px - 12 + 24 * (u if row % 2 == 0 else 1 - u)
                y = py - 10 + 8 * d + 4 * row
                p = [x, y, g.height_at(x, y) + 10]
            rssi, _dn, up = rs.link(rs.phones[0], f"d{d + 1}", p)
            if up >= 0:
                reads.append({"drone_id": f"d{d + 1}", "xyz": p, "dbm": rssi + rng.normal(0, 2), "t": k * 5})
    fit = C.localise(reads, g)
    return None if fit is None else (math.dist(fit["xy"], xy), fit["sigma_m"])


def _stats(res):
    ok = [r for r in res if r]
    e, s = np.array([r[0] for r in ok]), np.array([r[1] for r in ok])
    return len(ok), float(np.median(e)), float(np.percentile(e, 90)), float(np.mean(e <= 2.45 * s))


def test_rssi_localisation_error_statistics(site):
    """Synthetic: 24 buried phones (0.8-1.9 m deep, 6-60 dB/m rubble, 6 dB shadowing) seen by 3 drones that hover
    around and land on the pile. Honest numbers (docs/HOTSPOT.md): metres, not centimetres; sigma calibrated."""
    _sc, gw = site
    piles = list(gw.pile_xy.items())
    n, med, p90, cover = _stats([_trial(gw.grid, piles, i, True, "ring") for i in range(24)])
    assert n >= 18
    assert med < 4.0 and p90 < 14.0
    assert cover >= 0.75                               # truth inside the 95% radius (2.45 sigma) most of the time
    n, med, p90, cover = _stats([_trial(gw.grid, piles, i, True, "survey") for i in range(24)])
    assert n >= 10 and med < 10.0 and cover >= 0.7    # survey-only passes: worse, and the sigma says so


def test_localise_needs_geometry():
    one = [{"drone_id": "d1", "xyz": [0.0, 0.0, 10.0], "dbm": -60.0, "t": float(k)} for k in range(20)]
    assert C.localise(one) is None                     # one place: a range ring, not a position
    assert C.localise([]) is None


# ----------------------------------------------------------------------------- truth, determinism
def test_phone_truth_separate_stream_and_deterministic(site):
    sc, _gw = site
    before = json.dumps(sc.truth, sort_keys=True, default=str)
    a, b = C.phone_truth(sc, 7), C.phone_truth(sc, 7)
    assert a == b
    assert json.dumps(sc.truth, sort_keys=True, default=str) == before   # no side effects on the scenario
    assert make_scenario(7).truth == make_scenario(7).truth
    assert C.phone_truth(sc, 8) != a
    assert all(re.fullmatch(r"ph-[0-9a-f]{6}", p["token"]) for p in a)
    hook = C.attach_phone_truth(make_scenario(7))
    assert hook.truth["phones"] == a and hook.truth["victims"] == sc.truth["victims"]
    rates = [p["has_phone"] for s in range(40) for p in C.phone_truth(make_scenario(s), s)
             if s not in C.DEMO_PHONES]
    assert 0.5 < np.mean(rates) < 0.8


def test_comms_run_deterministic(site):
    sc, gw = site
    c1, e1 = _run(sc, gw)
    c2, e2 = _run(sc, gw)
    assert e1 == e2 and c1.state() == c2.state()
    assert any(e["type"] == "joined" for e in e1)


# ----------------------------------------------------------------------------- contract v1.4
def test_state_schema_v14(site):
    sc, gw = site
    comms, events = _run(sc, gw)
    st = json.loads(json.dumps(comms.state(), ensure_ascii=False))
    assert set(st) == {"backhaul", "hotspots", "devices", "portal"}
    bh = st["backhaul"]
    assert bh["kind"] == "mesh" and len(bh["gcs_xy"]) == 2 and isinstance(bh["internet"], bool)
    assert bh["uplink"] == "oneweb" and bh["internet"] is True
    for ln in bh["links"]:
        assert set(ln) == {"from", "to", "rssi_dbm", "mbps", "latency_ms"}
        assert (ln["to"] == "gcs" or ln["to"].startswith("d")) and ln["latency_ms"] >= C.HOP_LATENCY_MS
    for did in ("d1", "d2", "d3"):
        dl = comms.drone_link(did)
        assert set(dl) == {"link", "last_link_t", "queued_kb"} and dl["link"] in ("mesh", "stored")
    assert bh["links"], "landed drones near the staging area must reach the GCS"
    for h in st["hotspots"]:
        assert set(h) == {"drone_id", "ssid", "on", "xy", "z", "clients", "radius_m_est"}
        assert h["ssid"] == "RESCUE-WEIGUAN" and isinstance(h["on"], bool) and h["clients"] >= 0
    assert st["devices"]
    for d in st["devices"]:
        assert {"id", "first_seen", "last_seen", "est_xy", "est_sigma_m", "pile", "rssi", "status", "battery_pct",
                "messages", "replies"} <= set(d)
        assert d["status"] in ("trapped", "safe", "unknown")
        assert (d["est_xy"] is None) == (d["est_sigma_m"] is None)
        for r in d["rssi"]:
            assert set(r) == {"drone_id", "dbm", "t"}
        for m in d["messages"]:
            assert {"t", "text", "lang", "translation", "by"} <= set(m) and m["by"] in ("gemini", "fallback", "none")
            assert m.get("synthetic") is True          # sim messages are labelled synthetic
    assert set(st["portal"]) == {"url", "submissions"} and st["portal"]["submissions"] >= 1
    assert {e["type"] for e in events} <= {"joined", "left", "status", "message", "located", "evidence",
                                           "link_lost", "link_restored"}
    for e in events:
        if e["type"] == "evidence":
            assert e["pile"] in gw.pile_xy and e["sigma_m"] <= C.EVIDENCE_SIGMA_M and e["status"] != "safe"


def test_synthetic_messages_translate_offline():
    for kind, by_lang in C.SYNTH_MESSAGES.items():
        assert set(by_lang) == set(C.LANGS)
        for lang, msgs in by_lang.items():
            for i, m in enumerate(msgs):
                tr, by = C.translate_message(m, lang)
                assert tr == by_lang["en"][i] and by == ("none" if lang == "en" else "fallback")
    tr, by = C.translate_message("救命！我在樓梯，有3個人", "zh-TW")
    assert by == "fallback" and "help" in tr and "stairs" in tr and "number 3" in tr
    assert C.translate_message("zzz qqq", "vi") == (None, "none")
    fake = lambda prompt, schema, system=None: {"translation": "Help, 2 people under the stairs"}
    assert C.translate_message("救命，2個人在樓梯下", "zh-TW", use_llm=True, llm_call=fake)[1] == "gemini"
    bad = lambda prompt, schema, system=None: {"translation": "Help, 5 people"}       # numbers changed: rejected
    assert C.translate_message("救命，2個人", "zh-TW", use_llm=True, llm_call=bad)[1] == "fallback"


def test_replies_vetted_only(site):
    sc, gw = site
    comms, _ = _run(sc, gw)
    dev = comms.state()["devices"][0]["id"]
    for tpl in C.REPLY_TEMPLATES:
        assert set(C.REPLY_TEMPLATES[tpl]) == set(C.LANGS)
        r = comms.reply(dev, tpl, operator="op1", lang="tl")
        assert r["text"] == C.REPLY_TEMPLATES[tpl]["tl"] and r["confirmed_by"] == "op1"
    with pytest.raises(C.CommsError):
        comms.reply(dev, "help_coming", operator="")
    with pytest.raises(C.CommsError):
        comms.reply(dev, "we_found_you", operator="op1")
    assert comms.draft_free_text(dev, "Rescue in 20 minutes", "vi") is None          # no LLM: no machine text
    fake = lambda p, s, system=None: {"text": "Cứu hộ trong 20 phút"}
    draft = comms.draft_free_text(dev, "Rescue in 20 minutes", "vi", llm_call=fake)
    assert draft["status"] == "pending_approval"
    n = len(comms.devices[dev]["replies"])
    with pytest.raises(C.CommsError):
        comms.confirm_draft(draft, operator="")
    assert len(comms.devices[dev]["replies"]) == n                # nothing sent before confirmation
    comms.confirm_draft(draft, operator="op1")
    assert comms.devices[dev]["replies"][-1]["template"] == "free_text"
    with pytest.raises(C.CommsError):
        comms.draft_free_text(dev, "Survivor found, confirmed", "en")


# ----------------------------------------------------------------------------- portal API (pure + store)
GOOD = {"device": "a1b2c3d4e5f60718", "status": "trapped", "lang": "nan-TW", "message": "我佇一樓樓梯邊，拜託予我水。",
        "battery_pct": 23, "people": 2, "location": None}


def test_validate_portal():
    rec = C.validate_portal(dict(GOOD))
    assert rec["status"] == "trapped" and rec["lang"] == "nan-TW" and rec["battery_pct"] == 23
    assert C.validate_portal(dict(GOOD, lang="xx"))["lang"] == "und"
    assert C.validate_portal(dict(GOOD, message="a\x00b\x07c"))["text"] == "abc"
    fam = C.validate_portal(dict(GOOD, status="message", message="", family={"contact": "+886 912", "text": "I'm ok"}))
    assert fam["family"]["status"] == "queued_at_gcs"
    loc = C.validate_portal(dict(GOOD, location={"lat": 22.99, "lon": 120.2, "acc_m": 35}))["location"]
    assert loc == {"lat": 22.99, "lon": 120.2, "acc_m": 35.0}
    bad = [dict(GOOD, status="confirmed"), dict(GOOD, device="AB:CD"), dict(GOOD, device="x"),
           dict(GOOD, message="x" * 281), dict(GOOD, battery_pct=140), dict(GOOD, battery_pct=True),
           dict(GOOD, people=0), dict(GOOD, location={"lat": 95, "lon": 0}), dict(GOOD, status="message", message=""),
           dict(GOOD, message=5), "not a dict", dict(GOOD, family="x")]
    for b in bad:
        with pytest.raises(C.PortalError):
            C.validate_portal(b)


def test_portal_store_roundtrip_rate_limit_and_privacy(tmp_path):
    clock = [1000.0]
    path = str(tmp_path / "portal.json")
    with open(tmp_path / "state.json", "w") as f:
        f.write('{"schema":"larpverse.state/v1","t":412.0,"phase":"x","comms":{"backhaul":{"internet":true}}}')
    st = C.PortalStore(path, per_minute=3, per_hour=5, clock=lambda: clock[0])
    code, out = st.submit(dict(GOOD), "192.168.4.23")
    assert code == 200 and out["ok"]
    assert st.submit({"status": "bad"}, "192.168.4.23")[0] == 400
    for _ in range(2):
        assert st.submit(dict(GOOD), "192.168.4.23")[0] == 200
    assert st.submit(dict(GOOD), "192.168.4.23")[0] == 429             # 3 per minute
    assert st.submit(dict(GOOD), "10.42.0.77")[0] == 200                # per address
    clock[0] += 61
    assert st.submit(dict(GOOD), "192.168.4.23")[0] == 200
    clock[0] += 61
    assert st.submit(dict(GOOD), "192.168.4.23")[0] == 200             # 5th this hour
    clock[0] += 61
    assert st.submit(dict(GOOD), "192.168.4.23")[0] == 429             # 5 per hour
    raw = open(path, encoding="utf-8").read()
    assert "192.168" not in raw and "10.42.0.77" not in raw and st.addr_hash("192.168.4.23") not in raw
    code, dash = st.dashboard()
    assert code == 200 and set(dash) == {"submissions"}
    s0 = dash["submissions"][0]
    assert set(s0) == set(C.PortalStore.DASH_KEYS)
    assert s0["device"] == "ph-a1b2c3d4" and s0["t"] == 412.0 and s0["status"] == "trapped"
    assert s0["translation"] == "I am on the ground floor near the stairs. Water please." and s0["by"] == "fallback"
    assert st.status() == (200, {"internet": True})
    # operator replies -> comms_replies.json, phone reads only its own
    assert st.reply({"device": "ph-a1b2c3d4", "template": "stay_put", "lang": "nan-TW"})[0] == 200
    assert st.reply({"device": "ph-a1b2c3d4", "text": "Water drone in 10 min"})[0] == 400        # unconfirmed
    assert st.reply({"device": "ph-a1b2c3d4", "text": "We found you, confirmed", "confirmed_by": "op"})[0] == 400
    assert st.reply({"device": "ph-a1b2c3d4", "template": "nope"})[0] == 400
    assert st.reply({"device": "aa:bb:cc:dd:ee:ff", "template": "stay_put"})[0] == 400
    assert st.reply({"device": "ph-a1b2c3d4", "text": "Water drone in 10 min", "confirmed_by": "op2"})[0] == 200
    assert st.reply({"device": "ph-zzzz9999", "template": "help_coming"})[0] == 200
    code, inbox = st.inbox("a1b2c3d4e5f60718")
    assert code == 200 and [r["text"] for r in inbox["replies"]] == [C.REPLY_TEMPLATES["stay_put"]["nan-TW"],
                                                                     "Water drone in 10 min"]
    assert st.inbox("../etc")[0] == 400
    q = json.load(open(tmp_path / "comms_replies.json", encoding="utf-8"))
    assert q["schema"] == C.REPLIES_SCHEMA and all(i["status"] == "queued" for i in q["items"])


def test_mission_ingests_live_portal_and_replies(site, tmp_path):
    sc, gw = site
    path = str(tmp_path / "portal.json")
    st = C.PortalStore(path)
    st.submit(dict(GOOD), "127.0.0.1")
    comms = C.Comms(sc, gw.grid, BASE_XY, seed=7)
    ev = comms.ingest_portal(path, t=50.0)
    assert [e["type"] for e in ev] == ["status", "message"] and all(e["live"] for e in ev)
    d = comms.devices["ph-a1b2c3d4"]
    assert d["live"] and d["est_xy"] is None and d["status"] == "trapped" and d["messages"][0]["by"] == "fallback"
    assert comms.ingest_portal(path, t=60.0) == []                      # unchanged file: nothing new
    st.reply({"device": "ph-a1b2c3d4", "template": "help_coming", "lang": "nan-TW"})
    assert comms.ingest_replies(st.replies_path, t=70.0) == 1
    assert d["replies"][-1]["template"] == "help_coming" and comms.ingest_replies(st.replies_path) == 0


def test_no_mac_fields_anywhere(site, tmp_path):
    sc, gw = site
    comms, _ = _run(sc, gw)
    path = str(tmp_path / "portal.json")
    C.PortalStore(path).submit(dict(GOOD), "aa:bb:cc:dd:ee:ff")        # even an address that looks like a MAC
    comms.ingest_portal(path, t=1.0)
    blobs = [json.dumps(comms.state(), ensure_ascii=False), open(path, encoding="utf-8").read(),
             json.dumps(C.phone_truth(sc, 7), default=str)]

    def keys(o):
        if isinstance(o, dict):
            for k, v in o.items():
                yield k
                yield from keys(v)
        elif isinstance(o, list):
            for v in o:
                yield from keys(v)
    for b in blobs:
        assert not MAC_RE.search(b)
        assert not any(re.search(r"mac|bssid|imei|imsi|\bip\b|addr", k, re.I) for k in keys(json.loads(b)))
    src = open(os.path.join(ROOT, "sim", "comms.py"), encoding="utf-8").read()
    assert "probe" not in src.lower().replace("no probe-request", "").replace("probe paths", "")


def test_portal_page_offline_small_multilingual():
    path = os.path.join(ROOT, "dashboard", "portal", "index.html")
    html = open(path, encoding="utf-8").read()
    assert os.path.getsize(path) < 50 * 1024
    assert not re.search(r"(src|href)\s*=\s*[\"']?(https?:)?//", html)        # no CDN / external resources
    assert "@import" not in html and "fonts.googleapis" not in html
    for lang in C.LANGS:
        assert f'value="{lang}"' in html
    assert "/api/portal" in html and "navigator.getBattery" in html and "geolocation" in html
    assert 'name="viewport"' in html
