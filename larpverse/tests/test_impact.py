"""Impact model (sim/impact.py): survival curve, determinism, and like-for-like arms."""
import copy

import numpy as np

from sim import impact


def toy_row(pings=False, leads=False):
    sectors = [{"name": n, "kind": k, "area_m2": a, "centre": c} for n, k, a, c in (
        ("T1", "fallen_tower", 250.0, [0, 0]), ("T2", "fallen_tower", 300.0, [20, 0]),
        ("CN", "core_rubble", 600.0, [10, 15]), ("L1", "collapsed", 200.0, [60, 40]),
        ("M1", "mound", 150.0, [-50, -30]))]
    V = [("V1", "T1", "calm", 1.2), ("V2", "T2", "unconscious_slow", 1.5), ("V3", "CN", "irregular_apneic", 4.0),
         ("V4", "L1", "distressed_tachypnea", 0.8), ("V5", "M1", "unconscious_slow", 2.0)]
    victims = [{"id": i, "pile": s, "state": st, "responsive": st not in impact.NO_VOLUNTARY, "depth_m": d,
                "radar_reachable": d <= 3, "t_lead_s": (300.0 if leads else None),
                "t_ping_s": (900.0 if pings and st == "unconscious_slow" else None), "sound": None}
               for i, s, st, d in V]
    return {"seed": 0, "site": "toy", "start_pile": "T1", "sectors": sectors, "victims": victims,
            "sector_first_lead_s": ({s["name"]: 300.0 for s in sectors[:3]} if leads else {}),
            "flight_s": 5400.0, "mission_s": 3600.0, "battery_swaps": 1, "scans": 36, "false_pings": 0,
            "acoustic_source": "toy", "flight_model": "smooth", "version": {"head": "x", "sim_dirty_files": []}}


def test_survival_monotone_and_bounded():
    params, _ = impact.load_params()
    rng = np.random.default_rng(1)
    t = np.linspace(0, 200, 801)
    for p in [impact.central(params)] + [impact.draw(params, rng) for _ in range(50)]:
        s = impact.survival(t, p)
        assert s[0] == 1.0
        assert np.all(np.diff(s) <= 1e-12)
        assert np.all((s >= 0) & (s <= 1))
    assert impact.survival([np.inf], impact.central(params))[0] == 0.0


def test_draw_within_ranges():
    params, _ = impact.load_params()
    rng = np.random.default_rng(2)
    for _ in range(100):
        p = impact.draw(params, rng)
        for k, v in params.items():
            assert v["low"] - 1e-9 <= p[k] <= v["high"] + 1e-9, k


def test_deterministic():
    rows = [toy_row(pings=True, leads=True), toy_row()]
    a = impact.run_model(copy.deepcopy(rows), n=6, seed=5, tornado_m=2, progress=False)
    b = impact.run_model(copy.deepcopy(rows), n=6, seed=5, tornado_m=2, progress=False)
    assert a["headline"] == b["headline"]
    assert a["tornado"]["rows"] == b["tornado"]["rows"]


def perfect(params):
    p = impact.central(params)
    p.update(p_callout_responsive=1.0, p_callout_unresponsive=1.0, p_canine=1.0, p_camera=1.0,
             deep_detect_factor=1.0, acoustic_lead_keep=1.0, ping_keep=1.0)
    return p


def test_no_drone_evidence_means_same_as_today():
    params, _ = impact.load_params()
    sc = impact.Scenario(toy_row())
    for p in (impact.central(params), perfect(params)):
        e = impact.run_scenario_draw(sc, p, np.random.default_rng(3))
        assert np.array_equal(e["base"]["t_loc"], e["ours"]["t_loc"])
        assert np.isclose(e["base"]["p_alive"].sum(), e["ours"]["p_alive"].sum())


def test_perfect_baseline_equals_ours_when_timings_equal():
    """Baseline with perfect detection; give our drones pings at exactly the baseline's location times (no leads):
    both arms must then produce identical location times, extrications and survivors."""
    params, _ = impact.load_params()
    p = perfect(params)
    sc = impact.Scenario(toy_row())
    u = impact.victim_uniforms(sc.n, np.random.default_rng(4))
    t_base, _ = impact.locate(sc, p, u, "base")
    sc.t_ping = t_base - p["drone_deploy_h"] - p["pinpoint_h"]
    u_sec = np.ones(len(sc.order))                       # no acoustic lead kept (and none simulated)
    t_ours, m = impact.locate(sc, p, u, "ours", u_sec)
    assert np.allclose(t_ours, t_base)
    eb, eo = impact.outcome(sc, p, t_base, u), impact.outcome(sc, p, t_ours, u)
    assert np.allclose(eb[0], eo[0]) and np.isclose(eb[1].sum(), eo[1].sum())


def test_earlier_pings_never_hurt_survival_of_pinged():
    params, _ = impact.load_params()
    p = impact.central(params)
    p.update(ping_keep=1.0, acoustic_lead_keep=0.0)
    sc = impact.Scenario(toy_row(pings=True))
    e = impact.run_scenario_draw(sc, p, np.random.default_rng(6))
    pinged = np.isfinite(sc.t_ping)
    assert np.all(e["ours"]["t_loc"][pinged] <= e["base"]["t_loc"][pinged] + 1e-9)


def test_logistics_sane():
    params, _ = impact.load_params()
    lg = impact.logistics(impact.central(params))
    h = lg["half_tank_flight_min"]
    assert h["car alternator"] < h["inverter / truck generator"]
    d = lg["drones_sustained"]
    assert d["car alternator"] < 0.5 < d["fire truck, 8 kW hydraulic generator"] < d["heavy rescue truck, PTO generator"]
    assert 40 < h["car alternator"] < 120 and 150 < h["inverter / truck generator"] < 350   # IMPACT_EVIDENCE 6: 74 / 250


def test_entrapment_mix_and_skew():
    """Trapped mix 60/30/10 at the population's radar-only share; radar-only people skew to void / entombed."""
    params, _ = impact.load_params()
    p = impact.central(params)
    for w in (0.2, 0.35, 0.5):
        pa, pb = impact.type_probs(p, w)
        mix = impact.entrapment_mix(p)
        assert np.isclose(pa.sum(), 1) and np.isclose(pb.sum(), 1) and np.all(pa >= 0)
        assert pb[1] + pb[2] > mix[1] + mix[2] > pa[1] + pa[2]          # radar-only deeper than average, A shallower
        if w <= 0.35:                                                    # feasible: the overall mix is preserved
            assert np.allclose((1 - w) * pa + w * pb, mix)
    sc = impact.Scenario(toy_row())
    u = impact.victim_uniforms(sc.n, np.random.default_rng(9))
    t, h = impact.entrapment(sc, p, u)
    assert set(t) <= {0, 1, 2} and np.all(h > 0)
    keys = ("extrication_h_lightly_trapped", "extrication_h_void", "extrication_h_entombed_deep_pancake")
    assert np.allclose(h, np.array([p[k] for k in keys])[t] * u["ext"])


def test_v11_aliases():
    params, meta = impact.load_params()
    if (meta.get("version") or "") < "1.1":
        return
    for mine, theirs in (("canine_start_h", "canine_start_h_local"), ("extrication_teams",
                         "parallel_dig_worksites_per_building"), ("drone_deploy_h", "drone_airborne_after_quake_h"),
                         ("pinpoint_h", "alert_to_dig_start_h")):
        assert params[mine]["central"] == params[theirs]["central"] and params[mine]["origin"] != "placeholder"
