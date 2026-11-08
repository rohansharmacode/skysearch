"""Impact model: our drone search vs today's search in the first 72 h after a building collapse.

Three parts, all in this module (scripts/run_impact.py drives them, docs/IMPACT_MODEL.md explains them):

1. ``run_sim`` - EVALUATOR. Runs the real autonomous mission (``sim.grid_mission.GridMission``) on one seed and
   records, per hidden victim, the mission time of the first lead on their sector and of the first firm radar
   ping within 3 m (or never), plus whether the victim could answer a call-out. It reads ``scenario.truth``
   after / alongside the run for scoring only; the mission itself never sees it (RULES.md rule 9).
2. A Monte Carlo model of today's search (``baseline_locate``) and of the same search helped by the drones
   (``ours_locate``), on the SAME victims and sectors, with common random numbers per draw.
3. ``survival`` / ``outcomes``: time to locate + extrication -> probability alive at extrication (survival
   curve), rescuer-hours and cost; ``logistics`` for fuel -> flight minutes.

Parameters come from ``data/impact_params.json`` (researcher-owned, sourced, low / central / high). Anything the
file does not (yet) define falls back to ``PLACEHOLDERS`` below; every result records which parameters were
placeholders, and the report says so. This is a counterfactual model, not a claim about what happened.
"""
from __future__ import annotations

import copy
import json
import math
import os
import re
import subprocess

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PARAMS_PATH = os.path.join(ROOT, "data", "impact_params.json")
HIT_M = 3.0
NO_VOLUNTARY = ("unconscious_slow", "irregular_apneic")    # sim/callout.py: these never answer a call-out

# ----------------------------------------------------------------------------------------------- parameters
# PLACEHOLDERS: used only where data/impact_params.json has no entry of that name. Order-of-magnitude values
# from general USAR practice, NOT sourced; each result lists which of these were used.
_P = lambda lo, c, hi, unit, note="": {"low": lo, "central": c, "high": hi, "unit": unit, "note": note}
PLACEHOLDERS = {
    # survival of people extricated alive at time t (Coburn / Macintyre 2006 style table), % alive
    "survival_pct_30min": _P(85, 91, 95, "%"),
    "survival_pct_24h": _P(70, 81, 85, "%"),
    "survival_pct_48h": _P(30, 36.7, 45, "%"),
    "survival_pct_72h": _P(25, 33.7, 40, "%"),
    "survival_pct_96h": _P(12, 19, 25, "%"),
    "survival_pct_120h": _P(3, 7.4, 10, "%"),
    # today's search
    "search_start_h": _P(0.25, 0.5, 1.5, "h after collapse", "first organised search teams working the pile"),
    "search_teams": _P(4, 6, 12, "search crews at once",
                       "Weiguan: 4 sectors worked around the clock (IMPACT_EVIDENCE 2); more for a district"),
    "rescuers_per_search_team": _P(4, 6, 8, "people"),
    "hasty_search_rate_m2_h": _P(150, 300, 600, "m2 of rubble per team-hour",
                                 "call-out + listening pass; replaced by search_team_h_per_100m2_floor x hasty_share "
                                 "when the file has it"),
    "hasty_share": _P(0.1, 0.17, 0.3, "share of sector search effort",
                      "call-out / listening pass vs camera / void search"),
    "p_callout_responsive": _P(0.5, 0.7, 0.85, "probability", "call-out / listening finds a conscious, responsive victim"),
    "p_callout_unresponsive": _P(0.0, 0.02, 0.05, "probability", "call-out finds a silent / unconscious victim"),
    "canine_start_h": _P(1.0, 3.0, 8.0, "h after collapse"),
    "canine_teams": _P(2, 4, 8, "dog teams on this site", "Weiguan had 23 dogs for 2,000 rescuers"),
    "canine_rate_m2_h": _P(400, 800, 1600, "m2 per working dog-hour", "x duty cycle (6 h per 12 h shift)"),
    "p_canine": _P(0.4, 0.6, 0.8, "probability", "a dog alerts on a live victim, any state, < ~3 m deep"),
    "camera_rate_m2_h": _P(30, 60, 120, "m2 per team-hour", "void search with cameras, technical search"),
    "camera_start_h": _P(2.0, 6.0, 12.0, "h after collapse"),
    "p_camera": _P(0.3, 0.5, 0.7, "probability", "a search camera finds a victim, any state"),
    "debris_removal_start_h": _P(36, 64, 96, "h after collapse", "Weiguan heavy machinery from 64 h [S]"),
    "debris_removal_end_h": _P(144, 192, 240, "h after collapse", "Weiguan demolition done at 192 h [S]"),
    "pinpoint_h": _P(0.25, 0.75, 2.0, "h", "from an alert to a known dig spot (both arms)"),
    "extrication_h": _P(1.0, 4.0, 12.0, "h", "dig-out once the spot is known; same for both arms"),
    # our system
    "drone_deploy_h": _P(0.5, 1.0, 2.0, "h after collapse", "van on scene, drones up"),
    "ping_confirm_h": _P(0.1, 0.25, 0.5, "h", "a firm radar ping -> dig spot (camera / call-out check)"),
    # costs
    "cost_per_rescuer_h_usd": _P(40, 75, 150, "USD per rescuer-hour", "loaded cost incl. deployment"),
    "drone_price_usd": _P(50000, 61000, 87000, "USD per drone", "docs/HARDWARE.md"),
    "drone_life_flight_h": _P(300, 600, 1000, "flight hours"),
    "battery_pack_price_usd": _P(2500, 2800, 3200, "USD per DB2000", "docs/HARDWARE.md ~EUR 2,556"),
    "battery_cycles": _P(200, 400, 800, "cycles"),
    "van_price_usd": _P(40000, 51000, 70000, "USD", "docs/HARDWARE.md command van"),
    "van_life_deployments": _P(20, 50, 100, "deployments"),
    "operators": _P(2, 3, 4, "people per 3-drone van"),
    "operator_cost_h_usd": _P(40, 75, 150, "USD per operator-hour"),
    "electricity_usd_kwh": _P(0.1, 0.2, 0.4, "USD/kWh (fuel-equivalent)"),
    # energy / logistics
    "flight_power_kw_flycart30_loaded": _P(8.0, 9.7, 11.5, "kW", "docs/HARDWARE.md estimate"),
    "db2000_energy_wh": _P(1984.4, 1984.4, 1984.4, "Wh per pack (2 per aircraft)"),
    "inverter_gen_kw": _P(1.6, 1.8, 3.0, "kW rated (EU2200i class)"),
    "half_tank_L": _P(20, 25, 30, "L"),
    "alt_path_kwh_batt_per_L": _P(0.31, 0.5, 0.91, "kWh/L"),
    "gen_path_kwh_batt_per_L": _P(1.3, 1.7, 2.0, "kWh/L"),
    "battery_roundtrip_eff": _P(0.93, 0.95, 0.97, ""),
    "car_alternator_net_kw_at_idle": _P(0.6, 1.0, 1.6, "kW"),
    "firetruck_onboard_generator_kw": _P(8, 25, 40, "kW"),
    "drone_rotors_on_duty": _P(0.5, 0.65, 0.8, "share of time rotors on", "replaced by the simulated duty when known"),
    "charger_eff": _P(0.88, 0.9, 0.93, ""),
    "c8000_kw": _P(7.2, 7.2, 7.2, "kW", "C8000 max"),
    "hub_response_h": _P(0.5, 1.0, 2.0, "h", "city hub -> site by road"),
    "firetruck_response_h": _P(0.1, 0.25, 0.5, "h", "first fire truck on scene"),
    # Weiguan Jinlong 2016 (case-study anchor; researcher to replace)
    "weiguan_deaths": _P(115, 115, 115, "people"),
    "weiguan_rescued_alive": _P(175, 175, 175, "people"),
    "weiguan_extricated_total": _P(289, 290, 290, "people"),
    "weiguan_alive_day1": _P(164, 164, 164, "people"),
    "weiguan_alive_day2": _P(7, 7, 7, "people"),
    "weiguan_alive_day3": _P(4, 4, 4, "people"),
}


# model name -> name in data/impact_params.json (v1.0); the file's value replaces the placeholder
ALIASES = {
    "p_callout_responsive": "p_detect_acoustic_conscious",
    "p_callout_unresponsive": "p_detect_acoustic_unconscious",
    "p_canine": "p_detect_canine_live",
    "p_camera": "p_detect_camera_given_void_access",
    "extrication_h": "extrication_h_per_live_victim_rc",
    "cost_per_rescuer_h_usd": "cost_per_rescuer_hour_taiwan_usd",
    "flight_power_kw_flycart30_loaded": "flight_power_kw_loaded",
    "c8000_kw": "c8000_charger_kw",
}


def load_params(path=PARAMS_PATH):
    """{name: {low, central, high, unit, origin}}; origin = file kind (sourced|derived|assumption) or placeholder.
    The file's own names are kept too (they feed logistics and the Weiguan framing); ``ALIASES`` maps model names
    onto them; if the file has survival point values, the placeholder survival table is dropped."""
    out = {k: dict(v, origin="placeholder") for k, v in PLACEHOLDERS.items()}
    meta = {"path": os.path.relpath(path, ROOT), "exists": os.path.exists(path)}
    if meta["exists"]:
        with open(path) as f:
            doc = json.load(f)
        m = doc.get("_meta") or {}
        meta["version"] = m.get("version")
        meta["status"] = m.get("status") or m.get("conventions")
        meta["updated"] = m.get("updated")
        got = {}
        for p in doc.get("params", []):
            try:
                lo, c, hi = (float(p[k]) for k in ("low", "central", "high"))
            except (KeyError, TypeError, ValueError):
                continue
            lo, hi = min(lo, c, hi), max(lo, c, hi)
            got[p["name"]] = {"low": lo, "central": c, "high": hi, "unit": p.get("unit", ""),
                              "note": p.get("note", ""), "source_url": p.get("source_url", ""),
                              "origin": p.get("kind", "file")}
        if any(k.startswith("survival_pt_") for k in got):
            for k in [k for k in out if is_surv(k)]:
                del out[k]
            got = {k: v for k, v in got.items() if not is_surv(k) or k.startswith("survival_pt_")}
        else:
            got = {k: v for k, v in got.items() if not k.startswith("survival_window")}
        out.update(got)
        for mine, theirs in ALIASES.items():
            if theirs in got:
                out[mine] = dict(got[theirs], note=f"= {theirs}. " + got[theirs]["note"])
    return out, meta


def central(params):
    return {k: float(v["central"]) for k, v in params.items()}


def draw(params, rng, fixed=None):
    """One parameter vector: triangular(low, central, high) per parameter, independent, except the survival points,
    which share one quantile so the curve stays a curve."""
    fixed = fixed or {}
    out = {}
    u_surv = rng.uniform()
    for k, v in params.items():
        if k in fixed:
            out[k] = float(fixed[k])
            continue
        lo, c, hi = float(v["low"]), float(v["central"]), float(v["high"])
        u = u_surv if is_surv(k) else rng.uniform()
        out[k] = _tri_ppf(u, lo, c, hi)
    return out


def _tri_ppf(u, lo, c, hi):
    if hi <= lo:
        return lo
    fc = (c - lo) / (hi - lo)
    return lo + math.sqrt(u * (hi - lo) * (c - lo)) if u < fc else hi - math.sqrt((1 - u) * (hi - lo) * (hi - c))


# ------------------------------------------------------------------------------------------- survival curve
_SURV = re.compile(r"^survival_(?:pct_|pt_)(\d+(?:\.\d+)?)(min|h)(?:_pct)?$")


def is_surv(k):
    return bool(_SURV.match(k))


def survival_points(p):
    """[(hours, fraction alive at extrication)], starting at (0, 1) and made non-increasing."""
    pts = [(0.0, 1.0)]
    for key in sorted((k for k in p if is_surv(k)), key=_surv_hours):
        pts.append((_surv_hours(key), float(p[key]) / 100.0))
    t = [a for a, _ in pts]
    s = np.minimum.accumulate([b for _, b in pts])
    return list(zip(t, [float(x) for x in s]))


def _surv_hours(key):
    v, unit = _SURV.match(key).groups()
    return float(v) / 60.0 if unit == "min" else float(v)


def survival(t_h, p):
    """P(alive when extricated at t_h hours). Linear between the table points, flat after the last (a floor,
    which is generous to late rescues, i.e. conservative for us). Vectorised; NaN / inf (never) -> 0."""
    pts = survival_points(p)
    t = np.asarray(t_h, float)
    xs, ys = np.array([a for a, _ in pts]), np.array([b for _, b in pts])
    s = np.interp(np.where(np.isfinite(t), t, 0.0), xs, ys)
    return np.where(np.isfinite(t), s, 0.0)


# --------------------------------------------------------------------------------------------- simulations
def git_version():
    def g(*a):
        try:
            return subprocess.run(["git", *a], cwd=ROOT, capture_output=True, text=True, timeout=10).stdout
        except Exception:
            return ""
    dirty = [ln[3:] for ln in g("status", "--porcelain", "--", "sim", "acoustic").splitlines() if ln.strip()]
    return {"head": g("rev-parse", "--short", "HEAD").strip(), "sim_dirty_files": dirty}


def run_sim(seed, site="collapse", acoustic="stub", max_t=None, mission_scans=None, flight_model="smooth"):
    """EVALUATOR: one autonomous mission with the defaults (winch pod, literature penetration, physio breathing),
    then per-victim timings. Reads scenario.truth for scoring only."""
    import time

    from .evaluate_grid import score_run
    from .grid_mission import GridMission
    t0 = time.time()
    m = GridMission(seed=seed, site=site, acoustic=acoustic, verbose=False, max_t=max_t, mission_scans=mission_scans,
                    flight_model=flight_model)
    first_lead, why = {}, {}
    while not m.done:
        m.step()
        for n, p in m.piles.items():
            if n not in first_lead and (p["status"] in ("lead", "ping") or (p["priority"] or 0) >= m.priority_min):
                first_lead[n], why[n] = round(m.t, 1), p.get("why", "")
    sc = score_run(m)
    truth = m.scenario.truth                                      # EVALUATOR ONLY
    det = {d["id"]: d for d in sc["victim_detail"]}
    victims = []
    for v in truth["victims"]:
        st = (v.get("physio") or {}).get("state", "calm")
        d = det.get(v["id"], {})
        t_ping = d.get("t") if d.get("pinged") else None
        tl = first_lead.get(v["pile"])
        if t_ping is not None:
            tl = t_ping if tl is None else min(tl, t_ping)
        victims.append({
            "id": v["id"], "pile": v["pile"], "depth_m": round(float(v["depth"]), 2), "state": st,
            "sound": v.get("sound"), "responsive": st not in NO_VOLUNTARY,
            "radar_reachable": bool(v.get("radar_reachable", float(v["depth"]) <= 3.0)),
            "t_lead_s": tl, "lead_why": why.get(v["pile"], "")[:200] if tl is not None else None,
            "t_ping_s": t_ping, "ping_err_m": d.get("err_m")})
    pd = m.scenario.public_dict()
    sectors = [{"name": s["name"], "kind": s.get("kind"), "area_m2": s.get("area_m2"), "centre": s.get("centre")}
               for s in pd.get("sectors", [])] or [
        {"name": n, "kind": "pile", "area_m2": round(math.pi * 8.0 ** 2, 1), "centre": xy}
        for n, xy in ((n, p["xy"]) for n, p in m.piles.items())]
    return {"seed": seed, "site": site, "acoustic_source": m.acoustic_source, "acoustic_mode": acoustic,
            "flight_model": m.flight_model, "delivery": m.delivery, "platform": m.platform,
            "penetration": m.penetration, "mission_scans": m.mission_scans, "max_t": m.max_t,
            "start_pile": m.start_pile, "mission_s": round(m.t, 1), "scans": m.scans_used,
            "battery_swaps": m.n_swaps, "flight_s": round(m._fly_s, 1), "visits": list(m.visited),
            "false_pings": sc["false_pings"], "sector_first_lead_s": first_lead, "victims": victims,
            "sectors": sectors, "base_xy": pd.get("base_xy"), "version": git_version(),
            "wall_s": round(time.time() - t0, 1)}


# ------------------------------------------------------------------------------------- model assumptions
# Our own modelling knobs (not literature values): they discount the simulation, since its sensors are synthetic.
PLACEHOLDERS.update({
    "acoustic_lead_keep": _P(0.5, 0.8, 1.0, "share of simulated sector leads kept",
                             "the acoustic stub is built from hidden truth, cleaner than the real pipeline"),
    "ping_keep": _P(0.5, 0.8, 1.0, "share of simulated firm pings kept",
                    "through-rubble radar is a model; validated only line-of-sight on real data"),
    "extrication_teams": _P(3, 6, 12, "crews digging at once on the site"),
    "deep_detect_factor": _P(0.3, 0.5, 0.8, "x detection probability",
                             "call-out, dog and camera for people > 3 m inside the rubble"),
    "hazard_multiplier_unconscious_or_severe": _P(1.0, 1.5, 3.0, "x death hazard"),
    "frac_trapped_unconscious_at_rescue": _P(0.15, 0.28, 0.4, "fraction"),
})
CORE_KINDS = ("fallen_tower", "core_rubble", "pile")
ARMS = ("base", "ours", "ideal")        # today | ours as simulated | ours if the pod pings what it is sent to
METHODS = ["radar ping (ours)", "call-out / listening", "search dog", "search camera", "debris removal"]


def sector_order(row):
    """Today's search order: the reported sector first, then the collapse core nearest-first, then the rest of the
    district nearest to the core first. Same order for both arms (ours only moves lead sectors forward)."""
    secs = {s["name"]: s for s in row["sectors"]}
    start = row["start_pile"]
    c0 = np.asarray(secs[start]["centre"], float) if start in secs else np.zeros(2)
    core = [n for n, s in secs.items() if s.get("kind") in CORE_KINDS and n != start]
    rest = [n for n in secs if n not in core and n != start]
    cc = np.mean([secs[n]["centre"] for n in core + [start]], axis=0)
    core.sort(key=lambda n: float(np.hypot(*(np.asarray(secs[n]["centre"]) - c0))))
    rest.sort(key=lambda n: float(np.hypot(*(np.asarray(secs[n]["centre"]) - cc))))
    return ([start] if start in secs else []) + core + rest


class Scenario:
    """One simulated mission prepared for the Monte Carlo (arrays, hours)."""

    def __init__(self, row):
        self.row = row
        self.order = sector_order(row)
        self.idx = {n: k for k, n in enumerate(self.order)}
        area = {s["name"]: float(s.get("area_m2") or 200.0) for s in row["sectors"]}
        self.area = np.array([area[n] for n in self.order])
        V = row["victims"]
        self.n = len(V)
        self.sec = np.array([self.idx.get(v["pile"], 0) for v in V])
        self.responsive = np.array([bool(v["responsive"]) for v in V])
        self.deep = np.array([float(v["depth_m"]) > 3.0 for v in V])
        self.t_ping = np.array([np.nan if v["t_ping_s"] is None else v["t_ping_s"] / 3600 for v in V])
        lead = row.get("sector_first_lead_s", {})
        self.sec_lead = np.array([np.nan if lead.get(n) is None else lead[n] / 3600 for n in self.order])
        # upper bound ("ideal" arm): the pod firmly pings every silent / unconscious, radar-reachable victim in a
        # sector the drones had a lead on, by the end of the simulated sortie (if the simulation did not already)
        reach = np.array([bool(v["radar_reachable"]) for v in V])
        led = np.isfinite(self.sec_lead[self.sec]) if self.n else np.zeros(0, bool)
        self.ideal_eligible = (~self.responsive) & reach & led
        self.t_ping_ideal = np.where(self.ideal_eligible & ~np.isfinite(self.t_ping), row["mission_s"] / 3600,
                                     self.t_ping)
        self.flight_h = row["flight_s"] / 3600
        self.mission_h = row["mission_s"] / 3600


def _schedule(n_teams, queues, t0, prio_time=None):
    """List scheduling of search jobs over sectors in today's order.

    ``queues``: {kind: (durations per sector rank, earliest start h)}; the default order interleaves the kinds per
    sector (sector 1 kind a, sector 1 kind b, sector 2 kind a, ...), a job runs once its earliest start has passed.
    ``prio_time[sector]`` (ours): once a lead on that sector is known (time <= now), its jobs go first, earliest
    lead first. Returns {kind: (start array, end array)}."""
    import heapq
    kinds = list(queues)
    n_sec = len(next(iter(queues.values()))[0])
    st = {k: np.full(n_sec, np.nan) for k in kinds}
    en = {k: np.full(n_sec, np.nan) for k in kinds}
    head = {k: 0 for k in kinds}
    free = [float(t0)] * max(1, int(round(n_teams)))
    heapq.heapify(free)
    leads = [] if prio_time is None else sorted((float(t), s) for s, t in enumerate(prio_time) if t == t)
    left = n_sec * len(kinds)

    def advance(k):
        while head[k] < n_sec and st[k][head[k]] == st[k][head[k]]:
            head[k] += 1
    while left:
        now = heapq.heappop(free)
        pick = None
        for lt, s in leads:                                   # lead sectors first (ours)
            if lt > now:
                break
            for k in kinds:
                if st[k][s] != st[k][s] and queues[k][1] <= now:
                    pick = (k, s)
                    break
            if pick:
                break
        if pick is None:                                      # default order: earliest-ranked startable head
            best = None
            for j, k in enumerate(kinds):
                advance(k)
                if head[k] < n_sec and queues[k][1] <= now:
                    pos = head[k] * len(kinds) + j
                    if best is None or pos < best[0]:
                        best = (pos, k)
            if best is not None:
                pick = (best[1], head[best[1]])
        if pick is None:                                      # nothing startable: wait
            nxt = [queues[k][1] for k in kinds if queues[k][1] > now and head[k] < n_sec]
            nxt += [lt for lt, _ in leads if lt > now]
            heapq.heappush(free, min(nxt) if nxt else now + 1e-6)
            continue
        k, s = pick
        d = float(queues[k][0][s])
        st[k][s], en[k][s] = now, now + d
        heapq.heappush(free, now + d)
        left -= 1
    return {k: (st[k], en[k]) for k in kinds}


def _extricate(t_loc, ext_h, n_crews):
    """Crews dig people out in the order they are located; returns the extrication time per victim."""
    out = np.full(len(t_loc), np.inf)
    free = [0.0] * max(1, int(round(n_crews)))
    for i in np.argsort(t_loc, kind="stable"):
        if not np.isfinite(t_loc[i]):
            continue
        k = int(np.argmin(free))
        st = max(free[k], t_loc[i])
        out[i] = st + ext_h[i]
        free[k] = out[i]
    return out


def victim_uniforms(n, rng):
    return {k: rng.uniform(size=n) for k in ("pos_h", "det_h", "pos_c", "det_c", "pos_k", "det_k", "rem", "ping",
                                             "lead")} | {"ext": rng.lognormal(0.0, 0.5, size=n)}


def search_rates(p):
    """(call-out pass, camera pass) m2 per team-hour. With the sourced Weiguan calibration (sector-team-hours per
    100 m2, which includes extrication, so it is a slow, conservative-for-today search) it is split by hasty_share."""
    e = p.get("search_team_h_per_100m2_floor")
    if e:
        return 100.0 / (e * p["hasty_share"]), 100.0 / (e * (1.0 - p["hasty_share"]))
    return p["hasty_search_rate_m2_h"], p["camera_rate_m2_h"]


def locate(sc: Scenario, p, u, arm="base", u_sec=None):
    """Time to locate (h after the collapse) and the method, per victim, for one parameter draw.
    ``arm`` 'base' = today's search; 'ours' = the same teams, plus the drones' firm pings and lead-first ordering."""
    n_sec = len(sc.order)
    prio = None
    t_ping = np.full(sc.n, np.nan)
    if arm in ("ours", "ideal"):
        tpg = sc.t_ping_ideal if arm == "ideal" else sc.t_ping
        keep = (u_sec if u_sec is not None else np.zeros(n_sec)) < p["acoustic_lead_keep"]
        prio = np.where(keep, p["drone_deploy_h"] + sc.sec_lead, np.nan)
        kp = ((u["ping"] < p["ping_keep"]) | (arm == "ideal")) & np.isfinite(tpg)
        t_ping = np.where(kp, p["drone_deploy_h"] + tpg + p["ping_confirm_h"], np.nan)
        for i in np.where(kp)[0]:                        # a kept ping also puts its sector first
            s = sc.sec[i]
            tp = p["drone_deploy_h"] + tpg[i]
            prio[s] = tp if not np.isfinite(prio[s]) else min(prio[s], tp)
    hr, cr = search_rates(p)
    teams = _schedule(p["search_teams"], {"hasty": (sc.area / hr, p["search_start_h"]),
                                          "camera": (sc.area / cr, p["camera_start_h"])},
                      p["search_start_h"], prio)
    duty = p.get("canine_effective_search_h_per_12h_shift", 12.0) / 12.0
    dogs = _schedule(p["canine_teams"], {"dog": (sc.area / (p["canine_rate_m2_h"] * duty), p["canine_start_h"])},
                     p["canine_start_h"], prio)
    cand = np.full((sc.n, 5), np.inf)
    cand[:, 0] = np.where(np.isfinite(t_ping), t_ping, np.inf)
    pin = p["pinpoint_h"]
    for i in range(sc.n):
        s = sc.sec[i]
        f = p["deep_detect_factor"] if sc.deep[i] else 1.0
        a, b = teams["hasty"][0][s], teams["hasty"][1][s]
        pc = p["p_callout_responsive"] if sc.responsive[i] else p["p_callout_unresponsive"]
        if u["det_h"][i] < pc * f:
            cand[i, 1] = a + u["pos_h"][i] * (b - a) + pin
        a, b = dogs["dog"][0][s], dogs["dog"][1][s]
        if u["det_k"][i] < p["p_canine"] * f:
            cand[i, 2] = a + u["pos_k"][i] * (b - a) + pin
        a, b = teams["camera"][0][s], teams["camera"][1][s]
        if u["det_c"][i] < p["p_camera"] * f:
            cand[i, 3] = a + u["pos_c"][i] * (b - a) + pin
        cand[i, 4] = p["debris_removal_start_h"] + u["rem"][i] * max(0.0, p["debris_removal_end_h"]
                                                                    - p["debris_removal_start_h"])
    method = np.argmin(cand, axis=1)
    return cand[np.arange(sc.n), method], method


def outcome(sc, p, t_loc, u):
    """Extrication time and P(alive at extrication) per victim (same crews and dig times in both arms)."""
    t_ext = _extricate(t_loc, p["extrication_h"] * u["ext"], p["extrication_teams"])
    h = np.where(sc.responsive, 1.0, p.get("hazard_multiplier_unconscious_or_severe", 1.0))
    return t_ext, survival(t_ext, p) ** h


def run_scenario_draw(sc, p, rng):
    u = victim_uniforms(sc.n, rng)
    u_sec = rng.uniform(size=len(sc.order))
    res = {}
    for arm in ARMS:
        t, m = locate(sc, p, u, arm, u_sec)
        te, s = outcome(sc, p, t, u)
        res[arm] = {"t_loc": t, "method": m, "t_ext": te, "p_alive": s}
    return res


def rescuer_hours(p, t_loc, window_h=72.0):
    """Search rescuer-hours tied up before each person is located (mean per person, window-capped): the whole
    search force is busy searching while that person waits. Opportunity cost, not cash saved, unless the
    search phase actually ends sooner."""
    crew = p["search_teams"] * p["rescuers_per_search_team"]
    t = np.minimum(np.where(np.isfinite(t_loc), t_loc, window_h), window_h)
    return crew * np.maximum(0.0, t - p["search_start_h"])


def drone_costs(p, flight_h, mission_h, swaps, n_drones=3, standby_h=72.0):
    """USD for one simulated sortie (and for a 72 h deployment with operators on site the whole time)."""
    energy = flight_h * p["flight_power_kw_flycart30_loaded"] * p["electricity_usd_kwh"]
    airframe = flight_h * p["drone_price_usd"] / p["drone_life_flight_h"]
    packs = n_drones * 2 * (swaps + 1) * p["battery_pack_price_usd"] / p["battery_cycles"]
    van = p["van_price_usd"] / p["van_life_deployments"]
    ops_sortie = p["operators"] * p["operator_cost_h_usd"] * (mission_h + 1.0)
    ops_72 = p["operators"] * p["operator_cost_h_usd"] * standby_h
    hw = energy + airframe + packs
    return {"energy": energy, "airframe": airframe, "batteries": packs, "van": van, "operators_sortie": ops_sortie,
            "sortie_total": hw + van + ops_sortie, "deployment_72h_total": hw + van + ops_72}


# ------------------------------------------------------------------------------------------------ logistics
def logistics(p, duty=None):
    """Half a car tank -> flight minutes (alternator vs generator, the evidence file's fuel-to-battery paths), and how
    many drones each charger keeps flying continuously (charge power x charger efficiency / average demand, where
    demand = flight power x rotors-on duty; the duty comes from the simulation when given). ``p``: central values."""
    fp = p["flight_power_kw_flycart30_loaded"]
    duty = duty if duty is not None else p["drone_rotors_on_duty"]
    rt = p["battery_roundtrip_eff"]
    half = {"car alternator": p["half_tank_L"] * p["alt_path_kwh_batt_per_L"] * rt / fp * 60,
            "inverter / truck generator": p["half_tank_L"] * p["gen_path_kwh_batt_per_L"] * rt / fp * 60}
    chargers = {"car alternator": p["car_alternator_net_kw_at_idle"],
                "inverter generator (1.8 kW)": p["inverter_gen_kw"],
                "fire truck, 8 kW hydraulic generator": 8.0,
                "heavy rescue truck, PTO generator": p["firetruck_onboard_generator_kw"],
                "city hub (grid, 3 x C8000)": 3 * p["c8000_kw"]}
    sustained = {k: kw * p["charger_eff"] / (fp * duty) for k, kw in chargers.items()}
    sustained["city hub (grid, 3 x C8000)"] = min(sustained["city hub (grid, 3 x C8000)"], 3.0)   # 1 drone per C8000
    return {"half_tank_flight_min": half, "charge_kw": chargers, "drones_sustained": sustained,
            "demand_kw_per_drone": fp * duty / p["charger_eff"], "duty": duty, "flight_power_kw": fp,
            "flight_min_per_pack_pair": 2 * p["db2000_energy_wh"] / 1000 * 0.8 * rt / fp * 60}


def logistics_bands(params, duty=None):
    """{'low' (pessimistic), 'central', 'high' (optimistic)} logistics. Pessimistic = least fuel energy and charge
    power, most flight power; the half-tank minutes use the evidence file's own derived ranges when present."""
    c = central(params)
    pick = {"low": lambda k: params[k]["low"], "high": lambda k: params[k]["high"]}
    out = {"central": logistics(c, duty)}
    for end, other in (("low", "high"), ("high", "low")):
        q = dict(c)
        for k in ("half_tank_L", "alt_path_kwh_batt_per_L", "gen_path_kwh_batt_per_L", "battery_roundtrip_eff",
                  "car_alternator_net_kw_at_idle", "inverter_gen_kw", "firetruck_onboard_generator_kw", "c8000_kw",
                  "charger_eff"):
            q[k] = float(pick[end](k))
        q["flight_power_kw_flycart30_loaded"] = float(pick[other]("flight_power_kw_flycart30_loaded"))
        out[end] = logistics(q, duty)
    for k, name in (("car alternator", "half_tank_flight_min_alternator"),
                    ("inverter / truck generator", "half_tank_flight_min_generator")):
        if name in params:
            for end in ("low", "central", "high"):
                out[end]["half_tank_flight_min"][k] = float(params[name][end])
    return out

# ------------------------------------------------------------------------------------------------ the model
def _q(x, qs=(5, 50, 95)):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if not len(x):
        return {"median": None, "p5": None, "p95": None}
    a, b, c = np.percentile(x, qs)
    return {"median": float(b), "p5": float(a), "p95": float(c)}


def sim_summary(rows):
    """What the simulations show directly (no Monte Carlo)."""
    V = [v for r in rows for v in r["victims"]]
    lead = np.array([np.nan if v["t_lead_s"] is None else v["t_lead_s"] / 60 for v in V])
    ping = np.array([np.nan if v["t_ping_s"] is None else v["t_ping_s"] / 60 for v in V])
    unr = np.array([not v["responsive"] for v in V])
    reach = np.array([v["radar_reachable"] for v in V])
    core = np.array([v["pile"] in {n for r in rows for n, s in ((s["name"], s) for s in r["sectors"])
                                   if s.get("kind") in CORE_KINDS} for v in V])
    n = len(V)
    return {
        "n_seeds": len(rows), "n_victims": n, "victims_per_seed": n / max(len(rows), 1),
        "n_unresponsive": int(unr.sum()), "n_core": int(core.sum()), "n_radar_reachable": int(reach.sum()),
        "lead_share": float(np.isfinite(lead).mean()), "lead_share_core": float(np.isfinite(lead[core]).mean()),
        "lead_within_30min_share": float((lead <= 30).sum() / n), "lead_min": _q(lead),
        "ping_share": float(np.isfinite(ping).mean()), "pinged": int(np.isfinite(ping).sum()), "ping_min": _q(ping),
        "unresponsive_pinged": int((np.isfinite(ping) & unr).sum()),
        "unresponsive_lead": int((np.isfinite(lead) & unr).sum()),
        "ideal_eligible": int((np.isfinite(lead) & unr & reach).sum()),
        "false_pings": int(sum(r["false_pings"] for r in rows)),
        "scans_mean": float(np.mean([r["scans"] for r in rows])),
        "flight_h_mean": float(np.mean([r["flight_s"] for r in rows]) / 3600),
        "mission_h_mean": float(np.mean([r["mission_s"] for r in rows]) / 3600),
        "swaps_mean": float(np.mean([r["battery_swaps"] for r in rows])),
        "rotors_on_duty": float(np.mean([r["flight_s"] / (3 * max(r["mission_s"], 1)) for r in rows])),
        "acoustic_source": sorted({r["acoustic_source"] for r in rows}),
        "flight_model": sorted({r["flight_model"] for r in rows}),
        "site": sorted({r["site"] for r in rows}), "versions": sorted({r["version"]["head"] for r in rows}),
        "sim_dirty_files": sorted({f for r in rows for f in r["version"].get("sim_dirty_files", [])}),
        "per_seed": [{"seed": r["seed"], "victims": len(r["victims"]),
                      "unresponsive": sum(not v["responsive"] for v in r["victims"]),
                      "with_lead": sum(v["t_lead_s"] is not None for v in r["victims"]),
                      "pinged": sum(v["t_ping_s"] is not None for v in r["victims"]),
                      "scans": r["scans"], "swaps": r["battery_swaps"], "flight_h": r["flight_s"] / 3600,
                      "mission_min": r["mission_s"] / 60} for r in rows],
    }


def evaluate(scs, p, rng):
    """One parameter draw over all scenarios -> pooled per-victim arrays per arm."""
    out = {a: {k: [] for k in ("t_loc", "method", "t_ext", "p_alive")} for a in ARMS}
    resp = []
    for sc in scs:
        r = run_scenario_draw(sc, p, rng)
        for a in ARMS:
            for k in out[a]:
                out[a][k].append(r[a][k])
        resp.append(sc.responsive)
    for a in out:
        for k in out[a]:
            out[a][k] = np.concatenate(out[a][k])
    out["responsive"] = np.concatenate(resp)
    return out


def _delta_per100(scs, p, seed, m, arm="ideal"):
    rng = np.random.default_rng(seed)
    d = []
    for _ in range(m):
        e = evaluate(scs, p, rng)
        d.append(100 * (e[arm]["p_alive"].sum() - e["base"]["p_alive"].sum()) / len(e["responsive"]))
    return float(np.mean(d))


TORNADO_PARAMS = ["survival curve", "deep_detect_factor", "search_teams", "search_team_h_per_100m2_floor", "hasty_share",
                  "p_callout_responsive", "p_callout_unresponsive", "canine_start_h", "canine_teams", "p_canine",
                  "p_camera", "debris_removal_start_h", "pinpoint_h", "extrication_h", "extrication_teams",
                  "drone_deploy_h", "acoustic_lead_keep", "ping_keep", "search_start_h", "camera_start_h",
                  "hazard_multiplier_unconscious_or_severe", "canine_rate_m2_h"]


def tornado(scs, params, m=40, seed=7):
    c = central(params)
    base = _delta_per100(scs, c, seed, m)
    out = []
    for name in TORNADO_PARAMS:
        if name != "survival curve" and name not in params:
            continue
        vals = {}
        for end in ("low", "high"):
            p = dict(c)
            if name == "survival curve":
                for k in params:
                    if is_surv(k):
                        p[k] = float(params[k][end])
            else:
                p[name] = float(params[name][end])
            vals[end] = _delta_per100(scs, p, seed, m)
        out.append({"param": name, "low": vals["low"], "high": vals["high"], "swing": abs(vals["high"] - vals["low"]),
                    "range": ("set by the table" if name == "survival curve"
                              else [params[name]["low"], params[name]["high"]]),
                    "origin": "mixed" if name == "survival curve" else params[name]["origin"]})
    out.sort(key=lambda r: -r["swing"])
    return {"central_delta_per100": base, "rows": out, "m": m}


WEIGUAN_DAY1_BODIES = 13      # IMPACT_EVIDENCE.md section 4 (Tainan FB): 164 alive + 13 bodies brought out on day 1


def weiguan(e, p):
    """Weiguan-scale counterfactual for one draw, anchored on Weiguan's own late cohort.

    Late cohort = people extricated after day 1: extricated_total - alive_day1 - day-1 bodies (290 - 164 - 13 = 113);
    of them alive_day2 + alive_day3 = 11 came out alive (9.7%). From the simulation (same draw): q = share of the
    people today's search locates after 24 h whom the arm locates within 24 h, and s_new = their modelled chance of
    being alive at extrication in that arm. Extra survivors = late x q x max(0, s_new - observed late survival).
    Optimistic by construction: the late cohort is selected (deeper, more injured) beyond the hazard multiplier."""
    late_n = p["weiguan_extricated_total"] - p["weiguan_alive_day1"] - WEIGUAN_DAY1_BODIES
    s_obs = (p["weiguan_alive_day2"] + p["weiguan_alive_day3"]) / max(late_n, 1)
    late = e["base"]["t_loc"] > 24.0
    out = {"late_n": late_n, "s_obs_late": s_obs}
    for a in ARMS[1:]:
        moved = late & (e[a]["t_loc"] <= 24.0)
        q = float(moved.sum() / late.sum()) if late.any() else 0.0
        s_new = float(e[a]["p_alive"][moved].mean()) if moved.any() else 0.0
        out[a] = {"q": q, "s_new": s_new, "delta": late_n * q * max(0.0, s_new - s_obs)}
    return out


def what_if_pings(scs, params, qs=(0.0, 0.25, 0.5, 0.75, 1.0), m=40, seed=11):
    """What-if: the radar pod firmly pings a share q of the silent / unconscious, radar-reachable victims in sectors
    the drones had a lead on, by the end of the simulated sortie (instead of what the simulation pinged).
    Central parameters, common random numbers. Shows what the radar is worth if it performs at q."""
    c = central(params)
    c["ping_keep"] = 1.0
    out = []
    for q in qs:
        mod = []
        for sc in scs:
            s2 = copy.copy(sc)
            s2.t_ping = sc.t_ping.copy()
            u = np.random.default_rng([seed, sc.row["seed"]]).uniform(size=sc.n)
            reach = np.array([v["radar_reachable"] for v in sc.row["victims"]])
            led = np.isfinite(sc.sec_lead[sc.sec])
            pick = (~sc.responsive) & reach & led & (u < q) & ~np.isfinite(sc.t_ping)
            s2.t_ping[pick] = sc.mission_h
            mod.append(s2)
        out.append({"q": q, "delta_per100": _delta_per100(mod, c, seed, m, arm="ours"),
                    "eligible": int(sum(((~sc.responsive) & np.array([v["radar_reachable"] for v in sc.row["victims"]])
                                         & np.isfinite(sc.sec_lead[sc.sec])).sum() for sc in scs))})
    return out


def run_model(rows, n=1000, seed=2016, tornado_m=40, params_path=PARAMS_PATH, progress=True):
    params, meta = load_params(params_path)
    scs = [Scenario(r) for r in rows]
    rng = np.random.default_rng(seed)
    nv = sum(sc.n for sc in scs)
    keys = ["delta", "delta_per100", "delta_ideal", "delta_ideal_per100", "delta_rw_per100", "delta_ideal_rw_per100", "speedup_med", "drone_sortie", "drone_72h",
            "wg_delta", "wg_delta_ideal", "wg_q", "wg_q_ideal", "wg_snew", "wg_snew_ideal"]
    for a in ARMS:
        keys += [f"{x}_{a}" for x in ("surv", "within24", "within72", "unr24", "unr72", "rh", "cost")]
        keys += [f"loc_{a}_med", f"ext_{a}_med"]
    keep = {k: [] for k in keys}
    loc_hist = {a: [] for a in ARMS}
    meth = {a: {grp: np.zeros(len(METHODS) + 1) for grp in ("unresponsive", "responsive")} for a in ARMS}
    weig_n = None
    for d in range(n):
        p = draw(params, rng)
        e = evaluate(scs, p, rng)
        unr = ~e["responsive"]
        for a in ARMS:
            t = e[a]["t_loc"]
            keep[f"surv_{a}"].append(e[a]["p_alive"].sum())
            keep[f"loc_{a}_med"].append(np.median(t))
            keep[f"ext_{a}_med"].append(np.median(e[a]["t_ext"]))
            keep[f"within24_{a}"].append(float((t <= 24).mean()))
            keep[f"within72_{a}"].append(float((t <= 72).mean()))
            keep[f"unr24_{a}"].append(float((t[unr] <= 24).mean()) if unr.any() else np.nan)
            keep[f"unr72_{a}"].append(float((t[unr] <= 72).mean()) if unr.any() else np.nan)
            rh = rescuer_hours(p, t).mean()
            keep[f"rh_{a}"].append(rh)
            keep[f"cost_{a}"].append(rh * p["cost_per_rescuer_h_usd"])
            if d < 400:
                loc_hist[a].append(t)
            for grp, mask in (("unresponsive", unr), ("responsive", ~unr)):
                m = e[a]["method"][mask]
                within = t[mask] <= 72
                for k in range(len(METHODS)):
                    meth[a][grp][k] += np.sum((m == k) & within)
                meth[a][grp][len(METHODS)] += np.sum(~within)
        keep["speedup_med"].append(float(np.median(e["base"]["t_loc"] - e["ours"]["t_loc"])))
        w = p["frac_trapped_unconscious_at_rescue"]
        for a, key in (("ours", "delta"), ("ideal", "delta_ideal")):
            dv = e[a]["p_alive"] - e["base"]["p_alive"]
            keep[key].append(dv.sum())
            keep[key + "_per100"].append(100 * dv.sum() / nv)
            # reweighted to the sourced share of unconscious trapped people (the sim's state mix is ~50/50)
            du = dv[unr].mean() if unr.any() else 0.0
            dr = dv[~unr].mean() if (~unr).any() else 0.0
            keep[key + "_rw_per100" if key == "delta_ideal" else "delta_rw_per100"].append(100 * (w * du + (1 - w) * dr))
        dc = [drone_costs(p, sc.flight_h, sc.mission_h, sc.row["battery_swaps"]) for sc in scs]
        keep["drone_sortie"].append(np.mean([c["sortie_total"] for c in dc]))
        keep["drone_72h"].append(np.mean([c["deployment_72h_total"] for c in dc]))
        w = weiguan(e, p)
        for a, sfx in (("ours", ""), ("ideal", "_ideal")):
            keep["wg_delta" + sfx].append(w[a]["delta"])
            keep["wg_q" + sfx].append(w[a]["q"])
            keep["wg_snew" + sfx].append(w[a]["s_new"])
        if progress and (d + 1) % 200 == 0:
            print(f"  draw {d + 1}/{n}", flush=True)
    stats = {k: _q(v) for k, v in keep.items()}
    c = central(params)
    ssum = sim_summary(rows)
    t0 = tornado(scs, params, m=tornado_m)
    wi = what_if_pings(scs, params, m=tornado_m)
    hist = {a: np.concatenate(loc_hist[a]) for a in loc_hist}
    res = {
        "params_meta": meta,
        "params": params,
        "placeholders_used": sorted(k for k, v in params.items() if v["origin"] == "placeholder"),
        "sim": ssum,
        "n_victims_per_draw": nv,
        "n_draws": n,
        "stats": stats,
        "methods": METHODS + ["not located within 72 h"],
        "method_shares": {a: {g: (v / max(v.sum(), 1)).tolist() for g, v in meth[a].items()} for a in meth},
        "loc_ecdf": {a: {"q": list(range(0, 101)), "h": np.percentile(np.where(np.isfinite(hist[a]), hist[a], 1e4),
                                                                        list(range(0, 101))).tolist()}
                     for a in hist},
        "survival_points": {end: survival_points({k: float(v[end]) for k, v in params.items()
                                                  if is_surv(k)}) for end in ("low", "central", "high")},
        "drone_costs_central": drone_costs(c, ssum["flight_h_mean"], ssum["mission_h_mean"], ssum["swaps_mean"]),
        "logistics": logistics_bands(params, duty=ssum["rotors_on_duty"]),
        "tornado": t0,
        "what_if_pings": wi,
        "weiguan_late_cohort": c["weiguan_extricated_total"] - c["weiguan_alive_day1"] - WEIGUAN_DAY1_BODIES,
        "weiguan_late_survival_obs": (c["weiguan_alive_day2"] + c["weiguan_alive_day3"])
        / (c["weiguan_extricated_total"] - c["weiguan_alive_day1"] - WEIGUAN_DAY1_BODIES),
    }
    res["headline"] = headline(res)
    return res


def headline(res):
    s, sim = res["stats"], res["sim"]
    f = lambda q, k=1: None if q["median"] is None else [round(q["median"] * k, 2), round(q["p5"] * k, 2),
                                                        round(q["p95"] * k, 2)]
    out = {"note": "[median, p5, p95] over parameter draws; sim_* are direct simulation counts. Arms: base = today's "
                   "search; ours = with the drones as simulated; ideal = upper bound if the pod pings every silent, "
                   "radar-reachable victim in the sectors it is sent to",
           "sim_lead_within_30min_share": round(sim["lead_within_30min_share"], 3),
           "sim_lead_share_core": round(sim["lead_share_core"], 3), "sim_lead_min": sim["lead_min"],
           "sim_pinged": sim["pinged"], "sim_victims": sim["n_victims"],
           "sim_unresponsive": sim["n_unresponsive"], "sim_ideal_eligible": sim["ideal_eligible"],
           "extra_survivors_per100_ours": f(s["delta_rw_per100"]), "extra_survivors_per100_ideal": f(s["delta_ideal_rw_per100"]),
           "extra_survivors_per100_ours_simmix": f(s["delta_per100"]),
           "extra_survivors_per100_ideal_simmix": f(s["delta_ideal_per100"]),
           "weiguan_extra_survivors_ours": f(s["wg_delta"]), "weiguan_extra_survivors_ideal": f(s["wg_delta_ideal"]),
           "weiguan_q_ours_pct": f(s["wg_q"], 100), "weiguan_q_ideal_pct": f(s["wg_q_ideal"], 100),
           "weiguan_snew_ideal_pct": f(s["wg_snew_ideal"], 100),
           "drone_cost_sortie_usd": f(s["drone_sortie"]), "drone_cost_72h_usd": f(s["drone_72h"])}
    for a in ARMS:
        out[f"median_locate_h_{a}"] = f(s[f"loc_{a}_med"])
        out[f"within24_pct_{a}"] = f(s[f"within24_{a}"], 100)
        out[f"unresponsive_within24_pct_{a}"] = f(s[f"unr24_{a}"], 100)
        out[f"unresponsive_within72_pct_{a}"] = f(s[f"unr72_{a}"], 100)
        out[f"rescuer_h_per_person_{a}"] = f(s[f"rh_{a}"])
        out[f"rescuer_cost_per_person_usd_{a}"] = f(s[f"cost_{a}"])
    return out
