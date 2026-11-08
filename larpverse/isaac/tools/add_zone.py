"""Rehearsal helper: add an operator search zone to a running mission, like the dashboard's "Add as zone".

In the real take a person adds the zone live on the dashboard (scripts/serve.py POST /api/tasking). For
rehearsals and GPU tests this does the same file write: it waits until <out>/state.json reaches sim time --at,
then appends one zone to <out>/tasking.json (a new revision): the drones' first zone suggestion (a human-like
sound heard outside every zone), else the sector --sector. GridMission (run_mission --tasking auto) re-reads
the file on its next step.

    python isaac/tools/add_zone.py --out dashboard/data --init T1a --site collapse --seed 7   # before the run
    python isaac/tools/add_zone.py --out dashboard/data --at 116 [--sector T1b] [--radius 9]  # during it
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time


def _load(path):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def zone_from_state(state, sector=None, radius=9.0):
    sug = state.get("zone_suggestions") or []
    if sug and not sector:
        s = sug[0]
        c = s.get("centre") or s.get("center") or s.get("xy")
        r = max(float(s.get("radius_m") or 6.0), radius)
        poly = s.get("polygon") or [[c[0] + r * math.cos(a), c[1] + r * math.sin(a)]
                                    for a in (2 * math.pi * k / 12 for k in range(12))]
        why = s.get("why") or "sound heard by the drones"
        return {"id": f"OP-{s.get('id', 'S1')}", "name": "Operator zone: " + str(why)[:60], "priority": 1.0,
                "polygon": [[round(float(p[0]), 2), round(float(p[1]), 2)] for p in poly], "note": "added live",
                "suggestion_id": s.get("id")}
    for p in state.get("piles") or []:
        if p.get("id") == sector:
            x, y = p["xy"][:2]
            return {"id": f"OP-{sector}", "name": f"Operator zone {sector}", "priority": 1.0, "note": "added live",
                    "polygon": [[round(x + radius * math.cos(a), 2), round(y + radius * math.sin(a), 2)]
                                for a in (2 * math.pi * k / 12 for k in range(12))]}
    raise SystemExit(f"no zone suggestion and no sector {sector!r} in the state")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="dashboard/data")
    ap.add_argument("--at", type=float, default=0.0, help="sim time (s) to add the zone at")
    ap.add_argument("--sector", default=None, help="fallback (or forced) sector id, e.g. T1b")
    ap.add_argument("--radius", type=float, default=9.0)
    ap.add_argument("--timeout", type=float, default=3600.0)
    ap.add_argument("--init", default="", help="write revision 1 with these sectors' outlines (comma list) and exit")
    ap.add_argument("--site", default="collapse")
    ap.add_argument("--seed", type=int, default=7)
    a = ap.parse_args(argv)
    sp, tp = os.path.join(a.out, "state.json"), os.path.join(a.out, "tasking.json")
    if a.init:
        import sys
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        from sim.scenario import make_scenario
        from sim.tasking import sector_outline
        sc = make_scenario(a.seed, None, site=a.site) if a.site != "grid" else make_scenario(a.seed)
        piles = {p.name: p for p in sc.piles}
        zones = [{"id": n, "name": n, "priority": None, "note": "operator tasking before launch",
                  "polygon": sector_outline(sc, piles[n], 2.0).round(2).tolist()} for n in a.init.split(",") if n]
        os.makedirs(a.out, exist_ok=True)
        for f in ("state.json",):                     # a stale state from an earlier run would fire --at at once
            if os.path.exists(os.path.join(a.out, f)):
                os.remove(os.path.join(a.out, f))
        with open(tp, "w", encoding="utf-8") as f:
            json.dump({"schema": "larpverse.tasking/v1", "revision": 1, "source": "operator", "zones": zones}, f)
        print(f"[add_zone] {tp}: revision 1 with {[z['id'] for z in zones]}", flush=True)
        return
    t0 = time.time()
    while True:
        st = _load(sp) or {}
        if float(st.get("t") or 0.0) >= a.at and (st.get("zone_suggestions") or a.sector):
            break
        if time.time() - t0 > a.timeout:
            raise SystemExit("timed out waiting for the mission")
        time.sleep(0.2)
    tk = _load(tp) or {"schema": "larpverse.tasking/v1", "revision": 0, "source": "operator", "zones": []}
    z = zone_from_state(st, a.sector, a.radius)
    tk["zones"] = [q for q in tk.get("zones", []) if q.get("id") != z["id"]] + [z]
    tk["revision"] = int(tk.get("revision") or 0) + 1
    tk["source"], tk["updated_at"] = "operator", time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    tmp = tp + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(tk, f)
    os.replace(tmp, tp)
    print(f"[add_zone] t={st.get('t')}: added {z['id']} ({z['name']}) as revision {tk['revision']} -> {tp}", flush=True)


if __name__ == "__main__":
    main()
