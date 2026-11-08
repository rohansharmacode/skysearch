#!/usr/bin/env python3
"""End-to-end check of the dashboard <-> Isaac I/O loop (stdlib only; runs anywhere the ops server is reachable).

INPUT  (operator -> Isaac):  POST /api/tasking  -> dashboard/data/tasking.json  -> polled by run_mission --live every 1 s
OUTPUT (Isaac -> operator):  run_mission --live writes dashboard/data/{state,live,hud,motion}.json -> served by scripts/serve.py

  python3 scripts/isaac_io_check.py                       # status only (what Isaac is publishing right now)
  python3 scripts/isaac_io_check.py --send-pile A1        # task the zone around pile A1, then watch the mission react
  python3 scripts/isaac_io_check.py --send-zone "-28,15;-17,15;-17,27;-28,27" --watch 90
  python3 scripts/isaac_io_check.py --base http://localhost:8765 --data dashboard/data

PASS = the mission picked up the new tasking revision and moved the fleet toward / started work in that zone.
"""
import argparse
import json
import sys
import time
import urllib.request


def get(base, path, timeout=5):
    with urllib.request.urlopen(f"{base}/{path.lstrip('/')}", timeout=timeout) as r:
        return json.loads(r.read().decode())


def post(base, path, body, timeout=5):
    req = urllib.request.Request(f"{base}/{path.lstrip('/')}", data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def snapshot(base, data):
    out = {}
    for name in ("state", "live", "hud"):
        try:
            out[name] = get(base, f"{data}/{name}.json?_={time.time():.0f}")
        except Exception as e:                       # noqa: BLE001 - report, keep going
            out[name] = {"_error": f"{type(e).__name__}: {e}"}
    return out


def summary(s):
    st, lv = s["state"], s["live"]
    if "_error" in st:
        return f"state.json unavailable ({st['_error']})"
    m = st.get("mission", {})
    zones = {z.get("id"): z.get("status") for z in st.get("zones", [])}
    drones = ", ".join(f"{d.get('id')}:{d.get('status') or d.get('phase') or ''}" for d in st.get("drones", []))
    causal = (lv.get("causal") or [{}])[-1] if isinstance(lv.get("causal"), list) else lv.get("causal")
    pings = [p.get("id") for p in st.get("pings", [])]
    return (f"t={st.get('t')} phase={st.get('phase')} visited={m.get('visited')} drones=[{drones}]\n"
            f"   zones={zones}\n   pings={pings} banner={lv.get('banner')} badge={lv.get('badge')}\n"
            f"   causal={causal}")


def pile_polygon(state, pile, pad=1.0):
    for p in state.get("piles", []):
        if p.get("name") == pile or p.get("id") == pile:
            poly = p.get("polygon") or p.get("outline")
            if poly:
                return [[float(x), float(y)] for x, y in poly]
            cx, cy = p.get("xy") or p.get("center") or (None, None)
            r = float(p.get("radius_m", 6.0)) + pad
            if cx is not None:
                return [[cx - r, cy - r], [cx + r, cy - r], [cx + r, cy + r], [cx - r, cy + r]]
    raise SystemExit(f"pile {pile!r} not found in state.json piles[]")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", default="http://localhost:8765", help="ops server (scripts/serve.py), port-forwarded")
    ap.add_argument("--data", default="dashboard/data", help="URL path of the live run's output folder")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--send-pile", help="task the zone around this pile/sector id (polygon from state.json)")
    g.add_argument("--send-zone", help='polygon "x,y;x,y;x,y[;...]" in site metres')
    ap.add_argument("--watch", type=float, default=60.0, help="seconds to watch the mission react (wall clock)")
    a = ap.parse_args()

    s = snapshot(a.base, a.data)
    print("NOW:", summary(s))
    if not (a.send_pile or a.send_zone):
        return 0

    poly = (pile_polygon(s["state"], a.send_pile) if a.send_pile
            else [[float(v) for v in pt.split(",")] for pt in a.send_zone.split(";")])
    cur = get(a.base, "api/tasking")
    zid = f"OP{int(time.time()) % 100000}"
    zones = [z for z in cur.get("zones", []) if z.get("status") not in ("cancelled",)] + [
        {"id": zid, "name": f"Operator area {a.send_pile or ''}".strip(), "polygon": poly, "priority": 100,
         "status": "tasked", "note": "operator: scan this area now"}]
    res = post(a.base, "api/tasking", {"zones": zones, "source": "operator", "revision": cur.get("revision", 0)})
    print(f"SENT zone {zid} (revision {res.get('revision')}): {poly}")

    t0, seen = time.time(), None
    while time.time() - t0 < a.watch:
        time.sleep(2)
        s = snapshot(a.base, a.data)
        st = s["state"]
        if "_error" in st:
            continue
        line = summary(s)
        if line != seen:
            print(f"[+{time.time() - t0:4.0f}s]", line)
            seen = line
        z = {z.get("id"): z for z in st.get("zones", [])}.get(zid)
        rev_seen = (st.get("tasking") or {}).get("revision")
        moved = z and z.get("status") not in (None, "tasked", "unsearched")
        if moved or (rev_seen and rev_seen >= res.get("revision", 0) and z):
            print(f"PASS: Isaac/mission picked up {zid} (status {z.get('status') if z else '?'}, tasking rev {rev_seen})")
            return 0
    print("FAIL: no reaction within the watch window. Check that run_mission --live is running, that it polls "
          "the same dashboard/data/tasking.json this server writes, and the mission log.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
