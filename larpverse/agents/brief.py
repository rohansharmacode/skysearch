"""Rescuer handover brief (decision opportunity D) from the final state.json.

Facts come only from state fields (radar_candidates + vitals/dig_score, piles, mission incl. delivery and
battery swaps, scans/pods, zones, zone_proposals, heightmap, decisions). A template always builds the
brief; if an LLM is configured it rewrites the same facts more readably, and the rewrite is kept only if
it passes checks (wording rules of agents.gemini_decider, every pile id it names exists, every ping id is
mentioned in dig order, "not cleared" kept). Otherwise the template is used (by="template").

Wording: pings are candidate search regions; breathing notes are a triage aid that says "check" (no
diagnosis, no age); searched zones are "searched — not cleared"; pod delivery is "pod lowered onto rubble".

    python -m agents.brief --state dashboard/data/state.json [--out brief.md] [--json] [--provider none]
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys

from . import llm
from .gemini_decider import _zone_fallback, dig_order, wording_problem, zone_options

BANNED = re.compile(r"(?<!not )(?<!un)(?<!never )\bconfirmed\b|survivor(s)? found|person found|"
                    r"found (a |the )?(survivor|person)|\brescued\b|diagnos", re.I)
OCTANTS = ["east", "north-east", "north", "north-west", "west", "south-west", "south", "south-east"]
NOT_CLEARED = "searched — not cleared"
ZONE_WORDS = {"unsearched": "not searched yet", "surveying": "being searched", "scanning": "being searched",
              "searched_no_ping": f"{NOT_CLEARED} (no ping; no detection is not proof of absence)",
              "ping": "ping (candidate search region)", "lead": "open lead, no firm ping",
              "beyond_radar_reach": "radar could not reach it — not cleared", "cancelled": "cancelled by the operator"}

SCHEMA = {"type": "object", "properties": {"brief": {"type": "string"}}, "required": ["brief"],
          "additionalProperties": False}
PROMPT = """Rewrite these facts as a short handover brief for a ground rescue team (plain text,
at most 220 words): dig locations one line each in the given dig order (keep each breathing note and
its "check" wording), then open leads, then zones and piles not cleared, then battery swaps and pods.
Use only the facts given; keep every number, pile id and ping id exactly. Radar pings are candidate
search regions, not confirmed people: never write "confirmed", "found" or "rescued". Breathing notes
are a triage aid for rescuers to check, not a diagnosis: describe only the breathing, never age
(no child, infant, elderly) and never injury or consciousness as fact. Searched zones and piles
without a ping are "searched — not cleared". If delivery is "pod", say "pod lowered onto rubble",
never "landed". Return JSON {"brief": <text>}.

Facts (JSON):
"""


def surface_z(state, x, y):
    hm = state.get("heightmap") or {}
    try:
        ox, oy = hm["origin"]
        res, w, h, z = hm["res"], hm["w"], hm["h"], hm["z"]
        i = min(w - 1, max(0, int((x - ox) / res)))
        j = min(h - 1, max(0, int((y - oy) / res)))
        v = z[j * w + i]
        return None if v is None else float(v)
    except (KeyError, TypeError, ValueError, IndexError):
        return None


def flank(pile_xy, xy):
    dx, dy = xy[0] - pile_xy[0], xy[1] - pile_xy[1]
    d = math.hypot(dx, dy)
    if d < 1.0:
        return "centre", d
    return OCTANTS[int(((math.degrees(math.atan2(dy, dx)) % 360) + 22.5) // 45) % 8], d


def r95(cov):
    try:
        a, b, c = cov[0][0], cov[0][1], cov[1][1]
        lam = (a + c) / 2 + math.sqrt(((a - c) / 2) ** 2 + b * b)
        return 2.45 * math.sqrt(max(lam, 0.0))
    except (TypeError, IndexError, ValueError):
        return None


def _depth(state, c):
    if isinstance(c.get("depth_m"), (int, float)):
        return round(float(c["depth_m"]), 1)
    xy = c.get("xy") or [0, 0]
    sz = surface_z(state, *xy)
    return round(sz - c["z_est"], 1) if sz is not None and c.get("z_est") is not None else None


def _delivery(state):
    m = state.get("mission") or {}
    if m.get("delivery") in ("pod", "land"):
        return m["delivery"]
    if any(s.get("delivery") == "pod" for s in state.get("scans") or []) or state.get("pods"):
        return "pod"
    return "land"


def _pod_placements(state):
    """{pile: n distinct pod placements} from pod scans (origin = pod surface position)."""
    out = {}
    for s in state.get("scans") or []:
        if s.get("delivery") != "pod":
            continue
        pose = s.get("sensor_pose") or s.get("origin") or []
        if len(pose) >= 2:
            out.setdefault(s.get("pile_id") or "?", set()).add((round(pose[0], 1), round(pose[1], 1)))
    return {k: len(v) for k, v in sorted(out.items())}


def facts(state):
    piles = {p["id"]: p for p in state.get("piles") or []}
    m = state.get("mission") or {}
    visits = m.get("visited") or []
    visited = list(dict.fromkeys(visits))
    revisited = sorted({p for p in visits if visits.count(p) > 1})
    pod = _delivery(state) == "pod"
    scans = {s.get("scan_id"): s for s in state.get("scans") or []}
    digs, unresolved = [], []
    cands = state.get("radar_candidates") or []
    order = {r["ping"]: r for r in dig_order(cands, depth_of=lambda c: _depth(state, c))}
    for c in cands:
        pid = c.get("pile_id")
        p = piles.get(pid, {})
        xy = c.get("xy") or [0, 0]
        fl, off = flank(p.get("xy") or xy, xy)
        ev = c.get("evidence") or []
        row = {"ping": c.get("id"), "pile": pid, "xy": [round(xy[0], 1), round(xy[1], 1)],
               "flank": fl, "offset_m": round(off, 1), "depth_m": _depth(state, c),
               "radius95_m": None if r95(c.get("cov")) is None else round(r95(c.get("cov")), 1),
               "p_mass": c.get("p_mass"), "breathing_hz": c.get("breathing_hz"),
               "scans": len(ev), "pod_scans": sum(1 for s in ev if (scans.get(s) or {}).get("delivery") == "pod"),
               "status": c.get("status")}
        if c.get("id") in order:
            o = order[c["id"]]
            row.update(dig_rank=o["rank"], dig_score=o["dig_score"], condition=o["condition"], pattern=o["pattern"])
        (digs if c.get("status") == "candidate" else unresolved).append(row)
    digs.sort(key=lambda d: d.get("dig_rank", 99))
    pinged = {d["pile"] for d in digs}
    # A lead stays open until a radar ping resolves it: visiting a pile without a ping does not close it.
    leads = [{"pile": p["id"], "priority": p.get("priority"), "why": p.get("why"),
              "visited": p["id"] in visited, "revisited": p["id"] in revisited}
             for p in piles.values() if p.get("status") == "lead" and p["id"] not in pinged]
    leads.sort(key=lambda r: -(r["priority"] or 0))
    not_cleared = sorted(p["id"] for p in piles.values() if p["id"] not in visited)
    silent = sorted(p["id"] for p in piles.values() if p["id"] not in visited
                    and p.get("status") in ("no_signal_not_cleared", "dismissed_nonhuman", "unvisited"))
    searched_no_ping = sorted(p for p in visited if p in piles and p not in pinged)
    zones = [{"id": z.get("id"), "name": z.get("name"), "status": z.get("status"), "pings": z.get("pings") or []}
             for z in state.get("zones") or [] if z.get("id")]
    zopts = zone_options(state)
    sw = m.get("battery_swap")
    return {"t": state.get("t"), "visited": visited, "revisited": revisited,
            "delivery": "pod" if pod else "land", "pod_placements": _pod_placements(state) if pod else {},
            "digs": digs, "unresolved_radar": unresolved, "acoustic_leads": leads,
            "not_cleared": not_cleared, "silent_not_cleared": silent,
            "visited_without_firm_ping": searched_no_ping,
            "zones": zones, "zone_suggestion": _zone_fallback(zopts)[1] if zopts else None,
            "battery_swaps": m.get("battery_swaps") or 0,
            "battery_swap_now": sw.get("to_pile") if isinstance(sw, dict) else None,
            "battery_reserve": m.get("battery_reserve"),
            "scans_used": m.get("scans_used"), "scan_budget": m.get("scan_budget"),
            "agent_decisions": sum(1 for d in state.get("decisions") or [] if d.get("by") not in ("argmax", None))}


def template(f):
    pod = f.get("delivery") == "pod"
    searched = ", ".join(p + (" (revisited)" if p in f.get("revisited", []) else "") for p in f["visited"]) or "none"
    L = [f"RESCUER HANDOVER (mission t={f['t']:.0f} s; piles searched: {searched})"
         if isinstance(f.get("t"), (int, float)) else "RESCUER HANDOVER",
         "Radar pings are candidate search regions, not confirmed people. Breathing notes are a triage aid "
         "for rescuers to check, not medical findings.", "",
         "WHERE TO DIG (in dig order: breathing triage aid x location certainty x depth):"]
    if not f["digs"]:
        L.append("- No firm radar ping.")
    for i, d in enumerate(f["digs"], 1):
        depth = f"~{d['depth_m']} m below the surface" if d["depth_m"] is not None else "depth unknown"
        where = "pile centre" if d["flank"] == "centre" else f"{d['flank']} flank, {d['offset_m']} m from pile centre"
        src = f"{d['scans']} supporting scan{'s' if d['scans'] != 1 else ''}" + (f" from the pod lowered onto rubble" if pod and d.get("pod_scans") else "")
        L.append(f"{d.get('dig_rank', i)}. {d['ping']} on {d['pile']}: dig at ({d['xy'][0]}, {d['xy'][1]}), {where}, "
                 f"{depth}; 95% within {d['radius95_m']} m; confidence p {d['p_mass']}, {src}."
                 + (f" Breathing: {d['condition']} (dig score {d['dig_score']})." if d.get("condition") else ""))
    L += ["", "OPEN LEADS:"]
    for d in f["unresolved_radar"]:
        L.append(f"- {d['ping']} on {d['pile']} ({d['xy'][0]}, {d['xy'][1]}): insufficient radar evidence "
                 f"({d['scans']} scan{'s' if d['scans'] != 1 else ''}, p {d['p_mass']}). Recheck.")
    for r in f["acoustic_leads"]:
        how = ("searched twice by drone radar (revisit), no ping" if r.get("revisited") else
               "searched by drone radar, no ping" if r.get("visited") else "not searched")
        L.append(f"- {r['pile']} ({how}; acoustic priority {r['priority']}): {r['why']}")
    if not f["unresolved_radar"] and not f["acoustic_leads"]:
        L.append("- None.")
    if f.get("zones"):
        L += ["", "ZONES:"]
        for z in f["zones"]:
            st = ZONE_WORDS.get(z["status"], z["status"] or "unknown")
            L.append(f"- {z['id']}{' ' + z['name'] if z.get('name') else ''}: {st}"
                     + (f" ({', '.join(z['pings'])})" if z["pings"] else "") + ".")
    if f.get("zone_suggestion"):
        L.append(f"Operator's call: {f['zone_suggestion']}")
    L += ["", "NOT CLEARED:",
          "- Searched without a ping (" + NOT_CLEARED + "): " + (", ".join(f["visited_without_firm_ping"]) or "none") + ".",
          "- Silent piles: " + (", ".join(f["silent_not_cleared"]) or "none")
          + ". Silence is not absence; search these by hand or with dogs.",
          "- Not searched by drone radar: " + (", ".join(f["not_cleared"]) or "none") + "."]
    ops = [f"battery swaps: {f.get('battery_swaps', 0)}"
           + (f" (fleet returned to the pads to keep a {100 * f['battery_reserve']:.0f}% reserve)"
              if f.get("battery_swaps") and isinstance(f.get("battery_reserve"), (int, float)) else "")]
    if f.get("battery_swap_now"):
        ops.append(f"battery swap in progress before {f['battery_swap_now']}")
    if pod:
        pp = f.get("pod_placements") or {}
        ops.append("radar + mic pod lowered onto rubble by winch" +
                   (": " + ", ".join(f"{k} {v} placements" for k, v in pp.items()) if pp else ""))
    if f.get("scans_used") is not None:
        ops.append(f"radar scans used: {f['scans_used']}" + (f" of {f['scan_budget']}" if f.get("scan_budget") else ""))
    L += ["", "FLEET: " + "; ".join(ops) + "."]
    return "\n".join(L)


def check(text, f):
    if not text or BANNED.search(text) or wording_problem(text, pod=f.get("delivery") == "pod"):
        return False
    known = set(f["visited"]) | set(f["not_cleared"]) | {d["pile"] for d in f["digs"] + f["unresolved_radar"]} \
        | {r["pile"] for r in f["acoustic_leads"]}
    named = set(re.findall(r"\b([A-C][1-3])\b", text))
    if not named <= known:
        return False
    pos = [text.find(d["ping"]) for d in f["digs"]]
    if any(p < 0 for p in pos) or pos != sorted(pos):                 # every ping, in dig order
        return False
    if (f["not_cleared"] or f["visited_without_firm_ping"]) and "not cleared" not in text.lower():
        return False
    if any(d.get("pattern") for d in f["digs"]) and "check" not in text.lower():
        return False
    return True


def brief(state, provider=None):
    """Return {"brief", "by", "facts"}. Never raises on LLM failure."""
    f = facts(state)
    text = template(f)
    by = "template"
    if llm.available(provider):
        meta = {}
        out = llm.call_json(PROMPT + json.dumps(f, sort_keys=True, ensure_ascii=False), SCHEMA, provider=provider,
                            max_tokens=1500, meta=meta,
                            validate=lambda o: isinstance(o.get("brief"), str) and check(o["brief"].strip(), f))
        cand = (out or {}).get("brief") if isinstance(out, dict) else None
        if isinstance(cand, str) and check(cand.strip(), f):
            text = cand.strip()
            if "candidate search region" not in text.lower():
                text = "Radar pings are candidate search regions, not confirmed people.\n" + text
            by = meta.get("provider") or "llm"
    return {"brief": text, "by": by, "facts": f}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--state", default="dashboard/data/state.json")
    ap.add_argument("--out", default=None, help="also write the brief text here")
    ap.add_argument("--json", action="store_true", help="print {brief, by, facts} as JSON")
    ap.add_argument("--provider", default=None, choices=list(llm.PROVIDERS))
    a = ap.parse_args(argv)
    with open(a.state) as fh:
        state = json.load(fh)
    r = brief(state, provider=a.provider)
    if a.out:
        with open(a.out, "w") as fh:
            fh.write(r["brief"] + "\n")
    print(json.dumps(r, indent=2, ensure_ascii=False) if a.json else r["brief"])
    print(f"[brief] by {r['by']}", file=sys.stderr)


if __name__ == "__main__":
    main()
