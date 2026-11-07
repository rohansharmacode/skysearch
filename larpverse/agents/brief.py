"""Rescuer handover brief (decision opportunity D) from the final state.json.

Facts come only from state fields (radar_candidates, piles, mission, heightmap, decisions, dispatch).
A template always builds the brief; if an LLM is configured it rewrites the same facts more readably,
and the rewrite is kept only if it passes checks (no banned claims, every pile id it names exists,
every ping id is mentioned). Otherwise the template is used (by="template").

    python -m agents.brief --state dashboard/data/state.json [--out brief.md] [--json] [--provider none]
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys

from . import llm

BANNED = re.compile(r"(?<!not )(?<!un)(?<!never )\bconfirmed\b|survivor(s)? found|person found|"
                    r"found (a |the )?(survivor|person)|\brescued\b", re.I)
OCTANTS = ["east", "north-east", "north", "north-west", "west", "south-west", "south", "south-east"]

SCHEMA = {"type": "object", "properties": {"brief": {"type": "string"}}, "required": ["brief"],
          "additionalProperties": False}
PROMPT = """Rewrite these facts as a short handover brief for a ground rescue team (plain text,
at most 180 words, one line per dig location, then unresolved leads, then piles not cleared).
Use only the facts given; keep every number, pile id and ping id exactly. Radar pings are candidate
search regions, not confirmed people: never write "confirmed", "found" or "rescued". Silent piles
are not cleared. Return JSON {"brief": <text>}.

Facts (JSON):
"""


def surface_z(state, x, y):
    hm = state.get("heightmap") or {}
    try:
        ox, oy = hm["origin"]
        res, w, h, z = hm["res"], hm["w"], hm["h"], hm["z"]
        i = min(w - 1, max(0, int((x - ox) / res)))
        j = min(h - 1, max(0, int((y - oy) / res)))
        return float(z[j * w + i])
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


def facts(state):
    piles = {p["id"]: p for p in state.get("piles") or []}
    m = state.get("mission") or {}
    visited = m.get("visited") or []
    digs, unresolved = [], []
    for c in state.get("radar_candidates") or []:
        pid = c.get("pile_id")
        p = piles.get(pid, {})
        xy = c.get("xy") or [0, 0]
        sz = surface_z(state, *xy)
        depth = round(sz - c["z_est"], 1) if sz is not None and c.get("z_est") is not None else None
        fl, off = flank(p.get("xy") or xy, xy)
        row = {"ping": c.get("id"), "pile": pid, "xy": [round(xy[0], 1), round(xy[1], 1)],
               "flank": fl, "offset_m": round(off, 1), "depth_m": depth,
               "radius95_m": None if r95(c.get("cov")) is None else round(r95(c.get("cov")), 1),
               "p_mass": c.get("p_mass"), "breathing_hz": c.get("breathing_hz"),
               "scans": len(c.get("evidence") or []), "status": c.get("status")}
        (digs if c.get("status") == "candidate" else unresolved).append(row)
    pinged = {d["pile"] for d in digs}
    # A lead stays open until a radar ping resolves it: visiting a pile without a ping does not close it.
    leads = [{"pile": p["id"], "priority": p.get("priority"),
              "why": p.get("why"), "visited": p["id"] in visited}
             for p in piles.values() if p.get("status") == "lead" and p["id"] not in pinged]
    leads.sort(key=lambda r: -(r["priority"] or 0))
    not_cleared = sorted(p["id"] for p in piles.values() if p["id"] not in visited)
    silent = sorted(p["id"] for p in piles.values() if p["id"] not in visited
                    and p.get("status") in ("no_signal_not_cleared", "dismissed_nonhuman", "unvisited"))
    silent_visited = sorted(p["id"] for p in piles.values() if p["id"] in visited
                            and not [d for d in digs if d["pile"] == p["id"]])
    return {"t": state.get("t"), "visited": visited, "digs": sorted(digs, key=lambda d: -(d["p_mass"] or 0)),
            "unresolved_radar": unresolved, "acoustic_leads": leads, "not_cleared": not_cleared, "silent_not_cleared": silent,
            "visited_without_firm_ping": silent_visited,
            "agent_decisions": sum(1 for d in state.get("decisions") or [] if d.get("by") not in ("argmax", None))}


def template(f):
    L = [f"RESCUER HANDOVER (mission t={f['t']:.0f} s; piles searched: {', '.join(f['visited']) or 'none'})"
         if isinstance(f.get("t"), (int, float)) else "RESCUER HANDOVER",
         "Radar pings are candidate search regions, not confirmed people.", "", "WHERE TO DIG:"]
    if not f["digs"]:
        L.append("- No firm radar ping.")
    for d in f["digs"]:
        depth = f"~{d['depth_m']} m below the surface" if d["depth_m"] is not None else "depth unknown"
        where = "pile centre" if d["flank"] == "centre" else f"{d['flank']} flank, {d['offset_m']} m from pile centre"
        L.append(f"- {d['ping']} on {d['pile']}: dig at ({d['xy'][0]}, {d['xy'][1]}), {where}, {depth}; "
                 f"95% within {d['radius95_m']} m; confidence p {d['p_mass']}, breathing-like "
                 f"{d['breathing_hz']} Hz, {d['scans']} supporting scans.")
    L += ["", "UNRESOLVED LEADS:"]
    for d in f["unresolved_radar"]:
        L.append(f"- {d['ping']} on {d['pile']} ({d['xy'][0]}, {d['xy'][1]}): insufficient radar evidence "
                 f"({d['scans']} scans, p {d['p_mass']}). Recheck.")
    for r in f["acoustic_leads"]:
        L.append(f"- {r['pile']} ({'searched by drone radar, no ping' if r.get('visited') else 'not searched'}; acoustic priority {r['priority']}): {r['why']}")
    if not f["unresolved_radar"] and not f["acoustic_leads"]:
        L.append("- None.")
    L += ["", "SILENT PILES NOT CLEARED: " + (", ".join(f["silent_not_cleared"]) or "none")
          + ". Silence is not absence; search these by hand or with dogs.",
          "Not searched by drone radar (all not cleared): " + (", ".join(f["not_cleared"]) or "none") + "."]
    return "\n".join(L)


def check(text, f):
    if not text or BANNED.search(text):
        return False
    known = set(f["visited"]) | set(f["not_cleared"]) | {d["pile"] for d in f["digs"] + f["unresolved_radar"]} \
        | {r["pile"] for r in f["acoustic_leads"]}
    named = set(re.findall(r"\b([A-C][1-3])\b", text))
    if not named <= known:
        return False
    return all(d["ping"] in text for d in f["digs"])


def brief(state, provider=None):
    """Return {"brief", "by", "facts"}. Never raises on LLM failure."""
    f = facts(state)
    text = template(f)
    by = "template"
    if llm.available(provider):
        meta = {}
        out = llm.call_json(PROMPT + json.dumps(f, sort_keys=True), SCHEMA, provider=provider,
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
    print(json.dumps(r, indent=2) if a.json else r["brief"])
    print(f"[brief] by {r['by']}", file=sys.stderr)


if __name__ == "__main__":
    main()
