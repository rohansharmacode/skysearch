"""Compare dispatch policies for the autonomous grid mission over N seeds (BL-7).

Policies (same 3 drones, same radar, same scan budget, same battery model and swaps, same start pile):
  * acoustic  - ours: listen after each pile, dispatch to the highest acoustic priority, revisit open leads
  * nearest   - baseline: nearest unvisited pile next (radar only, no listening)
  * raster    - baseline: fixed lawnmower order A1 B1 C1 C2 B2 A2 A3 B3 C3 (radar only, no listening)
All three start at the scenario's start pile (the operator-reported collapse; the scenario always places a
victim there, so the first ping is by construction - metrics are also reported without it).

This module is an EVALUATOR: it reads hidden truth (``scenario.truth``) after each run to score pings. The
mission itself never sees it. Acoustic input is ``--acoustic stub`` by default: the synthetic stub that
fabricates clips FROM TRUTH (fast, but cleaner than the real pipeline) - the report says so.

Runs are cached per (seed, policy) in ``eval/grid_rows/`` so the work can be split into short calls:

    scripts/heavy.sh .venv/bin/python -m sim.evaluate_grid --seeds 1-5
    scripts/heavy.sh .venv/bin/python -m sim.evaluate_grid --seeds 6-10
    ...
    .venv/bin/python -m sim.evaluate_grid --report --seeds 1-20      # table -> docs/EVAL_GRID.md + eval/grid_results.json
"""
from __future__ import annotations

import argparse
import json
import os
import time

import numpy as np

from .grid_mission import MISSION_SCANS, GridMission

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
POLICIES = ["acoustic", "nearest", "raster"]
PILES = [f"{c}{r}" for r in (1, 2, 3) for c in "ABC"]
HIT_M = 3.0                 # a firm ping within this horizontal distance of a victim counts as pinging them
EMPTY_KINDS = {"silent", "dog", "generator", "settling"}


def parse_seeds(spec: str):
    out = []
    for part in spec.split(","):
        if "-" in part:
            a, b = part.split("-")
            out += list(range(int(a), int(b) + 1))
        elif part:
            out.append(int(part))
    return out


def score_run(m: GridMission) -> dict:
    """Truth-based scoring of a finished mission (EVALUATOR ONLY)."""
    truth = m.scenario.truth                                  # hidden truth: evaluator only
    victims = truth["victims"]
    roles = dict(truth["roles"])
    start = m.start_pile
    pings = [c for c in m.done_cands if c["status"] == "candidate"]
    hits, false, errs = {}, [], []
    for c in pings:
        d = [float(np.hypot(c["xy"][0] - v["xy"][0], c["xy"][1] - v["xy"][1])) for v in victims]
        k = int(np.argmin(d)) if d else -1
        if k >= 0 and d[k] <= HIT_M:
            v = victims[k]
            errs.append(d[k])
            if v["id"] not in hits or c["t_found"] < hits[v["id"]]["t"]:
                hits[v["id"]] = {"t": c["t_found"], "err_m": round(d[k], 2), "pile": v["pile"], "ping": c["id"],
                                 "depth_err_m": None if c.get("depth_source") != "radar_estimate"
                                 else round(abs(c["depth_m"] - v["depth"]), 2)}
        else:
            false.append({"ping": c["id"], "nearest_victim_m": round(min(d), 2) if d else None})
    non_start = [v for v in victims if v["pile"] != start]
    t_non_start = [hits[v["id"]]["t"] for v in non_start if v["id"] in hits]
    wasted = [p for p in m.visited if p != start and roles.get(p, "silent") in EMPTY_KINDS]
    return {
        "seed": m.seed, "policy": m.dispatch_policy, "start_pile": start,
        "victims": len(victims), "victims_non_start": len(non_start),
        "pinged": len(hits), "pinged_non_start": sum(1 for v in non_start if v["id"] in hits),
        "victim_detail": [{"id": v["id"], "pile": v["pile"], "sound": v.get("sound"),
                           "pinged": v["id"] in hits, **hits.get(v["id"], {})} for v in victims],
        "t_first_ping": min((h["t"] for h in hits.values()), default=None),
        "t_first_non_start": min(t_non_start, default=None),
        "t_all_pinged": max((h["t"] for h in hits.values()), default=None) if len(hits) == len(victims) else None,
        "loc_err_m": [round(e, 2) for e in errs],
        "false_pings": len(false), "false_detail": false,
        "visits": list(m.visited), "wasted_visits": len(wasted),
        "insufficient": sum(1 for c in m.done_cands if c["status"] != "candidate"),
        "scans": m.scans_used, "battery_swaps": m.n_swaps, "flight_s": round(m._fly_s, 1),
        "mission_s": round(m.t, 1), "acoustic_source": m.acoustic_source,
    }


def independent_start(seed):
    """Start pile drawn from the seed alone, independently of where the scenario placed anyone (R1-007)."""
    return PILES[int(np.random.default_rng([int(seed), 4242]).integers(0, 9))]


def run_one(seed, policy, radar, acoustic, scans, penetration="legacy", start="reported"):
    t0 = time.time()
    m = GridMission(seed=seed, radar=radar, acoustic=acoustic, verbose=False, dispatch_policy=policy,
                    mission_scans=scans, penetration=penetration,
                    launch_pile=independent_start(seed) if start == "independent" else None).run()
    r = score_run(m)
    r["wall_s"] = round(time.time() - t0, 2)
    return r


def _row_path(rows_dir, seed, policy, radar, acoustic, scans, penetration, start):
    sub = f"{radar}-{penetration}-{acoustic}-s{scans}" + ("-indep" if start == "independent" else "")
    return os.path.join(rows_dir, sub, f"seed{seed:03d}-{policy}.json")


# ------------------------------------------------------------------------------------------- stats
def _boot_ci(x, stat=np.mean, n=2000, seed=0):
    x = np.asarray(x, float)
    if len(x) == 0:
        return None, None, None
    rng = np.random.default_rng(seed)
    bs = [stat(x[rng.integers(0, len(x), len(x))]) for _ in range(n)]
    return float(stat(x)), float(np.percentile(bs, 2.5)), float(np.percentile(bs, 97.5))


def summarise(rows):
    out = {"n_seeds": len(rows)}
    vic = sum(r["victims"] for r in rows)
    vns = sum(r["victims_non_start"] for r in rows)
    out["victims_total"], out["victims_non_start_total"] = vic, vns
    out["pinged_total"] = sum(r["pinged"] for r in rows)
    out["pinged_non_start_total"] = sum(r["pinged_non_start"] for r in rows)
    out["frac_pinged"] = _boot_ci([r["pinged"] / max(r["victims"], 1) for r in rows])
    out["frac_pinged_non_start"] = _boot_ci([r["pinged_non_start"] / max(r["victims_non_start"], 1) for r in rows])
    fp = [r["t_first_non_start"] for r in rows if r["t_first_non_start"] is not None]
    out["t_first_non_start_median"] = _boot_ci(fp, np.median)
    out["t_first_non_start_none"] = sum(1 for r in rows if r["t_first_non_start"] is None)
    ta = [r["t_all_pinged"] for r in rows if r["t_all_pinged"] is not None]
    out["all_pinged_seeds"] = len(ta)
    out["t_all_pinged_median"] = float(np.median(ta)) if ta else None
    errs = [e for r in rows for e in r["loc_err_m"]]
    out["loc_err_median_m"] = float(np.median(errs)) if errs else None
    out["loc_err_p90_m"] = float(np.percentile(errs, 90)) if errs else None
    out["false_pings"] = sum(r["false_pings"] for r in rows)
    out["wasted_visits_mean"] = _boot_ci([r["wasted_visits"] for r in rows])
    out["insufficient_mean"] = float(np.mean([r["insufficient"] for r in rows]))
    out["flight_s_mean"] = _boot_ci([r["flight_s"] for r in rows])
    out["mission_s_mean"] = _boot_ci([r["mission_s"] for r in rows])
    out["scans_mean"] = float(np.mean([r["scans"] for r in rows]))
    out["swaps_mean"] = float(np.mean([r["battery_swaps"] for r in rows]))
    de = [d["depth_err_m"] for r in rows for d in r["victim_detail"] if d.get("depth_err_m") is not None]
    out["depth_estimates"] = len(de)
    out["depth_err_median_m"] = float(np.median(de)) if de else None
    return out


def paired(rows_a, rows_b, key):
    a = {r["seed"]: r[key] for r in rows_a}
    b = {r["seed"]: r[key] for r in rows_b}
    s = [x for x in a if x in b]
    return {"wins": sum(a[x] > b[x] for x in s), "ties": sum(a[x] == b[x] for x in s),
            "losses": sum(a[x] < b[x] for x in s)}


def _fmt_ci(t, fmt="{:.2f}", scale=1.0):
    v, lo, hi = t if isinstance(t, (list, tuple)) else (t, None, None)
    if v is None:
        return "n/a"
    s = fmt.format(v * scale)
    return s if lo is None else f"{s} [{fmt.format(lo * scale)}, {fmt.format(hi * scale)}]"


def section(cfg, summary, pair, rows, per_seed=False):
    """Markdown for one (penetration, start) configuration."""
    start_txt = ("the scenario's reported start pile (it always holds a victim, so that ping is by construction)"
                 if cfg["start"] == "reported" else "a start pile drawn from the seed alone, independent of the victims")
    L = [f"## Radar `{cfg['penetration']}` propagation, {cfg['start']} start", "",
         f"Start: {start_txt}. Propagation: " + (
             "optimistic flat 0.7 dB/m (NOT literature-backed; lets the radar see through 10+ m)."
             if cfg["penetration"] == "legacy" else
             "literature-grade (~3 m of rubble beneath a good scan, < 1 m from an 11 m stand-off)."), "",
         "| metric | " + " | ".join(POLICIES) + " |", "|---|" + "---|" * len(POLICIES)]

    def row(label, f):
        L.append(f"| {label} | " + " | ".join(f(summary[p]) for p in POLICIES) + " |")

    row("victims pinged (all)", lambda s: f"{s['pinged_total']}/{s['victims_total']}")
    row("victims pinged, excl. any at the start pile", lambda s: f"{s['pinged_non_start_total']}/{s['victims_non_start_total']}")
    row("share pinged per seed, excl. start", lambda s: _fmt_ci(s["frac_pinged_non_start"], "{:.0f}%", 100))
    row("time to first non-start ping, median s", lambda s: _fmt_ci(s["t_first_non_start_median"], "{:.0f}")
        + f" ({s['t_first_non_start_none']} seeds none)")
    row("seeds with every victim pinged", lambda s: f"{s['all_pinged_seeds']}/{s['n_seeds']}"
        + (f", median {s['t_all_pinged_median']:.0f} s" if s["t_all_pinged_median"] else ""))
    row("ping localisation error, median / p90 m", lambda s: "n/a" if s["loc_err_median_m"] is None else
        f"{s['loc_err_median_m']:.2f} / {s['loc_err_p90_m']:.2f}")
    row(f"false pings (> {HIT_M:.0f} m from any victim)", lambda s: str(s["false_pings"]))
    row("visits to empty / non-human piles per mission", lambda s: _fmt_ci(s["wasted_visits_mean"], "{:.1f}"))
    row("insufficient-evidence regions per mission", lambda s: f"{s['insufficient_mean']:.1f}")
    row("radar scans used per mission", lambda s: f"{s['scans_mean']:.1f}")
    row("battery swaps per mission", lambda s: f"{s['swaps_mean']:.1f}")
    row("drone flight time per mission, s (sum of 3)", lambda s: _fmt_ci(s["flight_s_mean"], "{:.0f}"))
    row("mission time, s", lambda s: _fmt_ci(s["mission_s_mean"], "{:.0f}"))
    row("depth estimates (radar) / median abs error m", lambda s: f"{s['depth_estimates']}"
        + (f" / {s['depth_err_median_m']:.2f}" if s["depth_err_median_m"] is not None else ""))
    L += ["", "Paired per seed, victims pinged excluding the start pile (acoustic vs baseline): "
          + "; ".join(f"vs {b}: {v['wins']} better, {v['ties']} same, {v['losses']} worse" for b, v in pair.items()) + "."]
    if per_seed:
        L += ["", "| seed | start | " + " | ".join(POLICIES) + " |", "|---|---|" + "---|" * len(POLICIES)]
        by = {(r["seed"], r["policy"]): r for r in rows}
        for seed in cfg["seed_list"]:
            cells = []
            for p in POLICIES:
                r = by.get((seed, p))
                cells.append("-" if not r else f"{r['pinged']}/{r['victims']} pinged, {len(r['visits'])} visits, "
                             f"{r['mission_s']:.0f} s" + (f", {r['false_pings']} false" if r["false_pings"] else ""))
            start = by.get((seed, POLICIES[0]), {}).get("start_pile", "?")
            L.append(f"| {seed} | {start} | " + " | ".join(cells) + " |")
    return "\n".join(L) + "\n"


def markdown(common, sections):
    head = [f"# Grid evaluation: dispatch policies ({common['n_seeds']} seeds)", "",
            f"Generated by `python -m sim.evaluate_grid --report` on {time.strftime('%Y-%m-%d')}. Seeds {common['seeds']}; "
            f"radar `{common['radar']}` (synthetic IF through the real DSP chain); acoustic `{common['acoustic']}`"
            + (" (**synthetic clips generated from the hidden truth**, cleaner than the real acoustic pipeline: "
               "treat the acoustic policy's numbers as an upper bound)" if common["acoustic"] == "stub" else "")
            + f"; mission budget {common['scans']} radar scans, 3 drones, battery reserve 25% with 60 s swaps at the "
            "pads, 8 scans max per pile visit, 3600 s limit. Brackets are bootstrap 95% CIs over seeds.", "",
            "Policies: **acoustic** (ours: listen after each pile, go to the highest acoustic priority, revisit open "
            "leads), **nearest** (nearest unvisited pile, radar only, no listening), **raster** (fixed lawnmower "
            "A1 B1 C1 C2 B2 A2 A3 B3 C3, radar only). All start at the same pile.", "",
            "## How to read this", "",
            "- A ping is a firm radar `candidate` (3 strong agreeing ranges, single dominant mode), matched to the "
            f"nearest hidden victim; within {HIT_M:.0f} m it counts, otherwise it is a false ping. Pings are candidate "
            "search regions, never confirmed people.",
            "- Victims sit at voice / weak-voice / knock piles plus the scenario's reported start pile. The acoustic "
            "policy only visits piles it heard something human-like from, so it spends less time and fewer scans; "
            "the baselines search until the scan budget runs out and can find victims the acoustic agent missed.",
            "- The acoustic policy stops when no lead at or above priority 20 is left; remaining piles stay "
            "\"not cleared\" for ground teams.",
            "- Everything is synthetic (terrain, radar returns, here the acoustic clips). The DSP extractor is "
            "validated on real breathing recordings; the through-rubble loss is a model (docs/RADAR_PENETRATION.md). "
            "Small N: differences inside the CIs are not evidence.", ""]
    return "\n".join(head) + "\n" + "\n".join(sections)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", default="1-20", help="e.g. 1-5 or 1,3,7")
    ap.add_argument("--policies", default=",".join(POLICIES))
    ap.add_argument("--radar", choices=["model", "dsp"], default="dsp")
    ap.add_argument("--acoustic", choices=["stub", "auto"], default="stub",
                    help="stub: synthetic clips from truth (fast); auto: real pipeline / cache (slow on a miss)")
    ap.add_argument("--scans", type=int, default=MISSION_SCANS, help="mission radar-scan budget")
    ap.add_argument("--start", choices=["reported", "independent"], default="reported",
                    help="reported: the scenario's start pile (always holds a victim); independent: drawn from the "
                         "seed alone (R1-007)")
    ap.add_argument("--penetration", choices=["legacy", "literature"], default="legacy",
                    help="radar through-rubble preset (sim/radar_dsp.py)")
    ap.add_argument("--combos", default="legacy:reported,legacy:independent,literature:reported,literature:independent",
                    help="report: penetration:start configurations to include (those with all rows)")
    ap.add_argument("--rows-dir", default=os.path.join(ROOT, "eval", "grid_rows"))
    ap.add_argument("--rerun", action="store_true", help="ignore cached rows")
    ap.add_argument("--report", action="store_true", help="only aggregate cached rows (no missions)")
    ap.add_argument("--out", default=os.path.join(ROOT, "eval", "grid_results.json"))
    ap.add_argument("--md", default=os.path.join(ROOT, "docs", "EVAL_GRID.md"))
    a = ap.parse_args(argv)
    seeds = parse_seeds(a.seeds)
    pols = [p for p in a.policies.split(",") if p]
    rows = []
    for seed in seeds:
        for pol in pols:
            path = _row_path(a.rows_dir, seed, pol, a.radar, a.acoustic, a.scans, a.penetration, a.start)
            if os.path.exists(path) and not a.rerun:
                with open(path) as f:
                    rows.append(json.load(f))
                continue
            if a.report:
                continue
            r = run_one(seed, pol, a.radar, a.acoustic, a.scans, a.penetration, a.start)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w") as f:
                json.dump(r, f, indent=1)
            rows.append(r)
            print(f"seed {seed:3d} {pol:8s}: pinged {r['pinged']}/{r['victims']} (non-start "
                  f"{r['pinged_non_start']}/{r['victims_non_start']}), false {r['false_pings']}, visits "
                  f"{'-'.join(r['visits'])}, {r['mission_s']:.0f} s, swaps {r['battery_swaps']}, {r['wall_s']:.1f} s wall",
                  flush=True)
    if not a.report:
        return
    combos = [c.split(":") for c in a.combos.split(",") if c]
    sections, results = [], {}
    common = {"seeds": a.seeds, "seed_list": seeds, "n_seeds": len(seeds), "radar": a.radar, "acoustic": a.acoustic,
              "scans": a.scans, "hit_m": HIT_M}
    for k, (pen, start) in enumerate(combos):
        rows = []
        for seed in seeds:
            for pol in POLICIES:
                path = _row_path(a.rows_dir, seed, pol, a.radar, a.acoustic, a.scans, pen, start)
                if os.path.exists(path):
                    with open(path) as f:
                        rows.append(json.load(f))
        if len(rows) < len(seeds) * len(POLICIES):
            print(f"{pen}:{start}: {len(rows)}/{len(seeds) * len(POLICIES)} rows; run them first - skipped")
            continue
        by_pol = {p: [r for r in rows if r["policy"] == p] for p in POLICIES}
        summary = {p: summarise(by_pol[p]) for p in POLICIES}
        pair = {b: paired(by_pol["acoustic"], by_pol[b], "pinged_non_start") for b in POLICIES[1:]}
        cfg = dict(common, penetration=pen, start=start)
        sections.append(section(cfg, summary, pair, rows, per_seed=(k == 0)))
        results[f"{pen}:{start}"] = {"config": cfg, "summary": summary, "paired_non_start": pair, "rows": rows}
    if not sections:
        return
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    with open(a.out, "w") as f:
        json.dump(results, f, indent=1)
    md = markdown(common, sections)
    if a.md:
        with open(a.md, "w") as f:
            f.write(md)
    print(md)
    print(f"saved {a.out}" + (f" and {a.md}" if a.md else ""))


if __name__ == "__main__":
    main()
