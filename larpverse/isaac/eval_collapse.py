"""Case study: the autonomous mission on the Weiguan Jinlong collapse-site reconstruction (EVALUATOR).

Runs the unchanged GridMission on the collapse-site map (``--site collapse``: 320 m reconstruction, 8 search
sectors Z1-Z8 in the 160 m core) over N seeds and three dispatch policies, and scores each run against the
hidden truth with ``sim.evaluate_grid.score_run``. Like sim/evaluate_grid.py it is an EVALUATOR: it reads
``scenario.truth`` only after a run; the mission never sees it.

Everything simulated is synthetic: the trapped people, their sounds, the radar returns and (with
``--acoustic stub``) the acoustic clips are invented, and say nothing about the real event. The real 2016
figures in ``CASE_2016`` are reported numbers from the team's case-study sources, kept here only to put the
simulated timeline next to the real one (docs/CASE_STUDY_WEIGUAN.md, docs/img/case_weiguan_timeline.png).

    python -m isaac.eval_collapse --seeds 1-30 --jobs 4                         # legacy radar, reported start
    python -m isaac.eval_collapse --seeds 1-30 --jobs 4 --penetration literature
    python -m isaac.eval_collapse --seeds 1-30 --jobs 4 --start independent
    python -m isaac.eval_collapse --seeds 1-30 --report                         # -> docs/EVAL_COLLAPSE.md
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from sim.evaluate_grid import HIT_M, POLICIES, _fmt_ci, paired, parse_seeds, section, summarise  # noqa: E402

SECTOR_NAMES = [f"Z{k}" for k in range(1, 9)]
# Raster baseline on this map: serpentine rows from the south staging street (the grid's A1..C3 order has no
# meaning here). Evaluator configuration only; the mission's decision logic is unchanged.
RASTER_COLLAPSE = ["Z8", "Z2", "Z4", "Z6", "Z5", "Z3", "Z1", "Z7"]

# Reported figures for the 6 Feb 2016 Weiguan Jinlong collapse (Yongkang District, Tainan), from the team's
# case-study sheet. Ranges are kept as reported; they differ between sources.
CASE_2016 = {
    "deaths_in_building": "114-115 (2 more elsewhere in Tainan) [BBC, 13 Feb 2016]",
    "rescued_alive": "175-289 [Reuters 13 Feb 2016; CNN 14 Feb 2016; Taipei Times 8 Feb 2016]",
    "injured": "550, of whom 289 were pulled out of the rubble [CNN 14 Feb 2016; ReliefWeb 14 Feb 2016]",
    "initially_trapped": "120-155+ in the first 24-48 h [Taipei Times 8 Feb 2016; Reuters 13 Feb 2016]",
    "rescue_duration": "7-7.5 days [BBC 13 Feb 2016; ReliefWeb 14 Feb 2016]",
    "last_live_rescue_h": "52-60 h after the quake (day 3) [BBC 13 Feb 2016; ReliefWeb 14 Feb 2016]",
    "response": "~3,900 emergency workers, 140 vehicles, 2 helicopters city-wide [Taipei Times 8 Feb 2016]",
}
CASE_SOURCES = [
    "BBC News, \"Taiwan quake death toll reaches 116, as rescue ends\", 13 Feb 2016. https://www.bbc.com/news/world-asia-35569224",
    "Reuters, \"Death toll at collapsed building in Taiwanese city reaches 114 as rescue efforts end\", 13 Feb 2016. "
    "https://www.reuters.com/article/world/death-toll-at-collapsed-building-in-taiwanese-city-reaches-114-as-rescue-efforts-idUSKCN0VM0A7/",
    "CNN, \"Taiwan earthquake: Death toll rises to 114\", 13-14 Feb 2016. https://www.cnn.com/2016/02/13/asia/taiwan-earthquake",
    "ReliefWeb / Xinhua, \"Taiwan quake death toll at 116 as search ends\", 14 Feb 2016. "
    "https://reliefweb.int/report/china-taiwan-province/taiwan-quake-death-toll-116-search-ends",
    "Taipei Times, \"Earthquake Aftermath: Quake death toll in Tainan rises to 29\", 8 Feb 2016. "
    "https://www.taipeitimes.com/News/front/archives/2016/02/08/2003639061",
    "Wikipedia, \"2016 southern Taiwan earthquake\". https://en.wikipedia.org/wiki/2016_southern_Taiwan_earthquake",
]


def independent_start(seed):
    """Start sector drawn from the seed alone, independently of where the scenario placed anyone (R1-007)."""
    return SECTOR_NAMES[int(np.random.default_rng([int(seed), 4242]).integers(0, len(SECTOR_NAMES)))]


def run_one(seed, policy, penetration="legacy", start="reported", radar="dsp", acoustic="stub"):
    """One mission on the collapse map, scored against the hidden truth (evaluator only)."""
    import sim.grid_mission as gm
    from sim.evaluate_grid import score_run
    from isaac import site
    gm.RASTER = RASTER_COLLAPSE
    t0 = time.time()
    kw = dict(seed=seed, radar=radar, acoustic=acoustic, verbose=False, dispatch_policy=policy,
              penetration=penetration)
    if start == "independent":
        kw["launch_pile"] = independent_start(seed)
    m = site.mission_for_site("collapse", **kw).run()
    r = score_run(m)
    truth = m.scenario.truth                                  # hidden truth: evaluator only
    occupied = {v["pile"] for v in truth["victims"]}
    flagged = {pid for pid, p in m.piles.items() if p.get("status") in ("ping", "lead")}
    r.update(penetration=penetration, start=start, site="collapse",
             flagged=sorted(flagged), victims_in_flagged=sum(1 for v in truth["victims"] if v["pile"] in flagged),
             occupied=sorted(occupied), flagged_occupied=len(flagged & occupied),
             flagged_empty=len(flagged - occupied), wall_s=round(time.time() - t0, 2),
             summary=m.final_summary())
    return r


def _row_path(rows_dir, seed, policy, penetration, start):
    return os.path.join(rows_dir, f"dsp-{penetration}-stub" + ("-indep" if start == "independent" else ""),
                        f"seed{seed:03d}-{policy}.json")


def _job(args):
    os.environ["OMP_NUM_THREADS"] = os.environ["OPENBLAS_NUM_THREADS"] = "1"
    seed, pol, pen, start, path = args
    r = run_one(seed, pol, pen, start)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        json.dump(r, f, indent=1)
    return r


def extra_rows(summary, rows):
    """Case-study rows: how much of the synthetic trapped population sits in a sector the mission flagged."""
    for p in POLICIES:
        rs = [r for r in rows if r["policy"] == p]
        summary[p]["victims_in_flagged_total"] = sum(r["victims_in_flagged"] for r in rs)
        summary[p]["frac_in_flagged"] = _boot_ci_mean([r["victims_in_flagged"] / max(r["victims"], 1) for r in rs])
        summary[p]["flagged_empty_mean"] = float(np.mean([r["flagged_empty"] for r in rs]))
        summary[p]["t_first_ping_median"] = (float(np.median([r["t_first_ping"] for r in rs if r["t_first_ping"] is not None]))
                                             if any(r["t_first_ping"] is not None for r in rs) else None)
    L = ["", "Case-study rows (sector triage: a sector is *flagged* if it ends with a ping or an open acoustic lead):", "",
         "| metric | " + " | ".join(POLICIES) + " |", "|---|" + "---|" * len(POLICIES)]
    L.append("| synthetic trapped people in a flagged sector | " + " | ".join(
        f"{summary[p]['victims_in_flagged_total']}/{summary[p]['victims_total']}" for p in POLICIES) + " |")
    L.append("| share per seed | " + " | ".join(_fmt_ci(summary[p]["frac_in_flagged"], "{:.0f}%", 100) for p in POLICIES) + " |")
    L.append("| flagged sectors with nobody in them, per mission | " + " | ".join(
        f"{summary[p]['flagged_empty_mean']:.1f}" for p in POLICIES) + " |")
    L.append("| time to first ping (incl. start sector), median s | " + " | ".join(
        "n/a" if summary[p]["t_first_ping_median"] is None else f"{summary[p]['t_first_ping_median']:.0f}"
        for p in POLICIES) + " |")
    return "\n".join(L) + "\n"


def _boot_ci_mean(x, n=2000, seed=0):
    x = np.asarray(x, float)
    if len(x) == 0:
        return None, None, None
    rng = np.random.default_rng(seed)
    bs = [x[rng.integers(0, len(x), len(x))].mean() for _ in range(n)]
    return float(x.mean()), float(np.percentile(bs, 2.5)), float(np.percentile(bs, 97.5))


def markdown(common, sections):
    head = [f"# Collapse-site evaluation: dispatch policies on the Weiguan reconstruction ({common['n_seeds']} seeds)", "",
            f"Generated by `python -m isaac.eval_collapse --report` on {time.strftime('%Y-%m-%d')}. Site: the 320 m "
            "collapse-site reconstruction (`--site collapse`, 8 search sectors Z1-Z8 in the 160 m core, staging on the "
            f"south street). Seeds {common['seeds']}; radar `dsp` (synthetic IF through the real DSP chain); acoustic "
            "`stub` (**synthetic clips generated from the hidden truth**, cleaner than the real acoustic pipeline: treat "
            "the acoustic policy's numbers as an upper bound); the mission's default scan budget, 3 drones, battery "
            "swaps at the pads. Brackets are bootstrap 95% CIs over seeds.", "",
            "Policies: **acoustic** (ours), **nearest** (nearest unvisited sector, radar only), **raster** (serpentine "
            "Z8 Z2 Z4 Z6 Z5 Z3 Z1 Z7 from the staging street, radar only). All start at the same sector.", "",
            "**The trapped people, sounds and radar returns are invented.** The geometry is a reconstruction of the "
            "2016 Weiguan Jinlong collapse site; nothing here describes where anyone actually was. See "
            "`docs/CASE_STUDY_WEIGUAN.md` for how this relates to the real 2016 response.", "",
            "## How to read this", "",
            f"- A ping is a firm radar `candidate`, matched to the nearest hidden person; within {HIT_M:.0f} m it counts. "
            "Pings are candidate search regions, never confirmed people.",
            "- The acoustic policy only visits sectors it heard something human-like from; the others stay \"not "
            "cleared\" for ground teams. The baselines search until the scan budget runs out.",
            "- Small N: differences inside the CIs are not evidence.", ""]
    return "\n".join(head) + "\n" + "\n".join(sections)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", default="1-30")
    ap.add_argument("--policies", default=",".join(POLICIES))
    ap.add_argument("--penetration", choices=["legacy", "literature"], default="legacy")
    ap.add_argument("--start", choices=["reported", "independent"], default="reported")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--combos", default="legacy:reported,legacy:independent,literature:reported")
    ap.add_argument("--rows-dir", default=os.path.join(ROOT, "eval", "collapse_rows"))
    ap.add_argument("--rerun", action="store_true")
    ap.add_argument("--report", action="store_true", help="only aggregate cached rows (no missions)")
    ap.add_argument("--out", default=os.path.join(ROOT, "eval", "collapse_results.json"))
    ap.add_argument("--md", default=os.path.join(ROOT, "docs", "EVAL_COLLAPSE.md"))
    a = ap.parse_args(argv)
    seeds = parse_seeds(a.seeds)
    if not a.report:
        todo = []
        for seed in seeds:
            for pol in [p for p in a.policies.split(",") if p]:
                path = _row_path(a.rows_dir, seed, pol, a.penetration, a.start)
                if a.rerun or not os.path.exists(path):
                    todo.append((seed, pol, a.penetration, a.start, path))
        with ProcessPoolExecutor(max_workers=a.jobs) as ex:
            for fut in as_completed([ex.submit(_job, t) for t in todo]):
                r = fut.result()
                print(f"seed {r['seed']:3d} {r['policy']:8s}: pinged {r['pinged']}/{r['victims']}, flagged "
                      f"{'-'.join(r['flagged']) or '-'}, visits {'-'.join(r['visits'])}, {r['mission_s']:.0f} s, "
                      f"{r['wall_s']:.1f} s wall", flush=True)
        return
    sections, results = [], {}
    common = {"seeds": a.seeds, "seed_list": seeds, "n_seeds": len(seeds)}
    for pen, start in (c.split(":") for c in a.combos.split(",") if c):
        rows = []
        for seed in seeds:
            for pol in POLICIES:
                path = _row_path(a.rows_dir, seed, pol, pen, start)
                if os.path.exists(path):
                    with open(path) as f:
                        rows.append(json.load(f))
        if len(rows) < len(seeds) * len(POLICIES):
            print(f"{pen}:{start}: {len(rows)}/{len(seeds) * len(POLICIES)} rows; run them first - skipped")
            continue
        summary = {p: summarise([r for r in rows if r["policy"] == p]) for p in POLICIES}
        by = {p: [r for r in rows if r["policy"] == p] for p in POLICIES}
        pair = {b: paired(by["acoustic"], by[b], "pinged_non_start") for b in POLICIES if b != "acoustic"}
        cfg = {"penetration": pen, "start": start, "seed_list": seeds}
        md = section(cfg, summary, pair, rows).replace("start pile", "start sector").replace("piles", "sectors")
        sections.append(md + extra_rows(summary, rows))
        results[f"{pen}:{start}"] = summary
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w") as f:
        json.dump({"common": {k: v for k, v in common.items() if k != "seed_list"}, "case_2016": CASE_2016,
                   "results": results}, f, indent=1)
    with open(a.md, "w") as f:
        f.write(markdown(common, sections))
    print(f"wrote {a.md} and {a.out}")


if __name__ == "__main__":
    main()
