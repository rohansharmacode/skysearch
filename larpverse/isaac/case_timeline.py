"""Figure: the simulated drone triage on the collapse-site reconstruction next to the reported 2016 timeline.

Reads eval/collapse_results.json (python -m isaac.eval_collapse --report) and writes
docs/img/case_weiguan_timeline.png. The 2016 lane is reported figures (isaac/eval_collapse.py CASE_2016);
the simulation lane is synthetic (invented trapped people, synthetic radar and acoustic clips).

    python -m isaac.case_timeline
"""
from __future__ import annotations

import argparse
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
SIM, REAL = "#2a78d6", "#eb6834"          # categorical slots 1 and 2 (validated pair)
MIN, HOUR, DAY = 1 / 60, 1.0, 24.0        # x axis in hours


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", default=os.path.join(ROOT, "eval", "collapse_results.json"))
    ap.add_argument("--out", default=os.path.join(ROOT, "docs", "img", "case_weiguan_timeline.png"))
    a = ap.parse_args(argv)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    for f in ("/usr/share/fonts/opentype/inter/Inter-Regular.otf", "/usr/share/fonts/opentype/inter/Inter-SemiBold.otf"):
        if os.path.exists(f):
            font_manager.fontManager.addfont(f)
    plt.rcParams.update({"font.family": "Inter", "font.size": 11, "axes.edgecolor": AXIS, "axes.labelcolor": INK2,
                         "xtick.color": MUTED, "ytick.color": INK2})
    res = json.load(open(a.results))["results"]
    leg = res["legacy:reported"]["acoustic"]
    lit = res.get("literature:reported", {}).get("acoustic")

    def ci_h(t):                                   # (value, lo, hi) seconds -> hours
        return [None if x is None else x / 3600 for x in t]

    fig, ax = plt.subplots(figsize=(12, 4.9), dpi=200)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)
    ax.set_xscale("log")
    ax.set_xlim(0.5 * MIN, 12 * DAY)
    ax.set_ylim(-0.6, 1.75)
    ticks = [MIN, 10 * MIN, HOUR, 6 * HOUR, DAY, 3 * DAY, 7 * DAY]
    ax.set_xticks(ticks)
    ax.set_xticklabels(["1 min", "10 min", "1 h", "6 h", "1 day", "3 days", "7 days"])
    ax.minorticks_off()
    ax.grid(axis="x", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    ax.set_yticks([1.0, 0.0])
    ax.set_yticklabels(["2016 response\n(reported)", "3-drone swarm\n(simulated)"], fontsize=11.5)
    ax.tick_params(axis="y", length=0)

    ax.set_xlabel("time since the quake (2016) or since drone launch (simulation), log scale", fontsize=10, color=MUTED)

    def band(y, x0, x1, color, label, above=True, dy=0.2, lw=9):
        ax.plot([x0, x1], [y, y], color=color, linewidth=lw, solid_capstyle="round", zorder=3)
        ax.text((x0 * x1) ** 0.5, y + (dy if above else -dy), label, ha="center", va="bottom" if above else "top",
                fontsize=10, color=INK, zorder=4)

    def dot(y, x, color, label, above=True, dy=0.2, ha="center"):
        ax.plot([x], [y], "o", markersize=9, color=color, markeredgecolor=SURFACE, markeredgewidth=2, zorder=4)
        ax.text(x, y + (dy if above else -dy), label, ha=ha, va="bottom" if above else "top", fontsize=10, color=INK,
                zorder=4)

    # 2016 (reported; t = 0 at the quake, 03:57 on 6 Feb 2016)
    band(1.17, 24 * HOUR, 48 * HOUR, REAL, "120–155+ reported trapped\n(first 24–48 h)", above=True, dy=0.08, lw=3)
    band(1.0, 52 * HOUR, 60 * HOUR, REAL, "last live rescues\n52–60 h", above=False)
    band(1.0, 7 * DAY, 7.5 * DAY, REAL, "search ends\n7–7.5 days", above=True)
    # simulation (synthetic; t = 0 at drone launch)
    fp = leg.get("t_first_ping_median")
    if fp:
        dot(0.0, fp / 3600, SIM, f"first candidate region\n{fp / 60:.1f} min (median)", above=True, ha="center")
    ms = ci_h(leg["mission_s_mean"])
    band(0.0, ms[1], ms[2], SIM, f"every sector triaged\n{leg['mission_s_mean'][0] / 60:.1f} min "
         f"(95% CI {leg['mission_s_mean'][1] / 60:.1f}–{leg['mission_s_mean'][2] / 60:.1f})", above=False, dy=0.18)
    ax.set_title("Weiguan Jinlong collapse, Tainan 2016: reported rescue timeline vs simulated drone triage",
                 loc="left", fontsize=13.5, color=INK, fontweight="semibold", pad=14)
    n = json.load(open(a.results))["common"]["n_seeds"]
    note = (f"Simulation: our acoustic-dispatch mission on the collapse-site reconstruction, {n} seeds, invented trapped "
            "people, synthetic radar (optimistic propagation) and acoustic clips; clock starts at drone launch, so add the "
            "time to get a team on site. \"Triaged\" = every sector pinged, flagged as an acoustic lead, or marked not cleared; "
            "not a rescue.")
    if lit:
        note += (f" With literature-grade radar propagation the mission ends in {lit['mission_s_mean'][0] / 60:.0f} min, "
                 f"but radar pings only {lit['pinged_total']} of {lit['victims_total']} invented people; the triage then "
                 "rests on listening.")
    note += " 2016 figures: BBC, Reuters, CNN, ReliefWeb, Taipei Times (Feb 2016)."
    fig.text(0.012, 0.015, note, fontsize=8.6, color=INK2, wrap=True, ha="left", va="bottom")
    fig.subplots_adjust(left=0.12, right=0.98, top=0.86, bottom=0.3)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    fig.savefig(a.out, facecolor=SURFACE)
    print(a.out)


if __name__ == "__main__":
    main()
