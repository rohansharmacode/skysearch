#!/usr/bin/env python
"""Impact model driver (sim/impact.py, docs/IMPACT_MODEL.md).

    # 1. real missions, cached per seed in eval/impact_rows/<config>/ (split so no call holds the slot > ~5 min)
    scripts/heavy.sh .venv/bin/python scripts/run_impact.py sim --seeds 1-3
    scripts/heavy.sh .venv/bin/python scripts/run_impact.py sim --seeds 4-6   ...
    # 2. Monte Carlo + charts + data/impact_results.json
    scripts/heavy.sh .venv/bin/python scripts/run_impact.py model --seeds 1-10
"""
from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim import impact  # noqa: E402

ROOT = impact.ROOT


def parse_seeds(spec):
    out = []
    for part in spec.split(","):
        if "-" in part:
            a, b = part.split("-")
            out += list(range(int(a), int(b) + 1))
        elif part:
            out.append(int(part))
    return out


def rows_dir(a):
    tag = f"{a.site}-{a.acoustic}-{a.flight}" + (f"-t{int(a.max_t)}" if a.max_t else "") \
        + (f"-s{a.scans}" if a.scans else "")
    return os.path.join(ROOT, "eval", "impact_rows", tag)


def cmd_sim(a):
    d = rows_dir(a)
    os.makedirs(d, exist_ok=True)
    for seed in parse_seeds(a.seeds):
        path = os.path.join(d, f"seed{seed:03d}.json")
        if os.path.exists(path) and not a.rerun:
            print(f"seed {seed}: cached {path}")
            continue
        r = impact.run_sim(seed, site=a.site, acoustic=a.acoustic, max_t=a.max_t, mission_scans=a.scans,
                           flight_model=a.flight)
        with open(path, "w") as f:
            json.dump(r, f, indent=1)
        v = r["victims"]
        print(f"seed {seed}: {len(v)} victims, {sum(x['t_lead_s'] is not None for x in v)} with a lead, "
              f"{sum(x['t_ping_s'] is not None for x in v)} pinged, {r['scans']} scans, {r['battery_swaps']} swaps, "
              f"{r['flight_s']:.0f} drone-s, {r['mission_s']:.0f} s mission, {r['wall_s']:.0f} s wall", flush=True)


def load_rows(a):
    d = rows_dir(a)
    rows = []
    for seed in parse_seeds(a.seeds):
        path = os.path.join(d, f"seed{seed:03d}.json")
        if os.path.exists(path):
            with open(path) as f:
                rows.append(json.load(f))
        else:
            print(f"missing {path}", file=sys.stderr)
    return rows


def cmd_model(a):
    """Monte Carlo for each site's rows; the first site is primary (charts, headline)."""
    out = {"sites": {}, "primary": None}
    for site in a.sites.split(","):
        a.site = site
        rows = load_rows(a)
        if not rows:
            print(f"{site}: no simulation rows; skipped", file=sys.stderr)
            continue
        print(f"{site}: {len(rows)} missions, {sum(len(r['victims']) for r in rows)} victims", flush=True)
        res = impact.run_model(rows, n=a.n if out["primary"] is None else a.n_secondary, seed=a.mc_seed,
                               tornado_m=a.tornado_m)
        res["config"] = {"rows_dir": os.path.relpath(rows_dir(a), ROOT), "seeds": a.seeds, "mc_seed": a.mc_seed}
        out["sites"][site] = res
        out["primary"] = out["primary"] or site
    if not out["sites"]:
        sys.exit("no simulation rows; run the sim step first")
    prim = out["sites"][out["primary"]]
    out["headline"] = prim["headline"]
    out["params_meta"] = prim["params_meta"]
    out["placeholders_used"] = prim["placeholders_used"]
    path = os.path.join(ROOT, "data", "impact_results.json")
    with open(path, "w") as f:
        json.dump(out, f, indent=1, default=float)
    print(f"saved {os.path.relpath(path, ROOT)}")
    if not a.no_charts:
        make_charts(prim, os.path.join(ROOT, "docs", "img"))
    for site, res in out["sites"].items():
        print(f"== {site}")
        print(json.dumps(res["headline"], default=float))

# ------------------------------------------------------------------------------------------------ charts
BG, PANEL, INK, INK2, RED, GRID = "#0e0f12", "#16181d", "#f2f2f2", "#9a9ca3", "#ff3b30", "#2a2d34"
GREY = "#8d929c"          # today's search
AMBER = "#ffb340"
PINK = "#ff9a8f"          # ours, upper bound (pod pings what it is sent to)
ARMS = (("base", GREY, "Today's search", "-"), ("ours", RED, "With our drones (as simulated)", "-"),
        ("ideal", PINK, "With our drones, radar at design performance (upper bound)", "--"))
SHORT = {"base": "today", "ours": "ours", "ideal": "upper bd"}


def _style(ax):
    ax.set_facecolor(PANEL)
    for sp in ax.spines.values():
        sp.set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=9)
    ax.xaxis.label.set_color(INK2)
    ax.yaxis.label.set_color(INK2)
    ax.grid(color=GRID, lw=0.6, alpha=0.8)
    ax.set_axisbelow(True)


def _fig(w, h, title, sub=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(w, h), facecolor=BG)
    fig.text(0.015, 0.975, title, color=INK, fontsize=14, weight="bold", ha="left", va="top")
    if sub:
        fig.text(0.015, 0.915, sub, color=INK2, fontsize=9.5, ha="left", va="top")
    return fig, plt


def _save(fig, plt, path):
    fig.savefig(path, dpi=130, facecolor=BG)
    plt.close(fig)
    print(f"wrote {os.path.relpath(path, ROOT)}")


def _rng(q, fmt="{:.0f}", k=1.0):
    return f"{fmt.format(q['median'] * k)} ({fmt.format(q['p5'] * k)}-{fmt.format(q['p95'] * k)})"


def make_charts(res, out):
    import numpy as np
    os.makedirs(out, exist_ok=True)
    st, sim = res["stats"], res["sim"]
    foot = (f"Model: {sim['n_seeds']} simulated missions on the Weiguan collapse site x {res['n_draws']} parameter draws. "
            "Synthetic sensors; counterfactual model, not a record of 2016.")

    # 1. time to locate (ECDF)
    fig, plt = _fig(11, 5.6, "How fast trapped people are located",
                    "Share of trapped people located by each hour after the collapse (all victims, all draws)")
    ax = fig.add_axes([0.07, 0.14, 0.62, 0.70])
    _style(ax)
    for arm, col, lab, ls in ARMS:
        e = res["loc_ecdf"][arm]
        ax.plot(e["h"], e["q"], color=col, lw=2.4, ls=ls, label=lab)
    for h in (24, 48, 72):
        ax.axvline(h, color=INK2, lw=0.8, ls=":")
        ax.text(h, 102, f"{h} h", color=INK2, fontsize=8, ha="center")
    ax.set_xscale("log")
    ax.set_xlim(0.5, 240)
    ax.set_ylim(0, 108)
    ax.set_xticks([0.5, 1, 2, 4, 8, 12, 24, 48, 72, 120, 240])
    ax.set_xticklabels(["0.5", "1", "2", "4", "8", "12", "24", "48", "72", "120", "240"])
    ax.set_xlabel("hours after the collapse (log scale)")
    ax.set_ylabel("% of trapped people located")
    ax.legend(facecolor=PANEL, edgecolor=GRID, labelcolor=INK, fontsize=9, loc="upper left")
    t = fig.add_axes([0.72, 0.14, 0.27, 0.70])
    t.axis("off")
    rows = [("Median time to locate", lambda a: _rng(st[f"loc_{a}_med"], "{:.1f}") + " h"),
            ("Located within 24 h", lambda a: _rng(st[f"within24_{a}"], "{:.0f}", 100) + "%"),
            ("Silent / unconscious located\nwithin 24 h", lambda a: _rng(st[f"unr24_{a}"], "{:.0f}", 100) + "%")]
    y = 0.98
    for lab, fn in rows:
        t.text(0, y, lab, color=INK, fontsize=10, weight="bold", va="top")
        yy = y - (0.13 if "\n" in lab else 0.08)
        for arm, col, _, _ in ARMS:
            t.text(0, yy, f"{SHORT[arm]:<9s}" + fn(arm), color=col, fontsize=9.5, va="top", family="monospace")
            yy -= 0.065
        y = yy - 0.05
    t.text(0, -0.02, "median (90% interval over parameter draws)", color=INK2, fontsize=8, va="top")
    fig.text(0.015, 0.02, foot, color=INK2, fontsize=8)
    _save(fig, plt, os.path.join(out, "impact_time_to_locate.png"))

    # 2. survival curve
    fig, plt = _fig(11, 5.6, "Every hour counts: survival of people dug out at each time",
                    "% of trapped people alive when extricated (shaded = low-high range); markers = median time each arm "
                    "gets people out")
    ax = fig.add_axes([0.07, 0.14, 0.88, 0.70])
    _style(ax)
    sp = res["survival_points"]
    xs = np.linspace(0, 120, 400)

    def curve(pts):
        return np.interp(xs, [a for a, _ in pts], [100 * b for _, b in pts])
    ax.fill_between(xs, curve(sp["low"]), curve(sp["high"]), color=INK, alpha=0.08, lw=0)
    ax.plot(xs, curve(sp["central"]), color=INK, lw=2)
    for a, b in sp["central"][1:]:
        ax.plot([a], [100 * b], "o", color=INK, ms=4)
    for key, col, lab, ty in (("ext_base_med", GREY, "today", 97), ("ext_ours_med", RED, "ours, as simulated", 90),
                              ("ext_ideal_med", PINK, "ours, upper bound", 83)):
        h = st[key]["median"]
        s_ = float(np.interp(h, [a for a, _ in sp["central"]], [100 * b for _, b in sp["central"]]))
        ax.axvline(h, color=col, lw=1.4, ls="--")
        ax.plot([h], [s_], "o", color=col, ms=9)
        ax.text(52, ty, f"{lab}: half of people dug out by {h:.1f} h (survival there {s_:.0f}%)", color=col,
                fontsize=10, weight="bold", va="center")
    ax.set_xlim(0, 120)
    ax.set_ylim(0, 105)
    ax.set_xticks([0, 12, 24, 36, 48, 60, 72, 96, 120])
    ax.set_xlabel("hours after the collapse when the person is dug out")
    ax.set_ylabel("% alive at extrication")
    fig.text(0.015, 0.02, "Survival: Kobe 1995 fire-brigade and Weiguan 2016 extrication outcomes, interpolated "
             "(docs/IMPACT_EVIDENCE.md 1b). Extrication time and crews are the same in every arm.", color=INK2, fontsize=8)
    _save(fig, plt, os.path.join(out, "impact_survival.png"))

    # 3. silent / unconscious victims by method
    fig, plt = _fig(11, 5.4, "Who finds the people who cannot call for help",
                    "Silent or unconscious trapped people (they do not answer a call-out): how each is first located, "
                    "within 72 h")
    ax = fig.add_axes([0.20, 0.27, 0.75, 0.56])
    _style(ax)
    ax.grid(False)
    cols = [RED, "#5ac8fa", "#ffcc00", "#af52de", "#6b6f78", "#2a2d34"]
    labs = res["methods"]
    names = [("ideal", "Ours, radar at\ndesign performance"), ("ours", "Ours, as\nsimulated"), ("base", "Today's search")]
    for yi, (arm, name) in enumerate(names):
        sh = np.array(res["method_shares"][arm]["unresponsive"]) * 100
        left = 0
        for k, v in enumerate(sh):
            ax.barh(yi, v, left=left, color=cols[k], edgecolor=BG, height=0.62)
            if v >= 5:
                ax.text(left + v / 2, yi, f"{v:.0f}%", color=BG if k in (1, 2) else INK, ha="center", va="center",
                        fontsize=10, weight="bold")
            left += v
    ax.set_yticks(range(len(names)))
    ax.set_yticklabels([n for _, n in names], color=INK, fontsize=10)
    ax.set_xlim(0, 100)
    ax.set_xlabel("% of silent / unconscious trapped people")
    hs = [plt.Rectangle((0, 0), 1, 1, color=c) for c in cols]
    ax.legend(hs, labs, ncol=3, facecolor=PANEL, edgecolor=GRID, labelcolor=INK, fontsize=9,
              loc="upper center", bbox_to_anchor=(0.45, -0.17))
    fig.text(0.015, 0.02, f"In the {sim['n_seeds']} simulated sorties the pod pinged {sim['unresponsive_pinged']} of "
             f"{sim['n_unresponsive']} silent/unconscious victims; {sim['ideal_eligible']} were radar-reachable in sectors "
             "the drones had a lead on (the upper bound pings those). A call-out cannot find them.", color=INK2, fontsize=8)
    _save(fig, plt, os.path.join(out, "impact_unconscious.png"))

    # 4. rescuer-hours and cost
    fig, plt = _fig(11, 5.4, "Rescuer time tied up searching, and what the drones cost",
                    "Left: rescuer-hours of the search force elapsed before each trapped person is located (mean per person). "
                    "Right: USD")
    ax = fig.add_axes([0.07, 0.14, 0.38, 0.68])
    _style(ax)
    for x, key, col in ((0, "rh_base", GREY), (1, "rh_ours", RED), (2, "rh_ideal", PINK)):
        q = st[key]
        ax.bar(x, q["median"], color=col, width=0.6)
        ax.errorbar(x, q["median"], [[q["median"] - q["p5"]], [q["p95"] - q["median"]]], color=INK, capsize=6, lw=1.2)
        ax.text(x, q["median"] * 0.5, f"{q['median']:,.0f}", color=BG, ha="center", fontsize=11, weight="bold")
    ax.set_xticks([0, 1, 2])
    ax.set_xticklabels(["Today", "Ours", "Upper\nbound"], color=INK, fontsize=10)
    ax.set_ylabel("rescuer-hours per person located")
    ax2 = fig.add_axes([0.56, 0.14, 0.40, 0.68])
    _style(ax2)
    bars = [("Rescuer time\nper person,\ntoday", st["cost_base"], GREY),
            ("Rescuer time\nper person,\nours", st["cost_ours"], RED),
            ("Rescuer time\nper person,\nupper bound", st["cost_ideal"], PINK),
            ("Drones: one\n1-h sortie", st["drone_sortie"], AMBER),
            ("Drones: 72-h\ndeployment", st["drone_72h"], AMBER)]
    for x, (lab, q, col) in enumerate(bars):
        ax2.bar(x, q["median"], color=col, width=0.6)
        ax2.errorbar(x, q["median"], [[q["median"] - q["p5"]], [q["p95"] - q["median"]]], color=INK, capsize=5, lw=1.1)
        ax2.text(x, q["p95"] * 1.03 + 50, f"${q['median']:,.0f}", color=INK, ha="center", fontsize=9, weight="bold")
    ax2.set_xticks(range(len(bars)))
    ax2.set_xticklabels([b[0] for b in bars], color=INK, fontsize=7.5)
    ax2.set_ylabel("USD (median, 90% interval)")
    fig.text(0.015, 0.02, "Rescuer time is an opportunity cost (teams freed to dig), not cash saved: the site is still "
             "searched in full. Drone cost = energy + airframe and battery wear + van share + operators.",
             color=INK2, fontsize=8)
    _save(fig, plt, os.path.join(out, "impact_cost.png"))

    # 5. tornado
    tr = res["tornado"]
    rows = tr["rows"][:5]
    fig, plt = _fig(11, 5.0, "What the answer depends on most",
                    f"Extra survivors per 100 trapped (upper-bound arm minus today) when one parameter moves from its low "
                    f"to its high value, the rest central (central = {tr['central_delta_per100']:.1f})")
    ax = fig.add_axes([0.30, 0.14, 0.66, 0.68])
    _style(ax)
    c0 = tr["central_delta_per100"]
    for i, r in enumerate(rows[::-1]):
        lo, hi = r["low"], r["high"]
        ax.barh(i, lo - c0, left=c0, color=GREY, height=0.55)
        ax.barh(i, hi - c0, left=c0, color=RED, height=0.55)
        ax.text(min(lo, hi) - 0.05, i, f"{min(lo, hi):.1f}", color=INK2, ha="right", va="center", fontsize=9)
        ax.text(max(lo, hi) + 0.05, i, f"{max(lo, hi):.1f}", color=INK2, ha="left", va="center", fontsize=9)
    ax.axvline(c0, color=INK, lw=1)
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels([PLAIN.get(r["param"], r["param"]) + ("" if r["param"] == "survival curve" else
                        f"\n{r['range'][0]:g}-{r['range'][1]:g}") for r in rows[::-1]], color=INK, fontsize=9)
    ax.set_xlabel("extra survivors per 100 trapped people")
    hs = [plt.Rectangle((0, 0), 1, 1, color=c) for c in (GREY, RED)]
    ax.legend(hs, ["parameter at its low value", "parameter at its high value"], facecolor=PANEL, edgecolor=GRID,
              labelcolor=INK, fontsize=9, loc="lower right")
    fig.text(0.015, 0.02, foot, color=INK2, fontsize=8)
    _save(fig, plt, os.path.join(out, "impact_tornado.png"))

    # 6. logistics
    lg = res["logistics"]
    c = lg["central"]
    fig, plt = _fig(12.5, 5.8, "Keeping heavy drones flying: fuel and charging",
                    f"Left: flight minutes for one heavy drone (FlyCart 30 class, ~{c['flight_power_kw']:.1f} kW) from half a "
                    "tank of an average car (25 L). Right: drones each charger keeps working continuously")
    ax = fig.add_axes([0.07, 0.16, 0.33, 0.64])
    _style(ax)
    fuel = list(c["half_tank_flight_min"])
    for i, k in enumerate(fuel):
        v = [lg[e]["half_tank_flight_min"][k] for e in ("low", "central", "high")]
        ax.bar(i, v[1], color=[GREY, RED][i], width=0.6)
        ax.errorbar(i, v[1], [[v[1] - min(v)], [max(v) - v[1]]], color=INK, capsize=6)
        ax.text(i, max(v) + 8, f"{v[1]:.0f} min", color=INK, ha="center", va="bottom", fontsize=12, weight="bold")
    ax.set_xticks(range(len(fuel)))
    ax.set_xticklabels(["through the\ncar alternator", "through a\ngenerator"], color=INK, fontsize=9.5)
    ax.set_ylabel("flight minutes from half a tank (pessimistic-optimistic)")
    ax.set_ylim(0, max(lg["high"]["half_tank_flight_min"].values()) * 1.15)
    ax2 = fig.add_axes([0.62, 0.16, 0.36, 0.64])
    _style(ax2)
    srcs = list(c["drones_sustained"])
    pal = [GREY, AMBER, "#ffcc00", RED, "#5ac8fa"]
    for i, k in enumerate(srcs):
        v = [lg[e]["drones_sustained"][k] for e in ("low", "central", "high")]
        ax2.barh(i, v[1], color=pal[i], height=0.55)
        ax2.errorbar(v[1], i, xerr=[[v[1] - min(v)], [max(v) - v[1]]], color=INK, capsize=5)
        ax2.text(max(v) + 0.1, i, f"{v[1]:.1f}", color=INK, va="center", fontsize=10, weight="bold")
    ax2.axvline(3, color=INK2, lw=0.8, ls=":")
    ax2.text(3.05, len(srcs) - 0.45, "our 3-drone team", color=INK2, fontsize=8)
    ax2.set_yticks(range(len(srcs)))
    ax2.set_yticklabels([k.replace(", ", ",\n").replace(" (", "\n(") for k in srcs], color=INK, fontsize=8.5)
    ax2.set_xlabel(f"drones kept working (rotors on {100 * c['duty']:.0f}% of the time, as simulated)")
    ax2.set_xlim(0, max(max(lg[e]["drones_sustained"].values()) for e in lg) * 1.15)
    fig.text(0.015, 0.02, "Fuel-to-battery paths and charger powers: docs/IMPACT_EVIDENCE.md section 6. Idling a car to burn "
             "half a tank takes ~14 h, so the alternator is a top-up, not a power plant. One C8000 charger per drone.",
             color=INK2, fontsize=8)
    _save(fig, plt, os.path.join(out, "impact_logistics.png"))

PLAIN = {
    "survival curve": "Survival table (low vs high)", "search_teams": "Search teams on site",
    "hasty_search_rate_m2_h": "Call-out search speed (m2/team-h)",
    "search_team_h_per_100m2_floor": "Search effort, team-h per 100 m2", "hasty_share": "Share of effort on call-out pass",
    "hazard_multiplier_unconscious_or_severe": "Death-hazard multiplier, unconscious", "canine_rate_m2_h": "Dog search speed", "p_callout_responsive": "Call-out finds a conscious person",
    "p_callout_unresponsive": "Call-out finds an unconscious person", "canine_start_h": "Search dogs arrive (h)",
    "canine_teams": "Search-dog teams", "p_canine": "Dog finds a person", "camera_rate_m2_h": "Camera search speed (m2/team-h)",
    "p_camera": "Camera finds a person", "debris_removal_start_h": "Debris removal reaches people (h)",
    "pinpoint_h": "Alert -> dig spot (h)", "extrication_h": "Dig-out time per person (h)",
    "extrication_teams": "Dig crews at once", "drone_deploy_h": "Drones airborne after collapse (h)",
    "acoustic_lead_keep": "Share of drone acoustic leads that are real", "ping_keep": "Share of radar pings that hold up",
    "search_start_h": "Search teams start (h)", "deep_detect_factor": "Detection factor, people > 3 m deep", "camera_start_h": "Search cameras arrive (h)",
}


def cmd_charts(a):
    """Redraw the charts from data/impact_results.json (primary site) without re-running the model."""
    with open(os.path.join(ROOT, "data", "impact_results.json")) as f:
        out = json.load(f)
    make_charts(out["sites"][out["primary"]], os.path.join(ROOT, "docs", "img"))


def cmd_table(a):
    """Markdown assumptions table from data/impact_results.json (for docs/IMPACT_MODEL.md)."""
    with open(os.path.join(ROOT, "data", "impact_results.json")) as f:
        res = json.load(f)["sites"]
    res = res[next(iter(res))]
    print("| parameter | low | central | high | unit | origin | note |")
    print("|---|---|---|---|---|---|---|")
    used = set(impact.PLACEHOLDERS) | {k for k in res["params"] if impact.is_surv(k)} | {
        "search_team_h_per_100m2_floor", "canine_effective_search_h_per_12h_shift", "half_tank_flight_min_alternator",
        "half_tank_flight_min_generator", "db2000_energy_wh"}
    skip = {"hasty_search_rate_m2_h", "camera_rate_m2_h", "hub_response_h", "firetruck_response_h", "weiguan_deaths",
            "weiguan_rescued_alive", "half_tank_L", "alt_path_kwh_batt_per_L", "gen_path_kwh_batt_per_L"}
    for k, v in sorted(res["params"].items()):
        if k not in used or k in skip:
            continue
        note = (v.get("note") or "").replace("|", "/")
        note = note if len(note) <= 90 else note[:87] + "..."
        print(f"| `{k}` | {v['low']:g} | {v['central']:g} | {v['high']:g} | {v.get('unit', '')} | "
              f"{v['origin']} | {note} |")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("sim", "model"):
        s = sub.add_parser(name)
        s.add_argument("--seeds", default="1-10")
        s.add_argument("--site", default="collapse")
        s.add_argument("--acoustic", default="stub", choices=["stub", "auto"])
        s.add_argument("--flight", default="smooth", choices=["smooth", "kinematic"])
        s.add_argument("--max-t", dest="max_t", type=float, default=None)
        s.add_argument("--scans", type=int, default=None)
    sub.choices["sim"].add_argument("--rerun", action="store_true")
    m = sub.choices["model"]
    m.add_argument("--n", type=int, default=2000, help="Monte Carlo parameter draws (primary site)")
    m.add_argument("--n-secondary", type=int, default=1000, help="draws for the other sites")
    m.add_argument("--tornado-m", type=int, default=40, help="draws per tornado end")
    m.add_argument("--sites", default="collapse,grid", help="comma list; the first is primary (charts)")
    m.add_argument("--mc-seed", type=int, default=2016)
    m.add_argument("--no-charts", action="store_true")
    sub.add_parser("table")
    sub.add_parser("charts")
    a = ap.parse_args(argv)
    {"sim": cmd_sim, "model": cmd_model, "table": cmd_table, "charts": cmd_charts}[a.cmd](a)


if __name__ == "__main__":
    main()
