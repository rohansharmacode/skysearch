"""Evaluate adaptive vs fixed-site scan policies over N seeds with the same scan budget.

This module (with radar_model.py) is allowed to read hidden ground truth: it compares the
mission's final belief with ``world.targets``.  It writes eval/results.json, which is never
served by the dashboard.

    nice -n 19 python -m sim.evaluate --n 30 --budget 6
"""
from __future__ import annotations

import argparse
import json
import os
import time

import numpy as np

from .coordinator import Mission

FALSE_ALERT_M = 3.0   # confident ("candidate") but this far from the truth


def run_one(seed: int, policy: str, budget: int, radar: str = "model") -> dict:
    t0 = time.time()
    m = Mission(seed, policy=policy, budget=budget, out_dir=None, verbose=False, radar=radar).run()
    runtime = time.time() - t0
    truth = next(t for t in m.world.targets if t["kind"] == "buried")["xy"]   # ground truth: evaluator only
    res = m.result
    if res["modes"]:
        est = res["modes"][0]["xy"]
    else:
        p = m.loc.belief()
        j, i = np.unravel_index(p.argmax(), p.shape)
        est = [float(m.grid.xs()[i]), float(m.grid.ys()[j])]
    err = float(np.hypot(est[0] - truth[0], est[1] - truth[1]))
    ix, iy = m.grid.index(*truth)
    in95 = bool(m.loc.region95()[iy, ix])
    resolved = res["status"] == "candidate"
    return {"seed": seed, "policy": policy, "error_m": round(err, 2), "truth_in_95": in95,
            "resolved": resolved, "false_alert": bool(resolved and err > FALSE_ALERT_M),
            "scans": m.scans_cmd, "relocations": len(m.relocations), "sim_time_s": round(m.t, 1),
            "runtime_s": round(runtime, 2), "area95_m2": round(float(m.loc.region95().sum() * m.grid.res ** 2), 1)}


def summarise(rows):
    a = lambda k: np.array([r[k] for r in rows], float)
    resolved = np.array([r["resolved"] for r in rows])
    err = a("error_m")
    return {"n": len(rows), "median_error_m": float(np.median(err)), "mean_error_m": float(err.mean()),
            "median_error_resolved_m": float(np.median(err[resolved])) if resolved.any() else None,
            "truth_in_95_pct": float(100 * a("truth_in_95").mean()), "resolved_pct": float(100 * resolved.mean()),
            "false_alerts": int(sum(r["false_alert"] for r in rows)), "mean_scans": float(a("scans").mean()),
            "mean_sim_time_s": float(a("sim_time_s").mean()), "mean_area95_m2": float(a("area95_m2").mean()),
            "mean_runtime_s": float(a("runtime_s").mean())}


def table(summary):
    cols = [("median err (m)", "median_error_m", "{:.2f}"), ("mean err (m)", "mean_error_m", "{:.2f}"),
            ("truth in 95% (%)", "truth_in_95_pct", "{:.0f}"), ("resolved (%)", "resolved_pct", "{:.0f}"),
            ("false alerts", "false_alerts", "{:d}"), ("mean scans", "mean_scans", "{:.2f}"),
            ("mean sim time (s)", "mean_sim_time_s", "{:.0f}"), ("mean 95% area (m2)", "mean_area95_m2", "{:.0f}"),
            ("runtime / mission (s)", "mean_runtime_s", "{:.2f}")]
    lines = ["| policy | " + " | ".join(c[0] for c in cols) + " |", "|---|" + "---|" * len(cols)]
    for pol, s in summary.items():
        lines.append(f"| {pol} | " + " | ".join(fmt.format(s[k]) for _, k, fmt in cols) + " |")
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=30)
    ap.add_argument("--first-seed", type=int, default=1)
    ap.add_argument("--budget", type=int, default=6)
    ap.add_argument("--out", default="eval/results.json")
    ap.add_argument("--radar", choices=["model", "dsp"], default="model")
    a = ap.parse_args(argv)
    rows = {"adaptive": [], "fixed": []}
    for seed in range(a.first_seed, a.first_seed + a.n):
        for pol in rows:
            rows[pol].append(run_one(seed, pol, a.budget, a.radar))
        print(f"seed {seed}: " + "  ".join(f"{p} err {rows[p][-1]['error_m']:.1f} m scans {rows[p][-1]['scans']}"
                                           f"{' ok' if rows[p][-1]['resolved'] else ''}" for p in rows), flush=True)
    summary = {p: summarise(r) for p, r in rows.items()}
    print()
    print(table(summary))
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    with open(a.out, "w") as f:
        json.dump({"config": {"n": a.n, "first_seed": a.first_seed, "budget": a.budget, "radar": a.radar,
                              "false_alert_m": FALSE_ALERT_M}, "summary": summary, "rows": rows}, f, indent=1)
    print(f"\nsaved {a.out}")


if __name__ == "__main__":
    main()
