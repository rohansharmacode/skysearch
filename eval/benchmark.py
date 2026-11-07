"""Monte Carlo comparison: SkySearch vs lawnmower baseline on identical worlds and events.
Usage: python -m eval.benchmark --seeds 40 --duration 600"""
import argparse, json, time
import numpy as np
from skysearch.config import Config
from skysearch.scenario import Scenario
from skysearch.swarm import Swarm
from eval.baseline import LawnmowerSwarm


def run(seed, cls, duration):
    cfg = Config(seed=seed, duration=duration)
    s = Scenario(cfg, swarm_cls=cls)
    while s.swarm.t < duration and not s.all_found():
        s.step()
    found = [p["found_t"] for p in s.world.people]
    sw = s.swarm
    if cls is Swarm:
        op_load = sum(1 for c in sw.candidates if c["status"] in ("confirmed", "review")) + sw.reviews
        op_false = sum(1 for c in sw.candidates if c["status"] in ("confirmed",) and sw.nearest_truth(c["xy"]) != "person")
    else:
        op_load, op_false = sw.operator_load, sw.operator_false
    ms = float(np.mean([d.compute_ms for d in sw.drones if d.compute_ms > 0]))
    return dict(found=found, all_t=max(found) if all(f is not None for f in found) else None,
                op_load=op_load, op_false=op_false, ms=ms)


def summarise(rows, duration):
    n = len(rows)
    times = [f if f is not None else duration for r in rows for f in r["found"]]
    found_rate = np.mean([f is not None for r in rows for f in r["found"]])
    all_rate = np.mean([r["all_t"] is not None for r in rows])
    t_all = [r["all_t"] if r["all_t"] is not None else duration for r in rows]
    boot = [np.median(np.random.default_rng(i).choice(times, len(times))) for i in range(1000)]
    return dict(runs=n, people_found_pct=round(100 * found_rate, 1), all_found_pct=round(100 * all_rate, 1),
                median_time_to_find_s=float(np.median(times)), median_ci95=[float(np.percentile(boot, 2.5)), float(np.percentile(boot, 97.5))],
                median_time_all_found_s=float(np.median(t_all)),
                operator_msgs_per_run=round(float(np.mean([r["op_load"] for r in rows])), 1),
                false_alarms_to_operator_per_run=round(float(np.mean([r["op_false"] for r in rows])), 2),
                brain_ms_per_tick=round(float(np.mean([r["ms"] for r in rows])), 2))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=40)
    ap.add_argument("--duration", type=float, default=600)
    a = ap.parse_args()
    out = {}
    for name, cls in [("skysearch", Swarm), ("lawnmower", LawnmowerSwarm)]:
        t0 = time.time()
        rows = [run(s, cls, a.duration) for s in range(100, 100 + a.seeds)]
        out[name] = summarise(rows, a.duration)
        print(name, json.dumps(out[name]), f"({time.time() - t0:.0f}s)", flush=True)
    json.dump(out, open("out/benchmark.json", "w"), indent=2)
