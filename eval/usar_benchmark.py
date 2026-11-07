"""Why three drone types? Same buildings, same aftershock, different fleets (earthquake scenario).

    python -m eval.usar_benchmark --seeds 12 --workers 3

Every fleet has 10 airframes: the coordinator mothership plus 9 others. Variants:
  skysearch      1 coordinator + 3 scouts (2 LiDAR, 1 radar) + 6 crevice drones (4 listeners, 2 confirmers);
                 2 listeners stay landed all mission, the other 2 lift off to search voids after the call-out
  scouts_only    1 coordinator + 9 scouts: big drones that cannot enter voids under 0.9 m
  crevice_only   1 coordinator + 9 crevice drones: no long-range LiDAR mapping
  no_listeners   SkySearch, but all 6 crevice drones confirm/explore (nobody lands to listen)
  no_radar       SkySearch without breathing radar (camera-only confirmation)
  sentinels_0/1, listeners_stay   how many listeners stay landed (0, 1, or all 4)
Default: survivors placed at random each seed; one aftershock between 110 and 170 s.
--story: storyboard-type buildings (A conscious in the west wing, B unconscious behind a collapsed shelf
until 110 s, C sealed in; fire in the east wing); A and B are scored.
"""
import argparse, json, os, time
from dataclasses import replace
from multiprocessing import Pool
import numpy as np
from skysearch.usar.config import USARConfig
from skysearch.usar.scenario import USARScenario

VARIANTS = {
    "skysearch": {},
    "scouts_only": dict(n_scouts=9, n_micro=0, n_listeners=0),
    "crevice_only": dict(n_scouts=0, n_micro=9, n_listeners=5, radar_scouts=()),
    "no_listeners": dict(n_listeners=0),
    "sentinels_1": dict(n_sentinels=1),
    "sentinels_0": dict(n_sentinels=0),
    "listeners_stay": dict(n_sentinels=4),
    "no_radar": dict(radar_scouts=(), micro_radar=False),
}


def run(job):
    name, seed, duration, story = job
    cfg = replace(USARConfig(seed=seed, duration=duration, story=story), **VARIANTS[name])
    s = USARScenario(cfg)
    f = s.fleet
    t0 = time.time()
    try:
        while f.t < duration and not s.all_found():
            s.step()
        err = None
    except Exception as e:                       # a variant the fleet code can't fly counts as a failure, reported
        err = f"{type(e).__name__}: {e}"
    found = [x["found_t"] for x in s.b.survivors]
    if story:          # storyboard placement: C is sealed in and silent by design, so score A and B only
        found = found[:2]
    false_pos = sum(1 for x in f.found if min(np.linalg.norm(sv["pos"] - x["xy"]) for sv in s.b.survivors) > 1.5)
    return dict(variant=name, seed=seed, found=found, lost=f.lost_drones, false_pos=false_pos,
                reviews=f.reviews, wall_s=round(time.time() - t0, 1), error=err)


def summarise(rows, duration):
    times = [t for r in rows for t in r["found"]]
    ok = [t for t in times if t is not None]
    return dict(runs=len(rows), people_found_pct=round(100 * len(ok) / len(times), 1),
                median_time_to_find_s=float(np.median([t if t is not None else duration for t in times])),
                all_found_pct=round(100 * float(np.mean([all(t is not None for t in r["found"]) for r in rows])), 1),
                false_confirmations_per_run=round(float(np.mean([r["false_pos"] for r in rows])), 2),
                human_reviews_per_run=round(float(np.mean([r["reviews"] for r in rows])), 2),
                drones_lost_per_run=round(float(np.mean([r["lost"] for r in rows])), 2),
                errors=sum(1 for r in rows if r["error"]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=12)
    ap.add_argument("--duration", type=float, default=600)
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--variants", default=",".join(VARIANTS))
    ap.add_argument("--out", default="out/usar_benchmark.json")
    ap.add_argument("--story", action="store_true",
                    help="storyboard-type buildings: A knocks, B unconscious behind a shelf until 110 s, fire in the east wing")
    a = ap.parse_args()
    names = a.variants.split(",")
    jobs = [(n, s, a.duration, a.story) for s in range(100, 100 + a.seeds) for n in names]
    with Pool(a.workers) as p:
        rows = p.map(run, jobs)
    res = {n: summarise([r for r in rows if r["variant"] == n], a.duration) for n in names}
    for n in names:
        print(f"{n:13s}", res[n])
    for r in rows:
        if r["error"]:
            print("ERROR", r["variant"], r["seed"], r["error"])
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(dict(summary=res, runs=rows), open(a.out, "w"), indent=1)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
