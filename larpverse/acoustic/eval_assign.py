"""Pile-assignment evaluation for the acoustic pipeline (EVALUATOR: reads scenario truth).

For each scenario seed it runs the numpy grid mission (sim/grid_mission.py, default 3-pile budget)
with acoustic.run.listen_and_rank (deterministic fallback agent) as its acoustic_fn, so every listen
uses the mission's real landing spots, visited-pile exclusions and listen seeds, and scores each
listen's per-pile assignment against the scenario truth.
Truth is read only here, after the analysis has finished; it never reaches dsp/run/agent.

    # one heavy job per seed (results cached in acoustic/cache/eval/<algo>/seed<N>.json)
    scripts/heavy.sh .venv-audio/bin/python -m acoustic.eval_assign run --seeds 7
    # table from the cache (cheap); --algos compares algorithm versions, e.g. before vs after
    .venv-audio/bin/python -m acoustic.eval_assign report --algos v1-baseline,<dsp.ALGO_VERSION>
    # run missing seeds then report (default 5 seeds: 3 7 11 19 23)
    .venv-audio/bin/python -m acoustic.eval_assign all
    # compact committed test fixture from the cached run (tests/fixtures/acoustic_seed7.json)
    .venv-audio/bin/python -m acoustic.eval_assign fixture --seeds 7

Metrics per pile (start pile excluded):
  detect   : any clip assigned to the pile        vs  pile has any sound source
  human    : human lead (human >= 0.4) on the pile vs  pile holds voice / weak_voice / knock
  confident: human lead whose association is not ambiguous (same truth as `human`)
  false_conf: confident human leads on piles with no voice/knock source (must be 0)
  false_hum : any human lead (incl. "ambiguous direction") on such piles
  voice_top@1: in the first listen, the dispatch (top-ranked) pile is the scenario's `voice` pile
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
if ROOT not in sys.path:
    sys.path.append(ROOT)
EVAL_DIR = os.path.join(HERE, "cache", "eval")
DEFAULT_SEEDS = [3, 7, 11, 19, 23]
HUMAN_KINDS = {"voice", "weak_voice", "knock"}
HUMAN_LEAD = 0.4


def algo_version() -> str:
    from acoustic import dsp
    return getattr(dsp, "ALGO_VERSION", "v1-baseline")


class _Stop(Exception):
    pass


def mission_first_listen(seed: int) -> dict:
    """Drone positions / exclusions / listen seed of the grid mission's first listen (numpy only)."""
    from sim.grid_mission import GridMission
    got = {}

    def fn(sc, drones, exclude, lseed):
        got.update(drones={k: [float(v[0]), float(v[1])] for k, v in drones.items()},
                   exclude=list(exclude), listen_seed=int(lseed))
        raise _Stop

    m = GridMission(seed, verbose=False, acoustic_fn=fn)
    try:
        m.run()
    except _Stop:
        pass
    if not got:
        raise RuntimeError(f"mission seed {seed} never reached a listen")
    got["start_pile"] = m.start_pile
    return got


def mission_listens(seed: int, max_listens: int = 3) -> list[dict]:
    """Run the numpy grid mission with the real acoustic pipeline (fallback agent) as its
    acoustic_fn and return every listen: config + larpverse.acoustic/v1 state."""
    from acoustic.run import listen_and_rank
    from sim.grid_mission import GridMission
    listens = []

    def fn(sc, drones, exclude, lseed):
        t0 = time.time()
        st = listen_and_rank(sc, drones, list(exclude), lseed, mode="fallback")
        listens.append({"drones": {k: [float(v[0]), float(v[1])] for k, v in drones.items()},
                        "exclude": list(exclude), "listen_seed": int(lseed),
                        "wall_s": round(time.time() - t0, 1), "state": st})
        if len(listens) >= max_listens:
            raise _Stop
        return st

    m = GridMission(seed, verbose=False, acoustic_fn=fn)
    try:
        m.run()
    except _Stop:
        pass
    for li in listens:
        li["start_pile"] = m.start_pile
    return listens


def _path(algo, seed):
    return os.path.join(EVAL_DIR, algo, f"seed{seed}.json")


def run_seed(seed: int, force=False, max_listens=3) -> dict:
    algo = algo_version()
    path = _path(algo, seed)
    if os.path.exists(path) and not force:
        with open(path) as f:
            return json.load(f)
    from sim.scenario import make_scenario
    t0 = time.time()
    listens = mission_listens(seed, max_listens)
    roles = dict(make_scenario(seed).truth["roles"])     # EVALUATOR ONLY, after the analysis ran
    rec = {"algo": algo, "seed": seed, "wall_s": round(time.time() - t0, 1), "truth_roles": roles,
           "listens": listens}
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path + ".tmp", "w") as f:
        json.dump(rec, f, indent=1)
    os.replace(path + ".tmp", path)
    return rec


def score_listen(roles: dict, li: dict, seed: int, k: int) -> dict:
    st = li["state"]
    excl = set(st.get("excluded_piles", [])) | set(li["exclude"])
    piles = [p for p in roles if p not in excl]
    by_pile = {}
    for c in st["clips"]:
        by_pile.setdefault(c["pile_id"], []).append(c)
    pst = {p["pile_id"]: p for p in st["piles"]}

    def ambiguous(c):
        return c.get("association", "confident") != "confident"

    rows, cnt = [], {m: [0, 0, 0] for m in ("detect", "human", "confident")}   # tp, fp, fn
    false_conf, false_hum = [], []
    for p in sorted(piles):
        kind = roles[p]
        cs = by_pile.get(p, [])
        hum = [c for c in cs if c["human"] >= HUMAN_LEAD]
        conf = [c for c in hum if not ambiguous(c)]
        t_src, t_hum = kind != "silent", kind in HUMAN_KINDS
        for m, pred, tr in (("detect", bool(cs), t_src), ("human", bool(hum), t_hum),
                            ("confident", bool(conf), t_hum)):
            cnt[m][0] += pred and tr
            cnt[m][1] += pred and not tr
            cnt[m][2] += tr and not pred
        if conf and not t_hum:
            false_conf.append(p)
        if hum and not t_hum:
            false_hum.append(p)
        top = max(cs, key=lambda c: c["priority"]) if cs else None
        rows.append({"pile": p, "truth": kind, "status": pst.get(p, {}).get("status", "-"),
                     "priority": pst.get(p, {}).get("priority"),
                     "class": top["sound_class"] if top else "-", "human": top["human"] if top else None,
                     "rel": top["reliability"] if top else None,
                     "assoc": (top.get("association", "confident") if top else "-"),
                     "assoc_conf": (top.get("association_conf") if top else None),
                     "transcript": (top["transcript"][:40] if top else "")})
    voice_pile = next((p for p, kk in roles.items() if kk == "voice"), None)
    disp = st["dispatch"]["pile_id"]
    return {"seed": seed, "listen": k, "rows": rows, "counts": cnt, "false_conf": false_conf,
            "false_hum": false_hum, "voice_pile": voice_pile, "dispatch": disp, "visited": sorted(excl),
            "voice_top": (disp == voice_pile) if voice_pile not in excl else None}


def score(rec: dict) -> list[dict]:
    return [score_listen(rec["truth_roles"], li, rec["seed"], k + 1) for k, li in enumerate(rec["listens"])]


def _pr(tp, fp, fn):
    p = tp / (tp + fp) if tp + fp else 1.0
    r = tp / (tp + fn) if tp + fn else 1.0
    return p, r


def report(algos, seeds, verbose=True):
    out = {}
    for algo in algos:
        recs = []
        for s in seeds:
            path = _path(algo, s)
            if os.path.exists(path):
                with open(path) as f:
                    recs.append(json.load(f))
        if not recs:
            print(f"[{algo}] no cached results")
            continue
        scs = [x for r in recs for x in score(r)]
        if verbose:
            print(f"\n=== {algo} ===")
            for sc in scs:
                print(f"seed {sc['seed']} listen {sc['listen']} (visited {','.join(sc['visited'])}): dispatch "
                      f"{sc['dispatch']} (voice pile {sc['voice_pile']}); false confident human "
                      f"{sc['false_conf'] or 'none'}; false human (any) {sc['false_hum'] or 'none'}")
                for r in sc["rows"]:
                    ac = "" if r["assoc_conf"] is None else f" {r['assoc_conf']:.2f}"
                    hm = "" if r["human"] is None else f"{r['human']:.2f}"
                    rl = "" if r["rel"] is None else f"{r['rel']:.2f}"
                    print(f"   {r['pile']} {r['truth']:10} -> {r['status']:22} {r['class']:9} hum {hm:4} rel {rl:4} "
                          f"{r['assoc']}{ac}  {r['transcript']}")
        tot = {m: [sum(sc["counts"][m][i] for sc in scs) for i in range(3)] for m in ("detect", "human", "confident")}
        first = [sc for sc in scs if sc["listen"] == 1]
        vt = [sc["voice_top"] for sc in first if sc["voice_top"] is not None]
        out[algo] = {"n": len(recs), "listens": len(scs), "tot": tot,
                     "false_conf": sum(len(sc["false_conf"]) for sc in scs),
                     "false_hum": sum(len(sc["false_hum"]) for sc in scs),
                     "voice_top": (sum(vt), len(vt))}
    if out:
        print(f"\nper-pile assignment over all mission listens (P/R; truth read by the evaluator only)")
        print(f"{'algo':16} {'seeds':>5} {'listens':>7}  {'detect P/R':>11}  {'human P/R':>11}  "
              f"{'confident P/R':>13}  {'false conf':>10}  {'false hum':>9}  {'voice top@1':>11}")
        for algo, o in out.items():
            cells = ["{:.2f}/{:.2f}".format(*_pr(*o["tot"][m])) for m in ("detect", "human", "confident")]
            print(f"{algo:16} {o['n']:>5} {o['listens']:>7}  {cells[0]:>11}  {cells[1]:>11}  {cells[2]:>13}  "
                  f"{o['false_conf']:>10}  {o['false_hum']:>9}  {o['voice_top'][0]}/{o['voice_top'][1]:<9}")
    return out


CLIP_KEYS = ("clip_id", "drone_id", "pile_id", "bearing_deg", "width_deg", "snr_db", "vad_speech_ratio",
             "transcript", "sound_class", "human", "urgency", "reliability", "priority", "doa_agree",
             "association", "association_conf", "ambiguous_with", "notes", "by", "bearings", "heuristics",
             "class_scores", "rationale", "reliability_agent")


def write_fixture(seed: int, path: str, algo=None) -> str:
    """Compact committed fixture of one seed's mission listens (for tests that must not run the
    heavy pipeline). `truth_roles_evaluator_only` is for test assertions, never for the pipeline."""
    algo = algo or algo_version()
    with open(_path(algo, seed)) as f:
        rec = json.load(f)
    out = {"schema": "larpverse.acoustic-fixture/v1", "algo": algo, "seed": seed,
           "truth_roles_evaluator_only": rec["truth_roles"], "listens": []}
    for li in rec["listens"]:
        st = li["state"]
        out["listens"].append({
            "drones": li["drones"], "exclude": li["exclude"], "listen_seed": li["listen_seed"],
            "state": {"schema": st["schema"], "algo": st.get("algo"), "seed": st["seed"],
                      "excluded_piles": st["excluded_piles"], "listening_drones": st["listening_drones"],
                      "clips": [{k: c.get(k) for k in CLIP_KEYS} for c in st["clips"]],
                      "piles": st["piles"], "dispatch": st["dispatch"], "log": st["log"]}})
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        json.dump(out, f, indent=1)
    return path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["run", "report", "all", "fixture"])
    ap.add_argument("--out", default=os.path.join(ROOT, "tests", "fixtures"),
                    help="fixture: directory for acoustic_seed<N>.json")
    ap.add_argument("--seeds", default=",".join(map(str, DEFAULT_SEEDS)))
    ap.add_argument("--n", type=int, default=None, help="use the first N default seeds")
    ap.add_argument("--algos", default=None, help="comma-separated algorithm versions to report "
                                                    "(default: v1-baseline and the current one)")
    ap.add_argument("--force", action="store_true", help="recompute even if cached")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args(argv)
    seeds = [int(s) for s in a.seeds.split(",") if s.strip()]
    if a.n:
        seeds = DEFAULT_SEEDS[: a.n] if a.n <= len(DEFAULT_SEEDS) else list(range(1, a.n + 1))
    if a.cmd in ("run", "all"):
        for s in seeds:
            t = time.time()
            run_seed(s, force=a.force)
            print(f"seed {s}: {algo_version()} done in {time.time() - t:.1f} s", flush=True)
    if a.cmd == "fixture":
        for s in seeds:
            print("wrote", write_fixture(s, os.path.join(a.out, f"acoustic_seed{s}.json")))
    if a.cmd in ("report", "all"):
        algos = a.algos.split(",") if a.algos else list(dict.fromkeys(["v1-baseline", algo_version()]))
        report(algos, seeds, verbose=not a.quiet)


if __name__ == "__main__":
    main()
