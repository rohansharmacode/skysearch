"""Acoustic pipeline: scenario -> synthesised arrays -> DoA -> separate -> VAD/class/ASR -> agent.

CLI (writes dashboard/data/acoustic.json):
    python -m acoustic.run --seed 3 --start-pile B2 [--drone-xy '{"d1":[x,y],...}'] [--exclude B2,A1]
                           [--agent auto|llm|replay|fallback] [--whisper base.en]
Library (no file written):
    from acoustic.run import listen_and_rank
    state = listen_and_rank(scenario, {"d1": (x, y), ...}, exclude_piles=["B2"], seed=3)
"""
from __future__ import annotations

import os

os.environ.setdefault("OMP_NUM_THREADS", "2")      # laptop-friendly; set before numpy loads

import argparse  # noqa: E402
import json  # noqa: E402
import re  # noqa: E402
import sys  # noqa: E402
import time  # noqa: E402

import numpy as np  # noqa: E402

from . import agent, dsp, models  # noqa: E402
from .scene import DATA_DIR, build_scene  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DEFAULT_OUT = os.path.join(ROOT, "dashboard", "data")
MIN_SNR_DB = 2.0
AMBIGUOUS_PRIORITY_CAP = 19    # one below sim.grid_mission.PRIORITY_MIN (20): ambiguous leads never dispatch alone
PILE_NAMES = [f"{c}{r}" for r in (1, 2, 3) for c in "ABC"]


SPEECH_BLOCK = 0.3        # a clip block "has speech" when this share of its VAD frames is speech
XTALK_MARGIN = 0.15       # another clip owns that speech when its VAD in the same block is this much higher


def gate_transcript(text, vad, cls):
    """Whisper output is only kept as evidence when the clip looks like speech (R1-003)."""
    if not text:
        return "", None
    if vad < 0.3:
        return "", f"transcript dropped: VAD {vad:.2f} too low"
    if cls not in ("voice", "shout") and vad < 0.5:
        return "", f"transcript dropped: {cls} clip with VAD {vad:.2f}"
    return text, None


def _words(t):
    return {w for w in re.findall(r"[a-z']+", (t or "").lower()) if len(w) > 2}


def clip_features(cid, det, did, snr, clip, n_drones, whisper, notes=()):
    vad = models.vad_speech_ratio(clip)
    cls, scores, heur, top = models.classify(clip, vad)
    raw = models.transcribe(clip, whisper) if vad >= 0.15 else ""
    text, why = gate_transcript(raw, vad, cls)
    pd = det["per_drone"][did]
    return {
        "clip_id": cid, "pile_id": det["pile_id"], "drone_id": did,
        "duration_s": round(len(clip) / 16000, 1), "continuous": det["continuous"],
        "bearing_deg": pd["bearing_deg"], "width_deg": pd["width_deg"], "snr_db": snr,
        "vad_speech_ratio": vad, "transcript": text, "sound_class": cls, "class_scores": scores,
        "heuristics": heur, "yamnet_top": [[lbl, sc] for lbl, sc in top],
        "doa": {"drones_agreeing": len(det["drones_consistent"]), "drones_listening": n_drones,
                "tri_err_m": det["tri_err_m"], "tri_resid_m": det["tri_resid_m"],
                "assigned": det["assigned"], "association": det["association"],
                "association_conf": det["assoc_conf"], "ambiguous_with": det["ambiguous_with"],
                "overlap_frac": det.get("dominated_frac", 0.0)},
        "notes": [n for n in list(notes) + [why] if n],
    }


def _talker(s):
    """A separated source is a talker when speech fills a good share of its clip's blocks."""
    sp = s["speech"][: len(s["blocks"])]
    return len(sp) > 0 and float(np.mean(sp >= SPEECH_BLOCK)) >= 0.4


def remove_crosstalk(seps, blk):
    """Speech that another separated source owns is cut out of a clip (R1-002).

    A block of clip c with speech is cross-talk from source o (kept, or an excluded pile's) when
    o is a talker associated with that time (its active blocks, +-1 block) and either c is not a
    talker itself, or o's clip covers the block with clearly more speech. The 0.3 m array cannot
    null a talker well below ~1 kHz, so a dog or debris beam still carries a nearby voice. Owned
    blocks are removed (if >= 2 blocks stay); otherwise the content cannot be pinned on this pile."""
    log = []
    for s in seps:
        act = s["det"]["active_blocks"]
        s["near"] = act | np.r_[act[1:], False] | np.r_[False, act[:-1]]
        s["talker"] = _talker(s)
    for s in seps:
        if s["det"]["excluded"]:
            continue
        drop, owners = [], set()
        for j, b in enumerate(s["blocks"]):
            # a non-talker (dog, debris, engine) gives up any speech that coincides with a talker
            if j >= len(s["speech"]) or s["speech"][j] < (SPEECH_BLOCK if s["talker"] else 0.1):
                continue
            for o in seps:
                if o is s or not o["talker"] or o["det"].get("continuous") or not o["near"][b]:
                    continue
                if b in o["pos"]:
                    owns = o["speech"][o["pos"][b]] >= s["speech"][j] + XTALK_MARGIN or not s["talker"]
                else:
                    owns = not s["talker"]
                if owns:
                    drop.append(j)
                    owners.add(o["det"]["pile_id"])
                    break
        if not drop:
            continue
        keep = [j for j in range(len(s["blocks"])) if j not in set(drop)]
        src = ", ".join(sorted(owners))
        if len(keep) >= 2:
            s["clip"] = np.concatenate([s["clip"][j * blk:(j + 1) * blk] for j in keep])
            s["notes"].append(f"suppressed cross-talk from {src} ({len(drop)} blocks)")
        else:
            s["xtalk_only"] = True
            s["notes"].append(f"content is cross-talk from {src}")
        log.append(f"{s['det']['pile_id']}: suppressed cross-talk from {src} ({len(drop)}/{len(s['blocks'])} blocks)")
    return log


def finalise_association(det, dominated):
    """Fold the clip-content check into the detection's association (in place).

    A clip whose blocks are mostly shared with a more strongly associated source may carry
    that source's speech or knocks: its content cannot be pinned on this pile."""
    det["dominated_frac"] = dominated
    det["assoc_conf"] = round(det["assoc_conf"] * (1.0 - 0.5 * dominated), 3)
    if dominated > 0.5 or det["assoc_conf"] < dsp.CONF_MIN:
        det["association"] = "ambiguous"
    det["assigned"] = det["association"] == "confident"


def apply_association(clips, dets_by_pile):
    """Association confidence feeds reliability (and so priority); ambiguous human leads are
    reported as "Lead (ambiguous direction)" instead of confident sources."""
    for c in clips:
        det = dets_by_pile[c["pile_id"]]
        c["association"] = det["association"]
        c["association_conf"] = det["assoc_conf"]
        c["ambiguous_with"] = det["ambiguous_with"]
        c["reliability_agent"] = c["reliability"]
        c["reliability"] = round(c["reliability"] * (0.5 + 0.5 * det["assoc_conf"]), 2)
        c["priority"] = agent.clip_priority(c)
        if det["association"] != "confident":
            c["reliability"] = min(c["reliability"], 0.45)
            # listed as a lead, but below the mission's dispatch threshold until a listen from
            # elsewhere pins the direction down
            c["priority"] = min(agent.clip_priority(c), AMBIGUOUS_PRIORITY_CAP)
    return clips


def analyse(scene, exclude_piles=(), mode="auto", whisper="base.en", save_clips=False, timing=None):
    """Everything downstream of the microphones. Sees only the Scene (public geometry + signals)."""
    T = timing if timing is not None else {}
    t = time.time()
    dets, B = dsp.localise_piles(scene, exclude=set(exclude_piles))
    log, kept = [], []
    explainers = [d for d in dets if d["excluded"]]     # visited piles' sources: null them, no clips
    for d in dets:
        if d["excluded"]:
            continue
        snr0 = dsp.extract_clip(scene, d, B, explainers)[1]
        if snr0 >= MIN_SNR_DB and d["posterior"] >= dsp.POST_MIN:
            kept.append(d)
        else:
            log.append(f"rejected {d['pile_id']} hypothesis: bearings fit {d['best_pile']} better "
                       f"(posterior {d['posterior']:.2f}), tri_err={d['tri_err_m']} m, SNR {snr0} dB")
    T["doa"] = time.time() - t

    t = time.time()
    blk = int(dsp.BLOCK_S * scene.fs)
    seps = []
    pool = kept + explainers
    for d in pool:
        did, snr, clip, dom, idx = dsp.extract_clip(scene, d, B, [o for o in pool if o is not d])
        if not d["excluded"]:
            finalise_association(d, dom)
        seps.append({"det": d, "did": did, "snr": snr, "clip": clip, "blocks": list(idx),
                     "pos": {int(b): j for j, b in enumerate(idx)},
                     "speech": models.block_speech(clip, blk), "notes": []})
    log += remove_crosstalk(seps, blk)
    T["separate"] = time.time() - t

    t = time.time()
    feats = []
    ref_text = {}                                       # transcripts of excluded piles' talkers
    for s_ in seps:
        if s_["det"]["excluded"] and np.mean(s_["speech"] >= SPEECH_BLOCK) >= 0.2:
            ref_text[s_["det"]["pile_id"]] = models.transcribe(s_["clip"], whisper)
    for s_ in seps:
        d = s_["det"]
        if d["excluded"]:
            continue
        f = clip_features(f"ac-{d['pile_id']}", d, s_["did"], s_["snr"], s_["clip"], len(scene.drones),
                          whisper, s_["notes"])
        if s_.get("xtalk_only"):
            f["transcript"] = ""
            d["association"], d["assigned"] = "ambiguous", False
            f["doa"].update(association="ambiguous", assigned=False)
        feats.append(f)
        if save_clips:
            import soundfile as sf
            cd = os.path.join(DATA_DIR, "clips")
            os.makedirs(cd, exist_ok=True)
            sf.write(os.path.join(cd, f"seed{scene.seed}_{d['pile_id']}.wav"),
                     s_["clip"] / (np.abs(s_["clip"]).max() + 1e-9) * 0.8, 16000)
    # backstop: the same words heard on two piles belong to the clip with more speech
    for f in feats:
        w = _words(f["transcript"])
        if len(w) < 2:
            continue
        rivals = [(p, t_, None) for p, t_ in ref_text.items()] + \
                 [(g["pile_id"], g["transcript"], g) for g in feats if g is not f]
        for p, t_, g in rivals:
            ov = len(w & _words(t_)) / len(w)
            if ov >= 0.5 and (g is None or g["vad_speech_ratio"] > f["vad_speech_ratio"]):
                f["notes"].append(f"transcript repeats {p}'s talker: dropped as cross-talk")
                log.append(f"{f['pile_id']}: suppressed cross-talk from {p} (transcript)")
                f["transcript"] = ""
                break
    T["recognise"] = time.time() - t

    t = time.time()
    judg, meta = agent.judge_clips(feats, mode=mode)
    clips = []
    for f in feats:
        j = judg[f["clip_id"]]
        c = {k: f[k] for k in ("clip_id", "drone_id", "pile_id", "bearing_deg", "width_deg", "snr_db",
                               "vad_speech_ratio", "transcript", "sound_class", "class_scores", "heuristics")}
        c.update(human=j["human"], urgency=j["urgency"], reliability=j["reliability"],
                 rationale=j["rationale"], by=j["by"])
        c["doa_agree"] = f"{f['doa']['drones_agreeing']}/{f['doa']['drones_listening']}"
        clips.append(c)
    for c, f in zip(clips, feats):
        c["notes"] = f["notes"]
    apply_association(clips, {d["pile_id"]: d for d in kept})
    piles_xy = {p: v for p, v in scene.piles.items() if p not in set(exclude_piles)}
    piles, dispatch = agent.aggregate(clips, piles_xy, scene.start_pile)
    clip_by = {c["clip_id"]: c for c in clips}
    for p in piles:                                     # association onto the pile list
        c = clip_by.get(p["top_clip"])
        if c is None:
            continue
        p["association"] = c["association"]
        if c["association"] != "confident" and p["status"] in ("lead", "radar_ping"):
            alt = ", ".join(c["ambiguous_with"])
            p["why"] = ("Lead (ambiguous direction" + (f"; could be {alt}" if alt else "") + "): " + p["why"])
    T["agent"] = time.time() - t

    return {
        "schema": "larpverse.acoustic/v1",
        "algo": dsp.ALGO_VERSION,
        "t": scene.duration,
        "seed": scene.seed, "start_pile": scene.start_pile, "synthetic": True,
        "excluded_piles": sorted(exclude_piles),
        "listening_drones": [{"id": k, "xy": [round(float(v[0]), 2), round(float(v[1]), 2)]}
                             for k, v in scene.drones.items()],
        "clips": clips, "piles": piles, "dispatch": dispatch,
        "agent": meta, "log": log, "timing_s": {k: round(v, 2) for k, v in T.items()},
    }


def listen_and_rank(scenario, drone_positions: dict | None = None, exclude_piles=(), seed: int | None = None,
                    mode="auto", whisper="base.en", return_truth=False):
    """Mission hook: listen from the landed drones and rank the piles. Returns the
    larpverse.acoustic/v1 dict (writes nothing). Only the scene synthesis reads scenario.truth;
    pass return_truth=True to also get the evaluation summary as a second value."""
    T = {}
    t0 = time.time()
    scene, truth = build_scene(scenario, drone_positions, seed)
    T["scene"] = time.time() - t0
    state = analyse(scene, exclude_piles, mode, whisper, save_clips=False, timing=T)
    state["timing_s"]["total"] = round(time.time() - t0, 2)
    return (state, truth) if return_truth else state


def write_state(state, out_dir=DEFAULT_OUT):
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "acoustic.json")
    with open(path + ".tmp", "w") as fh:
        json.dump(state, fh, indent=1)
    os.replace(path + ".tmp", path)
    return path


def print_report(state, truth=None):
    clip_by = {c["clip_id"]: c for c in state["clips"]}
    print(f"\nseed {state['seed']}  start pile {state['start_pile']}  drones "
          + ", ".join(f"{d['id']}@({d['xy'][0]:.1f},{d['xy'][1]:.1f})" for d in state["listening_drones"]))
    hdr = (f"{'pile':4} {'pri':>3}  {'status':22} {'class':9} {'hum':>4} {'urg':>4} {'rel':>4} "
           f"{'brg':>5} {'snr':>5} {'by':8} transcript")
    print(hdr)
    print("-" * len(hdr))
    for p in state["piles"]:
        c = clip_by.get(p["top_clip"])
        if c:
            tr = (c["transcript"][:58] + "…") if len(c["transcript"]) > 59 else c["transcript"]
            print(f"{p['pile_id']:4} {p['priority']:>3}  {p['status']:22} {c['sound_class']:9} {c['human']:4.2f} "
                  f"{c['urgency']:4.2f} {c['reliability']:4.2f} {c['bearing_deg']:5.0f} {c['snr_db']:5.1f} "
                  f"{c['by']:8} {tr}")
        else:
            print(f"{p['pile_id']:4} {p['priority']:>3}  {p['status']:22} {'-':9}")
    d = state["dispatch"]
    print(f"\nDISPATCH -> {d['pile_id']}: {d['reason']}")
    m = state["agent"]
    print(f"agent: mode={m['mode']} llm_calls={m['llm_calls']} cache_hits={m['cache_hits']} "
          f"fallback={m['fallback']}" + (f" error={m['error']}" if m["error"] else ""))
    print("timing (s): " + "  ".join(f"{k} {v:.1f}" for k, v in state["timing_s"].items()))
    if truth is not None:   # evaluation only; never passed to the analysis or agent
        found = {c["pile_id"] for c in state["clips"]}
        hit = sum(p in found for p in truth)
        false = sorted(found - set(truth))
        print(f"eval (truth, not seen by agent): {hit}/{len(truth)} sources found, false piles "
              f"{false or 'none'}; " + ", ".join(f"{p}={v['kind']}" for p, v in sorted(truth.items())))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--start-pile", default=None, choices=PILE_NAMES,
                    help="scenario start pile (default: the scenario's seeded choice)")
    ap.add_argument("--drone-xy", default=None, help='JSON {"d1": [x, y], ...} of landed drones')
    ap.add_argument("--exclude", default="", help="comma-separated piles to leave out of the ranking")
    ap.add_argument("--agent", default="auto", choices=["auto", "llm", "replay", "fallback"])
    ap.add_argument("--whisper", default="base.en", help="faster-whisper model (tiny.en, base.en)")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--no-truth", action="store_true", help="hide the evaluation line")
    a = ap.parse_args(argv)

    if ROOT not in sys.path:
        sys.path.insert(0, ROOT)
    from sim.scenario import make_scenario
    sc = make_scenario(a.seed, a.start_pile)
    drones = {k: tuple(v) for k, v in json.loads(a.drone_xy).items()} if a.drone_xy else None
    excl = [p.strip().upper() for p in a.exclude.split(",") if p.strip()]

    T = {}
    t0 = time.time()
    scene, truth = build_scene(sc, drones, a.seed)
    T["scene"] = time.time() - t0
    state = analyse(scene, excl, a.agent, a.whisper, save_clips=True, timing=T)
    state["timing_s"]["total"] = round(time.time() - t0, 2)
    path = write_state(state, a.out)
    print_report(state, None if a.no_truth else truth)
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
