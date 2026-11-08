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


HUMAN_CLASSES = ("voice", "shout", "knocking")
HUMAN_FLOOR = 0.45         # fallback human score when YAMNet itself says voice / shout / knocking


def wants_asr(vad, cls, heur):
    """Whisper runs on clips with voice activity, OR a human YAMNet class / human-over-animal frame:
    Silero misses shouted calls (VAD 0.00-0.09 on 6/13 real "help me" clips)."""
    return vad >= 0.15 or cls in ("voice", "shout") or heur.get("human_minus_animal", 0.0) >= models.HMA_MIN


def gate_transcript(text, vad, cls, heur=None):
    """Whisper output is only kept as evidence when the clip looks like speech (R1-003): VAD, or a
    voice/shout class backed by YAMNet's human-over-animal score. (transcribe already drops
    segments Whisper itself doubts: no_speech_prob / avg_logprob.)"""
    if not text:
        return "", None
    hma = (heur or {}).get("human_minus_animal", 0.0)
    shouted = cls in ("voice", "shout") and hma >= models.HMA_MIN
    if vad < 0.3 and not shouted:
        return "", f"transcript dropped: VAD {vad:.2f} too low"
    if cls not in ("voice", "shout") and vad < 0.5:
        return "", f"transcript dropped: {cls} clip with VAD {vad:.2f}"
    return text, None


def recognise(clip, whisper="base.en"):
    """Per-clip recognisers: VAD, YAMNet groups + heuristics, gated Whisper. Returns a dict."""
    vad = models.vad_speech_ratio(clip)
    cls, scores, heur, top = models.classify(clip, vad)
    raw = models.transcribe(clip, whisper) if wants_asr(vad, cls, heur) else ""
    text, why = gate_transcript(raw, vad, cls, heur)
    return {"vad": vad, "cls": cls, "scores": scores, "heur": heur, "top": top, "text": text, "why": why}


ROTOR_STEADY = 0.3        # loudness_steadiness at/above which a clip sounds like a machine / rotor
ROTOR_COMB = 0.25         # blade_pass comb share at/above which a clip carries a rotor's BPF harmonics
ROTOR_HUMAN_CAP = 0.3


def rotor_veto(f, j):
    """Request 0 (docs/ACOUSTIC_VALIDATION.md section 6): a voice/shout/knocking clip that sounds like
    a rotor (steady loudness or a blade-pass comb) and has no independent speech evidence (VAD < 0.3
    and human-minus-animal < 0.4) is not a human lead. ESC-50: helicopter false leads 5/12 -> 1/12,
    false leads 15 -> 7, knock + cough recall unchanged (17/24). Applies to LLM judgments too."""
    heur = f.get("heuristics", {})
    st = heur.get("steadiness", 0.0)
    # a BPF comb only counts on continuous sound: a repeated knock train also has spectral lines
    rotor = st >= ROTOR_STEADY or (heur.get("blade_pass", 0.0) >= ROTOR_COMB and st >= 0.15)
    speech = f["vad_speech_ratio"] >= 0.3 or heur.get("human_minus_animal", 0.0) >= 0.4
    if f["sound_class"] in HUMAN_CLASSES and rotor and not speech and j["human"] > ROTOR_HUMAN_CAP:
        j = dict(j, human=ROTOR_HUMAN_CAP, rationale=j["rationale"] + "; rotor-like (steady / blade-pass): vetoed")
    return j


def class_human_floor(f, j):
    """Human floor from the YAMNet class, then the rotor veto (rotor_veto). The per-clip judgment
    post-processing that validate_real also measures."""
    return rotor_veto(f, _class_floor(f, j))


def _class_floor(f, j):
    """The fallback scorer scales speech evidence by VAD, which collapses under noise; when YAMNet
    already names a human sound class the clip is at least a human lead (docs/ACOUSTIC_VALIDATION.md,
    Requests 1). Ambiguous association still
    caps the clip's priority, so this cannot create a confident lead from leaked sound."""
    if j.get("by") != "fallback" or j["human"] >= HUMAN_FLOOR:
        return j
    cls = f["sound_class"]
    heur = f.get("heuristics", {})
    kp = heur.get("knock_periodicity", 0.0)
    # each path needs a little more than the bare class: rotor noise and engines fire "knocking"
    # (so a knock pattern is required), barks under noise can read as "voice" (so no dog score).
    # Real audio, agent F1 eb8cdba -> now: ESC-50 0.46 -> 0.70, teammate clips 0.83 -> 0.81,
    # +rotor 0 dB: ESC 0.24 -> 0.28, teammate 0.64 -> 0.69, DroneAudioSet 0.49 -> 0.58.
    voiced = cls in ("voice", "shout") and f["class_scores"].get("dog", 0.0) < 0.1
    knocked = cls == "knocking" and kp >= 0.2
    if voiced or knocked:
        j = dict(j, human=HUMAN_FLOOR, rationale=j["rationale"] + f"; YAMNet class {cls}: human floor")
        if cls == "knocking":
            j["urgency"] = max(j["urgency"], round(0.55 * (0.4 + 0.6 * HUMAN_FLOOR), 2))
    return j


def _bigrams(t):
    """Word bigrams of a transcript: two talkers' scripts share single words ("anyone"), a leak
    repeats phrases ("my leg", "leg is")."""
    w = re.findall(r"[a-z']+", (t or "").lower())
    return set(zip(w, w[1:]))


def clip_features(cid, det, did, snr, clip, n_drones, whisper, notes=()):
    r = recognise(clip, whisper)
    vad, cls, scores, heur, top, text, why = (r[k] for k in ("vad", "cls", "scores", "heur", "top", "text", "why"))
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
            s["blocks"] = [s["blocks"][j] for j in keep]
            s["notes"].append(f"suppressed cross-talk from {src} ({len(drop)} blocks)")
        else:
            s["xtalk_only"] = True
            s["notes"].append(f"content is cross-talk from {src}")
        log.append(f"{s['det']['pile_id']}: suppressed cross-talk from {src} ({len(drop)}/{len(s['blocks'])} blocks)")
    log += _knock_crosstalk(seps, blk)
    return log


def _abs_onsets(s, blk):
    """Knock onsets of a clip mapped back to listen time (s), with their clip block index."""
    t = models.onset_times(s["clip"])
    if t is None:
        return []
    bs = dsp.BLOCK_S
    out = []
    for x in t:
        j = int(x // bs)
        if j < len(s["blocks"]):
            out.append((s["blocks"][j] * bs + (x - j * bs), j))
    return out


def _knock_crosstalk(seps, blk, tol=0.08):
    """The same knocks heard in two clips belong to the cleaner knocker (higher knock periodicity).

    A clip whose knock onsets mostly coincide (+-80 ms, the arrival spread across drones) with a
    stronger knocker's onsets loses those blocks, so a settling or dog pile next to a knocking victim
    does not inherit its pattern."""
    log = []
    for s in seps:
        s["kp"] = models.knock_periodicity(s["clip"])
        s["onsets"] = _abs_onsets(s, blk)
    for s in seps:
        if s["det"]["excluded"] or s["kp"] < 0.3 or len(s["onsets"]) < 3:
            continue
        for o in seps:
            if o is s or o["kp"] < 0.5 or o["kp"] < s["kp"] or not o["onsets"]:
                continue
            if o["kp"] == s["kp"] and o["snr"] <= s["snr"]:
                continue
            ot = np.array([x for x, _ in o["onsets"]])
            hit = [j for x, j in s["onsets"] if np.min(np.abs(ot - x)) <= tol]
            if len(hit) < 0.6 * len(s["onsets"]):
                continue
            drop = set(hit)
            keep = [j for j in range(len(s["blocks"])) if j not in drop]
            src = o["det"]["pile_id"]
            if len(keep) >= 2:
                s["clip"] = np.concatenate([s["clip"][j * blk:(j + 1) * blk] for j in keep])
                s["blocks"] = [s["blocks"][j] for j in keep]
                s["notes"].append(f"suppressed knock cross-talk from {src} ({len(drop)} blocks)")
            else:
                s["xtalk_only"] = True
                s["notes"].append(f"knocks are cross-talk from {src}")
            log.append(f"{s['det']['pile_id']}: suppressed knock cross-talk from {src}")
            break
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


def analyse(scene, exclude_piles=(), mode="auto", whisper="base.en", save_clips=False, timing=None,
            gemini_audio=None, rotors=None):
    """Everything downstream of the microphones. Sees only the Scene (public geometry + signals).
    rotors: {id: (x, y)} of our own drones hovering during the listen (their bearings are skipped)."""
    T = timing if timing is not None else {}
    t = time.time()
    dets, B = dsp.localise_piles(scene, exclude=set(exclude_piles), rotors=rotors)
    rotor_dets = [d for d in dets if d.get("rotor")]
    dets = [d for d in dets if not d.get("rotor")]
    log, kept = [], []
    if rotor_dets:
        log.append("skipped bearings toward hovering drones " + ", ".join(d["pile_id"][6:] for d in rotor_dets))
    explainers = [d for d in dets if d["excluded"]] + rotor_dets   # null them, no clips
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
        if d.get("rotor"):
            continue                                    # nulled in every beam, never separated
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
    # backstop: the same phrases heard on two piles belong to the clip with more speech
    for f in feats:
        w = _bigrams(f["transcript"])
        if not w:
            continue
        rivals = [(p, t_, None) for p, t_ in ref_text.items()] + \
                 [(g["pile_id"], g["transcript"], g) for g in feats if g is not f]
        for p, t_, g in rivals:
            shared = len(w & _bigrams(t_))
            if shared >= min(2, len(w)) and (g is None or g["vad_speech_ratio"] > f["vad_speech_ratio"]):
                f["notes"].append(f"transcript repeats {p}'s talker: dropped as cross-talk")
                log.append(f"{f['pile_id']}: suppressed cross-talk from {p} (transcript)")
                f["transcript"] = ""
                f["_ambiguous"] = True
                break
    for f in feats:     # sparse speech that sounds like a voice to YAMNet: likely another talker's leak
        if f["sound_class"] in ("voice", "shout") and f["vad_speech_ratio"] < 0.3:
            f["notes"].append(f"sparse speech (VAD {f['vad_speech_ratio']:.2f}): direction of the voice not pinned")
            f["_ambiguous"] = True
    for f in feats:
        if f.pop("_ambiguous", False):
            d = next(s_["det"] for s_ in seps if s_["det"]["pile_id"] == f["pile_id"])
            d["association"], d["assigned"] = "ambiguous", False
            f["doa"].update(association="ambiguous", assigned=False)
    T["recognise"] = time.time() - t

    t = time.time()
    judg, meta = agent.judge_clips(feats, mode=mode)
    dets_by_pile = {d["pile_id"]: d for d in kept}
    clips = []
    for f in feats:
        j = class_human_floor(f, judg[f["clip_id"]])
        c = {k: f[k] for k in ("clip_id", "drone_id", "pile_id", "bearing_deg", "width_deg", "snr_db",
                               "vad_speech_ratio", "transcript", "sound_class", "class_scores", "heuristics")}
        c.update(human=j["human"], urgency=j["urgency"], reliability=j["reliability"],
                 rationale=j["rationale"], by=j["by"])
        if j.get("by_note"):
            c["by_note"] = j["by_note"]
        c["doa_agree"] = f"{f['doa']['drones_agreeing']}/{f['doa']['drones_listening']}"
        det = dets_by_pile[f["pile_id"]]
        c["bearings"] = [{"drone_id": d, "bearing_deg": v["bearing_deg"], "sigma_deg": v["sigma_deg"],
                          "snr_db": det.get("snr_by_drone", {}).get(d), "measured": v["measured"]}
                         for d, v in det["per_drone"].items()]
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

    state = {
        "schema": "larpverse.acoustic/v1",
        "algo": dsp.ALGO_VERSION,
        "t": scene.duration,
        "seed": scene.seed, "start_pile": scene.start_pile, "synthetic": True,
        "excluded_piles": sorted(exclude_piles),
        "listening_drones": [{"id": k, "xy": [round(float(v[0]), 2), round(float(v[1]), 2)]}
                             for k, v in scene.drones.items()],
        "clips": clips, "piles": piles, "dispatch": dispatch,
        "agent": meta, "log": log, "timing_s": {},
    }
    if gemini_audio_enabled(gemini_audio, mode):
        t = time.time()
        sep_by = {s_["det"]["pile_id"]: s_ for s_ in seps}
        items = [{"clip_id": c["clip_id"], "pile_id": c["pile_id"], "drone_id": c["drone_id"],
                  "audio": sep_by[c["pile_id"]]["clip"],
                  "background": scene.signals[c["drone_id"]][0].astype(np.float32),
                  "reliability": c["reliability"], "association": c["association"]} for c in clips]
        add_gemini_audio(state, items, mode)
        T["gemini_audio"] = time.time() - t
    state["timing_s"] = {k: round(v, 2) for k, v in T.items()}
    return state


def gemini_audio_enabled(flag, mode) -> bool:
    """Stage 1 novelty -> stage 2 Gemini audio -> stage 3 sector ranking (acoustic/gemini_audio.py).

    flag: True/False forces it; None = env LARPVERSE_GEMINI_AUDIO (1/0) if set, else ON exactly when
    agents.llm resolves to Gemini with a key (or LLM_REPLAY), and never with mode="fallback"."""
    if flag is not None:
        return bool(flag)
    env = os.environ.get("LARPVERSE_GEMINI_AUDIO", "").strip().lower()
    if env in ("1", "true", "yes", "on"):
        return True
    if env in ("0", "false", "no", "off") or mode == "fallback":
        return False
    try:
        from agents import llm
        return llm.resolve()[0] == "gemini" and llm.available()
    except Exception:                                   # noqa: BLE001
        return False


def add_gemini_audio(state, items, mode="auto"):
    """Run the novelty -> Gemini audio stage on the separated clips and merge it into state (in place).

    Gemini answers are cached/replayed by agents.llm; without a key, or on any failure, gemini_audio
    falls back to local analysis per clip, and if the whole stage fails the v1 state is left as it
    was with the error in state["gemini_audio"]. Association still bounds the result: an ambiguous
    pile keeps its priority cap, so stage 3 cannot dispatch on a direction we could not pin down."""
    from . import gemini_audio
    gmode = {"llm": "live", "replay": "replay", "fallback": "fallback"}.get(mode, "auto")
    try:
        res = gemini_audio.analyse_sector_audio(items, mode=gmode)
        gemini_audio.merge_into_state(state, res)
    except Exception as e:                              # noqa: BLE001
        state["gemini_audio"] = {"mode": gmode, "error": f"{type(e).__name__}: {e}"[:300]}
        state["log"].append(f"gemini audio stage failed ({type(e).__name__}); v1 ranking kept")
        return state
    amb = {c["pile_id"] for c in state["clips"] if c.get("association") != "confident"}
    changed = False
    for p in state["piles"]:
        if p["pile_id"] in amb and p["priority"] > AMBIGUOUS_PRIORITY_CAP:
            p["priority"], changed = AMBIGUOUS_PRIORITY_CAP, True
    if changed:
        state["piles"].sort(key=lambda p: (-p["priority"], p["pile_id"]))
        leads = [p for p in state["piles"] if p["status"] in ("lead", "radar_ping")]
        for p in leads:
            p["status"] = "lead"
        if leads:
            leads[0]["status"] = "radar_ping"
            if state["dispatch"].get("pile_id") != leads[0]["pile_id"]:
                state["dispatch"] = {"pile_id": leads[0]["pile_id"],
                                     "reason": f"Send the drones to {leads[0]['pile_id']}: highest priority "
                                               f"({leads[0]['priority']}). {leads[0]['why']}"}
    return state


def listen_and_rank(scenario, drone_positions: dict | None = None, exclude_piles=(), seed: int | None = None,
                    mode="auto", whisper="base.en", return_truth=False, gemini_audio=None, rotors=None):
    """Mission hook: listen from the landed drones and rank the piles. Returns the
    larpverse.acoustic/v1 dict (writes nothing). Only the scene synthesis reads scenario.truth;
    pass return_truth=True to also get the evaluation summary as a second value. rotors: {id: (x, y)}
    of our own drones hovering during the listen (mission state); their bearings are skipped."""
    T = {}
    t0 = time.time()
    scene, truth = build_scene(scenario, drone_positions, seed, rotors=rotors)
    T["scene"] = time.time() - t0
    state = analyse(scene, exclude_piles, mode, whisper, save_clips=False, timing=T, gemini_audio=gemini_audio,
                    rotors=rotors)
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
    ap.add_argument("--rotor-xy", default=None, help='JSON {"d4": [x, y], ...} of our drones hovering during the listen')
    ap.add_argument("--agent", default="auto", choices=["auto", "llm", "replay", "fallback"])
    ap.add_argument("--gemini-audio", default="auto", choices=["auto", "on", "off"],
                    help="novelty -> Gemini audio stage (auto: on when a Gemini key is configured)")
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
    rotors = {k: tuple(v) for k, v in json.loads(a.rotor_xy).items()} if a.rotor_xy else None
    scene, truth = build_scene(sc, drones, a.seed, rotors=rotors)
    T["scene"] = time.time() - t0
    state = analyse(scene, excl, a.agent, a.whisper, save_clips=True, timing=T,
                    gemini_audio={"auto": None, "on": True, "off": False}[a.gemini_audio], rotors=rotors)
    state["timing_s"]["total"] = round(time.time() - t0, 2)
    path = write_state(state, a.out)
    print_report(state, None if a.no_truth else truth)
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
