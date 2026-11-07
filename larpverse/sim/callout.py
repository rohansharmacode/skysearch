"""Call-out and listen with the drones' loudspeaker (USAR practice), plus reassurance and guidance broadcasts.

Protocol per listen phase (drones landed, see ``plan_callout``):
  (a) silence: rotors are already off; the passive acoustic listen that precedes the call-out doubles as
      the quiet baseline (background rate of human-like sounds per sector). Ground crews are asked by
      radio to stop work (logged, not broadcast).
  (b) message: one drone (nearest the target) plays the fixed multilingual call-out (sim/guidance.MESSAGES,
      default order zh-TW, nan-TW, en, id, vi) with a short gap after each language;
  (c) listen: ``LISTEN_WINDOW_S`` s with every drone's microphones. Human-like sounds that start after a
      message segment (a call-and-response timing reference) and match a sector's bearings are evidence
      that someone there responds to voice (AVPU "V"); a knock pattern of about three is the instructed
      reply. No response does NOT rule out an unconscious person (or someone who cannot hear or move).
Then: reassurance ("Help is coming ...") towards each ping / responding sector, and spoken route guidance
to ambulatory people heard on open ground (or radar Doppler tracks), all logged in ``broadcasts``.

Ground-truth boundary (same split as sim/radar_dsp.py):
  PART 1  ``simulate_callout`` is an OBSERVATION GENERATOR. It reads ``scenario.truth`` (victims, their
          hidden physio state and depth, dogs, settling debris) and synthesises what the microphones
          measure: per-drone detections {t0, t1, class, class_conf, n_pulses, pulse_period_s, bearing_deg,
          snr_db}. It also adds synthetic walking-wounded bystanders (``walking_wounded``).
  PART 2  ``analyse_callout`` and everything after it see detections + public geometry only.

Numbers are labelled assumptions, not measurements: loudspeaker 110 dB SPL at 1 m (a small PA horn on a
heavy-lift drone), shout ~88 dB / knock on concrete ~80 dB / weak voice ~68 dB at 1 m, rubble loss 3 dB per
metre of burial (as acoustic/scene.py) plus 0-8 dB of path variability, site ambient 45 dB with rotors off.
"""
from __future__ import annotations

import math
import os

import numpy as np

from .guidance import (GUIDANCE, LANGS, MESSAGES, BroadcastLog, audio_key, cost_map, est_duration,
                       guidance_route, guidance_texts)

LISTEN_WINDOW_S = 20.0
GAP_S = 2.5                  # listen gap after each language segment (responses heard here too)
BASELINE_S = 24.0            # last part of the passive listen used as the quiet baseline
SPEAKER_DB_1M = 110.0
HEAR_DB = 55.0               # message level at the person needed to understand it (assumption)
AMBIENT_DB = 45.0
DET_SNR_DB = 6.0
RUBBLE_DB_PER_M = 3.0
EARSHOT_M = 40.0             # sectors farther than this from the speaker drone are not reported
HUMAN = ("voice", "shout", "knocking", "crying")
RESP_MAX_LATENCY_S = 10.0    # an onset this soon after a message segment counts as time-locked
SECTOR_RADIUS_M = 9.0        # a source farther than this from every pile centre is on open ground

# Hidden response model (PART 1 only): state -> (P respond | heard, modes [(kind, weight)], latency range s
# after the first message segment the person understands, P stop after each bout). People keep knocking /
# calling every 3-6 s until they stop, so a bout lost under the loudspeaker is usually followed by another.
RESPONSE_MODEL = {
    "calm":                 (0.90, [("knock3", 0.5), ("shout", 0.5)], (1.0, 4.0), 0.10),
    "distressed_tachypnea": (0.60, [("knock_fast", 0.5), ("weak_voice", 0.5)], (2.0, 8.0), 0.35),
    "unconscious_slow":     (0.00, [], (0, 0), 1.0),
    "irregular_apneic":     (0.00, [], (0, 0), 1.0),
    "child_calm":           (0.75, [("crying", 0.6), ("shout", 0.4)], (1.0, 6.0), 0.15),
    "ambulatory":           (0.90, [("shout", 1.0)], (1.0, 3.0), 0.20),
}
LEVEL_DB = {"knock3": 88.0, "knock_fast": 84.0, "shout": 90.0, "weak_voice": 72.0, "crying": 85.0,
            "bark": 90.0, "debris": 80.0, "voice": 82.0, "knock": 86.0}
OBS_CLASS = {"knock3": "knocking", "knock_fast": "knocking", "shout": "shout", "weak_voice": "voice",
             "crying": "crying", "bark": "dog", "debris": "debris", "voice": "voice", "knock": "knocking"}
LANG_PRIOR = (("zh-TW", 0.45), ("nan-TW", 0.35), ("id", 0.08), ("vi", 0.08), ("en", 0.04))   # Tainan residents
NO_VOLUNTARY = ("unconscious_slow", "irregular_apneic")
ASSOC_GATE_DEG = 12.0        # bearing error (beyond the pile's extent) to assign an event to a sector
ASSOC_MARGIN_DEG = 4.0       # ... and the next-best sector must be this much worse
UNDERSTANDS = {"zh-TW": {"zh-TW", "nan-TW"}, "nan-TW": {"zh-TW", "nan-TW"}, "en": {"en"}, "id": {"id"}, "vi": {"vi"}}


# ============================================================================ protocol
def plan_callout(t_start, drones, target, langs=LANGS, durations=None, listen_s=LISTEN_WINDOW_S):
    """Timeline of one call-out. ``drones``: {id: (x, y)} landed; ``target``: {"type", "id", "xy"}.
    ``durations``: {lang: seconds} of the rendered call-out audio (acoustic/speaker_assets manifest)."""
    txy = target["xy"]
    spk = min(drones, key=lambda d: (math.hypot(drones[d][0] - txy[0], drones[d][1] - txy[1]), d))
    segs, t = [], float(t_start)
    for lang in langs:
        d = float((durations or {}).get(lang) or est_duration(MESSAGES["callout"][lang], lang))
        segs.append({"lang": lang, "t0": round(t, 2), "t1": round(t + d, 2)})
        t += d + GAP_S
    listen = (round(segs[-1]["t1"], 2), round(segs[-1]["t1"] + listen_s, 2))
    return {"t_start": round(float(t_start), 2), "speaker_drone": spk,
            "drones": {k: [round(float(v[0]), 2), round(float(v[1]), 2)] for k, v in drones.items()},
            "target": dict(target, xy=[round(float(txy[0]), 2), round(float(txy[1]), 2)]),
            "baseline": (round(float(t_start) - BASELINE_S, 2), round(float(t_start), 2)),
            "segments": segs, "listen": listen, "t_end": listen[1], "langs": list(langs)}


def _masked(plan, t):
    return any(s["t0"] - 0.1 <= t <= s["t1"] + 0.2 for s in plan["segments"])


# ============================================================================ PART 1: generator (TRUTH)
def walking_wounded(scenario, seed):
    """HIDDEN synthetic bystanders on open ground (0-2 per seed; at least one for seed 7, the demo).
    Generator-only, like Scenario.truth; separate RNG so it never changes the rest of the scenario."""
    if getattr(scenario, "bystanders", True) is False:
        return []
    rng = np.random.default_rng([int(seed), 31])
    n = int(rng.integers(0, 3)) if int(seed) != 7 else 1
    piles = [(p.x, p.y) for p in scenario.piles]
    x0, y0, x1, y1 = getattr(scenario, "bounds", None) or (-36.0, -54.0, 36.0, 36.0)
    out = []
    for k in range(200):
        if len(out) >= n:
            break
        x, y = rng.uniform(x0 + 4, x1 - 4), rng.uniform(y0 + 12, y1 - 4)
        if min(math.hypot(x - a, y - b) for a, b in piles) < 13.0 or math.hypot(x, y + 47) < 15:
            continue
        out.append({"id": f"W{len(out) + 1}", "xy": [float(x), float(y)], "depth": 0.0,
                    "state": "ambulatory", "lang": _pick_lang(rng)})
    return out


def _pick_lang(rng):
    names, w = zip(*LANG_PRIOR)
    return names[int(rng.choice(len(names), p=np.asarray(w) / sum(w)))]


def _received_db(level_1m, src_xy, depth, extra_db, dst_xy):
    r = max(1.0, math.hypot(src_xy[0] - dst_xy[0], src_xy[1] - dst_xy[1]))
    return level_1m - 20 * math.log10(r) - RUBBLE_DB_PER_M * depth - extra_db


def _bout(kind, rng):
    """(duration s, n_pulses, pulse_period_s) of one response bout."""
    if kind == "knock3":
        per = rng.uniform(0.22, 0.4)
        n = int(rng.choice([3, 3, 3, 2, 4]))
        return (n - 1) * per + 0.12, n, per
    if kind in ("knock_fast", "knock"):
        per = rng.uniform(0.12, 0.2)
        n = int(rng.integers(4, 8))
        return (n - 1) * per + 0.1, n, per
    if kind == "crying":
        return rng.uniform(1.5, 3.0), 0, None
    if kind == "bark":
        return rng.uniform(0.4, 1.2), int(rng.integers(1, 4)), None
    if kind == "debris":
        return rng.uniform(0.3, 1.0), 0, None
    return rng.uniform(0.8, 2.0), 0, None


def _sources(scenario, plan, seed):
    """HIDDEN: emitting sources in [baseline start, window end]: (src_xy, depth, extra_db, kind, t0, owner)."""
    truth = scenario.truth
    rng = np.random.default_rng([int(seed), 4242])
    lrng = np.random.default_rng([int(scenario.seed), 7, 77])          # per-scenario languages (stable)
    spk = plan["drones"][plan["speaker_drone"]]
    tb0, tend = plan["baseline"][0], plan["t_end"]
    out = []
    people = [dict(v, state=(v.get("physio") or {}).get("state", "calm"), lang=_pick_lang(lrng))
              for v in truth.get("victims", [])] + walking_wounded(scenario, scenario.seed)
    for v in people:
        xy, depth = v["xy"], float(v.get("depth", 0.0))
        extra = float(rng.uniform(0, 8)) if depth > 0 else 0.0
        # spontaneous sounds (victims whose scenario role is a sound source), baseline and window alike
        snd = v.get("sound")
        if v["state"] in NO_VOLUNTARY:
            snd = None          # an unconscious / apnoeic person makes no voluntary sound (scenario roles may)
        if snd in ("voice", "weak_voice", "knock"):
            t = tb0 + rng.exponential(6.0)
            while t < tend:
                out.append((xy, depth, extra, {"voice": "voice", "weak_voice": "weak_voice", "knock": "knock"}[snd],
                            t, v["id"]))
                t += rng.exponential(8.0)
        p, modes, lat, p_stop = RESPONSE_MODEL.get(v["state"], RESPONSE_MODEL["calm"])
        heard_db = SPEAKER_DB_1M - 20 * math.log10(max(1.0, math.hypot(xy[0] - spk[0], xy[1] - spk[1]))) \
            - RUBBLE_DB_PER_M * depth - extra
        if heard_db < HEAR_DB or not modes or rng.uniform() >= p:
            continue
        seg = next((s for s in plan["segments"] if s["lang"] in UNDERSTANDS[v["lang"]]), plan["segments"][0])
        kinds, w = zip(*modes)
        kind = kinds[int(rng.choice(len(kinds), p=np.asarray(w) / sum(w)))]
        t = seg["t1"] + rng.uniform(*lat)
        while t < tend:
            out.append((xy, depth, extra, kind, t, v["id"]))
            if rng.uniform() < p_stop:
                break
            t += rng.uniform(3.0, 6.0)
    for s in truth.get("sounds", []):
        if s["kind"] == "dog":                       # dogs bark at loudspeakers; also spontaneously
            t = tb0 + rng.exponential(10.0)
            while t < tend:
                out.append((s["xy"], 0.3, 0.0, "bark", t, f"dog@{s['pile']}"))
                t += rng.exponential(12.0)
            if rng.uniform() < 0.7:
                t = plan["segments"][0]["t0"] + rng.uniform(0.5, 3.0)
                for _ in range(int(rng.integers(2, 5))):
                    out.append((s["xy"], 0.3, 0.0, "bark", t, f"dog@{s['pile']}"))
                    t += rng.uniform(1.5, 4.0)
        elif s["kind"] == "settling":
            t = tb0 + rng.exponential(25.0)
            while t < tend:
                out.append((s["xy"], 0.5, 0.0, "debris", t, f"debris@{s['pile']}"))
                t += rng.exponential(25.0)
    return out, rng


def simulate_callout(scenario, plan, seed=0):
    """PART 1 (TRUTH): per-drone detections during baseline + call-out + listen. Only this function (and
    the helpers above) read ``scenario.truth``. Returns {"detections": [...], "generator": "events-v1"}."""
    srcs, rng = _sources(scenario, plan, seed)
    dets = []
    for xy, depth, extra, kind, t0, _owner in srcs:
        dur, n, per = _bout(kind, rng)
        for did, dxy in plan["drones"].items():
            rx = _received_db(LEVEL_DB[kind], xy, depth, extra, dxy)
            snr = rx - AMBIENT_DB
            if _masked(plan, t0) and _masked(plan, t0 + dur):
                continue                              # wholly under the loudspeaker
            if rng.uniform() > 1.0 / (1.0 + math.exp(-(snr - DET_SNR_DB) / 2.0)):
                continue
            cls = OBS_CLASS[kind]
            u = rng.uniform()                         # classifier confusions
            if cls == "knocking" and u < 0.12:
                cls = "debris"
            elif cls == "debris" and u < 0.15:
                cls = "knocking"
            elif cls == "crying" and u < 0.3:
                cls = "voice"
            elif cls == "dog" and u < 0.05:
                cls = "voice"
            elif cls in ("voice", "shout") and u < 0.2:
                cls = "shout" if cls == "voice" else "voice"
            sig = 3.0 + 30.0 / max(snr, 3.0)
            brg = (math.degrees(math.atan2(xy[1] - dxy[1], xy[0] - dxy[0])) + rng.normal(0, sig)) % 360
            n_obs = max(0, n + int(rng.choice([0, 0, 0, 0, 1, -1]))) if n else 0
            dets.append({"drone_id": did, "t0": round(t0, 2), "t1": round(t0 + dur, 2), "class": cls,
                         "class_conf": round(float(rng.uniform(0.55, 0.92)), 2), "n_pulses": n_obs,
                         "pulse_period_s": None if per is None else round(per + rng.normal(0, 0.02), 3),
                         "bearing_deg": round(brg, 1), "snr_db": round(snr, 1)})
    dets.sort(key=lambda d: (d["t0"], d["drone_id"]))
    return {"detections": dets, "generator": "events-v1 (synthetic, from hidden truth)"}


# ============================================================================ PART 2: analysis (no truth)
def _group(dets, drones, sectors, tol=0.6):
    """Same physical event heard by several drones: onsets within ``tol`` s, same human/non-human class
    family, one detection per drone, and bearings that agree (they intersect in front of every drone, or
    point at the same sector). When two sources overlap in time, a detection joins the group it fits best."""
    groups = []
    for d in sorted(dets, key=lambda d: d["t0"]):
        hum = d["class"] in HUMAN
        best, best_err = None, 1e9
        for g in groups:
            if abs(g["t0"] - d["t0"]) > tol or d["drone_id"] in g["by"] or g["human"] != hum:
                continue
            trial = {"dets": g["dets"] + [d]}
            _, err, _, xy = _associate(trial, {"drones": drones}, sectors)
            rays = [(drones[x["drone_id"]], x["bearing_deg"]) for x in trial["dets"]]
            if (xy is None and triangulate(rays) is None) and err > ASSOC_GATE_DEG:
                continue
            if err < best_err:
                best, best_err = g, err
        if best is not None:
            best["dets"].append(d)
            best["by"].add(d["drone_id"])
        else:
            groups.append({"t0": d["t0"], "dets": [d], "by": {d["drone_id"]}, "human": hum})
    return groups


def triangulate(rays):
    """Least-squares intersection of bearing rays [(origin_xy, bearing_deg)]; None if < 2 or near-parallel."""
    if len(rays) < 2:
        return None
    A, b = np.zeros((2, 2)), np.zeros(2)
    for (ox, oy), brg in rays:
        u = np.array([math.cos(math.radians(brg)), math.sin(math.radians(brg))])
        P = np.eye(2) - np.outer(u, u)
        A += P
        b += P @ np.array([ox, oy])
    if np.linalg.cond(A) > 1e3:
        return None
    x = np.linalg.solve(A, b)
    for (ox, oy), brg in rays:                       # must lie in front of every drone
        if (x[0] - ox) * math.cos(math.radians(brg)) + (x[1] - oy) * math.sin(math.radians(brg)) < 0:
            return None
    return [float(x[0]), float(x[1])]


def _ang(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def _associate(g, plan, sectors):
    """Best sector for an event group from bearings (all drones that heard it): (sector, err_deg, xy)."""
    drones = plan["drones"]
    rays = [(drones[d["drone_id"]], d["bearing_deg"]) for d in g["dets"]]
    xy = triangulate(rays)
    if xy is not None:                              # far-field / near-parallel rays: position unusable
        cx = np.mean([o[0] for o, _ in rays])
        cy = np.mean([o[1] for o, _ in rays])
        span = max(math.hypot(a[0][0] - b[0][0], a[0][1] - b[0][1]) for a in rays for b in rays)
        if math.hypot(xy[0] - cx, xy[1] - cy) > min(EARSHOT_M, 2.5 * max(span, 1.0)):
            xy = None
    errs = []
    for sid, (sx, sy) in sectors.items():
        e = []
        for (ox, oy), brg in rays:
            r = max(1.0, math.hypot(sx - ox, sy - oy))
            ext = math.degrees(math.atan2(6.0, r))           # pile half-extent as seen from the drone
            e.append(max(0.0, _ang(brg, math.degrees(math.atan2(sy - oy, sx - ox))) - ext))
        errs.append((float(np.mean(e)), sid))
    errs.sort()
    best, err = errs[0][1], errs[0][0]
    margin = errs[1][0] - err if len(errs) > 1 else 90.0
    return best, err, margin, xy


def _poisson_sf(n, mu):
    """P(N >= n) for N ~ Poisson(mu)."""
    if n <= 0:
        return 1.0
    s, term = 0.0, math.exp(-mu)
    for k in range(n):
        s += term
        term *= mu / (k + 1)
    return max(0.0, 1.0 - s)


def _describe(g):
    cls = max(g["dets"], key=lambda d: d["class_conf"])["class"]
    pulses = [d["n_pulses"] for d in g["dets"] if d["n_pulses"]]
    pers = [d["pulse_period_s"] for d in g["dets"] if d["pulse_period_s"]]
    if cls == "knocking" and pulses:
        n = int(np.median(pulses))
        per = float(np.median(pers)) if pers else None
        if 2 <= n <= 4 and per and per >= 0.2:
            return "knock_x3", f"knock pattern x{n}"
        if n >= 4 and per and per < 0.2:
            return "knock_fast", f"fast knocking x{n}"
        return "knock", f"knocking x{n}"
    return cls, {"voice": "voice", "shout": "shouting", "crying": "crying (child-like?)"}.get(cls, cls)


def analyse_callout(plan, detections, sectors, pings=(), open_ground=None):
    """PART 2 (measurements only). ``sectors``: {pile_id: (x, y)} public pile centres; ``pings``: radar
    candidates [{id, xy, pile_id}]; ``open_ground(xy) -> bool`` (default: > SECTOR_RADIUS_M from every
    sector). Returns {sectors: {id: responsiveness}, pings: {id: responsiveness}, tracks: [...], events}."""
    spk = plan["drones"][plan["speaker_drone"]]
    seg_ends = [s["t1"] for s in plan["segments"]]
    t_first = plan["segments"][0]["t1"]
    tb0, tb1 = plan["baseline"]
    t_end = plan["t_end"]
    resp_dur = t_end - t_first - sum(s["t1"] - s["t0"] for s in plan["segments"][1:])
    if open_ground is None:
        def open_ground(xy):
            return min(math.hypot(xy[0] - x, xy[1] - y) for x, y in sectors.values()) > SECTOR_RADIUS_M
    groups = _group(detections, plan["drones"], sectors)
    events, per_sector, off = [], {}, []
    for g in groups:
        sid, err, margin, xy = _associate(g, plan, sectors)
        kind, words = _describe(g)
        ev = {"t": g["t0"], "drones": sorted(g["by"]), "class": max(g["dets"], key=lambda d: d["class_conf"])["class"],
              "kind": kind, "words": words, "sector": sid if err <= ASSOC_GATE_DEG and margin >= ASSOC_MARGIN_DEG else None,
              "assoc_err_deg": round(err, 1), "assoc_margin_deg": round(margin, 1),
              "xy": None if xy is None else [round(xy[0], 1), round(xy[1], 1)],
              "conf": round(float(np.mean([d["class_conf"] for d in g["dets"]])), 2),
              "snr_db": max(d["snr_db"] for d in g["dets"]),
              "phase": "baseline" if g["t0"] < plan["t_start"] else ("message" if g["t0"] < t_first else "response")}
        prev = [e for e in seg_ends if e <= g["t0"]]
        ev["latency_s"] = round(g["t0"] - prev[-1], 2) if prev else None
        ev["after_lang"] = plan["segments"][len(prev) - 1]["lang"] if prev else None
        if xy is not None and len(g["by"]) >= 2 and g["human"] and open_ground(xy):
            ev["sector"] = None
            ev["open_ground"] = True
            off.append(ev)
        events.append(ev)
        if ev["sector"]:
            per_sector.setdefault(ev["sector"], []).append(ev)

    out = {}
    for sid, (sx, sy) in sectors.items():
        r = math.hypot(sx - spk[0], sy - spk[1])
        if r > EARSHOT_M:
            continue
        evs = per_sector.get(sid, [])
        base = [e for e in evs if e["phase"] == "baseline" and e["class"] in HUMAN]
        resp = [e for e in evs if e["phase"] in ("message", "response") and e["t"] >= t_first and e["class"] in HUMAN]
        nonh = [e for e in evs if e["phase"] in ("message", "response") and e["class"] not in HUMAN]
        lam = max(len(base) / max(tb1 - tb0, 1.0), 0.01)
        p_chance = _poisson_sf(len(resp), lam * max(resp_dur, 1.0))
        # a pattern the sector was not making before (e.g. the instructed knock x3 after spontaneous fast
        # knocking, or a voice after silence) is judged against a small floor rate instead
        novel = [e for e in resp if e["kind"] not in {b["kind"] for b in base}]
        if novel:
            p_chance = min(p_chance, _poisson_sf(len(novel), 0.01 * max(resp_dur, 1.0)))
        locked = [e for e in resp if e["latency_s"] is not None and e["latency_s"] <= RESP_MAX_LATENCY_S]
        multi = [e for e in resp if len(e["drones"]) >= 2]
        k3 = [e for e in resp if e["kind"] == "knock_x3"]
        conf = (1.0 - p_chance) * (0.55 + 0.15 * bool(locked) + 0.15 * bool(multi) + 0.15 * bool(k3))
        conf *= float(np.mean([e["conf"] for e in resp])) / 0.75 if resp else 0.0
        conf = round(float(min(conf, 0.97)), 2)
        heard_db = SPEAKER_DB_1M - 20 * math.log10(max(r, 1.0)) - RUBBLE_DB_PER_M * 1.5
        reached = heard_db >= HEAR_DB + 3
        first = min(resp, key=lambda e: e["t"]) if resp else None
        res = {"sector": sid, "callout_t": plan["t_start"], "speaker_drone": plan["speaker_drone"],
               "range_m": round(r, 1), "est_message_db": round(heard_db, 0), "baseline_rate_per_min": round(60 * len(base) / max(tb1 - tb0, 1), 1),
               "n_response_events": len(resp), "p_chance": round(p_chance, 3), "confidence": conf,
               "events": [{k: e[k] for k in ("t", "kind", "words", "drones", "latency_s", "after_lang")} for e in resp[:6]]}
        strong = [e for e in resp if len(e["drones"]) >= 2]
        if resp and conf >= 0.6 and (len(strong) >= 2 or any(e["kind"] == "knock_x3" for e in strong)):
            kinds = [e["kind"] for e in resp]
            kind = "knock_x3" if "knock_x3" in kinds else max(set(kinds), key=kinds.count)
            drones = sorted({d for e in resp for d in e["drones"]})
            lat = f"{first['latency_s']:.1f} s after the {first['after_lang']} message" if first["latency_s"] is not None else ""
            res.update(status="responded", kind=kind, avpu="V",
                       lang_hint=first["after_lang"] if first["latency_s"] is not None and first["latency_s"] <= 4 else None,
                       text=f"Responded to call-out: {first['words']} at t {first['t']:.0f} s ({lat}; heard by "
                            f"{', '.join(drones)}). Responds to voice (AVPU V). Not a confirmed person: a candidate.")
        elif resp:
            res.update(status="possible_response", kind=first["kind"], avpu="unknown",
                       text=f"Possible response: {first['words']} at t {first['t']:.0f} s, but "
                            + ("this sector was already making such sounds before the call-out" if base else
                               "weak or single-drone evidence") + ". Treat as unconfirmed.")
        elif nonh:
            res.update(status="nonhuman_only", kind=nonh[0]["kind"], avpu="unknown",
                       text=f"Only {nonh[0]['words']} followed the call-out (dogs react to loudspeakers); "
                            "no human response. Not cleared.")
        else:
            res.update(status="no_response", kind=None, avpu="not V (A/V not shown)",
                       text=("No response to the call-out. This does not rule out an unconscious person "
                             "(or someone who cannot hear, move or make a sound). Not cleared.")
                       + ("" if reached else f" The message may not have reached this sector ({r:.0f} m away)."))
        res["message_reached"] = bool(reached)
        out[sid] = res

    ping_res = {}
    for p in pings or ():
        s = out.get(p.get("pile_id"))
        if s is None:
            continue
        others = [q for q in pings if q.get("pile_id") == p.get("pile_id") and q["id"] != p["id"]]
        ping_res[p["id"]] = dict(s, ping_id=p["id"], note="sector-level: the call-out cannot tell which person in "
                                                         "the sector responded" if others else None)

    tracks = []
    for e in off:
        near = next((t for t in tracks if math.hypot(t["xy"][0] - e["xy"][0], t["xy"][1] - e["xy"][1]) < 6.0), None)
        if near:
            near["evidence"].append(e["t"])
            continue
        tracks.append({"id": f"T{len(tracks) + 1}", "xy": e["xy"], "source": "callout_voice_open_ground",
                       "t": e["t"], "class": e["class"], "evidence": [e["t"]],
                       "why": f"{e['words']} heard by {', '.join(e['drones'])} triangulates to open ground "
                              f"({e['xy'][0]:.0f}, {e['xy'][1]:.0f}), away from the rubble: probably able to walk"})
    return {"sectors": out, "pings": ping_res, "tracks": tracks, "events": events}


# ============================================================================ fusion
TRIAGE_RANK = {"immediate": 1.0, "immediate (check)": 0.85, "assess": 0.55, "delayed (re-check)": 0.45, "delayed": 0.35}
ABNORMAL = ("rapid_shallow", "irregular_with_pauses", "slow_deep")


def fuse_triage(vitals, resp):
    """Respiratory hint (sim/vitals.py dict: pattern, triage_hint) + call-out responsiveness -> fused hint.
    START logic: cannot follow a simple command -> immediate. An aid for the dig order, never a diagnosis."""
    v = vitals or {}
    hint, pattern = v.get("triage_hint", "assess"), v.get("pattern")
    st = (resp or {}).get("status")
    out = {"triage_hint": hint, "avpu": (resp or {}).get("avpu", "unknown"), "changed_from": None,
           "reason": v.get("triage_reason", "no breathing pattern")}
    if st == "no_response" and (resp or {}).get("message_reached", True):
        if pattern in ABNORMAL:
            new, why = "immediate", "no response to the call-out + abnormal breathing: possible reduced consciousness"
        elif pattern in ("regular_calm", "fast_small"):
            new, why = "immediate (check)", ("no response to the call-out (START: cannot follow a command) with "
                                             "regular breathing; may simply not hear through the rubble")
        else:
            new, why = hint if hint != "delayed" else "immediate (check)", "no response to the call-out; breathing unknown"
    elif st == "responded":
        if pattern == "rapid_shallow":
            new, why = "immediate", "responds to voice but breathing is fast (START respiratory criterion)"
        elif pattern in ("irregular_with_pauses", "slow_deep"):
            new, why = "immediate (check)", "responds to voice, but breathing is irregular or slow"
        elif pattern == "regular_calm":
            new, why = "delayed", "responds to voice and breathes regularly"
        else:
            new, why = hint, "responds to voice (AVPU V); breathing pattern unknown"
    else:
        return out
    if new != hint:
        out["changed_from"] = hint
    out.update(triage_hint=new, reason=why, rank=TRIAGE_RANK.get(new, 0.55))
    return out


def association_boost(reliability, resp):
    """Call-and-response is a timing reference: a time-locked response from the same sector multiplies the
    odds that the sector's acoustic lead is a real, correctly associated person (likelihood ratio 1 + 3c)."""
    if not resp or resp.get("status") != "responded":
        return reliability, None
    r = min(max(float(reliability or 0.3), 0.01), 0.99)
    lr = 1.0 + 3.0 * resp["confidence"]
    odds = r / (1 - r) * lr
    new = round(odds / (1 + odds), 3)
    return new, f"call-and-response at {resp['sector']} (confidence {resp['confidence']:.2f}, LR {lr:.1f})"


def response_priority(resp):
    """Acoustic-priority-style score (0-100, same formula as the acoustic agent) for a sector that responded."""
    if not resp or resp.get("status") not in ("responded", "possible_response"):
        return None
    c = resp["confidence"] if resp["status"] == "responded" else 0.5 * resp["confidence"]
    urg = {"knock_fast": 0.75, "crying": 0.7, "voice": 0.65, "shout": 0.6, "knock_x3": 0.5, "knock": 0.5}.get(resp["kind"], 0.5)
    h = 0.5 + 0.45 * c
    return int(round(100 * h * (0.35 + 0.65 * urg) * (0.45 + 0.55 * c)))


# ============================================================================ mission facade
ASSET_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "acoustic", "data", "speaker")


def load_manifest(asset_dir=ASSET_DIR):
    """acoustic/speaker_assets.py manifest: {key: {lang, text, path, duration_s}} (empty if not built)."""
    import json
    try:
        with open(os.path.join(asset_dir, "manifest.json")) as f:
            return json.load(f).get("items", {})
    except (OSError, ValueError):
        return {}


class Speaker:
    """What the mission calls. ``begin_listen`` plans the call-out after the passive listen, runs the
    generator (``scenario`` goes only to PART 1) and the analysis, and schedules reassurance + guidance;
    results are revealed by time (``state(t)``, ``results_at(t)``)."""

    def __init__(self, sectors, grid, base_xy, seed=0, langs=LANGS, zones=(), enabled=True, asset_dir=ASSET_DIR):
        self.sectors = {k: (float(v[0]), float(v[1])) for k, v in sectors.items()}
        self.grid, self.base_xy, self.seed, self.langs = grid, tuple(base_xy), int(seed), tuple(langs)
        self.zones, self.enabled = list(zones or ()), enabled
        self.manifest = load_manifest(asset_dir)
        self.log = BroadcastLog(audio_lookup=self._audio)
        self.callouts = []
        self.guided = []                  # tracks already guided (xy), so a track is not re-guided every listen
        self._cmap = None
        self.n = 0

    def _audio(self, lang, text):
        it = self.manifest.get(audio_key(lang, text))
        if it and it.get("path"):
            return f"acoustic/data/speaker/{os.path.basename(it['path'])}", it.get("status", "rendered")
        return None, (it or {}).get("status") or "no audio yet (acoustic/speaker_assets.py)"

    def _dur(self, lang, text):
        it = self.manifest.get(audio_key(lang, text))
        return float(it["duration_s"]) if it and it.get("duration_s") else est_duration(text, lang)

    def _play_all(self, t, drone, kind, texts, target, heading=None, template=None):
        """Schedule one message in every language, back to back; returns the end time."""
        for lang in self.langs:
            d = self._dur(lang, texts[lang])
            self.log.add(t=t, drone_id=drone, kind=kind, lang=lang, text=texts[lang], text_en=texts["en"],
                         target=target, heading_deg=heading, template=template, duration_s=round(d, 2),
                         group=f"{kind}-{self.n}")
            t += d + GAP_S
        return t

    def callout_duration(self):
        segs = sum(self._dur(lang, MESSAGES["callout"][lang]) + GAP_S for lang in self.langs) - GAP_S
        return segs + LISTEN_WINDOW_S

    def begin_listen(self, scenario, t_callout, drones, target, pings=(), moving_tracks=(), callout_fn=None):
        """Returns (rotors_off_seconds_after_t_callout, result): the mission keeps the drones landed and
        silent until the call-out's listen window ends; reassurance and guidance follow from the hover
        (playing needs no silence), scheduled after that. ``target``: {"type": "sector"|"ping", "id", "xy"}.
        ``moving_tracks``: radar Doppler tracks [{id, xy}] (public). ``callout_fn`` replaces the generator."""
        if not self.enabled:
            return 0.0, None
        self.n += 1
        durs = {lang: self._dur(lang, MESSAGES["callout"][lang]) for lang in self.langs}
        plan = plan_callout(t_callout, drones, target, self.langs, durs)
        for s in plan["segments"]:
            self.log.add(t=s["t0"], drone_id=plan["speaker_drone"], kind="callout", lang=s["lang"],
                         text=MESSAGES["callout"][s["lang"]], text_en=MESSAGES["callout"]["en"],
                         target={"type": target["type"], "id": target["id"]}, template="callout",
                         duration_s=round(s["t1"] - s["t0"], 2), group=f"callout-{self.n}")
        obs = (callout_fn or (lambda sc, pl, sd: simulate_callout(sc, pl, sd)))(scenario, plan, self.seed + 977 * self.n)
        res = analyse_callout(plan, obs["detections"], self.sectors, pings)
        t = plan["t_end"] + 0.5
        # reassurance: every ping in earshot and every sector that responded
        targets = [{"type": "ping", "id": p["id"], "xy": p["xy"]} for p in pings or ()
                   if p["id"] in res["pings"] or p.get("pile_id") == target.get("id")]
        targets += [{"type": "sector", "id": s, "xy": self.sectors[s]} for s, r in res["sectors"].items()
                    if r["status"] == "responded" and not any(p.get("pile_id") == s for p in pings or ())]
        reassured = []
        if targets:
            tg = targets[0] if len(targets) == 1 else {"type": "sectors", "id": ",".join(x["id"] for x in targets)}
            spk = min(drones, key=lambda d: min(math.hypot(drones[d][0] - x["xy"][0], drones[d][1] - x["xy"][1])
                                                for x in targets))
            t = self._play_all(t, spk, "reassure", MESSAGES["reassure"], {"type": tg["type"], "id": tg["id"]},
                               template="reassure")
            reassured = [x["id"] for x in targets]
        # guidance: ambulatory people on open ground (call-out voices) and radar Doppler tracks
        guided = []
        for tr in list(res["tracks"]) + [dict(m, source="radar_doppler") for m in moving_tracks or ()]:
            if any(math.hypot(tr["xy"][0] - x, tr["xy"][1] - y) < 8.0 for x, y in self.guided):
                continue
            if self._cmap is None:
                self._cmap = cost_map(self.grid, self.zones)
            route = guidance_route(self.grid, tr["xy"], self.base_xy, cmap=self._cmap)
            if not route["ok"] or not route["legs"]:
                continue
            texts = guidance_texts(route["legs"], langs=self.langs if "en" in self.langs else self.langs + ("en",))
            spk = min(drones, key=lambda d: math.hypot(drones[d][0] - tr["xy"][0], drones[d][1] - tr["xy"][1]))
            t = self._play_all(t, spk, "guidance", texts, {"type": "track", "id": f"{self.n}-{tr['id']}",
                                                           "xy": tr["xy"]}, heading=route["heading_deg"],
                               template="guidance")
            self.guided.append(tuple(tr["xy"]))
            guided.append({"track": f"{self.n}-{tr['id']}", "xy": tr["xy"], "source": tr.get("source"),
                           "why": tr.get("why"), "route": route, "text_en": texts["en"]})
        rec = {"id": f"C{self.n}", "t_start": plan["t_start"], "t_end": round(t, 2), "listen_ends": plan["t_end"],
               "speaker_drone": plan["speaker_drone"], "target": plan["target"], "langs": plan["langs"],
               "steps": [{"step": "silence", "t0": plan["baseline"][0], "t1": plan["baseline"][1],
                          "note": "rotors off since landing; passive listen = quiet baseline; ground crews asked to "
                                  "stop work (radio)"}]
               + [{"step": "message", "lang": s["lang"], "t0": s["t0"], "t1": s["t1"]} for s in plan["segments"]]
               + [{"step": "listen", "t0": plan["listen"][0], "t1": plan["listen"][1]}],
               "sectors": res["sectors"], "pings": res["pings"], "tracks": res["tracks"],
               "events": res["events"], "reassured": reassured, "guided": guided,
               "generator": obs.get("generator"), "synthetic": True}
        self.callouts.append(rec)
        return plan["t_end"] - t_callout, rec

    def active(self, t):
        """{drone_id: {kind, lang, id}} for broadcasts playing at time t (dashboard: speaker icon on the drone)."""
        out = {}
        for b in self.log.playable(t):
            if b["t"] <= t < b["t"] + (b.get("duration_s") or 0):
                out[b["drone_id"]] = {"kind": b["kind"], "lang": b["lang"], "id": b["id"]}
        return out

    def results_at(self, t):
        return [c for c in self.callouts if c["listen_ends"] <= t]

    def state(self, t):
        """{"broadcasts", "callouts", "responsiveness"} as of time t (results only after their listen ends)."""
        done = self.results_at(t)
        resp = {}
        for c in done:                     # per sector: the call-out heard from closest (best geometry), then latest
            for sid, r in c["sectors"].items():
                prev = resp.get(sid)
                if prev is None or r["range_m"] <= prev["range_m"] + 2.0:
                    resp[sid] = {k: r[k] for k in ("status", "kind", "avpu", "confidence", "text", "callout_t",
                                                   "range_m", "message_reached")}
                    resp[sid]["callout_id"] = c["id"]
        live = [c for c in self.callouts if c["listen_ends"] > t]
        return {"broadcasts": self.log.state(t),
                "callouts": [dict(c, guided=[{k: g[k] for k in ("track", "xy", "source", "why", "text_en")}
                                             | {"path": g["route"]["path"], "legs": g["route"]["legs"]}
                                             for g in c["guided"]]) for c in done]
                + [{"id": c["id"], "t_start": c["t_start"], "listen_ends": c["listen_ends"], "status": "in_progress",
                    "speaker_drone": c["speaker_drone"], "target": c["target"]} for c in live if c["t_start"] <= t],
                "responsiveness": resp}
