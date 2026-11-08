"""Acoustic listen-and-rank for the grid mission: bridge to acoustic/ plus a SYNTHETIC numpy stub.

``listen(scenario, drone_positions, exclude_piles, seed, mode)`` returns ``(result, source)`` where
``result`` is a ``larpverse.acoustic/v1`` dict (clips, piles, dispatch, agent) and ``source`` says which
implementation produced it:

* ``real``       ``acoustic.run.listen_and_rank`` imported in this interpreter
* ``real-venv``  the same function run in ``.venv-audio`` as a subprocess (JSON over stdin/stdout)
* ``stub-from-truth``  ``listen_and_rank`` below: synthetic DoA + fixed scoring from HIDDEN TRUTH, numpy only

mode: ``auto`` (real -> real-venv -> stub), ``real`` (no stub fallback), ``stub``.

*** The stub FABRICATES OBSERVATIONS FROM HIDDEN GROUND TRUTH (``Scenario.truth['sounds']``). ***
Like sim/radar_model.py it is an observation generator; the coordinator only sees its output dict.
"""
from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys

import numpy as np

from acoustic import priority as tiers      # numpy/scipy only (the priority list: trapped vs safe on the surface)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AUDIO_PY = os.path.join(ROOT, ".venv-audio", "bin", "python")
# real-pipeline results are cached here (gitignored data/); extra read-only dirs (e.g. a committed demo cache)
# can be listed in ACOUSTIC_CACHE_DIRS (os.pathsep-separated). The old served-folder cache is still read.
CACHE_DIR = os.environ.get("ACOUSTIC_CACHE_DIR", os.path.join(ROOT, "data", "acoustic_cache"))


def _pipeline_version():
    """Hash of the acoustic pipeline code (acoustic/*.py, which also names the model ids) - part of the cache key,
    so a pipeline change never replays clips from an older version."""
    h = hashlib.sha1()
    d = os.path.join(ROOT, "acoustic")
    try:
        for name in sorted(os.listdir(d)):
            if name.endswith(".py"):
                with open(os.path.join(d, name), "rb") as f:
                    h.update(name.encode() + b"\0" + f.read())
    except OSError:
        return "none"
    return h.hexdigest()[:12]


ACOUSTIC_VERSION = _pipeline_version()
STUB_SOURCE = "stub-from-truth"
SNAP_M = 1.0                 # drone positions are snapped to this grid in the cache key
SCHEMA = "larpverse.acoustic/v1"
HUMAN_LEAD = 0.4             # same constants as acoustic/agent.py
SILENT_PRIORITY = 8
PILE_NAMES = [f"{c}{r}" for r in (1, 2, 3) for c in "ABC"]   # grid site only; listen_and_rank uses the scenario

# kind -> (sound_class, human, urgency, class score) for the fixed-scoring stub
_KIND = {
    "voice": ("voice", 0.90, 0.92, 0.80),
    "weak_voice": ("voice", 0.86, 0.66, 0.62),
    "knock": ("knocking", 0.65, 0.43, 0.55),
    "dog": ("dog", 0.04, 0.02, 0.80),
    "generator": ("engine", 0.10, 0.02, 0.60),
    "settling": ("debris", 0.02, 0.02, 0.55),
    # people SAFE on the surface: human, but heard in the open air (no rubble low-pass)
    "surface_voice": ("voice", 0.88, 0.30, 0.78),
    "footsteps": ("footsteps", 0.55, 0.10, 0.60),
}
_CLASSES = ["voice", "shout", "knocking", "dog", "engine", "debris", "noise"]
# rubble depth per kind, as acoustic/scene.KINDS synthesises it (0 = open air): the stub's stand-in for the DSP
# muffling feature (buried sounds lose their highs)
_DEPTH = {"voice": 0.4, "weak_voice": 1.0, "knock": 0.8, "dog": 0.0, "generator": 0.0, "settling": 0.3,
          "surface_voice": 0.0, "footsteps": 0.0}


def clip_priority(j) -> int:
    return int(round(100 * j["human"] * (0.35 + 0.65 * j["urgency"]) * (0.45 + 0.55 * j["reliability"])))


def _short(t, n=48):
    t = (t or "").strip()
    return t if len(t) <= n else t[: n - 1].rsplit(" ", 1)[0] + "…"


def _degrade(text, keep, rng):
    words = text.split()
    out = [w if rng.uniform() < keep else "…" for w in words]
    s = " ".join(out)
    while "… …" in s:
        s = s.replace("… …", "…")
    return s


# --------------------------------------------------------------------------- stub (SYNTHETIC)
def listen_and_rank(scenario, drone_positions: dict, exclude_piles=(), seed: int = 0) -> dict:
    """Synthetic stand-in for acoustic.run.listen_and_rank (same signature and output schema)."""
    exclude = set(exclude_piles or ())
    names = [p.name for p in scenario.piles]             # sector names come from the scenario (any site)
    rng = np.random.default_rng([int(seed), 17] + sorted(names.index(p) for p in exclude if p in names))
    drones = {k: (float(v[0]), float(v[1])) for k, v in drone_positions.items()}
    pile_xy = {p.name: (float(p.x), float(p.y)) for p in scenario.piles}
    clips = []
    for snd in list(scenario.truth.get("sounds", [])) + list(scenario.truth.get("surface_sounds") or []):   # GENERATOR
        pid = snd["pile"]
        if pid in exclude:
            continue
        sx, sy = snd["xy"]
        dist = {d: float(np.hypot(sx - o[0], sy - o[1])) for d, o in drones.items()}
        did = min(dist, key=dist.get)
        d = dist[did]
        weak = snd["kind"] == "weak_voice"
        snr = 21.0 + snd["level_db"] - 20 * np.log10(max(d, 3.0) / 7.0) - (5.0 if weak else 0.0) + rng.normal(0, 1.5)
        agreeing = sum(1 for v in dist.values() if v < d + 18.0 and snr > 4.0)
        if snr < 2.0 or agreeing < 2:
            continue
        if snd["kind"] not in _KIND:
            continue
        cls, human, urg, cscore = _KIND[snd["kind"]]
        width = float(np.clip(16.0 + 0.25 * d + max(0.0, 8.0 - snr), 18.0, 45.0))
        o = drones[did]
        bearing = float(np.degrees(np.arctan2(sy - o[1], sx - o[0])) + rng.normal(0, width / 6.0))
        bearing = (bearing + 180.0) % 360.0 - 180.0
        scores = {c: 0.0 for c in _CLASSES}
        scores[cls] = cscore
        rest = rng.dirichlet(np.ones(len(_CLASSES) - 1)) * (1.0 - cscore)
        for c, v in zip([c for c in _CLASSES if c != cls], rest):
            scores[c] = v
        if cls not in _CLASSES:          # footsteps: no v1 class of their own (YAMNet "Walk, footsteps")
            scores.pop(cls)
        scores = {c: round(float(v), 3) for c, v in scores.items()}
        text = ""
        if snd.get("text"):
            text = snd["text"] if snr > 9.0 else _degrade(snd["text"], 0.55 + 0.04 * snr, rng)
        r_snr = float(np.clip(snr / 15.0, 0, 1))
        r_doa = (agreeing / len(drones)) * float(np.clip(1.0 - (width - 20.0) / 80.0, 0.2, 1.0))
        rel = round(float(np.clip(0.6 * r_doa + 0.4 * r_snr, 0, 1)), 2)
        human = round(float(np.clip(human + rng.normal(0, 0.03), 0, 1)), 2)
        urgency = round(float(np.clip(urg + rng.normal(0, 0.04), 0, 1)), 2)
        rationale = (f"{cls} {scores.get(cls, cscore):.2f}" + (f"; transcript {len(text.split())} words" if text else "")
                     + f"; {agreeing}/{len(drones)} drones agree; width {width:.0f} deg, SNR {snr:.0f} dB")
        cid = f"ac-{pid}" if not any(c["clip_id"] == f"ac-{pid}" for c in clips) else f"ac-{pid}-{snd['kind']}"
        c = {"clip_id": cid, "drone_id": did, "pile_id": pid, "bearing_deg": round(bearing, 1),
             "width_deg": round(width, 1), "snr_db": round(float(snr), 1),
             "vad_speech_ratio": round(0.6 if text else 0.02, 2), "transcript": text, "sound_class": cls,
             "class_scores": scores, "human": human, "urgency": urgency, "reliability": rel,
             "rationale": rationale, "by": "fallback", "doa_agree": f"{agreeing}/{len(drones)}"}
        c["priority"] = clip_priority(c)
        d = _DEPTH.get(snd["kind"], 0.0)
        mdb = 2.0 if cls == "footsteps" else round(-16.0 - 45.0 * d, 1)      # no rng draw: other clips unchanged
        c["muffling"] = {"hf_ratio_db": mdb, "setting": tiers.setting_from_db(mdb), "synthetic": True}
        tiers.tier_v1_clip(c)
        tiers.dampen(c)
        if c["tier"] == "safe_surface":
            c["rationale"] += f"; {c['tier_why']}"
        clips.append(c)

    by_pile = {}
    for c in clips:
        by_pile.setdefault(c["pile_id"], []).append(c)
    piles = []
    for pid, xy in pile_xy.items():
        if pid in exclude:
            continue
        heard = sorted(by_pile.get(pid, []), key=lambda c: -c["priority"])
        cs = tiers.lead_clips(heard)
        if not cs:
            piles.append({"pile_id": pid, "xy": list(xy), "priority": SILENT_PRIORITY, "status": "no_signal_not_cleared",
                          "top_clip": heard[0]["clip_id"] if heard else None,
                          "why": tiers.SAFE_PILE_WHY if heard else
                          "No sound assigned to this pile during the listen. Silence is not absence: not cleared."})
            continue
        top = cs[0]
        pr = min(100, top["priority"] + 3 * (len(cs) - 1))
        if top["human"] < HUMAN_LEAD:
            st, why = "dismissed_nonhuman", (f"Heard {top['sound_class']} (human {top['human']:.2f}); dismissed as an "
                                            "acoustic lead, but the pile is not cleared.")
        else:
            said = f" saying \"{_short(top['transcript'])}\"" if top.get("transcript") else ""
            st, why = "lead", (f"{top['sound_class'].capitalize()}{said}; human {top['human']:.2f}, urgency "
                               f"{top['urgency']:.2f}, reliability {top['reliability']:.2f}.")
        piles.append({"pile_id": pid, "xy": list(xy), "priority": pr, "status": st, "top_clip": top["clip_id"], "why": why})
    piles.sort(key=lambda p: (-p["priority"], p["pile_id"]))
    leads = [p for p in piles if p["status"] == "lead"]
    if leads:
        p = leads[0]
        top = next(c for c in clips if c["clip_id"] == p["top_clip"])
        what = f"\"{_short(top['transcript'])}\"" if top.get("transcript") else top["sound_class"]
        dispatch = {"pile_id": p["pile_id"], "reason": (f"Send the radar drones to {p['pile_id']}: highest priority "
                                                       f"({p['priority']}) from {what}, {top['doa_agree']} drones "
                                                       f"agreeing on the bearing at {top['snr_db']:.0f} dB SNR.")}
    else:
        dispatch = {"pile_id": None, "reason": "No human-like acoustic lead; continue the systematic search and keep "
                                               "every pile marked not cleared."}
    return {"schema": SCHEMA, "seed": int(seed), "synthetic": True,
            "listening_drones": [{"id": k, "xy": list(v)} for k, v in drones.items()],
            "clips": clips, "piles": piles, "dispatch": dispatch, "priority_list": tiers.PRIORITY_LIST,
            "agent": {"mode": "stub", "model": None, "llm_calls": 0, "cache_hits": 0, "fallback": len(clips), "error": None},
            "log": []}


# --------------------------------------------------------------------------- real pipeline bridge
_SUBPROC = r"""
import json, sys
sys.path.insert(0, %r)
from sim.scenario import make_scenario
from acoustic.run import listen_and_rank
a = json.load(sys.stdin)
sc = make_scenario(a["scenario_seed"], a["start_pile"])
r = listen_and_rank(sc, {k: tuple(v) for k, v in a["drones"].items()}, a["exclude"], a["seed"])
sys.stdout.write("\n@@ACOUSTIC_JSON@@" + json.dumps(r))
"""


def _real_available() -> bool:
    try:
        with open(os.path.join(ROOT, "acoustic", "run.py")) as f:
            return "def listen_and_rank" in f.read()
    except OSError:
        return False


def _run_inprocess(scenario, drones, exclude, seed):
    from acoustic.run import listen_and_rank as real   # lazy: heavy deps live in .venv-audio
    return real(scenario, drones, list(exclude), seed)


def _run_subprocess(scenario, drones, exclude, seed, timeout=900):
    payload = json.dumps({"scenario_seed": scenario.seed, "start_pile": scenario.public_dict()["start_pile"],
                          "drones": {k: list(v) for k, v in drones.items()}, "exclude": list(exclude), "seed": seed})
    cmd = ["nice", "-n", "19", AUDIO_PY, "-c", _SUBPROC % ROOT]
    p = subprocess.run(cmd, input=payload, capture_output=True, text=True, cwd=ROOT, timeout=timeout)
    if p.returncode != 0 or "@@ACOUSTIC_JSON@@" not in p.stdout:
        raise RuntimeError(f"acoustic subprocess failed ({p.returncode}): {p.stderr.strip()[-400:]}")
    return json.loads(p.stdout.rsplit("@@ACOUSTIC_JSON@@", 1)[1])


def _valid(r):
    return isinstance(r, dict) and isinstance(r.get("clips"), list) and isinstance(r.get("piles"), list)


def _cache_key(scenario, drones, exclude, seed):
    d = {k: [round(float(v[0]) / SNAP_M) * SNAP_M, round(float(v[1]) / SNAP_M) * SNAP_M]
         for k, v in sorted(drones.items())}
    key = json.dumps({"scenario": [scenario.seed, scenario.public_dict()["start_pile"]], "drones": d,
                      "exclude": sorted(set(exclude)), "seed": int(seed), "version": ACOUSTIC_VERSION},
                     sort_keys=True)
    return hashlib.sha1(key.encode()).hexdigest()[:16] + ".json"


def _cache_path(scenario, drones, exclude, seed):
    return os.path.join(CACHE_DIR, _cache_key(scenario, drones, exclude, seed))


def _cache_lookup(scenario, drones, exclude, seed):
    """First valid cached result of this pipeline version: CACHE_DIR, then ACOUSTIC_CACHE_DIRS (read-only)."""
    name = _cache_key(scenario, drones, exclude, seed)
    dirs = [CACHE_DIR] + [d for d in os.environ.get("ACOUSTIC_CACHE_DIRS", "").split(os.pathsep) if d]
    for path in (os.path.join(d, name) for d in dirs):
        if os.path.exists(path):
            try:
                with open(path) as f:
                    r = json.load(f)
                if _valid(r) and r.get("_cache_version") == ACOUSTIC_VERSION:
                    return r
            except (OSError, ValueError):
                continue
    return None


def listen(scenario, drone_positions, exclude_piles=(), seed=0, mode="auto", log=None, cache=True):
    """Run the best available listen-and-rank; returns (result_dict, source).

    Real results are cached per (scenario, drone positions, excluded piles, seed) in
    dashboard/data/acoustic_cache/ so reruns (and Isaac runs on machines without .venv-audio) are instant.
    """
    say = log or (lambda m: print(m, file=sys.stderr))
    exclude_piles = list(exclude_piles or ())
    path = _cache_path(scenario, drone_positions, exclude_piles, seed)
    if mode != "stub" and cache:
        r = _cache_lookup(scenario, drone_positions, exclude_piles, seed)
        if r is not None:
            return r, "real-cached"
    r, src = _listen_uncached(scenario, drone_positions, exclude_piles, seed, mode, say)
    if cache and src.startswith("real"):
        r = dict(r, _cache_version=ACOUSTIC_VERSION)
        try:
            os.makedirs(CACHE_DIR, exist_ok=True)
            with open(path + ".tmp", "w") as f:
                json.dump(r, f)
            os.replace(path + ".tmp", path)
        except OSError:
            pass
    return r, src


def _listen_uncached(scenario, drone_positions, exclude_piles, seed, mode, say):
    if mode != "stub" and _real_available():
        try:
            r = _run_inprocess(scenario, drone_positions, exclude_piles, seed)
            if _valid(r):
                return r, "real"
        except Exception as e:                      # noqa: BLE001  (missing deps in this venv, etc.)
            err = f"{type(e).__name__}: {e}"
        else:
            err = "invalid result"
        if os.path.exists(AUDIO_PY):
            try:
                r = _run_subprocess(scenario, drone_positions, exclude_piles, seed)
                if _valid(r):
                    return r, "real-venv"
                err = "invalid result from .venv-audio"
            except Exception as e:                  # noqa: BLE001
                err = f"{type(e).__name__}: {e}"
        if mode == "real":
            raise RuntimeError(f"real acoustic pipeline unavailable: {err}")
        say(f"acoustic pipeline unavailable ({err[:160]}); using the synthetic stub")
    elif mode == "real":
        raise RuntimeError("acoustic.run.listen_and_rank not found")
    return listen_and_rank(scenario, drone_positions, exclude_piles, seed), STUB_SOURCE
