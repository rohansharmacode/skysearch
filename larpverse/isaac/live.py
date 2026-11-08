"""Live single-take mode (``run_mission.py --live``): the Isaac stage is the source of truth for the victims, live
unscripted triggers, and the ``--camera director-take`` spline camera. Pure python + numpy; every omni/pxr/carb
import is guarded, so ``--dry-run`` and tests run without Isaac.

World override (generator-only)
    ``LiveWorld`` reads the CURRENT dummies (``isaac.dummies.read_dummies(stage)`` -> ``[{id, xyz, active, clip,
    physio}]``; a list source in --dry-run) on every radar scan, every listen and once per wall second. Active dummies
    become the victims / sounds the observation generators see: the radar generator (``LiveRadar``, rebuilt per scan
    with this pile's current targets and a PhysX column probe for dragged debris), the acoustic generator
    (``make_acoustic_fn``) and the loudspeaker call-out generator (``LiveScenario.truth``). The generators keep their
    synthetic physics; only the source list comes from the stage. When the stage differs from the seeded scenario
    (a hidden dummy switched on, a dummy moved, a clip changed) every listen is a FRESH, UNCACHED synthesis.

Honesty boundary
    The mission's reasoning never reads the stage. ``LiveController`` decides only from generator OUTPUTS (pod-mic
    quick listens compared with what was heard before), operator tasking revisions and portal submissions in the
    mission state. Stage events are used for (a) cache freshness, (b) the on-screen event line and (c) the camera,
    all display / generator concerns.

Mission hooks
    Native only (sim/grid_mission.py, fa4e74b): ``set_world_sources`` (generators read the stage), ``ingest_listen``
    (pod-mic / idle-drone listens folded in without a gate) and ``preempt`` (cut the radar phase after the dwell
    under way; an operator order wins the gate). ``--live`` refuses to start without them.

Real acoustics (the take)
    ``AcousticService``: the real pipeline in ``.venv-audio`` as a persistent worker, fed the live dummy WAVs; gate
    listens are prefetched as soon as the listen landings are commanded, pod-mic listens run asynchronously, Gemini
    only on gate listens within a counted budget. ``python isaac/live.py --preflight`` fails loudly otherwise.

    python isaac/live.py --write-tasking default --out dashboard/data    # seed the dashboard's editable zones
    python isaac/live.py --cmd toggle --id H1 --out dashboard/data       # fallback trigger from a terminal
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
import time
import traceback

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

MANIFEST = os.path.join(REPO, "assets", "audio", "dummies", "manifest.json")
AUDIO_PY = os.path.join(REPO, ".venv-audio", "bin", "python")
AMBIENT_KINDS = ("dog", "generator", "settling")     # scenario sounds that are not people: kept from the scenario
KIND_ALIASES = {"shout": "voice", "speech": "voice", "scream": "voice", "call": "voice", "weak": "weak_voice",
                "weak_voice": "weak_voice", "moan": "weak_voice", "knocking": "knock", "tapping": "knock",
                "tap": "knock", "knock": "knock", "voice": "voice", "groan": "weak_voice", "cough": "weak_voice",
                "cry": "voice"}
NONHUMAN_KINDS = ("dog", "generator", "settling", "wind", "siren")
MOVE_EPS_M = 0.5                 # a dummy moved further than this is a world change
SIG_SNAP_M = 0.5                 # signature grid: same world within this
TAKE_GATE_LISTENS = 2            # rotors-off gate listens in a 60 s take (A2, then the redirected pile)
LISTEN_AFTER_DOWN_S = 66.0       # sim s from "all drones down" to the dispatch gate (26 s listen + call-out, seed 7)
GEMINI_BUDGET = 20               # Gemini calls per run (free tier ~20 requests / day / model); the pre-flight counts its own
QUICK_LISTEN_S = 20.0            # pod-mic quick listen cadence (sim s) while pods rest on the rubble
# pod-mic quick listens: listening window (s) of the real pipeline. The full rotors-off listen keeps the pipeline's
# own 26 s; a quick listen only has to catch a voice that started calling (the hidden dummies loop every ~9 s).
QUICK_LISTEN_DUR = float(os.environ.get("LARPVERSE_QUICK_LISTEN_DUR", "10"))
NEW_EVIDENCE_RISE = 15           # a pile's priority must rise this much over the most it was ever heard at
NEW_EVIDENCE_NEAR_M = 5.0        # ... and an earlier listen from no further than this extra must have heard it silent
NEW_EVIDENCE_RATIO = 1.6         # ... or from no further than 1.6x today's distance (~4 dB of spreading loss: a voice
                                 # heard at priority >= 50 from 12 m was not missed from 19 m; reh6b: idle drones landed
                                 # nearer A1 than the first listen's pods, so A1 never counted as new)
PREEMPT_MARGIN = 5               # ... and beat the current pile's priority by this to cut its radar phase short
QUICK_STRONG = 50                # one quick listen is enough when the new lead is this strong AND >= 2 drones' bearings
                                 # (50 = a voice-class call, human ~1.0; reh5: A1's new voice scored 56 once and waited
                                 # for a second listen that never came, while B3 44 heard twice triggered the re-plan)
                                 # agree on it (the real pipeline needs ~6 s per quick listen: two in a row outlast a dwell)
QUICK_PACE_CAP = 10.0            # take pace cap (sim s per video s) while the pod mics check a live change (real pipeline)
QUICK_PACE_TRIES = 3             # ... for at most this many quick listens submitted after the change
POLL_WALL_S = 1.0                # tasking / command-file / stage poll (wall seconds)
DEFAULT_KEYS = "U=hidden1,Y=hidden2"   # carb key -> dummy ("hiddenN" = N-th hidden dummy, sorted by id: grid H1_C3_vi,
                                       # H2_A1_id; collapse H1_RW_vi, H2_M39_id). H is Kit's hide key: not used.
DEFAULT_HIDDEN_CLIP = {"id": "hidden_voice_en", "kind": "voice", "lang": "en", "level_db": 0.0,
                       "text": "Help! We're down here, two of us, please hurry!"}
DEFAULT_HIDDEN_PHYSIO = "distressed_tachypnea"


def _say_default(m):
    print(f"[live] {m}", flush=True)


# =============================================================================== dummies -> victims / sounds
def norm_kind(kind):
    k = str(kind or "voice").lower()
    if k in NONHUMAN_KINDS:
        return k
    return KIND_ALIASES.get(k, "weak_voice" if "weak" in k else "knock" if ("knock" in k or "tap" in k) else "voice")


def physio_of(p):
    """``larpverse:physio`` (a state name, a JSON string or a dict) -> physio dict (sim/scenario.py convention)."""
    if isinstance(p, dict):
        return dict(p)
    if isinstance(p, str) and p.strip():
        s = p.strip()
        if s.startswith("{"):
            try:
                return dict(json.loads(s))
            except ValueError:
                return None
        from sim.scenario import PHYSIO_STATES
        if s in PHYSIO_STATES:
            return {"state": s, **{k: (lo + hi) / 2.0 for k, (lo, hi) in PHYSIO_STATES[s].items()}}
    return None


def load_manifest(path=MANIFEST):
    """Dummy-audio manifest (larpverse.dummy_audio/v1) -> ({key: clip profile}, [hidden dummy profiles]).
    Each clip is indexed by id, by its manifest ``file`` and by the repo-relative wav path the dummy prims carry in
    ``larpverse:clip``. Profile: {id, kind, text, text_en, lang, level_db, spl_db_1m, wav}. Missing -> ({}, [])."""
    try:
        with open(path) as f:
            doc = json.load(f)
    except (OSError, ValueError):
        return {}, []
    base = os.path.dirname(os.path.abspath(path))
    rel = os.path.relpath(base, REPO)
    raw = doc.get("clips", doc) if isinstance(doc, dict) else doc
    items = list(raw.items()) if isinstance(raw, dict) else list(enumerate(raw or []))
    clips = {}
    for key, e in items:
        if not isinstance(e, dict):
            continue
        cid = str(e.get("id") or e.get("clip") or key)
        f = e.get("file") or e.get("wav") or e.get("path") or f"{cid}.wav"
        spl = e.get("suggested_spl_db_1m")
        prof = {"id": cid, "kind": norm_kind(e.get("kind") or e.get("sound") or e.get("class")),
                "text": e.get("transcript") or e.get("text") or "", "text_en": e.get("transcript_en") or "",
                "lang": e.get("language") or e.get("lang"), "spl_db_1m": spl,
                "level_db": float(e["level_db"]) if e.get("level_db") is not None else level_from_spl(spl),
                "wav": f if os.path.isabs(f) else os.path.join(base, f)}
        for k in (cid, f, f"{rel}/{f}"):
            clips[k] = prof
    hidden = doc.get("hidden_dummies") or doc.get("hidden") or [] if isinstance(doc, dict) else []
    return clips, [h for h in hidden if isinstance(h, dict)]


def level_from_spl(spl):
    """Clip loudness (dB SPL at 1 m) -> the generators' level_db (scenario convention: a voice is about -2)."""
    if not spl:
        return -2.0
    return float(np.clip(float(spl) - 92.0, -12.0, 3.0))


def clip_profile(clip, manifest):
    if isinstance(clip, dict):
        return dict(clip, kind=norm_kind(clip.get("kind")))
    if not clip:
        return None
    stem = os.path.splitext(os.path.basename(str(clip)))[0]
    for k in (clip, stem):
        if k in manifest:
            return dict(manifest[k])
    return {"id": stem, "kind": norm_kind(stem), "text": "", "lang": None, "level_db": -2.0, "wav": None}


def pile_locator(scenario):
    """(x, y) -> sector name: inside a sector polygon if the site has them, else the nearest sector centre."""
    piles = list(scenario.piles)
    names = [p.name for p in piles]
    C = np.array([[float(p.x), float(p.y)] for p in piles])
    polys = [np.asarray(p.polygon, float) if getattr(p, "polygon", None) is not None and len(p.polygon) >= 3 else None
             for p in piles]

    def inside(P, x, y):
        xs, ys = P[:, 0], P[:, 1]
        xj, yj = np.roll(xs, 1), np.roll(ys, 1)
        hit = ((ys > y) != (yj > y)) & (x < (xj - xs) * (y - ys) / np.where(yj == ys, 1e-12, yj - ys) + xs)
        return bool(np.count_nonzero(hit) % 2)

    def locate(x, y):
        d = np.hypot(C[:, 0] - x, C[:, 1] - y)
        for k in np.argsort(d)[:6]:
            if polys[k] is not None and inside(polys[k], x, y):
                return names[k]
        return names[int(np.argmin(d))]
    return locate


def dummies_from_scenario(scenario, surface=None, hidden=None):
    """The seeded scenario as dummies (what isaac/dummies.py places on the stage), plus hidden, INACTIVE dummies.
    Used by --dry-run, tests and as the fallback when the stage has no dummies. ``hidden``: manifest hidden
    profiles; None -> one default hidden dummy in the empty sector nearest the start sector."""
    if hidden is None:
        try:                                     # the assets agent's plan = exactly what the stage will hold
            from isaac import dummies as DM
            man = DM.load_manifest()
            if man is not None:
                return [dict(e, xyz=[float(c) for c in e["xyz"]]) for e in DM.plan_dummies(scenario, man)]
        except ImportError:
            pass
    surface = surface or scenario.ground_height
    truth = scenario.truth
    snd = {(s["pile"], s["kind"]): s for s in truth.get("sounds", [])}
    out = []
    for v in truth.get("victims", []):
        x, y = (float(c) for c in v["xy"])
        z = float(v["z"]) if v.get("z") is not None else float(surface(x, y)) - float(v["depth"])
        clip = None
        if v.get("sound"):
            s = snd.get((v["pile"], v["sound"]), {})
            clip = {"id": f"scenario:{v['pile']}:{v['sound']}", "kind": v["sound"], "text": s.get("text", ""),
                    "level_db": float(s.get("level_db", -2.0)), "lang": s.get("lang")}
        out.append({"id": v["id"], "xyz": [x, y, z], "active": True, "clip": clip, "physio": v.get("physio"),
                    "breathing_hz": v.get("breathing_hz"), "hidden": False})
    taken = {v["pile"] for v in truth.get("victims", [])} | {s["pile"] for s in truth.get("sounds", [])}
    pxy = {p.name: (float(p.x), float(p.y)) for p in scenario.piles}
    sx, sy = pxy.get(truth.get("start_pile"), next(iter(pxy.values())))
    free = sorted((n for n in pxy if n not in taken), key=lambda n: (math.hypot(pxy[n][0] - sx, pxy[n][1] - sy), n))
    profs = list(hidden) if hidden else [dict(DEFAULT_HIDDEN_CLIP, physio=DEFAULT_HIDDEN_PHYSIO)]
    for k, prof in enumerate(profs):
        if prof.get("xyz") is not None:
            x, y, z = (float(c) for c in prof["xyz"])
        else:
            name = prof.get("pile") if prof.get("pile") in pxy else (free[k % len(free)] if free else list(pxy)[0])
            ox, oy = prof.get("offset", (1.5, -1.0))
            x, y = pxy[name][0] + ox, pxy[name][1] + oy
            z = float(surface(x, y)) - float(prof.get("depth", 0.9))
        clip = prof.get("clip") or {k2: prof[k2] for k2 in ("id", "kind", "text", "lang", "level_db", "wav") if k2 in prof}
        out.append({"id": prof.get("dummy_id") or f"H{k + 1}", "xyz": [x, y, z], "active": False, "clip": clip,
                    "physio": prof.get("physio") or DEFAULT_HIDDEN_PHYSIO, "hidden": True})
    return out


def world_from_dummies(dummies, locate, surface, manifest=None, probe=None, ref=None, ref_sounds=None, first_clip=None):
    """Active dummies -> (victims, sounds) in sim/scenario.py truth format (GENERATOR INPUT ONLY).

    ``ref`` {victim id: scenario victim} / ``ref_sounds`` {(pile, kind): scenario sound}: a dummy still where the
    scenario put it, with its first clip and no slab dragged over it, becomes exactly the scenario's victim /
    sound (depth below the planning surface, same level and transcript), so an untouched stage replays the CPU
    run bit for bit. A moved or newly active dummy gets its absolute stage z and its clip's profile. Non-human
    rows (dog, generator, settling) are sounds only."""
    victims, sounds = [], []
    for d in dummies:
        if not d.get("active", True):
            continue
        x, y, z = (float(c) for c in d["xyz"])
        base_top = float(surface(x, y))
        top = base_top
        if probe is not None:
            hz = probe(x, y)
            if hz is not None and hz > top + 0.3:
                top = float(hz)                    # dragged debris / a slab now over the dummy
        pile = locate(x, y)
        did = str(d["id"])
        same_clip = first_clip is None or first_clip.get(did) == d.get("clip")
        prof = clip_profile(d.get("clip"), manifest or {})
        kind = prof["kind"] if prof else None
        if d.get("human") is False or (kind in NONHUMAN_KINDS) or str(d.get("kind")) in NONHUMAN_KINDS:
            k = kind or norm_kind(d.get("kind"))
            rs = (ref_sounds or {}).get((d.get("pile") or pile, k))
            if rs is not None and same_clip and math.dist(rs["xy"], (x, y)) < MOVE_EPS_M + 1.5:
                sounds.append(dict(rs, xy=list(rs["xy"]), wav=(prof or {}).get("wav"), clip=(prof or {}).get("id"),
                                   dummy_id=did))
            elif k in AMBIENT_KINDS:
                sounds.append({"pile": pile, "kind": k, "xy": [x, y], "level_db": float(prof["level_db"] if prof else -2.0),
                               "clip": prof.get("id") if prof else None, "dummy_id": did})
            continue
        ph = physio_of(d.get("physio_json") or d.get("physio"))
        bhz = d.get("breathing_hz") or ((ph or {}).get("rate_bpm", 15.0) / 60.0)
        depth, zz, vid = max(0.0, top - z), z, did
        rv = (ref or {}).get(str(d.get("victim") or did))
        snd = None
        if rv is not None and top == base_top and math.dist(rv["xy"], (x, y)) < MOVE_EPS_M:
            rz = float(rv["z"]) if rv.get("z") is not None else base_top - float(rv["depth"])
            if abs(rz - z) < 0.3 or d.get("depth") is not None:
                depth, zz, vid, pile = float(rv["depth"]), rv.get("z"), rv["id"], rv["pile"]
                ph, bhz = rv.get("physio") or ph, rv.get("breathing_hz", bhz)
                if same_clip:
                    kind = rv.get("sound")
                    rs = (ref_sounds or {}).get((pile, kind)) if kind else None
                    snd = dict(rs, xy=list(rs["xy"])) if rs is not None else None
        victims.append({"id": vid, "pile": pile, "xy": [x, y], "depth": depth, "z": zz,
                        "breathing_hz": float(bhz), "sound": kind, "physio": ph, "dummy_id": did})
        if snd is not None:
            sounds.append(dict(snd, victim_id=vid, dummy_id=did, wav=(prof or {}).get("wav"),
                               clip=(prof or {}).get("id"), lang=(prof or {}).get("lang")))
        elif kind:
            sounds.append({"pile": pile, "kind": kind, "xy": [x, y], "level_db": float(prof.get("level_db", -2.0)),
                           "text": prof.get("text") or prof.get("text_en") or "", "text_en": prof.get("text_en") or "",
                           "lang": prof.get("lang"), "clip": prof.get("id"), "wav": prof.get("wav"),
                           "victim_id": vid, "dummy_id": did, "depth": depth})
    return victims, sounds


def signature(victims, sounds):
    snap = lambda v: int(round(float(v) / SIG_SNAP_M))   # noqa: E731
    vs = sorted((snap(v["xy"][0]), snap(v["xy"][1]), snap(v["depth"]), v.get("sound") or "") for v in victims)
    ss = sorted((s["pile"], s["kind"], snap(s["xy"][0]), snap(s["xy"][1]), (s.get("text") or "")[:40]) for s in sounds)
    return hashlib.sha1(json.dumps([vs, ss]).encode()).hexdigest()[:12]


# =============================================================================== dummy sources
class ListSource:
    """In-memory dummies (dry-run, tests, and the fallback when the stage has none)."""

    kind = "list"

    def __init__(self, dummies):
        self.dummies = [dict(d, xyz=list(d["xyz"])) for d in dummies]

    def read(self):
        return [dict(d, xyz=list(d["xyz"])) for d in self.dummies]

    def _get(self, did):
        return next((d for d in self.dummies if str(d["id"]) == str(did)), None)

    def set_active(self, did, on):
        d = self._get(did)
        if d is None:
            return False
        d["active"] = bool(on)
        return True

    def move(self, did, xyz):
        d = self._get(did)
        if d is None:
            return False
        d["xyz"] = [float(c) for c in xyz]
        return True

    def hidden_ids(self):
        hid = [str(d["id"]) for d in self.dummies if d.get("hidden")]
        return hid or [str(d["id"]) for d in self.dummies if not d.get("active", True)]


class StageSource:
    """Dummy prims on the Isaac stage via isaac.dummies (the assets agent's module). Writes go to the prim
    attributes ``larpverse:active`` / the translate op, exactly what a teammate's viewport edit would change."""

    kind = "stage"

    def __init__(self, stage, log=None):
        from isaac import dummies as _d          # ImportError until the assets agent lands it -> caller falls back
        self.stage, self.mod, self.say = stage, _d, log or _say_default
        self._first = None

    def read(self):
        out = []
        for d in self.mod.read_dummies(self.stage) or []:
            out.append(dict(d, id=str(d["id"]), xyz=[float(c) for c in d["xyz"]], active=bool(d.get("active", True))))
        if self._first is None:
            self._first = {d["id"]: d["active"] for d in out}
        return out

    def _prim(self, did):
        for d in self.mod.read_dummies(self.stage) or []:
            if str(d["id"]) == str(did) and (d.get("prim") or d.get("path")):
                return self.stage.GetPrimAtPath(d.get("prim") or d.get("path"))
        for prim in self.stage.Traverse():
            if prim.GetName() == str(did) and prim.HasAttribute("larpverse:clip"):
                return prim
        return None

    def set_active(self, did, on):
        fn = getattr(self.mod, "set_active", None) or getattr(self.mod, "set_dummy_active", None)
        if fn is not None:
            fn(self.stage, did, bool(on))
            return True
        prim = self._prim(did)
        if prim is None or not prim.IsValid():
            return False
        from pxr import Sdf
        attr = prim.GetAttribute("larpverse:active") or prim.CreateAttribute("larpverse:active", Sdf.ValueTypeNames.Bool)
        attr.Set(bool(on))
        return True

    def move(self, did, xyz):
        prim = self._prim(did)
        if prim is None or not prim.IsValid():
            return False
        from pxr import Gf, UsdGeom
        UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(*[float(c) for c in xyz]))
        return True

    def hidden_ids(self):
        rows = self.read()
        hid = [d["id"] for d in rows if d.get("hidden")]
        return hid or [k for k, on in (self._first or {}).items() if not on]


class SyncedSource:
    """Main-thread cache in front of a source, for ``--mission-thread``: the mission worker reads the last snapshot
    and queues writes; ``sync()`` (render loop, main thread, ~4 Hz) applies the writes to USD, re-reads the dummy
    prims and probes the PhysX column over each dummy. USD and PhysX are never touched off the main thread."""

    def __init__(self, inner, probe=None):
        import threading
        self.inner, self.probe = inner, probe
        self.kind = f"{inner.kind}+cache"
        self._lock = threading.Lock()
        self._ops, self._snap, self._cols = [], inner.read(), {}
        self._hidden = inner.hidden_ids()

    def sync(self):
        with self._lock:
            ops, self._ops = self._ops, []
        for op in ops:
            op()
        snap = self.inner.read()
        cols = {}
        if self.probe is not None:
            for d in snap:
                if d.get("active", True):
                    x, y = d["xyz"][:2]
                    cols[(round(x, 1), round(y, 1))] = self.probe(x, y)
        with self._lock:
            self._snap, self._cols = snap, cols

    def read(self):
        with self._lock:
            return [dict(d, xyz=list(d["xyz"])) for d in self._snap]

    def probe_cached(self, x, y):
        with self._lock:
            cols = dict(self._cols)
        best = min(cols, key=lambda k: math.hypot(k[0] - x, k[1] - y), default=None)
        if best is None or math.hypot(best[0] - x, best[1] - y) > 0.6:
            return None
        return cols[best]

    def set_active(self, did, on):
        if not any(str(d["id"]) == str(did) for d in self.read()):
            return False
        with self._lock:
            self._ops.append(lambda: self.inner.set_active(did, on))
        return True

    def move(self, did, xyz):
        with self._lock:
            self._ops.append(lambda: self.inner.move(did, xyz))
        return True

    def hidden_ids(self):
        return list(self._hidden)


def make_source(stage, scenario, surface=None, manifest_hidden=None, log=None):
    """Stage dummies when isaac.dummies is importable and the stage has some; else the scenario list (+ hidden)."""
    say = log or _say_default
    if stage is not None:
        try:
            src = StageSource(stage, say)
            ds = src.read()
            if ds:
                say(f"live world: {len(ds)} dummy prims on the stage ({sum(d['active'] for d in ds)} active, "
                    f"hidden {src.hidden_ids()})")
                return src
            say("live world: isaac.dummies found no dummy prims; using the scenario list + hidden dummies")
        except ImportError:
            say("live world: isaac/dummies.py not landed yet; using the scenario list + hidden dummies "
                "(hotkey / command-file triggers still work, viewport drags do not)")
        except Exception as e:                          # noqa: BLE001
            say(f"live world: reading stage dummies failed ({type(e).__name__}: {e}); using the scenario list")
    return ListSource(dummies_from_scenario(scenario, surface, manifest_hidden))


# =============================================================================== the live world
class LiveScenario:
    """The mission's scenario with ``truth`` served live from the stage (generators only read it)."""

    def __init__(self, base, live):
        object.__setattr__(self, "_base", base)
        object.__setattr__(self, "_live", live)

    @property
    def truth(self):
        return self._live.truth()

    def __getattr__(self, k):
        return getattr(self._base, k)


class LiveWorld:
    def __init__(self, scenario, source, surface=None, manifest=None, probe=None, log=None):
        self.base, self.source = scenario, source
        self.surface = surface or scenario.ground_height
        self.manifest = manifest if manifest is not None else load_manifest()[0]
        self.probe = probe
        self.say = log or _say_default
        self.locate = pile_locator(scenario)
        self.ambient = [dict(s) for s in scenario.truth.get("sounds", []) if s["kind"] in AMBIENT_KINDS]
        self.ref = {str(v["id"]): v for v in scenario.truth.get("victims", [])}
        self.ref_sounds = {(s["pile"], s["kind"]): s for s in scenario.truth.get("sounds", [])}
        rv, rs = scenario.truth.get("victims", []), scenario.truth.get("sounds", [])
        self.reference_sig = signature([dict(v, z=v.get("z")) for v in rv], rs)
        self.first_clip = None
        self.scenario = LiveScenario(scenario, self)
        import threading
        self._lock = threading.RLock()
        self.events, self.reads, self.pending_cause = [], 0, {}
        self.stage_ambient = False
        self.signatures = True         # whole-scan vitals.identify + displacement per scan for the HUD radar panel
        self.prefetch = {}             # listen_key -> acoustic job submitted when the drones landed (real pipeline)
        self.waiting = None            # {"job", "since"} while a listen waits for the pipeline (HUD)
        self.gemini_ok = None          # controller hook: may this listen use the Gemini stage (budget)
        self.last_scan, self.last_listen = None, None
        self.heard = {}                # pile -> {t, language, original, english, asr}: words heard (multilingual ASR, HUD)
        self._prev, self._victims, self._sounds, self.sig = None, [], [], None
        self.t = 0.0
        self.refresh(0.0)

    def note_heard(self, res, t=None, how="listen"):
        """Keep the original-language words of each transcribed clip (the state.json clip carries the English
        transcript and the language only) so the panel can show both. Display only."""
        for c in res.get("clips", []):
            if c.get("transcript") and c.get("pile_id"):
                self.heard[c["pile_id"]] = {"t": round(self.t if t is None else t, 1), "how": how,
                                            "language": c.get("language"), "original": c.get("transcript_original"),
                                            "english": c["transcript"] if c.get("translated", True) else None,
                                            "asr": c.get("asr_model")}

    # ------------------------------------------------------------------ reading the stage
    def refresh(self, t=None, why=None):
        """Re-read the dummies; record activations / moves as events (display + cache freshness only).
        Thread-safe: the render thread (after a stage sync) and the mission worker may both call it."""
        with self._lock:
            return self._refresh(t, why)

    def _refresh(self, t=None, why=None):
        if t is not None:
            self.t = float(t)
        ds = self.source.read()
        self.reads += 1
        cur = {str(d["id"]): d for d in ds}
        if self._prev is not None:
            for did, d in cur.items():
                p = self._prev.get(did)
                cause = self.pending_cause.pop(did, None)
                if p is None:
                    self._event("added", d, cause or "stage edit")
                elif bool(p.get("active", True)) != bool(d.get("active", True)):
                    self._event("activated" if d.get("active", True) else "deactivated", d, cause or "stage edit")
                elif math.dist(p["xyz"], d["xyz"]) > MOVE_EPS_M:
                    self._event("moved", d, cause or "viewport drag", frm=p["xyz"])
                elif p.get("clip") != d.get("clip"):
                    self._event("clip changed", d, cause or "stage edit")
            for did in set(self._prev) - set(cur):
                self._event("removed", self._prev[did], "stage edit")
        self._prev = {k: dict(v, xyz=list(v["xyz"])) for k, v in cur.items()}
        if self.first_clip is None:
            self.first_clip = {str(d["id"]): d.get("clip") for d in ds}
            self.stage_ambient = any(d.get("human") is False for d in ds)
        self._victims, self._sounds = world_from_dummies(ds, self.locate, self.surface, self.manifest, self.probe,
                                                         ref=self.ref, ref_sounds=self.ref_sounds,
                                                         first_clip=self.first_clip)
        self.sig = signature(self._victims, self._sounds + ([] if self.stage_ambient else self.ambient))   # = truth()
        return self

    def _event(self, what, d, cause, frm=None):
        x, y, z = d["xyz"]
        ev = {"t": round(self.t, 1), "wall": round(time.time(), 2), "what": what, "id": str(d["id"]),
              "xyz": [round(x, 2), round(y, 2), round(z, 2)], "sector": self.locate(x, y), "cause": cause}
        if frm is not None:
            ev["from"] = [round(c, 2) for c in frm]
        self.events.append(ev)
        self.say(f"STAGE EVENT t={ev['t']}: dummy {ev['id']} {what} near {ev['sector']} ({cause}); "
                 "the generators see it from the next scan / listen")

    @property
    def changed(self):
        """True when the stage world differs from the seeded scenario: every listen is then synthesised fresh."""
        return self.sig != self.reference_sig

    def truth(self):
        return {"start_pile": self.base.truth.get("start_pile"), "roles": self.base.truth.get("roles"),
                "victims": [dict(v, xy=list(v["xy"])) for v in self._victims],
                "sounds": [dict(s, xy=list(s["xy"])) for s in self._sounds]
                + ([] if self.stage_ambient else [dict(s) for s in self.ambient])}

    def targets_for(self, pile, sensor_pose=None):
        """GridWorld-format radar targets in ``pile`` (nearest to the sensor first: the generator models one)."""
        ts = [{"id": v["id"], "kind": "buried", "pile": v["pile"], "xy": list(v["xy"]), "depth": v["depth"],
               "breathing_hz": v["breathing_hz"], "emits_sound": v["sound"] is not None, "z": v.get("z"),
               "physio": v["physio"]} for v in self._victims if v["pile"] == pile]
        if sensor_pose is not None and len(ts) > 1:
            ts.sort(key=lambda t: math.hypot(t["xy"][0] - sensor_pose[0], t["xy"][1] - sensor_pose[1]))
        return ts

    def patch_grid(self, grid, targets):
        """Planning-window copy with the PhysX column height over each target (a dragged slab adds rubble)."""
        if self.probe is None or not targets:
            return grid
        from sim.world import Grid
        z = None
        for t in targets:
            x, y = t["xy"]
            hz = self.probe(x, y)
            if hz is None or hz <= float(grid.height_at(x, y)) + 0.3:
                continue
            if z is None:
                z = grid.z.copy()
            i, j = (int(v) for v in grid.index(x, y))
            r = max(1, int(round(1.2 / grid.res)))
            sl = (slice(max(j - r, 0), j + r + 1), slice(max(i - r, 0), i + r + 1))
            z[sl] = np.maximum(z[sl], hz)
        return grid if z is None else Grid(grid.origin, grid.res, grid.w, grid.h, z)

    # ------------------------------------------------------------------ triggers
    def set_active(self, did, on, cause):
        self.pending_cause[str(did)] = cause
        ok = self.source.set_active(did, on)
        if not ok:
            self.pending_cause.pop(str(did), None)
            self.say(f"trigger ({cause}): no dummy {did!r}")
        return ok

    def toggle(self, did, cause):
        cur = {str(d["id"]): d for d in self.source.read()}.get(str(did))
        if cur is None:
            self.say(f"trigger ({cause}): no dummy {did!r}")
            return False
        return self.set_active(did, not cur.get("active", True), cause)

    def move(self, did, xyz, cause):
        self.pending_cause[str(did)] = cause
        return self.source.move(did, xyz)

    def note_scan(self, pile, drone_id, scan_idx, t, obs, result, sig=None, site=None):
        dbg = getattr(result, "debug", None) or {}
        st = obs[0] if obs else {}
        rec = {"pile": pile, "drone_id": drone_id, "scan_id": st.get("scan_id", f"s{scan_idx}"),
               "t": round(float(t), 1), "quality": st.get("quality"), "breathing_hz": st.get("breathing_hz"),
               "ranges": st.get("range_candidates"), "vitals": (st.get("vitals") or [None])[0],
               "pod": bool((site or {}).get("pod")), "sig": sig,
               "wave": _thin(dbg.get("breathing_waveform"), 160),
               "range_axis": _thin(dbg.get("range_axis"), 120),
               "range_db": _thin(dbg.get("range_profile_db"), 120)}
        # several pods finish together: keep the most telling one on screen (a person-like return first)
        prev = self.last_scan
        if prev is not None and prev["t"] == rec["t"] and _sig_rank(prev) > _sig_rank(rec):
            return
        self.last_scan = rec


def _sig_rank(rec):
    f = ((rec or {}).get("sig") or {}).get("final") or {}
    return (f.get("source") == "human_breathing", f.get("source") not in (None, "no_signal"), f.get("conf") or 0.0)


def _thin(a, n):
    if a is None:
        return None
    a = np.asarray(a, float).ravel()
    if a.size == 0:
        return None
    idx = np.linspace(0, a.size - 1, min(n, a.size)).astype(int)
    return [round(float(v), 4) for v in a[idx]]


def scan_signature(iq, params, res=None, fps=10.0):
    """The pod scan's own signature for the HUD (same route as scripts/build_radar_signatures.py hud_stream, whole
    scan only): vitals.identify -> which return / is it breathing, then the motion-compensated displacement of that
    bin at ``fps``. Early partial verdicts can flip (docs/RADAR_SIGNATURES.md), so only the whole-dwell verdict is
    kept; the HUD shows it after VERDICT_AFTER_S of dwell."""
    from sim import vitals as V
    o = V.identify(iq, params, res=res)
    v = V.extract(V.motion_compensated(iq, params)[:, o["bin"]], params.fs_slow, kind="iq",
                  wavelength=params.wavelength, range_m=o["range_m"], rubble_path_m=o["range_m"], params=params,
                  keep_arrays=True)
    d = np.asarray(v["_displacement"], float)[::max(1, int(round(params.fs_slow / fps)))]
    d = 1e3 * (d - np.median(d)) if d.size else d
    ov = o.get("vitals") or {}
    return {"fps": fps, "dwell_s": round(float(params.dwell), 1), "range_m": round(float(o["range_m"]), 2),
            "wave_mm": [round(float(x), 3) for x in d],
            "final": {"source": o["source"], "condition": o["condition"], "conf": round(float(o["confidence"]), 2),
                      "text": ov.get("condition_text") or o.get("reason") or "", "hint": ov.get("triage_hint"),
                      "rate_bpm": ov.get("rate_bpm")}}


# =============================================================================== generators wired to the live world
class LiveRadar:
    """``sensor_factory`` product: each scan rebuilds the radar generator on this pile's CURRENT targets."""

    def __init__(self, pw, seed, radar, live):
        self.pw, self.seed, self.radar, self.live = pw, seed, radar, live
        self.last_result = None

    def _gen(self, sensor_pose=None):
        from sim.grid_world import PileWorld
        from sim.radar_model import make_generator
        tg = self.live.targets_for(self.pw.name, sensor_pose)
        return make_generator(PileWorld(self.pw.name, self.live.patch_grid(self.pw.grid, tg), tg), self.seed, self.radar)

    def radar_scan(self, scan_idx, drone_id, sensor_pose, site, t, dwell=None):
        self.live.refresh(t, "radar scan")
        gen = self._gen(sensor_pose)
        import sim.radar_model as RM
        cap = {}
        real_sim = RM.radar_dsp.simulate_if

        def capture(scene, params, rng, *a, **k):           # keep this scan's IF cube for the HUD signature
            iq = real_sim(scene, params, rng, *a, **k)
            cap["iq"], cap["params"] = iq, params
            return iq
        RM.radar_dsp.simulate_if = capture
        try:
            obs = gen.radar_scan(scan_idx, drone_id, sensor_pose, site, t, **({} if dwell is None else {"dwell": dwell}))
        finally:
            RM.radar_dsp.simulate_if = real_sim
        self.last_result = getattr(gen, "last_result", None)
        sig = None
        if cap and self.live.signatures:
            try:
                sig = scan_signature(cap["iq"], cap["params"], res=self.last_result)
            except Exception as e:                           # noqa: BLE001  (never let the HUD stall the mission)
                self.live.say(f"scan signature failed ({type(e).__name__}: {e})")
        self.live.note_scan(self.pw.name, drone_id, scan_idx, t, obs, self.last_result, sig=sig, site=site)
        return obs

    def __getattr__(self, k):              # legacy single-pile calls (acoustic_listen ...) on the current targets
        if k.startswith("_") or k in ("pw", "live", "seed", "radar"):
            raise AttributeError(k)
        return getattr(self._gen(), k)


_SUBPROC = r"""
import json, sys
sys.path.insert(0, %r)
from sim.scenario import make_scenario
from acoustic.run import listen_and_rank
a = json.load(sys.stdin)
sc = make_scenario(a["scenario_seed"], a["start_pile"], **({"site": a["site"]} if a.get("site") not in (None, "grid") else {}))
sc.truth = a["truth"]                      # the live stage sources (generator input only)
r = listen_and_rank(sc, {k: tuple(v) for k, v in a["drones"].items()}, a["exclude"], a["seed"])
sys.stdout.write("\n@@ACOUSTIC_JSON@@" + json.dumps(r))
"""


def find_prefetch(prefetch, drones, exclude, seed, sig, tol_m=1.5):
    """Job id of a prefetched listen for the same world, exclusions and seed with every drone within ``tol_m``."""
    for key, (jid, pos) in prefetch.items():
        if key[1:] != (tuple(sorted(set(exclude))), int(seed), sig) or set(pos) != set(drones):
            continue
        if all(math.dist(pos[k], drones[k][:2]) <= tol_m for k in drones):
            return jid
    return None


def listen_key(drones, exclude, seed, sig):
    d = tuple(sorted((k, int(round(float(v[0]))), int(round(float(v[1])))) for k, v in drones.items()))
    return (d, tuple(sorted(set(exclude))), int(seed), sig)


def _fresh_real(sc, drones, exclude, seed, allow_subprocess):
    try:
        from acoustic.run import listen_and_rank as real
        return real(sc, drones, list(exclude), seed), "real-live"
    except Exception as e:                                  # noqa: BLE001  (deps live in .venv-audio)
        err = f"{type(e).__name__}: {e}"
    if allow_subprocess and os.path.exists(AUDIO_PY):
        payload = json.dumps({"scenario_seed": sc.seed, "start_pile": sc.public_dict()["start_pile"],
                              "site": getattr(sc, "site", "grid"), "truth": sc.truth,
                              "drones": {k: list(v) for k, v in drones.items()}, "exclude": list(exclude), "seed": seed})
        p = subprocess.run(["nice", "-n", "19", AUDIO_PY, "-c", _SUBPROC % REPO], input=payload,
                           capture_output=True, text=True, cwd=REPO, timeout=900)
        if p.returncode == 0 and "@@ACOUSTIC_JSON@@" in p.stdout:
            return json.loads(p.stdout.rsplit("@@ACOUSTIC_JSON@@", 1)[1]), "real-venv-live"
        err = f"subprocess {p.returncode}: {p.stderr.strip()[-200:]}"
    raise RuntimeError(err)


# =============================================================================== real acoustic pipeline (the take)
_SERVER = r"""
import inspect, json, sys, time, traceback
sys.path.insert(0, %r)
from sim.scenario import make_scenario
import acoustic.run as AR
import acoustic.scene as AS
LR = AR.listen_and_rank
HAS_SRC = "sources" in inspect.signature(LR).parameters
SCENE_SRC = "sources" in inspect.signature(AS.build_scene).parameters
HAS_DUR = "duration" in inspect.signature(LR).parameters
cache = {}
# --- multilingual speech-to-text (request "asr": "multi"). The team's per-clip recogniser forces English
# (base.en, language="en"), which turns the Indonesian / Vietnamese voices into English-sounding nonsense
# (2026-11-08 rehearsal 4a: "Oh long, other young bees are better..."). Whisper's own language id fails on these
# TTS voices too (eSpeak Indonesian -> zh / th / hu / km at p 0.16-0.79, 2026-11-08 GPU diagnostic), but decoding
# WITH the right language reads them ("Ada yang bisa dengar saya, kaki saya terjepit" from the separated A1 clip).
# "multi" therefore swaps only the speech-to-text step of acoustic.run.recognise for a constrained language id: the
# clip is decoded once per language a Taiwan rescue team plans for (LARPVERSE_ASR_LANGS, default Mandarin,
# English, Indonesian, Vietnamese) and the best average log-probability wins; then Whisper translates FROM that
# language to English (with the source language forced; auto-detected translation hallucinated every time).
# Whisper's own keep rule (no_speech_prob < 0.6, avg_logprob > -1.0) stands, except that a separated clip the VAD
# calls speech (>= 0.8) keeps a confident decode even when Whisper's no-speech score is high (separation residue).
# The English text is the clip transcript the judge reads (its distress keywords are English); the language and
# the words as spoken ride along for the panel. VAD, YAMNet, the transcript gates and every score / ranking rule
# are the team's, unchanged.
import os, re
ORIG_RECOGNISE, ORIG_CF = AR.recognise, AR.clip_features
ASR = {"mode": "en", "last": None, "by_clip": {}, "cid": None}
ASR_DEBUG = os.environ.get("LARPVERSE_ASR_DEBUG")   # a .jsonl path: every decode's raw segments (tuning only)
RESCUE_LANGS = [x.strip() for x in os.environ.get("LARPVERSE_ASR_LANGS", "zh,en,id,vi").split(",") if x.strip()]
# tuned on the separated seed-7 clips (GPU, 2026-11-08 04:05, Whisper small, beam 5): with these settings only 'id'
# survives on the hidden Indonesian voice, 'en' on B1's English, and nothing on A3's unclear voice (every language
# decoded to garbage there, e.g. "Kill me, kill me..."); without them 'vi' won with a YouTube outro at lp -0.39
ASR_LP_MIN, ASR_NSP_MAX, ASR_VAD_SPEECH = -0.9, 0.6, 0.8
ASR_CR_MAX, ASR_MARGIN, ASR_MIN_S = 2.0, 0.10, 0.5
# Whisper's stock hallucinations on noise / music (subtitle credits, video outros); never shown as words
ASR_JUNK = re.compile(r"thank(s| you) for watching|subscri|like and share|see you (in the )?next|subtitles?|"
                      r"captions?|amara|transcri|copyright|all rights|\bmusic\b|applause|i'?ll be back|just kidding|"
                      r"\bvideo|\bchannel|kênh|hẹn gặp lại|留言|訂閱|订阅|下方|字幕|頻道|频道|"
                      # the same outros in Malay / Indonesian, Vietnamese, Chinese (reh6: B3 "Terima kasih kerana
                      # menonton!" = "thank you for watching" passed as Indonesian speech)
                      r"terima ?kasih (kerana |karena |telah |sudah |untuk |udah )?(menonton|nonton)|menonton|"
                      r"jangan lupa|sampai jumpa|selamat menonton|cảm ơn (các )?bạn đã (xem|theo dõi)|đăng ký|"
                      r"谢谢观看|謝謝觀看|感谢观看|感謝觀看|感谢收看|感謝收看|点赞|點贊|點讚|"
                      r"that'?s all for (today|now)|that is all,? sir", re.I)
def _looping(text):                                # repetition loop: few distinct words / characters
    t = text.strip()
    units = t.split() if len(t.split()) >= 4 else [c for c in t if not c.isspace()]
    return len(units) >= 6 and len(set(units)) / len(units) < 0.5
def _decode(wm, xn, lang, vad, task="transcribe"):
    segs = wm.transcribe(xn, language=lang, task=task, beam_size=5, temperature=[0.0, 0.2, 0.4],
                         compression_ratio_threshold=2.4, log_prob_threshold=-1.0,
                         no_repeat_ngram_size=3, condition_on_previous_text=False, vad_filter=False)[0]
    segs = list(segs)
    if ASR_DEBUG:
        try:
            with open(ASR_DEBUG, "a") as f:
                f.write(json.dumps({"clip": ASR["cid"], "lang": lang, "task": task, "vad": round(float(vad), 2),
                                    "segs": [[sg.text.strip()[:80], round(sg.avg_logprob, 2), round(sg.no_speech_prob, 2),
                                              round(sg.compression_ratio, 2), round(sg.start, 1), round(sg.end, 1)]
                                             for sg in segs]}, ensure_ascii=False) + "\n")
        except OSError:
            pass
    keep = []
    for sg in segs:
        txt = sg.text.strip()
        if not txt or sg.compression_ratio >= ASR_CR_MAX or _looping(txt) or ASR_JUNK.search(txt):
            continue
        if sg.avg_logprob <= ASR_LP_MIN:
            continue
        if sg.no_speech_prob >= ASR_NSP_MAX and vad < ASR_VAD_SPEECH:      # separation residue: VAD decides
            continue
        keep.append(sg)
    w = [max(sg.end - sg.start, 0.1) for sg in keep]
    if not keep or sum(w) < ASR_MIN_S:
        return None
    text = re.sub(r"\s+", " ", " ".join(sg.text.strip() for sg in keep)).strip()
    if _looping(text):
        return None
    return {"lang": lang, "lp": sum(sg.avg_logprob * k for sg, k in zip(keep, w)) / sum(w),
            "nsp": sum(sg.no_speech_prob * k for sg, k in zip(keep, w)) / sum(w), "text": text}
def asr_rescue(clip, vad):
    # constrained language id (see above). Honest beats wrong: no words unless one language clearly wins (ASR_MARGIN
    # in average log-probability over the runner-up) on clean, non-looping, non-stock segments
    from acoustic import gemini_audio as GA
    import numpy as np
    name = os.environ.get("LARPVERSE_ASR_MODEL") or GA.multilingual_whisper()
    if not name:                                     # no multilingual model on disk: the team's English path
        return None
    wm = GA._whisper(name)
    M = AR.models
    x = np.asarray(clip, np.float32)
    if hasattr(M, "_asr_window"):
        x = M._asr_window(x)
    xn = M._norm(x, 0.1)
    tries = sorted([d for d in (_decode(wm, xn, lg, vad) for lg in RESCUE_LANGS) if d], key=lambda d: -d["lp"])
    rec = {"tries": {d["lang"]: round(d["lp"], 2) for d in tries}, "asr": f"{name} (constrained language id: "
           f"{','.join(RESCUE_LANGS)})"}
    if not tries or (len(tries) > 1 and tries[0]["lp"] - tries[1]["lp"] < ASR_MARGIN):
        return dict(rec, language=None, lang_p=None, original=None, english=None, unclear=True)
    best = tries[0]
    en = best["text"] if best["lang"] == "en" else None
    if en is None:
        tr = _decode(wm, xn, best["lang"], vad, task="translate")
        en = tr["text"] if tr else None
        if en is None:                   # the translation was dropped (junk / loop / unsure): so are the words
            return dict(rec, language=None, lang_p=None, original=None, english=None, unclear=True)
    return dict(rec, language=best["lang"], lang_p=round(float(best["lp"]), 2), original=best["text"], english=en,
                unclear=False)
def recognise_multi(clip, whisper="base.en"):
    vad = AR.models.vad_speech_ratio(clip)
    cls, scores, heur, top = AR.models.classify(clip, vad)
    raw, ASR["last"] = "", None
    if AR.wants_asr(vad, cls, heur):
        x = asr_rescue(clip, vad)
        if x and not x.get("unclear"):
            raw = x["english"] or x["original"]
            ASR["last"] = x
    text, why = AR.gate_transcript(raw, vad, cls, heur)
    if not text:
        ASR["last"] = None
    return {"vad": vad, "cls": cls, "scores": scores, "heur": heur, "top": top, "text": text, "why": why}
def clip_features_multi(cid, *a, **k):
    ASR["last"], ASR["cid"] = None, cid
    f = ORIG_CF(cid, *a, **k)
    if ASR["last"] and f.get("transcript"):
        ASR["by_clip"][cid] = ASR["last"]       # analyse() keeps a fixed key list per clip: merged after the listen
    return f
def merge_asr(r):
    for c in r.get("clips", []):
        x = ASR["by_clip"].get(c.get("clip_id"))
        if x and c.get("transcript"):
            c.update(language=x["language"], language_p=x["lang_p"], transcript_original=x["original"],
                     asr_model=x["asr"], asr_tries=x.get("tries"), translated=bool(x.get("english")))
def set_asr(mode):
    mode = mode or "en"
    if mode == ASR["mode"]:
        return
    AR.recognise, AR.clip_features = (recognise_multi, clip_features_multi) if mode == "multi" else (ORIG_RECOGNISE, ORIG_CF)
    ASR["mode"] = mode
def scenario(a):
    k = (a["scenario_seed"], a["start_pile"], a.get("site") or "grid")
    if k not in cache:
        cache[k] = make_scenario(k[0], k[1], **({"site": k[2]} if k[2] != "grid" else {}))
    return cache[k]
print(json.dumps({"ready": True, "sources_api": HAS_SRC, "scene_sources": SCENE_SRC}), flush=True)
for line in sys.stdin:
    a = json.loads(line)
    t0 = time.time()
    try:
        sc = scenario(a)
        sc.truth = a["truth"]                         # the live stage sources (generator input only)
        kw = {"gemini_audio": a.get("gemini")} if "gemini_audio" in inspect.signature(LR).parameters else {}
        src = a.get("sources")
        used = "truth"
        set_asr(a.get("asr"))
        ASR["by_clip"] = {}
        dur = a.get("duration")                         # quick pod-mic listens: a shorter window than the full listen
        if src and HAS_SRC:
            kw["sources"], used = src, "sources"
        orig = AS.build_scene
        extra = {}
        if src and not HAS_SRC and SCENE_SRC:           # listen_and_rank without sources=: feed build_scene directly
            extra["sources"] = src
            used = "sources(build_scene)"
        if dur and HAS_DUR:
            kw["duration"] = float(dur)
        elif dur:
            extra["duration"] = float(dur)
        if extra:
            def bs(*x, **k):
                return orig(*x, **{**k, **extra})
            AS.build_scene = bs
            if hasattr(AR, "build_scene"):
                AR.build_scene = bs
        try:
            r = LR(sc, {k: tuple(v) for k, v in a["drones"].items()}, a["exclude"], a["seed"], **kw)
        finally:
            AS.build_scene = orig
            if hasattr(AR, "build_scene"):
                AR.build_scene = orig
        merge_asr(r)
        out = {"id": a["id"], "result": r, "elapsed_s": round(time.time() - t0, 2), "fed": used,
               "asr": ASR["mode"], "duration_s": dur}
    except Exception as e:
        out = {"id": a["id"], "error": f"{type(e).__name__}: {e}", "trace": traceback.format_exc()[-1500:]}
    sys.stdout.write(json.dumps(out) + "\n")
    sys.stdout.flush()
"""


def llm_calls(res):
    """Network LLM calls a listen made (acoustic agent triage + Gemini audio stage), cache hits excluded."""
    ag = (res or {}).get("agent") or {}
    n = int(ag.get("llm_calls") or 0)
    for k, v in ag.items():
        if isinstance(v, dict) and ("gemini" in k or k.startswith("stage")):
            n += max(0, int(v.get("calls") or 0) - int(v.get("cached") or 0))
    return n


def _sources_payload(truth, window=None):
    """Live sounds -> acoustic.scene.source_track dicts (dummy WAVs already carry their rubble loss). None if any
    sound has no clip (then the pipeline synthesises from the truth dicts instead). ``window``: a listening window
    shorter than the pipeline's 26 s squeezes the sources' start stagger into it (a caller loops regardless of when
    the listen starts; the profile of 2026-11-08 heard nothing from A1 in 10 s windows because its stagger started
    it at 9 s)."""
    k_t = min(1.0, float(window) / 26.0) if window else 1.0
    out = []
    for k, snd in enumerate(truth.get("sounds", [])):
        wav = snd.get("wav")
        if not wav or not os.path.exists(wav):
            return None
        out.append({"xy": [float(v) for v in snd["xy"]], "clip": wav, "kind": snd["kind"],
                    "level_db": 0.0, "depth_m": 0.0, "loop": True, "gap_s": 4.0 + (k % 3),
                    "t0_s": 0.5 + 1.7 * k * k_t, "id": snd.get("dummy_id") or snd.get("victim_id")})
    return out


def audio_env(py=AUDIO_PY):
    """Environment for the .venv-audio worker: the pip CUDA wheels' libraries (nvidia-cublas / cudnn / nvrtc) on
    LD_LIBRARY_PATH, so LARPVERSE_WHISPER_DEVICE=cuda finds libcublas.so.12 (2026-11-08 rehearsal 4b: it did not,
    and Whisper fell back to the CPU)."""
    import glob
    env = dict(os.environ)
    venv = os.path.dirname(os.path.dirname(py))
    libs = sorted(glob.glob(os.path.join(venv, "lib", "python3*", "site-packages", "nvidia", "*", "lib")))
    if libs:
        env["LD_LIBRARY_PATH"] = os.pathsep.join(libs + [p for p in [env.get("LD_LIBRARY_PATH", "")] if p])
    return env


class AcousticService:
    """The REAL acoustic pipeline (acoustic.run.listen_and_rank in ``.venv-audio``) as a persistent worker process:
    models load once, listens are submitted asynchronously (prefetched as soon as the drones are down) and fed the
    live dummy sources. Same machine as Isaac: nothing crosses the network during the take."""

    def __init__(self, scenario, python=AUDIO_PY, log=None, gemini=False, asr=None):
        import threading
        self.sc, self.py, self.say, self.gemini = scenario, python, log or _say_default, bool(gemini)
        self.asr = asr or os.environ.get("LARPVERSE_ASR", "en")      # "multi": multilingual speech-to-text (worker)
        self.proc, self.ready, self.info, self.err = None, False, {}, None
        self.results, self.n = {}, 0
        self.llm_calls = 0                   # network LLM calls made by the pipeline this run (Gemini budget)
        self._lock, self._cv = threading.Lock(), threading.Condition()
        self.latency = []

    def start(self, timeout=900.0):
        import threading
        if not os.path.exists(self.py):
            raise RuntimeError(f"{self.py} not found: run acoustic/setup.sh on this box (docs/VIDEO_PLAN.md section 6)")
        self.proc = subprocess.Popen(["nice", "-n", "5", self.py, "-u", "-c", _SERVER % REPO], cwd=REPO, text=True,
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=1,
                                     env=audio_env(self.py))
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._drain_err, daemon=True).start()
        t0 = time.time()
        with self._cv:
            while not self.ready and self.err is None and time.time() - t0 < timeout:
                self._cv.wait(1.0)
        if not self.ready:
            raise RuntimeError(f"acoustic worker did not start: {self.err or 'timeout'}")
        return self

    def _read(self):
        for line in self.proc.stdout:
            try:
                msg = json.loads(line)
            except ValueError:
                continue                                 # a library printed to stdout
            with self._cv:
                if msg.get("ready"):
                    self.ready, self.info = True, msg
                elif "id" in msg:
                    self.results[msg["id"]] = msg
                    if "elapsed_s" in msg:
                        self.latency.append(msg["elapsed_s"])
                    n = llm_calls(msg.get("result"))
                    if n:
                        self.llm_calls += n
                        self.say(f"Gemini calls: +{n} (listen {msg['id']}), {self.llm_calls} this run")
                self._cv.notify_all()
        with self._cv:
            self.err = self.err or f"worker exited ({self.proc.poll()})"
            self._cv.notify_all()

    def _drain_err(self):
        for line in self.proc.stderr:
            if "Traceback" in line or "Error" in line:
                self.say(f"[acoustic worker] {line.rstrip()[:200]}")

    def submit(self, truth, drones, exclude, seed, gemini=None, duration=None):
        """duration: listening window in s (None = the pipeline's own, acoustic.scene.DURATION)."""
        with self._lock:
            self.n += 1
            jid = self.n
        src = _sources_payload(truth, duration)
        req = {"id": jid, "scenario_seed": self.sc.seed, "start_pile": self.sc.public_dict()["start_pile"],
               "site": getattr(self.sc, "site", "grid"), "truth": truth, "sources": src,
               "drones": {k: [float(v[0]), float(v[1])] for k, v in drones.items()}, "exclude": list(exclude),
               "seed": int(seed), "gemini": self.gemini if gemini is None else bool(gemini), "asr": self.asr,
               "duration": duration}
        self.proc.stdin.write(json.dumps(req) + "\n")
        self.proc.stdin.flush()
        return jid

    def poll(self, jid):
        with self._cv:
            return self.results.get(jid)

    def wait(self, jid, timeout=600.0):
        t0 = time.time()
        with self._cv:
            while jid not in self.results and self.err is None and time.time() - t0 < timeout:
                self._cv.wait(0.5)
            r = self.results.get(jid)
        if r is None:
            raise RuntimeError(f"acoustic worker: no result for listen {jid} ({self.err or 'timeout'})")
        if "error" in r:
            raise RuntimeError(f"acoustic worker listen {jid}: {r['error']}")
        return r

    def close(self):
        if self.proc is not None and self.proc.poll() is None:
            try:
                self.proc.stdin.close()
                self.proc.wait(timeout=5)
            except Exception:                            # noqa: BLE001
                self.proc.kill()


def preflight_acoustic(scenario, live=None, log=None, gemini=False, timeout=900.0, asr=None):
    """FAIL LOUDLY unless the real pipeline runs on the live sources: starts the worker, runs one listen on the
    current stage world, checks the answer is the pipeline's (not the stub) and reports its latency."""
    say = log or _say_default
    svc = AcousticService(scenario, log=say, gemini=gemini, asr=asr).start(timeout)
    truth = live.truth() if live is not None else scenario.truth
    pxy = {p.name: (float(p.x), float(p.y)) for p in scenario.piles}
    sp = scenario.public_dict()["start_pile"]
    cx, cy = pxy[sp]
    drones = {"d1": (cx + 6.0, cy + 4.0), "d2": (cx - 6.0, cy + 3.0), "d3": (cx, cy - 7.0)}
    t0 = time.time()
    r = svc.wait(svc.submit(truth, drones, [sp], scenario.seed + 7), timeout)
    res = r["result"]
    mode = (res.get("agent") or {}).get("mode")
    if mode in (None, "stub") or "stub" in str(res.get("source", "")):   # real results are 'synthetic' too (scene)
        svc.close()
        raise RuntimeError("acoustic pre-flight: the worker answered with the STUB generator, not the pipeline")
    fed = r.get("fed")
    if _sources_payload(truth) and fed == "truth":
        say("acoustic pre-flight WARNING: listen_and_rank has no sources= yet; the pipeline synthesises from the "
            "stage positions + transcripts (TTS), not the dummy WAVs (acoustic agent: build_scene(sources=...))")
    lat = time.time() - t0
    calls = llm_calls(res)
    r2 = svc.wait(svc.submit(truth, drones, [sp], scenario.seed + 8, gemini=False), timeout)   # warm: the take's latency
    r = dict(r, elapsed_s=r2["elapsed_s"])
    say(f"acoustic pre-flight PASS: real pipeline in {AUDIO_PY} (mode {mode}, fed {fed}, speech-to-text "
        f"{r2.get('asr', 'en')}), warm listen {r['elapsed_s']:.1f} s (first one {lat:.1f} s incl. the model load), "
        f"{len(res.get('clips', []))} clips, timing {r2['result'].get('timing_s')}")
    say(f"Gemini: {calls} call(s) in the pre-flight listen ({'Gemini stage on' if gemini else 'Gemini stage off'}); "
        f"a take makes ~{TAKE_GATE_LISTENS} gate listens -> ~{TAKE_GATE_LISTENS * calls} calls (+ {calls} here) "
        f"against a free-tier budget of ~{GEMINI_BUDGET}/day/model")
    return svc, {"mode": mode, "fed": fed, "latency_s": r["elapsed_s"], "first_s": round(lat, 1), "asr": r2.get("asr"),
                 "timing_s": r2["result"].get("timing_s"),
                 "sources_api": svc.info.get("sources_api"), "scene_sources": svc.info.get("scene_sources"),
                 "gemini_calls": calls, "take_gemini_calls_est": TAKE_GATE_LISTENS * calls}


def make_acoustic_fn(live, mode="auto", log=None, service=None):
    """``acoustic_fn`` for GridMission: stage sources; cached results only while the stage equals the scenario."""
    from sim.acoustic_grid import listen as cached_listen
    from sim.acoustic_grid import listen_and_rank as stub
    say = log or _say_default

    def fn(scenario, drones, exclude, seed):
        live.refresh(None, "listen")
        fresh = live.changed
        if service is not None:                           # the take: the real pipeline on the live sources
            jid = find_prefetch(live.prefetch, drones, exclude, seed, live.sig)
            t0 = time.time()
            if jid is None:
                jid = service.submit(live.truth(), drones, exclude, seed,
                                     gemini=live.gemini_ok() if live.gemini_ok else None)
            else:
                say(f"listen: using the pipeline run prefetched when the landings were commanded (job {jid})")
            live.waiting = {"job": jid, "since": t0}
            try:
                r = service.wait(jid)
            finally:
                live.waiting = None
            res, src = r["result"], f"real-live ({r.get('fed')}, {r['elapsed_s']:.0f} s)"
            if time.time() - t0 > 0.5:
                say(f"listen waited {time.time() - t0:.1f} s for the real acoustic pipeline (mission clock held)")
            res = dict(res, live={"source": src, "fresh_uncached": True, "world": live.sig,
                                  "scenario_world": live.reference_sig})
            live.last_listen = {"t": round(live.t, 1), "source": src, "fresh": True, "n_clips": len(res.get("clips", []))}
            live.note_heard(res)
            return res
        if mode == "stub":
            res, src = stub(live.scenario, drones, exclude, seed), "stub-live"
        elif not fresh:
            res, src = cached_listen(live.base, drones, exclude, seed, mode=mode, log=say)
        else:
            try:
                res, src = _fresh_real(live.scenario, drones, exclude, seed, allow_subprocess=(mode == "real"))
            except Exception as e:                        # noqa: BLE001
                if mode == "real":
                    raise
                say(f"fresh real acoustic synthesis unavailable ({str(e)[:120]}); synthetic generator on the stage sources")
                res, src = stub(live.scenario, drones, exclude, seed), "stub-live"
        res = dict(res, live={"source": src, "fresh_uncached": bool(fresh or mode == "stub"), "world": live.sig,
                              "scenario_world": live.reference_sig})
        live.last_listen = {"t": round(live.t, 1), "source": src, "fresh": bool(fresh), "n_clips": len(res.get("clips", []))}
        return res
    return fn


# =============================================================================== the controller (mission side)
class LiveController:
    """Glue between the render loop, the live world and the mission: hooks, polling, quick listens, preempts."""

    def __init__(self, mission, live, out_dir=None, quick_s=QUICK_LISTEN_S, preempt=True, acoustic="auto",
                 radar="dsp", keys=DEFAULT_KEYS, log=None, clock=time.monotonic, service=None):
        self.m, self.live, self.out = mission, live, out_dir
        self.service = service             # AcousticService: the real pipeline (the take); None = in-process / stub
        self.gemini_budget, self._gemini_warned = GEMINI_BUDGET, False
        live.gemini_ok = self.gemini_ok
        self._ql_job = None                # (job id, points, t) of the pod-mic listen in flight
        self.say = log or _say_default
        self.quick_s, self.preempt_on, self.clock = float(quick_s), bool(preempt), clock
        self.heard = {}                    # pile -> highest priority ever heard (outputs only)
        self.silent_from = {}              # pile -> closest listening distance at which it was heard silent
        self._cand_prev = set()            # new-evidence candidates of the previous quick listen (need 2 in a row)
        self.strong = set()                # piles confirmed by one clear quick listen (QUICK_STRONG)
        self._pf = None                    # the prefetched full listen in flight: {jid, wall, need_t} (listen pace)
        self.quick = []                    # quick-listen summaries (HUD / live.json)
        self.replans = []                  # [{t, why, from, to}]
        self.n_changes = 0                 # quick listens folded in + re-plans (the loop rebuilds the state)
        self.inputs = []                   # live inputs: stage events, tasking revisions, portal reports
        self.input_pending = None          # {"since", "cause", "expect"} until the next dispatch
        self.expect = None                 # the pile the fleet would go to next, as ranked before any live input
        self.unheard = None                # {"t", "sector", "fresh"}: a live stage change the pod mics have not checked yet
        self._ql_wall = None               # wall time the pod-mic listen in flight was submitted
        self._lat = {"quick": None, "full": None}   # last compute seconds of each kind of real listen
        self._pace_said = None
        self.banner = None                 # {"text", "since", "t"}: PLAN CHANGED, ~5 s on screen
        self._state, self._n_dispatch, self._n_events = None, 0, 0
        self.threaded = False              # --mission-thread: USD syncs happen on the render thread only
        self._ql_t, self._ql_n = -1e9, 0
        self._last_poll = -1e9
        self._seen_res = None
        self._cmd_seq = None
        self._tk_rev = getattr(getattr(mission, "tracker", None), "revision", None)
        self._zones = set(mission.tracker.zone_ids()) if getattr(mission, "tracker", None) is not None else set()
        self._devices = set()
        self.hook = self._attach(acoustic, radar)
        self.keys = parse_keys(keys)
        self._key_queue = []
        self.test_triggers = []

    # ------------------------------------------------------------------ hooks
    def _attach(self, acoustic, radar):
        """Hand the live world to the mission's generators (GridMission.set_world_sources, fa4e74b)."""
        m, live = self.m, self.live
        if not all(hasattr(m, k) for k in ("set_world_sources", "ingest_listen", "preempt")):
            raise RuntimeError("sim/grid_mission.py has no live hooks (set_world_sources / ingest_listen / preempt, "
                               "fa4e74b): update the mission before --live")
        m.set_world_sources(live.scenario, sensor_factory=lambda pw, seed: LiveRadar(pw, seed, radar, live),
                            acoustic_fn=make_acoustic_fn(live, acoustic, self.say, service=self.service))
        return "native"

    # ------------------------------------------------------------------ per step
    def before_step(self):
        self.live.t = float(self.m.t)
        now = self.clock()
        if now - self._last_poll >= POLL_WALL_S:
            self._last_poll = now
            self.poll()
        elif self.out and now - getattr(self, "_cmd_stat_t", 0.0) >= 0.2:   # the live button: no 1 s wait (at x24
            self._cmd_stat_t = now                                           # 1 s is 24 sim s, reh6b)
            try:
                mt = os.stat(os.path.join(self.out, "live_cmd.json")).st_mtime
            except OSError:
                mt = None
            if mt is not None and mt != getattr(self, "_cmd_mtime", None):
                self._cmd_mtime = mt
                self._read_cmd_file()
        hit = False
        for kind, did in self._drain_keys():
            hit = self.live.toggle(did, f"hotkey {kind}") or hit
        for trig in list(self.test_triggers):            # tests / --dry-run rehearsals only
            if self.m.t >= trig["t"]:
                self.test_triggers.remove(trig)
                hit = self.apply_cmd(trig, cause="test-trigger (dry-run only)") or hit
        if hit:
            if hasattr(self.live.source, "sync") and not getattr(self, "threaded", False):
                self.live.source.sync()
            self.live.refresh(self.m.t, "trigger")      # log the stage event now (the generators read it anyway)

    def after_step(self, state):
        """Returns True when it changed the mission (folded a quick listen or re-planned)."""
        m = self.m
        n0 = self.n_changes
        self._state = state
        self._track_inputs(state)
        if m.last_acoustic is not None and m.last_acoustic is not self._seen_res:
            self._seen_res = m.last_acoustic
            self._note_heard(m.last_acoustic, fold=False)
        self._check_tasking()
        self._check_portal(state)
        self._prefetch_listen()
        self._quick_listen(state)
        self._track_dispatch(state)
        return self.n_changes != n0

    # ------------------------------------------------------------------ autonomy badge + PLAN CHANGED banner
    def _note_input(self, cause):
        self.inputs.append({"t": round(self.m.t, 1), "cause": cause, "since": self.clock()})
        if self.input_pending is None:
            self.input_pending = {"since": self.clock(), "cause": cause, "expect": self.expect}

    def _track_inputs(self, state):
        evs = self.live.events
        for e in evs[self._n_events:]:
            if e["what"] in ("activated", "moved", "added", "clip changed", "deactivated"):
                self._note_input(f"dummy {e['id']} {e['what']} ({e['cause']})")
                if e["what"] != "deactivated":
                    self.unheard = {"t": float(e.get("t") or self.m.t), "sector": e.get("sector"), "fresh": 0}
        self._n_events = len(evs)
        if self.input_pending is None:                     # what the fleet would do next without the live input
            m = self.m
            here = m.pm.pile if m.pm is not None else None
            cand = [p for p in (state.get("piles") or []) if p["id"] not in m.visited and p["id"] != here
                    and p.get("priority") is not None    # before any listen nothing is ranked: no guess (reh5 said A1
                    and (m._tasked_piles() is None or p["id"] in m._tasked_piles())]   # by name order)
            cand.sort(key=lambda p: (-(p["priority"] if p["priority"] is not None else -1), p["id"]))
            self.expect = cand[0]["id"] if cand else None

    def _set_banner(self, text):
        self.banner = {"text": text, "since": self.clock(), "t": round(self.m.t, 1)}
        self.say(text)

    def _track_dispatch(self, state):
        dis = state.get("dispatch") or []
        if len(dis) <= self._n_dispatch:
            return
        new = dis[-1]
        self._n_dispatch = len(dis)
        ip = self.input_pending
        if ip is None:
            return
        self.input_pending = None
        to, old = new.get("to_pile"), ip.get("expect")
        if to and to == ip.get("announced"):
            return                                         # the re-plan banner already named this pile
        q = next((e for r in reversed(self.quick) for e in r.get("new") or [] if e["pile"] == to), None) if to else None
        if old is None and not q and new.get("by") != "operator":
            return              # nothing was ranked before the input and nothing new was heard at ``to``: no "plan
                                # changed" claim (reh5b said "A1 -> B1": A1 was only the name-order guess)
        if to and to != old:
            why = (f"new human-like sound at {to}" if q else "operator order" if new.get("by") == "operator"
                   else "re-ranked on new evidence")
            self._set_banner(f"PLAN CHANGED: {old or 'next pile'} → {to} ({why}, unscripted live input)")

    def badge(self):
        if not self.inputs:
            return "AUTONOMOUS · no operator input"
        last = self.inputs[-1]
        if self.clock() - last["since"] <= BADGE_LIVE_S:
            return f"LIVE INPUT · {last['cause']}"
        return f"AUTONOMOUS · {len(self.inputs)} live input(s), last at t {last['t']:.0f}"

    def poll(self):
        """Wall-clock poll (1 s): tasking file, command file, stage dummies."""
        m = self.m
        if hasattr(m, "_poll_tasking"):
            m._poll_tasking()
        self._check_tasking()
        self._read_cmd_file()
        self.live.refresh(None, "poll")

    # ------------------------------------------------------------------ (a) keys + command file
    def on_key(self, key):
        did = self.keys.get(str(key).upper())
        if did is None:
            return False
        if did.startswith("hidden"):
            hid = self.live.source.hidden_ids()
            k = int(did[6:] or 1) - 1
            if k >= len(hid):
                self.say(f"hotkey {key}: no hidden dummy #{k + 1}")
                return True
            did = hid[k]
        self._key_queue.append((key, did))
        return True

    def _drain_keys(self):
        q, self._key_queue = self._key_queue, []
        return q

    def _read_cmd_file(self):
        if not self.out:
            return
        path = os.path.join(self.out, "live_cmd.json")
        try:
            with open(path) as f:
                doc = json.load(f)
        except (OSError, ValueError):
            return
        seq = doc.get("seq")
        if seq == self._cmd_seq:
            return
        first = self._cmd_seq is None and doc.get("wall", 0) < time.time() - 30
        self._cmd_seq = seq
        if first:                                          # a stale command from an earlier run
            return
        if self.apply_cmd(doc, cause="live_cmd.json (operator terminal)"):
            self.live.refresh(self.m.t, "trigger")

    def apply_cmd(self, c, cause):
        did = c.get("id")
        if did in (None, "") or str(did).startswith("hidden"):
            hid = self.live.source.hidden_ids()
            k = int(str(did or "hidden1")[6:] or 1) - 1
            did = hid[k] if 0 <= k < len(hid) else None
        if did is None:
            return False
        cmd = c.get("cmd", "toggle")
        if cmd == "toggle":
            return self.live.toggle(did, cause)
        if cmd in ("activate", "deactivate"):
            return self.live.set_active(did, cmd == "activate", cause)
        if cmd == "move" and c.get("xyz") is not None:
            return self.live.move(did, c["xyz"], cause)
        return False

    # ------------------------------------------------------------------ (b) operator tasking
    def _check_tasking(self):
        tr = getattr(self.m, "tracker", None)
        if tr is None or tr.revision == self._tk_rev:
            return
        self._tk_rev = tr.revision
        self._note_input(f"operator tasking revision {tr.revision}")
        zones = set(tr.zone_ids())
        new = [z for z in tr.zone_ids() if z not in self._zones]
        self._zones = zones
        m = self.m
        tasked = m._tasked_piles() or []
        cur = m.pm.pile if m.stage == "pile" and m.pm is not None else None
        target = None
        for z in reversed(new):                            # the newest zone the operator drew
            ps = [p for p in m._zone_search_piles(z) if p not in m.visited or p == cur]
            if ps:
                target, zid = ps[0], z
                break
        if target is not None and target != cur:
            why = f"operator re-task: new zone {zid} (tasking revision {tr.revision}) -> search {target} next"
            self._replan(why, to_pile=target)
        elif cur is not None and cur not in tasked:
            self._replan(f"operator re-task (revision {tr.revision}): {cur} is no longer inside a tasked zone")

    # ------------------------------------------------------------------ (d) portal (when the comms work lands)
    def _check_portal(self, state):
        comms = (state or {}).get("comms") or {}
        for d in comms.get("devices") or []:
            if d.get("status") != "trapped" or d.get("id") in self._devices or not d.get("est_xy"):
                continue
            self._devices.add(d.get("id"))
            self._note_input(f"phone portal: 'I'm trapped' ({d.get('id')})")
            pile = d.get("pile") or self.live.locate(*d["est_xy"])
            if pile in self.m.piles and pile not in self.m.visited:
                self._replan(f"portal: a phone reports 'I'm trapped' near {pile} (RSSI fit ±{d.get('est_sigma_m', '?')} m; "
                             "candidate region)", to_pile=pile)

    # ------------------------------------------------------------------ pod-mic quick listens (outputs only)
    def _listen_points(self, state):
        """Rotors-off microphones now: pods resting on the rubble (their drone hovers above, its own mics are not
        used) and drones landed on a patch (A4: idle drones land during pod rescans)."""
        pts = {}
        if getattr(self.m, "delivery", "pod") == "pod":
            pts = {p["drone_id"]: tuple(p["xy"][:2]) for p in state.get("pods") or [] if p.get("status") == "scanning"}
        for d in state.get("drones", []):
            if d["id"] not in pts and d.get("state") in ("landed", "listening") or (
                    getattr(self.m, "delivery", "pod") != "pod" and d.get("state") == "scanning"):
                pts.setdefault(d["id"], tuple(d["pose"][:2]))
        return pts

    def _listen_targets(self):
        """Where each drone will sit for the rotors-off listen: its pose if landed, else the destination of its
        'listen-*' landing job (known the moment the mission commands the landings). None if any is unknown."""
        m, out = self.m, {}
        for p in m.flight.poses():
            did = p["id"]
            st, _, job = m.flight.drone_state(did)
            if st in ("landed", "docked") and not job:
                out[did] = (float(p["pose"][0]), float(p["pose"][1]))
                continue
            route = getattr(m.flight.drones.get(did) if isinstance(m.flight.drones, dict) else None, "route", None)
            if job and str(job.get("site", "")).startswith("listen-") and route:
                out[did] = (float(route[-1][0]), float(route[-1][1]))
            else:
                return None
        return out

    def _prefetch_listen(self):
        """Real pipeline: submit the rotors-off listen the moment its positions are known - when the mission commands
        the listen landings at the end of the radar phase - so the synthesis overlaps the landing, the listen window
        and the call-out. The stage world goes in as it is then; a later change makes the world signature differ and
        the gate listen is synthesised again (never a stale world)."""
        m = self.m
        if self.service is None or m.stage != "listen":
            return
        drones = self._listen_targets()
        if not drones:
            return
        exclude = list(dict.fromkeys(m.visited))
        seed = m.seed + 101 * (m.n_listen + 1)
        self.live.refresh(m.t, "listen prefetch")
        if find_prefetch(self.live.prefetch, drones, exclude, seed, self.live.sig) is not None:
            return
        # the worker runs one listen at a time: a pod-mic listen still computing delays this one (rehearsal 5b: the
        # mission clock was held 10.2 s at the gate, the pace had assumed an idle worker)
        ahead = 0.0
        if self._ql_job is not None and self.service.poll(self._ql_job[0]) is None and self._ql_wall is not None:
            ahead = max(0.5, (self._lat["quick"] or 0.6 * self.listen_latency()) - (time.time() - self._ql_wall))
        jid = self.service.submit(self.live.truth(), drones, exclude, seed, gemini=self.gemini_ok())
        self.live.prefetch[listen_key(drones, exclude, seed, self.live.sig)] = (jid, dict(drones))
        self._pf = {"jid": jid, "wall": time.time(), "t": m.t, "ahead": ahead}   # listen pace (listen_pace)
        self.say(f"real acoustic pipeline: listen {m.n_listen + 1} submitted at t {m.t:.0f} (landing patches known); "
                 f"it overlaps the landing, the listen window and the call-out")

    def gemini_ok(self):
        """Gemini (zh/id/vi transcripts) only on the gate listens, within the day's budget (free tier ~20/model)."""
        svc = self.service
        if svc is None or not svc.gemini:
            return False
        if svc.llm_calls >= self.gemini_budget:
            if not self._gemini_warned:
                self._gemini_warned = True
                self.say(f"GEMINI BUDGET REACHED ({svc.llm_calls}/{self.gemini_budget} calls this run): further listens "
                         "run without the Gemini stage (local transcripts only)")
            return False
        return True

    def listen_latency(self):
        """Expected compute seconds of one full real listen (the gate's: passive window + call-out): the last one
        this take, else the pre-flight's warm listen. Pod-mic quick listens (10 s window) are not counted."""
        if self._lat["full"]:
            return float(self._lat["full"])
        # the pre-flight listen has no call-out: the gate's synthesises ~1.3x the audio (reh5b ~12 s vs 8.6 s)
        return 1.3 * float(getattr(self, "preflight_latency", 0.0) or 12.0)

    def listen_pace(self, t_now, base, floor=2.0):
        """Take pacing while the drones listen (rotors off, nothing moves): run the phase at ``base`` and slow it only
        while the prefetched real listen is still computing, just enough that the mission reaches the listen gate
        when the result is due. Replaces a fixed cap for the whole phase (rehearsal 4b: x4 for 177 sim s = 44 s of
        screen time; this is ~15 s for a 15 s pipeline)."""
        pf = self._pf
        need_t = getattr(self.m, "listen_end", None)       # set once every drone is down (+ the call-out)
        if self.service is None or pf is None or need_t is None:
            return base
        r = self.service.poll(pf["jid"])
        if r is not None:
            self._lat["full"] = r.get("elapsed_s") or self._lat["full"]
            return base
        left_sim = float(need_t) - float(t_now)
        if left_sim <= 0:
            return base
        left_wall = max(0.75, pf.get("ahead", 0.0) + self.listen_latency() + 0.5 - (time.time() - pf["wall"]))
        return max(floor, min(base, left_sim / left_wall))

    def _quick_listen(self, state):
        m = self.m
        if self._ql_job is not None:                         # async (real pipeline): fold it in when it arrives
            jid, pts, t_sub = self._ql_job
            r = self.service.poll(jid)
            if r is None:
                return
            self._ql_job = None
            self._lat["quick"] = r.get("elapsed_s") or self._lat["quick"]
            if "error" in r:
                self.say(f"pod-mic listen {jid} failed: {r['error']}")
                return
            self._handle_quick(r["result"], pts, t_sub)
            return
        if m.stage != "pile" or m.pm is None or m.t - self._ql_t < self.quick_s:
            return
        pts = self._listen_points(state)
        if len(pts) < 2:
            return
        self._ql_t, self._ql_n = m.t, self._ql_n + 1
        exclude = list(dict.fromkeys(m.visited))
        seed = m.seed + 5003 + 17 * self._ql_n
        if self.service is not None:
            self.live.refresh(m.t, "pod-mic listen")
            self._ql_job = (self.service.submit(self.live.truth(), pts, exclude, seed, gemini=False,
                                                duration=QUICK_LISTEN_DUR or None), pts, m.t)
            self._ql_wall = time.time()
            return
        res = m.acoustic_fn(m.scenario, pts, exclude, seed)
        self._handle_quick(res, pts, m.t)

    def quick_pace(self, base, cap=QUICK_PACE_CAP):
        """Take pacing after a live change while a pile is searched: the real pod-mic listen takes ~5 s of wall time,
        i.e. ~120 sim s at the x24 relocate pace, so at full pace the pile could end before a listen that includes
        the change returns (rehearsal 5b: the voice switched on at t 265 was never checked; the full listen then
        heard it weakly). Cap the pace until a quick listen submitted after the change reports new evidence, or
        QUICK_PACE_TRIES of them have returned, or the pile ends. Only the picture's speed changes, not the mission."""
        u, m = self.unheard, self.m
        on = (self.service is not None and u is not None and m.stage == "pile" and m.pm is not None
              and u["fresh"] < QUICK_PACE_TRIES)
        if on != bool(self._pace_said):
            self._pace_said = on
            self.say(f"take pace capped at x{cap:g} while the pod mics check the live change" if on else
                     "take pace back to normal (the pod mics have checked the live change)")
        return min(base, cap) if on else base

    def _handle_quick(self, res, pts, t_sub):
        m = self.m
        self.live.note_heard(res, t_sub, "pod-mic")
        new = self._note_heard(res, fold=True, points=pts)
        if self.unheard is not None and t_sub >= self.unheard["t"] - 1e-6:
            self.unheard["fresh"] += 1
            if new or self.unheard["fresh"] >= QUICK_PACE_TRIES:
                self.unheard = None
        rec = {"t": round(t_sub, 1), "t_result": round(m.t, 1), "n": self._ql_n,
               "points": {k: [round(v, 1) for v in xy] for k, xy in pts.items()},
               "clips": len(res.get("clips", [])), "new": [{"pile": p, "priority": pr, "was": was} for p, pr, was in new]}
        self.quick.append(rec)
        del self.quick[:-20]
        if not new or m.stage != "pile" or m.pm is None:
            return                                         # the pile ended meanwhile: the next gate re-ranks anyway
        pile, pr, was = max(new, key=lambda e: e[1])
        self.n_changes += 1
        m.ingest_listen(res, pts, source="pod-mic")
        how = ("heard clearly in one listen, bearings from several drones agree" if pile in self.strong
               else "confirmed by two consecutive listens")
        m._say(f"Pod-mic quick listen {self._ql_n}: NEW human-like sound at {pile} (priority {pr}; earlier listens from "
               f"as close heard {'nothing human-like' if not was else f'at most {was}'} there), {how}; not a "
               "confirmed person")
        cur = m.pm.pile
        cp = m.piles[cur]["priority"] or 0
        firm = any(c.get("status") == "candidate" for c in m.pm.cands)
        if not self.preempt_on:
            return
        if firm:
            m._say(f"Re-prioritise: {pile} ({pr}) queued; {cur} has a firm ping in progress, finishing that fix first")
        elif pr > cp + PREEMPT_MARGIN and pr >= m.priority_min:
            nxt = self.gate_target()
            top = "" if nxt in (None, pile) else f"; {nxt} ({m.piles[nxt]['priority']}) now ranks first"
            self._replan(f"new human-like call at {pile} (priority {pr}) outranks {cur} ({cp}){top}")

    def _note_heard(self, res, fold, points=None):
        """Track, per pile, the most it was heard at and the closest listen that heard it SILENT. Returns
        [(pile, priority, was)] that are NEW evidence: a lead now, at or above priority_min, where an earlier listen
        from no further away (+ NEW_EVIDENCE_NEAR_M) heard nothing human-like. Being closer is not new evidence;
        something that started calling is. A candidate must be heard in two consecutive quick listens (one lead at
        the detection threshold is generator / channel noise, not news). Outputs and listening positions only."""
        new, cands, diag = [], set(), []
        m = self.m
        pts = points or {d["id"]: tuple(d["xy"][:2]) for d in res.get("listening_drones") or []} or m._drone_xy()
        pxy = m.world.pile_xy
        clip_by = {c.get("clip_id"): c for c in res.get("clips", [])}

        def agree(p):                                   # drones whose bearings agree on the pile's top clip
            try:
                return int(str((clip_by.get(p.get("top_clip")) or {}).get("doa_agree", "0/0")).split("/")[0])
            except ValueError:
                return 0
        for p in res.get("piles", []):
            pid, pr = p.get("pile_id"), int(p.get("priority") or 0)
            if pid is None or pid not in pxy:
                continue
            d = min((math.hypot(pxy[pid][0] - x, pxy[pid][1] - y) for x, y in pts.values()), default=1e9)
            lead = p.get("status") == "lead" and pr >= m.priority_min
            was = self.heard.get(pid, 0)
            quiet = self.silent_from.get(pid)
            if fold and lead:
                diag.append(f"{pid} {pr}" + (" visited" if pid in m.visited else
                            f" (heard before at {was})" if quiet is None else
                            f" (silent from {quiet:.0f} m, now {d:.0f} m, agree {agree(p)})"))
            if fold and lead and pid not in m.visited and quiet is not None \
                    and quiet <= max(d + NEW_EVIDENCE_NEAR_M, NEW_EVIDENCE_RATIO * d) and pr >= was + NEW_EVIDENCE_RISE:
                cands.add(pid)
                strong = pr >= QUICK_STRONG and agree(p) >= 2
                if pid in self._cand_prev or strong:       # two consecutive quick listens, or one clear one
                    new.append((pid, pr, was))
                    if strong and pid not in self._cand_prev:
                        self.strong.add(pid)
            if lead and (not fold or pid not in cands or pid in self._cand_prev or pid in self.strong):
                self.heard[pid] = max(was, pr)
            else:
                self.silent_from[pid] = min(quiet if quiet is not None else 1e9, d)
        if fold:
            self._cand_prev = cands
            self.say(f"quick listen {self._ql_n}: leads " + (", ".join(diag) or "none") +
                     (f"; NEW {', '.join(p for p, _, _ in new)}" if new else "") +
                     (f"; candidates {', '.join(sorted(cands))}" if cands else ""))
        return new

    # ------------------------------------------------------------------ re-plan
    def gate_target(self):
        """The open pile the next dispatch gate ranks first, sorted as GridMission._gate sorts (display only: the
        gate itself decides, incl. battery swaps and revisits). None before any listen ranked a pile."""
        m = self.m
        here = m.pm.pile if m.pm is not None else None
        tasked = m._tasked_piles()
        ranked = sorted((p for p in m.piles.values() if p["id"] not in m.visited and p["id"] != here
                         and p.get("priority") is not None and p["priority"] >= m.priority_min
                         and (tasked is None or p["id"] in tasked)), key=lambda p: (-p["priority"], p["id"]))
        return ranked[0]["id"] if ranked else None

    def _replan(self, why, to_pile=None):
        """Native GridMission.preempt while a pile is being searched (the cut waits for the dwell under way; the
        cut pile stays not cleared). Between piles an operator order goes straight to the next dispatch gate."""
        m = self.m
        self.replans.append({"t": round(m.t, 1), "why": why, "from": m.pm.pile if m.pm is not None else None,
                             "to": to_pile, "next": to_pile or self.gate_target(), "stage": m.stage})
        self.n_changes += 1
        if self.input_pending is not None or to_pile is not None:
            # name the pile the fleet will fly to: the gate's top-ranked open pile, not the pile whose new sound
            # triggered the cut (reh5: B3 44 confirmed twice triggered it, A1 56 heard once ranked first and won)
            tgt = to_pile or self.gate_target() or next((e["pile"] for r in reversed(self.quick)
                                                        for e in sorted(r.get("new") or [], key=lambda e: -e["priority"])), None)
            here = m.pm.pile if m.pm is not None else (self.expect or "plan")
            if tgt and tgt != here:
                self._set_banner(f"PLAN CHANGED: {here} → {tgt} ({_short_why(why)}, unscripted live input)")
                if self.input_pending is not None:
                    self.input_pending["announced"] = tgt
        if m.stage == "pile" and m.pm is not None:
            m.preempt(why, to_pile=to_pile)
            m._say(f"RE-PLAN requested: {why}; finishing the radar dwell under way at {m.pm.pile}, then the gate")
        elif to_pile is not None:
            m._order = to_pile                              # GridMission._gate honours it first (fa4e74b)
            m._say(f"RE-PLAN queued: {why} (applies at the next dispatch gate)")

    # ------------------------------------------------------------------ output
    def to_json(self):
        last = self.live.events[-1] if self.live.events else None
        return {"schema": "larpverse.live/v1", "t": round(self.m.t, 1), "hook": self.hook,
                "world": {"source": self.live.source.kind, "signature": self.live.sig,
                          "scenario_signature": self.live.reference_sig, "changed": self.live.changed,
                          "active": [v["id"] for v in self.live._victims], "reads": self.live.reads},
                "events": self.live.events[-10:], "last_event": last, "quick_listens": self.quick[-5:],
                "replans": self.replans[-10:], "last_listen": self.live.last_listen, "heard": self.live.heard, "last_scan": self.live.last_scan,
                "acoustic": {"real_pipeline": self.service is not None,
                             "waiting": None if not self.live.waiting else
                             {"job": self.live.waiting["job"], "s": round(time.time() - self.live.waiting["since"], 1)},
                             "latency_s": (self.service.latency[-5:] if self.service is not None else None)},
                "keys": self.keys, "badge": self.badge(),
                "banner": None if self.banner is None else dict(self.banner),
                "causal": _causal(self._state, self),
                "lidar_flash": getattr(self, "lidar_flash", None)}

    def write(self):
        if not self.out:
            return
        path = os.path.join(self.out, "live.json")
        try:
            with open(path + ".tmp", "w") as f:
                json.dump(self.to_json(), f, separators=(",", ":"))
            os.replace(path + ".tmp", path)
        except OSError:
            pass


# Take pacing: sim seconds per VIDEO second, per mission phase (the HUD always shows the factor: "sped up xN").
# Tuned on seed 7 with the smooth flight model (sim/flight_motion.py, docs/VIDEO_PLAN.md section 2): 4 s pre-roll on
# the van, lift-off 4-8 s, LiDAR survey 8-15 s, pods 15-23 s, relocate / rescan cycles fast to the A2 ping at ~30 s,
# the listen + call-out at a watchable pace (30-43 s), the redirect and the next pile's survey and pods to 60 s.
DEFAULT_PACE = ("deploy=5,survey=6,recon_mapping=6,recon_listening=10,landing=5,radar=14,relocate=30,rescan=30,"
                "acoustic=10,dispatch=6,return=12,battery_swap=40,done=8")


def parse_pace(spec):
    """'survey=3,radar=12' -> {phase: speed}; 'off' / '' -> {}."""
    out = {}
    if not spec or spec == "off":
        return out
    for part in spec.split(","):
        if "=" in part:
            k, v = part.split("=", 1)
            out[k.strip()] = float(v)
    return out


def pace_for(pace, phase, default):
    return float(pace.get(phase, default)) if pace else float(default)


BADGE_LIVE_S = 10.0     # video s the badge reads LIVE INPUT after a stage event / tasking change / portal report


def _short_why(why):
    w = str(why)
    if "new human-like" in w:
        return "new voice heard"
    if "operator re-task" in w or "zone" in w:
        return "operator zone"
    if "portal" in w:
        return "phone portal report"
    return w[:40]


def set_speaker_audio(mission, mode="off", log=None):
    """Loudspeaker audio for the take. The call-out renders (acoustic/speaker_assets.py) use Piper voices whose
    licences are unclear or research-only (docs/DUMMY_AUDIO.md Requests), so the take serves NO audio file
    (``off``) or only eSpeak NG renders (``espeak``); ``any`` leaves it as is. The call-out itself, its timing
    (durations stay those of the rendered manifest, so the story does not move) and the on-screen speaker ring are
    unchanged: only ``broadcasts[].audio_url`` is withheld."""
    say = log or _say_default
    sp = getattr(mission, "speaker", None)
    if sp is None or mode == "any":
        return 0
    man = getattr(sp, "manifest", {}) or {}

    def lookup(lang, text):
        from sim.guidance import audio_key
        it = man.get(audio_key(lang, text))
        if mode == "espeak" and it and it.get("engine") == "espeak" and it.get("path"):
            return f"acoustic/data/speaker/{os.path.basename(it['path'])}", "rendered (eSpeak NG, GPL-3.0 engine output)"
        return None, ("muted for the take (Piper voice licences unclear; see docs/DUMMY_AUDIO.md)" if mode == "off"
                      else "no eSpeak NG render (python -m acoustic.speaker_assets --engine espeak --force)")
    sp._audio = lookup
    if getattr(sp, "log", None) is not None:
        sp.log.audio_lookup = lookup
    n = sum(1 for it in man.values() if it.get("engine") == "espeak")
    say(f"speaker audio for the take: {mode} ({n} eSpeak NG renders available); the on-screen call-out ring stays")
    return n


def parse_keys(spec):
    out = {}
    for part in (spec or "").split(","):
        if "=" in part:
            k, v = part.split("=", 1)
            out[k.strip().upper()] = v.strip()
    return out


def _causal(state, ctrl):
    try:
        from isaac.hud import causal_lines
        lj = {"last_scan": ctrl.live.last_scan, "quick_listens": ctrl.quick[-5:], "replans": ctrl.replans[-10:]}
        return causal_lines(state or {}, lj)
    except Exception:                                       # noqa: BLE001  (output only)
        return []


def parse_test_trigger(spec):
    """'activate:H1@150' / 'toggle:hidden1@120' / 'move:H1:-10,5,0.5@200' -> command dicts (TESTS ONLY)."""
    out = []
    for part in (spec or "").split(";"):
        part = part.strip()
        if not part:
            continue
        body, t = part.rsplit("@", 1)
        bits = body.split(":")
        c = {"cmd": bits[0], "id": bits[1] if len(bits) > 1 else None, "t": float(t)}
        if c["cmd"] == "move" and len(bits) > 2:
            c["xyz"] = [float(v) for v in bits[2].split(",")]
        out.append(c)
    return out


# =============================================================================== keyboard (carb.input, guarded)
class Hotkeys:
    """Isaac window hotkeys: carb.input keyboard events -> ``on_key(name)``. No-op without carb."""

    def __init__(self, on_key, keys, log=None):
        self.ok, self._sub, self._inp, self._kb = False, None, None, None
        say = log or _say_default
        try:
            import carb.input
            import omni.appwindow
            self._carb = carb.input
            self._inp = carb.input.acquire_input_interface()
            self._kb = omni.appwindow.get_default_app_window().get_keyboard()
            self._names = set(keys)
            self._on = on_key
            self._sub = self._inp.subscribe_to_keyboard_events(self._kb, self._cb)
            self.ok = True
            say(f"hotkeys armed in the Isaac window: {', '.join(f'{k} -> {v}' for k, v in keys.items())}")
        except Exception as e:                              # noqa: BLE001
            say(f"hotkeys unavailable ({type(e).__name__}: {e}); use the HUD button or live_cmd.json")

    def _cb(self, e, *_):
        try:
            if e.type == self._carb.KeyboardEventType.KEY_PRESS:
                name = getattr(e.input, "name", str(e.input)).split(".")[-1].upper()
                if name in self._names:
                    self._on(name)
        except Exception:                                   # noqa: BLE001  (never break the app's input loop)
            pass
        return True

    def close(self):
        if self.ok and self._sub is not None:
            try:
                self._inp.unsubscribe_to_keyboard_events(self._kb, self._sub)
            except Exception:                               # noqa: BLE001
                pass


# =============================================================================== director take camera
def _bezier(p0, p1, p2, p3, s):
    u = 1.0 - s
    return u ** 3 * p0 + 3 * u * u * s * p1 + 3 * u * s * s * p2 + s ** 3 * p3


def _smooth(s):
    return s * s * (3 - 2 * s)


def _spring(x, v, goal, dt, w):
    """Critically damped spring (exact step, stable for any dt): continuous position and velocity."""
    if dt <= 0:
        return x, v
    e = x - goal
    k = math.exp(-w * dt)
    tmp = (v + w * e) * dt
    return goal + (e + tmp) * k, (v - w * tmp) * k


def cam_float(eye, look, t, amp=1.0):
    """A gimbal-stabilised aerial camera never hangs perfectly still: the airframe drifts a few cm in the wind and
    the gimbal lags a little. Smooth, non-repeating drift (sums of incommensurate sines, 0.07-0.6 Hz): position
    ~0.25 % of the camera distance, aim ~0.05 deg, so it reads as a real drone shot, never as shake."""
    eye, look = np.asarray(eye, float), np.asarray(look, float)
    d = float(np.linalg.norm(eye - look)) or 1.0

    def n(ph, fr):
        return sum(math.sin(2 * math.pi * f * t + p) / (k + 1) for k, (f, p) in enumerate(zip(fr, ph))) / 1.83

    a = 0.0025 * d * float(amp)
    de = np.array([n((0.3, 2.1, 4.4), (0.071, 0.163, 0.41)), n((1.7, 3.9, 0.8), (0.089, 0.211, 0.53)),
                   0.6 * n((2.6, 0.2, 5.1), (0.067, 0.149, 0.37))]) * a
    dl = np.array([n((4.0, 1.1, 2.9), (0.113, 0.29, 0.61)), n((0.9, 5.3, 3.3), (0.097, 0.23, 0.57)),
                   n((3.1, 2.4, 0.5), (0.083, 0.19, 0.47))]) * (0.0007 * d * float(amp))
    return eye + de, look + de + dl


class TakeCamera:
    """``--camera director-take``: one continuous camera for the single take. The shot follows the mission
    (staging -> lift-off -> survey wide -> pods mid-shot -> listen wide -> dispatch follow with look-ahead -> end
    wide) and a live stage event (smooth re-target, held a few seconds). A shot change moves the GOAL along a cubic
    Bezier arc lifted above the rubble; the camera follows the goal with critically damped springs
    (isaac.motion.SmoothCamera, built-in fallback): position and velocity stay continuous, never a cut. Re-poses the USD camera + mirrors every frame
    (isaac/camera.py's black-viewport workaround) through an embedded DirectorCamera."""

    MID_DOWN_DEG = 25.0
    # opening shot: "van" = 9.9 m from the open door at eye height (docs/ISAAC_ASSETS.md); "wide" = an establishing
    # shot from 24 m south-west and 9 m up: van, drones on the pads and the rubble field beyond in one frame
    OPEN = "van"
    FLOAT = 0.0                                     # --cam-float amplitude (1 = a gimbal-stabilised drone camera)
    EYE_HALFLIFE, LOOK_HALFLIFE = 0.9, 0.45     # isaac.motion.SmoothCamera springs (s to close half the gap)
    W_EYE, W_LOOK = 2 * math.log(2) / 0.9, 2 * math.log(2) / 0.45   # the same springs, built in (fallback)

    def __init__(self, stage=None, pile_xy=None, base_xy=(0.0, -47.0), pads=None, surface=None, mirrors=(),
                 switch_viewport=True, orbit_period=180.0, van=None):
        from isaac.camera import DirectorCamera
        self.pile_xy = dict(pile_xy or {})
        self.base = np.array([float(base_xy[0]), float(base_xy[1])])
        pads = pads or [(base_xy[0] - 2.6, base_xy[1] - 2.0), (base_xy[0] + 2.6, base_xy[1] - 2.0)]
        self.pads = np.mean(np.asarray(pads, float)[:, :2], 0)
        self.surface = surface or (lambda x, y: 0.0)
        self.van = None if van is None else np.asarray(van[:3], float)   # (x, y, z) of isaac.van.staging_pose
        self.omega = 2 * math.pi / orbit_period
        self.az = math.radians(-120.0)
        z0 = float(self.surface(*self.pads))
        self.look = np.array([self.pads[0], self.pads[1], z0 + 1.5])
        self.eye = self.look + [16 * math.cos(self.az), 16 * math.sin(self.az), 5.0]
        if self.van is not None:                   # open exactly on the van shot: no settling move at t = 0
            self.az = math.atan2(-6.9, -5.8)
            self.look, d, h = self._van_open()
            self.eye = self.look + [d * math.cos(self.az), d * math.sin(self.az), h]
        self.dc = DirectorCamera(stage, mode="director", start=tuple(self.look), mirrors=mirrors,
                                 switch_viewport=switch_viewport)
        self.path, self.updates, self.mode = self.dc.path, 0, "director-take"
        self.shot, self.goal = ("van" if self.van is not None else "staging"), (self.eye.copy(), self.look.copy())
        self.trans = None
        self.hold = None                       # (until_video_t, look, label)
        self.vt = 0.0
        self.shots = [(0.0, self.shot)]
        self.v_eye, self.v_look = np.zeros(3), np.zeros(3)
        self.smoother = None
        try:                                       # isaac.motion.SmoothCamera (docs/MOTION.md R2): eye + look springs
            from isaac.motion import SmoothCamera
            self.smoother = SmoothCamera(self.eye, self.look, eye_halflife=self.EYE_HALFLIFE,
                                         look_halflife=self.LOOK_HALFLIFE)
        except Exception:                          # noqa: BLE001  (sim.flight_motion missing: built-in spring)
            self.smoother = None
        self._apply()

    @property
    def is_viewport(self):
        return self.dc.is_viewport

    def _van_open(self):
        """(look, distance, height) of the opening shot (OPEN)."""
        if self.OPEN == "wide":
            z = float(self.surface(*self.pads))
            look = np.array([0.55 * self.pads[0] + 0.45 * self.van[0], 0.55 * self.pads[1] + 0.45 * self.van[1], z + 1.2])
            return look, 24.0, 9.0
        return self.van + [-1.7, 0.4, 2.0], 9.9, -0.3

    def on_event(self, xyz, label="stage event", hold_s=5.0):
        self.hold = (self.vt + hold_s, np.asarray(xyz, float), label)

    def _shot(self, st, poses, active):
        ph = (st or {}).get("phase") or "deploy"
        t = float((st or {}).get("t") or 0.0)
        P = [p for p in poses.values()]
        air = [p for p in P if p[2] - float(self.surface(p[0], p[1])) > 1.0]
        pile = self.pile_xy.get(active) if active else None
        if self.hold is not None and self.vt < self.hold[0]:
            look = self.hold[1] + [0, 0, 1.0]
            return "event", look, 26.0, 12.0, None
        if ph == "deploy" and (t < 12.0 or not air):
            z = float(self.surface(*self.pads))
            if self.van is not None and not air:
                # van foreground (docs/ISAAC_ASSETS.md): from the south-west, through the open door to the operator
                # and the three screens, drones on the pads at frame left
                door, d, h = self._van_open()
                return "van", door, d, h, math.atan2(-6.9, -5.8)
            if self.van is not None:
                # lift-off: crane up beside the van and pan to the pads while the rotors spin up
                look = np.array([0.6 * self.pads[0] + 0.4 * self.van[0], 0.6 * self.pads[1] + 0.4 * self.van[1], z + 3.0])
                return "liftoff", look, 15.0, 7.0, math.atan2(-8.0, 3.0)
            return "staging", np.array([self.pads[0], self.pads[1], z + 1.5]), 16.0, 5.0, math.radians(-120.0)
        if ph in ("deploy", "dispatch", "return") and air:
            c = np.mean([p[:3] for p in air], 0)
            v = np.mean([[p[5], p[6]] for p in air], 0)
            sp = float(np.hypot(*v))
            head = math.atan2(v[1], v[0]) if sp > 1e-3 else self.az + math.pi
            look = c + [8.0 * math.cos(head), 8.0 * math.sin(head), -2.0] if sp > 1e-3 else c
            if ph == "return":
                return "follow", look, 30.0, 16.0, head + math.pi
            return "follow", look, 22.0, 10.0, head + math.pi
        if pile is not None:
            pz = float(self.surface(*pile))
            if ph in ("survey", "recon", "mapping"):
                # 34 m out, 20 m up (was 42/26): rehearsal 1 showed the drones only 40-100 px wide at the old distance
                return "survey", np.array([pile[0], pile[1], pz + 2.0]), 34.0, 20.0, None
            if ph in ("landing", "radar", "relocate", "rescan", "vitals"):
                pods = [p for p in ((st or {}).get("pods") or []) if p.get("status") in ("lowering", "scanning")]
                c = np.mean([[p["xy"][0], p["xy"][1]] for p in pods], 0) if pods else np.array(pile[:2])
                look = np.array([0.5 * c[0] + 0.5 * pile[0], 0.5 * c[1] + 0.5 * pile[1], pz + 1.5])
                return "pods", look, 28.0, 28.0 * math.tan(math.radians(self.MID_DOWN_DEG)), None
            if ph == "acoustic":
                # the landed drones (rotors off, one calling out) with the pile behind them, 28 m / 12 m: at the old
                # 48 m / 30 m wide shot the landed drones were nearly invisible (rehearsal 1)
                c = np.mean([p[:2] for p in P], 0) if P else np.array(pile[:2])
                look = np.array([0.65 * c[0] + 0.35 * pile[0], 0.65 * c[1] + 0.35 * pile[1], pz + 1.0])
                return "listen", look, 28.0, 12.0, None
        pings = (st or {}).get("pings") or []
        if ph in ("done", "battery_swap") or not P:
            c = np.mean([q["xy"] for q in pings], 0) if pings else self.base + [0, 40.0]
            return "end", np.array([c[0], c[1], 1.0]), 55.0, 50.0, None
        c = np.mean([p[:3] for p in P], 0)
        return "wide", c, 40.0, 25.0, None

    def observe(self, state, poses, active=None):
        name, look, d, h, az = self._shot(state, poses, active)
        if az is not None:
            dz = (az - self.az + math.pi) % (2 * math.pi) - math.pi
            self.az += dz * 0.03                       # swing behind the direction of travel, gently
        eye = look + [d * math.cos(self.az), d * math.sin(self.az), h]
        if self.pile_xy and name in ("survey", "pods", "listen", "event"):
            # keep the low shots over the open site: the optional town (isaac/town.py) starts just outside the pile
            # field, and a 12 m-high listen eye 28 m out must not end up inside a building
            P = np.array(list(self.pile_xy.values()), float)[:, :2]
            lo, hi = P.min(0) - 17.0, P.max(0) + 17.0
            for _ in range(8):
                if lo[0] <= eye[0] <= hi[0] and lo[1] <= eye[1] <= hi[1]:
                    break
                d, h = d * 0.88, h * 0.94
                eye = look + [d * math.cos(self.az), d * math.sin(self.az), h]
        gz = float(self.surface(eye[0], eye[1])) + (1.5 if name in ("van", "liftoff") else 4.0)
        eye[2] = max(eye[2], gz)                       # never below the rubble
        if name != self.shot or np.linalg.norm(eye - self.goal[0]) > 20.0:
            ge, gl = self._goal_now()
            dist = float(np.linalg.norm(eye - ge))
            self.trans = {"e0": ge, "l0": gl, "s": 0.0,
                          "dur": float(np.clip(dist / 12.0, 3.0, 7.0)), "lift": float(np.clip(0.3 * dist, 3.0, 18.0))}
            self.shots.append((round(self.vt, 2), name))
            self.shot = name
        self.goal = (eye, look)

    def _goal_now(self):
        """The effective goal: during a re-target the lifted Bezier arc from the old goal to the new one."""
        ge, gl = self.goal
        tr = self.trans
        if tr is None:
            return ge.copy(), gl.copy()
        s = _smooth(tr["s"])
        e0, up = tr["e0"], np.array([0.0, 0.0, tr["lift"]])
        return (_bezier(e0, e0 + up + 0.25 * (ge - e0), ge + up - 0.25 * (ge - e0), ge, s),
                tr["l0"] + (gl - tr["l0"]) * s)

    def update(self, dt, focus=None, spread=0.0):
        """Spring-follow the effective goal: position AND velocity stay continuous through every re-target (the
        arc moves the goal; the camera never jumps). isaac.motion.SmoothCamera when importable, else the same
        critically damped springs built in."""
        dt = max(float(dt), 0.0)
        self.vt += dt
        if self.shot not in ("follow", "staging", "van", "liftoff"):
            self.az += self.omega * dt                 # slow continuous orbit
        if self.trans is not None:
            self.trans["s"] = min(1.0, self.trans["s"] + dt / self.trans["dur"])
        ge, gl = self._goal_now()
        if self.trans is not None and self.trans["s"] >= 1.0:
            self.trans = None
        if self.smoother is not None:
            try:
                self.eye, self.look = (np.asarray(v, float) for v in self.smoother.update(ge, gl, dt))
                return self._apply()
            except Exception as e:                     # noqa: BLE001
                print(f"[camera] motion smoother failed ({type(e).__name__}: {e}); built-in spring", flush=True)
                self.smoother = None
        self.eye, self.v_eye = _spring(self.eye, self.v_eye, ge, dt, self.W_EYE)
        self.look, self.v_look = _spring(self.look, self.v_look, gl, dt, self.W_LOOK)
        return self._apply()

    def _apply(self):
        eye, look = self.eye.copy(), self.look.copy()
        if self.FLOAT > 0:                          # --cam-float: a stabilised aerial camera's slow float
            eye, look = cam_float(eye, look, self.vt, self.FLOAT)
        self.dc.eye, self.dc.target = eye, look
        M = self.dc._apply()
        self.updates += 1
        return M


# =============================================================================== stream frame time (>= 30 fps at 1080p)
# carb settings for the take, applied once after the stage is built (docs/VIDEO_PLAN.md section 5). UNVERIFIED on
# 6.0.1: every key is set inside try/except and logged; check them in Render Settings during the rehearsal.
TAKE_RENDER = [
    ("string", "/rtx/rendermode", "RaytracedLighting"),        # RTX real-time (not path tracing / interactive PT)
    ("int", "/rtx/post/aa/op", 3),                              # anti-aliasing = DLSS (0 none, 1 TAA, 2 FXAA, 3 DLSS)
    ("int", "/rtx/post/dlss/execMode", 0),                      # DLSS performance (0 perf, 1 balanced, 2 quality)
    ("bool", "/rtx/reflections/enabled", False),                # no glossy surfaces worth their cost on rubble
    ("bool", "/rtx/translucency/enabled", False),               # rotor blur discs use opacity, not refraction
    ("bool", "/rtx/indirectDiffuse/enabled", False),            # dusk dome + sun read fine without GI bounces
    ("int", "/rtx/directLighting/sampledLighting/samplesPerPixel", 1),
    ("bool", "/app/asyncRendering", False),                     # pose writes land in the frame they belong to
    ("int", "/app/runLoops/main/rateLimitFrequency", int(os.environ.get("LV_RATE", "60"))),   # fps opt-in: LV_RATE=30
    ("bool", "/app/runLoops/main/rateLimitEnabled", True),
]


def apply_take_render(log=None, extra=()):
    say = log or _say_default
    try:
        import carb
        cs = carb.settings.get_settings()
    except Exception as e:                                  # noqa: BLE001  (dry run)
        say(f"take render settings skipped ({type(e).__name__})")
        return 0
    n = 0
    for kind, key, val in list(TAKE_RENDER) + list(extra):
        try:
            getattr(cs, f"set_{kind}")(key, val)
            n += 1
        except Exception as e:                              # noqa: BLE001
            say(f"take render: {key} not set ({type(e).__name__}: {e})")
    say(f"take render settings: {n}/{len(TAKE_RENDER) + len(extra)} applied (RTX real-time, DLSS performance)")
    return n


# =============================================================================== CLI helpers
def write_default_tasking(out_dir, seed=7, start_pile=None, site="grid"):
    """Scenario-default zones (source ``scenario-default``: same story as the untasked CPU run) as a tasking file
    the operator then edits on the dashboard (each POST /api/tasking is a new revision)."""
    from sim.scenario import make_scenario
    from sim.tasking import default_tasking
    sc = make_scenario(seed, start_pile, site=site) if site != "grid" else make_scenario(seed, start_pile)
    tk = default_tasking(sc)
    doc = tk.to_dict() if hasattr(tk, "to_dict") else {
        "schema": "larpverse.tasking/v1", "zones": [{"id": z.id, "name": getattr(z, "name", z.id),
                                                     "polygon": [list(map(float, p)) for p in z.polygon],
                                                     "priority": getattr(z, "priority", None), "status": "tasked"}
                                                    for z in tk.zones]}
    doc = dict(doc, revision=max(1, int(doc.get("revision") or 0)), updated_at=time.strftime("%Y-%m-%dT%H:%M:%S"))
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "tasking.json")
    with open(path + ".tmp", "w") as f:
        json.dump(doc, f, indent=1)
    os.replace(path + ".tmp", path)
    return path


def write_cmd(out_dir, cmd="toggle", did=None, xyz=None):
    path = os.path.join(out_dir, "live_cmd.json")
    try:
        with open(path) as f:
            seq = int(json.load(f).get("seq", 0)) + 1
    except (OSError, ValueError):
        seq = 1
    doc = {"seq": seq, "cmd": cmd, "id": did, "wall": time.time(), **({"xyz": xyz} if xyz else {})}
    with open(path + ".tmp", "w") as f:
        json.dump(doc, f)
    os.replace(path + ".tmp", path)
    return doc


def profile_listen(seed, did, asrs, durs):
    """Where a real listen's seconds go, and what each speech-to-text mode hears (rehearsal tool, prints only)."""
    from sim.scenario import make_scenario
    sc = make_scenario(seed)
    hm = sc.heightmap(0.5)
    live = LiveWorld(sc, make_source(None, sc, surface=hm.height_at), surface=hm.height_at)
    live.set_active(did, True, "profile")
    live.refresh(0.0)
    pxy = {p.name: (float(p.x), float(p.y)) for p in sc.piles}
    sp = sc.public_dict()["start_pile"]
    cx, cy = pxy[sp]
    pods = {"d1": (cx + 3.0, cy + 2.0), "d2": (cx - 3.0, cy + 1.5), "d3": (cx, cy - 3.5)}
    for asr in asrs:
        svc = AcousticService(sc, asr=asr).start()
        try:
            for dur in durs:
                for rep in range(2):                     # first call warms the models
                    t0 = time.time()
                    r = svc.wait(svc.submit(live.truth(), pods, [sp], seed + 11 + rep, duration=dur))
                    res = r["result"]
                    print(f"\n=== asr {asr}  window {dur:g} s  run {rep + 1}: {time.time() - t0:.1f} s wall, worker "
                          f"{r['elapsed_s']} s, timing {res.get('timing_s')}", flush=True)
                    clips = {c["clip_id"]: c for c in res.get("clips", [])}
                    for p in res.get("piles", [])[:6]:
                        c = clips.get(p.get("top_clip")) or {}
                        print(f"  {p['pile_id']:3} pri {p['priority']:>3} {p['status']:22} {c.get('sound_class', '-'):8} "
                              f"hum {c.get('human', 0):.2f} urg {c.get('urgency', 0):.2f} lang {c.get('language')} "
                              f"\"{c.get('transcript', '')}\" orig \"{c.get('transcript_original', '')}\"", flush=True)
                    print(f"  dispatch -> {(res.get('dispatch') or {}).get('pile_id')}", flush=True)
        finally:
            svc.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.join(REPO, "dashboard", "data"))
    ap.add_argument("--write-tasking", choices=["default"], default=None)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--site", default="grid")
    ap.add_argument("--cmd", choices=["toggle", "activate", "deactivate", "move"], default=None)
    ap.add_argument("--id", default=None, help="dummy id (default: the first hidden dummy)")
    ap.add_argument("--xyz", default=None, help="move: x,y,z")
    ap.add_argument("--gemini", type=int, choices=[0, 1], default=1,
                    help="--preflight: include the Gemini audio stage (costs calls from the ~20/day free tier)")
    ap.add_argument("--preflight", action="store_true",
                    help="start the real acoustic pipeline worker (.venv-audio) and run one listen on the live dummies; "
                         "exit 1 LOUDLY if the take would fall back to stub acoustics")
    ap.add_argument("--profile-listen", action="store_true",
                    help="real pipeline timing: one hidden voice on (default H2_A1_id), drones at the start pile's pod "
                         "points; listens for each --asr x --dur variant; prints per-stage timing and what each pile got")
    ap.add_argument("--asr", default="en,multi", help="--profile-listen: comma list of en / multi")
    ap.add_argument("--dur", default="26,10", help="--profile-listen: comma list of listening windows (s)")
    a = ap.parse_args(argv)
    if a.profile_listen:
        profile_listen(a.seed, a.id or "H2_A1_id", a.asr.split(","), [float(v) for v in a.dur.split(",")])
        return
    if a.preflight:
        from sim.scenario import make_scenario
        sc = make_scenario(a.seed, site=a.site) if a.site != "grid" else make_scenario(a.seed)
        hm = sc.heightmap(0.5)
        live = LiveWorld(sc, make_source(None, sc, surface=hm.height_at), surface=hm.height_at)
        hid = live.source.hidden_ids()
        if hid:
            live.set_active(hid[-1], True, "preflight")      # check a CHANGED world too (fresh, uncached synthesis)
            live.refresh(0.0)
        try:
            svc, info = preflight_acoustic(sc, live, gemini=bool(a.gemini))
        except Exception as e:                               # noqa: BLE001
            print("\n" + "!" * 78 + f"\nACOUSTIC PRE-FLIGHT FAILED: {e}\nThe take would use STUB acoustics. Fix it "
                  "(docs/VIDEO_PLAN.md section 6) before rolling.\n" + "!" * 78, flush=True)
            sys.exit(1)
        svc.close()
        print(json.dumps({"preflight": "PASS", **info}))
        return
    if a.write_tasking:
        print(write_default_tasking(a.out, a.seed, site=a.site))
    if a.cmd:
        print(json.dumps(write_cmd(a.out, a.cmd, a.id, [float(v) for v in a.xyz.split(",")] if a.xyz else None)))
    if not (a.write_tasking or a.cmd):
        ap.print_help()


if __name__ == "__main__":
    try:
        main()
    except Exception:                                       # noqa: BLE001
        traceback.print_exc()
        sys.exit(1)
