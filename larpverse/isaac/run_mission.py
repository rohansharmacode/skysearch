"""Run the 3x3 rubble-grid SAR mission inside Isaac Sim (or ``--dry-run`` without omni/pxr).

    # Isaac Sim 6.0.x + Isaac Lab 3.0 (Brev launchable; the route the team verified on the L40S):
    cd /workspace/isaaclab && ./isaaclab.sh -p /workspace/skysearch/larpverse/isaac/run_mission.py --selftest --headless
    ./isaaclab.sh -p .../isaac/run_mission.py --livestream 2 --viz kit --speed 4 --show-truth   # watch at <brev URL>/viewer
    # Isaac Sim 4.5 / 5.x standalone (SimulationApp route):
    ~/isaacsim/python.sh isaac/run_mission.py --seed 7 --speed 4
    python isaac/run_mission.py --dry-run --max-steps 60                          # any laptop, numpy only
    # Sites (docs/SITE_WEIGUAN.md): --site grid (default) | collapse (real expanded site) | weiguan (procedural)
    #   | "mesh:<map.npz|.ply|.obj|.usd>[?sectors=weiguan]"
    ./isaaclab.sh -p /workspace/skysearch/larpverse/isaac/run_mission.py --site collapse \
        --site-usd /workspace/skysearch/collapse_site/collapse_site.usda --headless --show-truth --speed 4
    # Footage (isaac/photoreal.py, verified on the L40S; --launcher kit = the exact route it was recorded on):
    ... run_mission.py --headless --radar dsp --show-truth --photoreal 1 --record out/lv1 \
        --rec-cams director,wide,chase:d1,fpv:d2,top --rec-res 1920x1080 --rec-fps 30 --rec-speed 6 --rec-warp 4 \
        --rec-stop-t 250 [--rec-windows "director:120-250;wide:0-110"] [--rtx pathtracing --spp 16]

The brain is ``sim.grid_mission.GridMission`` (numpy; never sees ground truth). Isaac is the world and the
body: the stage is built from the same seeded ``sim.scenario``; drones are posed kinematically from the
mission state (interpolated every render frame); a spinning LiDAR raycasts the PhysX colliders while drones
survey and the points go back to the mission through ``ingest_pointcloud`` when a pile's survey ends.
``--replay DIR`` plays recorded ``frames/state_XXXX.json`` instead (also the fallback if the live import fails).
The mission writes ``<out>/state.json`` so the web dashboard shows the Isaac run live.
"""
from __future__ import annotations

import argparse
import glob
import inspect
import json
import math
import os
import sys
import time
import traceback

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

SURVEY_STATES = {"surveying", "survey", "mapping", "lidar"}
GROUNDED = {"docked", "landed", "scanning", "listening", "idle", "charging"}
LIGHT_DEFAULTS = {"sun": (1800.0, 2200.0), "sky": (350.0, 900.0)}   # dest: (basic look L40S-measured, photoreal)


def _base_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
                                 conflict_handler="resolve")
    ap.add_argument("--launcher", choices=["auto", "lab", "kit"], default="auto",
                    help="lab = Isaac Lab AppLauncher + SimulationContext (./isaaclab.sh -p, verified on 6.0.1); "
                         "kit = SimulationApp + timeline (~/isaacsim/python.sh, 4.5/5.x); auto picks lab if importable")
    ap.add_argument("--selftest", action="store_true",
                    help="check every risky Isaac call in order, print PASS/WARN/FAIL per step, save a frame, exit")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--start-pile", default=None, help="pile to begin at, e.g. B2 (default: scenario's)")
    ap.add_argument("--site", default="grid",
                    help="grid (synthetic 3x3) | collapse (the 3d_model map, isaac/collapse_site.py) | weiguan "
                         "(procedural stand-in) | mesh:<path>[?sectors=weiguan]")
    ap.add_argument("--site-usd", default=None,
                    help="--site collapse: path to collapse_site.usda (default: <repo>/collapse_site/collapse_site.usda "
                         "on branch 3d_model, $LARPVERSE_COLLAPSE_USD, next to the repo)")
    ap.add_argument("--site-lights", choices=["auto", "site", "ours", "both"], default="auto",
                    help="lights with a referenced site USD: auto = ours under --photoreal, else the site's own Sun/Sky")
    ap.add_argument("--no-scan-textures", dest="scan_textures", action="store_false",
                    help="drop the albedo maps derived from the third-party photogrammetry screenshots (licence unknown)")
    ap.add_argument("--tasking", default=None,
                    help="operator tasking zones (sim/tasking.py, contracts v1.2): PATH = tasking file re-read live "
                         "(scripts/serve.py POST /api/tasking writes <out>/tasking.json), auto = <out>/tasking.json, "
                         "default = the scenario's zones, none / unset = legacy (every sector)")
    ap.add_argument("--radar", choices=["model", "dsp"], default="dsp", help="dsp needs scipy in Isaac's python")
    g = ap.add_argument_group("mission (passed to sim.grid_mission.GridMission when it supports them)")
    g.add_argument("--platform", default=None, help="sim/platform.py preset: legacy_small | heavy_sar | industrial_sar")
    g.add_argument("--penetration", default=None, help="radar through-rubble model (sim/radar_dsp.py): literature | legacy")
    g.add_argument("--delivery", choices=["pod", "land"], default=None,
                   help="radar delivery: pod = winch-lowered pod (drawn hanging/resting), land = the drone perches")
    g.add_argument("--acoustic", choices=["auto", "stub", "real"], default=None,
                   help="acoustic agent (sim/grid_mission.py): stub = synthetic clips from the hidden truth (labelled), "
                        "real = the acoustic pipeline, auto = real when available")
    g.add_argument("--no-autonomous", dest="autonomous", action="store_false",
                   help="legacy fixed loop: 3 piles, no battery swaps, no revisits")
    ap.add_argument("--speed", type=float, default=4.0, help="sim-seconds per wall-second")
    ap.add_argument("--show-truth", action="store_true",
                    help="draw the scenario's hidden (invented) people as grey dummies, half sunk (display only)")
    ap.add_argument("--overlay-style", choices=["full", "simple"], default="full",
                    help="simple: only what matters now (current/new zone, latest dispatch, firm pings) for the judges")
    ap.add_argument("--line-scale", type=float, default=1.0, help="multiply overlay line and sensor-effect widths "
                    "(zones, dispatch arcs, rings, LiDAR beams, sound domes) for wide shots; the take uses ~4")
    ap.add_argument("--fx", type=int, default=1, help="live sensor effects (isaac/sensor_fx.py): LiDAR beams, "
                    "radar pulses, sound domes while listening, belief hotspot (0 = off)")
    ap.add_argument("--replay", default=None, help="dir with frames/state_XXXX.json (or the frames dir itself)")
    ap.add_argument("--out", default=os.path.join(REPO, "dashboard", "data"), help="state.json output dir")
    ap.add_argument("--frames", action="store_true", help="also write frames/state_XXXX.json")
    ap.add_argument("--dry-run", action="store_true", help="no omni/pxr: numpy heightmap LiDAR, null drawing")
    ap.add_argument("--max-steps", type=int, default=0, help="stop after N mission steps (0 = until done)")
    ap.add_argument("--comms", type=int, choices=[0, 1], default=None,
                    help="GridMission comms (drone Wi-Fi hotspot, phone portal leads): default = the mission's own (on); "
                         "0 = off, e.g. so a portal lead cannot pre-empt the hidden-dummy redirect")
    ap.add_argument("--stop-t", type=float, default=0.0,
                    help="freeze the picture at this sim time (s); with the stream open the last frame stays up")
    ap.add_argument("--hold-file", default="",
                    help="render, but hold at t 0 (before the pre-roll) until this file exists: arm the stream capture, "
                         "then touch it, so the take starts at launch")
    ap.add_argument("--stream-res", default="1920x1080",
                    help="--livestream: hide Kit's editor UI and size the window to this, so the stream is the full-frame "
                         "viewport at a constant resolution ('' = leave Kit's window alone)")
    ap.add_argument("--take-open", choices=["van", "wide"], default="van",
                    help="director-take opening shot: van = close on the open door (team default); wide = establishing "
                         "shot from 24 m: van, drones on the pads and the rubble field in one frame")
    ap.add_argument("--cinema", type=int, default=0,
                    help="1 = feature-film look (isaac/cinema.py): shutter motion blur, subtle lens aberration and "
                         "bloom, grain, dusty volumetric air, film grade. Pair with --hdri "
                         "kloofendal_48d_partly_cloudy_4k.hdr --sun 1600 --sun-elev 32 for the disaster-film sky")
    ap.add_argument("--cam-float", type=float, default=0.0,
                    help="director-take: slow drift of a gimbal-stabilised drone camera (1 = real aerial footage, "
                         "~0.25%% of the camera distance; 0 = locked tripod)")
    ap.add_argument("--camera", choices=["director", "director-take", "story", "free"], default="director",
                    help="director-take: one continuous spline camera for the single take (isaac/live.py TakeCamera); "
                         "story: state-driven framing of the active/new zone and dispatch, slow orbit, springs "
                         "(isaac/camera.py StoryCamera)")
    g = ap.add_argument_group("scene assets (docs/ISAAC_ASSETS.md)")
    g.add_argument("--van", type=int, choices=[0, 1], default=None,
                   help="command van at staging (isaac/van.py; default 1 with --photoreal 1, else 0); forces --operator 0")
    g.add_argument("--van-mdl", type=int, choices=[0, 1], default=0,
                   help="add OmniPBR outputs to the van's emissive screens/lamps if PreviewSurface emission reads too dim")
    g.add_argument("--dummies", type=int, choices=[0, 1], default=1,
                   help="victim manikins with SpatialAudio + larpverse:* attrs (isaac/dummies.py); replaces --show-truth ghosts")
    g.add_argument("--ambient", type=int, choices=[0, 1], default=1, help="--dummies: wind bed + distant siren")
    g = ap.add_argument_group("live single take (isaac/live.py, isaac/hud.py, docs/VIDEO_PLAN.md)")
    g.add_argument("--live", dest="live_take", action="store_true",
                   help="live take: the stage dummies are the victims (read every scan / listen), unscripted triggers "
                        "(hotkey, dashboard zone, viewport drag, portal), tasking polled every 1 s, live.json + hud.json")
    g.add_argument("--tasking", default=None,
                   help="none | default | auto (= <out>/tasking.json, re-read live) | PATH. Default: auto with --live, "
                        "else none (legacy). Seed editable zones: python isaac/live.py --write-tasking default")
    g.add_argument("--acoustic", choices=["auto", "stub", "real"], default=None,
                   help="acoustic generator/pipeline (default auto). With --live and a changed stage every listen is a "
                        "fresh uncached synthesis: real in-process if importable, else the synthetic generator")
    g.add_argument("--hud", choices=["off", "auto", "overlay", "window"], default=None,
                   help="in-Isaac perception HUD (default auto with --live, else off; no-op without omni.ui)")
    g.add_argument("--live-pace", default=None,
                   help="--live: sim s per video s per phase, e.g. 'survey=3,radar=12,relocate=24' (default: the take "
                        "table in isaac/live.py DEFAULT_PACE; 'off' = constant --speed). The HUD always shows the factor")
    g.add_argument("--live-preroll", type=float, default=None,
                   help="--live: video seconds the mission waits on the opening (van) shot before deploying (default 4)")
    g.add_argument("--allow-stub-acoustic", action="store_true",
                   help="--live outside --dry-run: start even if the REAL acoustic pipeline (.venv-audio worker) fails "
                        "its pre-flight. Rehearsal only: a take must not use the stub (docs/VIDEO_PLAN.md section 6)")
    g.add_argument("--live-gemini", type=int, choices=[0, 1], default=0,
                   help="--live real pipeline: Gemini audio stage on (adds seconds per listen + 10 RPM throttling)")
    g.add_argument("--take-render", type=int, choices=[0, 1], default=None,
                   help="--live: stream frame-time settings (RTX real-time, DLSS performance, no reflections / "
                        "translucency / GI; isaac/live.py TAKE_RENDER). Default 1 with --live unless --rtx pathtracing")
    g.add_argument("--live-warp", type=float, default=1.0,
                   help="--live: multiply --speed by this during radar/rescan/relocate (time-lapse; shown on the HUD)")
    g.add_argument("--live-asr", choices=["en", "multi"], default=None,
                   help="--live real pipeline speech-to-text: en = the pipeline's English-only Whisper (default); multi "
                        "= multilingual Whisper (language id + English translation; the Indonesian / Vietnamese voices "
                        "come out as words instead of English-sounding nonsense). Env LARPVERSE_ASR also sets it")
    g.add_argument("--live-quick-s", type=float, default=None,
                   help="--live: pod-mic quick listen every N sim s while pods rest on the rubble (default 20; 0 = off)")
    g.add_argument("--no-preempt", dest="preempt", action="store_false",
                   help="--live: never cut a radar phase short for new evidence / an operator zone (queue it instead)")
    g.add_argument("--live-keys", default=None, help="--live hotkeys, e.g. 'U=hidden1,Y=hidden2' or 'U=H1'")
    g.add_argument("--mission-thread", type=int, default=None,
                   help="step the mission on a worker thread so slow steps never stall a frame (default 1 with --live "
                        "outside --dry-run, else 0)")
    g.add_argument("--test-trigger", default=None,
                   help="TESTS / DRY RUN ONLY (refused otherwise): scripted trigger, e.g. 'activate:hidden1@150'")
    ap.add_argument("--drone-asset", nargs="?", const="auto", default=None,
                    help="reference Isaac's quadcopter USD (optionally a path/URL) instead of primitives")
    ap.add_argument("--drone-scale", type=float, default=1.6, help="small model only (heavy is true scale)")
    ap.add_argument("--drone-model", choices=["heavy", "small"], default="heavy",
                    help="heavy: FlyCart/T50-class coaxial X8 from sim.platform (docs/PLATFORM.md); small: old quad")
    ap.add_argument("--rtx-lidar", action="store_true", help="try RTX LiDAR (falls back to PhysX raycasts)")
    ap.add_argument("--zones", dest="zones", action="store_true", default=None,
                    help="draw operator zones / recon / proposals (default: on unless LARPVERSE_ZONES=0)")
    ap.add_argument("--no-zones", dest="zones", action="store_false")
    ap.add_argument("--overlay", choices=["auto", "debug", "usd"], default="auto",
                    help="debug = debug_draw lines/points; usd = prims only (safest for Movie Capture)")
    ap.add_argument("--rays", type=int, default=None, help="pin LiDAR rays per surveying drone per frame "
                    "(default: the --lidar-profile's: small 96, heavy auto-sized to --lidar-budget-ms)")
    ap.add_argument("--lidar-range", type=float, default=None, help="default: profile's (small 40, heavy 60)")
    from isaac.lidar import add_lidar_args
    add_lidar_args(ap)
    ap.add_argument("--render-fps", type=float, default=30.0, help="virtual frame rate for --dry-run (--record: --rec-fps)")
    ap.add_argument("--no-keep-open", action="store_true", help="close the GUI when the mission ends")
    ap.add_argument("--quiet", action="store_true")
    g = ap.add_argument_group("look (docs/ISAAC_VERIFIED_APIS.md)")
    g.add_argument("--look", choices=["basic", "pretty", "plain"], default="basic",
                   help="basic = UsdPreviewSurface materials; pretty = alias for --photoreal 1 (isaac/photoreal.py: MDL "
                        "materials, irregular rubble, HDRI dusk sky, debris, vegetation) + Flow dust; "
                        "plain = bare displayColor (diagnosis)")
    g.add_argument("--flow", type=int, default=1, help="--look pretty: Omni Flow dust under landing drones (0 = off)")
    g = ap.add_argument_group("Isaac Lab")
    g.add_argument("--fabric", action="store_true", help="Isaac Lab use_fabric=True (default off: pxr pose edits render)")
    # --photoreal, --assets, --hdri, --sky, --sun, --sun-elev, --veg, --operator, --record, --rec-*, --rtx, --spp:
    # the Isaac teammate's GPU-verified implementation (larpverse-isaac) is canonical; see isaac/photoreal.py.
    from isaac.photoreal import add_args as _photoreal_args
    _photoreal_args(ap)
    for act in ap._actions:                 # --sun/--sky: default depends on the look (resolved in parse_args)
        if act.dest in LIGHT_DEFAULTS:
            act.default = None
            b, p = LIGHT_DEFAULTS[act.dest]
            act.help = f"{act.help} (default {p:g} with --photoreal/--look pretty, else {b:g})"
    return ap


def parse_args(argv=None):
    """Two-phase parse: on the Isaac Lab route AppLauncher adds its own flags (--headless, --livestream,
    --enable_cameras, --device, --kit_args, ...); on the Kit route we add look-alikes, so the same command line
    works on both. Unknown flags are reported, not fatal."""
    argv = list(sys.argv[1:] if argv is None else argv)
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--dry-run", action="store_true")
    pre.add_argument("--launcher", default="auto")
    p, _ = pre.parse_known_args(argv)
    if p.dry_run:
        launcher = "none"
    else:
        from isaac import compat
        launcher = compat.resolve_launcher(p.launcher)
    ap = _base_parser()
    if launcher == "lab":
        from isaac import compat
        compat.add_lab_args(ap)
    else:
        from isaac.compat import add_kit_args
        add_kit_args(ap)
    a, unknown = ap.parse_known_args(argv)
    a.launcher, a.unknown = launcher, unknown
    if unknown:
        print(f"[isaac] ignoring unknown arguments: {unknown}", flush=True)
    if a.look == "pretty":                  # --look pretty is a thin alias for --photoreal 1
        a.photoreal = a.photoreal or 1
    for k, (basic, photo) in LIGHT_DEFAULTS.items():
        if getattr(a, k) is None:
            setattr(a, k, photo if a.photoreal else basic)
    live = int(getattr(a, "livestream", 0) or 0)
    a.live = live > 0
    if a.live and a.stream_res and hasattr(a, "kit_args"):     # Lab route: whole editor at 1440x900 otherwise
        w, h = (int(v) for v in a.stream_res.lower().split("x"))
        ka = str(a.kit_args or "")
        extra = [f for f in ("--/app/window/hideUi=true", f"--/app/window/width={w}", f"--/app/window/height={h}")
                 if f.split("=")[0] not in ka]
        a.kit_args = " ".join([ka] + extra).strip()
    from isaac.lidar import profile as lidar_profile     # resolve the LiDAR profile once (selftest reads a.rays)
    a.lidar_prof = lidar_profile(a.lidar_profile, a.lidar_budget_ms, a.rays, a.lidar_range)
    a.rays, a.lidar_range = a.lidar_prof["n_rays"], a.lidar_prof["max_range"]
    a.keep_open = not (a.dry_run or a.no_keep_open or a.record or a.selftest or (a.headless and not a.live))
    if a.tasking is None:
        a.tasking = "auto" if a.live_take else "none"
    if a.van is None:
        a.van = 1 if a.photoreal else 0
    if a.van and hasattr(a, "operator"):
        a.operator = 0                      # the van has its own seated operator
    if a.mission_thread is None:
        a.mission_thread = 1 if (a.live_take and not a.dry_run) else 0
    if a.hud is None:
        a.hud = "auto" if a.live_take else "off"
    if a.test_trigger and not a.dry_run:
        raise SystemExit("--test-trigger is for tests / dry-run rehearsals only: the take's change must be a live action")
    if a.live_take and a.replay:
        raise SystemExit("--live needs the live brain (GridMission), not --replay")
    return a


# ---------------------------------------------------------------------------- missions
def as_dict(s):
    return json.loads(s) if isinstance(s, (str, bytes)) else s


def _atomic_write(path, text):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(text)
    os.replace(tmp, path)


class ReplayMission:
    """Plays recorded state frames with the GridMission interface (step/done/t/build_state/scenario)."""

    def __init__(self, replay_dir, out_dir=None, seed=7, start_pile=None, site="grid"):
        from sim.scenario import make_scenario
        files = sorted(glob.glob(os.path.join(replay_dir, "frames", "state_*.json"))) or \
            sorted(glob.glob(os.path.join(replay_dir, "state_*.json")))
        if not files:
            raise FileNotFoundError(f"no state_XXXX.json frames under {replay_dir}")
        self.files, self.i, self.out_dir = files, 0, out_dir
        with open(files[0]) as f:
            self.state = json.load(f)
        meta = self.state.get("scenario") if isinstance(self.state.get("scenario"), dict) else {}
        seed = int(self.state.get("seed", meta.get("seed", seed)))
        site = (self.state.get("site") or {}).get("kind") or site
        start = start_pile or meta.get("start_pile") or self.state.get("start_pile")
        from isaac import collapse_site
        self.scenario = (collapse_site.make_collapse_scenario(seed, start) if collapse_site.is_core(site)
                         else make_scenario(seed, start, site=site))
        self.t = float(self.state.get("t", 0.0))
        self.dt = 1.0
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

    @property
    def done(self):
        return self.i >= len(self.files) - 1

    def step(self):
        if self.done:
            return
        self.i += 1
        with open(self.files[self.i]) as f:
            text = f.read()
        self.state = json.loads(text)
        self.t = float(self.state.get("t", self.t + 1.0))
        if self.out_dir:
            _atomic_write(os.path.join(self.out_dir, "state.json"), text)

    def build_state(self):
        return self.state


def load_mission(a, log):
    if a.replay:
        log(f"replaying {a.replay}")
        rm = ReplayMission(a.replay, a.out, a.seed, a.start_pile, a.site)
        for mj in (os.path.join(a.replay, "motion.json"), os.path.join(os.path.dirname(a.replay.rstrip("/")), "motion.json")):
            if os.path.exists(mj):                  # SmoothFlight.save_motion() (docs/MOTION.md R1): render-time poses
                os.environ["LARPVERSE_MOTION_JSON"] = mj
                log(f"flight motion from {mj}")
                break
        return rm
    try:
        from isaac.site import mission_for_site
        extra = {k: getattr(a, k) for k in ("platform", "penetration", "delivery", "acoustic")
                 if getattr(a, k, None) is not None}
        extra["autonomous"] = getattr(a, "autonomous", True)
        if getattr(a, "comms", None) is not None:
            extra["comms"] = bool(a.comms)
        tk = getattr(a, "tasking", None)
        if tk not in (None, "none"):
            extra["tasking"] = os.path.join(a.out, "tasking.json") if tk == "auto" else tk
            log(f"operator tasking: {extra['tasking']}")
        if getattr(a, "acoustic", None):
            extra["acoustic"] = a.acoustic
        return mission_for_site(a.site, seed=a.seed, start_pile=a.start_pile, radar=a.radar, out_dir=a.out,
                                frames=a.frames, verbose=not a.quiet, log=log, **extra)
    except Exception:
        err = traceback.format_exc()
        fb = os.path.join(a.out, "frames")
        if glob.glob(os.path.join(fb, "state_*.json")):
            log(f"live GridMission failed, replaying {fb} instead:\n{err}")
            return ReplayMission(fb, a.out, a.seed, a.start_pile, a.site)
        log("live GridMission failed and no recorded frames to replay "
            f"(record with: python -m sim.grid_mission --frames --out {a.out})")
        raise


# ---------------------------------------------------------------------------- helpers
def _lerp_angle(a, b, u):
    d = (b - a + 180.0) % 360.0 - 180.0
    return a + d * u


def interp_drones(prev, cur, u):
    """{id: (x, y, z, yaw, state, vx, vy)} linearly interpolated between two states."""
    pv = {d["id"]: d for d in prev.get("drones", [])}
    out = {}
    for d in cur.get("drones", []):
        p1 = d["pose"]
        p0 = pv.get(d["id"], d)["pose"]
        xyz = [p0[k] + (p1[k] - p0[k]) * u for k in range(3)]
        out[d["id"]] = (*xyz, _lerp_angle(p0[3], p1[3], u), d.get("state", ""), p1[0] - p0[0], p1[1] - p0[1])
    return out


def _mono_tangent(d0, d1):
    """Monotone (Fritsch-Carlson style) tangent from the two neighbouring slopes: no overshoot, so the skids never
    dip under a landing pad and a stopping drone never backs up."""
    if d0 * d1 <= 0.0:
        return 0.0
    m = 0.5 * (d0 + d1)
    lim = 3.0 * min(abs(d0), abs(d1))
    return max(-lim, min(lim, m))


class PoseTrack:
    """Render-time drone poses: every rendered frame samples the trajectory at the continuous render time
    ``t_render`` (sim seconds), never the last sim step.

    With ``isaac.motion`` (the flight-motion agent: jerk-limited trajectories, tilt into acceleration, flare,
    rotor RPM) its ``pose_at(drone_id, t)`` is used and the states are handed to it with ``ingest(state)``.
    Until it lands: a monotone cubic Hermite through the mission's step states (C1: continuous velocity, so
    tilt and camera follow are continuous too). The mission runs at least one step ahead of ``t_render``, so the
    curve always has the next point (no extrapolation)."""

    KEEP = 12

    def __init__(self, log=None, use_motion=True):
        self.hist = []                      # [(t, {id: (x, y, z, yaw, state)})]
        self.motion, self.motion_err = None, None
        self.say = log or (lambda m: None)
        if use_motion:
            try:
                import isaac.motion as M
                obj = M.Motion() if hasattr(M, "Motion") else M
                if hasattr(obj, "pose_at"):
                    self.motion = obj
                    self.say("drone poses from isaac.motion.pose_at (render-time trajectories)")
            except ImportError:
                pass
            except Exception as e:          # noqa: BLE001
                self.say(f"isaac.motion unavailable ({type(e).__name__}: {e}); Hermite render-time interpolation")

    def push(self, state):
        t = float(state.get("t", 0.0))
        if self.hist and t <= self.hist[-1][0] + 1e-9:
            self.hist.pop()                 # same t (e.g. a re-plan rebuilt the state): keep the newest
        self.hist.append((t, {d["id"]: (*map(float, d["pose"][:4]), d.get("state", "")) for d in state.get("drones", [])}))
        del self.hist[:-self.KEEP]
        if self.motion is not None and hasattr(self.motion, "ingest"):
            try:
                self.motion.ingest(state)
            except Exception as e:          # noqa: BLE001
                self._drop_motion(e)

    def _drop_motion(self, e):
        self.motion_err = f"{type(e).__name__}: {e}"
        self.say(f"isaac.motion failed ({self.motion_err}); falling back to Hermite interpolation")
        self.motion = None

    @property
    def t_first(self):
        return self.hist[0][0] if self.hist else 0.0

    @property
    def t_last(self):
        return self.hist[-1][0] if self.hist else 0.0

    MISS_DROP = 90                          # consecutive frames without a trajectory -> stop asking (Kinematic)

    def poses(self, t):
        """{id: (x, y, z, yaw_deg, state, vx, vy[, tilt, raw])}; vx, vy in m/s. With the motion model: tilt (roll,
        pitch) deg and its raw dict ({xyz, quat (w,x,y,z), rotor_rpm, winch_len, ground_z, tilt, vel, yaw}), which
        the loop hands to DroneViz.set_state."""
        if self.motion is not None:
            try:
                out = self._motion_poses(t)
                self.misses = 0
                return out
            except KeyError as e:           # no trajectory (KinematicFlight mission, or not planned yet)
                self.misses = getattr(self, "misses", 0) + 1
                if self.misses >= self.MISS_DROP:
                    self._drop_motion(e)
            except Exception as e:          # noqa: BLE001
                self._drop_motion(e)
        H = self.hist
        if not H:
            return {}
        t = min(max(t, H[0][0]), H[-1][0])
        k = 0
        while k < len(H) - 2 and H[k + 1][0] <= t:
            k += 1
        if len(H) == 1:
            return {i: (*p[:4], p[4], 0.0, 0.0) for i, p in H[0][1].items()}
        t0, t1 = H[k][0], H[k + 1][0]
        h = max(t1 - t0, 1e-6)
        s = min(max((t - t0) / h, 0.0), 1.0)
        out = {}
        for did, p1 in H[k + 1][1].items():
            p0 = H[k][1].get(did, p1)
            pm = H[k - 1][1].get(did, p0) if k > 0 else p0
            p2 = H[k + 2][1].get(did, p1) if k + 2 < len(H) else p1
            hm = (t0 - H[k - 1][0]) if k > 0 else h
            h2 = (H[k + 2][0] - t1) if k + 2 < len(H) else h
            vals, ders = [], []
            for ax in range(4):
                a, b, c, d = pm[ax], p0[ax], p1[ax], p2[ax]
                if ax == 3:                 # yaw: unwrap around p0
                    a = b + ((a - b + 180.0) % 360.0 - 180.0)
                    c = b + ((c - b + 180.0) % 360.0 - 180.0)
                    d = c + ((d - c + 180.0) % 360.0 - 180.0)
                s0, s1, s2 = (b - a) / max(hm, 1e-6), (c - b) / h, (d - c) / max(h2, 1e-6)
                m0, m1 = _mono_tangent(s0, s1) * h, _mono_tangent(s1, s2) * h
                h00, h10, h01, h11 = 2 * s ** 3 - 3 * s ** 2 + 1, s ** 3 - 2 * s ** 2 + s, -2 * s ** 3 + 3 * s ** 2, s ** 3 - s ** 2
                vals.append(h00 * b + h10 * m0 + h01 * c + h11 * m1)
                d00, d10, d01, d11 = 6 * s ** 2 - 6 * s, 3 * s ** 2 - 4 * s + 1, -6 * s ** 2 + 6 * s, 3 * s ** 2 - 2 * s
                ders.append((d00 * b + d10 * m0 + d01 * c + d11 * m1) / h)
            st = p0[4] if s < 0.5 else p1[4]
            out[did] = (vals[0], vals[1], vals[2], vals[3] % 360.0, st, ders[0], ders[1])
        return out

    def _motion_poses(self, t):
        out = {}
        ids = list(self.hist[-1][1]) if self.hist else []
        for did in ids:
            p = self.motion.pose_at(did, t)
            if isinstance(p, dict):
                x, y, z = p["xyz"][:3]
                v = p.get("vel") or (0.0, 0.0)
                out[did] = (float(x), float(y), float(z), float(p.get("yaw", p.get("yaw_deg", 0.0))),
                            p.get("state", self.hist[-1][1][did][4]), float(v[0]), float(v[1]),
                            tuple(p["tilt"]) if p.get("tilt") is not None else None, p)
            else:
                out[did] = tuple(p)
        return out


class MissionRunner:
    """Steps the mission ahead of the render clock: inline (default; deterministic) or on a worker thread
    (``--mission-thread``: a slow radar/acoustic step no longer stalls a rendered frame). ``target(t)`` asks for
    states up to sim time t; ``drain()`` returns the new states in order. The worker never touches USD: stage
    reads / writes go through a main-thread cache (isaac.live.SyncedSource)."""

    def __init__(self, step_fn, done_fn, threaded=False):
        import threading
        self.step_fn, self.done_fn, self.threaded = step_fn, done_fn, threaded
        self.q, self.err, self._target, self.latest_t = [], None, -1e9, None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.thread = None
        if threaded:
            self.thread = threading.Thread(target=self._loop, name="larpverse-mission", daemon=True)
            self.thread.start()

    def _one(self):
        new = self.step_fn()
        with self._lock:
            self.q.append(new)
            self.latest_t = float(new.get("t", 0.0))

    def _loop(self):
        while not self._stop.is_set():
            try:
                if not self.done_fn() and (self.latest_t is None or self.latest_t < self._target):
                    self._one()
                else:
                    self._stop.wait(0.002)
            except BaseException as e:     # noqa: BLE001  (surface it on the main thread)
                self.err = e
                return

    def target(self, t):
        self._target = t
        if not self.threaded:
            while not self.done_fn() and (self.latest_t is None or self.latest_t < t):
                self._one()
        elif self.err is not None:
            raise self.err

    def drain(self):
        with self._lock:
            q, self.q = self.q, []
        return q

    @property
    def idle(self):
        """No state pending and the mission will not produce more."""
        with self._lock:
            empty = not self.q
        return empty and self.done_fn()

    def stop(self):
        self._stop.set()
        if self.thread is not None:
            self.thread.join(timeout=5.0)


def is_surveying(drone_state, phase, airborne):
    if drone_state in SURVEY_STATES:
        return True
    return airborne and phase in ("survey", "surveying", "recon_mapping") and drone_state in ("flying", "hovering")


def nearest_pile(pile_xy, x, y, max_d=22.0):
    best = min(pile_xy, key=lambda k: math.hypot(pile_xy[k][0] - x, pile_xy[k][1] - y))
    return best if math.hypot(pile_xy[best][0] - x, pile_xy[best][1] - y) <= max_d else None


def call_ingest(mission, pts, pile, log):
    """Hand new world-frame LiDAR points to the mission. GridMission buffers them and rebuilds the active
    pile's planning window when its survey ends, so points are fed every step DURING the survey."""
    fn = getattr(mission, "ingest_pointcloud", None)
    if fn is None or len(pts) == 0:
        return False
    try:
        params = inspect.signature(fn).parameters
        if "pile" in params:
            fn(pts, pile=pile)
        elif "pile_id" in params:
            fn(pts, pile_id=pile)
        else:
            fn(pts)
        return True
    except Exception:
        log(f"ingest_pointcloud failed for {pile}:\n{traceback.format_exc()}")
        return False


# ---------------------------------------------------------------------------- main
def main(argv=None):
    a = parse_args(argv)
    say = (lambda m: None) if a.quiet else (lambda m: print(f"[isaac] {m}", flush=True))
    if a.selftest:
        from isaac.selftest import run as selftest
        return selftest(a, say)
    rt = app = None
    if not a.dry_run:                       # the app FIRST, before any omni/pxr import
        from isaac import compat
        rt = compat.make_runtime(a)
        app = rt.boot()
        for m in rt.log:
            say(m)
        say(f"launcher={rt.mode} {compat.versions()}")
    import numpy as np

    from isaac.camera import DirectorCamera, StoryCamera, focus_from_state, story_focus
    from isaac.drone_viz import DroneViz, NullDrone
    from isaac.lidar import HeightmapLidar, PointAccumulator, RtxLidar, make_lidar
    lprof = a.lidar_prof
    from isaac.overlay import Overlay, make_backend

    if a.out:
        os.makedirs(a.out, exist_ok=True)
    if a.site_usd:
        os.environ["LARPVERSE_COLLAPSE_USD"] = os.path.abspath(a.site_usd)
    mission = load_mission(a, say)
    sc = mission.scenario
    pile_xy = {p.name: (float(p.x), float(p.y)) for p in sc.piles}
    t0 = time.time()
    hm = sc.heightmap(0.5)                  # public geometry: overlay draping + dry-run LiDAR target
    say(f"site {getattr(sc, 'site', 'grid')}: {len(sc.piles)} sectors; heightmap {hm.w}x{hm.h} in {time.time() - t0:.1f} s")
    surface = hm.height_at
    from isaac.site import site_name

    stage, dd, rtx, dust = None, None, None, None
    if a.dry_run:
        lidar = make_lidar("heightmap", lprof, grid=hm)
    else:
        from isaac import compat, scene
        stage = rt.stage()
        site_usd = None
        from isaac import collapse_site
        if site_name(sc) == "collapse" and not collapse_site.is_collapse(a.site):   # (--site collapse: add_to_stage below)
            from sim.site_collapse import find_usd
            site_usd = os.path.abspath(a.site_usd) if a.site_usd else find_usd()
            if not site_usd or not os.path.exists(site_usd):
                raise FileNotFoundError("--site collapse needs collapse_site.usda: pass --site-usd <path> "
                                        "(teammate's checkout: <repo>/collapse_site/collapse_site.usda, branch 3d_model)")
            say(f"referencing {site_usd} at /World/CollapseSite (lights: {a.site_lights}, "
                f"scan textures: {'on' if a.scan_textures else 'off'})")
        n = scene.build(stage, sc, show_truth=a.show_truth and not a.dummies, physics_scene=rt.own_physics_scene, sun=a.sun,
                        sky=a.sky, materials="none" if a.look == "plain" else "preview", site_usd=site_usd,
                        site_lights=a.site_lights, scan_textures=a.scan_textures, photoreal=bool(a.photoreal))
        say(f"stage built: {n} rubble pieces, {scene.build.materials.bound} material bindings")
        if collapse_site.is_collapse(a.site):  # the real map: its own meshes, colliders, sun and sky
            collapse_site.add_to_stage(stage, a.site_usd or None, say)
            if not a.scan_textures:
                say(f"--no-scan-textures: blanked {scene.strip_scan_textures(stage)} scan-derived texture inputs")
        if a.photoreal:                     # visual only; colliders keep the scenario's piece poses
            from isaac import photoreal
            photoreal.dress(stage, sc, a, say)
        if a.van:                           # command van + staging dressing (visual only, no colliders)
            from isaac import van
            vx, vy, vz, vyaw = van.staging_pose(sc)
            van.hide_legacy_staging(stage)
            info = van.build_van(stage, "/World/Van", (vx, vy), vyaw, z=vz, mdl=bool(a.van_mdl))
            say(f"command van at ({vx:.1f}, {vy:.1f}) yaw {vyaw:g}: {info.get('total')} tris")
        if a.dummies:                       # the victims as stage prims: the live take's source of truth
            from isaac import dummies as DM
            man = DM.load_manifest()
            info = DM.build_dummies(stage, sc, man)
            if a.ambient:
                DM.build_ambient(stage, sc, man)
            say(f"dummies: {len(info.get('dummies', []))} prims under /World/Dummies ({info.get('tris')} tris); "
                f"hidden: {[e['id'] for e in info.get('dummies', []) if e.get('hidden')]}")
        if a.rtx == "pathtracing" and not a.record:     # interactive hero view; --record sets it below
            from isaac import photoreal
            photoreal.set_renderer(a, say)
        if a.overlay in ("auto", "debug"):
            dd = compat.debug_draw()
            if dd is None:
                say("debug_draw unavailable; overlay uses USD prims")
        lidar = make_lidar("physx", lprof, cast=compat.physx_raycaster())
    ids = [d["id"] for d in as_dict(mission.build_state()).get("drones", [])] or ["d1", "d2", "d3"]
    if a.dry_run:
        drones = {i: NullDrone(None, f"/World/Drones/{i}") for i in ids}
    else:
        from isaac import drone_mesh as _dm
        _dm.POD_MODE = str(getattr(a, "delivery", "") or "") == "pod"      # radar rides in the winch pod
        drones = {i: DroneViz(stage, f"/World/Drones/{i}", did=i, scale=a.drone_scale, asset=a.drone_asset,
                              model=a.drone_model, delivery=str(getattr(a, "delivery", "") or "pod"))
                  for i in ids}
    from isaac.pods import NullPods, PodViz
    # the team's heavy model carries its own winch pod (cable, sway, rests on the rubble); the mesh style does not
    own_pod = (not a.dry_run) and all(getattr(d, "hang", None) is not None for d in drones.values())
    pods = NullPods() if a.dry_run else PodViz(stage, ids, drones=drones if own_pod else None)
    from isaac.sensor_fx import NullFX, SensorFX     # LiDAR beams, radar pulses, sound domes, hotspot, dummies
    if a.overlay_style == "simple":                  # Rohan 2026-11-08: one idea at a time, nothing secondary
        from isaac import overlay as _ov, sensor_fx as _sfx
        _ov.SIMPLE = True
        _sfx.SIMPLE = True
        say("[overlay] simple style: active/new zone outlines, latest dispatch arc, firm pings; sensor effects "
            "for human sounds only")
    if float(a.line_scale) != 1.0:                    # wide single-take shots: thicker lines and effects
        from isaac import overlay as _ov, sensor_fx as _sfx
        _ov.LINE_W *= float(a.line_scale)
        _ov.BEACON_R *= float(a.line_scale) ** 0.5
        _sfx.set_scale(a.line_scale, log=say)
        say(f"[overlay] line width x{a.line_scale:g}: zone outlines {_ov.LINE_W * _ov.ZONE_W * 100:.0f} cm")
    fx = NullFX() if (a.dry_run or not a.fx) else SensorFX(stage, sc, surface, show_truth=a.show_truth, log=say,
                                                          seed=a.seed)
    from isaac.site import base_xy
    overlay = Overlay(make_backend("null" if a.dry_run else a.overlay, stage, dd), surface, pile_xy,
                      base_xy=base_xy(sc), zones=a.zones)
    mirrors = []
    if not a.dry_run:
        from isaac import compat
        mirrors = [compat.PERSP] if a.camera in ("director", "story") else []
    take = a.camera == "director-take"
    story_memo = {}
    if take:
        from isaac.live import TakeCamera
        from isaac.site import drone_pads
        TakeCamera.OPEN = a.take_open
        TakeCamera.FLOAT = float(a.cam_float)
        from isaac.van import staging_pose
        cam = TakeCamera(stage, pile_xy, base_xy=base_xy(sc), pads=drone_pads(sc), surface=surface,
                         mirrors=[] if a.dry_run else [compat.PERSP], switch_viewport=True,
                         van=staging_pose(sc) if (a.van or a.dry_run) else None)
    elif a.camera == "story":
        cam = StoryCamera(stage, start=(*base_xy(sc)[:2], 3.0), mirrors=mirrors, switch_viewport=True)
    else:
        cam = DirectorCamera(stage, mode=a.camera, mirrors=mirrors, switch_viewport=(a.camera == "director"))
    if not a.dry_run and a.look == "pretty" and a.flow:
        from isaac import look
        dust = look.FlowDust(stage, ids, log=say)

    if not a.dry_run:
        rt.start_physics(30)                # Lab: sim.reset() + steps; Kit: force_load + play + updates
        p0 = sc.piles[4]
        hit = lidar.cast((p0.x, p0.y, 25.0), (0, 0, -1), 60.0)
        if hit is None:
            say("WARNING PhysX raycast hit nothing (scene queries off? physics not started?); "
                "falling back to the heightmap LiDAR so the demo still shows points. Run --selftest.")
            lidar = HeightmapLidar(hm, a.lidar_range, a.rays)
        else:
            say(f"PhysX raycast self-test ok: hit z={hit[2]:.2f} at pile {p0.name}")
        if a.rtx_lidar:
            rtx = RtxLidar({i: d.path for i, d in drones.items()}, configs=lprof["rtx_configs"],
                           mount_z=0.35 if a.drone_model == "small" else 0.50)
            rtx = rtx if rtx.ok else None

    accum = PointAccumulator.for_profile(lprof)
    mount_z = 0.35 if a.drone_model == "small" else 0.50   # LiDAR puck above the skids (nose gimbal on the X8)
    cur = prev = as_dict(mission.build_state())
    steps, frame = 0, 0
    virtual = a.dry_run or bool(a.record)   # recording: fixed frame clock, so slow (path-traced) frames don't skip
    if a.record:
        a.speed = a.rec_speed
    base_speed = a.speed
    clock_v = 0.0
    clock = (lambda: clock_v) if virtual else time.monotonic
    tilt_f = {i: (0.0, 0.0) for i in ids}
    moved = {i: 0.0 for i in ids}
    start_xy = {d["id"]: d["pose"][:3] for d in cur.get("drones", [])}
    active = overlay.update(cur)
    last_frame_t = clock()
    finished_at = None
    rdt_v = 1.0 / (a.rec_fps if a.record else a.render_fps)
    rigs = rec = None
    if a.record and rt is not None:         # photoreal.Recorder: replicator render products per camera (headless-safe)
        from isaac import photoreal
        photoreal.set_renderer(a, say)
        specs = [c.strip() for c in a.rec_cams.split(",") if c.strip()]
        from isaac.site import bounds as site_bounds
        rw, rh = (int(v) for v in a.rec_res.split("x"))
        rigs = photoreal.Rigs(stage, [c for c in specs if c != "director"], surface,
                             fpv_tilt=a.fpv_tilt, chase_back=a.chase_back, chase_up=a.chase_up,
                             piles=pile_xy, bounds=site_bounds(sc), aspect=rw / rh)
        cams = ([("director", cam.path)] if "director" in specs else []) + [(r["name"], r["path"]) for r in rigs.rigs]
        rec = photoreal.Recorder(a.record, cams, a.rec_res, say, windows=photoreal.Recorder.parse_windows(a.rec_windows))
        if a.rec_windows and a.headless:        # nothing needs the viewport: render only the windowed cameras
            try:
                from omni.kit.viewport.utility import get_active_viewport
                get_active_viewport().updates_enabled = False
                say("[photoreal] viewport updates off (windowed recording)")
            except Exception as e:
                say(f"[photoreal] viewport stays on ({type(e).__name__}: {e})")
        for _ in range(10):
            rt.step()
    rec_i = lambda: frame // max(1, a.rec_every)    # noqa: E731  video frame index

    # ---- live single take: stage dummies -> generators, unscripted triggers, HUD (isaac/live.py, isaac/hud.py)
    ctrl = keys = None
    if a.cinema and not a.dry_run and not a.live_take:
        from isaac import cinema
        cinema.apply(a, say)
    if a.live_take:
        from isaac import live as L
        if not a.dry_run and (a.take_render if a.take_render is not None else a.rtx != "pathtracing"):
            L.apply_take_render(say)
        if a.cinema and not a.dry_run:          # after the take render settings, so the film look wins
            from isaac import cinema
            cinema.apply(a, say)
        src = L.make_source(stage, sc, surface=surface, log=say)     # stage dummies, else isaac.dummies' plan
        probe = None
        if not a.dry_run and not isinstance(lidar, HeightmapLidar):
            def probe(x, y):                    # top of whatever is over (x, y) now: rubble, terrain, dragged debris
                h = lidar.cast((float(x), float(y), 80.0), (0, 0, -1), 120.0)
                return None if h is None else float(h[2])
        world = L.LiveWorld(sc, src, surface=surface, probe=probe, log=say)
        service = None
        want_real = (a.acoustic or "auto") != "stub" and (not a.dry_run or a.acoustic == "real")
        if want_real:                           # the take: the REAL pipeline on the live dummies, or nothing
            try:
                service, info = L.preflight_acoustic(sc, world, log=say, gemini=bool(a.live_gemini), asr=a.live_asr)
                a.acoustic_info = info
            except Exception as e:              # noqa: BLE001
                msg = (f"ACOUSTIC PRE-FLIGHT FAILED: {e}. This take would use STUB acoustics (clips generated from "
                       "the world list). Fix .venv-audio (docs/VIDEO_PLAN.md section 6) or pass --allow-stub-acoustic "
                       "for a rehearsal.")
                say("!" * 78)
                say(msg)
                say("!" * 78)
                if not a.allow_stub_acoustic:
                    raise SystemExit(msg)
        ctrl = L.LiveController(mission, world, out_dir=a.out, quick_s=L.QUICK_LISTEN_S if a.live_quick_s is None
                                else (a.live_quick_s or 1e12), preempt=a.preempt, acoustic=a.acoustic or "auto",
                                radar=a.radar, keys=a.live_keys or L.DEFAULT_KEYS, log=say, clock=lambda: clock(),
                                service=service)
        if a.test_trigger:
            ctrl.test_triggers = L.parse_test_trigger(a.test_trigger)
            say(f"TEST TRIGGERS (dry run only, never in the take): {ctrl.test_triggers}")
        if not a.dry_run:
            keys = L.Hotkeys(ctrl.on_key, ctrl.keys, log=say)
        say(f"LIVE take: hook={ctrl.hook}, world source={src.kind}, hidden dummies {src.hidden_ids()}, "
            f"tasking={getattr(mission, 'tasking_path', None)}, quick listen every {ctrl.quick_s:g} s, "
            f"preempt={'on' if a.preempt else 'off'}; keys {ctrl.keys}")
    from isaac.hud import Hud
    first_key = list((ctrl.keys if ctrl else {}) or {"Y": None})[-1]     # the HUD button = the last key (Y: H2)
    hud = Hud(a.hud, out_dir=a.out if a.live_take else None, bounds=(cur.get("site") or {}).get("bounds"),
              piles=cur.get("piles"), speed=a.speed, log=say,
              on_button=(lambda: ctrl.on_key(first_key)) if ctrl is not None else None)
    n_events = 0

    pending = {}                            # pile -> [new points not yet handed to the mission]
    fed = {}                                # pile -> points ingested
    poses = {}

    try:                                    # GridMission's LiDAR window rebuild imports scipy.ndimage
        import scipy.ndimage  # noqa: F401
        can_ingest = True
    except ImportError:
        can_ingest = False
        say("scipy missing: LiDAR points are drawn but NOT fed to the mission (pip install scipy)")

    import threading
    pend_lock = threading.Lock()

    def feed():
        with pend_lock:
            if not can_ingest:
                pending.clear()
                return
            items = [(pile, chunks) for pile, chunks in pending.items() if chunks]
            for pile in list(pending):
                pending[pile] = []
        for pile, chunks in items:
            pts = np.concatenate(chunks)
            if call_ingest(mission, pts, pile, say):
                fed[pile] = fed.get(pile, 0) + len(pts)

    nstep = [0]

    def mission_done():
        return mission.done or bool(a.max_steps and nstep[0] >= a.max_steps)

    def do_step():                              # inline, or on the worker thread (--mission-thread)
        feed()                                  # before step(): the survey may end inside this step
        if ctrl is not None:
            ctrl.before_step()                  # 1 s poll (tasking, live_cmd.json, stage cache), hotkeys
        mission.step()
        nstep[0] += 1
        new = as_dict(mission.build_state())
        if ctrl is not None:
            if ctrl.after_step(new):            # pod-mic quick listen, re-task / portal re-plans (outputs only)
                new = as_dict(mission.build_state())
                if hasattr(mission, "_write_state"):
                    mission._write_state()      # the dashboard sees the re-plan now, not a step later
            ctrl.write()
        return new

    threaded = bool(a.mission_thread) and not isinstance(mission, ReplayMission)
    if threaded and ctrl is not None:
        ctrl.threaded = True
        ctrl.live.source = L.SyncedSource(ctrl.live.source, probe=ctrl.live.probe)
        ctrl.live.probe = ctrl.live.source.probe_cached
        ctrl.live.source.sync()
    track = PoseTrack(log=say)
    track.push(cur)
    t_render = float(cur.get("t", 0.0))         # sim time on screen: advances continuously every frame
    lead = 1.0                                  # the mission stays >= one step ahead (no extrapolation)
    runner = MissionRunner(do_step, mission_done, threaded=threaded)
    if threaded:
        say("mission steps on a worker thread (--mission-thread 1); USD stays on the main thread")
    sync_t = -1e9
    pace = {}
    preroll = 0.0
    if a.live_take:
        # "clear": a readable take under 60 s with the smooth-flight timings of 04a4d34 (survey t20-65, first 3-pod
        # dwell t85-130, rescans to t364, listen t364-541, dispatch t541-565). Measured on the dry-run frames
        # (seed 7, grid, stub acoustics): survey 6.4 s, pods down at A2 from 16.8 s, rescans 21-30.7 s at x24 (the hops
        # still read as flights), listen 30.7-41.1 s, flight to the next pile 41.1-45.9 s, its survey 45.9-51.4 s and its
        # pods down at 54.5 s. The real acoustic pipeline can slow the listen (its pace is capped from the latency).
        CLEAR = ("deploy=6,survey=7,recon_mapping=6,recon_listening=10,landing=6,radar=12,relocate=24,rescan=24,"
                 "acoustic=17,dispatch=5,return=12,battery_swap=40,done=8")
        pace = L.parse_pace(L.DEFAULT_PACE if a.live_pace is None else CLEAR if a.live_pace == "clear" else a.live_pace)
        preroll = 4.0 if a.live_preroll is None else float(a.live_preroll)
        info = getattr(a, "acoustic_info", None)
        if pace and info and info.get("latency_s"):
            # the prefetched listen starts when the drones are down; ~66 sim s later (26 s listen + call-out) the gate
            # needs it. Slow the listen phase so the real pipeline answers inside that window (floor x4: below that
            # the mission clock is simply held, shown on the HUD).
            cap = max(4.0, L.LISTEN_AFTER_DOWN_S / (float(info["latency_s"]) + 1.0))
            if cap < pace.get("acoustic", base_speed):
                say(f"real acoustic pipeline answers in {info['latency_s']:.0f} s: listen pace x{pace['acoustic']:g} -> "
                    f"x{cap:.1f} so it lands inside the listen window")
                pace["acoustic"] = round(cap, 1)
        if pace:
            a.speed = L.pace_for(pace, cur.get("phase"), base_speed)
            say(f"take pacing (sim s per video s): {pace}; pre-roll {preroll:g} s")

    hold_said = False
    try:
        while True:
            now = clock()
            rdt = rdt_v if virtual else max(now - last_frame_t, 1e-3)
            last_frame_t = now
            if threaded and ctrl is not None and now - sync_t >= 0.25:
                sync_t = now
                ctrl.live.source.sync()         # main thread: apply hotkey writes, re-read dummy prims, probe columns
                ctrl.live.refresh(None, "sync")  # stage events show within 0.25 s
            held = bool(a.hold_file) and not os.path.exists(a.hold_file)
            if held != hold_said:
                say(f"holding at t 0 until {a.hold_file} exists" if held else f"{a.hold_file} found: take starts")
                hold_said = held
            if held:
                pass                            # capture not armed yet: render the opening frame, keep the clock at 0
            elif preroll > 0:
                preroll -= rdt                  # opening shot: the mission has not started yet (shown on the HUD)
            elif (not runner.idle or t_render < track.t_last) and not (a.stop_t and t_render >= a.stop_t):
                t_render += rdt * a.speed
            runner.target(t_render + lead)
            for new in runner.drain():
                prev, cur = cur, new
                track.push(cur)
                steps += 1
                if a.record and a.rec_warp > 1:     # recording: compress the long radar stretches
                    from isaac.photoreal import WARP_PHASES
                    a.speed = a.rec_speed * (a.rec_warp if cur.get("phase") in WARP_PHASES else 1.0)
                elif a.live_take and pace:
                    a.speed = L.pace_for(pace, cur.get("phase"), base_speed)
                elif a.live_take and a.live_warp > 1:
                    from isaac.photoreal import WARP_PHASES
                    a.speed = base_speed * (a.live_warp if cur.get("phase") in WARP_PHASES else 1.0)
                active = overlay.update(cur)
                if steps % 20 == 0:
                    say(f"step {steps} t={cur.get('t')} phase={cur.get('phase')} pile={active} "
                        f"lidar={accum.total} pts")
            if t_render > track.t_last:         # the mission fell behind: slow the picture down, never jump
                t_render = track.t_last
            done = (runner.idle and t_render >= track.t_last - 1e-6) or bool(a.stop_t and t_render >= a.stop_t)
            poses = track.poses(t_render)
            motion_drives = False
            pod_z = {p["drone_id"]: float(p["z"]) for p in cur.get("pods") or []
                     if p.get("status") in ("scanning", "resting") and p.get("z") is not None}
            for did, pz in poses.items():
                x, y, z, yaw, st, vx, vy = pz[:7]
                dr = drones.get(did)
                if dr is None:
                    continue
                raw = pz[8] if len(pz) > 8 else None
                if raw is not None and raw.get("quat") is not None and hasattr(dr, "set_state"):
                    # flight motion (sim/flight_motion.py via isaac/motion.py): attitude, per-rotor rpm, winch
                    dr.set_state(raw["xyz"], raw["quat"], raw.get("rotor_rpm"), raw.get("winch_len"), rdt,
                                 sway_deg=raw.get("sway_deg"), ground_z=raw.get("ground_z", pod_z.get(did)))
                    motion_drives = motion_drives or raw.get("winch_len") is not None
                    tilt_f[did] = tuple(raw.get("tilt") or (0.0, 0.0))
                elif len(pz) > 7 and pz[7] is not None:   # the motion model's tilt into acceleration
                    tilt_f[did] = tuple(pz[7])
                else:
                    sp = math.hypot(vx, vy)
                    ang = math.radians(math.degrees(math.atan2(vy, vx)) - yaw) if sp > 0.05 else 0.0
                    tilt = min(14.0, 2.5 * sp) if not done else 0.0
                    want = (-math.sin(ang) * tilt, math.cos(ang) * tilt)
                    k = 1.0 - math.exp(-rdt / 0.4)
                    tf = tilt_f[did]
                    tilt_f[did] = (tf[0] + (want[0] - tf[0]) * k, tf[1] + (want[1] - tf[1]) * k)
                if raw is None or raw.get("quat") is None or not hasattr(dr, "set_state"):
                    dr.set_pose(x, y, z, yaw, tilt_f[did])
                    air = z - float(surface(x, y)) > 0.4
                    dr.spin(rdt, on=air or st not in GROUNDED)
                sx = start_xy.get(did, (x, y, z))
                moved[did] = max(moved[did], math.dist((x, y, z), sx))
                if not done and is_surveying(st, cur.get("phase"), z - float(surface(x, y)) > 1.5):
                    pl = active if active in pile_xy else nearest_pile(pile_xy, x, y)
                    if pl:
                        pts = rtx.read(did) if rtx is not None else lidar.sweep((x, y, z + mount_z), yaw, frame)
                        fx.lidar(did, (x, y, z + mount_z), pts)
                        new_pts = accum.add(pl, pts)
                        if new_pts is not None and len(new_pts):
                            with pend_lock:
                                pending.setdefault(pl, []).append(new_pts)
                            hud.add_points(new_pts)
                            show = new_pts
                            if a.photoreal and pl in pile_xy:   # the map, not hits sprayed over the whole site
                                from isaac.photoreal import LIDAR_DRAW_R
                                c = np.asarray(pile_xy[pl], float)[:2]
                                show = new_pts[np.hypot(new_pts[:, 0] - c[0], new_pts[:, 1] - c[1]) < LIDAR_DRAW_R]
                            if len(show):
                                overlay.add_points(show)
            if motion_drives or not isinstance(pods, PodViz):
                NullPods.update(pods, cur, poses)      # count / targets only: set_state already drew each winch pod
            else:                                      # winch-lowered radar pods (state.pods / winch_len_m)
                pods.update(cur, poses, dt=rdt)
            if dust is not None:
                dust.update(poses, surface)

            fx.frame(now, cur, poses)                  # LiDAR beams, radar pulses, sound domes, belief hotspot
            focus, spread = focus_from_state(cur, pile_xy, active, list(poses.values()))
            if a.camera == "story":
                focus, spread = story_focus(cur, pile_xy, active, list(poses.values()), story_memo)
            if take:
                if ctrl is not None and len(ctrl.live.events) > n_events:   # display only: frame the stage change
                    ev = ctrl.live.events[-1]
                    n_events = len(ctrl.live.events)
                    if ev["what"] in ("activated", "moved", "added"):
                        cam.on_event(ev["xyz"], f"{ev['id']} {ev['what']}")
                cam.observe(cur, poses, active)
            cam.update(rdt, focus, spread)
            hud.update(cur, ctrl, accum.total, now=clock(), speed=a.speed, sim_t=t_render)
            if rigs is not None:
                rigs.update(poses, focus, rdt)
            overlay.tick(now)
            frame += 1
            if rt is not None:
                if not rt.is_running():
                    break
                if rec is not None:
                    rec.prepare(rec_i())
                rt.step()
                if rec is not None and frame % max(1, a.rec_every) == 0:
                    rec.capture(rec_i())
                    rec.note(rec_i(), cur, active)
            if virtual:
                clock_v += rdt_v
            if a.record and a.rec_stop_t and float(cur.get("t", 0)) >= a.rec_stop_t:
                done = True
            if rec is not None and rec.done(rec_i()):
                done = True                     # past the last window
            if done:
                finished_at = finished_at or now
                if not a.keep_open or now - finished_at > 2.0:
                    break
    except Exception:
        traceback.print_exc()
        runner.stop()
        if rt is None or not a.keep_open:
            if rt is not None:
                rt.close()
            raise
        say("error above; keeping the window open for inspection")

    runner.stop()
    if keys is not None:
        keys.close()
    if ctrl is not None and ctrl.service is not None:
        ctrl.service.close()
    if ctrl is not None:
        ctrl.write()
    state_path = os.path.join(a.out, "state.json") if a.out else None
    summary = {
        "steps": steps, "t": cur.get("t"), "phase": cur.get("phase"), "frames": frame,
        "drone_moved_m": {k: round(v, 2) for k, v in moved.items()},
        "lidar_points": int(accum.total), "piles_scanned": sorted(accum.chunks), "ingested": fed,
        "overlay_updates": overlay.updates, "camera_updates": cam.updates,
        "state_json": state_path if state_path and os.path.exists(state_path) else None,
        "mission": type(mission).__name__, "launcher": a.launcher, "site": site_name(sc),
        "pods_max_visible": pods.max_visible, "fx": dict(fx.counts, dummies=fx.dummies),
        "recorded_frames": rec.n if rec is not None else 0,
        "hud": {"kind": hud.view.kind, "model_updates": hud.model.updates, "draws": hud.view.draws},
        "live": None if ctrl is None else {
            "hook": ctrl.hook, "world_source": ctrl.live.source.kind, "world_changed": ctrl.live.changed,
            "events": [f"t={e['t']} {e['id']} {e['what']} ({e['cause']})" for e in ctrl.live.events],
            "quick_listens": ctrl._ql_n, "new_evidence": [q["new"] for q in ctrl.quick if q["new"]],
            "replans": ctrl.replans, "visited": list(getattr(mission, "visited", [])),
            "dispatch": [(d.get("t"), d.get("from_pile"), d.get("to_pile")) for d in getattr(mission, "dispatch", [])],
            "last_listen": ctrl.live.last_listen, "world_reads": ctrl.live.reads,
            "real_acoustic": ctrl.service is not None,
            "acoustic_latency_s": ctrl.service.latency if ctrl.service is not None else None},
        "camera_shots": getattr(cam, "shots", None),
    }
    say("summary " + json.dumps(summary))
    if rec is not None and rec.n:
        fps = a.rec_fps / max(a.rec_every, 1)
        say(f"make the video (per camera): ffmpeg -framerate {fps:g} -i {a.record}/<cam>/f%05d.png "
            f"-c:v libx264 -pix_fmt yuv420p {a.record}/<cam>.mp4   (cams: {', '.join(rec.paths)})")

    if rt is not None:
        if a.keep_open:
            say("mission finished; app stays open (close the window / Ctrl+C to exit)")
            last = time.monotonic()
            try:
                while rt.is_running():
                    now = time.monotonic()
                    cam.update(now - last, focus_from_state(cur, pile_xy, active, list(poses.values()))[0])
                    overlay.tick(now)
                    last = now
                    rt.step()
            except KeyboardInterrupt:
                pass
        rt.close()
    return summary


if __name__ == "__main__":
    main()
