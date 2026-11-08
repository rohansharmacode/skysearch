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
    ap.add_argument("--viz-scale", type=float, default=1.0, help="multiply overlay line and sensor-effect widths "
                    "(zones, dispatch arcs, rings, LiDAR beams, sound domes) for wide shots; the take uses ~4")
    ap.add_argument("--smooth", type=int, default=1, help="C1-smooth drone motion through the 1 s mission states "
                    "(monotone cubic, one step behind; tilt from acceleration + drag): 1 on, 0 = linear (old)")
    ap.add_argument("--fx", type=int, default=1, help="live sensor effects (isaac/sensor_fx.py): LiDAR beams, "
                    "radar pulses, sound domes while listening, belief hotspot (0 = off)")
    ap.add_argument("--replay", default=None, help="dir with frames/state_XXXX.json (or the frames dir itself)")
    ap.add_argument("--out", default=os.path.join(REPO, "dashboard", "data"), help="state.json output dir")
    ap.add_argument("--frames", action="store_true", help="also write frames/state_XXXX.json")
    ap.add_argument("--dry-run", action="store_true", help="no omni/pxr: numpy heightmap LiDAR, null drawing")
    ap.add_argument("--max-steps", type=int, default=0, help="stop after N mission steps (0 = until done)")
    ap.add_argument("--camera", choices=["director", "story", "free"], default="director",
                    help="story = one continuous state-driven shot for the judges' take (frames the approach, "
                    "the active zone, new/suggested zones and the dispatch; slow orbit, spring-smoothed)")
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
    from isaac.lidar import profile as lidar_profile     # resolve the LiDAR profile once (selftest reads a.rays)
    a.lidar_prof = lidar_profile(a.lidar_profile, a.lidar_budget_ms, a.rays, a.lidar_range)
    a.rays, a.lidar_range = a.lidar_prof["n_rays"], a.lidar_prof["max_range"]
    a.keep_open = not (a.dry_run or a.no_keep_open or a.record or a.selftest or (a.headless and not a.live))
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
        return ReplayMission(a.replay, a.out, a.seed, a.start_pile, a.site)
    try:
        from isaac.site import mission_for_site
        extra = {k: getattr(a, k) for k in ("platform", "penetration", "delivery", "acoustic")
                 if getattr(a, k, None) is not None}
        extra["autonomous"] = getattr(a, "autonomous", True)
        tk = getattr(a, "tasking", None)
        if tk:                              # live operator zones: GridMission re-reads the file each step (mtime)
            extra["tasking"] = os.path.join(a.out, "tasking.json") if tk == "auto" else tk
            log(f"operator tasking: {extra['tasking']}")
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


G = 9.81
DRAG_TILT = 0.18        # 1/s: cruise tilt of a heavy multirotor ~ atan(DRAG_TILT * v / g) (10 m/s -> ~10 deg)
MAX_TILT = 22.0         # deg


def _mono_tangent(d0, d1):
    """Fritsch-Carlson tangent at a knot from the two adjacent secants (per axis): no overshoot, so a landing
    drone never dips into the rubble and a hover never wobbles."""
    m = np.where(d0 * d1 > 0, 0.5 * (d0 + d1), 0.0)
    lim = 3.0 * np.minimum(np.abs(d0), np.abs(d1))
    return np.clip(m, -lim, lim)


def smooth_drones(hist, u):
    """C1-smooth drone poses through the mission's 1 s states (one step behind the newest state): monotone
    cubic Hermite on the segment hist[-3] -> hist[-2], tangents from hist[-4] and hist[-1]. Returns
    {id: (x, y, z, yaw, state, vx, vy, ax, ay)} with velocity (m/s) and mean acceleration (m/s^2) in sim time."""
    A, B, C = hist[-3], hist[-2], hist[-1]
    P0 = hist[-4] if len(hist) >= 4 else A
    T = max(float(B.get("t", 0)) - float(A.get("t", 0)), 1e-3)
    pos = {k: {d["id"]: d for d in s.get("drones", [])} for k, s in (("p0", P0), ("a", A), ("b", B), ("c", C))}
    h00, h10, h01, h11 = 2 * u**3 - 3 * u**2 + 1, u**3 - 2 * u**2 + u, -2 * u**3 + 3 * u**2, u**3 - u**2
    d00, d10, d01, d11 = 6 * u**2 - 6 * u, 3 * u**2 - 4 * u + 1, -6 * u**2 + 6 * u, 3 * u**2 - 2 * u
    out = {}
    for did, db in pos["b"].items():
        da = pos["a"].get(did, db)
        p1, p2 = np.asarray(da["pose"][:3], float), np.asarray(db["pose"][:3], float)
        p0 = np.asarray(pos["p0"].get(did, da)["pose"][:3], float)
        p3 = np.asarray(pos["c"].get(did, db)["pose"][:3], float)
        m1, m2 = _mono_tangent(p1 - p0, p2 - p1), _mono_tangent(p2 - p1, p3 - p2)
        x = h00 * p1 + h10 * m1 + h01 * p2 + h11 * m2
        v = (d00 * p1 + d10 * m1 + d01 * p2 + d11 * m2) / T
        acc = (m2 - m1) / (T * T)          # segment-mean acceleration: the cubic's own 2nd derivative jumps at
                                           # every knot, which reads as attitude flicker; the tilt filter smooths this
        yaw = _lerp_angle(da["pose"][3], db["pose"][3], u)
        out[did] = (*map(float, x), yaw, db.get("state", ""), float(v[0]), float(v[1]), float(acc[0]), float(acc[1]))
    return out


def is_surveying(drone_state, phase, airborne):
    if drone_state in SURVEY_STATES:
        return True
    return airborne and phase in ("survey", "surveying") and drone_state in ("flying", "hovering")


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
        n = scene.build(stage, sc, show_truth=a.show_truth, physics_scene=rt.own_physics_scene, sun=a.sun,
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
    if float(a.viz_scale) != 1.0:                    # wide single-take shots: thicker lines and effects
        from isaac import overlay as _ov, sensor_fx as _sfx
        _ov.LINE_W *= float(a.viz_scale)
        _ov.BEACON_R *= float(a.viz_scale) ** 0.5
        _sfx.set_scale(a.viz_scale, log=say)
        say(f"[overlay] line width x{a.viz_scale:g}: zone outlines {_ov.LINE_W * _ov.ZONE_W * 100:.0f} cm")
    fx = NullFX() if (a.dry_run or not a.fx) else SensorFX(stage, sc, surface, show_truth=a.show_truth, log=say,
                                                          seed=a.seed)
    from isaac.site import base_xy
    overlay = Overlay(make_backend("null" if a.dry_run else a.overlay, stage, dd), surface, pile_xy,
                      base_xy=base_xy(sc), zones=a.zones)
    mirrors = []
    if not a.dry_run:
        from isaac import compat
        mirrors = [compat.PERSP] if a.camera in ("director", "story") else []
    story_memo = {}
    if a.camera == "story":
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
    hist = [cur]                            # last 4 states: smooth motion renders one step behind the newest
    smooth = bool(getattr(a, "smooth", 1))
    steps, frame = 0, 0
    virtual = a.dry_run or bool(a.record)   # recording: fixed frame clock, so slow (path-traced) frames don't skip
    if a.record:
        a.speed = a.rec_speed
    clock_v = 0.0
    clock = (lambda: clock_v) if virtual else time.monotonic
    last_step = next_step = clock()
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

    pending = {}                            # pile -> [new points not yet handed to the mission]
    fed = {}                                # pile -> points ingested
    poses = {}

    try:                                    # GridMission's LiDAR window rebuild imports scipy.ndimage
        import scipy.ndimage  # noqa: F401
        can_ingest = True
    except ImportError:
        can_ingest = False
        say("scipy missing: LiDAR points are drawn but NOT fed to the mission (pip install scipy)")

    def feed():
        if not can_ingest:
            pending.clear()
            return
        for pile, chunks in list(pending.items()):
            if chunks:
                pts = np.concatenate(chunks)
                if call_ingest(mission, pts, pile, say):
                    fed[pile] = fed.get(pile, 0) + len(pts)
            pending[pile] = []

    try:
        while True:
            now = clock()
            done = mission.done or (a.max_steps and steps >= a.max_steps)
            while not done and now >= next_step:
                feed()                         # before step(): the survey may end inside this step
                mission.step()
                steps += 1
                new = as_dict(mission.build_state())
                dt_sim = max(float(new.get("t", 0)) - float(cur.get("t", 0)), 1e-3)
                prev, cur = cur, new
                hist = (hist + [new])[-4:]
                if a.record and a.rec_warp > 1:     # recording: compress the long radar stretches
                    from isaac.photoreal import WARP_PHASES
                    a.speed = a.rec_speed * (a.rec_warp if cur.get("phase") in WARP_PHASES else 1.0)
                last_step, next_step = next_step, next_step + dt_sim / max(a.speed, 1e-3)
                if now - next_step > 1.0:      # fell behind (slow step): resync instead of spiralling
                    last_step, next_step = now, now + dt_sim / max(a.speed, 1e-3)
                active = overlay.update(hist[-2] if smooth and len(hist) >= 3 else cur)
                done = mission.done or (a.max_steps and steps >= a.max_steps)
                if steps % 20 == 0:
                    say(f"step {steps} t={cur.get('t')} phase={cur.get('phase')} pile={active} "
                        f"lidar={accum.total} pts")

            span = max(next_step - last_step, 1e-6)
            u = 1.0 if done else min(max((now - last_step) / span, 0.0), 1.0)
            dt_sim_cur = max(float(cur.get("t", 0)) - float(prev.get("t", 0)), 1e-3)
            if smooth and len(hist) >= 3 and not done:
                disp = hist[-2]
                poses = smooth_drones(hist, u)
            else:
                disp = cur
                poses = {k: (*v[:5], v[5] / dt_sim_cur, v[6] / dt_sim_cur, 0.0, 0.0) for k, v in interp_drones(prev, cur, u).items()}
            rdt = rdt_v if virtual else max(now - last_frame_t, 1e-3)
            last_frame_t = now
            for did, (x, y, z, yaw, st, vx, vy, ax, ay) in poses.items():
                dr = drones.get(did)
                if dr is None:
                    continue
                # multirotor attitude: lean into the acceleration plus the drag-balancing cruise tilt (flare
                # when braking falls out of the negative acceleration); sim-time units, not the speed-up
                ex, ey = ax + DRAG_TILT * vx, ay + DRAG_TILT * vy
                mag = math.hypot(ex, ey)
                ang = math.radians(math.degrees(math.atan2(ey, ex)) - yaw) if mag > 0.05 else 0.0
                tilt = min(MAX_TILT, math.degrees(math.atan2(mag, G))) if not done else 0.0
                want = (-math.sin(ang) * tilt, math.cos(ang) * tilt)
                k = 1.0 - math.exp(-rdt / 0.3)
                tf = tilt_f[did]
                tilt_f[did] = (tf[0] + (want[0] - tf[0]) * k, tf[1] + (want[1] - tf[1]) * k)
                dr.set_pose(x, y, z, yaw, tilt_f[did])
                air = z - float(surface(x, y)) > 0.4
                dr.spin(rdt, on=air or st not in GROUNDED)
                sx = start_xy.get(did, (x, y, z))
                moved[did] = max(moved[did], math.dist((x, y, z), sx))
                if not done and is_surveying(st, disp.get("phase"), z - float(surface(x, y)) > 1.5):
                    pl = active if active in pile_xy else nearest_pile(pile_xy, x, y)
                    if pl:
                        pts = rtx.read(did) if rtx is not None else lidar.sweep((x, y, z + mount_z), yaw, frame)
                        fx.lidar(did, (x, y, z + mount_z), pts)
                        new_pts = accum.add(pl, pts)
                        if new_pts is not None and len(new_pts):
                            pending.setdefault(pl, []).append(new_pts)
                            show = new_pts
                            if a.photoreal and pl in pile_xy:   # the map, not hits sprayed over the whole site
                                from isaac.photoreal import LIDAR_DRAW_R
                                c = np.asarray(pile_xy[pl], float)[:2]
                                show = new_pts[np.hypot(new_pts[:, 0] - c[0], new_pts[:, 1] - c[1]) < LIDAR_DRAW_R]
                            if len(show):
                                overlay.add_points(show)
            pods.update(disp, poses)                   # winch-lowered radar pods (state.pods / winch_len_m)
            fx.frame(now, disp, poses)
            if dust is not None:
                dust.update(poses, surface)

            focus, spread = focus_from_state(disp, pile_xy, active, list(poses.values()))
            if a.camera == "story":
                sf, sr = story_focus(disp, pile_xy, active, list(poses.values()), story_memo)
                cam.update(rdt, sf, sr)
            else:
                cam.update(rdt, focus, spread)
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
                    rec.note(rec_i(), disp, active)
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
        if rt is None or not a.keep_open:
            if rt is not None:
                rt.close()
            raise
        say("error above; keeping the window open for inspection")

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
