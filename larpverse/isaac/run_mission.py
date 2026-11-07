"""Run the 3x3 rubble-grid SAR mission inside Isaac Sim (or ``--dry-run`` without omni/pxr).

    # Isaac Sim 6.0.x + Isaac Lab 3.0 (Brev launchable; the route the team verified on the L40S):
    cd /workspace/isaaclab && ./isaaclab.sh -p /workspace/skysearch/larpverse/isaac/run_mission.py --selftest --headless
    ./isaaclab.sh -p .../isaac/run_mission.py --livestream 2 --viz kit --speed 4 --show-truth   # watch at <brev URL>/viewer
    # Isaac Sim 4.5 / 5.x standalone (SimulationApp route):
    ~/isaacsim/python.sh isaac/run_mission.py --seed 7 --speed 4
    python isaac/run_mission.py --dry-run --max-steps 60                          # any laptop, numpy only

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

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

SURVEY_STATES = {"surveying", "survey", "mapping", "lidar"}
GROUNDED = {"docked", "landed", "scanning", "listening", "idle", "charging"}


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
    ap.add_argument("--radar", choices=["model", "dsp"], default="dsp", help="dsp needs scipy in Isaac's python")
    ap.add_argument("--speed", type=float, default=4.0, help="sim-seconds per wall-second")
    ap.add_argument("--show-truth", action="store_true", help="draw ground-truth victim ghosts (display only)")
    ap.add_argument("--replay", default=None, help="dir with frames/state_XXXX.json (or the frames dir itself)")
    ap.add_argument("--out", default=os.path.join(REPO, "dashboard", "data"), help="state.json output dir")
    ap.add_argument("--frames", action="store_true", help="also write frames/state_XXXX.json")
    ap.add_argument("--dry-run", action="store_true", help="no omni/pxr: numpy heightmap LiDAR, null drawing")
    ap.add_argument("--max-steps", type=int, default=0, help="stop after N mission steps (0 = until done)")
    ap.add_argument("--camera", choices=["director", "free"], default="director")
    ap.add_argument("--drone-asset", nargs="?", const="auto", default=None,
                    help="reference Isaac's quadcopter USD (optionally a path/URL) instead of primitives")
    ap.add_argument("--drone-scale", type=float, default=1.6)
    ap.add_argument("--rtx-lidar", action="store_true", help="try RTX LiDAR (falls back to PhysX raycasts)")
    ap.add_argument("--overlay", choices=["auto", "debug", "usd"], default="auto",
                    help="debug = debug_draw lines/points; usd = prims only (safest for Movie Capture)")
    ap.add_argument("--rays", type=int, default=96, help="LiDAR rays per surveying drone per frame")
    ap.add_argument("--lidar-range", type=float, default=40.0)
    ap.add_argument("--render-fps", type=float, default=30.0, help="virtual frame rate for --dry-run / --record")
    ap.add_argument("--no-keep-open", action="store_true", help="close the GUI when the mission ends")
    ap.add_argument("--quiet", action="store_true")
    g = ap.add_argument_group("look (docs/ISAAC_VERIFIED_APIS.md)")
    g.add_argument("--look", choices=["basic", "pretty", "plain"], default="basic",
                   help="basic = UsdPreviewSurface materials; pretty = + HDRI dusk sky, MDL concrete, chipped rubble, "
                        "Flow dust; plain = bare displayColor (diagnosis)")
    g.add_argument("--sun", type=float, default=1800.0, help="sun intensity (L40S-measured; 3000 overhead washed out)")
    g.add_argument("--sky", type=float, default=350.0, help="dome intensity without HDRI")
    g.add_argument("--hdri", type=float, default=900.0, help="HDRI dome intensity with --look pretty")
    g.add_argument("--flow", type=int, default=1, help="--look pretty: Omni Flow dust under landing drones (0 = off)")
    g.add_argument("--rtx", choices=["realtime", "pathtracing"], default="realtime",
                   help="pathtracing: hero shots (~6.5 s/frame for 3 cams at 720p, 16 spp, on the L40S)")
    g.add_argument("--spp", type=int, default=16, help="path-tracing samples per pixel per frame")
    g = ap.add_argument_group("recording (Isaac Lab Camera sensor route, verified on 6.0.1)")
    g.add_argument("--record", default=None, help="dir: save the director view to <dir>/isaac/fNNNNN.png at a fixed "
                                                  "virtual frame rate (smooth video however slow the render is)")
    g.add_argument("--rec-res", default="1280x720")
    g.add_argument("--rec-every", type=int, default=1, help="save every Nth render frame")
    g.add_argument("--fabric", action="store_true", help="Isaac Lab use_fabric=True (default off: pxr pose edits render)")
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
    live = int(getattr(a, "livestream", 0) or 0)
    a.live = live > 0
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

    def __init__(self, replay_dir, out_dir=None, seed=7, start_pile=None):
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
        self.scenario = make_scenario(seed, start_pile or meta.get("start_pile") or self.state.get("start_pile"))
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
        return ReplayMission(a.replay, a.out, a.seed, a.start_pile)
    try:
        from sim.grid_mission import GridMission
        return GridMission(seed=a.seed, start_pile=a.start_pile, radar=a.radar, out_dir=a.out,
                           frames=a.frames, verbose=not a.quiet)
    except Exception:
        err = traceback.format_exc()
        fb = os.path.join(a.out, "frames")
        if glob.glob(os.path.join(fb, "state_*.json")):
            log(f"live GridMission failed, replaying {fb} instead:\n{err}")
            return ReplayMission(fb, a.out, a.seed, a.start_pile)
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

    from isaac.camera import DirectorCamera, focus_from_state
    from isaac.drone_viz import DroneViz, NullDrone
    from isaac.lidar import HeightmapLidar, PhysxLidar, PointAccumulator, RtxLidar
    from isaac.overlay import Overlay, make_backend

    if a.out:
        os.makedirs(a.out, exist_ok=True)
    mission = load_mission(a, say)
    sc = mission.scenario
    pile_xy = {p.name: (float(p.x), float(p.y)) for p in sc.piles}
    t0 = time.time()
    hm = sc.heightmap(0.5)                  # public geometry: overlay draping + dry-run LiDAR target
    say(f"heightmap {hm.w}x{hm.h} in {time.time() - t0:.1f} s")
    surface = hm.height_at

    stage, dd, rtx, grab, dust = None, None, None, None, None
    if a.dry_run:
        lidar = HeightmapLidar(hm, a.lidar_range, a.rays)
    else:
        from isaac import compat, scene
        stage = rt.stage()
        n = scene.build(stage, sc, show_truth=a.show_truth, physics_scene=rt.own_physics_scene, sun=a.sun,
                        sky=a.sky, materials="none" if a.look == "plain" else "preview")
        say(f"stage built: {n} rubble pieces, {scene.build.materials.bound} material bindings")
        if a.look == "pretty":
            from isaac import look
            look.apply(stage, sc, rt.mode, hdri=a.hdri, log=say)
        if compat.set_render_mode(a.rtx, a.spp):
            say(f"renderer: path tracing, {a.spp} spp, denoiser on")
        if a.overlay in ("auto", "debug"):
            dd = compat.debug_draw()
            if dd is None:
                say("debug_draw unavailable; overlay uses USD prims")
        lidar = PhysxLidar(compat.physx_raycaster(), a.lidar_range, a.rays)
        if a.record:                        # Isaac Lab Camera must exist before sim.reset()
            from isaac.selftest import _res
            grab = compat.FrameGrabber(rt, *_res(a.rec_res))
    ids = [d["id"] for d in as_dict(mission.build_state()).get("drones", [])] or ["d1", "d2", "d3"]
    if a.dry_run:
        drones = {i: NullDrone(None, f"/World/Drones/{i}") for i in ids}
    else:
        drones = {i: DroneViz(stage, f"/World/Drones/{i}", did=i, scale=a.drone_scale, asset=a.drone_asset)
                  for i in ids}
    overlay = Overlay(make_backend("null" if a.dry_run else a.overlay, stage, dd), surface, pile_xy)
    mirrors = []
    if not a.dry_run:
        from isaac import compat
        mirrors = ([compat.PERSP] if a.camera == "director" else []) + ([grab.mirror] if grab and grab.mirror else [])
    cam = DirectorCamera(stage, mode=a.camera, mirrors=mirrors, switch_viewport=(a.camera == "director"))
    if not a.dry_run and a.look == "pretty" and a.flow:
        from isaac import look
        dust = look.FlowDust(stage, ids, log=say)

    if not a.dry_run:
        rt.start_physics(30)                # Lab: sim.reset() + steps; Kit: force_load + play + updates
        if grab is not None:
            grab.attach_late()
            say(f"recording route: {grab.kind}{' (' + grab.err + ')' if grab.err else ''}")
        p0 = sc.piles[4]
        hit = lidar.cast((p0.x, p0.y, 25.0), (0, 0, -1), 60.0)
        if hit is None:
            say("WARNING PhysX raycast hit nothing (scene queries off? physics not started?); "
                "falling back to the heightmap LiDAR so the demo still shows points. Run --selftest.")
            lidar = HeightmapLidar(hm, a.lidar_range, a.rays)
        else:
            say(f"PhysX raycast self-test ok: hit z={hit[2]:.2f} at pile {p0.name}")
        if a.rtx_lidar:
            rtx = RtxLidar({i: d.path for i, d in drones.items()})
            rtx = rtx if rtx.ok else None

    accum = PointAccumulator()
    cur = prev = as_dict(mission.build_state())
    steps, frame = 0, 0
    virtual = a.dry_run or bool(a.record)   # fixed virtual frame rate: deterministic dry run, smooth recordings
    clock_v = 0.0
    clock = (lambda: clock_v) if virtual else time.monotonic
    last_step = next_step = clock()
    tilt_f = {i: (0.0, 0.0) for i in ids}
    moved = {i: 0.0 for i in ids}
    start_xy = {d["id"]: d["pose"][:3] for d in cur.get("drones", [])}
    active = overlay.update(cur)
    last_frame_t = clock()
    finished_at = None
    rdt_v = 1.0 / a.render_fps
    n_rec, t_rec = 0, time.time()
    if a.record:
        os.makedirs(os.path.join(a.record, "isaac"), exist_ok=True)

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
                last_step, next_step = next_step, next_step + dt_sim / max(a.speed, 1e-3)
                if now - next_step > 1.0:      # fell behind (slow step): resync instead of spiralling
                    last_step, next_step = now, now + dt_sim / max(a.speed, 1e-3)
                active = overlay.update(cur)
                done = mission.done or (a.max_steps and steps >= a.max_steps)
                if steps % 20 == 0:
                    say(f"step {steps} t={cur.get('t')} phase={cur.get('phase')} pile={active} "
                        f"lidar={accum.total} pts")

            span = max(next_step - last_step, 1e-6)
            u = 1.0 if done else min(max((now - last_step) / span, 0.0), 1.0)
            poses = interp_drones(prev, cur, u)
            rdt = rdt_v if virtual else max(now - last_frame_t, 1e-3)
            last_frame_t = now
            dt_sim_cur = max(float(cur.get("t", 0)) - float(prev.get("t", 0)), 1e-3)
            for did, (x, y, z, yaw, st, vx, vy) in poses.items():
                dr = drones.get(did)
                if dr is None:
                    continue
                sp = math.hypot(vx, vy) / dt_sim_cur
                ang = math.radians(math.degrees(math.atan2(vy, vx)) - yaw) if sp > 0.05 else 0.0
                tilt = min(14.0, 2.5 * sp) if not done else 0.0
                want = (-math.sin(ang) * tilt, math.cos(ang) * tilt)
                k = 1.0 - math.exp(-rdt / 0.4)
                tf = tilt_f[did]
                tilt_f[did] = (tf[0] + (want[0] - tf[0]) * k, tf[1] + (want[1] - tf[1]) * k)
                dr.set_pose(x, y, z, yaw, tilt_f[did])
                air = z - float(surface(x, y)) > 0.4
                dr.spin(rdt, on=air or st not in GROUNDED)
                sx = start_xy.get(did, (x, y, z))
                moved[did] = max(moved[did], math.dist((x, y, z), sx))
                if not done and is_surveying(st, cur.get("phase"), z - float(surface(x, y)) > 1.5):
                    pl = active if active in pile_xy else nearest_pile(pile_xy, x, y)
                    if pl:
                        pts = rtx.read(did) if rtx is not None else lidar.sweep((x, y, z + 0.35), yaw, frame)
                        new_pts = accum.add(pl, pts)
                        if new_pts is not None and len(new_pts):
                            pending.setdefault(pl, []).append(new_pts)
                            overlay.add_points(new_pts)
            if dust is not None:
                dust.update(poses, surface)

            focus, spread = focus_from_state(cur, pile_xy, active, list(poses.values()))
            cam.update(rdt, focus, spread)
            overlay.tick(now)
            frame += 1
            if rt is not None:
                if not rt.is_running():
                    break
                rt.step()
                if grab is not None and frame % max(a.rec_every, 1) == 0:
                    ok, _ = grab.save(os.path.join(a.record, "isaac", f"f{n_rec:05d}.png"))
                    n_rec += int(ok)
                    if n_rec and n_rec % 30 == 0:
                        say(f"recorded {n_rec} frames, {(time.time() - t_rec) / 30:.2f} s per frame")
                        t_rec = time.time()
            if virtual:
                clock_v += rdt_v
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
        "mission": type(mission).__name__, "launcher": a.launcher, "recorded_frames": n_rec,
    }
    say("summary " + json.dumps(summary))
    if n_rec:
        fps = a.render_fps / max(a.rec_every, 1)
        say(f"make the video: ffmpeg -framerate {fps:g} -i {a.record}/isaac/f%05d.png "
            f"-c:v libx264 -pix_fmt yuv420p {a.record}/isaac.mp4")

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
