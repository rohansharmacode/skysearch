"""Run the 3x3 rubble-grid SAR mission inside Isaac Sim (or ``--dry-run`` without omni/pxr).

    ~/isaacsim/python.sh isaac/run_mission.py --seed 7 --speed 4                 # Isaac GUI
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


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--start-pile", default=None, help="pile to begin at, e.g. B2 (default: scenario's)")
    ap.add_argument("--radar", choices=["model", "dsp"], default="model", help="dsp needs scipy in Isaac's python")
    ap.add_argument("--speed", type=float, default=4.0, help="sim-seconds per wall-second")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--livestream", action="store_true", help="enable the WebRTC livestream extension (headless)")
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
                    help="debug = debug_draw lines/points; usd = prims only (shows in Movie Capture)")
    ap.add_argument("--rays", type=int, default=96, help="LiDAR rays per surveying drone per frame")
    ap.add_argument("--lidar-range", type=float, default=40.0)
    ap.add_argument("--render-fps", type=float, default=30.0, help="virtual frame rate for --dry-run")
    ap.add_argument("--no-keep-open", action="store_true", help="close the GUI when the mission ends")
    ap.add_argument("--quiet", action="store_true")
    from isaac.photoreal import add_args as _photoreal_args     # --photoreal 1, --record DIR, --rtx pathtracing ...
    _photoreal_args(ap)
    return ap.parse_args(argv)


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
    app = None
    if not a.dry_run:                       # SimulationApp FIRST, before any omni/pxr import
        from isaac import compat
        app = compat.make_app(a.headless)
        if a.livestream:
            ok = any(compat.enable_ext(e) for e in ("omni.kit.livestream.webrtc", "omni.kit.livestream.native"))
            say(f"livestream extension {'enabled' if ok else 'NOT available'}")
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

    stage, dd, rtx = None, None, None
    if a.dry_run:
        lidar = HeightmapLidar(hm, a.lidar_range, a.rays)
    else:
        from isaac import compat, scene
        stage = compat.new_stage()
        for _ in range(3):
            app.update()
        n = scene.build(stage, sc, show_truth=a.show_truth)
        say(f"stage built: {n} rubble pieces")
        if a.photoreal:                     # visual only; colliders keep the scenario's piece poses
            from isaac import photoreal
            photoreal.dress(stage, sc, a, say)
        if a.overlay in ("auto", "debug"):
            dd = compat.debug_draw()
            if dd is None:
                say("debug_draw unavailable; overlay uses USD prims")
        lidar = PhysxLidar(compat.physx_raycaster(), a.lidar_range, a.rays)
    ids = [d["id"] for d in as_dict(mission.build_state()).get("drones", [])] or ["d1", "d2", "d3"]
    if a.dry_run:
        drones = {i: NullDrone(None, f"/World/Drones/{i}") for i in ids}
    else:
        drones = {i: DroneViz(stage, f"/World/Drones/{i}", did=i, scale=a.drone_scale, asset=a.drone_asset)
                  for i in ids}
    overlay = Overlay(make_backend("null" if a.dry_run else a.overlay, stage, dd), surface, pile_xy)
    cam = DirectorCamera(stage, mode=a.camera)

    if not a.dry_run:
        compat.force_load_physics()
        compat.timeline_play()
        for _ in range(30):                 # warm-up: PhysX parses colliders, renderer compiles shaders
            app.update()
        p0 = sc.piles[4]
        hit = lidar.cast((p0.x, p0.y, 25.0), (0, 0, -1), 60.0)
        if hit is None:
            say("WARNING PhysX raycast hit nothing (timeline playing? colliders parsed?); "
                "falling back to the heightmap LiDAR so the demo still shows points")
            lidar = HeightmapLidar(hm, a.lidar_range, a.rays)
        else:
            say(f"PhysX raycast self-test ok: hit z={hit[2]:.2f} at pile {p0.name}")
        if a.rtx_lidar:
            rtx = RtxLidar({i: d.path for i, d in drones.items()})
            rtx = rtx if rtx.ok else None

    accum = PointAccumulator()
    cur = prev = as_dict(mission.build_state())
    steps, frame = 0, 0
    clock_v = 0.0
    virtual = a.dry_run or bool(a.record)   # recording: fixed frame clock, so slow (path-traced) frames don't skip
    if a.record:
        a.speed = a.rec_speed
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
    if a.record and app is not None:
        from isaac import photoreal
        photoreal.set_renderer(a, say)
        specs = [c.strip() for c in a.rec_cams.split(",") if c.strip()]
        rigs = photoreal.Rigs(stage, [c for c in specs if c != "director"], surface)
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
            app.update()

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
                if a.record and a.rec_warp > 1:     # recording: compress the long radar stretches
                    from isaac.photoreal import WARP_PHASES
                    a.speed = a.rec_speed * (a.rec_warp if cur.get("phase") in WARP_PHASES else 1.0)
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
            rdt = max(now - last_frame_t, 1e-3) if not virtual else rdt_v
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
                            show = new_pts
                            if a.photoreal and pl in pile_xy:   # the map, not hits sprayed over the whole site
                                from isaac.photoreal import LIDAR_DRAW_R
                                c = np.asarray(pile_xy[pl], float)[:2]
                                show = new_pts[np.hypot(new_pts[:, 0] - c[0], new_pts[:, 1] - c[1]) < LIDAR_DRAW_R]
                            if len(show):
                                overlay.add_points(show)

            focus, spread = focus_from_state(cur, pile_xy, active, list(poses.values()))
            cam.update(rdt, focus, spread)
            if rigs is not None:
                rigs.update(poses, focus, rdt)
            overlay.tick(now)
            frame += 1
            if app is not None:
                if not app.is_running():
                    break
                if rec is not None:
                    rec.prepare(frame // max(1, a.rec_every))
                app.update()
                if rec is not None and frame % max(1, a.rec_every) == 0:
                    rec.capture(frame // max(1, a.rec_every))
                    rec.note(frame // max(1, a.rec_every), cur, active)
                if virtual:
                    clock_v += rdt_v
            else:
                clock_v += rdt_v
            if a.record and a.rec_stop_t and float(cur.get("t", 0)) >= a.rec_stop_t:
                done = True
            if rec is not None and rec.done(frame // max(1, a.rec_every)):
                done = True                     # past the last window
            if done:
                finished_at = finished_at or now
                if a.dry_run or a.headless or a.no_keep_open or now - finished_at > 2.0:
                    break
    except Exception:
        traceback.print_exc()
        if app is None or a.headless:
            if app is not None:
                app.close()
            raise
        say("error above; keeping the window open for inspection")

    state_path = os.path.join(a.out, "state.json") if a.out else None
    summary = {
        "steps": steps, "t": cur.get("t"), "phase": cur.get("phase"), "frames": frame,
        "drone_moved_m": {k: round(v, 2) for k, v in moved.items()},
        "lidar_points": int(accum.total), "piles_scanned": sorted(accum.chunks), "ingested": fed,
        "overlay_updates": overlay.updates, "camera_updates": cam.updates,
        "state_json": state_path if state_path and os.path.exists(state_path) else None,
        "mission": type(mission).__name__,
    }
    say("summary " + json.dumps(summary))

    if app is not None:
        if not a.headless and not a.no_keep_open:
            say("mission finished; window stays open (close it to exit)")
            last = time.monotonic()
            while app.is_running():
                now = time.monotonic()
                cam.update(now - last, focus_from_state(cur, pile_xy, active, list(poses.values()))[0])
                overlay.tick(now)
                last = now
                app.update()
        app.close()
    return summary


if __name__ == "__main__":
    main()
