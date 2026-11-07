"""SkySearch earthquake scenario inside NVIDIA Isaac Sim 6.0.1 + Isaac Lab 3.0 (Brev L40S launchable).

What you see in /viewer
  * The collapsed office building built from the same grid the swarm searches (same seed => same
    walls, rubble heaps, pancaked slabs over crevice voids, desks), with the roof cut away.
  * Real PhysX debris: concrete chunks fall when the scene starts (the earthquake) and again at
    every aftershock.
  * 10 drones moved by the SkySearch fleet code (skysearch/usar/fleet.py, unchanged): the
    coordinator mothership outside, 3 scouts flying at room height, 6 crevice drones that drop
    under slabs, land to listen, hover to confirm.
  * Overlays drawn in the viewport: LiDAR beams, radio links, microphone bearings, breathing-radar
    pings, smoke, victim beacons, and the crew's safe route at handoff.
  * One confirmer-drone camera rendered by Isaac (RTX) is shown in the dashboard's RGB tile.
Decisions use the same simulated sensor models as the CPU version (LiDAR ray casts on the grid,
microphones, radar, camera detector); Isaac renders the world, the debris physics and the camera.

Run (VS Code terminal on the Brev launchable; GPU must be running):
    cd /workspace/isaaclab && ./isaaclab.sh -p /workspace/skysearch/isaac/earthquake_scene.py \
        --livestream 2 --viz kit --enable_cameras --story
    # dashboard: <VS Code URL>/proxy/8090/      3D view: <brev URL>/viewer
Record a video headless (writes PNGs, then ffmpeg; see --help):
    ./isaaclab.sh -p /workspace/skysearch/isaac/earthquake_scene.py --headless --enable_cameras --story --record out/isaac
"""
import argparse, math, os, sys, time
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=3)
ap.add_argument("--story", action="store_true", help="team storyboard events (fire, victim B wakes, handoff)")
ap.add_argument("--speed", type=float, default=1.0, help="mission seconds per real second")
ap.add_argument("--port", type=int, default=8090, help="dashboard port (8080 is code-server on Brev)")
ap.add_argument("--record", default="", help="folder: save one Isaac frame + one dashboard frame per mission tick, then exit at handoff")
ap.add_argument("--rec_res", default="1280x720")
ap.add_argument("--debris", type=int, default=60, help="falling concrete chunks at start (PhysX)")
ap.add_argument("--sky", type=float, default=350.0, help="dome light intensity")
ap.add_argument("--sun", type=float, default=1800.0, help="sun (distant light) intensity")
ap.add_argument("--probe_frames", type=int, default=0, help="record this many ticks, print brightness, exit (lighting check)")

from isaaclab.app import AppLauncher  # noqa: E402
AppLauncher.add_app_launcher_args(ap)
a, _ = ap.parse_known_args()
a.enable_cameras = True
simulation_app = AppLauncher(a).app

import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
for _ext in ("isaacsim.core.experimental.prims", "isaacsim.core.experimental.objects",
             "isaacsim.core.experimental.materials", "isaacsim.util.debug_draw"):
    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(_ext, True)
import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.sensors.camera import Camera, CameraCfg  # noqa: E402
import isaacsim.core.experimental.objects as O  # noqa: E402
import isaacsim.core.experimental.prims as P  # noqa: E402
from isaacsim.core.experimental.materials import PreviewSurfaceMaterial  # noqa: E402
try:
    from isaacsim.util.debug_draw import _debug_draw
    draw = _debug_draw.acquire_debug_draw_interface()
except Exception:
    draw = None

from skysearch.usar.config import USARConfig  # noqa: E402
from skysearch.usar.building import OUT  # noqa: E402
from skysearch.usar.live import USARLive  # noqa: E402

FPS = 30
WALL_H = 3.0
# map (x right, y down = south) -> Isaac world (X east, Y north, Z up)
def W3(xy, z=0.0):
    return [float(xy[0]), -float(xy[1]), float(z)]


def look_at_quat(eye, target, up=(0.0, 0.0, 1.0)):
    """wxyz quaternion that points a USD camera (looks down local -Z, +Y up) from eye at target."""
    from scipy.spatial.transform import Rotation
    f = np.asarray(target, float) - np.asarray(eye, float); f /= np.linalg.norm(f)
    zc = -f
    x = np.cross(up, zc); x /= np.linalg.norm(x)
    y = np.cross(zc, x)
    qx, qy, qz, qw = Rotation.from_matrix(np.c_[x, y, zc]).as_quat()
    return [qw, qx, qy, qz]


def material(path, rgb, emissive=None):
    m = PreviewSurfaceMaterial(path)
    m.set_input_values("diffuseColor", list(rgb))
    if emissive is not None:
        try:
            m.set_input_values("emissiveColor", list(emissive))
        except Exception:
            pass
    return m


def runs(mask):
    """Row-wise runs of True cells -> (row, c0, c1) so walls/slabs become a few long boxes."""
    out = []
    for r in range(mask.shape[0]):
        row = mask[r]
        c = 0
        while c < len(row):
            if row[c]:
                c0 = c
                while c < len(row) and row[c]:
                    c += 1
                out.append((r, c0, c))
            else:
                c += 1
    return out


def boxes(name, mask, z0, z1, rgb, cell):
    """One batched set of boxes for all runs in a mask, from height z0 to z1 (arrays or scalars per cell)."""
    rr = runs(mask)
    if not rr:
        return None
    pos, scl = [], []
    for r, c0, c1 in rr:
        lo = float(np.mean(np.broadcast_to(z0, mask.shape)[r, c0:c1]))
        hi = float(np.mean(np.broadcast_to(z1, mask.shape)[r, c0:c1]))
        pos.append(W3(((c0 + c1) / 2 * cell, (r + 0.5) * cell), (lo + hi) / 2))
        scl.append([(c1 - c0) * cell, cell, max(hi - lo, 0.02)])
    prims = O.Cube([f"/World/Building/{name}/b{i}" for i in range(len(pos))], sizes=[1.0] * len(pos),
                   positions=np.array(pos), scales=np.array(scl))
    prims.apply_visual_materials(material(f"/World/Looks/{name}", rgb))
    return prims


def build_building(b, cfg):
    """Walls, rubble, collapsed slabs over voids, desks: all from the grid the swarm searches."""
    c = b.clear
    inside = (c < OUT)
    solid = inside & (c < 0.12)
    in_room = b.room_id >= 0
    walls = solid & ~in_room                         # structural walls between rooms
    rubble = solid & in_room                          # pancaked floor / debris piles
    tight = inside & (c >= 0.12) & (c < 0.25)         # gaps too small for any drone
    void = inside & (c >= 0.25) & (c < 0.9)           # crevices: only micro-drones fit
    low = inside & (c >= 0.9) & (c < 2.0)             # under desks / sagging ceiling: scouts fit
    rng = np.random.default_rng(cfg.seed + 11)
    heap = 1.2 + 0.9 * rng.random(c.shape)            # rubble heap heights
    boxes("walls", walls, 0.0, WALL_H, (0.62, 0.62, 0.60), cfg.cell)
    boxes("rubble", rubble, 0.0, heap, (0.42, 0.39, 0.35), cfg.cell)
    boxes("tight", tight, 0.0, np.clip(c + 0.6, 0, WALL_H), (0.47, 0.42, 0.36), cfg.cell)
    # a collapsed slab lies on top of each void: you see concrete, the drones see the gap below it
    boxes("slab_void", void, np.clip(c, 0.25, 0.9), np.clip(c, 0.25, 0.9) + 0.18, (0.55, 0.53, 0.50), cfg.cell)
    boxes("slab_low", low, np.clip(c, 0.9, 2.0), np.clip(c, 0.9, 2.0) + 0.15, (0.58, 0.57, 0.55), cfg.cell)
    floor = sim_utils.CuboidCfg(size=((b.x1 - b.x0) * cfg.cell, (b.y1 - b.y0) * cfg.cell, 0.1),
                                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.30, 0.29, 0.28)),
                                collision_props=sim_utils.CollisionPropertiesCfg())
    floor.func("/World/Building/floor", floor, translation=tuple(W3(((b.x0 + b.x1) / 2 * cfg.cell, (b.y0 + b.y1) / 2 * cfg.cell), -0.04)))


def spawn(cfg, path, pos):
    cfg.func(path, cfg, translation=tuple(float(v) for v in pos))
    return path


def drop_debris(cfg, b, n, center=None, radius=None, tag="d"):
    """Concrete chunks with real rigid-body physics, dropped from the upper floor (before sim.reset())."""
    rng = np.random.default_rng(int(time.time() * 1000) % 2**31)
    open_ = np.argwhere((b.clear >= 0.9) & (b.clear < OUT) & (b.room_id >= 0))
    if center is not None:
        d = np.hypot((open_[:, 1] + 0.5) * cfg.cell - center[0], (open_[:, 0] + 0.5) * cfg.cell - center[1])
        open_ = open_[d < radius] if (d < radius).any() else open_
    paths = []
    for i in range(n):
        r, c = open_[rng.integers(len(open_))]
        s = rng.uniform(0.15, 0.45, 3)
        chunk = sim_utils.CuboidCfg(size=tuple(float(v) for v in s),
                                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.48, 0.45)),
                                    rigid_props=sim_utils.RigidBodyPropertiesCfg(),
                                    mass_props=sim_utils.MassPropertiesCfg(mass=float(s.prod() * 2400)),
                                    collision_props=sim_utils.CollisionPropertiesCfg())
        p = f"/World/Debris/{tag}{i}"
        spawn(chunk, p, W3(((c + 0.5) * cfg.cell, (r + 0.5) * cfg.cell), WALL_H + rng.uniform(0.2, 2.5)))
        paths.append(p)
    return paths


def build_stage(scen, cfg):
    b, f = scen.b, scen.fleet
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / FPS, device=a.device, enable_scene_query_support=True))
    # measured on the L40S: dome 1200 + overhead sun 3000 blows every surface out to white at building scale.
    # Dimmer sky, and a low, tilted sun so walls, rubble and slabs get shading and shadows.
    dome = sim_utils.DomeLightCfg(intensity=a.sky, color=(0.80, 0.85, 1.0))
    dome.func("/World/Dome", dome)
    sun = sim_utils.DistantLightCfg(intensity=a.sun, angle=1.0, color=(1.0, 0.95, 0.85))
    sun.func("/World/Sun", sun, orientation=tuple(look_at_quat([0, 0, 0], [-0.55, 0.35, -0.75])))
    Wm, Hm = cfg.W * cfg.cell, cfg.H * cfg.cell
    ground = sim_utils.CuboidCfg(size=(Wm + 30, Hm + 30, 0.2), visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.16, 0.17, 0.18)),
                                 collision_props=sim_utils.CollisionPropertiesCfg())
    spawn(ground, "/World/Ground", W3((Wm / 2, Hm / 2), -0.15))
    build_building(b, cfg)
    # people (semantic label "person") and look-alike decoys (mannequins / clothes)
    person = sim_utils.CapsuleCfg(radius=0.18, height=1.3, axis="X", semantic_tags=[("class", "person")],
                                  visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.85, 0.55, 0.35)))
    decoy = sim_utils.CapsuleCfg(radius=0.17, height=1.2, axis="X", semantic_tags=[("class", "decoy")],
                                 visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.70, 0.68, 0.66)))
    people = [spawn(person, f"/World/People/p{i}", W3(s["pos"], 0.18)) for i, s in enumerate(b.survivors)]
    decoys = [spawn(decoy, f"/World/Decoys/k{i}", W3(d["pos"], 0.17)) for i, d in enumerate(b.decoys)]
    # drones: mothership (big), scouts, crevice micro-drones
    col = {"coordinator": (1.0, 0.84, 0.3), "mapper": (0.31, 0.76, 0.97), "radar": (0.15, 0.78, 0.85),
           "listener": (0.81, 0.58, 0.85), "confirmer": (0.0, 0.9, 1.0)}
    dpaths = []
    for d in f.drones:
        if d.kind == "mother":
            body = sim_utils.CylinderCfg(radius=0.9, height=0.3, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=col["coordinator"]))
        elif d.kind == "scout":
            body = sim_utils.CuboidCfg(size=(0.8, 0.8, 0.18), visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=col[d.role]))
        else:
            body = sim_utils.CuboidCfg(size=(0.32, 0.32, 0.1), visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=col[d.role]))
        dpaths.append(spawn(body, f"/World/Drones/{d.name}", W3(d.pos, 0.3)))
    # fire marker (moved into place when a fire starts)
    fire = sim_utils.SphereCfg(radius=0.45, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.45, 0.05), emissive_color=(1.0, 0.35, 0.0)))
    spawn(fire, "/World/Fire", (0.0, 0.0, -50.0))
    # cameras: one on a confirmer drone (shown in the dashboard), one for recording the overview
    focal = 12.0
    dcam = Camera(CameraCfg(prim_path="/World/Cams/drone", update_period=0, height=160, width=160, data_types=["rgb"],
                            spawn=sim_utils.PinholeCameraCfg(focal_length=focal, horizontal_aperture=2 * focal * math.tan(math.radians(50)),
                                                             clipping_range=(0.05, 200.0))))
    rw, rh = (int(v) for v in a.rec_res.split("x"))
    rec = Camera(CameraCfg(prim_path="/World/Cams/rec", update_period=0, height=rh, width=rw, data_types=["rgb"],
                           spawn=sim_utils.PinholeCameraCfg(focal_length=18.0, clipping_range=(0.1, 1.0e4)))) if a.record else None
    debris = drop_debris(cfg, b, a.debris)
    sim.reset()
    return sim, P.XformPrim(people), P.XformPrim(decoys), P.XformPrim(dpaths), dcam, rec, debris


def drone_z(d, b, mother_z):
    c = float(b.clear[b.cell_of(d.pos)])
    if d.kind == "mother":
        return mother_z
    if not d.alive:
        return 0.05
    if d.kind == "scout":
        return float(np.clip(min(1.5, c - 0.35), 0.5, 1.5)) if c < OUT else 1.6
    if d.state == "LISTENING (landed)":
        return 0.04
    if d.state == "CHARGE":
        return mother_z + 0.2
    if c < 0.9:
        return max(0.08, c * 0.45)
    return 0.7 if c < OUT else 1.0


def view_pose(t, b, cfg):
    """Slow orbit around the building, high enough to see into the roofless rooms."""
    cx, cy = (b.x0 + b.x1) / 2 * cfg.cell, (b.y0 + b.y1) / 2 * cfg.cell
    ang = math.radians(-100 + 25 * math.sin(t / 40))
    eye = [cx + 20 * math.cos(ang), -cy + 20 * math.sin(ang), 19.0]
    return eye, [cx, -cy, 0.0]


def overlays(scen, pos3):
    if draw is None:
        return
    draw.clear_lines(); draw.clear_points()
    f, b, cfg = scen.fleet, scen.b, scen.cfg
    S, E, C, Wd = [], [], [], []
    by = {d.name: p for d, p in zip(f.drones, pos3)}
    for d, p in zip(f.drones, pos3):
        if d.alive and d.kind == "scout" and d.lidar_hits is not None and len(d.lidar_hits):
            for e in d.lidar_hits[::3]:
                S.append(tuple(p)); E.append(tuple(W3(e, p[2]))); C.append((0.3, 0.76, 0.97, 0.35)); Wd.append(1)
        if d.alive and d.kind == "micro":
            for (t0, brg, sg) in d.bear_vis:
                if f.t - t0 < 6:
                    q = np.array([d.pos[0] + 8 * math.cos(brg), d.pos[1] + 8 * math.sin(brg)])
                    S.append(tuple(p)); E.append(tuple(W3(q, p[2]))); C.append((0.92, 0.5, 0.99, 1 - (f.t - t0) / 6)); Wd.append(3)
            for xy, solid in d.radar_dets:
                S.append(tuple(p)); E.append(tuple(W3(xy, 0.2))); C.append((0.0, 0.9, 1.0, 1)); Wd.append(2)
    for pa, pb, margin in f.links:
        S.append(tuple(W3(pa, 1.8))); E.append(tuple(W3(pb, 1.8))); C.append((0.5, 0.95, 0.5, 0.25)); Wd.append(1)
    if f.handoff_t is not None:
        for fd in f.found:
            pth = fd.get("path") or []
            for p0, p1 in zip(pth[:-1], pth[1:]):
                S.append(tuple(W3(p0, 0.08))); E.append(tuple(W3(p1, 0.08))); C.append((0.46, 1.0, 0.0, 1)); Wd.append(6)
    for fd in f.found:            # victim beacons: a light column above each pin
        col = (0.0, 0.9, 0.46, 1) if fd["conf"] == "HIGH" else (1.0, 0.77, 0.0, 1)
        S.append(tuple(W3(fd["xy"], 0.2))); E.append(tuple(W3(fd["xy"], 4.0))); C.append(col); Wd.append(8)
    if S:
        draw.draw_lines(S, E, C, Wd)
    pts, pc, ps = [], [], []
    if b.smoke.max() > 0.1:       # smoke: translucent grey points, denser where thicker
        rr, cc = np.nonzero(b.smoke > 0.15)
        rng = np.random.default_rng(int(f.t * 2))
        for k in rng.choice(len(rr), min(len(rr), 900), replace=False):
            v = float(b.smoke[rr[k], cc[k]])
            pts.append(tuple(W3(((cc[k] + rng.random()) * cfg.cell, (rr[k] + rng.random()) * cfg.cell), rng.uniform(0.3, 2.8))))
            pc.append((0.55, 0.56, 0.58, 0.15 + 0.4 * v)); ps.append(22)
    for fr in b.fires:
        for _ in range(12):
            j = np.random.normal(0, 0.25, 3)
            pts.append(tuple(np.array(W3(fr["xy"], 0.6)) + j)); pc.append((1.0, 0.5 + 0.4 * np.random.rand(), 0.0, 0.9)); ps.append(14)
    if pts:
        draw.draw_points(pts, pc, ps)


def main():
    cfg = USARConfig(seed=a.seed, story=a.story)
    live = USARLive(cfg, port=a.port, speed=a.speed)
    scen = live.scen
    sim, people, decoys, drones, dcam, rec, debris = build_stage(scen, cfg)
    live.serve()
    sub = max(1, int(round(cfg.dt * FPS / a.speed)))           # render frames per mission tick
    prev = np.array([d.pos for d in scen.fleet.drones], float)
    cur = prev.copy()
    zs = np.zeros(len(scen.fleet.drones))
    k, n_tick, last_pub, last_shock_t, shocks = 0, 0, 0.0, None, 0
    built_clear = scen.b.clear.copy()
    rec_dir = a.record
    if rec_dir:
        os.makedirs(os.path.join(rec_dir, "isaac"), exist_ok=True)
        os.makedirs(os.path.join(rec_dir, "dash"), exist_ok=True)
    print(f"[skysearch] earthquake scene ready: {len(scen.fleet.drones)} drones, {len(debris)} debris chunks, "
          f"{sub} render frames per 0.5 s mission tick", flush=True)
    while simulation_app.is_running():
        with live.lock:
            if live.drain():
                scen = live.scen
                prev = cur = np.array([d.pos for d in scen.fleet.drones], float)
            if k % sub == 0 and not live.paused:
                prev = np.array([d.pos for d in scen.fleet.drones], float)
                scen.step(); live.r.note_radar()
                cur = np.array([d.pos for d in scen.fleet.drones], float)
                n_tick += 1
            al = 1.0 if live.paused else (k % sub + 1) / sub
            pos2 = prev + (cur - prev) * al
            live.r.pos = {d.name: pos2[i] for i, d in enumerate(scen.fleet.drones)}
            b, f = scen.b, scen.fleet
            # earthquake: building shakes at start and during aftershocks (camera + drones jitter)
            shake = 0.0
            if f.t < 3.0 or b.shaking(f.t):
                shake = 0.06
            if b.last_shock is not None and b.last_shock["t"] != last_shock_t:
                # aftershock: show the new collapse (visual boxes; rigid bodies can't be added after sim.reset())
                last_shock_t = b.last_shock["t"]; shocks += 1
                newly_solid = (b.clear < 0.12) & (built_clear >= 0.12)
                newly_void = (b.clear >= 0.25) & (b.clear < 0.9) & (built_clear >= 0.9) & (b.clear < OUT)
                boxes(f"shock{shocks}_rubble", newly_solid, 0.0, 1.6, (0.45, 0.40, 0.34), cfg.cell)
                boxes(f"shock{shocks}_slab", newly_void, np.clip(b.clear, 0.25, 0.9), np.clip(b.clear, 0.25, 0.9) + 0.18,
                      (0.55, 0.53, 0.50), cfg.cell)
                built_clear = b.clear.copy()
            mother_z = 4.0
            for i, d in enumerate(f.drones):
                zs[i] += (drone_z(d, b, mother_z) - zs[i]) * 0.25
            pos3 = np.array([W3(p, z) for p, z in zip(pos2, zs)])
            if shake:
                pos3 += np.random.normal(0, shake * 0.5, pos3.shape)
            drones.set_world_poses(positions=pos3)
            people.set_world_poses(positions=np.array([W3(s["pos"], 0.18) for s in b.survivors]))
            if b.fires:
                P.XformPrim("/World/Fire").set_world_poses(positions=[W3(b.fires[0]["xy"], 0.5)])
            overlays(scen, pos3)
            # the confirmer camera: looks along the drone's heading, slightly down
            focus = next((d for d in f.alive("micro") if d.role == "confirmer" and d.state.startswith("HOVER")), None) \
                or next((d for d in f.alive("micro") if d.role == "confirmer"), None)
            if focus is not None:
                i = f.drones.index(focus)
                eye = pos3[i] + np.array([0, 0, 0.05])
                tgt = eye + np.array([math.cos(focus.heading), -math.sin(focus.heading), -0.25])
                P.XformPrim("/World/Cams/drone").set_world_poses(positions=[eye], orientations=[look_at_quat(eye, tgt)])
            # viewport + recording camera, re-posed every frame (docs/ISAAC6_API.md, open issue 4)
            eye, tgt = view_pose(k / FPS, b, cfg)
            if shake:
                eye = list(np.array(eye) + np.random.normal(0, shake * 2, 3))
            q = look_at_quat(eye, tgt)
            try:
                P.XformPrim("/OmniverseKit_Persp", reset_xform_op_properties=(k == 0)).set_world_poses(positions=[eye], orientations=[q])
            except Exception as e:          # headless runs may have no Kit viewport camera
                if k == 0:
                    print(f"[skysearch] no viewport camera ({type(e).__name__}); recording camera only", flush=True)
            if rec is not None:
                P.XformPrim("/World/Cams/rec").set_world_poses(positions=[eye], orientations=[q])
        sim.step()
        k += 1
        rgb = None
        if focus is not None:
            dcam.update(dt=sim.get_physics_dt())
            rgb = dcam.data.output["rgb"][0].cpu().numpy()[..., :3]
        if rec_dir and k % sub == 0:
            import matplotlib.pyplot as plt
            rec.update(dt=sim.get_physics_dt())
            img = rec.data.output["rgb"][0].cpu().numpy()[..., :3]
            plt.imsave(os.path.join(rec_dir, "isaac", f"f{n_tick:05d}.png"), img)
            live.r.rgb_img = rgb
            live.r.save(os.path.join(rec_dir, "dash", f"f{n_tick:05d}.png"), dpi=67)
            if a.probe_frames and n_tick >= a.probe_frames:
                print(f"[skysearch] probe: isaac frame mean={img.mean() * (255 if img.max() <= 1.0 else 1):.1f} "
                      f"p5={np.percentile(img, 5):.3f} p95={np.percentile(img, 95):.3f}", flush=True)
                break
            if scen.fleet.handoff_t is not None and scen.fleet.t > scen.fleet.handoff_t + 4:
                print(f"[skysearch] recorded {n_tick} ticks to {rec_dir}. Make the video with:\n"
                      f"  ffmpeg -framerate 6 -i {rec_dir}/isaac/f%05d.png -framerate 6 -i {rec_dir}/dash/f%05d.png "
                      f"-filter_complex \"[0:v]scale=1280:720[a];[1:v]scale=1280:720[b];[a][b]hstack\" -c:v libx264 -pix_fmt yuv420p {rec_dir}/isaac_story.mp4",
                      flush=True)
                break
        elif time.time() - last_pub > 1.0:
            live.publish_async({"rgb": rgb}); last_pub = time.time()
            if n_tick % 20 == 0:
                print(f"[skysearch] t={scen.fleet.t:.0f}s found={len(scen.fleet.found)} "
                      f"captions={scen.fleet.captions[-1][1] if scen.fleet.captions else ''}", flush=True)
    simulation_app.close()


if __name__ == "__main__":
    main()
