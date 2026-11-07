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
ap.add_argument("--collapse", action="store_true", help="physically simulated earthquake before launch (isaac/quake.py)")
ap.add_argument("--pga", type=float, default=0.45, help="--collapse: peak ground acceleration in g")
ap.add_argument("--dust_points", type=float, default=1.0, help="--collapse: opacity of the debug-draw dust points (0 = off, e.g. with Flow dust)")
ap.add_argument("--reveal", type=int, default=1, help="--collapse: hide rubble mounds/gravel until the upper floor comes down")
ap.add_argument("--rec_cams", default="", help="extra recording cameras, comma list: chase:C1, fpv:C1, fpv:S1, collapse "
                "(collapse = a person's eye height in the corridor, looking at the damage). Saved to <record>/isaac_<name>/")
ap.add_argument("--stop_t", type=float, default=0.0, help="--record: stop at this mission time (s) instead of at handoff")
ap.add_argument("--drone_light", type=float, default=12000.0, help="intensity of the small search light on each scout/crevice drone (0 = off)")
ap.add_argument("--look", type=int, default=1, help="1 = photoreal materials/sky/models/cinematic camera (isaac/look.py)")
ap.add_argument("--hdri", type=float, default=900.0, help="HDRI sky dome intensity when --look 1")
ap.add_argument("--jagged", type=int, default=0, help="--look 1: visual-only jagged meshes over box rubble (fallback; rubble.py does real shapes)")
ap.add_argument("--greenery", type=int, default=1, help="--look 1: trees, shrubs, lawn, paving")
ap.add_argument("--flow", type=int, default=1, help="--look 1: Omni Flow volumetric dust (collapse) and fire")
ap.add_argument("--rtx", default="realtime", choices=["realtime", "pathtracing"], help="renderer for recorded frames")
ap.add_argument("--spp", type=int, default=16, help="--rtx pathtracing: samples per pixel per frame")
ap.add_argument("--quake_every", type=int, default=5, help="--record: save every Nth physics frame during the quake (1 = 30 fps)")
ap.add_argument("--wall_mdl", default="concrete_smooth", help="--look 1: wall material key in look.MDL (concrete_smooth, concrete_formed, plaster)")
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


def boxes(name, mask, z0, z1, rgb, cell, per_cell=False, thin=None):
    """One batched set of boxes for all runs in a mask, from height z0 to z1 (arrays or scalars per cell).
    per_cell: one box per cell (jagged heights); thin: box footprint in metres (e.g. desk legs). Returns prim paths."""
    rr = [(r, c, c + 1) for r, c in np.argwhere(mask)] if per_cell else runs(mask)
    if not rr:
        return []
    pos, scl = [], []
    for r, c0, c1 in rr:
        lo = float(np.mean(np.broadcast_to(z0, mask.shape)[r, c0:c1]))
        hi = float(np.mean(np.broadcast_to(z1, mask.shape)[r, c0:c1]))
        pos.append(W3(((c0 + c1) / 2 * cell, (r + 0.5) * cell), (lo + hi) / 2))
        scl.append([thin or (c1 - c0) * cell, thin or cell, max(hi - lo, 0.02)])
    paths = [f"/World/Building/{name}/b{i}" for i in range(len(pos))]
    prims = O.Cube(paths, sizes=[1.0] * len(pos), positions=np.array(pos), scales=np.array(scl))
    prims.apply_visual_materials(material(f"/World/Looks/{name}", rgb))
    return paths


def collide(paths):
    """Static colliders on visual boxes, so falling debris hits walls, heaps and desks (--collapse)."""
    try:
        P.GeomPrim(paths, apply_collision_apis=True)
    except Exception as e:
        print(f"[skysearch] colliders failed for {paths[0].rsplit('/', 1)[0]}: {type(e).__name__}: {e}", flush=True)


def build_building(b, cfg, Q=None):
    """Walls, rubble, collapsed slabs over voids, desks: all from the grid the swarm searches."""
    if Q is not None:          # --collapse: static parts with colliders; slabs and debris are rigid bodies (quake.py)
        Q.build_static(lambda name, mask, z0, z1, rgb, **kw: boxes(name, mask, z0, z1, rgb, cfg.cell, **kw), collide)
        _floor(b, cfg)
        return
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
    _floor(b, cfg)


def _floor(b, cfg):
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


EXTRA_CAMS = []          # --rec_cams rigs: dict(mode, who, cam, path, tag, eye, h)


def build_stage(scen, cfg, Q=None):
    b, f = scen.b, scen.fleet
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / FPS, device=a.device, enable_scene_query_support=True))
    if a.rtx == "pathtracing":                  # final-quality frames: RTX path tracing + denoiser
        import carb
        cs = carb.settings.get_settings()
        cs.set_string("/rtx/rendermode", "PathTracing")
        cs.set_int("/rtx/pathtracing/spp", a.spp)
        cs.set_int("/rtx/pathtracing/totalSpp", a.spp)
        cs.set_bool("/rtx/pathtracing/optixDenoiser/enabled", True)
        print(f"[skysearch] renderer: path tracing, {a.spp} spp, denoiser on", flush=True)
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
    build_building(b, cfg, Q)
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
    for spec in [x.strip() for x in a.rec_cams.split(",") if x.strip()]:
        mode, _, who = spec.partition(":")
        path = f"/World/Cams/x{len(EXTRA_CAMS)}"
        focal = 8.5 if mode == "fpv" else 13.0              # FPV: wide (~100 deg), like a real FPV camera
        cam = Camera(CameraCfg(prim_path=path, update_period=0, height=rh, width=rw, data_types=["rgb"],
                               spawn=sim_utils.PinholeCameraCfg(focal_length=focal, clipping_range=(0.02, 1.0e4))))
        EXTRA_CAMS.append(dict(mode=mode, who=who or "C1", cam=cam, path=path, tag=spec.replace(":", "_"), eye=None, h=None))
    if a.drone_light > 0:           # small search lights: crevice voids are dark
        for d in f.drones:
            if d.kind == "mother":
                continue
            try:
                lc = sim_utils.SphereLightCfg(intensity=a.drone_light * (2.0 if d.kind == "scout" else 1.0), radius=0.03,
                                              color=(1.0, 0.97, 0.92))
                lc.func(f"/World/Drones/{d.name}/light", lc, translation=(0.0, 0.0, -0.06))
            except Exception as e:
                print(f"[skysearch] drone light off ({type(e).__name__}: {e})", flush=True)
                break
    if Q is not None:
        from isaaclab.assets import RigidObject, RigidObjectCfg
        Q.build_dynamic(RigidObject, RigidObjectCfg)
        Q.posts_boxes()                         # crushed-debris stacks under the slab corners (meshes)
        debris = []
    else:
        debris = drop_debris(cfg, b, a.debris)
    if a.look:                                  # photoreal look (isaac/look.py); visual only
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import look
        a._quake = Q                           # look.py places Flow dust at the quake epicentre
        look.apply(scen, cfg, a, W3, OUT, log=lambda m: print(m, flush=True))
    if Q is not None:
        Q.after_look()                          # rebar: steel, not the concrete look.apply put on /World/Quake
    sim.reset()
    if Q is not None:
        Q.after_reset(sim)
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


def _posts(name, xys, zs, rgb):
    pos = [W3(xy, z / 2) for xy, z in zip(xys, zs)]
    scl = [[0.16, 0.16, max(z, 0.05)] for z in zs]
    prims = O.Cube([f"/World/Building/{name}/b{i}" for i in range(len(pos))], sizes=[1.0] * len(pos),
                   positions=np.array(pos), scales=np.array(scl))
    prims.apply_visual_materials(material(f"/World/Looks/{name}", rgb))


def rig_pose(rc, scen, pos3, Q):
    """Eye/target for an extra recording camera: chase (3rd person), fpv (1st person) or collapse (corridor)."""
    f, b = scen.fleet, scen.b
    quaking = Q is not None and not Q.done
    d = next((x for x in f.drones if x.name == rc["who"]), None)
    if rc["mode"] == "collapse" or quaking or d is None:
        ex = Q.epi[0] if Q is not None else (b.x0 + b.x1) / 2 * scen.cfg.cell
        ey = Q.epi[1] if Q is not None else b.entrance[1]
        tq = Q.t if quaking else 12.0
        eye = np.array(W3((ex - 7.5 + 0.12 * tq, b.entrance[1] + 0.3), 1.65))
        tgt = np.array(W3((ex, ey), 1.2))
    else:
        i = f.drones.index(d)
        h = d.heading if rc["h"] is None else rc["h"] + ((d.heading - rc["h"] + math.pi) % (2 * math.pi) - math.pi) * 0.12
        rc["h"] = h
        fwd = np.array([math.cos(h), -math.sin(h), 0.0])
        p = pos3[i]
        if rc["mode"] == "fpv":            # lens at the front edge of the airframe, not inside the body
            eye = p + (0.30 if d.kind == "micro" else 0.60) * fwd + np.array([0, 0, 0.02])
            tgt = eye + 3.0 * fwd - np.array([0, 0, 0.5])
        else:
            back, up = (0.9, 0.22) if d.kind == "micro" else (2.2, 0.9)
            mf = np.array([math.cos(h), math.sin(h)])            # forward in map coordinates (y down)
            H_, W_ = b.clear.shape
            for sd in np.linspace(0.1, back, 10):                # don't put the camera inside a wall or slab
                q = d.pos - sd * mf
                r_, c_ = int(np.clip(q[1] / scen.cfg.cell, 0, H_ - 1)), int(np.clip(q[0] / scen.cfg.cell, 0, W_ - 1))
                if b.clear[r_, c_] < p[2] + up + 0.08:
                    back = max(0.2, sd - 0.25)
                    up = min(up, max(0.05, b.clear[r_, c_] - p[2] - 0.1))
                    break
            eye = p - back * fwd + np.array([0, 0, up])
            tgt = p + 2.0 * fwd - np.array([0, 0, 0.1])
    if rc["eye"] is not None and rc["mode"] != "fpv":
        eye = 0.8 * rc["eye"] + 0.2 * eye                  # camera operator smoothing
    rc["eye"] = eye
    if quaking:
        eye = eye + Q.camera_offset(Q.t) * 0.6
    return list(map(float, eye)), list(map(float, tgt))


def overlays(scen, pos3, extra_pts=None):
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
    if extra_pts and extra_pts[0]:
        pts += list(extra_pts[0]); pc += list(extra_pts[1]); ps += list(extra_pts[2])
    if pts:
        draw.draw_points(pts, pc, ps)


def main():
    cfg = USARConfig(seed=a.seed, story=a.story)
    live = USARLive(cfg, port=a.port, speed=a.speed)
    scen = live.scen
    Q = None
    if a.collapse:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import quake as quake_mod
        Q = quake_mod.Quake(scen, cfg, sim_utils, W3, WALL_H, pga=a.pga, seed=a.seed, log=lambda m: print(m, flush=True),
                             dust=a.dust_points, reveal=bool(a.reveal))
    sim, people, decoys, drones, dcam, rec, debris = build_stage(scen, cfg, Q)
    live.serve()
    sub = max(1, int(round(cfg.dt * FPS / a.speed)))           # render frames per mission tick
    prev = np.array([d.pos for d in scen.fleet.drones], float)
    cur = prev.copy()
    zs = np.zeros(len(scen.fleet.drones))
    k, n_tick, n_rec, last_pub, last_shock_t, shocks = 0, 0, 0, 0.0, None, 0
    built_clear = scen.b.clear.copy()
    rec_dir = a.record
    if rec_dir:
        os.makedirs(os.path.join(rec_dir, "isaac"), exist_ok=True)
        os.makedirs(os.path.join(rec_dir, "dash"), exist_ok=True)
        for rc in EXTRA_CAMS:
            os.makedirs(os.path.join(rec_dir, f"isaac_{rc['tag']}"), exist_ok=True)
    print(f"[skysearch] earthquake scene ready: {len(scen.fleet.drones)} drones, {len(debris)} debris chunks, "
          f"{sub} render frames per 0.5 s mission tick", flush=True)
    while simulation_app.is_running():
        with live.lock:
            if live.drain():
                scen = live.scen
                prev = cur = np.array([d.pos for d in scen.fleet.drones], float)
            quaking = Q is not None and not Q.done          # --collapse: physics earthquake first, then launch
            stepped = False
            if k % sub == 0 and not live.paused and not quaking:
                stepped = True
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
            if (Q is None and f.t < 3.0) or b.shaking(f.t):
                shake = 0.06
            live.r.banner = Q.banner() if quaking else None
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
            overlays(scen, pos3, Q.dust_points(f.t) if Q is not None else None)
            if a.look and a.flow:
                import look
                look.flow_update(k / FPS, scen, Q, W3)
            # the confirmer camera: looks along the drone's heading, slightly down
            focus = next((d for d in f.alive("micro") if d.role == "confirmer" and d.state.startswith("HOVER")), None) \
                or next((d for d in f.alive("micro") if d.role == "confirmer"), None)
            if focus is not None:
                i = f.drones.index(focus)
                eye = pos3[i] + np.array([0, 0, 0.05])
                tgt = eye + np.array([math.cos(focus.heading), -math.sin(focus.heading), -0.25])
                P.XformPrim("/World/Cams/drone").set_world_poses(positions=[eye], orientations=[look_at_quat(eye, tgt)])
            # viewport + recording camera, re-posed every frame (docs/ISAAC6_API.md, open issue 4)
            if a.look:
                import look
                eye, tgt = look.camera(k / FPS, scen, cfg, W3)
            else:
                eye, tgt = view_pose(k / FPS, b, cfg)
            if shake:
                eye = list(np.array(eye) + np.random.normal(0, shake * 2, 3))
            if quaking:                      # the ground (and so the building and the camera crew) moves
                eye = list(np.array(eye) + Q.camera_offset(Q.t))
            q = look_at_quat(eye, tgt)
            try:
                P.XformPrim("/OmniverseKit_Persp", reset_xform_op_properties=(k == 0)).set_world_poses(positions=[eye], orientations=[q])
            except Exception as e:          # headless runs may have no Kit viewport camera
                if k == 0:
                    print(f"[skysearch] no viewport camera ({type(e).__name__}); recording camera only", flush=True)
            if rec is not None:
                P.XformPrim("/World/Cams/rec").set_world_poses(positions=[eye], orientations=[q])
            for rc in EXTRA_CAMS:
                e2, t2 = rig_pose(rc, scen, pos3, Q)
                P.XformPrim(rc["path"]).set_world_poses(positions=[e2], orientations=[look_at_quat(e2, t2)])
        if quaking:
            Q.pre_step(1.0 / FPS)
        sim.step()
        if quaking:
            Q.post_step(1.0 / FPS)
        k += 1
        rgb = None
        if focus is not None:
            dcam.update(dt=sim.get_physics_dt())
            rgb = dcam.data.output["rgb"][0].cpu().numpy()[..., :3]
        if rec_dir and (stepped or (quaking and k % a.quake_every == 0)):
            import matplotlib.pyplot as plt
            rec.update(dt=sim.get_physics_dt())
            img = rec.data.output["rgb"][0].cpu().numpy()[..., :3]
            plt.imsave(os.path.join(rec_dir, "isaac", f"f{n_rec:05d}.png"), img)
            _now = time.time()
            if n_rec % 10 == 0:
                print(f"[skysearch] rec frame {n_rec}: {(_now - getattr(main, '_t10', _now)) / 10:.2f} s per recorded frame "
                      f"({1 + len(EXTRA_CAMS)} cameras, {a.rtx})", flush=True)
                main._t10 = _now
            live.r.rgb_img = rgb
            live.r.save(os.path.join(rec_dir, "dash", f"f{n_rec:05d}.png"), dpi=67)
            for rc in EXTRA_CAMS:
                rc["cam"].update(dt=sim.get_physics_dt())
                plt.imsave(os.path.join(rec_dir, f"isaac_{rc['tag']}", f"f{n_rec:05d}.png"),
                           rc["cam"].data.output["rgb"][0].cpu().numpy()[..., :3])
            n_rec += 1
            if a.stop_t and scen.fleet.t >= a.stop_t:
                print(f"[skysearch] --stop_t: recorded {n_rec} frames to {rec_dir}", flush=True)
                break
            if a.probe_frames and n_tick >= a.probe_frames:
                print(f"[skysearch] probe: isaac frame mean={img.mean() * (255 if img.max() <= 1.0 else 1):.1f} "
                      f"p5={np.percentile(img, 5):.3f} p95={np.percentile(img, 95):.3f}", flush=True)
                break
            if scen.fleet.handoff_t is not None and scen.fleet.t > scen.fleet.handoff_t + 4:
                print(f"[skysearch] recorded {n_rec} frames ({n_tick} mission ticks) to {rec_dir}. Make the video with:\n"
                      f"  ffmpeg -framerate 6 -i {rec_dir}/isaac/f%05d.png -framerate 6 -i {rec_dir}/dash/f%05d.png "
                      f"-filter_complex \"[0:v]scale=1280:720[a];[1:v]scale=1280:720[b];[a][b]hstack\" -c:v libx264 -pix_fmt yuv420p {rec_dir}/isaac_story.mp4",
                      flush=True)
                break
        elif time.time() - last_pub > 1.0:
            live.publish_async({"rgb": rgb, "banner": live.r.banner}); last_pub = time.time()
            if n_tick % 20 == 0 and n_tick != getattr(main, "_printed", -1):
                main._printed = n_tick
                print(f"[skysearch] t={scen.fleet.t:.0f}s found={len(scen.fleet.found)} "
                      f"captions={scen.fleet.captions[-1][1] if scen.fleet.captions else ''}", flush=True)
    simulation_app.close()


if __name__ == "__main__":
    main()
