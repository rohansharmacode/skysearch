"""Run the SkySearch swarm inside NVIDIA Isaac Sim (target: Isaac Sim 5.x on an NVIDIA Brev L40S).

STATUS: written against the Isaac Sim 5.x Python API but NOT yet run on a GPU. Expect to
fix import paths in the first hour on Brev; each Isaac call is isolated in a small helper.

What it does
  * Builds the same forest the CPU sim uses (same seed => same trees, people, deer).
  * Spawns N quadcopters (visual prims moved kinematically by the swarm brain).
  * Each drone carries a downward camera. Its semantic-segmentation output is our
    THERMAL PROXY: pixels labelled "person" or "animal" (warm bodies) become hot blobs,
    with the same canopy/altitude detection model as the CPU sim. Labelled as simulated.
  * The decision code is exactly skysearch/swarm.py; only the sensors change.
  * Debug-draw overlays in the viewport: sensor footprints, targets, candidates.
  * The mission-control dashboard (skysearch/live.py) runs alongside on port 8080 with
    the unscripted-event buttons.

Run on the NVIDIA Brev Isaac launchable (VS Code terminal, see docs/BREV_SETUP.md):
    cd /workspace/isaaclab && ./isaaclab.sh -p ~/skysearch/isaac/isaac_bridge.py --livestream 2 --viz kit
Run on a plain Isaac Sim install:
    ./python.sh skysearch/isaac/isaac_bridge.py --livestream
"""
import argparse, math, os, sys, time
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=7)
ap.add_argument("--drones", type=int, default=5)
ap.add_argument("--port", type=int, default=8080)
ap.add_argument("--cam_res", type=int, default=128)
ap.add_argument("--tree_stride", type=int, default=2, help="use every Nth tree (raise if the stage is slow)")

# Prefer Isaac Lab's AppLauncher (used by the NVIDIA Brev Isaac launchable: pass --livestream 2 --viz kit).
# Fall back to plain Isaac Sim SimulationApp (--livestream flag enables the WebRTC extension).
try:
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(ap)
    a, _ = ap.parse_known_args()
    simulation_app = AppLauncher(a).app
except ImportError:
    ap.add_argument("--livestream", action="store_true")
    a, _ = ap.parse_known_args()
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": a.livestream, "width": 1920, "height": 1080,
                                    "renderer": "RaytracedLighting"})
    if a.livestream:
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("omni.kit.livestream.webrtc")

import numpy as np  # noqa: E402
from isaacsim.core.api import World as SimWorld  # noqa: E402
from isaacsim.core.api.objects import GroundPlane, FixedCylinder, VisualCuboid, VisualCapsule, VisualCylinder  # noqa: E402
from isaacsim.core.prims import SingleXFormPrim  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
import isaacsim.core.utils.numpy.rotations as rot_utils  # noqa: E402
try:
    from isaacsim.core.utils.semantics import add_labels as _label
    def label(prim, cls): _label(prim, labels=[cls], instance_name="class")
except ImportError:  # older API name
    from isaacsim.core.utils.semantics import add_update_semantics as _label
    def label(prim, cls): _label(prim, cls)
try:
    from isaacsim.util.debug_draw import _debug_draw
    draw = _debug_draw.acquire_debug_draw_interface()
except Exception:
    draw = None
from pxr import Gf  # noqa: E402
import omni.usd  # noqa: E402

from skysearch.config import Config  # noqa: E402
from skysearch.world import World  # noqa: E402
from skysearch.scenario import Scenario  # noqa: E402
from skysearch.render import Renderer  # noqa: E402
from skysearch.live import LiveServer  # noqa: E402


class IsaacWorld(World):
    """Ground truth still evolves in Python (so people/deer/fire behave identically),
    but THERMAL perception comes from each drone's Isaac camera."""
    cams = {}

    def thermal(self, pos3):
        cam = self.cams.get(tuple(np.round(pos3[:2], 3)))
        if cam is None:
            return super().thermal(pos3)
        frame = cam.get_current_frame()
        seg = frame.get("semantic_segmentation")
        if seg is None or seg.get("data") is None:
            return []
        data, info = seg["data"], seg["info"].get("idToLabels", {})
        hot_ids = [int(k) for k, v in info.items() if any(c in str(v) for c in ("person", "animal"))]
        if not hot_ids:
            return []
        mask = np.isin(data, hot_ids)
        if not mask.any():
            return []
        from scipy.ndimage import label as cc
        lab, n = cc(mask)
        out = []
        x, y, z = pos3
        r = self.footprint_radius(z)
        res = mask.shape[0]
        alt_f = math.exp(-max(z - 20, 0) / 140)
        for i in range(1, n + 1):
            ys, xs = np.nonzero(lab == i)
            # pixel -> ground (camera looks straight down; image x = world +x, image y = world -y)
            gx = x + (xs.mean() / res - 0.5) * 2 * r
            gy = y - (ys.mean() / res - 0.5) * 2 * r
            can = self.canopy[self.cell_of((gx, gy))]
            if self.rng.random() < 0.6 * (1 - 0.9 * can) * alt_f * 1.5:   # canopy occlusion model
                out.append((gx, gy, float(len(xs)) ** 0.5))
        return out


def build_stage(w: World, cfg: Config, n_drones):
    sim = SimWorld(stage_units_in_meters=1.0, physics_dt=cfg.dt, rendering_dt=cfg.dt)
    GroundPlane("/World/Ground", size=cfg.area * 1.2, color=np.array([0.18, 0.25, 0.12]))
    # trees: trunk + canopy blob. 2500 prims is fine on an L40S; reduce if slow.
    for i, (p, r) in enumerate(zip(w.trees[::a.tree_stride], w.tree_r[::a.tree_stride])):
        h = cfg.tree_height * (0.8 + 0.4 * ((i * 7919) % 100) / 100)
        FixedCylinder(f"/World/Trees/t{i}", position=np.array([p[0], p[1], h / 2]), radius=0.25 * r,
                      height=h, color=np.array([0.35, 0.25, 0.15]))
        VisualCylinder(f"/World/Trees/c{i}", position=np.array([p[0], p[1], h * 0.85]), radius=2.2 * r,
                       height=h * 0.4, color=np.array([0.12, 0.35, 0.12]))
    people, deer = [], []
    for i, pp in enumerate(w.people):
        prim = VisualCapsule(f"/World/People/p{i}", position=np.array([*pp["pos"], 0.3]), radius=0.3, height=1.2,
                             color=np.array([0.9, 0.3, 0.2]), orientation=rot_utils.euler_angles_to_quats(np.array([90, 0, 0]), degrees=True))
        label(prim.prim, "person"); people.append(prim)
    for i, dd in enumerate(w.deer):
        prim = VisualCapsule(f"/World/Deer/d{i}", position=np.array([*dd["pos"], 0.8]), radius=0.35, height=1.0,
                             color=np.array([0.55, 0.4, 0.25]), orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True))
        label(prim.prim, "animal"); deer.append(prim)
    drones, cams = [], []
    for i in range(n_drones):
        root = f"/World/Drones/d{i}"
        body = VisualCuboid(root + "/body", position=np.array([w.base[0] + 12 * i, w.base[1], 0.2]),
                            scale=np.array([0.6, 0.6, 0.15]), color=np.array([0.2, 0.6, 0.9]))
        cam = Camera(prim_path=root + "/body/cam", resolution=(a.cam_res, a.cam_res), frequency=int(1 / cfg.dt),
                     orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True))
        drones.append(body); cams.append(cam)
    sim.reset()
    for cam in cams:
        cam.initialize()
        cam.set_horizontal_aperture(2 * cam.get_focal_length() * math.tan(math.radians(cfg.fov_deg / 2)))
        cam.add_semantic_segmentation_to_frame()
    return sim, people, deer, drones, cams


def overlays(scen):
    if draw is None:
        return
    draw.clear_lines(); draw.clear_points()
    sw, w = scen.swarm, scen.world
    starts, ends, cols, sizes = [], [], [], []
    for d in sw.alive():
        r = w.footprint_radius(d.pos[2])
        for k in range(24):
            a0, a1 = 2 * math.pi * k / 24, 2 * math.pi * (k + 1) / 24
            starts.append((d.pos[0] + r * math.cos(a0), d.pos[1] + r * math.sin(a0), 0.5))
            ends.append((d.pos[0] + r * math.cos(a1), d.pos[1] + r * math.sin(a1), 0.5))
            cols.append((0.3, 0.8, 1.0, 0.8)); sizes.append(2)
        if d.cand is not None:
            starts.append(tuple(d.pos)); ends.append((*d.cand["xy"], 0.5)); cols.append((1, 0.85, 0.2, 1)); sizes.append(3)
    if starts:
        draw.draw_lines(starts, ends, cols, sizes)
    pts = [(*c["xy"], 1.0) for c in sw.candidates]
    pc = [{"open": (1, .85, .2, 1), "confirmed": (0, 1, .45, 1), "rejected": (.6, .6, .6, 1)}.get(c["status"], (1, .5, .3, 1)) for c in sw.candidates]
    if pts:
        draw.draw_points(pts, pc, [18] * len(pts))


def main():
    cfg = Config(seed=a.seed, n_drones=a.drones, duration=1e9)
    import skysearch.scenario as sc
    sc.World = IsaacWorld
    scen = Scenario(cfg, random_events=False)
    w = scen.world
    sim, people, deer, drones, cams = build_stage(w, cfg, cfg.n_drones)
    live = LiveServer(scen, Renderer(scen), a.port); live.start()
    last = 0
    while simulation_app.is_running():
        live.apply_injections()
        if not live.paused:
            IsaacWorld.cams = {tuple(np.round(d.pos[:2], 3)): cams[d.id] for d in scen.swarm.drones}
            scen.step()
            for d, prim in zip(scen.swarm.drones, drones):
                prim.set_world_pose(position=np.array(d.pos) + np.array([0, 0, 0.2]))
            for p, prim in zip(w.people, people):
                prim.set_world_pose(position=np.array([*p["pos"], 0.3]))
            for dd, prim in zip(w.deer, deer):
                prim.set_world_pose(position=np.array([*dd["pos"], 0.8]))
            overlays(scen)
        sim.step(render=True)
        if time.time() - last > 0.5:
            live.publish(); last = time.time()
    simulation_app.close()


if __name__ == "__main__":
    main()
