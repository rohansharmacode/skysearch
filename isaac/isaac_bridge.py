"""Run the SkySearch swarm inside NVIDIA Isaac Sim, streamed from the NVIDIA Brev Isaac launchable.

Target: Isaac Sim 6.0.1 + Isaac Lab 3.0 (what the Brev launchable ships). Verified on a Brev L40S.
The API calls used here are documented with tested snippets in docs/ISAAC6_API.md.

What it does
  * Builds the same forest the CPU sim uses (same seed => same trees, people, deer).
  * Spawns N quadcopters (visual prims moved kinematically by the swarm brain).
  * Each drone carries a downward camera. Its semantic-segmentation output is our
    THERMAL PROXY: pixels labelled "person" or "animal" (warm bodies) become hot blobs,
    with the same canopy/altitude detection model as the CPU sim. Labelled as simulated.
  * The decision code is exactly skysearch/swarm.py; only the sensors change.
  * Debug-draw overlays in the viewport: sensor footprints, targets, candidates.
  * The mission-control dashboard (skysearch/live.py) runs alongside on port 8090 (open <VS Code URL>/proxy/8090/) with
    the unscripted-event buttons.

Run on the NVIDIA Brev Isaac launchable (VS Code terminal, see docs/BREV_SETUP.md):
    cd /workspace/isaaclab && ./isaaclab.sh -p /workspace/skysearch/isaac/isaac_bridge.py --livestream 2 --viz kit --enable_cameras
"""
import argparse, math, os, sys, time
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=7)
ap.add_argument("--drones", type=int, default=5)
ap.add_argument("--port", type=int, default=8090, help="dashboard port (8080 is taken by code-server on the Brev launchable)")
ap.add_argument("--cam_res", type=int, default=128)
ap.add_argument("--tree_stride", type=int, default=2, help="use every Nth tree (raise if the stage is slow)")

from isaaclab.app import AppLauncher  # noqa: E402
AppLauncher.add_app_launcher_args(ap)
a, _ = ap.parse_known_args()
a.enable_cameras = True          # Camera sensors need this, also under --livestream
simulation_app = AppLauncher(a).app

import numpy as np  # noqa: E402
import torch  # noqa: E402
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

from skysearch.config import Config  # noqa: E402
from skysearch.world import World  # noqa: E402
from skysearch.scenario import Scenario  # noqa: E402
from skysearch.render import Renderer  # noqa: E402
from skysearch.live import LiveServer  # noqa: E402

HOT = ("person", "animal")


def seg_hot_mask(cam):
    """Boolean HxW mask of warm-body pixels from an Isaac Lab Camera's semantic segmentation."""
    img = cam.data.output["semantic_segmentation"][0].cpu().numpy()          # H x W x 4 (colorized RGBA)
    info = cam.data.info["semantic_segmentation"] if isinstance(cam.data.info, dict) else cam.data.info[0]["semantic_segmentation"]
    keys = [k for k, v in info["idToLabels"].items() if v.get("class") in HOT]
    mask = np.zeros(img.shape[:2], bool)
    for k in keys:
        mask |= (img == np.array(eval(k), dtype=img.dtype)).all(-1)
    return mask


class IsaacWorld(World):
    """Ground truth still evolves in Python (so people/deer/fire behave identically),
    but THERMAL perception comes from each drone's Isaac camera."""
    frames = []      # [(drone pos3 when rendered, hot mask)] refreshed every render
    M_unit = None    # 2x2: world offset per (u, v) image offset per metre of height (calibrated)

    def thermal(self, pos3):
        if not self.frames or self.M_unit is None:
            return super().thermal(pos3)
        rp, mask = min(self.frames, key=lambda f: np.hypot(*(np.asarray(f[0][:2]) - pos3[:2])))
        if np.hypot(*(np.asarray(rp[:2]) - pos3[:2])) > 30 or not mask.any():
            return []
        from scipy.ndimage import label as cc
        lab, n = cc(mask)
        out = []
        x, y, z = rp
        h, wpx = mask.shape
        alt_f = math.exp(-max(z - 20, 0) / 140)
        for i in range(1, n + 1):
            ys, xs = np.nonzero(lab == i)
            uv = np.array([xs.mean() / wpx - 0.5, ys.mean() / h - 0.5])
            gx, gy = np.array([x, y]) + self.M_unit @ uv * z
            can = self.canopy[self.cell_of((gx, gy))]
            if self.rng.random() < 0.6 * (1 - 0.9 * can) * alt_f * 1.5:   # canopy occlusion model
                out.append((gx, gy, float(len(xs)) ** 0.5))
        return out


def spawn(cfg, path, pos):
    cfg.func(path, cfg, translation=tuple(float(v) for v in pos))
    return path


def build_stage(w: World, cfg: Config, n_drones):
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 30, device=a.device))
    sim_utils.DomeLightCfg(intensity=1500.0).func("/World/Dome", sim_utils.DomeLightCfg(intensity=1500.0))
    sim_utils.DistantLightCfg(intensity=2500.0).func("/World/Sun", sim_utils.DistantLightCfg(intensity=2500.0))
    L = cfg.area * 1.2
    ground = sim_utils.CuboidCfg(size=(L, L, 0.2), visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.18, 0.25, 0.12)))
    spawn(ground, "/World/Ground", (cfg.area / 2, cfg.area / 2, -0.1))
    # trees: trunk + canopy, created in two batched calls with one shared material each
    tp, tr = w.trees[::a.tree_stride], w.tree_r[::a.tree_stride]
    hs = np.array([cfg.tree_height * (0.8 + 0.4 * ((i * 7919) % 100) / 100) for i in range(len(tp))])
    trunks = O.Cylinder([f"/World/Trees/t{i}" for i in range(len(tp))], radii=0.25 * tr, heights=hs,
                        positions=np.c_[tp, hs / 2])
    crowns = O.Cylinder([f"/World/Trees/c{i}" for i in range(len(tp))], radii=2.2 * tr, heights=hs * 0.4,
                        positions=np.c_[tp, hs * 0.85])
    for prims, path, col in ((trunks, "/World/Looks/bark", (0.35, 0.25, 0.15)), (crowns, "/World/Looks/leaf", (0.12, 0.35, 0.12))):
        m = PreviewSurfaceMaterial(path); m.set_input_values("diffuseColor", list(col)); prims.apply_visual_materials(m)
    person = sim_utils.CapsuleCfg(radius=0.3, height=1.2, axis="X", semantic_tags=[("class", "person")],
                                  visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.3, 0.2)))
    deer = sim_utils.CapsuleCfg(radius=0.35, height=1.0, axis="X", semantic_tags=[("class", "animal")],
                                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.55, 0.4, 0.25)))
    people = [spawn(person, f"/World/People/p{i}", (*pp["pos"], 0.3)) for i, pp in enumerate(w.people)]
    deers = [spawn(deer, f"/World/Deer/d{i}", (*dd["pos"], 0.8)) for i, dd in enumerate(w.deer)]
    body = sim_utils.CuboidCfg(size=(0.6, 0.6, 0.15), visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.6, 0.9)))
    drones = [spawn(body, f"/World/Drones/d{i}", (w.base[0] + 12 * i, w.base[1], 0.2)) for i in range(n_drones)]
    calib = spawn(person, "/World/Calib", (0.0, 0.0, -500.0))     # parked out of sight; used once below
    focal = 18.0
    aperture = 2 * focal * math.tan(math.radians(cfg.fov_deg / 2))
    cams = [Camera(CameraCfg(prim_path=f"/World/Cams/c{i}", update_period=0, height=a.cam_res, width=a.cam_res,
                             data_types=["semantic_segmentation"],
                             spawn=sim_utils.PinholeCameraCfg(focal_length=focal, horizontal_aperture=aperture,
                                                              clipping_range=(0.1, 1.0e4))))
            for i in range(n_drones)]
    overview = Camera(CameraCfg(prim_path="/World/Cams/overview", update_period=0, height=360, width=640,
                                data_types=["rgb"], spawn=sim_utils.PinholeCameraCfg(focal_length=18.0, clipping_range=(0.1, 1.0e5))))
    sim.reset()
    eye, tgt = [cfg.area / 2, -cfg.area * 0.25, cfg.area * 0.55], [cfg.area / 2, cfg.area * 0.45, 0.0]
    sim.set_camera_view(eye, tgt)
    for path in ("/World/Cams/overview", "/OmniverseKit_Persp"):     # viewport camera = what /viewer streams
        P.XformPrim(path, reset_xform_op_properties=True).set_world_poses(positions=[eye], orientations=[look_at_quat(eye, tgt)])
    build_stage.overview = overview
    return sim, P.XformPrim(people) if people else None, P.XformPrim(deers) if deers else None, \
        P.XformPrim(drones), cams, P.XformPrim(calib)


def look_at_quat(eye, target, up=(0.0, 0.0, 1.0)):
    """wxyz quaternion that points a USD camera (looks down its local -Z, +Y up) from eye at target."""
    from scipy.spatial.transform import Rotation
    f = np.asarray(target, float) - np.asarray(eye, float); f /= np.linalg.norm(f)
    zc = -f
    x = np.cross(up, zc); x /= np.linalg.norm(x)
    y = np.cross(zc, x)
    qx, qy, qz, qw = Rotation.from_matrix(np.c_[x, y, zc]).as_quat()
    return [qw, qx, qy, qz]


def point_cams_down(sim, cams, positions):
    """Identity orientation = a USD camera looking straight down. Isaac Lab's
    Camera.set_world_poses_from_view does not move the render camera on Isaac Sim 6.0.1 /
    Isaac Lab 3.0, so poses are written with the experimental XformPrim instead."""
    paths = [c.cfg.prim_path for c in cams]
    P.XformPrim(paths).set_world_poses(positions=np.asarray(positions, float)[:, :3],
                                       orientations=np.tile([1.0, 0.0, 0.0, 0.0], (len(paths), 1)))


def calibrate(sim, cam, calib):
    """Find the image->world axis mapping by putting a warm target at +x then +y under camera 0."""
    z, d, uvs = 60.0, 10.0, []
    point_cams_down(sim, [cam], [(0.0, 0.0, z)])
    for off in ((d, 0.0), (0.0, d)):
        calib.set_world_poses(positions=[[off[0], off[1], 0.3]])
        for _ in range(4):
            sim.step()
        cam.update(dt=sim.get_physics_dt())
        ys, xs = np.nonzero(seg_hot_mask(cam))
        assert len(xs), "calibration target not visible"
        uvs.append([xs.mean() / cam.cfg.width - 0.5, ys.mean() / cam.cfg.height - 0.5])
    UV = np.array(uvs).T                                    # columns: uv for +x, uv for +y
    M_unit = (np.eye(2) * d) @ np.linalg.inv(UV) / z
    calib.set_world_poses(positions=[[0.0, 0.0, -500.0]])
    print(f"[skysearch] camera calibration uv(+x)={np.round(uvs[0], 3)} uv(+y)={np.round(uvs[1], 3)} M_unit={np.round(M_unit, 3).tolist()}", flush=True)
    return M_unit


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
    sim, people, deer, drones, cams, calib = build_stage(w, cfg, cfg.n_drones)
    IsaacWorld.M_unit = calibrate(sim, cams[0], calib)
    live = LiveServer(scen, Renderer(scen), a.port); live.start()
    last, n_det = 0, 0
    while simulation_app.is_running():
        live.apply_injections()
        if not live.paused:
            scen.step()
            pos = np.array([d.pos for d in scen.swarm.drones], dtype=float)
            drones.set_world_poses(positions=pos + np.array([0, 0, 0.2]))
            if people is not None:
                people.set_world_poses(positions=np.array([[*p["pos"], 0.3] for p in w.people]))
            if deer is not None:
                deer.set_world_poses(positions=np.array([[*d["pos"], 0.8] for d in w.deer]))
            point_cams_down(sim, cams, pos)
            overlays(scen)
        sim.step()
        if not live.paused:
            frames = []
            for d, cam in zip(scen.swarm.drones, cams):
                cam.update(dt=sim.get_physics_dt())
                frames.append((np.array(d.pos, dtype=float), seg_hot_mask(cam)))
            IsaacWorld.frames = frames
            hot = sum(int(m.any()) for _, m in frames)
            if hot:
                n_det += hot
        if time.time() - last > 0.5:
            live.publish(); last = time.time()
            if int(last) % 10 == 0:
                print(f"[skysearch] t={scen.swarm.t:.0f}s cameras-with-warm-pixels so far={n_det} candidates={len(scen.swarm.candidates)}", flush=True)
            if os.path.exists("/tmp/skysearch_snap"):      # touch this file to save what /viewer shows
                os.remove("/tmp/skysearch_snap")
                from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
                capture_viewport_to_file(get_active_viewport(), os.path.join(ROOT, "out", "viewport.png"))
                ov = build_stage.overview
                ov.update(dt=sim.get_physics_dt())
                import matplotlib.pyplot as plt
                img = ov.data.output["rgb"][0].cpu().numpy()
                plt.imsave(os.path.join(ROOT, "out", "overview.png"), img[..., :3])
                pp, qq = P.XformPrim("/World/Cams/overview").get_world_poses()
                vp, vq = P.XformPrim("/OmniverseKit_Persp").get_world_poses()
                print(f"[skysearch] overview rgb mean={img[..., :3].mean():.1f} pose={np.asarray(pp.numpy()).round(1).tolist()} q={np.asarray(qq.numpy()).round(3).tolist()} "
                      f"persp={np.asarray(vp.numpy()).round(1).tolist()} q={np.asarray(vq.numpy()).round(3).tolist()}", flush=True)
                print("[skysearch] saved out/viewport.png", flush=True)
    simulation_app.close()


if __name__ == "__main__":
    main()
