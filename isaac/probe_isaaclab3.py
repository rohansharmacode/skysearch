"""Probe the Isaac Lab 3.0 route on Isaac Sim 6.0.1: SimulationContext stepping, spawner shapes
(visual / fixed collider / dynamic rigid), semantic tags, a downward Camera sensor with semantic
segmentation, PhysX raycasts and debug draw. Prints PROBE PASS/FAIL lines.

    cd /workspace/isaaclab && ./isaaclab.sh -p /workspace/skysearch/isaac/probe_isaaclab3.py --headless --enable_cameras
"""
import argparse, traceback

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--fabric", type=int, default=0)
ap.add_argument("--fabric_render", type=int, default=0)
AppLauncher.add_app_launcher_args(ap)
a, _ = ap.parse_known_args()
app = AppLauncher(a).app

import numpy as np  # noqa: E402
import torch  # noqa: E402
import omni.kit.app  # noqa: E402
import carb  # noqa: E402
if a.fabric_render == 0:
    carb.settings.get_settings().set_bool("/rtx/hydra/readTransformsFromFabricInRenderDelegate", False)
print("PROBE CFG readTransformsFromFabricInRenderDelegate",
      carb.settings.get_settings().get("/rtx/hydra/readTransformsFromFabricInRenderDelegate"), flush=True)
import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.sensors.camera import Camera, CameraCfg  # noqa: E402

for _ext in ("isaacsim.core.experimental.prims", "isaacsim.core.experimental.objects", "isaacsim.core.experimental.materials", "isaacsim.util.debug_draw"):
    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(_ext, True)
import isaacsim.core.experimental.prims as P  # noqa: E402

R = {}
CAM = {}


def probe(name, fn):
    try:
        out = fn()
        R[name] = "PASS"
        print(f"PROBE PASS {name}: {'' if out is None else out}", flush=True)
    except Exception as e:  # noqa: BLE001
        R[name] = "FAIL"
        print(f"PROBE FAIL {name}: {type(e).__name__}: {e}", flush=True)
        traceback.print_exc()


sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 30, device=a.device, use_fabric=bool(a.fabric),
                                                          enable_scene_query_support=True))
print('PROBE CFG use_fabric', bool(a.fabric), flush=True)


def scene():
    sim_utils.GroundPlaneCfg().func("/World/Ground", sim_utils.GroundPlaneCfg())
    sim_utils.DistantLightCfg(intensity=3000.0).func("/World/Light", sim_utils.DistantLightCfg(intensity=3000.0))
    red = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.3, 0.2))
    # visual only (no physics)
    sim_utils.CylinderCfg(radius=0.5, height=6.0, visual_material=red).func("/World/Vis/trunk", sim_utils.CylinderCfg(radius=0.5, height=6.0, visual_material=red), translation=(5, 0, 3))
    # fixed collider: collision_props without rigid_props
    wall = sim_utils.CuboidCfg(size=(10, 0.3, 4), collision_props=sim_utils.CollisionPropertiesCfg(),
                               visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.5, 0.5)))
    wall.func("/World/Fixed/wall", wall, translation=(0, 8, 2))
    # dynamic rigid body
    box = sim_utils.CuboidCfg(size=(0.5, 0.5, 0.5), rigid_props=sim_utils.RigidBodyPropertiesCfg(),
                              mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
                              collision_props=sim_utils.CollisionPropertiesCfg(),
                              visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.6, 0.9)))
    from isaaclab.assets import RigidObject, RigidObjectCfg
    CAM["box"] = RigidObject(RigidObjectCfg(prim_path="/World/Dyn/box", spawn=box,
                                            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 3.0))))
    # semantic tags on a "person" capsule
    person = sim_utils.CapsuleCfg(radius=0.3, height=1.2, axis="X", visual_material=red,
                                  semantic_tags=[("class", "person")])
    person.func("/World/People/p0", person, translation=(0, 0, 0.3))


probe("spawn_shapes_and_semantic_tags", scene)



def make_cam():
    cfg = CameraCfg(prim_path="/World/Cams/c0", update_period=0, height=128, width=128,
                    data_types=["rgb", "semantic_segmentation", "distance_to_image_plane"],
                    spawn=sim_utils.PinholeCameraCfg(focal_length=18.0, horizontal_aperture=20.955,
                                                     clipping_range=(0.1, 1.0e4)))
    CAM["c"] = Camera(cfg=cfg)


probe("camera_sensor_create", make_cam)
probe("sim_reset", sim.reset)


def pose_cam():
    c = CAM["c"]
    eye = torch.tensor([[0.0, 0.0, 20.0]], device=sim.device)
    tgt = torch.tensor([[0.001, 0.0, 0.0]], device=sim.device)   # tiny x offset: straight-down look-at is degenerate
    c.set_world_poses_from_view(eye, tgt)


probe("camera_look_down", pose_cam)


def step():
    for _ in range(30):
        sim.step()
    b = CAM["box"]
    b.update(sim.get_physics_dt())
    z = float(b.data.root_pos_w[0, 2])
    assert z < 1.0, f"box did not fall: z={z:.2f}"
    return f"RigidObject.data.root_pos_w z after 1 s = {z:.2f} (spawned at 3.0)"


probe("physics_step_and_rigid_fall", step)


def seg():
    c = CAM["c"]
    c.update(dt=sim.get_physics_dt())
    s = c.data.output["semantic_segmentation"]
    info = c.data.info["semantic_segmentation"] if isinstance(c.data.info, dict) else c.data.info[0]["semantic_segmentation"]
    arr = s[0].cpu().numpy()
    rgb = c.data.output["rgb"][0].float().cpu().numpy()
    import matplotlib.pyplot as plt
    for _ in range(60):
        sim.step()
    c.update(dt=sim.get_physics_dt())
    rgb2 = c.data.output["rgb"][0].cpu().numpy()
    plt.imsave("/workspace/skysearch/out/probe_rgb.png", rgb2[..., :3])
    print(f"PROBE RGB mean first={rgb.mean():.1f} after 60 more steps={rgb2.mean():.1f}", flush=True)
    return f"shape={tuple(arr.shape)} dtype={arr.dtype} info={info}"


probe("camera_semantic_segmentation", seg)


def move_xform():
    x = P.XformPrim("/World/People/p0")
    x.set_world_poses(positions=[[2.0, 1.0, 0.3]])
    sim.step()
    return np.asarray(x.get_world_poses()[0].numpy()).round(2).tolist()


probe("xform_set_world_poses_during_sim", move_xform)


def person_px():
    c = CAM["c"]
    for _ in range(3):
        sim.step()
    c.update(dt=sim.get_physics_dt())
    arr = c.data.output["semantic_segmentation"][0].cpu().numpy().reshape(-1, 4)
    info = c.data.info["semantic_segmentation"]["idToLabels"] if isinstance(c.data.info, dict) else c.data.info[0]["semantic_segmentation"]["idToLabels"]
    keys = [eval(k) for k, v in info.items() if v.get("class") == "person"]
    return int(sum((arr == np.array(k, dtype=arr.dtype)).all(1).sum() for k in keys))


def seg_tracks_moves():
    x = P.XformPrim("/World/People/p0")
    x.set_world_poses(positions=[[0.0, 0.0, 0.3]])
    under = person_px()
    x.set_world_poses(positions=[[200.0, 0.0, 0.3]])
    away = person_px()
    return f"person pixels under camera={under}, after moving 200 m away={away} (want >0 then 0)"


probe("camera_sees_kinematic_moves", seg_tracks_moves)


def ray():
    from omni.physx import get_physx_scene_query_interface
    q = get_physx_scene_query_interface()
    h1 = q.raycast_closest((0.0, 0.0, 20.0), (0.0, 1.0, 0.0), 50.0)        # misses (ray at z=20 over the wall)
    h2 = q.raycast_closest((0.0, 0.0, 2.0), (0.0, 1.0, 0.0), 50.0)         # hits the wall at y = 8 - 0.15
    return {"over_wall": h1["hit"], "into_wall": h2["hit"], "dist": round(h2.get("distance", -1), 2),
            "collider": h2.get("collision")}


probe("physx_raycast_closest", ray)


def dd():
    from isaacsim.util.debug_draw import _debug_draw
    d = _debug_draw.acquire_debug_draw_interface()
    d.draw_lines([(0, 0, 1)], [(5, 5, 1)], [(1, 0, 0, 1)], [3])
    d.draw_points([(1, 1, 1)], [(0, 1, 0, 1)], [12])
    sim.step()


probe("debug_draw", dd)
def rgb_mean():
    c = CAM["c"]
    for _ in range(20):
        sim.step()
    c.update(dt=sim.get_physics_dt())
    return round(float(c.data.output["rgb"][0].float().mean()), 1)


def bisect():
    import isaacsim.core.experimental.objects as O
    from isaacsim.core.experimental.materials import PreviewSurfaceMaterial
    out = {"base": rgb_mean()}
    sim_utils.DomeLightCfg(intensity=1500.0).func("/World/Dome", sim_utils.DomeLightCfg(intensity=1500.0))
    out["+dome"] = rgb_mean()
    big = sim_utils.CuboidCfg(size=(1200, 1200, 0.2), visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.18, 0.25, 0.12)))
    big.func("/World/BigGround", big, translation=(0.0, 0.0, -0.1))
    out["+big_ground"] = rgb_mean()
    n = 50
    cyl = O.Cylinder([f"/World/T/t{i}" for i in range(n)], radii=np.full(n, 0.4), heights=np.full(n, 5.0),
                     positions=np.c_[np.random.uniform(-8, 8, (n, 2)), np.full(n, 2.5)])
    out["+exp_cylinders"] = rgb_mean()
    m = PreviewSurfaceMaterial("/World/Looks/leaf"); m.set_input_values("diffuseColor", [0.12, 0.35, 0.12]); cyl.apply_visual_materials(m)
    out["+exp_material"] = rgb_mean()
    c = CAM["c"]
    for name, eye, tgt in (("eye_z60", [0, 0, 60.0], [0.001, 0, 0]), ("eye_z200", [0, 0, 200.0], [0.001, 0, 0]),
                           ("oblique_800m", [0, -250.0, 550.0], [0, 450.0, 0])):
        c.set_world_poses_from_view(torch.tensor([eye], device=sim.device, dtype=torch.float32),
                                    torch.tensor([tgt], device=sim.device, dtype=torch.float32))
        out[name] = rgb_mean()
    import omni.usd
    from pxr import UsdGeom
    st = omni.usd.get_context().get_stage()
    out["metersPerUnit"] = UsdGeom.GetStageMetersPerUnit(st)
    cams = [p for p in st.Traverse() if p.IsA(UsdGeom.Camera)]
    out["cams"] = {str(p.GetPath()): tuple(UsdGeom.Camera(p).GetClippingRangeAttr().Get()) for p in cams}
    cp = [p for p in cams if str(p.GetPath()).startswith("/World/Cams/c0")][0]
    for z in (60.0, 80.0, 100.0, 200.0):
        c.set_world_poses_from_view(torch.tensor([[0, 0, z]], device=sim.device, dtype=torch.float32),
                                    torch.tensor([[1.0, 0, 0]], device=sim.device, dtype=torch.float32))
        m = rgb_mean()
        dep = c.data.output["distance_to_image_plane"][0].cpu().numpy()
        q = c.data.quat_w_world[0].cpu().numpy().round(3).tolist()
        out[f"z{z:.0f}"] = (m, float(np.nanmin(dep)), float(np.nanmedian(dep)), q, float(c.data.pos_w[0, 2]))
    xp = P.XformPrim("/World/Cams/c0")
    for z in (20.0, 60.0, 200.0, 800.0):
        xp.set_world_poses(positions=[[0.0, 0.0, z]], orientations=[[1.0, 0.0, 0.0, 0.0]])
        m = rgb_mean()
        dep = c.data.output["distance_to_image_plane"][0].cpu().numpy()
        out[f"xform_z{z:.0f}"] = (m, round(float(np.nanmedian(dep)), 1), float(c.data.pos_w[0, 2]),
                                  np.asarray(xp.get_world_poses()[0].numpy()).round(1).tolist())
    for far in ():
        UsdGeom.Camera(cp).GetClippingRangeAttr().Set((0.1, far))
        for z in (100.0, 150.0, 200.0, 800.0):
            c.set_world_poses_from_view(torch.tensor([[0, 0, z]], device=sim.device, dtype=torch.float32),
                                        torch.tensor([[0.001, 0, 0]], device=sim.device, dtype=torch.float32))
            out[f"far{far:.0e}_z{z:.0f}"] = rgb_mean()
    return out


probe("rgb_bisect", bisect)
print("PROBE SUMMARY", R, flush=True)
app.close()
