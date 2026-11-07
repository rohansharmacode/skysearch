"""Probe which Isaac Sim 6.0.1 APIs work in this launchable. Prints PASS/FAIL per feature.

    cd /workspace/isaaclab && ./isaaclab.sh -p /workspace/skysearch/isaac/probe_isaac6.py --headless
"""
import argparse, inspect, traceback

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(ap)
a, _ = ap.parse_known_args()
app = AppLauncher(a).app

import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402

for _ext in ("isaacsim.core.experimental.utils", "isaacsim.core.experimental.prims",
             "isaacsim.core.experimental.materials", "isaacsim.core.experimental.objects",
             "isaacsim.core.simulation_manager", "omni.replicator.core", "isaacsim.util.debug_draw"):
    ok = omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(_ext, True)
    print("EXT", _ext, ok, flush=True)

R = {}


def probe(name):
    def deco(fn):
        try:
            out = fn()
            R[name] = "PASS"
            print(f"PROBE PASS {name}: {out if out is not None else ''}", flush=True)
        except Exception as e:  # noqa: BLE001
            R[name] = "FAIL"
            print(f"PROBE FAIL {name}: {type(e).__name__}: {e}", flush=True)
            traceback.print_exc()
        return fn
    return deco


def sig(obj):
    try:
        return str(inspect.signature(obj))
    except Exception:  # noqa: BLE001
        return "?"


import isaacsim.core.experimental.objects as O  # noqa: E402
import isaacsim.core.experimental.prims as P  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402

for nm in ("Cube", "Cylinder", "Capsule", "Sphere", "GroundPlane", "Camera"):
    print("SIG objects." + nm, sig(getattr(O, nm, None)), flush=True)
for nm in ("XformPrim", "RigidPrim", "GeomPrim"):
    print("SIG prims." + nm, sig(getattr(P, nm, None)), flush=True)
print("SIG XformPrim.set_world_poses", sig(P.XformPrim.set_world_poses), flush=True)
print("SIG stage_utils members", [m for m in dir(stage_utils) if not m.startswith("_")], flush=True)


@probe("new_stage")
def _():
    stage_utils.create_new_stage()
    stage_utils.define_prim("/World", "Xform")


@probe("ground_plane")
def _():
    O.GroundPlane("/World/Ground", sizes=200.0)


@probe("visual_cube_cylinder_capsule")
def _():
    O.Cube("/World/Vis/cube", sizes=1.0, positions=[0, 0, 5])
    O.Cylinder("/World/Vis/cyl", radii=0.5, heights=3.0, positions=[3, 0, 1.5])
    O.Capsule("/World/Vis/cap", radii=0.3, heights=1.2, positions=[6, 0, 0.6])


@probe("fixed_collider_geomprim")
def _():
    O.Cube("/World/Fixed/wall", sizes=1.0, positions=[0, 10, 1], scales=[10, 0.3, 2])
    P.GeomPrim("/World/Fixed/wall", apply_collision_apis=True)


@probe("dynamic_rigid_body")
def _():
    O.Cube("/World/Dyn/box", sizes=0.5, positions=[0, 0, 3])
    P.GeomPrim("/World/Dyn/box", apply_collision_apis=True)
    rb = P.RigidPrim("/World/Dyn/box", masses=[1.0])
    return type(rb).__name__


@probe("display_color")
def _():
    from isaacsim.core.experimental.materials import PreviewSurfaceMaterial
    m = PreviewSurfaceMaterial("/World/Looks/red")
    m.set_input_values("diffuseColor", [1.0, 0.1, 0.1])
    O.Cube("/World/Vis/cube").apply_visual_materials(m)


@probe("xform_set_world_pose")
def _():
    x = P.XformPrim("/World/Vis/cube")
    x.set_world_poses(positions=[[1.0, 2.0, 7.0]])
    p, q = x.get_world_poses()
    return np.asarray(p.numpy() if hasattr(p, "numpy") else p).round(2).tolist()


@probe("semantic_labels")
def _():
    from isaacsim.core.experimental.utils.semantics import add_labels, get_labels
    add_labels("/World/Vis/cap", labels=["person"], taxonomy="class")
    return get_labels("/World/Vis/cap")


@probe("physics_step")
def _():
    from isaacsim.core.simulation_manager import SimulationManager
    import omni.timeline
    SimulationManager.setup_simulation(dt=1 / 30)
    omni.timeline.get_timeline_interface().play()
    for _ in range(30):
        app.update()
    p, _ = P.XformPrim("/World/Dyn/box").get_world_poses()
    return f"box z after 1s: {np.asarray(p.numpy() if hasattr(p, 'numpy') else p)[0][2]:.2f} (started 3.0)"


CAM = {}


@probe("camera_semantic_segmentation")
def _():
    import omni.replicator.core as rep
    from pxr import UsdGeom, Gf
    stage = stage_utils.get_current_stage()
    cam = UsdGeom.Camera.Define(stage, "/World/Cam")
    x = UsdGeom.Xformable(cam.GetPrim())
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(6, 0, 20))       # USD cameras look down -Z by default
    cam.CreateFocalLengthAttr(18.0)
    rp = rep.create.render_product("/World/Cam", (128, 128))
    ann = rep.AnnotatorRegistry.get_annotator("semantic_segmentation", init_params={"colorize": False})
    ann.attach(rp)
    for _ in range(10):
        app.update()
    d = ann.get_data()
    data, info = d["data"], d["info"]["idToLabels"]
    CAM.update(ann=ann)
    return f"shape={data.shape} labels={info} person_px={int(np.isin(data, [int(k) for k, v in info.items() if 'person' in str(v)]).sum())}"


@probe("debug_draw")
def _():
    import omni.kit.app
    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.util.debug_draw", True)
    from isaacsim.util.debug_draw import _debug_draw
    d = _debug_draw.acquire_debug_draw_interface()
    d.draw_lines([(0, 0, 1)], [(5, 5, 1)], [(1, 0, 0, 1)], [3])
    d.draw_points([(1, 1, 1)], [(0, 1, 0, 1)], [12])
    d.clear_lines(); d.clear_points()


@probe("physx_raycast")
def _():
    from omni.physx import get_physx_scene_query_interface
    q = get_physx_scene_query_interface()
    hit = q.raycast_closest((0, 0, 50), (0, 0, -1), 100.0)
    return {k: (round(v, 2) if isinstance(v, float) else v) for k, v in hit.items() if k in ("hit", "distance", "collision")}


print("PROBE SUMMARY", R, flush=True)
app.close()
