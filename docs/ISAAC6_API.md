# Isaac Sim 6.0.1 + Isaac Lab 3.0 API notes (verified on the Brev launchable)

Tested on the NVIDIA Brev Isaac launchable (`isaac-launchable-e2055c`, 1x L40S): Isaac Sim
`6.0.1-rc.7`, Isaac Lab `3.0.0`, run inside the `vscode` container with
`cd /workspace/isaaclab && ./isaaclab.sh -p SCRIPT.py ...`.
Every snippet below comes from `isaac/probe_isaac6.py` or `isaac/probe_isaaclab3.py`, both of which print
`PROBE PASS/FAIL` lines; re-run them after any image update.

## TL;DR

* The Isaac Sim **5.x core API is gone**: `isaacsim.core.api` (World, objects), `isaacsim.core.prims`
  (SingleXFormPrim), `isaacsim.core.utils`, `isaacsim.sensors.camera` only exist in `extsDeprecated`,
  are not enabled, and **crash even when force-enabled** (`PhysxManager has no attribute set_backend /
  _get_backend_utils`). Don't use them.
* Use **Isaac Lab 3** for the app, simulation stepping, shape spawning, rigid bodies and cameras, and the
  **`isaacsim.core.experimental.*`** extensions for batched prims, poses and materials.
* Isaac Lab 3 replaces Isaac Sim's `SimulationManager` with its own `PhysxManager`, so
  `isaacsim.core.simulation_manager.SimulationManager.setup_simulation()` fails. Step with
  `isaaclab.sim.SimulationContext` instead.
* `enable_scene_query_support` defaults to **False**. Turn it on, or PhysX raycasts never hit anything.
* **`Camera.set_world_poses_from_view()` does not move the render camera** (and `camera.data.pos_w`
  stays at 0). Move cameras with the experimental `XformPrim` instead (see below).
* Port **8080 inside the container is code-server**. Serve dashboards on another port (we use 8090) and open
  them through `<VS Code URL>/proxy/8090/`.
* Camera sensors need `--enable_cameras` (or `args.enable_cameras = True` before `AppLauncher`).

## App launch and streaming

```python
import argparse
from isaaclab.app import AppLauncher
ap = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(ap)
a, _ = ap.parse_known_args()
a.enable_cameras = True                 # needed for isaaclab.sensors.Camera
simulation_app = AppLauncher(a).app
```
Run: `./isaaclab.sh -p script.py --livestream 2 --viz kit --enable_cameras` (WebRTC, shown at
`<brev URL>/viewer`). For probes use `--headless --enable_cameras`.

Enable the experimental extensions *after* the app exists and *before* importing them:
```python
import omni.kit.app
em = omni.kit.app.get_app().get_extension_manager()
for ext in ("isaacsim.core.experimental.prims", "isaacsim.core.experimental.objects",
            "isaacsim.core.experimental.materials", "isaacsim.util.debug_draw"):
    em.set_extension_enabled_immediate(ext, True)
```

## World and stepping (replaces `isaacsim.core.api.World`)

```python
import isaaclab.sim as sim_utils
sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(
    dt=1 / 30, device="cuda:0",
    enable_scene_query_support=True,    # REQUIRED for raycasts
))
# ... spawn everything, create sensors ...
sim.reset()                             # starts physics; do this after spawning
while simulation_app.is_running():
    sim.step()                          # physics + render
```
`use_fabric` True or False both worked. With True (the default), read rigid-body state through
`RigidObject.data`, not USD.

## Shapes (visual, fixed collider, dynamic rigid body) via Isaac Lab spawners

```python
red = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.3, 0.2))
# visual only
cfg = sim_utils.CylinderCfg(radius=0.5, height=6.0, visual_material=red)
cfg.func("/World/Vis/trunk", cfg, translation=(5, 0, 3))
# fixed collider: collision_props WITHOUT rigid_props (raycasts hit it)
wall = sim_utils.CuboidCfg(size=(10, 0.3, 4), collision_props=sim_utils.CollisionPropertiesCfg())
wall.func("/World/Fixed/wall", wall, translation=(0, 8, 2))
# dynamic rigid body: wrap in RigidObject so you can read its state
from isaaclab.assets import RigidObject, RigidObjectCfg
box = sim_utils.CuboidCfg(size=(0.5, 0.5, 0.5), rigid_props=sim_utils.RigidBodyPropertiesCfg(),
                          mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
                          collision_props=sim_utils.CollisionPropertiesCfg())
rb = RigidObject(RigidObjectCfg(prim_path="/World/Dyn/box", spawn=box,
                                init_state=RigidObjectCfg.InitialStateCfg(pos=(0, 0, 3))))
# after sim.reset() + 30 steps: rb.update(dt); rb.data.root_pos_w[0, 2] -> 0.25 (fell and landed)
```
Also available: `CapsuleCfg(radius, height, axis="X"|"Y"|"Z")`, `SphereCfg`, `ConeCfg`, `DomeLightCfg`,
`DistantLightCfg`, `GroundPlaneCfg`. Reading a rigid body's pose with `XformPrim.get_world_poses()`
returned the stale spawn pose (z = 3.0); use `RigidObject.data`.

### Batched visual shapes (thousands of prims) via `isaacsim.core.experimental.objects`

```python
import isaacsim.core.experimental.objects as O
from isaacsim.core.experimental.materials import PreviewSurfaceMaterial
n = 1000
trunks = O.Cylinder([f"/World/Trees/t{i}" for i in range(n)], radii=r, heights=h, positions=np.c_[xy, h / 2])
m = PreviewSurfaceMaterial("/World/Looks/bark"); m.set_input_values("diffuseColor", [0.35, 0.25, 0.15])
trunks.apply_visual_materials(m)          # one shared material for all of them
```
Signatures: `O.Cube(paths, *, sizes, colors, positions, translations, orientations, scales)`,
`O.Cylinder/O.Capsule(paths, *, radii, heights, axes, ...)`, `O.Sphere(paths, *, radii, ...)`,
`O.GroundPlane(paths, *, sizes, colors, templates, ...)`. Make one a static collider with
`isaacsim.core.experimental.prims.GeomPrim(path, apply_collision_apis=True)`.
(`isaacsim.core.experimental.prims.RigidPrim(..., masses=...)` fails under Isaac Lab:
`PhysxManager has no attribute _physics_sim_view__warp`. Use Isaac Lab's `RigidObject`.)

## Moving prims (replaces `SingleXFormPrim.set_world_pose`)

```python
import isaacsim.core.experimental.prims as P
drones = P.XformPrim([f"/World/Drones/d{i}" for i in range(5)])          # batched view
drones.set_world_poses(positions=np.array([[x, y, z], ...]))              # N x 3
drones.set_world_poses(positions=..., orientations=np.array([[w, x, y, z], ...]))   # quats are wxyz
pos, quat = drones.get_world_poses()                                       # warp arrays -> .numpy()
```
Kinematic moves made this way show up in camera renders on the next `sim.step()` (verified: person pixels
present under the camera, then 0 after moving the person 200 m away). For prims Kit created (e.g.
`/OmniverseKit_Persp`) pass `reset_xform_op_properties=True` the first time, or it asserts
`Undefined 'xformOp:orient'`.

## Cameras with semantic segmentation (thermal proxy)

```python
from isaaclab.sensors.camera import Camera, CameraCfg
cam = Camera(CameraCfg(prim_path="/World/Cams/c0", update_period=0, height=128, width=128,
                       data_types=["rgb", "semantic_segmentation", "distance_to_image_plane"],
                       spawn=sim_utils.PinholeCameraCfg(focal_length=18.0, horizontal_aperture=20.78,
                                                        clipping_range=(0.1, 1.0e4))))
# create before sim.reset()
# point it: identity orientation = a USD camera looking straight DOWN (-Z)
P.XformPrim("/World/Cams/c0").set_world_poses(positions=[[x, y, 60.0]], orientations=[[1, 0, 0, 0]])
sim.step(); cam.update(dt=sim.get_physics_dt())
seg = cam.data.output["semantic_segmentation"][0]      # H x W x 4 uint8, colorized RGBA by default
info = cam.data.info["semantic_segmentation"]["idToLabels"]
# {'(0, 0, 0, 0)': {'class': 'BACKGROUND'}, '(0, 0, 0, 255)': {'class': 'UNLABELLED'},
#  '(33, 243, 3, 255)': {'class': 'person'}}   -> keys are colour strings; match pixels against them
```
For the straight-down camera, image +u (column) = world +x and image +v (row) = world −y (measured).
`isaac_bridge.py` calibrates this at startup with a test target, so it doesn't depend on the convention.
Horizontal aperture for a target FOV: `2 * focal * tan(fov / 2)`.

**Broken:** `cam.set_world_poses_from_view(eyes, targets)` leaves the camera where it was (depth `inf`,
`cam.data.pos_w` = 0). Oblique look-at: compute the quaternion yourself:
```python
def look_at_quat(eye, target, up=(0, 0, 1)):           # wxyz for a USD camera
    f = np.subtract(target, eye) / np.linalg.norm(np.subtract(target, eye)); zc = -f
    x = np.cross(up, zc); x /= np.linalg.norm(x); y = np.cross(zc, x)
    qx, qy, qz, qw = scipy.spatial.transform.Rotation.from_matrix(np.c_[x, y, zc]).as_quat()
    return [qw, qx, qy, qz]
```

## Semantic labels

Isaac Lab spawners: pass `semantic_tags=[("class", "person")]` to any shape cfg (verified in camera
segmentation). On an existing prim:
```python
from isaacsim.core.experimental.utils.semantics import add_labels, get_labels
add_labels("/World/People/p0", labels=["person"], taxonomy="class")
get_labels("/World/People/p0")   # {'class': ['person']}
```
(`add_labels` was verified to write the label; camera segmentation was verified with `semantic_tags`.)

## Debug draw (lines and points in the viewport)

```python
from isaacsim.util.debug_draw import _debug_draw          # enable "isaacsim.util.debug_draw" first
d = _debug_draw.acquire_debug_draw_interface()
d.draw_lines([(0, 0, 1)], [(5, 5, 1)], [(1, 0, 0, 1)], [3])   # starts, ends, rgba, widths
d.draw_points([(1, 1, 1)], [(0, 1, 0, 1)], [12])              # points, rgba, sizes
d.clear_lines(); d.clear_points()
```

## PhysX scene-query raycasts

```python
from omni.physx import get_physx_scene_query_interface
hit = get_physx_scene_query_interface().raycast_closest((0, 0, 2), (0, 1, 0), 50.0)
# {'hit': True, 'distance': 7.85, 'collision': '/World/Fixed/wall/geometry/mesh', 'position': ..., 'normal': ...}
```
Unchanged from 5.x, but it only works **while simulating** (after `sim.reset()`) and only with
`SimulationCfg(enable_scene_query_support=True)`; otherwise it silently returns `{'hit': False}`.
Only prims with collision APIs are hit (spawner `collision_props`, or `GeomPrim(..., apply_collision_apis=True)`).

## Open issue: far overview and Kit viewport render black (not solved yet)

What we know (all measured on the L40S):
* Read back with `XformPrim.get_world_poses()`, the overview camera `/World/Cams/overview` and the viewport camera
  `/OmniverseKit_Persp` are where we put them: eye (500, −250, 550), quat (0.899, 0.437, 0, 0) = pitched 52°
  down towards +y, which is correct for a USD camera looking down −Z. RGB mean is still 0.0 and the viewport
  PNG is uniformly black.
* In the small probe scene, a camera moved with `XformPrim` renders correctly at 20, 60, 200 and 800 m straight
  down (depth = height, mean pixel 100–220). Clipping ranges are fine (persp 0.01–1e7, sensors 0.1–1e4/1e5),
  and `metersPerUnit` = 1.0.
* The drone cameras are re-posed every frame and work. The overview and viewport are posed **once** and fail.
  The probe camera that worked was also posed once, so "posed once" alone does not explain it.

Candidate causes, most likely first, each with the check that settles it:
1. **Livestream viewport only renders for a connected client.** With `--livestream 2` and no browser on
   `/viewer`, the Kit viewport may never draw, so `capture_viewport_to_file` saves black. This would only
   explain the viewport, not the overview sensor. *Check:* open `/viewer`, then trigger a snapshot.
2. **Forest-scale scene, not the camera.** Untested in the probe: 2,500 experimental cylinders plus the 1.2 km
   cuboid ground seen obliquely from 800 m. *Check:* `--diag_view` (below) renders the same pose with trees
   hidden, and straight down at 60/200/550 m inside the real scene.
3. **Lighting/exposure at distance.** The bridge uses Dome 1500 + Distant 2500; the probe used Distant 3000
   + GroundPlaneCfg. *Check:* `--diag_view` repeats the shot with the sun at 20,000.
4. **Something re-applies the camera pose after our single set** (e.g. a sensor offset or a viewport-manager
   reset), with USD read-back still showing our value. *Check:* `--diag_view` re-poses the camera right before
   every shot and reports `depth_median` (`inf` means the render camera sees nothing at all).
   *Workaround if so:* re-set the overview and viewport pose every frame, the way the drone cameras are.
5. Far clip on the viewport camera. Unlikely (it reads 1e7), but `--diag_view` sets it to 1e6 explicitly.

**One-run diagnostic** (about 3 min on the GPU; saves PNGs to `/workspace/skysearch/out/diag/`):
```
cd /workspace/isaaclab && ./isaaclab.sh -p /workspace/skysearch/isaac/isaac_bridge.py --headless --enable_cameras --diag_view
```
Each `[diag]` line shows `rgb_mean`, `depth_median`, `finite_px` and the viewport PNG size. The first case that
turns non-black identifies the cause.
