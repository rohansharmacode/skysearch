# Isaac Sim APIs verified on a GPU (and what our adapter does with them)

Source: the team repo clone `/home/peroxide/Downloads/UCL/skysearch` (read only), branches `isaac6-port` and
`v2-earthquake`. Everything marked **verified** ran on the NVIDIA Brev Isaac launchable (1x L40S, Isaac Sim
`6.0.1-rc.7`, Isaac Lab `3.0.0`, run in the `vscode` container from `/workspace/isaaclab` with
`./isaaclab.sh -p SCRIPT.py`). References are `branch:path:line` (or a commit hash for facts that are only in a
commit message). Read with `git -C ../skysearch show origin/<branch>:<path>`.

Status words: **verified** = ran on the L40S and the team reports it working; **observed** = visible in their
GPU output files; **offline** = only checked with mocks / usd-core; **unverified** = nobody has run it.

## 1. App launch

| Fact | Status | Source | Our code |
|---|---|---|---|
| The launchable ships Isaac Sim 6.0.1 + Isaac Lab 3.0, **not 5.1** as our handoff assumed. | verified | isaac6-port:docs/BREV_SETUP.md:39-40; isaac6-port:docs/ISAAC6_API.md:3-5 | README + Requests |
| Launch through Isaac Lab: `AppLauncher.add_app_launcher_args(ap); a, _ = ap.parse_known_args(); a.enable_cameras = True; app = AppLauncher(a).app`. | verified | isaac6-port:docs/ISAAC6_API.md:27-37; v2-earthquake:isaac/earthquake_scene.py:55-59 | `compat.LabRuntime.boot`, `run_mission.parse_args` (two-phase parse) |
| Run: `cd /workspace/isaaclab && ./isaaclab.sh -p SCRIPT.py --livestream 2 --viz kit --enable_cameras`; probes use `--headless --enable_cameras`. WebRTC view at `<brev URL>/viewer` (one viewer tab only). | verified | isaac6-port:docs/ISAAC6_API.md:38-39; isaac6-port:docs/BREV_SETUP.md:23-29 | README commands |
| Camera sensors need `--enable_cameras` (or `a.enable_cameras = True` before `AppLauncher`), also under `--livestream`. | verified | isaac6-port:docs/ISAAC6_API.md:25; isaac6-port:isaac/isaac_bridge.py:34 | `LabRuntime.boot` forces it |
| Enable extensions **after** the app exists and **before** importing them: `isaacsim.core.experimental.{prims,objects,materials}`, `isaacsim.util.debug_draw` via `get_extension_manager().set_extension_enabled_immediate(ext, True)`. | verified | isaac6-port:docs/ISAAC6_API.md:41-48; v2-earthquake:isaac/earthquake_scene.py:62-65 | `compat.LAB_EXTS`, enabled in `LabRuntime.boot` |
| Port 8080 inside the container is code-server. Serve dashboards on another port and open `<VS Code URL>/proxy/<port>/`. | verified | isaac6-port:docs/ISAAC6_API.md:23-24; isaac6-port:docs/BREV_SETUP.md:30-31 | README (dashboard on 8765 -> `/proxy/8765/`) |
| Extra pip packages go in with `./isaaclab.sh -p -m pip install scipy matplotlib`. | verified | isaac6-port:docs/BREV_SETUP.md:18-21 | README |
| The 5.x core API (`isaacsim.core.api` World/objects, `isaacsim.core.prims.SingleXFormPrim`, `isaacsim.core.utils`, `isaacsim.sensors.camera`) only exists in extsDeprecated and **crashes** under Isaac Lab 3 (`PhysxManager has no attribute set_backend`). | verified | isaac6-port:docs/ISAAC6_API.md:11-14; commit c8b61d8 | we never import it |
| `SimulationApp` route (`from isaacsim import SimulationApp` + `omni.timeline` play + `app.update()`) on 6.x without Isaac Lab. | **unverified** | (not tested by the team) | kept as `KitRuntime` for 4.5/5.x; `--launcher kit` |

## 2. Simulation, stage and physics

| Fact | Status | Source | Our code |
|---|---|---|---|
| Isaac Lab 3 replaces `SimulationManager` with its own `PhysxManager`: `isaacsim.core.simulation_manager.SimulationManager.setup_simulation()` fails. Step with `isaaclab.sim.SimulationContext`. | verified | isaac6-port:docs/ISAAC6_API.md:17-19 | `LabRuntime` |
| `sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1/30, device=a.device, enable_scene_query_support=True))`; spawn everything; `sim.reset()`; loop `sim.step()` (physics + render). | verified | isaac6-port:docs/ISAAC6_API.md:50-62; v2-earthquake:isaac/earthquake_scene.py:218, 300, 530 | `LabRuntime.stage/start_physics/step` |
| `enable_scene_query_support` defaults to **False**; without it raycasts silently return `{'hit': False}`. (The first port, isaac6-port:isaac/isaac_bridge.py:108, still omitted it.) | verified | isaac6-port:docs/ISAAC6_API.md:20, 179-180 | set in `LabRuntime`; Kit route sets `physxScene:enableSceneQuerySupport` on `/World/physicsScene` (`compat.enable_scene_queries`) |
| `use_fabric` True or False both worked. Probe defaults: `use_fabric=False` + carb `/rtx/hydra/readTransformsFromFabricInRenderDelegate = False`, under which kinematic moves showed up in camera renders. | verified | isaac6-port:docs/ISAAC6_API.md:63-64; isaac6-port:isaac/probe_isaaclab3.py:12-25, 48-49, 157-166 | `LabRuntime` uses exactly that (pxr pose edits are the source of truth); `--fabric` to flip |
| Spawn on the launcher's current stage (`omni.usd.get_context().get_stage()`); the team never calls `new_stage()` under Isaac Lab. | verified (by use) | v2-earthquake:isaac/earthquake_scene.py:216-303; v2-earthquake:isaac/look.py:56-57 | Lab route skips `compat.new_stage()` |
| Fixed collider = shape with `collision_props` and no `rigid_props`; or `isaacsim.core.experimental.prims.GeomPrim(paths, apply_collision_apis=True)`. Only prims with collision APIs are hit by raycasts. | verified | isaac6-port:docs/ISAAC6_API.md:73-75, 101-102, 181; v2-earthquake:isaac/earthquake_scene.py:145-150 | our pxr `UsdPhysics.CollisionAPI` on cubes / triangle-mesh terrain is the same schema (offline) |
| Static triangle-mesh colliders written with pxr: `CollisionAPI` + `MeshCollisionAPI.approximation = "none"`; dynamic pieces `"convexHull"`. | offline (team's mocked + usd-core check) | v2-earthquake:isaac/quake.py:105-127; commit b21a6d7 | same pattern in `scene.heightfield_mesh` |
| `isaacsim.core.experimental.prims.RigidPrim(..., masses=...)` fails under Isaac Lab; use `isaaclab.assets.RigidObject` and read state from `RigidObject.data` (USD/XformPrim read-back is stale for rigid bodies). | verified | isaac6-port:docs/ISAAC6_API.md:76-87, 103-104 | n/a (no rigid bodies; drones are kinematic) |
| Stage is Z-up, `metersPerUnit` 1.0. | verified | isaac6-port:docs/ISAAC6_API.md:191-192 | unchanged |

## 3. Poses, cameras and the black viewport

| Fact | Status | Source | Our code |
|---|---|---|---|
| Move prims with the experimental batched `XformPrim`: `P.XformPrim(paths).set_world_poses(positions=Nx3, orientations=Nx4 wxyz)`; changes render on the next `sim.step()`. | verified | isaac6-port:docs/ISAAC6_API.md:106-116 | `compat.PoseWriter` (camera mirrors) |
| For Kit-created prims (`/OmniverseKit_Persp`) pass `reset_xform_op_properties=True` the first time, or it asserts `Undefined 'xformOp:orient'`. | verified | isaac6-port:docs/ISAAC6_API.md:116-118; v2-earthquake:isaac/earthquake_scene.py:519 | `PoseWriter` |
| `Camera.set_world_poses_from_view()` does **not** move the render camera (depth `inf`, `pos_w` 0). Compute the look-at quaternion yourself; identity orientation = USD camera looking straight down (-Z). | verified | isaac6-port:docs/ISAAC6_API.md:21-22, 141-149; isaac6-port:isaac/isaac_bridge.py:161-168 | we never use it; `camera.look_at` + `compat.mat_to_quat_wxyz` |
| **Black viewport / overview**: a far camera and `/OmniverseKit_Persp` posed **once** rendered black (RGB mean 0) on 6.0.1. Candidate causes: livestream viewport only draws with a `/viewer` client connected; lighting at distance; something re-applying the pose. Workaround used in every later GPU run: **re-pose the viewport and recording cameras every frame**. | verified problem; workaround verified by the earthquake recordings | isaac6-port:docs/ISAAC6_API.md:183-216; v2-earthquake:isaac/earthquake_scene.py:507-527; commit 4b1db3b (first full recording) | `DirectorCamera` re-applies its pose every frame and mirrors it to `/OmniverseKit_Persp` and the recording camera |
| Headless runs may have no Kit viewport camera (`/OmniverseKit_Persp` set fails). | verified (guarded in their code) | v2-earthquake:isaac/earthquake_scene.py:518-522 | `PoseWriter` disables itself after one failure; selftest reports it as WARN |
| `get_active_viewport().camera_path = path` (switching the viewport to our camera). | **unverified** (the team moved Persp instead) | n/a | still tried; selftest reads it back |
| `sim.set_camera_view(eye, target)` was called once at start in the bridge; it is part of the black-viewport setup, so don't rely on it. | unverified | isaac6-port:isaac/isaac_bridge.py:142 | not used |
| Recording camera: `Camera(CameraCfg(prim_path, update_period=0, height, width, data_types=["rgb"], spawn=PinholeCameraCfg(focal_length=18.0, clipping_range=(0.1, 1e4))))`, created **before** `sim.reset()`, posed with `XformPrim` every frame, read with `cam.update(dt=sim.get_physics_dt()); cam.data.output["rgb"][0].cpu().numpy()[..., :3]`. Produced the team's `media/isaac_story.mp4` and quake clips. | verified | v2-earthquake:isaac/earthquake_scene.py:266-267, 523-524, 538-545; isaac6-port:docs/ISAAC6_API.md:120-131 | `compat.FrameGrabber` (lab route), `--record`, selftest `render_frame` |
| Brightness probe of a recorded frame (mean, p5, p95) to detect black/washed-out output. | verified | v2-earthquake:isaac/earthquake_scene.py:558-560 | selftest `render_frame` reports `mean_rgb`, retries while < 3 |
| `omni.kit.viewport.utility.capture_viewport_to_file(get_active_viewport(), path)` exists on 6.0.1 but saved black under `--livestream 2` with no client. | verified (black) | isaac6-port:isaac/isaac_bridge.py:195, 209-210, 308-309; isaac6-port:docs/ISAAC6_API.md:197-199 | secondary capture in selftest only |
| Replicator `rep.create.render_product(cam, (w, h))` + `AnnotatorRegistry.get_annotator(...)`. | probed on 6.0.1, result not recorded | isaac6-port:isaac/probe_isaac6.py:129-147 | Kit-route `FrameGrabber` (unverified) |
| Horizontal aperture for a target FOV: `2 * focal * tan(fov/2)`. | verified | isaac6-port:docs/ISAAC6_API.md:139 | unchanged (focal 18, aperture 20.955) |

## 4. Scene queries (LiDAR)

| Fact | Status | Source | Our code |
|---|---|---|---|
| `from omni.physx import get_physx_scene_query_interface; get_physx_scene_query_interface().raycast_closest((0,0,2), (0,1,0), 50.0)` with **plain tuples** -> `{'hit': True, 'distance': 7.85, 'collision': '/World/Fixed/wall/geometry/mesh', 'position': ..., 'normal': ...}`. Unchanged from 5.x. | verified | isaac6-port:docs/ISAAC6_API.md:172-181; isaac6-port:isaac/probe_isaaclab3.py:169-178 | `compat.physx_raycaster` now passes tuples (carb.Float3 only as fallback) and exposes the raw dict for the selftest |
| Works only **while simulating** (after `sim.reset()`) and with scene queries on. | verified | isaac6-port:docs/ISAAC6_API.md:179-180 | raycast check runs after `start_physics()` |
| RTX LiDAR. | **never run** by the team | n/a | `--rtx-lidar` stays optional; PhysX LiDAR is the default |

## 5. Debug draw

| Fact | Status | Source | Our code |
|---|---|---|---|
| `from isaacsim.util.debug_draw import _debug_draw; d = _debug_draw.acquire_debug_draw_interface()`; `d.draw_lines(starts, ends, rgba, widths)`, `d.draw_points(points, rgba, sizes)`, `d.clear_lines()`, `d.clear_points()`; lists of tuples. | verified | isaac6-port:docs/ISAAC6_API.md:162-170; isaac6-port:isaac/probe_isaaclab3.py:181-189; v2-earthquake:isaac/earthquake_scene.py:71-75, 380-425 | `compat.debug_draw`, `overlay.DebugDrawBackend` (signatures already matched) |
| Debug-draw lines appear in Isaac Lab Camera-sensor frames (links/LiDAR lines at the left edge). | observed | v2-earthquake:media/preview/isaac_t3s_light_fixed.png | `--overlay auto` is fine for `--record`; `--overlay usd` still safest for Movie Capture |

## 6. Lighting, materials, assets, rendering

| Fact | Status | Source | Our code |
|---|---|---|---|
| Dome 1200 + overhead sun 3000 rendered every surface near-white at building scale. Dome **350** (cool, `(0.80, 0.85, 1.0)`) + a tilted warm sun **1800** (`angle=1.0`, colour `(1.0, 0.95, 0.85)`) gave colour, shading and shadows. | verified | commit a6f69e6; v2-earthquake:isaac/earthquake_scene.py:227-232 | `scene.build(sun=1800, sky=350, sun_elev=35)`, `--sun/--sky` (we had 2500 / 500) |
| Materials via `PreviewSurfaceCfg(diffuse_color=...)` or `isaacsim.core.experimental.materials.PreviewSurfaceMaterial(...).set_input_values("diffuseColor", rgb)`; every visible prim in their scenes had a material. Bare `displayColor` was never relied on. | verified | isaac6-port:docs/ISAAC6_API.md:66-98; v2-earthquake:isaac/earthquake_scene.py:99-107 | `scene.Materials`: real `UsdPreviewSurface` (pure UsdShade) on every gprim; `--look plain` = displayColor only |
| MDL: `sim_utils.MdlFileCfg(mdl_path=<url>, project_uvw=True, texture_scale=(s, s)).func(path, cfg)`; bind with `UsdShade.MaterialBindingAPI.Apply(p).Bind(mat, UsdShade.Tokens.strongerThanDescendants)`. Concrete_Rough / Concrete_Formed / Concrete_Smooth / Plaster / Carpet / Asphalt used. | verified | v2-earthquake:isaac/look.py:27-37, 60-79; commit 1f62514 | `look.make_mdl` (lab route), `--look pretty` |
| Assets root on 6.0: `isaaclab.utils.assets.ISAAC_NUCLEUS_DIR = .../Assets/Isaac/6.0/Isaac`, `NVIDIA_NUCLEUS_DIR = .../Assets/Isaac/6.0/NVIDIA` (S3). Exist: `Isaac/Robots/IsaacSim/Quadcopter/quadcopter.usd`, `Isaac/Robots/Bitcraze/Crazyflie/cf2x.usd`, `NVIDIA/Materials/Base/{Masonry/Concrete_*, Masonry/Brick_Wall_Brown, Natural/Dirt, Stone/Gravel, Natural/Asphalt}.mdl`, `NVIDIA/Assets/Skies/Clear/evening_road_01_4k.hdr`. | verified (listing) | v2-earthquake:isaac/assets_found.json ("roots", "drones", "materials", "skies"); v2-earthquake:isaac/probe_assets.py:20-26 | `compat.assets_root` reads Isaac Lab first; `drone_viz.ASSET_CANDIDATES`; `look.MDL_FILES`, `look.SKY` |
| HDRI sky: set `inputs:texture:file` (Asset) and `inputs:texture:format = "latlong"` on the dome, intensity ~900. | verified | v2-earthquake:isaac/look.py:159-166 | `look.apply` |
| Referenced assets keep their own units: People and ArchVis are centimetres and came in 100x too big (blacked the frame). Scale every reference by its `metersPerUnit`. | verified | commit 1f62514; v2-earthquake:isaac/look.py:124-149 | `drone_viz._reference_asset` adds a `units` scale op |
| Strip physics from referenced models: `RigidBodyAPI.rigidBodyEnabled=False`, `CollisionAPI.collisionEnabled=False`, remove `ArticulationRootAPI` + `physxArticulation:articulationEnabled=False`, `Joint.jointEnabled=False`. | verified | v2-earthquake:isaac/look.py:82-94 | `drone_viz._reference_asset` (added the articulation attr) |
| Hidden geometry still collides ("visibility is not physics"); visual jagged meshes over box colliders. | claimed in code; `--jagged` defaults off, so likely not GPU-run | v2-earthquake:isaac/look.py:293-370 (comment at 366) | `look.chip_rubble` relies on it; selftest `raycast` runs after the look, so it checks this |
| Path tracing: carb `/rtx/rendermode = "PathTracing"`, `/rtx/pathtracing/spp` and `/totalSpp = N`, `/rtx/pathtracing/optixDenoiser/enabled = True`. 16 spp at 1280x720 = 6.49 s per recorded frame for 3 cameras (real-time mode 2.24 s), after a ~9 min one-off shader compile (vegetation). | verified | v2-earthquake:isaac/earthquake_scene.py:219-226; commit 4bba6f5 | `compat.set_render_mode`, `--rtx pathtracing --spp 16` |
| Omni Flow volumetric dust/fire: enable `omni.flowusd`, carb `/rtx/flow/enabled` and `/rtx/flow/pathTracingEnabled`, reference `<ext path>/data/presets/Fire/Fire.usda`, set per-effect `layer`, emitter `radius`, `fuel 0 / temperature 0.15 / smoke 2.5 / coupleRateSmoke 3` for dust, `advection.buoyancyPerTemp 0.6`, `smoke.fade 0.08 / damping 0.05`, toggle `flowEmitterSphere.enabled`. `omni.blast` is not available. | verified (quake3 clips) | v2-earthquake:isaac/look.py:445-526; v2-earthquake:isaac/assets_found.json ("extensions"); commit 2e5e96c | `look.FlowDust` (dust under drones near the ground) |
| Small `SphereLightCfg` search lights on drones. | verified | v2-earthquake:isaac/earthquake_scene.py:275-285 | not adopted (open-air site) |
| Physics earthquake / irregular rubble (`quake.py`, `rubble.py`): convex-hull rigid bodies, gravity tilt as ground motion. | rubble.py offline only (commit b21a6d7) | v2-earthquake:isaac/quake.py, isaac/rubble.py | not adopted: our rubble is static by design; chipped visual twins instead |

## 7. What is still unverified in our adapter (the selftest checks each)

1. Kit route on any version: `SimulationApp` + timeline + `app.update()` physics, and our `/World/physicsScene` with
   `physxScene:enableSceneQuerySupport` (selftest `physics_parse`, `raycast`).
2. Viewport `camera_path` switching (selftest `camera_switch`). The verified fallback is driving
   `/OmniverseKit_Persp`, which we now do every frame.
3. `UsdPrimvarReader_float3` (per-vertex terrain colour) inside a `UsdPreviewSurface` under RTX: if unsupported the
   terrain renders in the reader's fallback colour (uniform dust), which is acceptable.
4. Kit-route recording via replicator (`FrameGrabber` "replicator"), and `CreateMdlMaterialPrim` for MDL on the Kit
   route.
5. pxr-authored geometry (our scene) under Isaac Lab: the team spawned with Isaac Lab spawners / experimental
   objects, we write the same schemas with pxr. Expected to work (Isaac Lab spawners are pxr underneath), but it is
   exactly what the selftest's `stage_build` + `raycast` + `render_frame` prove.
6. `look.chip_rubble` relying on invisible colliders still being hit.
7. RTX LiDAR (`--rtx-lidar`) on any version.
