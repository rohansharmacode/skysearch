# Isaac Sim adapter (3x3 rubble-grid SAR demo)

Isaac is the world and the body; the brain is `sim/grid_mission.py` (numpy, never sees ground truth).
`isaac/run_mission.py` builds the stage from the same seeded `sim/scenario.py`, poses three visual-only
drones from the mission state every render frame, raycasts a spinning LiDAR against the PhysX colliders
while the drones survey (points go back to the mission via `ingest_pointcloud`, which rebuilds the
landing-search heightmap), and draws pings, range rings, acoustic cones and dispatch arcs from the state.
The mission writes `<out>/state.json`, so the web dashboard shows the Isaac run live.

| File | What |
|---|---|
| `compat.py` | launch routes (`LabRuntime` = AppLauncher + SimulationContext, `KitRuntime` = SimulationApp), debug_draw, viewport + per-frame camera poses, PhysX raycaster, frame grabber + PNG writer, render mode, assets |
| `scene.py` | stage from the scenario (pxr only): terrain mesh, ~620 collidable rubble boxes, staging area, UsdPreviewSurface materials, measured lighting, `--show-truth` ghosts |
| `photoreal.py` | (Isaac teammate, GPU-verified) `--photoreal 1`: MDL materials, irregular rubble pieces as the colliders, debris, dusk HDRI + low sun, haze, vegetation, operator; `--record` multi-camera recorder (`Rigs`, `Recorder`, `--rec-windows`), `--rtx pathtracing` |
| `rubble.py` | (Isaac teammate) numpy/scipy broken-concrete geometry used by `photoreal.py` |
| `look.py` | `--look pretty` shim: forwards to `photoreal.dress`; keeps our Omni Flow dust (`--flow`) |
| `selftest.py` | `--selftest`: PASS/WARN/FAIL per risky call, saves a frame |
| `drone_viz.py` | primitive quadrotor (rotors spin, purple LiDAR puck, orange radar, teal mics, skids) or `--drone-asset` |
| `lidar.py` | `PhysxLidar` (Isaac), `HeightmapLidar` (dry-run), optional `RtxLidar` |
| `overlay.py` | state-driven visuals; debug_draw or USD prims (`--overlay usd` for Movie Capture) |
| `camera.py` | director camera with slow orbit (`--camera free` to fly it yourself) |
| `run_mission.py` | the CLI |

## Run it

Always run from anywhere; the script adds the repo root to `sys.path`. Python 3.10-3.12.
**Your first command on a GPU box is the selftest** (about 3-10 min incl. shader compile): it checks every risky
call in order and prints `SELFTEST PASS|WARN|FAIL|SKIP <step>: <detail>`, then saves `<out>/selftest.json` and
`<out>/selftest_frame.png`. Send the whole log + those two files back if anything FAILs.

Two launch routes, picked automatically (`--launcher auto|lab|kit`). Facts behind them:
[docs/ISAAC_VERIFIED_APIS.md](../docs/ISAAC_VERIFIED_APIS.md).

| Route | When | Boots with | Physics / stepping |
|---|---|---|---|
| **lab** (verified by the team on 6.0.1) | `./isaaclab.sh -p ...` (Isaac Lab importable) | Isaac Lab `AppLauncher` (its flags: `--headless`, `--livestream 2`, `--enable_cameras`, `--device`, `--viz kit` ...) | `SimulationContext(enable_scene_query_support=True, use_fabric=False)`, `sim.reset()`, `sim.step()` |
| **kit** (4.5 / 5.x fallback) | `~/isaacsim/python.sh ...` | `SimulationApp`; `--livestream [1\|2]` enables the livestream extension | our `/World/physicsScene` (scene queries on), timeline play, `app.update()` |

**0. Any laptop, no GPU (checks everything except omni/pxr):**

```bash
python isaac/run_mission.py --dry-run --max-steps 60 --out /tmp/isaac_dry
python isaac/run_mission.py --selftest --dry-run --out /tmp/isaac_st      # selftest plumbing only
python -m pytest -q tests/test_isaac_dryrun.py
```

**(a) Brev Isaac launchable (Isaac Sim 6.0.1 + Isaac Lab 3.0, L40S): the team's verified route.**
The launchable is 6.0.1, not 5.1. Put the repo at `/workspace/skysearch` (branch `larpverse-sensing`; our code is
in `larpverse/`). In the VS Code terminal:

```bash
cd /workspace/isaaclab
./isaaclab.sh -p -m pip install scipy                         # LiDAR->map rebuild and --radar dsp
L=/workspace/skysearch/larpverse
# 1) diagnosis (first command):
./isaaclab.sh -p $L/isaac/run_mission.py --selftest --headless --enable_cameras --out /workspace/selftest
# 2) live, streamed: watch at <brev URL>/viewer (keep one viewer tab)
./isaaclab.sh -p $L/isaac/run_mission.py --livestream 2 --viz kit --enable_cameras --speed 4 --show-truth --radar dsp
# 3) record headless on a virtual clock (smooth video however slow the render); PNGs per camera + timeline.csv
./isaaclab.sh -p $L/isaac/run_mission.py --headless --show-truth --radar dsp --photoreal 1 --record /workspace/rec \
    --rec-cams director,wide,chase:d1,fpv:d2,top --rec-res 1920x1080 --rec-fps 30 --rec-speed 6 --rec-warp 4 --rec-stop-t 250
#    (the teammate recorded with /workspace/isaaclab/_isaac_sim/python.sh ... --launcher kit is that exact route)
# dashboard: second terminal, then open <VS Code URL>/proxy/8765/dashboard/  (8080 is code-server)
cd $L && python3 -m http.server 8765
```

If `--viz kit` is rejected by your Isaac Lab version, drop it (unknown flags are reported, not fatal).

**(b) Isaac Sim workstation install (zip / launcher, e.g. `~/isaacsim`, 4.5 / 5.x):**

```bash
cd skysearch/larpverse        # repo root of our code (branch larpverse-sensing)
~/isaacsim/python.sh -m pip install scipy          # only needed for --radar dsp and the LiDAR window rebuild
~/isaacsim/python.sh isaac/run_mission.py --selftest --out /tmp/selftest
~/isaacsim/python.sh isaac/run_mission.py --seed 7 --speed 4 --out dashboard/data
# second terminal: dashboard
python3 -m http.server 8765    # open http://localhost:8765/dashboard/
```

Windows: `C:\isaacsim\python.bat isaac\run_mission.py --seed 7 --speed 4`.

**(c) pip install (Ubuntu 22.04+/Windows, Python 3.11 for 5.1):**

```bash
python3.11 -m venv ~/isaac-venv && source ~/isaac-venv/bin/activate
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
pip install scipy
cd skysearch/larpverse && python isaac/run_mission.py --selftest --out /tmp/selftest && python isaac/run_mission.py --seed 7 --speed 4
```

The first launch asks you to accept the EULA (or set `OMNI_KIT_ACCEPT_EULA=YES`) and spends several
minutes compiling shaders. That is normal.

**Recording** (`isaac/photoreal.py`, verified on the L40S). `--record DIR` writes `DIR/<cam>/fNNNNN.png` per
camera from replicator render products (headless-safe) plus `DIR/timeline.csv` (frame -> mission t, phase, pile,
pings) for cutting. Cameras: `--rec-cams director,wide,chase:d1,fpv:d2,top`. The mission runs on a virtual clock
at `--rec-fps` with `--rec-speed` sim-seconds per video second (`--rec-warp 4` speeds up radar/rescan/relocate),
`--rec-stop-t` ends it, `--rec-every N` thins previews, `--rec-windows "director:120-250;wide:0-110"` renders only
the frame ranges the cut uses (cheap path-traced passes). Budget: ~2.2 s per frame for 3 cameras at 720p
real-time and 6.5 s with `--rtx pathtracing --spp 16`. In the GUI, Movie Capture still works: pick camera
`/World/DirectorCam` and use `--overlay usd`.

**Look.** `--look basic` (default): real `UsdPreviewSurface` materials on every prim, low warm sun 1800 + cool sky
350 (the team's measured values; 1200/3000 washed out). `--photoreal 1` (or its alias `--look pretty`, which also
adds Omni Flow dust under drones near the ground; `--flow 0` to disable): NVIDIA MDL concrete / brick / gravel /
dirt / asphalt / rusted steel, every rubble box swapped for an irregular piece of the same pose and size that is
also the LiDAR's collider, debris, dusk HDRI (`--hdri FILE --sky 900`), low sun (`--sun 2200 --sun-elev 24`),
haze, vegetation (`--veg 0` off), operator (`--operator 0` off); LiDAR hits are drawn only within 11 m of the
pile being mapped. `--rtx pathtracing --spp 16` for hero shots. `--look plain` = bare displayColor, diagnosis only.

### Useful flags

`--selftest` · `--launcher auto|lab|kit` · `--look basic|pretty|plain` · `--photoreal 1` ·
`--rtx realtime|pathtracing --spp 16` · `--sun --sky --sun-elev --hdri FILE --assets ROOT --veg --operator` ·
`--record DIR --rec-cams --rec-res 1920x1080 --rec-fps 30 --rec-speed 6 --rec-warp --rec-stop-t --rec-windows
--rec-every` · `--fabric` ·
`--start-pile B2` · `--radar dsp` · `--speed 4` (sim-seconds per wall-second) · `--show-truth` (victim ghosts,
display only) · `--frames` (record `frames/state_XXXX.json`) · `--replay DIR` (play recorded frames, no brain)
· `--max-steps N` · `--camera free` · `--drone-asset [PATH|URL]` · `--drone-scale 1.6` · `--rtx-lidar` ·
`--overlay auto|debug|usd` · `--rays 96` · `--no-keep-open`.

**Backup plan.** Record a numpy run on any laptop and replay it in Isaac if the live brain breaks:

```bash
python -m sim.grid_mission --seed 7 --frames --out dashboard/data      # laptop
~/isaacsim/python.sh isaac/run_mission.py --replay dashboard/data       # Isaac box (copy the folder over)
```

If the live `GridMission` import fails and `<out>/frames/` exists, `run_mission.py` replays it automatically.

## Bring-up checklist (teammate, first 30 minutes)

1. **Selftest** (commands above). What each step means and what to do on FAIL/WARN:
   - `app_boot`: launcher, versions. FAIL: run through `./isaaclab.sh -p`, or force `--launcher kit` with `python.sh`.
   - `mission_import`: WARN `scipy MISSING` -> `./isaaclab.sh -p -m pip install scipy`.
   - `stage_build`: expect ~620 pieces, >620 colliders, upAxis Z, metersPerUnit 1.
   - `physics_parse`: lists physics scenes and `sceneQuery`. Lab route: `/physicsScene` from SimulationContext.
   - `raycast`: pile B2 and bare terrain at (11, 11) must hit, terrain z within 0.35 m of the scenario. FAIL means
     LiDAR silently falls back to the heightmap; fix before recording.
   - `lidar_sweep`: >0 of 96 rays hit from above A2.
   - `drone_pose`: pose read back from USD.
   - `camera_switch`: PASS if the viewport took `/World/DirectorCam`; WARN is fine headless if
     `/OmniverseKit_Persp` or `/World/Cams/rec` follow (we re-pose them every frame: the team's black-viewport fix).
   - `debug_draw`: WARN only means overlays use USD prims.
   - `render_frame`: `mean_rgb` of the saved frame; FAIL "black" -> see HINT (livestream client, shader compile,
     `--look plain`, `--sun 3000`). Open `selftest_frame.png`: piles, a green line, a red ping, a ring around B2.
2. **Dry-run with Isaac's python** if `mission_import` failed: `... run_mission.py --dry-run --max-steps 60 --out /tmp/d`.
3. **Live run**, streamed or GUI: three drones lift off at y ~ -49 and fly to A2, LiDAR points appear, red ping,
   cones, then A3. Log lines: `stage built`, `PhysX raycast self-test ok`.
4. **Raycasts return nothing** at runtime (`WARNING PhysX raycast hit nothing`): rerun `--selftest`; in the Script
   Editor: `from omni.physx import get_physx_scene_query_interface as q; q().raycast_closest((0,0,30),(0,0,-1),100)`.
5. **Viewport doesn't follow**: `--camera free` and fly it; on Brev, recordings use `/World/Cams/rec` anyway.
6. **No lines or points visible**: `--overlay usd` (USD BasisCurves and Points under `/World/Overlay`).
7. **Drones invisible or underground**: `/World/Drones/d1..d3`, staging pads y ~ -49; `--drone-scale 2.5`;
   `--drone-asset` falls back to primitives (log line `[drone_viz] ...`); referenced assets are scaled by their
   metersPerUnit.
8. **Window freezes at start**: shader compilation on the first run (5-10 min, ~9 min more for `--look pretty`).
9. **Too slow**: `--rays 48`, `--overlay usd`, lower `--speed`, `--look basic`.
10. **Dashboard not updating**: the mission writes `--out` (default `dashboard/data`). Serve the repo root on 8765;
    on Brev open `<VS Code URL>/proxy/8765/dashboard/`.

## Version notes

| | 4.5 | 5.1 | 6.0.1 + Isaac Lab 3.0 (Brev; verified by the team) |
|---|---|---|---|
| launch | `SimulationApp` (kit route) | same | Isaac Lab `AppLauncher` (lab route); 5.x core API crashes under Isaac Lab |
| physics | timeline + `app.update()` | same | `isaaclab.sim.SimulationContext` + `sim.step()`; `enable_scene_query_support=True` or rays miss |
| poses | pxr xform ops | pxr (experimental XformPrim if present) | pxr with `use_fabric=False`; Kit cameras via experimental `XformPrim(reset_xform_op_properties=True)` |
| debug draw ext | `isaacsim.util.debug_draw` | same | same (verified signatures) |
| assets root | `isaacsim.storage.native.get_assets_root_path` | same | `isaaclab.utils.assets.ISAAC_NUCLEUS_DIR` = `.../Assets/Isaac/6.0/Isaac` |
| quadcopter USD | `/Isaac/Robots/Quadcopter/quadcopter.usd` | `/Isaac/Robots/IsaacSim/Quadcopter/quadcopter.usd` (unverified) | `/Isaac/Robots/IsaacSim/Quadcopter/quadcopter.usd` (exists) |
| frame capture | replicator (unverified) | replicator (unverified) | Isaac Lab `Camera` sensor (verified) |
| RTX LiDAR | `IsaacSensorCreateRtxLidar` + annotator | same | moved; `--rtx-lidar` falls back (never run by anyone) |
| Pegasus | works | works | broken: not used |
| Python | 3.10 | 3.11 | 3.12 |

Outside `compat.py` we use only `pxr` (Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade), `omni.usd`, `omni.physx` scene
queries and carb settings; `look.py` adds Isaac Lab's `MdlFileCfg` (lab route) and `omni.flowusd`.
