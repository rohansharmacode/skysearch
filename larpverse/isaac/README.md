# Isaac Sim adapter (3x3 rubble-grid SAR demo)

Isaac is the world and the body; the brain is `sim/grid_mission.py` (numpy, never sees ground truth).
`isaac/run_mission.py` builds the stage from the same seeded `sim/scenario.py`, poses three visual-only
drones from the mission state every render frame, raycasts a spinning LiDAR against the PhysX colliders
while the drones survey (points go back to the mission via `ingest_pointcloud`, which rebuilds the
landing-search heightmap), and draws pings, range rings, acoustic cones and dispatch arcs from the state.
The mission writes `<out>/state.json`, so the web dashboard shows the Isaac run live.

| File | What |
|---|---|
| `compat.py` | SimulationApp import shim, debug_draw, viewport camera, PhysX raycaster, stage/timeline/assets helpers |
| `scene.py` | stage from the scenario (pxr only): terrain mesh, ~620 collidable rubble boxes, staging area, `--show-truth` ghosts |
| `drone_viz.py` | primitive quadrotor (rotors spin, purple LiDAR puck, orange radar, teal mics, skids) or `--drone-asset` |
| `lidar.py` | `PhysxLidar` (Isaac), `HeightmapLidar` (dry-run), optional `RtxLidar` |
| `overlay.py` | state-driven visuals; debug_draw or USD prims (`--overlay usd` for Movie Capture) |
| `camera.py` | director camera with slow orbit (`--camera free` to fly it yourself) |
| `run_mission.py` | the CLI |

## Run it

Always run from the repo root (the script adds it to `sys.path`). Python 3.10–3.12.

**0. Any laptop, no GPU (checks everything except omni/pxr):**

```bash
python isaac/run_mission.py --dry-run --max-steps 60 --out /tmp/isaac_dry
python -m pytest -q tests/test_isaac_dryrun.py
```

**(a) Isaac Sim workstation install (zip / launcher, e.g. `~/isaacsim`):**

```bash
cd ~/agentverse
~/isaacsim/python.sh -m pip install scipy          # only needed for --radar dsp and the LiDAR window rebuild
~/isaacsim/python.sh isaac/run_mission.py --seed 7 --speed 4 --out dashboard/data
# second terminal: dashboard
python3 -m http.server 8765    # open http://localhost:8765/dashboard/
```

Windows: `C:\isaacsim\python.bat isaac\run_mission.py --seed 7 --speed 4`.

**(b) pip install (Ubuntu 22.04+/Windows, Python 3.11 for 5.1):**

```bash
python3.11 -m venv ~/isaac-venv && source ~/isaac-venv/bin/activate
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
pip install scipy
cd ~/agentverse && python isaac/run_mission.py --seed 7 --speed 4
```

The first launch asks you to accept the EULA (or set `OMNI_KIT_ACCEPT_EULA=YES`) and spends several
minutes compiling shaders. That is normal.

**(c) Brev / cloud (Isaac Sim 5.1 Launchable, L40S):**

```bash
git clone <repo> agentverse && cd agentverse
# headless with the WebRTC livestream, then connect with the Isaac Sim WebRTC Streaming Client to the instance IP
OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh isaac/run_mission.py --headless --livestream --speed 4 --overlay usd
# or plain headless (state.json only); view the dashboard through an SSH tunnel:
ssh -L 8765:localhost:8765 <brev-host>   # then on the instance: python3 -m http.server 8765
```

To record video: run in the GUI (the Launchable's desktop/VNC, or the livestream client), open
*Window → Movie Capture*, pick camera `/World/DirectorCam`, and start capture. Use `--overlay usd` when
recording: debug_draw lines are a viewport overlay and may not appear in captured frames. If the cloud
image path differs, find `python.sh` with `ls ~/isaacsim* /isaac-sim 2>/dev/null`.

### Useful flags

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

1. **Dry-run first, with Isaac's python.** `~/isaacsim/python.sh isaac/run_mission.py --dry-run --max-steps 60 --out /tmp/d`.
   It must print a `summary` with `lidar_points > 0`. If it fails, the problem is Python deps, not Isaac.
2. **Headless smoke test.** `... run_mission.py --headless --max-steps 30 --no-keep-open`. Look for
   `stage built: 621 rubble pieces` and `PhysX raycast self-test ok`.
3. **Raycasts return nothing** (`WARNING PhysX raycast hit nothing`). The run falls back to the heightmap
   LiDAR so the demo still works. To fix it: check the timeline is playing (press Play), that
   `/World/physicsScene` exists, and that *Window → Physics → Debug* shows colliders on `/World/Rubble`.
   Try more warm-up updates (increase the `range(30)` loop in `run_mission.py`). Also check that
   `omni.physx` scene queries are the problem by running in the Script Editor:
   `from omni.physx import get_physx_scene_query_interface as q; q().raycast_closest((0,0,30),(0,0,-1),100)`.
4. **Viewport camera doesn't switch** (`[camera] could not switch the viewport`). Pick `DirectorCam` in the
   viewport's camera dropdown (top-left of the viewport). `--camera free` gives you the normal fly camera.
5. **`--radar dsp` fails with `No module named scipy`.** Run `~/isaacsim/python.sh -m pip install scipy`.
   The LiDAR heightmap rebuild in `grid_mission.py` also uses scipy. Without it, use `--radar model`.
6. **No lines or points visible.** debug_draw may not be enabled in your build. Use `--overlay usd`
   (USD BasisCurves and Points prims under `/World/Overlay`).
7. **Drones invisible or underground.** They are under `/World/Drones/d1..d3` and start at the staging
   pads (y ≈ -49). `--drone-scale 2.5` makes them easier to see. `--drone-asset` falls back to primitives
   if the quadcopter USD can't be found; check the log line `[drone_viz] ...`.
8. **Window freezes at start.** That's shader compilation on the first run (can take 5–10 min). Wait for it.
9. **Too slow.** Use `--rays 48` and `--overlay usd`, and lower `--speed`. The terrain is 34k quads and
   there are about 620 boxes, which is light for an RTX card.
10. **Dashboard not updating.** The mission writes `--out` (default `dashboard/data`). Serve the repo root
    with `python3 -m http.server 8765` and open `/dashboard/`.

## Version notes

| | 4.5 | 5.1 (recommended) | 6.x |
|---|---|---|---|
| `SimulationApp` | `from isaacsim import SimulationApp` | same | same |
| debug draw ext | `isaacsim.util.debug_draw` | same | same (unverified) |
| assets root | `isaacsim.storage.native.get_assets_root_path` | same | same (unverified) |
| quadcopter USD | `/Isaac/Robots/Quadcopter/quadcopter.usd` | vendor folders, e.g. `/Isaac/Robots/IsaacSim/Quadcopter/` (unverified) | as 5.x |
| RTX LiDAR | `IsaacSensorCreateRtxLidar` command + annotator | same | `isaacsim.sensors.experimental.rtx` (not wired; `--rtx-lidar` falls back) |
| Pegasus | works | works | broken: not used |
| Python | 3.10 | 3.11 | 3.12 |

Only these APIs are used outside `compat.py`: `pxr` (Usd, UsdGeom, UsdLux, UsdPhysics), `omni.usd`,
`omni.timeline`, `omni.physx` scene queries, and `omni.kit.viewport.utility`.
