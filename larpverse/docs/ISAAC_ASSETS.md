# Isaac scene assets: command van, victim dummies, heavy drone with real components

These assets are for the deliverable: a single take of at most 60 s from the Isaac stream, with perception on screen and one unscripted live change. Everything here is procedural USD (`pxr` only). It was built and checked offline with usd-core, without a GPU. Nothing has been rendered in Isaac yet. Every asset is visual only, with **no colliders**, so the LiDAR and the landing search never see the van, the dummies or their rubble cover.

| File | What |
|---|---|
| `isaac/van.py` | Command van and staging dressing. It also holds the mesh kit everything else uses: `Geo`, `Looks`, `Assembly`, `emit`, `count_tris`. CLI: test stages plus projection sheets. |
| `isaac/dummies.py` | Manikin dummies with SpatialAudio and `larpverse:*` attributes. It reads `assets/audio/dummies/manifest.json`. |
| `isaac/drone_viz.py` | Heavy X8 carrying the `data/hardware_bom.json` components. Also provides `set_state` (flight motion), rotor blur, the belly radar and label anchors. |
| `isaac/pods.py` | The BOM radar-module geometry, plus `PodViz`, which drives each drone's belly radar or draws separate radar markers. |
| `assets/textures/` | Livery (`van_livery_full/panel/stripes.png`, `van_rear.png`) and the three monitor screens (`dash_recon/map/detail.jpg`), regenerated with `python -m isaac.van --textures`. |
| `tests/test_isaac_assets.py` | 10 tests under `.venv` with pxr. |

Sanity sheets (matplotlib projections of the real meshes, painter's algorithm, displayColor; no textures):
`docs/img/isaac_assets_van.png`, `docs/img/isaac_assets_drone.png`, `docs/img/isaac_assets_dummies.png`.
Rebuild: `scripts/heavy.sh .venv/bin/python -m isaac.van --out /tmp/larpverse_assets` (about 15 s). It also writes
`assets_grid.usda` / `assets_collapse.usda` there, which you can open in usdview or Isaac.

## Triangle counts (measured)

| Asset | Triangles | Notes |
|---|---|---|
| Van, everything | 12,482 | Includes the interior, the operator, the roof kit, tent, generator, cables and cones. The budget was 50k. |
| Heavy drone (each) | 10,378 | Includes 8 rotors (blades + 2 blur discs each) and the belly radar. Three drones: 31k. |
| Separate radar marker (`PodViz` without drones) | 1,168 | Only used when `PodViz` is not given the drones. |
| Dummies, grid seed 7 | 21,300 | 9 prims: 4 victims, dog, generator, settling (audio only), 2 hidden. |
| Dummies, collapse seed 7 | 62,544 | 24 prims: 22 victims and 2 hidden. Dummies deeper than 1.4 m use a low-detail body. |

Total on the collapse site: about 106k triangles, against the 1.9M of the site terrain.

## Command van (`isaac/van.py`)

```python
from isaac import van
x, y, z, yaw = van.staging_pose(sc)        # 10 m east of staging, beside the pads; door faces the pads (west)
van.hide_legacy_staging(stage)             # deactivate scene.py's placeholder Truck / Mast / Beacon / Table / Screen
info = van.build_van(stage, "/World/Van", (x, y), yaw, screens=van.default_screens(), z=z)
# info = {"tris": {group: n}, "total": n, "anchors": {"door", "screens", "operator_head", "mast_top", "dish", ...}}
```

- **Pose.** Collapse site: (−46.0, −154.5), yaw −90. Grid site: (10.0, −49.0), yaw −90. Both are checked clear of the pads (more than 6 m) and inside the site bounds.
- **Frame.** In the van frame, +x is the front (it faces south at yaw −90), the door is on −y (west), and the origin is the ground under the cargo-box middle.
- **Body.** The cab has glazing, headlights, amber lamps and mirrors. The cargo box is 5.05 × 2.5 m. The livery is a generic "SEARCH & RESCUE / MOBILE COMMAND UNIT / CMD 1" in navy and amber, with no real organisation's branding. There are six wheels, a hinged side door swung open by 100° with two steps, amber/navy rear chevrons, and a ladder.
- **Lamps.** No red lamps anywhere: red means a radar ping.
- **Interior:**
  - a desk with three monitors (left: recon, centre: map with pings, right: LiDAR ping detail) as emissive textured UsdPreviewSurface (`emissiveColor` ← texture × 2.5);
  - a seated operator mannequin in a hi-vis vest with a headset, facing the screens, with their back to the door;
  - a comms rack with green and blue LEDs;
  - a ceiling light strip, plus `RectLight`s for the ceiling (1800), the screen glow (600) and the floodlight (9000). Scale them with `light_scale`.
- **Roof:**
  - a telescopic mesh-radio mast, 6 m extended (`mast_h`), with three omni antennas, a sector panel, guy lines and a **green coordinator beacon** on top;
  - a Starlink-style flat panel on a short mast;
  - an amber/white light bar on the cab;
  - an A/C unit and roof rails.
- **Outside:**
  - a 3 × 3 m blue pop-up tent behind the van, with a table, a stretcher on trestles, kit bags and a floodlight tripod;
  - a framed generator with a fuel can;
  - cable runs with a cable protector;
  - 7 cones along the pad edge.
- **Materials.** All UsdPreviewSurface, as used by scene.py and the team's verified spawners.
  - `mdl=True` adds an OmniPBR `mdl` surface output to the emissive materials (screens, lamps, beacon) in case RTX shows PreviewSurface emission too dim.
  - `mdl_override={"canopy": photoreal.mdl_material(...)}` binds any MDL over a material key.

## Victim dummies (`isaac/dummies.py`)

```python
from isaac import dummies as DM
man = DM.load_manifest()                       # larpverse.dummy_audio/v1 (None if missing: no clips, no hidden)
DM.build_dummies(stage, sc, man)               # /World/Dummies/<id>
DM.build_ambient(stage, sc, man)               # optional: wind bed (nonSpatial) + distant siren
rows = DM.read_dummies(stage)                  # pure read, every listen
DM.set_active(stage, "H1_C3_vi", True)         # the unscripted change (or tick larpverse:active in the Property panel)
```

**Manifest mapping:**
- Grid: a victim whose pile has a profile of the same `scenario_kind` uses that profile and its prim name. For seed 7 that gives `/World/Dummies/A2`, `A3`, `B1` and `B2`.
- Collapse: there are no pile matches, so each victim (`V1`…`V22`) takes a profile of its kind round-robin, and its physio comes from the scenario.
- Distractors (`C2_dog`, `C1_generator`, `B3_settling`) are built only where their pile exists (grid). The dog is a standing dog, the generator a small framed generator, and settling is audio only.
- Hidden profiles are built **inactive**:
  - on the grid at their own pile: `H1_C3_vi` at (22.1, −22.7, 2.4) and `H2_A1_id`;
  - on the collapse site, remapped to the nearest empty sectors from the start pile: `H1_RW_vi` (−41.3, −9.5, 0.8) and `H2_M39_id` (−46.5, 36.5, 0.2).
- Hidden dummies sit on the pile face toward staging, at `burial_depth_m` below the surface.

**Prim** `/World/Dummies/<id>` (Xform at the breathing point, absolute z from `isaac.site.victim_z`):

| Attribute | Type | Meaning |
|---|---|---|
| `larpverse:active` | bool | Sensor models use this dummy (the live loop skips inactive ones). |
| `larpverse:clip` | string | Current clip, repo-relative wav (`""` = silent). |
| `larpverse:physio` | string | `calm`, `distressed_tachypnea`, `unconscious_slow` or `irregular_apneic` for the radar generator; `none` for non-humans. |
| `larpverse:physio_json` | string | The full physio dict (rate, depth, cv, apnoea) when the scenario has it. |
| `larpverse:clips` / `reply_clips` | string[] | Idle playlist and after-call-out playlist. |
| `larpverse:schedule` | string (JSON) | The manifest's `idle` / `after_callout` blocks plus the story line. |
| `larpverse:kind`, `human`, `hidden`, `pile`, `victim`, `depth`, `pose`, `spl_db_1m`, `profile`, `id` | | Bookkeeping. |

**Children:**
- `Body/`: a manikin with navy or dusty coveralls, a tan vinyl head and hands, and black boots. No injuries, no blood.
- `Cover/`: broken slabs over the legs and one side, chunks and rebar, kept below the surface.
- `Audio`: a `UsdMedia.SpatialAudio` at the head.
  - `filePath` is the absolute wav path and `auralMode` is `spatial`.
  - `playbackMode` is `loopFromStart` for loop schedules.
  - `gain` is 1 when the dummy is active and has a clip, else 0. `set_active` keeps it in sync.

**Poses follow the breathing state:**
- calm: supine, with one arm up by the head (knocking);
- distressed: curled on the side;
- unconscious: prone;
- irregular: curled.

`read_dummies` returns `{id, prim, xyz (live world position), active, clip, physio, kind, human, hidden, pile, victim, clips, reply_clips, schedule, depth, pose, spl_db_1m}`. Dragging a dummy's prim in the viewport moves its source, and the test checks this.

## Heavy drone (`isaac/drone_viz.py`) and belly radar (`isaac/pods.py`)

**Geometry.** The airframe comes from `sim.platform` heavy_sar: coaxial X8, 2.2 m wheelbase, 1.375 m props, carbon arms with fold joints, chamfered fuselage and canopy, side DB2000 packs and splayed landing gear.

**Payload.** Every component sits at its `hardware_bom.json → isaac_mounts` position, and the test asserts each one:

| Part | BOM component | Mount / detail |
|---|---|---|
| `LidarPuck` | OS1-128 | r 0.043 m on a 45° nose-down bracket |
| `Gimbal` + `Thermal` | thermal camera | |
| `ArrayRing` with `Mic0..7` | mic ring | 1.0 m ring |
| `SpeakerHorn` | speaker | mouth down-aft |
| `ComputeBox` | Jetson | fins and a green status LED |
| `MeshRadio` | mesh radio | stacked on the compute box, blue LED |
| `MeshAntennaL/R` | mesh radio | under the rear arms |
| `WifiApAntenna` | Wi-Fi AP | down, blue |
| `PodLinkAntenna` | legacy prim name; the radar module is cabled to the drone | |
| `GnssMast` | GNSS | on a pole |
| `Winch` + drum | legacy prims, unused now that the radar is fixed under the belly | |
| Arm LEDs | | front white, rear in the drone's accent colour |

**Belly radar.** The radar-module parts follow the BOM radar-module frame: `PodRadarFace`, `PodMic0..3`, `PodLiftEye`, `PodStrobe`, `PodAntenna`, a 0.5 × 0.5 × 0.25 m case and a mounting bracket (`PodLiftEye` is a legacy prim name for the module's mounting eye). The radar module is fixed under the drone between the skids with its base at 0.175 m.

**The radar is the belly module, not a separate panel on the body.** `delivery="land"` puts the older `RadarPanel` back under the belly.

**Flight-motion route** (for `sim/flight_motion.py` / `isaac/motion.py`):

```python
d.set_state(pos, quat_wxyz, rotor_rpm, winch_len, dt, sway_deg=(roll, pitch), ground_z=None)
drone_viz.set_state(drones, "d1", pos, quat, rotor_rpm, winch_len, dt)     # fleet helper
```

- `pos` is the same point as `set_pose` (skid bottom + 0.15). `quat` is (w, x, y, z), or a `Gf.Quat`.
- `rotor_rpm` takes 8 values in `ROTOR_ORDER = [Rotor0_0, Rotor0_1, …, Rotor3_1]` (arm k at 45 + 90k°, lower then upper), or one number for all.
- Each rotor turns by rpm × 6 × dt degrees with alternating directions:
  - below 120 rpm: still blades;
  - from 120 to 450 rpm: blades plus a faint disc (spin-up and spin-down);
  - from 450 rpm (`BLUR_RPM`): a translucent motion-blur disc with a darker tip rim, and the blades are hidden.
- **Belly radar.** The radar module is fixed under the drone and tilts with the body; it does not hang or sway, so `winch_len` and `sway_deg` are legacy arguments that should be zero or ignored. With `ground_z` the drone sets down so the radar module rests on the surface.
- **Older route.** `set_pose` + `spin(dt)` still work. `spin` drives the hover rpm (1700), so the blur shows.

**Labels:**
- `component_anchors()` returns `[{key, part, label, bom_id, bom_name, parent, xyz}]` in the model frame.
- `DroneViz.anchors_world()` returns `{part: world xyz}` from the live transforms, including the belly radar.
- Every part prim carries `larpverse:part`.

**PodViz:**
- `PodViz(stage, ids, drones=drones)` drives each drone's own belly radar from `state.pods` / `winch_len_m` (legacy field names). A radar resting on the rubble sits at the state `z`; the radar is fixed under the drone and does not hang or sway.
- Without `drones=`, it draws a separate radar marker as before, and hides the drone's belly radar while the separate one is visible.

## Camera suggestions for the single take (collapse site)

The van is at (−46.0, −154.5) facing south, with its door facing the pads to the west. The pads are at (−58.6 / −56.0 / −53.4, −150). T1a is about 170 m north, at (−31, 14). Use one continuous director spline with no cuts, a 35 mm equivalent lens (focal length 24 on the 20.955 aperture), and re-pose every frame (black-viewport fix).

1. **0–6 s, van foreground.** Eye at (−53.5, −161.0, 1.7), looking at the door (−47.7, −154.1, 2.0) with `van_anchors()["door"]` transformed. The open door frames the operator's back and the three glowing dashboards, while the tent floodlight and the green mast beacon read against the dusk sky. The drones sit on the pads at frame left.
2. **6–12 s, lift-off.** Crane up to 9 m and pan left to the pads while the drones spin up: the blades turn into blur discs as rpm crosses 450. The van stays at frame right with the mast and Starlink panel in silhouette.
3. **12–34 s, fly-along.** Hand over to a chase on d2: 6 m behind, 3 m up, tilted down 15–20°, eased in over 2 s by blending the spline target from the pads to d2. Keep the other two drones in frame: they tilt into the acceleration and level off at cruise. The purple LiDAR paints in as they reach the pile.
4. **34–50 s, close pass with the landing.** While d1 descends onto a flat spot on its pile:
   - orbit d1 at 3.5 m, from the nose quarter (LiDAR on its 45° bracket, the GNSS mast) round to the belly (the radar module);
   - drop the camera with the drone until the radar rests on the rubble.
   Burn labels from `anchors_world()`: LiDAR, mic ring, speaker, Jetson, mesh radio, Wi-Fi AP, belly radar face.
5. **50–60 s, the unscripted change and its reaction.** Pull back to about 25 m, 30° down. When someone flips `larpverse:active` on `H1_RW_vi` (collapse) or `H1_C3_vi` (grid), its voice starts. The perception panel shows the new acoustic direction (yellow), and the coordinator's green dispatch line swings to the new sector. End on the red ping beacon.

## Requests (run_mission.py and the live loop, owned by other agents)

1. **`--van 0|1` (default 1 with `--photoreal 1`)** in `isaac/run_mission.py`, right after `photoreal.dress(...)`. This needs `--operator 0` when the van is on: the van has its own seated operator, and photoreal's standing operator would sit in the old table spot.
   ```python
   if a.van:
       from isaac import van
       x, y, z, yaw = van.staging_pose(sc)
       van.hide_legacy_staging(stage)
       van.build_van(stage, "/World/Van", (x, y), yaw, z=z, mdl=a.van_mdl)
   ```
   Optional `--van-mdl` (default 0): adds OmniPBR outputs to the emissive materials if the screens read too dim.
2. **`--dummies 0|1` (default 1)**: after the scene build, call `DM.build_dummies(stage, sc, DM.load_manifest())` and optionally `DM.build_ambient(stage, sc, man)`.
   - In the live loop, call `DM.read_dummies(stage)` at every listen and feed the radar and acoustic generators only `active` rows, using `xyz`, `physio` and `clip`. Isaac is the source of truth for the take.
   - When the dummies are on, drop `--show-truth` ghosts: the dummies replace them.
3. **Belly radars ride on the drone.** Replace `PodViz(stage, ids)` with `PodViz(stage, ids, drones=drones)`, and pass `dt` in the update (`pods.update(cur, poses, dt=frame_dt)`).
4. **Flight motion.** Where poses come from `isaac/motion.py`, call `drones[did].set_state(pos, quat_wxyz, rotor_rpm, winch_len, dt, ground_z=pod_surface_z)` instead of `set_pose` + `spin`. Keep `pods.update` only for the resting `z` when the motion module does not supply `ground_z`.
5. **Labels.** The overlay or HUD can project `drones["d1"].anchors_world()` and use the `component_anchors()` labels for the close pass.
6. **Test update (orchestrator).** In `tests/test_platform.py::test_heavy_drone_parts_from_spec`, change the required name `"RadarPanel"` to `"PodRadarFace"`. With the belly radar module the body has no separate radar panel any more, per the BOM and the coordinator's instruction. `heavy_parts(spec, did, delivery="land")` still has `RadarPanel`.
7. **Unverified on GPU, please report in FROM_ISAAC.md:**
   - SpatialAudio playback in the stream: does Kit's audio player pick up `UsdMedia.SpatialAudio` prims with `gain` toggled at runtime, and does the WebRTC client carry the audio?
   - how bright the PreviewSurface emissive screens read at dusk (try `--van-mdl 1` if dim);
   - translucent blur discs (UsdPreviewSurface `opacity` 0.30 / 0.12) under RTX real-time.
