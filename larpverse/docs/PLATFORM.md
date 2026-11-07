# Platform: heavy-lift SAR drones

User decision (Nov 2026): Larpverse flies **big** drones, agricultural / heavy-lift class (DJI FlyCart 30 /
Agras T50-class coaxial X8) or Matrice 400-class industrial airframes, not small FPV frames. A bigger airframe buys
endurance, a dense LiDAR, a lower-frequency radar with a bigger antenna, a wide mic array and a loudspeaker. It
costs a much bigger landing patch, a wide and powerful downwash, and a heavy load on unstable rubble.

Code: `sim/platform.py` (`PlatformSpec`, presets, `get_platform()`; `python -m sim.platform` prints every derived
number). Tests: `tests/test_platform.py`.

**Labels.** Every figure is either taken from a manufacturer spec page (linked in the table, fetched 7 Nov 2026)
or marked *estimate*. Estimates are listed in each preset's `estimates` field. Landing limits are screening
thresholds for a candidate patch. They are **not** a structural assessment, and nothing here claims a landing is
safe.

## 1. Presets

| | `heavy_sar` (default) | `industrial_sar` | `legacy_small` |
|---|---|---|---|
| Airframe class | FlyCart 30 / Agras T50 coaxial X8 | Matrice 400 quad-X | The sim's original abstract ~1 m quad (not a product) |
| Span, props unfolded | **3.085 m** (2800 × 3085 mm) [FC30][T50] | 1.705 m *estimate* (1.07 m wheelbase + one 25-in prop) [M400] | 1.0 m *estimate* |
| Wheelbase / props | 2.2 m, 8 × 54 in (1.375 m) [FC30] | 1.07 m, 4 × 25 in [M400] | 0.65 m, 0.33 m *estimate* |
| Mass: airframe + batteries / SAR payload / MTOW | 65 kg *estimate* (95 kg MTOW − 30 kg rated cargo) / 12 kg *estimate* / 95 kg [FC30] | 9.74 kg [M400] / 4 kg *estimate* / 15.8 kg [M400] | 3 / 1.2 / 4.5 kg *estimate* |
| Battery | 2 × DB2000, 1984.4 Wh each [FC30] | TB100, 977 Wh [M400] | 160 Wh *estimate* |
| Endurance with SAR payload (to 0%) | **24.6 min** *estimate*: linear between the spec's 29 min hover (empty) and 18 min (30 kg) [FC30] | **36 min** *estimate*: 53 min hover with H30T [M400] scaled by mass^1.5 to 13.7 kg | 10.4 min (sim: 0.16%/s) |
| Usable with the 25% reserve | 18.5 min | 27 min | 7.8 min |
| Cruise / climb / descend | 10 / 3 / 2 m/s *estimate* (spec max 20 m/s) | 10 / 3 / 2 m/s *estimate* (spec max 25 m/s) | 4 / 2 / 1.2 m/s (sim) |
| Wind, ingress | 12 m/s, IP55 [FC30] | 12 m/s, IP55 [M400] | n/a |
| LiDAR | **Ouster OS1-128**, 128 ch, 45° VFoV, 90 m @10% [OS1]; 1024 × 10 Hz mode = **1.31 M pts/s**; σ 3 cm *estimate*; mounted pitched 45° (§3) | **Hesai XT32**, 32 ch, 640k pts/s, 360° × 31°, 0.18° × 1°, ±1 cm / 0.5 cm (1σ), 0.8 kg [XT32] | Livox Mid-360 class [MID360], simulated as a 32-ch −18…−80° fan, 230k beams/s |
| Radar | 0.4–2 GHz UWB, **0.60 × 0.50 m** panel, 4 kg *estimate* | 0.4–2 GHz, 0.35 × 0.35 m *estimate* | 0.4–2 GHz, 0.20 × 0.15 m *estimate* |
| Rx gain @0.6 / 1.2 GHz | 8.8 / 14.8 dBi | 4.9 / 10.9 dBi | 0 / 4.8 dBi |
| Mic array | 8 MEMS, 1.0 m ring *estimate*: beam ≈ 20° at 1 kHz | 8, 0.8 m: ≈ 25° | 6, 0.3 m (`acoustic/scene.py`): ≈ 65° |
| Loudspeaker | DJI Zenmuse V1, **129 dB SPL @1 m** (on M400), 800 Hz–8 kHz, 690 g [V1]; ≈ 103 dB at 20 m (spreading only) | same | none |
| Thermal (optional) | Zenmuse H30T thermal channel, 1280 × 1024 [H30T] | none | none |
| Hover thrust (static load once landed) | **755 N** | 135 N | 41 N |
| Rotor-disk induced velocity (hover) | 7.2 m/s over 5.9 m² of disk | 6.6 m/s over 1.3 m² | 7.0 m/s over 0.34 m² |

Sources: [FC30] https://www.dji.com/flycart-30/specs · [T50] https://ag.dji.com/t50/specs ·
[M400] https://enterprise.dji.com/matrice-400/specs · [M350] https://enterprise.dji.com/matrice-350-rtk/specs
(not used for a preset: 0.96 kg payload on a single gimbal is too little) · [OS1]
https://ouster.com/products/hardware/os1-lidar-sensor · [XT32] https://www.hesaitech.com/product/xt32/ ·
[L2] https://enterprise.dji.com/zenmuse-l2/specs · [MID360] https://www.livoxtech.com/mid-360/specs ·
[V1] https://enterprise.dji.com/zenmuse-v1/specs · [H30T] https://enterprise.dji.com/zenmuse-h30-series/specs
(the H30T thermal resolution was not re-fetched; treat it as *to verify*).

Why OS1-128 rather than Zenmuse L2 on the heavy drone: the L2 (240k pts/s single return, up to 1.2 M with 5
returns, 70° × 75° non-repetitive FoV, 5 cm H / 4 cm V system accuracy at 150 m, 905 g [L2]) is a survey
sensor built for straight lines at 50–150 m. We orbit a pile at about 10 m and need 360° coverage for SLAM. A
spinning 128-channel unit matches that better, and the sim's ray-caster models spinning sensors. The L2 is a
valid alternative for a high-altitude site survey before the drones go low.

### What the bigger sensors buy (and what they don't)

- **Radar.** UWB emission is EIRP-limited (47 CFR 15.509/15.510: through-wall imaging below 960 MHz is
  restricted to law enforcement, fire and rescue). A higher-gain transmit antenna must therefore back off its
  power. Only the **receive** gain is a platform advantage. At 0.6 GHz, the sub-1 GHz part of the sweep that
  carries the deep returns, the 0.6 × 0.5 m panel gains **+8.8 dB** over the small antenna. In the
  literature-preset penetration model the deep end loses about 13 dB per metre of rubble (two-way,
  0.65 × 10 dB/m). So +8.8 dB is worth roughly **+0.6 m of reach** (*estimate*), not a different class of
  sensor. The narrower beam (≈ 24° vs ≈ 72° at 1.2 GHz) also rejects off-axis clutter and a walking rescuer
  better. The 3 m practical limit (§4.2.3 of the report) still holds.
- **Mics.** Angular resolution scales with aperture: about 20° vs 65° at 1 kHz, so bearings are 3× tighter and
  cones from three drones intersect in a smaller region. Big rotors are much louder than small ones, so
  perch-and-listen with rotors off stays mandatory.
- **Speaker.** 129 dB @1 m supports a call-out protocol ("if you can hear this, knock three times"). Rubble
  attenuation is not modelled, so we make no claim about what a buried person hears.
- **Endurance.** 2.4× the legacy figure: about 18 min of usable flight per pack instead of 8.

## 2. Landing footprint and the structural note

`PlatformSpec.landing_limits()` gives the screening thresholds for `sim/landing.py` (heightmap at 0.5 m):

| | heavy_sar | industrial_sar | legacy_small (= current landing.py) |
|---|---|---|---|
| Plane-fit patch | **7 × 7 cells = 3.5 m** (gear 1.6 × 1.4 m + 2 × 0.5 m touchdown error) | 5 × 5 = 2.5 m | 3 × 3 = 1.5 m |
| Obstacle clearance radius | **1.94 m** (prop tip 1.54 m + 0.4 m) | 1.25 m | 0.9 m |
| Step limit inside radius | 0.20 m within 1.55 m | 0.35 m within 1.15 m | 0.35 m within 0.9 m |
| Slope, flat ground / perch | **8° / 10°** | 12° / 20° | 12° / 20° |
| Roughness | 0.05 m | 0.07 m | 0.07 m |
| Preference | street-level / firm slab at the pile edge | nearest flat cell to the mound | nearest flat cell to the mound |

The heavy-drone thresholds are *estimates*: stricter slope and step limits because a 77 kg airframe on fixed
skids has no adaptive legs.

**Structural note (heavy drone on rubble).** Once the rotors stop, a 77 kg drone puts about 755 N on whatever
it stands on. A slab that looks flat in the LiDAR map can be cantilevered over a void, which is exactly where a
casualty may be, and the extra load can shift debris. Its downwash is no faster than a small drone's (about
7 m/s at the disk), but it is about 3 m wide and carries about 18× the thrust. That lifts dust, which blinds
the LiDAR and thermal camera, and it can dislodge loose fragments around the patch. The rules we propose:

1. Prefer **street-level ground or firm slabs at the pile edge** (the 11 m stand-off row of the reach table).
   Perch on the pile only on a ≥ 3.5 m patch that the map shows flat, never above a radar or acoustic lead.
2. Approach and land with a vertical descent from about 3 m. Never hover low over a suspected void.
3. Every landing cell is a **candidate**. The map cannot see voids, rebar or slab support. A human or structural
   specialist signs off on perching on the pile.

What it costs in reach: from the pile edge (slant 8–11 m), the modelled radar reach is 0.6–1.2 m of rubble, or
roughly 1.2–1.8 m with the +0.6 m from the bigger antenna. Perching on the pile (slant about 2 m) gives about 2.9 m. The heavy drone
therefore trades some perching opportunities for fewer, better-supported landings. The industrial preset (2.5 m
patch, 135 N) is the natural complement for perching closer.

## 3. Dense LiDAR in the numpy sim

`mapping/raycast_lidar.py` changes:

- `LidarConfig.from_spec(platform_or_lidar_spec, sim_fraction)` builds the sensor from a spec.
- `tilt_deg` pitches the spin axis about body y. 45° (the preset default, chosen from the study below) slants the
  scan plane down-forward. 90° puts the plane across the track, like a vertically mounted survey LiDAR. Points
  are returned in the **body** frame (extrinsic applied), so SLAM and odometry are unchanged.
- `sim_fraction` casts a fresh random subset of the beam grid each sweep. The fast ray-OBB path maps grid beam
  ids to subset rows, and a test checks it against the dense path.
- The default `LidarConfig()` is the legacy fan, bit-for-bit unchanged, so the report numbers and the mission
  reproduce.
- Maps stay voxel-downsampled at 0.1 m (`slam.MAP_VOXEL`).

Study: `scripts/heavy.sh .venv/bin/python -m mapping.raycast_lidar --study <levels> [--tilt 45]`. Every row uses
the same seed-7 pile B2 orbit (9.5 m radius, 11 m altitude, 120 sweeps at 10 Hz, one revolution), the same
KISS-ICP with odometry prior, and the same evaluation (`mapping/slam.py`).

| Sensor (sim) | Mount | Range σ | Beams/s (sensor) | Simulated fraction | Returns per sweep (sim) | Map pts (0.1 m voxels) | ATE RMSE | Height MAE / p95 | Within 10 cm | Landing IoU | SLAM sites valid on truth | Runtime |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| legacy | down fan | 2 cm | 230k | 1 | 22,872 | 441k | 4.6 cm | 4.9 / 14.2 cm | 91% | 0.980 | 10/16 | 59 s |
| legacy | down fan | 2 cm | 230k | 0.25 | 5,716 | 252k | 8.2 cm | 4.7 / 14.8 cm | 91% | 0.976 | 10/16 | 20 s |
| Ouster OS1-128 | tilt 90° | 3 cm | 1,311k | 0.25 | 13,548 | 439k | 9.4 cm | 7.8 / 17.3 cm | 82% | 0.977 | 10/16 | 27 s |
| Hesai XT32 | tilt 90° | 1 cm | 640k | 1 | 26,535 | 498k | 23.2 cm | 4.5 / 16.3 cm | 91% | 0.974 | 9/16 | 53 s |
| Ouster OS1-128 | tilt 90° | 3 cm | 1,311k | 0.5 | 27,093 | 588k | 8.3 cm | 7.9 / 18.0 cm | 80% | 0.975 | 11/16 | 49 s |
| Ouster OS1-128 | tilt 90° | 3 cm | 1,311k | 1 | 54,184 | 762k | 6.6 cm | 8.7 / 19.5 cm | 76% | 0.973 | 12/16 | 118 s |
| Ouster OS1-128 | tilt 90° | 2 cm | 1,311k | 0.5 | 27,093 | 556k | 8.4 cm | 5.4 / 15.7 cm | 90% | 0.978 | 11/16 | 61 s |
| Ouster OS1-128 | tilt 90° | 2 cm | 1,311k | 1 | 54,184 | 691k | 6.0 cm | 5.6 / 15.4 cm | 89% | 0.977 | 10/16 | 76 s |
| Hesai XT32 | tilt 45° | 1 cm | 640k | 1 | 24,038 | 539k | 8.1 cm | 4.3 / 14.6 cm | 92% | 0.968 | 8/16 | 54 s |
| Ouster OS1-128 | tilt 45° | 3 cm | 1,311k | 0.5 | 24,219 | 625k | 7.1 cm | 6.9 / 17.2 cm | 88% | 0.980 | 12/16 | 60 s |

Rows marked 90° were run before the presets moved to 45°. To reproduce them, pass `--tilt 90`; to reproduce the
σ = 2 cm control rows, pass `--sigma 0.02`. Seed 7, pile B2, 120 sweeps. One run per row (single seed, single pile), so differences of about 1 cm are within
run-to-run noise. Raw rows: `--out` JSON lines. Every row registered with KISS-ICP; none fell back.

What the numbers say:

1. **Density helps pose accuracy within a sensor.** OS1 at a quarter, half and full beam grid: ATE 9.4 → 8.3 →
   6.6 cm. The legacy fan at a quarter vs full grid: 8.2 → 4.6 cm.
2. **Height error is set by range noise, not by density.** The rasteriser keeps the **maximum** z per 0.5 m cell
   (`sim.world.heightmap_from_points`), so more points from a noisier sensor raise the cell maxima. OS1 at
   σ = 3 cm gets worse with density (MAE 7.8 → 8.7 cm). The same OS1 with σ = 2 cm stays at 5.4–5.6 cm, and the
   XT32 (σ = 1 cm) has the best MAE of the table (4.3–4.5 cm).
3. **The mount matters more than the point rate.**
   - With a 90° across-track plane, the XT32's narrow 31° fan leaves the along-track direction weakly
     constrained, and ATE reaches **23 cm**. Pitching it 45° brings that down to 8.1 cm.
   - The OS1 also gains from 45° (ATE 8.3 → 7.1 cm, MAE 7.9 → 6.9 cm, 80% → 88% of cells within 10 cm).
   - So the presets now mount both sensors at 45°.
   - The legacy fan is an idealised sensor that points all of its beams down (−18…−80°). It wastes no beams on
     the sky, which is why it still has the best ATE.
4. **Landing cells saturate at 0.5 m.** The feasible-cell IoU is 0.97–0.98 in every row, and 8–12 of 16 SLAM
   sites are valid on the true terrain. The 0.5 m heightmap, not the sensor, limits landing-cell quality. Denser
   LiDAR would let us move to a 0.25 m grid, which matters for the 3.5 m heavy-drone patch, but we have not
   tested that. (The report's 16/16 is for 300 sweeps; at the mission's 120 sweeps the legacy sensor also gets
   10/16.)
5. **Runtime.** A dense sensor simulated on a beam subset is cheaper than the legacy fan: OS1 at 0.25 takes 27 s
   against 59 s, because KISS-ICP downsamples every frame anyway. For the mission's `--slam`, `sim_fraction` 0.25
   to 0.5 is the right setting. The effective sensor rate stays 1.31 M beams/s, and the map reported in
   `metrics.json` is the simulated subset.

Not modelled: multiple returns, motion distortion inside a sweep (deskew off), reflectivity-dependent range,
dust in the downwash, and the OS1's real beam spacing (uniform here).

## 4. Isaac

**`isaac/lidar.py`**

- `PROFILES` + `add_lidar_args(ap)` give `--lidar-profile small|heavy` (default **heavy**) and `--lidar-budget-ms`.
  `profile(name, budget_ms, rays, max_range)` and `make_lidar(kind, prof, cast=…, grid=…)` build the LiDAR.
  An explicit `--rays N` pins the count and turns off auto-sizing.
- **small** is the original demo: 96 PhysX rays per drone per frame, 16 rings, 0.15 m accumulator voxel, 150k
  points per pile.
- **heavy**: 32 rings over −2…−88°, 60 m range, 0.10 m voxel, 400k points per pile. The ray count is sized to a
  **frame budget of 4 ms per surveying drone** (12 ms of a 33 ms frame for 3 drones):
  - The per-ray cost starts at an **assumed 20 µs** (`COST_US_ASSUMED`: a Python call, float conversions and
    one PhysX BVH query). This is an estimate; nobody has timed it on the L40S.
  - Every sweep is then **timed**, and the cost is tracked as an EMA. The next sweep casts `budget / cost` rays,
    clamped to 96–4096. At the assumed cost that is 200 rays per frame, already 2× the small profile. If PhysX
    turns out to be 5 µs per ray it settles at 800.
  - `lidar.stats()` reports the measured cost. Please log it once on the GPU, then replace the assumption with
    the measured value.
  - Batching: the directions come from one numpy call and go to PhysX as plain tuples (the verified fast form).
    The 6.0.1 Python scene-query API has no batched raycast. Isaac Lab's Warp `RayCaster` is GPU-batched, but
    it needs a single static mesh, and our rubble is many box colliders. It stays a later option (unverified).
- **Real density means RTX LiDAR.** Even 800 rays × 30 fps is about 2.4 × 10⁴ rays/s per drone, roughly 50×
  below an OS1-128. With `--rtx-lidar`, the heavy profile therefore tries the dense RTX configs in order:
  `OS1_REV7_128ch10hz1024res`, `OS1_REV6_128ch10hz1024res`, `Hesai_XT32_SD10` / `HESAI_XT32_SD10`, then
  `Example_Rotary` (`RtxLidar(..., configs=PROFILES["heavy"]["rtx_configs"], mount_z=…)`). The config names
  come from the 4.x/5.x sensor library. Every call is guarded, and the code falls back to PhysX if all of them
  fail. RTX LiDAR has **never been run by the team** on 6.0.1 (`docs/ISAAC_VERIFIED_APIS.md` §4).
- `PointAccumulator` is now vectorised: packed int64 voxel keys and sorted `searchsorted` membership replace
  the old per-point Python loop. A 10⁵-point RTX frame costs milliseconds, and a test checks it against a naive
  voxel set.
- `python -m isaac.lidar --lidar-profile heavy` runs a numpy dry-run benchmark: 192 rays per frame at 10 ms per
  frame for the heightmap ray-march. The PhysX path is a different cost.

**`isaac/drone_viz.py`**

- `DroneViz(..., model="heavy", spec=None)` is the default. It builds a true-scale FlyCart / T50-class
  coaxial X8 from `sim.platform` (`heavy_parts(spec)`):
  - airframe: 1.0 × 0.7 × 0.4 m fuselage with side battery packs, 4 arms on the 2.2 m wheelbase, a coaxial
    motor pair and two 1.375 m rotor discs per arm (counter-rotating, at a slower visual spin), and 0.6 m
    landing gear (two 1.6 m skids at a 1.4 m track);
  - payload: a nose gimbal with the LiDAR puck (drawn with its axis along +x; the sim mounts it pitched 45°) and the thermal camera, the
    0.6 × 0.5 m radar panel under the belly facing down, the speaker horn under the tail, and the 8-mic ring
    (1.0 m).
- Extent is 2.93 × 2.93 m with the X rotated 45°, against the spec's 2.80 × 3.085 m, and 1.3 m tall. Skid
  bottom sits at the mission pose z − 0.15, as before.
- `scale` is ignored for heavy, because the dimensions come from the spec in metres. `model="small"` or
  `--drone-asset` keep the old quad and the asset path.
- `heavy_parts` / `model_extent` are pure Python. pxr is not installed in `.venv`, so the USD build itself is
  **untested offline**, but it only uses the `_shape` helper that the small model already uses on the GPU, plus
  `UsdGeom.Cone`. Run `--selftest` on the GPU to confirm.
- `--dry-run` uses `NullDrone`, which is unchanged.

## 5. Requests (files owned by others)

These files belong to other builders. Each change below is backwards-compatible: with no platform given, every
default reproduces today's behaviour bit for bit. That was checked for `landing.py` (legacy output identical on
seed-7 pile B2) and `run_mission.py` (all 6 `tests/test_isaac_dryrun.py` pass with the patched file on top of the
current working tree).

### R-PLAT-1: battery, endurance and speeds from the spec (`sim/coordinator.py`, mission builder)

`KinematicFlight(grid, platform="heavy_sar")` takes speeds, terrain clearance and battery rates from
`PlatformSpec.flight_kwargs()`:

| | Flying | Perched |
|---|---|---|
| heavy_sar | 0.0678 %/s (24.6 min) | 0.00175 %/s (rotors off) |
| legacy (today) | 0.16 %/s | 0.05 %/s, 0.06 %/s scanning |

```diff
--- a/sim/coordinator.py
+++ b/sim/coordinator.py
@@ -58,7 +58,16 @@
 class KinematicFlight(FlightBackend):
     """Straight-line waypoint flight at cruise altitude with a heightmap collision check."""
 
-    def __init__(self, grid, starts=STARTS, cruise_alt=8.0, speed=4.0, climb=2.0, descend=1.2):
+    def __init__(self, grid, starts=STARTS, cruise_alt=8.0, speed=4.0, climb=2.0, descend=1.2,
+                 fly_rate=0.0016, idle_rate=0.0005, scan_rate=0.0006, terrain_clear=3.0, platform=None):
+        if platform is not None:          # sim.platform preset name or PlatformSpec (docs/PLATFORM.md)
+            from .platform import get_platform
+            kw = (get_platform(platform) if isinstance(platform, str) else platform).flight_kwargs()
+            cruise_alt, speed, climb, descend = kw["cruise_alt"], kw["speed"], kw["climb"], kw["descend"]
+            fly_rate, idle_rate, terrain_clear = kw["fly_rate"], kw["idle_rate"], kw["terrain_clear"]
+            scan_rate = idle_rate         # perched scan: rotors off, radar + compute on
+        self.fly_rate, self.idle_rate, self.scan_rate = fly_rate, idle_rate, scan_rate
+        self.terrain_clear = terrain_clear
         self.grid = grid
         self.starts = dict(starts)
         self.cruise, self.speed, self.climb, self.descend = cruise_alt, speed, climb, descend
@@ -74,8 +83,8 @@
 
     def plan(self, d, dest, land=True):
         p = d.pos
-        alt = max(self.cruise + self.stagger[d.id], self._terrain_max(p, dest) + 3.0)
-        if alt > 25.0:
+        alt = max(self.cruise + self.stagger[d.id], self._terrain_max(p, dest) + self.terrain_clear)
+        if alt > max(25.0, self.cruise + 15.0):
             return None
         route = []
         if p[2] < alt - 0.5:
@@ -99,7 +108,7 @@
         if route is None:
             return None
         t = self._route_time(d.pos, route)
-        if d.battery - (0.0016 * t + 0.0006 * SCAN_DWELL_S) < 0.15:
+        if d.battery - (self.fly_rate * t + self.scan_rate * SCAN_DWELL_S) < 0.15:
             return None
         return t
 
@@ -142,7 +151,7 @@
         for d in self.drones.values():
             if d.state == "scanning":
                 d.dwell_left -= dt
-                d.battery -= 0.0006 * dt
+                d.battery -= self.scan_rate * dt
                 if d.dwell_left <= 0:
                     d.state = "landed"
                     ev = {"type": "scan_complete", "drone_id": d.id, "site": d.job["site"], "pose": self._pose(d)}
@@ -150,10 +159,10 @@
                     events.append(ev)
                 continue
             if not d.route:
-                d.battery -= (0.0005 if d.state in ("landed", "docked") else 0.0016) * dt
+                d.battery -= (self.idle_rate if d.state in ("landed", "docked") else self.fly_rate) * dt
                 continue
             budget = dt
-            d.battery -= 0.0016 * dt
+            d.battery -= self.fly_rate * dt
             while d.route and budget > 1e-9:
                 w = d.route[0]
                 delta = w - d.pos
```

`sim/grid_mission.py` (currently has someone's uncommitted edits, so this is a hunk list, not a diff):

- Add a `platform=None` argument (and a `--platform heavy_sar|industrial_sar|legacy_small` CLI flag, default
  `legacy_small` until the demo is re-baselined), passed as
  `KinematicFlight(self.grid, starts=pads, platform=platform)`.
- `FLY_RATE = 0.0016` → read `self.flight.fly_rate`.
- `PILE_BATTERY = 0.16` → `0.16 * self.flight.fly_rate / 0.0016`. A pile is mostly flight and hover time, so this
  gives about 6.8% per pile for heavy_sar.
- Keep `RESERVE = 0.25` and `SWAP_S = 60` (a FlyCart pack swap is two 1984 Wh packs, so 60 s is optimistic;
  120 s is an *estimate*).
- `--slam`: `run_pile_slam(..., cfg=LidarConfig.from_spec(get_platform(platform), sim_fraction=0.25))`. 120
  sweeps of the OS1 at a quarter of the beams take 27 s against 59 s for the legacy fan (table in §3).
- `detect_landing_sites(..., limits=get_platform(platform).landing_limits())` (needs R-PLAT-2).

### R-PLAT-2: landing footprint (`sim/landing.py`, mission builder)

`detect_landing_sites(grid, centre, limits=spec.landing_limits())` changes four things:

- a k × k plane fit (k = 7 → 3.5 m patch for heavy_sar);
- the spec's slope, roughness, step and clearance limits;
- a step radius matching the gear;
- the heavy drone ranks firm off-pile cells first, with fewer perch slots spaced 4 m apart.

With `limits=None` the output is identical to today's. On seed-7 pile B2, heavy_sar gives 16 candidates, all off
the pile, with 6 rejected (5 roughness, 1 step). `mapping/slam.py` (`_feasible_mask`, `_valid_on`) keeps the legacy
limits until this lands.

```diff
--- a/sim/landing.py
+++ b/sim/landing.py
@@ -36,35 +36,39 @@
     return float((X * w).sum() / w.sum()), float((Y * w).sum() / w.sum())
 
 
-def patch_fields(grid: Grid):
-    """Vectorised 3x3 plane fit: returns slope_deg, roughness, plane gradients (a, b)."""
+def patch_fields(grid: Grid, k: int = 3):
+    """Vectorised k x k plane fit (k odd: 3 = legacy 1.5 m, 7 = heavy_sar 3.5 m patch, see sim/platform.py):
+    returns slope_deg, roughness, plane gradients (a, b), mean."""
     z = grid.z.astype(np.float64)
     h, w = z.shape
     res = grid.res
-    zp = np.pad(z, 1, mode="edge")
+    r = k // 2
+    offs = range(-r, r + 1)
+    zp = np.pad(z, r, mode="edge")
     a = np.zeros_like(z)
     b = np.zeros_like(z)
     mean = np.zeros_like(z)
     shifts = {}
-    for dy in (-1, 0, 1):
-        for dx in (-1, 0, 1):
-            s = zp[1 + dy:1 + dy + h, 1 + dx:1 + dx + w]
+    for dy in offs:
+        for dx in offs:
+            s = zp[r + dy:r + dy + h, r + dx:r + dx + w]
             shifts[(dy, dx)] = s
             a += dx * res * s
             b += dy * res * s
             mean += s
-    a /= 6 * res ** 2
-    b /= 6 * res ** 2
-    mean /= 9
+    sxx = k * sum(d * d for d in offs) * res ** 2      # = 6 res^2 for k = 3
+    a /= sxx
+    b /= sxx
+    mean /= k * k
     ss = np.zeros_like(z)
     for (dy, dx), s in shifts.items():
         ss += (s - (mean + a * dx * res + b * dy * res)) ** 2
-    rough = np.sqrt(ss / 9)
+    rough = np.sqrt(ss / (k * k))
     slope = np.degrees(np.arctan(np.hypot(a, b)))
     return slope, rough, a, b, mean
 
 
-def _clearance_step_approach(zpad, i, j, pad, a, b, zc, res):
+def _clearance_step_approach(zpad, i, j, pad, a, b, zc, res, step_r=LIM_CLEAR):
     """Window analyses for one patch centred at cell (j, i) (indices in unpadded grid)."""
     # clearance / step window: +-6 cells (3 m)
     R = 6
@@ -76,7 +80,7 @@
     obstacle = rel > OBSTACLE_H
     clearance = float(D[obstacle].min()) if obstacle.any() else 3.0
     clearance = min(clearance, 3.0)
-    inner = D <= 0.9
+    inner = D <= step_r
     step = float(np.abs(rel[inner]).max())
     # approach cone: window +-12 cells (6 m)
     R2 = 12
@@ -91,10 +95,16 @@
 
 
 def detect_landing_sites(grid: Grid, centre=None, max_sites: int = MAX_SITES,
-                         min_spacing: float = MIN_SPACING, search_radius: float = SEARCH_RADIUS):
-    """Return landing-site dicts (INTERFACES.md fields) with status candidate/rejected."""
+                         min_spacing: float = MIN_SPACING, search_radius: float = SEARCH_RADIUS, limits=None):
+    """Return landing-site dicts (INTERFACES.md fields) with status candidate/rejected.
+
+    ``limits`` = ``sim.platform.PlatformSpec.landing_limits()`` (None = legacy constants above)."""
+    L = limits or {}
+    lim_flat, lim_perch = L.get("slope_deg", LIM_SLOPE), L.get("perch_slope_deg", PERCH_SLOPE)
+    lim_rough, lim_step, lim_clear = L.get("rough_m", LIM_ROUGH), L.get("step_m", LIM_STEP), L.get("clear_m", LIM_CLEAR)
+    edge_first = str(L.get("prefer", "")).startswith("street")    # heavy drone: firm ground at the pile edge
     cx, cy = centre if centre is not None else mound_centre(grid)
-    slope, rough, a, b, mean = patch_fields(grid)
+    slope, rough, a, b, mean = patch_fields(grid, L.get("patch_cells", 3))
     X, Y = grid.mesh()
     rc = np.hypot(X - cx, Y - cy)
     pad = 14
@@ -103,10 +113,11 @@
     rows = []
     ground_ref = float(np.median(grid.z[rc > 20.0])) if (rc > 20.0).any() else 0.0
     perch = mean > ground_ref + 0.6               # on the rubble: the radar needs to be close (penetration limit)
-    lim_slope = np.where(perch, PERCH_SLOPE, LIM_SLOPE)
-    sel = (rc <= search_radius) & (slope <= lim_slope + 6) & (rough <= LIM_ROUGH + 0.05)
+    lim_slope = np.where(perch, lim_perch, lim_flat)
+    sel = (rc <= search_radius) & (slope <= lim_slope + 6) & (rough <= lim_rough + 0.05)
     for j, i in zip(*np.nonzero(sel)):
-        clearance, step, app = _clearance_step_approach(zpad, i, j, pad, a[j, i], b[j, i], mean[j, i], grid.res)
+        clearance, step, app = _clearance_step_approach(zpad, i, j, pad, a[j, i], b[j, i], mean[j, i], grid.res,
+                                                        L.get("step_r_m", LIM_CLEAR))
         s_slope = max(0.0, 1 - slope[j, i] / 10.0)
         s_rough = max(0.0, 1 - rough[j, i] / 0.06)
         s_clear = min(1.0, clearance / 3.0)
@@ -114,11 +125,11 @@
         reason = None
         if slope[j, i] > lim_slope[j, i]:
             reason = "slope"
-        elif rough[j, i] > LIM_ROUGH:
+        elif rough[j, i] > lim_rough:
             reason = "roughness"
-        elif step > LIM_STEP:
+        elif step > lim_step:
             reason = "step"
-        elif clearance < LIM_CLEAR:
+        elif clearance < lim_clear:
             reason = "clearance"
         elif not app:
             reason = "approach"
@@ -126,8 +137,8 @@
                          slope=float(slope[j, i]), rough=float(rough[j, i]), clearance=clearance,
                          score=float(score), reason=reason, r=float(rc[j, i]), perch=bool(perch[j, i])))
 
-    def rank(row):  # prefer good patches, mildly prefer the ones near the rubble
-        return row["score"] - 0.012 * row["r"]
+    def rank(row):  # prefer good patches, mildly prefer the ones near the rubble (heavy: off the pile first)
+        return row["score"] - 0.012 * row["r"] - (0.15 if edge_first and row["perch"] else 0.0)
 
     ok = sorted([r for r in rows if r["reason"] is None], key=rank, reverse=True)
     bad = sorted([r for r in rows if r["reason"] is not None and r["r"] < 12.0],
@@ -145,7 +156,7 @@
             if all(np.hypot(row["x"] - c["x"], row["y"] - c["y"]) >= spacing for c in chosen):
                 chosen.append(row)
 
-    take(on_rubble, max_sites // 2, 2.5)
+    take(on_rubble, max_sites // (4 if edge_first else 2), 2.5 if not edge_first else 4.0)
     take(ok, max_sites, min_spacing)
     rejected = []
     for row in bad:
```

### R-PLAT-3: Isaac flags (`isaac/run_mission.py`, Isaac owner)

This wires `--lidar-profile small|heavy` (default heavy), `--lidar-budget-ms`, `--drone-model heavy|small`
(default heavy) and the dense RTX configs.

- `--rays` and `--lidar-range` now default to the profile's values. Pass `--rays 96` for the old fixed count.
- The profile is resolved in `parse_args`, so `isaac/selftest.py` keeps reading `a.rays` / `a.lidar_range`.
- The diff is against the current working tree, which already has another agent's uncommitted edits.

Without this diff, `DroneViz` already defaults to the heavy model (also in `selftest.py`). Only the LiDAR
profile needs the flag.

```diff
--- a/isaac/run_mission.py
+++ b/isaac/run_mission.py
@@ -64,12 +64,17 @@
     ap.add_argument("--camera", choices=["director", "free"], default="director")
     ap.add_argument("--drone-asset", nargs="?", const="auto", default=None,
                     help="reference Isaac's quadcopter USD (optionally a path/URL) instead of primitives")
-    ap.add_argument("--drone-scale", type=float, default=1.6)
+    ap.add_argument("--drone-scale", type=float, default=1.6, help="small model only (heavy is true scale)")
+    ap.add_argument("--drone-model", choices=["heavy", "small"], default="heavy",
+                    help="heavy: FlyCart/T50-class coaxial X8 from sim.platform (docs/PLATFORM.md); small: old quad")
     ap.add_argument("--rtx-lidar", action="store_true", help="try RTX LiDAR (falls back to PhysX raycasts)")
     ap.add_argument("--overlay", choices=["auto", "debug", "usd"], default="auto",
                     help="debug = debug_draw lines/points; usd = prims only (safest for Movie Capture)")
-    ap.add_argument("--rays", type=int, default=96, help="LiDAR rays per surveying drone per frame")
-    ap.add_argument("--lidar-range", type=float, default=40.0)
+    ap.add_argument("--rays", type=int, default=None, help="pin LiDAR rays per surveying drone per frame "
+                    "(default: the --lidar-profile's: small 96, heavy auto-sized to --lidar-budget-ms)")
+    ap.add_argument("--lidar-range", type=float, default=None, help="default: profile's (small 40, heavy 60)")
+    from isaac.lidar import add_lidar_args
+    add_lidar_args(ap)
     ap.add_argument("--render-fps", type=float, default=30.0, help="virtual frame rate for --dry-run (--record: --rec-fps)")
     ap.add_argument("--no-keep-open", action="store_true", help="close the GUI when the mission ends")
     ap.add_argument("--quiet", action="store_true")
@@ -125,6 +130,9 @@
             setattr(a, k, photo if a.photoreal else basic)
     live = int(getattr(a, "livestream", 0) or 0)
     a.live = live > 0
+    from isaac.lidar import profile as lidar_profile     # resolve the LiDAR profile once (selftest reads a.rays)
+    a.lidar_prof = lidar_profile(a.lidar_profile, a.lidar_budget_ms, a.rays, a.lidar_range)
+    a.rays, a.lidar_range = a.lidar_prof["n_rays"], a.lidar_prof["max_range"]
     a.keep_open = not (a.dry_run or a.no_keep_open or a.record or a.selftest or (a.headless and not a.live))
     return a
 
@@ -269,7 +277,8 @@
 
     from isaac.camera import DirectorCamera, focus_from_state
     from isaac.drone_viz import DroneViz, NullDrone
-    from isaac.lidar import HeightmapLidar, PhysxLidar, PointAccumulator, RtxLidar
+    from isaac.lidar import HeightmapLidar, PointAccumulator, RtxLidar, make_lidar
+    lprof = a.lidar_prof
     from isaac.overlay import Overlay, make_backend
 
     if a.out:
@@ -284,7 +293,7 @@
 
     stage, dd, rtx, dust = None, None, None, None
     if a.dry_run:
-        lidar = HeightmapLidar(hm, a.lidar_range, a.rays)
+        lidar = make_lidar("heightmap", lprof, grid=hm)
     else:
         from isaac import compat, scene
         stage = rt.stage()
@@ -301,12 +310,13 @@
             dd = compat.debug_draw()
             if dd is None:
                 say("debug_draw unavailable; overlay uses USD prims")
-        lidar = PhysxLidar(compat.physx_raycaster(), a.lidar_range, a.rays)
+        lidar = make_lidar("physx", lprof, cast=compat.physx_raycaster())
     ids = [d["id"] for d in as_dict(mission.build_state()).get("drones", [])] or ["d1", "d2", "d3"]
     if a.dry_run:
         drones = {i: NullDrone(None, f"/World/Drones/{i}") for i in ids}
     else:
-        drones = {i: DroneViz(stage, f"/World/Drones/{i}", did=i, scale=a.drone_scale, asset=a.drone_asset)
+        drones = {i: DroneViz(stage, f"/World/Drones/{i}", did=i, scale=a.drone_scale, asset=a.drone_asset,
+                              model=a.drone_model)
                   for i in ids}
     from isaac.site import base_xy, site_name
     overlay = Overlay(make_backend("null" if a.dry_run else a.overlay, stage, dd), surface, pile_xy,
@@ -331,10 +341,11 @@
         else:
             say(f"PhysX raycast self-test ok: hit z={hit[2]:.2f} at pile {p0.name}")
         if a.rtx_lidar:
-            rtx = RtxLidar({i: d.path for i, d in drones.items()})
+            rtx = RtxLidar({i: d.path for i, d in drones.items()}, configs=lprof["rtx_configs"],
+                           mount_z=0.35 if a.drone_model == "small" else 0.50)
             rtx = rtx if rtx.ok else None
 
-    accum = PointAccumulator()
+    accum = PointAccumulator.for_profile(lprof)
     cur = prev = as_dict(mission.build_state())
     steps, frame = 0, 0
     virtual = a.dry_run or bool(a.record)   # recording: fixed frame clock, so slow (path-traced) frames don't skip
```

Also: the LiDAR origin in the sweep call (`lidar.sweep((x, y, z + 0.35), …)`) should be `z + 0.50` for the heavy
model. The puck sits on the nose gimbal about 0.5 m above the skids.

### R-PLAT-4: dashboard point-cloud budget (`sim/grid_mission.py` writer + `dashboard/viewer3d.js` + INTERFACES.md, orchestrator)

`viewer3d.js` builds JS arrays for every point inside its 9 m crop and loops over the whole cloud twice per
`show()`. A dense heavy-drone map runs to millions of points. Our 120-sweep OS1 orbit of a single pile already
gives 0.76 M points after the 0.1 m voxel filter, and a site is 9 piles. Shipping that to the browser would cost
tens of MB per poll and stall phones.

The proposal keeps the current URL as the low level of detail (LOD), so nothing breaks.

```diff
--- a/sim/grid_mission.py   (_write_pointcloud)
+++ b/sim/grid_mission.py
+    LOD0_MAX = 300_000          # browser budget: ~3.6 MB float32
+    TILE_M = 16.0
     def _write_pointcloud(self, pts):
-        pts.astype("<f4").tofile(os.path.join(self.out_dir, "pointcloud.bin.tmp"))
-        os.replace(os.path.join(self.out_dir, "pointcloud.bin.tmp"), os.path.join(self.out_dir, "pointcloud.bin"))
-        self.pc_count = int(len(pts))
+        pts = np.asarray(pts, "<f4")
+        def put(name, arr):
+            p = os.path.join(self.out_dir, name)
+            os.makedirs(os.path.dirname(p), exist_ok=True)
+            arr.astype("<f4").tofile(p + ".tmp"); os.replace(p + ".tmp", p)
+        lod0 = pts
+        if len(pts) > self.LOD0_MAX:     # uniform voxel thinning first, then a seeded random cap
+            from mapping.slam import voxel_down_sample
+            lod0 = voxel_down_sample(pts, 0.15)
+            if len(lod0) > self.LOD0_MAX:
+                lod0 = lod0[np.random.default_rng(0).choice(len(lod0), self.LOD0_MAX, replace=False)]
+        put("pointcloud.bin", lod0)      # same URL as today = LOD0
+        self.pc_tiles = []
+        if len(lod0) < len(pts):          # full resolution, one file per 16 m tile
+            k = np.floor(pts[:, :2] / self.TILE_M).astype(int)
+            for kx, ky in np.unique(k, axis=0):
+                m = (k[:, 0] == kx) & (k[:, 1] == ky)
+                name = f"pc_tiles/{kx}_{ky}.bin"
+                put(name, pts[m])
+                self.pc_tiles.append({"url": name, "count": int(m.sum()),
+                                      "bounds": [kx * self.TILE_M, ky * self.TILE_M, (kx + 1) * self.TILE_M, (ky + 1) * self.TILE_M]})
+        self.pc_count, self.pc_full_count = int(len(lod0)), int(len(pts))
```

- **state.json** (INTERFACES.md, additive):
  `"pointcloud": {"url": "pointcloud.bin", "count": 300000, "lod": 0, "full_count": 2400000,
  "tiles": [{"url": "pc_tiles/-2_1.bin", "count": 81234, "bounds": [x0, y0, x1, y1]}]}`.
  `tiles` is empty when everything fits in LOD0.
- **viewer3d.js** (`show()`): when `tiles` is present, fetch only the tiles whose bounds intersect the 9 m crop
  (at most 4) and cache them by URL. Otherwise use LOD0 as today. Build `Float32Array`s directly instead of
  `push`.
- **Map view**: keep drawing LOD0.
- Isaac `PointAccumulator` caps (150k small / 400k heavy per pile) are separate. They bound the GPU overlay, not
  the browser.

### R-PLAT-5: radar receive gain (`sim/radar_dsp.py`, radar owner)

Add a platform term to the reflector amplitude so the bigger antenna shows up in SNR:
`A = 70 · 10^((σ_dB − L_rubble + G_rx)/20) / R²`, with `G_rx = sim.platform.radar_rx_advantage_db(spec)`. That is
+8.8 dB for heavy_sar, +4.9 dB for industrial_sar and 0 for legacy, so today's numbers are unchanged.

- The term applies to every reflector, clutter included. It therefore improves SNR against the thermal-noise
  floor only, not signal-to-clutter, which is the honest effect of aperture gain under an EIRP limit.
- The narrower beam's clutter rejection is not modelled.
- Then re-measure the reach table (`reach_path_m`). The expectation is about +0.6 m at depth, an *estimate*
  until rerun.

### R-PLAT-6: docs (orchestrator)

- TECHNICAL_REPORT §3.2 (flight and battery) and §4.1.1 (sensor) should point here once R-PLAT-1 and
  R-PLAT-2 land.
- INTERFACES.md: optional `drones[].platform` ("heavy_sar") and `mission.platform` (the `sim.platform.summary()`
  dict) for the dashboard.
