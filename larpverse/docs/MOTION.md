# Flight motion: how the heavy drones move

**Why.** The recording looked janky. `KinematicFlight` (sim/coordinator.py) flies straight lines at constant speed. Velocity changes instantly, corners are sharp and there is no attitude. Isaac then interpolated linearly between 1 s mission steps.

Rendered at 60 fps, a 64 s scripted flight showed **25 velocity jumps and a peak jerk of about 37,000 m/s³**. Over a 1500 s seed-7 mission it showed **471 jumps**.

`sim/flight_motion.py` replaces that with a multirotor motion model. `isaac/motion.py` samples it at the render time, every frame. With both in place there are **0 velocity jumps**, and acceleration, jerk, tilt and yaw rate stay inside the limits below.

| File | What it is |
|---|---|
| `sim/flight_motion.py` | `plan()`, `flight_time()`, `Trajectory.sample()`, `SmoothFlight` (drop-in `KinematicFlight`), export/load |
| `isaac/motion.py` | `Motion` (the adapter `PoseTrack` in run_mission.py already looks for), `MotionView`, `SpringFollow`, `OneEuro`, `SmoothCamera` |
| `tests/test_flight_motion.py` | 33 tests, about 4 s |
| `scripts/motion_preview.py` | showcase and mission dry runs, 60 fps pose log, before/after metrics, profile plot |
| `animation/motion-preview.html` | three.js player for the pose log (vendored three, no network) |
| `docs/img/motion_profiles.png` | speed, acceleration, tilt and altitude vs time |

## The model

**Primitives.** Every flight piece is a superposition of terms, and each term is C2: position, velocity and acceleration are continuous, and the acceleration is zero at both ends. So any sum of them is also C2, including across a re-plan.

- **Leg moves.** Each straight leg becomes a rest-to-rest 1-D move. Its velocity profile is a jerk-limited S-curve: ramp up, cruise, ramp down. Each ramp is a quintic smootherstep, so acceleration and jerk are continuous inside it.
- **Horizontal and vertical parts.** These are separate moves with their own limits: cruise speed, climb rate and descent rate.

**Corners.** The next leg starts while the previous one is still decelerating; this is the overlap (`beta`). The overlap rounds the corner, with a radius set by speed and acceleration.

The rounded path is checked against the heightmap for rotor-disc clearance: the skids must clear the highest cell under the 3.1 m disc by 1 m, outside the takeoff and landing columns. If it fails, the planner falls back in this order:
1. Use the stop-then-turn geometry the coordinator already checked (`beta = 0`).
2. Raise the cruise legs in 3 m steps, up to the site ceiling.

**Re-plans in mid-flight.** The new trajectory takes the drone's current velocity and acceleration exactly.
- The speed along the new leg is carried by that leg's S-curve, so a re-command toward roughly the same direction never stops the drone.
- A *brake* term fades the cross-track velocity, and separately fades the inherited acceleration, under the jerk limit.
- If the result breaks a limit, a ladder of longer brakes follows, then a full stop (still C2).

The recon agent re-commands legs higher as it maps new structure. Those legs now climb while still flying forward.

**Phases.**

| Phase | What happens |
|---|---|
| Takeoff | Sequential launch, 1.5 s apart. Rotor spool-up on the pad (2.5 s). Lift-off at 0.8 m/s to a 2 m hover (2 s), during which the drone yaws toward its first leg. Then the climb, blended into the first leg. |
| Cruise | S-curve at up to 10 m/s, with a steady nose-down attitude against drag (8° at 10 m/s). |
| Arrival | Deceleration into a stabilised hover. Radar jobs hold 1.5 s before the landing starts. |
| Perch landing | Stabilised hover over the patch (1.5 s), descent at 2 m/s, slow final at 0.5 m/s below 3 m, flare to 0.2 m/s below 0.5 m, touchdown (skids 5 cm up at 0.19 m/s), rotor spin-down (3 s). |
| Radar landing | The drone lands on a flat spot with the same perch-landing profile, timed by `PodSpec.lower_s` / `raise_s` (8 s to land, 8 s to lift off). The belly radar is fixed under the drone, so it does not swing; it rests on the rubble from touchdown. A command that arrives while the drone is landed first lifts it off. |

**Attitude from the dynamics.** Specific thrust is f = a + k·|v_air|·v_air + g·ẑ, with the tilt clamped to 25°. The drone therefore:
- leans into acceleration;
- holds a nose-down cruise attitude;
- flares when braking;
- leans into a gust.

The quaternion comes from the thrust direction plus the yaw (differential flatness). Roll and pitch use isaac/drone_viz's convention: R = Rz(yaw)·Ry(pitch)·Rx(roll), and +pitch means nose down.

**Yaw.** Yaw tracks the heading of the velocity 1.5 s ahead, so it turns before the corner. While stationary it looks up to 6 s ahead. A tracker limits the rate to 45°/s and the acceleration to 45°/s². Yaw is held on the ground and during the landing descent.

**Hover realism (rendering only).** The mission uses the nominal path.
- **Drift and bob:** deterministic coloured noise, made of four seeded sinusoids per axis (0.04 to 0.32 Hz, 1/f amplitude). RMS is 7 cm horizontal, 3.5 cm vertical and 0.35° in yaw.
- **Gusts:** site-wide, from the seed, 2 to 4.5 m/s for 2.5 to 5 s, every 16 to 40 s. Each gust gives a delayed downwind displacement plus the tilt into the wind.
- Both fade out within 2 m of the ground.

**Rotors.** Per-rotor RPM is hover RPM × sqrt(|f|/g), plus two small differentials:
- ±2% for yaw torque (counter-rotating pairs);
- up to ±3% across the arms from the jerk (pitch and roll changes).

The rotor order and spin signs match `drone_viz.ROTOR_ORDER`: arms 0 to 3 at 45 + 90k°, lower rotor then upper. On the ground the RPM spools up from 0 and spins down to 0.

**Mission timing.** `SmoothFlight` is `KinematicFlight` with the same commands, events and states.
- Arrival happens at the trajectory's exact end time. The mission sees it at the first step at or after that time.
- The dwell and landing timers carry the sub-step remainder.
- `eta()` returns the `flight_time()` of the trajectory the command would build.

So the times the mission schedules with equal the times the drones fly (tested).

## Parameters and sources

| Parameter | Value | Source |
|---|---|---|
| Cruise / climb / descent | 10 / 3 / 2 m/s | `sim/platform.py` heavy_sar (docs/PLATFORM.md). These are operating values below the FlyCart 30 maxima on https://www.dji.com/flycart-30/specs. |
| Horizontal acceleration | 2.5 m/s² | Task limit. PX4 `MPC_ACC_HOR` default is 3 m/s²; we use less for a 77 kg airframe. |
| Horizontal / vertical jerk | 2 / 2 m/s³ | Estimate: half of PX4 `MPC_JERK_AUTO` (default 4 m/s³), for a heavy frame. |
| Vertical acceleration | 1.5 m/s² | Estimate, below PX4 `MPC_ACC_UP_MAX` 4 / `MPC_ACC_DOWN_MAX` 3 (defaults). |
| Tilt limit | 25° | Task limit. PX4 `MPC_TILTMAX_AIR` default is 45°; heavy agricultural frames fly well below that. |
| Yaw rate | 45°/s | PX4 `MPC_YAWRAUTO_MAX` default 45°/s. |
| Final descent / flare | 0.5 m/s below 3 m, 0.2 m/s below 0.5 m | Task. PX4 `MPC_LAND_SPEED` default 0.7 m/s, with the slow-down band below `MPC_LAND_ALT2`. |
| Cruise drag tilt | 8° at 10 m/s | Estimate (k = tan 8° · g / 10²). |
| Hover RPM | ~1000 rpm | Estimate from momentum theory, T = C_T·ρ·n²·D⁴, C_T 0.08, 54-in props, 77 kg / 8. `drone_viz.HOVER_RPM` uses 1700 as an order of magnitude; see Requests. |
| Spool-up / spin-down | 2.5 / 3 s | Estimate. |
| Hover drift | 7 cm RMS | Estimate, consistent with RTK position hold. |
| Land / lift off (radar jobs) | 8 / 8 s | `PodSpec` in sim/platform.py. |

PX4 defaults are from the parameter reference: https://docs.px4.io/main/en/advanced_config/parameter_reference.html. They are the generic-airframe defaults, used as an upper bound for a heavy airframe. The one-euro filter is from Casiez, Roussel and Vogel, CHI 2012.

## Before / after

All numbers are measured as a viewer sees them. Positions are sampled at 60 fps and differentiated by finite differences.
- **Before:** `KinematicFlight` stepped at 1 s and interpolated linearly per frame (the old `interp_drones`).
- **After:** `SmoothFlight.motion_sample(t)`, including drift and gusts.
- A **velocity jump** is a frame whose implied acceleration exceeds 5 m/s², twice the limit.

| | Max horizontal accel | Max jerk | Velocity jumps | Max yaw rate |
|---|---|---|---|---|
| Showcase, 64 s, 3 drones, before | 600 m/s² | 37,461 m/s³ | **25** | 175°/s |
| Showcase, after | 2.68 m/s² | 2.4 m/s³ | **0** | 45°/s |
| Seed-7 mission, 1500 s, before | 920 m/s² | 55,209 m/s³ | **471** | 180°/s |
| Seed-7 mission, after | 2.78 m/s² | 6.0 m/s³ | **0** | 45°/s |

Over the whole 1500 s mission, the planned path (analytic, without the disturbances) stays inside every limit:

| Quantity | Max |
|---|---|
| Horizontal / vertical acceleration | 2.50 / 1.50 m/s² |
| Horizontal / vertical jerk | 2.00 / 2.00 m/s³ |
| Tilt | 18.2° |
| Yaw rate | 45°/s |

All 14 touchdowns land at 0.19 m/s. Any value above a limit in the "after" rows comes from the hover drift and gusts layered on top for realism.

**Story timing (needs the mission agent's attention).** Real accelerations, spool-up, stabilised hovers and slow landings make every sortie longer.

| Seed 7, stub acoustics | KinematicFlight | SmoothFlight |
|---|---|---|
| Recon and tasking (default), recon done | t 146 | t 221 |
| Recon and tasking (default), A2-R1 ping | t 451 | t 593 |
| Recon and tasking (default), piles visited by t 1500 | A2, B2, A3 | A2, B2, A3 (plus an A3-R1 ping) |
| No tasking (as `--dry-run`), A2-R1 ping | t 275 | t 342 |
| No tasking (as `--dry-run`), first dispatch | t 381 | t 468 |

The story is unchanged; it is about 25-30% slower.

The ISAAC_BRIEF gate "A2 ping before t 110" is not met by either flight model in these configurations. It needs re-deriving for whichever configuration the take uses.

## Offline check (no GPU)

```
.venv/bin/python scripts/motion_preview.py showcase       # media/motion/showcase.json + docs/img/motion_profiles.png + metrics
scripts/heavy.sh .venv/bin/python scripts/motion_preview.py mission --max-t 1500 \
    --metrics-json media/motion/mission_metrics.json --motion-json media/motion/mission_motion.json --out media/motion/mission.json
# view: http://localhost:8765/animation/motion-preview.html   (?log=../media/motion/mission.json for the mission log)
scripts/heavy.sh --browser uv run --offline --no-project --with playwright --with imageio-ffmpeg python \
    scripts/record_animation.py "http://localhost:8765/animation/motion-preview.html" media/motion/motion_preview.mp4 \
    --duration 60 --fps 30 --width 1280 --height 720
```

`.venv` has neither playwright nor imageio-ffmpeg. The `uv --offline` line uses the copies already in the uv cache, so nothing is downloaded.

The showcase script on the seed-7 grid, with the three drones:
- **d1:** takeoff, cruise, hover over B3, landing, 6 s on the rubble with its belly radar resting on it, lift-off.
- **d2:** perch landing at the A3 edge, spin-down, then a fresh takeoff.
- **d3:** recon-style legs with a mid-flight re-plan to a higher waypoint.

The director camera uses springs only and never cuts. Its timeline: wide shot at the pads (0 s), d1 (9 s), d3 re-plan (17.5 s), d1 landing on the rubble (25.5 s), d2 landing (31 s), d1 lift-off (46 s), d2 takeoff (51 s).

## Requests

### R1, mission agent: `sim/grid_mission.py` (and optionally `sim/coordinator.py`)

Fly with `SmoothFlight` by default. Write the trajectories next to the frames so the Isaac `--replay` mode can sample them.

```diff
@@ imports
 from .coordinator import KinematicFlight, Mission
+from .flight_motion import SmoothFlight
@@ class GridMission.__init__ signature
-                 tasking=None, recon_accept_s=RECON_ACCEPT_S, comms=True, uplink="oneweb", comms_llm=False):
+                 tasking=None, recon_accept_s=RECON_ACCEPT_S, comms=True, uplink="oneweb", comms_llm=False,
+                 flight_model="smooth"):
@@ backends
-        self.flight = flight if flight is not None else KinematicFlight(
-            self.grid, starts=pads, platform=None if self.platform_spec is None else self.platform_spec,
-            max_alt=pd.get("flight_ceiling_m"))
+        Flight, fkw = (SmoothFlight, {"seed": seed}) if flight_model == "smooth" else (KinematicFlight, {})
+        self.flight = flight if flight is not None else Flight(
+            self.grid, starts=pads, platform=None if self.platform_spec is None else self.platform_spec,
+            max_alt=pd.get("flight_ceiling_m"), **fkw)
@@ def run(self)
         while not self.done:
             self.step()
+        self._write_motion()
         return self
@@ def _write_state(self): after the frames path is appended
         if self.frames:
             paths.append(os.path.join(self.out_dir, "frames", f"state_{self.k:04d}.json"))
+            if self.k % 30 == 0:
+                self._write_motion()
+
+    def _write_motion(self):
+        if self.out_dir and hasattr(self.flight, "save_motion"):
+            self.flight.save_motion(os.path.join(self.out_dir, "motion.json"))   # isaac --replay samples it
@@ main(): CLI
+    ap.add_argument("--flight", choices=["smooth", "kinematic"], default="smooth",
+                    help="smooth: sim/flight_motion.py multirotor trajectories (docs/MOTION.md); kinematic: legacy")
 ...
-                    recon_accept_s=a.recon_accept_s)
+                    recon_accept_s=a.recon_accept_s, flight_model=a.flight)
```

Notes:
- **Imports:** `SmoothFlight` imports `sim.coordinator`, not `grid_mission`, so there is no import cycle.
- **Optional, coordinator.py:** `Mission.__init__` can use `SmoothFlight(self.grid)` instead of `KinematicFlight(self.grid)`.
- **Retest the story:** `tests/test_demo_story.py`, `tests/test_grid_mission.py` and the take gates. Timings move by +25-30% (table above).
- **Battery:** the per-second battery model is unchanged.

### R2, live single-take agent: `isaac/run_mission.py`

`PoseTrack` already uses `isaac.motion.Motion().pose_at()`. I implemented it to the dict contract `PoseTrack` reads: `xyz`, `quat`, `rotor_rpm`, `winch_len`, `ground_z`, `tilt`, `vel` and `yaw`, plus `sway_deg`, `pod`, `phase` and `state`.

It finds the live mission's `SmoothFlight` by itself. It raises `KeyError` for a `KinematicFlight` mission, so `PoseTrack` falls back to its Hermite path. Verified with `--dry-run --max-steps 60` (SmoothFlight patched in): "drone poses from isaac.motion.pose_at", with no fallback.

Remaining:

```diff
@@ render loop, flight-motion branch
-                    dr.set_state(raw["xyz"], raw["quat"], raw.get("rotor_rpm"), raw.get("winch_len"), rdt,
-                                 ground_z=raw.get("ground_z", pod_z.get(did)))
+                    dr.set_state(raw["xyz"], raw["quat"], raw.get("rotor_rpm"), raw.get("winch_len"), rdt,
+                                 sway_deg=raw.get("sway_deg"), ground_z=raw.get("ground_z", pod_z.get(did)))
@@ load_mission(): replay
-        return ReplayMission(a.replay, a.out, a.seed, a.start_pile, a.site)
+        rm = ReplayMission(a.replay, a.out, a.seed, a.start_pile, a.site)
+        mj = os.path.join(a.replay, "motion.json")
+        if os.path.exists(mj):
+            os.environ["LARPVERSE_MOTION_JSON"] = mj     # isaac.motion.Motion() picks it up (R1 writes it)
+        return rm
```

- **Director camera:** spring the look-at and eye targets, so a change of focus is a glide and never a jump:
  - set up once: `from isaac.motion import SmoothCamera; cam_s = SmoothCamera(eye0, look0, eye_halflife=0.9, look_halflife=0.45)`;
  - then every frame: `eye, look = cam_s.update(eye_target, look_target, rdt)`.

  `SpringFollow` / `OneEuro` are there for single vectors. All three are frame-rate independent, so they are stable under `--rec-speed` and warp.
- **Timing:** keep `t_render` advancing by `rdt * speed`, as you do now. The trajectories are continuous in t, so any time-lapse factor stays smooth.

### R3, Isaac assets agent: `isaac/drone_viz.py`

`set_state(pos, quat, rotor_rpm, winch_len, dt, sway_deg, ground_z)` matches what `isaac.motion` produces: the rotor order and spin signs are the same as `ROTOR_ORDER`, and the quaternion is (w, x, y, z).

- **`HOVER_RPM`:** please set `HOVER_RPM = 1000.0` (the momentum-theory estimate above) or import `MotionSpec.from_platform("heavy_sar").hover_rpm`. Our RPMs sit around 950 to 1100 in flight, which is above your `BLUR_RPM` of 450, so the blur disc behaves either way.
- **`sway_deg` (legacy argument):** (roll, pitch) with R = Rx(roll)·Ry(pitch), where +pitch would have swung the module toward -x. The belly radar is fixed under the drone and does not swing, so please pass zero or ignore it; if `_apply_hang` is kept, confirm it uses the same convention.

### R4, orchestrator

- **Pad spacing.** `drone_pads` are 2.6 m apart on grid and collapse, but the heavy drone spans 3.085 m, so the rotor discs overlap on the pads. The sequential launch (1.5 s apart) hides it in the air, but not on the ground. Please space pads ≥ 4.5 m apart (scenario data). That belongs to the mission or scenario owner.
- **Preview MP4.** `media/motion/motion_preview.mp4` (60 s, 1280×720, 10 MB) is gitignored. Attach it to the release if Rohan needs it.
- **Gates.** Re-derive the take gates in ISAAC_BRIEF §5 ("A2 ping before t 110") with the smooth flight model once R1 lands.
