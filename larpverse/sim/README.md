# sim/ - Workstream B prototype (numpy only)

Run (from the project root, single process, low CPU):
`nice -n 19 .venv/bin/python -m sim.coordinator --seed 75 --out dashboard/data [--frames] [--realtime --speed 1]`
writes `pointcloud.bin` once and `state.json` every sim step (atomic rename); the default seed 75 is the demo.
`nice -n 19 .venv/bin/python -m sim.evaluate --n 30` -> markdown table + `eval/results.json` (adaptive vs fixed sites).
`.venv/bin/python -m pytest -q tests` -> localiser sanity tests. Schema: `contracts/INTERFACES.md`.
Grid demo (autonomous, one command): `scripts/heavy.sh .venv/bin/python -m sim.grid_mission --seed 7 --out dashboard/data`
`[--start-pile B2] [--radar dsp|model] [--penetration legacy|literature] [--acoustic auto|real|stub] [--scans 24]`
`[--piles N (cap)] [--dispatch acoustic|nearest|raster] [--no-autonomous] [--site grid|weiguan|collapse|mesh:..]`
`[--platform legacy_small|heavy_sar|industrial_sar] [--slam] [--decider none|gemini] [--realtime --speed 4] [--frames]`.
Autonomous loop: after each pile the landed drones listen, piles are re-ranked, the fleet goes to the top lead,
revisits an open lead once (only if radar coverage of its top 2 m >= 50%), swaps batteries at the pads when any
drone would drop below a 25% reserve (60 s swap), and stops when the scan budget, the time limit or the leads run
out; it ends with a MISSION SUMMARY (pinged / open leads / not cleared). Evaluation: `python -m sim.evaluate_grid`
(docs/EVAL_GRID.md). Radar penetration physics: docs/RADAR_PENETRATION.md (default `legacy` = optimistic; see there).
Per pile it reuses `Mission` (survey -> land -> radar -> relocate), then listens (`acoustic_grid.listen`: the acoustic/
pipeline via `.venv-audio`, cached in `dashboard/data/acoustic_cache/`, else a synthetic stub), ranks piles and dispatches
to the top unvisited one. State schema: INTERFACES.md "v1.1 grid extensions".

Files
- `world.py` heightmap/pointcloud stand-in for the Isaac scene; hidden `targets` (truth, private).
- `landing.py` plane-fit landing patches from the heightmap only (slope, roughness, clearance, descent cone).
- `sensor_model.py` nominal quality / detection / ranging-noise / rubble-path assumptions shared by all.
- `radar_model.py` SYNTHETIC radar + acoustic observation generator; the only non-eval reader of `targets`.
- `acoustic_model.py` SYNTHETIC direction-of-arrival cones (pure function of an emitter position).
- `localise.py` grid Bayesian filter (ring likelihood + outlier floor), modes, moving-track belief, 95% region.
- `planner.py` initial 3 sites (Fisher-information geometry), expected-information-gain next scan, fixed baseline.
- `grid_world.py` whole-site heightmap/pointcloud + 30 m per-pile planning windows; hidden victims (truth, private).
- `acoustic_grid.py` bridge to acoustic/ (in-process, `.venv-audio` subprocess, cache) + SYNTHETIC stub (reads truth).
- `grid_mission.py` multi-pile coordinator (PileMission per pile, listen/rank/dispatch, optional decider) + state.json.
- `coordinator.py` kinematic flight + mission state machine + state.json writer. `evaluate.py` truth-based scoring.

What is synthetic: terrain, LiDAR cloud, all radar and acoustic returns (range noise, dropouts, multipath, a
moving distractor), landing disturbances, drone flight (straight-line kinematics). Real in the sense of the
final system: landing detection, fusion, planning and the observation-driven rescan logic.
Isaac swap: replace `World` (grid + `sample_pointcloud`), `KinematicFlight` (FlightBackend protocol) and
`ObservationGenerator` (drones emit the same observation dicts). Heightmap cell centre = origin + (i + 0.5) * res.

State fields added by this builder (additive; to be appended to contracts/INTERFACES.md by the orchestrator)
- `mission`: `autonomous`, `dispatch_policy`, `radar`, `radar_penetration` (legacy|literature), `radar_reach_beneath_m`,
  `scan_budget`, `scans_used`, `battery_reserve`, `battery_swaps`, `battery_swap` ({to_pile, ends, since} | null),
  `summary` (final text, null until done). `phase` adds `battery_swap`; `stage` adds `swap`.
- `site`: `kind`, `geo` ({lat, lon, bearing_deg, name}); `staging` from the scenario. `pings`: firm stationary candidates.
- `piles[]`: `polygon` (CCW [[x,y]]), `scans`, `radar_coverage`, `radar_coverage_shallow` (0-1, top 2 m), `radar_reach_m`,
  `nonhuman_listens`, `visits` implicit in `mission.visited` (a pile may appear twice = revisit).
- `radar_candidates[]`: `depth_m`, `depth_sigma`, `depth_source` (radar_estimate|nominal), `p_kind`
  (share_of_pile_belief), `support` / `support_needed` (strong agreeing ranges / 3), `outliers` (scan ids outvoted),
  `t_found`, `visit`, optional `vitals` + `dig_score` (hook on hold).
- `scans[]`: `max_reliable_depth_m` (rubble seen through 2 m beneath, from the scan's quality), `vitals_look` (60 s dwell).
- `acoustic_clips[]`: `vad_speech_ratio`, `association`, `association_conf`, `ambiguous_with`, `bearings`
  ([{drone_id, origin, bearing_deg, width_deg, snr_db}]), `notes`. `drones[].track`: last <= 200 [x, y, z, t].
- `decisions[].options[]`: `score`; actions add `revisit`; flags add `battery_swap`, `scan_budget_used`.
- `mission.acoustic_source` value `stub-from-truth` replaces `stub` (synthetic clips from hidden truth; WARNING logged).
