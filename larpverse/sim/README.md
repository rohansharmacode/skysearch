# sim/ - Workstream B prototype (numpy only)

Run (from the project root, single process, low CPU):
`nice -n 19 .venv/bin/python -m sim.coordinator --seed 75 --out dashboard/data [--frames] [--realtime --speed 1]`
writes `pointcloud.bin` once and `state.json` every sim step (atomic rename); the default seed 75 is the demo.
`nice -n 19 .venv/bin/python -m sim.evaluate --n 30` -> markdown table + `eval/results.json` (adaptive vs fixed sites).
`.venv/bin/python -m pytest -q tests` -> localiser sanity tests. Schema: `contracts/INTERFACES.md`.
Grid demo (3x3 piles, `sim/scenario.py`): `nice -n 19 .venv/bin/python -m sim.grid_mission --seed 7 --out dashboard/data`
`[--start-pile B2] [--radar dsp] [--acoustic auto|real|stub] [--piles 3] [--decider none|gemini] [--realtime --speed 4] [--frames]`.
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
