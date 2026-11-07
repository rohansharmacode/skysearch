# Larpverse interfaces (v1)

Agree on these before anyone writes integration code. Change them only by editing this file and telling the integration owner.

## Frames and units

- **Site frame**: right-handed, metres, seconds, **x = east, y = north, z = up**. Origin is the rubble centre at ground level (z = 0). Isaac Sim's default stage is also Z-up and metres, so no conversion is needed.
- Angles are in **degrees**. Bearings are measured **counter-clockwise from +x (east)**, the maths convention, not compass bearings.
- Time `t` is sim seconds since mission start.
- Grids (`heightmap`, `belief`): row-major with y outer; cell (i, j) has its centre at `origin + (i + 0.5, j + 0.5) * res`.
- Localisation runs on the **2D search layer** (x, y). `z_est` is filled in later from the LiDAR surface height minus an estimated depth, and is for display only.

## Ground-truth boundary

Only `sim/radar_model.py` (observation generator) and `sim/evaluate.py` may read the hidden target position (`world.targets`). In grid mode the same holds for `GridWorld.targets` and `Scenario.truth` (see v1.1 below). The planner, localiser and dashboard must never import or receive it. The evaluator writes its own file, `eval.json`, which is never served by the dashboard.

## Coordinator to drone: commands

```json
{ "drone_id": "d3", "type": "goto_land_scan", "destination": [6.5, -9.0, 0.0],
  "landing_site_id": "L7", "scan_duration": 20.0 }
```

`type` is one of `takeoff`, `goto`, `goto_land_scan`, `hold`, `return`.

## Drone to coordinator: radar observation

```json
{ "scan_id": "s4", "drone_id": "d3", "sensor_pose": [6.5, -9.0, 0.4, 135.0],
  "timestamp": 41.2, "range_candidates": [7.9, 11.4],
  "confidence": [0.81, 0.22], "quality": 0.9,
  "breathing_hz": 0.26, "doppler_moving": false }
```

- `range_candidates`: ranges in metres to periodic (respiration-like) returns. It can be empty: no detection, which is **not** evidence of absence.
- `confidence`: one value per candidate in [0, 1], how respiration-like the return is.
- `quality`: scan-level [0, 1]. It drops with platform motion (bad landing), short dwell or interference.
- `doppler_moving`: true when the return shows bulk motion (a moving person or a distractor), so it is tracked separately from stationary candidates.

## Drone to coordinator: acoustic observation

```json
{ "obs_id": "a2", "drone_id": "d1", "origin": [-8.0, 5.0], "timestamp": 47.0,
  "bearing_deg": 210.0, "width_deg": 30.0, "class": "knocking", "confidence": 0.7 }
```

`class` is one of `voice`, `shout`, `knocking`, `tapping` or `unknown`. `width_deg` is the full width of the direction-of-arrival cone.

## Coordinator to dashboard: `state.json`

The coordinator writes the full state to `dashboard/data/state.json` (atomic write: write `state.json.tmp`, then rename). The dashboard polls it every second. The point cloud is a separate binary file because it is large.

```jsonc
{
  "schema": "larpverse.state/v1",
  "t": 52.0,
  "phase": "acoustic",            // deploy | survey | landing | radar | relocate | rescan | acoustic | done
  "site": { "name": "Rubble site A", "bounds": [-25, -25, 25, 25] },
  "heightmap": { "origin": [-25, -25], "res": 0.5, "w": 100, "h": 100, "z": [ /* w*h floats, row-major, y outer */ ] },
  "pointcloud": { "url": "pointcloud.bin", "count": 120000 },   // float32 x,y,z triples
  "drones": [ { "id": "d1", "pose": [x, y, z, yaw_deg], "state": "scanning", "battery": 0.78 } ],
  "landing_sites": [ { "id": "L3", "xy": [x, y], "z": 0.1, "slope_deg": 2.1, "roughness": 0.02,
                       "clearance": 1.4, "score": 0.86, "status": "used" } ],   // candidate | used | rejected
  "scans": [ /* radar observations as above, plus "used_weight": 0.0-1.0 */ ],
  "radar_candidates": [ { "id": "R1", "xy": [x, y], "z_est": -1.2, "cov": [[sxx, sxy], [sxy, syy]],
                          "p_mass": 0.82, "breathing_hz": 0.26, "kind": "stationary",
                          "status": "candidate", "evidence": ["s1", "s2", "s4"] } ],
  "acoustic": [ /* acoustic observations as above */ ],
  "belief": { "origin": [-25, -25], "res": 0.5, "w": 100, "h": 100, "p": [ /* normalised */ ] },
  "next_scan": { "drone_id": "d3", "site_id": "L7", "reason": "separates two modes 3.1 m apart", "score": 0.74 },
  "log": [ { "t": 38.0, "msg": "Scan s3 quality 0.21; mirror ambiguity remains" } ]
}
```

`status` on a radar candidate is `candidate` or `insufficient_evidence`; it is never "confirmed". The dashboard labels these as **candidate search regions**.

## Acoustic agent to dashboard: `acoustic.json`

Written by `python -m acoustic.run` to `dashboard/data/acoustic.json` (atomic write); the same dict is returned by `acoustic.run.listen_and_rank(scenario, drone_positions, exclude_piles, seed)` without writing a file. Built from the landed drones' mic arrays (rotors off). Only the scene synthesis reads `scenario.truth`; the analysis and agent see measurements only.

```jsonc
{
  "schema": "larpverse.acoustic/v1",
  "t": 26.0,                                   // end of the listening window, s
  "seed": 3, "start_pile": "B2", "synthetic": true, "excluded_piles": [],
  "listening_drones": [ { "id": "d1", "xy": [0.6, 8.0] } ],
  "clips": [ {                                 // one separated source per detected pile
    "clip_id": "ac-C1", "drone_id": "d1",      // drone whose beam gave the best SNR
    "pile_id": "C1", "bearing_deg": 66.0, "width_deg": 30.0,   // from that drone; width = full width at half height
    "snr_db": 11.1, "vad_speech_ratio": 0.55, "transcript": "Help, can anyone hear me? ...",
    "sound_class": "voice",                    // voice | shout | knocking | dog | engine | debris | noise
    "class_scores": { "voice": 0.65, "shout": 0.0, "knocking": 0.03, "dog": 0.0, "engine": 0.0, "debris": 0.0, "noise": 0.32 },
    "human": 0.9, "urgency": 0.94, "reliability": 0.81,         // agent judgment, each 0-1
    "priority": 77,                            // 0-100 = 100*human*(0.35+0.65*urgency)*(0.45+0.55*reliability)
    "rationale": "voice 0.65; VAD 0.55; keywords: help, trapped; 3/3 drones agree ...",
    "by": "fallback",                          // llm | fallback
    "doa_agree": "3/3"
  } ],
  "piles": [ {                                 // all nine piles minus excluded, sorted by priority
    "pile_id": "C1", "xy": [21.4, 22.7], "priority": 77,
    "status": "radar_ping",                    // radar_ping | lead | no_signal_not_cleared | dismissed_nonhuman
    "top_clip": "ac-C1", "why": "Voice saying \"Help, can anyone hear me?...\"; human 0.90, ..."
  } ],
  "dispatch": { "pile_id": "C1", "reason": "Send the radar drone to C1: ..." },   // pile_id null if no human lead
  "agent": { "mode": "auto", "model": "claude-opus-5-5", "llm_calls": 0, "cache_hits": 0, "fallback": 6, "error": null },
  "log": [ "rejected B1 hypothesis: assigned=False ..." ], "timing_s": { "total": 35.0 }
}
```

- `radar_ping`: the top human lead, where the radar drone should go next. `lead`: other human-like sources (human >= 0.4).
- `dismissed_nonhuman`: a source was heard but judged non-human (dog, generator, debris). The pile is still **not cleared**.
- `no_signal_not_cleared`: nothing heard. Silence is not absence; priority stays at a small baseline (8).
- Priority is a triage order, not proof of a person.

## v1.1 grid extensions (`state.json` from `sim/grid_mission.py`)

The 3x3 rubble-grid mission writes the same `larpverse.state/v1` document (all v1 fields keep their meaning, so a v1 reader still works) plus the fields below. Site frame and pile names come from `sim/scenario.py` (columns A/B/C west to east, rows 1/2/3 north to south, staging area south).

Changes to v1 fields in grid mode:

- `site.bounds` is `SITE_BOUNDS` = `[-36, -54, 36, 36]`; `heightmap` covers the whole site (`Scenario.heightmap(0.5)`, 144 x 180 cells); `pointcloud.bin` is one site-wide cloud.
- `radar_candidates` accumulate across piles. Each has `"pile_id"` and an id prefixed with it (`"A2-R1"`). Only stationary candidates are kept for a finished pile: the firm one(s), or else the top `insufficient_evidence` mode.
- `scans` accumulate across piles (scan ids keep counting: `s1`, `s2`, ...), each with `"pile_id"`. `landing_sites` = used sites of finished piles + all sites of the active pile, ids prefixed (`"A3-L4"`), each with `"pile_id"`; `next_scan.site_id` uses the same prefix.
- `belief` is for the active pile only (30 m window, its own `origin`/`w`/`h`, plus `"pile_id"`). It is `null` between piles (listening, dispatch, return).
- `acoustic` holds the human-like clips (human >= 0.4) as v1 acoustic observations (`confidence` = reliability), plus `"pile_id"`.
- `phase` adds `dispatch` (fleet in transit to the next pile) and `return` (flying back to the pads). `acoustic` means landed and listening.
- `drones[].state`: `docked | flying | hovering | descending | scanning | landed | listening`. `listening` = landed, rotors off, during the listening window.

New fields:

```jsonc
{
  "mission": { "kind": "grid", "seed": 7, "start_pile": "A2", "pile_budget": 3, "priority_min": 20,
               "visited": ["A2", "A3"], "active_pile": "A3",          // null while returning / done
               "stage": "pile",                                        // pile | listen | return | done
               "acoustic_source": "real-venv",                         // real | real-venv | real-cached | stub | custom
               "listen_ends": null,                                    // t when the current listen ends
               "in_transit": { "t": 122.0, "from_pile": "A2", "to_pile": "A3", "reason": "..." } },  // or null
  "staging": { "xy": [0.0, -47.0], "pads": [[-2.6, -49.0], [0.0, -49.0], [2.6, -49.0]] },
  "piles": [ { "id": "A3", "xy": [-20.6, -22.9],
               "status": "scanning",   // unvisited | surveying | scanning | ping | lead | dismissed_nonhuman | no_signal_not_cleared
               "priority": 77,         // acoustic priority 0-100, null if never ranked (start pile, unheard)
               "radar_candidate_ids": ["A3-R1"],
               "why": "Voice saying \"Help! ...\"; human 0.90, urgency 0.92, reliability 0.82." } ],
  "acoustic_clips": [ { "id": "1-ac-A3", "listen": 1, "t": 122.0, "pile_id": "A3", "drone_id": "d3",
                        "origin": [-25.3, -8.8],                     // listening drone xy (cone apex)
                        "bearing_deg": 288.0, "width_deg": 20.0, "snr_db": 14.0,
                        "class": "voice", "transcript": "Help! Can anyone hear me? ...",
                        "class_scores": { "voice": 0.8, "...": 0.0 },
                        "scores": { "human": 0.9, "urgency": 0.92, "reliability": 0.82 },
                        "priority": 77, "rationale": "...", "by": "llm", "doa_agree": "3/3" } ],
  "acoustic_agent": { "mode": "auto", "llm_calls": 0, "fallback": 3, "error": null },   // agent block of the last listen
  "dispatch": [ { "t": 122.0, "from_pile": "A2", "to_pile": "A3", "reason": "Send the radar drones to A3: ..." },
                { "t": 467.0, "from_pile": "B2", "to_pile": null, "reason": "pile budget (3) used" } ]
}
```

Pile status rules (coordinator, never truth): the active pile is `surveying` then `scanning`; when its radar phase ends it becomes `ping` (a firm `candidate`), otherwise `lead` if its acoustic priority >= `priority_min`, else `no_signal_not_cleared`. Unvisited piles take the acoustic agent's status (`radar_ping` is shown as `lead`); a later silent listen never lowers an earlier lead's priority. Dispatch goes to the highest-priority unvisited pile at or above `priority_min` (default 20) until `pile_budget` (default 3) piles are visited; then the drones return to the pads. Silent piles stay **not cleared**.

Ground-truth boundary in grid mode: `sim/grid_world.py` holds the victims (`GridWorld.targets`, generator-only, like `World.targets`); `sim/radar_model.py` and the stub in `sim/acoustic_grid.py` are the only non-eval readers. `sim/grid_mission.py` passes the scenario to the acoustic pipeline (whose scene synthesis is its own generator) but never reads `scenario.truth` itself.

### Dispatch decisions (`decisions`, optional decider)

At every dispatch gate (after a listen) the coordinator builds scored options and, if `GridMission(decider=...)` is given (CLI `--decider gemini` loads `agents.gemini_decider.make_decider()` lazily), calls `decider(context, options) -> {"choice": "o2", "rationale": str, "by": "gemini" | "fallback"}`. An unknown id or an exception falls back to the argmax option (highest-priority reachable unvisited pile at or above `priority_min`, else `stop`). Without a decider the argmax is used and recorded with `"by": "argmax"`.

```jsonc
"decisions": [ { "t": 122.0, "gate": "dispatch", "chosen": "o1", "by": "argmax", "latency_ms": 0.0,
                 "rationale": "Send the radar drones to A3: ...",
                 "options": [ { "id": "o1", "action": "dispatch",        // dispatch | rescan_current | stop
                                "pile_id": "A3", "priority": 69, "travel_s": 31.4, "battery_after_min": 0.68,
                                "evidence": "voice \"Help! ...\" (human 0.86, priority 69, fallback)",
                                "flags": [] } ] } ]                       // below_threshold | nonhuman | silent | unreachable | no_firm_ping | pile_budget_used
```

`context` passed to the decider: `{t, current_pile, visited, pile_budget, budget_left, priority_min, pings: [{id, pile_id, status, p_mass}], piles: [{id, status, priority, why}], fleet: [{id, battery, state}]}`. `dispatch` options are only offered while pile budget is left; `rescan_current` (re-run the radar at the current pile) only when it has no firm ping and has not been rescanned.

## acoustic/v1 pile-association fields (BL-3, additive)

Written by `acoustic/run.py` (`ALGO_VERSION` in `acoustic/dsp.py`); every v1 field keeps its meaning.

```jsonc
{
  "algo": "assoc-v7",                          // localisation/association version (cache keys should include it)
  "clips": [ {
    "association": "confident",                // confident | ambiguous
    "association_conf": 0.97,                  // 0-1: bearing likelihood of this pile over all nine piles
                                               //  x (1 - 0.5 x rival-block share) x (1 - 0.5 x overlapped share)
    "ambiguous_with": ["B3"],                  // piles that explain the same bearings almost as well
    "reliability": 0.81,                       // agent reliability x (0.5 + 0.5 x association_conf); <= 0.45 if ambiguous
    "reliability_agent": 0.84,                 // the agent's own value before the association factor
    "doa_agree": "2/3",                        // drones whose bearing is within min(15 deg, gate) of the pile centre
    "heuristics": { "knock_periodicity": 0.62, "hum_harmonicity": 0.2 },
    "notes": [ "suppressed cross-talk from A3 (2 blocks)", "transcript dropped: dog clip with VAD 0.26" ]
  } ],
  "piles": [ { "association": "ambiguous",     // association of the top clip (absent for silent piles)
               "why": "Lead (ambiguous direction; could be B3): Voice saying ..." } ]
}
```

- `ambiguous` = fewer than two drones agree, the pile is not the most likely pile for the measured bearings, its association confidence is below 0.6, or most of the clip overlaps a more strongly associated source. Status stays `lead` / `radar_ping` (the status enum is unchanged); `why` starts with "Lead (ambiguous direction ...)"; clip priority is capped at 19 (below the grid mission's `priority_min` 20), so an ambiguous lead is listed but never triggers a dispatch on its own.
- Visited / excluded piles still take part in association, so their sources are nulled and never reported as clips.
- `transcript` is empty unless the clip looks like speech (VAD >= 0.3, >= 0.5 for non-speech classes, Whisper `no_speech_prob` < 0.6 and `avg_logprob` > -1.0) and does not repeat another pile's talker.

## Operator tasking (v1.2): search zones drawn before deployment

The operator outlines the areas to search on the dashboard before launch. The dashboard POSTs the outlines to the local ops server (`scripts/serve.py`), which validates them and writes `dashboard/data/tasking.json` atomically. The mission reads the file at start and re-reads it each step, so new zones can be added mid-mission. In the simulation, if no tasking exists, the default zones are the scenario's rubble outlines: a convex hull of each pile's pieces plus a 2 m margin, marked `source: "scenario-default"`.

```jsonc
// dashboard/data/tasking.json  (GET/POST http://127.0.0.1:8765/api/tasking)
{
  "schema": "larpverse.tasking/v1",
  "revision": 3,                       // bumped by the server on every POST
  "source": "operator",                // or "scenario-default"
  "updated_at": "2026-11-07T21:05:00",
  "zones": [
    { "id": "Z1", "name": "A2 east face", "polygon": [[x, y], …],   // site metres, 3..400 vertices
      "priority": 1,                   // optional operator priority (1 = first)
      "status": "tasked",              // operator side: tasked | cancelled
      "note": "bedrooms were here" }
  ]
}
```

The mission reports progress in `state.json` without changing the operator's file:

```jsonc
"zones": [
  { "id": "Z1", "name": "A2 east face", "polygon": [...],
    "status": "unsearched|surveying|scanning|searched_no_ping|ping|lead|beyond_radar_reach|cancelled",
    "coverage": { "lidar_mapped": 0.0-1.0, "radar_reached": 0.0-1.0, "listened": true },
    "pings": ["A2-R1"], "last_searched_t": 412.0,
    "retask_reason": null }            // set when the zone is reopened by new evidence
],
"zone_suggestions": [                  // acoustic leads outside every tasked zone, for the operator to accept
  { "id": "S1", "centre": [x, y], "radius_m": 6, "why": "voice 'help' heard by d1 and d3, bearing overlap",
    "evidence": ["a7"] }
]
```

Rules:
- The mission searches only inside tasked zones.
- A zone marked `searched_no_ping` or `ping` is not revisited unless new evidence arrives: a new acoustic clip associated with it, or an operator re-task. The reason is recorded in `retask_reason`.
- No detection is not proof of absence. A searched zone without a ping is shown as "searched — not cleared".
