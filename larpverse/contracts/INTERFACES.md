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
  "algo": "assoc-v16",                          // localisation/association version (cache keys should include it)
  "clips": [ {
    "association": "confident",                // confident | ambiguous
    "association_conf": 0.97,                  // 0-1: bearing likelihood of this pile over all nine piles
                                               //  x (1 - 0.5 x rival-block share) x (1 - 0.5 x overlapped share)
    "ambiguous_with": ["B3"],                  // piles that explain the same bearings almost as well
    "reliability": 0.81,                       // agent reliability x (0.5 + 0.5 x association_conf); <= 0.45 if ambiguous
    "reliability_agent": 0.84,                 // the agent's own value before the association factor
    "doa_agree": "2/3",                        // drones whose bearing is within min(15 deg, gate) of the pile centre
    "heuristics": { "knock_periodicity": 0.62, "hum_harmonicity": 0.2, "human_minus_animal": 0.31,
                    "steadiness": 0.04, "blade_pass": 0.0 },             // rotor veto inputs
    "bearings": [ { "drone_id": "d1", "bearing_deg": 288.4, "sigma_deg": 1.6,   // per landed drone; sigma = bearing
                    "snr_db": 21.3, "measured": true } ],                         //  noise (gate width if not measured)
    "notes": [ "suppressed cross-talk from A3 (2 blocks)", "transcript dropped: dog clip with VAD 0.26" ]
  } ],
  "piles": [ { "association": "ambiguous",     // association of the top clip (absent for silent piles)
               "why": "Lead (ambiguous direction; could be B3): Voice saying ..." } ]
}
```

- `ambiguous` = fewer than two drones agree, the pile is not the most likely pile for the measured bearings, its association confidence is below 0.6, or most of the clip overlaps a more strongly associated source. Status stays `lead` / `radar_ping` (the status enum is unchanged); `why` starts with "Lead (ambiguous direction ...)"; clip priority is capped at 19 (below the grid mission's `priority_min` 20), so an ambiguous lead is listed but never triggers a dispatch on its own.
- `timing_s` (per phase) and `timing_stages_s` (vad, yamnet, whisper, beamform, srp_gcc, srp_clean, associate, heuristics) give the listen latency.
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

### Reconnaissance mode (v1.2): no zones tasked yet

When `tasking.json` has no active zones, the mission runs **recon** instead of waiting:

1. **Mapping flight.** A boustrophedon or spiral coverage path over `site.bounds`, or over an operator-given area of interest if one exists. It flies at a safe altitude (about 12–15 m above the highest mapped point), collision-checked against the growing map. The LiDAR builds the fresh site map, which is needed because post-collapse satellite imagery is stale. `state.phase = "recon_mapping"`.
2. **Perch-and-listen.** At intervals, each drone lands on a flat cell found in the fresh map, near unmapped or high-interest rubble, and listens with rotors off for about 15–20 s. Rotor ego-noise makes in-flight listening unreliable, so in-flight audio is only used for very loud events and is flagged as such. `state.phase = "recon_listening"`.
3. **Proposals.** The fresh heightmap is segmented into rubble mounds (connected areas rising well above street level), giving outline polygons. These go in `state.zone_proposals` together with acoustic leads. The operator accepts or edits them on the dashboard, which POSTs them to `/api/tasking`, and the targeted search starts.

```jsonc
"recon": { "status": "mapping|listening|proposing|done",
           "mapped_fraction": 0.0-1.0,            // of site.bounds (or the area of interest)
           "path": [[x, y, z], …],                // planned coverage path (for drawing)
           "perches": [{ "drone_id": "d2", "xy": [x, y], "t": 140.0, "clips": ["a3"] }] },
"zone_proposals": [
  { "id": "P1", "polygon": [[x, y], …], "kind": "rubble_mound",
    "height_max_m": 6.2, "area_m2": 310,
    "acoustic_leads": ["a3"], "score": 0.82,      // ranks proposals for the operator
    "why": "mound 6 m high; voice-like clip heard from it at a perch" }
]
```

### Field decisions (v1.2.1, orchestrator, answering the dashboard agent)

- **Unmapped heightmap cells:** producers write `heightmap.mask` (same shape as `heightmap.z`, `1` = mapped, `0` = not yet mapped). For unmapped cells the `z` value is `null`. Consumers accept either signal; `mask` wins if both exist.
- **Sector outlines:** `piles[].polygon` (`[[x, y], …]`, site metres, CCW) is canonical on every site. `piles[].outline` is read as an alias only; don't emit it in new code.
- **Drone tracks:** `drones[].track` is optional: the last ≤ 200 samples as `[x, y, z, t]`. Without it, consumers build tracks from successive frames.
- **Colour of `radar_ping`:** red is reserved for radar-confirmed candidates in `state.pings`. An acoustic recommendation `radar_ping` ("send radar here") renders as a **teal** lead, the same as `lead`.
- **`radar_reach_m`:** under the realistic penetration default it must be about 3 m (one metre of concrete or rubble costs about 10–20 dB). The figure of about 29 m from the seed-7 run is the legacy model and must not appear in demo state.

## vitals (BL-20, additive; `sim/vitals.py`, details in docs/VITALS.md and docs/RADAR_SIGNATURES.md)

Breathing-pattern features measured from the slow-time signal of a radar candidate's range bin, plus a confounder rejector. An AID for rescuers ordering the dig, never a diagnosis. Producers: `vitals.extract(...)` (one bin), `vitals.extract_from_obs(obs_debug)`, `vitals.identify(iq, params)` (whole scan: picks the return, adds scan context). Attached (a) to each radar observation as `"vitals": [ {...} | null ]` aligned with `range_candidates`, and (b) to each ping/candidate as `"vitals": {...}` (best-`pattern_conf` scan consistent with that ping). Absent or `null` = not measured (e.g. `radar=model`).

```jsonc
"vitals": {
  "version": "vitals-v1",
  "condition": "distressed_tachypnea",           // calm | distressed_tachypnea | unconscious_slow | irregular_apneic | unknown
  "condition_text": "rapid shallow breathing, 32/min -> possible injury, pain or distress, check",
  "source": "human_breathing",                   // human_breathing | no_signal | debris_shift | small_animal | machinery | rotor | moving_person
  "source_conf": 0.72, "source_reason": "breathing-like rhythm in the human range",
  "pattern": "rapid_shallow",                    // legacy alias: regular_calm | rapid_shallow | slow_regular | irregular_with_pauses | unclear
  "pattern_conf": 0.72, "pattern_reason": "rate 32/min > 30",
  "triage_hint": "immediate",                    // immediate | immediate (check) | urgent | delayed | assess | reject
  "triage_reason": "breathing over 30/min (START respiratory criterion for IMMEDIATE)",
  "priority_weight": 0.88,                       // hint rank (1, .85, .7, .55, .35, .15) shrunk toward 'assess' (0.55) as pattern_conf falls
  "rate_bpm": 31.6, "rate_ci": [29.8, 33.6],     // breaths/min (breath count if >= 3 clean intervals, else spectrum)
  "cv": 0.21, "amp_cv": 0.3,                     // robust breath timing / depth variability; null if too few breaths
  "depth_rel": 0.6, "depth_rel_conf": 0.12,      // 1.0 = 5 mm p-p; conf <= 0.3 ALWAYS; never used for the condition
  "apnea_events": 0, "apnea_events_per_min": 0.0, "apnea_longest_s": 0.0,   // pauses >= 10 s
  "movement": 0.03, "sqi": 0.81, "duration_s": 60.0,
  "signature": { "step_mm": 0.4, "line_hz": 0.53, "line_db": 31.0, "n_bursts": 0, "...": "rejector features" },
  "caveats": ["aid for rescuers, not a diagnosis: ...", "..."]
  // also: rate_spectral_bpm, rate_count_bpm, ie_ratio, snr_db, n_breaths, demod
}
```

- No age field; `fast_small` / `slow_deep` are no longer emitted.
- `source != human_breathing` -> `condition: "unknown"` and `triage_hint: "reject"` (weight 0.15), except `moving_person` -> `"urgent"` ("listen and call out").
- Dig order: `vitals.dig_score(v, p_mass, depth_m)` = `priority_weight x p_mass x clip(1 - depth_m / 4, 0.2, 1)`; no vitals -> the "assess" weight.
- Condition claims need a >= 45-60 s dwell; a 20 s scan is rate-only (a caveat says so). `unconscious_slow` text says "could be deep rest or reduced consciousness; the radar cannot tell which".
- Wording rule (UI, decider, brief): "possible", "check", "aid"; never "diagnosis", "confirmed", or "injured" / "unconscious" as fact.
- HUD playback streams for the signature library: `assets/radar_signatures/<class>.json`, format `radar-signature-hud-v1` (docs/RADAR_SIGNATURES.md).

### Mission state fields (v1.3, from the mission builder; additive)

- `mission`: `autonomous`, `dispatch_policy`, `radar`, `radar_penetration` (`legacy|literature`), `radar_reach_beneath_m`, `delivery` (`pod|land`), `scan_budget`, `scans_used`, `battery_reserve`, `battery_swaps`, `battery_swap` (`{to_pile, ends, since}` or `null`), `summary` (final text; `null` until done). `phase` adds `battery_swap`; `stage` adds `swap`. `acoustic_source` value `stub-from-truth` replaces `stub` (synthetic clips from hidden truth; a WARNING is logged).
- `site`: `kind`, `geo` (`{lat, lon, bearing_deg, name}`), `staging`. `pings` are firm, stationary candidates.
- `piles[]`: `polygon` (CCW `[[x, y]]`), `scans`, `radar_coverage`, `radar_coverage_shallow` (0–1, top 2 m), `radar_reach_m`, `nonhuman_listens`. A pile may appear twice in `mission.visited` (a revisit).
- `radar_candidates[]`: `depth_m`, `depth_sigma`, `depth_source` (`radar_estimate|nominal`), `p_kind` (share of pile belief), `support` / `support_needed` (strong agreeing ranges / 3), `outliers` (ids of outvoted scans; flagged, never a veto), `t_found`, `visit`, and optional `vitals` + `dig_score`.
- `scans[]`: `max_reliable_depth_m`, `vitals_look` (true for a 60 s dwell).
- `acoustic_clips[]`: `vad_speech_ratio`, `association`, `association_conf`, `ambiguous_with`, `bearings` (`[{drone_id, origin, bearing_deg, width_deg, snr_db}]`), `notes`.
- `drones[]`: `track` (last ≤ 200 `[x, y, z, t]`), `winch_len_m` (0 when the pod is stowed).
- `decisions[].options[]`: `score`. Actions add `revisit`; flags add `battery_swap` and `scan_budget_used`.

### Winch-lowered sensor pod (v1.3)

The heavy drone delivers the radar and a 4-mic array on a pod lowered by winch onto the rubble surface (see the PLAN decisions log). A drone carries at most one pod.

```jsonc
"pods": [ { "drone_id": "d1", "xy": [x, y], "z": 4.2,            // pod position (surface z when resting)
            "status": "stowed|lowering|scanning|raising",
            "placement": 2, "of": 3,                              // placement index for the current lead
            "lead": "a7", "t": 312.0 } ]
```
Scans made from a pod carry `scans[].delivery = "pod"` and have their origin at the pod's surface position (no air-gap path).

### Point cloud LOD and tiles (v1.3.1, R-PLAT-4)

```jsonc
"pointcloud": { "url": "pointcloud.bin",          // preview, ≤ 300k points (always written)
                "count": 300000, "lod": 0, "full_count": 2470000,
                "tiles": [ { "url": "pc_tiles/x+016_y-032.bin", "count": 144000,
                             "bounds": [xmin, ymin, xmax, ymax] } ] }   // 16 m square tiles, full resolution; optional
```
Viewers draw the preview first, then fetch only the tiles that touch the region around a clicked ping. With no `tiles`, they use the preview.

### Vitals condition names (v1.3.1)

The vitals output carries `vitals.condition` ∈ `calm | distressed_tachypnea | unconscious_slow | irregular_apneic` (or `unknown`). There is no age field anywhere. `vitals.pattern` is a legacy alias that consumers may still read.

### Rescue hotspot and comms relay (v1.4)

Each heavy drone runs an open Wi-Fi access point (2.4 GHz, SSID `RESCUE-<site>`). It's backhauled over the drones' existing mesh radio link to the operator's ground station (GCS); the GCS can add satellite or LTE internet when staging has it. A survivor whose phone connects sees a multilingual **captive portal**: "Help is coming. Are you trapped? [I'm trapped] [I'm safe] [send a message]". The portal also lets them message family when internet is up. Rescuers and residents on the surface can use it too.

The search gains a **consent-based** signal: a phone that connects has its RSSI, measured by several drones, used to estimate its position (blue). Portal answers go through the same multilingual Gemini path as audio, with fallback when the key is unavailable.

**Privacy rules:**
- Passive probe-request or MAC sniffing is **off** and isn't implemented.
- Devices get a random per-session token; MAC addresses are never stored.
- Messages stay at the GCS.
- Anything the operator sends to phones uses vetted templates, the same as the speaker; LLM translations go out only after the operator confirms.

```jsonc
"comms": {
  "backhaul": { "kind": "mesh", "gcs_xy": [x, y], "internet": true,
                "links": [ { "from": "d1", "to": "gcs", "rssi_dbm": -71, "mbps": 18.0 } ] },   // to: "gcs" or another drone (multi-hop)
  "hotspots": [ { "drone_id": "d1", "ssid": "RESCUE-WEIGUAN", "on": true, "xy": [x, y], "z": 6.0,
                  "clients": 2, "radius_m_est": 35 } ],
  "devices": [ { "id": "ph-7f3a", "first_seen": 212.0, "last_seen": 300.0,
                 "est_xy": [x, y], "est_sigma_m": 4.5, "pile": "A2",           // position from multi-drone RSSI; sector if inside one
                 "rssi": [ { "drone_id": "d1", "dbm": -78, "t": 298.0 } ],
                 "status": "trapped|safe|unknown", "battery_pct": 23,          // battery only if the portal reports it
                 "messages": [ { "t": 240.0, "text": "…", "lang": "nan", "translation": "…", "by": "gemini|fallback|none" } ],
                 "replies": [ { "t": 260.0, "template": "help_coming", "lang": "zh-TW", "confirmed_by": "operator" } ] } ],
  "portal": { "url": "http://10.42.0.1/", "submissions": 3 }
}
```

- `devices[].est_xy` is a **candidate region** (σ from the RSSI fit), never a confirmed position. RSSI through rubble is weak and multipath-heavy.
- A connected device whose position falls inside a zone counts as **new evidence**, so the zone can be re-searched.
- Colour: **blue** (#3b82f6) for comms, phones and hotspot coverage, everywhere.

### Speaker: call-out, reassurance and guidance (v1.3, from docs/SPEAKER.md §4)

```markdown
#### Speaker (v1.3, additive): loudspeaker broadcasts, call-out responsiveness, operator API

Heavy-lift drones carry a loudspeaker. Every played string is a vetted template (sim/guidance.py) or text an
operator typed; machine translations are `pending_approval` until an operator approves them. Never "confirmed".

state.json additions (sim/callout.Speaker.state; written by sim/grid_mission.py):

"broadcasts": [ { "id": "b23", "t": 600.1, "drone_id": "d3",
    "kind": "callout|reassure|guidance|operator", "lang": "zh-TW|nan-TW|en|id|vi",
    "text": "...", "text_en": "...", "audio_url": "acoustic/data/speaker/<key>.<lang>.wav" | null,
    "audio_status": "rendered|unvetted (...)|missing: record a native speaker|no audio yet ...",
    "target": { "type": "sector|ping|sectors|track|zone|drone", "id": "A3-R1", "xy": [x, y]? },
    "heading_deg": 271.6,            // guidance only: first leg, CCW from east (contract convention)
    "source": "template|operator|llm_translation", "template": "callout|reassure|guidance" | null,
    "status": "played|pending_approval|rejected", "approved_by": "op1" | null,
    "duration_s": 4.7, "review": "pending native-speaker review" | null, "group": "callout-2" } ],
    // scheduled items appear as "played" once t <= state.t; pending / rejected operator items are listed for the UI
"callouts": [ { "id": "C2", "t_start": 466.0, "listen_ends": 523.0, "t_end": 644.5, "speaker_drone": "d2",
    "target": { "type": "ping", "id": "A3-R1", "xy": [x, y] }, "langs": [...],
    "steps": [ { "step": "silence|message|listen", "lang"?, "t0", "t1" } ],
    "sectors": { "A3": <responsiveness> }, "pings": { "A3-R1": <responsiveness + ping_id, note> },
    "tracks": [ { "id": "T1", "xy": [x, y], "source": "callout_voice_open_ground", "evidence": [t...], "why": "..." } ],
    "guided": [ { "track": "2-T1", "xy", "source", "why", "text_en", "path": [[x, y]...], "legs": [{bearing_deg, dist_m, compass}] } ],
    "reassured": ["A3-R1"], "n_events": 41, "synthetic": true } ],   // results only after listen_ends; before: status "in_progress"
"responsiveness": { "A3": <responsiveness> },   // per pile: the call-out heard from closest (then latest)
<responsiveness> = { "status": "responded|possible_response|nonhuman_only|no_response", "kind": "knock_x3|knock_fast|knock|voice|shout|crying|..." | null,
    "avpu": "V|unknown|not V (A/V not shown)", "confidence": 0-1, "text": "...", "callout_t": 466.0,
    "range_m": 11.4, "message_reached": true, "events": [...]? , "lang_hint": "zh-TW"? }
piles[].responsiveness, radar_candidates[].responsiveness (same object); radar_candidates[].triage_fused
    { triage_hint, condition, avpu, reason, changed_from, rank } when the candidate has vitals
    (hints / ranks from sim/vitals.TRIAGE_RANK; no response raises, never lowers).
drones[].speaker = { kind, lang, id } while that drone is playing.

Rules: "no_response" never clears a pile and its text says it does not rule out an unconscious person; a
"responded" sector is a candidate, not a confirmed person; guidance routes use the LiDAR heightmap (rubble
> 0.4 m above local ground, +2 m margin) and tasked zones, never truth.

Operator API (scripts/serve.py, queue file dashboard/data/broadcast_queue.json, read by the mission each step):
GET  /api/broadcast                 -> { schema: "larpverse.broadcast_queue/v1", revision, items: [broadcast...] }
POST /api/broadcast                 { "drone_id": "d2" | "zone": "Z1", "kind": "operator|callout|reassure",
                                      "text": "...", "src_lang": "en", "langs": [...], "translate": true,
                                      "operator": "name" }
     -> { items, translation_problems }; operator text: scheduled; translations: pending_approval;
        callout / reassure: the vetted templates (text ignored)
POST /api/broadcast/approve         { "id": "op203011-2", "operator": "name", "approve": true|false }
```


### Gemini audio fields (acoustic/v1 additions, from docs/ACOUSTIC_GEMINI.md)

All additive: a v1 reader keeps working.

```jsonc
"clips": [ { /* v1 fields */,
  "novelty": 14.5,                      // stage-1 peak novelty (background ≈ 1-2, threshold 4)
  "forwarded": true,                    // something in this clip went to stage 2
  "dismissed": ["dismissed: dog"],      // stage-1 dismissals in this clip (kept visible)
  "language": "cmn", "languages": [{"code": "cmn", "name": "Mandarin"}],
  "transcript_original": "救命！我被困在樓梯下面…", "translation": "Help! I'm trapped under the stairs…",
  "distress": ["call_for_help", "trapped", "location_hint"],
  "audio_analysis": { "by": "gemini", "by_note": null, "sound_type": "call_for_help", "human_vocal": true,
                      "confidence": 0.95, "people_count": 1, "location_hints": ["under the stairs"],
                      "rationale": "...", "model": "gemini-3.5-flash-lite", "error": null,
                      "cross_check": null, "verify": {"ok": true, "similarity": 0.93, "...": "..."},
                      "distress_unverified": [], "t0": 7.3, "t1": 11.9 },
  "stage1": { "denoise": "noisereduce", "profile": "background 26.0 s (quietest 70%)",
              "events": [ {"cand_id": "d1-n1", "t0": 7.5, "t1": 11.7, "novelty_peak": 14.5,
                           "label": "likely_human", "reason": "Speech 0.92, VAD 1.00", "forwarded": true,
                           "dismissed": null, "group": "d1-g1"} ],
              "stage2_clips": [ {"cand_id": "d1-g1", "t0": 7.0, "t1": 12.2, "members": ["d1-n1"]} ] } } ],
"piles": [ { /* v1 */, "audio_priority": 83, "distress": ["trapped"], "languages": ["en"] } ],
"sector_tasks": [ { "rank": 1, "pile_id": "B1", "priority": 83,
                    "status": "lead",           // lead | uncertain_sound | dismissed_nonhuman | no_signal_not_cleared
                    "human": 0.9, "urgency": 1.0, "reliability": 0.85,
                    "languages": [{"code": "en", "name": "English"}], "distress": ["call_for_help", "trapped"],
                    "reasons": ["..."], "rationale": "...", "rationale_by": "gemini",   // or "template"
                    "top_event": "d1-g1", "events": 2, "forwarded": 1, "dismissed": [], "by": "gemini" } ],
"gemini_audio": { "prompt": "gemini-audio-v5", "mode": "auto",
                  "stage1": {"candidates": 5, "forwarded": 3, "dismissed": {"dog": 1}},
                  "stage2": {"calls": 3, "cached": 0, "fallback": 0, "errors": [], "latency_ms": 4500,
                             "audio_s": 13.5, "usage": {...}, "rationale": {"by": "gemini", ...}} }
```

With `reprioritise=True` (the default), `merge_into_state` raises a pile's `priority`/`why` to the stage-3 value
when that value is higher and the task is a `lead`, then re-picks `radar_ping` and `dispatch` from the top lead.


### Tasking and recon defaults (v1.4.1, settled)

- **No `tasking.json`, or no active zones: the mission runs recon** (the user's decision). The scenario's default sector zones are opt-in with `--tasking default`.
- `tasking.json` may carry `aoi` (one polygon, site metres), which restricts the recon coverage flight. `scripts/serve.py` validates it and passes it through.
- Mission `stage` adds `recon`. `phase` uses `recon_mapping` and `recon_listening` (above) while it's active.

### Link status and pod commands (v1.4.2, from docs/COMMS_ARCHITECTURE.md)

- `drones[]`: `link` (`ok|degraded|lost`), `last_link_t`, `queued_kb` (data held on board while the mesh is down; forwarded later).
- `comms.backhaul.links[]`: `latency_ms`.
- Commands from the van to the drones (MAVLink winch commands on the flight side; JSON on the ops API): `lower_pod {drone_id, xy}`, `raise_pod {drone_id}`, `relay_hold {drone_id, xy, alt}` (park a drone as a mesh relay).
- Backhaul for Tainan: OneWeb via Chunghwa Telecom, or LTE. Starlink is the generic option where it's licensed.

### Comms additions (v1.4.3, from docs/HOTSPOT.md §9)

- `comms.backhaul.uplink`: `oneweb|lte|starlink|null`. Optional `backhaul.relay_xy` (a dropped relay node).
- `devices[].live` is true for real portal phones (the live demo), which carry no RSSI position. `messages[].synthetic` is true for simulated messages, and the UI tags them "synthetic". `devices[].replies[].text` holds the template text that was sent.
- `piles[].comms_evidence`: `{device, status, t, est_xy, sigma_m}`. Phone evidence only ever raises a **lead**, never a ping, because a phone near-but-buried and one far-but-in-the-open can look alike in RSSI.
- Event names: `joined`, `left`, `status`, `message`, `located`, `evidence`, `link_lost`, `link_restored`.
- Language codes use `nan-TW`, matching the speaker templates, for Taiwanese Hokkien.
- Ops API (scripts/serve.py):
  - POST `/api/portal`: from phones; validated, rate-limited, no IP stored.
  - GET `/api/portal` → `{"submissions": [...]}`: localhost only.
  - POST `/api/comms/reply`: localhost only; templates, or free text with `confirmed_by`.
  - Captive-portal check URLs redirect to `/dashboard/portal/`.
  - `--lan` binds 0.0.0.0. It's off by default and prints a warning.
