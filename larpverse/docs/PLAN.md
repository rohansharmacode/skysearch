# Larpverse build plan (24 h MVP)

The goal is one honest end-to-end loop, visible in Isaac and on the dashboard:

**sense → pick landing cells → land → scan → fuse → ambiguity → autonomous relocation → rescan → candidate region**

Everything else comes after that loop works.

## What already exists in this repo

> This table is from the first hour and is out of date. For the current state, read `README.md` (module map) and the Backlog below. Current headline: the grid mission (`sim/grid_mission.py`), the 3×3 dashboard, the acoustic agent, Gemini/fallback agents, KISS-ICP mapping, and the Isaac adapter with `--selftest`.

| Path | State | What it is |
|---|---|---|
| `animation/index.html` | done | A 66 s interactive 3D concept animation for explaining the idea. Its localisation, ambiguity and relocation are computed, not keyframed. |
| `contracts/INTERFACES.md` | agreed v1 | Frames, units, command, observation and dashboard state schema, plus the ground-truth boundary. |
| `dashboard/` | working | Site map with red candidate dots, yellow acoustic cones, drones and landing sites. Clicking a dot opens the LiDAR detail view with the depth ping. It polls `data/state.json` and falls back to demo data. |
| `sim/` | first pass | numpy core: mock world, landing cells, synthetic radar, grid localiser, planner, coordinator, evaluator. It writes `dashboard/data/state.json`. |
| `docs/RESEARCH.md` | done | Open-source picks, licences and Isaac version traps. |

Run it all locally (no GPU needed):

```bash
cd ~/Downloads/UCL/agentverse && .venv/bin/python -m sim.coordinator --realtime --out dashboard/data
```

```bash
cd ~/Downloads/UCL/agentverse && python3 -m http.server 8765
```

Then open `http://localhost:8765/dashboard/` (live) and `http://localhost:8765/animation/` (concept).

## Architecture (matches the team diagram)

```
Dashboard  <- polls state.json --  Coordinator (sim/coordinator.py: localise, planner)
                                        |  commands (drone_id, destination, scan_duration)
                                        v
                          Drone adapter: mock (sim/world.py)  or  Isaac (isaac/)
                           |- LiDAR  -> point cloud -> SLAM map, heightmap, landing cells
                           |- Radar  -> synthetic IF -> breathing DSP -> range candidates
                           '- Acoustic -> simulated mic array -> VAD/YAMNet + SRP-PHAT -> bearing cone
```

Isaac is only the world and the body. The brain (`sim/localise.py` and `sim/planner.py`) is engine-agnostic numpy, importable inside Isaac's Python. If Isaac breaks at hour 20, the mock world still demos the same decisions.

## Critical risks (deal with these in hour 0)

1. **No GPU on the dev laptop.** It has no NVIDIA card and 14 GB RAM, while Isaac needs an RTX 4080-class card (16 GB VRAM) and 32 GB RAM.
   - Decide in the first hour: a teammate's RTX desktop, or Brev L40S (Isaac 5.1 Launchable, about $1–2/hr).
   - Pin that version and only follow docs for it.
2. **Isaac API churn** between 5.x and 6.x. Spike LiDAR and pose-setting first, before building on them.
3. **Integration drift.** One person owns `contracts/INTERFACES.md` and the main loop. Nobody changes a field without telling them.

## Workstreams and tasks

Owners are slots. Put names in when you split up. **P0** = needed for the demo, **P1** = makes it convincing, **P2** = nice to have.

### A. Isaac world and body (owner: A, on the GPU machine)

| # | Pri | Task | Done when |
|---|---|---|---|
| A1 | P0 | Get Isaac running headless and in the GUI on the chosen machine. Write a hello-world standalone script that loads a ground plane. | A screenshot is in `docs/` |
| A2 | P0 | Procedural rubble, `isaac/rubble.py`: seeded slabs, chunks and columns dropped as rigid bodies with convex-decomposition collision, settled for 2 s, then frozen. Export a USD. | Same seed gives the same pile |
| A3 | P0 | Three `quadcopter.usd` drones moved kinematically by `set_world_poses`, driven by coordinator waypoints. Altitude collision check against the heightmap. | Drones fly the mock mission path |
| A4 | P0 | LiDAR on each drone (RTX `Example_Rotary`, or PhysX raycast fallback). Accumulate a world-frame point cloud and write `pointcloud.bin` plus the heightmap grid in the `state.json` format. | The dashboard shows the Isaac cloud |
| A5 | P1 | `isaac/adapter.py` implements the same interface as `sim/world.py`, so `coordinator.py --world isaac` works. | One flag swaps mock for Isaac |
| A6 | P1 | Hidden survivor capsules placed in pile voids, with their pose exported only to the radar generator and evaluator. | Ground truth stays out of the planner's inputs |
| A7 | P2 | KISS-ICP on the per-drone scans instead of ground-truth poses, for a real SLAM story. | The map overlays the ground-truth cloud |
| A8 | P2 | Cinematic camera path and recording for the video. | MP4 exported |

### B. Sensing, fusion and planning (owner: B, any laptop)

| # | Pri | Task | Done when |
|---|---|---|---|
| B1 | P0 | Review the `sim/` first pass. Make sure the default seed shows a genuine ambiguity and a planner-chosen relocation. Run `tests/`. | Tests pass, and the log explains each decision |
| B2 | P1 | Radar DSP chain, `sim/radar_dsp.py`. Synthetic FMCW IF (range bins plus breathing phase, rubble attenuation, motion artefacts), then range FFT → phase unwrap → 0.1–0.5 Hz bandpass → peak. It outputs `range_candidates`, `confidence` and `breathing_hz` instead of the current direct noise model. | Spectrogram plot in `docs/` |
| B3 | P1 | Validate B2's extractor on Schellenberger 2020 recordings (CC BY 4.0). | Rate error table on real data |
| B4 | P1 | Acoustic chain, `sim/acoustic_dsp.py`: pyroomacoustics 4-mic array per landed drone, knock or shout clip at the survivor, Silero VAD / YAMNet class, SRP-PHAT bearing and cone width. | Real DoA cones in `state.json` |
| B5 | P1 | Evaluation: adaptive vs fixed sites, same scan budget, 30 seeds. Measure error, 95% coverage, false alerts, scans and runtime. | `eval/results.json` plus a slide table |
| B6 | P2 | Moving distractor (Doppler-moving track kept separate), then a periodic non-human distractor. | It shows as a separate track, not a candidate |

### C. Dashboard, story and video (owner: C)

| # | Pri | Task | Done when |
|---|---|---|---|
| C1 | P0 | Run the dashboard against live `state.json` from the coordinator. Fix any contract mismatches by changing code, not the contract, unless the integration owner agrees. | Live map updates every second |
| C2 | P1 | Timeline scrubber over `frames/state_XXXX.json`, so the demo can replay the decision moment. | Scrubbing shows ambiguity, then relocation |
| C3 | P1 | Belief-over-time strip and an entropy sparkline in the side rail. | Visible drop after the rescan |
| C4 | P1 | One-minute video: deploy, LiDAR, landing, ambiguous scans, relocation, collapsed region, acoustic cone, dashboard click-through. Use the concept animation as the storyboard. | MP4 under 60 s |
| C5 | P2 | Slides: problem, what's new (planning and uncertainty, not radar), the honest limits, eval table. | Deck |

### D. Mission Commander agent (owner: D, any laptop)

The full option map is in `docs/agent-decisions.html` (published: https://claude.ai/artifact/2DJSXqUQfkZGjKfFifxjay). One agent, three entry points, hooked into `Coordinator._decide()`.

| # | Pri | Task | Done when |
|---|---|---|---|
| D1 | P0 | `planner.options(k)` returns scored actions: relocate, extend dwell, acoustic lead, stop, return to base | The option list is logged at every gate |
| D2 | P0 | `sim/agent.py`: context from state (assert no ground truth), tool loop (preview, scan_detail, site_detail, commit), validator, fallback to argmax, record/replay | `--policy agent` completes seed 75 offline from replay |
| D3 | P0 | Operator intel intake (`ops/intel.jsonl` → priors, no-go zones, ignored sounds) and the handover brief | Brief text cites state fields only |
| D4 | P1 | `decisions[]` in `state.json` plus a dashboard Decision card: option table, pick, rationale, agent or fallback badge | Visible during the ambiguity moment |
| D5 | P1 | Evaluation comparing fixed, argmax and agent policies, with scenario packs (intel, battery, acoustic lead, unstable site) | Table in the slides |

### I. Integration owner (one person, all 24 h)

- I1 (P0): owns `contracts/INTERFACES.md`, merges, runs the end-to-end loop every 2 hours, and keeps `main` demoable.
- I2 (P0): enforces the ground-truth boundary. Run `grep -rn "targets" sim/` and confirm that only `radar_model.py`, `acoustic_model.py` and `evaluate.py` touch it.

## Timeline

| Hours | Milestone |
|---|---|
| 0–1 | GPU machine decided; everyone has run the mock loop and the dashboard; contract agreed |
| 1–6 | A1–A3 (Isaac scene, drones moving); B1; C1. **Single-drone sense–move–scan–update loop in mock** |
| 6–12 | A4–A5 (Isaac LiDAR into the dashboard, adapter); B2, B4; C2 |
| 12 | **Freeze the core loop in Isaac.** Only fixes after this |
| 12–18 | A6–A7; B3, B5; C3. Record backup footage from the mock run |
| 18–22 | Video (C4), slides (C5), eval numbers |
| 22–24 | Rehearse, buffer, nothing new |

## Rules that keep us honest (from the brief)

- The planner and localiser never read ground truth. Only the generators and the evaluator do.
- Three noisy ranges can't identify a faulty one. Allow a fourth scan and an `insufficient_evidence` outcome.
- Stationary does not mean trapped, and periodic does not mean human. The UI says "candidate search region".
- No detection is not proof of absence. Landing geometry is not structural safety.
- Keep LLM use bounded: any frontier-model orchestration sees summaries only, never ground truth, and never in the numeric loop.

## Running Claude Code on this (two 20x plans)

Give each session one workstream and point it at this file plus `contracts/INTERFACES.md`. Good first prompts:

- **A:** "Read docs/PLAN.md and contracts/INTERFACES.md. Do A1–A3 for Isaac Sim <version> on this machine. Start with a minimal standalone script and verify each step runs before adding the next."
- **B:** "Read docs/PLAN.md, contracts/INTERFACES.md and sim/. Do B2 then B4 using docs/RESEARCH.md picks. Keep numpy/scipy only in the core and gate heavy deps (torch for Silero/YAMNet) behind an optional import."
- **C:** "Read docs/PLAN.md and dashboard/. Do C1 then C2 against `python -m sim.coordinator --frames`."

## Backlog (orchestrator-owned; updated live)

| ID | Pri | Item | Owner / status |
|---|---|---|---|
| BL-1 | P0 | Reviewer pass 1 over the whole product | DONE 575253f: 46 findings (5 P0, 25 P1, 16 P2) |
| BL-2 | P0 | Isaac adapter cross-check against the team's verified Isaac 6.0.1 / Isaac Lab 3.0 API notes | DONE 629b128, 2e75dd2 (AppLauncher default, --selftest, --look pretty, --record); awaiting teammate GPU selftest |
| BL-3 | P0 | Acoustic pile assignment: voices attributed to wrong piles (B3/C3 in seed 7); triangulate across drones, reject inconsistent bearings | builder — running |
| BL-4 | P1 | Gemini decider + provider layer + intake + handover brief (agents/) | DONE 1ed9bae (fallback-tested; real API untested until key) |
| BL-5 | P1 | KISS-ICP SLAM (mapping/) then wire into grid mission landing maps | DONE 110fdd2 (ATE 3.6 cm B2, landing IoU 0.98); wiring → mission agent |
| BL-6 | P1 | Animation v2 driven by the seed-7 scenario | DONE 70442db; MP4 on release; artifact v4 |
| BL-7 | P1 | Grid evaluation: acoustic-priority order vs nearest-first vs fixed order; victims pinged within budget, time to first ping, false leads; N seeds | builder — running (with BL-8 + autonomy loop) |
| BL-8 | P0 | A3 never pings with the model radar after 8 scans — make `--radar dsp` default for the grid demo or tune; explain in log | queued |
| BL-9 | P2 | Pitch pack: one-page technical summary, 1-minute video script, deck (after BL-7 numbers) | queued |
| BL-10 | P1 | Dashboard fix batch: review R1-016…029 | builder — running |
| BL-13 | P1 | Review pass 1 brain batch (R1-001…015): acoustic items → BL-3 builder; sim items → mission builder | routed |
| BL-14 | P1 | Isaac handoff batch (R1-030…038): done by orchestrator, except R1-033 (needs final story), R1-037 (GPU label), R1-038 (depth wording after R1-006) | mostly done |
| BL-12 | P1 | Acoustic real-data validation (DroneAudioSet / ESC-50): detection P/R, Whisper under drone noise, vs SNR | builder — running |
| BL-15 | P0 | One-command demo (scripts/demo.sh) + dashboard replay recording + docs/DEMO.md runbook | builder — running |
| BL-16 | P0 | Gemini audio pipeline: open-model background/novelty filter → Gemini multilingual audio analysis → sector priorities (user design) | builder — running |
| BL-17 | P0 | Case-study map: Weiguan Jinlong collapse, Tainan 2016 (23.0051432 N, 120.2608904 E) — georeferenced basemap in dashboard | dashboard builder |
| BL-18 | P0 | Radar penetration realism: ~0.5–3 m through rubble, 10–20 dB/m concrete; coverage + 'beyond radar reach' | mission builder |
| BL-19 | P1 | Weiguan model in Isaac: Sketchfab 'VR-Weiguan Collapsed Building' (NCKU Museum, photogrammetry, 1M tris, licence/download not stated) — needs permission; else procedural stand-in of the toppled 16-storey U-shaped block | blocked on licence |
| BL-20 | P0 | Radar vital-sign triage: breathing pattern (rapid-shallow / slow-deep / irregular-apnoeic / calm / fast-small) → triage hint → dig-order priority; validated on real apnea data where possible | builder — running |
| BL-21 | P0 | Technical report (docs/TECHNICAL_REPORT.md + abstract) | builder — running |
| BL-22 | P1 | Visual consistency batch 4 (animation + isaac/overlay) + grid MP4 re-record | builder — running |
| BL-23 | P1 | Expanded Weiguan map from the Isaac teammate (Weiguan + surrounding collapsed buildings): mesh/heightmap import → scenario sectors | waiting on teammate push; importer being built |
| BL-24 | P0 | Operator tasking: operator circles search zones before launch; drones search only those; coverage tracking; no re-search without new evidence; outside-zone acoustic → zone suggestions (ops server scripts/serve.py + contract v1.2 + sim/tasking.py + dashboard Plan-search mode + Isaac zone overlay) | builders — running |
| BL-25 | P0 | Recon mode when no zones: coverage-path LiDAR mapping (fresh map; satellite is stale) + perch-and-listen (rotors off) + mound segmentation → zone proposals → operator accepts (auto-accept in --autonomous) | tasking builder (sim/recon.py) + dashboard + overlay |
| BL-26 | P1 | Heavy-lift (agri/industrial class) drone platform: specs, dense LiDAR (32–128 ch, 0.3–1.3 M pts/s), bigger landing footprint, endurance; Isaac heavy drone model + dense LiDAR profile | builder — running |
| BL-27 | P1 | Drone loudspeaker: multilingual call-out-and-listen (responsiveness → triage), reassurance after pings, guidance directions for walking survivors (A* to staging), operator broadcasts (LLM text only after operator confirms) | builder — running |
| BL-21 | P0 | Technical report | DRAFT 893bd37 (~11k words; placeholders until builders land) |
| BL-11 | P1 | KEY SAVED 19:58 → Gemini audio builder records live runs. Once GEMINI_API_KEY exists: record a seed-7 Gemini run and commit its replay cache (un-ignore one curated file) so the demo replays offline | blocked on key |

## Decisions log

| Date | Decision | Why | By |
|---|---|---|---|
| 2026-11-07 | Agent role = acoustic agent ranking piles + dispatch | user's vision: AI listens for cries for help, gives priority per pile | user |
| 2026-11-07 | Isaac uses kinematic drones + PhysX raycast LiDAR via pxr/omni.physx only | stable across Isaac 4.5–6.x; Pegasus broken on 6 | orchestrator |
| 2026-11-07 | Our code lives in larpverse/ on branch larpverse-sensing of the team repo | team repo already has SkySearch; avoid clobbering | orchestrator |
| 2026-11-07 | Gemini (REST, GEMINI_API_KEY, gemini-3.5-flash) is the decision LLM | user request; matches team's v2-earthquake briefing | user |
| 2026-11-07 | Radar returns stay a labelled synthetic generator; DSP validated on Schellenberger 2020 real data (0.39 bpm MAE; 3 subjects, line-of-sight CW at 0.4 m, no rubble) | Isaac RF doesn't model breathing through rubble | orchestrator |
| 2026-11-07 | LLM answers cached in agents/cache (gitignored); a curated demo replay file will be committed once recorded | demo must run offline and reproducibly | orchestrator |
| 2026-11-07 | kiss-icp 1.3.0 + open3d 0.20 accepted in .venv | SLAM agent installed before rules; numpy unchanged, tests pass | orchestrator |
| 2026-11-07 | Isaac default launch = Isaac Lab AppLauncher (6.0.1/Lab 3.0, Brev); SimulationApp fallback | team's GPU-verified facts | orchestrator |
| 2026-11-07 | usd-core allowed in .venv for offline pxr checks | lets us validate stage code without a GPU | orchestrator |
| 2026-11-07 | Isaac adapter defaults to --radar dsp | review R1-005/034: model radar never pings A3 | orchestrator |
| 2026-11-07 | Storyboard MP4s published as a pre-release on the team repo (larpverse-storyboard) | >100 MB, can't go in git; teammate needs them | orchestrator |
| 2026-11-07 | Case study = Weiguan Jinlong building collapse, Tainan, 6 Feb 2016; site frame anchored at 23.0051432 N, 120.2608904 E | user | user |
| 2026-11-07 | Audio pipeline = open-model novelty filter first, only outliers sent to Gemini (multilingual), then prioritisation | user design | user |
| 2026-11-07 | Radar modelled with realistic penetration (≈3 m max reliable); deep victims become acoustic-only leads | physics: 10–20 dB/m concrete at 0.4–2 GHz | orchestrator |
| 2026-11-07 | Vital-sign triage from radar is an aid for rescuers, not diagnosis; age-from-amplitude reported as low confidence only | amplitude confounded by range, aspect and rubble loss | orchestrator |
| 2026-11-07 | Video-cut builder paused until Isaac footage + brain fixes land | avoid cutting stale story; free a slot for vitals | orchestrator |
| 2026-11-07 | Acoustic pipeline: choose by measured comparison (ours vs Frances's) in docs/ACOUSTIC_VALIDATION.md; port her best parts | user delegated the choice | orchestrator |
| 2026-11-07 | Gemini usage unlimited (temporary hackathon key); use live for audio analysis, prioritisation and decisions; cache for offline replay | user | user |
| 2026-11-07 | Expanded Weiguan map comes from the Isaac teammate; we build the importer + procedural fallback | teammate already doing it | user |
| 2026-11-07 | Up to 10 concurrent agents (user: use the budget; heavy jobs still gated by heavy.sh) | speed | user |
| 2026-11-07 | Search areas are operator-tasked zones drawn on the dashboard before launch (sim default = rubble outlines); zones aren't re-searched without new evidence | user: realistic + avoids repeat searches | user |
| 2026-11-07 | Dev server is now scripts/serve.py (static + /api/tasking), same port 8765 | dashboard needs to save tasking | orchestrator |
| 2026-11-07 | No zones → recon: map the site + perch-and-listen, then propose zones; in-flight listening only for loud events (rotor ego-noise) | user: more data, realistic when satellite imagery is stale | user |
| 2026-11-07 | Platform = heavy-lift (agri/industrial) drones with dense LiDAR, bigger radar antenna, larger mic array, loudspeaker | user: bigger batteries + better sensors | user |
| 2026-11-07 | Speaker broadcasts use vetted multilingual templates; LLM-translated operator text only after the operator confirms | never auto-broadcast unreviewed LLM text to victims | orchestrator |
