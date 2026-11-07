# Larpverse: technical summary (draft for the one-page PDF)

Placeholders «…» are numbers still being produced; see the table at the top of `docs/PITCH_LARPVERSE.md`.

**User and problem.** The user is the incident commander or USAR team leader at a building collapse. Survivors are trapped in voids inside rubble piles, where thermal and RGB cameras can't see. A site has many piles and few crews, so the commander must triage which pile to work first and where on it to dig. Today listening devices and radar are placed by hand, often from unstable rubble, and crews dig on a best guess.

**What Larpverse does.** Three drones search a 3×3 field of rubble piles as one system and hand the commander **candidate search regions**, not confirmations.
1. **Map and land.** The drones orbit a pile with a spinning LiDAR. KISS-ICP SLAM builds the pile map, and every 1 m cell is scored for slope, roughness, step and clearance. The drones land on flat cells.
2. **Radar, rotors off.** Each landed drone looks for a respiration-like signal and reports a range. A Bayes grid fuses the ranges.
3. **Plan the next measurement.** When the ranges disagree, the planner moves one drone to the landing cell with the highest expected information gain and rescans. Three agreeing ranges give a **ping**: a position, a 95% region and a belief share. Otherwise the result is `insufficient_evidence`.
4. **Listen and triage.** Still landed, the drones listen to the other piles. Silero VAD, Whisper, YAMNet and SRP-PHAT direction finding turn audio into clips. An agent (Gemini, or a rule-based fallback) scores how human and how urgent each clip is, and the piles are ranked.
5. **Dispatch.** A dispatch gate weighs pile priority, travel time and battery, and sends the fleet to the next pile. The cycle repeats until the scan budget is spent. The commander then gets a handover brief: pings, open leads, and piles **not cleared**.

**Key features**
- **Measurement planning:** expected-information-gain relocation, not fixed landing patterns.
- **Uncertainty everywhere:** belief share, 95% region, an explicit `insufficient_evidence` outcome, and depth labelled "assumed 1.5 m" because ranges can't separate depth from position.
- **Acoustic agent:** multi-drone bearing association to piles, LCMV separation with nulls on other sources, cross-talk removal, gated transcripts.
- **Bounded LLM:** picks only from planner options; a validator enforces battery reserve, no-go piles and banned claims, and a deterministic fallback runs on any failure. Answers are recorded and replay offline.
- **One scenario everywhere:** the same seeded scenario drives Isaac Sim, the numpy simulator and the dashboard.

**Architecture and where each model runs.** Intended split below. Today everything runs on one laptop CPU without a GPU; Isaac Sim 6.0.1 runs on a cloud L40S.

| Component | Runs on | Latency |
| --- | --- | --- |
| LiDAR odometry (KISS-ICP), pile map, landing cells | drone (Jetson-class target) | 0.38 s per 23k-point scan on the laptop CPU, offline; to measure on Jetson |
| Radar breathing DSP (range FFT, phase, 0.1–0.5 Hz band-pass, peak, CFAR) | drone | not separately timed |
| Bayes fusion, EIG planner, ping logic | staging edge box | 0.7 s compute per single-pile mission (mean, 30 seeds) |
| Acoustic pipeline: association, LCMV, VAD, YAMNet, Whisper; fallback scorer | staging edge box | about 81 s per 26 s listen on the laptop CPU (cache miss); needs a GPU or smaller models to be real time |
| Dispatch gate, handover brief, dashboard | staging edge box | — |
| Gemini: dispatch pick, clip triage, operator notes, brief rewrite | cloud, optional | expected 1–4 s per call, 25 s timeout; not yet measured «GEMINI: …» |

**Evaluation.** Real data where stated; everything else is simulation only, not real-world performance.

| What | Result | Source and caveat |
| --- | --- | --- |
| Breathing DSP on **real** radar recordings | MAE **0.39 bpm** (176 windows of 30 s), 95% within 2 bpm | `docs/RADAR_VALIDATION.md`. Schellenberger 2020: 3 resting subjects, 24 GHz CW at 0.4 m, line of sight, **no rubble**. Validates the DSP only |
| SLAM pose error (B2, seed 7, 300 scans) | ATE **3.6 cm** vs 1.51 m odometry only | commit 110fdd2, `mapping.run`. Simulated LiDAR, one pile |
| Landing map vs true terrain | IoU **0.98**; 16/16 SLAM landing sites valid (odometry only 11/16) | same run. Geometry only, not structural safety |
| Planning vs fixed landing sites (single pile, 30 seeds, 6 scans) | resolved **43% vs 10%**; 95% area **100 vs 285 m²**; truth in 95% region 93% vs 93%; 0 false alerts each | `sim/evaluate.py`, 16:33 run on older code, provisional «B5-RERUN» |
| Acoustic dispatch vs nearest-first vs raster (multi-pile) | «EVAL-GRID: victims pinged, time to first ping, false leads, N seeds» | `docs/EVAL_GRID.md`; state whether the acoustic stub or the real pipeline was used |
| Acoustic front end on **real** audio | «ACOUSTIC-REAL: detection P/R, Whisper vs SNR» | `docs/ACOUSTIC_VALIDATION.md` |
| Seed-7 demo mission | 3 pings, each ~0.1–0.15 m from the hidden position; **1 of 4 hidden people missed** (faint voice at B1 attributed to empty C3) | `dashboard/data/state.json` 18:58. One seed, synthetic radar. «SEED7-RERUN» after the assignment fix |

**What the numbers say, including where they disagree with us.**
- **Planning earns its place on one pile:** same scan budget, four times as many runs resolve to a single region, with a third of the search area. Provisional until re-run.
- **The acoustic stage is the weakest link.** In the seed-7 demo it heard the faint voice but put it on the wrong pile, so the real person's pile ended "not cleared" and 12 scans went to an empty pile. «ASSIGN: …»
- **Depth is not measured.** Three ranges fix position on the pile, not depth below the surface.

**Limits**
- Radar returns are a labelled synthetic generator. Isaac does not model RF breathing through rubble.
- Acoustic scenes are simulated: TTS voices, synthetic knocks, one ESC-50 dog clip, free-field propagation.
- Flight is kinematic: no flight controller, no aerodynamics, no wind. Isaac renders the world, bodies and LiDAR; our code moves the drones.
- No drone-loss, aftershock, radio-loss or dust handling in this build.
- Gemini has not been called for real yet; every decision so far is `argmax` or `fallback`.

**Critical failure mode and safety**
- **Critical failure: a missed person.** No detection is not proof of absence. Silent piles end "not cleared", never "cleared", and the brief lists them.
- **False lead:** a ping needs three agreeing ranges; a lead without them ends as `insufficient_evidence`, not a ping.
- **Over-trust:** every ping is a candidate search region with its belief share, 95% region and assumed depth. Stationary is not trapped; periodic is not human.
- **Landing:** flat geometry says nothing about structural safety. People decide who goes onto a pile.
- **LLM:** sees summaries only, never ground truth, never the numeric loop; a validator and deterministic fallback guard every pick.
- **Ground truth:** only observation generators and evaluators read it; tests enforce the boundary.
- **Battery:** options that leave a drone under a 25% reserve are rejected; batteries swap at the staging pads.
- **Privacy:** audio is used only to score human presence and urgency. Transcripts are stored in the mission state for the commander, and there's no speaker identification.
