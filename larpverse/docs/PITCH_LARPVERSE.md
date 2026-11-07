# Larpverse: pitch pack (AgentVerse 2026, robotics track)

**One line:** three drones search a field of collapsed-building rubble piles, decide on their own where to measure next, and hand the incident commander **candidate search regions** with their uncertainty, plus a ranked list of which pile to search next based on cries for help.

## Open placeholders (fill before the pitch)

Numbers that other builders are producing now. Do not say any of these out loud until the source doc exists.

| Placeholder | Fills from | Status at writing (2026-11-07, ~20:00) |
|---|---|---|
| «EVAL-GRID: …» | `docs/EVAL_GRID.md` (`sim/evaluate_grid.py`: acoustic dispatch vs nearest-first vs fixed raster, N seeds) | builder running; doc not yet written |
| «ACOUSTIC-REAL: …» | `docs/ACOUSTIC_VALIDATION.md` (`acoustic/validate_real.py`: DroneAudioSet / ESC-50, detection precision/recall, Whisper under rotor noise vs SNR) | builder running; doc not yet written |
| «ASSIGN: …» | `acoustic/eval_assign.py report` (per-pile assignment accuracy, v1-baseline vs current) | builder running (BL-3) |
| «SEED7-RERUN: …» | seed-7 mission re-run after BL-3 lands | the committed demo state (18:58) misses the faint voice at B1, see Evidence |
| «B5-RERUN: …» | `python -m sim.evaluate --n 30 --budget 6` on current code | the numbers below come from a 16:33 run, gitignored |
| «DASH-MP4» | `media/larpverse-dashboard-seed7.mp4` | not recorded yet |
| «ISAAC-CLIP: …» | Isaac footage from the teammate (`larpverse-isaac` branch, `media/isaac/`) | contact sheets exist; final clips pending |
| «GEMINI: …» | first real Gemini run (BL-11) | blocked on the API key; every decision so far is the fallback |
| «STAT: …» | one checked real-world figure for the hook | not chosen; cite INSARAG or a named post-earthquake USAR report, and check it first |

## 1. Problem

- After a building collapse, survivors are trapped **inside** rubble piles, in voids under slabs. Thermal and RGB cameras see the surface, not the void.
- A real collapse site is **many piles**, not one. Search teams have to triage which pile to work first, with few people, few listening devices, and a closing time window. «STAT: one checked figure on trapped-person survival over time, or on the scale of a recent collapse, with its source.»
- Teams today place listening devices and radar by hand, often standing on unstable rubble, and **dig where they guess**. Each wrong guess costs hours.

## 2. User

The **USAR team leader / incident commander** at the site. They decide which pile a crew works next and where on that pile to dig first. They do not want a "found someone" alarm they can't trust. They want: where to look, how sure we are, what we haven't checked yet.

## 3. What's new (and what isn't)

We did **not** invent through-rubble radar, acoustic search, or LiDAR SLAM. Those exist. What we built is the **autonomy around them**:

1. **Measurement planning.** Drones choose where to land next by expected information gain over a Bayes belief of where a breathing source could be. One range is a ring; the planner picks the landing spot that best separates the competing explanations.
2. **Uncertainty-aware localisation.** Every result carries its uncertainty: a belief share, a 95% region, and an explicit `insufficient_evidence` outcome when the scans don't agree. A result is a **candidate search region**, never "a person found".
3. **An acoustic agent that triages piles.** With rotors off, the landed drones listen to the *other* piles. Real models (Silero VAD, Whisper, YAMNet, SRP-PHAT direction finding) turn the audio into clips; an agent (Gemini, or a rule-based fallback) scores how human and how urgent each one is; the piles are ranked; a dispatch gate sends the fleet to the next pile.
4. **Bounded LLM.** The LLM picks only from options the planner lists. Code validates every pick (battery reserve, no-go piles, banned claims such as "confirmed") and falls back to a deterministic choice on any failure. The LLM never sees ground truth and never touches the radar or Bayes maths.

## 4. How it works (the loop)

**survey → land → radar → ping → listen → rank → dispatch → repeat**

1. **Survey.** The three drones orbit the pile with a spinning LiDAR. KISS-ICP SLAM registers the scans into a pile map.
2. **Land.** Every 0.5 m cell is scored by a plane fit over its 1.5 m neighbourhood (slope, roughness, step, clearance). Drones land on the flat cells.
3. **Radar.** Rotors off, each landed drone scans for a respiration-like signal: range FFT → phase → 0.1–0.5 Hz band-pass → spectral peak → CFAR. Each detection is a range, i.e. a ring.
4. **Localise and plan.** A grid Bayes filter fuses the rings. If they disagree, the planner moves one drone to the cell with the highest expected information gain and rescans (seed 7: "Relocate d1 → L15 … EIG 0.56 nats").
5. **Ping.** When three ranges agree, the region becomes a ping: position, 95% region, belief share. **Depth is not measured**: it is shown as "assumed 1.5 m", because the ranges cannot separate depth from position.
6. **Listen.** Still landed, the drones record 26 s of audio and run the acoustic pipeline on the other piles.
7. **Rank and dispatch.** The agent scores each clip; piles get a priority; the dispatch gate weighs priority, travel time and battery and sends the fleet on. Every decision is logged with its options, its pick, and who made it (`gemini`, `argmax` or `fallback`).
8. **Handover.** When the scan budget runs out, the commander gets a brief built only from state: pings, open leads, and piles **not cleared**.

## 5. What's real and what's simulated

| Part | Real | Simulated |
|---|---|---|
| Rubble site | Same seeded scenario drives Isaac, the numpy sim and the dashboard (seed 7: 9 piles, ~600 pieces) | The site itself is procedural |
| 3D world, drone bodies, LiDAR geometry | Isaac Sim 6.0.1 on a cloud L40S ran the seed-7 scene, LiDAR returned points, drones moved (`docs/FROM_ISAAC.md`, 18:25) | Drones are moved kinematically by our code: no flight controller, no aerodynamics, no wind |
| SLAM | KISS-ICP 1.3 (real library) | Its input is our ray-cast LiDAR on the simulated terrain |
| Landing cells | Real plane-fit code on the map | Says nothing about structural safety |
| Radar | The breathing DSP chain, validated on real radar recordings | The radar returns: a **labelled synthetic** FMCW generator. Isaac does not model RF through rubble |
| Acoustic | Real models: Silero VAD, faster-whisper, YAMNet, SRP-PHAT/LCMV | The scenes: Piper TTS voices, numpy knocks, one ESC-50 dog clip, free-field pyroomacoustics propagation with a rubble low-pass |
| Agent / dispatch | Real code path, real validator and fallback; Gemini REST client tested against fake HTTP | No real Gemini call made yet (no key): every logged decision is `argmax` or `fallback` «GEMINI: …» |
| Dashboard | Real, polls the mission's `state.json` | — |

## 6. Evidence

Every number below is simulation unless it says "real data". Each has a source and a caveat.

**Radar DSP on real recordings** (`docs/RADAR_VALIDATION.md`)
- Breathing rate MAE **0.39 breaths/min** over 176 pooled 30 s windows; 91% within 1 bpm, 95% within 2 bpm.
- Data: Schellenberger et al. 2020 (CC BY 4.0), 3 resting subjects, 24 GHz CW radar about 0.4 m from the chest.
- **Caveat:** line of sight, no rubble, cooperative still subjects. It validates the signal processing only, not through-rubble sensing.

**SLAM and landing maps** (`python -m mapping.run --seed 7 --pile B2`, commit 110fdd2; metrics in the gitignored `dashboard/data/map_B2/metrics.json`)
- KISS-ICP trajectory error (ATE RMSE) **3.6 cm** over 300 scans, against **1.51 m** for odometry alone.
- Landing-cell map vs the true terrain: IoU **0.98** (precision 0.997, recall 0.985). All 16 landing sites from the SLAM map are valid on the true terrain (odometry alone: 11 of 16).
- **Caveat:** one pile, one seed, simulated LiDAR from our own ray-caster. 150 s of laptop CPU for 300 scans, run offline, not in real time.

**Planning vs fixed sites, single pile** (`sim/evaluate.py`, 30 seeds, 6-scan budget, model radar; `eval/results.json`)
- With planning, **43%** of runs resolve to one region, against **10%** with fixed landing sites.
- Mean 95% search area **100 m²** vs 285 m². Median position error 0.52 m vs 0.77 m.
- Truth inside the 95% region in 93% of runs for both. 0 false alerts for both.
- **Caveat:** provisional. Generated at 16:33 on older code, gitignored, single-target mock world. «B5-RERUN: …»

**Multi-pile dispatch** «EVAL-GRID: victims pinged within the budget, time to first ping, false leads; acoustic dispatch vs nearest-first vs raster; N seeds; acoustic stub or real.» Note: `sim/evaluate_grid.py` defaults to the acoustic **stub**, which builds clips from truth and is cleaner than the real pipeline. Say which one the table used.

**Acoustic front end on real audio** «ACOUSTIC-REAL: detection precision/recall on DroneAudioSet / ESC-50; Whisper word accuracy vs SNR under rotor noise.»

**Seed-7 demo run** (`dashboard/data/state.json`, run 18:58, `--radar dsp --acoustic auto`, real models from cache)
- Path A2 → A3 → B2 → C3. Three pings: A2-R1, A3-R1, B2-R1, each about 0.1–0.15 m horizontally from the hidden position (truth listed in `docs/review/2026-11-07-pass-1.md`). 24 of 24 radar scans, 745 s mission time.
- **What went wrong, and we say so:** the fourth hidden person (B1, faint voice saying "water") was **not heard at B1**. The pipeline attributed that voice to C3, an empty pile, which cost 12 radar scans and ended as an open lead. B1 ends the mission "not cleared (nothing heard)". This is the critical failure mode, and it is why the system never marks a pile clear. The acoustic assignment fix (BL-3) is in progress. «SEED7-RERUN: …» «ASSIGN: …»
- One seed, synthetic radar returns, simulated acoustic scenes.

## 7. Safety and failure modes

- **Missed person (critical).** No detection is not proof of absence. Silent piles end "not cleared", never "empty" or "cleared". The seed-7 run shows exactly this case at B1.
- **False lead.** An acoustic lead on an empty pile costs scans (seed 7: C3). Radar won't ping without three agreeing ranges, so the false lead ends as `insufficient_evidence`, not as a ping.
- **Over-trusting a ping.** A ping is a candidate search region with a belief share and a 95% region. Stationary does not mean trapped; periodic does not mean human. Depth is assumed, and labelled so.
- **Landing.** Flat landing geometry is not structural safety. A human decides whether anyone climbs onto a pile.
- **LLM.** It sees summaries only, picks from listed options, and is checked by a validator. On timeout, an invalid id, a battery-reserve or no-go violation, or a banned word ("confirmed", "survivor found"), the deterministic choice runs. Every decision records who made it.
- **Ground truth.** Only the observation generators and the evaluators read the hidden truth. Planner, localiser, agents and dashboard never do; tests strip `truth`/`targets` from prompts and check the boundary.
- **Battery.** The dispatch gate rejects options that leave a drone below a 25% reserve; drones swap batteries at the staging pads.
- **Not handled yet:** drone loss mid-mission, aftershocks, radio loss, dust. Say so if asked.

## 8. Where each component runs

The intended deployment split. Today everything runs on one laptop CPU (no GPU), and Isaac runs on a cloud L40S.

| Component | Runs on | Measured today |
|---|---|---|
| LiDAR odometry (KISS-ICP), pile map, landing-cell scoring | **On the drone** (Jetson-class target, not measured) | 0.38 s registration per 23k-point scan on the laptop CPU (300 scans: 114 s) |
| Radar breathing DSP (range FFT, phase, band-pass, peak, CFAR) | **On the drone** | not separately timed |
| Mic capture | **On the drone** | — |
| Bayes fusion, EIG planner, ping logic | **Staging edge box** (laptop or rugged PC at the staging area) | single-pile mission 0.7 s compute mean (`eval/results.json`) |
| Acoustic pipeline (association, LCMV, VAD, YAMNet, Whisper) and the acoustic agent's fallback scorer | **Staging edge box** | about 81 s wall per 26 s listen on the laptop CPU (review pass 1, cache miss); cached on replay |
| Dispatch gate, handover brief, dashboard | **Staging edge box** | — |
| Gemini (dispatch gate, per-clip triage, operator notes, brief rewrite) | **Cloud, optional** | expected 1–4 s per call, timeout 25 s; unmeasured «GEMINI: …». Offline: fallback and record/replay |

## 9. One-minute video storyboard

Matches the edit plan in `docs/DEMO.md`. Sources: `media/larpverse-grid-seed7.mp4` (concept animation, seed 7), Isaac footage «ISAAC-CLIP: …», `media/larpverse-dashboard-seed7.mp4` «DASH-MP4».

| Time | Shot | Voice-over | Caption on screen |
|---|---|---|---|
| 0:00–0:06 | Isaac wide shot: rubble field, 3 drones on the staging pad | "A collapsed building. Nine rubble piles. Where do rescuers dig first?" | "Simulated (Isaac Sim)" |
| 0:06–0:14 | Isaac: drones orbit A2, LiDAR sweep (else concept 0:00–0:14) | "Three drones map each pile with LiDAR and land on flat ground." | "Flat ≠ structurally safe" |
| 0:14–0:22 | Concept 0:19–0:27: ranges cross, ping | "Rotors off, radar looks for breathing under the rubble. Three ranges agree: a candidate search region." | "Synthetic radar returns · DSP validated on real recordings" |
| 0:22–0:32 | Concept 0:28–0:42 at ~1.4×: listening, transcripts, priority list | "Then they listen. Speech, knocks, dogs: an agent ranks the other piles by how human and how urgent." | "Simulated audio scenes · real models" |
| 0:32–0:42 | Dashboard 0:14–0:36 at ~2.2×: A2-R1, acoustic panel, dispatch to A3, A3-R1 | "This is the real pipeline on the same seed. Every ping and dispatch decision shows its evidence." | "Decision by: argmax / Gemini" |
| 0:42–0:47 | Isaac: drones fly to B2 | "The fleet moves to the next priority on its own." | — |
| 0:47–0:55 | Dashboard 2:06–2:11: ping inside the LiDAR model | "The operator opens the ping in the 3D map: where on the pile, with depth assumed, not measured." | "Depth assumed 1.5 m" |
| 0:55–1:00 | Final dashboard state, title card | "Candidates, not confirmations. Larpverse: search smarter, dig sooner." | "3 candidate regions · piles not cleared ≠ empty" |

Change from `docs/DEMO.md`: at 0:47 the plan says "roughly how deep". Depth is not measured, so the line above says "assumed".

## 10. Judge Q&A

1. **Does it detect survivors?** No. It produces candidate search regions with uncertainty, and a ranked list of piles to search next. A rescuer confirms.
2. **Can radar see through rubble?** Low-frequency UWB radar (below ~3 GHz) is the approach in the literature, with projects such as RADIOTECT aiming at 1–2 m of rubble; 24/77 GHz mmWave would not get through concrete (`docs/RESEARCH.md`). Our radar returns are a labelled synthetic generator. What we validated on real data is the breathing DSP: 0.39 bpm MAE, line of sight, no rubble.
3. **What's actually novel?** Not the sensors. It's the autonomy: drones choosing where to measure next by expected information gain, explicit uncertainty with an "insufficient evidence" outcome, and an acoustic agent that triages piles and drives dispatch.
4. **Why land instead of hovering?** Rotor noise swamps speech, and the radar needs a still platform for millimetre chest motion. Landing also gives three fixed, known viewpoints for fusing ranges.
5. **How accurate is a ping?** In the seed-7 run, 0.1–0.15 m horizontally from the hidden position. That is one seed, with synthetic radar. Depth is not measured; we show it as assumed. «EVAL-GRID» for more seeds.
6. **What if it misses someone?** That is the critical failure, and our demo run has one: a faint voice at B1 was attributed to the wrong pile. That's why no pile is ever marked clear; silent piles stay "not cleared", and the commander sees which ones.
7. **What happens with false alarms (a dog, a generator)?** YAMNet classes and the agent score them non-human (seed 7: dog → priority 1). A voice assigned to an empty pile costs scans (C3), but radar won't ping without three agreeing ranges.
8. **What does the LLM decide, and what if it's wrong?** It picks one option from the planner's list and writes a two-sentence rationale. A validator rejects unlisted ids, battery-reserve and no-go violations, and banned claims; then the deterministic choice runs. It never touches the maths or the flight.
9. **Does it need the internet?** No. Without a key or a network, everything falls back to deterministic rules, and recorded LLM answers replay offline.
10. **Has Gemini actually run?** Not yet; we're waiting on a key. Every decision in the demo is labelled `argmax` or `fallback`. «GEMINI: …»
11. **What runs on the drone?** LiDAR odometry, landing-cell scoring, radar DSP, mic capture. Fusion, planning, the acoustic pipeline and the agent run on an edge box at the staging area. The cloud is optional. On-drone latency is not yet measured on Jetson-class hardware.
12. **How is the audio real if the scenes are simulated?** The models are real (VAD, Whisper, YAMNet, direction finding). The scenes are synthesised: TTS voices, synthetic knocks, free-field propagation. «ACOUSTIC-REAL» covers real recordings with rotor noise.
13. **Is the Isaac footage the same mission?** Same seeded scenario (same piles, pieces and hidden positions) and the same mission code driving kinematic drones; Isaac ran it on a cloud L40S. The drones are not flown by a flight controller.
14. **How do you know the planner isn't cheating?** Only the observation generators and the evaluators read the hidden truth. Tests check that planner, localiser and prompts never see `truth` or `targets`.
15. **What would you need for a field test?** A real UWB radar module and a mic array on a drone, a flight controller (PX4) instead of kinematic motion, recordings on a USAR training rubble site to re-tune the acoustic association, and on-drone timing on Jetson.

## 11. Rubric map (weights from the team's v2 plan)

| Rubric (weight) | What we show | Honest gap |
|---|---|---|
| Perception-driven action (25%) | Rings disagree → EIG relocation; voice heard → pile priority → dispatch; dog/generator scored non-human | Radar returns synthetic; acoustic assignment error at B1 |
| Adaptation to change (25%) | Priorities re-ranked after every listen; `insufficient_evidence` instead of a forced fix; battery swaps | No live perturbation buttons (drone loss, aftershock) in this build |
| Operational autonomy (15%) | Whole mission runs without a human after the start pile; every decision logged with who made it | Edge box, not on-drone, for the acoustic stack |
| Relevance and impact (20%) | Matches the commander's triage problem: which pile, where on it, what's not cleared | «EVAL-GRID» is the impact number; not yet in |
| Ambition and technical quality (15%) | Real SLAM, real audio models, real-data DSP validation, Isaac 6.0.1 scene on GPU, bounded LLM | Kinematic flight; one-seed demo |
