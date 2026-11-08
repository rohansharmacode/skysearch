# Review pass 3: claims and consistency

Reviewer pass 3, 8 Nov 2026. Read against HEAD `22bbf4f` (VIDEO_PLAN as committed in `7f7fdbd`). Untracked work by other agents (`sim/impact.py`, `scripts/run_impact.py`, `docs/img/impact_*.png`) was not reviewed and must not be quoted until it is committed with its own doc. This file is the only one edited.

**Scope.** README, TECHNICAL_REPORT (+ abstract), PITCH_LARPVERSE, TECH_SUMMARY_LARPVERSE, SLIDES_OUTLINE, DEMO, ISAAC_BRIEF, ISAAC_QUICK, VIDEO_PLAN, EVAL_GRID, RADAR_PENETRATION, VITALS, RADAR_SIGNATURES, ACOUSTIC_VALIDATION, ACOUSTIC_GEMINI, DUMMY_AUDIO, HARDWARE, PLATFORM, COMMS_ARCHITECTURE, HOTSPOT, MOTION, SPEAKER, SITE_WEIGUAN, IMPACT_EVIDENCE, `data/impact_params.json`, PLAN decisions log, RESEARCH, the dashboard case-study popover (`dashboard/index.html:24-36`) and the HUD/dashboard condition strings.

**Headline.** The working docs (EVAL_GRID, RADAR_PENETRATION, VITALS, ACOUSTIC_*, IMPACT_EVIDENCE, VIDEO_PLAN) are mostly honest and current. The **judge-facing docs are not**. TECHNICAL_REPORT, its abstract, PITCH_LARPVERSE, TECH_SUMMARY, SLIDES_OUTLINE, DEMO and the README still describe the 7 Nov afternoon system: drones landing to scan, the optimistic `legacy` radar, the BL-3 bug at B1, a child vitals class and "Gemini never called". Anyone quoting them tonight would be wrong on reach, delivery, the seed-7 story, vitals and Gemini status.

## 0. Verified run and canonical numbers

**Seed-7 re-run done for this review.** It used the `tests/test_demo_story.py` configuration: the committed real-pipeline acoustic fixture, `tasking="default"`, `pod` delivery (the radar-on-the-rubble-surface scan model), `literature` penetration, `dsp` radar, physio waveform, SmoothFlight default. It ran on HEAD `sim/` through `scripts/heavy.sh`.

| Pile | Result | t (sim s) | Vitals condition | Depth |
|---|---|---|---|---|
| A2 | **ping A2-R1** | 364 | unconscious_slow | `radar_estimate` |
| B1 | **ping B1-R1** | 1712 | irregular_apneic | nominal |
| B2 | **ping B2-R1** | 2312 | calm | nominal |
| A3 | insufficient_evidence (open lead) | 2938 | – | nominal |

Visited A2 → A3 → B1 → B2. 34 scans, 3109 s mission time.

**This does not match the brief's "A2 and B1 ping; A3 and B2 stay open"** (`docs/runs/RESUME.md:90`). B2 pings in this configuration. The test itself (`tests/test_demo_story.py:52-55`) only asserts that A2 pings, because B1 and B2 sit at marginal SNR and depend on how well each radar seats on the rubble. **Fix:** commit one canonical seed-7 result, with its exact config (tasking, acoustic source, flight) and command, to a doc. Then quote it as "A2 always; B1 and B2 in this run; A3 open". The take runs real acoustics on live dummies, so its story can differ again. Also note: A2-R1 came back `radar_estimate`, which contradicts the "depth estimates stay nominal" claim at `RADAR_PENETRATION.md:86`.

**Canonical facts to quote** (source in brackets):

- **Radar reach** (`literature` is the default; heavy_sar, radar resting on the rubble; 45 s dwell; q 0.95; physio waveform):
  - **2.9 m beneath the landed radar**, 1.5 m at 6 m slant, 0.7 m at 11.5 m.
  - 20 s: 2.47 m. 60 s: 3.11 m. Legacy small radar at 45 s: 2.35 m. The old 3.6 m figure is withdrawn.
  - [RADAR_PENETRATION.md:26-34, 53]
- **Platform** [`sim/platform.py:248-268`, HARDWARE.md:12, 36-37, 296]:
  - The radar lives **in the belly radar module** under the drone: 4.9 kg, 0.48 × 0.48 m face (+7.6 dB).
  - Mission payload 14.0 kg.
  - Endurance 23.9 min to 0%, about 17.9 min usable with the 25% reserve.
- **Grid evaluation** [EVAL_GRID.md:1-35, 60-81; generated at `07f7ffb`, before SmoothFlight `fa4e74b`, so kinematic flight]:
  - 20 seeds, landed radar + `literature`, reported start: victims pinged excluding the start pile **acoustic 17/60 vs nearest 6/60 vs raster 5/60**. Per-seed share 28% [15, 42] vs 10% [3, 17] vs 8% [3, 15]. 0 false pings. 0/20 seeds with every victim pinged.
  - Independent start: 19/74 vs 10/74 vs 9/74.
  - Acoustics are the truth-fed **stub**, so the acoustic column is an **upper bound**.
- **Vitals** [VITALS.md:3]: condition only (calm / distressed_tachypnea / unconscious_slow / irregular_apneic / unknown). No age or child class.
- **Isaac** [ISAAC_VERIFIED_APIS.md:16]: Isaac Sim **6.0.1** (build 6.0.1-rc.7) + Isaac Lab 3.0 on a Brev L40S. The take has **not been rendered on a GPU yet** [ISAAC_BRIEF.md:12].
- **Uplink in Tainan** [HOTSPOT.md:3, HARDWARE.md:281, contracts:481]: **OneWeb via Chunghwa Telecom, or LTE**. Starlink was not commercially available in Taiwan; it is the generic option elsewhere.
- **Gemini** [ACOUSTIC_GEMINI.md:208, 294; `scripts/demo.sh:59-61`; `agents/llm.py:39`]:
  - The key is **free tier, about 20 requests/day per model**. `gemini-3.5-flash` was exhausted after 18 calls on 7 Nov.
  - Audio runs on `gemini-3.5-flash-lite`. The decider defaults to `gemini-3.5-flash`; demo.sh overrides it to `gemini-3.7-flash`.
  - No live Gemini **dispatch** decision is recorded: the cache has 0 decider rows.
- **Weiguan** [IMPACT_EVIDENCE.md:151-171, `impact_params.json`]:
  - M6.4 earthquake at 03:57, 6 Feb 2016.
  - **309 people in the building** (Pan 2019a: 76 uninjured, 118 injured, 115 dead).
  - **175 rescued alive** (164 / 7 / 4 on days 1–3). 290 extricated by rescuers (175 alive + 115 dead).
  - **115 dead**. Last live rescue at 62.3 h.
  - Do not derive "194 rescued" from 309 − 115: about 19 people got out without rescuers.

## 1. Stale or wrong claims, and internal contradictions

S = stale, W = wrong, C = contradiction between docs. Line numbers are at `22bbf4f`.

| # | Kind | Claim as written | File:line | Correct statement (source) |
|---|---|---|---|---|
| 1 | S | Radar default is `legacy`; "demo pings are produced under optimistic propagation" | TECHNICAL_REPORT.md:336, 675, 692; TECHNICAL_REPORT_ABSTRACT.md:15 | `literature` is the default with `--delivery pod` (the radar-on-the-rubble-surface scan model; RADAR_PENETRATION.md:53, 79). Legacy is a comparison row only |
| 2 | S | Reach table: q 0.9 at 2 m = 2.9 m, thresholds −17/−15/−11 dB (old 4–8 mm sinusoid, 20 s dwell, landed drone) | TECHNICAL_REPORT.md:321-328 | Replace with the landed-radar table: 2.92 / 1.50 / 0.70 m at 2 / 6 / 11.5 m (45 s, q 0.95), and the A90 table per dwell (RADAR_PENETRATION.md:17-33) |
| 3 | S | "≈3 m" / "about 3 m" reach | ISAAC_QUICK.md:40; FOR_ISAAC_CLAUDE.md:228; PLAN.md:188, 203; dashboard/index.html:95; EVAL_GRID.md:16, 62, 85; TASKING_INTEGRATION.md:63, 169; PLATFORM.md:58 | "about 2.9 m beneath the landed radar at a 45 s scan; under 1 m from 11 m away" (RADAR_PENETRATION.md:31). "≈3 m" is acceptable only as a rounded literature range; never use 3.6 m |
| 4 | S | Drones **land** on flat cells and scan with radar; "Why land instead of hovering?" | README.md:5-6, 8; TECHNICAL_REPORT.md:102, 253-266; ABSTRACT:5; PITCH_LARPVERSE.md:42-48, 142; TECH_SUMMARY_LARPVERSE.md:8-10; SLIDES_OUTLINE.md:32 | Correct as written: the heavy drone **lands on a flat spot** found on its LiDAR map, and its **belly radar (4.9 kg radar + mic module)** scans through the rubble from there: about 8 s to land, 45 s scan, 8 s to lift off. The drones also land off-pile with rotors off to **listen** (RADAR_PENETRATION.md:62-71; PLAN.md:203) |
| 5 | S | 0.6 × 0.5 m radar panel, 4 kg, under the belly; SAR payload 12 kg; endurance 24.6 min, 18.5 min usable | PLATFORM.md:23, 25-26, 30, 67, 206, 231; HARDWARE.md:294, 296 (sim column), 302-307 (mismatches 1-2 are now resolved) | Belly radar face 0.48 × 0.48 m (+7.6 dB) replaces the old 0.6 × 0.5 m panel, payload 14.0 kg, 23.9 min / ≈17.9 min usable (`sim/platform.py:248-268`; commit `6dc3a18`). ISAAC_ASSETS.md:133 and ISAAC_QUICK.md:43 say the radar module case is 0.5 × 0.5 m: align them on the 0.48 m face |
| 6 | S/C | Seed-7 story: "pings A2, A3, B2" (19:20 run), "B1 missed (BL-3)", C3 a false lead | PITCH_LARPVERSE.md:90-93, 97-98, 143-145, 159; TECH_SUMMARY_LARPVERSE.md:42, 46; DEMO.md:12-15, 18-19, 54-59, 97-101, 132-134; TECHNICAL_REPORT.md:454, 656-661, 671, 673; SLIDES_OUTLINE.md:36; SPEAKER.md:496-499; TASKING_INTEGRATION.md:51-53 | Use §0: A2-R1 (t 364), B1-R1 (1712), B2-R1 (2312); A3 insufficient_evidence (shallow 33/min breather, rocking seats). BL-3 is fixed (`26de7e9`): B1 is heard and ranked |
| 7 | S/C | "seed 7 pings **all four** victims (A2, A3, B1, B2)" | ISAAC_QUICK.md:41 (and `056029d` message) | ISAAC_QUICK is superseded (ISAAC_BRIEF.md:4); add a banner. A3 does not ping under the physio waveform |
| 8 | C | Kinematic-flight seed-7 result "B1 at 1247 s, B2 at 1709 s, A2 at 2502 s on a revisit" | RADAR_PENETRATION.md:81-84 | Superseded by SmoothFlight (`fa4e74b`). Replace with §0 and say which flight model |
| 9 | C | "Depth estimates stay nominal" | RADAR_PENETRATION.md:86 | A2-R1 is `radar_estimate` in the §0 run; EVAL_GRID.md:33 also reports 5–8 radar depth estimates per 20 seeds |
| 10 | C | Brief says "A2 and B1 ping; A3/B2 open leads" | docs/runs/RESUME.md:90 | Not reproduced by the committed demo-story config (§0). State the config, or correct it |
| 11 | S | Eval: no numbers / "«EVAL-GRID»"; the 43% vs 10% single-pile result as the planning headline | TECHNICAL_REPORT.md:630-632, 644; PITCH_LARPVERSE.md:80-86, 162; TECH_SUMMARY_LARPVERSE.md:39-40, 45; SLIDES_OUTLINE.md:35; ABSTRACT:12 | Headline = EVAL_GRID 17/60 vs 6/60 vs 5/60 with CIs, stub = upper bound, kinematic flight. The 43%/10% result is from 7 Nov 16:33 on older code and model radar; label it "older code, not re-run" or drop it |
| 12 | W (confusable) | "17/60" appears twice in EVAL_GRID: acoustic under the landed radar (line 21) and **nearest** under legacy (line 113) | EVAL_GRID.md:21, 113 | Always quote with its row: "17/60 (acoustic, landed radar, literature)". Never quote the legacy 42/60 |
| 13 | S | EVAL_GRID header doesn't state the flight model | EVAL_GRID.md:3 | Add "kinematic flight (generated at `07f7ffb`, before the SmoothFlight default)" |
| 14 | S | Vitals: `child_calm` state; "Seed 7 … A2 distressed tachypnoea, A3 calm, B1 irregular apnoeic, B2 child"; "possible child" capped at 0.45; "low-confidence child flag" | TECHNICAL_REPORT.md:369, 371, 383, 612, 678; PLAN.md:189 (superseded by :202) | No age or child class. Seed 7: A2 unconscious_slow, A3 distressed_tachypnea, B1 irregular_apneic, B2 calm (PLAN.md:202; VITALS.md:3) |
| 15 | S | Chest displacement 4–8 mm at a 20 s dwell | TECHNICAL_REPORT.md:266, 281, 284 | Physio waveform: about 5 mm peak-to-peak for a calm adult, 2–3 mm when rapid and shallow. 45 s landed radar scan, plus a 60 s vitals look (RADAR_PENETRATION.md:14, 70) |
| 16 | S | Flight is kinematic; straight legs at 4 m/s | README.md:44, 55; TECHNICAL_REPORT.md:157, 201, 695; PITCH_LARPVERSE.md:58, 163; TECH_SUMMARY_LARPVERSE.md:52; ABSTRACT:15 | SmoothFlight is the default: jerk-limited C2 trajectories, 10 m/s cruise, tilt from acceleration, hover drift (MOTION.md). It is still **not** a flight controller or aerodynamics. Say "smooth, physically plausible trajectories; no flight controller, aerodynamics or wind loads" |
| 17 | S | "Gemini has not been called / no key / every decision argmax or fallback" | PITCH_LARPVERSE.md:63, 118, 148; TECH_SUMMARY_LARPVERSE.md:30, 54; TECHNICAL_REPORT.md:205, 594, 697 | Gemini **audio** ran live: 251 clips on `gemini-3.5-flash-lite`, 0 fallbacks, cached (ACOUSTIC_GEMINI.md:208-212). The **dispatch** decider still has no recorded live decision |
| 18 | C | "Gemini usage unlimited (temporary hackathon key)" | PLAN.md:192 | Free tier, about 20 requests/day per model (ACOUSTIC_GEMINI.md:294; RESUME.md:29; demo.sh:59). Needs a new decision row (orchestrator) |
| 19 | C | Quota per model: "Lite models have more" vs "~20 requests/day PER MODEL" | ACOUSTIC_GEMINI.md:295 vs scripts/demo.sh:59 | Measured: flash 20/day (hit after 18); flash-lite answered 251+ calls on 7 Nov. State it as measured per model, and drop the blanket "20/day per model" or the "Lite has more" |
| 20 | C | Decider model: `gemini-3.5-flash` (AGENTS.md:31, TECHNICAL_REPORT.md:570, 808, PLAN.md:178, `agents/llm.py:39`) vs `gemini-3.7-flash` (demo.sh:61) | scripts/demo.sh:61 | Choose one. `gemini-3.7-flash` appears in no doc and is unverified here; check the model ID resolves before the demo, or the decider silently falls back |
| 21 | S | Gemini audio stage "not yet wired into run.py or the mission" | TECHNICAL_REPORT.md:18, 421 | Wired into `run.analyse` at `26de7e9` (ACOUSTIC_VALIDATION.md:7) |
| 22 | S | Acoustic real-audio validation "in progress / once it lands" | README.md:52; TECHNICAL_REPORT.md:17, 204, 456, 633, 652, 693; PITCH_LARPVERSE.md:88, 150 | Landed: ACOUSTIC_VALIDATION.md §5-6 (fill values in §4 below) |
| 23 | S | "Isaac 5.1 launchable"; "the ready-made Launchable ships 5.1" | RESEARCH.md:9, 30-31; PLAN.md:50 | Brev launchable ships Isaac Sim 6.0.1 + Lab 3.0 (ISAAC_VERIFIED_APIS.md:16). Add a superseded note to RESEARCH |
| 24 | S | "Starlink or OneWeb" as the cloud path; van "Starlink-style flat panel" / "Starlink panel" | COMMS_ARCHITECTURE.md:12; ISAAC_ASSETS.md:52, 167; isaac/van.py:16, 865 | Tainan: OneWeb via Chunghwa, or LTE; Starlink only where licensed (HOTSPOT.md:3). Call the prop a "satellite terminal" in captions and docs |
| 25 | C | Van mast height: 6 m in the Isaac model | ISAAC_BRIEF.md:118; ISAAC_ASSETS.md:51; isaac/van.py:15 | 9–12 m pneumatic mast in HARDWARE.md:280 and COMMS_ARCHITECTURE.md:66; 10.5 m in the link budget (HOTSPOT.md:51). Either model 10.5 m or caption it "mast shown lowered" |
| 26 | S | "The ISAAC_BRIEF gate 'A2 ping before t 110'" | VIDEO_PLAN.md:84, 340; MOTION.md:130; FOR_ISAAC_CLAUDE.md:85 | Obsolete: A2 pings at t 342 (VIDEO_PLAN.md:53) or t 364 (§0, default tasking). Delete it from the briefs |
| 27 | C | Trigger window: "presses Y **during the listen** … about 7 s of video later"; final take "during the listen window (about 33–40 s)" | ISAAC_BRIEF.md:23, 111 | VIDEO_PLAN.md:25, 55, 71-75, 247 (real pipeline): press **while the drones are landed on A2, video ≈17–29 s**. A press after ≈33 s forces a fresh full pipeline run (about 50–62 s on the laptop CPU; unmeasured on the L40S), and the mission clock holds. The 6.8 s reaction (VIDEO_PLAN.md:65-66) was measured with **stub** acoustics |
| 28 | S | Status 🟡 "mission switches to SmoothFlight in the next push", `pods_max_visible` = 0 in --live, "real acoustic pipeline landing" | ISAAC_BRIEF.md:54-56 | Landed: `fa4e74b` (SmoothFlight), `7f7fdbd` (`pods` counted, real-pipeline AcousticService). Update the marks |
| 29 | S | OBS: "Set the dashboard to its narrow layout: it is checked at 400 px" | VIDEO_PLAN.md:234 | Use `?layout=take` (verified at 640×1080, 960×1080 and 1920 in `c19bbc1`), as ISAAC_BRIEF.md:97 already says |
| 30 | C | On-screen transcript "Tolong! … kaki saya terjepit" for the A1 dummy | ISAAC_BRIEF.md:40 | With Whisper `base.en` (the take default) A1's Indonesian clip came back with **no transcript** (VIDEO_PLAN.md:80-82). Show the transcript only if a multilingual model or Gemini runs in the take |
| 31 | C | Primary demo site: "Collapse site (primary demo)" | SITE_WEIGUAN.md:8, 17; PLAN.md:200 | The take is timed and planned on `--site grid` (VIDEO_PLAN.md:23, 212). The collapse-site geometry *and* textures derive from NCKU screenshots (SITE_WEIGUAN.md:100-108; PLAN.md:201), so it can't be shown publicly without permission. Make grid the public demo |
| 32 | C | Acoustic array: 6-mic, 0.3 m ring in the simulation | TECHNICAL_REPORT.md:71, 391; acoustic/scene.py:25-26 | The hardware, the Isaac model and the platform spec use an **8-mic 1.0 m ring** (HARDWARE.md:26; ISAAC_BRIEF.md:124; PLATFORM.md "about 20° vs 65°"). Say "bearings simulated with a smaller 6-mic 0.3 m array (conservative)" |
| 33 | C | Commercial radar life detector range: "about 4.6 m (15 ft)" vs "4.5–7 m" / central 5.5 m "through rubble" | RADAR_PENETRATION.md:43 vs IMPACT_EVIDENCE.md:25 and `impact_params.json` `radar_breathing_range_m` | One wording everywhere: "vendor claims 4.5–7 m (GSSI LifeLocator, trade press); not independently verified". Expect the judge question "your 2.9 m is worse than a LifeLocator": answer that our model is conservative and literature-calibrated, and that our gain is placements per hour without people on the pile, not range |
| 34 | W | "Every 1 m cell is scored" | TECH_SUMMARY_LARPVERSE.md:8 | 0.5 m cells with a 1.5 m plane-fit neighbourhood (TECHNICAL_REPORT.md:241) |
| 35 | S | Model size "about 900 MB" | README.md:28; TECHNICAL_REPORT.md:738 | VIDEO_PLAN.md:166, 183: about 210 MB of models, and `acoustic/models/` is 314 MB. 900 MB is probably the whole `.venv-audio`; say which |
| 36 | S | Dev server `python3 -m http.server 8765` | README.md:22-24; TECHNICAL_REPORT.md:744 | `scripts/serve.py` (static + `/api/tasking`, `/api/portal`, `/api/broadcast`); http.server can't save tasking (PLAN.md:196) |
| 37 | S | "63 test functions in 7 files"; "BL-1 … BL-20"; "8–9 parallel builders" | TECHNICAL_REPORT.md:715, 722, 707 | 250 test functions in 22 files at `22bbf4f`; backlog to BL-27; up to 10 concurrent agents (PLAN.md:194) |
| 38 | S | HOTSPOT status "orchestrator applies" for the ops API and mission integration | HOTSPOT.md:15-17 | Landed in `cbe715f` and `82ac638` |
| 39 | S | Condition wording "slow regular breathing (possibly unconscious) - check first" | isaac/hud.py:48; ISAAC_BRIEF.md:38; VIDEO_PLAN.md:53 | Dashboard wording (`dashboard/vitals.js:40`): "slow, regular breathing: deep rest **or** reduced consciousness (the radar cannot tell which); call out and check responsiveness". HUD and dashboard sit side by side in the take, so they must match. See also overclaim O4 |
| 40 | S | MOTION story table "plus an A3-R1 ping" | MOTION.md:126 | Stub-acoustics run; label "stub acoustics". The physio + real-fixture story has A3 open |
| 41 | S | Dashboard popover: "search declared over on 13 Feb" | dashboard/index.html:30; TECHNICAL_REPORT.md:54 | Fine as written. Optionally add "(Tainan Fire Bureau: 14 Feb)" (IMPACT_EVIDENCE.md:163). The popover's 115 deaths and building facts match IMPACT_EVIDENCE §4 |
| 42 | S | TECHNICAL_REPORT §1.4 cites zh.wikipedia only | TECHNICAL_REPORT.md:50-55 | Add 309 / 175 / 115 with Pan 2019a and the Tainan FB e-paper (IMPACT_EVIDENCE.md:156-161). zh.wiki's "~200" residents is stale |
| 43 | S | FHWA "exact document to be cited" | TECHNICAL_REPORT.md:787; RADAR_PENETRATION.md:10, 40 | Still uncited: RADAR_PENETRATION names "the FHWA rule of thumb" with no document or URL. The RADIOTECT "≈2.7 m field tests" also has no link. Add both before quoting 10–20 dB/m as sourced |
| 44 | S | Docs that predate the deliverable but read as current | DEMO.md (whole: multi-cut video plan, BL-3, `--piles 3`); PITCH_LARPVERSE.md §9 storyboard; SLIDES_OUTLINE.md:15-21 (contact sheets from the first lv2 run) | PLAN.md:205 retired the cut plan for the deliverable. Put a "SUPERSEDED by VIDEO_PLAN.md / ISAAC_BRIEF.md" banner on each, or rewrite them |

## 2. Overclaims: safer wording

| # | Where | As written (or likely to be said) | Problem | Safer wording |
|---|---|---|---|---|
| O1 | IMPACT_EVIDENCE.md:32 | "the system **clears empty sectors** faster, **finds the silent victims**" | Contradicts our core rule that no pile is ever cleared. "Finds the silent victims" is shown only in simulation, within about 2.9 m, and the A3 shallow breather was missed | "In simulation, the system de-prioritises sectors with no evidence sooner (searched, **not cleared**), and its landed radar can flag a silent breathing person within about 2.9 m of rubble beneath it, which listening devices cannot" |
| O2 | IMPACT_EVIDENCE.md:27 | "brings breathing radar … to **many points per hour**" | Unquantified | "about one landing per minute of dwell (8 s to land, 45 s scan, 8 s to lift off) plus flight between placements; simulated timing" (RADAR_PENETRATION.md:70) |
| O3 | Any pitch line, VIDEO_PLAN/ISAAC_BRIEF captions | "detects unconscious victims", "hears survivors", "finds people" | Radar returns are a synthetic generator; the audio is synthesised from dummy clips | "flags candidate search regions **in simulation**: radar returns come from a labelled synthetic generator processed by a real DSP chain" |
| O4 | isaac/hud.py:48; ISAAC_BRIEF.md:38; VIDEO_PLAN.md:53 | "possibly unconscious" | On **real** data, healthy relaxed volunteers read "slow, regular" (the unconscious_slow class) in 38/57 windows (VITALS.md:120-126) | "slow, regular breathing: deep rest or reduced consciousness; check responsiveness · aid, not a diagnosis" |
| O5 | README.md:51; PITCH_LARPVERSE.md:61; TECHNICAL_REPORT.md:202 | "DSP chain, **validated** on real recordings" | The real data is line of sight, 24 GHz CW, 0.4 m, 3 (or 6) cooperative healthy subjects, no rubble | "breathing DSP checked on real line-of-sight 24 GHz recordings (no rubble): 0.39 bpm MAE" |
| O6 | ACOUSTIC_VALIDATION.md:3 ("work on real recordings") and any "validated acoustics" | | Single-channel, rotors off, near-field Freesound/ESC-50 clips, nothing through rubble. The rotor veto (`f1e3b44`) was **not re-validated** (ACOUSTIC_VALIDATION.md:152, 229) | "recognisers checked on real recordings (rotors off, single mic, no rubble): human-vs-non-human F1 0.70–0.82 by set" |
| O7 | Eval headline in any pitch | "2.8× more victims found" / "3× better" | 17/60 vs 6/60 uses truth-fed stub acoustics, kinematic flight, 20 seeds, wide CIs. Time to first ping is **not** better in the independent-start set (1413 s vs 1042 s; EVAL_GRID.md:69) | "In 20 simulated seeds, acoustic-led dispatch pinged 17 of 60 non-start victims against 6 and 5 for radar-only baselines (28% [15–42] vs 10% [3–17]); acoustic clips were generated from ground truth, so this is an upper bound. 0 false pings." Never claim it is faster |
| O8 | Any lives-saved figure (e.g. from untracked `sim/impact.py`) | single numbers of lives saved | IMPACT_EVIDENCE.md:169-171 says "N victims could have been saved" is unsupported | Ranges only (low / central / high), labelled "counterfactual model, not a claim about Weiguan", citing the [A] assumptions; nothing until the impact doc is committed |
| O9 | HARDWARE.md:9, 13 | "**Every** sensor is a part you can buy today"; "~$61k / $87k per drone" | The belly radar (B210 SFCW, absorber cavity, firmware) is a custom design that was never built or tested. "quote" in the price column means "price on request", not a quote we hold | "All sensors except the custom radar module (designed, not built) are off-the-shelf. Prices are estimates from list or reseller prices, Nov 2026; no supplier quotes obtained" |
| O10 | ISAAC_BRIEF.md:35 | "They move like real multirotors" | Trajectory model; no flight controller or aerodynamics | "smooth, physically plausible multirotor trajectories (jerk-limited, tilt from acceleration); not a flight-dynamics simulation" |
| O11 | VIDEO_PLAN.md:7-8; ISAAC_BRIEF.md:23 | "unscripted", "an unplanned change genuinely perceived" | The hidden dummies (H2 at A1, H1 at C3), their clips and the hotkeys are pre-staged, and the press window is rehearsed. Only the **moment** and the choice of trigger are live | Caption: "Live, unrehearsed moment: a teammate switches on a pre-placed hidden dummy (key Y) at a time they choose. The system hears it through the real acoustic pipeline on synthesised audio and re-plans." Say "time-lapse ×N" too |
| O12 | VIDEO_PLAN.md:10 "real perception" | "Real perception, not the stub" | The models are real; the audio is synthesised from dummy WAVs through a rubble model, and the radar IF is synthetic | "real recognisers (VAD, YAMNet, Whisper, direction finding) on synthesised scene audio; synthetic radar IF through the real DSP" |
| O13 | Any "Gemini agent decides" caption in the take | | The box runs without `.env` by default, so the decision is the deterministic scorer (VIDEO_PLAN.md:179) | Show the actual `by` badge: "decided by: rule-based fallback" or "Gemini" |
| O14 | IMPACT_EVIDENCE.md:25 | "Radar life detectors sense breathing to about 4.5–7 m [S]" | Vendor and trade-press claims, tagged [S] like peer-reviewed figures | "vendors claim 4.5–7 m (GSSI, trade press; not independently verified)" |
| O15 | VIDEO_PLAN.md:7 | "Nothing is … pre-recorded" | Valid for the stream, but the HUD can fall back to a library signature (`isaac/hud.py:231-245`) | It is labelled "NOT this scan" / "reference signature (no live scan)". Keep that label visible in the take, or say it in the caption |
| O16 | Hotspot claims (HOTSPOT.md) | phone localisation accuracy | 0.6 m median landed-around vs 7.1 m survey-only; the first fit on seed 7 was in the wrong sector (HOTSPOT.md:106) | Quote both numbers and the wrong-sector case; "consent-based; synthetic phones" |

## 3. Licence and credit obligations

| Asset | Licence / terms | Where it's used | Obligation | Status |
|---|---|---|---|---|
| **ESC-50** clips (K. J. Piczak 2015; audio from freesound.org) | CC BY-NC 3.0 | Committed: `assets/audio/dummies/{cough,cry,knock_door}_esc50.wav` + 12 rubble variants (modified); evaluation in `data/` (gitignored) | Credit Piczak **and** the Freesound authors of 101336, 108017 and 151080 (`data/audio_real/esc50/meta`). Mark the rubble variants as modified. **Non-commercial only**: no use in a commercial product or paid pitch material. Mirrored to the team GitHub, so add an attribution file next to the WAVs | README has no credit. Add a `CREDITS` note in `assets/audio/dummies/`, plus the video description (ISAAC_BRIEF.md:127; DUMMY_AUDIO.md:59) |
| **FLEURS** (Conneau et al. 2022) | CC BY 4.0 | Gemini eval only; data gitignored | Cite it wherever the ACOUSTIC_GEMINI numbers are quoted | Cited in ACOUSTIC_GEMINI.md:163, TECHNICAL_REPORT.md:801 |
| **Schellenberger et al. 2020** (Sci Data 7:291) | CC BY 4.0 | Real radar validation; committed plots derived from it (`docs/radar_real_validation.png`, `docs/vitals_validation.png`) | Attribution with DOI and licence wherever the numbers or plots appear, including the video if 0.39 bpm is shown | OK in README.md:51 and the docs. Put the DOI in the video description |
| **DREGON** (Strauss et al., IROS 2018) | "personal, educational and academic use only"; cite the paper; no redistribution. Its speech is TIMIT utterances (LDC-licensed) | DoA validation only; data gitignored in `data/audio_real/dregon/` | **Never commit or redistribute DREGON audio, and never put it in the video.** Results may be cited as academic validation with the paper citation. Don't use them as evidence in a commercial pitch; re-validate on our own recordings if this goes commercial | Kept local. Confirm `data/` stays ignored in the mirror to `rohansharmacode/skysearch` |
| **DroneAudioSet** (ahlab-drone-project, HF 2025) | MIT | Evaluation only (rows streamed, gitignored) | Cite it. The MIT notice applies only if we redistribute the data (we don't) | OK |
| **Frances (F. Valisalo) set**: Freesound/Wikimedia clips | CC0 per `frances/manifest.csv` | Committed dummy voices, dog, wind, siren | No obligation; courtesy credit to the Freesound authors (Gemesil, Legnalegna55, Bee_Abney, DRFX, darranambler, FunWithSound, brunoboselli) | Listed in DUMMY_AUDIO.md:26-55 |
| **Piper TTS voices** (rhasspy/piper-voices) | Code MIT. **Each voice follows its dataset:** `en_US-lessac-low` (Blizzard 2013 Lessac data: research or non-commercial terms, per DUMMY_AUDIO.md:61, from memory), `vi_VN-vivos-x_low` (VIVOS, CC BY-NC-SA 4.0 per the same note), `zh_CN-huayan-x_low` and `id_ID-news_tts-medium` (**unverified**; no MODEL_CARD on disk) | Committed TTS WAVs (en/zh/id/vi voices and weak voices + variants); speaker call-outs | Check each MODEL_CARD before public release. Credit "Piper (rhasspy), voices lessac / huayan / news_tts / vivos" in the description. If vivos is BY-NC-SA, the vi clips must carry BY-NC-SA and stay non-commercial | Open (DUMMY_AUDIO.md:60-62; TECHNICAL_REPORT.md:687) |
| **Gemini TTS** (nan-TW reassurance line) | Google Gemini API terms; AI-generated audio | `acoustic/data/speaker/` (gitignored); possibly heard in the take | Disclose "machine TTS, not native-verified" if it is audible; prefer text-only | SPEAKER.md:144 marks it unvetted |
| **NCKU Museum photogrammetry** ("VR-Weiguan Collapsed Building", Sketchfab) | No stated licence; possible NoAI tag | The collapse-site core **geometry and textures** were derived from screenshots | **No public showing of `--site collapse`** (even with `--no-scan-textures`, the geometry is derived) without NCKU permission. Use `--site grid` for the video | PLAN.md:201; SITE_WEIGUAN.md:100-108; RESUME open item |
| **OpenStreetMap** tiles | ODbL; tile usage policy | Dashboard basemap (Map mode) | "© OpenStreetMap contributors" visible whenever tiles show | Implemented (`dashboard/app.js:183, 240`). Keep `#attrib` visible in the take if a basemap is on |
| **Esri World Imagery** | Esri terms | Dashboard Satellite mode | "Imagery © Esri, Maxar, Earthstar Geographics, and the GIS User Community" visible | Implemented (`dashboard/app.js:186`) |
| **NVIDIA Isaac Sim 6.0.1 / Isaac Lab 3.0, MDL materials, HDRI** (`evening_road_01_4k`, shipped in NVIDIA Assets, originally Poly Haven CC0) | NVIDIA Omniverse licence; trademark guidelines | Rendering | Credit "Rendered in NVIDIA Isaac Sim". Don't imply NVIDIA endorsement | Add to the description |
| **Open-source models and libraries**: Silero VAD (MIT), YAMNet (Apache-2.0, AudioSet ontology CC BY-SA 4.0 / labels CC BY 4.0), Whisper / faster-whisper (MIT), KISS-ICP (MIT), Open3D (MIT), pyroomacoustics (MIT), noisereduce (MIT), three.js (MIT) | as listed | Pipeline, dashboard | Credit in the README and description. MIT/Apache notices are required only if we redistribute the code or weights. YAMNet and Silero weights sit in `acoustic/models/`: check that they are not committed | Partly credited (TECHNICAL_REPORT §12) |
| **Weiguan case-study facts** | Public sources | Dashboard popover, docs | Cite the Tainan Fire Bureau e-paper, Pan 2019 (JFMA), Chiu & Tseng 2018, Taipei Times and zh.wikipedia. It is a fatal disaster (115 dead): keep "layout synthetic" on every map | Done in the popover and IMPACT_EVIDENCE |
| **Brands**: DJI FlyCart 30 / Agras T50, Ouster, Harris Aerial | Trademarks | Docs, Isaac drone model | Say "FlyCart 30-class", not "DJI drone". No DJI logos on the model | Van livery is generic (ISAAC_BRIEF.md:116) |

## 4. TECHNICAL_REPORT «PENDING» placeholders: what fills each

| Marker (report lines) | Fills from | Value to insert |
|---|---|---|
| **EVAL-GRID** (16, 632, 644; also PITCH «EVAL-GRID») | `docs/EVAL_GRID.md` (`07f7ffb`) | 20 seeds, landed radar + `literature`, reported start, victims pinged excluding the start pile: **acoustic 17/60, nearest 6/60, raster 5/60**. Per-seed share 28% [15, 42] / 10% [3, 17] / 8% [3, 15]. Independent start 19/74 / 10/74 / 9/74. 0 false pings. Visits to empty or non-human piles 0.7 vs 2.8 vs 2.5. Ping error median 0.32 m. **Stub acoustics (truth-fed) = upper bound; kinematic flight.** Landing-ring rows: 0/60, 0/60, 1/60, which is why the radar sits on the rubble |
| **ACOUSTIC-REAL** (17, 204, 456, 633, 652) | `docs/ACOUSTIC_VALIDATION.md` §5-6 (`26de7e9`, assoc-v11) | Human vs non-human, agent P/R/F1: ESC-50 0.69/0.71/0.70 (AUC 0.88); teammate CC0 clips 0.79/0.85/0.81 (AUC 0.97); DroneAudioSet rotors off 0.90/0.76/0.82; + rotor 0 dB 0.90/0.43/0.58. "help" transcribed in 7/13 clips. DoA on real DREGON: rotors off 100% within 10° (static, n=192), 90% hand-carried; rotors on in real flight 17–24%. That supports land-and-listen. The rotor veto (`f1e3b44`) is not yet re-validated |
| **ACOUSTIC-GEMINI** (18, 443, 634) | `docs/ACOUSTIC_GEMINI.md` | `gemini-3.5-flash-lite`, prompt v5, 7 Nov: language ID 89/90 clean, 78/90 noisy (FLEURS, 9 languages). Distress-cue recall 0.76, precision 1.00 (24 TTS phrases). Invented cues 4/180 news clips. Latency median 1.6–1.8 s per clip. Knocks rarely called human (0/5). Offline fallback (base.en): 0/10 language ID for zh/id/vi. Also fix line 421: it **is** wired into `run.analyse` |
| **VITALS** (19, 203, 385, 635) | `docs/VITALS.md` | Synthetic radar (45–60 s): recall calm 95%, distressed 100%, unconscious_slow 84%, irregular_apneic 65%; pauses detected in 78% of 27 scans, 3% false. **Real** Schellenberger (line of sight): rate MAE 0.51/min (60 s windows, n=57) and 0.25/min (30 s, n=118); 6/6 voluntary breath holds detected, 0 false pause events. Caveat: healthy relaxed volunteers read as slow-regular in 38/57 windows. Remove `child_calm` and age (lines 369-383) |
| **RADAR-PEN** (20, 336, 787) | `docs/RADAR_PENETRATION.md` | Default `literature` + landed radar. Reach 2.92 / 1.50 / 0.70 m at 2 / 6 / 11.5 m (45 s, q 0.95). A90 −9 / −6 / +3 dB at 45 s. Loss 15 / 29 / 43 / 69 dB at 1 / 2 / 3 / 5 m. Solid fraction 0.65. The old 3.6 m was optimistic by about 0.7 m. **Still open:** the specific FHWA document and the RADIOTECT link |
| **SITE-WEIGUAN** (21, 192) | `docs/SITE_WEIGUAN.md` | Procedural stand-in assumptions plus the collapse site (320 m, from the teammate). Licence: geometry and textures derive from NCKU screenshots, so no public showing without permission |
| **B5-RERUN** (22, 630) | `python -m sim.evaluate --n 30 --budget 6` | **Still open**: `eval/results.json` is dated 7 Nov 16:33. Re-run, or move 43%/10% to "older code" |
| **SEED7-RERUN** (23, 661) | This review §0 (should become a committed doc) | A2-R1 t 364 (unconscious_slow), B1-R1 t 1712 (irregular_apneic), B2-R1 t 2312 (calm), A3 insufficient_evidence; 34 scans, 3109 s; SmoothFlight, fixture acoustics, default tasking. The A2-only, early-ping take timeline is in VIDEO_PLAN.md:53 (t 342) |
| **ISAAC-CLIP** (24, 181) | Teammate's GPU take (`FROM_ISAAC.md`) | **Still open**: "Nothing below has been rendered on a GPU yet" (ISAAC_BRIEF.md:12) |
| **GEMINI-LIVE** (25, 116, 205, 594, 636) | ACOUSTIC_GEMINI.md:208-212, 270-281 | **Partly filled**: Gemini audio ran live (251 clips, 0 fallbacks, median 1.6–1.8 s); the e2e seed-3 run used 3 audio calls + 1 rationale. The dispatch decider has **no** recorded live decision (0 decider rows in `agents/cache/llm.jsonl`). The free-tier quota of about 20/day per model limits live use |

Also stale in the report but not marked PENDING: §2.1 (6-mic), §3.2 (kinematic flight), §4.1.3 (perches and landing quality instead of the radar's seating on the rubble), §4.2.1 (20 s dwell, 4–8 mm), §5.6 ("planned" tasking and recon, which landed in `ec0c696`), §7 (child flag), §9.1 rows "penetration overestimated" and "triage hint", §10.1 (counts). The status line (line 6, "written against `9907ad6`") needs bumping.

## 5. Ready-to-paste "Credits & honesty" block

For the README (new section) and the video description. Facts only from the docs above. The `[…]` fields need a human check before posting.

```markdown
## Credits & honesty

**What is real and what is simulated.** Everything in the video is a simulation (NVIDIA Isaac Sim 6.0.1).
- Rubble, victims (training-manikin dummies) and their positions are synthetic. The case-study framing is
  the Weiguan Jinlong collapse, Tainan, 6 Feb 2016 (115 dead); the site layout shown is synthetic.
- Radar returns are a labelled synthetic generator. The breathing-detection DSP that processes them was
  checked on real line-of-sight radar recordings (Schellenberger et al. 2020: 0.39 breaths/min MAE; no rubble).
  Modelled reach through rubble: about 2.9 m beneath the landed drone's belly radar (45 s scan).
- Victim audio is synthesised from recorded and text-to-speech clips and passed through a rubble model.
  The recognisers (Silero VAD, YAMNet, Whisper, SRP-PHAT direction finding) are real models.
- Breathing "condition" labels are an aid for rescuers, not a diagnosis. A ping is a candidate search
  region, not a confirmed person. Silent piles are never "cleared".
- In 20 simulated seeds, acoustic-led dispatch put a radar ping on 17 of 60 non-start victims, against 6 and 5
  for radar-only baselines. The acoustic clips in that evaluation were generated from ground truth, so this
  is an upper bound. 0 false pings.
- The video is time-lapsed (×5–×30, shown on screen) in one continuous take. The single unscripted moment:
  a teammate switches on a pre-placed hidden dummy at a time of their choosing, and the system re-plans
  from what it hears.
- Hardware and prices are a design study (estimates, no supplier quotes). The belly radar module is designed, not built.
- Decisions are made by [Gemini (gemini-3.5-flash) | a rule-based fallback]; each one is labelled on screen.

**Data and audio credits.**
- ESC-50 (K. J. Piczak, ACM MM 2015), CC BY-NC 3.0. Clips from freesound.org: [author of 101336],
  [author of 108017], [author of 151080]; modified (trimmed, resampled, rubble filtering). Non-commercial use.
- Freesound CC0 recordings (collected by F. Valisalo): Gemesil, Legnalegna55, Bee_Abney, DRFX, darranambler,
  FunWithSound, brunoboselli.
- Piper TTS (rhasspy), voices en_US-lessac, zh_CN-huayan, id_ID-news_tts, vi_VN-vivos. Voice licences follow
  their datasets: [check each MODEL_CARD; vivos: CC BY-NC-SA 4.0].
- Schellenberger et al. 2020, Sci Data 7:291, doi:10.1038/s41597-020-00629-5 (CC BY 4.0).
- Google FLEURS (CC BY 4.0) and DroneAudioSet (ahlab-drone-project, MIT) for evaluation.
- DREGON (Strauss et al., IROS 2018) for direction-finding validation, academic use only; not redistributed.
- Map data © OpenStreetMap contributors; imagery © Esri, Maxar, Earthstar Geographics.
- Weiguan facts: Tainan City Fire Bureau; Pan et al. 2019 (JFMA); Chiu & Tseng 2018; Taipei Times.
- Software: NVIDIA Isaac Sim / Isaac Lab, KISS-ICP, Open3D, pyroomacoustics, Silero VAD, YAMNet,
  faster-whisper (OpenAI Whisper), noisereduce, three.js.
```

## 6. Top 10 fixes (in order)

1. **Rewrite or banner the judge-facing docs.** TECHNICAL_REPORT (+ abstract), PITCH_LARPVERSE, TECH_SUMMARY, SLIDES_OUTLINE, DEMO and README still describe landed radar, legacy propagation, BL-3, a child class and "Gemini never called" (§1 rows 1, 4, 6, 11, 14, 16, 17, 22).
2. **Pin the seed-7 story to one committed, reproducible run.** The brief's "A2 + B1 ping, B2 open" is not what the committed demo-story config gives: A2 (t 364), B1 (t 1712) and B2 (t 2312) ping, and A3 stays open. RADAR_PENETRATION.md:81-86 still has the kinematic run and "depths stay nominal", but A2-R1 is `radar_estimate` (§0; rows 8-10).
3. **Fix the trigger window in ISAAC_BRIEF** (lines 23, 111). "During the listen, about 33–40 s, ~7 s reaction" was a stub-acoustics measurement. With the real pipeline the press must come at video ≈17–29 s (VIDEO_PLAN.md:71-75), or the take stalls for a full pipeline run (row 27).
4. **Quote the eval as an upper bound, with CIs and the flight model.** Use 17/60 vs 6/60 vs 5/60 (stub acoustics, kinematic flight, 20 seeds). Never "faster" (independent start: 1413 s vs 1042 s). Never the legacy 42/60. Add the flight model to the EVAL_GRID header (rows 11-13, O7).
5. **IMPACT_EVIDENCE.md:32 says the system "clears empty sectors" and "finds the silent victims".** That contradicts the never-cleared rule and the A3 miss. Reword per O1. Also flag the vendor radar ranges (4.5–7 m) as claims, and prepare the "2.9 m vs LifeLocator" answer (row 33, O14).
6. **Licences before the video goes public:**
   - Add ESC-50 + Freesound author credits and a credits file next to the committed ESC-50 WAVs (non-commercial).
   - Verify the Piper voice MODEL_CARDs (vivos BY-NC-SA; lessac research-only?).
   - Never show `--site collapse` without NCKU permission: its geometry, not just its textures, is derived.
   - Keep DREGON out of the repo and the video (§3).
7. **Gemini consistency.**
   - PLAN.md:192 "unlimited" vs free tier 20/day.
   - demo.sh's `gemini-3.7-flash` vs `gemini-3.5-flash` everywhere else (verify the ID).
   - "Lite has more" vs "20/day per model".
   - The take caption must show the real `by` badge (rows 18-20, O13).
8. **Platform numbers.** PLATFORM.md and HARDWARE §14 still carry the 0.6 × 0.5 m 4 kg drone radar panel, 12 kg payload and 24.6 / 18.5 min. Current values: radar module 4.9 kg, 0.48 m face, payload 14 kg, 23.9 min (about 17.9 min usable). The van mast is 6 m in Isaac but 10.5 m in the link budget (rows 5, 25).
9. **Condition wording.** Make the HUD (`isaac/hud.py:48`) and ISAAC_BRIEF/VIDEO_PLAN say "deep rest or reduced consciousness (radar can't tell)", like the dashboard. Real healthy volunteers land in this class 38/57 times. Don't promise the Indonesian transcript on screen: base.en returns none (rows 30, 39, O4).
10. **Fill or close the «PENDING» markers** per §4. B5-RERUN, ISAAC-CLIP and the live Gemini dispatch remain genuinely open. Cite the actual FHWA document and the RADIOTECT source before calling 10–20 dB/m "sourced" (row 43).

## Requests (orchestrator)

- PLAN.md is orchestrator-owned. Add decision rows that supersede :192 (Gemini unlimited → free tier) and :200 (collapse site as the primary demo → grid site for the public video, pending NCKU).
- README.md is orchestrator-owned. Paste §5 and fix rows 4, 16, 22, 35 and 36.
- Commit a `docs/SEED7.md`, or a section in RADAR_PENETRATION, holding the canonical seed-7 run (§0) and its command.
