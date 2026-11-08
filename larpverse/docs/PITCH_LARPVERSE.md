# Larpverse: pitch pack (AgentVerse 2026, robotics track)

Updated 8 Nov 2026 to the current system. Every number comes from review pass 3 (`docs/review/PASS3_CLAIMS.md`) or the doc named next to it.

**One line:** three heavy drones search a field of collapsed-building rubble without anyone standing on it. They lower a breathing radar onto the pile on a winch, listen and call out, and hand the incident commander **candidate search regions**, a breathing **condition** for each one (an aid, not a diagnosis), and a ranked list of which pile to search next.

## Open placeholders

| Placeholder | Fills from | Status |
|---|---|---|
| «SEED7-CANON» | the mission agent's canonical seed-7 doc | not landed. Say only: "A2 pings (radar-only, silent); further pings vary by pod seating" |
| «ISAAC-CLIP» | the single-take video (docs/VIDEO_PLAN.md) | not rendered on a GPU yet |
| «GEMINI-DISPATCH» | a live Gemini dispatch decision | audio ran live; the dispatch decider has not |
| ~~«PENDING-IMPACT»~~ | docs/CASE_STUDIES.md, ECONOMICS.md, DEPLOYMENT.md, IMPACT_MODEL.md | filled 8 Nov (§6 Impact); ranges only |

## Words we never use

"Survivor found", "confirmed", "cleared", "diagnosis", any age or "child", and "faster" for the evaluation. Instead we say:
- "candidate search region" or "ping";
- "searched, not cleared";
- "breathing condition, an aid for rescuers".

## 1. Problem

- After a collapse, people are trapped in voids **inside** the rubble. Cameras and thermal imagers see the surface, and listening devices only hear people who can call or tap. In Bam (2003), 28% of injured survivors were unconscious when rescued (IMPACT_EVIDENCE.md).
- **Time kills fast.** At Weiguan Jinlong (Tainan, 6 Feb 2016), 309 people were in the building. 175 were rescued alive and 115 died. Live rescues ran 164 / 7 / 4 on days 1–3, and nobody came out alive after 62.3 h (Chiu & Tseng 2018; Pan et al. 2019; Tainan Fire Bureau).
- A collapse site is **many piles**. Crews place listening devices and radar life detectors by hand, standing on unstable rubble, one spot at a time.

## 2. User

The **USAR team leader / incident commander**. They decide which pile a crew works next and where to dig. They want to know where to look, how sure we are, what hasn't been checked yet, and whether someone is still breathing.

## 3. What's new (and what isn't)

We did not invent through-rubble radar, acoustic search or LiDAR SLAM. We built the **autonomy and delivery around them**:

1. **A winch-lowered sensor pod.** A heavy drone hovers and lowers a 4.9 kg radar and microphone pod onto the rubble. That puts the antenna on the surface above the void, which is the only geometry where realistic penetration works. From the old 11 m landing ring, the radar got 0–1 of 60 victims in our evaluation.
2. **Measurement planning.** The pod is re-placed where expected information gain is highest. Every result carries its uncertainty, and "insufficient evidence" is an explicit outcome.
3. **Breathing condition, with a non-human rejector.** Each ping gets a condition: calm, rapid-shallow, slow-regular or irregular with pauses. The wording is an aid ("slow, regular breathing: deep rest or reduced consciousness, check responsiveness"). Returns that look like debris, an animal, machinery or a rotor never become pings.
4. **Listen, call out, connect.** Landed off-pile with rotors off, the drones listen with real recognisers. They call out in zh-TW, Taiwanese Hokkien and English using vetted templates, and serve a rescue Wi-Fi portal that phones can choose to join.
5. **A bounded agent.** The LLM (Gemini, free tier, with fallback) picks only from listed options. Code validates every pick, and every decision shows who made it.

## 4. How it works

**survey → pod scans → ping + condition → listen and call out → rank → dispatch → repeat**

1. **Survey.** A LiDAR orbit, KISS-ICP SLAM, and a pile map.
2. **Pods.** Three drones each lower a pod near the lead. A placement is about 8 s down, a 45 s scan and 8 s up. The pod is re-placed by expected information gain.
3. **Ping.** Three agreeing strong ranges give a candidate search region, with a belief share and a depth labelled by its source. A 60 s vitals look adds the breathing condition.
4. **Listen and call out.** The drones land off the pile with rotors off. They listen to the other piles and play a call-out. Replies and phones joining the portal raise a pile's priority.
5. **Rank and dispatch.** The acoustic agent ranks the piles. The gate weighs priority, travel and the battery reserve, then sends the fleet on. Gemini or the fallback picks, and code checks the pick.
6. **Handover.** The commander gets a dig order, open leads, and piles that were searched but are **not cleared**.

## 5. What's real and what's simulated

| Part | Real | Simulated |
|---|---|---|
| Site | Weiguan case-study facts | the public demo is a synthetic 3×3 grid; the layout is labelled synthetic |
| 3D world, LiDAR | Isaac Sim 6.0.1 + Isaac Lab 3.0 on a cloud L40S; PhysX raycasts; KISS-ICP | the scene itself |
| Flight | – | smooth multirotor trajectories (jerk-limited, tilt from acceleration); no flight controller, no aerodynamics |
| Radar | the breathing DSP and vitals features, checked on real line-of-sight 24 GHz recordings | every through-rubble return (labelled synthetic generator) |
| Pod and hardware | parts and prices from datasheets (estimates, no supplier quotes) | the pod is designed, not built |
| Acoustics | real recognisers (VAD, YAMNet, Whisper, direction finding), checked on real recordings with rotors off | the victim audio: recorded and TTS clips passed through a rubble model |
| Gemini | audio analysis ran live (251 clips) | the dispatch decider has no live decision yet «GEMINI-DISPATCH» |
| Phones | link budget from published sources | every phone and message |

## 6. Evidence (quote each number with its caveat)

**Grid evaluation** (docs/EVAL_GRID.md; 20 seeds; pod, `literature` penetration, reported start; generated with kinematic flight):
- Acoustic-led dispatch put a radar ping on **17 of 60** non-start victims, against **6/60** (nearest-first) and **5/60** (raster).
- Per seed: 28% [15–42] vs 10% [3–17] vs 8% [3–15].
- 0 false pings. Visits to empty or non-human piles: 0.7 vs 2.8 and 2.5 per mission.
- **Caveat:** the acoustic clips were generated from ground truth, so this is an **upper bound**.
- 0/20 seeds had every victim pinged.
- With an independent start, it is 19/74 vs 10/74 vs 9/74, and the first non-start ping comes **later** (1413 s vs 1042 s). Never say "faster".

**Radar reach** (RADAR_PENETRATION.md, model): about **2.9 m of rubble beneath the pod** at a 45 s scan; 1.5 m at 6 m slant; 0.7 m from 11.5 m away. 10 m of rubble is hopeless.
- **Caveat:** synthetic propagation model. The concrete-loss source is still to be cited.
- If asked about a LifeLocator: vendors claim 4.5–7 m, not independently verified. Our model is conservative. Our gain is placements without people on the pile, not range.

**Breathing DSP on real recordings:** 0.39 breaths/min MAE over 176 windows (RADAR_VALIDATION.md).
- **Caveat:** Schellenberger et al. 2020, 24 GHz CW at 0.4 m, line of sight, healthy subjects, no rubble.

**Breathing condition:**
- Synthetic pod scans: recall 95 / 100 / 84 / 65% for calm / rapid-shallow / slow-regular / irregular (VITALS.md).
- Real recordings: rate MAE 0.51/min; 6 of 6 breath holds detected.
- **Caveat:** healthy relaxed volunteers read "slow, regular" in 38/57 windows. That is why it says "deep rest **or** reduced consciousness".

**Acoustic recognisers on real audio, rotors off** (ACOUSTIC_VALIDATION.md):
- Human vs non-human F1 0.70 (ESC-50), 0.81 (CC0 clips), 0.82 (DroneAudioSet); 0.58 with real rotor noise at 0 dB.
- Direction finding on real DREGON recordings: 100% within 10° with rotors off, 17–24% with rotors on in flight, which is why we land to listen.
- **Caveat:** single mic, no rubble; the rotor veto is not yet re-validated.

**Gemini audio, live** (ACOUSTIC_GEMINI.md):
- Language ID 89/90 clean and 78/90 noisy (9 languages).
- Distress-cue recall 0.76, precision 1.00.
- Median 1.6–1.8 s per clip; 0 fallbacks over 251 clips.
- **Caveat:** free tier, about 20 requests/day per model.

**Seed-7 demo:** A2 pings (radar-only, silent); further pings vary by pod seating «SEED7-CANON».
- In one review run, B1 and B2 also pinged, and A3 (a rapid, shallow breather) stayed an open lead.
- A missed person is the critical failure, and our demo shows the system saying so.

**Impact (model, not a record; v1.0 run, to be refreshed per docs/IMPACT_MODEL.md).** In a counterfactual model (docs/IMPACT_MODEL.md; stub acoustics), the system adds +0.2 (−0.1 to +0.7) survivors per 100 trapped as simulated, and +0.75 (−1.8 to +2.7) at the radar upper bound. On the Weiguan-scale site the pod made 0 firm pings in 170 people, so **we do not claim lives saved at Weiguan scale**. A city kit costs about $2.8M over 10 years ($1.1–6.3M) and breaks even at 0.56 lives per decade at Taiwan's value of life (docs/ECONOMICS.md). A truck-mounted kit could give a first radar candidate at about T+50 min (23–94) after a quake (docs/DEPLOYMENT.md).

**Rescue Wi-Fi** (HOTSPOT.md, synthetic phones): median 0.6 m error with the fleet landed around the pile, 7.1 m from the survey only. The first fit on seed 7 put one phone in the wrong sector.

## 7. Safety and failure modes

- **Missed person (critical).** No pile is ever "cleared". Piles read "searched, not cleared", and the brief lists open leads. Seed 7's A3 is an example.
- **Over-trust.** A ping is a candidate region with a belief share, 95% region and depth source. A condition is an aid: radar sees chest motion only, so it says nothing about perfusion or responsiveness.
- **False lead.** A ping needs 3 strong agreeing ranges. Non-human radar returns are rejected. Ambiguous audio can't trigger a dispatch on its own.
- **Speaker.** Only vetted templates play autonomously. Operator text and every translation wait for approval. The Hokkien TTS line has not been checked by a native speaker.
- **Phones.** Located only by consent (they join the portal). No sniffing, no cellular interception. Unconscious people are invisible to it.
- **LLM.** Listed options only, a validator, banned claims, a fallback, and a `by` badge on every decision.
- **Pod on rubble.** 4.9 kg; re-seated once if it rocks. Flat geometry is not structural safety.
- **Not handled:** drone or pod loss, aftershocks, dust, GNSS loss.

## 8. Where each component runs

| Component | Runs on |
|---|---|
| LiDAR SLAM, pod-site scoring, stage-1 audio | drone (Jetson AGX Orin; not timed) |
| Radar DSP, vitals, rejector | pod / drone |
| Fusion, planner, acoustic association, gate, dashboard, speaker queue, portal | command van (laptop today: about 50–62 s per real acoustic listen) |
| Gemini (audio, dispatch, brief) | cloud, optional, over OneWeb (via Chunghwa) or LTE in Tainan; fallback offline |

Radios: a 5.8 GHz drone mesh, a 5.2 GHz pod link and a 2.4 GHz rescue hotspot, all clear of the radar's 0.4–2 GHz sweep. Raw data stays on the drone.

## 9. The video

One continuous 60 s take, captured from the Isaac stream next to the live dashboard (docs/VIDEO_PLAN.md):
- per-phase time-lapse ×5–×30, shown on screen;
- **one unscripted moment:** a teammate switches on a pre-placed hidden dummy at a time they choose, and the system hears it through the real recognisers (on synthesised audio) and re-plans.

The old multi-cut storyboard is retired. «ISAAC-CLIP»

## 10. Judge Q&A

1. **Does it find survivors?** It flags candidate search regions in simulation, with a breathing condition as an aid. A rescuer confirms.
2. **Can radar see through rubble?** Our model says about 2.9 m beneath the pod and under 1 m from 11 m away. That is why we lower the radar onto the pile. The radar returns are a labelled synthetic generator; the DSP was checked on real line-of-sight recordings.
3. **Why a winch pod?** A 3 m heavy drone can't perch on a pile, and from the landing ring realistic penetration gave 0–1 of 60 victims. A 4.9 kg pod on the surface gets 1–2.5 m slant range, and nobody climbs the pile.
4. **What's novel?** Delivery plus autonomy: winch-pod placement chosen by expected information gain, explicit uncertainty, a non-human rejector, and an acoustic and phone triage that drives dispatch.
5. **How much better than a simple search?** In 20 simulated seeds, 17 of 60 non-start victims pinged vs 6 and 5. The acoustic clips came from ground truth, so it's an upper bound, and it isn't faster to the first ping.
6. **What if it misses someone?** It does: in seed 7, A3's rapid, shallow breather can stay an open lead. No pile is ever marked cleared.
7. **Is the condition a diagnosis?** No. It describes chest motion only. Healthy relaxed people read "slow, regular", so we say "deep rest or reduced consciousness, check responsiveness". There is no age inference.
8. **What does the LLM decide?** One option from the planner's list, checked by a validator; otherwise the fallback decides. Gemini audio analysis ran live; the dispatch decider has not yet run live (free-tier quota).
9. **Does it need the internet?** No. Fallbacks and replay caches run offline; the cloud is optional.
10. **What about phones?** Consent-based: an open rescue Wi-Fi with a portal. We never sniff or intercept.
11. **What would a field test need?** Build the pod, fly a heavy-lift airframe with a winch, and record on a USAR training rubble site.
12. **Impact numbers?** Ranges from a counterfactual model, never a claim about Weiguan: +0.2 (−0.1 to +0.7) extra survivors per 100 trapped as simulated, +0.75 (−1.8 to +2.7) if the pod works as designed. On the Weiguan-scale site the pod has not yet made a firm ping, so we don't claim lives saved at that scale.
13. **What does it cost?** About $0.67M up front for a 3-drone city kit, and $2.8M over 10 years ($1.1–6.3M). It breaks even at 0.56 lives per decade at Taiwan's value of a statistical life (docs/ECONOMICS.md).
14. **Why haven't international teams solved this?** They arrive after 12–56 h, and their live rescues were under 0.85% of the death toll in 14 earthquakes (Rom & Kelman 2020). Neighbours did 77.5% of Kobe's rescues (docs/CASE_STUDIES.md). A kit on a local truck could give a first radar candidate at about T+50 min (DEPLOYMENT.md).

## 11. Rubric map

| Rubric (weight) | What we show | Honest gap |
|---|---|---|
| Perception-driven action (25%) | Pod scans → EIG re-placement; voice, knock or phone → priority → dispatch; non-human returns rejected | radar returns and audio are synthetic |
| Adaptation to change (25%) | The live take's unscripted hidden dummy changes the plan; zones re-tasked mid-mission | the take has not been rendered on a GPU yet |
| Operational autonomy (15%) | Whole mission without a human after the start; every decision logged with who made it | edge box, not on-drone, for the acoustic stack |
| Relevance and impact (20%) | Commander's triage: which pile, where, breathing condition, what isn't cleared | impact is a model with wide ranges; evaluation is an upper bound; no Weiguan-scale pings yet |
| Ambition and technical quality (15%) | Isaac 6.0.1, real SLAM and recognisers, real-data DSP checks, bounded LLM, comms design | pod not built; no flight controller |

## Credits & honesty

See the README section "Credits & honesty". It covers ESC-50 (CC BY-NC, non-commercial), Freesound CC0 authors, Piper voices (licences to verify), Schellenberger 2020, FLEURS, DroneAudioSet, DREGON (academic use, not redistributed), OSM/Esri and NVIDIA Isaac Sim.
