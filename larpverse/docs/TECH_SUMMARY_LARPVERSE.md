# Larpverse: technical summary (one-page PDF draft)

Updated 8 Nov 2026. Sources: `docs/review/PASS3_CLAIMS.md` and the docs named. Open slots: «SEED7-CANON», «ISAAC-CLIP», «GEMINI-DISPATCH».

**User and problem.** The user is the incident commander at a building collapse. People are trapped in voids that cameras can't see, listening devices only hear people who can call or tap, and radar life detectors must be carried onto unstable piles one spot at a time. The commander has to choose which pile to work, and where to dig, before time runs out. At Weiguan Jinlong (Tainan, 2016), 309 people were in the building: 175 were rescued alive and 115 died, and nobody came out alive after 62.3 h.

**What Larpverse does.** Three heavy-lift drones search a field of rubble piles as one system. They hand the commander **candidate search regions**, a breathing **condition** for each one (an aid, not a diagnosis), and a ranked pile list.
1. **Map.** A LiDAR orbit and KISS-ICP SLAM build the pile map. Flat landing spots are scored on 0.5 m cells with a 1.5 m plane fit.
2. **Land and scan.** Each drone lands on a flat spot on the rubble, and its downward-facing 4.9 kg radar and microphone module, mounted on the drone's underside, scans through the rubble: about 8 s to set down, a 45 s scan, 8 s to lift off. A Bayes grid fuses the ranges, and the next landing spot is chosen by expected information gain.
3. **Ping and condition.** Three agreeing strong ranges give a ping with a belief share and a depth source; otherwise the result is `insufficient_evidence`. A 60 s vitals look labels the breathing calm, rapid-shallow, slow-regular or irregular with pauses. Non-human returns are rejected.
4. **Listen, call out, connect.** Landed off the pile with rotors off, the drones listen (Silero VAD, YAMNet, Whisper, SRP-PHAT with a rotor veto; Gemini audio when the quota allows). They call out in zh-TW, Taiwanese Hokkien and English, and serve a consent-based rescue Wi-Fi portal.
5. **Dispatch.** A gate weighs priority, travel and the 25% battery reserve. Gemini or a fallback picks from listed options, and code validates the pick. Every pile without a ping ends "searched, **not cleared**".

**Architecture and where it runs.** Today the mission runs on a laptop CPU and Isaac Sim 6.0.1 on a cloud L40S.

| Component | Runs on | Latency today |
| --- | --- | --- |
| LiDAR SLAM, landing-spot scoring, stage-1 audio | drone (Jetson AGX Orin target) | 0.38 s per 23k-point sweep, laptop, offline |
| Radar DSP, vitals, rejector | radar module / drone | not timed |
| Fusion, EIG planner, acoustic association and agent, gate, dashboard, speaker, portal | command van | about 50–62 s per real acoustic listen (laptop CPU) |
| Gemini: audio analysis, dispatch pick, brief | cloud, optional (OneWeb or LTE in Tainan) | audio 1.6–1.8 s median per clip; free tier, about 20/day per model |

Radios: a 5.8 GHz mesh and a 2.4 GHz hotspot, all clear of the radar's 0.4–2 GHz sweep. Raw data stays on board.

**Evaluation.** Simulation unless it says real data.

| What | Result | Caveat |
| --- | --- | --- |
| Multi-pile dispatch, 20 seeds (EVAL_GRID.md) | non-start victims pinged: acoustic **17/60** vs nearest 6/60 vs raster 5/60 (28% [15–42] vs 10% vs 8%); 0 false pings | acoustic clips generated from truth, so an **upper bound**; kinematic flight; not faster (independent start: first ping 1413 s vs 1042 s) |
| Radar reach (RADAR_PENETRATION.md) | about **2.9 m** of rubble beneath the landed drone's belly radar (45 s); 0.7 m from 11.5 m | synthetic propagation model |
| Breathing DSP, **real data** | 0.39 breaths/min MAE (176 windows) | 24 GHz CW, 0.4 m, line of sight, no rubble |
| Breathing condition | synthetic recall 95/100/84/65%; **real** rate MAE 0.51/min, 6/6 breath holds | relaxed volunteers read "slow, regular" 38/57 |
| Acoustic recognisers, **real data** | human vs non-human F1 0.70–0.82 rotors off; 0.58 with rotor noise at 0 dB | single mic, no rubble; rotor veto not re-validated |
| Gemini audio, live | language ID 89/90 clean, 78/90 noisy; cue recall 0.76, precision 1.00 | free tier |
| SLAM | ATE 3.6 cm; landing IoU 0.98 | one pile, own ray-caster |
| Seed-7 demo | A2 pings (radar-only, silent); further pings vary by how the radar seats on the rubble «SEED7-CANON» | one review run also pinged B1 and B2; A3 stayed open |

**Limits.**
- Through-rubble radar returns and victim audio are synthetic.
- The belly radar module is designed, not built; hardware costs are estimates.
- Smooth flight trajectories, but no flight controller or aerodynamics.
- The take has not been rendered on a GPU yet.
- The Gemini dispatch decider has not run live.
- No drone loss, aftershock or dust handling.

**Critical failure mode and safety.**
- **Missed person:** no pile is ever "cleared"; open leads go in the brief. Seed 7's A3 is a real example.
- **Over-trust:** pings are candidate regions; conditions are aids that describe chest motion only; no age inference.
- **Speaker:** vetted templates only; translations need operator approval.
- **Phones:** consent only; no sniffing or interception.
- **LLM:** listed options, a validator, banned claims ("confirmed", "survivor found"), a fallback, and a `by` badge.
- **Ground truth:** only generators and evaluators read it; tests enforce this.

**Impact (model, not a record; v1.0 run, to be refreshed per docs/IMPACT_MODEL.md).** In a counterfactual model (docs/IMPACT_MODEL.md; stub acoustics), the system adds +0.2 (−0.1 to +0.7) survivors per 100 trapped as simulated, and +0.75 (−1.8 to +2.7) at the radar upper bound. On the Weiguan-scale site the radar resting on the rubble surface made 0 firm pings in 170 people, so **we do not claim lives saved at Weiguan scale**. A city kit costs about $2.8M over 10 years ($1.1–6.3M) and breaks even at 0.56 lives per decade at Taiwan's value of life (docs/ECONOMICS.md). A truck-mounted kit could give a first radar candidate at about T+50 min (23–94) after a quake (docs/DEPLOYMENT.md).
