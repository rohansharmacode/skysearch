# Larpverse: slide outline (10 slides)

Updated 8 Nov 2026 to the current system: drones land on the rubble with a belly-mounted radar, `literature` radar, condition-only vitals, rotor veto, speaker, hotspot, single-take video. Every number on a slide carries its source in small type and its caveat in one line. Open slots: «SEED7-CANON», «ISAAC-CLIP», «GEMINI-DISPATCH».

## Visuals

| File | What it shows | Use on |
|---|---|---|
| «ISAAC-CLIP» frames from the single take (docs/VIDEO_PLAN.md) | van, lift-off, drones landed on the rubble, ping, re-plan | 1, 4, 7 (not rendered yet) |
| `docs/img/isaac_assets_drone.png`, `isaac_assets_van.png`, `isaac_assets_dummies.png` | heavy drone with belly-mounted radar, command van, training-manikin dummies (Isaac asset checks) | 4, 9 |
| `docs/img/isaac_grid_map.png` | seed-7 grid site, 9 piles (check that no ground-truth markers show) | 3 |
| `docs/img/report_radar_reach.png` | modelled reach through rubble vs range for the radar resting on the rubble surface (synthetic model) | 5 |
| `docs/img/radar_signatures.png` | synthetic breathing and confounder signatures through the real DSP | 6 |
| `docs/radar_real_validation.png`, `docs/vitals_validation.png` | real line-of-sight recordings: waveform, rate, breath holds | 8 |
| `docs/acoustic_real_validation.png` | recognisers on real audio | 8 |
| `docs/img/motion_profiles.png` | smooth flight profiles | backup |
| `docs/slam.png` | KISS-ICP pile map and landing cells | backup |
| `docs/screenshots/portal_390_*.png` | rescue Wi-Fi portal (zh, en, id) | 7 |
| Dashboard in `?layout=take` (fresh capture) | condition chip, dig order, waveform, decision badge | 6, 7 |

**Do not use:**
- `docs/screenshots/grid-*.png` and `ping-detail-A2-R1.png` (7 Nov: old depth wording, landed radar);
- `docs/img/impact_*.png` only with their caption from docs/IMPACT_MODEL.md ("counterfactual model; stub acoustics; upper bound needs the belly radar working"). Never pull out a single lives-saved number;
- the collapse-site renders (NCKU-derived geometry; private only).

## Slides

| # | Title | One line of copy | Visual |
|---|---|---|---|
| 1 | **Larpverse** | "Heavy drones that put a breathing radar on the rubble, so nobody has to stand on it, and tell rescuers where to dig first." | take frame: drones landed on the pile. Tag: "Simulated, NVIDIA Isaac Sim 6.0.1" |
| 2 | **Time kills; cameras can't see in** | "Weiguan Jinlong, 2016: 309 in the building, 175 rescued alive, 115 dead; nobody came out alive after 62.3 h. Listening devices only hear people who can call or tap." (IMPACT_EVIDENCE.md) | plain type on dark |
| 3 | **The user: the incident commander** | "Which pile, where on it, is someone breathing, and what hasn't been checked." | `isaac_grid_map.png` |
| 4 | **The loop** | "Survey → land on a flat spot → 45 s breathing scan with the belly radar → next landing spot by information gain → ping + condition → listen and call out → rank → dispatch." | drone asset image (belly radar); loop diagram |
| 5 | **Why land on the rubble** | "Modelled reach: about 2.9 m of rubble beneath the landed drone, 0.7 m from 11 m away. From a landing ring, realistic radar pinged 0–1 of 60 victims." (RADAR_PENETRATION.md, EVAL_GRID.md; synthetic model) | `report_radar_reach.png` |
| 6 | **Breathing condition, not diagnosis** | "Calm / rapid-shallow / slow-regular / irregular with pauses, as an aid to dig order. Debris, animals, machinery and rotors are rejected, never pinged." | `radar_signatures.png` + condition chip |
| 7 | **Listen, call out, connect** | "Rotors off: real recognisers, a rotor veto, call-outs in zh-TW, Hokkien and English, and a rescue Wi-Fi portal that phones choose to join." | portal screenshots + clip card |
| 8 | **Evidence** | "In 20 simulated seeds, acoustic-led dispatch pinged 17 of 60 non-start victims vs 6 and 5 for radar-only baselines (upper bound: acoustic clips generated from truth; 0 false pings; not faster). Breathing DSP 0.39 bpm MAE on real line-of-sight recordings (no rubble)." | small table + `radar_real_validation.png` |
| 9 | **What it can't do yet** | "Radar returns and victim audio are synthetic; the belly radar module is designed, not built; no flight controller. Seed 7: A2 pings; A3's rapid, shallow breather can stay an open lead. Nothing is ever 'cleared'." «SEED7-CANON» | dashboard capture with A3 "searched, not cleared" |
| 10 | **Where it runs, and next** | "Drone: SLAM, stage-1 audio. Belly radar module: radar DSP. Van: fusion, planner, agent, dashboard. Cloud optional (OneWeb or LTE). Next: build the belly radar, fly a drone that lands on rubble, record on a USAR training site. A city kit: about $2.8M over 10 years; first radar candidate about T+50 min after a quake (model)." (ECONOMICS.md, DEPLOYMENT.md) | van asset image + three-box diagram |

Q&A backup: the rubric map and judge Q&A in `docs/PITCH_LARPVERSE.md` §10–11.

## Copy rules for every slide

- Say "candidate search region" or "ping". Never "survivor found", "confirmed" or "cleared"; say "searched, not cleared".
- Say "breathing condition, an aid for rescuers". Never "diagnosis", and never an age or "child".
- Put "simulated" on every Isaac frame, and "synthetic radar returns" wherever radar appears.
- Quote the evaluation only with its row and caveat (upper bound, kinematic flight). Never "faster", never "3×".
- The decision badge must show who decided (Gemini / fallback / argmax).
- Credits on the last slide or in the description: README "Credits & honesty".
