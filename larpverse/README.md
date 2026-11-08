# Larpverse: autonomous heavy-drone search of collapsed-building rubble

Built for the AgentVerse 2026 robotics track. Case study: the Weiguan Jinlong collapse in Tainan, 6 Feb 2016: 309 people in the building, 175 rescued alive, 115 dead. The public demo runs on a synthetic 3×3 grid of rubble piles.

Three heavy-lift drones (FlyCart 30 / Agras T50 class) search the piles as one system, and nobody has to stand on the rubble:

1. **Map.** Each pile is mapped with a spinning LiDAR, registered by KISS-ICP SLAM.
2. **Pod scans.** A drone hovers and **lowers a 4.9 kg radar + microphone pod on a 20 m winch** onto the rubble: about 8 s down, a 45 s scan, 8 s up. A Bayes grid fuses the ranges, and the pod is re-placed where expected information gain is highest.
3. **Ping and condition.** Three agreeing ranges give a **candidate search region** ("ping"). A 60 s vitals look reads the breathing **condition**: calm, rapid-shallow, slow-regular or irregular with pauses. This is an aid for ordering the dig, not a diagnosis, and there is no age inference. Radar returns that look like debris, animals, machinery or rotors are rejected.
4. **Listen, call out, connect.** The drones land off the pile with rotors off. They listen to the other piles (Silero VAD, YAMNet, Whisper, SRP-PHAT with a rotor veto, and Gemini audio when the quota allows). They call out in zh-TW, Taiwanese Hokkien and English using vetted templates, and serve a consent-based rescue Wi-Fi portal.
5. **Dispatch.** An agent (Gemini, or a rule-based fallback) ranks the piles and picks the next one from listed options. Code validates every pick. Piles without a ping end "searched, **not cleared**".

The operator dashboard shows zones, pings with conditions, the dig order, transcripts, the speaker queue and every decision with who made it. Isaac Sim 6.0.1 + Isaac Lab 3.0 (Brev L40S) provides the 3D world, drone bodies, LiDAR geometry, the command van and the training-manikin dummies for the single-take video ([docs/VIDEO_PLAN.md](docs/VIDEO_PLAN.md)).

Full write-up: [docs/TECHNICAL_REPORT.md](docs/TECHNICAL_REPORT.md) (abstract: [docs/TECHNICAL_REPORT_ABSTRACT.md](docs/TECHNICAL_REPORT_ABSTRACT.md)).

## The problem, the models, and how their output is checked

**Problem.** After a building collapse, rescuers must decide which rubble pile to dig first, while people trapped inside are silent, weak or out of sight. Larpverse gives an incident commander a live operator console: click a pile, and three drones map it with LiDAR, land on spots flat enough to land on, feel for breathing with radar, listen for voices, and then pick the next pile themselves.

**Models (exact IDs).**
| Where | Model | Job |
|---|---|---|
| Each drone (audio) | `gemini-3.5-flash` (`GEMINI_AUDIO_MODEL`) | Per-clip audio analysis: human or not, transcript, distress cues, and whether the sound is muffled under rubble or out in the open air |
| Police van (command) | `gemini-4-argon` (`GEMINI_MODEL`) | Picks the next pile from the listed options, with a one-line reason |
| On-drone, offline option | Gemma 4 E2B (`google/gemma-4-E2B-it`, `LLM_PROVIDER=gemma`, `scripts/gemma_server.py`) | The same audio step with no network link (open weights, Apache 2.0) |
| On-box, always | Silero VAD, YAMNet, faster-whisper, SRP-PHAT | Detection, transcription and direction finding that run without any LLM |

**Verification loop.** No model output acts directly:
1. The decider may only choose from options the mission lists (`agents/gemini_decider.py`). Code validates the choice, its wording, the pile budget and the 25 % battery-reserve rule. An invalid or missing answer falls back to a deterministic rule (highest acoustic priority), labelled `fallback`.
2. Audio answers are schema-validated. Distress words must be reproduced by a second, independent transcription, or they are demoted. The sound priority list (`acoustic/priority.py`) ranks every sound as trapped > check > safe person on the surface > not a person. Only trapped/check sounds can pull the drones.
3. Every decision and every heard sound is labelled on screen with who made it: you, `Gemini 4 Argon · police van`, `Gemini 3.5 Flash · on each drone`, or the on-box models.
4. A radar ping is a candidate search region, never a confirmed person, and no pile is ever "cleared".

**Live console.** `isaac/hud/console.html` with `isaac/hud/picker.js`: choose one of three sites, pick a pile on a 3D LiDAR dot map, and watch Isaac Sim run the search. Keys live only in a gitignored `.env`.

## Quickstart (CPU, no GPU needed)

```bash
uv venv .venv && uv pip install --python .venv numpy scipy matplotlib pytest kiss-icp open3d
.venv/bin/python -m sim.grid_mission --seed 7 --out dashboard/data      # writes dashboard/data/state.json
.venv/bin/python scripts/serve.py --port 8765                           # http://localhost:8765/dashboard/
```

Mission defaults: `--radar dsp --delivery pod --penetration literature --flight smooth --tasking auto`, with comms on. With no `tasking.json`, the mission starts in recon mode. Add `--tasking default` to search the scenario's rubble outlines, as the demo-story test does. `serve.py` serves the dashboard plus the tasking, portal and broadcast APIs; `python -m http.server` can't save zones. One-command demo: `scripts/demo.sh --direct` (runbook: [docs/DEMO.md](docs/DEMO.md)).

Optional:
- **Real acoustic models:** `bash acoustic/setup.sh` builds `.venv-audio` with about 210 MB of models (`acoustic/models/` is 314 MB on disk). Then add `--acoustic real`.
- **Gemini:** put `GEMINI_API_KEY=…` in `.env` (gitignored) and add `--decider gemini`. The key is free tier, about 20 requests/day per model, so set `LLM_REPLAY=1` to replay cached answers. Everything falls back to rule-based choices without it.
- **Isaac Sim:** [docs/ISAAC_BRIEF.md](docs/ISAAC_BRIEF.md) and [isaac/README.md](isaac/README.md).

## Module map

| Path | What |
|---|---|
| `sim/scenario.py`, `sim/site_*.py` | Seeded sites: the 3×3 grid (public demo), a procedural Weiguan stand-in, and the collapse site (private: NCKU-derived geometry) |
| `sim/grid_mission.py` | The autonomous mission: tasking/recon → survey → pod scans → ping → listen + call-out → rank → dispatch |
| `sim/localise.py`, `sim/planner.py`, `sim/landing.py` | Grid Bayes localisation, EIG pod placement, pod-site and landing scoring |
| `sim/radar_dsp.py`, `sim/physio.py`, `sim/vitals.py`, `sim/radar_signatures.py` | Labelled-synthetic radar generator, real breathing DSP, physio chest waveform, condition + non-human rejector |
| `sim/platform.py`, `sim/flight_motion.py` | Heavy-lift platform and pod spec; smooth multirotor trajectories |
| `sim/callout.py`, `sim/guidance.py`, `sim/comms.py`, `sim/tasking.py`, `sim/recon.py` | Speaker call-out, rescue hotspot and mesh, operator zones, recon |
| `mapping/` | Ray-cast LiDAR, KISS-ICP SLAM, landing agreement |
| `acoustic/` | Scene synthesis, SRP-PHAT/LCMV association, rotor veto, VAD/YAMNet/Whisper, novelty filter, Gemini audio, agent |
| `agents/` | Gemini or fallback dispatch decider, intake, handover brief, record/replay |
| `dashboard/`, `scripts/serve.py` | Operator console (`?layout=take` for the video), portal, APIs |
| `isaac/` | Isaac Sim adapter: scene, motion, PhysX LiDAR, HUD, van, dummies, `--live` single take |
| `contracts/INTERFACES.md` | Frames, units, schemas (v1.4) |
| `docs/` | Report, pitch, evaluations, hardware, comms, reviews |

## Honest limits

- **Radar returns are a labelled synthetic generator.** The breathing DSP was checked on real line-of-sight 24 GHz recordings with no rubble (Schellenberger et al. 2020: 0.39 breaths/min MAE). The modelled reach is about 2.9 m of rubble beneath the pod ([docs/RADAR_PENETRATION.md](docs/RADAR_PENETRATION.md)).
- **Victim audio is synthesised** from recorded and TTS clips through a rubble model. The recognisers were checked on real recordings with rotors off: human vs non-human F1 0.70–0.82 ([docs/ACOUSTIC_VALIDATION.md](docs/ACOUSTIC_VALIDATION.md)). The rotor veto is not yet re-validated on real audio.
- **The evaluation is an upper bound.** In 20 simulated seeds, acoustic-led dispatch pinged 17 of 60 non-start victims, against 6 and 5 for radar-only baselines. The acoustic clips were generated from ground truth, flight was kinematic, and it is not faster ([docs/EVAL_GRID.md](docs/EVAL_GRID.md)).
- **A ping is a candidate search region, not a confirmed person.** No pile is ever "cleared", and the system misses some victims.
- **Conditions are an aid, not a diagnosis.** Radar sees chest motion only.
- **The pod is designed, not built.** Hardware prices are estimates.
- **Flight follows smooth trajectories** with no flight controller, aerodynamics or wind.
- **Planning code never reads ground truth.** Only the generators and evaluators do, and tests enforce it.

## Credits & honesty

**What is real and what is simulated.** Everything in the video is a simulation (NVIDIA Isaac Sim 6.0.1).
- Rubble, victims (training-manikin dummies) and their positions are synthetic. The case-study framing is the Weiguan Jinlong collapse, Tainan, 6 Feb 2016 (115 dead); the site layout shown is synthetic.
- Radar returns are a labelled synthetic generator. The breathing-detection DSP that processes them was checked on real line-of-sight radar recordings (Schellenberger et al. 2020: 0.39 breaths/min MAE; no rubble). Modelled reach through rubble: about 2.9 m beneath the winch-lowered pod (45 s scan).
- Victim audio is synthesised from recorded and text-to-speech clips and passed through a rubble model. The recognisers (Silero VAD, YAMNet, Whisper, SRP-PHAT direction finding) are real models.
- Breathing "condition" labels are an aid for rescuers, not a diagnosis. A ping is a candidate search region, not a confirmed person. Silent piles are never "cleared".
- In 20 simulated seeds, acoustic-led dispatch put a radar ping on 17 of 60 non-start victims, against 6 and 5 for radar-only baselines. The acoustic clips in that evaluation were generated from ground truth, so this is an upper bound. 0 false pings.
- The video is time-lapsed (×5–×30, shown on screen) in one continuous take. The single unscripted moment: a teammate switches on a pre-placed hidden dummy at a time of their choosing, and the system re-plans from what it hears.
- Hardware and prices are a design study (estimates, no supplier quotes). The radar pod is designed, not built.
- Next-pile decisions are made by `gemini-4-argon` (police van), or a rule-based fallback if it is unavailable or invalid. Audio is analysed by `gemini-3.5-flash` (per drone) or the on-box models. Each one is labelled on screen.

**Data and audio credits.**
- ESC-50 (K. J. Piczak, ACM MM 2015), CC BY-NC 3.0. Clips from freesound.org: [author of 101336], [author of 108017], [author of 151080]; modified (trimmed, resampled, rubble filtering). Non-commercial use.
- Freesound CC0 recordings (collected by F. Valisalo): Gemesil, Legnalegna55, Bee_Abney, DRFX, darranambler, FunWithSound, brunoboselli.
- Piper TTS (rhasspy), voices en_US-lessac, zh_CN-huayan, id_ID-news_tts, vi_VN-vivos. Voice licences follow their datasets: [check each MODEL_CARD; vivos: CC BY-NC-SA 4.0].
- Schellenberger et al. 2020, Sci Data 7:291, doi:10.1038/s41597-020-00629-5 (CC BY 4.0).
- Google FLEURS (CC BY 4.0) and DroneAudioSet (ahlab-drone-project, MIT) for evaluation.
- DREGON (Strauss et al., IROS 2018) for direction-finding validation, academic use only; not redistributed.
- Map data © OpenStreetMap contributors; imagery © Esri, Maxar, Earthstar Geographics.
- Weiguan facts: Tainan City Fire Bureau; Pan et al. 2019 (JFMA); Chiu & Tseng 2018; Taipei Times.
- Software: NVIDIA Isaac Sim / Isaac Lab, KISS-ICP, Open3D, pyroomacoustics, Silero VAD, YAMNet, faster-whisper (OpenAI Whisper), noisereduce, three.js.

The `[…]` fields need a human check before anything is posted publicly.
