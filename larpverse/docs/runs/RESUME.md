# RESUME: Larpverse build loop (read first after compaction)

Updated 2026-11-07 22:53 (real clock; earlier 23:xx/00:xx stamps in this file were estimates and run ~1 h fast).
- remote github.com/rohansharmacode/skysearch, branch `larpverse-sensing`, folder `larpverse/`;
- local clone of the team repo at ~/Downloads/UCL/skysearch.

## Pause history

Paused by the user at about 23:15 and resumed at about 23:20. All 11 agents were resumed via SendMessage; watchdog and heartbeat cron (17a9724f) re-armed.

## Standing instructions (user)

**Pace and budget**
- Build as fast and as well as possible. Usage and tokens are not a concern ("I WANT it used").
- Up to about 10–11 concurrent agents. Heavy jobs still go through `scripts/heavy.sh`: 3 slots, a 1.2 GB free-memory floor, and browser jobs wait outside the FIFO.
- Keep-awake is ON.

**Isaac and coordination**
- The Isaac Sim teammate is Rohan, plus his Claude. Coordinate by pushing instructions:
  - `docs/ISAAC_BRIEF.md` is the single current brief; update it when Isaac features land.
  - `docs/FOR_ISAAC_CLAUDE.md` is the running log.
  - They report in `docs/FROM_ISAAC.md` on their branch `larpverse-isaac`.
  - Their map is on branch `3d_model`, folder `collapse_site/`.
- Their implementation is canonical where GPU-verified: photoreal, recorder, ribbon overlay.

**Gemini**
- The key is saved in `.env` (gitignored). It's a temporary hackathon key, so use it freely.
- It is rate-limited (HTTP 429), so backoff and throttling live in `agents/llm.py`.
- Model `gemini-3.5-flash` (GEMINI_MODEL). **23:55: the key hit its DAILY quota (429). It is FREE TIER: ~20 req/day per model.** demo.sh now splits models: audio gemini-3.5-flash-lite (replay cache), decider gemini-3.7-flash; all agents fall back cleanly. Re-run the live decider pass and the Gemini audio eval when it resets.

**Product decisions by the user**
- **Audio:** an open-model novelty filter runs first; only outliers go to Gemini (multilingual), which feeds prioritisation.
- **Site:** case study = Weiguan Jinlong collapse, Tainan 2016 (23.0051432 N, 120.2608904 E). **The primary demo site is the teammate's 320 m `collapse_site` USD** (`site="collapse"`, being built).
- **Search zones:** operator-tasked zones are drawn on the dashboard before launch. With no zones, the drones run **recon**: mapping flight, perch-and-listen, then zone proposals. Zones aren't re-searched without new evidence.
- **Platform:** heavy-lift (agricultural or industrial class) drones with dense LiDAR, a bigger radar antenna, a mic array and a **loudspeaker**: multilingual call-out-and-listen, reassurance, and guidance for walking survivors. Speaker text uses vetted templates only; LLM-translated text plays only after the operator confirms.
- **Vital signs:** radar breathing pattern → triage hint → dig order. It's an aid for rescuers, not a diagnosis; age-from-amplitude is low confidence.
- **Audio pipeline choice:** decided by the measured comparison with Frances's pipeline (branch `acoustic-voice-detection`) in `docs/ACOUSTIC_VALIDATION.md`.
- Keep `animation/index.html` (v1) as is; bug fixes are fine.

## Infrastructure

- **Dev server:** preview "static" = `python3 scripts/serve.py --port 8765`. It serves the static repo root plus GET/POST `/api/tasking`, writing `dashboard/data/tasking.json`.
- **Watchdog:** `scripts/watchdog.sh` under Monitor (WARN|HARD, 30 min), re-armed on each expiry.
- **Heartbeat:** cron at :17 and :47 every hour.
- **Helper scripts:** `scripts/commit.sh` (named paths only), `scripts/push.sh` (git archive HEAD → team branch), `scripts/heavy.sh`, `scripts/set_gemini_key.sh`, `scripts/demo.sh` (one-command demo, `--piles 3` cap until the acoustic fix), `scripts/record_animation.py`, `scripts/record_dashboard.py`.
- **Artifacts (private):**
  - concept animation https://claude.ai/artifact/6bb51FL1ULwMKTcGqkVHz4
  - grid animation (seed 7) https://claude.ai/artifact/U5RX772n8VKTTGPA8yEWxZ (publish with `files={"scenario_seed7.js": …}`)
  - agent decision map https://claude.ai/artifact/2DJSXqUQfkZGjKfFifxjay
- **Videos:** in `media/`, gitignored. Uploaded to team release `larpverse-storyboard`: grid-search, concept and grid-seed7.
- **Network outage recovery:** wait for `getent hosts api.anthropic.com`, then resume each agent with SendMessage by ID ("check git status, rerun what failed").

## Running agents (IDs, for SendMessage)

| Agent ID | Area / owned files | State |
|---|---|---|
| a055b721c3e78b0e1 | **Mission** (sim/grid_mission, grid_world, acoustic_grid, coordinator, radar_model, radar_dsp, landing, localise, planner, evaluate(_grid); tests grid/radar/localise; docs/EVAL_GRID.md, RADAR_PENETRATION.md) | RESUMED 23:05: build WINCH POD delivery (decision in PLAN log), realistic penetration default, then physio on + vitals hooks 3-5, then LOD/tiles. Last round: legacy default kept (realistic = 0 pings from 11 m stand-off); seed 7 pings A2,A3,B2; eval 42/60 vs 17/20 |
| a1dbd08c7726451da | **Acoustic fix** (acoustic/dsp, run, scene, models, eval_assign; tests/test_acoustic) | DONE (eb8cdba, 26de7e9, assoc-v11): 0 false confident leads, voice first 5/5, B1+B2 found, knock recall 0.17→0.75. Gemini wired in run.analyse. Not done: elevation SRP (needs 3D scene), Frances noise-floor port, multilingual Whisper (awaiting USER approval for the 145 MB download) |
| a162587dbfaffdc2e | **Gemini audio** (acoustic/novelty, gemini_audio, eval_gemini; agents/llm.py audio, backoff and throttle; FLEURS data; docs/ACOUSTIC_GEMINI.md) | DONE (06013f5..1333702): prompt v5 kills hallucinated distress; 434 replay answers on flash-lite. Integration → acoustic fix agent |
| a2b76615bba793384 | **Real-audio validation + pipeline verdict** (acoustic/validate_real.py, docs/ACOUSTIC_VALIDATION.md, data/audio_real) | DONE: verdict ours; DREGON real DoA (dd8ab8a, data is academic-use only, kept local) |
| a1a856ada8463193a | **Vitals** (sim/physio.py, sim/vitals.py, scripts/validate_vitals.py, tests/test_vitals, docs/VITALS.md) | DONE 4da97d0: condition-only, 86% synthetic, distressed 100%, over-triage fixed (49→5 of 57). Hooks 3-5 unblocked. Callout fix → speaker. Known: over-triages calm slow breathers (5-8 bpm) |
| add66c0cd95d1fed8 | **Site** (sim/site_weiguan, site_mesh, `site="collapse"` from the teammate's USD, isaac/scene.py, photoreal.py, run_mission.py `--site`, isaac/site.py, mapping/slam per-scenario, usd-core in .venv) | DONE: collapse site (07d9ec7), pod visual + passthrough (75ea9b2). Brief 🟡→✅ pushed. Open: isaac/selftest.py still grid-only; licence: core geometry is also scan-derived |
| a5c508c060aa20b94 | **Dashboard** (dashboard/*) | RESUMED: comms/hotspot panel (blue), live /api/portal feed | Batch 2 R2 items, `?data` alias, Dig-order panel and vitals, clip language/translation, coverage hatch, tasking Plan-search mode, recon display and proposals, scaling to the collapse site |
| a9d09919957c913a4 | **Visual consistency** (animation/grid-search.html, index.html fixes, scenario_seed7.js via exporter, scripts/export_scenario_js.py, isaac/overlay.py) | DONE batch 4 + seed-7 physio story (3ae56b8). Grid MP4 re-recorded and uploaded to release 22:52; grid artifact republished (v5). Idle |
| a793a6498850fe65e | **Tasking + recon** (sim/tasking.py, sim/recon.py, tests, docs/TASKING_INTEGRATION.md integration diff) | DONE (7f7d904, 646a8ff, 2c1dced, 41231c4): diff in TASKING_INTEGRATION.md → mission. Idle. If the mission agent is the bottleneck, it can take grid_mission integrations once the pod work is committed |
| ac3d7a7f1dc704099 | **Platform** (sim/platform.py, mapping/raycast_lidar.py, isaac/lidar.py, isaac/drone_viz.py, docs/PLATFORM.md) | DONE (84b9dd9, 337db98). R-PLAT-1/2/5 → mission, R-PLAT-3 → site, R-PLAT-4 → dashboard (producer of 16 m tiles still unassigned) |
| ad524ba6da89b83ed | **Decider** — DONE (9730a11, 0958562); requests 1-3 → mission. Idle |
| a02bd98e325a265bc | **Comms / rescue hotspot** (sim/comms.py, tests/test_comms.py, docs/HOTSPOT.md, dashboard/portal/) | NEW 00:05: Wi-Fi AP + captive portal + mesh backhaul + RSSI phone localisation; serve.py /api/portal patch → orchestrator applies |
| a46e0ecedd0867d45 | **Dummy audio** (assets/audio/dummies/, scripts/build_dummy_audio.py, docs/DUMMY_AUDIO.md) | DONE 2b95209: 126 clips (57 MB), profiles + hidden C3 (vi) / A1 (id). Leakage + sources= hook → acoustic. USER: Hokkien native recordings, CC0 groan |
| ae7b1717c7d0c4b66 | **Hardware + comms architecture** (docs/HARDWARE.md, docs/COMMS_ARCHITECTURE.md, data/hardware_bom.json) | DONE (41a42bf, 3627d19): ~$61k/drone, 14 kg payload, raw data stays on drone, ArduPilot build airframe (Harris H6HL), custom USRP pod radar. Requests routed (assets, comms, mission) |
| a74b2c46c151e5b6f | **Flight motion** (sim/flight_motion.py, isaac/motion.py, scripts/motion_preview.py, animation/motion-preview.html, docs/MOTION.md) | NEW 23:15: user says recording is janky. Jerk-limited multirotor trajectories, tilt/flare, perch, rotor RPM, render-time interpolation, offline preview MP4 |
| aaa8c6e412c69ccaf | **Isaac scene assets** (isaac/van.py, dummies.py, drone_viz.py, pods.py; docs/ISAAC_ASSETS.md) | DONE 51634ef: van 12k tris, drone 10k with BOM parts, pod, dummies + hidden; hooks 1-5 → live agent |
| a5aae449f969b5952 | **Isaac live single take** (isaac/run_mission.py, live.py, hud.py; docs/VIDEO_PLAN.md) | NEW: stage = source of truth, live unscripted triggers, perception HUD, 60 s one-take plan |
| a5e7bdb232aa15f7b | **Speaker** (sim/callout.py, sim/guidance.py, acoustic/speaker_assets.py, docs/SPEAKER.md, tests/test_callout) | DONE (4d46fb4, a8eff22, c8cf446). serve.py /api/broadcast applied b90217d; mission patch §6 → mission (3 langs default); panel → dashboard. Hokkien voice needs native recording |

Paused: a946c57cc57fd5b2b (video cut, `scripts/cut_video.py`). Resume it when the Isaac re-record exists.

Done: tech report a56b6a6dcfe7aacab (`docs/TECHNICAL_REPORT.md`, about 11k words, «PENDING» placeholders to fill), pitch, demo runner, reviewers pass 1 and 2, Isaac merge, SLAM, radar DSP and validation, Gemini decider.

## Integration queue (orchestrator does or forwards these)

0. Quick Isaac brief sent: docs/ISAAC_QUICK.md (23:20). FULL brief due once van/dummies/live/motion land, about 00:30. Update ISAAC_BRIEF.md and send it to the user.
0. **DELIVERABLE (user, 22:55): ≤60 s single-take Isaac stream capture with perception on screen and ONE unscripted live change.** Everything serves this. The user said: no Whisper download, no new downloads for the audio work.


1. When **tasking, recon and speaker** diffs land (`docs/TASKING_INTEGRATION.md`, `docs/SPEAKER.md`), forward them to the mission agent to apply. Apply the `serve.py` broadcast patch myself (scripts/ are orchestrator-owned).
2. When the **vitals** hooks land, forward them to the mission agent; the dashboard Dig-order panel consumes the vitals fields.
3. ~~collapse site~~ DONE 23:40 (brief flipped). Still: run the mission on site="collapse" once the pod lands.
   - flip the 🟡 rows in `docs/ISAAC_BRIEF.md` to ✅ with verified commands, then push;
   - run the mission on `site="collapse"`.
4. After the acoustic fix, the A2 fix and vitals land, run the **rehearsal**: `scripts/demo.sh` with live Gemini.
5. Then **reviewer pass 3**, focused on the new site, zones/recon, vitals and speaker.
6. Then post **"READY TO RE-RECORD"** in `FOR_ISAAC_CLAUDE.md`, and resume the video-cut agent.
7. **Technical report:** (also strip child/age vitals wording; condition-only per user 22:50) re-run the report agent (a56b6a6dcfe7aacab via SendMessage) to fill the placeholders once the evaluation docs exist. Publish the final report (artifact or doc) for the user.
8. **Slides deck** at the end (Artifact quickstart, slides).

## Pending user decisions / asks

- Hackathon requirements: deadline, video length, repo public?, live-demo format. Not answered yet.
- Which team branch is the submission, and should larpverse-sensing / 3d_model be merged (PR)?
- NCKU Museum permission for the Weiguan scan textures. Public showing needs permission or a procedural-texture variant.
- Native-speaker check of the Taiwanese Hokkien call-out and voice lines.
- Optional: teammates record real distress-call audio (with consent) into `data/audio_team/`.

## Review loop

- Pass 1: 46 findings. Pass 2: 29/46 verified, 23 new (5 P0), routed.
- Pass 3 is due after the rehearsal. New angles: the collapse site at scale, operator tasking/recon UX, vitals honesty, speaker safety.
