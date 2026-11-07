# RESUME: Larpverse build loop (read first after compaction)

Updated 2026-11-07 ~22:15. Repo: /home/peroxide/Downloads/UCL/agentverse (local git, ~79 commits). The orchestrator mirrors committed HEAD into the team repo with `scripts/push.sh`:
- remote github.com/rohansharmacode/skysearch, branch `larpverse-sensing`, folder `larpverse/`;
- local clone of the team repo at ~/Downloads/UCL/skysearch.

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
- Model `gemini-3.5-flash` (GEMINI_MODEL).

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
| a055b721c3e78b0e1 | **Mission** (sim/grid_mission, grid_world, acoustic_grid, coordinator, radar_model, radar_dsp, landing, localise, planner, evaluate(_grid); tests grid/radar/localise; docs/EVAL_GRID.md, RADAR_PENETRATION.md) | P0s: R2-003 acoustic cache version, R2-002 A2 outlier veto, realistic penetration as default (WIP broke pings: no commit until test_grid_mission passes), depth estimate, vitals hooks, autonomy/eval. Then `--site` scenario-driven, altitude clearance, sector names, dashboard fields (vad/association/geo/radar_coverage) |
| a1dbd08c7726451da | **Acoustic fix** (acoustic/dsp, run, scene, models, eval_assign; tests/test_acoustic) | Top blocker: B1 missed, A3 cross-talk, knock-vs-engine, wire the novelty→Gemini stage into run.py, final eval table |
| a162587dbfaffdc2e | **Gemini audio** (acoustic/novelty, gemini_audio, eval_gemini; agents/llm.py audio, backoff and throttle; FLEURS data; docs/ACOUSTIC_GEMINI.md) | Live multilingual evaluation, replay cache |
| a2b76615bba793384 | **Real-audio validation + pipeline verdict** (acoustic/validate_real.py, docs/ACOUSTIC_VALIDATION.md, data/audio_real) | VERDICT: ours canonical (b2937fe..1d626f2). Fixes 1-4 → acoustic fix agent. RESUMED: DREGON real-DoA test (≤500 MB, licence check) |
| a1a856ada8463193a | **Vitals** (sim/physio.py, sim/vitals.py, scripts/validate_vitals.py, tests/test_vitals, docs/VITALS.md) | RESUMED 22:50: remove child/age class, condition-only, fix slow-breather over-triage. Hooks 1-2 ok for mission, 3-5 held. Known: over-triages calm slow breathers (5-8 bpm) |
| add66c0cd95d1fed8 | **Site** (sim/site_weiguan, site_mesh, `site="collapse"` from the teammate's USD, isaac/scene.py, photoreal.py, run_mission.py `--site`, isaac/site.py, mapping/slam per-scenario, usd-core in .venv) | **Top: collapse site as primary** |
| a5c508c060aa20b94 | **Dashboard** (dashboard/*) — DONE (fbe7e43, 9eeb709, 64ed989); questions answered in INTERFACES v1.2.1 | Batch 2 R2 items, `?data` alias, Dig-order panel and vitals, clip language/translation, coverage hatch, tasking Plan-search mode, recon display and proposals, scaling to the collapse site |
| a9d09919957c913a4 | **Visual consistency** (animation/grid-search.html, index.html fixes, scenario_seed7.js via exporter, scripts/export_scenario_js.py, isaac/overlay.py) | Overlay merged with the teammate's ribbons (6a0e0ed); animation recon/tasking beats; MP4 re-record |
| a793a6498850fe65e | **Tasking + recon** (sim/tasking.py, sim/recon.py, tests, docs/TASKING_INTEGRATION.md integration diff) | Scale to the collapse site |
| ac3d7a7f1dc704099 | **Platform** (sim/platform.py, mapping/raycast_lidar.py, isaac/lidar.py, isaac/drone_viz.py, docs/PLATFORM.md) | DONE (84b9dd9, 337db98). R-PLAT-1/2/5 → mission, R-PLAT-3 → site, R-PLAT-4 → dashboard (producer of 16 m tiles still unassigned) |
| a5e7bdb232aa15f7b | **Speaker** (sim/callout.py, sim/guidance.py, acoustic/speaker_assets.py, docs/SPEAKER.md, tests/test_callout) | serve.py patch and mission diff via Requests |

Paused: a946c57cc57fd5b2b (video cut, `scripts/cut_video.py`). Resume it when the Isaac re-record exists.

Done: tech report a56b6a6dcfe7aacab (`docs/TECHNICAL_REPORT.md`, about 11k words, «PENDING» placeholders to fill), pitch, demo runner, reviewers pass 1 and 2, Isaac merge, SLAM, radar DSP and validation, Gemini decider.

## Integration queue (orchestrator does or forwards these)

1. When **tasking, recon and speaker** diffs land (`docs/TASKING_INTEGRATION.md`, `docs/SPEAKER.md`), forward them to the mission agent to apply. Apply the `serve.py` broadcast patch myself (scripts/ are orchestrator-owned).
2. When the **vitals** hooks land, forward them to the mission agent; the dashboard Dig-order panel consumes the vitals fields.
3. When the **collapse site** lands:
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
