# RESUME — Larpverse build loop

Read this first after a context compaction or restart.

## Standing instructions (user, 2026-11-07)
- Speed over everything; usage/tokens not a concern. Keep 5 agents busy (4 builders + 1 reviewer), never more.
- Isaac Sim is run by a teammate on a GPU box; we write code + docs for it (docs/ISAAC_HANDOFF.md).
- Push to github.com/rohansharmacode/skysearch branch `larpverse-sensing` (folder larpverse/) via scripts/push.sh only. Never touch the team's other paths/branches. No CI.
- Gemini is the LLM for agent decisions (GEMINI_API_KEY in local .env, model gemini-3.5-flash). User will provide the key.
- Keep animation/index.html (v1) working as-is (bug fixes OK). v2 = animation/grid-search.html.

## Coordination with the Isaac teammate's Claude
- We post instructions in docs/FOR_ISAAC_CLAUDE.md (newest first) with every push.
- They push to branch `larpverse-isaac` and report in larpverse/docs/FROM_ISAAC.md. Each heartbeat: `git -C ~/Downloads/UCL/skysearch fetch -q origin && git -C ~/Downloads/UCL/skysearch log --oneline -3 origin/larpverse-isaac` and read FROM_ISAAC.md; merge their isaac/ fixes into our tree (they come back on the next push).
- User (2026-11-07 18:08): keep-awake ON; keep building; make models work with data and run autonomously; dashboard; working demo ASAP; don't worry about the computer.

## Infrastructure
- Local git in repo root (init 2026-11-07). Team clone at ~/Downloads/UCL/skysearch (push.sh mirrors `git archive HEAD`).
- Dev server: preview "static" (python http.server :8765, repo root).
- Watchdog: scripts/watchdog.sh under Monitor (WARN|HARD), re-arm every 30 min. Heartbeat cron every ~30 min.
- Keep-awake: user turned it on (18:08).

## Running agents
(see table below, kept current)

| Slot | Area / owned files | Started |
|---|---|---|
| 1 | Acoustic real-data validation: acoustic/validate_real.py, docs/ACOUSTIC_VALIDATION.md, data/audio_real/ | 18:25 |
| 2 | Mission autonomy + eval (BL-7/8): sim/* except scenario.py, tests/test_grid_mission.py, tests/test_radar_dsp.py, tests/test_localise.py, docs/EVAL_GRID.md | 18:20 |
| 3 | Dashboard fix batch R1-016…029 (BL-10): dashboard/*, docs/review/img/dash-b2-* | 18:55 |
| 4 | Animation on seed 7 (BL-6): animation/grid-search.html, animation/scenario_seed7.js, scripts/export_scenario_js.py, media/larpverse-grid-seed7.mp4 → orchestrator republishes artifact U5RX772n8VKTTGPA8yEWxZ | 18:42 |
| 5 | Acoustic pile assignment: acoustic/{dsp,run,scene,models,eval_assign}.py, tests/test_acoustic.py, acoustic/README.md | 18:08 |

## Review loop
- Pass 1 done (575253f): 46 findings. Batches routed: brain → acoustic + mission builders; dashboard → builder; Isaac handoff → orchestrator (done except R1-033/037/038); animation → animation builder.
- Next: reviewer pass 2 once brain + dashboard batches land (verify fixes; new angles: skeptical judge 5-min demo, Isaac teammate cold start).

## Done (high level)
- Concept animation v1 + grid-search v2 (published artifacts), MP4s in media/ (not pushed: >100 MB).
- sim/ numpy core, radar DSP (+ real-data validation 0.39 bpm MAE), acoustic pipeline (.venv-audio), grid mission + 3x3 dashboard, Isaac adapter (dry-run tested, never run on GPU), handoff doc.

## Next
See docs/PLAN.md backlog.

## Pending user decisions
- GEMINI_API_KEY in larpverse/.env (or repo-root .env locally).
- PR / merge of larpverse-sensing into the team's branch?
- Licences: ESC-50 dog clip (CC BY-NC), Piper lessac voice terms.
