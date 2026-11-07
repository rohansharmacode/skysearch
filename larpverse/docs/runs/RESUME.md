# RESUME — Larpverse build loop

Read this first after a context compaction or restart.

## Standing instructions (user, 2026-11-07)
- Speed over everything; usage/tokens not a concern. Keep 5 agents busy (4 builders + 1 reviewer), never more.
- Isaac Sim is run by a teammate on a GPU box; we write code + docs for it (docs/ISAAC_HANDOFF.md).
- Push to github.com/rohansharmacode/skysearch branch `larpverse-sensing` (folder larpverse/) via scripts/push.sh only. Never touch the team's other paths/branches. No CI.
- Gemini is the LLM for agent decisions (GEMINI_API_KEY in local .env, model gemini-3.5-flash). User will provide the key.
- Keep animation/index.html (v1) working as-is (bug fixes OK). v2 = animation/grid-search.html.

## Infrastructure
- Local git in repo root (init 2026-11-07). Team clone at ~/Downloads/UCL/skysearch (push.sh mirrors `git archive HEAD`).
- Dev server: preview "static" (python http.server :8765, repo root).
- Watchdog: scripts/watchdog.sh under Monitor (WARN|HARD), re-arm every 30 min. Heartbeat cron every ~30 min.
- Keep-awake was refused (user setting) — machine may sleep while user is away.

## Running agents
(see table below, kept current)

| Slot | Area / owned files | Started |
|---|---|---|

## Done (high level)
- Concept animation v1 + grid-search v2 (published artifacts), MP4s in media/ (not pushed: >100 MB).
- sim/ numpy core, radar DSP (+ real-data validation 0.39 bpm MAE), acoustic pipeline (.venv-audio), grid mission + 3x3 dashboard, Isaac adapter (dry-run tested, never run on GPU), handoff doc.

## Next
See docs/PLAN.md backlog.

## Pending user decisions
- GEMINI_API_KEY in larpverse/.env (or repo-root .env locally).
- PR / merge of larpverse-sensing into the team's branch?
- Licences: ESC-50 dog clip (CC BY-NC), Piper lessac voice terms.
