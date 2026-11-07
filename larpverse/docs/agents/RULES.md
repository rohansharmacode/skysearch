# Rules for every agent (Larpverse build loop)

Project: Larpverse — autonomous multi-drone rubble search-and-rescue demo (AgentVerse 2026). Repo root: /home/peroxide/Downloads/UCL/agentverse (local git). The orchestrator mirrors committed HEAD to github.com/rohansharmacode/skysearch, branch `larpverse-sensing`, folder `larpverse/`.

1. Read first: README.md, docs/PLAN.md (backlog + decisions), contracts/INTERFACES.md, and the files your prompt names. Product decisions in PLAN.md aren't yours to change; list disagreements under Requests.
2. Edit only the files your prompt gives you. Shared files (docs/PLAN.md, docs/runs/RESUME.md, contracts/INTERFACES.md, .gitignore, scripts/*.sh, README.md) belong to the orchestrator: ask via Requests. Exception: you may append a section to contracts/INTERFACES.md if your prompt says so.
3. Before editing a file, run `git status -- <file>`. If it has another agent's uncommitted changes, don't edit it; ask via Requests.
4. Commit early and often, locally, with `scripts/commit.sh "<message>" <path> [<path>…]` (commits only those paths). Never `git add -A`, `git commit -a`, `git stash`, `git reset`, `git checkout -- .`. Never push. Never trigger CI.
5. (Loosened 19:30 by the user: speed first. heavy.sh now allows 3 concurrent heavy jobs and only waits below 1.2 GB free.) Every heavy command goes through `scripts/heavy.sh` (test suites, mission runs, acoustic pipeline, SLAM, plotting large data, anything > ~30 s or > 500 MB). Browser jobs (headless Chrome, recordings, screenshots): `scripts/heavy.sh --browser …`. heavy.sh may queue for a while: never wrap it in a short `timeout`. Split long jobs so no single call holds the slot > ~5 min. Run only the tests for what you touched.
6. The machine has 14 GB RAM and the desktop app crashes when memory runs out. No parallel heavy processes, no background processes left running, few screenshots, close browsers after each step.
7. The dev server is the orchestrator's: http://localhost:8765 — scripts/serve.py (static repo root + GET/POST /api/tasking). Use it via curl / headless browser; never start another server.
8. Don't install packages or download large data without asking (Requests). Existing envs: `.venv` (numpy, scipy, matplotlib, pytest, usd-core?), `.venv-audio` (acoustic models), `.venv-map` (SLAM, if present).
9. Ground-truth rule: only observation generators (sim/radar_model.py, acoustic scene synthesis) and evaluators may read `scenario.truth` / `world.targets`. Planner, localiser, agents and dashboard never do. Never claim a person is "confirmed"; pings are candidate search regions.
10. Verify UI changes at ~400 px and desktop width. Python: run the relevant tests.
11. Final report, plain and short: what you changed (commit hashes), how you verified it (commands + results), what's left and why, Requests for the orchestrator.
