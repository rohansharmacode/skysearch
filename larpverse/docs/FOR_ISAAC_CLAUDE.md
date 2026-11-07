# Notes for the Isaac Sim Claude session (read on every pull)

You're the Claude Code session running Isaac Sim for the Larpverse demo. This file is the coordination channel from the build loop on the other laptop (the "orchestrator"). It's updated with every push to branch `larpverse-sensing`. **Newest entry first.** Read it after each `git pull`.

## How we coordinate

- **Our code:** `larpverse/` on branch `larpverse-sensing`. Treat it as read-only: our pushes replace that folder wholesale, so any edits you make there get overwritten.
- **Your work:** commit it on your own branch, **`larpverse-isaac`**, created from `larpverse-sensing`, and push it there. Put fixes to `larpverse/isaac/*` on that branch too. We fetch it, merge your changes into our tree, and they come back in the next `larpverse-sensing` push.
- **Reports:** write them in **`larpverse/docs/FROM_ISAAC.md`** on your branch, newest first. Include:
  - Isaac version and GPU;
  - each `--selftest` step with PASS/FAIL and the error text;
  - screenshots or footage paths (put large files in a GitHub Release, not git);
  - what you changed and why;
  - what you need from us.
- **What not to touch:** the team's `main`, `v2-earthquake` and `isaac6-port` branches. Don't push to `larpverse-sensing`.
- **Secrets:** put the Gemini key only in `larpverse/.env` (`GEMINI_API_KEY=…`). It's gitignored; never commit it.

## Start here (always)

1. `git fetch && git checkout larpverse-sensing && git pull`, then `cd larpverse`.
2. Read `docs/ISAAC_HANDOFF.md` (the full brief), `isaac/README.md` and this file.
3. Install scipy into Isaac's Python: `~/isaacsim/python.sh -m pip install scipy` (or the `isaaclab.sh -p -m pip` equivalent).
4. First command: run the dry run, then the self-test (exact form in `isaac/README.md`). Report every PASS/FAIL line in `FROM_ISAAC.md`.

---

## Updates

### 2026-11-07 18:10: build loop running

**Ready on the branch:**
- `sim/scenario.py`: the seeded 3×3 rubble grid. Seed 7 is the demo.
- `sim/grid_mission.py`: the multi-pile mission. It pings the start pile, listens, ranks the other piles, dispatches, and repeats.
- `isaac/`: the adapter (dry-run tested). Options include `--replay` and `--overlay usd`.
- `dashboard/`: the live 3×3 operator view. It polls `dashboard/data/state.json`, which your Isaac run writes.

**Being worked on now** (expect these in the next pushes):
1. **Isaac adapter checked against your team's GPU-verified Isaac 6.0.1 / Isaac Lab 3.0 notes** from the `isaac6-port` branch. This adds:
   - an **AppLauncher** launch path (`./isaaclab.sh -p larpverse/isaac/run_mission.py --livestream 2 …`), like the team's Brev flow;
   - a **`--selftest`** that prints PASS/FAIL for app boot, stage build, physics parse, raycast hit, camera switch, debug draw and frame capture;
   - a `--look pretty` presentation mode.

   Wait for this push before your first GPU run if you can. If you're already up, run what's there and report.
2. **Gemini decision agent:** it chooses dispatch, scores the audio clips, reads operator notes and writes the rescuer handover. It needs `GEMINI_API_KEY` in `larpverse/.env`, and everything works without it (fallbacks).
3. **Acoustic fix:** voices were being assigned to piles that have no voice. Being fixed with triangulation across the three drones.
4. **KISS-ICP SLAM** for the landing maps.

**What we need from you first:**
- Your Isaac version and GPU.
- The output of the dry run, and of the self-test once it lands.
- One screenshot of the stage after `--max-steps 1`.
