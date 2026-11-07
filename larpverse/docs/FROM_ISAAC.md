# From the Isaac Sim session (newest first)

## 2026-11-07 18:20 (branch `larpverse-isaac`)

**Setup:** Isaac Sim 6.0.1 on a Brev L40S (headless), driven from Rohan's desktop.

**Dry run:** PASS. `python isaac/run_mission.py --dry-run --max-steps 60`: three drones move, the LiDAR gets 42,160 points on A2, and `state.json` is written. `pytest tests`: 22 passed, 3 skipped.

**What we added.** Everything is opt-in, and runs without the new flags are unchanged.
- `isaac/photoreal.py` (new), `--photoreal 1`:
  - Each scenario box under `/World/Rubble/<pile>/<kind>_NNN` is swapped for an irregular concrete piece of the same size and pose. Pieces are slabs with bent rebar, blocks or rocks, built by `isaac/rubble.py` with a triangle collider. The box itself is set inactive, so the LiDAR and landing cells see the same geometry.
  - Visual-only debris per pile and a gravel patch under each pile.
  - NVIDIA MDL materials: concrete, brick, gravel, dirt, asphalt and rusted steel.
  - A dusk HDRI sky, a low sun, haze, dry shrubs and trees, and an operator figure at staging.
- Recording with `--record DIR` (PNG frames per camera, virtual clock), plus:
  - `--rec-cams director,chase:d1,fpv:d2,top`
  - `--rec-res 1920x1080`, `--rec-fps 30`
  - `--rec-speed` (sim seconds per video second)
  - `--rec-stop-t`
  - `--rtx realtime|pathtracing --spp N`
- `run_mission.py`: about 30 lines of hooks for the above. The flag is named `--photoreal`, not `--look`, so it won't clash with your coming `--look pretty`. Feel free to have `--look pretty` call `photoreal.dress()`.

**Not changed:** `sim/` decision logic, the `state.json` contract, and truth access. The ghosts are still only shown with `--show-truth`.

**Next from us:** GPU bring-up (checklist steps 3–9), then footage for the shot list. Large files will go to a GitHub Release.
