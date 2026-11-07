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


### 2026-11-07 18:25: first GPU bring-up (Isaac Sim 6.0.1 + Isaac Lab 3.0, Brev 1x L40S 48 GB)

- **Version / GPU:** Isaac Sim `6.0.1-rc.7` (Brev Isaac launchable, Isaac Lab 3.0.0), NVIDIA L40S, driver 595. Run with
  `/workspace/isaaclab/_isaac_sim/python.sh isaac/run_mission.py ...` from `larpverse/` (OMNI_KIT_ACCEPT_EULA=YES). scipy installed.
- **Test command:** `--seed 7 --headless --radar dsp --show-truth --photoreal 1 --max-steps 40 --record out/lv_test --rec-cams director,wide,chase:d1,fpv:d2,top --rec-res 1280x720 --rec-every 15`. Exit 0.

| Checklist step | Result |
|---|---|
| Stage built (~600 pieces) | PASS: `stage built: 621 rubble pieces` |
| Photoreal dressing | PASS: HDRI sky + sun 24°; crushed-concrete ground (4,572 faces); 621 irregular rubble pieces (183 with rebar); 34,200 debris stones; 70 grass/shrub clumps + 12 trees; operator model |
| LiDAR points > 0 | PASS: 17,440 at step 20, 30,048 at step 40; heightmap rebuilt from 10,359 points; 16 landing patches at A2 |
| Drones moving | PASS: d1 62.9 m, d2 67.6 m, d3 63.0 m by t=40 (survey → landing) |
| PNGs for every camera | PASS: 13 each for director, wide, chase_d1, fpv_d2, top |

- **Contact sheet:** `media/preview/isaac_lv_test_contact_sheet.png` (frame 10 of each camera).
- **What looks off:** (1) light-blue speckles all over the ground, apparently the LiDAR hit points drawn as an overlay. They read as noise in the wide shots; consider hiding them in `director`/`wide`, or thinning them out. (2) A haze band in the upper-left of `fpv_d2`. (3) One `[Error] Failed to resolve extension dependencies` line at startup, harmless (the run carries on).
- **Full recording:** started 18:23 on the same L40S: `--rec-res 1920x1080 --rec-fps 30 --rec-speed 6 --rec-warp 4 --rec-stop-t 250 --frames --out out/lv1_dash`. Clips go to `media/isaac/` when done.

