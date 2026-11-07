# From the Isaac Sim session (newest first)

### 2026-11-07 21:00: ISAAC_BRIEF.md pass (branch `larpverse-isaac`, rebased on `larpverse-sensing` 451feeb)

**Branch:** `larpverse-isaac` now starts from `larpverse-sensing`, because your tree already has every GPU-verified `isaac/` change: ribbons, Recorder, `--rec-windows`, `timeline.csv` and path tracing. On top of that it carries only the GPU media (lv2/pt2 timelines, dashboard state, contact sheets) plus the brief items below. Delivered as a merge commit (3adca99, parents: the old `larpverse-isaac` tip 277b520 and sensing 451feeb) whose tree equals the rebased result, so there was no force-push and no history was lost.

**Done from §5 (no GPU needed; dry run PASS, `--help` lists the flags):**
- **5, scene look:**
  - `--fog-intensity` 0.35 → **0.18**, `--fog-end` 450 → **700 m**, `--fog-start` stays 70 m (≥ the 60 m you asked for). The values are logged as `[photoreal] haze: …`.
  - `--fpv-tilt` default **40°** down (was ~19°).
  - Chase rig at **6 m behind, 3 m up** (`--chase-back`, `--chase-up`).
  - `--veg` now defaults to **auto**: off on `collapse` / `weiguan` / `mesh:` sites, on for the grid. `--veg 0/1` forces it.
  - Flow dust is untouched; your `--look pretty` decides.
- **6, identification overlays:** the Recorder writes **`cameras.jsonl`** next to `timeline.csv`. One line per saved frame and camera: `frame, cam, width, height, world` (USD row-major 4×4; camera looks down local −Z, +Y up), `focal_mm, h_aperture_mm, v_aperture_mm, clip`. That's enough to project pile/zone IDs and ping tags in `scripts/cut_video.py`. Failures only log once and never stop a recording.
- **Held:** no re-record until you post **READY TO RE-RECORD**. Both GPUs are off: the L40S is stopped and the RTX PRO 6000 box is deleted.

**Answers to §6:**
1. **Merge `3d_model`?** Yes. Merge only `collapse_site/` (generator, `collapse_site.usda`, `geo/*.usdc`, textures, previews) into `larpverse-sensing` at `<repo>/collapse_site/`. The branch is based on the old `main` (292551d), so don't take its root `README.md` / `.gitignore`. The demo should run from `larpverse-sensing`/`larpverse-isaac` with `--site collapse --site-usd <repo>/collapse_site/collapse_site.usda`.
2. **0.125 m heightfield as `.npz`?** Yes, regenerated from the `3d_model` generator (`prep.py` → `outer_layout.py` → `heightfield.py`) and committed as **`larpverse/media/isaac/collapse_heightfield_0p125m.npz`** (16.5 MB). It holds `H` float32 2561×2561 (terrain z in metres, −0.26…5.97; buildings and debris are separate USD prims and not in H), `res=0.125`, `extent=160`. Row 0 is y=+160 (north), column 0 is x=−160 (west). `col = (x+160)/0.125`, `row = (160−y)/0.125`, in the site frame (x east, y north).
3. **Original Weiguan blocks A–I?** Not labelled in the model. The scanned core in `collapse_site/generator/sitecfg.py` names `CORE_BUILDINGS` by position (NW_warehouse, N_site_office, N_damaged_hall, N_shed, NE_hut, E_tank, **Block6_remnant**, W_long_shed, S_warehouse, SE_red_roof, SE_white, SE_tower, SE_green_roof) plus the two fallen 16-storey towers (`SLABS`: W_tower, E_tower). Only `Block6_remnant` carries a block number. I wouldn't map A–I onto these without a source.
4. **Frame time on the L40S, collapse site + 3 drones + recording, debris on/off:** not measured yet. It needs `--site collapse` (🟡 on your side) and a GPU run. I'll measure it in the first collapse-site self-test after READY. From the forest and earthquake runs: real-time RTX with 3–5 cams at 1080p was ~2 s per recorded frame on the L40S, and path tracing at 16 spp was <1 s/frame per camera on an RTX PRO 6000.

**Licence (§5.8):** noted. For any public showing I'll render a procedural-texture variant of the scanned core (no scan-photo albedo).

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

