# From the Isaac Sim session (newest first)

### 2026-11-08 01:15: merged your full brief (b31fe88) into larpverse-isaac (c3c148b); dry-run findings

**Merge.** `larpverse-isaac` now has everything up to b31fe88 (live loop, director-take, HUD, van, dummies, heavy drone, PoseTrack motion, comms). Conflicts were only in `drone_viz.py` and `run_mission.py`:
- **run_mission:** resolved on *your* structure (PoseTrack, mission thread, `--live`, `director-take`, `hud.py`). Our extras sit on top and are off unless asked for:
  - `--camera story`;
  - `--overlay-style simple`;
  - `--line-scale` (renamed from `--viz-scale`; see below);
  - `isaac/sensor_fx.py` (LiDAR beams, radar pulses, sound domes, belief hotspot; `--fx 0` turns it off).
  - Our own Hermite smoothing (`--smooth`) is retired in favour of your PoseTrack.
- **drone_viz:** your heavy model is the default. `--drone-style mesh` (photoreal) keeps our smooth DJI-class body (`_build_parts_mesh`). Your integrated winch pods are used whenever the model carries one.

**Stream fixes from our on-box capture (live3, 1080p, 0 dropped frames), now in run_mission:**
- `--viz kit` crashed with "--viz-scale: invalid float value: 'kit'" (argparse prefix match). Renamed our flag to `--line-scale`, so your `--viz kit` works again.
- On the Lab route the stream was the whole Kit editor at 1440×900, with 739 frames dropped ("resolution differs from 1920x1080"). New `--stream-res 1920x1080` (default) adds `--/app/window/hideUi=true --/app/window/width=1920 --/app/window/height=1080` to `--kit_args` whenever `--livestream` is on, which gives a full-frame viewport.
- The Lab route starts stepping as soon as Kit is up, about 27 s before the stream connects. New `--hold-file PATH` renders the opening frame and holds t at 0 (before your pre-roll) until the file exists. Arm OBS, then `touch` the file.
- `--stop-t T` freezes the picture at sim t (ends dry and recorded runs).

**Your dry rehearsal (§6.2), identical on larpverse-sensing b31fe88 and on our merge:**
- It does **not** redirect to A1. At t 343 a portal phone report ("I'm trapped near B2", RSSI fit ±3.43 m) queues a re-plan to B2. The Y-stand-in at t 400 is then heard, but dispatch at t 485 is A2 → B2. Your brief expects A1 at t 471. It's the comms integration (aca080c, on by default). With the new `run_mission --comms 0` passthrough, the same rehearsal dispatches **A2 → A1 at t 485**. Should the take run with `--comms 0`, or is B2 intended?
- `tests/test_live.py::test_live_dryrun_with_trigger` fails on b31fe88 (`assert ([])`), likely the same cause.
- `tests/test_isaac_assets.py::test_heavy_parts_follow_the_bom` fails because `data/hardware_bom.json` isn't in the tree. `docs/HARDWARE.md` is missing too.
- Every run logs `isaac.motion failed (KeyError: 'no flight-motion trajectory for d1'); falling back to Hermite interpolation`. Is that expected until SmoothFlight lands?

**P1 (instanceable debris):** noted. We'll measure the survey-shot fps on the GPU first (the rehearsal running now) and do it if the fps is under 30.

**GPU rehearsal of the brief (self-test, stream/HUD/audio, Y take, collapse fps):** running now on the L40S from a clean checkout of b31fe88. Results follow in the next entry.

### 2026-11-08 00:40: rehearsal results (take1, live1) and the fixes now on larpverse-isaac (753999a, d01dcc6)

**Stream and capture (your Q1):**
- `--livestream 2` streams on the **Isaac Lab route** (`./isaaclab.sh -p run_mission.py ...`; Kit WebRTC on :49100, `/viewer` via nginx).
- On the **Kit launcher route** (`python.sh ... --launcher kit`) `--livestream` is ignored, and the Lab route writes **no PNGs** with `--record`. So: Lab route for the live take, Kit route for recorded backups.
- serve.py plus a private `brev port-forward ... -p 8765:8765` tunnel: the judge HUD, the dashboard and `/api/tasking` all work on Rohan's PC. Nothing is public.
- OBS capture, stream fps and audio are not measured yet: the viewer needs Rohan's own Brev sign-in in a browser on his PC.

**take1 glitch list (recorded, seed 7, director camera, 1080p30, rec-speed 4, 0.45 s per frame for 2 cams):**
- The story ran end to end: T1a beyond radar reach at t 87, zone OP-T1b added at t 116, dispatch to T1b at t 144.
- Look: too bright (14–28% clipped at f/9); rubble white with a blue cast; the fallen facade read as a clean grid; standing blocks had blurry blue scan facades; motion blur too strong.
- Clarity: the camera framed the tower, so the pods, the new zone and the dispatch happened off-centre. Zone outlines and dispatch arcs were invisible at that distance, LiDAR beams faint, radar rings small, drones too small.
- `heat 0` is expected: no radar lead on this seed, so the belief hotspot stays empty.

**Fixed (dry-run tested, `tests/test_isaac_dryrun.py` 8/8):**
- **Smooth heavy-multirotor motion** (`--smooth 1`, default). Monotone cubic paths one step behind the newest state (no overshoot, so no dips into the rubble). Tilt from acceleration plus drag, so the drone leans in and flares when braking.
- **`--camera story`:** one continuous state-driven shot.
  - It opens wide on the approach from staging and closes in on the active zone.
  - It widens for the listening domes and frames the drones' zone suggestion and the operator's new zone.
  - It holds on the zones while the drones fly home to swap batteries, then frames the dispatch flight in.
  - Critically damped springs and a slow orbit. Your `director-take` can replace it later; same interface.
- **`--viz-scale 4`:** zone and dispatch lines, rings, LiDAR beams and sound domes 4× wider for the wide shot.
- **Look:**
  - f/12;
  - darker, warmer rubble;
  - irregular, dirty fallen-tower facade (broken slabs, torn openings, 3 tints);
  - warm tint on the scan facade photos;
  - motion blur 0.15.
- **Drone hardware** (from your hardware list):
  - OS1-128 LiDAR on a nose bracket pitched 45°;
  - winch drum, and no radar panel in pod mode;
  - compute box, mesh radio with 2 antennas, blue Wi-Fi antenna;
  - red and green nav lights and a blinking white strobe.
- **Judge HUD:**
  - DECISION row (latest dispatch and why);
  - breathing condition on pings (`vitals.condition_text`);
  - green dispatch in the legend;
  - the correct step when opened late (fixes "Step 4 of 6" at done).
  - `isaac/hud/burn.js` burns the HUD into a recorded take, for the backup.
- **take2** rehearsal with all of this is rendering now (`--camera story --viz-scale 4 --lidar-profile heavy`).

### 2026-11-08 00:05: answers to ISAAC_QUICK.md, and what already exists for the single take (please don't rebuild it)

**Already on larpverse-isaac and dry-run tested.** Please build on these rather than duplicating them:
- **Live unscripted change:** `run_mission --tasking auto --out $L/dashboard/data`. The dashboard's "Add as zone" re-tasks the swarm mid-take: 5–30 s after the click in seeds 1–8, and ~28 s on seed 7 (zone added at t 116, dispatch at t 144). `isaac/tools/add_zone.py` gives the same write for rehearsals.
- **Dummies and acoustics in 3D** (`isaac/sensor_fx.py`, `--fx 1 --show-truth`):
  - grey half-sunk manikins at `truth.victims`;
  - teal wireframe sound domes from each sounding one while the fleet listens (grey for non-human);
  - LiDAR beams;
  - radar pulses around each scanning pod;
  - the belief hotspot.
  Your `--live` truth-from-Isaac and SpatialAudio can attach to the same `/World/SensorFX/Dummies/<id>` prims.
- **Perception on screen:** `isaac/hud/judge.html`, an OBS browser source over the stream that reads the same state.json. It shows:
  - step and caption, clock with the speed-up shown;
  - LiDAR, radar (last scan or ping), voices heard with an English gloss, non-human sounds, the drones' zone suggestion, zone status;
  - a mini-map;
  - an "operator added a zone" banner, then a "swarm adapts" banner.
  Screens: media/isaac/hud/ (next push). An in-Isaac `isaac/hud.py` would duplicate this. If you still want one, the radar breathing trace is the missing piece: add `scans[-1]` phase samples to state and judge.html will plot them.
- **Runbook:** `docs/LIVE_TAKE.md` (box, serve.py, SSH or `brev port-forward` tunnel, OBS scene, seed 7 story timeline).

**Your questions:**
1. Livestream: `--livestream 2` works on the L40S, and the Brev viewer `<brev URL>/viewer` loads behind the Brev sign-in. OBS capture at 1080p30 and audio in the stream are being tested now. Results to follow, with the rehearsal take and its glitch list.
2. Frame time (debris on/off, photoreal on/off, RTX modes): measuring now. For reference, the photoreal recorded run has been ~55 s for a short look test. Note that real-time is now `RealTimePathTracing` (Kit's default, DLAA plus ray reconstruction with `--rt-quality 1`).
3. Glitch list: comes with the rehearsal.
4. The core was not built by us: it is phupha-yao's `3d_model` generator, from screenshots of the NCKU scan. The Sketchfab page shows no download and no licence. The public-showing variant is `--urban-terrain mdl --no-scan-textures`.
5. Merging `3d_model` into sensing is phupha-yao's and Rohan's call. Our Isaac path only needs `collapse_site/collapse_site.usda` (`--site-usd`).

**The final take is still held** until READY TO RE-RECORD.

### 2026-11-07 23:45: your `--site collapse` merged into larpverse-isaac, live zones on the Isaac stage, single-take effects

**Merge (3b86342):** larpverse-sensing is merged into larpverse-isaac. Conflicts were resolved like this:
- `--site collapse` is now **your** scenario (`sim/site_collapse.py`, native `GridMission(site=...)`).
- Our compact 8-sector scenario is renamed `--site collapse-core`. It is used by `isaac/eval_collapse.py` and the case study.
- The map is loaded once, through `collapse_site.add_to_stage`, the GPU-tested path. Your `scene.build(site_usd=...)` path is kept but is not used for `--site collapse`.
- `--no-scan-textures` is honoured on our path.
- We kept our ribbon widths (`LINE_W 0.03`) and the urban-map dressing in `photoreal.dress`.

**New run_mission flags:**
- `--tasking auto|PATH|default`, passed to `GridMission(tasking=...)`;
- `--acoustic auto|stub|real`;
- `--fx 1` (below).

Tests: 185 passed and 5 skipped. Two things for you:
1. **Crash (please apply on larpverse-sensing):** `GridMission._finish_pile` raised `TypeError: unsupported format string passed to NoneType.__format__` when a sector had 0 pod placements, because `cov['radar_reach_m']` is None. Repro: seed 7 with an operator zone on T2b. On larpverse-isaac we guarded that one log line and changed no decision logic. During a live take this crash would end the take.
2. `tests/test_site_weiguan.py::test_grid_default_unchanged` fails on origin/larpverse-sensing itself, because the grid public md5 changed.

**Live operator zone for the judges' take** (the single continuous take, with "operator adds a new search zone" as the unscripted change):
- Run with `run_mission --tasking auto --out dashboard/data`. The dashboard's "Add as zone" (`scripts/serve.py` POST `/api/tasking`) then re-tasks the drones mid-take.
- `isaac/tools/add_zone.py` does the same file write for rehearsals: `--init T1a` before launch, and `--at 116` adds the drones' first sound suggestion.
- CPU rehearsal, seed 7: T1a survey ends t 46, pods and radar end t 88 with no ping, listening ends t 114 with voices near T1b and CN, the zone is added at ~116, and the swarm is dispatched to T1b at ~144.
- The re-task lands 5–30 s after the click in every seed we tried (1–8). The sector must have pod placements, otherwise the crash above.

**Sensor effects (`isaac/sensor_fx.py`, `--fx 1`):** these are display only, have no colliders and are never read by the mission.
- Purple LiDAR beams from each surveying drone to its newest returns.
- Orange radar pulses over the rubble around each scanning pod, out to ~3 m.
- Teal wireframe sound domes from each sounding dummy while the fleet listens (grey for non-human sounds).
- The localiser belief drawn as a glowing hotspot.
- With `--show-truth`: grey half-sunk dummies where the scenario hid its invented people. They replace the white capsules and are labelled simulated (R2-016).

**Look:**
- AI-refined scan textures: SD x4 upscaler, colour-locked to the original (`--scan-ai`).
- Kit's default RealTimePathTracing, with DLAA and ray reconstruction (`--rt-quality 1`).
- Facade v2 matched to the 2016 photos: thick grey floor slabs and dark crushed storeys.
- Excavators, a crane and fire trucks.

**Licence (your question):** we did not build the core. It comes from branch `3d_model` (phupha-yao's generator, from screenshots of the NCKU photogrammetry model). The Sketchfab model page (NCKU Cyber City lab) shows no download and no licence, so we have not used the mesh itself. For a public showing we still need NCKU's permission or the procedural variant (`--urban-terrain mdl --no-scan-textures`).

**Re-record** is still held until READY TO RE-RECORD.

### 2026-11-07 22:05: collapse site path-traced cut, and a Weiguan 2016 case-study simulation

**Path-traced collapse windows (034d94b, release `larpverse-collapse-pt`):** all 7 cameras, 1920×1080, 16 spp, `--replay` of the seed 9 CPU story (`python -m isaac.collapse_site --seed 9`), one process on the L40S, ~0.5 s per camera-frame (about 15 min for ~1,650 camera-frames). Review:
- **Read well:** `director`, `wide`, `map` (excellent ending shot, whole site north-up), `chase:d1` during deploy (operator and drones lifting off) and when landed on a slab, `mid` when the ping beacon and rings are up.
- **Weak on this map:** `fpv:d2` and `top` close-ups. The scan-photo texture is blobby within ~20 m, and the first-person view passes close to wreck faces. The cut uses director/chase there instead.
- Our 58 s cut opens with a case-study card (reported 2016 figures with sources, and a line saying the site is a reconstruction and the people, sounds, radar and drone data are simulated, per R2-016), then runs the mission with burned-in sector IDs, ping tags and a mission clock that only moves forward (no repeated action across shots).

**Case-study simulation (new, CPU only):** `isaac/eval_collapse.py` (evaluator: reads truth only after each run) runs the unchanged `GridMission` on `--site collapse` over 30 seeds × {acoustic, nearest, raster} × {legacy, literature} radar, plus an independent-start variant. Results: `docs/EVAL_COLLAPSE.md`; write-up with the 2016 figures and sources: `docs/CASE_STUDY_WEIGUAN.md`; figure: `docs/img/case_weiguan_timeline.png` (`python -m isaac.case_timeline`).
- Every sector is pinged, flagged or marked not cleared within 9.5–12.5 min of launch, against 52–60 h to the last live rescue and a 7–7.5 day search in 2016.
- With literature-grade radar, radar pings 3 of 305 invented people on this map (tall wrecks). The sectors flagged by listening hold 66% of them (74% with optimistic radar), against 1–30% for radar-only orders.
- Caveats are in the doc: about 10 invented people per seed, not 155; 7.2 of 8 sectors are occupied, so flagging is easy; `--acoustic stub` is an upper bound.
- `RASTER` for this map is set inside the evaluator (serpentine Z8 Z2 Z4 Z6 Z5 Z3 Z1 Z7 from the staging street); `sim/` is untouched.

### 2026-11-07 21:10: brief follow-ups (11eb199), and a regression you should know about

**Regression fixed: path-traced lines were back to 0.5 m tubes.** Your merge took `photoreal.dress()` from 68036c8, which sets `LINE_W = 0.006` but leaves `overlay.LINE_MESH` off. In RTX and path tracing, BasisCurves ignore that width and render as ~0.5 m yellow tubes (pt1 contact sheet). `dress()` now turns `LINE_MESH` on again (6–12 cm flat ribbons, as verified in pt2 at 85f7109). Please keep that line when you merge.

**New cameras for the shot list (§5.4, review T4):** `--rec-cams … ,mid,map`
- `mid` (or `mid:A2` to stay on one pile): 36 m out, 25° down, 35 mm lens on the 20.955 mm aperture (33° horizontal), on the active pile, with a slow ±6° drift for parallax. It sits WSW of the pile, so it's cross-lit by the SW sun. Use it for landing and radar (shots 5–6).
- `map`: straight down at 80 m, north up, east right, lens fitted to the site bounds (grid: 9.7 mm, whole site in frame). Sites too big for a 7 mm lens at 80 m get a higher camera instead (collapse site: 205 m). This is the ending shot that mirrors the dashboard (shot 9).
- `wide` and `map` use `isaac.site.bounds(sc)`, so they fit the collapse site. The grid keeps the exact orbit we recorded in lv2/pt2.

**Burn-in helper:** `photoreal.project(row, P)` (numpy only) turns world points into pixels for one `cameras.jsonl` row. It uses square pixels, the horizontal aperture fitted to the width and the principal point at the centre, which is how Kit renders a USD camera. It returns `(uv, depth)`; depth ≤ 0 means behind the camera. `scripts/cut_video.py` can import it, or copy it, to place pile/zone IDs and ping tags.

**Checks (no GPU):** offline with usd-core, every rig's target projects to (960, 540) at 1920×1080; `mid` is 36.0 m out at 25.0° down; `map` has all four bound corners in frame, east to the right, north up. `run_mission.py --dry-run --max-steps 60` PASS, and `tests/test_isaac_dryrun.py` 6 passed.

**Re-record command when you post READY** (L40S real-time first, then a path-traced pass of the chosen windows):
`--seed 7 --photoreal 1 --overlay usd --show-truth --record OUT --rec-cams director,wide,mid,map,chase:d1,fpv:d2,top --rec-res 1920x1080 --rec-fps 30 --rec-speed 6 --rec-warp 4 --rec-stop-t 380 --frames --out OUT_dash`, gated on `timeline.csv` (A2 ping before t 110, at least one dispatch).

**Media:** the 60 s path-traced draft cut of pt2 (A2 insufficient evidence → A3 ping → dispatch to B2, dashboard inserts) is at `larpverse/media/isaac/pt2/larpverse_grid_pathtraced_60s.mp4`. Its A3 shots are hazy; the haze settings from 3adca99 address that for the re-record.

**Licence variant (§5.8), status:** not a texture swap. `buildtex.py` derives the whole library from the scan photo (roof crops, texsynth exemplars), and `terrain_lowres.py` composites the photo into the core terrain tiles. So a scan-free core means re-running the generator with the photo weight at zero, plus non-photo roof and facade textures. Proposal: `collapse_site_noscan.usda`, a small override layer on `collapse_site.usda` that re-points the core terrain tiles and `scan_roofs` to regenerated textures. Tell us if pp would rather do it in the generator.

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

