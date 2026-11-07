# Isaac Sim brief for the teammate's Claude session

**Read this first, every time you pull.** It's the single, current, Isaac-only brief. Older docs (`ISAAC_HANDOFF.md`, `FOR_ISAAC_CLAUDE.md`) carry history; when they disagree, this file wins. Status labels:
- ✅ means committed and tested here: dry-run, tests, and USD checks without a GPU.
- 🟡 means it lands in the next push.
- 🎯 means you need to do it.

## 0. Coordination rules

- **Code:** branch `larpverse-sensing`, folder `larpverse/`. Our pushes replace that folder wholesale, so **don't edit it on that branch**.
- **Your work:** branch `larpverse-isaac` (rebased on `larpverse-sensing`) and branch `3d_model` (the map). We fetch both and merge your `isaac/` changes into ours. Your implementation stays canonical where it's GPU-verified; our builders adopted your photoreal pipeline, recorder and ribbon overlay that way.
- **Report back** in `larpverse/docs/FROM_ISAAC.md` on your branch, newest first. Include the Isaac version, the commands you ran, PASS/FAIL lines, contact-sheet paths and what you changed. Put large media in a GitHub Release.
- **Secrets:** `GEMINI_API_KEY` only goes in `larpverse/.env`, which is gitignored. Isaac doesn't need it.

## 1. What the demo is now

Heavy-lift drones (agricultural or industrial class) search the **real collapse site you built**: the 320 m × 320 m Weiguan Jinlong reconstruction plus the extrapolated neighbourhood. The story runs in this order:

1. **Recon:** with no zones tasked yet, the drones fly a coverage path and build a fresh LiDAR map, because satellite imagery predates the collapse. They perch to listen with rotors off, and the system proposes rubble outlines.
2. **Operator tasking:** the operator accepts or circles search zones on the dashboard. Zones already searched aren't re-searched without new evidence.
3. **Land and scan:** the drones land on flat cells (larger cells for the big drones). Radar looks for breathing: a ping marks a candidate position with depth, and the radar's reliable reach is about 3 m.
4. **Vital-sign triage:** the breathing pattern gives a triage hint. Rapid shallow breathing means possible distress; irregular breathing with pauses also scores high; calm, regular breathing scores lower; fast and small means possibly a child, at low confidence. The agent ranks the dig order from this.
5. **Listen and call out:** a speaker says "if you can hear me, knock three times" in several languages, then the drones listen. An open-model filter forwards unusual sounds to Gemini for multilingual analysis, and the results rank the zones. The drones dispatch to the top zone and the loop repeats.
6. **Operator view:** the operator sees pings, dig order, transcripts, zones and the LiDAR model with the depth marker.

Every radar return is a labelled synthetic generator (Isaac doesn't model breathing through rubble). Pings are **candidate search regions**, never "survivor found".

## 2. Runtime (verified on your box)

- Isaac Sim **6.0.1** + Isaac Lab **3.0** on Brev L40S, and a 2× RTX PRO 6000 box for path tracing.
- Launch through Isaac Lab (AppLauncher), or plain `SimulationApp` with `--launcher kit` (your `_isaac_sim/python.sh` route).
- Verified facts, with sources: `docs/ISAAC_VERIFIED_APIS.md`. These include scene queries enabled for PhysX raycasts, the black-viewport camera re-pose fix and the lighting values.

## 3. Commands

Set `L=/workspace/skysearch/larpverse`, then run each command from `/workspace/isaaclab`. Workstation: replace `./isaaclab.sh -p` with `~/isaacsim/python.sh` and add `--launcher kit`.

| Step | Command | Status |
|---|---|---|
| scipy (needed) | `./isaaclab.sh -p -m pip install scipy` | ✅ |
| Self-test | `./isaaclab.sh -p $L/isaac/run_mission.py --selftest --headless --enable_cameras --out /workspace/selftest` | ✅ |
| Dry run (no omni) | `./isaaclab.sh -p $L/isaac/run_mission.py --dry-run --max-steps 60 --out /tmp/d` | ✅ |
| Grid demo, photoreal, recorded | `./isaaclab.sh -p $L/isaac/run_mission.py --seed 7 --photoreal 1 --overlay usd --show-truth --record /workspace/frames --rec-cams director,wide,chase:d1,fpv:d2,top --rec-res 1920x1080 --rec-fps 30 --rec-speed 6 --rec-stop-t 380 --headless` | ✅ (story fix pending, §5) |
| **Your collapse map** | `./isaaclab.sh -p $L/isaac/run_mission.py --site collapse --site-usd /workspace/skysearch/collapse_site/collapse_site.usda --seed 7 --photoreal 1 --overlay usd …` | 🟡 |
| Heavy drones + dense LiDAR | add `--drone-model heavy --lidar-profile heavy`. Uses RTX LiDAR at an OS1-128 / Hesai-class density where available, otherwise more PhysX rays within a frame budget | 🟡 |
| Replay a CPU run (same story as the dashboard) | on any machine `python -m sim.grid_mission --seed 7 --radar dsp --frames --out dashboard/data`, then `… run_mission.py --replay $L/dashboard/data …` | ✅ |
| Path-traced hero shots | `--rtx pathtracing --spp 16` | ✅ |

**Flags you added, unchanged on our branch** ✅:
- `--photoreal`, `--assets`, `--hdri`, `--sky`, `--sun`, `--sun-elev`, `--veg`, `--operator`
- `--record`, `--rec-cams`, `--rec-res`, `--rec-fps`, `--rec-speed`, `--rec-warp`, `--rec-stop-t`, `--rec-windows` (`chase:d1:5-9` now parses), `--rec-every`
- `--rtx`, `--spp`

**Ours:**
- `--launcher auto|lab|kit`
- `--selftest`
- `--look pretty`, which is an alias for `--photoreal 1` plus Flow dust
- `--radar dsp`, the default
- `--site` 🟡

## 4. Overlay (`isaac/overlay.py`) ✅ merged with your ribbon lines (0f41c53)

- **Lines:** your `LINE_MESH` flat ribbons of exact width are canonical.
- **LiDAR:** a **single purple** (#9b6bff, brightness by height), shown only during the survey, then a 2.5 s fade-out (`LIDAR_SURVEY_ONLY`, `LIDAR_FADE_S`).
- **Ping:** an emissive pulsing red beacon (`BEACON_R`, `BEACON_GLOW`, `PULSE_HZ`) that reads in wide shots.
- **Acoustic cones:** shown only for human-classified clips with score ≥ 0.4.
- **Operator zones:** draped outlines coloured by status: tasked/unsearched dashed white, in-progress green, searched-no-ping grey, ping red, lead teal. Turn them off with `LARPVERSE_ZONES=0`.
- **Recon:** the coverage path, perch markers and proposed outlines (dashed grey).
- **Labels:** `label_specs()` returns the dashboard's pill-style text specs for burning in during post.

Colour language everywhere, which must match the dashboard and animation:

| Colour | Means |
|---|---|
| Red | Radar ping / candidate |
| Yellow | Human-audio direction |
| Teal | Acoustic priority |
| Purple | LiDAR |
| Green | Coordinator / dispatch |

## 5. 🎯 Your to-do list, in order

1. **Pull and switch.** After your current recording, rebase `larpverse-isaac` onto `larpverse-sensing`, or just use `larpverse-sensing`. Then tell us whether `3d_model` should be merged into it. That's our preference: it would put the map at `<repo>/collapse_site/`.
2. **Hold the final re-record** until we post **"READY TO RE-RECORD"** in `FOR_ISAAC_CLAUDE.md`. Three fixes are landing first:
   - A2 never pinged in your lv2 run because one disagreeing scan blocked the localisation. That's fixed on our side and being committed.
   - The acoustic ranking is getting cross-talk and misattribution fixes.
   - Vitals and triage on pings.

   When we post READY:
   - Record with `--rec-stop-t 380`.
   - **Gate each take on `timeline.csv`:** an A2 ping before t 110, and at least one dispatch before the stop.
   - If your Isaac story differs from the CPU run (your landing sites come from your own LiDAR), use `--replay` from our CPU frames so the video matches the dashboard.
3. **Collapse-site run** once `--site collapse` lands 🟡. Self-test it, then do one short run with `--max-steps 120`. Report:
   - raycast hits on your colliders;
   - landing cells found, so we can check footprint and clearance on your terrain;
   - frame time with debris on and off.
4. **Shots** for the 1-minute video, in order:
   1. Wide establishing shot of the whole collapse site at dusk.
   2. **Recon sweep:** three drones over the site while the purple LiDAR map paints in.
   3. One drone **perching**, rotors spinning down.
   4. Proposed zone outlines appear.
   5. **Mid shot** of drones landing on a rubble face near a void: 30–40 m out, 25° down, about 35 mm.
   6. Radar rings converge and the red beacon appears, with a ghost via `--show-truth` for the explanation shot only.
   7. Speaker **call-out** ring, then yellow **knock** replies.
   8. Dispatch line to the next zone.
   9. Top-down 80 m shot that mirrors the dashboard map.
5. **Scene look** (from our reviewer):
   - `--veg 0` on the collapse site, so it reads as urban, not a park.
   - Push fog start out to about 60 m and lower its density; the `fpv_d2` frame was hazy.
   - Tilt `fpv_d2` down 35–45° and put chase at 6 m behind, 3 m up.
   - Use Flow dust only for touchdowns.
6. **Identification overlays for post:** save each camera's pose and intrinsics per frame next to `timeline.csv`, so we can burn in pile/zone IDs, ping tags and a phase/clock lower-third with `scripts/cut_video.py`.
7. **Heavy drone model** 🟡: if you have a better agri/industrial airframe USD (6–8 rotors), say so. Ours is primitive-based with a sensor gimbal, LiDAR puck, radar panel and speaker horn.
8. **Licence:** keep a **procedural-texture variant** of the scanned core (no scan-photo albedo) for any public showing, unless NCKU grants permission for their model.

## 6. Answers we need in `FROM_ISAAC.md`

- Should `3d_model` merge into `larpverse-sensing`? Or which branch does the demo use?
- Do you have the 0.125 m heightfield as an `.npz`? It's faster for us than rasterising the USD.
- Which footprints are the original Weiguan blocks A–I, if known?
- Frame time on the L40S for the collapse site with 3 drones plus recording, with debris on and off.

## 7. Troubleshooting

| Symptom | Fix |
|---|---|
| No LiDAR points | Scene queries are off or colliders weren't cooked. Check the self-test raycast step and `ISAAC_VERIFIED_APIS.md`. On the collapse site the terrain cooks once at Play (1.9 M tris), so wait. |
| Black viewport | The camera re-pose fix is already in. Try `--camera free`. |
| Overlays missing in recordings | `--overlay usd` (ribbon meshes), not debug draw. |
| `--help` / dry-run crashes on `rubble` | scipy is missing. Install it (step 1). |
| Story differs from the dashboard | Use `--replay` on the CPU frames. |
| Slow frames on the collapse site | Hide `/CollapseSite/Debris` while iterating; `--rec-windows` renders only the cut ranges. |
