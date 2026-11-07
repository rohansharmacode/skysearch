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

### 2026-11-07 19:55: case study chosen (Weiguan Jinlong collapse, Tainan 2016)

- **Site:** the demo is now framed on the **Weiguan Jinlong building collapse** (Yongkang District, Tainan, 6 Feb 2016): 16 floors plus a basement, nine blocks A–I in a U-shape facing west, and the whole building toppled west to east across Yongda Road Sec. 2.
  - Our site frame is anchored at **23.0051432 N, 120.2608904 E** (`sim/scenario.py` `SITE_GEO`, exported as `geo` in the scenario JSON).
- **The model the user wants:** Sketchfab "VR-Weiguan Collapsed Building (Feb.7,2016)" by the **NCKU Museum**.
  - It's drone photogrammetry (DJI Phantom 3, 177 photos, Agisoft PhotoScan), about **1M triangles / 510k vertices**, and georeferenced.
  - **The Sketchfab page shows no licence and no download option**, so we **can't import it until the owner grants permission** (the user is checking). Don't scrape it or rip it from the viewer.
  - If permission arrives: convert GLB → USD and add it as a static triangle-mesh collider (1M tris is fine on an L40S; decimate to about 300k for faster PhysX raycasts if needed), aligned to the geo anchor. Our LiDAR, landing cells and radar path would then use that mesh.
- **Fallback, worth preparing now if you have time:** a procedural stand-in at the same location and dimensions (a U-shaped 16-storey block toppled east across the road: pancaked slabs on their sides, column stubs, debris spill), behind a flag such as `--site weiguan`. The 3×3 sector grid overlays its footprint.
- **Radar realism (the user asked):** through-rubble radar reaches about **0.5–3 m** of mixed rubble or reinforced concrete (concrete loss is about 10–20 dB/m at 0.4–2 GHz). A 16-storey toppled tower therefore can't be scanned through, only from accessible faces and voids. We're adding this to the model: deep victims become **acoustic-only leads**, and the dashboard will show radar coverage. In Isaac, shots of drones landing **on the sides and edges of the collapse**, close to voids, tell that story well.

### 2026-11-07 19:35: got your GPU bring-up. Great work, keep recording.

- We read `FROM_ISAAC.md` and your commits up to `68036c8`: every step passed on 6.0.1/L40S, and the photoreal pass and multi-camera recorder look great (`pt_test.png` reads as a real disaster site).
- **Don't switch branches mid-recording.** We're merging your `isaac/photoreal.py`, `rubble.py`, overlay changes and run_mission hooks into `larpverse-sensing`. Your flags stay canonical; our `--look pretty` will just call `photoreal.dress()`. Once the merge lands, the next entry here will say so with the exact flag list. Then rebase or switch at your convenience, after your current recording.
- **Shot notes from us** (a fuller reviewer list follows):
  - Keep the LiDAR speckle off the wide and director cameras. Your `LIDAR_DRAW_R` already does this, so keep it.
  - Make the red ping beacon visible in the wide shots: a thicker beacon, or a short emissive pulse.
  - Add one shot that holds on the drones landed around A2 with range rings for about 3 s.
- **Coming on our side:** the acoustic fix (B1 faint voice ranked, no cross-talk), a radar depth estimate (`depth_source`, `depth_sigma` on pings), and A3 pinging with dsp radar by default. Your re-records after our next push will show the corrected pile order.
- Footage and large files go to a GitHub Release. A good place is the existing `larpverse-storyboard` pre-release; add a `larpverse-isaac-footage` one if you prefer.

### 2026-11-07 19:00: review fixes to the handoff

- §7 of `docs/ISAAC_HANDOFF.md` is now Brev-first. The order is: self-test, scene check, live run, dashboard, record.
- The radar now defaults to `--radar dsp`, because the model radar never pinged A3.
- The storyboard videos are at https://github.com/rohansharmacode/skysearch/releases/tag/larpverse-storyboard.
- Known open issue on our side: the acoustic ranking for seed 7 is being fixed. B1's faint voice was missed, and A3's voice leaked into B2 and B3. Until our next push, the pile order you see after A2 may not match the handoff. That's expected, and it isn't an Isaac problem.

### 2026-11-07 18:40: adapter aligned with your team's verified Isaac 6.0.1 notes. Run the self-test first.

```bash
cd /workspace/isaaclab && ./isaaclab.sh -p /workspace/skysearch/larpverse/isaac/run_mission.py --selftest --headless --enable_cameras --out /workspace/selftest
```

Paste the PASS/WARN/FAIL lines, and attach `selftest_frame.png` (via a Release), in `docs/FROM_ISAAC.md` on branch `larpverse-isaac`.

Once it passes, the full run is:
```bash
./isaaclab.sh -p /workspace/skysearch/larpverse/isaac/run_mission.py --seed 7 --radar dsp --look pretty --livestream 2 --show-truth --overlay usd
```

Changes:
- AppLauncher is the default launcher.
- Scene queries are enabled, so raycasts hit.
- The black-viewport fix is in.
- Lighting and materials come from your team's setup.
- New flags: `--look pretty`, `--rtx pathtracing` and `--record DIR`.

The details are in `docs/ISAAC_VERIFIED_APIS.md`.

### 2026-11-07 18:25: SLAM and Gemini landed

- **KISS-ICP SLAM** (`mapping/`): `build_pile_map(scenario, pile, seed)` simulates the drone LiDAR orbit, registers the scans with KISS-ICP and gives a point cloud and heightmap.
  - Seed 7, pile B2: 3.6 cm trajectory error, 4.7 cm map error, and 98% landing-cell agreement with the true geometry.
  - **For Isaac:** your PhysX/RTX LiDAR scans can go straight into `mapping.slam.run_kiss_icp(scans, odom_poses)`, then `accumulate`, then `rasterise`. Install kiss-icp in Isaac's Python first: `-m pip install kiss-icp`. This is optional; ground-truth poses also work.
- **Gemini decision agent** (`agents/`): it makes the dispatch decision (`--decider gemini`), reads operator notes from `ops/intel.txt` and writes the rescuer handover (`python -m agents.brief`).
  - Without `GEMINI_API_KEY` it falls back to the built-in choice, so you don't need a key to run Isaac.

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
