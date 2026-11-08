# Isaac Sim brief (full version): read this first

> **⚠ SUBMISSIONS CLOSE 8 Nov 13:00 GMT.** Rehearsal take now (stub acoustics allowed for the rehearsal), push the MP4 and a glitch list by 06:00 UTC, final take by 12:00 UTC. See FOR_ISAAC_CLAUDE.md, 02:20 entry.

**For:** Rohan's Claude session running Isaac Sim. **From:** the Larpverse build loop. **Updated:** 8 Nov, about 01:00.
- This file replaces `ISAAC_QUICK.md` and every earlier brief. `FOR_ISAAC_CLAUDE.md` is the running log.
- **Detail lives in `docs/VIDEO_PLAN.md`:** the second-by-second plan, OBS settings, pre-flight checklist and trigger fallbacks.

**Status marks:**
- ✅ dry-run or unit tested on our side (no GPU here);
- 🟡 a fix is landing in the next push;
- 🎯 you do it on the GPU.

**Nothing below has been rendered on a GPU yet.** Your rehearsal is the first time.

---

## ★ Operator ↔ Isaac I/O (the demo loop)

| Direction | Path | Where |
|---|---|---|
| **Input:** the operator selects an area | Dashboard "Select area → Send drones" → `POST /api/tasking` → `dashboard/data/tasking.json` | scripts/serve.py, on the GPU box |
| | `run_mission --live` polls tasking.json every 1 s. A new zone is an order: the drones leave the dock for it (`--start idle`: nothing launches before the first zone) | isaac/live.py, GridMission.order() |
| **Output:** what the drones sense and decide | `run_mission --live` writes `dashboard/data/state.json` (map, drones, pods, pings, priority list, clips, suggestions), `live.json` (causal lines, PLAN CHANGED banner, badge), `hud.json` (radar waveform), `motion.json` | served by serve.py; read by the dashboard and animation/live-view.html |
| Other operator inputs | `/api/broadcast` (speaker), `/api/comms/reply` (phones), `/api/portal` (phone reports) | serve.py |

**One command on the GPU box:** `L=/workspace/skysearch/larpverse ISAACLAB=/workspace/isaaclab bash $L/scripts/isaac_demo.sh`. It writes an empty tasking file, starts serve.py, runs the acoustic preflight, then starts the live Isaac run. Add `STUB_ACOUSTIC=1` for rehearsals; `DRY=1` runs the same loop without a GPU.

**Check the loop end to end** from the laptop (with the port forwarded): `python3 scripts/isaac_io_check.py --send-pile A1 --watch 90`. It tasks an area, then prints what Isaac publishes until the mission reacts (PASS/FAIL).

## 1. The deliverable

> **Simulation video.** Up to **1 minute**, **captured from the stream**. Show **perception on screen next to the actions**, and **one unscripted change** in a **single continuous take**.

How we meet it:
- **One OBS capture, no cuts.** On the left, the Isaac livestream (`/viewer`). On the right, the live dashboard from the same run. Optionally, an in-Isaac HUD overlay too.
- **One continuous camera** (`--camera director-take`): spline arcs and spring smoothing, never a cut. It's a per-phase time-lapse (×5–×30), and the HUD always shows "sped up ×N".
- **The unscripted change:** a teammate presses **Y** in the Isaac window. That switches on a hidden manikin's Indonesian voice at A1. The system **hears it at its next listen**, re-plans and redirects the drones on camera.
  - **Timing with the real acoustic pipeline:** press while A2's pods are down (about 17–29 s of video). A later press forces a full listen re-run while the mission waits (currently 50–62 s on a laptop CPU; being cut to ≤ 8 s and faster on the GPU box). The "~7 s reaction" was measured with stub acoustics.
  - Fallback trigger: the operator draws a zone on the dashboard.
  - Nothing is scheduled: `--test-trigger` refuses to run outside `--dry-run`.
- **Honesty boundary, enforced in code:**
  - The mission never reads the stage. It re-plans only from sensor-generator outputs, operator tasking and the phone portal.
  - LiDAR is real Isaac raycasting.
  - Radar is a labelled synthetic IF generator with the real DSP chain, fed by the dummies' live positions.
  - Audio is synthesised from the dummies' live positions and clips.
  - Pings are "candidate regions, not a confirmed person".

## 2. What the run looks like (seed 7, grid site)

1. **Van and lift-off:** the operator in the command van at three screens, then three heavy 8-rotor drones lift off. They move like real multirotors: spool-up, lean into acceleration, flare when braking, soft perch landings.
2. **Survey:** a purple LiDAR map paints over pile A2.
3. **Winch pods:** pods lower onto the rubble and run 45 s radar dwells. The HUD shows the breathing waveform.
4. **Red ping at A2:** "slow, regular breathing — deep rest or reduced consciousness — check responsiveness", with "aid for rescuers, not a diagnosis".
5. **Listen:** the drones land off the pile and the loudspeaker calls out (zh-TW, Hokkien, en). Knock and shout replies show as yellow cones with transcripts.
6. **★ Unscripted change:** a teammate presses **Y**. A new voice at A1 is heard and becomes the top lead, and the fleet redirects to A1. The on-screen transcript of the Indonesian call needs Gemini (quota permitting); offline Whisper `base.en` gives none. Without Gemini, the clip shows as "voice-like, language not identified".
7. **End:** the pods lower at A1, with the dig order on screen.

Colours, everywhere: red = radar ping, yellow = human-audio direction, teal = acoustic priority, purple = LiDAR, green = dispatch, **blue = rescue Wi-Fi / phones**, white dashed = operator zones.

## 3. Status

| Piece | Status |
|---|---|
| `--live` loop: stage as the source of truth, hotkeys U/Y, tasking poll, mission off the render thread | ✅ dry run: stage event at t400 → dispatch to A1 at t471 (verified by us tonight) |
| `--camera director-take` (no-cut spline + springs) | ✅ unit test: never jumps |
| HUD (`--hud overlay\|window`): LiDAR, cones, top clip, breathing waveform, vitals, decision, dig order | ✅ no-op without omni; 🎯 does it appear **in the stream**? |
| Command van (`--van`), manikin dummies with SpatialAudio (`--dummies`), ambient sound (`--ambient`) | ✅ USD-checked; 🎯 looks, emissive screens at dusk, audio in the stream |
| Heavy drone with its real components (BOM positions), rotor blur discs, winch pod hanging plumb with sway | ✅ USD-checked; 🎯 blur discs under RTX real-time |
| Smooth multirotor motion (`isaac.motion`, rendered per frame) | ✅ in the renderer; 🟡 the mission switches to `SmoothFlight` (correct arrival timing) in the next push |
| Pods visible in `--live` runs | 🟡 bug: our `--live` dry run showed 0 pods; being fixed |
| Real acoustic pipeline on live dummy sources (not the synthetic stub) | 🟡 landing; `VIDEO_PLAN.md` will state exactly what the box needs |
| Collapse site (`--site collapse --site-usd …`) | ✅ dry run (60 steps, 98k LiDAR points, 11 pod placements on T1a); the take plan is timed on the grid site |
| Frame-time settings (`--take-render`) | ✅ applied with try/except; 🎯 confirm in Render Settings |

**Before the final take:** rehearse now, but wait for **READY TO RE-RECORD** in `FOR_ISAAC_CLAUDE.md`. That comes once the three 🟡 items land.

## 4. Commands

Set `L=/workspace/skysearch/larpverse`; Isaac commands run from `/workspace/isaaclab`. On the workstation, replace `./isaaclab.sh -p` with `~/isaacsim/python.sh` and add `--launcher kit`.

**Box side** (leave running):
```bash
cd $L && python3 isaac/live.py --write-tasking default --out dashboard/data    # editable zones, revision 1
python3 scripts/serve.py --port 8765                                         # dashboard + /api/tasking, /api/broadcast, /api/portal
```

**Dry rehearsal** (any machine, no GPU, about 1 min; prints the stage event, quick listens, re-plan and dispatch) ✅:
```bash
python3 isaac/live.py --write-tasking default --out /tmp/take
python3 isaac/run_mission.py --dry-run --live --camera director-take --acoustic stub --max-steps 560 --out /tmp/take \
    --test-trigger "activate:hidden2@400"      # dry run only: stands in for the teammate's key press
```

**Self-test** (once per session) ✅:
```bash
./isaaclab.sh -p $L/isaac/run_mission.py --selftest --headless --enable_cameras --out /workspace/selftest
```

**The take** 🎯 (run the pre-flight first: it must PASS, and it prints the warm listen latency and the Gemini calls it projects):
```bash
python3 $L/isaac/live.py --preflight
./isaaclab.sh -p $L/isaac/run_mission.py --live --livestream 2 --viz kit --enable_cameras \
    --site grid --seed 7 --photoreal 1 --veg 0 --camera director-take --hud overlay --overlay usd --radar dsp
# collapse site instead: --site collapse --site-usd /workspace/skysearch/collapse_site/collapse_site.usda
#   (hidden dummies H1_RW_vi / H2_M39_id; re-time it with the dry rehearsal on --site collapse first)
```
On screen with `--live`: a causal line for every action ("d1-d3 → A1 BECAUSE voice heard, 3/3 drones agree…"), a **PLAN CHANGED** banner after a live input, an **AUTONOMOUS · no operator input** badge, and "not a person: settling debris → no dig" when the radar rejects a return. Speaker audio is off by default (licences); the ring still shows. Defaults that `--live` sets for you: van on (with `--photoreal 1`), dummies on, heavy drones, winch pods, realistic radar, tasking polled every 1 s, mission thread on, take render settings, keys `U=hidden1, Y=hidden2`, 4 s pre-roll on the van.

**Client side** (Rohan's laptop):
```bash
brev port-forward <instance> --port 8765:8765      # or: ssh -L 8765:localhost:8765 <brev-host>
# window 1: <brev URL>/viewer                         (Isaac stream; ONE viewer tab only)
# window 2: http://localhost:8765/dashboard/?layout=take&state=data/state.json   (same run, live; take layout)
```
OBS: one scene at 1920×1080 / 30 fps. Put the viewer at 0,0 sized 1280×1080 and the dashboard (`?layout=take`: a LIVE event line, a big map, the top dig order, the live radar breathing waveform, the top clip) at 1280,0 sized 640×1080. The take layout is also checked at 960×1080 if you prefer a 50/50 split. Full settings are in `VIDEO_PLAN.md` §6.5.

## 5. 🎯 Your to-do, in order

1. **Pull** `larpverse-sensing`. Run the **self-test** and the **dry rehearsal**.
2. **Stream and capture check:** does `/viewer` stream? Can OBS capture it? Does the stream carry **audio** (dummy voices)? Does the **HUD overlay show inside the stream**?
3. **Rehearsal take** on the grid site with the take command, recorded with OBS, at least 60 s. Note:
   - the **fps** on the survey shot (target ≥30 at 1080p; fallbacks in `VIDEO_PLAN.md` §5);
   - every glitch (pops, jitter, camera snaps, drones vs camera judder), with the video time;
   - whether pressing **Y** produced `STAGE EVENT … H2_A1_id activated (hotkey Y)` and a redirect, and how many video seconds it took.
4. **The same on the collapse site** (`--site collapse --site-usd …`) with debris on and off, for frame time only.
5. **Report** in `larpverse/docs/FROM_ISAAC.md` on `larpverse-isaac`: answers to 2–4, plus screenshots (van door shot, a drone close-up, pods on the rubble, the HUD).
6. **Final take** only after **READY TO RE-RECORD**. The teammate pressing Y must not be the camera operator. They decide the moment themselves inside the trigger window, which VIDEO_PLAN.md will re-time with real acoustics (currently about 17–29 s of video, while the pods are down).

## 6. Assets you'll see (`docs/ISAAC_ASSETS.md`)

- **Command van** at staging (`isaac/van.py`, about 12k triangles):
  - generic "SEARCH & RESCUE / MOBILE COMMAND UNIT" livery (no real organisation's branding), open side door;
  - operator in a hi-vis vest at three emissive dashboard screens;
  - 6 m mast with mesh antennas, a satellite panel, an amber light bar, a rescue tent, a generator and cones.
- **Manikin dummies** (`isaac/dummies.py`): partly buried training manikins, posed by condition. No injuries.
  - Each has a `SpatialAudio` prim and `larpverse:active|clip|physio` attributes.
  - Hidden ones: `H1_C3_vi` (key U) and `H2_A1_id` (key Y); on the collapse site, `H1_RW_vi` and `H2_M39_id`.
  - No colliders, so the LiDAR doesn't see buried people.
- **Heavy drone** (`isaac/drone_viz.py`, about 10k triangles):
  - parts at their real positions from `data/hardware_bom.json`: OS1-128 LiDAR on a 45° nose mount, thermal gimbal, 8-mic ring, speaker horn, Jetson + mesh radio with antennas, GNSS mast, blue Wi-Fi antenna, winch and pod;
  - `component_anchors()` gives label points for a close pass.
- **Audio** (`assets/audio/dummies/`): 126 clips. Real CC0 shouts, TTS voices (zh-TW wording / en / id / vi), knocks, weak voices, distractors, and through-rubble variants.
  - Credit ESC-50 (CC BY-NC) in the video description.
- **Radar signatures** (`assets/radar_signatures/`, gallery `docs/img/radar_signatures.png`): 4 breathing conditions plus 6 rejects, with HUD playback streams.

## 7. Questions for `FROM_ISAAC.md`

1. Stream: does it work, can OBS capture it, is there audio in it, and does the HUD show in it?
2. fps on the grid and collapse sites: debris on/off, photoreal on/off.
3. The glitch list from the rehearsal, with video times.
4. The hotkey: did Y register, and what was the reaction time?
5. How was the collapse-site core built? This is for the licence: public showing needs NCKU permission or a procedural core.
6. Should `3d_model` be merged into `larpverse-sensing`?

## 8. Troubleshooting

| Symptom | Fix |
|---|---|
| No LiDAR points | Scene queries are off or colliders aren't cooked: check the self-test raycast step. The collapse terrain cooks once at Play (1.9M triangles), so wait for it. |
| Black viewport | The camera re-pose fix is in; also try `--camera free`. |
| HUD not in the stream | Use the split screen (dashboard on the right) and keep going. |
| Key Y does nothing | Click the HUD button, or tick `larpverse:active` on `/World/Dummies/H2_A1_id`, or run `python3 isaac/live.py --cmd toggle --id H2_A1_id`. |
| No redirect by about 50 s of video | Fallback trigger: draw a zone around A1 on the dashboard (it reacts in 1–4 s). |
| Below 30 fps | `--photoreal 0`, then hide `/World/CollapseSite/Debris`, then `--drone-model small`, then stream at 1600×900. |
| Drones look frozen | `--mission-thread 1` (the default with `--live`). Otherwise lower the pace: `--live-pace "radar=10,relocate=20,rescan=20"`. |
| scipy missing | `./isaaclab.sh -p -m pip install scipy` |
