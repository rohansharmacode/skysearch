# Isaac quick brief (SUPERSEDED: read docs/ISAAC_BRIEF.md)

**For:** Rohan's Claude session running Isaac Sim. **From:** the Larpverse build loop. **Date:** 7 Nov, ~23:20.
**Status:** this is the fast interim version. The full version, with the van, dummies, live triggers, HUD and smooth flight, follows in `docs/ISAAC_BRIEF.md` within about 1–1.5 h. Commands marked ✅ were dry-run-verified on our side tonight (no GPU here). Anything marked 🔜 isn't pushed yet.

## 1. The deliverable (hackathon rule)

> **Simulation video.** Up to **1 minute**, **captured from the stream**. Show **perception on screen next to the actions**, and **one unscripted change** in a **single continuous take**.

What this changes:
- **No cuts and no multi-camera edit.** The old shot list and `cut_video.py` plan are retired. Use one camera, moving continuously. Time can be sped up (`--speed`, `--rec-speed`), but say so on screen.
- **Capture the stream,** meaning the Isaac livestream client or viewport, with a screen recorder such as OBS. Don't use offline frame dumps for the final take.
- **Perception on screen:** next to the 3D action, show what the system senses: the LiDAR map painting in, sound-direction cones and the top clip, the radar breathing trace and ping, the condition label, and the agent's decision. In order of preference: an in-Isaac HUD panel 🔜, or the live dashboard placed beside the stream in the same OBS scene.
- **One unscripted change, live:** someone does something the system wasn't told about, and the drones visibly react. Options 🔜: a hotkey switches on a hidden dummy's voice, someone drags a dummy or a debris slab, the operator circles a new zone on the dashboard, or a phone joins the rescue Wi-Fi and taps "I'm trapped".
- **It has to look great:** drones must move like real heavy multirotors (tilting into acceleration, braking flare, soft perch landings, spinning rotors). The current straight-line kinematic motion looks janky, and a smooth flight-motion model is being built now 🔜.

## 2. Do these now, in order

Set `L=/workspace/skysearch/larpverse`, then run from `/workspace/isaaclab`. On the workstation, replace `./isaaclab.sh -p` with `~/isaacsim/python.sh` and add `--launcher kit`.

1. **Pull** branch `larpverse-sensing` (our code lives in `larpverse/`, and our pushes replace that folder, so don't edit it there).
2. **Check streaming and capture.** This is the most important answer we need:
   - Does `--livestream 2` work on your box, and can you view it from your laptop (WebRTC client or `<brev URL>/viewer`)?
   - Can you screen-record the client with OBS at 1080p and 30 fps or more? Does the stream carry audio? We have dummy voice clips coming.
   - Is the dashboard reachable from the same laptop? `python3 $L/scripts/serve.py --port 8765` runs on the Brev box, then `ssh -L 8765:localhost:8765 …` and open `http://localhost:8765/dashboard/?state=data/state.json&poll=1000`.
3. **Collapse site plus heavy drones plus belly radar landings** (`--delivery pod` is the radar-on-the-rubble-surface scan model), first as a dry run and then on the GPU:

   | Step | Command | Status |
   |---|---|---|
   | Dry run (no omni) | `./isaaclab.sh -p $L/isaac/run_mission.py --dry-run --site collapse --lidar-profile heavy --platform heavy_sar --delivery pod --max-steps 60 --out /tmp/d` | ✅ (60 steps, 98k LiDAR points, 11 radar placements on T1a) |
   | Live on GPU | `./isaaclab.sh -p $L/isaac/run_mission.py --site collapse --site-usd /workspace/skysearch/collapse_site/collapse_site.usda --platform heavy_sar --lidar-profile heavy --delivery pod --show-truth --speed 4 --livestream 2` | ✅ dry / GPU untested |
   | Grid site (fallback) | `./isaaclab.sh -p $L/isaac/run_mission.py --platform heavy_sar --delivery pod --speed 4 --livestream 2` | ✅ dry |
   | Self-test | `./isaaclab.sh -p $L/isaac/run_mission.py --selftest --headless --enable_cameras --out /workspace/selftest` | ✅ (grid scene only) |

4. **Measure** the frame rate in the stream on the collapse site with 3 drones: debris on and off, `--photoreal 1` on and off, and RTX real-time vs path tracing. **The target is a steady 30 fps or more at 1080p in the stream.** If it's lower, hide `/CollapseSite/Debris` while testing, and tell us which settings you used.
5. **Rehearse the single take** with what exists today: one continuous `--camera director` run, captured from the stream with OBS, 60–90 s. It's not the final; it's to find capture, frame-time and jank problems early. Note every visual glitch you see (pops, teleports, jitter, camera snaps) with the mission time.

## 3. Facts you need

- **The story:** heavy drones (FlyCart 30 / Agras T50 class) do recon mapping, then listen with rotors off. A loudspeaker asks "knock three times". The drone finds a flat spot on its LiDAR map and **lands** there, so its belly radar and microphones rest on the rubble (realistic radar reach through rubble is only about 3 m). The breathing pattern gives a condition hint ("irregular breathing with pauses — check first"; no age guesses), and the agent ranks the dig order.
- **Where things stand:** seed 7 on the grid now pings **all four victims** (A2, A3, B1, B2) with realistic penetration and the landed radar. A more realistic breathing waveform is being calibrated, so this may change honestly.
- **Colours**, the same everywhere: red = radar ping, yellow = human-audio direction, teal = acoustic priority, purple = LiDAR (shown only during the survey), green = dispatch, **blue = rescue Wi-Fi / phones** (new), white dashed = operator zones.
- **Real hardware for close-ups** (`docs/HARDWARE.md`, `data/hardware_bom.json` → `isaac_mounts`): an Ouster OS1-128 LiDAR pitched 45° on the nose; a belly radar module of 0.5 × 0.5 × 0.25 m (radar face, 4 corner mics, strobe); a speaker; an 8-mic ring; a compute box; a mesh radio plus two antennas; a GNSS mast; a blue Wi-Fi antenna. There is no separate radar panel on the drone body.
- **Licence:** the scanned core of the collapse map is derived from third-party scan screenshots. Before any **public** showing we need NCKU's permission or a procedural core. Please tell us how you built the core.

## 3a. 🔜 Coming in the full brief (about 1–1.5 h)

- **`--live`:** Isaac becomes the source of truth for victim dummies, so a live change is genuinely sensed. Also hotkeys for the unscripted change, and tasking and phone-portal triggers.
- **`--camera director-take`:** one smooth spline camera path with spring smoothing, for the single take.
- **An in-Isaac perception HUD** (`isaac/hud.py`): radar breathing trace, cones, condition label, decision line.
- **An operator command van** at staging: a monitor wall showing the dashboard, a mesh mast, a satellite dish, a light bar, a tent and a generator.
- **Victim dummies:** rescue-training manikins partly buried, each playing its own audio clip (SpatialAudio), plus hidden dummies for the live change.
- **Real multirotor motion:** jerk-limited trajectories, tilt and flare, perch landings, rotor RPM, and render-time interpolation so motion stays smooth at any speed.
- **A second-by-second 60 s plan** (`docs/VIDEO_PLAN.md`) with exact OBS and stream settings and a pre-flight checklist.

## 4. Reply in `larpverse/docs/FROM_ISAAC.md` on branch `larpverse-isaac`

1. Livestream works? Can it be captured with OBS? Is there audio in the stream?
2. Frame time on the collapse site: 3 drones, debris on/off, photoreal on/off.
3. The glitch list from the rehearsal take.
4. How the collapse-site core was made (licence).
5. Should `3d_model` be merged into `larpverse-sensing`?

**Don't** record the final take until we post **READY TO RE-RECORD** in `docs/FOR_ISAAC_CLAUDE.md`.
