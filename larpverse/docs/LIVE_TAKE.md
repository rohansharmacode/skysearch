# The judges' live take: one continuous minute, captured from the stream

**Rules (team, 2026-11-07):**
- Up to 1 minute, captured from the stream.
- Perception shown on screen next to the actions.
- One continuous take with no cuts.
- At least one unscripted change that the system adapts to live.

**Our unscripted change:** the operator adds a new search zone mid-mission.

## What the judges see

The OBS canvas is 1920×1080. It has two layers:
1. **Isaac Sim live stream** (WebRTC viewer, full frame). The real collapse map is shown with the facade matched to the 2016 photos, AI-refined scan textures, real-time path tracing with DLAA and the AI denoiser, and the drones.
   - Sensor effects are drawn in 3D (`isaac/sensor_fx.py`):
     - purple LiDAR beams and points;
     - orange radar pulses around each landed drone's belly radar;
     - teal sound domes from the grey dummies (simulated people) while the drones listen;
     - the belief hotspot.
   - Overlay lines (`isaac/overlay.py`): zones, range rings, acoustic cones, dispatch arcs, red ping beacons.
2. **Judge HUD** (`isaac/hud/judge.html`, a transparent browser source on top). It shows:
   - the step and a one-sentence caption;
   - the clock;
   - "what the swarm perceives right now";
   - the operator mini-map;
   - the colour legend;
   - a banner for the operator's change and for the swarm's re-plan.

The operator uses the team dashboard (`dashboard/`, "Add as zone") in a second browser window, which is not captured. Optionally, show the click as a small inset to prove it is live.

## Story with seed 7 (CPU rehearsal; the times are sim seconds, ×4 → video seconds)

| sim t | video s | what happens |
|---|---|---|
| 0–46 | 0–12 | launch from the staging lot; LiDAR survey of zone T1a (the operator's pre-launch zone) |
| 47–88 | 12–22 | 3 drones land on the rubble and their belly radars scan; no breathing signal (not cleared) |
| 88–114 | 22–29 | rotors off and listening: voices are heard near T1b ("Help! … my leg is trapped") and near CN, and the drones suggest a new zone |
| ~116 | ~29 | **the operator clicks "Add as zone" (live)** |
| ~130–144 | 33–36 | the coordinator re-plans, and d1–d3 are dispatched to the new zone |
| 150–200 | 38–50 | LiDAR survey of the new zone, then landed radar scans |

## Commands

On the GPU box, set `L=<repo>/larpverse` and `D=$L/dashboard/data`. (`--delivery pod` below is the radar-on-the-rubble-surface scan model.)

```bash
python3 $L/isaac/tools/add_zone.py --out $D --init T1a --site collapse --seed 7     # pre-launch zone (revision 1)
python3 $L/scripts/serve.py --port 8765 &                                          # dashboard + HUD + /api/tasking
./isaaclab.sh -p $L/isaac/run_mission.py --site collapse --site-usd <repo>/collapse_site/collapse_site.usda \
  --seed 7 --photoreal 1 --overlay usd --show-truth --fx 1 --tasking auto --out $D \
  --platform heavy_sar --delivery pod --penetration literature --acoustic stub \
  --scan-ai /workspace/scan_ai --rt-quality 1 --speed 4 --livestream 2 --headless
```

On Rohan's PC:
- Tunnel the box's port 8765 to localhost without making it public, e.g. `brev port-forward <instance> --port 8765:8765`.
- Viewer: `https://isaac-…brevlab.com/viewer` (Brev sign-in).
- HUD: `http://localhost:8765/isaac/hud/judge.html?speed=4`.
- Dashboard: `http://localhost:8765/dashboard/`.

OBS:
- Scene "Take": Window Capture (the viewer, full frame) plus Browser Source (the HUD URL, 1920×1080, transparent).
- Record at 1080p30, then cut nothing.

**Rehearsal without a person:** `python3 $L/isaac/tools/add_zone.py --out $D --at 116 &` does the "Add as zone" file write at sim t 116.

**Backup:** the same run headless with `--record … --rec-cams director --rec-speed 4 --rec-stop-t 210` instead of `--livestream`. It is one continuous camera, but not captured from the stream.

## Honesty labels (R2-016)

- The site is a reconstruction.
- The people (grey dummies), their sounds and the radar returns are synthetic, and the drones never see the dummies.
- A ping is a candidate search region, never a confirmed person.
- The HUD carries this line permanently.
