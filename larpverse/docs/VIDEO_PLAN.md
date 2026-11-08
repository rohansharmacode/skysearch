# The 60 s single take: plan, commands, checklist

**The rule.** "Simulation video. Up to 1 minute, captured from the stream. Show perception on screen next to the actions, and one unscripted change in a single continuous take."

**How we meet it.**
- **One continuous stream capture.** OBS records the Isaac livestream client plus the live dashboard fed by the same run (§1). The director camera moves on one spline and never cuts (§3).
- **Time-lapse, not editing.** The mission runs on a per-phase time-lapse, from ×5 to ×30. The HUD says so on every frame ("time-lapse ×N (sped up)"), and the voice-over or caption says it too. Nothing is cut, re-timed in post or pre-recorded.
- **One unscripted change.** A teammate who isn't driving the camera presses a key in the Isaac window. That switches on a hidden manikin's voice, or the operator draws a new zone on the dashboard. The system perceives it through its generators, re-plans, and redirects the drones on screen (§4). No trigger is scheduled: the run refuses `--test-trigger` outside `--dry-run`.

**Real perception, not the stub.** The take's listens go through the **real acoustic pipeline**: VAD, YAMNet classes, Whisper, DoA and the agent, in `.venv-audio`. It is fed the live dummies' own WAV clips at their live stage positions. `--live` runs a loud pre-flight and refuses to start if it would fall back to stub acoustics (§6.1b). The radar panel shows the actual pod scan's slow-time signal and its `vitals.identify` verdict.

Status: everything below is **dry-run tested here** (no GPU): `tests/test_live.py` and `python isaac/run_mission.py --dry-run --live`. The acoustic worker and pre-flight were also run here against `.venv-audio`: PASS, the dummy WAVs fed through `build_scene(sources=...)`, about 50–62 s per listen on this laptop CPU. The Isaac-only parts are guarded and **unverified on the L40S**: the omni.ui HUD, carb hotkeys, the render settings and SpatialAudio in the stream. The rehearsal (§6) checks each one.

## 0. TL;DR for Rohan

```bash
L=/workspace/skysearch/larpverse
cd $L && bash acoustic/setup.sh                                                  # once: .venv-audio + models (§6.1b)
python3 isaac/live.py --preflight                                                # must print PASS (real pipeline)
python3 isaac/live.py --write-tasking default --out dashboard/data              # editable zones, revision 1
python3 scripts/serve.py --port 8765 &                                           # dashboard + /api/tasking (same box)
cd /workspace/isaaclab && ./isaaclab.sh -p $L/isaac/run_mission.py --live --livestream 2 --viz kit --enable_cameras \
    --site grid --seed 7 --photoreal 1 --veg 0 --camera director-take --hud overlay --overlay usd --radar dsp
# client: <brev URL>/viewer (Isaac) + http://localhost:8765/dashboard/ (port-forwarded) side by side in OBS
# the unscripted change: a teammate presses  Y  in the Isaac window (H2, A1) while A2's pods are down (video ~17-30 s)
```

## 1. What is on screen: two layouts (recommendation: B)

| | A: in-Isaac HUD only | B: split screen in one OBS scene (recommended) |
|---|---|---|
| Frame | The Isaac stream, full 1920×1080, with the HUD overlay (`--hud overlay`) at the right of the viewport, or docked (`--hud window`) | The Isaac stream at 1280×1080 (left two-thirds) + the live dashboard at 640×1080 (right third), both live from the same run |
| Perception shown | LiDAR points/s + map coverage; bearing cones per drone + top clip (class, transcript, human score); the **radar slow-time chest displacement** of the pod scan (red/white on dark, last 10 s window) + range-bin intensity, with "analysing breathing... (n s)" until 45 s of dwell, then the whole-dwell `vitals.identify` verdict (early verdicts flip, docs/RADAR_SIGNATURES.md); the vitals label ("irregular breathing with pauses - check first") + "aid for rescuers, not a diagnosis"; the agent line + dig order; the stage-event line | Everything the dashboard draws: the LiDAR map, pings with depth, cones, transcripts, zones, vitals, dig order and decisions. In the left half: the HUD overlay as well, if it renders in the stream |
| Risk | omni.ui overlay in a `--livestream 2` stream is **unverified on 6.0.1**; if the viewport frame isn't in the streamed image, A shows no perception | Both halves are verified routes (dashboard on 8765 via the proxy / port-forward; Isaac /viewer). OBS composes the frame on the client, so nothing new runs on the GPU box |
| Honesty | Perception and actions in the same pixels | Literally "perception next to the actions" |

**What the judges should see in every frame.** These target perception-driven action, adaptation and autonomy. They appear on the HUD and in `live.json` / `hud.json` for the dashboard's event line.
- **Causal line: every action with its perception reason.** Built only from the mission's decisions, clips, pings, landing sites and scan notes. Examples:
  - "d1-d3 → A1 BECAUSE voice heard, 3/3 drones agree on bearing, priority 84"
  - "pod d2 placed at A2-P3 BECAUSE best-scored flat seat on the LiDAR map (slope 6°, roughness 1.2 cm, winch 6.8 m)"
  - "PING A2-R1 BECAUSE 3 agreeing radar ranges from 5 scans, breathing 9/min"
  - "C3: not a person: settling debris → no dig", the radar rejection shown as a perception-driven action
  - "new lead A1 (priority 84) BECAUSE a human-like sound was heard there twice where earlier listens heard nothing"
- **PLAN CHANGED banner, for about 5 s.** Example: "PLAN CHANGED: B2 → A1 (new voice heard, unscripted live input)". It fires when, after a live input, the fleet's next pile differs from the one ranked before the input, or when the input triggers a re-plan.
- **Autonomy badge.** It reads "AUTONOMOUS · no operator input" until a stage event, a tasking revision or a portal report arrives. Then it shows "LIVE INPUT · <what>" for 10 s, and afterwards "AUTONOMOUS · n live input(s), last at t …".
- **Decision attribution.** `[by: argmax | gemini | operator]` on the agent line.

**Speaker audio.** `--speaker-audio off` is the default with `--live`. The call-out renders use Piper voices with unclear or research-only licences (docs/DUMMY_AUDIO.md), so the take serves no call-out audio file. The call-out timing and the on-screen speaker ring are unchanged. `--speaker-audio espeak` serves eSpeak NG renders only; make them first with `python -m acoustic.speaker_assets --engine espeak --force`, which needs `espeak-ng` on the box.

**Recommendation: B**, with `--hud overlay` left on.
- If the rehearsal shows the overlay inside the streamed viewport, the left half carries the breathing waveform too.
- If it doesn't, nothing is lost: the HUD still writes `dashboard/data/hud.json` and `live.json` every frame (Request D1 asks the dashboard to show them).
- Use A only if the dashboard can't reach the client.

## 2. Second by second (seed 7, grid site, smooth flight, idle-drone listens, default pace)

Measured on the CPU with the current mission (`fa4e74b`: smooth flight, idle drones land to listen during rescans, native live hooks) and the pace table `isaac/live.py DEFAULT_PACE`. "Sim t" is mission seconds; pace is sim seconds per video second, shown on the HUD.

| Video s | Sim t | Pace | On screen (actions) | Perception panel | Camera (`director-take`, one spline) |
|---|---|---|---|---|---|
| 0–4 | 0 | pre-roll | Van at staging: open door, operator at the three screens, drones on the pads | "LIVE t 0"; zones on the dashboard | **van**: through the door from the SW |
| 4–8 | 1–20 | ×5 | Sequential lift-off; rotors spin up to blur; the drones lean into the climb | LiDAR 0 → first returns | **lift-off** crane, then **follow** with look-ahead |
| 8–15 | 20–65 | ×6 | Survey over A2: the purple LiDAR map paints in | LiDAR k pts/s, map coverage % | **survey**: 42 m out, 26 m up, slow orbit |
| 15–23 | 65–137 | ×5, then ×14 | Pods winch down; the first three radar dwells | The pod scan's chest displacement (red/white), "analysing breathing... (n s)" until 45 s of dwell, then the verdict; pod-mic listens | **pods**: 32 m, 25° down |
| 23–31 | 137–364 | ×30 | Relocations and rescans: one pod scans; **the idle drones land rotors-off and listen** | Each scan's signal and verdict; pod-mic listens (pods + landed drones) | slow orbit |
| ≈31 | 364 | | **Red ping beacon at A2-R1**. The listen landings are commanded, and with the real pipeline **the listen is submitted now** (prefetch on the landing patches) | **Vitals: "slow, regular breathing — deep rest or reduced consciousness — check responsiveness"** + "aid for rescuers, not a diagnosis"; dig order 1. A2-R1 | |
| 31–44 | 364–494 | ×10 | Drones land (rotors off, ≈34 s); call-out (zh-TW, nan-TW, en); replies | Bearing cones per drone; the top clip exactly as the pipeline returned it: Gemini transcript when available, else e.g. "voice-like, language not identified" | **listen**: 48 m, 30 m up, orbiting |
| ≈44 | 494 | | Gate: dispatch to the top-ranked pile (B2 without a change) | Agent line "dispatching d1-d3 to … [by: argmax or gemini or operator]: …" | **follow** toward the next pile |
| 44–60 | 494–575 | ×6 / ×5 | Flight, LiDAR survey and pods at the next pile | Coverage; first dwell of the new pile | **survey**, then **pods** |

**The trigger window for the take (real acoustic pipeline): video ≈17–30 s, while A2's pods are down and before the ping.**
- A press here is heard first by the **pod-mic listens** (pods resting + idle drones landed rotors-off), asynchronously on the real pipeline. Then it is heard by the gate listen, prefetched at ≈31 s, which already contains the changed world.
- **Press after ≈31 s.** The prefetched listen predates the change, so the gate listen is synthesised again (never a stale world). The gate then waits a full pipeline run while the landed drones are held and the camera keeps orbiting (§4).
- **Shifting window.** The acoustic agent is cutting the latency (target ≤ 8 s with per-stage timings). With ~8 s, a press up to ≈35 s still makes the gate. Re-measure with `--preflight` and move the window to suit.

**What the dummy trigger guarantees, and what it doesn't.**
- **Guaranteed:** perception. A new human-like lead at the dummy's sector, a new cone and a stage-event line. The pod-mic listen logs "NEW human-like sound at A1, confirmed by two consecutive listens".
- **Not guaranteed:** the redirect. The redirect depends on the ranking, which is computed, not scripted. If the operator **draws a zone** (§4 b), that is an order, and the fleet goes there after the dwell under way.

**Measured with STUB acoustics** (instant listens). These test the timing logic only. **They are not the take's reaction times**: with the real pipeline, add its latency (§6.1b).

| Trigger (stub) | Stage event | What follows |
|---|---|---|
| Y (H2, A1) during the rescans, t 251 (video ≈27 s) | t 251 | Pod-mic listens (pods + landed idle drones) confirm a new lead at A1, priority 84 (t 304). A2 has a firm ping in progress, so it is queued, not cut. At the gate (t 494) the ranking sent the fleet to **A3** (80): the gate listen heard A1 lower |
| Y during the gate listen, t 400 (before the mission's R1/A4 change) | t 400 | Gate at t 542: **A1** (84). The old timeline gave 6.8 s of video from press to redirect, a stub-acoustics number |
| Y in the first 3-pod dwell, t 91 | t 91 | Confirmed at t 125. A2 had no firm ping yet, so the radar phase is cut after the dwell and the fleet goes to A1. 2.8 s of video, stub only |

**Latency rule (automatic).** `--live` caps the listen pace at `66 / (latency + 1)` (floor ×4), using the warm latency the pre-flight measures (its second listen), and logs it.
- **Pipeline overruns:** the mission clock is held only while the drones are landed. The camera keeps its slow orbit and the HUD shows "real acoustic pipeline analysing… (n s)" (§4).
- **While the drones fly:** the mission runs ~0.75 s of video ahead of the picture (`--mission-thread 1`), so a slow step never freezes them.

**Real-pipeline check here** (seed 7; three drones landed around A2; A2 excluded; no Gemini):
- H2 off: A1 `no_signal_not_cleared`, priority 8.
- H2 on: A1 is the top pile at 56, with a voice clip from A1.
- Local Whisper is `base.en` (the multilingual download was declined), so zh/id/vi transcripts come only from **Gemini**. Without Gemini the HUD says "voice-like, language not identified" and promises nothing.
- Warm listen latency on this laptop: 50–72 s. The pre-flight prints the box's figure.

**Gemini budget.** The free tier allows about 20 requests per day per model.
- `--live` uses Gemini only on the **gate listens** (about 2 per take); pod-mic listens never use it.
- The pre-flight logs its own calls and projects the take's total.
- Every listen logs "Gemini calls: +n, N this run". Past `--gemini-budget` (default 20 minus the pre-flight's calls) the listens run without the Gemini stage.
- Run the pre-flight with `--gemini 0` (and `LLM_REPLAY=1`) on rehearsal days to save quota.

The ISAAC_BRIEF gate "A2 ping before t 110" is obsolete: A2 pings at t 364.

## 3. The camera: `--camera director-take`

`isaac/live.py TakeCamera`. Shots are chosen from the mission state each frame:

- **van** during the pre-roll;
- **lift-off**: crane up and pan to the pads;
- **follow**: the airborne fleet from behind, looking 8 m ahead along the velocity;
- **survey**: 42 m / 26 m, orbiting the active pile;
- **pods**: 32 m out, 25° down, centred between the lowered pods and the pile;
- **listen**: 48 m / 30 m;
- **event**: the live stage change, held 5 s;
- **end**: wide over the pings.

How it stays continuous:

- **No cuts.** A shot change starts a lifted cubic Bézier arc of 3–7 s. The arc moves the camera's *goal*, and the camera follows the goal through `isaac.motion.SmoothCamera`: critically damped springs, half-life 0.9 s for the eye and 0.45 s for the look-at. The built-in fallback uses the same springs. So position and velocity stay continuous through every re-target. `tests/test_live.py::test_take_camera_never_cuts` checks this: the per-frame step stays under 1.5 m and the acceleration stays bounded.
- **Black-viewport fix kept.** The pose is written to `/World/DirectorCam` and mirrored to `/OmniverseKit_Persp` every frame (isaac/camera.py).
- **Display only.** The camera may look at a stage event, because showing the viewer the teammate's action is the point. The mission never sees that event (§4).

## 4. The unscripted change: what it is and why it is honest

**Triggers.** All of them are live actions; none is scheduled.

| Trigger | How | What the system perceives | Typical reaction |
|---|---|---|---|
| **(a) Hidden dummy switched on** (recommended) | A teammate presses **Y** (H2_A1_id, Indonesian voice at A1) or **U** (H1_C3_vi, Vietnamese voice at C3) in the Isaac window: carb.input hotkeys, `--live-keys`. Kit may bind **H** to hide the selection, so we avoid it. Fallbacks, in order: the HUD button "Unscripted: switch on a hidden dummy" (same dummy as the last key, Y); tick `larpverse:active` in the Property panel; `python3 isaac/live.py --cmd toggle --id H2_A1_id` in the VS Code terminal | From the next scan or listen, the acoustic and call-out generators synthesise its voice from its stage position (a fresh, uncached synthesis), and the radar generator gets its chest. The mission hears it either in the next rotors-off listen or in a pod-mic quick listen (confirmed twice) | A new lead + cone (perception, guaranteed); a redirect at the gate (≈44 s video) if it outranks the others: computed, not scripted (§2) |
| **(b) Operator zone** | Draw or edit a zone on the dashboard. `POST /api/tasking` writes a new revision of `dashboard/data/tasking.json` | Polled every 1 s (wall). The newest zone is an order: the radar dwell under way finishes, then the fleet goes to it | 1–4 s of video |
| **(c) Drag a dummy or a debris slab** in the viewport | Select `/World/Dummies/<id>` (or a debris prim) and move it | The next radar scan uses the new position. A slab over a dummy is read by a PhysX column ray and deepens it. Debris needs a collider to count | Changes pings and cones from the next scan; the subtlest trigger to film |
| **(d) Phone portal "I'm trapped"** | Only once the comms work feeds `state.comms.devices` | A new trapped device with an RSSI fit becomes an order for its sector | As (b) |

**The honesty boundary.** Enforced in code, and tested in `tests/test_live.py`:
- **Reasoning never reads the stage.** `LiveController` re-plans only from:
  - generator **outputs**: a pod-mic quick listen that hears a human-like lead where an earlier listen, from no further away, heard nothing, confirmed by two consecutive listens;
  - operator tasking revisions;
  - portal devices.
- **Stage events are display and generator input only.** They change only cache freshness, the HUD event line and the camera.
- **Untouched dummies replay the scenario bit for bit.** The CPU dashboard run and the Isaac run tell the same story until someone touches the stage (`test_untouched_stage_is_the_scenario`).
- **Scripted triggers exist only in tests.** `--test-trigger` exits with an error outside `--dry-run`.
- **Labels say what is synthetic.** Every acoustic result says whether it came from the real pipeline or the synthetic generator: the HUD "Sources" line and `live.json` `last_listen.source`. Radar is synthetic IF plus the real DSP chain. Pings are "candidate regions, not a confirmed person".
- **Say it in the video.** Captions should say "sped up ×N (time-lapse)" and "a teammate switches on a hidden dummy, live".

## 5. Frame time: a steady ≥ 30 fps at 1080p

`--live` applies `isaac/live.py TAKE_RENDER` once; turn it off with `--take-render 0`. These are carb keys, each set in try/except and logged, and **unverified on 6.0.1**: confirm them in Render Settings during the rehearsal.

| Setting | Value | Why |
|---|---|---|
| `/rtx/rendermode` | `RaytracedLighting` (RTX real-time) | Path tracing is 6.5 s per frame (verified); never for the take |
| `/rtx/post/aa/op` | 3 (DLSS) | Render below 1080p, upscale |
| `/rtx/post/dlss/execMode` | 0 (performance) | Try 1 (balanced) if the LiDAR points shimmer |
| `/rtx/reflections/enabled`, `/rtx/translucency/enabled`, `/rtx/indirectDiffuse/enabled` | off | Rubble at dusk doesn't need them; the rotor blur discs are opacity, not refraction |
| `/rtx/directLighting/sampledLighting/samplesPerPixel` | 1 | |
| `/app/asyncRendering` | off | Pose writes land in their own frame (no drone-vs-camera judder) |
| `/app/runLoops/main/rateLimitFrequency` | 60 | |

**Scene and loop settings for the take:**

| What | Setting | Why |
|---|---|---|
| Vegetation | `--veg 0` | Grid and collapse sites read as urban; also saves frame time |
| LiDAR cost | `--lidar-budget-ms 4` (default) and `--overlay usd` | Point prims are capped. LiDAR is drawn only during the survey, then fades |
| Mission off the render thread | `--mission-thread 1` (default with `--live`) | A slow radar-DSP or acoustic step no longer stalls a frame. The renderer samples the trajectories at render time, `isaac.motion.Motion`, every frame. USD stays on the main thread |
| HUD | 4 Hz refresh | Throttled; the plots are 160 / 120 points |
| Debris instancing | Request P1 (photoreal owner) | Mark repeated rubble and debris pieces `instanceable` or use a PointInstancer. Today each piece is its own mesh |
| Offscreen detail (collapse site) | Hide `/World/CollapseSite/Debris` beyond about 60 m from the active pile, or set it `invisible` while iterating | ISAAC_BRIEF §7 |
| Dummies | Built in (deeper than 1.4 m uses low detail) | |
| Fog / haze | Fog start about 60 m (ISAAC_BRIEF §5) | |

**Measure before the take.** Watch the Kit FPS counter, and the `[isaac] step N` log lines, which should show no "fell behind" stalls. If it holds below 30 fps:
1. `--photoreal 0` (UsdPreviewSurface look);
2. `--drone-model small`;
3. hide Debris;
4. drop the stream to 1600×900.

## 6. Commands for Rohan

**0. Pull and install.** Branch `larpverse-sensing`; scipy is already in Isaac's python.

**1a. The real acoustic pipeline on the box (once).** We chose **the Brev box itself, in `.venv-audio`** over a laptop service:
- the same machine as Isaac, so nothing crosses the network during the take;
- the ~210 MB of models live on local disk;
- Whisper can use the GPU.

A laptop service would add a tunnel, a second machine to keep alive and a CPU Whisper (about 60 s per listen here).

```bash
cd $L
command -v uv || pip install --user uv                  # setup.sh uses uv (Python 3.12 venv, separate from Isaac's)
bash acoustic/setup.sh                                  # numpy scipy pyroomacoustics onnxruntime faster-whisper piper-tts
                                                         # soundfile ai-edge-litert; Silero VAD, YAMNet, Piper voice
uv pip install --python .venv-audio noisereduce        # denoise stage (docs/ACOUSTIC_GEMINI.md); not in setup.sh
.venv-audio/bin/python -c "from faster_whisper import WhisperModel as W; W('base.en', download_root='acoustic/models/whisper')"
# optional, faster Whisper on the L40S:  .venv-audio/bin/pip install nvidia-cublas-cu12 nvidia-cudnn-cu12
# zh/id/vi transcripts come from Gemini only (local Whisper is base.en; the multilingual download was declined):
#   put GEMINI_API_KEY in $L/.env (gitignored). Free tier ~20 calls/day/model: the take uses it on the ~2 gate listens
#   only, counts every call and stops at --gemini-budget (§2). Without a key: deterministic scorer, "language not identified".
```

**If the box cannot install it, copy ours.** `acoustic/models/` is 314 MB: `tar czf models.tgz acoustic/models`, then attach it to a Release and untar it on the box. Then run setup.sh, which skips existing files.

**1b. Pre-flight (every session; it fails loudly):**
```bash
python3 isaac/live.py --preflight         # starts the .venv-audio worker, switches a hidden dummy on, runs two listens
                                          # (first with Gemini, counted; second warm, timed). --gemini 0 saves quota
# PASS line: "real pipeline ... (mode auto, fed sources(build_scene)), warm listen N s"  -> note N (the latency)
# FAIL: a banner "ACOUSTIC PRE-FLIGHT FAILED ... The take would use STUB acoustics" and exit 1
```
`run_mission.py --live` runs the same pre-flight at start-up. If it fails, the run **exits**. `--allow-stub-acoustic` overrides that, but is for rehearsals only and must never be used for the take.

`fed sources(build_scene)` means the pipeline synthesised the arrays' audio from the dummies' own WAVs at their stage positions. `fed truth` would mean `listen_and_rank` had no sources route and used TTS from the transcripts instead; it is logged as a WARNING.

**1. Box side** (VS Code terminal on the Brev box):
```bash
L=/workspace/skysearch/larpverse
cd $L && python3 isaac/live.py --write-tasking default --out dashboard/data
python3 scripts/serve.py --port 8765            # leave running: dashboard + POST /api/tasking (+ portal when landed)
```

**2. Dry rehearsal on any machine (no GPU), about 1 min.** It uses the stub for speed, so it checks the timing logic, not the perception. It prints the stage event, the quick listens, the re-plan and the dispatch. Add `--acoustic real` to rehearse with the real pipeline: one listen per minute on a laptop.
```bash
python3 isaac/live.py --write-tasking default --out /tmp/take
python3 isaac/run_mission.py --dry-run --live --camera director-take --acoustic stub --max-steps 560 --out /tmp/take \
    --test-trigger "activate:hidden2@400"        # dry run only: stands in for the teammate's key press
```

**3. The take** (second terminal, from `/workspace/isaaclab`):
```bash
./isaaclab.sh -p $L/isaac/run_mission.py --live --livestream 2 --viz kit --enable_cameras \
    --site grid --seed 7 --photoreal 1 --veg 0 --camera director-take --hud overlay --overlay usd --radar dsp
# collapse site instead:  --site collapse --site-usd /workspace/skysearch/collapse_site/collapse_site.usda
#   (hidden dummies H1_RW_vi / H2_M39_id; re-measure §2 with step 2 using --site collapse first)
```

The `--live` defaults:
- `--tasking auto` (`dashboard/data/tasking.json`, polled every 1 s);
- `--acoustic auto`: on the box, every listen goes to the real pipeline worker (pre-flight first), fed the live dummies. Listens are prefetched when the drones are down; pod-mic listens run asynchronously; every result is uncached. There is no stub unless `--allow-stub-acoustic`;
- `--hud auto`, `--mission-thread 1`, `--take-render 1`;
- pace `DEFAULT_PACE` (`--live-pace`), 4 s pre-roll (`--live-preroll`);
- keys `U=hidden1,Y=hidden2` (`--live-keys`).

**4. Client side** (Rohan's laptop):
```bash
brev port-forward <instance> --port 8765:8765          # or: ssh -L 8765:localhost:8765 <brev-host>
# browser window 1: <brev URL>/viewer                      (Isaac WebRTC stream; ONE viewer tab only)
# browser window 2: http://localhost:8765/dashboard/       (same run, polled live)
```

**5. OBS** (one scene; nothing switches during the take):
- **Canvas and output:** 1920×1080, 30 fps (60 if the stream holds it). Recording: MKV, NVENC or x264 CBR 12 Mbps, keyframe 2 s, remux to MP4 after.
- **Source 1: Window Capture** of the /viewer window, cropped to the video element, placed at 0,0, 1280×1080 (layout B) or full canvas (layout A).
- **Source 2: Window Capture** (or a Browser source pointing at `http://localhost:8765/dashboard/`), placed at 1280,0, 640×1080. Set the dashboard to its narrow layout: it is checked at 400 px.
- **Audio:** desktop audio from the /viewer tab carries the dummies' SpatialAudio, if the WebRTC client carries it. This is unverified (ISAAC_ASSETS Request 7). Otherwise record without audio and say so.
- **Start/stop:** start OBS recording, then press Enter on the Isaac command (or start it first; the 4 s pre-roll holds on the van). Stop at 60 s on the clock.

## 7. Pre-flight checklist (every take)

1. Run `--selftest` once on the box today: raycast PASS, camera_switch PASS or WARN, render_frame not black.
1a. Run `python3 isaac/live.py --preflight`. It must print **PASS** with `fed sources(build_scene)`. Note the latency: above 10 s, the listen window can't absorb it (§2 latency rule), so put Whisper on the GPU or drop Gemini (§6.1a) before rolling.
2. Run the dry rehearsal (§6.2) and note the measured times.
3. Check the dashboard shows "revision 1, scenario-default" zones and the Isaac viewport shows the van door shot, then reset: rewrite `tasking.json` (§6.1) and restart `run_mission`.
4. Check the HUD overlay is visible **in the /viewer stream**. If not, use layout B with the dashboard carrying perception.
5. Check frame time ≥ 30 fps on the survey shot (§5).
6. Test the hotkey once in a scratch run: press **Y** and look for `STAGE EVENT … H2_A1_id activated (hotkey Y)` in the log and on the HUD. Then restart, so the take starts with the dummy off.
7. Brief the teammate, who must not be the camera operator. Press Y once while A2's pods are down (video ≈17–30 s, before the ping beacon). They decide the exact moment, or whether to draw a zone instead.
8. OBS: both windows in frame, recording armed, the clock visible to the director.
9. Close other /viewer tabs (only one client is allowed), then start.

## 8. If the trigger fails

| Symptom | Do this (still the same take; no cut) |
|---|---|
| Acoustic pre-flight fails | **No take.** Fix `.venv-audio` (§6.1a) or the models. A stub take would break the honesty rule |
| The gate shows "real acoustic pipeline analysing…" for a long time | The pipeline is slower than measured; the mission clock is held and the drones stay landed. Keep rolling if the 60 s still fits, else re-take after lowering the listen pace (`--live-pace "acoustic=5,..."`) |
| The key does nothing (no `STAGE EVENT` line) | Click the HUD button. Or tick `larpverse:active` on `/World/Dummies/H2_A1_id` in the Property panel. Or run `python3 isaac/live.py --cmd toggle --id H2_A1_id` in the box terminal (applied within 1 s) |
| Stage event seen but no redirect by video ≈50 s | The source may be out of earshot or below the priority of the planned target. Fall back to trigger (b): the operator draws a zone around A1 or C3 on the dashboard, which is an order and reacts in 1–4 s |
| The dashboard isn't reachable | Layout A, if the HUD shows in the stream. Otherwise re-check the port-forward before rolling |
| The stream stutters below 30 fps | §5 fallbacks, then re-take. A take is 60 s, so re-takes are cheap. Every re-take must again use a live, unscheduled press |
| The mission is behind the render (frozen-looking drones) | `--mission-thread 1` should prevent it. Otherwise lower the pace (`--live-pace "radar=10,relocate=20,rescan=20"`) |

If the reaction misses the 60 s window, keep the take only if the change and its perception are both on screen: a new cone, the stage-event line and the lead's transcript. Otherwise re-take.

## Appendix A. Requests: mission owner (`sim/grid_mission.py`): A1–A6 LANDED in fa4e74b

`isaac/live.py` now uses only the native hooks: `set_world_sources`, `ingest_listen` and `preempt`. The stub workarounds are removed, and `--live` refuses to start without the hooks. One small gap remains, **A7**: `preempt(reason, to_pile)` only acts while a pile is being searched. Between piles (listen, return), the live controller sets `mission._order` directly, which `_gate` honours first. A public `order(pile, reason)` would make that clean.

The original requests, kept for reference:

Today `isaac/live.py` installs a stub through the existing swap points (`scenario`, `sensor_factory`, `acoustic_fn`) and three private calls:
- `_finish_listen(gate=False)`, as recon does;
- `_finish_pile`;
- a `_gate` wrapper.

These native hooks would replace them. `LiveController` uses them automatically when present (`hasattr`).

**A1. `set_world_sources`.** A generator-only world override. **A2. `ingest_listen`.** An extra listen folded in without a gate. **A3. `preempt`.** Cut the radar phase after the dwell under way, then run the gate.

```diff
@@ class GridMission  (after _poll_tasking)
+    # ---- Isaac live take (isaac/live.py): generator-side world override + live re-planning hooks
+    def set_world_sources(self, scenario_view, sensor_factory=None, acoustic_fn=None):
+        """Generators read the victims / sounds from the Isaac stage: ``scenario_view.truth`` (call-out and acoustic
+        generators), ``sensor_factory(pile_world, seed)`` (radar, rebuilt per scan). The coordinator never reads
+        them. ``acoustic_fn`` bypasses the acoustic cache while the stage differs from the scenario."""
+        self.scenario = scenario_view
+        if sensor_factory is not None:
+            self.sensor_factory = sensor_factory
+            if self.pm is not None:
+                self.pm.sensors = sensor_factory(self.pm.world, self.seed)
+        if acoustic_fn is not None:
+            self.acoustic_fn = acoustic_fn
+
+    def ingest_listen(self, res, drones, source="pod-mic"):
+        """Fold an extra listen (pod mics resting on the rubble) into clips, pile priorities and zones; no gate."""
+        keep = self.acoustic_fn
+        self.acoustic_fn = lambda *_a: res
+        try:
+            self._finish_listen(drones=drones, gate=False)
+        finally:
+            self.acoustic_fn = keep
+        self._say(f"Extra listen ({source}) folded in: {len(res.get('clips', []))} clips")
+
+    def preempt(self, reason, to_pile=None):
+        """New evidence / operator order: end the active pile's radar phase once the dwell under way finishes, then
+        run the dispatch gate (``to_pile`` wins it). The cut pile stays not cleared and open for a revisit."""
+        self._preempt = (reason, to_pile)
+
@@ def step(self):  in the stage == "pile" branch, before `if self.pm.stage == "done" ...`
+            if getattr(self, "_preempt", None) and self.pm.scans and not self.pm.pending:
+                reason, to = self._preempt
+                self._preempt = None
+                self._say(f"RE-PLAN: {reason}; radar phase at {self.pm.pile} cut short after {len(self.pm.scans)} scans")
+                self._order, keep = to, self.listen_s
+                self.listen_s = 0.0
+                try:
+                    self._finish_pile()
+                finally:
+                    self.listen_s = keep
@@ def _gate(self, res):  first lines
+        if getattr(self, "_order", None) and self._order in self.piles:
+            to, self._order = self._order, None
+            self.dispatch.append({"t": round(self.t, 1), "from_pile": self.visited[-1], "to_pile": to,
+                                  "reason": "operator order / new evidence", "by": "operator"})
+            return self._start_pile(to, "operator order / new evidence")
```

**A4. Idle drones listen during rescans.** While one pod rescans, the other two drones hover, so there is no rotors-off listening point. Pod-mic quick listens need two or more, so a source that starts calling during A2's rescans (video 23–30 s) is heard only at the listen end. Please either let idle drones settle on a flat patch, rotors off, during a rescan, or lower their pods as listen-only mics.

**A5. Guard in `_finish_pile`.** `_radar_coverage(pm)` returns `radar_reach_m = None` when a pile ends with no completed scan. Then the `f"...{cov['radar_reach_m']:.1f}"` format raises `TypeError`, which I hit when cutting a pile early. The live controller now waits for at least one scan, but the mission should guard it too.

**A6. Clip language.** Copy `language` / `lang` from the acoustic clip into `acoustic_clips[]`, so the HUD and dashboard can show the language. The stub doesn't emit one; the real pipeline and Gemini do.

## Appendix B. Other requests

- **D1, dashboard owner.** Show `dashboard/data/live.json` (`last_event`, `replans`, `quick_listens`) and `hud.json` (`panels.radar.wave` / `range_db`: the live breathing waveform of the active pod scan). Ideally a narrow perception column for layout B: the event line, waveform, vitals label + "aid, not a diagnosis", agent line, dig order. Both files are rewritten every frame / step while `--live` runs. `dashboard/data/` is gitignored.
- **AC1, acoustic owner.** Add `sources=` to `acoustic.run.listen_and_rank` and pass it on to `build_scene(sources=...)`. Today the worker (`isaac/live.py _SERVER`) wraps `acoustic.scene.build_scene` / `acoustic.run.build_scene` to inject them. It switches to the parameter automatically once it exists. Each source is `{xy, clip: abs WAV, kind, level_db: 0, depth_m: 0 (rubble variants carry the loss), loop, gap_s, t0_s, id}`.
- **AC3, acoustic owner.** Report per-stage timings (synthesis, VAD, YAMNet, Whisper, agent) in the result, so the pre-flight can say where the ~60 s goes.
- **P1, photoreal owner.** Make debris and rubble pieces `instanceable` or use a PointInstancer for frame time (§5).
- **O1, orchestrator.**
  - Add the `--live`, `--camera director-take`, `--hud`, `--van` and `--dummies` lines to isaac/README.md and ISAAC_BRIEF §3.
  - Retire the "A2 before t 110" gate (§2).
  - Contracts: `live.json` (`larpverse.live/v1`) and `hud.json` (`larpverse.hud/v1`) are new, additive output files.
