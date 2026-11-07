# Live demo runbook (5 minutes) and 1-minute video plan

One command runs the whole demo: `scripts/demo.sh`. It runs the seed-7 autonomous grid mission at 4× speed (`--speed N` to change) and writes `dashboard/data/state.json`, which the dashboard polls every second. At the end it prints the result and the rescuer handover brief.

Everything below assumes the repo root (`/home/peroxide/Downloads/UCL/agentverse`) and the dev server on :8765.

## T-30 min: set up and rehearse

1. **Dev server.** `curl -sI http://localhost:8765/dashboard/ | head -1` should print `200`. If it doesn't, start the server in its own terminal and leave it running:
   `python3 -m http.server 8765 --bind 127.0.0.1 --directory /home/peroxide/Downloads/UCL/agentverse`
2. **Gemini (optional).** Put `GEMINI_API_KEY=…` in `.env`. Never show `.env` on screen. Without a key the demo still works: the dispatch gate uses argmax and the brief uses the template, and the dashboard labels both honestly.
3. **Rehearsal run, which also warms the caches:** `scripts/demo.sh --fast` takes about 1.5 min. Seed 7 is deterministic, so the live run then reuses the cached acoustic results in `dashboard/data/acoustic_cache/` and runs at the same speed every time.
4. **Back up the rehearsal run for replay mode:**
   `mkdir -p dashboard/data/backup && cp -r dashboard/data/frames dashboard/data/pointcloud.bin dashboard/data/backup/`
5. **Browser tabs, in this order:**
   1. Concept animation: http://localhost:8765/animation/grid-search.html (no `#s…` hash: the plain URL plays the seed-7 scenario)
   2. Dashboard: http://localhost:8765/dashboard/
   3. `media/larpverse-dashboard-seed7.mp4`, the last-resort video.
6. **Terminal.** Use a large font with the repo root as the working directory. Type `scripts/demo.sh` but don't press Enter yet.

## Run of show

The "Sim t" column is the mission clock shown top right on the dashboard (`t = …`). At the default ×4, the clock advances 4 s per wall second. With warm caches, compute adds only about 10 s in total, so the mission runs about 3:15 from Enter to "Mission complete". At ×6 it ran in 134 s. The important beats (three pings) are over by sim t ≈ 340, about 1:30 after Enter. Use the time after that (the C3 phase) for the click-through, but don't present C3 itself: see the known issue below.

| Wall | Sim t | Do | Say |
|---|---|---|---|
| 0:00 | – | Tab 1, concept animation, playing | "After a building collapse, rescuers face a field of rubble piles and little time. Larpverse sends three drones to find where to dig first." |
| 0:30 | – | Press Enter on `scripts/demo.sh`. Switch to tab 2. | "This is the real pipeline on a seeded 3×3 site: the same scenario as our Isaac scene. The terminal says which backends are live: real acoustic models, and Gemini or the rule-based fallback." |
| 0:35 | 0–31 | Point at the drones and pile A2 | "The operator picks the start pile, A2. The drones fly there, map it with LiDAR and score every 1 m cell for slope and roughness, so they can land somewhere flat." |
| 0:45 | 32–95 | Point at the range rings and the belief | "Landed, rotors off, each radar looks for a breathing-like signal. One range is just a ring. The first scans disagree, so the planner moves one drone to where it expects the most information." |
| 1:00 | 95 | **A2-R1** appears (red) | "Three independent ranges agree, so this is a ping: a candidate search region, not a confirmed person. Depth is assumed, not measured, and the dashboard says so." |
| 1:05 | 95–121 | Side panel: **Acoustic agent** | "While landed, the drones listen to the other piles. Real models: voice activity, Whisper, YAMNet, direction finding. A3: 'Help, can anyone hear me, my leg is trapped.' Priority 82. The dog and the debris are dismissed as non-human." |
| 1:10 | 121 | Map: the dispatch arrow to A3 | "The fleet goes to the top priority, A3." |
| 1:20 | 180 | **A3-R1** pinged | "Second ping, at the pile the audio pointed to." |
| 1:30 | 206 | Pile priority list | "The next listen makes B2 the top priority, with a voice. Dispatch to B2." |
| 2:00 | 336 | **B2-R1** pinged | "Third ping." |
| 2:05 | 362+ | Open **Dispatch decisions** (first decision). Don't point at C3. | "Each dispatch decision lists every option with its priority, travel time and battery, and who chose it: Gemini, or argmax when there's no key." |
| 2:40 | ~450 | Click **A2-R1** in the candidates list. Drag to orbit, then Close. | "The ping sits inside the LiDAR model of the pile, showing where on the pile. Depth is assumed at about 1.5 m; a radar depth estimate is coming." Only call it a radar estimate if the detail panel says "radar estimate". |
| 3:10 | ~600 | Map key ("? How to read this map") | "Silent piles are marked 'not cleared', not 'empty'. No detection is not proof that nobody is there." |
| 3:45 | 745 | Terminal: result and handover brief | "The rescuers get a handover brief built only from facts in the state: three pings and five silent piles not cleared. An LLM may rewrite it, but its version is kept only if it passes checks." |
| 4:15 | – | Q&A buffer | Honest limits: radar is a labelled synthetic generator (the DSP chain was validated on real recordings), acoustic scenes are simulated, flight is kinematic, depth is nominal for now, and the B1/C3 acoustic attribution bug below. Isaac runs the same scenario. |

Checked on seed 7 (`scripts/demo.sh`, ×6, 2026-11-07): visits A2 → A3 → B2 → C3 → C3, pings A2-R1, A3-R1, B2-R1, 5 argmax decisions, acoustic real-cached, 24/24 radar scans, 745 s of mission time, 134 s of wall time.

**Known issue (open bug BL-3, being fixed). Don't present C3 as a lead.** In the seed-7 run the drones **miss the faint-voice victim at B1**. That voice is attributed to C3, which is empty, so the fleet spends its remaining scans at C3 ("insufficient evidence") and B1 ends as "not cleared". If asked, say it plainly: "Our acoustic attribution currently assigns a faint voice to the wrong pile; that's the bug we're fixing. That's also why silent piles are never marked cleared." The template brief lists B1 under silent piles not cleared, which is correct.

## Fallbacks

| Problem | Do this | Say |
|---|---|---|
| Real acoustic models fail (missing `.venv-audio`, crash, slow) | `scripts/demo.sh --stub`. demo.sh also switches to the stub automatically, with a loud warning, when `.venv-audio` is missing. | "This run uses the labelled synthetic acoustic stub; the real models are cached from the rehearsal." |
| Acoustic step slow on the live laptop | Make sure the rehearsal ran (step 3). The cache is keyed on the scenario and drone positions, so the identical seed-7 run hits it. | – |
| No Gemini key, Gemini errors, or no network | Nothing to do: it falls back to argmax and the template brief automatically. To force this, run `scripts/demo.sh --no-gemini`. With a key, rehearse once online; `LLM_REPLAY=1 scripts/demo.sh` then replays the cached Gemini calls with no network (docs/AGENTS.md). | "Every decision shows who made it; here it's the deterministic rule." |
| Running long | Start it with `scripts/demo.sh --speed 6` (about 2:15). Or skip the C3 click-through: the three pings are in by sim t ≈ 340. | – |
| Dashboard shows "demo data" or "connection lost" | The server is down: restart it with the command in T-30 step 1, then reload. | – |
| Mission crashes, or the laptop is too slow | **Replay mode**, which needs no Mission: `.venv/bin/python scripts/record_dashboard.py --live --speed 6 --frames-dir dashboard/data/backup/frames --src-dir dashboard/data/backup`, then open http://localhost:8765/dashboard/?data=data/replay&poll=250 | "This is the recorded run from earlier, replayed through the same dashboard." |
| 3D view blank (three.js loads from a CDN and the room is offline) | The facts panel next to it still works. Show the end of the recorded video for the 3D view. | – |
| Nothing works | Play `media/larpverse-dashboard-seed7.mp4` (about 2 min) and narrate over it using the same beats. | – |

## Recording the dashboard video

```bash
scripts/demo.sh --fast        # or any finished run with --frames
scripts/heavy.sh --browser <rec-venv>/bin/python scripts/record_dashboard.py --out media/larpverse-dashboard-seed7.mp4
```

The recorder copies each frame into `dashboard/data/replay/` and loads `dashboard/?data=data/replay&poll=100`, so the live `state.json` is never touched. Each video second shows 3 mission frames, every 2nd simulated second, which is about 6× mission speed. At the end it holds the final state for 2 s, then clicks the first ping and records 5 s of its LiDAR detail view. You need playwright with system Chrome and imageio-ffmpeg in `<rec-venv>`.

Approximate timecodes in the dashboard video (video time = sim t / 6): A2-R1 at 0:16, dispatch to A3 at 0:20, A3-R1 at 0:30, dispatch to B2 at 0:34, B2-R1 at 0:56, C3 phase (the BL-3 misattribution, don't use it) from 1:00, mission complete at about 2:04, ping detail at 2:06–2:11.

## 1-minute video edit plan

Sources:
- `media/larpverse-grid-seed7.mp4` (concept animation, seed 7, 88 s, 1080p)
- Isaac footage from the teammate (Isaac Sim 6.0.1 on GPU, same seed-7 scenario; see docs/FROM_ISAAC.md). One clip per camera: `director`, `wide`, `chase_d1` (chase:d1), `fpv_d2` (fpv:d2), `top`. The full recording is 1920x1080 at 30 fps with `--rec-speed 6 --rec-stop-t 250`, so clip time ≈ sim t / 6 and it covers sim 0–250: A2 survey, A2-R1, dispatch to A3, A3-R1, and the start of the dispatch to B2. Clips land in `media/isaac/`.
- `media/larpverse-dashboard-seed7.mp4` (real pipeline, about 2:11)

| Out | Source and in–out | Shows | Voice-over / caption |
|---|---|---|---|
| 0:00–0:06 | Isaac `wide`, clip 0:00–0:06 (sim 0–35): the field, launch from staging | The site | "A collapsed building. Nine rubble piles. Where do rescuers dig first?" |
| 0:06–0:14 | Isaac `chase_d1` clip 0:01–0:05 (flight to A2, LiDAR survey), then `fpv_d2` clip 0:05–0:09 (landing approach, sim 32–55). If Isaac isn't ready, use concept 0:00–0:14. | LiDAR mapping, flat cells | "Three drones map each pile with LiDAR and land on flat ground." |
| 0:14–0:22 | Concept 0:19–0:27. Crop or cover the caption box from 0:24.4: that caption says "radar estimate of its depth", which overclaims, since depth is nominal. | Radar ranges cross, ping, sent to the dashboard | "Rotors off, radar looks for breathing under the rubble. Three ranges agree: a candidate search region." |
| 0:22–0:32 | Concept 0:28–0:42 at about 1.4× | Listening, transcripts, priority list | "Then they listen. Speech, knocks, dogs: an agent ranks the other piles by how human and how urgent." |
| 0:32–0:42 | Dashboard 0:14–0:36 at about 2.2× | Real pipeline: A2-R1, acoustic panel, dispatch to A3, A3-R1 | "This is the real pipeline on the same seed. Every ping and dispatch decision shows its evidence." |
| 0:42–0:47 | Isaac `director` clip 0:20–0:23 (dispatch to A3, sim 121–140), or `top` clip 0:34–0:42 (dispatch to B2, sim 206–250) | Autonomy | "The fleet moves to the next priority on its own." |
| 0:47–0:55 | Dashboard 2:06–2:11 (the ping detail), slowed to fill | Ping inside the LiDAR model | "The operator opens the ping inside the 3D map to see where on the pile. Depth is assumed at about 1.5 m for now." Say "radar depth estimate" only if the run's ping shows depth_source "radar_estimate". |
| 0:55–1:00 | Dashboard frame at about 0:58 (just after B2-R1, before the C3 phase) frozen, with a title card. Not the final state: there C3 shows as a lead (BL-3). | 3 pings; silent piles not cleared | "Candidates, not confirmations. Larpverse: search smarter, dig sooner." |

Edit notes:
- Keep the honesty captions visible ("candidate, not confirmed"; "synthetic radar, labelled").
- Burn in a small "simulated" tag on the Isaac and concept shots. Isaac known looks (FROM_ISAAC.md): LiDAR hit speckles in the `director`/`wide` shots, and a haze band in the upper left of `fpv_d2`. Prefer `chase_d1` and `top` if these are distracting.
- Don't show C3 or B1 as results: the B1 voice is misattributed to C3 (open bug BL-3). The dashboard cuts above stop before the C3 phase.
- Credit the real models: Silero VAD, Whisper, YAMNet, KISS-ICP.
