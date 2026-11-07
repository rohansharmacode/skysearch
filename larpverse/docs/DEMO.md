# Live demo runbook (5 minutes) and 1-minute video plan

One command runs the whole demo: `scripts/demo.sh`. It runs the seed-7 autonomous grid mission at 6× speed and writes `dashboard/data/state.json`, which the dashboard polls every second. At the end it prints the result and the rescuer handover brief.

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

The "sim t" column is the mission clock shown top right on the dashboard (`t = …`). At ×6 it advances about 6 s per wall second once the mission starts. Compute adds roughly 15–30 s of wall time at each radar phase.

| Wall | Sim t | Do | Say |
|---|---|---|---|
| 0:00 | – | Tab 1, concept animation, playing | "After a building collapse, rescuers face a field of rubble piles and little time. Larpverse sends three drones to find where to dig first." |
| 0:30 | – | Press Enter on `scripts/demo.sh`. Switch to tab 2. | "This is the real pipeline on a seeded 3×3 site: the same scenario as our Isaac scene. The terminal shows which backends are live: real acoustic models, and Gemini or the rule-based fallback." |
| 0:45 | 0–31 | Point at the drones and pile A2 | "The operator picks the start pile, A2. The drones fly there, map it with LiDAR and score every 1 m cell for slope and roughness, so they can land somewhere flat." |
| 1:05 | 32–95 | Point at the ranges and the belief | "Landed, rotors off, each radar looks for a breathing-like signal. One range is just a ring. Here the first scans disagree, so the planner moves one drone to the spot with the most expected information." |
| 1:20 | 95 | **A2-R1** appears (red) | "Three independent ranges agree, so this is a ping: a candidate search region, not a confirmed person. Depth is assumed, not measured, and the dashboard says so." |
| 1:30 | 95–121 | Side panel: **Acoustic agent**. Pick the listen tab. | "While landed, the drones listen to the other piles. Real models: voice activity, Whisper, YAMNet, direction finding. A3: 'Help, can anyone hear me, my leg is trapped.' Priority 82. The dog and the debris are dismissed as non-human." |
| 1:50 | 121 | **Dispatch decisions** panel: open the first decision | "The dispatch gate weighs every option: priority, travel time, battery. It sends the fleet to A3. Every decision is logged with who made it: Gemini, or argmax when there's no key." |
| 2:05 | 180 | **A3-R1** pinged | "Second ping, at the pile the audio pointed to." |
| 2:15 | 206 | Pile priority list | "Next listen: B2 is now the top priority, with voice and knocking. Dispatch to B2." |
| 2:40 | 336 | **B2-R1** pinged | "Third ping." |
| 2:50 | 362+ | Point at C3 | "C3 had a voice but the radar can't get three agreeing ranges. It stays an open lead, and we don't pretend it's a fix. Silent piles are marked 'not cleared', not 'empty'." |
| 3:20 | 745 | Mission complete. Click **A2-R1** in the candidates list. | "Here is the ping inside the LiDAR model of the pile: where on the pile, and the assumed depth below the surface." Drag to orbit. |
| 3:50 | – | Terminal: the handover brief | "At the end the rescuers get a handover brief built only from the facts in the state: three pings, one open lead, five piles not cleared. An LLM may rewrite it, but the rewrite is kept only if it passes checks." |
| 4:20 | – | Q&A buffer | Honest limits: radar is a labelled synthetic generator (the DSP chain was validated on real recordings), acoustic scenes are simulated, flight is kinematic, Isaac runs the same scenario. |

The demo output was checked on seed 7: A2 → A3 → B2 → C3, pings A2-R1, A3-R1, B2-R1, and C3 left as an open lead.

## Fallbacks

| Problem | Do this | Say |
|---|---|---|
| Real acoustic models fail (missing `.venv-audio`, crash, slow) | `scripts/demo.sh --stub`. demo.sh also switches to the stub automatically, with a loud warning, when `.venv-audio` is missing. | "This run uses the labelled synthetic acoustic stub; the real models are cached from the rehearsal." |
| Acoustic step slow on the live laptop | Make sure the rehearsal ran (step 3). The cache is keyed on the scenario and drone positions, so the identical seed-7 run hits it. | – |
| No Gemini key, or Gemini errors | Nothing to do: it falls back to argmax and the template brief automatically. To force this, run `scripts/demo.sh --no-gemini`. | "Every decision shows who made it; here it's the deterministic rule." |
| Running long | Ctrl-C, then `scripts/demo.sh --speed 12`. Or keep talking over the C3 phase: the important beats are over by sim t ≈ 340. | – |
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

Approximate timecodes in the dashboard video (video time = sim t / 6): A2-R1 at 0:16, dispatch to A3 at 0:20, A3-R1 at 0:30, dispatch to B2 at 0:34, B2-R1 at 0:56, C3 at 1:00 and after, mission complete at about 2:04, ping detail at 2:06–2:11.

## 1-minute video edit plan

Sources:
- `media/larpverse-grid-seed7.mp4` (concept animation, seed 7, 88 s, 1080p)
- Isaac footage from the teammate (same seed-7 scenario)
- `media/larpverse-dashboard-seed7.mp4` (real pipeline, about 2:11)

| Out | Source and in–out | Shows | Voice-over / caption |
|---|---|---|---|
| 0:00–0:06 | Isaac: wide establishing shot of the rubble field with the 3 drones on the staging pad | The site | "A collapsed building. Nine rubble piles. Where do rescuers dig first?" |
| 0:06–0:14 | Isaac: drones launch and orbit A2, LiDAR sweep. If Isaac isn't ready, use concept 0:00–0:14. | LiDAR mapping, flat cells | "Three drones map each pile with LiDAR and land on flat ground." |
| 0:14–0:22 | Concept 0:19–0:27 | Radar ranges cross, ping, sent to the dashboard | "Rotors off, radar looks for breathing under the rubble. Three ranges agree: a candidate search region." |
| 0:22–0:32 | Concept 0:28–0:42 at about 1.4× | Listening, transcripts, priority list | "Then they listen. Speech, knocks, dogs: an agent ranks the other piles by how human and how urgent." |
| 0:32–0:42 | Dashboard 0:14–0:36 at about 2.2× | Real pipeline: A2-R1, acoustic panel, dispatch to A3, A3-R1 | "This is the real pipeline on the same seed. Every ping and dispatch decision shows its evidence." |
| 0:42–0:47 | Isaac: drones flying to the next pile (B2) | Autonomy | "The fleet moves to the next priority on its own." |
| 0:47–0:55 | Dashboard 2:06–2:11 (the ping detail), slowed to fill | Ping inside the LiDAR model | "The operator opens the ping inside the 3D map: where on the pile, and roughly how deep." |
| 0:55–1:00 | Dashboard 2:04 (final state) freeze, with a title card | 3 pings, 1 open lead, silent piles not cleared | "Candidates, not confirmations. Larpverse: search smarter, dig sooner." |

Edit notes:
- Keep the honesty captions visible ("candidate, not confirmed"; "synthetic radar, labelled").
- Burn in a small "simulated" tag on the Isaac and concept shots.
- Credit the real models: Silero VAD, Whisper, YAMNet, KISS-ICP.
