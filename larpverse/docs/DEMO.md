# Live demo runbook (5 minutes) and 1-minute video plan

One command runs the demo: `scripts/demo.sh`. It runs the seed-7 autonomous grid mission at 4× speed and writes `dashboard/data/state.json`, which the dashboard polls every second. At the end it prints the result and the rescuer handover brief.

- **Speed.** Change it with `--speed N`.
- **Pile cap.** Until the acoustic attribution fix (BL-3) lands, it runs with a cap of 3 pile visits (`--piles 3`), so the run ends after the third pile instead of spending its second half on empty C3. `--all-piles` removes the cap.
- **On stage, use `--direct`** (or `DEMO_STAGE=1`). It runs the mission directly, so a queued job on the shared `heavy.sh` queue can't stall it with `[heavy] waiting`.

Everything below assumes the repo root (`/home/peroxide/Downloads/UCL/agentverse`) and the dev server on :8765.

## Known issues to be honest about

- **BL-3: the faint-voice victim at B1 is missed** (open bug, being fixed). In seed 7, that voice is attributed to C3, which is empty. So C3 ranks as a lead and gets scanned to "insufficient evidence", while B1 ends "not cleared".
  - **Don't present C3 as a real lead.**
  - If asked, say: "Our acoustic attribution currently assigns a faint voice to the wrong pile; that's the bug we're fixing. That's also why silent piles are never marked cleared."
- **Depth is nominal.** Pings are drawn about 1.5 m below the LiDAR surface ("depth (nominal)", "assumed, not measured"). A radar depth estimate is being added. Say "radar depth estimate" only if the ping's detail panel says "radar estimate" (state `depth_source`).
- **The tree is changing during the event** (BL-3, acoustic cache versioning, Gemini). Two runs on 2026-11-07 gave different stories:
  - **19:20, known good** (cached acoustic, argmax, ×6, full run): A2 → A3 → B2 → C3 → C3. Pings A2-R1 (t 95), A3-R1 (t 180) and B2-R1 (t 336). From the second listen (t 206) on, the side list shows C3 as "acoustic lead 53". That's the BL-3 misattribution, so don't dwell on it. 5 argmax decisions, 24/24 scans, 745 s of mission time, 134 s wall. These frames are saved in `dashboard/data/backup-seed7/`.
  - **19:55, after the acoustic cache stopped matching, with Gemini returning HTTP 429** (`--fast --piles 3`): A2 → A3 → C3. Pings A2-R1 (t 126) and A3-R1 (t 383), then C3 "insufficient evidence". 3 decisions (argmax, and fallback after the 429s), 680 s of mission time.
  - **So the rehearsal decides the plan** (next section).

## T-30 min: set up, rehearse, choose live or replay

1. **Dev server.** `curl -sI http://localhost:8765/dashboard/ | head -1` should print `200`. If it doesn't, start the server in its own terminal and leave it running:
   `python3 -m http.server 8765 --bind 127.0.0.1 --directory /home/peroxide/Downloads/UCL/agentverse`
2. **Gemini (optional).** Put `GEMINI_API_KEY=…` in `.env`, and never show `.env` on screen.
   - Without a key, the dispatch gate uses argmax and the brief uses the template. The dashboard labels both honestly.
   - With a key, rehearse online, then go live with `LLM_REPLAY=1` to replay the cached calls without hitting rate limits (docs/AGENTS.md). A 429 falls back to argmax and is labelled "fallback".
3. **Rehearsal run, which also warms the acoustic cache:** `scripts/demo.sh --fast --direct`. It takes about 1.5–2 min.
4. **Decide.** Read the `== Result ==` block:
   - **Live plan:** `piles visited : A2 -> A3 -> B2` with 3 pings. Run the live mission on stage.
   - **Replay plan:** anything else, for example `A2 -> A3 -> C3`. Present the known-good run through the same dashboard with replay mode (below), and say it's "our recorded run from earlier today".
5. **Browser tabs, in this order:**
   1. Concept animation: http://localhost:8765/animation/grid-search.html (no `#s…` hash: the plain URL plays the seed-7 scenario)
   2. Dashboard. Live plan: http://localhost:8765/dashboard/. Replay plan: http://localhost:8765/dashboard/?state=data/replay/state.json&poll=250
   3. `media/larpverse-dashboard-seed7.mp4`, the last-resort video (69 s, ends on B2-R1 and then the A3-R1 LiDAR detail).
6. **Terminal.** Use a large font with the repo root as the working directory. Type one of these but don't press Enter yet:
   - Live plan: `scripts/demo.sh --direct`
   - Replay plan: `.venv/bin/python scripts/record_dashboard.py --live --speed 4 --until-t 355 --frames-dir dashboard/data/backup-seed7/frames --src-dir dashboard/data/backup-seed7`

## Run of show (live plan, ×4, `--piles 3`)

"Sim t" is the mission clock, shown top right on the dashboard as `t = …`. At ×4 it advances 4 s per wall second, and warm caches add only about 10 s of compute. The replay plan follows the same beats and stops at sim t 355, just after B2-R1. The brief in the replay plan comes from the backup run: `.venv/bin/python -m agents.brief --state dashboard/data/replay/state.json --provider none`.

| Wall | Sim t | Do | Say |
|---|---|---|---|
| 0:00 | – | Tab 1, concept animation, playing | "After a building collapse, rescuers face a field of rubble piles and little time. Larpverse sends three drones to find where to dig first." |
| 0:30 | – | Press Enter. Switch to tab 2. | "This is the real pipeline on a seeded 3×3 site: the same scenario as our Isaac scene. The terminal says which backends are live: real acoustic models, and Gemini or the rule-based fallback." |
| 0:35 | 0–31 | Point at the drones and pile A2 | "The operator picks the start pile, A2. The drones fly there, map it with LiDAR and score every 1 m cell for slope and roughness, so they can land somewhere flat." |
| 0:45 | 32–95 | Point at the range rings and the belief | "Landed, rotors off, each radar looks for a breathing-like signal. One range is just a ring. The first scans disagree, so the planner moves one drone to where it expects the most information." |
| 1:00 | 95 | **A2-R1** appears (red) | "Three independent ranges agree, so this is a ping: a candidate search region, not a confirmed person. Depth is assumed at about 1.5 m, and the dashboard says so." |
| 1:05 | 95–121 | Side panel: **Acoustic agent** | "While landed, the drones listen to the other piles. Real models: voice activity, Whisper, YAMNet, direction finding. A3: 'Help, can anyone hear me, my leg is trapped.' The top priority. The dog and the debris are dismissed as non-human." |
| 1:10 | 121 | Map: the dispatch arrow to A3 | "The fleet goes to the top priority, A3." |
| 1:20 | 180 | **A3-R1** pinged | "Second ping, at the pile the audio pointed to." |
| 1:30 | 206 | Pile priority list | "The next listen makes B2 the top priority. Dispatch to B2." |
| 2:00 | 336 | **B2-R1** pinged | "Third ping." |
| 2:10 | ~360–400 | "Search loop ends: pile budget used"; drones return | "We capped this run at three piles. The rest are listed for the ground teams, and none of them is cleared." Don't single out C3 (BL-3). |
| 2:20 | – | Open **Dispatch decisions** (first decision) | "Each decision lists every option with its priority, travel time and battery, and who chose it: Gemini, or argmax and fallback when it wasn't available." |
| 2:45 | – | Click the top ping in the **Dig order** list (A3-R1 in the known-good run, which also shows what was heard there). Drag to orbit, then Close. | "The ping sits inside the LiDAR model of the pile, showing where on the pile. Depth is assumed at about 1.5 m; a radar depth estimate is coming." |
| 3:15 | – | Map key ("? How to read this map") | "Silent piles are marked 'not cleared', not 'empty'. No detection is not proof that nobody is there." |
| 3:40 | – | Terminal: the handover brief | "The rescuers get a handover brief built only from facts in the state: where to dig, unresolved leads, and silent piles not cleared. An LLM may rewrite it, but its version is kept only if it passes checks." |
| 4:10 | – | Q&A buffer | Honest limits: radar is a labelled synthetic generator (the DSP chain was validated on real recordings), acoustic scenes are simulated, flight is kinematic, depth is nominal, and BL-3 above. Isaac runs the same scenario. |

## Fallbacks

| Problem | Do this | Say |
|---|---|---|
| The rehearsal doesn't give A2 → A3 → B2 | Use the replay plan (T-30, step 4). | "This is our recorded run from earlier today, through the same dashboard." |
| Real acoustic models fail (missing `.venv-audio`, crash) | `scripts/demo.sh --direct --stub`. demo.sh also switches to the stub automatically, with a loud warning, when `.venv-audio` is missing. | "This run uses the labelled synthetic acoustic stub." |
| Acoustic step slow (cache miss: the acoustic code changed since the rehearsal) | Use the replay plan. | – |
| No Gemini key, Gemini 429s, or no network | Nothing to do: it falls back to argmax and the template brief automatically. Force it with `--no-gemini`, or use `LLM_REPLAY=1` after an online rehearsal. | "Every decision shows who made it; here it's the deterministic rule." |
| `[heavy] waiting` on stage | Ctrl-C, then `scripts/demo.sh --direct`. | – |
| Running long | `scripts/demo.sh --direct --speed 6`, or skip the decisions click-through. | – |
| Dashboard shows "demo data" or "connection lost" | The server is down: restart it with the command in T-30 step 1, then reload. | – |
| Mission crashes mid-run | Switch to the replay plan command, and the replay dashboard URL. | – |
| 3D view blank (three.js doesn't load offline) | The facts panel next to it still works. The video ends with the 3D view. | – |
| Nothing works | Play `media/larpverse-dashboard-seed7.mp4` and narrate over it using the same beats. | – |

## Recording the dashboard video

```bash
scripts/heavy.sh --browser <rec-venv>/bin/python scripts/record_dashboard.py \
  --frames-dir dashboard/data/backup-seed7/frames --src-dir dashboard/data/backup-seed7 \
  --until-t 355 --hold 3 --detail 6 --out media/larpverse-dashboard-seed7.mp4
```

- **How it works.** The recorder copies each frame into `dashboard/data/replay/` and loads `dashboard/?state=data/replay/state.json&poll=100`, so the live `state.json` is never touched. It checks that the dashboard shows each frame's `t` and aborts if the dashboard isn't following the replay.
- **Pacing.** It shows every 2nd simulated second, 3 frames per video second, so video time ≈ sim t / 6. It holds the last state, then clicks the first ping and records its LiDAR detail view.
- **Requirements.** playwright with system Chrome, and imageio-ffmpeg, in `<rec-venv>`.

Current `media/larpverse-dashboard-seed7.mp4`, from the 19:20 known-good run, cut at sim t 355:

| Video time | Shows |
|---|---|
| ≈ 0:16 | A2-R1 |
| ≈ 0:20 | dispatch to A3 |
| ≈ 0:30 | A3-R1 |
| ≈ 0:34 | dispatch to B2 |
| ≈ 0:56 | B2-R1 |
| ≈ 0:59–1:02 | hold on the final state (sim t 355) |
| ≈ 1:02–1:09 | A3-R1 LiDAR detail, with "Heard at this pile" (the first ping in the Dig order list) |

## 1-minute video edit plan

**Sources:**
- `media/larpverse-grid-seed7.mp4`: concept animation, seed 7, 88 s, 1080p.
- `media/larpverse-dashboard-seed7.mp4`: the real pipeline, 69 s, timecodes above.
- **Isaac footage**, from the teammate: Isaac Sim 6.0.1 on GPU, same seed 7 (docs/FROM_ISAAC.md). There's one clip per camera: `director`, `wide`, `chase_d1` (chase:d1), `fpv_d2` (fpv:d2) and `top`. Clip time = timeline frame / 30.

**Isaac gate (R2-005).** Cut an Isaac shot that shows a ping or a dispatch only if that run's `timeline.csv` shows it. The current `media/isaac/lv2/` run (17.8 s, stops at sim t 250) has **no firm ping and no dispatch**: A2 relocates and rescans until t 234 and ends "insufficient evidence" (R2-002). So from lv2, use only:

| lv2 clip time | Sim t | Phase |
|---|---|---|
| 0:00–0:05.2 | 1–32 | deploy and survey |
| 0:05.2–0:07.2 | 32–44 | landing |
| 0:07.2–0:08.0 | 44–64 | radar starts |

For a re-record (after R2-002, with `--rec-stop-t 380`), check the gate before cutting:

```bash
awk -F, 'NR>1 && ($3!=p || $4!=q) {printf "%.1fs t=%s %s %s pings=%s disp=%s\n",$1/30,$2,$3,$4,$5,$7; p=$3; q=$4}' media/isaac/<run>/timeline.csv
```

Use the ping and dispatch shots only if `disp` goes ≥ 1 after A2 and the run log has `PING A2-R1`. Take the timecodes for rows 5 and 7 of the plan from that output.

| Out | Source and in–out | Shows | Voice-over / caption |
|---|---|---|---|
| 0:00–0:06 | Isaac `wide` lv2 0:00–0:05 (deploy, survey), stretched to fill | The site | "A collapsed building. Nine rubble piles. Where do rescuers dig first?" |
| 0:06–0:14 | Isaac `chase_d1` lv2 0:01–0:05 (LiDAR survey of A2), then `top` lv2 0:05.2–0:07.2 (landing). If these read badly, use concept 0:06–0:14. | LiDAR mapping, flat cells | "Three drones map each pile with LiDAR and land on flat ground." |
| 0:14–0:22 | Concept 0:19–0:27. Crop or cover the caption box from 0:24.4: it says "radar estimate of its depth", but depth is nominal. | Radar ranges cross, ping, sent to the dashboard | "Rotors off, radar looks for breathing under the rubble. Three ranges agree: a candidate search region." |
| 0:22–0:32 | Concept 0:28–0:42 at about 1.4× | Listening, transcripts, priority list | "Then they listen. Speech, knocks, dogs: an agent ranks the other piles by how human and how urgent." |
| 0:32–0:42 | Dashboard 0:14–0:36 at about 2.2× | Real pipeline: A2-R1, acoustic panel, dispatch to A3, A3-R1 | "This is the real pipeline on the same seed. Every ping and dispatch decision shows its evidence." |
| 0:42–0:47 | Dashboard 0:34–0:56 at about 4× (fleet to B2, B2-R1). Swap in an Isaac dispatch shot only if a re-record passes the gate. | Autonomy | "The fleet moves to the next priority on its own." |
| 0:47–0:55 | Dashboard 1:02–1:09 (A3-R1 detail), slowed to fill | Ping inside the LiDAR model | "The operator opens the ping inside the 3D map to see where on the pile. Depth is assumed at about 1.5 m for now." |
| 0:55–1:00 | Dashboard ≈ 0:59 (final state, sim t 355) frozen, with a title card | 3 candidate regions; silent piles not cleared | "Candidates, not confirmations. Larpverse: search smarter, dig sooner." |

**Edit notes:**
- Keep the honesty captions visible ("candidate, not confirmed"; "synthetic radar, labelled").
- Burn in a small "simulated" tag on the Isaac and concept shots.
- Known Isaac looks (FROM_ISAAC.md, review pass 2): LiDAR speckle in `director`/`wide`/`top`, a haze band in `fpv_d2`, and drones only a few pixels wide in `director`/`wide`. Prefer `chase_d1`.
- Don't show C3 or B1 as results (BL-3). The dashboard video already stops before the C3 phase.
- Credit the real models: Silero VAD, Whisper, YAMNet, KISS-ICP.
