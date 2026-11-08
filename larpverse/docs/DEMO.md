# Live demo runbook (5 minutes)

Updated 8 Nov 2026 to the current system: drones land on the rubble with a belly-mounted radar, `literature` radar, condition-only vitals, rotor veto, speaker call-out, rescue Wi-Fi, smooth flight, Gemini on the free tier.

> **The judged deliverable is the 60 s single-take simulation video.** Its plan, commands and checklist are in [docs/VIDEO_PLAN.md](VIDEO_PLAN.md) and [docs/ISAAC_BRIEF.md](ISAAC_BRIEF.md). The multi-cut edit plan that used to live here is **retired** (PLAN.md decision, 7 Nov). This file covers only the live laptop demo and Q&A backup.

## What the audience should see

The seed-7 grid site (synthetic layout) runs autonomously on the dashboard:
- drones survey A2, land on flat spots on it, and scan down with their belly radars;
- **A2 pings**: a silent victim found by radar only, with a breathing condition;
- the drones land off-pile, listen and call out;
- the agent ranks the other piles and the fleet moves on.

After A2, further pings vary by how the radars seat on the rubble «SEED7-CANON». In one review run (PASS3 §0), B1 and B2 also pinged and A3 stayed an open lead. Do not promise more than A2 on stage.

## Known issues to be honest about

- **A3 can stay open.** Its victim breathes rapidly and shallowly (33/min, about 2.2 mm), and the landed drones can sit unevenly on its slabs. If it ends "insufficient evidence", say: "That's the critical failure mode, and the system says so: A3 is searched, not cleared, and it's in the brief as an open lead."
- **Depth.** It is shown with its source: "radar estimate" when three scans constrain it, otherwise "≈1.5 m, nominal (assumed)". Read out what the panel says.
- **Condition wording.** "Slow, regular breathing" means "deep rest **or** reduced consciousness; check responsiveness". Healthy relaxed volunteers read this way too. It is never a diagnosis, and there is no age.
- **Gemini.** It is free tier, about 20 requests/day per model. With a key, `demo.sh` uses `gemini-3.7-flash` for dispatch and `gemini-3.5-flash-lite` for audio. A 429 falls back, and the badge says "fallback". Read the badge out as it is.
- **Synthetic parts.** Say each one plainly: radar returns, victim audio, phones, and the layout.

## T-30 min: set up and rehearse

1. **Server.** `scripts/serve.py` (static files plus `/api/tasking`, `/api/portal`, `/api/broadcast`), not `http.server`, which can't save zones:
   ```bash
   .venv/bin/python scripts/serve.py --port 8765
   curl -sI http://localhost:8765/dashboard/ | head -1     # expect 200
   ```
   `demo.sh` still prints the old `http.server` hint when the server is down (Requests). Use `serve.py`.
2. **Gemini (optional).** Put the key in `.env` and never show it on screen. Rehearse once online, then go live with `LLM_REPLAY=1` so cached answers replay without spending quota.
3. **Zones.** `demo.sh` passes no `--tasking`. Without `dashboard/data/tasking.json` the mission starts in **recon** mode (a mapping flight and perch-and-listen first), and the A2 story starts later. For the scripted story, write the default zones first: `python3 isaac/live.py --write-tasking default --out dashboard/data` (as in VIDEO_PLAN.md).
4. **Rehearsal run** (also warms the acoustic cache): `scripts/demo.sh --fast --direct`.
5. **Timing.** With `--delivery pod` (radar on the rubble surface), the mission is long: about 3100 sim-s in the review run, so ×4 is about 13 min of wall time.
   - For a 5-minute slot, use `scripts/demo.sh --direct --speed 20`, or cap it with `--piles 2`.
   - **Measure the wall time at the rehearsal**, and write it here.
6. **Decide live or replay.**
   - **Live:** if the rehearsal printed an A2 ping and finished in your slot.
   - **Replay:** otherwise. Use the rehearsal frames at `http://localhost:8765/dashboard/?state=data/replay/state.json&poll=250`, and say "our recorded run from earlier today".
7. **Tabs.**
   1. The dashboard: `http://localhost:8765/dashboard/`, or `?layout=take` for the compact video layout.
   2. The rescue portal on a phone or narrow window: `/dashboard/portal/`.
   3. Backup: the latest take or dashboard recording, if one exists.

## Run of show (live)

| Wall | Do | Say |
|---|---|---|
| 0:00 | Dashboard, mission not started | "After a collapse, people are trapped where cameras can't see. At Weiguan in 2016, 115 died; nobody came out alive after 62 hours. Larpverse sends heavy drones that put a breathing radar on the rubble, so nobody has to stand on it." |
| 0:20 | Press Enter on `scripts/demo.sh --direct --speed N` | "Real pipeline, synthetic site. The terminal says which backends are live: real acoustic models, and Gemini or the fallback." |
| 0:30 | Drones over A2, LiDAR map | "They map the pile with LiDAR, then each lands on a flat spot on the rubble, and the 5 kg radar on its underside scans down through it." |
| 0:50 | Drones landed, rings appear | "A 45-second scan looks for breathing. One range is just a ring, so the planner sends a drone to land where it learns the most." |
| 1:30 | **A2 ping** + condition chip | "Three ranges agree: a candidate search region, not a confirmed person. The condition reads the breathing: it's an aid for the dig order, not a diagnosis. This person is silent, so only the radar found them." |
| 2:00 | Drones land off-pile; acoustic and speaker panels | "Rotors off, they listen to the other piles and call out in Mandarin, Hokkien and English. A rotor veto stops our own drones from being heard as people." |
| 2:30 | Dispatch decisions card | "Every option, with priority, travel and battery, and who decided: Gemini, or the fallback when the free tier runs out." |
| 3:00 | Portal on the phone | "Phones can choose to join a rescue Wi-Fi: 'I'm trapped' raises that pile to a lead. No sniffing, no interception." |
| 3:30 | Pile list, dig order | "Every pile without a ping says 'searched, not cleared'. No detection is not proof of absence." |
| 4:00 | Ping detail (3D) | "The ping inside the LiDAR model, with its depth source." |
| 4:30 | Brief in the terminal | "The handover brief: pings with conditions, open leads, and what's not cleared." |

If asked about results: "In 20 simulated seeds, acoustic-led dispatch pinged 17 of 60 non-start victims, against 6 and 5 for radar-only baselines. The acoustic clips were generated from ground truth, so that's an upper bound, and it isn't faster to the first ping." (docs/EVAL_GRID.md)

## Fallbacks

- **No `.venv-audio`:** `demo.sh` uses the stub and says so in red. Say "synthetic acoustic stub" on stage.
- **No key or a 429:** the decider uses argmax or fallback, and the brief uses the template. Both are labelled.
- **Mission too slow:** switch to the replay URL above.
- **Browser can't reach the tiles:** the basemap falls back to LiDAR relief offline.
