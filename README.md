# Larpverse: drones that search collapsed-building rubble for trapped people

**AgentVerse 2026, Robotics track.** Team on GitHub: rohansharmacode, fannivvv, phupha-yao, manocw.

After a building collapses, rescuers have to decide which rubble pile to dig first, while the people trapped inside are often silent, weak or out of sight. In the Weiguan Jinlong collapse (Tainan, 6 February 2016), 309 people were in the building: 175 were rescued alive and 115 died.

Larpverse gives the incident commander a live operator console. You click one rubble pile. From then on, three drones work on their own:

1. **Map.** They fly to the pile and map it with LiDAR.
2. **Land safely.** Each drone lands only where the LiDAR map shows the ground is flat and clear enough. Steep or cluttered spots are rejected.
3. **Feel for breathing.** A radar on the underside of each landed drone looks for the tiny chest movement of breathing, up to about 3 m down.
4. **Listen.** The rotors stop and the drones listen for voices and knocking. Gemini understands speech in any language and picks out cries for help, which go to the top of the priority list.
5. **Choose the next pile.** Gemini in the police van weighs everything heard and sensed and sends the drones to the next pile. Nobody tells it where to go.

Everything runs in NVIDIA Isaac Sim. The trapped people, their voices and the radar returns are simulated, and the console says so on screen.

## Start here

| What | Where |
|---|---|
| 1-minute demo video (screen recording of the live console) | [larpverse_console_taiwan_v10_final_noend.mp4](https://github.com/rohansharmacode/skysearch/blob/larpverse-rehearsals/final/larpverse_console_taiwan_v10_final_noend.mp4) |
| Live console (runs on our GPU server while it is switched on; switching between the three sites or pressing Reset takes about 3 s, because a preloaded Isaac Sim waits for each site) | https://descending-coleman-continually-cocktail.trycloudflare.com/ |
| The code | branch [`larpverse-isaac`](https://github.com/rohansharmacode/skysearch/tree/larpverse-isaac/larpverse), folder `larpverse/` |
| Voiceover script for the video | [final/voiceover_script_v10.md](https://github.com/rohansharmacode/skysearch/blob/larpverse-rehearsals/final/voiceover_script_v10.md) |
| Detailed README (modules, quickstart, honest limits, credits) | [larpverse/README.md](https://github.com/rohansharmacode/skysearch/blob/larpverse-isaac/larpverse/README.md) |
| Technical report | [larpverse/docs/TECHNICAL_REPORT.md](https://github.com/rohansharmacode/skysearch/blob/larpverse-isaac/larpverse/docs/TECHNICAL_REPORT.md) |

In the video, the operator clicks once at the start. After that nothing is scripted. Slow stretches are fast-forwarded (3×, 8×, 16×) and the two landings are slowed to ½×, and each speed change is labelled on screen. The live Isaac view renders at about 3 to 15 frames per second on one NVIDIA L40S, so motion can look choppy. That only affects the picture: the decisions come from the mission code, not the rendered frames.

## Models (exact IDs)

| Where | Model | Job |
|---|---|---|
| Police van (command) | `gemini-4-argon` | Picks the next pile from the options the mission lists, with a one-line reason |
| Each drone (audio) | `gemini-3.5-flash` (the demo video and the live console use `gemini-3.5-flash-lite` for lower latency) | For each sound clip: is it a person, what is said (any language), distress cues, and is it under rubble or out in the open |
| On-drone, offline option | Gemma 4 E2B (`google/gemma-4-E2B-it`) | The same audio step with no network link (open weights) |
| On the box, always | Silero VAD, YAMNet, faster-whisper, SRP-PHAT | Detection, transcription and direction finding with no LLM |

## How model output is checked

No model output acts directly.
- **Fixed choices only.** The van's Gemini can only choose from the piles the mission lists. Code checks the choice, the pile budget and a 25 % battery reserve. A missing or invalid answer falls back to a fixed rule, labelled `fallback` on screen.
- **Schema-checked answers.** Audio answers must match a schema.
- **Distress words verified.** A second, independent transcription must reproduce any distress words, or they are demoted.
- **Sound priority.** Every sound is ranked: trapped person > check > safe person on the surface > not a person. Only the top two can pull the drones.
- **Who decided is always shown.** Every decision and every sound on screen says who made it: you, Gemini 4 Argon in the van, Gemini on the drones, or the on-box models.
- **Pings are leads.** A radar ping is a candidate search region, never a confirmed person, and no pile is ever marked "cleared".

## What is real and what is simulated

- **Simulated:** the people, their voices and the radar returns, generated as labelled synthetic data.
- **Real models on that data:** the breathing-detection signal processing, the speech and sound models, LiDAR mapping and the planning.
- **Validation:** the breathing processing was checked on real radar recordings (no rubble). The sound recognisers were checked on real recordings with the rotors off.
- **Sites:** a synthetic 3×3 rubble grid, a procedural collapsed tower, and a Tainan collapsed-building model. The Tainan model's geometry is derived from NCKU's scan of the Weiguan Jinlong site, with scan textures off.
- **Hardware:** the drones and the belly radar are a design study. Prices are estimates.

Full limits: [larpverse/README.md#honest-limits](https://github.com/rohansharmacode/skysearch/blob/larpverse-isaac/larpverse/README.md#honest-limits).

## Run it yourself (CPU, no GPU)

```bash
git clone -b larpverse-isaac https://github.com/rohansharmacode/skysearch && cd skysearch/larpverse
uv venv .venv && uv pip install --python .venv numpy scipy matplotlib pytest kiss-icp open3d
.venv/bin/python -m sim.grid_mission --seed 7 --out dashboard/data
.venv/bin/python scripts/serve.py --port 8765        # then open http://localhost:8765/dashboard/
```

With no Gemini key, every decision uses the rule-based fallback and is labelled that way. To use Gemini, put `GEMINI_API_KEY=...` in `larpverse/.env`; that file is gitignored. The Isaac Sim live console needs a GPU: see [larpverse/isaac/README.md](https://github.com/rohansharmacode/skysearch/blob/larpverse-isaac/larpverse/isaac/README.md).

## Branches

| Branch | What it holds |
|---|---|
| `larpverse-isaac` | **The submission.** The mission, sensing, agents, Isaac Sim scenes and the live console. |
| `larpverse-sensing` | The team's working branch for sensing, agents and documents. Its latest one-page summary and slide assets are only here. |
| `3d_model` | The Tainan collapsed-building 3D model. |
| `larpverse-rehearsals` | Recorded takes and the final videos. |
| `main`, `v2-earthquake` | Earlier SkySearch prototypes from the first day of the hackathon (forest search, then an office-building earthquake). Kept for history; their old READMEs are in `docs/`. |
