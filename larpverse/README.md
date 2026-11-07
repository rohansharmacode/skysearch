# Larpverse: autonomous drone search of collapsed-building rubble

Built for the AgentVerse 2026 robotics track. Three drones search a 3×3 grid of rubble piles.

1. **LiDAR:** the drones map each pile with a spinning LiDAR, registered with KISS-ICP SLAM. They land on cells the map shows are flat.
2. **Radar:** landed drones scan for a respiration-like signal under the rubble. The three ranges are fused into a **candidate search region** with uncertainty, called a "ping".
3. **Acoustic agent:** with rotors off, the drones listen to the other piles. The pipeline is Silero VAD, Whisper speech-to-text, YAMNet sound classes and SRP-PHAT direction finding. An agent (Gemini, or a rule-based fallback) scores how human and how urgent each sound is, and ranks the piles.
4. **Dispatch:** the coordinator sends the fleet to the next pile, and the cycle repeats. The operator dashboard shows pings, priorities, transcripts and every decision, and opens the LiDAR model of a pile with the ping placed inside it.

Isaac Sim provides the 3D world, the drone bodies and the LiDAR geometry. The same seeded scenario drives Isaac, the numpy simulator and the dashboard. The Isaac adapter has been tested without a GPU only; nothing has run on Isaac yet. See [docs/ISAAC_HANDOFF.md](docs/ISAAC_HANDOFF.md).

## Quickstart (CPU, no GPU needed)

1. Create the environment and install the dependencies:
   ```bash
   uv venv .venv && uv pip install --python .venv numpy scipy matplotlib pytest kiss-icp open3d
   ```
2. Run a full autonomous grid mission (seed 7 is the demo). It writes `dashboard/data/state.json`:
   ```bash
   .venv/bin/python -m sim.grid_mission --seed 7 --radar dsp --out dashboard/data
   ```
3. Serve the repo and open the operator dashboard at http://localhost:8765/dashboard/ (the concept animation is at `/animation/grid-search.html`):
   ```bash
   python3 -m http.server 8765
   ```

Optional extras:
- **Real acoustic models** (about 900 MB; runs in a separate environment): `bash acoustic/setup.sh`, then add `--acoustic real` to the mission command.
- **Gemini as the decision-maker:** put `GEMINI_API_KEY=…` in `.env` (gitignored), then add `--decider gemini`. Everything falls back to rule-based choices without it.
- **Isaac Sim:** see [isaac/README.md](isaac/README.md) and [docs/ISAAC_HANDOFF.md](docs/ISAAC_HANDOFF.md).

## Module map

| Path | What |
|---|---|
| `sim/scenario.py` | Seeded 3×3 rubble grid (piles, about 600 pieces, hidden victims and sounds), shared by everything. |
| `sim/grid_mission.py` | The autonomous multi-pile mission: survey → land → radar → ping → listen → rank → dispatch. |
| `sim/localise.py`, `sim/planner.py` | Grid Bayes localisation, and next-scan selection by expected information gain. |
| `sim/radar_dsp.py` | Labelled-synthetic FMCW generator plus the real breathing DSP chain (range FFT, phase, band-pass, CFAR). |
| `mapping/` | Ray-cast LiDAR, KISS-ICP SLAM pile maps, and landing-cell agreement checks. |
| `acoustic/` | Mic-array scene, DoA, VAD, Whisper, YAMNet and the per-clip scorer. |
| `agents/` | Gemini or fallback dispatch decider, operator-note intake and rescuer handover, with record/replay. |
| `dashboard/` | Operator console: 3×3 map, pings, priorities, transcripts, decisions, LiDAR detail view. |
| `isaac/` | Isaac Sim adapter: pxr scene, kinematic drones, PhysX LiDAR, overlays, `--selftest`. |
| `animation/` | Interactive concept animations (storyboard). |
| `contracts/INTERFACES.md` | Frames, units and message schemas. |
| `docs/` | Plan, research, validations, review passes, Isaac handoff. |

## Honest limits

- **Radar is a labelled synthetic generator.** Isaac does not model RF breathing through rubble. The breathing DSP was checked on real recordings (Schellenberger et al. 2020: 0.39 breaths/min MAE). That data is 3 resting subjects, line-of-sight 24 GHz CW radar at about 0.4 m, with no rubble, so it validates the signal processing only. See [docs/RADAR_VALIDATION.md](docs/RADAR_VALIDATION.md).
- **Acoustic scenes are simulated** (TTS voices, synthesised knocks). Validation on real recordings is in progress: [docs/ACOUSTIC_VALIDATION.md](docs/ACOUSTIC_VALIDATION.md) once it lands.
- **A ping is a candidate search region, not a confirmed person.** Silent piles are "not cleared": no detection is not proof of absence.
- **Flat landing geometry says nothing about structural safety.**
- **Flight is kinematic:** no aerodynamics and no wind.
- **Planning code never reads ground truth.** Only the observation generators and the evaluators do, and tests enforce this.
