# Larpverse

Autonomous multi-drone search and rescue demo for the AgentVerse 2026 robotics track.

Three drones survey a rubble pile with LiDAR, land on flat cells, scan with a synthetic radar for respiration-like returns, and fuse the ranges into a candidate search region with uncertainty. When the evidence is ambiguous, they relocate and rescan. Microphones add direction-of-arrival cones that suggest where to search next.

- `docs/PLAN.md`: build plan, task split and timeline. **Start here.**
- `docs/RESEARCH.md`: open-source models, datasets and Isaac Sim notes.
- `contracts/INTERFACES.md`: frames, units and message schemas. Everyone codes to this.
- `animation/`: 3D concept animation, for explaining the idea.
- `dashboard/`: operator console that polls `dashboard/data/state.json`.
- `sim/`: engine-agnostic coordinator core (numpy) with a mock world. Isaac plugs in behind the same interface.

```bash
uv venv .venv && uv pip install numpy
```

```bash
.venv/bin/python -m sim.coordinator --realtime --out dashboard/data
```

```bash
python3 -m http.server 8765
```

Then open http://localhost:8765/dashboard/ and http://localhost:8765/animation/.
