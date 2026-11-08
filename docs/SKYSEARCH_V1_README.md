# SkySearch: autonomous search-and-rescue drone swarm (Agentverse 2026, Robotics track)

A fleet of drones finds missing people in a forest after a disaster. Each drone does the following on-board,
with no cloud connection:
- decides where to look next from a shared Bayesian search map
- breaks off to inspect thermal hits, then rules out deer and rocks itself
- asks the human coordinator only when genuinely unsure
- re-plans when a drone fails, a fire starts, or the person moves

## Quick start (CPU, any laptop)
```bash
pip install numpy scipy matplotlib
python -m skysearch.live            # mission control at http://localhost:8080 with unscripted-event buttons
python run_demo.py --out out/demo.mp4 --duration 300  # renders a ~50 s video
python -m eval.benchmark --seeds 60 # SkySearch vs lawnmower baseline, writes out/benchmark.json
```

## Isaac Sim on NVIDIA Brev
See [docs/BREV_SETUP.md](docs/BREV_SETUP.md). `isaac/isaac_bridge.py` runs the identical brain with Isaac cameras as sensors.

## Layout
| Path | What |
| --- | --- |
| `skysearch/swarm.py` | per-drone autonomy: sense, decide, act; radio links, operator hand-off |
| `skysearch/belief.py` | Bayesian search map, lost-person prior, decentralised fusion |
| `skysearch/planner.py` | probability-weighted Voronoi coverage, target choice, A* around fire |
| `skysearch/world.py` | ground truth + simulated thermal, RGB, LiDAR, microphone |
| `skysearch/live.py` | live dashboard + unscripted event injection |
| `eval/` | lawnmower baseline + Monte Carlo benchmark |
| `isaac/` | Isaac Sim bridge |
| `docs/` | feature plan, Brev setup, technical summary, pitch |

## Honesty notes
- All data is simulated.
- Thermal in Isaac is a proxy built from semantic segmentation of warm bodies.
- The RGB classifier is a calibrated stub (score depends on altitude, canopy and class) until YOLOv8n is plugged in.
- Benchmark numbers are from simulation and are not real-world performance.
