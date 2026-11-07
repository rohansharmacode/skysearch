# SkySearch: autonomous search-and-rescue drone swarm (Agentverse 2026, Robotics track)

A swarm of 10 drones searches a partly collapsed office building after an earthquake and hands the rescue crew a
map: where each trapped person is, how sure the swarm is, which rooms are still unsearched, and a safe crawl route.
Every flight and search decision is made on the drones, with no cloud in the loop.

| Drone | How many | Sensors | Job |
| --- | --- | --- | --- |
| **Coordinator (M)** | 1, large | radio relay, loudspeaker, docking pads | stays outside, relays radio, allocates tasks, calls quiet periods, swaps batteries |
| **Scouts (S1–S3)** | 3, medium | S1, S2: 360° LiDAR + ultrasound + RGB. S3: UWB through-rubble radar | map rooms and corridors; S3 takes over where smoke blinds LiDAR |
| **Crevice drones** | 6, palm-sized | microphone, RGB camera, ToF; C1, C2 also carry 60 GHz breathing radar | L1–L4 land and listen for knocking (2 stay landed, 2 lift off to search voids). C1, C2 fly into voids and confirm breathing |

## The demo mission (team storyboard)
1. Launch from the coordinator.
2. Scouts map while listeners land on flat spots picked from the LiDAR map.
3. Knock heard with a bearing. A confirmer hovers, breathing is confirmed, and Victim A is marked high confidence.
4. Fire in the east wing: smoke degrades LiDAR and RGB, so the mapper pulls out.
5. The coordinator hands the east wing to the radar scout S3.
6. A knock comes from a room already cleared (room 7). The room is re-opened, and radar finds breathing behind a collapsed shelf. Victim B is marked medium confidence, radar only.
7. Hand map to crew: pins with confidence, unsearched area hatched, safe route, and a radio briefing written by Gemini from the swarm's findings.

Video: `out/skysearch_story.mp4` (rendered with `scripts/render_story.py`). Isaac Sim version: `media/isaac_story.mp4`, once recorded.

## Quick start (CPU, any laptop)
```bash
pip install numpy scipy matplotlib
python -m skysearch.usar.live --story --port 8090        # mission control with unscripted-event buttons
python scripts/render_story.py --seed 3 --out out/skysearch_story.mp4 --workers 3   # storyboard video
GEMINI_API_KEY=... python scripts/render_story.py --seed 3 --gemini                 # with the Gemini crew briefing
python -m eval.usar_benchmark --seeds 12 --workers 3     # why three drone types (random placement)
python -m eval.usar_benchmark --seeds 12 --story --duration 300                     # storyboard-type buildings
```
Dashboard buttons: start a fire, make a trapped person knock, trigger an aftershock (click the map), kill a drone,
hand the map to the crew, pause, restart. The swarm is never told in advance; it has to notice through its sensors.

## Isaac Sim 6 on NVIDIA Brev
```bash
cd /workspace/isaaclab
./isaaclab.sh -p ~/skysearch/isaac/earthquake_scene.py --story --livestream 2 --enable_cameras      # live
./isaaclab.sh -p ~/skysearch/isaac/earthquake_scene.py --story --headless --enable_cameras --record out/isaac
```
The building is built from the same search grid (walls, rubble heaps, voids under slabs), with PhysX debris that
falls during the quake and aftershocks. The same swarm code drives the drones; the confirmer's RGB camera feed comes
from Isaac. Dashboard at `<VS Code URL>/proxy/8090/`. See [docs/BREV_SETUP.md](docs/BREV_SETUP.md) and
[docs/ISAAC6_API.md](docs/ISAAC6_API.md).

## Layout
| Path | What |
| --- | --- |
| `skysearch/usar/fleet.py` | per-drone autonomy and the coordinator: task auctions, listening posts, knock triangulation, smoke handover, confirmation, battery swaps, radio links |
| `skysearch/usar/building.py` | ground-truth building and every sensor model (LiDAR, ultrasound, UWB radar, mmWave radar, microphone, RGB) |
| `skysearch/usar/scenario.py` | events: quake, aftershocks, fire, storyboard timing |
| `skysearch/usar/render.py`, `live.py` | video renderer and live mission control |
| `scripts/briefing.py` | Gemini crew briefing (one call at handoff, facts only, template fallback) |
| `eval/usar_benchmark.py` | fleet-mix ablations |
| `isaac/earthquake_scene.py` | Isaac Sim 6 scene driven by the same swarm |
| `skysearch/*.py`, `eval/benchmark.py`, `isaac/isaac_bridge.py` | the earlier forest scenario (v1) |

## Honesty notes
- All data is simulated. Sensors are models with noise, range limits and degradation from dust and smoke, not real hardware.
- The world is a 2.5D grid (0.4 m cells with a clearance height each); flight is kinematic, with no aerodynamics.
- In Isaac, the building and drones are rendered and the debris is simulated by PhysX, but the drones are moved by the swarm code, not by a flight controller.
- Gemini only writes the crew briefing at handoff. The drones never wait for it, and it is told to use only the swarm's facts.
- Benchmark numbers come from 12 simulated buildings per variant and are not real-world performance.
