# SkySearch: technical summary (draft for the one-page PDF)

**User and problem.** A mountain rescue or fire service search coordinator looking for people lost in forest
after a disaster. Drones today are flown one per pilot, along fixed lawnmower lines. Every warm animal becomes a
radio call, and the coordinator drowns in false alarms.

**What SkySearch does.** A fleet of 5 drones searches as one system:
1. It starts from a lost-person-behaviour prior around the last known point.
2. It keeps a Bayesian search map of where the person could still be.
3. Each drone flies to the highest-value cells in its own region of the map.
4. On a thermal hit, a drone drops to 18 m and uses its RGB camera to rule out deer and rocks on-board.
5. A human is involved only when a drone is confident it has found someone, or genuinely unsure.

**Key features**
- **Four sensors:** thermal, RGB, LiDAR and microphone. LiDAR measures canopy density, which corrects the detection model, and avoids trunks at low altitude.
- **Coverage:** probability-weighted Voronoi coverage control. When a drone fails, the survivors' regions redraw at once.
- **Map sharing:** decentralised map fusion that never double-counts a look. Drones out of radio range keep working and merge when back in range.
- **Re-planning:** fires seen on thermal become no-fly zones, and paths reroute with A*.
- **Moving subjects:** searched-area evidence decays over time, so a person who walks into a searched area is not ruled out forever.

**Architecture and where each model runs.** Everything runs on each drone; the base station only relays to the coordinator.

| Component | Runs on | Latency |
| --- | --- | --- |
| Search map, planner, state machine | drone CPU (target: Jetson Orin Nano) | 0.26 ms per tick measured on a cloud CPU core; to re-measure on Orin |
| Thermal blob detector | drone | Isaac Sim proxy from semantic segmentation (simulated) |
| RGB person-vs-animal classifier | drone | currently a calibrated stub; YOLOv8n planned, latency to be measured |
| LiDAR avoidance, mic bearing | drone | simple geometry, under 1 ms |

**Evaluation (simulation only, not real-world performance).** 60 randomised forests, 2 missing people each, with a random drone failure and a random fire. Compared against the standard parallel-sweep (lawnmower) search using the same drones, sensors and batteries.

| Metric | SkySearch | Lawnmower baseline |
| --- | --- | --- |
| Median time to find a person | **106 s** (95% CI 91–141) | 176 s (95% CI 154–222) |
| People found within 10 min | 87.5% | 83.3% |
| Messages to the human coordinator per mission | **1.8** | 19.3 |
| False alarms reaching the coordinator per mission | **0** | 17.4 |

**Limits**
- Simulated sensors only. The thermal camera is a proxy, and the RGB classifier is a stub.
- About 12% of people are still not found within 10 minutes, mostly under dense canopy.
- Flight is kinematic: no wind and no full flight dynamics.
- Radio is modelled as a fixed range.

**Critical failure mode and safety**
- **Critical failure mode:** a missed person. Mitigations: a find needs two sensors to agree (thermal, then RGB); uncertain cases go to a human, never to auto-reject; and evidence decays so areas are revisited.
- **Collisions:** drones keep a 30 m separation, and LiDAR avoids trunks below the tree line.
- **Battery:** each drone returns with a 10% reserve.
- **Drone failure:** emergency landing, and the fleet re-plans.
- **Privacy:** no face recognition and no tracking of identified individuals. The system is scoped to search and rescue only.
