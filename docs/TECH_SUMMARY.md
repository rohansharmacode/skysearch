# SkySearch: technical summary (draft for the one-page PDF)

**User and problem.** The user is the search team leader of an urban search-and-rescue (USAR) team at a partly collapsed building, in the
first hour after an earthquake. Nobody knows how many people are inside or where. Sending people in is dangerous.
Today the team uses search dogs, listening devices placed by hand, cameras on poles, and at most one drone flown by a pilot from
outside. None of these can get into the voids under collapsed floors, where most survivors shelter.

**What SkySearch does.** A team of 10 drones searches the building as one system and hands the crew a map.
1. **Coordinator (1 large drone).** Stays outside. It relays radio, allocates tasks by auction, calls quiet periods and broadcasts "knock three times". It also swaps batteries on its docking pads.
2. **Scouts (3 medium drones).** S1 and S2 map rooms and corridors with LiDAR and ultrasound. S3 carries a UWB through-rubble radar and takes over mapping wherever smoke blinds LiDAR and cameras.
3. **Crevice drones (6 palm-sized drones).**
   - **Listeners L1–L4.** They land on flat spots picked from the LiDAR map and listen with microphone arrays. Bearings from several listeners are fused to locate a knock. Two stay landed all mission; the other two lift off to search voids 20 s after the call-out.
   - **Confirmers C1 and C2.** They fly into voids under 0.9 m with an RGB camera and a 60 GHz breathing radar.
4. **Confirmation rules.**
   - **High confidence:** the camera and breathing radar agree.
   - **Medium confidence:** radar only, for example behind an obstacle.
   - **Unsure:** a snapshot goes to the incident commander.
5. **Handoff to the crew.** The crew gets victim pins with confidence and evidence, rooms not yet searched, hazards, and a crawlable route outside smoke. Gemini writes a short radio briefing from those facts only.

**Adaptation (all unscripted in the live dashboard).**
- **Fire and smoke.** Mappers pull out and the radar scout takes over.
- **Knock from a cleared room.** The room is re-opened and gets priority.
- **Aftershock.** Every drone holds on its IMU reading, the map is marked stale, and every route is re-checked before use.
- **Drone loss.** Tasks are re-auctioned. If the coordinator is lost, its role passes to the drone with the best link.
- **Radio loss.** A drone out of contact keeps working on its own map and merges it when back in contact. A relay is sent towards it.
- **Low battery.** The drone returns with a reserve sized to its route home.

**Where each part runs.**

| Component | Runs on | Cost |
| --- | --- | --- |
| Mapping, task auctions, listening, confirmation, state machines | on the drones (coordinator logic runs on the large drone) | whole fleet step: median 4.1 ms per 0.5 s tick for all 10 drones, including the simulated sensors (0.4 ms per drone, p95 20 ms), on one cloud CPU core |
| Person detector on RGB | crevice drones | a calibrated model of a small detector (score depends on range, dust, smoke and burial); no trained network yet |
| Breathing radar, UWB radar, mic bearings, LiDAR | drones | signal models with noise, range and smoke/dust degradation |
| Crew briefing | cloud (Gemini), once, at handoff | not in the control loop; built-in template if offline |

**Evaluation (simulation only, not real-world performance).** All fleets have 10 airframes and the same buildings. Each building has randomised collapse, desks, decoys (mannequins, clothes) and one aftershock.
`python -m eval.usar_benchmark` (12 buildings, 3 people each, 10 min):

| Fleet | People found | Median time to find* | All found in run | Human reviews / run | Drones lost / run |
| --- | --- | --- | --- | --- | --- |
| SkySearch: 1 coordinator + 3 scouts + 6 crevice | 31/36 (86%) | 168 s | 67% | 0.75 | 0.58 |
| Scouts only (1 + 9) | 0/36 | — | 0% | 0 | 1.08 |
| Crevice drones only (1 + 9) | **35/36 (97%)** | **109 s** | 92% | 1.5 | 0 |
| SkySearch without listeners | 32/36 (89%) | 73 s | 75% | 0.25 | 0.08 |
| SkySearch without breathing radar | 32/36 (89%) | 170 s | 75% | **3.17** | 0.25 |

Storyboard-type buildings. Victim A is conscious in the west wing. Victim B is unconscious behind a collapsed shelf until 110 s. There is a fire in the east wing. Each run lasts 5 min, and A and B are scored. Command: `--story --duration 300`.

| Fleet | People found | All found in run | False confirmations / run |
| --- | --- | --- | --- |
| SkySearch | **24/24 (100%)** | **100%** | 0 |
| Scouts only | 0/24 | 0% | 0 |
| Crevice drones only | 23/24 (96%) | 92% | 0 |
| Without listeners | 18/24 (75%) | 50% | 0 |
| Without breathing radar | 11/24 (46%) | 0% | 0.08 |

\*Median over all people, with a person not found counted at the time limit.

**What the numbers say, including where they disagree with us.**
- **Small drones are essential.** Big drones alone found nobody, because people shelter in voids lower than 0.9 m.
- **Listening pays off when someone becomes findable late.** An example is a person who wakes up and knocks (100% vs 75%). When everyone is detectable from the start, landing to listen slows the search (median 168 s vs 73 s).
- **Breathing radar finds people behind obstacles.** It also cuts human reviews from 3.2 to 0.75 per run.
- **In this small building (28 × 18 m), 9 crevice drones did as well as or better than our mixed fleet.** The scouts' simulated contributions did not outweigh 3 more small drones. Those contributions are the LiDAR map for listening posts, the radio relay, and the radar sweep in smoke. The scouts were also caught by aftershocks more often. We have not shown that scouts earn their place. Larger buildings and longer missions, where the small drones' ~7 min battery and short radio range matter more, are untested.
- **Twelve buildings is a small sample.** One person is about 3 percentage points.

**Limits**
- Sensors are models, not hardware.
- The building is a 2.5D grid (0.4 m cells, each with a clearance height).
- Flight is kinematic: no aerodynamics, no downwash, no collisions between drones.
- Radio is a path-loss threshold.
- In Isaac Sim, the building, the debris physics (PhysX) and the drone camera are rendered on the GPU. The drones are moved by the swarm code, not by a flight controller.

**Critical failure mode and safety**
- **Critical failure: a missed person, or a room wrongly marked clear.**
  - A room is cleared only after camera coverage and all voids are searched.
  - A knock from a cleared room re-opens it.
  - Every victim pin shows its confidence and evidence. Radar-only finds are marked medium.
  - Unsure detections go to a human; the drones never auto-reject them.
- **False alarm (a mannequin or clothes).** A pin needs breathing on radar, or a human's confirmation.
- **Aftershock.** Every drone holds, the map is marked stale, and routes are re-checked. A drone trapped by debris lands to save power and keeps listening.
- **People decide.** Crew entry decisions stay with the team leader. Gemini sees only structured facts, never images, and is told not to add facts.
- **Privacy.** No face recognition and no identification. Cameras are used only to detect that a person is present.
