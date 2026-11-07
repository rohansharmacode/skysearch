# SkySearch: feature plan mapped to the Robotics judging rubric (earthquake version)

**One line:** 10 drones search a collapsed building after an earthquake, decide everything on board, and hand the
rescue crew a map with victims, confidence, unsearched areas and a safe route.

**User:** the USAR **search team leader**.
**Critical failure modes:** a missed person, or a room wrongly marked clear. The second risk is sending the crew in on a false alarm.

| Rubric (weight) | What we show on screen | Where in code |
| --- | --- | --- |
| **Perception-driven action (25%)** | **Knock heard:** bearings from landed listeners are fused into a location, and a confirmer flies in.<br>**Camera and breathing radar agree:** HIGH confidence.<br>**Breathing on radar behind an obstacle:** MEDIUM confidence, radar only.<br>**Smoke:** scattered LiDAR returns and a grey camera image make the mapper pull out.<br>**Aftershock felt on the IMU:** all drones hold.<br>**Decision log:** every line names the reading that caused the action. | `fleet.py: drone_tick`, `building.py` sensors |
| **Adaptation to change (25%)** | **Unscripted buttons:** fire, knock, aftershock, kill drone.<br>**Fire:** the radar scout takes over the smoky wing.<br>**Knock from a cleared room:** the room is re-opened.<br>**Aftershock:** the map goes stale and routes are re-checked.<br>**Drone loss:** tasks are re-auctioned, and the coordinator role is handed over if needed. | `fleet.py: allocate, radar_handover, triangulate`, `live.py` |
| **Operational autonomy (15%)** | **On-board decisions:** the whole fleet step takes a median 4 ms per 0.5 s tick for 10 drones, including simulated sensors.<br>**Radio dropout:** a drone keeps working and merges its map later, and a relay is sent.<br>**Battery:** each drone keeps a reserve sized to its route home, then swaps batteries on the coordinator.<br>**Humans:** only unsure detections go to a person.<br>**Gemini:** called once, at handoff, and never in the control loop. | `fleet.py`, `scripts/briefing.py` |
| **Relevance and impact (20%)** | **The user's workflow:** call-out, then quiet periods to listen, then confirmation, then the handover map. This mirrors USAR acoustic search.<br>**Ablation benchmark:** each drone type and sensor is tested on the same buildings. | `eval/usar_benchmark.py` |
| **Ambition and technical quality (15%)** | **Fleet:** three drone classes with a coordinator.<br>**Sensing:** acoustic triangulation, two radar types, and a 2.5D clearance map with void search.<br>**Isaac Sim 6:** the building is generated from the same map. The earthquake is simulated in PhysX: ground acceleration as a gravity tilt, slabs and blocks falling.<br>**Cameras:** drone first-person and chase views. | all, `isaac/` |

## Stretch ideas
1. **Real detector:** run a person detector on the Isaac RGB camera instead of the calibrated model, and quote its measured latency.
2. **Real flight:** PX4 or Pegasus flight dynamics for the scouts, so drones are flown, not placed.
3. **Larger building:** test a bigger site (multi-wing, longer missions) to see whether the scouts earn their place. The current benchmark says they don't in a 28 × 18 m building.
4. **Smarter listening:** spread the listeners out; in our runs, sentinels close together gave poor fixes for distant knocks.

## Demo video (about 1 min)
| Time | What happens |
| --- | --- |
| 0–10 s | Physics earthquake in Isaac: shaking, the upper floor pancakes, dust. |
| 10–25 s | Launch. Scouts map while listeners land. Knock heard, a bearing is drawn, and the confirmer goes in (chase cam). |
| 25–35 s | First-person view under the slab: breathing confirmed. Victim A, HIGH. |
| 35–45 s | Fire in the east wing, smoke blinds the mapper, and the radar scout takes over. |
| 45–55 s | Knock from cleared room 7. The room is re-opened and radar finds Victim B behind the shelf: MEDIUM. |
| 55–60 s | Hand map to crew: pins, unsearched area, safe route, and the Gemini briefing. |
