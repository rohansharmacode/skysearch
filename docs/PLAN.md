# SkySearch: feature plan mapped to the Robotics judging rubric

**One line:** a fleet of drones finds missing people in a forest after a disaster. The drones work out where to
look, rule out decoys themselves, and only bring in a human when they are unsure, entirely on-board and with no
cloud connection.

**User:** a mountain rescue or fire service **search coordinator**. **Critical failure mode:** missing a person.
The second is flooding the coordinator with false alarms so that a real find is missed.

We dropped the "criminals on the run" use case: it raises surveillance and civil-liberties questions that would
cost us on responsible design.

| Rubric (weight) | What we show on screen | Where in code |
| --- | --- | --- |
| **Perception-driven action (25%)** | **Thermal blob detected:** the drone breaks off, drops to 18 m, and reads the RGB classifier until it is sure.<br>**Voice heard by the microphone:** the drone raises the map along the bearing.<br>**LiDAR:** measures canopy density, which corrects the detection model, and dodges tree trunks when flying low.<br>**Fire:** seen on thermal, it becomes a no-fly zone.<br>**Decision log:** every line names the sensor reading that caused the action. | `swarm.py: drone_tick`, `world.py` sensors |
| **Adaptation to change (25%)** | **Unscripted buttons on the dashboard:** kill a drone, start a fire, move the missing person.<br>**When a drone dies:** the survivors' search areas visibly re-split (Voronoi regions redraw).<br>**When a fire appears:** paths reroute with A*.<br>**When the person moves:** old "searched" evidence decays, so the area gets revisited. | `planner.py`, `live.py` |
| **Operational autonomy (15%)** | **Decision cost:** brain compute per drone per tick is shown live (~0.2 ms on a laptop CPU). The edge budget is stated in the technical summary.<br>**Radio dropout:** a drone out of range keeps searching on its own map copy and merges when back.<br>**When unsure** (p between 0.10 and 0.92 after 8 s): it hovers and asks the human; it never guesses.<br>**Battery:** each drone returns with a reserve, swaps batteries, and relaunches. | `swarm.py`, `belief.py: fuse` |
| **Relevance and impact (20%)** | **Named user:** the search coordinator.<br>**Prior:** search starts from a lost-person-behaviour model (distance rings around the last known point).<br>**Critical failure handled on screen:** a deer looks like a person on thermal, and the drone rules it out itself.<br>**Benchmark:** compared with the standard lawnmower sweep. | `belief.py: lost_person_prior`, `eval/` |
| **Ambition and technical quality (15%)** | **Search model:** a Bayesian search map (Koopman search theory) with probability-weighted Voronoi coverage control (Cortés et al.).<br>**Fusion:** decentralised map fusion that never double-counts a look.<br>**Sensors:** four sensor types.<br>**Simulators:** Isaac Sim on Brev, plus a CPU twin with the same brain. | all |

## Stretch ideas, if ahead of schedule
1. **Real detector:** run YOLOv8n on the Isaac RGB camera instead of the semantic-label stub, and quote measured latency.
2. **Relay drone:** when a drone goes out of range, one drone repositions as a radio relay.
3. **Drop a supply:** once a person is confirmed, the beacon drone "drops" a thermal blanket or radio (pitch moment).
4. **Wind:** wind affects battery use, so battery-aware routing matters.
5. **Coordinator voice summary:** an on-device summary every 60 s (small local model). Keep it off the critical path.

## The 60-second video (single continuous take)
| Time | What happens |
| --- | --- |
| 0–10 s | Drones launch and spread into their regions. The heat map shows the prior around the last known point. |
| 10–25 s | A thermal blob appears. The drone drops low and reads p(person). It turns out to be a **deer**, which is ruled out on-board. |
| 25–40 s | Teammate clicks **Kill drone** (unscripted). Regions re-split and the others take over its area. |
| 40–55 s | Microphone hears a voice, a drone inspects, **CONFIRMED** at p > 0.92, and it hovers as a beacon while the operator is alerted. |
| 55–60 s | Teammate clicks **Start fire**. The no-fly zone appears and paths reroute. |
