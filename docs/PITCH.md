# 5-minute pitch outline (earthquake version)

1. **Hook (0:00–0:30).** Play the Isaac clip of the ground shaking and the corridor ceiling coming down.
   - Then say: "In the first hour after an earthquake, nobody knows who is inside or where. Most survivors are in spaces too small for a person or a big drone."
   - Cite one real figure (for example, from an INSARAG or UK ISAR source). Check it before using it.
2. **User and problem (0:30–1:00).** The user is the USAR search team leader.
   - Today: dogs, listening devices placed by hand, cameras on poles, one drone flown from outside.
   - Risks: sending people into an unstable building, and missing someone in a void.
3. **Live demo (1:00–3:00).** Mission control on the laptop. A judge presses the buttons:
   - start a fire, so the mapper is blinded and the radar scout takes over;
   - make a trapped person knock in a room already cleared, so the room re-opens and a crevice drone goes in;
   - trigger an aftershock, so every drone holds, the map goes stale and routes are re-checked;
   - kill a drone, so its tasks are re-auctioned.
   Then press **Hand map to crew**: pins with confidence, the unsearched area, the safe route, and the Gemini radio briefing.
4. **How it works (3:00–3:45).** One diagram: dashboard → coordinator → drones → sensors.
   - Three drone types, each doing what only it can.
   - Confidence comes from agreement between sensors: camera and breathing radar for HIGH, radar alone for MEDIUM.
   - Every decision is made on the drones.
5. **Evidence (3:45–4:20).** The ablation table: same buildings, 10 airframes each, simulation only.
   - Big drones alone found nobody.
   - Without listeners, 75% of victims were found; with them, 100%.
   - Without breathing radar, 46%.
   - Say the uncomfortable result too: in our small building, a fleet made only of crevice drones did as well as our mix. We have not yet shown the scouts earn their place, and larger buildings are next.
6. **What it can't do (4:20–4:45).** Sensors are models, flight is kinematic, and there is no real hardware yet.
   - In Isaac the debris is real PhysX and the rendering is real, but the drones are moved by our code, not by a flight controller.
   - Next steps: a real detector on the RGB feed, PX4 flight, then a test on a rubble training site.
7. **Close (4:45–5:00).** "The crew walks in knowing where to dig, how sure we are, and which way is safe."
