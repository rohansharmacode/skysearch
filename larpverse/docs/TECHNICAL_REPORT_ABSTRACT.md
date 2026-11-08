# Larpverse: abstract

After a building collapse, people are trapped in voids cameras cannot see, and the incident commander must choose which pile to search and where to dig. Larpverse, an autonomous three-drone system, supports that decision. Case study: Weiguan Jinlong, Tainan, 2016 (309 people in the building, 175 rescued alive, 115 dead).

**How it works.** Heavy-lift drones map each pile with LiDAR and SLAM. Each drone uses its LiDAR map to find a flat spot on the rubble and lands there, so nobody has to stand on the pile. Its downward-facing radar (a 4.9 kg radar and microphone module on the drone's underside) then looks for breathing through the rubble. A Bayes filter fuses the ranges; the drone lifts off and lands where it expects the most information. When three ranges agree, the result is a **candidate search region** with a breathing **condition** (an aid, not a diagnosis). Otherwise it is "insufficient evidence".

Landed off the pile with rotors off, the drones listen, call out in three languages, and run a rescue Wi-Fi hotspot that phones can choose to join. An agent (Gemini or a rule-based fallback) ranks the piles, and code validates every pick.

**Evidence.**
- Real data: breathing DSP at 0.39 breaths/min MAE on line-of-sight 24 GHz recordings (no rubble).
- Real data: acoustic recognisers at F1 0.70–0.82 with rotors off.
- Simulation: modelled reach is about 2.9 m beneath the landed drone's belly radar.
- Simulation, 20 seeds: acoustic-led dispatch pinged 17 of 60 non-start victims, against 6 and 5 for radar-only baselines. The acoustic clips were generated from ground truth, so this is an upper bound. There were no false pings, and the system is not faster overall.

**Limits.** Through-rubble radar returns and the victim audio are synthetic. The belly radar module is designed but not built. There is no flight controller. Some victims are missed, and no pile is ever "cleared".
