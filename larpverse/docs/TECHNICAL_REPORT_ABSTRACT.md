# Larpverse: abstract

After a building collapse, people are trapped in voids cameras cannot see, and the incident commander must choose which pile to search and where to dig. Larpverse, an autonomous three-drone system, supports that decision. Case study: Weiguan Jinlong, Tainan, 2016 (309 people in the building, 175 rescued alive, 115 dead).

**How it works.** Heavy-lift drones map each pile with LiDAR and SLAM. They then lower a 4.9 kg radar and microphone pod on a 20 m winch onto the rubble, so nobody has to stand on the pile. The pod looks for breathing through the rubble. A Bayes filter fuses the ranges; the pod is re-placed where it expects the most information. When three ranges agree, the result is a **candidate search region** with a breathing **condition** (an aid, not a diagnosis). Otherwise it is "insufficient evidence".

Landed off the pile with rotors off, the drones listen, call out in three languages, and run a rescue Wi-Fi hotspot that phones can choose to join. An agent (Gemini or a rule-based fallback) ranks the piles, and code validates every pick.

**Evidence.**
- Real data: breathing DSP at 0.39 breaths/min MAE on line-of-sight 24 GHz recordings (no rubble).
- Real data: acoustic recognisers at F1 0.70–0.82 with rotors off.
- Simulation: modelled reach is about 2.9 m beneath the pod.
- Simulation, 20 seeds: acoustic-led dispatch pinged 17 of 60 non-start victims, against 6 and 5 for radar-only baselines. The acoustic clips were generated from ground truth, so this is an upper bound. There were no false pings, and the system is not faster overall.

**Limits.** Through-rubble radar returns and the victim audio are synthetic. The pod is designed but not built. There is no flight controller. Some victims are missed, and no pile is ever "cleared".
