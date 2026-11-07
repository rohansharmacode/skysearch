# Larpverse: abstract

After a building collapse, survivors are trapped in voids inside rubble piles, where cameras and thermal imagers cannot see. The incident commander has to decide which pile to work first and where on it to dig. Larpverse is an autonomous three-drone system that supports that decision (case study: the Weiguan Jinlong collapse, Tainan, 2016).

**How it works.** The drones map each pile with LiDAR and KISS-ICP SLAM, then land on flat cells. With rotors off, each runs a low-frequency radar breathing-detection chain. A grid Bayes filter fuses the range rings. When the scans disagree, the planner relocates a drone to the landing site with the highest expected information gain. The output is a **candidate search region** with a belief share and a 95% region, or an explicit `insufficient_evidence`.

The landed drones then listen to the other piles. SRP-PHAT direction finding and multi-drone association feed Silero VAD, YAMNet and Whisper. A bounded agent (Gemini or a rule-based fallback) ranks the piles and picks the next one to visit. Code validates every pick; the agent never sees ground truth.

**Evidence.**
- Real data: the breathing DSP scores 0.39 breaths/min MAE on real 24 GHz recordings (3 subjects, line of sight, no rubble).
- Simulation: SLAM error 3.6 cm; landing-map IoU 0.98.
- Simulation: on one pile, planned relocation resolves 43% of 30 runs against 10% with fixed landing sites (provisional).
- Simulation: an acoustic association fix removed all confident false leads across 5 seeds, at some cost to recall on knocking.

**Limits.** Radar returns through rubble are a labelled synthetic generator, and its default propagation is still the optimistic preset. Acoustic demo scenes are synthetic. Depth is assumed, not measured. Flight is kinematic. Silent piles are never "cleared". Multi-pile, real-audio and vital-sign results are pending.
