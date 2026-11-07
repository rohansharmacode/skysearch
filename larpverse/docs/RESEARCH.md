# Open-source stack: what to use and why

These findings were checked on 7 Nov 2026. "Unverified" means we could not confirm it from a source. Check licences before you ship anything.

## Decisions at a glance

| Need | Use | Licence | Fallback |
|---|---|---|---|
| Simulator | Isaac Sim 5.1 (cloud Launchable) or 6.x if a local RTX box has it | NVIDIA | Our numpy mock world (`sim/world.py`) |
| Drones in Isaac | Shipped `IsaacSim/Quadcopter/quadcopter.usd`, posed kinematically each step | NVIDIA | `Bitcraze/Crazyflie/cf2x.usd` |
| Rubble | Procedural: drop seeded boxes and slabs as rigid bodies, settle about 2 s, freeze. Same recipe as MIT RubbleSim | ours | Poly Haven CC0 rocks and concrete barriers |
| LiDAR in Isaac | RTX LiDAR (`Example_Rotary`); PhysX raycast if RTX is fiddly | NVIDIA | Heightmap raycasts |
| SLAM / mapping | KISS-ICP 1.3 (pip, MIT) plus Open3D 0.20 (MIT) | MIT | Sim ground-truth poses with noise, then just stack the scans |
| Landing cells | numpy plane fit per cell: slope, roughness, step, clearance | ours | n/a |
| Radar signal chain | numpy FMCW IF generator → range FFT → phase → unwrap → 0.1–0.5 Hz Butterworth (scipy) → spectral peak | ours | OpenRadar (Apache-2.0, install from git) for reference DSP |
| Real radar data for validation | Schellenberger et al. 2020, Sci Data 7:291, 24 GHz CW, 30 subjects, CC BY 4.0 | CC BY 4.0 | Extremadura mmWave (Zenodo 18599983) |
| Voice activity | Silero VAD 6.2 (MIT, CPU) | MIT | n/a (webrtcvad is stale) |
| Sound class (knock, shout, scream) | YAMNet (TF Hub, 521 AudioSet classes) | Apache-2.0 | PANNs (MIT), LAION-CLAP zero-shot (CC0) |
| Direction of arrival | pyroomacoustics 0.10 SRP-PHAT or MUSIC (MIT). It also simulates the mic array | MIT | n/a |
| Distress-audio data | DroneAudioSet (MIT, on Hugging Face): screams, speech, knocks, rotor noise | MIT | DREGON; ESC-50 (non-commercial) |
| Point cloud in browser | three.js `Points` from a binary Float32 file | MIT | deck.gl PointCloudLayer |
| Localisation and next scan | numpy grid Bayes filter plus expected information gain | ours | n/a |

## Isaac Sim

- **Versions.** The latest is 6.2.0 (October 2026, Python 3.12). Install it with `pip install "isaacsim[all,extscache]==6.x" --extra-index-url https://pypi.nvidia.com`.
- **Version churn.** APIs moved in 6.0: `isaacsim.core.experimental.prims` and `isaacsim.sensors.experimental.rtx`. The older names are deprecated. Pin one version early and don't mix tutorials.
- **Hardware.** It needs at least an RTX 4080 (16 GB VRAM) and 32 GB RAM. A100 and H100 are not supported.
  - The dev laptop in this repo has no NVIDIA GPU and 14 GB RAM, so Isaac runs on a teammate's RTX machine or in the cloud.
  - Cloud option: Brev with 1× L40S, roughly $1–2/hr. The ready-made Launchable ships 5.1.
- **Drones.** Don't use Pegasus (works only on 5.1, broken on 6) or OmniDrones (Isaac 4.1 era). Use kinematic pose setting instead. We don't need aerodynamics.
- **Moving them.** Load the quadcopter USD three times and call `set_world_poses` each step from the coordinator's waypoint interpolation. Keep rigid-body physics off on the drones.
- **LiDAR.** On 6.x it is `Lidar.create(path, config="Example_Rotary")` plus `LidarSensor(..., annotators=["generic-model-output"])`. On 5.1 it is `IsaacSensorCreateRtxLidar` plus a point-cloud annotator. Both are unverified in detail, so read the docs for your installed version.
- **Rubble collisions.** Use convex decomposition so the LiDAR and drones collide with the pile properly.

## Radar realism (for the pitch)

- **Breathing signal.** Adult breathing is about 0.2–0.4 Hz with 4–12 mm of chest movement. Heartbeat is much weaker, so we don't claim it.
- **Real penetration.** Getting through rubble needs low-frequency UWB (under about 3 GHz). 24 and 77 GHz mmWave would not go through concrete. Our generator is labelled synthetic, and the slide should say so.
- **Credibility move.** Run the same breathing extractor on the Schellenberger recordings. That shows the DSP works on real signals even though the through-rubble propagation is simulated.
- **Literature for parameters.**
  - Yang et al. 2019, Applied Sciences 9(2):355, on UWB through-wall sensing at low SNR.
  - The RADIOTECT project, aiming at detection through 1–2 m of rubble.
  - Hang et al. 2019 review.

## Acoustic realism

- **Ego-noise.** Rotor noise swamps speech, which is another reason to land.
- **Pipeline.** Listen only with rotors off:
  1. pyroomacoustics simulates a 4-mic array per drone.
  2. A knock or shout clip from DroneAudioSet plays at the hidden survivor position.
  3. Silero VAD and YAMNet classify the sound.
  4. SRP-PHAT gives the bearing.
  5. The cone width comes from the spread of the SRP-PHAT peak.

## Licence traps

- RadarSimPy is GPL-3.0: use it as an offline cross-check only, never in the shipped code.
- FAST-LIO is GPL-2.0 and needs ROS, so avoid it.
- Sketchfab rubble models need a per-model licence check. Some carry NoAI tags.
- ESC-50 is non-commercial.
