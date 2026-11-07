# Acoustic front end: validation on real audio, and the choice between pipelines

**What this supports:** the recognisers in `acoustic/` work on real recordings. Silero VAD, the YAMNet class groups with the knock/hum heuristics, faster-whisper base.en and the agent's deterministic fallback scorer separate human sounds from non-human sounds and rank them, on real speech, screams, crying, knocks, dogs, engines and ambient noise, when the rotors are off. It also compares our pipeline head-to-head with the teammate pipeline (F. Valisalo, branch `acoustic-voice-detection`) and recommends one.

**What it does not support:** sensing through rubble, or listening with the rotors on. There is no real-data DoA validation, because none of the data has DoA ground truth (see Caveats).

Code under test: commit `eb8cdba` (acoustic BL-3). `models.py`, `agent.py`, `dsp.py`, `run.py` and `scene.py` were frozen into `data/audio_real/code_snapshot/eb8cdba/`. Other agents were editing `acoustic/` while these runs went on, so the snapshot keeps every number on one version. `models.py` + `agent.py` sha1 prefix: `50ad3f88`. The before/after rows use the earlier commit `8eb6d30`.

## Data (all under `data/audio_real/`, gitignored; details and licences in `SOURCE.md`)

| Set | What | n | Licence |
|---|---|---|---|
| ESC-50 | 12 clips x 14 classes. Human: crying_baby, door_wood_knock, coughing, breathing. Non-human: dog, engine, helicopter, chainsaw, wind, rain, crackling_fire, glass_breaking, footsteps, door_wood_creaks | 168 x 5 s | CC BY-NC 3.0 |
| DroneAudioSet, source-only ("rotors off") | Speaker in a room, mono mic1. Programmes File1,3,4,5,6 at 90 dB / 1 m, plus File1 at 8 more loudness/room/distance settings. Cut to 5 s windows inside the annotated segments: male, female (speech, then screams, then "help"/"auxilio" calls), crying, humansounds (knocks, claps, typing), nonhumansounds (rain, horn, phone, glass, vehicle, tap) | 108 + 216 windows | MIT |
| DroneAudioSet, drone-with-source ("rotors on") | The same File1 recorded on the drone. drone1, mic 50 cm. Throttle low at 9 settings, throttle high at 2. SNR -1 to -39 dB | 297 windows | MIT |
| DroneAudioSet, controlled mixes | Source-only windows plus real drone-only rotor noise (same mic) at segment SNR +10, +5, 0, -5, -10, -20 dB (throttle high) and 0, -10 dB (throttle low). ESC-50 and teammate clips mixed the same way | 864 + 504 + 86 | MIT / as above |
| Teammate recordings | Freesound / Wikimedia clips from branch `acoustic-voice-detection`. "help me" calls x13 (two non-English), dog x8, siren x6, machinery x6, radio x5, wind x5. First 14 s of each, the same as `run.extract_clip` | 43 | CC0 |
| ASR segments | The whole ~30 s male/female segments of each programme. Reference = Whisper base.en on the digital source file | 8 per condition | MIT |

About 460 MB was downloaded, 61 MB was copied from the local teammate clone, and 283 MB is on disk (47 MB of that is another agent's `fleurs/`). DroneAudioSet is 42.6 GB of parquet. `validate_real.py` reads each shard's footer by HTTP range request and streams only the audio column bytes up to the wanted row (about 5 MB per mono 150 s row). It uses a minimal pure-numpy parquet reader with libsnappy through ctypes, because pyarrow is not installed. The recording delay against the source file was measured by GCC-PHAT: 0.15-0.44 s where the peak is clear, otherwise 0.3 s is used.

## Method (`acoustic/validate_real.py`)

```
.venv-audio/bin/python -m acoustic.validate_real index | fetch --das           # data
scripts/heavy.sh .venv-audio/bin/python -m acoustic.validate_real run          # ours, resumable, <= 270 s per call
scripts/heavy.sh .venv-audio/bin/python -m acoustic.validate_real run --fv     # teammate classifier rule
scripts/heavy.sh .venv-audio/bin/python -m acoustic.validate_real whatif --whisper-raw
scripts/heavy.sh .venv-audio/bin/python -m acoustic.validate_real synth --family A|B
.venv/bin/python -m acoustic.validate_real report                              # tables, summary.json, docs/acoustic_real_validation.png
```

- **Ours, per clip.** Exactly `run.clip_features`: `vad_speech_ratio`, then `classify(x, vad)`, then `transcribe` when VAD >= 0.15, then `agent.fallback_judge`, then `clip_priority`. These are single-channel clips, so the DoA inputs to the agent are fixed to one neutral geometry (3/3 drones, 1 m triangulation error). `human` and `urgency` therefore come from the recognisers alone.
- **Two "flag human" rules are reported.** *Class*: the YAMNet group is voice, shout or knocking. *Agent*: `human >= 0.4`. The agent rule is the one that decides whether a pile becomes a lead (`agent.HUMAN_LEAD`).
- **Labels.** Positive means speech, scream, help call, crying, knock, cough or breathing, plus DroneAudioSet "humansounds". Negative means everything else.
- **Teammate rule.** Her `VoiceClassifier` logic re-implemented on the same clips with her model file and class lists. Per 2 s block (1 s hop), take the best 0.975 s YAMNet window score of max(human classes) minus max(animal classes). A block is voice at >= 0.2, and a clip is flagged with >= 2 voice blocks (her bearing clustering needs an array). Her `yamnet.tflite` has our size and class map but differs in some bytes.
- **Synthetic head-to-head.** Evaluator only: truth is read after both pipelines have run.
  - **Family A** is our scenario generator (`sim/scenario.make_scenario`) with 8 seeds, each scene having voice, weak voice, knock, dog, generator and settling debris on separate piles. Three landed 6-mic 0.3 m arrays at 8.6 m, 26 s, sensor/wind noise x1, x4 and x10. Ours is the full `run.analyse` (fallback agent). Hers is `detect_voice` per drone on the same signals.
  - **Family B** is her simulator (`simulate_site`): 8 seeds x 3 sites, an 8-mic 0.1 m array, 30 s, 0-2 voices per site (70% her real "help" clips, 30% her synthetic voice), and 1-2 distractors (her dog recordings, generator, engine, birds). Ambient 45, 60 and 70 dB SPL. Hers is her full `detect_voice`. Ours is our front end on one array, with no multi-drone association: CLEAN SRP-PHAT blocks, directions seen in at least 2 blocks, LCMV beams, then the same per-clip chain.
  - A flag counts for a source within 15 deg. Recall covers human sources: in A the voice, weak_voice and knock piles, and in B the voices within 30 m of the site. For ours in A, a source counts as found when its pile has a human lead (pile-level, which is stricter). For hers, at least one drone must point at it.

## Results

### 1. Human-sound detection on clean real audio (rotors off)

| Set (n pos/neg) | Class P / R / F1 | Agent P / R / F1 | Priority AUC |
|---|---|---|---|
| ESC-50 (48/120) | 0.80 / 0.77 / 0.79 | 0.88 / 0.31 / 0.46 | 0.92 |
| DroneAudioSet source-only, 1 m (87/21) | 0.90 / 0.86 / 0.88 | 0.94 / 0.70 / 0.80 | 0.84 |
| DroneAudioSet source-only, 8 other rooms / distances / 60 dB (176/40) | 0.90-1.00 / 0.77-0.95 | 0.95-1.00 / 0.64-0.91 | 0.86-1.00 |
| Teammate clips (13/30) | 0.79 / 0.85 / 0.81 | 0.91 / 0.77 / 0.83 | 0.97 |
| All clean ESC + DAS 1 m (135/141) | 0.87 / 0.83 / 0.85 | 0.93 / 0.56 / 0.70 | 0.93 |

- **What works (panel a).** On ESC-50, dog, engine, chainsaw and helicopter clips are never flagged (0/48), and glass and footsteps are flagged 0/24. Speech is flagged on 89% of windows, rotors off. Crying is flagged on 70-75%. When crying is missed, YAMNet calls it `dog` (5/20 DroneAudioSet windows, 2/12 ESC clips).
- **What is weak.** At the agent level, non-vocal human sounds are mostly missed. Real door knocks are flagged on only 2/12 clips even though 10/12 are classed `knocking`. Coughing is flagged 1/12, breathing 3/12, and DroneAudioSet humansounds 5/20. The agent needs `human >= 0.4`, and on knocks the class score alone does not reach it.
- **Before and after the BL-3 knock heuristic (`8eb6d30` -> `eb8cdba`, same clips).** ESC knock recall fell from 0.83 to 0.17. DroneAudioSet humansounds fell from 0.60 to 0.25. In exchange, ESC debris false flags (glass, creaks) fell from 0.50 to 0.04 and footsteps from 0.25 to 0.00. ESC agent precision rose from 0.62 to 0.88 and AUC from 0.85 to 0.92, but F1 fell from 0.60 to 0.46. The stricter knock grouping removes false positives and real knocks together.

### 2. Under rotor noise, against SNR (panel b)

| Condition | Class R | Agent P / R | Priority AUC |
|---|---|---|---|
| DroneAudioSet + throttle-high noise, +10 / 0 / -10 / -20 dB | 0.78 / 0.71 / 0.57 / 0.39 | 0.91/0.49, 0.91/0.33, 0.67/0.11, 0.69/0.13 | 0.80 / 0.76 / 0.60 / 0.40 |
| DroneAudioSet + throttle-low noise, 0 / -10 dB | 0.45 / 0.00 | 0.96/0.30, -/0.00 | 0.80 / 0.58 |
| ESC-50 + throttle-high noise, +10 / 0 / -10 dB | 0.79 / 0.67 / 0.46 | 0.78/0.44, 0.64/0.15, 0.40/0.08 | 0.89 / 0.81 / 0.64 |
| Real rotors-on, 90 dB source, throttle low (SNR -1 to -10 dB, 6 settings) | 0.18-0.45 | recall 0.00-0.18 | 0.26-0.70 |
| Real rotors-on, 60 dB source or throttle high (SNR -15 to -39 dB) | | recall 0.00-0.32 | 0.13-0.61 |

- **The bottleneck under noise is the agent, not YAMNet.** At 0 dB the YAMNet group is still right for 71% of human windows, but only 33% reach `human >= 0.4`. The fallback scales speech evidence by VAD (`0.5 + 0.5 * min(1, vad / 0.3)`), and Silero's speech ratio collapses under rotor noise. It is 0.00 at -10 dB for every segment.
- **Rotor noise imitates knocks.** At 0 dB, 15/20 crying windows and 6/21 non-human windows become `knocking`, because blade-pass pulses are periodic.
- **Real rotors-on audio fails, as expected.** Real drone-with-source recordings fall on or below the controlled-mix curves. This supports the design decision to land and stop the rotors before listening.

### 3. Speech recognition (panel c)

| Condition | Segments passing VAD gate | Median segment WER vs clean-source transcript | "help" recovered (segments whose reference says it) |
|---|---|---|---|
| Rotors off, 90 dB / 1 m | 8/8 | 0.28 | 1/5 |
| + rotor noise +10 / +5 / 0 dB | 8/8, 8/8, 6/8 | 0.67 / 0.73 / 1.05 | 2/5, 1/5, 0/5 |
| + rotor noise <= -5 dB, real rotors-on | 0-2/8 | 1.00 (empty) | 0 |
| Teammate "help me" clips, clean (13) | 7/13 | n/a | gated 2/13, ungated 6/13 |

- **WER is measured against an ASR reference.** The reference is Whisper base.en on the digital source, so WER measures degradation, not absolute accuracy. Scream tokens ("ah", "aah") are dropped and immediate repeats collapsed before scoring. Pooled WER is dominated by Whisper repetition loops ("Thank you. Thank you. ..."), so the table uses median segment WER.
- **The VAD gate drops shouted calls.** Silero gives a speech ratio of 0.00-0.09 on 6 of the 13 shouted "help me" clips, so they are never transcribed. Transcribing every clip finds "help" in 6/13 instead of 2/13. That is closer to 6 of the 11 English-titled clips, since two titles are French/Spanish.
- **Without the confidence filter, Whisper invents text.** With rotor noise at 0 dB it gives fluent, unrelated text. One 0 dB segment came back as "I'm going to try to get out of here" repeated. The confidence filter in `transcribe` exists for good reason.

### 4. Agent ranking (panel d)

The fallback scorer's clip priority ranks human above non-human with AUC 0.92 on ESC-50, 0.84 on DroneAudioSet rotors off (0.86-1.00 in the 8 other settings) and 0.97 on the teammate clips. With rotor noise it falls to 0.76 at 0 dB and 0.60 at -10 dB. The top of the ranking is clean: a human sound with keywords scores up to 52-60, and no non-human clip scores above 33.

### 5. Direction of arrival

There is no real-data result. DroneAudioSet gives speaker distances but no azimuths, and its 8-channel arrays have no published geometry in the dataset card. DREGON has DoA ground truth but was not downloaded (multi-GB zips); see Requests. The synthetic results below are the only DoA numbers.

## Head-to-head: ours vs the teammate pipeline

| | Ours (`acoustic/`) | Teammate (`acoustic/acoustic_pipeline.py`, branch `acoustic-voice-detection`) |
|---|---|---|
| DoA | CLEAN SRP-PHAT, 300-4000 Hz, 0.5 s blocks. Several sources per block, sidelobes subtracted | SRP-PHAT, 300-2000 Hz, 5 deg grid, per-bin noise-floor weighting. One direction per 2 s block |
| Multi-drone | Bearings from 3 drones associated to piles, triangulation, ambiguity flags, cross-talk removal | Per site only: bearing flags, no pile assignment |
| Separation | LCMV beams with nulls on the other sources | Delay-and-sum toward the single strongest direction |
| Recognition | Silero VAD, YAMNet 7 groups + knock and hum heuristics, Whisper ASR + keywords | YAMNet only: max(human) minus max(animal) >= 0.2 |
| Output | Per-pile priority 0-100 with human, urgency, reliability and rationale (LLM or fallback) | Bearing flags, "high" priority at >= 4 voice blocks |

**Same real clips (single channel; F1 for human vs non-human, panel e):**

| Set | Ours agent P / R / F1 | Teammate P / R / F1 | AUC ours / teammate |
|---|---|---|---|
| Teammate clips, clean | 0.91 / 0.77 / **0.83** | 0.73 / 0.85 / 0.79 | **0.97** / 0.89 |
| Teammate clips + rotor 0 dB | 0.78 / 0.54 / **0.64** | 0.50 / 0.31 / 0.38 | **0.90** / 0.77 |
| ESC-50 | 0.88 / 0.31 / 0.46 | 0.84 / 0.33 / **0.48** | **0.92** / 0.88 |
| ESC-50 + rotor 0 dB | 0.64 / 0.15 / **0.24** | 0.00 / 0.00 / - | **0.81** / 0.62 |
| DroneAudioSet rotors off | 0.94 / 0.70 / 0.80 | 1.00 / 0.74 / **0.85** | 0.84 / **0.91** |
| DroneAudioSet + rotor 0 dB | 0.91 / 0.33 / 0.49 | 1.00 / 0.14 / 0.24 | 0.76 / **0.83** |
| DroneAudioSet rotors on, -1 dB | 0.80 / 0.18 / 0.30 | 1.00 / 0.27 / **0.43** | 0.57 / **0.82** |

Where each one wins:
- **Teammate.** Higher recall on clean speech and crying: 0.92-1.00 against 0.70-0.96. Better priority ranking on DroneAudioSet speech under noise.
- **Ours.** Never flags dogs. Hers flags 3/8 of her own dog clips. Ours also degrades more gracefully when rotor noise is mixed into short clips.
- **Knocks and other non-vocal sounds.** Hers ignores them by design (DroneAudioSet humansounds 0/20). Ours catches some.

**Synthetic scenes (panel f):**

| Scenes | Pipeline | Human recall | Voice recall | Precision | False flags / scene | Bearing error median / p90 | Real-time factor |
|---|---|---|---|---|---|---|---|
| A (ours), 24 scenes, noise x1/x4/x10 | ours | 0.61 (n=72) | 0.85 (n=48) | **0.96** | **0.08** | **0.3 / 1.4 deg** | 0.71 |
| | teammate | **0.69** | **0.88** | 0.80 | 0.96 | 1.6 / 7.7 deg | **0.13** |
| A, noise x10 only | ours | 0.42 | 0.56 | 1.00 | 0.00 | 0.2 / 0.8 | 0.32 |
| | teammate | **0.79** | **1.00** | 0.82 | 0.88 | 1.6 / 6.7 | 0.13 |
| B (hers), 72 sites, ambient 45/60/70 dB | ours (one array, no association) | 0.52 (n=75) | 0.52 | 0.54 | 0.71 | 2.4 / 8.5 | 0.30 |
| | teammate | 0.49 | 0.49 | **0.67** | **0.31** | 2.2 / 5.0 | **0.05** |
| B, 70 dB only | ours | 0.36 | 0.36 | 0.40 | 0.62 | 3.4 / 10.8 | 0.20 |
| | teammate | **0.56** | **0.56** | 0.68 | 0.29 | 2.6 / 6.1 | 0.05 |

- **Precision and DoA, on our 3-drone scenes.** Ours raises about 10x fewer false human flags: 2 in 24 scenes against 23, with generator, dog and settling debris called "voice". Its bearings are about 5x tighter, and it assigns every lead to a pile.
- **Where ours fails.** Knocks: 3/24 knock piles end up as a human lead, because rubble-filtered knocks are classed `engine` or `noise`. Heavy noise: at x10, 6/8 weak voices produce no detection at all. That is a localisation failure, not a classification one. The teammate's per-bin noise-floor weighting keeps finding them, as bearings.
- **On the teammate's own scenes.** Our front end without its multi-drone association has the same recall but more false flags. These come from aliased directions on the 0.1 m array, where our band reaches 4 kHz but the array aliases above 2.2 kHz.
- **Runtime.** On 2 CPU threads both are faster than real time. Hers is 5-7x cheaper (no Whisper, no CLEAN). Ours takes 15-33 s for a 26 s, 3-drone listen.

## Caveats

- **Rotors off is not the same as through rubble.** Source-only DroneAudioSet audio is a loudspeaker in a room at 1-9 m. Nothing here has passed through rubble. The burial low-pass and attenuation remain simulated (`scene.through_rubble`). The ESC-50 and teammate clips are near-field Freesound recordings.
- **Rotor noise is real, but the mixing is additive.** The controlled mixes add real recorded drone noise (drone1, mic 50 cm) at chosen SNRs. The landed drone in our design has its rotors off, so these rows measure robustness, not the operating point. The real rotors-on rows use one drone and one mic.
- **Single channel.** All real-data classification uses one mic. Our 6-mic beamformer would add up to about 7.8 dB of array gain in diffuse noise.
- **Small n.** There are 12 clips per ESC class and 13 voice clips. The DroneAudioSet windows come from six programmes, so many windows share a speaker and a sentence, and only 8 ASR segments are distinct. Differences of 0.05-0.10 in F1 are within noise.
- **Labels are coarse.** "humansounds" mixes knocks, claps and typing. Cough and breathing were counted as human, which is our choice. Our agent instructions mention speech, crying and knocking only.
- **The ASR reference is itself ASR.** The help-keyword denominators are small (5 segments).
- **The teammate rule is re-implemented on single clips, without her bearing clustering.** Family B runs our front end without its main strength, multi-drone association.

## Recommendation

**Make `acoustic/` (ours) the canonical pipeline.** The mission needs pile-level, assigned and ranked leads with few false alarms. On our 3-drone scenes ours has precision 0.96 against 0.80, 0.08 false flags per scene against 0.96, and a 0.3 deg median bearing error against 1.6 deg. On the real teammate clips it has F1 0.83 against 0.79 and AUC 0.97 against 0.89, and it does not mistake dogs for people. It is also the only pipeline that does pile association, multi-source separation, knocks, speech content (help, trapped, water) and urgency. The teammate pipeline outputs bearings per site and would need all of that rebuilt.

Port these from the teammate pipeline, each with evidence:

1. **Her human-minus-animal YAMNet score, as an extra agent feature and an alternative path to "human" when VAD is low.**
   - Evidence: AUC on DroneAudioSet + rotor 0 dB of 0.83 against our 0.76. On real rotors-on audio at -1 dB, 0.82 against 0.57. On clean speech and crying, recall of 0.92-1.00 against 0.70-0.96.
   - How: max over 0.975 s windows of max(human) - max(animal). Her human list adds Baby cry, Wail/moan, Groan, Bellow and Whoop. Adding those classes to our voice/shout groups alone changed nothing (what-if `human_plus`). The gain comes from the max-window, minus-animal scoring.
2. **Her per-bin noise-floor weighting (10th percentile x 4 margin) as a soft weight in our SRP masking (`dsp.gcc_blocks`).**
   - Evidence: at noise x10 on our scenes she still finds 16/16 voices as bearings, while 6/8 of our weak voices are never localised.
3. **Not worth porting.**
   - Her YAMNet TFLite path: same model, same `ai-edge-litert` runtime.
   - Her single-direction-per-block DoA: it cannot separate concurrent sources and has 5x worse p90 bearing error.
   - Her 300-2000 Hz band: it suits her 0.1 m array, not our 0.3 m one.

## Requests (for the owners of `acoustic/models.py` / `acoustic/agent.py`; not edited here)

1. **Agent VAD gate under noise (`agent.fallback_judge`).**
   - Problem: speech evidence is scaled by VAD, so under rotor noise most correctly classified human clips fall below 0.4.
   - Option: also flag human when the YAMNet group is voice, shout or knocking.
   - Measured from the current results (agent F1, then the alternative):
     - DroneAudioSet + rotor 0 dB: 0.49 -> 0.81, +10 dB: 0.64 -> 0.84, rotors off: 0.80 -> 0.88.
     - ESC-50: 0.46 -> 0.79.
     - Real rotors-on -1 dB: 0.30 -> 0.61.
     - Teammate clips: 0.83 -> 0.81, precision 0.91 -> 0.79.
   - Risk: rotor noise makes the knocking class fire, so ESC + rotor 0 dB precision is 0.64 -> 0.58. Check with `acoustic.eval_assign` before adopting.
2. **Knock heuristic regression (`models.knock_periodicity`, BL-3 knock grouping).**
   - Real door-knock recall fell from 0.83 to 0.17 and DroneAudioSet humansounds from 0.60 to 0.25.
   - A smaller option: a floor when `sound_class == "knocking"`. Agent F1 on ESC 0.46 -> 0.62 and on DroneAudioSet rotors off 0.80 -> 0.88, with ESC precision 0.88 -> 0.83 and no precision change on DroneAudioSet.
   - Re-tune with `validate_real run` (about 1 min on the clean sets).
3. **Whisper gate.** `VAD >= 0.15` skips 6/13 shouted "help me" clips (VAD 0.00-0.09). Gate on VAD or YAMNet voice/shout instead. Keep the logprob/no-speech filter: without it, 0 dB rotor noise produces fluent invented text.
4. **Port items 1 and 2 of the Recommendation into `models.py` / `dsp.py`.** Re-run `validate_real` and `eval_assign` afterwards.
5. **DoA on real data.** Approve downloading a DREGON subset (Inria, 8-mic cube with DoA ground truth; a few hundred MB if only the "speech, source static" recordings are taken). That would give a real-data DoA error.
6. **`data/audio_real/fleurs/` (47 MB).** Written by `acoustic/eval_gemini fetch`, not by this task. Its SOURCE.md is there. It is listed here so the data owner knows.
