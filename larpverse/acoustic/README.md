# Acoustic recognition + priority agent (SYNTHETIC scenes, CPU only)

Three landed drones (rotors off, 6-mic 0.3 m arrays, 8.6 m around the start pile) listen for 26 s to the
`sim/scenario.py` 3x3 rubble grid. Pipeline: pyroomacoustics free-field synthesis (Piper TTS voices,
numpy knocks/generator/debris, one ESC-50 dog clip, rubble low-pass, wind) → GCC/SRP-PHAT per 0.5 s block
(stationary bins masked) + CLEAN sidelobe removal → multi-drone bearing association to piles → LCMV
separation with nulls on every other source (visited piles included) → cross-talk removal → Silero VAD
(ONNX) → YAMNet (TFLite) + knock/hum heuristics → faster-whisper base.en (gated) → agent.

    bash acoustic/setup.sh                                   # venv .venv-audio + ~210 MB of models
    .venv-audio/bin/python -m acoustic.run --seed 3 --start-pile B2
    .venv-audio/bin/python -m acoustic.run --seed 3 --drone-xy '{"d1":[0,8],"d2":[-7,-4],"d3":[7,-4]}' --exclude B2
    .venv-audio/bin/python -m pytest tests/test_acoustic.py

Output: `dashboard/data/acoustic.json` (schema in `contracts/INTERFACES.md`), clips in `acoustic/data/clips/`.
Library hook: `from acoustic.run import listen_and_rank; listen_and_rank(sc, drones, exclude_piles, seed)`.

Pile assignment (`dsp.localise_piles`, ALGO_VERSION in `dsp.py`):
- CLEAN per block and drone: take the SRP-PHAT peak, subtract the array's point-spread function for that
  direction (same frequency weighting), repeat. Sidelobes / leakage of a loud source are not new peaks.
- Association: each block's components are matched greedily and exclusively to piles. A pile needs >= 2
  drones within the gate (measurement sigma 3.5 deg + the pile's angular extent), or one strong drone with
  no other pile on that bearing. Visited / excluded piles compete too, so their sources explain their own
  energy (and are nulled during separation) instead of landing on a neighbour.
- Per pile: bearing likelihood over all nine piles (`association_conf`), share of blocks where a rival pile
  has >= 0.5 of the joint bearing likelihood on the same components, share of the clip dominated by
  another source. Below 0.6, or not the best pile, the clip is `association: "ambiguous"`: reliability is capped at 0.45, priority at 19 (below
  the mission's dispatch threshold) and a human lead reads
  "Lead (ambiguous direction ...)". Association confidence scales reliability (x 0.5-1.0) for every clip.
- Cross-talk: speech blocks in a clip that a talker elsewhere owns (same time, more speech in its own beam)
  are cut and the rest re-recognised; Whisper text is kept only with VAD >= 0.3 (>= 0.5 for non-speech
  classes), `no_speech_prob` < 0.6 and `avg_logprob` > -1.0; a transcript repeating another pile's
  talker (>= 2 shared word bigrams) is dropped and the clip becomes ambiguous; a voice-class clip with VAD < 0.3 is ambiguous
  (sparse speech is usually a leak). Knock onsets that coincide (+-80 ms) with a cleaner knocker's are
  cut the same way. Each clip lists what was done in `notes`.

Novelty -> Gemini audio stage (`acoustic/novelty.py`, `acoustic/gemini_audio.py`, another builder's code):
`run.analyse` hands it the separated (cross-talk-cleaned) clips with their association and reliability
and merges the result (`gemini_audio.merge_into_state`). On by default exactly when `agents.llm` resolves
to Gemini with a key (or `LLM_REPLAY=1`); `--gemini-audio on|off` or `LARPVERSE_GEMINI_AUDIO=1|0`
override, `--agent fallback` turns it off. Answers are cached/replayed by `agents.llm`; any failure keeps
the v1 ranking. Ambiguous piles keep their priority cap after stage 3.

Knocks: `models.knock_periodicity` = max(group score: >= 2 evenly spaced knock groups with the same
spacing +-15 %, one clear of debris bursts; rhythm score: onset-strength autocorrelation at 0.12-0.45 s,
ignored when YAMNet hears an engine or 50/60 Hz harmonics are present). A clear pattern without 50/60 Hz harmonics keeps YAMNet's "engine"
below "knocking". With VAD < 0.1 the headline class is never voice/shout unless YAMNet's
human-minus-animal score (teammate F. Valisalo's rule, `heuristics.human_minus_animal`) is >= 0.2.

Human flag (real-audio validation, docs/ACOUSTIC_VALIDATION.md): `run.class_human_floor` lifts a
fallback judgment to human 0.45 when YAMNet's class is voice/shout (no dog score) or knocking with a
knock pattern (>= 0.2). Whisper runs when VAD >= 0.15 or the class is voice/shout or human-minus-animal
>= 0.2 (shouted calls that Silero misses). Real clips, agent F1 eb8cdba -> now (same manifest, neutral
DoA): ESC-50 0.46 -> 0.70 (door-knock recall 0.17 -> 0.67), teammate clips 0.83 -> 0.81, with real
rotor noise mixed at 0 dB: ESC 0.24 -> 0.28, teammate 0.64 -> 0.69, DroneAudioSet 0.49 -> 0.58.

Rotors (Request 0, docs/ACOUSTIC_VALIDATION.md section 6): `listen_and_rank(..., rotors={id: (x, y)})`
takes our own drones hovering during the listen (public mission state). CLEAN components within 8 deg of
a hovering drone's bearing are skipped before association, and those bearings are nulled in every beam.
`run.rotor_veto` (inside `class_human_floor`) caps human at 0.3 for a voice/shout/knocking clip that
sounds like a rotor (`heuristics.steadiness` >= 0.3, or a `blade_pass` comb >= 0.25 on continuous sound)
without independent speech evidence (VAD < 0.3, human-minus-animal < 0.4). `scene.build_scene(rotors=)`
synthesises blade-pass noise for testing; `python -m acoustic.run --rotor-xy '{"d4": [x, y]}'`.

Weak talkers and live sources: CLEAN keeps components down to 0.08 x the strongest in a block, but one
below 0.2 x may only support a pile that every drone agrees on (a weak caller under a loud one survives,
drone-specific residual ghosts do not). `build_scene(sources=[...])` / `listen_and_rank(sources=...)`
take live dummies (isaac/live.py, assets/audio/dummies) in place of the scenario's sounds; regression:
`tests/fixtures/acoustic_dummy_seed7.json` (seed 7, hidden C3 caller, dog over the knocks).

Latency (`state["timing_s"]`, per stage in `state["timing_stages_s"]`): pyroomacoustics output via one FFT
per source and one inverse FFT per mic (`scene.fast_simulate`, identical to `room.simulate()`), LCMV
weights solved on a 1025-bin grid and interpolated (0.03 % from the per-bin solve), per-drone FFT and
delay-and-sum beam caches, Whisper only on voice/shout clips and their loudest 12 s, Gemini only on
voice/shout clips. Hidden-C3 dummy listen on this laptop (warm, models loaded once): 43 s -> 15.8 s for
26 s of audio, 7.2 s with `listen_and_rank(..., duration=13)`.

Evaluation (reads truth, evaluator only): `acoustic/eval_assign.py` replays the numpy grid mission with
the real pipeline and scores per-pile assignment over every listen, one heavy job per seed:

    scripts/heavy.sh .venv-audio/bin/python -m acoustic.eval_assign run --seeds 7
    .venv-audio/bin/python -m acoustic.eval_assign report        # v1-baseline vs current ALGO_VERSION
    # same listens as a cached reference run (mission landing spots change between sim versions)
    scripts/heavy.sh .venv-audio/bin/python -m acoustic.eval_assign run --replay v1-baseline --seeds 7

Agent (`agent.py`, owned by the agents/ builder): per clip it sees only measurements and returns
human/urgency/reliability + rationale through `agents/llm.py`: Gemini when `GEMINI_API_KEY` is set, else
Claude when `ANTHROPIC_API_KEY` is set (`LLM_PROVIDER` overrides; keys may live in `.env`); otherwise, or on
any failure, a deterministic scorer. LLM answers are recorded in `acoustic/cache/decisions.jsonl` and
replayed offline (`--agent replay`). Pile priority and dispatch are code.

Boundary: only `scene.build_scene` reads `scenario.truth`; the eval line in the CLI output is for us, not the agent.
Licences: pyroomacoustics/Silero/faster-whisper MIT, YAMNet Apache-2.0, Piper MIT (lessac voice: see its
model card). Dog bark: ESC-50 clip 1-100032-A-0 (Freesound 100032, K. Piczak, CC BY-NC 3.0): demo only.
