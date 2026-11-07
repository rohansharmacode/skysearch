# Acoustic recognition + priority agent (SYNTHETIC scenes, CPU only)

Three landed drones (rotors off, 6-mic 0.3 m arrays, 8.6 m around the start pile) listen for 26 s to the
`sim/scenario.py` 3x3 rubble grid. Pipeline: pyroomacoustics free-field synthesis (Piper TTS voices,
numpy knocks/generator/debris, one ESC-50 dog clip, rubble low-pass, wind) → SRP-PHAT per 0.5 s block
(stationary bins masked) → pile hypothesis tests + triangulation → LCMV/delay-and-sum separation →
Silero VAD (ONNX) → YAMNet (TFLite) + knock/hum heuristics → faster-whisper base.en → agent.

    bash acoustic/setup.sh                                   # venv .venv-audio + ~210 MB of models
    .venv-audio/bin/python -m acoustic.run --seed 3 --start-pile B2
    .venv-audio/bin/python -m acoustic.run --seed 3 --drone-xy '{"d1":[0,8],"d2":[-7,-4],"d3":[7,-4]}' --exclude B2
    .venv-audio/bin/python -m pytest tests/test_acoustic.py

Output: `dashboard/data/acoustic.json` (schema in `contracts/INTERFACES.md`), clips in `acoustic/data/clips/`.
Library hook: `from acoustic.run import listen_and_rank; listen_and_rank(sc, drones, exclude_piles, seed)`.

Agent (`agent.py`): per clip it sees only measurements and returns human/urgency/reliability + rationale.
With `ANTHROPIC_API_KEY` set it asks `claude-opus-5-5` (structured JSON, validated, server-side refusal
fallback); otherwise, or on any failure, a deterministic scorer. LLM answers are recorded in
`acoustic/cache/decisions.jsonl` and replayed offline (`--agent replay`). Pile priority and dispatch are code.

Boundary: only `scene.build_scene` reads `scenario.truth`; the eval line in the CLI output is for us, not the agent.
Licences: pyroomacoustics/Silero/faster-whisper MIT, YAMNet Apache-2.0, Piper MIT (lessac voice: see its
model card). Dog bark: ESC-50 clip 1-100032-A-0 (Freesound 100032, K. Piczak, CC BY-NC 3.0): demo only.
