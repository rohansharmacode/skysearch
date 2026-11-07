"""Acoustic recognition + priority agent for landed drones (SYNTHETIC scenes).

scene.py  : 3x3 rubble-pile site, landed mic arrays, source synthesis, pyroomacoustics free field
dsp.py    : SRP-PHAT DoA, pile hypothesis tests, triangulation, delay-and-sum beamforming, SNR
models.py : Silero VAD (ONNX), YAMNet (TFLite) + heuristics, faster-whisper ASR
agent.py  : per-clip judge (Claude when ANTHROPIC_API_KEY is set, deterministic fallback otherwise)
run.py    : CLI, writes dashboard/data/acoustic.json
"""
