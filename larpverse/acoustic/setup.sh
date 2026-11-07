#!/usr/bin/env bash
# One-off setup for the acoustic pipeline: separate py3.12 venv + small CPU models (~210 MB).
set -euo pipefail
cd "$(dirname "$0")/.."
uv venv .venv-audio --python 3.12 -q
nice -n 19 uv pip install --python .venv-audio numpy scipy pyroomacoustics onnxruntime faster-whisper \
  piper-tts soundfile anthropic pytest ai-edge-litert
M=acoustic/models; D=acoustic/data/esc50; mkdir -p "$M" "$D"
get() { [ -s "$2" ] || curl -sSLf -o "$2" "$1"; }
HF=https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/low
get $HF/en_US-lessac-low.onnx "$M/en_US-lessac-low.onnx"
get $HF/en_US-lessac-low.onnx.json "$M/en_US-lessac-low.onnx.json"
get https://raw.githubusercontent.com/snakers4/silero-vad/master/src/silero_vad/data/silero_vad.onnx "$M/silero_vad.onnx"
get https://storage.googleapis.com/download.tensorflow.org/models/tflite/task_library/audio_classification/android/lite-model_yamnet_classification_tflite_1.tflite "$M/yamnet.tflite"
get https://raw.githubusercontent.com/tensorflow/models/master/research/audioset/yamnet/yamnet_class_map.csv "$M/yamnet_class_map.csv"
get https://raw.githubusercontent.com/karolpiczak/ESC-50/master/audio/1-100032-A-0.wav "$D/1-100032-A-0.wav"
echo "ok (faster-whisper base.en downloads on first run into $M/whisper)"
