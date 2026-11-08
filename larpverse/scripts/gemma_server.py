"""Gemma 4 on the drone: a tiny local chat server for agents/llm.py (provider "gemma").

Stands in for the model running on each drone's own computer (Gemma 4 E2B: open weights, Apache 2.0,
native audio input), so the drones hear and decide with no network. Same machine as the sim; GPU.

    python3 scripts/gemma_server.py                      # google/gemma-4-E2B-it on 127.0.0.1:8090
    LLM_PROVIDER=gemma ...                               # then every agent call goes to it

API (OpenAI chat format, the subset llm._gemma sends):
    POST /v1/chat/completions {model, messages:[{role, content: str | [{type:"text",text} | {type:"input_audio",
         input_audio:{data:<base64 wav>, format:"wav"}}]}], max_tokens}
      -> {choices:[{message:{role:"assistant", content}}], usage:{prompt_tokens, completion_tokens}}
    GET /health -> {ok, model, device}
One request at a time (a lock); greedy decoding. Needs: pip install -U transformers accelerate soundfile (torch with CUDA).
"""
from __future__ import annotations

import base64
import json
import os
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MODEL_ID = os.environ.get("GEMMA_HF_ID", "google/gemma-4-E2B-it")
HOST, PORT = os.environ.get("GEMMA_HOST", "127.0.0.1"), int(os.environ.get("GEMMA_PORT", "8090"))
MAX_NEW = int(os.environ.get("GEMMA_MAX_NEW_TOKENS", "768"))

_lock = threading.Lock()
proc = model = None


def load():
    global proc, model
    import torch
    from transformers import AutoProcessor
    try:
        from transformers import AutoModelForMultimodalLM as Auto
    except ImportError:                                   # older transformers naming
        from transformers import AutoModelForImageTextToText as Auto
    t0 = time.time()
    proc = AutoProcessor.from_pretrained(MODEL_ID)
    model = Auto.from_pretrained(MODEL_ID, dtype=torch.bfloat16, device_map="cuda" if torch.cuda.is_available() else "cpu")
    model.eval()
    print(f"[gemma] loaded {MODEL_ID} on {model.device} in {time.time() - t0:.0f} s", flush=True)


def _parts(content, tmp):
    """OpenAI content -> Gemma chat-template parts (text first, audio after, as the model card shows)."""
    if isinstance(content, str):
        return [{"type": "text", "text": content}]
    text, audio = [], []
    for p in content or []:
        if p.get("type") == "text":
            text.append({"type": "text", "text": p.get("text", "")})
        elif p.get("type") == "input_audio":
            a = p.get("input_audio") or {}
            path = os.path.join(tmp, f"clip{len(audio)}.{a.get('format') or 'wav'}")
            with open(path, "wb") as fh:
                fh.write(base64.b64decode(a.get("data") or ""))
            audio.append({"type": "audio", "audio": path})
    return text + audio


def generate(body):
    import torch
    with tempfile.TemporaryDirectory() as tmp:
        msgs = []
        for m in body.get("messages") or []:
            if m.get("role") == "system":
                c = m.get("content")
                msgs.append({"role": "system", "content": c if isinstance(c, str) else " ".join(
                    p.get("text", "") for p in c or [] if p.get("type") == "text")})
            else:
                msgs.append({"role": m.get("role", "user"), "content": _parts(m.get("content"), tmp)})
        inputs = proc.apply_chat_template(msgs, tokenize=True, return_dict=True, return_tensors="pt",
                                          add_generation_prompt=True).to(model.device)
        n_in = inputs["input_ids"].shape[-1]
        with torch.inference_mode():
            out = model.generate(**inputs, max_new_tokens=min(int(body.get("max_tokens") or MAX_NEW), MAX_NEW),
                                 do_sample=False)
        raw = proc.decode(out[0][n_in:], skip_special_tokens=False)
        text = None
        try:                                              # strips any thinking channel, per the model card
            parsed = proc.parse_response(raw, prefix=inputs["input_ids"])
            text = parsed.get("content") if isinstance(parsed, dict) else (parsed if isinstance(parsed, str) else None)
        except Exception:                                 # noqa: BLE001
            pass
        if not text:
            text = proc.decode(out[0][n_in:], skip_special_tokens=True)
        return text.strip(), n_in, int(out.shape[-1] - n_in)


class H(BaseHTTPRequestHandler):
    def _send(self, code, obj):
        b = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        if self.path.startswith("/health"):
            return self._send(200, {"ok": model is not None, "model": MODEL_ID, "device": str(getattr(model, "device", "-"))})
        self._send(404, {"error": "not found"})

    def do_POST(self):
        if not self.path.startswith("/v1/chat/completions"):
            return self._send(404, {"error": "not found"})
        try:
            body = json.loads(self.rfile.read(int(self.headers.get("Content-Length") or 0)) or b"{}")
            t0 = time.time()
            with _lock:
                text, n_in, n_out = generate(body)
            print(f"[gemma] {n_in} in / {n_out} out tokens, {time.time() - t0:.1f} s", flush=True)
            self._send(200, {"model": MODEL_ID, "choices": [{"index": 0, "message": {"role": "assistant", "content": text},
                                                             "finish_reason": "stop"}],
                             "usage": {"prompt_tokens": n_in, "completion_tokens": n_out}})
        except Exception as e:                            # noqa: BLE001
            self._send(500, {"error": f"{type(e).__name__}: {str(e)[:300]}"})

    def log_message(self, *a):
        pass


if __name__ == "__main__":
    load()
    print(f"[gemma] serving on http://{HOST}:{PORT}/v1/chat/completions", flush=True)
    ThreadingHTTPServer((HOST, PORT), H).serve_forever()
