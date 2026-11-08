#!/usr/bin/env python3
"""Check the Gemini key in .env actually works and isn't rate-limited (never prints the key).

    python3 scripts/gemini_check.py            # 8 quick calls per demo model, reports OK / 429 / errors + latency
    python3 scripts/gemini_check.py --n 15     # heavier burst (each call uses a little quota)
"""
import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request

MODELS = [os.environ.get("GEMINI_MODEL", "gemini-3.7-flash"),            # decider + breathing analysis
          os.environ.get("GEMINI_AUDIO_MODEL", "gemini-3.5-flash-lite")]  # audio distress analysis


def load_key():
    if os.environ.get("GEMINI_API_KEY"):
        return os.environ["GEMINI_API_KEY"].strip()
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
    try:
        for line in open(os.path.join(root, ".env")):
            line = line.strip()
            if line.startswith("export "):
                line = line[7:]
            if line.startswith("GEMINI_API_KEY") and "=" in line:
                return line.split("=", 1)[1].strip().strip('"').strip("'")
    except FileNotFoundError:
        pass
    return None


def call(key, model):
    body = {"contents": [{"parts": [{"text": "Reply with the single word OK."}]}],
            "generationConfig": {"maxOutputTokens": 8}}
    req = urllib.request.Request(
        f"https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent",
        data=json.dumps(body).encode(), headers={"Content-Type": "application/json", "x-goog-api-key": key})
    t = time.time()
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            json.load(r)
        return 200, time.time() - t, ""
    except urllib.error.HTTPError as e:
        msg = e.read().decode(errors="replace")
        try:
            msg = json.loads(msg)["error"]["message"]
        except Exception:  # noqa: BLE001
            pass
        return e.code, time.time() - t, msg[:240]
    except Exception as e:  # noqa: BLE001
        return 0, time.time() - t, f"{type(e).__name__}: {e}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=8, help="calls per model (sent back to back)")
    a = ap.parse_args()
    key = load_key()
    if not key:
        print("NO KEY: run  bash scripts/set_gemini_key.sh  first (hidden prompt, saves to .env).")
        return 2
    print(f"key found (length {len(key)}, ends ...{key[-4:]}); {a.n} back-to-back calls per model\n")
    verdict = 0
    for m in MODELS:
        res = [call(key, m) for _ in range(a.n)]
        ok = sum(c == 200 for c, _, _ in res)
        rl = sum(c == 429 for c, _, _ in res)
        lat = sorted(t for c, t, _ in res if c == 200)
        med = f"{lat[len(lat) // 2]:.2f}s" if lat else "-"
        print(f"{m:26s} OK {ok}/{a.n}   429 rate-limited {rl}   median latency {med}")
        for c, _, msg in res:
            if c != 200:
                print(f"    first error HTTP {c}: {msg}")
                break
        if ok < a.n:
            verdict = 1
    print("\nVERDICT:", "PASS - key works with no rate limiting at this burst" if verdict == 0 else
          "PROBLEM - see errors above (429 'quota'/'per day' = free tier or daily cap; enable billing on the key's "
          "Google Cloud project, or use a key from a billed project)")
    return verdict


if __name__ == "__main__":
    sys.exit(main())
