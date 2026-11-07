#!/usr/bin/env bash
# Store the Gemini API key in .env (gitignored, chmod 600) without echoing it,
# then make one tiny test call to confirm the key works. The key never appears in
# the terminal, shell history, command arguments or git.
#   bash scripts/set_gemini_key.sh
set -euo pipefail
cd "$(dirname "$0")/.."
read -rsp "Paste your Gemini API key (input hidden), then press Enter: " KEY; echo
[ -n "$KEY" ] || { echo "No key entered; nothing changed."; exit 1; }
touch .env && chmod 600 .env
grep -v '^GEMINI_API_KEY=' .env > .env.tmp || true
printf 'GEMINI_API_KEY=%s\n' "$KEY" >> .env.tmp && mv .env.tmp .env && chmod 600 .env
unset KEY
git check-ignore -q .env && echo "Saved to $(pwd)/.env (gitignored)." || echo "WARNING: .env is not gitignored here; don't commit it."
# Test call: reads the key from .env inside Python, sends it as a header (not visible in ps).
python3 - <<'EOF'
import json, os, urllib.request
key = next((l.split("=", 1)[1].strip() for l in open(".env") if l.startswith("GEMINI_API_KEY=")), "")
model = os.environ.get("GEMINI_MODEL", "gemini-3.5-flash")
req = urllib.request.Request(
    f"https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent",
    data=json.dumps({"contents": [{"role": "user", "parts": [{"text": "Reply with the single word OK."}]}],
                     "generationConfig": {"maxOutputTokens": 50}}).encode(),
    headers={"Content-Type": "application/json", "x-goog-api-key": key}, method="POST")
try:
    with urllib.request.urlopen(req, timeout=30) as r:
        out = json.load(r)
    text = "".join(p.get("text", "") for p in out["candidates"][0]["content"]["parts"] if not p.get("thought"))
    print(f"Gemini key works (model {model}): {text.strip()[:40]!r}")
except Exception as e:
    print(f"Saved, but the test call failed ({type(e).__name__}: {str(e)[:120]}). "
          f"Check the key, or set GEMINI_MODEL to a model your key can use.")
EOF
