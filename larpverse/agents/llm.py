"""Tiny provider layer: one structured-JSON call, Gemini first, never raises.

    from agents import llm
    out = llm.call_json(prompt, schema, system="...")      # dict, or None on any failure

Provider: ``provider=`` argument, else env LLM_PROVIDER (gemini | anthropic | none), else gemini if
GEMINI_API_KEY is set, else anthropic if ANTHROPIC_API_KEY is set, else none. Keys are read from the
environment or from a local ``.env`` at the repo root (gitignored); they are never printed or cached.

Record/replay: every successful answer is appended to agents/cache/llm.jsonl keyed by a hash of
provider + model + system + prompt + schema. A later identical call is answered from the cache
(free, offline, deterministic). ``LLM_REPLAY=1`` forces replay-only: cache hit or None, no network.

Stdlib only (urllib), no SDKs.
"""
from __future__ import annotations

import hashlib
import json
import os
import time
import urllib.error
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CACHE_PATH = os.path.join(ROOT, "agents", "cache", "llm.jsonl")
GEMINI_URL = "https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent"
ANTHROPIC_URL = "https://api.anthropic.com/v1/messages"
DEFAULT_MODELS = {"gemini": "gemini-3.5-flash", "anthropic": "claude-opus-5-5"}
PROVIDERS = ("gemini", "anthropic", "none")

_env_loaded = False


# --------------------------------------------------------------------------- .env + provider
def load_dotenv(path=None):
    """Minimal KEY=VALUE parser; never overrides variables already set in the environment."""
    global _env_loaded
    path = path or os.path.join(ROOT, ".env")
    if path == os.path.join(ROOT, ".env"):
        if _env_loaded:
            return
        _env_loaded = True
    try:
        with open(path) as fh:
            lines = fh.read().splitlines()
    except OSError:
        return
    for line in lines:
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        k = k.strip()
        if k.startswith("export "):
            k = k[7:].strip()
        v = v.strip()
        if len(v) >= 2 and v[0] == v[-1] and v[0] in "\"'":
            v = v[1:-1]
        elif " #" in v:
            v = v.split(" #", 1)[0].rstrip()
        if k and k not in os.environ:
            os.environ[k] = v


def replay_only() -> bool:
    load_dotenv()
    return os.environ.get("LLM_REPLAY", "").strip().lower() in ("1", "true", "yes")


def resolve(provider=None) -> tuple[str, str | None]:
    """(provider, model). provider 'none' means no LLM (callers use their deterministic fallback)."""
    load_dotenv()
    p = (provider or os.environ.get("LLM_PROVIDER") or "").strip().lower()
    if p not in PROVIDERS:
        if os.environ.get("GEMINI_API_KEY"):
            p = "gemini"
        elif os.environ.get("ANTHROPIC_API_KEY"):
            p = "anthropic"
        else:
            p = "gemini" if replay_only() else "none"     # replay a recorded Gemini run with no key
    if p == "none":
        return "none", None
    model = os.environ.get("GEMINI_MODEL" if p == "gemini" else "ANTHROPIC_MODEL") or DEFAULT_MODELS[p]
    return p, model


def available(provider=None) -> bool:
    """True if call_json could answer (a key for the provider, or replay mode)."""
    p, _ = resolve(provider)
    if p == "none":
        return False
    return replay_only() or bool(_key(p))


def _key(p):
    return os.environ.get("GEMINI_API_KEY" if p == "gemini" else "ANTHROPIC_API_KEY", "").strip()


# --------------------------------------------------------------------------- cache
def cache_key(provider, model, system, prompt, schema) -> str:
    blob = json.dumps({"p": provider, "m": model, "s": system or "", "u": prompt, "schema": schema},
                      sort_keys=True)
    return hashlib.sha256(blob.encode()).hexdigest()[:32]


def cache_get(key, path=None):
    path = path or CACHE_PATH
    hit = None
    try:
        with open(path) as fh:
            for line in fh:
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                if r.get("key") == key:
                    hit = r.get("out")            # last write wins
    except OSError:
        return None
    return hit


def cache_put(key, provider, model, out, path=None):
    path = path or CACHE_PATH
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "a") as fh:
            fh.write(json.dumps({"key": key, "provider": provider, "model": model, "t": int(time.time()),
                                 "out": out}, sort_keys=True) + "\n")
    except OSError:
        pass


# --------------------------------------------------------------------------- HTTP (tests replace this)
class HTTPError(Exception):
    def __init__(self, status, body=""):
        super().__init__(f"HTTP {status}")
        self.status, self.body = status, body


def _http_post(url, headers, body, timeout):
    """POST JSON, return the decoded JSON response. Raises HTTPError / OSError / ValueError."""
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST",
                                 headers=dict(headers, **{"content-type": "application/json"}))
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode())
    except urllib.error.HTTPError as e:
        try:
            txt = e.read().decode(errors="replace")[:500]
        except Exception:                                       # noqa: BLE001
            txt = ""
        raise HTTPError(e.code, txt) from None


# --------------------------------------------------------------------------- schemas
_GEMINI_KEYS = {"type", "properties", "required", "items", "enum", "description", "nullable", "format",
                "minimum", "maximum", "minItems", "maxItems", "propertyOrdering"}


def gemini_schema(s):
    """JSON Schema -> Gemini responseSchema (OpenAPI subset: drop additionalProperties etc.)."""
    if isinstance(s, dict):
        out = {}
        for k, v in s.items():
            if k not in _GEMINI_KEYS:
                continue
            if k == "properties":
                out[k] = {pk: gemini_schema(pv) for pk, pv in v.items()}
                out.setdefault("propertyOrdering", list(v))
            elif k == "items":
                out[k] = gemini_schema(v)
            elif k == "type" and isinstance(v, str):
                out[k] = v.upper()
            else:
                out[k] = v
        return out
    return s


def _parse_json_text(text):
    t = (text or "").strip()
    if t.startswith("```"):
        t = t.split("\n", 1)[1] if "\n" in t else ""
        t = t.rsplit("```", 1)[0]
    t = t.strip()
    try:
        return json.loads(t)
    except ValueError:
        i, j = t.find("{"), t.rfind("}")
        if 0 <= i < j:
            return json.loads(t[i:j + 1])
        raise


# --------------------------------------------------------------------------- providers
def _gemini(model, system, prompt, schema, timeout, max_tokens, temperature):
    gen = {"temperature": temperature, "maxOutputTokens": max_tokens, "responseMimeType": "application/json"}
    if schema:
        gen["responseSchema"] = gemini_schema(schema)
    text_in = f"{system}\n\n{prompt}" if system else prompt
    body = {"contents": [{"role": "user", "parts": [{"text": text_in}]}], "generationConfig": gen}
    url = GEMINI_URL.format(model=model)
    headers = {"x-goog-api-key": _key("gemini")}
    try:
        resp = _post_retry(url, headers, body, timeout)
    except HTTPError as e:
        if e.status == 400 and schema:                 # schema rejected by this model: JSON mode only
            gen.pop("responseSchema", None)
            resp = _post_retry(url, headers, body, timeout)
        else:
            raise
    cands = resp.get("candidates") or []
    if not cands:
        raise ValueError(f"no candidates ({(resp.get('promptFeedback') or {}).get('blockReason', 'empty')})")
    c = cands[0]
    if c.get("finishReason") not in (None, "STOP"):
        raise ValueError(f"finishReason {c.get('finishReason')}")
    parts = (c.get("content") or {}).get("parts") or []
    text = "".join(p.get("text", "") for p in parts if not p.get("thought"))
    return _parse_json_text(text)


def _anthropic(model, system, prompt, schema, timeout, max_tokens, temperature):
    body = {"model": model, "max_tokens": max_tokens,
            "messages": [{"role": "user", "content": prompt}]}
    if system:
        body["system"] = system
    if schema:
        body["output_config"] = {"format": {"type": "json_schema", "schema": schema}}
    headers = {"x-api-key": _key("anthropic"), "anthropic-version": "2023-06-01"}
    resp = _post_retry(ANTHROPIC_URL, headers, body, timeout)
    if resp.get("stop_reason") in ("refusal", "max_tokens"):
        raise ValueError(f"stop_reason {resp.get('stop_reason')}")
    text = "".join(b.get("text", "") for b in resp.get("content") or [] if b.get("type") == "text")
    return _parse_json_text(text)


def _post_retry(url, headers, body, timeout):
    try:
        return _http_post(url, headers, body, timeout)
    except HTTPError as e:
        if e.status in (429, 500, 502, 503, 504):
            time.sleep(1.0)
            return _http_post(url, headers, body, timeout)
        raise


# --------------------------------------------------------------------------- public
def call_json(prompt, schema=None, *, system=None, provider=None, timeout=25, max_tokens=2048,
              temperature=0.2, cache=True, cache_path=None, meta=None) -> dict | None:
    """One structured-JSON call. Returns the parsed dict, or None (no provider, replay miss, network,
    HTTP, parse or timeout failure). Never raises. ``meta`` (a dict) is filled with
    provider, model, cached, latency_ms and error."""
    m = meta if meta is not None else {}
    p, model = resolve(provider)
    m.update(provider=p, model=model, cached=False, latency_ms=0.0, error=None)
    if p == "none":
        m["error"] = "no LLM provider configured"
        return None
    key = cache_key(p, model, system, prompt, schema)
    if cache or replay_only():
        hit = cache_get(key, cache_path)
        if isinstance(hit, dict):
            m["cached"] = True
            return hit
    if replay_only():
        m["error"] = "replay miss"
        return None
    if not _key(p):
        m["error"] = f"no {p} API key"
        return None
    t0 = time.time()
    try:
        fn = _gemini if p == "gemini" else _anthropic
        out = fn(model, system, prompt, schema, timeout, max_tokens, temperature)
    except HTTPError as e:
        m["error"] = f"{p} HTTP {e.status}"
        out = None
    except (OSError, ValueError, TypeError, KeyError, AttributeError) as e:
        m["error"] = f"{p} {type(e).__name__}: {str(e)[:120]}"
        out = None
    except Exception as e:                                      # noqa: BLE001  (never break the mission)
        m["error"] = f"{p} {type(e).__name__}"
        out = None
    m["latency_ms"] = round((time.time() - t0) * 1000, 1)
    if out is not None and not isinstance(out, dict):
        m["error"], out = "response is not a JSON object", None
    if out is not None and cache:
        cache_put(key, p, model, out, cache_path)
    return out
