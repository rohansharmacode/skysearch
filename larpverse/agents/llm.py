"""Tiny provider layer: one structured-JSON call, Gemini first, never raises.

    from agents import llm
    out = llm.call_json(prompt, schema, system="...")      # dict, or None on any failure

Provider: ``provider=`` argument, else env LLM_PROVIDER (gemini | anthropic | none), else gemini if
GEMINI_API_KEY is set, else anthropic if ANTHROPIC_API_KEY is set, else none. Keys are read from the
environment or from a local ``.env`` at the repo root (gitignored); they are never printed or cached.

Record/replay: every successful answer is appended to agents/cache/llm.jsonl keyed by a hash of
provider + model + system + prompt + schema. A later identical call is answered from the cache
(free, offline, deterministic). ``LLM_REPLAY=1`` forces replay-only: cache hit or None, no network.

Audio (Gemini only): ``call_json_audio(prompt, wav_bytes, schema)`` sends one clip as base64
``inline_data`` (audio/wav) plus the prompt; same provider rules, .env, validation and record/replay
cache, keyed additionally by the SHA-256 of the audio bytes. ``meta["usage"]`` gets Gemini's token
counts when the API reports them.

Stdlib only (urllib), no SDKs.
"""
from __future__ import annotations

import base64
import hashlib
import json
import os
import random
import re
import sys
import threading
import time
import urllib.error
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CACHE_PATH = os.path.join(ROOT, "agents", "cache", "llm.jsonl")
GEMINI_URL = "https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent"
ANTHROPIC_URL = "https://api.anthropic.com/v1/messages"
DEFAULT_MODELS = {"gemini": "gemini-3.7-flash", "anthropic": "claude-opus-5-5"}
PROVIDERS = ("gemini", "anthropic", "none")

_env_loaded = False


# --------------------------------------------------------------------------- .env + provider
def load_dotenv(path=None):
    """Minimal KEY=VALUE parser; never overrides variables already set in the environment."""
    global _env_loaded
    path = path or os.path.join(ROOT, ".env")
    if path == os.path.join(ROOT, ".env"):
        # no key yet: look again at most every 10 s, so a key added mid-run (scripts/set_gemini_key.sh) is
        # picked up without restarting Isaac or the audio worker
        has_key = os.environ.get("GEMINI_API_KEY") or os.environ.get("ANTHROPIC_API_KEY")
        if _env_loaded and (has_key or time.time() - _env_loaded < 10.0):
            return
        _env_loaded = time.time()
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
    def __init__(self, status, body="", retry_after=None):
        super().__init__(f"HTTP {status}")
        self.status, self.body, self.retry_after = status, body, retry_after

    @property
    def daily_quota(self) -> bool:
        """A per-day quota 429: retrying today is pointless."""
        return self.status == 429 and bool(re.search(r"per ?day|PerDay|requests per day", self.body or "", re.I))


# --------------------------------------------------------------------------- client-side rate limit
class TokenBucket:
    """Requests-per-minute bucket shared by every Gemini call in this process (thread-safe).
    Capacity = burst (default 2), refill = rpm / 60 per second. ``acquire`` blocks and logs when throttled."""

    def __init__(self, rpm, burst=2):
        self.rpm, self.cap = max(0.1, float(rpm)), max(1.0, float(burst))
        self.tokens, self.t = self.cap, time.monotonic()
        self.lock = threading.Lock()
        self.waited_s = 0.0

    def acquire(self, label="gemini"):
        with self.lock:
            now = time.monotonic()
            self.tokens = min(self.cap, self.tokens + (now - self.t) * self.rpm / 60.0)
            self.t = now
            wait = 0.0 if self.tokens >= 1 else (1 - self.tokens) * 60.0 / self.rpm
            self.tokens -= 1                                # reserve now; later callers queue behind
        if wait > 0.05:
            print(f"[llm] throttled: waiting {wait:.1f} s ({label}, GEMINI_RPM={self.rpm:g})", file=sys.stderr,
                  flush=True)
            time.sleep(wait)
            self.waited_s += wait
        return wait


_buckets = {}


def bucket(provider="gemini") -> TokenBucket:
    """GEMINI_RPM (default 10) / ANTHROPIC_RPM (default 50); GEMINI_BURST (default 2)."""
    load_dotenv()
    env = "GEMINI" if provider == "gemini" else "ANTHROPIC"
    rpm = float(os.environ.get(f"{env}_RPM") or (10 if provider == "gemini" else 50))
    burst = float(os.environ.get(f"{env}_BURST") or 2)
    b = _buckets.get(provider)
    if b is None or b.rpm != rpm or b.cap != burst:
        b = _buckets[provider] = TokenBucket(rpm, burst)
    return b


def _retry_after(e, body):
    """Seconds from a Retry-After header or Google's RetryInfo.retryDelay ("12s" / "12.5s") in the body."""
    try:
        h = e.headers.get("Retry-After") if getattr(e, "headers", None) else None
        if h:
            return float(h)
    except (TypeError, ValueError):
        pass
    m = re.search(r'"retryDelay"\s*:\s*"([0-9.]+)s"', body or "")
    return float(m.group(1)) if m else None


def _http_post(url, headers, body, timeout):
    """POST JSON, return the decoded JSON response. Raises HTTPError / OSError / ValueError.
    Real requests pass the provider's token bucket first (tests replace this function, so no throttling)."""
    bucket("anthropic" if "anthropic.com" in url else "gemini").acquire()
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST",
                                 headers=dict(headers, **{"content-type": "application/json"}))
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode())
    except urllib.error.HTTPError as e:
        try:
            txt = e.read().decode(errors="replace")[:2000]
        except Exception:                                       # noqa: BLE001
            txt = ""
        raise HTTPError(e.code, txt, _retry_after(e, txt)) from None


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
def _gemini(model, system, prompt, schema, timeout, max_tokens, temperature, audio=None, usage=None,
            thinking=None, retries=None):
    """audio: optional (mime, bytes) sent as inline_data before the text part. thinking: optional
    generationConfig.thinkingConfig (e.g. {"thinkingLevel": "low"}); dropped if the model rejects it."""
    gen = {"temperature": temperature, "maxOutputTokens": max_tokens, "responseMimeType": "application/json"}
    if schema:
        gen["responseSchema"] = gemini_schema(schema)
    if thinking:
        gen["thinkingConfig"] = dict(thinking)
    text_in = f"{system}\n\n{prompt}" if system else prompt
    parts = [{"text": text_in}]
    if audio is not None:
        parts.insert(0, {"inline_data": {"mime_type": audio[0], "data": base64.b64encode(audio[1]).decode()}})
    body = {"contents": [{"role": "user", "parts": parts}], "generationConfig": gen}
    url = GEMINI_URL.format(model=model)
    headers = {"x-goog-api-key": _key("gemini")}
    try:
        resp = _post_retry(url, headers, body, timeout, retries)
    except HTTPError as e:
        if e.status == 400 and (schema or thinking):   # schema/thinking rejected by this model: JSON mode only
            gen.pop("responseSchema", None)
            gen.pop("thinkingConfig", None)
            resp = _post_retry(url, headers, body, timeout, retries)
        else:
            raise
    if usage is not None and isinstance(resp.get("usageMetadata"), dict):
        u = resp["usageMetadata"]
        usage.update({k: u[k] for k in ("promptTokenCount", "candidatesTokenCount", "thoughtsTokenCount",
                                        "totalTokenCount") if isinstance(u.get(k), int)})
        for d in u.get("promptTokensDetails") or []:
            if isinstance(d, dict) and d.get("modality") == "AUDIO":
                usage["audioTokenCount"] = d.get("tokenCount")
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


RETRY_STATUS = (429, 500, 502, 503, 504)


def _post_retry(url, headers, body, timeout, retries=None):
    """Retries 429/5xx with exponential backoff + jitter (1, 2, 4 ... s, x0.75-1.25), honouring
    Retry-After / retryDelay, capped by LLM_MAX_BACKOFF_S per wait (default 20) and LLM_BACKOFF_BUDGET_S
    in total (default 45). A per-day quota 429 is raised at once. retries: default LLM_RETRIES (3)."""
    if retries is None:
        retries = int(os.environ.get("LLM_RETRIES", "3"))
    cap = float(os.environ.get("LLM_MAX_BACKOFF_S", "20"))
    budget = float(os.environ.get("LLM_BACKOFF_BUDGET_S", "45"))
    spent = 0.0
    for i in range(retries + 1):
        try:
            return _http_post(url, headers, body, timeout)
        except HTTPError as e:
            if e.status not in RETRY_STATUS or i == retries or e.daily_quota:
                raise
            wait = (e.retry_after if e.retry_after is not None else 2.0 ** i) * random.uniform(0.75, 1.25)
            wait = min(wait, cap)
            if spent + wait > budget:
                raise
            print(f"[llm] HTTP {e.status}: backing off {wait:.1f} s (attempt {i + 1}/{retries})", file=sys.stderr,
                  flush=True)
            time.sleep(wait)
            spent += wait


def _safe_validate(fn, out):
    try:
        return bool(fn(out))
    except Exception:                                           # noqa: BLE001
        return False


# --------------------------------------------------------------------------- public
def call_json(prompt, schema=None, *, system=None, provider=None, timeout=25, max_tokens=2048,
              temperature=0.2, cache=True, cache_path=None, meta=None, validate=None, model=None) -> dict | None:
    """One structured-JSON call. Returns the parsed dict, or None (no provider, replay miss, network,
    HTTP, parse or timeout failure). Never raises. ``meta`` (a dict) is filled with
    provider, model, cached, latency_ms and error. ``validate(out) -> bool``: only answers it accepts
    are recorded, so a rejected answer is never replayed (it is still returned for the caller to reject).
    ``model`` overrides the provider's default model."""
    m = meta if meta is not None else {}
    p, model_ = resolve(provider)
    model = (model or model_) if p != "none" else None
    m.update(provider=p, model=model, cached=False, latency_ms=0.0, error=None, busy=False)
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
        m["error"] = f"{p} HTTP {e.status}" + (" (daily quota)" if e.daily_quota else "")
        m["busy"] = e.status in RETRY_STATUS
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
    ok = out is not None and (validate is None or _safe_validate(validate, out))
    if ok and cache:
        cache_put(key, p, model, out, cache_path)
    return out


def call_json_audio(prompt, wav_bytes: bytes, schema=None, *, system=None, mime="audio/wav", provider=None,
                    timeout=60, max_tokens=4096, temperature=0.1, cache=True, cache_path=None, meta=None,
                    validate=None, thinking=None, retries=None, model=None) -> dict | None:
    """One structured-JSON call about an audio clip (Gemini audio understanding). Never raises.

    ``wav_bytes``: the encoded clip (e.g. 16 kHz mono WAV, keep it under ~15 s; inline requests must
    stay under 20 MB). Audio input is Gemini-only: the provider is Gemini unless ``provider`` or
    LLM_PROVIDER is "none"/"anthropic" (then None with meta["error"]). Same cache/replay/validate
    semantics as call_json; the cache key includes the audio's SHA-256 (``meta["audio_sha"]``).
    ``meta`` also gets ``usage`` (prompt/candidates/thoughts/audio token counts) on a live call.
    ``thinking``: optional thinkingConfig, e.g. {"thinkingLevel": "low"} (not part of the cache key).
    ``retries``: extra attempts on 429/5xx (default LLM_RETRIES = 3; see _post_retry). ``model``: override the Gemini model
    (default GEMINI_MODEL, else gemini-3.7-flash)."""
    m = meta if meta is not None else {}
    p, _ = resolve(provider)
    model = model or resolve("gemini")[1]
    asha = hashlib.sha256(wav_bytes or b"").hexdigest()
    m.update(provider=p, model=model, cached=False, latency_ms=0.0,
             error=None, busy=False, audio_sha=asha[:16], usage={}, throttled_ms=0.0)
    if p == "none":
        m["error"] = "no LLM provider configured"
        return None
    if p != "gemini":
        m["error"] = f"audio input needs gemini (provider is {p})"
        return None
    if not wav_bytes:
        m["error"] = "empty audio"
        return None
    key = cache_key("gemini", model, system, f"{prompt}\n[audio {mime} sha256 {asha}]", schema)
    if cache or replay_only():
        hit = cache_get(key, cache_path)
        if isinstance(hit, dict):
            m["cached"] = True
            return hit
    if replay_only():
        m["error"] = "replay miss"
        return None
    if not _key("gemini"):
        m["error"] = "no gemini API key"
        return None
    t0, w0 = time.time(), bucket("gemini").waited_s
    try:
        out = _gemini(model, system, prompt, schema, timeout, max_tokens, temperature,
                      audio=(mime, wav_bytes), usage=m["usage"], thinking=thinking, retries=retries)
    except HTTPError as e:
        m["error"] = f"gemini HTTP {e.status}" + (" (daily quota)" if e.daily_quota else "")
        m["busy"] = e.status in RETRY_STATUS
        out = None
    except (OSError, ValueError, TypeError, KeyError, AttributeError) as e:
        m["error"] = f"gemini {type(e).__name__}: {str(e)[:120]}"
        out = None
    except Exception as e:                                      # noqa: BLE001
        m["error"] = f"gemini {type(e).__name__}"
        out = None
    m["throttled_ms"] = round((bucket("gemini").waited_s - w0) * 1000, 1)       # client-side rate-limit wait
    m["latency_ms"] = round((time.time() - t0) * 1000 - m["throttled_ms"], 1)
    if out is not None and not isinstance(out, dict):
        m["error"], out = "response is not a JSON object", None
    ok = out is not None and (validate is None or _safe_validate(validate, out))
    if ok and cache:
        cache_put(key, "gemini", model, out, cache_path)
    return out
