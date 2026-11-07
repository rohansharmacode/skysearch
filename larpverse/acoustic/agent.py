"""Acoustic triage agent: judges each separated clip, then ranks piles and picks a dispatch.

Per clip it sees only observable measurements (transcript, class scores, VAD, SNR, DoA
agreement, pile assignment), never scene ground truth, and returns
{human, urgency, reliability} in [0, 1] plus a short rationale.

  * Claude (claude-opus-5-5, structured JSON output) when ANTHROPIC_API_KEY is set.
  * Deterministic fallback scorer otherwise, or when the LLM output fails validation.
  * Record/replay cache (acoustic/cache/decisions.jsonl) keyed by a hash of the features,
    so a demo replays the same LLM decisions offline.

Aggregation to a 0-100 pile priority and the dispatch choice are deterministic code; the LLM
never touches the numeric loop. Priority is not proof; silent piles are "not cleared".
"""
from __future__ import annotations

import hashlib
import json
import os
import re
import time

MODEL = "claude-opus-5-5"
PROMPT_VERSION = "acoustic-judge-v1"
CACHE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cache", "decisions.jsonl")
HUMAN_LEAD = 0.4            # below this a detected source is dismissed as non-human
SILENT_PRIORITY = 8         # unknown, not cleared: above dismissed sources, below any human lead

KEYWORDS = [
    (r"can'?t breathe|cannot breathe|can not breathe|breath", 0.5, "breathing"),
    (r"\bhelp\b", 0.35, "help"),
    (r"trapped|stuck|pinned|can'?t move|cannot move", 0.35, "trapped"),
    (r"hurt|injur|bleed|pain|broken", 0.35, "injury"),
    (r"water|thirst", 0.25, "water"),
    (r"hear me|anyone|someone|somebody", 0.2, "calling out"),
    (r"please|hurry", 0.1, "pleading"),
    (r"under here|down here", 0.1, "location"),
]
CLASS_URGENCY = {"shout": 0.6, "knocking": 0.55, "voice": 0.35}

SYSTEM = """You are the acoustic triage analyst for an urban search-and-rescue drone team.
Three drones have landed on a rubble field with rotors off and listened with small mic arrays.
The pipeline located sources by SRP-PHAT direction of arrival, assigned them to rubble piles by
triangulation, beamformed each source into a clip, then ran voice activity detection (Silero),
sound classification (YAMNet plus two heuristics) and speech-to-text (Whisper).

For every clip, judge from the measurements only:
- human: probability in [0, 1] that a person produced the sound. Speech, calling, crying and
  deliberate patterned knocking count; dogs, engines, generators and settling debris do not.
  A transcript on a clip with low VAD speech ratio may be leakage or an ASR hallucination.
- urgency: [0, 1], how urgently that person needs help, from content (help, trapped, injury,
  breathing, thirst) and signs of weakness. Use near 0 for non-human sources.
- reliability: [0, 1], how much to trust this detection and its pile assignment, from drone
  agreement, triangulation error, cone width, SNR and how plausible the transcript is.
- rationale: at most 25 words citing the measurements you relied on.

Do not invent facts that are not in the data. Priority is not proof of a person, and silence is
not proof of absence. Return one judgment per clip_id."""

SCHEMA = {
    "type": "object",
    "properties": {
        "judgments": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "clip_id": {"type": "string"},
                    "human": {"type": "number"},
                    "urgency": {"type": "number"},
                    "reliability": {"type": "number"},
                    "rationale": {"type": "string"},
                },
                "required": ["clip_id", "human", "urgency", "reliability", "rationale"],
                "additionalProperties": False,
            },
        }
    },
    "required": ["judgments"],
    "additionalProperties": False,
}

FORBIDDEN_KEYS = {"truth", "kind", "events_s", "targets"}


def _clip01(v):
    return float(min(1.0, max(0.0, v)))


# --------------------------------------------------------------------------- fallback
def keyword_hits(text: str):
    t = (text or "").lower()
    hits = [(name, w) for pat, w, name in KEYWORDS if re.search(pat, t)]
    return min(1.0, sum(w for _, w in hits)), [n for n, _ in hits]


def fallback_judge(f: dict) -> dict:
    cs = f["class_scores"]
    vad = f["vad_speech_ratio"]
    words = len((f.get("transcript") or "").split())
    kp = f.get("heuristics", {}).get("knock_periodicity", 0.0)
    kw, hits = keyword_hits(f.get("transcript", ""))

    e_speech = min(1.0, cs.get("voice", 0) + cs.get("shout", 0)) * (0.5 + 0.5 * min(1.0, vad / 0.3))
    e_words = (0.9 if words >= 3 else 0.6 if words else 0.0) if vad >= 0.15 else 0.0
    e_knock = 0.8 * min(1.0, cs.get("knocking", 0) + 0.5 * kp)
    human = max(e_speech, e_words, e_knock)
    nonhuman = cs.get("dog", 0) + cs.get("engine", 0) + cs.get("debris", 0)
    if words == 0:
        human *= 1.0 - 0.6 * min(1.0, nonhuman)
    human = _clip01(human)

    cls = f["sound_class"]
    base = CLASS_URGENCY.get(cls, 0.05)
    urg = max(kw, base) + (0.15 if kw > 0 and base > 0.3 else 0.0)
    urgency = _clip01(min(1.0, urg) * (0.4 + 0.6 * human))

    doa = f["doa"]
    width = f["width_deg"]
    tri = doa.get("tri_err_m")
    r_doa = (doa["drones_agreeing"] / doa["drones_listening"]) \
        * (0.0 if tri is None else 2.718 ** (-(tri / 6.0) ** 2)) \
        * min(1.0, max(0.2, 1.0 - (width - 20.0) / 80.0))
    r_snr = _clip01(f["snr_db"] / 15.0)
    reliability = 0.6 * r_doa + 0.4 * r_snr
    if not doa.get("assigned", False):
        reliability *= 0.5
    reliability = _clip01(reliability)

    bits = [f"{cls} {cs.get(cls, 0):.2f}", f"VAD {vad:.2f}"]
    if hits:
        bits.append("keywords: " + ", ".join(hits))
    if cls == "knocking":
        bits.append(f"knock regularity {kp:.2f}")
    bits.append(f"{doa['drones_agreeing']}/{doa['drones_listening']} drones agree"
                + (f", tri err {tri:.1f} m" if tri is not None else ""))
    bits.append(f"width {width:.0f} deg, SNR {f['snr_db']:.0f} dB")
    return {"human": round(human, 2), "urgency": round(urgency, 2), "reliability": round(reliability, 2),
            "rationale": "; ".join(bits)}


# --------------------------------------------------------------------------- LLM + cache
def feature_key(f: dict) -> str:
    blob = json.dumps({"v": PROMPT_VERSION, "model": MODEL, "f": f}, sort_keys=True)
    return hashlib.sha256(blob.encode()).hexdigest()[:24]


def load_cache(path=CACHE_PATH) -> dict:
    out = {}
    if os.path.exists(path):
        with open(path) as fh:
            for line in fh:
                try:
                    r = json.loads(line)
                    out[r["key"]] = r["judgment"]
                except (ValueError, KeyError):
                    continue
    return out


def append_cache(rows, path=CACHE_PATH):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "a") as fh:
        for r in rows:
            fh.write(json.dumps(r, sort_keys=True) + "\n")


def validate_judgment(j) -> dict | None:
    try:
        out = {}
        for k in ("human", "urgency", "reliability"):
            v = float(j[k])
            if not (0.0 <= v <= 1.0) or v != v:
                return None
            out[k] = round(v, 2)
        r = str(j["rationale"]).strip()
        if not r:
            return None
        out["rationale"] = r[:300]
        return out
    except (KeyError, TypeError, ValueError):
        return None


def llm_judge(features: list[dict]) -> tuple[dict, str | None]:
    """One structured-output call for all clips. Returns ({clip_id: judgment}, error)."""
    import anthropic

    client = anthropic.Anthropic(timeout=90.0, max_retries=2)
    try:
        resp = client.beta.messages.create(
            model=MODEL,
            max_tokens=8000,
            betas=["server-side-fallback-2026-07-01"],
            fallbacks="default",
            output_config={"effort": "low", "format": {"type": "json_schema", "schema": SCHEMA}},
            system=SYSTEM,
            messages=[{"role": "user", "content": json.dumps({"clips": features}, sort_keys=True)}],
        )
    except anthropic.AuthenticationError:
        return {}, "authentication failed"
    except anthropic.RateLimitError:
        return {}, "rate limited"
    except anthropic.APIStatusError as e:
        return {}, f"API error {e.status_code}"
    except anthropic.APIConnectionError:
        return {}, "network error"
    if resp.stop_reason == "refusal":
        return {}, "refused"
    if resp.stop_reason == "max_tokens":
        return {}, "truncated"
    text = next((b.text for b in resp.content if b.type == "text"), "")
    try:
        data = json.loads(text)
    except ValueError:
        return {}, "invalid JSON"
    ids = {f["clip_id"] for f in features}
    out = {}
    for j in data.get("judgments", []) if isinstance(data, dict) else []:
        cid = j.get("clip_id") if isinstance(j, dict) else None
        v = validate_judgment(j) if cid in ids else None
        if v:
            out[cid] = v
    return out, (None if len(out) == len(ids) else f"{len(ids) - len(out)} invalid/missing judgments")


def judge_clips(features: list[dict], mode="auto", cache_path=CACHE_PATH):
    """mode: auto (cache, then LLM if key, else fallback) | llm | replay (cache only) | fallback."""
    for f in features:
        assert not (FORBIDDEN_KEYS & set(f)), "ground truth must never reach the agent"
    meta = {"mode": mode, "model": MODEL, "llm_calls": 0, "cache_hits": 0, "fallback": 0, "error": None}
    out = {}
    cache = load_cache(cache_path) if mode in ("auto", "llm", "replay") else {}
    todo = []
    for f in features:
        k = feature_key(f)
        if k in cache and validate_judgment(cache[k]):
            out[f["clip_id"]] = dict(cache[k], by="llm")
            meta["cache_hits"] += 1
        else:
            todo.append(f)
    use_llm = mode == "llm" or (mode == "auto" and os.environ.get("ANTHROPIC_API_KEY"))
    if todo and use_llm:
        meta["llm_calls"] += 1
        got, err = llm_judge(todo)
        meta["error"] = err
        rows = []
        for f in todo:
            if f["clip_id"] in got:
                out[f["clip_id"]] = dict(got[f["clip_id"]], by="llm")
                rows.append({"key": feature_key(f), "clip_id": f["clip_id"], "model": MODEL,
                             "prompt": PROMPT_VERSION, "t": int(time.time()), "judgment": got[f["clip_id"]]})
        if rows:
            append_cache(rows, cache_path)
    for f in features:
        if f["clip_id"] not in out:
            out[f["clip_id"]] = dict(fallback_judge(f), by="fallback")
            meta["fallback"] += 1
    return out, meta


# --------------------------------------------------------------------------- aggregation
def clip_priority(j) -> int:
    return int(round(100 * j["human"] * (0.35 + 0.65 * j["urgency"]) * (0.45 + 0.55 * j["reliability"])))


def aggregate(clips: list[dict], piles_xy: dict, start_pile: str):
    """clips carry their judgment fields. Returns (piles, dispatch)."""
    by_pile = {}
    for c in clips:
        by_pile.setdefault(c["pile_id"], []).append(c)
    piles = []
    for pid, xy in piles_xy.items():
        cs = sorted(by_pile.get(pid, []), key=lambda c: -c["priority"])
        if not cs:
            why = "No sound assigned to this pile during the listen. Silence is not absence: not cleared."
            if pid == start_pile:
                why = "Drones are landed here; nothing heard from this pile. Not cleared."
            piles.append({"pile_id": pid, "xy": list(xy), "priority": SILENT_PRIORITY,
                          "status": "no_signal_not_cleared", "top_clip": None, "why": why})
            continue
        top = cs[0]
        pr = min(100, top["priority"] + 3 * (len(cs) - 1))
        if top["human"] < HUMAN_LEAD:
            st = "dismissed_nonhuman"
            why = (f"Heard {top['sound_class']} (human {top['human']:.2f}); dismissed as an acoustic lead, "
                   "but the pile is not cleared.")
        else:
            st = "lead"
            said = f" saying \"{_short(top['transcript'])}\"" if top.get("transcript") else ""
            why = (f"{top['sound_class'].capitalize()}{said}; human {top['human']:.2f}, urgency "
                   f"{top['urgency']:.2f}, reliability {top['reliability']:.2f}.")
        piles.append({"pile_id": pid, "xy": list(xy), "priority": pr, "status": st,
                      "top_clip": top["clip_id"], "why": why})
    piles.sort(key=lambda p: (-p["priority"], p["pile_id"]))
    leads = [p for p in piles if p["status"] == "lead"]
    if leads:
        p = leads[0]
        p["status"] = "radar_ping"
        top = next(c for c in clips if c["clip_id"] == p["top_clip"])
        what = f"\"{_short(top['transcript'])}\"" if top.get("transcript") else top["sound_class"]
        reason = (f"Send the radar drone to {p['pile_id']}: highest priority ({p['priority']}) from {what}, "
                  f"{top['doa_agree']} drones agreeing on the bearing at {top['snr_db']:.0f} dB SNR.")
        dispatch = {"pile_id": p["pile_id"], "reason": reason}
    else:
        dispatch = {"pile_id": None, "reason": "No human-like acoustic lead; continue the systematic "
                                               "search and keep every pile marked not cleared."}
    return piles, dispatch


def _short(t, n=48):
    t = (t or "").strip()
    return t if len(t) <= n else t[: n - 1].rsplit(" ", 1)[0] + "…"
