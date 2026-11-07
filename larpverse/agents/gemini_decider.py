"""Dispatch-gate decider for sim/grid_mission.py: numbers propose, the LLM decides, code verifies.

    from agents.gemini_decider import make_decider
    decide = make_decider()                       # provider from env (Gemini first), see agents/llm.py
    out = decide(context, options)                # {"choice", "rationale", "by", "latency_ms", ...}

``options`` are the mission's scored options (id, action, pile_id, priority, travel_s,
battery_after_min, evidence, flags). The LLM sees only observable evidence (acoustic transcripts,
classes and scores, radar ping statuses, batteries, travel, budget, operator notes); any key that
could carry scene ground truth is stripped before the prompt is built. It must pick one listed id.

Validator: the id must be listed, and the option must not be below the battery reserve, on a
no-go pile, or unreachable; the rationale must not claim a confirmed person. Otherwise the
deterministic fallback picks the highest-priority valid option (operator priors applied, ignored-
sound piles treated as silent), else stop, with by="fallback". No LLM configured -> fallback.
"""
from __future__ import annotations

import json
import re
import time

from . import intake, llm

PROMPT_VERSION = "dispatch-v1"
RESERVE = 0.25                     # battery fraction every drone must keep after the option
SILENT_PRIORITY = 8
FORBIDDEN_KEYS = {"truth", "victims", "targets", "victim", "target", "ground_truth"}
BANNED = re.compile(r"(?<!not )(?<!un)(?<!never )\bconfirmed\b|survivor(s)? found|person found|"
                    r"found (a |the )?(survivor|person)|definitely (a |there is a )?(person|survivor)", re.I)

SYSTEM = """You are the dispatch judgement of the mission commander for an urban search-and-rescue
drone team. Three drones search a 3x3 grid of rubble piles (A1..C3). At each pile they land, scan
with breathing radar, then listen with microphones (rotors off) and rank the other piles by sound.
You now choose what the fleet does next.

You see only observable evidence: acoustic transcripts, sound classes and scores, radar ping
statuses, drone batteries, travel times, the pile budget and any operator notes. The numeric
planner has already scored the options; you choose one, using judgement the formulas lack:
operator notes, competing leads, battery margins, weak or contradictory evidence.

Rules:
- Choose exactly one option id from the list. Do not invent options.
- Never choose an option flagged below_reserve, no_go or unreachable.
- Sound on an ignore_sound pile is our own crew or machinery, not a lead.
- Priority is triage order, not proof. Never say a person is confirmed or found; radar pings are
  candidate search regions. Silent piles are not cleared.
- rationale: at most 2 sentences, citing the evidence you relied on (pile ids, scores, transcripts,
  batteries, notes)."""

SCHEMA = {
    "type": "object",
    "properties": {"choice": {"type": "string"}, "rationale": {"type": "string"}},
    "required": ["choice", "rationale"],
    "additionalProperties": False,
}


# --------------------------------------------------------------------------- context hygiene
def scrub(obj):
    """Deep copy without any key that could carry ground truth."""
    if isinstance(obj, dict):
        return {k: scrub(v) for k, v in obj.items() if str(k).lower() not in FORBIDDEN_KEYS}
    if isinstance(obj, (list, tuple)):
        return [scrub(v) for v in obj]
    return obj


def annotate(options, constraints):
    """Add validator flags (below_reserve, no_go, sound_ignored) and the operator prior to each option."""
    nogo = set(constraints.get("no_go_piles") or [])
    ign = set(constraints.get("ignore_sound_piles") or [])
    pri = constraints.get("pile_priors") or {}
    out = []
    for o in options:
        o = dict(o)
        flags = list(o.get("flags") or [])
        b = o.get("battery_after_min")
        pid = o.get("pile_id")
        if o.get("action") != "stop" and isinstance(b, (int, float)) and b < RESERVE:
            flags.append("below_reserve")
        if pid in nogo and o.get("action") != "stop":
            flags.append("no_go")
        if pid in ign:
            flags.append("sound_ignored")
        if pid in pri:
            o["operator_prior"] = pri[pid]
        o["flags"] = sorted(set(flags), key=flags.index)
        out.append(o)
    return out


def _invalid(o):
    return bool({"below_reserve", "no_go", "unreachable"} & set(o.get("flags") or []))


def effective_priority(o):
    p = o.get("priority")
    if p is None:
        return None
    if "sound_ignored" in (o.get("flags") or []):
        p = min(p, SILENT_PRIORITY)
    return p * float(o.get("operator_prior", 1.0))


def fallback_choice(options, priority_min=20):
    """Highest effective-priority valid dispatch at or above priority_min, else stop."""
    ok = [o for o in options if o.get("action") == "dispatch" and not _invalid(o)
          and (effective_priority(o) or 0) >= priority_min]
    ok.sort(key=lambda o: (-effective_priority(o), str(o.get("pile_id"))))
    if ok:
        o = ok[0]
        adj = "" if effective_priority(o) == o["priority"] else f", {effective_priority(o):.0f} after operator notes"
        return o["id"], f"highest valid priority: {o['pile_id']} ({o['priority']}{adj})."
    stop = next((o for o in options if o.get("action") == "stop"), None)
    if stop is not None:
        return stop["id"], f"no valid pile at or above priority {priority_min}; return. Remaining piles stay not cleared."
    valid = [o for o in options if not _invalid(o)]
    return (valid or options)[0]["id"], "no stop option listed; first valid option."


def build_prompt(context, options):
    ctx = scrub(context)
    opts = scrub(options)
    doc = {"prompt_version": PROMPT_VERSION, "reserve_battery": RESERVE, "situation": ctx, "options": opts}
    return ("Situation and options (JSON). Reply with JSON {\"choice\": <option id>, \"rationale\": <text>}.\n"
            + json.dumps(doc, sort_keys=True, ensure_ascii=False))


def _clean_rationale(r):
    r = " ".join(str(r or "").split())
    sents = re.split(r"(?<=[.!?])\s+", r)
    r = " ".join(sents[:2])
    return r[:320]


def validate(out, options):
    """Return (choice, rationale) or (None, reason)."""
    if not isinstance(out, dict):
        return None, "no structured answer"
    ids = {o["id"]: o for o in options}
    c = str(out.get("choice", "")).strip()
    if c not in ids:
        return None, f"choice {c!r} not a listed option"
    if _invalid(ids[c]):
        return None, f"choice {c} breaks a hard limit ({', '.join(ids[c]['flags'])})"
    r = _clean_rationale(out.get("rationale"))
    if not r:
        return None, "empty rationale"
    if BANNED.search(r):
        return None, "rationale makes a banned claim"
    return c, r


# --------------------------------------------------------------------------- factory
def make_decider(provider=None, notes=None, constraints=None, priority_min=None):
    """Return decide(context, options). Operator notes: ``constraints`` dict, else ``notes`` text,
    else ops/intel.txt (read once, here)."""
    if constraints is None:
        constraints = intake.intake(notes, provider=provider) if notes is not None else \
            intake.load(provider=provider)
    prov, model = llm.resolve(provider)

    def decide(context, options):
        t0 = time.time()
        pmin = priority_min if priority_min is not None else context.get("priority_min", 20)
        opts = annotate(options, constraints)
        ctx = dict(context)
        if constraints.get("notes"):
            ctx["operator_notes"] = {k: constraints[k] for k in
                                     ("notes", "pile_priors", "ignore_sound_piles", "no_go_piles")}
        meta, err = {}, None
        out = None
        if llm.available(provider):
            out = llm.call_json(build_prompt(ctx, opts), SCHEMA, system=SYSTEM, provider=provider,
                                max_tokens=1024, meta=meta, validate=lambda o: validate(o, opts)[0] is not None)
            err = meta.get("error")
        else:
            err = "no LLM configured"
        choice, why = validate(out, opts) if out is not None else (None, err or "LLM failed")
        res = {"latency_ms": round((time.time() - t0) * 1000, 1), "provider": prov, "model": model}
        if choice is not None:
            res.update(choice=choice, rationale=why,
                       by=(meta.get("provider") or prov) + ("-replay" if meta.get("cached") else ""))
        else:
            fc, fwhy = fallback_choice(opts, pmin)
            res.update(choice=fc, rationale=f"Fallback ({why}): {fwhy}", by="fallback", fallback_reason=why)
        return res

    decide.constraints = constraints
    return decide
