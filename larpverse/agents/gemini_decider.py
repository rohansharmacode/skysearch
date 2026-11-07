"""Dispatch-gate decider for sim/grid_mission.py: numbers propose, the LLM decides, code verifies.

    from agents.gemini_decider import make_decider
    decide = make_decider()                       # provider from env (Gemini first), see agents/llm.py
    out = decide(context, options)                # {"choice", "rationale", "by", "latency_ms", ...}
    decide.suggest_zone(state)                    # "suggest tasking P1: ..." (zones are the operator's call)
    dig_order(state["radar_candidates"])          # pings ranked by vitals dig_score, condition in plain words

``options`` are the mission's scored options (id, action, pile_id, priority, score, travel_s,
battery_after_min, evidence, flags; optional pod ``placements``). Actions: dispatch | revisit |
rescan_current | stop. The LLM sees only observable evidence (acoustic transcripts, classes and scores,
radar ping statuses and breathing-pattern summaries, batteries, travel, budgets, operator notes); any key
that could carry scene ground truth is stripped before the prompt is built. It must pick one listed id.

Validator: the id must be listed, and the option must not be below the battery reserve, on a no-go
pile, or unreachable; the rationale must not claim a confirmed person, a diagnosis, an age, a skipped
battery swap, a zone action taken, or (pod delivery) a landing on the rubble. Otherwise the
deterministic fallback picks the highest-scoring valid option (operator priors applied, ignored-sound
piles treated as silent; revisits ranked by their discounted score), else stop, with by="fallback".
A battery swap is a constraint the mission forces (25% reserve): the rationale explains it, never
overrides it. No LLM configured -> fallback.
"""
from __future__ import annotations

import json
import re
import time

from . import intake, llm

PROMPT_VERSION = "dispatch-v2"
RESERVE = 0.25                     # battery fraction every drone must keep after the option
SILENT_PRIORITY = 8
ASSESS_WEIGHT = 0.55               # sim/vitals.py TRIAGE_RANK["assess"]: dig weight of a ping without vitals
MOVE_ACTIONS = ("dispatch", "revisit", "rescan_current")
FORBIDDEN_KEYS = {"truth", "victims", "targets", "victim", "target", "ground_truth", "physio", "hidden"}
VITALS_KEEP = ("pattern", "rate_bpm", "triage_hint", "pattern_conf", "apnea_events", "duration_s", "sqi")

# --------------------------------------------------------------------------- wording rules
BANNED = re.compile(r"(?<!not )(?<!un)(?<!never )\bconfirmed\b|survivor(s)? found|person found|"
                    r"found (a |the )?(survivor|person)|definitely (a |there is a )?(person|survivor)|diagnos", re.I)
# Age / child inference is never made (user decision 2026-11-07); banned in any breathing / vitals sentence.
AGE_WORDS = re.compile(r"\b(child|children|kids?|infants?|bab(y|ies)|toddlers?|elderly|old (man|woman|person)|"
                       r"teen(ager)?s?|adults?|young|aged?|years? old)\b", re.I)
VITALS_CTX = re.compile(r"breath|vital|triage|respirat|pattern|condition|/min|dig order|apn(o)?ea", re.I)
FACT_CLAIM = re.compile(r"\b(is|are|was|were|seems?|appears?)\s+(injured|unconscious|dead|deceased|dying)\b|"
                        r"\b(dead|deceased|expectant)\b", re.I)
SWAP_OVERRIDE = re.compile(r"\b(skip(ping)?|avoid(ing)?|without|no need for|overrid(e|ing)|postpon(e|ing)|"
                           r"defer(ring)?|cancel(ling)?)\b( (a|the|any))? (battery )?swap", re.I)
ZONE_ACTION = re.compile(r"\b(i|we) (have |will )?(task|create|add|activate|open)(ed|d)?\b|"
                         r"\b(tasked|activated|created|added|opened) (zone |proposal )?[PSZ]\d+\b|"
                         r"\b[PSZ]\d+ (is|has been|was) (now )?(tasked|activated|added)\b|\bnow tasking\b", re.I)
POD_LANDED = re.compile(r"\blanded\b|\bland(s|ing)? (on|onto|at) (the )?(rubble|pile|[A-C][1-3]\b|it\b)", re.I)


def _sentences(text):
    return [s for s in re.split(r"(?<=[.!?;:\n])\s+|\n|\s+[-—]\s+", text or "") if s.strip()]


def wording_problem(text, pod=False):
    """Return why ``text`` breaks the wording rules (str), or None if it is fine."""
    if BANNED.search(text or ""):
        return "banned claim (confirmed / found / diagnosis)"
    if FACT_CLAIM.search(text or ""):
        return "states a condition as fact"
    if SWAP_OVERRIDE.search(text or ""):
        return "overrides the forced battery swap"
    if ZONE_ACTION.search(text or ""):
        return "claims a zone action (zones are the operator's call)"
    if pod and POD_LANDED.search(text or ""):
        return "says landed; the pod is lowered onto the rubble"
    for s in _sentences(text):
        if VITALS_CTX.search(s) and AGE_WORDS.search(s):
            return "age inference in a breathing sentence"
    return None


# --------------------------------------------------------------------------- vitals -> plain words
def condition_phrase(v, first=False):
    """Plain-words breathing condition with 'check' ("irregular breathing with pauses — check first").

    Built from the pattern only (never the producer's free text), so no age or diagnosis can slip in.
    A triage aid for rescuers ordering the dig, not a diagnosis."""
    if not isinstance(v, dict) or not v:
        return "no breathing pattern measured — check on reaching it"
    pat = v.get("pattern") or "unclear"
    rate = v.get("rate_bpm")
    r = f" (~{rate:.0f}/min)" if isinstance(rate, (int, float)) else ""
    hint = str(v.get("triage_hint") or "assess")
    if pat == "irregular_with_pauses":
        what = "irregular breathing with pauses" if (v.get("apnea_events") or 0) > 0 or \
            (v.get("apnea_events_per_min") or 0) > 0 else "irregular breathing"
    elif pat == "rapid_shallow":
        what = "rapid shallow breathing"
    elif pat in ("slow_regular", "slow_deep"):
        what = "slow, regular breathing"
    elif pat == "regular_calm":
        what = "calm, regular breathing"
    elif pat == "fast_small":                           # legacy vitals-v1 class: condition only, no age
        what = "fast, regular breathing"
    else:
        return "breathing pattern unclear — check, rescan longer if possible"
    if hint.startswith("immediate"):
        tail = "check first" if first else "check early"
    elif hint == "urgent":
        tail = "check soon"
    elif hint.startswith("delayed"):
        tail = "check, then re-check later"
    else:
        tail = "check"
    conf = v.get("pattern_conf")
    low = isinstance(conf, (int, float)) and conf < 0.3
    short = isinstance(v.get("duration_s"), (int, float)) and v["duration_s"] < 45
    note = "; low confidence" if low else ""
    note += "; short scan, rate only" if short else ""
    return f"{what}{r} — {tail}{note}"


def _vitals_summary(v):
    if not isinstance(v, dict):
        return None
    out = {k: v[k] for k in VITALS_KEEP if v.get(k) is not None}
    out["condition"] = condition_phrase(v)
    return out


def dig_order(candidates, depth_of=None):
    """Firm pings ranked for the ground team: vitals dig_score (triage weight x p_mass x accessibility),
    ties by p_mass. Pings without dig_score get the 'assess' weight x p_mass x accessibility.
    Returns [{"rank", "ping", "pile", "dig_score", "pattern", "triage_hint", "condition"}]. Pure code."""
    rows = []
    for c in candidates or []:
        if c.get("status") != "candidate" or c.get("kind", "stationary") != "stationary":
            continue
        v = c.get("vitals") if isinstance(c.get("vitals"), dict) else None
        s = c.get("dig_score")
        if not isinstance(s, (int, float)):
            d = c.get("depth_m")
            if d is None and depth_of is not None:
                d = depth_of(c)
            acc = 1.0 if not isinstance(d, (int, float)) else min(1.0, max(0.2, 1 - d / 4.0))
            s = round(ASSESS_WEIGHT * float(c.get("p_mass") or 0.0) * acc, 3)
        rows.append({"ping": c.get("id"), "pile": c.get("pile_id"), "dig_score": s,
                     "p_mass": c.get("p_mass"), "pattern": (v or {}).get("pattern"),
                     "triage_hint": (v or {}).get("triage_hint"), "_v": v})
    rows.sort(key=lambda r: (-r["dig_score"], -(r["p_mass"] or 0), str(r["ping"])))
    for i, r in enumerate(rows, 1):
        r["rank"] = i
        r["condition"] = condition_phrase(r.pop("_v"), first=i == 1)
    return rows


# --------------------------------------------------------------------------- prompt
SYSTEM = """You are the dispatch judgement of the mission commander for an urban search-and-rescue
drone team searching rubble piles (A1..C3). At a pile the radar is delivered either by a drone landing
next to the pile or (delivery "pod") by the heavy drone lowering a radar+microphone pod on its winch
onto the rubble surface, 2-4 placements per lead. After each pile the drones listen (rotors off) and
rank the other piles by sound. You now choose what the fleet does next.

You see only observable evidence: acoustic transcripts, sound classes and scores, radar ping statuses
and breathing-pattern summaries, drone batteries, travel times, the pile and scan budgets and operator
notes. The numeric planner has already scored the options; you choose one, using judgement the
formulas lack: operator notes, competing leads, battery margins, weak or contradictory evidence.

Actions: dispatch = fly to an unvisited pile; revisit = go back once to an open lead that got no firm
radar fix (fresh placements add independent ranges); rescan_current = re-run the radar here; stop =
return to the staging area (remaining piles stay not cleared).

Rules:
- Choose exactly one option id from the list. Do not invent options.
- Never choose an option flagged below_reserve, no_go or unreachable.
- battery_swap on an option is forced by the 25% reserve rule: the fleet first returns to the pads for
  a battery swap. It is not your choice; never skip or override it. Mention it in the rationale. Do not
  prefer or avoid a pile because of the swap alone.
- pile_budget_used / scan_budget_used on stop: the budget is spent; stop.
- Sound on an ignore_sound pile is our own crew or machinery, not a lead.
- Priority is triage order, not proof. Never say a person is confirmed or found; radar pings are
  candidate search regions. Silent or searched piles are not cleared.
- Breathing patterns are a triage aid for rescuers to check, not a diagnosis. Describe only the
  breathing (rate, regular / irregular, pauses). Never infer age (no child, infant, elderly) and never
  state injury or consciousness as fact.
- With pod delivery say "pod lowered onto rubble at ...", never "landed".
- Zones and zone proposals are the operator's call: you may only suggest ("suggest tasking P1").
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


def _xy(p):
    if isinstance(p, dict):
        p = p.get("xy")
    try:
        return [round(float(p[0]), 1), round(float(p[1]), 1)]
    except (TypeError, IndexError, ValueError):
        return None


def placements(o):
    """Pod placements listed on an option (``placements`` or ``pod_placements``) as [[x, y], ...]."""
    raw = o.get("placements") or o.get("pod_placements") or []
    return [xy for xy in (_xy(p) for p in raw) if xy is not None]


def is_pod(context, options=()):
    m = context.get("mission") if isinstance(context.get("mission"), dict) else {}
    return (context.get("delivery") or m.get("delivery")) == "pod" or any(placements(o) for o in options)


def compact_context(context):
    """Prompt-sized context: vitals cut to a summary (no caveat prose), zones kept as suggestions input."""
    ctx = scrub(context)
    for p in ctx.get("pings") or []:
        if isinstance(p, dict) and "vitals" in p:
            p["vitals"] = _vitals_summary(p["vitals"])
    if ctx.get("pings") and any(isinstance(p, dict) and ("vitals" in p or "dig_score" in p) for p in ctx["pings"]):
        ctx["dig_order"] = [{k: r[k] for k in ("rank", "ping", "dig_score", "condition")}
                            for r in dig_order([dict(p, status=p.get("status", "candidate")) for p in ctx["pings"]])]
    return ctx


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


def _adjust(p, o):
    if p is None:
        return None
    if "sound_ignored" in (o.get("flags") or []):
        p = min(p, SILENT_PRIORITY)
    return p * float(o.get("operator_prior", 1.0))


def effective_priority(o):
    """Acoustic priority after operator notes (threshold test)."""
    return _adjust(o.get("priority"), o)


def effective_score(o):
    """Ranking value after operator notes: the mission's ``score`` (revisits are discounted), else the
    priority for a plain dispatch. None = not rankable (e.g. an unscored rescan)."""
    s = o.get("score")
    if s is None and o.get("action") == "dispatch":
        s = o.get("priority")
    return _adjust(s, o)


def _notes(o, pod):
    """Plain sentences the mission's constraints force onto a chosen option."""
    out = []
    if "battery_swap" in (o.get("flags") or []):
        out.append(f"Battery swap at the pads first: every drone must keep a {100 * RESERVE:.0f}% reserve "
                   "(forced by the reserve rule).")
    pl = placements(o)
    if pod and o.get("action") in MOVE_ACTIONS and pl:
        out.append(f"Pod lowered onto rubble at {len(pl)} placements: "
                   + ", ".join(f"({x}, {y})" for x, y in pl[:4]) + ".")
    return out


def fallback_choice(options, priority_min=20, pod=False):
    """Highest effective-score valid move (dispatch / revisit / scored rescan) whose effective priority
    is at or above priority_min, else stop."""
    ok = [o for o in options if o.get("action") in MOVE_ACTIONS and not _invalid(o)
          and effective_score(o) is not None and (effective_priority(o) or 0) >= priority_min]
    ok.sort(key=lambda o: (-effective_score(o), o.get("action") != "dispatch", str(o.get("pile_id"))))
    if ok:
        o = ok[0]
        adj = "" if effective_priority(o) == o["priority"] else f", {effective_priority(o):.0f} after operator notes"
        if o["action"] == "dispatch":
            why = f"highest valid priority: {o['pile_id']} ({o['priority']}{adj})."
        else:
            why = (f"{'revisit' if o['action'] == 'revisit' else 'rescan'} {o['pile_id']}: its lead (priority "
                   f"{o['priority']}{adj}) is still open without a firm radar fix and no unvisited pile scores higher.")
        return o["id"], " ".join([why] + _notes(o, pod))
    stop = next((o for o in options if o.get("action") == "stop"), None)
    if stop is not None:
        fl = set(stop.get("flags") or [])
        if "scan_budget_used" in fl:
            why = "scan budget used; return."
        elif "pile_budget_used" in fl:
            why = "pile budget used; return."
        else:
            why = f"no valid pile at or above priority {priority_min}; return."
        return stop["id"], why + " Remaining piles stay not cleared."
    valid = [o for o in options if not _invalid(o)]
    return (valid or options)[0]["id"], "no stop option listed; first valid option."


def build_prompt(context, options):
    ctx = compact_context(context)
    opts = scrub(options)
    doc = {"prompt_version": PROMPT_VERSION, "reserve_battery": RESERVE,
           "delivery": "pod" if is_pod(context, options) else "land", "situation": ctx, "options": opts}
    return ("Situation and options (JSON). Reply with JSON {\"choice\": <option id>, \"rationale\": <text>}.\n"
            + json.dumps(doc, sort_keys=True, ensure_ascii=False))


def _clean_rationale(r):
    r = " ".join(str(r or "").split())
    sents = re.split(r"(?<=[.!?])\s+", r)
    r = " ".join(sents[:2])
    return r[:320]


def validate(out, options, pod=False):
    """Return (choice, rationale) or (None, reason). Forced notes (battery swap, pod placements) are
    appended to an accepted rationale when it does not already mention them."""
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
    bad = wording_problem(r, pod)
    if bad:
        return None, f"rationale makes a banned claim: {bad}"
    for n in _notes(ids[c], pod):
        if ("swap" in n and "swap" not in r.lower()) or ("Pod" in n and "pod" not in r.lower()):
            r += " " + n
    return c, r


# --------------------------------------------------------------------------- zone suggestions
ZONE_SCHEMA = {"type": "object", "properties": {"proposal": {"type": "string"}, "text": {"type": "string"}},
               "required": ["proposal", "text"], "additionalProperties": False}
ZONE_PROMPT = """You advise the operator of a search-and-rescue drone team. The operator tasks search
zones; you only suggest. Pick the one proposal (rubble mound or acoustic zone suggestion) the operator
should task next, from the list below, weighing acoustic leads, score and size. Reply JSON
{"proposal": <id>, "text": "suggest tasking <id>: <one short reason citing the evidence>"}.
Never say a zone was tasked, never say a person is confirmed or found. Untasked areas are not cleared.

Proposals (JSON):
"""


def zone_options(state):
    """Untasked zone proposals (P*) and outside-zone acoustic suggestions (S*) from state."""
    zones = state.get("zones") or []
    taken = {str(z.get(k)) for z in zones for k in ("id", "proposal", "from_proposal", "source_id") if z.get(k)}
    out = []
    for p in state.get("zone_proposals") or []:
        if p.get("id") and str(p["id"]) not in taken:
            out.append({"id": p["id"], "kind": p.get("kind", "rubble_mound"), "score": p.get("score"),
                        "acoustic_leads": len(p.get("acoustic_leads") or []), "area_m2": p.get("area_m2"),
                        "height_max_m": p.get("height_max_m"), "why": p.get("why")})
    for s in state.get("zone_suggestions") or []:
        if s.get("id") and str(s["id"]) not in taken:
            out.append({"id": s["id"], "kind": "acoustic_suggestion", "score": s.get("score"),
                        "acoustic_leads": len(s.get("evidence") or []) or 1, "radius_m": s.get("radius_m"),
                        "why": s.get("why")})
    return out


def _zone_fallback(opts):
    best = sorted(opts, key=lambda o: (-(o["acoustic_leads"] > 0), -(o["score"] or 0), str(o["id"])))[0]
    bits = [b for b in (best.get("why"), f"score {best['score']}" if best.get("score") is not None else None) if b]
    return best["id"], f"suggest tasking {best['id']}: " + ("; ".join(bits) or best["kind"].replace("_", " ")) + "."


def suggest_zone(state, provider=None):
    """Which proposal the operator might task next. Returns None (nothing to suggest) or
    {"proposal", "text", "by"}; the text always starts "suggest tasking". Never tasks anything."""
    opts = zone_options(scrub(state))
    if not opts:
        return None
    ids = {o["id"] for o in opts}

    def ok(o):
        t = str(o.get("text") or "").strip()
        return o.get("proposal") in ids and t.lower().startswith(f"suggest tasking {o['proposal'].lower()}") \
            and len(t) <= 280 and wording_problem(t) is None

    if llm.available(provider):
        meta = {}
        out = llm.call_json(ZONE_PROMPT + json.dumps(opts, sort_keys=True, ensure_ascii=False), ZONE_SCHEMA,
                            provider=provider, max_tokens=512, meta=meta, validate=ok)
        if isinstance(out, dict) and ok(out):
            return {"proposal": out["proposal"], "text": " ".join(out["text"].split()),
                    "by": (meta.get("provider") or "llm") + ("-replay" if meta.get("cached") else "")}
    pid, text = _zone_fallback(opts)
    return {"proposal": pid, "text": text, "by": "fallback"}


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
        pod = is_pod(context, opts)
        ctx = dict(context)
        if constraints.get("notes"):
            ctx["operator_notes"] = {k: constraints[k] for k in
                                     ("notes", "pile_priors", "ignore_sound_piles", "no_go_piles")}
        meta, err = {}, None
        out = None
        if llm.available(provider):
            out = llm.call_json(build_prompt(ctx, opts), SCHEMA, system=SYSTEM, provider=provider,
                                max_tokens=1024, meta=meta,
                                validate=lambda o: validate(o, opts, pod)[0] is not None)
            err = meta.get("error")
        else:
            err = "no LLM configured"
        choice, why = validate(out, opts, pod) if out is not None else (None, err or "LLM failed")
        res = {"latency_ms": round((time.time() - t0) * 1000, 1), "provider": prov, "model": model}
        if choice is not None:
            res.update(choice=choice, rationale=why,
                       by=(meta.get("provider") or prov) + ("-replay" if meta.get("cached") else ""))
        else:
            fc, fwhy = fallback_choice(opts, pmin, pod)
            res.update(choice=fc, rationale=f"Fallback ({why}): {fwhy}", by="fallback", fallback_reason=why)
        chosen = next(o for o in opts if o["id"] == res["choice"])
        if "battery_swap" in chosen["flags"]:
            res["battery_swap"] = True                 # forced by the reserve rule, explained in the rationale
        return res

    decide.constraints = constraints
    decide.suggest_zone = lambda state: suggest_zone(state, provider=provider)
    decide.dig_order = dig_order
    return decide
