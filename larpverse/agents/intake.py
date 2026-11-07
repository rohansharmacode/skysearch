"""Operator-notes intake (decision opportunity C): free text -> structured, clamped constraints.

    {"pile_priors": {"A3": 2.0}, "ignore_sound_piles": ["B2"], "no_go_piles": ["C1"],
     "notes": "...", "by": "gemini" | "keyword" | "none"}

The LLM reads the incident commander's notes with a schema; code clamps every multiplier to
[0.3, 3], drops unknown pile ids, and falls back to keyword rules if the LLM is unavailable.
Notes come from ``ops/intel.txt`` (or env OPS_INTEL). These are operator claims, not sensor
evidence: they reweight priorities and veto piles, they never add a radar or acoustic detection.

    python -m agents.intake [--notes "..."] [--file ops/intel.txt]
"""
from __future__ import annotations

import argparse
import json
import os
import re

from . import llm

PILES = [f"{c}{r}" for r in (1, 2, 3) for c in "ABC"]
PRIOR_MIN, PRIOR_MAX = 0.3, 3.0
INTEL_PATH = os.path.join(llm.ROOT, "ops", "intel.txt")
PROMPT_VERSION = "intake-v1"

SCHEMA = {
    "type": "object",
    "properties": {
        "pile_priors": {"type": "array", "items": {
            "type": "object",
            "properties": {"pile": {"type": "string"}, "multiplier": {"type": "number"},
                           "reason": {"type": "string"}},
            "required": ["pile", "multiplier", "reason"], "additionalProperties": False}},
        "ignore_sound_piles": {"type": "array", "items": {"type": "string"}},
        "no_go_piles": {"type": "array", "items": {"type": "string"}},
    },
    "required": ["pile_priors", "ignore_sound_piles", "no_go_piles"],
    "additionalProperties": False,
}

PROMPT = """You turn an incident commander's notes into constraints for a drone search over a 3x3 grid
of rubble piles named A1..C3 (columns A/B/C west to east, rows 1/2/3 north to south).

Return JSON with:
- pile_priors: piles the notes make more or less likely to hold people, each with a multiplier in
  [0.3, 3] (above 1 = more likely, e.g. occupied rooms, last-known positions; below 1 = less likely,
  e.g. evacuated or empty) and a short reason quoting the note.
- ignore_sound_piles: piles where the notes say sound is from our own crews or machinery, so
  acoustic detections there should not count as leads.
- no_go_piles: piles the notes say drones must not land on (unstable, gas, collapse risk, exclusion).

Use only pile ids that the notes mention. Do not invent piles or facts. Empty lists are fine.

Notes:
"""

_PRIOR_UP = r"bedroom|occupied|family|famil|resident|people|person|someone|last seen|last known|" \
            r"living room|kitchen|classroom|office|shift|sleeping|likely|reported"
_PRIOR_DOWN = r"empty|unoccupied|evacuated|vacant|storage|garage|nobody|no one|accounted for|cleared by"
_IGNORE = r"our crew|our team|digging|excavat|machinery|generator|pump|jackhammer|ignore (the )?(noise|sound)|" \
          r"ignore noise|own crew|rescue team at|firefighters at"
_NOGO = r"no[- ]go|unstable|do not land|don'?t land|avoid|keep out|gas|collapse risk|exclusion|unsafe|structural"


def empty(notes="", by="none"):
    return {"pile_priors": {}, "ignore_sound_piles": [], "no_go_piles": [], "notes": notes, "by": by}


def clamp(raw, notes="", by="llm", valid=PILES):
    """Validate and clamp an LLM (or keyword) result into the canonical structure."""
    out = empty(notes, by)
    valid = set(valid)
    pri = raw.get("pile_priors") if isinstance(raw, dict) else None
    items = pri.items() if isinstance(pri, dict) else \
        ((d.get("pile"), d.get("multiplier")) for d in pri if isinstance(d, dict)) if isinstance(pri, list) else ()
    for pile, mult in items:
        pile = str(pile or "").strip().upper()
        try:
            m = float(mult)
        except (TypeError, ValueError):
            continue
        if pile in valid and m == m:
            out["pile_priors"][pile] = round(min(PRIOR_MAX, max(PRIOR_MIN, m)), 2)
    for k in ("ignore_sound_piles", "no_go_piles"):
        vals = raw.get(k) if isinstance(raw, dict) else None
        if isinstance(vals, list):
            out[k] = sorted({str(v).strip().upper() for v in vals if str(v).strip().upper() in valid})
    return out


def keyword_intake(notes, valid=PILES):
    raw = {"pile_priors": {}, "ignore_sound_piles": [], "no_go_piles": []}
    for sent in re.split(r"(?<=[.;!?\n])\s*", notes or ""):
        s = sent.lower()
        piles = [p for p in re.findall(r"\b([abc][123])\b", s, flags=re.I)]
        piles = [p.upper() for p in piles if p.upper() in valid]
        if not piles:
            continue
        for p in piles:
            if re.search(_NOGO, s):
                raw["no_go_piles"].append(p)
            if re.search(_IGNORE, s):
                raw["ignore_sound_piles"].append(p)
            elif re.search(_PRIOR_DOWN, s):
                raw["pile_priors"][p] = 0.5
            elif re.search(_PRIOR_UP, s):
                raw["pile_priors"][p] = 2.0
    return clamp(raw, notes, "keyword", valid)


def intake(notes, provider=None, valid=PILES):
    """Free text -> constraints. LLM first (schema + clamp), keyword fallback. Never raises."""
    notes = (notes or "").strip()
    if not notes:
        return empty()
    meta = {}
    raw = llm.call_json(PROMPT + notes, SCHEMA, provider=provider, max_tokens=1024, meta=meta,
                        validate=lambda o: all(k in o for k in SCHEMA["required"]))
    if isinstance(raw, dict) and all(k in raw for k in ("pile_priors", "ignore_sound_piles", "no_go_piles")):
        by = meta.get("provider") or "llm"
        return clamp(raw, notes, by + ("-replay" if meta.get("cached") else ""), valid)
    return keyword_intake(notes, valid)


def read_notes(path=None) -> str:
    path = path or os.environ.get("OPS_INTEL") or INTEL_PATH
    try:
        with open(path) as fh:
            return fh.read().strip()
    except OSError:
        return ""


def load(path=None, provider=None):
    """Read ops/intel.txt (if present) and return its constraints (empty structure if absent)."""
    return intake(read_notes(path), provider=provider)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--notes", default=None)
    ap.add_argument("--file", default=None)
    ap.add_argument("--provider", default=None, choices=list(llm.PROVIDERS))
    a = ap.parse_args(argv)
    notes = a.notes if a.notes is not None else read_notes(a.file)
    print(json.dumps(intake(notes, provider=a.provider), indent=2))


if __name__ == "__main__":
    main()
