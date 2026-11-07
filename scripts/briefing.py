"""Crew handover briefing written by Gemini from what the swarm found (one API call per mission).

The drones never wait for this: flight and search decisions stay on board. At handoff the
coordinator's structured findings (victims, confidence, evidence, rooms not searched, hazards,
route lengths) are turned into a short radio-style briefing for the rescue crew.
Without GEMINI_API_KEY, or if the call fails, the built-in template briefing is used.

    GEMINI_API_KEY=... python scripts/briefing.py --seed 3        # runs the storyboard, prints both
"""
from __future__ import annotations
import argparse, json, os, sys, urllib.request

MODEL = os.environ.get("GEMINI_MODEL", "gemini-3.5-flash")
URL = "https://generativelanguage.googleapis.com/v1beta/models/{m}:generateContent"

PROMPT = """You are the incident-command assistant for an urban search-and-rescue drone team.
Write the handover briefing the team leader reads to the fire crew over the radio before they
enter an earthquake-damaged building. Use ONLY the facts in the JSON below; never invent victims,
rooms, distances or hazards. Keep each confidence level and say what evidence it rests on.
Format: at most 6 short lines, plain text, no markdown, no bullet symbols. One line per victim
(label, room, confidence, evidence, route length, access), then a line on the
unsearched area and missing people, then one line on hazards. Under 110 words in total.

FACTS:
{facts}"""


def facts(fleet):
    b = fleet.b
    return {
        "mission_time_s": round(fleet.t),
        "unaccounted_at_start": fleet.n_unaccounted,
        "victims": [dict(label=f["label"], room=f["room"], confidence=f["conf"], evidence=f["evidence"],
                         void_height_m=round(float(f["void"]), 2),
                         route_from_west_entrance_m=None if f["route"] != f["route"] else round(float(f["route"])),
                         access="breach 1 obstacle at the end" if f["breach"] else "open crawl route, no breach")
                    for f in fleet.found],
        "rooms_not_fully_searched": [k for k, v in fleet.room_status.items() if v in ("unsearched", "searching")],
        "rooms_status": {str(k): v for k, v in fleet.room_status.items()},
        "people_still_missing": fleet.n_unaccounted - len(fleet.found),
        "smoke": "heavy smoke in the east wing; LiDAR and cameras blind there, mapped by radar only" if b.fires else None,
        "fire": [dict(room=int(b.room_id[b.cell_of(fr["xy"])]), wing=b.wing_of(fr["xy"])) for fr in b.fires],
        "drones_lost": [d.name for d in fleet.drones if not d.alive],
        "safe_route_rule": "crawlable (>= 0.6 m high), outside heavy smoke",
    }


def gemini_briefing(fleet, model=MODEL, timeout=25):
    """Returns briefing lines from Gemini, or None (no key, network error, empty answer)."""
    key = os.environ.get("GEMINI_API_KEY")
    if not key:
        return None
    body = {"contents": [{"role": "user", "parts": [{"text": PROMPT.format(facts=json.dumps(facts(fleet), indent=1))}]}],
            "generationConfig": {"temperature": 0.2, "maxOutputTokens": 4096}}
    req = urllib.request.Request(URL.format(m=model), data=json.dumps(body).encode(), method="POST",
                                 headers={"Content-Type": "application/json", "x-goog-api-key": key})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            out = json.load(r)
        parts = out["candidates"][0]["content"]["parts"]
        if os.environ.get("BRIEFING_DEBUG"):
            print(json.dumps(out, indent=1)[:3000], file=sys.stderr)
        text = "".join(p.get("text", "") for p in parts if not p.get("thought")).strip()
    except Exception as e:                       # never let the briefing break the mission
        print(f"[briefing] Gemini unavailable ({type(e).__name__}); using the template", file=sys.stderr)
        return None
    lines = [ln.strip(" -•*") for ln in text.splitlines() if ln.strip()]
    return lines or None


def apply(fleet, model=MODEL):
    """Replace the template briefing (after handoff) with Gemini's, keeping the header line."""
    if not fleet.briefing:
        return False
    lines = gemini_briefing(fleet, model)
    if not lines:
        return False
    fleet.briefing = [fleet.briefing[0] + f"  ·  written by {model}"] + lines
    fleet.say(f"Crew briefing drafted by {model} from the swarm's findings (facts only)")
    return True


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from skysearch.usar.config import USARConfig
    from skysearch.usar.scenario import USARScenario
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--model", default=MODEL)
    a = ap.parse_args()
    s = USARScenario(USARConfig(story=True, seed=a.seed))
    while s.fleet.briefing is None:
        s.step()
    print("\n".join(s.fleet.briefing), "\n")
    print("GEMINI:" if apply(s.fleet, a.model) else "(no Gemini key / call failed: template kept)")
    print("\n".join(s.fleet.briefing))
