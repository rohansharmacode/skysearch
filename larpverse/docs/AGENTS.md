# LLM agents (Gemini first)

Design: [agent-decisions.html](agent-decisions.html). Numbers propose, the LLM decides, code verifies,
and a deterministic fallback runs whenever the LLM is missing, slow, invalid or breaks a rule.
`agents/` is stdlib only (urllib REST, no SDK), so it imports in `.venv`, `.venv-audio` and Isaac's Python.

## What the LLM decides

| Where | Module | LLM output | Code check / fallback |
|---|---|---|---|
| Acoustic triage, per clip | `acoustic/agent.py` | human, urgency, reliability, rationale | range [0, 1]; else keyword/score fallback |
| Dispatch gate after each listen (`sim/grid_mission.py --decider gemini`) | `agents/gemini_decider.py` | one option id + rationale (max 2 sentences) | id listed; not `below_reserve` (battery after < 25%), `no_go` or `unreachable`; no "confirmed"/"survivor found"; else highest-priority valid option, else stop (`by: "fallback"`) |
| Operator notes (`ops/intel.txt`) | `agents/intake.py` | pile priors, ignored-sound piles, no-go piles | multipliers clamped to [0.3, 3], unknown piles dropped; keyword fallback |
| Rescuer handover | `agents/brief.py` | rewrite of template facts | no banned claims, no unknown pile ids, every ping id kept; else the template |

## What it never does

- Sees scene ground truth: prompts are built from observable state only; keys such as `truth` / `targets`
  are stripped and a test asserts they never reach a prompt.
- Flies drones, touches the radar/Bayes maths, or invents options. It picks from the planner's list.
- Says a person is confirmed. Radar pings are candidate search regions; silent piles are not cleared.
- Breaks the mission: every failure returns `None` and the deterministic path runs.

## Enable Gemini

Put the key in `.env` at the repo root (gitignored; never commit it):

```
GEMINI_API_KEY=...
# optional
GEMINI_MODEL=gemini-3.7-flash      # decider + brief; audio uses GEMINI_AUDIO_MODEL=gemini-3.5-flash-lite (free tier: ~20 requests/day per model)
LLM_PROVIDER=gemini          # gemini | anthropic | none (default: gemini if its key is set, else anthropic, else none)
```

```
.venv/bin/python -m sim.grid_mission --seed 7 --decider gemini --out dashboard/data   # LLM at the dispatch gate
.venv/bin/python -m sim.grid_mission --seed 7 --decider none                          # pure argmax
.venv/bin/python -m agents.intake --notes "Bedrooms were on the east side of A3"     # test notes
.venv/bin/python -m agents.brief --state dashboard/data/state.json                    # handover brief
```

Without a key everything still runs: `--decider gemini` logs `by: "fallback"` decisions.

## Record / replay

Every successful call is cached in `agents/cache/llm.jsonl` (gitignored), keyed by provider + model + prompt.
Rerunning the same seed is free and needs no network. `LLM_REPLAY=1` forces replay-only (cache or
fallback, never the network) for a live demo. The acoustic judge keeps its own per-clip cache
(`acoustic/cache/decisions.jsonl`).

## Cost and latency

One call per dispatch gate (2 per 3-pile mission), one per listen for acoustic triage, one for intake,
one for the brief: about 5 calls and ~10k input tokens per mission, a fraction of a cent on Flash.
Expect ~1-4 s per call; timeout 25 s (acoustic 90 s), one retry on 429/5xx. The sim clock waits
for the decider; with replay it costs milliseconds.

Tests (fake HTTP, no network): `.venv/bin/python -m pytest -q tests/test_agents.py`.
