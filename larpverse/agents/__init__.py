"""LLM decision layer (stdlib only, so it imports in any venv and in Isaac's Python).

  llm.py            provider layer: call_json(prompt, schema) via Gemini or Anthropic REST, record/replay cache
  gemini_decider.py dispatch-gate decider for sim/grid_mission.py (numbers propose, agent decides, code verifies)
  intake.py         operator notes -> pile priors, ignored-sound piles, no-go piles
  brief.py          rescuer handover brief from the final state.json
"""
