# Seed-7 story (canonical)

This is the run every demo, report and video should quote. It is deterministic: the same seed and flags give the same story. `tests/test_determinism.py` checks this across `PYTHONHASHSEED` values.

- **Code:** git HEAD `ad27b12` (plus this doc).
- **Acoustics:** the committed real-pipeline fixture `tests/fixtures/acoustic_seed7.json` (assoc-v11), replayed the same way as `tests/test_demo_story.py`.
- **Radar and flight defaults:** `--radar dsp --penetration literature --delivery pod --platform heavy_sar --flight smooth`, physio chest waveform, 45 s pod scans, 60 s vitals look, comms on, speaker on.

```python
# reproduce (same as tests/test_demo_story.py)
from test_demo_story import replay_fn            # tests/ on sys.path
from sim.grid_mission import GridMission
fn, _ = replay_fn()
m = GridMission(seed=7, acoustic_fn=fn, tasking="default").run()
```

The CLI equivalent with the live acoustic pipeline is below. Its acoustics come from `.venv-audio` or the cache instead of the fixture, so the clips can differ.

```bash
scripts/heavy.sh .venv/bin/python -m sim.grid_mission --seed 7 --tasking default --out dashboard/data
```

## Result

| ping | t (s) | where | vitals condition | dig score | depth |
|---|---|---|---|---|---|
| A2-R1 | 364 | (-21.9, 2.2) | `unconscious_slow`: "slow, regular breathing, 9/min … call out and check responsiveness" | 0.415 | 1.62 ± 0.45 m (radar estimate, 5 scans) |
| B1-R1 | 1712 | (1.5, 20.6) | `irregular_apneic`: "irregular breathing with 1 pause(s) over 10 s, 9/min … check" | 0.467 | nominal 1.5 m (σ 0.45) |
| B2-R1 | 2312 | (-2.4, -2.0) | `calm`: "calm, regular breathing, 13/min … re-check later" | 0.280 | nominal 1.5 m (σ 0.6) |

- **Visits:** A2 → A3 → B1 → B2 → A3 (revisit).
- **Open lead:** A3, priority 85. The radar has insufficient evidence after 16 scans.
- **Not cleared:** A1 and C3 (nothing heard); C1, C2 and B3 (non-human sound).
- **Totals:** 34 of 36 radar scans, 1 battery swap, mission time **3109 s**, 3511 drone-seconds of flight.
- Every vitals condition matches the hidden physio state. Pings are candidate search regions, not confirmed people.

## Sensitivity (honest note)

Detection is marginal under realistic penetration with real chest motion. Reach is about 2.9 m of rubble beneath a 45 s pod scan (docs/RADAR_PENETRATION.md), so pod seating decides some outcomes:

- **A2 is robust.** It pinged in every run variant: kinematic and smooth flight, with and without comms. It has 5 supporting scans.
- **B1 and B2 depend on seating.** Each rests on exactly 3 supporting scans, the minimum for a fix. A different scan schedule changes which seat draws a rough touchdown, and so whether the third strong range arrives within the budget.
  - Kinematic flight: B1 and B2 both ping.
  - Before the determinism fix, one hash-seed ordering lost B2: a different drone was picked for B1's vitals look, which shifted every later scan.
- **A3 does not ping in any variant.** Its victim breathes rapidly and shallowly (33/min, about 2.2 mm of chest motion), and its closest seats rock (quality 0.03–0.6).

Any change to flight timing, the scan budget or placement order can move B1 and B2. Re-run this file's command and update the table rather than quoting older numbers.
