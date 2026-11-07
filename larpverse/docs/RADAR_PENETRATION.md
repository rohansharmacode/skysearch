# Radar penetration through rubble

Short answer to "does 10 m of concrete kill the idea?": yes. Under the `literature` model, 10 m of rubble costs about 130 dB two-way, so nothing comes back. A good scan sees a breathing chest through about **3 m of rubble directly beneath it**. From an 11 m stand-off it sees through **less than 1 m**, because 1/R^4 spreading eats the margin.

Code: `sim/radar_dsp.py` (`two_way_loss_db`, `reach_path_m`, `reach_prob`, `use_penetration`). The synthetic IF generator, the model radar, the planner and the coverage map all use the same functions. Tests: `tests/test_radar_dsp.py`.

## Model

- **Band.** 0.4–2.0 GHz sweep: 1.6 GHz of bandwidth, 9.4 cm range bins, 512 bins, 48 m unambiguous range. The sub-1 GHz part of the sweep carries the deep returns.
- **Concrete loss** (one-way, solid). `alpha_c(f)` is about 10 dB/m below 1 GHz and rises linearly to 20 dB/m at 2 GHz. This is the FHWA GPR rule of thumb.
- **Solid path.** `rubble_path` is the length of the sensor→target segment that lies below the LiDAR heightmap surface. Of that length, `SOLID_FRACTION` = 0.65 counts as solid material; the rest is voids ("voids help"). An optional `REBAR_DB_PER_M` adds loss for rebar ("rebar hurts"); it defaults to 0.
- **Two-way loss.** Power is averaged over the sweep: `loss(L) = -10 log10 mean_f 10^(-2 L (0.65 alpha_c(f) + rebar) / 10)`. This gives 15 dB at 1 m, 29 dB at 2 m, 43 dB at 3 m and 69 dB at 5 m.
- **Spreading.** Chest amplitude per IF sample is `AMP_1M / R^2 · 10^(-loss/20)`, with `AMP_1M` = 70 (unit complex noise).
- **Detection.** No shortcut is taken here. The DSP chain (range FFT → clutter removal → respiration-band OS-CFAR → phase → band-pass) decides whether there is a detection. It was calibrated by measuring the chest amplitude at which 90% of scans detect at 6 m. The results, `A90_DB` (12 scans per 3 dB step), are −17 dB at landing quality 0.95, −15 dB at 0.6 and −11 dB at 0.3.
- **Reach.** `reach_path_m(range, quality)` is the rubble path at which the margin `20 log10(AMP_1M/R^2) − A90(q)` equals `loss(L)`.

  | slant range | 2 m | 4 m | 6 m | 9 m | 11.5 m |
  |---|---|---|---|---|---|
  | reach at q 0.95 | 2.95 m | 2.05 m | 1.53 m | 1.02 m | 0.73 m |

- **Per-scan field.** Every scan reports `max_reliable_depth_m` = `reach_path_m(2 m, measured quality)`, the rubble it could see through directly beneath. A target beyond reach returns *no detection*, and no detection is not absence.
- **Per pile.** `piles[].radar_coverage` and `radar_coverage_shallow` give the share of the pile volume (cells within 9 m that are more than 0.4 m above ground, sampled every 0.5 m down to the ground) that some scan reached, for the whole pile and for its top 2 m. `radar_reach_m` is the median per-scan reach. When the top-2 m coverage is below 0.5, a pile with no ping is reported as "beyond radar reach → acoustic lead only" and is not revisited.

## Sources (numbers used)

- Concrete attenuation of about 10 dB/m below 1 GHz and about 20 dB/m above. Source: the FHWA ground-penetrating radar rule of thumb for concrete (bridge deck / pavement GPR guidance).
- Through-rubble life detection:
  - The EU RADIOTECT project reports detection through roughly 1–2 m of reinforced concrete, and up to about 2.7 m in field tests.
  - Commercial UWB life detectors claim up to about 4.6 m (15 ft) through debris.
  - Typical reported operational depth is 0.5–3 m of mixed rubble.
- Void fraction of collapse debris is roughly 30–50% (demolition debris bulking 1.5–2×). This is where `SOLID_FRACTION` 0.65 comes from.

Not modelled: scattering, multipath inside the pile, moisture, layering and the real geometry of the voids. All of these are folded into the per-metre loss along a straight line.

## Presets and what they mean for the demo

| preset | loss | reach beneath | reach at 11 m | 10 m of rubble |
|---|---|---|---|---|
| `literature` | as above | ~3.0 m | ~0.7 m | ~130 dB, nothing |
| `legacy` | flat 0.7 dB/m one-way | ~30 m | ~8.5 m | 14 dB, still visible (**not credible**) |

The grid demo lands the drones on flat cells in a ring 9–12 m from the pile centres. Seed 7's victims sit 0.9–1.9 m deep, but the slant path from the ring to them crosses 1–3 m of pile flank. A3's is 2.9 m from the best ring cell.

- Under `literature`, every victim in the grid scenario is out of reach from the ring. The 20-seed evaluation pings 1 of 80 victims (docs/EVAL_GRID.md).
- Perching on the pile (patches up to 20° with a self-levelling radar mount, a vertical final descent, and adaptive legs settling on slabs) is implemented. On the 0.5 m heightmap it finds only 2–4 perches per pile, and their scan quality is low. That is not enough for three independent strong ranges.
- Under `legacy`, the demo pings A2, A3 and B2 as before. The mission log then states on its first line that the model is optimistic and not literature-backed.

**Default today: `legacy`** (`sim/grid_mission.py` `PENETRATION_DEFAULT`). The default stays there so the seed-7 demo and its tests keep working. `--penetration literature` shows the realistic limits.

Making `literature` the default needs one of the following:

- the radar placed on the pile: better perch detection (a finer LiDAR heightmap for slab tops) and a less punishing landing-quality model for perches;
- a scenario whose victims sit within about 1 m of rubble from the reachable landing cells;
- a scan rule that accepts fewer than three ranges with an honest uncertainty.

This is a product decision for the orchestrator.

## Reproduce the calibration

The calibration script lived in the session scratchpad. It sweeps chest amplitude from −48 to +21 dB at 6 m for qualities 0.95, 0.6 and 0.3 (12 to 16 seeds per point) and counts candidates within 0.3 m. To repeat it, write a short loop over `radar_dsp.Scene` / `simulate_if` / `process_if` with a `Breather(6.0, 10**(dB/20), ...)`.
