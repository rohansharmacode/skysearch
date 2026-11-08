# Radar penetration through rubble

Short answer to "does 10 m of concrete kill the idea?": yes. Under the `literature` model, 10 m of rubble costs about 130 dB two-way, so nothing comes back. A good scan sees a breathing chest through about **3 m of rubble directly beneath it**. From an 11 m stand-off it sees through **less than 1 m**, because 1/R^4 spreading eats the margin.

Code: `sim/radar_dsp.py` (`two_way_loss_db`, `reach_path_m`, `reach_prob`, `use_penetration`). The synthetic IF generator, the model radar, the planner and the coverage map all use the same functions. Tests: `tests/test_radar_dsp.py`.

## Model

- **Band.** 0.4–2.0 GHz sweep: 1.6 GHz of bandwidth, 9.4 cm range bins, 512 bins, 48 m unambiguous range. The sub-1 GHz part of the sweep carries the deep returns.
- **Concrete loss** (one-way, solid). `alpha_c(f)` is about 10 dB/m below 1 GHz and rises linearly to 20 dB/m at 2 GHz. This is the FHWA GPR rule of thumb.
- **Solid path.** `rubble_path` is the length of the sensor→target segment that lies below the LiDAR heightmap surface. Of that length, `SOLID_FRACTION` = 0.65 counts as solid material; the rest is voids ("voids help"). An optional `REBAR_DB_PER_M` adds loss for rebar ("rebar hurts"); it defaults to 0.
- **Two-way loss.** Power is averaged over the sweep: `loss(L) = -10 log10 mean_f 10^(-2 L (0.65 alpha_c(f) + rebar) / 10)`. This gives 15 dB at 1 m, 29 dB at 2 m, 43 dB at 3 m and 69 dB at 5 m.
- **Spreading.** Chest amplitude per IF sample is `AMP_1M / R^2 · 10^(-loss/20)`, with `AMP_1M` = 70 (unit complex noise).
- **Chest motion.** The synthetic chest wall follows `sim/physio.py`: breath-by-breath motion of about 5 mm peak-to-peak for a calm adult and about 2–3 mm for rapid, shallow breathing. This is the default. The earlier 4–8 mm *amplitude* sinusoid (8–16 mm peak-to-peak) overstated the signal by about 14 dB. It remains available only through `LARPVERSE_PHYSIO=0`.
- **Detection.** No shortcut is taken here. The DSP chain (range FFT → clutter removal → respiration-band OS-CFAR → phase → band-pass) decides whether there is a detection. Calibration measured the chest amplitude at which 90% of scans detect at 6 m, using the physio waveform of a calm adult (15/min, 5 mm peak-to-peak) and 10 scans per 3 dB step. `A90_DB`:

  | dwell | q 0.95 | q 0.6 | q 0.3 |
  |---|---|---|---|
  | 20 s | −3 dB | 0 dB | +9 dB (cap) |
  | 45 s (landed radar scan) | −9 dB | −6 dB | +3 dB (cap) |
  | 60 s (vitals look) | −11.5 dB | −8.5 dB | +0.5 dB (cap) |

  The old sinusoid needed −17 dB at 20 s. Low-quality scans level off below 90% detection, so their entries are caps, not measurements. A shallow, rapid breather (depth_rel ≈ 0.45) needs about 7 dB more than these figures.
- **Reach.** `reach_path_m(range, quality, dwell)` is the rubble path at which the margin `20 log10(AMP_1M/R^2) + G_rx − A90(q, dwell)` equals `loss(L)`. Here `G_rx` is the platform's receive-aperture advantage.

  heavy_sar belly radar resting on the rubble surface, q 0.95 (face 0.48 × 0.48 m: +7.6 dB):

  | dwell | 2 m | 6 m | 11.5 m |
  |---|---|---|---|
  | 20 s | 2.47 m | 1.07 m | 0.31 m |
  | 45 s (default landed radar scan) | **2.92 m** | 1.50 m | 0.70 m |
  | 60 s | 3.11 m | 1.68 m | 0.87 m |

  Legacy small radar, 45 s: 2.35 m beneath. The previous figure of 3.6 m beneath (sinusoid, 0.6 × 0.5 m panel) was optimistic by about 0.7 m.
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
| `literature` (default) | as above | 2.9 m (heavy_sar belly radar on the rubble surface, 45 s) | 0.7 m | ~130 dB, nothing |
| `legacy` | flat 0.7 dB/m one-way | ~30 m | ~8.5 m | 14 dB, still visible (**not credible**) |

The grid demo lands the drones on flat cells in a ring 9–12 m from the pile centres. Seed 7's victims sit 0.9–1.9 m deep, but the slant path from the ring to them crosses 1–3 m of pile flank. A3's is 2.9 m from the best ring cell.

- Under `literature`, every victim in the grid scenario is out of reach from the ring. The 20-seed evaluation pings 1 of 80 victims (docs/EVAL_GRID.md).
- Perching on the pile (patches up to 20° with a self-levelling radar mount, a vertical final descent, and adaptive legs settling on slabs) is implemented. On the 0.5 m heightmap it finds only 2–4 perches per pile, and their scan quality is low. That is not enough for three independent strong ranges.
- Under `legacy`, the demo pings A2, A3 and B2 as before. The mission log then states on its first line that the model is optimistic and not literature-backed.

## Why land on the rubble

The radar is mounted on the drone's underside, facing down: a 4.9 kg module with a 0.48 m face. A heavy drone (heavy_sar, FlyCart 30 class) uses its LiDAR map to find a flat spot on the rubble and lands there; this is the default, `--delivery pod` (the radar-on-the-rubble-surface scan model). The radar then sits in contact with the rubble, so the scan starts at the surface and there is no air gap.

Landing spots:

- The drone lands on the pile within 2 m (horizontal) of the lead's posterior mode first, then further out.
- On a revisit, the lead's posterior mode is the earlier radar mode. On a first visit it is the acoustic bearing crossing, if that falls on the pile, or else the pile centre.
- A landing costs about 8 s to set down, 45 s to scan and 8 s to lift off. A firm ping gets one extra 60 s vitals look.
- For the separate listening step, the drones land off the pile, with rotors off.

Reach is about 2.9 m beneath the landed drone's belly radar and 0.7 m from the old 11 m landing ring. The attenuation was **not** changed for this mode; only the geometry, the dwell and the seating model changed. The seating model:

- roughness is taken at the radar's footprint, about 0.35 × the 1.5 m plane fit;
- slopes up to 30° do not matter;
- if the drone sits unevenly, it lifts off and re-lands once.

**Default:** `--penetration literature --radar dsp --delivery pod`, with the physio chest waveform, 45 s landed radar scans and the vitals hooks on. `legacy` and `--delivery land` are kept for comparison only.

**Seed 7.** The canonical, deterministic run is in docs/SEED7_STORY.md. It uses smooth flight and the real-pipeline acoustic fixture.

- **Pings:** A2 at 364 s, B1 at 1712 s and B2 at 2312 s. A3 stays an open acoustic lead.
- **Totals:** 34 of 36 scans, mission time 3109 s.
- **Vitals:** each ping's condition matches the hidden state. A2 is `unconscious_slow`, B1 `irregular_apneic`, B2 `calm`.
- **Why A3 fails:** its victim breathes rapidly and shallowly (33/min, about 2.2 mm). The closest landing spots are uneven, so the radar seats poorly (scan quality 0.03–0.6), and after 16 landings the returns are weak (confidence ≤ 0.3) or empty.

**Depth.** The radar gives an estimate (`depth_source: "radar_estimate"`) only when at least 3 scans support the ping and the posterior sigma is ≤ 0.45 m. Otherwise the depth stays `nominal` (assumed 1.5 m).

- In seed 7, A2 has 5 supporting landed radar scans: **1.62 ± 0.45 m** (the evaluator's hidden value is 1.64 m).
- B1 and B2 stay `nominal`: their sigma is 0.45–0.6 m with 3 supporting scans.

## Reproduce the calibration

The physio calibration loops over `radar_dsp.Scene` / `simulate_if` / `process_if`:

- breather: `Breather(6.0, 10**(dB/20), physio={"state": "calm", "rate_bpm": 15, "depth_rel": 1.0, "cv": 0.05, "apnea_per_min": 0})`;
- parameters: `RadarParams(dwell=20|45|60)`;
- sweep: −24 to +9 dB in 3 dB steps, 10 seeds per point, at qualities 0.95, 0.6 and 0.3;
- detection: a candidate within 0.3 m of 6 m.
