# Breathing-pattern triage aid from radar (BL-20)

**What it claims:** from the radar slow-time signal at a ping, `sim/vitals.py` measures breathing rate, breath-to-breath variability, pauses > 10 s and movement, and turns them into a breathing *pattern* plus a START/SALT-style *respiratory* hint that helps rescuers order the dig. **It is an aid for rescuers, not a diagnosis.** It never says a person is confirmed, injured, unconscious or a child; it says what the chest motion looks like and how sure it is.

**What it does not claim:** anything validated through real rubble. Real-data numbers below come from a 24 GHz radar 0.4 m from a still subject in air. Through-rubble numbers are synthetic.

Files: `sim/physio.py` (synthetic chest motion, truth side), `sim/vitals.py` (features + classifier, planner side), `scripts/validate_vitals.py`, `tests/test_vitals.py`, `docs/vitals_validation.png`.

![validation](vitals_validation.png)

## 1. Synthetic chest motion (`sim/physio.py`, truth side only)

`chest_displacement(t, physio, rng) -> metres` turns the hidden `physio` dict from `sim/scenario.py` into a breath-by-breath waveform. It replaces the near-sinusoid of `radar_dsp._breath_displacement` once the mission builder adds the hook (Requests).

- Per breath: raised-cosine inhale over Ti, passive (exponential-like) exhale over Te, end-expiratory pause for slow breathing. Inhale share of the cycle 0.33-0.45 by state (I:E about 1:1.2 to 1:2).
- Interval and amplitude jitter per breath from `cv` (log-normal). Occasional sighs (2x breath).
- Apnea: Poisson onsets at `apnea_per_min`, 10.5-20 s of flat chest, with shallower breaths just before and after.
- Optional small body movements (0.5-2 s, 1-6 mm) for conscious states.
- Optional cardiac motion, about 0.1 mm at 55-110 bpm. At the rubble radar's ~25 cm wavelength that is ~0.005 rad of phase: **not reliably resolvable through rubble**. It is there for realism; `vitals.py` does not report heart rate.
- `depth_rel` 1.0 = 5 mm peak-to-peak chest excursion.

| hidden state (`scenario.PHYSIO_STATES`) | expected pattern |
|---|---|
| calm (12-16/min, CV 0.03-0.08) | regular_calm |
| distressed_tachypnea (26-36/min, shallow, CV 0.12-0.25) | rapid_shallow |
| unconscious_slow (6-10/min, deep, some pauses) | slow_deep |
| irregular_apneic (8-14/min, CV 0.25-0.45, 0.5-1.5 pauses/min) | irregular_with_pauses |
| child_calm (20-28/min, small, regular) | fast_small |

## 2. Features (`sim/vitals.py`)

Input: the complex slow-time samples of the candidate's range bin, motion-compensated as in `process_if` (`Xc[:, k]`). Also accepts phase, displacement in metres, or any displacement-proportional series.

1. **Demodulation.** Circle-fit + arctangent when the I/Q points trace a real arc (arc >= 0.6 rad, sqrt(minor/major variance) >= 0.25, ring residual < 0.6, median phase step < 0.15 rad). Otherwise principal-axis linear demodulation, scaled to metres by the amplitude the **propagation model** predicts for that range and rubble path (`radar_dsp.reflection_amplitude` x range-FFT gain). On the real 24 GHz data the arc is several radians, so the arctangent is used. On the synthetic rubble radar (wavelength ~25 cm) a breath is a short arc (~0.1-0.4 rad), so the path-loss scaling is used.
2. **Movement / artefacts.** 1 s frames whose 0.9-3.5 Hz energy exceeds 5x the scan median (and 0.15x the breathing rms). Reported as a fraction; breath intervals that touch a movement frame are dropped.
3. **Rate.** Spectral peak in 0.1-0.75 Hz (6-45/min) and time-domain breath peaks (prominence 0.3 x the 5-95% span, at least 1 s apart, both polarities tried because chest direction is unknown through rubble). The breath count wins when there are >= 3 clean intervals. CI: delta method on the interval median, at least +-0.5/min; spectrum-only gives +-30/T bpm.
4. **Variability.** Robust CV of breath intervals, 1.4826 x MAD / median (one sigh or one missed peak does not make a regular breather "irregular"). The classic std/mean is kept as `cv_classic` internally.
5. **I:E proxy.** Shorter half-cycle / longer half-cycle. Which half is the inhale is assumed (the shorter), since polarity is unknown.
6. **Relative depth.** Median breath peak-to-trough / 5 mm. **LOW confidence by construction**: `depth_rel_conf` <= 0.3 x SQI (arctangent) or 0.15 x SQI (path-loss scaled). Chest-wall amplitude depends on aspect, posture, range and the rubble in the path.
7. **Apnea.** 3 s moving-rms envelope of the 0.1-0.8 Hz band (drops post-hold drift and heartbeat); below 35% of the 80th-percentile level for >= 10 s, outside movement frames.
8. **Signal quality index (SQI).** Logistic in band SNR (breathing band vs a 2-5 Hz noise floor, midpoint 8 dB) x the DSP's candidate SNR term x (1 - movement)^1.5 x a breath-count term x (0.4 + 0.6 x scan quality).

## 3. Classifier and triage hint (rules, in this order)

No learned model: every threshold is a named constant at the top of `sim/vitals.py`.

| step | rule | pattern |
|---|---|---|
| 0 | SQI < 0.35, no rate, or < 2 breaths and no pause | unclear |
| 1 | any pause >= 10 s | irregular_with_pauses |
| 2 | rate > 30 | rapid_shallow |
| 3 | rate < 19.5 and robust CV >= 0.20 + 0.25 x (1 - quality) | irregular_with_pauses |
| 4 | rate < 11 | slow_deep |
| 5 | rate >= 27 (unless CV < 0.08), or 19.5-27 with CV >= 0.30 | rapid_shallow |
| 6 | 19.5-27 and regular | fast_small (confidence capped at 0.45; halved if a measured depth_rel > 1.0) |
| 7 | 11-19.5 and regular | regular_calm |

The CV threshold rises when scan quality is low because platform motion jitters breath timing. Above 19.5/min, rate decides before CV for the same reason. Through the radar, a child's measured CV reads 0.1-0.4 at low quality, so CV cannot separate a fast regular breather from a distressed one there.

`pattern_conf` = SQI x margin from the deciding threshold (in CI units) x evidence (number of intervals).

| pattern | `triage_hint` | why (rescuer-facing) |
|---|---|---|
| rapid_shallow, rate > 30 | immediate | START respiratory criterion for IMMEDIATE |
| rapid_shallow, 27-30 or irregular | immediate (check) | possible distress or shock |
| irregular_with_pauses | immediate (check) | possible airway problem or reduced consciousness |
| slow_deep | immediate (check) | slow breathing may mean reduced consciousness |
| fast_small | delayed (re-check) | normal for a child, elevated for an adult |
| regular_calm | delayed | respiration only; perfusion and responsiveness unknown |
| unclear | assess | needs a longer or better scan |

We never output "expectant": a radar cannot tell absent breathing from a chest out of view. `priority_weight(v)` maps the hint to a rank (immediate 1.0, immediate (check) 0.85, assess 0.55, delayed (re-check) 0.45, delayed 0.35) shrunk toward "assess" as `pattern_conf` falls. `dig_score(v, p_mass, depth_m)` = weight x localisation certainty x accessibility (clip(1 - depth/4 m, 0.2, 1)).

## 4. Validation

Reproduce: `scripts/heavy.sh .venv/bin/python scripts/validate_vitals.py --n 250 --plot docs/vitals_validation.png` (~4 min; add `--cache <file>.pkl` to re-tune thresholds without re-simulating). Tests: `.venv/bin/python -m pytest -q tests/test_vitals.py` (11 pass, ~3 s).

### 4a. Synthetic radar scans (truth only in the evaluator)

250 scans per dwell, states in rotation. Range 1.5-4 m, rubble path 0-3 m, landing quality 0.45-0.97, chest RCS jittered +-3 dB (sd), and the planner's rubble-path estimate off by up to +-30%. Chest rendered by `physio.chest_displacement` inside `radar_dsp.simulate_if` (stand-in for the requested hook), then the real `process_if` chain with the respiration band widened to 0.1-0.75 Hz. We take the DSP candidate within 0.5 m of the true range and run `vitals.extract` on that bin. Undetected scans (no candidate) are counted separately. The propagation model was `radar_dsp`'s `legacy` mode (0.7 dB/m) at run time.

| dwell | detected | pattern correct | triage hint agrees | rate MAE (SNR >= 15 dB, q >= 0.6) | within 1.5/min |
|---|---|---|---|---|---|
| 20 s | 168/250 | **67%** | 81% | 1.90/min | 69% |
| 60 s | 179/250 | **81%** | 88% | 0.69/min | 90% |

At 60 s, accuracy by measured scan quality: 78% (< 0.6), 71% (0.6-0.8), **94% (>= 0.8)**. It is flat across rubble path (80-82%) and DSP SNR (74-92% over 20-40+ dB): once the DSP has a candidate, landing quality matters more than SNR.

Confusion matrix, 60 s dwell (rows hidden state, columns pattern):

| | regular_calm | rapid_shallow | slow_deep | irregular_with_pauses | fast_small | unclear |
|---|---|---|---|---|---|---|
| calm | **45** | | | 1 | | |
| distressed_tachypnea | | **20** | 1 | | 3 | |
| unconscious_slow | 1 | | **34** | 7 | | |
| irregular_apneic | 8 | | 5 | **22** | | |
| child_calm | 1 | 7 | | | **24** | |

Recall: calm 98%, tachypnea 83%, unconscious 81%, irregular-apneic 63%, child 75%. Where it goes wrong:
- unconscious_slow -> irregular_with_pauses (7): these states do have occasional pauses, and the hint is the same ("immediate (check)").
- irregular_apneic -> regular_calm (8): windows that held no pause and modest variability. This is the dangerous direction (under-triage).
- child -> rapid_shallow (7) and tachypnea -> fast_small (3): the 26-28/min overlap. The child errors over-triage, which is the safe direction.
- Apnea detection through the simulated radar is weak: only 29% of irregular-apneic 60 s scans show a detected pause, although about 60% contain one. Residual platform and rubble motion fills the gap above the 35% envelope threshold. No false pauses were detected in regular states (0%).
- At **20 s dwell, irregular_apneic is almost never recognised (1/32)**. A 20 s window holds 2-4 slow breaths and at most one partial pause, so it mostly reads as slow_deep or regular_calm. **Pattern claims need a 45-60 s "vitals look".**
- Depth (path-loss scaled): measured/true depth_rel median 1.5x (p10-p90 0.9-2.6x), log correlation 0.70. That is enough to rank "small vs large" weakly and no more. It would be worse in reality because the simulator uses the same propagation model.

### 4b. Real radar: Schellenberger et al. 2020 (24 GHz CW, 0.4 m, supine, CC BY 4.0)

Data in `data/radar_real/` (gitignored; see `SOURCE.md`). Same `vitals.extract` on radar I/Q (decimated to 20 Hz) and on the thoracic-impedance reference.

**Resting** (GDN0001-0003, ~10 min each), rate vs impedance reference (breath count on both):

| window | windows | MAE | median abs error | within 1.5/min |
|---|---|---|---|---|
| 60 s (hop 30 s) | 57 | **0.44/min** | 0.10 | 91% |
| 30 s (hop 15 s) | 118 | **0.25/min** | 0.10 | 95% |

Per subject (60 s): GDN0001 0.24 (ref 7.6/min), GDN0002 0.68 (12.7/min), GDN0003 0.42 (4.9/min). Radar and reference got the same pattern in 75% (60 s) and 93% (30 s) of windows. **No false pauses in any of the 175 resting windows.**

Honest finding: these healthy, relaxed, supine volunteers breathe 5-8/min, so most windows are labelled **slow_deep** (60 s: 30/57) or **irregular_with_pauses** (19/57, from long, variable intervals), giving the hint "immediate (check)". Rate rules flag a calm person who breathes slowly. That over-triages, which is the safe direction for search and rescue, but it shows the pattern is not a diagnosis.

**Apnea** (GDN0004-0006, ~150 s each, two voluntary breath holds per file marked by `tfm_intervention`):

- **6/6 breath holds detected (13-55 s long), 0 false pause events.** Detected pauses cover 80-88% of the marked hold time; median onset lag 2.1 s. The impedance reference, run through the same detector, gives near-identical intervals.
- In 60 s windows (hop 10 s), 22/23 windows that contain >= 12 s of a hold are labelled irregular_with_pauses.
- These are voluntary breath holds by healthy volunteers, not pathological apnea or Cheyne-Stokes breathing.

## 5. Honest limits

- **Not through rubble.** Real data is 24 GHz at 0.4 m in air. The rubble radar is 0.4-2 GHz with tens of dB of loss, clutter in the same bin, and a landed drone that moves. All rubble numbers are synthetic and use our own propagation model.
- **Dwell.** 20 s supports rate only. Variability and apnea need >= 45-60 s, and a pattern seen once needs a re-scan before anyone acts on it.
- **Amplitude is unreliable.** depth_rel depends on aspect, posture, range, rubble loss and our model, so it is always low confidence. **Age inference is weak**: "fast_small" only means fast, regular and possibly small. A distressed adult can look the same; confidence is capped at 0.45.
- **Heart rate is not reported**, because ~0.1 mm cardiac motion is not resolvable through rubble.
- **Platform motion** jitters breath timing (CV) and fills apnea gaps; poor landings degrade the pattern before they degrade the rate.
- **Respiratory criteria only.** START/SALT also need perfusion, mental status and the ability to follow commands; a radar sees none of these. **Not medical advice.**
- Thresholds were tuned on the synthetic set (60 s) and on the real files (apnea band and fraction), then reported on the same data. No held-out set.

## 6. Contract (`vitals` object; text for contracts/INTERFACES.md, see Requests)

Attached per radar candidate (aligned with `range_candidates`) and per ping. All fields are JSON-safe; `null` when not measurable.

```jsonc
"vitals": {
  "version": "vitals-v1",
  "rate_bpm": 31.6, "rate_ci": [29.8, 33.6],      // breaths/min; breath count if >= 3 clean intervals, else spectrum
  "rate_spectral_bpm": 31.2, "rate_count_bpm": 31.6,
  "cv": 0.21,                                    // robust breath-interval CV (1.4826 MAD / median); null if < 3 intervals
  "ie_ratio": 0.72,                              // shorter / longer half-cycle (polarity unknown)
  "depth_rel": 0.6, "depth_rel_conf": 0.12,      // 1.0 = 5 mm p-p; conf <= 0.3 ALWAYS (low confidence)
  "apnea_events": 0, "apnea_events_per_min": 0.0, "apnea_longest_s": 0.0,   // pauses >= 10 s
  "movement": 0.03,                              // fraction of the window flagged as movement / artefact
  "snr_db": 18.4, "sqi": 0.81,                   // band SNR of the slow-time signal; signal quality index 0-1
  "n_breaths": 31, "duration_s": 60.0, "demod": "linear+pathloss",   // arctan | linear+pathloss | linear | displacement | phase
  "pattern": "rapid_shallow",                    // regular_calm | rapid_shallow | slow_deep | irregular_with_pauses | fast_small | unclear
  "pattern_conf": 0.72, "pattern_reason": "rate 32/min > 30",
  "triage_hint": "immediate",                    // immediate | immediate (check) | delayed (re-check) | delayed | assess
  "triage_reason": "breathing over 30/min (START respiratory criterion for IMMEDIATE)",
  "priority_weight": 0.88,                       // hint rank shrunk to 'assess' (0.55) by pattern_conf
  "caveats": ["aid for rescuers, not a diagnosis: ...", "depth_rel: LOW confidence ...", "..."]
}
```

Dig order (suggested, `vitals.dig_score`): `priority_weight x p_mass x clip(1 - depth_m / 4, 0.2, 1)`. A ping without vitals uses the "assess" weight. UI and LLM wording must keep "aid / possible / check", never "diagnosis", "confirmed", "injured", "unconscious" or "child" as fact.
