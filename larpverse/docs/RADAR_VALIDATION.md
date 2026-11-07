# Radar breathing extractor: validation on real data

**Claim this supports:** the DSP chain in `sim/radar_dsp.py` (arctangent phase demodulation, 0.1-0.5 Hz band-pass, spectral peak) recovers breathing rate from a real radar recording. **It does not support** any claim about sensing through rubble (see caveats).

## Dataset
Schellenberger et al. 2020, *Scientific Data* 7:291 (doi:10.1038/s41597-020-00629-5), CC BY 4.0, figshare doi:10.6084/m9.figshare.12186516. A 24 GHz six-port CW radar, about 0.4 m in front of the chest of a supine healthy subject, recorded at 2 kHz, with a Task Force Monitor reference. We used the **Resting** scenario for subjects GDN0001, GDN0002 and GDN0003 (about 10 minutes each, 3 files, 127 MB). The reference is the thoracic impedance channel `tfm_z0` (100 Hz). Details in `data/radar_real/SOURCE.md`. The data are gitignored.

## Method (`scripts/validate_real_breathing.py`)
1. Radar I/Q decimated from 2000 Hz to 20 Hz.
2. Per window: circle-fit centre removal (static clutter / DC), `unwrap(angle())`, via `radar_dsp.extract_displacement(..., "arctan")`.
3. 4th-order Butterworth band-pass 0.1-0.5 Hz, zero-padded spectrum, parabolic-interpolated peak (`radar_dsp.estimate_breathing`).
4. The reference rate comes from the **same** band-pass and peak picker applied to the impedance signal in the same window. So the numbers test the radar front end and demodulation, not the peak picker.
5. Sliding windows, hop 10 s. Error is the absolute rate difference per window.

Reproduce: `nice -n 19 .venv/bin/python scripts/validate_real_breathing.py data/radar_real/GDN000*_1_Resting.mat --plot docs/radar_real_validation.png`

## Results
30 s windows (headline), pooled over 176 windows: **MAE 0.39 bpm** (median 0.07), **91% within 1 bpm, 95% within 2 bpm**.

| Subject | Mean ref rate | MAE, 30 s | Within 2 bpm, 30 s | MAE, 20 s | Within 2 bpm, 20 s | Waveform \|r\| |
|---|---|---|---|---|---|---|
| GDN0001 | 7.7 bpm | 0.17 | 97% | 0.33 | 98% | 0.96 |
| GDN0002 | 12.3 bpm | 0.07 | 100% | 0.46 | 95% | 0.94 |
| GDN0003 | 7.5 bpm | 0.95 | 88% | 1.46 | 83% | 0.85 |
| Pooled | | 0.39 | 95% | 0.75 | 92% | |

Waveform \|r\| is the correlation between the band-passed radar displacement and the band-passed impedance over the full record (best lag within +-3 s, all under 0.2 s). The same pooled 30 s run with the simpler "pca" demodulation gives MAE 0.90 bpm and 84% within 2 bpm, so the arctangent method is the better choice on real data.

Plot: `docs/radar_real_validation.png` (top row: 60 s of radar waveform against the reference; bottom row: rate over time).

## Caveats
- **Not through-rubble.** This is 24 GHz CW radar at about 40 cm through air, from a still, supine, cooperative subject. It validates the DSP chain only. Penetration, clutter, multipath and platform motion in the rubble scenario remain simulated.
- Three subjects, resting only. We did not test Valsalva, apnea or tilt scenarios, or the other 27 subjects.
- The reference rate is a spectral peak of impedance, not a breath-by-breath count, and both signals share one peak picker. The waveform correlation gives a picker-independent check.
- Resting rates here are low (7-12 bpm), close to the 0.1 Hz (6 bpm) band edge. The worst windows (GDN0003) show isolated radar peaks at roughly 11-14 bpm while the reference stays near 7 bpm; we did not investigate the cause (not checked against movement or harmonics). The 0.1-0.5 Hz band was kept so it matches the simulator.
- No I/Q gain or phase imbalance correction was applied beyond circle-fit centring.
- 20 s windows have only about 2-3 breaths at these rates, so they degrade; use 30 s for claims.
