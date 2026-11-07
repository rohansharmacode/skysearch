"""Run the SAME breathing extractor used by sim/radar_dsp.py on REAL radar recordings.

Dataset (downloaded to data/radar_real/, see data/radar_real/SOURCE.md): Schellenberger et al., "A
dataset of clinically recorded radar vital signs with synchronised reference sensor signals",
Scientific Data 7:291 (2020), CC BY 4.0, figshare https://doi.org/10.6084/m9.figshare.12186516.
24 GHz six-port CW radar at ~0.4 m from the chest, 30 healthy subjects, with a Task Force Monitor
(TFM) reference including thoracic impedance (tfm_z0), the respiration reference used here.

Verified .mat contents (MATLAB v5, loads with scipy.io; v7.3 falls back to h5py), Resting scenario:
  radar_i, radar_q   radar baseband I/Q (2000 Hz = fs_radar), raw ADC counts
  tfm_z0             thoracic impedance, 100 Hz (= fs_z0): respiration reference
  fs_radar, fs_z0, fs_ecg, fs_icg, fs_bp   scalars;  measurement_info = [timestamp, scenario, subject]
  (also tfm_ecg1/2, tfm_icg, tfm_bp, tfm_intervention; tfm_param is empty in the Resting files)

Run from the repo root (never from inside the data dir):
    nice -n 19 .venv/bin/python scripts/validate_real_breathing.py data/radar_real/GDN000*_1_Resting.mat \
        --plot docs/radar_real_validation.png --json /tmp/metrics.json

Pipeline (identical to the simulator's extractor): decimate I/Q to 20 Hz -> per-window circle-fit
DC removal + arctangent demodulation + unwrap (radar_dsp.extract_displacement, method "arctan") ->
Butterworth band-pass 0.1-0.5 Hz -> zero-padded spectrum -> peak (radar_dsp.estimate_breathing).
The reference rate comes from the same band-pass + spectral peak applied to the impedance signal
in the same window, so the comparison isolates the radar front end/demodulation, not the peak picker.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
from scipy import signal

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from sim import radar_dsp as R  # noqa: E402

FS_OUT = 20.0


def load_mat(path: str) -> dict:
    """Load a .mat into a flat dict of numpy arrays (scipy for <=v7.2, h5py for v7.3)."""
    try:
        from scipy.io import loadmat
        raw = loadmat(path, squeeze_me=True, struct_as_record=False)
        return {k: v for k, v in raw.items() if not k.startswith("__")}
    except NotImplementedError:
        try:
            import h5py
        except ImportError as e:  # pragma: no cover
            raise SystemExit("MATLAB v7.3 file: install h5py (uv pip install --python .venv h5py)") from e
        out = {}
        with h5py.File(path, "r") as f:
            for k, v in f.items():
                if isinstance(v, h5py.Dataset):
                    out[k] = np.array(v).squeeze()
        return out


def decimate_to(x: np.ndarray, fs_in: float, fs_out: float = FS_OUT) -> np.ndarray:
    """Anti-aliased decimation in stages of at most 10 (works for real or complex input)."""
    q = int(round(fs_in / fs_out))
    if q < 1 or abs(fs_in / q - fs_out) > 1e-6 * fs_in:
        # non-integer ratio: polyphase resample
        return signal.resample_poly(x, int(round(fs_out * 1000)), int(round(fs_in * 1000)))
    while q > 1:
        step = next(d for d in (10, 8, 5, 4, 3, 2, q) if q % d == 0)
        x = signal.decimate(x, step, ftype="iir", zero_phase=True) if np.isrealobj(x) else \
            signal.decimate(x.real, step, zero_phase=True) + 1j * signal.decimate(x.imag, step, zero_phase=True)
        q //= step
    return x


def window_rates(x20: np.ndarray, fs: float, win_s: float, hop_s: float, method: str, band=(0.1, 0.5)):
    n, h = int(win_s * fs), int(hop_s * fs)
    rows = []
    for s in range(0, len(x20) - n + 1, h):
        seg = x20[s:s + n]
        disp = R.extract_displacement(seg, method) if np.iscomplexobj(seg) else seg - seg.mean()
        est = R.estimate_breathing(disp, fs, band)
        rows.append((s / fs, est["rate_hz"], est["peakiness"], est["band_ratio"]))
    return rows


def pick(d: dict, name: str):
    """Case-insensitive field lookup."""
    for k in d:
        if k.lower() == name.lower():
            return d[k]
    raise KeyError(name)


def analyse(path: str, win: float, hop: float, method: str, i_field: str, q_field: str, ref_field: str):
    d = load_mat(path)
    fs = float(np.asarray(pick(d, "fs_radar")).ravel()[0])
    iq = np.asarray(pick(d, i_field), float).ravel() + 1j * np.asarray(pick(d, q_field), float).ravel()
    z = np.asarray(pick(d, ref_field), float).ravel()
    fs_z = float(np.asarray(pick(d, "fs_z0")).ravel()[0]) if any(k.lower() == "fs_z0" for k in d) \
        else len(z) / (len(iq) / fs)
    x20 = decimate_to(iq, fs)
    z20 = decimate_to(z, fs_z)
    n = min(len(x20), len(z20))
    x20, z20 = x20[:n], z20[:n]
    rad = window_rates(x20, FS_OUT, win, hop, method)
    ref = window_rates(z20, FS_OUT, win, hop, "pca")
    t = np.array([r[0] for r in rad]) + win / 2
    br = np.array([np.nan if r[1] is None else 60 * r[1] for r in rad])
    bz = np.array([np.nan if r[1] is None else 60 * r[1] for r in ref])
    # whole-record displacement waveforms (display + waveform correlation): arctan demod on the full record
    disp = R.bandpass(R.extract_displacement(x20, method), FS_OUT)
    zbp = R.bandpass(z20 - z20.mean(), FS_OUT)
    # cross-correlation over +-3 s lag, sign-agnostic
    lags = np.arange(-int(3 * FS_OUT), int(3 * FS_OUT) + 1)
    def corr_at(l):
        a_, b_ = (disp[l:], zbp[:n - l]) if l >= 0 else (disp[:n + l], zbp[-l:])
        m = min(len(a_), len(b_))
        return np.corrcoef(a_[:m], b_[:m])[0, 1]
    cs = np.array([corr_at(l) for l in lags])
    k = np.argmax(np.abs(cs))
    return dict(path=path, subject=str(np.asarray(pick(d, "measurement_info")).ravel()[2]), fs=fs, dur=len(iq) / fs,
                t=t, radar_bpm=br, ref_bpm=bz, disp=disp, zbp=zbp, corr=float(cs[k]), lag_s=float(lags[k] / FS_OUT),
                pk=np.array([r[2] for r in rad]), band_ratio=np.array([r[3] for r in rad]))


def metrics(r: dict) -> dict:
    ok = np.isfinite(r["radar_bpm"]) & np.isfinite(r["ref_bpm"])
    e = np.abs(r["radar_bpm"][ok] - r["ref_bpm"][ok])
    return dict(subject=r["subject"], windows=int(ok.sum()), mae_bpm=float(e.mean()), median_ae_bpm=float(np.median(e)),
                within1=float(100 * np.mean(e <= 1)), within2=float(100 * np.mean(e <= 2)),
                ref_bpm_mean=float(np.mean(r["ref_bpm"][ok])), radar_bpm_mean=float(np.mean(r["radar_bpm"][ok])),
                wave_corr=r["corr"], lag_s=r["lag_s"], errs=e)


def plot(results, out, win):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    n = len(results)
    fig, ax = plt.subplots(2, n, figsize=(5.2 * n, 6.4), squeeze=False)
    for j, r in enumerate(results):
        m = metrics(r)
        t0 = 120.0  # show 60 s excerpt starting at 2 min
        sl = slice(int(t0 * FS_OUT), int((t0 + 60) * FS_OUT))
        tt = np.arange(len(r["disp"]))[sl] / FS_OUT
        sgn = np.sign(r["corr"]) or 1.0
        z = lambda v: (v - v.mean()) / (v.std() + 1e-12)
        a = ax[0, j]
        a.plot(tt, z(r["zbp"][sl]), color="#1b6ca8", lw=1.6, label="reference (thoracic impedance)")
        a.plot(tt, sgn * z(r["disp"][sl]), color="#d9531e", lw=1.2, label="radar (arctan phase, 0.1-0.5 Hz)")
        a.set_title(f"{r['subject']} Resting: waveforms, r = {abs(r['corr']):.2f}")
        a.set_xlabel("time (s)"); a.set_ylabel("z-scored amplitude"); a.legend(loc="upper right", fontsize=7)
        b = ax[1, j]
        b.plot(r["t"], r["ref_bpm"], "o-", color="#1b6ca8", ms=3, label="reference rate")
        b.plot(r["t"], r["radar_bpm"], "s--", color="#d9531e", ms=3, label="radar rate")
        b.set_ylim(5, 35)
        b.set_title(f"{win:.0f} s windows: MAE {m['mae_bpm']:.2f} bpm, {m['within2']:.0f}% within 2 bpm")
        b.set_xlabel("window centre (s)"); b.set_ylabel("breaths / min"); b.legend(loc="upper right", fontsize=7)
        for x in ax[:, j]:
            x.grid(alpha=.3)
    fig.suptitle("Schellenberger 2020 (24 GHz CW, CC BY 4.0): sim/radar_dsp.py extractor vs impedance reference", fontsize=10)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mat", nargs="+", help="path(s) to Schellenberger .mat recording(s)")
    ap.add_argument("--list", action="store_true", help="list variables in the first file and exit")
    ap.add_argument("--i-field", default="radar_i")
    ap.add_argument("--q-field", default="radar_q")
    ap.add_argument("--ref-field", default="tfm_z0", help="respiration reference (thoracic impedance)")
    ap.add_argument("--window", type=float, default=30.0, help="analysis window, s")
    ap.add_argument("--hop", type=float, default=10.0)
    ap.add_argument("--method", choices=["arctan", "pca"], default="arctan")
    ap.add_argument("--table", action="store_true", help="print per-window table")
    ap.add_argument("--plot", default=None, help="write PNG here")
    ap.add_argument("--json", default=None, help="write metrics JSON here")
    a = ap.parse_args(argv)

    if a.list:
        for k, v in load_mat(a.mat[0]).items():
            print(f"{k:20s} {getattr(v, 'shape', '')} {getattr(v, 'dtype', type(v).__name__)}")
        return
    results = [analyse(p, a.window, a.hop, a.method, a.i_field, a.q_field, a.ref_field) for p in a.mat]
    allerr, out = [], []
    for r in results:
        m = metrics(r)
        allerr.append(m.pop("errs"))
        out.append(m)
        print(f"{m['subject']}: {r['dur']:.0f} s, {m['windows']} windows ({a.window:.0f} s / hop {a.hop:.0f} s, {a.method}); "
              f"ref {m['ref_bpm_mean']:.1f} bpm, radar {m['radar_bpm_mean']:.1f} bpm; MAE {m['mae_bpm']:.2f}, median {m['median_ae_bpm']:.2f}, "
              f"within 1 bpm {m['within1']:.0f}%, within 2 bpm {m['within2']:.0f}%; waveform |r| {abs(m['wave_corr']):.2f} (lag {m['lag_s']:+.2f} s)")
        if a.table:
            for t, rb, zb, pk in zip(r["t"], r["radar_bpm"], r["ref_bpm"], r["pk"]):
                print(f"  t={t:6.0f}  radar {rb:5.1f}  ref {zb:5.1f}  err {rb - zb:+5.1f}  peakiness {pk:.2f}")
    e = np.concatenate(allerr)
    pooled = dict(windows=int(e.size), mae_bpm=float(e.mean()), median_ae_bpm=float(np.median(e)),
                  within1=float(100 * np.mean(e <= 1)), within2=float(100 * np.mean(e <= 2)))
    print(f"POOLED ({len(results)} files): {pooled['windows']} windows, MAE {pooled['mae_bpm']:.2f} bpm, "
          f"median {pooled['median_ae_bpm']:.2f}, within 1 bpm {pooled['within1']:.0f}%, within 2 bpm {pooled['within2']:.0f}%")
    if a.plot:
        plot(results, a.plot, a.window)
        print("wrote", a.plot)
    if a.json:
        with open(a.json, "w") as f:
            json.dump(dict(window_s=a.window, hop_s=a.hop, method=a.method, per_file=out, pooled=pooled), f, indent=1)


if __name__ == "__main__":
    main()
