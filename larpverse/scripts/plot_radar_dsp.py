"""Plot the radar DSP chain on SYNTHETIC scans: good scan vs low-quality scan side by side.

Rows: range profile, range-slow-time map (after clutter removal), extracted breathing waveform,
spectrum with the detected peak.  Saves docs/radar_dsp.png.

    nice -n 19 .venv/bin/python scripts/plot_radar_dsp.py [--out docs/radar_dsp.png] [--seed 3]
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from sim import radar_dsp as R  # noqa: E402

TRUE_R, TRUE_HZ, PATH_M = 9.5, 0.27, 2.5
INK, MUTED, ACCENT, WARN = "#1f2933", "#6b7785", "#2563eb", "#d1495b"


def scan(seed: int, quality: float):
    P = R.RadarParams()
    rng = np.random.default_rng(seed)
    sc = R.Scene(quality=quality)
    sc.clutter = R.default_clutter(rng)
    sc.breathers = [R.Breather(TRUE_R, R.reflection_amplitude(TRUE_R, PATH_M), TRUE_HZ, 6.0)]
    iq = R.simulate_if(sc, P, rng)
    return R.process_if(iq, P), P


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="docs/radar_dsp.png")
    ap.add_argument("--seed", type=int, default=3)
    a = ap.parse_args(argv)
    cases = [("Good scan (landing quality 0.95)", 0.95), ("Low-quality scan (landing quality 0.15)", 0.15)]
    fig, axes = plt.subplots(4, 2, figsize=(13, 13), constrained_layout=True)
    for col, (title, q) in enumerate(cases):
        res, P = scan(a.seed, q)
        d = res.debug
        ra = d["range_axis"]
        ax = axes[0, col]
        ax.set_title(f"{title}\nestimated quality {res.quality:.2f}, platform motion {d['motion_rms_mm']:.1f} mm rms",
                     fontsize=11, color=INK, loc="left")
        ax.plot(ra, d["range_profile_db"], color=MUTED, lw=1, label="mean range profile (dB)")
        ax2 = ax.twinx()
        ax2.plot(ra, d["snr_resp_db"], color=ACCENT, lw=1.2, label="respiration-band SNR (dB)")
        ax2.axhline(10 * np.log10(R.DspConfig().cfar_alpha), color=ACCENT, ls=":", lw=1)
        ax.axvline(TRUE_R, color="#2a9d8f", lw=1, ls="--", label="true chest range")
        for rc, cf in zip(res.range_candidates, res.confidence):
            ax2.plot([rc], [10 * np.log10(max(d["p_resp"][int(round(rc / P.range_res))] /
                                              max(d["threshold_resp"][int(round(rc / P.range_res))] / R.DspConfig().cfar_alpha, 1e-9), 1e-9))],
                     "v", color=WARN, ms=8)
            ax2.annotate(f"{rc:.2f} m  c={cf:.2f}", (rc, 0), xytext=(rc + 0.4, 28), color=WARN, fontsize=9)
        ax.set_xlim(0, 22)
        ax.set_xlabel("range (m)")
        ax.set_ylabel("|range FFT| (dB)", color=MUTED)
        ax2.set_ylabel("band SNR (dB), CFAR dotted", color=ACCENT)
        ax2.set_ylim(-10, 45)
        h1, l1 = ax.get_legend_handles_labels()
        ax.legend(h1, l1, loc="upper right", fontsize=8, frameon=False)

        ax = axes[1, col]
        i0, i1 = int(1.5 / P.range_res), int(22 / P.range_res)
        m = d["range_slow_map_db"][:, i0:i1]
        im = ax.imshow(m.T, origin="lower", aspect="auto", cmap="magma",
                       extent=[0, P.dwell, i0 * P.range_res, i1 * P.range_res], vmin=np.percentile(m, 5),
                       vmax=np.percentile(m, 99.9))
        ax.axhline(TRUE_R, color="#2a9d8f", lw=0.8, ls="--")
        ax.set_xlabel("slow time (s)")
        ax.set_ylabel("range (m)")
        ax.set_title("range / slow-time map after static clutter removal (dB)", fontsize=10, color=INK, loc="left")
        fig.colorbar(im, ax=ax, pad=0.01, fraction=0.04)

        ax = axes[2, col]
        ph = d["slow_time_phase"]
        ax.plot(d["t"], (ph - ph.mean()) / max(ph.std(), 1e-12), color=MUTED, lw=0.8, label="displacement (before band-pass)")
        wf = d["breathing_waveform"]
        ax.plot(d["t"], wf / max(wf.std(), 1e-12), color=ACCENT, lw=1.8, label="band-pass 0.1-0.5 Hz")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("normalised")
        ax.set_title(f"extracted breathing waveform at bin {d['bin'] * P.range_res:.2f} m", fontsize=10, color=INK, loc="left")
        ax.legend(loc="upper right", fontsize=8, frameon=False, ncol=2)

        ax = axes[3, col]
        f, p = d["spec_f"], d["spec_p"]
        sel = f <= 1.5
        ax.plot(f[sel], p[sel] / max(p[sel].max(), 1e-30), color=INK, lw=1.2)
        ax.axvspan(*P.resp_band, color=ACCENT, alpha=0.08, label="respiration band")
        ax.axvline(TRUE_HZ, color="#2a9d8f", ls="--", lw=1, label=f"true {TRUE_HZ:.2f} Hz")
        if d["spec_peak_hz"]:
            ax.axvline(d["spec_peak_hz"], color=WARN, lw=1.5, label=f"detected {d['spec_peak_hz']:.3f} Hz")
        ax.set_xlabel("frequency (Hz)")
        ax.set_ylabel("normalised power")
        top = res.candidates[0] if res.candidates else None
        sub = "" if not top else f"  band ratio {top['band_ratio']:.2f}, peakiness {top['peakiness']:.2f}"
        ax.set_title(f"spectrum of the band-passed waveform{sub}", fontsize=10, color=INK, loc="left")
        ax.legend(loc="upper right", fontsize=8, frameon=False)
    for ax in axes.ravel():
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    fig.suptitle("Radar DSP chain on SYNTHETIC IF data (1-3 GHz sweep, 7.5 cm bins, 20 Hz x 20 s)", fontsize=13,
                 color=INK, x=0.01, ha="left")
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    fig.savefig(a.out, dpi=130)
    print("saved", a.out)


if __name__ == "__main__":
    main()
