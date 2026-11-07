"""Validate sim/vitals.py: synthetic radar scans (truth only here, in the evaluator) and REAL radar data.

(a) SYNTHETIC: for each scan, draw a breathing state from sim/scenario.PHYSIO_STATES, render the chest
    with sim/physio.chest_displacement inside the SYNTHETIC IF generator (sim/radar_dsp.simulate_if,
    via a stand-in for the requested hook, see _install_physio_hook), run the real DSP chain
    (radar_dsp.process_if), take the DSP candidate nearest the true range (evaluator choice) and run
    vitals.extract on that bin's motion-compensated slow-time I/Q. Confusion matrix and accuracy vs
    SNR / rubble path / quality at 20 s and 60 s dwell. The chest RCS is jittered +-3 dB (sd) and the
    planner's rubble-path estimate is off by +-30% so the path-loss depth estimate is not circular.
(b) REAL: Schellenberger et al. 2020 (Scientific Data 7:291, CC BY 4.0), 24 GHz CW radar ~0.4 m from
    a supine subject's chest (see data/radar_real/SOURCE.md). Resting: rate per window vs the thoracic
    impedance reference (same vitals.extract on both, breath-count rate). Apnea: detected pauses vs
    the recorded intervention marker (tfm_intervention low = breath hold).

Run from the repo root (heavy, ~2-4 min):
    scripts/heavy.sh .venv/bin/python scripts/validate_vitals.py --n 250 --plot docs/vitals_validation.png \
        --json /tmp/vitals_metrics.json
    .venv/bin/python scripts/validate_vitals.py --fetch-apnea      # 3 Apnea files, ~31 MB, range requests
"""
from __future__ import annotations

import argparse
import json
import os
import struct
import subprocess
import sys
import zlib
from collections import Counter

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "scripts"))
from sim import physio, radar_dsp as R, vitals  # noqa: E402
from sim.scenario import PHYSIO_STATES  # noqa: E402

DATA = os.path.join(ROOT, "data", "radar_real")
STATES = list(PHYSIO_STATES)
LABELS = [physio.PATTERN_FOR_STATE[s] for s in STATES] + ["unclear"]
# widened respiration band so 26-36 bpm tachypnea (0.43-0.6 Hz) is inside it (Request to the DSP owner)
VITALS_RESP_BAND = (0.1, 0.75)
VITALS_OOB_BAND = (0.85, 3.0)
BLUE, ORANGE, AQUA, INK, INK2, GRID = "#2a78d6", "#eb6834", "#1baf7a", "#0b0b0b", "#52514e", "#e4e3df"
TRIAGE_OF = {p: vitals.triage(p, 35 if p == "rapid_shallow" else None)[0] for p in LABELS}


# ============================================================================ (a) synthetic
def _install_physio_hook():
    """Stand-in for the requested change in radar_dsp._breath_displacement: if the Breather carries a
    ``physio`` dict, render sim/physio.chest_displacement instead of the near-sinusoid."""
    orig = R._breath_displacement

    def hooked(b, params, rng):
        ph = getattr(b, "physio", None)
        return physio.chest_displacement(params.t_slow, ph, rng) if ph else orig(b, params, rng)
    R._breath_displacement = hooked


def front_end(iq, params, cfg):
    """The motion-compensated cube Xc exactly as radar_dsp.process_if builds it (for the candidate bin I/Q)."""
    X = R.range_fft(iq)
    m = X.shape[0]
    d_ref, _ = R._estimate_platform_motion(X, params, cfg)
    Xc = X * np.exp(-1j * 4 * np.pi * d_ref / params.wavelength)[:, None]
    tl = np.linspace(-1, 1, m)
    A = np.column_stack([d_ref, tl])
    coef, *_ = np.linalg.lstsq(A, Xc - Xc.mean(axis=0, keepdims=True), rcond=None)
    return Xc - A @ coef


def one_scan(i, dwell, seed=11):
    rng = np.random.default_rng([seed, i, int(dwell)])
    state = STATES[i % len(STATES)]
    ph = physio.sample_physio(state, rng)
    rng_m = float(rng.uniform(1.5, 4.0))
    path = float(rng.uniform(0.0, 3.0))
    quality = float(rng.uniform(0.45, 0.97))
    rcs = R.CHEST_RCS_DB + float(rng.normal(0, 3.0))
    params = R.RadarParams(dwell=float(dwell), resp_band=VITALS_RESP_BAND, oob_band=VITALS_OOB_BAND)
    cfg = R.DspConfig()
    scene = R.Scene(quality=quality)
    scene.clutter = R.default_clutter(rng, max_range=min(22.0, params.max_range - 2.0))
    b = R.Breather(rng_m, R.reflection_amplitude(rng_m, path, rcs, params), ph["rate_bpm"] / 60.0)
    b.physio = ph
    scene.breathers.append(b)
    # truth chest waveform for the plot / realised rate (same rng stream as the generator is NOT needed)
    iq = R.simulate_if(scene, params, rng)
    res = R.process_if(iq, params, cfg, keep_debug=False)
    row = dict(i=i, dwell=dwell, state=state, truth=physio.PATTERN_FOR_STATE[state], rate_true=ph["rate_bpm"],
               cv_true=ph["cv"], depth_true=ph["depth_rel"], apnea_rate_true=ph["apnea_per_min"],
               range_m=rng_m, path=path, quality=quality, detected=False)
    near = [c for c in res.candidates if abs(c["range_m"] - rng_m) < 0.5]
    if not near:
        return row
    c = max(near, key=lambda c: c["confidence"])
    Xc = front_end(iq, params, cfg)
    path_est = path * float(rng.uniform(0.7, 1.3))
    row.update(detected=True, snr_db=c["snr_db"], q_meas=res.quality, x=Xc[:, c["bin"]],
               args=dict(fs=params.fs_slow, wavelength=params.wavelength, range_m=c["range_m"], rubble_path_m=path_est,
                         quality=res.quality, snr_db=c["snr_db"]))
    return analyse_scan(row)


def analyse_scan(row):
    """vitals.extract on the cached candidate-bin I/Q (cheap; lets thresholds be re-tuned without re-simulating)."""
    if row["detected"]:
        a = dict(row["args"])
        fs = a.pop("fs")
        row["v"] = vitals.extract(row["x"], fs, kind="iq", keep_arrays=(row["i"] < 60), **a)
    return row


def run_synthetic(n, dwells, cache=None):
    import pickle
    if cache and os.path.exists(cache):
        with open(cache, "rb") as f:
            rows = pickle.load(f)
        if len(rows) == n * len(dwells):
            return [analyse_scan(r) for r in rows]
    _install_physio_hook()
    rows = [one_scan(i, dw) for dw in dwells for i in range(n)]
    if cache:
        with open(cache, "wb") as f:
            pickle.dump([{k: v for k, v in r.items() if k != "v"} for r in rows], f)
    return rows


def confusion(rows):
    M = np.zeros((len(STATES), len(LABELS)), int)
    for r in rows:
        if r["detected"]:
            M[STATES.index(r["state"]), LABELS.index(r["v"]["pattern"])] += 1
    return M


def summarise_synth(rows, dwell):
    rs = [r for r in rows if r["dwell"] == dwell]
    det = [r for r in rs if r["detected"]]
    M = confusion(det)
    acc = float(np.trace(M[:, :len(STATES)]) / max(M.sum(), 1))
    called = [r for r in det if r["v"]["pattern"] != "unclear"]
    acc_called = float(np.mean([r["v"]["pattern"] == r["truth"] for r in called])) if called else 0.0
    tri_ok = float(np.mean([TRIAGE_OF[r["v"]["pattern"]] == TRIAGE_OF[r["truth"]] for r in called])) if called else 0.0
    good = [r for r in det if r["snr_db"] >= 15 and r["q_meas"] >= 0.6]
    rate_err = np.array([abs(r["v"]["rate_bpm"] - r["rate_true"]) for r in good
                         if r["state"] != "irregular_apneic" and r["v"]["rate_bpm"] is not None])
    by_snr = []
    for lo, hi in ((0, 20), (20, 25), (25, 30), (30, 40), (40, 99)):
        b = [r for r in det if lo <= r["snr_db"] < hi]
        if b:
            by_snr.append(dict(snr=f"{lo}-{hi}", n=len(b), acc=float(np.mean([r["v"]["pattern"] == r["truth"] for r in b])),
                               unclear=float(np.mean([r["v"]["pattern"] == "unclear" for r in b]))))
    by_path = []
    for lo, hi in ((0, 1), (1, 2), (2, 3.01)):
        b = [r for r in rs if lo <= r["path"] < hi]
        bd = [r for r in b if r["detected"]]
        by_path.append(dict(path=f"{lo}-{hi:g} m", n=len(b), detected=len(bd) / max(len(b), 1),
                            acc=float(np.mean([r["v"]["pattern"] == r["truth"] for r in bd])) if bd else None))
    by_q = []
    for lo, hi in ((0.0, 0.6), (0.6, 0.8), (0.8, 1.0)):
        b = [r for r in det if lo <= r["q_meas"] < hi]
        if b:
            by_q.append(dict(q=f"{lo}-{hi:g}", n=len(b), acc=float(np.mean([r["v"]["pattern"] == r["truth"] for r in b]))))
    ap = [r for r in det if r["state"] == "irregular_apneic"]
    ap_hit = float(np.mean([r["v"]["apnea_events"] > 0 for r in ap])) if ap else None
    noap = [r for r in det if r["state"] in ("calm", "child_calm", "distressed_tachypnea")]
    ap_fp = float(np.mean([r["v"]["apnea_events"] > 0 for r in noap])) if noap else None
    dr = np.array([(r["v"]["depth_rel"], r["depth_true"]) for r in good if r["v"]["depth_rel"] is not None])
    depth_corr = float(np.corrcoef(np.log(np.maximum(dr[:, 0], 1e-3)), np.log(dr[:, 1]))[0, 1]) if len(dr) > 5 else None
    depth_ratio = (np.percentile(dr[:, 0] / dr[:, 1], [10, 50, 90]).round(2).tolist()) if len(dr) > 5 else None
    per_state = {s: dict(n=int(M[k].sum()), recall=float(M[k, LABELS.index(physio.PATTERN_FOR_STATE[s])] / max(M[k].sum(), 1)))
                 for k, s in enumerate(STATES)}
    return dict(dwell=dwell, scans=len(rs), detected=len(det), accuracy=acc, accuracy_called=acc_called,
                triage_agreement_called=tri_ok, unclear_frac=float(np.mean([r["v"]["pattern"] == "unclear" for r in det])),
                rate_mae_good=float(rate_err.mean()) if rate_err.size else None,
                rate_within_1p5_good=float(np.mean(rate_err <= 1.5)) if rate_err.size else None, n_good=len(rate_err),
                apnea_state_hit=ap_hit, apnea_false_alarm=ap_fp, depth_log_corr=depth_corr, depth_ratio_p10_50_90=depth_ratio,
                per_state=per_state, by_snr=by_snr, by_path=by_path, by_quality=by_q, confusion=M.tolist())


# ============================================================================ (b) real data
ZIP_URL = "https://ndownloader.figshare.com/files/22515785"
APNEA_FILES = [("GDN0004_3_Apnea.mat", 674646573, 10215740), ("GDN0005_3_Apnea.mat", 896507605, 10524633),
               ("GDN0006_3_Apnea.mat", 1113182808, 10066201)]   # (name, local-header offset, size) from the zip CD


def fetch_apnea(dest=DATA):
    """HTTP range requests out of datasets_subject_01_to_10_scidata.zip (stored, uncompressed entries)."""
    os.makedirs(dest, exist_ok=True)
    for name, off, size in APNEA_FILES:
        out = os.path.join(dest, name)
        if os.path.exists(out) and os.path.getsize(out) == size:
            continue
        hdr = subprocess.run(["curl", "-sfL", "-r", f"{off}-{off + 29}", ZIP_URL], check=True, capture_output=True).stdout
        sig, _, _, meth, _, _, crc, _, _, nl, el = struct.unpack("<IHHHHHIIIHH", hdr)
        assert sig == 0x04034B50 and meth == 0
        start = off + 30 + nl + el
        subprocess.run(["curl", "-sfL", "-r", f"{start}-{start + size - 1}", ZIP_URL, "-o", out], check=True)
        assert zlib.crc32(open(out, "rb").read()) & 0xFFFFFFFF == crc, f"CRC mismatch {name}"
        print("fetched", name, size)


def load_real(path):
    import validate_real_breathing as VR
    d = VR.load_mat(path)
    fs = float(np.asarray(d["fs_radar"]).ravel()[0])
    iq = np.asarray(d["radar_i"], float) + 1j * np.asarray(d["radar_q"], float)
    x20 = VR.decimate_to(iq, fs)
    z20 = VR.decimate_to(np.asarray(d["tfm_z0"], float), float(np.asarray(d["fs_z0"]).ravel()[0]))
    n = min(len(x20), len(z20))
    marker = None
    if "tfm_intervention" in d and np.size(d["tfm_intervention"]):
        iv = np.asarray(d["tfm_intervention"], float)
        fi = float(np.asarray(d.get("fs_intervention", fs)).ravel()[0])
        lo = iv < 0.5 * (iv.max() + iv.min())
        marker = [(a / fi, b / fi) for a, b in vitals._runs(lo)]
    return x20[:n], z20[:n], marker, str(np.asarray(d["measurement_info"]).ravel()[2])


LAMBDA_24G = R.C0 / 24.0e9


def run_real_resting(files, win, hop):
    out = []
    for p in files:
        x, z, _, subj = load_real(p)
        for s in range(0, len(x) - int(win * 20) + 1, int(hop * 20)):
            sl = slice(s, s + int(win * 20))
            vr = vitals.extract(x[sl], 20.0, kind="iq", wavelength=LAMBDA_24G)
            vz = vitals.extract(z[sl], 20.0, kind="relative")
            out.append(dict(subject=subj, t=s / 20 + win / 2, radar=vr, ref=vz))
    return out


def summarise_resting(rows):
    ok = [r for r in rows if r["radar"]["rate_bpm"] and r["ref"]["rate_bpm"]]
    e = np.array([abs(r["radar"]["rate_bpm"] - r["ref"]["rate_bpm"]) for r in ok])
    subj = {}
    for s in sorted({r["subject"] for r in ok}):
        es = np.array([abs(r["radar"]["rate_bpm"] - r["ref"]["rate_bpm"]) for r in ok if r["subject"] == s])
        subj[s] = dict(windows=len(es), mae=float(es.mean()), within_1p5=float(np.mean(es <= 1.5)),
                       ref_rate=float(np.mean([r["ref"]["rate_bpm"] for r in ok if r["subject"] == s])))
    pats = Counter(r["radar"]["pattern"] for r in rows)
    agree = float(np.mean([r["radar"]["pattern"] == r["ref"]["pattern"] for r in rows]))
    ap_fp = float(np.mean([r["radar"]["apnea_events"] > 0 for r in rows]))
    cvd = np.array([abs(r["radar"]["cv"] - r["ref"]["cv"]) for r in ok if r["radar"]["cv"] is not None and r["ref"]["cv"] is not None])
    depth = [r["radar"]["depth_rel"] for r in ok if r["radar"]["depth_rel"] is not None]
    return dict(windows=len(rows), mae_bpm=float(e.mean()), median_ae=float(np.median(e)), within_1p5=float(np.mean(e <= 1.5)),
                per_subject=subj, radar_patterns=dict(pats), pattern_agree_with_ref=agree, apnea_false_alarm_windows=ap_fp,
                cv_abs_diff_median=float(np.median(cvd)) if cvd.size else None,
                depth_rel_median=float(np.median(depth)) if depth else None)


def run_real_apnea(files):
    out = []
    for p in files:
        x, z, marker, subj = load_real(p)
        vr = vitals.extract(x, 20.0, kind="iq", wavelength=LAMBDA_24G, keep_arrays=True)
        vz = vitals.extract(z, 20.0, kind="relative", keep_arrays=True)
        holds = [(a, b) for a, b in (marker or []) if b - a >= 10]
        det = vr["_apneas_s"]
        ovl = lambda a, b, c, d: max(0.0, min(b, d) - max(a, c))
        hits = [any(ovl(a, b, c, d) >= 0.5 * (b - a) for c, d in det) for a, b in holds]
        fps = [any(ovl(c, d, a, b) > 0 for a, b in holds) for c, d in det]
        cover = sum(ovl(a, b, c, d) for a, b in holds for c, d in det) / max(sum(b - a for a, b in holds), 1e-9)
        onset = [min(abs(c - a) for c, d in det) for a, b in holds if det]
        # 60 s windows centred on each marked hold and on breathing stretches: does the window pattern flag pauses?
        win = []
        for s in range(0, len(x) - 1200 + 1, 200):
            t0, t1 = s / 20, s / 20 + 60
            has = any(ovl(a, b, t0, t1) >= 12 for a, b in holds)
            v = vitals.extract(x[s:s + 1200], 20.0, kind="iq", wavelength=LAMBDA_24G)
            win.append(dict(t0=t0, has_hold=has, pattern=v["pattern"], apnea=v["apnea_events"]))
        out.append(dict(subject=subj, duration=len(x) / 20, holds=holds, detected=det, hits=hits,
                        false_events=int(len(fps) - sum(fps)), coverage=cover, onset_err_s=onset,
                        ref_detected=vz["_apneas_s"], windows=win, disp=vr["_displacement"], radar=vr))
    return out


def summarise_apnea(rows):
    holds = sum(len(r["holds"]) for r in rows)
    hits = sum(sum(r["hits"]) for r in rows)
    w = [x for r in rows for x in r["windows"]]
    wh = [x for x in w if x["has_hold"]]
    wn = [x for x in w if not x["has_hold"]]
    return dict(files=len(rows), holds=holds, holds_detected=hits, false_events=sum(r["false_events"] for r in rows),
                coverage=[round(r["coverage"], 2) for r in rows],
                onset_err_s_median=float(np.median([e for r in rows for e in r["onset_err_s"]])) if hits else None,
                hold_lengths_s=[round(b - a) for r in rows for a, b in r["holds"]],
                win_with_hold=len(wh), win_hold_flagged=float(np.mean([x["pattern"] == "irregular_with_pauses" for x in wh])) if wh else None,
                win_without_hold=len(wn), win_nohold_flagged=float(np.mean([x["apnea"] > 0 for x in wn])) if wn else None)


# ============================================================================ plot
def plot(rows, syn60, resting, apnea, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap
    plt.rcParams.update({"font.size": 8, "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.spines.top": False, "axes.spines.right": False})
    fig, ax = plt.subplots(2, 3, figsize=(15, 8.6))
    # (1) confusion matrix, 60 s dwell
    M = np.array(syn60["confusion"])
    cmap = LinearSegmentedColormap.from_list("b", ["#fcfcfb", "#cde2fb", "#86b6ef", "#2a78d6", "#184f95"])
    a = ax[0, 0]
    rn = M / np.maximum(M.sum(1, keepdims=True), 1)
    a.imshow(rn, cmap=cmap, vmin=0, vmax=1, aspect="auto")
    for i in range(M.shape[0]):
        for j in range(M.shape[1]):
            if M[i, j]:
                a.text(j, i, str(M[i, j]), ha="center", va="center", color="#ffffff" if rn[i, j] > 0.55 else INK, fontsize=8)
    a.set_xticks(range(len(LABELS)), [l.replace("_", "\n") for l in LABELS], fontsize=7)
    a.set_yticks(range(len(STATES)), [s.replace("_", " ") for s in STATES], fontsize=7)
    a.set_xlabel("vitals.py pattern"); a.set_ylabel("hidden state (synthetic)")
    a.set_title(f"Synthetic, 60 s dwell: {100 * syn60['accuracy']:.0f}% correct of {syn60['detected']} detected scans", color=INK)
    # (2) accuracy vs measured scan quality (landing / platform motion), 20 s and 60 s
    a = ax[0, 1]
    for dw, col in ((20, ORANGE), (60, BLUE)):
        s = summarise_synth(rows, dw)
        xs = np.arange(len(s["by_quality"]))
        a.plot(xs, [100 * b["acc"] for b in s["by_quality"]], "o-", color=col, lw=2, ms=7, label=f"{dw} s dwell")
        for x_, b in zip(xs, s["by_quality"]):
            a.annotate(f"{100 * b['acc']:.0f}%", (x_, 100 * b["acc"]), textcoords="offset points", xytext=(0, 8),
                       ha="center", fontsize=7, color=INK2)
        a.set_xticks(xs, [f"quality {b['q']}\n(n={b['n']})" for b in s["by_quality"]], fontsize=7)
    a.set_ylim(0, 105); a.set_ylabel("% of detected scans with the right pattern"); a.set_xlabel("scan quality measured by the DSP")
    a.set_title("Synthetic: pattern accuracy vs scan quality and dwell", color=INK)
    a.legend(fontsize=7, frameon=False, loc="lower right"); a.grid(color=GRID, lw=.6)
    # (3) example: truth chest vs radar-recovered displacement
    a = ax[0, 2]
    ex = [r for r in rows if r["dwell"] == 60 and r["detected"] and "_displacement" in r["v"] and r["state"] == "irregular_apneic"
          and r["v"]["apnea_events"] > 0]
    ex = max(ex, key=lambda r: r["q_meas"]) if ex else next((r for r in rows if r["detected"] and "_displacement" in r["v"]), None)
    if ex is not None:
        d = ex["v"]["_displacement"]
        t = np.arange(d.size) / 20.0
        a.plot(t, (d - np.median(d)) / (np.std(d) + 1e-30), color=BLUE, lw=1.2, label="radar-recovered chest displacement")
        for k, (s0, s1) in enumerate(ex["v"]["_apneas_s"]):
            a.axvspan(s0, s1, color=ORANGE, alpha=.18, lw=0, label="detected pause > 10 s" if k == 0 else None)
        a.set_title(f"Synthetic '{ex['state'].replace('_', ' ')}' scan -> {ex['v']['pattern'].replace('_', ' ')}\n"
                    f"hint '{ex['v']['triage_hint']}', rate {ex['v']['rate_bpm']:.0f}/min, SNR {ex['snr_db']:.0f} dB, quality {ex['q_meas']:.2f}",
                    color=INK)
        a.set_xlabel("time (s)"); a.set_ylabel("displacement (z-scored; absolute mm unreliable)"); a.legend(fontsize=7, frameon=False, loc="upper right"); a.grid(color=GRID, lw=.6)
    # (4) synthetic rate: measured vs true
    a = ax[1, 0]
    good = [r for r in rows if r["dwell"] == 60 and r["detected"] and r["v"]["pattern"] != "unclear"]
    a.plot([5, 40], [5, 40], color=INK2, lw=1, ls=":")
    a.scatter([r["rate_true"] for r in good], [r["v"]["rate_bpm"] for r in good], s=14, color=BLUE, alpha=.6, edgecolor="none")
    a.set_xlabel("true mean rate (bpm, synthetic)"); a.set_ylabel("vitals.py rate (bpm)")
    a.set_title(f"Synthetic rate, 60 s, SNR >= 15 dB: MAE {syn60['rate_mae_good']:.2f} bpm", color=INK); a.grid(color=GRID, lw=.6)
    # (5) real Resting: radar vs reference rate
    a = ax[1, 1]
    for subj, col in zip(sorted({r["subject"] for r in resting}), (BLUE, ORANGE, AQUA)):
        rr = [r for r in resting if r["subject"] == subj]
        a.scatter([r["ref"]["rate_bpm"] for r in rr], [r["radar"]["rate_bpm"] for r in rr], s=16, color=col, alpha=.75,
                  edgecolor="none", label=subj)
    a.plot([4, 20], [4, 20], color=INK2, lw=1, ls=":")
    sr = summarise_resting(resting)
    a.set_xlabel("reference rate, thoracic impedance (bpm)"); a.set_ylabel("radar rate, vitals.py (bpm)")
    a.set_title(f"REAL 24 GHz radar, Resting, 60 s windows: MAE {sr['mae_bpm']:.2f} bpm", color=INK)
    a.legend(fontsize=7, frameon=False); a.grid(color=GRID, lw=.6)
    # (6) real Apnea: detected pauses vs marker
    a = ax[1, 2]
    for k, r in enumerate(apnea):
        y = 3.0 * k
        d = r["disp"]
        t = np.arange(d.size) / 20.0
        from scipy import signal as S
        w = S.sosfiltfilt(S.butter(3, (0.05, 1.2), "bandpass", fs=20, output="sos"), d)
        a.plot(t, y + 0.9 * w / (np.percentile(np.abs(w), 95) + 1e-12), color=INK2, lw=.7)
        for (s0, s1) in r["holds"]:
            a.add_patch(plt.Rectangle((s0, y - 1.15), s1 - s0, 0.3, color=BLUE, lw=0))
        for (s0, s1) in r["detected"]:
            a.add_patch(plt.Rectangle((s0, y - 1.5), s1 - s0, 0.3, color=ORANGE, lw=0))
        a.text(r["duration"] + 2, y, r["subject"], fontsize=7, color=INK, va="center")
    a.set_yticks([]); a.set_xlabel("time (s)")
    sa = summarise_apnea(apnea)
    a.set_title(f"REAL Apnea: {sa['holds_detected']}/{sa['holds']} breath holds found, {sa['false_events']} false", color=INK)
    a.plot([], [], color=BLUE, lw=6, label="marked breath hold (reference)")
    a.plot([], [], color=ORANGE, lw=6, label="vitals.py pause > 10 s")
    a.set_xlim(0, 175); a.set_ylim(-1.8, 3.0 * len(apnea) + 0.6)
    a.legend(fontsize=7, frameon=False, loc="upper center", ncol=2, bbox_to_anchor=(0.5, 1.0))
    fig.suptitle("sim/vitals.py breathing-pattern aid. Top row + bottom left: SYNTHETIC radar scans through simulated rubble. "
                 "Bottom middle/right: REAL 24 GHz CW radar at 0.4 m in air (Schellenberger 2020, CC BY 4.0). Not medical advice.",
                 fontsize=9, color=INK)
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)


def _clean(o):
    if isinstance(o, dict):
        return {k: _clean(v) for k, v in o.items() if not k.startswith("_") and k not in ("disp", "v", "radar", "ref", "x")}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    return o


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=250, help="synthetic scans per dwell")
    ap.add_argument("--dwells", default="20,60")
    ap.add_argument("--win", type=float, default=60.0, help="real Resting window (s)")
    ap.add_argument("--hop", type=float, default=30.0)
    ap.add_argument("--fetch-apnea", action="store_true")
    ap.add_argument("--skip-synth", action="store_true")
    ap.add_argument("--cache", default=None, help="pickle of simulated candidate-bin I/Q (re-tune without re-simulating)")
    ap.add_argument("--plot", default=None)
    ap.add_argument("--json", default=None)
    a = ap.parse_args(argv)
    if a.fetch_apnea:
        fetch_apnea()
        return
    res = {}
    rows = []
    if not a.skip_synth:
        rows = run_synthetic(a.n, [float(d) for d in a.dwells.split(",")], a.cache)
        for dw in sorted({r["dwell"] for r in rows}):
            s = summarise_synth(rows, dw)
            res[f"synthetic_{dw:.0f}s"] = s
            print(f"SYNTH {dw:.0f}s: {s['detected']}/{s['scans']} detected; pattern acc {100 * s['accuracy']:.0f}% "
                  f"(of non-unclear {100 * s['accuracy_called']:.0f}%, triage agreement {100 * s['triage_agreement_called']:.0f}%), "
                  f"unclear {100 * s['unclear_frac']:.0f}%; rate MAE (SNR>=15, q>=.6, n={s['n_good']}) {s['rate_mae_good']:.2f} bpm, "
                  f"within 1.5 {100 * s['rate_within_1p5_good']:.0f}%; apnea-state hit {s['apnea_state_hit']:.2f}, "
                  f"false apnea {s['apnea_false_alarm']:.2f}; depth log-corr {s['depth_log_corr']}, ratio p10/50/90 {s['depth_ratio_p10_50_90']}")
            print("   per state recall:", {k: f"{v['recall']:.2f} (n={v['n']})" for k, v in s["per_state"].items()})
            print("   by SNR:", s["by_snr"]); print("   by path:", s["by_path"]); print("   by quality:", s["by_quality"])
            print("   confusion (rows", STATES, "cols", LABELS, "):"); [print("    ", r) for r in s["confusion"]]
    rest_files = sorted(os.path.join(DATA, f) for f in os.listdir(DATA) if f.endswith("_Resting.mat"))
    apn_files = sorted(os.path.join(DATA, f) for f in os.listdir(DATA) if f.endswith("_Apnea.mat"))
    resting = run_real_resting(rest_files, a.win, a.hop) if rest_files else []
    if resting:
        res["real_resting"] = summarise_resting(resting)
        print("REAL Resting:", json.dumps(res["real_resting"]))
        res["real_resting_30s"] = summarise_resting(run_real_resting(rest_files, 30.0, 15.0))
        print("REAL Resting 30 s:", json.dumps(res["real_resting_30s"]))
    apnea = run_real_apnea(apn_files) if apn_files else []
    if apnea:
        res["real_apnea"] = summarise_apnea(apnea)
        res["real_apnea_detail"] = [dict(subject=r["subject"], holds=[[round(x, 1) for x in h] for h in r["holds"]],
                                         detected=[[round(x, 1) for x in h] for h in r["detected"]],
                                         ref_detected=[[round(x, 1) for x in h] for h in r["ref_detected"]],
                                         radar_rate=r["radar"]["rate_bpm"], movement=r["radar"]["movement"]) for r in apnea]
        print("REAL Apnea:", json.dumps(res["real_apnea"]))
        for d in res["real_apnea_detail"]:
            print("   ", d)
    if a.plot and rows and resting and apnea:
        plot(rows, res["synthetic_60s"], resting, apnea, a.plot)
        print("wrote", a.plot)
    if a.json:
        with open(a.json, "w") as f:
            json.dump(_clean(res), f, indent=1)


if __name__ == "__main__":
    main()
