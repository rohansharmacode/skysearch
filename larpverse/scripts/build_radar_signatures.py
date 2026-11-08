"""Radar signature library: assets, HUD playback streams, identification confusion matrix and gallery figure.

Every scan is SYNTHETIC (sim/radar_signatures.render -> sim/radar_dsp.simulate_if) and goes through the real
chain (radar_dsp.process_if + sim/vitals.identify). Truth is read only here, by the evaluator.

    scripts/heavy.sh .venv/bin/python scripts/build_radar_signatures.py eval --n 25 --seed0 0  --out /tmp/a.json
    scripts/heavy.sh .venv/bin/python scripts/build_radar_signatures.py eval --n 25 --seed0 25 --out /tmp/b.json
    scripts/heavy.sh .venv/bin/python scripts/build_radar_signatures.py report /tmp/a.json /tmp/b.json --json docs/radar_signatures_eval.json
    scripts/heavy.sh .venv/bin/python scripts/build_radar_signatures.py assets      # assets/radar_signatures/* + gallery

Outputs
  assets/radar_signatures/<class>.npz   slow-time I/Q + displacement, range-slow-time map (dB), spectrogram
  assets/radar_signatures/<class>.png   range-time map + displacement + spectrogram
  assets/radar_signatures/<class>.json  10 Hz HUD playback stream (format in docs/RADAR_SIGNATURES.md)
  docs/img/radar_signatures.png         presentation gallery (dark, one row per class)
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from collections import Counter

import numpy as np
from scipy import signal

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT)
from sim import radar_dsp as R, radar_signatures as RS, vitals as V  # noqa: E402

OUT = os.path.join(ROOT, "assets", "radar_signatures")
GALLERY = os.path.join(ROOT, "docs", "img", "radar_signatures.png")
PRED = list(RS.HUMAN) + ["no_signal", "debris_shift", "small_animal", "machinery", "rotor", "moving_person"]
TRUTH_PRED = {**{c: c for c in RS.HUMAN}, **{c: RS.EXPECTED[c][0] for c in RS.CONFOUNDERS}}
# dark presentation palette: red = radar
BG, PANEL, INK, INK2, RED, GRID = "#0e0f12", "#16181d", "#f2f2f2", "#9a9ca3", "#ff3b30", "#2a2d34"
HUMAN_TAG = {"calm": "stable for now", "distressed_tachypnea": "check soon", "unconscious_slow": "check responsiveness",
             "irregular_apneic": "check first"}


def verdict_label(o):
    return o["condition"] if o["source"] == "human_breathing" else o["source"]


# ============================================================================ evaluation
def run_eval(n, seed0, out):
    rows = []
    for cls in RS.CLASSES:
        for i in range(n):
            seed = 1000 + seed0 + i
            dwell = float(np.random.default_rng([seed, 5]).uniform(45.0, 60.0))
            sc = RS.render(cls, seed, dwell)
            o = V.identify(sc["iq"], sc["params"])
            t = sc["truth"]
            rows.append(dict(cls=cls, seed=seed, dwell=dwell, pred=verdict_label(o), conf=o["confidence"],
                             range_true=t["range_m"], range_pred=o["range_m"], detected=o["dsp"]["candidates"] > 0,
                             quality=t["quality"], hint=o["vitals"]["triage_hint"]))
        print(cls, Counter(r["pred"] for r in rows if r["cls"] == cls), flush=True)
    with open(out, "w") as f:
        json.dump(rows, f)


def report(paths, out_json=None):
    rows = [r for p in paths for r in json.load(open(p))]
    M = np.zeros((len(RS.CLASSES), len(PRED)), int)
    for r in rows:
        M[RS.CLASSES.index(r["cls"]), PRED.index(r["pred"])] += 1
    n = M.sum(1)
    correct = np.array([M[i, PRED.index(TRUTH_PRED[c])] for i, c in enumerate(RS.CLASSES)])
    # binary: "is there a person here?" person = the 4 breathing conditions + a moving / knocking person
    person = list(RS.HUMAN) + ["moving_person"]
    p_rows = [RS.CLASSES.index(c) for c in person]
    n_rows = [RS.CLASSES.index(c) for c in RS.CONFOUNDERS if c != "moving_person"]
    p_cols = [PRED.index(c) for c in person]
    tp = M[np.ix_(p_rows, p_cols)].sum()
    fn = M[p_rows].sum() - tp
    fp = M[np.ix_(n_rows, p_cols)].sum()
    tn = M[n_rows].sum() - fp
    det = {c: float(np.mean([r["detected"] for r in rows if r["cls"] == c])) for c in RS.CLASSES}
    res = dict(n_per_class=n.tolist(), classes=list(RS.CLASSES), predicted=PRED, confusion=M.tolist(),
               recall={c: float(correct[i] / max(n[i], 1)) for i, c in enumerate(RS.CLASSES)},
               accuracy=float(correct.sum() / n.sum()),
               person_vs_not=dict(tp=int(tp), fn=int(fn), fp=int(fp), tn=int(tn),
                                 sensitivity=float(tp / max(tp + fn, 1)), specificity=float(tn / max(tn + fp, 1))),
               moving_person_flagged_urgent=float(np.mean([r["hint"] == "urgent" for r in rows if r["cls"] == "moving_person"])),
               dsp_detection_rate=det, dwell_s=[45, 60])
    w = max(len(c) for c in RS.CLASSES)
    print(" " * (w + 2) + " ".join(f"{p[:9]:>9s}" for p in PRED))
    for i, c in enumerate(RS.CLASSES):
        print(f"{c:{w}s}  " + " ".join(f"{v:9d}" for v in M[i]) + f"   recall {100 * res['recall'][c]:.0f}%  (DSP detected {100 * det[c]:.0f}%)")
    print(f"overall {100 * res['accuracy']:.0f}%; person vs not: sensitivity {100 * res['person_vs_not']['sensitivity']:.0f}%, "
          f"specificity {100 * res['person_vs_not']['specificity']:.0f}% ({res['person_vs_not']})")
    if out_json:
        with open(out_json, "w") as f:
            json.dump(res, f, indent=1)
    return res


# ============================================================================ assets
def displacement_series(o, sc):
    v = V.extract(V.motion_compensated(sc["iq"], sc["params"])[:, o["bin"]], sc["params"].fs_slow, kind="iq",
                  wavelength=sc["params"].wavelength, range_m=o["range_m"], rubble_path_m=o["range_m"],
                  params=sc["params"], keep_arrays=True)
    return v["_displacement"], v


def spectrogram(d, fs, win_s=6.0):
    f, t, Z = signal.stft(signal.detrend(d), fs, nperseg=int(win_s * fs), noverlap=int(win_s * fs) - int(fs / 2),
                          boundary=None, padded=False)
    return f, t, 20 * np.log10(np.abs(Z) + 1e-12)


def hud_stream(sc, hop_s=2.0, min_s=15.0, trail_s=60.0, fps=10.0):
    """Causal playback: a verdict every hop_s from everything seen so far (up to trail_s), held between updates."""
    P, iq = sc["params"], sc["iq"]
    fs = P.fs_slow
    full = V.identify(iq, P)
    d, _ = displacement_series(full, sc)
    step = int(round(fs / fps))
    wave = d[::step]
    wave = 1e3 * (wave - np.median(wave))
    frames, last, prev_raw, shown = [], None, None, None
    for k in range(len(wave)):
        t = k / fps
        if t >= min_s and (last is None or t - last["t_upd"] >= hop_s - 1e-9):
            b = int(t * fs)
            a = max(0, b - int(trail_s * fs))
            Pw = R.RadarParams(**{**P.__dict__, "dwell": (b - a) / fs})
            o = V.identify(iq[a:b], Pw)
            v = o["vitals"]
            raw = o["condition"] if o["source"] == "human_breathing" else o["source"]
            if shown is None or raw == prev_raw:              # hysteresis: a new verdict must repeat once to be shown
                shown = raw
            prev_raw = raw
            last = {"t_upd": t, "rate_bpm": v["rate_bpm"], "condition": o["condition"], "source": o["source"],
                    "conf": round(float(o["confidence"]), 2), "hint": v["triage_hint"], "text": v["condition_text"],
                    "shown": shown}
        frames.append({"t": round(t, 1), "w": [max(0, k - int(10 * fps)), k + 1],
                       **({k_: last[k_] for k_ in ("rate_bpm", "condition", "source", "conf", "hint", "text", "shown")} if last else
                          {"rate_bpm": None, "condition": "unknown", "source": None, "conf": 0.0, "hint": "assess",
                           "text": "listening... (needs 15 s)", "shown": None})})
    return {"format": "radar-signature-hud-v1", "class": sc["truth"]["class"], "label": sc["truth"]["label"],
            "synthetic": True, "fps": fps, "window_s": 10.0, "units": "mm (relative; amplitude is low-confidence)",
            "range_m": full["range_m"], "final": {"source": full["source"], "condition": full["condition"],
                                                  "conf": round(float(full["confidence"]), 2), "text": full["vitals"]["condition_text"]},
            "wave_mm": [round(float(x), 3) for x in wave], "frames": frames}


def _cmap():
    from matplotlib.colors import LinearSegmentedColormap
    return LinearSegmentedColormap.from_list("radar", [BG, "#3a0d0b", "#a3150f", RED, "#ff9a8f", "#ffffff"])


def _style(ax):
    ax.set_facecolor(PANEL)
    for sp in ax.spines.values():
        sp.set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=7)
    ax.xaxis.label.set_color(INK2); ax.yaxis.label.set_color(INK2)


def draw_row(axs, rec, first, last):
    """range-time map | displacement | spectrogram, shared conventions for asset PNGs and the gallery."""
    cmap = _cmap()
    a, b, c = axs
    keep = rec["r_axis"] >= 1.0                      # the DSP ignores < 1 m (antenna leakage, pod ground bounce)
    rt = rec["rt_db"][:, keep]
    vmax = np.percentile(rt, 99.7)
    a.imshow(rt.T, origin="lower", aspect="auto", cmap=cmap, vmin=vmax - 30, vmax=vmax,
             extent=[0, rec["t"][-1], rec["r_axis"][keep][0], rec["r_axis"][-1]])
    a.axhline(rec["range_m"], color=INK, lw=0.6, ls=":", alpha=0.7)
    a.set_ylabel("range (m)" if first else "", fontsize=7)
    b.plot(rec["t"], rec["disp_mm"], color=INK, lw=0.8)
    lim = max(np.percentile(np.abs(rec["disp_mm"]), 99.5), 1e-3) * 1.15
    b.set_ylim(-lim, lim)
    for s0, s1 in rec.get("pauses", []):
        b.axvspan(s0, s1, color=RED, alpha=0.25, lw=0)
    f, tt, S = rec["spec"]
    sel = f <= 8.0
    smax = np.percentile(S[sel], 99.5)
    c.imshow(S[sel], origin="lower", aspect="auto", cmap=cmap, vmin=smax - 35, vmax=smax,
             extent=[tt[0], tt[-1], 0, f[sel][-1]])
    c.axhspan(0.1, 0.75, color=INK, alpha=0.06, lw=0)
    for ax in axs:
        _style(ax)
        ax.set_xlim(0, rec["t"][-1])
        if not last:
            ax.set_xticklabels([])
    if last:
        for ax in axs:
            ax.set_xlabel("time (s)", fontsize=7)
    if first:
        a.set_title("range vs time, 1-10 m (clutter removed; dotted = identified return)", color=INK2, fontsize=8, loc="left")
        b.set_title("chest / surface displacement at the return (relative mm)", color=INK2, fontsize=8, loc="left")
        c.set_title("micro-Doppler spectrogram of the phase (0-8 Hz; breathing band shaded)", color=INK2, fontsize=8, loc="left")


def asset_record(cls, seed=0, dwell=60.0):
    sc = RS.render(cls, seed, dwell)
    P = sc["params"]
    o = V.identify(sc["iq"], P)
    d, v = displacement_series(o, sc)
    Xc = V.motion_compensated(sc["iq"], P)
    Y = Xc - Xc.mean(axis=0, keepdims=True)
    nr = int(np.ceil(10.0 / P.range_res)) + 1
    rt_db = (20 * np.log10(np.abs(Y[:, :nr]) + 1e-9)).astype(np.float32)
    spec = spectrogram(d, P.fs_slow)
    return sc, o, v, dict(t=P.t_slow, r_axis=P.range_axis[:nr], rt_db=rt_db, disp_mm=1e3 * (d - np.median(d)),
                          spec=spec, range_m=o["range_m"], pauses=v.get("_apneas_s", []), bin=o["bin"])


def pick_exemplars(eval_paths):
    """Showcase instances: per class, the highest-confidence CORRECTLY identified scan from the evaluation rows.
    (Curated on purpose for the video; the confusion matrix is the honest picture.)"""
    rows = [r for p in eval_paths for r in json.load(open(p))]
    out = {}
    for cls in RS.CLASSES:
        ok = [r for r in rows if r["cls"] == cls and r["pred"] == TRUTH_PRED[cls]]
        ok.sort(key=lambda r: (-r["conf"], r["seed"]))
        out[cls] = (ok[0]["seed"], ok[0]["dwell"]) if ok else (0, 60.0)
    return out


def build_assets(seed=0, dwell=60.0, gallery=True, picks=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(OUT, exist_ok=True)
    os.makedirs(os.path.dirname(GALLERY), exist_ok=True)
    recs = []
    for cls in RS.CLASSES:
        sd, dw = (picks or {}).get(cls, (seed, dwell))
        sc, o, v, rec = asset_record(cls, sd, dw)
        P = sc["params"]
        f, tt, S = rec["spec"]
        np.savez_compressed(os.path.join(OUT, f"{cls}.npz"), synthetic=True, label=RS.LABEL[cls], fs_slow=P.fs_slow,
                            render_seed=sd, dwell_s=dw,
                            wavelength_m=P.wavelength, range_res_m=P.range_res, range_m=o["range_m"], bin=o["bin"],
                            slow_time_iq=V.motion_compensated(sc["iq"], P)[:, o["bin"]].astype(np.complex64),
                            displacement_mm=rec["disp_mm"].astype(np.float32), t_s=P.t_slow.astype(np.float32),
                            range_axis_m=rec["r_axis"].astype(np.float32), range_slow_time_db=rec["rt_db"].astype(np.float16),
                            spec_f_hz=f.astype(np.float32), spec_t_s=tt.astype(np.float32), spec_db=S.astype(np.float16),
                            verdict=json.dumps({"source": o["source"], "condition": o["condition"],
                                                "confidence": o["confidence"], "text": v["condition_text"]}))
        with open(os.path.join(OUT, f"{cls}.json"), "w") as fh:
            json.dump(hud_stream(sc), fh, separators=(",", ":"))
        fig, axs = plt.subplots(1, 3, figsize=(15, 3.0), facecolor=BG)
        draw_row(axs, rec, True, True)
        fig.suptitle(f"{RS.LABEL[cls]}  ->  identified: {verdict_text(o)}   (SYNTHETIC)", color=INK, fontsize=10, x=0.01, ha="left")
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, f"{cls}.png"), dpi=110, facecolor=BG)
        plt.close(fig)
        recs.append((cls, o, rec))
        print("asset", cls, verdict_text(o), flush=True)
    if gallery:
        fig = plt.figure(figsize=(17, 2.05 * len(recs) + 1.0), facecolor=BG)
        gs = fig.add_gridspec(len(recs), 4, width_ratios=[1.25, 1.6, 1.6, 1.6], hspace=0.25, wspace=0.14,
                              left=0.01, right=0.99, top=1 - 1.0 / (2.05 * len(recs) + 1.0), bottom=0.035)
        for i, (cls, o, rec) in enumerate(recs):
            lab = fig.add_subplot(gs[i, 0]); lab.axis("off"); lab.set_facecolor(BG)
            human = cls in RS.HUMAN or cls == "moving_person"
            short = RS.LABEL[cls].split(" (")[0].split(":")[0]
            lab.text(0.02, 0.78, ("PERSON  " if human else "NOT A PERSON  ") + short,
                     color=RED if human else INK, fontsize=9.5, weight="bold", transform=lab.transAxes, va="center", wrap=True)
            sub = HUMAN_TAG.get(cls, "reject" if cls not in ("moving_person",) else "listen & call out")
            lab.text(0.02, 0.52, sub, color=INK2, fontsize=8.5, transform=lab.transAxes, va="center")
            lab.text(0.02, 0.22, "identified: " + verdict_text(o), color=INK, fontsize=8, transform=lab.transAxes,
                     va="center", wrap=True)
            draw_row([fig.add_subplot(gs[i, j]) for j in (1, 2, 3)], rec, i == 0, i == len(recs) - 1)
        fig.suptitle("Radar breathing / micro-Doppler signatures from a sensor pod on the rubble  -  SYNTHETIC generator, "
                     "real DSP chain (sim/radar_dsp.py + sim/vitals.py)", color=INK, fontsize=12, x=0.01, y=0.995, ha="left", va="top")
        fig.savefig(GALLERY, dpi=110, facecolor=BG)
        plt.close(fig)
        print("wrote", GALLERY)


def verdict_text(o):
    if o["source"] == "human_breathing":
        return f"{o['condition'].replace('_', ' ')} ({100 * o['confidence']:.0f}%)"
    return f"{o['source'].replace('_', ' ')} ({100 * o['confidence']:.0f}%)"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    e = sp.add_parser("eval"); e.add_argument("--n", type=int, default=25); e.add_argument("--seed0", type=int, default=0)
    e.add_argument("--out", required=True)
    r = sp.add_parser("report"); r.add_argument("paths", nargs="+"); r.add_argument("--json", default=None)
    a = sp.add_parser("assets"); a.add_argument("--seed", type=int, default=0); a.add_argument("--no-gallery", action="store_true")
    a.add_argument("--pick", nargs="*", default=None, help="eval JSON(s): use a correctly identified instance per class")
    x = ap.parse_args(argv)
    if x.cmd == "eval":
        run_eval(x.n, x.seed0, x.out)
    elif x.cmd == "report":
        report(x.paths, x.json)
    else:
        build_assets(x.seed, gallery=not x.no_gallery, picks=pick_exemplars(x.pick) if x.pick else None)


if __name__ == "__main__":
    main()
