"""Offline smoothness check for sim/flight_motion.py (no GPU, no Isaac): pose logs, profile plots, metrics.

    # scripted ~60 s showcase on the seed-7 grid: takeoff, cruise, mid-flight re-plan, pod winch, perch landing
    .venv/bin/python scripts/motion_preview.py showcase            # -> media/motion/showcase.json + docs/img/motion_profiles.png
    # seed-7 mission dry run, SmoothFlight vs the old KinematicFlight + Isaac's per-step lerp (heavy: use heavy.sh)
    scripts/heavy.sh .venv/bin/python scripts/motion_preview.py mission --max-t 600

Both modes print before/after metrics measured the way a renderer sees the motion: positions sampled at
60 fps, differentiated by finite differences (velocity jumps = frames whose implied acceleration exceeds
5 m/s^2, i.e. twice the 2.5 m/s^2 limit).  "Before" = KinematicFlight stepped at the mission dt (1 s) and
linearly interpolated per frame (isaac/run_mission.py interp_drones); "after" = SmoothFlight.motion_sample(t).

The pose log plays in animation/motion-preview.html (three.js, vendored); record it with
    scripts/heavy.sh --browser uv run --offline --no-project --with playwright --with imageio-ffmpeg python \\
        scripts/record_animation.py "http://localhost:8765/animation/motion-preview.html" media/motion/motion_preview.mp4 --duration 60
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from sim.coordinator import LAND_HEIGHT, KinematicFlight  # noqa: E402
from sim.flight_motion import SmoothFlight  # noqa: E402

FPS = 60
DRONES = ("d1", "d2", "d3")


# --------------------------------------------------------------------------- site
def seed7_site():
    from sim.grid_world import GridWorld
    from sim.scenario import make_scenario
    sc = make_scenario(7)
    w = GridWorld(sc)
    pd = sc.public_dict()
    pads = {f"d{i + 1}": tuple(xy) for i, xy in enumerate(pd["drone_pads"][:3])}
    return w.grid, pads, dict(w.pile_xy)


def flat_spot(grid, centre, towards, r0=7.0, r1=13.0):
    """A flat, low cell between r0 and r1 m from ``centre`` on the side facing ``towards`` (perch landing)."""
    c, tw = np.asarray(centre, float), np.asarray(towards, float)
    base = math.atan2(tw[1] - c[1], tw[0] - c[0])
    best = None
    for r in np.arange(r0, r1, 0.5):
        for da in np.radians(np.arange(-60, 61, 5)):
            x, y = c[0] + r * math.cos(base + da), c[1] + r * math.sin(base + da)
            ring = [(x + 1.6 * math.cos(a), y + 1.6 * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 12, endpoint=False)]
            hs = np.array([grid.height_at(px, py) for px, py in ring] + [grid.height_at(x, y)])
            score = float(hs.max() - hs.min()) + 0.02 * abs(da)
            if best is None or score < best[0]:
                best = (score, x, y, float(grid.height_at(x, y)))
    return best[1:]


# --------------------------------------------------------------------------- scripted showcase
def showcase_script(grid, pads, piles):
    """Mission-like reactive script: returns (on_step(flight, t, events) -> None, notes)."""
    b3 = piles["B3"]
    pod_z = float(grid.height_at(*b3))
    land = flat_spot(grid, piles["A3"], pads["d2"])
    st = {"d1": "takeoff", "d2": "takeoff", "d3": "takeoff", "t_d3": None}
    notes = {"pod": [b3[0], b3[1], pod_z], "perch": list(land)}

    def on_step(fl, t, events):
        states = {p["id"]: p["state"] for p in fl.poses()}
        done = {e["drone_id"] for e in events if e["type"] == "scan_complete"}
        arrived = {e["drone_id"] for e in events if e["type"] == "arrived"}
        if st["d1"] == "takeoff" and states["d1"] == "hovering":
            fl.command({"drone_id": "d1", "type": "goto_pod_scan", "destination": [b3[0], b3[1], pod_z],
                        "hover_z": pod_z + 8.0, "scan_duration": 6.0, "landing_site_id": "B3-pod"})
            st["d1"] = "pod"
        if st["d2"] == "takeoff" and states["d2"] == "hovering":
            fl.command({"drone_id": "d2", "type": "goto_land_scan", "destination": [land[0], land[1], land[2]],
                        "scan_duration": 6.0, "landing_site_id": "A3-perch"})
            st["d2"] = "perch"
        elif st["d2"] == "perch" and "d2" in done:
            fl.command({"drone_id": "d2", "type": "goto", "destination": [-8.0, -40.0, 12.0]})
            st["d2"] = "back"
        if st["d3"] == "takeoff" and states["d3"] == "hovering":
            fl.command({"drone_id": "d3", "type": "goto", "destination": [30.0, -32.0, 14.0]})
            st["d3"], st["t_d3"] = "leg1", t
        elif st["d3"] == "leg1" and t - st["t_d3"] >= 6.0:      # mid-flight re-plan (new waypoint, higher)
            fl.command({"drone_id": "d3", "type": "goto", "destination": [24.0, -6.0, 17.0]})
            st["d3"] = "leg2"
        elif st["d3"] == "leg2" and "d3" in arrived:
            fl.command({"drone_id": "d3", "type": "goto", "destination": [-4.0, -8.0, 17.0]})
            st["d3"] = "leg3"
    return on_step, notes


def run_script(flight, on_step, t_max, dt=1.0):
    """Step like the mission does (t += dt; events = step(dt, t); react).  Returns per-step pose records."""
    t = 0.0
    for d in DRONES:
        flight.command({"drone_id": d, "type": "takeoff"})
    steps = [(0.0, {p["id"]: list(p["pose"]) for p in flight.poses()},
              {p["id"]: p["state"] for p in flight.poses()}, {p["id"]: p.get("winch_len_m", 0.0) for p in flight.poses()})]
    while t < t_max - 1e-9:
        t += dt
        ev = flight.step(dt, t)
        on_step(flight, t, ev)
        steps.append((t, {d: [float(v) for v in flight.drones[d].pos] + [float(flight.drones[d].yaw)] for d in flight.drones},
                      {p["id"]: p["state"] for p in flight.poses()}, {d: float(flight.drones[d].winch) for d in flight.drones}))
    return steps


# --------------------------------------------------------------------------- sampling + metrics
def lerp_frames(steps, fps=FPS):
    """The old renderer: per-frame linear interpolation between mission steps (isaac/run_mission.interp_drones)."""
    ts = np.array([s[0] for s in steps])
    tf = np.arange(ts[0], ts[-1] + 1e-9, 1.0 / fps)
    out = {}
    for d in DRONES:
        P = np.array([s[1][d] for s in steps])
        yaw = np.unwrap(np.radians(P[:, 3]))
        out[d] = {"t": tf, "p": np.column_stack([np.interp(tf, ts, P[:, k]) for k in range(3)]),
                  "yaw": np.degrees(np.interp(tf, ts, yaw))}
    return out


def smooth_frames(flight, t0, t1, fps=FPS, full=False):
    tf = np.arange(t0, t1 + 1e-9, 1.0 / fps)
    out = {}
    for d in DRONES:
        S = [flight.motion_sample(d, t) for t in tf]
        rec = {"t": tf, "p": np.array([s.pos for s in S]), "yaw": np.degrees(np.unwrap(np.radians([s.yaw for s in S]))),
               "tilt": np.array([s.tilt for s in S]), "acc": np.array([s.acc for s in S]),
               "vel": np.array([s.vel for s in S])}
        if full:
            rec["S"] = S
        out[d] = rec
    return out


def metrics(frames, fps=FPS, jump_acc=5.0):
    """Finite-difference metrics of rendered positions: what a viewer sees."""
    res = {}
    for d, r in frames.items():
        p = r["p"]
        v = np.diff(p, axis=0) * fps
        a = np.diff(v, axis=0) * fps
        j = np.diff(a, axis=0) * fps
        dv = np.linalg.norm(np.diff(v, axis=0), axis=1)
        yr = np.abs(np.diff(r["yaw"])) * fps
        res[d] = {"max_speed_mps": float(np.linalg.norm(v[:, :2], axis=1).max()),
                  "max_acc_h": float(np.linalg.norm(a[:, :2], axis=1).max()),
                  "max_acc_v": float(np.abs(a[:, 2]).max()),
                  "max_jerk": float(np.linalg.norm(j, axis=1).max()),
                  "vel_jumps": int((dv > jump_acc / fps).sum()),
                  "max_yaw_rate_dps": float(yr.max())}
    agg = {k: (max if k != "vel_jumps" else sum)(m[k] for m in res.values()) for k in next(iter(res.values()))}
    return agg, res


def nominal_metrics(flight, t0, t1):
    """Analytic limits on the planned path (not finite differences): accel, jerk, tilt, yaw rate, touchdowns."""
    out = {"a_h": 0.0, "a_v": 0.0, "j_h": 0.0, "j_v": 0.0, "tilt": 0.0, "yaw_rate": 0.0, "touchdown_vz": []}
    ts = np.arange(t0, t1, 0.02)
    for d, tl in flight.timelines.items():
        for tr in tl.pieces[1:]:
            a_, b_ = max(tr.t0, t0), min(tr.t_end, t1)
            k = (ts >= a_) & (ts < b_)
            nxt = [p.t0 for p in tl.pieces if p.t0 > tr.t0]
            if nxt:
                k &= ts < min(nxt)
            if not k.any():
                continue
            _, v, a, j = tr.nominal(ts[k])
            out["a_h"] = max(out["a_h"], float(np.hypot(a[:, 0], a[:, 1]).max()))
            out["a_v"] = max(out["a_v"], float(np.abs(a[:, 2]).max()))
            out["j_h"] = max(out["j_h"], float(np.hypot(j[:, 0], j[:, 1]).max()))
            out["j_v"] = max(out["j_v"], float(np.abs(j[:, 2]).max()))
            for t in ts[k][::5]:
                s = tr.sample(t, dist=tl.dist)
                out["tilt"] = max(out["tilt"], s.tilt)
                out["yaw_rate"] = max(out["yaw_rate"], abs(s.yaw_rate))
            if tr.touchdown is not None and tr.touchdown <= t1 and (not nxt or tr.touchdown < min(nxt)):
                out["touchdown_vz"].append(round(float(-touchdown_speed(tr)), 3))
    return out


def touchdown_speed(tr, h=0.05):
    """Vertical speed when the skids are ``h`` above the touchdown point."""
    zt = tr.nominal(tr.touchdown)[0][0][2]
    ts = np.linspace(tr.touchdown - 20.0, tr.touchdown, 4001)
    p, v, _, _ = tr.nominal(ts)
    i = int(np.argmax(p[:, 2] <= zt + h))
    return float(v[i, 2])


# --------------------------------------------------------------------------- outputs
def pose_log(flight, grid, piles, notes, t0, t1, fps=FPS, title="showcase"):
    tf = np.arange(t0, t1 + 1e-9, 1.0 / fps)
    log = {"title": title, "fps": fps, "t0": float(t0), "t1": float(t1), "drones": {},
           "spec": {"span_m": flight.mspec.span_m, "hover_rpm": flight.mspec.hover_rpm, "n_rotors": flight.mspec.n_rotors},
           "piles": {k: [round(v[0], 2), round(v[1], 2)] for k, v in piles.items()}, "notes": notes}
    g = grid
    step = 2                                    # 1 m heightmap for the preview ground
    z = g.z[::step, ::step]
    log["height"] = {"origin": [float(g.origin[0]), float(g.origin[1])], "res": g.res * step, "w": int(z.shape[1]),
                     "h": int(z.shape[0]), "z": [round(float(v), 2) for v in z.ravel()]}
    phases = []
    for d in DRONES:
        S = [flight.motion_sample(d, t) for t in tf]
        ph = []
        for s in S:
            if s.phase not in phases:
                phases.append(s.phase)
            ph.append(phases.index(s.phase))
        log["drones"][d] = {
            "p": [[round(float(v), 3) for v in s.pos] for s in S],
            "q": [[round(float(v), 4) for v in s.quat] for s in S],
            "rpm": [[round(float(v)) for v in s.rotor_rpm] for s in S],
            "w": [round(s.winch_len, 3) for s in S],
            "pod": [None if s.pod is None else [round(float(v), 3) for v in s.pod] for s in S],
            "ph": ph, "tilt": [round(s.tilt, 2) for s in S], "spd": [round(float(np.hypot(*s.vel[:2])), 2) for s in S],
            "vz": [round(float(s.vel[2]), 2) for s in S]}
    log["phases"] = phases
    return log


def plot_profiles(after, before, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cols = {"d1": "#2a78d6", "d2": "#eb6834", "d3": "#1baf7a"}
    fig, ax = plt.subplots(4, 1, figsize=(11, 9.5), sharex=True)
    plt.rcParams.update({"font.size": 9})
    fps = FPS
    for d, r in after.items():
        t = r["t"]
        ax[0].plot(t, np.hypot(r["vel"][:, 0], r["vel"][:, 1]), color=cols[d], lw=1.6, label=f"{d} (smooth)")
        ax[1].plot(t, np.hypot(r["acc"][:, 0], r["acc"][:, 1]), color=cols[d], lw=1.6)
        ax[2].plot(t, r["tilt"], color=cols[d], lw=1.6)
        ax[3].plot(t, r["p"][:, 2], color=cols[d], lw=1.6)
    if before is not None:            # d1 only, the old path as the renderer drew it
        r = before["d1"]
        v = np.diff(r["p"], axis=0) * fps
        a = np.diff(v, axis=0) * fps
        ax[0].plot(r["t"][1:], np.hypot(v[:, 0], v[:, 1]), color="#8a8984", lw=1.0, ls="--", label="d1 before (per-step lerp)")
        ah = np.clip(np.hypot(a[:, 0], a[:, 1]), 0, 12)
        ax[1].plot(r["t"][2:], ah, color="#8a8984", lw=0.8, ls="--")
        ax[3].plot(r["t"], r["p"][:, 2], color="#8a8984", lw=1.0, ls="--")
    ax[1].axhline(2.5, color="#52514e", lw=0.8, ls=":")
    ax[1].text(0.5, 2.6, "2.5 m/s² limit", color="#52514e", fontsize=8)
    ax[1].set_ylim(0, 6)
    ax[2].axhline(25, color="#52514e", lw=0.8, ls=":")
    ax[2].text(0.5, 23.2, "25° tilt clamp", color="#52514e", fontsize=8)
    for a_, lab in zip(ax, ("horizontal speed (m/s)", "horizontal accel (m/s²)\n(before: clipped at 12)",
                            "tilt (deg)", "altitude (m)")):
        a_.set_ylabel(lab)
        a_.grid(alpha=0.25, lw=0.5)
        for s in ("top", "right"):
            a_.spines[s].set_visible(False)
    ax[3].set_xlabel("mission time (s), sampled at 60 fps")
    ax[0].legend(loc="upper right", ncol=4, frameon=False, fontsize=8)
    fig.suptitle(title, fontsize=11, x=0.01, ha="left")
    fig.tight_layout()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=120)
    plt.close(fig)


def fmt(agg):
    return (f"max speed {agg['max_speed_mps']:.2f} m/s | max accel h {agg['max_acc_h']:.2f} v {agg['max_acc_v']:.2f} m/s² | "
            f"max jerk {agg['max_jerk']:.1f} m/s³ | velocity jumps {agg['vel_jumps']} | max yaw rate {agg['max_yaw_rate_dps']:.0f} °/s")


# --------------------------------------------------------------------------- modes
def cmd_showcase(a):
    grid, pads, piles = seed7_site()
    on_step, notes = showcase_script(grid, pads, piles)
    fl = SmoothFlight(grid, starts=pads, platform="heavy_sar", seed=a.seed)
    t0 = time.time()
    run_script(fl, on_step, a.t_max)
    on_old, _ = showcase_script(grid, pads, piles)
    old = KinematicFlight(grid, starts=pads, platform="heavy_sar")
    old_steps = run_script(old, on_old, a.t_max)
    after = smooth_frames(fl, 0.0, a.t_max)
    before = lerp_frames(old_steps)
    ag_b, _ = metrics(before)
    ag_a, _ = metrics(after)
    nm = nominal_metrics(fl, 0.0, a.t_max)
    print(f"showcase {a.t_max:.0f} s (built in {time.time() - t0:.1f} s)")
    print("  before:", fmt(ag_b))
    print("  after :", fmt(ag_a))
    print("  planned path:", {k: (round(v, 3) if isinstance(v, float) else v) for k, v in nm.items()})
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    log = pose_log(fl, grid, piles, notes, 0.0, a.t_max)
    log["metrics"] = {"before": ag_b, "after": ag_a, "planned": nm}
    with open(a.out, "w") as f:
        json.dump(log, f, separators=(",", ":"))
    print(f"  wrote {a.out} ({os.path.getsize(a.out) / 1e6:.1f} MB)")
    if a.plot:
        plot_profiles(after, before, a.plot, "Heavy SAR drone motion, seed-7 showcase: smooth model (colour) vs the old "
                                              "straight-line steps as rendered (grey dashed, d1)")
        print(f"  wrote {a.plot}")
    if a.metrics_json:
        with open(a.metrics_json, "w") as f:
            json.dump({"before": ag_b, "after": ag_a, "planned": nm}, f, indent=1)


def cmd_mission(a):
    import sim.grid_mission as gm
    res = {}
    for name in ("before", "after"):
        orig = gm.KinematicFlight
        if name == "after":
            gm.KinematicFlight = lambda grid, **kw: SmoothFlight(grid, seed=a.seed, **kw)
        try:
            out = os.path.join(a.scratch, name)
            os.makedirs(out, exist_ok=True)
            m = gm.GridMission(a.seed, None, "dsp", out, False, False, acoustic="stub", tasking=os.path.join(out, "tasking.json"))
            steps = [(m.t, {p["id"]: list(p["pose"]) for p in m.flight.poses()})]
            t0 = time.time()
            first_scan = None
            while not m.done and m.t < a.max_t:
                m.step()
                steps.append((m.t, {d: [float(v) for v in m.flight.drones[d].pos] + [float(m.flight.drones[d].yaw)]
                                    for d in m.flight.drones}))
                if first_scan is None and any(p["state"] == "scanning" for p in m.flight.poses()):
                    first_scan = m.t
            wall = time.time() - t0
        finally:
            gm.KinematicFlight = orig
        s = m.summary()
        story = {"t_end": m.t, "first_scanning_t": first_scan, "visited": s.get("visited"),
                 "pings": [c["id"] for c in s.get("pings", []) if c.get("status") == "candidate"], "wall_s": round(wall, 1)}
        if name == "before":
            fr = lerp_frames([(t, p, None, None) for t, p in steps])
        else:
            fr = smooth_frames(m.flight, 0.0, m.t)
            res["planned"] = nominal_metrics(m.flight, 0.0, m.t)
            if a.out:
                piles = dict(m.world.pile_xy)
                log = pose_log(m.flight, m.grid, piles, {}, 0.0, min(m.t, a.log_s), fps=30, title=f"seed {a.seed} mission")
                with open(a.out, "w") as f:
                    json.dump(log, f, separators=(",", ":"))
                print(f"  wrote {a.out}")
            if a.motion_json:
                m.flight.save_motion(a.motion_json)
        agg, _ = metrics(fr)
        res[name] = {"metrics": agg, "story": story}
        print(f"{name}: {fmt(agg)}\n        story {story}", flush=True)
    if "planned" in res:
        print("planned path (after):", res["planned"])
    if a.metrics_json:
        with open(a.metrics_json, "w") as f:
            json.dump(res, f, indent=1, default=float)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)
    s = sub.add_parser("showcase")
    s.add_argument("--seed", type=int, default=7)
    s.add_argument("--t-max", type=float, default=64.0)
    s.add_argument("--out", default=os.path.join(ROOT, "media/motion/showcase.json"))
    s.add_argument("--plot", default=os.path.join(ROOT, "docs/img/motion_profiles.png"))
    s.add_argument("--metrics-json", default=None)
    m = sub.add_parser("mission")
    m.add_argument("--seed", type=int, default=7)
    m.add_argument("--max-t", type=float, default=600.0)
    m.add_argument("--scratch", default=os.path.join(ROOT, "media/motion/mission"))
    m.add_argument("--out", default=None, help="pose log JSON (30 fps, first --log-s seconds)")
    m.add_argument("--log-s", type=float, default=240.0)
    m.add_argument("--motion-json", default=None, help="SmoothFlight.save_motion() output (Isaac --replay input)")
    m.add_argument("--metrics-json", default=None)
    a = ap.parse_args(argv)
    (cmd_showcase if a.mode == "showcase" else cmd_mission)(a)


if __name__ == "__main__":
    main()
