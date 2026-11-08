"""sim/flight_motion.py + isaac/motion.py: continuity, limits, landing, clearance, determinism, timing."""
import math
import os

import numpy as np
import pytest

from sim.flight_motion import (Disturbance, MotionSpec, SmoothFlight, Trajectory, check_clearance, check_limits,
                               flight_time, load_motion, plan)
from sim.world import Grid

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SPEC = MotionSpec.from_platform("heavy_sar")
FS = 1000.0                     # continuity checks sample at 1 kHz


def flat_grid(z=0.0, n=400, res=0.5, origin=(-100.0, -100.0)):
    return Grid(origin, res, n, n, np.full((n, n), z, np.float32))


def paths():
    """A representative set: takeoff+cruise+perch landing, pod hover, a long leg, a short hop."""
    return {
        "land": ([(0, 0, 0.15), (0, 0, 14), (80, 30, 14), (80, 30, 2.15)], dict(kind="land", from_ground=True)),
        "pod": ([(0, 0, 14), (40, -20, 14), (40, -20, 11)], dict(kind="pod")),
        "long": ([(0, 0, 12), (180, 0, 12)], dict(kind="hover")),
        "hop": ([(0, 0, 12), (6, 3, 12)], dict(kind="hover")),
        "takeoff": ([(0, 0, 0.15), (0, 0, 12)], dict(kind="takeoff", from_ground=True)),
    }


def replans():
    """Mid-flight re-plans from the state of a cruising drone: same way, sideways, reverse, climb-first route."""
    base = plan([(0, 0, 12), (200, 0, 12)], 0.0, SPEC)
    t = 12.0
    p, v, a, _ = (x[0] for x in base.nominal(t))
    out = {}
    for name, route in {"ahead": [(150, 40, 12)], "side": [(p[0], 80, 12)], "reverse": [(-50, 0, 12)],
                        "climb": [(p[0], p[1], 20), (160, -30, 20)]}.items():
        out[name] = (base, plan([p] + [np.array(w, float) for w in route], t, SPEC, v0=v, a0=a), t)
    return out


def finite_diffs(tr, t0, t1):
    ts = np.arange(t0, t1, 1.0 / FS)
    p, v, a, _ = tr.nominal(ts) if isinstance(tr, Trajectory) else tr(ts)
    return p, v, a


def assert_c2(p, v, a, tag):
    dv = np.linalg.norm(np.diff(v, axis=0), axis=1) * FS      # implied |accel|
    da = np.linalg.norm(np.diff(a, axis=0), axis=1) * FS      # implied |jerk|
    dp = np.linalg.norm(np.diff(p, axis=0), axis=1) * FS      # implied speed
    assert dp.max() < 11.5, (tag, dp.max())
    # with C0 velocity and acceleration these stay at the limits; a jump of 0.01 m/s would read as 10 m/s^2
    assert dv.max() < 1.05 * math.hypot(SPEC.a_h, SPEC.a_v), (tag, dv.max())
    assert da.max() < 1.05 * math.hypot(SPEC.j_h, SPEC.j_v), (tag, da.max())


# --------------------------------------------------------------------------- continuity (C2)
@pytest.mark.parametrize("name", list(paths()))
def test_c2_inside_and_at_joins(name):
    path, kw = paths()[name]
    tr = plan(path, 0.0, SPEC, **kw)
    assert_c2(*finite_diffs(tr, -0.5, tr.t_end + 1.0), name)
    for tb in tr.breaks():                                    # explicit join check: both sides agree
        pa, va, aa, _ = (x[0] for x in tr.nominal(tb - 1e-6))
        pb, vb, ab, _ = (x[0] for x in tr.nominal(tb + 1e-6))
        assert np.abs(pb - pa).max() < 1e-4 and np.abs(vb - va).max() < 1e-4 and np.abs(ab - aa).max() < 1e-4, (name, tb)


@pytest.mark.parametrize("name", list(replans()))
def test_c2_across_midflight_replan(name):
    base, new, t = replans()[name]

    def joined(ts):
        p0, v0, a0, j0 = base.nominal(ts)
        p1, v1, a1, j1 = new.nominal(ts)
        m = (ts >= t)[:, None]
        return np.where(m, p1, p0), np.where(m, v1, v0), np.where(m, a1, a0), None

    assert_c2(*finite_diffs(joined, t - 3.0, new.t_end + 0.5), name)
    pa, va, aa, _ = (x[0] for x in base.nominal(t))
    pb, vb, ab, _ = (x[0] for x in new.nominal(t))
    assert np.allclose(pa, pb) and np.allclose(va, vb) and np.allclose(aa, ab)


def test_same_direction_replan_keeps_cruising():
    base, new, t = replans()["ahead"]
    ts = np.arange(t, t + 6.0, 0.1)
    _, v, _, _ = new.nominal(ts)
    assert np.hypot(v[:, 0], v[:, 1]).min() > 8.5        # no stop-and-go when the new goal is roughly ahead


# --------------------------------------------------------------------------- limits
@pytest.mark.parametrize("name", list(paths()))
def test_limits_accel_jerk_tilt_yaw(name):
    path, kw = paths()[name]
    tr = plan(path, 0.0, SPEC, **kw)
    ok, m = check_limits(tr, SPEC, dt=0.01)
    assert ok, m
    dist = Disturbance(7, "d1", SPEC)
    tilt, yr = 0.0, 0.0
    for t in np.arange(0.0, tr.t_end, 0.05):
        s = tr.sample(t, dist=dist)
        tilt, yr = max(tilt, s.tilt), max(yr, abs(s.yaw_rate))
    assert tilt <= SPEC.tilt_max_deg + 1e-6
    assert yr <= SPEC.yaw_rate_dps + 0.5


@pytest.mark.parametrize("name", list(replans()))
def test_limits_after_replan(name):
    _, new, _ = replans()[name]
    ok, m = check_limits(new, SPEC, dt=0.01)
    assert ok, (name, m)


def test_leans_into_acceleration_and_flares_when_braking():
    tr = plan([(0, 0, 12), (100, 0, 12)], 0.0, SPEC)
    acc = tr.sample(2.0)          # accelerating along +x: nose down (+pitch), thrust tilted forward
    brk = tr.sample(tr.t_end - 2.5)
    cru = tr.sample(tr.t_end / 2)
    assert acc.pitch > 5.0 and brk.pitch < -2.0
    assert 6.0 < cru.pitch < 10.0                     # steady nose-down against drag at cruise
    tz = plan([(0, 0, 12), (0, 0, 12.01)], 0.0, SPEC)
    assert abs(tz.sample(0.0).tilt) < 1.0


def test_yaw_turns_before_the_corner():
    tr = plan([(0, 0, 12), (60, 0, 12), (60, 0, 13)], 0.0, SPEC, yaw0=90.0)
    y0, _ = tr.yaw_at(0.0)
    y1, _ = tr.yaw_at(3.0)
    assert abs(y0 - 90.0) < 1e-6 and abs(y1) < 45.0   # rotates towards the leg heading while it accelerates


# --------------------------------------------------------------------------- phases
def touchdown_speed(tr, h=0.05):
    zt = tr.nominal(tr.touchdown)[0][0][2]
    ts = np.linspace(tr.touchdown - 30.0, tr.touchdown, 6001)
    p, v, _, _ = tr.nominal(ts)
    return abs(float(v[int(np.argmax(p[:, 2] <= zt + h)), 2]))


def test_touchdown_soft_and_phases_in_order():
    path, kw = paths()["land"]
    tr = plan(path, 0.0, SPEC, **kw)
    assert touchdown_speed(tr) <= 0.5
    assert touchdown_speed(tr) <= SPEC.flare_mps + 1e-6
    names = [nm for _, _, nm in sorted(tr.phases)]
    order = ["spool-up", "liftoff", "hover-2m", "climb", "cruise", "stabilise", "descent", "final", "flare", "spin-down"]
    pos = [names.index(n) for n in order]
    assert pos == sorted(pos), names
    # rotors: spool from 0 on the pad, hover in flight, spin down to 0 after touchdown
    assert tr.sample(0.0).rotor_rpm.max() == 0.0
    assert abs(tr.sample(10.0).rotor_rpm.mean() - SPEC.hover_rpm) < 0.15 * SPEC.hover_rpm
    assert tr.sample(tr.touchdown + SPEC.spin_down_s + 0.1).rotor_rpm.max() == 0.0
    assert not tr.sample(tr.touchdown + 0.5).airborne


def test_climbing_needs_more_rpm_than_hover():
    tr = plan([(0, 0, 12), (0, 0, 30)], 0.0, SPEC)
    up = tr.sample(1.5).rotor_rpm.mean()
    assert up > SPEC.hover_rpm * 1.02


def test_winch_s_curve_and_pod_settles():
    g = flat_grid(0.0)
    fl = SmoothFlight(g, starts={"d1": (0.0, 0.0)}, platform="heavy_sar", seed=7)
    fl.drones["d1"].state = "hovering"
    fl.timelines["d1"].pieces[0] = Trajectory.hold(-1e9, np.array([0.0, 0.0, 10.15]), SPEC)
    fl.drones["d1"].pos = np.array([0.0, 0.0, 10.15])
    fl.command({"drone_id": "d1", "type": "goto_pod_scan", "destination": [5.0, 0.0, 0.0], "hover_z": 8.0,
                "scan_duration": 5.0, "landing_site_id": "x"})
    t, phases, done = 0.0, [], None
    while t < 80 and done is None:
        t += 1.0
        ev = fl.step(1.0, t)
        phases.append(fl.drones["d1"].job and fl.drones["d1"].job.get("phase"))
        if any(e["type"] == "scan_complete" for e in ev):
            done = t
    assert done is not None and "lowering" in phases and "scanning" in phases
    w = fl.timelines["d1"].winch
    lower = [e for e in w.events if e["kind"] == "lower"][0]
    ts = np.linspace(lower["t"], lower["t"] + lower["T"], 200)
    L = np.array([w.length(x) for x in ts])
    assert np.all(np.diff(L) >= -1e-9) and abs(L[-1] - 8.0) < 1e-6
    assert abs(w.speed(lower["t"] + 1e-3)) < 0.05                # S-curve: starts at zero cable speed
    s_end = fl.motion_sample("d1", lower["t"] + lower["T"] + 0.2)
    assert np.allclose(s_end.pod, [5.0, 0.0, 0.0], atol=1e-6)   # resting on its placement point
    mid = fl.motion_sample("d1", lower["t"] + 0.5 * lower["T"])
    assert mid.pod is not None and mid.pod[2] > 0.5              # still hanging


# --------------------------------------------------------------------------- clearance
def seed7():
    from sim.grid_world import GridWorld
    from sim.scenario import make_scenario
    sc = make_scenario(7)
    w = GridWorld(sc)
    pads = {f"d{i + 1}": tuple(xy) for i, xy in enumerate(sc.public_dict()["drone_pads"][:3])}
    return w.grid, pads, w.pile_xy


def _fly_all(fl, dests, typ):
    out = []
    for did, dest in dests:
        d = fl.drones[did]
        route = fl.plan(d, np.array(dest, float))
        assert route is not None
        kind = "land" if typ == "land" else "pod"
        tr = plan([d.pos.copy()] + route, 0.0, fl.mspec, kind=kind, from_ground=True, grid=fl.grid, max_alt=fl.max_alt)
        out.append(tr)
    return out


def test_no_heightmap_collision_seed7_grid_paths():
    g, pads, piles = seed7()
    fl = SmoothFlight(g, starts=pads, platform="heavy_sar", seed=7)
    pad = np.array(pads["d1"])
    for name, (x, y) in piles.items():
        c = np.array([x, y])
        u = (pad - c) / np.linalg.norm(pad - c)
        lx, ly = c + 12.0 * u                                   # a perch at the pile edge, facing the pads
        z = float(g.height_at(x, y))
        for tr in _fly_all(fl, [("d1", (lx, ly, float(g.height_at(lx, ly)) + 0.15)), ("d2", (x, y, z + 9.0))],
                           "land" if True else "pod"):
            ok, margin, tw = check_clearance(tr, g, fl.mspec)
            assert ok, (name, margin, tw)
        trp = _fly_all(fl, [("d3", (x, y, z + 9.0))], "pod")[0]
        assert check_clearance(trp, g, fl.mspec)[0], name


def test_no_heightmap_collision_collapse_site():
    hm = os.path.join(ROOT, "assets/maps/collapse_site_heightmap.npz")
    if not os.path.exists(hm):
        pytest.skip("collapse heightmap not present")
    d = np.load(hm, allow_pickle=True)
    z = d["z"].astype(np.float32)
    g = Grid(tuple(float(v) for v in d["origin"]), float(d["res"]), z.shape[1], z.shape[0], z)
    pads = {"d1": (-58.6, -150.0), "d2": (-56.0, -150.0), "d3": (-53.4, -150.0)}
    fl = SmoothFlight(g, starts=pads, platform="heavy_sar", seed=7)
    targets = [(-31.06, 14.28), (14.5, 13.25), (14.5, -2.25), (-31.0, -16.5)]   # T1a, T2a, T2b, T1c
    for x, y in targets:
        zt = float(g.height_at(x, y))
        tr = _fly_all(fl, [("d1", (x, y, zt + 9.0))], "pod")[0]
        ok, margin, tw = check_clearance(tr, g, fl.mspec)
        assert ok, ((x, y), margin, tw)
        # and back from there to the pad (a long leg across standing buildings), landing
        p_end = tr.p_end
        fl.drones["d2"].pos = p_end.copy()
        route = fl.plan(fl.drones["d2"], np.array([pads["d2"][0], pads["d2"][1], float(g.height_at(*pads["d2"])) + 0.15]))
        back = plan([p_end] + route, 0.0, fl.mspec, kind="land", grid=g, max_alt=fl.max_alt)
        assert check_clearance(back, g, fl.mspec)[0]
        fl.drones["d2"].pos = np.array([pads["d2"][0], pads["d2"][1], float(g.height_at(*pads["d2"])) + 0.15])


def test_corner_unblends_when_the_rounded_corner_would_clip():
    """Pod approach: the descent to the hover height blends into the deceleration (rounded corner).  A mound under
    that rounded corner forces the stop-then-descend geometry the coordinator checked."""
    path = [np.array([0, 0, 15.0]), np.array([60, 0, 15.0]), np.array([60, 0, 5.0])]
    blended = plan(path, 0.0, SPEC, kind="pod")
    ts = np.arange(0, blended.t_end, 0.05)
    p = blended.nominal(ts)[0]
    k = int(np.argmin(np.abs(p[:, 0] - 52.0)))
    assert p[k, 2] < 14.0                                        # the corner really is rounded
    g = flat_grid(0.0)
    zz = g.z.copy()
    ix, iy = g.index(np.array([47.0, 55.0]), np.array([0.0, 0.0]))
    zz[iy[0] - 6:iy[0] + 6, ix[0]:ix[1]] = float(p[k, 2]) - 1.0  # a mound right under the rounded corner
    g.z = zz
    checked = plan(path, 0.0, SPEC, kind="pod", grid=g)
    assert not check_clearance(blended, g, SPEC)[0]
    assert check_clearance(checked, g, SPEC)[0]
    assert checked.info["beta"] == 0.0 and checked.duration > blended.duration


# --------------------------------------------------------------------------- determinism + timing
def _scripted(seed):
    g = flat_grid(0.0)
    fl = SmoothFlight(g, starts={"d1": (0.0, 0.0), "d2": (4.0, 0.0)}, platform="heavy_sar", seed=seed)
    for d in fl.drones:
        fl.command({"drone_id": d, "type": "takeoff"})
    t = 0.0
    sent = False
    while t < 40.0:
        t += 1.0
        fl.step(1.0, t)
        if not sent and fl.drones["d1"].state == "hovering":
            fl.command({"drone_id": "d1", "type": "goto", "destination": [40.0, 10.0, 14.0]})
            fl.command({"drone_id": "d2", "type": "goto_land_scan", "destination": [-30.0, 5.0, 0.0], "scan_duration": 5}),
            sent = True
    return fl


def test_deterministic_by_seed():
    a, b, c = _scripted(7), _scripted(7), _scripted(8)
    for t in (3.3, 17.77, 31.0):
        for d in ("d1", "d2"):
            sa, sb, sc = a.motion_sample(d, t), b.motion_sample(d, t), c.motion_sample(d, t)
            assert np.array_equal(sa.pos, sb.pos) and np.array_equal(sa.quat, sb.quat)
            assert np.array_equal(sa.rotor_rpm, sb.rotor_rpm)
    s7, s8 = a.motion_sample("d1", 31.0), c.motion_sample("d1", 31.0)
    assert not np.allclose(s7.pos, s8.pos)                         # hover drift differs by seed
    assert np.allclose(a.motion_sample("d1", 31.0, nominal=True).pos, c.motion_sample("d1", 31.0, nominal=True).pos)


def test_arrival_times_match_flight_time_and_eta():
    g = flat_grid(0.0)
    fl = SmoothFlight(g, starts={"d1": (0.0, 0.0)}, platform="heavy_sar", seed=7)
    dest = np.array([70.0, -20.0, 0.15])
    eta = fl.eta("d1", dest)
    fl.command({"drone_id": "d1", "type": "goto_land_scan", "destination": [70.0, -20.0, 0.0], "scan_duration": 5.0})
    tr = fl.drones["d1"].traj
    route = fl.drones["d1"].route
    ft = flight_time([np.array([0.0, 0.0, 0.15])] + route, SPEC, kind="land", from_ground=True, grid=g, max_alt=fl.max_alt)
    assert abs(tr.duration - ft) < 1e-9 and abs(eta - ft) < 1e-9
    t, arrived = 0.0, None
    while arrived is None and t < 200:
        t += 1.0
        fl.step(1.0, t)
        if fl.drones["d1"].state == "scanning":
            arrived = t
    assert arrived == math.ceil(ft)                                # first mission step at or after the arrival
    assert abs(fl.drones["d1"].dwell_left - (5.0 - (arrived - ft))) < 1e-9   # dwell starts at the true arrival


def test_motion_export_roundtrip(tmp_path):
    fl = _scripted(7)
    p = tmp_path / "motion.json"
    fl.save_motion(str(p))
    tls = load_motion(str(p))
    for t in (5.0, 22.5, 39.0):
        for d in ("d1", "d2"):
            a, b = fl.motion_sample(d, t), tls[d].sample(t)
            assert np.allclose(a.pos, b.pos) and np.allclose(a.quat, b.quat) and np.allclose(a.rotor_rpm, b.rotor_rpm)


def test_kinematic_baseline_has_discontinuities():
    """The old model, rendered the old way, is what the user saw: velocity jumps at the step joins."""
    from sim.coordinator import KinematicFlight
    g = flat_grid(0.0)
    kf = KinematicFlight(g, starts={"d1": (0.0, 0.0)}, platform="heavy_sar")
    kf.command({"drone_id": "d1", "type": "goto_land_scan", "destination": [60.0, 0.0, 0.0], "scan_duration": 5.0})
    ts, P = [0.0], [kf.drones["d1"].pos.copy()]
    for k in range(1, 40):
        kf.step(1.0, float(k))
        ts.append(float(k))
        P.append(kf.drones["d1"].pos.copy())
    P = np.array(P)
    tf = np.arange(0, 39, 1 / 60)
    p = np.column_stack([np.interp(tf, ts, P[:, i]) for i in range(3)])
    v = np.diff(p, axis=0) * 60
    jumps = int((np.linalg.norm(np.diff(v, axis=0), axis=1) > 5.0 / 60).sum())
    assert jumps >= 3


# --------------------------------------------------------------------------- isaac/motion.py (no omni)
def test_isaac_motion_view_and_camera_helpers():
    from isaac.drone_viz import NullDrone
    from isaac.motion import MotionView, OneEuro, SpringFollow
    fl = _scripted(7)
    view = MotionView.attach(fl)
    prev = {"t": 20.0, "drones": [{"id": "d1", "state": "flying"}, {"id": "d2", "state": "flying"}]}
    cur = {"t": 21.0, "drones": [{"id": "d1", "state": "flying"}, {"id": "d2", "state": "flying"}]}
    a = view.interp_drones(prev, cur, 0.25)
    assert set(a) == {"d1", "d2"} and len(a["d1"]) == 7
    s = fl.motion_sample("d1", 20.25)
    assert np.allclose(a["d1"][:3], s.pos)
    nd = NullDrone()
    out = view.apply(nd, "d1", 20.25, 1 / 30)
    assert nd.pose is not None and np.allclose(nd.pose[:3], out.pos)
    sp = SpringFollow([0.0, 0.0, 0.0], halflife=0.5)
    xs = [sp.update([10.0, 0.0, 0.0], 1 / 60)[0] for _ in range(600)]
    assert max(xs) <= 10.0 + 1e-9 and abs(xs[-1] - 10.0) < 1e-3     # critically damped: no overshoot
    sp2 = SpringFollow([0.0, 0.0, 0.0], halflife=0.5)
    for _ in range(30):
        sp2.update([10.0, 0, 0], 1 / 30)
    assert abs(sp2.x[0] - xs[59]) < 1e-6                            # frame-rate independent
    f = OneEuro()
    ys = [f.update([math.sin(i / 10) + (0.05 if i % 2 else -0.05)], 1 / 60)[0] for i in range(300)]
    assert np.std(np.diff(ys)) < np.std(np.diff([math.sin(i / 10) + (0.05 if i % 2 else -0.05) for i in range(300)]))


def test_isaac_motion_adapter_for_posetrack():
    """isaac/run_mission.py PoseTrack calls isaac.motion.Motion(), .ingest(state), .pose_at(did, t) -> dict."""
    import isaac.motion as M
    fl = _scripted(7)                                 # newest SmoothFlight: Motion() finds it by itself
    mo = M.Motion()
    mo.ingest({"t": 20.0, "drones": [{"id": "d1", "state": "flying"}, {"id": "d2", "state": "flying"}]})
    p = mo.pose_at("d1", 20.5)
    for k in ("xyz", "quat", "rotor_rpm", "winch_len", "tilt", "vel", "yaw"):
        assert k in p
    assert np.allclose(p["xyz"], fl.motion_sample("d1", 20.5).pos) and len(p["rotor_rpm"]) == 8
    with pytest.raises(KeyError):
        mo.pose_at("d9", 1.0)
