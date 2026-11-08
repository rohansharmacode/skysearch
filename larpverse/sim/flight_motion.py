"""Multirotor flight motion for the heavy SAR drone: smooth, C2-continuous, limit-respecting trajectories.

Replaces the straight-line, instant-velocity legs of ``sim.coordinator.KinematicFlight`` with how a
FlyCart 30 / Agras T50-class coaxial X8 (~77 kg, 3.1 m span) actually moves.  docs/MOTION.md has the model,
the parameters with sources and the integration requests.

    from sim.flight_motion import plan, flight_time, MotionSpec, SmoothFlight
    spec = MotionSpec.from_platform("heavy_sar")
    tr = plan([p0, (x0, y0, 14), (x1, y1, 14), (x1, y1, z1)], t0=0.0, spec=spec, kind="land", from_ground=True)
    s = tr.sample(12.34)          # s.pos, s.vel, s.acc, s.quat (w, x, y, z), s.rotor_rpm (8,), s.winch_len
    flight_time(path, spec, kind="land", from_ground=True) == tr.duration      # by construction

    flight = SmoothFlight(grid, starts=pads, platform="heavy_sar", seed=7)     # drop-in KinematicFlight
    flight.motion_sample("d1", t_render)                                        # any render rate

The model
---------
* **Primitives.**  Every motion is a superposition of rest-to-rest 1-D moves along straight legs, each with a
  jerk-limited S-curve velocity profile (quintic "smootherstep" velocity ramps: acceleration and jerk are
  continuous inside a ramp, acceleration is zero at both ends), plus one *brake* term that takes over the
  velocity and acceleration the drone already has when it is re-commanded mid-flight.  A sum of C2 terms is C2,
  so position, velocity and acceleration are continuous everywhere, including across re-plans.
* **Corners.**  The next leg starts while the previous one is still decelerating (overlap ``beta``), which
  rounds the corner with a radius set by speed and acceleration, as industrial blending does.  The rounded
  path is checked against the heightmap (rotor-disc clearance); on a conflict the corner falls back to the
  stop-and-turn geometry the coordinator already checked, then the cruise altitude is raised.
* **Phases.**  Takeoff = rotor spool-up on the ground, lift-off to a 2 m hover, short hover, climb.  Landing =
  decelerate into a stabilised hover, descend, slow final descent at 0.5 m/s, flare to 0.2 m/s, touchdown,
  rotor spin-down.  Winch = S-curve cable payout with a damped pendulum sway that has died out when the pod
  touches the rubble.
* **Attitude** from the dynamics (differential flatness): thrust direction = commanded acceleration + air drag
  + gravity, tilt clamped to 25 deg, so the drone leans into acceleration, holds a nose-down cruise attitude
  against drag, flares when braking and leans into a gust.  Yaw tracks the velocity heading 1.5 s ahead
  (it turns before the corner) through a rate- and acceleration-limited filter (<= 45 deg/s).
* **Hover realism.**  Deterministic coloured drift/bob (sum of seeded sinusoids) and site-wide wind gusts with
  a position response, both faded out near the ground.  Rendering only: the mission sees the nominal path.
* **Rotors.**  Per-rotor RPM from thrust (hover RPM x sqrt(thrust / weight)) plus small yaw / pitch-roll
  differentials, spool-up and spin-down on the ground.

Pure numpy; importable in Isaac's python.  No ground truth is read here.
"""
from __future__ import annotations

import bisect
import math
import threading
import weakref
import zlib
from dataclasses import asdict, dataclass

import numpy as np

from .coordinator import LAND_HEIGHT, SCAN_DWELL_S, STARTS, KinematicFlight

G = 9.81
RHO = 1.225

# --------------------------------------------------------------------------- smootherstep algebra
# S(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5 : velocity ramp shape.  S' max 1.875, S'' max 10/sqrt(3).
S1_MAX, S2_MAX = 1.875, 10.0 / math.sqrt(3.0)


def _S(u):
    return u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)


def _S1(u):
    return 30.0 * u * u * (1.0 - u) ** 2


def _S2(u):
    return 60.0 * u * (1.0 - u) * (1.0 - 2.0 * u)


def _IS(u):                      # integral of S from 0 to u
    return u ** 4 * (2.5 - 3.0 * u + u * u)


def ramp_time(dv, a_max, j_max):
    """Duration of a smootherstep velocity ramp of |dv| that respects |a| <= a_max and |jerk| <= j_max."""
    dv = abs(float(dv))
    if dv < 1e-9:
        return 0.0
    return max(S1_MAX * dv / a_max, math.sqrt(S2_MAX * dv / j_max))


def smooth01(x):
    """Smootherstep on a clipped argument (C2), for blends and fades."""
    u = np.clip(x, 0.0, 1.0)
    return _S(u)


# --------------------------------------------------------------------------- spec
@dataclass(frozen=True)
class MotionSpec:
    """Motion limits for one airframe.  Defaults: heavy_sar (docs/MOTION.md has the sources)."""
    name: str = "heavy_sar"
    cruise_mps: float = 10.0          # platform cruise (FlyCart 30 spec max 20 m/s)
    climb_mps: float = 3.0            # platform climb rate
    descend_mps: float = 2.0          # platform descent rate
    a_h: float = 2.5                  # horizontal acceleration limit, m/s^2
    j_h: float = 2.0                  # horizontal jerk limit, m/s^3
    a_v: float = 1.5                  # vertical acceleration limit
    j_v: float = 2.0                  # vertical jerk limit
    tilt_max_deg: float = 25.0
    yaw_rate_dps: float = 45.0
    yaw_acc_dps2: float = 45.0
    yaw_gain: float = 1.2             # 1/s, heading-tracking gain
    yaw_lead_s: float = 1.5           # yaw looks this far ahead along the path (turns into the corner)
    drag_tilt_deg: float = 8.0        # steady nose-down pitch at cruise (air drag)
    mass_kg: float = 77.0
    n_rotors: int = 8
    hover_rpm: float = 1050.0         # estimate (momentum theory, C_T ~0.08, 54-in props)
    spool_s: float = 2.5              # rotors 0 -> hover RPM on the ground
    spin_down_s: float = 3.0          # hover RPM -> 0 after touchdown
    liftoff_h: float = 2.0            # first hover height after lift-off
    liftoff_mps: float = 0.8
    liftoff_hover_s: float = 2.0
    land_stabilise_s: float = 1.5     # stabilised hover over the patch before the descent
    pod_stabilise_s: float = 1.5      # stabilised hover before the winch starts paying out
    final_h: float = 3.0              # slow final descent below this height
    final_mps: float = 0.5
    flare_h: float = 0.5
    flare_mps: float = 0.2
    span_m: float = 3.085
    skid_m: float = LAND_HEIGHT       # pose point height above the skids
    clear_m: float = 1.0              # min rotor-disc clearance over terrain away from the takeoff / landing columns
    launch_stagger_s: float = 1.5     # sequential launch from the pads (takeoff command only)
    winch_lower_s: float = 8.0
    winch_raise_s: float = 8.0
    sway_m: float = 0.18              # initial pod pendulum amplitude
    sway_zeta: float = 0.12
    drift_h_m: float = 0.07           # hover drift RMS (horizontal, per axis)
    drift_v_m: float = 0.035
    drift_yaw_deg: float = 0.35
    gust_gap_s: tuple = (16.0, 40.0)
    gust_dur_s: tuple = (2.5, 5.0)
    gust_mps: tuple = (2.0, 4.5)
    gust_disp_k: float = 0.022        # m of downwind displacement per (m/s)^2 of gust

    @property
    def drag_k(self):
        """Quadratic drag acceleration coefficient: a_drag = k |v_air| v_air (gives drag_tilt_deg at cruise)."""
        return math.tan(math.radians(self.drag_tilt_deg)) * G / self.cruise_mps ** 2

    @classmethod
    def from_platform(cls, platform=None, **over):
        from .platform import get_platform
        p = platform if hasattr(platform, "cruise_mps") else get_platform(platform)
        hov = _hover_rpm(p)
        pod = getattr(p, "pod", None)
        kw = dict(name=p.name, cruise_mps=p.cruise_mps, climb_mps=p.climb_mps, descend_mps=p.descend_mps,
                  mass_kg=p.takeoff_mass_kg, n_rotors=p.n_rotors, hover_rpm=hov, span_m=p.span_m,
                  winch_lower_s=getattr(pod, "lower_s", 8.0), winch_raise_s=getattr(pod, "raise_s", 8.0))
        if p.name == "legacy_small":
            kw.update(a_h=3.0, j_h=4.0, a_v=2.0, j_v=4.0, yaw_rate_dps=90.0, yaw_acc_dps2=120.0, spool_s=1.2,
                      spin_down_s=1.5, liftoff_h=1.0, liftoff_hover_s=0.8, final_h=1.5, clear_m=0.6)
        kw.update(over)
        return cls(**kw)

    def to_dict(self):
        return asdict(self)


def _hover_rpm(p, c_t=0.08):
    """Hover RPM estimate: T = C_T rho n^2 D^4 per rotor (coaxial pairs both carry ~1/n of the weight)."""
    t = p.takeoff_mass_kg * G / p.n_rotors
    n = math.sqrt(t / (c_t * RHO * p.prop_d_m ** 4))
    return round(60.0 * n, -1)


# --------------------------------------------------------------------------- 1-D profiles
class Profile1D:
    """Piecewise velocity profile made of smootherstep ramps (va -> vb over T; va == vb is a cruise).
    Starts and ends at rest.  ``eval`` is vectorised and returns (x, v, a, j)."""

    def __init__(self, segs=()):
        segs = [(float(T), float(va), float(vb)) for T, va, vb in segs if T > 1e-9]
        self.segs = segs
        self.T = np.array([s[0] for s in segs], float)
        self.va = np.array([s[1] for s in segs], float)
        self.vb = np.array([s[2] for s in segs], float)
        self.t0 = np.concatenate([[0.0], np.cumsum(self.T)[:-1]]) if segs else np.zeros(0)
        dist = 0.5 * (self.va + self.vb) * self.T
        self.x0 = np.concatenate([[0.0], np.cumsum(dist)[:-1]]) if segs else np.zeros(0)
        self.duration = float(self.T.sum()) if segs else 0.0
        self.length = float(dist.sum()) if segs else 0.0

    @property
    def t_decel(self):
        """Duration of the final ramp to rest (the part a following leg may overlap)."""
        return float(self.T[-1]) if len(self.T) else 0.0

    def breaks(self):
        return list(self.t0) + [self.duration]

    def eval(self, t):
        t = np.asarray(t, float)
        if not self.segs:
            z = np.zeros_like(t)
            return z, z, z, z
        tc = np.clip(t, 0.0, self.duration)
        k = np.clip(np.searchsorted(self.t0, tc, side="right") - 1, 0, len(self.T) - 1)
        T, va, vb = self.T[k], self.va[k], self.vb[k]
        u = np.clip((tc - self.t0[k]) / T, 0.0, 1.0)
        dv = vb - va
        x = self.x0[k] + va * T * u + dv * T * _IS(u)
        v = va + dv * _S(u)
        a = dv / T * _S1(u)
        j = dv / T ** 2 * _S2(u)
        out = (t < 0.0) | (t > self.duration)
        v, a, j = np.where(out, 0.0, v), np.where(out, 0.0, a), np.where(out, 0.0, j)
        x = np.where(t < 0.0, 0.0, np.where(t > self.duration, self.length, x))
        return x, v, a, j

    # ---- builders
    @classmethod
    def rest_to_rest(cls, L, vmax, a_max, j_max):
        """Shortest jerk-limited S-curve over distance L: ramp up, cruise, ramp down."""
        L = float(L)
        if L < 1e-6:
            return cls()

        def need(vp):                        # distance used by the two ramps at peak speed vp
            return vp * ramp_time(vp, a_max, j_max)

        vp = float(vmax)
        if need(vp) > L:
            lo, hi = 0.0, vp
            for _ in range(60):
                mid = 0.5 * (lo + hi)
                lo, hi = (mid, hi) if need(mid) <= L else (lo, mid)
            vp = lo
        Ta = ramp_time(vp, a_max, j_max)
        Tc = max(0.0, (L - vp * Ta) / vp) if vp > 0 else 0.0
        return cls([(Ta, 0.0, vp), (Tc, vp, vp), (Ta, vp, 0.0)])

    @classmethod
    def from_speed(cls, L, v_s, vmax, a_max, j_max):
        """S-curve over L that STARTS at speed v_s > 0 (a re-plan keeps the speed it has along the new leg):
        ramp v_s -> vp, cruise, ramp to rest.  Returns (profile, overshoot_m): when L is shorter than the
        stopping distance the profile stops ``overshoot_m`` past the goal and the caller adds a move back."""
        L, v_s = float(L), float(v_s)
        if v_s < 1e-3:
            return cls.rest_to_rest(L, vmax, a_max, j_max), 0.0
        T_stop = ramp_time(v_s, a_max, j_max)
        d_stop = 0.5 * v_s * T_stop
        if L <= d_stop:
            return cls([(T_stop, v_s, 0.0)]), d_stop - L

        def dist(vp):
            T1, T2 = ramp_time(vp - v_s, a_max, j_max), ramp_time(vp, a_max, j_max)
            return 0.5 * (v_s + vp) * T1 + 0.5 * vp * T2

        lo = min(v_s, vmax)
        if dist(lo) > L:                        # only when v_s > vmax: slow down straight to rest
            return cls([(T_stop, v_s, 0.0)]), max(0.0, d_stop - L)
        vp = float(vmax)
        if dist(vp) > L:
            a, b = lo, vp
            for _ in range(60):
                mid = 0.5 * (a + b)
                a, b = (mid, b) if dist(mid) <= L else (a, mid)
            vp = a
        T1, T2 = ramp_time(vp - v_s, a_max, j_max), ramp_time(vp, a_max, j_max)
        Tc = max(0.0, (L - dist(vp)) / vp)
        return cls([(T1, v_s, vp), (Tc, vp, vp), (T2, vp, 0.0)]), 0.0

    @classmethod
    def scheduled(cls, L, levels, a_max, j_max):
        """Rest-to-rest over L through speed levels [(v, until_remaining_m), ...] (decreasing remaining, last 0):
        the ramp from level k to level k+1 ends when ``until_remaining`` of level k is left.  Used for the
        landing descent: descend rate, then the slow final, then the flare."""
        L = float(L)
        lv = [(float(v), float(r)) for v, r in levels if r < L - 0.05 or r == 0.0]
        if not lv:
            return cls.rest_to_rest(L, levels[-1][0], a_max, j_max)

        def build(v0):
            vs = [v0] + [v for v, _ in lv[1:]]
            rem = [r for _, r in lv]
            segs, x = [], 0.0
            T = ramp_time(vs[0], a_max, j_max)
            segs.append((T, 0.0, vs[0]))
            x += 0.5 * vs[0] * T
            for k in range(len(vs)):
                nxt = vs[k + 1] if k + 1 < len(vs) else 0.0
                Tr = ramp_time(vs[k] - nxt, a_max, j_max)
                dr = 0.5 * (vs[k] + nxt) * Tr
                end = L - rem[k] if k + 1 < len(vs) else L
                cruise = end - dr - x
                if cruise < -1e-9:
                    return None, cruise
                segs.append((cruise / vs[k], vs[k], vs[k]))
                segs.append((Tr, vs[k], nxt))
                x = end
            return segs, 0.0

        segs, _ = build(lv[0][0])
        if segs is None:                    # top speed too high for this drop: bisect it down
            lo, hi = (lv[1][0] if len(lv) > 1 else 0.05), lv[0][0]
            for _ in range(60):
                mid = 0.5 * (lo + hi)
                s, _ = build(mid)
                lo, hi = (mid, hi) if s is not None else (lo, mid)
            segs, _ = build(lo)
        if segs is None:
            return cls.rest_to_rest(L, lv[-1][0], a_max, j_max)
        p = cls(segs)
        return p if abs(p.length - L) < 1e-6 else cls.rest_to_rest(L, lv[-1][0], a_max, j_max)

    def to_list(self):
        return [list(s) for s in self.segs]


class Brake:
    """Takes an initial velocity v0 to rest over Tv (smootherstep) and fades the initial acceleration a0 to zero
    over its own, shorter Ta (quintic Hermite, |H1''| <= 3.94), so a re-plan inherits (v0, a0) exactly (C2) without
    a long acceleration tail.  Displacement at the end: Tv v0 / 2 + a0 Ta^2 / 10."""
    H1DD_MAX = 3.94

    def __init__(self, T, v0, a0, Ta=None):
        self.Tv, self.v0, self.a0 = float(T), np.asarray(v0, float), np.asarray(a0, float)
        self.Ta = float(Ta) if Ta is not None else float(T)
        self.T = max(self.Tv, self.Ta)

    @property
    def end(self):
        return 0.5 * self.Tv * self.v0 + 0.1 * self.a0 * self.Ta ** 2

    def eval(self, t):
        t = np.asarray(t, float)[..., None]
        Tv, Ta, v0, a0 = self.Tv, self.Ta, self.v0, self.a0
        u = np.clip(t / Tv, 0.0, 1.0)
        w = np.clip(t / Ta, 0.0, 1.0)
        H1 = w - 6 * w ** 3 + 8 * w ** 4 - 3 * w ** 5
        H1d = 1 - 18 * w ** 2 + 32 * w ** 3 - 15 * w ** 4
        H1dd = -36 * w + 96 * w ** 2 - 60 * w ** 3
        IH1 = 0.5 * w ** 2 - 1.5 * w ** 4 + 1.6 * w ** 5 - 0.5 * w ** 6
        x = Tv * v0 * (u - _IS(u)) + a0 * Ta ** 2 * IH1
        v = v0 * (1.0 - _S(u)) + a0 * Ta * H1
        a = -v0 / Tv * _S1(u) + a0 * H1d
        j = -v0 / Tv ** 2 * _S2(u) + a0 / Ta * H1dd
        return x, v, a, j

    @classmethod
    def for_state(cls, v0, a0, spec, scale=1.0, j_frac=1.0):
        """``j_frac``: share of the jerk limit the acceleration fade may use (0.5 when it overlaps a carried
        S-curve that uses the other half)."""
        v0, a0 = np.asarray(v0, float), np.asarray(a0, float)
        vh, vz = float(np.hypot(v0[0], v0[1])), abs(float(v0[2]))
        Tv = max(ramp_time(vh, spec.a_h, spec.j_h), ramp_time(vz, spec.a_v, spec.j_v), 0.6) * scale
        Ta = max(0.5, cls.H1DD_MAX * float(np.hypot(a0[0], a0[1])) / (j_frac * spec.j_h),
                 cls.H1DD_MAX * abs(float(a0[2])) / spec.j_v) * (scale if j_frac < 1.0 else 1.0)
        for _ in range(12):                # make the brake alone respect the limits
            b = cls(Tv, v0, a0, Ta)
            ts = np.linspace(0.0, b.T, 160)
            _, _, a, j = b.eval(ts)
            ok = (np.hypot(a[:, 0], a[:, 1]).max() <= spec.a_h * 1.001 and np.abs(a[:, 2]).max() <= spec.a_v * 1.001
                  and np.hypot(j[:, 0], j[:, 1]).max() <= spec.j_h * 1.001 and np.abs(j[:, 2]).max() <= spec.j_v * 1.001)
            if ok or np.hypot(a0[0], a0[1]) > spec.a_h * 1.001:
                return b
            Tv *= 1.15
            Ta *= 1.1
        return cls(Tv, v0, a0, Ta)


class _Move:
    """One straight 1-D move: displacement u * prof.x(t - t_rel)."""

    def __init__(self, t_rel, u, prof, tag="move"):
        self.t_rel, self.u, self.prof, self.tag = float(t_rel), np.asarray(u, float), prof, tag

    @property
    def t_end(self):
        return self.t_rel + self.prof.duration


# --------------------------------------------------------------------------- trajectory
class Sample:
    """One sampled state.  Mission code uses pos / vel / acc / yaw; renderers use the rest."""
    __slots__ = ("t", "pos", "vel", "acc", "jerk", "yaw", "yaw_rate", "roll", "pitch", "quat", "tilt",
                 "rotor_rpm", "winch_len", "pod", "airborne", "phase")

    def __init__(self, **kw):
        for k in self.__slots__:
            setattr(self, k, kw.get(k))

    def as_dict(self, nd=4):
        out = {}
        for k in self.__slots__:
            v = getattr(self, k)
            if isinstance(v, np.ndarray):
                v = [round(float(x), nd) for x in v]
            elif isinstance(v, (float, np.floating)):
                v = round(float(v), nd)
            elif isinstance(v, (list, tuple)) and v and isinstance(v[0], (float, np.floating)):
                v = [round(float(x), nd) for x in v]
            out[k] = v
        return out


class Trajectory:
    """A time-parameterised flight piece starting at absolute time ``t0`` (holds its final pose after ``t_end``).

    ``sample(t)`` gives the full rendered state (nominal path + hover disturbances when a ``dist`` is
    attached); ``nominal(t)`` (vectorised) gives the planned path only, which is what the mission uses."""

    def __init__(self, t0, p0, moves=(), brake=None, kind="hold", ground_start=False, ground_end=False,
                 spool=None, touchdown=None, rpm0=0.0, zref=(None, None), spec=None, yaw0=0.0, yaw_rate0=0.0,
                 phases=None, info=None):
        self.t0, self.p0 = float(t0), np.asarray(p0, float)
        self.moves, self.brake, self.kind = list(moves), brake, kind
        self.ground_start, self.ground_end = bool(ground_start), bool(ground_end)
        self.spool, self.touchdown, self.rpm0 = spool, touchdown, float(rpm0)
        self.zref = zref
        self.spec = spec or MotionSpec()
        self.yaw0, self.yaw_rate0 = float(yaw0), float(yaw_rate0)
        self.phases = phases or []           # [(t_abs_start, t_abs_end, name)]
        self.info = info or {}
        ends = [m.t_end for m in self.moves] + [self.brake.T if self.brake is not None else 0.0, 0.0]
        self.duration = float(max(ends) + self.info.get("tail_s", 0.0))
        self.t_end = self.t0 + self.duration
        self._yaw = None
        self.dist = None                     # Disturbance (rendering only)

    # ---- nominal path
    @property
    def p_end(self):
        return self.nominal(self.t_end + 1.0)[0][0]

    def nominal(self, t):
        """(pos, vel, acc, jerk), each (n, 3), for absolute times t (scalar or array)."""
        tl = np.atleast_1d(np.asarray(t, float)) - self.t0
        n = len(tl)
        p = np.tile(self.p0, (n, 1))
        v, a, j = np.zeros((n, 3)), np.zeros((n, 3)), np.zeros((n, 3))
        if self.brake is not None:
            bx, bv, ba, bj = self.brake.eval(np.clip(tl, 0.0, None))
            p += bx
            v += bv
            a += ba
            j += bj
        for m in self.moves:
            x, mv, ma, mj = m.prof.eval(tl - m.t_rel)
            p += x[:, None] * m.u
            v += mv[:, None] * m.u
            a += ma[:, None] * m.u
            j += mj[:, None] * m.u
        return p, v, a, j

    def breaks(self):
        """Absolute times where a primitive changes (for continuity tests)."""
        out = {self.t0}
        if self.brake is not None:
            out.add(self.t0 + self.brake.T)
        for m in self.moves:
            out.update(self.t0 + m.t_rel + b for b in m.prof.breaks())
        return sorted(out)

    def phase_at(self, t):
        name = self.kind
        for a, b, nm in self.phases:
            if a <= t < b:
                name = nm
        return name

    def in_descent(self, t):
        return any(nm in ("descent", "final", "flare") and a <= t < b for a, b, nm in self.phases)

    # ---- yaw
    def _build_yaw(self):
        sp = self.spec
        if not self.moves and self.brake is None and abs(self.yaw_rate0) < 1e-9:
            self._yaw = (np.array([self.t0, self.t0 + 1.0]), np.array([self.yaw0] * 2), np.zeros(2))
            return
        dt, t1 = 0.05, self.t_end + 4.0
        ts = np.arange(self.t0, t1 + dt, dt)
        _, v_ahead, _, _ = self.nominal(ts + sp.yaw_lead_s)
        spd = np.hypot(v_ahead[:, 0], v_ahead[:, 1])
        head = np.degrees(np.arctan2(v_ahead[:, 1], v_ahead[:, 0]))
        moving = spd > 0.8
        target = np.empty(len(ts))
        idx_moving = np.flatnonzero(moving).tolist()     # stationary: look up to 6 s ahead for the next heading
        cur = self.yaw0
        hold = ("stabilise", "descent", "final", "flare", "spool-up", "spin-down", "winch-in", "pre-launch")
        for i in range(len(ts)):
            if self.on_ground(ts[i]) or self.phase_at(ts[i]) in hold:
                pass
            elif moving[i]:
                cur = head[i]
            else:
                k = bisect.bisect_left(idx_moving, i)
                if k < len(idx_moving) and idx_moving[k] - i <= int(6.0 / dt):
                    cur = head[idx_moving[k]]
            target[i] = cur
        # rate- and acceleration-limited heading tracker, 4 substeps per table step
        yaw, rate = self.yaw0, self.yaw_rate0
        Y, R = np.empty(len(ts)), np.empty(len(ts))
        sub = dt / 4
        for i in range(len(ts)):
            Y[i], R[i] = yaw, rate
            for _ in range(4):
                err = (target[i] - yaw + 180.0) % 360.0 - 180.0
                want = np.clip(sp.yaw_gain * err, -sp.yaw_rate_dps, sp.yaw_rate_dps)
                # brake early enough that the rate can reach `want` without exceeding the accel limit
                stop = math.sqrt(2.0 * sp.yaw_acc_dps2 * abs(err)) * 0.9
                want = math.copysign(min(abs(want), stop), want) if err else 0.0
                rate += float(np.clip(want - rate, -sp.yaw_acc_dps2 * sub, sp.yaw_acc_dps2 * sub))
                yaw += rate * sub
        self._yaw = (ts, Y, R)

    def yaw_at(self, t):
        """(yaw_deg, yaw_rate_dps) at absolute time t (cubic Hermite on a 20 Hz table)."""
        if self._yaw is None:
            self._build_yaw()
        ts, Y, R = self._yaw
        t = float(t)
        if t <= ts[0]:
            return float(Y[0]), float(R[0]) if t == ts[0] else 0.0
        if t >= ts[-1]:
            return float(Y[-1]), 0.0
        i = min(int((t - ts[0]) / (ts[1] - ts[0])), len(ts) - 2)
        h = ts[i + 1] - ts[i]
        s = (t - ts[i]) / h
        y0, y1, r0, r1 = Y[i], Y[i + 1], R[i], R[i + 1]
        h00, h10, h01, h11 = 2 * s ** 3 - 3 * s ** 2 + 1, s ** 3 - 2 * s ** 2 + s, -2 * s ** 3 + 3 * s ** 2, s ** 3 - s ** 2
        y = h00 * y0 + h10 * h * r0 + h01 * y1 + h11 * h * r1
        d00, d10, d01, d11 = 6 * s ** 2 - 6 * s, 3 * s ** 2 - 4 * s + 1, -6 * s ** 2 + 6 * s, 3 * s ** 2 - 2 * s
        r = (d00 * y0 + d01 * y1) / h + d10 * r0 + d11 * r1
        return float(y), float(r)

    # ---- rotors and ground contact
    def on_ground(self, t):
        if self.ground_start and self.spool is not None and t < self.spool[1]:
            return True
        if self.ground_end and self.touchdown is not None and t >= self.touchdown:
            return True
        return self.ground_start and self.spool is None and self.kind == "hold"

    def rpm_override(self, t):
        """Rotor RPM fraction of hover on the ground (spool-up / spin-down / off), None while flying."""
        if self.ground_start and self.spool is not None and t < self.spool[1]:
            a, b = self.spool
            return float(self.rpm0 + (1.0 - self.rpm0) * smooth01((t - a) / max(b - a, 1e-6)))
        if self.ground_end and self.touchdown is not None and t >= self.touchdown:
            return float(1.0 - smooth01((t - self.touchdown) / self.spec.spin_down_s))
        if self.kind == "hold" and self.ground_start:
            return float(self.rpm0 * (1.0 - smooth01((t - self.t0) / self.spec.spin_down_s)))
        return None

    def air_factor(self, z):
        """0 on the ground -> 1 above 2 m over the takeoff / landing reference (fades the disturbances)."""
        f = 1.0
        for zr in self.zref:
            if zr is not None:
                f = min(f, float(smooth01((z - zr - self.spec.skid_m) / 2.0)))
        return f

    # ---- full sample
    def sample(self, t, dist=None, winch=None, nominal=False):
        """Full state at absolute time t: pos, vel, acc, jerk, quat (w,x,y,z), roll/pitch/yaw (deg, ZYX as in
        isaac/drone_viz.quat_ypr), rotor_rpm (n_rotors,), winch_len (m) and pod position."""
        sp = self.spec
        p, v, a, j = (x[0] for x in self.nominal(t))
        yaw, yrate = self.yaw_at(t)
        ground = self.on_ground(t)
        wind = np.zeros(3)
        dist = dist if dist is not None else self.dist
        if dist is not None and not nominal:
            f = 0.0 if ground else self.air_factor(p[2])
            if f > 0.0:
                dp, dv, da, dj, w, dyaw = dist.eval(t)
                p, v, a, j = p + f * dp, v + f * dv, a + f * da, j + f * dj
                wind = f * w
                yaw += f * dyaw
        att = attitude(a, v, wind, yaw, sp, ground=ground)
        ovr = self.rpm_override(t)
        rpm = rotor_rpm(att["thrust"], j, yrate, yaw, sp, frac=ovr)
        wl, pod = (winch.length(t), None) if winch is not None else (0.0, None)
        if winch is not None and wl > 1e-3:
            pod = winch.pod_pos(t, p, wl)
        return Sample(t=float(t), pos=p, vel=v, acc=a, jerk=j, yaw=float(yaw), yaw_rate=float(yrate),
                      roll=att["roll"], pitch=att["pitch"], quat=att["quat"], tilt=att["tilt"], rotor_rpm=rpm,
                      winch_len=float(wl), pod=pod, airborne=not ground, phase=self.phase_at(t))

    # ---- serialisation (for --replay in Isaac)
    def to_dict(self):
        return {"t0": self.t0, "p0": self.p0.tolist(), "kind": self.kind,
                "moves": [{"t": m.t_rel, "u": m.u.tolist(), "segs": m.prof.to_list(), "tag": m.tag} for m in self.moves],
                "brake": None if self.brake is None else {"T": self.brake.Tv, "Ta": self.brake.Ta,
                                                          "v0": self.brake.v0.tolist(), "a0": self.brake.a0.tolist()},
                "ground_start": self.ground_start, "ground_end": self.ground_end, "spool": self.spool,
                "touchdown": self.touchdown, "rpm0": self.rpm0, "zref": list(self.zref), "yaw0": self.yaw0,
                "yaw_rate0": self.yaw_rate0, "phases": self.phases, "info": self.info}

    @classmethod
    def from_dict(cls, d, spec):
        moves = [_Move(m["t"], m["u"], Profile1D(m["segs"]), m.get("tag", "move")) for m in d["moves"]]
        br = d.get("brake")
        brake = None if br is None else Brake(br["T"], br["v0"], br["a0"], br.get("Ta"))
        return cls(d["t0"], d["p0"], moves, brake, d["kind"], d["ground_start"], d["ground_end"],
                   None if d["spool"] is None else tuple(d["spool"]), d["touchdown"], d["rpm0"], tuple(d["zref"]),
                   spec, d["yaw0"], d["yaw_rate0"], [tuple(x) for x in d["phases"]], d.get("info"))

    @classmethod
    def hold(cls, t0, p, spec, ground=False, yaw0=0.0, rpm0=0.0):
        return cls(t0, p, kind="hold", ground_start=ground, ground_end=False, spec=spec, yaw0=yaw0,
                   rpm0=rpm0 if ground else 1.0, zref=(float(p[2]) - spec.skid_m if ground else None, None))


# --------------------------------------------------------------------------- attitude / rotors
def quat_from_R(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q) * (1 if q[0] >= 0 else -1)


def attitude(acc, vel, wind, yaw_deg, spec, ground=False):
    """Thrust direction from the dynamics: f = a + k |v_air| v_air + g z (specific thrust), tilt clamped.
    Returns roll / pitch / yaw (deg, R = Rz(yaw) Ry(pitch) Rx(roll), +pitch = nose down), quat (w,x,y,z),
    tilt (deg) and the specific thrust vector."""
    acc, vel, wind = np.asarray(acc, float), np.asarray(vel, float), np.asarray(wind, float)
    v_air = vel - wind
    f = acc + spec.drag_k * np.linalg.norm(v_air) * v_air + np.array([0.0, 0.0, G])
    if ground:
        f = np.array([0.0, 0.0, G])
    fz = max(f[2], 0.3 * G)
    fh = f[:2]
    nh = float(np.hypot(*fh))
    lim = math.tan(math.radians(spec.tilt_max_deg)) * fz
    if nh > lim:
        fh = fh * (lim / nh)
    f = np.array([fh[0], fh[1], fz])
    b3 = f / np.linalg.norm(f)
    psi = math.radians(yaw_deg)
    xc = np.array([math.cos(psi), math.sin(psi), 0.0])
    b2 = np.cross(b3, xc)
    b2 /= np.linalg.norm(b2)
    b1 = np.cross(b2, b3)
    R = np.column_stack([b1, b2, b3])
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -R[2, 0]))))
    roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, b3[2]))))
    return {"roll": roll, "pitch": pitch, "yaw": yaw, "quat": quat_from_R(R), "tilt": tilt,
            "thrust": f, "R": R}


def rotor_layout(n=8):
    """[(arm_angle_rad, spin_sign, level)] in isaac/drone_viz ROTOR_ORDER: coaxial X8 = arms 0..3 at 45 + 90k deg,
    each lower (level 0) then upper (level 1), sign = +1 if (arm + level) is odd else -1 (the two rotors of an arm
    counter-rotate, so yaw torque cancels); quad-X: 4 arms, alternating."""
    if n == 8:
        return [(math.radians(45 + 90 * k), 1 if (k + lvl) % 2 else -1, lvl) for k in range(4) for lvl in (0, 1)]
    return [(math.radians(45 + 360 / n * k), (1 if k % 2 == 0 else -1), 0) for k in range(n)]


def rotor_rpm(thrust, jerk, yaw_rate, yaw_deg, spec, frac=None):
    """Per-rotor RPM.  Collective from the specific thrust (rpm ~ sqrt(thrust)); small differentials: yaw torque
    (CW vs CCW sets) and the pitch/roll rate change (jerk across the arms).  ``frac`` overrides the collective
    (fraction of hover RPM) on the ground: spool-up, spin-down, off."""
    lay = rotor_layout(spec.n_rotors)
    if frac is not None:
        return np.array([spec.hover_rpm * frac for _ in lay])
    coll = spec.hover_rpm * math.sqrt(max(np.linalg.norm(thrust), 0.0) / G)
    psi = math.radians(yaw_deg)
    out = []
    for ang, sgn, _ in lay:
        arm = np.array([math.cos(ang + psi), math.sin(ang + psi)])
        d_tilt = -0.03 * float(np.dot(jerk[:2], arm)) / max(spec.j_h, 1e-6)
        d_yaw = 0.02 * sgn * float(np.clip(yaw_rate / max(spec.yaw_rate_dps, 1e-6), -1, 1))
        out.append(coll * (1.0 + d_tilt + d_yaw))
    return np.array(out)


# --------------------------------------------------------------------------- disturbances
class Disturbance:
    """Deterministic hover drift / bob (coloured noise: seeded sinusoids, ~1/f amplitude) and site-wide wind
    gusts with a downwind position response.  ``eval(t)`` -> (dp, dv, da, dj, wind_vec, dyaw_deg)."""

    def __init__(self, seed=7, did="d1", spec=None):
        sp = spec or MotionSpec()
        self.spec = sp
        rng = np.random.default_rng(zlib.crc32(f"drift:{seed}:{did}".encode()))
        comps = []
        for ax, rms in ((0, sp.drift_h_m), (1, sp.drift_h_m), (2, sp.drift_v_m)):
            f = rng.uniform(0.04, 0.32, 4)
            amp = f ** -0.8
            amp *= rms * math.sqrt(2.0) / math.sqrt(float((amp ** 2).sum()))
            for k in range(4):
                comps.append((ax, 2 * math.pi * f[k], amp[k], rng.uniform(0, 2 * math.pi)))
        self.comps = comps
        self.yaw_comps = [(2 * math.pi * rng.uniform(0.03, 0.15), sp.drift_yaw_deg * rng.uniform(0.5, 1.0),
                           rng.uniform(0, 2 * math.pi)) for _ in range(2)]
        wr = np.random.default_rng(zlib.crc32(f"wind:{seed}".encode()))
        base = wr.uniform(0, 2 * math.pi)
        lag = np.random.default_rng(zlib.crc32(f"lag:{seed}:{did}".encode())).uniform(0.0, 1.5)
        gusts, t = [], wr.uniform(5.0, 15.0)
        while t < 7200.0:
            dur = wr.uniform(*sp.gust_dur_s)
            w = wr.uniform(*sp.gust_mps)
            th = base + wr.normal(0.0, math.radians(20))
            gusts.append((t + lag, dur, w, math.cos(th), math.sin(th)))
            t += dur + wr.uniform(*sp.gust_gap_s)
        self.gusts = gusts
        self._g_t = [g[0] for g in gusts]

    @staticmethod
    def _bump(u):
        """64 u^3 (1-u)^3 (peak 1 at u = 0.5; value, slope and curvature zero at both ends) and derivatives."""
        b = 64 * u ** 3 * (1 - u) ** 3
        b1 = 192 * u ** 2 * (1 - u) ** 2 * (1 - 2 * u)
        b2 = 384 * u * (1 - u) * (1 - 5 * u + 5 * u * u)
        b3 = 384 * (1 - 12 * u + 30 * u * u - 20 * u ** 3)
        return b, b1, b2, b3

    def eval(self, t):
        t = float(t)
        dp, dv, da, dj = np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3)
        for ax, w, A, ph in self.comps:
            s, c = math.sin(w * t + ph), math.cos(w * t + ph)
            dp[ax] += A * s
            dv[ax] += A * w * c
            da[ax] -= A * w * w * s
            dj[ax] -= A * w ** 3 * c
        dyaw = sum(A * math.sin(w * t + ph) for w, A, ph in self.yaw_comps)
        wind = np.zeros(3)
        k = bisect.bisect_right(self._g_t, t)
        for g in self.gusts[max(0, k - 3):k]:
            t0, dur, W, cx, cy = g
            if t0 <= t <= t0 + dur:                       # wind speed bump
                b, _, _, _ = self._bump((t - t0) / dur)
                wind[0] += W * b * cx
                wind[1] += W * b * cy
            # position response: a longer, delayed bump downwind
            r0, rd = t0 + 0.5, dur + 2.0
            if r0 <= t <= r0 + rd:
                u = (t - r0) / rd
                b, b1, b2, b3 = self._bump(u)
                A = self.spec.gust_disp_k * W * W
                for ax, c in ((0, cx), (1, cy)):
                    dp[ax] += A * c * b
                    dv[ax] += A * c * b1 / rd
                    da[ax] += A * c * b2 / rd ** 2
                    dj[ax] += A * c * b3 / rd ** 3
        return dp, dv, da, dj, wind, dyaw


# --------------------------------------------------------------------------- winch
class Winch:
    """Winch cable length over time (S-curve payout / reel-in) and the hanging pod with a damped sway."""

    def __init__(self, spec=None, seed=7, did="d1"):
        self.spec = spec or MotionSpec()
        self.events = []                 # dicts: t, T, L0, L1, kind, pod (x,y,z ground point), dir
        self._rng = np.random.default_rng(zlib.crc32(f"winch:{seed}:{did}".encode()))

    def add(self, t, L0, L1, T, kind, pod=None):
        self.events = [e for e in self.events if e["t"] < t]
        th = float(self._rng.uniform(0, 2 * math.pi))
        self.events.append({"t": float(t), "T": float(T), "L0": float(L0), "L1": float(L1), "kind": kind,
                            "pod": None if pod is None else [float(v) for v in pod], "dir": [math.cos(th), math.sin(th)]})

    def _event(self, t):
        e = None
        for ev in self.events:
            if ev["t"] <= t:
                e = ev
        return e

    def length(self, t):
        e = self._event(t)
        if e is None:
            return 0.0
        u = (t - e["t"]) / e["T"] if e["T"] > 0 else 1.0
        return float(e["L0"] + (e["L1"] - e["L0"]) * smooth01(u))

    def speed(self, t, h=1e-3):
        return (self.length(t + h) - self.length(t - h)) / (2 * h)

    def pod_pos(self, t, drone_pos, L):
        """Pod centre: hanging under the drone with a damped pendulum sway, or resting on its ground point."""
        e = self._event(t)
        sp = self.spec
        hang = np.asarray(drone_pos, float) - np.array([0.0, 0.0, L])
        if e is None or e["pod"] is None:
            return hang
        ground = np.array(e["pod"], float)
        u = min(max((t - e["t"]) / e["T"], 0.0), 1.0) if e["T"] > 0 else 1.0
        # sway: A e^{-zeta w t} sin(phase), w = sqrt(g / L(t)); windowed so it is zero at contact
        n = 48
        ts = np.linspace(e["t"], t, n) if t > e["t"] else np.array([e["t"], e["t"]])
        Ls = np.array([max(self.length(x), 0.4) for x in ts])
        w = np.sqrt(G / Ls)
        phase = float(np.trapezoid(w, ts)) if hasattr(np, "trapezoid") else float(np.trapz(w, ts))
        decay = math.exp(-sp.sway_zeta * phase)
        win = float(1.0 - smooth01(u)) if e["kind"] == "lower" else float(smooth01(u))
        amp = sp.sway_m * decay * win * (1.0 if e["kind"] == "lower" else 0.5)
        sway = amp * math.sin(phase) * np.array([e["dir"][0], e["dir"][1], 0.0])
        sway += 0.35 * amp * math.cos(phase) * np.array([-e["dir"][1], e["dir"][0], 0.0])
        rest = ground[2] + 0.0
        pos = hang + sway
        if pos[2] <= rest + 1e-3 or (e["kind"] == "lower" and u >= 1.0) or (e["kind"] == "raise" and self.length(t) >= e["L0"] - 1e-3):
            return np.array([ground[0], ground[1], rest])
        # blend the hanging xy towards the ground point in the last 0.6 m (the line pulls it there)
        k = float(smooth01(1.0 - (pos[2] - rest) / 0.6))
        pos[:2] = pos[:2] * (1 - k) + ground[:2] * k
        return pos

    def to_list(self):
        return self.events

    def load(self, events):
        self.events = [dict(e) for e in events]


# --------------------------------------------------------------------------- planning
def _legs(pts):
    return [(pts[k], pts[k + 1]) for k in range(len(pts) - 1)]


def _profiles_for_leg(a, b, spec, landing=False, liftoff=False, v_start=0.0, vmax=None):
    """[(unit_vec, Profile1D, tag, t_offset)] for the horizontal and vertical parts of a straight leg a -> b.
    ``v_start``: horizontal speed already flown along the leg (re-plan); may add a move back on overshoot."""
    d = np.asarray(b, float) - np.asarray(a, float)
    dh = float(np.hypot(d[0], d[1]))
    out = []
    if dh > 1e-3:
        u = np.array([d[0] / dh, d[1] / dh, 0.0])
        if v_start > 1e-3:      # carried ramp: 80% accel / 50% jerk, headroom for the brake term it overlaps
            prof, over = Profile1D.from_speed(dh, v_start, vmax or spec.cruise_mps, 0.8 * spec.a_h, 0.5 * spec.j_h)
            out.append((u, prof, "cruise", 0.0))
            if over > 1e-3:
                out.append((-u, Profile1D.rest_to_rest(over, spec.cruise_mps, spec.a_h, spec.j_h), "cruise",
                            prof.duration))
        else:
            out.append((u, Profile1D.rest_to_rest(dh, spec.cruise_mps, spec.a_h, spec.j_h), "cruise", 0.0))
    dz = float(d[2])
    if abs(dz) > 1e-3:
        u = np.array([0.0, 0.0, 1.0 if dz > 0 else -1.0])
        if landing and dz < 0:
            prof = Profile1D.scheduled(abs(dz), [(spec.descend_mps, spec.final_h), (spec.final_mps, spec.flare_h),
                                                 (spec.flare_mps, 0.0)], spec.a_v, spec.j_v)
            tag = "descent"
        elif liftoff:
            prof, tag = Profile1D.rest_to_rest(abs(dz), spec.liftoff_mps, spec.a_v, spec.j_v), "liftoff"
        else:
            prof = Profile1D.rest_to_rest(abs(dz), spec.climb_mps if dz > 0 else spec.descend_mps, spec.a_v, spec.j_v)
            tag = "climb" if dz > 0 else "descend"
        out.append((u, prof, tag, 0.0))
    return out


def _build(path, t0, spec, v0, a0, kind, from_ground, beta, brake_scale, overlap_brake, delay, rpm0, yaw0, yaw_rate0,
           carry=True, zref=None):
    pts = [np.asarray(p, float) for p in path]
    p_start = pts[0].copy()
    moving = (np.linalg.norm(v0) > 0.05 or np.linalg.norm(a0) > 0.05) and not from_ground
    rest = [p.copy() for p in pts[1:]]
    climb_first = len(rest) >= 2 and float(np.hypot(*(rest[0][:2] - p_start[:2]))) < 0.5
    # re-plan with "carry": the speed along the first horizontal leg is kept by that leg's S-curve; only the
    # cross-track velocity and the current acceleration go into the brake term
    h_idx = next((i for i, w in enumerate(rest) if float(np.hypot(*(w[:2] - (rest[i - 1] if i else p_start)[:2]))) > 0.5), None)
    v_carry_max = spec.cruise_mps
    v_par, carry = 0.0, bool(carry and moving and delay <= 0 and h_idx is not None and h_idx <= 1)
    brake = None
    if moving:
        vb = np.asarray(v0, float).copy()
        if carry:
            goal = rest[h_idx]
            u = goal[:2] - p_start[:2]
            u = u / max(np.linalg.norm(u), 1e-9)
            T_fix = None
            for _ in range(30):      # fixed point: leg direction from where the brake leaves the drone
                v_par = max(0.0, float(v0[0] * u[0] + v0[1] * u[1]))
                vb = np.asarray(v0, float) - v_par * np.array([u[0], u[1], 0.0])
                if T_fix is None:
                    b0 = Brake.for_state(vb, a0, spec, brake_scale, j_frac=0.5)
                    T_fix = (b0.Tv, b0.Ta)
                brake = Brake(T_fix[0], vb, a0, T_fix[1])
                un = goal[:2] - (p_start[:2] + brake.end[:2])
                un = un / max(np.linalg.norm(un), 1e-9)
                if np.linalg.norm(un - u) < 1e-12:
                    break
                u = un
            # exact velocity continuity: the carried leg runs along u, ending at the goal's projection
            if v_par < 1e-3 or float(np.dot(goal[:2] - (p_start[:2] + brake.end[:2]), u)) <= 0.0:
                carry = False
            else:
                # the fading forward acceleration still adds ~0.2 a0 Ta of speed: aim the S-curve that much lower
                a_par = max(0.0, float(a0[0] * u[0] + a0[1] * u[1]))
                v_carry_max = max(1.0, spec.cruise_mps - 0.2 * a_par * brake.Ta)
                vs_xy = p_start[:2] + brake.end[:2]
                L = float(np.dot(goal[:2] - vs_xy, u))
                goal[:2] = vs_xy + L * u            # moves the goal by the fixed-point residual (< 1e-9 m)
        if brake is None or not carry:
            vb, v_par = np.asarray(v0, float), 0.0
            brake = Brake.for_state(vb, a0, spec, brake_scale)
    vs = p_start + (brake.end if brake is not None else 0.0)
    if not rest:
        rest = [vs.copy()]
    if brake is not None and climb_first:
        rest[0][:2] = vs[:2]                    # the climb happens where the brake leaves the drone
    if from_ground:
        if not climb_first or rest[0][2] <= p_start[2] + 0.1:
            rest.insert(0, np.array([p_start[0], p_start[1], p_start[2] + spec.liftoff_h]))
        lift_top = min(rest[0][2], p_start[2] + spec.liftoff_h)
        if rest[0][2] > lift_top + 0.3:
            rest.insert(0, np.array([p_start[0], p_start[1], lift_top]))
    seq = [vs] + rest
    legs = _legs(seq)
    moves, phases = [], []
    t = delay
    spool = None
    if from_ground:
        spool = (t0 + t, t0 + t + spec.spool_s)
        phases.append((spool[0], spool[1], "spool-up"))
        t += spec.spool_s
    elif brake is not None and not overlap_brake and not carry:
        t += brake.T
    if delay > 0:
        phases.insert(0, (t0, t0 + delay, "winch-in" if kind != "takeoff" else "pre-launch"))
    prev_dec, prev_kind, prev_start, prev_end = 0.0, None, t, t
    touchdown = None
    n_legs = len(legs)
    for k, (a, b) in enumerate(legs):
        last = k == n_legs - 1
        landing = kind == "land" and last and b[2] < a[2] - 1e-3 and float(np.hypot(*(b[:2] - a[:2]))) < 1e-3
        liftoff = from_ground and k == 0
        carried = carry and k == h_idx
        parts = _profiles_for_leg(a, b, spec, landing=landing, liftoff=liftoff, v_start=v_par if carried else 0.0,
                                  vmax=v_carry_max if carried else None)
        if not parts:
            continue
        horiz = any(tag == "cruise" for _, _, tag, _ in parts)
        lkind = "h" if horiz else "v"
        dur = max(off + p.duration for _, p, _, off in parts)
        dec = max(p.t_decel for _, p, _, _ in parts)
        if carry and climb_first and k == 0 and brake is not None:
            start = t + brake.Ta                              # let the inherited vertical accel fade first
        elif k == 0 or prev_kind is None or carried:
            start = t
        elif landing:
            start = prev_end + spec.land_stabilise_s
            phases.append((t0 + prev_end, t0 + start, "stabilise"))
        elif from_ground and k == 1 and legs[0][1][2] - legs[0][0][2] <= spec.liftoff_h + 0.31:
            start = prev_end + spec.liftoff_hover_s          # hover at 2 m after lift-off
            phases.append((t0 + prev_end, t0 + start, "hover-2m"))
        elif brake is not None and overlap_brake and k == 1 and climb_first:
            start = prev_start                                # airborne re-plan: climb and translate together
        else:
            b_eff = beta * (1.0 if (prev_kind != lkind) else 0.5)
            start = max(prev_start, prev_end - b_eff * prev_dec)
        for u, prof, tag, off in parts:
            moves.append(_Move(start + off, u, prof, tag))
            if tag == "descent":
                # name the descent sub-phases from the profile's segments (cruise levels)
                segs = prof.segs
                tt = start
                names = []
                for (T, va, vb) in segs:
                    lvl = max(va, vb)
                    nm = "flare" if lvl <= spec.flare_mps + 1e-6 else "final" if lvl <= spec.final_mps + 1e-6 else "descent"
                    names.append((t0 + tt, t0 + tt + T, nm))
                    tt += T
                phases.extend(names)
                touchdown = t0 + start + prof.duration
            else:
                phases.append((t0 + start + off, t0 + start + off + prof.duration, tag))
        prev_start, prev_end, prev_dec, prev_kind = start, start + dur, dec, lkind
    tail = 0.0
    if kind == "pod":
        tail = spec.pod_stabilise_s
    end_rel = max([m.t_end for m in moves] + [brake.T if brake is not None else 0.0, t])
    if kind == "pod":
        phases.append((t0 + end_rel, t0 + end_rel + tail, "stabilise"))
    if touchdown is not None:
        phases.append((touchdown, touchdown + spec.spin_down_s, "spin-down"))
    zref = tuple(zref) if zref is not None else (float(p_start[2]) - spec.skid_m if from_ground else None,
                                                 float(pts[-1][2]) - spec.skid_m if kind == "land" else None)
    prim_end = max([m.t_end for m in moves] + [brake.T if brake is not None else 0.0, 0.0])
    tr = Trajectory(t0, p_start, moves, brake, kind=kind, ground_start=from_ground, ground_end=kind == "land",
                    spool=spool, touchdown=touchdown, rpm0=rpm0, zref=zref, spec=spec, yaw0=yaw0, yaw_rate0=yaw_rate0,
                    phases=phases, info={"tail_s": end_rel - prim_end + tail, "carry": carry,
                                         "beta": beta, "brake_scale": brake_scale, "overlap_brake": overlap_brake})
    return tr


def check_limits(tr, spec, dt=0.05):
    """Max |a_h|, |a_v|, |j_h|, |j_v|, horizontal speed over the nominal path, and whether all are in limits."""
    ts = np.arange(tr.t0, tr.t_end + dt, dt)
    _, v, a, j = tr.nominal(ts)
    m = {"a_h": float(np.hypot(a[:, 0], a[:, 1]).max()), "a_v": float(np.abs(a[:, 2]).max()),
         "j_h": float(np.hypot(j[:, 0], j[:, 1]).max()), "j_v": float(np.abs(j[:, 2]).max()),
         "v_h": float(np.hypot(v[:, 0], v[:, 1]).max())}
    ok = (m["a_h"] <= spec.a_h * 1.01 and m["a_v"] <= spec.a_v * 1.01 and m["j_h"] <= spec.j_h * 1.01
          and m["j_v"] <= spec.j_v * 1.01 and m["v_h"] <= spec.cruise_mps * 1.1)
    return ok, m


def check_clearance(tr, grid, spec, dt=0.25):
    """Rotor-disc clearance over the heightmap along the nominal path.  Returns (ok, worst_margin_m, t_worst).
    Away from the takeoff / landing columns the lowest point of the airframe (skids) must clear the highest
    cell under the rotor disc by ``clear_m``; inside the columns it must not go below the surface."""
    if grid is None:
        return True, float("inf"), None
    ts = np.arange(tr.t0, tr.t_end + dt, dt)
    p, _, _, _ = tr.nominal(ts)
    r = spec.span_m / 2.0
    ring = [(0.0, 0.0)] + [(r * math.cos(a), r * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)]
    h_disc = np.max([grid.height_at(p[:, 0] + dx, p[:, 1] + dy) for dx, dy in ring], axis=0)
    h_c = grid.height_at(p[:, 0], p[:, 1])
    a0, a1 = tr.p0, tr.p_end
    col = r + 0.5
    near = (np.hypot(p[:, 0] - a0[0], p[:, 1] - a0[1]) < col) | (np.hypot(p[:, 0] - a1[0], p[:, 1] - a1[1]) < col)
    skid = p[:, 2] - spec.skid_m
    margin = np.where(near, skid - h_c + 0.05 + spec.clear_m * 0.0, skid - h_disc - spec.clear_m)
    # inside a column the requirement is "not below the surface" (margin >= -0.05 m slack -> >= 0 here)
    i = int(np.argmin(margin))
    return bool(margin[i] >= 0.0), float(margin[i]), float(ts[i])


def plan(path, t0=0.0, spec=None, v0=(0, 0, 0), a0=(0, 0, 0), kind="hover", from_ground=False, grid=None,
         max_alt=None, delay=0.0, rpm0=None, yaw0=0.0, yaw_rate0=0.0, zref=None):
    """Waypoint path -> smooth ``Trajectory`` starting at ``t0``.

    path      : [start, w1, ..., wn] (x, y, z); the coordinator's route is [climb point?, cruise point, dest]
    kind      : "hover" (stop at wn), "pod" (hover + stabilise), "land" (last leg is a perch landing), "takeoff"
    v0, a0    : current velocity / acceleration (a mid-flight re-plan keeps them; C2 across the switch)
    from_ground: start with rotor spool-up and a lift-off to a 2 m hover
    grid      : heightmap for the clearance check; on a conflict the corners are un-blended, then the cruise
                altitude is raised in 3 m steps (up to ``max_alt``)
    """
    spec = spec or MotionSpec()
    v0, a0 = np.asarray(v0, float), np.asarray(a0, float)
    rpm0 = (0.0 if from_ground else 1.0) if rpm0 is None else rpm0
    path = [np.asarray(p, float) for p in path]
    base = (t0, spec, v0, a0, kind, from_ground)
    if grid is not None and zref is None:             # ground under the start / end: fades the hover drift
        zref = (float(grid.height_at(path[0][0], path[0][1])), float(grid.height_at(path[-1][0], path[-1][1])))
    tail = (delay, rpm0, yaw0, yaw_rate0)
    # limits ladder for re-plans: carry the along-track speed (longer brakes), then brake fully (overlapped,
    # then sequential).  Paths from rest have no brake and pass on the first try.
    tries = [(True, 1.0, True), (True, 1.3, True), (True, 1.7, True), (True, 2.5, True),
             (False, 1.0, True), (False, 1.3, True), (False, 1.7, True), (False, 1.0, False)]
    tr = None
    for cy, bs, ov in tries:
        tr = _build(path, *base, 1.0, bs, ov, *tail, carry=cy, zref=zref)
        if tr.brake is None or check_limits(tr, spec)[0]:
            break
    beta, bs, ov, cy = tr.info["beta"], tr.info["brake_scale"], tr.info["overlap_brake"], tr.info["carry"]
    tail = tail + (cy, zref)
    if grid is None or check_clearance(tr, grid, spec)[0]:
        return tr
    tr0 = _build(path, *base, 0.0, bs, ov, *tail)     # stop-and-turn corners: the coordinator's checked geometry
    if check_clearance(tr0, grid, spec)[0]:
        return tr0
    if len(path) >= 3:                                # raise the interior (cruise) waypoints
        top = max(float(p[2]) for p in path[1:-1]) if len(path) > 2 else float(path[-1][2])
        for k in range(1, 6):
            dz = 3.0 * k
            if max_alt is not None and top + dz > max_alt:
                break
            pp = [path[0]] + [p + np.array([0, 0, dz]) if abs(float(p[2]) - top) < 0.6 else p for p in path[1:-1]] + [path[-1]]
            if kind in ("hover", "pod", "takeoff") and abs(float(path[-1][2]) - top) < 0.6:
                pp[-1] = path[-1]
            for b in (beta, 0.0):
                trx = _build(pp, *base, b, bs, ov, *tail)
                if check_clearance(trx, grid, spec)[0]:
                    trx.info["raised_m"] = dz
                    return trx
    tr0.info["clearance_warning"] = check_clearance(tr0, grid, spec)[1]
    return tr0


def flight_time(path, spec=None, **kw):
    """Duration (s) of ``plan(path, ...)``: the arrival time the mission must schedule with.  Same keyword
    arguments as ``plan`` (kind, from_ground, v0, a0, grid, max_alt, delay)."""
    kw.pop("t0", None)
    return plan(path, 0.0, spec, **kw).duration


# --------------------------------------------------------------------------- timeline
class MotionTimeline:
    """All flight pieces of one drone, back to back.  ``sample(t)`` works for any t (past pieces are kept), so a
    renderer can sample anywhere between the last two mission steps."""

    def __init__(self, did, p0, yaw0, spec, seed=7, ground=True):
        self.did, self.spec, self.seed = did, spec, seed
        self._lock = threading.Lock()        # the mission may add pieces on a worker thread while Isaac samples
        self.pieces = [Trajectory.hold(-1e9, p0, spec, ground=ground, yaw0=yaw0)]
        self._t = [-1e9]
        self.dist = Disturbance(seed, did, spec)
        self.winch = Winch(spec, seed, did)

    def add(self, tr):
        tr.dist = self.dist
        with self._lock:
            k = bisect.bisect_left(self._t, tr.t0)
            self.pieces, self._t = self.pieces[:k] + [tr], self._t[:k] + [tr.t0]

    def piece_at(self, t):
        with self._lock:
            return self.pieces[max(0, bisect.bisect_right(self._t, t) - 1)]

    def nominal(self, t):
        tr = self.piece_at(t)
        p, v, a, j = (x[0] for x in tr.nominal(t))
        yaw, rate = tr.yaw_at(t)
        return p, v, a, yaw, rate

    def rpm_frac(self, t):
        tr = self.piece_at(t)
        o = tr.rpm_override(t)
        return 1.0 if o is None else o

    def sample(self, t, nominal=False):
        s = self.piece_at(t).sample(t, dist=self.dist, winch=self.winch, nominal=nominal)
        if s.winch_len > 1e-3 and s.phase in ("hover", "pod", "stabilise", "takeoff", "hold"):
            sp = self.winch.speed(t)
            s.phase = "winch-payout" if sp > 0.02 else "winch-reel-in" if sp < -0.02 else "pod-on-rubble"
        return s

    def breaks(self):
        out = []
        for tr in self.pieces[1:]:
            out.extend(tr.breaks())
        return sorted(set(out))

    def to_dict(self):
        return {"id": self.did, "seed": self.seed, "pieces": [p.to_dict() for p in self.pieces],
                "winch": self.winch.to_list()}

    @classmethod
    def from_dict(cls, d, spec):
        first = Trajectory.from_dict(d["pieces"][0], spec)
        tl = cls(d["id"], first.p0, first.yaw0, spec, d.get("seed", 7), ground=first.ground_start)
        tl.pieces, tl._t = [], []
        for pd in d["pieces"]:  # noqa: B007
            tr = Trajectory.from_dict(pd, spec)
            tr.dist = tl.dist
            tl.pieces.append(tr)
            tl._t.append(tr.t0)
        tl.winch.load(d.get("winch", []))
        return tl


# --------------------------------------------------------------------------- mission backend
class SmoothFlight(KinematicFlight):
    """Drop-in ``KinematicFlight`` whose drones follow ``plan()`` trajectories.

    * Same constructor, commands, events and states; arrival happens at the trajectory's exact end time and
      the dwell / winch timers carry the sub-step remainder, so mission timing equals the trajectory timing.
    * ``eta()`` = ``flight_time()`` of the trajectory the command would build (from rest), so dispatch scores
      use the same clock the drones fly.
    * ``motion_sample(did, t)`` / ``timelines`` for rendering at any frame rate; ``export_motion()`` /
      ``save_motion(path)`` for an Isaac ``--replay``.
    """

    GROUND_STATES = ("docked", "landed", "scanning")

    def __init__(self, grid, starts=STARTS, platform=None, seed=7, motion=None, t0=0.0, **kw):
        super().__init__(grid, starts=starts, platform=platform, **kw)
        if motion is not None:
            self.mspec = motion
        elif platform is not None:
            self.mspec = MotionSpec.from_platform(platform)
        else:      # legacy numbers from the KinematicFlight defaults
            self.mspec = MotionSpec.from_platform("legacy_small", cruise_mps=self.speed, climb_mps=self.climb,
                                                  descend_mps=self.descend)
        self.seed = seed
        self._now = float(t0)
        self.timelines = {did: MotionTimeline(did, d.pos, d.yaw, self.mspec, seed, ground=True)
                          for did, d in self.drones.items()}
        self.launch_order = {did: i for i, did in enumerate(self.drones)}
        for d in self.drones.values():
            d.traj = None
        _ACTIVE.append(weakref.ref(self))

    # ---- planning / timing
    def _on_ground(self, d):
        return d.state in self.GROUND_STATES

    def _kind_for(self, typ, dest_z=None, x=None, y=None):
        if typ in ("goto_land_scan", "return"):
            return "land"
        if typ == "goto_pod_scan":
            return "pod"
        if typ == "takeoff":
            return "takeoff"
        if typ == "eta" and dest_z is not None:
            return "land" if dest_z - float(self.grid.height_at(x, y)) < 2.0 else "pod"
        return "hover"

    def _route_time(self, start, route):
        return flight_time([np.asarray(start, float)] + list(route), self.mspec, kind="hover", grid=self.grid,
                           max_alt=self.max_alt)

    def eta(self, drone_id, dest_xyz):
        d = self.drones[drone_id]
        route = self.plan(d, np.asarray(dest_xyz, float))
        if route is None:
            return None
        kind = self._kind_for("eta", float(dest_xyz[2]), float(dest_xyz[0]), float(dest_xyz[1]))
        t = flight_time([d.pos.copy()] + route, self.mspec, kind=kind, from_ground=self._on_ground(d),
                        grid=self.grid, max_alt=self.max_alt)
        if d.battery - (self.fly_rate * t + self.scan_rate * SCAN_DWELL_S) < 0.15:
            return None
        return t

    # ---- commands
    def command(self, cmd):
        d = self.drones[cmd["drone_id"]]
        typ = cmd["type"]
        was_ground = self._on_ground(d)
        tl = self.timelines[d.id]
        now = self._now
        if typ == "scan":
            return super().command(cmd)
        old_job = d.job
        p, v, a, yaw, yrate = tl.nominal(now)
        d.pos = p.copy()
        ok = super().command(cmd)
        if typ == "hold":
            if not was_ground and (np.linalg.norm(v) > 0.05 or np.linalg.norm(a) > 0.05):
                tr = plan([p], now, self.mspec, v0=v, a0=a, kind="hover", yaw0=yaw, yaw_rate0=yrate, grid=self.grid,
                          max_alt=self.max_alt)
                tl.add(tr)
            d.traj = None
            return ok
        if ok is False or not d.route:
            d.traj = None
            return ok
        delay = 0.0
        wl = tl.winch.length(now)
        if wl > 0.05:                                     # reel the pod in before moving off
            T = max(2.0, self.mspec.winch_raise_s * min(1.0, wl / 8.0))
            pod = old_job.get("pod") if old_job and old_job.get("type") == "pod" else None
            tl.winch.add(now, wl, 0.0, T, "raise", pod=pod)
            delay = T
        if typ == "takeoff" and was_ground:
            delay += self.mspec.launch_stagger_s * self.launch_order.get(d.id, 0)
        kind = self._kind_for(typ)
        route = [np.asarray(w, float).copy() for w in d.route]
        if kind != "land" and len(route) >= 2 and not was_ground:
            # no dip: when the drone and its goal are both above the coordinator's cruise level, stay up
            # (higher than the checked cruise altitude is never closer to the terrain)
            floor = min(float(p[2]), float(route[-1][2]))
            for w in route[:-1]:
                w[2] = max(w[2], min(floor, self.max_alt))
            route = [w for k, w in enumerate(route) if k == len(route) - 1 or np.linalg.norm(w - route[k + 1]) > 1e-3]
        d.route = route
        tr = plan([p] + route, now, self.mspec, v0=v, a0=a, kind=kind,
                  from_ground=was_ground, grid=self.grid, max_alt=self.max_alt, delay=delay,
                  rpm0=tl.rpm_frac(now), yaw0=yaw, yaw_rate0=yrate)
        tl.add(tr)
        d.traj = tr
        return ok

    # ---- dynamics
    def step(self, dt, t):
        self._now = float(t)
        events = []
        for d in self.drones.values():
            tl = self.timelines[d.id]
            p, v, a, yaw, _ = tl.nominal(t)
            d.pos, d.yaw = p.copy(), (float(yaw) + 180.0) % 360.0 - 180.0
            j = d.job
            if j and j.get("type") == "pod" and not d.route and j["phase"] != "transit":
                d.battery -= self.fly_rate * dt
                j["left"] -= dt
                if j["phase"] == "lowering":
                    d.winch = tl.winch.length(t)
                    if j["left"] <= 0:
                        j["phase"], d.state, d.winch = "scanning", "scanning", j["drop"]
                        j["left"], j["t"] = j["dwell"] + j["left"], t
                elif j["phase"] == "scanning" and j["left"] <= 0:
                    events.append({"type": "scan_complete", "drone_id": d.id, "site": j["site"], "dwell": j["dwell"],
                                   "pose": [j["pod"][0], j["pod"][1], j["pod"][2], float(d.yaw)], "delivery": "pod"})
                    t_raise = t + j["left"]
                    tl.winch.add(t_raise, j["drop"], 0.0, self.pod_raise_s, "raise", pod=j["pod"])
                    j["phase"], d.state, j["t"] = "raising", "hovering", t
                    j["left"] = self.pod_raise_s + j["left"]
                    d.winch = tl.winch.length(t)
                elif j["phase"] == "raising":
                    d.winch = tl.winch.length(t)
                    if j["left"] <= 0:
                        d.winch, d.job = 0.0, None
                continue
            if d.state == "scanning":
                d.dwell_left -= dt
                d.battery -= self.scan_rate * dt
                if d.dwell_left <= 0:
                    d.state = "landed"
                    events.append({"type": "scan_complete", "drone_id": d.id, "site": d.job["site"], "pose": self._pose(d),
                                   "dwell": d.job.get("dwell", SCAN_DWELL_S)})
                    d.job = None
                continue
            if not d.route:
                d.battery -= (self.idle_rate if d.state in ("landed", "docked") else self.fly_rate) * dt
                continue
            d.battery -= self.fly_rate * dt
            tr = d.traj
            d.state = "descending" if (tr is not None and tr.in_descent(t)) else "flying"
            if d.pos[2] < self.grid.height_at(d.pos[0], d.pos[1]) - 0.01:
                events.append({"type": "collision", "drone_id": d.id})
            if tr is None or t >= tr.t_end - 1e-9:
                over = 0.0 if tr is None else t - tr.t_end
                if tr is not None:
                    d.pos = tr.p_end.copy()
                d.route = []
                typ = d.job["type"] if d.job else None
                if typ == "goto_land_scan":
                    d.state = "scanning"
                    d.dwell_left = d.job["dwell"] - over
                elif typ == "pod":
                    d.state = "hovering"
                    t_arr = t - over
                    d.job.update(phase="lowering", left=self.pod_lower_s - over, t=t)
                    tl.winch.add(t_arr, 0.0, d.job["drop"], self.pod_lower_s, "lower", pod=d.job["pod"])
                    d.winch = tl.winch.length(t)
                elif typ == "return":
                    d.state = "docked"
                    d.job = None
                else:
                    d.state = "hovering"
                    d.job = None
                    events.append({"type": "arrived", "drone_id": d.id, "pose": self._pose(d)})
                d.traj = None
        return events

    # ---- rendering / export
    def motion_sample(self, drone_id, t, nominal=False):
        return self.timelines[drone_id].sample(float(t), nominal=nominal)

    def export_motion(self):
        return {"version": 1, "spec": self.mspec.to_dict(), "seed": self.seed,
                "drones": {did: tl.to_dict() for did, tl in self.timelines.items()}}

    def save_motion(self, path):
        import json
        import os
        tmp = f"{path}.tmp"
        with open(tmp, "w") as f:
            json.dump(self.export_motion(), f, separators=(",", ":"), default=_json_default)
        os.replace(tmp, path)


_ACTIVE = []


def active_flight():
    """The most recently created SmoothFlight still alive (isaac.motion.Motion() finds the live mission this way)."""
    while _ACTIVE:
        fl = _ACTIVE[-1]()
        if fl is not None:
            return fl
        _ACTIVE.pop()
    return None


def _json_default(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    raise TypeError(type(o))


def load_motion(data):
    """{drone_id: MotionTimeline} from ``SmoothFlight.export_motion()`` output (dict or JSON path)."""
    if isinstance(data, str):
        import json
        with open(data) as f:
            data = json.load(f)
    sd = dict(data["spec"])
    for k in ("gust_gap_s", "gust_dur_s", "gust_mps"):
        sd[k] = tuple(sd[k])
    spec = MotionSpec(**sd)
    return {did: MotionTimeline.from_dict(d, spec) for did, d in data["drones"].items()}
