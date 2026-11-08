"""Render-time motion for Isaac: sample the flight trajectories at the RENDER time, every frame.

The mission steps at dt = 1 s; the renderer draws 30-60 frames per mission second (more with --rec-speed
< 1, fewer per sim second with time-lapse).  Interpolating between the last two mission states (the old
``interp_drones``) draws straight segments with a velocity jump at every step.  Instead, each rendered frame
asks the drone's trajectory (sim/flight_motion.py) for its pose at the frame's mission time:

    from isaac.motion import MotionView, SpringFollow
    view = MotionView.attach(mission)                    # live: mission.flight is a SmoothFlight
    view = MotionView.from_file(".../motion.json")       # --replay: SmoothFlight.save_motion() output
    t_r = view.t_render(prev, cur, u)                    # frame time inside [prev.t, cur.t]
    poses = view.interp_drones(prev, cur, u)             # same tuples as run_mission.interp_drones
    view.apply(drone_viz, did, t_r, rdt)                 # set_pose with roll/pitch + rotor spin from RPM
    pod = view.pose_at(did, t_r).pod                     # hanging / resting pod position (or None)

``SpringFollow`` (critically damped spring) and ``OneEuro`` smooth a director camera's look-at target and
position; both are frame-rate independent.

No omni import is needed for any of this; ``HAVE_OMNI`` is only informational and ``UsdPoseWriter`` (direct
xform-op writes) is a no-op without Isaac.
"""
from __future__ import annotations

import math

import numpy as np

try:                                   # informational only; nothing below requires Isaac
    import omni  # noqa: F401
    HAVE_OMNI = True
except Exception:                      # noqa: BLE001
    HAVE_OMNI = False

from sim.flight_motion import load_motion

VISUAL_SPIN_DPS = 1200.0      # rotor blade visual rate at hover RPM (true ~6000 deg/s strobes at 30 fps)


class MotionView:
    """Pose lookup by (drone, mission time) over SmoothFlight timelines (live) or an exported motion.json."""

    def __init__(self, timelines, source="live"):
        self.timelines = timelines            # {did: sim.flight_motion.MotionTimeline}
        self.source = source
        self._rotor_ang = {}                  # did -> (n,) visual blade angles (deg)
        self._last = {}

    # ---- construction
    @classmethod
    def attach(cls, mission_or_flight):
        """MotionView over a live mission (``mission.flight``) or a flight backend; None if it is not smooth."""
        fl = getattr(mission_or_flight, "flight", mission_or_flight)
        tls = getattr(fl, "timelines", None)
        return cls(tls, "live") if tls else None

    @classmethod
    def from_file(cls, path):
        import os
        if not path or not os.path.exists(path):
            return None
        return cls(load_motion(path), "file")

    def __contains__(self, did):
        return did in self.timelines

    # ---- timing
    @staticmethod
    def t_render(prev, cur, u):
        """Mission time of a rendered frame: prev.t + u (cur.t - prev.t), u in [0, 1] as run_mission computes it."""
        t0, t1 = float(prev.get("t", 0.0)), float(cur.get("t", 0.0))
        return t0 + (t1 - t0) * min(max(float(u), 0.0), 1.0)

    # ---- sampling
    def pose_at(self, did, t, nominal=False):
        """sim.flight_motion.Sample: pos, vel, acc, yaw/roll/pitch (deg), quat (w,x,y,z), tilt, rotor_rpm,
        winch_len, pod, airborne, phase."""
        return self.timelines[did].sample(float(t), nominal=nominal)

    def poses_at(self, t):
        return {did: self.pose_at(did, t) for did in self.timelines}

    def interp_drones(self, prev, cur, u):
        """Drop-in for run_mission.interp_drones: {id: (x, y, z, yaw, state, vx, vy)} at the frame time.
        vx, vy are per-step displacements (velocity x the step length), as the old function returned."""
        t = self.t_render(prev, cur, u)
        dt_step = max(float(cur.get("t", 0.0)) - float(prev.get("t", 0.0)), 1e-3)
        out = {}
        for d in cur.get("drones", []):
            did = d["id"]
            if did not in self.timelines:
                continue
            s = self.pose_at(did, t)
            out[did] = (float(s.pos[0]), float(s.pos[1]), float(s.pos[2]), float(s.yaw), d.get("state", ""),
                        float(s.vel[0]) * dt_step, float(s.vel[1]) * dt_step)
        return out

    def attitude(self, did, t):
        """(roll_deg, pitch_deg) in isaac/drone_viz.set_pose's tilt convention (R = Rz(yaw) Ry(pitch) Rx(roll))."""
        s = self.pose_at(did, t)
        return s.roll, s.pitch

    # ---- applying to the stage
    def rotor_angles(self, did, s, rdt):
        """Advance and return the per-rotor visual blade angles (deg) from the per-rotor RPM."""
        hov = max(self.timelines[did].spec.hover_rpm, 1.0)
        ang = self._rotor_ang.get(did)
        if ang is None:
            ang = np.zeros(len(s.rotor_rpm))
        from sim.flight_motion import rotor_layout
        signs = np.array([sg for _, sg, _ in rotor_layout(len(s.rotor_rpm))], float)
        ang = (ang + signs * VISUAL_SPIN_DPS * (np.asarray(s.rotor_rpm) / hov) * float(rdt)) % 360.0
        self._rotor_ang[did] = ang
        return ang

    @staticmethod
    def pod_sway(s):
        """(roll, pitch) deg of the pod cable from the plumb line, and the pod's rest z (None while hanging), for
        DroneViz.set_state(sway_deg=..., ground_z=...).  R = Rx(roll) Ry(pitch): +pitch swings the pod to -x."""
        if s.pod is None or s.winch_len <= 1e-3:
            return (0.0, 0.0), None
        L = max(s.winch_len, 0.3)
        hang = np.asarray(s.pos, float) - np.array([0.0, 0.0, s.winch_len])
        dx, dy = float(s.pod[0] - hang[0]), float(s.pod[1] - hang[1])
        resting = float(s.pod[2]) > float(hang[2]) + 1e-3
        if resting:
            return (0.0, 0.0), float(s.pod[2])
        return (math.degrees(math.atan2(dy, L)), math.degrees(math.atan2(-dx, L))), None

    def apply(self, dr, did, t, rdt):
        """Pose a DroneViz / NullDrone at mission time t: translation, full attitude, per-rotor RPM, winch + pod
        sway.  Uses ``set_state`` when the drone has it (isaac/drone_viz), else ``set_pose`` + ``spin``.
        Returns the Sample (use .pod for the pod, .phase for labels)."""
        s = self.pose_at(did, t)
        if hasattr(dr, "set_state"):
            sway, gz = self.pod_sway(s)
            dr.set_state([float(v) for v in s.pos], tuple(float(v) for v in s.quat), [float(r) for r in s.rotor_rpm],
                         float(s.winch_len), float(rdt), sway_deg=sway, ground_z=gz)
            return s
        dr.set_pose(float(s.pos[0]), float(s.pos[1]), float(s.pos[2]), float(s.yaw), (float(s.roll), float(s.pitch)))
        if hasattr(dr, "spin"):
            frac = float(np.mean(s.rotor_rpm)) / max(self.timelines[did].spec.hover_rpm, 1.0)
            dr.spin(rdt * max(frac, 0.0), on=frac > 0.02)
        return s


class Motion:
    """Adapter for isaac/run_mission.py ``PoseTrack`` (``isaac.motion.Motion()``, ``ingest(state)``,
    ``pose_at(did, t) -> dict``).  Finds the live mission's SmoothFlight by itself (``sim.flight_motion.
    active_flight()``); for ``--replay`` call ``attach(<replay dir>/motion.json)`` (or set LARPVERSE_MOTION_JSON).
    ``pose_at`` raises KeyError when no trajectory is known for the drone (a KinematicFlight mission), so
    PoseTrack falls back to its Hermite interpolation."""

    def __init__(self, source=None):
        self.view = None
        self.states = {}
        if source is not None:
            self.attach(source)

    def attach(self, source):
        """source: a mission / flight backend with timelines, a MotionView, or a motion.json path."""
        if isinstance(source, MotionView):
            self.view = source
        elif isinstance(source, str):
            self.view = MotionView.from_file(source)
        else:
            self.view = MotionView.attach(source)
        return self.view is not None

    def _ensure(self):
        if self.view is None:
            import os
            from sim.flight_motion import active_flight
            fl = active_flight()
            if fl is not None:
                self.view = MotionView.attach(fl)
            elif os.environ.get("LARPVERSE_MOTION_JSON"):
                self.view = MotionView.from_file(os.environ["LARPVERSE_MOTION_JSON"])
        return self.view

    def ingest(self, state):
        self._ensure()
        for d in state.get("drones", []):
            self.states[d["id"]] = d.get("state", "")

    def pose_at(self, did, t):
        v = self._ensure()
        if v is None or did not in v:
            raise KeyError(f"no flight-motion trajectory for {did}")
        s = v.pose_at(did, t)
        sway, gz = MotionView.pod_sway(s)
        ev = v.timelines[did].winch._event(float(t))
        if gz is None and s.winch_len > 1e-3 and ev is not None and ev.get("pod") is not None:
            gz = float(ev["pod"][2])                 # rest height for the pod under the drone (DroneViz clamps)
        out = {"xyz": [float(x) for x in s.pos], "quat": [float(q) for q in s.quat],
               "rotor_rpm": [float(r) for r in s.rotor_rpm], "winch_len": float(s.winch_len),
               "tilt": (float(s.roll), float(s.pitch)), "vel": [float(x) for x in s.vel], "yaw": float(s.yaw),
               "sway_deg": sway, "pod": None if s.pod is None else [float(x) for x in s.pod],
               "phase": s.phase, "airborne": bool(s.airborne)}
        if gz is not None:
            out["ground_z"] = gz
        if did in self.states:
            out["state"] = self.states[did]
        return out


# --------------------------------------------------------------------------- camera smoothing
class SpringFollow:
    """Critically damped spring towards a moving target (exact per-step solution, stable at any dt).
    ``halflife`` ~ seconds to close half the gap; 0.35-0.8 s suits a director camera."""

    def __init__(self, x0, halflife=0.5):
        self.x = np.asarray(x0, float).copy()
        self.v = np.zeros_like(self.x)
        self.halflife = float(halflife)

    def update(self, target, dt):
        target = np.asarray(target, float)
        w = 2.0 * 0.6931471805599453 / max(self.halflife, 1e-4)        # omega for the requested half-life
        x0, v0 = self.x - target, self.v
        e = math.exp(-w * dt)
        j0 = v0 + w * x0
        self.x = target + (x0 + j0 * dt) * e
        self.v = (v0 - j0 * w * dt) * e
        return self.x.copy()


class OneEuro:
    """One-euro filter (Casiez et al., CHI 2012): smooths jitter when slow, low lag when fast."""

    def __init__(self, min_cutoff=0.6, beta=0.4, d_cutoff=1.0):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.x = None
        self.dx = None

    @staticmethod
    def _alpha(cutoff, dt):
        r = 2 * math.pi * cutoff * dt
        return r / (r + 1)

    def update(self, x, dt):
        x = np.asarray(x, float)
        if self.x is None:
            self.x, self.dx = x.copy(), np.zeros_like(x)
            return x.copy()
        dt = max(dt, 1e-4)
        dx = (x - self.x) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        self.dx = a_d * dx + (1 - a_d) * self.dx
        cutoff = self.min_cutoff + self.beta * float(np.linalg.norm(self.dx))
        a = self._alpha(cutoff, dt)
        self.x = a * x + (1 - a) * self.x
        return self.x.copy()


class SmoothCamera:
    """Director-camera helper: springs on the eye and the look-at target, so a new focus (another drone, an
    event) is reached with a smooth glide, never a cut.  ``update(eye_target, look_target, dt) -> (eye, look)``."""

    def __init__(self, eye, look, eye_halflife=0.9, look_halflife=0.45):
        self.eye = SpringFollow(eye, eye_halflife)
        self.look = SpringFollow(look, look_halflife)

    def update(self, eye_target, look_target, dt):
        return self.eye.update(eye_target, dt), self.look.update(look_target, dt)


class UsdPoseWriter:
    """Optional: write a drone pose straight to a prim's translate / orient ops.  No-op without Isaac."""

    def __init__(self, stage=None, path=None):
        self.ok = False
        if not HAVE_OMNI or stage is None or path is None:
            return
        try:
            from pxr import Gf, UsdGeom
            xf = UsdGeom.Xformable(stage.GetPrimAtPath(path))
            xf.ClearXformOpOrder()
            self.t_op = xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
            self.q_op = xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
            self.Gf, self.ok = Gf, True
        except Exception:                         # noqa: BLE001
            self.ok = False

    def write(self, s):
        if not self.ok:
            return False
        w, x, y, z = (float(v) for v in s.quat)
        self.t_op.Set(self.Gf.Vec3d(*(float(v) for v in s.pos)))
        self.q_op.Set(self.Gf.Quatd(w, self.Gf.Vec3d(x, y, z)))
        return True
