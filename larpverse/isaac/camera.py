"""Director camera: a UsdGeom.Camera set as the viewport camera that follows the active pile / drones with a
slow orbit. ``--camera free`` places it once and leaves it for the user to fly.

The pose is re-applied EVERY frame, and mirrored onto ``mirrors`` (e.g. /OmniverseKit_Persp, which is what the
Brev /viewer livestream shows, and the Isaac Lab recording camera /World/Cams/rec). That is the team's verified
workaround for the black viewport on Isaac Sim 6.0.1, where a camera posed once rendered black
(docs/ISAAC_VERIFIED_APIS.md, "Black viewport"). Switching the viewport's camera_path is tried too, but is
unverified on 6.x.

The pose maths is numpy (``look_at``) so ``--dry-run`` exercises it without pxr; ``stage=None`` skips USD.
"""
from __future__ import annotations

import math

import numpy as np


def look_at(eye, target, up=(0, 0, 1)):
    """4x4 camera-to-world matrix in USD's row-vector convention (camera looks down -Z, +Y up)."""
    eye, target, up = (np.asarray(v, float) for v in (eye, target, up))
    z = eye - target
    z /= np.linalg.norm(z) or 1.0
    x = np.cross(up, z)
    if np.linalg.norm(x) < 1e-6:
        x = np.array([1.0, 0, 0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    M = np.eye(4)
    M[0, :3], M[1, :3], M[2, :3], M[3, :3] = x, y, z, eye
    return M


class DirectorCamera:
    def __init__(self, stage=None, path="/World/DirectorCam", mode="director", start=(0.0, -40.0, 3.0),
                 orbit_period=150.0, dist=40.0, height=24.0, mirrors=(), switch_viewport=True):
        self.mode, self.path = mode, path
        self.target = np.array(start, float)
        self.dist, self.height = dist, height
        self.omega = 2 * math.pi / orbit_period
        self.az = math.radians(-100.0)
        self.eye = self.target + [self.dist * math.cos(self.az), self.dist * math.sin(self.az), self.height]
        self.op = None
        self.updates = 0
        self.is_viewport = False
        self.mirrors = []
        if stage is not None:
            from pxr import Gf, UsdGeom
            self.Gf = Gf
            cam = UsdGeom.Camera.Define(stage, path)
            cam.CreateFocalLengthAttr(18.0)
            cam.CreateHorizontalApertureAttr(20.955)
            cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 3000.0))
            xf = UsdGeom.Xformable(cam)
            xf.ClearXformOpOrder()
            self.op = xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble)
            from isaac import compat
            self.mirrors = [compat.PoseWriter(p) for p in mirrors]
            self._apply()
            if switch_viewport:
                self.is_viewport = compat.set_viewport_camera(path)
                if not self.is_viewport:
                    print(f"[camera] viewport not switched to {path} (headless, or API drift); "
                          f"driving {', '.join(mirrors) or 'nothing else'} every frame instead")

    def quat_wxyz(self, M=None):
        from isaac.compat import mat_to_quat_wxyz
        M = look_at(self.eye, self.target) if M is None else M
        return mat_to_quat_wxyz(M[:3, :3].T)          # rows are the camera axes -> column convention

    def _apply(self):
        M = look_at(self.eye, self.target)
        if self.op is not None:
            self.op.Set(self.Gf.Matrix4d(*M.flatten().tolist()))
            if self.mirrors:
                q = self.quat_wxyz(M)
                for w in self.mirrors:
                    w.set(self.eye, q, M.tolist())
        self.updates += 1
        return M

    def update(self, dt, focus_xyz=None, spread=0.0):
        """Ease towards ``focus_xyz`` and orbit; ``spread`` (m) pulls the camera back to fit the drones."""
        if self.mode == "free":
            return None
        return self.update_pose(dt, focus_xyz, spread)

    def update_pose(self, dt, focus_xyz=None, spread=0.0):
        if focus_xyz is not None:
            k = 1.0 - math.exp(-dt / 2.5)           # ~2.5 s time constant: slow, cinematic
            self.target += (np.asarray(focus_xyz, float) - self.target) * k
        self.az += self.omega * dt
        d = self.dist + 0.8 * spread
        want = self.target + [d * math.cos(self.az), d * math.sin(self.az), self.height + 0.4 * spread]
        self.eye += (want - self.eye) * (1.0 - math.exp(-dt / 1.5))
        return self._apply()


def focus_from_state(state, pile_xy, active_pile, poses):
    """Focus point and drone spread: the active pile blended with the deployed drones' centroid."""
    pts = [p[:3] for p in poses]
    if active_pile and active_pile in pile_xy:
        px, py = pile_xy[active_pile][:2]
        near = [p for p in pts if math.hypot(p[0] - px, p[1] - py) < 25]
        if near:
            c = np.mean(near, 0)
            f = np.array([0.6 * px + 0.4 * c[0], 0.6 * py + 0.4 * c[1], 2.0 + 0.3 * max(0.0, c[2] - 2.0)])
        else:
            f = np.array([px, py, 2.0])
        spread = max((math.hypot(p[0] - f[0], p[1] - f[1]) for p in near), default=0.0)
        return f, min(spread, 25.0)
    if pts:
        c = np.mean(pts, 0)
        return np.array([c[0], c[1], max(1.0, c[2] * 0.5)]), min(max(math.hypot(p[0] - c[0], p[1] - c[1]) for p in pts), 25.0)
    return None, 0.0


# ------------------------------------------------------------------------------ story camera (single take)
STORY_PITCH = 36.0          # deg below the horizon
STORY_FOV_FIT = math.tan(math.radians(25.0))   # half-FOV used to fit the action (18 mm lens: hfov ~60 deg)
STORY_R = (12.0, 95.0)      # m: framed radius limits (close enough to see drones, wide enough for the approach)
STORY_NEAR = 55.0           # m: things this close to the active zone share its frame
STORY_ORBIT = 2 * math.pi / 300.0   # rad per render second: a slow drift, ~70 deg over a one-minute take
STORY_OMEGA = 1.1           # 1/s: critically damped spring on focus and distance (no snaps, no overshoot)
AWAY = {"return", "battery_swap", "done"}


def _poly_centre(z):
    poly = z.get("polygon")
    if poly:
        P = np.asarray(poly, float)[:, :2]
        return P.mean(0)
    c = z.get("centre") or z.get("center") or z.get("xy")
    return np.asarray(c, float)[:2] if c is not None else None


def story_focus(state, pile_xy, active_pile, poses, memo=None):
    """What the judges must see right now -> (focus xyz, framed radius m), from the mission state only.

    The active zone anchors the shot. Drones near it, new or suggested zones near it and the dispatch target
    share the frame. Drones flying in (deploy, dispatch) are kept in frame, so the shot opens wide and closes
    in as they arrive; drones flying home to swap batteries are let go, and the shot stays on the zones
    (where the operator's change appears)."""
    pts = [np.asarray(p[:2], float) for p in poses]
    phase = str(state.get("phase") or "")
    memo = {} if memo is None else memo
    anchor = None
    if phase in AWAY and memo.get("anchor") is not None:
        anchor = memo["anchor"]                         # drones gone home: stay where the story is
    elif active_pile and active_pile in pile_xy:
        anchor = np.asarray(pile_xy[active_pile][:2], float)
    if phase == "dispatch":
        for d in reversed(state.get("dispatch") or []):
            if d.get("to_pile") and d["to_pile"] in pile_xy:
                anchor = np.asarray(pile_xy[d["to_pile"]][:2], float)
                break
    if anchor is not None and phase not in AWAY:
        memo["anchor"] = anchor
    if anchor is None:
        if not pts:
            return None, 0.0
        c = np.mean(pts, 0)
        return np.array([c[0], c[1], 3.0]), STORY_R[0]
    keep = [anchor]
    for z in list(state.get("zones") or []) + list(state.get("zone_suggestions") or []):
        if not isinstance(z, dict) or str(z.get("status", "")) in ("searched_no_ping",):
            continue
        c = _poly_centre(z)
        if c is not None and np.hypot(*(c - anchor)) < STORY_NEAR:
            keep.append(c)
    near = [p for p in pts if np.hypot(*(p - anchor)) < STORY_NEAR]
    keep += near
    if phase not in AWAY and len(near) < len(pts):
        keep += pts                                     # inbound: frame the whole approach
    K = np.asarray(keep)
    lo, hi = K.min(0), K.max(0)
    c = 0.5 * (lo + hi)
    r = float(np.max(np.hypot(K[:, 0] - c[0], K[:, 1] - c[1]))) + 7.0
    m = state.get("mission") if isinstance(state.get("mission"), dict) else {}
    if m.get("stage") == "listen":
        r = max(r, 20.0)                                # the sound domes around the active zone
    return np.array([c[0], c[1], 3.0]), float(np.clip(r, *STORY_R))


class _Spring:
    """Critically damped spring: smooth start and stop, never overshoots (a camera operator's hands)."""
    def __init__(self, x, omega):
        self.x, self.v, self.w = np.array(x, float), np.zeros_like(np.array(x, float)), omega

    def step(self, goal, dt):
        w, goal = self.w, np.asarray(goal, float)
        e = math.exp(-w * dt)
        d = self.x - goal
        tmp = (self.v + w * d) * dt
        self.x = goal + (d + tmp) * e
        self.v = (self.v - w * tmp) * e
        return self.x


class StoryCamera(DirectorCamera):
    """``--camera story``: one continuous, state-driven shot for the judges' take. It frames whatever the
    mission is doing (approach, survey, pods, listening, the operator's new zone, the dispatch) with a slow
    orbit and spring-smoothed moves, so the stream needs no cuts."""

    def __init__(self, stage=None, path="/World/DirectorCam", start=(0.0, -40.0, 3.0), az_deg=-115.0, **kw):
        super().__init__(stage, path=path, mode="story", start=start, **kw)
        self.az = math.radians(az_deg)
        self.focus = _Spring(np.asarray(start, float), STORY_OMEGA)
        self.radius = _Spring(np.array([40.0]), STORY_OMEGA * 0.8)
        self._place(float(self.radius.x[0]))

    def _place(self, r):
        d = r / STORY_FOV_FIT
        p = math.radians(STORY_PITCH)
        self.target = self.focus.x.copy()
        self.eye = self.target + [d * math.cos(p) * math.cos(self.az), d * math.cos(p) * math.sin(self.az), d * math.sin(p)]
        return self._apply()

    def update(self, dt, focus_xyz=None, spread=0.0):
        dt = float(min(max(dt, 0.0), 0.25))
        if focus_xyz is not None:
            self.focus.step(focus_xyz, dt)
            self.radius.step([max(float(spread), STORY_R[0])], dt)
        self.az += STORY_ORBIT * dt
        return self._place(float(self.radius.x[0]))
