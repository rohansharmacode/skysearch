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
