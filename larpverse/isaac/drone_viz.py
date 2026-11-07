"""Visual-only quadrotor (no rigid body, no collider), posed kinematically through xform ops.

    d = DroneViz(stage, "/World/Drones/d1")          # USD primitives
    d = DroneViz(stage, path, asset="auto")          # Isaac quadcopter USD, physics stripped; falls back
    d.set_pose(x, y, z, yaw_deg, tilt)               # tilt = forward pitch deg, or (roll, pitch)
    d.spin(dt, on=True)                              # rotor discs

``NullDrone`` has the same interface without pxr (``--dry-run``). pxr is imported lazily so this module
is importable outside Isaac.
"""
from __future__ import annotations

import math

COL = {
    "body": (0.16, 0.18, 0.21), "arm": (0.10, 0.11, 0.12), "rotor": (0.75, 0.78, 0.82),
    "blade": (0.05, 0.05, 0.06), "lidar": (0.55, 0.30, 0.85), "radar": (0.98, 0.55, 0.12),
    "mic": (0.15, 0.75, 0.72), "skid": (0.22, 0.22, 0.24),
}
ACCENT = {"d1": (0.20, 0.55, 0.95), "d2": (0.95, 0.80, 0.20), "d3": (0.30, 0.85, 0.45)}
ASSET_CANDIDATES = (
    "/Isaac/Robots/IsaacSim/Quadcopter/quadcopter.usd",     # 5.x vendor layout
    "/Isaac/Robots/Quadcopter/quadcopter.usd",              # 4.x layout
    "/Isaac/Robots/Crazyflie/cf2x.usd",
    "/Isaac/Robots/Bitcraze/Crazyflie/cf2x.usd",
)


def quat_ypr(yaw_deg, pitch_deg=0.0, roll_deg=0.0):
    """(w, x, y, z) for R = Rz(yaw) @ Ry(pitch) @ Rx(roll); positive pitch = nose (+x) down."""
    cy, sy = math.cos(math.radians(yaw_deg) / 2), math.sin(math.radians(yaw_deg) / 2)
    cp, sp = math.cos(math.radians(pitch_deg) / 2), math.sin(math.radians(pitch_deg) / 2)
    cr, sr = math.cos(math.radians(roll_deg) / 2), math.sin(math.radians(roll_deg) / 2)
    return (cy * cp * cr + sy * sp * sr, cy * cp * sr - sy * sp * cr,
            cy * sp * cr + sy * cp * sr, sy * cp * cr - cy * sp * sr)


def _tilt(tilt):
    if tilt is None:
        return 0.0, 0.0
    if isinstance(tilt, (tuple, list)):
        return float(tilt[0]), float(tilt[1])
    return 0.0, float(tilt)


class NullDrone:
    """Same interface as DroneViz, records poses only (dry-run / tests)."""

    def __init__(self, stage=None, path="", **_):
        self.path = path
        self.pose = None
        self.history = []
        self.rotor_angle = 0.0

    def set_pose(self, x, y, z, yaw_deg, tilt=None):
        roll, pitch = _tilt(tilt)
        self.pose = (float(x), float(y), float(z), float(yaw_deg), roll, pitch)
        if not self.history or self.history[-1][:3] != self.pose[:3]:
            self.history.append(self.pose)

    def spin(self, dt, on=True):
        self.rotor_angle = (self.rotor_angle + (2400.0 if on else 0.0) * dt) % 360.0


class DroneViz:
    ARM = 0.32        # hub-to-rotor distance (m) before scale

    def __init__(self, stage, path, did="d1", scale=1.6, asset=None, skid_offset=0.15):
        from pxr import Gf, UsdGeom
        self.Gf, self.UsdGeom = Gf, UsdGeom
        self.stage, self.path = stage, path
        self.rotor_ops = []
        self.rotor_angle = 0.0
        root = UsdGeom.Xform.Define(stage, path)
        xf = UsdGeom.Xformable(root)
        xf.ClearXformOpOrder()
        self.t_op = xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        self.q_op = xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
        self.t_op.Set(Gf.Vec3d(0, 0, -100))
        self.q_op.Set(Gf.Quatd(1, 0, 0, 0))
        model = UsdGeom.Xform.Define(stage, path + "/Model")      # skid bottom at the mission's pose z - 0.15
        mxf = UsdGeom.Xformable(model)
        mxf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0, 0, -skid_offset))
        mxf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(scale, scale, scale))
        self.model = path + "/Model"
        used_asset = False
        if asset:
            used_asset = self._reference_asset(asset)
        if not used_asset:
            self._primitives(did)
        self._payload()

    # ------------------------------------------------------------------ geometry helpers
    def _shape(self, kind, path, t=(0, 0, 0), s=(1, 1, 1), rgb=(1, 1, 1), rot_z=None, **attrs):
        Gf, UsdGeom = self.Gf, self.UsdGeom
        g = getattr(UsdGeom, kind).Define(self.stage, path)
        for k, v in attrs.items():
            getattr(g, "Create" + k + "Attr")(v)
        if kind == "Cube":
            g.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
        xf = UsdGeom.Xformable(g)
        xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, t)))
        if rot_z is not None:
            xf.AddRotateZOp().Set(float(rot_z))
        xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, s)))
        g.CreateDisplayColorAttr([Gf.Vec3f(*map(float, rgb))])
        return g

    def _primitives(self, did):
        m, A = self.model, self.ARM
        acc = ACCENT.get(did, (0.9, 0.9, 0.9))
        self._shape("Cube", m + "/Body", (0, 0, 0.26), (0.30, 0.22, 0.10), COL["body"], Size=1.0)
        self._shape("Cube", m + "/Stripe", (0.0, 0, 0.315), (0.31, 0.06, 0.012), acc, Size=1.0)
        for i in range(4):
            a = math.radians(45 + 90 * i)
            cx, cy = A * math.cos(a), A * math.sin(a)
            self._shape("Cube", f"{m}/Arm{i}", (cx / 2, cy / 2, 0.27), (A, 0.035, 0.025), COL["arm"],
                        rot_z=math.degrees(a), Size=1.0)
            self._shape("Cylinder", f"{m}/Motor{i}", (cx, cy, 0.30), (1, 1, 1), COL["arm"],
                        Radius=0.025, Height=0.05, Axis="Z")
            rotor = self.UsdGeom.Xform.Define(self.stage, f"{m}/Rotor{i}")
            rxf = self.UsdGeom.Xformable(rotor)
            rxf.AddTranslateOp(self.UsdGeom.XformOp.PrecisionDouble).Set(self.Gf.Vec3d(cx, cy, 0.33))
            self.rotor_ops.append((rxf.AddRotateZOp(), 1 if i % 2 else -1))
            disc = self._shape("Cylinder", f"{m}/Rotor{i}/Disc", (0, 0, 0), (1, 1, 1), COL["rotor"],
                               Radius=0.13, Height=0.004, Axis="Z")
            try:
                disc.CreateDisplayOpacityAttr([0.35])
            except Exception:
                pass
            self._shape("Cube", f"{m}/Rotor{i}/Blade", (0, 0, 0.004), (0.26, 0.025, 0.006), COL["blade"], Size=1.0)
        for side in (-1, 1):   # skids: two rails + four legs, rail bottom at z = 0
            self._shape("Cube", f"{m}/Skid{side + 1}", (0, side * 0.13, 0.01), (0.34, 0.02, 0.02), COL["skid"], Size=1.0)
            for fb in (-1, 1):
                self._shape("Cube", f"{m}/Leg{side + 1}{fb + 1}", (fb * 0.09, side * 0.12, 0.11),
                            (0.015, 0.015, 0.2), COL["skid"], Size=1.0)

    def _payload(self):
        m, A = self.model, self.ARM
        self._shape("Cylinder", m + "/LidarPuck", (0, 0, 0.345), (1, 1, 1), COL["lidar"], Radius=0.055, Height=0.07, Axis="Z")
        self._shape("Cube", m + "/RadarPanel", (0.17, 0, 0.2), (0.02, 0.16, 0.11), COL["radar"], Size=1.0)
        for i in range(4):
            a = math.radians(45 + 90 * i)
            self._shape("Sphere", f"{m}/Mic{i}", (0.8 * A * math.cos(a), 0.8 * A * math.sin(a), 0.2), (1, 1, 1),
                        COL["mic"], Radius=0.025)

    def _reference_asset(self, asset):
        """Reference Isaac's quadcopter under Model/Asset and strip every physics API. False on failure."""
        from pxr import Usd, UsdPhysics
        try:
            from isaac import compat
            urls = []
            if asset not in ("auto", True):
                urls.append(asset)
            root = compat.assets_root()
            if root:
                urls += [root + c for c in ASSET_CANDIDATES]
            import os

            def exists(u):
                return compat.url_exists(u) if "://" in u else os.path.exists(u)
            url = next((u for u in urls if u.endswith((".usd", ".usda", ".usdc")) and exists(u)), None)
            if url is None:
                print("[drone_viz] no quadcopter asset found; using primitives")
                return False
            prim = self.stage.DefinePrim(self.model + "/Asset", "Xform")
            prim.GetReferences().AddReference(url)
            if not prim.GetChildren():
                raise RuntimeError(f"{url} composed to an empty prim")
            for _ in range(8):                         # de-instance so the physics APIs can be edited
                inst = [p for p in Usd.PrimRange(prim) if p.IsInstance()]
                if not inst:
                    break
                for p in inst:
                    p.SetInstanceable(False)
            for p in Usd.PrimRange(prim):
                if p.HasAPI(UsdPhysics.RigidBodyAPI):
                    UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
                    p.RemoveAPI(UsdPhysics.RigidBodyAPI)
                if p.HasAPI(UsdPhysics.ArticulationRootAPI):
                    p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                    try:
                        from pxr import PhysxSchema
                        p.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
                    except Exception:
                        pass
                if p.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
                if p.IsA(UsdPhysics.Joint):
                    UsdPhysics.Joint(p).CreateJointEnabledAttr(False)
            print(f"[drone_viz] {self.path}: referenced {url} (physics stripped)")
            return True
        except Exception as e:   # noqa: BLE001
            print(f"[drone_viz] asset reference failed ({e}); using primitives")
            try:
                self.stage.RemovePrim(self.model + "/Asset")
            except Exception:
                pass
            return False

    # ------------------------------------------------------------------ runtime
    def set_pose(self, x, y, z, yaw_deg, tilt=None):
        roll, pitch = _tilt(tilt)
        w, qx, qy, qz = quat_ypr(yaw_deg, pitch, roll)
        self.t_op.Set(self.Gf.Vec3d(float(x), float(y), float(z)))
        self.q_op.Set(self.Gf.Quatd(w, self.Gf.Vec3d(qx, qy, qz)))

    def spin(self, dt, on=True):
        if not on:
            return
        self.rotor_angle = (self.rotor_angle + 2400.0 * dt) % 360.0
        for op, sgn in self.rotor_ops:
            op.Set(float(sgn * self.rotor_angle))
