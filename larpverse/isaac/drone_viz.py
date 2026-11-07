"""Visual-only drone (no rigid body, no collider), posed kinematically through xform ops.

    d = DroneViz(stage, "/World/Drones/d1")          # heavy-lift coaxial X8 from sim.platform (default)
    d = DroneViz(stage, path, model="small")         # the original ~1 m quad (scaled by ``scale``)
    d = DroneViz(stage, path, asset="auto")          # Isaac quadcopter USD, physics stripped; falls back
    d.set_pose(x, y, z, yaw_deg, tilt)               # tilt = forward pitch deg, or (roll, pitch)
    d.spin(dt, on=True)                              # rotor discs

``model="heavy"`` builds a FlyCart 30 / Agras T50-class airframe at true scale from ``sim.platform``
(``heavy_sar``: 2.2 m wheelbase, 4 arms x coaxial pairs = 8 rotors of 1.375 m, ~0.6 m landing gear) with the
SAR payload: LiDAR puck on the nose gimbal (drawn with its axis along +x; the sim mounts it pitched 45 deg), radar panel under the
belly facing down, loudspeaker horn at the tail, 8-mic ring, thermal camera.  ``scale`` is ignored for it
(dimensions are metres from the spec).  ``heavy_parts`` is the pure-python parts list (tested without pxr).

``NullDrone`` has the same interface without pxr (``--dry-run``). pxr is imported lazily so this module
is importable outside Isaac.
"""
from __future__ import annotations

import math

COL = {
    "body": (0.16, 0.18, 0.21), "arm": (0.10, 0.11, 0.12), "rotor": (0.75, 0.78, 0.82),
    "blade": (0.05, 0.05, 0.06), "lidar": (0.55, 0.30, 0.85), "radar": (0.98, 0.55, 0.12),
    "mic": (0.15, 0.75, 0.72), "skid": (0.22, 0.22, 0.24), "speaker": (0.85, 0.85, 0.82),
    "battery": (0.30, 0.32, 0.36), "thermal": (0.12, 0.12, 0.13),
}
MODELS = ("heavy", "small")
# --photoreal: real surface materials per part (UsdPreviewSurface; RTX and path tracing) instead of flat colours.
PBR = False
PROP_DISC = 0.06       # PBR: opacity of the spinning-prop blur disc (0 hides it; the blades still turn)
PBR_LOOK = {           # key -> (rgb, roughness, metallic, clearcoat); rgb None = the part's own colour
    "body": ((0.19, 0.20, 0.22), 0.32, 0.0, 0.8),       # painted composite shell
    "stripe": (None, 0.35, 0.0, 0.6),
    "carbon": ((0.022, 0.022, 0.026), 0.26, 0.0, 0.9),  # carbon-fibre arms and landing gear
    "metal": ((0.50, 0.51, 0.53), 0.30, 1.0, 0.0),      # motor bells
    "gimbal": ((0.09, 0.09, 0.10), 0.35, 0.6, 0.0),
    "plastic": ((0.03, 0.03, 0.035), 0.45, 0.0, 0.0),   # prop blades
    "radome": ((0.80, 0.80, 0.78), 0.55, 0.0, 0.2),     # radar panel cover
    "battery": ((0.10, 0.11, 0.12), 0.40, 0.0, 0.4),
    "lens": ((0.015, 0.015, 0.02), 0.06, 0.0, 1.0),     # LiDAR window, thermal lens
    "mic": ((0.04, 0.04, 0.045), 0.6, 0.0, 0.0),
    "speaker": ((0.70, 0.70, 0.68), 0.5, 0.0, 0.0),
    "disc": ((0.10, 0.10, 0.11), 0.5, 0.0, 0.0),
}
PART_KEY = (("Fuselage", "body"), ("Stripe", "stripe"), ("Battery", "battery"), ("Arm", "carbon"),
            ("Motor", "metal"), ("Skid", "carbon"), ("Leg", "carbon"), ("Brace", "carbon"), ("Gimbal", "gimbal"),
            ("LidarPuck", "lens"), ("Thermal", "lens"), ("RadarPanel", "radome"), ("SpeakerHorn", "speaker"),
            ("Mic", "mic"), ("Blade", "plastic"), ("Disc", "disc"))
GEAR_H = 0.60          # heavy: skid bottom to fuselage underside (m, estimate)


def heavy_parts(spec=None, did="d1"):
    """Parts of the heavy-lift model: [(kind, name, translate, scale, rgb, rot_z, attrs)] in metres, skid bottom
    at z = 0, +x forward.  Geometry from ``sim.platform`` (wheelbase, prop diameter, mic aperture, radar panel)."""
    if spec is None:
        from sim.platform import get_platform
        spec = get_platform("heavy_sar")
    acc = ACCENT.get(did, (0.9, 0.9, 0.9))
    P = []

    def add(kind, name, t, s=(1, 1, 1), rgb=(1, 1, 1), rot_z=None, **attrs):
        P.append((kind, name, tuple(map(float, t)), tuple(map(float, s)), rgb, rot_z, attrs))

    z0 = GEAR_H                                    # fuselage underside
    fl, fw, fh = 1.0, 0.70, 0.40
    add("Cube", "Fuselage", (0, 0, z0 + fh / 2), (fl, fw, fh), COL["body"], Size=1.0)
    add("Cube", "Stripe", (0, 0, z0 + fh + 0.006), (fl * 0.9, 0.10, 0.012), acc, Size=1.0)
    for sd in (-1, 1):                             # side-slung battery packs
        add("Cube", f"Battery{sd + 1}", (-0.05, sd * (fw / 2 + 0.06), z0 + 0.20), (0.50, 0.12, 0.26), COL["battery"], Size=1.0)
    R = spec.wheelbase_m / 2                       # hub-to-motor
    rp = spec.prop_d_m / 2
    za = z0 + fh + 0.10                            # arm height
    pairs = spec.n_rotors // 2 if spec.layout.startswith("coaxial") else spec.n_rotors
    for i in range(pairs):
        a = math.radians(45 + 360.0 / pairs * i)
        cx, cy = R * math.cos(a), R * math.sin(a)
        add("Cube", f"Arm{i}", (cx / 2, cy / 2, za), (R, 0.08, 0.08), COL["arm"], rot_z=math.degrees(a), Size=1.0)
        levels = ((0, za - 0.12), (1, za + 0.14)) if spec.layout.startswith("coaxial") else ((0, za + 0.10),)
        for lvl, zr in levels:
            add("Cylinder", f"Motor{i}_{lvl}", (cx, cy, zr), (1, 1, 1), COL["arm"],
                Radius=0.09, Height=0.10, Axis="Z")
            add("Rotor", f"Rotor{i}_{lvl}", (cx, cy, zr + (0.06 if lvl else -0.06)), (1, 1, 1), COL["rotor"],
                None, radius=rp, sign=(1 if (i + lvl) % 2 else -1))
    for sd in (-1, 1):                             # landing gear: two skids, two legs each, cross bars
        y = sd * 0.70
        add("Cube", f"Skid{sd + 1}", (0, y, 0.03), (1.60, 0.06, 0.06), COL["skid"], Size=1.0)
        for fb in (-1, 1):
            add("Cube", f"Leg{sd + 1}{fb + 1}", (fb * 0.45, y, z0 / 2), (0.05, 0.05, z0), COL["skid"], Size=1.0)
        add("Cube", f"Brace{sd + 1}", (0, sd * 0.52, z0 - 0.03), (0.95, 0.36, 0.04), COL["skid"], Size=1.0)
    # payload
    gx, gz = fl / 2 + 0.08, z0 - 0.10              # nose gimbal
    add("Cube", "Gimbal", (gx, 0, gz + 0.06), (0.14, 0.14, 0.12), COL["arm"], Size=1.0)
    add("Cylinder", "LidarPuck", (gx + 0.05, 0, gz - 0.06), (1, 1, 1), COL["lidar"], Radius=0.07, Height=0.10, Axis="X")
    add("Cube", "Thermal", (gx + 0.02, 0.13, gz - 0.02), (0.10, 0.08, 0.08), COL["thermal"], Size=1.0)
    rw, rh = spec.radar.aperture_m
    add("Cube", "RadarPanel", (-0.05, 0, z0 - 0.05), (rh, rw, 0.05), COL["radar"], Size=1.0)
    if spec.speaker is not None:
        add("Cone", "SpeakerHorn", (-fl / 2 - 0.02, 0, z0 - 0.08), (1, 1, 1), COL["speaker"], Radius=0.13, Height=0.20, Axis="Z")
    rm = spec.mics.aperture_m / 2
    for k in range(spec.mics.n_mics):
        a = math.radians(22.5 + 360.0 / spec.mics.n_mics * k)
        add("Sphere", f"Mic{k}", (rm * math.cos(a), rm * math.sin(a), z0 + fh + 0.03), (1, 1, 1), COL["mic"], Radius=0.03)
    return P


def model_extent(parts):
    """(x_extent_m, y_extent_m, z_min, z_max) of a parts list, rotor discs included (rotated cubes approximated
    by their half-diagonal)."""
    ex = ey = 0.0
    zmin, zmax = 1e9, -1e9
    for kind, _, t, s, _, rz, attrs in parts:
        if kind == "Rotor":
            hx = hy = attrs["radius"]
        elif kind == "Cube":
            hx, hy = (s[0] / 2, s[1] / 2) if rz is None else (math.hypot(s[0], s[1]) / 2,) * 2
        else:
            hx = hy = attrs.get("Radius", 0.0)
        ex, ey = max(ex, abs(t[0]) + hx), max(ey, abs(t[1]) + hy)
        h = s[2] / 2 if kind == "Cube" else attrs.get("Height", 2 * attrs.get("Radius", 0.0)) / 2
        zmin, zmax = min(zmin, t[2] - h), max(zmax, t[2] + h)
    return 2 * ex, 2 * ey, zmin, zmax
ACCENT = {"d1": (0.20, 0.55, 0.95), "d2": (0.95, 0.80, 0.20), "d3": (0.30, 0.85, 0.45)}
ASSET_CANDIDATES = (
    "/Isaac/Robots/IsaacSim/Quadcopter/quadcopter.usd",     # 5.x/6.0 layout (exists on 6.0: assets_found.json)
    "/Isaac/Robots/Quadcopter/quadcopter.usd",              # 4.x layout
    "/Isaac/Robots/Bitcraze/Crazyflie/cf2x.usd",            # exists on 6.0 (assets_found.json); tiny, scale up
    "/Isaac/Robots/Crazyflie/cf2x.usd",
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

    def __init__(self, stage, path, did="d1", scale=1.6, asset=None, skid_offset=0.15, model="heavy", spec=None):
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
        heavy = model == "heavy" and not asset
        if heavy:
            scale = 1.0                                             # true metres from sim.platform
        mxf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(scale, scale, scale))
        self.model = path + "/Model"
        self.spin_rate = 1200.0 if heavy else 2400.0               # deg/s, visual only
        if heavy:
            self._build_parts(heavy_parts(spec, did))
            return
        used_asset = False
        if asset:
            used_asset = self._reference_asset(asset)
        if not used_asset:
            self._primitives(did)
        self._payload()

    def _build_parts(self, parts):
        Gf, UsdGeom, m = self.Gf, self.UsdGeom, self.model
        for kind, name, t, s, rgb, rot_z, attrs in parts:
            if kind != "Rotor":
                self._shape(kind, f"{m}/{name}", t, s, rgb, rot_z, **attrs)
                continue
            rotor = UsdGeom.Xform.Define(self.stage, f"{m}/{name}")
            rxf = UsdGeom.Xformable(rotor)
            rxf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*t))
            self.rotor_ops.append((rxf.AddRotateZOp(), attrs["sign"]))
            disc = self._shape("Cylinder", f"{m}/{name}/Disc", (0, 0, 0), (1, 1, 1), rgb,
                               Radius=attrs["radius"], Height=0.006, Axis="Z")
            try:
                disc.CreateDisplayOpacityAttr([0.25])
            except Exception:
                pass
            self._shape("Cube", f"{m}/{name}/Blade", (0, 0, 0.006), (2 * attrs["radius"], 0.09, 0.012),
                        COL["blade"], Size=1.0)
            if PBR and PROP_DISC <= 0:
                UsdGeom.Imageable(disc).MakeInvisible()
        if PBR:
            self._materials()

    def _materials(self):
        """--photoreal: bind a real surface material to every part, by part name (PART_KEY)."""
        from pxr import Sdf, Usd, UsdShade
        did = self.path.rstrip("/").split("/")[-1]
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(self.model)):
            if not prim.IsA(self.UsdGeom.Gprim):
                continue
            key = next((k for pre, k in PART_KEY if prim.GetName().startswith(pre)), None)
            if key is None:
                continue
            rgb, rough, metal, coat = PBR_LOOK[key]
            if rgb is None:
                c = self.UsdGeom.Gprim(prim).GetDisplayColorAttr().Get()
                rgb = tuple(c[0]) if c else (0.8, 0.8, 0.8)
            mpath = f"/World/Looks/Drone/{key}" + (f"_{did}" if key == "stripe" else "")
            mat = UsdShade.Material.Get(self.stage, mpath)
            if not mat:
                mat = UsdShade.Material.Define(self.stage, mpath)
                sh = UsdShade.Shader.Define(self.stage, mpath + "/Surface")
                sh.CreateIdAttr("UsdPreviewSurface")
                sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(self.Gf.Vec3f(*map(float, rgb)))
                sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(rough))
                sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metal))
                sh.CreateInput("clearcoat", Sdf.ValueTypeNames.Float).Set(float(coat))
                sh.CreateInput("clearcoatRoughness", Sdf.ValueTypeNames.Float).Set(0.08)
                if key == "disc":
                    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(max(PROP_DISC, 0.0)))
                mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)

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
            mpu = compat.meters_per_unit(url)        # references don't convert units (cm assets come in 100x)
            if abs(mpu - 1.0) > 1e-6:
                from pxr import Gf, UsdGeom
                UsdGeom.Xformable(prim).AddScaleOp(UsdGeom.XformOp.PrecisionDouble, "units").Set(Gf.Vec3d(mpu, mpu, mpu))
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
                    try:                               # what look.py does on 6.0.1 (strip_physics)
                        from pxr import Sdf
                        p.CreateAttribute("physxArticulation:articulationEnabled", Sdf.ValueTypeNames.Bool).Set(False)
                    except Exception:
                        pass
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
        self.rotor_angle = (self.rotor_angle + self.spin_rate * dt) % 360.0
        for op, sgn in self.rotor_ops:
            op.Set(float(sgn * self.rotor_angle))
