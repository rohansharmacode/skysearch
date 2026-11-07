"""Build the rubble stage (grid | weiguan | mesh site) from sim.scenario (pxr only, so it is stable across Isaac versions).

Every rubble piece and the terrain get PhysX collision, so the LiDAR raycasts hit real Isaac geometry.
Drones are visual-only (no rigid body, no collider) and are posed kinematically each step.

Materials: by default every gprim gets a real ``UsdPreviewSurface`` material (pure UsdShade), so the look
does not depend on whether RTX honours bare ``displayColor``. ``materials="none"`` leaves displayColor only
(diagnosis); ``--look pretty`` (isaac/look.py) swaps in NVIDIA MDL concrete on top of this.
Lighting defaults are the values the team measured on the L40S (dome 350 cool + low warm sun 1800; dome 1200
+ overhead sun 3000 washed everything to white): docs/ISAAC_VERIFIED_APIS.md, "Lighting".
"""
import math

import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

from isaac.site import base_xy, bounds, victim_z
from sim.scenario import rot_matrix

PALETTE = {
    "concrete": [(0.56, 0.55, 0.52), (0.49, 0.47, 0.45), (0.62, 0.59, 0.55), (0.43, 0.41, 0.39)],
    "brick": [(0.53, 0.38, 0.30), (0.48, 0.31, 0.25)],
}
BRIGHT = (0.93, 0.99, 1.05)          # per-piece brightness buckets (one shared material per bucket)


def _xform(prim_or_schema, t=(0, 0, 0), quat=None, scale=None):
    xf = UsdGeom.Xformable(prim_or_schema)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, t)))
    if quat is not None:
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat)
    if scale is not None:
        xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, scale)))
    return xf


def _color(gprim, rgb):
    UsdGeom.Gprim(gprim).CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, rgb))]))


def _collide(prim, mesh=False):
    UsdPhysics.CollisionAPI.Apply(prim)
    if mesh:
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("none")


# ---------------------------------------------------------------------------- materials (UsdShade only)
class Materials:
    """UsdPreviewSurface materials, created on demand and cached by name. ``enabled=False`` = displayColor only."""

    def __init__(self, stage, enabled=True, root="/World/Looks"):
        self.stage, self.enabled, self.root = stage, enabled, root
        self.cache = {}
        self.bound = 0

    def get(self, name, rgb, roughness=0.85, primvar_fallback=None):
        if name in self.cache:
            return self.cache[name]
        path = f"{self.root}/{name}"
        mat = UsdShade.Material.Define(self.stage, path)
        sh = UsdShade.Shader.Define(self.stage, path + "/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        diffuse = sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f)
        if primvar_fallback is not None:          # per-vertex colours (terrain) through a primvar reader
            rd = UsdShade.Shader.Define(self.stage, path + "/ColorReader")
            rd.CreateIdAttr("UsdPrimvarReader_float3")
            rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("displayColor")
            rd.CreateInput("fallback", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(*map(float, primvar_fallback)))
            diffuse.ConnectToSource(rd.CreateOutput("result", Sdf.ValueTypeNames.Float3))
        else:
            diffuse.Set(Gf.Vec3f(*map(float, rgb)))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        self.cache[name] = mat
        return mat

    def bind(self, prim, name, rgb, **kw):
        if not self.enabled:
            return None
        mat = self.get(name, rgb, **kw)
        bind(prim, mat)
        self.bound += 1
        return mat


def bind(prim, mat, strength=None):
    try:
        api = UsdShade.MaterialBindingAPI.Apply(prim)
    except AttributeError:                        # very old USD: non-applied API
        api = UsdShade.MaterialBindingAPI(prim)
    if strength is None:
        api.Bind(mat)
    else:
        api.Bind(mat, strength)


# ---------------------------------------------------------------------------- geometry
def box(stage, path, center, size, rgb, quat=None, collide=True, mats=None, mat_name=None, mat_rgb=None):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])   # fallback extent is for size 2
    _xform(c, center, quat, size)
    _color(c, rgb)
    if collide:
        _collide(c.GetPrim())
    if mats is not None:
        mats.bind(c.GetPrim(), mat_name or path.rsplit("/", 1)[-1], rgb if mat_rgb is None else mat_rgb)
    return c


def heightfield_mesh(stage, path, fn, bounds, res, rgb_fn):
    x0, y0, x1, y1 = bounds
    xs = np.arange(x0, x1 + 1e-6, res)
    ys = np.arange(y0, y1 + 1e-6, res)
    X, Y = np.meshgrid(xs, ys)
    Z = fn(X, Y)
    nx, ny = len(xs), len(ys)
    pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
    ii, jj = np.meshgrid(np.arange(nx - 1), np.arange(ny - 1))
    a = (jj * nx + ii).ravel()
    idx = np.stack([a, a + 1, a + nx + 1, a + nx], 1).ravel()
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([4] * (len(idx) // 4)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(idx.astype(np.int32)))
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateExtentAttr([Gf.Vec3f(*map(float, pts.min(0))), Gf.Vec3f(*map(float, pts.max(0)))])
    cols = rgb_fn(Z.ravel()).astype(np.float32)
    pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar("displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.vertex)
    pv.Set(Vt.Vec3fArray.FromNumpy(cols))
    _collide(m.GetPrim(), mesh=True)
    return m


def piece_pose(pc):
    """(center, Gf.Quatd, size, R) of a scenario piece; R is column-convention (world = R @ local)."""
    R = rot_matrix(pc.rpy_deg)
    q = Gf.Matrix3d(*R.T.flatten().tolist()).ExtractRotation().GetQuat()
    return pc.center, q, pc.size, R


def sun_transform(elev_deg=35.0, azim_deg=205.0):
    """4x4 (USD row-vector) transform aiming a DistantLight (shines along its local -Z) from azimuth/elevation."""
    from isaac.camera import look_at
    el, az = math.radians(elev_deg), math.radians(azim_deg)
    d = -np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    M = look_at((0.0, 0.0, 0.0), d)
    return Gf.Matrix4d(*M.flatten().tolist())


def reference_site(stage, usd_path, lights=True, scan_textures=True, root="/World/CollapseSite"):
    """Reference an external site USD (the teammate's collapse_site.usda) at ``root``. Its terrain and
    buildings carry triangle-mesh colliders, so the PhysX LiDAR hits them; debris is visual only.
    ``lights=False`` deactivates its Sun/Sky (our lighting or --photoreal's is used instead);
    ``scan_textures=False`` drops the albedo maps derived from the third-party photogrammetry screenshots."""
    prim = stage.DefinePrim(root, "Xform")
    prim.GetReferences().AddReference(usd_path)
    if not lights:
        lp = stage.GetPrimAtPath(root + "/Lights")
        if lp.IsValid():
            lp.SetActive(False)
    if not scan_textures:
        strip_scan_textures(stage, root)
    return prim


SCAN_TEXTURE_KEYS = ("scan_", "terrain_r1c1_albedo", "terrain_r1c2_albedo", "terrain_r2c1_albedo",
                     "terrain_r2c2_albedo", "facade_tower")


def strip_scan_textures(stage, root="/World/CollapseSite", keys=SCAN_TEXTURE_KEYS):
    """Best effort: blank every texture file whose name comes from the scan screenshots (core terrain tiles,
    scanned roofs, tower faces). OmniPBR then uses its constant colour; UsdUVTexture its (grey) fallback."""
    n = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() != "Shader" or not str(prim.GetPath()).startswith(root):
            continue
        sh = UsdShade.Shader(prim)
        for inp in sh.GetInputs():
            v = inp.Get()
            if isinstance(v, Sdf.AssetPath) and any(k in v.path for k in keys):
                inp.Set(Sdf.AssetPath(""))
                if sh.GetIdAttr().Get() == "UsdUVTexture":
                    sh.CreateInput("fallback", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(0.5, 0.48, 0.45, 1.0))
                n += 1
    return n


def build(stage, sc, show_truth=False, terrain_res=0.5, physics_scene=True, sun=1800.0, sky=350.0,
          sun_elev=35.0, materials="preview", site_usd=None, site_lights="auto", scan_textures=True,
          photoreal=False):
    """Returns the number of rubble pieces. ``physics_scene=False`` under Isaac Lab (SimulationContext owns
    /physicsScene and already has scene queries on); the Kit route defines /World/physicsScene here.

    ``site_usd`` (collapse site): reference that USD at /World/CollapseSite instead of building terrain and
    boxes. ``site_lights``: site (its Sun/Sky only) | ours (ours only) | both | auto (ours with --photoreal,
    whose sky/sun replace ours anyway, else the site's)."""
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    if physics_scene:
        ps = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
        ps.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        ps.CreateGravityMagnitudeAttr(9.81)
        from isaac import compat
        compat.enable_scene_queries(ps.GetPrim())
    mats = Materials(stage, enabled=(materials != "none"))
    if site_lights == "auto":
        site_lights = "ours" if photoreal else "site"
    if site_usd:
        reference_site(stage, site_usd, lights=site_lights in ("site", "both"), scan_textures=scan_textures)
    ours = not site_usd or site_lights in ("ours", "both")

    # low warm sun (long shadows) + dim cool sky: the L40S-measured values, dusk-ish
    if ours:
        sunl = UsdLux.DistantLight.Define(stage, "/World/Sun")
        sunl.CreateIntensityAttr(float(sun))
        sunl.CreateAngleAttr(1.0)
        sunl.CreateColorAttr(Gf.Vec3f(1.0, 0.9, 0.78))
        sxf = UsdGeom.Xformable(sunl)
        sxf.ClearXformOpOrder()
        sxf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(sun_transform(sun_elev))
        dome = UsdLux.DomeLight.Define(stage, "/World/Sky")
        dome.CreateIntensityAttr(float(sky))
        dome.CreateColorAttr(Gf.Vec3f(0.80, 0.85, 1.0))

    n = 0
    if not site_usd:   # the referenced site brings its own terrain, buildings and colliders
        # terrain (grid/Weiguan: sector mounds + undulation; mesh sites: the loaded surface) with a flat apron
        x0, y0, x1, y1 = bounds(sc)
        tb = (x0 - 6, y0 - 6, x1 + 6, y1 + 6)

        def dust(z):
            k = 0.36 + 0.02 * np.sin(z * 9.0) - 0.02 * np.clip(z, 0, 4)
            return np.stack([k * 1.08, k, k * 0.9], 1)
        terrain = heightfield_mesh(stage, "/World/Terrain", sc.ground_height, tb, terrain_res, dust)
        mats.bind(terrain.GetPrim(), "terrain", (0.37, 0.34, 0.31), roughness=0.95, primvar_fallback=(0.37, 0.34, 0.31))
        apron = UsdGeom.Mesh.Define(stage, "/World/Apron")
        apron.CreatePointsAttr([Gf.Vec3f(-300, -300, -0.06), Gf.Vec3f(300, -300, -0.06), Gf.Vec3f(300, 300, -0.06), Gf.Vec3f(-300, 300, -0.06)])
        apron.CreateFaceVertexCountsAttr([4])
        apron.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        apron.CreateExtentAttr([Gf.Vec3f(-300, -300, -0.06), Gf.Vec3f(300, 300, -0.06)])
        apron.CreateSubdivisionSchemeAttr("none")
        _color(apron, (0.33, 0.31, 0.28))
        _collide(apron.GetPrim(), mesh=True)
        mats.bind(apron.GetPrim(), "apron", (0.33, 0.31, 0.28), roughness=0.95)

        # rubble pieces (one shared material per palette colour x brightness bucket)
        rng = np.random.default_rng(sc.seed)
        for p in sc.piles:
            UsdGeom.Xform.Define(stage, f"/World/Rubble/{p.name}")
            for k, pc in enumerate(p.pieces):
                center, q, size, _ = piece_pose(pc)
                pal = PALETTE[pc.material]
                ci = int(rng.integers(0, len(pal)))
                f = rng.uniform(0.9, 1.08)
                bi = int(np.argmin([abs(f - b) for b in BRIGHT]))
                rgb = np.array(pal[ci]) * f
                box(stage, f"/World/Rubble/{p.name}/{pc.kind}_{k:03d}", center, size, rgb, q, mats=mats,
                    mat_name=f"{pc.material}_{ci}_{bi}", mat_rgb=np.array(pal[ci]) * BRIGHT[bi])
                n += 1

    # staging area: pad, truck, coordinator mast, operator dashboard table, acoustic-agent server
    bx, by = base_xy(sc)
    box(stage, "/World/Staging/Pad", (bx, by - 1.5, 0.05), (16, 7, 0.1), (0.17, 0.2, 0.23), mats=mats)
    box(stage, "/World/Staging/Truck", (bx + 7, by - 3.5, 1.15), (4.6, 2.3, 2.3), (0.85, 0.87, 0.88), mats=mats)
    box(stage, "/World/Staging/Mast", (bx + 7, by - 2.2, 4.0), (0.1, 0.1, 3.4), (0.6, 0.64, 0.68), collide=False, mats=mats)
    box(stage, "/World/Staging/CoordinatorBeacon", (bx + 7, by - 2.2, 5.8), (0.3, 0.3, 0.3), (0.43, 0.83, 0.43), collide=False, mats=mats)
    box(stage, "/World/Staging/DashboardTable", (bx - 3.2, by + 0.4, 0.95), (2.2, 1.0, 0.08), (0.36, 0.39, 0.42), mats=mats)
    box(stage, "/World/Staging/DashboardScreen", (bx - 3.2, by + 0.6, 1.7), (1.3, 0.05, 0.8), (0.11, 0.29, 0.48), collide=False, mats=mats)
    box(stage, "/World/Staging/AcousticAgent", (bx + 3.6, by + 0.4, 0.7), (0.9, 0.8, 1.4), (0.08, 0.35, 0.4), mats=mats)

    if show_truth:   # display only; nothing in the brain reads these prims
        for v in sc.truth.get("victims", []):
            x, y = v["xy"]
            cap = UsdGeom.Capsule.Define(stage, f"/World/GroundTruth/{v['id']}")
            cap.CreateRadiusAttr(0.2)
            cap.CreateHeightAttr(0.9)
            cap.CreateAxisAttr("X")
            _xform(cap, (x, y, victim_z(sc, v)))          # Weiguan/mesh: v["z"] (inside the void)
            _color(cap, (0.95, 0.95, 0.95))
            mats.bind(cap.GetPrim(), "truth_ghost", (0.95, 0.95, 0.95), roughness=0.4)
    build.materials = mats
    return n
