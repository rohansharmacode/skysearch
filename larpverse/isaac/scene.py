"""Build the 3x3 rubble-grid stage from sim.scenario (pxr only, so it is stable across Isaac versions).

Every rubble piece and the terrain get PhysX collision, so the LiDAR raycasts hit real Isaac geometry.
Drones are visual-only (no rigid body, no collider) and are posed kinematically each step.
"""
import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, Vt

from sim.scenario import BASE_XY, SITE_BOUNDS, rot_matrix

PALETTE = {
    "concrete": [(0.56, 0.55, 0.52), (0.49, 0.47, 0.45), (0.62, 0.59, 0.55), (0.43, 0.41, 0.39)],
    "brick": [(0.53, 0.38, 0.30), (0.48, 0.31, 0.25)],
}


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


def box(stage, path, center, size, rgb, quat=None, collide=True):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])   # fallback extent is for size 2
    _xform(c, center, quat, size)
    _color(c, rgb)
    if collide:
        _collide(c.GetPrim())
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


def build(stage, sc, show_truth=False, terrain_res=0.5):
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    ps = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    ps.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    ps.CreateGravityMagnitudeAttr(9.81)

    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(2500)
    sun.CreateAngleAttr(1.0)
    sun.CreateColorAttr(Gf.Vec3f(1.0, 0.9, 0.78))
    _xform(sun, (0, 0, 0), Gf.Rotation(Gf.Vec3d(1, 1, 0), 50).GetQuat())
    dome = UsdLux.DomeLight.Define(stage, "/World/Sky")
    dome.CreateIntensityAttr(500)
    dome.CreateColorAttr(Gf.Vec3f(0.62, 0.72, 0.82))

    # terrain (nine mounds + undulation) with a flat apron around it
    x0, y0, x1, y1 = SITE_BOUNDS
    tb = (x0 - 6, y0 - 6, x1 + 6, y1 + 6)

    def dust(z):
        k = 0.36 + 0.02 * np.sin(z * 9.0) - 0.02 * np.clip(z, 0, 4)
        return np.stack([k * 1.08, k, k * 0.9], 1)
    heightfield_mesh(stage, "/World/Terrain", sc.ground_height, tb, terrain_res, dust)
    apron = UsdGeom.Mesh.Define(stage, "/World/Apron")
    apron.CreatePointsAttr([Gf.Vec3f(-300, -300, -0.06), Gf.Vec3f(300, -300, -0.06), Gf.Vec3f(300, 300, -0.06), Gf.Vec3f(-300, 300, -0.06)])
    apron.CreateFaceVertexCountsAttr([4])
    apron.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    apron.CreateExtentAttr([Gf.Vec3f(-300, -300, -0.06), Gf.Vec3f(300, 300, -0.06)])
    apron.CreateSubdivisionSchemeAttr("none")
    _color(apron, (0.33, 0.31, 0.28))
    _collide(apron.GetPrim(), mesh=True)

    # rubble pieces
    rng = np.random.default_rng(sc.seed)
    n = 0
    for p in sc.piles:
        UsdGeom.Xform.Define(stage, f"/World/Rubble/{p.name}")
        for k, pc in enumerate(p.pieces):
            R = rot_matrix(pc.rpy_deg)
            q = Gf.Matrix3d(*R.T.flatten().tolist()).ExtractRotation().GetQuat()
            pal = PALETTE[pc.material]
            rgb = np.array(pal[int(rng.integers(0, len(pal)))]) * rng.uniform(0.9, 1.08)
            box(stage, f"/World/Rubble/{p.name}/{pc.kind}_{k:03d}", pc.center, pc.size, rgb, q)
            n += 1

    # staging area: pad, truck, coordinator mast, operator dashboard table, acoustic-agent server
    bx, by = BASE_XY
    box(stage, "/World/Staging/Pad", (bx, by - 1.5, 0.05), (16, 7, 0.1), (0.17, 0.2, 0.23))
    box(stage, "/World/Staging/Truck", (bx + 7, by - 3.5, 1.15), (4.6, 2.3, 2.3), (0.85, 0.87, 0.88))
    box(stage, "/World/Staging/Mast", (bx + 7, by - 2.2, 4.0), (0.1, 0.1, 3.4), (0.6, 0.64, 0.68), collide=False)
    box(stage, "/World/Staging/CoordinatorBeacon", (bx + 7, by - 2.2, 5.8), (0.3, 0.3, 0.3), (0.43, 0.83, 0.43), collide=False)
    box(stage, "/World/Staging/DashboardTable", (bx - 3.2, by + 0.4, 0.95), (2.2, 1.0, 0.08), (0.36, 0.39, 0.42))
    box(stage, "/World/Staging/DashboardScreen", (bx - 3.2, by + 0.6, 1.7), (1.3, 0.05, 0.8), (0.11, 0.29, 0.48), collide=False)
    box(stage, "/World/Staging/AcousticAgent", (bx + 3.6, by + 0.4, 0.7), (0.9, 0.8, 1.4), (0.08, 0.35, 0.4))

    if show_truth:   # display only; nothing in the brain reads these prims
        for v in sc.truth.get("victims", []):
            x, y = v["xy"]
            cap = UsdGeom.Capsule.Define(stage, f"/World/GroundTruth/{v['id']}")
            cap.CreateRadiusAttr(0.2)
            cap.CreateHeightAttr(0.9)
            cap.CreateAxisAttr("X")
            _xform(cap, (x, y, float(sc.ground_height(x, y)) - v["depth"]))
            _color(cap, (0.95, 0.95, 0.95))
    return n
