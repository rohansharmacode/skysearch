"""Photoreal dressing + headless recording for the rubble-grid demo (``run_mission.py --photoreal 1 --record DIR``).

Visual only: the mission brain, the scenario and the state.json contract are untouched. The rubble keeps the
scenario's layout exactly: every scenario box is swapped for an irregular piece (broken slab with rebar, chipped
block, fractured lump, broken wall stub) with the SAME centre, orientation and size, and that piece is the
PhysX collider the LiDAR raycasts hit (static triangle mesh). So point clouds, landing cells and pings match the
numpy sim within a few centimetres.

  dress(stage, sc, a, log)   materials (NVIDIA MDL: concrete, brick, dirt, rusted steel), dusk HDRI sky + low sun,
                             irregular rubble, gravel and debris scattered around each pile (visual only),
                             sparse dry grass / shrubs / trees at the site edges, an operator at the staging area
  Rigs(stage, specs)         extra recording cameras: chase:d1 (3rd person behind a drone), fpv:d1 (1st person)
  Recorder(out, cams, res)   writes PNG frames per camera from render products (works headless)
  set_renderer(a)            --rtx pathtracing --spp N (RTX path tracing + OptiX denoiser), else real-time

Tested on Isaac Sim 6.0.1 (Brev L40S) through the SkySearch scene, which uses the same MDLs, HDRIs and assets.
"""
from __future__ import annotations

import math
import os

import numpy as np

from isaac import rubble as RB

S3 = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0"


LIDAR_DRAW_R = 11.0                                   # --photoreal: draw LiDAR hits only on the pile being mapped
WARP_PHASES = ("radar", "rescan", "relocate")     # long, repetitive stretches: shortened when recording


def add_args(ap):
    g = ap.add_argument_group("photoreal look and recording (isaac/photoreal.py)")
    g.add_argument("--photoreal", type=int, default=0, help="1 = photoreal materials, irregular rubble, sky, vegetation")
    g.add_argument("--assets", default="", help="assets root (default: Isaac's, else the public S3 mirror for 6.0)")
    g.add_argument("--hdri", default="evening_road_01_4k.hdr", help="sky HDRI under NVIDIA/Assets/Skies/*/")
    g.add_argument("--sky", type=float, default=900.0, help="HDRI dome intensity")
    g.add_argument("--sun", type=float, default=2200.0, help="sun intensity")
    g.add_argument("--sun-elev", type=float, default=24.0, help="sun elevation (deg): low = long dusk shadows")
    g.add_argument("--veg", type=int, default=1, help="dry grass, shrubs and a few trees around the site")
    g.add_argument("--operator", type=int, default=1, help="a person model (operator) at the dashboard table")
    g.add_argument("--record", default="", help="write PNG frames per camera to this folder (virtual clock)")
    g.add_argument("--rec-res", default="1920x1080")
    g.add_argument("--rec-fps", type=float, default=30.0, help="video frames per second")
    g.add_argument("--rec-speed", type=float, default=6.0, help="sim seconds per video second when recording")
    g.add_argument("--rec-cams", default="director", help="comma list: director, wide, chase:d1, fpv:d2, top")
    g.add_argument("--rec-stop-t", type=float, default=0.0, help="stop recording at this mission time (s)")
    g.add_argument("--rec-warp", type=float, default=1.0,
                   help="recording: speed up the long radar/rescan/relocate phases by this factor (e.g. 4)")
    g.add_argument("--rec-every", type=int, default=1, help="save every Nth frame (preview runs)")
    g.add_argument("--rec-windows", default="",
                   help='render/save only these frame ranges per camera, e.g. "director:120-250,560-700;wide:0-110" '
                        "(path-traced passes of just the shots the cut uses)")
    g.add_argument("--rtx", choices=["realtime", "pathtracing"], default="realtime")
    g.add_argument("--spp", type=int, default=16, help="--rtx pathtracing: samples per pixel per frame")
    return g


# ------------------------------------------------------------------------------------------------ assets
def _roots(a):
    root = a.assets
    if not root:
        try:
            from isaac import compat
            root = compat.assets_root() or S3
        except Exception:
            root = S3
    return root.rstrip("/") + "/NVIDIA", root.rstrip("/") + "/Isaac"


def mdl_paths(nv):
    return {
        "concrete": f"{nv}/Materials/Base/Masonry/Concrete_Rough.mdl",
        "concrete2": f"{nv}/Materials/Base/Masonry/Concrete_Formed.mdl",
        "brick": f"{nv}/Materials/Base/Masonry/Brick_Wall_Red.mdl",
        "brick2": f"{nv}/Materials/Base/Masonry/Brick_Wall_Brown.mdl",
        "dirt": f"{nv}/Materials/Base/Natural/Dirt.mdl",
        "gravel": f"{nv}/Materials/Base/Stone/Gravel.mdl",
        "asphalt": f"{nv}/Materials/Base/Natural/Asphalt.mdl",
        "rust": f"{nv}/Materials/Base/Metals/RustedMetal.mdl",
    }


def mdl_material(stage, path, url, scale=(0.5, 0.5)):
    """MDL material with tri-planar projected UVs (so procedural meshes and boxes texture without UVs)."""
    from pxr import Gf, Sdf, UsdShade
    name = os.path.splitext(os.path.basename(url))[0]
    sh = None
    try:
        import omni.kit.commands
        omni.kit.commands.execute("CreateMdlMaterialPrim", mtl_url=url, mtl_name=name, mtl_path=path, select_new_prim=False)
        for c in stage.GetPrimAtPath(path).GetChildren():
            if c.GetTypeName() == "Shader":
                sh = UsdShade.Shader(c)
    except Exception:
        sh = None
    mat = UsdShade.Material.Define(stage, path)
    if sh is None:                                   # plain pxr (no kit command): same structure
        sh = UsdShade.Shader.Define(stage, path + "/Shader")
        sh.SetSourceAsset(Sdf.AssetPath(url), "mdl")
        sh.SetSourceAssetSubIdentifier(name, "mdl")
        out = sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
        for o in (mat.CreateSurfaceOutput("mdl"), mat.CreateDisplacementOutput("mdl"), mat.CreateVolumeOutput("mdl")):
            o.ConnectToSource(out)
    sh.GetPrim().CreateAttribute("inputs:project_uvw", Sdf.ValueTypeNames.Bool).Set(True)
    sh.GetPrim().CreateAttribute("inputs:texture_scale", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(*map(float, scale)))
    return mat


def bind(stage, path, mat):
    from pxr import UsdShade
    p = stage.GetPrimAtPath(path)
    if p.IsValid():
        UsdShade.MaterialBindingAPI.Apply(p).Bind(mat, UsdShade.Tokens.strongerThanDescendants)


def _units(url):
    try:
        from pxr import Usd, UsdGeom
        return float(UsdGeom.GetStageMetersPerUnit(Usd.Stage.Open(url, load=Usd.Stage.LoadNone)))
    except Exception:
        return 1.0


def reference(stage, path, url, pos, yaw_deg=0.0, scale=1.0):
    """Reference a USD asset (unit-corrected), physics stripped so it is purely visual."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    x = UsdGeom.Xform.Define(stage, path)
    x.GetPrim().GetReferences().AddReference(url)
    s = scale * _units(url)
    xf = UsdGeom.Xformable(x)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, pos)))
    xf.AddRotateZOp(UsdGeom.XformOp.PrecisionDouble).Set(float(yaw_deg))
    xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(s, s, s))
    for p in Usd.PrimRange(x.GetPrim()):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
    return x.GetPrim()


# ------------------------------------------------------------------------------------------------ meshes
def _mesh(stage, path, P, F, rgb=(0.5, 0.5, 0.5), collide=False, smooth=False):
    from pxr import Gf, UsdGeom, UsdPhysics, Vt
    m = UsdGeom.Mesh.Define(stage, path)
    P = np.ascontiguousarray(P, np.float32); F = np.ascontiguousarray(F, np.int32)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(P))
    m.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(F), 3, np.int32)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(F.reshape(-1)))
    m.CreateSubdivisionSchemeAttr("none")
    if smooth:
        m.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(RB.smooth_normals(P, F))); m.SetNormalsInterpolation("vertex")
    else:
        m.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(RB.normals(P, F))); m.SetNormalsInterpolation("faceVarying")
    m.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, P.min(0))), Gf.Vec3f(*map(float, P.max(0)))]))
    m.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, rgb))]))
    if collide:
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("none")
    return m


def _xf(stage, path, t, q=None):
    from pxr import Gf, UsdGeom
    x = UsdGeom.Xform.Define(stage, path)
    xf = UsdGeom.Xformable(x)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, t)))
    if q is not None:
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(q)
    return x


def _column(rng, sx, sy, sz):
    """Broken column lying along x: chipped block plus bent bars sticking out of both ends."""
    mesh = RB.block(rng, sx, sy, sz, chip=0.08)
    bars = []
    for end in (-1, 1):
        for _ in range(rng.integers(2, 5)):
            p0 = np.array([end * (sx / 2 - 0.05), rng.uniform(-0.35, 0.35) * sy, rng.uniform(-0.35, 0.35) * sz])
            L = rng.uniform(0.15, 0.6)
            mid = p0 + np.array([end * L * 0.6, 0, 0]) + rng.normal(0, 0.04, 3)
            bars += [RB._bar(p0, mid, r=0.012), RB._bar(mid, mid + (np.array([end, 0, 0]) + rng.normal(0, 0.6, 3)) * L * 0.4, r=0.012)]
    return mesh, RB.merge(bars)


def _wall(rng, sx, sy, sz):
    """Standing broken brick/concrete wall stub, ragged top, centred like the box it replaces."""
    n = max(2, int(round(sx / 0.3)))
    tops = sz * rng.uniform(0.75, 1.0, n)
    (P, F), bars, _, _ = RB.wall_run(rng, sx, sy, tops, step=0.05, notch=0.25 * sz)
    P = P - np.array([0, 0, sz / 2], np.float32)
    if len(bars[0]):
        bars = (bars[0] - np.array([0, 0, sz / 2], np.float32), bars[1])
    return (P, F), bars


def dress_rubble(stage, sc, mats, log, seed=0):
    """Swap each scenario box for an irregular piece with the same pose and size (the LiDAR's collider)."""
    from pxr import Gf
    from sim.scenario import rot_matrix
    rng = np.random.default_rng(1000 + seed)
    n = nb = 0
    for p in sc.piles:
        for k, pc in enumerate(p.pieces):
            box = f"/World/Rubble/{p.name}/{pc.kind}_{k:03d}"
            R = rot_matrix(pc.rpy_deg)
            q = Gf.Matrix3d(*R.T.flatten().tolist()).ExtractRotation().GetQuat()
            sx, sy, sz = (float(v) for v in pc.size)
            if pc.kind == "column":
                mesh, bars = _column(rng, sx, sy, sz)
            elif pc.kind == "wall":
                mesh, bars = _wall(rng, sx, sy, sz)
            else:
                mesh, bars = RB.piece_for_box(rng, sx, sy, sz, rebar_p=0.55 if pc.material == "concrete" else 0.0)
            root = f"/World/RubbleMesh/{p.name}/{pc.kind}_{k:03d}"
            _xf(stage, root, pc.center, q)
            tint = (0.55, 0.53, 0.50) if pc.material == "concrete" else (0.50, 0.33, 0.26)
            _mesh(stage, root + "/geo", *mesh, tint, collide=True)
            key = pc.material + ("2" if rng.random() < 0.35 else "")
            bind(stage, root + "/geo", mats[key])
            if bars is not None and len(bars[0]):
                _mesh(stage, root + "/rebar", *bars, (0.25, 0.14, 0.09), smooth=True)
                bind(stage, root + "/rebar", mats["rust"])
                nb += 1
            old = stage.GetPrimAtPath(box)
            if old.IsValid():
                old.SetActive(False)                 # the irregular piece replaces the box (render + collider)
            n += 1
    log(f"[photoreal] rubble: {n} irregular pieces ({nb} with rebar) replace the scenario boxes, same poses")
    return n


def scatter_debris(stage, sc, mats, log, seed=0):
    """Gravel, brick bits and fist-sized concrete around and on every pile (visual only: no collider, so the
    LiDAR and landing cells are unaffected)."""
    rng = np.random.default_rng(2000 + seed)
    tot = 0
    for p in sc.piles:
        for mat, n, smin, smax in (("concrete", 2400, 0.025, 0.09), ("brick", 700, 0.03, 0.12), ("concrete2", 700, 0.10, 0.45)):
            parts = []
            for _ in range(n):
                r = abs(rng.normal(0, 0.62)) + rng.uniform(0, 0.15)          # dense on the pile, thinning outwards
                ang = rng.uniform(0, 2 * np.pi)
                x = p.x + r * p.sx * math.cos(ang)
                y = p.y + r * p.sy * math.sin(ang)
                z = float(np.asarray(sc.ground_height(np.array([x]), np.array([y]))).ravel()[0])
                s = rng.uniform(smin, smax)
                P, F = RB.rock(rng, s, s * rng.uniform(0.6, 1.0), s * rng.uniform(0.35, 0.7), n=9 if s < 0.1 else 14)
                c, si = math.cos(rng.uniform(0, 6.28)), math.sin(rng.uniform(0, 6.28))
                P = P @ np.array([[c, si, 0], [-si, c, 0], [0, 0, 1]], np.float32).T
                P = P + np.array([x, y, z - P[:, 2].min() - 0.35 * s * 0.5], np.float32)
                parts.append((P, F))
            P, F = RB.merge(parts)
            path = f"/World/Debris/{p.name}_{mat}"
            _mesh(stage, path, P, F, (0.5, 0.48, 0.45) if mat.startswith("concrete") else (0.5, 0.33, 0.26))
            bind(stage, path, mats[mat])
            tot += n
    log(f"[photoreal] debris: {tot} stones and fragments scattered over {len(sc.piles)} piles (visual only)")


def rubble_ground(stage, sc, mat, log):
    """The terrain under and around each pile gets a crushed-concrete gravel material (a GeomSubset of the
    terrain mesh), so a pile reads as a heap of debris, not a sand dune with slabs on it. Geometry unchanged."""
    from pxr import UsdGeom, UsdShade, Vt
    prim = stage.GetPrimAtPath("/World/Terrain")
    if not prim.IsValid():
        return
    m = UsdGeom.Mesh(prim)
    P = np.asarray(m.GetPointsAttr().Get(), float)
    cnt = np.asarray(m.GetFaceVertexCountsAttr().Get())
    idx = np.asarray(m.GetFaceVertexIndicesAttr().Get())
    if not len(cnt) or not np.all(cnt == cnt[0]):
        return
    C = P[idx.reshape(-1, int(cnt[0]))].mean(1)
    rng = np.random.default_rng(sc.seed + 5)
    wob = rng.normal(0, 0.12, len(C))                       # ragged edge, not a perfect ellipse
    sel = np.zeros(len(C), bool)
    for p in sc.piles:
        e = ((C[:, 0] - p.x) / (1.25 * p.sx)) ** 2 + ((C[:, 1] - p.y) / (1.25 * p.sy)) ** 2
        sel |= e < 1.0 + wob
    faces = np.nonzero(sel)[0].astype(np.int32)
    sub = UsdGeom.Subset.CreateGeomSubset(m, "rubble", UsdGeom.Tokens.face, Vt.IntArray.FromNumpy(faces),
                                          familyName=UsdShade.Tokens.materialBind)
    UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mat, UsdShade.Tokens.strongerThanDescendants)
    log(f"[photoreal] crushed-concrete ground under the piles: {len(faces)} of {len(C)} terrain faces")


VEG_TREES = ("Black_Oak", "Gray_Birch", "White_Pine", "Shumard_Oak")
VEG_SHRUBS = ("Switchgrass", "Fountain_Grass_Tall", "Pampas_Grass", "Cedar_Shrub")   # no red foliage: red = radar ping
PEOPLE = ("male_adult_construction_01_new/male_adult_construction_01_new.usd",
          "male_adult_construction_05_new/male_adult_construction_05_new.usd")


def vegetation(stage, sc, nv, log, seed=0):
    """Dry grass clumps and shrubs between the piles, a few trees at the edges; nothing on piles, flight paths
    over the grid centre lines, or the staging area."""
    from sim.scenario import BASE_XY
    rng = np.random.default_rng(3000 + seed)
    pts = []

    def free(x, y, gap):
        if any(math.hypot(x - p.x, y - p.y) < max(p.sx, p.sy) * 1.6 for p in sc.piles):
            return False
        if abs(x - BASE_XY[0]) < 14 and abs(y - BASE_XY[1]) < 8:
            return False
        return all(math.hypot(x - a, y - b) > gap for a, b in pts)

    n_s = n_t = 0
    for _ in range(3000):
        if n_s >= 70:
            break
        x, y = rng.uniform(-34, 34), rng.uniform(-40, 34)
        if free(x, y, 2.5):
            z = float(np.asarray(sc.ground_height(np.array([x]), np.array([y]))).ravel()[0])
            u = f"{nv}/Assets/Vegetation/Shrub/{VEG_SHRUBS[n_s % len(VEG_SHRUBS)]}.usd"
            try:
                reference(stage, f"/World/Vegetation/shrub{n_s:02d}", u, (x, y, z - 0.03), rng.uniform(0, 360), rng.uniform(0.6, 1.1))
            except Exception as e:
                log(f"[photoreal] shrubs off ({type(e).__name__}: {e})"); break
            pts.append((x, y)); n_s += 1
    for _ in range(2000):
        if n_t >= 12:
            break
        ang = rng.uniform(0, 2 * np.pi)
        x, y = 46 * math.cos(ang) + rng.normal(0, 3), -6 + 50 * math.sin(ang) + rng.normal(0, 3)
        if free(x, y, 7.0):
            u = f"{nv}/Assets/Vegetation/Trees/{VEG_TREES[n_t % len(VEG_TREES)]}.usd"
            try:
                reference(stage, f"/World/Vegetation/tree{n_t:02d}", u, (x, y, -0.05), rng.uniform(0, 360), rng.uniform(0.8, 1.15))
            except Exception as e:
                log(f"[photoreal] trees off ({type(e).__name__}: {e})"); break
            pts.append((x, y)); n_t += 1
    log(f"[photoreal] vegetation: {n_s} grass/shrub clumps, {n_t} trees")


def sky_and_sun(stage, a, nv, log):
    from pxr import Gf, Sdf, UsdGeom, UsdLux
    sky = None
    for sub in ("Clear", "Evening", "Cloudy", "Night", "Studio", ""):
        cand = f"{nv}/Assets/Skies/{sub}/{a.hdri}" if sub else a.hdri
        try:
            import omni.client
            if omni.client.stat(cand)[0] == omni.client.Result.OK:
                sky = cand; break
        except Exception:
            sky = f"{nv}/Assets/Skies/Clear/{a.hdri}"; break
    dome = UsdLux.DomeLight.Define(stage, "/World/Sky")
    dome.CreateIntensityAttr(float(a.sky))
    dome.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
    if sky:
        dome.CreateTextureFileAttr(Sdf.AssetPath(sky))
        dome.CreateTextureFormatAttr("latlong")
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(float(a.sun))
    sun.CreateAngleAttr(0.8)
    sun.CreateColorAttr(Gf.Vec3f(1.0, 0.80, 0.60))
    xf = UsdGeom.Xformable(sun)
    xf.ClearXformOpOrder()
    # light travels along -Z of the prim: tilt it down to the elevation, from the south-west
    xf.AddRotateZOp().Set(-135.0)
    xf.AddRotateXOp().Set(90.0 - float(a.sun_elev))
    log(f"[photoreal] sky {sky or '(none)'} at {a.sky}, sun {a.sun} at {a.sun_elev} deg elevation")


def dress(stage, sc, a, log=print):
    """Call right after scene.build(stage, sc) and before PhysX loads the stage."""
    nv, isa = _roots(a)
    M = mdl_paths(nv)
    mats = {}
    for k, s in (("concrete", (0.6, 0.6)), ("concrete2", (0.6, 0.6)), ("brick", (0.35, 0.35)), ("brick2", (0.35, 0.35)),
                 ("dirt", (0.12, 0.12)), ("gravel", (0.3, 0.3)), ("asphalt", (0.25, 0.25)), ("rust", (1.0, 1.0))):
        try:
            mats[k] = mdl_material(stage, f"/World/Looks/{k}", M[k], s)
        except Exception as e:
            log(f"[photoreal] material {k} failed: {type(e).__name__}: {e}")
    sky_and_sun(stage, a, nv, log)
    if "dirt" in mats:
        bind(stage, "/World/Terrain", mats["dirt"])
        bind(stage, "/World/Apron", mats["dirt"])
    if "gravel" in mats:
        rubble_ground(stage, sc, mats["gravel"], log)
    if "asphalt" in mats:
        bind(stage, "/World/Staging/Pad", mats["asphalt"])
    dress_rubble(stage, sc, mats, log, seed=sc.seed)
    scatter_debris(stage, sc, mats, log, seed=sc.seed)
    if a.veg:
        vegetation(stage, sc, nv, log, seed=sc.seed)
    if a.operator:
        from sim.scenario import BASE_XY
        bx, by = BASE_XY
        try:
            reference(stage, "/World/Operator", f"{isa}/People/Characters/{PEOPLE[0]}", (bx - 3.2, by - 0.4, 0.0), 90.0)
            log("[photoreal] operator model at the dashboard table")
        except Exception as e:
            log(f"[photoreal] operator off ({type(e).__name__}: {e})")
    try:                                           # light dusty haze (real-time renderer)
        import carb
        s = carb.settings.get_settings()
        s.set_bool("/rtx/fog/enabled", True)
        s.set_float("/rtx/fog/fogColorIntensity", 0.35)
        s.set_float("/rtx/fog/fogStartDist", 70.0)
        s.set_float("/rtx/fog/fogEndDist", 450.0)
        s.set_float("/rtx/fog/fogColor/0", 0.78); s.set_float("/rtx/fog/fogColor/1", 0.70); s.set_float("/rtx/fog/fogColor/2", 0.60)
    except Exception:
        pass


def set_renderer(a, log=print):
    try:
        import carb
        s = carb.settings.get_settings()
        if a.rtx == "pathtracing":
            s.set_string("/rtx/rendermode", "PathTracing")
            s.set_int("/rtx/pathtracing/spp", int(a.spp))
            s.set_int("/rtx/pathtracing/totalSpp", int(a.spp))
            s.set_bool("/rtx/pathtracing/optixDenoiser/enabled", True)
        else:
            s.set_string("/rtx/rendermode", "RaytracedLighting")
        log(f"[photoreal] renderer: {a.rtx}" + (f" ({a.spp} spp + denoiser)" if a.rtx == "pathtracing" else ""))
    except Exception as e:
        log(f"[photoreal] renderer settings skipped ({type(e).__name__}: {e})")


# ------------------------------------------------------------------------------------------------ cameras
def _look_at(eye, target, up=(0, 0, 1)):
    eye, target, up = (np.asarray(v, float) for v in (eye, target, up))
    z = eye - target; z /= np.linalg.norm(z) or 1.0
    x = np.cross(up, z)
    x = x / np.linalg.norm(x) if np.linalg.norm(x) > 1e-6 else np.array([1.0, 0, 0])
    y = np.cross(z, x)
    M = np.eye(4); M[0, :3], M[1, :3], M[2, :3], M[3, :3] = x, y, z, eye
    return M


class Rigs:
    """Extra recording cameras that follow drones: chase:<id> (behind and above) and fpv:<id> (nose camera),
    plus top (straight down on the active pile) and wide (slow high orbit of the whole grid and staging)."""

    def __init__(self, stage, specs, surface):
        from pxr import Gf, UsdGeom
        self.Gf, self.surface, self.rigs = Gf, surface, []
        for spec in specs:
            mode, _, who = spec.partition(":")
            if mode not in ("chase", "fpv", "top", "wide"):
                continue
            path = f"/World/RecCams/{mode}_{who or 'x'}"
            cam = UsdGeom.Camera.Define(stage, path)
            cam.CreateFocalLengthAttr(9.0 if mode == "fpv" else 16.0)
            cam.CreateHorizontalApertureAttr(20.955)
            cam.CreateClippingRangeAttr(Gf.Vec2f(0.03, 3000.0))
            xf = UsdGeom.Xformable(cam); xf.ClearXformOpOrder()
            self.rigs.append(dict(mode=mode, who=who or "d1", path=path, op=xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble),
                                  eye=None, tgt=None, name=spec.replace(":", "_")))

    def update(self, poses, focus, dt):
        for r in self.rigs:
            if r["mode"] == "wide":                       # establishing / pull-back: whole grid at dusk
                r["th"] = r.get("th", -math.pi / 2 - 0.45) + dt * 2 * math.pi / 140.0
                eye = np.array([64 * math.cos(r["th"]), 64 * math.sin(r["th"]) - 10.0, 36.0])
                tgt = np.array([0.0, -10.0, 0.0])
            elif r["mode"] == "top":
                if focus is None:
                    continue
                eye = np.array([focus[0], focus[1] - 0.01, focus[2] + 26.0]); tgt = np.array(focus, float)
            else:
                p = poses.get(r["who"])
                if p is None:
                    continue
                x, y, z, yaw = p[0], p[1], p[2], math.radians(p[3])
                fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])
                pos = np.array([x, y, z])
                if r["mode"] == "fpv":
                    eye = pos + 0.45 * fwd + [0, 0, -0.08]
                    tgt = eye + fwd * 10 + [0, 0, -3.5]
                else:
                    eye = pos - 5.5 * fwd + [0, 0, 2.2]
                    eye[2] = max(eye[2], float(self.surface(eye[0], eye[1])) + 1.2)
                    tgt = pos + 2.0 * fwd
            k = 1.0 - math.exp(-dt / (0.25 if r["mode"] == "fpv" else 0.6))
            r["eye"] = eye if r["eye"] is None else r["eye"] + (eye - r["eye"]) * k
            r["tgt"] = tgt if r["tgt"] is None else r["tgt"] + (tgt - r["tgt"]) * k
            M = _look_at(r["eye"], r["tgt"])
            r["op"].Set(self.Gf.Matrix4d(*M.flatten().tolist()))


class Recorder:
    """PNG frames per camera from replicator render products (headless-safe).

    windows: {cam: [(f0, f1), ...]} (from --rec-windows "cam:f0-f1,f0-f1;cam2:..."). When given, only those
    cameras are rendered, and only around those frame ranges: their render products are created a few frames
    before a window opens and destroyed after it closes, so a path-traced pass costs only the frames the cut
    uses. Without windows every camera renders every frame (as before)."""

    PRE = 4                                            # frames of warm-up before a window (denoiser, latency)

    def __init__(self, out_dir, cams, res="1920x1080", log=print, windows=None):
        import omni.replicator.core as rep
        self.rep = rep
        self.wh = tuple(int(v) for v in res.split("x"))
        self.out, self.log, self.n = out_dir, log, 0
        self.paths = dict(cams)
        self.win = windows or {}
        self.live = {}                                 # name -> (render product, annotator)
        for name in self.paths:
            if not self.win or name in self.win:
                os.makedirs(os.path.join(out_dir, name), exist_ok=True)
        if not self.win:
            for name in self.paths:
                self._open(name)
        log(f"[photoreal] recording {list(self.paths)} at {self.wh[0]}x{self.wh[1]} to {out_dir}"
            + (f"; windows {self.win}" if self.win else ""))

    @staticmethod
    def parse_windows(spec):
        out = {}
        for part in filter(None, (p.strip() for p in (spec or "").split(";"))):
            cam, _, rngs = part.partition(":")
            out[cam.strip().replace(":", "_")] = [tuple(int(v) for v in r.split("-")) for r in rngs.split(",") if r.strip()]
        return out

    def _open(self, name):
        rp = self.rep.create.render_product(self.paths[name], self.wh)
        ann = self.rep.AnnotatorRegistry.get_annotator("rgb")
        ann.attach([rp])
        self.live[name] = (rp, ann)

    def _close(self, name):
        rp, ann = self.live.pop(name)
        for fn in (lambda: ann.detach(), lambda: rp.destroy()):
            try:
                fn()
            except Exception as e:                     # older replicator: leave it attached but stop saving
                self.log(f"[photoreal] close {name}: {type(e).__name__}: {e}")

    def _in(self, name, idx, pre=0):
        if not self.win:
            return True
        return any(f0 - pre <= idx <= f1 for f0, f1 in self.win.get(name, ()))

    def prepare(self, idx):
        """Before rendering frame idx: open render products whose window starts soon, close finished ones."""
        if not self.win:
            return
        for name in self.paths:
            want = self._in(name, idx, self.PRE)
            if want and name not in self.live and name in self.win:
                self._open(name)
            elif not want and name in self.live:
                self._close(name)

    def done(self, idx):
        return bool(self.win) and all(idx > max(f1 for _, f1 in w) for w in self.win.values() if w)

    def capture(self, idx):
        from PIL import Image
        ok = 0
        for name, (rp, ann) in list(self.live.items()):
            if not self._in(name, idx):
                continue
            d = ann.get_data()
            if d is None or getattr(d, "size", 0) == 0:
                continue
            img = np.asarray(d)[..., :3]
            Image.fromarray(img).save(os.path.join(self.out, name, f"f{idx:05d}.png"), compress_level=1)
            ok += 1
        self.n += ok > 0
        return ok

    def note(self, idx, state, pile):
        """timeline.csv: video frame -> mission time, phase, active pile, pings so far (for cutting the shots)."""
        f = os.path.join(self.out, "timeline.csv")
        new = not os.path.exists(f)
        with open(f, "a") as fh:
            if new:
                fh.write("frame,t,phase,pile,pings,clips,dispatches\n")
            fh.write(f"{idx},{float(state.get('t', 0)):.2f},{state.get('phase')},{pile},"
                     f"{len(state.get('radar_candidates') or [])},{len(state.get('acoustic_clips') or [])},"
                     f"{len(state.get('dispatch') or [])}\n")
