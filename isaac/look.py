"""Photoreal look for the earthquake scene: real materials, sky, models and a cinematic camera.

Used by isaac/earthquake_scene.py (``--look``, on by default). It only changes what you *see*:
the fleet logic, the sensor models and the physics are untouched.

  * Materials: NVIDIA MDL library (plaster walls, rough/formed concrete rubble and slabs, carpet
    floor, asphalt outside, rough concrete debris), with tri-planar UVs so plain cubes texture well.
  * Lighting: an HDRI sky dome (visible as the background) plus a low, warm sun with shadows.
  * Models: Bitcraze Crazyflie for the 6 crevice micro-drones, the Isaac quadcopter for the 3 scouts,
    and the same quadcopter scaled up for the coordinator. People are Isaac People characters lying
    on the floor. Office furniture (ArchVis desks, chairs, shelves) stands in open room cells.
    Every referenced model has its physics switched off, so it follows the fleet's kinematic poses
    and never falls or collides on its own.
  * Camera: a low cinematic orbit, and close-ups on the action at the storyboard beats
    (victim A, smoke in the east wing, victim B), blended smoothly in and out.

All asset paths come from isaac/assets_found.json (probed on the Brev L40S, Isaac Sim 6.0.1).
"""
import math

import numpy as np
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

import isaaclab.sim as sim_utils

NV = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/NVIDIA"
IS = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac"

MDL = {
    "plaster": f"{NV}/Materials/Base/Wall_Board/Plaster.mdl",
    "concrete_rough": f"{NV}/Materials/Base/Masonry/Concrete_Rough.mdl",
    "concrete_formed": f"{NV}/Materials/Base/Masonry/Concrete_Formed.mdl",
    "concrete_smooth": f"{NV}/Materials/Base/Masonry/Concrete_Smooth.mdl",
    "carpet": f"{NV}/Materials/Base/Carpet/Carpet_Berber_Gray.mdl",
    "asphalt": f"{NV}/Materials/Base/Natural/Asphalt.mdl",
}
SKY = f"{NV}/Assets/Skies/Clear/evening_road_01_4k.hdr"
CRAZYFLIE = f"{IS}/Robots/Bitcraze/Crazyflie/cf2x.usd"
QUAD = f"{IS}/Robots/IsaacSim/Quadcopter/quadcopter.usd"
PEOPLE = [f"{IS}/People/Characters/{p}" for p in (
    "male_adult_construction_01_new/male_adult_construction_01_new.usd",
    "F_Business_02/F_Business_02.usd",
    "male_adult_construction_05_new/male_adult_construction_05_new.usd",
    "F_Medical_01/F_Medical_01.usd",
)]
FURNITURE = [f"{NV}/Assets/ArchVis/{p}" for p in (
    "Residential/Furniture/Desks/Desk_01.usd",
    "Residential/Furniture/Chairs/Chair_Desk.usd",
    "Commercial/Seating/Jobba/Jobba_Chair.usd",
    "Commercial/Storage/Contemporary/Contemporary_LowHalfDesk.usd",
    "Residential/Furniture/Bookshelves/TetriminoShelf.usd",
)]


def _stage():
    return omni.usd.get_context().get_stage()


def make_mdl(name, key, scale=(0.5, 0.5)):
    path = f"/World/Looks/mdl_{name}"
    cfg = sim_utils.MdlFileCfg(mdl_path=MDL[key], project_uvw=True, texture_scale=scale)
    cfg.func(path, cfg)
    return path


def bind_tree(root, mat_path):
    """Bind a material directly on every geometry prim under root (overrides any per-prim binding)."""
    st = _stage()
    prim = st.GetPrimAtPath(root)
    if not prim.IsValid():
        return 0
    mat = UsdShade.Material(st.GetPrimAtPath(mat_path))
    n = 0
    for p in Usd.PrimRange(prim):
        if p.IsA(UsdGeom.Gprim):
            UsdShade.MaterialBindingAPI.Apply(p).Bind(mat, UsdShade.Tokens.strongerThanDescendants)
            n += 1
    return n


def strip_physics(root):
    """Make a referenced model purely visual: no rigid bodies, colliders, joints or articulations."""
    st = _stage()
    for p in Usd.PrimRange(st.GetPrimAtPath(root)):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            p.CreateAttribute("physxArticulation:articulationEnabled", sdf_bool()).Set(False)
        if p.IsA(UsdPhysics.Joint):
            UsdPhysics.Joint(p).CreateJointEnabledAttr(False)


def sdf_bool():
    from pxr import Sdf
    return Sdf.ValueTypeNames.Bool


def hide(path):
    p = _stage().GetPrimAtPath(path)
    if p.IsValid():
        UsdGeom.Imageable(p).MakeInvisible()


def hide_placeholder(parent, keep="model"):
    """Hide every geometry prim under parent except the referenced model (and lights/cameras).
    Returns (hidden placeholder gprims, visible model gprims)."""
    st = _stage()
    hidden = shown = 0
    for p in Usd.PrimRange(st.GetPrimAtPath(parent)):
        rel = str(p.GetPath())[len(parent):]
        if not p.IsA(UsdGeom.Gprim):
            continue
        if rel.startswith(f"/{keep}"):
            shown += 1
        else:
            UsdGeom.Imageable(p).MakeInvisible(); hidden += 1
    return hidden, shown


_MPU = {}


def units(usd):
    """metersPerUnit of an asset. Isaac People and ArchVis are authored in centimetres; our stage is
    in metres, and a plain reference does not convert, so cm assets come in 100x too big."""
    if usd not in _MPU:
        try:
            _MPU[usd] = float(UsdGeom.GetStageMetersPerUnit(Usd.Stage.Open(usd, load=Usd.Stage.LoadNone)))
        except Exception:
            _MPU[usd] = 1.0
    return _MPU[usd]


def add_model(parent, usd, scale=1.0, rot_xyz_deg=(0, 0, 0), offset=(0, 0, 0)):
    """Reference a USD model as a child of an existing (moved) prim; hide the placeholder geometry."""
    path = f"{parent}/model"
    scale = scale * units(usd)
    cfg = sim_utils.UsdFileCfg(usd_path=usd, scale=(scale, scale, scale))
    from scipy.spatial.transform import Rotation
    qx, qy, qz, qw = Rotation.from_euler("xyz", rot_xyz_deg, degrees=True).as_quat()
    cfg.func(path, cfg, translation=tuple(float(v) for v in offset), orientation=(qw, qx, qy, qz))
    strip_physics(path)
    h, s = hide_placeholder(parent)
    add_model.stats.append((parent, h, s))
    return path


add_model.stats = []


def apply(scen, cfg, a, W3, OUT, log=print):
    """Call in build_stage() after the placeholder scene exists and before sim.reset()."""
    st = _stage()
    b, f = scen.b, scen.fleet
    # -- sky and sun -----------------------------------------------------------------------
    dome = st.GetPrimAtPath("/World/Dome")
    if dome.IsValid():
        dome.GetAttribute("inputs:texture:file").Set(SKY) if dome.GetAttribute("inputs:texture:file") else \
            dome.CreateAttribute("inputs:texture:file", __import__("pxr").Sdf.ValueTypeNames.Asset).Set(SKY)
        dome.CreateAttribute("inputs:texture:format", __import__("pxr").Sdf.ValueTypeNames.Token).Set("latlong")
        dome.GetAttribute("inputs:intensity").Set(float(a.hdri))
        dome.GetAttribute("inputs:color").Set(Gf.Vec3f(1.0, 1.0, 1.0)) if dome.GetAttribute("inputs:color") else None
    # -- materials -------------------------------------------------------------------------
    mats = {k: make_mdl(k, v, s) for k, v, s in (
        ("walls", a.wall_mdl if getattr(a, "wall_mdl", "") in MDL else "concrete_smooth", (0.35, 0.35)), ("rubble", "concrete_rough", (0.7, 0.7)),
        ("slab", "concrete_formed", (0.6, 0.6)), ("floor", "carpet", (0.4, 0.4)),
        ("ground", "asphalt", (0.25, 0.25)), ("debris", "concrete_rough", (1.0, 1.0)))}
    counts = {
        "walls": bind_tree("/World/Building/walls", mats["walls"]),
        "rubble": bind_tree("/World/Building/rubble", mats["rubble"]) + bind_tree("/World/Building/tight", mats["rubble"]),
        "slabs": bind_tree("/World/Building/slab_void", mats["slab"]) + bind_tree("/World/Building/slab_low", mats["slab"]),
        "floor": bind_tree("/World/Building/floor", mats["floor"]),
        "ground": bind_tree("/World/Ground", mats["ground"]),
        "debris": bind_tree("/World/Debris", mats["debris"]) + bind_tree("/World/Quake", mats["debris"]),
    }
    # -- drones ------------------------------------------------------------------------------
    nd = 0
    for d in f.drones:
        parent = f"/World/Drones/{d.name}"
        try:
            if d.kind == "mother":
                add_model(parent, QUAD, scale=2.4)
            elif d.kind == "scout":
                add_model(parent, QUAD, scale=1.0)
            else:
                add_model(parent, CRAZYFLIE, scale=2.5)
            nd += 1
        except Exception as e:  # keep the placeholder body if a model fails to load
            log(f"[look] drone model for {d.name} failed: {type(e).__name__}: {e}")
    # -- people lying on the floor ----------------------------------------------------------------
    npl = 0
    for i, s in enumerate(b.survivors):
        try:
            add_model(f"/World/People/p{i}", PEOPLE[i % len(PEOPLE)], scale=1.0,
                      rot_xyz_deg=(90, 0, 37 * i), offset=(0, 0, -0.05))
            npl += 1
        except Exception as e:
            log(f"[look] person {i} failed: {type(e).__name__}: {e}")
    # -- office furniture in open cells (not on top of anyone) ------------------------------------
    rng = np.random.default_rng(cfg.seed + 5)
    open_ = np.argwhere((b.clear >= 2.0) & (b.clear < OUT) & (b.room_id >= 0))
    taken = [np.asarray(s["pos"], float) for s in b.survivors] + [np.asarray(dd["pos"], float) for dd in b.decoys]
    nf = 0
    if len(open_) and not getattr(a, "collapse", False):     # --collapse brings its own (dynamic) furniture
        for r in np.unique(b.room_id[b.room_id >= 0]):
            cells = open_[b.room_id[open_[:, 0], open_[:, 1]] == r]
            for _ in range(min(3, len(cells))):
                rr, cc = cells[rng.integers(len(cells))]
                xy = np.array([(cc + 0.5) * cfg.cell, (rr + 0.5) * cfg.cell])
                if any(np.hypot(*(xy - t)) < 1.2 for t in taken):
                    continue
                taken.append(xy)
                path = f"/World/Furniture/f{nf}"
                try:
                    u = FURNITURE[nf % len(FURNITURE)]
                    c = sim_utils.UsdFileCfg(usd_path=u, scale=(units(u),) * 3)
                    from scipy.spatial.transform import Rotation
                    qx, qy, qz, qw = Rotation.from_euler("z", rng.uniform(0, 360), degrees=True).as_quat()
                    tilt = Rotation.from_euler("xy", rng.normal(0, 8, 2), degrees=True)   # shaken, not tidy
                    q = (tilt * Rotation.from_quat([qx, qy, qz, qw])).as_quat()
                    c.func(path, c, translation=tuple(W3(xy, 0.0)), orientation=(q[3], q[0], q[1], q[2]))
                    strip_physics(path)
                    nf += 1
                except Exception as e:
                    log(f"[look] furniture {path} failed: {type(e).__name__}: {e}")
    for par, h, s in add_model.stats[:4] + add_model.stats[-3:]:
        log(f"[look] {par}: hid {h} placeholder prims, model has {s} geometry prims")
    log(f"[look] asset units (m per unit): { {k.rsplit('/', 1)[-1]: v for k, v in _MPU.items()} }")
    if getattr(a, "greenery", 1):
        try:
            greenery(b, cfg, W3, seed=cfg.seed, log=log)
        except Exception as e:
            log(f"[look] greenery failed: {type(e).__name__}: {e}")
    if getattr(a, "flow", 1):
        import sys as _s
        flow_setup(getattr(_s.modules.get("__main__"), "Q", None), W3, log=log)
    if getattr(a, "jagged", 0):
        try:
            chunkify(seed=cfg.seed, log=log)
        except Exception as e:
            log(f"[look] rubble reshaping failed, keeping boxes: {type(e).__name__}: {e}")
    log(f"[look] materials bound {counts}; drone models {nd}/{len(f.drones)}; people {npl}/{len(b.survivors)}; furniture {nf}")


# ---------------------------------------------------------------------------------------------
# camera: low cinematic orbit + close-ups at the storyboard beats
# ---------------------------------------------------------------------------------------------
BEATS = [(30.0, 50.0, "victimA"), (58.0, 80.0, "smoke"), (124.0, 146.0, "victimB")]
_cam = {"eye": None, "tgt": None}


def _focus_xy(kind, scen):
    f, b = scen.fleet, scen.b
    if kind == "victimA" and f.found:
        return np.asarray(f.found[0]["xy"], float)
    if kind == "victimB" and len(f.found) > 1:
        return np.asarray(f.found[1]["xy"], float)
    if kind == "smoke" and b.fires:
        return np.asarray(b.fires[0]["xy"], float)
    conf = [d for d in f.alive("micro") if d.role == "confirmer" and d.state.startswith("HOVER")]
    if conf:
        return np.asarray(conf[0].pos, float)
    return None


def camera(t_render, scen, cfg, W3):
    """Returns (eye, target) in Isaac world coordinates; smoothed so cuts are never jumpy."""
    b, f = scen.b, scen.fleet
    cx, cy = (b.x0 + b.x1) / 2 * cfg.cell, (b.y0 + b.y1) / 2 * cfg.cell
    ang = math.radians(-100 + 30 * math.sin(t_render / 35))
    eye = np.array([cx + 21 * math.cos(ang), -cy + 21 * math.sin(ang), 11.0])
    tgt = np.array([cx, -cy, 0.5])
    for t0, t1, kind in BEATS:
        if t0 <= f.t <= t1:
            xy = _focus_xy(kind, scen)
            if xy is not None:
                p = np.array(W3(xy, 0.4))
                side = math.radians(-120 + 20 * math.sin(t_render / 8))
                eye = p + np.array([5.0 * math.cos(side), 5.0 * math.sin(side), 3.6 if kind != "smoke" else 5.5])
                tgt = p
            break
    if _cam["eye"] is None:
        _cam["eye"], _cam["tgt"] = eye, tgt
    k = 0.04                                   # ~1 s blend at 30 render fps
    _cam["eye"] = _cam["eye"] + (eye - _cam["eye"]) * k
    _cam["tgt"] = _cam["tgt"] + (tgt - _cam["tgt"]) * k
    return list(_cam["eye"]), list(_cam["tgt"])


# ---------------------------------------------------------------------------------------------
# rubble: jagged concrete chunks instead of perfect boxes (visual only; colliders stay boxes)
# ---------------------------------------------------------------------------------------------
def _jagged_hull(size, rng, flat):
    """Triangle mesh of an irregular convex chunk that fits inside a box of `size` (centred at 0).
    Corners are cut back by random amounts and faces get random bumps, so no two pieces match.
    `flat` pieces (slab panels) keep their thickness but get broken, uneven outlines."""
    from scipy.spatial import ConvexHull
    h = np.asarray(size, float) / 2
    pts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                cut = rng.uniform(0.05, 0.45, 3) * (np.array([1, 1, 0.25]) if flat else 1)
                pts.append(np.array([sx, sy, sz]) * h * (1 - cut))
    n_face = 18 if flat else 14
    for _ in range(n_face):                         # bumps on the faces: broken, not sawn
        v = rng.uniform(-1, 1, 3)
        ax = rng.integers(0, 2 if flat else 3)
        v[ax] = np.sign(v[ax] or 1)
        depth = rng.uniform(0.0, 0.35 if not flat else 0.25)
        q = v * h
        q[ax] *= (1 - depth)
        pts.append(q)
    pts = np.array(pts)
    hull = ConvexHull(pts)
    return pts, hull.simplices, hull.equations[:, :3]


def chunkify(groups=("panels", "chunks", "walltops"), seed=7, log=print):
    from pxr import Sdf, Vt
    st = _stage()
    rng = np.random.default_rng(seed)
    mat = make_mdl("rubble_chunks", "concrete_rough", (0.8, 0.8))
    mat_slab = make_mdl("rubble_slab", "concrete_formed", (0.6, 0.6))
    bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_], useExtentsHint=False)
    n = 0
    for g in groups:
        root = st.GetPrimAtPath(f"/World/Quake/{g}")
        if not root.IsValid():
            continue
        for piece in root.GetChildren():
            if not piece.GetName().startswith("p_"):
                continue
            # the box's size in the piece's own frame
            rng_box = bbc.ComputeUntransformedBound(piece).ComputeAlignedRange()
            size = np.array(rng_box.GetSize())
            if not np.all(np.isfinite(size)) or size.min() <= 0:
                continue
            ctr = np.array(rng_box.GetMidpoint())
            flat = size.min() < 0.35 * np.sort(size)[1]
            # orient so the thin axis is z for _jagged_hull, then rotate back
            order = np.argsort(-size) if flat else np.arange(3)
            pts, tris, _ = _jagged_hull(size[order], rng, flat)
            inv = np.argsort(order)
            pts = pts[:, inv] + ctr
            # outward winding
            c = pts.mean(0)
            tri = []
            for t in tris:
                a, b, d = pts[t[0]], pts[t[1]], pts[t[2]]
                if np.dot(np.cross(b - a, d - a), a - c) < 0:
                    t = [t[0], t[2], t[1]]
                tri.append(t)
            tri = np.array(tri)
            mesh = UsdGeom.Mesh.Define(st, f"{piece.GetPath()}/chunk")
            mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, q)) for q in pts]))
            mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(tri)))
            mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for i in tri.ravel()]))
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreateDoubleSidedAttr(True)
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
                UsdShade.Material(st.GetPrimAtPath(mat_slab if g == "panels" else mat)), UsdShade.Tokens.strongerThanDescendants)
            for q in Usd.PrimRange(piece):      # hide the box (it still collides: visibility is not physics)
                if q.IsA(UsdGeom.Gprim) and q.GetName() != "chunk":
                    UsdGeom.Imageable(q).MakeInvisible()
            n += 1
    log(f"[look] rubble: {n} pieces re-shaped as jagged concrete (box colliders unchanged)")
    return n


# ---------------------------------------------------------------------------------------------
# greenery around the building (trees, shrubs, lawn, paving by the entrance)
# ---------------------------------------------------------------------------------------------
VEG = f"{NV}/Assets/Vegetation"
TREES = [f"{VEG}/Trees/{t}.usd" for t in ("Black_Oak", "Red_Oak", "Gray_Birch", "White_Pine", "Shumard_Oak", "Scarlet_Oak")]
SHRUBS = [f"{VEG}/Shrub/{t}.usd" for t in ("Cedar_Shrub", "Pearl_Bush", "Burning_Bush", "Fountain_Grass_Tall", "Pampas_Grass", "Switchgrass")]
GROUND_MDL = {"grass": f"{NV}/Materials/Base/Natural/Grass_Countryside.mdl",
              "paving": f"{NV}/Materials/vMaterials_2/Ground/Paving_Stones.mdl"}


def greenery(b, cfg, W3, seed=1, log=print):
    from scipy.spatial.transform import Rotation
    rng = np.random.default_rng(seed)
    x0, x1, y0, y1 = b.x0 * cfg.cell, b.x1 * cfg.cell, b.y0 * cfg.cell, b.y1 * cfg.cell
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    # lawn under everything (the building's ground and floor sit on top of it)
    lawn = sim_utils.CuboidCfg(size=(x1 - x0 + 120, y1 - y0 + 120, 0.1),
                               visual_material=sim_utils.MdlFileCfg(mdl_path=GROUND_MDL["grass"], project_uvw=True,
                                                                    texture_scale=(0.15, 0.15)))
    lawn.func("/World/Lawn", lawn, translation=tuple(W3((cx, cy), -0.32)))
    # paving apron by the west entrance (incident command post side)
    pav = sim_utils.CuboidCfg(size=(9.0, y1 - y0 + 8, 0.06),
                              visual_material=sim_utils.MdlFileCfg(mdl_path=GROUND_MDL["paving"], project_uvw=True,
                                                                   texture_scale=(0.5, 0.5)))
    pav.func("/World/Paving", pav, translation=tuple(W3((x0 - 4.5, cy), -0.04)))
    placed = []

    def ok(xy, gap):
        if x0 - 12 < xy[0] < x0 and y0 - 6 < xy[1] < y1 + 6:   # keep west entrance, command post, flight path clear
            return False
        if x0 - 1.5 < xy[0] < x1 + 1.5 and y0 - 1.5 < xy[1] < y1 + 1.5:
            return False
        return all(np.hypot(*(xy - p)) > gap for p in placed)

    def put(kind, i, u, xy, s):
        c = sim_utils.UsdFileCfg(usd_path=u, scale=(units(u) * s,) * 3)
        q = Rotation.from_euler("z", rng.uniform(0, 360), degrees=True).as_quat()
        path = f"/World/Greenery/{kind}{i}"
        c.func(path, c, translation=tuple(W3(xy, -0.27)), orientation=(q[3], q[0], q[1], q[2]))
        strip_physics(path)
        placed.append(xy)

    nt = ns = 0
    for _ in range(400):
        if nt >= 16:
            break
        ang = rng.uniform(0, 2 * np.pi)
        r = rng.uniform(8, 22)
        xy = np.array([cx + ((x1 - x0) / 2 * 1.05 + r) * np.cos(ang), cy + ((y1 - y0) / 2 * 1.05 + r) * np.sin(ang)])
        if ok(xy, 5.0):
            try:
                put("tree", nt, TREES[nt % len(TREES)], xy, rng.uniform(0.8, 1.15)); nt += 1
            except Exception as e:
                log(f"[look] tree failed: {type(e).__name__}: {e}"); break
    for _ in range(600):
        if ns >= 28:
            break
        t, off = rng.uniform(0, 1), rng.uniform(1.8, 4.0)
        xy = [np.array([x0 + t * (x1 - x0), y0 - off]), np.array([x0 + t * (x1 - x0), y1 + off]),
              np.array([x1 + off, y0 + t * (y1 - y0)]), np.array([x0 - off, y0 + t * (y1 - y0)])][rng.integers(4)]
        if ok(xy, 1.6):
            try:
                put("shrub", ns, SHRUBS[ns % len(SHRUBS)], xy, rng.uniform(0.8, 1.3)); ns += 1
            except Exception as e:
                log(f"[look] shrub failed: {type(e).__name__}: {e}"); break
    log(f"[look] greenery: lawn + paving, {nt} trees, {ns} shrubs")


# ---------------------------------------------------------------------------------------------
# Omni Flow volumetric dust (collapse) and fire/smoke (storyboard fire)
# ---------------------------------------------------------------------------------------------
FLOW = {"emitters": [], "fire": None}


def _flow_preset():
    import carb
    import omni.kit.app
    em = omni.kit.app.get_app().get_extension_manager()
    em.set_extension_enabled_immediate("omni.flowusd", True)
    root = em.get_extension_path_by_module("omni.flowusd")
    carb.settings.get_settings().set_bool("/rtx/flow/enabled", True)
    carb.settings.get_settings().set_bool("/rtx/flow/pathTracingEnabled", True)
    return f"{root}/data/presets/Fire/Fire.usda"


def _setf(p, name, v):
    from pxr import Sdf
    at = p.GetAttribute(name)
    (at if at else p.CreateAttribute(name, Sdf.ValueTypeNames.Float)).Set(float(v))


def flow_effect(path, kind, pos, radius, layer):
    """Reference NVIDIA's Flow fire preset and retune it in metres.
    kind 'dust': smoke only, cool (little buoyancy), slow to clear. kind 'fire': flames + smoke."""
    st = _stage()
    prim = st.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(_flow_preset())
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
    for child in ("flowEmitterSphere", "flowSimulate", "flowOffscreen", "flowRender"):
        c = st.GetPrimAtPath(f"{path}/{child}")
        if c.IsValid() and c.GetAttribute("layer"):
            c.GetAttribute("layer").Set(int(layer))
    em = st.GetPrimAtPath(f"{path}/flowEmitterSphere")
    sim = st.GetPrimAtPath(f"{path}/flowSimulate")
    _setf(em, "radius", radius)
    _setf(sim, "densityCellSize", max(0.04, radius * 0.12))
    if kind == "dust":
        _setf(em, "fuel", 0.0); _setf(em, "temperature", 0.15); _setf(em, "smoke", 2.5); _setf(em, "coupleRateSmoke", 3.0)
        adv = st.GetPrimAtPath(f"{path}/flowSimulate/advection")
        if adv.IsValid():
            _setf(adv, "buoyancyPerTemp", 0.6)
        sm = st.GetPrimAtPath(f"{path}/flowSimulate/advection/smoke")
        if sm.IsValid():
            _setf(sm, "fade", 0.08); _setf(sm, "damping", 0.05)
        FLOW["emitters"].append(em)
    else:
        FLOW["fire"] = (prim, em)
    en = em.GetAttribute("enabled")
    if en:
        en.Set(False)
    return path


def flow_setup(Q, W3, log=print):
    """Dust emitters at the damage epicentre and along the corridor (when --collapse), plus a fire emitter."""
    try:
        if Q is not None and getattr(Q, "epi", None) is not None:
            epi = np.asarray(Q.epi, float)
            for i, off in enumerate([(0, 0), (3.5, 1.0), (-3.0, -1.5), (6.5, -0.5)]):
                flow_effect(f"/World/Flow/dust{i}", "dust", W3(epi + np.array(off), 1.2), 1.4, layer=1 + i)
        flow_effect("/World/Flow/fire", "fire", (0.0, 0.0, -50.0), 0.45, layer=9)
        log(f"[look] Flow: {len(FLOW['emitters'])} dust emitters, fire emitter ready")
    except Exception as e:
        log(f"[look] Flow unavailable ({type(e).__name__}: {e}); keeping point dust/fire")


def flow_update(t_render, scen, Q, W3):
    """Per render frame: dust on from the quake start for 25 s; fire follows the storyboard fire."""
    pre = float(getattr(Q, "PRE", 2.0)) if Q is not None else 0.0
    on = pre <= t_render <= pre + 25.0
    for em in FLOW["emitters"]:
        at = em.GetAttribute("enabled")
        if at and bool(at.Get()) != on:
            at.Set(on)
    if FLOW["fire"] is not None:
        prim, em = FLOW["fire"]
        if scen.b.fires:
            UsdGeom.Xformable(prim).GetOrderedXformOps()[0].Set(Gf.Vec3d(*map(float, W3(scen.b.fires[0]["xy"], 0.3))))
            at = em.GetAttribute("enabled")
            if at and not at.Get():
                at.Set(True)
