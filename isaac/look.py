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
    hide(f"{parent}/geometry")
    return path


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
    log(f"[look] asset units (m per unit): { {k.rsplit('/', 1)[-1]: v for k, v in _MPU.items()} }")
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
