"""``--look pretty``: presentability layer for the rubble grid. Visual only: colliders, LiDAR targets, the mission
and the state.json contract are untouched.

Adapted (not copied) from the team's GPU-verified skysearch ``v2-earthquake:isaac/look.py`` (Isaac Sim 6.0.1 /
Isaac Lab 3.0, L40S); see docs/ISAAC_VERIFIED_APIS.md for line references.

* HDRI sky: ``evening_road_01_4k.hdr`` on the existing /World/Sky dome (texture:file + latlong), dusk light.
* MDL materials with tri-planar projection (``project_uvw``) so plain geometry textures well:
  Concrete_Rough / Brick_Wall_Brown on rubble, Dirt on the terrain, Gravel on the apron, Asphalt on the pad.
  Isaac Lab route: ``MdlFileCfg`` (verified). Kit route: ``CreateMdlMaterialPrim`` (UNVERIFIED).
* Chipped rubble: each box gets a visual "truncated, jittered box" twin (numpy, no scipy); the boxes stay as the
  invisible PhysX colliders (visibility is not physics), so LiDAR hits the same faces it always did. Corner cuts
  are at most 20% of the piece size, so points sit on the visible faces except at the chipped corners.
* Omni Flow dust under each drone when it is near the ground (landing / take-off), from NVIDIA's Fire preset
  retuned as cool smoke (verified on 6.0.1 in the team's quake3 clips).
* Path tracing switch lives in ``compat.set_render_mode`` (``--rtx pathtracing --spp 16``).

Every step is guarded: a failure logs one line and leaves the basic look in place.
"""
from __future__ import annotations

import numpy as np

MDL_FILES = {   # relative to <assets root>/NVIDIA; all present on 6.0 (skysearch isaac/assets_found.json)
    "concrete": ("Materials/Base/Masonry/Concrete_Rough.mdl", 0.7),
    "brick": ("Materials/Base/Masonry/Brick_Wall_Brown.mdl", 0.5),
    "terrain": ("Materials/Base/Natural/Dirt.mdl", 0.25),
    "apron": ("Materials/Base/Stone/Gravel.mdl", 0.3),
    "pad": ("Materials/Base/Natural/Asphalt.mdl", 0.3),
}
SKY = "Assets/Skies/Clear/evening_road_01_4k.hdr"


# ---------------------------------------------------------------------------- geometry (numpy only)
def chipped_box(rng, cut=(0.04, 0.2), noise=0.015):
    """Truncated unit cube ([-0.5, 0.5]^3) with random corner cuts and vertex jitter.
    Returns (V (24, 3), T (44, 3)) with outward-wound triangles."""
    corners = [(sx, sy, sz) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    V, idx = [], {}
    for ci, c in enumerate(corners):
        a = rng.uniform(cut[0], cut[1], 3)
        for ax in range(3):
            v = 0.5 * np.array(c, float)
            v[ax] -= c[ax] * a[ax]
            idx[(ci, ax)] = len(V)
            V.append(v)
    V = np.array(V) + rng.normal(0.0, noise, (24, 3))
    tris = [[idx[(ci, 0)], idx[(ci, 1)], idx[(ci, 2)]] for ci in range(8)]          # corner facets
    for k in range(3):                                                             # octagonal faces
        for s in (-1, 1):
            ids = [idx[(ci, ax)] for ci, c in enumerate(corners) if c[k] == s for ax in range(3) if ax != k]
            P = V[ids]
            ctr = P.mean(0)
            u, w = [a for a in range(3) if a != k]
            order = [ids[i] for i in np.argsort(np.arctan2(P[:, w] - ctr[w], P[:, u] - ctr[u]))]
            tris += [[order[0], order[i], order[i + 1]] for i in range(1, len(order) - 1)]
    T = np.array(tris)
    a, b, d = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    flip = (((a + b + d) / 3 - V.mean(0)) * np.cross(b - a, d - a)).sum(1) < 0
    T[flip] = T[flip][:, [0, 2, 1]]
    return V, T


def merge_pieces(pieces, seed=0):
    """pieces: [(center, R (column convention), size, rgb)] -> (points, normals, colours), one triangle per 3 rows."""
    rng = np.random.default_rng(seed)
    P, N, C = [], [], []
    for center, R, size, rgb in pieces:
        V, T = chipped_box(rng)
        W = np.asarray(center, float) + (np.asarray(R, float) @ (V * np.asarray(size, float)).T).T
        tri = W[T]                                                   # (44, 3, 3)
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        P.append(tri.reshape(-1, 3))
        N.append(np.repeat(n, 3, 0))
        C.append(np.tile(np.asarray(rgb, float), (len(T) * 3, 1)))
    if not P:
        z = np.zeros((0, 3), np.float32)
        return z, z, z
    return (np.concatenate(P).astype(np.float32), np.concatenate(N).astype(np.float32),
            np.concatenate(C).astype(np.float32))


# ---------------------------------------------------------------------------- USD / Isaac parts
def _nvidia_root():
    from isaac import compat
    r = compat.assets_root()
    return f"{r}/NVIDIA" if r else None


def make_mdl(stage, mode, name, url, scale):
    """MDL material prim at /World/Looks/mdl_<name>. Returns the UsdShade.Material or raises."""
    from pxr import Sdf, UsdShade
    path = f"/World/Looks/mdl_{name}"
    if mode == "lab":    # verified: look.py:60-64
        import isaaclab.sim as sim_utils
        cfg = sim_utils.MdlFileCfg(mdl_path=url, project_uvw=True, texture_scale=(scale, scale))
        cfg.func(path, cfg)
    else:                # Kit command used by Isaac's own material helpers; UNVERIFIED here
        import omni.kit.commands
        mtl = url.rsplit("/", 1)[-1].rsplit(".", 1)[0]
        omni.kit.commands.execute("CreateMdlMaterialPrim", mtl_url=url, mtl_name=mtl, mtl_path=path)
        sh = stage.GetPrimAtPath(path + "/Shader")
        if sh.IsValid():
            sh.CreateAttribute("inputs:project_uvw", Sdf.ValueTypeNames.Bool).Set(True)
            sh.CreateAttribute("inputs:texture_scale", Sdf.ValueTypeNames.Float2).Set((scale, scale))
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"material {path} not created")
    return UsdShade.Material(prim)


def chip_rubble(stage, sc, mats=None, log=print):
    """Visual chipped twins of every rubble box, merged per pile and material; hides the boxes (colliders stay)."""
    from pxr import Gf, Sdf, UsdGeom, Vt

    from isaac.scene import piece_pose
    out = {}
    for p in sc.piles:
        groups = {}
        for k, pc in enumerate(p.pieces):
            prim = stage.GetPrimAtPath(f"/World/Rubble/{p.name}/{pc.kind}_{k:03d}")
            rgb = (0.5, 0.5, 0.5)
            if prim.IsValid():
                c = UsdGeom.Gprim(prim).GetDisplayColorAttr().Get()
                rgb = tuple(c[0]) if c else rgb
            center, _, size, R = piece_pose(pc)
            groups.setdefault(pc.material, []).append((center, R, size, rgb))
        for mat, pieces in groups.items():
            P, N, C = merge_pieces(pieces, seed=sc.seed * 101 + sum(map(ord, p.name)))
            path = f"/World/RubbleLook/{p.name}_{mat}"
            m = UsdGeom.Mesh.Define(stage, path)
            m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(P))
            m.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(P) // 3, 3, np.int32)))
            m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.arange(len(P), dtype=np.int32)))
            m.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(N))
            m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
            m.CreateSubdivisionSchemeAttr("none")
            m.CreateExtentAttr([Gf.Vec3f(*map(float, P.min(0))), Gf.Vec3f(*map(float, P.max(0)))])
            UsdGeom.PrimvarsAPI(m).CreatePrimvar("displayColor", Sdf.ValueTypeNames.Color3fArray,
                                                 UsdGeom.Tokens.vertex).Set(Vt.Vec3fArray.FromNumpy(C))
            if mats is not None:
                mats.bind(m.GetPrim(), f"rubble_look_{mat}", (0.5, 0.5, 0.5), primvar_fallback=tuple(C.mean(0)))
            out.setdefault(mat, []).append(path)
        UsdGeom.Imageable(stage.GetPrimAtPath(f"/World/Rubble/{p.name}")).MakeInvisible()
    log(f"[look] chipped rubble: {sum(len(v) for v in out.values())} merged meshes; boxes hidden (still colliders)")
    return out


def apply(stage, sc, mode, hdri=900.0, chip=True, mdl=True, log=print):
    """Call after scene.build() and before physics starts. Returns a dict of what was applied."""
    from pxr import Gf, Sdf, UsdGeom

    from isaac import compat
    from isaac.scene import bind
    rep = {"hdri": False, "mdl": 0, "chipped": 0}
    root = _nvidia_root()
    from isaac import scene
    mats = getattr(scene.build, "materials", None)
    looks = {}
    if chip:
        try:
            looks = chip_rubble(stage, sc, mats, log)
            rep["chipped"] = sum(len(v) for v in looks.values())
        except Exception as e:   # noqa: BLE001
            log(f"[look] chipped rubble failed, keeping boxes: {type(e).__name__}: {e}")
    if root and hdri:
        url = f"{root}/{SKY}"
        dome = stage.GetPrimAtPath("/World/Sky")
        if dome.IsValid() and (compat.url_exists(url) or "://" not in url):
            try:      # look.py:160-166
                dome.CreateAttribute("inputs:texture:file", Sdf.ValueTypeNames.Asset).Set(url)
                dome.CreateAttribute("inputs:texture:format", Sdf.ValueTypeNames.Token).Set("latlong")
                dome.GetAttribute("inputs:intensity").Set(float(hdri))
                dome.GetAttribute("inputs:color").Set(Gf.Vec3f(1.0, 1.0, 1.0))
                rep["hdri"] = True
            except Exception as e:   # noqa: BLE001
                log(f"[look] HDRI sky failed: {type(e).__name__}: {e}")
        else:
            log(f"[look] HDRI not found at {url}; keeping the plain dome")
    if root and mdl:
        targets = {
            "concrete": looks.get("concrete") or [f"/World/Rubble/{p.name}" for p in sc.piles],
            "brick": looks.get("brick", []),
            "terrain": ["/World/Terrain"], "apron": ["/World/Apron"], "pad": ["/World/Staging/Pad"],
        }
        for key, paths in targets.items():
            if not paths:
                continue
            rel, scale = MDL_FILES[key]
            try:
                mat = make_mdl(stage, mode, key, f"{root}/{rel}", scale)
            except Exception as e:   # noqa: BLE001
                log(f"[look] MDL {key} failed ({type(e).__name__}: {e}); keeping the preview material")
                continue
            from pxr import Usd, UsdShade
            for path in paths:
                prim = stage.GetPrimAtPath(path)
                if not prim.IsValid():
                    continue
                for q in Usd.PrimRange(prim):
                    if q.IsA(UsdGeom.Gprim):
                        bind(q, mat, UsdShade.Tokens.strongerThanDescendants)
                        rep["mdl"] += 1
    log(f"[look] pretty: {rep}")
    return rep


class FlowDust:
    """Omni Flow dust puff under each drone while it is within a few metres of the ground. Guarded: ``ok`` False
    means Flow is unavailable and nothing is drawn. Preset + parameters from look.py:448-496 (verified on 6.0.1)."""

    def __init__(self, stage, drone_ids, log=print, radius=1.2):
        self.stage, self.ok, self.fx = stage, False, {}
        try:
            preset = self._preset()
            for i, did in enumerate(drone_ids):
                self.fx[did] = self._effect(f"/World/FX/dust_{did}", preset, radius, layer=1 + i)
            self.ok = True
            log(f"[look] Flow dust: {len(self.fx)} emitters")
        except Exception as e:   # noqa: BLE001
            log(f"[look] Flow unavailable ({type(e).__name__}: {e}); no dust")

    @staticmethod
    def _preset():
        import carb
        import omni.kit.app
        em = omni.kit.app.get_app().get_extension_manager()
        em.set_extension_enabled_immediate("omni.flowusd", True)
        root = em.get_extension_path_by_module("omni.flowusd")
        cs = carb.settings.get_settings()
        cs.set_bool("/rtx/flow/enabled", True)
        cs.set_bool("/rtx/flow/pathTracingEnabled", True)
        return f"{root}/data/presets/Fire/Fire.usda"

    def _setf(self, prim, name, v):
        from pxr import Sdf
        if not prim.IsValid():
            return
        at = prim.GetAttribute(name)
        (at if at else prim.CreateAttribute(name, Sdf.ValueTypeNames.Float)).Set(float(v))

    def _effect(self, path, preset, radius, layer):
        from pxr import Gf, UsdGeom
        st = self.stage
        prim = st.DefinePrim(path, "Xform")
        prim.GetReferences().AddReference(preset)
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        op = xf.AddTranslateOp()
        op.Set(Gf.Vec3d(0.0, 0.0, -50.0))
        for child in ("flowEmitterSphere", "flowSimulate", "flowOffscreen", "flowRender"):
            c = st.GetPrimAtPath(f"{path}/{child}")
            if c.IsValid() and c.GetAttribute("layer"):
                c.GetAttribute("layer").Set(int(layer))
        em = st.GetPrimAtPath(f"{path}/flowEmitterSphere")
        self._setf(em, "radius", radius)
        self._setf(st.GetPrimAtPath(f"{path}/flowSimulate"), "densityCellSize", max(0.04, radius * 0.12))
        for k, v in (("fuel", 0.0), ("temperature", 0.15), ("smoke", 2.5), ("coupleRateSmoke", 3.0)):
            self._setf(em, k, v)                       # cool smoke, no flame
        self._setf(st.GetPrimAtPath(f"{path}/flowSimulate/advection"), "buoyancyPerTemp", 0.6)
        sm = st.GetPrimAtPath(f"{path}/flowSimulate/advection/smoke")
        self._setf(sm, "fade", 0.08)
        self._setf(sm, "damping", 0.05)
        en = em.GetAttribute("enabled")
        if en:
            en.Set(False)
        return {"op": op, "em": em, "on": False}

    def update(self, poses, surface):
        """poses: {id: (x, y, z, ...)}; surface(x, y) -> ground z."""
        if not self.ok:
            return
        from pxr import Gf
        for did, p in poses.items():
            fx = self.fx.get(did)
            if fx is None:
                continue
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            g = float(np.asarray(surface(np.array([x]), np.array([y]))).ravel()[0])
            on = 0.25 < z - g < 3.5
            if on:
                fx["op"].Set(Gf.Vec3d(x, y, g + 0.3))
            if on != fx["on"]:
                at = fx["em"].GetAttribute("enabled")
                if at:
                    at.Set(on)
                fx["on"] = on
