"""Grid-site look under ``--photoreal 1`` (``--grid-look 1``, the default there).

The old grid render (pt2) read as a toy: near-white MDL concrete "cards", saturated orange MDL brick, an orange
sun and haze. This module, called from ``photoreal.dress`` on non-urban sites after the rubble is dressed:

* re-materials the irregular rubble pieces (``/World/RubbleMesh``) and the scattered stones (``/World/Debris``)
  with the baked, dusty textures of the Blender facade kit (``isaac/assets/facade_kit/textures``: concrete
  ~0.29 linear albedo, broken-concrete "fracture" ~0.37, brick ~(0.28, 0.20, 0.17)), box-projected UVs per
  face, a few tints so no two neighbouring pieces match. Geometry, poses and colliders are unchanged.
* turns the sun from orange (1.0, 0.80, 0.60) to a dusty afternoon white (1.0, 0.91, 0.80).
* sets a real-camera grade with neutral dusty haze (scratchpad look research): saturation 0.85, contrast 1.05,
  slight cool gain, AO, fine grain, light vignette. Exposure is left as it is.

Everything is a visual change only: nothing the planner or the LiDAR reads changes.
"""
from __future__ import annotations

import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
KIT_TEX = os.path.join(HERE, "assets", "facade_kit", "textures")
ROOT = "/World/GridLook"
TILE_M = {"concrete": 2.4, "fracture": 1.6, "brick": 1.4}       # metres per texture repeat
# albedo multipliers (UsdUVTexture scale) per variant: dusty greys, a darker wet-ish grey, a warm dust cast
TINTS = {
    "concrete": ((1.00, 1.00, 1.00), (0.82, 0.82, 0.81), (0.96, 0.92, 0.86), (0.74, 0.74, 0.74)),
    "fracture": ((0.88, 0.87, 0.85), (0.78, 0.76, 0.73)),
    "brick": ((0.92, 0.92, 0.92), (0.80, 0.84, 0.86)),            # 0.80/0.84/0.86 = dust over the red
}
SUN_RGB = (1.0, 0.86, 0.70)   # look check 02:40: a white sun under the blue evening dome cast the frame blue
GRID_GRADE = {
    "/rtx/post/colorcorr/enabled": True,
    "/rtx/post/colorcorr/saturation": [0.95, 0.95, 0.95],
    "/rtx/post/colorcorr/contrast": [1.08, 1.08, 1.08],
    "/rtx/post/colorcorr/gain": [1.0, 1.0, 1.0],
    "/rtx/post/colorcorr/offset": [0.0, 0.0, 0.0],
    "/rtx/ambientOcclusion/enabled": True, "/rtx/ambientOcclusion/rayLength": 2.0,
    "/rtx/post/tvNoise/enabled": True, "/rtx/post/tvNoise/enableFilmGrain": True,
    "/rtx/post/tvNoise/grainAmount": 0.018, "/rtx/post/tvNoise/grainSize": 1.1,
    "/rtx/post/tvNoise/enableVignetting": True, "/rtx/post/tvNoise/vignettingSize": 107.0,
    "/rtx/post/tvNoise/vignettingStrength": 0.07,
    "/rtx/post/tvNoise/enableScanlines": False, "/rtx/post/tvNoise/enableScrollBug": False,
    "/rtx/post/tvNoise/enableVignettingFlickering": False, "/rtx/post/tvNoise/enableGhostFlickering": False,
    "/rtx/post/tvNoise/enableWaveDistortion": False, "/rtx/post/tvNoise/enableVerticalLines": False,
    "/rtx/post/tvNoise/enableRandomSplotches": False,
}   # haze: photoreal's own warm distance haze is kept (a blue-grey fog colour washed the frame out, look check 02:40)


def box_uv(P, F, tile=2.0):
    """Face-varying UVs by box projection: each triangle is projected on the plane facing its normal's
    dominant axis, in metres / ``tile``. Returns (len(F) * 3, 2) float32 in face-vertex order."""
    P = np.asarray(P, np.float64)
    F = np.asarray(F, np.int64).reshape(-1, 3)
    a, b, c = P[F[:, 0]], P[F[:, 1]], P[F[:, 2]]
    ax = np.argmax(np.abs(np.cross(b - a, c - a)), axis=1)                # 0: x-facing, 1: y, 2: z
    keep = np.array([[1, 2], [0, 2], [0, 1]])[ax]                          # (M, 2) axes kept per face
    V = P[F]                                                               # (M, 3, 3)
    uv = np.take_along_axis(V, keep[:, None, :].repeat(3, 1), axis=2)     # (M, 3, 2)
    return (uv.reshape(-1, 2) / float(tile)).astype(np.float32)


def _material(stage, path, tex, tint, rough_scale=1.0):
    from pxr import Gf, Sdf, UsdShade
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/PBR")
    sh.CreateIdAttr("UsdPreviewSurface")
    rd = UsdShade.Shader.Define(stage, path + "/st")
    rd.CreateIdAttr("UsdPrimvarReader_float2")
    rd.CreateInput("varname", Sdf.ValueTypeNames.String).Set("st")
    for kind, inp, typ, cs in (("albedo", "diffuseColor", Sdf.ValueTypeNames.Color3f, "sRGB"),
                               ("rough", "roughness", Sdf.ValueTypeNames.Float, "raw"),
                               ("normal", "normal", Sdf.ValueTypeNames.Normal3f, "raw")):
        tx = UsdShade.Shader.Define(stage, f"{path}/{kind}")
        tx.CreateIdAttr("UsdUVTexture")
        tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(os.path.join(KIT_TEX, f"{tex}_{kind}.png")))
        tx.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set(cs)
        tx.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        tx.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
        if kind == "albedo":
            tx.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*map(float, tint), 1.0))
            o = tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        elif kind == "normal":
            tx.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(2, 2, 2, 1))
            tx.CreateInput("bias", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(-1, -1, -1, 0))
            o = tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        else:
            if rough_scale != 1.0:
                tx.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(rough_scale, rough_scale, rough_scale, 1))
            o = tx.CreateOutput("r", Sdf.ValueTypeNames.Float)
        sh.CreateInput(inp, typ).ConnectToSource(o)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def materials(stage, log=print):
    """{texture set: [Material, ...]} or {} when the facade kit textures are missing."""
    if not all(os.path.exists(os.path.join(KIT_TEX, f"{t}_albedo.png")) for t in TINTS):
        log(f"[grid-look] facade kit textures missing in {KIT_TEX}; rubble keeps its MDL materials")
        return {}
    return {t: [_material(stage, f"{ROOT}/Looks/{t}_{i}", t, tint) for i, tint in enumerate(tints)]
            for t, tints in TINTS.items()}


def _set_st(mesh, tile):
    from pxr import Sdf, UsdGeom, Vt
    P = np.asarray(mesh.GetPointsAttr().Get(), np.float32)
    cnt = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
    if not len(P) or not len(cnt) or not np.all(cnt == 3):
        return False
    F = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
    pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    pv.Set(Vt.Vec2fArray.FromNumpy(np.ascontiguousarray(box_uv(P, F, tile))))
    return True


def rubble(stage, looks, log=print, seed=0):
    """Bind the dusty kit materials to the irregular rubble pieces and the debris stones."""
    from pxr import UsdGeom, UsdShade
    if not looks:
        return 0
    rng = np.random.default_rng(5100 + int(seed))
    n = 0
    root = stage.GetPrimAtPath("/World/RubbleMesh")
    for prim in (root.GetChildren() if root.IsValid() else []):              # per pile
        for piece in prim.GetChildren():                                     # per piece: .../geo, .../rebar
            geo = stage.GetPrimAtPath(piece.GetPath().AppendChild("geo"))
            if not geo.IsValid():
                continue
            brick = piece.GetName().startswith(("brick",)) or "brick" in _bound_name(geo)
            tex = "brick" if brick else ("fracture" if rng.random() < 0.28 else "concrete")
            if _set_st(UsdGeom.Mesh(geo), TILE_M[tex]):
                mat = looks[tex][int(rng.integers(0, len(looks[tex])))]
                UsdShade.MaterialBindingAPI.Apply(geo).Bind(mat, UsdShade.Tokens.strongerThanDescendants)
                n += 1
    m = 0
    deb = stage.GetPrimAtPath("/World/Debris")
    for geo in (deb.GetChildren() if deb.IsValid() else []):
        name = geo.GetName()
        tex = "brick" if "brick" in name else ("fracture" if name.endswith("concrete2") else "concrete")
        if _set_st(UsdGeom.Mesh(geo), TILE_M[tex] * 0.5):
            mat = looks[tex][int(rng.integers(0, len(looks[tex])))]
            UsdShade.MaterialBindingAPI.Apply(geo).Bind(mat, UsdShade.Tokens.strongerThanDescendants)
            m += 1
    log(f"[grid-look] rubble: {n} pieces and {m} debris meshes on the dusty facade-kit textures "
        f"(concrete / broken concrete / brick, {sum(len(v) for v in TINTS.values())} tints)")
    return n


def _bound_name(prim):
    from pxr import UsdShade
    try:
        mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        return mat.GetPath().name.lower() if mat else ""
    except Exception:
        return ""


def sun(stage, log=print, rgb=SUN_RGB):
    from pxr import Gf, UsdLux
    p = stage.GetPrimAtPath("/World/Sun")
    if p.IsValid():
        UsdLux.DistantLight(p).GetColorAttr().Set(Gf.Vec3f(*rgb))
        log(f"[grid-look] sun colour {rgb} (was orange)")


def grade(log=print, extra=None):
    try:
        import carb
        s = carb.settings.get_settings()
    except Exception as e:                                  # noqa: BLE001  (dry run / plain pxr)
        log(f"[grid-look] grade skipped ({type(e).__name__})")
        return 0
    ok = 0
    for k, v in {**GRID_GRADE, **(extra or {})}.items():
        try:
            if isinstance(v, bool):
                s.set_bool(k, v)
            elif isinstance(v, int):
                s.set_int(k, v)
            elif isinstance(v, float):
                s.set_float(k, v)
            else:
                s.set(k, v)
            ok += 1
        except Exception:                                   # noqa: BLE001
            pass
    log(f"[grid-look] grade: {ok}/{len(GRID_GRADE)} settings (saturation 0.95, contrast 1.08, AO, grain; sky, sun strength and haze as photoreal)")
    return ok


def apply(stage, sc, a, log=print):
    looks = materials(stage, log)
    rubble(stage, looks, log, seed=int(getattr(sc, "seed", 0) or 0))
    sun(stage, log)
    grade(log)
