"""Loader for the render-grade heavy-lift drone built in Blender (``isaac/tools/blender_drone.py``).

    from isaac.blender_drone import build
    rotors = build(stage, "/World/Drones/d1/Model", did="d1", emissive=drone_viz.EMISSIVE)   # None -> fall back

The asset (``isaac/assets/drone/heavy_sar.usdc`` + ``textures/``, UsdPreviewSurface materials) is referenced
straight onto the Model prim, so its parts sit at ``Model/<part>`` (``Model/Strobe`` blinks through
``DroneViz._strobe``) and Model's own translate (-skid_offset) / scale ops stay in charge.  Metres, Z up, +X forward,
skid bottom at z = 0.  Each ``Rotor{i}_{lvl}`` Xform (hub on the motor axis) gets a RotateZ op; it holds ``Blades``
(the two blades), ``Hub`` (hub + motor bell), ``Blur_lo`` / ``Blur_hi`` (prop-blur discs, hidden in the asset).
Returns rotor dicts in drone_viz's format: {op, sign, angle, state, name, blades, lo, hi}.
"""
from __future__ import annotations

import os
import re

ASSET = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets", "drone", "heavy_sar.usdc")
EMISSIVE_KEYS = {"M_NavRed": "nav_red", "M_NavGreen": "nav_green", "M_Strobe": "strobe"}   # asset material -> key
_ROTOR = re.compile(r"Rotor(\d+)_([01])")


def available(url=None):
    return os.path.exists(url or ASSET)


def build(stage, model_path, did="d1", log=print, emissive=None, url=None):
    """Reference the Blender drone onto ``model_path``. Returns the rotor dicts, or None (asset missing/broken:
    the caller builds its own model)."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade
    url = url or ASSET
    if not os.path.exists(url):
        log(f"[blender_drone] {url} missing; using the built-in model")
        return None
    prim = stage.GetPrimAtPath(model_path)
    if not prim or not prim.IsValid():
        prim = UsdGeom.Xform.Define(stage, model_path).GetPrim()
    refs = prim.GetReferences()
    try:
        refs.AddReference(url)
        if not prim.GetChildren():
            raise RuntimeError("composed to an empty prim")
        rotors = []
        for p in Usd.PrimRange(prim):
            name = p.GetName()
            if _ROTOR.fullmatch(name) and p.IsA(UsdGeom.Xformable):
                rotors.append(p)
            elif emissive and p.IsA(UsdShade.Material) and name in EMISSIVE_KEYS and EMISSIVE_KEYS[name] in emissive:
                em = emissive[EMISSIVE_KEYS[name]]
                for ch in Usd.PrimRange(p):
                    if ch.IsA(UsdShade.Shader) and UsdShade.Shader(ch).GetIdAttr().Get() == "UsdPreviewSurface":
                        UsdShade.Shader(ch).CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
                            Gf.Vec3f(*map(float, em)))
        if not rotors:
            raise RuntimeError("no Rotor{i}_{lvl} prims")
        out = []
        for p in sorted(rotors, key=lambda q: q.GetName()):
            i, lvl = map(int, _ROTOR.fullmatch(p.GetName()).groups())
            a = p.GetAttribute("userProperties:spinSign")
            sign = int(a.Get()) if a and a.HasValue() else (1 if i % 2 == 0 else -1) * (1 if lvl else -1)
            op = UsdGeom.Xformable(p).AddRotateZOp(UsdGeom.XformOp.PrecisionFloat, "spin")   # after the hub translate
            kids = {k: p.GetChild(k) for k in ("Blades", "Blur_lo", "Blur_hi")}
            ok = all(k.IsValid() for k in kids.values())
            out.append(dict(op=op, sign=sign, angle=0.0, state="still", name=p.GetName(),
                            blades=kids["Blades"] if ok else None, lo=kids["Blur_lo"] if ok else None,
                            hi=kids["Blur_hi"] if ok else None))
        log(f"[blender_drone] {model_path}: {os.path.basename(url)} ({len(out)} rotors)")
        return out
    except Exception as e:   # noqa: BLE001
        log(f"[blender_drone] {url} failed ({type(e).__name__}: {e}); using the built-in model")
        try:
            refs.RemoveReference(Sdf.Reference(url))
        except Exception:
            pass
        return None
