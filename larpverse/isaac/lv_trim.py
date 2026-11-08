"""Crude, opt-in stage trims for the console take (visual only; sim/ and state.json untouched).

LV_CROP=1        deactivate site prims whose bounds centre is outside a disc around the main structure (radius chosen
                 so the listed piles stay in) and outside a corridor to the police van
LV_NODEBRIS=1    deactivate floor debris / gravel / scatter / litter prims (by name)
LV_NOFIGURES=1   deactivate crew / operator / police figure prims (by name; never /World/Dummies)
LV_GLOWDUMMIES=1 grey low-poly mannequins at the simulated people's TRUE positions (hidden truth, display only) with a
                 grey pin + disc at the surface; a dummy turns emissive orange-red (and stays) once state.json -- the
                 mission's OWN outputs -- shows a radar ping / candidate region or a TRAPPED-tier clip at its pile.
                 The planner, decider and acoustic models never read these prims.
"""
from __future__ import annotations

import math
import os
import re

PROTECT = ("/World/Drones", "/World/Van", "/World/Overlay", "/World/SensorFX", "/World/Looks", "/World/LvDummies",
           "/World/physicsScene", "/World/PhysicsScene", "/World/Lights", "/World/Sky", "/World/Cameras", "/World/Dummies")
DEBRIS = re.compile(r"debris|gravel|scatter|litter|pebble|rubble_small|smallrubble|clutter", re.I)
FIGURES = re.compile(r"crew|operator|police(man)?$|worker|firefighter|responder|human|person|mannequin", re.I)


def on(name):
    return os.environ.get(name) == "1"


def _centre(cache, prim):
    try:
        r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            return None, 0.0
        lo, hi = r.GetMin(), r.GetMax()
        return ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2), max(hi[0] - lo[0], hi[1] - lo[1])
    except Exception:  # noqa: BLE001
        return None, 0.0


def apply(stage, piles_xy, centre_piles, keep_piles, van_xy, log=print):
    """piles_xy: {pile: (x, y)}. Returns a summary dict."""
    from pxr import Usd, UsdGeom
    out = {"crop": 0, "debris": 0, "figures": 0}
    if not (on("LV_CROP") or on("LV_NODEBRIS") or on("LV_NOFIGURES")):
        return out
    pts = [piles_xy[p] for p in centre_piles if p in piles_xy] or list(piles_xy.values())[:6]
    cx, cy = sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)
    R = max([math.hypot(piles_xy[p][0] - cx, piles_xy[p][1] - cy) for p in keep_piles if p in piles_xy] + [30.0]) + 25.0
    vx, vy = van_xy if van_xy else (cx, cy)

    def keep_xy(x, y):
        if math.hypot(x - cx, y - cy) <= R:
            return True
        dx, dy = vx - cx, vy - cy                              # corridor (20 m wide) from the centre to the van
        L2 = dx * dx + dy * dy or 1.0
        u = max(0.0, min(1.0, ((x - cx) * dx + (y - cy) * dy) / L2))
        return math.hypot(x - (cx + u * dx), y - (cy + u * dy)) <= 20.0
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])

    def walk(prim, depth):
        path = str(prim.GetPath())
        if any(path == p or path.startswith(p + "/") for p in PROTECT):
            return
        name = prim.GetName()
        if on("LV_NOFIGURES") and FIGURES.search(name):
            prim.SetActive(False); out["figures"] += 1; return
        if on("LV_NODEBRIS") and DEBRIS.search(name):
            prim.SetActive(False); out["debris"] += 1; return
        if on("LV_CROP") and depth >= 2:
            c, size = _centre(cache, prim)
            if c is not None and size < 2 * R and not keep_xy(*c):
                prim.SetActive(False); out["crop"] += 1; return
        if depth < 5:
            for ch in list(prim.GetChildren()):
                walk(ch, depth + 1)
    world = stage.GetPrimAtPath("/World")
    for ch in list(world.GetChildren()):
        walk(ch, 1)
    out["radius_m"], out["centre"] = round(R, 1), (round(cx, 1), round(cy, 1))
    log(f"[trim] crop r={R:.0f} m around ({cx:.0f}, {cy:.0f}) + van corridor: {out}")
    return out


class GlowDummies:
    """Grey mannequins + surface pins at the true positions; glow once the mission's own state flags their pile."""

    def __init__(self, stage, victims, surface, log=print):
        from pxr import Gf, Sdf, UsdGeom, UsdShade
        self.stage, self.log, self.items, self.lit = stage, log, [], set()
        root = "/World/LvDummies"
        UsdGeom.Xform.Define(stage, root)

        def mat(name, rgb, emissive=None):
            m = UsdShade.Material.Define(stage, f"{root}/Looks/{name}")
            sh = UsdShade.Shader.Define(stage, f"{root}/Looks/{name}/S")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
            if emissive:
                sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
            m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            return m
        self.grey = mat("Grey", (0.55, 0.56, 0.58))
        self.glow = mat("Glow", (1.0, 0.35, 0.12), emissive=(12.0, 3.2, 0.8))
        for k, v in enumerate(victims):
            x, y = float(v["xy"][0]), float(v["xy"][1])
            zs = float(surface(x, y))
            zb = zs - float(v.get("depth", 1.0))
            base = f"{root}/P{k}"
            UsdGeom.Xform.Define(stage, base)
            body = UsdGeom.Capsule.Define(stage, f"{base}/Body")
            body.CreateAxisAttr("X"); body.CreateRadiusAttr(0.22); body.CreateHeightAttr(1.1)
            UsdGeom.XformCommonAPI(body).SetTranslate(Gf.Vec3d(x, y, zb))
            head = UsdGeom.Sphere.Define(stage, f"{base}/Head")
            head.CreateRadiusAttr(0.16)
            UsdGeom.XformCommonAPI(head).SetTranslate(Gf.Vec3d(x + 0.8, y, zb + 0.05))
            pin = UsdGeom.Cylinder.Define(stage, f"{base}/Pin")
            pin.CreateAxisAttr("Z"); pin.CreateRadiusAttr(0.07); pin.CreateHeightAttr(zs + 2.0 - zb)
            UsdGeom.XformCommonAPI(pin).SetTranslate(Gf.Vec3d(x, y, (zs + 2.0 + zb) / 2))
            disc = UsdGeom.Cylinder.Define(stage, f"{base}/Disc")
            disc.CreateAxisAttr("Z"); disc.CreateRadiusAttr(0.7); disc.CreateHeightAttr(0.04)
            UsdGeom.XformCommonAPI(disc).SetTranslate(Gf.Vec3d(x, y, zs + 2.0))
            prims = [body.GetPrim(), head.GetPrim(), pin.GetPrim(), disc.GetPrim()]
            for p in prims:
                UsdShade.MaterialBindingAPI.Apply(p).Bind(self.grey)
            self.items.append({"pile": v.get("pile"), "prims": prims, "id": v.get("id")})
        log(f"[trim] {len(self.items)} grey dummies (display only) at {[i['pile'] for i in self.items]}")

    def update(self, state):
        """Light a dummy when the mission's OWN state shows a ping / candidate region / TRAPPED clip at its pile."""
        from pxr import UsdShade
        flagged = set()
        for c in (state.get("pings") or []) + (state.get("radar_candidates") or []):
            if isinstance(c, dict) and c.get("pile_id"):
                flagged.add(c["pile_id"])
        for c in state.get("acoustic_clips") or []:
            if isinstance(c, dict) and c.get("tier") == "trapped" and c.get("pile_id"):
                flagged.add(c["pile_id"])
        for it in self.items:
            if it["pile"] in flagged and it["pile"] not in self.lit:
                for p in it["prims"]:
                    UsdShade.MaterialBindingAPI.Apply(p).Bind(self.glow)
                self.lit.add(it["pile"])
                self.log(f"[trim] DUMMY LIT at {it['pile']} (the mission's own lead there)")
