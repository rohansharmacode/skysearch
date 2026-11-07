"""Winch-lowered radar pod (heavy platform): a small box hanging on a cable under its drone, resting on the
rubble during the scan. Drawn from the mission state (display only):

    state["pods"]    = [{"drone_id", "xy": [x, y], "z": <pod base height, m>, "status"}, ...]
    state["drones"][i]["winch_len_m"]   cable paid out (m); used when a pod has no explicit z / no entry

A pod is hidden while ``status`` is stowed / docked / off (or absent with winch_len_m ~ 0). ``z`` is taken as
the pod's BASE (it rests on the surface while scanning); the cable runs from the drone to the pod's top.
pxr only on the Isaac path; ``NullPods`` (dry run) keeps the same interface and just records what it would draw.
"""
from __future__ import annotations

POD_SIZE = (0.45, 0.35, 0.30)          # x, y, z (m): radar head + battery in a rugged case
HIDDEN = {"stowed", "docked", "off", "none", ""}


def pod_targets(state, poses):
    """{drone_id: (px, py, pz_base, status)} for every visible pod; drones without one are omitted."""
    pods = {p.get("drone_id"): p for p in (state.get("pods") or []) if isinstance(p, dict)}
    winch = {d.get("id"): d.get("winch_len_m") for d in state.get("drones", [])}
    out = {}
    for did, pose in poses.items():
        x, y, z = pose[:3]
        p = pods.get(did)
        wl = winch.get(did)
        if p is not None:
            st = str(p.get("status") or "")
            if st.lower() in HIDDEN:
                continue
            px, py = (p.get("xy") or (x, y))[:2]
            pz = p.get("z")
            if pz is None:
                pz = z - float(wl or 0.0) - POD_SIZE[2]
            out[did] = (float(px), float(py), float(pz), st)
        elif wl is not None and float(wl) > 0.05:
            out[did] = (float(x), float(y), float(z) - float(wl) - POD_SIZE[2], "lowering")
    return out


class NullPods:
    """Dry-run stand-in: same update() signature, counts what would be drawn."""

    def __init__(self, *_, **__):
        self.updates = 0
        self.max_visible = 0
        self.last = {}

    def update(self, state, poses):
        self.last = pod_targets(state, poses)
        self.updates += 1
        self.max_visible = max(self.max_visible, len(self.last))
        return self.last


class PodViz(NullPods):
    """USD pods: /World/Pods/<id>/Box (cube) + /World/Pods/<id>/Cable (2-point BasisCurves)."""

    def __init__(self, stage, ids, root="/World/Pods"):
        super().__init__()
        from pxr import Gf, UsdGeom, Vt
        self._Gf, self._UsdGeom, self._Vt = Gf, UsdGeom, Vt
        self.stage, self.prims = stage, {}
        UsdGeom.Xform.Define(stage, root)
        for did in ids:
            box = UsdGeom.Cube.Define(stage, f"{root}/{did}/Box")
            box.CreateSizeAttr(1.0)
            box.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.55, 0.10)])          # safety orange
            xf = UsdGeom.Xformable(box)
            xf.ClearXformOpOrder()
            t = xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
            xf.AddScaleOp().Set(Gf.Vec3f(*POD_SIZE))
            cab = UsdGeom.BasisCurves.Define(stage, f"{root}/{did}/Cable")
            cab.CreateTypeAttr("linear")
            cab.CreateCurveVertexCountsAttr([2])
            cab.CreateWidthsAttr([0.025, 0.025])
            cab.CreateDisplayColorAttr([Gf.Vec3f(0.08, 0.08, 0.08)])
            pts = cab.CreatePointsAttr([Gf.Vec3f(0, 0, 0), Gf.Vec3f(0, 0, 0)])
            for g in (box, cab):
                UsdGeom.Imageable(g).MakeInvisible()
            self.prims[did] = (box, t, cab, pts)

    def update(self, state, poses):
        tg = super().update(state, poses)
        Gf, UsdGeom = self._Gf, self._UsdGeom
        for did, (box, t, cab, pts) in self.prims.items():
            if did not in tg or did not in poses:
                UsdGeom.Imageable(box).MakeInvisible()
                UsdGeom.Imageable(cab).MakeInvisible()
                continue
            px, py, pz, _ = tg[did]
            x, y, z = poses[did][:3]
            t.Set(Gf.Vec3d(px, py, pz + POD_SIZE[2] / 2))
            pts.Set([Gf.Vec3f(float(x), float(y), float(z)), Gf.Vec3f(px, py, pz + POD_SIZE[2])])
            UsdGeom.Imageable(box).MakeVisible()
            UsdGeom.Imageable(cab).MakeVisible()
        return tg
