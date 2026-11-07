"""Winch-lowered radar + mic pod (heavy platform), drawn from the mission state (display only):

    state["pods"]    = [{"drone_id", "xy": [x, y], "z": <pod base height, m>, "status"}, ...]
    state["drones"][i]["winch_len_m"]   cable paid out (m); used when a pod has no explicit z / no entry

A pod is hidden while ``status`` is stowed / docked / off (or absent with winch_len_m ~ 0). ``z`` is the pod's
BASE (it rests on the surface while scanning).

Geometry follows data/hardware_bom.json (``isaac_mounts``, parent "pod", pod frame: origin at the centre of the
pod base, +z up): 0.50 x 0.50 x 0.25 m IP65 case, ground-coupled radar face (PodRadarFace) on the base, four
corner MEMS mics at (+-0.21, +-0.21, 0.255) (PodMic0..3), lift eye at 0.27 m (PodLiftEye, the cable attach
point), white strobe (PodStrobe), 5 GHz link antenna (PodAntenna).  ``pod_parts()`` is that list (pure python);
``build_pod`` writes it as meshes (isaac.van mesh kit).

Two ways to draw it:
  * ``PodViz(stage, ids, drones=drones)`` (preferred): drives each heavy DroneViz's own winch: the pod hangs
    from the real winch on the cable, sways (``sway_deg``), and rests on the surface at the state's ``z``.
  * ``PodViz(stage, ids)`` (what run_mission does today): a separate pod + cable per drone under /World/Pods;
    the drone's own stowed pod is hidden while this one is shown, so there is never a duplicate.
``NullPods`` (dry run) keeps the same interface and just records what it would draw.
"""
from __future__ import annotations

import math

POD_SIZE = (0.50, 0.50, 0.25)          # x, y, z (m): data/hardware_bom.json pod_enclosure
EYE_Z = 0.27                           # lift eye (cable attach) above the pod base
HIDDEN = {"stowed", "docked", "off", "none", ""}

POD_MATS = {
    "pod_case": dict(rgb=(0.17, 0.18, 0.19), rough=0.55),
    "pod_face": dict(rgb=(0.98, 0.55, 0.12), rough=0.6),          # radar face: orange (BOM colour)
    "pod_bumper": dict(rgb=(0.04, 0.04, 0.04), rough=0.85),
    "pod_mic": dict(rgb=(0.15, 0.75, 0.72), rough=0.9),           # teal: acoustics
    "pod_metal": dict(rgb=(0.60, 0.60, 0.60), rough=0.35, metal=0.9),
    "pod_antenna": dict(rgb=(0.23, 0.51, 0.96), rough=0.5),       # comms blue #3b82f6
    "pod_strobe": dict(rgb=(1.0, 1.0, 1.0), rough=0.2, emissive=(5.0, 5.0, 5.0)),
    "pod_led": dict(rgb=(0.2, 0.8, 0.3), rough=0.3, emissive=(0.3, 2.2, 0.4)),
    "cable": dict(rgb=(0.15, 0.15, 0.15), rough=0.7),
}


def pod_parts():
    """[(name, kind, pos (pod frame), size / dims dict, material, label)] from the BOM isaac_mounts."""
    sx, sy, sz = POD_SIZE
    P = [("PodCase", "box", (0, 0, 0.13), (sx - 0.02, sy - 0.02, 0.20), "pod_case", "pod case (IP65)"),
         ("PodLid", "box", (0, 0, 0.24), (sx - 0.04, sy - 0.04, 0.02), "pod_case", None),
         ("PodRadarFace", "box", (0, 0, 0.01), (0.48, 0.48, 0.02), "pod_face",
          "PodRadarFace: UWB radar 0.4-2 GHz, ground-coupled"),
         ("PodFaceBand", "box", (0, 0, 0.045), (sx, sy, 0.03), "pod_face", None)]
    for i, (cx, cy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
        P.append((f"PodBumper{i}", "cyl", (cx * (sx / 2 - 0.02), cy * (sy / 2 - 0.02), 0.125), dict(r=0.03, h=0.25),
                  "pod_bumper", None))
        P.append((f"PodMic{i}", "sphere", (cx * 0.21, cy * 0.21, 0.255), dict(r=0.02), "pod_mic",
                  "PodMic: MEMS mic (4-mic pod array)" if i == 0 else None))
        P.append((f"PodMicPost{i}", "cyl", (cx * 0.21, cy * 0.21, 0.24), dict(r=0.006, h=0.03), "pod_metal", None))
        P.append((f"PodBridle{i}", "line", (cx * 0.20, cy * 0.20, 0.25), (0, 0, EYE_Z), "cable", None))
    P += [("PodLiftEye", "torus", (0, 0, EYE_Z), dict(R=0.022, r=0.006), "pod_metal", "PodLiftEye: winch hook point"),
          ("PodStrobe", "sphere", (0.18, 0, 0.26), dict(r=0.015), "pod_strobe", "PodStrobe"),
          ("PodAntenna", "cyl", (-0.18, 0, 0.29), dict(r=0.008, h=0.08), "pod_antenna", "PodAntenna: 5 GHz link to the drone"),
          ("PodLed", "box", (0.12, -sy / 2 + 0.005, 0.18), (0.03, 0.006, 0.012), "pod_led", None)]
    return P


def build_pod(stage, path, looks, mats=None):
    """Write the pod meshes under ``path`` (an Xform the caller poses: origin = pod base centre).
    Returns the triangle count."""
    from isaac.van import Assembly, xform
    xform(stage, path)
    A = Assembly(stage, path, looks, dict(POD_MATS, **(mats or {})))
    for name, kind, pos, dims, mat, _ in pod_parts():
        g = A.g(mat, "Geo")
        if kind == "box":
            g.box(dims, pos)
        elif kind == "cyl":
            g.cyl(dims["r"], dims["h"], pos, n=12)
        elif kind == "sphere":
            g.sphere(dims["r"], pos, n=10, m=6)
        elif kind == "torus":
            g.torus(dims["R"], dims["r"], pos, axis="x", n=12, m=6)
        elif kind == "line":
            g.cyl_between(pos, dims, 0.003, n=4, caps=False)
    return A.flush()


def pod_anchors():
    """{part name: (x, y, z) pod frame} for every labelled pod part (close-up labels)."""
    return {name: pos for name, kind, pos, dims, mat, label in pod_parts() if label}


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
    """USD pods. With ``drones`` (DroneViz by id, heavy model): each drone's own winch pod is driven (hangs on
    the cable, sways, rests at the state z). Without: /World/Pods/<id>/Pod (meshes) + Cable (BasisCurves), and
    the drone's stowed pod (``<drone_root>/<id>/Model/Hang``) is hidden while the separate pod is shown."""

    def __init__(self, stage, ids, root="/World/Pods", drones=None, drone_root="/World/Drones", sway_deg=2.0):
        super().__init__()
        from pxr import Gf, UsdGeom
        self._Gf, self._UsdGeom = Gf, UsdGeom
        self.stage, self.prims, self.drones = stage, {}, drones or {}
        self.drone_root, self.sway_deg, self.t = drone_root, float(sway_deg), 0.0
        self.tris = 0
        if all(hasattr(self.drones.get(d), "set_winch") for d in ids) and self.drones:
            return                                              # integrated winch pods: nothing of our own
        from isaac.van import Looks
        UsdGeom.Xform.Define(stage, root)
        looks = Looks(stage, root + "/Looks")
        for did in ids:
            self.tris += build_pod(stage, f"{root}/{did}/Pod", looks)
            pod = UsdGeom.Xformable(stage.GetPrimAtPath(f"{root}/{did}/Pod"))
            pod.ClearXformOpOrder()
            t = pod.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
            cab = UsdGeom.BasisCurves.Define(stage, f"{root}/{did}/Cable")
            cab.CreateTypeAttr("linear")
            cab.CreateCurveVertexCountsAttr([2])
            cab.CreateWidthsAttr([0.012, 0.012])
            cab.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.12, 0.12)])
            pts = cab.CreatePointsAttr([Gf.Vec3f(0, 0, 0), Gf.Vec3f(0, 0, 0)])
            for g in (pod, cab):
                UsdGeom.Imageable(g).MakeInvisible()
            self.prims[did] = (pod, t, cab, pts)

    def update(self, state, poses, dt=1 / 30.0):
        tg = super().update(state, poses)
        self.t += dt
        if not self.prims:                                      # integrated: drive the drones' winches
            for did, d in self.drones.items():
                if did not in poses:
                    continue
                if did not in tg:
                    d.set_winch(0.0)
                    continue
                px, py, pz, st = tg[did]
                x, y, z = poses[did][:3]
                hang_z = d.winch_world_z(z)
                L = max(0.0, hang_z - EYE_Z - pz)
                resting = st.lower() in ("resting", "scanning", "listening", "down", "on_surface")
                a = 0.0 if resting else self.sway_deg * min(1.0, L / 6.0)
                sway = (a * math.sin(2 * math.pi * self.t / 3.1), 0.7 * a * math.sin(2 * math.pi * self.t / 2.3 + 1.0))
                d.set_winch(L, sway_deg=sway, ground_z=pz if resting else None)
            return tg
        Gf, UsdGeom = self._Gf, self._UsdGeom
        for did, (pod, t, cab, pts) in self.prims.items():
            hang = self.stage.GetPrimAtPath(f"{self.drone_root}/{did}/Model/Hang")
            if did not in tg or did not in poses:
                UsdGeom.Imageable(pod).MakeInvisible()
                UsdGeom.Imageable(cab).MakeInvisible()
                if hang.IsValid():
                    UsdGeom.Imageable(hang).MakeVisible()
                continue
            px, py, pz, _ = tg[did]
            x, y, z = poses[did][:3]
            t.Set(Gf.Vec3d(px, py, pz))
            wz = self.drones[did].winch_world_z(z) if did in self.drones and hasattr(self.drones[did], "winch_world_z") \
                else z + WINCH_DZ
            pts.Set([Gf.Vec3f(float(x), float(y), float(wz)), Gf.Vec3f(px, py, pz + EYE_Z)])
            UsdGeom.Imageable(pod).MakeVisible()
            UsdGeom.Imageable(cab).MakeVisible()
            if hang.IsValid():
                UsdGeom.Imageable(hang).MakeInvisible()
        return tg


WINCH_DZ = 0.445 - 0.15                  # winch drum underside above the mission pose point (heavy model)
