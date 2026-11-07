"""``--look pretty`` shim. The canonical photoreal look is the Isaac teammate's GPU-verified ``isaac/photoreal.py``
(``--photoreal 1``): ``--look pretty`` sets ``photoreal=1`` in run_mission.parse_args, so ``photoreal.dress()``
does the materials, irregular rubble colliders, HDRI sky, debris and vegetation. Path tracing is
``photoreal.set_renderer`` (``--rtx pathtracing --spp 16``).

What stays here (ours, not in photoreal):
* ``FlowDust``: Omni Flow dust under each drone near the ground with ``--look pretty`` (``--flow 0`` = off), from
  NVIDIA's Fire preset retuned as cool smoke (verified on 6.0.1 in the team's quake3 clips).
* ``chipped_box`` / ``merge_pieces``: numpy chipped-box geometry (unit-tested; no longer drawn by default).
* ``apply()``: kept for old callers; forwards to ``photoreal.dress``.

The old chipped-twin + MDL path (``chip_rubble``) was removed: photoreal deactivates the boxes and replaces them
with irregular collider meshes, so the twins would have doubled the rubble.
"""
from __future__ import annotations

import argparse

import numpy as np


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


# ---------------------------------------------------------------------------- photoreal forwarding
def photoreal_defaults(**over):
    """argparse.Namespace with photoreal.add_args defaults (+ overrides), for callers without run_mission's args."""
    from isaac import photoreal
    ap = argparse.ArgumentParser(add_help=False)
    photoreal.add_args(ap)
    a = ap.parse_args([])
    a.photoreal = 1
    for k, v in over.items():
        setattr(a, k, v)
    return a


def apply(stage, sc, mode=None, a=None, log=print, **_ignored):
    """Back-compat: ``--look pretty`` == ``photoreal.dress``. Call after scene.build() and before physics starts."""
    from isaac import photoreal
    photoreal.dress(stage, sc, a if a is not None else photoreal_defaults(), log)
    return {"photoreal": True}


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
