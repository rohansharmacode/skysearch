"""Damaged-facade kit placer for the fallen towers (--site collapse, --facade-kit DIR).

The kit (isaac/assets/facade_kit/facade_kit.usdc + textures/, built by isaac/tools/blender_facade_kit.py in
Blender) holds broken slab edges, tiled infill walls with torn windows, crushed-storey voids, columns, beam ends
and debris chunks, each for one bay x one storey in the module frame:

    x = U  across the face (bay, unit 3.0 m)     y = V  along the fallen tower (storey, unit 2.9 m)
    z = N  outward from the scanned face plane    origin: bay centre on the face plane (slabs: storey line,
                                                  columns / beams: bay line, debris: base centre)

plan_face() takes the per-quad frame urban_look.facade() computes (A0, A1, B0, B1, U, V, N, nst, nb) and lays
the modules out with the same mix as the box facade (slabs at every storey line with ~15 % fallen out, ~50 %
crushed storeys, the rest tiled walls, columns at the bay lines, a few beam ends and debris), seeded, with
small random tilt / offset / scale. place() writes one UsdGeom.PointInstancer per module type whose
prototypes reference the kit prims. Visual only: no colliders, no physics APIs.
"""
from __future__ import annotations

import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
BAY, STOREY = 3.0, 2.9
KINDS = ("slab", "wall", "void", "column", "beam", "debris")
USD_NAME = "facade_kit.usdc"


def default_dir():
    """The repo's kit (isaac/assets/facade_kit) if it was built, else ''."""
    d = os.path.join(HERE, "assets", "facade_kit")
    return d if os.path.exists(os.path.join(d, USD_NAME)) else ""


class Kit:
    """A built kit folder: the USD path and the module names per type."""

    def __init__(self, folder):
        folder = os.path.abspath(os.path.expanduser(folder))
        self.folder = folder
        self.usd = os.path.join(folder, USD_NAME)
        if not os.path.exists(self.usd):
            raise FileNotFoundError(f"no {USD_NAME} in {folder} (build it: isaac/tools/blender_facade_kit.py)")
        mods = []
        meta = os.path.join(folder, "kit.json")
        if os.path.exists(meta):
            mods = json.load(open(meta)).get("modules", [])
        if not mods:
            from pxr import Usd
            st = Usd.Stage.Open(self.usd, load=Usd.Stage.LoadNone)
            mods = [p.GetName() for p in st.GetPrimAtPath("/FacadeKit").GetChildren()]
        self.variants = {k: sorted([m for m in mods if m.rsplit("_", 1)[0] == k], key=lambda m: int(m.rsplit("_", 1)[1]))
                         for k in KINDS}
        missing = [k for k in ("slab", "wall", "void") if not self.variants[k]]
        if missing:
            raise ValueError(f"facade kit {folder} has no {missing} modules")

    def counts(self):
        return {k: len(v) for k, v in self.variants.items()}


def default_variants(modules):
    return {k: sum(1 for m in modules if m.rsplit("_", 1)[0] == k) for k in KINDS}


# ------------------------------------------------------------------------------------------------ maths
def _axis_rot(axis, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    if axis == 0:
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == 1:
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _rot(U, V, N, du=0.0, dv=0.0, dn=0.0):
    """Module-to-world rotation (columns U, V, N) with small local tilts (deg) about the module x, y, z."""
    R0 = np.column_stack([U, V, N])
    return R0 @ _axis_rot(0, du) @ _axis_rot(1, dv) @ _axis_rot(2, dn)


def quat(R):
    """(w, x, y, z) of a proper rotation matrix (column-vector convention)."""
    m = R
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = (0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = ((m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s)
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = ((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s)
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = ((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s)
    q = np.array(q)
    return q / np.linalg.norm(q)


# ------------------------------------------------------------------------------------------------ layout
def plan_face(A0, A1, B0, B1, U, V, N, nst, nb, rng, variants, zmin=-0.4, face_up=None, missing=0.15,
              crushed=0.5, columns_missing=0.18, beams=0.07, debris=0.35):
    """Kit modules for one tower face quad.  Returns [(kind, variant, position, R (3x3), scale (3))]; world =
    position + R @ (scale * module_point). A0, A1 low-V corners, B0, B1 high-V corners, U across (bays),
    V along (storeys), N outward; nst storeys x nb bays (urban_look.facade's frame)."""
    A0, A1, B0, B1, U, V, N = (np.asarray(x, float) for x in (A0, A1, B0, B1, U, V, N))
    W, L = np.linalg.norm(A1 - A0), np.linalg.norm(B0 - A0)
    bw, sh = W / nb, L / nst
    su, sv = bw / BAY, sh / STOREY
    up = float(N[2]) if face_up is None else face_up
    items = []

    def at(s, t, off=0.0):
        return (1 - t) * ((1 - s) * A0 + s * A1) + t * ((1 - s) * B0 + s * B1) + N * off

    def add(kind, p, R, s, var=None):
        n = variants.get(kind, 0)
        if n == 0 or p[2] < zmin:
            return
        items.append((kind, int(rng.integers(n)) if var is None else var, p, R, np.asarray(s, float)))

    # floor slabs at every storey line, broken per bay; ~15 % fallen out
    for j in range(nst + 1):
        for i in range(nb):
            if rng.random() < missing:
                continue
            p = at((i + 0.5) / nb, j / nst, rng.normal(0, 0.05)) + V * rng.normal(0, 0.05) + U * rng.normal(0, 0.06)
            sag = rng.normal(0, 3.5) + (rng.choice((-1, 1)) * rng.uniform(6, 12) if rng.random() < 0.1 else 0.0)
            R = _rot(U, V, N, sag, rng.normal(0, 1.2), rng.normal(0, 1.0))
            if rng.random() < 0.5:
                R = R @ _axis_rot(2, 180.0)                     # slabs are symmetric in V: flip the broken end
            add("slab", p, R, (su * rng.uniform(0.94, 1.03), rng.uniform(0.9, 1.1), rng.uniform(0.85, 1.12)))
    # storeys: ~50 % crushed voids, the rest tiled infill walls; debris in the crushed ones
    for j in range(nst):
        for i in range(nb):
            p = at((i + 0.5) / nb, (j + 0.5) / nst, 0.05 + rng.normal(0, 0.03))   # backs clear of the scan face
            if rng.random() < crushed:
                R = _rot(U, V, N, rng.normal(0, 2.0), rng.normal(0, 1.5), rng.normal(0, 1.0))
                add("void", p, R, (su, sv, rng.uniform(0.85, 1.15)))
                if up > 0.15 and rng.random() < debris:
                    q = p + U * rng.uniform(-0.35, 0.35) * bw + V * rng.uniform(-0.3, 0.3) * sh + N * 0.75
                    Rr = _rot(U, V, N, rng.uniform(-40, 40), rng.uniform(-40, 40), rng.uniform(0, 360))
                    k = rng.uniform(0.7, 1.4)
                    add("debris", q, Rr, (k, k, k))
            else:
                R = _rot(U, V, N, rng.normal(0, 2.5), rng.normal(0, 1.5), rng.normal(0, 0.8))
                add("wall", p + N * rng.normal(0, 0.05), R, (su, sv, rng.uniform(0.9, 1.1)))
    # columns at every second bay line (tiled fronts, flush with the walls), a few broken beam ends
    for j in range(nst):
        for i in range(nb + 1):
            if i % 2 or rng.random() < columns_missing:
                continue
            p = at(i / nb, (j + 0.5) / nst, rng.normal(0, 0.03))
            R = _rot(U, V, N, rng.normal(0, 3.0), rng.normal(0, 2.0), rng.normal(0, 1.5))
            add("column", p, R, (rng.uniform(0.92, 1.08), sv, rng.uniform(0.9, 1.1)))
    for j in range(1, nst):
        for i in range(1, nb):
            if rng.random() < beams:
                p = at(i / nb, j / nst)
                R = _rot(U, V, N, rng.normal(0, 8), rng.normal(0, 8), rng.normal(0, 5))
                add("beam", p, R, (1.0, 1.0, rng.uniform(0.8, 1.2)))
    return items


# ------------------------------------------------------------------------------------------------ USD
def place(stage, path, kit, items, log=print):
    """One PointInstancer per module type under `path`, prototypes referencing the kit prims. Returns
    {kind: instance count}."""
    from pxr import Gf, Sdf, UsdGeom, Vt
    UsdGeom.Xform.Define(stage, path)
    counts = {}
    for kind in KINDS:
        sel = [it for it in items if it[0] == kind]
        names = kit.variants.get(kind, [])
        if not sel or not names:
            continue
        ip = f"{path}/{kind}"
        inst = UsdGeom.PointInstancer.Define(stage, ip)
        rel = inst.CreatePrototypesRel()
        stage.DefinePrim(f"{ip}/Prototypes", "Scope")
        for nm in names:
            pp = stage.DefinePrim(f"{ip}/Prototypes/{nm}", "Xform")
            pp.GetReferences().AddReference(kit.usd, Sdf.Path(f"/FacadeKit/{nm}"))
            rel.AddTarget(pp.GetPath())
        P = np.array([it[2] for it in sel], np.float32)
        Q = np.array([quat(it[3]) for it in sel])
        S = np.array([it[4] for it in sel], np.float32)
        inst.CreateProtoIndicesAttr(Vt.IntArray([int(it[1]) for it in sel]))
        inst.CreatePositionsAttr(Vt.Vec3fArray.FromNumpy(P))
        inst.CreateOrientationsAttr(Vt.QuathArray([Gf.Quath(float(w), float(x), float(y), float(z)) for w, x, y, z in Q]))
        inst.CreateScalesAttr(Vt.Vec3fArray.FromNumpy(S))
        r = 2.5
        inst.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, P.min(0) - r)), Gf.Vec3f(*map(float, P.max(0) + r))]))
        counts[kind] = len(sel)
    return counts


def load(folder, log=print):
    """Kit or None (logged) - never raises, so the caller can fall back to the box facade."""
    if not folder:
        return None
    try:
        return Kit(folder)
    except Exception as e:
        log(f"[urban] facade kit {folder!r} not used ({type(e).__name__}: {e}); box facade instead")
        return None
