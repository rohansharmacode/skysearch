"""--photoreal on the collapse map (--site collapse): take the stretched scan photo off the rubble, the fallen
towers and the floor slabs, and build that detail in 3D instead.

* rubble ground: terrain faces above street level (z > RUBBLE_Z, ragged edge) get crushed-concrete, gravel or
  rocky-soil MDL in patches; flat ground in the scanned core gets dusty concrete and dirt (GeomSubsets of the
  map's terrain tiles, geometry unchanged);
* fallen towers: plaster/concrete MDL on the four long faces and broken concrete on the ends, plus a 3D facade
  (storey slab edges, columns, window frames with glass or dark openings, balconies) and broken column stubs
  with bent rebar at both breaks;
* floor slabs: formed concrete;
* 3D debris: tens of thousands of concrete chunks, blocks, slab shards, brick, wall-tile flakes, rebar and
  broken column pieces on the rubble (one PointInstancer), and the map's own scatter re-materialled;
* a rescue crew at the staging area.

Visual only: nothing added here has a collider and no geometry the LiDAR sees is moved, so the planner's map,
the landing cells and a --replay are unchanged.
"""
from __future__ import annotations

import math

import numpy as np

SITE = "/World/CollapseSite"
ROOT = "/World/UrbanLook"
CORE = (-60.0, 60.0, -56.0, 56.0)          # x0, x1, y0, y1 of the scanned core (the blurry scan photo)
OUTER_R = 125.0                            # rubble mounds re-materialled / dressed out to this radius
RUBBLE_Z = 0.35                            # terrain above this (m) is rubble; the streets sit at ~0
TOWERS = ("core_W_tower", "core_E_tower")
SLABS = ("core_P1", "core_P2", "core_P3", "core_P4")
STOREY_M = 2.9                             # 16 storeys along the 46 m fallen towers
CREW = (                                    # (character, dx, dy, yaw) around the staging base
    ("F_Medical_01/F_Medical_01.usd", 5.0, 2.2, 200.0),
    ("male_adult_construction_05_new/male_adult_construction_05_new.usd", -2.2, 3.0, 160.0),
    ("male_adult_construction_01_new/male_adult_construction_01_new.usd", 9.0, 0.6, 120.0),
    ("male_adult_construction_05_new/male_adult_construction_05_new.usd", 11.2, 3.4, 215.0),
)
FACE_MAT = "concrete3"                     # fallen-tower faces: smooth grey concrete (plaster read too white)
# The map's own grade (collapse_site/apply_look.py), with less haze: fixed ACES exposure, AO, film grain.
GRADE = {
    "/rtx/post/tonemap/op": 6, "/rtx/post/tonemap/filmIso": 100.0, "/rtx/post/tonemap/cameraShutter": 50.0,
    "/rtx/post/tonemap/fNumber": 5.0, "/rtx/post/histogram/enabled": False,
    "/rtx/ambientOcclusion/enabled": True, "/rtx/ambientOcclusion/rayLength": 3.0,
    "/rtx/indirectDiffuse/enabled": True, "/rtx/shadows/enabled": True, "/rtx/reflections/enabled": True,
    "/rtx/post/colorcorr/enabled": True, "/rtx/post/colorcorr/saturation": [1.0, 1.0, 1.0],
    "/rtx/post/colorcorr/contrast": [1.15, 1.15, 1.15], "/rtx/post/colorcorr/gamma": [1.0, 1.0, 1.0],
    "/rtx/post/colorcorr/gain": [1.0, 1.0, 1.0], "/rtx/post/colorcorr/offset": [0.0, 0.0, 0.0],
    "/rtx/post/tvNoise/enabled": True, "/rtx/post/tvNoise/enableVignetting": True,
    "/rtx/post/tvNoise/vignettingSize": 107.0, "/rtx/post/tvNoise/vignettingStrength": 0.45,
    "/rtx/post/tvNoise/enableFilmGrain": True, "/rtx/post/tvNoise/grainAmount": 0.02,
    "/rtx/post/tvNoise/grainSize": 1.4, "/rtx/post/tvNoise/enableScanlines": False,
    "/rtx/post/tvNoise/enableScrollBug": False, "/rtx/post/tvNoise/enableVignettingFlickering": False,
    "/rtx/post/tvNoise/enableGhostFlickering": False, "/rtx/post/tvNoise/enableWaveDistortion": False,
    "/rtx/post/tvNoise/enableVerticalLines": False, "/rtx/post/tvNoise/enableRandomSplotches": False,
    "/rtx/fog/enabled": True, "/rtx/fog/fogZup/enabled": True, "/rtx/fog/fogColor": [0.78, 0.74, 0.68],
    "/rtx/fog/fogColorIntensity": 1.0, "/rtx/fog/fogStartDist": 90.0, "/rtx/fog/fogEndDist": 1200.0,
    "/rtx/fog/fogDistanceDensity": 0.2, "/rtx/fog/fogStartHeight": 0.0, "/rtx/fog/fogEndHeight": 60.0,
    "/rtx/fog/fogHeightDensity": 0.15,
}
EXTRA_MDL = {"stucco": "Masonry/Stucco.mdl", "soil": "Natural/Soil_Rocky.mdl",
             "concrete3": "Masonry/Concrete_Smooth.mdl"}

BOX_F = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
                  [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7], [1, 2, 6], [1, 6, 5]], np.int32)
BOX_S = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
                  [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]], float)


# ------------------------------------------------------------------------------------------------ helpers
def _pv(stage, path, rgb, rough=0.6, metal=0.0, coat=0.0):
    """UsdPreviewSurface material (renders in RTX and path tracing; no texture fetch)."""
    from pxr import Gf, Sdf, UsdShade
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*map(float, rgb)))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(rough))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metal))
    sh.CreateInput("clearcoat", Sdf.ValueTypeNames.Float).Set(float(coat))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _box(c, U, V, N, hu, hv, hn):
    P = c + (BOX_S[:, :1] * hu) * U + (BOX_S[:, 1:2] * hv) * V + (BOX_S[:, 2:3] * hn) * N
    return P.astype(np.float32), BOX_F.copy()


def _emit(stage, path, parts, mat, rgb=(0.6, 0.6, 0.6)):
    from isaac import photoreal as PR
    from isaac import rubble as RB
    if not parts:
        return 0
    P, F = RB.merge(parts)
    m = PR._mesh(stage, path, P, F, rgb)
    m.CreateDoubleSidedAttr(True)
    if mat is not None:
        PR.bind(stage, path, mat)
    return len(parts)


def _mesh_arrays(prim):
    from pxr import UsdGeom
    m = UsdGeom.Mesh(prim)
    P = np.asarray(m.GetPointsAttr().Get(), float)
    cnt = np.asarray(m.GetFaceVertexCountsAttr().Get())
    idx = np.asarray(m.GetFaceVertexIndicesAttr().Get())
    xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
    M = np.array(xf, float)
    if not np.allclose(M, np.eye(4)):
        P = P @ M[:3, :3] + M[3, :3]
    return m, P, cnt, idx


def _noise(seed, cell, x, y):
    """Smooth value noise in [0, 1] (bilinear over a random lattice), for material patches."""
    rng = np.random.default_rng(seed)
    n = int(math.ceil(2 * (OUTER_R + 60) / cell)) + 2
    G = rng.random((n, n))
    gx, gy = (x + OUTER_R + 60) / cell, (y + OUTER_R + 60) / cell
    i, j = np.clip(gx.astype(int), 0, n - 2), np.clip(gy.astype(int), 0, n - 2)
    fx, fy = np.clip(gx - i, 0, 1), np.clip(gy - j, 0, 1)
    fx, fy = fx * fx * (3 - 2 * fx), fy * fy * (3 - 2 * fy)
    return (G[i, j] * (1 - fx) * (1 - fy) + G[i + 1, j] * fx * (1 - fy) + G[i, j + 1] * (1 - fx) * fy
            + G[i + 1, j + 1] * fx * fy)


def _in_core(x, y, pad=0.0):
    return (x > CORE[0] - pad) & (x < CORE[1] + pad) & (y > CORE[2] - pad) & (y < CORE[3] + pad)


# ------------------------------------------------------------------------------------------------ quaternions
def _qmul(a, b):
    w1, x1, y1, z1 = a.T
    w2, x2, y2, z2 = b.T
    return np.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], 1)


def _q_axis(axis, ang):
    axis = axis / np.maximum(np.linalg.norm(axis, axis=1, keepdims=True), 1e-9)
    h = ang[:, None] / 2
    return np.hstack([np.cos(h), axis * np.sin(h)])


def _q_random(rng, n):
    u1, u2, u3 = rng.random((3, n))
    a, b = np.sqrt(1 - u1), np.sqrt(u1)
    return np.stack([b * np.cos(2 * np.pi * u3), a * np.sin(2 * np.pi * u2), a * np.cos(2 * np.pi * u2),
                     b * np.sin(2 * np.pi * u3)], 1)


def _q_on_surface(rng, nrm, max_tilt_deg):
    """Yaw at random, tilt up to max_tilt off the surface normal, then align z with the normal."""
    n = len(nrm)
    yaw = _q_axis(np.tile([0.0, 0.0, 1.0], (n, 1)), rng.uniform(0, 2 * np.pi, n))
    ta = rng.uniform(0, 2 * np.pi, n)
    tilt = _q_axis(np.stack([np.cos(ta), np.sin(ta), np.zeros(n)], 1), np.radians(rng.uniform(0, max_tilt_deg, n)))
    z = np.array([0.0, 0.0, 1.0])
    ax = np.cross(np.tile(z, (n, 1)), nrm)
    s = np.linalg.norm(ax, axis=1)
    ax[s < 1e-6] = [1.0, 0.0, 0.0]
    align = _q_axis(ax, np.arccos(np.clip(nrm[:, 2], -1, 1)))
    return _qmul(align, _qmul(tilt, yaw))


# ------------------------------------------------------------------------------------------------ terrain
def terrain(stage, mats, log, seed=0, rematerial=True):
    """Rubble/flat material patches on the map's terrain tiles (rematerial=False keeps the scan photo, e.g. the
    AI-refined one). Returns the rubble faces (centroid, normal, area, triangles) for the debris scatter."""
    from pxr import Usd, UsdGeom, UsdShade, Vt
    root = stage.GetPrimAtPath(f"{SITE}/Terrain")
    if not root.IsValid():
        log("[urban] no map terrain under /World/CollapseSite/Terrain; ground left as it is")
        return None
    pool, nsel, ntot = [], 0, 0
    for prim in root.GetChildren():
        if not prim.IsA(UsdGeom.Mesh) or not prim.GetName().startswith("terrain_r"):
            continue
        m, P, cnt, idx = _mesh_arrays(prim)
        if not len(cnt) or not np.all(cnt == 3):
            continue
        T = P[idx.reshape(-1, 3)]
        C = T.mean(1)
        cr = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        area = 0.5 * np.linalg.norm(cr, axis=1)
        nrm = cr / np.maximum(2 * area, 1e-9)[:, None]
        nrm[nrm[:, 2] < 0] *= -1
        r = np.hypot(C[:, 0], C[:, 1])
        thr = RUBBLE_Z + 0.25 * (_noise(seed + 11, 3.0, C[:, 0], C[:, 1]) - 0.5)
        rub = (C[:, 2] > thr) & (r < OUTER_R)
        flat = ~rub & _in_core(C[:, 0], C[:, 1])
        n1 = _noise(seed + 1, 9.0, C[:, 0], C[:, 1])
        n2 = _noise(seed + 2, 6.0, C[:, 0], C[:, 1])
        groups = {"concrete": rub & (n1 < 0.52), "gravel": rub & (n1 >= 0.52) & (n2 < 0.62),
                  "soil": rub & (n1 >= 0.52) & (n2 >= 0.62),
                  "concrete3": flat & (n2 < 0.55), "dirt": flat & (n2 >= 0.55)}
        for name, sel in (groups.items() if rematerial else ()):
            faces = np.nonzero(sel)[0].astype(np.int32)
            if not len(faces) or name not in mats:
                continue
            sub = UsdGeom.Subset.CreateGeomSubset(m, f"look_{name}", UsdGeom.Tokens.face, Vt.IntArray.FromNumpy(faces),
                                                  familyName=UsdShade.Tokens.materialBind)
            UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mats[name], UsdShade.Tokens.strongerThanDescendants)
            nsel += len(faces)
        if rematerial:
            UsdGeom.Subset.SetFamilyType(m, UsdShade.Tokens.materialBind, UsdGeom.Tokens.nonOverlapping)
        ntot += len(C)
        k = np.nonzero(rub)[0]
        pool.append((C[k], nrm[k], area[k], T[k]))
    if not pool:
        return None
    C, N, A, T = (np.concatenate([p[i] for p in pool]) for i in range(4))
    if rematerial:
        log(f"[urban] terrain: {nsel} of {ntot} faces re-materialled (rubble: concrete/gravel/rocky soil; core "
            f"streets: concrete/dirt); {len(C)} rubble faces, {A.sum():.0f} m2")
    else:
        log(f"[urban] terrain: scan photo kept; {len(C)} rubble faces ({A.sum():.0f} m2) take the 3D debris")
    return C, N, A, T


# ------------------------------------------------------------------------------------------------ towers
def _quads(prim):
    """The two quads (4 corners each) of a 4-triangle tower face mesh, with their outward normals."""
    _, P, cnt, idx = _mesh_arrays(prim)
    tri = idx.reshape(-1, 3)
    out = []
    for k in range(0, len(tri) - 1, 2):
        ids = list(dict.fromkeys(tri[k].tolist() + tri[k + 1].tolist()))
        if len(ids) != 4:
            continue
        Q = P[ids]
        n = np.cross(P[tri[k][1]] - P[tri[k][0]], P[tri[k][2]] - P[tri[k][0]])
        out.append((Q, n / max(np.linalg.norm(n), 1e-9)))
    return out


def _frame(Q, n, centre):
    """Bilinear frame of a long tower face: A0, A1 (low-y end) and B0, B1 (high-y end), U across, V along."""
    if np.dot(n, Q.mean(0) - centre) < 0:
        n = -n
    o = np.argsort(Q[:, 1])
    lo, hi = Q[o[:2]], Q[o[2:]]
    A0, A1 = lo
    u = A1 - A0
    B0, B1 = hi if np.dot(hi[1] - hi[0], u) > 0 else hi[::-1]
    U = u / np.linalg.norm(u)
    N = n - U * np.dot(n, U)
    N /= np.linalg.norm(N)
    V = np.cross(N, U)
    return A0, A1, B0, B1, U, V, N


def facade(stage, mats, looks, log, seed=0, slabs=True):
    """Plaster/concrete on the fallen towers and a 3D facade: storey slab edges, columns, framed windows
    (glass or dark broken openings), balconies with parapets on the upper west faces, and broken column stubs
    with bent rebar at both breaks."""
    from isaac import photoreal as PR
    from isaac import rubble as RB
    rng = np.random.default_rng(3100 + seed)
    tot = 0
    for t in TOWERS:
        base = f"{SITE}/Buildings/Scanned/{t}"
        if not stage.GetPrimAtPath(base).IsValid():
            continue
        for part, key in (("scanfaces", FACE_MAT), ("facades", FACE_MAT), ("breaks", "concrete")):
            if key in mats:
                PR.bind(stage, f"{base}/{part}", mats[key])
        quads = []
        for part in ("scanfaces", "facades"):
            p = stage.GetPrimAtPath(f"{base}/{part}")
            if p.IsValid():
                quads += _quads(p)
        if not quads:
            continue
        centre = np.concatenate([q for q, _ in quads]).mean(0)
        bins = {k: [] for k in ("band", "frame", "glass", "void", "balcony", "stub", "rebar")}
        upper = sorted([i for i, (q, n) in enumerate(quads) if q[:, 2].mean() > centre[2]],
                       key=lambda i: quads[i][0][:, 0].mean())
        for fi, (Q, n) in enumerate(quads):
            A0, A1, B0, B1, U, V, N = _frame(Q, n, centre)
            W, L = np.linalg.norm(A1 - A0), np.linalg.norm(B0 - A0)

            def at(s, tt, off=0.0):
                return (1 - tt) * ((1 - s) * A0 + s * A1) + tt * ((1 - s) * B0 + s * B1) + N * off

            nst, nb = max(2, int(round(L / STOREY_M))), max(2, int(round(W / 3.0)))
            sh, bw = L / nst, W / nb
            for j in range(nst + 1):                                     # slab edge at every floor
                c = at(0.5, j / nst, 0.06)
                if c[2] > -0.6:
                    bins["band"].append(_box(c, U, V, N, W / 2 * 0.995, 0.16, 0.09))
            for i in range(nb + 1):                                      # columns between the bays
                bins["band"].append(_box(at(i / nb, 0.5, 0.04), U, V, N, 0.18, L / 2, 0.06))
            for j in range(nst):
                for i in range(nb):
                    c = at((i + 0.5) / nb, (j + 0.5) / nst)
                    if c[2] < -0.4:
                        continue                                         # under the rubble
                    hu, hv = 0.31 * bw, 0.26 * sh
                    r = rng.random()
                    bins["glass" if r < 0.55 else "void"].append(_box(c + N * 0.012, U, V, N, hu, hv, 0.012))
                    if r < 0.9:                                          # frame (missing on 1 in 10)
                        for du, dv, a, b in ((hu, 0, 0.035, hv + 0.035), (-hu, 0, 0.035, hv + 0.035),
                                             (0, hv, hu, 0.035), (0, -hv, hu, 0.035)):
                            bins["frame"].append(_box(c + U * du + V * dv + N * 0.04, U, V, N, a, b, 0.04))
                    if upper and fi == upper[0] and rng.random() < 0.8:  # balcony slab + parapet (west face)
                        f0 = at((i + 0.5) / nb, j / nst + 0.06 / L)
                        bins["balcony"].append(_box(f0 + N * 0.55, U, V, N, 0.45 * bw, 0.08, 0.55))
                        bins["balcony"].append(_box(f0 + N * 1.08 + V * 0.5, U, V, N, 0.45 * bw, 0.5, 0.06))
        # breaks: broken column stubs with bent rebar sticking out of both ends
        Ps = np.concatenate([q for q, _ in quads])
        for end, sgn in ((Ps[:, 1].min(), -1.0), (Ps[:, 1].max(), 1.0)):
            E = Ps[np.abs(Ps[:, 1] - end) < 0.5]
            E = np.unique(np.round(E, 3), axis=0)
            if len(E) < 4:
                continue
            ec = E.mean(0)
            ang = np.arctan2(E[:, 2] - ec[2], E[:, 0] - ec[0])
            D = E[np.argsort(ang)][:4]
            Y = np.array([0.0, sgn, 0.0])
            for a in (0.12, 0.37, 0.63, 0.88):
                for b in (0.12, 0.37, 0.63, 0.88):
                    p = (1 - a) * ((1 - b) * D[0] + b * D[1]) + a * ((1 - b) * D[3] + b * D[2])
                    if p[2] < -0.3 or rng.random() < 0.25:
                        continue
                    ln = rng.uniform(0.25, 1.3)
                    X = np.array([1.0, 0.0, 0.0])
                    Z = np.cross(X, Y)
                    bins["stub"].append(_box(p + Y * ln / 2, X, Y, Z, 0.24, ln / 2, 0.24))
                    for cx, cz in ((-0.17, -0.17), (0.17, -0.17), (0.17, 0.17), (-0.17, 0.17)):
                        p0 = p + X * cx + Z * cz + Y * (ln - 0.05)
                        L1 = rng.uniform(0.3, 0.9)
                        mid = p0 + Y * L1 * 0.6 + rng.normal(0, 0.05, 3)
                        tip = mid + (Y + rng.normal(0, 0.7, 3)) * L1 * 0.4
                        bins["rebar"] += [RB._bar(mid, p0, r=0.014), RB._bar(mid, tip, r=0.014)]
        out = f"{ROOT}/Facade/{t}"
        for k, parts in bins.items():
            mat = {"band": mats.get("concrete2"), "balcony": mats.get(FACE_MAT), "stub": mats.get("concrete2"),
                   "rebar": mats.get("rust")}.get(k, looks.get(k))
            tot += _emit(stage, f"{out}/{k}", parts, mat)
    for s in SLABS:
        for part in (("top", "edges") if slabs else ("edges",)):
            if "concrete2" in mats:
                PR.bind(stage, f"{SITE}/Buildings/Scanned/{s}/{part}", mats["concrete2"])
    log(f"[urban] fallen towers: plaster and concrete instead of the scan photo, {tot} facade/break pieces "
        f"(storey edges, columns, windows, balconies, column stubs, rebar); slabs: formed concrete")
    return tot


# ------------------------------------------------------------------------------------------------ debris
def _prototypes(rng):
    """[(name, kind, [(P, F, material key)], (smin, smax), sink, max_tilt)]  unit-sized pieces."""
    from isaac import photoreal as PR
    from isaac import rubble as RB
    out = []
    for k in range(7):
        P, F = RB.rock(rng, 1.0, rng.uniform(0.55, 1.0), rng.uniform(0.35, 0.7), n=16)
        out.append((f"chunk{k}", "chunk", [(P, F, "concrete" if k % 2 else "concrete2")], (0.16, 1.3), 0.12, 180))
    for k in range(3):
        P, F = RB.block(rng, 1.0, rng.uniform(0.5, 0.9), rng.uniform(0.3, 0.6), chip=0.12)
        out.append((f"block{k}", "block", [(P, F, "concrete2")], (0.3, 1.2), 0.1, 180))
    for k in range(4):
        P, F = RB.slab(rng, 1.0, rng.uniform(0.55, 1.0), T=0.18, step=0.08)
        out.append((f"slab{k}", "slab", [(P, F, "concrete2" if k % 2 else "concrete")], (0.6, 2.8), 0.02, 32))
    for k in range(2):
        parts = []
        for _ in range(rng.integers(3, 7)):
            c = rng.uniform(-0.5, 0.5, 3) * [0.6, 0.6, 0.2]
            P, F = RB.block(rng, 0.23, 0.11, 0.06, chip=0.1)
            parts.append((P + c.astype(np.float32), F))
        P, F = RB.merge(parts)
        out.append((f"bricks{k}", "brick", [(P, F, "brick")], (0.6, 1.4), 0.02, 25))
    P, F = RB.slab(rng, 1.0, 0.8, T=0.04, step=0.1)
    out.append(("tiles0", "tile", [(P, F, "tile")], (0.12, 0.4), 0.0, 40))
    for k in range(3):
        bars = []
        for _ in range(rng.integers(2, 5)):
            p0 = np.array([rng.uniform(-0.5, 0.5), rng.uniform(-0.2, 0.2), 0.0])
            mid = p0 + np.array([rng.uniform(0.2, 0.5), rng.normal(0, 0.15), rng.uniform(0.0, 0.35)])
            tip = mid + np.array([rng.uniform(0.1, 0.4), rng.normal(0, 0.25), rng.uniform(-0.1, 0.5)])
            bars += [RB._bar(p0, mid, r=0.012), RB._bar(mid, tip, r=0.012)]
        P, F = RB.merge(bars)
        out.append((f"rebar{k}", "rebar", [(P, F, "rust")], (0.7, 1.8), 0.0, 25))
    for k in range(2):
        (P, F), (Pb, Fb) = PR._column(rng, 1.0, 0.32, 0.32)
        out.append((f"column{k}", "column", [(P, F, "concrete2"), (Pb, Fb, "rust")], (1.2, 2.6), 0.08, 30))
    return out


def debris(stage, mats, looks, faces, log, seed=0, n_core=32000, n_outer=9000):
    """Concrete chunks, blocks, slab shards, brick, tile flakes, rebar and broken columns on the rubble faces
    (one PointInstancer; visual only)."""
    from pxr import Gf, Sdf, UsdGeom, Vt
    from isaac import photoreal as PR
    if faces is None:
        return 0
    C, N, A, T = faces
    rng = np.random.default_rng(4200 + seed)
    protos = _prototypes(rng)
    inst_path = f"{ROOT}/Debris"
    inst = UsdGeom.PointInstancer.Define(stage, inst_path)
    rel = inst.CreatePrototypesRel()
    for name, kind, parts, _, _, _ in protos:
        proot = f"{inst_path}/Prototypes/{name}"
        UsdGeom.Xform.Define(stage, proot)
        for k, (P, F, mk) in enumerate(parts):
            pth = f"{proot}/g{k}"
            PR._mesh(stage, pth, P, F, (0.55, 0.53, 0.5))
            mat = mats.get(mk, looks.get(mk))
            if mat is not None:
                PR.bind(stage, pth, mat)
        rel.AddTarget(Sdf.Path(proot))
    kinds = [p[1] for p in protos]
    mix = {"chunk": 0.50, "block": 0.10, "slab": 0.13, "brick": 0.10, "tile": 0.07, "rebar": 0.07, "column": 0.03}
    core = _in_core(C[:, 0], C[:, 1], pad=4.0)
    out = []
    for sel, n in ((core, n_core), (~core, n_outer)):
        k = np.nonzero(sel)[0]
        if not len(k) or n <= 0:
            continue
        f = k[rng.choice(len(k), n, p=A[k] / A[k].sum())]
        a, b = rng.random((2, n))
        flip = a + b > 1
        a[flip], b[flip] = 1 - a[flip], 1 - b[flip]
        pos = T[f, 0] + a[:, None] * (T[f, 1] - T[f, 0]) + b[:, None] * (T[f, 2] - T[f, 0])
        kind = rng.choice(list(mix), n, p=np.array(list(mix.values())) / sum(mix.values()))
        out.append((pos, N[f], kind))
    pos, nrm, kind = (np.concatenate([o[i] for o in out]) for i in range(3))
    n = len(pos)
    idx = np.empty(n, np.int32)
    scl = np.empty((n, 3))
    q = np.empty((n, 4))
    for kd in mix:
        m = np.nonzero(kind == kd)[0]
        if not len(m):
            continue
        choices = [i for i, kk in enumerate(kinds) if kk == kd]
        idx[m] = rng.choice(choices, len(m))
        smin, smax = protos[choices[0]][3]
        s = smin * (smax / smin) ** (rng.random(len(m)) ** 1.9)            # many small, few big
        if kd == "slab":
            scl[m] = np.stack([s, s * rng.uniform(0.55, 1.0, len(m)), rng.uniform(0.6, 1.4, len(m))], 1)
        elif kd == "tile":
            scl[m] = np.stack([s, s, np.ones(len(m))], 1)
        else:
            scl[m] = np.repeat(s[:, None], 3, 1)
        tilt = protos[choices[0]][5]
        q[m] = _q_random(rng, len(m)) if tilt >= 180 else _q_on_surface(rng, nrm[m], tilt)
        pos[m] += nrm[m] * (protos[choices[0]][4] * s)[:, None]
    inst.CreateProtoIndicesAttr(Vt.IntArray.FromNumpy(idx))
    inst.CreatePositionsAttr(Vt.Vec3fArray.FromNumpy(pos.astype(np.float32)))
    inst.CreateScalesAttr(Vt.Vec3fArray.FromNumpy(scl.astype(np.float32)))
    inst.CreateOrientationsAttr(Vt.QuathArray([Gf.Quath(float(w), float(x), float(y), float(z)) for w, x, y, z in q]))
    counts = {kd: int((kind == kd).sum()) for kd in mix}
    log(f"[urban] 3D debris: {n} pieces on the rubble ({', '.join(f'{v} {k}' for k, v in counts.items())}); "
        "visual only")
    # the map's own scatter: same pieces, real materials instead of the blurry atlas
    nb = 0
    for p in ("chunk", "slabshard", "block", "brick", "tile"):
        for k in range(8):
            pth = f"{SITE}/Debris/Scatter/Prototypes/{p}{k}"
            if stage.GetPrimAtPath(pth).IsValid():
                key = {"brick": "brick", "tile": "tile"}.get(p, "concrete2" if k % 2 else "concrete")
                mat = mats.get(key, looks.get(key))
                if mat is not None:
                    PR.bind(stage, pth, mat)
                    nb += 1
    if nb:
        log(f"[urban] the map's 56k scatter: {nb} prototypes moved from the photo atlas to concrete/brick materials")
    return n


# ------------------------------------------------------------------------------------------------ crew
def crew(stage, sc, isa, log):
    from isaac import photoreal as PR
    from isaac.site import base_xy
    bx, by = base_xy(sc)
    n = 0
    for k, (ch, dx, dy, yaw) in enumerate(CREW):
        try:
            PR.reference(stage, f"{ROOT}/Crew/p{k}", f"{isa}/People/Characters/{ch}", (bx + dx, by + dy, 0.0), yaw)
            n += 1
        except Exception as e:
            log(f"[urban] crew member {k} off ({type(e).__name__}: {e})")
    log(f"[urban] rescue crew: {n} people at the staging area")
    return n


# ------------------------------------------------------------------------------------------------ scan AI
def scan_ai(stage, folder, log):
    """Point the map's scan-photo materials (terrain tiles, scan roofs) at the AI-refined textures in
    ``folder`` (isaac/tools/ai_textures.py): same layout and colours, re-drawn fine detail."""
    import os
    from pxr import Sdf
    n = 0
    looks = stage.GetPrimAtPath(f"{SITE}/Looks")
    if not looks.IsValid() or not folder or not os.path.isdir(folder):
        log(f"[urban] scan AI textures: none ({folder or 'no --scan-ai folder'}); scan photo as shipped")
        return 0
    for mat in looks.GetChildren():
        name = mat.GetName()
        alb, nrm = (os.path.join(folder, f"{name}_{k}.png") for k in ("albedo", "normal"))
        if not os.path.exists(alb):
            continue
        for sh, attr, path in (("OmniPBR", "inputs:diffuse_texture", alb), ("OmniPBR", "inputs:normalmap_texture", nrm),
                               ("albedoTex", "inputs:file", alb), ("normalTex", "inputs:file", nrm)):
            p = stage.GetPrimAtPath(f"{mat.GetPath()}/{sh}")
            if p.IsValid() and os.path.exists(path):
                a = p.GetAttribute(attr)
                if a and a.IsValid():
                    a.Set(Sdf.AssetPath(os.path.abspath(path)))
        n += 1
    log(f"[urban] scan AI textures: {n} map materials now use the AI-refined scan photos from {folder}")
    return n


# Real-time quality for the live take: DLSS family. Keys differ between Kit versions; unknown keys are harmless.
RT_QUALITY = {
    "/rtx/post/aa/op": 4,                       # 3 = DLSS super resolution, 4 = DLAA (AI anti-aliasing at native res)
    "/rtx/post/dlss/execMode": 2,               # quality
    "/rtx/rendermode": "RealTimePathTracing",   # "RTX Real-Time 2.0": DLSS ray reconstruction (AI denoiser)
    "/rtx/post/motionblur/enabled": True, "/rtx/post/motionblur/exposureFraction": 0.5,
}


def rt_quality(a=None, log=print):
    """DLAA/DLSS and ray reconstruction for the live real-time renderer (--rt-quality 1)."""
    try:
        import carb
        s = carb.settings.get_settings()
        modes = s.get("/rtx/rendermode")
        for k, v in RT_QUALITY.items():
            if k == "/rtx/rendermode" and getattr(a, "rtx", "realtime") == "pathtracing":
                continue
            if isinstance(v, bool):
                s.set_bool(k, v)
            elif isinstance(v, int):
                s.set_int(k, v)
            elif isinstance(v, float):
                s.set_float(k, v)
            else:
                s.set_string(k, v)
        log(f"[urban] real-time quality: DLAA/DLSS quality, ray reconstruction mode (was {modes!r}), motion blur")
    except Exception as e:
        log(f"[urban] real-time quality skipped ({type(e).__name__}: {e})")


def grade(a=None, log=print):
    """The map's photographic grade (fixed exposure, ACES, AO, grain), replacing photoreal's generic haze."""
    try:
        import carb
        s = carb.settings.get_settings()
        ok = 0
        for k, v in GRADE.items():
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
            except Exception:
                pass
        log(f"[urban] grade: {ok}/{len(GRADE)} render settings (fixed ACES exposure, contrast 1.15, light z-up haze)")
    except Exception as e:
        log(f"[urban] grade skipped ({type(e).__name__}: {e})")


# ------------------------------------------------------------------------------------------------ entry
def dress(stage, sc, a, mats, nv, isa, log=print):
    """Called by photoreal.dress on the collapse map (after the map is referenced, before PhysX loads)."""
    from isaac import photoreal as PR
    seed = int(getattr(sc, "seed", 0) or 0)
    for k, rel in EXTRA_MDL.items():
        if k not in mats:
            try:
                scale = {"stucco": (0.5, 0.5), "soil": (0.18, 0.18), "concrete3": (0.3, 0.3)}[k]
                mats[k] = PR.mdl_material(stage, f"/World/Looks/{k}", f"{nv}/Materials/Base/{rel}", scale)
            except Exception as e:
                log(f"[urban] material {k} failed: {type(e).__name__}: {e}")
    looks = {"glass": _pv(stage, f"{ROOT}/Looks/glass", (0.035, 0.045, 0.055), 0.06, 0.0, 1.0),
             "void": _pv(stage, f"{ROOT}/Looks/void", (0.012, 0.011, 0.010), 0.95),
             "frame": _pv(stage, f"{ROOT}/Looks/frame", (0.62, 0.63, 0.64), 0.35, 0.8),
             "tile": _pv(stage, f"{ROOT}/Looks/tile", (0.80, 0.78, 0.72), 0.3, 0.0, 0.5)}
    scan = str(getattr(a, "urban_terrain", "scan")) == "scan"
    if scan:
        scan_ai(stage, getattr(a, "scan_ai", ""), log)
    faces = terrain(stage, mats, log, seed, rematerial=not scan)
    facade(stage, mats, looks, log, seed, slabs=not scan)
    debris(stage, mats, looks, faces, log, seed, n_core=int(getattr(a, "urban_debris", 32000)),
           n_outer=int(getattr(a, "urban_debris", 32000) * 0.28))
    if getattr(a, "crew", 1):
        crew(stage, sc, isa, log)
