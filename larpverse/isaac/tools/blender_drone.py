#!/usr/bin/env python3
"""Procedural, render-grade model of the heavy-lift SAR drone (Blender ``bpy``, headless) -> USD for Isaac Sim.

Same airframe as ``sim.platform.get_platform("heavy_sar")``: coaxial X8 (4 arms at 45 + 90 i deg, upper and lower
rotor on each arm), 2.2 m wheelbase, 54-inch two-blade props, landing gear footprint 1.6 x 1.4 m, fuselage
underside 0.60 m above the skid bottom.  Generic enterprise cargo/SAR design (FlyCart 30 / Agras T50 class), no
brand names, logos or copied designs.

    pip install bpy usd-core            # bpy 5.x wheel (Python 3.13); usd-core only for the pxr fix-up/validation
    python blender_drone.py --out isaac/assets/drone                       # -> heavy_sar.usdc + textures/
    python blender_drone.py --out isaac/assets/drone --render PREVIEW_DIR  # + Cycles CPU previews
    python blender_drone.py --postprocess isaac/assets/drone/heavy_sar.usdc  # pxr fix-up + validation only

USD contract (isaac/drone_viz.py STYLE "blender"): metres, Z up, +X forward, skid bottom at z = 0; default prim
/HeavySAR; each prop is an Xform ``Rotor{i}_{lvl}`` (i = arm 0..3 at 45 + 90 i deg, lvl 1 upper / 0 lower) whose
origin is the hub on the motor axis, with ``userProperties:spinSign`` (+1 = counter-clockwise from above);
the white anti-collision strobe lens is the prim ``Strobe``; nav lights / strobe / LEDs carry an emissiveColor.
Materials are UsdPreviewSurface (Blender's exporter) with the carbon weave baked to PNGs in ``textures/``.
"""
from __future__ import annotations

import argparse
import math
import os
import struct
import subprocess
import sys
import zlib

import numpy as np

# ----------------------------------------------------------------------------------------------- layout (metres)
Z0 = 0.60                    # skid bottom -> fuselage underside (drone_mesh.GEAR_H)
R_ARM = 2.20 / 2             # hub -> motor axis (wheelbase / 2)
PROP_R = 1.375 / 2           # 54-inch prop
GEAR_L, GEAR_W = 1.6, 1.4    # landing-gear footprint (skid length x track)
Z_TIP = Z0 + 0.54            # motor pod centre (arm tip)
COAX = 0.17                  # pod centre -> prop hub (upper +, lower -)
OPACITY_USD = {"M_BlurLo": 0.07, "M_BlurHi1": 0.30, "M_BlurHi2": 0.17, "M_BlurHi3": 0.08}   # prop-blur discs
EMISSIVE_USD = {             # UsdPreviewSurface emissiveColor for Isaac RTX (same scale as drone_viz.EMISSIVE)
    "M_NavRed": (160.0, 8.0, 4.0), "M_NavGreen": (8.0, 130.0, 16.0), "M_Strobe": (260.0, 260.0, 270.0),
    "M_Led": (4.0, 40.0, 8.0),
}


def spin_sign(i, lvl):
    """+1 = counter-clockwise seen from above (positive RotateZ). Arms alternate, upper/lower contra-rotate."""
    return (1 if i % 2 == 0 else -1) * (1 if lvl == 1 else -1)


# ----------------------------------------------------------------------------------------------- tiny PNG writer
def write_png(path, arr):
    """uint8 HxWxC (C = 1, 3, 4), row 0 = top of the image."""
    arr = np.ascontiguousarray(arr, np.uint8)
    if arr.ndim == 2:
        arr = arr[:, :, None]
    h, w, c = arr.shape
    raw = b"".join(b"\x00" + arr[y].tobytes() for y in range(h))

    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    ihdr = struct.pack(">IIBBBBB", w, h, 8, {1: 0, 3: 2, 4: 6}[c], 0, 0, 0)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def _srgb(lin):
    lin = np.clip(lin, 0, 1)
    return np.where(lin <= 0.0031308, lin * 12.92, 1.055 * lin ** (1 / 2.4) - 0.055)


def carbon_textures(texdir, S=512, T=8, seed=3):
    """2x2 twill carbon weave (T tows per tile): albedo (sRGB), roughness, OpenGL normal map. Tileable."""
    rng = np.random.default_rng(seed)
    v, u = (np.mgrid[0:S, 0:S] + 0.5) / S * T          # v up (flipped on write), u right; in tow units
    iu, iv = np.floor(u).astype(int), np.floor(v).astype(int)
    fu, fv = u - iu, v - iv
    warp = ((iu + iv) % 4) < 2                          # warp tow (fibres along v) on top
    # where along its 2-cell float the visible tow is (0..1), for the over/under undulation
    pos_w = (((iu + iv) % 4) + fv) / 2.0                # warp float spans 2 cells in v
    pos_f = (((iu + iv) % 4 - 2) + fu) / 2.0            # weft float spans 2 cells in u
    across = np.where(warp, fu, fv)
    along = np.where(warp, pos_w, pos_f)
    prof = np.sin(np.pi * np.clip(across, 0, 1)) ** 0.45
    und = 0.75 + 0.25 * np.sin(np.pi * np.clip(along, 0, 1))
    hgt = prof * und
    # fibre striations: fine noise across the tow, stretched along it
    nf = rng.standard_normal((S, 64))
    stri = np.interp(np.linspace(0, 64, S, endpoint=False), np.arange(64), nf[0])
    stri_u = np.tile(stri[None, :], (S, 1))
    stri_v = np.tile(stri[::-1][:, None], (1, S))
    fib = np.where(warp, stri_u, stri_v) * 0.08
    sheen = np.where(warp, 1.0, 0.62)                   # the two fibre directions reflect differently
    alb = 0.028 * sheen * (0.55 + 0.45 * hgt) * (1 + fib)
    albedo = np.stack([alb, alb * 1.01, alb * 1.06], -1)
    rough = 0.30 + 0.16 * (1 - hgt) + 0.04 * fib
    gy, gx = np.gradient(hgt)                           # d/dv (rows), d/du (cols)
    k = 3.0
    n = np.stack([-gx * k * S / T * 0.04, -gy * k * S / T * 0.04, np.ones_like(hgt)], -1)
    n /= np.linalg.norm(n, axis=-1, keepdims=True)
    flip = lambda a: a[::-1]                             # row 0 = top = v max
    os.makedirs(texdir, exist_ok=True)
    paths = {
        "albedo": os.path.join(texdir, "carbon_albedo.png"),
        "rough": os.path.join(texdir, "carbon_rough.png"),
        "normal": os.path.join(texdir, "carbon_normal.png"),
    }
    write_png(paths["albedo"], flip(_srgb(albedo) * 255 + 0.5).astype(np.uint8))
    write_png(paths["rough"], flip(np.clip(rough, 0, 1) * 255 + 0.5).astype(np.uint8))
    write_png(paths["normal"], flip((n * 0.5 + 0.5) * 255 + 0.5).astype(np.uint8))
    return paths


def composite_textures(texdir, S=512, seed=5):
    """Subtle composite-shell roughness breakup (orange-peel + light wear), tileable, Non-Color."""
    rng = np.random.default_rng(seed)

    def tile_noise(cells):
        g = rng.standard_normal((cells, cells))
        f = np.fft.fft2(np.pad(g, 0))
        x = np.real(np.fft.ifft2(f))
        idx = np.linspace(0, cells, S, endpoint=False)
        i0 = np.floor(idx).astype(int)
        t = idx - i0
        t = t * t * (3 - 2 * t)
        a = x[np.ix_(i0 % cells, i0 % cells)]
        b = x[np.ix_((i0 + 1) % cells, i0 % cells)]
        c = x[np.ix_(i0 % cells, (i0 + 1) % cells)]
        d = x[np.ix_((i0 + 1) % cells, (i0 + 1) % cells)]
        ty, tx = t[:, None], t[None, :]
        return (a * (1 - ty) + b * ty) * (1 - tx) + (c * (1 - ty) + d * ty) * tx
    n = 0.5 * tile_noise(8) + 0.3 * tile_noise(32) + 0.2 * tile_noise(128)
    n = (n - n.mean()) / (n.std() + 1e-9)
    rough = np.clip(0.5 + 0.06 * n, 0, 1)
    p = os.path.join(texdir, "composite_rough.png")
    write_png(p, (rough * 255 + 0.5).astype(np.uint8))
    return {"rough": p}


# ----------------------------------------------------------------------------------------------- geometry (numpy)
def _frame(w):
    w = np.asarray(w, float)
    w = w / np.linalg.norm(w)
    t = np.array([0.0, 0.0, 1.0]) if abs(w[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(t, w)
    u /= np.linalg.norm(u)
    v = np.cross(w, u)
    return u, v, w


def rot_to(xaxis, zhint=(0, 0, 1)):
    """3x3 matrix whose columns are a right-handed frame with x along ``xaxis`` and z as close to zhint."""
    x = np.asarray(xaxis, float)
    x /= np.linalg.norm(x)
    z = np.asarray(zhint, float)
    z = z - x * np.dot(z, x)
    if np.linalg.norm(z) < 1e-6:
        z = np.array([0.0, 1.0, 0.0]) - x * x[1]
    z /= np.linalg.norm(z)
    y = np.cross(z, x)
    return np.stack([x, y, z], 1)


class Part:
    """Geometry accumulator -> one Blender object (multi-material, optional UVs)."""

    def __init__(self, name):
        self.name, self.P, self.F, self.UV, self.M, self.mats, self.n, self.has_uv = name, [], [], [], [], [], 0, False

    def add(self, P, F, mat, uv=None):
        if mat not in self.mats:
            self.mats.append(mat)
        mi = self.mats.index(mat)
        P = np.asarray(P, float).reshape(-1, 3)
        o = self.n
        self.P.append(P)
        self.F += [tuple(int(k) + o for k in f) for f in F]
        self.M += [mi] * len(F)
        if uv is None:
            self.UV += [[(0.0, 0.0)] * len(f) for f in F]
        else:
            self.has_uv = True
            self.UV += [list(map(tuple, c)) for c in uv]
        self.n += len(P)
        return self

    def addg(self, g, mat):
        return self.add(g[0], g[1], mat, g[2] if len(g) > 2 else None)


def xf(g, R=None, t=(0, 0, 0)):
    """Transform a (P, F[, UV]) geometry tuple: p' = R p + t."""
    P = np.asarray(g[0], float)
    if R is not None:
        P = P @ np.asarray(R, float).T
    P = P + np.asarray(t, float)
    return (P,) + tuple(g[1:])


def lathe(profile, n=32, c=(0, 0, 0), axis=(0, 0, 1), uvs=None):
    """Surface of revolution. profile: [(r, h)] along the axis; r == 0 ends become poles. uvs = (su, sv) metres
    per UV unit (around, along) for tiled textures."""
    prof = np.asarray(profile, float)
    u, v, w = _frame(axis)
    c = np.asarray(c, float)
    ang = np.linspace(0, 2 * math.pi, n, endpoint=False)
    ca, sa = np.cos(ang), np.sin(ang)
    P, rings = [], []
    for r, h in prof:
        if r <= 1e-9:
            rings.append([len(P)])
            P.append(c + w * h)
        else:
            idx = list(range(len(P), len(P) + n))
            P += list(c + w * h + r * (ca[:, None] * u + sa[:, None] * v))
            rings.append(idx)
    F, UV = [], []
    seg = np.r_[0, np.cumsum(np.hypot(np.diff(prof[:, 0]), np.diff(prof[:, 1])))]
    rmax = max(prof[:, 0].max(), 1e-6)
    su, sv = uvs if uvs else (2 * math.pi * rmax, seg[-1] if seg[-1] > 0 else 1.0)
    for k in range(len(rings) - 1):
        a, b = rings[k], rings[k + 1]
        v0, v1 = seg[k] / sv, seg[k + 1] / sv
        for j in range(n):
            j1 = (j + 1) % n
            u0 = 2 * math.pi * rmax * j / n / su
            u1 = 2 * math.pi * rmax * (j + 1) / n / su
            if len(a) == 1 and len(b) == 1:
                continue
            if len(a) == 1:
                F.append((a[0], b[j1], b[j]))
                UV.append([((u0 + u1) / 2, v0), (u1, v1), (u0, v1)])
            elif len(b) == 1:
                F.append((a[j], a[j1], b[0]))
                UV.append([(u0, v0), (u1, v0), ((u0 + u1) / 2, v1)])
            else:
                F.append((a[j], a[j1], b[j1], b[j]))
                UV.append([(u0, v0), (u1, v0), (u1, v1), (u0, v1)])
    return np.array(P), F, UV


def cyl(p0, p1, r, n=24, cap=True, chamfer=0.0, r1=None, uvs=None):
    """Cylinder / frustum from p0 to p1 with optional chamfered caps (via lathe)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    L = np.linalg.norm(p1 - p0)
    r1 = r if r1 is None else r1
    ch = min(chamfer, 0.45 * L, 0.45 * min(r, r1))
    prof = []
    if cap:
        prof.append((0.0, 0.0))
    if ch > 0:
        prof += [(r - ch, 0.0), (r, ch), (r1, L - ch), (r1 - ch, L)]
    else:
        prof += [(r, 0.0), (r1, L)]
    if cap:
        prof.append((0.0, L))
    return lathe(prof, n, p0, p1 - p0, uvs)


def tube_path(pts, r, n=12, cap=True, uvs=None):
    """Swept circular tube along a polyline (parallel-transport frames)."""
    pts = np.asarray(pts, float)
    T = np.gradient(pts, axis=0)
    T /= np.linalg.norm(T, axis=1, keepdims=True)
    u, v, _ = _frame(T[0])
    ang = np.linspace(0, 2 * math.pi, n, endpoint=False)
    P, F, UV = [], [], []
    L = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    su, sv = uvs if uvs else (2 * math.pi * r, max(L[-1], 1e-6))
    for k, p in enumerate(pts):
        if k:
            t0, t1 = T[k - 1], T[k]
            ax = np.cross(t0, t1)
            s = np.linalg.norm(ax)
            if s > 1e-9:
                ax /= s
                a = math.atan2(s, np.dot(t0, t1))
                rot = lambda x: x * math.cos(a) + np.cross(ax, x) * math.sin(a) + ax * np.dot(ax, x) * (1 - math.cos(a))
                u, v = rot(u), rot(v)
        P += list(p + r * (np.cos(ang)[:, None] * u + np.sin(ang)[:, None] * v))
    m = len(pts)
    for k in range(m - 1):
        for j in range(n):
            j1 = (j + 1) % n
            F.append((k * n + j, k * n + j1, (k + 1) * n + j1, (k + 1) * n + j))
            u0, u1 = 2 * math.pi * r * j / n / su, 2 * math.pi * r * (j + 1) / n / su
            UV.append([(u0, L[k] / sv), (u1, L[k] / sv), (u1, L[k + 1] / sv), (u0, L[k + 1] / sv)])
    if cap:
        c0, c1 = len(P), len(P) + 1
        P += [pts[0], pts[-1]]
        for j in range(n):
            j1 = (j + 1) % n
            F.append((c0, j1, j))
            F.append((c1, (m - 1) * n + j, (m - 1) * n + j1))
            UV += [[(0, 0)] * 3, [(0, 0)] * 3]
    return np.array(P), F, UV


def rbox(c, size, r=0.004, seg=3, R=None):
    """Rounded box (bmesh bevel), centred at c, axes = columns of R."""
    import bmesh
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    sx, sy, sz = size
    for vert in bm.verts:
        vert.co.x *= sx
        vert.co.y *= sy
        vert.co.z *= sz
    r = min(r, 0.49 * min(size))
    if r > 1e-5 and seg > 0:
        bmesh.ops.bevel(bm, geom=list(bm.edges) + list(bm.verts), offset=r, offset_type="OFFSET", segments=seg,
                        profile=0.5, affect="EDGES", clamp_overlap=True)
    P = np.array([vv.co[:] for vv in bm.verts])
    bm.verts.index_update()
    F = [tuple(vv.index for vv in f.verts) for f in bm.faces]
    bm.free()
    return xf((P, F), R, c)


def superellipsoid(c, a, b, h, e1, e2, nu=64, nv=32, deform=None):
    """Rounded body (half sizes a, b, h): small e = boxy. UV = (azimuth, elevation). Poles as triangle fans."""
    f = lambda w, e: np.sign(w) * np.abs(w) ** e
    vv = np.linspace(-math.pi / 2, math.pi / 2, nv + 1)[1:-1]
    uu = np.linspace(-math.pi, math.pi, nu, endpoint=False)
    V, U = np.meshgrid(vv, uu, indexing="ij")
    P = np.stack([a * f(np.cos(V), e1) * f(np.cos(U), e2), b * f(np.cos(V), e1) * f(np.sin(U), e2),
                  h * f(np.sin(V), e1)], -1).reshape(-1, 3)
    P = np.vstack([P, [[0, 0, -h], [0, 0, h]]])
    if deform is not None:
        P = deform(P)
    P = P + np.asarray(c, float)
    rows = nv - 1
    F, UV = [], []
    for i in range(rows - 1):
        for j in range(nu):
            j1 = (j + 1) % nu
            F.append((i * nu + j, i * nu + j1, (i + 1) * nu + j1, (i + 1) * nu + j))
            UV.append([(j / nu, i / rows), ((j + 1) / nu, i / rows), ((j + 1) / nu, (i + 1) / rows), (j / nu, (i + 1) / rows)])
    s, nn = rows * nu, rows * nu + 1
    for j in range(nu):
        j1 = (j + 1) % nu
        F.append((s, j1, j))
        F.append((nn, (rows - 1) * nu + j, (rows - 1) * nu + j1))
        UV += [[(j / nu, 0), ((j + 1) / nu, 0), (j / nu, 0)], [(j / nu, 1), (j / nu, 1), ((j + 1) / nu, 1)]]
    return P, F, UV, (rows, nu)


def grid_normals(P, rows, nu):
    """Vertex normals of a superellipsoid grid (rows x nu, wrapped in u)."""
    G = P[: rows * nu].reshape(rows, nu, 3)
    du = np.roll(G, -1, 1) - np.roll(G, 1, 1)
    dv = np.zeros_like(G)
    dv[1:-1] = G[2:] - G[:-2]
    dv[0], dv[-1] = G[1] - G[0], G[-1] - G[-2]
    N = np.cross(du, dv)
    N /= np.maximum(np.linalg.norm(N, axis=-1, keepdims=True), 1e-12)
    return G, N


def ribbon(G, N, path_idx, width=0.0024, lift=0.0007, closed=True):
    """Thin strip floating on a grid surface along a list of (i, j) grid indices: reads as a panel seam."""
    pts = np.array([G[i, j] for i, j in path_idx])
    nrm = np.array([N[i, j] for i, j in path_idx])
    T = np.roll(pts, -1, 0) - np.roll(pts, 1, 0) if closed else np.gradient(pts, axis=0)
    B = np.cross(nrm, T)
    B /= np.maximum(np.linalg.norm(B, axis=1, keepdims=True), 1e-12)
    base = pts + nrm * lift
    P = np.vstack([base - B * width / 2, base + B * width / 2])
    m = len(pts)
    F = [(k, (k + 1) % m, m + (k + 1) % m, m + k) for k in range(m if closed else m - 1)]
    return P, F


def naca(m, p, t, n=14):
    beta = np.linspace(0, math.pi, n)
    xc = (1 - np.cos(beta)) / 2
    yt = 5 * t * (0.2969 * np.sqrt(xc) - 0.1260 * xc - 0.3516 * xc ** 2 + 0.2843 * xc ** 3 - 0.1036 * xc ** 4)
    if m > 0:
        yc = np.where(xc < p, m / p ** 2 * (2 * p * xc - xc ** 2), m / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * xc - xc ** 2))
        dy = np.where(xc < p, 2 * m / p ** 2 * (p - xc), 2 * m / (1 - p) ** 2 * (p - xc))
    else:
        yc = dy = np.zeros_like(xc)
    th = np.arctan(dy)
    xu, zu = xc - yt * np.sin(th), yc + yt * np.cos(th)
    xl, zl = xc + yt * np.sin(th), yc - yt * np.cos(th)
    X = np.r_[xu[::-1], xl[1:-1]]
    Z = np.r_[zu[::-1], zl[1:-1]]
    return X, Z


def blade(R, sign=1, r0=0.040, ns=30, pitch=0.43, n_af=13):
    """One prop blade along +x: cambered NACA sections, chord taper, washout twist (constant geometric pitch),
    slight back-swept rounded tip. sign +1: leading edge on +y (counter-clockwise rotor)."""
    s = np.linspace(0, 1, ns) ** 0.9
    s = np.r_[s[:-4], 1 - (1 - np.linspace(s[-5], 1, 5)[1:]) ** 1.0]
    s = np.unique(np.clip(s, 0, 1))
    r = r0 + (R - r0) * s
    chord = np.interp(s, [0, 0.05, 0.11, 0.22, 0.32, 0.55, 0.80, 0.92, 1.0],
                      [0.026, 0.028, 0.070, 0.112, 0.118, 0.100, 0.075, 0.058, 0.044])
    tip = np.clip((s - 0.93) / 0.07, 0, 1)
    chord = chord * np.sqrt(np.clip(1 - tip ** 2, 0.02, 1))
    tk = np.interp(s, [0, 0.06, 0.13, 0.25, 1.0], [0.62, 0.55, 0.22, 0.13, 0.085])
    cam = np.interp(s, [0, 0.08, 0.25, 1.0], [0.0, 0.0, 0.045, 0.035])
    beta = np.arctan(pitch / (2 * math.pi * np.maximum(r, 0.115)))
    yoff = -0.018 * s ** 2.2                              # trailing-side sweep of the pitch axis toward the tip
    secs = []
    for k in range(len(s)):
        X, Zc = naca(cam[k], 0.4, tk[k], n_af)
        y = (0.30 - X) * chord[k] + yoff[k]
        z = Zc * chord[k]
        cb, sb = math.cos(beta[k]), math.sin(beta[k])
        yy, zz = y * cb - z * sb, y * sb + z * cb
        secs.append(np.stack([np.full_like(yy, r[k]), yy, zz], 1))
    m = len(secs[0])
    P = np.vstack(secs)
    F = []
    for k in range(len(secs) - 1):
        for j in range(m):
            j1 = (j + 1) % m
            F.append((k * m + j, k * m + j1, (k + 1) * m + j1, (k + 1) * m + j))
    c0, c1 = len(P), len(P) + 1
    P = np.vstack([P, secs[0].mean(0), secs[-1].mean(0)])
    for j in range(m):
        j1 = (j + 1) % m
        F.append((c0, j1, j))
        F.append((c1, (len(secs) - 1) * m + j, (len(secs) - 1) * m + j1))
    if sign < 0:                                          # mirror (y -> -y) and flip winding
        P[:, 1] *= -1
        F = [tuple(reversed(f)) for f in F]
    return P, F


def fan_boxes(c, axis, n, r_in, r_out, h, t, z0=0.0, phase=0.0):
    """n radial fins (thin boxes) around an axis through c: cooling fins."""
    u, v, w = _frame(axis)
    P, F = [], []
    for k in range(n):
        a = phase + 2 * math.pi * k / n
        d = math.cos(a) * u + math.sin(a) * v
        R = np.stack([d, np.cross(w, d), w], 1)
        g = rbox(np.asarray(c) + d * (r_in + r_out) / 2 + w * (z0 + h / 2), ((r_out - r_in), t, h), r=0.0, seg=0, R=R)
        o = len(P)
        P += list(g[0])
        F += [tuple(i + o for i in f) for f in g[1]]
    return np.array(P), F


# ----------------------------------------------------------------------------------------------- bpy helpers
def bpy_setup():
    import bpy
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.unit_settings.system = "METRIC"
    sc.unit_settings.scale_length = 1.0
    return bpy


def make_material(bpy, name, base, rough=0.5, metal=0.0, coat=0.0, coat_rough=0.05, emit=None, emit_strength=0.0,
                  spec=0.5, tex=None, ior=1.5):
    mat = bpy.data.materials.new(name)
    try:
        mat.use_nodes = True
    except Exception:
        pass
    nt = mat.node_tree
    bsdf = nt.nodes.get("Principled BSDF") or nt.nodes.new("ShaderNodeBsdfPrincipled")
    I = bsdf.inputs
    I["Base Color"].default_value = (*base, 1.0)
    I["Roughness"].default_value = rough
    I["Metallic"].default_value = metal
    I["IOR"].default_value = ior
    I["Specular IOR Level"].default_value = spec
    I["Coat Weight"].default_value = coat
    I["Coat Roughness"].default_value = coat_rough
    if emit is not None:
        I["Emission Color"].default_value = (*emit, 1.0)
        I["Emission Strength"].default_value = emit_strength
    mat.diffuse_color = (*base, 1.0)
    if tex:
        def img_node(path, noncolor, x):
            n = nt.nodes.new("ShaderNodeTexImage")
            n.image = bpy.data.images.load(path, check_existing=True)
            if noncolor:
                n.image.colorspace_settings.name = "Non-Color"
            n.location = (x, 0)
            n.extension = "REPEAT"
            n.interpolation = "Linear"
            return n
        if "albedo" in tex:
            nt.links.new(img_node(tex["albedo"], False, -600).outputs["Color"], I["Base Color"])
        if "rough" in tex:
            nt.links.new(img_node(tex["rough"], True, -600).outputs["Color"], I["Roughness"])
        if "normal" in tex:
            nm = nt.nodes.new("ShaderNodeNormalMap")
            nm.inputs["Strength"].default_value = 1.0
            nt.links.new(img_node(tex["normal"], True, -900).outputs["Color"], nm.inputs["Color"])
            nt.links.new(nm.outputs["Normal"], I["Normal"])
    return mat


def make_materials(bpy, texdir):
    ct = carbon_textures(texdir)
    cp = composite_textures(texdir)
    M = {}
    add = lambda n, *a, **k: M.__setitem__(n, make_material(bpy, n, *a, **k))
    add("M_HullDark", (0.040, 0.042, 0.047), 0.45, coat=0.25, coat_rough=0.25, tex=cp)
    add("M_ShellLight", (0.46, 0.47, 0.48), 0.38, coat=0.35, coat_rough=0.18, tex=cp)
    add("M_Carbon", (0.03, 0.03, 0.033), 0.35, coat=1.0, coat_rough=0.06, tex=ct)
    add("M_AnodDark", (0.085, 0.088, 0.095), 0.34, metal=1.0)
    add("M_Alu", (0.78, 0.78, 0.80), 0.24, metal=1.0)
    add("M_Steel", (0.60, 0.60, 0.62), 0.30, metal=1.0)
    add("M_Prop", (0.018, 0.018, 0.020), 0.36, coat=0.3, coat_rough=0.2)
    add("M_Rubber", (0.014, 0.014, 0.015), 0.88)
    add("M_Glass", (0.004, 0.005, 0.007), 0.04, coat=1.0, coat_rough=0.01, spec=1.0)
    add("M_LidarAlu", (0.50, 0.51, 0.53), 0.42, metal=1.0)
    add("M_PlasticBlack", (0.022, 0.022, 0.025), 0.55)
    add("M_Foam", (0.05, 0.05, 0.055), 0.98)
    add("M_WifiBlue", (0.015, 0.09, 0.55), 0.38, coat=0.2)
    add("M_Orange", (0.80, 0.16, 0.012), 0.42, coat=0.3)
    add("M_BandWhite", (0.62, 0.62, 0.62), 0.45, coat=0.2)
    add("M_SpeakerWhite", (0.58, 0.58, 0.56), 0.50)
    add("M_SpeakerDark", (0.018, 0.018, 0.02), 0.75)
    add("M_Battery", (0.065, 0.068, 0.074), 0.42, coat=0.1)
    add("M_BatteryTop", (0.26, 0.27, 0.28), 0.40)
    add("M_Seam", (0.006, 0.006, 0.007), 0.7)
    add("M_Cable", (0.70, 0.42, 0.02), 0.6)
    add("M_NavRed", (0.55, 0.02, 0.01), 0.15, coat=1.0, emit=(1.0, 0.04, 0.015), emit_strength=30.0)
    add("M_NavGreen", (0.02, 0.50, 0.08), 0.15, coat=1.0, emit=(0.04, 1.0, 0.15), emit_strength=30.0)
    add("M_Strobe", (0.90, 0.90, 0.90), 0.15, coat=1.0, emit=(1.0, 1.0, 1.0), emit_strength=45.0)
    add("M_Led", (0.02, 0.3, 0.05), 0.2, emit=(0.1, 1.0, 0.2), emit_strength=8.0)
    for name, a in (("M_BlurLo", 0.07), ("M_BlurHi1", 0.30), ("M_BlurHi2", 0.17), ("M_BlurHi3", 0.08)):
        add(name, (0.025, 0.025, 0.028), 0.45)                     # prop-blur discs (hidden in the asset)
        M[name].node_tree.nodes["Principled BSDF"].inputs["Alpha"].default_value = a
    return M


def build_object(bpy, part, M, coll, parent=None, sharp_deg=38.0):
    me = bpy.data.meshes.new(part.name)
    P = np.vstack(part.P)
    me.from_pydata(P.tolist(), [], part.F)
    me.polygons.foreach_set("material_index", part.M)
    if part.has_uv:
        uvl = me.uv_layers.new(name="UVMap")
        flat = np.array([c for f in part.UV for c in f], float)
        uvl.data.foreach_set("uv", flat.ravel())
    me.validate(clean_customdata=False)
    for m in part.mats:
        me.materials.append(M[m])
    me.shade_smooth()
    me.set_sharp_from_angle(angle=math.radians(sharp_deg))
    me.update()
    ob = bpy.data.objects.new(part.name, me)
    coll.objects.link(ob)
    if parent is not None:
        ob.parent = parent
    return ob


def tri_count(bpy, objs):
    n = 0
    for ob in objs:
        if ob.type == "MESH":
            n += sum(len(p.vertices) - 2 for p in ob.data.polygons)
    return n


# ----------------------------------------------------------------------------------------------- the airframe
def build_drone(bpy, M):
    coll = bpy.data.collections.new("HeavySAR")
    bpy.context.scene.collection.children.link(coll)
    parts, objs = [], []

    def emit(p, parent=None, sharp=38.0):
        ob = build_object(bpy, p, M, coll, parent, sharp)
        objs.append(ob)
        return ob

    # ---------------- fuselage: dark lower hull (streamlined pod) + light-grey top shell
    def hull_def(P):
        x, y, z = P[:, 0], P[:, 1], P[:, 2]
        nose = np.clip((x - 0.18) / 0.40, 0, 1)
        tail = np.clip((-x - 0.25) / 0.33, 0, 1)
        y = y * (1 - 0.30 * nose ** 2 - 0.18 * tail ** 2)
        lower = z < 0
        z = np.where(lower, z * (1 - 0.40 * nose ** 1.6 - 0.25 * tail ** 1.8), z * (1 - 0.15 * nose ** 2))
        z = z + 0.035 * nose ** 2 + 0.02 * tail ** 2
        return np.stack([x, y, z], 1)
    hc = (0.0, 0.0, Z0 + 0.16)
    Ph, Fh, UVh, (hr, hu) = superellipsoid(hc, 0.58, 0.30, 0.16, 0.42, 0.32, nu=96, nv=40, deform=hull_def)
    UVh = [[(u * 6.0, v * 2.0) for u, v in c] for c in UVh]          # composite breakup tiles ~0.4 m
    hull = Part("Fuselage").add(Ph, Fh, "M_HullDark", UVh)
    G, N = grid_normals(Ph, hr, hu)
    # panel seams: parting line around the hull, two meridian seams per side, belly hatch outline
    seams = Part("PanelSeams")
    mid = hr // 2
    seams.add(*ribbon(G, N, [(mid + 3, j) for j in range(hu)]), "M_Seam")
    for jc in (int(hu * 0.33), int(hu * 0.42), int(hu * 0.58), int(hu * 0.67), int(hu * 0.08), int(hu * 0.92)):
        path = [(i, jc) for i in range(5, hr - 5)]
        seams.add(*ribbon(G, N, path, closed=False), "M_Seam")
    # fasteners along the parting line
    fas = Part("Fasteners")
    for j in range(0, hu, 6):
        p, n_ = G[mid + 3 + 2, j], N[mid + 3 + 2, j]
        fas.addg(cyl(p - n_ * 0.001, p + n_ * 0.0022, 0.0042, n=10, chamfer=0.0012), "M_Steel")
    # side vents (grille insert + slats) on both sides
    vent = Part("Vents")
    for sd in (-1, 1):
        for x0 in (-0.13, 0.07):
            near = np.argmin(np.abs(Ph[:, 0] - x0) + np.abs(Ph[:, 2] - (Z0 + 0.17)) + 10 * (np.sign(Ph[:, 1]) != sd))
            ys = abs(Ph[near, 1])
            Rv = np.eye(3)
            vent.addg(rbox((x0, sd * (ys - 0.002), Z0 + 0.17), (0.15, 0.010, 0.075), r=0.006, seg=2, R=Rv), "M_PlasticBlack")
            for k in range(6):
                zz = Z0 + 0.145 + k * 0.0105
                vent.addg(rbox((x0, sd * (ys + 0.002), zz), (0.135, 0.004, 0.0032), r=0.0012, seg=1), "M_AnodDark")
            for xs in (-0.068, 0.068):
                for zs in (-0.031, 0.031):
                    vent.addg(cyl((x0 + xs, sd * (ys + 0.002), Z0 + 0.17 + zs), (x0 + xs, sd * (ys + 0.0045), Z0 + 0.17 + zs),
                                  0.0035, n=8, chamfer=0.001), "M_Steel")
    # nose sensor window with two obstacle-avoidance cameras
    nose_i = np.argmax(Ph[:, 0] - 5 * np.abs(Ph[:, 1]) - 5 * np.abs(Ph[:, 2] - (Z0 + 0.20)))
    xn = Ph[nose_i, 0]
    gi = np.unravel_index(np.argmin(np.linalg.norm(G - [xn, 0.0, Z0 + 0.19], axis=-1)), G.shape[:2])
    pn, nn_ = G[gi], N[gi]
    if nn_[0] < 0:
        nn_ = -nn_
    Rn = rot_to(nn_, (0, 0, 1))                                       # x = outward normal of the nose
    vent.addg(rbox(pn + nn_ * 0.0008 - nn_ * 0.006, (0.014, 0.17, 0.058), r=0.006, seg=3, R=Rn), "M_Glass")
    for sd in (-1, 1):                                                # stereo obstacle-avoidance cameras
        cpos = pn + nn_ * 0.0008 + Rn[:, 1] * sd * 0.055
        vent.addg(cyl(cpos - nn_ * 0.002, cpos + nn_ * 0.0012, 0.0115, n=20, chamfer=0.0015), "M_AnodDark")
        vent.addg(cyl(cpos + nn_ * 0.0010, cpos + nn_ * 0.0022, 0.0068, n=20), "M_Glass")

    def shell_def(P):
        x, y, z = P[:, 0], P[:, 1], P[:, 2]
        fr = np.clip((x - 0.05) / 0.45, 0, 1)
        z = np.where(z > 0, z * (1 - 0.45 * fr ** 1.5), z)
        y = y * (1 - 0.12 * fr ** 2)
        return np.stack([x, y, z], 1)
    sc_ = (-0.02, 0.0, Z0 + 0.36)
    Ps, Fs, UVs, (sr, su) = superellipsoid(sc_, 0.50, 0.235, 0.13, 0.45, 0.42, nu=88, nv=36, deform=shell_def)
    shell = Part("Canopy").add(Ps, Fs, "M_ShellLight", [[(u * 5.0, v * 1.5) for u, v in c] for c in UVs])
    Gs, Ns = grid_normals(Ps, sr, su)
    for jc in (int(su * 0.25), int(su * 0.75)):                      # longitudinal seams over the top
        seams.add(*ribbon(Gs, Ns, [(i, jc) for i in range(sr // 2 + 2, sr)], closed=False), "M_Seam")
    seams.add(*ribbon(Gs, Ns, [(sr // 2 + 4, j) for j in range(su)]), "M_Seam")
    for j in range(3, su, 8):
        p, n_ = Gs[sr // 2 + 6, j], Ns[sr // 2 + 6, j]
        fas.addg(cyl(p - n_ * 0.001, p + n_ * 0.0022, 0.0042, n=10, chamfer=0.0012), "M_Steel")

    def top_z(P, x, y, rad=0.03):
        sel = (np.abs(P[:, 0] - x) < rad) & (np.abs(P[:, 1] - y) < rad)
        return float(P[sel, 2].max()) if sel.any() else Z0 + 0.49

    def hull_top(x, y=0.0):
        return top_z(Ph, x, y)

    def shell_top(x, y=0.0):
        return top_z(Ps, x, y)

    # ---------------- top deck: compute box (finned), GNSS mast, strobe, radio + 2 whips, batteries
    deck = Part("ComputeBox")
    zt = shell_top(0.28) - 0.012
    deck.addg(rbox((0.28, 0, zt + 0.006), (0.20, 0.15, 0.012), r=0.004, seg=2), "M_AnodDark")
    deck.addg(rbox((0.28, 0, zt + 0.036), (0.18, 0.135, 0.05), r=0.006, seg=3), "M_PlasticBlack")
    for k in range(11):
        yy = -0.06 + k * 0.012
        deck.addg(rbox((0.28, yy, zt + 0.071), (0.16, 0.0025, 0.022), r=0.0008, seg=1), "M_AnodDark")
    for xs in (-0.085, 0.085):                                        # connectors on the rear face
        deck.addg(cyl((0.28 - 0.09, xs * 0.5, zt + 0.03), (0.28 - 0.105, xs * 0.5, zt + 0.03), 0.007, n=12,
                      chamfer=0.001), "M_Alu")
    for xs in (-0.09, 0.09):
        for ys in (-0.067, 0.067):
            deck.addg(cyl((0.28 + xs, ys, zt + 0.011), (0.28 + xs, ys, zt + 0.015), 0.004, n=8, chamfer=0.001), "M_Steel")
    gn = Part("GpsMast")
    zg = shell_top(0.12)
    gn.addg(cyl((0.12, 0, zg - 0.01), (0.12, 0, zg + 0.025), 0.022, n=20, chamfer=0.004), "M_AnodDark")
    gn.addg(cyl((0.12, 0, zg + 0.02), (0.12, 0, Z0 + 0.775), 0.011, n=16, uvs=(0.05, 0.05)), "M_Carbon")
    gn.addg(cyl((0.12, 0, Z0 + 0.770), (0.12, 0, Z0 + 0.785), 0.02, n=20, chamfer=0.003), "M_AnodDark")
    puck = Part("GpsPuck")
    puck.addg(lathe([(0, 0), (0.058, 0), (0.060, 0.004), (0.060, 0.012), (0.052, 0.024), (0.034, 0.032), (0, 0.035)],
                    40, (0.12, 0, Z0 + 0.785)), "M_ShellLight")
    puck.addg(cyl((0.12, 0, Z0 + 0.784), (0.12, 0, Z0 + 0.79), 0.0605, n=40), "M_PlasticBlack")
    zs = shell_top(0.03)
    strobe_base = Part("StrobeBase").addg(cyl((0.03, 0, zs - 0.01), (0.03, 0, zs + 0.016), 0.03, n=24, chamfer=0.004),
                                          "M_PlasticBlack")
    strobe = Part("Strobe").addg(lathe([(0, 0), (0.024, 0), (0.024, 0.006), (0.019, 0.019), (0.010, 0.026), (0, 0.028)],
                                       24, (0.03, 0, zs + 0.016)), "M_Strobe")
    radio = Part("RadioBox")
    zr = shell_top(-0.06) - 0.01
    radio.addg(rbox((-0.06, 0, zr + 0.024), (0.10, 0.12, 0.048), r=0.006, seg=3), "M_PlasticBlack")
    radio.addg(rbox((-0.06, 0, zr + 0.049), (0.085, 0.10, 0.004), r=0.0015, seg=1), "M_AnodDark")
    ants = Part("Antennas")
    for sd in (-1, 1):
        b = np.array([-0.075, sd * 0.045, zr + 0.05])
        ants.addg(cyl(b, b + [0, 0, 0.012], 0.0075, n=12, chamfer=0.001), "M_Alu")
        knee = b + [0, 0, 0.022]
        ants.addg(cyl(b + [0, 0, 0.012], knee, 0.0065, n=12, chamfer=0.002), "M_PlasticBlack")
        d = np.array([-0.30, sd * 0.32, 1.0])
        d /= np.linalg.norm(d)
        ants.addg(cyl(knee, knee + d * 0.20, 0.0062, n=12, r1=0.0042, chamfer=0.002), "M_Rubber")
    bats = Part("Batteries")
    for sd in (-1, 1):
        cx, cy = -0.265, sd * 0.088
        zb = shell_top(cx, cy) - 0.035
        bats.addg(rbox((cx, cy, zb + 0.075), (0.28, 0.165, 0.15), r=0.014, seg=3), "M_Battery")
        bats.addg(rbox((cx, cy, zb + 0.151), (0.25, 0.14, 0.006), r=0.0025, seg=2), "M_BatteryTop")
        for k in range(5):                                         # grip ribs on the outer side
            bats.addg(rbox((cx - 0.08 + 0.04 * k, cy + sd * 0.083, zb + 0.08), (0.012, 0.004, 0.09), r=0.0015, seg=1),
                      "M_Battery")
        hp = [(cx - 0.085, cy, zb + 0.152)]
        for a in np.linspace(0, math.pi, 13)[1:-1]:
            hp.append((cx - 0.085 * math.cos(a), cy, zb + 0.152 + 0.045 * math.sin(a) ** 0.6))
        hp.append((cx + 0.085, cy, zb + 0.152))
        bats.addg(tube_path(hp, 0.0085, n=12), "M_Rubber")
        for k in range(4):                                         # charge LEDs + button on the rear face
            bats.addg(rbox((cx - 0.141, cy - 0.03 + 0.016 * k, zb + 0.12), (0.003, 0.009, 0.004), r=0.001, seg=1), "M_Led")
        bats.addg(cyl((cx - 0.139, cy + 0.045, zb + 0.12), (cx - 0.144, cy + 0.045, zb + 0.12), 0.006, n=12,
                      chamfer=0.0015), "M_PlasticBlack")
    bay = Part("BatteryBay")
    zbay = shell_top(-0.265) - 0.03
    bay.addg(rbox((-0.265, 0, zbay), (0.31, 0.36, 0.03), r=0.01, seg=3), "M_AnodDark")

    # ---------------- arms: carbon tubes, root fairings, fold hinge, end clamps, motor pods, coaxial motors, props
    arms = Part("Arms")
    joints = Part("ArmJoints")
    pods = Part("MotorPods")
    stators = Part("Motors")
    mics = Part("MicCapsules")
    bands = Part("ArmBands")
    rotors = []
    for i in range(4):
        a = math.radians(45 + 90 * i)
        d = np.array([math.cos(a), math.sin(a), 0.0])
        root = d * 0.20 + [0, 0, Z0 + 0.445]
        tipc = d * R_ARM + [0, 0, Z_TIP]
        end = tipc - d * 0.050
        w = (end - root) / np.linalg.norm(end - root)
        Ra = rot_to(w)
        L = np.linalg.norm(end - root)
        arms.addg(cyl(root, end, 0.032, n=28, cap=True, uvs=(0.05, 0.05)), "M_Carbon")
        # shoulder fairing where the arm leaves the body
        joints.addg(rbox(root + w * 0.13, (0.15, 0.085, 0.08), r=0.025, seg=4, R=Ra), "M_HullDark")
        # fold hinge: two collars, knuckle under the arm, hinge pin, knurled lock sleeve
        hj = root + w * 0.33
        for s_ in (-1, 1):
            joints.addg(cyl(hj + w * s_ * 0.012, hj + w * s_ * 0.058, 0.040, n=28, chamfer=0.004), "M_AnodDark")
        side = Ra[:, 1]
        down = -Ra[:, 2]
        joints.addg(rbox(hj + down * 0.036, (0.075, 0.06, 0.032), r=0.008, seg=3, R=Ra), "M_AnodDark")
        joints.addg(cyl(hj + down * 0.04 - side * 0.034, hj + down * 0.04 + side * 0.034, 0.0085, n=14, chamfer=0.002), "M_Alu")
        knurl = [(0, -0.012)] + [(0.0435 if k % 2 == 0 else 0.0415, -0.012 + 0.024 * k / 11) for k in range(12)] + [(0, 0.012)]
        joints.addg(lathe(knurl, 36, hj, w), "M_Orange" if i in (0, 3) else "M_BandWhite")
        for s_ in (-1, 1):                                          # collar clamp screws
            p = hj + w * s_ * 0.035 + Ra[:, 2] * 0.040
            joints.addg(cyl(p, p + Ra[:, 2] * 0.004, 0.0045, n=10, chamfer=0.001), "M_Steel")
        # tip clamp into the pod + orientation band
        joints.addg(cyl(end - w * 0.07, end + w * 0.012, 0.039, n=28, chamfer=0.004), "M_AnodDark")
        for s_ in (-1, 1):
            p = end - w * 0.035 + side * s_ * 0.039
            joints.addg(cyl(p, p + side * s_ * 0.004, 0.0045, n=10, chamfer=0.001), "M_Steel")
        bands.addg(cyl(end - w * 0.15, end - w * 0.10, 0.0328, n=28, cap=False), "M_Orange" if i in (0, 3) else "M_BandWhite")
        # motor pod (vertical streamlined column carrying both motors)
        pods.addg(lathe([(0, -0.10), (0.040, -0.10), (0.054, -0.090), (0.060, -0.070), (0.060, 0.070), (0.054, 0.090),
                         (0.040, 0.10), (0, 0.10)], 36, tipc), "M_HullDark")
        pods.addg(cyl(tipc + [0, 0, -0.012], tipc + [0, 0, 0.012], 0.0608, n=36, cap=False), "M_Seam")
        # nav light on the outer face of the pod: red port (+y: arms 0, 1), green starboard (arms 2, 3)
        nl = Part(f"NavRed{i}" if i in (0, 1) else f"NavGreen{i}")
        nl.addg(lathe([(0, 0), (0.024, 0), (0.024, 0.004), (0.020, 0.014), (0.011, 0.020), (0, 0.022)], 24,
                      tipc + d * 0.055 + [0, 0, -0.03], d), "M_NavRed" if i in (0, 1) else "M_NavGreen")
        pods.addg(cyl(tipc + d * 0.050 + [0, 0, -0.03], tipc + d * 0.060 + [0, 0, -0.03], 0.028, n=24, chamfer=0.002),
                  "M_PlasticBlack")
        parts.append(nl)
        # coaxial motors: stationary stator housings with radial cooling fins (upper on top, lower inverted)
        for lvl, sg in ((1, 1), (0, -1)):
            zb0 = Z_TIP + sg * 0.095
            zb1 = Z_TIP + sg * 0.128
            c0 = np.array([tipc[0], tipc[1], zb0])
            stators.addg(cyl(c0, [tipc[0], tipc[1], zb0 + sg * 0.010], 0.066, n=40, chamfer=0.003), "M_AnodDark")
            stators.addg(cyl([tipc[0], tipc[1], zb0 + sg * 0.008], [tipc[0], tipc[1], zb1], 0.070, n=40, chamfer=0.002),
                         "M_AnodDark")
            fins = fan_boxes(c0 + [0, 0, sg * 0.010] if sg > 0 else c0 + [0, 0, -0.010 - 0.024], (0, 0, 1), 24,
                             0.068, 0.081, 0.024, 0.0028)
            stators.add(fins[0], fins[1], "M_Alu")
            for k in range(4):                                           # mount screws on the flange
                aa = math.radians(45 + 90 * k)
                p = c0 + [0.058 * math.cos(aa), 0.058 * math.sin(aa), sg * 0.010]
                stators.addg(cyl(p, p + [0, 0, sg * 0.004], 0.0042, n=10, chamfer=0.001), "M_Steel")
            rotors.append((i, lvl, np.array([tipc[0], tipc[1], Z_TIP + sg * COAX])))
        # two mic capsules on top of each arm (foam windscreens on short stalks)
        for f in (0.30, 0.62):
            p = root + (end - root) * f + Ra[:, 2] * 0.032
            mics.addg(cyl(p - Ra[:, 2] * 0.004, p + Ra[:, 2] * 0.010, 0.012, n=14, chamfer=0.002), "M_AnodDark")
            mics.addg(cyl(p + Ra[:, 2] * 0.010, p + Ra[:, 2] * 0.032, 0.0045, n=10), "M_PlasticBlack")
            ball = superellipsoid(p + Ra[:, 2] * 0.050, 0.022, 0.022, 0.026, 1.0, 1.0, nu=20, nv=12)
            mics.add(ball[0], ball[1], "M_Foam")

    # ---------------- landing gear: carbon legs + braces, alloy joints, skids with rubber feet
    gear = Part("LandingGear")
    gj = Part("GearJoints")
    yk = GEAR_W / 2
    for fb in (-1, 1):                                                    # cross-members under the belly
        x = fb * 0.27
        gear.addg(cyl((x, -0.19, Z0 - 0.022), (x, 0.19, Z0 - 0.022), 0.02, n=20, uvs=(0.05, 0.05)), "M_Carbon")
        for sd in (-1, 1):
            gj.addg(rbox((x, sd * 0.14, Z0 - 0.010), (0.075, 0.06, 0.045), r=0.008, seg=3), "M_AnodDark")
    for sd in (-1, 1):
        y = sd * yk
        legtop = {}
        for fb in (-1, 1):
            p0 = np.array([fb * 0.27, sd * 0.19, Z0 - 0.022])
            p1 = np.array([fb * 0.42, y, 0.046])
            legtop[fb] = (p0, p1)
            gear.addg(cyl(p0, p1, 0.022, n=20, uvs=(0.05, 0.05)), "M_Carbon")
            gj.addg(rbox(p0 + [0, sd * 0.005, -0.004], (0.06, 0.05, 0.05), r=0.010, seg=3), "M_AnodDark")
            gj.addg(rbox((fb * 0.42, y, 0.044), (0.075, 0.052, 0.058), r=0.012, seg=3), "M_AnodDark")
            for s_ in (-1, 1):
                gj.addg(cyl((fb * 0.42 + s_ * 0.022, y + 0.026 * sd, 0.044), (fb * 0.42 + s_ * 0.022, y + 0.031 * sd, 0.044),
                            0.0045, n=10, chamfer=0.001), "M_Steel")
        q0 = legtop[-1][0] + (legtop[-1][1] - legtop[-1][0]) * 0.45
        q1 = legtop[1][0] + (legtop[1][1] - legtop[1][0]) * 0.45
        gear.addg(cyl(q0, q1, 0.014, n=16, uvs=(0.05, 0.05)), "M_Carbon")
        for q in (q0, q1):
            gj.addg(superellipsoid(q, 0.024, 0.024, 0.024, 0.6, 0.6, nu=16, nv=10)[:2], "M_AnodDark")
        gear.addg(cyl((-GEAR_L / 2 + 0.03, y, 0.034), (GEAR_L / 2 - 0.03, y, 0.034), 0.022, n=22, uvs=(0.05, 0.05)), "M_Carbon")
        for e in (-1, 1):                                                 # rubber end caps + foot pads
            gj.addg(lathe([(0, 0), (0.0235, 0), (0.0235, 0.03), (0.018, 0.042), (0, 0.046)], 22,
                          (e * (GEAR_L / 2 - 0.05), y, 0.034), (e, 0, 0)), "M_Rubber")
        for xf_ in (-0.62, -0.20, 0.20, 0.62):
            gj.addg(rbox((xf_, y, 0.017), (0.10, 0.052, 0.034), r=0.010, seg=3), "M_Rubber")

    # ---------------- payload: nose LiDAR (OS1-128 class, 45 deg down) on its bracket
    lid = Part("Lidar")
    lp = np.array([0.665, 0.0, Z0 + 0.105])
    ax = np.array([math.sin(math.radians(45)), 0.0, -math.cos(math.radians(45))])
    lid.addg(lathe([(0, 0), (0.0415, 0), (0.0425, 0.002), (0.0425, 0.026), (0.0415, 0.027)], 48, lp, ax), "M_LidarAlu")
    lid.addg(lathe([(0.0415, 0.027), (0.0412, 0.029), (0.0412, 0.058), (0.0415, 0.060)], 48, lp, ax), "M_Glass")
    lid.addg(lathe([(0.0415, 0.060), (0.0425, 0.061), (0.0425, 0.070), (0.040, 0.0735), (0, 0.0735)], 48, lp, ax), "M_LidarAlu")
    fins = fan_boxes(lp + ax * 0.0735, ax, 18, 0.010, 0.038, 0.004, 0.0025)
    lid.add(fins[0], fins[1], "M_LidarAlu")
    lid.addg(cyl(lp - ax * 0.008, lp - ax * 0.0005, 0.032, n=32, chamfer=0.002), "M_AnodDark")
    Rl = rot_to(np.cross(ax, [0, 1, 0]) * -1, ax)                       # plate frame: z = sensor axis
    br = Part("LidarBracket")
    plate_c = lp - ax * 0.012
    br.addg(rbox(plate_c, (0.115, 0.10, 0.008), r=0.004, seg=2, R=rot_to((ax[2], 0, -ax[0]), ax)), "M_AnodDark")
    xs_n = Ph[nose_i, 0]
    for sd in (-1, 1):
        a0 = np.array([xs_n - 0.10, sd * 0.042, Z0 + 0.075])
        a1 = plate_c + np.array([ax[2], 0, -ax[0]]) * 0.035 + [0, sd * 0.042, 0]
        mid_ = (a0 + a1) / 2
        br.addg(rbox(mid_, (np.linalg.norm(a1 - a0) + 0.02, 0.007, 0.032), r=0.003, seg=2, R=rot_to(a1 - a0, (0, 0, 1))), "M_AnodDark")
        for t_ in (0.2, 0.8):
            q = a0 + (a1 - a0) * t_ + [0, sd * 0.004, 0]
            br.addg(cyl(q, q + [0, sd * 0.004, 0], 0.0045, n=10, chamfer=0.001), "M_Steel")
    br.addg(rbox((xs_n - 0.10, 0, Z0 + 0.075), (0.05, 0.10, 0.03), r=0.006, seg=2), "M_AnodDark")
    for k in range(4):
        aa = math.radians(45 + 90 * k)
        p = plate_c + rot_to((ax[2], 0, -ax[0]), ax) @ np.array([0.045 * math.cos(aa) * 1.1, 0.04 * math.sin(aa), 0.0])
        br.addg(cyl(p - ax * 0.0045, p - ax * 0.008, 0.0042, n=10, chamfer=0.001), "M_Steel")

    # ---------------- winch (drum + frame + gearmotor + line) under the belly
    wn = Part("WinchFrame")
    wx = -0.08
    wn.addg(rbox((wx, 0, Z0 - 0.006 + 0.004), (0.24, 0.30, 0.012), r=0.004, seg=2), "M_AnodDark")
    for sd in (-1, 1):
        wn.addg(rbox((wx, sd * 0.128, Z0 - 0.075), (0.20, 0.012, 0.14), r=0.012, seg=3), "M_AnodDark")
        for xs in (-0.075, 0.075):
            wn.addg(cyl((wx + xs, sd * 0.134, Z0 - 0.02), (wx + xs, sd * 0.139, Z0 - 0.02), 0.005, n=10, chamfer=0.001), "M_Steel")
    wd = Part("WinchDrum")
    wd.addg(lathe([(0, -0.12), (0.075, -0.12), (0.076, -0.112), (0.058, -0.106), (0.058, 0.106), (0.076, 0.112),
                   (0.075, 0.12), (0, 0.12)], 40, (wx, 0, Z0 - 0.085), (0, 1, 0)), "M_Alu")
    for k in range(12):                                                    # wraps of line
        yy = -0.098 + k * 0.0178
        wd.addg(lathe([(0.058, -0.008), (0.064, -0.006), (0.065, 0.0), (0.064, 0.006), (0.058, 0.008)], 36,
                      (wx, yy, Z0 - 0.085), (0, 1, 0)), "M_Cable")
    wd.addg(cyl((wx, 0.134, Z0 - 0.085), (wx, 0.215, Z0 - 0.085), 0.036, n=28, chamfer=0.004), "M_AnodDark")
    wd.addg(cyl((wx, 0.215, Z0 - 0.085), (wx, 0.225, Z0 - 0.085), 0.03, n=28, chamfer=0.003), "M_PlasticBlack")
    wd.addg(cyl((wx + 0.02, -0.06, Z0 - 0.165), (wx + 0.02, 0.06, Z0 - 0.165), 0.011, n=16, chamfer=0.002), "M_Steel")
    wd.addg(tube_path([(wx + 0.02, 0.0, Z0 - 0.15), (wx + 0.02, 0.0, Z0 - 0.21)], 0.0032, n=8), "M_Cable")
    hook = [(wx + 0.02, 0, Z0 - 0.21)] + [(wx + 0.02 + 0.018 * math.sin(t), 0, Z0 - 0.228 - 0.018 * math.cos(t))
                                          for t in np.linspace(-0.3, 4.2, 14)]
    wd.addg(tube_path(hook, 0.0045, n=10), "M_Steel")

    # ---------------- loudspeaker horn (rear underside, pointing down)
    sp = Part("SpeakerHorn")
    sx = -0.42
    zt_ = Z0 + 0.02
    sp.addg(rbox((sx, 0, Z0 - 0.002), (0.11, 0.13, 0.02), r=0.005, seg=2), "M_AnodDark")
    sp.addg(lathe([(0, zt_ - Z0 + 0.0), (0.052, -0.012), (0.054, -0.016), (0.054, -0.062), (0.050, -0.066)], 36,
                  (sx, 0, Z0), (0, 0, 1)), "M_SpeakerWhite")
    prof_out = [(0.050 + 0.062 * (k / 10) ** 2.2, -0.066 - 0.10 * k / 10) for k in range(11)]
    lip = [(prof_out[-1][0] + 0.004, prof_out[-1][1] - 0.002), (prof_out[-1][0] + 0.002, prof_out[-1][1] - 0.008),
           (prof_out[-1][0] - 0.006, prof_out[-1][1] - 0.008)]
    sp.addg(lathe(prof_out + lip, 40, (sx, 0, Z0)), "M_SpeakerWhite")
    prof_in = [(p[0] - 0.007, p[1] - 0.006) for p in prof_out[::-1]] + [(0.0, -0.075)]
    sp.addg(lathe([lip[-1]] + prof_in, 40, (sx, 0, Z0)), "M_SpeakerDark")

    # ---------------- blue rescue Wi-Fi antenna at the rear
    wf = Part("WifiAnt")
    xw = -0.50
    zw = hull_top(xw)
    wf.addg(cyl((xw, 0, zw - 0.015), (xw, 0, zw + 0.02), 0.020, n=20, chamfer=0.004), "M_PlasticBlack")
    wf.addg(cyl((xw, 0, zw + 0.02), (xw, 0, zw + 0.03), 0.0095, n=14, chamfer=0.001), "M_Alu")
    wf.addg(lathe([(0, 0), (0.012, 0), (0.0125, 0.01), (0.0125, 0.20), (0.010, 0.212), (0.005, 0.218), (0, 0.219)], 18,
                  (xw, 0, zw + 0.03)), "M_WifiBlue")

    # ---------------- emit all static parts
    for p in [hull, seams, fas, vent, shell, deck, gn, puck, strobe_base, radio, ants, bats, bay, arms, joints, pods,
              stators, mics, bands, gear, gj, lid, br, wn, wd, sp, wf] + parts:
        emit(p)
    strobe_ob = emit(strobe, sharp=60.0)

    # ---------------- rotors: Xform (Empty) at the hub, prop + spinning bell under it
    rotor_objs = []
    for i, lvl, hub in rotors:
        sg = spin_sign(i, lvl)
        em = bpy.data.objects.new(f"Rotor{i}_{lvl}", None)
        em.empty_display_type = "CIRCLE"
        em.empty_display_size = PROP_R
        em.location = tuple(map(float, hub))
        em["spinSign"] = int(sg)
        coll.objects.link(em)
        bl = Part(f"Blades{i}_{lvl}")
        b0 = blade(PROP_R, sg)
        bl.add(b0[0], b0[1], "M_Prop")
        Rz = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1.0]])
        b1 = xf(b0, Rz)
        bl.add(b1[0], b1[1], "M_Prop")
        objs.append(build_object(bpy, bl, M, coll, parent=em, sharp_deg=40.0))
        blo = Part(f"Blur_lo{i}_{lvl}")
        blo.addg(lathe([(0.055, -0.001), (PROP_R, -0.001), (PROP_R, 0.001), (0.055, 0.001), (0.055, -0.001)], 64), "M_BlurLo")
        bhi = Part(f"Blur_hi{i}_{lvl}")
        for (r0_, r1_), mname in (((0.055, 0.26), "M_BlurHi1"), ((0.26, 0.50), "M_BlurHi2"), ((0.50, PROP_R), "M_BlurHi3")):
            bhi.addg(lathe([(r0_, -0.001), (r1_, -0.001), (r1_, 0.001), (r0_, 0.001), (r0_, -0.001)], 64), mname)
        for p_ in (blo, bhi):
            objs.append(build_object(bpy, p_, M, coll, parent=em, sharp_deg=80.0))
        pr = Part(f"Hub{i}_{lvl}")
        up = 1 if lvl == 1 else -1                       # bell toward the pod: below the upper prop, above the lower
        # hub clamp plates, folding pivot bolts, spinner
        pr.addg(cyl((0, 0, -0.016), (0, 0, -0.009), 0.050, n=36, chamfer=0.002), "M_AnodDark")
        pr.addg(cyl((0, 0, 0.009), (0, 0, 0.016), 0.050, n=36, chamfer=0.002), "M_AnodDark")
        for s_ in (-1, 1):
            pr.addg(cyl((s_ * 0.038, 0, -0.019), (s_ * 0.038, 0, 0.020), 0.0065, n=12, chamfer=0.0015), "M_Steel")
        tipz = 0.016 if up > 0 else -0.016
        pr.addg(lathe([(0, 0), (0.024, 0), (0.022, 0.008), (0.014, 0.016), (0, 0.019)], 24, (0, 0, tipz), (0, 0, up)),
                "M_AnodDark")
        # bell (rotating motor housing) with side cooling ribs and a vented top
        bz0, bz1 = -0.016 * up, -0.047 * up
        pr.addg(lathe([(0.0, 0.0), (0.058, 0.0), (0.078, -0.002), (0.083, -0.006), (0.083, -0.031)], 48,
                      (0, 0, bz0), (0, 0, up)), "M_AnodDark")
        ribs = fan_boxes((0, 0, bz0), (0, 0, up), 30, 0.081, 0.0865, 0.020, 0.0032, z0=-0.029, phase=0.05)
        P_r = np.asarray(ribs[0])
        pr.add(P_r, ribs[1], "M_Alu")
        vents = fan_boxes((0, 0, bz0), (0, 0, up), 8, 0.030, 0.066, 0.0015, 0.010, z0=0.0, phase=0.2)
        pr.add(vents[0], vents[1], "M_Seam")
        pr.addg(cyl((0, 0, bz1 + up * 0.0005), (0, 0, bz1 - up * 0.002), 0.0705, n=40, cap=False), "M_Alu")
        ob = build_object(bpy, pr, M, coll, parent=em, sharp_deg=40.0)
        rotor_objs.append(em)
        objs.append(ob)
    return coll, objs, rotor_objs, strobe_ob


# ----------------------------------------------------------------------------------------------- export
def export_usd(bpy, coll, path):
    import bpy as _b
    for ob in _b.context.scene.objects:
        ob.select_set(ob.name in coll.all_objects)
    kw = dict(filepath=path, selected_objects_only=True, export_materials=True, generate_preview_surface=True,
              export_textures_mode="KEEP", relative_paths=True, root_prim_path="/HeavySAR", export_subdivision="IGNORE",
              convert_orientation=False, export_custom_properties=True, evaluation_mode="RENDER",
              convert_world_material=False, export_lights=False, export_cameras=False, export_animation=False,
              export_uvmaps=True, rename_uvmaps=True, export_normals=True, xform_op_mode="TRS",
              meters_per_unit=1.0, convert_scene_units="METERS", use_instancing=False, triangulate_meshes=False,
              author_blender_name=False)
    props = {p.identifier for p in bpy.ops.wm.usd_export.get_rna_type().properties}
    kw = {k: v for k, v in kw.items() if k in props}
    bpy.ops.wm.usd_export(**kw)
    return path


def postprocess(path):
    """pxr fix-up + validation (runs without bpy): default prim, Z up, metres, emissive nav/strobe materials,
    rotor pivots; prints the prim tree, extents and triangle count."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade
    st = Usd.Stage.Open(path)
    root = st.GetPrimAtPath("/HeavySAR")
    if root and st.GetDefaultPrim() != root:
        st.SetDefaultPrim(root)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    Usd.ModelAPI(root).SetKind("component")
    import re
    layer = st.GetRootLayer()
    renames = []
    for prim in st.Traverse():
        m = re.fullmatch(r"(Blades|Hub|Blur_lo|Blur_hi)\d+_[01]", prim.GetName())
        if m and re.fullmatch(r"Rotor\d+_[01]", prim.GetParent().GetName()):
            renames.append((prim.GetPath(), prim.GetPath().GetParentPath().AppendChild(m.group(1))))
    if renames:
        edit = Sdf.BatchNamespaceEdit()
        for a, b in renames:
            edit.Add(a, b)
        if not layer.Apply(edit):
            print("[postprocess] WARNING: rotor child rename failed")
    for prim in st.Traverse():
        if prim.GetName() in ("Blur_lo", "Blur_hi"):
            UsdGeom.Imageable(prim).MakeInvisible()
    for prim in st.Traverse():
        if prim.IsA(UsdShade.Material):
            op_ = OPACITY_USD.get(prim.GetName())
            if op_ is not None:
                for ch in Usd.PrimRange(prim):
                    if ch.IsA(UsdShade.Shader) and UsdShade.Shader(ch).GetIdAttr().Get() == "UsdPreviewSurface":
                        UsdShade.Shader(ch).CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(op_))
            em = EMISSIVE_USD.get(prim.GetName())
            if em:
                for ch in Usd.PrimRange(prim):
                    if ch.IsA(UsdShade.Shader) and UsdShade.Shader(ch).GetIdAttr().Get() == "UsdPreviewSurface":
                        sh = UsdShade.Shader(ch)
                        inp = sh.GetInput("emissiveColor") or sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f)
                        src = inp.GetConnectedSources()[0] if inp.HasConnectedSource() else None
                        if src:
                            inp.DisconnectSource()
                        inp.Set(Gf.Vec3f(*em))
    st.GetRootLayer().Save()
    validate(path)


def validate(path):
    from pxr import Usd, UsdGeom, UsdShade
    st = Usd.Stage.Open(path)
    print(f"[validate] {path}")
    print(f"  upAxis={UsdGeom.GetStageUpAxis(st)} metersPerUnit={UsdGeom.GetStageMetersPerUnit(st)} "
          f"defaultPrim={st.GetDefaultPrim().GetPath()}")
    tris, meshes = 0, 0
    for prim in st.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            c = UsdGeom.Mesh(prim).GetFaceVertexCountsAttr().Get()
            tris += int(sum(max(0, k - 2) for k in c))
            meshes += 1
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(st.GetDefaultPrim()).ComputeAlignedRange()
    print(f"  meshes={meshes} triangles={tris}")
    print(f"  extent min={tuple(round(v, 3) for v in bb.GetMin())} max={tuple(round(v, 3) for v in bb.GetMax())}")
    rot = [p for p in st.Traverse() if p.GetName().startswith("Rotor")]
    for p in rot:
        x = UsdGeom.Xformable(p)
        m = x.GetLocalTransformation()
        sgn = p.GetAttribute("userProperties:spinSign").Get() if p.HasAttribute("userProperties:spinSign") else None
        print(f"  {p.GetPath()}  ops={[o.GetOpName() for o in x.GetOrderedXformOps()]} hub={tuple(round(v, 3) for v in m.ExtractTranslation())} spinSign={sgn}")
    s = [p for p in st.Traverse() if p.GetName() == "Strobe"]
    print(f"  Strobe: {[str(p.GetPath()) for p in s]}")
    mats = [p for p in st.Traverse() if p.IsA(UsdShade.Material)]
    for m in mats:
        sh = [UsdShade.Shader(c) for c in Usd.PrimRange(m) if c.IsA(UsdShade.Shader)]
        ps = [s_ for s_ in sh if s_.GetIdAttr().Get() == "UsdPreviewSurface"]
        tex = [s_.GetInput("file").Get() for s_ in sh if s_.GetIdAttr().Get() == "UsdUVTexture"]
        em = ps[0].GetInput("emissiveColor").Get() if ps and ps[0].GetInput("emissiveColor") else None
        print(f"  material {m.GetName():16s} preview={bool(ps)} emissive={em} tex={[str(t.path) for t in tex if t]}")
    print("  prim tree:")
    for prim in st.Traverse():
        depth = len(prim.GetPath().pathString.split("/")) - 2
        if depth <= 2 and not prim.IsA(UsdShade.Shader):
            print("   " + "  " * depth + f"{prim.GetName()} ({prim.GetTypeName()})")
    return tris


# ----------------------------------------------------------------------------------------------- previews (Cycles CPU)
def look_at(cam, target):
    from mathutils import Vector
    d = Vector(target) - cam.location
    cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()


def setup_render_scene(bpy, samples=48, res=(1280, 720)):
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = samples
    sc.cycles.use_adaptive_sampling = True
    sc.cycles.adaptive_threshold = 0.02
    sc.cycles.use_denoising = True
    try:
        sc.cycles.denoiser = "OPENIMAGEDENOISE"
    except Exception:
        pass
    sc.cycles.max_bounces = 6
    sc.cycles.diffuse_bounces = 3
    sc.cycles.glossy_bounces = 3
    sc.cycles.transmission_bounces = 2
    sc.cycles.caustics_reflective = False
    sc.cycles.caustics_refractive = False
    sc.render.resolution_x, sc.render.resolution_y = res
    sc.render.resolution_percentage = 100
    sc.render.image_settings.file_format = "PNG"
    sc.view_settings.view_transform = "AgX"
    sc.view_settings.exposure = -0.3
    try:
        sc.view_settings.look = "AgX - Medium High Contrast"
    except Exception:
        pass
    # sky + sun
    world = bpy.data.worlds.new("Sky")
    sc.world = world
    try:
        world.use_nodes = True
    except Exception:
        pass
    nt = world.node_tree
    bg = nt.nodes.get("Background") or nt.nodes.new("ShaderNodeBackground")
    sky = nt.nodes.new("ShaderNodeTexSky")
    for t in ("MULTIPLE_SCATTERING", "SINGLE_SCATTERING", "NISHITA", "HOSEK_WILKIE"):
        try:
            sky.sky_type = t
            break
        except Exception:
            continue
    elev, azim = math.radians(38), math.radians(-35)
    for k, v in (("sun_disc", False), ("sun_elevation", elev), ("sun_rotation", azim), ("altitude", 50.0),
                 ("air_density", 1.0), ("aerosol_density", 1.6), ("dust_density", 1.6), ("ozone_density", 1.0)):
        try:
            setattr(sky, k, v)
        except Exception:
            pass
    nt.links.new(sky.outputs["Color"], bg.inputs["Color"])
    bg.inputs["Strength"].default_value = 0.12
    out = nt.nodes.get("World Output") or nt.nodes.new("ShaderNodeOutputWorld")
    nt.links.new(bg.outputs["Background"], out.inputs["Surface"])
    sun_d = bpy.data.lights.new("Sun", "SUN")
    sun_d.energy = 5.4
    sun_d.angle = math.radians(0.6)
    sun_d.color = (1.0, 0.96, 0.90)
    sun = bpy.data.objects.new("Sun", sun_d)
    sc.collection.objects.link(sun)
    # the sun shines FROM azimuth azim, elevation elev
    from mathutils import Vector
    to_sun = Vector((math.cos(elev) * math.cos(azim), math.cos(elev) * math.sin(azim), math.sin(elev)))
    sun.rotation_euler = (-to_sun).to_track_quat("-Z", "Y").to_euler()
    # concrete-ish ground (render only): albedo/roughness breakup, joints, cracks, stains
    me = bpy.data.meshes.new("Ground")
    s = 400.0
    me.from_pydata([(-s, -s, 0), (s, -s, 0), (s, s, 0), (-s, s, 0)], [], [(0, 1, 2, 3)])
    g = bpy.data.objects.new("Ground", me)
    sc.collection.objects.link(g)
    mat = bpy.data.materials.new("Concrete")
    try:
        mat.use_nodes = True
    except Exception:
        pass
    t = mat.node_tree
    bs = t.nodes["Principled BSDF"]
    L = t.links
    tc = t.nodes.new("ShaderNodeTexCoord")

    def node(kind, **inputs):
        n = t.nodes.new(kind)
        for k, v in inputs.items():
            if k.startswith("_"):
                setattr(n, k[1:], v)
            else:
                n.inputs[k].default_value = v
        return n

    def math_(op, a_, b_=None):
        n = t.nodes.new("ShaderNodeMath")
        n.operation = op
        for k, v in enumerate((a_, b_)):
            if v is None:
                continue
            if isinstance(v, (int, float)):
                n.inputs[k].default_value = v
            else:
                L.new(v, n.inputs[k])
        return n.outputs[0]
    sep = t.nodes.new("ShaderNodeSeparateXYZ")
    L.new(tc.outputs["Object"], sep.inputs[0])
    joints = None
    for axis, pitch in ((0, 4.5), (1, 3.0)):                         # saw-cut slab joints
        d = math_("ABSOLUTE", math_("SUBTRACT", math_("FRACT", math_("ADD", math_("MULTIPLY", sep.outputs[axis], 1 / pitch), 0.5)), 0.5))
        d = math_("MULTIPLY", d, pitch)
        mr = node("ShaderNodeMapRange", **{"From Min": 0.002, "From Max": 0.006, "To Min": 0.35, "To Max": 1.0})
        L.new(d, mr.inputs["Value"])
        joints = mr.outputs["Result"] if joints is None else math_("MULTIPLY", joints, mr.outputs["Result"])
    big = node("ShaderNodeTexNoise", Scale=0.08, Detail=6.0, Roughness=0.55)
    mid = node("ShaderNodeTexNoise", Scale=0.9, Detail=8.0, Roughness=0.6)
    fine = node("ShaderNodeTexNoise", Scale=60.0, Detail=4.0, Roughness=0.7)
    agg = node("ShaderNodeTexVoronoi", Scale=180.0)
    for n in (big, mid, fine, agg):
        L.new(tc.outputs["Object"], n.inputs["Vector"])
    ramp = t.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].position = 0.38
    ramp.color_ramp.elements[0].color = (0.10, 0.092, 0.080, 1)
    ramp.color_ramp.elements[1].position = 0.62
    ramp.color_ramp.elements[1].color = (0.26, 0.245, 0.22, 1)
    v = math_("ADD", math_("MULTIPLY", big.outputs["Fac"], 0.55), math_("MULTIPLY", mid.outputs["Fac"], 0.30))
    v = math_("ADD", v, math_("MULTIPLY", fine.outputs["Fac"], 0.15))
    v = math_("ADD", v, math_("MULTIPLY", math_("LESS_THAN", agg.outputs["Distance"], 0.12), 0.06))
    L.new(v, ramp.inputs["Fac"])
    col = t.nodes.new("ShaderNodeMix")
    col.data_type = "RGBA"
    col.blend_type = "MULTIPLY"
    col.inputs["Factor"].default_value = 1.0
    L.new(ramp.outputs["Color"], col.inputs[6])
    L.new(joints, col.inputs[7])
    stain_n = node("ShaderNodeTexNoise", Scale=0.35, Detail=5.0, Roughness=0.65)
    L.new(tc.outputs["Object"], stain_n.inputs["Vector"])
    stain = node("ShaderNodeMapRange", **{"From Min": 0.55, "From Max": 0.68, "To Min": 1.0, "To Max": 0.55})
    L.new(stain_n.outputs["Fac"], stain.inputs["Value"])
    col2 = t.nodes.new("ShaderNodeMix")
    col2.data_type = "RGBA"
    col2.blend_type = "MULTIPLY"
    col2.inputs["Factor"].default_value = 1.0
    L.new(col.outputs[2], col2.inputs[6])
    L.new(stain.outputs["Result"], col2.inputs[7])
    L.new(col2.outputs[2], bs.inputs["Base Color"])
    rr = node("ShaderNodeMapRange", **{"To Min": 0.80, "To Max": 0.95})
    L.new(mid.outputs["Fac"], rr.inputs["Value"])
    L.new(rr.outputs["Result"], bs.inputs["Roughness"])
    bump = node("ShaderNodeBump", Strength=0.35, Distance=0.003)
    L.new(math_("ADD", fine.outputs["Fac"], math_("MULTIPLY", joints, 0.6)), bump.inputs["Height"])
    L.new(bump.outputs["Normal"], bs.inputs["Normal"])
    g.data.materials.append(mat)
    cam_d = bpy.data.cameras.new("Cam")
    cam = bpy.data.objects.new("Cam", cam_d)
    sc.collection.objects.link(cam)
    sc.camera = cam
    return cam


def render_views(bpy, outdir, views, samples, res):
    cam = setup_render_scene(bpy, samples, res)
    sc = bpy.context.scene
    for ob in sc.objects:                                   # prop-blur discs are for the sim's rpm states only
        if ob.name.startswith("Blur_"):
            ob.hide_render = True
    os.makedirs(outdir, exist_ok=True)
    shots = {
        "front34": dict(loc=(4.3, -3.1, 1.85), target=(0.0, 0, 0.78), lens=45, dof=None),
        "payload": dict(loc=(1.85, -1.15, 0.50), target=(0.32, 0.0, 0.60), lens=42, dof=None),
        "far30m": dict(loc=(25.0, -16.0, 3.2), target=(0, 0, 0.7), lens=70, dof=None),
        "top": dict(loc=(2.6, -2.0, 3.9), target=(0, 0, 0.9), lens=35, dof=None),
    }
    out = []
    for v in views:
        s = shots[v]
        cam.location = s["loc"]
        cam.data.lens = s["lens"]
        cam.data.clip_start = 0.05
        look_at(cam, s["target"])
        sc.render.filepath = os.path.join(outdir, f"drone_{v}.png")
        bpy.ops.render.render(write_still=True)
        out.append(sc.render.filepath)
        print(f"[render] {sc.render.filepath}", flush=True)
    return out


# ----------------------------------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", default="", help="output folder for heavy_sar.usdc and textures/")
    ap.add_argument("--name", default="heavy_sar.usdc")
    ap.add_argument("--render", default="", help="folder for Cycles CPU previews")
    ap.add_argument("--views", default="front34,payload,far30m")
    ap.add_argument("--samples", type=int, default=48)
    ap.add_argument("--res", default="1280x720")
    ap.add_argument("--blend", default="", help="also save the .blend here")
    ap.add_argument("--postprocess", default="", help="pxr fix-up + validation of an exported file (no bpy)")
    ap.add_argument("--pxr-python", default=os.environ.get("LARPVERSE_PXR_PYTHON", sys.executable),
                    help="python with usd-core for the fix-up step (run as a subprocess)")
    a = ap.parse_args(argv)
    if a.postprocess:
        postprocess(a.postprocess)
        return 0
    bpy = bpy_setup()
    out = os.path.abspath(a.out or ".")
    texdir = os.path.join(out, "textures")
    M = make_materials(bpy, texdir)
    coll, objs, rotors, strobe = build_drone(bpy, M)
    print(f"[blender_drone] objects={len(objs)} rotors={len(rotors)} triangles={tri_count(bpy, objs)}", flush=True)
    if a.out:
        usd = os.path.join(out, a.name)
        export_usd(bpy, coll, usd)
        print(f"[blender_drone] exported {usd} ({os.path.getsize(usd) / 1e6:.2f} MB)", flush=True)
        r = subprocess.run([a.pxr_python, os.path.abspath(__file__), "--postprocess", usd])
        if r.returncode:
            print(f"[blender_drone] WARNING: pxr fix-up failed (rc {r.returncode}); run with a usd-core python: "
                  f"python {__file__} --postprocess {usd}")
    if a.blend:
        os.makedirs(os.path.dirname(os.path.abspath(a.blend)), exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(a.blend))
    if a.render:
        w, h = map(int, a.res.lower().split("x"))
        render_views(bpy, os.path.abspath(a.render), [v for v in a.views.split(",") if v], a.samples, (w, h))
    return 0


if __name__ == "__main__":
    sys.exit(main())
