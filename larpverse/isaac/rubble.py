"""Irregular broken-concrete geometry for the physics earthquake (pure numpy; USD writing is in quake.py).

Every function returns (points (N,3) float32, faces (M,3) int32), centred on the piece's own origin, with
faces wound counter-clockwise seen from outside. Sizes in metres; z is up.
  rock()       convex chunk with fractured flat faces (collision: its own convex hull, exact)
  block()      a chipped, slightly skewed block (wall tops, the packed upper-floor layer; convex hull)
  slab()       a broken floor-slab piece: jagged outline, slightly uneven top (collision: convex hull)
  rebar()      bent steel bars sticking out of a slab or wall edge (visual only)
  wall_run()   a damaged wall segment with a ragged, notched top (static triangle-mesh collider)
  heap()       a rubble mound: height field over the rubble cells, in world x/y (static triangle-mesh collider)
  pebbles()    thousands of small stones merged into one mesh (visual only)
"""
from __future__ import annotations
import numpy as np
from scipy.ndimage import gaussian_filter, binary_dilation, zoom
from scipy.spatial import ConvexHull


def _orient_out(P, F):
    """Make every triangle face away from the centroid (convex shapes)."""
    c = P.mean(0)
    a, b, d = P[F[:, 0]], P[F[:, 1]], P[F[:, 2]]
    n = np.cross(b - a, d - a)
    flip = ((a - c) * n).sum(1) < 0
    F = F.copy()
    F[flip] = F[flip][:, [0, 2, 1]]
    return F


def _hull(P):
    h = ConvexHull(P)
    idx = np.unique(h.simplices)
    remap = -np.ones(len(P), int)
    remap[idx] = np.arange(len(idx))
    Q = P[idx]
    F = remap[h.simplices]
    Q = Q - Q.mean(0)
    return Q.astype(np.float32), _orient_out(Q, F).astype(np.int32)


def rock(rng, sx, sy, sz, n=18):
    """Convex concrete chunk ~ (sx, sy, sz), with a few flat fracture planes."""
    d = rng.normal(size=(n, 3))
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    P = d * 0.5 * rng.uniform(0.75, 1.1, (n, 1)) * np.array([sx, sy, sz])
    for _ in range(rng.integers(2, 5)):                     # fracture planes: clip points to a plane
        nrm = rng.normal(size=3); nrm /= np.linalg.norm(nrm)
        off = rng.uniform(0.25, 0.42) * np.array([sx, sy, sz]) @ np.abs(nrm)
        s = P @ nrm
        over = s > off
        P[over] -= np.outer(s[over] - off, nrm)
    P += rng.normal(0, 0.004, P.shape)
    return _hull(P)


def block(rng, sx, sy, sz, chip=0.18):
    """Box-like concrete piece with chipped corners and slightly uneven faces (convex)."""
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    P = []
    for cx in (-1, 1):
        for cy in (-1, 1):
            for cz in (-1, 1):
                c = np.array([cx * hx, cy * hy, cz * hz])
                if rng.random() < 0.55:                     # chip this corner: replace it by 3 points
                    f = rng.uniform(0.08, chip, 3) * np.array([sx, sy, sz])
                    P += [c - [cx * f[0], 0, 0], c - [0, cy * f[1], 0], c - [0, 0, cz * f[2]]]
                else:
                    P.append(c)
    P = np.array(P, float)
    P += rng.normal(0, 0.012, P.shape) * np.array([sx, sy, sz]) / max(sx, sy, sz) ** 0.5
    return _hull(P)


def _poly_mesh(outline, z_top, z_bot):
    """Prism over a star-shaped outline (K,2): fan-triangulated top/bottom + side quads."""
    K = len(outline)
    c = outline.mean(0)
    top = np.c_[outline, z_top]
    bot = np.c_[outline, np.full(K, z_bot)]
    P = np.r_[top, bot, [[c[0], c[1], np.mean(z_top)]], [[c[0], c[1], z_bot]]]
    ct, cb = 2 * K, 2 * K + 1
    F = []
    for i in range(K):
        j = (i + 1) % K
        F += [[ct, i, j], [cb, K + j, K + i], [i, K + i, K + j], [i, K + j, j]]
    F = np.array(F)
    # wind outward: top faces +z, bottom -z, sides away from the outline centre
    a, b, d = P[F[:, 0]], P[F[:, 1]], P[F[:, 2]]
    n = np.cross(b - a, d - a)
    ref = (a + b + d) / 3 - np.r_[c, np.mean(z_top) / 2 + z_bot / 2]
    flip = (n * ref).sum(1) < 0
    F[flip] = F[flip][:, [0, 2, 1]]
    return P.astype(np.float32), F.astype(np.int32)


def slab(rng, sx, sy, T=0.18, step=0.12):
    """Broken slab piece: rectangle outline with chipped, notched edges and an uneven top."""
    pts = []
    corners = [(-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2)]
    for k in range(4):
        (x0, y0), (x1, y1) = corners[k], corners[(k + 1) % 4]
        L = np.hypot(x1 - x0, y1 - y0)
        m = max(2, int(L / step))
        nx, ny = (y1 - y0) / L, -(x1 - x0) / L            # outward normal of this edge (CCW outline)
        for i in range(m):
            f = i / m
            depth = rng.uniform(0, 0.035) + (rng.uniform(0.05, 0.14) if rng.random() < 0.08 else 0)
            depth = min(depth, 0.35 * min(sx, sy))
            pts.append((x0 + f * (x1 - x0) - nx * depth, y0 + f * (y1 - y0) - ny * depth))
    outline = np.array(pts)
    rng.normal(0, 0.012, len(outline))            # drawn and dropped, so the rest of the layout stays as tested
    z_top = np.full(len(outline), T / 2)          # flat faces: noise on a fan-triangulated top reads as creases
    return _poly_mesh(outline, z_top, -T / 2)


def _bar(p0, p1, r=0.008, sides=6):
    d = p1 - p0
    L = np.linalg.norm(d)
    if L < 1e-6:
        return np.zeros((0, 3)), np.zeros((0, 3), int)
    d /= L
    a = np.cross(d, [0, 0, 1.0]) if abs(d[2]) < 0.9 else np.cross(d, [1.0, 0, 0])
    a /= np.linalg.norm(a); b = np.cross(d, a)
    ang = np.linspace(0, 2 * np.pi, sides, endpoint=False)
    ring = np.outer(np.cos(ang), a) * r + np.outer(np.sin(ang), b) * r
    P = np.r_[p0 + ring, p1 + ring]
    F = []
    for i in range(sides):
        j = (i + 1) % sides
        F += [[i, j, sides + j], [i, sides + j, sides + i]]
    return P, np.array(F)


def merge(parts):
    Ps, Fs, o = [], [], 0
    for P, F in parts:
        if len(P) == 0:
            continue
        Ps.append(P); Fs.append(np.asarray(F) + o); o += len(P)
    if not Ps:
        return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.int32)
    return np.concatenate(Ps).astype(np.float32), np.concatenate(Fs).astype(np.int32)


def rebar(rng, sx, sy, z=0.0, n=None):
    """Bent bars sticking out of the edges of a sx x sy piece (local frame)."""
    n = n if n is not None else int(rng.integers(2, 6))
    parts = []
    for _ in range(n):
        side = rng.integers(4)
        t = rng.uniform(-0.4, 0.4)
        if side == 0:
            p0, out = np.array([t * sx, -sy / 2, z]), np.array([0, -1.0, 0])
        elif side == 1:
            p0, out = np.array([sx / 2, t * sy, z]), np.array([1.0, 0, 0])
        elif side == 2:
            p0, out = np.array([t * sx, sy / 2, z]), np.array([0, 1.0, 0])
        else:
            p0, out = np.array([-sx / 2, t * sy, z]), np.array([-1.0, 0, 0])
        L = rng.uniform(0.12, 0.4)
        mid = p0 + out * L * 0.5 + rng.normal(0, 0.03, 3)
        end = mid + (out + rng.normal(0, 0.5, 3)) * L * 0.5        # bent once
        parts += [_bar(p0 - out * 0.05, mid), _bar(mid, end)]
    return merge(parts)


def wall_run(rng, length, thick, top_cells, step=0.08, notch=0.22):
    """Damaged wall segment along local x (centred), ragged top following `top_cells` (one height per 0.4 m
    cell, interpolated) plus noise and shallow V-notches. Returns ((P, F), rebar_mesh, top_profile, xs)."""
    n = len(top_cells)
    xs = np.linspace(-length / 2, length / 2, max(3, int(length / step) + 1))
    xc = (np.arange(n) + 0.5) * length / n - length / 2
    top = np.interp(xs, xc, top_cells) + rng.normal(0, 0.035, len(xs))
    for _ in range(int(length / 1.0) + 1):                         # V-notches where chunks broke off
        c = rng.uniform(-length / 2, length / 2); w = rng.uniform(0.12, 0.35); dpt = rng.uniform(0.08, max(notch, 0.09))   # short walls (weiguan): notch < 0.08 made high < low
        top -= dpt * np.clip(1 - np.abs(xs - c) / w, 0, None)
    top = np.clip(top, 0.6, None)
    tf = top + rng.normal(0, 0.025, len(xs))                        # front and back tops differ a little
    tb = top + rng.normal(0, 0.025, len(xs))
    y0, y1 = -thick / 2, thick / 2
    P, F = [], []
    nx = len(xs)
    for i, x in enumerate(xs):                                      # front bottom, front top, back bottom, back top
        P += [[x, y0, 0.0], [x, y0, tf[i]], [x, y1, 0.0], [x, y1, tb[i]]]
    for i in range(nx - 1):
        a, b = 4 * i, 4 * (i + 1)
        F += [[a, b, b + 1], [a, b + 1, a + 1]]                     # front (-y)
        F += [[a + 2, a + 3, b + 3], [a + 2, b + 3, b + 2]]         # back (+y)
        F += [[a + 1, b + 1, b + 3], [a + 1, b + 3, a + 3]]         # top
    e = 4 * (nx - 1)
    F += [[0, 1, 3], [0, 3, 2], [e, e + 2, e + 3], [e, e + 3, e + 1]]   # end caps
    P = np.array(P, np.float32)
    F = np.array(F, np.int32)
    bars = []
    for _ in range(max(1, int(length / 0.6))):                      # exposed bars where the top broke
        i = rng.integers(nx)
        p0 = np.array([xs[i], rng.uniform(y0 + 0.08, y1 - 0.08), min(tf[i], tb[i]) - 0.1])
        bars.append(_bar(p0, p0 + np.r_[rng.normal(0, 0.12, 2), rng.uniform(0.12, 0.3)]))
    return (P, F), merge(bars), np.maximum(tf, tb), xs


def heap(heights, region, cell, rng, sub=3):
    """Rubble mound over `region` (H,W bool, map grid), peak heights `heights` (H,W, metres) per cell.
    Returns (P, F, sample) with P in WORLD x/y (x = col*cell, y = -row*cell) and z up; the outline drops to the
    floor at the region border. sample(x_world, y_world) -> surface height (for pebbles)."""
    H, W = region.shape
    Hc = np.where(region, heights, 0.0)
    Hs = gaussian_filter(zoom(Hc, sub, order=1), sigma=0.9 * sub)
    R = zoom(region.astype(np.uint8), sub, order=0).astype(bool)
    Hs = np.where(R, Hs, 0.0) * 1.25
    Hs += gaussian_filter(rng.normal(0, 0.10, Hs.shape), 0.7) * R
    Hs = np.clip(Hs, 0.0, None)
    rows, cols = Hs.shape
    s = cell / sub
    # vertex heights: mean of the in-region sub-cells around a vertex; 0 on the region border
    pad = np.pad(Hs, 1); Rp = np.pad(R, 1)
    acc = np.zeros((rows + 1, cols + 1)); cnt = np.zeros((rows + 1, cols + 1)); allin = np.ones((rows + 1, cols + 1), bool)
    for dr in (0, 1):
        for dc in (0, 1):
            acc += pad[dr:dr + rows + 1, dc:dc + cols + 1] * Rp[dr:dr + rows + 1, dc:dc + cols + 1]
            cnt += Rp[dr:dr + rows + 1, dc:dc + cols + 1]
            allin &= Rp[dr:dr + rows + 1, dc:dc + cols + 1]
    Zv = np.where(allin, acc / np.maximum(cnt, 1), 0.0)
    vid = -np.ones((rows + 1, cols + 1), int)
    P, F = [], []
    for r, c in np.argwhere(R):
        ids = []
        for (rr, cc) in ((r, c), (r, c + 1), (r + 1, c + 1), (r + 1, c)):
            if vid[rr, cc] < 0:
                vid[rr, cc] = len(P)
                P.append([cc * s + rng.normal(0, 0.02) * allin[rr, cc], -rr * s + rng.normal(0, 0.02) * allin[rr, cc],
                          Zv[rr, cc]])
            ids.append(vid[rr, cc])
        a, b, d, e = ids                         # world: x right, y up the screen = -row, so (a, d, b) is CCW from +z
        F += [[a, d, b], [a, e, d]]
    P = np.array(P, np.float32); F = np.array(F, np.int32)

    def sample(x, y):
        c = np.clip((np.asarray(x) / s).astype(int), 0, cols - 1)
        r = np.clip((-np.asarray(y) / s).astype(int), 0, rows - 1)
        return Hs[r, c]
    return P, F, sample


def pebbles(rng, xys, z0, n_each=1, smin=0.02, smax=0.09):
    """Many small stones (visual only), merged into one mesh. xys in local metres; z0 per point."""
    parts = []
    for (x, y), z in zip(xys, z0):
        for _ in range(n_each):
            s = rng.uniform(smin, smax)
            P, F = rock(rng, s, s * rng.uniform(0.6, 1.0), s * rng.uniform(0.35, 0.7), n=9)
            P = P + np.array([x + rng.normal(0, 0.12), y + rng.normal(0, 0.12), z + 0.4 * (P[:, 2].max())])
            parts.append((P, F))
    return merge(parts)


def normals(P, F):
    """Face-varying normals (one per face corner) for crisp facets."""
    a, b, c = P[F[:, 0]], P[F[:, 1]], P[F[:, 2]]
    n = np.cross(b - a, c - a)
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    return np.repeat(n, 3, axis=0).astype(np.float32)


def smooth_normals(P, F):
    """Area-weighted vertex normals (for the heap and rebar)."""
    a, b, c = P[F[:, 0]], P[F[:, 1]], P[F[:, 2]]
    n = np.cross(b - a, c - a)
    N = np.zeros_like(P, dtype=np.float64)
    for k in range(3):
        np.add.at(N, F[:, k], n)
    N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
    return N.astype(np.float32)


def piece_for_box(rng, sx, sy, sz, rebar_p=0.3):
    """An irregular piece that stands in for a box of size (sx, sy, sz) in the box's own frame (same centre, same
    axes), so a scene built from boxes keeps its layout, collisions and LiDAR returns roughly unchanged.
    Thin boxes become broken slab pieces (sometimes with rebar), long ones chipped blocks, the rest fractured lumps.
    Returns (mesh, rebar_mesh_or_None)."""
    dims = sorted((sx, sy, sz))
    if sz <= 0.4 * min(sx, sy):
        mesh = slab(rng, sx, sy, T=max(sz, 0.03), step=max(0.05, min(sx, sy) / 8))
        bars = rebar(rng, sx, sy, 0.0) if rng.random() < rebar_p and min(sx, sy) > 0.3 else None
        return mesh, bars
    if dims[2] > 2.2 * dims[1]:
        return block(rng, sx, sy, sz, chip=0.12), None
    return rock(rng, sx * 1.15, sy * 1.15, sz * 1.15, n=16), None
