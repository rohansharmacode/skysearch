"""Scenario from an EXTERNAL static mesh or heightmap (e.g. the Isaac team's Weiguan map, expanded).

    from sim.scenario import make_scenario
    sc = make_scenario(7, site="mesh:maps/weiguan_xl.npz?sectors=weiguan")
    sc = make_scenario_from_mesh("maps/wg.ply", seed=7, sectors="weiguan", geo={...})

Pipeline: load (``.npz`` heightmap, ``.ply`` mesh or point cloud, ``.obj`` mesh, ``.usd/.usda/.usdc`` via
usd-core when installed) -> rasterise to a max-z surface grid in the site frame (metres, x east, y north,
z up, origin at the geo anchor) -> estimate bare ground (morphological opening, or the file's ``ground``)
-> segment search sectors (building footprints from the file, else connected components above ground,
large ones split by k-means) -> hidden truth (victims in covered cells near faces, sounds) from a separate
RNG. Interchange format: docs/SITE_WEIGUAN.md, "Sending us a map".

The result is a normal ``Scenario``: ``piles`` = sectors (no box pieces; the surface is the terrain),
``ground_height`` = the loaded surface (so Isaac's terrain mesh, the LiDAR ray-caster and GridWorld all see
the rubble), ``heightmap`` = the surface resampled. ``truth`` stays HIDDEN GROUND TRUTH.

``synth_expanded_npz`` writes a procedural stand-in of the expanded map (Weiguan in the centre plus twelve
surrounding damaged buildings within ~150 m; HYPOTHETICAL, see ``EXPANDED_LAYOUT``) in the same
interchange format, so the loader, mission and dashboard can be exercised before the real export lands.
"""
from __future__ import annotations

import hashlib
import json
import os
from urllib.parse import parse_qs

import numpy as np

from .scenario import Pile, Scenario
from .world import Grid

MIN_H = 1.0            # m above ground counts as rubble / building
MIN_AREA = 30.0        # m^2: smaller blobs are loose debris, not a sector
MAX_AREA = 900.0       # m^2: larger components are split into ~equal sectors
GROUND_WINDOW = 110.0  # m: morphological opening window for the ground estimate (> widest wreck)


# ------------------------------------------------------------------------------------------- loading
def _read_ply(path):
    """Minimal PLY reader (ascii / binary_little_endian): vertices (N,3) and faces (M,3) or None."""
    with open(path, "rb") as f:
        if f.readline().strip() != b"ply":
            raise ValueError(f"{path}: not a PLY file")
        fmt, elems, cur = None, [], None
        while True:
            line = f.readline().decode("ascii", "replace").strip()
            if line.startswith("format"):
                fmt = line.split()[1]
            elif line.startswith("element"):
                _, name, n = line.split()
                cur = {"name": name, "n": int(n), "props": []}
                elems.append(cur)
            elif line.startswith("property"):
                cur["props"].append(line.split()[1:])
            elif line == "end_header":
                break
        types = {"char": "i1", "uchar": "u1", "short": "i2", "ushort": "u2", "int": "i4", "uint": "u4",
                 "float": "f4", "double": "f8", "int8": "i1", "uint8": "u1", "int16": "i2", "uint16": "u2",
                 "int32": "i4", "uint32": "u4", "float32": "f4", "float64": "f8"}
        V = F = None
        if fmt == "ascii":
            rest = f.read().decode("ascii").split("\n")
            k = 0
            for e in elems:
                rows = rest[k:k + e["n"]]
                k += e["n"]
                if e["name"] == "vertex":
                    names = [p[-1] for p in e["props"]]
                    a = np.array([r.split() for r in rows], float)
                    V = a[:, [names.index(c) for c in "xyz"]]
                elif e["name"] == "face":
                    F = _fan([list(map(int, r.split()[1:])) for r in rows])
            return V, F
        if fmt != "binary_little_endian":
            raise ValueError(f"{path}: PLY format {fmt} not supported (use ascii or binary_little_endian)")
        for e in elems:
            if e["name"] == "vertex":
                dt = np.dtype([(p[-1], "<" + types[p[0]]) for p in e["props"]])
                a = np.frombuffer(f.read(dt.itemsize * e["n"]), dt)
                V = np.stack([a["x"], a["y"], a["z"]], 1).astype(float)
            elif e["name"] == "face":
                p = e["props"][0]                      # list <count type> <index type> vertex_indices
                ct, it = "<" + types[p[1]], "<" + types[p[2]]
                faces = []
                for _ in range(e["n"]):
                    c = int(np.frombuffer(f.read(np.dtype(ct).itemsize), ct)[0])
                    faces.append(np.frombuffer(f.read(np.dtype(it).itemsize * c), it).tolist())
                    if len(e["props"]) > 1:
                        raise ValueError("PLY faces with extra properties are not supported")
                F = _fan(faces)
            else:
                raise ValueError(f"PLY element {e['name']} before faces is not supported")
        return V, F


def _fan(polys):
    tris = [(p[0], p[i], p[i + 1]) for p in polys for i in range(1, len(p) - 1)]
    return np.array(tris, int) if tris else None


def _read_obj(path):
    V, polys = [], []
    with open(path) as f:
        for line in f:
            if line.startswith("v "):
                V.append([float(t) for t in line.split()[1:4]])
            elif line.startswith("f "):
                polys.append([int(t.split("/")[0]) for t in line.split()[1:]])
    V = np.array(V, float)
    polys = [[i - 1 if i > 0 else len(V) + i for i in p] for p in polys]
    return V, _fan(polys)


def _read_usd(path):
    try:
        from pxr import Usd, UsdGeom
    except ImportError as e:
        raise ImportError("USD input needs usd-core (pip install usd-core) in this venv; or export .ply/.obj/.npz") from e
    stage = Usd.Stage.Open(path)
    mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
    yup = UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.y
    Vs, Fs, off = [], [], 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(prim)
        pts = np.array(m.GetPointsAttr().Get(), float)
        if len(pts) == 0:
            continue
        M = np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()), float)
        pts = np.c_[pts, np.ones(len(pts))] @ M            # USD row-vector convention
        counts = list(m.GetFaceVertexCountsAttr().Get())
        idx = list(m.GetFaceVertexIndicesAttr().Get())
        polys, k = [], 0
        for c in counts:
            polys.append([off + i for i in idx[k:k + c]])
            k += c
        Vs.append(pts[:, :3])
        Fs.append(_fan(polys))
        off += len(pts)
    V = np.concatenate(Vs) * mpu
    if yup:
        V = np.c_[V[:, 0], -V[:, 2], V[:, 1]]
    return V, np.concatenate([f for f in Fs if f is not None])


def rasterise(V, F, origin, res, w, h):
    """Max-z surface over the grid from triangles (dense barycentric sampling) or points; NaN = no data."""
    z = np.full((h, w), -np.inf)

    def splat(P):
        ix = np.floor((P[:, 0] - origin[0]) / res).astype(int)
        iy = np.floor((P[:, 1] - origin[1]) / res).astype(int)
        ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        np.maximum.at(z, (iy[ok], ix[ok]), P[ok, 2])

    splat(V)
    if F is not None and len(F):
        T = V[F]
        e = np.max(np.linalg.norm(T[:, [1, 2, 0], :2] - T[:, :, :2], axis=2), axis=1)
        nsub = np.clip(np.ceil(e / (0.5 * res)).astype(int), 1, 400)
        for n in np.unique(nsub):
            i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1))
            m = (i + j) <= n
            a, b = i[m] / n, j[m] / n
            B = np.stack([1 - a - b, a, b], 1)            # (m, 3)
            sel = T[nsub == n]
            step = max(1, 2_000_000 // len(B))
            for k in range(0, len(sel), step):
                splat(np.einsum("mk,tkd->tmd", B, sel[k:k + step]).reshape(-1, 3))
    z[~np.isfinite(z)] = np.nan
    return z


def fill_nan(z, max_gap_cells=4, floor=None):
    """Fill cells without data: small holes from the nearest valid cell, wide empty areas (no ground in
    the export) with ``floor`` (default: 2nd percentile of the data)."""
    from scipy import ndimage
    bad = np.isnan(z)
    if not bad.any():
        return z
    if bad.all():
        return np.zeros_like(z)
    dist, idx = ndimage.distance_transform_edt(bad, return_indices=True)
    out = z[tuple(idx)]
    out[dist > max_gap_cells] = float(np.percentile(z[~bad], 2)) if floor is None else floor
    return out


def estimate_ground(z, res, window_m=GROUND_WINDOW):
    """Bare ground under rubble/buildings: grey opening wider than any wreck, smoothed, <= surface."""
    from scipy import ndimage
    k = max(3, int(window_m / res) | 1)
    g = ndimage.maximum_filter(ndimage.minimum_filter(z, size=k, mode="nearest"), size=k, mode="nearest")
    g = ndimage.uniform_filter(g, size=max(3, (k // 4) | 1), mode="nearest")
    return np.minimum(g, z)


def load_surface(path, res=None, bounds=None, up="z"):
    """(Grid surface, ground (h,w) or None, meta dict) from .npz / .ply / .obj / .usd*."""
    ext = os.path.splitext(path)[1].lower()
    meta = {}
    if ext == ".npz":
        d = np.load(path, allow_pickle=False)
        origin, r = tuple(float(v) for v in d["origin"]), float(d["res"])
        z = np.asarray(d["z"], np.float32)
        ground = np.asarray(d["ground"], np.float32) if "ground" in d else None
        if "ground_z" in d and ground is None:
            ground = np.full_like(z, float(d["ground_z"]))
        for k in ("meta", "footprints"):
            if k in d:
                meta[k] = json.loads(str(d[k]))
        g = Grid(origin, r, z.shape[1], z.shape[0], fill_nan(z.astype(float), 1 << 30).astype(np.float32))
        if res is not None and abs(res - r) > 1e-9 or bounds is not None:
            g, ground = _resample(g, ground, res or r, bounds or g_bounds(g))
        return g, ground, meta
    if ext == ".ply":
        V, F = _read_ply(path)
    elif ext == ".obj":
        V, F = _read_obj(path)
    elif ext in (".usd", ".usda", ".usdc"):
        V, F = _read_usd(path)
    else:
        raise ValueError(f"{path}: unsupported map format {ext} (npz, ply, obj, usd/usda/usdc)")
    if up == "y":
        V = np.c_[V[:, 0], -V[:, 2], V[:, 1]]
    res = res or 0.5
    if bounds is None:
        lo, hi = V[:, :2].min(0) - 2.0, V[:, :2].max(0) + 2.0
        bounds = (np.floor(lo[0] / res) * res, np.floor(lo[1] / res) * res,
                  np.ceil(hi[0] / res) * res, np.ceil(hi[1] / res) * res)
    x0, y0, x1, y1 = bounds
    w, h = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
    z = rasterise(V, F, (x0, y0), res, w, h)
    floor = float(np.percentile(V[:, 2], 1))              # lowest geometry = street level if no ground plane
    return Grid((x0, y0), res, w, h, fill_nan(z, floor=floor).astype(np.float32)), None, meta


def g_bounds(g):
    return (g.origin[0], g.origin[1], g.origin[0] + g.w * g.res, g.origin[1] + g.h * g.res)


def _resample(g, ground, res, bounds):
    x0, y0, x1, y1 = bounds
    w, h = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
    n = Grid((x0, y0), res, w, h, np.zeros((h, w), np.float32))
    X, Y = n.mesh()
    n.z = g.height_at(X, Y).astype(np.float32)
    if ground is not None:
        ground = Grid(g.origin, g.res, g.w, g.h, ground).height_at(X, Y).astype(np.float32)
    return n, ground


# --------------------------------------------------------------------------------------- segmentation
def _kmeans(P, k, iters=25):
    """Deterministic k-means (farthest-point init) on (N,2) cell centres -> labels."""
    c = [P[np.argmin(P[:, 0] + P[:, 1])]]
    for _ in range(1, k):
        d = np.min([np.sum((P - q) ** 2, 1) for q in c], 0)
        c.append(P[int(np.argmax(d))])
    C = np.array(c, float)
    for _ in range(iters):
        lab = np.argmin(((P[:, None, :] - C[None]) ** 2).sum(-1), 1)
        C = np.array([P[lab == i].mean(0) if (lab == i).any() else C[i] for i in range(k)])
    return np.argmin(((P[:, None, :] - C[None]) ** 2).sum(-1), 1)


def segment(g, ground, footprints=None, centres=None, min_h=MIN_H, min_area=MIN_AREA, max_area=MAX_AREA):
    """Cell -> sector name ('' = not a sector) and sector list.

    footprints {name: [[x, y], ...]} (polygons, site frame) win; then ``centres`` {name: (x, y)} claim the
    components they sit on (Voronoi among them, e.g. Weiguan blocks A-I); remaining components > min_area
    become W1..Wn by distance from the origin, those > max_area split by k-means (W3a, W3b, ...).
    """
    from matplotlib.path import Path
    from scipy import ndimage
    X, Y = g.mesh()
    above = (g.z - ground) > min_h
    above = ndimage.binary_closing(ndimage.binary_opening(above), iterations=2) & ((g.z - ground) > 0.3)
    owner = np.full(g.z.shape, "", dtype=object)
    sectors = []
    cell = g.res ** 2

    def add(name, mask, source):
        owner[mask] = name
        sectors.append({"name": name, "source": source, "mask": mask})

    taken = np.zeros_like(above)
    for name, poly in (footprints or {}).items():
        m = Path(np.asarray(poly, float)).contains_points(np.c_[X.ravel(), Y.ravel()]).reshape(X.shape) & above & ~taken
        if m.sum() * cell >= min_area:
            add(name, m, "footprint")
            taken |= m
    lab, n = ndimage.label(above & ~taken)
    if centres:
        names = list(centres)
        C = np.array([centres[k] for k in names], float)
        comps = set()
        for cx, cy in C:
            d = np.hypot(X - cx, Y - cy)
            near = (lab > 0) & (d < 8.0)
            if near.any():
                comps.add(int(lab[near][np.argmin(d[near])]))
        if comps:
            m = np.isin(lab, list(comps))
            nearest = np.argmin((X[m][:, None] - C[:, 0]) ** 2 + (Y[m][:, None] - C[:, 1]) ** 2, 1)
            for i, name in enumerate(names):
                mm = np.zeros_like(m)
                mm[m] = nearest == i
                if mm.any():
                    add(name, mm, "centre")
            lab[m] = 0
    comps = []
    for i in range(1, n + 1):
        m = lab == i
        a = m.sum() * cell
        if a >= min_area:
            comps.append((float(np.hypot(X[m].mean(), Y[m].mean())), i, a))
    for k, (_, i, a) in enumerate(sorted(comps)):
        m = lab == i
        name = f"W{k + 1}"
        parts = int(np.ceil(a / max_area))
        if parts == 1:
            add(name, m, "component")
            continue
        P = np.c_[X[m], Y[m]]
        sub = _kmeans(P, parts)
        for j in range(parts):
            mm = np.zeros_like(m)
            mm[m] = sub == j
            add(f"{name}{chr(97 + j)}", mm, "split")
    return owner, sectors


# ------------------------------------------------------------------------------------------ scenario
class MeshScenario(Scenario):
    """Scenario whose terrain is a loaded surface grid (no box pieces)."""

    site = "mesh"

    def __init__(self, seed, piles, truth, surface, ground, source, meta, sectors):
        super().__init__(seed, piles, truth, (np.zeros(5), np.zeros(5), np.zeros(5), np.zeros(5)))
        self.surface, self.bare_ground, self.source, self.meta = surface, ground, source, meta
        self.bounds = tuple(float(v) for v in g_bounds(surface))
        self.geo = dict(meta.get("geo") or _default("GEO"))
        self.base_xy = tuple(meta.get("base_xy") or _default("BASE_XY"))
        self.drone_pads = [tuple(p) for p in (meta.get("drone_pads") or _default("DRONE_PADS"))]
        self._sectors = sectors

    def ground_height(self, x, y):
        """The loaded SURFACE (the rubble is terrain here; there are no separate pieces)."""
        return self.surface.height_at(x, y)

    def heightmap(self, res=0.5, bounds=None):
        if bounds is None and abs(res - self.surface.res) < 1e-9:
            g = self.surface
            return Grid(g.origin, g.res, g.w, g.h, g.z.astype(np.float32).copy())
        return _resample(self.surface, None, res, bounds or self.bounds)[0]

    def public_dict(self):
        d = super().public_dict()
        g = self.surface
        d.update({"site": "mesh", "bounds": list(self.bounds), "geo": dict(self.geo), "base_xy": list(self.base_xy),
                  "drone_pads": [list(p) for p in self.drone_pads], "spacing": None,
                  "terrain": {"kind": "heightmap", "source": os.path.basename(self.source),
                              "md5": _md5(self.source), "origin": list(g.origin), "res": g.res, "w": g.w, "h": g.h},
                  "sectors": self._sectors})
        return d


def _default(name):
    from . import site_weiguan
    return getattr(site_weiguan, name)


def _md5(path):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def make_scenario_from_mesh(path, seed=7, start_pile=None, geo=None, sectors=None, footprints=None,
                            res=None, bounds=None, up="z", min_h=MIN_H, max_area=MAX_AREA):
    """Build a Scenario from an external map. ``sectors``: None | "weiguan" | {name: (x, y)} centres."""
    from .site_weiguan import CENTRES, TruthSupport, make_truth
    surface, ground, meta = load_surface(path, res=res, bounds=bounds, up=up)
    if ground is None:
        ground = estimate_ground(surface.z.astype(float), surface.res)
    ground = np.minimum(np.asarray(ground, float), surface.z)
    if geo:
        meta["geo"] = geo
    if sectors == "weiguan":
        sectors = dict(CENTRES)
    elif sectors is None and "sectors" in meta.get("meta", {}):
        sectors = meta["meta"]["sectors"]
    footprints = footprints or meta.get("footprints")
    for k in ("geo", "base_xy", "drone_pads"):
        if k in meta.get("meta", {}) and k not in meta:
            meta[k] = meta["meta"][k]
    owner, secs = segment(surface, ground, footprints, sectors, min_h=min_h, max_area=max_area)
    if not secs:
        raise ValueError(f"{path}: no rubble above {min_h} m found (check units / up axis / ground)")
    X, Y = surface.mesh()
    cover = surface.z - ground
    piles, public = [], []
    for s in secs:
        m = s["mask"]
        cx, cy = float(X[m].mean()), float(Y[m].mean())
        h = float(cover[m].max())
        sx, sy = float(max(2.0, 2 * X[m].std())), float(max(2.0, 2 * Y[m].std()))
        piles.append(Pile(name=s["name"], x=round(cx, 3), y=round(cy, 3), h=round(h, 3), sx=round(sx, 3),
                          sy=round(sy, 3), rot=0.0, lump=(0.0, 0.0)))
        public.append({"name": s["name"], "source": s["source"], "area_m2": round(float(m.sum()) * surface.res ** 2, 1),
                       "max_h": round(h, 2), "bbox": [round(float(v), 2) for v in
                                                      (X[m].min(), Y[m].min(), X[m].max(), Y[m].max())]})
    names = [p.name for p in piles]
    if start_pile is not None and start_pile not in names:
        raise ValueError(f"start_pile {start_pile!r} not in sectors {names}")
    sc = MeshScenario(seed, piles, {}, surface, ground, path, meta, public)
    owner_s = np.where(owner == "", "", owner).astype(str)
    sup = TruthSupport(surface, ground, owner_s, solids=None, edge_bias=0.7)
    gg = Grid(surface.origin, surface.res, surface.w, surface.h, ground)
    sc.truth = make_truth(seed, start_pile, names, sup, gg.height_at, site="mesh", salt=2017)
    return sc


def make_from_spec(seed, start_pile, spec):
    """``site="mesh:<path>[?sectors=weiguan&res=0.5&up=y]"`` (the sim.scenario dispatch hook)."""
    path, _, q = spec.partition("?")
    opts = {k: v[-1] for k, v in parse_qs(q).items()}
    return make_scenario_from_mesh(path, seed, start_pile, sectors=opts.get("sectors"),
                                   res=float(opts["res"]) if "res" in opts else None, up=opts.get("up", "z"))


# ------------------------------------------------------------------------ expanded map (HYPOTHETICAL)
# Weiguan in the centre plus twelve surrounding damaged buildings within ~150 m. These are NOT real
# buildings: the 2016 earthquake brought down Weiguan alone in this block. They give the swarm a
# district-scale search (multiple wrecks, a standing tall block as a flight obstacle, varied collapse
# modes). Yongda Road (x 20.5..41.5) and the staging area on it stay clear.
#   xy: footprint centre (m); size: footprint (m) before collapse; yaw (deg); storeys; state:
#   pancake (floors stacked ~0.6 m per storey) | partial (half standing, half pancaked) |
#   lean (toppled sideways, lying heap) | standing (damaged but standing, debris skirt)
EXPANDED_LAYOUT = [
    {"name": "W1", "xy": (-62, 42), "size": (18, 30), "yaw": 4, "storeys": 7, "state": "pancake"},
    {"name": "W2", "xy": (-70, -35), "size": (20, 14), "yaw": -6, "storeys": 5, "state": "partial"},
    {"name": "W3", "xy": (72, 48), "size": (16, 40), "yaw": 0, "storeys": 8, "state": "lean"},
    {"name": "W4", "xy": (80, -18), "size": (25, 20), "yaw": 10, "storeys": 4, "state": "pancake"},
    {"name": "W5", "xy": (-22, 86), "size": (30, 16), "yaw": -3, "storeys": 6, "state": "partial"},
    {"name": "W6", "xy": (2, 112), "size": (18, 18), "yaw": 0, "storeys": 12, "state": "standing"},
    {"name": "W7", "xy": (-112, 8), "size": (14, 24), "yaw": 0, "storeys": 3, "state": "pancake"},
    {"name": "W8", "xy": (110, 62), "size": (20, 20), "yaw": 20, "storeys": 5, "state": "pancake"},
    {"name": "W9", "xy": (-42, -108), "size": (22, 16), "yaw": -12, "storeys": 6, "state": "lean"},
    {"name": "W10", "xy": (78, -100), "size": (16, 30), "yaw": 0, "storeys": 4, "state": "partial"},
    {"name": "W11", "xy": (-120, -72), "size": (12, 12), "yaw": 30, "storeys": 3, "state": "pancake"},
    {"name": "W12", "xy": (122, -70), "size": (18, 22), "yaw": 0, "storeys": 7, "state": "standing"},
]
EXPANDED_BOUNDS = (-160.0, -160.0, 160.0, 160.0)


def _rect(xy, size, yaw, grow=0.0):
    c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
    hx, hy = size[0] / 2 + grow, size[1] / 2 + grow
    return [[xy[0] + u * c - v * s, xy[1] + u * s + v * c] for u, v in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]


def synth_expanded_npz(out, res=0.5, seed=2016):
    """Procedural stand-in of the expanded map in the interchange format (see docs/SITE_WEIGUAN.md)."""
    from scipy import ndimage

    from . import site_weiguan as wg
    rng = np.random.default_rng(seed)
    x0, y0, x1, y1 = EXPANDED_BOUNDS
    w, h = int((x1 - x0) / res), int((y1 - y0) / res)
    g = Grid((x0, y0), res, w, h, np.zeros((h, w), np.float32))
    X, Y = g.mesh()
    z = np.zeros((h, w))
    core = wg._build_geometry_scenario(0).heightmap(res)    # the Weiguan stand-in, pasted in the centre
    i0, j0 = int(round((core.origin[0] - x0) / res)), int(round((core.origin[1] - y0) / res))
    z[j0:j0 + core.h, i0:i0 + core.w] = core.z
    rough = ndimage.zoom(rng.uniform(-1, 1, (h // 4 + 1, w // 4 + 1)), 4, order=1)[:h, :w]
    blocky = ndimage.zoom(rng.uniform(0, 1, (h // 2 + 1, w // 2 + 1)), 2, order=0)[:h, :w]
    footprints = {}
    for b in EXPANDED_LAYOUT:
        cx, cy = b["xy"]
        c, s = np.cos(np.radians(b["yaw"])), np.sin(np.radians(b["yaw"]))
        u, v = (X - cx) * c + (Y - cy) * s, -(X - cx) * s + (Y - cy) * c
        hx, hy = b["size"][0] / 2, b["size"][1] / 2
        H = 3.0 * b["storeys"]
        inside = (np.abs(u) <= hx) & (np.abs(v) <= hy)
        dout = np.hypot(np.clip(np.abs(u) - hx, 0, None), np.clip(np.abs(v) - hy, 0, None))
        skirt = np.clip(1 - dout / 5.0, 0, 1) * (1.2 + 0.6 * rough)
        st = b["state"]
        if st == "pancake":
            hz = np.where(inside, 0.6 * b["storeys"] + 0.8 * rough + 0.9 * blocky, 0)
        elif st == "partial":
            hz = np.where(inside & (u > 0), H - 1.5 * blocky, np.where(inside, 0.7 * b["storeys"] + rough + blocky, 0))
        elif st == "lean":       # toppled along +u: lying heap ~ footprint width high, twice as long
            lie = (u >= -hx) & (u <= -hx + H) & (np.abs(v) <= hy)
            prof = min(2 * hy, 0.45 * H) * (1 - 0.5 * (u + hx) / H)
            hz = np.where(lie, prof + rough + 1.2 * blocky, 0)
            dout = np.hypot(np.clip(np.abs(u - (-hx + H / 2)) - H / 2, 0, None), np.clip(np.abs(v) - hy, 0, None))
            skirt = np.clip(1 - dout / 5.0, 0, 1) * (1.2 + 0.6 * rough)
            hx_l = H / 2
            footprints[b["name"]] = _rect((cx + (-hx + hx_l) * c, cy + (-hx + hx_l) * s), (H, 2 * hy), b["yaw"], 4.0)
        else:                    # standing: full height block, debris skirt
            hz = np.where(inside, H, 0)
        z = np.maximum(z, np.maximum(hz, skirt))
        footprints.setdefault(b["name"], _rect(b["xy"], b["size"], b["yaw"], 4.0))
    z += 0.02 * rough
    meta = {"geo": dict(wg.GEO), "base_xy": list(wg.BASE_XY), "drone_pads": [list(p) for p in wg.DRONE_PADS],
            "sectors": {k: list(v) for k, v in wg.CENTRES.items()}, "frame": "site: m, x east, y north, z up",
            "source": "sim/site_mesh.py synth_expanded_npz (HYPOTHETICAL surroundings)"}
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    np.savez_compressed(out, origin=np.array([x0, y0]), res=np.float64(res), z=z.astype(np.float32),
                        ground=np.zeros((h, w), np.float32), meta=json.dumps(meta),
                        footprints=json.dumps(footprints))
    return out


def render(sc, out, truth=True):
    """Sector map of a mesh scenario (hillshade + sector outlines + labels [+ hidden truth])."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    g = sc.surface
    x0, y0, x1, y1 = sc.bounds
    gy, gx = np.gradient(g.z.astype(float), g.res)
    shade = np.clip(0.6 - 0.35 * gx + 0.35 * gy, 0, 1)
    fig, ax = plt.subplots(figsize=(10, 9.4))
    ax.imshow(shade, origin="lower", extent=(x0, x1, y0, y1), cmap="gray", vmin=0, vmax=1.2)
    im = ax.imshow(np.ma.masked_less(g.z - sc.bare_ground, 0.5), origin="lower", extent=(x0, x1, y0, y1),
                   cmap="magma_r", alpha=0.55, vmin=0, vmax=20)
    fig.colorbar(im, ax=ax, label="height above ground (m)", shrink=0.75)
    for p, s in zip(sc.piles, sc.public_dict()["sectors"]):
        bx0, by0, bx1, by1 = s["bbox"]
        ax.text(p.x, p.y, p.name, fontsize=9 if len(p.name) < 3 else 7, weight="bold", ha="center", va="center",
                color="#063", bbox=dict(fc="white", ec="none", alpha=0.6, pad=0.5))
    ax.plot(*zip(*sc.drone_pads), "bs", ms=3)
    ax.text(sc.base_xy[0], sc.base_xy[1] - 6, "staging", ha="center", fontsize=8, color="b")
    if truth:
        mk = {"shallow": "#1a9850", "mid": "#fdae61", "deep": "#d73027"}
        for v in sc.truth["victims"]:
            ax.plot(*v["xy"], "o", ms=5, mfc=mk[v["depth_class"]], mec="k", mew=0.5)
    ax.set_xlabel("x east (m)"); ax.set_ylabel("y north (m)")
    ax.set_title(f"{os.path.basename(sc.source)}: {len(sc.piles)} search sectors"
                 + (f", HIDDEN TRUTH seed {sc.seed} (dots: green <1.5 m, orange 1.5-3 m, red >4 m deep)" if truth else ""),
                 fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="External map -> Scenario; or write the procedural expanded map.")
    ap.add_argument("map", help="input map (.npz/.ply/.obj/.usd*); with --synth, the .npz to write")
    ap.add_argument("--synth", action="store_true", help="write the HYPOTHETICAL expanded Weiguan map first")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--sectors", default=None, help="'weiguan' to name the central wreck's sectors A-I")
    ap.add_argument("--render", help="PNG sector map")
    ap.add_argument("--public-out", help="write the public scenario JSON")
    a = ap.parse_args()
    if a.synth:
        synth_expanded_npz(a.map)
    sc = make_scenario_from_mesh(a.map, a.seed, sectors=a.sectors)
    if a.render:
        render(sc, a.render)
    if a.public_out:
        with open(a.public_out, "w") as f:
            f.write(sc.public_json(indent=1))
    print(f"{a.map}: {len(sc.piles)} sectors {[p.name for p in sc.piles]}, {len(sc.truth['victims'])} victims")
