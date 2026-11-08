"""Earthquake-hit town around the 3x3 rubble grid (``--town DIR``, isaac/photoreal.py; visual only).

The grid site reads as one block of a real city after a quake: every heap is a collapsed building on its own lot,
with streets between the lots, kerbs and sidewalks, broken perimeter walls, parked cars under dust, leaning utility
poles with sagging wires, and a ring of standing (some cracked or partly collapsed) 2-6 storey buildings outside
the grid. The kit (isaac/assets/town_kit: town_kit.usdc + textures/, built in Blender by
isaac/tools/blender_town_kit.py) holds the repeated pieces; this module lays them out and writes:

    /World/Town/Looks          the kit's shared ground materials (referenced)
    /World/Town/Ground/*       draped ground: asphalt, sidewalks, kerbs, yards, rubble beds, dust (unique meshes)
    /World/Town/Decals/*       faded lane markings, patches, quake cracks, manhole covers
    /World/Town/<kind>         one UsdGeom.PointInstancer per kit module kind (buildings, cars, poles, walls ...)
    /World/Town/Wires, Tape    sagging power lines between the poles, rescue cordon tape near the staging area

Visual only: no colliders and no physics APIs, so the LiDAR, the landing search and the mission never see it.

    plan = town.plan(sc, seed)            # pure numpy: layout + placements + clearance checks (no pxr)
    kit = town.load(town.default_dir())   # None (logged) if the kit is missing
    town.place(stage, "/World/Town", kit, plan, log)

Clearances (``plan``): every placement stays outside the pile footprints ((x-px)/(1.35 sx))^2 + (..)^2 > 1, out of
the staging corridor (x -12..12, y -60..-36) and the van area, and >= 3 m below every drone track point within
3 m horizontally (flat litter: 1 m). Tracks: ``tracks`` (N,3) if given, else the straight legs between the piles
and the staging pads at the lowest cruise altitude plus the recorded take (``tracks/grid_s<seed>.json`` next to
this module's kit folder, written by ``python3 isaac/town.py record --seed 7``: the dry run's drone poses and the
director-take camera). Buildings near the camera's paths are capped below the eye and its sight lines.
"""
from __future__ import annotations

import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
USD_NAME = "town_kit.usdc"
KINDS = ("bldg", "car", "truck", "wreck", "pole", "light", "tree", "wall", "gate", "stub", "debris", "sheet",
         "litter", "manhole", "post")
STAGING = (-12.0, -60.0, 12.0, -36.0)          # drone pads, operator: keep open (x0, y0, x1, y1)
VAN_AREA = (-12.0, -62.0, 22.0, -36.0)         # isaac/van.py: van at (10, -49), tent, generator, cones
PILE_K = 1.35                                  # footprint ellipse factor (placements stay outside)
EXTENT = (-148.0, -144.0, 148.0, 152.0)        # town ground (beyond: flat far ground); multiples of 8 m
NEAR = (-72.0, -84.0, 72.0, 74.0)              # full ground detail (kerb faces, kerb strips) inside this box
CRUISE_MIN = 8.0                               # sim KinematicFlight default cruise altitude (lowest drone)
CLEAR_V, CLEAR_H, CLEAR_H_FLAT = 3.0, 3.0, 1.0

# ------------------------------------------------------------------------------------------------ kit specs
# Building variants: built by isaac/tools/blender_town_kit.py from these specs. Module frame: x along the frontage
# (W), the street facade at y = 0 facing -y, the building extends to +y (depth D), z up from the street level.
# ground: shop = 3.6 m storey with roller shutters and sign bands; facade: plaster | tile | brick (RC frame + brick
# infill); damage: none | cracked | corner (corner bays collapsed) | soft (ground storey crushed, leaning) |
# facade (infill panels fallen out) | top (top storey pancaked).
BLDG = [
    dict(W=10.0, D=10.0, n=3, ground="shop", facade="plaster", damage="cracked", balc=1, roof="tanks"),
    dict(W=12.0, D=11.0, n=4, ground="shop", facade="plaster", damage="none", balc=2, roof="solar"),
    dict(W=8.0, D=9.0, n=2, ground="res", facade="brick", damage="cracked", balc=0, roof="tanks"),
    dict(W=14.0, D=11.0, n=5, ground="shop", facade="tile", damage="cracked", balc=0, roof="shed"),
    dict(W=10.0, D=10.0, n=4, ground="res", facade="plaster", damage="corner", balc=1, roof="tanks"),
    dict(W=16.0, D=12.0, n=6, ground="shop", facade="plaster", damage="none", balc=2, roof="solar"),
    dict(W=12.0, D=10.0, n=3, ground="shop", facade="plaster", damage="soft", balc=1, roof="tanks"),
    dict(W=9.0, D=9.5, n=3, ground="res", facade="brick", damage="facade", balc=0, roof="tanks"),
    dict(W=13.0, D=11.0, n=5, ground="shop", facade="tile", damage="none", balc=0, roof="tanks"),
    dict(W=11.0, D=10.0, n=4, ground="res", facade="plaster", damage="top", balc=2, roof="solar"),
    dict(W=15.0, D=11.0, n=4, ground="shop", facade="plaster", damage="cracked", balc=1, roof="shed"),
    dict(W=8.0, D=9.0, n=2, ground="shop", facade="plaster", damage="none", balc=0, roof="tanks"),
]
GROUND_STOREY, STOREY, PARAPET = 3.6, 3.0, 1.0


def bldg_height(spec):
    """Nominal top (m above street) of a building variant incl. parapet and rooftop tanks / sheds."""
    h = (GROUND_STOREY if spec["ground"] == "shop" else STOREY) + (spec["n"] - 1) * STOREY
    return h + PARAPET + 2.0


# cars: L, W, H (m), x forward, origin at ground centre. Paint is a constant material; colours are prototypes.
CARS = [dict(name="sedan", L=4.55, W=1.78, H=1.45), dict(name="hatch", L=4.0, W=1.72, H=1.50),
        dict(name="suv", L=4.6, W=1.86, H=1.72), dict(name="van", L=4.85, W=1.90, H=1.98),
        dict(name="pickup", L=5.25, W=1.86, H=1.80)]
TRUCK = dict(L=6.9, W=2.25, H=3.1)
CAR_COLOURS = [(0.50, 0.50, 0.48), (0.38, 0.39, 0.40), (0.20, 0.205, 0.21), (0.055, 0.075, 0.12),
               (0.22, 0.075, 0.06), (0.47, 0.46, 0.42), (0.045, 0.045, 0.047), (0.16, 0.17, 0.14),
               (0.30, 0.27, 0.21)]                       # dusty white, silver, grey, dark blue, dull red, beige ...
PLASTER_TINTS = [(0.80, 0.75, 0.66), (0.80, 0.69, 0.52), (0.70, 0.70, 0.68), (0.82, 0.68, 0.58),
                 (0.86, 0.84, 0.79), (0.66, 0.70, 0.62), (0.64, 0.68, 0.72), (0.76, 0.62, 0.50)]
DIMS = {"pole": (0.8, 7.6), "light": (1.6, 6.6), "tree": (2.4, 6.5), "wall": (1.6, 2.3), "gate": (1.9, 2.4),
        "stub": (2.0, 3.0), "debris": (1.6, 1.0), "sheet": (1.2, 0.3), "litter": (1.2, 0.08), "manhole": (0.45, 0.03),
        "post": (0.3, 1.1), "wreck": (2.4, 1.9)}       # kind: (footprint radius, top) when kit.json has none
FLAT = ("sheet", "litter", "manhole")
TINTS_PER_BLDG = 4                    # plaster tints per building variant (each tint = one more prototype copy of
                                      # the variant's plaster faces; the rest of the module is instanced once)


def bldg_tints(var):
    return [(3 * var + k) % len(PLASTER_TINTS) for k in range(TINTS_PER_BLDG)]

# ground classes: offset above the terrain (m)
OFF = {"asphalt": 0.02, "lot": 0.035, "sidewalk": 0.10, "kerb": 0.112}
EXTRA = {"bed": 0.012, "dust": 0.006}


APRON_Z = -0.06                                # isaac/scene.py: flat apron under the terrain (shows where it dips)


def surface(sc, x, y):
    """Visible ground: the terrain, or the flat apron where the terrain dips below it."""
    return np.maximum(np.asarray(sc.ground_height(np.asarray(x, float), np.asarray(y, float)), float).ravel(), APRON_Z)


def default_dir():
    """The repo's kit (isaac/assets/town_kit) if it was built, else ''."""
    d = os.path.join(HERE, "assets", "town_kit")
    return d if os.path.exists(os.path.join(d, USD_NAME)) else ""


class Kit:
    """A built kit folder: the USD, the module names per kind and their dims (kit.json)."""

    def __init__(self, folder):
        folder = os.path.abspath(os.path.expanduser(folder))
        self.folder = folder
        self.usd = os.path.join(folder, USD_NAME)
        if not os.path.exists(self.usd):
            raise FileNotFoundError(f"no {USD_NAME} in {folder} (build it: isaac/tools/blender_town_kit.py)")
        meta = json.load(open(os.path.join(folder, "kit.json")))
        self.meta = meta
        self.modules = meta["modules"]
        self.dims = meta.get("dims", {})
        self.tinted = set(meta.get("tinted", []))                # modules with a /tint part (plaster, paint)
        self.variants = {k: sorted([m for m in self.modules if m.rsplit("_", 1)[0] == k],
                                   key=lambda m: int(m.rsplit("_", 1)[1])) for k in KINDS}
        if not self.variants["bldg"]:
            raise ValueError(f"town kit {folder} has no bldg modules")

    def counts(self):
        return {k: len(v) for k, v in self.variants.items() if v}


def load(folder, log=print):
    """Kit or None (logged) - never raises, so the caller can skip the town."""
    if not folder:
        return None
    if not os.path.isabs(folder) and not os.path.isdir(folder):   # e.g. isaac/assets/town_kit from the IsaacLab cwd
        rel = os.path.join(os.path.dirname(HERE), folder)          # (rehearsal 5): resolve against the repo
        folder = rel if os.path.isdir(rel) else folder
    try:
        return Kit(folder)
    except Exception as e:
        log(f"[town] kit {folder!r} not used ({type(e).__name__}: {e})")
        return None


def _kit_dims():
    d = default_dir()
    if d:
        try:
            return json.load(open(os.path.join(d, "kit.json"))).get("dims", {})
        except Exception:
            return {}
    return {}


# ------------------------------------------------------------------------------------------------ helpers
def _snap(v, q=0.5):
    return round(float(v) / q) * q


def _rot2(yaw_deg):
    a = math.radians(yaw_deg)
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def quat_zyx(yaw, rx=0.0, ry=0.0):
    """(w, x, y, z) of R = Rz(yaw) @ Ry(ry) @ Rx(rx) (degrees)."""
    def q(axis, d):
        h = math.radians(d) / 2
        v = [0.0, 0.0, 0.0]
        v[axis] = math.sin(h)
        return np.array([math.cos(h), *v])

    def mul(a, b):
        w1, x1, y1, z1 = a
        w2, x2, y2, z2 = b
        return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                         w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])
    return mul(mul(q(2, yaw), q(1, ry)), q(0, rx))


def rot_zyx(yaw, rx=0.0, ry=0.0):
    a, b, c = (math.radians(v) for v in (yaw, ry, rx))
    Rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    Ry = np.array([[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(c), -math.sin(c)], [0, math.sin(c), math.cos(c)]])
    return Rz @ Ry @ Rx


def _sd_rrect(x, y, x0, y0, x1, y1, r):
    """Signed distance to a rounded rectangle (negative inside)."""
    cx, cy, hx, hy = (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2, (y1 - y0) / 2
    r = min(r, hx, hy)
    qx, qy = np.abs(x - cx) - hx + r, np.abs(y - cy) - hy + r
    out = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0))
    return out + np.minimum(np.maximum(qx, qy), 0) - r


def _in_rect(x, y, r, pad=0.0):
    return (r[0] - pad <= x) & (x <= r[2] + pad) & (r[1] - pad <= y) & (y <= r[3] + pad)


# ------------------------------------------------------------------------------------------------ layout
class Layout:
    """Street lines and blocks of the town around the 3x3 grid (site frame, metres).

    xlines / ylines: [(centre, total width, carriageway, kind)]; N-S streets at x = centre, E-W at y = centre.
    blocks: dicts with kerb (x0, y0, x1, y1) = the kerb line, lot = the property line (kerb inset by the sidewalk
    of each side), kind pile | ring, pile name, sides {W,E,S,N: sidewalk width or 0 at the town's outer edge}."""

    def __init__(self, sc):
        self.sc = sc
        piles = list(sc.piles)
        cols, rows = {}, {}
        for p in piles:
            cols.setdefault(p.name[0], []).append(p)
            rows.setdefault(p.name[1:], []).append(p)
        ca, cb, cc = (cols[k] for k in sorted(cols))
        r1, r2, r3 = (rows[k] for k in sorted(rows))
        K = 1.2
        xa = _snap((max(p.x + K * p.sx for p in ca) + min(p.x - K * p.sx for p in cb)) / 2)
        xb = _snap((max(p.x + K * p.sx for p in cb) + min(p.x - K * p.sx for p in cc)) / 2)
        y12 = _snap((min(p.y - K * p.sy for p in r1) + max(p.y + K * p.sy for p in r2)) / 2)
        y23 = _snap((min(p.y - K * p.sy for p in r2) + max(p.y + K * p.sy for p in r3)) / 2)
        ring = (9.0, 6.0, "ring")
        self.xlines = [(-121.5, *ring), (-93.0, *ring), (-64.5, *ring), (-34.5, 12.0, 8.0, "main"),
                       (xa, 9.0, 6.0, "inner"), (xb, 9.0, 6.0, "inner"), (34.5, 12.0, 8.0, "main"),
                       (64.5, *ring), (93.0, *ring), (121.5, *ring)]
        self.ylines = [(-120.0, *ring), (-91.5, *ring), (-63.0, *ring), (-35.5, 12.0, 8.0, "main"),
                       (y23, 9.0, 6.0, "inner"), (y12, 9.0, 6.0, "inner"), (34.5, 12.0, 8.0, "main"),
                       (64.5, *ring), (93.0, *ring), (121.5, *ring)]
        self.plaza = (-34.5, -63.0, 34.5, -35.5)          # line centres bounding the open square (staging)
        self.blocks = []
        X = [None] + self.xlines + [None]
        Y = [None] + self.ylines + [None]
        for i in range(len(X) - 1):
            for j in range(len(Y) - 1):
                lx0, lx1, ly0, ly1 = X[i], X[i + 1], Y[j], Y[j + 1]
                x0 = EXTENT[0] if lx0 is None else lx0[0] + lx0[2] / 2
                x1 = EXTENT[2] if lx1 is None else lx1[0] - lx1[2] / 2
                y0 = EXTENT[1] if ly0 is None else ly0[0] + ly0[2] / 2
                y1 = EXTENT[3] if ly1 is None else ly1[0] - ly1[2] / 2
                cxl, cyl = (lx0[0] if lx0 else EXTENT[0]), (ly0[0] if ly0 else EXTENT[1])
                cxh, cyh = (lx1[0] if lx1 else EXTENT[2]), (ly1[0] if ly1 else EXTENT[3])
                if cxl >= self.plaza[0] and cxh <= self.plaza[2] and cyl >= self.plaza[1] and cyh <= self.plaza[3]:
                    continue                                  # the open square in front of the staging area
                ws = {"W": 0.0 if lx0 is None else (lx0[1] - lx0[2]) / 2, "E": 0.0 if lx1 is None else (lx1[1] - lx1[2]) / 2,
                      "S": 0.0 if ly0 is None else (ly0[1] - ly0[2]) / 2, "N": 0.0 if ly1 is None else (ly1[1] - ly1[2]) / 2}
                lot = (x0 + ws["W"], y0 + ws["S"], x1 - ws["E"], y1 - ws["N"])
                inside = [p for p in piles if lot[0] - 3 < p.x < lot[2] + 3 and lot[1] - 3 < p.y < lot[3] + 3]
                kinds = {lx0[3] if lx0 else "edge", lx1[3] if lx1 else "edge", ly0[3] if ly0 else "edge",
                         ly1[3] if ly1 else "edge"}
                radius = 2.5 if kinds & {"main", "inner"} else 1.5
                self.blocks.append(dict(kerb=(x0, y0, x1, y1), lot=lot, sides=ws, radius=radius,
                                        kind="pile" if inside else "ring", pile=inside[0].name if inside else None))

    # --- implicit ground classes (negative inside)
    def kerb_sd(self, x, y):
        """Signed distance to the nearest block's kerb outline (negative inside a block)."""
        d = np.full(np.shape(x), 1e9)
        for b in self.blocks:
            x0, y0, x1, y1 = b["kerb"]
            d = np.minimum(d, _sd_rrect(x, y, x0, y0, x1, y1, b["radius"] if min(b["sides"].values()) > 0 else 0.0))
        return d

    def lot_sd(self, x, y):
        d = np.full(np.shape(x), 1e9)
        for b in self.blocks:
            x0, y0, x1, y1 = b["lot"]
            d = np.minimum(d, _sd_rrect(x, y, x0, y0, x1, y1, 0.0))
        return d

    def street_segments(self):
        """Carriageway pieces between intersections: (axis, centre, lo, hi, carriage, kind) for markings,
        parking, poles. axis 'x' = N-S street at x = centre spanning y in [lo, hi]."""
        segs = []
        for axis, lines, cross in (("x", self.xlines, self.ylines), ("y", self.ylines, self.xlines)):
            cuts = [EXTENT[1] if axis == "x" else EXTENT[0]] + [c for c, *_ in cross] + \
                   [EXTENT[3] if axis == "x" else EXTENT[2]]
            widths = [0.0] + [car for _, _, car, _ in cross] + [0.0]
            for c, w, car, kind in lines:
                for k in range(len(cuts) - 1):
                    lo, hi = cuts[k] + widths[k] / 2, cuts[k + 1] - widths[k + 1] / 2
                    mid = (lo + hi) / 2
                    px, py = (c, mid) if axis == "x" else (mid, c)
                    if self.plaza[0] < px < self.plaza[2] and self.plaza[1] < py < self.plaza[3] and kind != "main":
                        continue                              # inner lines do not cross the square
                    if axis == "y" and kind != "main" and self.plaza[1] < c < self.plaza[3] and \
                            self.plaza[0] - 1 < mid < self.plaza[2] + 1:
                        continue
                    segs.append((axis, c, lo, hi, car, kind))
        return segs


# ------------------------------------------------------------------------------------------------ piles
class Piles:
    def __init__(self, sc):
        self.P = np.array([[p.x, p.y, p.sx, p.sy] for p in sc.piles], float)
        self.names = [p.name for p in sc.piles]

    def rho(self, x, y):
        """Footprint radius in units of the pile ellipse (min over piles), and the nearest pile index."""
        x, y = np.asarray(x, float), np.asarray(y, float)
        r = np.stack([np.hypot((x - px) / sx, (y - py) / sy) for px, py, sx, sy in self.P])
        return r.min(0), r.argmin(0)

    def outside(self, pts, k=PILE_K, pad=0.0):
        """True if every point (N,2) is outside every pile ellipse k*s (+pad m)."""
        pts = np.atleast_2d(pts)
        for px, py, sx, sy in self.P:
            e = ((pts[:, 0] - px) / (k * sx + pad)) ** 2 + ((pts[:, 1] - py) / (k * sy + pad)) ** 2
            if (e <= 1.0).any():
                return False
        return True


def _ragged(rng, n=9):
    """Smooth ragged radius factor R(theta) for heap beds / dust: (amps, phases)."""
    k = np.arange(2, 2 + n)
    return rng.uniform(0.02, 0.07, n) / np.sqrt(k / 2), rng.uniform(0, 2 * np.pi, n), k


def _rag(theta, rag):
    a, ph, k = rag
    return (a[None, :] * np.sin(np.multiply.outer(theta, k) + ph[None, :])).sum(-1)


# ------------------------------------------------------------------------------------------------ clearance
def densify(tracks, step=0.5):
    """[(N,3|4) polylines] or (N,3) points -> points every <= step m along each polyline."""
    if tracks is None:
        return np.zeros((0, 3))
    if isinstance(tracks, np.ndarray) and tracks.ndim == 2:
        return tracks[:, :3].astype(float)
    out = []
    for t in tracks:
        t = np.asarray(t, float)[:, :3]
        if len(t) < 2:
            out.append(t)
            continue
        d = np.linalg.norm(np.diff(t, axis=0), axis=1)
        for a, b, L in zip(t[:-1], t[1:], d):
            n = max(1, int(math.ceil(L / step)))
            s = np.linspace(0, 1, n, endpoint=False)[:, None]
            out.append(a + (b - a) * s)
        out.append(t[-1:])
    return np.concatenate(out) if out else np.zeros((0, 3))


class Ceiling:
    """Raster of the lowest drone point within h m horizontally (per cell), for fast clearance queries."""

    def __init__(self, pts, h=CLEAR_H, res=0.5, bounds=(-70.0, -80.0, 70.0, 70.0)):
        self.res, self.b, self.h = res, bounds, h
        nx, ny = int((bounds[2] - bounds[0]) / res) + 1, int((bounds[3] - bounds[1]) / res) + 1
        self.z = np.full((ny, nx), np.inf)
        if len(pts):
            r = int(math.ceil(h / res))
            off = [(di, dj) for di in range(-r, r + 1) for dj in range(-r, r + 1) if math.hypot(di, dj) * res <= h + res * 0.71]
            i = np.round((pts[:, 0] - bounds[0]) / res).astype(int)
            j = np.round((pts[:, 1] - bounds[1]) / res).astype(int)
            for di, dj in off:
                ii, jj = i + di, j + dj
                ok = (ii >= 0) & (ii < nx) & (jj >= 0) & (jj < ny)
                np.minimum.at(self.z, (jj[ok], ii[ok]), pts[ok, 2])

    def min_in(self, x, y, r):
        """Lowest drone z over a disc (x, y, r)."""
        b, res = self.b, self.res
        i0, i1 = int(math.floor((x - r - b[0]) / res)), int(math.ceil((x + r - b[0]) / res))
        j0, j1 = int(math.floor((y - r - b[1]) / res)), int(math.ceil((y + r - b[1]) / res))
        ny, nx = self.z.shape
        i0, j0, i1, j1 = max(i0, 0), max(j0, 0), min(i1, nx - 1), min(j1, ny - 1)
        if i0 > i1 or j0 > j1:
            return math.inf
        sub = self.z[j0:j1 + 1, i0:i1 + 1]
        X = b[0] + np.arange(i0, i1 + 1) * res
        Y = b[1] + np.arange(j0, j1 + 1) * res
        m = np.hypot(X[None, :] - x, Y[:, None] - y) <= r + res * 0.71
        return float(sub[m].min()) if m.any() else math.inf

    def min_rect(self, corners):
        """Lowest drone z over a convex polygon (K,2) (buildings)."""
        b, res = self.b, self.res
        lo, hi = corners.min(0), corners.max(0)
        ny, nx = self.z.shape
        i0, i1 = max(int((lo[0] - b[0]) / res), 0), min(int((hi[0] - b[0]) / res) + 1, nx - 1)
        j0, j1 = max(int((lo[1] - b[1]) / res), 0), min(int((hi[1] - b[1]) / res) + 1, ny - 1)
        if i0 > i1 or j0 > j1:
            return math.inf
        X, Y = np.meshgrid(b[0] + np.arange(i0, i1 + 1) * res, b[1] + np.arange(j0, j1 + 1) * res)
        inside = np.ones(X.shape, bool)
        K = len(corners)
        for k in range(K):
            a, c = corners[k], corners[(k + 1) % K]
            inside &= (c[0] - a[0]) * (Y - a[1]) - (c[1] - a[1]) * (X - a[0]) >= -res
        sub = self.z[j0:j1 + 1, i0:i1 + 1]
        return float(sub[inside].min()) if inside.any() else math.inf


def proxy_tracks(sc, z=CRUISE_MIN):
    """Straight legs between every pair of piles and from the staging pads to every pile at the lowest cruise
    altitude (sim KinematicFlight: cruise_alt 8 m + stagger), as polylines."""
    from isaac.site import drone_pads
    pts = [(p.x, p.y) for p in sc.piles]
    legs = []
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            legs.append(np.array([[*pts[i], z], [*pts[j], z]]))
    for px, py in drone_pads(sc):
        for q in pts:
            legs.append(np.array([[px, py, z], [*q, z]]))
    return legs


def proxy_eyes(sc):
    """Director-take camera stand-ins (isaac/live.py TakeCamera shots): (eye, look) around every pile at every
    azimuth for the survey / pods / listen / event framings."""
    out = []
    for p in sc.piles:
        pz = float(np.asarray(sc.ground_height(np.array([p.x]), np.array([p.y]))).ravel()[0])
        look = np.array([p.x, p.y, pz + 1.5])
        for az in np.radians(np.arange(0, 360, 7.5)):
            for d, h in ((42.0, 26.0), (32.0, 15.0), (48.0, 30.0), (26.0, 12.0), (22.0, 10.0)):
                eye = np.array([p.x + d * math.cos(az), p.y + d * math.sin(az), pz + 1.5 + h])
                out.append((eye, look))
    return out


def recorded(seed, site="grid", folder=None):
    """The recorded take for (site, seed): (tracks [polylines], eyes [(eye, look)]) or (None, None)."""
    folder = folder or os.path.join(HERE, "assets", "town_kit", "tracks")
    f = os.path.join(folder, f"{site}_s{seed}.json")
    if not os.path.exists(f):
        return None, None
    d = json.load(open(f))
    tracks = [np.asarray(t, float) for t in d.get("tracks", {}).values()]
    eyes = [(np.asarray(e, float), np.asarray(l, float)) for e, l in d.get("eyes", [])]
    return tracks, eyes


# ------------------------------------------------------------------------------------------------ plan
class _Placer:
    def __init__(self, sc, seed, tracks, eyes, dims, boxes=None):
        from isaac.site import base_xy
        self.sc, self.rng = sc, np.random.default_rng(9100 + seed)
        self.L = Layout(sc)
        self.piles = Piles(sc)
        self.base = base_xy(sc)
        self.dims = dims
        self.boxes = boxes or {}                             # module: ((lo x, lo y), (hi x, hi y)) from kit.json
        pts = densify(tracks)
        self.track_pts = pts
        self.ceil = Ceiling(pts, CLEAR_H)
        self.ceil_flat = Ceiling(pts, CLEAR_H_FLAT)
        self.eyes = eyes or []
        self.items, self.wires, self.tape = [], [], []
        self.dropped = {}
        self.occupied = []                                   # (x, y, r) of placed things (no overlaps)

    def g(self, x, y):
        return float(surface(self.sc, np.array([x]), np.array([y]))[0])

    def level(self, x, y):
        """Street-level z at (x, y): terrain + ground-class offset."""
        k = self.L.kerb_sd(np.array([x]), np.array([y]))[0]
        lo = self.L.lot_sd(np.array([x]), np.array([y]))[0]
        off = OFF["asphalt"] if k > 0 else (OFF["lot"] if lo < 0 else OFF["sidewalk"])
        return self.g(x, y) + off

    def free(self, x, y, r):
        return all(math.hypot(x - a, y - b) >= r + c for a, b, c in self.occupied)

    def keepout(self, pts):
        pts = np.atleast_2d(pts)
        for R in (STAGING, VAN_AREA):
            if _in_rect(pts[:, 0], pts[:, 1], R, 0.5).any():
                return True
        return False

    def drone_ok(self, x, y, r, top, flat=False):
        c = (self.ceil_flat if flat else self.ceil).min_in(x, y, r)
        return c - top >= CLEAR_V

    def add(self, kind, var, x, y, yaw=0.0, z=None, rx=0.0, ry=0.0, s=(1.0, 1.0, 1.0), r=None, top=None,
            fp=None, mat=None, check=True, occupy=True):
        """Place one instance if it passes the pile / staging / drone checks. fp: footprint points (K,2)
        (default: the disc r). top: height of the instance above its base."""
        dr, dt = self.dims.get(f"{kind}_{var}", None) or (None, None)
        if dr is None:
            dr, dt = DIMS.get(kind, (1.0, 1.0))
        r = dr * max(s[0], s[1]) if r is None else r
        top = dt * s[2] if top is None else top
        z = self.level(x, y) if z is None else z
        if fp is None:
            a = np.linspace(0, 2 * np.pi, 12, endpoint=False)
            fp = np.c_[x + r * np.cos(a), y + r * np.sin(a)]
        bx = self.boxes.get(f"{kind}_{var}")
        if bx is not None and kind != "bldg":                # + the module's real bounds (rubble spilling sideways)
            (lx, ly), (hx, hy) = bx
            loc = np.array([[lx, ly], [hx, ly], [hx, hy], [lx, hy], [0.5 * (lx + hx), ly], [0.5 * (lx + hx), hy],
                            [lx, 0.5 * (ly + hy)], [hx, 0.5 * (ly + hy)]]) * [s[0], s[1]]
            fp = np.concatenate([np.asarray(fp, float), loc @ _rot2(yaw).T + [x, y]])
            r = max(r, float(np.linalg.norm(fp - [x, y], axis=1).max()))
        if check:
            why = None
            if not self.piles.outside(fp):
                why = "pile"
            elif self.keepout(fp):
                why = "staging"
            elif not self.drone_ok(x, y, r, z + top, flat=kind in FLAT):
                why = "drone"
            if why:
                self.dropped[why] = self.dropped.get(why, 0) + 1
                return None
        it = dict(kind=kind, var=int(var), pos=(float(x), float(y), float(z)), yaw=float(yaw), rx=float(rx),
                  ry=float(ry), scale=tuple(float(v) for v in s), r=float(r), top=float(z + top), mat=mat)
        self.items.append(it)
        if occupy:
            self.occupied.append((x, y, r * 0.8))
        return it


def _nvar(kind, dims):
    n = sum(1 for k in dims if k.rsplit("_", 1)[0] == kind)
    return n or {"car": len(CARS), "truck": 1, "wreck": 2, "pole": 2, "light": 2, "tree": 3, "wall": 6, "gate": 2,
                 "stub": 3, "debris": 4, "sheet": 3, "litter": 3, "manhole": 1, "post": 1}.get(kind, 1)


def plan(sc, seed=7, tracks=None, eyes=None, kit=None, log=None):
    """Layout and placements for the grid site (pure numpy). Returns a dict: layout, items (instances), wires,
    tape, decals, report (counts, dropped by reason, track / eye sources). Deterministic per seed.

    tracks: None = proxy legs + the recorded take for this seed if present; else polylines [(N,3)] or (N,3)
    points (only these are used). eyes: [(eye, look)] camera samples (None = proxy + recorded)."""
    if getattr(sc, "site", "grid") != "grid" or len(sc.piles) != 9:
        return {"items": [], "wires": [], "tape": [], "decals": {}, "layout": None,
                "report": {"skipped": f"town layout is for the 3x3 grid site (site {getattr(sc, 'site', 'grid')})"}}
    src = []
    if tracks is None:
        rt, re_ = recorded(seed)
        tracks = proxy_tracks(sc) + (rt or [])
        src = ["proxy legs"] + (["recorded take"] if rt else [])
        if eyes is None:
            eyes = proxy_eyes(sc) + (re_ or [])
    else:
        src = ["given"]
        if eyes is None:
            _, re_ = recorded(seed)
            eyes = proxy_eyes(sc) + (re_ or [])
    dims = {}
    kd = kit.dims if kit is not None else _kit_dims()
    dims = {k: (v["r"], v["top"]) for k, v in kd.items() if "r" in v}
    boxes = {k: (tuple(v["lo"][:2]), tuple(v["hi"][:2])) for k, v in kd.items() if "lo" in v}
    P = _Placer(sc, seed, tracks, eyes, dims, boxes)
    _buildings(P)
    _lots(P)
    _streets(P)
    _square(P)
    rep = {"tracks": src, "track_points": int(len(P.track_pts)), "eyes": len(P.eyes), "dropped": P.dropped,
           "counts": {}}
    for it in P.items:
        rep["counts"][it["kind"]] = rep["counts"].get(it["kind"], 0) + 1
    rep["wires"] = len(P.wires)
    out = {"seed": seed, "layout": P.L, "items": P.items, "wires": P.wires, "tape": P.tape,
           "decals": _decals(P), "report": rep, "ragged": P.ragged if hasattr(P, "ragged") else None}
    if log:
        log(f"[town] plan: {rep['counts']}, {len(P.wires)} wires, dropped {P.dropped}")
    return out


# --- buildings in the ring blocks
def _bspec(i, dims):
    s = dict(BLDG[i % len(BLDG)])
    d = dims.get(f"bldg_{i}")
    s["H"] = d[1] if d else bldg_height(s)
    return s


def _height_cap(P, corners):
    """Max building top (m) at a footprint: 4 m below every camera eye over or next to it, 1.5 m under every
    camera sight line (eye -> look) that crosses it."""
    if not hasattr(P, "_E"):
        E = np.array([e for e, _ in P.eyes], float).reshape(-1, 3)
        Lk = np.array([l for _, l in P.eyes], float).reshape(-1, 3)
        s = np.linspace(0.0, 1.0, 41)[None, :, None]
        P._E = E
        P._S = (E[:, None, :] + (Lk - E)[:, None, :] * s).reshape(-1, 3)
    cap = math.inf
    lo, hi = corners.min(0) - 4.0, corners.max(0) + 4.0
    E = P._E
    m = (E[:, 0] >= lo[0]) & (E[:, 0] <= hi[0]) & (E[:, 1] >= lo[1]) & (E[:, 1] <= hi[1])
    if m.any():
        cap = min(cap, float(E[m, 2].min()) - 4.0)
    S = P._S
    m = (S[:, 0] >= lo[0] + 3) & (S[:, 0] <= hi[0] - 3) & (S[:, 1] >= lo[1] + 3) & (S[:, 1] <= hi[1] - 3)
    if m.any():
        Q = S[m]
        inside = np.ones(len(Q), bool)
        K = len(corners)
        for k in range(K):
            a, b = corners[k], corners[(k + 1) % K]
            inside &= (b[0] - a[0]) * (Q[:, 1] - a[1]) - (b[1] - a[1]) * (Q[:, 0] - a[0]) >= -1.0
        if inside.any():
            cap = min(cap, float(Q[inside, 2].min()) - 1.5)
    return cap


def _buildings(P):
    rng, L = P.rng, P.L
    n = len(BLDG)
    specs = [_bspec(i, P.dims) for i in range(n)]
    order = rng.permutation(n)
    k = 0
    last_var, last_tint = -1, -1
    for b in L.blocks:
        if b["kind"] != "ring":
            continue
        x0, y0, x1, y1 = b["lot"]
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        # sides by priority: facing the grid first, its opposite next, then the other two between them
        sides = []
        for name, (nx, ny) in (("W", (-1, 0)), ("E", (1, 0)), ("S", (0, -1)), ("N", (0, 1))):
            if b["sides"][name] <= 0:
                continue                                       # outer edge of the town: no street
            toward = -(nx * cx + ny * cy) / (math.hypot(cx, cy) + 1e-6)
            sides.append((toward, name, nx, ny))
        sides.sort(reverse=True)
        taken = {"W": 0.0, "E": 0.0, "S": 0.0, "N": 0.0}       # depth used at each side (for the corners)
        depth_x, depth_y = x1 - x0, y1 - y0
        for _, name, nx, ny in sides:
            along_x = ny != 0                                  # frontage runs along x for S / N sides
            if along_x:
                a0, a1 = x0 + taken["W"], x1 - taken["E"]
                room = depth_y - taken["S" if name == "N" else "N"]
            else:
                a0, a1 = y0 + taken["S"], y1 - taken["N"]
                room = depth_x - taken["W" if name == "E" else "E"]
            if a1 - a0 < 6.0 or room < 6.0:
                continue
            pos, maxd = a0 + rng.uniform(0.0, 0.6), 0.0
            while a1 - pos >= 6.0:
                tries = 0
                while True:
                    var = int(order[k % n])
                    k += 1
                    tries += 1
                    if var != last_var or tries > 3:
                        break
                sp = specs[var]
                W = sp["W"] * rng.uniform(0.92, 1.08)
                rest = a1 - pos
                if rest - W < 6.0:                             # last one fills the frontage (scaled +-25 %)
                    W = rest - rng.uniform(0.0, 0.8) if rest <= sp["W"] * 1.3 else W
                if W > rest:
                    W = rest
                sxs = W / sp["W"]
                if not 0.7 <= sxs <= 1.35:
                    pos += W + 0.5
                    continue
                D = min(sp["D"], room / 2 + 1.0 if len(sides) > 1 else room - 1.0, room - 0.5)
                sys_ = D / sp["D"]
                mid = pos + W / 2
                if along_x:
                    fx, fy = mid, (y0 if name == "S" else y1)
                else:
                    fx, fy = (x0 if name == "W" else x1), mid
                yaw = math.degrees(math.atan2(ny, nx)) + 90.0       # module -y (facade) -> outward normal
                R = _rot2(yaw)
                loc = np.array([[-W / 2, 0], [W / 2, 0], [W / 2, D], [-W / 2, D]])
                corners = loc @ R.T + [fx, fy]
                cap = _height_cap(P, corners)
                cdz = P.ceil.min_rect(corners) - CLEAR_V
                cap = min(cap, cdz)
                ok = sp["H"] <= cap
                if not ok:                                     # pick a lower variant that fits the frontage
                    low = [v for v in range(n) if specs[v]["H"] <= cap and 0.75 <= W / specs[v]["W"] <= 1.3
                           and v != last_var]
                    if low:
                        var = int(rng.choice(low))
                        sp = specs[var]
                        sxs = W / sp["W"]
                        D = min(sp["D"], D / sys_ * 1.0) if sys_ > 0 else sp["D"]
                        D = min(sp["D"], room / 2 + 1.0 if len(sides) > 1 else room - 1.0, room - 0.5)
                        sys_ = D / sp["D"]
                        ok = True
                if ok:
                    zb = min(P.level(*c) for c in corners) - 0.02
                    tint = int(rng.choice([t for t in bldg_tints(var) if t != last_tint]))
                    it = P.add("bldg", var, fx, fy, yaw, z=zb, s=(sxs, sys_, 1.0), r=0.0, top=sp["H"],
                               fp=corners, mat=tint, check=False, occupy=False)
                    it["fp"] = corners.tolist()
                    last_var, last_tint = var, tint
                    maxd = max(maxd, D)
                    if sp["damage"] in ("corner", "soft", "facade", "top") and rng.random() < 0.8:
                        _front_debris(P, fx, fy, yaw, W, nx, ny)
                else:
                    P.dropped["bldg_height"] = P.dropped.get("bldg_height", 0) + 1
                pos += W + (0.0 if rng.random() < 0.7 else rng.uniform(0.4, 1.6))
            taken[name] = maxd


def _front_debris(P, fx, fy, yaw, W, nx, ny):
    """Fallen facade bits on the sidewalk / kerb in front of a damaged building."""
    rng = P.rng
    for _ in range(int(rng.integers(1, 4))):
        t = rng.uniform(-0.45, 0.45) * W
        R = _rot2(yaw)
        q = R @ np.array([t, 0.0]) + [fx, fy] + np.array([nx, ny]) * rng.uniform(0.6, 2.6)
        var = int(rng.integers(_nvar("debris", P.dims)))
        k = rng.uniform(0.7, 1.3)
        P.add("debris", var, q[0], q[1], rng.uniform(0, 360), s=(k, k, k * rng.uniform(0.7, 1.1)))


# --- pile lots: perimeter walls, gates, wall stubs, yard bits
def _lots(P):
    rng = P.rng
    nwall = _nvar("wall", P.dims)
    for b in P.L.blocks:
        if b["kind"] != "pile":
            continue
        x0, y0, x1, y1 = b["lot"]
        style = int(rng.integers(nwall))
        hs = rng.uniform(0.85, 1.1)
        gate_side = rng.choice(["W", "E", "S", "N"])
        for name in ("W", "E", "S", "N"):
            if name in "SN":
                a0, a1, c = x0, x1, (y0 if name == "S" else y1)
            else:
                a0, a1, c = y0, y1, (x0 if name == "W" else x1)
            Lside = a1 - a0
            nseg = max(1, int(round(Lside / 3.0)))
            seg = Lside / nseg
            yaw = 0.0 if name in "SN" else 90.0
            gate_at = int(rng.integers(nseg)) if name == gate_side else -1
            for s in range(nseg):
                m = a0 + (s + 0.5) * seg
                x, y = (m, c) if name in "SN" else (c, m)
                inward = {"W": (0.13, 0), "E": (-0.13, 0), "S": (0, 0.13), "N": (0, -0.13)}[name]
                x, y = x + inward[0], y + inward[1]
                ux, uy = (1.0, 0.0) if name in "SN" else (0.0, 1.0)
                fp = np.array([[x - ux * seg / 2, y - uy * seg / 2], [x + ux * seg / 2, y + uy * seg / 2], [x, y]])
                if not P.piles.outside(fp, pad=0.3):
                    continue
                if s == gate_at:
                    P.add("gate", int(rng.integers(_nvar("gate", P.dims))), x, y, yaw + (180 if rng.random() < 0.5 else 0),
                          s=(seg / 3.6, 1.0, rng.uniform(0.9, 1.05)), fp=fp, r=seg / 2)
                    continue
                if rng.random() < 0.18:
                    continue                                   # fallen section
                var = style if rng.random() < 0.6 else int(rng.integers(nwall))
                P.add("wall", var, x, y, yaw + (180 if rng.random() < 0.5 else 0),
                      s=(seg / 3.0, 1.0, hs * rng.uniform(0.92, 1.08)), fp=fp, r=seg / 2)
        # ground-storey wall stubs just outside the footprint, toward the lot corners
        pi = P.piles.names.index(b["pile"])
        px, py, sx, sy = P.piles.P[pi]
        for _ in range(int(rng.integers(1, 3))):
            for _try in range(12):
                cx_, cy_ = rng.choice([x0 + 1.5, x1 - 1.5]), rng.choice([y0 + 1.5, y1 - 1.5])
                ang = math.atan2(cy_ - py, cx_ - px) + rng.normal(0, 0.25)
                rr = PILE_K + 1.15 / min(sx, sy) + rng.uniform(0.0, 0.12)    # module bounds: rubble +-1 m
                x, y = px + rr * sx * math.cos(ang), py + rr * sy * math.sin(ang)
                if not (x0 + 0.3 < x < x1 - 0.3 and y0 + 0.3 < y < y1 - 0.3):
                    continue
                yaw = math.degrees(ang) + 90 + rng.normal(0, 12)
                R = _rot2(yaw)
                fp = np.array([[-1.8, -0.3], [1.8, -0.3], [1.8, 0.3], [-1.8, 0.3]]) @ R.T + [x, y]
                if P.free(x, y, 1.8) and P.add("stub", int(rng.integers(_nvar("stub", P.dims))), x, y, yaw, fp=fp,
                                               r=1.9, s=(1.0, 1.0, rng.uniform(0.75, 1.0))):
                    break
        # yard litter, sheets, a debris pile at the footprint edge
        for kind, n in (("sheet", int(rng.integers(1, 3))), ("litter", int(rng.integers(1, 3))),
                        ("debris", int(rng.integers(0, 2)))):
            for _ in range(n):
                for _try in range(8):
                    ang = rng.uniform(0, 2 * np.pi)
                    rr = PILE_K * 1.04 + rng.uniform(0.0, 0.5)
                    x, y = px + rr * sx * math.cos(ang), py + rr * sy * math.sin(ang)
                    if not (x0 < x < x1 and y0 < y < y1):
                        continue
                    if P.add(kind, int(rng.integers(_nvar(kind, P.dims))), x, y, rng.uniform(0, 360),
                             rx=rng.normal(0, 4) if kind == "sheet" else 0.0, occupy=False):
                        break


# --- streets: parked cars, trucks, poles + wires, street lights, trees, manholes, litter
def _streets(P):
    rng, L = P.rng, P.L
    segs = L.street_segments()
    ncar = _nvar("car", P.dims)
    car_names = []
    poles_by_seg = []
    for si, (axis, c, lo, hi, car, kind) in enumerate(segs):
        length = hi - lo
        if length < 8:
            continue
        u = np.array([0.0, 1.0]) if axis == "x" else np.array([1.0, 0.0])      # along the street
        v = np.array([1.0, 0.0]) if axis == "x" else np.array([0.0, 1.0])      # across
        cpt = (lambda s, t: (np.array([c, 0.0]) if axis == "x" else np.array([0.0, c])) + u * s + v * t)
        near_grid = abs(c) < 40 and (lo < 40 and hi > -40)
        # parked cars along both kerbs
        p_car = {"inner": 0.34, "main": 0.42, "ring": 0.36}[kind]
        for side in (-1, 1):
            s = lo + rng.uniform(2.5, 5.0)
            while s < hi - 3.0:
                if rng.random() < p_car:
                    big = rng.random() < (0.06 if kind != "inner" else 0.02)
                    if big:
                        Lc, var, knd = TRUCK["L"], 0, "truck"
                    else:
                        var = int(rng.integers(ncar))
                        Lc, knd = CARS[var % len(CARS)]["L"] if var < len(CARS) else 4.5, "car"
                    off = car / 2 - 1.05 - rng.uniform(0.0, 0.25)
                    askew = rng.random() < 0.12
                    yaw0 = (0.0 if axis == "y" else 90.0) + (0 if side * rng.choice([-1, 1]) > 0 else 180)
                    yaw = yaw0 + (rng.normal(0, 14) if askew else rng.normal(0, 1.5))
                    if askew:
                        off -= rng.uniform(0.3, 1.6)
                    q = cpt(s + Lc / 2, side * off)
                    R = _rot2(yaw)
                    half = np.array([Lc / 2, 1.0])
                    fp = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1], [0, 0]]) * half @ R.T + q
                    crushed = False
                    if knd == "car" and not P.piles.outside(fp, k=PILE_K, pad=2.2) and rng.random() < 0.6:
                        crushed = True
                    if P.free(q[0], q[1], Lc / 2):
                        if crushed:
                            it = P.add("wreck", int(rng.integers(_nvar("wreck", P.dims))), q[0], q[1], yaw, fp=fp,
                                       r=Lc / 2, mat=int(rng.choice(len(CAR_COLOURS), p=_car_p())))
                        else:
                            it = P.add(knd, var, q[0], q[1], yaw, fp=fp, r=Lc / 2,
                                       mat=int(rng.choice(len(CAR_COLOURS), p=_car_p())))
                        if it:
                            car_names.append(it)
                    s += Lc + rng.uniform(0.8, 2.5)
                else:
                    s += rng.uniform(4.5, 8.0)
        # utility poles on one side, wires between them; street lights on the other side of main streets
        side = 1 if int(round(c)) % 2 else -1
        ws = (9.0 - 6.0) / 2 if kind != "main" else 2.0
        spacing = rng.uniform(24.0, 31.0)
        poles = []
        s = lo + rng.uniform(3.0, 9.0)
        while s < hi - 3.0:
            q = cpt(s, side * (car / 2 + 0.45))
            lean = rng.normal(0, 2.5)
            if rng.random() < 0.18:
                lean = rng.choice([-1, 1]) * rng.uniform(7, 16)
            lean2 = rng.normal(0, 2.0)
            yaw = (0.0 if axis == "x" else 90.0) + rng.normal(0, 3)
            var = int(rng.integers(_nvar("pole", P.dims)))
            it = P.add("pole", var, q[0], q[1], yaw, rx=lean, ry=lean2, r=0.8)
            if it:
                poles.append(it)
            s += spacing * rng.uniform(0.85, 1.15)
        poles_by_seg.append(((axis, c, side), poles))
        if kind in ("main", "ring") or rng.random() < 0.5:
            s = lo + rng.uniform(8.0, 16.0)
            while s < hi - 4.0:
                q = cpt(s, -side * (car / 2 + 0.5))
                lean = rng.normal(0, 2.0) + (rng.choice([-1, 1]) * rng.uniform(6, 14) if rng.random() < 0.15 else 0)
                yaw = (90.0 if axis == "y" else 0.0) + (90 if -side > 0 else -90) + (0 if axis == "x" else 0)
                P.add("light", int(rng.integers(_nvar("light", P.dims))), q[0], q[1],
                      math.degrees(math.atan2(*(-side * v)[::-1])), rx=lean * 0.3, ry=lean, r=1.6)
                s += rng.uniform(26.0, 36.0)
        # street trees on the wide sidewalks (main and ring streets), none over the heaps (checked)
        if kind != "inner":
            s = lo + rng.uniform(4.0, 10.0)
            while s < hi - 3.0:
                if rng.random() < (0.5 if kind == "main" else 0.32):
                    t = -side * (car / 2 + ws * 0.55)
                    q = cpt(s, t)
                    var = int(rng.integers(_nvar("tree", P.dims)))
                    k = rng.uniform(0.8, 1.12)
                    if P.free(q[0], q[1], 1.5):
                        P.add("tree", var, q[0], q[1], rng.uniform(0, 360), s=(k, k, k * rng.uniform(0.9, 1.1)),
                              r=2.4 * k, rx=rng.normal(0, 2), ry=rng.normal(0, 2))
                s += rng.uniform(9.0, 15.0)
        # manholes and street litter (flat)
        s = lo + rng.uniform(5, 20)
        while s < hi - 2:
            q = cpt(s, rng.normal(0, car * 0.15))
            P.add("manhole", 0, q[0], q[1], rng.uniform(0, 360), occupy=False)
            s += rng.uniform(25, 45)
        for _ in range(int(rng.poisson(length / (18.0 if near_grid else 40.0)))):
            q = cpt(rng.uniform(lo, hi), rng.uniform(-car / 2, car / 2) + (rng.choice([-1, 1]) * (car / 2 + 0.6)
                                                                            if rng.random() < 0.3 else 0))
            kind2 = "litter" if rng.random() < 0.6 else "sheet"
            P.add(kind2, int(rng.integers(_nvar(kind2, P.dims))), q[0], q[1], rng.uniform(0, 360),
                  rx=rng.normal(0, 3) if kind2 == "sheet" else 0.0, occupy=False)
    # wires: consecutive poles along a street line and side (catenaries, sag grows with lean)
    lines = {}
    for key, poles in poles_by_seg:
        lines.setdefault(key, []).extend(poles)
    for (axis, c, side), poles in lines.items():
        poles.sort(key=lambda it: it["pos"][1] if axis == "x" else it["pos"][0])
        for a, b in zip(poles[:-1], poles[1:]):
            if np.hypot(a["pos"][0] - b["pos"][0], a["pos"][1] - b["pos"][1]) < 40.0:
                _wire_span(P, a, b)


def _car_p():
    p = np.array([0.24, 0.2, 0.14, 0.08, 0.05, 0.1, 0.08, 0.05, 0.06])
    return p / p.sum()


POLE_ARMS = (-0.62, 0.0, 0.62)                      # insulator offsets along the crossarm (module y), top z
POLE_TOP = 7.25


def _pole_attach(it, k):
    """World point of insulator k on a placed pole (module: crossarm along y at z = POLE_TOP)."""
    R = rot_zyx(it["yaw"], it["rx"], it["ry"])
    s = np.asarray(it["scale"])
    return np.asarray(it["pos"]) + R @ (np.array([0.0, POLE_ARMS[k], POLE_TOP + 0.12]) * s)


def _wire_span(P, a, b):
    rng = P.rng
    pa = [_pole_attach(a, k) for k in range(3)]
    pb = [_pole_attach(b, k) for k in range(3)]
    # match the crossarm ends that are closest (arms may face opposite ways)
    if np.linalg.norm(pa[0] - pb[0]) > np.linalg.norm(pa[0] - pb[2]):
        pb = pb[::-1]
    lean = max(abs(a["rx"]), abs(b["rx"]))
    for k in range(3):
        if k == 1 and rng.random() < 0.5:
            continue
        A, B = pa[k], pb[k]
        span = np.linalg.norm(B[:2] - A[:2])
        sag = 0.012 * span * rng.uniform(0.8, 1.4) + (0.06 * lean if lean > 5 else 0.0)
        t = np.linspace(0, 1, max(9, int(span / 0.75)))
        pts = A + (B - A) * t[:, None]
        pts[:, 2] -= sag * 4 * t * (1 - t)
        # clearance: every wire point (0.75 m steps) >= 3 m below the drones within 3 m
        ok = all(P.ceil.min_in(x, y, 0.0) - z >= CLEAR_V for x, y, z in pts)
        ok = ok and P.piles.outside(pts[:, :2])
        if ok:
            P.wires.append(pts[np.unique(np.linspace(0, len(pts) - 1, max(7, int(span / 3.5))).round().astype(int))])
        else:
            P.dropped["wire"] = P.dropped.get("wire", 0) + 1


# --- the open square: parked cars in bays at the sides, cordon tape around the staging area
def _square(P):
    rng = P.rng
    x0, y0, x1, y1 = -30.0, -59.0, 30.0, -40.5
    ncar = _nvar("car", P.dims)
    for xa, xb in ((x0 + 1.0, -14.0), (24.0, x1 - 1.0)):
        for row_y, yaw in ((y0 + 3.0, 90.0), (y1 - 3.0, -90.0)):
            x = xa
            while x < xb - 1.2:
                if rng.random() < 0.55:
                    var = int(rng.integers(ncar))
                    q = (x + rng.normal(0, 0.12), row_y + rng.normal(0, 0.25))
                    yy = yaw + rng.normal(0, 4) + (180 if rng.random() < 0.3 else 0)
                    R = _rot2(yy)
                    fp = np.array([[-2.3, -1.0], [2.3, -1.0], [2.3, 1.0], [-2.3, 1.0]]) @ R.T + q
                    P.add("car", var, q[0], q[1], yy, fp=fp, r=2.3, mat=int(rng.choice(len(CAR_COLOURS), p=_car_p())))
                x += 2.6
    # rescue cordon: posts with tape along the corridor sides
    for xs in (-13.0, 23.0):
        posts = []
        for y in np.arange(-58.0, -39.0, 3.2):
            yy = y + rng.normal(0, 0.15)
            it = P.add("post", 0, xs + rng.normal(0, 0.12), yy, rng.uniform(0, 360), r=0.3, occupy=False)
            if it:
                posts.append(it)
        for a, b in zip(posts[:-1], posts[1:]):
            A = np.asarray(a["pos"]) + [0, 0, 0.95]
            B = np.asarray(b["pos"]) + [0, 0, 0.95]
            t = np.linspace(0, 1, 6)[:, None]
            pts = A + (B - A) * t
            pts[:, 2] -= 0.12 * 4 * t[:, 0] * (1 - t[:, 0])
            P.tape.append(pts)


# --- decals: faded markings, patches, cracks
def _decals(P):
    rng, L = P.rng, P.L
    marks, patches, cracks = [], [], []
    for axis, c, lo, hi, car, kind in L.street_segments():
        u = np.array([0.0, 1.0]) if axis == "x" else np.array([1.0, 0.0])
        v = np.array([1.0, 0.0]) if axis == "x" else np.array([0.0, 1.0])
        o = np.array([c, 0.0]) if axis == "x" else np.array([0.0, c])
        s = lo + 2.0 if kind == "main" else hi                  # side streets: no markings
        while s < hi - 4.0:                                     # dashed centre line, faded and broken
            ln = 3.0 * rng.uniform(0.55, 1.0)
            if rng.random() > 0.15:
                a = o + u * s
                b = o + u * (s + ln)
                pts = np.array([a, b])
                rho, _ = P.piles.rho(pts[:, 0], pts[:, 1])
                if rho.min() > 1.55:
                    marks.append((a, b, 0.11))
            s += 8.0
        if kind == "main":                                      # edge lines (more worn)
            for side in (-1, 1):
                s = lo + 1.0
                while s < hi - 1.0:
                    ln = rng.uniform(2.0, 9.0)
                    if rng.random() < 0.55:
                        a = o + u * s + v * side * (car / 2 - 0.35)
                        b = o + u * min(s + ln, hi - 1) + v * side * (car / 2 - 0.35)
                        rho, _ = P.piles.rho(np.array([a[0], b[0]]), np.array([a[1], b[1]]))
                        if rho.min() > 1.5:
                            marks.append((a, b, 0.10))
                    s += ln + rng.uniform(0.5, 6.0)
        for _ in range(int(rng.poisson((hi - lo) / 22.0))):     # asphalt patches (darker repairs)
            cc = o + u * rng.uniform(lo + 1, hi - 1) + v * rng.uniform(-car / 2 + 0.6, car / 2 - 0.6)
            w, h = rng.uniform(0.8, 2.6), rng.uniform(0.6, 1.8)
            yaw = rng.normal(0, 4) + (0 if axis == "y" else 90)
            patches.append((cc, w, h, yaw))
    # quake cracks across the streets near the heaps: jagged polylines
    for p in P.sc.piles:
        for _ in range(int(rng.integers(1, 3))):
            ang = rng.uniform(0, 2 * np.pi)
            r0 = PILE_K * 1.1
            start = np.array([p.x + r0 * p.sx * math.cos(ang), p.y + r0 * p.sy * math.sin(ang)])
            d = np.array([math.cos(ang), math.sin(ang)])
            d = d @ _rot2(rng.normal(0, 35)).T
            pts = [start]
            for _k in range(int(rng.integers(8, 18))):
                d = d @ _rot2(rng.normal(0, 22)).T
                pts.append(pts[-1] + d * rng.uniform(0.35, 0.9))
            cracks.append((np.array(pts), rng.uniform(0.025, 0.07)))
    return {"marks": marks, "patches": patches, "cracks": cracks}


# ------------------------------------------------------------------------------------------------ validation
def check(pl, sc, tracks, eyes=None, h=CLEAR_H, v=CLEAR_V):
    """Brute-force validation of a plan against track points (independent of the raster): returns a list of
    violations [(kind, why, detail)]."""
    pts = densify(tracks)
    piles = Piles(sc)
    bad = []
    for it in pl["items"]:
        x, y, z = it["pos"]
        fp = np.asarray(it.get("fp")) if it.get("fp") is not None else None
        if it["kind"] != "bldg":
            a = np.linspace(0, 2 * np.pi, 16, endpoint=False)
            fp = np.c_[x + it["r"] * np.cos(a), y + it["r"] * np.sin(a)]
        if not piles.outside(fp):
            bad.append((it["kind"], "pile", (x, y)))
        if _in_rect(fp[:, 0], fp[:, 1], STAGING).any() or _in_rect(fp[:, 0], fp[:, 1], VAN_AREA).any():
            bad.append((it["kind"], "staging", (x, y)))
        hr = CLEAR_H_FLAT if it["kind"] in FLAT else h
        if it["kind"] == "bldg":
            c = fp.mean(0)
            rad = np.linalg.norm(fp - c, axis=1).max()
            near = pts[np.hypot(pts[:, 0] - c[0], pts[:, 1] - c[1]) < rad + hr]
        else:
            near = pts[np.hypot(pts[:, 0] - x, pts[:, 1] - y) < it["r"] + hr]
        if len(near) and (near[:, 2] - it["top"]).min() < v:
            bad.append((it["kind"], "drone", (x, y, float((near[:, 2] - it["top"]).min()))))
    for w in pl["wires"]:
        for q in w:
            near = pts[np.hypot(pts[:, 0] - q[0], pts[:, 1] - q[1]) < h]
            if len(near) and (near[:, 2] - q[2]).min() < v:
                bad.append(("wire", "drone", tuple(q)))
                break
    return bad


# ------------------------------------------------------------------------------------------------ ground meshes
def _cells(bounds, levels):
    """Quadtree-ish lattice. levels: [(size, refine_fn)] from coarse to fine; a cell of size levels[i] is split
    into cells of size levels[i+1] where refine_fn_i(x0, y0, x1, y1) is True. Returns (N, 4, 2) corners
    (v0 (i,j), v1 (i+1,j), v2 (i+1,j+1), v3 (i,j+1)): fan-triangulated (0,1,2), (0,2,3) like the terrain mesh
    (fine cells on the terrain lattice are exactly parallel to it)."""
    x0, y0, x1, y1 = bounds
    s0 = levels[0][0]
    CX, CY = np.meshgrid(np.arange(x0, x1 - 1e-6, s0), np.arange(y0, y1 - 1e-6, s0))
    CX, CY, S = CX.ravel(), CY.ravel(), np.full(CX.size, s0)
    done = []
    for (size, fn), nxt in zip(levels[:-1], [lv[0] for lv in levels[1:]]):
        ref = fn(CX, CY, CX + size, CY + size)
        done.append((CX[~ref], CY[~ref], S[~ref]))
        k = int(round(size / nxt))
        d = np.arange(k) * nxt
        di, dj = np.meshgrid(d, d)
        CX = (CX[ref][:, None] + di.ravel()[None, :]).ravel()
        CY = (CY[ref][:, None] + dj.ravel()[None, :]).ravel()
        S = np.full(CX.size, nxt)
    done.append((CX, CY, S))
    X = np.concatenate([d[0] for d in done])
    Y = np.concatenate([d[1] for d in done])
    S = np.concatenate([d[2] for d in done])
    return np.stack([np.c_[X, Y], np.c_[X + S, Y], np.c_[X + S, Y + S], np.c_[X, Y + S]], 1), S


def _split(T, f):
    """Split triangles T (N,3,3) by the iso-line f = 0 of per-vertex values f (N,3) (linear along the edges).
    Returns (neg (M,3,3), pos (K,3,3), cuts (C,2,3), cut_pos_dir (C,2)): cut segments and, for each, the
    horizontal direction toward the positive side. Winding is kept."""
    s = f < 0
    cnt = s.sum(1)
    neg, pos = [T[cnt == 3]], [T[cnt == 0]]
    mix = (cnt == 1) | (cnt == 2)
    if not mix.any():
        return neg[0], pos[0], np.zeros((0, 2, 3)), np.zeros((0, 2))
    Tm, fm, sm = T[mix], f[mix], s[mix]
    lone_neg = sm.sum(1) == 1
    i0 = np.where(lone_neg, np.argmax(sm, 1), np.argmax(~sm, 1))
    i1, i2 = (i0 + 1) % 3, (i0 + 2) % 3
    r = np.arange(len(Tm))
    A, B, C = Tm[r, i0], Tm[r, i1], Tm[r, i2]
    fa, fb, fc = fm[r, i0], fm[r, i1], fm[r, i2]
    pab = A + (B - A) * (fa / (fa - fb))[:, None]
    pac = A + (C - A) * (fa / (fa - fc))[:, None]
    lone = np.stack([A, pab, pac], 1)
    quad = np.concatenate([np.stack([pab, B, C], 1), np.stack([pab, C, pac], 1)])
    qneg = np.concatenate([~lone_neg, ~lone_neg])
    neg += [lone[lone_neg], quad[qneg]]
    pos += [lone[~lone_neg], quad[~qneg]]
    mid = (pab + pac) / 2
    to_pos = np.where(lone_neg[:, None], (B + C) / 2 - mid, A - mid)[:, :2]
    return np.concatenate(neg), np.concatenate(pos), np.stack([pab, pac], 1), to_pos


def _fv(T, fn):
    P = T.reshape(-1, 3)
    return np.asarray(fn(P[:, 0], P[:, 1]), float).reshape(-1, 3)


def _weld(T):
    """Triangle soup (N,3,3) -> (P, F) with coincident vertices merged (1 mm)."""
    P = T.reshape(-1, 3)
    key = np.round(P * 1000.0).astype(np.int64)
    _, idx, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    F = inv.reshape(-1, 3)
    F = F[(F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])]     # slivers collapsed by the weld
    return P[idx].astype(np.float32), F.astype(np.int32)


def ground(sc, layout, seed=7, rag_seed=None):
    """Draped ground cover over the town extent (pure numpy, vectorized): {material: (P (N,3), F (M,3))} plus
    vertical faces (kerbs, lot steps: "face_kerb"). Classes: asphalt / sidewalk / kerb / lot / bed (crushed debris
    under and around each heap) / dust (dust layer spilling onto the streets). Cells: 0.5 m on the terrain lattice
    near the heaps (rough mound terrain), 1 m at kerb corners, 2 m elsewhere on the terrain, 4 m over the flat
    apron. Offsets grow with the local quad twist (0.5 m cells) or the terrain's deviation from the cell's plane
    (bigger cells), so the cover stays above the terrain whichever diagonal a renderer picks."""
    from isaac.site import bounds as site_bounds
    rng = np.random.default_rng(4200 + seed)
    piles = Piles(sc)
    rags_bed = [_ragged(rng) for _ in sc.piles]
    rags_dust = [_ragged(rng) for _ in sc.piles]
    bx0, by0, bx1, by1 = site_bounds(sc)
    tb = (bx0 - 6, by0 - 6, bx1 + 6, by1 + 6)                 # scene.build terrain bounds (0.5 m lattice)

    def f_region(x, y, base, rags, lobes=0.0):
        best = np.full(np.shape(x), 1e9)
        for (px, py, sx, sy), rag in zip(piles.P, rags):
            th = np.arctan2((y - py) / sy, (x - px) / sx)
            rho = np.hypot((x - px) / sx, (y - py) / sy)
            R = base * (1.0 + _rag(np.ravel(th), rag).reshape(np.shape(th)) * (1 + lobes))
            best = np.minimum(best, (rho - R) * (sx + sy) / 2)
        return best

    f_bed = lambda x, y: f_region(x, y, 1.26, rags_bed)
    f_dust = lambda x, y: f_region(x, y, 1.55, rags_dust, lobes=0.8)
    corners = []
    for b in layout.blocks:
        x0, y0, x1, y1 = b["kerb"]
        corners += [(x0, y0), (x1, y0), (x0, y1), (x1, y1)]
    corners = np.array(corners)

    def near_corner(x0, y0, x1, y1, pad):
        out = np.zeros(len(x0), bool)
        for qx, qy in corners:
            out |= (x0 < qx + pad) & (x1 > qx - pad) & (y0 < qy + pad) & (y1 > qy - pad)
        return out

    def near_pile(x0, y0, x1, y1, lim):
        rho = np.full(len(x0), 1e9)
        for cx, cy in ((x0, y0), (x1, y0), (x0, y1), (x1, y1), ((x0 + x1) / 2, (y0 + y1) / 2)):
            rho = np.minimum(rho, piles.rho(cx, cy)[0])
        return rho < lim

    def in_tb(x0, y0, x1, y1):
        return (x1 > tb[0]) & (x0 < tb[2]) & (y1 > tb[1]) & (y0 < tb[3])

    def in_near(x0, y0, x1, y1):
        return (x1 > NEAR[0]) & (x0 < NEAR[2]) & (y1 > NEAR[1]) & (y0 < NEAR[3])

    cells, size = _cells(EXTENT, [
        (8.0, in_near),
        (4.0, lambda *c: in_tb(*c) | near_corner(*c, 3.5)),
        (2.0, lambda *c: near_corner(*c, 3.0) | (in_tb(*c) & near_pile(*c, 2.4))),
        (1.0, lambda *c: in_tb(*c) & near_pile(*c, 2.1)),
        (0.5, None)])
    # vertex heights + lift: 0.5 m cells by the quad twist, bigger cells by the terrain's rise above their plane
    V = cells.reshape(-1, 2)
    key = np.round(V * 4).astype(np.int64)
    _, idx_u, inv = np.unique(key[:, 0] * 10_000_000 + key[:, 1], return_index=True, return_inverse=True)
    UV = V[idx_u]
    Zu = surface(sc, UV[:, 0], UV[:, 1])
    Z = Zu[inv].reshape(-1, 4)
    lift = np.where(size < 0.75, np.abs(Z[:, 0] + Z[:, 2] - Z[:, 1] - Z[:, 3]) * 0.5, 0.0)
    big = np.nonzero(size >= 0.75)[0]
    for s_ in (1.0, 2.0, 4.0, 8.0):
        sel = big[size[big] == s_]
        if not len(sel):
            continue
        k = int(round(s_ / 0.5))
        g = np.linspace(0.0, 1.0, k + 1)
        U, W = np.meshgrid(g, g)
        U, W = U.ravel(), W.ravel()
        for c0 in range(0, len(sel), 4000):
            cs = sel[c0:c0 + 4000]
            x = cells[cs, 0, 0][:, None] + U[None, :] * s_
            y = cells[cs, 0, 1][:, None] + W[None, :] * s_
            zt = surface(sc, x.ravel(), y.ravel()).reshape(x.shape)
            z0, z1, z2, z3 = (Z[cs, i][:, None] for i in range(4))
            zp = np.where(U >= W, z0 + (z1 - z0) * U + (z2 - z1) * W, z0 + (z2 - z3) * U + (z3 - z0) * W)
            lift[cs] = np.maximum(0.0, (zt - zp).max(1)) + 0.004
    extra_u = np.zeros(len(UV))
    np.maximum.at(extra_u, inv, np.repeat(lift, 4))
    Z = (Zu + extra_u)[inv].reshape(-1, 4)
    P3 = np.concatenate([cells, Z[..., None]], -1)
    T = np.concatenate([P3[:, [0, 1, 2]], P3[:, [0, 2, 3]]])
    near = np.concatenate([size, size]) < 7.9
    # partition: asphalt | block -> lot | sidewalk | kerb strip; then bed / dust overlays on every class.
    # Outside NEAR (behind the first ring of buildings): flat classes, no kerb strip, no vertical faces.
    faces = []                                                  # (cuts, to_pos dirs, lo, hi, face toward +?)
    soups = {}
    for Tn, full in ((T[near], True), (T[~near], False)):
        off = OFF if full else {"asphalt": OFF["asphalt"], "lot": OFF["lot"], "sidewalk": OFF["lot"], "kerb": OFF["lot"]}
        blk, asph, cut, dirp = _split(Tn, _fv(Tn, layout.kerb_sd))
        lot, rest, cut2, dirp2 = _split(blk, _fv(blk, layout.lot_sd))
        if full:
            faces += [(cut, dirp, OFF["asphalt"], OFF["kerb"], True), (cut2, dirp2, OFF["lot"], OFF["sidewalk"], False)]
            side, kerb, _, _ = _split(rest, _fv(rest, lambda x, y: layout.kerb_sd(x, y) + 0.16))
        else:
            side, kerb = rest, rest[:0]
        for base, Tb in (("asphalt", asph), ("lot", lot), ("sidewalk", side), ("kerb", kerb)):
            if not len(Tb):
                continue
            if full:
                bed, other, _, _ = _split(Tb, _fv(Tb, f_bed))
                dust, plain, _, _ = _split(other, _fv(other, f_dust)) if len(other) else (other, other, None, None)
            else:
                bed, dust, plain = Tb[:0], Tb[:0], Tb
            for mat, Tm, o in (("bed", bed, off[base] + EXTRA["bed"]), ("dust", dust, off[base] + EXTRA["dust"]),
                               (base, plain, off[base])):
                if len(Tm):
                    Tm = Tm.copy()
                    Tm[..., 2] += o
                    soups.setdefault(mat, []).append(Tm)
    res = {}
    for mat, lst in soups.items():
        Tm = np.concatenate(lst)
        a = np.cross(Tm[:, 1] - Tm[:, 0], Tm[:, 2] - Tm[:, 0])[:, 2]
        res[mat] = _weld(Tm[a > 1e-7])
    # vertical faces (kerb: facing the street; lot step: facing the lot)
    quads = []
    for cut, dirp, lo, hi, toward_pos in faces:
        if not len(cut):
            continue
        a, b = cut[:, 0], cut[:, 1]
        seg = (b - a)[:, :2]
        nrm = np.c_[seg[:, 1], -seg[:, 0]]                      # right-hand normal of a -> b (quad a,b,b',a')
        flip = (np.sum(nrm * dirp, 1) < 0) == toward_pos
        a2, b2 = np.where(flip[:, None], b, a), np.where(flip[:, None], a, b)
        q = np.stack([a2 + [0, 0, lo - 0.03], b2 + [0, 0, lo - 0.03], b2 + [0, 0, hi], a2 + [0, 0, hi]], 1)
        quads.append(q[np.linalg.norm(seg, axis=1) > 1e-4])
    if quads:
        Q = np.concatenate(quads)
        res["face_kerb"] = _weld(np.concatenate([Q[:, [0, 1, 2]], Q[:, [0, 2, 3]]]))
    return res


def far_ground(r=420.0):
    """Flat quads from the town extent out to r (beyond the apron's dirt): {material: (P, F)}."""
    x0, y0, x1, y1 = EXTENT
    rects = [(-r, -r, r, y0), (-r, y1, r, r), (-r, y0, x0, y1), (x1, y0, r, y1)]
    P, F = [], []
    for a, b, c, d in rects:
        o = len(P)
        P += [(a, b, 0.0), (c, b, 0.0), (c, d, 0.0), (a, d, 0.0)]
        F += [(o, o + 1, o + 2), (o, o + 2, o + 3)]
    return {"far": (np.array(P, np.float32), np.array(F, np.int32))}


def decal_meshes(sc, layout, decals, lattice_z):
    """Markings / patches / cracks as thin draped strips: {material: (P, F)}; lattice_z(x, y) = cover height."""
    out = {}

    def strip(mat, a, b, w, lift):
        a, b = np.asarray(a, float), np.asarray(b, float)
        L = np.linalg.norm(b - a)
        if L < 1e-3:
            return
        n = max(1, int(math.ceil(L / 1.0)))
        d = (b - a) / L
        nrm = np.array([-d[1], d[0]]) * w / 2
        t = np.linspace(0, 1, n + 1)[:, None]
        c = a + (b - a) * t
        side = np.concatenate([c - nrm, c + nrm])
        z = lattice_z(side[:, 0], side[:, 1]) + lift
        Pp, Ff = out.setdefault(mat, ([], []))
        o = sum(len(q) for q in Pp)
        Pp.append(np.c_[side, z])
        k = n + 1
        Ff.append(np.array([[o + i, o + i + 1, o + k + i + 1] for i in range(n)] +
                           [[o + i, o + k + i + 1, o + k + i] for i in range(n)]))

    for a, b, w in decals["marks"]:
        strip("marking", a, b, w, 0.008)
    for c, w, h, yaw in decals["patches"]:
        R = _rot2(yaw)
        a = R @ np.array([-w / 2, 0.0]) + c
        b = R @ np.array([w / 2, 0.0]) + c
        strip("patch", a, b, h, 0.005)
    for pts, w in decals["cracks"]:
        for a, b in zip(pts[:-1], pts[1:]):
            strip("crack", a, b, w * np.random.default_rng(int(abs(a[0] * 100))).uniform(0.6, 1.3), 0.007)
    return {m: (np.concatenate(P).astype(np.float32), np.concatenate(F).astype(np.int32)) for m, (P, F) in out.items()}


def tube_mesh(polylines, r=0.012, sides=4):
    """Thin tubes along polylines (wires, tape): (P, F)."""
    P, F = [], []
    for pts in polylines:
        pts = np.asarray(pts, float)
        o = len(P)
        for i, p in enumerate(pts):
            d = pts[min(i + 1, len(pts) - 1)] - pts[max(i - 1, 0)]
            d /= np.linalg.norm(d) + 1e-9
            a = np.cross(d, [0, 0, 1.0])
            if np.linalg.norm(a) < 1e-6:
                a = np.array([1.0, 0, 0])
            a /= np.linalg.norm(a)
            b = np.cross(d, a)
            for k in range(sides):
                t = 2 * np.pi * k / sides
                P.append(p + (a * math.cos(t) + b * math.sin(t)) * r)
        for i in range(len(pts) - 1):
            for k in range(sides):
                j = (k + 1) % sides
                q0, q1 = o + i * sides + k, o + i * sides + j
                q2, q3 = o + (i + 1) * sides + j, o + (i + 1) * sides + k
                F += [(q0, q1, q2), (q0, q2, q3)]
    return np.array(P, np.float32).reshape(-1, 3), np.array(F, np.int32).reshape(-1, 3)


def ribbon_mesh(polylines, w=0.05):
    """Vertical ribbons along polylines (cordon tape): (P, F)."""
    P, F = [], []
    for pts in polylines:
        o = len(P)
        for p in pts:
            P += [p + [0, 0, -w / 2], p + [0, 0, w / 2]]
        for i in range(len(pts) - 1):
            a = o + 2 * i
            F += [(a, a + 2, a + 3), (a, a + 3, a + 1)]
    return np.array(P, np.float32).reshape(-1, 3), np.array(F, np.int32).reshape(-1, 3)


# ------------------------------------------------------------------------------------------------ USD
GROUND_MATS = ("asphalt", "sidewalk", "kerb", "lot", "bed", "dust", "face_kerb", "far", "marking", "patch", "crack",
               "wire", "tape")


def _usd_mesh(stage, path, P, F, uv_scale=None, normals="flat"):
    from pxr import Gf, Sdf, UsdGeom, Vt
    m = UsdGeom.Mesh.Define(stage, path)
    P = np.ascontiguousarray(P, np.float32)
    F = np.ascontiguousarray(F, np.int32)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(P))
    m.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(F), 3, np.int32)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(F.reshape(-1)))
    m.CreateSubdivisionSchemeAttr("none")
    a, b, c = P[F[:, 0]], P[F[:, 1]], P[F[:, 2]]
    n = np.cross(b - a, c - a)
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-9)
    if normals == "smooth":
        vn = np.zeros_like(P)
        for k in range(3):
            np.add.at(vn, F[:, k], n)
        vn /= np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-9)
        m.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(vn.astype(np.float32)))
        m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    else:
        m.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.repeat(n, 3, 0).astype(np.float32)))
        m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    if uv_scale:
        st = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        if normals == "vface":
            pass
        # planar world UVs for flat ground, box-ish for vertical faces (use x+y along the face)
        nn = np.zeros_like(P)
        for k in range(3):
            np.add.at(nn, F[:, k], np.abs(n))
        vert = nn[:, 2] < 0.5 * np.linalg.norm(nn, axis=1)
        u = np.where(vert, (P[:, 0] + P[:, 1]) / uv_scale, P[:, 0] / uv_scale)
        v = np.where(vert, P[:, 2] / uv_scale * 4.0, P[:, 1] / uv_scale)
        st.Set(Vt.Vec2fArray.FromNumpy(np.c_[u, v].astype(np.float32)))
    m.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, P.min(0))), Gf.Vec3f(*map(float, P.max(0)))]))
    m.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.3, 0.29, 0.27)]))
    return m


def lattice_z_fn(sc, layout):
    """Cover height at (x, y): terrain + class offset (decals sit on it; 2 m cells are near-planar there)."""
    def f(x, y):
        x, y = np.asarray(x, float), np.asarray(y, float)
        k = layout.kerb_sd(x, y)
        lo = layout.lot_sd(x, y)
        off = np.where(k > 0, OFF["asphalt"], np.where(lo < 0, OFF["lot"], OFF["sidewalk"]))
        return surface(sc, x, y) + off + 0.012
    return f


def place(stage, root, kit, pl, log=print, sc=None):
    """Write the town under root: ground cover, decals, wires, tape and one PointInstancer per module kind.
    Returns {name: count}. Visual only: no colliders, no physics APIs."""
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt
    if not pl.get("items"):
        log(f"[town] nothing to place ({pl.get('report', {}).get('skipped', 'empty plan')})")
        return {}
    sc = sc or pl["layout"].sc
    UsdGeom.Xform.Define(stage, root)
    looks = stage.DefinePrim(f"{root}/Looks", "Scope")
    looks.GetReferences().AddReference(kit.usd, Sdf.Path("/TownKit/Looks"))
    counts = {}

    def bind(prim, name):
        mp = stage.GetPrimAtPath(f"{root}/Looks/{name}")
        if mp.IsValid():
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(UsdShade.Material(mp))

    # ground cover
    UsdGeom.Xform.Define(stage, f"{root}/Ground")
    G = ground(sc, pl["layout"], pl["seed"])
    G.update(far_ground())
    uvs = kit.meta.get("uv_tile", {})
    ntri = 0
    for mat, (P, F) in G.items():
        m = _usd_mesh(stage, f"{root}/Ground/{mat}", P, F, uv_scale=uvs.get(mat, 4.0))
        bind(m.GetPrim(), {"face_kerb": "kerb", "face_bed": "bed"}.get(mat, mat))
        ntri += len(F)
    counts["ground_tris"] = ntri
    # decals
    UsdGeom.Xform.Define(stage, f"{root}/Decals")
    D = decal_meshes(sc, pl["layout"], pl["decals"], lattice_z_fn(sc, pl["layout"]))
    for mat, (P, F) in D.items():
        m = _usd_mesh(stage, f"{root}/Decals/{mat}", P, F, uv_scale=uvs.get(mat, 2.0))
        bind(m.GetPrim(), mat)
        counts["decal_tris"] = counts.get("decal_tris", 0) + len(F)
    if pl["wires"]:
        P, F = tube_mesh(pl["wires"], 0.011, 3)
        m = _usd_mesh(stage, f"{root}/Wires", P, F, normals="smooth")
        bind(m.GetPrim(), "wire")
        counts["wire_tris"] = len(F)
    if pl["tape"]:
        P, F = ribbon_mesh(pl["tape"], 0.07)
        m = _usd_mesh(stage, f"{root}/Tape", P, F, uv_scale=1.0)
        m.CreateDoubleSidedAttr(True)
        bind(m.GetPrim(), "tape")
    # instances
    for kind in KINDS:
        sel = [it for it in pl["items"] if it["kind"] == kind]
        names = kit.variants.get(kind, [])
        if not sel or not names:
            continue
        ip = f"{root}/{kind}"
        inst = UsdGeom.PointInstancer.Define(stage, ip)
        rel = inst.CreatePrototypesRel()
        stage.DefinePrim(f"{ip}/Prototypes", "Scope")
        protos, idx = {}, []
        for it in sel:
            var = it["var"] % len(names)
            key = (names[var], it.get("mat"))
            if key not in protos:
                # prototype = the module's shell (instanceable: one copy whatever the tint) + its tinted part
                # (plaster / paint faces, own copy per tint with the colour override)
                m = names[var]
                nm = m + ("" if it.get("mat") is None else f"_m{it['mat']}")
                pp = stage.DefinePrim(f"{ip}/Prototypes/{nm}", "Xform")
                for part in ("shell", "tint"):
                    if part == "tint" and m not in kit.tinted:
                        continue
                    q = stage.DefinePrim(f"{ip}/Prototypes/{nm}/{part}", "Xform")
                    q.GetReferences().AddReference(kit.usd, Sdf.Path(f"/TownKit/{m}/{part}"))
                    q.SetInstanceable(part == "shell" or it.get("mat") is None)
                _override_material(stage, pp, kind, it.get("mat"))
                rel.AddTarget(pp.GetPath())
                protos[key] = len(protos)
            idx.append(protos[key])
        Pn = np.array([it["pos"] for it in sel], np.float32)
        Q = [quat_zyx(it["yaw"], it["rx"], it["ry"]) for it in sel]
        S = np.array([it["scale"] for it in sel], np.float32)
        inst.CreateProtoIndicesAttr(Vt.IntArray(idx))
        inst.CreatePositionsAttr(Vt.Vec3fArray.FromNumpy(Pn))
        inst.CreateOrientationsAttr(Vt.QuathArray([Gf.Quath(float(w), float(x), float(y), float(z)) for w, x, y, z in Q]))
        inst.CreateScalesAttr(Vt.Vec3fArray.FromNumpy(S))
        r = 25.0 if kind == "bldg" else 6.0
        inst.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, Pn.min(0) - r)), Gf.Vec3f(*map(float, Pn.max(0) + r))]))
        counts[kind] = len(sel)
    log(f"[town] placed under {root}: {counts}")
    return counts


def _override_material(stage, proto, kind, mat):
    """Per-prototype colour: plaster tint for buildings (albedo texture scale), paint for cars."""
    from pxr import Gf, Sdf
    if mat is None:
        return
    p = str(proto.GetPath())
    if kind == "bldg":
        r, g, b = PLASTER_TINTS[mat % len(PLASTER_TINTS)]
        o = stage.OverridePrim(f"{p}/tint/Looks/plaster/albedo")
        o.CreateAttribute("inputs:scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(r, g, b, 1.0))
    elif kind in ("car", "truck", "wreck"):
        rgb = CAR_COLOURS[mat % len(CAR_COLOURS)]
        o = stage.OverridePrim(f"{p}/tint/Looks/paint/PBR")
        o.CreateAttribute("inputs:diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        top = [0.45 * c + 0.55 * d for c, d in zip(rgb, (0.34, 0.32, 0.29))]      # roof / bonnet under dust
        o = stage.OverridePrim(f"{p}/tint/Looks/paint_top/PBR")
        o.CreateAttribute("inputs:diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*top))


def dress(stage, sc, folder, log=print, seed=None):
    """photoreal.dress hook: plan + place the town if the kit exists (never raises past the caller's try)."""
    kit = load(folder, log)
    if kit is None:
        return {}
    pl = plan(sc, sc.seed if seed is None else seed, kit=kit, log=log)
    return place(stage, "/World/Town", kit, pl, log, sc=sc)


# ------------------------------------------------------------------------------------------------ CLI
def record(seed=7, out=None, extra=()):
    """Run the take's dry run (run_mission --dry-run --live --camera director-take --acoustic stub) to the end,
    keeping every drone pose and the director camera's eye/look, and save tracks/grid_s<seed>.json (decimated)."""
    import sys
    import tempfile
    sys.path.insert(0, os.path.dirname(HERE))
    import sim.grid_mission as gm
    from isaac import camera as cam_mod
    poses, eyes = {}, []
    orig = gm.GridMission._drones_out

    def patched(self):
        for d in self.flight.poses():
            poses.setdefault(d["id"], []).append([float(v) for v in d["pose"][:3]] + [round(float(self.t), 2)])
        return orig(self)
    o_apply = cam_mod.DirectorCamera._apply

    def apply(self):
        eyes.append((self.eye.tolist(), self.target.tolist()))
        return o_apply(self)
    gm.GridMission._drones_out = patched
    cam_mod.DirectorCamera._apply = apply
    from isaac import run_mission
    tmp = tempfile.mkdtemp(prefix="town_rec_")
    try:
        run_mission.main(["--dry-run", "--live", "--camera", "director-take", "--acoustic", "stub", "--comms", "0",
                          "--live-pace", "clear", "--seed", str(seed), "--out", tmp, "--quiet", *extra])
    finally:
        gm.GridMission._drones_out = orig
        cam_mod.DirectorCamera._apply = o_apply
    # decimate: keep a pose when it moved > 0.4 m or every 10 s
    tr = {}
    for k, v in poses.items():
        v = np.asarray(v)
        keep = [0]
        for i in range(1, len(v)):
            if np.linalg.norm(v[i, :3] - v[keep[-1], :3]) > 0.4 or v[i, 3] - v[keep[-1], 3] > 10:
                keep.append(i)
        tr[k] = np.round(v[keep], 2).tolist()
    ev = []
    for e, l in eyes[::3]:
        ev.append([np.round(e, 1).tolist(), np.round(l, 1).tolist()])
    out = out or os.path.join(HERE, "assets", "town_kit", "tracks", f"grid_s{seed}.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump({"seed": seed, "site": "grid", "source": "run_mission --dry-run --live --camera director-take "
               "--acoustic stub --comms 0 --live-pace clear", "tracks": tr, "eyes": ev}, open(out, "w"))
    print(f"[town] recorded {sum(len(v) for v in tr.values())} drone poses, {len(ev)} camera samples -> {out}")
    return out


if __name__ == "__main__":
    import argparse
    import sys
    sys.path.insert(0, os.path.dirname(HERE))
    ap = argparse.ArgumentParser(description="town layout tools")
    sub = ap.add_subparsers(dest="cmd")
    r = sub.add_parser("record", help="record the take's drone tracks + camera for the clearance checks")
    r.add_argument("--seed", type=int, default=7)
    r.add_argument("--out", default="")
    p = sub.add_parser("plan", help="print the plan summary")
    p.add_argument("--seed", type=int, default=7)
    a = ap.parse_args()
    if a.cmd == "record":
        record(a.seed, a.out or None)
    elif a.cmd == "plan":
        from sim.scenario import make_scenario
        pl = plan(make_scenario(a.seed), a.seed, log=print)
        print(json.dumps(pl["report"], indent=1))
