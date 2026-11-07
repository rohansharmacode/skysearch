"""The rescue scene around the collapse (as in the reference photos of the real site): yellow excavators working
the rubble edge, a blue crawler crane with a lattice boom, red fire trucks with ladders on the street. Smooth
procedural meshes (isaac/drone_mesh.py primitives), visual only (no colliders), placed on the street level of the
map's terrain and away from the drones' flight paths. Generic designs, no branding.
"""
from __future__ import annotations

import math

import numpy as np

from isaac.drone_mesh import merge, superellipsoid, tube

ROOT = "/World/UrbanLook/Props"
YELLOW, BLUE, RED = (0.80, 0.58, 0.04), (0.06, 0.22, 0.62), (0.62, 0.04, 0.03)
DARK, STEEL, GLASS, WHITE = (0.05, 0.05, 0.05), (0.35, 0.36, 0.38), (0.02, 0.025, 0.03), (0.85, 0.85, 0.83)
# (kind, x, y, yaw deg): south street, rubble edges, east yard (drone paths go north from the staging base)
LAYOUT = (("excavator", -52.0, -2.0, 15.0), ("excavator", -4.0, -47.0, 95.0), ("excavator", 33.0, 6.0, 190.0),
          ("crane", 47.0, 2.0, 180.0), ("firetruck", -14.0, -60.5, 0.0), ("firetruck", 20.0, -60.0, 180.0))


def _box(c, hx, hy, hz, e=0.15):
    return superellipsoid(c, hx, hy, hz, e, e, nu=24, nv=8)


def excavator():
    """{material key: [(P, F), ...]} in metres, +x forward, ground at z = 0."""
    y, d, s, g = [], [], [], []
    for sd in (-1, 1):                                        # tracks with sprockets
        d.append(_box((0, sd * 1.15, 0.45), 2.3, 0.33, 0.45, 0.3))
        for x in (-1.9, 1.9):
            s.append(tube((x, sd * 0.8, 0.45), (x, sd * 1.5, 0.45), 0.42, sides=16))
    y.append(_box((-0.2, 0, 1.25), 1.6, 1.25, 0.32))          # turntable deck
    y.append(_box((-1.2, 0, 1.75), 0.9, 1.2, 0.5, 0.3))       # engine housing / counterweight
    y.append(_box((0.75, 0.55, 2.05), 0.55, 0.5, 0.8, 0.12))  # cab
    g.append(_box((1.31, 0.55, 2.25), 0.02, 0.42, 0.5, 0.1))  # cab windows
    g.append(_box((0.75, 1.06, 2.25), 0.45, 0.02, 0.45, 0.1))
    piv, elb, tip = np.array([0.9, -0.35, 1.7]), np.array([3.8, -0.35, 4.6]), np.array([5.6, -0.35, 1.4])
    y.append(tube(piv, elb, 0.26, sides=10))                  # boom
    y.append(tube(elb, tip, 0.2, sides=10))                   # stick
    s.append(tube(piv + [0.4, 0, -0.2], elb - [0.6, 0, 0.9], 0.08))   # hydraulic rams
    s.append(tube(elb + [0, 0, 0.3], elb + [1.2, 0, -0.9], 0.07))
    d.append(_box(tip + [0.25, 0, -0.35], 0.45, 0.55, 0.38, 0.35))   # bucket
    return {"yellow": y, "dark": d, "steel": s, "glass": g}


def crane(height=34.0, angle=62.0):
    b, d, s = [], [], []
    for sd in (-1, 1):
        d.append(_box((0, sd * 1.7, 0.55), 3.4, 0.45, 0.55, 0.3))
    b.append(_box((-0.6, 0, 1.55), 2.6, 1.6, 0.55))            # body
    b.append(_box((-2.4, 0, 2.3), 0.8, 1.5, 0.75, 0.3))        # counterweight
    b.append(_box((1.2, -1.15, 2.3), 0.7, 0.45, 0.7, 0.12))    # cab
    a = math.radians(angle)
    base = np.array([1.0, 0.0, 2.2])
    ax = np.array([math.cos(a), 0.0, math.sin(a)])
    L = height / math.sin(a)
    w = 0.55
    corners = [np.array([0, sy * w, sz * w]) for sy, sz in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    perp = np.array([-math.sin(a), 0.0, math.cos(a)])
    def pt(t, k):
        c = corners[k]
        return base + ax * t + np.array([0, c[1], 0]) + perp * c[2] * (1 - 0.5 * t / L)
    for k in range(4):
        b.append(tube(pt(0, k), pt(L, k), 0.07, sides=8))
    n = int(L / 1.6)
    for i in range(n):
        t0, t1 = i * L / n, (i + 1) * L / n
        for k in range(4):
            s.append(tube(pt(t0, k), pt(t1, (k + 1) % 4), 0.035, sides=6))
    tipp = base + ax * L
    s.append(tube(tipp, tipp - np.array([0, 0, height * 0.75]), 0.025, sides=6))     # hoist rope
    d.append(_box(tipp - np.array([0, 0, height * 0.75 + 0.5]), 0.35, 0.35, 0.5, 0.4))  # hook block
    return {"blue": b, "dark": d, "steel": s}


def firetruck():
    r, d, s, g, w = [], [], [], [], []
    r.append(_box((2.9, 0, 1.55), 1.0, 1.22, 1.05, 0.12))      # cab
    g.append(_box((3.86, 0, 1.95), 0.03, 1.05, 0.45, 0.1))     # windscreen
    r.append(_box((-0.9, 0, 1.6), 2.9, 1.25, 1.1, 0.1))        # body
    w.append(_box((-0.9, 0, 1.1), 2.92, 1.27, 0.06, 0.1))      # white stripe
    for x in (2.7, -0.5, -2.2):
        for sd in (-1, 1):
            d.append(tube((x, sd * 0.95, 0.5), (x, sd * 1.3, 0.5), 0.5, sides=16))
    for sd in (-1, 1):                                           # ladder rails + rungs
        s.append(tube((-3.6, sd * 0.45, 2.95), (2.4, sd * 0.45, 2.95), 0.06, sides=8))
    for k in range(16):
        x = -3.5 + k * 0.38
        s.append(tube((x, -0.45, 2.95), (x, 0.45, 2.95), 0.025, sides=6))
    r.append(_box((3.2, 0, 2.7), 0.25, 0.7, 0.07, 0.3))         # light bar
    return {"red": r, "dark": d, "steel": s, "glass": g, "white": w}


BUILDERS = {"excavator": excavator, "crane": crane, "firetruck": firetruck}
COLOURS = {"yellow": (YELLOW, 0.45, 0.0, 0.3), "blue": (BLUE, 0.45, 0.0, 0.3), "red": (RED, 0.4, 0.0, 0.5),
           "dark": (DARK, 0.8, 0.0, 0.0), "steel": (STEEL, 0.4, 0.8, 0.0), "glass": (GLASS, 0.05, 0.0, 1.0),
           "white": (WHITE, 0.4, 0.0, 0.3)}


def _ground(stage, x, y, site="/World/CollapseSite"):
    """Median terrain height within 3 m of (x, y) from the map's terrain tiles (streets sit at ~0)."""
    from pxr import UsdGeom
    tile = f"{site}/Terrain/terrain_r{int(np.clip((160 - y) // 80, 0, 3))}c{int(np.clip((x + 160) // 80, 0, 3))}"
    p = stage.GetPrimAtPath(tile)
    if not p.IsValid():
        return 0.0
    P = np.asarray(UsdGeom.Mesh(p).GetPointsAttr().Get(), float)
    m = np.hypot(P[:, 0] - x, P[:, 1] - y) < 3.0
    return float(np.median(P[m, 2])) if m.any() else 0.0


def add(stage, log=print, layout=LAYOUT):
    from pxr import Gf, UsdGeom
    from isaac import photoreal as PR
    from isaac.urban_look import _pv
    looks = {k: _pv(stage, f"{ROOT}/Looks/{k}", *v) for k, v in COLOURS.items()}
    n = 0
    for k, (kind, x, y, yaw) in enumerate(layout):
        root = f"{ROOT}/{kind}{k}"
        xf = UsdGeom.Xformable(UsdGeom.Xform.Define(stage, root))
        xf.ClearXformOpOrder()
        xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(x, y, _ground(stage, x, y)))
        xf.AddRotateZOp(UsdGeom.XformOp.PrecisionDouble).Set(float(yaw))
        for key, parts in BUILDERS[kind]().items():
            if not parts:
                continue
            P, F = merge(parts)
            m = PR._mesh(stage, f"{root}/{key}", P, F, COLOURS[key][0], smooth=True)
            m.CreateDoubleSidedAttr(True)
            PR.bind(stage, f"{root}/{key}", looks[key])
        n += 1
    log(f"[urban] rescue equipment: {n} props (yellow excavators, a blue crawler crane, red fire trucks); visual only")
    return n
