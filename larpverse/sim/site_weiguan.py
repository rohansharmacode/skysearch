"""Procedural stand-in for the Weiguan Jinlong collapse site (Yongkang District, Tainan, 6 Feb 2016).

``make_weiguan(seed, start_pile)`` returns a ``Scenario`` (same structure as the synthetic 3x3 grid in
sim/scenario.py), so GridWorld, the mission, the Isaac scene builder and the animation exporter can use it
unchanged. Select it with ``sim.scenario.make_scenario(seed, site="weiguan")``.

This is NOT a survey of the real wreck. The NCKU Museum photogrammetry model has no stated licence or
download, so the geometry is generated from public descriptions. Facts used (zh.wikipedia 維冠金龍大樓):
anchor 23.0051432 N, 120.2608904 E; 16 storeys + 1 basement; nine blocks A-I in a U shape facing west;
the building toppled from west to east across Yongda Road Section 2 ("horizontal collapse"); roughly a
third of the structure stayed above ground. Everything else is an ASSUMPTION (see docs/SITE_WEIGUAN.md):

* Site frame: metres, x east, y north, origin = the wiki anchor, taken as the centre of the original
  footprint. Original footprint ~40 m (E-W) x 60 m (N-S): a spine of five blocks C, D, E, F, G along the
  east side (x 6..20 m, 14 m deep) and two 14 m-deep wings to the west (north wing A, B; south wing H, I),
  so the U opens west. Block lettering runs round the U from the north-west tip: A B C | D E F | G H I.
  The real lettering of the blocks is not known to us; this mapping is a modelling choice.
* Storey pitch 2.9 m. Yongda Road Sec. 2 runs north-south immediately east of the spine, x 20.5..41.5
  (21 m kerb to kerb incl. sidewalks). Street level is z = 0 everywhere (flat).
* The spine pivoted about its east base edge and lies on its side across the road: floor slabs are now
  near-vertical and stacked eastwards like fallen books (storeys 1-3 crushed tight at the base, 4-13
  intact-ish with ~2.6 m room voids between slabs, 14-16 shattered beyond the far kerb). The former
  courtyard facade is now the top surface (broken panels at 10-14 m, some missing = openings into the
  room voids). Former N-S partitions are now horizontal shelves / lean-to wedges inside the rooms.
* The original spine footprint (x 6..20) keeps a crushed basement/ground-floor layer and sheared column
  stubs leaning east. The wings pancaked in place (5-7 layers, 0.6-1.2 m pancake voids), dragged and
  tilted towards the spine; heights 5-9 m. Debris spills across the road, round the ends and into the
  courtyard.
* Search sectors = the nine blocks, mapped onto where each block's debris now lies (the spine blocks'
  sectors are centred on the toppled stack over the road). Pieces belong to the nearest sector centre.
* Staging: on Yongda Road south of the debris (assumed closed to traffic), drone pads beside it.

Geometry uses a FIXED RNG (it is one real site); only the hidden truth (victims, sounds, start sector)
depends on ``seed``, through its own RNG streams. ``Scenario.truth`` stays HIDDEN GROUND TRUTH: only the
observation generators and evaluators may read it.
"""
from __future__ import annotations

import functools

import numpy as np

from .scenario import PHYSIO_STATES, Pile, Piece, Scenario, rot_matrix

SITE = "weiguan"
GEOM_SEED = 20160206
BOUNDS = (-32.0, -64.0, 68.0, 42.0)          # xmin, ymin, xmax, ymax (200 x 212 cells at 0.5 m)
BASE_XY = (31.0, -55.0)                      # staging on Yongda Road, south of the debris
DRONE_PADS = [(28.4, -57.5), (31.0, -57.5), (33.6, -57.5)]
SPACING = 12.0                               # nominal sector spacing (m)
GEO = {"lat": 23.0051432, "lon": 120.2608904, "bearing_deg": 0.0,
       "name": "Weiguan Jinlong collapse, Yongkang District, Tainan (2016)"}
STOREY = 2.9
PIVOT_X = 21.0                               # east base edge of the spine (+1 m kick) = where the lying stack starts
ROAD_X = (20.5, 41.5)
SECTORS = "ABCDEFGHI"
# original (standing) block footprints: x0, x1, y0, y1
BLOCKS = {
    "A": (-20.0, -7.0, 16.0, 30.0), "B": (-7.0, 6.0, 16.0, 30.0), "C": (6.0, 20.0, 16.0, 30.0),
    "D": (6.0, 20.0, 16 / 3, 16.0), "E": (6.0, 20.0, -16 / 3, 16 / 3), "F": (6.0, 20.0, -16.0, -16 / 3),
    "G": (6.0, 20.0, -30.0, -16.0), "H": (-7.0, 6.0, -30.0, -16.0), "I": (-20.0, -7.0, -30.0, -16.0),
}
SPINE = "CDEFG"
LYING_H = {"C": 11.0, "D": 13.0, "E": 13.8, "F": 12.4, "G": 10.2}      # top of the toppled stack (m)
PANCAKE_H = {"A": 5.2, "B": 8.6, "H": 8.0, "I": 4.8}                  # top of the pancaked wing (m)
# sector centres where each block's debris lies after the collapse
CENTRES = {"A": (-13.0, 23.0), "B": (-0.5, 23.0), "C": (36.0, 23.0), "D": (36.0, 10.7), "E": (36.0, 0.0),
           "F": (36.0, -10.7), "G": (36.0, -23.0), "H": (-0.5, -23.0), "I": (-13.0, -23.0)}

# --- hidden-truth vocabulary -----------------------------------------------------------------------
# One acoustic role per non-start sector (kinds the acoustic pipeline already knows).
ROLE_KINDS = ["voice", "voice", "voice", "weak_voice", "knock", "knock", "dog", "settling"]
LANGS = ["cmn", "nan", "en", "id"]
# TTS lines (truth: what the pipeline should recover). nan = Taiwanese Hokkien, Han + Tai-lo; the
# Hokkien lines are approximate, have them checked by a speaker before using them on camera.
VOICE_TEXT = {
    "cmn": {"voice": "救命！有人聽得到嗎？我的腳被壓住了。", "weak_voice": "有人嗎……水……"},
    "nan": {"voice": "救人喔！有人聽著無？我的跤予壓牢咧。", "weak_voice": "有人無……水……"},
    "en": {"voice": "Help! Can anyone hear me? My leg is trapped.", "weak_voice": "hello... is anyone there... water..."},
    "id": {"voice": "Tolong! Ada yang bisa dengar saya? Kaki saya terjepit.", "weak_voice": "halo... ada orang... air..."},
}
ROMANISED = {"nan": {"voice": "Kiù-lâng--ooh! Ū lâng thiann-tio̍h--bô? Guá ê kha hōo ah-tiâu--leh.",
                     "weak_voice": "Ū lâng--bô... tsuí..."},
             "cmn": {"voice": "Jiùmìng! Yǒu rén tīng de dào ma? Wǒ de jiǎo bèi yā zhù le.",
                     "weak_voice": "Yǒu rén ma... shuǐ..."}}
EN_GLOSS = {"voice": "Help! Can anyone hear me? My leg is trapped.", "weak_voice": "Anyone there... water..."}
TTS_LANG = {"cmn": "zh-TW", "nan": "nan-TW", "en": "en-US", "id": "id-ID"}
FLEURS = {"cmn": "cmn_hans_cn", "nan": None, "en": "en_us", "id": "id_id"}   # FLEURS has no Taiwanese Hokkien
DEPTH_CLASS = {"shallow": (0.5, 1.5), "mid": (1.5, 3.0), "deep": (4.0, 7.0)}
RADAR_REACH_M = 3.0


class WeiguanScenario(Scenario):
    """Scenario with its own bounds, staging and site metadata (everything else inherited)."""

    site = SITE
    bounds = BOUNDS
    base_xy = BASE_XY
    drone_pads = DRONE_PADS
    geo = GEO

    def heightmap(self, res=0.5, bounds=None):
        return super().heightmap(res, bounds or BOUNDS)

    def public_dict(self):
        d = super().public_dict()
        d.update({"site": SITE, "spacing": SPACING, "bounds": list(BOUNDS), "geo": dict(GEO),
                  "base_xy": list(BASE_XY), "drone_pads": [list(p) for p in DRONE_PADS],
                  "site_info": site_info()})
        from .site_mesh import piles_with_polygons
        return piles_with_polygons(d, self.piles)


def site_info():
    """Static context for maps (public, no truth): road, original footprint, collapse direction."""
    return {
        "name": GEO["name"], "case_study": "Weiguan Jinlong building, 16 storeys + 1 basement, toppled west to east",
        "sectors_are": "blocks A-I of the U-shaped building, mapped onto where their debris lies",
        "collapse_azimuth_deg": 0.0,                      # toppled towards +x (east), maths convention
        "road": {"name": "Yongda Road Sec. 2", "x": list(ROAD_X), "y": [BOUNDS[1], BOUNDS[3]]},
        "blocks_before": {k: [round(v, 3) for v in b] for k, b in BLOCKS.items()},
        "pivot_x": PIVOT_X, "storey_m": STOREY, "storeys": 16,
        "assumed": True, "doc": "docs/SITE_WEIGUAN.md",
    }


# ------------------------------------------------------------------------------------------------ geometry
def _rpy(roll, pitch, yaw):
    return (round(float(roll), 3), round(float(pitch), 3), round(float(yaw), 3))


def _r(v, n=3):
    return tuple(round(float(x), n) for x in v)


def _half_z(size, rpy):
    """Half of the vertical extent of a rotated box."""
    R = rot_matrix(rpy)
    return 0.5 * float(np.abs(R[2]) @ np.asarray(size, float))


class _Frame:
    """Block-local frame for a toppled spine block: u east from the pivot, v north from the block axis."""

    def __init__(self, x0, yc, yaw_deg):
        self.x0, self.yc, self.yaw = x0, yc, yaw_deg
        self.c, self.s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))

    def piece(self, kind, u, v, z, size, rpy, mat):
        x = self.x0 + u * self.c - v * self.s
        y = self.yc + u * self.s + v * self.c
        return Piece(kind, _r((x, y, z)), _r(size), _rpy(rpy[0], rpy[1], rpy[2] + self.yaw), mat)


def _spine_block(name, g, out):
    """Toppled spine block: fallen-book slabs, facade top, shelves, columns, end walls, shattered tail."""
    U = g.uniform
    x0b, x1b, y0, y1 = BLOCKS[name]
    W = y1 - y0
    H = LYING_H[name]
    f = _Frame(PIVOT_X + U(-0.5, 0.5), (y0 + y1) / 2, U(-3.0, 3.0))
    hw = W / 2

    def panels(n, gap):
        edges = np.linspace(-hw, hw, n + 1) + np.r_[0, U(-0.6, 0.6, n - 1), 0]
        return [((a + b) / 2, b - a - gap) for a, b in zip(edges[:-1], edges[1:])]

    def slab(u, v, w, h, lean, mat="concrete", roll=0.0):
        rpy = (roll, lean, U(-2, 2))
        size = (0.28, w, h)
        hz = _half_z(size, rpy)
        out.append(f.piece("slab", u + 0.5 * h * np.sin(np.radians(lean)), v, hz - 0.05, size, rpy, mat))

    # storeys 1-3: crushed tight at the base (books packed together)
    for k in range(3):
        for v, w in panels(2, U(0.2, 0.6)):
            slab(0.3 + 0.7 * k, v, w, H * U(0.45, 0.7), U(15, 32), roll=U(-4, 4))
    # storeys 4-13: near-vertical slabs ~2.9 m apart, room voids between them
    us = [2.4 + STOREY * k + U(-0.15, 0.15) for k in range(10)]
    for k, u in enumerate(us):
        for v, w in panels(int(g.integers(2, 4)), U(0.15, 0.7)):
            lean = U(-3, 8)
            if g.uniform() < 0.2:                      # snapped slab: lower stub + upper part tipped east
                hl = H * U(0.45, 0.6)
                slab(u, v, w, hl, lean)
                hu = H * U(0.93, 0.98) - hl
                rpy = (U(-6, 6), lean + U(15, 30), U(-4, 4))
                size = (0.28, w * U(0.8, 1.0), hu)
                out.append(f.piece("slab", u + 0.6 + 0.5 * hu * np.sin(np.radians(rpy[1])), v,
                                   hl + _half_z(size, rpy) - 0.2, size, rpy, "concrete"))
            else:
                slab(u, v, w, H * U(0.94, 1.0) - 0.1, lean, roll=U(-2, 2))
    # facade (the former courtyard face) lying on top of the rooms; missing panels = openings
    for k in range(9):
        u = (us[k] + us[k + 1]) / 2 + 0.5
        for v, w in panels(int(g.integers(3, 5)), 0.15):
            if g.uniform() < 0.24:
                continue
            out.append(f.piece("wall", u, v, H + U(-0.15, 0.1), (STOREY - 0.05, w, 0.22),
                               (U(-4, 4), U(-4, 4), U(-2, 2)), "concrete" if g.uniform() < 0.6 else "brick"))
    # room interiors: horizontal shelves (former N-S partitions), lean-to wedges, broken columns
    for k in range(9):
        um = (us[k] + us[k + 1]) / 2 + 0.4
        for _ in range(int(g.integers(1, 3))):
            pitch = U(-12, 12) if g.uniform() < 0.6 else U(28, 50) * g.choice([-1, 1])
            out.append(f.piece("wall", um, U(-hw + 1.5, hw - 1.5), H * U(0.2, 0.8),
                               (2.5, U(2.0, 4.5), 0.15), (U(-8, 8), pitch, U(-5, 5)), "brick"))
        if g.uniform() < 0.7:
            out.append(f.piece("column", um, U(-hw + 1, hw - 1), U(1.0, H - 1.5),
                               (U(1.4, 2.8), 0.45, 0.45), (0, U(-25, 25), U(-15, 15)), "concrete"))
    # end walls (E-W walls stay vertical, normal along y), broken
    for v in (-hw + 0.15, hw - 0.15):
        for k in range(10):
            if g.uniform() < 0.45:
                h = H * U(0.4, 0.95)
                out.append(f.piece("wall", us[k] + 1.45, v, h / 2 - 0.05, (STOREY - 0.1, 0.2, h),
                                   (U(-5, 5), 0, U(-4, 4)), "brick"))
    # storeys 14-16: shattered beyond the far kerb
    u_end = us[-1] + 1.0
    for k in range(3):
        for _ in range(4):
            pitch = U(30, 80)
            size = (0.28, U(2.0, 5.0), U(2.0, 6.0) * (1.0 - 0.2 * k))
            rpy = (U(-15, 15), pitch, U(-20, 20))
            out.append(f.piece("slab", u_end + 2.2 * k + U(0, 2.5), U(-hw + 1, hw - 1), _half_z(size, rpy) - 0.1,
                               size, rpy, "concrete"))
    # original footprint (x 6..20): crushed basement / ground-floor layer and sheared column stubs
    for _ in range(5):
        size = (U(3.0, 6.0), U(3.0, 5.0), 0.3)
        out.append(Piece("slab", _r((U(x0b + 2, x1b - 1), U(y0 + 1.5, y1 - 1.5), U(0.3, 2.4))), _r(size),
                         _rpy(U(-12, 12), U(-18, 8), U(0, 360)), "concrete"))
    for _ in range(int(g.integers(4, 8))):
        lean, L = U(8, 40), U(1.5, 3.6)
        rpy = (0, -(90 - lean), U(-12, 12))
        out.append(Piece("column", _r((U(x0b + 1, x1b), U(y0 + 1, y1 - 1), _half_z((L, 0.55, 0.55), rpy))),
                         _r((L, 0.55, 0.55)), _rpy(*rpy), "concrete"))
    return f, us, u_end


def _wing_block(name, g, out):
    """Pancaked wing: stacked, tilted floor slabs with pancake voids, column stubs, fallen facade walls."""
    U = g.uniform
    x0, x1, y0, y1 = BLOCKS[name]
    Ht = PANCAKE_H[name]
    north = y0 > 0
    L = int(g.integers(5, 8))
    zs = np.linspace(0.35, Ht - 0.3, L)
    for i, z in enumerate(zs):
        drag = 0.35 * i                                   # upper layers dragged east towards the spine
        xe = np.linspace(x0, x1, 4)
        ye = np.linspace(y0, y1, 3)
        for a in range(3):
            for b in range(2):
                if g.uniform() < 0.12 and 0 < i < L - 1:
                    continue                              # missing panel: open pocket
                w, d = xe[a + 1] - xe[a] + U(-0.3, 0.6), ye[b + 1] - ye[b] + U(-0.3, 0.6)
                cx, cy = (xe[a] + xe[a + 1]) / 2 + drag + U(-0.4, 0.4), (ye[b] + ye[b + 1]) / 2 + U(-0.4, 0.4)
                ramp = (cx - x0) / (x1 - x0)               # stack rises slightly towards the spine
                out.append(Piece("slab", _r((cx, cy, z * (0.85 + 0.25 * ramp) + U(-0.15, 0.15))), _r((w, d, 0.3)),
                                 _rpy(U(-7, 7), U(-9, 3), U(-4, 4)), "concrete"))
        if 0 < i:                                         # crushed column stubs propping the layer above
            for _ in range(int(g.integers(2, 5))):
                h = max(0.4, zs[i] - zs[i - 1] - 0.3)
                out.append(Piece("column", _r((U(x0 + 1, x1 - 1) + drag, U(y0 + 1, y1 - 1), z - h / 2 - 0.1)),
                                 _r((h, 0.5, 0.5)), _rpy(0, -90 + U(-15, 15), U(0, 360)), "concrete"))
    for _ in range(int(g.integers(3, 6))):              # columns sticking out of the stack
        Lc = U(2.0, 4.0)
        rpy = (0, -U(20, 55), U(0, 360))
        out.append(Piece("column", _r((U(x0, x1), U(y0, y1), U(1.0, Ht))), _r((Lc, 0.5, 0.5)), _rpy(*rpy), "concrete"))
    yo = y1 + 1.2 if north else y0 - 1.2                  # outer facade fallen outwards / leaning
    for _ in range(int(g.integers(3, 6))):
        h = U(2.0, 3.2)
        out.append(Piece("wall", _r((U(x0 + 1, x1 - 1), yo + U(-0.6, 0.6), U(0.5, 1.2))), _r((U(2.5, 4.0), 0.25, h)),
                         _rpy((1 if north else -1) * U(55, 80), 0, U(-8, 8)), "brick"))


def _chunks(g, out, n, xr, yr, avoid=None):
    """Loose debris chunks on the ground (``avoid(x, y)`` -> True skips spots inside a stack)."""
    U = g.uniform
    k = 0
    while k < n:
        x, y = U(*xr), U(*yr)
        if avoid is not None and avoid(x, y):
            continue
        s = U(0.3, 1.2)
        size = (s, s * U(0.6, 1.3), s * U(0.45, 1.0))
        out.append(Piece("chunk", _r((x, y, size[2] * 0.3)), _r(size), _rpy(U(0, 360), U(0, 360), U(0, 360)),
                         "brick" if g.uniform() < 0.35 else "concrete"))
        k += 1


@functools.lru_cache(maxsize=1)
def _geometry():
    """(pieces, sector mound params, noise) for the site; fixed RNG, cached (the site never changes)."""
    g = np.random.default_rng(GEOM_SEED)
    U = g.uniform
    pieces = []
    spine = {}
    for name in SPINE:
        spine[name] = _spine_block(name, g, pieces)
    for name in "ABHI":
        _wing_block(name, g, pieces)

    def in_stack(x, y):
        for name, (f, us, u_end) in spine.items():
            dx, dy = x - f.x0, y - f.yc
            u, v = dx * f.c + dy * f.s, -dx * f.s + dy * f.c
            if -0.5 < u < u_end + 1 and abs(v) < (BLOCKS[name][3] - BLOCKS[name][2]) / 2 - 0.3:
                return True
        return False

    # spill: across and along the road, round the stack ends, beyond the shattered top, into the courtyard
    _chunks(g, pieces, 260, (PIVOT_X - 1, 66.0), (-37.0, 37.0), avoid=in_stack)
    _chunks(g, pieces, 110, (-24.0, 21.0), (-34.0, 34.0),
            avoid=lambda x, y: (-20 < x < 6.5 and abs(y) > 16.5) or (x > 6.5 and abs(y) < 29))
    _chunks(g, pieces, 60, (6.0, 21.0), (-31.0, 31.0))

    noise = (U(0.1, 0.3, 5), U(-0.4, 0.4, 5), U(-0.4, 0.4, 5), U(0, 2 * np.pi, 5))
    mounds = {}
    for name in SECTORS:
        cx, cy = CENTRES[name]
        if name in SPINE:
            mounds[name] = dict(h=U(2.4, 3.4), sx=7.0, sy=U(4.0, 5.0), rot=float(np.radians(U(-4, 4))),
                                lump=(U(-6, 6), U(-2, 2)))
        else:
            mounds[name] = dict(h=U(1.4, 2.4), sx=U(5.2, 6.2), sy=U(4.2, 5.0), rot=float(np.radians(U(-8, 8))),
                                lump=(U(-3, 3), U(-2, 2)))
    return tuple(pieces), mounds, noise


def sector_of(x, y):
    """Nearest sector centre (vectorised): the sector a point belongs to."""
    names = list(CENTRES)
    c = np.array([CENTRES[n] for n in names])
    x, y = np.asarray(x, float), np.asarray(y, float)
    d = (x[..., None] - c[:, 0]) ** 2 + (y[..., None] - c[:, 1]) ** 2
    idx = np.argmin(d, axis=-1)
    return np.array(names)[idx]


def _build_geometry_scenario(seed):
    pieces, mounds, noise = _geometry()
    piles = []
    for name in SECTORS:
        cx, cy = CENTRES[name]
        m = mounds[name]
        piles.append(Pile(name=name, x=cx, y=cy, h=m["h"], sx=m["sx"], sy=m["sy"], rot=m["rot"], lump=m["lump"]))
    owner = sector_of([p.center[0] for p in pieces], [p.center[1] for p in pieces])
    by = {p.name: p for p in piles}
    for pc, o in zip(pieces, owner):
        by[str(o)].pieces.append(pc)
    from .site_mesh import _hull, attach_polygons
    polys = {}
    for p in piles:                                   # sector outline: hull of its pieces' footprints (CCW)
        xy = np.concatenate([np.asarray([f[:, :2] for f in _box_faces(pc)]).reshape(-1, 2) for pc in p.pieces])
        polys[p.name] = _hull(xy[:, 0], xy[:, 1])
    attach_polygons(piles, polys, {n: ("toppled_spine" if n in SPINE else "pancaked_wing") for n in SECTORS})
    return WeiguanScenario(seed, piles, {}, noise)


# --------------------------------------------------------------------------------------------- truth
class Solids:
    """Inside-any-piece test (oriented boxes, vectorised over pieces)."""

    def __init__(self, pieces):
        self.c = np.array([p.center for p in pieces], float)
        self.h = np.array([p.size for p in pieces], float) / 2
        self.R = np.stack([rot_matrix(p.rpy_deg) for p in pieces])

    def inside(self, p, margin=0.25):
        loc = np.einsum("kji,kj->ki", self.R, np.asarray(p, float)[None, :] - self.c)   # R^T (p - c)
        return bool(np.any(np.all(np.abs(loc) <= self.h + margin, axis=1)))


class TruthSupport:
    """What victim placement needs: surface grid, bare ground under it, sector of each cell, solids."""

    def __init__(self, hm, ground, owner, solids=None, edge_bias=0.0):
        self.hm, self.ground, self.owner, self.solids = hm, np.asarray(ground, float), np.asarray(owner), solids
        self.cover = hm.z - self.ground
        self.X, self.Y = hm.mesh()
        gy, gx = np.gradient(hm.z.astype(float), hm.res)
        # near a face: a steep step within ~1.5 m (open edges, slab ends, crevasses)
        from scipy.ndimage import maximum_filter
        self.edge = maximum_filter(np.hypot(gx, gy), size=max(3, int(round(3.0 / hm.res)) | 1)) > 1.0
        self.edge_bias = edge_bias

    def face_dist(self, p, r=5.0):
        """3D distance from a point to the nearest heightmap surface sample within r metres."""
        m = (np.abs(self.X - p[0]) <= r) & (np.abs(self.Y - p[1]) <= r)
        d = np.sqrt((self.X[m] - p[0]) ** 2 + (self.Y[m] - p[1]) ** 2 + (self.hm.z[m] - p[2]) ** 2)
        return float(d.min())

    def place(self, rng, sector, depth_cls, used, ground_fn):
        """Victim spot in a void: (xy, z, depth below the heightmap, face distance) or None."""
        lo, hi = DEPTH_CLASS[depth_cls]
        for _ in range(600):
            depth = float(rng.uniform(lo, hi))
            ok = (self.owner == sector) & (self.cover > depth + 0.4)
            if self.edge_bias and rng.uniform() < self.edge_bias and (ok & self.edge).any():
                ok = ok & self.edge
            ok = np.argwhere(ok)
            if len(ok) == 0:
                return None
            j, i = ok[int(rng.integers(0, len(ok)))]
            x, y = float(self.X[j, i] + rng.uniform(-0.2, 0.2)), float(self.Y[j, i] + rng.uniform(-0.2, 0.2))
            z = float(self.hm.height_at(x, y)) - depth
            if z < float(ground_fn(x, y)) + 0.35 or (self.solids is not None and self.solids.inside((x, y, z))):
                continue
            if any(np.hypot(x - u[0], y - u[1]) < 2.5 for u in used):
                continue
            return [round(x, 3), round(y, 3)], round(z, 3), round(depth, 3), round(self.face_dist((x, y, z)), 3)
        return None


def make_truth(seed, start_pile, names, sup: TruthSupport, ground_fn, site=SITE, salt=2016,
               role_sectors=None, start_choices=None, silent_victim_p=0.35):
    """Hidden truth for any site: start sector, one acoustic role per other sector, victims in voids, sounds.

    Separate RNG streams ([seed, salt, 1] placement/roles, [seed, salt, 99] physiology), so the geometry
    never depends on it. Deep emitters (knock x2, weak voice) go to sectors with real cover (> 5.5 m over
    >= 5 m^2) so they exist as radar-unreachable, acoustic-only finds. Sectors beyond the 8 roles are silent.
    ``role_sectors`` limits the acoustic roles (and deep emitters) to those sectors (e.g. the collapse core);
    ``start_choices`` the random start sector; silent sectors get a victim with ``silent_victim_p``.
    """
    trng = np.random.default_rng([seed, salt, 1])
    prng = np.random.default_rng([seed, salt, 99])
    names = list(names)
    pool = list(start_choices or names)
    start = start_pile or pool[int(trng.integers(0, len(pool)))]
    others = [n for n in names if n != start]
    trng.shuffle(others)
    eligible = [n for n in others if role_sectors is None or n in role_sectors]
    min_cells = 5.0 / sup.hm.res ** 2
    deep_ok = [n for n in eligible if ((sup.owner == n) & (sup.cover > 5.5)).sum() >= min_cells]
    deep_kinds = ["knock", "knock", "weak_voice"]
    roles = dict(zip(deep_ok[:3], deep_kinds))
    rest = list(ROLE_KINDS)
    for k in list(roles.values()):
        rest.remove(k)
    trng.shuffle(rest)
    free = [n for n in eligible if n not in roles]
    roles.update(zip(free, rest + ["silent"] * max(0, len(free) - len(rest))))
    roles.update({n: "silent" for n in others if n not in roles})
    roles = {n: roles[n] for n in others}
    langs = list(LANGS)
    trng.shuffle(langs)
    plan = [(start, "shallow", None)]                      # reported collapse spot: a radar-findable survivor
    if trng.uniform() < 0.6:
        plan.append((start, "mid", None))
    li = 0
    for name in others:
        kind = roles[name]
        if kind in ("voice", "weak_voice", "knock"):
            if kind in ("knock", "weak_voice"):
                cls = "deep"
            else:
                cls = "deep" if trng.uniform() < 0.3 else ("mid" if trng.uniform() < 0.6 else "shallow")
            lang = None
            if kind in ("voice", "weak_voice"):
                lang, li = langs[li % len(langs)], li + 1
            plan.append((name, cls, (kind, lang)))
            if trng.uniform() < 0.5:
                plan.append((name, "shallow" if trng.uniform() < 0.5 else "mid", None))
        elif trng.uniform() < (0.5 if kind != "silent" else silent_victim_p):
            plan.append((name, "mid" if trng.uniform() < 0.5 else "shallow", None))

    victims, sounds, used = [], [], []
    for name, cls, snd in plan:
        got = sup.place(trng, name, cls, used, ground_fn)
        if got is None and cls == "deep":
            cls = "mid"
            got = sup.place(trng, name, cls, used, ground_fn)
        if got is None:
            continue
        xy, z, depth, fd = got
        used.append(xy)
        v = {"pile": name, "xy": xy, "z": z, "depth": depth, "face_dist": fd, "depth_class": cls,
             "radar_reachable": bool(depth <= RADAR_REACH_M), "sound": snd[0] if snd else None}
        if snd:
            kind, lang = snd
            level = float(trng.uniform(-6, -2) if cls == "deep" else trng.uniform(-4, 0))
            s = {"pile": name, "kind": kind, "xy": xy, "level_db": level}
            if lang:
                s.update(lang=lang, text=VOICE_TEXT[lang][kind], tts_lang=TTS_LANG[lang], fleurs=FLEURS[lang],
                         text_en=EN_GLOSS[kind])
                if lang in ROMANISED:
                    s["romanised"] = ROMANISED[lang][kind]
                v["lang"] = lang
            sounds.append(s)
        victims.append(v)
    for name, kind in roles.items():                       # non-human sources
        if kind in ("dog", "settling"):
            got = sup.place(trng, name, "shallow", used, ground_fn) or sup.place(trng, name, "mid", used, ground_fn)
            if got:
                used.append(got[0])
                sounds.append({"pile": name, "kind": kind, "xy": got[0], "level_db": float(trng.uniform(-6, 0))})
    # the radar generator uses the first victim per pile: shallowest first; ids in sector order
    order_names = {n: k for k, n in enumerate(names)}
    victims.sort(key=lambda v: (order_names[v["pile"]], v["depth"]))
    states = list(PHYSIO_STATES)
    perm = list(prng.permutation(len(victims)))            # every breathing state appears once (>= 5 victims)
    for rank, k in enumerate(perm):
        st = states[rank] if rank < len(states) else states[int(prng.integers(0, len(states)))]
        victims[k]["physio"] = {"state": st, **{key: float(prng.uniform(*r_)) for key, r_ in PHYSIO_STATES[st].items()}}
        victims[k]["breathing_hz"] = float(prng.uniform(0.2, 0.33))   # same convention as the grid site
    victims = [{"id": f"V{k + 1}", **v} for k, v in enumerate(victims)]
    return {"site": site, "start_pile": start, "roles": roles, "victims": victims, "sounds": sounds,
            "_note": "HIDDEN GROUND TRUTH: depth is below the 0.5 m heightmap surface; z is absolute (site frame)."}


@functools.lru_cache(maxsize=1)
def _truth_support():
    sc = _build_geometry_scenario(0)
    hm = sc.heightmap(0.5)
    X, Y = hm.mesh()
    return sc, TruthSupport(hm, sc.ground_height(X, Y), sector_of(X, Y),
                            Solids([pc for p in sc.piles for pc in p.pieces]))


def make_weiguan(seed: int = 7, start_pile: str | None = None) -> WeiguanScenario:
    if start_pile is not None and start_pile not in SECTORS:
        raise ValueError(f"start_pile {start_pile!r}: Weiguan sectors are A..I")
    sc = _build_geometry_scenario(seed)
    base, sup = _truth_support()
    sc.truth = make_truth(seed, start_pile, SECTORS, sup, base.ground_height)
    return sc


# --------------------------------------------------------------------------------------------- render
def _box_faces(pc):
    R = rot_matrix(pc.rpy_deg)
    h = np.asarray(pc.size) / 2
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h
    P = corners @ R.T + np.asarray(pc.center)
    idx = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
    return [P[list(f)] for f in idx]


def render(sc, out_site, out_sectors, truth=True):
    """docs images: oblique 3D + top-down heightmap (out_site), labelled sector map (out_sectors)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    hm = sc.heightmap(0.5)
    x0, y0, x1, y1 = BOUNDS
    col = {("slab", "concrete"): "#9a9a94", ("wall", "concrete"): "#b8b4a8", ("wall", "brick"): "#a5644a",
           ("column", "concrete"): "#6f6f6a", ("chunk", "concrete"): "#8a8780", ("chunk", "brick"): "#94573f",
           ("slab", "brick"): "#a5644a"}
    fig = plt.figure(figsize=(16, 7.4))
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    faces, fc = [], []
    for p in sc.piles:
        for pc in p.pieces:
            fs = _box_faces(pc)
            faces += fs
            fc += [col.get((pc.kind, pc.material), "#999")] * len(fs)
    pcoll = Poly3DCollection(faces, facecolors=fc, edgecolors="#00000030", linewidths=0.15)
    ax.add_collection3d(pcoll)
    X, Y = np.meshgrid(np.arange(-26, 66, 2.0), np.arange(-40, 40, 2.0))
    ax.plot_surface(X, Y, sc.ground_height(X, Y), color="#c9c2b0", alpha=0.55, linewidth=0, shade=False)
    ax.plot([ROAD_X[0]] * 2, [-40, 40], [0.02, 0.02], color="#333", lw=0.8)
    ax.plot([ROAD_X[1]] * 2, [-40, 40], [0.02, 0.02], color="#333", lw=0.8)
    ax.set_xlim(-26, 66); ax.set_ylim(-44, 44); ax.set_zlim(0, 30)
    ax.set_box_aspect((92, 88, 30))
    ax.view_init(elev=28, azim=-62)
    ax.set_xlabel("x east (m)"); ax.set_ylabel("y north (m)"); ax.set_zlabel("z (m)")
    ax.set_title("Weiguan Jinlong stand-in: oblique view from SW (procedural, assumed geometry)")
    ax2 = fig.add_subplot(1, 2, 2)
    im = ax2.imshow(hm.z, origin="lower", extent=(x0, x1, y0, y1), cmap="terrain", vmin=-1, vmax=15)
    fig.colorbar(im, ax=ax2, label="surface height (m)", shrink=0.8)
    for n, (bx0, bx1, by0, by1) in BLOCKS.items():
        ax2.add_patch(Rectangle((bx0, by0), bx1 - bx0, by1 - by0, fill=False, ls="--", lw=0.7, ec="#555"))
        ax2.text((bx0 + bx1) / 2, (by0 + by1) / 2, n.lower(), color="#555", fontsize=7, ha="center", va="center")
    ax2.axvspan(*ROAD_X, color="#000", alpha=0.06)
    ax2.text(31, 38, "Yongda Rd Sec. 2", ha="center", fontsize=8)
    for p in sc.piles:
        ax2.text(p.x, p.y, p.name, fontsize=12, weight="bold", ha="center", va="center", color="#c00000")
    bx, by = BASE_XY
    ax2.plot(*zip(*DRONE_PADS), "ks", ms=3)
    ax2.text(bx, by + 2.5, "staging", ha="center", fontsize=8)
    ax2.set_title("Top view: surface heightmap (0.5 m); dashed = blocks before collapse (lower-case)")
    ax2.set_xlabel("x east (m)"); ax2.set_ylabel("y north (m)")
    fig.tight_layout()
    fig.savefig(out_site, dpi=110)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 8.6))
    X, Y = hm.mesh()
    own = sector_of(X, Y)
    cov = (hm.z - sc.ground_height(X, Y)) > 0.4
    lab = np.vectorize(lambda s: SECTORS.index(s))(own).astype(float)
    lab[~cov & (hm.z < 0.4)] = np.nan
    ax.imshow(lab, origin="lower", extent=(x0, x1, y0, y1), cmap="tab10", alpha=0.35, vmin=0, vmax=9)
    cs = ax.contour(X, Y, hm.z, levels=[2, 5, 8, 11], colors="k", linewidths=0.4)
    ax.clabel(cs, fontsize=6, fmt="%d m")
    for p in sc.piles:
        ax.text(p.x, p.y, p.name, fontsize=16, weight="bold", ha="center", va="center")
        ax.text(p.x, p.y - 2.6, f"{len(p.pieces)} pcs", fontsize=7, ha="center")
    ax.axvspan(*ROAD_X, color="#000", alpha=0.05)
    ax.plot(*zip(*DRONE_PADS), "ks", ms=3)
    ax.text(BASE_XY[0], BASE_XY[1] + 2.5, "staging / pads", ha="center", fontsize=8)
    if truth:
        mk = {"shallow": "#1a9850", "mid": "#fdae61", "deep": "#d73027"}
        for v in sc.truth["victims"]:
            ax.plot(*v["xy"], "o", ms=6, mfc=mk[v["depth_class"]], mec="k", mew=0.5)
            ax.text(v["xy"][0] + 0.8, v["xy"][1] + 0.6, f'{v["id"]} {v["depth"]:.1f} m', fontsize=6)
        for s in sc.truth["sounds"]:
            ax.text(s["xy"][0] + 0.8, s["xy"][1] - 1.4, s["kind"] + (f' [{s["lang"]}]' if s.get("lang") else ""),
                    fontsize=6, color="#2b5d9c")
        ax.set_title(f"Search sectors A-I (blocks) and HIDDEN TRUTH, seed {sc.seed} "
                     "(green < 1.5 m, orange 1.5-3 m, red > 4 m deep)", fontsize=9)
    ax.set_xlim(-30, 66); ax.set_ylim(-62, 42)
    ax.set_aspect("equal")
    ax.set_xlabel("x east (m)"); ax.set_ylabel("y north (m)")
    fig.tight_layout()
    fig.savefig(out_sectors, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Render the Weiguan stand-in (docs/img).")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out", default="docs/img/weiguan_site.png")
    ap.add_argument("--sectors-out", default="docs/img/weiguan_sectors.png")
    a = ap.parse_args()
    sc = make_weiguan(a.seed)
    render(sc, a.out, a.sectors_out)
    print(f"{sum(len(p.pieces) for p in sc.piles)} pieces, {len(sc.truth['victims'])} victims -> {a.out}, {a.sectors_out}")
