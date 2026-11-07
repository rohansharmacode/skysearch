"""The real expanded collapse site (Isaac teammate's ``collapse_site/``, branch ``3d_model``) as a Scenario.

    sc = make_scenario(7, site="collapse")          # alias "weiguan-real"

Source: ``collapse_site.usda`` (320 m x 320 m, metres, x east, y north, z up, origin = centre of the scanned
Weiguan site): the two fallen 16-storey towers + floor plates rebuilt from the photogrammetry screenshots,
13 scanned buildings, 193 procedural lots (standing / tilted / pancake / collapsed / open), 44 rubble mounds,
terrain tiles; debris is a visual-only PointInstancer (ignored here).

Heightmap: terrain + building meshes rasterised (max z) at 0.5 m over [-160, 160]^2, cached in
``data/maps/collapse_site_heightmap.npz`` (gitignored; rebuild with ``python -m sim.site_collapse --build``
when the USD changes); a copy is committed at ``assets/maps/collapse_site_heightmap.npz`` (1.6 MB) so the
sim runs without the USD. Bare ground: morphological opening of the surface (street level ~0 m).

Sectors (search regions) come from the generator's own layout (``generator/sitecfg.py`` + ``outer_layout.py``,
imported read-only; the layout numbers are copied into the cache so the USD folder is only needed to rebuild):
  T1a-c / T2a-c   the west / east fallen tower, thirds north -> south
  CN, CS          the rubble field between the towers (with the broken floor plates), north / south half
  RN RS RW RE     the core rubble zones north / south / west / east of the towers
  L###            collapsed and pancaked lots (### = the lot index of the USD prim ``<type>_###``)
  M01..M44        rubble mounds
Standing / tilted lots and the scanned buildings are NOT sectors: they are listed in ``site_info.obstacles``
(polygon + height) so flight can avoid them (tallest: SE tower 24 m).

Hidden truth (``make_truth``, separate RNG): the eight acoustic roles and the start sector are in the tower
collapse core (the reported collapse); a few silent victims are spread over the district's lots and mounds.
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np

from .scenario import Pile
from .site_mesh import MeshScenario, estimate_ground, load_surface, rasterise, segment
from .world import Grid

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CACHE = os.path.join(REPO, "data", "maps", "collapse_site_heightmap.npz")          # local rebuild (gitignored)
COMMITTED = os.path.join(REPO, "assets", "maps", "collapse_site_heightmap.npz")    # committed copy (1.6 MB)
EXTENT = 160.0
RES = 0.5
GEO = {"lat": 23.0051432, "lon": 120.2608904, "bearing_deg": 0.0,
       "name": "Weiguan Jinlong collapse, Yongkang District, Tainan (2016): scanned core + extrapolated district"}
# staging: open lot at the south edge beside the N-S street x = -63.5 (16 x 9 m clear to < 0.3 m in the 0.5 m
# heightmap; the street intersections all have wreckage within 8 m)
BASE_XY = (-56.0, -152.5)
DRONE_PADS = [(-58.6, -150.0), (-56.0, -150.0), (-53.4, -150.0)]
CORE = ["T1a", "T1b", "T1c", "T2a", "T2b", "T2c", "CN", "CS", "RN", "RS", "RW", "RE"]


def usd_candidates():
    env = os.environ.get("LARPVERSE_COLLAPSE_SITE")
    return [p for p in (env,
                        os.path.join(REPO, "collapse_site", "collapse_site.usda"),
                        os.path.join(os.path.dirname(REPO), "collapse_site", "collapse_site.usda"),  # skysearch/ (3d_model)
                        os.path.join(REPO, "assets", "collapse_site", "collapse_site.usda"),
                        os.path.join(os.path.dirname(REPO), "collapse_site_src", "collapse_site", "collapse_site.usda"))
            if p]


def find_usd():
    for p in usd_candidates():
        if os.path.exists(p):
            return p
    return None


# ------------------------------------------------------------------------------------------- layout
def _rect_poly(rect, yaw_deg=0.0):
    x0, y0, x1, y1 = rect
    cx, cy, hw, hh = (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2, (y1 - y0) / 2
    a = np.radians(yaw_deg)
    c, s = np.cos(a), np.sin(a)
    pts = np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]])
    return np.stack([cx + pts[:, 0] * c - pts[:, 1] * s, cy + pts[:, 0] * s + pts[:, 1] * c], 1)


def _circle(x, y, r, n=16):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.stack([x + r * np.cos(a), y + r * np.sin(a)], 1)


def layout_from_generator(usd_path):
    """Footprints from the teammate's generator (read-only import): sectors + obstacles, site frame."""
    gen = os.path.join(os.path.dirname(os.path.abspath(usd_path)), "generator")
    if gen not in sys.path:
        sys.path.insert(0, gen)
    import outer_layout as O   # noqa: E402  (their module; numpy only)
    import sitecfg as S        # noqa: E402
    sectors, obstacles = {}, []
    for name, y0, y1, xw, xr, xe, hs in S.SLABS:               # fallen towers: tent prisms along y
        tag = "T1" if name.startswith("W") else "T2"
        xs, _ = S.px2w(np.array([min(xw), max(xe)]), np.array([0, 0]))
        _, ys = S.px2w(np.array([0, 0]), np.array([y0, y1]))
        yt, yb = max(ys), min(ys)
        for k, suf in enumerate("abc"):
            ya, yb_ = yt - (yt - yb) * k / 3, yt - (yt - yb) * (k + 1) / 3
            sectors[tag + suf] = {"kind": "fallen_tower", "polygon": _rect_poly((xs[0] - 1, yb_, xs[1] + 1, ya)),
                                  "usd_prim": f"/CollapseSite/Buildings/Scanned/core_{name}"}
    zones = {z[0]: S.poly_w(z[1]) for z in S.RUBBLE_ZONES}
    main = zones["main"]
    ymid = (main[:, 1].min() + main[:, 1].max()) / 2
    x0, x1 = main[:, 0].min(), main[:, 0].max()
    sectors["CN"] = {"kind": "core_rubble", "polygon": _rect_poly((x0, ymid, x1, main[:, 1].max())),
                     "usd_prim": "/CollapseSite/Buildings/Scanned/core_P1"}
    sectors["CS"] = {"kind": "core_rubble", "polygon": _rect_poly((x0, main[:, 1].min(), x1, ymid)),
                     "usd_prim": "/CollapseSite/Buildings/Scanned/core_P2"}
    for key, name in (("north", "RN"), ("south", "RS"), ("west", "RW"), ("east", "RE")):
        sectors[name] = {"kind": "core_rubble", "polygon": zones[key], "usd_prim": "/CollapseSite/Terrain"}
    for name, poly, h, *_ in S.CORE_BUILDINGS:
        obstacles.append({"name": f"core_{name}", "polygon": S.poly_w(poly), "height": float(h), "kind": "scanned"})
    L = O.make_layout()
    for i, f in enumerate(L["features"]):
        poly = _rect_poly(f["rect"], f.get("yaw", 0.0))
        prim = f"/CollapseSite/Buildings/Surroundings/{f['type']}_{i:03d}"
        if f["type"] in ("collapsed", "pancake"):
            sectors[f"L{i:03d}"] = {"kind": f["type"], "polygon": poly, "usd_prim": prim}
        elif f["type"] in ("standing", "tilted"):
            obstacles.append({"name": f"L{i:03d}", "polygon": poly, "kind": f["type"],
                              "height": float(3.1 * f.get("storeys", 2) + 1.0), "usd_prim": prim})
    for k, m in enumerate(L["mounds"]):
        sectors[f"M{k + 1:02d}"] = {"kind": "mound", "polygon": _circle(m["x"], m["y"], m["r"]), "usd_prim": None}
    return sectors, obstacles


# -------------------------------------------------------------------------------------------- cache
def build_cache(usd_path=None, out=CACHE, log=print):
    """USD -> 0.5 m heightmap + footprints (npz, interchange format of docs/SITE_WEIGUAN.md)."""
    from .site_mesh import _read_usd
    usd_path = usd_path or find_usd()
    if not usd_path:
        raise FileNotFoundError(f"collapse_site.usda not found; tried {usd_candidates()} (set LARPVERSE_COLLAPSE_SITE)")
    V, F = _read_usd(usd_path, include=("/CollapseSite/Terrain", "/CollapseSite/Buildings"))
    log(f"[collapse] {len(V)} vertices, {0 if F is None else len(F)} triangles from {usd_path}")
    n = int(round(2 * EXTENT / RES))
    z = rasterise(V, F, (-EXTENT, -EXTENT), RES, n, n)
    holes = int(np.isnan(z).sum())
    from .site_mesh import fill_nan
    z = fill_nan(z, floor=float(np.nanpercentile(z, 1))).astype(np.float32)
    ground = estimate_ground(z.astype(float), RES).astype(np.float32)
    sectors, obstacles = layout_from_generator(usd_path)
    fp = {k: np.round(v["polygon"], 3).tolist() for k, v in sectors.items()}
    meta = {"geo": GEO, "base_xy": list(BASE_XY), "drone_pads": [list(p) for p in DRONE_PADS],
            "frame": "site: m, x east, y north, z up; origin = centre of the scanned Weiguan site",
            "source": os.path.relpath(usd_path, REPO) if usd_path.startswith(os.path.dirname(REPO)) else usd_path,
            "sector_info": {k: {"kind": v["kind"], "usd_prim": v["usd_prim"]} for k, v in sectors.items()},
            "obstacles": [{**o, "polygon": np.round(o["polygon"], 3).tolist()} for o in obstacles],
            "holes_filled": holes}
    os.makedirs(os.path.dirname(out), exist_ok=True)
    np.savez_compressed(out, origin=np.array([-EXTENT, -EXTENT]), res=np.float64(RES), z=z, ground=ground,
                        footprints=json.dumps(fp), meta=json.dumps(meta))
    log(f"[collapse] heightmap {n}x{n} at {RES} m: z {float(z.min()):.2f}..{float(z.max()):.2f} m, "
        f"{holes} empty cells filled, {len(fp)} sectors, {len(obstacles)} obstacles -> {out}")
    return out


# ------------------------------------------------------------------------------------------ scenario
class CollapseScenario(MeshScenario):
    site = "collapse"

    def public_dict(self):
        d = super().public_dict()
        meta = self.meta.get("meta", {})
        d["site"] = "collapse"
        d["site_info"] = {"name": GEO["name"], "usd": meta.get("source"), "usd_root": "/CollapseSite",
                          "obstacles": meta.get("obstacles", []),
                          "max_obstacle_h": max([o["height"] for o in meta.get("obstacles", [])] or [0.0]),
                          "doc": "docs/SITE_WEIGUAN.md", "licence_note": "core textures from third-party "
                          "photogrammetry screenshots (licence unknown): see docs/SITE_WEIGUAN.md"}
        return d


def cache_path():
    """data/maps rebuild if present, else the committed copy, else None."""
    return next((p for p in (CACHE, COMMITTED) if os.path.exists(p)), None)


def make_collapse(seed=7, start_pile=None, cache=None):
    from .site_weiguan import TruthSupport, make_truth
    cache = cache or cache_path()
    if cache is None:
        cache = build_cache(out=CACHE)
    surface, ground, meta = load_surface(cache)
    fp = meta["footprints"]
    info = meta["meta"]["sector_info"]
    meta.update(geo=dict(GEO), base_xy=list(BASE_XY), drone_pads=[list(p) for p in DRONE_PADS])
    obst = [o["polygon"] for o in meta["meta"].get("obstacles", [])]     # standing buildings: never a sector
    owner, secs = segment(surface, ground, footprints=fp, centres=None, min_h=0.5, min_area=12.0, auto=False,
                          exclude_polys=obst)
    X, Y = surface.mesh()
    cover = surface.z - ground
    order = {n: k for k, n in enumerate(fp)}
    secs.sort(key=lambda s: order[s["name"]])
    piles, public = [], []
    for s in secs:
        m = s["mask"]
        cx, cy = float(X[m].mean()), float(Y[m].mean())
        h = float(cover[m].max())
        piles.append(Pile(name=s["name"], x=round(cx, 3), y=round(cy, 3), h=round(h, 3),
                          sx=round(float(max(2.0, 2 * X[m].std())), 3), sy=round(float(max(2.0, 2 * Y[m].std())), 3),
                          rot=0.0, lump=(0.0, 0.0)))
        public.append({"name": s["name"], "kind": info[s["name"]]["kind"], "usd_prim": info[s["name"]]["usd_prim"],
                       "area_m2": round(float(m.sum()) * RES ** 2, 1), "max_h": round(h, 2),
                       "centre": [round(cx, 2), round(cy, 2)], "polygon": fp[s["name"]],
                       "bbox": [round(float(v), 2) for v in (X[m].min(), Y[m].min(), X[m].max(), Y[m].max())]})
    names = [p.name for p in piles]
    if start_pile is not None and start_pile not in names:
        raise ValueError(f"start_pile {start_pile!r} not a collapse-site sector")
    from .site_mesh import attach_polygons, ccw
    for s in public:
        s["polygon"] = ccw(s["polygon"])
    attach_polygons(piles, {s["name"]: s["polygon"] for s in public}, {s["name"]: s["kind"] for s in public})
    sc = CollapseScenario(seed, piles, {}, surface, ground, cache, meta, public)
    core = [n for n in CORE if n in names]
    sup = TruthSupport(surface, ground, np.where(owner == "", "", owner).astype(str), solids=None, edge_bias=0.7)
    gg = Grid(surface.origin, surface.res, surface.w, surface.h, ground)
    rest = len(names) - len(core)
    sc.truth = make_truth(seed, start_pile, names, sup, gg.height_at, site="collapse", salt=2018,
                          role_sectors=core, start_choices=[n for n in core if n.startswith("T")] or core,
                          silent_victim_p=min(0.35, 8.0 / max(rest, 1)))
    return sc


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Collapse site: build the heightmap cache from the USD; render.")
    ap.add_argument("--build", action="store_true", help="(re)build data/maps/collapse_site_heightmap.npz")
    ap.add_argument("--usd", default=None, help="collapse_site.usda (default: search, or $LARPVERSE_COLLAPSE_SITE)")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--render", help="PNG sector map")
    a = ap.parse_args()
    if a.build:
        build_cache(a.usd)
    sc = make_collapse(a.seed)
    from collections import Counter
    kinds = Counter(s["kind"] for s in sc.public_dict()["sectors"])
    V = sc.truth["victims"]
    print(f"{len(sc.piles)} sectors {dict(kinds)}; {len(V)} victims ({sum(v['depth'] > 4 for v in V)} deep), "
          f"start {sc.truth['start_pile']}, roles { {k: v for k, v in sc.truth['roles'].items() if v != 'silent'} }")
    if a.render:
        from .site_mesh import render
        render(sc, a.render)
