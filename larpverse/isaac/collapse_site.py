"""``--site collapse``: the 3d_model collapse map (``collapse_site/collapse_site.usda``) as a mission site.

Mission side (numpy + scipy only, so the CPU dry run, ``--replay`` and Isaac all build the same scenario):
  the planner's public surface is ``isaac/maps/collapse_core_0p5m.npz``: the map's terrain tiles + buildings
  rasterised max-z at 0.5 m over the core (-80..80 m; debris instances and the 2 km horizon apron left out).
  Search sectors Z1..Z8 are the wreck cells within SECTOR_R of fixed centres (the two fallen 16-storey
  towers, the rubble field between them, the NW hall), Voronoi-split; victims are placed by the team's
  ``site_weiguan.make_truth`` exactly as for ``mesh:`` sites. Staging is on the street south of the wreck.
Isaac side: ``add_to_stage`` references the USD at /World/CollapseSite (its own colliders, sun and sky) and
  hides the scenario's heightfield terrain (kept as a collider; it is the same surface at 0.5 m).

The ``--site collapse --site-usd <repo>/collapse_site/collapse_site.usda`` interface matches ISAAC_BRIEF §3.
"""
from __future__ import annotations

import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SURFACE = os.path.join(HERE, "maps", "collapse_core_0p5m.npz")
SECTORS = {                      # name -> centre (m, site frame); see collapse_site/previews/hero_top.jpg
    "Z1": (-28.0, 14.0),         # west fallen tower, north half
    "Z2": (-28.0, -14.0),        # west fallen tower, south half
    "Z3": (-10.0, 12.0),         # rubble field between the towers, north
    "Z4": (-8.0, -14.0),         # rubble field, south
    "Z5": (12.0, 14.0),          # east fallen tower, north half
    "Z6": (12.0, -14.0),         # east fallen tower, south half
    "Z7": (-18.0, 38.0),         # NW damaged hall
    "Z8": (-46.0, -25.0),        # wreck block west of the south tower
}
SECTOR_R = 8.0                   # compact sectors (like the grid piles): 16 m left the radar with two far modes
BASE_XY = (4.0, -63.0)
DRONE_PADS = [(1.4, -65.0), (4.0, -65.0), (6.6, -65.0)]
USD = os.environ.get("LARPVERSE_COLLAPSE_USD", "")       # set by run_mission --site-usd


def default_usd():
    cands = [USD, os.path.join(HERE, "..", "..", "collapse_site", "collapse_site.usda"),
             os.path.join(HERE, "..", "collapse_site", "collapse_site.usda")]
    return next((os.path.abspath(c) for c in cands if c and os.path.exists(c)), "")


CORE_SITE = "collapse-core"     # our compact 8-sector scenario on the same map (case study, isaac/eval_collapse.py)


def is_collapse(site) -> bool:
    """Either scenario on the 3d_model map: ``collapse`` (the team's sim/site_collapse.py, 157 sectors, staging
    (-56, -152.5)) or ``collapse-core`` (ours below: sectors Z1-Z8, staging on the south street)."""
    return str(site or "").startswith("collapse")


def is_core(site) -> bool:
    return str(site or "") == CORE_SITE


def make_collapse_scenario(seed=7, start_pile=None):
    from scipy import ndimage
    from sim.site_mesh import MeshScenario, estimate_ground, load_surface
    from sim.site_weiguan import TruthSupport, make_truth
    from sim.scenario import Pile
    from sim.world import Grid
    surface, _, _ = load_surface(SURFACE)
    ground = np.minimum(estimate_ground(surface.z.astype(float), surface.res), surface.z)
    X, Y = surface.mesh()
    cover = surface.z - ground
    above = ndimage.binary_closing(ndimage.binary_opening(cover > 1.0), iterations=2) & (cover > 0.3)
    names = list(SECTORS)
    C = np.array([SECTORS[n] for n in names], float)
    d2 = (X[..., None] - C[:, 0]) ** 2 + (Y[..., None] - C[:, 1]) ** 2
    near = np.argmin(d2, -1)
    inside = above & (np.sqrt(np.min(d2, -1)) <= SECTOR_R)
    owner = np.full(surface.z.shape, "", dtype=object)
    piles, public = [], []
    for i, n in enumerate(names):
        m = inside & (near == i)
        if m.sum() * surface.res ** 2 < 20.0:
            continue
        owner[m] = n
        cx, cy, h = float(X[m].mean()), float(Y[m].mean()), float(cover[m].max())
        piles.append(Pile(name=n, x=round(cx, 3), y=round(cy, 3), h=round(h, 3), sx=round(float(max(2.0, 2 * X[m].std())), 3),
                          sy=round(float(max(2.0, 2 * Y[m].std())), 3), rot=0.0, lump=(0.0, 0.0)))
        public.append({"name": n, "source": "centre", "area_m2": round(float(m.sum()) * surface.res ** 2, 1),
                       "max_h": round(h, 2), "bbox": [round(float(v), 2) for v in (X[m].min(), Y[m].min(), X[m].max(), Y[m].max())]})
    names = [p.name for p in piles]
    if start_pile is not None and start_pile not in names:
        raise ValueError(f"start_pile {start_pile!r} not in sectors {names}")
    meta = {"base_xy": list(BASE_XY), "drone_pads": [list(p) for p in DRONE_PADS]}
    sc = MeshScenario(seed, piles, {}, surface, ground, SURFACE, meta, public)
    sc.site = CORE_SITE
    sup = TruthSupport(surface, ground, owner.astype(str), solids=None, edge_bias=0.7)
    gg = Grid(surface.origin, surface.res, surface.w, surface.h, ground)
    sc.truth = make_truth(seed, start_pile, names, sup, gg.height_at, site="mesh", salt=2017)
    return sc


def add_to_stage(stage, usd=None, log=print, collide_terrain=True):
    """Reference the collapse map into the mission stage and hide the planner's stand-in terrain."""
    from pxr import UsdGeom
    usd = usd or default_usd()
    if not usd:
        raise FileNotFoundError("--site collapse needs --site-usd <repo>/collapse_site/collapse_site.usda (3d_model)")
    prim = stage.DefinePrim("/World/CollapseSite", "Xform")
    prim.GetReferences().AddReference(os.path.abspath(usd))
    hidden = []
    for p in ("/World/Terrain", "/World/Apron"):
        q = stage.GetPrimAtPath(p)
        if q and q.IsValid():
            UsdGeom.Imageable(q).MakeInvisible()
            hidden.append(p)
            if not collide_terrain:
                q.SetActive(False)
    for p in ("/World/Sun", "/World/Sky"):            # the map's own sun (ESE 38 deg, matches its baked shadows) + sky
        q = stage.GetPrimAtPath(p)
        if q and q.IsValid():
            q.SetActive(False)
    log(f"[collapse] referenced {usd} at /World/CollapseSite; hid {hidden} (colliders {'kept' if collide_terrain else 'off'}); "
        "using the map's sun and sky")
    return prim


def main(argv=None):
    """CPU story for the dashboard and for Isaac --replay:  python -m isaac.collapse_site --seed 9 --out DIR"""
    import argparse
    ap = argparse.ArgumentParser(description=main.__doc__)
    ap.add_argument("--seed", type=int, default=9, help="9: pings at t 95 (Z3) and 200 (Z4), dispatches 121/226")
    ap.add_argument("--start-pile", default=None)
    ap.add_argument("--radar", default="dsp")
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-t", type=float, default=900.0)
    a = ap.parse_args(argv)
    from isaac.site import mission_for_site
    m = mission_for_site(CORE_SITE, seed=a.seed, start_pile=a.start_pile, radar=a.radar, out_dir=a.out, frames=True)
    while not m.done and m.t < a.max_t:
        m.step()
    print(f"collapse site seed {a.seed}: t {m.t:.0f} s, frames in {a.out}/frames")


if __name__ == "__main__":
    main()
