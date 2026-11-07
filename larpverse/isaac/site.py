"""Per-scenario site values for the Isaac adapter (pure python: safe in --dry-run without omni/pxr).

The grid site keeps the module constants of sim/scenario.py; the Weiguan and external-map sites carry their
own ``bounds`` / ``base_xy`` / ``drone_pads`` / ``geo`` (sim/site_weiguan.py, sim/site_mesh.py).

``mission_for_site`` builds a GridMission on any site: natively when GridMission accepts ``site=``, else
through a scoped shim (the scenario factory and pads are swapped only while the constructor runs; the
written state gets the scenario's bounds, staging and geo). Remove the shim once sim/grid_mission.py has
``site=`` (the native path is taken automatically).
"""
from __future__ import annotations

import inspect


def site_name(sc) -> str:
    return getattr(sc, "site", "grid")


def bounds(sc):
    from sim.scenario import SITE_BOUNDS
    return tuple(getattr(sc, "bounds", SITE_BOUNDS))


def base_xy(sc):
    from sim.scenario import BASE_XY
    return tuple(getattr(sc, "base_xy", BASE_XY))


def drone_pads(sc):
    from sim.scenario import DRONE_PADS
    return [tuple(p) for p in getattr(sc, "drone_pads", DRONE_PADS)]


def geo(sc):
    from sim.scenario import SITE_GEO
    return dict(getattr(sc, "geo", SITE_GEO))


def victim_z(sc, v, surface=None):
    """Absolute height of a victim (display only): ``z`` when the site gives it, else the grid convention."""
    if "z" in v:
        return float(v["z"])
    x, y = v["xy"]
    fn = surface or sc.ground_height
    return float(fn(x, y)) - float(v["depth"])


def mission_for_site(site: str, log=None, **kw):
    """GridMission on ``site`` (grid | weiguan | collapse | mesh:<path>); kw go to the GridMission constructor.
    Keywords this GridMission does not accept yet (e.g. ``delivery`` before the mission agent adds it) are
    dropped with a log line, so the adapter runs against older and newer mission code alike."""
    import sim.grid_mission as gm
    params = inspect.signature(gm.GridMission.__init__).parameters
    dropped = [k for k in kw if k not in params]
    for k in dropped:
        kw.pop(k)
    if dropped and log:
        log(f"GridMission does not take {dropped} yet; ignoring them")
    if site == "grid" or "site" in params:
        return gm.GridMission(**({"site": site} if site != "grid" else {}), **kw)
    from sim.scenario import make_scenario as _make

    class SiteMission(gm.GridMission):
        def _state(self, *a, **k):
            st = super()._state(*a, **k)
            sc = self.scenario
            st["site"] = {**st.get("site", {}), "name": f"{_label(sc)} · seed {self.seed}",
                          "bounds": list(bounds(sc)), "geo": geo(sc), "kind": site_name(sc)}
            st["staging"] = {"xy": list(base_xy(sc)), "pads": [list(p) for p in drone_pads(sc)]}
            return st

    saved = gm.make_scenario, gm.DRONE_PADS
    seed, start = kw.get("seed", 7), kw.get("start_pile")
    sc = _make(seed, start, site=site)
    gm.make_scenario = lambda *_a, **_k: sc
    gm.DRONE_PADS = drone_pads(sc)
    try:
        return SiteMission(**kw)
    finally:
        gm.make_scenario, gm.DRONE_PADS = saved


def _label(sc):
    s = site_name(sc)
    if s == "weiguan":
        return "Weiguan Jinlong collapse (procedural stand-in)"
    if s == "mesh":
        import os
        return f"Map {os.path.basename(getattr(sc, 'source', 'mesh'))}"
    return "Rubble grid 3x3"
