"""3x3 rubble-grid world (SYNTHETIC) built from the shared seeded scenario (sim/scenario.py).

Same role as ``World`` in sim/world.py, for the whole site instead of one mound:

* ``GridWorld.grid``               whole-site heightmap (``Scenario.heightmap``), used for flight
* ``GridWorld.sample_pointcloud``  LiDAR/SLAM stand-in sampled from that heightmap
* ``GridWorld.window(pile)``       a planning window (``Grid``) around one pile for landing detection,
                                   the localiser and the planner; cells nearer another pile are flattened
                                   to local ground so neighbouring piles do not confuse them
* ``GridWorld.pile_world(pile)``   a ``World``-like object for the radar generator at that pile

GROUND-TRUTH BOUNDARY: ``targets`` (victims from ``Scenario.truth``) are hidden truth. Only the
observation generators (sim/radar_model.py, sim/acoustic_grid.py) and evaluators may read them; the
coordinator just hands ``pile_world(...)`` to ``make_generator`` exactly as Mission does with ``World``.
Isaac swap: an adapter supplies its own ``grid`` (rasterised SLAM cloud) and real sensor observations.
"""
from __future__ import annotations

import numpy as np

from .scenario import Scenario
from .world import Grid, face_points

WINDOW_M = 30.0        # planning window edge (m); > 2 * 20 m / sqrt(2) so the localiser sees far-field ground


class PileWorld:
    """World-like view of one pile: ``grid`` (planning window), ``targets`` and ``target_position``."""

    def __init__(self, name, grid, targets):
        self.name = name
        self.grid = grid
        self.targets = targets            # HIDDEN: radar generator only

    def target_position(self, tgt, t=0.0):
        x, y = tgt["xy"]
        if tgt.get("z") is not None:              # explicit victim height (multi-storey sites)
            return np.array([x, y, float(tgt["z"])])
        return np.array([x, y, self.grid.height_at(x, y) - tgt["depth"]])


class GridWorld:
    def __init__(self, scenario: Scenario, res: float = 0.5, window_m: float = WINDOW_M):
        self.scenario = scenario
        self.seed = scenario.seed
        self.res = res
        self.window_m = window_m
        self.grid = scenario.heightmap(res)
        self.pile_xy = {p.name: (float(p.x), float(p.y)) for p in scenario.piles}
        # ---- hidden truth (PRIVATE: generators / evaluators only)
        self.targets = [{"id": v["id"], "kind": "buried", "pile": v["pile"], "xy": list(v["xy"]),
                         "depth": float(v["depth"]), "breathing_hz": float(v["breathing_hz"]),
                         "emits_sound": v.get("sound") is not None,
                         "z": None if v.get("z") is None else float(v["z"])}
                        for v in scenario.truth.get("victims", [])]

    # ---- public (non-truth) API ------------------------------------------------
    def heightmap(self) -> Grid:
        return self.grid

    def window(self, pile: str, z_source: Grid | None = None) -> Grid:
        """Planning window (copy) centred on ``pile``; other piles' Voronoi cells set to local ground."""
        g = z_source or self.grid
        cx, cy = self.pile_xy[pile]
        n = int(round(self.window_m / g.res))
        i0 = int(round((cx - self.window_m / 2 - g.origin[0]) / g.res))
        j0 = int(round((cy - self.window_m / 2 - g.origin[1]) / g.res))
        i0 = min(max(i0, 0), g.w - n)
        j0 = min(max(j0, 0), g.h - n)
        z = g.z[j0:j0 + n, i0:i0 + n].astype(np.float32).copy()
        w = Grid((g.origin[0] + i0 * g.res, g.origin[1] + j0 * g.res), g.res, n, n, z)
        X, Y = w.mesh()
        d_own = np.hypot(X - cx, Y - cy)
        d_other = np.min([np.hypot(X - x, Y - y) for name, (x, y) in self.pile_xy.items() if name != pile], axis=0)
        theirs = d_other < d_own
        ring = (~theirs) & (d_own > 11.0)
        ground = float(np.median(z[ring])) if ring.any() else float(np.median(z))
        w.z = np.where(theirs, np.minimum(z, ground), z).astype(np.float32)
        return w

    def pile_world(self, pile: str, grid: Grid | None = None) -> PileWorld:
        return PileWorld(pile, grid if grid is not None else self.window(pile),
                         [t for t in self.targets if t["pile"] == pile])

    def target_position(self, tgt, t=0.0):
        x, y = tgt["xy"]
        if tgt.get("z") is not None:              # explicit victim height (multi-storey sites)
            return np.array([x, y, float(tgt["z"])])
        return np.array([x, y, self.grid.height_at(x, y) - tgt["depth"]])

    def sample_pointcloud(self, n: int, seed: int | None = None, pile_frac: float = 0.62,
                          pile_radius: float = 9.5) -> np.ndarray:
        """float32 (N,3) surface points over the whole site, denser on the piles, plus vertical faces."""
        rng = np.random.default_rng(self.seed * 7919 + 5 if seed is None else seed)
        g = self.grid
        n_face = int(0.12 * n)
        n_surf = n - n_face
        n_pile = int(pile_frac * n_surf)
        centres = np.array(list(self.pile_xy.values()))
        k = rng.integers(0, len(centres), n_pile)
        rr = pile_radius * np.sqrt(rng.uniform(0, 1, n_pile))
        aa = rng.uniform(0, 2 * np.pi, n_pile)
        x0, y0, x1, y1 = g.bounds
        sx = np.concatenate([centres[k, 0] + rr * np.cos(aa), rng.uniform(x0, x1, n_surf - n_pile)])
        sy = np.concatenate([centres[k, 1] + rr * np.sin(aa), rng.uniform(y0, y1, n_surf - n_pile)])
        sz = g.height_at(sx, sy) + rng.normal(0, 0.015, n_surf)
        X, Y = g.mesh()
        near = np.min([np.hypot(X - cx, Y - cy) for cx, cy in centres], axis=0) < pile_radius + 1.0
        pts = np.concatenate([np.stack([sx, sy, sz], 1), face_points(g, rng, n_face, mask=near)])
        rng.shuffle(pts)
        return pts.astype(np.float32)
