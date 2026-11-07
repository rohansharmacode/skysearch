"""Procedural stand-in for the Isaac Sim rubble scene (SYNTHETIC).

Site frame (INTERFACES.md): x east, y north, z up, metres, origin at the rubble
centre at ground level.  The heightmap cell (ix, iy) has its centre at
``origin + ((ix + 0.5) * res, (iy + 0.5) * res)``; ``z`` is row-major, y outer.

GROUND-TRUTH BOUNDARY: ``World.targets`` is hidden truth.  Only
``sim/radar_model.py`` (observation generator) and ``sim/evaluate.py`` may read
it.  Everything else uses ``World.grid`` / ``World.sample_pointcloud``.

Isaac swap: replace ``World`` by something that exposes ``grid`` (a ``Grid``
built by rasterising the SLAM point cloud, see ``heightmap_from_points``) and
``sample_pointcloud``; the hidden-target part disappears (real sensors).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Grid:
    """Regular 2D grid (heightmap, belief...) on the site."""

    origin: tuple
    res: float
    w: int
    h: int
    z: np.ndarray  # (h, w) heights in metres

    def xs(self):
        return self.origin[0] + (np.arange(self.w) + 0.5) * self.res

    def ys(self):
        return self.origin[1] + (np.arange(self.h) + 0.5) * self.res

    def mesh(self):
        X, Y = np.meshgrid(self.xs(), self.ys())
        return X, Y

    def index(self, x, y):
        """Nearest cell (ix, iy), clamped to the grid."""
        ix = np.clip(np.floor((np.asarray(x) - self.origin[0]) / self.res).astype(int), 0, self.w - 1)
        iy = np.clip(np.floor((np.asarray(y) - self.origin[1]) / self.res).astype(int), 0, self.h - 1)
        return ix, iy

    def height_at(self, x, y):
        """Bilinear surface height at (x, y); scalars or arrays."""
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        fx = (x - self.origin[0]) / self.res - 0.5
        fy = (y - self.origin[1]) / self.res - 0.5
        fx = np.clip(fx, 0, self.w - 1.000001)
        fy = np.clip(fy, 0, self.h - 1.000001)
        i0 = np.floor(fx).astype(int)
        j0 = np.floor(fy).astype(int)
        tx = fx - i0
        ty = fy - j0
        z = self.z
        out = (z[j0, i0] * (1 - tx) * (1 - ty) + z[j0, i0 + 1] * tx * (1 - ty)
               + z[j0 + 1, i0] * (1 - tx) * ty + z[j0 + 1, i0 + 1] * tx * ty)
        return float(out) if out.ndim == 0 else out

    @property
    def bounds(self):
        return [self.origin[0], self.origin[1],
                self.origin[0] + self.w * self.res, self.origin[1] + self.h * self.res]


def heightmap_from_points(pts, origin, res, w, h):
    """Rasterise a point cloud (N,3) into a max-height map, filling holes.

    This is how the Isaac/SLAM cloud becomes the planning heightmap.
    """
    ix = np.clip(((pts[:, 0] - origin[0]) / res).astype(int), 0, w - 1)
    iy = np.clip(((pts[:, 1] - origin[1]) / res).astype(int), 0, h - 1)
    z = np.full((h, w), -np.inf)
    np.maximum.at(z, (iy, ix), pts[:, 2])
    for _ in range(8):
        hole = ~np.isfinite(z)
        if not hole.any():
            break
        zp = np.pad(np.where(hole, np.nan, z), 1, constant_values=np.nan)
        nb = np.stack([zp[1 + dy:1 + dy + h, 1 + dx:1 + dx + w]
                       for dy in (-1, 0, 1) for dx in (-1, 0, 1)])
        with np.errstate(all="ignore"):
            fill = np.nanmean(nb, axis=0)
        z = np.where(hole & np.isfinite(fill), fill, z)
    return Grid(tuple(origin), res, w, h, np.where(np.isfinite(z), z, 0.0).astype(np.float32))


def face_points(g: Grid, rng, n_face: int, mask=None) -> np.ndarray:
    """LiDAR-like points on the vertical faces of the heightmap (cell edges with a > 0.25 m step).

    ``mask`` (h, w bool, optional) restricts the faces to cells where it is true.
    """
    z = g.z
    faces = []
    for axis in (0, 1):
        dz = np.abs(np.diff(z, axis=axis))
        big = dz > 0.25
        if mask is not None:
            big &= mask[:-1, :] if axis == 0 else mask[:, :-1]
        jj, ii = np.nonzero(big)
        z1 = z[jj, ii]
        z2 = z[jj + 1, ii] if axis == 0 else z[jj, ii + 1]
        cx = g.origin[0] + (ii + (0.5 if axis == 0 else 1.0)) * g.res
        cy = g.origin[1] + (jj + (1.0 if axis == 0 else 0.5)) * g.res
        faces.append(np.stack([cx, cy, z1, z2], 1))
    faces = np.concatenate(faces)
    if len(faces) == 0 or n_face <= 0:
        return np.zeros((0, 3))
    pick = faces[rng.integers(0, len(faces), n_face)]
    u = rng.uniform(0, 1, n_face)
    return np.stack([pick[:, 0] + rng.normal(0, 0.02, n_face),
                     pick[:, 1] + rng.normal(0, 0.02, n_face),
                     pick[:, 2] + u * (pick[:, 3] - pick[:, 2])], 1)


def _value_noise(rng, h, w, cell, amp):
    """Smooth value noise: random lattice every `cell` cells, bilinear upsampled."""
    gh, gw = h // cell + 3, w // cell + 3
    lat = rng.normal(0.0, 1.0, (gh, gw))
    gy = np.arange(h) / cell
    gx = np.arange(w) / cell
    j0 = np.floor(gy).astype(int)
    i0 = np.floor(gx).astype(int)
    ty = (gy - j0)[:, None]
    tx = (gx - i0)[None, :]
    a = lat[j0][:, i0]
    b = lat[j0][:, i0 + 1]
    c = lat[j0 + 1][:, i0]
    d = lat[j0 + 1][:, i0 + 1]
    return amp * (a * (1 - tx) * (1 - ty) + b * tx * (1 - ty) + c * (1 - tx) * ty + d * tx * ty)


def _rect_mask(X, Y, cx, cy, a, b, theta):
    c, s = np.cos(theta), np.sin(theta)
    u = (X - cx) * c + (Y - cy) * s
    v = -(X - cx) * s + (Y - cy) * c
    return (np.abs(u) < a) & (np.abs(v) < b)


class World:
    """Seeded rubble site: 50x50 m, 0.5 m cells, ~10 m radius / 3 m tall mound."""

    SIZE = 50.0
    RES = 0.5

    def __init__(self, seed: int = 3, mound_radius: float = 10.0, mound_height: float = 3.0):
        self.seed = seed
        rng = np.random.default_rng(seed)
        n = int(self.SIZE / self.RES)
        origin = (-self.SIZE / 2, -self.SIZE / 2)
        grid = Grid(origin, self.RES, n, n, np.zeros((n, n), np.float32))
        X, Y = grid.mesh()
        r = np.hypot(X, Y)
        phi = np.arctan2(Y, X)

        # ---- ground: flat-ish with gentle undulation
        ground = (_value_noise(rng, n, n, 20, 0.05) + _value_noise(rng, n, n, 6, 0.02)
                  + rng.normal(0, 0.006, (n, n)) + 0.004 * X)

        # ---- mound base profile with an irregular outline
        p = rng.uniform(0, 2 * np.pi, 3)
        r_eff = mound_radius * (1 + 0.18 * np.sin(2 * phi + p[0]) + 0.10 * np.sin(3 * phi + p[1])
                                + 0.06 * np.sin(5 * phi + p[2]))
        t = np.clip(r / r_eff, 0, 1)
        base = mound_height * (1 - t ** 1.7) ** 1.4
        mound = base > 0.05
        rough = (_value_noise(rng, n, n, 6, 0.16) + _value_noise(rng, n, n, 3, 0.09)
                 + rng.normal(0, 0.035, (n, n)))
        z = ground + base + np.where(mound, rough * np.clip(base / 0.6, 0, 1), 0.0)

        # ---- steep chunks (blocks of debris)
        for _ in range(13):
            ang = rng.uniform(0, 2 * np.pi)
            rad = rng.uniform(0.5, 9.5)
            m = _rect_mask(X, Y, rad * np.cos(ang), rad * np.sin(ang), rng.uniform(0.35, 0.9),
                           rng.uniform(0.35, 0.9), rng.uniform(0, np.pi))
            z = z + m * (rng.uniform(0.4, 1.3) + rng.normal(0, 0.06, (n, n)))

        # ---- flat slab facets, slightly tilted to follow the local slope
        gy_, gx_ = np.gradient(base, self.RES)
        self._slab_count = 8
        for _ in range(self._slab_count):
            ang = rng.uniform(0, 2 * np.pi)
            rad = rng.uniform(0.5, 8.2)
            cx, cy = rad * np.cos(ang), rad * np.sin(ang)
            m = _rect_mask(X, Y, cx, cy, rng.uniform(1.5, 2.7), rng.uniform(1.2, 2.2), rng.uniform(0, np.pi))
            if not m.any():
                continue
            ci, cj = int((cx - origin[0]) / self.RES), int((cy - origin[1]) / self.RES)
            hc = float(base[cj, ci] + ground[cj, ci]) + rng.uniform(0.0, 0.25)
            tilt = np.tan(np.radians(rng.uniform(0.3, 3.5)))
            a2 = rng.uniform(0, 2 * np.pi)
            sx = 0.35 * gx_[cj, ci] + tilt * np.cos(a2)
            sy = 0.35 * gy_[cj, ci] + tilt * np.sin(a2)
            sigma = rng.uniform(0.004, 0.05)  # per-slab surface roughness
            plane = hc + sx * (X - cx) + sy * (Y - cy)
            z = np.where(m, plane + rng.normal(0, sigma, (n, n)), z)

        # ---- clutter off the mound (crates, vehicles, wall segments)
        keep_out = (X < -16) & (Y < -16)  # staging corner for the drones
        for _ in range(14):
            for _try in range(20):
                cx, cy = rng.uniform(-23, 23, 2)
                if np.hypot(cx, cy) > 13.5 and not (cx < -15 and cy < -15):
                    break
            m = _rect_mask(X, Y, cx, cy, rng.uniform(0.3, 0.9), rng.uniform(0.3, 0.9), rng.uniform(0, np.pi))
            z = z + m * rng.uniform(0.2, 1.0)
        for _ in range(2):
            ang = rng.uniform(0, 2 * np.pi)
            rad = rng.uniform(16, 21)
            cx, cy = rad * np.cos(ang), rad * np.sin(ang)
            if cx < -14 and cy < -14:
                continue
            m = _rect_mask(X, Y, cx, cy, rng.uniform(2.0, 3.5), 0.3, rng.uniform(0, np.pi))
            z = z + m * 1.1
        z = np.where(keep_out, ground, z)

        grid.z = z.astype(np.float32)
        self.grid = grid
        self.seed_rng = rng

        # ---- hidden truth (PRIVATE: radar_model / evaluate only)
        ang = rng.uniform(0, 2 * np.pi)
        rad = rng.uniform(1.5, 6.5)
        tx, ty = rad * np.cos(ang), rad * np.sin(ang)
        self.targets = [
            {"id": "T1", "kind": "buried", "xy": [float(tx), float(ty)],
             "depth": float(rng.uniform(1.0, 2.2)), "breathing_hz": float(rng.uniform(0.2, 0.33)),
             "emits_sound": True},
            {"id": "D1", "kind": "distractor", "moving": True, "radius": float(rng.uniform(15.5, 18.5)),
             "angle0": float(rng.uniform(0, 2 * np.pi)), "speed": 0.5},
        ]

    # ---- public (non-truth) API ------------------------------------------------
    def heightmap(self) -> Grid:
        return self.grid

    def sample_pointcloud(self, n: int, seed: int | None = None) -> np.ndarray:
        """float32 (N,3) surface points: LiDAR/SLAM stand-in with noise and vertical faces."""
        rng = np.random.default_rng(self.seed * 7919 + 1 if seed is None else seed)
        g = self.grid
        n_face = int(0.10 * n)
        n_surf = n - n_face
        n_mound = int(0.55 * n_surf)
        rr = 14.0 * np.sqrt(rng.uniform(0, 1, n_mound))
        aa = rng.uniform(0, 2 * np.pi, n_mound)
        sx = np.concatenate([rr * np.cos(aa), rng.uniform(-25, 25, n_surf - n_mound)])
        sy = np.concatenate([rr * np.sin(aa), rng.uniform(-25, 25, n_surf - n_mound)])
        sz = g.height_at(sx, sy) + rng.normal(0, 0.015, n_surf)
        surf = np.stack([sx, sy, sz], 1)

        face_pts = face_points(g, rng, n_face)
        pts = np.concatenate([surf, face_pts])
        rng.shuffle(pts)
        return pts.astype(np.float32)

    # ---- hidden-truth physics (called by radar_model / evaluate only) -----------
    def target_position(self, tgt: dict, t: float = 0.0):
        """3D position of a target at time t (z below the surface for buried)."""
        if tgt["kind"] == "buried":
            x, y = tgt["xy"]
            return np.array([x, y, self.grid.height_at(x, y) - tgt["depth"]])
        a = tgt["angle0"] + tgt["speed"] * t / tgt["radius"]
        x, y = tgt["radius"] * np.cos(a), tgt["radius"] * np.sin(a)
        return np.array([x, y, self.grid.height_at(x, y) + 0.9])
