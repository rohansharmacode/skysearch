"""Ground-truth world + simulated sensors.

Nothing in skysearch/brain.py or planner.py may read this module's ground truth
directly: the swarm only sees what the sensor functions here return. The same
sensor interface is implemented against Isaac Sim in isaac/isaac_bridge.py.
"""
from __future__ import annotations
import math
import numpy as np
from scipy.ndimage import gaussian_filter

from .config import Config


class World:
    def __init__(self, cfg: Config, rng: np.random.Generator):
        self.cfg, self.rng = cfg, rng
        n = cfg.n
        # canopy density 0..0.85, smooth forest with clearings
        raw = gaussian_filter(rng.random((n, n)), sigma=4)
        raw = (raw - raw.min()) / (raw.max() - raw.min())
        self.canopy = np.clip(raw * 1.15 - 0.1, 0.0, 0.85)
        # "satellite" canopy the swarm starts with: blurred and biased
        self.canopy_prior = np.clip(gaussian_filter(self.canopy, 3) + rng.normal(0, 0.12, (n, n)), 0, 0.9)
        # individual trees for LiDAR (density follows canopy)
        k = 2500
        p = self.canopy.flatten() + 1e-6
        idx = rng.choice(n * n, size=k, p=p / p.sum())
        cy, cx = np.divmod(idx, n)
        self.trees = np.stack([(cx + rng.random(k)) * cfg.cell, (cy + rng.random(k)) * cfg.cell], 1)
        self.tree_r = rng.uniform(1.0, 2.5, k)

        self.base = np.array([60.0, 60.0])
        self.lkp = np.array([320.0, 300.0])  # last known point (car park / trailhead)
        self.people = []
        for i in range(cfg.n_people):
            # Lost-person-behaviour style distance from LKP: lognormal, median ~380 m
            for _ in range(100):
                d = rng.lognormal(math.log(380), 0.45)
                a = rng.uniform(0, 2 * math.pi)
                pos = self.lkp + d * np.array([math.cos(a), math.sin(a)])
                if np.all(pos > 40) and np.all(pos < cfg.area - 40):
                    break
            self.people.append(dict(pos=pos, moving=(i == 1), calls=(i == 1),
                                    heading=rng.uniform(0, 2 * math.pi), found_t=None))
        self.deer = [dict(pos=rng.uniform(80, cfg.area - 80, 2), heading=rng.uniform(0, 2 * math.pi))
                     for _ in range(cfg.n_deer)]
        self.fire = np.zeros((n, n), bool)

    # ---------------- helpers ----------------
    def cell_of(self, xy):
        c = self.cfg.cell
        n = self.cfg.n
        return (int(np.clip(xy[1] // c, 0, n - 1)), int(np.clip(xy[0] // c, 0, n - 1)))

    def ignite(self, xy):
        r, c = self.cell_of(xy)
        self.fire[max(r - 1, 0):r + 2, max(c - 1, 0):c + 2] = True

    # ---------------- dynamics ----------------
    def step(self, t: float):
        dt = self.cfg.dt
        for p in self.people:
            if p["moving"] and p["found_t"] is None:
                p["heading"] += self.rng.normal(0, 0.15)
                p["pos"] = np.clip(p["pos"] + 0.5 * dt * np.array([math.cos(p["heading"]), math.sin(p["heading"])]),
                                   20, self.cfg.area - 20)
        for d in self.deer:
            d["heading"] += self.rng.normal(0, 0.3)
            d["pos"] = np.clip(d["pos"] + 1.2 * dt * np.array([math.cos(d["heading"]), math.sin(d["heading"])]),
                               20, self.cfg.area - 20)
        if self.fire.any() and int(t / dt) % 4 == 0:  # spread every 2 s
            f = self.fire
            nb = np.zeros_like(f)
            nb[1:, :] |= f[:-1, :]; nb[:-1, :] |= f[1:, :]
            nb[:, 1:] |= f[:, :-1]; nb[:, :-1] |= f[:, 1:]
            spread = nb & ~f & (self.rng.random(f.shape) < 0.05 + 0.12 * self.canopy)
            self.fire |= spread

    # ---------------- sensors ----------------
    def footprint_radius(self, z):
        return z * math.tan(math.radians(self.cfg.fov_deg / 2))

    def thermal(self, pos3):
        """Thermal camera: list of hot blobs (x, y, size_px). People and deer both look hot."""
        x, y, z = pos3
        r = self.footprint_radius(z)
        out = []
        alt_f = math.exp(-max(z - 20, 0) / 140)
        for src, is_person in [(p, True) for p in self.people] + [(d, False) for d in self.deer]:
            dx, dy = src["pos"] - (x, y)
            if dx * dx + dy * dy > r * r:
                continue
            can = self.canopy[self.cell_of(src["pos"])]
            pd = 0.6 * (1 - 0.9 * can) * alt_f
            if self.rng.random() < pd:
                noise = 1.5 + 0.05 * z
                size = (9 if is_person else 11) * 40 / max(z, 10) + self.rng.normal(0, 1.5)
                out.append((src["pos"][0] + self.rng.normal(0, noise), src["pos"][1] + self.rng.normal(0, noise), size))
        if self.rng.random() < 0.004:  # sun-warmed rock false positive
            a, d = self.rng.uniform(0, 2 * math.pi), self.rng.uniform(0, r)
            out.append((x + d * math.cos(a), y + d * math.sin(a), self.rng.uniform(3, 9)))
        return out

    def rgb_person_score(self, pos3, target_xy):
        """RGB classifier score that the blob at target_xy is a person. Informative only when low."""
        x, y, z = pos3
        best, kind = 1e9, None
        for p in self.people:
            d = np.linalg.norm(p["pos"] - target_xy)
            if d < best:
                best, kind = d, "person"
        for d_ in self.deer:
            d = np.linalg.norm(d_["pos"] - target_xy)
            if d < best:
                best, kind = d, "deer"
        horiz = np.linalg.norm(np.array([x, y]) - target_xy)
        a = float(np.clip((60 - z) / 42, 0, 1)) * (1.0 if horiz < 15 else 0.3)
        if best > 12:
            kind = "rock"
        can = self.canopy[self.cell_of(target_xy)]
        a *= (1 - 0.5 * can)
        mean = 0.5 + a * (0.42 if kind == "person" else -0.38)
        return float(np.clip(mean + self.rng.normal(0, 0.1), 0.01, 0.99))

    def microphone(self, pos3):
        """Detects a conscious person calling for help. Returns noisy bearing (rad) or None."""
        x, y, z = pos3
        for p in self.people:
            if not p["calls"] or p["found_t"] is not None:
                continue
            d = np.linalg.norm(p["pos"] - (x, y))
            if d < 140 and z < 70 and self.rng.random() < 0.03:
                dx, dy = p["pos"] - (x, y)
                return math.atan2(dy, dx) + self.rng.normal(0, 0.35)
        return None

    def lidar_canopy(self, pos3):
        """Downward LiDAR: measured canopy density of the cell below (fraction of returns from canopy)."""
        rc = self.cell_of(pos3[:2])
        return rc, float(np.clip(self.canopy[rc] + self.rng.normal(0, 0.04), 0, 1))

    def lidar_scan(self, pos3, n_rays=24):
        """Horizontal LiDAR ring: distance to nearest trunk per ray (only matters below tree tops)."""
        x, y, z = pos3
        rng_ = self.cfg.lidar_range
        dists = np.full(n_rays, rng_)
        if z > self.cfg.tree_height + 2:
            return dists
        rel = self.trees - (x, y)
        near = np.where(np.einsum("ij,ij->i", rel, rel) < (rng_ + 3) ** 2)[0]
        angs = np.linspace(0, 2 * math.pi, n_rays, endpoint=False)
        dirs = np.stack([np.cos(angs), np.sin(angs)], 1)
        for i in near:
            proj = dirs @ rel[i]
            perp2 = rel[i] @ rel[i] - proj ** 2
            hit = (proj > 0) & (perp2 < self.tree_r[i] ** 2)
            d = proj - np.sqrt(np.maximum(self.tree_r[i] ** 2 - perp2, 0))
            dists = np.where(hit & (d < dists), np.maximum(d, 0), dists)
        return dists
