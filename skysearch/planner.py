"""Probability-weighted coverage control (Lloyd / Voronoi) + target selection + A* around hazards."""
from __future__ import annotations
import heapq
import numpy as np

from .config import Config


class Partition:
    """Each live drone owns a Voronoi region around an anchor; anchors drift to the
    probability-weighted centroid of their region (Cortes et al. coverage control)."""

    def __init__(self, cfg: Config, anchors: dict[int, np.ndarray]):
        self.cfg = cfg
        self.anchors = {k: v.astype(float) for k, v in anchors.items()}
        c = (np.arange(cfg.n) + 0.5) * cfg.cell
        self.X, self.Y = np.meshgrid(c, c)
        self.label = np.zeros((cfg.n, cfg.n), int)

    def update(self, alive: list[int], post, hazard, step=0.5):
        for k in list(self.anchors):
            if k not in alive:
                del self.anchors[k]
        if not self.anchors:
            return
        ids = list(self.anchors)
        A = np.array([self.anchors[i] for i in ids])
        d2 = (self.X[None] - A[:, 0, None, None]) ** 2 + (self.Y[None] - A[:, 1, None, None]) ** 2
        lab = np.argmin(d2, 0)
        self.label = np.array(ids)[lab]
        w = post * ~hazard
        for j, i in enumerate(ids):
            m = (lab == j)
            s = w[m].sum()
            if s > 1e-12:
                cen = np.array([(self.X[m] * w[m]).sum() / s, (self.Y[m] * w[m]).sum() / s])
                self.anchors[i] += step * (cen - self.anchors[i])


def pick_target(cfg: Config, did, pos, post, canopy, hazard, partition: Partition, taken):
    from .belief import pd_model
    own = partition.label == did
    if not own.any():
        own = np.ones_like(own)
    pdm = 0.6 * (1 - 0.9 * canopy) * np.exp(-max(cfg.search_alt - 20, 0) / 140)
    d = np.hypot(partition.X - pos[0], partition.Y - pos[1])
    score = post * pdm / (1.0 + d / 250.0)
    score = np.where(own & ~hazard, score, -1)
    for t in taken:
        r, c = int(t[1] // cfg.cell), int(t[0] // cfg.cell)
        score[max(r - 2, 0):r + 3, max(c - 2, 0):c + 3] = -1
    r, c = np.unravel_index(np.argmax(score), score.shape)
    return np.array([(c + 0.5) * cfg.cell, (r + 0.5) * cfg.cell])


def segment_hits_hazard(cfg, a, b, hazard):
    n = int(np.linalg.norm(b - a) / (cfg.cell / 2)) + 2
    for t in np.linspace(0, 1, n):
        p = a + t * (b - a)
        r, c = int(np.clip(p[1] // cfg.cell, 0, cfg.n - 1)), int(np.clip(p[0] // cfg.cell, 0, cfg.n - 1))
        if hazard[r, c]:
            return True
    return False


def astar(cfg: Config, a, b, hazard):
    n = cfg.n
    s = (int(a[1] // cfg.cell), int(a[0] // cfg.cell))
    g = (int(b[1] // cfg.cell), int(b[0] // cfg.cell))
    s = tuple(np.clip(s, 0, n - 1)); g = tuple(np.clip(g, 0, n - 1))
    openq = [(0.0, s)]
    came, cost = {s: None}, {s: 0.0}
    while openq:
        _, cur = heapq.heappop(openq)
        if cur == g:
            break
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == dc == 0:
                    continue
                nb = (cur[0] + dr, cur[1] + dc)
                if not (0 <= nb[0] < n and 0 <= nb[1] < n) or (hazard[nb] and nb != g):
                    continue
                nc = cost[cur] + (1.414 if dr and dc else 1.0)
                if nc < cost.get(nb, 1e18):
                    cost[nb], came[nb] = nc, cur
                    heapq.heappush(openq, (nc + np.hypot(nb[0] - g[0], nb[1] - g[1]), nb))
    if g not in came:
        return [b]
    path, cur = [], g
    while cur is not None:
        path.append(np.array([(cur[1] + 0.5) * cfg.cell, (cur[0] + 0.5) * cfg.cell]))
        cur = came[cur]
    path.reverse()
    return path[1:] + [b]
