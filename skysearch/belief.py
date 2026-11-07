"""Bayesian search map (Koopman search theory).

Each drone keeps its own copy of `logmiss`: the accumulated log-probability that a
person in each cell would have been missed by every look so far. Posterior = prior *
exp(logmiss), normalised. Copies are fused by elementwise minimum when drones can
talk, which never double-counts a look (conservative under overlap).
"""
from __future__ import annotations
import math
import numpy as np
from scipy.ndimage import gaussian_filter

from .config import Config


def lost_person_prior(cfg: Config, lkp, canopy_prior):
    """Ring-shaped prior around the last known point (lost-person behaviour statistics):
    lognormal distance (median ~380 m) and a mild preference for denser cover."""
    n = cfg.n
    c = (np.arange(n) + 0.5) * cfg.cell
    X, Y = np.meshgrid(c, c)
    d = np.hypot(X - lkp[0], Y - lkp[1]) + 1.0
    mu, s = math.log(380), 0.55
    pdf = np.exp(-((np.log(d) - mu) ** 2) / (2 * s * s)) / (d * d)  # per-area density
    pdf *= (0.7 + 0.6 * canopy_prior)
    return pdf / pdf.sum()


def pd_model(z, canopy):
    """Swarm's own model of per-look thermal detection probability. Mirrors physics, not truth."""
    return 0.6 * (1 - 0.9 * canopy) * math.exp(-max(z - 20, 0) / 140)


class Belief:
    def __init__(self, cfg: Config, prior, canopy_est):
        self.cfg = cfg
        self.prior = prior
        self.canopy = canopy_est.copy()        # refined by LiDAR as drones overfly
        self.logmiss = np.zeros_like(prior)

    def observe_footprint(self, xy, z, r):
        """Negative information: we looked and saw no person here."""
        cfg = self.cfg
        n = cfg.n
        c0 = int(max((xy[0] - r) // cfg.cell, 0)); c1 = int(min((xy[0] + r) // cfg.cell, n - 1))
        r0 = int(max((xy[1] - r) // cfg.cell, 0)); r1 = int(min((xy[1] + r) // cfg.cell, n - 1))
        cc = (np.arange(c0, c1 + 1) + 0.5) * cfg.cell
        rr = (np.arange(r0, r1 + 1) + 0.5) * cfg.cell
        X, Y = np.meshgrid(cc, rr)
        inside = (X - xy[0]) ** 2 + (Y - xy[1]) ** 2 <= r * r
        alt_f = math.exp(-max(z - 20, 0) / 140)
        pd = 0.6 * (1 - 0.9 * self.canopy[r0:r1 + 1, c0:c1 + 1]) * alt_f
        self.logmiss[r0:r1 + 1, c0:c1 + 1] += np.where(inside, np.log1p(-np.clip(pd, 0, 0.99)), 0)

    def bump(self, xy, bearing, rng_m=140, factor=1.2):
        """Positive cue (heard a voice on a bearing): raise belief in a cone."""
        cfg = self.cfg
        c = (np.arange(cfg.n) + 0.5) * cfg.cell
        X, Y = np.meshgrid(c, c)
        dx, dy = X - xy[0], Y - xy[1]
        d = np.hypot(dx, dy)
        ang = np.abs((np.arctan2(dy, dx) - bearing + np.pi) % (2 * np.pi) - np.pi)
        cone = (d < rng_m) & (ang < 0.6)
        self.logmiss[cone] += factor   # positive log-evidence

    def clear_around(self, xy, radius=40):
        cfg = self.cfg
        c = (np.arange(cfg.n) + 0.5) * cfg.cell
        X, Y = np.meshgrid(c, c)
        self.logmiss[np.hypot(X - xy[0], Y - xy[1]) < radius] -= 8.0

    def decay(self, k=0.995):
        """People move: old negative evidence slowly loses weight."""
        self.logmiss *= k

    def posterior(self):
        p = self.prior * np.exp(np.clip(self.logmiss, -30, 30))
        return p / p.sum()

    def pos(self):
        """Cumulative probability of success so far (prior mass we would have detected)."""
        return float((self.prior * (1 - np.exp(np.minimum(self.logmiss, 0)))).sum())

    @staticmethod
    def fuse(beliefs):
        if len(beliefs) < 2:
            return
        lm = np.minimum.reduce([b.logmiss for b in beliefs])
        # positive bumps should survive fusion: take max of positive parts
        pos = np.maximum.reduce([np.maximum(b.logmiss, 0) for b in beliefs])
        fused = np.where(pos > 0, pos, lm)
        can = np.mean([b.canopy for b in beliefs], axis=0)
        for b in beliefs:
            b.logmiss = fused.copy()
            b.canopy = can.copy()
