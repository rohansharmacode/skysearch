"""Grid Bayesian filter on the 2D (x, y) search layer.

Never sees ground truth: it takes observation dicts (INTERFACES.md) and the
heightmap only.

Radar likelihood per observation (stationary):
    L(cell) = eps + sum_k  w_k * exp(-0.5 * ((r_k - r(cell)) / sigma(cell))^2),  w_k = confidence_k * quality
i.e. a Gaussian ring around the sensor for every range candidate plus a uniform
outlier floor `eps`.  r(cell) is the 3D slant range to the cell assuming a nominal
burial depth below the heightmap surface; sigma(cell) adds the depth uncertainty.
Posterior = prior * prod(L) in log space.  Observations with doppler_moving=true
never touch this belief; they feed a separate random-walk moving-track belief.
An empty candidate list is "no detection" and is NOT evidence of absence.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

from .world import Grid


@dataclass
class LocConfig:
    depth_nom: float = 1.5        # nominal burial depth below the surface (m)
    sigma_depth: float = 0.5      # depth uncertainty (m)
    eps: float = 0.015            # uniform outlier floor of the likelihood
    prior_floor: float = 0.08     # prior mass outside the rubble footprint (relative)
    dominant_min: float = 0.70    # dominant mode must hold this p_mass
    significant_min: float = 0.15 # a second mode this heavy means ambiguity
    max_sigma: float = 2.0        # dominant mode semi-major sigma limit (m)
    mode_radius: float = 4.0      # mass assignment radius around a mode (m)
    merge_dist: float = 2.0       # modes closer than this are merged (m)
    min_rel: float = 0.30         # peak must exceed this fraction of the global peak
    min_report_mass: float = 0.05
    ac_weight: float = 0.7
    ac_eps: float = 0.35
    move_q: float = 0.7           # random-walk std (m / sqrt(s)) of the moving-track belief


def loc_sigma_range(quality: float) -> float:
    """Ranging sigma (m) the filter assumes for a scan of given quality (a bit conservative)."""
    return 0.25 + 0.4 * (1.0 - quality)


def predicted_range_field(grid: Grid, sensor_pose, depth_nom: float):
    """3D slant range from the sensor to every cell (at nominal depth) and |dz|/r."""
    X, Y = grid.mesh()
    sx, sy, sz = sensor_pose[:3]
    dz = sz - (grid.z - depth_nom)
    r = np.sqrt((X - sx) ** 2 + (Y - sy) ** 2 + dz ** 2)
    return r, np.abs(dz) / np.maximum(r, 1e-6)


def ring_likelihood(grid, sensor_pose, ranges, weights, quality, cfg: LocConfig, field=None):
    r, ratio = field if field is not None else predicted_range_field(grid, sensor_pose, cfg.depth_nom)
    sigma = np.sqrt(loc_sigma_range(quality) ** 2 + (cfg.sigma_depth * ratio) ** 2)
    g = np.zeros_like(r)
    for rk, wk in zip(ranges, weights):
        g += wk * np.exp(-0.5 * ((rk - r) / sigma) ** 2)
    return cfg.eps + np.minimum(g, 1.0)


def _smooth(p, n=2):
    for _ in range(n):
        q = np.pad(p, 1, mode="edge")
        p = 0.25 * q[1:-1, :-2] + 0.5 * q[1:-1, 1:-1] + 0.25 * q[1:-1, 2:]
        q = np.pad(p, 1, mode="edge")
        p = 0.25 * q[:-2, 1:-1] + 0.5 * q[1:-1, 1:-1] + 0.25 * q[2:, 1:-1]
    return p


def extract_modes(p, grid: Grid, cfg: LocConfig):
    """Modes of a normalised belief: [{xy, cov, p_mass, peak}] sorted by p_mass desc."""
    X, Y = grid.mesh()
    ps = _smooth(p, 2)
    ramp = np.linspace(0, 1, ps.size).reshape(ps.shape)
    cmp_ = ps * (1 + 1e-9 * ramp)
    pad = np.pad(cmp_, 2, mode="constant", constant_values=-1)
    is_max = cmp_ >= sliding_window_view(pad, (5, 5)).max(axis=(2, 3))
    thr = cfg.min_rel * ps.max()
    jj, ii = np.nonzero(is_max & (ps > thr))
    order = np.argsort(-ps[jj, ii])
    peaks = []
    for k in order:
        x, y = X[jj[k], ii[k]], Y[jj[k], ii[k]]
        if all(np.hypot(x - a, y - b) >= cfg.merge_dist for a, b, _ in peaks):
            peaks.append((x, y, ps[jj[k], ii[k]]))
        if len(peaks) >= 6:
            break
    if not peaks:
        return []
    px = np.array([q[0] for q in peaks])
    py = np.array([q[1] for q in peaks])
    d = np.hypot(X[None] - px[:, None, None], Y[None] - py[:, None, None])
    nearest = d.argmin(axis=0)
    dmin = d.min(axis=0)
    modes = []
    for m in range(len(peaks)):
        sel = (nearest == m) & (dmin <= cfg.mode_radius)
        w = p[sel]
        mass = float(w.sum())
        if mass <= 0:
            continue
        mx = float((X[sel] * w).sum() / mass)
        my = float((Y[sel] * w).sum() / mass)
        dx, dy = X[sel] - mx, Y[sel] - my
        c = np.array([[(w * dx * dx).sum(), (w * dx * dy).sum()],
                      [(w * dx * dy).sum(), (w * dy * dy).sum()]]) / mass
        c += np.eye(2) * grid.res ** 2 / 12
        modes.append({"xy": [mx, my], "cov": c, "p_mass": mass, "peak": float(peaks[m][2])})
    modes.sort(key=lambda m: -m["p_mass"])
    return modes


def _wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class Localiser:
    def __init__(self, grid: Grid, cfg: LocConfig | None = None):
        self.grid = grid
        self.cfg = cfg or LocConfig()
        self.obs = []              # stationary radar observations (as received)
        self.moving_obs = []
        self.acoustic = []
        self._prior = self._rubble_prior()
        self.logb = np.log(self._prior)
        self.pm = np.full(grid.z.shape, 1.0 / grid.z.size)  # moving-track belief
        self._pm_t = None
        self._p = None
        self._fields = {}

    # ---- prior from LiDAR geometry only
    def _rubble_prior(self):
        g = self.grid
        X, Y = g.mesh()
        w = np.clip(g.z - 1.0, 0, None)
        cx = (X * w).sum() / max(w.sum(), 1e-9)
        cy = (Y * w).sum() / max(w.sum(), 1e-9)
        ground = np.median(g.z[np.hypot(X - cx, Y - cy) > 20.0])
        mask = ((g.z - ground) > 0.4) & (np.hypot(X - cx, Y - cy) < 13.0)
        mask = _smooth(mask.astype(float), 2)
        mask = np.clip(mask / max(mask.max(), 1e-9), 0, 1)
        prior = self.cfg.prior_floor + (1 - self.cfg.prior_floor) * mask
        self.centre = (float(cx), float(cy))
        return prior / prior.sum()

    # ---- updates
    def _field(self, pose):
        key = tuple(np.round(pose[:3], 2))
        if key not in self._fields:
            self._fields[key] = predicted_range_field(self.grid, pose, self.cfg.depth_nom)
        return self._fields[key]

    def obs_likelihood(self, obs):
        w = [c * obs["quality"] for c in obs["confidence"]]
        return ring_likelihood(self.grid, obs["sensor_pose"], obs["range_candidates"], w, obs["quality"],
                               self.cfg, self._field(obs["sensor_pose"]))

    def update_radar(self, obs: dict) -> str:
        """Fold in a radar observation. Returns 'stationary', 'moving' or 'empty'."""
        if obs.get("doppler_moving"):
            self.moving_obs.append(obs)
            t = obs["timestamp"]
            dt = 5.0 if self._pm_t is None else max(t - self._pm_t, 0.0)
            self._pm_t = t
            sig = self.cfg.move_q * np.sqrt(dt) / self.grid.res
            self.pm = self._blur(self.pm, sig) * self.obs_likelihood(obs)
            self.pm /= self.pm.sum()
            return "moving"
        self.obs.append(obs)
        self._p = None
        if not obs["range_candidates"]:
            return "empty"
        self.logb = self.logb + np.log(self.obs_likelihood(obs))
        return "stationary"

    def update_acoustic(self, obs: dict):
        self.acoustic.append(obs)
        X, Y = self.grid.mesh()
        ox, oy = obs["origin"]
        phi = np.degrees(np.arctan2(Y - oy, X - ox))
        d = _wrap180(phi - obs["bearing_deg"])
        sigma = max(obs["width_deg"] / 3.0, 5.0)
        w = obs["confidence"] * self.cfg.ac_weight
        L = self.cfg.ac_eps + w * np.exp(-0.5 * (d / sigma) ** 2)
        L = np.where(np.hypot(X - ox, Y - oy) < 1.0, self.cfg.ac_eps + w, L)
        self.logb = self.logb + np.log(L)
        self._p = None

    @staticmethod
    def _blur(p, sigma_cells):
        if sigma_cells < 0.3:
            return p
        rad = int(min(max(3 * sigma_cells, 1), 60))
        k = np.exp(-0.5 * (np.arange(-rad, rad + 1) / sigma_cells) ** 2)
        k /= k.sum()
        out = p
        for axis in (0, 1):
            pad = [(0, 0), (0, 0)]
            pad[axis] = (rad, rad)
            q = np.pad(out, pad, mode="constant")
            acc = np.zeros_like(out)
            n = out.shape[axis]
            for s, ks in enumerate(k):
                acc += ks * (q[s:s + n, :] if axis == 0 else q[:, s:s + n])
            out = acc
        return out

    # ---- queries
    @property
    def n_informative(self):
        return sum(1 for o in self.obs if o["range_candidates"])

    def belief(self):
        if self._p is None:
            lb = self.logb - self.logb.max()
            p = np.exp(lb)
            self._p = p / p.sum()
        return self._p

    def entropy(self, p=None):
        p = self.belief() if p is None else p
        q = p[p > 0]
        return float(-(q * np.log(q)).sum())

    def region95(self):
        """Boolean mask of the smallest set of cells holding 95% of the belief."""
        p = self.belief().ravel()
        order = np.argsort(-p)
        cum = np.cumsum(p[order])
        k = int(np.searchsorted(cum, 0.95)) + 1
        mask = np.zeros(p.size, bool)
        mask[order[:k]] = True
        return mask.reshape(self.grid.z.shape)

    def extract(self):
        """Candidate regions + overall status."""
        cfg = self.cfg
        if self.n_informative == 0:
            return {"status": "insufficient_evidence", "modes": [], "n_significant": 0,
                    "reason": "no respiration-like returns yet", "stationary_tracks": []}
        modes = extract_modes(self.belief(), self.grid, cfg)
        modes = [m for m in modes if m["p_mass"] >= cfg.min_report_mass]
        sig = [m for m in modes if m["p_mass"] >= cfg.significant_min]
        reason = None
        if not modes:
            reason = "no localised mode"
        else:
            dom = modes[0]
            smax = float(np.sqrt(np.linalg.eigvalsh(dom["cov"]).max()))
            dom["sigma_major"] = smax
            if len(sig) > 1:
                sep = float(np.hypot(sig[0]["xy"][0] - sig[1]["xy"][0], sig[0]["xy"][1] - sig[1]["xy"][1]))
                reason = f"ambiguity: {len(sig)} significant modes, top two {sep:.1f} m apart"
            elif dom["p_mass"] < cfg.dominant_min:
                reason = f"low confidence: dominant region holds {dom['p_mass']:.0%} of belief"
            elif smax > cfg.max_sigma:
                reason = f"high uncertainty: dominant region sigma {smax:.1f} m"
        return {"status": "insufficient_evidence" if reason else "candidate", "modes": modes,
                "n_significant": len(sig), "reason": reason or "single dominant mode"}

    def moving_tracks(self):
        if not self.moving_obs:
            return []
        modes = extract_modes(self.pm, self.grid, self.cfg)
        return [m for m in modes[:2] if m["p_mass"] >= 0.2]

    def strong_support(self, xy, min_agree=0.5, min_strength=0.35):
        """Scan ids whose ranges pass through ``xy`` with confidence x quality >= ``min_strength``."""
        return [o["scan_id"] for o in self.obs if o["range_candidates"] and self.agreement(o, xy) > min_agree
                and max(o["confidence"]) * o["quality"] >= min_strength]

    def estimate_depth(self, xy, cov, max_sigma=0.45, min_scans=3):
        """Burial depth below the LiDAR surface at a mode, from the slant ranges of the scans that support it.

        For each supporting scan, the range that best matches the mode is compared with the predicted slant
        range sqrt(horizontal^2 + (sensor_z - (surface - d))^2) over a depth grid d in [0.2, 3.0] m (flat
        prior), marginalised over 5 sigma points of the mode's xy covariance (horizontal position error is
        the main confounder). Returns {depth_m, depth_sigma, depth_source, n_scans}: ``radar_estimate`` when
        at least ``min_scans`` scans support the mode and the posterior sigma <= ``max_sigma``, else the
        nominal depth (``cfg.depth_nom``) with ``depth_source: "nominal"`` and the posterior (or prior) sigma.
        """
        cfg = self.cfg
        d = np.arange(0.2, 3.0001, 0.05)
        sup = [o for o in self.obs if o["range_candidates"] and self.agreement(o, xy) > 0.5]
        out = {"depth_m": cfg.depth_nom, "depth_sigma": round(float(np.std(d)), 2), "depth_source": "nominal",
               "n_scans": len(sup)}
        if not sup:
            return out
        cov = np.asarray(cov, float)
        ev, V = np.linalg.eigh(cov)
        pts, wts = [np.asarray(xy, float)], [1 / 3]
        for k in (0, 1):
            for sgn in (-1, 1):
                pts.append(np.asarray(xy, float) + sgn * np.sqrt(max(ev[k], 0.0) * 3.0) * V[:, k])   # unscented, kappa = 1
                wts.append(1 / 6)
        post = np.zeros_like(d)
        for p, wt in zip(pts, wts):
            surf = float(self.grid.height_at(p[0], p[1]))
            ll = np.zeros_like(d)
            for o in sup:
                sx, sy, sz = o["sensor_pose"][:3]
                r_pred = np.sqrt((p[0] - sx) ** 2 + (p[1] - sy) ** 2 + (sz - (surf - d)) ** 2)
                sig = loc_sigma_range(o["quality"])
                g = np.zeros_like(d)
                for rk, ck in zip(o["range_candidates"], o["confidence"]):
                    g = np.maximum(g, ck * o["quality"] * np.exp(-0.5 * ((rk - r_pred) / sig) ** 2))
                ll += np.log(cfg.eps + g)
            post += wt * np.exp(ll - ll.max())
        post /= post.sum()
        mean = float((d * post).sum())
        sd = float(np.sqrt(((d - mean) ** 2 * post).sum()))
        out["depth_sigma"] = round(sd, 2)
        if len(sup) >= min_scans and sd <= max_sigma:
            out.update(depth_m=round(mean, 2), depth_source="radar_estimate")
        return out

    def agreement(self, obs, xy):
        """How well observation `obs` supports location xy: 0 (outlier) .. 1 (ring passes through)."""
        if not obs["range_candidates"]:
            return 0.0
        L = self.obs_likelihood(obs)
        ix, iy = self.grid.index(xy[0], xy[1])
        e = self.cfg.eps
        return float(np.clip((L[iy, ix] - e) / max(L.max() - e, 1e-9), 0, 1))

    def used_weights(self, result=None):
        """scan_id -> weight: agreement with the best-supported significant mode."""
        result = result or self.extract()
        locs = [m["xy"] for m in result["modes"] if m["p_mass"] >= self.cfg.significant_min] \
            or [m["xy"] for m in result["modes"][:1]]
        out = {}
        for o in self.obs:
            out[o["scan_id"]] = max((self.agreement(o, xy) for xy in locs), default=0.0)
        mt = self.moving_tracks()
        for o in self.moving_obs:
            out[o["scan_id"]] = self.agreement_moving(o, mt[0]["xy"]) if mt else 0.0
        return out

    def agreement_moving(self, obs, xy):
        L = self.obs_likelihood(obs)
        ix, iy = self.grid.index(xy[0], xy[1])
        return float(np.clip((L[iy, ix] - self.cfg.eps) / max(L.max() - self.cfg.eps, 1e-9), 0, 1))
