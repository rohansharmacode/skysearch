"""Next-scan selection.

Never sees ground truth: only the heightmap, the landing sites, the localiser
belief and drone state.

* initial_sites(): choose the first 3 sites maximising expected Fisher-information
  geometry (E over prior samples of log det of the summed range-information matrices),
  which rewards angular spread around the rubble, short ranges and good landing quality.
* choose_next(): adaptive.  Expected information gain (EIG) of a scan from each unused
  site, by Monte Carlo over hypotheses drawn from the current belief:
      EIG = E_x~belief [ p_detect(x) * ( H(belief) - H(belief | simulated range y(x)) ) ]
  minus a travel-time cost; picks the best (drone, site) pair.
* fixed_next(): baseline policy that ignores the belief (angular-sweep site order).
"""
from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations, permutations

import numpy as np

from .localise import LocConfig, loc_sigma_range, predicted_range_field
from .radar_dsp import LEVEL_DEG, PERCH_ROUGH_FACTOR, POD_MAX_SLOPE, POD_ROUGH_FACTOR, reach_prob
from .sensor_model import SCAN_DWELL_S, detection_prob, range_sigma, rubble_path, scan_quality

LANDED_SENSOR_HEIGHT = 0.25
POD_SENSOR_HEIGHT = 0.05       # winch pod: antenna face resting on the rubble surface (contact, no air gap)


@dataclass
class Choice:
    drone_id: str
    site_id: str
    score: float       # net gain normalised by ln 2, in [0, 1]
    eig: float         # nats
    travel_s: float
    reason: str


def sensor_pose_at(site):
    return [site["xy"][0], site["xy"][1], site["z"] + (POD_SENSOR_HEIGHT if site.get("pod") else LANDED_SENSOR_HEIGHT), 0.0]


def pdet(range_m, quality, path_m, dwell=SCAN_DWELL_S):
    """Expected detection probability of a breathing return: range / quality term times the through-rubble
    penetration limit (radar_dsp.reach_prob). Uses hypothesised targets only, never truth."""
    return detection_prob(range_m, quality, 0.0) * reach_prob(range_m, path_m, quality, dwell=dwell)


def site_quality(site, dwell=SCAN_DWELL_S):
    if site.get("pod"):                # pod seated on the surface: tilt within its spec does not matter
        return scan_quality(site["roughness"] * POD_ROUGH_FACTOR, max(0.0, site["slope_deg"] - POD_MAX_SLOPE),
                            max(dwell, 45.0))                       # pod scans dwell 45 s (coordinator.POD_DWELL_S)
    rough = site["roughness"] * (PERCH_ROUGH_FACTOR if site.get("perch") else 1.0)
    return scan_quality(rough, max(0.0, site["slope_deg"] - LEVEL_DEG), dwell)   # levelled mount


class Planner:
    def __init__(self, grid, loc_cfg: LocConfig | None = None, seed: int = 0, k_samples: int = 96,
                 travel_cost: float = 0.004):
        self.grid = grid
        self.cfg = loc_cfg or LocConfig()
        self.seed = seed
        self.K = k_samples
        self.lam = travel_cost  # nats per second of flight
        X, Y = grid.mesh()
        self._XY = (X.ravel(), Y.ravel())

    # ------------------------------------------------------------ initial sites
    def initial_sites(self, loc, sites, drones, travel):
        """Return ([(drone_id, site_id)], explanation)."""
        cand = [s for s in sites if s["status"] == "candidate"]
        rng = np.random.default_rng(self.seed + 11)
        p = loc.belief().ravel()
        idx = rng.choice(p.size, size=48, p=p)
        X, Y = self.grid.mesh()
        px, py = X.ravel()[idx], Y.ravel()[idx]
        ground_z = self.grid.z.ravel()[idx] - self.cfg.depth_nom
        info = []
        for s in cand:
            pose = sensor_pose_at(s)
            dx, dy = px - pose[0], py - pose[1]
            dh = np.hypot(dx, dy) + 1e-6
            r = np.sqrt(dh ** 2 + (pose[2] - ground_z) ** 2)
            q = site_quality(s)
            tgt = np.stack([px, py, ground_z], 1)
            path = rubble_path(self.grid, pose[:3], tgt)
            w = pdet(r, q, path, 45.0 if s.get("pod") else SCAN_DWELL_S) / range_sigma(q, path) ** 2
            u = np.stack([dx / dh, dy / dh], 1)
            info.append(w[:, None, None] * u[:, :, None] * u[:, None, :])  # (K,2,2)
        k = min(3, len(cand), len(drones))
        if k == 0:                     # nothing safe to land or lower a pod on: the caller marks it unreachable
            return [], "no safe landing site or pod placement"
        best, best_val = None, -np.inf
        for combo in combinations(range(len(cand)), k):
            J = sum(info[c] for c in combo) + 0.05 * np.eye(2)
            val = float(np.mean(np.log(np.linalg.det(J))))
            if val > best_val:
                best, best_val = combo, val
        chosen = [cand[i] for i in best]
        # assign drones to sites minimising total travel time
        ids = [d["id"] for d in drones]
        best_perm, best_cost = None, np.inf
        for perm in permutations(range(len(chosen)), min(len(ids), len(chosen))):
            cost = 0.0
            for di, si in zip(range(len(ids)), perm):
                t = travel(ids[di], chosen[si])
                cost += t if t is not None else 1e6
            if cost < best_cost:
                best_perm, best_cost = perm, cost
        assign = [(ids[di], chosen[si]["id"]) for di, si in zip(range(len(ids)), best_perm)]
        qs = ", ".join(f"{s['id']} q~{site_quality(s):.2f}" for s in chosen)
        return assign, f"best trilateration geometry (E[log det FIM]={best_val:.2f}): {qs}"

    # ------------------------------------------------------------------- adaptive
    def _eig(self, loc, p_sup, sup, site, rng_normals, rng_pick):
        """Monte-Carlo expected information gain (nats) of scanning from `site`."""
        q = site_quality(site)
        pose = sensor_pose_at(site)
        r_full, ratio_full = predicted_range_field(self.grid, pose, self.cfg.depth_nom)
        r_c = r_full.ravel()[sup]
        sig_c = np.sqrt(loc_sigma_range(q) ** 2 + (self.cfg.sigma_depth * ratio_full.ravel()[sup]) ** 2)
        r_true = r_c[rng_pick]                                    # (K,)
        Xs, Ys = self._XY
        hyp = np.stack([Xs[sup][rng_pick], Ys[sup][rng_pick], self.grid.z.ravel()[sup][rng_pick] - self.cfg.depth_nom], 1)
        path = rubble_path(self.grid, pose[:3], hyp)
        p_det = pdet(r_true, q, path, 45.0 if site.get("pod") else SCAN_DWELL_S)
        y = r_true + rng_normals * range_sigma(q, path)
        w = 0.75 * q
        L = self.cfg.eps + np.minimum(w * np.exp(-0.5 * ((y[:, None] - r_c[None, :]) / sig_c[None, :]) ** 2), 1.0)
        post = p_sup[None, :] * L
        post /= post.sum(axis=1, keepdims=True)
        H = -(post * np.log(np.maximum(post, 1e-300))).sum(axis=1)
        H0 = -(p_sup * np.log(np.maximum(p_sup, 1e-300))).sum()
        return float(np.mean(p_det * (H0 - H))), q

    def choose_next(self, loc, result, sites, drones, travel, used_scans: int = 0):
        """drones: idle drone dicts [{'id', ...}]; travel(drone_id, site)->seconds or None."""
        cand = [s for s in sites if s["status"] == "candidate"]
        if not cand or not drones:
            return None
        p = loc.belief().ravel()
        sup = np.nonzero(p > 1e-4 * p.max())[0]
        p_sup = p[sup] / p[sup].sum()
        rng = np.random.default_rng(self.seed * 1000 + used_scans)
        pick = rng.choice(len(sup), size=self.K, p=p_sup)
        normals = rng.normal(0, 1, self.K)
        best, best_net, table = None, -np.inf, []
        for s in cand:
            eig, q = self._eig(loc, p_sup, sup, s, normals, pick)
            for d in drones:
                t = travel(d["id"], s)
                if t is None:
                    continue
                net = eig - self.lam * t
                table.append((net, eig, t, d["id"], s))
                if net > best_net:
                    best, best_net = (eig, t, d["id"], s), net
        if best is None:
            return None
        eig, t, did, s = best
        return Choice(did, s["id"], float(np.clip(best_net / np.log(2), 0, 1)), eig, t,
                      self._explain(loc, result, s, eig, t))

    def _explain(self, loc, result, site, eig, t):
        modes = result["modes"]
        pose = sensor_pose_at(site)
        tag = f"EIG {eig:.2f} nats, {t:.0f} s flight"
        if result["n_significant"] >= 2:
            a, b = modes[0], modes[1]
            ra = np.sqrt((a["xy"][0] - pose[0]) ** 2 + (a["xy"][1] - pose[1]) ** 2 + (pose[2] - (self.grid.height_at(*a["xy"]) - self.cfg.depth_nom)) ** 2)
            rb = np.sqrt((b["xy"][0] - pose[0]) ** 2 + (b["xy"][1] - pose[1]) ** 2 + (pose[2] - (self.grid.height_at(*b["xy"]) - self.cfg.depth_nom)) ** 2)
            sep = float(np.hypot(a["xy"][0] - b["xy"][0], a["xy"][1] - b["xy"][1]))
            return f"separates two modes {sep:.1f} m apart: predicted ranges {ra:.1f} vs {rb:.1f} m ({tag})"
        if modes:
            m = modes[0]
            used = [o["sensor_pose"] for o in loc.obs if o["range_candidates"]]
            b_new = np.degrees(np.arctan2(m["xy"][1] - pose[1], m["xy"][0] - pose[0]))
            diffs = [abs((np.degrees(np.arctan2(m["xy"][1] - u[1], m["xy"][0] - u[0])) - b_new + 180) % 360 - 180)
                     for u in used] or [180.0]
            return f"tightens the dominant region (sigma {np.sqrt(np.linalg.eigvalsh(m['cov']).max()):.1f} m) with a look {min(diffs):.0f} deg from earlier scans ({tag})"
        return f"no localisation yet: gather a return from a new vantage ({tag})"

    # ------------------------------------------------------------------- baseline
    def fixed_next(self, loc, sites, drones, travel, centre):
        """Belief-blind baseline: next site = most angularly separated from the sites already used."""
        cand = [s for s in sites if s["status"] == "candidate"]
        used = [s for s in sites if s["status"] == "used"]
        if not cand or not drones:
            return None

        def ang(s):
            return np.degrees(np.arctan2(s["xy"][1] - centre[1], s["xy"][0] - centre[0]))

        def sep(s):
            return min((abs((ang(s) - ang(u) + 180) % 360 - 180) for u in used), default=180.0)

        best = max(cand, key=lambda s: (round(sep(s) / 15.0), s["score"]))
        opts = [(travel(d["id"], best), d["id"]) for d in drones]
        opts = [o for o in opts if o[0] is not None]
        if not opts:
            return None
        t, did = min(opts)
        return Choice(did, best["id"], 0.0, 0.0, t,
                      f"fixed sweep: next site {sep(best):.0f} deg from nearest used site (belief ignored)")
