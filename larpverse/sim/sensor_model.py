"""Nominal (assumed) radar sensor model shared by the generator, localiser and planner.

SYNTHETIC.  These are the *assumed* characteristics of the drone radar; the
true generator (radar_model.py) adds jitter, false returns and rubble
attenuation computed from the actual geometry.  Nothing here touches targets.
"""
from __future__ import annotations

import numpy as np

SCAN_DWELL_S = 20.0


def scan_quality(roughness: float, slope_deg: float, dwell_s: float = SCAN_DWELL_S) -> float:
    """Expected scan-level quality in [0,1]: drops with rough/sloped landing and short dwell."""
    q_r = np.exp(-(roughness / 0.045) ** 2)
    q_s = np.exp(-(slope_deg / 9.0) ** 2)
    q_d = 1.0 - np.exp(-dwell_s / 8.0)
    return float(np.clip(q_r * q_s * q_d, 0.02, 0.97))


def detection_prob(range_m, quality, rubble_path_m: float = 2.0):
    """Probability that the respiration return is detected at all."""
    p = np.sqrt(np.clip(quality, 0, 1)) * np.exp(-(np.asarray(range_m) / 25.0) ** 2) \
        * np.exp(-rubble_path_m / 18.0)
    return np.clip(p, 0.0, 1.0)


def range_sigma(quality, rubble_path_m: float = 2.0):
    """1-sigma range error (m): ~0.15-0.3 m for good scans, worse with low quality / rubble."""
    return 0.15 + 0.25 * (1.0 - np.asarray(quality)) + 0.03 * rubble_path_m


def rubble_path(grid, p_sensor, p_targets, n: int = 40):
    """Length (m) of each sensor->target segment that lies below the heightmap surface.

    p_sensor: (3,), p_targets: (K,3) or (3,).  Used by the generator with the true target and by
    the planner with hypothesised targets (surface - nominal depth).
    """
    ps = np.asarray(p_sensor, float)
    pt = np.atleast_2d(np.asarray(p_targets, float))
    ts = np.linspace(0.0, 1.0, n)[None, :, None]
    P = ps[None, None, :] + ts * (pt[:, None, :] - ps[None, None, :])
    below = P[..., 2] < grid.height_at(P[..., 0], P[..., 1]) - 0.02
    out = below.mean(axis=1) * np.linalg.norm(pt - ps[None, :], axis=1)
    return out if np.ndim(p_targets) > 1 else float(out[0])
