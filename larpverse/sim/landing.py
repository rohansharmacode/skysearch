"""Landing-patch detection from the heightmap only (no targets, no hard-coded sites).

For every cell: local plane fit over a ~1.5 m footprint (3x3 cells at 0.5 m)
-> slope, roughness (residual std); then for plausible cells:
  clearance : horizontal distance to the nearest "obstacle" cell (>0.3 m above the
              fitted plane) within 3 m, capped at 3 m (metres, rotor needs >= 0.9)
  step      : max |height difference| to the plane inside the rotor radius (0.9 m)
  approach  : no terrain poking through a 30-degree descent cone above the patch
Candidates are picked greedily by (score, proximity to the mound) with a minimum spacing.
"""
from __future__ import annotations

import numpy as np

from .world import Grid

LIM_SLOPE = 12.0       # deg, flat ground
PERCH_SLOPE = 20.0     # deg, perches on the rubble (adaptive legs + self-levelling radar mount)
LIM_ROUGH = 0.07       # m
LIM_CLEAR = 0.9        # m (rotor radius + margin)
LIM_STEP = 0.35        # m
OBSTACLE_H = 0.30      # m above fitted plane
CONE_HALF_DEG = 15.0   # final descent is vertical from ~3 m above the patch (perching on the pile)
MIN_SPACING = 4.0
SEARCH_RADIUS = 14.0
MAX_SITES = 16
MAX_REJECTED = 6


def mound_centre(grid: Grid):
    """Rubble centre estimate from the heightmap (height-weighted centroid of tall cells)."""
    X, Y = grid.mesh()
    w = np.clip(grid.z - 1.0, 0, None)
    if w.sum() < 1e-6:
        return 0.0, 0.0
    return float((X * w).sum() / w.sum()), float((Y * w).sum() / w.sum())


def patch_fields(grid: Grid):
    """Vectorised 3x3 plane fit: returns slope_deg, roughness, plane gradients (a, b)."""
    z = grid.z.astype(np.float64)
    h, w = z.shape
    res = grid.res
    zp = np.pad(z, 1, mode="edge")
    a = np.zeros_like(z)
    b = np.zeros_like(z)
    mean = np.zeros_like(z)
    shifts = {}
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            s = zp[1 + dy:1 + dy + h, 1 + dx:1 + dx + w]
            shifts[(dy, dx)] = s
            a += dx * res * s
            b += dy * res * s
            mean += s
    a /= 6 * res ** 2
    b /= 6 * res ** 2
    mean /= 9
    ss = np.zeros_like(z)
    for (dy, dx), s in shifts.items():
        ss += (s - (mean + a * dx * res + b * dy * res)) ** 2
    rough = np.sqrt(ss / 9)
    slope = np.degrees(np.arctan(np.hypot(a, b)))
    return slope, rough, a, b, mean


def _clearance_step_approach(zpad, i, j, pad, a, b, zc, res):
    """Window analyses for one patch centred at cell (j, i) (indices in unpadded grid)."""
    # clearance / step window: +-6 cells (3 m)
    R = 6
    win = zpad[j + pad - R:j + pad + R + 1, i + pad - R:i + pad + R + 1]
    offs = (np.arange(-R, R + 1)) * res
    DX, DY = np.meshgrid(offs, offs)
    D = np.hypot(DX, DY)
    rel = win - (zc + a * DX + b * DY)
    obstacle = rel > OBSTACLE_H
    clearance = float(D[obstacle].min()) if obstacle.any() else 3.0
    clearance = min(clearance, 3.0)
    inner = D <= 0.9
    step = float(np.abs(rel[inner]).max())
    # approach cone: window +-12 cells (6 m)
    R2 = 12
    win2 = zpad[j + pad - R2:j + pad + R2 + 1, i + pad - R2:i + pad + R2 + 1]
    offs2 = (np.arange(-R2, R2 + 1)) * res
    DX2, DY2 = np.meshgrid(offs2, offs2)
    D2 = np.hypot(DX2, DY2)
    limit = np.maximum(D2 - 1.0, 0.0) / np.tan(np.radians(CONE_HALF_DEG))
    blocked = (win2 - zc > limit) & (D2 > 1.0)
    approach_ok = not bool(blocked.any())
    return clearance, step, approach_ok


def detect_landing_sites(grid: Grid, centre=None, max_sites: int = MAX_SITES,
                         min_spacing: float = MIN_SPACING, search_radius: float = SEARCH_RADIUS):
    """Return landing-site dicts (INTERFACES.md fields) with status candidate/rejected."""
    cx, cy = centre if centre is not None else mound_centre(grid)
    slope, rough, a, b, mean = patch_fields(grid)
    X, Y = grid.mesh()
    rc = np.hypot(X - cx, Y - cy)
    pad = 14
    zpad = np.pad(grid.z.astype(np.float64), pad, mode="edge")

    rows = []
    ground_ref = float(np.median(grid.z[rc > 20.0])) if (rc > 20.0).any() else 0.0
    perch = mean > ground_ref + 0.6               # on the rubble: the radar needs to be close (penetration limit)
    lim_slope = np.where(perch, PERCH_SLOPE, LIM_SLOPE)
    sel = (rc <= search_radius) & (slope <= lim_slope + 6) & (rough <= LIM_ROUGH + 0.05)
    for j, i in zip(*np.nonzero(sel)):
        clearance, step, app = _clearance_step_approach(zpad, i, j, pad, a[j, i], b[j, i], mean[j, i], grid.res)
        s_slope = max(0.0, 1 - slope[j, i] / 10.0)
        s_rough = max(0.0, 1 - rough[j, i] / 0.06)
        s_clear = min(1.0, clearance / 3.0)
        score = 0.35 * s_slope + 0.35 * s_rough + 0.20 * s_clear + 0.10 * (1.0 if app else 0.0)
        reason = None
        if slope[j, i] > lim_slope[j, i]:
            reason = "slope"
        elif rough[j, i] > LIM_ROUGH:
            reason = "roughness"
        elif step > LIM_STEP:
            reason = "step"
        elif clearance < LIM_CLEAR:
            reason = "clearance"
        elif not app:
            reason = "approach"
        rows.append(dict(i=int(i), j=int(j), x=float(X[j, i]), y=float(Y[j, i]), z=float(mean[j, i]),
                         slope=float(slope[j, i]), rough=float(rough[j, i]), clearance=clearance,
                         score=float(score), reason=reason, r=float(rc[j, i]), perch=bool(perch[j, i])))

    def rank(row):  # prefer good patches, mildly prefer the ones near the rubble
        return row["score"] - 0.012 * row["r"]

    ok = sorted([r for r in rows if r["reason"] is None], key=rank, reverse=True)
    bad = sorted([r for r in rows if r["reason"] is not None and r["r"] < 12.0],
                 key=lambda r: r["score"] - 0.012 * r["r"], reverse=True)

    # Reserve slots for patches ON the rubble (closest to the buried casualty); fill the
    # rest with the best flat ground around it.  Both greedy with a minimum spacing.
    on_rubble = [r for r in ok if r["z"] > ground_ref + 0.6]
    chosen = []

    def take(rows, limit, spacing):
        for row in rows:
            if len(chosen) >= limit:
                break
            if all(np.hypot(row["x"] - c["x"], row["y"] - c["y"]) >= spacing for c in chosen):
                chosen.append(row)

    take(on_rubble, max_sites // 2, 2.5)
    take(ok, max_sites, min_spacing)
    rejected = []
    for row in bad:
        if len(rejected) >= MAX_REJECTED:
            break
        if all(np.hypot(row["x"] - c["x"], row["y"] - c["y"]) >= 3.0 for c in chosen + rejected):
            rejected.append(row)

    sites = []
    for k, row in enumerate(chosen + rejected, 1):
        sites.append({
            "id": f"L{k}", "xy": [round(row["x"], 2), round(row["y"], 2)], "z": round(row["z"], 3),
            "slope_deg": round(row["slope"], 2), "roughness": round(row["rough"], 4),
            "clearance": round(row["clearance"], 2), "score": round(row["score"], 3),
            "status": "candidate" if row["reason"] is None else "rejected", "perch": row["perch"],
            **({"reason": row["reason"]} if row["reason"] else {}),
        })
    return sites
