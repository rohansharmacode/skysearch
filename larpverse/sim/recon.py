"""Reconnaissance mode (contracts/INTERFACES.md, "Reconnaissance mode (v1.2)"): no zones tasked yet.

Instead of idling, the fleet maps the site and proposes search zones:

1. Mapping flight: a boustrophedon coverage path over ``site.bounds`` (or an operator area of interest),
   split into one strip per drone, flown at the max mapped height + ``alt_margin_m``. Post-collapse
   satellite imagery is stale, so the map is built from scratch: the simulated LiDAR reveals the site
   heightmap within ``lidar_radius_m`` of each airborne drone (the same heightmap the mission's
   ``sample_pointcloud`` LiDAR stand-in draws from; an Isaac/raycast LiDAR would call ``ingest_points``).
   The flight altitude is re-planned every step from the growing map (newly mapped obstacles raise it).
2. Perch-and-listen: every ``perch_every_m`` of path the drones land together on flat cells of the fresh
   map (sim/landing.py cell scoring: slope, roughness, step), near unmapped or high rubble and spread out,
   and listen with rotors off for ``listen_s``. In-flight audio is not used (rotor ego-noise).
3. Proposals: the fresh heightmap is segmented into rubble mounds (connected cells well above street level,
   large components split at their cores, small fragments merged into a neighbour or dropped), outlined by
   a buffered convex hull and scored by height, area and acoustic leads -> ``zone_proposals``.

    rc = Recon(world.grid, flight, bounds=SITE_BOUNDS, pile_xy=world.pile_xy)
    while rc.status in ("mapping", "listening"):
        events = flight.step(dt, t); rc.step(t, events)
        if rc.wants_listen: rc.add_listen(clips, t)        # mission runs the acoustic pipeline
    state.update(rc.to_state())                            # {"recon": {...}, "zone_proposals": [...]}

Ground-truth boundary: recon sees the heightmap (sensor stand-in), flight poses and acoustic clips only.
"""
from __future__ import annotations

from collections import Counter

import numpy as np

from .landing import LIM_ROUGH, LIM_SLOPE, LIM_STEP, patch_fields
from .tasking import (Tasking, Zone, as_poly, buffer_convex, ccw, clip_human, clip_position, convex_hull,
                      points_in_polygon, polygon_area, rasterise, rasterise_box)
from .world import Grid

LIDAR_RADIUS_M = 12.0          # ground swath radius of the down-looking LiDAR at recon altitude (small sites)
LIDAR_RADIUS_MAX_M = 25.0      # large sites (320 m collapse site): higher legs, wider swath, fewer lanes
LANE_OVERLAP = 0.8             # lane spacing = 2 * radius * overlap
ALT_MARGIN_M = 13.0            # flight altitude above the highest mapped point (contract: about 12-15 m)
PERCH_EVERY_M = 90.0           # path length between perch-and-listen rounds (per drone)
RECON_LISTEN_S = 18.0          # rotors-off listening window at a perch (contract: about 15-20 s)
PERCH_ROUNDS = 4               # large sites: about this many perch rounds per drone path (perch_every auto)
RECON_RESERVE = 0.3            # a drone below this battery fraction flies back to its pad for a swap
RECON_SWAP_S = 60.0
PERCH_SPACING_M = 15.0         # perches of one round at least this far apart
STREET_ABOVE_M = 1.0           # mound cells rise at least this far above street level
MIN_MOUND_M2 = 20.0
MIN_MOUND_H = 1.5
SPLIT_M2 = 600.0               # components larger than this are split at their cores
MERGE_GAP_M = 3.0              # fragments (< 60 m2) within this gap join their neighbour
MAX_PROPOSAL_M2 = 1200.0       # still larger after core splitting -> k-means parts (collapse-site heaps)
GROUND_WINDOW_M = 60.0         # local ground: grey opening window (wider than the widest wreck footprint)
STANDING_MIN_H = 8.0           # standing-building test: roof at least this high above ground
ROOF_MIN_H = 3.0               # roof cue: planar patch at least this high ...
ROOF_LEVEL_DEG = 0.5           # ... and level (standing roof), or tilted as one rigid plane (leaning building):
ROOF_RECT_MIN = 0.55            # roof fill of its min-area rectangle (damaged / stepped roofs are not perfect)
ROOF_TILT_DEG = (4.0, 16.0)    # pancaked slabs sit at 0.6-2.5 deg and loose rubble is rough -> neither
ROOF_MIN_M2 = 15.0             # ... and this large, rectangular, with facades dropping around it
OUTLINE_MARGIN_M = 1.0


# ============================================================================ coverage path
def coverage_path(bounds, n_drones=3, lidar_radius_m=LIDAR_RADIUS_M, overlap=LANE_OVERLAP, aoi=None):
    """Boustrophedon lanes (north-south) over ``bounds`` (or the bbox of ``aoi`` clipped to it), split into
    ``n_drones`` east-west strips. Returns [[[x, y], ...] per drone]."""
    x0, y0, x1, y1 = map(float, bounds)
    poly = as_poly(aoi) if aoi is not None else None
    if poly is not None:
        x0, y0 = max(x0, poly[:, 0].min()), max(y0, poly[:, 1].min())
        x1, y1 = min(x1, poly[:, 0].max()), min(y1, poly[:, 1].max())
    lane = 2.0 * lidar_radius_m * overlap
    inset = min(0.5 * lidar_radius_m, 0.25 * (y1 - y0))
    paths = []
    edges = np.linspace(x0, x1, n_drones + 1)
    for k in range(n_drones):
        a, b = edges[k], edges[k + 1]
        n = max(1, int(np.ceil((b - a) / lane)))
        pts = []
        for i in range(n):
            x = a + (i + 0.5) * (b - a) / n
            ya, yb = y0 + inset, y1 - inset
            if poly is not None:                       # clip the lane to the area of interest
                ys = np.linspace(y0, y1, 200)
                inside = ys[points_in_polygon(np.full_like(ys, x), ys, poly)]
                if not len(inside):
                    continue
                ya, yb = inside.min() + min(inset, 0.25 * np.ptp(inside)), inside.max() - min(inset, 0.25 * np.ptp(inside))
            seg = [[x, ya], [x, yb]] if (i + k) % 2 == 0 else [[x, yb], [x, ya]]
            pts += seg
        paths.append([[round(float(px), 2), round(float(py), 2)] for px, py in pts])
    return paths


def path_length(path):
    p = np.asarray(path, float)
    return float(np.hypot(*np.diff(p[:, :2], axis=0).T).sum()) if len(p) > 1 else 0.0


def swept_mask(paths, radius_m, origin, res, w, h, step_m=1.0):
    """Cells within ``radius_m`` of any path (sampled every ``step_m``)."""
    xs = origin[0] + (np.arange(w) + 0.5) * res
    ys = origin[1] + (np.arange(h) + 0.5) * res
    X, Y = np.meshgrid(xs, ys)
    out = np.zeros((h, w), bool)
    for path in paths:
        p = np.asarray(path, float)
        for a, b in zip(p[:-1], p[1:]):
            n = max(2, int(np.hypot(*(b - a)[:2]) / step_m) + 1)
            for t in np.linspace(0, 1, n):
                q = a + t * (b - a)
                out |= (X - q[0]) ** 2 + (Y - q[1]) ** 2 <= radius_m ** 2
    return out


def path_coverage(paths, bounds, radius_m=LIDAR_RADIUS_M, res=1.0):
    """Fraction of ``bounds`` within LiDAR swath of the planned paths."""
    x0, y0, x1, y1 = bounds
    w, h = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
    return float(swept_mask(paths, radius_m, (x0, y0), res, w, h).mean())


# ============================================================================ map analysis
def flat_cells(z, seen, res):
    """Bool mask of landable cells in a partly mapped heightmap (landing.py slope/roughness limits plus a
    1 m step check against the neighbourhood; unmapped cells and their rim are excluded)."""
    from scipy.ndimage import binary_erosion, maximum_filter, minimum_filter
    zz = np.where(seen, z, np.nan)
    g = Grid((0.0, 0.0), res, z.shape[1], z.shape[0], np.nan_to_num(zz, nan=1e3))
    slope, rough, _, _, mean = patch_fields(g)
    k = max(3, int(round(2.0 / res)) | 1)
    step = maximum_filter(g.z, size=k) - minimum_filter(g.z, size=k)
    ok = (slope <= LIM_SLOPE) & (rough <= LIM_ROUGH) & (step <= LIM_STEP + 0.15)
    return ok & binary_erosion(seen, iterations=k)


def street_level(z, seen, area=None):
    sel = seen if area is None else (seen & area)
    return float(np.percentile(z[sel], 40)) if sel.any() else 0.0


def ground_level(z, seen, res, area=None, window_m=GROUND_WINDOW_M):
    """Local street level: grey opening of the mapped surface (2 m block minima, ``window_m`` window), so
    sloping streets and a site full of buildings still get a sensible reference. Unmapped -> street level."""
    from scipy import ndimage as ndi
    street = street_level(z, seen, area)
    f = max(1, int(round(2.0 / res)))
    H, W = z.shape
    h2, w2 = -(-H // f), -(-W // f)
    pad = np.full((h2 * f, w2 * f), np.inf)
    pad[:H, :W] = np.where(seen, z, np.inf)
    low = pad.reshape(h2, f, w2, f).min(axis=(1, 3))
    low = np.where(np.isfinite(low), low, street)
    k = max(3, int(round(window_m / (f * res))))
    g = ndi.maximum_filter(ndi.minimum_filter(low, size=k), size=k)
    return np.repeat(np.repeat(g, f, 0), f, 1)[:H, :W]


def min_rect_area(pts, pad=0.0):
    """Area of the minimum-area enclosing rectangle of 2D points (rotating calipers over hull edges);
    ``pad`` is added to each side length (cell size, when the points are cell centres)."""
    hull = convex_hull(pts)
    if len(hull) < 3:
        return 0.0
    best = np.inf
    for a, b in zip(hull, np.roll(hull, -1, axis=0)):
        e = b - a
        n = np.hypot(*e)
        if n < 1e-9:
            continue
        u = e / n
        v = np.array([-u[1], u[0]])
        pu, pv = hull @ u, hull @ v
        best = min(best, (np.ptp(pu) + pad) * (np.ptp(pv) + pad))
    return float(best)


def kmeans_split(P, k, iters=20):
    """Deterministic k-means (farthest-point init) labels for points P (N, 2)."""
    C = [P[np.argmin(P[:, 0] + P[:, 1])]]
    for _ in range(1, k):
        d = np.min([np.hypot(*(P - c).T) for c in C], 0)
        C.append(P[int(np.argmax(d))])
    C = np.array(C, float)
    for _ in range(iters):
        lab = np.argmin(((P[:, None, :] - C[None]) ** 2).sum(-1), 1)
        C = np.array([P[lab == j].mean(0) if (lab == j).any() else C[j] for j in range(k)])
    return lab


def standing_core(hc, m, res, min_h=STANDING_MIN_H):
    """Standing-building test on one component (heights above ground ``hc``, mask ``m``): a tall, roof-flat,
    rectangular top whose surroundings drop steeply (walls). Returns the roof mask or None (rubble)."""
    from scipy import ndimage as ndi
    hmax = float(hc[m].max())
    if hmax < min_h:
        return None
    core = m & (hc >= 0.85 * hmax)
    lab, n = ndi.label(core)
    if n == 0:
        return None
    sizes = ndi.sum(np.ones_like(lab), lab, range(1, n + 1))
    core = lab == (int(np.argmax(sizes)) + 1)
    a = float(core.sum()) * res * res
    if a < 40.0:
        return None
    gz = Grid((0.0, 0.0), res, hc.shape[1], hc.shape[0], hc.astype(np.float32))
    slope, rough, *_ = patch_fields(gz)
    inner = ndi.binary_erosion(core, iterations=2)
    sel = inner if inner.sum() >= 10 else core
    flat = np.median(slope[sel]) < 8.0 and np.median(rough[sel]) < 0.12
    jj, ii = np.nonzero(core)
    rect = a / max(min_rect_area(np.stack([ii * res, jj * res], 1), pad=res), 1e-9)
    ring = ndi.binary_dilation(core, iterations=max(1, int(round(2.0 / res)))) & ~core
    steep = ring.any() and float(np.median(hc[ring])) < 0.5 * hmax
    return core if (flat and rect >= 0.7 and steep) else None


def roof_buildings(hgt, seen, res, min_h=ROOF_MIN_H, min_m2=ROOF_MIN_M2):
    """Standing / tilted buildings from LiDAR geometry alone (no prior footprints): planar roof patches at
    least ``min_h`` above the ground that are either LEVEL (median slope < ROOF_LEVEL_DEG: an intact roof) or
    one rigid TILTED plane (ROOF_TILT_DEG, residual < 0.15 m: a leaning block), rectangular (>= 0.65 of their
    min-area rectangle) and ringed by facades (the ground 1.5 m outside the roof edge sits at most half the
    roof height). Pancaked lots are flat but not level (slabs settle at 0.6-2.5 deg on the collapse site) and
    rubble is rough, so both stay search zones. Returns [(j0, i0, roof mask)] in cropped coordinates."""
    from scipy import ndimage as ndi
    g = Grid((0.0, 0.0), res, hgt.shape[1], hgt.shape[0], hgt.astype(np.float32))
    slope, rough, *_ = patch_fields(g)
    roof = seen & (hgt >= min_h) & (rough < 0.06) & (slope < 35.0)
    roof = ndi.binary_opening(roof, iterations=1)
    lab, n = ndi.label(roof)
    ring_px = max(1, int(round(1.5 / res)))
    out = []
    for k, sl in enumerate(ndi.find_objects(lab), 1):
        if sl is None:
            continue
        j0, j1 = max(0, sl[0].start - ring_px - 1), min(hgt.shape[0], sl[0].stop + ring_px + 1)
        i0, i1 = max(0, sl[1].start - ring_px - 1), min(hgt.shape[1], sl[1].stop + ring_px + 1)
        m = lab[j0:j1, i0:i1] == k
        a = float(m.sum()) * res * res
        if a < min_m2:
            continue
        hc = hgt[j0:j1, i0:i1]
        jj, ii = np.nonzero(m)
        A = np.stack([ii * res, jj * res, np.ones(len(ii))], 1)
        coef, *_ = np.linalg.lstsq(A, hc[jj, ii], rcond=None)
        resid = float(np.std(hc[jj, ii] - A @ coef))
        tilt = float(np.median(slope[j0:j1, i0:i1][m]))
        level = tilt < ROOF_LEVEL_DEG and resid < 0.3
        leaning = ROOF_TILT_DEG[0] <= tilt <= ROOF_TILT_DEG[1] and resid < 0.15
        if not (level or leaning):
            continue
        if a / max(min_rect_area(np.stack([ii * res, jj * res], 1), pad=res), 1e-9) < ROOF_RECT_MIN:
            continue
        ring = ndi.binary_dilation(m, iterations=ring_px) & ~ndi.binary_dilation(m, iterations=max(1, ring_px - 1))
        if not ring.any() or float(np.median(hc[ring])) > 0.5 * float(np.median(hc[m])):
            continue
        out.append((j0, i0, m))
    return out


def _outline(m, x0, y0, res, margin_m):
    from scipy import ndimage as ndi
    edge = m & ~ndi.binary_erosion(m)
    jj, ii = np.nonzero(edge if edge.any() else m)
    xs, ys = x0 + (ii + 0.5) * res, y0 + (jj + 0.5) * res
    corners = np.concatenate([np.stack([xs + dx * res / 2, ys + dy * res / 2], 1) for dx in (-1, 1) for dy in (-1, 1)])
    return buffer_convex(convex_hull(corners), margin_m, n_arc=8)


def segment_mounds(z, seen, origin, res, area=None, above_m=STREET_ABOVE_M, min_m2=MIN_MOUND_M2,
                   min_h=MIN_MOUND_H, split_m2=SPLIT_M2, max_m2=MAX_PROPOSAL_M2, merge_gap_m=MERGE_GAP_M,
                   margin_m=OUTLINE_MARGIN_M, standing=None, obstacles=None, roof_cue=True):
    """Rubble mounds / collapsed lots in a (partly) mapped heightmap: [{polygon, area_m2, height_max_m,
    peak_xy, cells}]. Cells rising ``above_m`` over the local ground (``ground_level``) form components;
    standing buildings (``standing_core``) are cut out (their footprints appended to ``standing`` if a list
    is given: obstacles, not zones) leaving their debris skirt; large components are split at their height
    cores, still-oversized ones (> ``max_m2``) by k-means; fragments < 60 m2 join a neighbour within
    ``merge_gap_m`` or are dropped if smaller than ``min_m2`` / lower than ``min_h``.
    Buildings are removed BEFORE the components are formed: known ``obstacles`` (prior footprints, e.g. the
    scenario's ``site_info.obstacles``: [{polygon, height}]) and, with ``roof_cue``, ``roof_buildings`` found
    from the map geometry alone; both are appended to ``standing``."""
    from scipy import ndimage as ndi
    ground = ground_level(z, seen, res, area)
    hgt = np.where(seen, z - ground, 0.0)
    slope_all = patch_fields(Grid((0.0, 0.0), res, z.shape[1], z.shape[0], hgt.astype(np.float32)))[0]
    hi = seen & (hgt > above_m)
    if area is not None:
        hi &= area
    cut = np.zeros_like(hi)
    cell = res * res
    grow = max(1, int(round(1.0 / res)))
    for o in obstacles or []:
        j0, j1, i0, i1, sub = rasterise_box(o["polygon"], origin, res, z.shape[1], z.shape[0], dilate_m=1.0)
        if sub.size and (sub & seen[j0:j1, i0:i1]).any():
            cut[j0:j1, i0:i1] |= sub
            if standing is not None:
                standing.append({"polygon": ccw(o["polygon"]), "height_m": round(float(o.get("height", 0.0)), 1),
                                 "source": "prior"})
    if roof_cue:
        for j0, i0, m in roof_buildings(hgt, seen, res):
            mm = ndi.binary_dilation(np.pad(m, grow), iterations=grow)
            jj, ii = np.nonzero(mm)
            jj, ii = jj - grow + j0, ii - grow + i0
            ok = (jj >= 0) & (jj < z.shape[0]) & (ii >= 0) & (ii < z.shape[1])
            if cut[jj[ok], ii[ok]].mean() > 0.5:
                continue                               # already a known obstacle
            cut[jj[ok], ii[ok]] = True
            if standing is not None:
                standing.append({"polygon": _outline(m, origin[0] + i0 * res, origin[1] + j0 * res, res, 0.0),
                                 "height_m": round(float(hgt[j0:j0 + m.shape[0], i0:i0 + m.shape[1]][m].max()), 1),
                                 "source": "roof_cue"})
    if roof_cue:                            # any sizeable LEVEL surface >= ROOF_MIN_H up is a roof (pancaked
        lvl = seen & (hgt >= ROOF_MIN_H) & (slope_all < ROOF_LEVEL_DEG)     # slabs and rubble are never level)
        lvl = ndi.binary_opening(ndi.binary_closing(lvl, iterations=2) & (hgt >= ROOF_MIN_H), iterations=1)
        lab_l, n_l = ndi.label(lvl & ~cut)
        for k, sl in enumerate(ndi.find_objects(lab_l), 1):
            if sl is None:
                continue
            m = lab_l[sl] == k
            if m.sum() * cell < ROOF_MIN_M2:
                continue
            j0, i0 = sl[0].start, sl[1].start
            mm = ndi.binary_dilation(np.pad(m, grow), iterations=grow)
            jj, ii = np.nonzero(mm)
            jj, ii = jj - grow + j0, ii - grow + i0
            ok = (jj >= 0) & (jj < z.shape[0]) & (ii >= 0) & (ii < z.shape[1])
            cut[jj[ok], ii[ok]] = True
            if standing is not None:
                standing.append({"polygon": _outline(m, origin[0] + i0 * res, origin[1] + j0 * res, res, 0.0),
                                 "height_m": round(float(hgt[sl][m].max()), 1), "source": "level_roof"})
    hi = ndi.binary_closing(ndi.binary_opening(hi & ~cut, iterations=1), iterations=2) & seen & (hgt > 0.3) & ~cut
    lab, n = ndi.label(hi, structure=np.ones((3, 3)))
    cell = res * res
    pad = max(2, int(round((merge_gap_m + 2.0) / res)))
    parts = []                                      # (slice_j, slice_i, mask) in cropped coordinates
    for k, sl in enumerate(ndi.find_objects(lab), 1):
        if sl is None:
            continue
        j0, j1 = max(0, sl[0].start - pad), min(z.shape[0], sl[0].stop + pad)
        i0, i1 = max(0, sl[1].start - pad), min(z.shape[1], sl[1].stop + pad)
        m = lab[j0:j1, i0:i1] == k
        hc = hgt[j0:j1, i0:i1]
        pieces = [m]
        core = standing_core(hc, m, res) if m.sum() * cell >= 60.0 else None
        if core is None:                    # level roof cells dominate the tall part: a (damaged) standing block
            tall = m & (hc >= ROOF_MIN_H)
            lvl = ndi.binary_closing(tall & (slope_all[j0:j1, i0:i1] < ROOF_LEVEL_DEG), iterations=2) & tall
            if lvl.sum() * cell >= ROOF_MIN_M2 and lvl.sum() >= 0.6 * max(tall.sum(), 1):
                core = lvl
        if core is not None:
            if standing is not None:
                standing.append({"polygon": _outline(core, origin[0] + i0 * res, origin[1] + j0 * res, res, 0.0),
                                 "height_m": round(float(hc[core].max()), 1), "source": "core"})
            skirt = m & ~ndi.binary_dilation(core, iterations=max(1, int(round(1.5 / res))))
            a = skirt.sum() * cell
            pieces = []
            if a >= min_m2:
                jj, ii = np.nonzero(skirt)
                kk = max(2, int(np.ceil(a / (0.5 * split_m2))))
                labk = kmeans_split(np.stack([ii, jj], 1).astype(float), min(kk, max(1, len(ii) // 20)))
                for q in range(labk.max() + 1):
                    mm = np.zeros_like(skirt)
                    mm[jj[labk == q], ii[labk == q]] = True
                    pieces.append(mm)
        out = []
        for m2 in pieces:
            a = m2.sum() * cell
            if a > split_m2:
                hmax = float(hc[m2].max())
                cl, nc = ndi.label(m2 & (hc > above_m + 0.5 * (hmax - above_m)), structure=np.ones((3, 3)))
                sizes = ndi.sum(np.ones_like(cl), cl, range(1, nc + 1)) * cell
                keep = [q + 1 for q, sz in enumerate(sizes) if sz >= min_m2]
                if len(keep) >= 2:
                    _, idx = ndi.distance_transform_edt(~np.isin(cl, keep), return_indices=True)
                    near = cl[idx[0], idx[1]]
                    out += [m2 & (near == q) for q in keep]
                    continue
            out.append(m2)
        for m2 in out:
            a = m2.sum() * cell
            if a > max_m2:
                jj, ii = np.nonzero(m2)
                labk = kmeans_split(np.stack([ii, jj], 1).astype(float), int(np.ceil(a / max_m2)))
                for q in range(labk.max() + 1):
                    mm = np.zeros_like(m2)
                    mm[jj[labk == q], ii[labk == q]] = True
                    parts.append((j0, i0, mm))
            else:
                parts.append((j0, i0, m2))
    # merge fragments into a neighbour within merge_gap_m (cropped boxes, full-frame offsets)
    big = [p for p in parts if p[2].sum() * cell >= 60.0]
    small = [p for p in parts if p[2].sum() * cell < 60.0]
    gap = max(1, int(round(merge_gap_m / res)))

    def full_idx(p):
        jj, ii = np.nonzero(p[2])
        return jj + p[0], ii + p[1]

    groups = [[b] for b in big]
    boxes = []
    for b in big:
        jj, ii = full_idx(b)
        boxes.append(set(zip(jj.tolist(), ii.tolist())))
    for sm in small:
        g = ndi.binary_dilation(np.pad(sm[2], gap), iterations=gap)
        jj, ii = np.nonzero(g)
        cells = set(zip((jj - gap + sm[0]).tolist(), (ii - gap + sm[1]).tolist()))
        hit = next((q for q, bx in enumerate(boxes) if not bx.isdisjoint(cells)), None)
        if hit is None:
            groups.append([sm])
        else:
            groups[hit].append(sm)
    mounds = []

    def emit(jj, ii, depth=0):
        """Outline cells (convex hull + margin); if the hull swallows a building that was cut out (> 15% of
        its area on cut cells), split the cells in two (k-means) and outline each part, up to 4 levels."""
        a = float(len(jj) * cell)
        if a < min_m2:
            return
        hv = hgt[jj, ii]
        h = float(hv.max())
        if h < min_h:
            return
        j0, i0 = jj.min(), ii.min()
        m = np.zeros((jj.max() - j0 + 1, ii.max() - i0 + 1), bool)
        m[jj - j0, ii - i0] = True
        poly = _outline(m, origin[0] + i0 * res, origin[1] + j0 * res, res, margin_m)
        if cut.any() and depth < 4 and len(jj) >= 2 * min_m2 / cell:
            b0, b1, c0, c1, sub = rasterise_box(poly, origin, res, z.shape[1], z.shape[0])
            if sub.any() and (cut[b0:b1, c0:c1] & sub).sum() > 0.15 * sub.sum():
                lab2 = kmeans_split(np.stack([ii, jj], 1).astype(float), 2)
                for q in (0, 1):
                    emit(jj[lab2 == q], ii[lab2 == q], depth + 1)
                return
        k = int(np.argmax(hv))
        mounds.append({"polygon": poly, "area_m2": polygon_area(poly), "height_max_m": h,
                       "peak_xy": (float(origin[0] + (ii[k] + 0.5) * res), float(origin[1] + (jj[k] + 0.5) * res)),
                       "cells": int(len(jj))})

    for grp in groups:
        emit(np.concatenate([full_idx(p)[0] for p in grp]), np.concatenate([full_idx(p)[1] for p in grp]))
    return sorted(mounds, key=lambda d: (-d["peak_xy"][1], d["peak_xy"][0]))


def score_proposals(mounds, clips=(), pile_xy=None, human_min=0.4):
    """zone_proposals (contract) from mounds and the recon clips heard so far, best first."""
    out = []
    pos = []
    for c in clips:
        if clip_human(c) < human_min:
            continue
        p, how = clip_position(c, pile_xy)
        if p is not None and how != "single bearing":
            pos.append((c, p))
    for m in mounds:
        leads = [c for c, p in pos if points_in_polygon(p[0], p[1], m["polygon"])]
        h, a = m["height_max_m"], m["area_m2"]
        ac = max([(c.get("priority") or 0) / 100.0 for c in leads] + [0.0])
        if leads and ac == 0.0:
            ac = 0.5
        score = 0.35 * min(h / 6.0, 1.0) + 0.25 * min(a / 400.0, 1.0) + 0.4 * ac
        top = max(leads, key=lambda c: c.get("priority") or 0) if leads else None
        if top is not None:
            cls = top.get("class") or top.get("sound_class") or "sound"
            said = top.get("transcript") or ""
            heard = f"; {cls}-like clip" + (f" '{said[:40]}'" if said else "") + " heard from it at a perch"
        else:
            heard = "; nothing heard from it yet (not cleared)"
        out.append({"polygon": [[round(float(x), 2), round(float(y), 2)] for x, y in m["polygon"]],
                    "kind": "rubble_mound", "height_max_m": round(h, 1), "area_m2": round(a),
                    "acoustic_leads": [str(c.get("id") or c.get("clip_id")) for c in leads],
                    "score": round(float(score), 2),
                    "why": f"mound {h:.0f} m high, {a:.0f} m²{heard}"})
    out.sort(key=lambda p: -p["score"])
    for i, p in enumerate(out, 1):
        p["id"] = f"P{i}"
    return [{"id": p["id"], **{k: v for k, v in p.items() if k != "id"}} for p in out]


def proposals_to_tasking(proposals, min_score=0.0, top_k=None, revision=0, source="auto-accepted-proposals"):
    """Tasking from accepted proposals (zone ids = proposal ids, priority = rank)."""
    acc = [p for p in proposals if p["score"] >= min_score][:top_k]
    return Tasking([Zone(p["id"], f"Mound {p['id']}", p["polygon"], float(i + 1), "tasked", p["why"])
                    for i, p in enumerate(acc)], revision, source)


# ============================================================================ recon controller
class Recon:
    """Drives the fleet through mapping, perch-and-listen and proposals over a FlightBackend."""

    def __init__(self, source_grid, flight, bounds=None, aoi=None, pile_xy=None, lidar_radius_m=None,
                 alt_margin_m=ALT_MARGIN_M, perch_every_m=None, listen_s=RECON_LISTEN_S, say=None, t0=0.0,
                 reserve=RECON_RESERVE, swap_s=RECON_SWAP_S, obstacles=None):
        self.src = source_grid                      # LiDAR stand-in: heightmap revealed within sensor range
        self.flight = flight
        g = source_grid
        self.origin, self.res, self.w, self.h = tuple(g.origin), float(g.res), int(g.w), int(g.h)
        self.bounds = list(bounds) if bounds is not None else list(g.bounds)
        self.aoi = as_poly(aoi) if aoi is not None else None
        X, Y = np.meshgrid(self.origin[0] + (np.arange(self.w) + 0.5) * self.res,
                           self.origin[1] + (np.arange(self.h) + 0.5) * self.res)
        self.X, self.Y = X, Y
        x0, y0, x1, y1 = self.bounds
        self.area = (X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)
        if self.aoi is not None:
            self.area &= rasterise(self.aoi, self.origin, self.res, self.w, self.h)
        self.z = np.zeros((self.h, self.w), np.float32)
        self.seen = np.zeros((self.h, self.w), bool)
        self.pile_xy = dict(pile_xy or {})
        self.obstacles = list(obstacles or [])         # optional prior building footprints [{polygon, height}]
        if lidar_radius_m is None:                 # auto: ~12 m on the 72 x 90 m grid, 25 m on a 320 m site
            lidar_radius_m = float(np.clip(np.sqrt(self.area.sum() * self.res ** 2) / 7.0, LIDAR_RADIUS_M,
                                           LIDAR_RADIUS_MAX_M))
        self.lidar_radius_m, self.alt_margin_m = lidar_radius_m, alt_margin_m
        self.perch_every_m, self.listen_s = perch_every_m, listen_s
        self.say = say or (lambda m: None)
        self.drones = list(flight.drones)
        self.paths = dict(zip(self.drones, coverage_path(self.bounds, len(self.drones), lidar_radius_m,
                                                         aoi=None if self.aoi is None else self.aoi.tolist())))
        if perch_every_m is None:                  # 90 m on small sites; ~PERCH_ROUNDS rounds on large ones
            perch_every_m = max(PERCH_EVERY_M, max(path_length(p) for p in self.paths.values()) / (PERCH_ROUNDS + 1))
        self.perch_every_m, self.reserve, self.swap_s = perch_every_m, reserve, swap_s
        self.swaps, self._swap_end = 0, {}
        self._hm_state, self._hm_t = None, None
        self.cursor = {d: 0 for d in self.drones}
        self.since = {d: 0.0 for d in self.drones}
        self.last_xy = {}
        self.mode = {d: "takeoff" for d in self.drones}   # takeoff | path | to_perch | swap | done
        self.status = "mapping"                           # mapping | listening | proposing | done
        self.perches, self.clips, self.proposals, self.standing = [], [], [], []
        self.leg = {}                                     # drone -> {"to": [x, y], "alt": commanded altitude}
        self.round = None                                 # {"drones": [...], "listen_end": t or None}
        self.wants_listen = None                          # {drone_id: xy} when a listen window has ended
        self.alt = None
        self._cruise0 = getattr(flight, "cruise", None)
        self.t = float(t0)
        self._reveal_initial()
        self._update_alt()
        for d in self.drones:
            flight.command({"drone_id": d, "type": "takeoff"})
        L = [path_length(p) for p in self.paths.values()]
        self.say(f"RECON: no search zones tasked yet -> mapping the site ({self._area_m2():.0f} m², "
                 f"{len(self.drones)} strips, {np.mean(L):.0f} m of path per drone (~{np.max(L) / 4.0 / 60:.0f} min at "
                 f"4 m/s), LiDAR swath ±{lidar_radius_m:.0f} m, each leg flown {alt_margin_m:.0f} m above the tallest "
                 f"mapped structure along it); perch-and-listen every "
                 f"{perch_every_m:.0f} m; satellite imagery is stale after a collapse, so the map starts empty")

    # ---------------------------------------------------------------- map
    def _area_m2(self):
        return float(self.area.sum()) * self.res ** 2

    def _reveal_initial(self):
        for p in self.flight.poses():
            self.reveal(p["pose"][:2], radius_m=6.0)

    def reveal(self, xy, radius_m=None):
        r = self.lidar_radius_m if radius_m is None else radius_m
        i0 = max(0, int((xy[0] - r - self.origin[0]) / self.res) - 1)
        i1 = min(self.w, int((xy[0] + r - self.origin[0]) / self.res) + 2)
        j0 = max(0, int((xy[1] - r - self.origin[1]) / self.res) - 1)
        j1 = min(self.h, int((xy[1] + r - self.origin[1]) / self.res) + 2)
        if i0 >= i1 or j0 >= j1:
            return
        m = (self.X[j0:j1, i0:i1] - xy[0]) ** 2 + (self.Y[j0:j1, i0:i1] - xy[1]) ** 2 <= r * r
        self.z[j0:j1, i0:i1][m] = self.src.z[j0:j1, i0:i1][m]
        self.seen[j0:j1, i0:i1] |= m

    def ingest_points(self, pts):
        """Real/raycast LiDAR points (N, 3) in the site frame: max height per cell."""
        pts = np.asarray(pts, float).reshape(-1, 3)
        i = np.floor((pts[:, 0] - self.origin[0]) / self.res).astype(int)
        j = np.floor((pts[:, 1] - self.origin[1]) / self.res).astype(int)
        ok = (i >= 0) & (i < self.w) & (j >= 0) & (j < self.h)
        i, j, zz = i[ok], j[ok], pts[ok, 2]
        fresh = ~self.seen[j, i]
        self.z[j[fresh], i[fresh]] = -1e9
        np.maximum.at(self.z, (j, i), zz.astype(np.float32))
        self.seen[j, i] = True

    @property
    def mapped_fraction(self):
        return float((self.seen & self.area).sum()) / max(int(self.area.sum()), 1)

    def mapped_grid(self):
        """sim.world.Grid of the fresh map (unmapped cells at street level)."""
        z = np.where(self.seen, self.z, street_level(self.z, self.seen, self.area)).astype(np.float32)
        return Grid(self.origin, self.res, self.w, self.h, z)

    def _update_alt(self):
        """Site-wide reference: highest mapped point + margin (used for legs over unmapped ground)."""
        top = float(self.z[self.seen].max()) if self.seen.any() else 0.0
        alt = top + self.alt_margin_m
        if self.alt is None or alt > self.alt + 0.5:
            self.alt = alt

    def leg_alt(self, a, b):
        """Altitude for a leg a -> b: tallest mapped structure within the LiDAR swath of the leg + margin; if
        part of the corridor is unmapped, the highest mapped point anywhere + margin (unknown = assume tall)."""
        r = self.lidar_radius_m * 0.5 + 3.0
        x0, x1 = min(a[0], b[0]) - r, max(a[0], b[0]) + r
        y0, y1 = min(a[1], b[1]) - r, max(a[1], b[1]) + r
        i0, i1 = max(0, int((x0 - self.origin[0]) / self.res)), min(self.w, int((x1 - self.origin[0]) / self.res) + 1)
        j0, j1 = max(0, int((y0 - self.origin[1]) / self.res)), min(self.h, int((y1 - self.origin[1]) / self.res) + 1)
        X, Y = self.X[j0:j1, i0:i1], self.Y[j0:j1, i0:i1]
        e = np.asarray(b[:2], float) - np.asarray(a[:2], float)
        L2 = max(float(e @ e), 1e-9)
        t = np.clip(((X - a[0]) * e[0] + (Y - a[1]) * e[1]) / L2, 0, 1)
        near = np.hypot(X - (a[0] + t * e[0]), Y - (a[1] + t * e[1])) <= r
        seen = self.seen[j0:j1, i0:i1] & near
        if not near.any():
            return self.alt
        if seen.sum() < 0.98 * near.sum():
            return self.alt
        return float(self.z[j0:j1, i0:i1][seen].max()) + self.alt_margin_m

    def _fly(self, d, x, y):
        """Command a leg at its own altitude (KinematicFlight routes at max(cruise, terrain + 3))."""
        p = next(q["pose"] for q in self.flight.poses() if q["id"] == d)
        alt = self.leg_alt(p[:2], (x, y))
        if self._cruise0 is not None:
            self.flight.cruise = alt - self.flight.stagger.get(d, 0.0) if hasattr(self.flight, "stagger") else alt
        self.leg[d] = {"to": [x, y], "alt": alt}
        ok = self.flight.command({"drone_id": d, "type": "goto", "destination": [x, y, alt]})
        if not ok and self._cruise0 is not None:     # backend ceiling (KinematicFlight: 25 m): its own terrain + 3 m
            self.flight.cruise = self._cruise0
            ok = self.flight.command({"drone_id": d, "type": "goto", "destination": [x, y, self._cruise0]})
            if ok and not getattr(self, "_ceiling_said", False):
                self._ceiling_said = True
                self.say(f"RECON: {alt:.0f} m is above the flight backend's ceiling; flying its terrain-following "
                         "altitude (terrain + 3 m) instead")
        return ok

    def _replan(self):
        """Newly mapped obstacles on a drone's current leg -> re-command the leg higher."""
        for d, lg in self.leg.items():
            if self.mode[d] != "path":
                continue
            p = next(q["pose"] for q in self.flight.poses() if q["id"] == d)
            need = self.leg_alt(p[:2], lg["to"])
            if need > lg["alt"] + 0.5:
                self.say(f"RECON: {d} leg to ({lg['to'][0]:.0f}, {lg['to'][1]:.0f}) re-planned: newly mapped "
                         f"structure -> {need:.1f} m")
                self._fly(d, *lg["to"])

    # ---------------------------------------------------------------- control loop
    def _goto_next(self, d):
        path = self.paths[d]
        if self.cursor[d] >= len(path):
            self.mode[d] = "done"
            self.flight.command({"drone_id": d, "type": "hold"})
            return
        x, y = path[self.cursor[d]]
        self.cursor[d] += 1
        self.mode[d] = "path"
        if not self._fly(d, x, y):
            self.say(f"RECON: {d} cannot reach waypoint ({x:.0f}, {y:.0f}) safely; skipping it")
            self._goto_next(d)

    def step(self, t, events=()):
        self.t = float(t)
        poses = {p["id"]: p for p in self.flight.poses()}
        for d, p in poses.items():
            xy = p["pose"][:2]
            if p["state"] in ("flying", "hovering") and self.mode[d] in ("path", "takeoff", "to_perch", "done"):
                self.reveal(xy)
            if d in self.last_xy and self.mode[d] == "path":
                self.since[d] += float(np.hypot(xy[0] - self.last_xy[d][0], xy[1] - self.last_xy[d][1]))
            self.last_xy[d] = xy
        self._update_alt()
        self._replan()
        arrived = {e["drone_id"] for e in events if e.get("type") == "arrived"}
        if self.status == "mapping":
            for d in self.drones:
                st, bat, job = self.flight.drone_state(d)
                if self.mode[d] == "path" and bat < self.reserve:
                    self.mode[d] = "swap"
                    self._swap_end[d] = None
                    self.cursor[d] = max(0, self.cursor[d] - 1)
                    self.flight.command({"drone_id": d, "type": "return"})
                    self.say(f"RECON: {d} battery {100 * bat:.0f}% < {100 * self.reserve:.0f}% reserve -> back to its pad "
                             f"for a {self.swap_s:.0f} s battery swap, then resumes its strip")
                    continue
                if self.mode[d] == "swap":
                    if st == "docked" and self._swap_end[d] is None:
                        self._swap_end[d] = self.t + self.swap_s
                    elif self._swap_end[d] is not None and self.t >= self._swap_end[d]:
                        if hasattr(self.flight, "swap_battery"):
                            self.flight.swap_battery(d)
                        self.swaps += 1
                        self.mode[d] = "takeoff"
                        self.flight.command({"drone_id": d, "type": "takeoff"})
                    continue
                if self.mode[d] == "takeoff" and st == "hovering" and job is None:
                    self._goto_next(d)
                elif self.mode[d] == "path" and (d in arrived or (st == "hovering" and job is None)):
                    self._goto_next(d)
            flying = [d for d in self.drones if self.mode[d] in ("path", "done")]
            due = all(self.since[d] >= self.perch_every_m or self.mode[d] == "done" for d in self.drones) \
                and any(self.since[d] >= self.perch_every_m * 0.5 for d in self.drones)
            if len(flying) == len(self.drones) and due:
                self._start_round()
            elif all(self.mode[d] == "done" for d in self.drones):
                self._finish_mapping()
        elif self.status == "listening":
            r = self.round
            if r["listen_end"] is None and all(self.flight.drone_state(d)[0] == "scanning" for d in r["drones"]):
                r["listen_end"] = self.t + self.listen_s
                self.say(f"RECON: {len(r['drones'])} drones perched, rotors off, listening for {self.listen_s:.0f} s")
            elif r["listen_end"] is None and self.t - r["t"] > 120.0:
                self.say("RECON: perch landing timed out; resuming the mapping flight")
                self._end_round()
            elif r["listen_end"] is not None and self.t >= r["listen_end"] and self.wants_listen is None \
                    and not r.get("heard"):
                self.wants_listen = {d: tuple(poses[d]["pose"][:2]) for d in r["drones"]}
        return self.status

    @property
    def phase(self):
        return {"mapping": "recon_mapping", "listening": "recon_listening"}.get(self.status, "recon_proposing")

    # ---------------------------------------------------------------- perch-and-listen
    def perch_points(self, near, n=None, avoid=()):
        """Flat cells of the fresh map near unmapped or high rubble, spread out; one per entry of ``near``
        ({drone_id: xy}). Returns {drone_id: [x, y, z]} (drones without a suitable cell are left out)."""
        from scipy.ndimage import uniform_filter
        flat = flat_cells(self.z, self.seen, self.res) & self.area
        if not flat.any():
            return {}
        street = street_level(self.z, self.seen, self.area)
        interest = ((~self.seen) & self.area).astype(float) + 2.0 * (self.seen & (self.z - street > 1.5))
        k = max(3, int(round(16.0 / self.res)) | 1)
        interest = uniform_filter(interest, size=k)
        out, used = {}, [np.asarray(a, float) for a in avoid]
        jj, ii = np.nonzero(flat)
        cx, cy = self.X[jj, ii], self.Y[jj, ii]
        base = interest[jj, ii]
        for d, xy in near.items():
            dist = np.hypot(cx - xy[0], cy - xy[1])
            ok = dist <= 30.0
            for u in used:
                ok &= np.hypot(cx - u[0], cy - u[1]) >= PERCH_SPACING_M
            if not ok.any():
                continue
            s = np.where(ok, base - 0.004 * dist, -np.inf)
            b = int(np.argmax(s))
            p = np.array([cx[b], cy[b]])
            used.append(p)
            out[d] = [round(float(p[0]), 2), round(float(p[1]), 2), round(float(self.z[jj[b], ii[b]]), 3)]
        return out

    def _start_round(self):
        near = {d: self.last_xy[d] for d in self.drones}
        prev = [p["xy"] for p in self.perches[-6:]]
        pts = self.perch_points(near, avoid=prev)
        for d in self.drones:
            self.since[d] = 0.0
        if len(pts) < 2:                              # one drone cannot localise a sound: skip this round
            self.say("RECON: fewer than two flat perch cells mapped near the drones; continuing the mapping flight")
            return
        self.status = "listening"
        self.round = {"drones": list(pts), "listen_end": None, "t": self.t}
        for d, p in pts.items():
            self.mode[d] = "to_perch"
            self.flight.command({"drone_id": d, "type": "goto_land_scan", "destination": p,
                                 "landing_site_id": f"perch-{len(self.perches) + 1}", "scan_duration": 1e6})
            self.perches.append({"drone_id": d, "xy": p[:2], "t": round(self.t, 1), "clips": []})
        self.say("RECON: perch-and-listen round " + ", ".join(f"{d} -> ({p[0]:.0f}, {p[1]:.0f})" for d, p in pts.items())
                 + " (flat cells of the fresh map near unmapped / high rubble)")

    def add_listen(self, clips, t=None):
        """Clips heard during the perch round (mission clip dicts: id, drone_id, ...). Resumes mapping."""
        mine = [p for p in self.perches if self.round and p["drone_id"] in self.round["drones"]][-len(self.round["drones"]):]
        for c in clips:
            self.clips.append(c)
            for p in mine:
                if p["drone_id"] == c.get("drone_id"):
                    p["clips"].append(str(c.get("id") or c.get("clip_id")))
        self.wants_listen = None
        self.round["heard"] = True
        self._end_round()

    def _end_round(self):
        resume = self.round["drones"] if self.round else []
        self.round = None
        for d in resume:
            if self.mode[d] == "to_perch":
                self.mode[d] = "path"
                self.cursor[d] = max(0, self.cursor[d] - 1)       # re-fly to the waypoint it was heading for
                self._goto_next(d)
        if all(self.mode[d] == "done" for d in self.drones):
            self._finish_mapping()
        else:
            self.status = "mapping"

    # ---------------------------------------------------------------- proposals
    def _finish_mapping(self):
        if self.status in ("proposing", "done"):
            return
        self.propose()
        self.status = "proposing"
        for d in self.drones:
            self.flight.command({"drone_id": d, "type": "return"})
        if self._cruise0 is not None:
            self.flight.cruise = max(self._cruise0, self.alt - self.alt_margin_m + 3.0)
        self.say(f"RECON: mapping done ({100 * self.mapped_fraction:.0f}% of the area mapped); "
                 f"{len(self.proposals)} zone proposals for the operator: "
                 + "; ".join(f"{p['id']} score {p['score']:.2f} ({p['why']})" for p in self.proposals[:5])
                 + ". Drones return to the pads and wait for tasking")

    def propose(self):
        self.standing = []
        mounds = segment_mounds(self.z, self.seen, self.origin, self.res, area=self.area, standing=self.standing,
                                obstacles=self.obstacles)
        if self.standing:
            src = Counter(s_.get("source") for s_ in self.standing)
            self.say(f"RECON: {len(self.standing)} standing building(s) kept as flight obstacles, not search zones "
                     f"({src.get('prior', 0)} from prior footprints, {len(self.standing) - src.get('prior', 0)} "
                     "found from the map: planar roof, rectangular, facades)")
        self.proposals = score_proposals(mounds, self.clips, self.pile_xy)
        return self.proposals

    def finish(self):
        """Tasking accepted: recon over (restores the flight backend's cruise altitude)."""
        self.status = "done"
        if self._cruise0 is not None:
            self.flight.cruise = self._cruise0

    # ---------------------------------------------------------------- state.json
    def heightmap_state(self, every_s=5.0):
        """``heightmap`` block of the fresh recon map (contract v1.2.1): ``z`` null and ``mask`` 0 where not
        yet mapped. Cached; rebuilt at most every ``every_s`` sim seconds (large sites)."""
        if self._hm_state is None or self._hm_t is None or self.t - self._hm_t >= every_s:
            z = np.round(self.z.astype(np.float64), 3)
            zl = np.where(self.seen, z, np.nan).ravel().tolist()
            self._hm_state = {"origin": list(self.origin), "res": self.res, "w": self.w, "h": self.h,
                              "z": [None if v != v else v for v in zl],
                              "mask": self.seen.astype(np.uint8).ravel().tolist()}
            self._hm_t = self.t
        return self._hm_state

    def to_state(self):
        path = [[x, y, round(float(self.alt), 1)] for d in self.drones for x, y in self.paths[d]]
        return {"recon": {"status": self.status, "mapped_fraction": round(self.mapped_fraction, 3), "path": path,
                          "perches": [dict(p, xy=list(p["xy"])) for p in self.perches]},
                "zone_proposals": [dict(p) for p in self.proposals]}
