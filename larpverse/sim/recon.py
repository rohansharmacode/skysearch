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

import numpy as np

from .landing import LIM_ROUGH, LIM_SLOPE, LIM_STEP, patch_fields
from .tasking import (Tasking, Zone, as_poly, buffer_convex, clip_human, clip_position, convex_hull,
                      points_in_polygon, polygon_area, rasterise)
from .world import Grid

LIDAR_RADIUS_M = 12.0          # ground swath radius of the down-looking LiDAR at recon altitude (conservative)
LANE_OVERLAP = 0.8             # lane spacing = 2 * radius * overlap
ALT_MARGIN_M = 13.0            # flight altitude above the highest mapped point (contract: about 12-15 m)
PERCH_EVERY_M = 90.0           # path length between perch-and-listen rounds (per drone)
RECON_LISTEN_S = 18.0          # rotors-off listening window at a perch (contract: about 15-20 s)
PERCH_SPACING_M = 15.0         # perches of one round at least this far apart
STREET_ABOVE_M = 1.0           # mound cells rise at least this far above street level
MIN_MOUND_M2 = 20.0
MIN_MOUND_H = 1.5
SPLIT_M2 = 600.0               # components larger than this are split at their cores
MERGE_GAP_M = 3.0              # fragments (< 60 m2) within this gap join their neighbour
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


def segment_mounds(z, seen, origin, res, area=None, above_m=STREET_ABOVE_M, min_m2=MIN_MOUND_M2,
                   min_h=MIN_MOUND_H, split_m2=SPLIT_M2, merge_gap_m=MERGE_GAP_M, margin_m=OUTLINE_MARGIN_M):
    """Rubble mounds in a (partly) mapped heightmap: [{polygon, area_m2, height_max_m, peak_xy, cells}]."""
    from scipy import ndimage as ndi
    street = street_level(z, seen, area)
    hi = seen & (z - street > above_m)
    if area is not None:
        hi &= area
    hi = ndi.binary_closing(ndi.binary_opening(hi, iterations=1), iterations=2) & seen
    lab, n = ndi.label(hi, structure=np.ones((3, 3)))
    cell = res * res
    comps = [lab == i for i in range(1, n + 1)]
    # split large components at their cores (local height threshold), cells go to the nearest core
    out = []
    for m in comps:
        if m.sum() * cell > split_m2:
            hmax = float(z[m].max() - street)
            core, nc = ndi.label(m & (z - street > above_m + 0.5 * (hmax - above_m)), structure=np.ones((3, 3)))
            sizes = ndi.sum(np.ones_like(core), core, range(1, nc + 1)) * cell
            keep = [i + 1 for i, s in enumerate(sizes) if s >= min_m2]
            if len(keep) >= 2:
                cores = np.isin(core, keep)
                _, idx = ndi.distance_transform_edt(~cores, return_indices=True)
                near = core[idx[0], idx[1]]
                out += [m & (near == k) for k in keep]
                continue
        out.append(m)
    # merge fragments into a neighbour within merge_gap_m, drop what is still too small or low
    big = [m for m in out if m.sum() * cell >= 60.0]
    small = [m for m in out if m.sum() * cell < 60.0]
    gap = max(1, int(round(merge_gap_m / res)))
    for m in small:
        grown = ndi.binary_dilation(m, iterations=gap)
        hit = [i for i, b in enumerate(big) if (grown & b).any()]
        if hit:
            big[hit[0]] = big[hit[0]] | m
        else:
            big.append(m)
    xs = origin[0] + (np.arange(z.shape[1]) + 0.5) * res
    ys = origin[1] + (np.arange(z.shape[0]) + 0.5) * res
    mounds = []
    for m in big:
        a = float(m.sum() * cell)
        h = float(z[m].max() - street)
        if a < min_m2 or h < min_h:
            continue
        jj, ii = np.nonzero(m)
        corners = np.concatenate([np.stack([xs[ii] + dx * res / 2, ys[jj] + dy * res / 2], 1)
                                  for dx in (-1, 1) for dy in (-1, 1)])
        poly = buffer_convex(convex_hull(corners), margin_m, n_arc=8)
        k = int(np.argmax(np.where(m, z, -np.inf)))
        mounds.append({"polygon": poly, "area_m2": polygon_area(poly), "height_max_m": h,
                       "peak_xy": (float(xs[k % z.shape[1]]), float(ys[k // z.shape[1]])), "cells": int(m.sum())})
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

    def __init__(self, source_grid, flight, bounds=None, aoi=None, pile_xy=None, lidar_radius_m=LIDAR_RADIUS_M,
                 alt_margin_m=ALT_MARGIN_M, perch_every_m=PERCH_EVERY_M, listen_s=RECON_LISTEN_S, say=None, t0=0.0):
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
        self.lidar_radius_m, self.alt_margin_m = lidar_radius_m, alt_margin_m
        self.perch_every_m, self.listen_s = perch_every_m, listen_s
        self.say = say or (lambda m: None)
        self.drones = list(flight.drones)
        self.paths = dict(zip(self.drones, coverage_path(self.bounds, len(self.drones), lidar_radius_m,
                                                         aoi=None if self.aoi is None else self.aoi.tolist())))
        self.cursor = {d: 0 for d in self.drones}
        self.since = {d: 0.0 for d in self.drones}
        self.last_xy = {}
        self.mode = {d: "takeoff" for d in self.drones}   # takeoff | path | to_perch | perched | done
        self.status = "mapping"                           # mapping | listening | proposing | done
        self.perches, self.clips, self.proposals = [], [], []
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
                 f"{len(self.drones)} strips, {np.mean(L):.0f} m of path per drone, LiDAR swath ±{lidar_radius_m:.0f} m, "
                 f"altitude {self.alt:.1f} m = highest mapped point + {alt_margin_m:.0f} m); perch-and-listen every "
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
        top = float(self.z[self.seen].max()) if self.seen.any() else 0.0
        alt = top + self.alt_margin_m
        if self.alt is None or alt > self.alt + 0.5:
            if self.alt is not None:
                self.say(f"RECON: newly mapped high point {top:.1f} m -> flight altitude raised to {alt:.1f} m")
            self.alt = alt
            if self._cruise0 is not None:
                self.flight.cruise = alt              # KinematicFlight: routes fly at max(cruise, terrain + 3)

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
        if not self.flight.command({"drone_id": d, "type": "goto", "destination": [x, y, self.alt]}):
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
        arrived = {e["drone_id"] for e in events if e.get("type") == "arrived"}
        if self.status == "mapping":
            for d in self.drones:
                st, _, job = self.flight.drone_state(d)
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
        mounds = segment_mounds(self.z, self.seen, self.origin, self.res, area=self.area)
        self.proposals = score_proposals(mounds, self.clips, self.pile_xy)
        return self.proposals

    def finish(self):
        """Tasking accepted: recon over (restores the flight backend's cruise altitude)."""
        self.status = "done"
        if self._cruise0 is not None:
            self.flight.cruise = self._cruise0

    # ---------------------------------------------------------------- state.json
    def to_state(self):
        path = [[x, y, round(float(self.alt), 1)] for d in self.drones for x, y in self.paths[d]]
        return {"recon": {"status": self.status, "mapped_fraction": round(self.mapped_fraction, 3), "path": path,
                          "perches": [dict(p, xy=list(p["xy"])) for p in self.perches]},
                "zone_proposals": [dict(p) for p in self.proposals]}
