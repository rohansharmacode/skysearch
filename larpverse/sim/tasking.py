"""Operator tasking (contracts/INTERFACES.md, "Operator tasking (v1.2)"): search zones and their progress.

Before deployment the operator outlines the rubble zones to search on the dashboard; the ops server writes
``dashboard/data/tasking.json``. The mission searches only inside tasked zones, tracks coverage per zone and
never re-searches a finished zone unless new evidence arrives (a new human-like acoustic clip associated
with it, or an operator re-task). Pure numpy; no dependency on the mission internals.

    from sim.tasking import load_tasking, default_tasking, ZoneTracker
    tk = load_tasking("dashboard/data/tasking.json") or default_tasking(scenario)
    tr = ZoneTracker(tk, piles=scenario.piles)          # piles: objects with .name .x .y .pieces (public geometry)
    tr.start_search("A2", t); tr.mark_scanning("A2", t); tr.add_scan(obs); tr.finish_search("A2", t, pings=["A2-R1"])
    tr.add_clip(clip)                                   # -> zone id (may reopen a finished zone) or None
    tr.suggest_zone(outside_clips)                      # -> zone_suggestions
    tr.update_tasking(load_tasking(path))               # live revision merge; progress kept
    state.update(tr.to_state())                         # {"zones": [...], "zone_suggestions": [...]}

Ground-truth boundary: the tracker sees zone outlines, public pile geometry, observation dicts (radar scans,
acoustic clips) and drone positions only. No detection is not proof of absence: ``searched_no_ping`` is a
searched zone that is NOT cleared.
"""
from __future__ import annotations

import hashlib
import json
import os
from dataclasses import dataclass, field

import numpy as np

SCHEMA = "larpverse.tasking/v1"
SITE_BOUNDS = (-36.0, -54.0, 36.0, 36.0)          # same as sim.scenario.SITE_BOUNDS (kept local: no import cycle)
ZONE_STATUSES = ("unsearched", "surveying", "scanning", "searched_no_ping", "ping", "lead",
                 "beyond_radar_reach", "cancelled")
CLOSED = ("searched_no_ping", "ping", "beyond_radar_reach")   # not revisited without new evidence
ACTIVE = ("surveying", "scanning")
HUMAN_MIN = 0.4                 # clips at or above this human score count as evidence / suggestions
REACH_CAP_M = 3.0               # radar max reliable depth used for zone coverage (literature ~3 m)
REACH_MIN = 0.2                 # finished zone with less radar coverage than this -> beyond_radar_reach
LISTEN_RANGE_M = 45.0           # a zone counts as listened to when a listening drone is this close
CLUSTER_M = 10.0                # suggestion clustering distance
SUGGEST_MIN_RADIUS = 6.0
NOMINAL_RANGE_M = 15.0          # single-bearing clip: assumed source distance for a suggestion


# ============================================================================ geometry (vectorised)
def as_poly(poly) -> np.ndarray:
    p = np.asarray(poly, float).reshape(-1, 2)
    if len(p) > 1 and np.allclose(p[0], p[-1]):
        p = p[:-1]
    return p


def ccw(poly) -> np.ndarray:
    """Polygon with counter-clockwise vertex order (contract v1.2.1: polygons are CCW, site metres)."""
    p = as_poly(poly)
    x, y = p[:, 0], p[:, 1]
    signed = np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))
    return p[::-1].copy() if signed < 0 else p


def points_in_polygon(x, y, poly) -> np.ndarray:
    """Even-odd rule, vectorised over points (any shape). Points exactly on an edge may go either way."""
    p = as_poly(poly)
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    inside = np.zeros(np.broadcast(x, y).shape, bool)
    xj, yj = p[-1]
    for xi, yi in p:
        cross = (yi > y) != (yj > y)
        with np.errstate(divide="ignore", invalid="ignore"):
            xint = (xj - xi) * (y - yi) / (yj - yi) + xi
        inside ^= cross & (x < xint)
        xj, yj = xi, yi
    return inside


def polygon_area(poly) -> float:
    p = as_poly(poly)
    x, y = p[:, 0], p[:, 1]
    return float(abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))) / 2.0)


def polygon_centroid(poly) -> tuple:
    p = as_poly(poly)
    x, y = p[:, 0], p[:, 1]
    xn, yn = np.roll(x, -1), np.roll(y, -1)
    c = x * yn - xn * y
    a = c.sum() / 2.0
    if abs(a) < 1e-9:
        return float(x.mean()), float(y.mean())
    return float(((x + xn) * c).sum() / (6 * a)), float(((y + yn) * c).sum() / (6 * a))


def distance_to_polygon(x, y, poly):
    """Distance (m) from point(s) to the zone: 0 inside, else to the nearest edge. Vectorised."""
    p = as_poly(poly)
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    d = np.full(np.broadcast(x, y).shape, np.inf)
    a = p
    b = np.roll(p, -1, axis=0)
    for (ax, ay), (bx, by) in zip(a, b):
        ex, ey = bx - ax, by - ay
        L2 = ex * ex + ey * ey
        t = np.clip(((x - ax) * ex + (y - ay) * ey) / L2, 0, 1) if L2 > 0 else 0.0
        d = np.minimum(d, np.hypot(x - (ax + t * ex), y - (ay + t * ey)))
    d = np.where(points_in_polygon(x, y, p), 0.0, d)
    return float(d) if d.ndim == 0 else d


def convex_hull(points) -> np.ndarray:
    """Andrew's monotone chain; counter-clockwise, no repeated end point."""
    pts = np.unique(np.asarray(points, float).reshape(-1, 2), axis=0)
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for q in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 0:
            lower.pop()
        lower.append(tuple(q))
    for q in pts[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 0:
            upper.pop()
        upper.append(tuple(q))
    return np.array(lower[:-1] + upper[:-1])


def buffer_convex(poly, margin_m, n_arc=12) -> np.ndarray:
    """Convex polygon grown by ``margin_m`` (Minkowski sum with a disc, arcs sampled at ``n_arc`` points)."""
    p = as_poly(poly)
    if margin_m <= 0:
        return convex_hull(p)
    a = np.linspace(0, 2 * np.pi, n_arc, endpoint=False)
    ring = margin_m / np.cos(np.pi / n_arc) * np.stack([np.cos(a), np.sin(a)], 1)   # circumscribed: >= margin
    return convex_hull((p[:, None, :] + ring[None, :, :]).reshape(-1, 2))


def raster_mesh(origin, res, w, h):
    xs = origin[0] + (np.arange(w) + 0.5) * res
    ys = origin[1] + (np.arange(h) + 0.5) * res
    return np.meshgrid(xs, ys)


def rasterise(poly, origin, res, w, h, dilate_m=0.0) -> np.ndarray:
    """Bool (h, w) mask of the cells whose centre lies in the polygon (or within ``dilate_m`` of it).
    Grid convention as contracts/INTERFACES.md: row-major, y outer, cell centre origin + (i + 0.5) * res."""
    p = as_poly(poly)
    out = np.zeros((h, w), bool)
    x0, y0 = p.min(0) - dilate_m - res
    x1, y1 = p.max(0) + dilate_m + res
    i0, i1 = max(0, int(np.floor((x0 - origin[0]) / res))), min(w, int(np.ceil((x1 - origin[0]) / res)) + 1)
    j0, j1 = max(0, int(np.floor((y0 - origin[1]) / res))), min(h, int(np.ceil((y1 - origin[1]) / res)) + 1)
    if i0 >= i1 or j0 >= j1:
        return out
    X, Y = raster_mesh((origin[0] + i0 * res, origin[1] + j0 * res), res, i1 - i0, j1 - j0)
    sub = distance_to_polygon(X, Y, p) <= dilate_m if dilate_m > 0 else points_in_polygon(X, Y, p)
    out[j0:j1, i0:i1] = sub
    return out


def rasterise_box(poly, origin, res, w, h, dilate_m=0.0):
    """Cropped ``rasterise``: (j0, j1, i0, i1, sub-mask) over the polygon's bounding box (scales to hundreds
    of zones on a 320 m site without a full-site mask per zone)."""
    p = as_poly(poly)
    x0, y0 = p.min(0) - dilate_m - res
    x1, y1 = p.max(0) + dilate_m + res
    i0, i1 = max(0, int(np.floor((x0 - origin[0]) / res))), min(w, int(np.ceil((x1 - origin[0]) / res)) + 1)
    j0, j1 = max(0, int(np.floor((y0 - origin[1]) / res))), min(h, int(np.ceil((y1 - origin[1]) / res)) + 1)
    if i0 >= i1 or j0 >= j1:
        return 0, 0, 0, 0, np.zeros((0, 0), bool)
    X, Y = raster_mesh((origin[0] + i0 * res, origin[1] + j0 * res), res, i1 - i0, j1 - j0)
    sub = distance_to_polygon(X, Y, p) <= dilate_m if dilate_m > 0 else points_in_polygon(X, Y, p)
    return j0, j1, i0, i1, sub


def grid_mask(poly, grid, dilate_m=0.0) -> np.ndarray:
    """``rasterise`` on any grid-like object with ``origin``, ``res``, ``w``, ``h`` (sim.world.Grid)."""
    return rasterise(poly, grid.origin, grid.res, grid.w, grid.h, dilate_m)


def piece_footprint(piece) -> np.ndarray:
    """(8, 2) xy corners of a box piece (sim.scenario.Piece or its dict)."""
    get = (lambda k: piece[k]) if isinstance(piece, dict) else (lambda k: getattr(piece, k))
    r, p, y = np.radians(get("rpy_deg"))
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    R = (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
         @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))
    half = np.asarray(get("size"), float) / 2
    s = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)], float) * half
    return (s @ R.T + np.asarray(get("center"), float))[:, :2]


def _pile_attr(p, k):
    return p[k] if isinstance(p, dict) else getattr(p, k)


def pile_outline(pile, margin_m=2.0) -> np.ndarray:
    """Convex hull of a pile's piece footprints plus ``margin_m`` (a disc around the centre if no pieces)."""
    pcs = _pile_attr(pile, "pieces") or []
    if pcs:
        pts = np.concatenate([piece_footprint(pc) for pc in pcs])
    else:
        a = np.linspace(0, 2 * np.pi, 16, endpoint=False)
        pts = np.stack([_pile_attr(pile, "x") + 6 * np.cos(a), _pile_attr(pile, "y") + 6 * np.sin(a)], 1)
    return buffer_convex(convex_hull(pts), margin_m)


STANDING_KINDS = {"standing", "intact", "building", "standing_building", "damaged_standing", "obstacle"}


def pile_kind(scenario, pile):
    """Sector kind if the scenario says (``Pile.kind`` / ``.state``, ``scenario.sector_kind(name)`` or a
    public sector entry's ``kind`` / ``state``), else None (= rubble)."""
    for k in ("kind", "state", "sector_kind"):
        v = getattr(pile, k, None) if not isinstance(pile, dict) else pile.get(k)
        if isinstance(v, str):
            return v
    name = _pile_attr(pile, "name")
    fn = getattr(scenario, "sector_kind", None)
    if callable(fn):
        try:
            return fn(name)
        except Exception:                                  # noqa: BLE001
            pass
    for sec in (getattr(scenario, "_sectors", None) or getattr(scenario, "sectors", None) or []):
        if isinstance(sec, dict) and sec.get("name") == name:
            return sec.get("kind") or sec.get("state")
    return None


def sector_outline(scenario, pile, margin_m=2.0) -> np.ndarray:
    """Zone outline of one scenario sector: its own polygon if the scenario provides one (``Pile.polygon`` /
    ``outline`` / ``footprint`` or a public sector ``polygon``), else the hull of its pieces' footprints, else
    its public bbox, else an ellipse from its spread (``sx``, ``sy``); plus ``margin_m``."""
    name = _pile_attr(pile, "name")
    for k in ("polygon", "outline", "footprint"):
        v = getattr(pile, k, None) if not isinstance(pile, dict) else pile.get(k)
        if v is not None and len(as_poly(v)) >= 3:
            return buffer_convex(convex_hull(as_poly(v)), margin_m)
    pcs = _pile_attr(pile, "pieces") if (isinstance(pile, dict) and "pieces" in pile) or hasattr(pile, "pieces") else None
    if pcs:
        return pile_outline(pile, margin_m)
    for sec in (getattr(scenario, "_sectors", None) or getattr(scenario, "sectors", None) or []):
        if isinstance(sec, dict) and sec.get("name") == name:
            if sec.get("polygon") is not None:
                return buffer_convex(convex_hull(as_poly(sec["polygon"])), margin_m)
            if sec.get("bbox") is not None:
                x0, y0, x1, y1 = sec["bbox"]
                return buffer_convex(np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], float), margin_m)
    cx, cy = float(_pile_attr(pile, "x")), float(_pile_attr(pile, "y"))
    sx = float(getattr(pile, "sx", 6.0) if not isinstance(pile, dict) else pile.get("sx", 6.0))
    sy = float(getattr(pile, "sy", 6.0) if not isinstance(pile, dict) else pile.get("sy", 6.0))
    rot = float(getattr(pile, "rot", 0.0) if not isinstance(pile, dict) else pile.get("rot", 0.0))
    a = np.linspace(0, 2 * np.pi, 24, endpoint=False)
    u, v = sx * np.cos(a), sy * np.sin(a)
    pts = np.stack([cx + u * np.cos(rot) - v * np.sin(rot), cy + u * np.sin(rot) + v * np.cos(rot)], 1)
    return buffer_convex(pts, margin_m)


def piles_in_zone(poly, piles, min_frac=0.2):
    """[(pile name, fraction of its pieces' centres inside the zone)] for piles with fraction >= ``min_frac``
    (or whose centre is inside), largest fraction first. A zone may cover part of a pile or several piles."""
    out = []
    q = as_poly(poly)
    (bx0, by0), (bx1, by1) = q.min(0) - 15.0, q.max(0) + 15.0      # cheap prefilter (hundreds of sectors)
    for p in piles:
        pcs = _pile_attr(p, "pieces") or []
        cx, cy = float(_pile_attr(p, "x")), float(_pile_attr(p, "y"))
        if not (bx0 <= cx <= bx1 and by0 <= cy <= by1):
            continue
        if pcs:
            c = np.array([(pc["center"] if isinstance(pc, dict) else pc.center)[:2] for pc in pcs], float)
            frac = float(points_in_polygon(c[:, 0], c[:, 1], poly).mean())
        else:
            frac = 0.0
        if points_in_polygon(cx, cy, poly):
            frac = max(frac, min_frac)
        if frac >= min_frac:
            out.append((_pile_attr(p, "name"), round(frac, 3)))
    return sorted(out, key=lambda e: (-e[1], e[0]))


# ============================================================================ tasking document
@dataclass
class Zone:
    id: str
    name: str
    polygon: list
    priority: float | None = None
    status: str = "tasked"            # operator side: tasked | cancelled
    note: str = ""
    retask: object = None             # optional operator re-task token (any new value reopens a closed zone)

    def to_dict(self):
        d = {"id": self.id, "name": self.name, "polygon": [[round(float(x), 2), round(float(y), 2)] for x, y in self.polygon],
             "priority": self.priority, "status": self.status, "note": self.note}
        if self.retask is not None:
            d["retask"] = self.retask
        return d


@dataclass
class Tasking:
    zones: list = field(default_factory=list)
    revision: int = 0
    source: str = "operator"
    updated_at: str | None = None
    schema: str = SCHEMA
    aoi: list | None = None           # optional operator area of interest (polygon) for recon mapping

    @classmethod
    def from_dict(cls, doc) -> "Tasking":
        if not isinstance(doc, dict) or not isinstance(doc.get("zones"), list):
            raise ValueError("tasking must be an object with a 'zones' list")
        zones, seen = [], set()
        for i, z in enumerate(doc["zones"]):
            if not isinstance(z, dict):
                continue
            poly = ccw(z.get("polygon") or [[0, 0]])
            if len(poly) < 3 or not np.isfinite(poly).all() or polygon_area(poly) < 1e-6:
                continue                                   # degenerate outline: ignored
            zid = str(z.get("id") or f"Z{i + 1}")
            if zid in seen:
                zid = f"{zid}-{i + 1}"
            seen.add(zid)
            pr = z.get("priority")
            st = str(z.get("status") or "tasked")
            zones.append(Zone(zid, str(z.get("name") or zid), poly.tolist(),
                              float(pr) if isinstance(pr, (int, float)) and not isinstance(pr, bool) else None,
                              "cancelled" if st == "cancelled" else "tasked", str(z.get("note") or ""), z.get("retask")))
        aoi = None
        if isinstance(doc.get("aoi"), list):
            a = as_poly(doc["aoi"])
            aoi = a.tolist() if len(a) >= 3 and np.isfinite(a).all() and polygon_area(a) > 1.0 else None
        return cls(zones, int(doc.get("revision") or 0), str(doc.get("source") or "operator"),
                   doc.get("updated_at"), str(doc.get("schema") or SCHEMA), aoi)

    def to_dict(self):
        d = {"schema": self.schema, "revision": self.revision, "source": self.source,
             "updated_at": self.updated_at, "zones": [z.to_dict() for z in self.zones]}
        if self.aoi:
            d["aoi"] = [list(map(float, p)) for p in self.aoi]
        return d

    def active(self):
        return [z for z in self.zones if z.status != "cancelled"]

    def digest(self):
        return hashlib.sha1(json.dumps(self.to_dict()["zones"], sort_keys=True).encode()).hexdigest()


def load_tasking(path, allow_empty=False) -> Tasking | None:
    """Tasking from ``path``; None if the file is missing, unreadable or invalid, or (unless ``allow_empty``)
    has no usable zone. An empty tasking (no zones, maybe an ``aoi``) means: run recon (sim/recon.py)."""
    try:
        with open(path, encoding="utf-8") as f:
            tk = Tasking.from_dict(json.load(f))
    except (OSError, ValueError, TypeError):
        return None
    return tk if (tk.zones or allow_empty) else None


def default_tasking(scenario, margin_m=2.0) -> Tasking:
    """One zone per collapsed / rubble sector (standing buildings are obstacles, not zones): the sector's
    outline (convex hull of its pieces' footprints on the grid site) + ``margin_m``; id = sector name."""
    zones = []
    for p in scenario.piles:
        kind = pile_kind(scenario, p)
        if kind is not None and kind.lower() in STANDING_KINDS:
            continue
        n = _pile_attr(p, "name")
        zones.append(Zone(n, f"Pile {n}" if kind is None else f"{n} ({kind})",
                          sector_outline(scenario, p, margin_m).round(2).tolist()))
    return Tasking(zones, 0, "scenario-default")


# ============================================================================ clip geometry
def _clip_get(c, *keys, default=None):
    for k in keys:
        if isinstance(c, dict) and c.get(k) is not None:
            return c[k]
    return default


def _clip_human(c):
    s = _clip_get(c, "scores") or {}
    h = s.get("human") if isinstance(s, dict) else None
    if h is None:
        h = _clip_get(c, "human", "confidence")
    return float(h) if h is not None else 1.0


def _clip_rays(c):
    """[(origin xy, unit direction)] from a clip's per-drone bearings (or its single bearing)."""
    rays = []
    for b in (_clip_get(c, "bearings") or [c]):
        o, brg = b.get("origin"), b.get("bearing_deg")
        if o is None or brg is None:
            continue
        a = np.radians(float(brg))
        rays.append((np.asarray(o, float)[:2], np.array([np.cos(a), np.sin(a)])))
    return rays


def triangulate(rays):
    """Least-squares intersection of >= 2 bearing rays; None if near-parallel or behind an origin."""
    if len(rays) < 2:
        return None
    A, b = np.zeros((2, 2)), np.zeros(2)
    for o, d in rays:
        P = np.eye(2) - np.outer(d, d)
        A += P
        b += P @ o
    if np.linalg.det(A) < 0.05 * len(rays):
        return None
    p = np.linalg.solve(A, b)
    if any(np.dot(p - o, d) <= 0 for o, d in rays):
        return None
    return p


def clip_position(clip, pile_xy=None):
    """Best position estimate of a clip's source: explicit xy, else >= 2-bearing triangulation, else the
    associated pile's centre, else ``NOMINAL_RANGE_M`` along its bearing. Returns (xy, how) or (None, None)."""
    xy = _clip_get(clip, "xy")
    if xy is not None:
        return np.asarray(xy, float)[:2], "position"
    rays = _clip_rays(clip)
    p = triangulate(rays)
    if p is not None:
        return p, "bearing overlap"
    pid = _clip_get(clip, "pile_id")
    if pile_xy and pid in pile_xy:
        return np.asarray(pile_xy[pid], float), f"associated with pile {pid}"
    if rays:
        o, d = rays[0]
        return o + NOMINAL_RANGE_M * d, "single bearing"
    return None, None


def clip_human(clip):
    return _clip_human(clip)


# ============================================================================ tracker
class ZoneTracker:
    """Per-zone progress: status machine, coverage, pings, evidence, revisit rules, suggestions."""

    def __init__(self, tasking: Tasking | None, piles=None, pile_xy=None, bounds=SITE_BOUNDS, res=0.5,
                 reach_cap_m=REACH_CAP_M, reach_min=REACH_MIN, revisit_max=2, listen_range_m=LISTEN_RANGE_M,
                 human_min=HUMAN_MIN):
        self.piles = list(piles or [])
        self.pile_xy = dict(pile_xy or {})
        for p in self.piles:
            self.pile_xy.setdefault(_pile_attr(p, "name"), (float(_pile_attr(p, "x")), float(_pile_attr(p, "y"))))
        x0, y0, x1, y1 = bounds
        self.origin, self.res = (float(x0), float(y0)), float(res)
        self.w, self.h = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
        self.lidar = np.zeros((self.h, self.w), bool)
        self.radar = np.zeros((self.h, self.w), bool)
        self.reach_cap_m, self.reach_min, self.revisit_max = reach_cap_m, reach_min, revisit_max
        self.listen_range_m, self.human_min = listen_range_m, human_min
        self.zones = {}                 # id -> dict (insertion order = operator order)
        self.scans_seen = set()
        self.clips_seen = {}            # clip id -> zone id or None
        self._outside = {}              # clip id -> clip (human-like, outside every zone)
        self.suggestions = []
        self._n_sugg = 0
        self.revision, self.source, self._digest = None, None, None
        self.events = []                # (t, message) for the mission log
        if tasking is not None:
            self.update_tasking(tasking)

    # ---------------------------------------------------------------- zones
    def _new_zone(self, z: Zone):
        st = {"id": z.id, "name": z.name, "polygon": as_poly(z.polygon), "priority": z.priority, "note": z.note,
              "retask_token": z.retask, "status": "unsearched", "pings": [], "last_searched_t": None,
              "retask_reason": None, "reopened": False, "visits": 0, "evidence": [], "listened": False,
              "before_cancel": None}
        self._geom(st)
        return st

    def _geom(self, st):
        j0, j1, i0, i1, sub = rasterise_box(st["polygon"], self.origin, self.res, self.w, self.h)
        st["box"], st["sub"] = (j0, j1, i0, i1), sub
        st["n_cells"] = int(sub.sum())
        p = st["polygon"]
        st["bbox"] = (*p.min(0), *p.max(0))
        st["area_m2"] = polygon_area(st["polygon"])
        st["centroid"] = polygon_centroid(st["polygon"])
        st["piles"] = piles_in_zone(st["polygon"], self.piles) if self.piles else []

    def zone(self, zid):
        return self.zones[zid]

    def zone_ids(self, active_only=True):
        return [k for k, z in self.zones.items() if not active_only or z["status"] != "cancelled"]

    def zone_piles(self, zid):
        """Pile names whose pieces lie inside the zone (largest share first)."""
        return [n for n, _ in self.zones[zid]["piles"]]

    def zones_for_pile(self, pile):
        return [k for k, z in self.zones.items() if z["status"] != "cancelled" and pile in self.zone_piles(k)]

    def zone_at(self, x, y, margin_m=0.0):
        """Active zone containing (x, y) (within ``margin_m``): highest operator priority, then nearest centroid."""
        hits = [k for k, z in self.zones.items() if z["status"] != "cancelled"
                and z["bbox"][0] - margin_m <= x <= z["bbox"][2] + margin_m
                and z["bbox"][1] - margin_m <= y <= z["bbox"][3] + margin_m
                and distance_to_polygon(x, y, z["polygon"]) <= margin_m]
        if not hits:
            return None
        return min(hits, key=lambda k: (self._prio_key(k), np.hypot(x - self.zones[k]["centroid"][0],
                                                                   y - self.zones[k]["centroid"][1])))

    def _prio_key(self, zid):
        p = self.zones[zid]["priority"]
        return p if p is not None else 1e9

    def mask_on(self, zid, grid, dilate_m=0.0):
        """Zone mask on another grid (e.g. a pile's planning window); ``dilate_m`` grows it."""
        return grid_mask(self.zones[zid]["polygon"], grid, dilate_m)

    def distance(self, zid, x, y):
        return distance_to_polygon(x, y, self.zones[zid]["polygon"])

    # ---------------------------------------------------------------- status machine
    def _event(self, t, msg):
        self.events.append((None if t is None else round(float(t), 1), msg))
        return msg

    def start_search(self, zid, t):
        z = self.zones[zid]
        if z["status"] == "cancelled":
            raise ValueError(f"zone {zid} is cancelled")
        if z["status"] not in ACTIVE:
            z["visits"] += 1
        z["status"] = "surveying"
        z["reopened"] = False             # the re-task is being acted on; retask_reason stays as the record
        z["last_searched_t"] = round(float(t), 1)
        return self._event(t, f"zone {zid}: search started (visit {z['visits']})")

    def mark_scanning(self, zid, t=None):
        z = self.zones[zid]
        if z["status"] in ("unsearched", "surveying"):
            z["status"] = "scanning"
        if t is not None:
            z["last_searched_t"] = round(float(t), 1)

    def finish_search(self, zid, t, pings=(), lead=False):
        """End of a search of the zone: ping | beyond_radar_reach | lead | searched_no_ping."""
        z = self.zones[zid]
        for p in pings:
            if p not in z["pings"]:
                z["pings"].append(p)
        z["last_searched_t"] = round(float(t), 1)
        if z["status"] == "cancelled":
            return z["status"]
        cov = self.coverage(zid)
        if pings:
            z["status"] = "ping"
        elif cov["radar_reached"] < self.reach_min:
            z["status"] = "beyond_radar_reach"
        elif lead:
            z["status"] = "lead"
        else:
            z["status"] = "searched_no_ping"
        self._event(t, f"zone {zid}: {z['status']} (lidar {cov['lidar_mapped']:.0%}, radar {cov['radar_reached']:.0%}"
                       f"{', listened' if cov['listened'] else ''}; not cleared)" if z["status"] != "ping" else
                    f"zone {zid}: ping {', '.join(z['pings'])}")
        return z["status"]

    def add_ping(self, zid, ping_id):
        z = self.zones[zid]
        if ping_id not in z["pings"]:
            z["pings"].append(ping_id)

    def reopen(self, zid, reason, t=None):
        z = self.zones[zid]
        z["retask_reason"] = reason
        z["reopened"] = True
        self._event(t, f"zone {zid} reopened: {reason}")

    def should_visit(self, zid) -> bool:
        """Dispatch candidate? unsearched / in progress: yes; lead: while visits < revisit_max; finished
        (searched_no_ping, ping, beyond_radar_reach): only after new evidence or an operator re-task."""
        z = self.zones.get(zid)
        if z is None or z["status"] == "cancelled":
            return False
        if z["status"] in ("unsearched",) + ACTIVE:
            return True
        if z["status"] == "lead":
            return z["reopened"] or z["visits"] < self.revisit_max
        return bool(z["reopened"])

    def visit_candidates(self):
        """Zone ids with should_visit(), operator priority first, then operator order."""
        order = {k: i for i, k in enumerate(self.zones)}
        return sorted((k for k in self.zones if self.should_visit(k)), key=lambda k: (self._prio_key(k), order[k]))

    # ---------------------------------------------------------------- coverage
    def mark_lidar_mask(self, mask):
        self.lidar |= np.asarray(mask, bool)

    def _disc(self, layer, xy, r):
        i0 = max(0, int((xy[0] - r - self.origin[0]) / self.res) - 1)
        i1 = min(self.w, int((xy[0] + r - self.origin[0]) / self.res) + 2)
        j0 = max(0, int((xy[1] - r - self.origin[1]) / self.res) - 1)
        j1 = min(self.h, int((xy[1] + r - self.origin[1]) / self.res) + 2)
        if i0 >= i1 or j0 >= j1:
            return
        X, Y = raster_mesh((self.origin[0] + i0 * self.res, self.origin[1] + j0 * self.res), self.res, i1 - i0, j1 - j0)
        layer[j0:j1, i0:i1] |= (X - xy[0]) ** 2 + (Y - xy[1]) ** 2 <= r * r

    def mark_lidar_disc(self, xy, radius_m):
        self._disc(self.lidar, xy, radius_m)

    def mark_lidar_points(self, pts):
        pts = np.asarray(pts, float).reshape(-1, pts.shape[-1] if np.ndim(pts) == 2 else 2)
        i = np.floor((pts[:, 0] - self.origin[0]) / self.res).astype(int)
        j = np.floor((pts[:, 1] - self.origin[1]) / self.res).astype(int)
        ok = (i >= 0) & (i < self.w) & (j >= 0) & (j < self.h)
        self.lidar[j[ok], i[ok]] = True

    def add_scan(self, obs) -> bool:
        """Radar scan (contract observation): cells within the scan's reliable depth (capped at
        ``reach_cap_m``) of the landed sensor count as reached. Idempotent per scan id."""
        sid = obs.get("scan_id")
        if sid is not None and sid in self.scans_seen:
            return False
        if sid is not None:
            self.scans_seen.add(sid)
        if obs.get("doppler_moving"):
            return False
        reach = obs.get("max_reliable_depth_m")
        reach = self.reach_cap_m if reach is None else min(float(reach), self.reach_cap_m)
        x, y = obs["sensor_pose"][:2]
        self._disc(self.radar, (x, y), reach)
        return True

    def mark_listened(self, drone_xy, range_m=None):
        """Listening window over: zones within ``range_m`` of a listening drone count as listened to."""
        r = self.listen_range_m if range_m is None else range_m
        pts = list(drone_xy.values()) if isinstance(drone_xy, dict) else list(drone_xy)
        for z in self.zones.values():
            if z["status"] != "cancelled" and any(distance_to_polygon(p[0], p[1], z["polygon"]) <= r for p in pts):
                z["listened"] = True

    def coverage(self, zid):
        z = self.zones[zid]
        n = max(z["n_cells"], 1)
        j0, j1, i0, i1 = z["box"]
        return {"lidar_mapped": round(float((self.lidar[j0:j1, i0:i1] & z["sub"]).sum()) / n, 3),
                "radar_reached": round(float((self.radar[j0:j1, i0:i1] & z["sub"]).sum()) / n, 3),
                "listened": bool(z["listened"])}

    # ---------------------------------------------------------------- acoustic evidence
    def clip_xy(self, clip):
        """``clip_position`` with this tracker's pile centres."""
        return clip_position(clip, self.pile_xy)

    def assign_clip(self, clip):
        """Zone id for a clip: the first active zone containing its explicit xy, its >= 2-bearing
        triangulation or its associated pile's centre (the acoustic pipeline's association), in that order;
        with none of these, the first zone along its single bearing ray (2..60 m); else None. Noisy bearings
        therefore never pull a pile-associated clip out of its zone (no spurious suggestions)."""
        pts = []
        xy = _clip_get(clip, "xy")
        if xy is not None:
            pts.append(np.asarray(xy, float)[:2])
        tri = triangulate(_clip_rays(clip))
        if tri is not None:
            pts.append(tri)
        pid = _clip_get(clip, "pile_id")
        if pid in self.pile_xy:
            pts.append(np.asarray(self.pile_xy[pid], float))
        for p in pts:
            zid = self.zone_at(p[0], p[1], margin_m=1.0)
            if zid is not None:
                return zid
        if pts:
            return None
        for o, d in _clip_rays(clip)[:1]:
            for r in np.arange(2.0, 60.0, 0.5):
                q = o + r * d
                zid = self.zone_at(q[0], q[1])
                if zid is not None:
                    return zid
        return None

    def add_clip(self, clip, t=None):
        """Record a clip: returns its zone id (None = outside every zone; human-like ones feed suggestions).
        A new human-like clip in a finished zone reopens it (retask_reason)."""
        cid = str(_clip_get(clip, "id", "clip_id", "obs_id", default=f"c{len(self.clips_seen) + 1}"))
        if cid in self.clips_seen:
            return self.clips_seen[cid]
        zid = self.assign_clip(clip)
        self.clips_seen[cid] = zid
        human = _clip_human(clip) >= self.human_min
        t = _clip_get(clip, "t", "timestamp", default=t)
        if zid is None:
            if human:
                self._outside[cid] = dict(clip, id=cid)
            return None
        z = self.zones[zid]
        if not human:
            return zid
        z["evidence"].append(cid)
        z["listened"] = True
        newer = t is None or z["last_searched_t"] is None or float(t) > z["last_searched_t"]
        if z["status"] in CLOSED + ("lead",) and newer and not z["reopened"]:
            cls = _clip_get(clip, "class", "sound_class", default="sound")
            said = _clip_get(clip, "transcript", default="")
            self.reopen(zid, f"new acoustic clip {cid} ({cls}" + (f" \"{said[:40]}\"" if said else "")
                        + f") after the search at t={z['last_searched_t']}", t)
        return zid

    def suggest_zone(self, clips_outside=()):
        """Cluster the human-like clips heard outside every zone into suggestions
        {id, centre, radius_m, why, evidence}. Accumulates across calls; ids are stable."""
        for c in clips_outside:
            cid = str(_clip_get(c, "id", "clip_id", "obs_id", default=f"c{len(self._outside) + 1}"))
            if _clip_human(c) >= self.human_min and self.assign_clip(c) is None:
                self._outside[cid] = dict(c, id=cid)
        pts = []
        for cid, c in self._outside.items():
            p, how = self.clip_xy(c)
            if p is None or self.assign_clip(c) is not None:     # now inside a (new) zone
                continue
            pts.append((cid, c, p, how))
        pts.sort(key=lambda e: -_clip_human(e[1]))
        clusters = []
        for e in pts:
            for cl in clusters:
                if np.hypot(*(e[2] - np.mean([q[2] for q in cl], 0))) <= CLUSTER_M:
                    cl.append(e)
                    break
            else:
                clusters.append([e])
        old, out = list(self.suggestions), []
        for cl in clusters:
            P = np.array([q[2] for q in cl])
            c = P.mean(0)
            single = all(q[3] == "single bearing" for q in cl)
            rad = max(SUGGEST_MIN_RADIUS, float(np.max(np.hypot(*(P - c).T))) + 3.0, 10.0 if single else 0.0)
            match = next((s for s in old if np.hypot(c[0] - s["centre"][0], c[1] - s["centre"][1]) <= CLUSTER_M), None)
            if match is not None:
                old.remove(match)
                sid = match["id"]
            else:
                self._n_sugg += 1
                sid = f"S{self._n_sugg}"
            top = cl[0][1]
            cls = _clip_get(top, "class", "sound_class", default="sound")
            said = _clip_get(top, "transcript", default="")
            drones = sorted({b.get("drone_id") for q in cl for b in (_clip_get(q[1], "bearings") or [q[1]])
                             if b.get("drone_id")})
            hows = sorted({q[3] for q in cl})
            why = (f"{cls}" + (f" '{said[:40]}'" if said else "") + " heard"
                   + (f" by {' and '.join(drones)}" if drones else "") + f", {', '.join(hows)}; outside every tasked zone")
            out.append({"id": sid, "centre": [round(float(c[0]), 1), round(float(c[1]), 1)],
                        "radius_m": round(rad, 1), "why": why, "evidence": [q[0] for q in cl]})
        self.suggestions = out
        return out

    # ---------------------------------------------------------------- live tasking revisions
    def update_tasking(self, tasking: Tasking | None, t=None):
        """Merge a tasking revision: new zones -> unsearched, cancelled / removed zones stop being candidates,
        existing progress kept. A redrawn, un-cancelled or ``retask``-flagged finished zone is reopened.
        Returns the list of change messages (empty if nothing changed)."""
        if tasking is None:
            return []
        dig = tasking.digest()
        if dig == self._digest and tasking.revision == self.revision:
            return []
        first = self._digest is None
        rev = tasking.revision
        msgs, seen = [], set()
        for z in tasking.zones:
            seen.add(z.id)
            st = self.zones.get(z.id)
            if st is None:
                st = self.zones[z.id] = self._new_zone(z)
                if z.status == "cancelled":
                    st["status"] = "cancelled"
                if not first:
                    msgs.append(f"new zone {z.id} ({z.name}, {st['area_m2']:.0f} m²"
                                + (f", piles {', '.join(self.zone_piles(z.id))}" if st["piles"] else "") + ")")
                continue
            st["name"], st["priority"], st["note"] = z.name, z.priority, z.note
            redrawn = not (len(st["polygon"]) == len(z.polygon) and np.allclose(st["polygon"], z.polygon, atol=0.01))
            if redrawn:
                st["polygon"] = as_poly(z.polygon)
                self._geom(st)
            if z.status == "cancelled":
                if st["status"] != "cancelled":
                    st["before_cancel"] = st["status"]
                    st["status"] = "cancelled"
                    msgs.append(f"zone {z.id} cancelled by the operator")
                continue
            if st["status"] == "cancelled":
                st["status"] = st["before_cancel"] or "unsearched"
                if st["status"] in ACTIVE:
                    st["status"] = "unsearched" if not st["last_searched_t"] else "searched_no_ping"
                msgs.append(f"zone {z.id} re-tasked after a cancel")
                if st["status"] in CLOSED + ("lead",):
                    self.reopen(z.id, f"operator re-task (revision {rev})", t)
            elif st["status"] in CLOSED + ("lead",) and (redrawn or (z.retask is not None and z.retask != st["retask_token"])):
                self.reopen(z.id, f"operator {'redrew the zone' if redrawn else 're-task'} (revision {rev})", t)
                msgs.append(f"zone {z.id} reopened by the operator")
            elif redrawn:
                msgs.append(f"zone {z.id} redrawn")
            st["retask_token"] = z.retask
        for zid in list(self.zones):
            if zid in seen:
                continue
            st = self.zones[zid]
            if st["status"] == "unsearched" and not st["evidence"]:
                del self.zones[zid]                      # never searched, nothing heard: just drop it
                msgs.append(f"zone {zid} removed")
            elif st["status"] != "cancelled":
                st["before_cancel"], st["status"] = st["status"], "cancelled"
                msgs.append(f"zone {zid} no longer tasked (kept as cancelled; progress kept)")
        self.revision, self.source, self._digest = rev, tasking.source, dig
        if self._outside:
            self.suggest_zone([])                       # leads inside a newly tasked zone stop being suggestions
        for m in msgs:
            self._event(t, f"tasking revision {rev}: {m}")
        return msgs

    # ---------------------------------------------------------------- state.json
    def to_state(self):
        zones = []
        for zid, z in self.zones.items():
            zones.append({"id": zid, "name": z["name"],
                          "polygon": [[round(float(x), 2), round(float(y), 2)] for x, y in z["polygon"]],
                          "status": z["status"], "coverage": self.coverage(zid), "pings": list(z["pings"]),
                          "last_searched_t": z["last_searched_t"], "retask_reason": z["retask_reason"]})
        return {"zones": zones, "zone_suggestions": [dict(s) for s in self.suggestions]}

    def summary(self):
        """Short text for logs: 'A2 ping, B1 searched_no_ping, ...'."""
        return ", ".join(f"{k} {z['status']}" for k, z in self.zones.items())


def tasking_mtime(path):
    try:
        return os.stat(path).st_mtime_ns
    except OSError:
        return None
