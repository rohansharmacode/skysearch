"""Mission overlay driven ONLY by the state dict (contracts/INTERFACES.md v1 + v1.1 grid extensions).

Geometry is computed here in numpy; a backend draws it:
  * ``DebugDrawBackend``  isaacsim.util.debug_draw lines/points (+ USD prims for the ping spheres/beacons)
  * ``UsdBackend``        USD prims only (BasisCurves, Points, Spheres) - shows up in every renderer/capture
  * ``NullBackend``       records counts; used by ``--dry-run`` (no omni/pxr)

What is drawn: red ping spheres at (x, y, z_est) with a tall emissive red beacon (slow pulse, readable from
wide shots) and a 2-sigma surface ring per radar candidate; amber range rings for the active pile's scans;
yellow cones for human-audio clips (scores.human >= 0.4, the dashboard rule); a green dispatch arc; LiDAR
points in one purple (brightness by height), shown during the survey and faded out after it.
Operator search zones (state["zones"]) are thin outlines draped on the surface: dashed white = tasked /
unsearched, green = being searched, grey = searched (not cleared), red = ping, teal = acoustic lead. Recon
(state["recon"]): dashed low-alpha green coverage path in the air, green perch rings, and fine-dashed grey
proposed outlines (state["zone_proposals"]) until accepted (then they arrive as zones).

Colour system (same as dashboard/ and animation/): red = radar ping / candidate search region, yellow =
human-audio direction cone, teal = acoustic priority, amber = radar range rings, green = coordinator /
dispatch, purple = LiDAR. Text (``label_specs``) uses the dashboard pill style: IBM Plex Mono on dark.
"""
from __future__ import annotations

import math
import os

import numpy as np

RED = (0.95, 0.12, 0.10, 1.0)
RED_DIM = (0.95, 0.45, 0.25, 0.7)
AMBER = (1.0, 0.68, 0.10, 0.9)
YELLOW = (1.0, 0.92, 0.15, 0.95)
GREEN = (0.20, 0.95, 0.35, 1.0)
HUMAN = {"voice", "weak_voice", "shout", "knock", "knocking", "tapping", "human"}   # no longer used for cones (R2-007)
LIDAR_RGB = (0.61, 0.42, 1.0)       # #9b6bff, the animation's LiDAR purple (sRGB values)
# Drawing scale (metres per width unit for lines, point diameter, how many recent scans get range rings).
# isaac/photoreal.py lowers these for footage: thick tubes and big dots read as props in a realistic render.
LINE_W = 0.03
POINT_W = 0.09
MAX_SCANS = 12
LINE_MESH = False        # True: draw lines as flat ribbon meshes LINE_W * width metres wide (exact in every renderer)
# Ping beacon: column radius (m), emissive gain and pulse rate. ~0.3 m wide so it reads in wide shots.
BEACON_R = 0.15
BEACON_GLOW = 4.0
PULSE_HZ = 0.8                      # same period as the animation/dashboard ping ring
# LiDAR display: only while surveying, then fade out over LIDAR_FADE_S (render-clock seconds) and start a
# fresh map at the next survey. LIDAR_SURVEY_ONLY = False keeps the old behaviour (points stay forever).
LIDAR_SURVEY_ONLY = True
LIDAR_FADE_S = 2.5
SURVEY_PHASES = {"survey", "surveying", "mapping", "lidar", "recon_mapping"}
# Operator search zones (state["zones"], INTERFACES "Operator tasking (v1.2)"): thin draped outlines.
# Off with LARPVERSE_ZONES=0, Overlay(..., zones=False) or overlay.ZONES = False (run_mission --no-zones).
ZONES = os.environ.get("LARPVERSE_ZONES", "1") not in ("0", "false", "off", "no")
TEAL = (0.13, 0.76, 0.84, 1.0)      # #22c3d6 acoustic priority / lead
GREY = (0.55, 0.60, 0.65, 0.85)
WHITE = (0.90, 0.93, 0.95, 0.80)
ZONE_STYLE = {   # status -> (rgba, dashed)
    "tasked": (WHITE, True), "unsearched": (WHITE, True),
    "surveying": (GREEN, False), "scanning": (GREEN, False), "in_progress": (GREEN, False),
    "searched_no_ping": (GREY, False), "beyond_radar_reach": (GREY, True),
    "ping": (RED, False), "lead": (TEAL, False),
}
RECON_PATH = (*GREEN[:3], 0.35)     # recon coverage path in the air: dashed, low alpha
PROPOSAL = (*GREY[:3], 0.75)        # dashboard-proposed rubble outlines (until the operator accepts them)
ZONE_W = 2.0                        # width units (x LINE_W); thin enough not to read as a prop
# --overlay-style simple (Rohan, 2026-11-08: "too complicated"): one idea at a time. Only the zones that matter
# now (being searched, or newly added), drawn as clean flat outlines above the rubble; the latest dispatch arc;
# firm pings. No range rings, audio cones, recon paths, proposals, weak candidates or finished-zone outlines.
SIMPLE = False
SIMPLE_ZONES = {"surveying", "scanning", "in_progress", "unsearched", "tasked", "ping"}
CONE_W = 1.6                        # width units: thinner than zones (take2: 4x-wide cones read as loops)
CONE_HUMAN_MIN = 0.4                # dashboard/app.js: (c.scores?.human ?? 0) >= 0.4
# Footage clarity (set by photoreal --clean-overlay; the dashboard and state.json are unchanged):
HIDE_WEAK = False                   # skip "insufficient_evidence" candidates (no beacon, ring or tag)
RING_TOP = 0                        # >0: per radar scan draw only the N most confident range rings
SMOOTH_DRAPE = 0.0                  # >0: draped lines ride over the rubble (running max over +-this many m), not into it
ACTIVE = {"active", "survey", "surveying", "scanning", "searching", "in_progress", "radar", "landing",
          "relocate", "rescan", "acoustic"}
# Dashboard label look for any burn-in text (post-process / drawtext): dark pill, IBM Plex Mono, coloured border.
LABEL_STYLE = {
    "font": "IBM Plex Mono", "weight": 500, "size_px": 13, "fg": "#e4ebf0", "bg": "rgba(8,12,16,0.78)",
    "radius_px": 3, "pad_px": (3, 6),
    "border": {"pile": "rgba(150,180,200,0.18)", "ping": "#ff4545", "priority": "#22c3d6", "cone": "#ffe14a",
               "dispatch": "#6fd36f", "drone": "rgba(150,180,200,0.18)"},
    "text": {"pile": "#e4ebf0", "ping": "#ffd0d0", "priority": "#c4f3f8", "cone": "#fff3b0",
             "dispatch": "#c9f5c9", "drone": "#e4ebf0"},
}


def _ride_over(p, z, win):
    """Heights for a draped polyline that ride over the rubble instead of following every bump: running max
    over +-win metres of arc length, then a light running mean (keeps the line clear of the debris)."""
    d = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(p, axis=0).T))])
    zz = np.empty_like(z)
    j0 = j1 = 0
    for i in range(len(z)):
        while d[i] - d[j0] > win:
            j0 += 1
        while j1 + 1 < len(z) and d[j1 + 1] - d[i] <= win:
            j1 += 1
        zz[i] = z[j0:j1 + 1].max()
    k = max(1, min(len(zz) // 2, 3))
    ker = np.ones(2 * k + 1) / (2 * k + 1)
    return np.convolve(np.pad(zz, k, mode="edge"), ker, mode="valid")


def lidar_colors(z, lo=-0.5, hi=5.0):
    """Single-hue LiDAR purple, brightness 0.45..1 by height, RGBA float32. (No red/yellow: those mean ping/cone.)"""
    t = np.clip((np.asarray(z, float).ravel() - lo) / (hi - lo), 0, 1)
    rgb = (0.45 + 0.55 * t)[:, None] * np.asarray(LIDAR_RGB, float)[None, :]
    return np.concatenate([rgb, np.ones((len(rgb), 1))], 1).astype(np.float32)


height_colors = lidar_colors        # old name, kept for callers


class Overlay:
    def __init__(self, backend, surface_fn, pile_xy=None, base_xy=(0.0, -47.0), zones=None):
        self.b = backend
        self.zones = zones                   # None = follow the module flag ZONES
        self.surface = surface_fn            # (x, y) arrays -> z; public geometry (state heightmap or scenario)
        self.pile_xy = dict(pile_xy or {})
        self.base_xy = base_xy
        self.n_lines = 0
        self.n_pings = 0
        self.n_points = 0
        self.updates = 0
        self.labels = []                     # label_specs() of the last update (for burn-in text)
        self._surveying = False              # LiDAR display state (survey-only + fade)
        self._new_pts = False
        self._lidar_shown = False
        self._live_t = None
        self._alpha = 1.0

    # ------------------------------------------------------------------ helpers
    def _drape(self, xy, lift=0.15, max_seg=0.8):
        """Densify an (N, 2) polyline and lay it on the surface. Returns (M, 3)."""
        xy = np.asarray(xy, float)
        out = [xy[:1]]
        for a, b in zip(xy[:-1], xy[1:]):
            n = max(1, int(math.ceil(np.hypot(*(b - a)) / max_seg)))
            out.append(a + (b - a) * (np.arange(1, n + 1) / n)[:, None])
        p = np.concatenate(out)
        z = np.asarray(self.surface(p[:, 0], p[:, 1]), float)
        if SMOOTH_DRAPE > 0 and len(z) > 2:
            z = _ride_over(p, z, SMOOTH_DRAPE)
        return np.column_stack([p, z + lift])

    def _flat(self, ring, lift=0.6, max_seg=0.6):
        """Outline at one height just above the rubble inside it (no up-and-down drape over a fallen tower)."""
        ring = np.asarray(ring, float)
        dense = [ring[:1]]
        for a, b in zip(ring[:-1], ring[1:]):
            n = max(1, int(math.ceil(np.linalg.norm(b - a) / max_seg)))
            dense.append(a + (b - a) * (np.arange(1, n + 1) / n)[:, None])
        pts = np.concatenate(dense)
        lo, hi = ring.min(0), ring.max(0)
        gx, gy = np.meshgrid(np.linspace(lo[0], hi[0], 9), np.linspace(lo[1], hi[1], 9))
        zs = np.concatenate([np.asarray(self.surface(pts[:, 0], pts[:, 1]), float).ravel(),
                             np.asarray(self.surface(gx.ravel(), gy.ravel()), float).ravel()])
        z = float(np.percentile(zs, 90)) + lift
        return np.column_stack([pts, np.full(len(pts), z)])

    @staticmethod
    def _dashed(poly, rgba, width, acc, dash=1.0, gap=0.6):
        """Segments of a dense 3D polyline, keeping only the 'on' part of a dash pattern (by arc length)."""
        if len(poly) < 2:
            return
        d = np.linalg.norm(np.diff(poly, axis=0), axis=1)
        mid = np.concatenate([[0.0], np.cumsum(d)[:-1]]) + d / 2
        on = (mid % (dash + gap)) < dash
        if on.any():
            acc.append((poly[:-1][on], poly[1:][on], np.tile(rgba, (int(on.sum()), 1)), np.full(int(on.sum()), width)))

    @staticmethod
    def _segs(poly, rgba, width, acc):
        if len(poly) < 2:
            return
        acc.append((poly[:-1], poly[1:], np.tile(rgba, (len(poly) - 1, 1)), np.full(len(poly) - 1, width)))

    def _drone_xy(self, state, did):
        for d in state.get("drones", []):
            if d.get("id") == did:
                return d["pose"][:2]
        return None

    def active_pile(self, state):
        """Pile id of the current focus: an active-status pile, else the nearest pile to the deployed drones."""
        piles = state.get("piles") or []
        m = state.get("mission")
        if isinstance(m, dict) and m.get("active_pile"):
            return m["active_pile"]
        for p in piles:
            if p.get("status") in ACTIVE:
                return p["id"]
        for k in ("active_pile", "current_pile"):
            if state.get(k):
                return state[k]
        xy = [d["pose"][:2] for d in state.get("drones", [])
              if d.get("state") not in ("docked",) and np.hypot(d["pose"][0] - self.base_xy[0], d["pose"][1] - self.base_xy[1]) > 8]
        pxy = {p["id"]: p["xy"] for p in piles} or self.pile_xy
        if not xy or not pxy:
            return None
        c = np.mean(xy, 0)
        return min(pxy, key=lambda k: np.hypot(pxy[k][0] - c[0], pxy[k][1] - c[1]))

    # ------------------------------------------------------------------ main update
    def update(self, state):
        for p in state.get("piles") or []:
            self.pile_xy[p["id"]] = p["xy"]
        segs, pings = [], []
        active = self.active_pile(state)
        surveying = str(state.get("phase") or "") in SURVEY_PHASES
        if LIDAR_SURVEY_ONLY and surveying and not self._surveying and self._lidar_shown:
            self.b.clear_points()            # next pile's survey: start a fresh map
            self._lidar_shown, self._alpha = False, 1.0
        self._surveying = surveying

        # radar candidates: ping sphere + beacon + 2-sigma surface ring
        for c in state.get("radar_candidates") or []:
            x, y = map(float, c["xy"][:2])
            zs = float(self.surface(np.array([x]), np.array([y]))[0])
            ze = float(c.get("z_est", zs - 1.2) if c.get("z_est") is not None else zs - 1.2)
            weak = c.get("status") == "insufficient_evidence"
            if weak and (HIDE_WEAK or SIMPLE):
                continue
            col = RED_DIM if weak else RED
            pings.append({"id": str(c.get("id", f"R{len(pings)}")), "pos": (x, y, ze), "top": zs + 14.0,
                          "rgba": col, "radius": 0.35})
            cov = np.asarray(c.get("cov") or [[1, 0], [0, 1]], float)
            try:
                w, V = np.linalg.eigh(cov)
                w = np.sqrt(np.maximum(w, 0.04)) * 2.0
            except np.linalg.LinAlgError:
                w, V = np.array([2.0, 2.0]), np.eye(2)
            a = np.linspace(0, 2 * np.pi, 49)
            ell = (V @ np.stack([w[0] * np.cos(a), w[1] * np.sin(a)])).T + [x, y]
            if not SIMPLE:
                self._segs(self._drape(ell, 0.12), col, 3.0, segs)

        # operator search zones: ground-hugging outlines coloured by mission status
        if (ZONES if self.zones is None else self.zones):
            for z in state.get("zones") or []:
                poly = z.get("polygon") or []
                st = str(z.get("status") or "unsearched")
                if st == "cancelled" or len(poly) < 3:
                    continue
                rgba, dashed = ZONE_STYLE.get(st, (WHITE, True))
                ring = np.asarray([q[:2] for q in poly] + [poly[0][:2]], float)
                if SIMPLE:
                    if st not in SIMPLE_ZONES:
                        continue
                    line = self._flat(ring)
                else:
                    line = self._drape(ring, 0.12, max_seg=0.4)
                (self._dashed if dashed else self._segs)(line, rgba, ZONE_W, segs)

        # recon (INTERFACES "Reconnaissance mode (v1.2)"): coverage path in the air, perches, proposed outlines
        if (ZONES if self.zones is None else self.zones) and not SIMPLE:
            rc = state.get("recon") if isinstance(state.get("recon"), dict) else {}
            path = np.asarray([q[:3] for q in rc.get("path") or [] if len(q) >= 3], float)
            if len(path) >= 2 and rc.get("status") != "done":
                dense = [path[:1]]
                for a, b in zip(path[:-1], path[1:]):
                    n = max(1, int(math.ceil(np.linalg.norm(b - a) / 0.8)))
                    dense.append(a + (b - a) * (np.arange(1, n + 1) / n)[:, None])
                self._dashed(np.concatenate(dense), RECON_PATH, 1.5, segs, dash=1.5, gap=1.0)
            a = np.linspace(0, 2 * np.pi, 25)
            for pc in rc.get("perches") or []:
                xy = pc.get("xy") if isinstance(pc, dict) else None
                if xy is None or len(xy) < 2:
                    continue
                ring = np.stack([xy[0] + 0.9 * np.cos(a), xy[1] + 0.9 * np.sin(a)], 1)
                self._segs(self._drape(ring, 0.15, max_seg=0.4), GREEN, ZONE_W, segs)
            accepted = {str(z.get("id")) for z in state.get("zones") or []} | \
                       {str(z.get("proposal_id")) for z in state.get("zones") or [] if z.get("proposal_id")}
            for zp in state.get("zone_proposals") or []:
                poly = zp.get("polygon") or []
                if len(poly) < 3 or str(zp.get("id")) in accepted or zp.get("status") in ("accepted", "rejected"):
                    continue                  # accepted proposals come back as state["zones"] (tasked style)
                ring = np.asarray([q[:2] for q in poly] + [poly[0][:2]], float)
                self._dashed(self._drape(ring, 0.12, max_seg=0.4), PROPOSAL, ZONE_W, segs, dash=0.6, gap=0.6)

        # amber range rings for the active pile's scans
        pxy = self.pile_xy.get(active) if active else None
        scans = []
        for s in state.get("scans") or []:
            sp = s.get("pile_id") or s.get("pile")
            if active and sp is not None:
                if sp != active:
                    continue
            elif pxy is not None and np.hypot(s["sensor_pose"][0] - pxy[0], s["sensor_pose"][1] - pxy[1]) > 16:
                continue
            scans.append(s)
        for s in ([] if SIMPLE else scans[-MAX_SCANS:]):
            sx, sy = s["sensor_pose"][:2]
            confs = s.get("confidence") or []
            wgt = float(s.get("used_weight", 1.0) if s.get("used_weight") is not None else 1.0)
            rings = list(enumerate(s.get("range_candidates") or []))
            if RING_TOP > 0:
                rings = sorted(rings, key=lambda ir: -(float(confs[ir[0]]) if ir[0] < len(confs) else 0.5))[:RING_TOP]
            for i, r in rings:
                cf = float(confs[i]) if i < len(confs) else 0.5
                rgba = (*AMBER[:3], 0.25 + 0.7 * cf * max(wgt, 0.2))
                a = np.linspace(0, 2 * np.pi, max(32, int(r * 6)) + 1)
                ring = np.stack([sx + r * np.cos(a), sy + r * np.sin(a)], 1)
                self._segs(self._drape(ring, 0.2), rgba, 2.0 if cf < 0.5 else 3.0, segs)

        # yellow cones for human audio: scores.human >= 0.4 only (dashboard rule), latest listen only
        clips = state.get("acoustic_clips")
        if clips is None:
            clips = state.get("acoustic") or []
        listens = [c.get("listen") for c in clips if c.get("listen") is not None]
        last_listen = max(listens) if listens else None
        for c in ([] if SIMPLE else clips):
            sc = c.get("scores")
            if isinstance(sc, dict):
                human = float(sc.get("human") or 0.0) >= CONE_HUMAN_MIN
            else:                             # legacy v1 `acoustic` entries carry no scores: dashboard draws them
                human = bool(c.get("human", True))
            if last_listen is not None and c.get("listen", last_listen) != last_listen:
                continue
            if not human or c.get("bearing_deg") is None:
                continue
            o = c.get("origin") or self._drone_xy(state, c.get("drone_id"))
            if o is None:
                continue
            ox, oy = map(float, o[:2])
            b, hw = math.radians(float(c["bearing_deg"])), math.radians(float(c.get("width_deg", 30.0)) / 2)
            L = float(c.get("length_m", 22.0))
            arc = np.linspace(b - hw, b + hw, 13)
            edge = np.stack([ox + L * np.cos(arc), oy + L * np.sin(arc)], 1)
            cone = np.vstack([[ox, oy], edge, [[ox, oy]]])
            self._segs(self._drape(cone, 0.35), YELLOW, CONE_W, segs)
            mid = np.array([[ox, oy], [ox + L * np.cos(b), oy + L * np.sin(b)]])
            self._segs(self._drape(mid, 0.35), (*YELLOW[:3], 0.45), 1.5, segs)

        # green dispatch arc(s): base -> target
        disp = [d for d in state.get("dispatch") or [] if isinstance(d, dict)]
        if SIMPLE:
            disp = [d for d in disp if d.get("to_pile") or d.get("xy") or d.get("target_xy")][-1:]
        for d in disp:
            if not isinstance(d, dict):
                continue
            tgt = d.get("xy") or d.get("target_xy") or d.get("to")
            if not (isinstance(tgt, (list, tuple)) and len(tgt) >= 2):
                tgt = self.pile_xy.get(d.get("to_pile") or d.get("pile_id") or d.get("pile"))
            if tgt is None:
                continue
            src = d.get("from") if isinstance(d.get("from"), (list, tuple)) else \
                self.pile_xy.get(d.get("from_pile"), self.base_xy)
            p0, p1 = np.array(src[:2], float), np.array(tgt[:2], float)
            u = np.linspace(0, 1, 33)
            xy = p0 + (p1 - p0) * u[:, None]
            z0 = self.surface(xy[:, 0], xy[:, 1])
            if SIMPLE:                                # one clean arc over everything, not hugging the rubble
                top = float(np.max(np.asarray(z0, float)))
                arc = np.column_stack([xy, np.full(len(u), top + 1.5) + 8.0 * 4 * u * (1 - u)])
            else:
                arc = np.column_stack([xy, np.asarray(z0, float) + 0.4 + 10.0 * 4 * u * (1 - u)])
            self._segs(arc, GREEN, 4.0, segs)

        if segs:
            P0 = np.concatenate([s[0] for s in segs])
            P1 = np.concatenate([s[1] for s in segs])
            C = np.concatenate([s[2] for s in segs])
            W = np.concatenate([s[3] for s in segs])
        else:
            P0 = P1 = np.zeros((0, 3))
            C, W = np.zeros((0, 4)), np.zeros(0)
        self.b.set_lines(P0, P1, C, W)
        self.b.set_pings(pings)
        self.n_lines, self.n_pings = len(P0), len(pings)
        self.labels = self.label_specs(state)
        self.updates += 1
        return active

    def label_specs(self, state):
        """World-anchored text in the dashboard style (LABEL_STYLE), for burn-in by a post-process.

        Pile IDs (white), "A2-R1 · candidate search region" tags (red border) and teal acoustic-priority
        badges on acoustic-lead piles. Same names and wording as the dashboard and the animation; a ping is never "confirmed".
        """
        out = []
        cands = {str(c.get("id")): c for c in state.get("radar_candidates") or []}
        for p in state.get("piles") or []:
            x, y = map(float, p["xy"][:2])
            z = float(self.surface(np.array([x]), np.array([y]))[0])
            out.append({"kind": "pile", "text": str(p["id"]), "xyz": (x, y, z + 4.0)})
            pr = p.get("priority")
            if pr is not None and p.get("status") in ("lead", "acoustic_lead"):     # dashboard's teal badge
                out.append({"kind": "priority", "text": f"priority {int(round(float(pr)))}", "xyz": (x, y, z + 3.0)})
        for cid, c in cands.items():
            x, y = map(float, c["xy"][:2])
            z = float(self.surface(np.array([x]), np.array([y]))[0])
            weak = c.get("status") == "insufficient_evidence"
            if weak and (HIDE_WEAK or SIMPLE):
                continue
            out.append({"kind": "ping", "text": f"{cid} · {'weak candidate' if weak else 'candidate search region'}",
                        "xyz": (x, y, z + 2.0)})
        for o in out:
            o["fg"] = LABEL_STYLE["text"][o["kind"]]
            o["border"] = LABEL_STYLE["border"][o["kind"]]
        return out

    def add_points(self, pts):
        if pts is None or len(pts) == 0:
            return
        if LIDAR_SURVEY_ONLY and self._alpha < 1.0:      # points arriving again: show the map at full strength
            self._alpha = 1.0
            self.b.set_points_alpha(1.0)
        self.b.add_points(np.asarray(pts, np.float32), lidar_colors(pts[:, 2]))
        self.n_points += len(pts)
        self._new_pts = self._lidar_shown = True

    def tick(self, t):
        if LIDAR_SURVEY_ONLY and self._lidar_shown:
            if self._surveying or self._new_pts or self._live_t is None or t < self._live_t:
                self._live_t = t                 # (t < live_t: the caller switched clocks; restart the fade)
            self._new_pts = False
            a = float(np.clip(1.0 - (t - self._live_t) / max(LIDAR_FADE_S, 1e-6), 0.0, 1.0))
            a = math.ceil(a * 8) / 8             # 8 steps: each step rewrites the point widths
            if a != self._alpha:
                self._alpha = a
                self.b.set_points_alpha(a)
        self.b.tick(t)


# ---------------------------------------------------------------------- backends
class NullBackend:
    def __init__(self):
        self.lines = 0
        self.pings = []
        self.points = 0

    def set_lines(self, P0, P1, C, W):
        self.lines = len(P0)

    def set_pings(self, pings):
        self.pings = pings

    def add_points(self, P, C):
        self.points += len(P)

    def clear_points(self):
        self.points = 0

    def set_points_alpha(self, a):
        self.alpha = a

    def tick(self, t):
        pass


class UsdBackend:
    """USD prims for everything. Lines: one linear BasisCurves prim (2 verts per segment)."""

    def __init__(self, stage, root="/World/Overlay", point_cap=400000, flush_every=1500):
        from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt
        self.Gf, self.Sdf, self.UsdGeom, self.UsdShade, self.Vt = Gf, Sdf, UsdGeom, UsdShade, Vt
        self.stage, self.root = stage, root
        UsdGeom.Xform.Define(stage, root)
        self.curves = UsdGeom.BasisCurves.Define(stage, root + "/Lines")
        self.curves.CreateTypeAttr("linear")
        self.curves.CreateWrapAttr("nonperiodic")
        self.pts_prim = UsdGeom.Points.Define(stage, root + "/LidarPoints")
        self.pts_buf, self.col_buf = [], []
        self.n_pts, self.unflushed = 0, 0
        self.point_cap, self.flush_every = point_cap, flush_every
        self.ping_prims = {}
        self.live = set()
        self.pt_scale = 1.0                  # LiDAR fade: point widths shrink to 0, then the prim hides

    def _ribbons(self, P0, P1, C, W):
        """Flat quads along each segment (horizontal, width LINE_W * W m): thin painted-light lines."""
        UsdGeom, Vt = self.UsdGeom, self.Vt
        if not hasattr(self, "ribbon"):
            self.ribbon = UsdGeom.Mesh.Define(self.stage, self.root + "/LineRibbons")
            self.ribbon.CreateDoubleSidedAttr(True)
        n = len(P0)
        if n == 0:
            self.ribbon.CreateVisibilityAttr("invisible")
            return
        self.ribbon.CreateVisibilityAttr("inherited")
        P0, P1 = np.asarray(P0, np.float32), np.asarray(P1, np.float32)
        d = P1[:, :2] - P0[:, :2]
        L = np.maximum(np.hypot(d[:, 0], d[:, 1]), 1e-6)
        nrm = np.stack([-d[:, 1] / L, d[:, 0] / L, np.zeros(n)], 1).astype(np.float32)
        h = (np.asarray(W, np.float32) * LINE_W / 2)[:, None] * nrm
        pts = np.empty((4 * n, 3), np.float32)
        pts[0::4], pts[1::4], pts[2::4], pts[3::4] = P0 - h, P1 - h, P1 + h, P0 + h
        m = self.ribbon
        m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
        m.CreateFaceVertexCountsAttr(Vt.IntArray([4] * n))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.arange(4 * n, dtype=np.int32)))
        lo, hi = pts.min(0), pts.max(0)
        m.CreateExtentAttr([self.Gf.Vec3f(*map(float, lo)), self.Gf.Vec3f(*map(float, hi))])
        pv = UsdGeom.PrimvarsAPI(m)
        pv.CreatePrimvar("displayColor", self.Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.uniform).Set(
            Vt.Vec3fArray.FromNumpy(np.asarray(C, np.float32)[:, :3].copy()))

    def set_lines(self, P0, P1, C, W):
        UsdGeom, Vt = self.UsdGeom, self.Vt
        n = len(P0)
        if LINE_MESH:
            self.curves.CreateVisibilityAttr("invisible")
            return self._ribbons(P0, P1, C, W)
        if n == 0:
            self.curves.CreateVisibilityAttr("invisible")
            return
        self.curves.CreateVisibilityAttr("inherited")
        pts = np.empty((2 * n, 3), np.float32)
        pts[0::2], pts[1::2] = P0, P1
        self.curves.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
        self.curves.CreateCurveVertexCountsAttr(Vt.IntArray([2] * n))
        self.curves.CreateWidthsAttr(Vt.FloatArray.FromNumpy(np.repeat(np.asarray(W, np.float32) * LINE_W, 2)))
        self.curves.SetWidthsInterpolation(UsdGeom.Tokens.vertex)
        lo, hi = pts.min(0), pts.max(0)
        self.curves.CreateExtentAttr([self.Gf.Vec3f(*map(float, lo - 1)), self.Gf.Vec3f(*map(float, hi + 1))])
        pv = UsdGeom.PrimvarsAPI(self.curves)
        pv.CreatePrimvar("displayColor", self.Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.uniform).Set(
            Vt.Vec3fArray.FromNumpy(np.asarray(C, np.float32)[:, :3].copy()))
        pv.CreatePrimvar("displayOpacity", self.Sdf.ValueTypeNames.FloatArray, UsdGeom.Tokens.uniform).Set(
            Vt.FloatArray.FromNumpy(np.asarray(C, np.float32)[:, 3].copy()))

    def _ping_prims(self, pid):
        if pid in self.ping_prims:
            return self.ping_prims[pid]
        UsdGeom, Gf = self.UsdGeom, self.Gf
        safe = "".join(ch if ch.isalnum() else "_" for ch in pid)
        sph = UsdGeom.Sphere.Define(self.stage, f"{self.root}/Pings/{safe}")
        sph.CreateRadiusAttr(1.0)
        sxf = UsdGeom.Xformable(sph)
        st, ss = sxf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble), sxf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble)
        cyl = UsdGeom.Cylinder.Define(self.stage, f"{self.root}/Beacons/{safe}")
        cyl.CreateRadiusAttr(1.0)                    # radius = BEACON_R via the scale op (pulsed in tick)
        cyl.CreateHeightAttr(1.0)
        cyl.CreateAxisAttr("Z")
        cxf = UsdGeom.Xformable(cyl)
        ct, cs = cxf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble), cxf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble)
        rec = {"sph": sph, "st": st, "ss": ss, "cyl": cyl, "ct": ct, "cs": cs, "r": 0.35, "h": 1.0,
               "rgb": RED[:3], "emis": self._glow_material(safe, (sph, cyl)), "live": True}
        self.ping_prims[pid] = rec
        return rec

    def _glow_material(self, safe, gprims):
        """Emissive UsdPreviewSurface for a ping (sphere + beacon), so it glows in RTX wide shots.
        Returns the emissiveColor input (pulsed in tick) or None if materials fail (displayColor still works)."""
        UsdShade, Sdf, Gf = self.UsdShade, self.Sdf, self.Gf
        try:
            path = f"{self.root}/Looks/Ping_{safe}"
            mat = UsdShade.Material.Define(self.stage, path)
            sh = UsdShade.Shader.Define(self.stage, path + "/Shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*map(float, RED[:3])))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
            emis = sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f)
            emis.Set(Gf.Vec3f(*(BEACON_GLOW * float(c) for c in RED[:3])))
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            for g in gprims:
                try:
                    api = UsdShade.MaterialBindingAPI.Apply(g.GetPrim())
                except AttributeError:               # very old USD: non-applied API
                    api = UsdShade.MaterialBindingAPI(g.GetPrim())
                api.Bind(mat)
            return emis
        except Exception:
            return None

    def set_pings(self, pings):
        Gf = self.Gf
        seen = set()
        for p in pings:
            rec = self._ping_prims(p["id"])
            seen.add(p["id"])
            x, y, z = p["pos"]
            rec["st"].Set(Gf.Vec3d(x, y, z))
            rec["r"] = p["radius"]
            rec["ss"].Set(Gf.Vec3d(*(3 * [p["radius"]])))
            h = max(0.5, p["top"] - z)
            rec["h"], rec["rgb"] = h, tuple(p["rgba"][:3])
            rec["ct"].Set(Gf.Vec3d(x, y, z + h / 2))
            rec["cs"].Set(Gf.Vec3d(BEACON_R, BEACON_R, h))
            col = [Gf.Vec3f(*map(float, p["rgba"][:3]))]
            rec["sph"].CreateDisplayColorAttr(col)
            rec["cyl"].CreateDisplayColorAttr(col)
            for g in (rec["sph"], rec["cyl"]):
                g.CreateVisibilityAttr("inherited")
        for pid, rec in self.ping_prims.items():
            if pid not in seen:
                rec["sph"].CreateVisibilityAttr("invisible")
                rec["cyl"].CreateVisibilityAttr("invisible")
        self.live = seen

    def clear_points(self):
        self.pts_buf, self.col_buf = [], []
        self.n_pts, self.unflushed, self.pt_scale = 0, 0, 1.0
        self.pts_prim.CreateVisibilityAttr("invisible")

    def set_points_alpha(self, a):
        """LiDAR fade: 1 = full map, 0 = hidden. Widths shrink (displayOpacity needs a material in RTX)."""
        self.pt_scale = float(a)
        if a <= 0:
            self.pts_prim.CreateVisibilityAttr("invisible")
            return
        self.flush_points()

    def add_points(self, P, C):
        if self.n_pts >= self.point_cap:
            return
        self.pts_buf.append(P)
        self.col_buf.append(C[:, :3])
        self.n_pts += len(P)
        self.unflushed += len(P)
        if self.unflushed >= self.flush_every:
            self.flush_points()

    def flush_points(self):
        if not self.pts_buf:
            return
        Vt = self.Vt
        P = np.concatenate(self.pts_buf).astype(np.float32)
        C = np.concatenate(self.col_buf).astype(np.float32)
        self.pts_buf, self.col_buf = [P], [C]
        self.pts_prim.CreateVisibilityAttr("inherited")
        self.pts_prim.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(P))
        self.pts_prim.CreateWidthsAttr(Vt.FloatArray.FromNumpy(np.full(len(P), POINT_W * self.pt_scale, np.float32)))
        self.pts_prim.SetWidthsInterpolation(self.UsdGeom.Tokens.vertex)
        lo, hi = P.min(0), P.max(0)
        self.pts_prim.CreateExtentAttr([self.Gf.Vec3f(*map(float, lo)), self.Gf.Vec3f(*map(float, hi))])
        pv = self.UsdGeom.PrimvarsAPI(self.pts_prim)
        pv.CreatePrimvar("displayColor", self.Sdf.ValueTypeNames.Color3fArray, self.UsdGeom.Tokens.vertex).Set(
            Vt.Vec3fArray.FromNumpy(C))
        self.unflushed = 0

    def tick(self, t):
        w = 0.5 + 0.5 * math.sin(2 * math.pi * PULSE_HZ * t)   # slow pulse, 0..1
        s = 1.0 + 0.35 * w                                       # ping sphere
        b = BEACON_R * (0.8 + 0.5 * w)                           # beacon column width
        g = BEACON_GLOW * (0.5 + 0.5 * w)                        # emissive gain
        Gf = self.Gf
        for pid, rec in self.ping_prims.items():
            if pid in self.live:
                r = rec["r"] * s
                rec["ss"].Set(Gf.Vec3d(r, r, r))
                rec["cs"].Set(Gf.Vec3d(b, b, rec["h"]))
                if rec["emis"] is not None:
                    rec["emis"].Set(Gf.Vec3f(*(g * float(c) for c in rec["rgb"])))


class DebugDrawBackend(UsdBackend):
    """Lines and points through debug_draw (cheap, viewport overlay); pings stay USD prims.
    Signatures verified on 6.0.1: draw_lines(starts, ends, rgba, widths), draw_points(points, rgba, sizes),
    clear_lines(), clear_points(); lists of tuples. The team's Isaac Lab Camera recordings appear to include
    the lines (media/preview/isaac_t3s_light_fixed.png); Movie Capture is untested, so --overlay usd for that."""

    def __init__(self, stage, dd, point_cap=600000):
        super().__init__(stage)
        self.dd = dd
        self.dd_points = 0
        self.dd_cap = point_cap

    def set_lines(self, P0, P1, C, W):
        self.dd.clear_lines()
        if len(P0):
            self.dd.draw_lines([tuple(map(float, p)) for p in P0], [tuple(map(float, p)) for p in P1],
                               [tuple(map(float, c)) for c in C], [float(w) for w in W])

    def add_points(self, P, C):
        if self.dd_points >= self.dd_cap:
            return
        self.dd.draw_points([tuple(map(float, p)) for p in P], [tuple(map(float, c)) for c in C], [4.0] * len(P))
        self.dd_points += len(P)

    def clear_points(self):
        self.dd.clear_points()
        self.dd_points = 0

    def set_points_alpha(self, a):
        if a <= 0:                                   # debug_draw points can't be dimmed: hide at the end of the fade
            self.clear_points()


def make_backend(kind, stage=None, dd=None):
    """kind: auto | debug | usd | null."""
    if kind == "null" or stage is None:
        return NullBackend()
    if kind in ("auto", "debug") and dd is not None:
        return DebugDrawBackend(stage, dd)
    return UsdBackend(stage)
