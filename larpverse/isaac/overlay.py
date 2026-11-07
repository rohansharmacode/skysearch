"""Mission overlay driven ONLY by the state dict (contracts/INTERFACES.md v1 + v1.1 grid extensions).

Geometry is computed here in numpy; a backend draws it:
  * ``DebugDrawBackend``  isaacsim.util.debug_draw lines/points (+ USD prims for the ping spheres/beacons)
  * ``UsdBackend``        USD prims only (BasisCurves, Points, Spheres) - shows up in every renderer/capture
  * ``NullBackend``       records counts; used by ``--dry-run`` (no omni/pxr)

What is drawn: red ping spheres at (x, y, z_est) with a tall thin red beacon and a 2-sigma surface ring per
radar candidate; amber range rings for the active pile's scans; yellow cones for human-classified acoustic
clips; a green dispatch arc; LiDAR points coloured by height (incremental).
"""
from __future__ import annotations

import math

import numpy as np

RED = (0.95, 0.12, 0.10, 1.0)
RED_DIM = (0.95, 0.45, 0.25, 0.7)
AMBER = (1.0, 0.68, 0.10, 0.9)
YELLOW = (1.0, 0.92, 0.15, 0.95)
GREEN = (0.20, 0.95, 0.35, 1.0)
HUMAN = {"voice", "weak_voice", "shout", "knock", "knocking", "tapping", "human"}
# Drawing scale (metres per width unit for lines, point diameter, how many recent scans get range rings).
# isaac/photoreal.py lowers these for footage: thick tubes and big dots read as props in a realistic render.
LINE_W = 0.03
POINT_W = 0.09
MAX_SCANS = 12
ACTIVE = {"active", "survey", "surveying", "scanning", "searching", "in_progress", "radar", "landing",
          "relocate", "rescan", "acoustic"}


def height_colors(z, lo=-0.5, hi=5.0):
    """Blue -> teal -> yellow -> red ramp, RGBA float32."""
    t = np.clip((np.asarray(z, float) - lo) / (hi - lo), 0, 1)
    stops = np.array([[0.15, 0.30, 0.95], [0.10, 0.80, 0.80], [0.95, 0.90, 0.20], [0.95, 0.25, 0.15]])
    f = t * (len(stops) - 1)
    i = np.minimum(f.astype(int), len(stops) - 2)
    u = (f - i)[:, None]
    rgb = stops[i] * (1 - u) + stops[i + 1] * u
    return np.concatenate([rgb, np.ones((len(rgb), 1))], 1).astype(np.float32)


class Overlay:
    def __init__(self, backend, surface_fn, pile_xy=None, base_xy=(0.0, -47.0)):
        self.b = backend
        self.surface = surface_fn            # (x, y) arrays -> z; public geometry (state heightmap or scenario)
        self.pile_xy = dict(pile_xy or {})
        self.base_xy = base_xy
        self.n_lines = 0
        self.n_pings = 0
        self.n_points = 0
        self.updates = 0

    # ------------------------------------------------------------------ helpers
    def _drape(self, xy, lift=0.15, max_seg=0.8):
        """Densify an (N, 2) polyline and lay it on the surface. Returns (M, 3)."""
        xy = np.asarray(xy, float)
        out = [xy[:1]]
        for a, b in zip(xy[:-1], xy[1:]):
            n = max(1, int(math.ceil(np.hypot(*(b - a)) / max_seg)))
            out.append(a + (b - a) * (np.arange(1, n + 1) / n)[:, None])
        p = np.concatenate(out)
        return np.column_stack([p, np.asarray(self.surface(p[:, 0], p[:, 1]), float) + lift])

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

        # radar candidates: ping sphere + beacon + 2-sigma surface ring
        for c in state.get("radar_candidates") or []:
            x, y = map(float, c["xy"][:2])
            zs = float(self.surface(np.array([x]), np.array([y]))[0])
            ze = float(c.get("z_est", zs - 1.2) if c.get("z_est") is not None else zs - 1.2)
            weak = c.get("status") == "insufficient_evidence"
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
            self._segs(self._drape(ell, 0.12), col, 3.0, segs)

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
        for s in scans[-MAX_SCANS:]:
            sx, sy = s["sensor_pose"][:2]
            confs = s.get("confidence") or []
            wgt = float(s.get("used_weight", 1.0) if s.get("used_weight") is not None else 1.0)
            for i, r in enumerate(s.get("range_candidates") or []):
                cf = float(confs[i]) if i < len(confs) else 0.5
                rgba = (*AMBER[:3], 0.25 + 0.7 * cf * max(wgt, 0.2))
                a = np.linspace(0, 2 * np.pi, max(32, int(r * 6)) + 1)
                ring = np.stack([sx + r * np.cos(a), sy + r * np.sin(a)], 1)
                self._segs(self._drape(ring, 0.2), rgba, 2.0 if cf < 0.5 else 3.0, segs)

        # yellow cones for human-classified acoustic clips
        clips = state.get("acoustic_clips")
        if clips is None:
            clips = state.get("acoustic") or []
        for c in clips:
            human = c.get("human")            # explicit flag wins; else the classifier's label
            if human is None:
                human = c.get("class") in HUMAN or float((c.get("scores") or {}).get("human", 0)) >= 0.5
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
            self._segs(self._drape(cone, 0.35), YELLOW, 3.0, segs)
            mid = np.array([[ox, oy], [ox + L * np.cos(b), oy + L * np.sin(b)]])
            self._segs(self._drape(mid, 0.35), (*YELLOW[:3], 0.45), 1.5, segs)

        # green dispatch arc(s): base -> target
        for d in state.get("dispatch") or []:
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
        self.updates += 1
        return active

    def add_points(self, pts):
        if pts is None or len(pts) == 0:
            return
        self.b.add_points(np.asarray(pts, np.float32), height_colors(pts[:, 2]))
        self.n_points += len(pts)

    def tick(self, t):
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

    def tick(self, t):
        pass


class UsdBackend:
    """USD prims for everything. Lines: one linear BasisCurves prim (2 verts per segment)."""

    def __init__(self, stage, root="/World/Overlay", point_cap=400000, flush_every=1500):
        from pxr import Gf, Sdf, UsdGeom, Vt
        self.Gf, self.Sdf, self.UsdGeom, self.Vt = Gf, Sdf, UsdGeom, Vt
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

    def set_lines(self, P0, P1, C, W):
        UsdGeom, Vt = self.UsdGeom, self.Vt
        n = len(P0)
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
        cyl.CreateRadiusAttr(0.06)
        cyl.CreateHeightAttr(1.0)
        cyl.CreateAxisAttr("Z")
        cxf = UsdGeom.Xformable(cyl)
        ct, cs = cxf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble), cxf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble)
        rec = {"sph": sph, "st": st, "ss": ss, "cyl": cyl, "ct": ct, "cs": cs, "r": 0.35, "live": True}
        self.ping_prims[pid] = rec
        return rec

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
            rec["ct"].Set(Gf.Vec3d(x, y, z + h / 2))
            rec["cs"].Set(Gf.Vec3d(1, 1, h))
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
        self.pts_prim.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(P))
        self.pts_prim.CreateWidthsAttr(Vt.FloatArray.FromNumpy(np.full(len(P), POINT_W, np.float32)))
        self.pts_prim.SetWidthsInterpolation(self.UsdGeom.Tokens.vertex)
        lo, hi = P.min(0), P.max(0)
        self.pts_prim.CreateExtentAttr([self.Gf.Vec3f(*map(float, lo)), self.Gf.Vec3f(*map(float, hi))])
        pv = self.UsdGeom.PrimvarsAPI(self.pts_prim)
        pv.CreatePrimvar("displayColor", self.Sdf.ValueTypeNames.Color3fArray, self.UsdGeom.Tokens.vertex).Set(
            Vt.Vec3fArray.FromNumpy(C))
        self.unflushed = 0

    def tick(self, t):
        s = 1.0 + 0.35 * math.sin(t * 4.0)          # pulse the ping spheres
        for pid, rec in self.ping_prims.items():
            if pid in self.live:
                r = rec["r"] * s
                rec["ss"].Set(self.Gf.Vec3d(r, r, r))


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


def make_backend(kind, stage=None, dd=None):
    """kind: auto | debug | usd | null."""
    if kind == "null" or stage is None:
        return NullBackend()
    if kind in ("auto", "debug") and dd is not None:
        return DebugDrawBackend(stage, dd)
    return UsdBackend(stage)
