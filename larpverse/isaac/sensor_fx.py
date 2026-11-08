"""Live sensor effects for the judges' single take: every sensor visibly doing its job, next to the action.

Display only. Nothing here feeds the mission: the planner never reads these prims or the hidden truth. All
prims are emissive USD meshes under /World/SensorFX (they render in the live RTX stream, path tracing and
Movie Capture alike) and carry no colliders, so LiDAR, landing and replays are unchanged.

  Dummies    grey mannequins where the scenario hid its (invented) trapped people, half sunk into the rubble.
             Drawn only with --show-truth; they stand for "simulated people", never real ones.
  Sound      teal wireframe domes swelling out of each sounding dummy while the drones listen with rotors off
             (mission stage "listen"); grey for non-human sounds (dog, settling debris).
  LiDAR      purple beams from each surveying drone's LiDAR puck to its newest returns (the animation's fan).
  Radar      orange pulses spreading over the rubble from each pod (or landed drone) while its radar dwells,
             out to its ~3 m reach.
  Hotspot    the radar localiser's belief (state["belief"]) as a glowing amber-to-red patch on the rubble.

Colours follow the animation/dashboard: LiDAR #9b6bff, radar #ff9f43, acoustic #22c3d6, detection #ff4545.
"""
from __future__ import annotations

import math

import numpy as np

ROOT = "/World/SensorFX"
LIDAR = (0.61, 0.42, 1.0)
RADAR = (1.0, 0.36, 0.04)        # deep orange: take2's (1, .62, .26) at glow 6 rendered gold, next to yellow cones
ACOUSTIC = (0.13, 0.76, 0.84)
NOISE = (0.55, 0.60, 0.65)
HEAT = ((1.0, 0.68, 0.10), (1.0, 0.38, 0.08), (1.0, 0.10, 0.06))   # belief bins: amber, orange, red
DUMMY_RGB = (0.50, 0.51, 0.53)
HUMAN_SOUNDS = {"voice", "weak_voice", "shout", "knock", "knocking", "tapping"}

GLOW = 6.0                      # emissive gain (RTX nits-ish scale used by the overlay's beacons)
BEAMS_PER_DRONE = 26
BEAM_W = 0.022                  # m
SOUND_W, NOISE_W = 0.05, 0.035  # m: dome wire widths (human-like, other sounds)
RADAR_W = 0.07                  # m: radar pulse ring width at the start of a wave
SOUND_R = 8.0                   # m: dome radius at the end of a wave (take2: 14 m domes swamped the frame)
SOUND_PERIOD = 2.2              # s (wall clock): one wave
SOUND_WAVES = 2
SIMPLE = False                  # --overlay-style simple: no grey non-human domes, fewer LiDAR beams
BEAM_MAX = 28.0                 # m: LiDAR beams drawn at most this long (take2: far returns streaked to the frame edge)
SIMPLE_BEAMS, SIMPLE_BEAM_MAX = 8, 18.0   # simple style: a short downward fan per drone; longer (shallow) rays are
                                          # dropped, not cut (reh5b 10 s: rays to far returns read as rain / wires)
SIMPLE_BEAM_MIN_H = 0.3        # simple style: keep only rays that land on rubble (public heightmap above the street), so
                                # rays cast while the drones fly between piles don't streak across a low camera like rain
RADAR_R = 3.5                   # m: pulse ring radius at the end (the literature reach ~3 m)
RADAR_PERIOD = 1.3
RADAR_WAVES = 3
BELIEF_MIN = 2.5                # cells this many times the uniform belief and above are drawn
DUMMY_SINK = 0.12               # m: torso centre this far below the local rubble surface


# ------------------------------------------------------------------------------------------- geometry
def _perp(d):
    d = d / max(np.linalg.norm(d), 1e-9)
    a = np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    n1 = np.cross(d, a)
    n1 /= max(np.linalg.norm(n1), 1e-9)
    return n1, np.cross(d, n1)


def cross_ribbons(P0, P1, w):
    """Two crossed flat quads per segment (visible from any side): (points (8n, 3), counts, indices)."""
    P0, P1 = np.asarray(P0, np.float32).reshape(-1, 3), np.asarray(P1, np.float32).reshape(-1, 3)
    n = len(P0)
    if n == 0:
        return np.zeros((0, 3), np.float32), [], []
    w = np.broadcast_to(np.asarray(w, np.float32), (n,))[:, None] / 2
    d = P1 - P0
    L = np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-6)
    d = d / L
    ref = np.where(np.abs(d[:, 2:3]) < 0.9, [[0.0, 0.0, 1.0]], [[1.0, 0.0, 0.0]]).astype(np.float32)
    n1 = np.cross(d, ref)
    n1 /= np.maximum(np.linalg.norm(n1, axis=1, keepdims=True), 1e-9)
    n2 = np.cross(d, n1)
    pts = np.empty((8 * n, 3), np.float32)
    for k, nn in enumerate((n1, n2)):
        h = nn * w
        pts[8 * np.arange(n) + 4 * k + 0] = P0 - h
        pts[8 * np.arange(n) + 4 * k + 1] = P1 - h
        pts[8 * np.arange(n) + 4 * k + 2] = P1 + h
        pts[8 * np.arange(n) + 4 * k + 3] = P0 + h
    return pts, [4] * (2 * n), np.arange(8 * n, dtype=np.int32)


def polyline_segs(poly):
    poly = np.asarray(poly, float)
    return poly[:-1], poly[1:]


def dome_segments(c, r, rings=(0.0, 0.55), meridians=4, n=36):
    """Wireframe hemisphere of radius r at c: latitude rings (elevation in rad) + meridian arcs."""
    c = np.asarray(c, float)
    P0, P1 = [], []
    a = np.linspace(0, 2 * np.pi, n + 1)
    for el in rings:
        rr, z = r * math.cos(el), r * math.sin(el)
        ring = np.stack([c[0] + rr * np.cos(a), c[1] + rr * np.sin(a), np.full_like(a, c[2] + z)], 1)
        p0, p1 = polyline_segs(ring)
        P0.append(p0); P1.append(p1)
    e = np.linspace(0, 1.35, 7)
    for k in range(meridians):
        th = 2 * np.pi * k / meridians
        arc = np.stack([c[0] + r * np.cos(e) * math.cos(th), c[1] + r * np.cos(e) * math.sin(th),
                        c[2] + r * np.sin(e)], 1)
        p0, p1 = polyline_segs(arc)
        P0.append(p0); P1.append(p1)
    return np.concatenate(P0), np.concatenate(P1)


def mannequin(rng):
    """Grey crash-test-style dummy lying on its back along +x (feet at -x), centred at the origin: (P, F)."""
    from isaac.drone_mesh import merge, sphere, superellipsoid, tube
    parts = [superellipsoid((0.18, 0, 0), 0.26, 0.17, 0.11, 0.5, 0.5, nu=20, nv=10),     # chest
             superellipsoid((-0.14, 0, 0), 0.13, 0.155, 0.10, 0.5, 0.5, nu=18, nv=8),    # pelvis
             tube((0.44, 0, 0), (0.52, 0, 0.01), 0.05, sides=10),                          # neck
             sphere((0.64, 0, 0.03), 0.105, n=10)]                                         # head
    for s in (-1, 1):
        sh = np.array([0.36, s * 0.2, 0.0])
        el = sh + [-0.25, s * rng.uniform(0.04, 0.16), rng.uniform(-0.02, 0.06)]
        wr = el + [-0.24 + rng.uniform(-0.05, 0.12), s * rng.uniform(-0.02, 0.1), rng.uniform(-0.02, 0.05)]
        parts += [tube(sh, el, 0.048, sides=10), tube(el, wr, 0.04, sides=10), sphere(wr, 0.045, n=6)]
        hp = np.array([-0.22, s * 0.09, 0.0])
        kn = hp + [-0.42, s * rng.uniform(0.0, 0.08), rng.uniform(0.0, 0.08)]
        an = kn + [-0.4, s * rng.uniform(0.0, 0.05), rng.uniform(-0.06, 0.0)]
        parts += [tube(hp, kn, 0.07, sides=10), tube(kn, an, 0.055, sides=10),
                  superellipsoid(an + [-0.02, 0, 0.05], 0.05, 0.045, 0.08, 0.4, 0.4, nu=10, nv=6)]
    return merge(parts)


# ------------------------------------------------------------------------------------------- USD side
class _Dyn:
    """One emissive mesh prim rebuilt per frame (empty -> invisible)."""

    def __init__(self, stage, path, rgb, gain=GLOW):
        from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt
        self.Gf, self.Vt, self.UsdGeom = Gf, Vt, UsdGeom
        self.mesh = UsdGeom.Mesh.Define(stage, path)
        self.mesh.CreateDoubleSidedAttr(True)
        self.mesh.CreateSubdivisionSchemeAttr("none")
        self.mesh.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
        self.mesh.CreateVisibilityAttr("invisible")
        mat = UsdShade.Material.Define(stage, path + "_Look")
        sh = UsdShade.Shader.Define(stage, path + "_Look/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*(0.25 * c for c in rgb)))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
        self.emis = sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f)
        self.rgb, self.gain = rgb, gain
        self.emis.Set(Gf.Vec3f(*(gain * c for c in rgb)))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(self.mesh.GetPrim()).Bind(mat)
        self.shown = False

    def set(self, pts, counts, idx):
        if len(pts) == 0:
            if self.shown:
                self.mesh.GetVisibilityAttr().Set("invisible")
                self.shown = False
            return
        Vt, Gf = self.Vt, self.Gf
        self.mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(pts, np.float32))) \
            if self.mesh.GetPointsAttr() else self.mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
        self.mesh.CreateFaceVertexCountsAttr(Vt.IntArray(list(counts)))
        self.mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.asarray(idx, np.int32)))
        lo, hi = pts.min(0), pts.max(0)
        self.mesh.CreateExtentAttr([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))])
        if not self.shown:
            self.mesh.GetVisibilityAttr().Set("inherited")
            self.shown = True

    def glow(self, k):
        self.emis.Set(self.Gf.Vec3f(*(self.gain * k * c for c in self.rgb)))


class NullFX:
    """Dry-run stand-in with the same calls; counts what would be drawn (tests)."""

    def __init__(self, *_, **__):
        self.counts = {"beams": 0, "sound": 0, "radar": 0, "heat": 0}
        self.dummies = 0
        self._beams = {}

    def lidar(self, did, origin, pts):
        if pts is not None and len(pts):
            self._beams[did] = (origin, pts)

    def frame(self, now, state, poses):
        st = analyse(state, poses)
        self.counts["beams"] = max(self.counts["beams"], sum(min(len(p), BEAMS_PER_DRONE) for _, p in self._beams.values()))
        self.counts["radar"] = max(self.counts["radar"], len(st["radar"]))
        self.counts["sound"] = max(self.counts["sound"], int(st["listening"]))
        self._beams = {}
        return st


def set_scale(k, log=print):
    """Thicker effects for wide shots (--line-scale): widths x k, radar ring reach unchanged (it means ~3 m)."""
    global BEAM_W, SOUND_W, NOISE_W, RADAR_W
    k = float(k)
    if abs(k - 1.0) < 1e-6:
        return
    ks = k ** 0.5                                      # domes are big already: thicker wire only by sqrt(k)
    BEAM_W, SOUND_W, NOISE_W, RADAR_W = BEAM_W * k, SOUND_W * ks, NOISE_W * ks, RADAR_W * k
    log(f"[fx] widths x{k:g}: LiDAR beam {BEAM_W * 100:.0f} cm, sound wire {SOUND_W * 100:.0f} cm, radar ring {RADAR_W * 100:.0f} cm")


def analyse(state, poses):
    """What to draw this frame, from the mission state alone: {listening, radar: [(x, y, z)], belief}."""
    m = state.get("mission") if isinstance(state.get("mission"), dict) else {}
    listening = m.get("stage") == "listen" and m.get("listen_ends") is not None
    radar = []
    pods = [p for p in state.get("pods") or [] if isinstance(p, dict)]
    have = set()
    for p in pods:
        if str(p.get("status")) == "scanning" and p.get("xy") is not None:
            radar.append((float(p["xy"][0]), float(p["xy"][1]), float(p.get("z") or 0.0)))
            have.add(p.get("drone_id"))
    for d in state.get("drones") or []:
        if d.get("id") not in have and str(d.get("state")) == "scanning":
            x, y, z = (poses.get(d["id"]) or d["pose"])[:3]
            radar.append((float(x), float(y), float(z) - 0.3))
    bel = state.get("belief")
    if SIMPLE and not any(c.get("status") in ("insufficient_evidence", "candidate")
                          for c in state.get("radar_candidates") or []):
        bel = None                  # simple style: no belief heat until the radar actually has a lead
    return {"listening": listening, "radar": radar, "belief": bel}


class SensorFX(NullFX):
    def __init__(self, stage, sc, surface, show_truth=False, log=print, seed=7):
        super().__init__()
        from pxr import UsdGeom
        self.stage, self.surface, self.log = stage, surface, log
        UsdGeom.Xform.Define(stage, ROOT)
        self.beam_mesh = _Dyn(stage, f"{ROOT}/LidarBeams", LIDAR, GLOW * 1.2)
        self.sound_mesh = _Dyn(stage, f"{ROOT}/SoundWaves", ACOUSTIC, GLOW)
        self.noise_mesh = _Dyn(stage, f"{ROOT}/NoiseWaves", NOISE, GLOW * 0.5)
        self.radar_mesh = _Dyn(stage, f"{ROOT}/RadarPulses", RADAR, GLOW)
        self.heat = [_Dyn(stage, f"{ROOT}/Hotspot{k}", rgb, GLOW * (0.6 + 0.4 * k)) for k, rgb in enumerate(HEAT)]
        self._belief_key = None
        truth = getattr(sc, "truth", None) or {}
        self.sounds = []
        for s in truth.get("sounds") or []:
            x, y = map(float, s["xy"][:2])
            self.sounds.append((x, y, self._z(x, y), str(s.get("kind")) in HUMAN_SOUNDS, float(s.get("level_db", -6.0))))
        if show_truth:
            self._dummies(truth.get("victims") or [], seed)
            gt = stage.GetPrimAtPath("/World/GroundTruth")       # the old white capsules, now replaced
            if gt and gt.IsValid():
                UsdGeom.Imageable(gt).MakeInvisible()
        log(f"[fx] sensor effects: LiDAR beams, radar pulses, {len(self.sounds)} sound sources "
            f"({sum(1 for s in self.sounds if s[3])} human), belief hotspot"
            + (f", {self.dummies} grey dummies (simulated people)" if show_truth else ""))

    def _z(self, x, y):
        return float(np.asarray(self.surface(np.array([x]), np.array([y]))).ravel()[0])

    def _dummies(self, victims, seed):
        from pxr import Gf, UsdGeom
        from isaac import photoreal as PR
        from isaac.urban_look import _pv
        look = _pv(self.stage, f"{ROOT}/Looks/Dummy", DUMMY_RGB, 0.55, 0.0, 0.2)
        rng = np.random.default_rng(seed + 404)
        for v in victims:
            x, y = map(float, v["xy"][:2])
            P, F = mannequin(rng)
            path = f"{ROOT}/Dummies/{v['id']}"
            xf = UsdGeom.Xformable(UsdGeom.Xform.Define(self.stage, path))
            xf.ClearXformOpOrder()
            # local slope: lie along the rubble (heights 0.8 m either side along the body axis)
            yaw = float(rng.uniform(0, 360))
            ux, uy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            zh, zf = self._z(x + 0.8 * ux, y + 0.8 * uy), self._z(x - 0.8 * ux, y - 0.8 * uy)
            pitch = -math.degrees(math.atan2(zh - zf, 1.6))
            sink = DUMMY_SINK + (0.06 if v.get("depth_class") == "deep" else 0.0)
            xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(x, y, max(zh, zf, self._z(x, y)) - sink))
            xf.AddRotateZOp(UsdGeom.XformOp.PrecisionDouble).Set(yaw)
            xf.AddRotateYOp(UsdGeom.XformOp.PrecisionDouble).Set(float(np.clip(pitch, -35, 35)))
            xf.AddRotateXOp(UsdGeom.XformOp.PrecisionDouble).Set(float(rng.uniform(-25, 25)))   # rolled a little
            m = PR._mesh(self.stage, f"{path}/Body", P, F, DUMMY_RGB, smooth=True)
            m.CreateDoubleSidedAttr(True)
            PR.bind(self.stage, f"{path}/Body", look)
            self.dummies += 1

    # ---------------------------------------------------------------------------------------- per frame
    def frame(self, now, state, poses):
        st = analyse(state, poses)
        # LiDAR beams (this frame's returns only)
        P0, P1 = [], []
        for did, (o, pts) in self._beams.items():
            pts = np.asarray(pts, float)[:, :3]
            o = np.asarray(o, float)
            if SIMPLE and len(pts):
                if len(pts) > 256:      # bound the per-frame heightmap lookups
                    pts = pts[np.linspace(0, len(pts) - 1, 256).astype(int)]
                keep = np.linalg.norm(pts - o[None], axis=1) <= SIMPLE_BEAM_MAX
                try:
                    keep &= np.asarray(self.surface(pts[:, 0], pts[:, 1]), float).ravel() > SIMPLE_BEAM_MIN_H
                except Exception:   # noqa: BLE001  (a surface that can't take arrays: keep the length rule only)
                    pass
                pts = pts[keep]
                if not len(pts):
                    continue
            nb = SIMPLE_BEAMS if SIMPLE else BEAMS_PER_DRONE
            if len(pts) > nb:
                pts = pts[np.linspace(0, len(pts) - 1, nb).astype(int)]
            d = pts - o[None]
            L = np.linalg.norm(d, axis=1, keepdims=True)
            pts = o[None] + d * np.minimum(1.0, BEAM_MAX / np.maximum(L, 1e-6))
            P0.append(np.repeat(o[None], len(pts), 0))
            P1.append(pts)
        self.beam_mesh.set(*cross_ribbons(np.concatenate(P0), np.concatenate(P1), BEAM_W) if P0 else ([], [], []))
        self.counts["beams"] = max(self.counts["beams"], sum(len(p) for p in P1))
        self._beams = {}
        # sound domes while the fleet listens (rotors off): teal = human-like, grey = other sounds
        segs = {True: ([], []), False: ([], [])}
        if st["listening"] and self.sounds:
            for k, (x, y, z, human, db) in enumerate(self.sounds):
                if SIMPLE and not human:
                    continue
                rmax = SOUND_R * (0.6 + 0.4 * float(np.clip(1 + db / 12.0, 0, 1)))
                for w in range(SOUND_WAVES):
                    f = ((now / SOUND_PERIOD) + w / SOUND_WAVES + 0.37 * k) % 1.0
                    a, b = dome_segments((x, y, z), 0.4 + f * rmax)
                    segs[human][0].append(a)
                    segs[human][1].append(b)
            self.counts["sound"] += 1
        for mesh, human, w0 in ((self.sound_mesh, True, SOUND_W), (self.noise_mesh, False, NOISE_W)):
            A, B = segs[human]
            mesh.set(*cross_ribbons(np.concatenate(A), np.concatenate(B), w0) if A else ([], [], []))
        # radar pulses over the rubble around each scanning pod
        A, B, W = [], [], []
        ang = np.linspace(0, 2 * np.pi, 41)
        for k, (x, y, z) in enumerate(st["radar"]):
            for w in range(RADAR_WAVES):
                f = ((now / RADAR_PERIOD) + w / RADAR_WAVES + 0.29 * k) % 1.0
                r = 0.3 + f * RADAR_R
                xs, ys = x + r * np.cos(ang), y + r * np.sin(ang)
                zs = np.asarray(self.surface(xs, ys), float).ravel() + 0.08
                ring = np.stack([xs, ys, np.maximum(zs, z - 0.5)], 1)
                a, b = polyline_segs(ring)
                A.append(a); B.append(b); W.append(np.full(len(a), RADAR_W * (1.0 - 0.7 * f)))
        self.radar_mesh.set(*cross_ribbons(np.concatenate(A), np.concatenate(B), np.concatenate(W)) if A else ([], [], []))
        self.counts["radar"] = max(self.counts["radar"], len(st["radar"]))
        # belief hotspot (rebuilt only when the belief changes)
        self._hotspot(st["belief"])
        return st

    def _hotspot(self, bel):
        key = None if not isinstance(bel, dict) else (bel.get("pile_id"), tuple(bel.get("origin") or ()), len(bel.get("p") or ()),
                                                       round(float(np.sum(np.asarray(bel.get("p") or [0.0])[::97])), 6))
        if key == self._belief_key:
            return
        self._belief_key = key
        if key is None or not bel.get("p"):
            for h in self.heat:
                h.set([], [], [])
            return
        w, hgt, res = int(bel["w"]), int(bel["h"]), float(bel["res"])
        p = np.asarray(bel["p"], float).reshape(hgt, w)
        u = 1.0 / max(np.count_nonzero(p > 0), 1)
        lvl = p / u
        x0, y0 = map(float, bel["origin"][:2])
        for k, h in enumerate(self.heat):
            if SIMPLE and k < len(self.heat) - 1:
                h.set([], [], [])       # simple style: only the strongest bins (the "dig here" patch)
                continue
            lo = BELIEF_MIN * (3 ** k)
            hi = BELIEF_MIN * (3 ** (k + 1)) if k < len(self.heat) - 1 else np.inf
            jj, ii = np.nonzero((lvl >= lo) & (lvl < hi))
            if len(ii) == 0:
                h.set([], [], [])
                continue
            cx, cy = x0 + (ii + 0.5) * res, y0 + (jj + 0.5) * res
            q = res * 0.48
            xs = np.stack([cx - q, cx + q, cx + q, cx - q], 1).ravel()
            ys = np.stack([cy - q, cy - q, cy + q, cy + q], 1).ravel()
            zs = np.asarray(self.surface(xs, ys), float).ravel() + 0.06 + 0.02 * k
            pts = np.stack([xs, ys, zs], 1).astype(np.float32)
            h.set(pts, [4] * len(ii), np.arange(4 * len(ii), dtype=np.int32))
