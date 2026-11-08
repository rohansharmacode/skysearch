"""A realistic heavy-lift SAR drone built from smooth meshes (pure numpy, no pxr): the same platform as
``sim.platform`` "heavy_sar" (coaxial X8, 2.2 m wheelbase, 54-inch props, gear footprint 1.6 x 1.4 m), drawn in
the style of current enterprise cargo/agri drones: a rounded two-tone fuselage with a dark nose visor, battery
packs and a GNSS mast on top, four carbon arms with fold joints, coaxial motor pods with tapered two-blade
props, a two-leg landing gear on skids, and the payload underneath (ball gimbal with thermal/zoom camera,
LiDAR puck, radar panel, speaker) plus the 8-microphone array on the arms. Generic design, no branding.

``dji_parts(spec, did)`` returns the same part tuples as ``drone_viz.heavy_parts`` with two extra kinds:
("Mesh", name, (0,0,0), (1,1,1), rgb, None, {"P": (N,3), "F": (M,3)}) and rotors with {"style": "mesh"}.
Metres, +x forward, skid bottom at z = 0.
"""
from __future__ import annotations

import math

import numpy as np

GEAR_H = 0.60
POD_MODE = False        # --delivery pod: the radar rides in the winch pod, so no radar panel under the drone
NAV_R = 0.035           # m: navigation light lens radius (red port, green starboard, white strobe on top)


def _grid_faces(rows, cols, wrap=True):
    F = []
    cc = cols if wrap else cols - 1
    for i in range(rows - 1):
        for j in range(cc):
            a, b = i * cols + j, i * cols + (j + 1) % cols
            c, d = a + cols, b + cols
            F += [(a, b, d), (a, d, c)]
    return np.array(F, np.int32)


def superellipsoid(c, a, b, h, e1=0.35, e2=0.3, nu=40, nv=18):
    """Rounded box (half sizes a, b, h) centred at c; e -> 0 is boxy, 1 is an ellipsoid."""
    f = lambda w, e: np.sign(w) * np.abs(w) ** e
    v = np.linspace(-math.pi / 2, math.pi / 2, nv + 1)
    u = np.linspace(-math.pi, math.pi, nu, endpoint=False)
    V, U = np.meshgrid(v, u, indexing="ij")
    P = np.stack([a * f(np.cos(V), e1) * f(np.cos(U), e2), b * f(np.cos(V), e1) * f(np.sin(U), e2),
                  h * f(np.sin(V), e1)], -1).reshape(-1, 3)
    F = _grid_faces(nv + 1, nu)
    return (P + np.asarray(c, float)).astype(np.float32), F


def tube(p0, p1, r, sides=14, caps=True, r1=None):
    """Capped cylinder (or cone frustum with r1) from p0 to p1."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    ax = p1 - p0
    L = np.linalg.norm(ax)
    w = ax / max(L, 1e-9)
    t = np.array([1.0, 0, 0]) if abs(w[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(w, t); u /= np.linalg.norm(u)
    v = np.cross(w, u)
    ang = np.linspace(0, 2 * math.pi, sides, endpoint=False)
    ring = np.cos(ang)[:, None] * u + np.sin(ang)[:, None] * v
    r1 = r if r1 is None else r1
    P = np.vstack([p0 + r * ring, p1 + r1 * ring])
    F = list(_grid_faces(2, sides))
    if caps:
        P = np.vstack([P, p0, p1])
        c0, c1 = 2 * sides, 2 * sides + 1
        for j in range(sides):
            k = (j + 1) % sides
            F += [(c0, k, j), (c1, sides + j, sides + k)]
    return P.astype(np.float32), np.array(F, np.int32)


def sphere(c, r, n=18):
    return superellipsoid(c, r, r, r, 1.0, 1.0, nu=2 * n, nv=n)


def blades(R, hub=0.05, n_blades=2, chord=0.11, thick=0.016, twist=(20.0, 6.0), ns=14):
    """Tapered, twisted propeller blades about the z axis (radius R), as one mesh."""
    s = np.linspace(0, 1, ns)
    r = hub + (R - hub) * s
    cs = chord * np.interp(s, [0, 0.12, 0.35, 0.85, 1.0], [0.55, 0.85, 1.0, 0.55, 0.18])
    tw = np.radians(np.interp(s, [0, 1], twist))
    sec = np.array([[0.5, 0.0], [0.15, 0.5], [-0.25, 0.42], [-0.5, 0.0], [-0.25, -0.18], [0.15, -0.2]])  # airfoil (x chord, z thick)
    m = len(sec)
    Ps, Fs = [], []
    for b in range(n_blades):
        rot = 2 * math.pi * b / n_blades
        cb, sb = math.cos(rot), math.sin(rot)
        P = []
        for k in range(ns):
            ch = sec[:, 0] * cs[k]
            th = sec[:, 1] * thick * (1.0 - 0.6 * s[k])
            c, sn = math.cos(tw[k]), math.sin(tw[k])
            y, z = ch * c - th * sn, ch * sn + th * c        # chord along y (blade runs along x), pitched by twist
            x = np.full(m, r[k])
            P.append(np.stack([x * cb - y * sb, x * sb + y * cb, z], 1))
        P = np.vstack(P)
        F = list(_grid_faces(ns, m))
        n0 = len(P)
        P = np.vstack([P, P[:m].mean(0, keepdims=True), P[-m:].mean(0, keepdims=True)])
        for j in range(m):
            k = (j + 1) % m
            F += [(n0, k, j), (n0 + 1, (ns - 1) * m + j, (ns - 1) * m + k)]
        Ps.append(P); Fs.append(np.array(F, np.int32))
    return merge(list(zip(Ps, Fs)))


def merge(parts):
    P, F, o = [], [], 0
    for p, f in parts:
        P.append(np.asarray(p, np.float32)); F.append(np.asarray(f, np.int32) + o); o += len(p)
    return np.vstack(P), np.vstack(F)


def vertex_normals(P, F):
    n = np.zeros_like(P, dtype=float)
    fn = np.cross(P[F[:, 1]] - P[F[:, 0]], P[F[:, 2]] - P[F[:, 0]])
    for k in range(3):
        np.add.at(n, F[:, k], fn)
    return (n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)).astype(np.float32)


def dji_parts(spec=None, did="d1"):
    if spec is None:
        from sim.platform import get_platform
        spec = get_platform("heavy_sar")
    from isaac.drone_viz import ACCENT, COL
    acc = ACCENT.get(did, (0.9, 0.9, 0.9))
    out = []

    def mesh(name, PF, rgb):
        out.append(("Mesh", name, (0.0, 0.0, 0.0), (1.0, 1.0, 1.0), rgb, None, {"P": PF[0], "F": PF[1]}))

    z0 = GEAR_H
    # fuselage: dark lower hull, light canopy, accent stripe, glossy nose visor, batteries, GNSS mast
    mesh("Fuselage", superellipsoid((0, 0, z0 + 0.20), 0.60, 0.34, 0.20, 0.45, 0.3), COL["body"])
    mesh("Canopy", superellipsoid((-0.02, 0, z0 + 0.33), 0.50, 0.30, 0.11, 0.5, 0.35), (0.80, 0.81, 0.82))
    mesh("Stripe", superellipsoid((0.0, 0, z0 + 0.435), 0.40, 0.045, 0.012, 0.3, 0.3, nu=24, nv=6), acc)
    mesh("Visor", superellipsoid((0.50, 0, z0 + 0.24), 0.10, 0.24, 0.09, 0.6, 0.5), (0.02, 0.02, 0.025))
    for sd in (-1, 1):
        mesh(f"Battery{sd + 1}", superellipsoid((-0.33, sd * 0.13, z0 + 0.46), 0.17, 0.11, 0.12, 0.25, 0.2),
             COL["battery"])
    mesh("GpsMast", tube((0.12, 0, z0 + 0.42), (0.12, 0, z0 + 0.62), 0.012), (0.1, 0.1, 0.1))
    mesh("GpsPuck", superellipsoid((0.12, 0, z0 + 0.64), 0.065, 0.065, 0.025, 0.4, 1.0, nu=24, nv=8), (0.85, 0.85, 0.83))
    # arms with fold joints, coaxial motor pods, two mic capsules per arm
    R = spec.wheelbase_m / 2
    rp = spec.prop_d_m / 2
    mics = []
    for i in range(4):
        a = math.radians(45 + 90 * i)
        d = np.array([math.cos(a), math.sin(a), 0.0])
        root = np.array([0.30 * np.sign(d[0]), 0.20 * np.sign(d[1]), z0 + 0.30])
        tip = np.array([R * d[0], R * d[1], z0 + 0.40])
        w = (tip - root) / np.linalg.norm(tip - root)
        mesh(f"Arm{i}", tube(root, tip - w * 0.06, 0.036), COL["arm"])
        mesh(f"Hinge{i}", tube(root + w * 0.10, root + w * 0.24, 0.05), (0.1, 0.1, 0.1))
        zc = tip[2]
        mesh(f"Column{i}", tube((tip[0], tip[1], zc - 0.13), (tip[0], tip[1], zc + 0.13), 0.045), (0.1, 0.1, 0.1))
        mesh(f"MotorTop{i}", tube((tip[0], tip[1], zc + 0.05), (tip[0], tip[1], zc + 0.14), 0.085, r1=0.075),
             COL["arm"])
        mesh(f"MotorBot{i}", tube((tip[0], tip[1], zc - 0.05), (tip[0], tip[1], zc - 0.14), 0.085, r1=0.075),
             COL["arm"])
        for lvl, zr, sgn in ((1, zc + 0.17, 1 if i % 2 else -1), (0, zc - 0.17, -1 if i % 2 else 1)):
            out.append(("Rotor", f"Rotor{i}_{lvl}", (float(tip[0]), float(tip[1]), float(zr)), (1, 1, 1), COL["rotor"],
                        None, {"radius": rp, "sign": sgn, "style": "mesh"}))
        for f in (0.38, 0.68):
            p = root + (tip - root) * f + np.array([0, 0, -0.05])
            mics.append(p)
    for k, p in enumerate(mics[: spec.mics.n_mics]):
        mesh(f"Mic{k}", merge([tube(p + [0, 0, 0.04], p, 0.008), sphere(p, 0.028, n=10)]), COL["mic"])
    # landing gear: two legs per side down to skids
    for sd in (-1, 1):
        y = sd * spec.gear_footprint_m[1] / 2
        L = spec.gear_footprint_m[0]
        mesh(f"Skid{sd + 1}", tube((-L / 2, y, 0.035), (L / 2, y, 0.035), 0.03), COL["skid"])
        for fb in (-1, 1):
            mesh(f"Leg{sd + 1}{fb + 1}", tube((fb * 0.28, sd * 0.24, z0 + 0.05), (fb * 0.42, y, 0.05), 0.028), COL["skid"])
            mesh(f"Foot{sd + 1}{fb + 1}", sphere((fb * 0.42, y, 0.05), 0.04, n=10), COL["skid"])
        mesh(f"Brace{sd + 1}", tube((-0.35, sd * 0.47, z0 * 0.55), (0.35, sd * 0.47, z0 * 0.55), 0.02), COL["skid"])
    # payload: ball gimbal (thermal/zoom), LiDAR puck, radar panel, speaker
    gx = 0.46
    mesh("YokeGimbal", tube((gx, 0, z0 + 0.04), (gx, 0, z0 - 0.07), 0.022), (0.1, 0.1, 0.1))
    mesh("Gimbal", sphere((gx + 0.02, 0, z0 - 0.15), 0.085), (0.12, 0.12, 0.13))
    mesh("Thermal", tube((gx + 0.085, 0.025, z0 - 0.15), (gx + 0.11, 0.025, z0 - 0.15), 0.028), COL["thermal"])
    mesh("LensZoom", tube((gx + 0.085, -0.03, z0 - 0.13), (gx + 0.105, -0.03, z0 - 0.13), 0.02), COL["thermal"])
    # Ouster OS1-128 class LiDAR (85 mm x 73 mm) on a nose bracket, pitched 45 deg down to sweep the rubble ahead
    lp = np.array([0.66, 0.0, z0 + 0.13])
    ax = np.array([math.sin(math.radians(45)), 0.0, -math.cos(math.radians(45))])
    mesh("LidarBracket", tube((0.52, 0, z0 + 0.20), lp - ax * 0.01, 0.018), (0.1, 0.1, 0.1))
    mesh("LidarPuck", tube(lp, lp + ax * 0.062, 0.0425), COL["lidar"])
    mesh("LidarCap", tube(lp + ax * 0.062, lp + ax * 0.075, 0.0425), (0.1, 0.1, 0.1))
    if not POD_MODE:
        rw, rh = spec.radar.aperture_m
        mesh("RadarPanel", superellipsoid((-0.12, 0, z0 - 0.02), rh / 2, rw / 2, 0.03, 0.25, 0.15), COL["radar"])
    else:                                       # winch drum for the radar/mic pod (the pod itself: isaac/pods.py)
        mesh("WinchDrum", tube((-0.05, -0.11, z0 - 0.06), (-0.05, 0.11, z0 - 0.06), 0.07), (0.12, 0.12, 0.13))
        mesh("WinchFrame", superellipsoid((-0.05, 0, z0 + 0.0), 0.12, 0.15, 0.03, 0.2, 0.2, nu=20, nv=8), (0.1, 0.1, 0.1))
    if spec.speaker is not None:
        mesh("SpeakerHorn", tube((-0.50, 0, z0 + 0.06), (-0.50, 0, z0 - 0.10), 0.05, r1=0.10), COL["speaker"])
    # compute box, mesh radio with two whip antennas, blue rescue-Wi-Fi antenna (data/hardware_bom.json mounts)
    mesh("ComputeBox", superellipsoid((0.30, 0, z0 + 0.47), 0.10, 0.08, 0.035, 0.15, 0.15, nu=20, nv=8), (0.10, 0.11, 0.12))
    mesh("RadioBox", superellipsoid((-0.10, 0, z0 + 0.47), 0.07, 0.06, 0.03, 0.15, 0.15, nu=20, nv=8), (0.10, 0.11, 0.12))
    for sd in (-1, 1):
        base = np.array([-0.12, sd * 0.05, z0 + 0.50])
        mesh(f"Antenna{sd + 1}", merge([tube(base, base + [-0.04, sd * 0.06, 0.24], 0.006),
                                         tube(base, base + [0, 0, 0.03], 0.012)]), (0.04, 0.04, 0.045))
    mesh("WifiAnt", merge([tube((-0.56, 0, z0 + 0.28), (-0.56, 0, z0 + 0.52), 0.011),
                           superellipsoid((-0.56, 0, z0 + 0.46), 0.012, 0.025, 0.06, 0.3, 0.3, nu=16, nv=8)]),
         (0.08, 0.30, 0.95))
    # navigation lights: red port arm tip, green starboard, white anti-collision strobe on the canopy
    for i, key in ((1, "NavRed"), (0, "NavGreen")):            # arm 1 at 135 deg (port), arm 0 at 45 deg (starboard)
        a = math.radians(45 + 90 * i) if i == 1 else math.radians(-45)
        p = (R * 0.86 * math.cos(a), R * 0.86 * math.sin(a), z0 + 0.36)
        mesh(key, sphere(p, NAV_R, n=12), (1.0, 0.1, 0.05) if key == "NavRed" else (0.1, 1.0, 0.2))
    for i in (2, 3):
        a = math.radians(45 + 90 * i)
        mesh(f"NavRear{i}", sphere((R * 0.86 * math.cos(a), R * 0.86 * math.sin(a), z0 + 0.36), NAV_R * 0.8, n=12),
             (1.0, 0.1, 0.05) if i == 2 else (0.1, 1.0, 0.2))
    mesh("Strobe", sphere((-0.20, 0, z0 + 0.585), NAV_R * 1.2, n=12), (1.0, 1.0, 1.0))
    return out
