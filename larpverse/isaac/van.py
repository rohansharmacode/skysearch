"""Operator command van at the staging point, plus the small procedural mesh kit every Larpverse asset uses.

    from isaac import van
    van.build_van(stage, "/World/Van", xy, yaw_deg, screens=van.default_screens(), z=0.0)
    pose = van.staging_pose(sc)                  # recommended (x, y, z, yaw) next to the drone pads
    van.hide_legacy_staging(stage)               # deactivate scene.py's placeholder truck / mast / table

What it builds (all visual: no colliders, so the LiDAR and the landing search never see it), in the van's
local frame (+x = front, door on -y, origin = ground under the middle of the body):
  * box van: cab with glazing, headlights and mirrors, cargo box with a generic "SEARCH & RESCUE / MOBILE
    COMMAND" livery (procedural textures, no real organisation's branding), six wheels, a hinged side door
    swung open with steps, rear chevrons and ladder;
  * inside: a desk with three monitors showing dashboard screenshots (emissive, so they read at dusk), a
    seated operator mannequin in a hi-vis vest with a headset, a comms rack with status LEDs, ceiling light;
  * roof: a telescopic mesh-radio mast (~6 m extended) with three omni antennas, a sector panel, a green
    coordinator beacon and guy lines; a Starlink-style flat-panel terminal; an amber/white light bar; A/C unit;
  * outside: a pop-up rescue tent (table, stretcher on trestles, kit bags, floodlight), a framed generator
    with a fuel can, cable runs with a cable protector, and traffic cones marking the pad edge.

Materials are UsdPreviewSurface (pure UsdShade, what scene.py and the team's verified Isaac Lab spawners
use); ``mdl=True`` adds an OmniPBR ``mdl`` surface output to the emissive materials as well (RTX prefers it).
``mdl_override={"tent": url, ...}`` binds an MDL file (photoreal.mdl_material) over a material key.

The mesh kit (``Geo``, ``Looks``, ``Assembly``, ``T``/``Rx``/``Ry``/``Rz``) writes plain UsdGeom.Mesh with
vertex normals and ``st`` UVs, so the triangle count is exact and shading does not depend on how a renderer
tessellates USD implicit primitives.  numpy only at import; pxr is imported lazily.

CLI (no GPU): ``python -m isaac.van --out <dir>`` builds the asset test stages (grid + collapse sites, van,
dummies, 3 heavy drones), saves .usda files and writes the matplotlib projection sheets to docs/img/.
"""
from __future__ import annotations

import json
import math
import os

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEX_DIR = os.path.join(REPO, "assets", "textures")
SCREENS = ("dash_recon.jpg", "dash_map.jpg", "dash_detail.jpg")          # left, centre, right monitor
SCREEN_SOURCES = {"dash_recon.jpg": "docs/review/img/dash-b3-recon-1440.png",
                  "dash_map.jpg": "docs/review/img/dash-b3-fixture-1920.png",
                  "dash_detail.jpg": "docs/review/img/dash-b3-detail-1440.png"}
LIVERY = ("van_livery_full.png", "van_livery_panel.png", "van_livery_stripes.png", "van_rear.png")
TRI_BUDGET = 50_000


# =============================================================================================== transforms
def Rx(d):
    c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)


def Ry(d):
    c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)


def Rz(d):
    c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def T(t=(0, 0, 0), R=None, s=None):
    """4x4 column-convention matrix: p_world = R @ (s * p) + t."""
    M = np.eye(4)
    A = np.eye(3) if R is None else np.asarray(R, float)
    if s is not None:
        A = A @ np.diag(np.broadcast_to(np.asarray(s, float), 3))
    M[:3, :3] = A
    M[:3, 3] = t
    return M


def frame_from(z_axis, x_hint=(1, 0, 0)):
    """3x3 rotation whose local +z maps to ``z_axis``."""
    z = np.asarray(z_axis, float)
    z = z / (np.linalg.norm(z) + 1e-12)
    x = np.asarray(x_hint, float)
    if abs(np.dot(x, z)) > 0.95:
        x = np.array([0.0, 1.0, 0.0]) if abs(z[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    x = x - np.dot(x, z) * z
    x /= np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], 1)


# =============================================================================================== geometry
class Geo:
    """Triangle soup with per-vertex normals and UVs. Every primitive takes an optional 4x4 ``M``."""

    def __init__(self):
        self.P, self.N, self.UV, self.F = [], [], [], []
        self.n = 0
        self.has_uv = False

    @property
    def tris(self):
        return sum(len(f) for f in self.F)

    def add(self, P, N, F, UV=None, M=None, double=False):
        P = np.asarray(P, float).reshape(-1, 3)
        N = np.asarray(N, float).reshape(-1, 3)
        F = np.asarray(F, np.int64).reshape(-1, 3)
        if M is not None:
            A, t = M[:3, :3], M[:3, 3]
            P = P @ A.T + t
            N = N @ np.linalg.inv(A)
        N = N / (np.linalg.norm(N, axis=1, keepdims=True) + 1e-12)
        if UV is None:
            UV = np.zeros((len(P), 2))
        else:
            self.has_uv = True
        UV = np.asarray(UV, float).reshape(-1, 2)
        self.P.append(P); self.N.append(N); self.UV.append(UV); self.F.append(F + self.n)
        self.n += len(P)
        if double:
            self.P.append(P); self.N.append(-N); self.UV.append(UV); self.F.append(F[:, ::-1] + self.n)
            self.n += len(P)
        return self

    def merge(self, other, M=None):
        if other.n:
            P, N, UV, F = other.arrays()
            self.add(P, N, F, UV if other.has_uv else None, M)
        return self

    def arrays(self):
        if not self.n:
            return np.zeros((0, 3)), np.zeros((0, 3)), np.zeros((0, 2)), np.zeros((0, 3), np.int64)
        return np.concatenate(self.P), np.concatenate(self.N), np.concatenate(self.UV), np.concatenate(self.F)

    # ------------------------------------------------------------------ primitives
    def box(self, size, c=(0, 0, 0), M=None, R=None, double=False):
        """Axis-aligned box of ``size`` centred at ``c`` (then rotated by R about c, then M)."""
        sx, sy, sz = (np.asarray(size, float) / 2).tolist()
        P, N, UV, F = [], [], [], []
        for ax in range(3):
            for sg in (-1, 1):
                n = np.zeros(3); n[ax] = sg
                b, cc = [(1, 2), (2, 0), (0, 1)][ax]
                quad = []
                for u, v in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                    p = np.zeros(3); p[ax] = sg; p[b] = u; p[cc] = v
                    quad.append(p * (sx, sy, sz))
                k = len(P)
                P += quad; N += [n] * 4; UV += [(0, 0), (1, 0), (1, 1), (0, 1)]
                tri = [(k, k + 1, k + 2), (k, k + 2, k + 3)]
                if np.dot(np.cross(quad[1] - quad[0], quad[2] - quad[0]), n) < 0:
                    tri = [(a, cc_, bb) for a, bb, cc_ in tri]
                F += tri
        return self.add(P, N, F, None, _compose(M, c, R), double)

    def cyl(self, r, h, c=(0, 0, 0), axis="z", n=16, r_top=None, caps=True, M=None, R=None, uv=False):
        """Cylinder / frustum (``r_top``) / cone (``r_top=0``) of height h centred at c along ``axis``."""
        rt = r if r_top is None else r_top
        a = np.linspace(0, 2 * math.pi, n + 1)
        ca, sa = np.cos(a), np.sin(a)
        slope = (r - rt) / h if h else 0.0
        P, N, UV, F = [], [], [], []
        for i in range(n + 1):
            P += [(r * ca[i], r * sa[i], -h / 2), (rt * ca[i], rt * sa[i], h / 2)]
            nn = (ca[i], sa[i], slope)
            N += [nn, nn]
            UV += [(i / n, 0), (i / n, 1)]
        for i in range(n):
            b0, t0, b1, t1 = 2 * i, 2 * i + 1, 2 * i + 2, 2 * i + 3
            F += [(b0, b1, t1)] + ([(b0, t1, t0)] if rt > 0 else [])
        if caps:
            for z, sg, rr in ((h / 2, 1, rt), (-h / 2, -1, r)):
                if rr <= 0:
                    continue
                k = len(P)
                P.append((0, 0, z)); N.append((0, 0, sg)); UV.append((0.5, 0.5))
                for i in range(n):
                    P.append((rr * ca[i], rr * sa[i], z)); N.append((0, 0, sg))
                    UV.append((0.5 + 0.5 * ca[i], 0.5 + 0.5 * sa[i]))
                for i in range(n):
                    j, jn = k + 1 + i, k + 1 + (i + 1) % n
                    F.append((k, j, jn) if sg > 0 else (k, jn, j))
        Ra = {"z": np.eye(3), "x": Ry(90), "y": Rx(-90)}[axis] if isinstance(axis, str) else frame_from(axis)
        R2 = Ra if R is None else np.asarray(R) @ Ra
        return self.add(P, N, F, UV if uv else None, _compose(M, c, R2))

    def cyl_between(self, p0, p1, r, n=8, r1=None, caps=True):
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        d = p1 - p0
        L = float(np.linalg.norm(d))
        if L < 1e-6:
            return self
        return self.cyl(r, L, (p0 + p1) / 2, axis=d / L, n=n, r_top=r1, caps=caps)

    def tube(self, pts, r, n=6):
        pts = np.asarray(pts, float)
        for a, b in zip(pts[:-1], pts[1:]):
            self.cyl_between(a, b, r, n=n, caps=False)
            self.sphere(r, a, n=n, m=4)
        return self

    def sphere(self, r, c=(0, 0, 0), n=16, m=10, M=None, R=None, z_range=(-1.0, 1.0)):
        """UV sphere / ellipsoid (r = scalar or (a, b, c)); ``z_range`` clips to a band (domes)."""
        a, b, cz = np.broadcast_to(np.asarray(r, float), 3)
        lat = np.linspace(math.asin(z_range[0]), math.asin(z_range[1]), m + 1)
        lon = np.linspace(0, 2 * math.pi, n + 1)
        P, N, UV, F = [], [], [], []
        for j, la in enumerate(lat):
            for i, lo in enumerate(lon):
                u = np.array([math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la)])
                P.append(u * (a, b, cz)); N.append(u / (a, b, cz)); UV.append((i / n, j / m))
        W = n + 1
        for j in range(m):
            for i in range(n):
                v00, v01, v10, v11 = j * W + i, j * W + i + 1, (j + 1) * W + i, (j + 1) * W + i + 1
                if not (j == 0 and z_range[0] <= -1.0):
                    F.append((v00, v01, v11))
                if not (j == m - 1 and z_range[1] >= 1.0):
                    F.append((v00, v11, v10))
        return self.add(P, N, F, None, _compose(M, c, R))

    def ellipsoid(self, c, axes, n=14, m=9):
        """Oriented ellipsoid: ``axes`` = three (unnormalised) semi-axis vectors."""
        A = np.stack([np.asarray(v, float) for v in axes], 1)
        L = np.linalg.norm(A, axis=0)
        return self.sphere(tuple(L), (0, 0, 0), n, m, M=T(c, A / L))

    def capsule(self, p0, p1, r0, r1=None, n=10, rings=3):
        """Tapered capsule between two points (limbs)."""
        r1 = r0 if r1 is None else r1
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        d = p1 - p0
        L = float(np.linalg.norm(d))
        Rm = frame_from(d if L > 1e-6 else (0, 0, 1))
        rows = [(-math.pi / 2 + math.pi / 2 * k / rings, 0.0, r0) for k in range(rings + 1)]
        rows += [(math.pi / 2 * k / rings, L, r1) for k in range(rings + 1)]
        lon = np.linspace(0, 2 * math.pi, n + 1)
        P, N, F = [], [], []
        for la, zo, rr in rows:
            for lo in lon:
                u = np.array([math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la)])
                P.append(u * rr + (0, 0, zo)); N.append(u)
        W = n + 1
        for j in range(len(rows) - 1):
            for i in range(n):
                v00, v01, v10, v11 = j * W + i, j * W + i + 1, (j + 1) * W + i, (j + 1) * W + i + 1
                if j > 0:
                    F.append((v00, v01, v11))
                if j < len(rows) - 2:
                    F.append((v00, v11, v10))
        return self.add(P, N, F, None, T(p0, Rm))

    def torus(self, R, r, c=(0, 0, 0), axis="z", n=24, m=8, M=None):
        P, N, F = [], [], []
        for i in range(n + 1):
            u = 2 * math.pi * i / n
            for j in range(m + 1):
                v = 2 * math.pi * j / m
                nn = np.array([math.cos(v) * math.cos(u), math.cos(v) * math.sin(u), math.sin(v)])
                P.append(np.array([R * math.cos(u), R * math.sin(u), 0]) + r * nn); N.append(nn)
        W = m + 1
        for i in range(n):
            for j in range(m):
                a, b, cc, d = i * W + j, (i + 1) * W + j, (i + 1) * W + j + 1, i * W + j + 1
                F += [(a, b, cc), (a, cc, d)]
        Ra = {"z": np.eye(3), "x": Ry(90), "y": Rx(-90)}[axis]
        return self.add(P, N, F, None, _compose(M, c, Ra))

    def quad(self, p0, p1, p2, p3, uv=(0, 0, 1, 1), double=False):
        """Planar quad p0..p3 counter-clockwise seen from the front; uv rect (u0, v0, u1, v1) maps p0 -> (u0, v0)."""
        P = np.array([p0, p1, p2, p3], float)
        n = np.cross(P[1] - P[0], P[2] - P[0])
        u0, v0, u1, v1 = uv
        return self.add(P, [n] * 4, [(0, 1, 2), (0, 2, 3)], [(u0, v0), (u1, v0), (u1, v1), (u0, v1)],
                        double=double)

    def tri(self, a, b, c, double=False):
        n = np.cross(np.subtract(b, a), np.subtract(c, a))
        return self.add([a, b, c], [n] * 3, [(0, 1, 2)], double=double)

    def prism(self, profile, a0, a1, plane="yz", M=None):
        """Convex 2-D profile extruded between a0 and a1. plane 'yz' extrudes along x, 'xz' along y."""
        pr = np.asarray(profile, float)
        if 0.5 * np.sum(pr[:, 0] * np.roll(pr[:, 1], -1) - np.roll(pr[:, 0], -1) * pr[:, 1]) < 0:
            pr = pr[::-1]

        def lift(u, v, a):
            return (a, u, v) if plane == "yz" else (u, a, v)
        k = len(pr)
        G = Geo()
        for i in range(k):
            (u0, v0), (u1, v1) = pr[i], pr[(i + 1) % k]
            G.quad(lift(u0, v0, a0), lift(u1, v1, a0), lift(u1, v1, a1), lift(u0, v0, a1))
        for a in (a0, a1):
            for i in range(1, k - 1):
                G.tri(lift(*pr[0], a), lift(*pr[i], a), lift(*pr[i + 1], a))
        P, N, UV, F = G.arrays()                      # orient every face away from the centroid (convex);
        cen = P.mean(0)                               # vertices are per-face here, so flat normals are safe
        for t in range(len(F)):
            a_, b_, c_ = P[F[t]]
            nn = np.cross(b_ - a_, c_ - a_)
            if np.dot(nn, (a_ + b_ + c_) / 3 - cen) < 0:
                F[t] = F[t][::-1]
                nn = -nn
            N[F[t]] = nn
        return self.add(P, N, F, None, M)


def _compose(M, c, R):
    L = T(c, R)
    return L if M is None else M @ L


# =============================================================================================== materials
class Looks:
    """UsdPreviewSurface materials cached by name (+ optional OmniPBR ``mdl`` output for emissive ones)."""

    def __init__(self, stage, root="/World/Looks/Assets", mdl=False):
        self.stage, self.root, self.mdl = stage, root, mdl
        self.cache, self.rgb = {}, {}

    def get(self, name, rgb=(0.5, 0.5, 0.5), rough=0.6, metal=0.0, emissive=None, tex=None, tex_emissive=0.0,
            opacity=None, clearcoat=0.0):
        if name in self.cache:
            return self.cache[name]
        from pxr import Gf, Sdf, UsdShade
        path = f"{self.root}/{name}"
        mat = UsdShade.Material.Define(self.stage, path)
        sh = UsdShade.Shader.Define(self.stage, path + "/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        F, C3 = Sdf.ValueTypeNames.Float, Sdf.ValueTypeNames.Color3f
        sh.CreateInput("roughness", F).Set(float(rough))
        sh.CreateInput("metallic", F).Set(float(metal))
        if clearcoat:
            sh.CreateInput("clearcoat", F).Set(float(clearcoat))
            sh.CreateInput("clearcoatRoughness", F).Set(0.08)
        if opacity is not None:
            sh.CreateInput("opacity", F).Set(float(opacity))
        diffuse = sh.CreateInput("diffuseColor", C3)
        if tex:
            st = UsdShade.Shader.Define(self.stage, path + "/St")
            st.CreateIdAttr("UsdPrimvarReader_float2")
            st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            st_out = st.CreateOutput("result", Sdf.ValueTypeNames.Float2)

            def texture(nm, scale):
                tx = UsdShade.Shader.Define(self.stage, f"{path}/{nm}")
                tx.CreateIdAttr("UsdUVTexture")
                tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(tex))
                tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st_out)
                tx.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
                tx.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
                tx.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
                tx.CreateInput("fallback", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*map(float, rgb), 1.0))
                if scale != 1.0:
                    tx.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(scale, scale, scale, 1.0))
                return tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
            diffuse.ConnectToSource(texture("Tex", 1.0 if not tex_emissive else 0.25))
            if tex_emissive:
                sh.CreateInput("emissiveColor", C3).ConnectToSource(texture("TexE", float(tex_emissive)))
        else:
            diffuse.Set(Gf.Vec3f(*map(float, rgb)))
        if emissive is not None:
            sh.CreateInput("emissiveColor", C3).Set(Gf.Vec3f(*map(float, emissive)))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        if self.mdl and (emissive is not None or tex_emissive):
            self._omnipbr(mat, path, rgb, rough, emissive, tex, tex_emissive)
        self.cache[name] = mat
        self.rgb[name] = tuple(map(float, rgb if emissive is None else np.clip(np.asarray(emissive) / max(1.0, max(emissive)), 0, 1)))
        return mat

    def _omnipbr(self, mat, path, rgb, rough, emissive, tex, tex_emissive):
        """OmniPBR (core MDL shipped with Kit) as the mdl render context: emissive intensity in nits."""
        from pxr import Gf, Sdf, UsdShade
        sh = UsdShade.Shader.Define(self.stage, path + "/MDL")
        sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
        sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
        sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*map(float, rgb)))
        sh.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(float(rough))
        sh.CreateInput("enable_emission", Sdf.ValueTypeNames.Bool).Set(True)
        if tex:
            sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(tex))
            sh.CreateInput("emissive_color_texture", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(tex))
            sh.CreateInput("emissive_intensity", Sdf.ValueTypeNames.Float).Set(600.0 * float(tex_emissive))
        else:
            e = np.asarray(emissive, float)
            sh.CreateInput("emissive_color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*(e / max(1e-6, e.max())).tolist()))
            sh.CreateInput("emissive_intensity", Sdf.ValueTypeNames.Float).Set(600.0 * float(e.max()))
        mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.CreateOutput("out", Sdf.ValueTypeNames.Token))


def emit(stage, path, geo, mat=None, rgb=(0.5, 0.5, 0.5), label=None):
    """Write ``geo`` as one UsdGeom.Mesh at ``path`` (vertex normals, st UVs), bind ``mat``. Returns triangles."""
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt
    P, N, UV, F = geo.arrays()
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(P, np.float32)))
    m.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(F), 3, np.int32)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.ascontiguousarray(F.reshape(-1), np.int32)))
    m.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(N, np.float32)))
    m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    m.CreateSubdivisionSchemeAttr("none")
    if len(P):
        m.CreateExtentAttr([Gf.Vec3f(*map(float, P.min(0))), Gf.Vec3f(*map(float, P.max(0)))])
    if geo.has_uv:
        pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        pv.Set(Vt.Vec2fArray.FromNumpy(np.ascontiguousarray(UV, np.float32)))
    m.CreateDisplayColorAttr([Gf.Vec3f(*map(float, rgb))])
    if mat is not None:
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    if label:
        m.GetPrim().CreateAttribute("larpverse:label", Sdf.ValueTypeNames.String).Set(label)
    return len(F)


class Assembly:
    """Collects geometry per (group, material) and writes one Mesh per pair under ``root``."""

    def __init__(self, stage, root, looks, mats):
        self.stage, self.root, self.looks, self.mats = stage, root, looks, mats
        self.parts = {}
        self.tris = {}

    def g(self, mat, group="Body"):
        key = (group, mat)
        if key not in self.parts:
            self.parts[key] = Geo()
        return self.parts[key]

    def flush(self, override=None):
        from pxr import UsdGeom
        for (group, mat), geo in self.parts.items():
            if not geo.n:
                continue
            gp = f"{self.root}/{group}"
            if not self.stage.GetPrimAtPath(gp).IsValid():
                UsdGeom.Xform.Define(self.stage, gp)
            spec = self.mats[mat]
            m = self.looks.get(mat, **spec)
            if override and mat in override:
                m = override[mat]
            n = emit(self.stage, f"{gp}/{mat}", geo, m, self.looks.rgb.get(mat, spec.get("rgb", (0.5, 0.5, 0.5))))
            self.tris[group] = self.tris.get(group, 0) + n
        self.parts = {}
        return sum(self.tris.values())


def xform(stage, path, t=(0, 0, 0), yaw_deg=0.0, scale=None, R=None):
    from pxr import Gf, UsdGeom
    x = UsdGeom.Xform.Define(stage, path)
    xf = UsdGeom.Xformable(x)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, t)))
    if R is not None:
        q = Gf.Matrix3d(*np.asarray(R, float).T.flatten().tolist()).ExtractRotation().GetQuat()
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(q)
    elif yaw_deg:
        xf.AddRotateZOp(UsdGeom.XformOp.PrecisionDouble).Set(float(yaw_deg))
    if scale is not None:
        xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*np.broadcast_to(np.asarray(scale, float), 3).tolist()))
    return x


def count_tris(stage, root):
    """Triangles of every Mesh under ``root`` (fan-triangulated face counts)."""
    from pxr import Usd, UsdGeom
    n = 0
    prim = stage.GetPrimAtPath(root)
    if not prim.IsValid():
        return 0
    for p in Usd.PrimRange(prim):
        if p.IsA(UsdGeom.Mesh):
            c = UsdGeom.Mesh(p).GetFaceVertexCountsAttr().Get()
            if c is not None:
                n += int(np.sum(np.maximum(np.asarray(c) - 2, 0)))
    return n


# =============================================================================================== textures
def tex_path(name):
    return os.path.join(TEX_DIR, name)


def default_screens():
    """Absolute paths of the three monitor textures (left, centre, right)."""
    return [tex_path(s) for s in SCREENS]


def make_textures(out_dir=TEX_DIR, force=False):
    """Write the livery textures (PIL + matplotlib's DejaVu font) and the downscaled dashboard screenshots."""
    from PIL import Image, ImageDraw, ImageFont
    os.makedirs(out_dir, exist_ok=True)
    try:
        import matplotlib
        fdir = os.path.join(os.path.dirname(matplotlib.__file__), "mpl-data", "fonts", "ttf")
        bold = os.path.join(fdir, "DejaVuSans-Bold.ttf")
        ImageFont.truetype(bold, 10)
    except Exception:
        bold = None

    def font(px):
        return ImageFont.truetype(bold, px) if bold else ImageFont.load_default()
    WHITE, NAVY, AMBER, GREY = (236, 238, 238), (20, 40, 72), (246, 176, 24), (60, 66, 74)

    def bands(im, v_lo=0.13, v_hi=0.25):
        W, H = im.size
        d = ImageDraw.Draw(im)
        y_navy = (int(H * (1 - v_hi)), int(H * (1 - v_lo)))
        d.rectangle([0, y_navy[0], W, y_navy[1]], fill=NAVY)
        d.rectangle([0, y_navy[0] - int(H * 0.035), W, y_navy[0] - int(H * 0.012)], fill=AMBER)
        d.rectangle([0, y_navy[1] + int(H * 0.012), W, y_navy[1] + int(H * 0.03)], fill=AMBER)
        d.rectangle([0, H - int(H * 0.035), W, H], fill=GREY)
        return d

    def centred(d, W, y, text, px, fill=NAVY):
        f = font(px)
        w = d.textlength(text, font=f)
        d.text(((W - w) / 2, y), text, font=f, fill=fill)

    out = {}
    p = os.path.join(out_dir, LIVERY[0])
    if force or not os.path.exists(p):           # +y side, full 5.05 x 2.5 m wall
        im = Image.new("RGB", (2048, 1014), WHITE)
        d = bands(im)
        centred(d, 2048, 300, "SEARCH & RESCUE", 150)
        centred(d, 2048, 478, "MOBILE COMMAND UNIT", 64, GREY)
        centred(d, 2048, im.size[1] - 228, "CMD 1", 70, WHITE)
        im.save(p); out["full"] = p
    p = os.path.join(out_dir, LIVERY[1])
    if force or not os.path.exists(p):           # -y side, rear panel 2.4 x 2.5 m
        im = Image.new("RGB", (1024, 1066), WHITE)
        d = bands(im)
        centred(d, 1024, 250, "SEARCH &", 130)
        centred(d, 1024, 400, "RESCUE", 130)
        centred(d, 1024, 560, "MOBILE COMMAND", 52, GREY)
        im.save(p); out["panel"] = p
    p = os.path.join(out_dir, LIVERY[2])
    if force or not os.path.exists(p):           # stripes only (front panel, door): stretches cleanly
        im = Image.new("RGB", (256, 1024), WHITE)
        bands(im)
        im.save(p); out["stripes"] = p
    p = os.path.join(out_dir, LIVERY[3])
    if force or not os.path.exists(p):           # rear: amber / navy chevrons (generic high-vis pattern)
        W, H = 1024, 1024
        im = Image.new("RGB", (W, H), WHITE)
        d = ImageDraw.Draw(im)
        lo = int(H * 0.62)
        for k in range(-12, 16):
            x0 = k * 110
            col = AMBER if k % 2 else NAVY
            d.polygon([(x0, H), (x0 + 110, H), (x0 + 110 + (H - lo), lo), (x0 + (H - lo), lo)], fill=col)
        d.rectangle([0, lo - 18, W, lo], fill=GREY)
        centred(d, W, 180, "KEEP BACK", 96)
        centred(d, W, 300, "DRONE OPERATIONS", 56, GREY)
        im.save(p); out["rear"] = p
    for name, src in SCREEN_SOURCES.items():
        p = os.path.join(out_dir, name)
        if force or not os.path.exists(p):
            im = Image.open(os.path.join(REPO, src)).convert("RGB")
            w = 1280
            im = im.resize((w, int(round(im.size[1] * w / im.size[0]))), Image.LANCZOS)
            im.save(p, quality=88); out[name] = p
    return out


# =============================================================================================== van
VAN_MATS = {
    "paint": dict(rgb=(0.86, 0.87, 0.86), rough=0.35, clearcoat=0.6),
    "trim": dict(rgb=(0.07, 0.075, 0.08), rough=0.55),
    "chassis": dict(rgb=(0.05, 0.05, 0.055), rough=0.7, metal=0.3),
    "tyre": dict(rgb=(0.035, 0.035, 0.035), rough=0.9),
    "hub": dict(rgb=(0.55, 0.56, 0.58), rough=0.35, metal=0.9),
    "glass": dict(rgb=(0.03, 0.04, 0.05), rough=0.06, metal=0.2),
    "alu": dict(rgb=(0.70, 0.72, 0.74), rough=0.3, metal=0.9),
    "headlight": dict(rgb=(0.9, 0.9, 0.85), rough=0.2, emissive=(3.0, 2.9, 2.6)),
    "amber_lamp": dict(rgb=(0.95, 0.55, 0.05), rough=0.2, emissive=(4.0, 1.9, 0.15)),
    "white_lamp": dict(rgb=(0.95, 0.95, 0.95), rough=0.2, emissive=(3.5, 3.5, 3.4)),
    "beacon_green": dict(rgb=(0.2, 0.8, 0.3), rough=0.3, emissive=(0.4, 3.0, 0.6)),
    "led_green": dict(rgb=(0.2, 0.8, 0.3), rough=0.3, emissive=(0.3, 2.2, 0.4)),
    "led_blue": dict(rgb=(0.2, 0.5, 0.9), rough=0.3, emissive=(0.3, 1.0, 2.4)),
    "floor": dict(rgb=(0.12, 0.12, 0.13), rough=0.95),
    "interior": dict(rgb=(0.62, 0.63, 0.64), rough=0.8),
    "desk": dict(rgb=(0.20, 0.21, 0.23), rough=0.5),
    "plastic": dict(rgb=(0.04, 0.04, 0.045), rough=0.45),
    "fabric": dict(rgb=(0.10, 0.11, 0.13), rough=0.95),
    "rack": dict(rgb=(0.08, 0.085, 0.09), rough=0.5, metal=0.5),
    "dish": dict(rgb=(0.92, 0.92, 0.91), rough=0.45),
    "ceiling_light": dict(rgb=(0.95, 0.93, 0.88), rough=0.3, emissive=(2.2, 2.1, 1.9)),
    "canopy": dict(rgb=(0.10, 0.24, 0.50), rough=0.85),
    "canopy_trim": dict(rgb=(0.88, 0.89, 0.90), rough=0.8),
    "table": dict(rgb=(0.80, 0.80, 0.78), rough=0.6),
    "stretcher": dict(rgb=(0.95, 0.47, 0.08), rough=0.5),
    "kitbag": dict(rgb=(0.12, 0.42, 0.26), rough=0.85),
    "case": dict(rgb=(0.16, 0.17, 0.18), rough=0.6),
    "generator": dict(rgb=(0.92, 0.70, 0.08), rough=0.5),
    "cable": dict(rgb=(0.03, 0.03, 0.03), rough=0.7),
    "protector": dict(rgb=(0.85, 0.65, 0.05), rough=0.7),
    "cone": dict(rgb=(0.98, 0.42, 0.05), rough=0.55),
    "reflective": dict(rgb=(0.95, 0.95, 0.95), rough=0.25),
    "flood": dict(rgb=(0.95, 0.92, 0.85), rough=0.2, emissive=(5.0, 4.7, 4.1)),
    # operator mannequin
    "skin": dict(rgb=(0.64, 0.52, 0.44), rough=0.55),
    "uniform": dict(rgb=(0.10, 0.12, 0.16), rough=0.9),
    "hivis": dict(rgb=(0.80, 0.92, 0.10), rough=0.7),
    "boots": dict(rgb=(0.04, 0.04, 0.04), rough=0.6),
}

# body dimensions (m), van frame: +x front, door on -y, origin on the ground under the cargo-box middle
CARGO = dict(x0=-3.5, x1=1.55, w=2.5, z0=0.92, z1=3.45)
DOOR = dict(x0=-1.1, x1=0.2, z0=1.0, z1=3.0, open_deg=100.0)
FLOOR_Z = 1.0
DESK = dict(x0=-2.1, x1=0.7, y0=0.5, y1=1.2, top=FLOOR_Z + 0.75)
MAST_H = 6.0


def staging_pose(sc):
    """Recommended van pose (x, y, z, yaw_deg): 10 m east of the staging centre, beside the drone pads, front
    facing south, open door facing the pads (west), so a camera south-east of the van sees the door, the
    operator at the screens and the drones lifting off behind it.  Same offsets on the grid and collapse sites
    (both checked clear of buildings and inside the site bounds)."""
    from isaac.site import base_xy
    bx, by = base_xy(sc)
    x, y = bx + 10.0, by - 2.0
    try:
        z = float(max(sc.ground_height(x + dx, y + dy) for dx in (-1.2, 1.2) for dy in (-3.6, 3.6)))
    except Exception:
        z = 0.0
    return x, y, z, -90.0


def hide_legacy_staging(stage):
    """Deactivate scene.py's placeholder staging boxes the van replaces (truck, mast, beacon, table, screen)."""
    n = 0
    for name in ("Truck", "Mast", "CoordinatorBeacon", "DashboardTable", "DashboardScreen"):
        p = stage.GetPrimAtPath(f"/World/Staging/{name}")
        if p.IsValid():
            p.SetActive(False)
            n += 1
    return n


def build_van(stage, root, xy, yaw_deg=-90.0, screens=None, z=0.0, outside=True, operator=True,
              lights=True, mdl=False, mdl_override=None, mast_h=MAST_H, light_scale=1.0):
    """Build the command van at ``root`` (world pose xy, z, yaw). Returns {"tris": {...}, "total": n,
    "anchors": {name: local xyz}}. ``screens``: three image paths (left, centre, right); default dashboard shots.
    ``mdl_override``: {material key: UsdShade.Material} (e.g. photoreal.mdl_material) bound instead."""
    from pxr import Sdf
    if screens is None:
        screens = default_screens()
    if not all(os.path.exists(s) for s in screens if s):
        make_textures()
    root_x = xform(stage, root, (xy[0], xy[1], z), yaw_deg)
    prim = root_x.GetPrim()
    prim.CreateAttribute("larpverse:asset", Sdf.ValueTypeNames.String).Set("command_van")
    looks = Looks(stage, root + "/Looks", mdl=mdl)
    A = Assembly(stage, root, looks, VAN_MATS)
    _van_body(A)
    _van_textured(stage, root, looks, A)
    _van_interior(stage, root, looks, A, screens)
    _van_roof(A, mast_h)
    if operator:
        _van_operator(A)
    if outside:
        _van_outside(A)
    A.flush(mdl_override)
    if lights:
        _van_lights(stage, root, light_scale, mast_h)
    total = count_tris(stage, root)
    return {"tris": dict(A.tris), "total": total, "anchors": van_anchors(mast_h)}


def van_anchors(mast_h=MAST_H):
    """Label / camera anchor points in the van frame (metres)."""
    return {"door": ((DOOR["x0"] + DOOR["x1"]) / 2, -CARGO["w"] / 2 - 0.4, 2.0),
            "screens": (-0.7, 0.95, DESK["top"] + 0.4),
            "operator_head": (-0.7, 0.07, FLOOR_Z + 1.18),
            "mast_top": (-2.9, 0.75, CARGO["z1"] + mast_h + 0.4),
            "dish": (-1.0, -0.55, CARGO["z1"] + 0.45),
            "light_bar": (2.2, 0.0, 2.85),
            "tent": (-6.5, -1.0, 2.0),
            "generator": (2.0, 3.6, 0.6)}


def _van_body(A):
    x0, x1, w, z0, z1 = CARGO["x0"], CARGO["x1"], CARGO["w"], CARGO["z0"], CARGO["z1"]
    hw, L, xm, H = w / 2, x1 - x0, (x0 + x1) / 2, z1 - z0
    paint, trim = A.g("paint"), A.g("trim")
    t = 0.06
    paint.box((L, w, 0.1), (xm, 0, z0 + 0.03))                                 # floor slab
    paint.box((L, w, t), (xm, 0, z1 - t / 2))                                  # roof
    paint.box((t, w, H), (x1 - t / 2, 0, z0 + H / 2))                          # front wall
    paint.box((t, w, H), (x0 + t / 2, 0, z0 + H / 2))                          # rear wall
    paint.box((L, t, H), (xm, hw - t / 2, z0 + H / 2))                         # +y wall
    dx0, dx1, dz1 = DOOR["x0"], DOOR["x1"], DOOR["z1"]
    paint.box((dx0 - x0, t, H), ((x0 + dx0) / 2, -hw + t / 2, z0 + H / 2))     # -y wall around the door
    paint.box((x1 - dx1, t, H), ((dx1 + x1) / 2, -hw + t / 2, z0 + H / 2))
    paint.box((dx1 - dx0, t, z1 - dz1), ((dx0 + dx1) / 2, -hw + t / 2, (dz1 + z1) / 2))
    for sy in (-1, 1):                                                          # rounded roof edges
        paint.cyl(0.05, L, (xm, sy * (hw - 0.03), z1 - 0.03), axis="x", n=10)
    trim.box((L + 0.02, w + 0.04, 0.08), (xm, 0, z0 - 0.02))                   # rubbing strip / sill
    trim.box((dx1 - dx0 + 0.08, 0.03, 0.05), ((dx0 + dx1) / 2, -hw - 0.012, dz1 + 0.025))
    # cab: convex lower (bonnet) + upper (cabin) prisms in x-z, extruded across y
    cw = 1.15
    paint.prism([(1.58, 0.62), (3.80, 0.62), (3.85, 1.35), (3.35, 1.62), (1.58, 1.62)], -cw, cw, plane="xz")
    paint.prism([(1.58, 1.62), (3.35, 1.62), (2.78, 2.70), (1.58, 2.76)], -cw, cw, plane="xz")
    glass = A.g("glass")
    d = np.array([2.78 - 3.35, 2.70 - 1.62]); d /= np.linalg.norm(d)
    nrm = np.array([d[1], -d[0]]) * 0.012
    a = np.array([3.35, 1.62]) + d * 0.08 + nrm
    b = np.array([3.35, 1.62]) + d * (np.hypot(0.57, 1.08) - 0.08) + nrm
    glass.quad((a[0], -1.02, a[1]), (a[0], 1.02, a[1]), (b[0], 1.02, b[1]), (b[0], -1.02, b[1]))
    for sy in (-1, 1):                                                          # side windows
        yy = sy * (cw + 0.006)
        q = [(1.80, yy, 1.78), (3.12, yy, 1.78), (2.70, yy, 2.58), (1.80, yy, 2.60)]
        glass.quad(*(q if sy < 0 else q[::-1]))
        trim.box((0.05, 0.12, 0.30), (2.98, sy * (cw + 0.16), 2.08))           # mirrors
        trim.box((0.40, 0.04, 0.03), (3.05, sy * (cw + 0.04), 2.0))
        A.g("headlight").box((0.03, 0.34, 0.15), (3.85, sy * 0.78, 1.18))
        A.g("amber_lamp").box((0.03, 0.10, 0.08), (3.85, sy * 1.02, 1.18))
        A.g("amber_lamp").box((0.03, 0.06, 0.18), (x0 - 0.02, sy * 1.1, 1.25))  # rear lamps (amber, not red)
        A.g("white_lamp").box((0.03, 0.06, 0.08), (x0 - 0.02, sy * 1.1, 1.08))
    trim.box((0.03, 1.0, 0.32), (3.86, 0, 0.98))                                 # grille
    trim.box((0.16, 2.36, 0.22), (3.90, 0, 0.72))                                # bumper
    trim.box((0.12, 2.5, 0.18), (x0 - 0.06, 0, 0.82))                            # rear bumper
    trim.box((0.02, 0.025, 2.3), (x0 - 0.012, 0, z0 + 1.2))                      # rear door seam
    for sy in (-1, 1):
        trim.box((0.04, 0.05, 0.25), (x0 - 0.03, sy * 0.12, 1.9))                # handles
    # chassis, wheels, steps
    A.g("chassis").box((7.1, 0.9, 0.22), (0.15, 0, 0.66))
    A.g("chassis").cyl(0.22, 0.9, (-0.2, 0.62, 0.6), axis="x", n=14)            # fuel tank
    for wx, ys in ((2.75, (1.0,)), (-1.95, (0.80, 1.10))):
        for sy in (-1, 1):
            for yy in ys:
                c = (wx, sy * yy, 0.42)
                A.g("tyre").cyl(0.42, 0.27, c, axis="y", n=22)
                A.g("tyre").torus(0.36, 0.06, (c[0], c[1] + sy * 0.12, c[2]), axis="y", n=22, m=6)
                A.g("hub").cyl(0.24, 0.275, c, axis="y", n=16)
                A.g("hub").cyl(0.07, 0.30, c, axis="y", n=8)
    for k, (hz, dy) in enumerate(((0.36, 0.42), (0.66, 0.2))):                  # door steps
        A.g("alu").box((1.1, 0.30, 0.04), ((DOOR["x0"] + DOOR["x1"]) / 2, -hw - dy, hz))
        A.g("chassis").box((0.04, 0.04, 0.32), (DOOR["x0"] + 0.1, -hw - dy, hz - 0.17))
        A.g("chassis").box((0.04, 0.04, 0.32), (DOOR["x1"] - 0.1, -hw - dy, hz - 0.17))
    A.g("alu").cyl_between((DOOR["x1"] + 0.06, -hw - 0.05, 1.3), (DOOR["x1"] + 0.06, -hw - 0.05, 2.5), 0.018)
    for zz in np.arange(1.0, z1 - 0.1, 0.32):                                   # rear ladder
        A.g("alu").cyl_between((x0 - 0.10, 0.62, zz), (x0 - 0.10, 0.98, zz), 0.015, n=6)
    for yy in (0.62, 0.98):
        A.g("alu").cyl_between((x0 - 0.10, yy, 0.95), (x0 - 0.10, yy, z1 + 0.25), 0.02, n=6)


def _door_frame():
    """Open hinged side door: hinge at the door's rear edge, swung outwards by open_deg."""
    hw = CARGO["w"] / 2
    a = math.radians(-DOOR["open_deg"])
    width = DOOR["x1"] - DOOR["x0"]
    hinge = np.array([DOOR["x0"], -hw - 0.03])
    dirv = np.array([math.cos(a), math.sin(a)])
    return hinge, dirv, width


def _van_textured(stage, root, looks, A):
    """Livery quads (own materials: one texture each) and the open side door."""
    x0, x1, w, z0, z1 = CARGO["x0"], CARGO["x1"], CARGO["w"], CARGO["z0"], CARGO["z1"]
    hw, eps = w / 2, 0.004
    zt = z1 - 0.06
    full = Geo().quad((x1, hw + eps, z0), (x0, hw + eps, z0), (x0, hw + eps, zt), (x1, hw + eps, zt))
    panel = Geo().quad((x0, -hw - eps, z0), (DOOR["x0"], -hw - eps, z0), (DOOR["x0"], -hw - eps, zt),
                       (x0, -hw - eps, zt))
    stripes = Geo().quad((DOOR["x1"], -hw - eps, z0), (x1, -hw - eps, z0), (x1, -hw - eps, zt),
                         (DOOR["x1"], -hw - eps, zt))
    rear = Geo().quad((x0 - eps, hw, z0), (x0 - eps, -hw, z0), (x0 - eps, -hw, zt), (x0 - eps, hw, zt))
    hinge, dv, wd = _door_frame()                                                # door: outer face striped
    nv = np.array([dv[1], -dv[0]])                                               # outward-ish normal (-y side)
    p0 = np.append(hinge, DOOR["z0"]); p1 = np.append(hinge + dv * wd, DOOR["z0"])
    off = np.append(nv * 0.025, 0)
    stripes.quad(p0 + off, p1 + off, p1 + off + (0, 0, DOOR["z1"] - DOOR["z0"]),
                 p0 + off + (0, 0, DOOR["z1"] - DOOR["z0"]), uv=(0, 0.03, 1, 0.83))
    cen = (p0 + p1) / 2 + (0, 0, (DOOR["z1"] - DOOR["z0"]) / 2)
    yaw = math.degrees(math.atan2(dv[1], dv[0]))
    A.g("paint").box((wd, 0.045, DOOR["z1"] - DOOR["z0"]), cen, R=Rz(yaw))
    A.g("glass").box((0.45, 0.05, 0.5), cen + (0, 0, 0.45), R=Rz(yaw))           # door window
    for nm, g in (("livery_full", full), ("livery_panel", panel), ("livery_stripes", stripes), ("livery_rear", rear)):
        tex = tex_path({"livery_full": LIVERY[0], "livery_panel": LIVERY[1], "livery_stripes": LIVERY[2],
                        "livery_rear": LIVERY[3]}[nm])
        m = looks.get(nm, rgb=(0.86, 0.87, 0.86), rough=0.35, tex=tex, clearcoat=0.6)
        n = emit(stage, f"{root}/Body/{nm}", g, m, (0.86, 0.87, 0.86))
        A.tris["Body"] = A.tris.get("Body", 0) + n


def _van_interior(stage, root, looks, A, screens):
    hw, f = CARGO["w"] / 2, FLOOR_Z
    x0, x1 = CARGO["x0"], CARGO["x1"]
    G = "Interior"
    A.g("floor", G).box((x1 - x0 - 0.14, CARGO["w"] - 0.14, 0.01), ((x0 + x1) / 2, 0, f - 0.003))
    A.g("interior", G).box((x1 - x0 - 0.14, 0.01, CARGO["z1"] - f - 0.1), ((x0 + x1) / 2, hw - 0.065, (f + CARGO["z1"]) / 2 - 0.03))
    A.g("interior", G).box((x1 - x0 - 0.14, CARGO["w"] - 0.14, 0.01), ((x0 + x1) / 2, 0, CARGO["z1"] - 0.065))
    A.g("ceiling_light", G).box((2.6, 0.16, 0.02), (-1.0, 0.0, CARGO["z1"] - 0.08))
    d = DESK
    A.g("desk", G).box((d["x1"] - d["x0"], d["y1"] - d["y0"], 0.04), ((d["x0"] + d["x1"]) / 2, (d["y0"] + d["y1"]) / 2, d["top"] - 0.02))
    for px in (d["x0"] + 0.26, d["x1"] - 0.26):                                 # pedestal drawers
        A.g("desk", G).box((0.5, 0.6, 0.72), (px, (d["y0"] + d["y1"]) / 2 + 0.02, f + 0.36))
        for k in range(3):
            A.g("alu", G).box((0.2, 0.01, 0.02), (px, d["y0"] - 0.0, f + 0.15 + 0.22 * k))
    A.g("desk", G).box((d["x1"] - d["x0"], 0.03, 0.55), ((d["x0"] + d["x1"]) / 2, d["y1"] - 0.03, f + 0.45))
    # monitors on a beam, angled towards the operator; screens are their own textured, emissive meshes
    beam_z = d["top"] + 0.05
    A.g("plastic", G).box((2.3, 0.06, 0.04), (-0.7, 1.08, beam_z + 0.35))
    A.g("plastic", G).box((0.10, 0.10, 0.4), (-0.7, 1.08, beam_z + 0.15))
    A.g("plastic", G).box((0.35, 0.25, 0.015), (-0.7, 1.05, d["top"] + 0.008))
    mons = [(-1.47, 0.97, 18.0, 0.62, 0.37), (-0.70, 1.02, 0.0, 0.72, 0.43), (0.07, 0.97, -18.0, 0.62, 0.37)]
    zc = d["top"] + 0.40
    for k, (mx, my, yaw, mw, mh) in enumerate(mons):
        R = Rz(yaw)
        c = np.array([mx, my, zc])
        A.g("plastic", G).box((mw + 0.03, 0.035, mh + 0.03), c, R=R)
        A.g("plastic", G).box((0.08, 0.06, 0.08), c + R @ (0, 0.05, 0), R=R)
        A.g("led_blue", G).box((0.012, 0.004, 0.006), c + R @ (mw / 2 - 0.02, -0.019, -mh / 2 - 0.006), R=R)
        e = -0.0185
        corners = [c + R @ np.array(v) for v in ((-mw / 2, e, -mh / 2), (mw / 2, e, -mh / 2), (mw / 2, e, mh / 2), (-mw / 2, e, mh / 2))]
        g = Geo().quad(*corners)
        src = screens[k] if k < len(screens) else None
        m = looks.get(f"screen{k}", rgb=(0.05, 0.08, 0.12), rough=0.15, tex=src, tex_emissive=2.5) if src else \
            looks.get(f"screen{k}", rgb=(0.05, 0.08, 0.12), rough=0.15, emissive=(0.1, 0.25, 0.45))
        n = emit(stage, f"{root}/{G}/Screen{k}", g, m, (0.25, 0.35, 0.45), label=f"Operator screen {k}")
        A.tris[G] = A.tris.get(G, 0) + n
    A.g("plastic", G).box((0.44, 0.15, 0.02), (-0.70, d["y0"] + 0.22, d["top"] + 0.01))        # keyboard
    A.g("plastic", G).box((0.06, 0.10, 0.025), (-0.38, d["y0"] + 0.22, d["top"] + 0.012))      # mouse
    A.g("case", G).box((0.30, 0.20, 0.10), (0.35, d["y0"] + 0.25, d["top"] + 0.05))            # radio base
    A.g("plastic", G).box((0.06, 0.05, 0.16), (0.48, d["y0"] + 0.20, d["top"] + 0.08), R=Ry(20))
    A.g("led_green", G).box((0.03, 0.005, 0.01), (0.30, d["y0"] + 0.148, d["top"] + 0.07))
    A.g("plastic", G).box((0.34, 0.24, 0.015), (-1.65, d["y0"] + 0.22, d["top"] + 0.008), R=Rz(10))   # laptop
    A.g("plastic", G).box((0.34, 0.015, 0.22), (-1.66, d["y0"] + 0.34, d["top"] + 0.12), R=Rz(10) @ Rx(-15))
    # comms rack at the rear: mesh radio, router, UPS; status LEDs
    rx_ = x0 + 0.42
    A.g("rack", G).box((0.62, 0.65, 1.9), (rx_, 0.75, f + 0.95))
    for k in range(8):
        zz = f + 0.35 + 0.19 * k
        A.g("plastic", G).box((0.012, 0.56, 0.15), (rx_ + 0.31, 0.75, zz))
        A.g("led_green" if k % 3 else "led_blue", G).box((0.006, 0.02, 0.012), (rx_ + 0.318, 0.52, zz + 0.03))
        A.g("led_green", G).box((0.006, 0.02, 0.012), (rx_ + 0.318, 0.56, zz + 0.03))
    # chair (operator faces +y, towards the screens)
    cx, cy = -0.70, 0.02
    for k in range(5):
        a = math.radians(90 + 72 * k)
        A.g("plastic", G).box((0.30, 0.04, 0.03), (cx + 0.15 * math.cos(a), cy + 0.15 * math.sin(a), f + 0.07), R=Rz(math.degrees(a)))
        A.g("plastic", G).sphere(0.025, (cx + 0.3 * math.cos(a), cy + 0.3 * math.sin(a), f + 0.025), n=8, m=5)
    A.g("alu", G).cyl(0.025, 0.36, (cx, cy, f + 0.26), n=10)
    A.g("fabric", G).box((0.48, 0.48, 0.07), (cx, cy, f + 0.445))
    A.g("fabric", G).box((0.46, 0.07, 0.55), (cx, cy - 0.27, f + 0.80), R=Rx(-8))
    A.g("plastic", G).box((0.05, 0.05, 0.3), (cx, cy - 0.27, f + 0.55))


def _van_roof(A, mast_h):
    z1 = CARGO["z1"]
    G = "Roof"
    for sy in (-1, 1):
        A.g("alu", G).box((4.6, 0.04, 0.04), (-1.0, sy * 1.02, z1 + 0.08))
        for xx in (-3.1, -1.0, 1.1):
            A.g("alu", G).box((0.04, 0.04, 0.08), (xx, sy * 1.02, z1 + 0.04))
    A.g("interior", G).box((0.9, 0.8, 0.22), (0.4, 0.2, z1 + 0.11))             # A/C unit
    A.g("trim", G).box((0.5, 0.5, 0.01), (0.4, 0.2, z1 + 0.225))
    # light bar on the cab roof: amber ends, white work lights in the middle (no red: red = radar ping)
    A.g("trim", G).box((0.30, 1.70, 0.06), (2.2, 0, 2.79))
    for sy in (-1, 1):
        A.g("amber_lamp", G).box((0.26, 0.48, 0.07), (2.2, sy * 0.58, 2.855))
        A.g("trim", G).box((0.06, 0.04, 0.08), (2.2, sy * 0.80, 2.73))
    A.g("white_lamp", G).box((0.26, 0.58, 0.07), (2.2, 0, 2.855))
    # Starlink-style flat-panel terminal on a short mast, tilted towards the sky
    dz = z1 + 0.05
    A.g("dish", G).cyl(0.035, 0.32, (-1.0, -0.55, dz + 0.16), n=10)
    A.g("dish", G).box((0.36, 0.30, 0.05), (-1.0, -0.55, dz + 0.02))
    Rd = Rz(-30) @ Ry(-22)
    A.g("dish", G).box((0.575, 0.511, 0.035), (-1.0, -0.55, dz + 0.40), R=Rd)
    A.g("trim", G).box((0.40, 0.30, 0.02), np.array([-1.0, -0.55, dz + 0.40]) + Rd @ (0, 0, -0.025), R=Rd)
    # telescopic mesh-radio mast, rear right corner
    mx, my = -2.9, 0.75
    A.g("trim", G).box((0.36, 0.36, 0.06), (mx, my, z1 + 0.03))
    seg = mast_h / 4 + 0.12                                                     # 4 nested sections, 12 cm overlap
    for k, r in enumerate((0.085, 0.068, 0.054, 0.042)):
        zb = z1 + 0.06 + k * (mast_h / 4)
        A.g("alu", G).cyl(r, seg, (mx, my, zb + seg / 2), n=14)
        A.g("trim", G).cyl(r + 0.012, 0.06, (mx, my, zb + seg - 0.03), n=14)
    zt = z1 + 0.06 + mast_h
    A.g("trim", G).box((0.06, 0.95, 0.05), (mx, my, zt))
    for yy, L in ((-0.42, 0.75), (0.0, 0.9), (0.42, 0.75)):                      # omni antennas
        A.g("plastic", G).cyl(0.014, L, (mx, my + yy, zt + 0.03 + L / 2), n=8)
        A.g("plastic", G).cyl(0.03, 0.08, (mx, my + yy, zt + 0.05), n=8)
    A.g("dish", G).box((0.09, 0.24, 0.55), (mx - 0.12, my, zt - 0.55), R=Rz(0))  # sector panel
    A.g("beacon_green", G).sphere(0.07, (mx, my, zt + 0.98), n=12, m=8)        # coordinator beacon (green)
    for gx, gy in ((-3.4, 1.15), (-3.4, -1.15), (-1.6, 1.15)):                  # guy lines from 2/3 height
        A.g("cable", G).cyl_between((mx, my, z1 + mast_h * 0.68), (gx, gy, z1 + 0.05), 0.006, n=5)


def _van_operator(A):
    """Seated operator mannequin (pose from isaac.dummies) facing the screens (+y)."""
    from isaac.dummies import POSES, mannequin
    M = T((-0.70, 0.05, FLOOR_Z), Rz(90))
    mannequin(A, POSES["seated"], M, group="Operator", mats=dict(skin="skin", cloth="uniform", boots="boots"),
              vest="hivis", headset="plastic", detail=1)


def _van_outside(A):
    G = "Outside"
    # rescue tent behind the van (van rear = -x), 3 x 3 m pop-up
    tx, ty = -6.6, -1.0
    h_e, h_p, s = 2.05, 2.8, 1.5
    corners = [(tx - s, ty - s), (tx + s, ty - s), (tx + s, ty + s), (tx - s, ty + s)]
    for cx_, cy_ in corners:
        A.g("alu", G).cyl(0.025, h_e, (cx_, cy_, h_e / 2), n=8)
        A.g("trim", G).box((0.12, 0.12, 0.02), (cx_, cy_, 0.01))
    peak = (tx, ty, h_p)
    for k in range(4):
        a, b = corners[k], corners[(k + 1) % 4]
        A.g("canopy", G).tri((a[0], a[1], h_e), (b[0], b[1], h_e), peak, double=True)
        A.g("canopy_trim", G).quad((a[0], a[1], h_e - 0.25), (b[0], b[1], h_e - 0.25), (b[0], b[1], h_e),
                                    (a[0], a[1], h_e), double=True)
    A.g("canopy", G).quad((tx - s, ty + s, 0.0), (tx + s, ty + s, 0.0), (tx + s, ty + s, h_e - 0.25),
                          (tx - s, ty + s, h_e - 0.25), double=True)             # back wall
    A.g("table", G).box((1.8, 0.75, 0.04), (tx, ty + 0.6, 0.74))
    for ddx in (-0.8, 0.8):
        for ddy in (-0.3, 0.3):
            A.g("alu", G).cyl(0.015, 0.72, (tx + ddx, ty + 0.6 + ddy, 0.36), n=6)
    A.g("case", G).box((0.5, 0.35, 0.2), (tx - 0.4, ty + 0.6, 0.86))
    A.g("kitbag", G).box((0.55, 0.3, 0.3), (tx + 0.45, ty + 0.62, 0.91))
    A.g("kitbag", G).box((0.6, 0.35, 0.35), (tx + 1.0, ty - 1.0, 0.175))
    A.g("case", G).box((0.55, 0.4, 0.3), (tx - 1.0, ty - 1.05, 0.15))
    for ddx in (-0.65, 0.65):                                                   # stretcher on trestles
        A.g("alu", G).box((0.06, 0.5, 0.04), (tx + ddx, ty - 0.35, 0.56))
        for ddy in (-0.2, 0.2):
            A.g("alu", G).cyl(0.015, 0.56, (tx + ddx, ty - 0.35 + ddy, 0.28), n=6)
    A.g("stretcher", G).box((1.85, 0.42, 0.05), (tx, ty - 0.35, 0.605))
    for ddx in (-0.6, 0.0, 0.6):
        A.g("trim", G).box((0.05, 0.44, 0.012), (tx + ddx, ty - 0.35, 0.634))
    # floodlight tripod at the tent corner, aimed at the work area
    fx, fy = tx + s + 0.4, ty - s - 0.2
    for k in range(3):
        a = math.radians(120 * k + 30)
        A.g("trim", G).cyl_between((fx + 0.45 * math.cos(a), fy + 0.45 * math.sin(a), 0), (fx, fy, 1.5), 0.015, n=6)
    A.g("alu", G).cyl(0.02, 1.0, (fx, fy, 2.0), n=8)
    Rf = Rz(30) @ Ry(-70)
    A.g("trim", G).box((0.10, 0.36, 0.26), (fx, fy, 2.55), R=Rf)
    A.g("flood", G).box((0.01, 0.30, 0.20), np.array([fx, fy, 2.55]) + Rf @ (0.055, 0, 0), R=Rf)
    # generator in a tube frame, beside the cab (+y side), with a fuel can
    gx, gy = 2.0, 3.6
    A.g("generator", G).box((0.85, 0.55, 0.52), (gx, gy, 0.36))
    A.g("trim", G).box((0.3, 0.02, 0.18), (gx + 0.15, gy - 0.28, 0.42))
    A.g("led_green", G).box((0.02, 0.006, 0.02), (gx + 0.24, gy - 0.292, 0.47))
    A.g("chassis", G).cyl(0.03, 0.18, (gx - 0.3, gy + 0.1, 0.70), n=8)
    fr = 0.02
    for ddx in (-0.48, 0.48):
        for ddy in (-0.32, 0.32):
            A.g("trim", G).cyl(fr, 0.68, (gx + ddx, gy + ddy, 0.34), n=6)
        A.g("trim", G).cyl_between((gx + ddx, gy - 0.32, 0.68), (gx + ddx, gy + 0.32, 0.68), fr, n=6)
        A.g("trim", G).cyl_between((gx + ddx, gy - 0.32, 0.02), (gx + ddx, gy + 0.32, 0.02), fr, n=6)
    for ddy in (-0.32, 0.32):
        A.g("trim", G).cyl_between((gx - 0.48, gy + ddy, 0.68), (gx + 0.48, gy + ddy, 0.68), fr, n=6)
    A.g("generator", G).box((0.18, 0.12, 0.30), (gx + 0.75, gy + 0.3, 0.15))
    A.g("trim", G).cyl(0.02, 0.05, (gx + 0.78, gy + 0.3, 0.32), n=6)
    # cable runs (generator -> van inlet, generator -> tent), lying on the ground with a little slack
    rng = np.random.default_rng(3)

    def run(pts, z=0.012):
        pts = np.asarray(pts, float)
        dense = []
        for a, b in zip(pts[:-1], pts[1:]):
            k = max(2, int(np.linalg.norm(b - a) / 0.6))
            for t in np.linspace(0, 1, k, endpoint=False):
                p = a + (b - a) * t
                dense.append((p[0] + rng.normal(0, 0.03), p[1] + rng.normal(0, 0.03), z))
        dense.append((*pts[-1], z))
        return dense
    A.g("cable", G).tube(run([(gx - 0.45, gy - 0.2), (1.2, 2.4), (1.0, 1.6), (0.9, 1.32)]) + [(0.9, 1.30, 0.75)], 0.014)
    A.g("cable", G).tube(run([(gx - 0.45, gy + 0.1), (0.0, 2.5), (-2.8, 2.2), (-4.6, 1.3), (-5.7, 0.2), (tx + 0.6, ty + 0.9)]), 0.014)
    A.g("protector", G).box((0.5, 0.9, 0.05), (-4.3, 1.55, 0.025), R=Rz(-30))
    A.g("trim", G).box((0.12, 0.92, 0.052), (-4.3, 1.55, 0.026), R=Rz(-30))
    A.g("trim", G).box((0.12, 0.12, 0.04), (0.9, 1.32, 0.75))                    # inlet box on the van wall
    # traffic cones along the pad edge (door side) and at the tent
    for cx_, cy_ in [(x, -4.6) for x in (-3.0, -1.5, 0.0, 1.5, 3.0)] + [(-8.6, -2.8), (-8.6, 0.8)]:
        A.g("trim", G).box((0.36, 0.36, 0.03), (cx_, cy_, 0.015))
        A.g("cone", G).cyl(0.15, 0.62, (cx_, cy_, 0.34), r_top=0.025, n=14)
        A.g("reflective", G).cyl(0.107, 0.12, (cx_, cy_, 0.36), r_top=0.088, n=14, caps=False)


def _van_lights(stage, root, k=1.0, mast_h=MAST_H):
    """Small lights so the interior and the work area read at dusk (RectLights emit along their local -z)."""
    from pxr import Gf, UsdLux
    specs = [("CeilingLight", (-1.0, 0.0, CARGO["z1"] - 0.10), (0, 0, 0), 2.6, 0.2, 1800.0, (1.0, 0.92, 0.80)),
             ("ScreenGlow", (-0.7, 0.88, DESK["top"] + 0.40), (-90, 0, 0), 2.1, 0.45, 600.0, (0.75, 0.85, 1.0)),
             ("Flood", (-6.6 + 1.9, -1.0 - 1.7, 2.55), (0, -20, 0), 0.3, 0.2, 9000.0, (1.0, 0.94, 0.85))]
    for name, t, rpy, w, h, inten, col in specs:
        path = f"{root}/Lights/{name}"
        L = UsdLux.RectLight.Define(stage, path)
        L.CreateWidthAttr(float(w)); L.CreateHeightAttr(float(h))
        L.CreateIntensityAttr(float(inten * k)); L.CreateColorAttr(Gf.Vec3f(*col))
        R = Rz(rpy[2]) @ Ry(rpy[1]) @ Rx(rpy[0])
        xform(stage, path, t, R=R)


# =============================================================================================== preview CLI
def _world_tris(stage, root):
    """(tri vertices (n,3,3) world, rgb (n,3)) of every visible Mesh under root (for the projection sheets)."""
    from pxr import Usd, UsdGeom
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    out_t, out_c = [], []
    prim = stage.GetPrimAtPath(root)
    for p in Usd.PrimRange(prim):
        if not p.IsA(UsdGeom.Mesh):
            continue
        if UsdGeom.Imageable(p).ComputeVisibility() == UsdGeom.Tokens.invisible:
            continue
        m = UsdGeom.Mesh(p)
        P = np.asarray(m.GetPointsAttr().Get(), float)
        idx = np.asarray(m.GetFaceVertexIndicesAttr().Get(), np.int64)
        cnt = np.asarray(m.GetFaceVertexCountsAttr().Get(), np.int64)
        if len(P) == 0 or not np.all(cnt == 3):
            continue
        W = np.asarray(cache.GetLocalToWorldTransform(p), float)           # row-vector convention
        Pw = np.c_[P, np.ones(len(P))] @ W
        col = m.GetDisplayColorAttr().Get()
        rgb = np.asarray(col[0] if col else (0.6, 0.6, 0.6), float)
        out_t.append(Pw[:, :3][idx.reshape(-1, 3)])
        out_c.append(np.repeat(rgb[None], len(idx) // 3, 0))
    if not out_t:
        return np.zeros((0, 3, 3)), np.zeros((0, 3))
    return np.concatenate(out_t), np.concatenate(out_c)


def project(ax, tris, rgb, view="top", title="", light=(0.4, -0.5, 0.75), bounds=None):
    """Painter's-algorithm projection of triangles onto a matplotlib axis (top / side / front)."""
    from matplotlib.collections import PolyCollection
    if not len(tris):
        ax.set_title(title + " (empty)"); return
    if bounds is not None:
        lo, hi = np.asarray(bounds[0]), np.asarray(bounds[1])
        c = tris.mean(1)
        keep = np.all((c >= lo) & (c <= hi), 1)
        tris, rgb = tris[keep], rgb[keep]
    ij, depth_axis, sgn = {"top": ((0, 1), 2, 1), "side": ((0, 2), 1, -1), "front": ((1, 2), 0, 1),
                           "back": ((1, 2), 0, -1)}[view]
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    L = np.asarray(light, float); L /= np.linalg.norm(L)
    shade = 0.35 + 0.65 * np.abs(n @ L)
    col = np.clip(rgb * shade[:, None], 0, 1)
    order = np.argsort(sgn * tris[:, :, depth_axis].mean(1))
    poly = tris[order][:, :, list(ij)]
    if view == "back":
        poly[:, :, 0] *= -1
    ax.add_collection(PolyCollection(poly, facecolors=col[order], edgecolors="none", antialiased=False))
    ax.autoscale_view(); ax.set_aspect("equal"); ax.set_title(title, fontsize=9)
    ax.tick_params(labelsize=7)


def build_preview_stages(out_dir, sites=("grid", "collapse"), seed=7, n_drones=3, log=print):
    """Asset test stages: van at the staging pose, dummies (manifest), 3 heavy drones at the pads, one pod
    lowered. Saves <out_dir>/assets_<site>.usda. Returns {site: (stage, info)}."""
    from pxr import Usd, UsdGeom

    from isaac import dummies as DM
    from isaac.drone_viz import DroneViz
    from isaac.pods import PodViz
    from isaac.site import drone_pads
    from sim.scenario import make_scenario
    os.makedirs(out_dir, exist_ok=True)
    res = {}
    manifest = DM.load_manifest()
    for site in sites:
        sc = make_scenario(seed, site=site) if site != "grid" else make_scenario(seed)
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
        x, y, z, yaw = staging_pose(sc)
        vinfo = build_van(stage, "/World/Van", (x, y), yaw, z=z)
        dinfo = DM.build_dummies(stage, sc, manifest)
        ids = [f"d{i + 1}" for i in range(n_drones)]
        drones = {}
        for i, did in enumerate(ids):
            px, py = drone_pads(sc)[i % len(drone_pads(sc))]
            d = DroneViz(stage, f"/World/Drones/{did}", did=did)
            gz = float(sc.ground_height(px, py))
            d.set_pose(px, py, gz + (0.15 if i else 6.0), 90.0)
            drones[did] = d
        pods = PodViz(stage, ids)
        poses = {did: tuple(map(float, UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/Drones/{did}")).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation())) for did in ids}
        pods.update({"drones": [{"id": "d1", "winch_len_m": 3.0}], "pods": [{"drone_id": "d1", "status": "lowering"}]}, poses)
        path = os.path.join(out_dir, f"assets_{site}.usda")
        stage.GetRootLayer().Export(path)
        info = {"van": vinfo, "dummies": dinfo, "drone_tris": count_tris(stage, "/World/Drones/d1"),
                "pod_tris": count_tris(stage, "/World/Pods/d1"), "usda": path,
                "dummy_tris": count_tris(stage, "/World/Dummies")}
        log(f"[assets] {site}: van {vinfo['total']} tris, dummies {info['dummy_tris']} tris "
            f"({len(dinfo['dummies'])} prims), drone {info['drone_tris']} tris each, pod {info['pod_tris']} -> {path}")
        res[site] = (stage, info)
    return res


def preview_sheets(stages, img_dir=os.path.join(REPO, "docs", "img"), log=print):
    """Write the projection sanity images: van (top/side/back), heavy drone (top/side/front), dummies."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from pxr import Usd, UsdGeom
    os.makedirs(img_dir, exist_ok=True)
    out = []
    stage, info = stages.get("collapse") or next(iter(stages.values()))
    vt, vc = _world_tris(stage, "/World/Van")
    vt = vt - np.array(UsdGeom.Xformable(stage.GetPrimAtPath("/World/Van")).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation())
    Rinv = Rz(90.0)                                     # undo the -90 deg staging yaw: van frame for the sheet
    vt = vt @ Rinv.T
    fig, axs = plt.subplots(1, 3, figsize=(17, 6.2), gridspec_kw=dict(width_ratios=[1.25, 1.25, 0.7]))
    project(axs[0], vt, vc, "top", "Command van + tent + generator: top (x front, door -y)")
    project(axs[1], vt, vc, "side", "Side, door side (-y), mast extended",
            bounds=((-4.2, -9, -1), (4.5, 9, 12)))
    inside = ((-3.4, -1.15, 0.95), (1.5, 1.2, 3.36))
    project(axs[2], vt, vc, "side", "Interior cut (roof + walls off)", bounds=(inside[0], (inside[1][0], 1.18, inside[1][2])))
    fig.suptitle(f"isaac/van.py, {info['van']['total']} triangles (budget {TRI_BUDGET})", fontsize=11)
    fig.tight_layout()
    p = os.path.join(img_dir, "isaac_assets_van.png"); fig.savefig(p, dpi=110); plt.close(fig); out.append(p)

    dt, dc = _world_tris(stage, "/World/Drones/d2")
    dt = dt - np.array(UsdGeom.Xformable(stage.GetPrimAtPath("/World/Drones/d2")).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation())
    dt = dt @ Rz(-90).T
    from isaac.drone_viz import component_anchors
    anchors = component_anchors()
    fig, axs = plt.subplots(1, 3, figsize=(18, 6.4))
    for ax, view, ttl in ((axs[0], "top", "Heavy drone: top (+x forward)"), (axs[1], "side", "Side (pod stowed)"),
                          (axs[2], "front", "Front (LiDAR on 45 deg mount)")):
        project(ax, dt, dc, view, ttl)
        ij = {"top": (0, 1), "side": (0, 2), "front": (1, 2)}[view]
        for a in anchors:
            p_ = np.asarray(a["xyz"]) + (0, 0, -0.15)
            ax.plot(p_[ij[0]], p_[ij[1]], "o", ms=3, color="#e0218a")
            if view == "side" or (view == "top" and a["key"] in ("lidar", "jetson", "gnss", "mesh_radio", "speaker", "mic_array")):
                ax.annotate(a["key"], (p_[ij[0]], p_[ij[1]]), fontsize=6.5, color="#9d1463", xytext=(4, 3),
                            textcoords="offset points")
    fig.suptitle(f"isaac/drone_viz.py heavy_sar, {info['drone_tris']} triangles per drone; magenta = component_anchors()", fontsize=11)
    fig.tight_layout()
    p = os.path.join(img_dir, "isaac_assets_drone.png"); fig.savefig(p, dpi=110); plt.close(fig); out.append(p)

    from isaac import dummies as DM
    ds = DM.read_dummies(stage)
    st_g = stages.get("grid", (stage, info))[0]
    dg = DM.read_dummies(st_g)
    fig = plt.figure(figsize=(18, 7.2))
    picks = [d for d in dg if d["kind"] in ("voice", "weak_voice", "knock", "silent")][:4]
    picks += [d for d in dg if d.get("hidden")][:1] + [d for d in dg if d["kind"] == "dog"][:1]
    for k, d in enumerate(picks):
        ax = fig.add_subplot(2, len(picks), k + 1)
        t, c = _world_tris(st_g, d["prim"])
        t = t - np.asarray(d["xyz"])
        project(ax, t, c, "top", f"{d['id']} ({d['kind']}, {d['physio'] or '-'}){' HIDDEN' if d['hidden'] else ''}")
        ax = fig.add_subplot(2, len(picks), len(picks) + k + 1)
        project(ax, t, c, "side", f"side, pose {d['pose']}")
    fig.suptitle(f"isaac/dummies.py: grid seed 7 ({len(dg)} prims) and collapse seed 7 ({len(ds)} prims); "
                 "body + local rubble cover (visual only)", fontsize=11)
    fig.tight_layout()
    p = os.path.join(img_dir, "isaac_assets_dummies.png"); fig.savefig(p, dpi=105); plt.close(fig); out.append(p)
    for q in out:
        log(f"[assets] wrote {q}")
    return out


def _main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description="Build the Isaac asset test stages and projection sheets (no GPU).")
    ap.add_argument("--out", default="/tmp/larpverse_assets")
    ap.add_argument("--img", default=os.path.join(REPO, "docs", "img"))
    ap.add_argument("--textures", action="store_true", help="(re)generate assets/textures first")
    ap.add_argument("--no-sheets", action="store_true")
    a = ap.parse_args(argv)
    if a.textures:
        print(json.dumps(make_textures(force=True), indent=1))
    st = build_preview_stages(a.out)
    if not a.no_sheets:
        preview_sheets(st, a.img)


if __name__ == "__main__":
    _main()
