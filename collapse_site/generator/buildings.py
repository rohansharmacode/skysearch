"""Building geometry: scanned core buildings (photo roofs) + procedural wrecked neighbourhood.
Produces build/buildings.pkl : list of dict(name, material, V, F, UV, N, xform)"""
import json, pickle, numpy as np, mapbox_earcut as earcut
import sitecfg as S

LIB = json.load(open('build/texlib.json'))
STOREY = 3.1


class MeshBuf:
    def __init__(self):
        self.V, self.F, self.UV, self.N = [], [], [], []
        self.n = 0

    def quad(self, p0, p1, p2, p3, uv):
        """p0..p3 CCW (seen from outside), uv list of 4"""
        p = np.array([p0, p1, p2, p3], float)
        nrm = np.cross(p[1] - p[0], p[3] - p[0]);
        if np.linalg.norm(nrm) < 1e-9:
            nrm = np.cross(p[2] - p[1], p[0] - p[1])
        nrm = nrm / (np.linalg.norm(nrm) + 1e-12)
        self.V += list(p); self.UV += list(uv); self.N += [nrm] * 4
        b = self.n; self.F += [(b, b + 1, b + 2), (b, b + 2, b + 3)]; self.n += 4

    def tri(self, p0, p1, p2, uv):
        p = np.array([p0, p1, p2], float)
        nrm = np.cross(p[1] - p[0], p[2] - p[0]); nrm /= (np.linalg.norm(nrm) + 1e-12)
        self.V += list(p); self.UV += list(uv); self.N += [nrm] * 3
        b = self.n; self.F += [(b, b + 1, b + 2)]; self.n += 3

    def poly_top(self, pts2, z, uvfun, up=True):
        pts2 = np.asarray(pts2, float)
        idx = earcut.triangulate_float64(pts2, np.array([len(pts2)], np.uint32)).reshape(-1, 3)
        for t in idx:
            a, b, c = pts2[t[0]], pts2[t[1]], pts2[t[2]]
            # earcut keeps input winding; force CCW for +z
            cr = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
            if (cr < 0) == up:
                b, c = c, b
            zz = z if np.isscalar(z) else None
            P3 = [np.array([q[0], q[1], zz if zz is not None else z(q)]) for q in (a, b, c)]
            self.tri(*P3, [uvfun(q) for q in (a, b, c)])

    def empty(self):
        return self.n == 0

    def arrays(self):
        return (np.array(self.V, np.float32), np.array(self.F, np.int32),
                np.array(self.UV, np.float32), np.array(self.N, np.float32))


def wall_strip(mb, a, b, z0, z1, u0, tile):
    """vertical quad from 2D point a to b (outward normal = right-hand side when walking a->b CCW footprint)"""
    L = np.hypot(*(np.asarray(b) - a))
    tw, th = tile
    uv = [(u0 / tw, z0 / th), ((u0 + L) / tw, z0 / th), ((u0 + L) / tw, z1 / th), (u0 / tw, z1 / th)]
    mb.quad((a[0], a[1], z0), (b[0], b[1], z0), (b[0], b[1], z1), (a[0], a[1], z1), uv)
    return u0 + L


def ccw(pts):
    pts = np.asarray(pts, float)
    a = 0.5 * np.sum(pts[:, 0] * np.roll(pts[:, 1], -1) - np.roll(pts[:, 0], -1) * pts[:, 1])
    return pts if a > 0 else pts[::-1]


OUT = []


def emit(name, mat, mb, xform=None, collide=True):
    if mb.empty():
        return
    V, F, UV, N = mb.arrays()
    OUT.append(dict(name=name, material=mat, V=V, F=F, UV=UV, N=N, xform=xform, collide=collide))


# ------------------------------------------------------------------ core (scanned) buildings
def core_buildings():
    W, Hh = LIB['scan_roofs']['px']
    def roof_uv(q):
        u, v = S.w2px(q[0], q[1])
        return (float(u) / W, 1.0 - float(v) / Hh)
    for name, poly, h, wallmat, style in S.CORE_BUILDINGS:
        pts = ccw(S.poly_w(poly))
        walls = MeshBuf(); roof = MeshBuf()
        tile = LIB[wallmat]['size'] if wallmat in LIB else LIB[LIB[wallmat]['src']]['size']
        u = 0.0
        ztop = lambda q: h
        if style == 'gable_x':
            ymid = pts[:, 1].mean(); half = (pts[:, 1].max() - pts[:, 1].min()) / 2
            ztop = lambda q: h + 1.8 * (1 - abs(q[1] - ymid) / half)
        for i in range(len(pts)):
            a, b = pts[i], pts[(i + 1) % len(pts)]
            if style == 'gable_x':
                # wall up to eave, plus gable triangle on short (x-facing) ends
                u = wall_strip(walls, a, b, -0.4, h, u, tile)
                za, zb = ztop(a), ztop(b)
                if abs(a[0] - b[0]) < 0.5:   # east/west gable ends
                    ym = ymid
                    m = np.array([a[0] + (b[0] - a[0]) * (ym - a[1]) / (b[1] - a[1] + 1e-9), ym])
                    walls.tri((a[0], a[1], h), (b[0], b[1], h), (m[0], m[1], h + 1.8),
                              [(0, h / tile[1]), (1, h / tile[1]), (0.5, (h + 1.8) / tile[1])])
            else:
                u = wall_strip(walls, a, b, -0.4, h, u, tile)
        if style == 'gable_x':
            ymid = pts[:, 1].mean()
            x0, x1 = pts[:, 0].min(), pts[:, 0].max(); y0, y1 = pts[:, 1].min(), pts[:, 1].max()
            for (ya, yb) in [(y0, ymid), (ymid, y1)]:
                q = [(x0, ya), (x1, ya), (x1, yb), (x0, yb)]
                P3 = [(x, y, ztop((x, y))) for x, y in q]
                roof.quad(*P3, [roof_uv(p) for p in q])
            # overhang eaves
        else:
            roof.poly_top(pts, h, roof_uv)
            # parapet lip (thin) so roofs read as solid blocks
        emit(f'core_{name}_walls', 'facade_B' if wallmat == 'facade_tower' else wallmat, walls)
        emit(f'core_{name}_roof', 'scan_roofs', roof)


# ------------------------------------------------------------------ fallen towers + floor plates (solid)
def scan_uv(q):
    W, Hh = LIB['scan_roofs']['px']
    u, v = S.w2px(q[0], q[1])
    return (float(u) / W, 1.0 - float(v) / Hh)


def fallen_towers():
    zW = 1.2
    for name, y0, y1, xw, xr, xe, hh in S.SLABS:
        top, bot = [], []
        for k, yy in enumerate((y0, y1)):
            XW, _ = S.px2w(xw[k], yy); XR, Y_ = S.px2w(xr[k], yy); XE, _ = S.px2w(xe[k], yy)
            zR = zW + (XR - XW); zE = zR - (XE - XR)
            XB = XW + (XE - XR); zB = zW + (zE - zR)
            sec = [(float(XW), float(Y_), zW), (float(XR), float(Y_), zR), (float(XE), float(Y_), zE), (float(XB), float(Y_), zB)]
            (top if k == 0 else bot).append(sec)
        A = top[0]; B = bot[0]     # A at north end (y0 = smaller v = larger y), B at south end
        photo, side, ends = MeshBuf(), MeshBuf(), MeshBuf()
        # long faces: W->R (window face), R->E (balcony face) photo-mapped; E->B and B->W hidden-ish facades
        for i in range(4):
            a0, a1 = A[i], A[(i + 1) % 4]; b0, b1 = B[i], B[(i + 1) % 4]
            quad = (b0, b1, a1, a0)     # outward when walking north->south... verify via normal below
            pts = np.array(quad)
            nrm = np.cross(pts[1] - pts[0], pts[3] - pts[0])
            cen = np.mean(np.array(A + B), 0)
            if np.dot(nrm, pts.mean(0) - cen) < 0:
                quad = quad[::-1]
            if i in (0, 1):
                photo.quad(*quad, [scan_uv(p) for p in quad])
            else:
                tile = LIB['facade_A']['size']
                L = np.linalg.norm(np.array(a1) - np.array(a0)); Lz = np.linalg.norm(np.array(b0) - np.array(a0))
                uvq = [(0, 0), (L / tile[0], 0), (L / tile[0], Lz / tile[1]), (0, Lz / tile[1])]
                side.quad(*quad, uvq)
        ct = LIB['concrete']['size']
        for sec, flip in ((A, False), (B, True)):
            pts = sec[::-1] if not flip else sec
            pts = list(pts)
            cen = np.mean(np.array(A + B), 0)
            nrm = np.cross(np.array(pts[1]) - pts[0], np.array(pts[2]) - pts[0])
            if np.dot(nrm, np.mean(np.array(pts), 0) - cen) < 0:
                pts = pts[::-1]
            ends.quad(*pts, [(p[0] / ct[0], p[2] / ct[1]) for p in pts])
        emit(f'core_{name}_scanfaces', 'scan_roofs', photo)
        emit(f'core_{name}_facades', 'facade_A', side)
        emit(f'core_{name}_breaks', 'rubble', ends)


def floor_plates():
    th = 0.55
    for name, poly, h0, h1 in S.PLATES:
        pw = ccw(S.poly_w(poly))
        pw_orig = S.poly_w(poly)
        a = (pw_orig[0] + pw_orig[1]) / 2; b = (pw_orig[2] + pw_orig[3]) / 2
        d = b - a; L2 = float((d ** 2).sum())
        zf = lambda q: h0 + (h1 - h0) * float(np.clip(((q[0] - a[0]) * d[0] + (q[1] - a[1]) * d[1]) / L2, 0, 1))
        top, sides = MeshBuf(), MeshBuf()
        top.poly_top(pw, zf, scan_uv)
        ct = LIB['concrete']['size']
        u = 0.0
        for i in range(len(pw)):
            p, q = pw[i], pw[(i + 1) % len(pw)]
            L = float(np.hypot(*(q - p)))
            zp, zq = zf(p), zf(q)
            sides.quad((p[0], p[1], zp - th - 2.5), (q[0], q[1], zq - th - 2.5), (q[0], q[1], zq), (p[0], p[1], zp),
                       [(u / ct[0], 0), ((u + L) / ct[0], 0), ((u + L) / ct[0], 1), (u / ct[0], 1)])
            u += L
        emit(f'core_{name}_top', 'scan_roofs', top)
        emit(f'core_{name}_edges', 'concrete', sides)


# ------------------------------------------------------------------ outer procedural buildings
FACADES = ['facade_A', 'facade_B', 'facade_A_warm', 'facade_B_tile', 'facade_A_pale', 'facade_A', 'facade_B',
           'plaster_beige', 'plaster_pink', 'facade_balcony']
ROOFS = ['roof_concrete', 'roof_concrete', 'roof_green', 'roof_red', 'roof_white', 'roof_metal']


def local_frame(rect, yaw):
    x0, y0, x1, y1 = rect
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    return cx, cy, (x1 - x0), (y1 - y0), yaw


def column_building(fid, f, rng, hb_storeys, damage):
    """Grid-of-columns building with a broken, stepped damage profile. Built in local coords centred at 0."""
    cx, cy, w, d, yaw = local_frame(f['rect'], f.get('yaw', 0.0))
    cell = 3.0
    nx, ny = max(2, int(round(w / cell))), max(2, int(round(d / cell)))
    xs = np.linspace(-w / 2, w / 2, nx + 1); ys = np.linspace(-d / 2, d / 2, ny + 1)
    Hb = hb_storeys * STOREY + 0.5
    hgt = np.full((nx, ny), Hb)
    # damage: 1-3 'bites' where floors failed
    for k in range(1 + int(damage * 4)):
        ex, ey = rng.uniform(-w / 2, w / 2), rng.uniform(-d / 2, d / 2)
        if rng.random() < 0.6:   # bias to corners/edges
            ex = np.sign(ex) * w / 2; ey = ey if rng.random() < 0.5 else np.sign(ey) * d / 2
        R = rng.uniform(3, 4 + 10 * damage)
        depth = rng.uniform(0.3, 1.0) * Hb
        for i in range(nx):
            for j in range(ny):
                c = np.hypot((xs[i] + xs[i + 1]) / 2 - ex, (ys[j] + ys[j + 1]) / 2 - ey)
                if c < R:
                    cut = depth * (1 - (c / R) ** 2)
                    hgt[i, j] = min(hgt[i, j], Hb - cut)
    # quantise to broken floor levels, jitter
    hgt = np.where(hgt < Hb - 0.01, np.maximum(0.6, np.floor(hgt / STOREY) * STOREY + rng.uniform(-0.6, 0.8, hgt.shape)), Hb)
    fac = FACADES[rng.integers(len(FACADES))]; roofm = ROOFS[rng.integers(len(ROOFS))]
    ftile = LIB[fac]['size']; rtile = LIB[roofm]['size']
    walls, roof, broken = MeshBuf(), MeshBuf(), MeshBuf()
    uoff = rng.uniform(0, 10)
    rflip = rng.integers(4)

    ro = rng.uniform(0, 1, 2)
    def roof_uv(x, y):
        u, v = x / rtile[0] + ro[0], y / rtile[1] + ro[1]
        for _ in range(rflip):
            u, v = v, -u
        return (u, v)
    btile = LIB['rubble']['size']
    for i in range(nx):
        for j in range(ny):
            x0, x1, y0, y1 = xs[i], xs[i + 1], ys[j], ys[j + 1]
            z = hgt[i, j]
            q = [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]
            if z >= Hb - 0.01:
                roof.quad(*q, [roof_uv(x0, y0), roof_uv(x1, y0), roof_uv(x1, y1), roof_uv(x0, y1)])
            else:
                broken.quad(*q, [(x0 / btile[0], y0 / btile[1]), (x1 / btile[0], y0 / btile[1]),
                                 (x1 / btile[0], y1 / btile[1]), (x0 / btile[0], y1 / btile[1])])
            # side faces: 4 directions
            for (di, dj, a, b) in [(1, 0, (x1, y0), (x1, y1)), (-1, 0, (x0, y1), (x0, y0)),
                                   (0, 1, (x1, y1), (x0, y1)), (0, -1, (x0, y0), (x1, y0))]:
                ii, jj = i + di, j + dj
                outer = not (0 <= ii < nx and 0 <= jj < ny)
                zn = -0.4 if outer else hgt[ii, jj]
                if zn >= z - 1e-3:
                    continue
                if outer:
                    # u coordinate continuous along each facade side
                    if di != 0:
                        u0 = (a[1] + d / 2) if di == 1 else (d / 2 - a[1])
                    else:
                        u0 = (w / 2 - a[0]) if dj == 1 else (a[0] + w / 2)
                    wall_strip(walls, a, b, zn, z, u0 + uoff, ftile)
                else:
                    L = np.hypot(b[0] - a[0], b[1] - a[1])
                    uu = (a[0] + a[1]) / btile[0]
                    broken.quad((a[0], a[1], zn), (b[0], b[1], zn), (b[0], b[1], z), (a[0], a[1], z),
                                [(uu, zn / btile[1]), (uu + L / btile[0], zn / btile[1]), (uu + L / btile[0], z / btile[1]), (uu, z / btile[1])])
    return walls, roof, broken, fac, roofm, Hb


def xform_for(f, extra_rot=None, sink=0.0):
    cx, cy, w, d, yaw = local_frame(f['rect'], f.get('yaw', 0.0))
    return dict(translate=(cx, cy, -sink), yaw=yaw, tilt=extra_rot)


def pancake(fid, f, rng):
    cx, cy, w, d, yaw = local_frame(f['rect'], f.get('yaw', 0.0))
    n = f['floors']
    slabs, roof, inner = MeshBuf(), MeshBuf(), MeshBuf()
    roofm = ROOFS[rng.integers(len(ROOFS))]
    ctile = LIB['concrete']['size']
    z = 0.0
    boxes = []
    for k in range(n):
        gap = rng.uniform(0.35, 1.3) if k > 0 else rng.uniform(0.2, 0.8)
        z += gap
        th = 0.4
        ox, oy = rng.normal(0, 0.35, 2) * (k / n + 0.3)
        ax, ay = rng.normal(0, 2.0, 2) * (0.3 + 0.7 * f['damage'])   # tilt deg
        shrink = rng.uniform(0, 0.8)
        boxes.append((ox, oy, w - shrink, d - shrink * rng.uniform(0.5, 1.5), z, th, ax, ay))
        z += th
    for k, (ox, oy, bw, bd, bz, th, ax, ay) in enumerate(boxes):
        top = k == len(boxes) - 1
        corners = np.array([[-bw / 2, -bd / 2], [bw / 2, -bd / 2], [bw / 2, bd / 2], [-bw / 2, bd / 2]])
        def zf(p, base):
            return base + np.tan(np.deg2rad(ax)) * p[0] + np.tan(np.deg2rad(ay)) * p[1]
        P0 = [(c[0] + ox, c[1] + oy, zf(c, bz)) for c in corners]
        P1 = [(c[0] + ox, c[1] + oy, zf(c, bz + th)) for c in corners]
        uvt = [((c[0] + ox) / ctile[0], (c[1] + oy) / ctile[1]) for c in corners]
        if top:
            rt = LIB[roofm]['size']
            ru = [((c[0] + ox) / rt[0], (c[1] + oy) / rt[1]) for c in corners]
            roof.quad(*P1, ru)
        else:
            slabs.quad(*P1, uvt)
        slabs.quad(P0[3], P0[2], P0[1], P0[0], uvt[::-1])
        for i in range(4):
            a0, b0, b1, a1 = P0[i], P0[(i + 1) % 4], P1[(i + 1) % 4], P1[i]
            L = np.hypot(b0[0] - a0[0], b0[1] - a0[1])
            slabs.quad(a0, b0, b1, a1, [(0, 0), (L / ctile[0], 0), (L / ctile[0], th / ctile[1]), (0, th / ctile[1])])
    # dark crushed interior core so the stack doesn't read as floating plates
    iw, idd = w * 0.82, d * 0.82
    zt = boxes[-1][4]
    for (a, b) in [((-iw / 2, -idd / 2), (iw / 2, -idd / 2)), ((iw / 2, -idd / 2), (iw / 2, idd / 2)),
                   ((iw / 2, idd / 2), (-iw / 2, idd / 2)), ((-iw / 2, idd / 2), (-iw / 2, -idd / 2))]:
        wall_strip(inner, a, b, -0.3, zt, 0, LIB['rubble']['size'])
    return slabs, roof, inner, roofm


def wall_remnants(fid, f, rng):
    x0, y0, x1, y1 = f['rect']
    out = []
    fac = FACADES[rng.integers(len(FACADES))]
    for k in range(f.get('walls', 0)):
        side = rng.integers(4)
        L = rng.uniform(3, 9); Hh = rng.uniform(2.5, 8.0); th = 0.3
        if side < 2:
            y = y0 + 0.5 if side == 0 else y1 - 0.5
            xa = rng.uniform(x0, max(x0 + 0.1, x1 - L)); a, b = (xa, y), (xa + L, y)
        else:
            x = x0 + 0.5 if side == 2 else x1 - 0.5
            ya = rng.uniform(y0, max(y0 + 0.1, y1 - L)); a, b = (x, ya), (x, ya + L)
        mb, mc = MeshBuf(), MeshBuf()
        a, b = np.array(a), np.array(b)
        dvec = (b - a) / np.linalg.norm(b - a); nvec = np.array([dvec[1], -dvec[0]]) * th / 2
        # jagged top: several segments with different heights
        segs = int(max(2, L / 1.5)); tile = LIB[fac]['size']
        for s in range(segs):
            p = a + (b - a) * s / segs; q = a + (b - a) * (s + 1) / segs
            hz = Hh * rng.uniform(0.45, 1.0) if s not in (0, segs - 1) else Hh * rng.uniform(0.2, 0.7)
            wall_strip(mb, p + nvec, q + nvec, -0.5, hz, s * L / segs, tile)
            wall_strip(mc, q - nvec, p - nvec, -0.5, hz, s * L / segs, LIB['concrete']['size'])
            mc.quad((p[0] + nvec[0], p[1] + nvec[1], hz), (q[0] + nvec[0], q[1] + nvec[1], hz),
                    (q[0] - nvec[0], q[1] - nvec[1], hz), (p[0] - nvec[0], p[1] - nvec[1], hz),
                    [(0, 0), (1, 0), (1, 0.1), (0, 0.1)])
        tilt = (float(rng.uniform(-12, 12)), float(rng.uniform(-6, 6)))
        piv = ((a + b) / 2).tolist()
        out.append((mb, mc, fac, dict(pivot=piv, tilt=tilt)))
    return out


def outer():
    lay = json.load(open('build/outer_layout_final.json'))
    for fid, f in enumerate(lay['features']):
        rng = np.random.default_rng(f['seed'])
        t = f['type']
        nm = f'{t}_{fid:03d}'
        if t in ('standing', 'tilted'):
            walls, roof, broken, fac, roofm, Hb = column_building(fid, f, rng, f['storeys'], f['damage'])
            tilt = None; sink = 0.0
            if t == 'tilted':
                ang = f['tilt_deg']; dd = f['tilt_dir']
                cx, cy, w, d, yaw = local_frame(f['rect'], 0)
                axis = [(0, 1), (0, -1), (-1, 0), (1, 0)][dd]
                edge = [(w / 2, 0), (-w / 2, 0), (0, d / 2), (0, -d / 2)][dd]
                tilt = dict(axis=axis, deg=ang, pivot=edge)
                sink = f['sink']
            xf = xform_for(f, tilt, sink)
            emit(f'{nm}_walls', fac, walls, xf); emit(f'{nm}_roof', roofm, roof, xf); emit(f'{nm}_broken', 'rubble', broken, xf)
        elif t == 'pancake':
            slabs, roof, inner, roofm = pancake(fid, f, rng)
            xf = xform_for(f)
            emit(f'{nm}_slabs', 'concrete', slabs, xf); emit(f'{nm}_roof', roofm, roof, xf); emit(f'{nm}_core', 'rubble', inner, xf)
        elif t == 'collapsed':
            for k, (mb, mc, fac, xf) in enumerate(wall_remnants(fid, f, rng)):
                xfd = dict(translate=(0, 0, 0), yaw=0.0, wall_tilt=xf)
                emit(f'{nm}_wall{k}_face', fac, mb, xfd); emit(f'{nm}_wall{k}_back', 'concrete', mc, xfd)


if __name__ == '__main__':
    core_buildings()
    fallen_towers()
    floor_plates()
    outer()
    pickle.dump(OUT, open('build/buildings.pkl', 'wb'))
    nt = sum(len(o['F']) for o in OUT)
    print(len(OUT), 'mesh prims,', nt, 'triangles')
