"""Build the global heightfield (0.125 m grid over 320 m) + class maps."""
import json, numpy as np, cv2
import sitecfg as S

N = S.HN
rng = np.random.default_rng(5)
xs = (-S.EXTENT + np.arange(N) * S.HRES).astype(np.float32)
ys = (S.EXTENT - np.arange(N) * S.HRES).astype(np.float32)
X, Y = np.meshgrid(xs, ys)


def value_noise(shape, cell, seed, octaves=4, persistence=0.5):
    """fBm value noise in [0,1]; cell = feature size in samples for the first octave."""
    r = np.random.default_rng(seed)
    out = np.zeros(shape, np.float32); amp = 1.0; tot = 0.0
    for o in range(octaves):
        c = max(2, int(cell / (2 ** o)))
        g = r.random((shape[0] // c + 3, shape[1] // c + 3)).astype(np.float32)
        up = cv2.resize(g, (g.shape[1] * c, g.shape[0] * c), interpolation=cv2.INTER_CUBIC)
        ox, oy = r.integers(0, c, 2)
        out += amp * up[oy:oy + shape[0], ox:ox + shape[1]]
        tot += amp; amp *= persistence
    out /= tot
    lo, hi = np.percentile(out[::7, ::7], [1, 99])
    return np.clip((out - lo) / (hi - lo + 1e-6), 0, 1)


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0, 1)
    return t * t * (3 - 2 * t)


def poly_mask_w(poly_w, blur_m=0.0):
    """rasterise a world-space polygon onto the heightfield grid (float mask)."""
    c, r = S.w2h(poly_w[:, 0], poly_w[:, 1])
    pts = np.round(np.stack([c, r], 1) * 4).astype(np.int32)
    m = np.zeros((N, N), np.uint8)
    cv2.fillPoly(m, [pts], 255, lineType=cv2.LINE_AA, shift=2)
    m = m.astype(np.float32) / 255
    if blur_m > 0:
        m = cv2.GaussianBlur(m, (0, 0), blur_m / S.HRES)
    return m


def rect_poly(rect, yaw_deg=0.0, grow=0.0):
    x0, y0, x1, y1 = rect
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    hw, hh = (x1 - x0) / 2 + grow, (y1 - y0) / 2 + grow
    a = np.deg2rad(yaw_deg); ca, sa = np.cos(a), np.sin(a)
    pts = np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]])
    return np.stack([cx + pts[:, 0] * ca - pts[:, 1] * sa, cy + pts[:, 0] * sa + pts[:, 1] * ca], 1)


def rect_sdf(rect, yaw_deg=0.0):
    """signed distance (m) to a (rotated) rectangle; negative inside."""
    x0, y0, x1, y1 = rect
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    hw, hh = (x1 - x0) / 2, (y1 - y0) / 2
    a = np.deg2rad(-yaw_deg); ca, sa = np.cos(a), np.sin(a)
    lx = (X - cx) * ca - (Y - cy) * sa
    ly = (X - cx) * sa + (Y - cy) * ca
    qx, qy = np.abs(lx) - hw, np.abs(ly) - hh
    outside = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0))
    inside = np.minimum(np.maximum(qx, qy), 0)
    return outside + inside


def build():
    # ---------------- shared noise fields ----------------
    n_big = value_noise((N, N), 96, 1)       # ~12 m features
    n_mid = value_noise((N, N), 24, 2)       # ~3 m
    n_small = value_noise((N, N), 6, 3, octaves=3)   # ~0.75 m
    chunk = np.abs(value_noise((N, N), 10, 4, octaves=3) - 0.5) * 2    # ridged -> blocky-ish
    rubble_tex = 0.55 * n_mid + 0.3 * chunk + 0.15 * n_small         # [0,1]

    H = (n_big - 0.5) * 0.25 + (n_small - 0.5) * 0.04                  # gentle ground

    # ---------------- core (photo) ----------------
    U, V = S.w2px(X, Y)
    U = U.astype(np.float32); V = V.astype(np.float32)
    valid = cv2.imread('build/valid.png', 0)
    vs = cv2.remap(valid, U, V, cv2.INTER_LINEAR, borderValue=0).astype(np.float32) / 255
    dist = cv2.distanceTransform((vs > 0.5).astype(np.uint8), cv2.DIST_L2, 5) * S.HRES
    core_w = smoothstep(0.0, 5.0, dist)                                    # feather 5 m inward
    np.save('build/core_w.npy', core_w.astype(np.float16))

    photo = cv2.imread('build/ortho_clean.png').astype(np.float32) / 255
    lum = photo @ np.array([0.114, 0.587, 0.299], np.float32)
    L = cv2.remap(lum, U, V, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
    hp_fine = L - cv2.GaussianBlur(L, (0, 0), 0.6 / S.HRES)
    hp_mid = cv2.GaussianBlur(L, (0, 0), 0.35 / S.HRES) - cv2.GaussianBlur(L, (0, 0), 2.0 / S.HRES)

    Hc = np.zeros((N, N), np.float32)
    rub_core = np.zeros((N, N), np.float32)
    for name, poly, base in S.RUBBLE_ZONES:
        m = poly_mask_w(S.poly_w(poly), blur_m=2.8)
        m = smoothstep(0.25, 0.75, m + (n_mid - 0.5) * 0.55 + (n_big - 0.5) * 0.35)
        h = base * m * (0.45 + 0.9 * n_big) * (0.7 + 0.6 * rubble_tex)
        Hc = np.maximum(Hc, h)
        rub_core = np.maximum(rub_core, m * min(1.0, base / 1.5))

    # fallen towers + broken floor plates are separate solid meshes (buildings.py); keep rubble below them
    slab_mask = np.zeros((N, N), np.float32)
    for name, y0, y1, xw, xr, xe, hh in S.SLABS:
        poly = [(xw[0], y0), (xe[0], y0), (xe[1], y1), (xw[1], y1)]
        m = poly_mask_w(S.poly_w(poly))
        slab_mask = np.maximum(slab_mask, m)
    for name, poly, h0, h1 in S.PLATES:
        slab_mask = np.maximum(slab_mask, poly_mask_w(S.poly_w(poly)))
    inner = cv2.erode(slab_mask, np.ones((9, 9), np.uint8))
    Hc = Hc * (1 - inner) + np.minimum(Hc, 0.9) * inner

    # micro relief from the photo (bright chunks up, dark gaps down)
    Lm = cv2.medianBlur((np.clip(L, 0, 1) * 255).astype(np.uint8), 5).astype(np.float32) / 255
    hp_chunk = Lm - cv2.GaussianBlur(Lm, (0, 0), 0.8 / S.HRES)
    amp = 0.05 + 0.32 * rub_core * (1 - slab_mask)
    Hc += amp * np.clip(hp_chunk, -0.3, 0.3) * 1.6 + 0.45 * rub_core * (1 - slab_mask) * hp_mid

    for poly, h in S.CORE_BUMPS:
        m = poly_mask_w(S.poly_w(poly), blur_m=0.15)
        Hc = np.maximum(Hc, h * np.clip(m * 1.3, 0, 1))

    # flatten under core buildings (their own meshes sit there)
    bld_core = np.zeros((N, N), np.float32)
    for name, poly, h, *_ in S.CORE_BUILDINGS:
        bld_core = np.maximum(bld_core, poly_mask_w(S.poly_w(poly)))
    bld_grow = cv2.dilate(bld_core, np.ones((5, 5), np.uint8))
    Hc = Hc * (1 - bld_grow) + np.minimum(Hc, 0.15) * bld_grow

    H = H * (1 - core_w) + (Hc + (n_small - 0.5) * 0.03) * core_w

    # ---------------- outer neighbourhood ----------------
    lay = json.load(open('build/outer_layout.json'))
    street = S.street_mask_w(X, Y).astype(np.float32)
    street_s = cv2.GaussianBlur(street, (0, 0), 0.25 / S.HRES)
    outside = 1 - core_w
    H = H - 0.12 * street_s * outside          # carriageway below kerb level
    rub_out = np.zeros((N, N), np.float32)
    bld_out = np.zeros((N, N), np.float32)

    def window(cx, cy, rad):
        c0, r0 = S.w2h(cx - rad, cy + rad); c1, r1 = S.w2h(cx + rad, cy - rad)
        return slice(max(0, int(r0)), min(N, int(r1) + 2)), slice(max(0, int(c0)), min(N, int(c1) + 2))

    for f in lay['features']:
        x0, y0, x1, y1 = f['rect']; yaw = f.get('yaw', 0.0)
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        rad = max(x1 - x0, y1 - y0) / 2 + 9
        sl = window(cx, cy, rad)
        Xs, Ys = X[sl], Y[sl]
        global_X, global_Y = X, Y
        nb, nm, ck, ns = n_big[sl], n_mid[sl], chunk[sl], n_small[sl]
        sd = rect_sdf_local(f['rect'], yaw, Xs, Ys)
        if f['type'] in ('collapsed', 'open'):
            sd = sd + (nm - 0.5) * 3.5 + (nb - 0.5) * 3.0
        rt = rubble_tex[sl]
        typ = f['type']
        if typ == 'collapsed':
            hgt = f['height']; spill = 3.5
            core_ = smoothstep(spill, -min(x1 - x0, y1 - y0) * 0.3, sd)
            h = hgt * core_ * (0.55 + 0.6 * nb) + core_ * (rt - 0.5) * 1.1
            H[sl] = np.maximum(H[sl], h)
            rub_out[sl] = np.maximum(rub_out[sl], smoothstep(spill + 1, -1.5, sd))
        elif typ in ('standing', 'tilted', 'pancake'):
            dmg = f.get('damage', 0.3)
            skirt = smoothstep(2.5 + 2 * dmg, 0.0, sd) * (sd > -0.2)
            h = skirt * (0.3 + 1.6 * dmg) * (0.3 + 0.9 * nm) + skirt * (rt - 0.5) * 0.5
            H[sl] = np.maximum(H[sl], h)
            inside = (sd < 0.3).astype(np.float32)
            H[sl] = H[sl] * (1 - inside) + np.minimum(H[sl], 0.05) * inside
            bld_out[sl] = np.maximum(bld_out[sl], (sd < 0).astype(np.float32))
            rub_out[sl] = np.maximum(rub_out[sl], skirt * min(1, 0.4 + dmg))
        else:  # open lot
            inside = smoothstep(0.5, -1.0, sd)
            h = inside * (rt - 0.35) * 0.35
            H[sl] = np.maximum(H[sl], h)
            rub_out[sl] = np.maximum(rub_out[sl], inside * 0.25)
            if f.get('mound'):
                r2 = np.random.default_rng(f['seed'])
                mx = r2.uniform(x0 + 3, x1 - 3); my = r2.uniform(y0 + 3, y1 - 3)
                lay['mounds'].append(dict(x=mx, y=my, r=float(r2.uniform(2.5, 5.0)),
                                          h=float(r2.uniform(0.8, 2.2)), seed=f['seed'] + 1))
    for m in lay['mounds']:
        sl = window(m['x'], m['y'], m['r'] + 2)
        rr = np.hypot(X[sl] - m['x'], Y[sl] - m['y'])
        ang = np.arctan2(Y[sl] - m['y'], X[sl] - m['x'])
        r2 = np.random.default_rng(m['seed'])
        ph = r2.uniform(0, 6.28, 3)
        R = m['r'] * (1 + 0.18 * np.sin(2 * ang + ph[0]) + 0.1 * np.sin(3 * ang + ph[1]) + 0.06 * np.sin(5 * ang + ph[2]))
        prof = np.clip(1 - (rr / R) ** 2, 0, 1) ** 1.4
        h = m['h'] * prof * (0.7 + 0.5 * n_mid[sl]) + prof * (rubble_tex[sl] - 0.5) * 0.9
        h *= (1 - core_w[sl])
        H[sl] = np.maximum(H[sl], h)
        rub_out[sl] = np.maximum(rub_out[sl], smoothstep(1.15, 0.6, rr / R))

    rub_out *= outside
    # light debris everywhere outside the scan (streets get dust/rocks)
    H += outside * (np.clip(chunk - 0.6, 0, 1) * 0.25) * (1 - bld_out)
    np.save('build/H.npy', H.astype(np.float32))
    np.save('build/rub_out.npy', rub_out.astype(np.float16))
    np.save('build/rub_core.npy', (rub_core * core_w).astype(np.float16))
    np.save('build/slab_mask.npy', (slab_mask * core_w).astype(np.float16))
    np.save('build/street.npy', (street_s * outside).astype(np.float16))
    np.save('build/bld_mask.npy', np.maximum(bld_out, bld_core * core_w).astype(np.float16))
    json.dump(lay, open('build/outer_layout_final.json', 'w'), indent=1)
    print('H range', float(H.min()), float(H.max()))


def rect_sdf_local(rect, yaw_deg, Xs, Ys):
    x0, y0, x1, y1 = rect
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    hw, hh = (x1 - x0) / 2, (y1 - y0) / 2
    a = np.deg2rad(-yaw_deg); ca, sa = np.cos(a), np.sin(a)
    lx = (Xs - cx) * ca - (Ys - cy) * sa
    ly = (Xs - cx) * sa + (Ys - cy) * ca
    qx, qy = np.abs(lx) - hw, np.abs(ly) - hh
    return np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0)


if __name__ == '__main__':
    build()
