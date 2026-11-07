"""Patch-based texture synthesis (stochastic 'over' compositing of feathered exemplar patches)."""
import numpy as np, cv2

PHOTO = None


def photo():
    global PHOTO
    if PHOTO is None:
        PHOTO = cv2.imread('build/ortho_clean.png').astype(np.float32) / 255
    return PHOTO


EX = {
    'dirt': [(352,108,430,150), (560,112,690,165), (600,292,700,330), (430,100,520,125), (640,215,700,250)],
    'yard': [(590,285,720,385), (585,380,700,452), (560,560,600,610)],
    'rubble': [(380,420,450,510), (262,420,300,520), (300,240,450,300), (150,545,245,600), (440,560,560,620),
               (560,300,600,450), (395,300,450,400)],
    'rubble_dark': [(128,260,166,420), (175,550,250,600)],
    'road': [(548,560,606,650), (520,600,560,660)],
    'concrete': [(652,602,744,648), (300,160,350,200)],
}


def exemplars(key, scale=1.0):
    P = photo(); out = []
    for (x0, y0, x1, y1) in EX[key]:
        c = P[y0:y1, x0:x1]
        if scale != 1.0:
            c = cv2.resize(c, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        out.append(c)
    return out


def feather(h, w, f=0.3):
    y = np.minimum(np.arange(h), np.arange(h)[::-1]) / max(1, h * f)
    x = np.minimum(np.arange(w), np.arange(w)[::-1]) / max(1, w * f)
    m = np.clip(np.minimum(y[:, None], x[None, :]), 0, 1)
    return (m * m * (3 - 2 * m)).astype(np.float32)


def synth(shape, exs, seed, patch=(40, 90), density=2.2, match_mean=True, rot_any=False):
    """Fill `shape` (h,w) by over-compositing random feathered patches from exemplar list."""
    rng = np.random.default_rng(seed)
    H, W = shape
    out = np.zeros((H, W, 3), np.float32)
    # start from a blurred random mosaic so the canvas has no holes
    base = np.mean([e.reshape(-1, 3).mean(0) for e in exs], 0)
    out[:] = base
    area = patch[0] * patch[1]
    n = int(density * H * W / area) + 4
    gmean = np.mean([e.mean((0, 1)) for e in exs], 0)
    for k in range(n):
        e = exs[rng.integers(len(exs))]
        ph = int(rng.integers(patch[0], patch[1] + 1)); pw = int(rng.integers(patch[0], patch[1] + 1))
        ph = min(ph, e.shape[0]); pw = min(pw, e.shape[1])
        y0 = rng.integers(0, e.shape[0] - ph + 1); x0 = rng.integers(0, e.shape[1] - pw + 1)
        p = e[y0:y0 + ph, x0:x0 + pw]
        r = rng.integers(4)
        p = np.rot90(p, r)
        if rng.random() < 0.5:
            p = p[:, ::-1]
        if match_mean:
            p = p + (gmean - p.mean((0, 1))) * 0.6
        p = np.ascontiguousarray(p)
        hh, ww = p.shape[:2]
        ty = rng.integers(-hh // 2, H - hh // 2); tx = rng.integers(-ww // 2, W - ww // 2)
        a = feather(hh, ww, 0.3)[..., None]
        sy0, sx0 = max(0, -ty), max(0, -tx)
        dy0, dx0 = max(0, ty), max(0, tx)
        dy1, dx1 = min(H, ty + hh), min(W, tx + ww)
        if dy1 <= dy0 or dx1 <= dx0:
            continue
        pp = p[sy0:sy0 + dy1 - dy0, sx0:sx0 + dx1 - dx0]; aa = a[sy0:sy0 + dy1 - dy0, sx0:sx0 + dx1 - dx0]
        out[dy0:dy1, dx0:dx1] = out[dy0:dy1, dx0:dx1] * (1 - aa) + pp * aa
    return np.clip(out, 0, 1)


_AUTO = {}


def auto_exemplars(kind, size=44, n=36, seed=0):
    """Pick clean exemplar windows from the photo automatically.
    kind: 'dirt' (smooth ground), 'rubble' (busy debris), 'dark' (shadowed debris)."""
    key = (kind, size, n)
    if key in _AUTO:
        return _AUTO[key]
    import sitecfg as S
    P = photo()
    hsv = cv2.cvtColor((P * 255).astype(np.uint8), cv2.COLOR_BGR2HSV).astype(np.float32) / 255
    lum = P.mean(2); sat = hsv[..., 1]
    excl = np.zeros(lum.shape, np.uint8)
    for name, poly, *_ in S.CORE_BUILDINGS:
        cv2.fillPoly(excl, [np.array(poly, np.int32)], 1)
    for name, y0, y1, xw, xr, xe, hh in S.SLABS:
        cv2.fillPoly(excl, [np.array([(xw[0], y0), (xe[0], y0), (xe[1], y1), (xw[1], y1)], np.int32)], 1)
    for name, poly, *_ in S.PLATES:
        cv2.fillPoly(excl, [np.array(poly, np.int32)], 1)
    for poly in S.LEAN_FIX:
        cv2.fillPoly(excl, [np.array(poly, np.int32)], 1)
    for (cx, cy) in [(369,69),(620,199),(670,87),(659,195),(642,270),(304,230)]:
        cv2.circle(excl, (cx, cy), 30, 1, -1)
    valid = cv2.imread('build/valid.png', 0) > 0
    excl = cv2.dilate(excl, np.ones((7, 7), np.uint8)).astype(bool) | ~cv2.erode(valid.astype(np.uint8), np.ones((15, 15), np.uint8)).astype(bool)
    rub = np.zeros(lum.shape, np.uint8)
    for name, poly, base in S.RUBBLE_ZONES:
        if base >= 1.2:
            cv2.fillPoly(rub, [np.array(poly, np.int32)], 1)
    hf = np.abs(lum - cv2.GaussianBlur(lum, (0, 0), 2.0))
    cands = []
    for y in range(0, lum.shape[0] - size, size // 4):
        for x in range(0, lum.shape[1] - size, size // 4):
            if excl[y:y + size, x:x + size].any():
                continue
            l = lum[y:y + size, x:x + size]; s = sat[y:y + size, x:x + size]
            colorful = ((s > 0.38) & (l > 0.15)).mean()
            bright_obj = (l > 0.82).mean()
            tex = hf[y:y + size, x:x + size].mean()
            inrub = rub[y:y + size, x:x + size].mean()
            m = l.mean()
            if kind == 'dirt':
                ok = (colorful < 0.015 and bright_obj < 0.01 and tex < 0.045 and 0.36 < m < 0.66 and inrub < 0.2
                      and (l < 0.25).mean() < 0.03)
                score = -tex
            elif kind == 'rubble':
                ok = colorful < 0.05 and bright_obj < 0.04 and tex > 0.05 and 0.25 < m < 0.6 and inrub > 0.8
                score = tex
            else:
                ok = colorful < 0.04 and tex > 0.035 and 0.12 < m < 0.3
                score = tex
            if ok:
                cands.append((score, x, y))
    cands.sort(reverse=True)
    picked = []
    for sc, x, y in cands:
        if all(abs(x - px) > size * 0.7 or abs(y - py) > size * 0.7 for _, px, py in picked):
            picked.append((sc, x, y))
        if len(picked) >= n:
            break
    out = []
    for _, x, y in picked:
        p = P[y:y + size, x:x + size].copy()
        # tame residual saturated specks so repeats don't read as repeated objects
        hsvp = cv2.cvtColor((p * 255).astype(np.uint8), cv2.COLOR_BGR2HSV).astype(np.float32)
        hsvp[..., 1] = np.minimum(hsvp[..., 1], 95)
        p = cv2.cvtColor(hsvp.astype(np.uint8), cv2.COLOR_HSV2BGR).astype(np.float32) / 255
        out.append(p)
    _AUTO[key] = out
    print(f'auto exemplars {kind}: {len(out)} picked from {len(cands)}')
    return out
