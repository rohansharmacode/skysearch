"""Texture library for buildings, slabs and debris, all derived from the scan photo (+ procedural detail).
Writes <name>_albedo.jpg / _normal.png and build/texlib.json with physical tile sizes (metres)."""
import json, os, numpy as np, cv2
import sitecfg as S
import texsynth as T
from heightfield import value_noise

OUT = 'out/collapse_site/textures'
os.makedirs(OUT, exist_ok=True)
LIB = {}
P = cv2.imread('build/ortho_fixed.png').astype(np.float32) / 255
rng = np.random.default_rng(3)


def sharpen(img, a=0.8, s=2.0):
    return np.clip(img + a * (img - cv2.GaussianBlur(img, (0, 0), s)), 0, 1)


def normal_from(img, strength=2.0, invert=False, blur=1.0):
    img = np.asarray(img, np.float32)
    L = cv2.GaussianBlur(img.mean(2) if img.ndim == 3 else img, (0, 0), blur)
    if invert:
        L = 1 - L
    gx = cv2.Sobel(L, cv2.CV_32F, 1, 0, ksize=3) * strength
    gy = -cv2.Sobel(L, cv2.CV_32F, 0, 1, ksize=3) * strength
    n = np.dstack([-gx, -gy, np.ones_like(gx)]); n /= np.linalg.norm(n, axis=2, keepdims=True)
    return ((n * 0.5 + 0.5) * 255).astype(np.uint8)


def make_tileable(img, f=0.25):
    h, w = img.shape[:2]
    r = np.roll(img, (h // 2, w // 2), (0, 1))
    wy = np.minimum(np.arange(h), np.arange(h)[::-1]) / (h * f); wx = np.minimum(np.arange(w), np.arange(w)[::-1]) / (w * f)
    m = np.clip(np.minimum(wy[:, None], wx[None, :]), 0, 1)[..., None]
    return img * m + r * (1 - m)


def save(name, alb, size_m, nstrength=2.0, ninvert=False, rough=0.85, metal=0.0, nrm=None):
    alb = np.asarray(alb, np.float32)
    cv2.imwrite(f'{OUT}/{name}_albedo.jpg', (np.clip(alb, 0, 1) * 255 + 0.5).astype(np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 92])
    n = nrm if nrm is not None else normal_from(alb, nstrength, ninvert)
    cv2.imwrite(f'{OUT}/{name}_normal.png', n[..., ::-1], [cv2.IMWRITE_PNG_COMPRESSION, 9])
    LIB[name] = dict(size=list(map(float, size_m)), rough=rough, metal=metal)


def crop_up(box, scale, stretch_x=1.0, rot=0):
    x0, y0, x1, y1 = box
    c = P[y0:y1, x0:x1]
    if rot:
        c = np.rot90(c, rot)
    c = cv2.resize(c, None, fx=scale * stretch_x, fy=scale, interpolation=cv2.INTER_LANCZOS4)
    return sharpen(np.clip(c, 0, 1), 0.6, 1.5)


def grime(shape, seed, streak=True):
    h, w = shape
    n = value_noise((h, w), max(8, h // 8), seed, octaves=4)
    g = 0.88 + 0.16 * n
    if streak:   # vertical rain streaks
        s = np.random.default_rng(seed).random((1, w)).astype(np.float32)
        s = cv2.GaussianBlur(np.repeat(s, h, 0), (0, 0), 3)
        fall = np.linspace(0.2, 1.0, h, dtype=np.float32)[:, None]
        g *= 1 - 0.18 * np.clip((s - 0.5) * 3, 0, 1) * fall
    return g[..., None]


# ---------------------------------------------------------------- facades from the fallen towers
def facade_from_slab(name, box, stretch, storeys_px=20, n_storeys=4, y_start=None):
    x0, y0, x1, y1 = box
    ys = y_start if y_start is not None else y0
    c = P[ys:ys + storeys_px * n_storeys, x0:x1]
    sc = 128 / storeys_px
    c = cv2.resize(c, None, fx=sc * stretch, fy=sc, interpolation=cv2.INTER_LANCZOS4)
    c = sharpen(np.clip(c, 0, 1), 0.9, 2.0)
    c = make_tileable(c, 0.12)
    c = c * grime(c.shape[:2], hash(name) % 1000)
    w_m = (x1 - x0) * S.MPP * stretch
    save(name, c, (w_m, 3.1 * n_storeys), nstrength=3.0, rough=0.82)


facade_from_slab('facade_A', (176, 0, 222, 0), 1.45, y_start=330)          # W tower window face
facade_from_slab('facade_B', (462, 0, 518, 0), 1.45, y_start=330)          # E tower window face
facade_from_slab('facade_balcony', (227, 0, 255, 0), 2.0, y_start=300)     # balcony face
for nm, src, tint in [('facade_A_warm', 'facade_A', (0.93, 1.0, 1.10)), ('facade_B_tile', 'facade_B', (1.08, 1.02, 0.92)),
                      ('facade_A_pale', 'facade_A', (1.12, 1.12, 1.10))]:
    a = cv2.imread(f'{OUT}/{src}_albedo.jpg').astype(np.float32) / 255
    a = np.clip(a * np.array(tint, np.float32), 0, 1)
    save(nm, a, LIB[src]['size'], nstrength=3.0, rough=0.82)
LIB['facade_tower'] = dict(LIB['facade_B'], src='facade_B')


# ---------------------------------------------------------------- procedural walls (colours sampled from the photo)
def sample_col(box):
    x0, y0, x1, y1 = box
    return np.median(P[y0:y1, x0:x1].reshape(-1, 3), 0)


def corrugated(name, col, seed):
    h = w = 1024   # 4 m x 4 m
    x = np.arange(w, dtype=np.float32)
    rib = 0.5 + 0.5 * np.sin(x / w * 4.0 / 0.2 * 2 * np.pi)       # 20 cm ribs
    shade = 0.82 + 0.25 * rib
    img = np.ones((h, w, 3), np.float32) * col[None, None, :] * shade[None, :, None]
    img *= grime((h, w), seed)
    rust = value_noise((h, w), 64, seed + 1, octaves=4)
    rm = np.clip((rust - 0.72) * 4, 0, 1)[..., None] * np.linspace(0.3, 1, h, dtype=np.float32)[:, None, None]
    img = img * (1 - rm * 0.6) + np.array([0.18, 0.28, 0.42], np.float32) * rm * 0.6   # BGR rust
    # dent / tear lines
    r = np.random.default_rng(seed)
    for k in range(6):
        p = r.integers(0, w, 2); q = p + r.integers(-200, 200, 2)
        cv2.line(img, tuple(int(v) for v in p), tuple(int(v) for v in q), (0.25, 0.25, 0.25), 2, cv2.LINE_AA)
    nh = np.repeat(rib[None, :], h, 0)
    save(name, make_tileable(img, 0.1), (4.0, 4.0), nrm=normal_from(nh[..., None].repeat(3, 2), 6.0), rough=0.55, metal=0.0)


corrugated('metal_white', sample_col((180, 105, 340, 155)) * 1.0, 31)
corrugated('metal_green', sample_col((370, 630, 490, 685)), 32)


def plaster_windows(name, col, seed, bay=3.6, storey=3.1, win=(1.6, 1.4)):
    pxm = 160   # px per metre
    w = int(bay * 2 * pxm); h = int(storey * 2 * pxm)
    n = value_noise((h, w), 48, seed, octaves=5)
    img = np.ones((h, w, 3), np.float32) * col * (0.9 + 0.18 * n)[..., None]
    r = np.random.default_rng(seed)
    for by in range(2):
        for bx in range(2):
            cx = int((bx + 0.5) * bay * pxm); cy = int((by + 0.45) * storey * pxm)
            ww, hh = int(win[0] * pxm / 2), int(win[1] * pxm / 2)
            cv2.rectangle(img, (cx - ww - 10, cy - hh - 10), (cx + ww + 10, cy + hh + 10), tuple(float(v) * 1.12 for v in col), -1)
            glass = np.array([0.16, 0.15, 0.14], np.float32) * (0.7 + 0.6 * r.random())
            if r.random() < 0.45:   # smashed / open window: darker hole
                glass = glass * 0.4
            cv2.rectangle(img, (cx - ww, cy - hh), (cx + ww, cy + hh), tuple(float(v) for v in glass), -1)
            cv2.line(img, (cx, cy - hh), (cx, cy + hh), tuple(float(v) * 0.9 for v in col), 6)
        # floor band
        yb = int((by + 1) * storey * pxm) - 18
        img[yb:yb + 18] *= 0.86
    img *= grime((h, w), seed + 5)
    # dust / soot from the scan's rubble luminance
    rx = T.synth((h // 4, w // 4), T.auto_exemplars('rubble', size=44, n=36), seed=seed, patch=(20, 40), density=2.0)
    rl = cv2.resize(rx.mean(2), (w, h), interpolation=cv2.INTER_CUBIC)
    img = img * (0.78 + 0.45 * rl[..., None])
    img = cv2.GaussianBlur(img, (0, 0), 1.2)
    # cracks
    for k in range(5):
        p = np.array([r.integers(0, w), r.integers(0, h)], float); ang = r.uniform(0, 6.28); pts = [p.copy()]
        for s_ in range(30):
            ang += r.normal(0, 0.5); p = p + 8 * np.array([np.cos(ang), np.sin(ang)]); pts.append(p.copy())
        cv2.polylines(img, [np.array(pts, np.int32)], False, (0.2, 0.2, 0.2), 2, cv2.LINE_AA)
    img = make_tileable(img, 0.06)
    save(name, img, (bay * 2, storey * 2), nstrength=2.5, rough=0.9)


plaster_windows('plaster_light', sample_col((700, 470, 728, 540)) * 0.95, 41)
plaster_windows('plaster_grey', sample_col((440, 160, 512, 195)) * 0.95, 42)
plaster_windows('plaster_beige', np.array([0.62, 0.70, 0.76], np.float32), 43)
plaster_windows('plaster_pink', np.array([0.62, 0.62, 0.74], np.float32), 44)


# ---------------------------------------------------------------- roofs (photo crops) + generic
ROOFS = {
    'roof_concrete': ((654, 603, 742, 646), 0.85),
    'roof_green': ((614, 662, 700, 724), 0.8),
    'roof_red': ((624, 470, 696, 542), 0.8),
    'roof_white': ((182, 102, 344, 156), 0.55),
    'roof_metal': ((242, 628, 494, 686), 0.55),
}
def roof_concrete_like(name, col, seed, stains=0.35):
    n = 1024
    base = np.ones((n, n, 3), np.float32) * np.asarray(col, np.float32)
    n1 = value_noise((n, n), 96, seed, octaves=5); n2 = value_noise((n, n), 12, seed + 1, octaves=3)
    img = base * (0.86 + 0.2 * n1 + 0.08 * n2)[..., None]
    st = np.clip((value_noise((n, n), 160, seed + 2, octaves=4) - 0.55) * 3, 0, 1)[..., None] * stains
    img = img * (1 - st * 0.45)
    # expansion joints / membrane seams
    for k in range(0, n, 256):
        img[k:k + 3] *= 0.75; img[:, k:k + 3] *= 0.75
    r = np.random.default_rng(seed)
    for k in range(int(6 * stains + 2)):      # debris specks, fallen bits
        c = (int(r.integers(0, n)), int(r.integers(0, n)))
        cv2.circle(img, c, int(r.integers(3, 14)), tuple(float(v) * 0.6 for v in col), -1)
    return make_tileable(img, 0.15)


def roof_tiles(col, seed):
    n = 1024; img = np.zeros((n, n, 3), np.float32)
    r = np.random.default_rng(seed)
    rows = 32; tw = 40
    for i in range(rows):
        y0 = i * (n // rows)
        off = (tw // 2) * (i % 2)
        for x in range(-tw, n + tw, tw):
            c = np.asarray(col, np.float32) * r.uniform(0.75, 1.15)
            cv2.rectangle(img, (x + off, y0), (x + off + tw - 3, y0 + n // rows - 2), tuple(float(v) for v in c), -1)
        img[y0 + n // rows - 4: y0 + n // rows] *= 0.55
    img *= (0.85 + 0.25 * value_noise((n, n), 64, seed + 3, octaves=4))[..., None]
    return make_tileable(img, 0.1)


roof_cfg = {
    'roof_concrete': roof_concrete_like('roof_concrete', sample_col((680, 606, 742, 646)) * 1.05, 61, 0.5),
    'roof_green': roof_concrete_like('roof_green', sample_col((640, 690, 700, 724)), 62, 0.3),
    'roof_red': roof_tiles(sample_col((640, 480, 690, 540)), 63),
}
for nm, img in roof_cfg.items():
    save(nm, img, (8.0, 8.0), nstrength=2.5, rough=0.85)
for nm, src in (('roof_white', 'metal_white'), ('roof_metal', 'metal_green')):
    a = cv2.imread(f'{OUT}/{src}_albedo.jpg').astype(np.float32) / 255
    nrm = cv2.imread(f'{OUT}/{src}_normal.png')[..., ::-1]
    save(nm, a, LIB[src]['size'], rough=0.55, nrm=nrm)

# ---------------------------------------------------------------- broken concrete / rubble / plain concrete
rubx = T.auto_exemplars('rubble', size=44, n=36) + T.auto_exemplars('dark', size=44, n=18)
rub = T.synth((256, 256), rubx, seed=51, patch=(30, 44), density=3.0)
rub = sharpen(cv2.resize(rub, (1024, 1024), interpolation=cv2.INTER_LANCZOS4), 0.7, 2)
det = T.synth((1024, 1024), rubx, seed=52, patch=(30, 60), density=2.0)
Ld = det.mean(2); rub = np.clip(rub * (1 + 0.5 * (Ld - cv2.GaussianBlur(Ld, (0, 0), 4))[..., None] / 0.4), 0, 1)
save('rubble', make_tileable(rub, 0.15), (6.0, 6.0), nstrength=4.0, rough=0.95)

conc_src = [P[606:646, 656:700], P[160:196, 440:510], P[250:290, 232:254]]
conc = T.synth((256, 256), conc_src, seed=53, patch=(20, 36), density=3.0, match_mean=True)
conc = cv2.resize(conc, (1024, 1024), interpolation=cv2.INTER_LANCZOS4)
conc = cv2.cvtColor(cv2.cvtColor((conc * 255).astype(np.uint8), cv2.COLOR_BGR2HSV), cv2.COLOR_HSV2BGR).astype(np.float32) / 255
g = np.array([0.6, 0.62, 0.63], np.float32)
conc = conc * 0.5 + g * 0.5
nn_ = value_noise((1024, 1024), 24, 54, octaves=5)
conc = conc * (0.85 + 0.25 * nn_)[..., None] * grime((1024, 1024), 55, streak=False)
# rebar-dotted broken edge band
save('concrete', make_tileable(conc, 0.15), (3.0, 3.0), nstrength=3.0, rough=0.9)

# debris atlas (chunks): mix of concrete, rubble, brick, tile
atlas = np.zeros((1024, 1024, 3), np.float32)
atlas[:512, :512] = cv2.resize(conc, (512, 512))
atlas[:512, 512:] = cv2.resize(rub, (512, 512))
brick = np.ones((512, 512, 3), np.float32) * np.array([0.30, 0.36, 0.55], np.float32)
for yy in range(0, 512, 32):
    cv2.line(brick, (0, yy), (511, yy), (0.62, 0.64, 0.66), 4)
    off = 0 if (yy // 32) % 2 == 0 else 32
    for xx in range(off, 512, 64):
        cv2.line(brick, (xx, yy), (xx, yy + 32), (0.62, 0.64, 0.66), 4)
brick *= (0.8 + 0.35 * value_noise((512, 512), 16, 56))[..., None]
atlas[512:, :512] = brick
tile = cv2.resize(P[468:540, 626:696], (512, 512), interpolation=cv2.INTER_CUBIC)   # red roof tiles
atlas[512:, 512:] = tile * 0.9
save('debris_atlas', atlas, (1.0, 1.0), nstrength=3.0, rough=0.9)

# core roof photo (shared by all scanned buildings): 4x upscale of the cleaned orthophoto
up = sharpen(cv2.resize(P, None, fx=4, fy=4, interpolation=cv2.INTER_LANCZOS4), 0.6, 3.0)
save('scan_roofs', up, (P.shape[1] * S.MPP, P.shape[0] * S.MPP), nstrength=2.5, rough=0.8)
LIB['scan_roofs']['px'] = [P.shape[1], P.shape[0]]

json.dump(LIB, open('build/texlib.json', 'w'), indent=1)
print(sorted(LIB))

# far-field ground (tiles across the 2 km horizon plane)
dirt = T.auto_exemplars('dirt', size=34, n=30)
gf = T.synth((256, 256), dirt, seed=71, patch=(22, 34), density=3.0)
spots = T.synth((256, 256), rubx, seed=72, patch=(6, 14), density=0.6)
m = (np.random.default_rng(73).random((256, 256)) > 0.96).astype(np.float32)
m = cv2.GaussianBlur(cv2.dilate(m, np.ones((3, 3), np.uint8)), (0, 0), 0.8)[..., None]
gf = gf * (1 - m) + spots * m
gf = sharpen(cv2.resize(gf, (1024, 1024), interpolation=cv2.INTER_LANCZOS4), 0.6, 2.0)
save('ground_far', make_tileable(gf, 0.2), (40.0, 40.0), nstrength=2.0, rough=0.95)
json.dump(LIB, open('build/texlib.json', 'w'), indent=1)
