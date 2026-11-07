"""High-res terrain tiles: 4x4 tiles of 80 m, 2048^2 albedo/normal + 1024^2 roughness, decimated meshes."""
import os, sys, numpy as np, cv2, pyfqmr
import sitecfg as S
import texsynth as T

OUT = 'out/collapse_site'
os.makedirs(f'{OUT}/textures', exist_ok=True)
os.makedirs('build/meshes', exist_ok=True)
TP = 2048          # texture px per tile
LP = 512           # low-res px per tile
TW = 80.0          # tile width (m)
HP = int(TW / S.HRES)  # 640 heightfield cells per tile


def up4(img, ti, tj, m=8):
    pad = np.pad(img, ((m, m), (m, m)) + ((0, 0),) * (img.ndim - 2), mode='reflect')
    c = pad[ti * LP: (ti + 1) * LP + 2 * m, tj * LP: (tj + 1) * LP + 2 * m]
    u = cv2.resize(c, None, fx=4, fy=4, interpolation=cv2.INTER_LANCZOS4)
    return u[4 * m: 4 * m + TP, 4 * m: 4 * m + TP]


def h_tile(H, ti, tj, res):
    """bilinear sample of the heightfield at the tile's texture pixel centres"""
    j = np.arange(res); i = np.arange(res)
    step = TW / res
    cols = (tj * TW + (j + 0.5) * step) / S.HRES
    rows = (ti * TW + (i + 0.5) * step) / S.HRES
    mc, mr = np.meshgrid(cols.astype(np.float32), rows.astype(np.float32))
    return cv2.remap(H, mc, mr, cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)


def textures(ti, tj, alb, cls, H, rubx, dirt):
    base = up4(alb, ti, tj)
    cl = up4(cls, ti, tj)
    street, rub, slab = cl[..., 0], cl[..., 1], cl[..., 2]
    # unsharp the upscaled base a little (photo-like crispness)
    bl = cv2.GaussianBlur(base, (0, 0), 3.0)
    base = base + 0.7 * (base - bl)
    bl2 = cv2.GaussianBlur(base, (0, 0), 1.2)
    base = base + 0.35 * (base - bl2)
    det = T.synth((TP, TP), rubx + dirt, seed=1000 + ti * 4 + tj, patch=(30, 64), density=2.0)
    Ld = det.mean(2)
    hp = Ld - cv2.GaussianBlur(Ld, (0, 0), 3.0)
    hp2 = Ld - cv2.GaussianBlur(Ld, (0, 0), 10.0)
    k = (0.28 + 0.32 * rub - 0.12 * street) * (1 - 0.75 * slab)
    edge = cv2.GaussianBlur(np.abs(cv2.Laplacian(cv2.GaussianBlur(base.mean(2), (0, 0), 2.0), cv2.CV_32F)), (0, 0), 3)
    k = k * np.clip(1.0 - edge * 25, 0.25, 1.0)
    lumb = base.mean(2, keepdims=True) + 1e-3
    albedo = base * (1 + (k * (0.9 * hp + 0.35 * hp2))[..., None] / np.clip(lumb, 0.15, 1))
    albedo = np.clip(albedo, 0, 1)
    # ----- normal -----
    Ht = h_tile(H, ti, tj, TP)
    hpH = Ht - cv2.GaussianBlur(Ht, (0, 0), 0.45 / (TW / TP))
    hd = (0.030 * hp + 0.02 * hp2) * (0.4 + 1.2 * rub) * (1 - 0.6 * slab) / 0.1
    hh = 0.8 * hpH + hd
    d = TW / TP
    gx = cv2.Sobel(hh, cv2.CV_32F, 1, 0, ksize=3) / (8 * d)
    gy = -cv2.Sobel(hh, cv2.CV_32F, 0, 1, ksize=3) / (8 * d)      # +y = north = up in image
    n = np.dstack([-gx, -gy, np.ones_like(gx)])
    n /= np.linalg.norm(n, axis=2, keepdims=True)
    nrm = ((n * 0.5 + 0.5) * 255).astype(np.uint8)
    # ----- roughness -----
    lum = cv2.resize(albedo.mean(2), (TP // 2, TP // 2), interpolation=cv2.INTER_AREA)
    st2 = cv2.resize(street, (TP // 2, TP // 2), interpolation=cv2.INTER_AREA)
    rough = np.clip(0.93 - 0.10 * st2 - 0.12 * (lum - 0.45) + 0.05 * (cv2.GaussianBlur(np.random.default_rng(ti * 7 + tj).random(lum.shape).astype(np.float32), (0, 0), 6) - 0.5) * 8, 0.55, 1.0)
    name = f'terrain_r{ti}c{tj}'
    cv2.imwrite(f'{OUT}/textures/{name}_albedo.jpg', (albedo * 255 + 0.5).astype(np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 90])
    cv2.imwrite(f'{OUT}/textures/{name}_normal.jpg', nrm[..., ::-1], [cv2.IMWRITE_JPEG_QUALITY, 94])
    cv2.imwrite(f'{OUT}/textures/{name}_rough.jpg', (rough * 255).astype(np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 90])
    return name


def mesh(ti, tj, H, target):
    sub = H[ti * HP: (ti + 1) * HP + 1, tj * HP: (tj + 1) * HP + 1]
    n = sub.shape[0]
    x0 = -S.EXTENT + tj * TW; ytop = S.EXTENT - ti * TW
    jj, ii = np.meshgrid(np.arange(n), np.arange(n))
    V = np.stack([x0 + jj * S.HRES, ytop - ii * S.HRES, sub], -1).reshape(-1, 3).astype(np.float64)
    idx = np.arange(n * n).reshape(n, n)
    a = idx[:-1, :-1].ravel(); b = idx[:-1, 1:].ravel(); c = idx[1:, :-1].ravel(); d = idx[1:, 1:].ravel()
    # CCW seen from +z (rows go south): a(top-left) c(bottom-left) b(top-right)
    F = np.concatenate([np.stack([a, c, b], 1), np.stack([b, c, d], 1)]).astype(np.int32)
    s = pyfqmr.Simplify()
    s.setMesh(V, F)
    s.simplify_mesh(target_count=target, aggressiveness=6, preserve_border=True, verbose=0)
    v, f, _ = s.getMesh()
    np.savez_compressed(f'build/meshes/terrain_r{ti}c{tj}.npz', v=v.astype(np.float32), f=f.astype(np.int32),
                        x0=x0, ytop=ytop)
    return len(v), len(f)


if __name__ == '__main__':
    H = np.load('build/H.npy')
    alb = np.load('build/albedo_lowres.npy').astype(np.float32) / 65535
    cls = np.load('build/class_lowres.npy').astype(np.float32)
    rubx = T.auto_exemplars('rubble', size=44, n=36) + T.auto_exemplars('dark', size=44, n=18)
    dirt = T.auto_exemplars('dirt', size=34, n=30)
    which = [(i, j) for i in range(4) for j in range(4)]
    if len(sys.argv) > 1:
        which = [tuple(map(int, a.split(','))) for a in sys.argv[1:]]
    for ti, tj in which:
        central = ti in (1, 2) and tj in (1, 2)
        nv, nf = mesh(ti, tj, H, 260000 if central else 70000)
        textures(ti, tj, alb, cls, H, rubx, dirt)
        print('tile', ti, tj, nv, nf, flush=True)
