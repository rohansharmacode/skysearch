"""Global low-res (photo-scale 0.156 m/px, 2048^2) terrain albedo composite."""
import json, numpy as np, cv2
import sitecfg as S
import texsynth as T
from heightfield import value_noise, smoothstep

G = 2048
GS = 2 * S.EXTENT / G
jj, ii = np.meshgrid(np.arange(G), np.arange(G))
GX = (-S.EXTENT + (jj + 0.5) * GS).astype(np.float32)
GY = (S.EXTENT - (ii + 0.5) * GS).astype(np.float32)
HC, HR = S.w2h(GX, GY)
HC = HC.astype(np.float32); HR = HR.astype(np.float32)


def from_h(a):
    return cv2.remap(np.asarray(a, np.float32), HC, HR, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)


def lean_fix(P):
    """replace the leaning tower facade that the camera projected onto the ground"""
    P = P.copy()
    ex = T.exemplars('road')
    for poly in S.LEAN_FIX:
        pts = np.array(poly, np.int32)
        x0, y0 = pts.min(0) - 6; x1, y1 = pts.max(0) + 6
        fill = T.synth((y1 - y0, x1 - x0), ex, seed=77, patch=(14, 26), density=3.0)
        fill = fill * 0.85
        m = np.zeros(P.shape[:2], np.float32)
        cv2.fillPoly(m, [pts], 1.0)
        m = cv2.GaussianBlur(m, (0, 0), 2.0)[..., None]
        reg = P[y0:y1, x0:x1]
        P[y0:y1, x0:x1] = reg * (1 - m[y0:y1, x0:x1]) + fill * m[y0:y1, x0:x1]
    return P


def build():
    H = np.load('build/H.npy')
    core_w = from_h(np.load('build/core_w.npy'))
    rub_out = from_h(np.load('build/rub_out.npy'))
    street = from_h(np.load('build/street.npy'))
    bld = from_h(np.load('build/bld_mask.npy'))
    Hg = from_h(H)

    dirt = T.auto_exemplars('dirt', size=34, n=30)
    rubx = T.auto_exemplars('rubble', size=44, n=36) + T.auto_exemplars('dark', size=44, n=18)
    print('synth ground'); ground = T.synth((G, G), dirt, seed=1, patch=(22, 34), density=2.6)
    # scattered debris on open ground
    spots = T.synth((G, G), rubx, seed=3, patch=(8, 18), density=1.0)
    r_ = np.random.default_rng(21)
    dots = (r_.random((G, G)) > 0.985).astype(np.float32)
    dots = cv2.dilate(dots, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    clus = smoothstep(0.55, 0.9, value_noise((G, G), 40, 22, octaves=3))
    dots = np.maximum(dots * (0.35 + 0.65 * clus), (r_.random((G, G)) > 0.93) * clus)
    spm = cv2.GaussianBlur(dots, (0, 0), 0.7)[..., None] * 0.9
    ground = ground * (1 - spm) + spots * spm
    print('synth rubble'); rubble = T.synth((G, G), rubx, seed=2, patch=(22, 44), density=2.6)
    n1 = value_noise((G, G), 64, 11); n2 = value_noise((G, G), 12, 12, octaves=3); n3 = value_noise((G, G), 3, 13, octaves=2)
    # asphalt
    ex_road = np.concatenate([e.reshape(-1, 3) for e in T.exemplars('road')])
    base_col = np.array([0.37, 0.37, 0.38], np.float32)
    asphalt = base_col[None, None, :] * (0.85 + 0.25 * n2[..., None] + 0.12 * (n3[..., None] - 0.5))
    dust = smoothstep(0.35, 0.8, n1 * 0.7 + n2 * 0.3)[..., None]
    asphalt = asphalt * (1 - 0.55 * dust) + ground * 0.55 * dust
    road = asphalt.astype(np.float32)
    # lane markings (faded, dashed) + cracks
    mark = np.zeros((G, G), np.float32)
    for c, w in S.STREETS_X:
        x = int((c + S.EXTENT) / GS)
        for y in range(0, G, 40):
            cv2.line(mark, (x, y), (x, y + 18), 1.0, 1, cv2.LINE_AA)
    for c, w in S.STREETS_Y:
        y = int((S.EXTENT - c) / GS)
        for x in range(0, G, 40):
            cv2.line(mark, (x, y), (x + 18, y), 1.0, 1, cv2.LINE_AA)
    mark *= (0.35 + 0.4 * n2) * (1 - dust[..., 0])
    road = road * (1 - mark[..., None]) + np.array([0.78, 0.8, 0.8], np.float32) * mark[..., None]
    rng = np.random.default_rng(9)
    crack = np.zeros((G, G), np.float32)
    for k in range(2600):
        p = rng.uniform(0, G, 2); ang = rng.uniform(0, 2 * np.pi)
        pts = [p.copy()]
        for s in range(int(rng.integers(4, 14))):
            ang += rng.normal(0, 0.6); p = p + 3.0 * np.array([np.cos(ang), np.sin(ang)])
            pts.append(p.copy())
        cv2.polylines(crack, [np.round(np.array(pts) * 4).astype(np.int32)], False, 1.0, 1, cv2.LINE_AA, shift=2)
    crack = cv2.GaussianBlur(crack, (0, 0), 0.4) * 0.35
    road = road * (1 - crack[..., None] * street[..., None] * 0.8)

    # composite outer
    rub_m = smoothstep(0.25, 0.75, rub_out + (n2 - 0.5) * 0.45 + (n3 - 0.5) * 0.2)[..., None]
    st = street[..., None]
    out = ground * (1 - st) + road * st
    out = out * (1 - rub_m) + rubble * rub_m
    # baked ambient occlusion / contact darkening for the synthesized area
    cav = np.clip(cv2.GaussianBlur(Hg, (0, 0), 2.0 / GS) - Hg - 0.15, 0, 3) * (1 - street)
    contact = cv2.GaussianBlur(bld, (0, 0), 1.0 / GS) * (1 - bld)
    ao = np.clip(1 - 0.10 * cav - 0.22 * contact, 0.55, 1)[..., None]
    # mild height-tint: tops of mounds dustier/lighter
    out = out * ao * (0.92 + 0.12 * np.clip(Hg / 4, 0, 1))[..., None]
    # large-scale tonal variation (damp patches, dust plumes)
    tone = (0.92 + 0.16 * n1)[..., None] * np.array([1.0, 1.0, 1.0], np.float32)
    warm = (value_noise((G, G), 160, 23, octaves=2) - 0.5)[..., None] * np.array([-0.03, 0.0, 0.035], np.float32)
    out = out * tone + warm

    # photo core (lean-fixed), warped into world grid
    P = lean_fix(T.photo())
    U, V = S.w2px(GX, GY)
    Pc = cv2.remap(P, U.astype(np.float32), V.astype(np.float32), cv2.INTER_CUBIC, borderMode=cv2.BORDER_REFLECT)
    # colour-match synthesized ring to the photo near its border (global gain)
    ring = (core_w > 0.02) & (core_w < 0.6)
    if ring.any():
        gain = (Pc[ring].mean(0) + 1e-3) / (out[ring].mean(0) + 1e-3)
        gain = np.clip(gain, 0.8, 1.25)
        out = out * (0.5 + 0.5 * gain)
    cw = core_w[..., None]
    out = out * (1 - cw) + Pc * cw
    out = np.clip(out, 0, 1)
    np.save('build/albedo_lowres.npy', (out * 65535).astype(np.uint16))
    cls = np.stack([st[..., 0] * (1 - cw[..., 0]), np.maximum(rub_m[..., 0] * (1 - cw[..., 0]),
                    from_h(np.load('build/rub_core.npy'))), from_h(np.load('build/slab_mask.npy'))], -1)
    np.save('build/class_lowres.npy', cls.astype(np.float16))
    cv2.imwrite('build/albedo_lowres.jpg', (out * 255).astype(np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 92])


if __name__ == '__main__':
    build()
