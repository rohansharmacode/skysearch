"""Debris prototypes (broken concrete, slab shards, bricks, rebar, tiles, pipes) + scatter."""
import pickle, json, numpy as np, cv2
from scipy.spatial import ConvexHull
import sitecfg as S

rng = np.random.default_rng(77)
# atlas quadrants (u0, v0) of a 0.5 x 0.5 region; v measured bottom-up
Q = {'concrete': (0.0, 0.5), 'rubble': (0.5, 0.5), 'brick': (0.0, 0.0), 'tile': (0.5, 0.0)}


def faceted(points, quad, uvscale):
    hull = ConvexHull(points)
    V, F, UV, N = [], [], [], []
    c = points.mean(0)
    u0, v0 = Q[quad]
    for simp in hull.simplices:
        p = points[simp]
        n = np.cross(p[1] - p[0], p[2] - p[0]); n /= np.linalg.norm(n) + 1e-12
        if np.dot(n, p.mean(0) - c) < 0:
            p = p[::-1]; n = -n
        ax = np.argmax(np.abs(n))
        k = [i for i in range(3) if i != ax]
        b = len(V)
        for q in p:
            uu = (q[k[0]] * uvscale) % 1.0; vv = (q[k[1]] * uvscale) % 1.0
            V.append(q); N.append(n); UV.append((u0 + 0.02 + 0.46 * uu, v0 + 0.02 + 0.46 * vv))
        F.append((b, b + 1, b + 2))
    return np.array(V, np.float32), np.array(F, np.int32), np.array(UV, np.float32), np.array(N, np.float32)


def chunk(seed, quad, flat=1.0, npts=18):
    r = np.random.default_rng(seed)
    p = r.normal(0, 1, (npts, 3)); p /= np.linalg.norm(p, axis=1, keepdims=True)
    p *= r.uniform(0.55, 1.0, (npts, 1))
    p = p * np.array([0.5, 0.5 * r.uniform(0.6, 1.0), 0.5 * flat])
    # cut a couple of flat 'fracture' planes
    for k in range(2):
        nrm = r.normal(0, 1, 3); nrm /= np.linalg.norm(nrm); d = r.uniform(0.15, 0.3)
        proj = p @ nrm
        p = p - np.maximum(proj - d, 0)[:, None] * nrm
    return faceted(p, quad, 1.3)


def plate(seed, quad, th):
    r = np.random.default_rng(seed)
    k = r.integers(5, 9)
    ang = np.sort(r.uniform(0, 2 * np.pi, k)); rad = r.uniform(0.3, 0.5, k)
    xy = np.stack([np.cos(ang) * rad, np.sin(ang) * rad * r.uniform(0.6, 1.0)], 1)
    pts = np.concatenate([np.c_[xy, np.full(k, th / 2)], np.c_[xy, np.full(k, -th / 2)]])
    return faceted(pts, quad, 1.0)


def box(sx, sy, sz, quad, uvs=1.0):
    pts = np.array([[x, y, z] for x in (-sx / 2, sx / 2) for y in (-sy / 2, sy / 2) for z in (-sz / 2, sz / 2)])
    return faceted(pts, quad, uvs)


def tube(path, radius, seg=6):
    path = np.asarray(path, float)
    V, F, UV, N = [], [], [], []
    rings = []
    for i, p in enumerate(path):
        t = path[min(i + 1, len(path) - 1)] - path[max(i - 1, 0)]; t /= np.linalg.norm(t)
        a = np.cross(t, [0, 0, 1.0]);
        if np.linalg.norm(a) < 1e-3: a = np.cross(t, [1.0, 0, 0])
        a /= np.linalg.norm(a); b = np.cross(t, a)
        rings.append([p + radius * (np.cos(th) * a + np.sin(th) * b) for th in np.linspace(0, 2 * np.pi, seg, endpoint=False)])
    for i in range(len(path) - 1):
        for j in range(seg):
            q = [rings[i][j], rings[i][(j + 1) % seg], rings[i + 1][(j + 1) % seg], rings[i + 1][j]]
            n = np.cross(q[1] - q[0], q[3] - q[0]); n /= np.linalg.norm(n) + 1e-12
            b0 = len(V); V += q; N += [n] * 4; UV += [(0.6, 0.6)] * 4
            F += [(b0, b0 + 1, b0 + 2), (b0, b0 + 2, b0 + 3)]
    return np.array(V, np.float32), np.array(F, np.int32), np.array(UV, np.float32), np.array(N, np.float32)


def rebar(seed):
    r = np.random.default_rng(seed)
    p = [np.zeros(3)]; d = np.array([1.0, 0, 0.0]); L = r.uniform(1.2, 2.6)
    for k in range(4):
        d = d + r.normal(0, 0.45, 3); d /= np.linalg.norm(d)
        p.append(p[-1] + d * L / 4)
    p = np.array(p); p -= p.mean(0)
    return tube(p, 0.014, 6)


def tile_shard(seed):
    r = np.random.default_rng(seed)
    V, F, UV, N = plate(seed, 'tile', 0.025)
    V[:, 2] += 0.06 * np.sin(V[:, 0] * 9)   # curved roof tile
    return V, F, UV, N


PROTOS = []   # (name, material, (V,F,UV,N), weight, scale_range)
for i in range(6):
    PROTOS.append((f'chunk{i}', 'debris_atlas', chunk(100 + i, 'concrete' if i % 2 == 0 else 'rubble', flat=rng.uniform(0.45, 0.9)), 'chunk'))
for i in range(3):
    PROTOS.append((f'slabshard{i}', 'debris_atlas', plate(200 + i, 'concrete', rng.uniform(0.12, 0.22)), 'plate'))
PROTOS.append(('brick0', 'debris_atlas', box(0.23, 0.11, 0.065, 'brick', 3.0), 'brick'))
PROTOS.append(('brick1', 'debris_atlas', box(0.19, 0.09, 0.06, 'brick', 3.0), 'brick'))
PROTOS.append(('tile0', 'debris_atlas', tile_shard(300), 'tile'))
for i in range(3):
    PROTOS.append((f'rebar{i}', 'rebar', rebar(400 + i), 'rebar'))
PROTOS.append(('pipe0', 'concrete', tube(np.array([[-1.0, 0, 0], [1.0, 0, 0]]), 0.09, 10), 'pipe'))
PROTOS.append(('block0', 'debris_atlas', box(0.39, 0.19, 0.19, 'concrete', 2.0), 'block'))

def car(seed):
    r = np.random.default_rng(seed)
    parts = []
    def bx(cx, cy, cz, sx, sy, sz):
        p = np.array([[x, y, z] for x in (-sx / 2, sx / 2) for y in (-sy / 2, sy / 2) for z in (-sz / 2, sz / 2)])
        return p + [cx, cy, cz]
    body = bx(0, 0, 0.62, 4.3, 1.78, 0.72); cab = bx(-0.25, 0, 1.25, 2.2, 1.55, 0.58)
    out = []
    for pts in (body, cab):
        pts = pts + r.normal(0, 0.06, pts.shape) * (pts[:, 2:3] > 0.9)     # crumpled roof
        out.append(faceted(pts, 'concrete', 0.5))
    V = np.concatenate([o[0] for o in out]); F = np.concatenate([out[0][1], out[1][1] + len(out[0][0])])
    UV = np.concatenate([o[2] for o in out]); N = np.concatenate([o[3] for o in out])
    return V, F, UV, N


for i, col in enumerate(['car_white', 'car_silver', 'car_red', 'car_blue', 'car_dark']):
    PROTOS.append((f'car{i}', col, car(500 + i), 'car'))
PROTOS.append(('pole0', 'concrete', tube(np.array([[0, 0, 0.0], [0, 0, 9.0]]), 0.14, 8), 'pole'))

KIND_IDX = {}
for i, p in enumerate(PROTOS):
    KIND_IDX.setdefault(p[3], []).append(i)

# --------------------------------------------------------------------- scatter
H = np.load('build/H.npy')
N = H.shape[0]
gy, gx = np.gradient(H, S.HRES)          # gy along rows (south) -> d/dy = -gy
core_r = np.load('build/rub_core.npy').astype(np.float32)
slab = np.load('build/slab_mask.npy').astype(np.float32)
outr = np.load('build/rub_out.npy').astype(np.float32)
bld = np.load('build/bld_mask.npy').astype(np.float32)
street = np.load('build/street.npy').astype(np.float32)
core_w = np.load('build/core_w.npy').astype(np.float32)
bld_d = cv2.dilate(bld, np.ones((5, 5), np.uint8))
dens = (core_r * (1 - 0.7 * slab) * 1.0 + outr * 0.9 + 0.06 * (1 - core_w) + 0.05 * street) * (1 - bld_d)
dens = np.clip(dens, 0, None)


def sample(n, weight_map, seed):
    r = np.random.default_rng(seed)
    w = weight_map.ravel().astype(np.float64); w /= w.sum()
    idx = r.choice(len(w), n, p=w)
    i, j = np.divmod(idx, N)
    jx = j + r.uniform(-0.5, 0.5, n); iy = i + r.uniform(-0.5, 0.5, n)
    x = -S.EXTENT + jx * S.HRES; y = S.EXTENT - iy * S.HRES
    z = cv2.remap(H, jx.astype(np.float32)[None], iy.astype(np.float32)[None], cv2.INTER_LINEAR)[0]
    nx = -cv2.remap(gx, jx.astype(np.float32)[None], iy.astype(np.float32)[None], cv2.INTER_LINEAR)[0]
    ny = cv2.remap(gy, jx.astype(np.float32)[None], iy.astype(np.float32)[None], cv2.INTER_LINEAR)[0]
    nrm = np.stack([nx, ny, np.ones_like(nx)], 1); nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)
    return np.stack([x, y, z], 1), nrm


def quat_from(nrm, r, tilt_rand=0.6):
    """random yaw about normal + extra random tilt; returns (w,x,y,z)"""
    n = len(nrm)
    yaw = r.uniform(0, 2 * np.pi, n)
    t = r.normal(0, tilt_rand, (n, 3)); t[:, 2] = 0
    up = nrm + t; up /= np.linalg.norm(up, axis=1, keepdims=True)
    z = np.array([0, 0, 1.0])
    axis = np.cross(np.broadcast_to(z, up.shape), up); s = np.linalg.norm(axis, axis=1); c = up[:, 2]
    ang = np.arctan2(s, c)
    axis = axis / np.maximum(s, 1e-9)[:, None]
    q1 = np.c_[np.cos(ang / 2), axis * np.sin(ang / 2)[:, None]]
    q2 = np.c_[np.cos(yaw / 2), np.zeros((n, 2)), np.sin(yaw / 2)]
    w1, v1 = q1[:, :1], q1[:, 1:]; w2, v2 = q2[:, :1], q2[:, 1:]
    w = w1 * w2 - np.sum(v1 * v2, 1, keepdims=True)
    v = w1 * v2 + w2 * v1 + np.cross(v1, v2)
    return np.c_[w, v]


pos, ori, scl, pid = [], [], [], []
plan = [  # kind, count, scale lognormal (mu,sigma) in metres, tilt, sink fraction
    ('chunk', 26000, (np.log(0.38), 0.55), 0.7, 0.25),
    ('chunk', 4000, (np.log(1.05), 0.35), 0.6, 0.35),
    ('plate', 2500, (np.log(1.8), 0.35), 0.5, 0.15),
    ('plate', 7000, (np.log(0.9), 0.4), 0.45, 0.05),
    ('brick', 9000, (0.0, 0.15), 1.2, 0.2),
    ('tile', 2500, (np.log(0.8), 0.2), 0.8, 0.0),
    ('rebar', 2600, (0.0, 0.25), 1.4, -0.1),
    ('pipe', 140, (0.0, 0.3), 0.3, 0.2),
    ('block', 2400, (0.0, 0.15), 1.0, 0.2),
]
r = np.random.default_rng(5)
for k, (kind, n, (mu, sg), tilt, sink) in enumerate(plan):
    P, Nn = sample(n, dens if kind not in ('tile',) else dens * (1 - core_w) + 0.02 * core_r, 900 + k)
    s = np.exp(r.normal(mu, sg, n))
    s = np.clip(s, 0.08, 3.5 if kind in ('chunk', 'plate') else 2.0)
    sc = np.stack([s, s * r.uniform(0.7, 1.3, n), s * r.uniform(0.7, 1.1, n)], 1)
    if kind in ('brick', 'block', 'pipe'):
        sc = np.stack([s, s, s], 1)
    P = P.copy(); P[:, 2] -= sink * sc[:, 2] * 0.5
    ids = r.choice(KIND_IDX[kind], n)
    pos.append(P); ori.append(quat_from(Nn, r, tilt)); scl.append(sc); pid.append(ids)

# crushed / abandoned vehicles along kerbs, utility poles (many leaning or down)
lay = json.load(open('build/outer_layout_final.json'))
cars = []
for c, w in S.STREETS_X:
    for y in np.arange(-150, 150, 9.0):
        if r.random() < 0.28:
            cars.append((c + r.choice([-1, 1]) * (w / 2 - 1.4), y + r.uniform(-2, 2), 90 + r.normal(0, 12)))
for c, w in S.STREETS_Y:
    for x in np.arange(-150, 150, 9.0):
        if r.random() < 0.28:
            cars.append((x + r.uniform(-2, 2), c + r.choice([-1, 1]) * (w / 2 - 1.4), r.normal(0, 12)))
cars = np.array(cars)
ci, cj = S.w2h(cars[:, 0], cars[:, 1])
cz = cv2.remap(H, ci.astype(np.float32)[None], cj.astype(np.float32)[None], cv2.INTER_LINEAR)[0]
nc = len(cars)
yaw = np.deg2rad(cars[:, 2])
roll = np.where(r.random(nc) < 0.12, np.deg2rad(r.choice([90, 180], nc)), np.deg2rad(r.normal(0, 4, nc)))
def qmul(a, b):
    w1, v1 = a[:, :1], a[:, 1:]; w2, v2 = b[:, :1], b[:, 1:]
    return np.c_[w1 * w2 - np.sum(v1 * v2, 1, keepdims=True), w1 * v2 + w2 * v1 + np.cross(v1, v2)]
qy = np.c_[np.cos(yaw / 2), np.zeros((nc, 2)), np.sin(yaw / 2)]
qr = np.c_[np.cos(roll / 2), np.sin(roll / 2), np.zeros((nc, 2))]
q = qmul(qy, qr)
zoff = np.where(np.abs(roll) > 1.0, np.where(np.abs(roll) > 2.5, 1.65, 0.9), 0.0)
pos.append(np.c_[cars[:, :2], cz + zoff - 0.05]); ori.append(q); scl.append(np.ones((nc, 3)))
pid.append(r.choice(KIND_IDX['car'], nc))
poles = []
for c, w in S.STREETS_X:
    for y in np.arange(-155, 155, 24.0):
        if r.random() < 0.7: poles.append((c + w / 2 + 0.4, y + r.uniform(-3, 3)))
for c, w in S.STREETS_Y:
    for x in np.arange(-155, 155, 24.0):
        if r.random() < 0.7: poles.append((x + r.uniform(-3, 3), c - w / 2 - 0.4))
poles = np.array(poles); npo = len(poles)
pi_, pj_ = S.w2h(poles[:, 0], poles[:, 1])
pz = cv2.remap(H, pi_.astype(np.float32)[None], pj_.astype(np.float32)[None], cv2.INTER_LINEAR)[0]
tilt = np.deg2rad(np.where(r.random(npo) < 0.25, r.uniform(70, 88, npo), r.uniform(0, 22, npo)))
az = r.uniform(0, 2 * np.pi, npo)
ax_ = np.c_[-np.sin(az), np.cos(az), np.zeros(npo)]
qp = np.c_[np.cos(tilt / 2), ax_ * np.sin(tilt / 2)[:, None]]
pos.append(np.c_[poles, pz - 0.6]); ori.append(qp); scl.append(np.ones((npo, 3))); pid.append(np.full(npo, KIND_IDX['pole'][0]))
print('cars', nc, 'poles', npo)

pos = np.concatenate(pos).astype(np.float32); ori = np.concatenate(ori).astype(np.float32)
scl = np.concatenate(scl).astype(np.float32); pid = np.concatenate(pid).astype(np.int32)
pickle.dump(dict(protos=[(p[0], p[1], p[2]) for p in PROTOS], pos=pos, ori=ori, scale=scl, pid=pid),
            open('build/debris.pkl', 'wb'))
print('instances', len(pos), 'prototypes', len(PROTOS), 'proto tris', sum(len(p[2][1]) for p in PROTOS))
