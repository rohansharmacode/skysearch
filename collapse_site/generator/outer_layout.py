"""Procedural, seeded layout of the wrecked neighbourhood around the scanned core."""
import json, numpy as np
import sitecfg as S

STOREY = 3.1


def make_layout(seed=11):
    rng = np.random.default_rng(seed)
    lots = []
    for (x0, y0, x1, y1) in S.blocks():
        w, d = x1 - x0, y1 - y0
        # split the block into 1 or 2 rows along its short side, lots along its long side
        long_is_x = w >= d
        L0, L1 = (x0, x1) if long_is_x else (y0, y1)
        D0, D1 = (y0, y1) if long_is_x else (x0, x1)
        depth = D1 - D0
        rows = [(D0, D1)] if depth < 30 else [(D0, D0 + depth / 2 - 0.5), (D0 + depth / 2 + 0.5, D1)]
        for (r0, r1) in rows:
            p = L0
            while p < L1 - 6:
                lw = float(rng.uniform(10, 22))
                if L1 - (p + lw) < 8:
                    lw = L1 - p
                q = p + lw
                # inset
                a0, a1 = p + rng.uniform(0.6, 1.6), q - rng.uniform(0.6, 1.6)
                b0, b1 = r0 + rng.uniform(1.0, 2.5), r1 - rng.uniform(1.0, 2.5)
                if b1 - b0 > 9 and rng.random() < 0.35:            # backyard / shallower building
                    if rng.random() < 0.5: b1 -= rng.uniform(2, (b1 - b0) * 0.35)
                    else: b0 += rng.uniform(2, (b1 - b0) * 0.35)
                rect = (a0, b0, a1, b1) if long_is_x else (b0, a0, b1, a1)
                lots.append(rect)
                p = q
    feats = []
    for rect in lots:
        x0, y0, x1, y1 = rect
        if min(x1 - x0, y1 - y0) < 5:
            continue
        r = rng.random()
        typ = ('standing' if r < 0.27 else 'tilted' if r < 0.39 else 'pancake' if r < 0.57
               else 'collapsed' if r < 0.91 else 'open')
        area = (x1 - x0) * (y1 - y0)
        if typ in ('pancake', 'tilted') and area > 380:
            typ = 'collapsed' if rng.random() < 0.6 else 'standing'
        f = dict(type=typ, rect=[round(v, 3) for v in rect], seed=int(rng.integers(1 << 30)),
                 yaw=float(rng.uniform(-3.5, 3.5)))
        if typ == 'standing':
            f['storeys'] = int(rng.integers(2, 6))
            f['damage'] = float(rng.uniform(0.35, 0.9))
        elif typ == 'tilted':
            f['storeys'] = int(rng.integers(2, 5))
            f['tilt_deg'] = float(rng.uniform(5, 13))
            f['tilt_dir'] = int(rng.integers(4))           # 0:+x 1:-x 2:+y 3:-y
            f['sink'] = float(rng.uniform(0.6, 1.8))
            f['damage'] = float(rng.uniform(0.25, 0.6))
        elif typ == 'pancake':
            f['floors'] = int(rng.integers(3, 6))
            f['damage'] = float(rng.uniform(0.2, 0.5))
        elif typ == 'collapsed':
            f['height'] = float(rng.uniform(2.2, 5.5))
            f['walls'] = int(rng.integers(0, 4))
        else:
            f['mound'] = bool(rng.random() < 0.6)
        feats.append(f)
    # extra "smaller rubble mounds" (requested), spread around the ring and outer streets
    mounds = []
    tries = 0
    while len(mounds) < 34 and tries < 4000:
        tries += 1
        x, y = rng.uniform(-152, 152, 2)
        if max(abs(x), abs(y)) < 56:
            continue
        rad = float(rng.uniform(3.0, 8.5))
        if any(np.hypot(x - m['x'], y - m['y']) < m['r'] + rad + 2 for m in mounds):
            continue
        blocked = False
        for f in feats:
            if f['type'] in ('standing', 'tilted', 'pancake'):
                x0, y0, x1, y1 = f['rect']
                if x0 - rad - 1.5 < x < x1 + rad + 1.5 and y0 - rad - 1.5 < y < y1 + rad + 1.5:
                    blocked = True; break
        if blocked:
            continue
        mounds.append(dict(x=float(x), y=float(y), r=rad, h=float(rng.uniform(1.0, 3.4)),
                           seed=int(rng.integers(1 << 30))))
    # a ring of mid-size mounds in the transition band right around the scan
    for k in range(10):
        ang = 2 * np.pi * (k + rng.uniform(-0.3, 0.3)) / 10
        rr = rng.uniform(58.5, 60)
        x, y = rr * np.cos(ang), rr * np.sin(ang)
        mounds.append(dict(x=float(x), y=float(y), r=float(rng.uniform(2.5, 4.5)),
                           h=float(rng.uniform(0.8, 2.0)), seed=int(rng.integers(1 << 30))))
    return dict(features=feats, mounds=mounds)


if __name__ == '__main__':
    L = make_layout()
    from collections import Counter
    print(Counter(f['type'] for f in L['features']), len(L['mounds']), 'mounds')
    json.dump(L, open('build/outer_layout.json', 'w'), indent=1)
