"""Seeded 3x3 rubble-grid scenario, shared by the numpy GridWorld and the Isaac scene builder.

Site frame (contracts/INTERFACES.md): metres, x east, y north, z up. Pile names: columns A/B/C
west -> east, rows 1/2/3 north -> south, so A1 is the north-west pile. Staging area is south of row 3.

Everything here is geometry except ``Scenario.truth``, which is HIDDEN GROUND TRUTH: only the
observation generators (sim/radar_model.py, acoustic synthesis) and evaluators may read it.

    from sim.scenario import make_scenario
    sc = make_scenario(seed=7)                # or make_scenario(7, start_pile="B2")
    sc.ground_height(x, y)                    # bare mounds, no pieces (vectorised)
    grid = sc.heightmap(res=0.5)              # mounds + rasterised pieces (sim.world.Grid)
    sc.public_json()                          # piles + pieces, no truth (safe for planners / Isaac)
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field

import numpy as np

SPACING = 22.0
COLS = "ABC"
BASE_XY = (0.0, -47.0)                     # staging area (coordinator, operator dashboard, acoustic agent)
SITE_BOUNDS = (-36.0, -54.0, 36.0, 36.0)   # xmin, ymin, xmax, ymax
DRONE_PADS = [(-2.6, -49.0), (0.0, -49.0), (2.6, -49.0)]
# Case study anchor: Weiguan Jinlong building collapse, Yongkang District, Tainan (6 Feb 2016).
# zh.wikipedia 維冠金龍大樓: 23.0051432 N, 120.2608904 E. Site frame origin = this point; x east, y north.
SITE_GEO = {"lat": 23.0051432, "lon": 120.2608904, "bearing_deg": 0.0,
            "name": "Weiguan Jinlong collapse, Yongkang District, Tainan (2016)"}
SOUND_KINDS = ["voice", "weak_voice", "knock", "dog", "generator", "settling"]
TRANSCRIPTS = {   # what a TTS voice says for the synthetic human sources (truth; the pipeline must recover it)
    "voice": "Help! Can anyone hear me? My leg is trapped.",
    "weak_voice": "hello... is anyone there... water...",
}


@dataclass
class Piece:
    kind: str            # slab | chunk | column | wall
    center: tuple        # x, y, z (m)
    size: tuple          # full extents along the local x, y, z axes (m)
    rpy_deg: tuple       # roll, pitch, yaw (deg), applied as R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    material: str        # concrete | brick


@dataclass
class Pile:
    name: str
    x: float
    y: float
    h: float
    sx: float
    sy: float
    rot: float           # rad
    lump: tuple          # (dx, dy) of the secondary lump
    pieces: list = field(default_factory=list)


def rot_matrix(rpy_deg):
    r, p, y = np.radians(rpy_deg)
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


class Scenario:
    def __init__(self, seed, piles, truth, noise):
        self.seed = seed
        self.piles = piles
        self.truth = truth            # HIDDEN: generators / evaluators only
        self._noise = noise           # (amps, kx, ky, phases) for deterministic ground undulation

    # ---------------------------------------------------------------- geometry
    def pile(self, name):
        return next(p for p in self.piles if p.name == name)

    def mound(self, p, x, y):
        dx, dy = x - p.x, y - p.y
        c, s = np.cos(p.rot), np.sin(p.rot)
        u, v = dx * c + dy * s, -dx * s + dy * c
        m = p.h * np.exp(-((u / p.sx) ** 2 + (v / p.sy) ** 2) * 1.4)
        m = m + 0.8 * np.exp(-((dx - p.lump[0]) ** 2 + (dy - p.lump[1]) ** 2) / 4.5)
        return np.where(dx * dx + dy * dy < 170.0, m, 0.0)

    def ground_height(self, x, y):
        """Bare terrain: the nine mounds plus gentle undulation. Pieces sit on top of this."""
        x = np.asarray(x, float)
        y = np.asarray(y, float)
        m = sum(self.mound(p, x, y) for p in self.piles)
        a, kx, ky, ph = self._noise
        n = sum(a[i] * np.sin(kx[i] * x + ky[i] * y + ph[i]) for i in range(len(a)))
        rough = np.clip(m / 0.9, 0, 1) * sum(0.08 * np.sin(kx[i] * 7 * x + ky[i] * 7 * y + ph[i] * 3) for i in range(len(a)))
        return m + 0.03 * n + rough

    def heightmap(self, res=0.5, bounds=SITE_BOUNDS):
        """sim.world.Grid of the top surface: terrain plus the upward-facing faces of every piece."""
        from .world import Grid
        x0, y0, x1, y1 = bounds
        w, h = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
        g = Grid((x0, y0), res, w, h, np.zeros((h, w), np.float32))
        X, Y = g.mesh()
        z = self.ground_height(X, Y).astype(np.float32)
        for p in self.piles:
            for pc in p.pieces:
                for pt in _top_face_points(pc, step=res * 0.5):
                    i = int((pt[0] - x0) / res)
                    j = int((pt[1] - y0) / res)
                    if 0 <= i < w and 0 <= j < h and pt[2] > z[j, i]:
                        z[j, i] = pt[2]
        g.z = z
        return g

    # ---------------------------------------------------------------- export
    def public_dict(self):
        return {
            "schema": "larpverse.scenario/v1", "seed": self.seed, "spacing": SPACING,
            "bounds": list(SITE_BOUNDS), "geo": dict(SITE_GEO), "base_xy": list(BASE_XY), "drone_pads": [list(p) for p in DRONE_PADS],
            "start_pile": self.truth["start_pile"],   # where the mission begins: an operator choice, not truth
            "noise": [list(map(float, v)) for v in self._noise],
            "piles": [{**{k: v for k, v in asdict(p).items() if k != "pieces"},
                       "pieces": [asdict(pc) for pc in p.pieces]} for p in self.piles],
        }

    def public_json(self, **kw):
        return json.dumps(self.public_dict(), **kw)

    def truth_json(self, **kw):
        return json.dumps(self.truth, **kw)


def _top_face_points(pc: Piece, step=0.25):
    """Points on the faces of a box piece whose normals point upwards (for rasterising)."""
    R = rot_matrix(pc.rpy_deg)
    c = np.asarray(pc.center)
    half = np.asarray(pc.size) / 2
    out = []
    for axis in range(3):
        for sgn in (-1, 1):
            n = R[:, axis] * sgn
            if n[2] < 0.25:
                continue
            a, b = [k for k in range(3) if k != axis]
            na, nb = max(2, int(2 * half[a] / step) + 1), max(2, int(2 * half[b] / step) + 1)
            ua, ub = np.meshgrid(np.linspace(-half[a], half[a], na), np.linspace(-half[b], half[b], nb))
            local = np.zeros((ua.size, 3))
            local[:, axis] = sgn * half[axis]
            local[:, a] = ua.ravel()
            local[:, b] = ub.ravel()
            out.append(local @ R.T + c)
    return np.concatenate(out) if out else np.zeros((0, 3))


def make_scenario(seed: int = 7, start_pile: str | None = None) -> Scenario:
    rng = np.random.default_rng(seed)
    U = rng.uniform
    noise = (U(0.3, 1.0, 5), U(-0.4, 0.4, 5), U(-0.4, 0.4, 5), U(0, 2 * np.pi, 5))
    piles = []
    for r in range(3):
        for c in range(3):
            piles.append(Pile(name=f"{COLS[c]}{r + 1}", x=(c - 1) * SPACING + U(-1.5, 1.5), y=(1 - r) * SPACING + U(-1.5, 1.5),
                              h=U(2.6, 3.7), sx=U(5.0, 6.2), sy=U(4.4, 5.6), rot=U(0, np.pi), lump=(U(-2, 2), U(-2, 2))))
    sc = Scenario(seed, piles, {}, noise)

    # pieces sit on the bare mounds
    for p in piles:
        brick = U(0.05, 0.5)
        mat = lambda: "brick" if rng.uniform() < brick else "concrete"
        for _ in range(40):
            rr, a = 6.0 * rng.uniform() ** 0.7, U(0, 2 * np.pi)
            x, y = p.x + rr * np.cos(a), p.y + rr * np.sin(a)
            size = (U(1.2, 3.4), U(0.8, 2.2), U(0.14, 0.3))
            z = float(sc.ground_height(x, y)) + U(-0.15, 0.2)
            p.pieces.append(Piece("slab", (x, y, z), size, (U(-32, 32), U(-32, 32), U(0, 360)), mat()))
        for _ in range(26):
            rr, a = 7.2 * rng.uniform() ** 0.6, U(0, 2 * np.pi)
            x, y = p.x + rr * np.cos(a), p.y + rr * np.sin(a)
            s = U(0.3, 1.0)
            size = (s, s * U(0.6, 1.3), s * U(0.45, 1.0))
            p.pieces.append(Piece("chunk", (x, y, float(sc.ground_height(x, y)) + size[2] * 0.25), size,
                                  (U(0, 360), U(0, 360), U(0, 360)), mat()))
        for _ in range(int(rng.integers(0, 3))):
            rr, a = U(0.5, 5.0), U(0, 2 * np.pi)
            x, y = p.x + rr * np.cos(a), p.y + rr * np.sin(a)
            p.pieces.append(Piece("column", (x, y, float(sc.ground_height(x, y)) + 0.2), (U(2.5, 4.5), 0.5, 0.5),
                                  (0, U(-15, 15), U(0, 360)), "concrete"))
        if rng.uniform() < 0.5:
            a = U(0, 2 * np.pi)
            cx, cy, d = p.x + 4.6 * np.cos(a), p.y + 4.6 * np.sin(a), (-np.sin(a), np.cos(a))
            for k, hgt in enumerate([3.4, 3.9, 2.0, 0.0, 2.9, 1.6]):
                if not hgt:
                    continue
                o = (k - 2.5) * 0.86
                x, y = cx + d[0] * o, cy + d[1] * o
                p.pieces.append(Piece("wall", (x, y, float(sc.ground_height(x, y)) - 0.25 + hgt / 2), (0.86, 0.3, hgt),
                                      (0, 0, float(np.degrees(a)) + 90), "brick"))

    # ------------------------------------------------ hidden truth (generators / evaluators only)
    names = [p.name for p in piles]
    start = start_pile or names[int(rng.integers(0, 9))]
    others = [n for n in names if n != start]
    rng.shuffle(others)
    roles = dict(zip(others, SOUND_KINDS + ["silent"] * (len(others) - len(SOUND_KINDS))))
    victims, sounds = [], []

    def spot(name, rmax=2.8):
        p = sc.pile(name)
        a, rr = U(0, 2 * np.pi), U(0.5, rmax)
        return [float(p.x + rr * np.cos(a)), float(p.y + rr * np.sin(a))]

    victims.append({"id": "V1", "pile": start, "xy": spot(start), "depth": float(U(1.2, 1.9)),
                    "breathing_hz": float(U(0.2, 0.33)), "sound": None})
    for name, kind in roles.items():
        if kind == "silent":
            continue
        xy = spot(name)
        snd = {"pile": name, "kind": kind, "xy": xy, "level_db": float(U(-6, 0))}
        if kind in TRANSCRIPTS:
            snd["text"] = TRANSCRIPTS[kind]
        sounds.append(snd)
        if kind in ("voice", "weak_voice", "knock"):
            victims.append({"id": f"V{len(victims) + 1}", "pile": name, "xy": xy, "depth": float(U(0.8, 1.8)),
                            "breathing_hz": float(U(0.2, 0.33)), "sound": kind})
    sc.truth = {"start_pile": start, "roles": roles, "victims": victims, "sounds": sounds}
    return sc


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Write the public scenario (and optionally the hidden truth) to JSON.")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--start-pile")
    ap.add_argument("--out", default="scenario.json")
    ap.add_argument("--truth-out", help="separate file for hidden truth (generators/evaluators only)")
    a = ap.parse_args()
    sc = make_scenario(a.seed, a.start_pile)
    with open(a.out, "w") as f:
        f.write(sc.public_json(indent=1))
    if a.truth_out:
        with open(a.truth_out, "w") as f:
            f.write(sc.truth_json(indent=1))
    print(f"seed {a.seed}: start {sc.truth['start_pile']}, {sum(len(p.pieces) for p in sc.piles)} pieces -> {a.out}")
