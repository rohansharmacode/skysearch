"""Ground truth for a partially collapsed 3-storey office building after an earthquake,
plus every sensor model. The swarm code never reads this module's arrays directly:
it only receives sensor readings returned by the methods below.

The building is represented as a grid of CLEARANCE values (metres of free vertical space
at each 0.4 m cell on the ground floor):
    99   outside the building (open air)
    2.8  open room / corridor
    0.75 under a desk (people shelter here: "drop, cover, hold on")
    0.2-1.2  rubble voids where upper floors pancaked (only small drones fit)
    0    wall / solid debris
A drone can enter a cell only if its clearance >= the drone's min_clear.
"""
from __future__ import annotations
import math
import numpy as np
from .grid import dist_field
from scipy.ndimage import gaussian_filter, binary_dilation

from .config import USARConfig

OUT = 99.0
ROOM = 2.8


class Building:
    def __init__(self, cfg: USARConfig, rng: np.random.Generator):
        self.cfg, self.rng = cfg, rng
        H, W = cfg.H, cfg.W
        c = np.full((H, W), OUT)
        # footprint: cols 10..80, rows 8..52  (28 m x 17.6 m)
        self.x0, self.x1, self.y0, self.y1 = 10, 80, 8, 52
        c[self.y0:self.y1, self.x0:self.x1] = ROOM
        # exterior walls
        c[self.y0, self.x0:self.x1] = 0; c[self.y1 - 1, self.x0:self.x1] = 0
        c[self.y0:self.y1, self.x0] = 0; c[self.y0:self.y1, self.x1 - 1] = 0
        # central corridor (rows 28..32) and rooms either side
        for r in (27, 33):
            c[r, self.x0:self.x1] = 0
        for x in range(self.x0 + 14, self.x1 - 1, 14):
            c[self.y0:27, x] = 0
            c[33:self.y1, x] = 0
        # doors into corridor (3 cells = 1.2 m) and main entrance + broken windows
        for x in range(self.x0 + 5, self.x1 - 1, 14):
            c[27, x:x + 3] = ROOM; c[33, x:x + 3] = ROOM
        c[28:33, self.x0] = ROOM                 # main entrance (west)
        self.entrance = np.array([self.x0 * cfg.cell - 0.6, 30.5 * cfg.cell])
        for (r, x) in [(self.y0, 30), (self.y1 - 1, 55), (self.y0, 68)]:
            c[r, x:x + 3] = ROOM                  # blown-out windows
        # desks (people shelter under them)
        self.desk_cells = []
        for _ in range(26):
            r = int(rng.integers(self.y0 + 2, self.y1 - 3)); x = int(rng.integers(self.x0 + 2, self.x1 - 4))
            if 27 <= r <= 33:
                continue
            if (c[r:r + 2, x:x + 3] == ROOM).all():
                c[r:r + 2, x:x + 3] = 0.75
                self.desk_cells.append((r, x))
        self.clear = c
        # rooms: north band 1-5 and south band 6-10 (west to east), corridor = 0
        self.room_id = np.full((H, W), -1, int)
        self.rooms = {}
        cols = list(range(self.x0, self.x1, 14))
        for band, (r0, r1) in enumerate([(self.y0 + 1, 27), (34, self.y1 - 1)]):
            for k, cx in enumerate(cols):
                rid = 1 + band * 5 + k
                c0, c1 = cx + 1, min(cx + 14, self.x1 - 1)
                self.room_id[r0:r1, c0:c1] = rid
                self.rooms[rid] = dict(rows=(r0, r1), cols=(c0, c1),
                                       center=np.array([(c0 + c1) / 2 * cfg.cell, (r0 + r1) / 2 * cfg.cell]),
                                       wing="west" if rid in (1, 2, 6, 7) else "east")
        self.room_id[28:33, self.x0 + 1:self.x1 - 1] = 0
        self.rooms[0] = dict(rows=(28, 33), cols=(self.x0 + 1, self.x1 - 1),
                             center=np.array([(self.x0 + self.x1) / 2 * cfg.cell, 30.5 * cfg.cell]), wing="both")
        # main pancake collapse in the east wing: upper floors came down
        self.collapse((float(rng.uniform(55, 70)) * cfg.cell, float(rng.uniform(16, 44)) * cfg.cell), radius=6.5, severity=1.0)
        self.collapse((float(rng.uniform(22, 40)) * cfg.cell, float(rng.uniform(12, 22)) * cfg.cell), radius=3.5, severity=0.7)
        self.dust = np.zeros((H, W))
        self.dust[self.y0:self.y1, self.x0:self.x1] = 0.15
        self.shake_until = -1.0
        self.last_shock = None

        # survivors: mostly in voids or under desks, some conscious (make noise)
        self.survivors = []
        void = (self.clear >= 0.25) & (self.clear < 0.9)
        # only voids a crevice drone could physically reach from outside (through any gap >= 0.25 m)
        reach = np.isfinite(dist_field(self.clear >= 0.25, self.cell_of(self.entrance)))
        cand = np.argwhere(void & reach)
        for i in range(cfg.n_survivors):
            for _ in range(200):           # people are spread out: at least 3 m apart
                r, x = cand[rng.integers(len(cand))]
                if all(np.linalg.norm(self.center((r, x)) - s["pos"]) > 3.0 for s in self.survivors):
                    break
            self.survivors.append(dict(pos=self.center((r, x)), conscious=i != 2, next_sound=float(rng.uniform(3, 15)),
                                       found_t=None, buried=float(rng.uniform(0.0, 0.5)), name="ABCDEFG"[i], wake_t=None))
        # decoys: mannequins / piles of clothing that look human on camera
        open_ = np.argwhere((self.clear >= 0.25) & (self.clear < OUT))
        self.decoys = [dict(pos=self.center(tuple(open_[rng.integers(len(open_))]))) for _ in range(cfg.n_decoys)]
        self.events = []      # (t, text) ground-truth events for the log
        self.smoke = np.zeros((H, W))
        self.fires = []
        self.callout_t = None
        rr, cc = np.mgrid[0:H, 0:W]
        self._XC, self._YC = (cc + 0.5) * cfg.cell, (rr + 0.5) * cfg.cell
        if cfg.story:
            self.place_story()

    # -------------------------------------------------------------- storyboard placement
    def place_story(self):
        """Victim A: west wing void, conscious and knocking. Victim B: room 7, behind a collapsed shelf
        (no camera line of sight, no way in), unconscious until cfg.t_knock_b. Victim C: inside the
        east-wing pancake collapse, silent."""
        cfg, rng = self.cfg, self.rng
        reach = np.isfinite(dist_field(self.clear >= 0.25, self.cell_of(self.entrance)))
        void = (self.clear >= 0.25) & (self.clear < 0.9)
        west = np.isin(self.room_id, [1, 2, 6])
        cand = np.argwhere(void & reach & west)
        if not len(cand):
            cand = np.argwhere(void & reach)
        a = self.center(tuple(cand[rng.integers(len(cand))]))
        # victim B: shelf fell against the south wall of room 7, leaving a small void behind it
        r0, r1 = self.rooms[7]["rows"]; c0, c1 = self.rooms[7]["cols"]
        br, bc = r1 - 1, (c0 + c1) // 2 + 2
        self.clear[br - 2, bc - 2:bc + 3] = 0.0            # shelf top (fallen)
        self.clear[br - 2:br + 1, bc - 2] = 0.0
        self.clear[br - 2:br + 1, bc + 2] = 0.0
        self.clear[br - 1:br + 1, bc - 1:bc + 2] = 0.5     # the void she is in
        b = self.center((br, bc))
        # victim C: enclosed void in the east wing collapse
        east = np.isin(self.room_id, [4, 5, 9, 10])
        cc = np.argwhere(void & ~reach & east)
        if not len(cc):
            cc = np.argwhere(void & east)
        c = self.center(tuple(cc[rng.integers(len(cc))]))
        mk = lambda name, pos, cons, wake, buried: dict(pos=pos, conscious=cons, next_sound=float(rng.uniform(4, 8)),
                                                        found_t=None, buried=buried, name=name, wake_t=wake)
        self.survivors = [mk("A", a, True, None, 0.1), mk("B", b, False, cfg.t_knock_b, 0.6), mk("C", c, False, None, 0.5)]
        self.story_fire = self.center((int(np.mean(self.rooms[5]["rows"])), int(np.mean(self.rooms[5]["cols"]))))

    def wing_of(self, xy):
        r, c = self.cell_of(xy)
        rid = int(self.room_id[r, c])
        if rid > 0:
            return self.rooms[rid]["wing"]
        return "east" if c >= self.rooms[3]["cols"][0] else "west"

    def start_fire(self, xy, t):
        self.fires.append(dict(xy=np.asarray(xy, float), t=t))

    # -------------------------------------------------------------- geometry
    def center(self, rc):
        return np.array([(rc[1] + 0.5) * self.cfg.cell, (rc[0] + 0.5) * self.cfg.cell])

    def cell_of(self, xy):
        c = self.cfg.cell
        return (int(np.clip(xy[1] // c, 0, self.cfg.H - 1)), int(np.clip(xy[0] // c, 0, self.cfg.W - 1)))

    def inside(self, xy):
        r, c = self.cell_of(xy)
        return self.y0 <= r < self.y1 and self.x0 <= c < self.x1

    def line_cells(self, a, b):
        n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) / (self.cfg.cell * 0.5)) + 2
        ts = np.linspace(0, 1, n)
        pts = a[None] + ts[:, None] * (b - a)[None]
        rr = np.clip((pts[:, 1] // self.cfg.cell).astype(int), 0, self.cfg.H - 1)
        cc = np.clip((pts[:, 0] // self.cfg.cell).astype(int), 0, self.cfg.W - 1)
        keep = np.ones(len(rr), bool); keep[1:] = (rr[1:] != rr[:-1]) | (cc[1:] != cc[:-1])
        return rr[keep], cc[keep]

    # -------------------------------------------------------------- dynamics
    def collapse(self, xy, radius=3.0, severity=0.8, t=0.0):
        """Upper-floor slabs drop: clearance falls to a rubble field with voids."""
        cfg = self.cfg
        rr, cc = np.mgrid[0:cfg.H, 0:cfg.W]
        d = np.hypot((cc + 0.5) * cfg.cell - xy[0], (rr + 0.5) * cfg.cell - xy[1])
        zone = (d < radius) & (self.clear < OUT)
        noise = gaussian_filter(self.rng.random((cfg.H, cfg.W)), 1.2)
        noise = (noise - noise.min()) / (noise.max() - noise.min() + 1e-9)
        rubble = np.where(noise > 0.62, 0.0, 0.15 + 1.1 * noise)        # solid chunks + voids
        rubble = np.where(d < radius * 0.35 * severity, np.minimum(rubble, 0.6), rubble)
        self.clear = np.where(zone, np.minimum(self.clear, rubble * (1.3 - 0.3 * severity) + 0.0), self.clear)
        self.clear[self.clear < 0.12] = 0.0
        for s in getattr(self, "survivors", []):      # people who survive are in a survivable void
            r, c = self.cell_of(s["pos"])
            self.clear[r, c] = max(self.clear[r, c], 0.3)
        return zone

    def aftershock(self, t, xy=None, radius=None):
        cfg = self.cfg
        if xy is None:
            xy = np.array([self.rng.uniform(self.x0 + 6, self.x1 - 6), self.rng.uniform(self.y0 + 5, self.y1 - 5)]) * cfg.cell
        radius = radius or float(self.rng.uniform(2.5, 4.0))
        zone = self.collapse(xy, radius, severity=0.8, t=t)
        rr, cc = np.mgrid[0:cfg.H, 0:cfg.W]
        d = np.hypot((cc + 0.5) * cfg.cell - xy[0], (rr + 0.5) * cfg.cell - xy[1])
        self.dust = np.clip(self.dust + 0.9 * np.exp(-(d / (radius * 2.2)) ** 2) * (self.clear < OUT), 0, 1)
        self.shake_until = t + 4.0
        self.last_shock = dict(t=t, xy=np.asarray(xy, float), r=radius)
        for s in self.survivors:
            if np.linalg.norm(s["pos"] - xy) < radius and self.rng.random() < 0.5 and s["conscious"]:
                s["conscious"] = False      # knocked unconscious: stops making noise
        return xy, radius, zone

    def step(self, t):
        # dust slowly settles and spreads
        self.dust = np.clip(gaussian_filter(self.dust, 0.6) * 0.996, 0, 1)
        self.dust[self.clear >= OUT] *= 0.5
        # smoke from fires: pours out of the fire, flows through open space, blocked by solid walls
        if self.fires:
            cfg = self.cfg
            for f in self.fires:
                d = np.hypot(self._XC - f["xy"][0], self._YC - f["xy"][1])
                self.smoke += (1.2 if t - f["t"] < 10 else 0.5) * cfg.dt * np.exp(-(d / 1.8) ** 2)
            open_ = (self.clear >= 0.12).astype(float)
            sm = gaussian_filter(self.smoke * open_, 1.1)
            norm = gaussian_filter(open_, 1.1) + 1e-6
            self.smoke = np.clip(sm / norm * open_ * 0.999, 0, 1)
            self.smoke[self.clear >= OUT] *= 0.3
        for s in self.survivors:
            if s.get("wake_t") is not None and not s["conscious"] and t >= s["wake_t"]:
                s["conscious"] = True
                s["next_sound"] = t + 0.5

    def shaking(self, t):
        return t < self.shake_until

    # -------------------------------------------------------------- sensors
    def imu_shock(self, t):
        """Every drone's IMU feels the aftershock (vibration through the air + debris)."""
        return self.shaking(t)

    def lidar(self, pos, rays, rng_m):
        """3-D LiDAR at ~1.2 m: returns observed cells with measured clearance.
        Beams stop at anything lower than the sensor; dust scatters beams."""
        cfg = self.cfg
        r0, c0 = self.cell_of(pos)
        dust = self.dust[r0, c0]
        smoke = self.smoke[r0, c0]
        rmax = max(0.6, rng_m * (1 - 0.7 * dust - 0.8 * smoke))
        a = 2 * np.pi * np.arange(rays) / rays
        steps = np.arange(0, rmax + 1e-6, cfg.cell * 0.5)
        pts = pos[None, None, :] + steps[None, :, None] * np.stack([np.cos(a), np.sin(a)], 1)[:, None, :]
        rr = np.clip((pts[..., 1] // cfg.cell).astype(int), 0, cfg.H - 1)
        cc = np.clip((pts[..., 0] // cfg.cell).astype(int), 0, cfg.W - 1)
        v = self.clear[rr, cc]
        block = v < 1.0
        first = np.where(block.any(1), block.argmax(1), len(steps) - 1)
        good = self.rng.random(rays) >= (0.6 * dust + 0.85 * smoke)
        mask = (np.arange(len(steps))[None, :] <= first[:, None]) & good[:, None]
        last = np.minimum(first, len(steps) - 1)
        ends = pts[np.arange(rays), last][good]
        nbad = int((~good).sum() * min(1.0, 2 * smoke))
        if nbad:          # smoke backscatter: spurious returns at random ranges (rejected by SLAM, shown on screen)
            ang = self.rng.uniform(0, 2 * np.pi, nbad); rad = self.rng.uniform(0.3, rmax, nbad)
            ends = np.vstack([ends, pos + np.stack([np.cos(ang), np.sin(ang)], 1) * rad[:, None]
                              + self.rng.normal(0, 0.15, (nbad, 2))])
        self.last_lidar_ends = ends                                       # beam end points (for display)
        self.last_lidar_quality = float(good.mean())
        rr, cc, v = rr[mask], cc[mask], v[mask]
        noisy = np.where(v >= OUT, v, np.maximum(0.0, v + self.rng.normal(0, 0.04, v.shape)))
        return rr, cc, noisy

    def ultrasound(self, pos, rng_m, n=8):
        """Short-range sonar ring: distance to the nearest obstacle (< 0.9 m clearance) per sector.
        Unaffected by dust or darkness."""
        d = np.full(n, rng_m)
        for k in range(n):
            a = 2 * math.pi * k / n
            rr, cc = self.line_cells(pos, pos + rng_m * np.array([math.cos(a), math.sin(a)]))
            for i, (r, c) in enumerate(zip(rr, cc)):
                if self.clear[r, c] < 0.9:
                    d[k] = np.hypot((c + 0.5) * self.cfg.cell - pos[0], (r + 0.5) * self.cfg.cell - pos[1])
                    break
        return d

    def _disk(self, R):
        """Cached cell offsets for a disk of radius R and the cells on the line to each one."""
        if not hasattr(self, "_disk_cache"):
            self._disk_cache = {}
        if R not in self._disk_cache:
            offs = [(dr, dc) for dr in range(-R, R + 1) for dc in range(-R, R + 1) if dr * dr + dc * dc <= R * R]
            K = 2 * R + 2
            OR = np.zeros((len(offs), K), int); OC = np.zeros((len(offs), K), int)
            for i, (dr, dc) in enumerate(offs):
                n = 2 * max(abs(dr), abs(dc)) + 2
                ts = np.linspace(0, 1, n)[:-1]
                rr, cc = np.round(ts * dr).astype(int), np.round(ts * dc).astype(int)
                tgt = (rr == dr) & (cc == dc)
                rr[tgt] = 0; cc[tgt] = 0
                OR[i, :len(rr)] = rr; OC[i, :len(cc)] = cc
            self._disk_cache[R] = (OR, OC, np.array(offs))
        return self._disk_cache[R]

    def local_map(self, pos, rng_m, min_clear):
        """Micro-drone ToF + camera depth: cells within range and line of sight."""
        cfg = self.cfg
        R = int(rng_m / cfg.cell)
        r0, c0 = self.cell_of(pos)
        OR, OC, T = self._disk(R)
        tr, tc = r0 + T[:, 0], c0 + T[:, 1]
        inb = (tr >= 0) & (tr < cfg.H) & (tc >= 0) & (tc < cfg.W)
        rr = np.clip(r0 + OR, 0, cfg.H - 1); cc = np.clip(c0 + OC, 0, cfg.W - 1)
        ok = inb & ~(self.clear[rr, cc] < min_clear * 0.6).any(1)
        tr, tc = tr[ok], tc[ok]
        return tr, tc, self.clear[tr, tc]

    def camera(self, pos, heading, rng_m, fov_deg):
        """Person detector on the micro-drone camera (with LED). Returns list of (xy, score).
        Real survivors show breathing/micro-motion up close; mannequins and clothes don't."""
        out = []
        r0, c0 = self.cell_of(pos)
        vis = rng_m * max(0.0, 1 - 0.75 * self.dust[r0, c0] - 0.95 * self.smoke[r0, c0])
        for kind, items in (("person", self.survivors), ("decoy", self.decoys)):
            for s in items:
                v = s["pos"] - pos
                d = float(np.linalg.norm(v))
                if d > vis or d < 1e-6:
                    continue
                ang = abs((math.atan2(v[1], v[0]) - heading + math.pi) % (2 * math.pi) - math.pi)
                if ang > math.radians(fov_deg / 2):
                    continue
                rr, cc = self.line_cells(pos, s["pos"])
                if (self.clear[rr[1:-1], cc[1:-1]] < 0.12).any():
                    continue
                buried = s.get("buried", 0.0)
                if self.rng.random() > 0.7 * (1 - buried) * (1 - 0.6 * self.dust[r0, c0]):
                    continue
                close = d < 1.6       # breathing / micro-motion resolvable from ~1.5 m with the LED on
                if kind == "person":
                    mu = 0.93 if close else 0.72
                else:
                    mu = 0.25 if close else 0.62
                out.append((s["pos"] + self.rng.normal(0, 0.1, 2), float(np.clip(self.rng.normal(mu, 0.08), 0.01, 0.99))))
        return out

    def radar_breathing(self, pos, rng_m, max_solid):
        """Breathing radar (UWB / 60 GHz mmWave): chest motion of 0.2-0.5 Hz is visible through rubble.
        Works in smoke, dust and darkness. Mannequins and clothes don't breathe."""
        out = []
        for s in self.survivors:
            d = float(np.linalg.norm(s["pos"] - pos))
            if d > rng_m:
                continue
            rr, cc = self.line_cells(pos, s["pos"])
            solid = int((self.clear[rr[1:-1], cc[1:-1]] < 0.12).sum())
            if solid > max_solid or self.rng.random() > 0.85 - 0.15 * solid:
                continue
            out.append((s["pos"] + self.rng.normal(0, 0.12 + 0.05 * d, 2), solid))
        return out

    def radar_map(self, pos, rng_m, max_solid=2):
        """Coarse occupancy from the UWB radar: sees through smoke and thin walls, low resolution."""
        cfg = self.cfg
        R = int(rng_m / cfg.cell)
        r0, c0 = self.cell_of(pos)
        OR, OC, T = self._disk(R)
        tr, tc = r0 + T[:, 0], c0 + T[:, 1]
        inb = (tr >= 0) & (tr < cfg.H) & (tc >= 0) & (tc < cfg.W)
        rr = np.clip(r0 + OR, 0, cfg.H - 1); cc = np.clip(c0 + OC, 0, cfg.W - 1)
        ok = inb & ((self.clear[rr, cc] < 0.12).sum(1) <= max_solid)
        tr, tc = tr[ok], tc[ok]
        v = self.clear[tr, tc]
        coarse = np.where(v >= OUT, OUT, np.where(v >= 0.9, 1.6, np.where(v >= 0.25, 0.5, 0.0)))
        return tr, tc, coarse

    def sound_events(self, t):
        """Conscious survivors tap on pipes / call out every 8-20 s."""
        ev = []
        if self.callout_t is None:
            return ev          # people knock in reply to the rescuers' call-out
        for s in self.survivors:
            if s["conscious"] and s["found_t"] is None and t >= max(s["next_sound"], self.callout_t + 2):
                s["next_sound"] = t + float(self.rng.uniform(5, 12))
                ev.append(s)
        return ev

    def hear(self, pos, src, quiet):
        """Microphone array: bearing to a sound source, or None. Sound loses ~1.5 dB per
        rubble cell and 6 dB per distance doubling; rotor noise masks it unless the drone
        is perched in a quiet period."""
        rr, cc = self.line_cells(pos, src["pos"])
        solid = int((self.clear[rr, cc] < 0.2).sum())
        d = max(float(np.linalg.norm(src["pos"] - pos)), 0.3)
        level = 85 - 20 * math.log10(d) - 1.5 * solid
        floor = 38 if quiet else 58
        if level + self.rng.normal(0, 3) < floor:
            return None
        v = src["pos"] - pos
        sigma = math.radians(10 + 1.2 * solid)
        return math.atan2(v[1], v[0]) + self.rng.normal(0, sigma), sigma, level

    def path_loss(self, a, b):
        """Radio path loss (dB): free space + per-wall and per-rubble attenuation."""
        rr, cc = self.line_cells(a, b)
        v = self.clear[rr, cc]
        walls = int(((v < 0.12)).sum())
        rubble = int(((v >= 0.12) & (v < 1.0)).sum())
        d = max(float(np.linalg.norm(b - a)), 0.5)
        return 40 + 20 * math.log10(d) + 1.6 * walls + 0.8 * rubble
