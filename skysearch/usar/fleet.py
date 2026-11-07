"""Heterogeneous rescue fleet: 1 coordinator mothership, 3 mapping scouts, 6 crevice micro-drones.

Who decides what
- Every drone runs its own loop: sense, update its own copy of the map, act on its current task,
  avoid obstacles, manage battery, react to aftershocks. It keeps working when the radio drops.
- The COORDINATOR role (normally on the mothership) merges maps, fuses sound bearings into an
  acoustic map, turns the map into tasks (frontiers for scouts, voids / sounds / sightings for
  micro-drones) and auctions them. If the mothership is lost, a scout takes the role over.
- A human incident commander is asked only to confirm a find the drones are unsure about.
"""
from __future__ import annotations
import math
import time
from dataclasses import dataclass, field
import numpy as np
from scipy.ndimage import binary_dilation, binary_erosion, label, maximum_filter

from .config import USARConfig, DroneSpec
from .building import Building, OUT
from .grid import dist_field

S8 = np.ones((3, 3), bool)
# states
IDLE, EXPLORE, VOID, INVESTIGATE, CONFIRM, BEACON, ASK, RELAY, LISTEN, HOLD, RETURN, DOCKED, FAILED, COORD = (
    "IDLE", "MAP", "VOID", "INVESTIGATE", "CONFIRM", "BEACON", "ASK HUMAN", "RELAY", "LISTEN", "HOLD", "RETURN", "CHARGE",
    "FAILED", "COORDINATE")
TRAPPED = "TRAPPED"
PERCH, TOPOST, PULLOUT, HOVER, SWEEP = "LISTENING (landed)", "TO POST", "PULL OUT", "HOVER: mic + radar", "RADAR SWEEP"


class Map:
    """A drone's belief about the building: clearance per cell (NaN = never seen)."""

    def __init__(self, H, W):
        self.clear = np.full((H, W), np.nan)
        self.stamp = np.full((H, W), -1.0)
        self.stale = np.zeros((H, W), bool)

    def write(self, r, c, v, t):
        self.clear[r, c] = v
        self.stamp[r, c] = t
        self.stale[r, c] = False

    def merge_from(self, other: "Map"):
        newer = other.stamp > self.stamp
        self.clear[newer] = other.clear[newer]
        self.stamp[newer] = other.stamp[newer]
        self.stale = np.where(newer, other.stale, self.stale | other.stale)

    def copy_from(self, other: "Map"):
        self.clear = other.clear.copy(); self.stamp = other.stamp.copy(); self.stale = other.stale.copy()

    def known(self):
        return ~np.isnan(self.clear)

    nofly = None          # set by the fleet: cells too close to a fire

    def trav(self, spec: DroneSpec):
        k = self.known()
        ok = k & (np.nan_to_num(self.clear, nan=0) >= spec.min_clear)
        if Map.nofly is not None:
            ok &= ~Map.nofly
        if spec.size >= 0.5:      # wide drones need a 3-cell-wide passage
            ok = binary_erosion(ok, structure=np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], bool), border_value=1) | (ok & (np.nan_to_num(self.clear) >= OUT))
        return ok


@dataclass
class Drone:
    id: int
    name: str
    spec: DroneSpec
    pos: np.ndarray
    map: Map
    state: str = IDLE
    heading: float = 0.0
    battery: float = 1.0
    task: dict | None = None
    link: bool = True
    last_link_pos: np.ndarray | None = None
    offline_since: float | None = None
    trail: list = field(default_factory=list)
    note: str = ""
    outbox: list = field(default_factory=list)
    D: np.ndarray | None = None
    D_key: tuple | None = None
    hold_until: float = 0.0
    ultra_mode: bool = False
    lidar_hits: tuple | None = None
    cam_dets: list = field(default_factory=list)
    compute_ms: float = 0.0
    alt: float = 1.2
    home_D: np.ndarray | None = None
    home_t: float | None = None
    optimistic: bool = False
    scan_left: int = 0
    role: str = ""
    has_radar: bool = False
    health: dict = field(default_factory=dict)
    radar_dets: list = field(default_factory=list)
    bear_vis: list = field(default_factory=list)
    post: tuple | None = None
    still: int = 0
    last_pos: np.ndarray | None = None
    radar_mode: bool = False
    released: bool = False

    @property
    def kind(self):
        return self.spec.kind

    @property
    def alive(self):
        return self.state != FAILED


class Fleet:
    def __init__(self, cfg: USARConfig, bld: Building, rng):
        self.cfg, self.b, self.rng = cfg, bld, rng
        self.t = 0.0
        H, W = cfg.H, cfg.W
        self.base = np.array([1.5, bld.entrance[1] - 4.0])          # incident command post outside
        self.drones: list[Drone] = []
        mk = lambda i, name, spec, p: Drone(i, name, spec, np.array(p, float), Map(H, W))
        self.drones.append(mk(0, "M", cfg.mother, bld.entrance + [-1.8, 0]))
        for i in range(cfg.n_scouts):
            d = mk(1 + i, f"S{i + 1}", cfg.scout, bld.entrance + [-1.2, -1.2 + 1.2 * i])
            d.has_radar = (i + 1) in cfg.radar_scouts
            d.role = "radar" if d.has_radar else "mapper"
            self.drones.append(d)
        for i in range(cfg.n_micro):
            lis = i < cfg.n_listeners
            name = f"L{i + 1}" if lis else f"C{i - cfg.n_listeners + 1}"
            d = mk(1 + cfg.n_scouts + i, name, cfg.micro, bld.entrance + [-2.2, -1.5 + 0.6 * i])
            d.role = "listener" if lis else "confirmer"
            d.has_radar = (not lis) and cfg.micro_radar  # confirmers carry a 60 GHz breathing radar
            self.drones.append(d)
        self.drones[0].role = "coordinator"
        # everyone knows the outside is open air
        out = bld.clear >= OUT
        for d in self.drones:
            d.map.clear[out] = OUT; d.map.stamp[out] = 0.0
        self.coord_id = 0
        self.cmap = Map(H, W); self.cmap.copy_from(self.drones[0].map)
        self.acoustic = np.zeros((H, W))
        self.bear_groups = {}
        self.fixes = []
        self.candidates: list[dict] = []
        self.sound_tasks: list[dict] = []
        self.found: list[dict] = []
        self.log: list[tuple[float, str]] = []
        self.captions: list[tuple[float, str]] = []
        self.flags = set()
        self.seen = np.zeros((H, W), bool)              # searched for people (camera close-up or hovering radar)
        self.radar_cells = np.zeros((H, W), bool)       # mapped by radar only (coarse)
        self.room_status = {rid: "unsearched" for rid in bld.rooms if rid != 0}
        self.ban = np.full((H, W), -1.0)
        self.priority_sounds = []
        self.callout_t = None
        self.room_reopen_t = {}
        self.handoff_t = None
        self.briefing = None
        self.n_unaccounted = cfg.n_survivors
        self.next_quiet = cfg.quiet_every
        self.quiet_until = -1.0
        self.ask_queue = []
        self.reviews = 0
        self.decoys_rejected = 0
        self.lost_drones = 0
        self.aftershock_seen = False
        rr, cc = np.mgrid[0:H, 0:W]
        self.XC, self.YC = (cc + 0.5) * cfg.cell, (rr + 0.5) * cfg.cell
        # exterior stations the mothership can hover at (ring around the building)
        ring = []
        x0, x1, y0, y1 = bld.x0 * cfg.cell - 1.4, bld.x1 * cfg.cell + 1.0, bld.y0 * cfg.cell - 1.4, bld.y1 * cfg.cell + 1.0
        for x in np.arange(x0, x1, 2.0):
            ring += [(x, y0), (x, y1)]
        for y in np.arange(y0, y1, 2.0):
            ring += [(x0, y), (x1, y)]
        self.stations = np.array(ring)
        # facade survey: the mothership first circles the building with its LiDAR + camera to find
        # every opening (broken windows, wall breaches) the smaller drones can use as a way in
        ex, ey = bld.entrance
        loop = [(x0, ey), (x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, ey)]
        self.survey = []
        for (ax, ay), (bx, by) in zip(loop[:-1], loop[1:]):
            n = max(1, int(np.hypot(bx - ax, by - ay) / 1.5))
            self.survey += [np.array([ax + (bx - ax) * k / n, ay + (by - ay) * k / n]) for k in range(1, n + 1)]
        self.survey_done = False
        self.openings_seen = 0
        self.links = []

    # ------------------------------------------------------------------ helpers
    def say(self, s):
        if any(m == s and self.t - t < 25 for t, m in self.log[-40:]):
            return                                   # don't repeat the same line in the log
        self.log.append((self.t, s))

    def caption(self, text, key=None):
        """Big on-screen caption for the demo (also logged)."""
        if key is not None:
            if key in self.flags:
                return
            self.flags.add(key)
        self.captions.append((self.t, text))
        self.say("» " + text)

    def room_of(self, xy):
        return int(self.b.room_id[self.cell(xy)])

    def alive(self, kind=None):
        return [d for d in self.drones if d.alive and (kind is None or d.kind == kind)]

    def coord(self):
        return self.drones[self.coord_id]

    def cell(self, xy):
        return self.b.cell_of(xy)

    def quiet(self):
        return self.t < self.quiet_until

    def kill(self, did, why="hit by falling debris"):
        d = self.drones[did]
        if not d.alive:
            return
        d.state, d.note, d.task = FAILED, why, None
        self.lost_drones += 1
        self.say(f"{d.name} lost: {why}")
        if did == self.coord_id:
            cands = [s for s in self.alive("scout")]
            if cands:
                new = min(cands, key=lambda s: self.b.path_loss(s.pos, self.base))
                self.coord_id = new.id
                new.map.merge_from(self.cmap)
                self.say(f"Coordinator role handed over to {new.name} (it has the best link to command)")

    # ------------------------------------------------------------------ radio
    def update_links(self):
        nodes = [("base", self.base, 120.0)] + [(d.id, d.pos, d.spec.radio_db) for d in self.alive()]
        adj = {k: [] for k, _, _ in nodes}
        self.links = []
        for i, (a, pa, ra) in enumerate(nodes):
            for b_, pb, rb in nodes[i + 1:]:
                if np.linalg.norm(pa - pb) > 30:
                    continue
                pl = self.b.path_loss(pa, pb)
                thr = min(ra, rb)
                if pl < thr:
                    adj[a].append(b_); adj[b_].append(a)
                    self.links.append((pa.copy(), pb.copy(), thr - pl))
        root = self.coord_id if self.coord().alive else "base"
        if self.coord().alive and self.coord().kind == "mother":
            adj[root].append("base"); adj["base"].append(root)      # outdoor long-range link
        seen, stack = set(), [root]
        while stack:
            u = stack.pop()
            if u in seen:
                continue
            seen.add(u); stack += adj[u]
        for d in self.alive():
            was = d.link
            d.link = d.id in seen
            if d.link:
                d.last_link_pos = d.pos.copy(); d.offline_since = None
                for m in d.outbox:
                    self.receive(d, m)
                d.outbox.clear()
                if not was:
                    self.say(f"{d.name} back in radio contact: map merged")
            elif was:
                d.offline_since = self.t
                self.say(f"{d.name} lost radio contact inside the rubble: continuing on its own")

    def send(self, d, msg):
        (self.receive(d, msg) if d.link else d.outbox.append(msg))

    def receive(self, d, msg):
        kind = msg[0]
        if kind == "bearing":
            _, pos, bearing, sigma, key = msg
            self.bear_groups.setdefault(key, dict(t=self.t, rays=[]))["rays"].append((pos, bearing, sigma))
        elif kind == "candidate":
            pass
        elif kind == "found":
            c = msg[1]
            self.say(f"COMMAND: {d.name} reports survivor at ({c['xy'][0]:.1f}, {c['xy'][1]:.1f}) m, "
                     f"void height {c['void']:.2f} m, {c['route']:.0f} m from entrance. Rescue team tasked")
        elif kind == "ask":
            self.reviews += 1
            self.ask_queue.append((self.t + 8.0, d.id, msg[1]))
            self.say(f"COMMAND asked to check {d.name}'s camera snapshot (p={msg[1]['p']:.2f})")

    # ------------------------------------------------------------------ acoustic map
    def add_bearing(self, pos, bearing, sigma, w=0.25):
        """A lone bearing: weak evidence spread along the ray."""
        dx, dy = self.XC - pos[0], self.YC - pos[1]
        ang = np.abs((np.arctan2(dy, dx) - bearing + np.pi) % (2 * np.pi) - np.pi)
        d = np.hypot(dx, dy)
        self.acoustic += w * np.exp(-0.5 * (ang / sigma) ** 2) * (d < 14) * (d > 0.3)

    def triangulate(self):
        """Match the same tap heard by several drones (time-stamped audio) and intersect the bearings
        by weighted least squares. Result: a position estimate with an error ellipse."""
        for key in [k for k, g in self.bear_groups.items() if self.t - g["t"] >= 1.0]:
            rays = self.bear_groups.pop(key)["rays"]
            P = np.array([r[0] for r in rays]); ang = np.array([r[1] for r in rays]); sg = np.array([r[2] for r in rays])
            spread = np.ptp(P, axis=0).max() if len(P) > 1 else 0.0
            if len(rays) < 2 or spread < 1.5:
                for p, a, sgm in rays:
                    self.add_bearing(p, a, sgm)
                continue
            U = np.stack([np.cos(ang), np.sin(ang)], 1); Nn = np.stack([-U[:, 1], U[:, 0]], 1)
            rng_est = np.full(len(rays), 5.0)
            for _ in range(2):
                w = 1.0 / (sg * rng_est) ** 2
                A = (w[:, None, None] * Nn[:, :, None] * Nn[:, None, :]).sum(0)
                bvec = (w[:, None, None] * Nn[:, :, None] * Nn[:, None, :] @ P[:, :, None]).sum(0)[:, 0]
                if abs(np.linalg.det(A)) < 1e-9:
                    break
                x = np.linalg.solve(A, bvec)
                rng_est = np.maximum(np.linalg.norm(x - P, axis=1), 0.5)
            else:
                ahead = ((x - P) * U).sum(1) > 0
                cov = np.linalg.inv(A)
                bx0, bx1 = (self.b.x0 + 1.5) * self.cfg.cell, (self.b.x1 - 1.5) * self.cfg.cell
                by0, by1 = (self.b.y0 + 1.5) * self.cfg.cell, (self.b.y1 - 1.5) * self.cfg.cell
                xc = np.array([np.clip(x[0], bx0, bx1), np.clip(x[1], by0, by1)])
                if np.linalg.norm(xc - x) < 2.5:
                    x = xc
                if ahead.all() and self.b.inside(x) and np.trace(cov) < 25:
                    sig = np.sqrt(max(np.linalg.eigvalsh(cov).max(), 0.25))
                    self.acoustic += 2.5 * np.exp(-0.5 * ((self.XC - x[0]) ** 2 + (self.YC - x[1]) ** 2) / sig ** 2)
                    self.fixes.append(dict(t=self.t, xy=x, cov=cov, rays=rays))
                    self.fixes = self.fixes[-12:]
                    rid = self.room_of(x)
                    if rid > 0 and self.room_status.get(rid) == "clear":
                        self.room_status[rid] = "re-opened"
                        self.room_reopen_t[rid] = self.t
                        self.caption(f"Knock from cleared room {rid}: re-opening", key=f"reopen{rid}")
                    if self.room_status.get(rid) == "re-opened":
                        self.priority_sounds.append((self.t, x.copy()))
                    continue
            for p, a, sgm in rays:
                self.add_bearing(p, a, sgm)

    # ------------------------------------------------------------------ coordinator
    def coordinate(self):
        if self.b.fires:
            nf = np.zeros_like(self.seen)
            for fr in self.b.fires:
                nf |= np.hypot(self.XC - fr["xy"][0], self.YC - fr["xy"][1]) < 3.0
            Map.nofly = nf
        c = self.coord()
        if not c.alive or not c.link:
            return
        linked = [d for d in self.alive() if d.link]
        for d in linked:
            self.cmap.merge_from(d.map)
        for d in linked:
            d.map.copy_from(self.cmap)
        self.triangulate()
        self.acoustic *= 0.985
        for f in self.found:
            self.acoustic[np.hypot(self.XC - f["xy"][0], self.YC - f["xy"][1]) < 2.0] = 0
        # sound hypotheses: peaks of the acoustic map
        self.sound_tasks = []
        self.priority_sounds = [(t0, xy) for t0, xy in self.priority_sounds if self.t - t0 < 40]
        mtrav = self.cmap.trav(self.cfg.micro)

        def snap(xy):          # fly to the nearest spot a micro-drone can actually reach
            dd = np.where(mtrav, np.hypot(self.XC - xy[0], self.YC - xy[1]), np.inf)
            k = np.unravel_index(np.argmin(dd), dd.shape)
            return (int(k[0]), int(k[1])) if dd[k] < 3.0 else self.cell(xy)
        if self.priority_sounds:
            xy = self.priority_sounds[-1][1]
            rc = snap(xy)
            self.sound_tasks.append(dict(type="sound", rc=rc, xy=self.b.center(rc), score=99.0))
        A = self.acoustic.copy()
        for _ in range(3):
            r, cc = np.unravel_index(np.argmax(A), A.shape)
            if A[r, cc] < 2.0:
                break
            rc = snap(self.b.center((r, cc)))
            self.sound_tasks.append(dict(type="sound", rc=rc, xy=self.b.center(rc), score=A[r, cc]))
            A[np.hypot(self.XC - self.XC[r, cc], self.YC - self.YC[r, cc]) < 2.5] = 0
        # quiet periods
        if self.t >= self.next_quiet:
            self.quiet_until = self.t + self.cfg.quiet_len
            self.next_quiet = self.t + self.cfg.quiet_every
            self.say("Quiet period: all drones perch and listen for tapping (6 s)")
        self.update_rooms()
        self.assign_posts()
        lis = [d for d in self.alive("micro") if d.role == "listener"]
        perched = [d for d in lis if d.state == PERCH]
        if lis and (len(perched) >= min(2, len(lis)) or self.t > 60) and self.callout_t is None:
            self.caption(f"Mapping west and east wings · {len(perched)} of {len(lis)} listeners in position", key="listeners")
            self.callout_t = self.t
            self.b.callout_t = self.t
            self.say("M loudspeaker call-out: \"Rescue team! If you can hear us, knock three times\"")
        if self.callout_t is not None and self.t > self.callout_t + 20:
            # after the call-out, only the sentinels keep listening; the rest search voids (they still hear when close)
            for d in sorted(lis, key=lambda d: d.id)[self.cfg.n_sentinels:]:
                if not d.released:
                    d.released, d.post = True, None
                    if d.state in (PERCH, TOPOST):
                        d.state, d.task, d.D_key = IDLE, None, None
                    self.say(f"{d.name} lifts off its listening post to search voids "
                             f"({self.cfg.n_sentinels} listeners stay landed)")
        self.allocate(linked)
        self.place_mother()

    def update_rooms(self):
        b = self.b
        known = self.cmap.known()
        c = np.nan_to_num(self.cmap.clear, nan=1.0)
        free = (~known) | (c >= 0.25)
        for rid in self.room_status:
            m = (b.room_id == rid) & free
            if not m.any():
                continue
            cov = float(self.seen[m].mean())
            b.rooms[rid]["cov"] = cov
            st = self.room_status[rid]
            if st == "victim":
                continue
            busy = any(cd["status"] in ("open", "review") and self.room_of(cd["xy"]) == rid for cd in self.candidates) or \
                any(self.room_of(tk["xy"]) == rid for tk in self.sound_tasks)
            if st == "unsearched" and cov > 0.05:
                st = "searching"
            if st == "searching" and cov >= 0.85 and not busy:
                st = "clear"
                self.say(f"Room {rid} searched (cameras + voids): clear")
            if st == "re-opened" and not busy and self.t - self.room_reopen_t.get(rid, 0) > 40:
                st = "clear"
            self.room_status[rid] = st

    def assign_posts(self):
        """Listeners land on LiDAR-verified flat spots spread over the building (one per quadrant)."""
        b, cfg = self.b, self.cfg
        known = self.cmap.known() & ~self.cmap.stale
        c = np.nan_to_num(self.cmap.clear, nan=0)
        flat = known & (c >= 0.9) & (c < OUT) & (b.room_id >= 0)
        if not flat.any():
            return
        X0, X1, Y0, Y1 = b.x0 * cfg.cell, b.x1 * cfg.cell, b.y0 * cfg.cell, b.y1 * cfg.cell
        quads = [(0.22, 0.3), (0.22, 0.75), (0.75, 0.3), (0.75, 0.75), (0.5, 0.5), (0.5, 0.2)]
        lis = [d for d in self.drones if d.kind == "micro" and d.role == "listener"]
        taken = [d.post for d in lis if d.post is not None and d.alive]
        for i, d in enumerate(lis):
            if not d.alive or d.post is not None or d.released:
                continue
            qx, qy = quads[i % len(quads)]
            tx, ty = X0 + qx * (X1 - X0), Y0 + qy * (Y1 - Y0)
            dd = np.hypot(self.XC - tx, self.YC - ty)
            for p in taken:
                dd = np.where(np.hypot(self.XC - self.XC[p], self.YC - self.YC[p]) < 6.0, np.inf, dd)
            dd = np.where(flat & (b.smoke < 0.2), dd, np.inf)
            k = np.unravel_index(np.argmin(dd), dd.shape)
            if not np.isfinite(dd[k]) or (dd[k] > 6.0 and self.t < 40):
                continue                      # that part of the building isn't mapped yet
            d.post = (int(k[0]), int(k[1]))
            taken.append(d.post)
            self.say(f"{d.name} assigned listening post in room {self.b.room_id[d.post]} (flat surface from LiDAR)")
        for d in lis:
            if d.alive and d.post is not None and d.state in (IDLE, VOID, RELAY):
                d.task, d.state, d.D_key = dict(type="post", rc=d.post), TOPOST, None

    def frontier_clusters(self, spec, exclude_scout_trav=False):
        m = self.cmap
        tr = m.trav(spec)
        unknown = ~m.known() | m.stale
        # wide drones' traversable map is eroded, so look 2 cells out for the unknown boundary
        fr = tr & binary_dilation(unknown, structure=S8, iterations=2 if spec.size >= 0.5 else 1) \
            & ~(np.nan_to_num(m.clear) >= OUT)
        if exclude_scout_trav:
            fr &= ~m.trav(self.cfg.scout)
        if spec.kind == "scout":
            fr &= self.ban < self.t
        lab, n = label(fr, structure=S8)
        out = []
        for k in range(1, n + 1):
            cells = np.argwhere(lab == k)
            mid = cells[len(cells) // 2]
            out.append(dict(rc=tuple(mid), size=len(cells)))
        return out

    def allocate(self, linked):
        cfg = self.cfg
        for c in self.candidates:          # a claim only holds while that drone is still working on it
            if c.get("by") is not None:
                o = self.drones[c["by"]]
                if not (o.alive and o.task is not None and o.task.get("cand") is c):
                    c["by"] = None
        # ---- scouts: frontier exploration
        scouts = [d for d in linked if d.kind == "scout" and d.state in (IDLE, EXPLORE, RELAY) and d.id != self.coord_id
                  and not (d.task and d.task.get("locked"))]
        fronts = self.frontier_clusters(cfg.scout)
        near_smoke = maximum_filter(self.b.smoke, size=9) > 0.15     # within ~1.6 m of thick smoke
        for fr in fronts:
            fr["smoky"] = bool(near_smoke[fr["rc"]])
        self.auction(scouts, fronts, "frontier", EXPLORE)
        radar = [d for d in scouts if d.has_radar and d.task is None]
        if radar and self.b.fires:
            sweeps = []
            for rid, st in self.room_status.items():
                if st in ("unsearched", "searching"):
                    m = (self.b.room_id == rid) & (self.b.smoke > 0.25) & ~self.seen & self.cmap.trav(cfg.scout)
                    if m.sum() > 6:
                        pts = np.argwhere(m)
                        sweeps.append(dict(type="sweep", rc=tuple(pts[len(pts) // 2]), room=rid))
            if sweeps:
                self.auction(radar, sweeps, "sweep", EXPLORE)
                for d in radar:
                    if d.task is not None:
                        fw = self.b.wing_of(self.b.fires[0]["xy"])
                        if "smoke" in self.flags:
                            self.caption(f"{fw.capitalize()} wing mapping handed to radar drone {d.name}", key="radar_handover")
        for s in scouts:
            if s.task is None:
                s.state = RELAY        # nothing to map: act as a radio relay near the micro-drones
        # ---- micro: confirm > sound > void
        micros = [d for d in linked if d.kind == "micro" and d.state in (IDLE, VOID, INVESTIGATE, RELAY)
                  and not (d.task and d.task.get("type") == "sound" and d.state == INVESTIGATE
                           and np.linalg.norm(d.pos - self.b.center(d.task["rc"])) < 3.0)
                  and not (d.task and d.task.get("type") in ("confirm", "post"))
                  and not (d.role == "listener" and d.post is not None)]
        tasks = []
        for c in self.candidates:
            if c["status"] == "open" and c.get("by") is None:
                tasks.append(dict(type="confirm", rc=self.cell(c["xy"]), cand=c, w=0.2, standoff=2))
        for s in self.sound_tasks:
            hot = self.room_status.get(self.room_of(s["xy"])) == "re-opened"
            tasks.append(dict(type="sound", rc=s["rc"], w=0.15 if hot else 0.35, xy=s["xy"]))
        for f in self.frontier_clusters(cfg.micro, exclude_scout_trav=True):
            tasks.append(dict(type="void", rc=f["rc"], w=1.0))
        # if a micro has been out of contact for long, send a relay toward its last link point
        for d in self.alive("micro"):
            if not d.link and d.offline_since is not None and self.t - d.offline_since > 15 and d.last_link_pos is not None:
                tasks.append(dict(type="relay", rc=self.cell(d.last_link_pos), w=0.5, for_id=d.id))
        self.auction(micros, tasks, None, None)

    def auction(self, agents, tasks, ttype, state):
        if not agents:
            return
        if not tasks:
            for a in agents:
                a.task = None
            return
        costs = []
        fields = {}
        for a in agents:
            D = dist_field(a.map.trav(a.spec), self.cell(a.pos))
            fields[a.id] = D
            for j, tk in enumerate(tasks):
                d = D[tk["rc"]]
                if np.isfinite(d):
                    w = tk.get("w", 1.0)
                    if tk.get("smoky"):
                        if not a.has_radar:
                            continue                  # LiDAR/camera useless in smoke
                        w *= 0.3                      # the radar drone takes the smoky areas
                    if tk.get("type") in ("confirm", "sound") and a.role == "listener":
                        w *= 3.0                      # confirmers do the close checks
                    sticky = 0.7 if (a.task and a.task.get("rc") == tk["rc"]) else 1.0
                    costs.append((d * w * sticky, a.id, j))
        costs.sort()
        used_a, used_t = set(), set()
        assign = {}
        for cst, aid, j in costs:
            if aid in used_a or j in used_t:
                continue
            used_a.add(aid); used_t.add(j); assign[aid] = j
        for a in agents:
            if a.id in assign:
                tk = dict(tasks[assign[a.id]])
                tk.setdefault("type", ttype)
                if (a.task is not None and a.task.get("type") == tk["type"] == "sound"
                        and np.hypot(a.task["rc"][0] - tk["rc"][0], a.task["rc"][1] - tk["rc"][1]) <= 6):
                    continue                              # same sound source, keep flying to it
                old = a.task
                new = (old is None or old.get("type") != tk["type"]
                       or np.hypot(old["rc"][0] - tk["rc"][0], old["rc"][1] - tk["rc"][1]) > 6)
                a.task = tk
                st = state or {"confirm": INVESTIGATE, "sound": INVESTIGATE, "void": VOID, "relay": RELAY}[tk["type"]]
                if tk.get("smoky") and a.has_radar:
                    fw = self.b.wing_of(self.b.fires[0]["xy"]) if self.b.fires else "east"
                    if "smoke" in self.flags:
                        self.caption(f"{fw.capitalize()} wing mapping handed to radar drone {a.name}", key="radar_handover")
                if new and tk["type"] in ("sound", "confirm", "relay"):
                    what = {"sound": "tapping heard: triangulated source", "confirm": "camera sighting to confirm",
                            "relay": "relay for a drone out of contact"}[tk["type"]]
                    self.say(f"{a.name} assigned: {what} at ({self.b.center(tk['rc'])[0]:.1f}, {self.b.center(tk['rc'])[1]:.1f})")
                    if tk["type"] == "confirm":
                        tk["cand"]["by"] = a.id
                a.state = st
            elif a.state not in (RELAY,):
                a.task = None
                a.state = IDLE

    def place_mother(self):
        m = self.drones[0]
        if not m.alive:
            return
        others = [d for d in self.alive() if d.id != 0]
        if not others or int(self.t / self.cfg.dt) % 20 or not self.survey_done:
            return
        if any(d.state in (RETURN, DOCKED) for d in others):
            return                          # hold the docking position while drones are inbound
        best, bscore = None, -1e9
        for st in self.stations[::2]:
            score = 0.0
            for d in others:
                pl = self.b.path_loss(st, d.pos)
                score += min(d.spec.radio_db - pl, 10)
            score -= 0.3 * np.linalg.norm(st - m.pos)
            if score > bscore:
                best, bscore = st, score
        if m.task is None or np.linalg.norm(m.task["xy"] - best) > 3:
            if m.task is not None:
                self.say(f"M repositions outside to ({best[0]:.0f}, {best[1]:.0f}) to keep the fleet in radio range")
            m.task = dict(type="station", xy=np.array(best, float))

    # ------------------------------------------------------------------ operator
    def operator(self):
        for item in list(self.ask_queue):
            due, did, cand = item
            if self.t < due:
                continue
            self.ask_queue.remove(item)
            truth = self.nearest_truth(cand["xy"])
            yes = (truth == "person") if self.rng.random() < 0.95 else (truth != "person")
            d = self.drones[did]
            if yes:
                self.confirm_found(d, cand, "MEDIUM", "confirmed by incident commander from snapshot", by_human=True)
            else:
                cand["status"] = "rejected"; self.decoys_rejected += 1
                self.say(f"COMMAND: {d.name}'s snapshot is not a person (mannequin/clothing). Carry on")
            if d.state == ASK:
                d.state, d.task = IDLE, None

    def nearest_truth(self, xy):
        best, kind = 1.0, "none"
        for s in self.b.survivors:
            if np.linalg.norm(s["pos"] - xy) < best:
                best, kind = np.linalg.norm(s["pos"] - xy), "person"
        for s in self.b.decoys:
            if np.linalg.norm(s["pos"] - xy) < best:
                best, kind = np.linalg.norm(s["pos"] - xy), "decoy"
        return kind

    def confirm_found(self, d, cand, conf="HIGH", evidence="", by_human=False):
        cand["status"] = "confirmed"
        open_s = [s for s in self.b.survivors if s["found_t"] is None and np.linalg.norm(s["pos"] - cand["xy"]) < 1.2]
        if open_s:
            min(open_s, key=lambda s: np.linalg.norm(s["pos"] - cand["xy"]))["found_t"] = self.t
        r, c = self.cell(cand["xy"])
        void = float(np.nan_to_num(self.cmap.clear[r, c], nan=0.5))
        path, breach = self.crew_route(cand["xy"])
        route = len(path) * self.cfg.cell if path else float("nan")
        label = "ABCDEFGH"[len(self.found)]
        info = dict(xy=cand["xy"], void=min(void, 2.8), route=route, p=cand["p"], t=self.t, conf=conf,
                    evidence=evidence, label=label, path=path, breach=breach, room=self.room_of(cand["xy"]))
        self.found.append(info)
        rid = info["room"]
        if rid > 0:
            self.room_status[rid] = "victim"
        self.acoustic[np.hypot(self.XC - cand["xy"][0], self.YC - cand["xy"][1]) < 2.0] = 0
        if not by_human:
            self.say(f"{d.name} CONFIRMED Victim {label} ({conf}: {evidence}). Hovering as beacon")
        self.send(d, ("found", info))

    def crew_route(self, xy):
        """Safe route for the rescue crew: known space they can crawl through (>= 0.6 m), out of heavy smoke."""
        c = np.nan_to_num(self.cmap.clear, nan=0)
        ok = self.cmap.known() & (c >= 0.6) & (self.b.smoke < 0.6)
        start = self.cell(self.b.entrance)
        D = dist_field(ok, start)
        d2 = np.hypot(self.XC - xy[0], self.YC - xy[1])
        k = np.unravel_index(np.argmin(np.where(np.isfinite(D), d2, np.inf)), d2.shape)
        if not np.isfinite(D[k]):
            return [], True
        breach = bool(d2[k] > 0.9)
        path, cur = [], k
        while D[cur] > 0 and len(path) < 400:
            path.append(self.b.center(cur))
            r, cc = cur
            nb = [(r + a, cc + e) for a in (-1, 0, 1) for e in (-1, 0, 1)
                  if (a or e) and 0 <= r + a < self.cfg.H and 0 <= cc + e < self.cfg.W]
            cur = min(nb, key=lambda x: D[x])
        path.append(self.b.center(cur))
        return path[::-1], breach

    def inspect_room(self, rid):
        """Coordinator sends the nearest LiDAR mapper to (re)map a room."""
        room = self.b.rooms[rid]
        ok = (self.b.room_id == rid) & self.cmap.trav(self.cfg.scout)
        if not ok.any():
            return
        pts = np.argwhere(ok)
        rc = tuple(int(v) for v in pts[np.argmin(np.hypot(self.XC[ok] - room["center"][0], self.YC[ok] - room["center"][1]))])
        mappers = [d for d in self.alive("scout") if not d.has_radar]
        if not mappers:
            return
        d = min(mappers, key=lambda m: np.linalg.norm(m.pos - room["center"]))
        d.task, d.state, d.D_key = dict(type="frontier", rc=rc, locked=True), EXPLORE, None
        self.say(f"Coordinator: {d.name} to map room {rid} in detail (only {100 * room.get('cov', 0):.0f}% searched)")

    def radar_handover(self, mapper, wing):
        """A LiDAR mapper is blinded by smoke: the coordinator hands its area to the radar scout."""
        radar = [r for r in self.alive("scout") if r.has_radar and r.state not in (RETURN, DOCKED)]
        if not radar or "radar_handover" in self.flags:
            return
        a = min(radar, key=lambda r: np.linalg.norm(r.pos - mapper.pos))
        trav = (self.cmap.trav(self.cfg.scout) | ~self.cmap.known()) & (self.b.room_id >= 0)
        tgt = trav & (self.b.smoke > 0.2) & ~self.seen
        for fr in self.b.fires:
            tgt &= np.hypot(self.XC - fr["xy"][0], self.YC - fr["xy"][1]) > 3.0      # no-fly zone around the fire
        if not tgt.any():
            return
        dd = np.hypot(self.XC - mapper.pos[0], self.YC - mapper.pos[1])
        k = np.unravel_index(np.argmin(np.where(tgt, dd, np.inf)), dd.shape)
        a.task, a.state, a.D_key = dict(type="sweep", rc=(int(k[0]), int(k[1])), locked=True, room=int(self.b.room_id[k])), EXPLORE, None
        self.caption(f"{wing.capitalize()} wing mapping handed to radar drone {a.name}", key="radar_handover")
        self.say(f"Coordinator: {mapper.name} blinded by smoke; {a.name} takes over with UWB radar")

    def handoff(self):
        """Mission summary for the rescue crew (template; scripts/briefing.py can rewrite it with Gemini)."""
        if self.handoff_t is not None:
            return
        self.handoff_t = self.t
        for f in self.found:
            f["path"], f["breach"] = self.crew_route(f["xy"])
            f["route"] = len(f["path"]) * self.cfg.cell if f["path"] else float("nan")
        unsearched = [rid for rid, st in self.room_status.items() if st in ("unsearched", "searching")]
        lines = [f"CREW BRIEFING  t = {self.t:.0f} s  ·  {len(self.found)} of {self.n_unaccounted} unaccounted located"]
        for f in self.found:
            lines.append(f"Victim {f['label']}: {f['conf']} confidence ({f['evidence']}). Room {f['room']}, void "
                         f"{f['void']:.2f} m high. Route {f['route']:.0f} m from west entrance"
                         + (", breach 1 obstacle at the end." if f["breach"] else "."))
        if unsearched:
            wings = sorted({self.b.rooms[r]["wing"] for r in unsearched})
            lines.append(f"Not searched: rooms {', '.join(map(str, unsearched))} ({' + '.join(wings)} wing)"
                         + (": smoke, radar map only." if self.b.fires else "."))
        missing = self.n_unaccounted - len(self.found)
        if missing > 0:
            lines.append(f"{missing} person(s) still unaccounted: most likely in the unsearched area.")
        if self.b.fires:
            lines.append("Hazards: active fire and smoke in the east wing; aftershock risk; re-check routes before entry.")
        self.briefing = lines
        self.caption("Handing map to crew")

    # ------------------------------------------------------------------ main tick
    def step(self):
        cfg, b = self.cfg, self.b
        self.t += cfg.dt
        tick = int(round(self.t / cfg.dt))
        if tick % 2 == 0:
            self.update_links()
        if tick % 4 == 0:
            self.coordinate()
        self.operator()
        sounds = b.sound_events(self.t)
        for d in self.alive():
            t0 = time.perf_counter()
            self.drone_tick(d, sounds)
            d.compute_ms = 0.8 * d.compute_ms + 0.2 * (time.perf_counter() - t0) * 1000

    def drone_tick(self, d: Drone, sounds):
        cfg, b = self.cfg, self.b
        t = self.t
        # ---------------- aftershock reflex (IMU) ----------------
        if b.imu_shock(t):
            if d.state != HOLD:
                d.prev_state = d.state
                d.state = HOLD
                if d.id == self.coord_id and not self.aftershock_seen:
                    self.say("AFTERSHOCK felt on every IMU: all drones hold position, map marked stale")
                    self.aftershock_seen = True
            return
        if d.state == HOLD:
            self.aftershock_seen = False
            r, c = self.cell(d.pos)
            if b.clear[r, c] < d.spec.min_clear * 0.8 and d.kind != "mother":
                self.kill(d.id, "trapped by shifting debris in the aftershock")
                return
            d.state, d.task, d.D_key = IDLE, None, None
            if d.id == self.coord_id:
                inside = (np.nan_to_num(self.cmap.clear) < OUT) & self.cmap.known()
                self.cmap.stale |= inside
                for o in self.alive():
                    o.map.stale |= inside
                self.say("Coordinator: re-scanning the building, every route re-checked before use")
        # ---------------- battery ----------------
        if d.state == TRAPPED:
            d.battery = max(0.0, d.battery - cfg.dt / d.spec.battery_s * 0.1)   # landed: only mic + radio on
            for src in sounds:
                h = b.hear(d.pos, src, True)
                if h is not None:
                    self.send(d, ("bearing", d.pos.copy(), h[0], h[1], (id(src), round(t, 1))))
            return
        drain = 0.0 if d.kind == "mother" else cfg.dt / d.spec.battery_s * (
            0.06 if d.state == PERCH else 0.3 if d.state in (LISTEN, DOCKED) else 1.0)
        if d.state == DOCKED:
            d.battery = min(1.0, d.battery + cfg.dt / cfg.swap_s)
            if d.battery >= 1.0:
                d.state = IDLE
                self.say(f"{d.name}: battery swapped on the mothership, redeploying")
            d.pos = self.drones[0].pos + np.array([-0.5 + 0.2 * (d.id % 6), 0.5])
            return
        d.battery = max(0.0, d.battery - drain)
        if d.battery <= 0:
            self.kill(d.id, "battery empty")
            return
        dock = self.drones[0].pos if self.drones[0].alive else self.base
        if d.kind != "mother" and (d.home_t is None or t - d.home_t > 8):
            d.home_D, d.home_t = self.route_field(d, self.cell(dock)), t
        hd = d.home_D[self.cell(d.pos)] if d.home_D is not None else np.inf
        route_m = hd * cfg.cell if np.isfinite(hd) else np.linalg.norm(d.pos - dock) * 1.5
        home = route_m * 1.5 / d.spec.speed / d.spec.battery_s + 0.06
        if d.state == PERCH:
            home *= 0.6                        # landed: drains slowly, can stay out longer
        if d.kind != "mother" and d.state not in (RETURN, DOCKED, BEACON) and d.battery < home:
            self.say(f"{d.name} battery {d.battery * 100:.0f}%: returning to {'mothership dock' if d.kind == 'micro' else 'base'}")
            if d.task and d.task.get("cand"):
                d.task["cand"]["by"] = None
            d.state, d.task, d.D_key = RETURN, None, None

        # ---------------- sense ----------------
        quiet = self.quiet()
        r0, c0 = self.cell(d.pos)
        dust = b.dust[r0, c0]
        smoke = b.smoke[r0, c0]
        d.still = d.still + 1 if (d.last_pos is not None and np.linalg.norm(d.pos - d.last_pos) < 0.05) else 0
        d.last_pos = d.pos.copy()
        q_lidar = float(np.clip(1 - 0.5 * dust - 0.9 * smoke, 0, 1))
        q_rgb = float(np.clip(1 - 0.6 * dust - 0.97 * smoke, 0, 1))
        d.health = {"LiDAR": q_lidar if d.kind != "micro" else None, "RGB": q_rgb,
                    "radar": 1.0 if d.has_radar else None, "mic": 1.0 if d.kind == "micro" else None}
        if d.kind == "scout" and smoke > 0.25:
            src = min(b.fires, key=lambda fr: np.linalg.norm(fr["xy"] - d.pos))["xy"] if b.fires else d.pos
            wing = b.wing_of(src)
            self.caption(f"Smoke in {wing} wing: LiDAR and RGB degraded", key="smoke")
            if not d.has_radar and d.state not in (PULLOUT, RETURN):
                self.say(f"{d.name}: smoke. LiDAR returns scattered, camera grey: pulling out")
                self.radar_handover(d, wing)
                safe = self.cmap.known() & (np.nan_to_num(self.cmap.clear, nan=0) >= 0.9) & (b.smoke < 0.08) & (b.room_id >= 0)
                D = self.route_field(d, (r0, c0))
                D = np.where(safe, D, np.inf)
                if np.isfinite(D).any():
                    k = np.unravel_index(np.argmin(D), D.shape)
                    d.task, d.state, d.D_key = dict(type="pullout", rc=(int(k[0]), int(k[1]))), PULLOUT, None
            elif d.has_radar and not d.radar_mode:
                d.radar_mode = True
                self.say(f"{d.name}: in smoke, mapping with UWB radar (coarse, sees through smoke and thin walls)")
        if d.kind == "scout" and d.radar_mode and smoke < 0.1:
            d.radar_mode = False
        if d.state == PULLOUT and smoke < 0.1 and (d.task is None or d.task.get("type") != "pullout"):
            d.state = IDLE
        if d.kind == "scout" and d.radar_mode:
            rr, cc, vv = b.radar_map(d.pos, cfg.radar_range_scout * 0.8, max_solid=2)
            fresh = ~d.map.known()[rr, cc] | (d.map.stale[rr, cc])
            d.map.write(rr[fresh], cc[fresh], vv[fresh], t)
            self.radar_cells[rr, cc] = True
        if d.has_radar and d.still >= 4 and (d.kind == "micro" or d.radar_mode):   # needs a still platform
            rng_r, ms = (cfg.radar_range_micro, 1) if d.kind == "micro" else (cfg.radar_range_scout, 3)
            d.radar_dets = b.radar_breathing(d.pos, rng_r, ms)
            rr, cc, _ = b.radar_map(d.pos, rng_r, max_solid=ms)
            self.seen[rr, cc] = True
            for xy, solid in d.radar_dets:
                near = [cd for cd in self.candidates if np.linalg.norm(cd["xy"] - xy) < 1.0 and cd["status"] != "rejected"]
                if near:
                    near[0]["breath"] = near[0].get("breath", 0) + 1
                    near[0]["solid"] = solid
                elif not any(np.linalg.norm(f["xy"] - xy) < 1.0 for f in self.found):
                    cnd = dict(xy=xy, scores=[], p=0.5, status="open", by=None, t=t, breath=1, solid=solid, src="radar")
                    self.candidates.append(cnd)
                    if d.kind == "micro" and d.role == "confirmer" and not (d.task and d.task.get("type") == "confirm"):
                        cnd["by"] = d.id
                        d.task, d.state, d.D_key = dict(type="confirm", rc=self.cell(xy), cand=cnd, standoff=2), INVESTIGATE, None
                    self.say(f"{d.name} radar: breathing signal {np.linalg.norm(xy - d.pos):.1f} m away"
                             + (f" behind {solid} obstacle(s)" if solid else ""))
        else:
            d.radar_dets = []
        if d.kind == "scout" and q_rgb > 0.5 and not d.radar_mode:
            rr, cc, vv = b.lidar(d.pos, 36, cfg.scout_cam_range)        # what the scout camera can see
            ok = b.clear[rr, cc] >= 0.9
            self.seen[rr[ok], cc[ok]] = True
            for xy, p in b.camera(d.pos, d.heading, cfg.scout_cam_range, 90):
                if b.clear[self.cell(xy)] < 0.9:
                    continue                          # person in a void: not visible from the open room
                if p > 0.55 and not any(np.linalg.norm(cd["xy"] - xy) < 1.0 for cd in self.candidates):
                    self.candidates.append(dict(xy=xy, scores=[], p=p, status="open", by=None, t=t, breath=0, src="rgb"))
                    self.say(f"{d.name} camera: possible person (score {p:.2f}): asking a confirmer to check")
        if d.kind == "scout":
            if dust > 0.45 and not d.ultra_mode:
                d.ultra_mode = True
                self.say(f"{d.name}: dust blinds LiDAR (density {dust:.2f}); switching to ultrasound navigation at half speed")
            elif dust < 0.3 and d.ultra_mode:
                d.ultra_mode = False
                self.say(f"{d.name}: dust settling, LiDAR mapping resumed")
            if not d.ultra_mode:
                rr, cc, vv = b.lidar(d.pos, cfg.lidar_rays, cfg.lidar_range)
                if not d.radar_mode:
                    d.map.write(rr, cc, vv, t)
                d.lidar_hits = b.last_lidar_ends
            else:
                us = b.ultrasound(d.pos, cfg.ultra_range)
                for k, dist in enumerate(us):
                    a = 2 * math.pi * k / len(us)
                    rr, cc = b.line_cells(d.pos, d.pos + dist * np.array([math.cos(a), math.sin(a)]))
                    d.map.write(rr[:-1], cc[:-1], np.maximum(b.clear[rr[:-1], cc[:-1]], 0.9), t)
                d.lidar_hits = None
        elif d.kind == "micro":
            rr, cc, vv = b.local_map(d.pos, cfg.micro_map_range, cfg.micro.min_clear)
            d.map.write(rr, cc, vv, t)
            if q_rgb > 0.3:
                self.seen[rr, cc] = True
            d.cam_dets = b.camera(d.pos, d.heading, cfg.cam_range, cfg.cam_fov_deg)
            for src in sounds:
                h = b.hear(d.pos, src, d.state == PERCH or (quiet and d.state == LISTEN))
                if h is not None:
                    bearing, sigma, level = h
                    self.send(d, ("bearing", d.pos.copy(), bearing, sigma, (id(src), round(t, 1))))
                    d.bear_vis.append((t, bearing, sigma)); d.bear_vis = d.bear_vis[-4:]
                    compass = (math.degrees(bearing) + 90) % 360
                    if d.state == PERCH or quiet:
                        self.say(f"{d.name} microphone: knock heard, bearing {compass:.0f}° ({level:.0f} dB)")
                    if d.state == PERCH:
                        self.caption(f"Knock heard, bearing {compass:.0f}° ({d.name})", key=f"knock{id(src)}")
            for xy, p in d.cam_dets:
                if d.state in (BEACON, ASK, RETURN, PERCH, TOPOST):
                    break
                known = [c for c in self.candidates if np.linalg.norm(c["xy"] - xy) < 1.0]
                if any(c["status"] in ("confirmed", "rejected", "review") for c in known):
                    continue
                busy = bool(d.task and d.task.get("type") == "confirm")
                if not known and p > 0.5:
                    cand = dict(xy=xy, scores=[], p=p, status="open", by=None, t=t, breath=0, src="rgb")
                    self.candidates.append(cand)
                    self.say(f"{d.name} camera: possible person (score {p:.2f})"
                             + ("" if busy else ". Moving in close to check for breathing"))
                    known = [cand]
                cnd = known[0] if known else None
                if d.role == "listener" and d.post is not None:
                    break                            # listeners report, confirmers go and check
                if cnd is not None and not busy and cnd["status"] == "open" and cnd.get("by") in (None, d.id):
                    cnd["by"] = d.id
                    d.task = dict(type="confirm", rc=self.cell(cnd["xy"]), cand=cnd, standoff=2)
                    d.state, d.D_key = INVESTIGATE, None
                    break
        # ---------------- quiet period ----------------
        if quiet and d.kind in ("micro", "scout") and d.state in (IDLE, EXPLORE, VOID, INVESTIGATE, RELAY):
            d.prev_state = d.state; d.state = LISTEN
        if not quiet and d.state == LISTEN:
            d.state = getattr(d, "prev_state", IDLE)
        if d.state == LISTEN:
            return

        # ---------------- confirm close up: fuse RGB + breathing radar ----------------
        if d.state == INVESTIGATE and d.task and d.task.get("type") == "confirm":
            cand = d.task["cand"]
            dist = np.linalg.norm(d.pos - cand["xy"])
            if dist < 2.4:      # yaw to point the camera + LED at the target
                v = cand["xy"] - d.pos
                d.heading = math.atan2(v[1], v[0])
                d.cam_dets = b.camera(d.pos, d.heading, cfg.cam_range, cfg.cam_fov_deg)
            near_enough = 2.6 if (cand.get("src") == "radar" and cand.get("breath", 0) > 0) else 2.0
            d.task["hover"] = dist < near_enough and (cand.get("breath", 0) > 0 or dist < 0.8 + 0.4 * d.task.get("standoff", 2))
            if dist < near_enough:
                if cand.get("close_ticks", 0) == 0:
                    self.say(f"{d.name} hovering at target: camera + breathing radar")
                for xy, p in d.cam_dets:
                    if np.linalg.norm(xy - cand["xy"]) < 0.8:
                        cand["scores"].append(p)
                cand.setdefault("close_ticks", 0)
                cand["close_ticks"] += 1
                sc = cand["scores"][-6:]
                br = cand.get("breath", 0)
                if sc:
                    cand["p"] = float(np.mean(sc))
                vlabel = "ABCDEFGH"[len(self.found)]
                if len(sc) >= 2 and cand["p"] > 0.85 and br >= 2:
                    self.confirm_found(d, cand, "HIGH", "RGB sees a person + breathing on radar")
                    self.caption(f"Hovering · Breathing confirmed · Victim {vlabel}, high confidence")
                    d.state, d.hold_until = BEACON, t + 10
                elif br >= 4 and len(sc) == 0 and cand["close_ticks"] >= 10:
                    self.confirm_found(d, cand, "MEDIUM", "breathing on radar, no camera line of sight")
                    self.caption(f"Breathing behind obstacle · Victim {vlabel}, radar only, medium confidence")
                    d.state, d.hold_until = BEACON, t + 10
                elif len(sc) >= 3 and cand["p"] < 0.5 and br == 0 and cand["close_ticks"] >= 8:
                    cand["status"] = "rejected"; self.decoys_rejected += 1
                    self.say(f"{d.name}: not a person (camera {cand['p']:.2f}, no breathing on radar). Marked as decoy")
                    d.state, d.task = IDLE, None
                elif len(sc) >= 6 and br == 0 and cand["close_ticks"] >= 14:
                    cand["status"] = "review"
                    d.state = ASK
                    self.send(d, ("ask", cand))
                    self.say(f"{d.name} UNSURE (camera {cand['p']:.2f}, no breathing yet): snapshot sent to incident commander")
                if cand["status"] == "open" and len(cand["scores"]) < 2 and br < 2 and cand["close_ticks"] in (8, 16):
                    d.task["standoff"] = max(0, d.task.get("standoff", 2) - 1)   # no clear view: get closer
                    d.D_key = None
                if cand["status"] == "open" and cand["close_ticks"] > 36:
                    cand["by"] = None
                    d.state, d.task = IDLE, None
        if d.state == BEACON and t > d.hold_until:
            d.state, d.task = IDLE, None

        # ---------------- autonomy when out of contact ----------------
        if not d.link and d.kind == "micro" and d.state in (IDLE, VOID) and d.offline_since is not None:
            if t - d.offline_since > 30 and d.last_link_pos is not None:
                d.task = dict(type="rejoin", rc=self.cell(d.last_link_pos)); d.state = RELAY
            elif d.task is None or d.task.get("type") != "void":
                fr = d.map.trav(d.spec) & binary_dilation(~d.map.known(), structure=S8)
                pts = np.argwhere(fr)
                if len(pts):
                    k = np.argmin(np.hypot(pts[:, 0] - r0, pts[:, 1] - c0))
                    d.task, d.state = dict(type="void", rc=tuple(pts[k])), VOID

        # ---------------- act ----------------
        if d.kind == "mother":
            rr, cc, vv = b.lidar(d.pos, cfg.lidar_rays, cfg.lidar_range)     # facade LiDAR, always on
            d.map.write(rr, cc, vv, t)
            d.lidar_hits = b.last_lidar_ends
            if not self.survey_done:
                tgt = self.survey[0]
                if np.linalg.norm(tgt - d.pos) < 0.3:
                    self.survey.pop(0)
                    if not self.survey:
                        self.survey_done = True
                        wall = (b.clear < OUT) & binary_dilation(b.clear >= OUT, structure=S8)
                        op = wall & (np.nan_to_num(d.map.clear, nan=0) >= cfg.micro.min_clear)
                        n_op = label(op, structure=S8)[1]
                        self.say(f"M facade survey complete: {n_op} openings into the building mapped")
                        tgt = d.pos
                    else:
                        tgt = self.survey[0]
                d.state = "SURVEY"
            else:
                tgt = d.task["xy"] if d.task else d.pos
                d.state = COORD if self.coord_id == 0 else IDLE
            v = tgt - d.pos; n = np.linalg.norm(v)
            if n > 0.05:
                d.pos = d.pos + v / n * min(n, d.spec.speed * cfg.dt)
                d.heading = math.atan2(v[1], v[0])
            return
        if d.state == RETURN:
            self.navigate(d, self.cell(dock))
            if np.linalg.norm(d.pos - dock) < 0.8:
                d.state = DOCKED if d.kind == "micro" else IDLE
                if d.kind == "scout":
                    d.battery = 1.0
            return
        if d.state == HOVER:
            d.heading += math.pi / 6                 # slow yaw: camera + LED sweep
            heard = [bv for bv in d.bear_vis if bv[0] == t]
            if heard and d.role == "confirmer" and getattr(d, "homing", 0) < 4 and not d.radar_dets:
                a = heard[-1][1]
                tgt = d.pos + 1.5 * np.array([math.cos(a), math.sin(a)])
                d.homing = getattr(d, "homing", 0) + 1
                d.task, d.state, d.D_key = dict(type="sound", rc=self.cell(tgt), homing=True), INVESTIGATE, None
                self.say(f"{d.name} hears the knock itself (bearing {(math.degrees(a) + 90) % 360:.0f}°): moving 1.5 m closer")
                return
            if t > d.hold_until:
                d.state, d.homing = IDLE, 0
            return
        if d.state == SWEEP:
            if t > d.hold_until:
                d.state, d.task = IDLE, None
            return
        if d.state in (BEACON, ASK, PERCH):
            return
        if d.scan_left > 0 and d.state not in (INVESTIGATE,) or (d.scan_left > 0 and d.task is None):
            d.scan_left -= 1
            d.heading += math.pi / 3          # turn on the spot, camera + LED sweep
            return
        if d.task is not None and d.task.get("hover"):
            pass                                     # holding still for the breathing radar
        elif d.task is not None:
            stand_off = d.task.get("standoff", {"confirm": 2, "sound": 1}.get(d.task.get("type"), 0))
            arrived = self.navigate(d, d.task["rc"], stop_at=stand_off)
            if arrived and d.task is not None and d.task.get("type") == "post":
                d.state, d.task = PERCH, None
                self.say(f"{d.name} landed on its post in room {self.room_of(d.pos)}: rotors off, listening")
            elif arrived and d.task is not None and d.task.get("type") == "pullout":
                d.state, d.task = IDLE, None
            elif arrived and d.task is not None and d.task.get("type") == "sweep":
                d.state, d.hold_until = SWEEP, t + 4.0
                d.radar_mode = True
                self.say(f"{d.name} landed in room {d.task['room']}: radar sweep for breathing through smoke and rubble")
            if arrived and d.task is not None and d.task.get("type") == "frontier":
                rr0, cc0 = d.task["rc"]
                self.ban[max(0, rr0 - 3):rr0 + 4, max(0, cc0 - 3):cc0 + 4] = self.t + 90
            if arrived and d.task is not None and d.task.get("type") in ("frontier", "void", "sound", "rejoin"):
                if d.kind == "micro" and d.task.get("type") == "sound":
                    d.state, d.hold_until = HOVER, t + 8.0
                    self.say(f"{d.name} at the sound source: hovering, listening + breathing radar")
                elif d.kind == "micro" and d.task.get("type") == "void":
                    d.scan_left = 3
                if d.task.get("type") == "sound":
                    self.acoustic[np.hypot(self.XC - d.pos[0], self.YC - d.pos[1]) < 1.6] *= 0.4
                d.task = None
                if d.state not in (RELAY, HOVER):
                    d.state = IDLE
        elif d.state == RELAY and d.kind == "scout":
            micros = [m for m in self.alive("micro") if m.state not in (DOCKED, RETURN)]
            if micros and (d.task is None) and int(t * 2) % 10 == 0:
                # hold a relay spot in open space between the coordinator and the micro-drones
                ctr = np.mean([m.pos for m in micros], axis=0) * 0.7 + self.drones[0].pos * 0.3
                ok = self.cmap.trav(cfg.scout) & (b.room_id >= 0) & (b.smoke < 0.2)
                dd = np.where(ok, np.hypot(self.XC - ctr[0], self.YC - ctr[1]), np.inf)
                d.relay_rc = np.unravel_index(np.argmin(dd), dd.shape)
            if getattr(d, "relay_rc", None) is not None:
                self.navigate(d, tuple(int(v) for v in d.relay_rc))
        d.trail.append(d.pos.copy()); d.trail = d.trail[-60:]

    def route_field(self, d: Drone, target):
        """Distance field to target over cells the drone knows it fits through. If no known route
        exists, plan optimistically through unexplored cells (explore on the way)."""
        trav = d.map.trav(d.spec)
        D = dist_field(trav, target)
        r, c = self.cell(d.pos)
        d.optimistic = False
        if not np.isfinite(D[r, c]):
            unk = ~d.map.known()
            D = dist_field(trav | unk, target)
            d.optimistic = bool(np.isfinite(D[r, c]))
        return D

    def navigate(self, d: Drone, target, stop_at=0):
        cfg = self.cfg
        key = (target, int(self.t // 2))
        if d.D_key != key:
            d.D = self.route_field(d, target)
            d.D_key = key
        r, c = self.cell(d.pos)
        D = d.D
        if D[r, c] <= stop_at:
            return True
        best, bd = None, D[r, c]
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                rr, cc = r + dr, c + dc
                if 0 <= rr < cfg.H and 0 <= cc < cfg.W and D[rr, cc] < bd:
                    best, bd = (rr, cc), D[rr, cc]
        if best is None:
            if not np.isfinite(D[r, c]):
                if d.state == RETURN:
                    d.state, d.task = TRAPPED, None
                    self.say(f"{d.name} TRAPPED: every way out is blocked. Landing to save power; "
                             f"it keeps listening and relaying from inside")
                else:
                    if d.task and d.task.get("cand"):
                        d.task["cand"]["by"] = None
                    d.task, d.state = None, IDLE      # unreachable now: give the task back
            return False
        # true-world check before moving (ultrasound/ToF): is the next cell actually passable?
        if self.b.clear[best] < d.spec.min_clear:
            d.map.write(np.array([best[0]]), np.array([best[1]]), np.array([self.b.clear[best]]), self.t)
            d.D_key = None
            if d.kind == "scout":
                d.note = "blocked: re-planning"
            return False
        wp = self.b.center(best)
        v = wp - d.pos; n = np.linalg.norm(v)
        speed = d.spec.speed * (0.5 if d.ultra_mode else 1.0)
        if n > 1e-6:
            step = v / n * min(n, speed * cfg.dt)
            d.pos = d.pos + step
            d.heading = math.atan2(v[1], v[0])
        d.note = "ultrasound nav" if d.ultra_mode else ""
        return False
