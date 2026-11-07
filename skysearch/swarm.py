"""On-board autonomy for each drone + the swarm plumbing (radio links, operator).

Every decision a drone makes is driven by its own sensor readings and its own copy
of the search map. The base station only relays messages to the human operator.
"""
from __future__ import annotations
import math
import time
from dataclasses import dataclass, field
import numpy as np
from scipy.ndimage import binary_dilation

from .config import Config
from .belief import Belief, lost_person_prior
from .planner import Partition, pick_target, segment_hits_hazard, astar

SEARCH, INVESTIGATE, CONFIRMED, REVIEW, RETURN, SWAP, FAILED = (
    "SEARCH", "INSPECT", "CONFIRMED", "ASK HUMAN", "RETURN", "SWAP", "FAILED")


@dataclass
class Drone:
    id: int
    pos: np.ndarray
    belief: Belief
    state: str = SEARCH
    battery: float = 1.0
    target: np.ndarray | None = None
    path: list = field(default_factory=list)
    cand: dict | None = None
    hold_t: float = 0.0
    link: bool = True
    trail: list = field(default_factory=list)
    compute_ms: float = 0.0
    note: str = ""
    known_fire: np.ndarray | None = None
    outbox: list = field(default_factory=list)


def logit(p):
    return math.log(p / (1 - p))


class Swarm:
    def __init__(self, cfg: Config, world, rng):
        self.cfg, self.world, self.rng = cfg, world, rng
        prior = lost_person_prior(cfg, world.lkp, world.canopy_prior)
        self.prior = prior
        self.drones = []
        for i in range(cfg.n_drones):
            p = np.array([world.base[0] + 12 * i, world.base[1], 0.0])
            b = Belief(cfg, prior, world.canopy_prior)
            self.drones.append(Drone(i, p, b, known_fire=np.zeros((cfg.n, cfg.n), bool)))
        ang = np.linspace(0, 2 * math.pi, cfg.n_drones, endpoint=False)
        anchors = {i: world.lkp + 250 * np.array([math.cos(a), math.sin(a)]) for i, a in enumerate(ang)}
        self.part = Partition(cfg, anchors)
        self.candidates = []      # every candidate the swarm has reported (for display / audit)
        self.rejected = []        # (xy, t) places recently ruled out
        self.log = []             # (t, text)
        self.operator_queue = []  # (due_t, drone_id, cand)
        self.alerts = 0
        self.false_alarms_filtered = 0
        self.reviews = 0
        self.t = 0.0
        self.hazard = np.zeros((cfg.n, cfg.n), bool)

    # ------------------------------------------------------------ utils
    def say(self, text):
        self.log.append((self.t, text))

    def alive(self):
        return [d for d in self.drones if d.state != FAILED]

    def fail(self, did, why="motor fault"):
        d = self.drones[did]
        if d.state == FAILED:
            return
        d.state, d.note = FAILED, why
        self.say(f"D{did} {why}: emergency landing. Re-splitting its area among {len(self.alive())} drones")

    # ------------------------------------------------------------ comms
    def update_links(self):
        cfg = self.cfg
        nodes = [("base", self.world.base)] + [(d.id, d.pos[:2]) for d in self.alive()]
        adj = {k: [] for k, _ in nodes}
        for i, (a, pa) in enumerate(nodes):
            for b, pb in nodes[i + 1:]:
                if np.linalg.norm(pa - pb) < cfg.comm_range:
                    adj[a].append(b); adj[b].append(a)
        seen, comps = set(), []
        for k, _ in nodes:
            if k in seen:
                continue
            stack, comp = [k], []
            while stack:
                u = stack.pop()
                if u in seen:
                    continue
                seen.add(u); comp.append(u); stack += adj[u]
            comps.append(comp)
        for comp in comps:
            ds = [self.drones[k] for k in comp if k != "base"]
            linked = "base" in comp
            for d in ds:
                if d.link and not linked and self.t - getattr(d, "_lt", -99) > 40:
                    self.say(f"D{d.id} out of radio range: continuing autonomously")
                    d._lt = self.t
                if not d.link and linked and self.t - getattr(d, "_lt2", -99) > 40:
                    self.say(f"D{d.id} back in range: merging its search map")
                    d._lt2 = self.t
                d.link = linked
            Belief.fuse([d.belief for d in ds])
            kf = np.logical_or.reduce([d.known_fire for d in ds]) if ds else None
            for d in ds:
                d.known_fire = kf.copy()
            if linked:
                for d in ds:
                    for m in d.outbox:
                        self.deliver(d, m)
                    d.outbox.clear()

    def deliver(self, d, msg):
        kind, cand = msg
        if kind == "confirmed":
            self.alerts += 1
            self.say(f"OPERATOR: D{d.id} reports person at ({cand['xy'][0]:.0f},{cand['xy'][1]:.0f}), "
                     f"p={cand['p']:.2f}. Ground team dispatched")
        elif kind == "review":
            self.reviews += 1
            self.operator_queue.append((self.t + 6.0, d.id, cand))
            self.say(f"OPERATOR asked to review D{d.id} snapshot (p={cand['p']:.2f})")

    def operator(self):
        for item in list(self.operator_queue):
            due, did, cand = item
            if self.t < due:
                continue
            self.operator_queue.remove(item)
            truth = self.nearest_truth(cand["xy"]) == "person"
            verdict = truth if self.rng.random() < 0.95 else not truth
            d = self.drones[did]
            if verdict:
                cand["status"] = "confirmed"
                self.mark_found(cand["xy"])
                self.say(f"OPERATOR confirms D{did} find: person. Ground team dispatched")
                self.alerts += 1
            else:
                cand["status"] = "rejected"
                self.rejected.append((cand["xy"].copy(), self.t))
                self.say(f"OPERATOR: D{did} snapshot is not a person. Resume search")
            if d.state == REVIEW:
                d.state, d.cand = SEARCH, None

    def nearest_truth(self, xy):
        w = self.world
        best, kind = 15.0, "rock"
        for p in w.people:
            dd = np.linalg.norm(p["pos"] - xy)
            if dd < best:
                best, kind = dd, "person"
        for dr in w.deer:
            dd = np.linalg.norm(dr["pos"] - xy)
            if dd < best:
                best, kind = dd, "deer"
        return kind

    def mark_found(self, xy):
        for p in self.world.people:
            if p["found_t"] is None and np.linalg.norm(p["pos"] - xy) < 25:
                p["found_t"] = self.t
        for d in self.alive():
            d.belief.clear_around(xy)

    # ------------------------------------------------------------ main tick
    def step(self):
        cfg, w = self.cfg, self.world
        self.t += cfg.dt
        if int(self.t / cfg.dt) % 4 == 0:
            self.update_links()
        self.operator()
        alive = self.alive()
        if int(self.t / cfg.dt) % 10 == 0 and alive:
            ref = next((d for d in alive if d.link), alive[0])
            self.hazard = binary_dilation(ref.known_fire, iterations=2)
            self.part.update([d.id for d in alive], ref.belief.posterior(), self.hazard)
        for d in alive:
            t0 = time.perf_counter()
            self.drone_tick(d)
            d.compute_ms = 0.8 * d.compute_ms + 0.2 * (time.perf_counter() - t0) * 1000
        for d in self.drones:
            if d.state == FAILED and d.pos[2] > 0:
                d.pos[2] = max(0.0, d.pos[2] - 3 * cfg.dt)
        if int(self.t / cfg.dt) % 10 == 0:
            for d in alive:
                d.belief.decay(0.998)
        self.rejected = [(xy, t) for xy, t in self.rejected if self.t - t < 90]

    def drone_tick(self, d: Drone):
        cfg, w = self.cfg, self.world
        x, y, z = d.pos
        airborne = z > 5
        # ---------------- sense ----------------
        blobs = w.thermal(d.pos) if airborne else []
        if airborne:
            r = w.footprint_radius(z)
            d.belief.observe_footprint(d.pos[:2], z, r)
            rc, can = w.lidar_canopy(d.pos)
            d.belief.canopy[rc] = 0.5 * d.belief.canopy[rc] + 0.5 * can
            # fire is obvious on thermal from further away
            fr = 2.0 * r
            n = cfg.n
            c0 = int(max((x - fr) // cfg.cell, 0)); c1 = int(min((x + fr) // cfg.cell, n - 1))
            r0 = int(max((y - fr) // cfg.cell, 0)); r1 = int(min((y + fr) // cfg.cell, n - 1))
            newfire = w.fire[r0:r1 + 1, c0:c1 + 1] & ~d.known_fire[r0:r1 + 1, c0:c1 + 1]
            if newfire.any():
                if not d.known_fire.any() and not any(o.known_fire.any() for o in self.alive()):
                    self.say(f"D{d.id} thermal: FIRE detected. No-fly zone set, all drones re-routing")
                d.known_fire[r0:r1 + 1, c0:c1 + 1] |= w.fire[r0:r1 + 1, c0:c1 + 1]
            bearing = w.microphone(d.pos)
            if bearing is not None:
                d.belief.bump(d.pos[:2], bearing)
                self.say(f"D{d.id} microphone: voice heard, bearing {math.degrees(bearing) % 360:.0f} deg. Raising map along it")
        hazard = binary_dilation(d.known_fire, iterations=2) if d.known_fire.any() else self.hazard

        # ---------------- decide ----------------
        home_d = np.linalg.norm(d.pos[:2] - w.base)
        need = (home_d / cfg.speed + z / cfg.climb) / cfg.battery_s + cfg.reserve_frac
        if d.state in (SEARCH, INVESTIGATE, CONFIRMED) and d.battery < need:
            self.say(f"D{d.id} battery {d.battery * 100:.0f}%: returning for swap")
            if d.cand and d.state == INVESTIGATE:
                self.say(f"D{d.id} hands its unconfirmed target to the swarm map")
                X, Y = self.part.X, self.part.Y
                near_c = np.hypot(X - d.cand["xy"][0], Y - d.cand["xy"][1]) < 40
                d.belief.logmiss[near_c] = np.maximum(d.belief.logmiss[near_c], 0) + 2.0
                d.cand["status"] = "handed off"
            d.state, d.cand, d.path = RETURN, None, []

        if d.state == SEARCH and blobs:
            for bx, by, size in blobs:
                xy = np.array([bx, by])
                if any(np.linalg.norm(xy - c["xy"]) < 30 and c["status"] in ("confirmed", "open")
                       for c in self.candidates):
                    continue
                if any(np.linalg.norm(xy - r_) < 30 for r_, _ in self.rejected):
                    continue
                cand = dict(xy=xy, lo=0.0, ticks=0, lost=0, status="open", p=0.5, by=d.id, t=self.t, size=size)
                self.candidates.append(cand)
                d.cand, d.state, d.path = cand, INVESTIGATE, []
                self.say(f"D{d.id} thermal blob ({size:.0f}px) at ({bx:.0f},{by:.0f}): breaking off to inspect at {cfg.inspect_alt:.0f} m")
                break

        if d.state == INVESTIGATE and d.cand is not None:
            c = d.cand
            near = [b for b in blobs if np.hypot(b[0] - c["xy"][0], b[1] - c["xy"][1]) < 18]
            if near:
                c["xy"] = 0.7 * c["xy"] + 0.3 * np.array(near[0][:2])
                c["lost"] = 0
            horiz = np.linalg.norm(d.pos[:2] - c["xy"])
            if horiz < 10 and z < cfg.inspect_alt + 4:
                s = w.rgb_person_score(d.pos, c["xy"])
                c["lo"] += 0.6 * logit(s)
                c["ticks"] += 1
                c["p"] = 1 / (1 + math.exp(-c["lo"]))
                if not near:
                    c["lost"] += 1
                if c["p"] > 0.92:
                    c["status"] = "confirmed"
                    self.mark_found(c["xy"])
                    d.state, d.hold_t = CONFIRMED, 8.0
                    d.outbox.append(("confirmed", c))
                    self.say(f"D{d.id} CONFIRMED person (p={c['p']:.2f}). Holding position as beacon")
                elif c["p"] < 0.10:
                    c["status"] = "rejected"
                    self.false_alarms_filtered += 1
                    self.rejected.append((c["xy"].copy(), self.t))
                    self.say(f"D{d.id} ruled out blob (p={c['p']:.2f}, likely animal/rock). Resuming search")
                    d.state, d.cand = SEARCH, None
                elif c["ticks"] > 16 or c["lost"] > 12:
                    c["status"] = "review"
                    d.state = REVIEW
                    d.outbox.append(("review", c))
                    self.say(f"D{d.id} UNSURE (p={c['p']:.2f}): hovering and asking the operator")

        if d.state == CONFIRMED:
            d.hold_t -= cfg.dt
            if d.hold_t <= 0:
                d.state, d.cand = SEARCH, None

        # ---------------- act ----------------
        if d.state == SEARCH:
            if d.target is None or np.linalg.norm(d.pos[:2] - d.target) < 15 or int(self.t / cfg.dt) % 6 == d.id % 6:
                taken = [o.target for o in self.alive() if o is not d and o.target is not None]
                d.target = pick_target(cfg, d.id, d.pos[:2], d.belief.posterior(), d.belief.canopy, hazard, self.part, taken)
                d.path = []
            self.goto(d, d.target, cfg.search_alt, hazard)
        elif d.state == INVESTIGATE:
            self.goto(d, d.cand["xy"], cfg.inspect_alt, hazard)
        elif d.state in (CONFIRMED, REVIEW):
            self.goto(d, d.cand["xy"] if d.cand else d.pos[:2], cfg.inspect_alt + 10, hazard)
        elif d.state == RETURN:
            if np.linalg.norm(d.pos[:2] - w.base) < 8:
                self.goto(d, w.base, 0.0, hazard)
                if d.pos[2] < 0.5:
                    d.state, d.hold_t = SWAP, cfg.swap_s
            else:
                self.goto(d, w.base, cfg.search_alt, hazard)
        elif d.state == SWAP:
            d.hold_t -= cfg.dt
            if d.hold_t <= 0:
                d.battery, d.state, d.target = 1.0, SEARCH, None
                self.say(f"D{d.id} battery swapped: relaunching")

        drain = cfg.dt / cfg.battery_s if d.state != SWAP else 0.0
        d.battery = max(0.0, d.battery - drain)
        if d.battery <= 0 and d.state != SWAP:
            self.fail(d.id, "battery depleted")
        d.trail.append(d.pos[:2].copy())
        d.trail = d.trail[-80:]

    def goto(self, d: Drone, xy, alt, hazard):
        cfg, w = self.cfg, self.world
        tgt = np.asarray(xy, float)
        if hazard.any() and not d.path and segment_hits_hazard(cfg, d.pos[:2], tgt, hazard):
            d.path = astar(cfg, d.pos[:2], tgt, hazard)
        way = d.path[0] if d.path else tgt
        if d.path and np.linalg.norm(d.pos[:2] - way) < 12:
            d.path.pop(0)
            way = d.path[0] if d.path else tgt
        v = way - d.pos[:2]
        dist = np.linalg.norm(v)
        step = min(cfg.speed * cfg.dt, dist)
        vel = v / dist * step if dist > 1e-6 else np.zeros(2)
        # separation from other drones at similar altitude
        for o in self.alive():
            if o is d:
                continue
            dv = d.pos[:2] - o.pos[:2]
            dd = np.linalg.norm(dv)
            if dd < cfg.sep and abs(o.pos[2] - d.pos[2]) < 10 and dd > 1e-6:
                vel += dv / dd * (cfg.sep - dd) * 0.15
        # LiDAR obstacle avoidance below tree tops
        if d.pos[2] < cfg.tree_height + 2 and d.pos[2] > 1:
            scan = w.lidar_scan(d.pos)
            angs = np.linspace(0, 2 * math.pi, len(scan), endpoint=False)
            close = scan < 6
            if close.any():
                rep = -np.array([np.cos(angs[close]), np.sin(angs[close])]).T * (6 - scan[close, None])
                vel += rep.sum(0) * 0.3
                d.note = "LiDAR avoid"
            else:
                d.note = ""
        d.pos[:2] += vel
        dz = alt - d.pos[2]
        d.pos[2] += np.clip(dz, -cfg.climb * cfg.dt, cfg.climb * cfg.dt)
        # takeoff only after reaching base area / climbing first
