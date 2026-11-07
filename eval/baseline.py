"""Baseline: the standard practice of fixed parallel-strip ("lawnmower") sweeps.
Same drones, same sensors, same battery. No search map, no re-planning, and every
thermal blob is sent to the human operator to judge."""
import numpy as np
from skysearch.swarm import Swarm, SEARCH, RETURN, SWAP, FAILED


class LawnmowerSwarm(Swarm):
    def __init__(self, cfg, world, rng):
        super().__init__(cfg, world, rng)
        w = cfg.area / cfg.n_drones
        lane = 2 * world.footprint_radius(cfg.search_alt) * 0.9
        self.routes = {}
        for d in self.drones:
            xs = np.arange(d.id * w + lane / 2, (d.id + 1) * w, lane)
            pts = []
            for k, x in enumerate(xs):
                ys = (20, cfg.area - 20) if k % 2 == 0 else (cfg.area - 20, 20)
                pts += [np.array([x, ys[0]]), np.array([x, ys[1]])]
            self.routes[d.id] = pts
            d.wp = 0
        self.operator_load = 0
        self.operator_false = 0

    def drone_tick(self, d):
        cfg, w = self.cfg, self.world
        z = d.pos[2]
        blobs = w.thermal(d.pos) if z > 5 else []
        if z > 5:
            d.belief.observe_footprint(d.pos[:2], z, w.footprint_radius(z))
        for bx, by, _ in blobs:
            xy = np.array([bx, by])
            if any(np.linalg.norm(xy - r_) < 30 for r_, _ in self.rejected):
                continue
            self.operator_load += 1
            kind = self.nearest_truth(xy)
            if kind == "person":
                self.mark_found(xy)
            else:
                self.operator_false += 1
            self.rejected.append((xy, self.t))
        hazard = self.hazard
        if d.known_fire is not None:
            fr = 2 * w.footprint_radius(max(z, 1))
            n = cfg.n
            x, y = d.pos[:2]
            c0 = int(max((x - fr) // cfg.cell, 0)); c1 = int(min((x + fr) // cfg.cell, n - 1))
            r0 = int(max((y - fr) // cfg.cell, 0)); r1 = int(min((y + fr) // cfg.cell, n - 1))
            d.known_fire[r0:r1 + 1, c0:c1 + 1] |= w.fire[r0:r1 + 1, c0:c1 + 1]
        home_d = np.linalg.norm(d.pos[:2] - w.base)
        need = (home_d / cfg.speed + z / cfg.climb) / cfg.battery_s + cfg.reserve_frac
        if d.state == SEARCH and d.battery < need:
            d.state, d.path = RETURN, []
        if d.state == SEARCH:
            route = self.routes[d.id]
            if d.wp >= len(route):
                d.wp = 0
            tgt = route[d.wp]
            if np.linalg.norm(d.pos[:2] - tgt) < 10:
                d.wp += 1; d.path = []
            self.goto(d, tgt, cfg.search_alt, hazard)
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
                d.battery, d.state = 1.0, SEARCH
        d.battery = max(0.0, d.battery - (cfg.dt / cfg.battery_s if d.state != SWAP else 0))
        d.trail.append(d.pos[:2].copy()); d.trail = d.trail[-80:]
