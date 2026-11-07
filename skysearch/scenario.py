"""Builds a run: world + swarm + disruption events. Events can be timed randomly (seeded)
so the swarm is never told when they will happen."""
import numpy as np
from .config import Config
from .world import World
from .swarm import Swarm


class Scenario:
    def __init__(self, cfg: Config, random_events=True, swarm_cls=Swarm):
        self.cfg = cfg
        self.rng = np.random.default_rng(cfg.seed)
        self.world = World(cfg, self.rng)
        self.swarm = swarm_cls(cfg, self.world, np.random.default_rng(cfg.seed + 1))
        er = np.random.default_rng(cfg.seed + 2)
        self.events = []
        if random_events:
            self.events.append((float(er.uniform(80, 120)), "fail", int(er.integers(0, cfg.n_drones))))
            fire_xy = self.world.lkp + er.uniform(-250, 250, 2)
            self.events.append((float(er.uniform(130, 170)), "fire", np.clip(fire_xy, 100, cfg.area - 100)))

    def inject(self, kind, arg):
        if kind == "fail":
            self.swarm.fail(arg, "motor fault")
        elif kind == "fire":
            self.world.ignite(arg)
        elif kind == "move_person":
            self.world.people[0]["pos"] = np.asarray(arg, float)

    def step(self):
        t = self.swarm.t
        for ev in list(self.events):
            if t >= ev[0]:
                self.inject(ev[1], ev[2]); self.events.remove(ev)
        self.world.step(t)
        self.swarm.step()

    def all_found(self):
        return all(p["found_t"] is not None for p in self.world.people)
