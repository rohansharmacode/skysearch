"""Earthquake scenario runner: building + fleet + events (storyboard, random, or live from the dashboard)."""
import numpy as np
from .config import USARConfig
from .building import Building
from .fleet import Fleet
from .grid import dist_field


class USARScenario:
    def __init__(self, cfg: USARConfig, random_events=True, fleet_cls=Fleet, t_handoff=None):
        self.cfg = cfg
        self.rng = np.random.default_rng(cfg.seed)
        self.b = Building(cfg, self.rng)
        self.fleet = fleet_cls(cfg, self.b, np.random.default_rng(cfg.seed + 1))
        er = np.random.default_rng(cfg.seed + 2)
        self.events = []
        self.t_handoff = t_handoff
        self.fire_pending = False
        if cfg.story:
            self.fire_pending = True     # fires once a LiDAR mapper is working in the east wing (or at t_fire + 40)
            self.t_handoff = t_handoff or cfg.t_knock_b + 60
        elif random_events:
            self.events.append((float(er.uniform(110, 170)), "aftershock", None))
        self.fleet.caption(f"Mission: primary search · {cfg.n_survivors} unaccounted")

    @property
    def swarm(self):
        return self.fleet

    def inject(self, kind, arg=None):
        f, b = self.fleet, self.b
        if kind == "story_fire":
            # post-quake fire in the east wing, a few metres ahead of the LiDAR mapper working there
            arg = b.story_fire
            ms = [d for d in f.alive("scout") if not d.has_radar]
            if ms:
                m = max(ms, key=lambda d: d.pos[0])
                ahead = (b.room_id >= 0) & (b.clear >= 0.9) & (b.clear < 90) & (f.XC > m.pos[0] + 1.0)
                for d in f.alive("micro"):
                    if d.state == "LISTENING (landed)":      # never start a fire on a landed listener
                        ahead &= np.hypot(f.XC - d.pos[0], f.YC - d.pos[1]) > 2.6
                dd = np.hypot(f.XC - m.pos[0], f.YC - m.pos[1])
                ok = ahead & (dd >= 3.4) & (dd <= 5.0)
                for r, c in np.argwhere(ok):         # open line of sight: the smoke reaches the mapper
                    rr, cc = b.line_cells(m.pos, b.center((r, c)))
                    if (b.clear[rr, cc] < 0.12).any():
                        ok[r, c] = False
                if ok.any():
                    k = np.unravel_index(np.argmax(np.where(ok, f.XC, -1)), dd.shape)
                    arg = b.center(k)
            kind = "fire"
        if kind == "aftershock":
            xy, rad, _ = b.aftershock(f.t, None if arg is None else np.asarray(arg, float))
            f.say(f"[event] AFTERSHOCK: section near ({xy[0]:.0f}, {xy[1]:.0f}) m collapses further")
        elif kind == "fire":
            xy = np.asarray(arg, float)
            b.start_fire(xy, f.t)
            rid = int(b.room_id[b.cell_of(xy)])
            f.say(f"[event] fire breaks out{f' in room {rid}' if rid > 0 else ''}: smoke spreading")
        elif kind == "fail":
            f.kill(int(arg), "motor failure")
        elif kind == "knock":
            silent = [s for s in b.survivors if not s["conscious"] and s["found_t"] is None]
            if arg is not None and silent:
                s = min(silent, key=lambda s: np.linalg.norm(s["pos"] - np.asarray(arg, float)))
            else:
                s = silent[0] if silent else None
            if s is not None:
                s["conscious"], s["next_sound"] = True, f.t + 0.5
                f.say("[event] a trapped person wakes up and starts knocking (swarm not told where)")
        elif kind == "move_person":
            s = next((s for s in b.survivors if s["found_t"] is None), None)
            if s is not None:
                s["pos"] = np.asarray(arg, float); s["conscious"] = True; s["next_sound"] = f.t + 1
                f.say("[event] a survivor crawls to a new spot (swarm not told)")
        elif kind == "inspect":
            f.inspect_room(int(arg))
        elif kind == "handoff":
            f.handoff()

    def step(self):
        t = self.fleet.t
        if self.fire_pending and t >= self.cfg.t_fire:
            col4 = self.b.rooms[4]["cols"][0] * self.cfg.cell
            near = [d for d in self.fleet.alive("scout") if not d.has_radar and d.pos[0] > col4 - 3.0
                    and self.b.room_id[self.b.cell_of(d.pos)] >= 0]
            if near or t >= self.cfg.t_fire + 40:
                self.fire_pending = False
                self.inject("story_fire")
        for ev in list(self.events):
            if t >= ev[0]:
                self.inject(ev[1], ev[2]); self.events.remove(ev)
        if self.t_handoff is not None and t >= self.t_handoff:
            self.fleet.handoff()
        self.b.step(t)
        self.fleet.step()

    def all_found(self):
        return all(s["found_t"] is not None for s in self.b.survivors)
