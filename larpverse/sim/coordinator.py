"""Mission coordinator: kinematic drones + observation-driven scan loop.

Phases: deploy -> survey -> landing -> radar -> (relocate -> rescan)* -> acoustic -> done.

Isaac swap points (all small interfaces):
  * world   : ``World`` -> needs ``.grid`` (Grid heightmap) and ``.sample_pointcloud(n)``
  * flight  : ``FlightBackend`` protocol below (``KinematicFlight`` is the stand-in)
  * sensors : ``ObservationGenerator`` in radar_model.py (real drones emit the same dicts)
The coordinator itself never reads ground truth; it only sees observation dicts.

    python -m sim.coordinator --seed 3 --out dashboard/data [--frames] [--realtime]
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

from .landing import detect_landing_sites
from .localise import Localiser
from .planner import LANDED_SENSOR_HEIGHT, Planner, sensor_pose_at
from .radar_model import make_generator
from .sensor_model import SCAN_DWELL_S
from .world import World

DEFAULT_SEED = 75   # first 3 scans ambiguous: low-quality rough touchdown on the rubble -> planner relocates
LAND_HEIGHT = 0.15          # skid height above the patch plane
STARTS = {"d1": (-22.5, -22.5), "d2": (-20.5, -22.5), "d3": (-22.5, -20.5)}


# --------------------------------------------------------------------------- flight
class FlightBackend:
    """Protocol the coordinator needs from the flight stack (Isaac/PX4 or the kinematic stand-in)."""

    def command(self, cmd: dict): ...                       # contract commands (INTERFACES.md)
    def step(self, dt: float, t: float) -> list: ...        # advance; return events
    def poses(self) -> list: ...                            # [{'id','pose','state','battery'}]
    def eta(self, drone_id: str, dest_xyz) -> float | None: ...   # seconds, None if unreachable


class _Drone:
    def __init__(self, did, xy, z0):
        self.id = did
        self.pos = np.array([xy[0], xy[1], z0], float)
        self.yaw = 45.0
        self.state = "docked"
        self.battery = 1.0
        self.route = []
        self.job = None
        self.dwell_left = 0.0


class KinematicFlight(FlightBackend):
    """Straight-line waypoint flight at cruise altitude with a heightmap collision check."""

    def __init__(self, grid, starts=STARTS, cruise_alt=8.0, speed=4.0, climb=2.0, descend=1.2):
        self.grid = grid
        self.starts = dict(starts)
        self.cruise, self.speed, self.climb, self.descend = cruise_alt, speed, climb, descend
        self.drones = {d: _Drone(d, xy, grid.height_at(*xy) + LAND_HEIGHT) for d, xy in starts.items()}
        self.stagger = {d: 1.0 * i for i, d in enumerate(starts)}

    # --- planning
    def _terrain_max(self, a, b, margin=1.5):
        n = max(2, int(np.hypot(b[0] - a[0], b[1] - a[1]) / 0.5))
        xs = np.linspace(a[0], b[0], n)
        ys = np.linspace(a[1], b[1], n)
        return float(max(self.grid.height_at(xs, ys).max(), self.grid.height_at(b[0], b[1]) + margin))

    def plan(self, d, dest, land=True):
        p = d.pos
        alt = max(self.cruise + self.stagger[d.id], self._terrain_max(p, dest) + 3.0)
        if alt > 25.0:
            return None
        route = []
        if p[2] < alt - 0.5:
            route.append(np.array([p[0], p[1], alt]))
        route.append(np.array([dest[0], dest[1], alt]))
        route.append(np.array([dest[0], dest[1], dest[2]]))
        return route

    def _route_time(self, start, route):
        t, cur = 0.0, np.array(start, float)
        for w in route:
            dh = np.hypot(*(w[:2] - cur[:2]))
            dz = w[2] - cur[2]
            t += dh / self.speed if dh > 1e-6 else abs(dz) / (self.climb if dz > 0 else self.descend)
            cur = w
        return t

    def eta(self, drone_id, dest_xyz):
        d = self.drones[drone_id]
        route = self.plan(d, dest_xyz)
        if route is None:
            return None
        t = self._route_time(d.pos, route)
        if d.battery - (0.0016 * t + 0.0006 * SCAN_DWELL_S) < 0.15:
            return None
        return t

    # --- commands
    def command(self, cmd):
        d = self.drones[cmd["drone_id"]]
        typ = cmd["type"]
        d.job = None
        if typ == "takeoff":
            d.route = [np.array([d.pos[0], d.pos[1], self.cruise + self.stagger[d.id]])]
            d.state = "flying"
            d.job = {"type": typ}
        elif typ in ("goto", "goto_land_scan"):
            dest = np.array(cmd["destination"], float)
            if typ == "goto_land_scan":
                dest[2] += LAND_HEIGHT
            route = self.plan(d, dest)
            if route is None:
                d.state = "hovering"
                return False
            d.route = route
            d.state = "flying"
            d.job = {"type": typ, "site": cmd.get("landing_site_id"), "dwell": cmd.get("scan_duration", SCAN_DWELL_S)}
        elif typ == "return":
            home = np.array([*self.starts[d.id], self.grid.height_at(*self.starts[d.id]) + LAND_HEIGHT])
            d.route = self.plan(d, home) or []
            d.job = {"type": "return"}
        elif typ == "hold":
            d.route = []
            d.state = "hovering" if d.pos[2] > self.grid.height_at(d.pos[0], d.pos[1]) + 0.5 else d.state
        return True

    # --- dynamics
    def step(self, dt, t):
        events = []
        for d in self.drones.values():
            if d.state == "scanning":
                d.dwell_left -= dt
                d.battery -= 0.0006 * dt
                if d.dwell_left <= 0:
                    d.state = "landed"
                    ev = {"type": "scan_complete", "drone_id": d.id, "site": d.job["site"], "pose": self._pose(d)}
                    d.job = None
                    events.append(ev)
                continue
            if not d.route:
                d.battery -= (0.0005 if d.state in ("landed", "docked") else 0.0016) * dt
                continue
            budget = dt
            d.battery -= 0.0016 * dt
            while d.route and budget > 1e-9:
                w = d.route[0]
                delta = w - d.pos
                dh = float(np.hypot(delta[0], delta[1]))
                dist = float(np.linalg.norm(delta))
                vert_only = dh < 1e-6
                v = (self.climb if delta[2] > 0 else self.descend) if vert_only else self.speed
                if dist < 1e-6:
                    d.route.pop(0)
                    continue
                step_len = v * budget
                if step_len >= dist:
                    d.pos = w.copy()
                    budget -= dist / v
                    d.route.pop(0)
                else:
                    d.pos = d.pos + delta / dist * step_len
                    budget = 0.0
                if dh > 1e-6:
                    d.yaw = float(np.degrees(np.arctan2(delta[1], delta[0])))
                d.state = ("descending" if (vert_only and delta[2] < 0) else "flying")
            if d.pos[2] < self.grid.height_at(d.pos[0], d.pos[1]) - 0.01:
                events.append({"type": "collision", "drone_id": d.id})
            if not d.route:
                typ = d.job["type"] if d.job else None
                if typ == "goto_land_scan":
                    d.state = "scanning"
                    d.dwell_left = d.job["dwell"]
                elif typ == "return":
                    d.state = "docked"
                    d.job = None
                else:
                    d.state = "hovering"
                    d.job = None
                    events.append({"type": "arrived", "drone_id": d.id, "pose": self._pose(d)})
        return events

    def _pose(self, d):
        return [float(d.pos[0]), float(d.pos[1]), float(d.pos[2]), float(d.yaw)]

    def poses(self):
        return [{"id": d.id, "pose": [round(float(v), 2) for v in self._pose(d)], "state": d.state,
                 "battery": round(max(d.battery, 0.0), 3)} for d in self.drones.values()]

    def drone_state(self, did):
        d = self.drones[did]
        return d.state, d.battery, d.job


# ------------------------------------------------------------------------- mission
class Mission:
    def __init__(self, seed=DEFAULT_SEED, policy="adaptive", budget=6, out_dir=None, frames=False,
                 realtime=False, speed=1.0, dt=1.0, n_points=120000, verbose=True, max_t=900.0, radar="model",
                 world=None, flight=None, sensors=None, centre=None, survey_radius=16.0, scan_offset=0, t0=0.0,
                 log=None):
        """Single-pile mission. The keyword-only extras let a multi-pile coordinator (sim/grid_mission.py)
        or an Isaac adapter inject the backends: ``world`` (needs ``.grid``; its grid is the planning window),
        ``flight`` (FlightBackend, may be shared across piles), ``sensors`` (observation generator),
        ``centre`` (pile centre for the survey and landing search), ``scan_offset`` (continue scan ids),
        ``t0`` (mission clock) and ``log`` (shared log list). Defaults reproduce the original behaviour."""
        self.seed, self.policy, self.budget = seed, policy, budget
        self.out_dir, self.frames, self.realtime, self.speed, self.dt = out_dir, frames, realtime, speed, dt
        self.verbose, self.max_t = verbose, max_t
        self.centre, self.survey_radius = centre, survey_radius
        self.landing_kw = {}           # extra detect_landing_sites() kwargs (e.g. search_radius per pile)
        # --- backends (Isaac swap points)
        self.world = world if world is not None else World(seed)
        self.grid = self.world.grid
        self.sensors = sensors if sensors is not None else make_generator(self.world, seed, radar)
        self.flight = flight if flight is not None else KinematicFlight(self.grid)
        # --- mission brain (never touches truth)
        self.loc = Localiser(self.grid)
        self.planner = Planner(self.grid, self.loc.cfg, seed=seed)
        self.t = float(t0)
        self.k = 0
        self.stage = "takeoff"
        self.phase = "deploy"
        self.sites = []
        self.scans = []
        self.acoustic = []
        self.log = log if log is not None else []
        self.cands = []
        self.result = {"status": "insufficient_evidence", "modes": [], "n_significant": 0, "reason": "no data"}
        self.pending = {}              # drone_id -> next_scan dict for active scan jobs
        self.next_scan = None
        self.scans_cmd = 0
        self.scan_idx = scan_offset
        self.ac_idx = 0
        self.rescanning = False
        self.queue = {}
        self.listen_end = None
        self.relocations = []          # (scan_no, drone, site, reason) for reports
        self._belief_json = None
        self._hm_json = json.dumps([round(float(v), 3) for v in self.grid.z.ravel()], separators=(",", ":"))
        self.pc_count = 0
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
            if frames:
                os.makedirs(os.path.join(out_dir, "frames"), exist_ok=True)
            pts = self.world.sample_pointcloud(n_points)
            pts.astype("<f4").tofile(os.path.join(out_dir, "pointcloud.bin"))
            self.pc_count = int(len(pts))
        if world is None:
            self._say("Mission start: 3 drones at staging corner, taking off")
        for d in self.flight.drones:
            self.flight.command({"drone_id": d, "type": "takeoff"})

    # ---- logging
    def _say(self, msg):
        self.log.append({"t": round(self.t, 1), "msg": msg})
        if self.verbose:
            print(f"[t={self.t:6.1f}] {msg}", flush=True)

    # ---- helpers
    def travel(self, did, site):
        return self.flight.eta(did, [site["xy"][0], site["xy"][1], site["z"]])

    def _idle(self):
        out = []
        for p in self.flight.poses():
            st, bat, job = self.flight.drone_state(p["id"])
            if job is None and st in ("landed", "hovering") and p["id"] not in self.pending:
                out.append({"id": p["id"], "battery": bat})
        return out

    def _site(self, sid):
        return next(s for s in self.sites if s["id"] == sid)

    def _issue_scan(self, did, sid, reason, score):
        s = self._site(sid)
        s["status"] = "used"
        self.flight.command({"drone_id": did, "type": "goto_land_scan",
                             "destination": [s["xy"][0], s["xy"][1], s["z"]],
                             "landing_site_id": sid, "scan_duration": SCAN_DWELL_S})
        self.scans_cmd += 1
        self.pending[did] = {"drone_id": did, "site_id": sid, "reason": reason, "score": round(float(score), 2)}

    # ---- main step
    def step(self):
        self.t += self.dt
        self.k += 1
        for ev in self.flight.step(self.dt, self.t):
            self._on_event(ev)
        self._advance()
        self._update_phase()
        self._write_state()
        if self.realtime:
            time.sleep(self.dt / self.speed)

    def run(self):
        self._write_state()
        while self.stage != "done" and self.t < self.max_t:
            self.step()
        return self

    def _on_event(self, ev):
        if ev["type"] == "collision":
            self._say(f"WARNING collision flag for {ev['drone_id']}")
        elif ev["type"] == "scan_complete":
            self._on_scan(ev)
        elif ev["type"] == "arrived" and self.stage == "survey":
            q = self.queue.get(ev["drone_id"], [])
            if q:
                wp = q.pop(0)
                self.flight.command({"drone_id": ev["drone_id"], "type": "goto", "destination": wp})

    def _all_hover(self):
        return all(self.flight.drone_state(d)[0] == "hovering" and self.flight.drone_state(d)[2] is None
                   for d in self.flight.drones)

    def _advance(self):
        if self.stage == "takeoff" and self._all_hover():
            self._say("All drones airborne; starting LiDAR survey of the rubble")
            cx, cy = self.centre if self.centre is not None else (0.0, 0.0)
            R = self.survey_radius
            for i, did in enumerate(self.flight.drones):
                base = np.radians(-60 + 120 * i)
                self.queue[did] = [[cx + R * np.cos(base + np.radians(a)), cy + R * np.sin(base + np.radians(a)), 9.0 + i]
                                   for a in (0, 40, 80, 120)]
                wp = self.queue[did].pop(0)
                self.flight.command({"drone_id": did, "type": "goto", "destination": wp})
            self.stage = "survey"
        elif self.stage == "survey" and self._all_hover() and not any(self.queue.values()):
            self._before_landing()
            self.sites = detect_landing_sites(self.grid, centre=self.centre, **self.landing_kw)
            n_c = sum(s["status"] == "candidate" for s in self.sites)
            n_r = len(self.sites) - n_c
            n_slab = sum(1 for s in self.sites if s["status"] == "candidate" and s["z"] > 0.8)
            self._say(f"Survey complete: {n_c} landing patches found ({n_slab} on the rubble), {n_r} rejected "
                      f"(slope/step/approach)")
            self.stage = "decide"
        elif self.stage == "decide" and not self.pending:
            self._decide()
        elif self.stage == "listen" and self.t >= self.listen_end:
            self._finish_acoustic()

    def _before_landing(self):
        """Hook: survey finished, landing detection about to run (e.g. rebuild the grid from LiDAR)."""

    def _update_phase(self):
        if self.stage == "takeoff":
            self.phase = "deploy"
        elif self.stage == "survey":
            self.phase = "survey"
        elif self.stage == "listen":
            self.phase = "acoustic"
        elif self.stage == "done":
            self.phase = "done"
        elif self.pending:
            states = [self.flight.drone_state(d)[0] for d in self.pending]
            if any(s in ("flying", "descending", "hovering", "landed") for s in states):
                self.phase = "relocate" if self.rescanning else "landing"
            else:
                self.phase = "rescan" if self.rescanning else "radar"
        # else keep the previous phase while between decisions

    # ---- decision logic (observation-driven, never scripted)
    def _decide(self):
        idle = self._idle()
        if self.scans_cmd == 0:
            assign, why = self.planner.initial_sites(self.loc, self.sites, idle, self.travel)
            if not assign:
                self._say("No drone can reach any landing site; aborting")
                return self._start_listen()
            self._say(f"Initial sites chosen: {why}")
            for did, sid in assign:
                self._issue_scan(did, sid, "initial trilateration geometry", 0.0)
                self._say(f"{did} -> {sid} for a radar scan")
            self.next_scan = None
            return
        res = self.result
        reason = None if res["status"] == "candidate" else res["reason"]
        if reason is None:
            self._say(f"Localisation converged: {self._summ(res)}; stopping radar scans ({self.scans_cmd} used)")
            return self._start_listen()
        if self.scans_cmd >= self.budget:
            self._say(f"Scan budget ({self.budget}) exhausted; reporting insufficient_evidence ({reason})")
            return self._start_listen()
        self._say(f"Rescan needed: {reason}")
        if self.policy == "fixed":
            ch = self.planner.fixed_next(self.loc, self.sites, idle, self.travel, self.loc.centre)
        else:
            ch = self.planner.choose_next(self.loc, res, self.sites, idle, self.travel, self.scans_cmd)
        if ch is None:
            self._say("No reachable landing site left; stopping with insufficient_evidence")
            return self._start_listen()
        self.rescanning = True
        self._issue_scan(ch.drone_id, ch.site_id, ch.reason, ch.score)
        self.relocations.append({"scan_no": self.scans_cmd, "drone": ch.drone_id, "site": ch.site_id,
                                 "reason": ch.reason, "trigger": reason})
        self._say(f"Relocate {ch.drone_id} -> {ch.site_id}: {ch.reason}")

    def _rescan_reason(self, res):
        if res["status"] != "candidate":
            return res["reason"]
        dom = res["modes"][0]["xy"]
        support = sum(1 for o in self.loc.obs if o["range_candidates"]
                      and self.loc.agreement(o, dom) > 0.5 and max(o["confidence"]) * o["quality"] >= 0.35)
        if support < 3:
            return f"{support} of 3 strong ranges support R1 so far; need 3 for an independent fix"
        for o in self.scans:
            if o.get("doppler_moving") or not o["range_candidates"]:
                continue
            if o["quality"] >= 0.5 and max(o["confidence"]) * o["quality"] >= 0.4 and o.get("used_weight", 1) < 0.25:
                return f"disagreement: trusted scan {o['scan_id']} (q={o['quality']}) contradicts the consensus"
        return None

    def _summ(self, res):
        if not res["modes"]:
            return "no mode"
        m = res["modes"][0]
        return f"R1 at ({m['xy'][0]:.1f}, {m['xy'][1]:.1f}) holds {m['p_mass']:.0%}"

    def _on_scan(self, ev):
        did, sid = ev["drone_id"], ev["site"]
        self.pending.pop(did, None)
        site = self._site(sid)
        pose = sensor_pose_at(site)
        pose[3] = ev["pose"][3]
        self.scan_idx += 1
        obs_list = self.sensors.radar_scan(self.scan_idx, did, pose, site, self.t)
        for o in obs_list:
            kind = self.loc.update_radar(o)
            o["used_weight"] = 0.0
            self.scans.append(o)
            if kind == "moving":
                self._say(f"Scan {o['scan_id']}: doppler_moving return at {o['range_candidates'][0]} m -> moving track, "
                          f"not fused into the stationary belief")
            else:
                rc = ", ".join(f"{r:.1f} m (c={c:.2f})" for r, c in zip(o["range_candidates"], o["confidence"])) or "no detection"
                self._say(f"Scan {o['scan_id']} {did}@{sid}: quality {o['quality']:.2f}, ranges {rc}")
        stat = [o for o in obs_list if not o["doppler_moving"]][0]
        if stat["quality"] < 0.4:
            self._say(f"Scan {stat['scan_id']} quality {stat['quality']:.2f} is low (rough touchdown or platform motion at {sid}): "
                      f"its ranges are down-weighted")
        self._refresh()
        res = self.result
        if res["status"] == "insufficient_evidence" and res["modes"]:
            if res["n_significant"] >= 2:
                self._say(f"Belief ambiguous after {stat['scan_id']}: {res['reason']}")
            else:
                self._say(f"Belief not conclusive: {res['reason']}")
        elif res["status"] == "candidate":
            self._say(f"Belief: {self._summ(res)}")

    def _refresh(self):
        loc = self.loc
        self.result = loc.extract()
        uw = loc.used_weights(self.result)
        for o in self.scans:
            o["used_weight"] = round(float(uw.get(o["scan_id"], 0.0)), 2)
        res = self.result
        why = self._rescan_reason(res)      # mission-level acceptance rules on top of the filter's status
        if why:
            res["status"], res["reason"] = "insufficient_evidence", why
        cands = []
        for k, m in enumerate(res["modes"][:4]):
            ev = [o["scan_id"] for o in loc.obs if loc.agreement(o, m["xy"]) > 0.5]
            hz = [(o["breathing_hz"], o["quality"]) for o in loc.obs
                  if o["scan_id"] in ev and o["breathing_hz"] is not None]
            bhz = float(sum(h * q for h, q in hz) / sum(q for _, q in hz)) if hz else None
            c = m["cov"]
            cands.append({"id": f"R{k + 1}", "xy": [round(m["xy"][0], 2), round(m["xy"][1], 2)],
                          "z_est": round(float(self.grid.height_at(*m["xy"])) - loc.cfg.depth_nom, 2),
                          "cov": [[round(float(c[0, 0]), 3), round(float(c[0, 1]), 3)],
                                  [round(float(c[1, 0]), 3), round(float(c[1, 1]), 3)]],
                          "p_mass": round(m["p_mass"], 3), "breathing_hz": None if bhz is None else round(bhz, 3),
                          "kind": "stationary",
                          "status": "candidate" if (res["status"] == "candidate" and k == 0) else "insufficient_evidence",
                          "evidence": ev})
        tracks = loc.moving_tracks()
        for k, m in enumerate(tracks):
            c = m["cov"]
            cands.append({"id": f"M{k + 1}", "xy": [round(m["xy"][0], 2), round(m["xy"][1], 2)], "z_est": 0.9,
                          "cov": [[round(float(c[0, 0]), 3), round(float(c[0, 1]), 3)],
                                  [round(float(c[1, 0]), 3), round(float(c[1, 1]), 3)]],
                          "p_mass": round(m["p_mass"], 3), "breathing_hz": None, "kind": "moving",
                          "status": "candidate" if (len(tracks) == 1 and m["p_mass"] >= 0.5) else "insufficient_evidence",
                          "evidence": [o["scan_id"] for o in loc.moving_obs]})
        self.cands = cands
        self._belief_json = None

    # ---- acoustic stage
    def _start_listen(self):
        self.stage = "listen"
        self.listen_end = self.t + 12.0
        self.pending = {}
        self.next_scan = None
        self._say("Radar phase over; landed drones now listening with microphone arrays (12 s)")

    def _finish_acoustic(self):
        heard = 0
        for p in self.flight.poses():
            self.ac_idx += 1
            obs = self.sensors.acoustic_listen(self.ac_idx, p["id"], p["pose"][:2], self.t)
            if obs is None:
                self._say(f"{p['id']}: no sound heard")
                continue
            heard += 1
            self.acoustic.append(obs)
            self.loc.update_acoustic(obs)
            self._say(f"{p['id']}: {obs['class']} heard, bearing {obs['bearing_deg']:.0f} deg +/-{obs['width_deg'] / 2:.0f} "
                      f"(conf {obs['confidence']:.2f}) fused as a weak cone")
        before = self.result["status"]
        self._refresh()
        top = self.cands[0] if self.cands and self.cands[0]["kind"] == "stationary" else None
        if top:
            for a in self.acoustic:
                b = np.degrees(np.arctan2(top["xy"][1] - a["origin"][1], top["xy"][0] - a["origin"][0]))
                d = abs((b - a["bearing_deg"] + 180) % 360 - 180)
                self._say(f"Acoustic {a['obs_id']} {'agrees with' if d <= a['width_deg'] / 2 else 'does not cover'} "
                          f"{top['id']} (offset {d:.0f} deg)")
        self._say(f"Mission complete: status {self.result['status']}"
                  f"{' (was ' + before + ' before acoustics)' if before != self.result['status'] else ''}; {self._summ(self.result)}; "
                  f"{self.scans_cmd} radar scans, {len(self.relocations)} autonomous relocation(s)")
        self.stage = "done"
        self.phase = "done"

    # ---- state.json
    def _next_scan_out(self):
        if not self.pending:
            return None
        ns = list(self.pending.values())[-1]
        return dict(ns)

    def build_state(self):
        if self._belief_json is None:
            p = self.loc.belief().ravel()
            self._belief_json = "[" + ",".join(f"{v:.4g}" for v in p) + "]"
        sites = [{k: v for k, v in s.items() if k != "reason"} for s in self.sites]
        state = {
            "schema": "larpverse.state/v1", "t": round(self.t, 1), "phase": self.phase,
            "site": {"name": "Rubble site A", "bounds": self.grid.bounds},
            "heightmap": {"origin": list(self.grid.origin), "res": self.grid.res, "w": self.grid.w,
                          "h": self.grid.h, "z": "__HM__"},
            "pointcloud": {"url": "pointcloud.bin", "count": self.pc_count},
            "drones": self.flight.poses(),
            "landing_sites": sites,
            "scans": self.scans,
            "radar_candidates": self.cands,
            "acoustic": self.acoustic,
            "belief": {"origin": list(self.grid.origin), "res": self.grid.res, "w": self.grid.w,
                       "h": self.grid.h, "p": "__BEL__"},
            "next_scan": self._next_scan_out(),
            "log": self.log,
        }
        s = json.dumps(state, separators=(",", ":"))
        return s.replace('"__HM__"', self._hm_json).replace('"__BEL__"', self._belief_json)

    def _write_state(self):
        if not self.out_dir:
            return
        s = self.build_state()
        for path in [os.path.join(self.out_dir, "state.json")] + (
                [os.path.join(self.out_dir, "frames", f"state_{self.k:04d}.json")] if self.frames else []):
            tmp = path + ".tmp"
            with open(tmp, "w") as f:
                f.write(s)
            os.replace(tmp, path)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=DEFAULT_SEED)
    ap.add_argument("--out", default="dashboard/data")
    ap.add_argument("--frames", action="store_true", help="also write frames/state_XXXX.json each step")
    ap.add_argument("--realtime", action="store_true", help="sleep to match sim time")
    ap.add_argument("--speed", type=float, default=1.0, help="realtime speed multiplier")
    ap.add_argument("--budget", type=int, default=6, help="max radar scans")
    ap.add_argument("--policy", choices=["adaptive", "fixed"], default="adaptive")
    ap.add_argument("--radar", choices=["model", "dsp"], default="model",
                    help="model: direct noise model (default); dsp: synthetic IF signals through the real DSP chain")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args(argv)
    t0 = time.time()
    m = Mission(a.seed, a.policy, a.budget, a.out, a.frames, a.realtime, a.speed, verbose=not a.quiet, radar=a.radar)
    m.run()
    print(f"done in {time.time() - t0:.1f} s wall, {m.t:.0f} s sim; wrote {a.out}/state.json", file=sys.stderr)


if __name__ == "__main__":
    main()
