"""Multi-pile grid mission: survey -> land -> radar -> ping -> listen -> rank -> dispatch, pile after pile.

The radar part at each pile is the existing single-pile ``Mission`` (sim/coordinator.py) with injected
backends: a shared ``KinematicFlight`` over the whole-site heightmap, a 30 m planning window around the
active pile (landing detection, localiser, planner) and a radar generator built for that pile. After the
radar phase the landed drones listen; ``sim.acoustic_grid.listen`` (the acoustic/ pipeline when available,
a synthetic stub otherwise) returns per-pile priorities, and the fleet is dispatched to the top-priority
unvisited pile until the pile budget is used or nothing is left above the priority threshold.

The coordinator never reads ground truth: it builds the generators from ``GridWorld.pile_world`` exactly
as Mission does with ``World`` and only sees observation dicts.

    python -m sim.grid_mission --seed 7 [--start-pile B2] [--radar dsp] [--realtime] [--speed 4] [--frames] \
        [--acoustic auto|stub|real] [--piles 3] --out dashboard/data

Isaac swap points (see contracts/INTERFACES.md, "v1.1 grid extensions"):
  * flight   : pass ``flight=`` (FlightBackend over the whole site; drone ids d1..d3)
  * world    : pass ``world=`` (``.grid`` site heightmap, ``.window(pile)``, ``.pile_world(pile)``,
               ``.sample_pointcloud(n)``, ``.pile_xy``) or feed LiDAR with ``ingest_pointcloud``
  * sensors  : pass ``sensor_factory=`` (pile_world, seed) -> object with ``radar_scan(...)``
  * acoustic : pass ``acoustic_fn=`` (scenario, drone_positions, exclude_piles, seed) -> acoustic/v1 dict
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

from .acoustic_grid import listen as acoustic_listen
from .coordinator import KinematicFlight, Mission
from .grid_world import GridWorld
from .radar_model import make_generator
from .scenario import BASE_XY, DRONE_PADS, SITE_BOUNDS, make_scenario
from .world import heightmap_from_points

LISTEN_S = 26.0                # listening window with rotors off (matches acoustic/scene.py)
PRIORITY_MIN = 20              # dispatch only to piles at or above this acoustic priority
SURVEY_RADIUS = 13.0
LANDING_RADIUS = 12.5
PILE_MAX_T = 600.0
SLAM_SCANS = 120               # LiDAR sweeps per pile under --slam (~50 s, ~0.2 m ATE)
SLAM_CACHE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dashboard", "data", "slam_cache")
V1_CLASS = {"voice": "voice", "shout": "shout", "knocking": "knocking", "tapping": "tapping"}


class PileMission(Mission):
    """Mission at one pile; hands control back to GridMission when the radar phase is over."""

    def __init__(self, *a, pile=None, owner=None, **kw):
        self.pile, self.owner = pile, owner
        super().__init__(*a, **kw)
        self.landing_kw = {"search_radius": LANDING_RADIUS}

    def _say(self, msg):
        super()._say(f"[{self.pile}] {msg}")

    def _before_landing(self):
        if self.owner is not None:
            self.owner._on_survey_done(self)

    def _start_listen(self):              # radar phase over -> GridMission takes over (listen / dispatch)
        self.pending = {}
        self.next_scan = None
        self.stage = "done"


class GridMission:
    def __init__(self, seed=7, start_pile=None, radar="model", out_dir=None, frames=False, verbose=True,
                 realtime=False, speed=1.0, dt=1.0, pile_budget=3, priority_min=PRIORITY_MIN, scan_budget=8,
                 policy="adaptive", acoustic="auto", n_points=360000, max_t=3000.0,
                 world=None, flight=None, sensor_factory=None, acoustic_fn=None, decider=None,
                 slam=False, slam_scans=SLAM_SCANS, slam_kw=None, slam_cache=SLAM_CACHE):
        self.seed, self.radar, self.policy, self.scan_budget = seed, radar, policy, scan_budget
        self.out_dir, self.frames, self.verbose = out_dir, frames, verbose
        self.realtime, self.speed, self.dt, self.max_t = realtime, speed, dt, max_t
        self.pile_budget, self.priority_min, self.acoustic_mode = pile_budget, priority_min, acoustic
        # --- backends (Isaac swap points)
        self.scenario = make_scenario(seed, start_pile)
        self.start_pile = self.scenario.public_dict()["start_pile"]     # operator choice, not truth
        self.world = world if world is not None else GridWorld(self.scenario)
        self.grid = self.world.grid
        pads = {f"d{i + 1}": tuple(xy) for i, xy in enumerate(DRONE_PADS)}
        self.flight = flight if flight is not None else KinematicFlight(self.grid, starts=pads)
        self.sensor_factory = sensor_factory or (lambda pw, s: make_generator(pw, s, radar))
        self.acoustic_fn = acoustic_fn
        self.decider = decider         # optional (context, options) -> {"choice", "rationale", "by"}
        self.slam, self.slam_scans, self.slam_kw, self.slam_cache = slam, slam_scans, dict(slam_kw or {}), slam_cache
        self.slam_metrics = {}         # pile -> metrics line (ATE, map error) for the log / state
        self._terrain = self.grid.z.copy()   # sim-diagnostic reference for the SLAM map-error line only
        self.decisions = []
        # --- mission state (never touches truth)
        self.t, self.k = 0.0, 0
        self.stage = "pile"            # pile | listen | return | done
        self.phase = "deploy"
        self.log = []
        self.piles = {name: {"id": name, "xy": [round(x, 2), round(y, 2)], "status": "unvisited", "priority": None,
                             "radar_candidate_ids": [], "why": "Not visited, nothing heard yet. Not cleared."}
                      for name, (x, y) in self.world.pile_xy.items()}
        self.visited = []
        self.done_scans, self.done_cands, self.done_sites = [], [], []
        self.clips, self.acoustic_obs, self.dispatch = [], [], []
        self.n_listen = 0
        self.listen_end = None
        self.acoustic_source = None
        self.last_acoustic = None
        self.pm = None
        self.n_scans = 0
        self._ingested = []
        self._pc_dirty = False
        self._hm_cache = None
        self.pc_count = 0
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
            if frames:
                os.makedirs(os.path.join(out_dir, "frames"), exist_ok=True)
            self._write_pointcloud(self.world.sample_pointcloud(n_points))
        self._say(f"Grid mission start (seed {seed}): 3 drones at the staging area, start pile {self.start_pile} "
                  f"(operator choice), pile budget {pile_budget}")
        self._start_pile(self.start_pile, reason="operator-selected start pile")

    # ------------------------------------------------------------------ helpers
    def _say(self, msg):
        self.log.append({"t": round(self.t, 1), "msg": msg})
        if self.verbose:
            print(f"[t={self.t:6.1f}] {msg}", flush=True)

    @property
    def done(self):
        return self.stage == "done"

    def _drone_xy(self):
        return {p["id"]: (float(p["pose"][0]), float(p["pose"][1])) for p in self.flight.poses()}

    def _start_pile(self, name, reason):
        cx, cy = self.world.pile_xy[name]
        pw = self.world.pile_world(name)
        prev = self.visited[-1] if self.visited else None
        self.visited.append(name)
        self.pm = PileMission(self.seed, self.policy, self.scan_budget, verbose=self.verbose, max_t=1e9, dt=self.dt,
                              world=pw, flight=self.flight, sensors=self.sensor_factory(pw, self.seed),
                              centre=(cx, cy), survey_radius=SURVEY_RADIUS, scan_offset=self.n_scans, t0=self.t,
                              log=self.log, pile=name, owner=self)
        self.pm_t0 = self.t
        if self.slam:
            self._slam_ingest(name)
        p = self.piles[name]
        p["status"] = "surveying"
        p["why"] = f"Fleet dispatched here ({reason}); LiDAR survey and landing search under way."
        self.stage = "pile"          # PileMission.__init__ already commanded the takeoff
        self._say(f"{'Dispatch' if prev else 'Deploy'} d1-d3 -> pile {name} at ({cx:.1f}, {cy:.1f}): {reason}")

    # ------------------------------------------------------------------ LiDAR ingest (Isaac)
    def ingest_pointcloud(self, points):
        """Feed world-frame LiDAR points (N,3). Used to rebuild the active pile's planning window
        before landing detection, and served as pointcloud.bin."""
        pts = np.asarray(points, np.float32).reshape(-1, 3)
        if len(pts):
            self._ingested.append(pts)
            self._pc_dirty = True

    def _ingested_points(self):
        if not self._ingested:
            return None
        pts = np.concatenate(self._ingested)
        if len(pts) > 1_500_000:
            pts = pts[np.random.default_rng(0).choice(len(pts), 1_500_000, replace=False)]
        self._ingested = [pts]
        return pts

    def _slam_map(self, name):
        """(points, metrics) of a LiDAR-SLAM orbit of pile ``name`` (mapping.slam), cached per (seed, pile, n)."""
        kw = self.slam_kw
        path = None
        if self.slam_cache and not kw:
            path = os.path.join(self.slam_cache, f"seed{self.seed}-{name}-n{self.slam_scans}.npz")
            if os.path.exists(path):
                try:
                    z = np.load(path)
                    return z["points"], dict(json.loads(str(z["metrics"])), cached=True)
                except (OSError, ValueError, KeyError):
                    pass
        from mapping.slam import ate, height_error, run_pile_slam   # lazy: kiss-icp / open3d
        r = run_pile_slam(self.scenario, name, seed=self.seed, n_scans=self.slam_scans, evaluate=False, **kw)
        pts, g = r["cloud"], r["grid"]
        rmse, _, end = ate(r["est_poses"], r["gt_poses"])
        # sim diagnostic: SLAM heightmap vs the simulated terrain on the observed cells near the pile
        ref = g.__class__(g.origin, g.res, g.w, g.h, np.zeros((g.h, g.w), np.float32))
        X, Y = ref.mesh()
        site = self.grid.__class__(self.grid.origin, self.grid.res, self.grid.w, self.grid.h, self._terrain)
        ref.z = site.height_at(X, Y).astype(np.float32)
        he = height_error(g, ref, g.observed, r["centre"])
        m = {"pile": name, "method": r["method"], "n_scans": self.slam_scans, "map_points": int(len(pts)),
             "ate_rmse_m": round(rmse, 3), "final_drift_m": round(end, 3),
             "map_mae_m": he.get("mae_m"), "map_p95_m": he.get("p95_abs_m"), "coverage": he.get("coverage"),
             "runtime_s": r["metrics"]["runtime"].get("total_s"), "note": r.get("note")}
        if path:
            try:
                os.makedirs(self.slam_cache, exist_ok=True)
                np.savez_compressed(path + ".tmp.npz", points=pts.astype(np.float32), metrics=json.dumps(m))
                os.replace(path + ".tmp.npz", path)
            except OSError:
                pass
        return pts, m

    def _slam_ingest(self, name):
        try:
            pts, m = self._slam_map(name)
        except Exception as e:                       # noqa: BLE001  (missing kiss-icp etc.: keep the mission going)
            self._say(f"[{name}] LiDAR SLAM unavailable ({type(e).__name__}: {str(e)[:120]}); using the prior map")
            return
        self.slam_metrics[name] = m
        self.ingest_pointcloud(pts)
        mae = "n/a" if m.get("map_mae_m") is None else f"{100 * m['map_mae_m']:.0f} cm"
        self._say(f"[{name}] LiDAR SLAM ({m['method']}, {m['n_scans']} sweeps{', cached' if m.get('cached') else ''}): "
                  f"{m['map_points']} map points, ATE {100 * m['ate_rmse_m']:.0f} cm, final drift "
                  f"{100 * m['final_drift_m']:.0f} cm, map height error {mae} MAE (sim diagnostic vs scenario terrain)")

    def _on_survey_done(self, pm):
        pts = self._ingested_points()
        if pts is None:
            return
        g = pm.grid
        x0, y0, x1, y1 = g.bounds
        sel = (pts[:, 0] >= x0) & (pts[:, 0] < x1) & (pts[:, 1] >= y0) & (pts[:, 1] < y1)
        if sel.sum() < 2000:
            return
        from scipy.ndimage import maximum_filter
        sub = pts[sel]
        new = heightmap_from_points(sub, g.origin, g.res, g.w, g.h)
        hit = np.zeros((g.h, g.w), bool)
        ix = np.clip(((sub[:, 0] - g.origin[0]) / g.res).astype(int), 0, g.w - 1)
        iy = np.clip(((sub[:, 1] - g.origin[1]) / g.res).astype(int), 0, g.h - 1)
        hit[iy, ix] = True
        covered = maximum_filter(hit, size=7)
        # the window keeps neighbouring piles flattened (GridWorld.window); only this pile's cells take the map
        X, Y = g.mesh()
        cx, cy = self.world.pile_xy[pm.pile]
        d_own = np.hypot(X - cx, Y - cy)
        d_other = np.min([np.hypot(X - x, Y - y) for n, (x, y) in self.world.pile_xy.items() if n != pm.pile], axis=0)
        g.z[:] = np.where(covered & (d_own <= d_other), new.z, g.z)
        # write back into the site map (flight + dashboard) and restart the brain on the new map
        i0 = int(round((g.origin[0] - self.grid.origin[0]) / g.res))
        j0 = int(round((g.origin[1] - self.grid.origin[1]) / g.res))
        self.grid.z[j0:j0 + g.h, i0:i0 + g.w] = np.where(covered, new.z, self.grid.z[j0:j0 + g.h, i0:i0 + g.w])
        self._hm_cache = None
        from .localise import Localiser
        from .planner import Planner
        pm.loc = Localiser(g)
        pm.planner = Planner(g, pm.loc.cfg, seed=self.seed)
        pm._say(f"Heightmap window rebuilt from {int(sel.sum())} LiDAR points")

    # ------------------------------------------------------------------ main loop
    def step(self):
        if self.stage == "pile":
            self.pm.step()
            self.t, self.k = self.pm.t, self.k + 1
            self._track_pile()
            if self.pm.stage == "done" or self.t - self.pm_t0 > PILE_MAX_T:
                self._finish_pile()
        else:
            self.t += self.dt
            self.k += 1
            self.flight.step(self.dt, self.t)
            if self.stage == "listen" and self.t >= self.listen_end:
                self._finish_listen()
            elif self.stage == "return" and all(p["state"] == "docked" for p in self.flight.poses()):
                self.stage = "done"
                self._say("All drones docked at the staging area. Mission complete.")
        self._update_phase()
        if self.t >= self.max_t and self.stage != "done":
            self._say(f"Mission time limit ({self.max_t:.0f} s) reached; stopping")
            self.stage = "done"
        self._write_state()
        if self.realtime:
            time.sleep(self.dt / self.speed)

    def run(self):
        self._write_state()
        while not self.done:
            self.step()
        return self

    def _update_phase(self):
        if self.stage == "pile":
            ph = self.pm.phase
            if len(self.visited) > 1 and self.pm.stage in ("takeoff", "survey"):
                cx, cy = self.world.pile_xy[self.pm.pile]
                far = np.mean([np.hypot(x - cx, y - cy) for x, y in self._drone_xy().values()]) > SURVEY_RADIUS + 3
                if self.pm.stage == "takeoff" or far:
                    ph = "dispatch"
            self.phase = ph
        elif self.stage == "listen":
            self.phase = "acoustic"
        elif self.stage == "return":
            self.phase = "return"
        else:
            self.phase = "done"

    # ------------------------------------------------------------------ per-pile bookkeeping
    def _pref(self, sid):
        return f"{self.pm.pile}-{sid}"

    def _live_cands(self, pm):
        out = []
        for c in pm.cands:
            if c["kind"] != "stationary":
                continue
            c = dict(c, id=f"{pm.pile}-{c['id']}", pile_id=pm.pile,
                     evidence=list(c.get("evidence", [])))
            out.append(c)
        return out

    def _track_pile(self):
        p = self.piles[self.pm.pile]
        if self.pm.phase in ("landing", "radar", "relocate", "rescan") or self.pm.scans_cmd:
            if p["status"] == "surveying":
                p["status"] = "scanning"
                p["why"] = "Drones landed on flat cells; radar scanning for a breathing-like return."

    def _finish_pile(self):
        pm, name = self.pm, self.pm.pile
        p = self.piles[name]
        for o in pm.scans:
            o["pile_id"] = name
        self.done_scans.extend(pm.scans)
        self.n_scans = pm.scan_idx
        self.done_sites.extend(dict(s, id=f"{name}-{s['id']}", pile_id=name) for s in pm.sites if s["status"] == "used")
        cands = self._live_cands(pm)
        firm = [c for c in cands if c["status"] == "candidate"]
        keep = firm or ([cands[0]] if cands and cands[0]["p_mass"] >= 0.3 else [])
        self.done_cands.extend(keep)
        p["radar_candidate_ids"] = [c["id"] for c in keep]
        res = pm.result
        if firm:
            c = firm[0]
            depth = pm.grid.height_at(*c["xy"]) - c["z_est"]
            p["status"] = "ping"
            p["why"] = (f"Radar ping {c['id']} at ({c['xy'][0]:.1f}, {c['xy'][1]:.1f}), p {c['p_mass']:.2f}, "
                        f"≈{depth:.1f} m below the surface, {len(c['evidence'])} supporting scans. Candidate search "
                        "region, not a confirmed person.")
            self._say(f"PING {c['id']} at pile {name}: ({c['xy'][0]:.1f}, {c['xy'][1]:.1f}), depth ≈{depth:.1f} m, "
                      f"p {c['p_mass']:.2f} -> sent to the operator dashboard")
        else:
            lead = (p["priority"] or 0) >= self.priority_min
            p["status"] = "lead" if lead else "no_signal_not_cleared"
            p["why"] = (f"Radar inconclusive after {pm.scans_cmd} scans ({res.get('reason', 'no data')})"
                        + ("; acoustic lead stands" if lead else "") + ". No detection is not proof of absence: not cleared.")
            self._say(f"No firm radar ping at {name} ({res.get('reason', 'no data')}); pile stays not cleared")
        self.stage = "listen"
        self.listen_end = self.t + LISTEN_S
        self._say(f"Radar phase at {name} over; landed drones listen to the other piles for {LISTEN_S:.0f} s "
                  "and stream clips to the acoustic agent")

    # ------------------------------------------------------------------ acoustic + dispatch
    def _finish_listen(self):
        self.n_listen += 1
        drones = self._drone_xy()
        exclude = list(self.visited)
        if self.acoustic_fn is not None:
            res, src = self.acoustic_fn(self.scenario, drones, exclude, self.seed + 101 * self.n_listen), "custom"
        else:
            res, src = acoustic_listen(self.scenario, drones, exclude, self.seed + 101 * self.n_listen,
                                       mode=self.acoustic_mode, log=self._say)
        self.acoustic_source, self.last_acoustic = src, res
        by = (res.get("agent") or {}).get("mode")
        self._say(f"Acoustic agent ({src}{', mode ' + by if by and by != src else ''}): {len(res['clips'])} clips from "
                  f"{len(res['piles'])} unvisited piles")
        for c in res["clips"]:
            origin = drones.get(c.get("drone_id"), (0.0, 0.0))
            clip = {"id": f"{self.n_listen}-{c['clip_id']}", "listen": self.n_listen, "t": round(self.t, 1),
                    "pile_id": c["pile_id"], "drone_id": c.get("drone_id"),
                    "origin": [round(origin[0], 2), round(origin[1], 2)],
                    "bearing_deg": c.get("bearing_deg"), "width_deg": c.get("width_deg"), "snr_db": c.get("snr_db"),
                    "class": c.get("sound_class"), "transcript": c.get("transcript", ""),
                    "class_scores": c.get("class_scores", {}),
                    "scores": {k: c.get(k) for k in ("human", "urgency", "reliability")},
                    "priority": c.get("priority"), "rationale": c.get("rationale", ""), "by": c.get("by", "fallback"),
                    "doa_agree": c.get("doa_agree")}
            self.clips.append(clip)
            said = f" \"{clip['transcript']}\"" if clip["transcript"] else ""
            self._say(f"Clip {clip['id']} ({clip['drone_id']}, bearing {clip['bearing_deg']:.0f}°): "
                      f"{clip['class']}{said}; human {clip['scores']['human']:.2f}, urgency "
                      f"{clip['scores']['urgency']:.2f} -> priority {clip['priority']} for {clip['pile_id']} [{clip['by']}]")
            if (clip["scores"]["human"] or 0) >= 0.4 and clip["bearing_deg"] is not None:
                self.acoustic_obs.append({"obs_id": f"a{len(self.acoustic_obs) + 1}", "drone_id": clip["drone_id"],
                                          "origin": clip["origin"], "timestamp": clip["t"],
                                          "bearing_deg": clip["bearing_deg"], "width_deg": clip["width_deg"],
                                          "class": V1_CLASS.get(clip["class"], "unknown"),
                                          "confidence": clip["scores"]["reliability"], "pile_id": clip["pile_id"]})
        for ap in res["piles"]:
            p = self.piles.get(ap["pile_id"])
            if p is None or ap["pile_id"] in self.visited:
                continue
            new = int(ap["priority"])
            if ap["status"] == "no_signal_not_cleared" and (p["priority"] or 0) > new:
                continue                                   # silence from here does not erase an earlier lead
            st = ap["status"] if ap["status"] in ("lead", "dismissed_nonhuman", "no_signal_not_cleared") else "lead"
            p.update(priority=new, status=st, why=ap["why"])
        ranked = sorted((p for p in self.piles.values() if p["id"] not in self.visited and p["priority"] is not None),
                        key=lambda p: (-p["priority"], p["id"]))
        self._say("Pile priority: " + ", ".join(f"{p['id']} {p['priority']}" for p in ranked[:6]))
        here = self.visited[-1]
        nxt, reason = self._decide_dispatch(ranked, res)
        self.dispatch.append({"t": round(self.t, 1), "from_pile": here, "to_pile": nxt["id"] if nxt else None,
                              "reason": reason})
        if nxt:
            self._start_pile(nxt["id"], reason)
        else:
            left = [f"{p['id']} {p['priority']}" for p in ranked if p["status"] == "lead"]
            self._say(f"Search loop ends: {reason}. Open leads for ground teams: {', '.join(left) or 'none'}. "
                      "Drones return to the staging area")
            for d in self.flight.drones:
                self.flight.command({"drone_id": d, "type": "return"})
            self.pm = None
            self.stage = "return"

    # ------------------------------------------------------------------ dispatch gate (optional decider)
    PILE_BATTERY = 0.14          # rough battery cost of one pile (survey + landings + scans + listen)

    def _options(self, ranked, res):
        """Scored options at the dispatch gate. Returns (options, argmax_option_id, argmax_reason)."""
        here = self.visited[-1]
        poses = {p["id"]: p for p in self.flight.poses()}
        bat_min = min(p["battery"] for p in poses.values())
        clips_by = {}
        for c in self.clips:
            clips_by.setdefault(c["pile_id"], []).append(c)
        budget_left = self.pile_budget - len(self.visited)
        opts = []

        def travel(pid):
            x, y = self.world.pile_xy[pid]
            ts = [self.flight.eta(d, [x, y, float(self.grid.height_at(x, y)) + 9.0]) for d in poses]
            return None if any(t is None for t in ts) else round(max(ts), 1)

        if budget_left > 0:
            for p in ranked[:6]:
                ts = travel(p["id"])
                cs = sorted(clips_by.get(p["id"], []), key=lambda c: -(c["priority"] or 0))[:2]
                ev = "; ".join(f"{c['class']}" + (f" \"{c['transcript'][:60]}\"" if c["transcript"] else "")
                               + f" (human {c['scores']['human']}, priority {c['priority']}, {c['by']})" for c in cs) \
                    or "no clip assigned"
                flags = [f for f, on in (("below_threshold", p["priority"] < self.priority_min),
                                         ("nonhuman", p["status"] == "dismissed_nonhuman"),
                                         ("silent", p["status"] == "no_signal_not_cleared"),
                                         ("unreachable", ts is None)) if on]
                opts.append({"id": f"o{len(opts) + 1}", "action": "dispatch", "pile_id": p["id"],
                             "priority": p["priority"], "travel_s": ts,
                             "battery_after_min": None if ts is None else
                             round(bat_min - 0.0016 * ts - self.PILE_BATTERY, 2),
                             "evidence": ev, "flags": flags})
            cur = self.piles[here]
            if cur["status"] != "ping" and self.visited.count(here) < 2:
                opts.append({"id": f"o{len(opts) + 1}", "action": "rescan_current", "pile_id": here,
                             "priority": cur["priority"], "travel_s": 0.0,
                             "battery_after_min": round(bat_min - self.PILE_BATTERY, 2),
                             "evidence": f"radar at {here}: {cur['why'][:160]}", "flags": ["no_firm_ping"]})
        opts.append({"id": f"o{len(opts) + 1}", "action": "stop", "pile_id": None, "priority": None, "travel_s": None,
                     "battery_after_min": round(bat_min, 2), "evidence": "return to the staging area",
                     "flags": ["pile_budget_used"] if budget_left <= 0 else []})
        stop = opts[-1]["id"]
        if budget_left <= 0:
            return opts, stop, f"pile budget ({self.pile_budget}) used"
        ok = [o for o in opts if o["action"] == "dispatch" and "unreachable" not in o["flags"]
              and o["priority"] >= self.priority_min]
        if not ok:
            return opts, stop, f"no unvisited pile at or above priority {self.priority_min}; remaining piles stay not cleared"
        top = ok[0]
        d = res.get("dispatch") or {}
        p = self.piles[top["pile_id"]]
        reason = d["reason"] if d.get("pile_id") == top["pile_id"] and d.get("reason") else \
            f"highest acoustic priority ({p['priority']}) among unvisited piles: {p['why']}"
        return opts, top["id"], reason

    def _decide_dispatch(self, ranked, res):
        opts, best, reason = self._options(ranked, res)
        by, rationale, latency = "argmax", reason, 0.0
        chosen = best
        if self.decider is not None and len(opts) > 1:
            ctx = {"t": round(self.t, 1), "current_pile": self.visited[-1], "visited": list(self.visited),
                   "pile_budget": self.pile_budget, "budget_left": self.pile_budget - len(self.visited),
                   "priority_min": self.priority_min,
                   "pings": [{"id": c["id"], "pile_id": c["pile_id"], "status": c["status"], "p_mass": c["p_mass"]}
                             for c in self.done_cands],
                   "piles": [{k: p[k] for k in ("id", "status", "priority", "why")} for p in self.piles.values()],
                   "fleet": [{"id": p["id"], "battery": p["battery"], "state": p["state"]} for p in self.flight.poses()]}
            t0 = time.time()
            try:
                out = self.decider(ctx, opts) or {}
                ids = {o["id"] for o in opts}
                if out.get("choice") in ids:
                    chosen, by = out["choice"], str(out.get("by", "decider"))
                    rationale = str(out.get("rationale") or reason)
                else:
                    by, rationale = "fallback", f"decider returned invalid choice {out.get('choice')!r}; argmax: {reason}"
            except Exception as e:                       # noqa: BLE001  (never let the agent stall the mission)
                by, rationale = "fallback", f"decider error {type(e).__name__}: {e}; argmax: {reason}"
            latency = (time.time() - t0) * 1000
        opt = next(o for o in opts if o["id"] == chosen)
        self.decisions.append({"t": round(self.t, 1), "gate": "dispatch", "options": opts, "chosen": chosen, "by": by,
                               "rationale": rationale, "latency_ms": round(latency, 1)})
        if by != "argmax":
            self._say(f"Decision ({by}, {latency:.0f} ms): {opt['action']} {opt['pile_id'] or ''} - {rationale}")
        if opt["action"] == "stop":
            return None, rationale if by != "argmax" else reason
        return self.piles[opt["pile_id"]], rationale

    # ------------------------------------------------------------------ state.json
    def _write_pointcloud(self, pts):
        pts.astype("<f4").tofile(os.path.join(self.out_dir, "pointcloud.bin.tmp"))
        os.replace(os.path.join(self.out_dir, "pointcloud.bin.tmp"), os.path.join(self.out_dir, "pointcloud.bin"))
        self.pc_count = int(len(pts))

    def _hm_list(self):
        if self._hm_cache is None:
            z = np.round(self.grid.z.astype(np.float64), 3).ravel().tolist()
            self._hm_cache = (z, json.dumps(z, separators=(",", ":")))
        return self._hm_cache

    def _drones_out(self):
        out = []
        for d in self.flight.poses():
            d = dict(d)
            if self.stage == "listen" and d["state"] in ("landed", "scanning"):
                d["state"] = "listening"
            out.append(d)
        return out

    def _state(self, hm_placeholder=False):
        g = self.grid
        pm = self.pm if self.stage == "pile" else None
        scans = self.done_scans + ([dict(o, pile_id=pm.pile) for o in pm.scans] if pm else [])
        cands = self.done_cands + (self._live_cands(pm) if pm else [])
        sites = list(self.done_sites)
        next_scan = None
        belief = None
        if pm:
            sites += [dict({k: v for k, v in s.items() if k != "reason"}, id=f"{pm.pile}-{s['id']}", pile_id=pm.pile)
                      for s in pm.sites]
            ns = pm._next_scan_out()
            if ns:
                next_scan = dict(ns, site_id=f"{pm.pile}-{ns['site_id']}", pile_id=pm.pile)
            if pm.scans:
                bg = pm.grid
                belief = {"origin": list(bg.origin), "res": bg.res, "w": bg.w, "h": bg.h, "pile_id": pm.pile,
                          "p": [float(f"{v:.4g}") for v in pm.loc.belief().ravel()]}
        upcoming = None
        if self.stage == "pile" and self.dispatch and self.dispatch[-1]["to_pile"] == self.visited[-1] \
                and self.pm.stage in ("takeoff", "survey"):
            upcoming = self.dispatch[-1]
        return {
            "schema": "larpverse.state/v1", "t": round(self.t, 1), "phase": self.phase,
            "site": {"name": f"Rubble grid 3x3 · seed {self.seed}", "bounds": list(SITE_BOUNDS)},
            "heightmap": {"origin": list(g.origin), "res": g.res, "w": g.w, "h": g.h,
                          "z": "__HM__" if hm_placeholder else self._hm_list()[0]},
            "pointcloud": {"url": "pointcloud.bin", "count": self.pc_count},
            "drones": self._drones_out(),
            "landing_sites": sites,
            "scans": scans,
            "radar_candidates": cands,
            "acoustic": self.acoustic_obs,
            "belief": belief,
            "next_scan": next_scan,
            "log": self.log[-200:],
            # ---- v1.1 grid extensions
            "mission": {"kind": "grid", "seed": self.seed, "start_pile": self.start_pile,
                        "pile_budget": self.pile_budget, "priority_min": self.priority_min,
                        "visited": list(self.visited), "active_pile": self.visited[-1] if self.stage in ("pile", "listen") else None,
                        "stage": self.stage, "acoustic_source": self.acoustic_source,
                        "listen_ends": round(self.listen_end, 1) if self.stage == "listen" else None,
                        "in_transit": upcoming},
            "staging": {"xy": list(BASE_XY), "pads": [list(p) for p in DRONE_PADS]},
            "piles": [dict(p) for p in self.piles.values()],
            "acoustic_clips": self.clips,
            "acoustic_agent": (self.last_acoustic or {}).get("agent"),
            "dispatch": self.dispatch,
            "decisions": self.decisions,
            "slam": list(self.slam_metrics.values()) if self.slam else None,
        }

    def build_state(self):
        """Full state dict (contract v1 + v1.1 grid extensions), exactly what is written to state.json."""
        return self._state()

    def _write_state(self):
        if not self.out_dir:
            return
        if self._pc_dirty:
            pts = self._ingested_points()
            if pts is not None:
                self._write_pointcloud(pts)
            self._pc_dirty = False
        s = json.dumps(self._state(hm_placeholder=True), separators=(",", ":"), ensure_ascii=False)
        s = s.replace('"__HM__"', self._hm_list()[1], 1)
        paths = [os.path.join(self.out_dir, "state.json")]
        if self.frames:
            paths.append(os.path.join(self.out_dir, "frames", f"state_{self.k:04d}.json"))
        for path in paths:
            tmp = path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                f.write(s)
            os.replace(tmp, path)

    # ------------------------------------------------------------------ report
    def summary(self):
        return {"visited": list(self.visited), "dispatch": self.dispatch,
                "pings": [{"id": c["id"], "pile": c["pile_id"], "xy": c["xy"], "z_est": c["z_est"],
                           "status": c["status"], "p_mass": c["p_mass"]} for c in self.done_cands],
                "priorities": {p["id"]: p["priority"] for p in self.piles.values()},
                "statuses": {p["id"]: p["status"] for p in self.piles.values()},
                "acoustic_source": self.acoustic_source, "t": self.t, "scans": len(self.done_scans)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--start-pile", default=None, help="A1..C3 (default: seeded random pile)")
    ap.add_argument("--out", default="dashboard/data")
    ap.add_argument("--radar", choices=["model", "dsp"], default="model")
    ap.add_argument("--acoustic", choices=["auto", "stub", "real"], default="auto",
                    help="auto: acoustic/ pipeline if importable or via .venv-audio, else the synthetic stub")
    ap.add_argument("--piles", type=int, default=3, help="pile budget")
    ap.add_argument("--priority-min", type=int, default=PRIORITY_MIN)
    ap.add_argument("--frames", action="store_true")
    ap.add_argument("--realtime", action="store_true")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--decider", choices=["none", "gemini"], default="none",
                    help="dispatch-gate agent (agents.gemini_decider.make_decider); falls back to argmax")
    ap.add_argument("--slam", action="store_true",
                    help="build each pile's map with LiDAR SLAM (mapping.slam, KISS-ICP); cached in dashboard/data/slam_cache")
    ap.add_argument("--slam-scans", type=int, default=SLAM_SCANS)
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args(argv)
    t0 = time.time()
    decider = None
    if a.decider == "gemini":
        try:
            from agents.gemini_decider import make_decider
            decider = make_decider()
        except Exception as e:                           # noqa: BLE001
            print(f"gemini decider unavailable ({type(e).__name__}: {e}); using argmax", file=sys.stderr)
    m = GridMission(a.seed, a.start_pile, a.radar, a.out, a.frames, not a.quiet, a.realtime, a.speed,
                    pile_budget=a.piles, priority_min=a.priority_min, acoustic=a.acoustic, decider=decider,
                    slam=a.slam, slam_scans=a.slam_scans)
    m.run()
    s = m.summary()
    print(f"done in {time.time() - t0:.1f} s wall, {m.t:.0f} s sim; visited {' -> '.join(s['visited'])}; "
          f"pings {[c['id'] for c in s['pings'] if c['status'] == 'candidate']}; acoustic {s['acoustic_source']}; "
          f"wrote {a.out}/state.json", file=sys.stderr)


if __name__ == "__main__":
    main()
