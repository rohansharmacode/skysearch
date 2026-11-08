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
  * speaker  : ``speaker=True`` (default): loudspeaker call-out after each passive listen, reassurance after
               a ping / response, route guidance to people on open ground (sim/callout.py, contract v1.3);
               pass ``callout_fn=`` (scenario, plan, seed) -> {"detections": [...]} to replace the generator
  * comms    : ``comms=True`` (default): drone Wi-Fi hotspots + mesh backhaul to the van, captive-portal events,
               consent-based phone localisation (sim/comms.py, contract v1.4); a located phone inside a pile is
               new evidence (lead / re-open), never a confirmed person. CLI ``--no-comms``, ``--uplink``
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

from .acoustic_grid import STUB_SOURCE, listen as acoustic_listen
from .coordinator import KinematicFlight, Mission
from .grid_world import GridWorld
from . import radar_dsp
from .radar_dsp import REACH_REF_RANGE_M, reach_path_m
from .landing import detect_landing_sites, detect_pod_sites
from .radar_model import make_generator
from .recon import Recon, proposals_to_tasking
from .tasking import Tasking, ZoneTracker, default_tasking, load_tasking, tasking_mtime
from .sensor_model import rubble_path
from .scenario import BASE_XY, DRONE_PADS, SITE_BOUNDS, make_scenario
try:
    from .scenario import SITE_GEO
except ImportError:                    # older scenario module
    SITE_GEO = None
from .world import heightmap_from_points

LISTEN_S = 26.0                # listening window with rotors off (matches acoustic/scene.py)
PRIORITY_MIN = 20              # dispatch only to piles at or above this acoustic priority
SURVEY_RADIUS = 13.0
LANDING_RADIUS = 12.5
PILE_MAX_T = 600.0
SLAM_SCANS = 120               # LiDAR sweeps per pile under --slam (~50 s, ~0.2 m ATE)
SLAM_CACHE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dashboard", "data", "slam_cache")
MISSION_SCANS = 24             # autonomous: radar scans for the whole mission (all piles, all drones), landing ring
MISSION_SCANS_POD = 36         # winch pod: a re-placement is ~20 s + a 36 s winch/scan cycle instead of a re-land
RESERVE = 0.25                 # every drone keeps this battery fraction; below it the fleet swaps packs
SWAP_S = 60.0                  # battery swap time at the pads once all drones are docked
VITALS_LOOK_S = 60.0           # one 60 s look per firm ping (pod re-lowered / landed drone in place): patterns need 45-60 s
CALLOUT_LANGS = ("zh-TW", "nan-TW", "en")
REVISIT_MIN_COVERAGE = 0.25    # revisit an open lead only if the radar can reach most of its top 2 m
REVISIT_DISCOUNT = 0.5         # an open lead already scanned ranks at half its acoustic priority
# radar_dsp presets: literature | legacy (docs/RADAR_PENETRATION.md). Default stays "legacy" so the seed-7
# demo keeps its pings; with "literature" the landing ring is out of radar reach (product decision pending).
PENETRATION_DEFAULT = "literature"   # realistic (~3 m beneath); "legacy" only for comparison
DELIVERY_DEFAULT = "pod"             # winch-lowered sensor pod on the pile surface; "land" = the old landing ring
RASTER = ["A1", "B1", "C1", "C2", "B2", "A2", "A3", "B3", "C3"]   # fixed lawnmower baseline (row by row)
V1_CLASS = {"voice": "voice", "shout": "shout", "knocking": "knocking", "tapping": "tapping"}
# operator tasking (contracts v1.2, sim/tasking.py) and recon (sim/recon.py)
LAND_ZONE_MARGIN_M = 4.0       # landing patches may lie this far outside the tasked zone (radar looks in)
MIN_ZONE_SITES = 3             # ... unless fewer than this many candidates are left inside it
LOC_ZONE_MARGIN_M = 1.0        # localiser prior: full weight inside the zone (+ this margin) ...
ZONE_PRIOR_FLOOR = 0.02        # ... and this relative weight outside it
LIDAR_SURVEY_M = SURVEY_RADIUS + 5.0   # the per-pile LiDAR survey maps cells this close to the pile centre
RECON_ACCEPT_S = 30.0          # autonomous: auto-accept recon proposals after this long without operator tasking
RECON_ACCEPT_MIN = 0.3         # ... those scoring at least this


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

    zone_id = None
    zone_mask = None                      # bool on self.grid: landing patches allowed (zone + LAND_ZONE_MARGIN_M)

    def _advance(self):
        before = self.stage
        super()._advance()
        if before == "survey" and self.stage == "decide" and self.zone_mask is not None:
            cand = [s for s in self.sites if s["status"] == "candidate"]
            out = []
            for s in cand:
                i, j = self.grid.index(*s["xy"])
                if not self.zone_mask[j, i]:
                    out.append(s)
            if len(cand) - len(out) >= MIN_ZONE_SITES:
                for s in out:
                    s["status"], s["reason"] = "rejected", "outside_zone"
                if out:
                    self._say(f"{len(out)} landing patches lie outside tasked zone {self.zone_id} "
                              f"(+{LAND_ZONE_MARGIN_M:.0f} m); not used")
            elif out:                     # too few inside: keep them all rather than strand the radar
                self._say(f"only {len(cand) - len(out)} landing patches inside tasked zone {self.zone_id} "
                          f"(+{LAND_ZONE_MARGIN_M:.0f} m); also using the {len(out)} nearby ones outside it")


class GridMission:
    def __init__(self, seed=7, start_pile=None, radar="dsp", out_dir=None, frames=False, verbose=True,
                 realtime=False, speed=1.0, dt=1.0, pile_budget=None, priority_min=PRIORITY_MIN, scan_budget=8,
                 policy="adaptive", acoustic="auto", n_points=360000, max_t=None,
                 world=None, flight=None, sensor_factory=None, acoustic_fn=None, decider=None,
                 slam=False, slam_scans=SLAM_SCANS, slam_kw=None, slam_cache=SLAM_CACHE,
                 autonomous=True, mission_scans=None, dispatch_policy="acoustic", reserve=RESERVE, swap_s=SWAP_S,
                 listen_s=None, revisit_max=2, penetration=None, launch_pile=None, site="grid", platform=None,
                 delivery=None, speaker=True, callout_fn=None,
                 tasking=None, recon_accept_s=RECON_ACCEPT_S, comms=True, uplink="oneweb", comms_llm=False):
        """``autonomous`` (default): keep searching until the mission scan budget (``mission_scans``, default
        24), the time limit or the open leads run out; battery swaps at the pads; open leads without a firm
        ping are revisited once. ``pile_budget`` caps pile visits (None = no cap; legacy mode uses 3).
        ``dispatch_policy``: acoustic (ours) | nearest | raster (baselines for sim/evaluate_grid.py; they do
        not listen unless ``listen_s`` is given).
        ``tasking`` (contracts v1.2): None / "none" = legacy (every pile, no zones); "default" = scenario-default
        zones; a path = operator tasking file, re-read each step (mtime check; no active zones -> recon first);
        or a ``sim.tasking.Tasking``. ``recon_accept_s``: autonomous auto-accept delay for recon proposals."""
        if dispatch_policy not in ("acoustic", "nearest", "raster"):
            raise ValueError(f"dispatch_policy {dispatch_policy!r}")
        self.delivery = delivery or DELIVERY_DEFAULT
        if self.delivery not in ("pod", "land"):
            raise ValueError(f"delivery {self.delivery!r}")
        if platform is None and self.delivery == "pod":
            platform = "heavy_sar"                     # the winch pod rides on the heavy drone
        self.penetration = "literature" if (penetration or PENETRATION_DEFAULT) == "realistic" \
            else (penetration or PENETRATION_DEFAULT)
        radar_dsp.use_penetration(self.penetration)     # process-wide: generator, planner and coverage agree
        rx = 0.0
        if platform not in (None, "legacy_small"):
            from .platform import get_platform, radar_rx_advantage_db
            rx = radar_rx_advantage_db(get_platform(platform))
        radar_dsp.use_rx_gain_db(rx)                    # R-PLAT-5: bigger receive aperture -> more SNR
        self.seed, self.radar, self.policy, self.scan_budget = seed, radar, policy, scan_budget
        self.out_dir, self.frames, self.verbose = out_dir, frames, verbose
        self.autonomous, self.dispatch_policy, self.reserve, self.swap_s = autonomous, dispatch_policy, reserve, swap_s
        self.revisit_max = revisit_max
        self.mission_scans = mission_scans if mission_scans is not None else (
            (MISSION_SCANS_POD if self.delivery == "pod" else MISSION_SCANS) if autonomous else None)
        self.listen_s = float(listen_s) if listen_s is not None else (LISTEN_S if dispatch_policy == "acoustic" else 0.0)
        max_t = max_t if max_t is not None else (3600.0 if autonomous else 3000.0)
        pile_budget = pile_budget if pile_budget is not None else (None if autonomous else 3)
        self.realtime, self.speed, self.dt, self.max_t = realtime, speed, dt, max_t
        self.pile_budget, self.priority_min, self.acoustic_mode = pile_budget, priority_min, acoustic
        # --- backends (Isaac swap points)
        self.site = site
        self.scenario = make_scenario(seed, start_pile, site=site) if site != "grid" else make_scenario(seed, start_pile)
        pd = self.scenario.public_dict()         # site layout is scenario data, not module constants
        self.bounds = list(pd.get("bounds") or SITE_BOUNDS)
        self.base_xy = list(pd.get("base_xy") or BASE_XY)
        self.pads = [list(p) for p in (pd.get("drone_pads") or DRONE_PADS)]
        self.geo = pd.get("geo") or SITE_GEO
        meta = pd.get("site")
        self.site_name = (meta.get("name") if isinstance(meta, dict) else None) or \
            (self.geo or {}).get("name") if site != "grid" else "Rubble grid 3x3"
        # operator input; when not given, the scenario's suggested start (reported collapse) is used
        # ``launch_pile`` (evaluation): start somewhere chosen independently of where the scenario put people
        self.start_pile = launch_pile or start_pile or self.scenario.public_dict()["start_pile"]
        self.world = world if world is not None else GridWorld(self.scenario)
        self.grid = self.world.grid
        pads = {f"d{i + 1}": tuple(xy) for i, xy in enumerate(self.pads[:3])}
        self.platform = platform           # sim.platform preset (None / "legacy_small": today's numbers)
        self.platform_spec = None
        if platform not in (None, "legacy_small"):
            from .platform import get_platform
            self.platform_spec = get_platform(platform)
        self.flight = flight if flight is not None else KinematicFlight(
            self.grid, starts=pads, platform=None if self.platform_spec is None else self.platform_spec,
            max_alt=pd.get("flight_ceiling_m"))
        self.sensor_factory = sensor_factory or (lambda pw, s: make_generator(pw, s, radar))
        self.acoustic_fn = acoustic_fn
        self.decider = decider         # optional (context, options) -> {"choice", "rationale", "by"}
        self.slam, self.slam_scans, self.slam_kw, self.slam_cache = slam, slam_scans, dict(slam_kw or {}), slam_cache
        self.slam_metrics = {}         # pile -> metrics line (ATE, map error) for the log / state
        self._terrain = self.grid.z.copy()   # sim-diagnostic reference for the SLAM map-error line only
        self.decisions = []
        # loudspeaker (contract v1.3): call-out + listen per listen phase, reassurance, guidance; never reads truth
        # itself: the scenario goes only to the call-out response generator, as with the acoustic pipeline
        self.speaker, self.callout_fn = None, callout_fn
        if speaker and self.listen_s > 0:
            from .callout import Speaker
            # product decision: 3 call-out languages by default (zh-TW, nan-TW, en); others only when selected / detected
            self.speaker = Speaker(self.world.pile_xy, self.grid, self.base_xy, seed=seed, langs=CALLOUT_LANGS)
        self._pending_callout = None
        self._zone_ai = []             # decider zone suggestions (suggestions only)
        # --- mission state (never touches truth)
        self.t, self.k = 0.0, 0
        self.stage = "pile"            # pile | listen | return | done
        self.phase = "deploy"
        self.log = []
        polys = {getattr(p, "name", None): getattr(p, "polygon", None) for p in getattr(self.scenario, "piles", [])}
        self.piles = {name: {"id": name, "xy": [round(x, 2), round(y, 2)], "status": "unvisited", "priority": None,
                             "polygon": self._polygon(polys.get(name), x, y),
                             "radar_candidate_ids": [], "why": "Not visited, nothing heard yet. Not cleared."}
                      for name, (x, y) in self.world.pile_xy.items()}
        self._tracks = {}              # drone id -> last <= 200 [x, y, z, t] samples (drones[].track)
        # rescue hotspot + comms relay (contract v1.4): the phone truth lives in the RadioSim generator inside
        # sim.comms (like the call-out generator); the mission only sees joined devices, RSSI fits and portal taps
        self.comms, self._comms_seen = None, set()
        if comms:
            from .comms import Comms
            self.comms = Comms(self.scenario, self.grid, self.base_xy, seed=seed, site="WEIGUAN",
                               sectors={n: {"xy": p["xy"], "polygon": p["polygon"]} for n, p in self.piles.items()},
                               internet=uplink is not None, gcs_uplink=uplink, use_llm=comms_llm)
        self.visited = []
        self.done_scans, self.done_cands, self.done_sites = [], [], []
        self.clips, self.acoustic_obs, self.dispatch = [], [], []
        self.n_listen = 0
        self.listen_end = None
        self.acoustic_source = None
        self.last_acoustic = None
        self.pm = None
        self.n_scans = 0               # observation index counter (scan ids)
        self.scans_used = 0            # radar scans commanded, whole mission
        self.n_swaps = 0
        self.swap = None               # {"to_pile", "reason", "ends", "t"} while returning / swapping
        self._fly_s = 0.0              # drone-seconds airborne
        self._ingested = []
        self._pc_dirty = False
        self._hm_cache = None
        self.pc_count = 0
        # --- operator tasking zones (contracts v1.2) + recon
        self.tracker, self.recon, self.tasking_path, self._tk_mtime = None, None, None, None
        self.recon_accept_s, self._recon_ready_t, self._retask_pending = recon_accept_s, None, False
        self._zone_of = {}             # pile -> zone id it is being searched for
        tk = None
        if tasking is not None and tasking != "none":
            if isinstance(tasking, Tasking):
                tk = tasking
            elif tasking == "default":
                tk = default_tasking(self.scenario)
            else:
                self.tasking_path, self._tk_mtime = str(tasking), tasking_mtime(tasking)
                tk = load_tasking(tasking, allow_empty=True)
            self.tracker = ZoneTracker(None, piles=self.scenario.piles, pile_xy=self.world.pile_xy,
                                       bounds=self.bounds, revisit_max=revisit_max)
            self.tracker.update_tasking(tk, t=0.0)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
            if frames:
                os.makedirs(os.path.join(out_dir, "frames"), exist_ok=True)
            self._write_pointcloud(self.world.sample_pointcloud(n_points))
        budget = (f"autonomous: up to {self.mission_scans} radar scans, {self.max_t:.0f} s, battery reserve "
                  f"{100 * reserve:.0f}% with swaps at the pads" if autonomous else "fixed loop")
        cap = f", pile cap {pile_budget}" if pile_budget is not None else ""
        self._say(f"Grid mission start (seed {seed}, {radar} radar, {dispatch_policy} dispatch): 3 drones at the staging "
                  f"area, start pile {self.start_pile} (operator choice); {budget}{cap}")
        dw = 45.0 if self.delivery == "pod" else 20.0
        r2, r11 = reach_path_m(REACH_REF_RANGE_M, 0.9, dwell=dw), reach_path_m(11.0, 0.9, dwell=dw)
        self._say(f"Radar penetration model '{self.penetration}': a good scan sees a breathing chest through about "
                  f"{r2:.1f} m of rubble directly beneath it and {r11:.1f} m from an 11 m stand-off"
                  + ("; deeper than that is beyond radar reach (acoustic lead only)" if self.penetration == "literature"
                     else ". WARNING: LEGACY optimistic model (flat 0.7 dB/m), not literature-backed; "
                     "--penetration literature limits a scan to ~3 m beneath it"))
        if self.tracker is None:
            self._start_pile(self.start_pile, reason="operator-selected start pile")
        else:
            self._say(f"Operator tasking: {len(self.tracker.zone_ids())} active zone(s) "
                      f"({self.tracker.source or 'none'}, revision {self.tracker.revision})"
                      + (f" from {self.tasking_path}" if self.tasking_path else "")
                      + "; the drones search only inside tasked zones")
            self._flush_zone_events()
            if self._tasked_piles():
                self._start_pile(*self._first_zone_pile())
            else:
                self._start_recon(tk.aoi if tk is not None else None)

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

    # ------------------------------------------------------------------ tasking zones (v1.2)
    def _flush_zone_events(self):
        if self.tracker is not None:
            for _, msg in self.tracker.events:
                self._say(f"Zones: {msg}")
            self.tracker.events.clear()

    def _zone_search_piles(self, zid):
        """Piles searched for zone ``zid``: those with pieces inside it, else the nearest pile within 15 m."""
        ps = self.tracker.zone_piles(zid)
        if ps:
            return ps
        cx, cy = self.tracker.zone(zid)["centroid"]
        n, (x, y) = min(self.world.pile_xy.items(), key=lambda e: np.hypot(e[1][0] - cx, e[1][1] - cy))
        return [n] if np.hypot(x - cx, y - cy) <= 15.0 else []

    def _pile_zone(self, pile):
        """Zone to search ``pile`` for: an active (surveying/scanning) one, else the best should_visit() one."""
        if self.tracker is None:
            return None
        cands = [z for z in self.tracker.visit_candidates() if pile in self._zone_search_piles(z)]
        act = [z for z in cands if self.tracker.zone(z)["status"] in ("surveying", "scanning")]
        return (act or cands or [None])[0]

    def _tasked_piles(self):
        """Piles inside zones that should be visited (None = no tasking: every pile)."""
        if self.tracker is None:
            return None
        return list(dict.fromkeys(p for z in self.tracker.visit_candidates() for p in self._zone_search_piles(z)))

    def _reopened(self, pile):
        z = self._pile_zone(pile)
        return z is not None and self.tracker.zone(z)["reopened"]

    def _operator_tasked(self):
        return self.tracker is not None and self.tracker.source != "scenario-default"

    def _first_zone_pile(self):
        piles = self._tasked_piles()
        if self.start_pile in piles:
            return self.start_pile, f"operator-selected start pile, inside tasked zone {self._pile_zone(self.start_pile)}"
        ranked = sorted(piles, key=lambda n: -(self.piles[n]["priority"] or 0))
        n = ranked[0]
        z = self._pile_zone(n)
        pr = self.piles[n]["priority"]
        return n, (f"first tasked zone {z} ({self.tracker.zone(z)['name']})"
                   + (f", acoustic priority {pr}" if pr is not None else ", operator order"))

    def _zone_prior(self, pm):
        """Restrict the localiser to the zone: prior weight ZONE_PRIOR_FLOOR outside it."""
        if self.tracker is None or pm.zone_id is None:
            return
        m = self.tracker.mask_on(pm.zone_id, pm.grid, LOC_ZONE_MARGIN_M)
        if m.any():
            pm.loc.logb = pm.loc.logb + np.log(np.where(m, 1.0, ZONE_PRIOR_FLOOR))
            pm.loc._p = None

    def _poll_tasking(self):
        """Re-read the operator tasking file when its mtime changes; merge the revision live."""
        if self.tracker is None or not self.tasking_path:
            return
        m = tasking_mtime(self.tasking_path)
        if m == self._tk_mtime:
            return
        self._tk_mtime = m
        tk = load_tasking(self.tasking_path, allow_empty=True)
        if tk is None:
            return
        if self.tracker.update_tasking(tk, t=self.t):
            self._flush_zone_events()
            self._retask_pending = True
            if self.stage == "recon" and self._tasked_piles():
                self._end_recon(f"operator tasking revision {tk.revision}")

    # ------------------------------------------------------------------ recon (no zones tasked)
    def _start_recon(self, aoi=None):
        self.stage = "recon"
        obstacles = (self.scenario.public_dict().get("site_info") or {}).get("obstacles")   # prior footprints
        self.recon = Recon(self.grid, self.flight, bounds=self.bounds, aoi=aoi, pile_xy=self.world.pile_xy,
                           say=self._say, t0=self.t, obstacles=obstacles)

    def _recon_step(self):
        self.t += self.dt
        self.k += 1
        rc = self.recon
        rc.step(self.t, self.flight.step(self.dt, self.t))
        if rc.wants_listen:
            n0 = len(self.clips)
            self._finish_listen(drones=dict(rc.wants_listen), gate=False)
            rc.add_listen(self.clips[n0:], self.t)
        if rc.status == "proposing":
            if self._recon_ready_t is None:
                self._recon_ready_t = self.t
                if self.autonomous:
                    self._say(f"RECON: waiting for operator tasking; auto-accepting proposals scoring >= "
                              f"{RECON_ACCEPT_MIN} after {self.recon_accept_s:.0f} s (unattended demo)")
            elif self.autonomous and self.t - self._recon_ready_t >= self.recon_accept_s:
                tk = proposals_to_tasking(rc.proposals, min_score=RECON_ACCEPT_MIN)
                if not tk.zones:
                    self._say("RECON: no proposal scores high enough to search; mission ends")
                    self.stage = "return"
                    return
                self.tracker.update_tasking(tk, t=self.t)
                self._flush_zone_events()
                self._end_recon(f"no operator tasking {self.recon_accept_s:.0f} s after the proposals: auto-accepted "
                                f"{', '.join(z.id for z in tk.zones)}")

    def _end_recon(self, reason):
        self.recon.finish()
        self._say(f"RECON over: {reason}. Targeted search of the tasked zones starts")
        self._start_pile(*self._first_zone_pile())

    def _start_pile(self, name, reason):
        cx, cy = self.world.pile_xy[name]
        pw = self.world.pile_world(name)
        prev = self.visited[-1] if self.visited else None
        revisit = name in self.visited
        self.visited.append(name)
        budget = self.scan_budget
        if self.mission_scans is not None:
            budget = max(3, min(budget, self.mission_scans - self.scans_used))
        self.pm = PileMission(self.seed, self.policy, budget, verbose=self.verbose, max_t=1e9, dt=self.dt,
                              world=pw, flight=self.flight, sensors=self.sensor_factory(pw, self.seed),
                              centre=(cx, cy), survey_radius=SURVEY_RADIUS, scan_offset=self.n_scans, t0=self.t,
                              log=self.log, pile=name, owner=self)
        self.pm_t0 = self.t
        self.pm.vitals_look_s = VITALS_LOOK_S
        if self.platform_spec is not None:
            self.pm.landing_kw["limits"] = self.platform_spec.landing_limits()
        if self.delivery == "pod":
            pod = getattr(self.platform_spec, "pod", None)
            lead = self._lead_xy(name)
            self.pm.site_detector = lambda g, c: detect_pod_sites(
                g, c, lead_xy=lead, max_slope=getattr(pod, "max_slope_deg", 30.0), winch_m=getattr(pod, "winch_m", 20.0),
                hover_clear=getattr(pod, "hover_clear_m", 5.0))
        self.pm.prior_obs = [o for o in self.done_scans if o.get("pile_id") == name] if revisit else []
        zid = self._pile_zone(name)
        if zid is not None:
            self._zone_of[name] = self.pm.zone_id = zid
            self.pm.zone_mask = self.tracker.mask_on(zid, self.pm.grid, LAND_ZONE_MARGIN_M)
            self._zone_prior(self.pm)
            if self.tracker.zone(zid)["status"] not in ("surveying", "scanning"):
                self.tracker.start_search(zid, self.t)
            reason = f"{reason} [zone {zid}]"
            self._flush_zone_events()
        self._preload(self.pm)
        if self.slam:
            self._slam_ingest(name)
        p = self.piles[name]
        p["status"] = "surveying"
        p["why"] = f"Fleet dispatched here ({reason}); LiDAR survey and landing search under way."
        self.stage = "pile"          # PileMission.__init__ already commanded the takeoff
        verb = "Revisit" if revisit else "Dispatch" if prev else "Deploy"
        self._say(f"{verb} d1-d3 -> pile {name} at ({cx:.1f}, {cy:.1f}): {reason}"
                  + (f" [{len(self.pm.prior_obs)} earlier scans kept in the belief; up to {budget} new scans]" if revisit else ""))

    def _placement(self, pod):
        """1-based index of this pod's placement among the active pile's scans (contract: placement / of)."""
        if not self.pm:
            return None
        ids = [s["id"] for s in self.pm.sites if s.get("status") == "used"]
        return ids.index(pod["site"]) + 1 if pod.get("site") in ids else None

    def _lead_xy(self, name):
        """Lead posterior mode for pile ``name``: on a revisit, the earlier radar mode there; else the least-squares
        crossing of the human-like bearing cones assigned to it if it falls on the pile (within 4 m of its
        centre); else the pile centre."""
        cx, cy = self.world.pile_xy[name]
        prev = [c for c in self.done_cands if c.get("pile_id") == name and c.get("kind", "stationary") == "stationary"]
        if prev:
            return (float(prev[0]["xy"][0]), float(prev[0]["xy"][1]))
        obs = [o for o in self.acoustic_obs if o.get("pile_id") == name and o.get("bearing_deg") is not None]
        if len(obs) >= 2:
            A, b = np.zeros((2, 2)), np.zeros(2)
            for o in obs:
                th = np.radians(o["bearing_deg"])
                n = np.array([-np.sin(th), np.cos(th)])                 # normal to the bearing line
                w = float(o.get("confidence") or 0.5)
                A += w * np.outer(n, n)
                b += w * n * (n @ np.asarray(o["origin"], float))
            if np.linalg.cond(A) < 1e3:
                x = np.linalg.solve(A, b)
                if np.hypot(x[0] - cx, x[1] - cy) <= 4.0:
                    return (float(x[0]), float(x[1]))
        return (cx, cy)

    def _preload(self, pm):
        """Revisit: fold the pile's earlier radar observations into the (new) localiser."""
        for o in getattr(pm, "prior_obs", []):
            pm.loc.update_radar({k: v for k, v in o.items() if k not in ("pile_id", "used_weight")})

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
        if self.tracker is not None:
            self.tracker.mark_lidar_disc(self.world.pile_xy[pm.pile], LIDAR_SURVEY_M)
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
        self._zone_prior(pm)
        self._preload(pm)
        pm._say(f"Heightmap window rebuilt from {int(sel.sum())} LiDAR points")

    # ------------------------------------------------------------------ main loop
    def step(self):
        self._poll_tasking()
        if self.comms is not None:
            self._comms_step()
        if self.speaker is not None and self.out_dir:     # operator broadcasts approved on the dashboard (v1.3)
            n = self.speaker.ingest_operator(os.path.join(self.out_dir, "broadcast_queue.json"), self.t, self._drone_xy())
            if n:
                self._say(f"Operator broadcast: {n} approved message(s) queued on the loudspeaker")
        if self.stage == "recon":
            self._recon_step()
        elif self.stage == "pile":
            self.pm.step()
            self.t, self.k = self.pm.t, self.k + 1
            self._track_pile()
            if self.pm.stage == "done" or self.t - self.pm_t0 > PILE_MAX_T:
                self._finish_pile()
        else:
            self.t += self.dt
            self.k += 1
            self.flight.step(self.dt, self.t)
            docked = all(p["state"] == "docked" for p in self.flight.poses())
            if self.stage == "listen" and self.listen_end is None and all(
                    p["state"] in ("landed", "docked") and not self.flight.drone_state(p["id"])[2] for p in self.flight.poses()):
                self.listen_end = self.t + self.listen_s
                self._say(f"All drones down, rotors off: listening for {self.listen_s:.0f} s")
                if self.speaker is not None and self._pending_callout:
                    self.listen_end += self._begin_callout(*self._pending_callout)
                self._pending_callout = None
            if self.stage == "listen" and self.listen_end is not None and self.t >= self.listen_end:
                self._finish_listen()
            elif self.stage == "swap":
                if self.swap["ends"] is None and docked:
                    self.swap["ends"] = self.t + self.swap_s
                    self._say(f"Fleet docked; swapping batteries ({self.swap_s:.0f} s)")
                elif self.swap["ends"] is not None and self.t >= self.swap["ends"]:
                    for d in self.flight.drones:
                        self._swap_battery(d)
                    self.n_swaps += 1
                    nxt, reason = self.swap["to_pile"], self.swap["reason"]
                    self.swap = None
                    self._say(f"Batteries swapped (swap {self.n_swaps}); redeploying to {nxt}")
                    self._start_pile(nxt, reason)
            elif self.stage == "return" and docked and self._retask_pending and self.visited \
                    and self._targets() and self.t < self.max_t:
                self._retask_pending = False         # new tasking while returning: one more dispatch gate
                self._gate(None)
            elif self.stage == "return" and docked:
                self.stage = "done"
                self._say("All drones docked at the staging area. Mission complete.")
                self._say(self.final_summary())
        self._fly_s += self.dt * sum(1 for p in self.flight.poses() if p["state"] in ("flying", "descending", "hovering"))
        self._update_phase()
        if self.t >= self.max_t and self.stage != "done":
            self._say(f"Mission time limit ({self.max_t:.0f} s) reached; stopping")
            self.stage = "done"
            self._say(self.final_summary())
        self._write_state()
        if self.realtime:
            time.sleep(self.dt / self.speed)

    def run(self):
        self._write_state()
        while not self.done:
            self.step()
        return self

    def _swap_battery(self, did):
        if hasattr(self.flight, "swap_battery"):
            self.flight.swap_battery(did)
        elif hasattr(self.flight, "drones") and hasattr(self.flight.drones[did], "battery"):
            self.flight.drones[did].battery = 1.0

    def _update_phase(self):
        if self.stage == "pile":
            ph = self.pm.phase
            if len(self.visited) > 1 and self.pm.stage in ("takeoff", "survey"):
                cx, cy = self.world.pile_xy[self.pm.pile]
                far = np.mean([np.hypot(x - cx, y - cy) for x, y in self._drone_xy().values()]) > SURVEY_RADIUS + 3
                if self.pm.stage == "takeoff" or far:
                    ph = "dispatch"
            self.phase = ph
        elif self.stage == "recon":
            self.phase = self.recon.phase
        elif self.stage == "listen":
            self.phase = "acoustic"
        elif self.stage == "return":
            self.phase = "return"
        elif self.stage == "swap":
            self.phase = "return" if self.swap["ends"] is None else "battery_swap"
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
        zid = self._zone_of.get(self.pm.pile)
        if zid is not None and zid in self.tracker.zones:
            for o in self.pm.scans:
                self.tracker.add_scan(o)
            if self.pm.scans_cmd or self.pm.phase in ("landing", "radar", "relocate", "rescan"):
                self.tracker.mark_scanning(zid, self.t)
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
        self.scans_used += pm.scans_cmd
        p["scans"] = p.get("scans", 0) + pm.scans_cmd
        self.done_sites.extend(dict(s, id=f"{name}-{s['id']}", pile_id=name) for s in pm.sites if s["status"] == "used")
        cands = self._live_cands(pm)
        firm = [c for c in cands if c["status"] == "candidate"]
        keep = firm or ([cands[0]] if cands and cands[0]["p_mass"] >= 0.3 else [])
        for c in keep:
            c["t_found"] = round(self.t, 1)
            c["visit"] = self.visited.count(name)
        self.done_cands = [c for c in self.done_cands if c["pile_id"] != name] + keep   # a revisit supersedes
        p["radar_candidate_ids"] = [c["id"] for c in keep]
        cov = self._radar_coverage(pm)
        p.update(cov)
        reach_txt = (f"Radar reached {100 * cov['radar_coverage']:.0f}% of the pile volume "
                     f"({100 * cov['radar_coverage_shallow']:.0f}% of its top 2 m) from the "
                     f"{'pod placements' if self.delivery == 'pod' else 'landing sites'} used; "
                     + (f"per-scan reach ≈{cov['radar_reach_m']:.1f} m of rubble directly beneath" if cov.get('radar_reach_m') is not None
                      else "no scan reached the rubble"))
        res = pm.result
        for c in firm:
            self._attach_vitals(pm, c)
        if firm:
            c = firm[0]
            p["status"] = "ping"
            p["why"] = (f"Radar ping {c['id']} at ({c['xy'][0]:.1f}, {c['xy'][1]:.1f}), {self._depth_words(c)}, "
                        f"{c['support']} strong agreeing ranges ({len(c['evidence'])} supporting scans). Candidate "
                        "search region, not a confirmed person.")
            self._say(f"PING {c['id']} at pile {name}: ({c['xy'][0]:.1f}, {c['xy'][1]:.1f}), {self._depth_words(c)}, "
                      f"belief share {c['p_mass']:.2f} -> sent to the operator dashboard")
        else:
            lead = (p["priority"] or 0) >= self.priority_min
            beyond = cov["radar_coverage_shallow"] < REVISIT_MIN_COVERAGE
            p["status"] = "lead" if lead else "no_signal_not_cleared"
            p["why"] = (f"Radar inconclusive after {p['scans']} scans ({res.get('reason', 'no data')}). {reach_txt}"
                        + (": much of the pile is beyond radar reach" if beyond else "")
                        + ("; acoustic lead stands" + (" (acoustic lead only)" if beyond else "") if lead else "")
                        + ". No detection is not proof of absence: not cleared.")
            self._say(f"No firm radar ping at {name} ({res.get('reason', 'no data')}). Evidence: {pm.explain_evidence()}. "
                      f"{reach_txt}. Pile stays not cleared"
                      + (("; beyond radar reach from these sites -> acoustic lead only, for ground teams" if beyond
                          else "; its acoustic lead stays open for a revisit") if lead else ""))
        zid = self._zone_of.get(name)
        if zid is not None and zid in self.tracker.zones:
            for o in pm.scans:
                self.tracker.add_scan(o)
            for c in firm:
                self.tracker.add_ping(zid, c["id"])
            zp = self._zone_search_piles(zid)
            if all(q in self.visited for q in zp):
                self.tracker.finish_search(zid, self.t, pings=list(self.tracker.zone(zid)["pings"]),
                                           lead=any(self.piles[q]["status"] == "lead" for q in zp))
            self._flush_zone_events()
        if self.listen_s > 0:
            self.stage = "listen"
            if self.delivery == "pod" and self._land_for_listen(pm):
                self.listen_end = None          # starts once all drones are down with rotors off
                self._pending_callout = (name, firm, pm)
                self._say(f"Radar phase at {name} over; drones land off the pile (rotors off) to listen to the other "
                          f"piles for {self.listen_s:.0f} s")
            else:
                self.listen_end = self.t + self.listen_s
                self._say(f"Radar phase at {name} over; landed drones listen to the other piles for {self.listen_s:.0f} s "
                          "and stream clips to the acoustic agent")
                if self.speaker is not None:
                    self.listen_end += self._begin_callout(name, firm, pm)
        else:
            self._gate(None)

    def _attach_vitals(self, pm, c):
        """Ping vitals from the radar's own output: the vitals-look scan (else the best supporting scan), the
        range candidate nearest the ping's predicted range, depth term rescaled for the rubble path. Plus the
        dig-order score (sim/vitals.dig_score). Nothing here is truth; absent vitals stay absent."""
        try:
            from .vitals import dig_score, with_rubble_path
        except Exception:                            # noqa: BLE001  (vitals module in flux)
            return
        pt = np.array([c["xy"][0], c["xy"][1], c["z_est"]], float)
        best = None
        for o in pm.scans:
            if o.get("doppler_moving") or not o.get("vitals") or o["scan_id"] not in c.get("evidence", []) + \
                    [s["scan_id"] for s in pm.scans if s.get("vitals_look")]:
                continue
            ps = np.asarray(o["sensor_pose"][:3], float)
            r = float(np.linalg.norm(pt - ps))
            k = int(np.argmin([abs(rc - r) for rc in o["range_candidates"]]))
            v = o["vitals"][k] if k < len(o["vitals"]) else None
            if not v or abs(o["range_candidates"][k] - r) > 1.5:
                continue
            key = (bool(o.get("vitals_look")), v.get("sqi", 0.0))
            if best is None or key > best[0]:
                best = (key, o, v, r, ps)
        depth = c.get("depth_m")
        if best is None:
            c["dig_score"] = dig_score(None, c["p_mass"], depth)
            return
        _, o, v, r, ps = best
        v = dict(with_rubble_path(v, r, float(rubble_path(pm.grid, ps, pt))), scan_id=o["scan_id"])
        c["vitals"] = v
        c["dig_score"] = dig_score(v, c["p_mass"], depth)
        self._say(f"Vitals at {c['id']} from {o['scan_id']}{' (60 s look)' if o.get('vitals_look') else ''}: "
                  f"pattern {v.get('pattern')}, rate {v.get('rate_bpm')}/min, hint {v.get('triage_hint')} "
                  f"(confidence {v.get('pattern_conf')}); dig score {c['dig_score']}. A triage aid for rescuers to check")

    def _land_for_listen(self, pm):
        """Pod delivery: the drones hover while the pods scan, but listening needs rotors off, so each drone lands
        on the nearest free flat patch around the pile (landing-ring detector, platform limits)."""
        kw = {"search_radius": 14.0}
        if self.platform_spec is not None:
            kw["limits"] = self.platform_spec.landing_limits()
        sites = [s for s in detect_landing_sites(pm.grid, centre=self.world.pile_xy[pm.pile], **kw)
                 if s["status"] == "candidate" and not s.get("perch")
                 and abs(float(self.grid.height_at(*s["xy"])) - s["z"]) < 0.3]   # the window flattens other piles
        poses = {p["id"]: p["pose"] for p in self.flight.poses()}
        if len(sites) < len(poses):
            self._say("WARNING: not enough flat patches to land for the listen; listening while hovering (rotor noise)")
            return False
        taken = set()
        for did, pose in poses.items():
            s = min((s for s in sites if s["id"] not in taken),
                    key=lambda s: np.hypot(s["xy"][0] - pose[0], s["xy"][1] - pose[1]))
            taken.add(s["id"])
            z = max(float(s["z"]), float(self.grid.height_at(*s["xy"])))     # skids on the highest point of the patch
            self.flight.command({"drone_id": did, "type": "goto_land_scan", "destination": [s["xy"][0], s["xy"][1], z],
                                 "landing_site_id": f"listen-{s['id']}", "scan_duration": 0.5})
        return True

    def _radar_coverage(self, pm):
        """Share of the pile's rubble volume within radar reach of the landing sites actually scanned from.

        Volume = 1 m cells within 9 m of the pile centre and > 0.4 m above local ground, sampled every 0.5 m
        from 0.25 m below the surface down to ground. A point is in reach if some scan's sensor sees it through
        no more rubble (heightmap path) than ``reach_path_m(range, that scan's quality)``. LiDAR heightmap +
        scan metadata only; no truth.
        """
        g = pm.grid
        cx, cy = self.world.pile_xy[pm.pile]
        X, Y = g.mesh()
        Xs, Ys, Zs = X[::2, ::2].ravel(), Y[::2, ::2].ravel(), g.z[::2, ::2].ravel()
        dc = np.hypot(Xs - cx, Ys - cy)
        ring = (np.hypot(X - cx, Y - cy) > 11.0)
        ground = float(np.median(g.z[ring])) if ring.any() else float(np.min(g.z))
        sel = (dc <= 9.0) & (Zs - ground > 0.4)
        pts, depth = [], []
        for x, y, z in zip(Xs[sel], Ys[sel], Zs[sel]):
            for d in np.arange(0.25, z - ground, 0.5):
                pts.append((x, y, z - d))
                depth.append(d)
        scans = [o for o in pm.scans if not o.get("doppler_moving")]
        if not pts:
            return {"radar_coverage": 1.0, "radar_coverage_shallow": 1.0, "radar_reach_m": None}
        pts, depth = np.array(pts), np.array(depth)
        hit = np.zeros(len(pts), bool)
        for o in scans:
            ps = np.asarray(o["sensor_pose"][:3], float)
            r = np.linalg.norm(pts - ps[None, :], axis=1)
            hit |= rubble_path(g, ps, pts) <= reach_path_m(r, o["quality"], dwell=o.get("dwell_s", 20.0))
        sh = depth <= 2.0
        reach = [o.get("max_reliable_depth_m") for o in scans if o.get("max_reliable_depth_m") is not None]
        if not reach and scans:
            reach = [reach_path_m(REACH_REF_RANGE_M, o["quality"], dwell=o.get("dwell_s", 20.0)) for o in scans]
        return {"radar_coverage": round(float(hit.mean()), 3),
                "radar_coverage_shallow": round(float(hit[sh].mean()) if sh.any() else 1.0, 3),
                "radar_reach_m": round(float(np.median(reach)), 2) if reach else None}

    @staticmethod
    def _depth_words(c):
        if c.get("depth_source") == "radar_estimate":
            return f"≈{c['depth_m']:.1f} ± {c['depth_sigma']:.1f} m below the surface (radar estimate)"
        return f"depth not measured (assumed {c.get('depth_m') or 1.5:.1f} m; the ranges cannot separate depth from position)"

    # ------------------------------------------------------------------ acoustic + dispatch
    def _finish_listen(self, drones=None, gate=True):
        self.n_listen += 1
        drones = drones or self._drone_xy()
        exclude = list(dict.fromkeys(self.visited))
        if self.acoustic_fn is not None:
            res, src = self.acoustic_fn(self.scenario, drones, exclude, self.seed + 101 * self.n_listen), "custom"
        else:
            res, src = acoustic_listen(self.scenario, drones, exclude, self.seed + 101 * self.n_listen,
                                       mode=self.acoustic_mode, log=self._say)
        by = (res.get("agent") or {}).get("mode")
        if by == "stub":
            src = STUB_SOURCE
        self.acoustic_source, self.last_acoustic = src, res
        if src.startswith("stub"):
            self._say("WARNING: acoustic clips are SYNTHETIC, generated from the scenario's hidden truth "
                      "(stub-from-truth), not heard by the acoustic pipeline")
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
                    "doa_agree": c.get("doa_agree"), "vad_speech_ratio": c.get("vad_speech_ratio"),
                    "association": c.get("association"), "association_conf": c.get("association_conf"),
                    "ambiguous_with": list(c.get("ambiguous_with") or []),
                    "bearings": self._clip_bearings(c, drones), "notes": list(c.get("notes") or [])}
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
        if self.tracker is not None:
            self.tracker.mark_listened(drones)
            n_sugg = len(self.tracker.suggestions)
            for clip in self.clips[-len(res["clips"]):] if res["clips"] else []:
                self.tracker.add_clip(clip)
            for s in ([] if self.stage == "recon" else self.tracker.suggest_zone([])[n_sugg:]):
                self._say(f"Zone suggestion {s['id']} at ({s['centre'][0]:.0f}, {s['centre'][1]:.0f}), r {s['radius_m']:.0f} m: "
                          f"{s['why']} -> for the operator to accept")
            self._flush_zone_events()
        for ap in res["piles"]:
            p = self.piles.get(ap["pile_id"])
            if p is None or ap["pile_id"] in self.visited:
                continue
            new = int(ap["priority"])
            if ap["status"] == "no_signal_not_cleared" and (
                    (p["priority"] or 0) > new or p["status"] in ("lead", "dismissed_nonhuman")):
                continue                                   # silence from here does not erase what was heard before
            st = ap["status"] if ap["status"] in ("lead", "dismissed_nonhuman", "no_signal_not_cleared") else "lead"
            why = ap["why"]
            if st == "dismissed_nonhuman":         # one non-human listen is not enough to dismiss a pile (R2-006)
                p["nonhuman_listens"] = p.get("nonhuman_listens", 0) + 1
                if p["nonhuman_listens"] < 2:
                    st = "no_signal_not_cleared"
                    why = f"Heard a non-human-like sound once (needs 2 listens to dismiss). {why}"
            p.update(priority=new, status=st, why=why)
        self._apply_callout()
        if gate:
            self._gate(res)

    # ------------------------------------------------------------------ rescue hotspot / comms (v1.4)
    COMMS_PRIORITY = {"trapped": 85, "unknown": 50}     # a located connected phone is a lead, never a confirmation

    def _comms_step(self):
        evs = self.comms.step(self.t, self.flight.poses())
        if self.out_dir and self.k % 5 == 0:             # live captive-portal submissions + operator replies
            evs += self.comms.ingest_portal(os.path.join(self.out_dir, "portal.json"), self.t)
            self.comms.ingest_replies(os.path.join(self.out_dir, "comms_replies.json"), self.t)
        for ev in evs:
            dev, typ = ev.get("device"), ev["type"]
            if typ == "joined" and dev not in self._comms_seen:
                self._comms_seen.add(dev)
                self._say(f"Phone {dev} joined the rescue Wi-Fi via {ev['drone_id']} (consent: it opened the portal)")
            elif typ == "status":
                self._say(f"Phone {dev}{' (live portal)' if ev.get('live') else ''}: '{ev['status']}'"
                          + (f", battery {ev['battery_pct']}%" if ev.get("battery_pct") is not None else ""))
            elif typ == "message":
                tr = f" -> \"{ev['translation']}\" [{ev['by']}]" if ev.get("translation") and ev["lang"] != "en" else ""
                self._say(f"Phone {dev} message ({ev['lang']}): \"{ev['text']}\"{tr}")
            elif typ == "located":
                self._say(f"Phone {dev}: RSSI fit {ev['est_xy']} +-{ev['sigma_m']:.1f} m"
                          + (f" in {ev['pile']}" if ev.get("pile") else "") + " (candidate region from multi-drone RSSI)")
            elif typ == "evidence":
                self._comms_evidence(ev)
            elif typ in ("link_lost", "link_restored"):
                self._say(f"{ev['drone_id']} mesh link {'lost: storing data on board' if typ == 'link_lost' else 'restored'}"
                          f" ({ev['queued_kb']:.0f} kB queued)")

    def _comms_evidence(self, ev):
        """A connected phone located inside a pile (sigma <= 6 m): new evidence. Raises the pile to a lead (or
        re-opens it for a revisit if it was already searched); never changes a ping, never 'confirmed'."""
        p = self.piles.get(ev["pile"])
        if p is None:
            return
        pr = self.COMMS_PRIORITY.get(ev["status"], 50)
        why = (f"Connected phone {ev['device']} ({ev['status']}) located at {ev['est_xy']} +-{ev['sigma_m']:.1f} m "
               "by multi-drone Wi-Fi RSSI: candidate region, not a confirmed person.")
        searched = ev["pile"] in self.visited
        self._say(f"New evidence for {ev['pile']}: {why}" + (" Pile already searched: re-opened for a revisit."
                                                             if searched and p["status"] != "ping" else ""))
        if p["status"] != "ping" and (p["priority"] or 0) < pr:
            p.update(priority=pr, status="lead", why=f"{why} {p['why']}".strip())
        p["comms_evidence"] = {"device": ev["device"], "status": ev["status"], "t": ev["t"],
                               "est_xy": ev["est_xy"], "sigma_m": ev["sigma_m"]}

    # ------------------------------------------------------------------ loudspeaker (v1.3)
    def _begin_callout(self, name, firm, pm):
        """Plan the call-out after the passive listen. Returns the extra landed seconds: message + listen window
        (rotors off) + the reassurance / guidance broadcasts that follow (played while still landed nearby)."""
        t_call = self.t + self.listen_s
        if firm:
            target = {"type": "ping", "id": firm[0]["id"], "xy": firm[0]["xy"]}
        else:
            target = {"type": "sector", "id": name, "xy": self.world.pile_xy[name]}
        pings = [{"id": c["id"], "xy": c["xy"], "pile_id": c["pile_id"]} for c in self.done_cands
                 if c["status"] == "candidate"]
        moving = [{"id": f"{name}-{c['id']}", "xy": c["xy"]} for c in pm.cands
                  if c.get("kind") == "moving" and c.get("status") == "candidate"]
        extra, rec = self.speaker.begin_listen(self.scenario, t_call, self._drone_xy(), target, pings, moving,
                                               callout_fn=self.callout_fn)
        self._say(f"Call-out {rec['id']} planned: after the passive listen {rec['speaker_drone']} plays the rescue "
                  f"call-out ({', '.join(rec['langs'])}) towards {target['id']}, then all drones listen "
                  f"{rec['listen_ends'] - rec['steps'][-1]['t0']:.0f} s for knocks or voices "
                  f"(+{extra:.0f} s landed incl. reassurance / guidance)")
        return extra

    def _apply_callout(self):
        """Fold the last call-out into pile / ping state: responsiveness, priority for responding piles,
        call-and-response boost of this listen's clips, and log lines. Measurements only (no truth)."""
        from .acoustic_grid import clip_priority
        from .callout import association_boost, fuse_triage, response_priority
        if self.speaker is None or not self.speaker.callouts or self.speaker.callouts[-1].get("applied"):
            return
        rec = self.speaker.callouts[-1]
        rec["applied"] = True
        quiet = []
        for sid, r in rec["sectors"].items():
            p = self.piles.get(sid)
            if p is None:
                continue
            if r["status"] == "no_response":
                quiet.append(sid)
                continue
            self._say(f"Call-out {rec['id']} (listen ended t {rec['listen_ends']:.0f} s) [{sid}]: {r['text']}")
            if r["status"] != "responded":
                continue
            for c in self.clips:
                if c["pile_id"] == sid and c["listen"] == self.n_listen and c["scores"].get("reliability") is not None:
                    rel, note = association_boost(c["scores"]["reliability"], r)
                    c["scores"]["reliability"] = rel
                    c["priority"] = clip_priority({k: c["scores"][k] or 0.0 for k in ("human", "urgency", "reliability")})
                    c.setdefault("notes", []).append(note)
            pr = response_priority(r)
            if p["status"] != "ping" and pr is not None and pr > (p["priority"] or 0):
                p.update(priority=pr, status="lead",
                         why=f"{r['text']} {p['why']}" if p["why"] else r["text"])
        for sid, agg in self.speaker.state(self.t)["responsiveness"].items():   # strongest evidence + history
            if sid in self.piles:
                self.piles[sid]["responsiveness"] = agg
        if quiet:
            self._say(f"Call-out {rec['id']}: no response from {', '.join(quiet)} (does not rule out an unconscious "
                      "person; not cleared)")
        for c in self.done_cands:
            pr = rec["pings"].get(c["id"])
            old = c.get("responsiveness")
            rank = {"responded": 3, "possible_response": 2, "nonhuman_only": 1, "no_response": 0}
            if pr and (old is None or rank[pr["status"]] >= rank[old["status"]]):    # strongest evidence wins
                c["responsiveness"] = {k: pr[k] for k in ("status", "kind", "avpu", "confidence", "text", "callout_t")}
                if c.get("vitals"):
                    c["triage_fused"] = fuse_triage(c["vitals"], pr)
        if rec["reassured"]:
            self._say(f"Reassurance played from t {rec['listen_ends']:.0f} s ({', '.join(rec['langs'])}) towards {', '.join(rec['reassured'])}: "
                      "\"Help is coming. Stay still, save your energy, tap if you can hear me.\"")
        for g in rec["guided"]:
            self._say(f"Guidance played to {g['track']} at ({g['xy'][0]:.0f}, {g['xy'][1]:.0f}) ({g['source']}): "
                      f"\"{g['text_en']}\" Route {g['route']['length_m']:.0f} m clear of rubble and tasked zones")


    @staticmethod
    def _clip_bearings(c, drones):
        """Per-drone bearings of a clip: [{drone_id, origin, bearing_deg, width_deg, snr_db}] (as reported)."""
        src = c.get("bearings") or c.get("per_drone")
        out = []
        if isinstance(src, dict):
            src = [dict(v, drone_id=k) for k, v in src.items()]
        for b in src or [{"drone_id": c.get("drone_id"), "bearing_deg": c.get("bearing_deg"),
                          "width_deg": c.get("width_deg"), "snr_db": c.get("snr_db")}]:
            o = drones.get(b.get("drone_id"), (None, None))
            out.append({"drone_id": b.get("drone_id"), "origin": None if o[0] is None else [round(o[0], 2), round(o[1], 2)],
                        "bearing_deg": b.get("bearing_deg"), "width_deg": b.get("width_deg"), "snr_db": b.get("snr_db")})
        return out

    def _suggest_zone(self):
        """With an LLM decider and zone proposals waiting, ask it which proposal the operator might task next.
        Recorded as a suggestion only (zone_suggestions[].why, kind decider_suggestion); never tasks anything."""
        if self.decider is None or self.recon is None:
            return
        props = self.recon.to_state().get("zone_proposals") or []
        if not props:
            return
        try:
            from agents.gemini_decider import suggest_zone
            out = suggest_zone(self.build_state())
        except Exception as e:                       # noqa: BLE001
            self._say(f"zone suggestion unavailable ({type(e).__name__})")
            return
        if out and not any(z.get("proposal") == out.get("proposal") for z in self._zone_ai):
            self._zone_ai.append({"proposal": out.get("proposal"), "why": out.get("text"), "by": out.get("by"),
                                  "t": round(self.t, 1), "kind": "decider_suggestion"})
            self._say(f"Decider ({out.get('by')}): {out.get('text')}")

    def _gate(self, res):
        self._suggest_zone()
        """Dispatch gate after a pile (and its listen): re-rank, then dispatch / revisit / swap / stop."""
        tasked = self._tasked_piles()
        ranked = sorted((p for p in self.piles.values() if p["id"] not in self.visited and p["priority"] is not None
                         and (tasked is None or p["id"] in tasked)), key=lambda p: (-p["priority"], p["id"]))
        if ranked:
            self._say("Pile priority: " + ", ".join(f"{p['id']} {p['priority']}" for p in ranked[:6]))
        here = self.visited[-1]
        opt, reason = self._decide_dispatch(ranked, res or {})
        nxt = opt["pile_id"] if opt["action"] != "stop" else None
        self.dispatch.append({"t": round(self.t, 1), "from_pile": here, "to_pile": nxt, "reason": reason})
        if nxt is None:
            self._end_loop(reason)
        elif "battery_swap" in opt["flags"]:
            low = ", ".join(f"{p['id']} {100 * p['battery']:.0f}%" for p in self.flight.poses())
            self._say(f"Battery check before {nxt}: {low}; the next pile needs about {100 * self.PILE_BATTERY:.0f}% plus "
                      f"travel and every drone must keep a {100 * self.reserve:.0f}% reserve -> the fleet returns to the "
                      f"pads for a battery swap, then flies to {nxt}")
            self.swap = {"to_pile": nxt, "reason": reason, "ends": None, "t": round(self.t, 1)}
            for d in self.flight.drones:
                self.flight.command({"drone_id": d, "type": "return"})
            self.pm = None
            self.stage = "swap"
        else:
            self._start_pile(nxt, reason)

    def _end_loop(self, reason):
        left = [f"{p['id']} {p['priority']}" for p in self.piles.values() if p["status"] == "lead"]
        self._say(f"Search loop ends: {reason}. Open leads for ground teams: {', '.join(left) or 'none'}. "
                  "Drones return to the staging area")
        for d in self.flight.drones:
            self.flight.command({"drone_id": d, "type": "return"})
        self.pm = None
        self.stage = "return"

    # ------------------------------------------------------------------ dispatch gate (optional decider)
    @property
    def FLY_RATE(self):            # battery fraction per second of flight (from the flight backend)
        return float(getattr(self.flight, "fly_rate", 0.0016))

    @property
    def PILE_BATTERY(self):        # rough battery cost of one pile (survey + landings + scans + listen)
        return 0.16 * self.FLY_RATE / 0.0016

    def _leg_s(self, a, b):
        """Rough flight time (s) between two xy points: cruise at 4 m/s plus climb and descent."""
        return float(np.hypot(b[0] - a[0], b[1] - a[1])) / 4.0 + 12.0

    def _targets(self):
        """(pile_id, action, score) in this policy's preference order (no truth, no budget checks)."""
        tasked = self._tasked_piles()
        unvisited = [n for n in self.piles if n not in self.visited and (tasked is None or n in tasked)]
        if self.dispatch_policy == "nearest":
            c = np.mean(list(self._drone_xy().values()), axis=0)
            order = sorted(unvisited, key=lambda n: (self._leg_s(c, self.world.pile_xy[n]), n))
            return [(n, "dispatch", None) for n in order]
        if self.dispatch_policy == "raster":
            return [(n, "dispatch", None) for n in RASTER if n in unvisited]
        out = [(p["id"], "dispatch", p["priority"]) for p in sorted(
            (self.piles[n] for n in unvisited), key=lambda p: (-(p["priority"] if p["priority"] is not None else -1), p["id"]))]
        if self.autonomous:
            for name in dict.fromkeys(self.visited):
                p = self.piles[name]
                if tasked is not None and name not in tasked:
                    continue                     # finished zone: no revisit without new evidence / re-task
                if self.tracker is not None and self._reopened(name):
                    out.append((name, "rescan_current" if name == self.visited[-1] else "revisit",
                                max(p["priority"] or 0, self.priority_min)))
                elif p["status"] != "ping" and (p["priority"] or 0) >= self.priority_min \
                        and self.visited.count(name) < self.revisit_max \
                        and p.get("radar_coverage_shallow", 1.0) >= REVISIT_MIN_COVERAGE:
                    out.append((name, "rescan_current" if name == self.visited[-1] else "revisit",
                                round(p["priority"] * REVISIT_DISCOUNT, 1)))
        return sorted(out, key=lambda o: -(o[2] if o[2] is not None else -1))

    def _options(self, ranked, res):
        """Scored options at the dispatch gate. Returns (options, argmax_option_id, argmax_reason)."""
        here = self.visited[-1]
        poses = {p["id"]: p for p in self.flight.poses()}
        bat_min = min(p["battery"] for p in poses.values())
        clips_by = {}
        for c in self.clips:
            clips_by.setdefault(c["pile_id"], []).append(c)
        visits_left = None if self.pile_budget is None else self.pile_budget - len(self.visited)
        scans_left = None if self.mission_scans is None else self.mission_scans - self.scans_used
        why_no_budget = (f"pile budget ({self.pile_budget} visits) used" if visits_left is not None and visits_left <= 0
                         else f"mission scan budget used ({self.scans_used}/{self.mission_scans}; a fix needs 3 scans)"
                         if scans_left is not None and scans_left < 3 else None)
        opts = []

        def travel(pid):
            x, y = self.world.pile_xy[pid]
            ts = [self.flight.eta(d, [x, y, float(self.grid.height_at(x, y)) + 9.0]) for d in poses]
            return None if any(t is None for t in ts) else round(max(ts), 1)

        if why_no_budget is None:
            targets = self._targets() if self.autonomous or self.dispatch_policy != "acoustic" else \
                [(p["id"], "dispatch", p["priority"]) for p in ranked]
            for pid, action, score in targets[:8]:
                p = self.piles[pid]
                ts = 0.0 if action == "rescan_current" else travel(pid)
                bat_after = None if ts is None else round(bat_min - self.FLY_RATE * ts - self.PILE_BATTERY, 2)
                flags = []
                if self.autonomous and (bat_after is None or bat_after < self.reserve):
                    pads = list(getattr(self.flight, "starts", {}).values()) or [self.base_xy]
                    back = max(self._leg_s(xy, pads[0]) for xy in self._drone_xy().values())
                    out = self._leg_s(pads[0], self.world.pile_xy[pid])
                    ts, bat_after = round(back + self.swap_s + out, 1), round(1.0 - self.FLY_RATE * out - self.PILE_BATTERY, 2)
                    flags.append("battery_swap")
                cs = sorted(clips_by.get(pid, []), key=lambda c: -(c["priority"] or 0))[:2]
                ev = "; ".join(f"{c['class']}" + (f" \"{c['transcript'][:60]}\"" if c["transcript"] else "")
                               + f" (human {c['scores']['human']}, priority {c['priority']}, {c['by']})" for c in cs) \
                    or "no clip assigned"
                if action != "dispatch":
                    ev = f"radar at {pid} after {p.get('scans', 0)} scans: {p['why'][:160]} | acoustic: {ev}"
                prio = p["priority"]
                flags += [f for f, on in (
                    ("below_threshold", self.dispatch_policy == "acoustic" and (prio or 0) < self.priority_min),
                    ("nonhuman", p["status"] == "dismissed_nonhuman"),
                    ("silent", p["status"] == "no_signal_not_cleared" or prio is None),
                    ("no_firm_ping", action != "dispatch"),
                    ("unreachable", ts is None),
                    ("tasked", self._operator_tasked() or (self.tracker is not None and self._reopened(pid)))) if on]
                placements = [s["xy"] for s in self.done_sites if s.get("pile_id") == pid and s.get("pod")] \
                    if self.delivery == "pod" else None
                opts.append({"id": f"o{len(opts) + 1}", "action": action, "pile_id": pid, "priority": prio,
                             **({"placements": placements} if placements is not None else {}),
                             "score": score, "travel_s": ts, "battery_after_min": bat_after, "evidence": ev,
                             "flags": flags})
            if not self.autonomous:
                cur = self.piles[here]
                if cur["status"] != "ping" and self.visited.count(here) < 2:
                    opts.append({"id": f"o{len(opts) + 1}", "action": "rescan_current", "pile_id": here,
                                 "priority": cur["priority"], "score": None, "travel_s": 0.0,
                                 "battery_after_min": round(bat_min - self.PILE_BATTERY, 2),
                                 "evidence": f"radar at {here}: {cur['why'][:160]}", "flags": ["no_firm_ping"]})
        opts.append({"id": f"o{len(opts) + 1}", "action": "stop", "pile_id": None, "priority": None, "score": None,
                     "travel_s": None, "battery_after_min": round(bat_min, 2), "evidence": "return to the staging area",
                     "flags": (["pile_budget_used"] + (["scan_budget_used"] if "scan budget" in why_no_budget else []))
                     if why_no_budget else []})
        stop = opts[-1]["id"]
        if why_no_budget:
            return opts, stop, why_no_budget
        ok = [o for o in opts if o["action"] != "stop" and "unreachable" not in o["flags"]
              and (self.autonomous or o["action"] == "dispatch")]
        if self.dispatch_policy == "acoustic":
            ok = [o for o in ok if (o["priority"] or 0) >= self.priority_min or "tasked" in o["flags"]]
        if not ok:
            if self.dispatch_policy == "acoustic":
                return opts, stop, (f"no pile left with an acoustic lead at or above priority {self.priority_min}"
                                    + (" (open leads already revisited)" if self.autonomous and any(
                                        self.visited.count(n) >= self.revisit_max for n in self.visited) else "")
                                    + "; remaining piles stay not cleared")
            return opts, stop, "every pile visited"
        top = ok[0]
        p = self.piles[top["pile_id"]]
        if self.dispatch_policy == "nearest":
            reason = f"nearest unvisited pile ({top['pile_id']}, {top['travel_s']:.0f} s away); systematic baseline"
        elif self.dispatch_policy == "raster":
            reason = f"next pile in the fixed raster order ({top['pile_id']}); systematic baseline"
        elif top["action"] != "dispatch" and self.tracker is not None and self._reopened(top["pile_id"]):
            z = self._pile_zone(top["pile_id"])
            reason = f"revisit {top['pile_id']}: zone {z} reopened ({self.tracker.zone(z)['retask_reason']})"
        elif top["action"] == "dispatch" and "tasked" in top["flags"] and (p["priority"] or 0) < self.priority_min:
            pr = "" if p["priority"] is None else f", priority {p['priority']}"
            reason = f"operator-tasked zone {self._pile_zone(top['pile_id'])} not searched yet (no acoustic lead{pr})"
        elif top["action"] != "dispatch":
            reason = (f"revisit {top['pile_id']}: its acoustic lead (priority {p['priority']}) is still open after "
                      f"{p.get('scans', 0)} radar scans without a firm fix, and no unvisited pile scores higher; fresh "
                      "landing sites add independent ranges to the same belief")
        else:
            d = res.get("dispatch") or {}
            reason = d["reason"] if d.get("pile_id") == top["pile_id"] and d.get("reason") else \
                f"highest acoustic priority ({p['priority']}) among open piles: {p['why']}"
        return opts, top["id"], reason

    def _decide_dispatch(self, ranked, res):
        opts, best, reason = self._options(ranked, res)
        by, rationale, latency = "argmax", reason, 0.0
        chosen = best
        if self.decider is not None and len(opts) > 1:
            ctx = {"t": round(self.t, 1), "current_pile": self.visited[-1], "visited": list(self.visited),
                   "pile_budget": self.pile_budget,
                   "budget_left": None if self.pile_budget is None else self.pile_budget - len(self.visited),
                   "scan_budget_left": None if self.mission_scans is None else self.mission_scans - self.scans_used,
                   "priority_min": self.priority_min,
                   "delivery": self.delivery,
                   "pings": [{"id": c["id"], "pile_id": c["pile_id"], "status": c["status"], "p_mass": c["p_mass"],
                              **({"vitals": c["vitals"]} if c.get("vitals") else {}),
                              **({"dig_score": c["dig_score"]} if c.get("dig_score") is not None else {})}
                             for c in self.done_cands],
                   "zone_proposals": (self.recon.to_state().get("zone_proposals") if self.recon is not None else []),
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
            return opt, rationale if by != "argmax" else reason
        return opt, rationale

    # ------------------------------------------------------------------ report
    def final_summary(self):
        """One plain paragraph: what was pinged, what is still open, what is not cleared, what was spent."""
        pings = [c for c in self.done_cands if c["status"] == "candidate"]
        ping_s = "; ".join(f"{c['id']} at ({c['xy'][0]:.1f}, {c['xy'][1]:.1f}), t {c.get('t_found', 0):.0f} s"
                           for c in pings) or "none"
        leads = sorted((p for p in self.piles.values() if p["status"] == "lead"), key=lambda p: -(p["priority"] or 0))
        lead_s = "; ".join(f"{p['id']} priority {p['priority']}"
                           + ((f" (radar insufficient evidence after {p['scans']} scans"
                               + (f"; radar reached only {100 * p['radar_coverage_shallow']:.0f}% of its top 2 m: "
                                  "acoustic lead only" if p.get("radar_coverage_shallow", 1) < REVISIT_MIN_COVERAGE else "")
                               + ")") if p.get("scans") else " (not visited)")
                           for p in leads) or "none"
        nc = []
        for p in self.piles.values():
            if p["status"] in ("ping", "lead"):
                continue
            what = {"dismissed_nonhuman": "non-human sound", "no_signal_not_cleared": "nothing heard",
                    "unvisited": "never heard or visited"}.get(p["status"], p["status"])
            nc.append(f"{p['id']} ({what}{', radar inconclusive' if p.get('scans') else ''})")
        return (f"MISSION SUMMARY: pinged (candidate search regions, not confirmed people): {ping_s}. "
                f"Open leads: {lead_s}. Not cleared: {', '.join(nc) or 'none'}. "
                f"Spent {self.scans_used} radar scans" + (f" of {self.mission_scans}" if self.mission_scans else "")
                + f", {len(self.visited)} pile visits, {self.n_swaps} battery swap(s), {self.t:.0f} s mission time, "
                f"{self._fly_s:.0f} drone-seconds of flight.")

    # ------------------------------------------------------------------ state.json
    LOD0_MAX = 300_000             # browser preview budget (~3.6 MB float32)
    TILE_M = 16.0

    def _write_pointcloud(self, pts):
        """pointcloud.bin = preview (<= 300k points); above that, full resolution in 16 m tiles (contract v1.3.1)."""
        pts = np.asarray(pts, "<f4").reshape(-1, 3)

        def put(name, arr):
            path = os.path.join(self.out_dir, name)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            arr.astype("<f4").tofile(path + ".tmp")
            os.replace(path + ".tmp", path)

        lod0 = pts
        if len(pts) > self.LOD0_MAX:             # voxel thinning first, then a seeded random cap
            try:
                from mapping.slam import voxel_down_sample
                lod0 = voxel_down_sample(pts, 0.15)
            except Exception:                    # noqa: BLE001  (mapping deps missing: random thinning only)
                lod0 = pts
            if len(lod0) > self.LOD0_MAX:
                lod0 = lod0[np.random.default_rng(0).choice(len(lod0), self.LOD0_MAX, replace=False)]
        put("pointcloud.bin", lod0)
        self.pc_tiles = []
        if len(lod0) < len(pts):
            k = np.floor(pts[:, :2] / self.TILE_M).astype(int)
            for kx, ky in np.unique(k, axis=0):
                sel = (k[:, 0] == kx) & (k[:, 1] == ky)
                x0, y0 = int(kx * self.TILE_M), int(ky * self.TILE_M)
                name = f"pc_tiles/x{x0:+04d}_y{y0:+04d}.bin"
                put(name, pts[sel])
                self.pc_tiles.append({"url": name, "count": int(sel.sum()),
                                      "bounds": [x0, y0, x0 + self.TILE_M, y0 + self.TILE_M]})
        self.pc_count, self.pc_full_count = int(len(lod0)), int(len(pts))

    def _hm_list(self):
        if self._hm_cache is None:
            z = np.round(self.grid.z.astype(np.float64), 3).ravel().tolist()
            self._hm_cache = (z, json.dumps(z, separators=(",", ":")))
        return self._hm_cache

    @staticmethod
    def _polygon(poly, x, y, r=9.0, n=16):
        """Sector outline, CCW [[x, y], ...] in site metres: the scenario's own polygon, else a 9 m circle."""
        if poly is not None and len(poly) >= 3:
            pts = [[round(float(a), 2), round(float(b), 2)] for a, b in poly]
            area = sum(pts[i][0] * pts[i - 1][1] - pts[i - 1][0] * pts[i][1] for i in range(len(pts)))
            return pts[::-1] if area > 0 else pts          # shoelace > 0 here means clockwise
        return [[round(x + r * np.cos(a), 2), round(y + r * np.sin(a), 2)]
                for a in np.linspace(0, 2 * np.pi, n, endpoint=False)]

    def _drones_out(self):
        out = []
        playing = self.speaker.active(self.t) if self.speaker is not None else {}
        for d in self.flight.poses():
            d = dict(d)
            tr = self._tracks.setdefault(d["id"], [])
            if not tr or self.t - tr[-1][3] >= 2.0:
                tr.append([d["pose"][0], d["pose"][1], d["pose"][2], round(self.t, 1)])
                del tr[:-200]
            d["track"] = list(tr)
            if (self.stage == "listen" or (self.stage == "recon" and self.recon.status == "listening")) \
                    and d["state"] in ("landed", "scanning"):
                d["state"] = "listening"
            if self.speaker is not None and d["id"] in playing:
                d["speaker"] = playing[d["id"]]
            if self.comms is not None:                   # v1.4: mesh link, store-and-forward backlog
                d.update(self.comms.drone_link(d["id"]))
            out.append(d)
        return out

    def _state(self, hm_placeholder=False):
        st = self._state_base(hm_placeholder)
        if self._zone_ai:
            st["zone_suggestions"] = list(st.get("zone_suggestions") or []) + list(self._zone_ai)
        return st

    def _state_base(self, hm_placeholder=False):
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
            "site": {"name": f"{self.site_name} · seed {self.seed}", "bounds": list(self.bounds), "kind": self.site,
                     **({"geo": dict(self.geo)} if self.geo else {})},
            "heightmap": self.recon.heightmap_state() if self.stage == "recon" else   # v1.2.1: mask + null z
            {"origin": list(g.origin), "res": g.res, "w": g.w, "h": g.h,
             "z": "__HM__" if hm_placeholder else self._hm_list()[0]},
            "pointcloud": {"url": "pointcloud.bin", "count": self.pc_count,
                           **({"lod": 0, "full_count": self.pc_full_count, "tiles": self.pc_tiles}
                              if getattr(self, "pc_tiles", None) else {})},
            "drones": self._drones_out(),
            "landing_sites": sites,
            "scans": scans,
            "radar_candidates": cands,
            "pods": [dict(p, placement=self._placement(p), of=(self.pm.budget if self.pm else None),
                          lead=(self.pm.pile if self.pm else None)) for p in getattr(self.flight, "pods", lambda: [])()],
            "pings": [c for c in cands if c["status"] == "candidate" and c.get("kind") == "stationary"],
            "acoustic": self.acoustic_obs,
            "belief": belief,
            "next_scan": next_scan,
            "log": self.log[-300:],
            # ---- v1.1 grid extensions
            "mission": {"kind": "grid", "seed": self.seed, "start_pile": self.start_pile,
                        "pile_budget": self.pile_budget, "priority_min": self.priority_min,
                        "visited": list(self.visited), "active_pile": self.visited[-1] if self.stage in ("pile", "listen") else None,
                        "stage": self.stage, "acoustic_source": self.acoustic_source,
                        "listen_ends": round(self.listen_end, 1) if self.stage == "listen" and self.listen_end else None,
                        "in_transit": upcoming,
                        "radar_penetration": self.penetration, "delivery": self.delivery,
                        "platform": self.platform or "legacy_small",
                        "radar_reach_beneath_m": round(reach_path_m(REACH_REF_RANGE_M, 0.9,
                                                                    dwell=45.0 if self.delivery == "pod" else 20.0), 2),
                        "autonomous": self.autonomous, "dispatch_policy": self.dispatch_policy, "radar": self.radar,
                        "scan_budget": self.mission_scans, "scans_used": self.scans_used,
                        "battery_reserve": self.reserve, "battery_swaps": self.n_swaps,
                        "battery_swap": None if self.swap is None else
                        {"to_pile": self.swap["to_pile"], "ends": self.swap["ends"], "since": self.swap["t"]},
                        "summary": self.final_summary() if self.stage == "done" else None},
            "staging": {"xy": list(self.base_xy), "pads": [list(p) for p in self.pads]},
            "piles": [dict(p) for p in self.piles.values()],
            "acoustic_clips": self.clips,
            "acoustic_agent": (self.last_acoustic or {}).get("agent"),
            "dispatch": self.dispatch,
            "decisions": self.decisions,
            "slam": list(self.slam_metrics.values()) if self.slam else None,
            # ---- v1.4 rescue hotspot + comms relay
            "comms": self.comms.state() if self.comms is not None else None,
            # ---- v1.3 speaker (broadcasts, callouts, responsiveness per sector)
            **(self.speaker.state(self.t) if self.speaker is not None else
               {"broadcasts": [], "callouts": [], "responsiveness": {}}),
            # ---- v1.2 operator tasking + recon
            "tasking": None if self.tracker is None else {"source": self.tracker.source, "revision": self.tracker.revision,
                                                          "path": self.tasking_path},
            **(self.tracker.to_state() if self.tracker is not None else {"zones": [], "zone_suggestions": []}),
            **(self.recon.to_state() if self.recon is not None else {"recon": None, "zone_proposals": []}),
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
                           "depth_m": c.get("depth_m"), "depth_source": c.get("depth_source"),
                           "status": c["status"], "p_mass": c["p_mass"], "t_found": c.get("t_found")}
                          for c in self.done_cands],
                "priorities": {p["id"]: p["priority"] for p in self.piles.values()},
                "statuses": {p["id"]: p["status"] for p in self.piles.values()},
                "acoustic_source": self.acoustic_source, "t": self.t, "scans": self.scans_used,
                "battery_swaps": self.n_swaps, "flight_s": round(self._fly_s, 1),
                "text": self.final_summary()}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--start-pile", default=None, help="A1..C3 (default: seeded random pile)")
    ap.add_argument("--out", default="dashboard/data")
    ap.add_argument("--radar", choices=["model", "dsp"], default="dsp",
                    help="dsp (default): synthetic IF through the real DSP chain; model: direct noise model")
    ap.add_argument("--acoustic", choices=["auto", "stub", "real"], default="auto",
                    help="auto: acoustic/ pipeline if importable or via .venv-audio, else the synthetic stub")
    ap.add_argument("--site", default="grid",
                    help="grid | weiguan | collapse | mesh:<path> (sim/scenario.make_scenario)")
    ap.add_argument("--delivery", choices=["pod", "land"], default=DELIVERY_DEFAULT,
                    help="pod: winch-lowered sensor pod on the pile surface (default); land: the old landing ring")
    ap.add_argument("--platform", default=None,
                    help="sim/platform.py preset: legacy_small (today) | heavy_sar | industrial_sar")
    ap.add_argument("--penetration", choices=list(radar_dsp.PENETRATION_MODELS) + ["realistic"], default=PENETRATION_DEFAULT,
                    help="radar through-rubble model: literature (~3 m beneath) or legacy (optimistic 0.7 dB/m)")
    ap.add_argument("--piles", type=int, default=None, help="cap on pile visits (legacy loop default 3; autonomous: none)")
    ap.add_argument("--no-autonomous", dest="autonomous", action="store_false",
                    help="legacy fixed loop: 3 piles, no battery swaps, no revisits")
    ap.add_argument("--scans", type=int, default=None, help=f"mission radar-scan budget (autonomous default {MISSION_SCANS})")
    ap.add_argument("--dispatch", choices=["acoustic", "nearest", "raster"], default="acoustic",
                    help="dispatch policy (nearest / raster are evaluation baselines)")
    ap.add_argument("--priority-min", type=int, default=PRIORITY_MIN)
    ap.add_argument("--frames", action="store_true")
    ap.add_argument("--realtime", action="store_true")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--decider", choices=["none", "gemini"], default="none",
                    help="dispatch-gate agent (agents.gemini_decider.make_decider); falls back to argmax")
    ap.add_argument("--slam", action="store_true",
                    help="build each pile's map with LiDAR SLAM (mapping.slam, KISS-ICP); cached in dashboard/data/slam_cache")
    ap.add_argument("--slam-scans", type=int, default=SLAM_SCANS)
    ap.add_argument("--tasking", default="auto",
                    help="auto (default: <out>/tasking.json, re-read live; no active zones -> recon first) | PATH | "
                         "default (scenario pile outlines) | none (legacy: every pile, no zones)")
    ap.add_argument("--recon-accept-s", type=float, default=RECON_ACCEPT_S,
                    help="autonomous: auto-accept recon zone proposals after this many seconds without operator tasking")
    ap.add_argument("--no-speaker", dest="speaker", action="store_false",
                    help="disable the loudspeaker call-out / reassurance / guidance (contract v1.3)")
    ap.add_argument("--no-comms", dest="comms", action="store_false",
                    help="disable the rescue hotspot / comms relay (contract v1.4)")
    ap.add_argument("--uplink", default="oneweb", choices=["oneweb", "lte", "starlink", "none"],
                    help="van internet backhaul (Tainan: OneWeb via Chunghwa Telecom, or LTE); none = local only")
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
                    slam=a.slam, slam_scans=a.slam_scans, autonomous=a.autonomous, mission_scans=a.scans,
                    dispatch_policy=a.dispatch, penetration=a.penetration, site=a.site,
                    platform=a.platform, delivery=a.delivery, speaker=a.speaker,
                    tasking=os.path.join(a.out, "tasking.json") if a.tasking == "auto" else a.tasking,
                    recon_accept_s=a.recon_accept_s, comms=a.comms,
                    uplink=None if a.uplink == "none" else a.uplink)
    m.run()
    s = m.summary()
    print(f"done in {time.time() - t0:.1f} s wall, {m.t:.0f} s sim; visited {' -> '.join(s['visited'])}; "
          f"pings {[c['id'] for c in s['pings'] if c['status'] == 'candidate']}; acoustic {s['acoustic_source']}; "
          f"wrote {a.out}/state.json", file=sys.stderr)


if __name__ == "__main__":
    main()
