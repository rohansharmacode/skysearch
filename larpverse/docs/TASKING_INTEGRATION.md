# Operator tasking zones + recon: GridMission integration patch (v1.2 / v1.2.1)

Owner of this doc, `sim/tasking.py` and `sim/recon.py`: tasking agent. `sim/grid_mission.py` belongs to the mission
builder; this is the exact patch to apply there. It was tested on a scratch copy (results below).

**Base:** working-tree `sim/grid_mission.py` as of 2026-11-07 ~22:40 (md5 `a0eabf69`: launch_pile,
site=, platform=, delivery=, vitals look). Apply with `patch -p1 < <(sed -n '/^--- a\/sim/,$p' docs/TASKING_INTEGRATION.md | sed '/^\`\`\`/,$d')`
or by hand; every hunk is anchored on short, stable context, so a later base needs only small fix-ups.

## What the patch does

- **CLI** `--tasking auto|PATH|default|none` (default `auto` = `<out>/tasking.json`), `--recon-accept-s` (30).
  Constructor `GridMission(tasking=None, recon_accept_s=30)`: `None` keeps the legacy behaviour (every pile, `zones: []`),
  so tests and `evaluate_grid` are unchanged.
- **Tasking:** `ZoneTracker(tasking, piles=scenario.piles, pile_xy, bounds=self.bounds)`. Zones are the search units. A
  zone maps to the pile(s) whose pieces/centre lie inside it (`tracker.zone_piles`); a pile-less zone takes the
  nearest pile within 15 m. Dispatch candidates are the piles of zones with `should_visit()`. Acoustic priority
  still orders them. Operator-tasked zones are searched even without an acoustic lead (flag `tasked`, bypasses
  `priority_min`). Scenario-default zones keep the old `priority_min` gate, so seed 7 behaves as before.
- **Inside a zone:** landing patches outside zone + 4 m are rejected (`reason: outside_zone`), unless fewer than 3
  candidates would remain. The localiser prior is multiplied by 0.02 outside zone + 1 m. Both are re-applied
  after the LiDAR window rebuild.
- **Status machine:** `start_search` on dispatch, `mark_scanning` and `add_scan` while scanning,
  `mark_lidar_disc` (survey radius + 5 m) when the survey is done. `finish_search` runs once all of a zone's
  piles are done, giving `ping | beyond_radar_reach (radar_reached < 0.2) | lead | searched_no_ping`.
  A finished zone is revisited only when `reopened` (a new human-like clip associated with it, or an operator
  re-task); that revisit bypasses `priority_min` and carries `retask_reason` into the dispatch reason.
- **Acoustic:** after each listen, `mark_listened` runs and every clip goes through `tracker.add_clip`. Human-like
  clips outside every zone become `zone_suggestions`, which are logged but not logged during recon.
  `_finish_listen(drones=None, gate=True)` is reused for recon perches with `gate=False`.
- **Live tasking:** `_poll_tasking()` runs at the top of `step()`. It is a cheap mtime check, followed by `load_tasking`
  and `update_tasking` on change. New zones join as unsearched. Cancelled zones stop being candidates but keep
  their progress. If a revision lands while the fleet is returning, one more dispatch gate runs.
- **Recon (no active zones):** `stage = "recon"`, and `Recon(self.grid, self.flight, bounds=self.bounds, aoi=tasking.aoi,
  obstacles=public_dict().site_info.obstacles)` drives the fleet:
  - A boustrophedon mapping flight, with each leg flown 13 m above the tallest mapped structure along it.
  - Battery swaps at the pads.
  - Synchronised perch-and-listen rounds on flat cells. The acoustic pipeline runs on the perched drones only.
  - `zone_proposals` at the end.
  The phases are `recon_mapping | recon_listening | recon_proposing`, and perched drones show as `listening`.
  While recon runs, `state.heightmap` is the fresh map with `mask` and `z: null` for unmapped cells (v1.2.1).
  An operator POST ends recon. With `autonomous`, proposals scoring at least 0.3 are auto-accepted in memory
  after `recon_accept_s` (source `auto-accepted-proposals`); the operator's file is never written.
- **State:** `tasking: {source, revision, path}`, plus `zones`, `zone_suggestions`, `recon` and `zone_proposals`,
  in exactly the shapes of the contract and `dashboard/fixtures`. Polygons are CCW, in site metres.

## Scratch test results (seed 7, `--acoustic stub`, scripts/heavy.sh)

| run | result |
|---|---|
| A `--tasking default` | 9 zones, A2→A3→B2→A3 revisit; pings A2, B2, A3; zones A2/A3/B2 `ping`, other 6 `unsearched` (scan budget). Same piles as the legacy loop |
| B `auto`, no tasking file | recon 176 s: 100% mapped, 3 perch rounds, 9 proposals (one per pile; B2 knock lead ranked first). Auto-accept after 30 s, then A2→B2→A3→A1→B1→C3; pings A2, B2, A3; zones 4 `ping` / 3 `searched_no_ping` / 2 `unsearched` |
| C operator file + live revisions | rev1 Z1(A3), Z2(C3): Z1 `ping`, C3 started. rev2 at t=300 (cancel Z2, add Z3=B1): Z2 `cancelled` with its progress kept, Z3 searched → `searched_no_ping`. rev3 during the return (`retask` Z1): `retask_reason` "operator re-task (revision 3)", redeploy, A3 ping again. B2 knock outside every zone → suggestion S1 (evidence: 4 clips) |
| `tests/test_grid_mission.py` on the patched copy | 18 passed, 1 skipped (legacy path unchanged) |
| D `--site collapse` auto | recon 99.9% of 320 m mapped, 18 perches, 96 standing buildings cut (93 prior + 3 from roof cues), 94 proposals → 82 auto-accepted. T1a searched, then T1b dispatched on a voice lead (priority 85). It then crashed in `planner.initial_sites`: "0 pod placements" at T1b means `best` is None with fewer than 3 candidates. This is a pre-existing planner/pod issue that needs a guard, not this patch |

## Notes for the mission builder

- `KinematicFlight.plan` refuses altitudes above 25 m. On the collapse site (towers up to 24 m) recon legs want
  about 37 m, so recon falls back to the backend's terrain + 3 m and logs it once. Raise the ceiling (for example 60 m) for the
  collapse site.
- `planner.initial_sites` needs a guard for fewer than 3 candidates (collapse run D).
- Radar coverage per zone uses a 3 m reach disc around each landed sensor (`REACH_CAP_M`), as the contract asks.

## Patch

```diff
--- a/sim/grid_mission.py
+++ b/sim/grid_mission.py
@@ -37,6 +37,8 @@
 from .radar_dsp import REACH_REF_RANGE_M, reach_path_m
 from .landing import detect_landing_sites, detect_pod_sites
 from .radar_model import make_generator
+from .recon import Recon, proposals_to_tasking
+from .tasking import Tasking, ZoneTracker, default_tasking, load_tasking, tasking_mtime
 from .sensor_model import rubble_path
 from .scenario import BASE_XY, DRONE_PADS, SITE_BOUNDS, make_scenario
 try:
@@ -65,6 +67,14 @@
 DELIVERY_DEFAULT = "pod"             # winch-lowered sensor pod on the pile surface; "land" = the old landing ring
 RASTER = ["A1", "B1", "C1", "C2", "B2", "A2", "A3", "B3", "C3"]   # fixed lawnmower baseline (row by row)
 V1_CLASS = {"voice": "voice", "shout": "shout", "knocking": "knocking", "tapping": "tapping"}
+# operator tasking (contracts v1.2, sim/tasking.py) and recon (sim/recon.py)
+LAND_ZONE_MARGIN_M = 4.0       # landing patches may lie this far outside the tasked zone (radar looks in)
+MIN_ZONE_SITES = 3             # ... unless fewer than this many candidates are left inside it
+LOC_ZONE_MARGIN_M = 1.0        # localiser prior: full weight inside the zone (+ this margin) ...
+ZONE_PRIOR_FLOOR = 0.02        # ... and this relative weight outside it
+LIDAR_SURVEY_M = SURVEY_RADIUS + 5.0   # the per-pile LiDAR survey maps cells this close to the pile centre
+RECON_ACCEPT_S = 30.0          # autonomous: auto-accept recon proposals after this long without operator tasking
+RECON_ACCEPT_MIN = 0.3         # ... those scoring at least this
 
 
 class PileMission(Mission):
@@ -87,6 +97,29 @@
         self.next_scan = None
         self.stage = "done"
 
+    zone_id = None
+    zone_mask = None                      # bool on self.grid: landing patches allowed (zone + LAND_ZONE_MARGIN_M)
+
+    def _advance(self):
+        before = self.stage
+        super()._advance()
+        if before == "survey" and self.stage == "decide" and self.zone_mask is not None:
+            cand = [s for s in self.sites if s["status"] == "candidate"]
+            out = []
+            for s in cand:
+                i, j = self.grid.index(*s["xy"])
+                if not self.zone_mask[j, i]:
+                    out.append(s)
+            if len(cand) - len(out) >= MIN_ZONE_SITES:
+                for s in out:
+                    s["status"], s["reason"] = "rejected", "outside_zone"
+                if out:
+                    self._say(f"{len(out)} landing patches lie outside tasked zone {self.zone_id} "
+                              f"(+{LAND_ZONE_MARGIN_M:.0f} m); not used")
+            elif out:                     # too few inside: keep them all rather than strand the radar
+                self._say(f"only {len(cand) - len(out)} landing patches inside tasked zone {self.zone_id} "
+                          f"(+{LAND_ZONE_MARGIN_M:.0f} m); also using the {len(out)} nearby ones outside it")
+
 
 class GridMission:
     def __init__(self, seed=7, start_pile=None, radar="dsp", out_dir=None, frames=False, verbose=True,
@@ -96,12 +129,16 @@
                  slam=False, slam_scans=SLAM_SCANS, slam_kw=None, slam_cache=SLAM_CACHE,
                  autonomous=True, mission_scans=None, dispatch_policy="acoustic", reserve=RESERVE, swap_s=SWAP_S,
                  listen_s=None, revisit_max=2, penetration=None, launch_pile=None, site="grid", platform=None,
-                 delivery=None):
+                 delivery=None,
+                 tasking=None, recon_accept_s=RECON_ACCEPT_S):
         """``autonomous`` (default): keep searching until the mission scan budget (``mission_scans``, default
         24), the time limit or the open leads run out; battery swaps at the pads; open leads without a firm
         ping are revisited once. ``pile_budget`` caps pile visits (None = no cap; legacy mode uses 3).
         ``dispatch_policy``: acoustic (ours) | nearest | raster (baselines for sim/evaluate_grid.py; they do
-        not listen unless ``listen_s`` is given)."""
+        not listen unless ``listen_s`` is given).
+        ``tasking`` (contracts v1.2): None / "none" = legacy (every pile, no zones); "default" = scenario-default
+        zones; a path = operator tasking file, re-read each step (mtime check; no active zones -> recon first);
+        or a ``sim.tasking.Tasking``. ``recon_accept_s``: autonomous auto-accept delay for recon proposals."""
         if dispatch_policy not in ("acoustic", "nearest", "raster"):
             raise ValueError(f"dispatch_policy {dispatch_policy!r}")
         self.delivery = delivery or DELIVERY_DEFAULT
@@ -187,6 +224,22 @@
         self._pc_dirty = False
         self._hm_cache = None
         self.pc_count = 0
+        # --- operator tasking zones (contracts v1.2) + recon
+        self.tracker, self.recon, self.tasking_path, self._tk_mtime = None, None, None, None
+        self.recon_accept_s, self._recon_ready_t, self._retask_pending = recon_accept_s, None, False
+        self._zone_of = {}             # pile -> zone id it is being searched for
+        tk = None
+        if tasking is not None and tasking != "none":
+            if isinstance(tasking, Tasking):
+                tk = tasking
+            elif tasking == "default":
+                tk = default_tasking(self.scenario)
+            else:
+                self.tasking_path, self._tk_mtime = str(tasking), tasking_mtime(tasking)
+                tk = load_tasking(tasking, allow_empty=True)
+            self.tracker = ZoneTracker(None, piles=self.scenario.piles, pile_xy=self.world.pile_xy,
+                                       bounds=self.bounds, revisit_max=revisit_max)
+            self.tracker.update_tasking(tk, t=0.0)
         if out_dir:
             os.makedirs(out_dir, exist_ok=True)
             if frames:
@@ -203,7 +256,18 @@
                   + ("; deeper than that is beyond radar reach (acoustic lead only)" if self.penetration == "literature"
                      else ". WARNING: LEGACY optimistic model (flat 0.7 dB/m), not literature-backed; "
                      "--penetration literature limits a scan to ~3 m beneath it"))
-        self._start_pile(self.start_pile, reason="operator-selected start pile")
+        if self.tracker is None:
+            self._start_pile(self.start_pile, reason="operator-selected start pile")
+        else:
+            self._say(f"Operator tasking: {len(self.tracker.zone_ids())} active zone(s) "
+                      f"({self.tracker.source or 'none'}, revision {self.tracker.revision})"
+                      + (f" from {self.tasking_path}" if self.tasking_path else "")
+                      + "; the drones search only inside tasked zones")
+            self._flush_zone_events()
+            if self._tasked_piles():
+                self._start_pile(*self._first_zone_pile())
+            else:
+                self._start_recon(tk.aoi if tk is not None else None)
 
     # ------------------------------------------------------------------ helpers
     def _say(self, msg):
@@ -218,6 +282,118 @@
     def _drone_xy(self):
         return {p["id"]: (float(p["pose"][0]), float(p["pose"][1])) for p in self.flight.poses()}
 
+    # ------------------------------------------------------------------ tasking zones (v1.2)
+    def _flush_zone_events(self):
+        if self.tracker is not None:
+            for _, msg in self.tracker.events:
+                self._say(f"Zones: {msg}")
+            self.tracker.events.clear()
+
+    def _zone_search_piles(self, zid):
+        """Piles searched for zone ``zid``: those with pieces inside it, else the nearest pile within 15 m."""
+        ps = self.tracker.zone_piles(zid)
+        if ps:
+            return ps
+        cx, cy = self.tracker.zone(zid)["centroid"]
+        n, (x, y) = min(self.world.pile_xy.items(), key=lambda e: np.hypot(e[1][0] - cx, e[1][1] - cy))
+        return [n] if np.hypot(x - cx, y - cy) <= 15.0 else []
+
+    def _pile_zone(self, pile):
+        """Zone to search ``pile`` for: an active (surveying/scanning) one, else the best should_visit() one."""
+        if self.tracker is None:
+            return None
+        cands = [z for z in self.tracker.visit_candidates() if pile in self._zone_search_piles(z)]
+        act = [z for z in cands if self.tracker.zone(z)["status"] in ("surveying", "scanning")]
+        return (act or cands or [None])[0]
+
+    def _tasked_piles(self):
+        """Piles inside zones that should be visited (None = no tasking: every pile)."""
+        if self.tracker is None:
+            return None
+        return list(dict.fromkeys(p for z in self.tracker.visit_candidates() for p in self._zone_search_piles(z)))
+
+    def _reopened(self, pile):
+        z = self._pile_zone(pile)
+        return z is not None and self.tracker.zone(z)["reopened"]
+
+    def _operator_tasked(self):
+        return self.tracker is not None and self.tracker.source != "scenario-default"
+
+    def _first_zone_pile(self):
+        piles = self._tasked_piles()
+        if self.start_pile in piles:
+            return self.start_pile, f"operator-selected start pile, inside tasked zone {self._pile_zone(self.start_pile)}"
+        ranked = sorted(piles, key=lambda n: -(self.piles[n]["priority"] or 0))
+        n = ranked[0]
+        z = self._pile_zone(n)
+        pr = self.piles[n]["priority"]
+        return n, (f"first tasked zone {z} ({self.tracker.zone(z)['name']})"
+                   + (f", acoustic priority {pr}" if pr is not None else ", operator order"))
+
+    def _zone_prior(self, pm):
+        """Restrict the localiser to the zone: prior weight ZONE_PRIOR_FLOOR outside it."""
+        if self.tracker is None or pm.zone_id is None:
+            return
+        m = self.tracker.mask_on(pm.zone_id, pm.grid, LOC_ZONE_MARGIN_M)
+        if m.any():
+            pm.loc.logb = pm.loc.logb + np.log(np.where(m, 1.0, ZONE_PRIOR_FLOOR))
+            pm.loc._p = None
+
+    def _poll_tasking(self):
+        """Re-read the operator tasking file when its mtime changes; merge the revision live."""
+        if self.tracker is None or not self.tasking_path:
+            return
+        m = tasking_mtime(self.tasking_path)
+        if m == self._tk_mtime:
+            return
+        self._tk_mtime = m
+        tk = load_tasking(self.tasking_path, allow_empty=True)
+        if tk is None:
+            return
+        if self.tracker.update_tasking(tk, t=self.t):
+            self._flush_zone_events()
+            self._retask_pending = True
+            if self.stage == "recon" and self._tasked_piles():
+                self._end_recon(f"operator tasking revision {tk.revision}")
+
+    # ------------------------------------------------------------------ recon (no zones tasked)
+    def _start_recon(self, aoi=None):
+        self.stage = "recon"
+        obstacles = (self.scenario.public_dict().get("site_info") or {}).get("obstacles")   # prior footprints
+        self.recon = Recon(self.grid, self.flight, bounds=self.bounds, aoi=aoi, pile_xy=self.world.pile_xy,
+                           say=self._say, t0=self.t, obstacles=obstacles)
+
+    def _recon_step(self):
+        self.t += self.dt
+        self.k += 1
+        rc = self.recon
+        rc.step(self.t, self.flight.step(self.dt, self.t))
+        if rc.wants_listen:
+            n0 = len(self.clips)
+            self._finish_listen(drones=dict(rc.wants_listen), gate=False)
+            rc.add_listen(self.clips[n0:], self.t)
+        if rc.status == "proposing":
+            if self._recon_ready_t is None:
+                self._recon_ready_t = self.t
+                if self.autonomous:
+                    self._say(f"RECON: waiting for operator tasking; auto-accepting proposals scoring >= "
+                              f"{RECON_ACCEPT_MIN} after {self.recon_accept_s:.0f} s (unattended demo)")
+            elif self.autonomous and self.t - self._recon_ready_t >= self.recon_accept_s:
+                tk = proposals_to_tasking(rc.proposals, min_score=RECON_ACCEPT_MIN)
+                if not tk.zones:
+                    self._say("RECON: no proposal scores high enough to search; mission ends")
+                    self.stage = "return"
+                    return
+                self.tracker.update_tasking(tk, t=self.t)
+                self._flush_zone_events()
+                self._end_recon(f"no operator tasking {self.recon_accept_s:.0f} s after the proposals: auto-accepted "
+                                f"{', '.join(z.id for z in tk.zones)}")
+
+    def _end_recon(self, reason):
+        self.recon.finish()
+        self._say(f"RECON over: {reason}. Targeted search of the tasked zones starts")
+        self._start_pile(*self._first_zone_pile())
+
     def _start_pile(self, name, reason):
         cx, cy = self.world.pile_xy[name]
         pw = self.world.pile_world(name)
@@ -242,6 +418,15 @@
                 g, c, lead_xy=lead, max_slope=getattr(pod, "max_slope_deg", 30.0), winch_m=getattr(pod, "winch_m", 20.0),
                 hover_clear=getattr(pod, "hover_clear_m", 5.0))
         self.pm.prior_obs = [o for o in self.done_scans if o.get("pile_id") == name] if revisit else []
+        zid = self._pile_zone(name)
+        if zid is not None:
+            self._zone_of[name] = self.pm.zone_id = zid
+            self.pm.zone_mask = self.tracker.mask_on(zid, self.pm.grid, LAND_ZONE_MARGIN_M)
+            self._zone_prior(self.pm)
+            if self.tracker.zone(zid)["status"] not in ("surveying", "scanning"):
+                self.tracker.start_search(zid, self.t)
+            reason = f"{reason} [zone {zid}]"
+            self._flush_zone_events()
         self._preload(self.pm)
         if self.slam:
             self._slam_ingest(name)
@@ -351,6 +536,8 @@
                   f"{100 * m['final_drift_m']:.0f} cm, map height error {mae} MAE (sim diagnostic vs scenario terrain)")
 
     def _on_survey_done(self, pm):
+        if self.tracker is not None:
+            self.tracker.mark_lidar_disc(self.world.pile_xy[pm.pile], LIDAR_SURVEY_M)
         pts = self._ingested_points()
         if pts is None:
             return
@@ -382,12 +569,16 @@
         from .planner import Planner
         pm.loc = Localiser(g)
         pm.planner = Planner(g, pm.loc.cfg, seed=self.seed)
+        self._zone_prior(pm)
         self._preload(pm)
         pm._say(f"Heightmap window rebuilt from {int(sel.sum())} LiDAR points")
 
     # ------------------------------------------------------------------ main loop
     def step(self):
-        if self.stage == "pile":
+        self._poll_tasking()
+        if self.stage == "recon":
+            self._recon_step()
+        elif self.stage == "pile":
             self.pm.step()
             self.t, self.k = self.pm.t, self.k + 1
             self._track_pile()
@@ -416,6 +607,10 @@
                     self.swap = None
                     self._say(f"Batteries swapped (swap {self.n_swaps}); redeploying to {nxt}")
                     self._start_pile(nxt, reason)
+            elif self.stage == "return" and docked and self._retask_pending and self.visited \
+                    and self._targets() and self.t < self.max_t:
+                self._retask_pending = False         # new tasking while returning: one more dispatch gate
+                self._gate(None)
             elif self.stage == "return" and docked:
                 self.stage = "done"
                 self._say("All drones docked at the staging area. Mission complete.")
@@ -451,6 +646,8 @@
                 if self.pm.stage == "takeoff" or far:
                     ph = "dispatch"
             self.phase = ph
+        elif self.stage == "recon":
+            self.phase = self.recon.phase
         elif self.stage == "listen":
             self.phase = "acoustic"
         elif self.stage == "return":
@@ -475,6 +672,12 @@
         return out
 
     def _track_pile(self):
+        zid = self._zone_of.get(self.pm.pile)
+        if zid is not None and zid in self.tracker.zones:
+            for o in self.pm.scans:
+                self.tracker.add_scan(o)
+            if self.pm.scans_cmd or self.pm.phase in ("landing", "radar", "relocate", "rescan"):
+                self.tracker.mark_scanning(zid, self.t)
         p = self.piles[self.pm.pile]
         if self.pm.phase in ("landing", "radar", "relocate", "rescan") or self.pm.scans_cmd:
             if p["status"] == "surveying":
@@ -528,6 +731,17 @@
                       f"{reach_txt}. Pile stays not cleared"
                       + (("; beyond radar reach from these sites -> acoustic lead only, for ground teams" if beyond
                           else "; its acoustic lead stays open for a revisit") if lead else ""))
+        zid = self._zone_of.get(name)
+        if zid is not None and zid in self.tracker.zones:
+            for o in pm.scans:
+                self.tracker.add_scan(o)
+            for c in firm:
+                self.tracker.add_ping(zid, c["id"])
+            zp = self._zone_search_piles(zid)
+            if all(q in self.visited for q in zp):
+                self.tracker.finish_search(zid, self.t, pings=list(self.tracker.zone(zid)["pings"]),
+                                           lead=any(self.piles[q]["status"] == "lead" for q in zp))
+            self._flush_zone_events()
         if self.listen_s > 0:
             self.stage = "listen"
             if self.delivery == "pod" and self._land_for_listen(pm):
@@ -644,9 +858,9 @@
         return f"depth not measured (assumed {c.get('depth_m') or 1.5:.1f} m; the ranges cannot separate depth from position)"
 
     # ------------------------------------------------------------------ acoustic + dispatch
-    def _finish_listen(self):
+    def _finish_listen(self, drones=None, gate=True):
         self.n_listen += 1
-        drones = self._drone_xy()
+        drones = drones or self._drone_xy()
         exclude = list(dict.fromkeys(self.visited))
         if self.acoustic_fn is not None:
             res, src = self.acoustic_fn(self.scenario, drones, exclude, self.seed + 101 * self.n_listen), "custom"
@@ -687,6 +901,15 @@
                                           "bearing_deg": clip["bearing_deg"], "width_deg": clip["width_deg"],
                                           "class": V1_CLASS.get(clip["class"], "unknown"),
                                           "confidence": clip["scores"]["reliability"], "pile_id": clip["pile_id"]})
+        if self.tracker is not None:
+            self.tracker.mark_listened(drones)
+            n_sugg = len(self.tracker.suggestions)
+            for clip in self.clips[-len(res["clips"]):] if res["clips"] else []:
+                self.tracker.add_clip(clip)
+            for s in ([] if self.stage == "recon" else self.tracker.suggest_zone([])[n_sugg:]):
+                self._say(f"Zone suggestion {s['id']} at ({s['centre'][0]:.0f}, {s['centre'][1]:.0f}), r {s['radius_m']:.0f} m: "
+                          f"{s['why']} -> for the operator to accept")
+            self._flush_zone_events()
         for ap in res["piles"]:
             p = self.piles.get(ap["pile_id"])
             if p is None or ap["pile_id"] in self.visited:
@@ -703,7 +926,8 @@
                     st = "no_signal_not_cleared"
                     why = f"Heard a non-human-like sound once (needs 2 listens to dismiss). {why}"
             p.update(priority=new, status=st, why=why)
-        self._gate(res)
+        if gate:
+            self._gate(res)
 
     @staticmethod
     def _clip_bearings(c, drones):
@@ -721,8 +945,9 @@
 
     def _gate(self, res):
         """Dispatch gate after a pile (and its listen): re-rank, then dispatch / revisit / swap / stop."""
-        ranked = sorted((p for p in self.piles.values() if p["id"] not in self.visited and p["priority"] is not None),
-                        key=lambda p: (-p["priority"], p["id"]))
+        tasked = self._tasked_piles()
+        ranked = sorted((p for p in self.piles.values() if p["id"] not in self.visited and p["priority"] is not None
+                         and (tasked is None or p["id"] in tasked)), key=lambda p: (-p["priority"], p["id"]))
         if ranked:
             self._say("Pile priority: " + ", ".join(f"{p['id']} {p['priority']}" for p in ranked[:6]))
         here = self.visited[-1]
@@ -768,7 +993,8 @@
 
     def _targets(self):
         """(pile_id, action, score) in this policy's preference order (no truth, no budget checks)."""
-        unvisited = [n for n in self.piles if n not in self.visited]
+        tasked = self._tasked_piles()
+        unvisited = [n for n in self.piles if n not in self.visited and (tasked is None or n in tasked)]
         if self.dispatch_policy == "nearest":
             c = np.mean(list(self._drone_xy().values()), axis=0)
             order = sorted(unvisited, key=lambda n: (self._leg_s(c, self.world.pile_xy[n]), n))
@@ -780,7 +1006,12 @@
         if self.autonomous:
             for name in dict.fromkeys(self.visited):
                 p = self.piles[name]
-                if p["status"] != "ping" and (p["priority"] or 0) >= self.priority_min \
+                if tasked is not None and name not in tasked:
+                    continue                     # finished zone: no revisit without new evidence / re-task
+                if self.tracker is not None and self._reopened(name):
+                    out.append((name, "rescan_current" if name == self.visited[-1] else "revisit",
+                                max(p["priority"] or 0, self.priority_min)))
+                elif p["status"] != "ping" and (p["priority"] or 0) >= self.priority_min \
                         and self.visited.count(name) < self.revisit_max \
                         and p.get("radar_coverage_shallow", 1.0) >= REVISIT_MIN_COVERAGE:
                     out.append((name, "rescan_current" if name == self.visited[-1] else "revisit",
@@ -833,7 +1064,8 @@
                     ("nonhuman", p["status"] == "dismissed_nonhuman"),
                     ("silent", p["status"] == "no_signal_not_cleared" or prio is None),
                     ("no_firm_ping", action != "dispatch"),
-                    ("unreachable", ts is None)) if on]
+                    ("unreachable", ts is None),
+                    ("tasked", self._operator_tasked() or (self.tracker is not None and self._reopened(pid)))) if on]
                 opts.append({"id": f"o{len(opts) + 1}", "action": action, "pile_id": pid, "priority": prio,
                              "score": score, "travel_s": ts, "battery_after_min": bat_after, "evidence": ev,
                              "flags": flags})
@@ -854,7 +1086,7 @@
         ok = [o for o in opts if o["action"] != "stop" and "unreachable" not in o["flags"]
               and (self.autonomous or o["action"] == "dispatch")]
         if self.dispatch_policy == "acoustic":
-            ok = [o for o in ok if (o["priority"] or 0) >= self.priority_min]
+            ok = [o for o in ok if (o["priority"] or 0) >= self.priority_min or "tasked" in o["flags"]]
         if not ok:
             if self.dispatch_policy == "acoustic":
                 return opts, stop, (f"no pile left with an acoustic lead at or above priority {self.priority_min}"
@@ -868,6 +1100,12 @@
             reason = f"nearest unvisited pile ({top['pile_id']}, {top['travel_s']:.0f} s away); systematic baseline"
         elif self.dispatch_policy == "raster":
             reason = f"next pile in the fixed raster order ({top['pile_id']}); systematic baseline"
+        elif top["action"] != "dispatch" and self.tracker is not None and self._reopened(top["pile_id"]):
+            z = self._pile_zone(top["pile_id"])
+            reason = f"revisit {top['pile_id']}: zone {z} reopened ({self.tracker.zone(z)['retask_reason']})"
+        elif top["action"] == "dispatch" and "tasked" in top["flags"] and (p["priority"] or 0) < self.priority_min:
+            pr = "" if p["priority"] is None else f", priority {p['priority']}"
+            reason = f"operator-tasked zone {self._pile_zone(top['pile_id'])} not searched yet (no acoustic lead{pr})"
         elif top["action"] != "dispatch":
             reason = (f"revisit {top['pile_id']}: its acoustic lead (priority {p['priority']}) is still open after "
                       f"{p.get('scans', 0)} radar scans without a firm fix, and no unvisited pile scores higher; fresh "
@@ -970,7 +1208,8 @@
                 tr.append([d["pose"][0], d["pose"][1], d["pose"][2], round(self.t, 1)])
                 del tr[:-200]
             d["track"] = list(tr)
-            if self.stage == "listen" and d["state"] in ("landed", "scanning"):
+            if (self.stage == "listen" or (self.stage == "recon" and self.recon.status == "listening")) \
+                    and d["state"] in ("landed", "scanning"):
                 d["state"] = "listening"
             out.append(d)
         return out
@@ -1001,8 +1240,9 @@
             "schema": "larpverse.state/v1", "t": round(self.t, 1), "phase": self.phase,
             "site": {"name": f"{self.site_name} · seed {self.seed}", "bounds": list(self.bounds), "kind": self.site,
                      **({"geo": dict(self.geo)} if self.geo else {})},
-            "heightmap": {"origin": list(g.origin), "res": g.res, "w": g.w, "h": g.h,
-                          "z": "__HM__" if hm_placeholder else self._hm_list()[0]},
+            "heightmap": self.recon.heightmap_state() if self.stage == "recon" else   # v1.2.1: mask + null z
+            {"origin": list(g.origin), "res": g.res, "w": g.w, "h": g.h,
+             "z": "__HM__" if hm_placeholder else self._hm_list()[0]},
             "pointcloud": {"url": "pointcloud.bin", "count": self.pc_count},
             "drones": self._drones_out(),
             "landing_sites": sites,
@@ -1038,6 +1278,11 @@
             "dispatch": self.dispatch,
             "decisions": self.decisions,
             "slam": list(self.slam_metrics.values()) if self.slam else None,
+            # ---- v1.2 operator tasking + recon
+            "tasking": None if self.tracker is None else {"source": self.tracker.source, "revision": self.tracker.revision,
+                                                          "path": self.tasking_path},
+            **(self.tracker.to_state() if self.tracker is not None else {"zones": [], "zone_suggestions": []}),
+            **(self.recon.to_state() if self.recon is not None else {"recon": None, "zone_proposals": []}),
         }
 
     def build_state(self):
@@ -1109,6 +1354,11 @@
     ap.add_argument("--slam", action="store_true",
                     help="build each pile's map with LiDAR SLAM (mapping.slam, KISS-ICP); cached in dashboard/data/slam_cache")
     ap.add_argument("--slam-scans", type=int, default=SLAM_SCANS)
+    ap.add_argument("--tasking", default="auto",
+                    help="auto (default: <out>/tasking.json, re-read live; no active zones -> recon first) | PATH | "
+                         "default (scenario pile outlines) | none (legacy: every pile, no zones)")
+    ap.add_argument("--recon-accept-s", type=float, default=RECON_ACCEPT_S,
+                    help="autonomous: auto-accept recon zone proposals after this many seconds without operator tasking")
     ap.add_argument("--quiet", action="store_true")
     a = ap.parse_args(argv)
     t0 = time.time()
@@ -1123,7 +1373,9 @@
                     pile_budget=a.piles, priority_min=a.priority_min, acoustic=a.acoustic, decider=decider,
                     slam=a.slam, slam_scans=a.slam_scans, autonomous=a.autonomous, mission_scans=a.scans,
                     dispatch_policy=a.dispatch, penetration=a.penetration, site=a.site,
-                    platform=a.platform, delivery=a.delivery)
+                    platform=a.platform, delivery=a.delivery,
+                    tasking=os.path.join(a.out, "tasking.json") if a.tasking == "auto" else a.tasking,
+                    recon_accept_s=a.recon_accept_s)
     m.run()
     s = m.summary()
     print(f"done in {time.time() - t0:.1f} s wall, {m.t:.0f} s sim; visited {' -> '.join(s['visited'])}; "
```
