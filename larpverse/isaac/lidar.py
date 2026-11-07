"""Spinning LiDAR for the surveying drones.

* ``PhysxLidar``     rays through ``compat.physx_raycaster`` (hits the scene.py colliders). Isaac only.
* ``HeightmapLidar`` numpy ray-march against ``Scenario.heightmap()``; used by ``--dry-run`` (no omni/pxr).
* ``RtxLidar``       optional ``--rtx-lidar`` path (RTX sensor + annotator); guarded, falls back to PhysX.

All return world-frame (N, 3) float32 hit points; ``PointAccumulator`` keeps them per pile.

``--lidar-profile small|heavy`` (``add_lidar_args`` / ``make_lidar``; default heavy, see docs/PLATFORM.md):

* small: 96 PhysX rays per drone per frame, 16 rings, 0.15 m map voxel (the original demo).
* heavy: PhysX rays sized to a frame-time budget (``budget_ms`` per surveying drone per frame, default 4 ms =
  12 ms of a 33 ms frame for 3 drones).  The per-ray cost starts at the ASSUMED ``COST_US_ASSUMED`` (20 us:
  python call + float conversion + one PhysX BVH query; an estimate, nobody has timed it on the L40S) and is
  then MEASURED every sweep (EMA), so the ray count settles at what the GPU box actually affords (clamped to
  96..4096).  That is still ~10^4 rays/s per drone, two orders below an OS1-128 (1.31 M pts/s), so for real
  density the heavy profile prefers the RTX LiDAR with a dense config (OS1-128 / Hesai XT32 profiles, tried
  in order, every call guarded; falls back to PhysX).  RTX LiDAR has never been run by the team on 6.0.1.
"""
from __future__ import annotations

import time

import numpy as np

GOLDEN = 0.6180339887
COST_US_ASSUMED = 20.0     # per PhysX raycast from python, before the first measurement (estimate)

PROFILES = {
    "small": {"n_rays": 96, "channels": 16, "elev": (-4.0, -80.0), "max_range": 40.0, "budget_ms": None,
              "min_rays": 96, "max_rays": 96, "voxel": 0.15, "cap_per_pile": 150_000,
              "rtx_configs": ("Example_Rotary",)},
    "heavy": {"n_rays": 192, "channels": 32, "elev": (-2.0, -88.0), "max_range": 60.0, "budget_ms": 4.0,
              "min_rays": 96, "max_rays": 4096, "voxel": 0.10, "cap_per_pile": 400_000,
              # RTX LiDAR config names shipped with Isaac Sim 4.x/5.x (OS1 REV7/REV6 128ch 1024x10, Hesai XT32);
              # unverified on 6.0.1, tried in order, then the generic example.
              "rtx_configs": ("OS1_REV7_128ch10hz1024res", "OS1_REV6_128ch10hz1024res", "Hesai_XT32_SD10",
                              "HESAI_XT32_SD10", "Example_Rotary")},
}


def add_lidar_args(ap):
    """``--lidar-profile`` (+ ``--lidar-budget-ms``) for run_mission's parser."""
    ap.add_argument("--lidar-profile", choices=sorted(PROFILES), default="heavy",
                    help="heavy: PhysX rays sized to a frame budget + dense RTX config if --rtx-lidar; small: 96 rays")
    ap.add_argument("--lidar-budget-ms", type=float, default=None,
                    help="heavy profile: PhysX LiDAR time per surveying drone per frame (default 4 ms)")
    return ap


def profile(name="heavy", budget_ms=None, rays=None, max_range=None):
    """Profile dict with optional overrides (an explicit ``--rays`` pins the count: no auto-sizing)."""
    p = dict(PROFILES[name])
    if budget_ms is not None and p["budget_ms"] is not None:
        p["budget_ms"] = float(budget_ms)
    if rays is not None and rays != p["n_rays"]:
        p.update(n_rays=int(rays), min_rays=int(rays), max_rays=int(rays), budget_ms=None)
    if max_range is not None:
        p["max_range"] = float(max_range)
    return p


def make_lidar(kind, prof, cast=None, grid=None, seed=0):
    """``kind`` = "physx" (needs ``cast``) or "heightmap" (needs ``grid``)."""
    kw = {k: prof[k] for k in ("channels", "elev")}
    if kind == "physx":
        return PhysxLidar(cast, prof["max_range"], prof["n_rays"], seed=seed, budget_ms=prof["budget_ms"],
                          min_rays=prof["min_rays"], max_rays=prof["max_rays"], **kw)
    return HeightmapLidar(grid, prof["max_range"], prof["n_rays"], seed=seed, **kw)


def ray_dirs(n_rays=96, frame=0, yaw_deg=0.0, elev=(-4.0, -80.0), channels=16):
    """Unit ray directions for one frame of a spinning multi-channel LiDAR (world frame).

    ``channels`` elevation rings from elev[0] to elev[1]; n_rays // channels azimuth columns spread evenly,
    the whole pattern rotating by a golden-ratio turn each frame so successive frames fill the circle.
    """
    channels = max(1, min(channels, n_rays))
    cols = max(1, n_rays // channels)
    el = np.radians(np.linspace(elev[0], elev[1], channels))
    base = 2 * np.pi * ((frame * GOLDEN) % 1.0) + np.radians(yaw_deg)
    az = base + 2 * np.pi * np.arange(cols) / cols
    A, E = np.meshgrid(az, el)
    A = A + (np.arange(channels)[:, None] * 0.37)       # stagger channels so columns are not radial lines
    ce = np.cos(E)
    return np.stack([ce * np.cos(A), ce * np.sin(A), np.sin(E)], -1).reshape(-1, 3)


class HeightmapLidar:
    """Ray-march against a heightmap Grid (sim.world.Grid). Pure numpy."""

    def __init__(self, grid, max_range=40.0, n_rays=96, noise=0.01, seed=0, channels=16, elev=(-4.0, -80.0)):
        self.grid, self.max_range, self.n_rays, self.noise = grid, max_range, n_rays, noise
        self.channels, self.elev = channels, elev
        self.step = grid.res * 0.5
        self.rng = np.random.default_rng(seed)

    def sweep(self, origin, yaw_deg=0.0, frame=0):
        o = np.asarray(origin, float)
        d = ray_dirs(self.n_rays, frame, yaw_deg, self.elev, self.channels)
        ts = np.arange(self.step, self.max_range + 1e-9, self.step)
        P = o[None, None, :] + d[:, None, :] * ts[None, :, None]           # (rays, steps, 3)
        below = P[..., 2] <= self.grid.height_at(P[..., 0], P[..., 1])
        hit = below.any(1)
        if not hit.any():
            return np.zeros((0, 3), np.float32)
        k = below.argmax(1)[hit]
        dh = d[hit]
        lo = np.where(k > 0, ts[np.maximum(k - 1, 0)], 0.0)
        hi = ts[k]
        for _ in range(6):                                                 # bisection refine
            mid = 0.5 * (lo + hi)
            p = o + dh * mid[:, None]
            inside = p[:, 2] <= self.grid.height_at(p[:, 0], p[:, 1])
            hi = np.where(inside, mid, hi)
            lo = np.where(inside, lo, mid)
        r = hi + self.rng.normal(0, self.noise, hi.shape)
        return (o + dh * r[:, None]).astype(np.float32)


class PhysxLidar:
    """Rays through PhysX scene queries (``compat.physx_raycaster()``). ~100 rays/drone/frame is cheap.
    Verified API (6.0.1): ``raycast_closest`` hits only colliders, only while simulating, and only with scene-query
    support on (Isaac Lab: ``SimulationCfg(enable_scene_query_support=True)``), else every ray silently misses.

    With ``budget_ms`` the ray count adapts: each sweep is timed, the per-ray cost (``cost_us``, EMA, seeded with
    ``COST_US_ASSUMED``) is updated and the next sweep casts ``budget_ms / cost_us`` rays (``min_rays..max_rays``).
    Directions are generated in one numpy call and handed over as plain tuples (the verified fast form)."""

    def __init__(self, cast, max_range=40.0, n_rays=96, noise=0.01, seed=0, channels=16, elev=(-4.0, -80.0),
                 budget_ms=None, min_rays=96, max_rays=4096):
        self.cast, self.max_range, self.n_rays, self.noise = cast, max_range, n_rays, noise
        self.channels, self.elev = channels, elev
        self.budget_ms, self.min_rays, self.max_rays = budget_ms, min_rays, max_rays
        self.rng = np.random.default_rng(seed)
        self.misses = 0
        self.cost_us = COST_US_ASSUMED
        self.cost_measured = False
        self.rays_cast = 0
        if budget_ms:
            self.n_rays = self._rays_for_budget()

    def _rays_for_budget(self):
        return int(np.clip(self.budget_ms * 1000.0 / max(self.cost_us, 0.1), self.min_rays, self.max_rays))

    def sweep(self, origin, yaw_deg=0.0, frame=0):
        o = tuple(float(v) for v in origin)
        dirs = ray_dirs(self.n_rays, frame, yaw_deg, self.elev, self.channels).tolist()
        cast, rng = self.cast, self.max_range
        t0 = time.perf_counter()
        out = [p for p in (cast(o, d, rng) for d in dirs) if p is not None]
        dt_us = (time.perf_counter() - t0) * 1e6
        self.rays_cast += len(dirs)
        if dirs:
            per = dt_us / len(dirs)
            self.cost_us = per if not self.cost_measured else 0.8 * self.cost_us + 0.2 * per
            self.cost_measured = True
            if self.budget_ms:
                self.n_rays = self._rays_for_budget()
        if not out:
            self.misses += 1
            return np.zeros((0, 3), np.float32)
        pts = np.asarray(out, float)
        if self.noise:
            v = pts - np.asarray(o)
            r = np.linalg.norm(v, axis=1, keepdims=True)
            pts = np.asarray(o) + v * (1 + self.rng.normal(0, self.noise, r.shape) / np.maximum(r, 1e-6))
        return pts.astype(np.float32)

    def stats(self):
        return {"rays_per_sweep": self.n_rays, "cost_us_per_ray": round(self.cost_us, 2),
                "cost_measured": self.cost_measured, "budget_ms": self.budget_ms, "rays_cast": self.rays_cast}


class RtxLidar:
    """Optional RTX LiDAR per drone (Isaac 4.5/5.x command + annotator). Every call is guarded.

    UNVERIFIED across versions: the ``IsaacSensorCreateRtxLidar`` command, the annotator name and whether
    its points are already world-frame. The team never ran an RTX LiDAR; on 6.x the sensor API moved
    (``isaacsim.sensors.experimental.rtx``), so expect this to fall back there. If anything fails, ``ok`` is
    False and the caller uses PhysxLidar, which IS the verified route (PhysX scene queries).
    """

    ANNOTATORS = ("RtxSensorCpuIsaacCreateRTXLidarScanBuffer", "IsaacCreateRTXLidarScanBuffer")

    def __init__(self, drone_paths, config="Example_Rotary", configs=None, mount_z=0.35):
        """``configs``: config names tried in order (first that creates a prim wins), e.g.
        ``PROFILES["heavy"]["rtx_configs"]``; ``config`` alone keeps the old behaviour."""
        self.ok = False
        self.annot = {}
        self.config = None
        try:
            import omni.kit.commands
            import omni.replicator.core as rep
            from pxr import Gf
            for did, path in drone_paths.items():
                prim = None
                for cfg in (configs or (config,)):
                    try:
                        _, prim = omni.kit.commands.execute(
                            "IsaacSensorCreateRtxLidar", path="/Lidar", parent=path, config=cfg,
                            translation=Gf.Vec3d(0, 0, float(mount_z)), orientation=Gf.Quatd(1, 0, 0, 0))
                    except Exception as e:   # noqa: BLE001
                        print(f"[lidar] RTX config {cfg} failed ({type(e).__name__}: {e})")
                        prim = None
                    if prim is not None and prim.IsValid():
                        self.config = cfg
                        break
                if prim is None:
                    raise RuntimeError("no RTX LiDAR config could be created")
                rp = rep.create.render_product(prim.GetPath(), [1, 1], name=f"lidar_{did}")
                ann = None
                for name in self.ANNOTATORS:
                    try:
                        ann = rep.AnnotatorRegistry.get_annotator(name)
                        break
                    except Exception:
                        continue
                if ann is None:
                    raise RuntimeError("no RTX lidar point annotator")
                ann.attach([rp])
                self.annot[did] = ann
            self.ok = bool(self.annot)
        except Exception as e:   # noqa: BLE001 - any API drift means fall back
            print(f"[lidar] RTX LiDAR unavailable ({type(e).__name__}: {e}); using PhysX raycasts")
            self.ok = False

    def read(self, did):
        ann = self.annot.get(did)
        if ann is None:
            return np.zeros((0, 3), np.float32)
        try:
            data = ann.get_data()
            pts = data["data"] if isinstance(data, dict) else data
            pts = np.asarray(pts, np.float32).reshape(-1, 3)
            return pts[np.isfinite(pts).all(1)]
        except Exception:
            return np.zeros((0, 3), np.float32)


class PointAccumulator:
    """World-frame points per pile, voxel-thinned so long surveys stay bounded (one point per voxel).

    Vectorised: voxel keys are packed into int64 and kept sorted per pile; membership is a ``searchsorted``, so
    a dense sweep (10^4-10^5 points from RTX) costs milliseconds instead of a python loop per point."""

    _OFF = 1 << 20          # +-1M voxels per axis (10 km at 1 cm): plenty for the site

    def __init__(self, voxel=0.15, cap_per_pile=150000):
        self.voxel, self.cap = voxel, cap_per_pile
        self.chunks = {}
        self.keys = {}
        self.total = 0

    @classmethod
    def for_profile(cls, prof):
        return cls(voxel=prof["voxel"], cap_per_pile=prof["cap_per_pile"])

    def _pack(self, pts):
        v = np.floor(pts / self.voxel).astype(np.int64) + self._OFF
        return (v[:, 0] << 42) | (v[:, 1] << 21) | v[:, 2]

    def add(self, pile, pts):
        if pts is None or len(pts) == 0:
            return pts
        seen = self.keys.get(pile, np.zeros(0, np.int64))
        if len(seen) >= self.cap:
            return pts[:0]
        k = self._pack(np.asarray(pts, np.float64))
        uk, first = np.unique(k, return_index=True)               # first point of each new voxel
        if len(seen):
            i = np.minimum(np.searchsorted(seen, uk), len(seen) - 1)
            fresh = seen[i] != uk
            uk, first = uk[fresh], first[fresh]
        room = self.cap - len(seen)
        if len(uk) > room:
            uk, first = uk[:room], first[:room]
        first = np.sort(first)
        new = pts[first]
        if len(new):
            self.keys[pile] = np.union1d(seen, uk)
            self.chunks.setdefault(pile, []).append(new)
            self.total += len(new)
        return new

    def points(self, pile=None):
        piles = [pile] if pile is not None else list(self.chunks)
        arrs = [c for p in piles for c in self.chunks.get(p, [])]
        return np.concatenate(arrs).astype(np.float32) if arrs else np.zeros((0, 3), np.float32)

    def count(self, pile):
        return sum(len(c) for c in self.chunks.get(pile, []))


if __name__ == "__main__":   # python -m isaac.lidar [--lidar-profile heavy]: dry-run sweep + accumulator throughput
    import argparse
    ap = add_lidar_args(argparse.ArgumentParser())
    a = ap.parse_args()
    from sim.scenario import make_scenario
    sc = make_scenario(7)
    hm = sc.heightmap(0.5)
    prof = profile(a.lidar_profile, a.lidar_budget_ms)
    p = sc.piles[4]
    lid = make_lidar("heightmap", prof, grid=hm)
    acc = PointAccumulator.for_profile(prof)
    t0 = time.perf_counter()
    n = 0
    for f in range(60):
        pts = lid.sweep((p.x + 10, p.y, 12.0), 3 * f, f)
        n += len(acc.add(p.name, pts))
    dt = time.perf_counter() - t0
    print({"profile": a.lidar_profile, "rays_per_frame": lid.n_rays, "frames": 60, "kept_points": n,
           "ms_per_frame_numpy_heightmap": round(1000 * dt / 60, 2), **{k: prof[k] for k in ("budget_ms", "voxel")}})
