"""Spinning LiDAR for the surveying drones.

* ``PhysxLidar``     rays through ``compat.physx_raycaster`` (hits the scene.py colliders). Isaac only.
* ``HeightmapLidar`` numpy ray-march against ``Scenario.heightmap()``; used by ``--dry-run`` (no omni/pxr).
* ``RtxLidar``       optional ``--rtx-lidar`` path (RTX sensor + annotator); guarded, falls back to PhysX.

All return world-frame (N, 3) float32 hit points; ``PointAccumulator`` keeps them per pile.
"""
from __future__ import annotations

import numpy as np

GOLDEN = 0.6180339887


def ray_dirs(n_rays=96, frame=0, yaw_deg=0.0, elev=(-4.0, -80.0), channels=16):
    """Unit ray directions for one frame of a spinning multi-channel LiDAR (world frame).

    ``channels`` elevation rings from elev[0] to elev[1]; n_rays // channels azimuth columns spread evenly,
    the whole pattern rotating by a golden-ratio turn each frame so successive frames fill the circle.
    """
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

    def __init__(self, grid, max_range=40.0, n_rays=96, noise=0.01, seed=0):
        self.grid, self.max_range, self.n_rays, self.noise = grid, max_range, n_rays, noise
        self.step = grid.res * 0.5
        self.rng = np.random.default_rng(seed)

    def sweep(self, origin, yaw_deg=0.0, frame=0):
        o = np.asarray(origin, float)
        d = ray_dirs(self.n_rays, frame, yaw_deg)
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
    """Rays through PhysX scene queries (``compat.physx_raycaster()``). ~100 rays/drone/frame is cheap."""

    def __init__(self, cast, max_range=40.0, n_rays=96, noise=0.01, seed=0):
        self.cast, self.max_range, self.n_rays, self.noise = cast, max_range, n_rays, noise
        self.rng = np.random.default_rng(seed)
        self.misses = 0

    def sweep(self, origin, yaw_deg=0.0, frame=0):
        o = np.asarray(origin, float)
        out = []
        for d in ray_dirs(self.n_rays, frame, yaw_deg):
            p = self.cast(o, d, self.max_range)
            if p is not None:
                out.append(p)
        if not out:
            self.misses += 1
            return np.zeros((0, 3), np.float32)
        pts = np.asarray(out, float)
        if self.noise:
            v = pts - o
            r = np.linalg.norm(v, axis=1, keepdims=True)
            pts = o + v * (1 + self.rng.normal(0, self.noise, r.shape) / np.maximum(r, 1e-6))
        return pts.astype(np.float32)


class RtxLidar:
    """Optional RTX LiDAR per drone (Isaac 4.5/5.x command + annotator). Every call is guarded.

    UNVERIFIED across versions: the ``IsaacSensorCreateRtxLidar`` command, the annotator name and whether
    its points are already world-frame. If anything fails, ``ok`` is False and the caller uses PhysxLidar.
    """

    ANNOTATORS = ("RtxSensorCpuIsaacCreateRTXLidarScanBuffer", "IsaacCreateRTXLidarScanBuffer")

    def __init__(self, drone_paths, config="Example_Rotary"):
        self.ok = False
        self.annot = {}
        try:
            import omni.kit.commands
            import omni.replicator.core as rep
            from pxr import Gf
            for did, path in drone_paths.items():
                _, prim = omni.kit.commands.execute(
                    "IsaacSensorCreateRtxLidar", path="/Lidar", parent=path, config=config,
                    translation=Gf.Vec3d(0, 0, 0.35), orientation=Gf.Quatd(1, 0, 0, 0))
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
    """World-frame points per pile, voxel-thinned so long surveys stay bounded."""

    def __init__(self, voxel=0.15, cap_per_pile=150000):
        self.voxel, self.cap = voxel, cap_per_pile
        self.chunks = {}
        self.keys = {}
        self.total = 0

    def add(self, pile, pts):
        if pts is None or len(pts) == 0:
            return pts
        seen = self.keys.setdefault(pile, set())
        if len(seen) >= self.cap:
            return pts[:0]
        vk = np.floor(pts / self.voxel).astype(np.int64)
        keep = []
        for i, k in enumerate(map(tuple, vk)):
            if k not in seen:
                seen.add(k)
                keep.append(i)
        new = pts[keep]
        if len(new):
            self.chunks.setdefault(pile, []).append(new)
            self.total += len(new)
        return new

    def points(self, pile=None):
        piles = [pile] if pile is not None else list(self.chunks)
        arrs = [c for p in piles for c in self.chunks.get(p, [])]
        return np.concatenate(arrs).astype(np.float32) if arrs else np.zeros((0, 3), np.float32)

    def count(self, pile):
        return sum(len(c) for c in self.chunks.get(pile, []))
