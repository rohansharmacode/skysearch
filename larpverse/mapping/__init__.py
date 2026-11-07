"""LiDAR SLAM + mapping for the rubble-grid scenario (SYNTHETIC sensor, real KISS-ICP registration).

    from mapping import build_pile_map
    pts, grid, metrics = build_pile_map(scenario, "B2", seed=7)

``pts`` is the voxel-downsampled SLAM cloud (float32 Nx3, site frame), ``grid`` a ``sim.world.Grid``
heightmap rasterised from it, ``metrics`` a dict (ATE, map error, landing-cell agreement, runtime).
"""
from .slam import build_pile_map  # noqa: F401
