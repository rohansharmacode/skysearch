"""SYNTHETIC acoustic direction-of-arrival (DOA) observation generator.

Pure function of an emitter position: it never touches the World.  The caller
(radar_model.ObservationGenerator, the only module allowed to read hidden
targets) supplies the source position.  Bearings follow the INTERFACES.md
convention: degrees counter-clockwise from +x (east).
"""
from __future__ import annotations

import numpy as np

AUDIBLE_RANGE_M = 13.0
CLASSES = ["knocking", "tapping", "voice", "shout", "unknown"]
CLASS_P = [0.35, 0.30, 0.20, 0.05, 0.10]


def synthesize_doa(rng, obs_id, drone_id, origin_xy, timestamp, source_xy, depth_m=1.5,
                   emit_prob=0.8):
    """Return an acoustic observation dict, or None if nothing is heard."""
    ox, oy = origin_xy
    dx, dy = source_xy[0] - ox, source_xy[1] - oy
    d = float(np.hypot(dx, dy))
    p_hear = emit_prob * np.exp(-(d / AUDIBLE_RANGE_M) ** 2) * np.exp(-depth_m / 8.0)
    if rng.uniform() < 0.04:  # wind / machinery false alarm: random bearing, low confidence
        return {"obs_id": obs_id, "drone_id": drone_id, "origin": [round(ox, 2), round(oy, 2)],
                "timestamp": round(timestamp, 1), "bearing_deg": round(float(rng.uniform(-180, 180)), 1),
                "width_deg": 60.0, "class": "unknown", "confidence": round(float(rng.uniform(0.1, 0.3)), 2)}
    if rng.uniform() > p_hear:
        return None
    sigma = 6.0 + 0.9 * d
    bearing = np.degrees(np.arctan2(dy, dx)) + rng.normal(0, sigma)
    bearing = (bearing + 180.0) % 360.0 - 180.0
    width = float(np.clip(2.5 * sigma, 20.0, 70.0))
    conf = float(np.clip(0.85 * np.exp(-(d / 18.0) ** 2) + rng.normal(0, 0.07), 0.2, 0.95))
    cls = str(rng.choice(CLASSES, p=CLASS_P))
    return {"obs_id": obs_id, "drone_id": drone_id, "origin": [round(ox, 2), round(oy, 2)],
            "timestamp": round(timestamp, 1), "bearing_deg": round(float(bearing), 1),
            "width_deg": round(width, 1), "class": cls, "confidence": round(conf, 2)}
