"""SYNTHETIC radar / acoustic observation generator.

*** THIS MODULE FABRICATES SENSOR DATA FROM HIDDEN GROUND TRUTH. ***
It is the ONLY non-evaluation module allowed to read ``World.targets`` (see the
ground-truth boundary in contracts/INTERFACES.md).  Its outputs follow the
radar / acoustic observation schemas exactly; in the real system the same
dicts come from the drones.  The coordinator, localiser and planner only ever
see those dicts.

With ``radar='dsp'`` (``DspObservationGenerator``) the radar observations instead come out of the
synthetic raw-signal generator + real DSP chain in radar_dsp.py; this module only turns hidden truth
into the scene description the generator renders.

Effects modelled (default 'model' radar): range noise scaled with (1-quality) and the length of the
slant path through rubble, detection dropout growing with range / rubble /
poor quality, quality loss from rough or sloped landings and short dwell,
occasional interference, multipath-biased (+1..3 m) and clutter false returns,
and a moving distractor that yields separate ``doppler_moving`` observations.
"""
from __future__ import annotations

import numpy as np

from . import acoustic_model, radar_dsp
from .sensor_model import SCAN_DWELL_S, detection_prob, range_sigma, rubble_path, scan_quality


class ObservationGenerator:
    """Stand-in for the drones' radar and microphone front-ends."""

    def __init__(self, world, seed: int = 0):
        self.world = world
        self.seed = seed
        self._buried = next((t for t in world.targets if t["kind"] == "buried"), None)   # None: nobody here
        self._moving = [t for t in world.targets if t.get("moving")]

    def _rng(self, stream: str, idx: int):
        # per-observation stream so that policies compared on a seed see identical noise per scan index
        return np.random.default_rng([self.seed, {"radar": 1, "acoustic": 2}[stream], idx])

    # ------------------------------------------------------- landing quality
    @staticmethod
    def _landing_quality(rng, site: dict, dwell: float) -> float:
        """Scan quality in [0.03, 0.97] from the actual touchdown (planned patch x what happened)."""
        true_rough = site["roughness"] * (radar_dsp.PERCH_ROUGH_FACTOR if site.get("perch") else 1.0) \
            * float(np.exp(rng.normal(0, 0.3)))
        # the 0.5 m heightmap hides loose debris: sometimes the touchdown is much rougher than mapped
        p_upset = 0.08 + (0.17 if site["z"] > 0.8 else 0.0)
        if rng.uniform() < p_upset:
            true_rough = max(true_rough, 0.02) * float(rng.uniform(2.0, 4.0))
        true_slope = max(0.0, site["slope_deg"] * float(np.exp(rng.normal(0, 0.2))) - radar_dsp.LEVEL_DEG)  # levelled mount
        quality = scan_quality(true_rough, true_slope, dwell) * float(rng.uniform(0.9, 1.05))
        if rng.uniform() < 0.06:  # gust / interference during the dwell
            quality *= float(rng.uniform(0.3, 0.6))
        quality = float(np.clip(quality, 0.03, 0.97))
        return quality

    # ------------------------------------------------------------------ radar
    def radar_scan(self, scan_idx: int, drone_id: str, sensor_pose, site: dict, t: float,
                   dwell: float = SCAN_DWELL_S):
        """Return [stationary_obs] (+ [moving_obs] if the distractor is detected)."""
        rng = self._rng("radar", scan_idx)
        grid = self.world.grid
        ps = np.array(sensor_pose[:3], float)

        quality = self._landing_quality(rng, site, dwell)

        # --- stationary buried target (if any)
        cands = []
        true_range = None
        if self._buried is not None:
            pt = self.world.target_position(self._buried, t)
            true_range = float(np.linalg.norm(pt - ps))
            path = rubble_path(grid, ps, pt)
        if true_range is not None and rng.uniform() < float(detection_prob(true_range, quality, 0.0)
                                                             * radar_dsp.reach_prob(true_range, path, quality)):
            sig = float(range_sigma(quality, path))
            conf = float(np.clip(0.45 + 0.5 * quality + rng.normal(0, 0.07), 0.2, 0.98))
            cands.append((max(0.5, true_range + rng.normal(0, sig)), conf))
            if rng.uniform() < 0.22:  # multipath copy, biased long
                cands.append((true_range + rng.uniform(1.0, 3.0), float(rng.uniform(0.2, 0.5))))
        if rng.uniform() < 0.08 + 0.35 * (1 - quality):  # clutter / false periodic return
            cands.append((float(rng.uniform(4.0, 16.0)), float(rng.uniform(0.15, 0.4))))
        cands.sort(key=lambda c: -c[1])
        hz = None
        if cands and true_range is not None and any(abs(c[0] - true_range) < 1.0 for c in cands):
            hz = round(float(self._buried["breathing_hz"] + rng.normal(0, 0.01)), 3)
        elif cands:
            hz = round(float(rng.uniform(0.2, 0.33)), 3)
        stat = {
            "scan_id": f"s{scan_idx}", "drone_id": drone_id,
            "sensor_pose": [round(float(v), 2) for v in sensor_pose], "timestamp": round(float(t), 1),
            "range_candidates": [round(c[0], 2) for c in cands],
            "confidence": [round(c[1], 2) for c in cands], "quality": round(quality, 2),
            "breathing_hz": hz, "doppler_moving": False,
            "max_reliable_depth_m": round(radar_dsp.reach_path_m(radar_dsp.REACH_REF_RANGE_M, quality), 2),
        }
        out = [stat]

        # --- moving distractor -> separate doppler_moving observation
        for mv in self._moving:
            pm = self.world.target_position(mv, t)
            rm = float(np.linalg.norm(pm - ps))
            if rng.uniform() < float(detection_prob(rm, quality, 0.0)) * 0.9:
                out.append({
                    "scan_id": f"s{scan_idx}m", "drone_id": drone_id,
                    "sensor_pose": stat["sensor_pose"], "timestamp": stat["timestamp"],
                    "range_candidates": [round(rm + rng.normal(0, 0.25), 2)],
                    "confidence": [round(float(rng.uniform(0.5, 0.85)), 2)], "quality": round(quality, 2),
                    "breathing_hz": None, "doppler_moving": True,
                })
        return out

    # --------------------------------------------------------------- acoustic
    def acoustic_listen(self, obs_idx: int, drone_id: str, origin_xy, t: float):
        rng = self._rng("acoustic", obs_idx)
        tgt = self._buried
        if tgt is None:
            return None
        return acoustic_model.synthesize_doa(rng, f"a{obs_idx}", drone_id, origin_xy, t, tgt["xy"],
                                             depth_m=tgt["depth"], emit_prob=0.8 if tgt["emits_sound"] else 0.0)

    # ------------------------------------------------- display-only waveform
    @staticmethod
    def slow_time_waveform(breathing_hz: float, quality: float, duration: float = 20.0, fs: float = 20.0,
                           seed: int = 0):
        """Synthetic radar slow-time phase (rad): breathing sinusoid + drift + noise. For display only."""
        rng = np.random.default_rng(seed)
        t = np.arange(0, duration, 1 / fs)
        amp = 0.8 * quality
        phase = amp * np.sin(2 * np.pi * breathing_hz * t + rng.uniform(0, 2 * np.pi))
        phase += 0.15 * amp * np.sin(2 * np.pi * 2.3 * breathing_hz * t)  # harmonic
        phase += 0.05 * t / duration + rng.normal(0, 0.12 + 0.35 * (1 - quality), t.size)
        return t, phase


class DspObservationGenerator(ObservationGenerator):
    """Radar observations from SYNTHETIC raw IF signals + the real DSP chain (sim/radar_dsp.py).

    *** FABRICATES SENSOR DATA FROM HIDDEN GROUND TRUTH. ***  Truth is used only to place the
    reflectors of the synthetic IF generator; everything the coordinator sees (ranges, confidence,
    quality, breathing rate, doppler flag) is measured from the signal by ``radar_dsp.process_if``.
    The acoustic path is unchanged.
    """

    def __init__(self, world, seed: int = 0, params=None, cfg=None, fan: bool = False):
        super().__init__(world, seed)
        self.params = params or radar_dsp.RadarParams()
        self.cfg = cfg
        self.fan = fan                 # add a 1.2 Hz non-human periodic source (distractor demo)
        self.last_result = None        # ScanResult of the last scan (debug arrays), never read by the planner

    def build_scene(self, rng, sensor_pose, site: dict, t: float, quality: float, dwell: float):
        P = self.params
        ps = np.array(sensor_pose[:3], float)
        scene = radar_dsp.Scene(quality=quality)
        scene.clutter = radar_dsp.default_clutter(rng, max_range=min(22.0, P.max_range - 2.0))
        if self._buried is not None:
            pt = self.world.target_position(self._buried, t)
            r = float(np.linalg.norm(pt - ps))
            path = rubble_path(self.world.grid, ps, pt)
            amp = radar_dsp.reflection_amplitude(r, path, radar_dsp.CHEST_RCS_DB)
            hz = float(self._buried["breathing_hz"])
            mm = float(rng.uniform(4.0, 8.0))
            scene.breathers.append(radar_dsp.Breather(r, amp, hz, mm, irregularity=0.03))
            if rng.uniform() < 0.22:   # multipath copy of the chest return, delayed 1-3 m, weaker
                scene.breathers.append(radar_dsp.Breather(r + float(rng.uniform(1.0, 3.0)), amp * float(rng.uniform(0.3, 0.5)),
                                                          hz, mm, irregularity=0.03))
        frames = t - dwell + P.t_slow
        for mv in self._moving:
            track = np.array([np.linalg.norm(self.world.target_position(mv, tt) - ps) for tt in frames])
            scene.movers.append(radar_dsp.Mover(track, radar_dsp.reflection_amplitude(track.mean(), 0.0, radar_dsp.PERSON_RCS_DB)))
        if self.fan:
            rf = float(rng.uniform(6.0, 16.0))
            scene.vibrators.append(radar_dsp.Vibrator(rf, radar_dsp.reflection_amplitude(rf, 0.0, 24.0), 1.2, 3.0))
        return scene

    def radar_scan(self, scan_idx: int, drone_id: str, sensor_pose, site: dict, t: float,
                   dwell: float = SCAN_DWELL_S):
        rng = self._rng("radar", scan_idx)
        quality = self._landing_quality(rng, site, dwell)
        params = self.params if abs(dwell - self.params.dwell) < 1e-6 else \
            radar_dsp.RadarParams(**{**self.params.__dict__, "dwell": float(dwell)})
        rng_sig = np.random.default_rng([self.seed, 3, scan_idx])
        scene = self.build_scene(rng_sig, sensor_pose, site, t, quality, dwell)
        iq = radar_dsp.simulate_if(scene, params, rng_sig)
        obs, self.last_result = radar_dsp.radar_observations(iq, f"s{scan_idx}", drone_id, sensor_pose, t,
                                                             params, self.cfg, debug=True)
        return obs


def make_generator(world, seed: int, radar: str = "model", **kw):
    """Factory used by the coordinator: radar='model' (default, direct noise model) or 'dsp'."""
    if radar == "dsp":
        return DspObservationGenerator(world, seed, **kw)
    return ObservationGenerator(world, seed)
