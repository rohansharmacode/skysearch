"""Drone platform specs: airframe, battery/endurance, sensors, landing footprint (one source of truth).

User decision (Nov 2026): the fleet uses **big** drones, agricultural / heavy-lift class (DJI FlyCart 30 /
Agras T50-class coaxial X8) or Matrice 400-class industrial, not small FPV frames.  A bigger airframe buys
endurance, payload for a dense LiDAR, a lower-frequency radar with a larger antenna, a wide mic array and a
loudspeaker; it costs a much bigger landing patch, strong downwash and a heavy load on unstable rubble.

    from sim.platform import get_platform
    spec = get_platform("heavy_sar")          # default; also "industrial_sar", "legacy_small"
    spec.flight_kwargs()                      # KinematicFlight speeds + battery rates
    spec.landing_limits()                     # landing.py thresholds for this footprint
    spec.lidar.pts_per_s, spec.radar.rx_gain_dbi(0.6)

Figures come from manufacturer spec pages (``sources``; docs/PLATFORM.md has the table).  Anything not on a
spec page is in ``estimates`` and is labelled "estimate" in the docs.  Nothing here claims a landing is
safe: the landing limits are screening thresholds for a candidate patch, not a structural assessment.

Pure python + math (no numpy needed); importable everywhere, including Isaac's python.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

C = 299_792_458.0
RHO_AIR = 1.225            # kg/m^3, sea level
G0 = 9.81


# --------------------------------------------------------------------------- sensors
@dataclass(frozen=True)
class LidarSpec:
    model: str
    channels: int
    pts_per_s: float               # single return
    rate_hz: float                 # spin / frame rate
    hfov_deg: float
    vfov_deg: tuple                # (min, max) elevation in the sensor frame
    h_res_deg: float               # azimuth step at rate_hz
    range_10pct_m: float           # range on 10% reflectivity targets
    range_sigma_m: float           # 1-sigma range precision
    weight_kg: float
    mount_tilt_deg: float = 90.0   # spin axis pitched to body +x: scan plane across track, looks down + sideways
    mount_z_m: float = 0.0         # sensor height above the mission pose point (skid level + LAND_HEIGHT)
    spinning: bool = True

    @property
    def beams_per_sweep(self) -> int:
        return int(round(self.pts_per_s / self.rate_hz))


@dataclass(frozen=True)
class RadarSpec:
    band_ghz: tuple                # (f_lo, f_hi), stepped-frequency / UWB bio-radar
    aperture_m: tuple              # (w, h) of the antenna panel
    weight_kg: float
    efficiency: float = 0.5        # aperture efficiency (estimate, typical for horns / Vivaldi arrays)

    def rx_gain_dbi(self, f_ghz: float) -> float:
        """Aperture gain G = eff * 4 pi A / lambda^2, floored at 0 dBi (electrically small antenna).
        Only the RECEIVE gain counts as a platform advantage: UWB emission is EIRP-limited (e.g. FCC
        47 CFR 15.509/15.510), so a higher-gain transmit antenna must back off its power."""
        lam = C / (f_ghz * 1e9)
        a = self.aperture_m[0] * self.aperture_m[1]
        return 10 * math.log10(max(1.0, self.efficiency * 4 * math.pi * a / lam ** 2))

    def beamwidth_deg(self, f_ghz: float) -> float:
        """~ lambda / D half-power beamwidth along the larger side (capped at 180)."""
        lam = C / (f_ghz * 1e9)
        return min(180.0, math.degrees(lam / max(self.aperture_m)))


@dataclass(frozen=True)
class MicArraySpec:
    n_mics: int
    aperture_m: float              # ring diameter
    weight_kg: float

    def beamwidth_deg(self, f_hz: float = 1000.0) -> float:
        """~ lambda / D angular resolution of a delay-and-sum beam (capped at 180)."""
        return min(180.0, math.degrees(343.0 / f_hz / self.aperture_m))


@dataclass(frozen=True)
class SpeakerSpec:
    model: str
    spl_db_1m: float
    band_hz: tuple
    weight_kg: float

    def spl_at(self, r_m: float) -> float:
        """Free-field spherical spreading only (no rubble, no air absorption)."""
        return self.spl_db_1m - 20 * math.log10(max(r_m, 1.0))


@dataclass(frozen=True)
class ThermalSpec:
    model: str
    resolution: tuple
    weight_kg: float


# --------------------------------------------------------------------------- platform
@dataclass(frozen=True)
class PlatformSpec:
    name: str
    airframe: str
    layout: str                    # "coaxial-X8", "quad-X"
    n_rotors: int
    span_m: float                  # tip-to-tip with propellers unfolded (largest horizontal dimension)
    wheelbase_m: float             # diagonal motor-to-motor
    prop_d_m: float
    empty_mass_kg: float           # airframe incl. batteries, no payload
    payload_kg: float              # SAR payload carried (sensors + compute + mounts)
    mtow_kg: float
    battery_wh: float
    endurance_min: float           # hover-equivalent flight time with the SAR payload, to 0%
    cruise_mps: float
    climb_mps: float
    descend_mps: float
    cruise_alt_m: float            # above the pads; KinematicFlight adds terrain clearance
    terrain_clear_m: float         # min height over terrain on a leg
    max_wind_mps: float
    ip_rating: str
    idle_power_w: float            # rotors off, radar/LiDAR/compute on (perched scan / listen)
    gear_footprint_m: tuple        # landing-gear contact rectangle (L, W)
    lidar: LidarSpec
    radar: RadarSpec
    mics: MicArraySpec
    speaker: SpeakerSpec | None = None
    thermal: ThermalSpec | None = None
    sources: dict = field(default_factory=dict)
    estimates: tuple = ()
    notes: tuple = ()

    # ----------------------------------------------------------------- derived: mass, downwash
    @property
    def takeoff_mass_kg(self) -> float:
        return self.empty_mass_kg + self.payload_kg

    def disk_area_m2(self) -> float:
        """Coaxial pairs share one disk; a quad-X has one disk per rotor."""
        disks = self.n_rotors // 2 if self.layout.startswith("coaxial") else self.n_rotors
        return disks * math.pi * (self.prop_d_m / 2) ** 2

    def downwash_mps(self) -> float:
        """Momentum-theory induced velocity at the rotor disk in hover, v_i = sqrt(T / (2 rho A)).
        The fully developed wake below is ~2 v_i.  Big and small drones have similar v_i (disk loading scales
        with size); the difference is the wake: ~3 m wide and ~18x the thrust (momentum flux) for heavy_sar."""
        return math.sqrt(self.takeoff_mass_kg * G0 / (2 * RHO_AIR * self.disk_area_m2()))

    def static_load_n(self) -> float:
        """Weight on the patch after touchdown (rotors off). What matters on rubble is this load on a slab
        that may be cantilevered over a void, not a contact pressure."""
        return self.takeoff_mass_kg * G0

    # ----------------------------------------------------------------- derived: battery / flight model
    @property
    def fly_rate(self) -> float:
        """Battery fraction per second of flight (hover-equivalent; cruise at ~10 m/s is usually no worse)."""
        return 1.0 / (self.endurance_min * 60.0)

    @property
    def idle_rate(self) -> float:
        """Battery fraction per second while perched with rotors off (scan / listen)."""
        return self.idle_power_w / (self.battery_wh * 3600.0)

    def flight_kwargs(self) -> dict:
        """Keyword arguments for ``sim.coordinator.KinematicFlight`` (see docs/PLATFORM.md Requests)."""
        return {"cruise_alt": self.cruise_alt_m, "speed": self.cruise_mps, "climb": self.climb_mps,
                "descend": self.descend_mps, "fly_rate": round(self.fly_rate, 6),
                "idle_rate": round(self.idle_rate, 7), "terrain_clear": self.terrain_clear_m}

    # ----------------------------------------------------------------- derived: landing footprint
    def landing_limits(self, res: float = 0.5) -> dict:
        """Screening thresholds for ``sim.landing`` on a ``res`` heightmap.

        patch_cells : odd plane-fit window covering the gear footprint + 2 x 0.5 m touchdown error
        clear_m     : prop-tip radius + 0.4 m (obstacles > OBSTACLE_H above the plane; legacy -> 0.9 m)
        step_r_m    : radius inside which |height - plane| <= step_m (the gear footprint)
        perch       : whether perching on the rubble (vs flat ground beside it) is allowed at all
        """
        need = max(self.gear_footprint_m) + 1.0          # +-0.5 m touchdown position error
        cells = max(3, int(math.ceil(need / res)) | 1)
        heavy = self.takeoff_mass_kg > 25.0
        return {
            "patch_cells": cells, "patch_m": round(cells * res, 2),
            "clear_m": round(self.span_m / 2 + 0.4, 2),
            "step_r_m": round(max(self.gear_footprint_m) / 2 + 0.75, 2),
            "step_m": 0.20 if heavy else 0.35,
            "slope_deg": 8.0 if heavy else 12.0,
            "perch_slope_deg": 10.0 if heavy else 20.0,
            "rough_m": 0.05 if heavy else 0.07,
            "perch": True,
            "perch_min_patch_m": round(cells * res, 2),
            "prefer": ("street-level / firm slab at the pile edge" if heavy else "nearest flat cell to the mound"),
        }


# --------------------------------------------------------------------------- presets
_SRC = {
    "flycart30": "https://www.dji.com/flycart-30/specs",
    "t50": "https://ag.dji.com/t50/specs",
    "m400": "https://enterprise.dji.com/matrice-400/specs",
    "m350": "https://enterprise.dji.com/matrice-350-rtk/specs",
    "os1": "https://ouster.com/products/hardware/os1-lidar-sensor",
    "xt32": "https://www.hesaitech.com/product/xt32/",
    "l2": "https://enterprise.dji.com/zenmuse-l2/specs",
    "mid360": "https://www.livoxtech.com/mid-360/specs",
    "v1": "https://enterprise.dji.com/zenmuse-v1/specs",
    "h30t": "https://enterprise.dji.com/zenmuse-h30-series/specs",
}

OS1_128 = LidarSpec("Ouster OS1-128 (1024x10 mode)", channels=128, pts_per_s=128 * 1024 * 10, rate_hz=10.0,
                    hfov_deg=360.0, vfov_deg=(-22.5, 22.5), h_res_deg=360 / 1024, range_10pct_m=90.0,
                    range_sigma_m=0.03, weight_kg=0.45, mount_tilt_deg=90.0, mount_z_m=0.25)
XT32 = LidarSpec("Hesai XT32", channels=32, pts_per_s=640_000, rate_hz=10.0, hfov_deg=360.0,
                 vfov_deg=(-16.0, 15.0), h_res_deg=0.18, range_10pct_m=80.0, range_sigma_m=0.01,
                 weight_kg=0.80, mount_tilt_deg=90.0, mount_z_m=0.20)
MID360_LIKE = LidarSpec("Livox Mid-360-class (sim abstraction: 32 ch, -18..-80 deg fan)", channels=32,
                        pts_per_s=32 * 720 * 10, rate_hz=10.0, hfov_deg=360.0, vfov_deg=(-80.0, -18.0),
                        h_res_deg=0.5, range_10pct_m=40.0, range_sigma_m=0.02, weight_kg=0.265,
                        mount_tilt_deg=0.0, mount_z_m=0.35)

HEAVY_SAR = PlatformSpec(
    name="heavy_sar",
    airframe="Heavy-lift coaxial X8, DJI FlyCart 30 / Agras T50 class (54-in props, 2.2 m wheelbase)",
    layout="coaxial-X8", n_rotors=8, span_m=3.085, wheelbase_m=2.20, prop_d_m=1.375,
    empty_mass_kg=65.0, payload_kg=12.0, mtow_kg=95.0, battery_wh=2 * 1984.4,
    endurance_min=24.6, cruise_mps=10.0, climb_mps=3.0, descend_mps=2.0, cruise_alt_m=12.0,
    terrain_clear_m=5.0, max_wind_mps=12.0, ip_rating="IP55", idle_power_w=250.0,
    gear_footprint_m=(1.6, 1.4),
    lidar=OS1_128,
    radar=RadarSpec(band_ghz=(0.4, 2.0), aperture_m=(0.60, 0.50), weight_kg=4.0),
    mics=MicArraySpec(n_mics=8, aperture_m=1.0, weight_kg=1.0),
    speaker=SpeakerSpec("DJI Zenmuse V1", spl_db_1m=129.0, band_hz=(800, 8000), weight_kg=0.69),
    thermal=ThermalSpec("DJI Zenmuse H30T (thermal channel)", (1280, 1024), 0.92),
    sources={"airframe dims, wheelbase, 54-in props, 8 rotors, 42.5 kg w/o battery, MTOW 95 kg, DB2000 1984.4 Wh x2, "
             "hover 29 min empty / 18 min at 30 kg, 12 m/s wind, IP55": _SRC["flycart30"],
             "same 2.2 m wheelbase / 54-in X8 frame (Agras T50: 2800x3085 mm unfolded)": _SRC["t50"],
             "LiDAR: 128 ch, 45 deg VFoV, 90 m @10%": _SRC["os1"],
             "speaker: 129 dB @1 m (on M400), 800 Hz-8 kHz, 690 g": _SRC["v1"],
             "thermal 1280x1024": _SRC["h30t"]},
    estimates=("empty_mass_kg 65 = MTOW 95 kg minus the 30 kg rated cargo (42.5 kg frame + two DB2000 packs)",
               "payload_kg 12 (OS1 0.45 + radar 4 + mics 1 + V1 0.69 + H30T 0.92 + compute 1.5 + gimbal/mounts 3.4)",
               "endurance_min 24.6: linear between the spec's 29 min (empty) and 18 min (30 kg) at 12 kg",
               "cruise 10 m/s (spec max 20 m/s), climb 3 / descend 2 m/s, cruise_alt 12 m",
               "idle_power_w 250 (radar ~40 W, LiDAR ~20 W, compute ~60 W, avionics/links ~130 W)",
               "gear_footprint 1.6 x 1.4 m", "radar panel 0.6 x 0.5 m, 4 kg", "mic ring 8 x MEMS, 1.0 m, 1 kg",
               "OS1 range sigma 3 cm, mount tilt 90 deg"),
    notes=("Downwash ~2x the disk value lifts dust and loose fragments: approach and land at the pile edge, never "
           "hover low over a suspected void.",
           "A ~77 kg drone on unstable rubble can shift debris onto a void: prefer street-level ground or a "
           "large firm slab at the pile edge; perch on the pile only on a >= 3 m slab that the map shows flat, "
           "and treat every landing cell as a candidate, not as structurally safe."),
)

INDUSTRIAL_SAR = PlatformSpec(
    name="industrial_sar",
    airframe="Industrial quad-X, DJI Matrice 400 class (25-in props, 1.07 m wheelbase)",
    layout="quad-X", n_rotors=4, span_m=1.705, wheelbase_m=1.07, prop_d_m=0.635,
    empty_mass_kg=9.74, payload_kg=4.0, mtow_kg=15.8, battery_wh=977.0,
    endurance_min=36.0, cruise_mps=10.0, climb_mps=3.0, descend_mps=2.0, cruise_alt_m=10.0,
    terrain_clear_m=4.0, max_wind_mps=12.0, ip_rating="IP55", idle_power_w=120.0,
    gear_footprint_m=(0.80, 0.70),
    lidar=XT32,
    radar=RadarSpec(band_ghz=(0.4, 2.0), aperture_m=(0.35, 0.35), weight_kg=1.5),
    mics=MicArraySpec(n_mics=8, aperture_m=0.8, weight_kg=0.4),
    speaker=SpeakerSpec("DJI Zenmuse V1", spl_db_1m=129.0, band_hz=(800, 8000), weight_kg=0.69),
    thermal=None,
    sources={"1070 mm wheelbase, 25-in props, 9.74 kg with battery, MTOW 15.8 kg, payload 6 kg, TB100 977 Wh, "
             "hover 53 min with H30T, 25 m/s, 12 m/s wind, IP55": _SRC["m400"],
             "LiDAR: 32 ch, 640k pts/s, 0.18 x 1 deg, 31 deg VFoV, +-1 cm / 0.5 cm, 0.8 kg": _SRC["xt32"],
             "speaker: 129 dB @1 m": _SRC["v1"]},
    estimates=("span 1.705 m = wheelbase + one prop diameter",
               "payload_kg 4 (XT32 0.8 + radar 1.5 + mics 0.4 + V1 0.69 + compute 0.6)",
               "endurance_min 36: spec 53 min hover at 10.66 kg scaled by (m / m_spec)^-1.5 to 13.7 kg",
               "cruise 10 m/s, climb 3 / descend 2 m/s", "idle_power_w 120", "gear 0.8 x 0.7 m",
               "radar 0.35 x 0.35 m", "XT32 range sigma 1 cm"),
    notes=("Fallback / complement: fits a 1.5-2 m patch, much weaker downwash; carries the XT32 + a smaller radar.",),
)

LEGACY_SMALL = PlatformSpec(
    name="legacy_small",
    airframe="The sim's original abstract ~1 m quad (not a product): sim/coordinator.py KinematicFlight defaults",
    layout="quad-X", n_rotors=4, span_m=1.0, wheelbase_m=0.65, prop_d_m=0.33,
    empty_mass_kg=3.0, payload_kg=1.2, mtow_kg=4.5, battery_wh=160.0,
    endurance_min=round(1 / 0.0016 / 60, 2), cruise_mps=4.0, climb_mps=2.0, descend_mps=1.2, cruise_alt_m=8.0,
    terrain_clear_m=3.0, max_wind_mps=10.0, ip_rating="n/a", idle_power_w=288.0,
    gear_footprint_m=(0.34, 0.26),
    lidar=MID360_LIKE,
    radar=RadarSpec(band_ghz=(0.4, 2.0), aperture_m=(0.20, 0.15), weight_kg=0.6),
    mics=MicArraySpec(n_mics=6, aperture_m=0.30, weight_kg=0.15),
    speaker=None, thermal=None,
    sources={"LiDAR class: 360 x 59 deg, 200k pts/s, 40 m @10%": _SRC["mid360"],
             "6-mic 0.3 m array: acoustic/scene.py (N_MICS, MIC_RADIUS)": "acoustic/scene.py"},
    estimates=("everything except the battery rates (0.0016 /s flying, 0.0005 /s landed) taken from the sim code",),
    notes=("Kept for comparison and so the seed-7 demo reproduces.",),
)

PRESETS = {p.name: p for p in (HEAVY_SAR, INDUSTRIAL_SAR, LEGACY_SMALL)}
DEFAULT = "heavy_sar"


def get_platform(name: str | None = None) -> PlatformSpec:
    name = name or DEFAULT
    if name in ("heavy", "big"):
        name = "heavy_sar"
    if name in ("small", "legacy"):
        name = "legacy_small"
    if name not in PRESETS:
        raise KeyError(f"unknown platform {name!r}; choose from {sorted(PRESETS)}")
    return PRESETS[name]


def radar_rx_advantage_db(spec: PlatformSpec, ref: PlatformSpec = LEGACY_SMALL, f_ghz: float = 0.6) -> float:
    """Receive-gain advantage over ``ref`` at ``f_ghz`` (0.6 GHz: the sub-1 GHz part that carries deep returns)."""
    return spec.radar.rx_gain_dbi(f_ghz) - ref.radar.rx_gain_dbi(f_ghz)


def summary(spec: PlatformSpec) -> dict:
    """Key numbers (for docs/PLATFORM.md and the dashboard)."""
    return {
        "name": spec.name, "airframe": spec.airframe, "span_m": spec.span_m,
        "takeoff_mass_kg": round(spec.takeoff_mass_kg, 1), "endurance_min": spec.endurance_min,
        "usable_min_25pct_reserve": round(0.75 * spec.endurance_min, 1), "cruise_mps": spec.cruise_mps,
        "downwash_disk_mps": round(spec.downwash_mps(), 1), "hover_thrust_n": round(spec.static_load_n()),
        "rotor_disk_m2": round(spec.disk_area_m2(), 2),
        "lidar": spec.lidar.model, "lidar_pts_per_s": int(spec.lidar.pts_per_s),
        "radar_rx_gain_dbi_0.6GHz": round(spec.radar.rx_gain_dbi(0.6), 1),
        "radar_rx_gain_dbi_1.2GHz": round(spec.radar.rx_gain_dbi(1.2), 1),
        "radar_rx_advantage_db_0.6GHz": round(radar_rx_advantage_db(spec), 1),
        "mic_beamwidth_deg_1kHz": round(spec.mics.beamwidth_deg(1000.0), 1),
        "speaker_spl_db_at_20m": None if spec.speaker is None else round(spec.speaker.spl_at(20.0), 1),
        "fly_rate_per_s": round(spec.fly_rate, 6), "idle_rate_per_s": round(spec.idle_rate, 7),
        "landing": spec.landing_limits(),
    }


if __name__ == "__main__":   # python -m sim.platform
    import json
    print(json.dumps({k: summary(v) for k, v in PRESETS.items()}, indent=1))
