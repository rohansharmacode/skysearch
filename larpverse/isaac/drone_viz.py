"""Visual-only drone (no rigid body, no collider), posed kinematically through xform ops.

    d = DroneViz(stage, "/World/Drones/d1")          # heavy-lift coaxial X8 with the real SAR payload (default)
    d = DroneViz(stage, path, model="small")         # the original ~1 m quad (scaled by ``scale``)
    d = DroneViz(stage, path, asset="auto")          # Isaac quadcopter USD, physics stripped; falls back
    d.set_state(pos, quat_wxyz, rotor_rpm, winch_len, dt)   # flight-motion route (sim/flight_motion.py)
    d.set_pose(x, y, z, yaw_deg, tilt); d.spin(dt)   # older route (mission poses, hover rpm)
    d.set_winch(L, sway_deg=(roll, pitch), ground_z=None)   # pod on its cable (PodViz(drones=...) drives it)
    d.anchors_world()                                # {part: world xyz} for close-up labels

``model="heavy"`` builds a FlyCart 30-class coaxial X8 at true scale from ``sim.platform`` (``heavy_sar``: 2.2 m
wheelbase, 8 x 1.375 m props) carrying the components of data/hardware_bom.json at its ``isaac_mounts``
positions (drone frame: +x nose, +y left, +z up, z = 0 at the skid bottom): LidarPuck (Ouster OS1-128, r 0.043 m,
spin axis pitched 45 deg nose-down on its bracket), Gimbal + Thermal, 8-mic ArrayRing (Mic0..Mic7, 1.0 m),
SpeakerHorn (mouth down-aft), ComputeBox (Jetson AGX Orin) with MeshRadio stacked on it, MeshAntennaL/R under
the rear arms, WifiApAntenna and PodLinkAntenna pointing down, GnssMast, Winch, side Battery packs, landing
gear, status LEDs; and the POD (pods.pod_parts: PodRadarFace, PodMic0..3, PodLiftEye, PodStrobe, PodAntenna)
stowed between the skids, lowered on the winch cable.  In pod mode the drone body carries NO radar panel
(``delivery="land"`` puts the old RadarPanel back under the belly for --delivery land comparisons).

Rotors: ``rotor_rpm`` (8 values, order ``ROTOR_ORDER`` = arm 0..3 at 45 + 90k deg, each lower then upper) turns
each rotor by rpm x 6 x dt degrees.  Above ``BLUR_RPM`` the blades are hidden and a translucent motion-blur disc
(dark rim) shows instead; between ``SLOW_RPM`` and ``BLUR_RPM`` blades and a faint disc (spin-up / spin-down);
below, still blades.  That reads far better on a 30 fps video than stepping blades.

``heavy_parts`` is the pure-python parts list (tested without pxr); ``component_anchors()`` the label anchors
(BOM names).  ``NullDrone`` has the same interface without pxr (``--dry-run``).
"""
from __future__ import annotations

import json
import math
import os

import numpy as np

COL = {
    "body": (0.16, 0.18, 0.21), "arm": (0.10, 0.11, 0.12), "rotor": (0.75, 0.78, 0.82),
    "blade": (0.05, 0.05, 0.06), "lidar": (0.55, 0.30, 0.85), "radar": (0.98, 0.55, 0.12),
    "mic": (0.15, 0.75, 0.72), "skid": (0.22, 0.22, 0.24), "speaker": (0.85, 0.85, 0.82),
    "battery": (0.30, 0.32, 0.36), "thermal": (0.12, 0.12, 0.13), "comms": (0.23, 0.51, 0.96),
    "compute": (0.20, 0.22, 0.25), "gnss": (0.90, 0.90, 0.90), "winch": (0.25, 0.25, 0.27),
    "metal": (0.62, 0.63, 0.65),
}
MODELS = ("heavy", "small")
# --photoreal: real surface materials per part (UsdPreviewSurface; RTX and path tracing) instead of flat colours.
PBR = False
STYLE = "blocks"       # "mesh": the smooth heavy-lift model of isaac/drone_mesh.py (photoreal sets it)
PROP_DISC = 0.06       # PBR: opacity of the spinning-prop blur disc (0 hides it; the blades still turn)
PBR_LOOK = {           # key -> (rgb, roughness, metallic, clearcoat); rgb None = the part's own colour
    "body": ((0.19, 0.20, 0.22), 0.32, 0.0, 0.8),       # painted composite shell
    "stripe": (None, 0.35, 0.0, 0.6),
    "carbon": ((0.022, 0.022, 0.026), 0.26, 0.0, 0.9),  # carbon-fibre arms and landing gear
    "metal": ((0.32, 0.33, 0.35), 0.30, 1.0, 0.0),      # motor bells
    "canopy": ((0.80, 0.81, 0.82), 0.30, 0.0, 0.9),     # light top shell
    "gimbal": ((0.09, 0.09, 0.10), 0.35, 0.6, 0.0),
    "plastic": ((0.03, 0.03, 0.035), 0.45, 0.0, 0.0),   # prop blades
    "radome": ((0.80, 0.80, 0.78), 0.55, 0.0, 0.2),     # radar panel cover
    "battery": ((0.10, 0.11, 0.12), 0.40, 0.0, 0.4),
    "lens": ((0.015, 0.015, 0.02), 0.06, 0.0, 1.0),     # LiDAR window, thermal lens
    "mic": ((0.04, 0.04, 0.045), 0.6, 0.0, 0.0),
    "speaker": ((0.70, 0.70, 0.68), 0.5, 0.0, 0.0),
    "disc": ((0.10, 0.10, 0.11), 0.5, 0.0, 0.0),
    "wifi": ((0.08, 0.30, 0.95), 0.40, 0.0, 0.3),        # blue rescue-Wi-Fi antenna (colour code: blue = Wi-Fi)
    "nav_red": ((0.6, 0.05, 0.03), 0.3, 0.0, 0.5),
    "nav_green": ((0.05, 0.6, 0.1), 0.3, 0.0, 0.5),
    "strobe": ((0.9, 0.9, 0.9), 0.3, 0.0, 0.5),
}
EMISSIVE = {"nav_red": (160.0, 8.0, 4.0), "nav_green": (8.0, 130.0, 16.0), "strobe": (260.0, 260.0, 270.0)}
STROBE = (1.0, 0.09)   # s: anti-collision strobe period and flash length (render clock)
PART_KEY = (("Fuselage", "body"), ("Canopy", "canopy"), ("Visor", "lens"), ("GpsMast", "gimbal"),
            ("GpsPuck", "radome"), ("Hinge", "gimbal"), ("Column", "gimbal"), ("Foot", "plastic"),
            ("YokeGimbal", "gimbal"), ("LensZoom", "lens"), ("LidarCap", "gimbal"), ("Hub", "gimbal"),
            ("Stripe", "stripe"), ("Battery", "battery"), ("Arm", "carbon"),
            ("Motor", "metal"), ("Skid", "carbon"), ("Leg", "carbon"), ("Brace", "carbon"), ("Gimbal", "gimbal"),
            ("LidarPuck", "lens"), ("Thermal", "lens"), ("RadarPanel", "radome"), ("SpeakerHorn", "speaker"),
            ("Mic", "mic"), ("Blade", "plastic"), ("Disc", "disc"), ("WifiAnt", "wifi"), ("Antenna", "plastic"),
            ("ComputeBox", "battery"), ("RadioBox", "battery"), ("WinchDrum", "metal"), ("WinchFrame", "carbon"),
            ("NavRed", "nav_red"), ("NavGreen", "nav_green"), ("NavRear2", "nav_red"), ("NavRear3", "nav_green"),
            ("Strobe", "strobe"), ("LidarBracket", "gimbal"))
GEAR_H = 0.60          # heavy: skid bottom to fuselage underside (m, estimate)
WINCH_BOTTOM = 0.445   # underside of the winch (cable exit), BOM Winch pos z 0.52 - 0.15 / 2
POD_STOW_Z = 0.175     # stowed pod base (BOM "Pod (stowed)": centre 0.30, height 0.25)
HOVER_RPM = 1000.0     # 54-in props on a ~80 kg X8 (sim/flight_motion.py estimate, docs/MOTION.md): visual only
BLUR_RPM = 450.0
SLOW_RPM = 120.0
ROTOR_ORDER = [f"Rotor{i}_{lvl}" for i in range(4) for lvl in (0, 1)]
BOM_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data", "hardware_bom.json")

DRONE_MATS = {
    "body": dict(rgb=(0.16, 0.18, 0.21), rough=0.45, clearcoat=0.3),
    "carbon": dict(rgb=(0.06, 0.065, 0.07), rough=0.35, clearcoat=0.5),
    "arm": dict(rgb=(0.10, 0.11, 0.12), rough=0.5),
    "metal": dict(rgb=(0.62, 0.63, 0.65), rough=0.3, metal=0.9),
    "blade": dict(rgb=(0.05, 0.05, 0.06), rough=0.4),
    "blur_hi": dict(rgb=(0.10, 0.10, 0.11), rough=0.5, opacity=0.30),
    "blur_lo": dict(rgb=(0.10, 0.10, 0.11), rough=0.5, opacity=0.12),
    "lidar": dict(rgb=(0.55, 0.30, 0.85), rough=0.4),
    "lidar_window": dict(rgb=(0.04, 0.04, 0.05), rough=0.08, metal=0.3),
    "radar": dict(rgb=(0.98, 0.55, 0.12), rough=0.6),
    "mic": dict(rgb=(0.15, 0.75, 0.72), rough=0.9),
    "skid": dict(rgb=(0.22, 0.22, 0.24), rough=0.5),
    "speaker": dict(rgb=(0.85, 0.85, 0.82), rough=0.5),
    "battery": dict(rgb=(0.30, 0.32, 0.36), rough=0.55),
    "thermal": dict(rgb=(0.12, 0.12, 0.13), rough=0.4),
    "glass": dict(rgb=(0.02, 0.02, 0.03), rough=0.05, metal=0.4),
    "comms": dict(rgb=(0.23, 0.51, 0.96), rough=0.5),
    "compute": dict(rgb=(0.20, 0.22, 0.25), rough=0.45, metal=0.5),
    "gnss": dict(rgb=(0.90, 0.90, 0.90), rough=0.4),
    "winch": dict(rgb=(0.25, 0.25, 0.27), rough=0.5, metal=0.4),
    "led_white": dict(rgb=(1.0, 1.0, 1.0), rough=0.2, emissive=(4.0, 4.0, 4.0)),
    "led_green": dict(rgb=(0.2, 0.8, 0.3), rough=0.3, emissive=(0.3, 2.5, 0.45)),
    "led_blue": dict(rgb=(0.2, 0.5, 0.9), rough=0.3, emissive=(0.3, 1.0, 2.6)),
}
ACCENT = {"d1": (0.20, 0.55, 0.95), "d2": (0.95, 0.80, 0.20), "d3": (0.30, 0.85, 0.45)}
ASSET_CANDIDATES = (
    "/Isaac/Robots/IsaacSim/Quadcopter/quadcopter.usd",     # 5.x/6.0 layout (exists on 6.0: assets_found.json)
    "/Isaac/Robots/Quadcopter/quadcopter.usd",              # 4.x layout
    "/Isaac/Robots/Bitcraze/Crazyflie/cf2x.usd",            # exists on 6.0 (assets_found.json); tiny, scale up
    "/Isaac/Robots/Crazyflie/cf2x.usd",
)

# label anchors: key -> (part name, BOM id, short label)
ANCHORS = (
    ("lidar", "LidarPuck", "lidar", "LiDAR (45 deg mount)"),
    ("thermal", "Thermal", "thermal", "Thermal camera"),
    ("mic_array", "Mic1", "mic_ring", "8-mic array ring"),
    ("speaker", "SpeakerHorn", "speaker", "Loudspeaker"),
    ("jetson", "ComputeBox", "compute", "Jetson compute"),
    ("mesh_radio", "MeshRadio", "mesh_radio", "Mesh radio"),
    ("mesh_antenna", "MeshAntennaL", "mesh_radio", "Mesh antennas"),
    ("wifi_ap", "WifiApAntenna", "wifi_ap", "Wi-Fi AP (survivor phones)"),
    ("gnss", "GnssMast", "gnss", "RTK GNSS"),
    ("pod_link", "PodLinkAntenna", "pod_link", "Pod link 5 GHz"),
    ("winch", "Winch", "winch", "Winch"),
    ("battery", "Battery0", None, "Battery (2 x DB2000)"),
    ("landing_gear", "Skid0", None, "Landing gear"),
    ("pod_radar", "PodRadarFace", "pod_antennas", "Pod radar face"),
    ("pod_mics", "PodMic0", "pod_mics", "Pod mics"),
    ("pod_eye", "PodLiftEye", "pod_enclosure", "Pod lift eye"),
    ("pod_strobe", "PodStrobe", "pod_enclosure", "Pod strobe"),
    ("pod_antenna", "PodAntenna", "pod_cpu", "Pod antenna"),
)


def load_bom(path=BOM_PATH):
    try:
        with open(path) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def _bom_mounts(bom):
    return {m.get("part"): m for m in (bom or {}).get("isaac_mounts", []) if isinstance(m, dict)}


def _bom_names(bom):
    out = {}
    for it in (bom or {}).get("drone_payload", []) + ((bom or {}).get("pod") or {}).get("items", []):
        if isinstance(it, dict) and "id" in it:
            out[it["id"]] = it.get("name", "")
    return out


def heavy_parts(spec=None, did="d1", delivery="pod", bom=None):
    """Parts of the heavy-lift model: [(kind, name, translate, scale, rgb, rot_z, attrs)] in metres, skid bottom
    at z = 0, +x forward.  Airframe from ``sim.platform``; payload at the data/hardware_bom.json isaac_mounts.
    attrs: Radius / Height / Size / Axis (USD-style) plus mat, group ("Pod" = the winch pod, pod frame shifted
    to its stowed place), Rot (rx, ry, rz deg), Tube / Chamfer (cube drawn as a tube / chamfered box),
    Throat (horn), ZRange (dome), Tubeness for Torus."""
    if spec is None:
        from sim.platform import get_platform
        spec = get_platform("heavy_sar")
    bm = _bom_mounts(load_bom() if bom is None else bom)
    acc = ACCENT.get(did, (0.9, 0.9, 0.9))
    P = []

    def add(kind, name, t, s=(1, 1, 1), rgb=(1, 1, 1), rot_z=None, **attrs):
        P.append((kind, name, tuple(map(float, t)), tuple(map(float, s)), rgb, rot_z, attrs))

    def at(part, default):
        m = bm.get(part)
        p = m.get("pos_m") if m else None
        return tuple(p) if isinstance(p, (list, tuple)) and len(p) == 3 else default

    z0 = GEAR_H                                    # fuselage underside
    fl, fw, fh = 1.0, 0.70, 0.40
    add("Cube", "Fuselage", (0, 0, z0 + fh / 2), (fl, fw, fh), COL["body"], Size=1.0, Chamfer=0.08, mat="body")
    add("Sphere", "Canopy", (0.05, 0, z0 + fh - 0.005), (0.44, 0.30, 0.09), COL["body"], Radius=1.0,
        ZRange=(0.0, 1.0), mat="body")
    add("Cube", "Stripe", (0.15, 0, z0 + fh + 0.004), (0.45, 0.10, 0.012), acc, Size=1.0, mat="accent")
    for sd in (-1, 1):                             # side-slung battery packs (2 x DB2000)
        add("Cube", f"Battery{sd + 1}", (-0.05, sd * (fw / 2 + 0.06), z0 + 0.20), (0.50, 0.12, 0.26), COL["battery"],
            Size=1.0, Chamfer=0.02, mat="battery")
        add("Cube", f"BatteryHandle{sd + 1}", (-0.05, sd * (fw / 2 + 0.125), z0 + 0.30), (0.24, 0.02, 0.03),
            COL["arm"], Size=1.0, mat="arm")
    R = spec.wheelbase_m / 2                       # hub-to-motor
    rp = spec.prop_d_m / 2
    za = z0 + fh + 0.10                            # arm height
    pairs = spec.n_rotors // 2 if spec.layout.startswith("coaxial") else spec.n_rotors
    for i in range(pairs):
        a = math.radians(45 + 360.0 / pairs * i)
        ca, sa = math.cos(a), math.sin(a)
        cx, cy = R * ca, R * sa
        add("Cube", f"Arm{i}", (cx / 2, cy / 2, za), (R, 0.08, 0.08), COL["arm"], rot_z=math.degrees(a), Size=1.0,
            Tube=True, mat="carbon")
        add("Cube", f"ArmJoint{i}", (0.42 * ca, 0.42 * sa, za), (0.12, 0.11, 0.11), COL["arm"],
            rot_z=math.degrees(a), Size=1.0, mat="arm")
        coax = spec.layout.startswith("coaxial")
        levels = ((0, za - 0.12), (1, za + 0.14)) if coax else ((0, za + 0.10),)
        if coax:
            add("Cylinder", f"MotorMount{i}", (cx, cy, za + 0.01), (1, 1, 1), COL["arm"], Radius=0.05, Height=0.26,
                Axis="Z", mat="arm")
        for lvl, zr in levels:
            add("Cylinder", f"Motor{i}_{lvl}", (cx, cy, zr), (1, 1, 1), COL["arm"],
                Radius=0.09, Height=0.10, Axis="Z", mat="metal")
            add("Rotor", f"Rotor{i}_{lvl}", (cx, cy, zr + (0.06 if lvl else -0.06)), (1, 1, 1), COL["rotor"],
                None, radius=rp, sign=(1 if (i + lvl) % 2 else -1))
        front = ca > 0                             # arm status LEDs under the lower motors (no red: red = ping)
        add("Sphere", f"LedArm{i}", (cx, cy, za - 0.20), (1, 1, 1), (1, 1, 1) if front else acc, Radius=0.028,
            mat="led_white" if front else "led_accent")
    for sd in (-1, 1):                             # landing gear: two skids, two legs each, cross bars
        y = sd * 0.70
        add("Cube", f"Skid{sd + 1}", (0, y, 0.03), (1.60, 0.06, 0.06), COL["skid"], Size=1.0, Tube=True, mat="skid")
        for fb in (-1, 1):
            add("Cube", f"Leg{sd + 1}{fb + 1}", (fb * 0.45, y * 0.93, z0 / 2), (0.05, 0.05, z0), COL["skid"], Size=1.0,
                Rot=(sd * 4.0, 0, 0), mat="skid")
        add("Cube", f"Brace{sd + 1}", (0, sd * 0.52, z0 - 0.03), (0.95, 0.36, 0.04), COL["skid"], Size=1.0, mat="skid")
    # ---------------------------------------------------------------- payload (BOM isaac_mounts)
    gx, gy, gz = at("Gimbal", (0.58, 0.0, 0.56))
    add("Cube", "Gimbal", (gx, gy, gz), (0.14, 0.14, 0.12), (0.10, 0.11, 0.12), Size=1.0, Chamfer=0.02, mat="arm")
    tx, ty, tz = at("Thermal", (0.60, 0.13, 0.48))
    add("Cube", "Thermal", (tx, ty, tz), (0.06, 0.06, 0.06), COL["thermal"], Size=1.0, mat="thermal")
    add("Cylinder", "ThermalLens", (tx + 0.035, ty, tz), (1, 1, 1), COL["thermal"], Radius=0.018, Height=0.012,
        Axis="X", mat="glass")
    lx, ly, lz = at("LidarPuck", (0.63, 0.0, 0.44))
    t45 = math.radians(45.0)
    ax_ = (math.sin(t45), 0.0, math.cos(t45))      # spin axis pitched 45 deg nose-down
    add("Cylinder", "LidarPuck", (lx, ly, lz), (1, 1, 1), COL["lidar"], Radius=0.043, Height=0.074, Axis=ax_,
        mat="lidar")
    add("Cylinder", "LidarWindow", (lx, ly, lz), (1, 1, 1), COL["thermal"], Radius=0.0435, Height=0.03, Axis=ax_,
        mat="lidar_window")
    bx_, bz_ = lx - ax_[0] * 0.045, lz - ax_[2] * 0.045
    add("Cube", "LidarMount", (bx_, ly, bz_), (0.11, 0.10, 0.012), COL["metal"], Size=1.0, Rot=(0, 45, 0), mat="metal")
    add("Cube", "LidarStrut", ((bx_ + 0.47) / 2, ly, (bz_ + z0) / 2 - 0.01),
        (math.hypot(0.47 - bx_, z0 - bz_), 0.03, 0.025), COL["metal"], Size=1.0,
        Rot=(0, math.degrees(math.atan2(bz_ - z0, 0.47 - bx_)) * -1, 0), mat="metal")
    if spec.speaker is not None:
        sx_, sy_, sz_ = at("SpeakerHorn", (-0.52, 0.0, 0.52))
        hz = (-math.sin(math.radians(40)), 0.0, -math.cos(math.radians(40)))     # mouth down-aft
        add("Cone", "SpeakerHorn", (sx_, sy_, sz_), (1, 1, 1), COL["speaker"], Radius=0.07, Height=0.13, Throat=0.03,
            Axis=hz, mat="speaker")
        add("Cube", "SpeakerBracket", (sx_ + 0.03, sy_, (sz_ + z0) / 2 + 0.02), (0.04, 0.10, z0 - sz_), COL["metal"],
            Size=1.0, mat="metal")
    rm = spec.mics.aperture_m / 2
    zr_ = z0 + fh + 0.03
    add("Torus", "ArrayRing", (0, 0, zr_), (1, 1, 1), COL["mic"], Radius=rm, Tubeness=0.01, mat="carbon")
    for k in range(4):
        a = math.radians(90 * k)
        add("Cube", f"RingStrut{k}", (0.40 * math.cos(a), 0.40 * math.sin(a) * 0.9, z0 + fh + 0.015),
            (0.22, 0.02, 0.02), COL["arm"], rot_z=90 * k, Size=1.0, mat="carbon")
    for k in range(spec.mics.n_mics):
        a = math.radians(22.5 + 360.0 / spec.mics.n_mics * k)
        add("Sphere", f"Mic{k}", (rm * math.cos(a), rm * math.sin(a), zr_), (1, 1, 1), COL["mic"], Radius=0.03, mat="mic")
    cx_, cy_, cz_ = at("ComputeBox", (-0.25, 0.0, 1.06))
    add("Cube", "ComputeBox", (cx_, cy_, cz_), (0.25, 0.18, 0.08), COL["compute"], Size=1.0, Chamfer=0.01, mat="compute")
    for k in range(5):
        add("Cube", f"ComputeFin{k}", (cx_ - 0.08 + 0.04 * k, cy_ + 0.095, cz_), (0.008, 0.01, 0.06), COL["metal"],
            Size=1.0, mat="metal")
    add("Cube", "LedStatus", (cx_ + 0.126, cy_ + 0.05, cz_ + 0.02), (0.004, 0.02, 0.01), (0.2, 0.8, 0.3), Size=1.0,
        mat="led_green")
    mx_, my_, mz_ = at("MeshRadio", (-0.25, 0.0, 1.12))
    add("Cube", "MeshRadio", (mx_, my_, mz_), (0.07, 0.07, 0.03), COL["comms"], Size=1.0, mat="comms")
    add("Cube", "LedMesh", (mx_ + 0.036, my_, mz_), (0.004, 0.015, 0.008), (0.2, 0.5, 0.9), Size=1.0, mat="led_blue")
    for nm, dflt in (("MeshAntennaL", (-0.39, 0.39, 0.95)), ("MeshAntennaR", (-0.39, -0.39, 0.95))):
        p = at(nm, dflt)
        add("Cylinder", nm, p, (1, 1, 1), COL["comms"], Radius=0.01, Height=0.20, Axis="Z", mat="comms")
        add("Cylinder", nm + "Base", (p[0], p[1], p[2] + 0.105), (1, 1, 1), COL["arm"], Radius=0.016, Height=0.02,
            Axis="Z", mat="arm")
    gx2, gy2, gz2 = at("GnssMast", (0.15, 0.0, 1.25))
    add("Cylinder", "GnssPole", (gx2, gy2, (z0 + fh + gz2) / 2), (1, 1, 1), COL["metal"], Radius=0.008,
        Height=gz2 - (z0 + fh), Axis="Z", mat="metal")
    add("Cylinder", "GnssMast", (gx2, gy2, gz2), (1, 1, 1), COL["gnss"], Radius=0.04, Height=0.03, Axis="Z", mat="gnss")
    add("Sphere", "GnssDome", (gx2, gy2, gz2 + 0.015), (0.04, 0.04, 0.015), COL["gnss"], Radius=1.0, ZRange=(0, 1),
        mat="gnss")
    wx, wy, wz = at("WifiApAntenna", (-0.30, 0.20, 0.48))
    add("Cylinder", "WifiApAntenna", (wx, wy, wz), (1, 1, 1), COL["comms"], Radius=0.012, Height=0.18, Axis="Z", mat="comms")
    add("Cube", "WifiAp", (wx, wy, z0 - 0.015), (0.08, 0.06, 0.025), COL["compute"], Size=1.0, mat="compute")
    px_, py_, pz_ = at("PodLinkAntenna", (0.25, 0.0, 0.55))
    add("Cylinder", "PodLinkAntenna", (px_, py_, pz_), (1, 1, 1), COL["comms"], Radius=0.01, Height=0.08, Axis="Z",
        mat="comms")
    if delivery == "land":                          # old body radar (--delivery land comparisons only)
        rw, rh = spec.radar.aperture_m
        add("Cube", "RadarPanel", (-0.05, 0, z0 - 0.05), (rh, rw, 0.05), COL["radar"], Size=1.0, mat="radar")
    else:
        wx2, wy2, wz2 = at("Winch", (0.0, 0.0, 0.52))
        add("Cube", "Winch", (wx2, wy2, wz2), (0.30, 0.20, 0.15), COL["winch"], Size=1.0, Chamfer=0.015, mat="winch")
        add("Cylinder", "WinchDrum", (wx2, wy2, wz2 - 0.01), (1, 1, 1), COL["metal"], Radius=0.055, Height=0.24,
            Axis="Y", mat="metal")
        from isaac.pods import pod_parts
        for name, kind, pos, dims, mat, _ in pod_parts():       # stowed: pod base at POD_STOW_Z
            t = (pos[0], pos[1], pos[2] + POD_STOW_Z)
            if kind == "box":
                add("Cube", name, t, dims, (0.5, 0.5, 0.5), Size=1.0, mat=mat, group="Pod")
            elif kind == "cyl":
                add("Cylinder", name, t, (1, 1, 1), (0.5, 0.5, 0.5), Radius=dims["r"], Height=dims["h"], Axis="Z",
                    mat=mat, group="Pod")
            elif kind == "sphere":
                add("Sphere", name, t, (1, 1, 1), (0.5, 0.5, 0.5), Radius=dims["r"], mat=mat, group="Pod")
            elif kind == "torus":
                add("Torus", name, t, (1, 1, 1), (0.6, 0.6, 0.6), Radius=dims["R"], Tubeness=dims["r"], Axis="X",
                    mat=mat, group="Pod")
            elif kind == "line":
                add("Line", name, t, (1, 1, 1), (0.15, 0.15, 0.15), End=(dims[0], dims[1], dims[2] + POD_STOW_Z),
                    mat=mat, group="Pod")
    return P


def model_extent(parts):
    """(x_extent_m, y_extent_m, z_min, z_max) of a parts list, rotor discs included (rotated cubes approximated
    by their half-diagonal)."""
    ex = ey = 0.0
    zmin, zmax = 1e9, -1e9
    for kind, _, t, s, _, rz, attrs in parts:
        if kind == "Line":
            continue
        if kind == "Mesh":
            P = attrs["P"]
            ex, ey = max(ex, float(abs(P[:, 0]).max())), max(ey, float(abs(P[:, 1]).max()))
            zmin, zmax = min(zmin, float(P[:, 2].min())), max(zmax, float(P[:, 2].max()))
            continue
        if kind == "Rotor":
            hx = hy = attrs["radius"]
        elif kind == "Cube":
            hx, hy = (s[0] / 2, s[1] / 2) if rz is None else (math.hypot(s[0], s[1]) / 2,) * 2
        elif kind == "Torus":
            hx = hy = attrs["Radius"] + attrs.get("Tubeness", 0.0)
        else:
            hx, hy = attrs.get("Radius", 0.0) * s[0], attrs.get("Radius", 0.0) * s[1]
        ex, ey = max(ex, abs(t[0]) + hx), max(ey, abs(t[1]) + hy)
        if kind == "Cube":
            h = s[2] / 2
        elif kind == "Torus":
            h = attrs.get("Tubeness", 0.0)
        else:
            h = attrs.get("Height", 2 * attrs.get("Radius", 0.0) * s[2]) / 2
        zmin, zmax = min(zmin, t[2] - h), max(zmax, t[2] + h)
    return 2 * ex, 2 * ey, zmin, zmax


def component_anchors(spec=None, delivery="pod", bom=None):
    """Label anchors for a close-up: [{key, part, label, bom_id, bom_name, parent, xyz}], xyz in the drone model
    frame (z = 0 at the skid bottom; pod parts at their STOWED place: use DroneViz.anchors_world() live)."""
    bom = load_bom() if bom is None else bom
    names = _bom_names(bom)
    parts = {p[1]: p for p in heavy_parts(spec, "d1", delivery, bom)}
    out = []
    for key, part, bom_id, short in ANCHORS:
        p = parts.get(part)
        if p is None:
            continue
        out.append(dict(key=key, part=part, label=short, bom_id=bom_id or "", bom_name=names.get(bom_id, ""),
                        parent="pod" if p[6].get("group") == "Pod" else "drone", xyz=p[2]))
    return out


def quat_ypr(yaw_deg, pitch_deg=0.0, roll_deg=0.0):
    """(w, x, y, z) for R = Rz(yaw) @ Ry(pitch) @ Rx(roll); positive pitch = nose (+x) down."""
    cy, sy = math.cos(math.radians(yaw_deg) / 2), math.sin(math.radians(yaw_deg) / 2)
    cp, sp = math.cos(math.radians(pitch_deg) / 2), math.sin(math.radians(pitch_deg) / 2)
    cr, sr = math.cos(math.radians(roll_deg) / 2), math.sin(math.radians(roll_deg) / 2)
    return (cy * cp * cr + sy * sp * sr, cy * cp * sr - sy * sp * cr,
            cy * sp * cr + sy * cp * sr, sy * cp * cr - cy * sp * sr)


def _tilt(tilt):
    if tilt is None:
        return 0.0, 0.0
    if isinstance(tilt, (tuple, list)):
        return float(tilt[0]), float(tilt[1])
    return 0.0, float(tilt)


def _quat_tuple(q):
    """(w, x, y, z) from a 4-sequence (w first) or a Gf.Quat*."""
    if hasattr(q, "GetReal"):
        im = q.GetImaginary()
        return float(q.GetReal()), float(im[0]), float(im[1]), float(im[2])
    w, x, y, z = (float(v) for v in q)
    return w, x, y, z


def set_state(drones, drone_id, pos, quat, rotor_rpm=None, winch_len=None, dt=1 / 30.0, **kw):
    """Fleet helper for the flight-motion loop: drones[drone_id].set_state(...). False if unknown."""
    d = drones.get(drone_id)
    if d is None:
        return False
    d.set_state(pos, quat, rotor_rpm, winch_len, dt, **kw)
    return True


class NullDrone:
    """Same interface as DroneViz, records poses only (dry-run / tests)."""

    def __init__(self, stage=None, path="", **_):
        self.path = path
        self.pose = None
        self.history = []
        self.rotor_angle = 0.0
        self.rpm = [0.0] * 8
        self.winch = 0.0

    def set_pose(self, x, y, z, yaw_deg, tilt=None):
        roll, pitch = _tilt(tilt)
        self.pose = (float(x), float(y), float(z), float(yaw_deg), roll, pitch)
        if not self.history or self.history[-1][:3] != self.pose[:3]:
            self.history.append(self.pose)

    def set_state(self, pos, quat, rotor_rpm=None, winch_len=None, dt=1 / 30.0, **kw):
        w, x, y, z = _quat_tuple(quat)
        yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
        self.set_pose(pos[0], pos[1], pos[2], yaw)
        if rotor_rpm is not None:
            self.rpm = [float(r) for r in rotor_rpm]
            self.rotor_angle = (self.rotor_angle + 6.0 * (sum(self.rpm) / max(1, len(self.rpm))) * dt) % 360.0
        if winch_len is not None:
            self.set_winch(winch_len)

    def set_winch(self, length, sway_deg=(0.0, 0.0), ground_z=None):
        self.winch = float(length)

    def winch_world_z(self, z):
        return float(z) - 0.15 + WINCH_BOTTOM

    def spin(self, dt, on=True):
        self.rotor_angle = (self.rotor_angle + (2400.0 if on else 0.0) * dt) % 360.0


class DroneViz:
    ARM = 0.32        # hub-to-rotor distance (m) before scale

    def __init__(self, stage, path, did="d1", scale=1.6, asset=None, skid_offset=0.15, model="heavy", spec=None,
                 delivery="pod", looks_root="/World/Drones/Looks"):
        from pxr import Gf, UsdGeom
        self.Gf, self.UsdGeom = Gf, UsdGeom
        self.stage, self.path, self.did = stage, path, did
        self.rotors = []                      # [{op, sign, angle, blades, lo, hi, state}]
        self.rotor_ops = []                   # (op, sign): kept for older callers
        self.rotor_angle = 0.0
        self.skid_offset = float(skid_offset)
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.pos = (0.0, 0.0, -100.0)
        self.winch, self.sway, self.ground_z = 0.0, (0.0, 0.0), None
        self.hang = None
        self.tris = 0
        root = UsdGeom.Xform.Define(stage, path)
        xf = UsdGeom.Xformable(root)
        xf.ClearXformOpOrder()
        self.t_op = xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        self.q_op = xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
        self.t_op.Set(Gf.Vec3d(0, 0, -100))
        self.q_op.Set(Gf.Quatd(1, 0, 0, 0))
        mdl = UsdGeom.Xform.Define(stage, path + "/Model")      # skid bottom at the mission's pose z - 0.15
        mxf = UsdGeom.Xformable(mdl)
        mxf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0, 0, -skid_offset))
        heavy = model == "heavy" and not asset
        if heavy:
            scale = 1.0                                             # true metres from sim.platform
        mxf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(scale, scale, scale))
        self.model = path + "/Model"
        self.heavy = heavy
        self.spin_rate = 1200.0 if heavy else 2400.0               # deg/s for spin() on the small model
        if heavy:
            if STYLE == "blender":            # isaac/tools/blender_drone.py asset; None -> the models below
                from isaac.blender_drone import build
                rot = build(stage, self.model, did, log=print, emissive=EMISSIVE)
                if rot is not None:
                    self.rotors, self.rotor_ops = rot, [(r["op"], r["sign"]) for r in rot]
                    return
            if STYLE == "mesh":
                from isaac.drone_mesh import dji_parts
                self._build_parts_mesh(dji_parts(spec, did))
            else:
                self._build_parts(heavy_parts(spec, did, delivery), looks_root)
            return
        used_asset = False
        if asset:
            used_asset = self._reference_asset(asset)
        if not used_asset:
            self._primitives(did)
        self._payload()

    def _build_parts_mesh(self, parts):
        """STYLE 'mesh' (--drone-style mesh): the smooth DJI-class model of isaac/drone_mesh.py."""
        Gf, UsdGeom, m = self.Gf, self.UsdGeom, self.model
        for kind, name, t, s, rgb, rot_z, attrs in parts:
            if kind == "Mesh":
                self._meshpart(f"{m}/{name}", attrs["P"], attrs["F"], rgb)
                continue
            if kind != "Rotor":
                self._shape(kind, f"{m}/{name}", t, s, rgb, rot_z, **attrs)
                continue
            rotor = UsdGeom.Xform.Define(self.stage, f"{m}/{name}")
            rxf = UsdGeom.Xformable(rotor)
            rxf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*t))
            self.rotor_ops.append((rxf.AddRotateZOp(), attrs["sign"]))
            disc = self._shape("Cylinder", f"{m}/{name}/Disc", (0, 0, 0), (1, 1, 1), rgb,
                               Radius=attrs["radius"], Height=0.006, Axis="Z")
            try:
                disc.CreateDisplayOpacityAttr([0.25])
            except Exception:
                pass
            if attrs.get("style") == "mesh":
                from isaac import drone_mesh as DM
                self._meshpart(f"{m}/{name}/Blade", *DM.blades(attrs["radius"]), COL["blade"])
                self._meshpart(f"{m}/{name}/Hub", *DM.tube((0, 0, -0.02), (0, 0, 0.03), 0.045), COL["arm"])
            else:
                self._shape("Cube", f"{m}/{name}/Blade", (0, 0, 0.006), (2 * attrs["radius"], 0.09, 0.012),
                            COL["blade"], Size=1.0)
            if PBR and PROP_DISC <= 0:
                UsdGeom.Imageable(disc).MakeInvisible()
        if PBR:
            self._materials()

    def _materials(self):
        """--photoreal: bind a real surface material to every part, by part name (PART_KEY)."""
        from pxr import Sdf, Usd, UsdShade
        did = self.path.rstrip("/").split("/")[-1]
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(self.model)):
            if not prim.IsA(self.UsdGeom.Gprim):
                continue
            key = next((k for pre, k in PART_KEY if prim.GetName().startswith(pre)), None)
            if key is None:
                continue
            rgb, rough, metal, coat = PBR_LOOK[key]
            if rgb is None:
                c = self.UsdGeom.Gprim(prim).GetDisplayColorAttr().Get()
                rgb = tuple(c[0]) if c else (0.8, 0.8, 0.8)
            mpath = f"/World/Looks/Drone/{key}" + (f"_{did}" if key == "stripe" else "")
            mat = UsdShade.Material.Get(self.stage, mpath)
            if not mat:
                mat = UsdShade.Material.Define(self.stage, mpath)
                sh = UsdShade.Shader.Define(self.stage, mpath + "/Surface")
                sh.CreateIdAttr("UsdPreviewSurface")
                sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(self.Gf.Vec3f(*map(float, rgb)))
                sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(rough))
                sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metal))
                sh.CreateInput("clearcoat", Sdf.ValueTypeNames.Float).Set(float(coat))
                sh.CreateInput("clearcoatRoughness", Sdf.ValueTypeNames.Float).Set(0.08)
                if key == "disc":
                    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(max(PROP_DISC, 0.0)))
                if key in EMISSIVE:
                    sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(self.Gf.Vec3f(*EMISSIVE[key]))
                mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)

    # ------------------------------------------------------------------ geometry helpers
    def _meshpart(self, path, P, F, rgb=(0.5, 0.5, 0.5)):
        from pxr import Vt
        from isaac.drone_mesh import vertex_normals
        Gf, UsdGeom = self.Gf, self.UsdGeom
        P = np.ascontiguousarray(P, np.float32); F = np.ascontiguousarray(F, np.int32)
        g = UsdGeom.Mesh.Define(self.stage, path)
        g.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(P))
        g.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(F), 3, np.int32)))
        g.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(F.reshape(-1)))
        g.CreateSubdivisionSchemeAttr("none")
        g.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(vertex_normals(P, F)))
        g.SetNormalsInterpolation("vertex")
        g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, P.min(0))), Gf.Vec3f(*map(float, P.max(0)))]))
        g.CreateDisplayColorAttr([Gf.Vec3f(*map(float, rgb))])
        g.CreateDoubleSidedAttr(True)
        return g

    # ------------------------------------------------------------------ heavy model (meshes)
    def _build_parts(self, parts, looks_root):
        import numpy as np

        from isaac import pods as PD
        from isaac.van import Geo, Looks, Rx, Ry, Rz, emit, frame_from, xform
        Gf, UsdGeom, m = self.Gf, self.UsdGeom, self.model
        acc = ACCENT.get(self.did, (0.9, 0.9, 0.9))
        mats = dict(DRONE_MATS, **PD.POD_MATS)
        mats["accent"] = dict(rgb=acc, rough=0.4)
        mats["led_accent"] = dict(rgb=acc, rough=0.3, emissive=tuple(2.5 * c for c in acc))
        looks = Looks(self.stage, looks_root)

        def mat(key):
            name = key if key not in ("accent", "led_accent") else f"{key}_{self.did}"
            spec = mats.get(key, dict(rgb=(0.5, 0.5, 0.5)))
            return looks.get(name, **spec), spec.get("rgb", (0.5, 0.5, 0.5))

        def rot_of(rot_z, attrs):
            R = np.eye(3)
            if attrs.get("Rot"):
                rx, ry, rz = attrs["Rot"]
                R = Rz(rz) @ Ry(ry) @ Rx(rx)
            if rot_z is not None:
                R = Rz(rot_z) @ R
            return R

        # pod hang: Model/Hang (at the winch exit, counter-rotated to stay vertical) / Cable + Pod
        has_pod = any(p[6].get("group") == "Pod" for p in parts)
        if has_pod:
            self.hang = xform(self.stage, m + "/Hang", (0, 0, WINCH_BOTTOM))
            hx = UsdGeom.Xformable(self.hang)
            self.hang_q = hx.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
            self.hang_q.Set(Gf.Quatd(1, 0, 0, 0))
            cab = xform(self.stage, m + "/Hang/Cable")
            self.cable_s = UsdGeom.Xformable(cab).AddScaleOp(UsdGeom.XformOp.PrecisionDouble)
            self.cable_s.Set(Gf.Vec3d(1, 1, 1e-3))
            cm, crgb = mat("cable")
            self.tris += emit(self.stage, m + "/Hang/Cable/Line", Geo().cyl(0.006, 1.0, (0, 0, -0.5), n=6, caps=False),
                              cm, crgb)
            UsdGeom.Imageable(cab).MakeInvisible()
            self.cable = cab
            pod = xform(self.stage, m + "/Hang/Pod", (0, 0, -PD.EYE_Z))
            self.pod_t = UsdGeom.Xformable(pod).GetOrderedXformOps()[0]
        for kind, name, t, s, rgb, rot_z, attrs in parts:
            if kind == "Rotor":
                self._rotor(name, t, attrs, mat, Geo, emit, xform)
                continue
            g = Geo()
            R = rot_of(rot_z, attrs)
            if kind == "Cube":
                if attrs.get("Tube"):
                    g.cyl(min(s[1], s[2]) / 2, s[0], axis="x", n=10, R=R)
                elif attrs.get("Chamfer"):
                    c = min(attrs["Chamfer"], s[1] / 3, s[2] / 3)
                    w, h = s[1] / 2, s[2] / 2
                    prof = [(-w + c, -h), (w - c, -h), (w, -h + c), (w, h - c), (w - c, h), (-w + c, h), (-w, h - c), (-w, -h + c)]
                    from isaac.van import T as _T
                    g.prism(prof, -s[0] / 2, s[0] / 2, plane="yz", M=_T((0, 0, 0), R))
                else:
                    g.box(s, R=R)
            elif kind == "Cylinder":
                ax = attrs.get("Axis", "Z")
                ax = ax.lower() if isinstance(ax, str) else ax
                g.cyl(attrs["Radius"], attrs["Height"], axis=ax, n=16 if attrs["Radius"] > 0.03 else 8, R=R)
            elif kind == "Cone":
                ax = attrs.get("Axis", "Z")
                ax = ax.lower() if isinstance(ax, str) else ax
                g.cyl(attrs.get("Throat", 0.0), attrs["Height"], axis=ax, n=16, r_top=attrs["Radius"], caps=True, R=R)
            elif kind == "Sphere":
                zr = attrs.get("ZRange", (-1.0, 1.0))
                r = attrs["Radius"]
                g.sphere((r * s[0], r * s[1], r * s[2]), n=16 if r * max(s) > 0.05 else 10,
                         m=8 if r * max(s) > 0.05 else 6, R=R, z_range=zr)
            elif kind == "Torus":
                ax = attrs.get("Axis", "Z").lower()
                g.torus(attrs["Radius"], attrs.get("Tubeness", 0.01), axis=ax, n=48 if attrs["Radius"] > 0.2 else 12, m=6)
            elif kind == "Line":
                g.cyl_between((0, 0, 0), np.subtract(attrs["End"], t), 0.003, n=4, caps=False)
            if attrs.get("group") == "Pod":
                base = f"{m}/Hang/Pod/{name}"
                tl = (t[0], t[1], t[2] - POD_STOW_Z)
            else:
                base = f"{m}/{name}"
                tl = t
            mt, mrgb = mat(attrs.get("mat", "body"))
            mesh_x = xform(self.stage, base, tl)
            self.tris += emit(self.stage, base + "/Geo", g, mt, mrgb)
            mesh_x.GetPrim().CreateAttribute("larpverse:part", self._S().String).Set(name)

    def _S(self):
        from pxr import Sdf
        return Sdf.ValueTypeNames

    def _rotor(self, name, t, attrs, mat, Geo, emit, xform):
        UsdGeom = self.UsdGeom
        from isaac.van import Rx, Rz
        r = attrs["radius"]
        sign = attrs["sign"]
        rx = xform(self.stage, f"{self.model}/{name}", t)
        op = UsdGeom.Xformable(rx).AddRotateZOp()
        hub = Geo().cyl(0.05, 0.04, (0, 0, 0.0), n=12)
        hub.sphere((0.04, 0.04, 0.025), (0, 0, 0.02 * (1 if sign > 0 else 1)), n=10, m=4, z_range=(0, 1))
        mm, mrgb = mat("metal")
        self.tris += emit(self.stage, f"{self.model}/{name}/Hub", hub, mm, mrgb)
        blades = Geo()
        for k in (0, 1):                                  # two tapered, slightly pitched blades
            R = Rz(180.0 * k)
            L = r - 0.05
            for j, (x0, c) in enumerate(((0.05, 0.11), (0.05 + L * 0.5, 0.095), (0.05 + L * 0.85, 0.07))):
                seg = L * (0.5 if j == 0 else 0.35 if j == 1 else 0.15)
                blades.box((seg, c, 0.012), (0, 0, 0), M=_m4(R, (x0 + seg / 2) * _v(R, 1), Rx(sign * 7.0)))
        bm_, brgb = mat("blade")
        self.tris += emit(self.stage, f"{self.model}/{name}/Blades", blades, bm_, brgb)
        prims = {}
        for key, mk in (("lo", "blur_lo"), ("hi", "blur_hi")):
            g = Geo().cyl(r, 0.004, (0, 0, 0.004), n=32)
            if key == "hi":                               # darker rim where the blade tips sweep
                g.torus(r - 0.035, 0.012, (0, 0, 0.004), n=32, m=3)
            mt, mrgb = mat(mk)
            self.tris += emit(self.stage, f"{self.model}/{name}/Blur_{key}", g, mt, mrgb)
            prims[key] = self.stage.GetPrimAtPath(f"{self.model}/{name}/Blur_{key}")
            UsdGeom.Imageable(prims[key]).MakeInvisible()
        rot = dict(op=op, sign=sign, angle=0.0, state="still", name=name,
                   blades=self.stage.GetPrimAtPath(f"{self.model}/{name}/Blades"), lo=prims["lo"], hi=prims["hi"])
        self.rotors.append(rot)
        self.rotor_ops.append((op, sign))

    # ------------------------------------------------------------------ small model (USD primitives)
    def _shape(self, kind, path, t=(0, 0, 0), s=(1, 1, 1), rgb=(1, 1, 1), rot_z=None, **attrs):
        Gf, UsdGeom = self.Gf, self.UsdGeom
        g = getattr(UsdGeom, kind).Define(self.stage, path)
        for k, v in attrs.items():
            getattr(g, "Create" + k + "Attr")(v)
        if kind == "Cube":
            g.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
        xf = UsdGeom.Xformable(g)
        xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, t)))
        if rot_z is not None:
            xf.AddRotateZOp().Set(float(rot_z))
        xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, s)))
        g.CreateDisplayColorAttr([Gf.Vec3f(*map(float, rgb))])
        return g

    def _primitives(self, did):
        m, A = self.model, self.ARM
        acc = ACCENT.get(did, (0.9, 0.9, 0.9))
        self._shape("Cube", m + "/Body", (0, 0, 0.26), (0.30, 0.22, 0.10), COL["body"], Size=1.0)
        self._shape("Cube", m + "/Stripe", (0.0, 0, 0.315), (0.31, 0.06, 0.012), acc, Size=1.0)
        for i in range(4):
            a = math.radians(45 + 90 * i)
            cx, cy = A * math.cos(a), A * math.sin(a)
            self._shape("Cube", f"{m}/Arm{i}", (cx / 2, cy / 2, 0.27), (A, 0.035, 0.025), COL["arm"],
                        rot_z=math.degrees(a), Size=1.0)
            self._shape("Cylinder", f"{m}/Motor{i}", (cx, cy, 0.30), (1, 1, 1), COL["arm"],
                        Radius=0.025, Height=0.05, Axis="Z")
            rotor = self.UsdGeom.Xform.Define(self.stage, f"{m}/Rotor{i}")
            rxf = self.UsdGeom.Xformable(rotor)
            rxf.AddTranslateOp(self.UsdGeom.XformOp.PrecisionDouble).Set(self.Gf.Vec3d(cx, cy, 0.33))
            op = rxf.AddRotateZOp()
            self.rotor_ops.append((op, 1 if i % 2 else -1))
            self.rotors.append(dict(op=op, sign=1 if i % 2 else -1, angle=0.0, state="still", name=f"Rotor{i}",
                                    blades=None, lo=None, hi=None))
            disc = self._shape("Cylinder", f"{m}/Rotor{i}/Disc", (0, 0, 0), (1, 1, 1), COL["rotor"],
                               Radius=0.13, Height=0.004, Axis="Z")
            try:
                disc.CreateDisplayOpacityAttr([0.35])
            except Exception:
                pass
            self._shape("Cube", f"{m}/Rotor{i}/Blade", (0, 0, 0.004), (0.26, 0.025, 0.006), COL["blade"], Size=1.0)
        for side in (-1, 1):   # skids: two rails + four legs, rail bottom at z = 0
            self._shape("Cube", f"{m}/Skid{side + 1}", (0, side * 0.13, 0.01), (0.34, 0.02, 0.02), COL["skid"], Size=1.0)
            for fb in (-1, 1):
                self._shape("Cube", f"{m}/Leg{side + 1}{fb + 1}", (fb * 0.09, side * 0.12, 0.11),
                            (0.015, 0.015, 0.2), COL["skid"], Size=1.0)

    def _payload(self):
        m, A = self.model, self.ARM
        self._shape("Cylinder", m + "/LidarPuck", (0, 0, 0.345), (1, 1, 1), COL["lidar"], Radius=0.055, Height=0.07, Axis="Z")
        self._shape("Cube", m + "/RadarPanel", (0.17, 0, 0.2), (0.02, 0.16, 0.11), COL["radar"], Size=1.0)
        for i in range(4):
            a = math.radians(45 + 90 * i)
            self._shape("Sphere", f"{m}/Mic{i}", (0.8 * A * math.cos(a), 0.8 * A * math.sin(a), 0.2), (1, 1, 1),
                        COL["mic"], Radius=0.025)

    def _reference_asset(self, asset):
        """Reference Isaac's quadcopter under Model/Asset and strip every physics API. False on failure."""
        from pxr import Usd, UsdPhysics
        try:
            from isaac import compat
            urls = []
            if asset not in ("auto", True):
                urls.append(asset)
            root = compat.assets_root()
            if root:
                urls += [root + c for c in ASSET_CANDIDATES]

            def exists(u):
                return compat.url_exists(u) if "://" in u else os.path.exists(u)
            url = next((u for u in urls if u.endswith((".usd", ".usda", ".usdc")) and exists(u)), None)
            if url is None:
                print("[drone_viz] no quadcopter asset found; using primitives")
                return False
            prim = self.stage.DefinePrim(self.model + "/Asset", "Xform")
            prim.GetReferences().AddReference(url)
            if not prim.GetChildren():
                raise RuntimeError(f"{url} composed to an empty prim")
            mpu = compat.meters_per_unit(url)        # references don't convert units (cm assets come in 100x)
            if abs(mpu - 1.0) > 1e-6:
                from pxr import Gf, UsdGeom
                UsdGeom.Xformable(prim).AddScaleOp(UsdGeom.XformOp.PrecisionDouble, "units").Set(Gf.Vec3d(mpu, mpu, mpu))
            for _ in range(8):                         # de-instance so the physics APIs can be edited
                inst = [p for p in Usd.PrimRange(prim) if p.IsInstance()]
                if not inst:
                    break
                for p in inst:
                    p.SetInstanceable(False)
            for p in Usd.PrimRange(prim):
                if p.HasAPI(UsdPhysics.RigidBodyAPI):
                    UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
                    p.RemoveAPI(UsdPhysics.RigidBodyAPI)
                if p.HasAPI(UsdPhysics.ArticulationRootAPI):
                    p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                    try:                               # what look.py does on 6.0.1 (strip_physics)
                        from pxr import Sdf
                        p.CreateAttribute("physxArticulation:articulationEnabled", Sdf.ValueTypeNames.Bool).Set(False)
                    except Exception:
                        pass
                    try:
                        from pxr import PhysxSchema
                        p.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
                    except Exception:
                        pass
                if p.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
                if p.IsA(UsdPhysics.Joint):
                    UsdPhysics.Joint(p).CreateJointEnabledAttr(False)
            print(f"[drone_viz] {self.path}: referenced {url} (physics stripped)")
            return True
        except Exception as e:   # noqa: BLE001
            print(f"[drone_viz] asset reference failed ({e}); using primitives")
            try:
                self.stage.RemovePrim(self.model + "/Asset")
            except Exception:
                pass
            return False

    # ------------------------------------------------------------------ runtime
    def _strobe(self, dt):
        """Anti-collision strobe: a short white flash every STROBE[0] s (phase offset per drone)."""
        if not hasattr(self, "_strobe_img"):
            p = self.stage.GetPrimAtPath(f"{self.model}/Strobe")
            self._strobe_img = self.UsdGeom.Imageable(p) if p and p.IsValid() else None
            self._strobe_t = (hash(self.path) % 97) / 97.0 * STROBE[0]
            self._strobe_on = None
        if self._strobe_img is None:
            return
        self._strobe_t = (self._strobe_t + float(dt)) % STROBE[0]
        on = self._strobe_t < STROBE[1]
        if on != self._strobe_on:
            (self._strobe_img.MakeVisible if on else self._strobe_img.MakeInvisible)()
            self._strobe_on = on

    def set_pose(self, x, y, z, yaw_deg, tilt=None):
        roll, pitch = _tilt(tilt)
        self._apply(( float(x), float(y), float(z)), quat_ypr(yaw_deg, pitch, roll))

    def set_state(self, pos, quat, rotor_rpm=None, winch_len=None, dt=1 / 30.0, sway_deg=None, ground_z=None):
        """Flight-motion route. pos = the mission pose point (skid bottom + skid_offset), quat = (w, x, y, z) body
        attitude (tilted into the acceleration), rotor_rpm = 8 values in ROTOR_ORDER (or one value for all),
        winch_len = cable paid out (m), sway_deg = (roll, pitch) of the pod on its cable, ground_z = surface
        under the pod (it rests there instead of sinking)."""
        self._apply(tuple(map(float, pos[:3])), _quat_tuple(quat))
        if rotor_rpm is not None:
            self.set_rpm(rotor_rpm, dt)
        if winch_len is not None or sway_deg is not None:
            self.set_winch(self.winch if winch_len is None else winch_len,
                           self.sway if sway_deg is None else sway_deg, ground_z)
        elif self.hang is not None:
            self._apply_hang()

    def _apply(self, pos, q):
        Gf = self.Gf
        self.pos, self.q = pos, q
        self.t_op.Set(Gf.Vec3d(*pos))
        self.q_op.Set(Gf.Quatd(q[0], Gf.Vec3d(q[1], q[2], q[3])))
        if self.hang is not None:
            self._apply_hang()

    def set_rpm(self, rotor_rpm, dt):
        if isinstance(rotor_rpm, (int, float)):
            rotor_rpm = [float(rotor_rpm)] * len(self.rotors)
        rpm = list(rotor_rpm) or [0.0]
        UsdGeom = self.UsdGeom
        for k, r in enumerate(self.rotors):
            w = abs(float(rpm[k % len(rpm)]))
            r["angle"] = (r["angle"] + r["sign"] * w * 6.0 * dt) % 360.0
            r["op"].Set(float(r["angle"]))
            st = "blur" if w >= BLUR_RPM else ("spin" if w >= SLOW_RPM else "still")
            if st != r["state"] and r["blades"] is not None:
                vis = {"still": (1, 0, 0), "spin": (1, 1, 0), "blur": (0, 0, 1)}[st]
                for prim, v in zip((r["blades"], r["lo"], r["hi"]), vis):
                    (UsdGeom.Imageable(prim).MakeVisible if v else UsdGeom.Imageable(prim).MakeInvisible)()
            r["state"] = st
        self.rotor_angle = self.rotors[0]["angle"] if self.rotors else 0.0

    def spin(self, dt, on=True):
        """Older route: hover rpm while ``on`` (heavy model shows the blur disc); off keeps the blades still."""
        self._strobe(dt)
        if not self.heavy or not self.rotors:
            if not on:
                return
            self.rotor_angle = (self.rotor_angle + self.spin_rate * dt) % 360.0
            for op, sgn in self.rotor_ops:
                op.Set(float(sgn * self.rotor_angle))
            return
        self.set_rpm(HOVER_RPM if on else 0.0, dt)

    def winch_world_z(self, z):
        """World z of the cable exit for a pose point at height z (tilt ignored: < 2 cm at 10 deg)."""
        return float(z) - self.skid_offset + WINCH_BOTTOM

    def set_winch(self, length, sway_deg=(0.0, 0.0), ground_z=None):
        """Pay out ``length`` m of cable; the pod hangs plumb (counter-rotated against the body attitude) with a
        small (roll, pitch) sway, or rests on ``ground_z`` when the cable would put it below the surface."""
        self.winch = max(0.0, float(length))
        self.sway = tuple(float(v) for v in (sway_deg or (0.0, 0.0)))[:2]
        self.ground_z = ground_z
        if self.hang is not None:
            self._apply_hang()

    def _apply_hang(self):
        Gf, UsdGeom = self.Gf, self.UsdGeom
        from isaac.pods import EYE_Z
        L, sway = self.winch, self.sway
        if self.ground_z is not None:                 # exact cable-exit height under the current attitude
            w, x, y, z = self.q
            dz = WINCH_BOTTOM - self.skid_offset
            top = self.pos[2] + dz * (1 - 2 * (x * x + y * y))
            L_max = max(0.0, top - EYE_Z - float(self.ground_z))
            if L >= L_max - 1e-3:
                L, sway = L_max, (0.0, 0.0)
        w, x, y, z = self.q
        qd = Gf.Quatd(w, Gf.Vec3d(x, y, z))
        qs = quat_ypr(0.0, sway[1], sway[0])
        qh = qd.GetInverse() * Gf.Quatd(qs[0], Gf.Vec3d(*qs[1:]))
        self.hang_q.Set(qh)
        self.cable_s.Set(Gf.Vec3d(1, 1, max(L, 1e-3)))
        (UsdGeom.Imageable(self.cable).MakeVisible if L > 0.02 else UsdGeom.Imageable(self.cable).MakeInvisible)()
        self.pod_t.Set(Gf.Vec3d(0, 0, -L - EYE_Z))
        self.winch_eff = L

    def anchors_world(self, delivery="pod"):
        """{part name: world (x, y, z)} of every label anchor, from the live stage transforms (pod included)."""
        from pxr import Usd
        cache = self.UsdGeom.XformCache(Usd.TimeCode.Default())
        out = {}
        for a in component_anchors(delivery=delivery):
            sub = "Hang/Pod/" if a["parent"] == "pod" else ""
            p = self.stage.GetPrimAtPath(f"{self.model}/{sub}{a['part']}")
            if p.IsValid():
                t = cache.GetLocalToWorldTransform(p).ExtractTranslation()
                out[a["part"]] = (float(t[0]), float(t[1]), float(t[2]))
        return out


def _v(R, k):
    """k-th column of R (unit axis)."""
    import numpy as np
    return np.asarray(R)[:, 0] * k


def _m4(R, t, R2=None):
    import numpy as np
    M = np.eye(4)
    M[:3, :3] = R if R2 is None else R @ R2
    M[:3, 3] = t
    return M
