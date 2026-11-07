"""Parameters for the earthquake (urban search and rescue) scenario. Units: metres, seconds, dB."""
from dataclasses import dataclass, field


@dataclass
class DroneSpec:
    kind: str
    size: float          # body diameter (m)
    min_clear: float     # vertical clearance it needs to fly through a cell
    speed: float         # m/s indoors
    battery_s: float     # endurance
    radio_db: float      # max tolerable path loss for a link
    color: str


@dataclass
class USARConfig:
    cell: float = 0.4
    W: int = 90                    # grid columns (36 m)
    H: int = 60                    # grid rows (24 m)
    dt: float = 0.5
    duration: float = 600.0
    seed: int = 3
    n_scouts: int = 3
    n_micro: int = 6
    n_survivors: int = 3
    n_decoys: int = 3
    lidar_range: float = 10.0
    lidar_rays: int = 72
    ultra_range: float = 3.0
    cam_range: float = 3.0
    cam_fov_deg: float = 100.0
    micro_map_range: float = 2.0   # micro ToF/camera local mapping
    quiet_every: float = 90.0      # coordinator calls a quiet period ("everyone stop, listen")
    quiet_len: float = 6.0
    swap_s: float = 15.0           # automatic battery swap on the mothership's docking pads
    n_listeners: int = 4           # micro-drones that land on flat spots and listen (rest are confirmers)
    n_sentinels: int = 2           # listeners that stay landed all mission; the others lift off to search after the call-out
    scout_cam_range: float = 5.0
    radar_range_micro: float = 2.6 # 60 GHz mmWave breathing radar on micro-drones
    radar_range_scout: float = 4.0 # UWB through-rubble radar on the radar scout (S3)
    radar_scouts: tuple = (3,)     # which scouts (1-based) carry the UWB radar
    micro_radar: bool = True       # confirmers carry the 60 GHz breathing radar (ablation switch)
    story: bool = False            # place victims / fire / knock per the team's demo storyboard
    t_fire: float = 48.0           # story: post-quake fire starts in the east wing
    t_knock_b: float = 110.0       # story: victim B (behind a shelf in room 7) regains consciousness
    mother: DroneSpec = field(default_factory=lambda: DroneSpec("mother", 1.4, 99, 3.0, 1500, 110, "#FFD54F"))
    scout: DroneSpec = field(default_factory=lambda: DroneSpec("scout", 0.6, 0.9, 2.0, 900, 100, "#4FC3F7"))
    micro: DroneSpec = field(default_factory=lambda: DroneSpec("micro", 0.15, 0.25, 1.2, 420, 92, "#F06292"))
