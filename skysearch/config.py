"""All tunable parameters in one place. Units: metres, seconds."""
from dataclasses import dataclass


@dataclass
class Config:
    area: float = 1000.0          # square search area side
    cell: float = 20.0            # belief-grid cell size
    dt: float = 0.5               # control tick
    n_drones: int = 5
    search_alt: float = 60.0      # wide thermal sweep
    inspect_alt: float = 18.0     # low pass for RGB confirmation
    speed: float = 12.0           # horizontal cruise
    climb: float = 4.0            # vertical speed
    fov_deg: float = 60.0         # thermal camera field of view
    comm_range: float = 450.0     # radio link range (drone-drone and drone-base)
    battery_s: float = 420.0      # flight endurance at cruise (short so swaps show in demo)
    reserve_frac: float = 0.10    # battery kept in reserve on top of the trip home
    swap_s: float = 15.0          # battery swap time at base
    sep: float = 30.0             # min horizontal separation between drones at same altitude
    tree_height: float = 22.0
    lidar_range: float = 15.0
    duration: float = 300.0
    n_people: int = 2
    n_deer: int = 6
    seed: int = 7

    @property
    def n(self) -> int:
        return int(self.area / self.cell)
