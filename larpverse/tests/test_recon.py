"""Fast checks for reconnaissance mode (sim/recon.py, contracts v1.2): coverage path, perches, proposals."""
import numpy as np
import pytest

from sim.coordinator import KinematicFlight
from sim.grid_world import GridWorld
from sim.landing import LIM_SLOPE, patch_fields
from sim.recon import Recon, coverage_path, path_coverage, proposals_to_tasking
from sim.scenario import DRONE_PADS, SITE_BOUNDS, make_scenario
from sim.tasking import points_in_polygon


@pytest.fixture(scope="module")
def recon():
    sc = make_scenario(7)
    w = GridWorld(sc)
    fl = KinematicFlight(w.grid, starts={f"d{i + 1}": xy for i, xy in enumerate(DRONE_PADS)})
    rc = Recon(w.grid, fl, bounds=SITE_BOUNDS, pile_xy=w.pile_xy)
    t, heard = 0, []
    while rc.status in ("mapping", "listening") and t < 2000:
        t += 1
        rc.step(t, fl.step(1.0, t))
        if rc.wants_listen:
            heard.append(dict(rc.wants_listen))
            d = next(iter(rc.wants_listen))
            rc.add_listen([{"id": f"r{len(heard)}-x", "drone_id": d, "class": "voice", "scores": {"human": 0.9},
                            "priority": 70, "pile_id": "B2"}], t)
    return sc, w, rc, heard, t


def test_path_covers_bounds():
    paths = coverage_path(SITE_BOUNDS, 3)
    assert len(paths) == 3 and all(len(p) >= 2 for p in paths)
    assert path_coverage(paths, SITE_BOUNDS) >= 0.9
    x0, y0, x1, y1 = SITE_BOUNDS
    assert all(x0 <= x <= x1 and y0 <= y <= y1 for p in paths for x, y in p)
    aoi = [[-20, -20], [20, -20], [20, 20], [-20, 20]]
    pa = coverage_path(SITE_BOUNDS, 3, aoi=aoi)
    assert path_coverage(pa, (-20, -20, 20, 20)) >= 0.9
    assert all(-20 <= x <= 20 and -20 <= y <= 20 for p in pa for x, y in p)


def test_recon_maps_site_and_perches_on_flat_cells(recon):
    sc, w, rc, heard, t = recon
    assert rc.status == "proposing" and t < 2000
    assert rc.mapped_fraction >= 0.9
    assert rc.alt >= float(w.grid.z.max()) + 12.0 - 0.6          # altitude from the mapped max + margin
    assert heard and all(len(h) >= 2 for h in heard)
    slope, rough, *_ = patch_fields(w.grid)
    for p in rc.perches:
        i, j = w.grid.index(*p["xy"])
        assert slope[j, i] <= LIM_SLOPE + 1e-6
    xy = np.array([p["xy"] for p in rc.perches])
    for k in range(0, len(xy), 3):                                # perches of a round are spread out
        r = xy[k:k + 3]
        d = np.hypot(*(r[:, None, :] - r[None, :, :]).transpose(2, 0, 1))
        assert d[np.triu_indices(len(r), 1)].min() >= 15.0 - 1e-6


def test_proposals_contain_the_nine_piles(recon):
    sc, w, rc, _, _ = recon
    props = rc.proposals
    assert 9 <= len(props) <= 11
    for name, (x, y) in w.pile_xy.items():
        assert sum(points_in_polygon(x, y, p["polygon"]) for p in props) == 1, name
    for p in props:
        assert p["kind"] == "rubble_mound" and 0 <= p["score"] <= 1 and p["area_m2"] < 600
        assert any(points_in_polygon(x, y, p["polygon"]) for x, y in w.pile_xy.values()), "false mound"
    b2 = next(p for p in props if points_in_polygon(*w.pile_xy["B2"], p["polygon"]))
    assert b2["acoustic_leads"] and b2 is props[0]                  # an acoustic lead ranks first
    st = rc.to_state()
    assert set(st["recon"]) == {"status", "mapped_fraction", "path", "perches"}
    assert all(len(q) == 3 for q in st["recon"]["path"]) and st["zone_proposals"] == props
    tk = proposals_to_tasking(props, top_k=4)
    assert [z.id for z in tk.zones] == [p["id"] for p in props[:4]] and tk.source == "auto-accepted-proposals"


def test_heightmap_state_marks_unmapped_cells(recon):
    _, _, rc, _, _ = recon
    hm = rc.heightmap_state(every_s=0)
    assert len(hm["z"]) == len(hm["mask"]) == hm["w"] * hm["h"]
    assert all((m == 0) == (z is None) for z, m in zip(hm["z"], hm["mask"]))


def test_large_site_path_and_standing_buildings():
    from sim.recon import segment_mounds
    big = (-160.0, -160.0, 160.0, 160.0)
    assert path_coverage(coverage_path(big, 3, lidar_radius_m=25.0), big, radius_m=25.0) >= 0.9
    res = 0.5
    n = 240                                           # 120 m x 120 m synthetic block: one tower, one mound
    X, Y = np.meshgrid((np.arange(n) + 0.5) * res, (np.arange(n) + 0.5) * res)
    z = 0.02 * np.sin(X / 7.0)
    tower = (np.abs(X - 30) < 9) & (np.abs(Y - 30) < 12)
    z = np.where(tower, 22.0, z)
    z = z + 6.0 * np.exp(-((X - 85) ** 2 + (Y - 80) ** 2) / 60.0) * (1 + 0.1 * np.sin(3 * X) * np.cos(3 * Y))
    standing = []
    mounds = segment_mounds(z, np.ones_like(z, bool), (0.0, 0.0), res, standing=standing)
    assert len(standing) == 1 and standing[0]["height_m"] == pytest.approx(22.0, abs=0.5)
    assert len(mounds) == 1 and points_in_polygon(85, 80, mounds[0]["polygon"])
    assert not points_in_polygon(30, 30, mounds[0]["polygon"])


def test_collapse_site_zones_and_recon():
    """320 m collapse site: default zones = rubble sectors only; recon maps it with 3 drones (battery swaps),
    proposals cover the sectors and leave the standing buildings (site_info.obstacles prior) out."""
    try:
        sc = make_scenario(7, site="collapse")
    except Exception as e:                                         # noqa: BLE001  (site cache not built here)
        pytest.skip(f"collapse site unavailable: {e}")
    from sim.tasking import ZoneTracker, default_tasking
    pd = sc.public_dict()
    tk = default_tasking(sc)
    assert len(tk.zones) == len(sc.piles) >= 100
    tr = ZoneTracker(tk, piles=sc.piles, bounds=pd["bounds"])
    assert all(tr.zone_piles(z) for z in tr.zone_ids())
    g = sc.heightmap(0.5)
    fl = KinematicFlight(g, starts={f"d{i + 1}": tuple(xy) for i, xy in enumerate(pd["drone_pads"][:3])})
    obs = pd["site_info"]["obstacles"]
    rc = Recon(g, fl, bounds=pd["bounds"], obstacles=obs)
    t = 0
    while rc.status in ("mapping", "listening") and t < 4000:
        t += 1
        rc.step(t, fl.step(1.0, t))
        if rc.wants_listen:
            rc.add_listen([], t)
    assert rc.status == "proposing" and rc.mapped_fraction >= 0.9
    props = rc.proposals
    inside = sum(any(points_in_polygon(p.x, p.y, q["polygon"]) for q in props) for p in sc.piles)
    assert inside >= 0.85 * len(sc.piles)
    swallowed = [o for o in obs if any(points_in_polygon(*np.mean(o["polygon"], 0), q["polygon"]) for q in props)]
    assert len(swallowed) <= 5
