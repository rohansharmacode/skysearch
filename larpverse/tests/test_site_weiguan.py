"""Fast tests for the Weiguan case-study site (sim/site_weiguan.py) and external maps (sim/site_mesh.py)."""
import hashlib
import json

import numpy as np
import pytest

from sim.grid_world import GridWorld
from sim.scenario import make_scenario
from sim.site_weiguan import BOUNDS, SECTORS, _truth_support

GRID7_PUBLIC_MD5 = "c4ad614af1c481eaf10e6660a0dc5575"     # sim/scenario.py grid seed 7, before site= existed
GRID7_TRUTH_CORE_MD5 = "e37c314704ea13f88e72d54d08a5fd1b"   # victims/sounds/roles, physio excluded (vitals owns it)


@pytest.fixture(scope="module")
def wg():
    return make_scenario(7, site="weiguan")


def test_grid_default_unchanged():
    sc = make_scenario(7)
    assert hashlib.md5(sc.public_json().encode()).hexdigest() == GRID7_PUBLIC_MD5
    t = sc.truth
    core = {"start": t["start_pile"], "roles": t["roles"], "sounds": t["sounds"],
            "v": [{k: v[k] for k in ("id", "pile", "xy", "depth", "sound")} for v in t["victims"]]}
    assert hashlib.md5(json.dumps(core, sort_keys=True).encode()).hexdigest() == GRID7_TRUTH_CORE_MD5
    assert make_scenario(7, site="grid").public_json() == sc.public_json()


def test_deterministic_and_geometry_seed_independent(wg):
    again = make_scenario(7, site="weiguan")
    assert again.public_json() == wg.public_json() and again.truth_json() == wg.truth_json()
    other = make_scenario(3, site="weiguan")
    strip = lambda d: {k: v for k, v in d.items() if k not in ("seed", "start_pile")}   # noqa: E731
    assert strip(other.public_dict()) == strip(wg.public_dict())      # one real site: geometry fixed
    assert other.truth_json() != wg.truth_json()


def test_sectors_and_pieces(wg):
    assert [p.name for p in wg.piles] == list(SECTORS) == list("ABCDEFGHI")
    pcs = [pc for p in wg.piles for pc in p.pieces]
    assert 500 < len(pcs) <= 2000
    c = np.array([pc.center for pc in pcs])
    x0, y0, x1, y1 = BOUNDS
    assert (c[:, 0] > x0).all() and (c[:, 0] < x1).all() and (c[:, 1] > y0).all() and (c[:, 1] < y1).all()
    assert (c[:, 2] > -1).all() and (c[:, 2] < 16).all()
    d = wg.public_dict()
    assert d["site"] == "weiguan" and d["bounds"] == list(BOUNDS) and d["geo"]["lat"] == pytest.approx(23.0051432)
    json.dumps(d)


def test_truth_victims_in_voids(wg):
    _, sup = _truth_support()
    hm = wg.heightmap(0.5)
    V = wg.truth["victims"]
    assert len(V) >= 8
    assert sum(v["depth"] > 4.0 for v in V) >= 3                        # radar-unreachable, acoustic-only
    assert sum(0.5 <= v["depth"] <= 3.0 for v in V) >= len(V) // 2
    for v in V:
        assert v["pile"] in SECTORS and set(v["physio"]) >= {"state", "rate_bpm"}
        assert hm.height_at(*v["xy"]) - v["z"] == pytest.approx(v["depth"], abs=2e-3)
        assert not sup.solids.inside((*v["xy"], v["z"]), margin=0.2)
        if v["depth"] > 4.0:
            assert v["sound"] in ("knock", "weak_voice", "voice")       # findable acoustically
    langs = {s.get("lang") for s in wg.truth["sounds"]} - {None}
    assert langs == {"cmn", "nan", "en", "id"}
    assert wg.truth["start_pile"] in SECTORS and len(wg.truth["roles"]) == 8


def test_gridworld_from_weiguan(wg):
    w = GridWorld(wg)
    assert w.grid.z.shape == (212, 200) and w.grid.z.max() > 10
    for name in "AE":
        assert w.window(name).z.shape == (60, 60)
        assert w.pile_world(name).targets == [t for t in w.targets if t["pile"] == name]


def _box_mesh(x0, y0, x1, y1, h):
    V = np.array([[x, y, z] for z in (0, h) for y in (y0, y1) for x in (x0, x1)], float)
    F = [[4, 5, 7, 6], [0, 1, 3, 2], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
    return V, F


def test_mesh_npz_and_ply(tmp_path):
    res = 0.5
    x = np.arange(-60, 60, res) + res / 2
    X, Y = np.meshgrid(x, x)
    z = np.zeros_like(X)
    z[(np.abs(X + 25) < 8) & (np.abs(Y) < 10)] = 6.0           # two wrecks
    z[(np.abs(X - 25) < 6) & (np.abs(Y - 20) < 6)] = 4.0
    p = tmp_path / "m.npz"
    np.savez(p, origin=np.array([-60.0, -60.0]), res=res, z=z.astype(np.float32))
    sc = make_scenario(5, site=f"mesh:{p}")
    names = [q.name for q in sc.piles]
    assert names == ["W1", "W2"]
    assert sc.pile("W1").x == pytest.approx(-25, abs=1) and sc.pile("W2").x == pytest.approx(25, abs=1)   # by distance
    assert sc.heightmap(0.5).z.max() == pytest.approx(6.0)
    assert sc.truth["victims"] and all(v["pile"] in names for v in sc.truth["victims"])
    json.dumps(sc.public_dict())
    # ascii PLY mesh -> same surface heights
    V, F = _box_mesh(-10, -5, 10, 5, 7.5)
    ply = tmp_path / "b.ply"
    with open(ply, "w") as f:
        f.write(f"ply\nformat ascii 1.0\nelement vertex {len(V)}\nproperty float x\nproperty float y\nproperty float z\n"
                f"element face {len(F)}\nproperty list uchar int vertex_indices\nend_header\n")
        f.writelines(f"{a} {b} {c}\n" for a, b, c in V)
        f.writelines(f"{len(q)} {' '.join(map(str, q))}\n" for q in F)
    from sim.site_mesh import load_surface
    g, _, _ = load_surface(str(ply), res=0.5, bounds=(-40, -40, 40, 40))
    assert g.height_at(0, 0) == pytest.approx(7.5) and g.height_at(-30, 30) == pytest.approx(0.0)


def test_expanded_map_names_weiguan_blocks(tmp_path):
    from sim.site_mesh import EXPANDED_LAYOUT, synth_expanded_npz
    p = synth_expanded_npz(str(tmp_path / "xl.npz"))
    sc = make_scenario(7, site=f"mesh:{p}")
    names = {q.name for q in sc.piles}
    assert set("ABCDEFGHI") <= names and {b["name"] for b in EXPANDED_LAYOUT} <= names
    assert all(np.hypot(q.x, q.y) < 160 for q in sc.piles)
    assert sum(v["depth"] > 4.0 for v in sc.truth["victims"]) >= 2


def test_collapse_site_from_cache():
    """site="collapse": the committed 0.5 m heightmap of the teammate's USD, generator footprints as sectors."""
    from sim.site_collapse import CORE, cache_path
    if cache_path() is None:
        pytest.skip("data/maps/collapse_site_heightmap.npz missing (python -m sim.site_collapse --build)")
    sc = make_scenario(7, site="collapse")
    d = sc.public_dict()
    names = [p.name for p in sc.piles]
    assert set(CORE) <= set(names) and len(names) > 120
    assert d["bounds"] == [-160.0, -160.0, 160.0, 160.0] and d["geo"]["lat"] == pytest.approx(23.0051432)
    hm = sc.heightmap(0.5)
    X, Y = hm.mesh()
    bx, by = d["base_xy"]
    assert hm.z[np.hypot(X - bx, Y - by) < 6].max() < 0.4                 # staging on clear ground
    for e in d["piles"]:                                                    # contracts v1.2.1: CCW polygon
        P = np.asarray(e["polygon"], float)
        assert len(P) >= 3 and np.sum(P[:, 0] * np.roll(P[:, 1], -1) - np.roll(P[:, 0], -1) * P[:, 1]) > 0
    kinds = {e["kind"] for e in d["piles"]}
    assert "standing" not in kinds and "tilted" not in kinds              # obstacles, not sectors
    assert d["site_info"]["max_obstacle_h"] >= 20
    t = sc.truth
    assert t["start_pile"] in CORE and all(n in CORE for n, k in t["roles"].items() if k != "silent")
    assert sum(v["depth"] > 4 for v in t["victims"]) >= 2
    for v in t["victims"]:
        assert hm.height_at(*v["xy"]) - v["z"] == pytest.approx(v["depth"], abs=2e-3)
    assert make_scenario(7, site="weiguan-real").truth_json() == sc.truth_json()
