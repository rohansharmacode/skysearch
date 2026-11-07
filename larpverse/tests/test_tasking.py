"""Fast checks for operator tasking zones (sim/tasking.py, contracts v1.2)."""
import json

import numpy as np
import pytest

from sim.scenario import make_scenario
from sim.tasking import (Tasking, ZoneTracker, default_tasking, distance_to_polygon, load_tasking,
                         piece_footprint, points_in_polygon, polygon_area, polygon_centroid, rasterise)

SQ = [[0, 0], [10, 0], [10, 10], [0, 10]]
ZONE_KEYS = {"id", "name", "polygon", "status", "coverage", "pings", "last_searched_t", "retask_reason"}


@pytest.fixture(scope="module")
def sc():
    return make_scenario(7)


def doc(zones, rev=1):
    return Tasking.from_dict({"schema": "larpverse.tasking/v1", "revision": rev, "source": "operator", "zones": zones})


def test_geometry():
    x = np.array([5, 11, -1, 9.9, 5])
    y = np.array([5, 5, 5, 9.9, 10.5])
    assert points_in_polygon(x, y, SQ).tolist() == [True, False, False, True, False]
    concave = [[0, 0], [10, 0], [10, 10], [5, 3], [0, 10]]
    assert not points_in_polygon(5, 6, concave) and points_in_polygon(5, 1, concave)
    assert polygon_area(SQ) == pytest.approx(100) and polygon_centroid(SQ) == pytest.approx((5, 5))
    assert distance_to_polygon(13, 5, SQ) == pytest.approx(3) and distance_to_polygon(5, 5, SQ) == 0
    m = rasterise(SQ, (0, 0), 0.5, 40, 40)
    assert m.sum() == 400 and m[:20, :20].all() and not m[20:, :].any()


def test_default_zones_contain_their_piles(sc):
    tk = default_tasking(sc)
    assert tk.source == "scenario-default" and [z.id for z in tk.zones] == [p.name for p in sc.piles]
    for z, p in zip(tk.zones, sc.piles):
        corners = np.concatenate([piece_footprint(pc) for pc in p.pieces])
        assert points_in_polygon(corners[:, 0], corners[:, 1], z.polygon).all()
        assert distance_to_polygon(corners[:, 0], corners[:, 1], z.polygon).max() == 0
        others = [q for q in sc.piles if q is not p]
        assert not any(points_in_polygon(q.x, q.y, z.polygon) for q in others)
    tr = ZoneTracker(tk, piles=sc.piles)
    assert all(tr.zone_piles(k) == [k] for k in tr.zone_ids())


def test_load_and_state_shape(tmp_path, sc):
    assert load_tasking(tmp_path / "missing.json") is None
    p = tmp_path / "tasking.json"
    p.write_text(json.dumps({"zones": []}))
    assert load_tasking(p) is None and load_tasking(p, allow_empty=True).zones == []
    p.write_text(json.dumps({"revision": 3, "zones": [{"id": "Z1", "name": "east", "polygon": SQ},
                                                      {"id": "bad", "polygon": [[0, 0], [1, 1]]}]}))
    tk = load_tasking(p)
    assert tk.revision == 3 and [z.id for z in tk.zones] == ["Z1"]
    st = ZoneTracker(tk).to_state()
    assert set(st) == {"zones", "zone_suggestions"} and set(st["zones"][0]) == ZONE_KEYS
    assert st["zones"][0]["status"] == "unsearched"
    assert set(st["zones"][0]["coverage"]) == {"lidar_mapped", "radar_reached", "listened"}


def test_status_transitions_and_coverage(sc):
    tr = ZoneTracker(default_tasking(sc), piles=sc.piles)
    z = "A2"
    cx, cy = tr.zone(z)["centroid"]
    assert tr.should_visit(z)
    tr.start_search(z, 10.0)
    assert tr.zone(z)["status"] == "surveying" and tr.should_visit(z)
    tr.mark_lidar_disc((cx, cy), 18.0)
    tr.mark_scanning(z, 40.0)
    assert tr.zone(z)["status"] == "scanning"
    for k, (dx, dy) in enumerate([(0, 0), (4, 0), (-4, 0), (0, 4), (0, -4), (3, 3), (-3, -3), (3, -3), (-3, 3)]):
        assert tr.add_scan({"scan_id": f"s{k}", "sensor_pose": [cx + dx, cy + dy, 3.0, 0], "quality": 0.9,
                            "max_reliable_depth_m": 9.0})
    assert not tr.add_scan({"scan_id": "s0", "sensor_pose": [cx, cy, 3, 0]})       # idempotent
    cov = tr.coverage(z)
    assert cov["lidar_mapped"] == 1.0 and 0.3 < cov["radar_reached"] < 1.0      # capped at 3 m reach
    assert tr.finish_search(z, 200.0, pings=["A2-R1"]) == "ping"
    assert tr.zone(z)["pings"] == ["A2-R1"] and not tr.should_visit(z)
    # a zone scanned from one spot only is beyond radar reach
    tr.start_search("B2", 210.0)
    bx, by = tr.zone("B2")["centroid"]
    tr.add_scan({"scan_id": "s99", "sensor_pose": [bx + 7, by, 1, 0], "max_reliable_depth_m": 3.0})
    assert tr.finish_search("B2", 300.0) == "beyond_radar_reach" and not tr.should_visit("B2")
    tr.mark_listened({"d1": (cx, cy - 30)})
    assert tr.coverage("A3")["listened"] and not tr.coverage("C1")["listened"]


def _clip(cid, pile, t, human=0.9, cls="voice"):
    return {"id": cid, "pile_id": pile, "t": t, "class": cls, "transcript": "help", "scores": {"human": human},
            "drone_id": "d1", "origin": [0, -40], "bearing_deg": 90.0}


def test_no_revisit_without_new_evidence_then_retask_on_new_clip(sc):
    tr = ZoneTracker(default_tasking(sc), piles=sc.piles)
    tr.add_clip(_clip("1-ac-B1", "B1", 50.0))            # heard before the search: not new evidence
    tr.start_search("B1", 60.0)
    tr.add_scan({"scan_id": "s1", "sensor_pose": [*tr.zone("B1")["centroid"], 2, 0]})
    for k in range(6):
        x, y = tr.zone("B1")["centroid"]
        tr.add_scan({"scan_id": f"s{k + 2}", "sensor_pose": [x + 4 * np.cos(k), y + 4 * np.sin(k), 2, 0]})
    assert tr.finish_search("B1", 300.0) == "searched_no_ping"
    assert not tr.should_visit("B1") and tr.zone("B1")["retask_reason"] is None
    assert tr.add_clip(_clip("1-ac-B1", "B1", 50.0)) == "B1" and not tr.should_visit("B1")   # same clip again
    assert tr.add_clip(_clip("2-ac-B1", "B1", 320.0, human=0.1, cls="dog")) == "B1"
    assert not tr.should_visit("B1")                                                     # non-human: no reopen
    assert tr.add_clip(_clip("3-ac-B1", "B1", 330.0)) == "B1"
    assert tr.should_visit("B1") and "3-ac-B1" in tr.zone("B1")["retask_reason"]
    assert "B1" in tr.visit_candidates()
    tr.start_search("B1", 400.0)
    assert tr.zone("B1")["visits"] == 2 and tr.zone("B1")["retask_reason"]          # reason kept as record


def test_suggestions_for_outside_clips():
    tr = ZoneTracker(doc([{"id": "Z1", "polygon": SQ}]), pile_xy={"C1": (40.0, 40.0)})
    inside = _clip("a1", None, 5.0)
    inside.update(xy=[5, 5])
    assert tr.add_clip(inside) == "Z1"
    out1 = {"id": "a2", "t": 6, "class": "voice", "transcript": "help me", "scores": {"human": 0.9},
            "bearings": [{"drone_id": "d1", "origin": [0, 30], "bearing_deg": 0.0},
                         {"drone_id": "d3", "origin": [30, 0], "bearing_deg": 90.0}]}       # crosses at (30, 30)
    out2 = _clip("a3", "C1", 7.0)                                                          # pile C1 at (40, 40)
    out3 = dict(_clip("a4", None, 8.0, human=0.1), origin=[60, -40])                        # non-human: ignored
    for c in (out1, out2, out3):
        assert tr.add_clip(c) is None
    s = tr.suggest_zone([out1, out2, out3])
    assert len(s) == 2 and {tuple(x["evidence"]) for x in s} == {("a2",), ("a3",)}
    a2 = next(x for x in s if x["evidence"] == ["a2"])
    assert a2["centre"] == pytest.approx([30, 30], abs=0.2) and a2["radius_m"] >= 6
    assert "d1 and d3" in a2["why"] and "bearing overlap" in a2["why"]
    ids = {x["id"] for x in s}
    assert {x["id"] for x in tr.suggest_zone([])} == ids                                    # stable ids
    assert tr.to_state()["zone_suggestions"] == s


def test_tasking_revision_merge():
    tr = ZoneTracker(doc([{"id": "Z1", "polygon": SQ}, {"id": "Z2", "polygon": [[20, 0], [30, 0], [30, 10]]}]))
    tr.start_search("Z1", 1.0)
    tr.add_scan({"scan_id": "s1", "sensor_pose": [5, 5, 0, 0]})
    tr.finish_search("Z1", 50.0)
    st1 = tr.zone("Z1")["status"]
    msgs = tr.update_tasking(doc([{"id": "Z1", "polygon": SQ}, {"id": "Z2", "polygon": [[20, 0], [30, 0], [30, 10]],
                                                                  "status": "cancelled"},
                                  {"id": "Z3", "polygon": [[0, 20], [10, 20], [10, 30]]}], rev=2))
    assert any("Z3" in m for m in msgs) and any("Z2 cancelled" in m for m in msgs)
    assert tr.zone("Z1")["status"] == st1 and tr.zone("Z1")["last_searched_t"] == 50.0     # progress kept
    assert tr.zone("Z2")["status"] == "cancelled" and not tr.should_visit("Z2")
    assert tr.zone("Z3")["status"] == "unsearched" and tr.should_visit("Z3")
    assert not tr.should_visit("Z1")
    assert tr.update_tasking(doc([{"id": "Z1", "polygon": SQ}, {"id": "Z2", "polygon": [[20, 0], [30, 0], [30, 10]],
                                                                  "status": "cancelled"},
                                  {"id": "Z3", "polygon": [[0, 20], [10, 20], [10, 30]]}], rev=2)) == []
    # operator re-task of a finished zone, and removal of an untouched one
    tr.update_tasking(doc([{"id": "Z1", "polygon": SQ, "retask": 1}, {"id": "Z2", "polygon": [[20, 0], [30, 0], [30, 10]]}],
                          rev=3))
    assert tr.should_visit("Z1") and "operator" in tr.zone("Z1")["retask_reason"]
    assert "Z3" not in tr.zones and tr.zone("Z2")["status"] == "unsearched"
