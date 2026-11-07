"""Isaac scene assets (isaac/van.py, isaac/dummies.py, isaac/drone_viz.py, isaac/pods.py) checked offline with
usd-core: prim paths, transforms, material bindings, SpatialAudio prims + larpverse:* attributes, triangle
budgets, the live-loop API (read_dummies / set_active) and the flight-motion API (set_state / set_winch)."""
import json
import math
import os

import numpy as np
import pytest

from sim.scenario import make_scenario

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ------------------------------------------------------------------------------ pure python (no pxr)
def test_plan_dummies_grid_seed7_matches_manifest():
    from isaac import dummies as DM
    from isaac.site import victim_z
    man = DM.load_manifest()
    assert man is not None and man["schema"].startswith("larpverse.dummy_audio")
    sc = make_scenario(7)
    plan = {e["id"]: e for e in DM.plan_dummies(sc, man)}
    assert {"A2", "A3", "B1", "B2", "C2_dog", "C1_generator", "B3_settling", "H1_C3_vi", "H2_A1_id"} <= set(plan)
    truth = {v["id"]: v for v in sc.truth["victims"]}
    for e in plan.values():
        if e["victim"]:
            v = truth[e["victim"]]
            assert e["physio"] == v["physio"]["state"] and e["human"]
            assert e["xyz"][2] == pytest.approx(victim_z(sc, v))
            assert e["active"] and not e["hidden"]
        for c in e["clips"] + e["reply_clips"]:
            assert os.path.exists(os.path.join(REPO, c)), c
    assert plan["A3"]["clip"] and plan["A3"]["kind"] == "voice" and plan["A3"]["reply_clips"]
    assert plan["A2"]["clip"] == "" and plan["A2"]["kind"] == "silent"
    assert plan["C2_dog"]["physio"] == "none" and not plan["C2_dog"]["human"]
    h = plan["H1_C3_vi"]
    assert h["hidden"] and not h["active"] and h["pile"] == "C3" and h["clip"] and h["physio"] == "calm"
    p = {q.name: q for q in sc.piles}["C3"]
    assert math.hypot(h["xyz"][0] - p.x, h["xyz"][1] - p.y) < max(p.sx, p.sy)
    assert h["xyz"][2] == pytest.approx(sc.ground_height(*h["xyz"][:2]) - 0.5)


def test_plan_dummies_collapse_seed7():
    from isaac import dummies as DM
    sc = make_scenario(7, site="collapse")
    plan = DM.plan_dummies(sc, DM.load_manifest())
    humans = [e for e in plan if e["victim"]]
    hidden = [e for e in plan if e["hidden"]]
    assert len(humans) == len(sc.truth["victims"]) == 22
    assert len(hidden) == 2 and all(not e["active"] for e in hidden)
    piles = {p.name for p in sc.piles}
    victim_piles = {v["pile"] for v in sc.truth["victims"]}
    assert all(e["pile"] in piles and e["pile"] not in victim_piles for e in hidden)
    assert not any(e["kind"] in ("dog", "generator", "settling") for e in plan)      # grid-only distractors
    assert len({e["id"] for e in plan}) == len(plan)
    voiced = [e for e in humans if e["kind"] in ("voice", "weak_voice", "knock")]
    assert voiced and all(e["clip"] for e in voiced)


def test_heavy_parts_follow_the_bom():
    from isaac.drone_viz import ROTOR_ORDER, _bom_mounts, component_anchors, heavy_parts, load_bom, model_extent
    from sim.platform import get_platform
    bom = load_bom()
    assert bom is not None
    spec = get_platform("heavy_sar")
    parts = {p[1]: p for p in heavy_parts(spec, "d1")}
    assert "RadarPanel" not in parts                          # pod mode: the radar lives in the pod
    assert "RadarPanel" in {p[1] for p in heavy_parts(spec, "d1", delivery="land")}
    for m in bom["isaac_mounts"]:
        part, pos = m.get("part"), m.get("pos_m")
        if part in parts and isinstance(pos, list):
            got = np.asarray(parts[part][2])
            if m.get("parent") == "pod":
                got = got - (0, 0, 0.175)
            assert np.allclose(got, pos, atol=1e-6), part
    for need in ("LidarPuck", "Gimbal", "Thermal", "SpeakerHorn", "ComputeBox", "GnssMast", "MeshAntennaL",
                 "MeshAntennaR", "MeshRadio", "WifiApAntenna", "PodLinkAntenna", "Winch", "PodRadarFace",
                 "PodMic0", "PodMic3", "PodLiftEye", "PodStrobe", "PodAntenna", "Battery0", "Skid0", "LedArm0"):
        assert need in parts, need
    assert parts["LidarPuck"][6]["Radius"] == pytest.approx(0.043)
    assert sum(p[0] == "Rotor" for p in parts.values()) == 8 == len(ROTOR_ORDER)
    assert sum(n.startswith("Mic") for n in parts) == spec.mics.n_mics
    ex, ey, z0, z1 = model_extent(list(parts.values()))
    assert abs(max(ex, ey) - spec.span_m) < 0.3 and z0 == pytest.approx(0.0, abs=1e-6) and 1.0 < z1 < 1.6
    keys = {a["key"]: a for a in component_anchors()}
    assert keys["lidar"]["bom_name"].startswith("Ouster") and keys["pod_radar"]["parent"] == "pod"
    assert set(_bom_mounts(bom)) & {a["part"] for a in keys.values()}


def test_staging_pose_clear_of_the_pads():
    from isaac.site import base_xy, drone_pads
    from isaac.van import staging_pose
    for sc in (make_scenario(7), make_scenario(7, site="collapse")):
        x, y, z, yaw = staging_pose(sc)
        bx, by = base_xy(sc)
        assert abs(x - bx - 10) < 1e-9 and yaw == -90.0 and -0.5 < z < 1.0
        for px, py in drone_pads(sc):
            assert math.hypot(x - px, y - py) > 6.0          # rotor disc 1.5 m + van half-width 1.25 m + margin
        if getattr(sc, "bounds", None):
            x0, y0, x1, y1 = sc.bounds
            assert x0 + 8 < x < x1 - 8 and y0 + 4 < y < y1 - 4


# ------------------------------------------------------------------------------ USD (usd-core)
@pytest.fixture(scope="module", autouse=True)
def _forget_pxr():
    """tests/test_isaac_dryrun.py asserts the dry run never imports pxr; it runs after this file, so drop the
    pxr modules we loaded once this module is done (nothing later in the suite imports pxr)."""
    import sys
    yield
    for m in [m for m in sys.modules if m == "pxr" or m.startswith("pxr.")]:
        del sys.modules[m]


@pytest.fixture(scope="module")
def stages(tmp_path_factory):
    pytest.importorskip("pxr")
    from isaac.van import build_preview_stages
    return build_preview_stages(str(tmp_path_factory.mktemp("assets")), log=lambda *_: None)


def _meshes(stage, root):
    from pxr import Usd, UsdGeom
    return [p for p in Usd.PrimRange(stage.GetPrimAtPath(root)) if p.IsA(UsdGeom.Mesh)]


def test_stage_structure_bindings_and_budgets(stages):
    from pxr import UsdGeom, UsdShade
    from isaac.van import TRI_BUDGET, count_tris
    for site, (stage, info) in stages.items():
        assert os.path.exists(info["usda"])
        for path in ("/World/Van/Body", "/World/Van/Interior/Screen0", "/World/Van/Interior/Screen2",
                     "/World/Van/Roof", "/World/Van/Outside", "/World/Van/Operator", "/World/Van/Lights/ScreenGlow",
                     "/World/Drones/d1/Model/LidarPuck", "/World/Drones/d3/Model/Hang/Pod/PodRadarFace",
                     "/World/Drones/d2/Model/Rotor3_1/Blur_hi", "/World/Pods/d1/Pod"):
            assert stage.GetPrimAtPath(path).IsValid(), (site, path)
        for root in ("/World/Van", "/World/Drones", "/World/Dummies", "/World/Pods"):
            ms = _meshes(stage, root)
            assert ms
            for m in ms:                                      # every visible mesh has a real material
                mat, _ = UsdShade.MaterialBindingAPI(m).ComputeBoundMaterial()
                assert mat and mat.GetPrim().IsValid(), str(m.GetPath())
                n = UsdGeom.Mesh(m)
                cnt = np.asarray(n.GetFaceVertexCountsAttr().Get())
                idx = np.asarray(n.GetFaceVertexIndicesAttr().Get())
                pts = np.asarray(n.GetPointsAttr().Get())
                assert np.all(cnt == 3) and idx.max() < len(pts) and len(n.GetNormalsAttr().Get()) == len(pts)
        assert info["van"]["total"] < TRI_BUDGET
        assert info["drone_tris"] < 14_000 and 3 * info["drone_tris"] < TRI_BUDGET
        assert count_tris(stage, "/World/Dummies") / max(1, len(info["dummies"]["dummies"])) < 4_000
    # screens: textured emissive UsdPreviewSurface with the dashboard screenshots
    stage = stages["grid"][0]
    mat, _ = UsdShade.MaterialBindingAPI(stage.GetPrimAtPath("/World/Van/Interior/Screen1")).ComputeBoundMaterial()
    sh = UsdShade.Shader(stage.GetPrimAtPath(str(mat.GetPath()) + "/Shader"))
    assert sh.GetInput("emissiveColor").HasConnectedSource()
    f = UsdShade.Shader(stage.GetPrimAtPath(str(mat.GetPath()) + "/TexE")).GetInput("file").Get().path
    assert f.endswith("dash_map.jpg") and os.path.exists(f)


def test_van_transform_and_no_colliders(stages):
    from pxr import Usd, UsdGeom, UsdPhysics
    from isaac.van import staging_pose
    stage, _ = stages["collapse"]
    x, y, z, yaw = staging_pose(make_scenario(7, site="collapse"))
    M = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Van")).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = M.ExtractTranslation()
    assert (t[0], t[1], t[2]) == pytest.approx((x, y, z))
    fwd = M.TransformDir((1, 0, 0))                      # van front faces south (yaw -90)
    assert fwd[1] == pytest.approx(-1.0, abs=1e-9)
    for root in ("/World/Van", "/World/Drones", "/World/Dummies", "/World/Pods"):
        assert not any(p.HasAPI(UsdPhysics.CollisionAPI) for p in Usd.PrimRange(stage.GetPrimAtPath(root)))


def test_dummies_audio_attrs_and_live_toggle(stages):
    from pxr import Usd, UsdGeom, UsdMedia
    from isaac import dummies as DM
    stage, info = stages["grid"]
    rows = {r["id"]: r for r in DM.read_dummies(stage)}
    plan = {e["id"]: e for e in info["dummies"]["dummies"]}
    assert set(rows) == set(plan)
    for k, r in rows.items():
        assert r["xyz"] == pytest.approx(plan[k]["xyz"], abs=1e-6)
        assert r["active"] == plan[k]["active"] and r["physio"] == plan[k]["physio"] and r["clip"] == plan[k]["clip"]
        au = UsdMedia.SpatialAudio(stage.GetPrimAtPath(r["prim"] + "/Audio"))
        assert au and au.GetAuralModeAttr().Get() == UsdMedia.Tokens.spatial
        assert au.GetGainAttr().Get() == (1.0 if (r["active"] and r["clip"]) else 0.0)
        if r["clip"]:
            assert os.path.exists(au.GetFilePathAttr().Get().path)
        json.loads(r["schedule"])
    assert rows["A3"]["physio"] == "distressed_tachypnea" and rows["C2_dog"]["physio"] == "none"
    # the unscripted change: switch the hidden dummy on, then drag it 3 m
    assert not rows["H1_C3_vi"]["active"]
    assert DM.set_active(stage, "H1_C3_vi", True)
    r = {q["id"]: q for q in DM.read_dummies(stage)}["H1_C3_vi"]
    assert r["active"] and UsdMedia.SpatialAudio(stage.GetPrimAtPath(r["prim"] + "/Audio")).GetGainAttr().Get() == 1.0
    op = UsdGeom.Xformable(stage.GetPrimAtPath(r["prim"])).GetOrderedXformOps()[0]
    v = op.Get()
    op.Set(type(v)(v[0] + 3.0, v[1], v[2]))
    r2 = {q["id"]: q for q in DM.read_dummies(stage)}["H1_C3_vi"]
    assert r2["xyz"][0] == pytest.approx(r["xyz"][0] + 3.0)
    assert not DM.set_active(stage, "nope", True)
    DM.set_active(stage, "H1_C3_vi", False)
    assert stages["collapse"][0].GetPrimAtPath("/World/Dummies/V1").GetAttribute("larpverse:physio").Get()
    _ = Usd


def test_drone_set_state_rotor_blur_and_hanging_pod(stages):
    from pxr import Usd, UsdGeom
    from isaac.drone_viz import BLUR_RPM, SLOW_RPM, quat_ypr, set_state
    from isaac.pods import EYE_Z
    stage = stages["grid"][0]
    from isaac.drone_viz import DroneViz
    d = DroneViz(stage, "/World/Test/d9", did="d1")
    vis = lambda p: UsdGeom.Imageable(stage.GetPrimAtPath(p)).ComputeVisibility() != UsdGeom.Tokens.invisible  # noqa: E731
    r0 = "/World/Test/d9/Model/Rotor0_0"
    assert vis(r0 + "/Blades") and not vis(r0 + "/Blur_hi")
    q = quat_ypr(30.0, 12.0, -5.0)                      # tilted into the acceleration
    assert set_state({"d9": d}, "d9", (10.0, 20.0, 30.0), q, [BLUR_RPM + 500] * 8, 0.0, dt=1 / 30)
    assert not vis(r0 + "/Blades") and vis(r0 + "/Blur_hi")
    d.set_state((10.0, 20.0, 30.0), q, [(SLOW_RPM + BLUR_RPM) / 2] * 8, 0.0)
    assert vis(r0 + "/Blades") and vis(r0 + "/Blur_lo") and not vis(r0 + "/Blur_hi")
    a0 = d.rotors[0]["angle"]
    d.set_state((10.0, 20.0, 30.0), q, [600.0] * 8, 0.0, dt=0.01)
    assert (d.rotors[0]["angle"] - a0) % 360 == pytest.approx((d.rotors[0]["sign"] * 600 * 6 * 0.01) % 360)
    cache = lambda: UsdGeom.XformCache(Usd.TimeCode.Default())  # noqa: E731
    pod = "/World/Test/d9/Model/Hang/Pod"
    d.set_state((10.0, 20.0, 30.0), q, [1700.0] * 8, 6.0)
    t = cache().GetLocalToWorldTransform(stage.GetPrimAtPath(pod)).ExtractTranslation()
    top = d.winch_world_z(30.0)
    assert t[2] == pytest.approx(top - 6.0 - EYE_Z, abs=0.05)         # hangs plumb despite the 12 deg pitch
    h = cache().GetLocalToWorldTransform(stage.GetPrimAtPath("/World/Test/d9/Model/Hang")).ExtractTranslation()
    assert math.hypot(t[0] - h[0], t[1] - h[1]) < 1e-6
    d.set_winch(6.0, sway_deg=(3.0, 0.0))
    t2 = cache().GetLocalToWorldTransform(stage.GetPrimAtPath(pod)).ExtractTranslation()
    assert 0.2 < math.hypot(t2[0] - h[0], t2[1] - h[1]) < 0.5           # ~6.3 m x sin 3 deg
    d.set_winch(40.0, ground_z=10.0)                                   # rests on the surface, no sinking
    t3 = cache().GetLocalToWorldTransform(stage.GetPrimAtPath(pod)).ExtractTranslation()
    assert t3[2] == pytest.approx(10.0, abs=1e-6)
    aw = d.anchors_world()
    assert aw["PodRadarFace"][2] == pytest.approx(10.01, abs=1e-6) and "LidarPuck" in aw


def test_podviz_drives_the_drone_winch(stages):
    from pxr import Usd, UsdGeom
    from isaac.drone_viz import DroneViz
    from isaac.pods import PodViz
    stage = stages["grid"][0]
    ds = {"e1": DroneViz(stage, "/World/Test2/e1", did="d1")}
    ds["e1"].set_pose(0.0, 0.0, 12.0, 0.0)
    pv = PodViz(stage, ["e1"], root="/World/Test2/Pods", drones=ds)
    assert not stage.GetPrimAtPath("/World/Test2/Pods/e1").IsValid()   # no duplicate pod
    st = {"pods": [{"drone_id": "e1", "xy": [0, 0], "z": 3.0, "status": "resting"}], "drones": [{"id": "e1"}]}
    pv.update(st, {"e1": (0.0, 0.0, 12.0)})
    t = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(
        stage.GetPrimAtPath("/World/Test2/e1/Model/Hang/Pod")).ExtractTranslation()
    assert t[2] == pytest.approx(3.0, abs=1e-6)
    pv.update({"pods": [], "drones": [{"id": "e1", "winch_len_m": 0.0}]}, {"e1": (0.0, 0.0, 12.0)})
    t = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(
        stage.GetPrimAtPath("/World/Test2/e1/Model/Hang/Pod")).ExtractTranslation()
    assert t[2] == pytest.approx(12.0 - 0.15 + 0.175, abs=1e-6)        # stowed between the skids


def test_textures_present():
    from isaac.van import LIVERY, SCREENS, TEX_DIR
    for f in LIVERY + SCREENS:
        assert os.path.getsize(os.path.join(TEX_DIR, f)) > 1000, f
