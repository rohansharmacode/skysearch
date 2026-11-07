"""``run_mission.py --selftest``: one command that checks every risky Isaac call in order and prints
``SELFTEST PASS|WARN|FAIL|SKIP <step>: <detail>`` per step, then a summary, writes ``<out>/selftest.json`` and
saves the frame it rendered to ``<out>/selftest_frame.png``. Steps continue after a failure wherever they can,
so the first GPU run gives a full diagnosis.

    cd /workspace/isaaclab && ./isaaclab.sh -p /workspace/skysearch/larpverse/isaac/run_mission.py \\
        --selftest --headless --enable_cameras --out /workspace/selftest          # Isaac Lab route (Brev)
    ~/isaacsim/python.sh isaac/run_mission.py --selftest --headless --out /tmp/selftest    # Kit route (4.5/5.x)
    python isaac/run_mission.py --selftest --dry-run --out /tmp/selftest                    # plumbing, no GPU

Order: app boot -> extensions -> mission import -> stage build -> look -> scene objects -> physics parse ->
raycast -> LiDAR sweep -> drone pose -> camera switch -> debug draw -> USD overlay -> render frame -> dust.
"""
from __future__ import annotations

import json
import math
import os
import time
import traceback

HINTS = {
    "app_boot": "Isaac Lab route: run through ./isaaclab.sh -p (or force --launcher kit with ~/isaacsim/python.sh). "
                "First boot compiles shaders for several minutes; set OMNI_KIT_ACCEPT_EULA=YES.",
    "physics_parse": "No UsdPhysics.Scene, or scene queries off. Lab route: SimulationCfg(enable_scene_query_support=True) "
                     "is set in compat.LabRuntime; Kit route: /World/physicsScene with PhysxSceneAPI.enableSceneQuerySupport.",
    "raycast": "Rays only hit while simulating (after sim.reset()/timeline play) and with scene queries on. "
               "The run falls back to the heightmap LiDAR, so the demo still works, but fix this first.",
    "camera_switch": "Use --camera free and fly the viewport, or record from /World/Cams/rec (--record).",
    "render_frame": "Black frame: (1) under --livestream the viewport may only draw with a /viewer client connected; "
                    "(2) shaders still compiling: rerun; (3) try --look plain and --sun 3000; (4) Camera sensors need "
                    "--enable_cameras on the Isaac Lab route.",
    "debug_draw": "Overlay falls back to USD prims (--overlay usd), so this is cosmetic.",
}


class Report:
    def __init__(self, say):
        self.say, self.rows = say, []

    def step(self, name, fn, skip=None):
        if skip:
            return self._row(name, "SKIP", skip)
        t0 = time.time()
        try:
            res = fn()
            status, detail = res if isinstance(res, tuple) and res and res[0] in ("PASS", "WARN", "FAIL") else ("PASS", res)
        except Exception as e:   # noqa: BLE001
            status = "FAIL"
            tb = traceback.format_exc().strip().splitlines()
            detail = f"{type(e).__name__}: {e} | {' / '.join(tb[-3:-1])}"
        if status in ("FAIL", "WARN") and name in HINTS:
            detail = f"{detail}  HINT: {HINTS[name]}"
        return self._row(name, status, detail, time.time() - t0)

    def _row(self, name, status, detail, dt=0.0):
        detail = "" if detail is None else (json.dumps(detail, default=str) if isinstance(detail, (dict, list)) else str(detail))
        self.rows.append({"step": name, "status": status, "detail": detail, "seconds": round(dt, 2)})
        print(f"SELFTEST {status:4s} {name}: {detail}", flush=True)
        return status

    def counts(self):
        c = {}
        for r in self.rows:
            c[r["status"]] = c.get(r["status"], 0) + 1
        return c


def run(a, say=print):
    """Returns {'rows': [...], 'counts': {...}}; exits the app at the end on a real Isaac box."""
    out = a.out or "."
    os.makedirs(out, exist_ok=True)
    R = Report(say)
    ctx = {}
    dry = a.dry_run
    no_isaac = "dry-run: needs Isaac" if dry else None

    # 1. app boot ------------------------------------------------------------------------------
    def boot():
        from isaac import compat
        rt = compat.make_runtime(a)
        ctx["app"] = rt.boot()
        ctx["rt"] = rt
        v = compat.versions()
        live = getattr(a, "livestream", 0)
        return f"launcher={rt.mode} versions={v} headless={getattr(a, 'headless', None)} livestream={live} {' '.join(rt.log)}"
    if R.step("app_boot", boot, skip=no_isaac) == "FAIL":
        return _finish(a, R, ctx, out)

    # 2. extensions ----------------------------------------------------------------------------
    def exts():
        from isaac import compat
        names = list(compat.LAB_EXTS) + ["omni.physx", "omni.kit.viewport.utility", "omni.replicator.core"]
        st = {n: compat.enable_ext(n) for n in names}
        missing = [n for n, ok in st.items() if not ok]
        dd_ok = st.get("isaacsim.util.debug_draw") or compat.enable_ext("omni.isaac.debug_draw")
        return ("PASS" if dd_ok and st["omni.physx"] else "WARN"), {"enabled": [n for n, ok in st.items() if ok], "missing": missing}
    R.step("extensions", exts, skip=no_isaac)

    # 3. mission import (pure python: scipy, GridMission) -------------------------------------------
    def mission():
        from sim.scenario import make_scenario
        ctx["sc"] = sc = make_scenario(a.seed, a.start_pile)
        try:
            import scipy  # noqa: F401
            sp = "scipy ok"
        except ImportError:
            sp = "scipy MISSING (pip install scipy into Isaac's python: LiDAR->map and --radar dsp need it)"
        from sim.grid_mission import GridMission
        m = GridMission(seed=a.seed, start_pile=a.start_pile, radar=a.radar,
                        out_dir=os.path.join(out, "selftest_mission"), verbose=False)
        m.step()
        return ("PASS" if sp == "scipy ok" else "WARN"), f"{len(sc.piles)} piles, GridMission stepped to t={m.t}; {sp}"
    R.step("mission_import", mission)

    # 4. stage build ---------------------------------------------------------------------------
    def stage_build():
        from pxr import UsdGeom, UsdPhysics

        from isaac import scene
        rt, sc = ctx["rt"], ctx["sc"]
        t0 = time.time()
        stage = ctx["stage"] = rt.stage()
        n = scene.build(stage, sc, show_truth=False, physics_scene=rt.own_physics_scene, sun=a.sun, sky=a.sky,
                        materials="none" if a.look == "plain" else "preview")
        cols = sum(1 for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI))
        return {"pieces": n, "colliders": cols, "upAxis": UsdGeom.GetStageUpAxis(stage),
                "metersPerUnit": UsdGeom.GetStageMetersPerUnit(stage), "materials_bound": scene.build.materials.bound,
                "seconds": round(time.time() - t0, 1)}
    R.step("stage_build", stage_build, skip=no_isaac or ("no scenario" if "sc" not in ctx else None))
    have_stage = "stage" in ctx

    # 5. look (optional) -------------------------------------------------------------------------
    def look_step():
        from isaac import look
        rep = look.apply(ctx["stage"], ctx["sc"], ctx["rt"].mode, hdri=a.hdri, log=say)
        return ("PASS" if rep["mdl"] and rep["hdri"] else "WARN"), rep
    R.step("look_pretty", look_step, skip=no_isaac or ("--look is not pretty" if a.look != "pretty" else None)
           or (None if have_stage else "no stage"))

    # 6. scene objects: drones, overlay backend, recording camera, director camera (BEFORE physics start) ---
    def objects():
        from isaac import compat
        from isaac.camera import DirectorCamera
        from isaac.drone_viz import DroneViz
        stage, rt = ctx["stage"], ctx["rt"]
        ctx["drones"] = {i: DroneViz(stage, f"/World/Drones/{i}", did=i, scale=a.drone_scale, asset=a.drone_asset)
                         for i in ("d1", "d2", "d3")}
        grab = ctx["grab"] = compat.FrameGrabber(rt, *_res(a.rec_res))
        mirrors = [compat.PERSP] + ([grab.mirror] if grab.mirror else [])
        ctx["cam"] = DirectorCamera(stage, mode="director", mirrors=mirrors)
        return f"3 drones, grabber={grab.kind or 'late'}{(' (' + grab.err + ')') if grab.err else ''}, mirrors={mirrors}"
    R.step("scene_objects", objects, skip=no_isaac or (None if have_stage else "no stage"))

    # 7. physics parse -----------------------------------------------------------------------------
    def physics():
        from pxr import UsdPhysics
        rt, stage = ctx["rt"], ctx["stage"]
        rt.start_physics(30)
        scenes = []
        for p in stage.Traverse():
            if p.IsA(UsdPhysics.Scene):
                at = p.GetAttribute("physxScene:enableSceneQuerySupport")
                scenes.append(f"{p.GetPath()} sceneQuery={at.Get() if at and at.IsValid() else 'unset'}")
        ctx["physics"] = True
        return ("PASS" if scenes else "WARN"), {"physics_scenes": scenes or "none found on the composed stage",
                                                "dt": rt.physics_dt}
    R.step("physics_parse", physics, skip=no_isaac or (None if have_stage else "no stage"))

    # 8. raycast ------------------------------------------------------------------------------------
    def raycast():
        from isaac import compat
        sc = ctx["sc"]
        cast = ctx["cast"] = compat.physx_raycaster()
        p = next(q for q in sc.piles if q.name == "B2")
        h1 = cast.raw((p.x, p.y, 30.0), (0, 0, -1), 60.0)
        tx, ty = 11.0, 11.0
        h2 = cast.raw((tx, ty, 30.0), (0, 0, -1), 60.0)
        g = float(sc.ground_height(tx, ty))

        def short(h):
            if not h or not h.get("hit"):
                return {"hit": False}
            pos = h.get("position")
            try:
                pos = [round(float(pos[i]), 2) for i in range(3)]
            except TypeError:
                pos = [round(pos.x, 2), round(pos.y, 2), round(pos.z, 2)]
            return {"hit": True, "z": pos[2], "collider": str(h.get("collision", "?")), "dist": round(float(h.get("distance", -1)), 2)}
        s1, s2 = short(h1), short(h2)
        det = {"pile_B2": s1, "terrain_11_11": s2, "terrain_expected_z": round(g, 2)}
        if not s1["hit"] and not s2["hit"]:
            return "FAIL", det
        if s2["hit"] and abs(s2["z"] - g) > 0.35:
            return "WARN", det
        return "PASS", det
    R.step("raycast", raycast, skip=no_isaac or (None if ctx.get("physics") else "physics not started"))

    # 9. LiDAR sweep --------------------------------------------------------------------------------
    def sweep():
        from isaac.lidar import HeightmapLidar, PhysxLidar
        sc = ctx["sc"]
        p = next(q for q in sc.piles if q.name == "A2")
        if dry:
            lid = HeightmapLidar(sc.heightmap(0.5), a.lidar_range, a.rays)
        else:
            lid = PhysxLidar(ctx["cast"], a.lidar_range, a.rays)
        pts = lid.sweep((p.x, p.y - 12.0, 8.0), 90.0, 0)
        if len(pts) == 0:
            return "FAIL", f"0 of {a.rays} rays hit"
        return f"{len(pts)}/{a.rays} rays hit, z range {pts[:, 2].min():.2f}..{pts[:, 2].max():.2f}"
    R.step("lidar_sweep", sweep, skip=None if dry or "cast" in ctx else "no raycaster")

    # 10. drone pose --------------------------------------------------------------------------------
    def drone_pose():
        from isaac import compat
        d = ctx["drones"]["d1"]
        d.set_pose(1.0, -40.0, 6.0, 30.0, (0.0, 8.0))
        d.spin(0.05)
        ctx["rt"].step()
        t = compat.world_translation(d.path)
        err = math.dist(t, (1.0, -40.0, 6.0))
        return ("PASS" if err < 1e-3 else "FAIL"), f"set (1, -40, 6) read back {tuple(round(v, 3) for v in t)}"
    R.step("drone_pose", drone_pose, skip=no_isaac or (None if "drones" in ctx else "no drones"))

    # 11. camera switch -----------------------------------------------------------------------------
    def camera():
        from isaac import compat
        cam = ctx["cam"]
        p = next(q for q in ctx["sc"].piles if q.name == "B2")
        cam.target[:] = (p.x, p.y, 2.0)
        cam.eye[:] = (p.x - 18.0, p.y - 26.0, 16.0)
        cam._apply()
        ctx["rt"].step()
        det = {"viewport_switched": cam.is_viewport, "viewport_camera": compat.viewport_camera_path()}
        ok_any = cam.is_viewport
        for w in cam.mirrors:
            t = compat.world_translation(w.path) if w.ok else None
            good = t is not None and math.dist(t, cam.eye) < 0.05
            det[w.path] = f"{w.route} ok" if good else (w.err or f"pose not applied (read {t})")
            ok_any = ok_any or good
        if cam.is_viewport:
            return "PASS", det
        return ("WARN" if ok_any else "FAIL"), det
    R.step("camera_switch", camera, skip=no_isaac or (None if "cam" in ctx else "no camera"))

    # 12. debug draw --------------------------------------------------------------------------------
    def ddraw():
        from isaac import compat
        dd = compat.debug_draw()
        if dd is None:
            return "WARN", "debug_draw extension unavailable"
        p = next(q for q in ctx["sc"].piles if q.name == "B2")
        dd.draw_lines([(p.x - 6, p.y, 3.0)], [(p.x + 6, p.y, 3.0)], [(0.2, 0.95, 0.35, 1.0)], [4.0])
        dd.draw_points([(p.x, p.y, 4.0)], [(1.0, 0.9, 0.1, 1.0)], [14.0])
        ctx["rt"].step()
        return f"{compat.DEBUG_DRAW_EXT}: draw_lines/draw_points ok (left on for the frame)"
    R.step("debug_draw", ddraw, skip=no_isaac or (None if have_stage else "no stage"))

    # 13. USD overlay --------------------------------------------------------------------------------
    def usd_overlay():
        import numpy as np

        from isaac.overlay import Overlay, make_backend
        sc = ctx["sc"]
        pile_xy = {p.name: (p.x, p.y) for p in sc.piles}
        hm = sc.heightmap(0.5)
        if dry:
            ov = Overlay(make_backend("null"), hm.height_at, pile_xy)
        else:
            ov = Overlay(make_backend("usd", ctx["stage"]), hm.height_at, pile_xy)
        p = sc.piles[4]
        state = {"radar_candidates": [{"id": "SELFTEST", "xy": [p.x, p.y], "z_est": -1.0, "cov": [[1, 0], [0, 1]]}],
                 "scans": [{"pile_id": p.name, "sensor_pose": [p.x - 5, p.y, 1, 0], "range_candidates": [5.0], "confidence": [0.9]}],
                 "drones": [], "piles": [{"id": k, "xy": list(v), "status": "active" if k == p.name else "pending"} for k, v in pile_xy.items()]}
        ov.update(state)
        ov.add_points(np.array([[p.x, p.y, 3.0], [p.x + 1, p.y, 3.2]], np.float32))
        if not dry:
            ov.b.flush_points()
            ok = ctx["stage"].GetPrimAtPath("/World/Overlay/Lines").IsValid()
            return ("PASS" if ok else "FAIL"), f"{ov.n_lines} line segments, {ov.n_pings} ping prims under /World/Overlay"
        return f"null backend: {ov.n_lines} segments, {ov.n_pings} pings"
    R.step("overlay_usd", usd_overlay, skip=None if "sc" in ctx and (dry or have_stage) else "no scenario/stage")

    # 14. render frame ------------------------------------------------------------------------------
    def frame():
        from isaac import compat
        path = os.path.join(out, "selftest_frame.png")
        if dry:
            import numpy as np
            img = np.zeros((36, 64, 3), np.uint8)
            img[..., 0] = np.linspace(0, 255, 64, dtype=np.uint8)[None, :]
            compat.write_png(path, img)
            return f"dry-run: PNG writer ok -> {path}"
        rt, grab = ctx["rt"], ctx["grab"]
        grab.attach_late()
        for _ in range(60):
            rt.step()
        ok, mean = grab.save(path)
        tries = 1
        while ok and mean is not None and mean < 3.0 and tries < 3:      # shaders may still be compiling
            for _ in range(120):
                rt.step()
            ok, mean = grab.save(path)
            tries += 1
        vp_path = os.path.join(out, "selftest_viewport.png")
        vp = compat.capture_viewport(vp_path)
        for _ in range(10):
            rt.step()
        det = {"route": grab.kind, "saved": path if ok else None, "mean_rgb": None if mean is None else round(mean, 1),
               "tries": tries, "viewport_png_bytes": os.path.getsize(vp_path) if vp and os.path.exists(vp_path) else None}
        if grab.err:
            det["note"] = grab.err
        if not ok:
            return ("WARN" if det["viewport_png_bytes"] else "FAIL"), det
        if mean is not None and mean < 3.0:
            return "FAIL", det | {"problem": "frame is black"}
        return "PASS", det
    R.step("render_frame", frame, skip=None if dry or "grab" in ctx else "no grabber")

    # 15. Flow dust (optional) -----------------------------------------------------------------------
    def dust():
        from isaac import look
        fx = look.FlowDust(ctx["stage"], ["d1"], log=say)
        if not fx.ok:
            return "WARN", "Flow unavailable"
        fx.update({"d1": (0.0, -40.0, 1.5)}, ctx["sc"].ground_height)
        ctx["rt"].step()
        return "omni.flowusd preset referenced, emitter toggled"
    R.step("flow_dust", dust, skip=no_isaac or ("--look is not pretty" if a.look != "pretty" else None)
           or (None if ctx.get("physics") else "no stage"))

    # 16. RTX LiDAR (optional) ------------------------------------------------------------------------
    def rtx():
        from isaac.lidar import RtxLidar
        r = RtxLidar({"d1": ctx["drones"]["d1"].path})
        return ("PASS" if r.ok else "WARN"), "RTX LiDAR created" if r.ok else "unavailable; PhysX raycast LiDAR is used"
    R.step("rtx_lidar", rtx, skip=no_isaac or (None if a.rtx_lidar and "drones" in ctx else "not requested (--rtx-lidar)"))

    return _finish(a, R, ctx, out)


def _res(s):
    w, h = (int(v) for v in str(s).lower().split("x"))
    return w, h


def _finish(a, R, ctx, out):
    c = R.counts()
    res = {"rows": R.rows, "counts": c, "launcher": getattr(a, "launcher", None)}
    try:
        with open(os.path.join(out, "selftest.json"), "w") as f:
            json.dump(res, f, indent=1)
    except OSError:
        pass
    print(f"SELFTEST SUMMARY {c}  (details: {os.path.join(out, 'selftest.json')}; frame: "
          f"{os.path.join(out, 'selftest_frame.png')})", flush=True)
    if c.get("FAIL"):
        print("SELFTEST next: send the whole log + selftest.json + selftest_frame.png to the team.", flush=True)
    else:
        print("SELFTEST next: run the mission, e.g. --speed 4 --show-truth --radar dsp (add --livestream 2 on Brev).", flush=True)
    rt = ctx.get("rt")
    if rt is not None and rt.app is not None:
        rt.close()
    return res
