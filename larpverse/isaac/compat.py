"""Version shims so the adapter runs on Isaac Sim 6.0.x + Isaac Lab 3.0 (Brev launchable) and on Isaac Sim
4.5 / 5.x standalone. Everything version-specific lives here and degrades gracefully.

Two launch routes (``--launcher auto|lab|kit``), see docs/ISAAC_VERIFIED_APIS.md for the evidence:

* ``LabRuntime``  ``./isaaclab.sh -p isaac/run_mission.py ...``: Isaac Lab ``AppLauncher`` boots the app,
  ``isaaclab.sim.SimulationContext(SimulationCfg(enable_scene_query_support=True))`` owns physics, and the
  loop calls ``sim.step()``. This is the route the team verified on the Brev L40S (Isaac Sim 6.0.1 /
  Isaac Lab 3.0). On 6.x the 5.x core API (``isaacsim.core.api`` World, SimulationManager) crashes under
  Isaac Lab, so we do not touch it.
* ``KitRuntime``  ``~/isaacsim/python.sh isaac/run_mission.py ...``: plain ``SimulationApp``,
  ``omni.timeline`` play and ``app.update()``. Fallback for 4.5 / 5.x installs without Isaac Lab.

Only pxr, omni.usd, omni.physx scene queries and carb settings are used outside this module.
"""
from __future__ import annotations

import importlib
import importlib.util
import math
import os
import struct
import zlib

# Experimental extensions the team enables right after AppLauncher on 6.0.1 (ISAAC6_API.md:41-48).
LAB_EXTS = ("isaacsim.core.experimental.prims", "isaacsim.core.experimental.objects",
            "isaacsim.core.experimental.materials", "isaacsim.util.debug_draw")
PERSP = "/OmniverseKit_Persp"


# ---------------------------------------------------------------------------- launcher selection
def lab_available() -> bool:
    try:
        return importlib.util.find_spec("isaaclab") is not None
    except (ImportError, ValueError):
        return False


def resolve_launcher(choice: str) -> str:
    """auto -> lab when Isaac Lab is importable (``./isaaclab.sh -p``), else kit (SimulationApp)."""
    if choice == "auto":
        return "lab" if lab_available() else "kit"
    return choice


def add_kit_args(ap):
    """Kit-route equivalents of AppLauncher's flags, so one command line works on both routes."""
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--livestream", type=int, nargs="?", const=2, default=0,
                    help="1 = native, 2 = WebRTC (as Isaac Lab's AppLauncher); bare flag = 2")
    ap.add_argument("--enable_cameras", action="store_true", help="accepted for parity with AppLauncher")
    ap.add_argument("--device", default="cuda:0")


def add_lab_args(ap):
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(ap)


# ---------------------------------------------------------------------------- runtimes
class KitRuntime:
    """SimulationApp + omni.timeline + app.update() (Isaac Sim 4.5 / 5.x standalone)."""
    mode = "kit"
    own_physics_scene = True            # scene.build defines /World/physicsScene

    def __init__(self, a):
        self.a, self.app, self.sim = a, None, None
        self.log = []

    def boot(self):
        live = int(getattr(self.a, "livestream", 0) or 0)
        self.app = make_app(bool(self.a.headless) or live > 0)
        if live > 0:
            names = ("omni.kit.livestream.webrtc",) if live == 2 else ("omni.kit.livestream.native",)
            ok = any(enable_ext(e) for e in names + ("omni.kit.livestream.webrtc", "omni.kit.livestream.native"))
            self.log.append(f"livestream extension {'enabled' if ok else 'NOT available'}")
        return self.app

    def stage(self):
        st = new_stage()
        for _ in range(3):
            self.app.update()
        return st

    def start_physics(self, warmup=30):
        force_load_physics()
        timeline_play()
        for _ in range(warmup):
            self.app.update()

    def step(self):
        self.app.update()

    @property
    def physics_dt(self):
        return 1.0 / 60.0

    def is_running(self):
        return self.app.is_running()

    def close(self):
        self.app.close()


class LabRuntime(KitRuntime):
    """Isaac Lab AppLauncher + SimulationContext (verified on Isaac Sim 6.0.1 / Isaac Lab 3.0, Brev L40S)."""
    mode = "lab"
    own_physics_scene = False           # SimulationContext defines its own physics scene

    def boot(self):
        from isaaclab.app import AppLauncher
        self.a.enable_cameras = True    # Camera sensors (recording / selftest frame) need it, also under livestream
        self.app = AppLauncher(self.a).app
        for ext in LAB_EXTS:
            enable_ext(ext)
        return self.app

    def stage(self):
        import carb
        import isaaclab.sim as sim_utils
        import omni.usd
        fabric = bool(getattr(self.a, "fabric", False))
        if not fabric:   # probe_isaaclab3.py defaults: USD is the source of truth, pxr pose edits render
            carb.settings.get_settings().set_bool("/rtx/hydra/readTransformsFromFabricInRenderDelegate", False)
        self.sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(
            dt=1.0 / 30.0, device=getattr(self.a, "device", "cuda:0"), use_fabric=fabric,
            enable_scene_query_support=True))           # default False => raycasts silently miss
        return omni.usd.get_context().get_stage()

    def start_physics(self, warmup=30):
        self.sim.reset()                 # starts physics; spawn everything (incl. Camera sensors) before this
        for _ in range(warmup):
            self.sim.step()

    def step(self):
        self.sim.step()

    @property
    def physics_dt(self):
        try:
            return float(self.sim.get_physics_dt())
        except Exception:
            return 1.0 / 30.0


def make_runtime(a):
    return LabRuntime(a) if a.launcher == "lab" else KitRuntime(a)


def make_app(headless: bool, width: int = 1600, height: int = 900):
    """Create SimulationApp. Must run before importing any omni/pxr module."""
    try:
        from isaacsim import SimulationApp            # 4.5+
    except ImportError:
        from omni.isaac.kit import SimulationApp      # 4.0-4.2
    return SimulationApp({"headless": headless, "width": width, "height": height})


def versions():
    """Best-effort version strings (Kit build, Isaac Sim, Isaac Lab)."""
    out = {}
    try:
        import omni.kit.app
        out["kit"] = omni.kit.app.get_app().get_build_version()
    except Exception:
        pass
    for mod, attr in (("isaacsim.core.version", "get_version"), ("omni.isaac.version", "get_version")):
        try:
            v = getattr(importlib.import_module(mod), attr)()
            out["isaacsim"] = ".".join(str(x) for x in v[:3]) if isinstance(v, (tuple, list)) else str(v)
            break
        except Exception:
            continue
    try:
        import isaaclab
        out["isaaclab"] = getattr(isaaclab, "__version__", "?")
    except Exception:
        pass
    return out


# ---------------------------------------------------------------------------- extensions / stage / physics
def enable_ext(name: str) -> bool:
    try:
        import omni.kit.app
        mgr = omni.kit.app.get_app().get_extension_manager()
        mgr.set_extension_enabled_immediate(name, True)
        return bool(mgr.is_extension_enabled(name))
    except Exception:
        return False


DEBUG_DRAW_EXT = None


def debug_draw():
    """Debug-draw interface (points/lines overlay) or None if the extension is unavailable.
    Verified on 6.0.1: ``from isaacsim.util.debug_draw import _debug_draw`` after enabling the extension."""
    global DEBUG_DRAW_EXT
    for ext in ("isaacsim.util.debug_draw", "omni.isaac.debug_draw"):
        if not enable_ext(ext):
            continue
        try:
            dd = importlib.import_module(ext + "._debug_draw").acquire_debug_draw_interface()
            DEBUG_DRAW_EXT = ext
            return dd
        except Exception:
            continue
    return None


def new_stage():
    """Fresh stage in the running Kit app (Kit route only; under Isaac Lab use the launcher's stage)."""
    import omni.usd
    ctx = omni.usd.get_context()
    ctx.new_stage()
    return ctx.get_stage()


def timeline_play():
    import omni.timeline
    omni.timeline.get_timeline_interface().play()


def force_load_physics():
    """Make PhysX parse the stage now so scene queries see the colliders (no-op if unavailable)."""
    try:
        from omni.physx import get_physx_interface
        get_physx_interface().force_load_physics_from_usd()
        return True
    except Exception:
        return False


def enable_scene_queries(prim) -> bool:
    """PhysxSceneAPI.enableSceneQuerySupport = True on a UsdPhysics.Scene prim (Isaac Lab defaults it off)."""
    try:
        from pxr import PhysxSchema
        PhysxSchema.PhysxSceneAPI.Apply(prim).CreateEnableSceneQuerySupportAttr(True)
        return True
    except Exception:
        try:
            from pxr import Sdf
            prim.CreateAttribute("physxScene:enableSceneQuerySupport", Sdf.ValueTypeNames.Bool).Set(True)
            return True
        except Exception:
            return False


def physx_raycaster():
    """Return f(origin3, dir3, max_dist) -> (x, y, z) | None using PhysX scene queries.

    Verified on 6.0.1 with plain tuples: ``raycast_closest((0,0,2), (0,1,0), 50.0)`` -> dict with ``hit``,
    ``distance``, ``collision`` (prim path), ``position``, ``normal``. Hits only while simulating and with
    scene-query support on; otherwise ``{'hit': False}`` silently. carb.Float3 is the 4.x/5.x fallback."""
    from omni.physx import get_physx_scene_query_interface
    sq = get_physx_scene_query_interface()
    use_carb = [False]

    def raw(o, d, dist):
        n = (float(d[0]) ** 2 + float(d[1]) ** 2 + float(d[2]) ** 2) ** 0.5 or 1.0   # unit direction
        oo = (float(o[0]), float(o[1]), float(o[2]))
        dd = (float(d[0]) / n, float(d[1]) / n, float(d[2]) / n)
        if not use_carb[0]:
            try:
                return sq.raycast_closest(oo, dd, float(dist))
            except TypeError:
                use_carb[0] = True
        import carb
        return sq.raycast_closest(carb.Float3(*oo), carb.Float3(*dd), float(dist))

    def cast(o, d, dist):
        hit = raw(o, d, dist)
        if not hit or not hit.get("hit"):
            return None
        p = hit["position"]
        try:
            return (float(p[0]), float(p[1]), float(p[2]))
        except TypeError:
            return (p.x, p.y, p.z)
    cast.raw = raw
    return cast


# ---------------------------------------------------------------------------- cameras / poses
def get_viewport():
    try:
        from omni.kit.viewport.utility import get_active_viewport
        return get_active_viewport()
    except Exception:
        return None


def set_viewport_camera(path: str) -> bool:
    """Point the active viewport at ``path`` and read it back. UNVERIFIED on 6.x (the team moved
    /OmniverseKit_Persp instead); headless Isaac Lab runs usually have no viewport at all."""
    vp = get_viewport()
    if vp is None:
        return False
    try:
        vp.camera_path = path
        return str(vp.camera_path) == path
    except Exception:
        return False


def viewport_camera_path():
    vp = get_viewport()
    try:
        return str(vp.camera_path) if vp is not None else None
    except Exception:
        return None


def mat_to_quat_wxyz(R):
    """3x3 rotation (column-vector convention: world = R @ local) -> (w, x, y, z)."""
    m = [[float(R[i][j]) for j in range(3)] for i in range(3)]
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        w, x, y, z = (m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        w, x, y, z = (m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        w, x, y, z = (m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    return (w / n, x / n, y / n, z / n)


class PoseWriter:
    """Set a prim's world pose every frame. Verified route on 6.0.1: experimental ``XformPrim.set_world_poses``
    (quat wxyz; ``reset_xform_op_properties=True`` the first time for Kit-created prims like
    /OmniverseKit_Persp). Fallback (4.5 / no experimental API): pxr transform op, in the session layer for
    Kit's own cameras. Disables itself after the first failure and remembers why."""

    def __init__(self, path):
        self.path, self.xp, self.op, self.ok, self.err, self.route = path, None, None, True, None, None

    def set(self, pos, quat_wxyz, M=None):
        if not self.ok:
            return False
        if self.route in (None, "xformprim"):
            try:
                if self.xp is None:
                    import isaacsim.core.experimental.prims as P
                    self.xp = P.XformPrim(self.path, reset_xform_op_properties=True)
                self.xp.set_world_poses(positions=[list(map(float, pos))], orientations=[list(map(float, quat_wxyz))])
                self.route = "xformprim"
                return True
            except Exception as e:   # noqa: BLE001 - no experimental API (4.5/5.x) or it refused: try pxr
                if self.route == "xformprim":
                    self.ok, self.err = False, f"XformPrim: {type(e).__name__}: {e}"
                    return False
                self.route, self.err = "pxr", f"XformPrim unavailable ({type(e).__name__}); pxr fallback"
        try:
            return self._pxr(pos, quat_wxyz, M)
        except Exception as e:   # noqa: BLE001
            self.ok, self.err = False, f"{type(e).__name__}: {e}"
            return False

    def _pxr(self, pos, q, M):
        import omni.usd
        from pxr import Gf, Usd, UsdGeom
        st = omni.usd.get_context().get_stage()
        prim = st.GetPrimAtPath(self.path)
        if not prim.IsValid():
            raise RuntimeError(f"{self.path} does not exist")
        if self.op is None:
            layer = st.GetSessionLayer() if self.path.startswith("/OmniverseKit") else st.GetEditTarget().GetLayer()
            with Usd.EditContext(st, layer):
                xf = UsdGeom.Xformable(prim)
                xf.ClearXformOpOrder()
                self.op = xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble)
        if M is None:
            rot = Gf.Matrix4d().SetRotate(Gf.Quatd(q[0], Gf.Vec3d(q[1], q[2], q[3])))
            M = [[rot[i][j] for j in range(4)] for i in range(4)]
            M[3] = [float(pos[0]), float(pos[1]), float(pos[2]), 1.0]
        self.op.Set(Gf.Matrix4d(*[float(v) for row in M for v in row]))
        return True


def world_translation(path):
    import omni.usd
    from pxr import Usd, UsdGeom
    prim = omni.usd.get_context().get_stage().GetPrimAtPath(path)
    if not prim.IsValid():
        return None
    t = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
    return (t[0], t[1], t[2])


# ---------------------------------------------------------------------------- frame capture
class FrameGrabber:
    """One RGB frame from the director's point of view, saved as PNG.

    * ``lab``        Isaac Lab ``Camera`` sensor at /World/Cams/rec (verified route: earthquake_scene.py
                     --record). Must be created BEFORE ``sim.reset()``; posed every frame like the director.
    * ``replicator`` render product + "rgb" annotator on the director camera (Kit route; UNVERIFIED).
    * ``viewport``   ``capture_viewport_to_file`` (async, viewport only; black under livestream with no client).
    """
    REC = "/World/Cams/rec"

    def __init__(self, runtime, width=1280, height=720, cam_path="/World/DirectorCam"):
        self.rt, self.w, self.h, self.cam_path = runtime, int(width), int(height), cam_path
        self.kind, self.cam, self.ann, self.err = None, None, None, None
        self.mirror = None
        if runtime.mode == "lab":
            try:
                import isaaclab.sim as sim_utils
                from isaaclab.sensors.camera import Camera, CameraCfg
                self.cam = Camera(CameraCfg(prim_path=self.REC, update_period=0, height=self.h, width=self.w,
                                            data_types=["rgb"],
                                            spawn=sim_utils.PinholeCameraCfg(focal_length=18.0, horizontal_aperture=20.955,
                                                                             clipping_range=(0.1, 1.0e4))))
                self.kind, self.mirror = "lab", self.REC
            except Exception as e:   # noqa: BLE001
                self.err = f"Isaac Lab Camera failed: {type(e).__name__}: {e}"

    def attach_late(self):
        """Kit route: build the replicator render product once the director camera exists."""
        if self.kind is not None:
            return
        try:
            import omni.replicator.core as rep
            rp = rep.create.render_product(self.cam_path, (self.w, self.h))
            ann = rep.AnnotatorRegistry.get_annotator("rgb")
            ann.attach([rp])
            self.kind, self.ann = "replicator", ann
        except Exception as e:   # noqa: BLE001
            self.err = (self.err + "; " if self.err else "") + f"replicator failed: {type(e).__name__}: {e}"
            self.kind = "viewport"

    def grab(self):
        """HxWx3 uint8 numpy array or None."""
        import numpy as np
        img = None
        if self.kind == "lab":
            self.cam.update(dt=self.rt.physics_dt)
            img = self.cam.data.output["rgb"][0].cpu().numpy()[..., :3]
        elif self.kind == "replicator":
            d = self.ann.get_data()
            d = d.get("data") if isinstance(d, dict) else d
            img = np.asarray(d)
            if img.ndim != 3 or img.size == 0:
                return None
            img = img[..., :3]
        if img is None:
            return None
        img = np.asarray(img)
        if img.dtype != np.uint8:
            img = np.clip(img * (255.0 if float(img.max(initial=0)) <= 1.0 else 1.0), 0, 255).astype(np.uint8)
        return img

    def save(self, path):
        """Save one frame. Returns (ok, mean brightness or None)."""
        if self.kind == "viewport":
            ok = capture_viewport(path)
            return ok, None
        img = self.grab()
        if img is None:
            return False, None
        write_png(path, img)
        return True, float(img.mean())


def capture_viewport(path) -> bool:
    try:
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        vp = get_active_viewport()
        if vp is None:
            return False
        capture_viewport_to_file(vp, path)
        return True
    except Exception:
        return False


def write_png(path, rgb):
    """Minimal RGB8 PNG writer (no matplotlib / PIL needed inside Isaac's python)."""
    import numpy as np
    a = np.ascontiguousarray(np.asarray(rgb, np.uint8)[..., :3])
    h, w = a.shape[:2]
    raw = b"".join(b"\x00" + a[r].tobytes() for r in range(h))

    def chunk(t, data):
        return struct.pack(">I", len(data)) + t + data + struct.pack(">I", zlib.crc32(t + data) & 0xFFFFFFFF)
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


# ---------------------------------------------------------------------------- rendering / assets
def set_render_mode(mode="realtime", spp=16):
    """RTX real-time or path tracing (+ OptiX denoiser). Settings verified on 6.0.1 (earthquake_scene.py:219-226)."""
    if mode != "pathtracing":
        return False
    import carb
    cs = carb.settings.get_settings()
    cs.set_string("/rtx/rendermode", "PathTracing")
    cs.set_int("/rtx/pathtracing/spp", int(spp))
    cs.set_int("/rtx/pathtracing/totalSpp", int(spp))
    cs.set_bool("/rtx/pathtracing/optixDenoiser/enabled", True)
    return True


def assets_root():
    """Isaac assets root URL WITHOUT the trailing /Isaac (e.g. .../Assets/Isaac/6.0), or None.
    Isaac Lab exposes ISAAC_NUCLEUS_DIR = <root>/Isaac and NVIDIA_NUCLEUS_DIR = <root>/NVIDIA (assets_found.json)."""
    try:
        from isaaclab.utils import assets as A
        d = getattr(A, "ISAAC_NUCLEUS_DIR", None)
        if d and d.rstrip("/").endswith("/Isaac"):
            return d.rstrip("/")[: -len("/Isaac")]
    except Exception:
        pass
    for mod in ("isaacsim.storage.native", "omni.isaac.nucleus", "omni.isaac.core.utils.nucleus"):
        try:
            root = importlib.import_module(mod).get_assets_root_path()
            if root:
                return root.rstrip("/")
        except Exception:
            continue
    return None


def url_exists(url: str) -> bool:
    try:
        import omni.client
        result, _ = omni.client.stat(url)
        return result == omni.client.Result.OK
    except Exception:
        return False


def meters_per_unit(url: str) -> float:
    """metersPerUnit of an asset layer. A plain reference does not convert units, so cm assets (Isaac People,
    ArchVis) come in 100x too big (verified on 6.0.1, look.py:127-135 / commit 1f62514)."""
    try:
        from pxr import Usd, UsdGeom
        return float(UsdGeom.GetStageMetersPerUnit(Usd.Stage.Open(url, load=Usd.Stage.LoadNone)))
    except Exception:
        return 1.0
