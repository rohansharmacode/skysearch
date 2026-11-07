"""Version shims so the adapter runs on Isaac Sim 4.5, 5.x and 6.x.

Only stable APIs are used elsewhere: pxr (USD), omni.usd, omni.timeline, omni.physx scene queries.
Anything version-specific lives here and degrades gracefully.
"""
import importlib


def make_app(headless: bool, width: int = 1600, height: int = 900):
    """Create SimulationApp. Must run before importing any omni/pxr module."""
    try:
        from isaacsim import SimulationApp            # 4.5+
    except ImportError:
        from omni.isaac.kit import SimulationApp      # 4.0-4.2
    return SimulationApp({"headless": headless, "width": width, "height": height})


def enable_ext(name: str) -> bool:
    try:
        import omni.kit.app
        mgr = omni.kit.app.get_app().get_extension_manager()
        mgr.set_extension_enabled_immediate(name, True)
        return bool(mgr.is_extension_enabled(name))
    except Exception:
        return False


def debug_draw():
    """Debug-draw interface (points/lines overlay) or None if the extension is unavailable."""
    for ext in ("isaacsim.util.debug_draw", "omni.isaac.debug_draw"):
        if not enable_ext(ext):
            continue
        try:
            return importlib.import_module(ext + "._debug_draw").acquire_debug_draw_interface()
        except Exception:
            continue
    return None


def set_viewport_camera(path: str) -> bool:
    try:
        from omni.kit.viewport.utility import get_active_viewport
        vp = get_active_viewport()
        if vp is None:
            return False
        vp.camera_path = path
        return True
    except Exception:
        return False


def physx_raycaster():
    """Return f(origin3, dir3, max_dist) -> (x, y, z) | None using PhysX scene queries."""
    import carb
    from omni.physx import get_physx_scene_query_interface
    sq = get_physx_scene_query_interface()

    def cast(o, d, dist):
        n = (float(d[0]) ** 2 + float(d[1]) ** 2 + float(d[2]) ** 2) ** 0.5 or 1.0   # PhysX wants a unit dir
        hit = sq.raycast_closest(carb.Float3(float(o[0]), float(o[1]), float(o[2])),
                                 carb.Float3(float(d[0]) / n, float(d[1]) / n, float(d[2]) / n), float(dist))
        if not hit or not hit.get("hit"):
            return None
        p = hit["position"]
        try:
            return (p[0], p[1], p[2])
        except TypeError:
            return (p.x, p.y, p.z)
    return cast


def new_stage():
    """Fresh stage in the running Kit app; returns the Usd.Stage."""
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


def assets_root():
    """Isaac assets root URL (Nucleus or S3 mirror) or None."""
    for mod in ("isaacsim.storage.native", "omni.isaac.nucleus", "omni.isaac.core.utils.nucleus"):
        try:
            root = importlib.import_module(mod).get_assets_root_path()
            if root:
                return root
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
