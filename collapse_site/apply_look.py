"""Open the collapse site in Isaac Sim and apply its RTX post-processing look.

Two ways to use it:

1) Inside the Isaac Sim GUI (any version 4.5 / 5.x / 6.x):
   File > Open > collapse_site.usda, then Window > Script Editor > File > Open > apply_look.py > Run.
   It only changes render settings, so it is safe to run on any stage.

2) Standalone, from Isaac's Python:
   ~/isaacsim/python.sh apply_look.py                       # opens collapse_site.usda next to this file
   ~/isaacsim/python.sh apply_look.py --camera Oblique_South
   ~/isaacsim/python.sh apply_look.py --headless --frames 120

The same values are also stored in the stage's customLayerData["renderSettings"], so Kit may restore
them on its own when the stage opens. Unknown keys on older or newer versions are skipped harmlessly.
"""
import os
import sys

SETTINGS = {
    # tone mapping / exposure (ACES, fixed exposure so the baked photo textures keep their look)
    "/rtx/post/tonemap/op": 6,
    "/rtx/post/tonemap/filmIso": 100.0,
    "/rtx/post/tonemap/cameraShutter": 50.0,
    "/rtx/post/tonemap/fNumber": 5.0,
    "/rtx/post/histogram/enabled": False,
    # lighting quality
    "/rtx/ambientOcclusion/enabled": True,
    "/rtx/ambientOcclusion/rayLength": 3.0,
    "/rtx/indirectDiffuse/enabled": True,
    "/rtx/shadows/enabled": True,
    "/rtx/reflections/enabled": True,
    # colour grade: a little more contrast + saturation (matches the scan viewer's look)
    "/rtx/post/colorcorr/enabled": True,
    "/rtx/post/colorcorr/saturation": [1.08, 1.08, 1.08],
    "/rtx/post/colorcorr/contrast": [1.12, 1.12, 1.12],
    "/rtx/post/colorcorr/gamma": [1.0, 1.0, 1.0],
    "/rtx/post/colorcorr/gain": [1.0, 1.0, 1.0],
    "/rtx/post/colorcorr/offset": [0.0, 0.0, 0.0],
    # vignette + light film grain (all the other 'TV noise' effects off)
    "/rtx/post/tvNoise/enabled": True,
    "/rtx/post/tvNoise/enableVignetting": True,
    "/rtx/post/tvNoise/vignettingSize": 107.0,
    "/rtx/post/tvNoise/vignettingStrength": 0.55,
    "/rtx/post/tvNoise/enableFilmGrain": True,
    "/rtx/post/tvNoise/grainAmount": 0.025,
    "/rtx/post/tvNoise/grainSize": 1.4,
    "/rtx/post/tvNoise/enableScanlines": False,
    "/rtx/post/tvNoise/enableScrollBug": False,
    "/rtx/post/tvNoise/enableVignettingFlickering": False,
    "/rtx/post/tvNoise/enableGhostFlickering": False,
    "/rtx/post/tvNoise/enableWaveDistortion": False,
    "/rtx/post/tvNoise/enableVerticalLines": False,
    "/rtx/post/tvNoise/enableRandomSplotches": False,
    # subtle bloom
    "/rtx/post/lensFlares/enabled": True,
    "/rtx/post/lensFlares/flareScale": 0.04,
    # dusty haze (Z-up height fog)
    "/rtx/fog/enabled": True,
    "/rtx/fog/fogZup/enabled": True,
    "/rtx/fog/fogColor": [0.78, 0.74, 0.68],
    "/rtx/fog/fogColorIntensity": 1.0,
    "/rtx/fog/fogStartDist": 60.0,
    "/rtx/fog/fogEndDist": 900.0,
    "/rtx/fog/fogDistanceDensity": 0.35,
    "/rtx/fog/fogStartHeight": 0.0,
    "/rtx/fog/fogEndHeight": 60.0,
    "/rtx/fog/fogHeightDensity": 0.25,
}


def apply_look(verbose=True):
    import carb
    s = carb.settings.get_settings()
    ok = 0
    for k, v in SETTINGS.items():
        try:
            if isinstance(v, list):
                s.set_float_array(k, v)
            elif isinstance(v, bool):
                s.set_bool(k, v)
            elif isinstance(v, int):
                s.set_int(k, v)
            else:
                s.set_float(k, float(v))
            ok += 1
        except Exception as e:  # key renamed in this version: skip
            if verbose:
                print(f"[collapse_site] skipped {k}: {e}")
    if verbose:
        print(f"[collapse_site] applied {ok}/{len(SETTINGS)} render settings")


def set_camera(name):
    try:
        from omni.kit.viewport.utility import get_active_viewport
        get_active_viewport().camera_path = f"/CollapseSite/Cameras/{name}"
    except Exception as e:
        print(f"[collapse_site] could not switch camera: {e}")


def _inside_kit():
    try:
        import omni.kit.app
        return omni.kit.app.get_app() is not None
    except Exception:
        return False


if _inside_kit() and __name__ in ("__main__", "builtins") and "--standalone" not in sys.argv:
    # Script Editor: just apply to the running app
    apply_look()
elif __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--usd", default=os.path.join(here, "collapse_site.usda"))
    ap.add_argument("--camera", default="Oblique_South")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--frames", type=int, default=0, help="run N frames then exit (0 = until window closed)")
    ap.add_argument("--standalone", action="store_true")
    a = ap.parse_args()
    try:
        from isaacsim import SimulationApp
    except ImportError:
        from omni.isaac.kit import SimulationApp
    app = SimulationApp({"headless": a.headless, "width": 1920, "height": 1080, "renderer": "RayTracedLighting"})
    import omni.usd
    omni.usd.get_context().open_stage(a.usd)
    for _ in range(30):
        app.update()
    apply_look()
    set_camera(a.camera)
    n = 0
    while app.is_running() and (a.frames == 0 or n < a.frames):
        app.update(); n += 1
    app.close()
