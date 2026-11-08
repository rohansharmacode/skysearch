"""``--cinema 1``: a feature-film / VFX-plate look for the take (Rohan 2026-11-08: "take inspiration from movies,
make it ultra photorealistic using CGI").

What film VFX does that a raw real-time render lacks, and what we set (all in try/except, every key read back
and logged, so the rehearsal log shows which keys this Kit build knows):

* lens and sensor: a ~72 deg shutter's motion blur (exposure fraction 0.2; take1 showed 0.5 smears the pans and
  the look research puts real drone footage at 0.15-0.25), very subtle lateral chromatic aberration at the frame edges, a soft bloom/flare on the sun's
  speculars (flare scale 0.04: the collapse map's verified value), fine grain, light vignette;
* atmosphere: dusty air in the sunlight (RTX volumetric in-scattering, low density, only where the take renderer
  supports it) on top of the distance haze, so the sun reads as shafts through dust after a collapse;
* grade: a restrained "disaster film" grade, i.e. warm-grey dust, lifted blacks, slightly cool shadows, less
  saturation than video. It is applied after the grid / urban grade, so it wins where keys overlap.

Lighting is set with the existing flags; the recommended disaster-film sky is ``--hdri
kloofendal_48d_partly_cloudy_4k.hdr`` (soft partly cloudy daylight) with ``--sun 1600 --sun-elev 32``.
Nothing here touches the stage, the mission or what the sensors see.
"""
from __future__ import annotations

CINEMA = {
    # lens / sensor
    "/rtx/post/motionblur/enabled": True,
    "/rtx/post/motionblur/exposureFraction": 0.2,
    "/rtx/post/motionblur/numSamples": 8,
    "/rtx/post/chromaticAberration/enabled": True,
    "/rtx/post/chromaticAberration/strengthR": -0.004,
    "/rtx/post/chromaticAberration/strengthG": 0.0,
    "/rtx/post/chromaticAberration/strengthB": 0.004,
    "/rtx/post/lensFlares/enabled": True,
    "/rtx/post/lensFlares/flareScale": 0.04,
    "/rtx/post/tvNoise/enabled": True,
    "/rtx/post/tvNoise/enableFilmGrain": True,
    "/rtx/post/tvNoise/grainAmount": 0.018,
    "/rtx/post/tvNoise/grainSize": 1.15,
    "/rtx/post/tvNoise/colorAmount": 0.25,
    "/rtx/post/tvNoise/enableVignetting": True,
    "/rtx/post/tvNoise/vignettingSize": 105.0,
    "/rtx/post/tvNoise/vignettingStrength": 0.05,
    # grade: dusty, slightly lifted blacks, cool shadows / warm mids, less saturation than broadcast video
    "/rtx/post/colorcorr/enabled": True,
    "/rtx/post/colorcorr/saturation": [0.82, 0.82, 0.82],
    "/rtx/post/colorcorr/contrast": [1.06, 1.06, 1.06],
    "/rtx/post/colorcorr/gamma": [1.0, 1.0, 1.0],
    "/rtx/post/colorcorr/gain": [1.0, 0.99, 0.97],
    "/rtx/post/colorcorr/offset": [0.004, 0.005, 0.008],
    # atmosphere: dusty air (distance haze + volumetric in-scattering where supported)
    "/rtx/fog/enabled": True,
    "/rtx/fog/fogColor": [0.76, 0.74, 0.71],
    "/rtx/raytracing/inscattering/enabled": True,
    "/rtx/raytracing/inscattering/densityMult": 0.25,
    "/rtx/raytracing/inscattering/maxDistance": 400.0,
    "/rtx/raytracing/inscattering/singleScatteringAlbedo": [0.92, 0.90, 0.86],
    "/rtx/raytracing/inscattering/transmittanceColor": [0.96, 0.94, 0.91],
    "/rtx/raytracing/inscattering/transmittanceMeasurementDistance": 250.0,
    "/rtx/raytracing/inscattering/useDetailNoise": True,
}
DISASTER_SKY = dict(hdri="kloofendal_48d_partly_cloudy_4k.hdr", sun=1600.0, sun_elev=32.0)


def apply(a=None, log=print, extra=None):
    """Set the film look. Returns {"known": [...], "new": [...], "failed": [...]}: ``new`` keys did not exist in
    this Kit build before we set them (probably no effect), which the rehearsal log reports."""
    out = {"known": [], "new": [], "failed": []}
    try:
        import carb
        s = carb.settings.get_settings()
    except Exception as e:                                  # noqa: BLE001  (dry run / plain pxr)
        log(f"[cinema] film look skipped ({type(e).__name__})")
        return out
    for k, v in {**CINEMA, **(extra or {})}.items():
        try:
            existed = s.get(k) is not None
            if isinstance(v, bool):
                s.set_bool(k, v)
            elif isinstance(v, int):
                s.set_int(k, v)
            elif isinstance(v, float):
                s.set_float(k, v)
            else:
                s.set(k, v)
            out["known" if existed else "new"].append(k)
        except Exception:                                   # noqa: BLE001
            out["failed"].append(k)
    log(f"[cinema] film look: {len(out['known'])} settings applied; unknown to this Kit (likely no effect): "
        f"{', '.join(k.split('/rtx/')[-1] for k in out['new']) or 'none'}; failed: {len(out['failed'])}")
    return out
