"""``--cinema 1``: a feature-film / VFX-plate look for the take (Rohan 2026-11-08: "take inspiration from movies,
make it ultra photorealistic using CGI").

What film VFX does that a raw real-time render lacks, and what we set (all in try/except, every key read back
and logged, so the rehearsal log shows which keys this Kit build knows):

* lens and sensor: a ~72 deg shutter's motion blur (exposure fraction 0.2; take1 showed 0.5 smears the pans and
  the look research puts real drone footage at 0.15-0.25), very subtle lateral chromatic aberration at the frame edges, a soft bloom/flare on the sun's
  speculars (flare scale 0.04: the collapse map's verified value), fine grain, light vignette;
* (removed after the 02:40 look check: the film grade, the warm haze colour and volumetric in-scattering washed the
  grid out, and this Kit build doesn't know /rtx/raytracing/inscattering).

Lighting is left to photoreal's defaults (evening_road sky, warm sun): the 02:40 look check rated that the most
natural light. Only lens/sensor behaviour is added here.
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
    # No grade, haze or sky changes: the look check (02:40) found the film grade + warm haze + bright HDRI washed out
    # (low contrast, tan cast) and /rtx/raytracing/inscattering unknown to this Kit. The light stays photoreal's.
}
DISASTER_SKY = None   # was kloofendal + sun 1600: overexposed and washed out on the grid (look check 02:40); keep the default sky


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
