"""Generative-AI refinement of the collapse map's scan textures (run on the GPU box, in its own venv).

The map's terrain and roof textures come from the photogrammetry scan's screenshots: real colours, but only
~15 cm per pixel, Lanczos-blown-up 4x, so close shots look smeared ("wrapped"). This keeps the real colours and
layout and lets a text-guided diffusion upscaler (stabilityai/stable-diffusion-x4-upscaler) re-draw the fine
detail: broken concrete edges, rebar, gravel, dust. Each texture is taken back to its true resolution, cut into
overlapping crops, upscaled 4x with a scene prompt, feather-blended, and colour-matched to the original at low
frequency (so the AI adds detail, never moves or recolours things). A matching normal map is derived from the
new detail. Output: <out>/<name>_albedo.png and _normal.png, used by --scan-ai <out> (isaac/urban_look.py).

    python3 -m venv /workspace/aienv && /workspace/aienv/bin/pip install torch diffusers transformers accelerate \
        safetensors opencv-python-headless pillow
    /workspace/aienv/bin/python isaac/tools/ai_textures.py --src <repo>/collapse_site/textures --out /workspace/scan_ai
"""
from __future__ import annotations

import argparse
import glob
import os
import time

import numpy as np

PROMPTS = {
    "terrain": "aerial drone photograph of an earthquake building collapse site in Taiwan, rubble of broken "
               "reinforced concrete slabs, bent rebar, bricks, ceramic wall tiles, dust, debris, dusty street, "
               "sharp focus, highly detailed, photorealistic, natural daylight",
    "scan_roofs": "aerial drone photograph of rooftops and collapsed concrete floor slabs after an earthquake, "
                  "dusty concrete, debris, cracks, sharp focus, highly detailed, photorealistic",
}
NEG = "blurry, painting, illustration, cartoon, cgi, text, watermark, people, oversaturated, smooth plastic"


def feather(h, w, m):
    wy = np.minimum(np.arange(h) + 1, np.arange(h)[::-1] + 1).astype(np.float32)
    wx = np.minimum(np.arange(w) + 1, np.arange(w)[::-1] + 1).astype(np.float32)
    return np.clip(np.minimum.outer(wy, wx) / m, 0.02, 1.0)


def upscale(pipe, img, prompt, crop=256, ov=48, steps=25, noise=30, guidance=7.0, seed=0):
    """4x diffusion upscale of an (H, W, 3) uint8 image by overlapping crops, feather-blended."""
    import torch
    from PIL import Image
    H, W = img.shape[:2]
    out = np.zeros((4 * H, 4 * W, 3), np.float32)
    acc = np.zeros((4 * H, 4 * W, 1), np.float32)
    step = crop - ov
    ys = list(range(0, max(H - crop, 0) + 1, step)) or [0]
    xs = list(range(0, max(W - crop, 0) + 1, step)) or [0]
    if ys[-1] + crop < H:
        ys.append(H - crop)
    if xs[-1] + crop < W:
        xs.append(W - crop)
    g = torch.Generator("cuda").manual_seed(seed)
    wgt = feather(4 * crop, 4 * crop, 4 * ov)[..., None]
    for y in ys:
        for x in xs:
            c = img[y:y + crop, x:x + crop]
            res = pipe(prompt=prompt, negative_prompt=NEG, image=Image.fromarray(c), num_inference_steps=steps,
                       noise_level=noise, guidance_scale=guidance, generator=g).images[0]
            r = np.asarray(res, np.float32)
            hh, ww = r.shape[:2]
            out[4 * y:4 * y + hh, 4 * x:4 * x + ww] += r * wgt[:hh, :ww]
            acc[4 * y:4 * y + hh, 4 * x:4 * x + ww] += wgt[:hh, :ww]
    return out / np.maximum(acc, 1e-6)


def colour_match(new, ref, sigma):
    """Keep the AI's high-frequency detail, but the original's low-frequency colour (no drift, no recolouring)."""
    import cv2
    ref = cv2.resize(ref.astype(np.float32), (new.shape[1], new.shape[0]), interpolation=cv2.INTER_CUBIC)
    lo_new = cv2.GaussianBlur(new, (0, 0), sigma)
    lo_ref = cv2.GaussianBlur(ref, (0, 0), sigma)
    return np.clip(new - lo_new + lo_ref, 0, 255)


def normal_from(albedo, strength=2.5, base=None):
    """Tangent-space normal map from the detail of the albedo (high-pass luminance as height)."""
    import cv2
    L = cv2.cvtColor(albedo.astype(np.uint8), cv2.COLOR_RGB2GRAY).astype(np.float32) / 255.0
    h = L - cv2.GaussianBlur(L, (0, 0), 6)
    gx = cv2.Sobel(h, cv2.CV_32F, 1, 0, ksize=3) * strength
    gy = cv2.Sobel(h, cv2.CV_32F, 0, 1, ksize=3) * strength
    n = np.dstack([-gx, -gy, np.ones_like(h)])
    if base is not None:                                   # combine with the map's own (macro) normal map
        b = cv2.resize(base, (n.shape[1], n.shape[0])).astype(np.float32) / 127.5 - 1.0
        n = np.dstack([n[..., 0] + b[..., 0], n[..., 1] + b[..., 1], n[..., 2] * b[..., 2]])
    n /= np.linalg.norm(n, axis=2, keepdims=True)
    return ((n * 0.5 + 0.5) * 255).astype(np.uint8)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="collapse_site/textures")
    ap.add_argument("--out", required=True)
    ap.add_argument("--only", default="", help="comma list of names (e.g. terrain_r1c1,scan_roofs) for a quick test")
    ap.add_argument("--true-px", type=int, default=512, help="terrain tiles' true resolution (generator LP)")
    ap.add_argument("--roof-down", type=float, default=2.0, help="scan_roofs: downscale before the 4x upscale")
    ap.add_argument("--steps", type=int, default=25)
    ap.add_argument("--noise", type=int, default=30, help="upscaler noise level: higher = more invented detail")
    a = ap.parse_args(argv)
    import cv2
    import torch
    from diffusers import StableDiffusionUpscalePipeline
    os.makedirs(a.out, exist_ok=True)
    pipe = StableDiffusionUpscalePipeline.from_pretrained("stabilityai/stable-diffusion-x4-upscaler",
                                                          torch_dtype=torch.float16).to("cuda")
    pipe.set_progress_bar_config(disable=True)
    jobs = [(os.path.basename(f)[:-len("_albedo.jpg")], f) for f in sorted(glob.glob(os.path.join(a.src, "terrain_r*_albedo.jpg")))]
    jobs.append(("scan_roofs", os.path.join(a.src, "scan_roofs_albedo.jpg")))
    only = {s for s in a.only.split(",") if s}
    for name, f in jobs:
        if only and name not in only:
            continue
        t = time.time()
        src = cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2RGB)
        if name == "scan_roofs":
            small = cv2.resize(src, (int(src.shape[1] / a.roof_down), int(src.shape[0] / a.roof_down)),
                               interpolation=cv2.INTER_AREA)
            prompt = PROMPTS["scan_roofs"]
        else:
            small = cv2.resize(src, (a.true_px, a.true_px), interpolation=cv2.INTER_AREA)
            prompt = PROMPTS["terrain"]
        up = upscale(pipe, small, prompt, steps=a.steps, noise=a.noise, seed=hash(name) & 0xFFFF)
        up = colour_match(up, src, sigma=max(up.shape[:2]) / 512 * 6)
        nf = os.path.join(a.src, f"{name}_normal.jpg")
        if not os.path.exists(nf):
            nf = os.path.join(a.src, f"{name}_normal.png")
        base = cv2.cvtColor(cv2.imread(nf), cv2.COLOR_BGR2RGB) if os.path.exists(nf) else None
        cv2.imwrite(os.path.join(a.out, f"{name}_albedo.png"), cv2.cvtColor(up.astype(np.uint8), cv2.COLOR_RGB2BGR))
        cv2.imwrite(os.path.join(a.out, f"{name}_normal.png"),
                    cv2.cvtColor(normal_from(up, base=base), cv2.COLOR_RGB2BGR))
        print(f"[ai_textures] {name}: {small.shape[1]}x{small.shape[0]} -> {up.shape[1]}x{up.shape[0]} "
              f"in {time.time() - t:.0f} s", flush=True)


if __name__ == "__main__":
    main()
