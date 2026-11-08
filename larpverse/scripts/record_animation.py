"""Render an animation page to MP4, frame by frame, using its window.seek(t) hook.

    python scripts/record_animation.py http://localhost:8765/animation/index.html out.mp4 --duration 66
    python scripts/record_animation.py "http://localhost:8765/animation/grid-search.html#s4242" out.mp4 --duration 88

Needs: playwright (uses the system Google Chrome) and imageio-ffmpeg (bundled ffmpeg with libx264).
"""
import argparse
import subprocess
import sys
import time

import imageio_ffmpeg
from playwright.sync_api import sync_playwright

HIDE = "#bar, #layers, #follow, #bigPlay, #errbar { display: none !important; }"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("url")
    ap.add_argument("out")
    ap.add_argument("--duration", type=float, required=True)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--probe", action="store_true", help="render 3 frames, report timing and GPU, then stop")
    a = ap.parse_args()

    ff = subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error", "-f", "image2pipe", "-framerate", str(a.fps),
                           "-c:v", "mjpeg", "-i", "-", "-c:v", "libx264", "-preset", "veryfast", "-crf", "18", "-threads", "2", "-x264-params", "rc-lookahead=10:ref=2",
                           "-pix_fmt", "yuv420p", "-movflags", "+faststart", a.out], stdin=subprocess.PIPE) if not a.probe else None
    with sync_playwright() as p:
        b = p.chromium.launch(channel="chrome", headless=True,
                              args=["--enable-gpu", "--ignore-gpu-blocklist", "--use-angle=vulkan", "--enable-features=Vulkan", "--enable-unsafe-swiftshader"])
        pg = b.new_page(viewport={"width": a.width, "height": a.height}, device_scale_factor=1)
        pg.goto(a.url)
        pg.wait_for_function("typeof window.seek === 'function' && !document.getElementById('loading')", timeout=120000)
        pg.evaluate("document.fonts.ready")
        pg.add_style_tag(content=HIDE)
        gpu = pg.evaluate("""() => { const gl = document.createElement('canvas').getContext('webgl');
            const e = gl && gl.getExtension('WEBGL_debug_renderer_info'); return e ? gl.getParameter(e.UNMASKED_RENDERER_WEBGL) : 'unknown'; }""")
        print("renderer:", gpu, flush=True)
        n = int(round((a.duration - a.start) * a.fps)) if not a.probe else 3
        t0 = time.time()
        for i in range(n + 1):
            t = min(a.duration, a.start + i / a.fps)
            pg.evaluate("t => { window.seek(t); return new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r))); }", t)
            img = pg.screenshot(type="jpeg", quality=92)
            if ff:
                ff.stdin.write(img)
            if i % (a.fps * 5) == 0:
                el = time.time() - t0
                print(f"frame {i}/{n} t={t:.1f}s  {el / (i + 1):.3f}s/frame  eta {el / (i + 1) * (n - i):.0f}s", flush=True)
        b.close()
    if ff:
        ff.stdin.close()
        ff.wait()
        print("wrote", a.out, flush=True)


if __name__ == "__main__":
    sys.exit(main())
