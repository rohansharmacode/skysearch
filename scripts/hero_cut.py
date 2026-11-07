"""Cut one 'hero' video from a multi-camera earthquake_scene.py recording.

    python scripts/hero_cut.py out/quake4 out/quake4/hero  [--quake_frames 375]

Writes <dst>/f%05d.png at 30 fps (quake frames 1:1, mission ticks held 5 frames each, i.e. 6 ticks/s),
switching cameras at the storyboard beats, with the mission dashboard as an inset during the search.
Encode: ffmpeg -framerate 30 -i <dst>/f%05d.png -c:v libx264 -pix_fmt yuv420p -crf 20 hero.mp4
"""
import argparse, os
from PIL import Image

ap = argparse.ArgumentParser()
ap.add_argument("src"); ap.add_argument("dst")
ap.add_argument("--quake_frames", type=int, default=375, help="recorded frames that belong to the earthquake (30 fps)")
ap.add_argument("--hold", type=int, default=5, help="output frames per mission tick")
a = ap.parse_args()
cams = {c: os.path.join(a.src, d) for c, d in (("orbit", "isaac"), ("corridor", "isaac_collapse"),
                                                 ("chase", "isaac_chase_C1"), ("fpv", "isaac_fpv_C1"), ("dash", "dash"))}
n = len([f for f in os.listdir(cams["orbit"]) if f.endswith(".png")])
Q = min(a.quake_frames, n)


def cam_for(i):
    """Which camera to show for recorded frame i (the storyboard)."""
    if i < Q:                                   # the earthquake, real time
        q = i / max(Q, 1)
        return "orbit" if q < 0.18 else "corridor" if q < 0.55 else "orbit" if q < 0.78 else "corridor"
    t = (i - Q) * 0.5                           # mission seconds
    if t < 14: return "orbit"                   # launch
    if t < 28: return "chase"                   # crevice drone flies in
    if t < 46: return "fpv"                     # under the slab: victim A confirmed
    if t < 58: return "chase"
    return "orbit"                              # smoke in the east wing, radar takes over


os.makedirs(a.dst, exist_ok=True)
k = 0
for i in range(n):
    name = f"f{i:05d}.png"
    c = cam_for(i)
    p = os.path.join(cams[c], name)
    if not os.path.exists(p):
        p = os.path.join(cams["orbit"], name)
    img = Image.open(p).convert("RGB")
    W, H = img.size
    if i >= Q and os.path.exists(os.path.join(cams["dash"], name)):
        d = Image.open(os.path.join(cams["dash"], name)).convert("RGB")
        dw = int(W * 0.36); dh = int(d.height * dw / d.width)
        d = d.resize((dw, dh))
        img.paste(d, (W - dw - 16, H - dh - 16))
    reps = 1 if i < Q else a.hold
    for _ in range(reps):
        img.save(os.path.join(a.dst, f"f{k:05d}.png"), compress_level=1)
        k += 1
print(f"hero: {n} recorded frames ({Q} quake) -> {k} output frames at 30 fps = {k / 30:.1f} s")
