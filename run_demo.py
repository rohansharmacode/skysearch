"""Render a mission video.  python run_demo.py --seed 7 --duration 300 --out out/demo.mp4"""
import argparse, os, subprocess, tempfile
from skysearch.config import Config
from skysearch.scenario import Scenario
from skysearch.render import Renderer

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=7)
ap.add_argument("--duration", type=float, default=300)
ap.add_argument("--every", type=int, default=1, help="render every Nth tick")
ap.add_argument("--fps", type=int, default=12)
ap.add_argument("--out", default="out/demo.mp4")
ap.add_argument("--stills", default="", help="comma-separated times to save PNG stills")
a = ap.parse_args()

scen = Scenario(Config(seed=a.seed, duration=a.duration))
r = Renderer(scen)
stills = {float(x) for x in a.stills.split(",") if x}
tmp = tempfile.mkdtemp()
k = 0
tick = 0
while scen.swarm.t < a.duration:
    scen.step(); tick += 1
    if tick % a.every == 0:
        r.frame().savefig(f"{tmp}/f{k:05d}.png", facecolor=r.fig.get_facecolor()); k += 1
    for s_ in list(stills):
        if scen.swarm.t >= s_:
            r.frame().savefig(a.out.replace(".mp4", f"_t{int(s_)}.png"), facecolor=r.fig.get_facecolor()); stills.remove(s_)
os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(a.fps), "-i", f"{tmp}/f%05d.png",
                "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23", a.out], check=True)
print("wrote", a.out, k, "frames")
