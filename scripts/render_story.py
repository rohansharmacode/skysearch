"""Render the storyboard demo video (earthquake scenario) to MP4.

    python scripts/render_story.py --seed 4 --out out/skysearch_story.mp4 --workers 3

The simulation runs once, in order; frames are drawn in parallel worker processes.
Drone positions are interpolated between 0.5 s simulation ticks so motion looks smooth.
"""
import argparse, os, pickle, subprocess, sys, time
from multiprocessing import Pool
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from skysearch.usar.config import USARConfig
from skysearch.usar.scenario import USARScenario
from skysearch.usar.render import USARRenderer

_R = None


def draw(job):
    global _R
    i, blob, outdir, speed = job
    s, breath, pos, focus = pickle.loads(blob)
    if _R is None:
        _R = USARRenderer(s, speed_label=speed)
    _R.s, _R.breath, _R.pos, _R.focus_name = s, breath, pos, focus
    _R._last_note = s.fleet.t           # history comes from the main process
    _R.save(os.path.join(outdir, f"f{i:05d}.png"))
    return i


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=4)
    ap.add_argument("--out", default="out/skysearch_story.mp4")
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--sub", type=int, default=2, help="frames per 0.5 s tick")
    ap.add_argument("--fps", type=int, default=12)
    ap.add_argument("--tail", type=float, default=5.0, help="seconds to hold the final frame")
    ap.add_argument("--gemini", action="store_true", help="crew briefing written by Gemini (needs GEMINI_API_KEY)")
    a = ap.parse_args()
    cfg = USARConfig(story=True, seed=a.seed)
    s = USARScenario(cfg)
    speed = f"{cfg.dt * a.fps / a.sub:.0f}× real time"
    R = USARRenderer(s, speed_label=speed)
    outdir = os.path.splitext(a.out)[0] + "_frames"
    os.makedirs(outdir, exist_ok=True)
    t_end = s.t_handoff + 1.0

    def jobs():
        i = 0
        while s.fleet.t < t_end:
            prev = {d.name: d.pos.copy() for d in s.fleet.drones}
            s.step()
            R.note_radar()
            if a.gemini and s.fleet.briefing and not s.fleet.flags & {"gemini"}:
                s.fleet.flags.add("gemini")
                import briefing
                briefing.apply(s.fleet)
            focus = R.pick_focus().name
            R.focus = R.pick_focus()
            for k in range(1, a.sub + 1):
                al = k / a.sub
                pos = {d.name: prev[d.name] + (d.pos - prev[d.name]) * al for d in s.fleet.drones}
                yield (i, pickle.dumps((s, list(R.breath), pos, focus)), outdir, speed)
                i += 1
    t0 = time.time()
    with Pool(a.workers) as pool:
        n = 0
        for _ in pool.imap(draw, jobs(), chunksize=2):
            n += 1
            if n % 50 == 0:
                print(f"{n} frames, sim t = {s.fleet.t:.1f} s, {time.time() - t0:.0f} s", flush=True)
    hold = int(a.tail * a.fps)
    last = os.path.join(outdir, f"f{n - 1:05d}.png")
    for j in range(hold):
        os.link(last, os.path.join(outdir, f"f{n + j:05d}.png")) if not os.path.exists(os.path.join(outdir, f"f{n + j:05d}.png")) else None
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(a.fps), "-i", os.path.join(outdir, "f%05d.png"),
           "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", "-preset", "medium", "-movflags", "+faststart", a.out]
    subprocess.run(cmd, check=True)
    print("wrote", a.out, f"({n + hold} frames, {(n + hold) / a.fps:.0f} s)")
    for t, m in s.fleet.captions:
        print(f"{t:6.1f}  {m}")


if __name__ == "__main__":
    main()
