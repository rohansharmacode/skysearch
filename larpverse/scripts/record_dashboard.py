"""Record the operator dashboard as an MP4 by replaying a finished run's frames, or replay them live.

A run made with ``--frames`` (scripts/demo.sh does this) leaves dashboard/data/frames/state_XXXX.json, one per
simulated second. This script copies them one at a time to dashboard/data/replay/state.json (plus the run's
pointcloud.bin) and points the dashboard at that folder with ``?data=data/replay``, so the live run's own
dashboard/data/state.json is never touched and the orchestrator's dev server (repo root on :8765) serves it.

Record (headless Chrome, frame by frame, deterministic pacing; run through the browser slot):
    scripts/heavy.sh --browser <rec-venv>/bin/python scripts/record_dashboard.py \
        --out media/larpverse-dashboard-seed7.mp4

Live replay for a demo fallback (no browser, no Mission: just paces the frames; Ctrl-C stops):
    .venv/bin/python scripts/record_dashboard.py --live --speed 6
    then open http://localhost:8765/dashboard/?data=data/replay&poll=250

Video pacing: every ``--step``-th mission frame is shown, ``--per-sec`` of them per video second (default every
2nd frame, 3 per second = 6x mission speed), then the last state is held and the first ping in the side list is
clicked to show its LiDAR detail view for ``--detail`` seconds.

Needs (record mode only): playwright (system Google Chrome, channel="chrome") and imageio-ffmpeg.
"""
import argparse
import glob
import json
import os
import shutil
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def write_atomic(path, data):
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)


def load_frames(frames_dir):
    files = sorted(glob.glob(os.path.join(frames_dir, "state_*.json")))
    if not files:
        sys.exit(f"no frames in {frames_dir}: run the mission with --frames first (scripts/demo.sh does)")
    return files


def prepare_replay(src_dir, replay_dir, first_frame):
    os.makedirs(replay_dir, exist_ok=True)
    with open(first_frame, "rb") as f:
        raw = f.read()
    pc = (json.loads(raw).get("pointcloud") or {}).get("url") or "pointcloud.bin"
    src_pc = os.path.join(src_dir, pc)
    if os.path.exists(src_pc):
        dst = os.path.join(replay_dir, pc)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copyfile(src_pc, dst)
    else:
        print(f"warning: {src_pc} missing; the LiDAR view will use demo points", file=sys.stderr)
    write_atomic(os.path.join(replay_dir, "state.json"), raw)


def frame_t(raw):
    try:
        return float(json.loads(raw).get("t", 0.0))
    except (ValueError, AttributeError):
        return 0.0


# ------------------------------------------------------------------ live replay (no browser)
def live(a, files):
    replay_state = os.path.join(a.replay_dir, "state.json")
    prepare_replay(a.src_dir, a.replay_dir, files[0])
    rel = os.path.relpath(a.replay_dir, os.path.join(ROOT, "dashboard"))
    print(f"live replay of {len(files)} frames at x{a.speed}: open {a.url}?data={rel}&poll=250  (Ctrl-C stops)", flush=True)
    prev_t = None
    try:
        for i, path in enumerate(files):
            with open(path, "rb") as f:
                raw = f.read()
            t = frame_t(raw)
            if prev_t is not None:
                time.sleep(max(0.0, (t - prev_t) / a.speed))
            prev_t = t
            write_atomic(replay_state, raw)
            if i % 60 == 0:
                print(f"  frame {i}/{len(files)}  t={t:.0f} s", flush=True)
    except KeyboardInterrupt:
        print("\nstopped; the dashboard keeps the last replayed state", flush=True)
        return 0
    print("replay finished (final state stays on the dashboard)", flush=True)
    return 0


# ------------------------------------------------------------------ recording
def record(a, files):
    import imageio_ffmpeg
    from playwright.sync_api import sync_playwright

    replay_state = os.path.join(a.replay_dir, "state.json")
    idx = list(range(0, len(files), a.step))
    if idx[-1] != len(files) - 1:
        idx.append(len(files) - 1)
    per_sample = max(1, round(a.fps / a.per_sec))
    shots = max(1, min(a.shots, per_sample))
    reps = [per_sample // shots + (1 if k < per_sample % shots else 0) for k in range(shots)]
    total_s = len(idx) * per_sample / a.fps + a.hold + a.detail
    print(f"{len(files)} frames -> {len(idx)} samples (every {a.step}), {a.per_sec}/s video -> ~{total_s:.0f} s of video", flush=True)

    os.makedirs(os.path.dirname(os.path.abspath(a.out)) or ".", exist_ok=True)
    prepare_replay(a.src_dir, a.replay_dir, files[idx[0]])
    rel = os.path.relpath(a.replay_dir, os.path.join(ROOT, "dashboard"))
    url = f"{a.url}?data={rel}&poll=100"

    ff = subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error", "-f", "image2pipe",
                           "-framerate", str(a.fps), "-c:v", "mjpeg", "-i", "-", "-c:v", "libx264", "-preset", "medium",
                           "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart", a.out], stdin=subprocess.PIPE)
    written = 0

    def emit(img, n):
        nonlocal written
        for _ in range(n):
            ff.stdin.write(img)
        written += n

    with sync_playwright() as p:
        b = p.chromium.launch(channel="chrome", headless=True,
                              args=["--enable-gpu", "--ignore-gpu-blocklist", "--use-angle=vulkan",
                                    "--enable-features=Vulkan", "--enable-unsafe-swiftshader"])
        try:
            pg = b.new_page(viewport={"width": a.width, "height": a.height}, device_scale_factor=1)
            pg.goto(url)
            pg.wait_for_function("document.getElementById('conn')?.classList.contains('live')", timeout=60000)
            pg.evaluate("document.fonts.ready")
            shot = lambda: pg.screenshot(type="jpeg", quality=90)
            settle = "() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)))"
            wait_t = """t => { const m = /t = (-?[\\d.]+)/.exec(document.getElementById('simT')?.textContent || '');
                               return m && Math.abs(+m[1] - t) < 0.06; }"""
            t0, missed = time.time(), 0
            for n, i in enumerate(idx):
                with open(files[i], "rb") as f:
                    raw = f.read()
                write_atomic(replay_state, raw)
                try:
                    pg.wait_for_function(wait_t, arg=frame_t(raw), timeout=4000, polling=30)
                except Exception:                     # noqa: BLE001  (dashboard skipped a poll: keep going)
                    missed += 1
                for r in reps:
                    pg.evaluate(settle)
                    emit(shot(), r)
                if n % 50 == 0:
                    el = time.time() - t0
                    print(f"sample {n}/{len(idx)} t={frame_t(raw):.0f} s  {el / (n + 1):.2f} s/sample  eta {el / (n + 1) * (len(idx) - n):.0f} s", flush=True)
            if missed:
                print(f"warning: {missed} samples were captured before the dashboard showed them", flush=True)

            def realtime(seconds):          # capture at wall-clock pace so animations play at true speed
                n, base, start = round(seconds * a.fps), written, time.time()
                while written - base < n:
                    img = shot()
                    due = round((time.time() - start) * a.fps) - (written - base)
                    emit(img, max(1, min(due, n - (written - base))))

            realtime(a.hold)
            btn = pg.locator("#cands [data-id]").first
            if btn.count():
                print("opening ping detail:", btn.get_attribute("data-id"), flush=True)
                btn.click()
                try:
                    pg.wait_for_function("!document.getElementById('detail').hidden && "
                                         "(document.querySelector('#v3d canvas') || document.querySelector('#v3d .v3dmsg'))",
                                         timeout=30000)
                except Exception:                     # noqa: BLE001
                    print("warning: LiDAR view did not appear within 30 s", flush=True)
                pg.wait_for_timeout(800)              # let the cloud upload / first orbit frame draw
                realtime(a.detail)
            else:
                print("warning: no ping in the side list; skipping the detail view", flush=True)
        finally:
            b.close()
    ff.stdin.close()
    ff.wait()
    print(f"wrote {a.out}: {written} frames, {written / a.fps:.1f} s at {a.fps} fps", flush=True)
    return 0 if ff.returncode == 0 else ff.returncode


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames-dir", default=os.path.join(ROOT, "dashboard", "data", "frames"))
    ap.add_argument("--src-dir", default=os.path.join(ROOT, "dashboard", "data"), help="where the run's pointcloud.bin is")
    ap.add_argument("--replay-dir", default=os.path.join(ROOT, "dashboard", "data", "replay"))
    ap.add_argument("--url", default="http://localhost:8765/dashboard/")
    ap.add_argument("--out", default=os.path.join(ROOT, "media", "larpverse-dashboard-seed7.mp4"))
    ap.add_argument("--live", action="store_true", help="no browser: pace the frames into the replay folder")
    ap.add_argument("--speed", type=float, default=6.0, help="--live: mission seconds per wall second")
    ap.add_argument("--step", type=int, default=2, help="record: show every Nth mission frame")
    ap.add_argument("--per-sec", type=float, default=3.0, help="record: mission samples per video second")
    ap.add_argument("--shots", type=int, default=2, help="record: screenshots per sample (rest are repeats)")
    ap.add_argument("--hold", type=float, default=2.0, help="record: seconds on the final state before the detail")
    ap.add_argument("--detail", type=float, default=5.0, help="record: seconds of the first ping's LiDAR detail")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    a = ap.parse_args()
    a.replay_dir = os.path.abspath(a.replay_dir)
    if not a.replay_dir.startswith(os.path.join(ROOT, "dashboard") + os.sep):
        sys.exit("--replay-dir must be inside dashboard/ (the dashboard only reads ?data= folders under it)")
    if a.step < 1 or a.per_sec <= 0 or a.speed <= 0:
        sys.exit("--step, --per-sec and --speed must be positive")
    files = load_frames(a.frames_dir)
    return live(a, files) if a.live else record(a, files)


if __name__ == "__main__":
    sys.exit(main())
