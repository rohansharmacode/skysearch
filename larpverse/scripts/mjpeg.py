"""MJPEG view of the Isaac stream for the operator console (isaac/hud/console.html).

WebRTC needs UDP, which a private `brev port-forward` (an SSH tunnel) does not carry. The box already receives the
Isaac WebRTC stream in a Chromium on Xvfb (live2/kiosk_take.py), so this grabs that screen with ffmpeg (x11grab),
encodes JPEG frames and serves the newest one to every client as multipart/x-mixed-replace:

    GET /stream.mjpg   the live view (scripts/serve.py proxies it on its own port, so one port-forward is enough)
    GET /frame.jpg     the newest frame once

Stdlib + ffmpeg only. Binds to 127.0.0.1. Example:
    python3 scripts/mjpeg.py --display :99 --size 1920x1080 --out 1280x720 --fps 20 --port 8766
"""
import argparse
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BOUNDARY = "lvframe"
_frame, _seq = None, 0
_cond = threading.Condition()


def grab(a):
    """ffmpeg -> concatenated JPEGs on stdout; split on the SOI/EOI markers; restart if ffmpeg exits."""
    global _frame, _seq
    w, h = a.out.split("x")
    cmd = ["ffmpeg", "-nostdin", "-loglevel", "error", "-f", "x11grab", "-draw_mouse", "0", "-framerate", str(a.fps),
           "-video_size", a.size, "-i", f"{a.display}+{a.offset}",
           "-vf", f"crop={a.crop},scale={w}:{h}" if a.crop else f"scale={w}:{h}",
           "-q:v", str(a.quality), "-f", "image2pipe", "-c:v", "mjpeg", "-"]
    while True:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        buf = b""
        while True:
            chunk = p.stdout.read(65536)
            if not chunk:
                break
            buf += chunk
            while True:
                s = buf.find(b"\xff\xd8")
                e = buf.find(b"\xff\xd9", s + 2) if s >= 0 else -1
                if s < 0 or e < 0:
                    if s > 0:
                        buf = buf[s:]
                    break
                with _cond:
                    _frame, _seq = buf[s:e + 2], _seq + 1
                    _cond.notify_all()
                buf = buf[e + 2:]
        p.wait()
        time.sleep(1.0)                      # Xvfb not up yet / restarted: try again


class H(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        route = self.path.split("?")[0]
        if route == "/frame.jpg":
            f = _frame
            if f is None:
                self.send_error(503, "no frame yet")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(f)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(f)
            return
        if route != "/stream.mjpg":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last = -1
        try:
            while True:
                with _cond:
                    _cond.wait_for(lambda: _seq != last, timeout=5.0)
                    f, last = _frame, _seq
                if f is None:
                    continue
                self.wfile.write(f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(f)}\r\n\r\n".encode())
                self.wfile.write(f)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--display", default=":99")
    ap.add_argument("--size", default="1920x1080", help="x11grab capture size")
    ap.add_argument("--offset", default="0,0", help="x11grab offset x,y")
    ap.add_argument("--crop", default="", help="ffmpeg crop w:h:x:y before scaling (empty = whole capture)")
    ap.add_argument("--out", default="1280x720", help="served size")
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--quality", type=int, default=5, help="JPEG -q:v (2 best .. 31 worst)")
    ap.add_argument("--port", type=int, default=8766)
    a = ap.parse_args()
    threading.Thread(target=grab, args=(a,), daemon=True).start()
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), H)
    srv.daemon_threads = True
    print(f"[mjpeg] {a.display} {a.size} -> {a.out} @ {a.fps} fps on http://127.0.0.1:{a.port}/stream.mjpg", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
