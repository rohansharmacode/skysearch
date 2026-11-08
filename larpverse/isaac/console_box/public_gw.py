"""Public gateway for the operator console (behind a cloudflared quick tunnel). Stdlib only, 127.0.0.1:8770.

Forwards to scripts/serve.py (127.0.0.1:8765) ONLY an allowlist:
  GET  /                                   -> 302 to the console
  GET  /isaac/hud/console.html             (and nothing else under isaac/)
  GET  /dashboard/data/{state,live,hud,site,goto,console}.json, /dashboard/data/pointcloud.bin
  GET  /stream.mjpg, /frame.jpg            (streamed through unbuffered)
  POST /api/goto   (at most one per 3 s)   POST /api/site (one per 90 s)   POST /api/reset (one per 20 s)
Everything else is 404 (dotfiles, scripts/, agents/, sim/, listings). While /tmp/recording.lock exists, public POSTs get
423 so nobody can wreck a recording; the box's own kiosk talks to 8765 directly and is unaffected.
"""
import http.client
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

UP = ("127.0.0.1", 8765)
CONSOLE = "/isaac/hud/console.html"
HOME = CONSOLE + "?state=../../dashboard/data/state.json&stream=/stream.mjpg"
GET_OK = {CONSOLE, "/stream.mjpg", "/frame.jpg", "/assets/audio/dummies/manifest.json"} | {f"/dashboard/data/{n}" for n in (
    "state.json", "live.json", "hud.json", "site.json", "goto.json", "console.json", "pointcloud.bin")}
POST_GAP = {"/api/goto": 3.0, "/api/site": 90.0, "/api/reset": 20.0}
LOCK = "/tmp/recording.lock"
_last, _mu = {}, threading.Lock()


class H(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"

    def log_message(self, fmt, *args):
        pass

    def _send(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _relay(self, method, path, body=None):
        c = http.client.HTTPConnection(*UP, timeout=30)
        hdrs = {"Content-Type": "application/json"} if body is not None else {}
        c.request(method, path, body=body, headers=hdrs)
        r = c.getresponse()
        self.send_response(r.status)
        for k in ("Content-Type", "Content-Length", "Cache-Control"):
            if r.getheader(k):
                self.send_header(k, r.getheader(k))
        self.end_headers()
        try:
            while True:
                chunk = r.read1(65536) if hasattr(r, "read1") else r.read(65536)
                if not chunk:
                    break
                self.wfile.write(chunk)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            c.close()

    def do_GET(self):
        route, _, query = self.path.partition("?")
        if route == "/":
            self.send_response(302)
            self.send_header("Location", HOME)
            self.end_headers()
            return
        if route not in GET_OK:
            return self._send(404, {"error": "not found"})
        return self._relay("GET", route + (("?" + query) if query and route == CONSOLE else ""))

    def do_POST(self):
        route = self.path.split("?")[0]
        if route not in POST_GAP:
            return self._send(404, {"error": "not found"})
        if os.path.exists(LOCK):
            return self._send(423, {"error": "A recording is running, try again in a minute."})
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0 or n > 4096:
            return self._send(400, {"error": "empty or too large body"})
        body = self.rfile.read(n)
        with _mu:
            now = time.time()
            wait = POST_GAP[route] - (now - _last.get(route, 0.0))
            if wait > 0:
                return self._send(429, {"error": f"Too fast: try again in {wait:.0f} s."})
            _last[route] = now
        return self._relay("POST", route, body)


if __name__ == "__main__":
    srv = ThreadingHTTPServer(("127.0.0.1", 8770), H)
    srv.daemon_threads = True
    print("[public_gw] 127.0.0.1:8770 -> 127.0.0.1:8765 (allowlist)", flush=True)
    srv.serve_forever()
