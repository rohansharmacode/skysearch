"""Live mission control for the earthquake scenario, with buttons for UNSCRIPTED events.

A teammate or a judge clicks: start a fire, make a trapped person knock, trigger an aftershock,
kill a drone, hand the map to the crew. The swarm is never told in advance; it has to notice
through its own sensors and re-plan.

    python -m skysearch.usar.live --port 8090 [--story] [--seed 4] [--speed 1.5]
Open http://localhost:8090 (on Brev: port 8090, because VS Code already uses 8080).
"""
from __future__ import annotations
import argparse, io, json, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
import numpy as np

from .config import USARConfig
from .scenario import USARScenario
from .render import USARRenderer

PAGE = """<!doctype html><html><head><title>SkySearch USAR mission control</title>
<style>body{background:#0E1117;color:#eee;font-family:system-ui;margin:0}
#bar{padding:8px 14px;display:flex;gap:8px;align-items:center;flex-wrap:wrap}
button,select{background:#263238;color:#fff;border:1px solid #455A64;padding:7px 12px;border-radius:6px;cursor:pointer;font-size:14px}
button:hover{background:#37474F} button.on{background:#6A1B9A}
img{width:100%;display:block;cursor:crosshair} #hint{color:#90A4AE;font-size:13px}</style></head><body>
<div id=bar><b>Unscripted events:</b>
<button id=b_fire onclick="arm('fire')">🔥 Fire (click map)</button>
<button id=b_knock onclick="arm('knock')">✊ Knock (click near a person)</button>
<button id=b_aftershock onclick="arm('aftershock')">🌋 Aftershock (click map)</button>
<select id=d></select><button onclick="go('fail',document.getElementById('d').value)">💥 Kill drone</button>
<button onclick="go('handoff','')">📋 Hand map to crew</button>
<button onclick="go('pause','')">⏯ Pause</button>
<button onclick="go('reset','')">↺ Restart</button><span id=hint></span></div>
<img id=v src="frame.png" onclick="clk(event)">
<script>
let mode=null;
fetch('info').then(r=>r.json()).then(j=>{let s=document.getElementById('d');
 j.drones.forEach((n,i)=>{let o=document.createElement('option');o.value=i;o.text=n;s.appendChild(o)})});
function arm(m){mode=m;document.querySelectorAll('button').forEach(b=>b.classList.remove('on'));
 document.getElementById('b_'+m).classList.add('on');document.getElementById('hint').innerText='click the map'}
function go(k,a){fetch('inject?kind='+k+'&arg='+a).then(r=>r.text()).then(t=>document.getElementById('hint').innerText=t)}
function clk(e){if(!mode)return;const r=e.target.getBoundingClientRect();
 const fx=(e.clientX-r.left)/r.width, fy=(e.clientY-r.top)/r.height;
 go(mode,'f'+fx.toFixed(4)+','+fy.toFixed(4));mode=null;
 document.querySelectorAll('button').forEach(b=>b.classList.remove('on'))}
setInterval(()=>{document.getElementById('v').src='frame.png?'+Date.now()},600);
</script></body></html>"""


class USARLive:
    def __init__(self, cfg: USARConfig, port=8090, speed=1.5, render_every=2):
        self.cfg, self.port, self.speed, self.render_every = cfg, port, speed, render_every
        self.lock = threading.Lock()
        self.queue = []
        self.paused = False
        self.png = b""
        self.msg = ""
        self._busy = False
        self.reset()

    # ---- used by an external loop (isaac/earthquake_scene.py drives the stepping) ----
    def drain(self):
        """Apply queued button presses. Returns True if the scenario was restarted."""
        restarted = False
        while self.queue:
            k, a = self.queue.pop(0)
            restarted |= k == "reset"
            self.msg = self.apply(k, a)
        return restarted

    def publish_async(self, extra=None):
        """Render the dashboard from a snapshot of the scenario in a background thread."""
        if self._busy:
            return
        import pickle
        snap = pickle.loads(pickle.dumps(self.scen))
        breath, pos = list(self.r.breath), dict(self.r.pos)
        self._busy = True

        def work():
            try:
                r = USARRenderer(snap, speed_label="live · Isaac Sim") if not hasattr(self, "_r2") else self._r2
                self._r2 = r
                r.s, r.breath, r.pos, r._last_note = snap, breath, pos, snap.fleet.t
                r.rgb_img = None if extra is None else extra.get("rgb")
                buf = io.BytesIO()
                r.frame().savefig(buf, format="png", dpi=72, facecolor="#0E1117")
                self.png = buf.getvalue()
            finally:
                self._busy = False
        threading.Thread(target=work, daemon=True).start()

    def serve(self):
        httpd = ThreadingHTTPServer(("0.0.0.0", self.port), self.handler())
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        print(f"SkySearch USAR mission control on http://0.0.0.0:{self.port}", flush=True)

    def reset(self):
        self.scen = USARScenario(self.cfg, random_events=False)
        self.r = USARRenderer(self.scen, speed_label="live")

    def to_map(self, arg):
        """Figure fractions from the browser (origin top-left) -> map metres."""
        fx, fy = (float(v) for v in arg[1:].split(","))
        x0, y0, x1, y1, Wm, Hm = self.r.map_box
        x = (fx - x0) / (x1 - x0) * Wm
        y = (y1 - (1 - fy)) / (y1 - y0) * Hm
        if not (0 <= x <= Wm and 0 <= y <= Hm):
            return None
        return np.array([x, y])

    def apply(self, kind, arg):
        s, f = self.scen, self.scen.fleet
        if kind == "pause":
            self.paused = not self.paused
            return "paused" if self.paused else "running"
        if kind == "reset":
            self.reset()
            return "restarted"
        if kind in ("fire", "aftershock", "knock"):
            xy = self.to_map(arg) if arg.startswith("f") else None
            if kind != "aftershock" and xy is None:
                return "click inside the map"
            if kind == "knock":
                silent = [p for p in s.b.survivors if p["found_t"] is None]
                if not silent:
                    return "everyone is already found"
                p = min(silent, key=lambda p: np.linalg.norm(p["pos"] - xy))
                p["conscious"], p["next_sound"], p["wake_t"] = True, f.t + 0.5, None
                f.say("[event] a trapped person starts knocking (swarm not told where)")
                return f"person {p['name']} knocks"
            s.inject(kind, xy)
            return f"{kind} injected"
        if kind == "fail":
            i = int(arg)
            f.kill(i, "motor failure")
            return f"{f.drones[i].name} killed"
        if kind == "handoff":
            f.handoff()
            return "map handed to crew"
        return "unknown event"

    def loop(self):
        n = 0
        while True:
            t0 = time.time()
            with self.lock:
                while self.queue:
                    k, a = self.queue.pop(0)
                    self.msg = self.apply(k, a)
                if not self.paused:
                    self.scen.step()
                    self.r.note_radar()
                    n += 1
                if n % self.render_every == 0 or self.paused:
                    buf = io.BytesIO()
                    self.r.frame().savefig(buf, format="png", dpi=72, facecolor="#0E1117")
                    self.png = buf.getvalue()
            time.sleep(max(0.0, self.cfg.dt / self.speed - (time.time() - t0)))

    def handler(self):
        srv = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def send(self, body, ctype):
                self.send_response(200); self.send_header("Content-Type", ctype)
                self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(body)

            def do_GET(self):
                u = urlparse(self.path)
                if u.path == "/":
                    self.send(PAGE.encode(), "text/html")
                elif u.path == "/frame.png":
                    self.send(srv.png, "image/png")
                elif u.path == "/info":
                    self.send(json.dumps({"drones": [d.name for d in srv.scen.fleet.drones]}).encode(), "application/json")
                elif u.path == "/inject":
                    q = parse_qs(u.query)
                    srv.queue.append((q["kind"][0], q.get("arg", [""])[0]))
                    self.send(f"sent: {q['kind'][0]}".encode(), "text/plain")
                elif u.path == "/state":
                    f = srv.scen.fleet
                    self.send(json.dumps({"t": f.t, "found": [dict(label=x["label"], conf=x["conf"], room=x["room"])
                                                              for x in f.found],
                                          "captions": f.captions[-5:], "rooms": f.room_status}).encode(), "application/json")
                else:
                    self.send_response(404); self.end_headers()
        return H

    def run(self):
        self.serve()
        self.loop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--seed", type=int, default=4)
    ap.add_argument("--story", action="store_true", help="play the team storyboard events (fire, victim B wakes)")
    ap.add_argument("--speed", type=float, default=1.5, help="simulation speed vs real time")
    a = ap.parse_args()
    USARLive(USARConfig(seed=a.seed, story=a.story), port=a.port, speed=a.speed).run()


if __name__ == "__main__":
    main()
