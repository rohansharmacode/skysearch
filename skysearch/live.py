"""Live mission-control web dashboard with buttons for UNSCRIPTED disruptions.

A teammate (or a judge) clicks: kill a drone, start a fire, move the missing person.
The swarm is never told in advance; it has to notice and re-plan.

    python -m skysearch.live --port 8080 [--speed 2]
Open http://localhost:8080 . Used by both the CPU sim and isaac/isaac_bridge.py.
"""
from __future__ import annotations
import argparse, io, json, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
import numpy as np

PAGE = """<!doctype html><html><head><title>SkySearch mission control</title>
<style>body{background:#0E1117;color:#eee;font-family:system-ui;margin:0}
#bar{padding:8px 14px;display:flex;gap:10px;align-items:center}
button{background:#263238;color:#fff;border:1px solid #455A64;padding:8px 14px;border-radius:6px;cursor:pointer;font-size:15px}
button:hover{background:#37474F} img{width:100%;display:block;cursor:crosshair}
#hint{color:#90A4AE;font-size:13px}</style></head><body>
<div id=bar><b>Unscripted events:</b>
<button onclick="mode='fire'">🔥 Start fire (then click map)</button>
<button onclick="mode='person'">🧍 Move person (then click map)</button>
<select id=d></select><button onclick="go('fail',document.getElementById('d').value)">💥 Kill drone</button>
<button onclick="go('pause',0)">⏯ Pause</button><span id=hint></span></div>
<img id=v src="frame.png" onclick="clk(event)">
<script>
let mode=null;
fetch('info').then(r=>r.json()).then(j=>{let s=document.getElementById('d');
 for(let i=0;i<j.n;i++){let o=document.createElement('option');o.value=i;o.text='D'+i;s.appendChild(o)}});
function go(k,a){fetch('inject?kind='+k+'&arg='+a);document.getElementById('hint').innerText='sent: '+k+' '+a}
function clk(e){if(!mode)return;const im=e.target,r=im.getBoundingClientRect();
 const fx=(e.clientX-r.left)/r.width, fy=(e.clientY-r.top)/r.height;
 // map axes occupy [0.01,0.53] x [0.03,0.92] of the figure (see render.py)
 const x=(fx-0.01)/0.52*1000, y=(1-fy-0.03)/0.89*1000;
 go(mode==='fire'?'fire':'move_person', x.toFixed(0)+','+y.toFixed(0)); mode=null}
setInterval(()=>{document.getElementById('v').src='frame.png?'+Date.now()},700);
</script></body></html>"""


class LiveServer:
    def __init__(self, scen, renderer, port=8080):
        self.scen, self.r, self.port = scen, renderer, port
        self.png = b""
        self.lock = threading.Lock()
        self.queue = []
        self.paused = False

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
                    with srv.lock:
                        self.send(srv.png, "image/png")
                elif u.path == "/info":
                    self.send(json.dumps({"n": srv.scen.cfg.n_drones}).encode(), "application/json")
                elif u.path == "/inject":
                    q = parse_qs(u.query)
                    srv.queue.append((q["kind"][0], q.get("arg", ["0"])[0]))
                    self.send(b"ok", "text/plain")
                else:
                    self.send_response(404); self.end_headers()
        return H

    def start(self):
        httpd = ThreadingHTTPServer(("0.0.0.0", self.port), self.handler())
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        print(f"mission control on http://0.0.0.0:{self.port}")

    def apply_injections(self):
        while self.queue:
            kind, arg = self.queue.pop(0)
            if kind == "pause":
                self.paused = not self.paused
            elif kind == "fail":
                self.scen.inject("fail", int(arg))
            elif kind in ("fire", "move_person"):
                xy = np.array([float(v) for v in arg.split(",")])
                self.scen.inject(kind, xy)
                self.scen.swarm.say(f"[unscripted] {kind.replace('_', ' ')} at ({xy[0]:.0f},{xy[1]:.0f}) (swarm not told)")

    def publish(self):
        buf = io.BytesIO()
        self.r.frame().savefig(buf, format="png", dpi=70, facecolor=self.r.fig.get_facecolor())
        with self.lock:
            self.png = buf.getvalue()


def main():
    from .config import Config
    from .scenario import Scenario
    from .render import Renderer
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--speed", type=float, default=2.0, help="sim seconds per wall second")
    ap.add_argument("--scripted", action="store_true", help="also fire the random timed events")
    a = ap.parse_args()
    cfg = Config(seed=a.seed, duration=1e9)
    scen = Scenario(cfg, random_events=a.scripted)
    live = LiveServer(scen, Renderer(scen), a.port)
    live.start()
    last_pub = 0
    while True:
        t0 = time.time()
        live.apply_injections()
        if not live.paused:
            scen.step()
        if time.time() - last_pub > 0.5:
            live.publish(); last_pub = time.time()
        time.sleep(max(0, cfg.dt / a.speed - (time.time() - t0)))


if __name__ == "__main__":
    main()
