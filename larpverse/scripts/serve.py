"""Local ops server for the demo: serves the repo (dashboard, animations) and accepts operator tasking.

    python3 scripts/serve.py            # http://127.0.0.1:8765/dashboard/

GET  /...               static files from the repo root (no-cache, so polling always sees fresh state)
GET  /api/tasking       current operator tasking (dashboard/data/tasking.json) or {"zones": []}
POST /api/tasking       replace the tasking with a JSON body {"zones": [...], ...}; validated, written atomically
GET  /api/broadcast     operator loudspeaker queue (dashboard/data/broadcast_queue.json; contract v1.3)
POST /api/broadcast     {drone_id | zone, kind: operator|callout|reassure, text, src_lang, langs, translate, operator}
                        -> operator text queued; machine translations (Gemini, optional) stay pending_approval
POST /api/broadcast/approve  {id, operator, approve: true|false} -> release or reject one pending item
POST /api/portal        captive portal (phones; contract v1.4): {device, status, lang, message, battery_pct, people,
                        location, family} -> validated, rate-limited per address (salted hash, in memory only),
                        stored in dashboard/data/portal.json without any IP
GET  /api/portal        (operator, localhost only) {"submissions": [{id, t, device, status, text, lang, translation,
                        by, battery_pct}]}
GET  /api/portal/status phone: {"internet": bool} (family messages shown only when the van has internet)
GET  /api/portal/inbox?device=<token>  phone: vetted replies queued for that phone only
POST /api/comms/reply   (operator, localhost only) {device, template | text, lang, confirmed_by}: templates directly,
                        free text only with confirmed_by -> dashboard/data/comms_replies.json
GET  /generate_204, /hotspot-detect.html, ...  captive-portal probes -> 302 to /dashboard/portal/
POST /api/live          the take's unscripted change from any browser (isaac/hud/live_button.html):
                        {cmd: activate|deactivate|toggle, id: <dummy id or hidden1/hidden2>} -> dashboard/data/live_cmd.json,
                        the same file `python3 isaac/live.py --cmd ...` writes; the running take applies it within 1 s.
                        (Hotkeys typed into the WebRTC viewer don't reach Kit's carb input on the headless box.)
Stdlib only. Binds to 127.0.0.1 by default; --lan binds 0.0.0.0 for the live phone demo (prints a warning).
"""
import argparse
import json
import os
import sys
import time
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TASKING = os.path.join(ROOT, "dashboard", "data", "tasking.json")
BROADCAST_Q = os.path.join(ROOT, "dashboard", "data", "broadcast_queue.json")
PORTAL = os.path.join(ROOT, "dashboard", "data", "portal.json")
LOCAL = ("127.0.0.1", "::1", "::ffff:127.0.0.1")
CAPTIVE_PROBES = ("/generate_204", "/gen_204", "/hotspot-detect.html", "/library/test/success.html",
                  "/connecttest.txt", "/ncsi.txt", "/canonical.html", "/success.txt")
_portal_store = None
if ROOT not in sys.path:                     # sim.guidance (stdlib-only parts) for the broadcast API
    sys.path.insert(0, ROOT)
MAX_BODY = 512 * 1024
MAX_ZONES, MAX_VERTS = 64, 400


def _polygon(poly, what):
    if not isinstance(poly, list) or not (3 <= len(poly) <= MAX_VERTS):
        raise ValueError(f"{what}: polygon needs 3..{MAX_VERTS} [x, y] points (site metres)")
    pts = []
    for p in poly:
        if not (isinstance(p, (list, tuple)) and len(p) == 2 and all(isinstance(v, (int, float)) for v in p)):
            raise ValueError(f"{what}: each point must be [x, y] numbers")
        if any(abs(v) > 5000 for v in p):
            raise ValueError(f"{what}: point outside ±5 km of the site origin")
        pts.append([round(float(p[0]), 2), round(float(p[1]), 2)])
    return pts


def validate(doc):
    if not isinstance(doc, dict) or not isinstance(doc.get("zones"), list):
        raise ValueError("body must be an object with a 'zones' list")
    if len(doc["zones"]) > MAX_ZONES:
        raise ValueError(f"at most {MAX_ZONES} zones")
    out = []
    for i, z in enumerate(doc["zones"]):
        pts = _polygon(z.get("polygon") if isinstance(z, dict) else None, f"zone {i}")
        zid = str(z.get("id") or f"Z{i + 1}")[:24]
        out.append({"id": zid, "name": str(z.get("name") or zid)[:60], "polygon": pts,
                    "priority": z.get("priority") if isinstance(z.get("priority"), (int, float)) else None,
                    "status": str(z.get("status") or "tasked")[:32], "note": str(z.get("note") or "")[:200]})
    res = {"schema": "larpverse.tasking/v1", "zones": out, "source": str(doc.get("source") or "operator")[:40],
           "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S"), "revision": int(doc.get("revision") or 0) + 1}
    if doc.get("aoi") is not None:            # optional recon area of interest (contract v1.2): one polygon
        res["aoi"] = _polygon(doc["aoi"], "aoi")
    return res


def portal_store():
    """One PortalStore per process: its rate-limit salt is random per process and never written to disk."""
    global _portal_store
    if _portal_store is None:
        from sim.comms import PortalStore                # stdlib-only part of sim.comms (numpy optional)
        _portal_store = PortalStore(PORTAL, use_llm=os.environ.get("LARPVERSE_PORTAL_LLM") == "1")
    return _portal_store


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=ROOT, **kw)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def _json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _local(self):
        return self.client_address[0] in LOCAL

    def _portal_get(self, route):
        from urllib.parse import parse_qs, urlsplit
        st = portal_store()
        if route == "/api/portal/status":
            return self._json(*st.status())
        if route == "/api/portal/inbox":               # cheap read of this token's replies only
            return self._json(*st.inbox((parse_qs(urlsplit(self.path).query).get("device") or [""])[0]))
        if not self._local():                            # messages are for the operator only
            return self._json(HTTPStatus.FORBIDDEN, {"error": "operator endpoint (localhost only)"})
        return self._json(*st.dashboard())

    def _body(self, limit):
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0 or n > limit:
            raise ValueError("empty or too large body")
        return json.loads(self.rfile.read(n))

    def do_GET(self):
        route = self.path.split("?")[0]
        if route in ("/api/portal", "/api/portal/status", "/api/portal/inbox"):
            return self._portal_get(route)
        if route in CAPTIVE_PROBES:                      # phones' connectivity checks -> show the portal
            self.send_response(HTTPStatus.FOUND)
            self.send_header("Location", "/dashboard/portal/")
            self.send_header("Content-Length", "0")
            self.end_headers()
            return None
        if self.path.split("?")[0] == "/api/broadcast":
            from sim.guidance import queue_load          # stdlib-only part of sim.guidance (numpy optional)
            return self._json(HTTPStatus.OK, queue_load(BROADCAST_Q))
        if self.path.split("?")[0] == "/api/tasking":
            try:
                with open(TASKING) as f:
                    return self._json(HTTPStatus.OK, json.load(f))
            except FileNotFoundError:
                return self._json(HTTPStatus.OK, {"schema": "larpverse.tasking/v1", "zones": []})
        return super().do_GET()

    def do_POST(self):
        route = self.path.split("?")[0]
        if route in ("/api/broadcast", "/api/broadcast/approve"):
            return self._broadcast(route)
        if route in ("/api/portal", "/api/comms/reply"):
            return self._portal_post(route)
        if route == "/api/live":
            return self._live_cmd()
        if route != "/api/tasking":
            return self._json(HTTPStatus.NOT_FOUND, {"error": "unknown endpoint"})
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0 or n > MAX_BODY:
            return self._json(HTTPStatus.BAD_REQUEST, {"error": "empty or too large body"})
        try:
            doc = validate(json.loads(self.rfile.read(n)))
        except (ValueError, json.JSONDecodeError) as e:
            return self._json(HTTPStatus.BAD_REQUEST, {"error": str(e)})
        os.makedirs(os.path.dirname(TASKING), exist_ok=True)
        tmp = TASKING + ".tmp"
        with open(tmp, "w") as f:
            json.dump(doc, f, indent=1)
        os.replace(tmp, TASKING)
        return self._json(HTTPStatus.OK, doc)

    def _live_cmd(self):
        """The take's live change (isaac/live.py write_cmd format): a person presses a button, nothing is scheduled."""
        import re
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0 or n > 4096:
            return self._json(HTTPStatus.BAD_REQUEST, {"error": "empty or too large body"})
        try:
            body = json.loads(self.rfile.read(n))
        except json.JSONDecodeError as e:
            return self._json(HTTPStatus.BAD_REQUEST, {"error": str(e)})
        cmd, did = body.get("cmd"), body.get("id")
        if cmd not in ("activate", "deactivate", "toggle") or not isinstance(did, str) or not re.fullmatch(r"[A-Za-z0-9_]{1,40}", did):
            return self._json(HTTPStatus.BAD_REQUEST, {"error": "cmd must be activate|deactivate|toggle and id a dummy id"})
        path = os.path.join(os.path.dirname(TASKING), "live_cmd.json")
        try:
            with open(path) as f:
                seq = int(json.load(f).get("seq", 0)) + 1
        except (OSError, ValueError):
            seq = 1
        doc = {"seq": seq, "cmd": cmd, "id": did, "wall": time.time(), "by": "live_button"}
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path + ".tmp", "w") as f:
            json.dump(doc, f)
        os.replace(path + ".tmp", path)
        return self._json(HTTPStatus.OK, doc)

    def _broadcast(self, route):
        """Operator loudspeaker API (contract v1.3). Nothing machine-translated is ever playable before an
        operator approves it; the mission (sim/callout.Speaker.ingest_operator) plays only released items."""
        from sim.guidance import BroadcastError, queue_decide, queue_post
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0 or n > 16 * 1024:
            return self._json(HTTPStatus.BAD_REQUEST, {"error": "empty or too large body"})
        try:
            body = json.loads(self.rfile.read(n))
            if route == "/api/broadcast":
                out = queue_post(BROADCAST_Q, body)        # may call Gemini (agents.llm: .env key, cache, backoff)
            else:
                out = queue_decide(BROADCAST_Q, str(body.get("id") or ""), str(body.get("operator") or "")[:40],
                                   approve=bool(body.get("approve", True)))
        except (BroadcastError, ValueError, json.JSONDecodeError) as e:
            return self._json(HTTPStatus.BAD_REQUEST, {"error": str(e)})
        return self._json(HTTPStatus.OK, out)

    def _portal_post(self, route):
        """Captive portal (phones) and operator replies (contract v1.4). Validation, size limits and the rate
        limit live in sim.comms (PortalStore / validate_portal / validate_reply); no IP is ever stored."""
        from sim.comms import MAX_BODY as PORTAL_MAX
        st = portal_store()
        if route == "/api/comms/reply" and not self._local():
            return self._json(HTTPStatus.FORBIDDEN, {"error": "operator endpoint (localhost only)"})
        try:
            body = self._body(PORTAL_MAX if route == "/api/portal" else 16 * 1024)
        except (ValueError, json.JSONDecodeError) as e:
            return self._json(HTTPStatus.BAD_REQUEST, {"error": str(e)[:200]})
        if route == "/api/portal":
            return self._json(*st.submit(body, self.client_address[0]))
        return self._json(*st.reply(body))

    def log_message(self, fmt, *args):  # quiet: only API calls and errors
        if "/api/" in (self.path or "") or (args and str(args[1]).startswith(("4", "5"))):
            super().log_message(fmt, *args)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--lan", action="store_true",
                    help="bind 0.0.0.0 so phones on the demo hotspot can open /dashboard/portal/ (OFF by default)")
    a = ap.parse_args()
    if a.lan:
        a.host = "0.0.0.0"
        print("WARNING: --lan binds 0.0.0.0: every device on this network can read the repo's static files and POST "
              "to /api/portal and /api/tasking. Use only on the closed demo hotspot; operator endpoints "
              "(GET /api/portal, /api/comms/reply) stay localhost-only.", flush=True)
    print(f"Larpverse ops server: http://{a.host}:{a.port}/dashboard/  "
          "(APIs: /api/tasking, /api/broadcast, /api/portal, /api/comms/reply; phones: /dashboard/portal/)")
    ThreadingHTTPServer((a.host, a.port), Handler).serve_forever()
