"""Local ops server for the demo: serves the repo (dashboard, animations) and accepts operator tasking.

    python3 scripts/serve.py            # http://127.0.0.1:8765/dashboard/

GET  /...               static files from the repo root (no-cache, so polling always sees fresh state)
GET  /api/tasking       current operator tasking (dashboard/data/tasking.json) or {"zones": []}
POST /api/tasking       replace the tasking with a JSON body {"zones": [...], ...}; validated, written atomically
Stdlib only. Binds to 127.0.0.1 by default (LAN demo: --host 0.0.0.0 at your own risk).
"""
import argparse
import json
import os
import time
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TASKING = os.path.join(ROOT, "dashboard", "data", "tasking.json")
MAX_BODY = 512 * 1024
MAX_ZONES, MAX_VERTS = 64, 400


def validate(doc):
    if not isinstance(doc, dict) or not isinstance(doc.get("zones"), list):
        raise ValueError("body must be an object with a 'zones' list")
    if len(doc["zones"]) > MAX_ZONES:
        raise ValueError(f"at most {MAX_ZONES} zones")
    out = []
    for i, z in enumerate(doc["zones"]):
        poly = z.get("polygon") if isinstance(z, dict) else None
        if not isinstance(poly, list) or not (3 <= len(poly) <= MAX_VERTS):
            raise ValueError(f"zone {i}: polygon needs 3..{MAX_VERTS} [x, y] points (site metres)")
        pts = []
        for p in poly:
            if not (isinstance(p, (list, tuple)) and len(p) == 2 and all(isinstance(v, (int, float)) for v in p)):
                raise ValueError(f"zone {i}: each point must be [x, y] numbers")
            if any(abs(v) > 5000 for v in p):
                raise ValueError(f"zone {i}: point outside ±5 km of the site origin")
            pts.append([round(float(p[0]), 2), round(float(p[1]), 2)])
        zid = str(z.get("id") or f"Z{i + 1}")[:24]
        out.append({"id": zid, "name": str(z.get("name") or zid)[:60], "polygon": pts,
                    "priority": z.get("priority") if isinstance(z.get("priority"), (int, float)) else None,
                    "status": str(z.get("status") or "tasked")[:32], "note": str(z.get("note") or "")[:200]})
    return {"schema": "larpverse.tasking/v1", "zones": out, "source": str(doc.get("source") or "operator")[:40],
            "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S"), "revision": int(doc.get("revision") or 0) + 1}


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

    def do_GET(self):
        if self.path.split("?")[0] == "/api/tasking":
            try:
                with open(TASKING) as f:
                    return self._json(HTTPStatus.OK, json.load(f))
            except FileNotFoundError:
                return self._json(HTTPStatus.OK, {"schema": "larpverse.tasking/v1", "zones": []})
        return super().do_GET()

    def do_POST(self):
        if self.path.split("?")[0] != "/api/tasking":
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

    def log_message(self, fmt, *args):  # quiet: only API calls and errors
        if "/api/" in (self.path or "") or (args and str(args[1]).startswith(("4", "5"))):
            super().log_message(fmt, *args)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    a = ap.parse_args()
    print(f"Larpverse ops server: http://{a.host}:{a.port}/dashboard/  (tasking API: /api/tasking)")
    ThreadingHTTPServer((a.host, a.port), Handler).serve_forever()
