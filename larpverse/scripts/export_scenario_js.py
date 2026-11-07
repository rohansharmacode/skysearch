"""Export a seeded rubble-grid scenario (sim/scenario.py) as a small JS file for the concept animation.

    .venv/bin/python scripts/export_scenario_js.py                  # seed 7 -> animation/scenario_seed7.js
    .venv/bin/python scripts/export_scenario_js.py --seed 3 --out /tmp/s3.js
    .venv/bin/python scripts/export_scenario_js.py --site weiguan   # -> animation/scenario_weiguan.js

The file sets ``window.LARP_SCENARIO``. Everything is already converted to the three.js frame the
animation uses: x = east, y = up, z = -north (so the staging area, site y = -47, sits at z = +47).

Weiguan site (sim/site_weiguan.py): same layout plus ``site``, ``bounds`` and ``site_info`` (road, blocks
before the collapse, three frame); the terrain is still ``Scenario.ground_height`` (nine sector mounds +
noise, same formula), and every piece is a box as before. Victims carry ``y`` (absolute height, three
frame), voices carry ``lang``.

  site (x, y, z)      ->  three (x, z, -y)            basis change S = [[1,0,0],[0,0,1],[0,-1,0]], det +1
  piece rotation R    ->  quaternion of S @ R          (R = Rz(yaw) Ry(pitch) Rx(roll), sim.scenario.rot_matrix)
                          applied to BoxGeometry(size[0], size[1], size[2]) whose local axes = the piece's site axes
  pile rot (rad)      ->  -rot                         (mound: u = dx cos + dz sin must equal dx cos + dy sin)
  lump (dx, dy)       ->  (dx, -dy)

Terrain: the animation ports ``Scenario.ground_height`` exactly, so it only needs the noise params
(``noise.a/kx/ky/ph``, used with site y = -z). A coarse height grid is included for cross-checking.

The ``story`` block is STORYBOARD TRUTH (display only): start pile, sound roles, sound positions,
transcripts, victim positions/depths and breathing states (``victims[].physio``, drives the dig-order beat). It exists so the video can show what the sensors are meant
to find. The planner, agents and dashboard never read this file.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, ROOT)
from sim.scenario import TRANSCRIPTS, make_scenario, rot_matrix  # noqa: E402

S = np.array([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])   # site -> three


def r(v, n=3):
    return [round(float(x), n) for x in v]


def quat(M):
    """(x, y, z, w) of a proper rotation matrix (Shepperd's method)."""
    t = np.trace(M)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        q = [(M[2, 1] - M[1, 2]) / s, (M[0, 2] - M[2, 0]) / s, (M[1, 0] - M[0, 1]) / s, 0.25 * s]
    elif M[0, 0] > M[1, 1] and M[0, 0] > M[2, 2]:
        s = np.sqrt(1.0 + M[0, 0] - M[1, 1] - M[2, 2]) * 2
        q = [0.25 * s, (M[0, 1] + M[1, 0]) / s, (M[0, 2] + M[2, 0]) / s, (M[2, 1] - M[1, 2]) / s]
    elif M[1, 1] > M[2, 2]:
        s = np.sqrt(1.0 + M[1, 1] - M[0, 0] - M[2, 2]) * 2
        q = [(M[0, 1] + M[1, 0]) / s, 0.25 * s, (M[1, 2] + M[2, 1]) / s, (M[0, 2] - M[2, 0]) / s]
    else:
        s = np.sqrt(1.0 + M[2, 2] - M[0, 0] - M[1, 1]) * 2
        q = [(M[0, 2] + M[2, 0]) / s, (M[1, 2] + M[2, 1]) / s, 0.25 * s, (M[1, 0] - M[0, 1]) / s]
    q = np.array(q)
    return q / np.linalg.norm(q)


def to3(x, y, z=0.0):
    return [x, z, -y]


def build(seed: int, start_pile: str | None = None, site: str = "grid") -> dict:
    sc = make_scenario(seed, start_pile, site=site)
    pub = sc.public_dict()
    piles = []
    for p in sc.piles:
        pieces = []
        for pc in p.pieces:
            M = S @ rot_matrix(pc.rpy_deg)
            pieces.append({"k": pc.kind, "c": r(to3(*pc.center)), "s": r(pc.size), "q": r(quat(M), 5),
                           "m": pc.material, "rpy": r(pc.rpy_deg, 2)})
        piles.append({"name": p.name, "x": round(p.x, 4), "z": round(-p.y, 4), "h": round(p.h, 4),
                      "sx": round(p.sx, 4), "sz": round(p.sy, 4), "rot": round(-p.rot, 5),
                      "lx": round(p.lump[0], 4), "lz": round(-p.lump[1], 4), "pieces": pieces})
    a, kx, ky, ph = sc._noise
    # coarse bare-terrain grid (three frame), for cross-checking the JS port of ground_height
    bx0, by0, bx1, by1 = pub["bounds"]
    xs = np.arange(bx0, bx1 + 0.01, 2.0)
    zs = np.arange(-by1, -by0 + 0.01, 2.0)
    X, Z = np.meshgrid(xs, zs)
    hg = sc.ground_height(X, -Z)
    t = sc.truth
    story = {
        "_note": "STORYBOARD TRUTH (display only). Hidden ground truth from sim/scenario.py, shown in the video "
                 "so viewers see what the sensors should find. Never read by planner, agents or dashboard.",
        "start_pile": t["start_pile"],
        "roles": t["roles"],
        "sounds": [{"pile": s["pile"], "kind": s["kind"], "x": round(s["xy"][0], 3), "z": round(-s["xy"][1], 3),
                    "level_db": round(s["level_db"], 2), **({"text": s["text"]} if "text" in s else {}),
                    **{k: s[k] for k in ("lang", "text_en", "romanised") if k in s}}
                   for s in t["sounds"]],
        "victims": [{"id": v["id"], "pile": v["pile"], "x": round(v["xy"][0], 3), "z": round(-v["xy"][1], 3),
                     "depth": round(v["depth"], 3), "sound": v["sound"],
                     **({"y": round(v["z"], 3), "depth_class": v["depth_class"]} if "z" in v else {}),
                     **({"lang": v["lang"]} if "lang" in v else {}),
                     **({"physio": {k: (round(x, 3) if isinstance(x, float) else x) for k, x in v["physio"].items()}}
                        if "physio" in v else {})} for v in t["victims"]],
        "transcripts": TRANSCRIPTS,
    }
    extra = {}
    if site != "grid":
        info = pub.get("site_info", {})
        road = info.get("road")
        extra = {"site": pub.get("site", site), "bounds": pub["bounds"], "geo": pub["geo"],
                 "site_info": {**info, **({"road_three": {"x": road["x"], "z": [-road["y"][1], -road["y"][0]]}}
                                          if road else {})}}
    return {
        "schema": "larpverse.scenario.three/v1", "seed": seed, "source": "sim/scenario.py make_scenario",
        "frame": "three.js: x east, y up, z = -north (site y). Piece q = quaternion of S @ R(rpy), size along local box axes.",
        "spacing": pub["spacing"], "base": {"x": pub["base_xy"][0], "z": -pub["base_xy"][1]},
        "drone_pads": [{"x": x, "z": -y} for x, y in pub["drone_pads"]],
        "noise": {"a": r(a, 6), "kx": r(kx, 6), "ky": r(ky, 6), "ph": r(ph, 6),
                  "formula": "m + 0.03*sum a sin(kx x + ky y + ph) + clip(m/0.9,0,1)*sum 0.08 sin(7kx x + 7ky y + 3ph), y = -z"},
        "terrain_grid": {"x0": float(xs[0]), "z0": float(zs[0]), "res": 2.0, "nx": len(xs), "nz": len(zs),
                         "h": [round(float(v), 2) for v in hg.ravel()]},
        "piles": piles,
        "story": story,
        **extra,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--start-pile")
    ap.add_argument("--site", choices=["grid", "weiguan"], default="grid")
    ap.add_argument("--out")
    a = ap.parse_args()
    name = f"scenario_seed{a.seed}.js" if a.site == "grid" else \
        (f"scenario_{a.site}.js" if a.seed == 7 else f"scenario_{a.site}_seed{a.seed}.js")
    out = a.out or os.path.join(ROOT, "animation", name)
    d = build(a.seed, a.start_pile, a.site)
    body = json.dumps(d, separators=(",", ":"))
    with open(out, "w") as f:
        site_arg = "" if a.site == "grid" else f" --site {a.site}"
        f.write(f"// Generated by scripts/export_scenario_js.py --seed {a.seed}{site_arg} from sim/scenario.py. Do not edit.\n"
                "// Frame: three.js (x east, y up, z = -north). `story` = STORYBOARD TRUTH, display only.\n"
                f"window.LARP_SCENARIO = {body};\n")
    n = sum(len(p["pieces"]) for p in d["piles"])
    print(f"seed {a.seed}: start {d['story']['start_pile']}, {n} pieces, {os.path.getsize(out) // 1024} KB -> {out}")


if __name__ == "__main__":
    main()
