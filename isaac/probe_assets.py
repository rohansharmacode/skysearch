"""List the real 3D assets this Isaac Sim install can reach, for the photoreal earthquake scene.

    cd /workspace/isaaclab && ./isaaclab.sh -p /workspace/skysearch/isaac/probe_assets.py --headless
Writes isaac/assets_found.json (commit it) and prints a summary. Takes 1-3 min (asset server listing).
Looks for: MDL materials (concrete, plaster, brick, carpet, wood, dirt), office furniture, people,
drone models, HDRI skies, and the Flow (smoke/fire) and Blast (destruction) extensions.
"""
import argparse, json, os, re, sys, time
from isaaclab.app import AppLauncher
ap = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(ap)
a, _ = ap.parse_known_args()
simulation_app = AppLauncher(a).app

import omni.kit.app  # noqa: E402
import omni.client  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets_found.json")
res = {"roots": {}, "found": {}, "extensions": {}, "errors": []}
try:
    from isaaclab.utils import assets as A
    for k in ("ISAAC_NUCLEUS_DIR", "NVIDIA_NUCLEUS_DIR", "ISAACLAB_NUCLEUS_DIR"):
        res["roots"][k] = getattr(A, k, None)
except Exception as e:
    res["errors"].append(f"isaaclab.utils.assets: {e}")
print("PROBE roots", res["roots"], flush=True)

T0 = time.time()


def walk(url, depth, keep, exts, cap=200, budget=60):
    """Breadth-first listing; returns file URLs whose name matches `keep` and ends with one of `exts`."""
    out, frontier, t0 = [], [(url.rstrip("/"), 0)], time.time()
    while frontier and len(out) < cap and time.time() - t0 < budget:
        u, d = frontier.pop(0)
        try:
            r, entries = omni.client.list(u)
        except Exception as e:
            res["errors"].append(f"list {u}: {e}"); continue
        if r != omni.client.Result.OK:
            res["errors"].append(f"list {u}: {r}"); continue
        for e in entries:
            p = f"{u}/{e.relative_path}"
            is_dir = bool(e.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN)
            if is_dir and d < depth and not e.relative_path.startswith((".", "_")):
                frontier.append((p, d + 1))
            elif p.lower().endswith(exts) and (keep is None or re.search(keep, p, re.I)):
                out.append(p)
    return out


R = res["roots"]
I, N, L = R.get("ISAAC_NUCLEUS_DIR"), R.get("NVIDIA_NUCLEUS_DIR"), R.get("ISAACLAB_NUCLEUS_DIR")
jobs = []
if N:
    jobs += [
        ("materials", f"{N}/Materials", 4, r"concrete|cement|plaster|brick|asphalt|carpet|wood|dirt|dust|stone|rubble|drywall|tile|rust|metal|glass|ground|gravel",
         (".mdl",)),
        ("skies", f"{N}/Assets/Skies", 3, None, (".hdr", ".exr", ".dds")),
        ("archvis_furniture", f"{N}/Assets/ArchVis", 5, r"desk|chair|table|shelf|cabinet|monitor|computer|office|lamp|sofa|plant|door|bookcase|locker",
         (".usd", ".usda", ".usdc", ".usdz")),
        ("digitaltwin", f"{N}/Assets/DigitalTwin", 4, r"rubble|debris|concrete|pallet|barrel|cone|barrier|rack|box|crate", (".usd", ".usda", ".usdc")),
        ("nvidia_assets_top", f"{N}/Assets", 1, None, (".usd",)),
    ]
if I:
    jobs += [
        ("environments", f"{I}/Environments", 3, None, (".usd", ".usda", ".usdc")),
        ("props", f"{I}/Props", 3, None, (".usd", ".usda", ".usdc")),
        ("people", f"{I}/People", 4, None, (".usd", ".usda", ".usdc")),
        ("drones", f"{I}/Robots", 3, r"crazyflie|bitcraze|quad|iris|drone|copter|cf2", (".usd", ".usda", ".usdc")),
        ("isaac_materials", f"{I}/Materials", 3, None, (".mdl", ".usd")),
    ]
if L:
    jobs += [("isaaclab", f"{L}", 3, r"crazyflie|material|concrete|Robots/Bitcraze", (".usd", ".mdl"))]

for name, url, depth, keep, exts in jobs:
    t = time.time()
    found = walk(url, depth, keep, exts)
    res["found"][name] = found
    print(f"PROBE {name}: {len(found)} files from {url} ({time.time() - t:.0f}s)", flush=True)
    for p in found[:25]:
        print("   ", p)

em = omni.kit.app.get_app().get_extension_manager()
try:
    for e in em.get_extensions():
        n = e.get("name", "") if isinstance(e, dict) else str(e)
        if re.search(r"flow|blast|destruct|fracture|people|anim\.graph|replicator\.core|particle|wind|volum", n, re.I):
            res["extensions"][n] = bool(e.get("enabled")) if isinstance(e, dict) else None
except Exception as e:
    res["errors"].append(f"extensions: {e}")
print("PROBE extensions", json.dumps(res["extensions"], indent=1), flush=True)
for ext in ("omni.flowusd", "omni.usd.schema.flow", "omni.blast", "omni.blast.core"):
    try:
        ok = em.set_extension_enabled_immediate(ext, True)
        res["extensions"][ext + " (enable)"] = bool(ok)
    except Exception as e:
        res["extensions"][ext + " (enable)"] = f"error: {e}"
print("PROBE enable", {k: v for k, v in res["extensions"].items() if k.endswith("(enable)")}, flush=True)

json.dump(res, open(OUT, "w"), indent=1)
print(f"PROBE wrote {OUT} in {time.time() - T0:.0f}s; errors: {len(res['errors'])}", flush=True)
for e in res["errors"][:15]:
    print("   err:", e)
simulation_app.close()
