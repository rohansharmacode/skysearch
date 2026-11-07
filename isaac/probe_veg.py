"""List vegetation / exterior / vehicle / ground-material assets on the Isaac 6.0 asset server.

    cd /workspace/isaaclab && ./isaaclab.sh -p /workspace/skysearch/isaac/probe_veg.py --headless
Writes isaac/assets_veg.json.
"""
import argparse, json, os, re
from isaaclab.app import AppLauncher
ap = argparse.ArgumentParser(); AppLauncher.add_app_launcher_args(ap)
a, _ = ap.parse_known_args(); app = AppLauncher(a).app
import omni.client  # noqa: E402
NV = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/NVIDIA"
IS = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac"
roots = [f"{NV}/Assets/Vegetation", f"{NV}/Assets/ArchVis/Exterior", f"{NV}/Assets/ArchVis/Residential/Plants",
         f"{NV}/Assets/ArchVis/Commercial/Plants", f"{NV}/Assets/Vehicles", f"{IS}/Props", f"{IS}/Environments/Outdoor",
         f"{NV}/Materials/Base/Natural", f"{NV}/Materials/vMaterials_2/Ground", f"{NV}/Assets/ArchVis/Residential/Outdoor"]
out = {}


def walk(u, depth=0, acc=None):
    acc = [] if acc is None else acc
    if depth > 5 or len(acc) > 400:
        return acc
    r, entries = omni.client.list(u)
    if r != omni.client.Result.OK:
        return acc
    for e in entries:
        p = f"{u}/{e.relative_path}"
        if e.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN:
            walk(p, depth + 1, acc)
        elif re.search(r"\.(usd|usda|usdz|mdl)$", e.relative_path):
            acc.append(p)
    return acc


for r in roots:
    out[r] = walk(r)
    print("VEG", r.split("/6.0/")[-1], len(out[r]), flush=True)
json.dump(out, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets_veg.json"), "w"), indent=1)
pat = re.compile(r"tree|bush|shrub|grass|plant|hedge|palm|oak|birch|pine|car|sedan|truck|van|bench|lamp|bollard|cone|barrier|Grass|Soil|Gravel|Moss", re.I)
for r, L in out.items():
    for x in L:
        if pat.search(x.rsplit("/", 1)[-1]):
            print("HIT", x.split("/6.0/")[-1], flush=True)
app.close()
