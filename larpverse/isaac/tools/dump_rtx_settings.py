"""Print RTX carb settings relevant to anti-aliasing, DLSS/DLAA, denoising and ray reconstruction.

    /workspace/isaaclab/_isaac_sim/python.sh isaac/tools/dump_rtx_settings.py
Prints every key under /rtx and /rtx-transient whose name matches dlss|aa|aaop|rendermode|denois|reconstruction|realtime|rr|upscal
with its current value, then the render-mode names this Kit accepts.
"""
import re

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import carb  # noqa: E402

s = carb.settings.get_settings()
pat = re.compile(r"dlss|/aa|aaop|aa_|rendermode|denois|reconstruction|realtime|/rr|rr/|upscal|dlaa|nrd|optix", re.I)


def walk(path, node, out):
    if isinstance(node, dict):
        for k, v in node.items():
            walk(f"{path}/{k}", v, out)
    else:
        out.append((path, node))


rows = []
for root in ("/rtx", "/rtx-transient", "/rtx-defaults"):
    d = s.get_settings_dictionary(root)
    try:
        tree = d.get_dict() if hasattr(d, "get_dict") else dict(d)
    except Exception:
        tree = s.get(root)
    if isinstance(tree, dict):
        walk(root, tree, rows)
hits = [(k, v) for k, v in rows if pat.search(k)]
print(f"RTXDUMP {len(rows)} keys scanned, {len(hits)} matching", flush=True)
for k, v in sorted(hits):
    print(f"RTXDUMP {k} = {v!r}", flush=True)
print("RTXDUMP rendermode current:", s.get("/rtx/rendermode"), flush=True)
for k in ("/rtx/rendermode", "/rtx/post/aa/op", "/rtx/post/dlss/execMode", "/rtx-transient/dlssg/enabled",
          "/rtx/newDenoiser/enabled", "/rtx/rtpt/dlss/enabled", "/rtx/directLighting/sampledLighting/enabled"):
    print(f"RTXDUMP probe {k} = {s.get(k)!r}", flush=True)
try:                                   # mode names the Render Settings UI offers
    import omni.kit.app
    em = omni.kit.app.get_app().get_extension_manager()
    for ext in ("omni.rtx.settings.core", "omni.rtx.window.settings"):
        p = em.get_extension_path_by_module(ext.replace("-", "_")) if hasattr(em, "get_extension_path_by_module") else None
        print("RTXDUMP ext", ext, p, flush=True)
except Exception as e:
    print("RTXDUMP ext lookup failed", e, flush=True)
app.close()
