"""Town kit for the grid site's earthquake-hit neighbourhood (isaac/town.py, --town DIR), built in Blender (headless).

    pip install bpy usd-core pillow numpy scipy      # bpy 5.x wheel (Python 3.13); usd-core for the USD stage
    python blender_town_kit.py --out isaac/assets/town_kit                    # textures + modules + USD
    python blender_town_kit.py --out DIR --stage preview --preview /mnt/project-files/larpverse/blender/

Stages (like blender_facade_kit.py): build = bake the procedural Cycles materials to tileable PNG sets and build the
module geometry (numpy, cached outside the kit folder); usd = write town_kit.usdc (one prim per module, Looks per
module + /TownKit/Looks for the ground); preview = Cycles CPU renders of the real grid stage (isaac/scene.py build +
isaac/photoreal.py dress_rubble / scatter_debris, the command van, and the town placed by isaac/town.py), converted
from the pxr stage into Blender (PointInstancers -> collection instances, UsdPreviewSurface -> Principled).

Module frames (metres, Z-up, the same in Blender and USD; isaac/town.py places them):
    bldg_*    x along the frontage (W), street facade at y = 0 facing -y, building toward +y (depth D), z up from the
              street level (plinth to z = -0.6). W, D, storeys and damage come from isaac/town.py BLDG.
    car_*, truck_*, wreck_*   x forward, origin on the ground at the body centre.
    pole_*    concrete pole on z, crossarm along y at z = 7.25 (insulators at y = -0.62, 0, 0.62: town.POLE_ARMS).
    light_*   steel pole on z, the arm reaching +x.          tree_*   trunk base at the origin.
    wall_*, gate_*   3.0 / 3.6 m segment along x, centred, thickness along y.    stub_*  ground-storey wall stub.
    debris_*, sheet_*, litter_*, manhole_*, post_*   base centre at the origin.
Textures are tileable (4D torus-mapped noise): one tile = TILE[name] metres. Licence-clean: everything procedural.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, REPO)
from isaac import town as TW                                    # noqa: E402  (specs shared with the placer)

# texture sets: (pixels, tile metres)
TEX = {"asphalt": (1024, 8.0), "lot": (1024, 8.0), "bed": (512, 4.0), "plaster": (512, 4.0), "dust": (512, 6.0),
       "paving": (512, 2.0), "concrete": (512, 3.0), "brick": (512, 2.0), "tile": (512, 2.0), "roof": (512, 6.0),
       "metal": (512, 2.0), "shutter": (512, 2.0), "interior": (256, 3.0), "fracture": (512, 2.0)}
TILE = {k: v[1] for k, v in TEX.items()}
# albedo multiplier per material in the USD (UsdUVTexture scale); retune without re-baking (--stage usd)
SCALE = {"kerb": (0.95, 0.95, 0.93), "patch": (0.72, 0.72, 0.72), "far": (0.92, 0.9, 0.88),
         "dust": (0.78, 0.77, 0.76), "bed": (0.86, 0.85, 0.84), "asphalt": (1.12, 1.11, 1.10),
         "plaster": (0.66, 0.62, 0.55)}                   # untinted plaster (walls, gates, stubs); buildings override
# constant materials: rgb (linear), roughness, metallic
CONST = {
    "glass": ((0.022, 0.026, 0.03), 0.06, 0.0), "glass_dusty": ((0.05, 0.05, 0.048), 0.35, 0.0),
    "frame": ((0.42, 0.42, 0.40), 0.55, 0.0), "frame_dark": ((0.07, 0.07, 0.07), 0.5, 0.2),
    "paint": ((0.40, 0.40, 0.39), 0.42, 0.0), "paint_top": ((0.33, 0.31, 0.28), 0.8, 0.0),
    "tyre": ((0.025, 0.025, 0.025), 0.9, 0.0), "trim": ((0.04, 0.04, 0.042), 0.65, 0.0),
    "chrome": ((0.40, 0.40, 0.40), 0.35, 0.8), "lamp": ((0.55, 0.52, 0.45), 0.2, 0.0),
    "taillamp": ((0.16, 0.07, 0.04), 0.3, 0.0), "rail": ((0.06, 0.06, 0.065), 0.55, 0.6),
    "steel": ((0.30, 0.30, 0.29), 0.5, 0.7), "wire": ((0.02, 0.02, 0.02), 0.6, 0.0),
    "tank_w": ((0.45, 0.44, 0.41), 0.6, 0.0), "tank_b": ((0.035, 0.035, 0.035), 0.55, 0.0),
    "tank_s": ((0.40, 0.40, 0.39), 0.35, 0.85), "dish": ((0.48, 0.47, 0.45), 0.5, 0.0),
    "ac": ((0.46, 0.46, 0.44), 0.6, 0.0), "grille": ((0.08, 0.08, 0.08), 0.6, 0.3),
    "bark": ((0.10, 0.085, 0.07), 0.9, 0.0), "leaf": ((0.075, 0.095, 0.045), 0.75, 0.0),
    "leaf2": ((0.10, 0.11, 0.06), 0.8, 0.0), "leaf3": ((0.055, 0.075, 0.045), 0.75, 0.0),
    "sign0": ((0.22, 0.26, 0.30), 0.5, 0.0), "sign1": ((0.40, 0.37, 0.30), 0.5, 0.0),
    "sign2": ((0.12, 0.17, 0.13), 0.5, 0.0), "sign3": ((0.38, 0.30, 0.14), 0.5, 0.0),
    "sign4": ((0.30, 0.30, 0.31), 0.5, 0.0), "awning": ((0.30, 0.29, 0.24), 0.85, 0.0),
    "door": ((0.09, 0.07, 0.055), 0.6, 0.0), "paper": ((0.40, 0.39, 0.36), 0.9, 0.0),
    "tarp": ((0.06, 0.12, 0.22), 0.7, 0.0), "plastic": ((0.30, 0.31, 0.30), 0.6, 0.0),
    "wood": ((0.20, 0.14, 0.09), 0.85, 0.0), "solar": ((0.03, 0.04, 0.06), 0.15, 0.2),
    "tape": ((0.55, 0.45, 0.04), 0.5, 0.0), "post": ((0.45, 0.40, 0.10), 0.6, 0.0),
    "marking": ((0.36, 0.355, 0.33), 0.8, 0.0), "crack": ((0.035, 0.032, 0.03), 0.95, 0.0),
    "rust": ((0.15, 0.075, 0.045), 0.8, 0.3), "mattress": ((0.38, 0.36, 0.32), 0.95, 0.0),
}
GROUND_LOOKS = {"asphalt": "asphalt", "patch": "asphalt", "sidewalk": "paving", "kerb": "concrete", "lot": "lot",
                "bed": "bed", "dust": "dust", "far": "lot", "marking": None, "crack": None, "wire": None, "tape": None}
TINT = {"bldg": ("plaster",), "car": ("paint", "paint_top"), "truck": ("paint", "paint_top"),
        "wreck": ("paint", "paint_top")}                # faces town.py re-colours per prototype (module/tint part)
UV_TILE = {"asphalt": 8.0, "patch": 8.0, "sidewalk": 2.0, "kerb": 2.0, "face_kerb": 2.0, "lot": 8.0, "bed": 4.0,
           "dust": 6.0, "far": 8.0, "marking": 2.0, "crack": 1.0}


# ================================================================================================ Blender: textures
def _bpy():
    import bpy
    return bpy


class N:
    """Shader node helper with tileable 4D (torus) coordinates on a UV [0,1]^2 plane."""

    def __init__(self, mat, tw=1.0, th=1.0):
        self.t = mat.node_tree
        self.t.nodes.clear()
        self.tw, self.th = tw, th
        tc = self.n("ShaderNodeTexCoord")
        sep = self.n("ShaderNodeSeparateXYZ")
        self.t.links.new(tc.outputs["UV"], sep.inputs[0])
        self.u, self.v = sep.outputs["X"], sep.outputs["Y"]

    def n(self, kind, **kw):
        nd = self.t.nodes.new(kind)
        for k, v in kw.items():
            setattr(nd, k, v)
        return nd

    def _in(self, sock, v):
        if hasattr(v, "is_output") or (hasattr(v, "node") and hasattr(v, "identifier")):
            self.t.links.new(v, sock)
        elif isinstance(v, (tuple, list)):
            sock.default_value = tuple(v) + ((1.0,) if len(v) == 3 and len(sock.default_value) == 4 else ())
        else:
            sock.default_value = v

    def m(self, op, a, b=None, clamp=False):
        nd = self.n("ShaderNodeMath", operation=op, use_clamp=clamp)
        self._in(nd.inputs[0], a)
        if b is not None:
            self._in(nd.inputs[1], b)
        return nd.outputs[0]

    def add(self, *xs):
        out = xs[0]
        for x in xs[1:]:
            out = self.m("ADD", out, x)
        return out

    def mul(self, a, b):
        return self.m("MULTIPLY", a, b)

    def mix(self, f, a, b, blend="MIX"):
        nd = self.n("ShaderNodeMix", data_type="RGBA", blend_type=blend, clamp_result=True)
        ins = {s.identifier: s for s in nd.inputs}
        self._in(ins["Factor_Float"], f)
        self._in(ins["A_Color"], a)
        self._in(ins["B_Color"], b)
        return {s.identifier: s for s in nd.outputs}["Result_Color"]

    def mixf(self, f, a, b):
        nd = self.n("ShaderNodeMix", data_type="FLOAT", clamp_factor=True)
        ins = {s.identifier: s for s in nd.inputs}
        self._in(ins["Factor_Float"], f)
        self._in(ins["A_Float"], a)
        self._in(ins["B_Float"], b)
        return {s.identifier: s for s in nd.outputs}["Result_Float"]

    def band(self, f, lo, hi):
        nd = self.n("ShaderNodeMapRange", interpolation_type="SMOOTHSTEP", clamp=True)
        self._in(nd.inputs["Value"], f)
        nd.inputs[1].default_value = lo
        nd.inputs[2].default_value = hi
        return nd.outputs["Result"]

    def ramp(self, f, stops, interp="LINEAR"):
        nd = self.n("ShaderNodeValToRGB")
        cr = nd.color_ramp
        cr.interpolation = interp
        while len(cr.elements) < len(stops):
            cr.elements.new(0.5)
        for el, (p, c) in zip(cr.elements, stops):
            el.position = p
            el.color = tuple(c) + (1.0,) if len(c) == 3 else tuple(c)
        self._in(nd.inputs[0], f)
        return nd.outputs[0]

    def torus(self, su=1.0, sv=1.0):
        """(vector, w) on the flat 4D torus: 1 unit = 1 m (times su across u, sv across v): seamless tiling."""
        ru, rv = self.tw * su / (2 * math.pi), self.th * sv / (2 * math.pi)
        au = self.m("MULTIPLY", self.u, 2 * math.pi)
        av = self.m("MULTIPLY", self.v, 2 * math.pi)
        cx = self.n("ShaderNodeCombineXYZ")
        self._in(cx.inputs[0], self.m("MULTIPLY", self.m("COSINE", au), ru))
        self._in(cx.inputs[1], self.m("MULTIPLY", self.m("SINE", au), ru))
        self._in(cx.inputs[2], self.m("MULTIPLY", self.m("COSINE", av), rv))
        return cx.outputs[0], self.m("MULTIPLY", self.m("SINE", av), rv)

    def lin(self):
        """Linear metres (u * tile_w, v * tile_h, 0): for patterns that repeat an integer number of times."""
        cx = self.n("ShaderNodeCombineXYZ")
        self._in(cx.inputs[0], self.m("MULTIPLY", self.u, self.tw))
        self._in(cx.inputs[1], self.m("MULTIPLY", self.v, self.th))
        return cx.outputs[0]

    def noise(self, tc, scale, detail=4.0, rough=0.55, dist=0.0, seed=0.0):
        vec, w = tc
        nd = self.n("ShaderNodeTexNoise", noise_dimensions="4D")
        self._in(nd.inputs["Vector"], vec)
        self._in(nd.inputs["W"], self.m("ADD", w, seed * 17.31) if seed else w)
        nd.inputs["Scale"].default_value = scale
        nd.inputs["Detail"].default_value = detail
        nd.inputs["Roughness"].default_value = rough
        nd.inputs["Distortion"].default_value = dist
        return nd.outputs[0], nd.outputs[1]

    def vor(self, tc, scale, feature="F1", rnd=1.0, seed=0.0):
        vec, w = tc
        nd = self.n("ShaderNodeTexVoronoi", voronoi_dimensions="4D", feature=feature)
        self._in(nd.inputs["Vector"], vec)
        self._in(nd.inputs["W"], self.m("ADD", w, seed * 13.7) if seed else w)
        nd.inputs["Scale"].default_value = scale
        nd.inputs["Randomness"].default_value = rnd
        return nd

    def sep(self, col):
        s = self.n("ShaderNodeSeparateColor")
        self._in(s.inputs[0], col)
        return s.outputs[0], s.outputs[1], s.outputs[2]

    def brick(self, vec, w, h, mortar, c1=(0, 0, 0), c2=(1, 1, 1), offset=0.5, smooth=0.1):
        br = self.n("ShaderNodeTexBrick", offset=offset, offset_frequency=2, squash=1.0, squash_frequency=2)
        self._in(br.inputs["Vector"], vec)
        br.inputs["Color1"].default_value = (*c1, 1)
        br.inputs["Color2"].default_value = (*c2, 1)
        br.inputs["Mortar"].default_value = (2, 2, 2, 1)
        br.inputs["Scale"].default_value = 1.0
        br.inputs["Mortar Size"].default_value = mortar
        br.inputs["Mortar Smooth"].default_value = smooth
        br.inputs["Bias"].default_value = 0.0
        br.inputs["Brick Width"].default_value = w
        br.inputs["Row Height"].default_value = h
        r = self.m("MINIMUM", self.sep(br.outputs["Color"])[0], 1.0)            # per-brick random 0..1
        return r, br.outputs[1]                                                   # mortar factor (1 in joints)


def _dust(T, tc, col, amount, seed, rgb=(0.36, 0.335, 0.30)):
    f1, _ = T.noise(tc, 0.35, 5, 0.6, seed=seed)
    f2, _ = T.noise(tc, 3.0, 3, 0.5, seed=seed + 1)
    m = T.band(T.add(T.mul(f1, 0.8), T.mul(f2, 0.3)), 0.40, 0.80)
    m = T.mul(m, amount)
    return T.mix(m, col, rgb), m


def mat_asphalt(T):
    """Aged urban asphalt, linear ~0.07-0.11: aggregate speckle, alligator cracks in patches, sealed cracks,
    darker repairs, tyre polish, a patchy dust coat."""
    tc = T.torus()
    f0, _ = T.noise(tc, 0.25, 5, 0.6, seed=1)
    base = T.ramp(f0, [(0.25, (0.050, 0.050, 0.049)), (0.55, (0.070, 0.069, 0.067)), (0.8, (0.092, 0.090, 0.086))])
    agg = T.vor(tc, 55.0, seed=2)
    stone = T.m("SUBTRACT", 1.0, T.band(agg.outputs["Distance"], 0.05, 0.22))
    sc = T.ramp(T.sep(agg.outputs["Color"])[0], [(0.0, (0.05, 0.05, 0.05)), (0.5, (0.16, 0.155, 0.145)),
                                                 (1.0, (0.24, 0.23, 0.21))])
    fa, _ = T.noise(tc, 2.0, 2, 0.5, seed=3)
    base = T.mix(T.mul(T.mul(stone, T.band(fa, 0.35, 0.7)), 0.55), base, sc)
    # alligator cracking in patches + long sealed cracks
    cr = T.vor(tc, 2.2, "DISTANCE_TO_EDGE", seed=4)
    fpatch, _ = T.noise(tc, 0.45, 3, 0.5, seed=5)
    crack = T.mul(T.m("SUBTRACT", 1.0, T.band(cr.outputs["Distance"], 0.006, 0.02)), T.band(fpatch, 0.55, 0.65))
    lc = T.vor(tc, 0.35, "DISTANCE_TO_EDGE", seed=6)
    seal = T.mul(T.m("SUBTRACT", 1.0, T.band(lc.outputs["Distance"], 0.004, 0.012)),
                 T.band(T.noise(tc, 0.6, 2, 0.5, seed=7)[0], 0.45, 0.6))
    base = T.mix(T.mul(seal, 0.9), base, (0.03, 0.03, 0.03))
    base = T.mix(T.mul(crack, 0.9), base, (0.02, 0.02, 0.02))
    # cut-and-patch repairs: rectangular, darker and smoother (an integer number of 2.0 x 1.0 m cells per tile)
    rid, rj = T.brick(T.lin(), 2.0, 1.0, 0.004, offset=0.37)
    rep = T.mul(T.m("GREATER_THAN", T.m("FRACT", T.mul(rid, 7.77)), 0.9), T.band(T.noise(tc, 0.3, 2, 0.5, seed=8)[0], 0.4, 0.5))
    base = T.mix(T.mul(rep, 0.85), base, (0.042, 0.042, 0.041))
    col, dm = _dust(T, tc, base, 0.45, 9, (0.22, 0.207, 0.185))
    fd, _ = T.noise(tc, 8.0, 3, 0.6, seed=10)
    col = T.mix(T.mul(T.band(fd, 0.55, 0.75), 0.35), col, (0.20, 0.19, 0.17))
    rough = T.add(0.84, T.mul(fa, 0.08))
    rough = T.mixf(rep, rough, 0.72)
    rough = T.mixf(dm, rough, 0.96)
    h = T.add(T.mul(stone, 0.5), T.mul(fd, 0.2))
    h = T.m("SUBTRACT", h, T.mul(crack, 1.2))
    h = T.m("SUBTRACT", h, T.mul(seal, 0.4))
    return col, rough, h, 0.006


def mat_paving(T):
    """Grey interlocking concrete pavers (0.25 x 0.125 m), sand joints, a few broken or missing, dusty."""
    lv = T.lin()
    tc = T.torus()
    r, joint = T.brick(lv, 0.25, 0.125, 0.006)
    col = T.ramp(r, [(0.0, (0.24, 0.235, 0.225)), (0.4, (0.30, 0.295, 0.28)), (0.7, (0.27, 0.265, 0.255)),
                     (1.0, (0.34, 0.33, 0.31))])
    f0, _ = T.noise(tc, 0.7, 4, 0.55, seed=21)
    col = T.mix(0.45, col, T.ramp(f0, [(0.3, (0.75, 0.75, 0.75)), (0.7, (1, 1, 1))]), "MULTIPLY")
    col = T.mix(joint, col, (0.26, 0.24, 0.21))
    fm, _ = T.noise(tc, 1.8, 2, 0.5, seed=22)
    miss = T.mul(T.m("GREATER_THAN", T.m("FRACT", T.mul(r, 9.17)), 0.93), T.band(fm, 0.55, 0.62))
    col = T.mix(miss, col, (0.12, 0.11, 0.095))
    st, _ = T.noise(tc, 1.1, 3, 0.6, seed=23)
    col = T.mix(T.mul(T.band(st, 0.6, 0.75), 0.6), col, (0.10, 0.095, 0.09))
    col, dm = _dust(T, tc, col, 0.55, 24, (0.34, 0.315, 0.28))
    rough = T.mixf(joint, 0.86, 0.95)
    rough = T.mixf(dm, rough, 0.96)
    tilt = T.mul(T.m("FRACT", T.mul(r, 3.7)), 0.3)
    h = T.add(T.m("SUBTRACT", 1.0, joint), tilt)
    h = T.m("SUBTRACT", h, T.mul(miss, 1.2))
    return col, rough, h, 0.006


def mat_lot(T):
    """Compacted dusty grey-brown yard: earth, fine gravel, scattered angular stones, broken slab remains, stains,
    dry weeds."""
    tc = T.torus()
    f0, _ = T.noise(tc, 0.3, 5, 0.6, seed=31)
    f1, _ = T.noise(tc, 1.4, 4, 0.6, seed=41)
    base = T.ramp(f0, [(0.25, (0.14, 0.128, 0.108)), (0.5, (0.205, 0.188, 0.16)), (0.78, (0.26, 0.245, 0.215))])
    base = T.mix(0.35, base, T.ramp(f1, [(0.3, (0.78, 0.78, 0.78)), (0.7, (1.08, 1.08, 1.08))]), "MULTIPLY")
    g = T.vor(tc, 28.0, seed=32)
    grav = T.m("SUBTRACT", 1.0, T.band(g.outputs["Distance"], 0.1, 0.3))
    gc = T.ramp(T.sep(g.outputs["Color"])[0], [(0.0, (0.10, 0.095, 0.09)), (0.5, (0.27, 0.26, 0.24)),
                                               (1.0, (0.36, 0.35, 0.33))])
    fg, _ = T.noise(tc, 1.5, 2, 0.5, seed=33)
    base = T.mix(T.mul(T.mul(grav, T.band(fg, 0.4, 0.7)), 0.8), base, gc)
    # scattered angular stones / concrete bits (whole voronoi cells, 5-15 cm)
    sv = T.vor(tc, 7.0, seed=42)
    se = T.vor(tc, 7.0, "DISTANCE_TO_EDGE", seed=42)
    sr, sg, sb = T.sep(sv.outputs["Color"])
    stone = T.mul(T.m("GREATER_THAN", sr, 0.86), T.band(se.outputs["Distance"], 0.02, 0.06))
    stone = T.mul(stone, T.band(T.noise(tc, 0.6, 2, 0.5, seed=43)[0], 0.35, 0.55))
    base = T.mix(stone, base, T.ramp(sg, [(0.0, (0.16, 0.155, 0.145)), (0.6, (0.33, 0.32, 0.30)),
                                         (0.85, (0.28, 0.17, 0.11)), (1.0, (0.40, 0.385, 0.36))]))
    # broken concrete slab remains (cells), cracked
    sl = T.vor(tc, 0.45, seed=34)
    fs, _ = T.noise(tc, 0.4, 3, 0.5, seed=35)
    slab = T.mul(T.band(T.sep(sl.outputs["Color"])[2], 0.55, 0.57), T.band(fs, 0.5, 0.55))
    sc_ = T.ramp(T.noise(tc, 3.0, 4, 0.6, seed=36)[0], [(0.3, (0.24, 0.235, 0.22)), (0.7, (0.33, 0.32, 0.30))])
    edge = T.vor(tc, 0.45, "DISTANCE_TO_EDGE", seed=34)
    slab = T.mul(slab, T.band(edge.outputs["Distance"], 0.01, 0.04))
    base = T.mix(slab, base, sc_)
    cr = T.vor(tc, 3.0, "DISTANCE_TO_EDGE", seed=37)
    crack = T.mul(T.m("SUBTRACT", 1.0, T.band(cr.outputs["Distance"], 0.004, 0.012)), slab)
    base = T.mix(T.mul(crack, 0.85), base, (0.05, 0.048, 0.045))
    # damp / oil stains
    fo, _ = T.noise(tc, 0.9, 3, 0.55, seed=44)
    base = T.mix(T.mul(T.band(fo, 0.66, 0.74), 0.55), base, (0.07, 0.065, 0.06))
    # dry weeds
    fw, _ = T.noise(tc, 6.0, 3, 0.6, seed=38)
    fw2, _ = T.noise(tc, 0.5, 2, 0.5, seed=39)
    weed = T.mul(T.band(fw, 0.62, 0.7), T.band(fw2, 0.55, 0.7))
    base = T.mix(T.mul(weed, 0.7), base, (0.17, 0.155, 0.085))
    col, dm = _dust(T, tc, base, 0.35, 40, (0.31, 0.29, 0.255))
    rough = T.add(0.9, T.mul(fg, 0.07))
    rough = T.mixf(slab, rough, 0.86)
    h = T.add(T.mul(grav, 0.5), T.mul(slab, 0.8), T.mul(f0, 0.6), T.mul(stone, 1.2))
    h = T.m("SUBTRACT", h, T.mul(crack, 0.8))
    return col, rough, h, 0.012


def mat_bed(T):
    """Crushed-concrete debris bed under and around the heaps: an angular mosaic of fragments (concrete, render,
    brick, tile) with dark voids between them, dust fill in places."""
    tc = T.torus()
    f0, _ = T.noise(tc, 0.5, 4, 0.6, seed=51)
    fill = T.ramp(f0, [(0.3, (0.22, 0.205, 0.185)), (0.7, (0.31, 0.29, 0.26))])
    out = fill
    hsum = T.mul(f0, 0.3)
    for k, (scale, cover) in enumerate(((3.2, 0.62), (9.0, 0.5), (22.0, 0.45))):
        v = T.vor(tc, scale, seed=52 + 3 * k)
        e = T.vor(tc, scale, "DISTANCE_TO_EDGE", seed=52 + 3 * k)
        r, g, b = T.sep(v.outputs["Color"])
        fc = T.ramp(r, [(0.0, (0.16, 0.155, 0.145)), (0.25, (0.30, 0.29, 0.27)), (0.5, (0.38, 0.365, 0.34)),
                        (0.72, (0.25, 0.24, 0.225)), (0.9, (0.44, 0.42, 0.39)), (1.0, (0.33, 0.31, 0.28))])
        fc = T.mix(T.band(g, 0.8, 0.81), fc, (0.29, 0.16, 0.105))           # brick
        fc = T.mix(T.band(g, 0.06, 0.05), fc, (0.43, 0.38, 0.31))           # beige tile / render
        fn, _ = T.noise(tc, scale * 3.0, 3, 0.6, seed=60 + k)
        fc = T.mix(0.35, fc, T.ramp(fn, [(0.3, (0.75, 0.75, 0.75)), (0.7, (1.1, 1.1, 1.1))]), "MULTIPLY")
        on = T.m("LESS_THAN", b, cover)                                      # fragment present in this cell
        body = T.mul(on, T.band(e.outputs["Distance"], 0.03, 0.09))
        gap = T.mul(on, T.m("SUBTRACT", 1.0, T.band(e.outputs["Distance"], 0.0, 0.05)))
        out = T.mix(T.mul(gap, 0.75), out, (0.06, 0.056, 0.05))
        out = T.mix(body, out, fc)
        hsum = T.add(hsum, T.mul(body, T.add(0.6, T.mul(r, 0.6)) if k == 0 else T.mul(body, 0.5 / (k + 1))))
    col, dm = _dust(T, tc, out, 0.4, 54, (0.34, 0.315, 0.28))
    rough = T.add(0.88, T.mul(f0, 0.08))
    return col, rough, hsum, 0.03


def mat_dust(T):
    """Settled demolition dust over streets: pale grey-beige, thin and thick patches, grit, tyre / foot scuffs."""
    tc = T.torus()
    f0, _ = T.noise(tc, 0.4, 5, 0.65, seed=61)
    col = T.ramp(f0, [(0.25, (0.17, 0.16, 0.145)), (0.5, (0.27, 0.255, 0.23)), (0.75, (0.35, 0.33, 0.295))])
    g = T.vor(tc, 30.0, seed=62)
    grit = T.m("SUBTRACT", 1.0, T.band(g.outputs["Distance"], 0.1, 0.3))
    fg, _ = T.noise(tc, 2.0, 2, 0.5, seed=63)
    col = T.mix(T.mul(T.mul(grit, T.band(fg, 0.5, 0.8)), 0.6), col,
                T.ramp(T.sep(g.outputs["Color"])[0], [(0, (0.12, 0.115, 0.11)), (1, (0.40, 0.38, 0.35))]))
    sc_, _ = T.noise(T.torus(1.0, 0.15), 1.0, 3, 0.6, seed=64)
    col = T.mix(T.mul(T.band(sc_, 0.6, 0.72), 0.35), col, (0.13, 0.125, 0.115))
    rough = T.add(0.93, T.mul(f0, 0.05))
    h = T.add(T.mul(grit, 0.4), T.mul(f0, 0.5))
    return col, rough, h, 0.008


def mat_plaster(T):
    """Painted cement render (near-neutral ~0.5; buildings tint it per prototype): trowel noise, grime streaks,
    repaired patches, spalls showing grey render and brick, sparse cracks, dust."""
    tc = T.torus()
    f0, _ = T.noise(tc, 0.5, 5, 0.55, seed=71)
    base = T.ramp(f0, [(0.3, (0.47, 0.47, 0.465)), (0.6, (0.53, 0.53, 0.525)), (0.85, (0.57, 0.57, 0.565))])
    ft, _ = T.noise(tc, 6.0, 4, 0.6, seed=72)
    base = T.mix(0.25, base, T.ramp(ft, [(0.3, (0.85, 0.85, 0.85)), (0.7, (1, 1, 1))]), "MULTIPLY")
    # grime streaks down the wall (along v)
    fs, _ = T.noise(T.torus(9.0, 0.5), 1.0, 5, 0.6, seed=73)
    fc, _ = T.noise(tc, 0.6, 2, 0.5, seed=74)
    st = T.mul(T.band(fs, 0.5, 0.7), T.band(fc, 0.4, 0.62))
    base = T.mix(T.mul(st, 0.5), base, (0.20, 0.19, 0.17), "MULTIPLY")
    # repaired patches (slightly different paint)
    pv = T.vor(tc, 0.6, seed=75)
    patch = T.mul(T.band(T.sep(pv.outputs["Color"])[0], 0.86, 0.88), T.band(pv.outputs["Distance"], 0.4, 0.35))
    base = T.mix(T.mul(patch, 0.6), base, (0.58, 0.575, 0.56))
    # spalls: grey render, brick behind
    fsp, _ = T.noise(tc, 1.1, 4, 0.65, dist=0.3, seed=76)
    fsp2, _ = T.noise(tc, 0.35, 2, 0.5, seed=77)
    sp = T.mul(T.band(fsp, 0.66, 0.68), T.band(fsp2, 0.5, 0.62))
    deep = T.mul(T.band(fsp, 0.71, 0.72), sp)
    base = T.mix(sp, base, (0.27, 0.265, 0.25))
    base = T.mix(deep, base, (0.27, 0.15, 0.10))
    # cracks
    cv = T.vor(tc, 0.9, "DISTANCE_TO_EDGE", seed=78)
    fcm, _ = T.noise(tc, 0.8, 2, 0.5, seed=79)
    crack = T.mul(T.m("SUBTRACT", 1.0, T.band(cv.outputs["Distance"], 0.003, 0.009)), T.band(fcm, 0.58, 0.64))
    base = T.mix(T.mul(crack, 0.85), base, (0.06, 0.055, 0.05))
    col, dm = _dust(T, tc, base, 0.25, 80, (0.40, 0.38, 0.345))
    rough = T.add(0.86, T.mul(ft, 0.08))
    h = T.add(T.mul(ft, 0.4), T.mul(f0, 0.3))
    h = T.m("SUBTRACT", h, T.mul(sp, 0.8))
    h = T.m("SUBTRACT", h, T.mul(crack, 0.6))
    return col, rough, h, 0.006


def mat_concrete(T):
    """Weathered cast concrete (frames, kerbs, poles, walls): warm grey 0.25-0.36, stains, pores, dust."""
    tc = T.torus()
    f0, _ = T.noise(tc, 0.6, 6, 0.6, seed=91)
    base = T.ramp(f0, [(0.3, (0.23, 0.222, 0.205)), (0.55, (0.30, 0.29, 0.27)), (0.75, (0.35, 0.34, 0.315))])
    f1, _ = T.noise(tc, 7.0, 4, 0.6, seed=92)
    base = T.mix(0.3, base, T.ramp(f1, [(0.3, (0.65, 0.65, 0.65)), (0.7, (1, 1, 1))]), "MULTIPLY")
    fs, _ = T.noise(T.torus(9.0, 0.4), 1.0, 4, 0.55, seed=93)
    fc, _ = T.noise(tc, 0.7, 2, 0.5, seed=94)
    st = T.mul(T.band(fs, 0.52, 0.72), T.band(fc, 0.42, 0.62))
    base = T.mix(T.mul(st, 0.5), base, (0.12, 0.115, 0.105))
    pore = T.m("SUBTRACT", 1.0, T.band(T.vor(tc, 50.0, seed=95).outputs["Distance"], 0.05, 0.16))
    pore = T.mul(pore, T.band(T.noise(tc, 3.0, 2, 0.5, seed=96)[0], 0.45, 0.65))
    base = T.mix(T.mul(pore, 0.55), base, (0.12, 0.115, 0.105))
    col, dm = _dust(T, tc, base, 0.4, 97, (0.38, 0.355, 0.32))
    rough = T.add(0.84, T.mul(f1, 0.1))
    rough = T.mixf(dm, rough, 0.96)
    h = T.add(T.mul(f1, 0.3), T.mul(f0, 0.4))
    h = T.m("SUBTRACT", h, T.mul(pore, 0.5))
    return col, rough, h, 0.008


def mat_fracture(T):
    """Fresh broken concrete: light paste, grey/brown aggregate, very rough."""
    tc = T.torus()
    f0, _ = T.noise(tc, 2.0, 5, 0.6, seed=101)
    paste = T.ramp(f0, [(0.3, (0.33, 0.315, 0.29)), (0.7, (0.42, 0.40, 0.365))])
    v = T.vor(tc, 30.0, seed=102)
    stone = T.m("SUBTRACT", 1.0, T.band(v.outputs["Distance"], 0.28, 0.40))
    sc_ = T.ramp(T.sep(v.outputs["Color"])[0], [(0.0, (0.17, 0.165, 0.16)), (0.35, (0.30, 0.29, 0.28)),
                                                (0.6, (0.27, 0.22, 0.17)), (1.0, (0.40, 0.38, 0.35))])
    base = T.mix(T.mul(stone, 0.85), paste, sc_)
    col, dm = _dust(T, tc, base, 0.4, 103, (0.38, 0.355, 0.32))
    f2, _ = T.noise(tc, 14.0, 3, 0.6, seed=104)
    h = T.add(T.mul(stone, 0.45), T.mul(f2, 0.7), T.mul(f0, 0.5))
    return col, T.add(0.9, T.mul(f2, 0.08)), h, 0.02


def mat_brick(T):
    """Hollow clay brick infill (0.25 x 0.125 m), dusty red-brown ~(0.30, 0.19, 0.13), grey mortar, chipped."""
    lv = T.lin()
    tc = T.torus()
    r, mortar = T.brick(lv, 0.25, 0.125, 0.011)
    col = T.ramp(r, [(0.0, (0.26, 0.16, 0.11)), (0.5, (0.31, 0.195, 0.135)), (1.0, (0.35, 0.225, 0.155))])
    f0, _ = T.noise(tc, 5.0, 4, 0.6, seed=111)
    col = T.mix(0.4, col, T.ramp(f0, [(0.3, (0.7, 0.7, 0.7)), (0.7, (1, 1, 1))]), "MULTIPLY")
    col = T.mix(mortar, col, (0.29, 0.28, 0.26))
    col, dm = _dust(T, tc, col, 0.45, 112, (0.36, 0.335, 0.30))
    f2, _ = T.noise(tc, 25.0, 4, 0.6, seed=113)
    h = T.add(T.m("SUBTRACT", 1.0, mortar), T.mul(f2, 0.5))
    return col, T.add(0.88, T.mul(f0, 0.08)), h, 0.008


def mat_tile(T):
    """Small beige ceramic facade tiles (0.1 x 0.05 m), light grout, grime, missing tiles, dust."""
    lv = T.lin()
    tc = T.torus()
    r, grout = T.brick(lv, 0.1, 0.05, 0.005, offset=0.0, smooth=0.15)
    col = T.ramp(r, [(0.0, (0.48, 0.40, 0.30)), (0.5, (0.54, 0.46, 0.35)), (0.8, (0.46, 0.39, 0.29)),
                     (1.0, (0.57, 0.50, 0.40))])
    col = T.mix(grout, col, (0.36, 0.34, 0.31))
    fs, _ = T.noise(T.torus(9.0, 0.6), 1.0, 5, 0.6, seed=121)
    fc, _ = T.noise(tc, 0.8, 2, 0.5, seed=122)
    st = T.mul(T.band(fs, 0.56, 0.74), T.band(fc, 0.45, 0.66))
    col = T.mix(T.mul(st, 0.5), col, (0.20, 0.18, 0.15), "MULTIPLY")
    fm, _ = T.noise(tc, 2.0, 3, 0.6, seed=123)
    miss = T.mul(T.m("GREATER_THAN", T.m("FRACT", T.mul(r, 7.31)), 0.4), T.band(fm, 0.66, 0.70))
    col = T.mix(miss, col, (0.17, 0.165, 0.155))
    col, dm = _dust(T, tc, col, 0.25, 124, (0.40, 0.38, 0.345))
    rough = T.mixf(grout, 0.35, 0.9)
    rough = T.mixf(st, rough, 0.75)
    rough = T.mixf(dm, rough, 0.9)
    h = T.m("SUBTRACT", T.m("SUBTRACT", 1.0, grout), T.mul(miss, 1.5))
    return col, rough, h, 0.003


def mat_roof(T):
    """Flat roof: cement screed with bitumen membrane patches, ponding stains, gravel, dust."""
    tc = T.torus()
    f0, _ = T.noise(tc, 0.4, 5, 0.6, seed=131)
    base = T.ramp(f0, [(0.3, (0.20, 0.195, 0.185)), (0.7, (0.29, 0.28, 0.265))])
    mv = T.vor(tc, 0.35, seed=132)
    memb = T.band(T.sep(mv.outputs["Color"])[0], 0.5, 0.52)
    base = T.mix(memb, base, T.ramp(T.noise(tc, 4.0, 3, 0.6, seed=133)[0], [(0.3, (0.05, 0.05, 0.05)),
                                                                           (0.7, (0.09, 0.088, 0.085))]))
    fp, _ = T.noise(tc, 0.7, 3, 0.5, seed=134)
    base = T.mix(T.mul(T.band(fp, 0.6, 0.7), 0.5), base, (0.12, 0.115, 0.105))
    col, dm = _dust(T, tc, base, 0.5, 135, (0.36, 0.335, 0.30))
    rough = T.mixf(memb, 0.9, 0.7)
    h = T.add(T.mul(f0, 0.4), T.mul(memb, 0.3))
    return col, rough, h, 0.008


def mat_metal(T):
    """Corrugated galvanised sheet (76 mm pitch along u): dull grey, rust blooms, streaks, dust."""
    tc = T.torus()
    f0, _ = T.noise(tc, 0.8, 4, 0.55, seed=141)
    base = T.ramp(f0, [(0.3, (0.24, 0.245, 0.25)), (0.7, (0.33, 0.335, 0.34))])
    fr, _ = T.noise(tc, 1.5, 5, 0.65, dist=0.4, seed=142)
    rust = T.band(fr, 0.6, 0.7)
    base = T.mix(rust, base, (0.17, 0.085, 0.045))
    fs, _ = T.noise(T.torus(10.0, 0.4), 1.0, 4, 0.6, seed=143)
    base = T.mix(T.mul(T.band(fs, 0.58, 0.75), 0.5), base, (0.12, 0.08, 0.06))
    col, dm = _dust(T, tc, base, 0.4, 144, (0.36, 0.335, 0.30))
    wave = T.m("SINE", T.mul(T.u, 2 * math.pi * 26.0))
    rough = T.mixf(rust, 0.55, 0.85)
    rough = T.mixf(dm, rough, 0.92)
    h = T.add(T.mul(wave, 1.0), T.mul(fr, 0.1))
    return col, rough, h, 0.012


def mat_shutter(T):
    """Roller shutter: horizontal slats (77 mm), faded grey-blue paint, grime, rust at the bottom slats, dust."""
    tc = T.torus()
    slat = T.m("FRACT", T.mul(T.v, 26.0))
    groove = T.band(slat, 0.9, 0.97)
    f0, _ = T.noise(tc, 0.7, 4, 0.55, seed=151)
    base = T.ramp(f0, [(0.3, (0.22, 0.235, 0.25)), (0.7, (0.30, 0.31, 0.32))])
    fs, _ = T.noise(T.torus(12.0, 0.4), 1.0, 4, 0.6, seed=152)
    base = T.mix(T.mul(T.band(fs, 0.55, 0.75), 0.5), base, (0.10, 0.09, 0.08))
    fr, _ = T.noise(tc, 2.0, 4, 0.6, seed=153)
    base = T.mix(T.mul(T.band(fr, 0.62, 0.72), 0.8), base, (0.16, 0.08, 0.045))
    base = T.mix(T.mul(groove, 0.6), base, (0.05, 0.05, 0.05))
    col, dm = _dust(T, tc, base, 0.35, 154, (0.36, 0.335, 0.30))
    rough = T.mixf(dm, 0.55, 0.9)
    h = T.add(T.m("SINE", T.mul(slat, math.pi)), T.mul(groove, -0.8))
    return col, rough, h, 0.008


def mat_interior(T):
    """Dark interior seen through openings: dim walls, furniture shapes, shadowed (albedo ~0.03-0.12)."""
    tc = T.torus()
    f0, _ = T.noise(tc, 0.8, 5, 0.6, seed=161)
    base = T.ramp(f0, [(0.3, (0.035, 0.033, 0.03)), (0.6, (0.075, 0.07, 0.063)), (0.85, (0.12, 0.11, 0.10))])
    v = T.vor(tc, 1.2, seed=162)
    furn = T.band(T.sep(v.outputs["Color"])[0], 0.7, 0.72)
    base = T.mix(T.mul(furn, 0.6), base, (0.02, 0.018, 0.016))
    h = T.add(T.mul(f0, 0.5), T.mul(furn, 0.3))
    return base, T.add(0.85, T.mul(f0, 0.1)), h, 0.01


MATS = {"asphalt": mat_asphalt, "lot": mat_lot, "bed": mat_bed, "plaster": mat_plaster, "dust": mat_dust,
        "paving": mat_paving, "concrete": mat_concrete, "brick": mat_brick, "tile": mat_tile, "roof": mat_roof,
        "metal": mat_metal, "shutter": mat_shutter, "interior": mat_interior, "fracture": mat_fracture}


def bake_textures(out, log=print, only=None):
    """Bake each material on a tile-sized UV plane: albedo (sRGB PNG), roughness (L), tangent normal (RGB)."""
    bpy = _bpy()
    from PIL import Image
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = 1
    sc.render.bake.margin = 0
    os.makedirs(os.path.join(out, "textures"), exist_ok=True)
    res = {}
    for name, fn in MATS.items():
        n, tile = TEX[name]
        files = {k: f"textures/{name}_{k}.png" for k in ("albedo", "rough", "normal")}
        res[name] = files
        if only and name not in only:
            continue
        me = bpy.data.meshes.new("bake_" + name)
        h = tile / 2
        me.from_pydata([(-h, -h, 0), (h, -h, 0), (h, h, 0), (-h, h, 0)], [], [(0, 1, 2, 3)])
        uv = me.uv_layers.new(name="UVMap")
        for i, (u, v) in enumerate([(0, 0), (1, 0), (1, 1), (0, 1)]):
            uv.data[i].uv = (u, v)
        ob = bpy.data.objects.new("bake_" + name, me)
        sc.collection.objects.link(ob)
        for o in sc.objects:
            o.select_set(False)
        ob.select_set(True)
        bpy.context.view_layer.objects.active = ob
        mat = bpy.data.materials.new("bake_" + name)
        mat.use_nodes = True
        T = N(mat, tile, tile)
        col, rough, hgt, bdist = fn(T)
        bump = T.n("ShaderNodeBump")
        bump.inputs["Strength"].default_value = 1.0
        bump.inputs["Distance"].default_value = bdist
        T._in(bump.inputs["Height"], hgt)
        bsdf = T.n("ShaderNodeBsdfPrincipled")
        T._in(bsdf.inputs["Normal"], bump.outputs["Normal"])
        emi = T.n("ShaderNodeEmission")
        outn = T.n("ShaderNodeOutputMaterial")
        img_node = T.n("ShaderNodeTexImage")
        me.materials.append(mat)
        for kind in ("albedo", "rough", "normal"):
            img = bpy.data.images.new(f"{name}_{kind}", n, n, alpha=False, float_buffer=True, is_data=True)
            img.colorspace_settings.name = "Non-Color"
            img_node.image = img
            T.t.nodes.active = img_node
            if kind == "normal":
                T.t.links.new(bsdf.outputs[0], outn.inputs["Surface"])
                bpy.ops.object.bake(type="NORMAL", normal_space="TANGENT", use_clear=True, margin=0)
            else:
                T._in(emi.inputs["Color"], col if kind == "albedo" else rough)
                emi.inputs["Strength"].default_value = 1.0
                T.t.links.new(emi.outputs[0], outn.inputs["Surface"])
                bpy.ops.object.bake(type="EMIT", use_clear=True, margin=0)
            A = np.empty(n * n * 4, np.float32)
            img.pixels.foreach_get(A)
            A = A.reshape(n, n, 4)[::-1, :, :3]
            if kind == "albedo":
                lin = np.clip(A, 0, 1)
                log(f"[town-kit] {name}: {n}px / {tile:g} m, albedo mean (linear) {lin.reshape(-1, 3).mean(0).round(3)}")
                A = np.where(lin <= 0.0031308, 12.92 * lin, 1.055 * np.power(lin, 1 / 2.4) - 0.055)
                im = Image.fromarray((A * 255 + 0.5).astype(np.uint8), "RGB")
            elif kind == "rough":
                im = Image.fromarray((np.clip(A[..., 0], 0, 1) * 255 + 0.5).astype(np.uint8), "L")
            else:
                im = Image.fromarray((np.clip(A, 0, 1) * 255 + 0.5).astype(np.uint8), "RGB")
            im.save(os.path.join(out, files[kind]), optimize=True)
            bpy.data.images.remove(img)
        bpy.data.objects.remove(ob)
        bpy.data.meshes.remove(me)
    return res


# ================================================================================================ geometry (numpy)
class MB:
    """Mesh builder: indexed points, triangles with per-corner normals and box-projected UVs (metres / TILE),
    one material name per triangle."""

    def __init__(self, rng=None):
        self.P, self.F, self.N, self.UV, self.M = [], [], [], [], []
        self.n = 0
        self.rng = rng or np.random.default_rng(0)
        self.uvoff = {}

    def _uvo(self, mat):
        if mat not in self.uvoff:
            self.uvoff[mat] = self.rng.uniform(0, 1, 2)
        return self.uvoff[mat]

    def tri_uv(self, p, n, mat):
        """Box projection with a right-handed tangent frame per axis (normal maps light consistently)."""
        t = TILE.get(mat, 1.0)
        a = int(np.argmax(np.abs(n)))
        s = 1.0 if n[a] >= 0 else -1.0
        if a == 2:
            u, v = p[:, 0], p[:, 1] * s
        elif a == 1:
            u, v = -p[:, 0] * s, p[:, 2]
        else:
            u, v = p[:, 1] * s, p[:, 2]
        o = self._uvo(mat)
        return np.c_[u / t + o[0], v / t + o[1]]

    def mesh(self, P, F, mat, smooth=False, mats=None):
        """Append a triangle mesh; flat or smooth (area-weighted vertex) normals."""
        P = np.asarray(P, float).reshape(-1, 3)
        F = np.asarray(F, np.int64).reshape(-1, 3)
        if not len(F):
            return self
        a, b, c = P[F[:, 0]], P[F[:, 1]], P[F[:, 2]]
        fn = np.cross(b - a, c - a)
        ln = np.linalg.norm(fn, axis=1)
        keep = ln > 1e-10
        F, fn, ln = F[keep], fn[keep], ln[keep]
        fn = fn / ln[:, None]
        if smooth:
            vn = np.zeros_like(P)
            for k in range(3):
                np.add.at(vn, F[:, k], fn * ln[:, None])
            vn /= np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-9)
            cn = vn[F]
        else:
            cn = np.repeat(fn[:, None, :], 3, 1)
        self.P.append(P)
        self.F.append(F + self.n)
        self.N.append(cn)
        mm = [mat] * len(F) if mats is None else list(np.asarray(mats)[keep])
        uv = np.zeros((len(F), 3, 2))
        for i, (f, n_, m_) in enumerate(zip(F, fn, mm)):
            uv[i] = self.tri_uv(P[f], n_, m_)
        self.UV.append(uv)
        self.M += mm
        self.n += len(P)
        return self

    def poly(self, pts, mat, smooth=False):
        pts = np.asarray(pts, float)
        F = [(0, i, i + 1) for i in range(1, len(pts) - 1)]
        return self.mesh(pts, F, mat, smooth)

    def quad(self, a, b, c, d, mat):
        return self.poly([a, b, c, d], mat)

    def box(self, x0, x1, y0, y1, z0, z1, mat, skip=(), mats=None):
        """Axis-aligned box; skip faces in ('-x','+x','-y','+y','-z','+z'); mats per face overrides."""
        v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
        faces = {"-z": (0, 3, 2, 1), "+z": (4, 5, 6, 7), "-y": (0, 1, 5, 4), "+y": (2, 3, 7, 6),
                 "-x": (3, 0, 4, 7), "+x": (1, 2, 6, 5)}
        for k, f in faces.items():
            if k in skip:
                continue
            self.quad(*[v[i] for i in f], (mats or {}).get(k, mat))
        return self

    def obox(self, c, half, R, mat, skip=()):
        """Oriented box: centre c, half sizes, rotation R (3x3)."""
        sub = MB(self.rng)
        sub.box(-half[0], half[0], -half[1], half[1], -half[2], half[2], mat, skip)
        return self.merge(sub, R, c)

    def cyl(self, c, r, h, n, mat, caps=(True, True), smooth=True, r2=None, axis=2):
        c = np.asarray(c, float)
        r2 = r if r2 is None else r2
        a = np.linspace(0, 2 * np.pi, n, endpoint=False)
        ring0 = np.c_[r * np.cos(a), r * np.sin(a), np.zeros(n)]
        ring1 = np.c_[r2 * np.cos(a), r2 * np.sin(a), np.full(n, h)]
        P = np.concatenate([ring0, ring1])
        if axis != 2:
            perm = {0: [2, 0, 1], 1: [1, 2, 0]}[axis]
            P = P[:, perm]
        P = P + c
        F = []
        for i in range(n):
            j = (i + 1) % n
            F += [(i, j, n + j), (i, n + j, n + i)]
        self.mesh(P, F, mat, smooth)
        if caps[0]:
            self.poly(P[:n][::-1], mat)
        if caps[1]:
            self.poly(P[n:], mat)
        return self

    def tube(self, pts, r, mat, sides=6, caps=False):
        pts = np.asarray(pts, float)
        rings = []
        for i, p in enumerate(pts):
            d = pts[min(i + 1, len(pts) - 1)] - pts[max(i - 1, 0)]
            d /= np.linalg.norm(d) + 1e-12
            a = np.cross(d, [0, 0, 1.0])
            if np.linalg.norm(a) < 1e-6:
                a = np.cross(d, [1.0, 0, 0])
            a /= np.linalg.norm(a)
            b = np.cross(d, a)
            t = np.linspace(0, 2 * np.pi, sides, endpoint=False)
            rings.append(p + np.outer(np.cos(t), a) * r + np.outer(np.sin(t), b) * r)
        P = np.concatenate(rings)
        F = []
        for i in range(len(pts) - 1):
            for k in range(sides):
                j = (k + 1) % sides
                F += [(i * sides + k, i * sides + j, (i + 1) * sides + j), (i * sides + k, (i + 1) * sides + j, (i + 1) * sides + k)]
        return self.mesh(P, F, mat, True)

    def prism(self, poly2d, z0, z1, mat, top=None, bottom=False, axis_map=None):
        """Extruded 2D polygon (x, y) between z0 and z1 (star-shaped, fan triangulated)."""
        poly2d = np.asarray(poly2d, float)
        n = len(poly2d)
        lo = np.c_[poly2d, np.full(n, z0)]
        hi = np.c_[poly2d, np.full(n, z1)]
        if axis_map is not None:
            lo, hi = axis_map(lo), axis_map(hi)
        for i in range(n):
            j = (i + 1) % n
            self.quad(lo[i], lo[j], hi[j], hi[i], mat)
        self.poly(hi, top or mat)
        if bottom:
            self.poly(lo[::-1], mat)
        return self

    def merge(self, other, R=None, t=None, s=None):
        if not other.F:
            return self
        P = np.concatenate(other.P)
        Nn = np.concatenate(other.N)
        if s is not None:
            P = P * np.asarray(s, float)
            Nn = Nn / np.asarray(s, float)
            Nn /= np.linalg.norm(Nn, axis=-1, keepdims=True)
        if R is not None:
            P = P @ np.asarray(R).T
            Nn = Nn @ np.asarray(R).T
        if t is not None:
            P = P + np.asarray(t, float)
        self.P.append(P)
        self.F.append(np.concatenate(other.F) + self.n)
        self.N.append(Nn)
        # re-project UVs in the new frame (keeps texel scale in metres)
        F = np.concatenate(other.F)
        M = other.M
        uv = np.zeros((len(F), 3, 2))
        for i, (f, m_) in enumerate(zip(F, M)):
            p = P[f]
            fn = np.cross(p[1] - p[0], p[2] - p[0])
            uv[i] = self.tri_uv(p, fn / (np.linalg.norm(fn) + 1e-12), m_)
        self.UV.append(uv)
        self.M += list(M)
        self.n += len(P)
        return self

    def arrays(self):
        P = np.concatenate(self.P)
        F = np.concatenate(self.F)
        Nn = np.concatenate(self.N).reshape(-1, 3)
        UV = np.concatenate(self.UV).reshape(-1, 2)
        # weld coincident points (normals / UVs stay per corner)
        key = np.round(P * 2000).astype(np.int64)
        _, idx, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
        return dict(P=P[idx].astype(np.float32), F=inv.ravel()[F].astype(np.int32), N=Nn.astype(np.float32),
                    UV=UV.astype(np.float32), M=np.array(self.M))

    def ntris(self):
        return sum(len(f) for f in self.F)


def Rz(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])


def Rx(deg):
    a = math.radians(deg)
    return np.array([[1.0, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def Ry(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1.0, 0], [-math.sin(a), 0, math.cos(a)]])


def rock_mesh(rng, sx, sy, sz, mat="fracture", n=14):
    from isaac import rubble as RB
    P, F = RB.rock(rng, sx, sy, sz, n=n)
    return MB(rng).mesh(P, F, mat)


def jag_top(rng, x0, x1, base, amp, step=0.25, smooth=0.6):
    """Ragged top line heights over x0..x1."""
    xs = np.arange(x0, x1 + 1e-6, step)
    if xs[-1] < x1 - 1e-6:
        xs = np.append(xs, x1)
    z = base + np.cumsum(rng.normal(0, amp, len(xs)))
    z = base + (z - base) * smooth
    return xs, z


def wall_panel(mb, x0, x1, z0, ztops, y0, y1, mat_face, mat_back=None, mat_core="brick", xs=None):
    """Vertical wall slab along x with a ragged top (ztops at xs), thickness y0..y1, faces -y = mat_face."""
    mat_back = mat_back or mat_face
    xs = np.linspace(x0, x1, len(ztops)) if xs is None else xs
    for i in range(len(xs) - 1):
        a, b = xs[i], xs[i + 1]
        za, zb = ztops[i], ztops[i + 1]
        mb.quad((a, y0, z0), (b, y0, z0), (b, y0, zb), (a, y0, za), mat_face)
        mb.quad((b, y1, z0), (a, y1, z0), (a, y1, za), (b, y1, zb), mat_back)
        mb.quad((a, y0, za), (b, y0, zb), (b, y1, zb), (a, y1, za), mat_core)
    mb.quad((x0, y1, z0), (x0, y0, z0), (x0, y0, ztops[0]), (x0, y1, ztops[0]), mat_core)
    mb.quad((x1, y0, z0), (x1, y1, z0), (x1, y1, ztops[-1]), (x1, y0, ztops[-1]), mat_core)
    return mb


# ------------------------------------------------------------------------------------------------ buildings
class Bldg:
    """One building variant from isaac/town.py BLDG (see the module docstring for the frame)."""

    COL, BEAM, REC = 0.42, 0.55, 0.12                  # column width, beam depth, infill recess behind the frame

    def __init__(self, rng, spec, idx):
        self.rng, self.s, self.idx = rng, spec, idx
        self.mb = MB(rng)
        self.W, self.D, self.n = spec["W"], spec["D"], spec["n"]
        self.h0 = TW.GROUND_STOREY if spec["ground"] == "shop" else TW.STOREY
        self.levels = [0.0] + [self.h0 + k * TW.STOREY for k in range(self.n)]       # floor levels, roof last
        self.nb = max(2, int(round(self.W / 3.7)))
        self.nd = max(2, int(round(self.D / 3.7)))
        self.frame = "concrete" if spec["facade"] == "brick" else "plaster"
        self.infill = {"plaster": "plaster", "tile": "tile", "brick": "brick"}[spec["facade"]]
        self.missing = set()                          # (face, bay, storey) infill panels left open (damage)
        self.cut = None                               # corner collapse: (bay index from, storey from)

    # --- facade grid (front y=0 facing -y, back y=D facing +y, sides x=+-W/2): u along the face, z up
    def face_frame(self, face):
        W, D = self.W, self.D
        if face == "front":
            return np.array([-W / 2, 0, 0]), np.array([1.0, 0, 0]), np.array([0, -1.0, 0]), W
        if face == "back":
            return np.array([W / 2, D, 0]), np.array([-1.0, 0, 0]), np.array([0, 1.0, 0]), W
        if face == "left":
            return np.array([-W / 2, D, 0]), np.array([0, -1.0, 0]), np.array([-1.0, 0, 0]), D
        return np.array([W / 2, 0, 0]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), D

    def P3(self, face, u, z, d=0.0):
        """Point on a face: u along it, z up, d outward (+) / inward (-) from the frame plane."""
        o, U, Nn, _ = self.face_frame(face)
        return o + U * u + Nn * d + np.array([0, 0, z])

    def fquad(self, face, u0, u1, z0, z1, d, mat):
        a, b = self.P3(face, u0, z0, d), self.P3(face, u1, z0, d)
        c, e = self.P3(face, u1, z1, d), self.P3(face, u0, z1, d)
        self.mb.quad(a, b, c, e, mat)

    def fbox(self, face, u0, u1, z0, z1, d0, d1, mat, skip=()):
        """Box on a face: u0..u1, z0..z1, depth d0..d1 (outward positive)."""
        o, U, Nn, _ = self.face_frame(face)
        sub = MB(self.rng)
        sub.box(u0, u1, d0, d1, z0, z1, mat, skip=skip)
        R = np.column_stack([U, Nn, [0, 0, 1.0]])           # local (u, d, z) -> world
        if np.linalg.det(R) < 0:                            # keep winding: mirror -> flip
            R2 = np.column_stack([U, -Nn, [0, 0, 1.0]])
            sub2 = MB(self.rng)
            sub2.box(u0, u1, -d1, -d0, z0, z1, mat, skip=tuple(s.replace("y", "Y") for s in skip))
            self.mb.merge(sub2, R2, o)
        else:
            self.mb.merge(sub, R, o)

    def opening(self, face, u0, u1, z0, z1, kind, depth=0.22):
        """A window / door / shop opening: frame-less reveal + glass / shutter / dark interior."""
        mb, rng = self.mb, self.rng
        d_in = -self.REC - depth
        # reveals (inner sides of the opening)
        for (a0, a1, b0, b1) in ((u0, u0, z0, z1), (u1, u1, z1, z0)):
            p = [self.P3(face, a0, b0, -self.REC), self.P3(face, a1, b1, -self.REC), self.P3(face, a1, b1, d_in),
                 self.P3(face, a0, b0, d_in)]
            mb.quad(*p, self.infill if self.infill != "tile" else "plaster")
        mb.quad(self.P3(face, u0, z0, -self.REC), self.P3(face, u0, z0, d_in), self.P3(face, u1, z0, d_in),
                self.P3(face, u1, z0, -self.REC), "concrete")
        mb.quad(self.P3(face, u1, z1, -self.REC), self.P3(face, u1, z1, d_in), self.P3(face, u0, z1, d_in),
                self.P3(face, u0, z1, -self.REC), "plaster")
        if kind == "shop":
            hz = rng.choice([0.0, 0.25, 0.55, 0.9, 1.0], p=[0.3, 0.2, 0.2, 0.15, 0.15])
            zs = z1 - (z1 - z0) * hz                                   # shutter bottom
            self.fbox(face, u0 - 0.05, u1 + 0.05, z1, z1 + 0.32, -self.REC, 0.05, "metal")     # shutter housing
            if zs < z1 - 0.05:
                bend = rng.random() < 0.25
                if bend:                                               # shutter bulged / torn by the quake
                    k = 5
                    for i in range(k):
                        ua, ub = u0 + (u1 - u0) * i / k, u0 + (u1 - u0) * (i + 1) / k
                        da = -self.REC - 0.05 + 0.18 * math.sin(math.pi * i / k)
                        db = -self.REC - 0.05 + 0.18 * math.sin(math.pi * (i + 1) / k)
                        za = zs + rng.uniform(0, 0.4) if i else zs
                        mb.quad(self.P3(face, ua, za, da), self.P3(face, ub, zs, db), self.P3(face, ub, z1, -self.REC - 0.05),
                                self.P3(face, ua, z1, -self.REC - 0.05), "shutter")
                else:
                    self.fquad(face, u0, u1, zs, z1, -self.REC - 0.06, "shutter")
            if zs > z0 + 0.05:
                self.fquad(face, u0, u1, z0, zs, d_in + 0.02, "interior")          # dark shop interior
                if rng.random() < 0.6:                                             # shop window glass
                    self.fquad(face, u0, u1, z0 + 0.4, zs, -self.REC - 0.12, "glass" if rng.random() < 0.6 else "glass_dusty")
            return
        self.fquad(face, u0, u1, z0, z1, d_in, "interior")
        if kind == "door":
            self.fquad(face, u0 + 0.05, u1 - 0.05, z0, z1 - 0.05, -self.REC - 0.1, "door")
            return
        r = rng.random()
        if r < 0.12:                                                   # blown out: dark hole
            return
        dd = -self.REC - 0.10
        fw = 0.06
        # window frame (thin boxes) + glass, sometimes a roller blind half down
        self.fbox(face, u0, u1, z0, z0 + fw, dd - 0.05, dd, "frame", skip=("-z",))
        self.fbox(face, u0, u1, z1 - fw, z1, dd - 0.05, dd, "frame", skip=("+z",))
        self.fbox(face, u0, u0 + fw, z0 + fw, z1 - fw, dd - 0.05, dd, "frame")
        self.fbox(face, u1 - fw, u1, z0 + fw, z1 - fw, dd - 0.05, dd, "frame")
        if u1 - u0 > 1.0:
            um = (u0 + u1) / 2
            self.fbox(face, um - fw / 2, um + fw / 2, z0 + fw, z1 - fw, dd - 0.05, dd, "frame")
        g = "glass" if r < 0.8 else "glass_dusty"
        self.fquad(face, u0 + fw, u1 - fw, z0 + fw, z1 - fw, dd - 0.025, g)
        if rng.random() < 0.35:                                        # external roller blind (common)
            zb = z1 - (z1 - z0) * rng.uniform(0.15, 0.9)
            self.fbox(face, u0 - 0.02, u1 + 0.02, z1, z1 + 0.18, -self.REC, 0.02, "plaster")
            self.fquad(face, u0, u1, zb, z1, -self.REC - 0.03, "shutter")

    def panel(self, face, b, k, u0, u1, kind):
        """Infill between two columns (u0..u1) for storey k with one opening of `kind` (or none)."""
        mb, rng = self.mb, self.rng
        z0 = self.levels[k] + (0.15 if k else 0.0)
        z1 = self.levels[k + 1] - self.BEAM
        if (face, b, k) in self.missing:                              # fallen-out infill: broken stumps + interior
            zt = z0 + rng.uniform(0.2, 0.8)
            xs, tops = jag_top(rng, u0, u1, zt, 0.12, 0.3)
            sub = MB(rng)
            wall_panel(sub, u0, u1, z0, tops, -self.REC - 0.2, -self.REC, self.infill, "interior", "brick", xs=xs)
            self.fquad(face, u0, u1, z0, z1, -self.REC - 2.5, "interior")
            self.fquad(face, u0, u1, z0 + 0.01, z0 + 0.01, 0, "interior")
            # floor of the exposed room
            mb.quad(self.P3(face, u0, z0 + 0.02, -self.REC), self.P3(face, u1, z0 + 0.02, -self.REC),
                    self.P3(face, u1, z0 + 0.02, -self.REC - 2.5), self.P3(face, u0, z0 + 0.02, -self.REC - 2.5), "interior")
            for x0_, x1_, ztop in zip(xs[:-1], xs[1:], tops[:-1]):
                self.fquad(face, x0_, x1_, z0, ztop, -self.REC, self.infill)
            return
        if kind is None:
            self.fquad(face, u0, u1, z0, z1, -self.REC, self.infill)
            return
        w = u1 - u0
        if kind == "shop":
            ou0, ou1, oz0, oz1 = u0 + 0.12, u1 - 0.12, z0, z1 - 0.15
        elif kind == "door":
            dw = 1.1
            c = u0 + w * rng.uniform(0.3, 0.7)
            ou0, ou1, oz0, oz1 = c - dw / 2, c + dw / 2, z0, z0 + 2.2
        elif kind == "balc":
            ow = min(2.4, w * 0.75)
            c = (u0 + u1) / 2
            ou0, ou1, oz0, oz1 = c - ow / 2, c + ow / 2, z0, z0 + 2.25
        elif kind == "small":
            ow = min(0.7, w * 0.4)
            c = u0 + w * rng.uniform(0.3, 0.7)
            ou0, ou1, oz0, oz1 = c - ow / 2, c + ow / 2, z0 + 1.3, z0 + 2.0
        else:
            ow = min(1.7, w * 0.6) if w > 1.6 else w * 0.6
            c = (u0 + u1) / 2
            ou0, ou1, oz0, oz1 = c - ow / 2, c + ow / 2, z0 + 0.9, min(z0 + 2.3, z1 - 0.1)
        # wall around the opening (4 pieces on the infill plane)
        if ou0 > u0:
            self.fquad(face, u0, ou0, z0, z1, -self.REC, self.infill)
        if ou1 < u1:
            self.fquad(face, ou1, u1, z0, z1, -self.REC, self.infill)
        if oz0 > z0:
            self.fquad(face, ou0, ou1, z0, oz0, -self.REC, self.infill)
        if oz1 < z1:
            self.fquad(face, ou0, ou1, oz1, z1, -self.REC, self.infill)
        self.opening(face, ou0, ou1, oz0, oz1, {"shop": "shop", "door": "door"}.get(kind, "win"))
        if kind == "win" and rng.random() < 0.18 and face in ("front", "back"):     # AC unit under the window
            uc = rng.uniform(u0 + 0.5, u1 - 0.5)
            zc = oz0 - rng.uniform(0.1, 0.4)
            self.fbox(face, uc - 0.42, uc + 0.42, zc - 0.55, zc, 0.0, 0.32, "ac")
            self.fquad(face, uc - 0.35, uc + 0.35, zc - 0.5, zc - 0.05, 0.325, "grille")
        if kind == "win" and rng.random() < 0.3:                      # sill
            self.fbox(face, ou0 - 0.08, ou1 + 0.08, oz0 - 0.06, oz0, -self.REC, 0.04, "concrete")

    def columns_beams(self, face, nb, length, kmax, hidden_bays=()):
        """RC frame on a face: columns at bay lines, beams (slab edges) at every floor."""
        bw = length / nb
        for i in range(nb + 1):
            u = i * bw
            u0, u1 = max(0.0, u - self.COL / 2), min(length, u + self.COL / 2)
            top = self.levels[kmax]
            if self.cut and face == "front" and i > self.cut[0]:
                top = self.levels[self.cut[1]]
            if self.cut and face == "right" and self.cut is not None:
                top = self.levels[self.cut[1]] if u < self.cut[2] else top
            self.fbox(face, u0, u1, -0.6, top, -self.COL / 2 - 0.1, 0.0, self.frame, skip=("-z",))
        for k in range(1, kmax + 1):
            z = self.levels[k]
            u0, u1 = 0.0, length
            if self.cut and face == "front" and k > self.cut[1]:
                u1 = (self.cut[0]) * bw + self.COL / 2
            self.fbox(face, u0, u1, z - self.BEAM, z + 0.15, -0.3, 0.0, self.frame)

    def build(self):
        s, rng = self.s, self.rng
        dmg = s["damage"]
        W, D = self.W, self.D
        nst = self.n
        if dmg == "corner":
            self.cut = (self.nb - 2 if self.nb > 2 else 1, max(1, nst - 2), D * rng.uniform(0.35, 0.6))
        if dmg == "facade":
            for _ in range(int(rng.integers(3, 6))):
                self.missing.add(("front", int(rng.integers(self.nb)), int(rng.integers(1, nst))))
        if dmg in ("cracked", "soft", "top"):
            for _ in range(int(rng.integers(1, 3))):
                self.missing.add(("front", int(rng.integers(self.nb)), int(rng.integers(1, nst))))
        # plinth / ground slab edge
        self.mb.box(-W / 2, W / 2, 0.02, D, -0.6, 0.12, "concrete", skip=("-z",))
        faces = [("front", self.nb, W), ("back", self.nb, W), ("left", self.nd, D), ("right", self.nd, D)]
        for face, nb, length in faces:
            self.columns_beams(face, nb, length, nst)
            bw = length / nb
            for k in range(nst):
                for b in range(nb):
                    if self.cut and face == "front" and b > self.cut[0] - 1 and k >= self.cut[1]:
                        continue
                    u0, u1 = b * bw + self.COL / 2, (b + 1) * bw - self.COL / 2
                    kind = self._kind(face, b, k, nb)
                    self.panel(face, b, k, u0, u1, kind)
                    if kind == "balc":
                        self.balcony(face, u0 - 0.1, u1 + 0.1, self.levels[k])
        # interior volume behind the facades (dark box per storey: floors, ceilings seen through openings)
        for k in range(nst):
            z = self.levels[k] + 0.16
            self.mb.quad((-W / 2 + 0.4, 0.4, z), (W / 2 - 0.4, 0.4, z), (W / 2 - 0.4, D - 0.4, z),
                         (-W / 2 + 0.4, D - 0.4, z), "interior")
        self.roof()
        if s["ground"] == "shop":
            self.signs()
        self.drainpipes()
        if dmg == "corner":
            self.corner_collapse()
        if dmg == "top":
            self.pancake_top()
        if dmg in ("cracked", "soft", "facade", "top", "corner"):
            self.cracks()
        if dmg == "soft":
            self.soft_storey()
        return self.mb

    def _kind(self, face, b, k, nb):
        s, rng = self.s, self.rng
        if face == "front":
            if k == 0:
                if s["ground"] == "shop":
                    return "door" if (b == nb // 2 and nb > 2 and rng.random() < 0.5) else "shop"
                return "door" if b == nb // 2 else "win"
            if s["balc"] and (b % 2 == (1 if s["balc"] == 1 else b % 2)) and rng.random() < 0.85:
                return "balc"
            return "win"
        if face == "back":
            if k == 0:
                return "win" if rng.random() < 0.6 else None
            return "win" if rng.random() < 0.85 else "small"
        # side (party) walls: mostly blank, a small window near the front / back
        if k > 0 and b in (0, nb - 1) and rng.random() < 0.55:
            return "small"
        return None

    def balcony(self, face, u0, u1, z):
        rng = self.rng
        dep = rng.uniform(1.0, 1.35)
        self.fbox(face, u0, u1, z - 0.15, z + 0.12, 0.0, dep, "concrete")
        if rng.random() < 0.55:                                    # solid parapet (plastered)
            self.fbox(face, u0, u1, z + 0.12, z + 1.05, dep - 0.1, dep, self.frame)
            self.fbox(face, u0, u0 + 0.1, z + 0.12, z + 1.05, 0.0, dep - 0.1, self.frame)
            self.fbox(face, u1 - 0.1, u1, z + 0.12, z + 1.05, 0.0, dep - 0.1, self.frame)
        else:                                                      # steel railing: top rail + bars
            self.fbox(face, u0, u1, z + 1.0, z + 1.05, dep - 0.06, dep - 0.01, "rail")
            for uu in np.arange(u0 + 0.06, u1, 0.16):
                self.fbox(face, uu, uu + 0.02, z + 0.12, z + 1.0, dep - 0.05, dep - 0.03, "rail")
        if rng.random() < 0.3:                                     # clutter: a tank / crate on the balcony
            uc = rng.uniform(u0 + 0.4, u1 - 0.4)
            self.fbox(face, uc - 0.3, uc + 0.3, z + 0.12, z + 0.8, 0.2, 0.7, rng.choice(["plastic", "tank_w", "wood"]))

    def roof(self):
        s, rng, mb = self.s, self.rng, self.mb
        W, D = self.W, self.D
        zr = self.levels[-1] + 0.15
        cut = self.cut
        # roof slab top (skip the collapsed corner)
        if cut:
            xc = -W / 2 + cut[0] * (W / self.nb)
            mb.quad((-W / 2, 0, zr), (xc, 0, zr), (xc, D, zr), (-W / 2, D, zr), "roof")
            mb.quad((xc, cut[2], zr), (W / 2, cut[2], zr), (W / 2, D, zr), (xc, D, zr), "roof")
        else:
            mb.quad((-W / 2, 0, zr), (W / 2, 0, zr), (W / 2, D, zr), (-W / 2, D, zr), "roof")
        # parapet with a broken top in places
        for (a, b) in (((-W / 2, 0), (W / 2, 0)), ((W / 2, 0), (W / 2, D)), ((W / 2, D), (-W / 2, D)),
                       ((-W / 2, D), (-W / 2, 0))):
            a, b = np.array(a), np.array(b)
            L = np.linalg.norm(b - a)
            u = (b - a) / L
            nrm = np.array([u[1], -u[0]])
            xs, tops = jag_top(rng, 0.0, L, TW.PARAPET, 0.06, 0.6)
            if rng.random() < 0.35:                                # a chunk broken out of the parapet
                i0 = int(rng.integers(0, max(1, len(xs) - 6)))
                tops[i0:i0 + int(rng.integers(3, 7))] = rng.uniform(0.1, 0.4)
            tops = np.clip(tops, 0.08, 1.15)
            sub = MB(rng)
            wall_panel(sub, 0.0, L, zr - 0.1, zr + tops, -0.2, 0.0, self.frame, self.frame, self.frame, xs=xs)
            R = np.array([[u[0], -nrm[0], 0], [u[1], -nrm[1], 0], [0, 0, 1.0]])
            if np.linalg.det(R) < 0:
                R[:, 1] *= -1
            mb.merge(sub, R, (a[0], a[1], 0.0))
        # stair house, tanks, solar heaters, dishes, antennas
        sx, sy = rng.uniform(-W / 2 + 2.0, W / 2 - 4.5), rng.uniform(D * 0.35, D - 3.6)
        if not (cut and sx + 2.6 > -W / 2 + cut[0] * W / self.nb):
            mb.box(sx, sx + 2.6, sy, sy + 3.2, zr, zr + 2.5, self.frame, skip=("-z",))
            mb.box(sx - 0.1, sx + 2.7, sy - 0.1, sy + 3.3, zr + 2.5, zr + 2.65, "concrete", skip=("-z",))
            mb.quad((sx + 0.8, sy - 0.01, zr), (sx + 1.8, sy - 0.01, zr), (sx + 1.8, sy - 0.01, zr + 2.1),
                    (sx + 0.8, sy - 0.01, zr + 2.1), "door")
        roof = s["roof"]
        ntank = int(rng.integers(1, 4))
        xmax = W / 2 - 1.0 if not cut else -W / 2 + cut[0] * W / self.nb - 1.0
        for _ in range(ntank):
            cx, cy = rng.uniform(-W / 2 + 1.0, max(-W / 2 + 1.2, xmax)), rng.uniform(1.2, D - 1.2)
            mat = rng.choice(["tank_w", "tank_b", "tank_s", "tank_w"])
            if rng.random() < 0.5:                                 # vertical tank on a stand
                for dx in (-0.4, 0.4):
                    for dy in (-0.4, 0.4):
                        mb.box(cx + dx - 0.03, cx + dx + 0.03, cy + dy - 0.03, cy + dy + 0.03, zr, zr + 0.7, "steel")
                mb.cyl((cx, cy, zr + 0.7), 0.55, 1.25, 12, mat)
            else:                                                  # horizontal tank (Turkey / Taiwan roofs)
                mb.cyl((cx - 0.9, cy, zr + 0.65), 0.5, 1.8, 12, mat, axis=0)
                mb.box(cx - 0.7, cx + 0.7, cy - 0.35, cy + 0.35, zr, zr + 0.2, "steel")
        if roof == "solar":
            for _ in range(int(rng.integers(1, 4))):
                cx, cy = rng.uniform(-W / 2 + 1.5, max(-W / 2 + 1.6, xmax - 0.5)), rng.uniform(1.5, D - 2.5)
                sub = MB(rng)
                sub.box(-1.0, 1.0, -0.6, 0.6, -0.03, 0.03, "frame")
                sub.quad((-0.95, -0.55, 0.035), (0.95, -0.55, 0.035), (0.95, 0.55, 0.035), (-0.95, 0.55, 0.035), "solar")
                mb.merge(sub, Rx(-38), (cx, cy, zr + 0.75))
                mb.cyl((cx - 1.0, cy + 0.55, zr + 1.25), 0.22, 2.0, 10, "tank_s", axis=0)
                mb.box(cx - 0.9, cx + 0.9, cy - 0.5, cy + 0.6, zr, zr + 0.06, "steel")
        if roof == "shed":                                         # sheet-metal rooftop addition
            cx = rng.uniform(-W / 2 + 0.5, max(-W / 2 + 0.6, xmax - 4.0))
            cy = rng.uniform(D * 0.4, D - 3.2)
            mb.box(cx, cx + 4.0, cy, cy + 3.0, zr, zr + 2.3, "metal", skip=("-z",))
        for _ in range(int(rng.integers(0, 3))):                   # satellite dishes on the parapet / roof
            cx, cy = rng.uniform(-W / 2 + 0.6, max(-W / 2 + 0.7, xmax)), rng.uniform(0.6, D - 0.6)
            mb.box(cx - 0.02, cx + 0.02, cy - 0.02, cy + 0.02, zr, zr + 1.0, "steel")
            sub = MB(rng)
            sub.cyl((0, 0, 0), 0.38, 0.12, 12, "dish", caps=(False, True), r2=0.30)
            mb.merge(sub, Rz(rng.uniform(0, 360)) @ Rx(65), (cx, cy, zr + 1.0))
        if rng.random() < 0.6:                                     # antenna mast
            cx, cy = rng.uniform(-W / 2 + 1, max(-W / 2 + 1.1, xmax)), rng.uniform(1, D - 1)
            mb.cyl((cx, cy, zr), 0.025, 3.2, 5, "steel", caps=(False, True))
            for h in (2.4, 2.9):
                mb.box(cx - 0.6, cx + 0.6, cy - 0.01, cy + 0.01, zr + h, zr + h + 0.02, "steel")

    def signs(self):
        rng, W = self.rng, self.W
        bw = W / self.nb
        z0 = self.h0 - self.BEAM - 0.05
        for b in range(self.nb):
            if rng.random() < 0.25:
                continue
            u0, u1 = b * bw + 0.3, (b + 1) * bw - 0.3
            mat = f"sign{int(rng.integers(5))}"
            dz = rng.uniform(-0.05, 0.05)
            self.fbox("front", u0, u1, z0 + dz, z0 + 0.75 + dz, 0.0, 0.08, mat)
            if rng.random() < 0.25:                                # sign hanging loose at one end
                self.fbox("front", u0 + 0.2, u1 - 0.2, z0 - 0.8, z0 - 0.1, 0.05, 0.12, mat)
            if rng.random() < 0.3:                                 # faded awning
                sub = MB(rng)
                L = u1 - u0
                sub.quad((0, 0, 0), (L, 0, 0), (L, -1.1, -0.45), (0, -1.1, -0.45), "awning")
                sub.quad((0, -1.1, -0.45), (L, -1.1, -0.45), (L, 0, 0), (0, 0, 0), "awning")
                self.mb.merge(sub, None, (-self.W / 2 + u0, 0.0, z0 - 0.15))

    def drainpipes(self):
        rng = self.rng
        top = self.levels[-1] + 0.5
        for u in (0.25, self.W - 0.25):
            if rng.random() < 0.6:
                p0 = self.P3("front", u, -0.3, 0.12)
                self.mb.cyl(p0, 0.05, top + 0.3, 6, "plastic", caps=(False, False))

    def cracks(self):
        """X-cracks between openings and along the beams (dark strips just proud of the face) + spalls."""
        rng = self.rng
        for _ in range(int(rng.integers(4, 10))):
            face = rng.choice(["front", "front", "left", "right", "back"])
            length = self.W if face in ("front", "back") else self.D
            k = int(rng.integers(0, max(1, self.n - 1)))
            z0 = self.levels[k] + 0.3
            u = rng.uniform(0.5, length - 0.5)
            d = rng.choice([-1, 1])
            pts = [(u, z0)]
            for _s in range(int(rng.integers(4, 9))):
                du, dz = d * rng.uniform(0.15, 0.4), rng.uniform(0.15, 0.4)
                pts.append((float(np.clip(pts[-1][0] + du, 0.1, length - 0.1)), pts[-1][1] + dz))
            for (u0, z0_), (u1, z1_) in zip(pts[:-1], pts[1:]):
                w = rng.uniform(0.008, 0.02)
                a, b = self.P3(face, u0, z0_, 0.006), self.P3(face, u1, z1_, 0.006)
                n2 = np.cross(b - a, self.face_frame(face)[2])
                n2 = n2 / (np.linalg.norm(n2) + 1e-9) * w
                # on the frame plane (columns / beams) or the infill: put it just in front of both
                for dd in (0.006, -self.REC + 0.006):
                    a2, b2 = self.P3(face, u0, z0_, dd), self.P3(face, u1, z1_, dd)
                    self.mb.quad(a2 - n2, b2 - n2, b2 + n2, a2 + n2, "crack")

    def corner_collapse(self):
        """Debris and hanging slabs at the collapsed corner bays."""
        rng, mb = self.rng, self.mb
        W, D = self.W, self.D
        nb, k = self.cut[0], self.cut[1]
        x0 = -W / 2 + nb * (W / self.nb)
        for j in range(k, self.n + 1):
            if rng.random() < 0.7:                                 # slab hanging from the break, hinged
                L = rng.uniform(1.5, 3.0)
                sub = MB(rng)
                sub.box(0, L, -1.5, 1.5, -0.18, 0.0, "concrete", mats={"+x": "fracture", "-y": "fracture", "+y": "fracture"})
                ang = rng.uniform(25, 70)
                mb.merge(sub, Ry(ang) @ Rz(rng.normal(0, 8)), (x0 + 0.1, rng.uniform(1.5, D - 1.5),
                                                                  self.levels[min(j, self.n)] + 0.1))
        # exposed floor edges on the break line (interior visible)
        for j in range(k, self.n):
            zj = self.levels[j]
            mb.quad((x0, 0.0, zj), (x0, D, zj), (x0, D, zj + TW.STOREY - 0.2), (x0, 0.0, zj + TW.STOREY - 0.2), "interior")
        # rubble heap at the foot of the corner
        hx = W / 2 - x0 + 2.5
        n = 22
        H = np.zeros((n, n))
        g = np.linspace(-1, 1, n)
        X, Y = np.meshgrid(g, g)
        H = np.clip(1 - (X ** 2 + Y ** 2), 0, None) * rng.uniform(1.6, 2.6)
        H += rng.normal(0, 0.12, H.shape) * (H > 0)
        xs = np.linspace(x0 - 0.5, x0 + hx, n)
        ys = np.linspace(-3.0, D * 0.8, n)
        P = np.array([(xs[i], ys[j], max(H[j, i], 0.0) - 0.2) for j in range(n) for i in range(n)])
        F = []
        for j in range(n - 1):
            for i in range(n - 1):
                a = j * n + i
                F += [(a, a + 1, a + n + 1), (a, a + n + 1, a + n)]
        mb.mesh(P, F, "fracture", smooth=True)
        for _ in range(14):
            c = (rng.uniform(x0, x0 + hx), rng.uniform(-3.0, D * 0.6), rng.uniform(0.2, 1.4))
            s = rng.uniform(0.3, 1.0)
            mb.merge(rock_mesh(rng, s * 1.6, s, s * 0.4, "concrete" if rng.random() < 0.6 else "fracture"),
                     Rz(rng.uniform(0, 360)) @ Rx(rng.uniform(-30, 30)), c)

    def pancake_top(self):
        """The top storey crushed: its slab tilted down onto the storey below, debris in the gap."""
        rng, mb = self.rng, self.mb
        W, D = self.W, self.D
        z = self.levels[-2] + 0.9
        sub = MB(rng)
        sub.box(-W / 2 - 0.2, W / 2 + 0.2, -0.3, D + 0.3, -0.2, 0.15, "concrete", mats={"+z": "roof", "-y": "fracture"})
        mb.merge(sub, Rx(rng.uniform(2.5, 5.0)) @ Ry(rng.uniform(-3, 3)), (0, 0, z))
        for _ in range(16):
            c = (rng.uniform(-W / 2, W / 2), rng.uniform(-0.5, 1.0), self.levels[-2] + rng.uniform(0.1, 0.8))
            s = rng.uniform(0.25, 0.7)
            mb.merge(rock_mesh(rng, s * 1.5, s, s * 0.5, "fracture"), Rz(rng.uniform(0, 360)), c)

    def soft_storey(self):
        """Ground storey crushed: shift everything above the first floor down and lean it (done on the arrays in
        build_bldg); here: debris band along the front."""
        rng, mb = self.rng, self.mb
        for _ in range(18):
            c = (rng.uniform(-self.W / 2, self.W / 2), rng.uniform(-1.2, 0.6), rng.uniform(0.0, 0.5))
            s = rng.uniform(0.3, 0.8)
            mb.merge(rock_mesh(rng, s * 1.4, s, s * 0.5, "fracture" if rng.random() < 0.6 else "brick"),
                     Rz(rng.uniform(0, 360)), c)


def build_bldg(rng, idx):
    spec = TW.BLDG[idx]
    b = Bldg(rng, spec, idx)
    mb = b.build()
    A = mb.arrays()
    if spec["damage"] == "soft":                                   # crush the ground storey, lean the rest
        P = A["P"].astype(float)
        drop = b.h0 - 1.1
        up = P[:, 2] > b.h0 - 0.6
        P[up, 2] -= drop
        lo = (~up) & (P[:, 2] > 0.2)
        P[lo, 2] = 0.2 + (P[lo, 2] - 0.2) * (0.9 / max(b.h0 - 0.8, 0.1))
        ang = math.radians(rng.uniform(2.5, 4.0))
        P[up, 1] -= np.tan(ang) * (P[up, 2] - 1.0)
        P[up, 0] += np.tan(ang * 0.4) * (P[up, 2] - 1.0)
        A["P"] = P.astype(np.float32)
    return A, b


# ------------------------------------------------------------------------------------------------ vehicles
def car_mb(rng, spec, crushed=False):
    """Low-poly car: extruded side profile with a tapered greenhouse, wheels, lamps, mirrors."""
    L, Wd, H = spec["L"], spec["W"], spec["H"]
    name = spec["name"]
    mb = MB(rng)
    hw = Wd / 2
    r = 0.31 if name in ("sedan", "hatch") else 0.35
    zb = 0.18                                                   # body bottom
    belt = {"sedan": 0.85, "hatch": 0.88, "suv": 1.02, "van": 1.05, "pickup": 1.0}[name]
    xf, xr = L / 2, -L / 2
    # lower body profile (x, z) CCW, side view
    hood = {"sedan": 0.95, "hatch": 0.75, "suv": 0.85, "van": 0.55, "pickup": 0.95}[name]
    trunk = {"sedan": 0.75, "hatch": 0.15, "suv": 0.2, "van": 0.12, "pickup": 1.95}[name]
    prof = [(xr + 0.05, zb), (xf - 0.05, zb), (xf, zb + 0.18), (xf, belt - 0.12), (xf - 0.15, belt - 0.02),
            (xf - hood, belt), (xr + trunk, belt), (xr + 0.08, belt - 0.03), (xr, belt - 0.15), (xr, zb + 0.16)]
    prof = np.array(prof)
    taper = 0.04
    def side(y):
        return np.c_[prof[:, 0], np.full(len(prof), y), prof[:, 1]]
    S0, S1 = side(-hw), side(hw)
    n = len(prof)
    for i in range(n):
        j = (i + 1) % n
        a, b, c, d = S0[i], S0[j], S1[j], S1[i]
        dz = (a[2] + b[2]) / 2
        top = dz > belt - 0.13 and abs(a[0] - b[0]) > 0.3
        mb.quad(a, d, c, b, "paint_top" if top else "paint")
    mb.poly(S0, "paint")
    mb.poly(S1[::-1], "paint")
    # greenhouse: bottom at belt (inset), top at roof (more inset)
    if name == "pickup":
        cab = (xr + trunk - 0.05, xf - hood)
    else:
        cab = (xr + trunk + 0.02, xf - hood)
    c0, c1 = cab
    rt = H
    ins_b, ins_t = 0.06, 0.16 + taper
    lean_f = {"sedan": 0.75, "hatch": 0.6, "suv": 0.55, "van": 0.45, "pickup": 0.55}[name]
    lean_r = {"sedan": 0.55, "hatch": 0.15, "suv": 0.1, "van": 0.05, "pickup": 0.08}[name]
    if crushed:
        rt = belt + (H - belt) * rng.uniform(0.25, 0.45)
    bot = [(c1, -hw + ins_b), (c1, hw - ins_b), (c0, hw - ins_b), (c0, -hw + ins_b)]
    top = [(c1 - lean_f, -hw + ins_t), (c1 - lean_f, hw - ins_t), (c0 + lean_r, hw - ins_t), (c0 + lean_r, -hw + ins_t)]
    B = np.array([(x, y, belt) for x, y in bot])
    T = np.array([(x, y, rt) for x, y in top])
    if crushed:
        T[:, 1] += rng.normal(0, 0.06)
        T[:, 0] += rng.normal(0, 0.15)
    gl = "glass" if not crushed else "interior"
    for i in range(4):
        j = (i + 1) % 4
        mb.quad(B[i], B[j], T[j], T[i], gl if not (crushed and rng.random() < 0.3) else "glass_dusty")
    mb.poly(T, "paint_top")
    # window pillars (thin paint strips) on the sides: B-pillar
    xm = (c0 + c1) / 2
    for sgn in (-1, 1):
        y = sgn * (hw - ins_b + 0.005)
        yt = sgn * (hw - ins_t + 0.005)
        p = [(xm - 0.06, y, belt), (xm + 0.06, y, belt), (xm + 0.06 - (lean_f - lean_r) * 0.0, yt, rt),
             (xm - 0.06, yt, rt)]
        p = np.array(p)
        mb.quad(*(p if sgn < 0 else p[::-1]), "paint")
    # wheels
    for x in (xf - 0.85 if name != "van" else xf - 0.75, xr + 0.85):
        for sgn in (-1, 1):
            c = np.array([x, sgn * (hw - 0.12), r])
            sub = MB(rng)
            sub.cyl((0, -0.11, 0), r, 0.22, 12, "tyre", axis=1)
            sub.cyl((0, 0.115 * 1, 0), r * 0.55, 0.005, 10, "chrome", axis=1, caps=(False, True))
            mb.merge(sub, None if sgn > 0 else Rz(180), c)
    # bumpers, lamps, plates, mirrors
    mb.box(xf - 0.02, xf + 0.06, -hw + 0.05, hw - 0.05, zb + 0.08, zb + 0.32, "trim")
    mb.box(xr - 0.06, xr + 0.02, -hw + 0.05, hw - 0.05, zb + 0.08, zb + 0.32, "trim")
    for sgn in (-1, 1):
        mb.quad((xf + 0.002, sgn * (hw - 0.12) - 0.15, belt - 0.22), (xf + 0.002, sgn * (hw - 0.12) + 0.15, belt - 0.22),
                (xf + 0.002, sgn * (hw - 0.12) + 0.15, belt - 0.1), (xf + 0.002, sgn * (hw - 0.12) - 0.15, belt - 0.1),
                "lamp")
        mb.quad((xr - 0.002, sgn * (hw - 0.12) + 0.14, belt - 0.25), (xr - 0.002, sgn * (hw - 0.12) - 0.14, belt - 0.25),
                (xr - 0.002, sgn * (hw - 0.12) - 0.14, belt - 0.1), (xr - 0.002, sgn * (hw - 0.12) + 0.14, belt - 0.1),
                "taillamp")
        mb.box(c1 - 0.1, c1 + 0.02, sgn * hw - 0.02 + (0.12 if sgn > 0 else -0.12) * 0.5, sgn * hw + (0.14 if sgn > 0 else -0.14),
               belt + 0.05, belt + 0.18, "trim") if False else None
    if name == "pickup":                                        # bed walls
        mb.box(xr, xr + trunk - 0.1, -hw, -hw + 0.05, belt, belt + 0.35, "paint")
        mb.box(xr, xr + trunk - 0.1, hw - 0.05, hw, belt, belt + 0.35, "paint")
        mb.box(xr, xr + 0.05, -hw, hw, belt, belt + 0.35, "paint")
        if rng.random() < 0.8:
            mb.merge(rock_mesh(rng, 0.8, 0.6, 0.3, "fracture"), None, (xr + 0.8, 0.1, belt + 0.1))
    if name in ("suv", "van") and rng.random() < 0.6:          # roof rails
        for sgn in (-1, 1):
            mb.box(c0 + lean_r + 0.1, c1 - lean_f - 0.1, sgn * (hw - ins_t - 0.04) - 0.02, sgn * (hw - ins_t - 0.04) + 0.02,
                   rt, rt + 0.05, "trim")
    return mb


def car_module(rng, var):
    spec = TW.CARS[var % len(TW.CARS)]
    return car_mb(rng, spec)


def truck_module(rng, var):
    """Small two-axle truck: cab, chassis, dropside cargo bed with some load (or a box body)."""
    mb = MB(rng)
    L, Wd, H = TW.TRUCK["L"], TW.TRUCK["W"], TW.TRUCK["H"]
    hw = Wd / 2
    xf = L / 2
    # cab
    cab0 = xf - 1.75
    mb.box(cab0, xf, -hw + 0.05, hw - 0.05, 0.75, 2.45, "paint", mats={"+z": "paint_top"})
    mb.quad((xf + 0.003, -hw + 0.18, 1.55), (xf + 0.003, hw - 0.18, 1.55), (xf + 0.003, hw - 0.18, 2.3),
            (xf + 0.003, -hw + 0.18, 2.3), "glass")
    for sgn in (-1, 1):
        mb.quad((cab0 + 0.9, sgn * (hw - 0.05 + 0.003), 1.6), (xf - 0.15, sgn * (hw - 0.05 + 0.003), 1.6),
                (xf - 0.15, sgn * (hw - 0.05 + 0.003), 2.25), (cab0 + 0.9, sgn * (hw - 0.05 + 0.003), 2.25), "glass")
    mb.box(xf - 0.02, xf + 0.08, -hw, hw, 0.45, 0.75, "trim")
    # chassis
    mb.box(-L / 2, cab0, -0.5, 0.5, 0.55, 0.8, "trim")
    # cargo
    if var == 0:
        mb.box(-L / 2, cab0 - 0.1, -hw, hw, 0.8, 0.95, "wood", mats={"+z": "wood"})
        for sgn in (-1, 1):
            mb.box(-L / 2, cab0 - 0.1, sgn * hw - (0.04 if sgn > 0 else 0), sgn * hw + (0 if sgn > 0 else 0.04), 0.95, 1.55, "paint")
        mb.box(-L / 2, -L / 2 + 0.04, -hw, hw, 0.95, 1.55, "paint")
        for _ in range(6):                                      # load: sacks / rubble
            c = (rng.uniform(-L / 2 + 0.6, cab0 - 0.6), rng.uniform(-hw + 0.5, hw - 0.5), 1.1)
            mb.merge(rock_mesh(rng, rng.uniform(0.5, 0.9), rng.uniform(0.4, 0.7), 0.35,
                               rng.choice(["fracture", "brick"])), Rz(rng.uniform(0, 360)), c)
    else:
        mb.box(-L / 2, cab0 - 0.1, -hw - 0.05, hw + 0.05, 0.8, H, "metal", mats={"+z": "paint_top"})
    for x in (xf - 1.0, -L / 2 + 1.3, -L / 2 + 2.35):
        for sgn in (-1, 1):
            sub = MB(rng)
            sub.cyl((0, -0.13, 0), 0.47, 0.26, 12, "tyre", axis=1)
            mb.merge(sub, None if sgn > 0 else Rz(180), (x, sgn * (hw - 0.17), 0.47))
    return mb


def wreck_module(rng, var):
    """A parked car crushed by facade debris: flattened roof, broken glass, a slab and chunks on top."""
    from isaac import rubble as RB
    spec = TW.CARS[[0, 1, 2][var % 3]]
    mb = car_mb(rng, spec, crushed=True)
    P, F = RB.slab(rng, rng.uniform(2.0, 3.0), rng.uniform(1.4, 2.0), T=0.2)
    sub = MB(rng).mesh(P, F, "concrete")
    mb.merge(sub, Ry(rng.uniform(-18, 18)) @ Rx(rng.uniform(-12, 12)) @ Rz(rng.uniform(-30, 30)),
             (rng.uniform(-0.6, 0.6), rng.uniform(-0.3, 0.3), spec["H"] * 0.75))
    for _ in range(8):
        s = rng.uniform(0.2, 0.6)
        c = (rng.uniform(-spec["L"] / 2, spec["L"] / 2), rng.uniform(-1.3, 1.3), rng.uniform(0.0, 1.2))
        mb.merge(rock_mesh(rng, s * 1.4, s, s * 0.6, rng.choice(["fracture", "brick", "concrete"])),
                 Rz(rng.uniform(0, 360)), c)
    return mb


# ------------------------------------------------------------------------------------------------ street furniture
def pole_module(rng, var):
    """Concrete distribution pole (octagonal, tapered), steel crossarm with 3 insulators; var 1: transformer +
    a lower telecom arm. Crossarm along y at z = town.POLE_TOP."""
    mb = MB(rng)
    mb.cyl((0, 0, -0.5), 0.17, 8.0, 8, "concrete", caps=(False, True), r2=0.10, smooth=True)
    z = TW.POLE_TOP
    mb.box(-0.05, 0.05, -0.85, 0.85, z - 0.08, z, "steel")
    mb.box(-0.02, 0.02, -0.5, 0.0, z - 0.6, z - 0.08, "steel")
    for y in TW.POLE_ARMS:
        mb.cyl((0, y, z), 0.04, 0.14, 6, "glass_dusty", caps=(False, True))
    if var == 1:
        mb.cyl((0.42, 0, z - 2.2), 0.32, 0.95, 10, "steel")
        mb.box(0.08, 0.2, -0.05, 0.05, z - 2.2, z - 1.4, "steel")
        mb.box(-0.04, 0.04, -0.55, 0.55, z - 1.6, z - 1.52, "steel")
    mb.box(-0.12, 0.12, -0.12, 0.12, 2.6, 3.0, "steel") if rng.random() < 0.5 else None
    return mb


def light_module(rng, var):
    """Galvanised street-light column, 6.4 m, curved arm (+x) with a flat luminaire."""
    mb = MB(rng)
    mb.cyl((0, 0, -0.3), 0.09, 6.6, 8, "steel", caps=(False, True), r2=0.055)
    mb.cyl((0, 0, -0.3), 0.14, 0.9, 8, "steel", caps=(False, True), r2=0.11)
    arm = [(0, 0, 6.0), (0.3, 0, 6.35), (0.8, 0, 6.5), (1.5, 0, 6.52)]
    mb.tube(arm, 0.035, "steel", sides=6)
    if var == 1:
        mb.tube([(0, 0, 6.0), (-0.3, 0, 6.35), (-0.8, 0, 6.5), (-1.5, 0, 6.52)], 0.035, "steel", sides=6)
        mb.box(-1.95, -1.3, -0.14, 0.14, 6.42, 6.55, "steel")
        mb.quad((-1.92, -0.12, 6.415), (-1.32, -0.12, 6.415), (-1.32, 0.12, 6.415), (-1.92, 0.12, 6.415), "lamp")
    mb.box(1.3, 1.95, -0.14, 0.14, 6.42, 6.55, "steel")
    mb.quad((1.32, 0.12, 6.415), (1.92, 0.12, 6.415), (1.92, -0.12, 6.415), (1.32, -0.12, 6.415), "lamp")
    return mb


def _blob(rng, c, r, n_sub=2, jag=0.3):
    """Displaced icosphere (foliage clump)."""
    t = (1 + 5 ** 0.5) / 2
    V = np.array([(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t),
                  (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)], float)
    F = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6),
         (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10),
         (8, 6, 7), (9, 8, 1)]
    V = list(V / np.linalg.norm(V, axis=1, keepdims=True))
    for _ in range(n_sub):
        cache, F2 = {}, []
        def mid(a, b):
            k = (min(a, b), max(a, b))
            if k not in cache:
                m = (V[a] + V[b]) / 2
                V.append(m / np.linalg.norm(m))
                cache[k] = len(V) - 1
            return cache[k]
        for a, b, cc in F:
            ab, bc, ca = mid(a, b), mid(b, cc), mid(cc, a)
            F2 += [(a, ab, ca), (b, bc, ab), (cc, ca, bc), (ab, bc, ca)]
        F = F2
    V = np.array(V)
    ph = rng.uniform(0, 6.28, 6)
    d = 1 + jag * (0.5 * np.sin(3 * V[:, 0] + ph[0]) * np.sin(2.6 * V[:, 1] + ph[1]) +
                   0.35 * np.sin(5.3 * V[:, 2] + ph[2]) + 0.25 * np.sin(7.1 * V[:, 0] + 6.3 * V[:, 1] + ph[3]))
    d += rng.normal(0, jag * 0.15, len(V))
    P = V * d[:, None] * np.asarray(r) + np.asarray(c)
    return P, np.array(F)


def tree_module(rng, var):
    """Street tree: var 0 broad (plane / mulberry), 1 columnar (cypress), 2 small ornamental. Dusty greens."""
    mb = MB(rng)
    H = [6.5, 7.5, 4.6][var]
    leaf = ["leaf", "leaf3", "leaf2"][var]
    if var == 1:
        mb.cyl((0, 0, -0.2), 0.12, 1.6, 7, "bark", caps=(False, False), r2=0.09)
        for k in range(7):
            z = 1.0 + k * 0.9
            r = 0.9 * (1 - k / 8) + 0.25
            P, F = _blob(rng, (rng.normal(0, 0.08), rng.normal(0, 0.08), z + 0.4), (r, r, 0.85), 2, 0.25)
            mb.mesh(P, F, leaf, smooth=True)
        return mb
    trunk_h = H * 0.42
    mb.tube([(0, 0, -0.2), (0.05, 0.02, trunk_h * 0.5), (rng.normal(0, 0.15), rng.normal(0, 0.15), trunk_h)],
            0.16 if var == 0 else 0.1, "bark", sides=7)
    nb = 4 if var == 0 else 3
    tips = []
    for k in range(nb):
        a = 2 * np.pi * k / nb + rng.normal(0, 0.3)
        tip = np.array([math.cos(a) * H * 0.22, math.sin(a) * H * 0.22, trunk_h + H * rng.uniform(0.22, 0.32)])
        mb.tube([(0, 0, trunk_h - 0.2), tip * [0.5, 0.5, 1] + [0, 0, -0.4], tip], 0.07 if var == 0 else 0.05, "bark", sides=5)
        tips.append(tip)
    nbl = 9 if var == 0 else 6
    R = H * (0.3 if var == 0 else 0.33)
    for k in range(nbl):
        a = rng.uniform(0, 2 * np.pi)
        rr = rng.uniform(0.0, 0.75) * R
        c = (math.cos(a) * rr, math.sin(a) * rr, trunk_h + H * rng.uniform(0.18, 0.45))
        s = R * rng.uniform(0.45, 0.7)
        P, F = _blob(rng, c, (s, s, s * 0.75), 2, 0.35)
        mb.mesh(P, F, leaf, smooth=True)
    return mb


def wall_module(rng, var):
    """3.0 m perimeter wall segment along x (y thickness), base at z = 0 (footing to -0.3)."""
    mb = MB(rng)
    L = 3.0
    t = 0.2
    if var in (0, 5):                                           # plastered block wall, coping
        H = rng.uniform(1.8, 2.1)
        xs, tops = jag_top(rng, -L / 2, L / 2, H, 0.01, 0.5)
        wall_panel(mb, -L / 2, L / 2, -0.3, tops, -t / 2, t / 2, "plaster", "plaster", "plaster", xs=xs)
        mb.box(-L / 2, L / 2, -t / 2 - 0.04, t / 2 + 0.04, H, H + 0.08, "concrete")
        if var == 5:
            mb.box(L / 2 - 0.4, L / 2, -0.22, 0.22, -0.3, H + 0.35, "plaster")
            mb.box(L / 2 - 0.45, L / 2 + 0.05, -0.27, 0.27, H + 0.35, H + 0.45, "concrete")
        # crack
        u = rng.uniform(-1, 1)
        pts = [(u, 0.1)]
        for _ in range(6):
            pts.append((pts[-1][0] + rng.normal(0, 0.12), pts[-1][1] + 0.3))
        for (a, za), (b, zb) in zip(pts[:-1], pts[1:]):
            if zb < H:
                mb.quad((a - 0.015, -t / 2 - 0.004, za), (b - 0.015, -t / 2 - 0.004, zb), (b + 0.015, -t / 2 - 0.004, zb),
                        (a + 0.015, -t / 2 - 0.004, za), "crack") if False else \
                    mb.quad((a + 0.015, -t / 2 - 0.004, za), (b + 0.015, -t / 2 - 0.004, zb), (b - 0.015, -t / 2 - 0.004, zb),
                            (a - 0.015, -t / 2 - 0.004, za), "crack")
    elif var == 1:                                              # plaster wall with a broken top, partly down
        xs, tops = jag_top(rng, -L / 2, L / 2, 1.9, 0.12, 0.25)
        tops[: len(tops) // 3] = np.linspace(0.5, tops[len(tops) // 3], len(tops) // 3) + rng.normal(0, 0.08, len(tops) // 3)
        wall_panel(mb, -L / 2, L / 2, -0.3, tops, -t / 2, t / 2, "plaster", "plaster", "brick", xs=xs)
        for _ in range(7):
            s = rng.uniform(0.15, 0.4)
            mb.merge(rock_mesh(rng, s * 1.6, s, s * 0.6, rng.choice(["brick", "fracture"])), Rz(rng.uniform(0, 360)),
                     (rng.uniform(-L / 2, -L / 6), rng.uniform(-0.8, 0.8), s * 0.15))
    elif var == 2:                                              # exposed hollow-brick wall, ragged
        xs, tops = jag_top(rng, -L / 2, L / 2, 1.7, 0.1, 0.25)
        wall_panel(mb, -L / 2, L / 2, -0.3, tops, -t / 2, t / 2, "brick", "brick", "brick", xs=xs)
    elif var == 3:                                              # low wall + steel railing (one bay bent)
        mb.box(-L / 2, L / 2, -t / 2, t / 2, -0.3, 0.85, "plaster")
        mb.box(-L / 2, L / 2, -t / 2 - 0.03, t / 2 + 0.03, 0.85, 0.92, "concrete")
        bend = rng.uniform(0.0, 0.5)
        mb.box(-L / 2, L / 2, -0.02, 0.02, 1.85, 1.89, "rail")
        for x in np.arange(-L / 2 + 0.06, L / 2, 0.13):
            lean = bend * max(0.0, math.sin((x + L / 2) / L * math.pi)) if var == 3 else 0.0
            mb.quad((x, 0.0, 0.92), (x + 0.02, 0.0, 0.92), (x + 0.02, -lean, 1.85), (x, -lean, 1.85), "rail")
            mb.quad((x + 0.02, 0.0, 0.92), (x, 0.0, 0.92), (x, -lean, 1.85), (x + 0.02, -lean, 1.85), "rail")
    else:                                                       # var 4: concrete block wall knocked down
        xs, tops = jag_top(rng, -L / 2, L / 2, 0.6, 0.1, 0.3)
        wall_panel(mb, -L / 2, L / 2, -0.3, np.clip(tops, 0.2, 1.0), -t / 2, t / 2, "concrete", "concrete", "fracture", xs=xs)
        for _ in range(12):
            mb.merge(MB(rng).box(-0.2, 0.2, -0.1, 0.1, -0.1, 0.1, "concrete", mats={"+x": "fracture"}),
                     Rz(rng.uniform(0, 360)) @ Rx(rng.uniform(-40, 40)),
                     (rng.uniform(-L / 2, L / 2), rng.uniform(-1.4, 1.4) * rng.choice([1, -1]) * 0.8 + 0.6, 0.08))
    return mb


def gate_module(rng, var):
    """3.6 m opening: two pillars, a steel gate leaf hanging open / fallen (var 1)."""
    mb = MB(rng)
    for x in (-1.8, 1.8):
        mb.box(x - 0.22, x + 0.22, -0.22, 0.22, -0.3, 2.2, "plaster")
        mb.box(x - 0.27, x + 0.27, -0.27, 0.27, 2.2, 2.32, "concrete")
    leaf = MB(rng)
    leaf.box(0, 1.65, -0.02, 0.02, 0.05, 0.1, "rail")
    leaf.box(0, 1.65, -0.02, 0.02, 1.75, 1.8, "rail")
    for x in np.arange(0.05, 1.65, 0.14):
        leaf.box(x, x + 0.025, -0.012, 0.012, 0.1, 1.75, "rail")
    leaf.box(0.0, 1.65, -0.006, 0.006, 0.1, 0.8, "metal")
    if var == 0:
        mb.merge(leaf, Rz(-rng.uniform(60, 110)), (-1.55, 0.0, 0.0))
        mb.merge(leaf, Rz(180 + rng.uniform(-15, 5)), (1.55, 0.0, 0.0))
    else:
        mb.merge(leaf, Rx(86) @ Rz(rng.uniform(-20, 20)), (-1.4, 0.1, 0.05))
        mb.merge(leaf, Rz(180 + rng.uniform(5, 30)) @ Rx(rng.uniform(-8, 8)), (1.55, 0.0, 0.0))
    return mb


def stub_module(rng, var):
    """Ground-storey remnant (<= 3 m): RC corner column with a beam stub and bent bars, broken brick infill with
    plaster on one face, debris at its foot."""
    mb = MB(rng)
    H = [2.9, 2.4, 1.8][var % 3]
    mb.box(-1.9, -1.45, -0.22, 0.22, -0.3, H, "concrete", mats={"+z": "fracture"})
    for _ in range(4):                                          # bars out of the column top
        p0 = np.array([rng.uniform(-1.85, -1.5), rng.uniform(-0.18, 0.18), H - 0.05])
        d = np.array([rng.normal(0.3, 0.4), rng.normal(0, 0.4), 1.0])
        pts = [p0]
        for k in range(5):
            d = d + rng.normal(0, 0.35, 3)
            pts.append(pts[-1] + d / np.linalg.norm(d) * 0.12)
        mb.tube(pts, 0.008, "rust", sides=4)
    if var != 2:
        mb.box(-1.95, -0.6, -0.25, 0.25, H - 0.45, H, "concrete", mats={"+x": "fracture", "+z": "fracture"})
    xs, tops = jag_top(rng, -1.45, 1.9, H * 0.85, 0.16, 0.3)
    tops = np.minimum(tops, H - 0.05)
    tops[-3:] = np.linspace(tops[-4], 0.4, 3)
    wall_panel(mb, -1.45, 1.9, -0.3, np.clip(tops, 0.3, H), -0.12, 0.08, "plaster", "brick", "brick", xs=xs)
    for _ in range(9):
        s = rng.uniform(0.15, 0.45)
        mb.merge(rock_mesh(rng, s * 1.6, s, s * 0.6, rng.choice(["brick", "fracture", "plaster"])),
                 Rz(rng.uniform(0, 360)), (rng.uniform(-1.5, 2.0), rng.uniform(-1.0, 1.0), s * 0.1))
    return mb


def debris_module(rng, var):
    """Small rubble heap (fallen facade): mound + slabs + chunks + a sheet or bars."""
    from isaac import rubble as RB
    mb = MB(rng)
    n = 14
    sx, sy, h = [(2.6, 1.8, 0.7), (1.8, 1.4, 0.5), (3.2, 2.0, 0.9), (1.6, 1.2, 0.45)][var % 4]
    g = np.linspace(-1, 1, n)
    X, Y = np.meshgrid(g, g)
    Hh = np.clip(1 - (X ** 2 + Y ** 2), 0, None) ** 0.8 * h + rng.normal(0, 0.05, X.shape)
    Hh[0, :] = Hh[-1, :] = Hh[:, 0] = Hh[:, -1] = -0.05
    P = np.c_[X.ravel() * sx / 2, Y.ravel() * sy / 2, Hh.ravel()]
    F = []
    for j in range(n - 1):
        for i in range(n - 1):
            a = j * n + i
            F += [(a, a + 1, a + n + 1), (a, a + n + 1, a + n)]
    mb.mesh(P, F, "fracture", smooth=True)
    for _ in range(10 + var * 2):
        s = rng.uniform(0.15, 0.55)
        m = rng.choice(["fracture", "brick", "plaster", "concrete"], p=[0.35, 0.3, 0.2, 0.15])
        mb.merge(rock_mesh(rng, s * 1.5, s, s * 0.5, m), Rz(rng.uniform(0, 360)) @ Rx(rng.uniform(-25, 25)),
                 (rng.normal(0, sx * 0.25), rng.normal(0, sy * 0.25), h * 0.3 + rng.uniform(0, h * 0.5)))
    P2, F2 = RB.slab(rng, rng.uniform(0.9, 1.6), rng.uniform(0.6, 1.0), T=0.14)
    mb.merge(MB(rng).mesh(P2, F2, "plaster"), Rx(rng.uniform(-30, 30)) @ Rz(rng.uniform(0, 360)), (0.2, 0.0, h * 0.7))
    return mb


def sheet_module(rng, var):
    """Corrugated roof sheet (2.2 x 0.9 m), flat / bent / crumpled, lying on the ground."""
    mb = MB(rng)
    nx, ny = 24, 6
    xs, ys = np.linspace(-1.1, 1.1, nx), np.linspace(-0.45, 0.45, ny)
    X, Y = np.meshgrid(xs, ys)
    Z = 0.018 * np.sin(Y / 0.076 * 2 * np.pi)
    if var == 1:
        Z = Z + np.clip(X - 0.2, 0, None) ** 2 * 0.35
    if var == 2:
        Z = Z + 0.15 * np.sin(X * 2.4 + 0.5) * np.sin(Y * 3.0) + 0.08 * np.abs(np.sin(X * 4.0))
    Z = Z + 0.03
    P = np.c_[X.ravel(), Y.ravel(), Z.ravel()]
    F = []
    for j in range(ny - 1):
        for i in range(nx - 1):
            a = j * nx + i
            F += [(a, a + 1, a + nx + 1), (a, a + nx + 1, a + nx)]
    F = np.array(F)
    mb.mesh(P, F, "metal")
    mb.mesh(P - [0, 0, 0.004], F[:, ::-1], "rust")
    return mb


def litter_module(rng, var):
    """Scattered paper, plastic, cloth, a tarp or a mattress (flat, < 0.25 m)."""
    mb = MB(rng)
    n = 14 + var * 4
    for _ in range(n):
        m = rng.choice(["paper", "plastic", "tarp", "wood", "paper", "mattress"], p=[0.35, 0.2, 0.1, 0.15, 0.1, 0.1])
        if m == "mattress" and var != 2:
            m = "paper"
        w, h = (rng.uniform(0.15, 0.5), rng.uniform(0.1, 0.4)) if m not in ("mattress", "wood") else \
            ((1.9, 0.9) if m == "mattress" else (rng.uniform(0.8, 1.6), 0.1))
        c = np.array([rng.normal(0, 0.7), rng.normal(0, 0.7), 0.02 + rng.uniform(0, 0.04)])
        R = Rz(rng.uniform(0, 360)) @ Rx(rng.normal(0, 8))
        if m == "mattress":
            mb.obox(c + [0, 0, 0.08], (w / 2, h / 2, 0.09), R, m)
        elif m == "wood":
            mb.obox(c + [0, 0, 0.03], (w / 2, 0.05, 0.02), R, m)
        else:
            q = np.array([[-w / 2, -h / 2, 0], [w / 2, -h / 2, 0.02], [w / 2, h / 2, 0], [-w / 2, h / 2, 0.03]]) @ R.T + c
            mb.poly(q, m)
            mb.poly(q[::-1] - [0, 0, 0.002], m)
    return mb


def manhole_module(rng, var):
    mb = MB(rng)
    mb.cyl((0, 0, -0.02), 0.42, 0.03, 16, "frame_dark")
    mb.cyl((0, 0, -0.02), 0.34, 0.035, 16, "steel", caps=(False, True))
    return mb


def post_module(rng, var):
    """Cordon post: weighted base, 1.1 m pole."""
    mb = MB(rng)
    mb.cyl((0, 0, 0), 0.18, 0.08, 10, "plastic", r2=0.14)
    mb.cyl((0, 0, 0.08), 0.025, 1.0, 6, "post")
    return mb


def bldg_module(rng, var):
    A, b = build_bldg(rng, var)
    return A


KIT = {"bldg": (bldg_module, len(TW.BLDG)), "car": (car_module, len(TW.CARS)), "truck": (truck_module, 2),
       "wreck": (wreck_module, 2), "pole": (pole_module, 2), "light": (light_module, 2), "tree": (tree_module, 3),
       "wall": (wall_module, 6), "gate": (gate_module, 2), "stub": (stub_module, 3), "debris": (debris_module, 4),
       "sheet": (sheet_module, 3), "litter": (litter_module, 3), "manhole": (manhole_module, 1), "post": (post_module, 1)}


def _cache(out):
    import hashlib
    import tempfile
    d = os.path.join(tempfile.gettempdir(), "town_kit_build_" + hashlib.md5(out.encode()).hexdigest()[:8])
    os.makedirs(d, exist_ok=True)
    return os.path.join(d, "kit_geo.npz")


def build_kit(out, seed=7, log=print):
    rng = np.random.default_rng(seed)
    mods, dims = {}, {}
    total = 0
    for kind, (fn, nvar) in KIT.items():
        for v in range(nvar):
            r = fn(rng, v)
            A = r if isinstance(r, dict) else r.arrays()
            name = f"{kind}_{v}"
            mods[name] = A
            P = A["P"]
            lo, hi = P.min(0), P.max(0)
            rad = float(np.hypot(np.abs(P[:, 0]).max(), np.abs(P[:, 1]).max())) if kind != "bldg" else 0.0
            dims[name] = {"r": round(rad, 3), "top": round(float(hi[2]), 3), "lo": lo.round(3).tolist(),
                          "hi": hi.round(3).tolist(), "tris": int(len(A["F"]))}
            total += len(A["F"])
            log(f"[town-kit] {name}: {len(A['F'])} tris, mats {sorted(set(A['M'].tolist()))}, "
                f"bounds {lo.round(2)} .. {hi.round(2)}")
    log(f"[town-kit] {len(mods)} modules, {total} triangles")
    os.makedirs(out, exist_ok=True)
    np.savez_compressed(_cache(out), **{f"{k}__{a}": v[a] for k, v in mods.items() for a in ("P", "F", "N", "UV", "M")})
    return mods, dims


# ================================================================================================ USD
def write_usd(out, log=print):
    """town_kit.usdc: /TownKit/<module> (Xform) / geo (Mesh, GeomSubsets per material) + Looks per module, and
    /TownKit/Looks with the ground materials (isaac/town.py references it)."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt
    meta = json.load(open(os.path.join(out, "kit.json")))
    Z = np.load(_cache(out))
    path = os.path.join(out, "town_kit.usdc")
    if os.path.exists(path):
        os.remove(path)
    st = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = UsdGeom.Xform.Define(st, "/TownKit")
    st.SetDefaultPrim(root.GetPrim())
    root.GetPrim().SetCustomDataByKey("frame", "see isaac/tools/blender_town_kit.py docstring (module frames)")

    def material(mpath, name, tex=None, scale=None):
        mat = UsdShade.Material.Define(st, mpath)
        sh = UsdShade.Shader.Define(st, mpath + "/PBR")
        sh.CreateIdAttr("UsdPreviewSurface")
        tex = tex or (name if name in meta["textures"] else None)
        if tex is None:
            rgb, r, m = CONST[name]
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(r)
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(m)
        else:
            files = meta["textures"][tex]
            rd = UsdShade.Shader.Define(st, mpath + "/st")
            rd.CreateIdAttr("UsdPrimvarReader_float2")
            rd.CreateInput("varname", Sdf.ValueTypeNames.String).Set("st")
            for kind, inp, typ, cs in (("albedo", "diffuseColor", Sdf.ValueTypeNames.Color3f, "sRGB"),
                                       ("rough", "roughness", Sdf.ValueTypeNames.Float, "raw"),
                                       ("normal", "normal", Sdf.ValueTypeNames.Normal3f, "raw")):
                tx = UsdShade.Shader.Define(st, f"{mpath}/{kind}")
                tx.CreateIdAttr("UsdUVTexture")
                tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set("./" + files[kind])
                tx.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set(cs)
                tx.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
                tx.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
                tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
                if kind == "albedo":
                    k = scale or SCALE.get(name, (1.0, 1.0, 1.0))
                    tx.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*k, 1.0))
                    o = tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
                elif kind == "normal":
                    tx.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(2, 2, 2, 1))
                    tx.CreateInput("bias", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(-1, -1, -1, 0))
                    o = tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
                else:
                    o = tx.CreateOutput("r", Sdf.ValueTypeNames.Float)
                sh.CreateInput(inp, typ).ConnectToSource(o)
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return mat

    UsdGeom.Scope.Define(st, "/TownKit/Looks")
    for g, tex in GROUND_LOOKS.items():
        material(f"/TownKit/Looks/{g}", g, tex=tex)
    n_tri = 0
    tinted = []
    for name in meta["modules"]:
        P, F, Nn, UV = (Z[f"{name}__{a}"] for a in ("P", "F", "N", "UV"))
        Ma = np.array([str(m) for m in Z[f"{name}__M"]])
        UsdGeom.Xform.Define(st, f"/TownKit/{name}")
        tm = np.isin(Ma, TINT.get(name.rsplit("_", 1)[0], ()))
        for part, sel in (("shell", ~tm), ("tint", tm)):
            if not sel.any():
                continue
            if part == "tint":
                tinted.append(name)
            fi = np.nonzero(sel)[0]
            Fp = F[fi]
            used, Fl = np.unique(Fp.reshape(-1), return_inverse=True)
            Pp = P[used]
            Fl = Fl.reshape(-1, 3)
            corner = (fi[:, None] * 3 + np.arange(3)[None, :]).reshape(-1)
            root_p = f"/TownKit/{name}/{part}"
            UsdGeom.Xform.Define(st, root_p)
            mesh = UsdGeom.Mesh.Define(st, f"{root_p}/mesh")
            mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(Pp, np.float32)))
            mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(Fl), 3, np.int32)))
            mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.ascontiguousarray(Fl.reshape(-1), np.int32)))
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreateDoubleSidedAttr(name.startswith(("litter", "sheet", "gate", "bldg", "tree")))
            mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(Nn[corner], np.float32)))
            mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
            pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                                         UsdGeom.Tokens.faceVarying)
            pv.Set(Vt.Vec2fArray.FromNumpy(np.ascontiguousarray(UV[corner], np.float32)))
            mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, Pp.min(0))), Gf.Vec3f(*map(float, Pp.max(0)))]))
            mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.3, 0.29, 0.27)]))
            Ms = Ma[fi]
            names = sorted(set(Ms.tolist()))
            for nm in names:
                mat = material(f"{root_p}/Looks/{nm}", nm)
                if len(names) == 1:
                    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
                else:
                    idx = np.nonzero(Ms == nm)[0].astype(np.int32)
                    sub = UsdGeom.Subset.CreateGeomSubset(mesh, nm, UsdGeom.Tokens.face, Vt.IntArray.FromNumpy(idx),
                                                          "materialBind", UsdGeom.Tokens.partition)
                    UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mat)
        n_tri += len(F)
    meta["tinted"] = tinted
    json.dump(meta, open(os.path.join(out, "kit.json"), "w"), indent=1)
    st.GetRootLayer().Save()
    log(f"[town-kit] wrote {path}: {len(meta['modules'])} modules, {n_tri} triangles")
    return path


def _tex_budget(out):
    from PIL import Image
    tot_file, tot_gpu = 0, 0
    for f in sorted(os.listdir(os.path.join(out, "textures"))):
        p = os.path.join(out, "textures", f)
        im = Image.open(p)
        ch = {"L": 1, "RGB": 4, "RGBA": 4}[im.mode]
        tot_file += os.path.getsize(p)
        tot_gpu += im.size[0] * im.size[1] * ch * 4 / 3
    return tot_file, tot_gpu


# ================================================================================================ preview
class Preview:
    """The real grid stage (scene.build + photoreal's rubble / debris + the command van) + the town, converted from
    pxr into Blender and rendered with Cycles CPU from director-take-like views."""

    def __init__(self, kit_dir, town=True, seed=7, log=print):
        self.kit_dir, self.town, self.seed, self.log = kit_dir, town, seed, log
        self.mats, self.protos = {}, {}

    # --- stage
    def stage(self):
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade
        from sim.scenario import make_scenario
        from isaac import scene, photoreal, van
        sc = make_scenario(self.seed)
        st = Usd.Stage.CreateInMemory()
        scene.build(st, sc, materials="preview", photoreal=True)
        fk = os.path.join(REPO, "isaac", "assets", "facade_kit")
        mats = {}

        def pmat(name, tex=None, rgb=(0.5, 0.5, 0.5), rough=0.9, tile=1.5, scale=1.0):
            mp = f"/World/Looks/pv_{name}"
            m = UsdShade.Material.Define(st, mp)
            sh = UsdShade.Shader.Define(st, mp + "/PBR")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
            if tex:
                tx = UsdShade.Shader.Define(st, mp + "/albedo")
                tx.CreateIdAttr("UsdUVTexture")
                tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(tex)
                tx.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(scale, scale, scale, 1))
                sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
                    tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))
                m.GetPrim().SetCustomDataByKey("box_tile", tile)
            else:
                sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            return m
        # stand-ins for Isaac's MDLs (Concrete_Rough / Formed, Brick_Wall_Red / Brown, Dirt, Gravel, Asphalt, Rust):
        # albedo levels read off the Isaac grid frames (light concrete, orange brick, orange dirt)
        mats["concrete"] = pmat("concrete", os.path.join(fk, "textures", "concrete_albedo.png"), tile=2.0, scale=1.45)
        mats["concrete2"] = pmat("concrete2", os.path.join(fk, "textures", "fracture_albedo.png"), tile=1.5, scale=1.25)
        mats["brick"] = pmat("brick", os.path.join(fk, "textures", "brick_albedo.png"), tile=1.0, scale=1.3)
        mats["brick2"] = pmat("brick2", os.path.join(fk, "textures", "brick_albedo.png"), tile=1.2, scale=1.05)
        mats["dirt"] = pmat("dirt", rgb=(0.40, 0.24, 0.13), rough=0.95)
        mats["gravel"] = pmat("gravel", os.path.join(fk, "textures", "fracture_albedo.png"), tile=1.0, scale=0.9)
        mats["asphalt"] = pmat("asphalt", rgb=(0.07, 0.07, 0.07), rough=0.8)
        mats["rust"] = pmat("rust", rgb=(0.16, 0.08, 0.05), rough=0.7)
        for p in ("/World/Terrain", "/World/Apron"):
            photoreal.bind(st, p, mats["dirt"])
        photoreal.rubble_ground(st, sc, mats["gravel"], self.log)
        photoreal.bind(st, "/World/Staging/Pad", mats["asphalt"])
        photoreal.dress_rubble(st, sc, mats, self.log, seed=sc.seed)
        photoreal.scatter_debris(st, sc, mats, self.log, seed=sc.seed)
        if self.town:
            TW.dress(st, sc, self.kit_dir, self.log)
        try:
            vx, vy, vz, vyaw = van.staging_pose(sc)
            van.hide_legacy_staging(st)
            van.build_van(st, "/World/Van", (vx, vy), vyaw, z=vz)
        except Exception as e:
            self.log(f"[preview] van skipped ({type(e).__name__}: {e})")
        self.sc = sc
        return st

    # --- pxr -> bpy
    def bl_material(self, umat, box_tile=None):
        bpy = _bpy()
        from pxr import UsdShade
        key = str(umat.GetPath()) + (f"#box{box_tile}" if box_tile else "")
        if key in self.mats:
            return self.mats[key]
        mat = bpy.data.materials.new(key.rsplit("/", 1)[-1][:50])
        mat.use_nodes = True
        nt = mat.node_tree
        nt.nodes.clear()
        bsdf = nt.nodes.new("ShaderNodeBsdfPrincipled")
        outn = nt.nodes.new("ShaderNodeOutputMaterial")
        nt.links.new(bsdf.outputs[0], outn.inputs["Surface"])
        surf = umat.ComputeSurfaceSource()[0]
        if not surf or surf.GetIdAttr().Get() != "UsdPreviewSurface":
            bsdf.inputs["Base Color"].default_value = (0.4, 0.4, 0.4, 1)
            self.mats[key] = mat
            return mat
        coord = None
        if box_tile:
            tc = nt.nodes.new("ShaderNodeTexCoord")
            mp = nt.nodes.new("ShaderNodeMapping")
            mp.inputs["Scale"].default_value = (1 / box_tile,) * 3
            nt.links.new(tc.outputs["Object"], mp.inputs[0])
            coord = mp.outputs[0]
        else:
            uvn = nt.nodes.new("ShaderNodeUVMap")
            uvn.uv_map = "st"
            coord = uvn.outputs[0]

        def tex_of(inp):
            if not inp or not inp.HasConnectedSource():
                return None
            src = inp.GetConnectedSource()[0]
            sh = UsdShade.Shader(src.GetPrim())
            if sh.GetIdAttr().Get() != "UsdUVTexture":
                return None
            f = sh.GetInput("file").Get()
            path = f.resolvedPath or f.path
            if not path or not os.path.exists(path):
                return None
            sc_ = sh.GetInput("scale")
            sc_ = tuple(sc_.Get()) if sc_ and sc_.Get() is not None else (1, 1, 1, 1)
            cs = sh.GetInput("sourceColorSpace")
            cs = cs.Get() if cs else "auto"
            im = bpy.data.images.load(path, check_existing=True)
            node = nt.nodes.new("ShaderNodeTexImage")
            node.image = im
            if box_tile:
                node.projection = "BOX"
                node.projection_blend = 0.2
            nt.links.new(coord, node.inputs[0])
            return node, sc_, cs

        def const(name, default):
            i = surf.GetInput(name)
            if i and not i.HasConnectedSource() and i.Get() is not None:
                return i.Get()
            return default
        d = tex_of(surf.GetInput("diffuseColor"))
        if d:
            node, sc_, cs = d
            node.image.colorspace_settings.name = "sRGB"
            mix = nt.nodes.new("ShaderNodeMix")
            mix.data_type, mix.blend_type = "RGBA", "MULTIPLY"
            ins = {s.identifier: s for s in mix.inputs}
            ins["Factor_Float"].default_value = 1.0
            nt.links.new(node.outputs[0], ins["A_Color"])
            ins["B_Color"].default_value = (sc_[0], sc_[1], sc_[2], 1)
            nt.links.new({s.identifier: s for s in mix.outputs}["Result_Color"], bsdf.inputs["Base Color"])
        else:
            c = const("diffuseColor", (0.18, 0.18, 0.18))
            bsdf.inputs["Base Color"].default_value = (*c, 1)
        r = tex_of(surf.GetInput("roughness"))
        if r:
            r[0].image.colorspace_settings.name = "Non-Color"
            nt.links.new(r[0].outputs[0], bsdf.inputs["Roughness"])
        else:
            bsdf.inputs["Roughness"].default_value = float(const("roughness", 0.8))
        bsdf.inputs["Metallic"].default_value = float(const("metallic", 0.0))
        nn = tex_of(surf.GetInput("normal"))
        if nn and not box_tile:
            nn[0].image.colorspace_settings.name = "Non-Color"
            nm = nt.nodes.new("ShaderNodeNormalMap")
            nm.uv_map = "st"
            nt.links.new(nn[0].outputs[0], nm.inputs["Color"])
            nt.links.new(nm.outputs[0], bsdf.inputs["Normal"])
        em = const("emissiveColor", (0, 0, 0))
        if max(em) > 0:
            bsdf.inputs["Emission Color"].default_value = (*em, 1)
            bsdf.inputs["Emission Strength"].default_value = 1.0
        op = const("opacity", 1.0)
        if op < 1.0:
            bsdf.inputs["Alpha"].default_value = float(op)
        self.mats[key] = mat
        return mat

    def bl_mesh(self, prim, name):
        """bpy mesh from a UsdGeom.Mesh / Cube (local coordinates), with st, normals and material slots."""
        bpy = _bpy()
        from pxr import Usd, UsdGeom, UsdShade
        if prim.IsA(UsdGeom.Cube):
            s = UsdGeom.Cube(prim).GetSizeAttr().Get() or 2.0
            h = s / 2
            P = np.array([(-h, -h, -h), (h, -h, -h), (h, h, -h), (-h, h, -h), (-h, -h, h), (h, -h, h), (h, h, h), (-h, h, h)])
            faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (2, 3, 7, 6), (3, 0, 4, 7), (1, 2, 6, 5)]
            me = bpy.data.meshes.new(name)
            me.from_pydata(P.tolist(), [], faces)
            bm = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
            if bm:
                me.materials.append(self.bl_material(bm, box_tile=1.0))
            return me
        m = UsdGeom.Mesh(prim)
        P = np.asarray(m.GetPointsAttr().Get(), np.float64)
        cnt = np.asarray(m.GetFaceVertexCountsAttr().Get(), np.int64)
        idx = np.asarray(m.GetFaceVertexIndicesAttr().Get(), np.int64)
        if not len(P) or not len(cnt):
            return None
        me = bpy.data.meshes.new(name)
        starts = np.concatenate([[0], np.cumsum(cnt)[:-1]])
        if (cnt == 3).all():
            me.vertices.add(len(P))
            me.vertices.foreach_set("co", P.astype(np.float32).ravel())
            me.loops.add(len(idx))
            me.loops.foreach_set("vertex_index", idx.astype(np.int32))
            me.polygons.add(len(cnt))
            me.polygons.foreach_set("loop_start", starts.astype(np.int32))
            me.update(calc_edges=True)
        else:
            me.from_pydata(P.tolist(), [], [idx[s:s + c].tolist() for s, c in zip(starts, cnt)])
        pv = UsdGeom.PrimvarsAPI(m).GetPrimvar("st")
        has_st = pv and pv.HasValue()
        if has_st:
            uv = np.asarray(pv.ComputeFlattened(), np.float32)
            interp = pv.GetInterpolation()
            if interp == "vertex":
                uv = uv[idx]
            if len(uv) == len(idx):
                lay = me.uv_layers.new(name="st")
                lay.data.foreach_set("uv", uv.ravel())
        bad = me.validate(clean_customdata=False)
        nrm = m.GetNormalsAttr().Get()
        if nrm is not None and len(nrm) and not bad:
            Nn = np.asarray(nrm, np.float32)
            interp = m.GetNormalsInterpolation()
            try:
                if interp == "vertex" and len(Nn) == len(P):
                    me.normals_split_custom_set_from_vertices(Nn.tolist())
                elif len(Nn) == len(idx):
                    me.normals_split_custom_set(Nn.tolist())
            except Exception:
                pass
        # materials: subsets or the prim binding
        subs = UsdGeom.Subset.GetGeomSubsets(m, UsdGeom.Tokens.face, "materialBind")
        box = None if has_st else 1.5
        if subs:
            mi = np.zeros(len(cnt), np.int32)
            for k, sub in enumerate(subs):
                bm = UsdShade.MaterialBindingAPI(sub.GetPrim()).ComputeBoundMaterial()[0]
                me.materials.append(self.bl_material(bm, box) if bm else None)
                ii = np.asarray(sub.GetIndicesAttr().Get(), np.int64)
                mi[ii] = k
            me.polygons.foreach_set("material_index", mi)
        else:
            bm = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
            if bm:
                tile = bm.GetPrim().GetCustomDataByKey("box_tile")
                me.materials.append(self.bl_material(bm, tile if (tile and not has_st) else box))
        me.update()
        return me

    def convert(self, st):
        """Every mesh / cube outside the instancers -> objects; PointInstancers -> collection instances."""
        bpy = _bpy()
        from mathutils import Matrix
        from pxr import Usd, UsdGeom
        sc = bpy.context.scene
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        n_obj = n_inst = 0
        protos_coll = bpy.data.collections.new("protos")
        sc.collection.children.link(protos_coll)
        protos_coll.hide_render = False
        it = iter(Usd.PrimRange(st.GetPseudoRoot()))
        for prim in it:
            if not prim.IsActive():
                it.PruneChildren()
                continue
            if prim.IsA(UsdGeom.Imageable):
                vis = UsdGeom.Imageable(prim).ComputeVisibility()
                if vis == UsdGeom.Tokens.invisible:
                    it.PruneChildren()
                    continue
            if prim.IsA(UsdGeom.PointInstancer):
                self._instancer(prim, cache, protos_coll)
                n_inst += 1
                it.PruneChildren()
                continue
            if prim.GetName() in ("Looks",) or prim.IsA(UsdGeom.Camera):
                it.PruneChildren()
                continue
            if prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Cube):
                me = self.bl_mesh(prim, prim.GetName())
                if me is None:
                    continue
                ob = bpy.data.objects.new(prim.GetName(), me)
                M = cache.GetLocalToWorldTransform(prim)
                ob.matrix_world = Matrix([[M[c][r] for c in range(4)] for r in range(4)])
                sc.collection.objects.link(ob)
                n_obj += 1
        self.log(f"[preview] converted {n_obj} meshes, {n_inst} instancers")

    def _instancer(self, prim, cache, protos_coll):
        bpy = _bpy()
        from mathutils import Matrix
        from pxr import Usd, UsdGeom
        pi = UsdGeom.PointInstancer(prim)
        targets = pi.GetPrototypesRel().GetTargets()
        colls = []
        for t in targets:
            pp = prim.GetStage().GetPrimAtPath(t)
            coll = bpy.data.collections.new(t.name[:60])
            protos_coll.children.link(coll)
            base = cache.GetLocalToWorldTransform(pp).GetInverse()
            for q in Usd.PrimRange(pp, Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate)):
                if q.IsA(UsdGeom.Mesh):
                    me = self.bl_mesh(q, q.GetName())
                    if me is None:
                        continue
                    ob = bpy.data.objects.new(q.GetName(), me)
                    M = cache.GetLocalToWorldTransform(q) * base
                    ob.matrix_world = Matrix([[M[c][r] for c in range(4)] for r in range(4)])
                    coll.objects.link(ob)
            colls.append(coll)
        protos_coll.hide_render = True
        for c in colls:
            c.hide_render = False
        xs = pi.ComputeInstanceTransformsAtTime(Usd.TimeCode.Default(), Usd.TimeCode.Default())
        idx = pi.GetProtoIndicesAttr().Get()
        W = cache.GetLocalToWorldTransform(prim)
        sc = bpy.context.scene
        for k, (M, pi_) in enumerate(zip(xs, idx)):
            Mw = M * W
            e = bpy.data.objects.new(f"{prim.GetName()}_{k}", None)
            e.instance_type = "COLLECTION"
            e.instance_collection = colls[pi_]
            e.matrix_world = Matrix([[Mw[c][r] for c in range(4)] for r in range(4)])
            sc.collection.objects.link(e)

    # --- scene + shots
    def setup(self, samples=40, res=(1280, 720)):
        bpy = _bpy()
        sc = bpy.context.scene
        sc.render.engine = "CYCLES"
        sc.cycles.device = "CPU"
        sc.cycles.samples = samples
        sc.cycles.use_denoising = True
        sc.cycles.max_bounces = 4
        sc.cycles.diffuse_bounces = 2
        sc.cycles.glossy_bounces = 2
        sc.cycles.transparent_max_bounces = 2
        sc.render.resolution_x, sc.render.resolution_y = res
        sc.render.film_transparent = False
        sc.view_settings.view_transform = "AgX"
        looks = [i.identifier for i in sc.view_settings.bl_rna.properties["look"].enum_items]
        sc.view_settings.look = next((l for l in looks if "Medium High" in l), "None")
        sc.view_settings.exposure = 0.0
        w = bpy.data.worlds.new("w")
        sc.world = w
        w.use_nodes = True
        nt = w.node_tree
        nt.nodes.clear()
        tc = nt.nodes.new("ShaderNodeTexCoord")
        sep = nt.nodes.new("ShaderNodeSeparateXYZ")
        nt.links.new(tc.outputs["Generated"], sep.inputs[0])
        ramp = nt.nodes.new("ShaderNodeValToRGB")
        cr = ramp.color_ramp
        cr.elements[0].position, cr.elements[0].color = 0.5, (0.62, 0.56, 0.48, 1)
        cr.elements[1].position, cr.elements[1].color = 1.0, (0.36, 0.45, 0.60, 1)
        e = cr.elements.new(0.56)
        e.color = (0.66, 0.62, 0.56, 1)
        nt.links.new(sep.outputs["Z"], ramp.inputs[0])
        bg = nt.nodes.new("ShaderNodeBackground")
        nt.links.new(ramp.outputs[0], bg.inputs[0])
        bg.inputs[1].default_value = 0.85
        wo = nt.nodes.new("ShaderNodeOutputWorld")
        nt.links.new(bg.outputs[0], wo.inputs[0])
        sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
        sun.data.energy = 4.2
        sun.data.color = (1.0, 0.82, 0.64)
        sun.data.angle = math.radians(0.8)
        # photoreal.sky_and_sun: from the south-west, 24 deg elevation (light travels north-east and down)
        el, az_from = math.radians(24.0), math.radians(225.0)
        d = -np.array([math.cos(el) * math.cos(az_from), math.cos(el) * math.sin(az_from), math.sin(el)])
        from mathutils import Vector
        sun.rotation_euler = Vector(d.tolist()).to_track_quat("-Z", "Y").to_euler()
        sc.collection.objects.link(sun)
        cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
        cam.data.lens = 18.0                                     # isaac/camera.py DirectorCamera: 18 mm on 20.955 mm
        cam.data.sensor_width = 20.955
        cam.data.sensor_fit = "HORIZONTAL"
        cam.data.clip_start, cam.data.clip_end = 0.1, 3000
        sc.collection.objects.link(cam)
        sc.camera = cam
        self.cam = cam

    def shoot(self, eye, look, path, lens=18.0):
        bpy = _bpy()
        from mathutils import Vector
        d = Vector(look) - Vector(eye)
        self.cam.location = eye
        self.cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
        self.cam.data.lens = lens
        sc = bpy.context.scene
        sc.render.filepath = path
        bpy.ops.render.render(write_still=True)
        self.log(f"[preview] {path}")
        return path


def shots(sc):
    """Director-take-like views: (name, eye, look, lens)."""
    P = {p.name: p for p in sc.piles}
    g = lambda x, y: float(np.asarray(sc.ground_height(np.array([x]), np.array([y]))).ravel()[0])
    a2 = P["A2"]
    pz = g(a2.x, a2.y)
    az = math.radians(-35.0)
    eye_a2 = (a2.x + 32 * math.cos(az), a2.y + 32 * math.sin(az), pz + 1.5 + 15.0)
    return [
        ("wide_se", (52.0, -58.0, 46.0), (-2.0, 2.0, 0.0), 18.0),
        ("a2_mid", eye_a2, (a2.x, a2.y, pz + 1.5), 18.0),
        ("skyline_low", (12.0, -14.0, 6.5), (60.0, 40.0, 9.0), 18.0),
        ("survey_b1", (0.4 + 42 * math.cos(math.radians(130)), 22.0 + 42 * math.sin(math.radians(130)), 28.0),
         (0.4, 22.0, 3.0), 18.0),
        ("van", (2.0, -56.2, 1.7), (8.3, -48.6, 2.0), 18.0),
    ]


def preview(out, dest, log=print, samples=40, res=(1280, 720), town=True, only=None, tag=""):
    bpy = _bpy()
    bpy.ops.wm.read_factory_settings(use_empty=True)
    pv = Preview(out, town=town, log=log)
    st = pv.stage()
    pv.setup(samples, res)
    pv.convert(st)
    os.makedirs(dest, exist_ok=True)
    outs = []
    for name, eye, look, lens in shots(pv.sc):
        if only and name not in only:
            continue
        fn = os.path.join(dest, f"town_{name}{tag}.png")
        outs.append(pv.shoot(eye, look, fn, lens))
    return outs


# ================================================================================================ main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="kit folder (town_kit.usdc + kit.json + textures/)")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--stage", choices=["all", "build", "geo", "usd", "preview"], default="all",
                    help="geo = rebuild the module geometry only (keep the baked textures)")
    ap.add_argument("--pxr-python", default="", help="python with usd-core, if pxr is not importable here")
    ap.add_argument("--preview", default="", help="folder for Cycles preview renders (needs bpy + pxr)")
    ap.add_argument("--shots", default="", help="comma list of preview shots (default: all)")
    ap.add_argument("--tag", default="", help="suffix for the preview file names (e.g. _iter1)")
    ap.add_argument("--no-town", action="store_true", help="preview the stage without the town (before)")
    ap.add_argument("--samples", type=int, default=40)
    ap.add_argument("--res", default="1280x720")
    ap.add_argument("--textures", default="", help="comma list: bake only these texture sets")
    a = ap.parse_args(argv)
    out = os.path.abspath(a.out)
    os.makedirs(out, exist_ok=True)
    if a.stage in ("all", "build", "geo"):
        meta_p = os.path.join(out, "kit.json")
        meta = json.load(open(meta_p)) if os.path.exists(meta_p) else {}
        if a.stage != "geo":
            bpy = _bpy()
            bpy.ops.wm.read_factory_settings(use_empty=True)
            tex = bake_textures(out, only=set(a.textures.split(",")) if a.textures else None)
        else:
            tex = meta["textures"]
        mods, dims = build_kit(out, a.seed)
        fs, gpu = _tex_budget(out)
        json.dump({"modules": list(mods.keys()), "textures": tex, "seed": a.seed, "dims": dims,
                   "uv_tile": UV_TILE, "tile_m": TILE, "triangles": int(sum(len(m["F"]) for m in mods.values())),
                   "texture_bytes": {"files": int(fs), "gpu_rgba8_mips": int(gpu)},
                   "frame": "see isaac/tools/blender_town_kit.py"},
                  open(meta_p, "w"), indent=1)
        print(f"[town-kit] textures: {fs / 1e6:.1f} MB files, ~{gpu / 1e6:.1f} MB GPU (RGBA8 + mips)")
    if a.stage in ("all", "build", "geo", "usd"):
        try:
            import pxr  # noqa: F401
            ok = True
        except Exception:
            ok = False
        if ok:
            write_usd(out)
        elif a.pxr_python:
            subprocess.check_call([a.pxr_python, os.path.abspath(__file__), "--out", out, "--stage", "usd"])
        else:
            print("[town-kit] pxr not importable: run again with --stage usd under a python with usd-core")
    if a.preview and a.stage in ("all", "preview"):
        w, h = map(int, a.res.split("x"))
        preview(out, a.preview, samples=a.samples, res=(w, h), town=not a.no_town,
                only=set(a.shots.split(",")) if a.shots else None, tag=a.tag)


if __name__ == "__main__":
    main()
