"""Damaged-facade module kit for the fallen Weiguan Jinlong towers (--site collapse), built in Blender (headless).

    pip install bpy pillow numpy          # bpy 5.x wheel, Python 3.13
    python blender_facade_kit.py --out isaac/assets/facade_kit [--pxr-python /path/to/python-with-usd-core]
    python blender_facade_kit.py --out DIR --preview /mnt/project-files/larpverse/blender/   # + Cycles CPU renders

Builds broken, chipped geometry (booleans with noisy cutters, voxel remesh, fracture displacement, decimate),
bakes procedural Cycles materials (noise / voronoi / brick + dirt masks) to shared PNG texture sets, and writes
one USD (facade_kit.usdc) with one prim per module and UsdPreviewSurface materials that reference the PNGs by
relative path (textures/*.png). The USD step needs pxr: in-process if importable, else --pxr-python.

Module frame (metres, Z-up file, same in Blender and USD):
    x = U  across the face, one bay      (unit bay 3.0 m: x in [-1.5, 1.5])
    y = V  along the fallen tower, storey (unit storey 2.9 m: y in [-1.45, 1.45]; slab line at y = 0 for slabs)
    z = N  outward from the scanned face plane (z = 0 is the face plane, everything sits at z >= 0)
    origin: bay centre on the face plane (slab_*: bay centre on the storey line; column_*/beam_*: bay line;
            debris_*: base centre). The placer scales x by bay/3.0 and y by storey/2.9.
Depth layout: slab fronts at z ~0.95, column fronts ~0.85, tiled wall faces ~0.78, crushed-storey backs at 0.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys

import numpy as np

BAY, STOREY = 3.0, 2.9
SLAB_T = 0.50                       # slab + beam edge thickness (V)
GAP = (STOREY - SLAB_T) / 2         # half height of the storey between slabs (1.2)
TEX = {"concrete": 1024, "fracture": 1024, "tile_a": 1024, "tile_b": 1024, "interior": 512, "brick": 512}
# albedo multiplier per texture set in the USD (UsdUVTexture inputs:scale) and the previews: the tiles bake
# ~0.45 linear, Isaac's fixed-exposure grade read 0.34 flat tiles near-white in sun (urban_look.TILE_TINTS), so
# land at ~0.35 / ~0.31. Retune without re-baking: edit and run --stage usd.
ALBEDO_SCALE = {"tile_a": 0.74, "tile_b": 0.70}
# constant (untextured) materials: rgb (linear), roughness, metallic
CONST = {"alu": ((0.50, 0.50, 0.49), 0.45, 0.85), "rust": ((0.16, 0.085, 0.05), 0.75, 0.25),
         "wire": ((0.035, 0.035, 0.035), 0.55, 0.0), "glass": ((0.03, 0.04, 0.045), 0.08, 0.0)}


# ================================================================================================ Blender side
def _bpy():
    import bpy
    return bpy


class Nodes:
    """Tiny helper for building shader node trees."""

    def __init__(self, mat):
        self.t = mat.node_tree
        self.t.nodes.clear()
        self.y = 0

    def n(self, kind, **kw):
        nd = self.t.nodes.new(kind)
        for k, v in kw.items():
            setattr(nd, k, v)
        nd.location = (0, self.y)
        self.y -= 200
        return nd

    def _in(self, sock, v):
        if hasattr(v, "is_output") or (hasattr(v, "node") and hasattr(v, "identifier")):
            self.t.links.new(v, sock)
        elif isinstance(v, (tuple, list)):
            sock.default_value = tuple(v) + ((1.0,) if len(v) == 3 and len(sock.default_value) == 4 else ())
        else:
            sock.default_value = v

    def math(self, op, a, b=None, clamp=False):
        nd = self.n("ShaderNodeMath", operation=op, use_clamp=clamp)
        self._in(nd.inputs[0], a)
        if b is not None:
            self._in(nd.inputs[1], b)
        return nd.outputs[0]

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

    def vmul(self, v, s):
        if isinstance(s, (int, float)):
            s = (float(s),) * 3
        nd = self.n("ShaderNodeVectorMath", operation="MULTIPLY")
        self._in(nd.inputs[0], v)
        self._in(nd.inputs[1], s)
        return nd.outputs[0]

    def vadd(self, a, b):
        nd = self.n("ShaderNodeVectorMath", operation="ADD")
        self._in(nd.inputs[0], a)
        self._in(nd.inputs[1], b)
        return nd.outputs[0]

    def noise(self, vec, scale, detail=4.0, rough=0.55, dist=0.0, kind="FBM", seed=0.0):
        nd = self.n("ShaderNodeTexNoise", noise_type=kind, noise_dimensions="4D")
        self._in(nd.inputs["Vector"], vec)
        nd.inputs["W"].default_value = seed
        nd.inputs["Scale"].default_value = scale
        nd.inputs["Detail"].default_value = detail
        nd.inputs["Roughness"].default_value = rough
        nd.inputs["Distortion"].default_value = dist
        return nd.outputs["Fac"], nd.outputs["Color"]

    def voronoi(self, vec, scale, feature="F1", rnd=1.0, seed=0.0):
        nd = self.n("ShaderNodeTexVoronoi", feature=feature, voronoi_dimensions="4D")
        self._in(nd.inputs["Vector"], vec)
        nd.inputs["W"].default_value = seed
        nd.inputs["Scale"].default_value = scale
        nd.inputs["Randomness"].default_value = rnd
        return nd

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

    def band(self, f, lo, hi):
        """smoothstep-ish 0..1 from lo to hi (map range, smoothstep)."""
        nd = self.n("ShaderNodeMapRange", interpolation_type="SMOOTHSTEP", clamp=True)
        self._in(nd.inputs["Value"], f)
        nd.inputs["From Min"].default_value = lo
        nd.inputs["From Max"].default_value = hi
        return nd.outputs["Result"]

    def coord(self, scale=(1.0, 1.0, 1.0)):
        tc = self.n("ShaderNodeTexCoord")
        return self.vmul(tc.outputs["Object"], scale)


def _dust(T, p, col, amount=0.35, seed=7.0):
    """Post-collapse grey-brown dust layer, patchy, heavier in large blotches."""
    f1, _ = T.noise(p, 1.1, 5, 0.6, seed=seed)
    f2, _ = T.noise(p, 9.0, 3, 0.5, seed=seed + 1)
    m = T.band(T.math("ADD", T.math("MULTIPLY", f1, 0.8), T.math("MULTIPLY", f2, 0.25)), 0.42, 0.78)
    m = T.math("MULTIPLY", m, amount)
    return T.mix(m, col, (0.40, 0.375, 0.33)), m


def mat_concrete(T, seed=0.0):
    """Weathered, dusty reinforced concrete (slab edges, columns): warm grey ~0.25-0.38, water stains."""
    p = T.coord()
    f0, _ = T.noise(p, 0.55, 6, 0.6, seed=seed)
    base = T.ramp(f0, [(0.30, (0.22, 0.212, 0.195)), (0.55, (0.31, 0.295, 0.27)), (0.75, (0.37, 0.35, 0.32))])
    f1, _ = T.noise(p, 7.0, 4, 0.6, seed=seed + 3)
    base = T.mix(0.35, base, T.ramp(f1, [(0.3, (0.6, 0.6, 0.6)), (0.7, (1.0, 0.98, 0.95))]), "MULTIPLY")
    # water stains: long streaks along V (y), clustered
    fs, _ = T.noise(T.vmul(p, (9.0, 0.35, 9.0)), 1.0, 4, 0.55, seed=seed + 5)
    fc, _ = T.noise(p, 0.7, 2, 0.5, seed=seed + 6)
    st = T.math("MULTIPLY", T.band(fs, 0.52, 0.72), T.band(fc, 0.42, 0.62))
    base = T.mix(T.math("MULTIPLY", st, 0.55), base, (0.11, 0.105, 0.095))
    # pores / small spalls
    vor = T.voronoi(p, 55.0)
    pore = T.math("SUBTRACT", 1.0, T.band(vor.outputs["Distance"], 0.05, 0.16))
    fp, _ = T.noise(p, 3.0, 2, 0.5, seed=seed + 8)
    pore = T.math("MULTIPLY", pore, T.band(fp, 0.45, 0.65))
    base = T.mix(T.math("MULTIPLY", pore, 0.6), base, (0.12, 0.115, 0.105))
    col, dm = _dust(T, p, base, 0.45, seed + 11)
    rough = T.math("ADD", 0.84, T.math("MULTIPLY", f1, 0.1))
    rough = T.mixf(st, rough, 0.72)
    rough = T.mixf(dm, rough, 0.97)
    f3, _ = T.noise(p, 30.0, 3, 0.6, seed=seed + 13)
    h = T.math("ADD", T.math("MULTIPLY", f3, 0.35), T.math("MULTIPLY", f0, 0.5))
    h = T.math("SUBTRACT", h, T.math("MULTIPLY", pore, 0.5))
    return col, rough, h, 0.010


def mat_fracture(T, seed=20.0):
    """Fresh broken concrete: lighter paste with grey/brown aggregate, very rough, strong relief."""
    p = T.coord()
    f0, _ = T.noise(p, 2.0, 5, 0.6, seed=seed)
    paste = T.ramp(f0, [(0.3, (0.34, 0.325, 0.30)), (0.7, (0.43, 0.41, 0.375))])
    pd = T.vadd(p, T.vmul(T.noise(p, 6.0, 2, 0.5, seed=seed + 1)[1], 0.012))
    vor = T.voronoi(pd, 32.0)
    stone = T.math("SUBTRACT", 1.0, T.band(vor.outputs["Distance"], 0.28, 0.40))
    sep = T.n("ShaderNodeSeparateColor")
    T.t.links.new(vor.outputs["Color"], sep.inputs[0])
    sc = T.ramp(sep.outputs[0], [(0.0, (0.17, 0.165, 0.16)), (0.35, (0.30, 0.29, 0.28)), (0.6, (0.27, 0.22, 0.17)),
                                 (0.85, (0.42, 0.40, 0.37)), (1.0, (0.22, 0.20, 0.19))])
    base = T.mix(T.math("MULTIPLY", stone, 0.85), paste, sc)
    f2, _ = T.noise(p, 14.0, 3, 0.6, seed=seed + 2)
    base = T.mix(0.3, base, T.ramp(f2, [(0.3, (0.7, 0.7, 0.7)), (0.7, (1.0, 1.0, 1.0))]), "MULTIPLY")
    col, dm = _dust(T, p, base, 0.4, seed + 5)
    rough = T.math("ADD", 0.9, T.math("MULTIPLY", f2, 0.08))
    f3, _ = T.noise(p, 22.0, 5, 0.65, seed=seed + 7)
    h = T.math("ADD", T.math("MULTIPLY", stone, 0.45), T.math("MULTIPLY", f3, 0.7))
    h = T.math("ADD", h, T.math("MULTIPLY", f0, 0.6))
    return col, rough, h, 0.020


def mat_tile(T, dirty=0.0, seed=40.0):
    """Taiwan apartment facade: small beige / off-white ceramic tiles (95 x 45 mm, 6 mm grout) with grime
    streaks, missing tiles (cement bed showing) and cracks. Mapped once over a 3.0 x 2.9 m bay."""
    p = T.coord()
    br = T.n("ShaderNodeTexBrick", offset=0.0, offset_frequency=2, squash=1.0, squash_frequency=2)
    T._in(br.inputs["Vector"], p)
    br.inputs["Color1"].default_value = (0, 0, 0, 1)
    br.inputs["Color2"].default_value = (1, 1, 1, 1)
    br.inputs["Mortar"].default_value = (2, 2, 2, 1)
    br.inputs["Scale"].default_value = 1.0
    br.inputs["Mortar Size"].default_value = 0.0055
    br.inputs["Mortar Smooth"].default_value = 0.15
    br.inputs["Bias"].default_value = 0.0
    br.inputs["Brick Width"].default_value = 0.101
    br.inputs["Row Height"].default_value = 0.051
    sep = T.n("ShaderNodeSeparateColor")
    T.t.links.new(br.outputs["Color"], sep.inputs[0])
    r = T.math("MINIMUM", sep.outputs[0], 1.0)                    # per-tile random 0..1 (mortar clipped)
    grout = br.outputs["Factor"]                                 # 1 in the grout lines
    tc = T.ramp(r, [(0.0, (0.60, 0.49, 0.36)), (0.5, (0.67, 0.56, 0.42)), (0.8, (0.56, 0.46, 0.34)),
                    (1.0, (0.71, 0.61, 0.48))])
    col = T.mix(grout, tc, (0.38, 0.355, 0.32))                  # light cement grout (no plank stripes at range)
    # large-scale discolouration
    f0, _ = T.noise(p, 0.8, 4, 0.55, seed=seed)
    col = T.mix(0.5, col, T.ramp(f0, [(0.3, (0.78, 0.76, 0.72)), (0.7, (1.0, 1.0, 1.0))]), "MULTIPLY")
    # grime streaks running down the original facade (along -V = -y)
    fs, _ = T.noise(T.vmul(p, (11.0, 1.3, 11.0)), 1.0, 5, 0.6, seed=seed + 2)   # short, sparse (no wood grain)
    fc, _ = T.noise(p, 0.9, 2, 0.5, seed=seed + 3)
    grad = T.n("ShaderNodeSeparateXYZ")
    T._in(grad.inputs[0], p)
    low = T.band(grad.outputs["Y"], 1.2, -1.4)                    # more grime towards the bottom of the bay
    streak = T.math("MULTIPLY", T.band(fs, 0.57 - 0.04 * dirty, 0.76), T.band(fc, 0.45 - 0.1 * dirty, 0.68))
    streak = T.math("ADD", streak, T.math("MULTIPLY", low, 0.18 + 0.15 * dirty), clamp=True)
    col = T.mix(T.math("MULTIPLY", streak, 0.45 + 0.15 * dirty), col, (0.20, 0.18, 0.15), "MULTIPLY")
    col = T.mix(T.math("MULTIPLY", streak, 0.25), col, (0.16, 0.15, 0.13))
    # cracks (distorted voronoi edges) and missing tiles clustered near cracks / patches
    pd = T.vadd(p, T.vmul(T.noise(p, 2.2, 3, 0.6, seed=seed + 4)[1], 0.18))
    vo = T.voronoi(pd, 0.9, feature="DISTANCE_TO_EDGE", seed=seed)
    crack = T.math("SUBTRACT", 1.0, T.band(vo.outputs["Distance"], 0.003, 0.008))
    fcm, _ = T.noise(p, 1.4, 2, 0.5, seed=seed + 5)
    crack = T.math("MULTIPLY", crack, T.band(fcm, 0.55, 0.62))
    near = T.math("SUBTRACT", 1.0, T.band(vo.outputs["Distance"], 0.02, 0.09))
    fm, _ = T.noise(p, 2.4, 3, 0.6, seed=seed + 6)
    rnd = T.math("FRACT", T.math("MULTIPLY", r, 7.31))
    pmiss = T.math("ADD", T.math("MULTIPLY", near, T.band(fcm, 0.45, 0.6)), T.band(fm, 0.66 - 0.06 * dirty, 0.74))
    miss = T.math("GREATER_THAN", T.math("MULTIPLY", pmiss, T.math("ADD", rnd, 0.35)), 0.62)
    miss = T.math("MULTIPLY", miss, T.math("SUBTRACT", 1.0, grout))
    fb, _ = T.noise(p, 25.0, 3, 0.6, seed=seed + 7)
    bed = T.ramp(fb, [(0.3, (0.13, 0.125, 0.115)), (0.7, (0.22, 0.21, 0.195))])
    col = T.mix(miss, col, bed)
    col = T.mix(T.math("MULTIPLY", crack, 0.85), col, (0.05, 0.048, 0.045))
    col, dm = _dust(T, p, col, 0.22 + 0.15 * dirty, seed + 9)
    rough = T.math("ADD", 0.30, T.math("MULTIPLY", streak, 0.45))
    rough = T.mixf(grout, rough, 0.9)
    rough = T.mixf(miss, rough, 0.95)
    rough = T.mixf(dm, rough, 0.9)
    tilt = T.math("MULTIPLY", T.math("FRACT", T.math("MULTIPLY", r, 3.7)), 0.25)
    h = T.math("ADD", T.math("SUBTRACT", 1.0, grout), tilt)
    h = T.math("SUBTRACT", h, T.math("MULTIPLY", miss, 1.6))
    h = T.math("SUBTRACT", h, T.math("MULTIPLY", crack, 0.8))
    h = T.math("ADD", h, T.math("MULTIPLY", T.math("MULTIPLY", fb, miss), 0.6))
    return col, rough, h, 0.004


def mat_interior(T, seed=60.0):
    """Crushed storey interior: dark sooty concrete, plaster remnants, dust (albedo ~0.04-0.15)."""
    p = T.coord()
    f0, _ = T.noise(p, 1.2, 6, 0.65, seed=seed)
    base = T.ramp(f0, [(0.3, (0.05, 0.047, 0.043)), (0.6, (0.10, 0.093, 0.084)), (0.8, (0.14, 0.13, 0.115))])
    fpl, _ = T.noise(p, 2.5, 4, 0.6, seed=seed + 1)
    plaster = T.band(fpl, 0.62, 0.68)
    base = T.mix(T.math("MULTIPLY", plaster, 0.8), base, (0.19, 0.18, 0.165))
    col, dm = _dust(T, p, base, 0.22, seed + 3)
    f2, _ = T.noise(p, 18.0, 4, 0.6, seed=seed + 4)
    rough = T.math("ADD", 0.88, T.math("MULTIPLY", f2, 0.1))
    h = T.math("ADD", T.math("MULTIPLY", f2, 0.5), T.math("MULTIPLY", plaster, 0.4))
    return col, rough, h, 0.010


def mat_brick(T, seed=80.0):
    """Broken red-brick infill (inside the tiled walls): dusty red-brown bricks, grey mortar."""
    p = T.coord()
    br = T.n("ShaderNodeTexBrick", offset=0.5, offset_frequency=2)
    T._in(br.inputs["Vector"], p)
    br.inputs["Color1"].default_value = (0.24, 0.10, 0.06, 1)
    br.inputs["Color2"].default_value = (0.32, 0.15, 0.09, 1)
    br.inputs["Mortar"].default_value = (0.30, 0.29, 0.27, 1)
    br.inputs["Scale"].default_value = 1.0
    br.inputs["Mortar Size"].default_value = 0.012
    br.inputs["Brick Width"].default_value = 0.23
    br.inputs["Row Height"].default_value = 0.065
    f0, _ = T.noise(p, 6.0, 4, 0.6, seed=seed)
    col = T.mix(0.4, br.outputs["Color"], T.ramp(f0, [(0.3, (0.6, 0.6, 0.6)), (0.7, (1.0, 1.0, 1.0))]), "MULTIPLY")
    col, dm = _dust(T, p, col, 0.5, seed + 2)
    rough = T.math("ADD", 0.88, T.math("MULTIPLY", f0, 0.1))
    f2, _ = T.noise(p, 30.0, 4, 0.6, seed=seed + 3)
    h = T.math("ADD", T.math("SUBTRACT", 1.0, br.outputs["Factor"]), T.math("MULTIPLY", f2, 0.5))
    return col, rough, h, 0.010


MATS = {"concrete": mat_concrete, "fracture": mat_fracture, "tile_a": lambda T: mat_tile(T, 0.0, 40.0),
        "tile_b": lambda T: mat_tile(T, 1.0, 47.0), "interior": mat_interior, "brick": mat_brick}


def bake_textures(out, samples=1, log=print):
    """Bake each procedural material on a 3.0 x 2.9 m UV plane (metres in object space) to albedo (sRGB),
    roughness (L) and tangent-space normal PNGs. Returns {set: {albedo, rough, normal}} relative paths."""
    bpy = _bpy()
    from PIL import Image
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = samples
    sc.render.bake.margin = 0
    me = bpy.data.meshes.new("bakeplane")
    hx, hy = BAY / 2, STOREY / 2
    me.from_pydata([(-hx, -hy, 0), (hx, -hy, 0), (hx, hy, 0), (-hx, hy, 0)], [], [(0, 1, 2, 3)])
    uv = me.uv_layers.new(name="UVMap")
    for i, (u, v) in enumerate([(0, 0), (1, 0), (1, 1), (0, 1)]):
        uv.data[i].uv = (u, v)
    ob = bpy.data.objects.new("bakeplane", me)
    sc.collection.objects.link(ob)
    for o in sc.objects:
        o.select_set(False)
    ob.select_set(True)
    bpy.context.view_layer.objects.active = ob
    os.makedirs(os.path.join(out, "textures"), exist_ok=True)
    res = {}
    for name, fn in MATS.items():
        n = TEX[name]
        mat = bpy.data.materials.new("bake_" + name)
        mat.use_nodes = True
        T = Nodes(mat)
        col, rough, h, bdist = fn(T)                            # bdist: relief depth (m) per height unit
        bump = T.n("ShaderNodeBump")
        bump.inputs["Strength"].default_value = 1.0
        bump.inputs["Distance"].default_value = bdist
        T._in(bump.inputs["Height"], h)
        bsdf = T.n("ShaderNodeBsdfPrincipled")
        T._in(bsdf.inputs["Normal"], bump.outputs["Normal"])
        emi = T.n("ShaderNodeEmission")
        outn = T.n("ShaderNodeOutputMaterial")
        img_node = T.n("ShaderNodeTexImage")
        me.materials.clear()
        me.materials.append(mat)
        files = {}
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
            A = A.reshape(n, n, 4)[::-1, :, :3]                     # Blender rows are bottom-up
            if kind == "albedo":
                A = np.clip(A, 0, 1)
                A = np.where(A <= 0.0031308, 12.92 * A, 1.055 * np.power(A, 1 / 2.4) - 0.055)
                im = Image.fromarray((A * 255 + 0.5).astype(np.uint8), "RGB")
            elif kind == "rough":
                im = Image.fromarray((np.clip(A[..., 0], 0, 1) * 255 + 0.5).astype(np.uint8), "L")
            else:
                im = Image.fromarray((np.clip(A, 0, 1) * 255 + 0.5).astype(np.uint8), "RGB")
            rel = f"textures/{name}_{kind}.png"
            im.save(os.path.join(out, rel), optimize=True)
            files[kind] = rel
            if kind == "albedo":
                lin = np.where(A <= 0.04045, A / 12.92, ((A + 0.055) / 1.055) ** 2.4)
                log(f"[kit] {name}: {n}px, albedo mean (linear) {lin.reshape(-1, 3).mean(0).round(3)}")
            bpy.data.images.remove(img)
        res[name] = files
    bpy.data.objects.remove(ob)
    return res


# ------------------------------------------------------------------------------------------------ geometry
def _new_obj(name, bm):
    bpy = _bpy()
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    return ob


def _box(name, x0, x1, y0, y1, z0, z1):
    import bmesh
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    for v in bm.verts:
        v.co.x = x0 if v.co.x < 0 else x1
        v.co.y = y0 if v.co.y < 0 else y1
        v.co.z = z0 if v.co.z < 0 else z1
    return _new_obj(name, bm)


def _cutter(rng, c, r, jag=0.38, sub=1):
    """Noisy ellipsoid for boolean bites (broken edges)."""
    import bmesh
    from mathutils import Euler, Vector, noise
    bm = bmesh.new()
    bmesh.ops.create_icosphere(bm, subdivisions=sub, radius=1.0)
    off = Vector(rng.uniform(-50, 50, 3).tolist())
    rot = Euler(rng.uniform(0, math.pi, 3).tolist()).to_matrix()
    for v in bm.verts:
        d = v.co.copy()
        k = 1.0 + jag * noise.fractal(d * 2.2 + off, 0.6, 2.0, 3)        # angular chips, not round dents
        v.co = rot @ Vector((d.x * r[0] * k, d.y * r[1] * k, d.z * r[2] * k)) + Vector(c)
    return _new_obj("cut", bm)


def _prism(poly, z0, z1):
    """Extruded 2D polygon (x, y) between z0 and z1, for torn openings / jagged breaks."""
    import bmesh
    bm = bmesh.new()
    lo = [bm.verts.new((x, y, z0)) for x, y in poly]
    hi = [bm.verts.new((x, y, z1)) for x, y in poly]
    bm.faces.new(lo[::-1])
    bm.faces.new(hi)
    n = len(poly)
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((lo[i], lo[j], hi[j], hi[i]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return _new_obj("prism", bm)


def _jagged_rect(rng, cx, cy, hw, hh, step=0.09, amp=0.06, tear=0.0):
    """Irregular (torn) outline around a rectangle, CCW."""
    pts = []
    corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
    per = 4 * (hw + hh)
    ph = rng.uniform(0, 2 * math.pi, 4)
    fr = rng.integers(2, 5, 2)
    s0 = 0.0
    for k in range(4):
        (ax, ay), (bx, by) = corners[k], corners[(k + 1) % 4]
        L = math.hypot(bx - ax, by - ay)
        m = max(2, int(L / step))
        nx, ny = (by - ay) / L, -(bx - ax) / L                  # outward normal for CCW
        for i in range(m):
            t = i / m
            u = (s0 + t * L) / per                              # smooth along the perimeter: torn, not spiky
            a = (amp * (0.5 * math.sin(2 * math.pi * fr[0] * u + ph[0]) + 0.3 * math.sin(2 * math.pi * 7 * u + ph[1]))
                 + rng.normal(0, amp * 0.35)
                 + tear * max(0.0, math.sin(2 * math.pi * fr[1] * u + ph[2])) ** 2)
            pts.append((cx + ax + (bx - ax) * t + nx * a, cy + ay + (by - ay) * t + ny * a))
        s0 += L
    return pts


def _apply_mods(ob):
    bpy = _bpy()
    dg = bpy.context.evaluated_depsgraph_get()
    ev = ob.evaluated_get(dg)
    me = bpy.data.meshes.new_from_object(ev)
    old = ob.data
    ob.modifiers.clear()
    ob.data = me
    bpy.data.meshes.remove(old)


def _boolean(ob, cutters, op="DIFFERENCE"):
    bpy = _bpy()
    for c in cutters:
        m = ob.modifiers.new("b", "BOOLEAN")
        m.object = c
        m.operation = op
        m.solver = "EXACT"
        _apply_mods(ob)
        bpy.data.objects.remove(c)


def _remesh(ob, voxel):
    m = ob.modifiers.new("r", "REMESH")
    m.mode = "VOXEL"
    m.voxel_size = voxel
    m.adaptivity = 0.0
    _apply_mods(ob)


def _decimate(ob, target_tris):
    nt = sum(len(p.vertices) - 2 for p in ob.data.polygons)
    if nt > target_tris:
        m = ob.modifiers.new("d", "DECIMATE")
        m.ratio = target_tris / nt
        _apply_mods(ob)


def _displace(ob, planes, seed, amp_frac=0.022, amp_form=0.0015, scale=7.0):
    """Fracture relief: vertices off the formed planes get fractal noise along the normal."""
    from mathutils import Vector, noise
    me = ob.data
    nv = len(me.vertices)
    co = np.empty(nv * 3, np.float32)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    nr = np.empty(nv * 3, np.float32)
    me.vertex_normals.foreach_get("vector", nr)
    nr = nr.reshape(-1, 3)
    on = np.zeros(nv, bool)
    for ax, val, sgn, _m in planes:
        on |= np.abs(co[:, ax] - val) < 0.012
    off = Vector((seed * 13.1, seed * 7.7, seed * 3.3))
    d = np.array([noise.fractal(Vector(p.tolist()) * scale + off, 0.55, 2.1, 4) for p in co], np.float32)
    d2 = np.array([noise.noise(Vector(p.tolist()) * 40.0 + off) for p in co], np.float32)
    amp = np.where(on, amp_form, amp_frac)
    co = co + nr * ((d + 0.3 * d2) * amp)[:, None]
    me.vertices.foreach_set("co", co.reshape(-1))
    me.update()


def _finish(ob, planes, default, uvoff=(0.0, 0.0), sharp_deg=30.0, smooth=True):
    """Triangulate; per-face material from the formed planes (else default); box-projected UVs in the module
    frame (x in [-1.5, 1.5] -> u 0..1, y in [-1.45, 1.45] -> v 0..1, z / 3 for the depth axis)."""
    import bmesh
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    bmesh.ops.triangulate(bm, faces=bm.faces[:])
    bm.normal_update()
    uvl = bm.loops.layers.uv.new("st")
    names = []
    tags = []
    ou, ov = uvoff
    for f in bm.faces:
        c, n = f.calc_center_median(), f.normal
        mat = default
        for ax, val, sgn, m in planes:
            if abs(c[ax] - val) < 0.02 and n[ax] * sgn > 0.75:
                mat = m
                break
        tags.append(mat)
        a = max(range(3), key=lambda k: abs(n[k]))
        for lp in f.loops:
            p = lp.vert.co
            if a == 2:
                u, v = (p.x + BAY / 2) / BAY, (p.y + STOREY / 2) / STOREY
            elif a == 0:
                u, v = p.z / BAY + 0.31, (p.y + STOREY / 2) / STOREY
            else:
                u, v = (p.x + BAY / 2) / BAY, p.z / BAY + 0.57
            if not m_is_tile(mat):
                u, v = u + ou, v + ov
            lp[uvl].uv = (u, v)
        f.smooth = smooth and mat == default                    # cast / tiled faces flat: no pillowy gradients
    for e in bm.edges:
        if len(e.link_faces) == 2:
            ang = e.link_faces[0].normal.angle(e.link_faces[1].normal, 0.0)
            e.smooth = ang < math.radians(sharp_deg)
    for nm in tags:
        if nm not in names:
            names.append(nm)
    for f, nm in zip(bm.faces, tags):
        f.material_index = names.index(nm)
    bm.to_mesh(ob.data)
    bm.free()
    ob["mats"] = json.dumps(names)
    return ob


def m_is_tile(m):
    return m.startswith("tile")


def fractured_box(rng, x0, x1, y0, y1, z0, z1, planes, default, bites=(), cuts=(), voxel=0.025, amp=0.022,
                  tris=4000, seed=0, uvoff=None):
    """Box -> boolean bites / prisms -> voxel remesh -> fracture relief -> decimate -> materials + UVs.
    planes: [(axis, value, sign, material)] formed (cast) faces; everything else is a fracture surface."""
    ob = _box("piece", x0, x1, y0, y1, z0, z1)
    cs = [_cutter(rng, c, r) for c, r in bites] + list(cuts)
    if cs:
        _boolean(ob, cs)
    _remesh(ob, voxel)
    _displace(ob, planes, seed, amp)
    _decimate(ob, tris)
    uvoff = uvoff if uvoff is not None else tuple(rng.uniform(0, 0.4, 2))
    return _finish(ob, planes, default, uvoff)


def tube(pts, r, mat, sides=6, name="tube"):
    """Swept tube along a polyline (rebar, wires, bent frames)."""
    import bmesh
    from mathutils import Vector
    pts = [Vector(p) for p in pts]
    bm = bmesh.new()
    rings = []
    for i, p in enumerate(pts):
        d = (pts[min(i + 1, len(pts) - 1)] - pts[max(i - 1, 0)]).normalized()
        a = d.cross(Vector((0, 0, 1))) if abs(d.z) < 0.9 else d.cross(Vector((1, 0, 0)))
        a.normalize()
        b = d.cross(a)
        rings.append([bm.verts.new(p + (a * math.cos(t) + b * math.sin(t)) * r)
                      for t in np.linspace(0, 2 * math.pi, sides, endpoint=False)])
    for i in range(len(rings) - 1):
        for k in range(sides):
            j = (k + 1) % sides
            bm.faces.new((rings[i][k], rings[i][j], rings[i + 1][j], rings[i + 1][k]))
    bm.faces.new(rings[0][::-1])
    bm.faces.new(rings[-1])
    ob = _new_obj(name, bm)
    return _finish(ob, [], mat, (0, 0), sharp_deg=80.0)


def bent_bar(rng, p0, d, L, bend=0.6, n=6):
    """Polyline of a rebar end: straight out of the concrete, then bent / kinked."""
    p0, d = np.asarray(p0, float), np.asarray(d, float) / np.linalg.norm(d)
    side = rng.normal(0, 1, 3)
    side -= d * side.dot(d)
    side /= max(np.linalg.norm(side), 1e-6)
    pts, p, dd = [p0 - d * 0.05], p0.copy(), d.copy()
    for k in range(n):
        p = p + dd * (L / n)
        pts.append(p.copy())
        if k >= n // 3:
            dd = dd + side * bend * rng.uniform(0.3, 1.0) + rng.normal(0, 0.08, 3)
            dd /= np.linalg.norm(dd)
    return [tuple(q) for q in pts]


def chunk(rng, size, tile_face=False, seed=0, tris=900):
    """Concrete rubble chunk (slab shard): two cast faces, fractured sides, optional tiled face."""
    sx, sy, sz = size
    planes = [(2, sz, 1, "tile_a" if tile_face else "concrete"), (2, 0.0, -1, "concrete")]
    bites = [((rng.uniform(-sx, sx) / 2 * 1.4, rng.uniform(-sy, sy) / 2 * 1.4, sz * rng.uniform(0, 1)),
              (rng.uniform(0.25, 0.5) * sx, rng.uniform(0.25, 0.5) * sy, sz * rng.uniform(0.6, 1.4)))
             for _ in range(3)]
    return fractured_box(rng, -sx / 2, sx / 2, -sy / 2, sy / 2, 0.0, sz, planes, "fracture", bites,
                         voxel=min(sx, sy, sz) / 7, amp=min(sx, sy, sz) * 0.08, tris=tris, seed=seed)


def _xform(ob, t=(0, 0, 0), rx=0.0, ry=0.0, rz=0.0):
    from mathutils import Euler, Matrix, Vector
    M = Matrix.Translation(Vector(t)) @ Euler((rx, ry, rz)).to_matrix().to_4x4()
    ob.data.transform(M)
    return ob


# ------------------------------------------------------------------------------------------------ modules
def slab_module(rng, var):
    """Slab edge segment: 0.5 m thick (V), 3 m long (U), front at z ~0.95, chipped / broken, rebar ends."""
    t = SLAB_T / 2
    z1 = 1.05
    planes = [(2, z1, 1, "concrete"), (1, t, 1, "concrete"), (1, -t, -1, "concrete")]
    x0, x1 = -BAY / 2, BAY / 2
    bites, cuts, parts = [], [], []
    nb = (3, 5, 7, 4, 6)[var % 5]
    for _ in range(nb):                                         # chipped front edges
        bites.append(((rng.uniform(x0, x1), rng.choice((-t, t)) * rng.uniform(0.6, 1.1), z1 - rng.uniform(0, 0.1)),
                      (rng.uniform(0.12, 0.35), rng.uniform(0.06, 0.16), rng.uniform(0.08, 0.2))))
    if var in (1, 2):                                           # broken end: big jagged bite, slab shortened
        xe = x1 - rng.uniform(0.25, 0.6)
        poly = [(xe + rng.normal(0, 0.05), -t - 0.1)] + [(xe + rng.normal(0, 0.12) + 0.05 * k, -t + k * 0.12)
                                                         for k in range(1, 9)] + [(x1 + 0.3, t + 0.1), (x1 + 0.3, -t - 0.1)]
        cuts.append(_prism(poly, -0.1, z1 + 0.2))
        for _ in range(3):
            bites.append(((xe + rng.uniform(-0.1, 0.2), rng.uniform(-t, t), rng.uniform(0.2, z1)), (0.25, 0.18, 0.3)))
    if var == 4:                                                # big corner bite
        bites.append(((x0 + 0.2, t * 0.8, z1 - 0.05), (0.7, 0.32, 0.45)))
    if var == 3:                                                # cracked through, two pieces
        xm = rng.uniform(-0.4, 0.4)
        poly = [(xm - 0.04 + rng.normal(0, 0.03), -t - 0.1 + k * (2 * t + 0.2) / 8) for k in range(9)]
        poly2 = [(p[0] + 0.07, p[1]) for p in poly[::-1]]
        cuts.append(_prism(poly + poly2, -0.1, z1 + 0.2))
    ob = fractured_box(rng, x0, x1, -t, t, 0.0, z1, planes, "fracture", bites, cuts, voxel=0.022, amp=0.022,
                       tris=5000, seed=var + 1)
    if var == 3:                                                # sag the +x piece
        me = ob.data
        for v in me.vertices:
            if v.co.x > 0:
                v.co.z -= 0.06 + 0.04 * v.co.x
                v.co.y -= 0.03 * v.co.x
    parts.append(ob)
    if var in (1, 2, 3):                                        # exposed bars at the break
        xe = (x1 - 0.4) if var != 3 else 0.0
        for yy in (-t + 0.06, t - 0.06):
            for zz in np.linspace(0.15, z1 - 0.1, 5 if var != 3 else 4):
                if rng.random() < 0.25:
                    continue
                L = rng.uniform(0.25, 0.7) if var != 3 else 0.2
                parts.append(tube(bent_bar(rng, (xe, yy, zz), (1, 0, 0), L, bend=0.5 if var != 3 else 0.05),
                                  0.007, "rust"))
    return parts


def wall_module(rng, var):
    """Tiled infill wall between two slabs (y in [-1.2, 1.2]), tile face at z ~0.78, broken brick inside,
    torn window opening + bent aluminium frame in some, missing-tile pockets, interior back plate."""
    z0, z1 = 0.60, 0.78
    tile = "tile_b" if var in (2, 3, 5) else "tile_a"
    planes = [(2, z1, 1, tile), (2, z0, -1, "interior")]
    x0, x1, y0, y1 = -BAY / 2 + 0.02, BAY / 2 - 0.02, -GAP, GAP     # full bay: columns (every 2nd line) overlap
    bites, cuts, parts = [], [], []
    for _ in range(int(rng.integers(3, 7) + (4 if var == 5 else 0))):                    # missing-tile / plaster pockets on the face
        bites.append(((rng.uniform(x0, x1), rng.uniform(y0, y1), z1 + 0.01),
                      (rng.uniform(0.08, 0.3), rng.uniform(0.06, 0.25), rng.uniform(0.012, 0.03))))
    for _ in range(int(rng.integers(2, 5))):                    # broken edges against the slabs / columns
        e = rng.integers(4)
        p = [(rng.uniform(x0, x1), y0), (rng.uniform(x0, x1), y1), (x0, rng.uniform(y0, y1)), (x1, rng.uniform(y0, y1))][e]
        bites.append(((p[0], p[1], (z0 + z1) / 2), (rng.uniform(0.15, 0.4), rng.uniform(0.12, 0.3), 0.25)))
    win = None
    if var in (0, 1, 3, 6):                                     # window opening: wide, torn, blown out, small
        hw, hh = {0: (0.80, 0.42), 1: (0.62, 0.55), 3: (0.9, 0.7), 6: (0.28, 0.3)}[var]
        cx = rng.uniform(-0.45, 0.45) if var != 6 else rng.choice((-0.8, 0.8)) + rng.normal(0, 0.1)
        cy = rng.uniform(0.0, 0.3)
        poly = _jagged_rect(rng, cx, cy, hw, hh, 0.08, 0.02 if var in (0, 6) else 0.07,
                            tear=0.0 if var in (0, 6) else 0.12)
        cuts.append(_prism(poly, z0 - 0.1, z1 + 0.1))
        win = (cx, cy, hw, hh)
    if var == 2:                                                # wall broken off: only the lower ~60 % stands
        yt = rng.uniform(-0.1, 0.3)
        poly = [(x1 + 0.3, y1 + 0.3), (x0 - 0.3, y1 + 0.3)] + [(x0 - 0.3 + k * (x1 - x0 + 0.6) / 14,
                                                               yt + rng.normal(0, 0.12) + 0.22 * math.sin(k * 0.9))
                                                              for k in range(15)]
        cuts.append(_prism(poly[::-1] if _area(poly) < 0 else poly, z0 - 0.1, z1 + 0.1))
    if var == 4:                                                # big impact hole, cracked
        poly = _jagged_rect(rng, rng.uniform(-0.6, 0.6), rng.uniform(-0.4, 0.3), 0.5, 0.42, 0.07, 0.10, tear=0.18)
        cuts.append(_prism(poly, z0 - 0.1, z1 + 0.1))
    ob = fractured_box(rng, x0, x1, y0, y1, z0, z1, planes, "brick", bites, cuts, voxel=0.022, amp=0.015,
                       tris=6000, seed=10 + var)
    _skew(ob, rng)
    parts.append(ob)
    # interior back plate (seen through the openings): dark, at the face plane
    back = fractured_box(rng, -BAY / 2, BAY / 2, y0, y1, 0.0, 0.05, [(2, 0.05, 1, "interior")], "interior",
                         voxel=0.06, amp=0.01, tris=600, seed=30 + var)
    parts.append(back)
    if win is not None and var in (0, 1, 6):                            # bent aluminium window frame in the opening
        cx, cy, hw, hh = win
        hw, hh = hw - 0.06, hh - 0.06
        zf = z1 - 0.06
        fr = [(cx - hw, cy - hh), (cx + hw, cy - hh), (cx + hw, cy + hh), (cx - hw, cy + hh), (cx - hw, cy - hh)]
        keep = rng.integers(2, 5)
        pts = []
        for k in range(keep):
            (ax, ay), (bx, by) = fr[k], fr[k + 1]
            for s in np.linspace(0, 1, 6, endpoint=False):
                pts.append((ax + (bx - ax) * s, ay + (by - ay) * s, zf))
        pts.append((*fr[keep], zf))
        P = np.array(pts)
        tt = np.linspace(0, 1, len(P))
        P[:, 2] += 0.22 * tt ** 2 * (1 if var == 1 else 0.4)    # the loose end bends out of the wall
        P[:, 0] += 0.06 * np.sin(tt * 5.0) * (var == 1)
        parts.append(tube([tuple(p) for p in P], 0.022, "alu", sides=4))
        if var == 0:                                            # mullion + a glass shard in a corner
            parts.append(tube([(cx, cy - hh, zf), (cx + 0.03, cy, zf + 0.02), (cx + 0.05, cy + hh * 0.6, zf + 0.09)],
                              0.018, "alu", sides=4))
    return parts


def _area(poly):
    return 0.5 * sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]))


def _skew(ob, rng):
    """Small out-of-plane lean / bulge: walls are never perfectly flat after a collapse."""
    a, b = rng.normal(0, 0.03), rng.normal(0, 0.03)
    for v in ob.data.vertices:
        v.co.z += a * v.co.y + b * v.co.x + 0.03 * math.cos(v.co.x * 1.1) * math.cos(v.co.y * 1.3)


def void_module(rng, var):
    """Crushed storey: dark recessed interior, broken sagging ceiling, debris on the floor, dangling rebar
    and wires, wall remnants at the sides. y in [-1.2, 1.2], back at z = 0."""
    parts = []
    y0, y1 = -GAP, GAP
    back = fractured_box(rng, -BAY / 2, BAY / 2, y0, y1, 0.0, 0.08, [(2, 0.08, 1, "interior")], "interior",
                         bites=[((rng.uniform(-1, 1), rng.uniform(-1, 1), 0.1), (0.4, 0.3, 0.05))],
                         voxel=0.05, amp=0.02, tris=900, seed=40 + var)
    parts.append(back)
    # sagging ceiling plate hanging from the slab above (y = y1), fractured, hinged at the back
    ang = rng.uniform(0.25, 0.6)
    cw = rng.uniform(1.4, 2.4)
    cx = rng.uniform(-0.6, 0.6)
    ce = fractured_box(rng, cx - cw / 2, cx + cw / 2, -0.06, 0.06, 0.0, rng.uniform(0.6, 0.85),
                       [(1, 0.06, 1, "interior"), (1, -0.06, -1, "concrete")], "fracture",
                       bites=[((cx + rng.uniform(-cw, cw) / 2, 0, 0.8), (0.4, 0.2, 0.3)) for _ in range(3)],
                       voxel=0.02, amp=0.015, tris=1500, seed=50 + var)
    _xform(ce, (0, y1 - 0.02, 0.08), rx=ang)                    # rotate about x: front edge drops
    parts.append(ce)
    # debris heap on the floor (slab below, y = y0)
    heap = fractured_box(rng, -1.3, 1.3, -0.15, 0.25, 0.0, 0.75, [], "fracture",
                         bites=[((rng.uniform(-1.3, 1.3), 0.4, rng.uniform(0.2, 0.9)), (0.5, 0.35, 0.4))
                                for _ in range(5)], voxel=0.04, amp=0.06, tris=1500, seed=60 + var)
    _xform(heap, (rng.uniform(-0.2, 0.2), y0 + 0.12, 0.04))
    parts.append(heap)
    for k in range(int(rng.integers(5, 9))):
        s = rng.uniform(0.12, 0.38)
        c = chunk(rng, (s * rng.uniform(1, 1.8), s * rng.uniform(0.6, 1.2), s * rng.uniform(0.3, 0.5)),
                  tile_face=rng.random() < 0.3, seed=70 + 10 * var + k, tris=400)
        _xform(c, (rng.uniform(-1.3, 1.3), y0 + rng.uniform(0.1, 0.55), rng.uniform(0.1, 0.8)),
               *rng.uniform(-1.2, 1.2, 3))
        parts.append(c)
    # wall remnants at the sides (tiled outside, brick break)
    for sx in (-1, 1):
        if rng.random() < 0.7:
            h = rng.uniform(0.6, 1.8)
            w = rng.uniform(0.25, 0.5)
            xa = sx * (BAY / 2 - 0.24) - (w if sx > 0 else 0)
            wr = fractured_box(rng, xa, xa + w, y0, y0 + h, 0.6, 0.78, [(2, 0.78, 1, "tile_b")], "brick",
                               bites=[((xa + w / 2 * (1 - sx), y0 + h, 0.7), (0.3, 0.35, 0.3))],
                               voxel=0.025, amp=0.015, tris=900, seed=80 + var + sx)
            parts.append(wr)
    # dangling rebar and wires from the ceiling
    for k in range(int(rng.integers(3, 7))):
        x = rng.uniform(-1.3, 1.3)
        p0 = (x, y1 + 0.02, rng.uniform(0.3, 0.9))
        L = rng.uniform(0.5, 1.5)
        parts.append(tube(bent_bar(rng, p0, (0, -1, rng.normal(0.2, 0.2)), L, bend=0.35), 0.006, "rust"))
    for k in range(int(rng.integers(2, 4))):
        x = rng.uniform(-1.2, 1.2)
        L = rng.uniform(0.7, 1.6)
        sag = np.linspace(0, 1, 9)
        pts = [(x + 0.3 * s + rng.normal(0, 0.01), y1 - L * s, 0.5 + 0.35 * math.sin(s * 2.5) + rng.normal(0, 0.02))
               for s in sag]
        parts.append(tube(pts, 0.007, "wire", sides=5))
    if var == 2:                                                # second fallen ceiling piece leaning on the heap
        c2 = fractured_box(rng, -0.6, 0.6, -0.06, 0.06, 0.0, 0.7, [(1, 0.06, 1, "interior")], "fracture",
                           bites=[((0.3, 0, 0.7), (0.4, 0.2, 0.3))], voxel=0.02, amp=0.015, tris=900, seed=95)
        _xform(c2, (rng.uniform(-0.6, 0.6), y0 + 0.5, 0.15), rx=1.0, rz=rng.uniform(-0.4, 0.4))
        parts.append(c2)
    if var == 3:                                                # a bent window frame lying in the rubble
        pts = [(-0.9, y0 + 0.4, 0.7), (-0.2, y0 + 0.45, 0.78), (0.3, y0 + 0.7, 0.72), (0.5, y0 + 1.2, 0.6)]
        parts.append(tube(pts, 0.022, "alu", sides=4))
    return parts


def column_module(rng, var):
    """Column at a bay line between two slabs: 0.45 x 0.5 m, front at z ~0.85; spalled corners, a mid-height
    break with the bent rebar cage exposed, or a crushed stub."""
    hx = 0.225
    z0, z1 = 0.35, 0.80
    planes = [(2, z1, 1, "tile_a" if var != 2 else "concrete"), (0, hx, 1, "concrete"), (0, -hx, -1, "concrete")]
    parts = []
    if var == 0:
        bites = [((rng.choice((-hx, hx)), rng.uniform(-GAP, GAP), z1), (0.1, rng.uniform(0.15, 0.35), 0.1))
                 for _ in range(4)]
        parts.append(fractured_box(rng, -hx, hx, -GAP, GAP, z0, z1, planes, "fracture", bites, voxel=0.02,
                                   amp=0.02, tris=2500, seed=100))
        return parts
    yb = rng.uniform(-0.3, 0.2) if var == 1 else rng.uniform(-0.75, -0.45)
    lo = fractured_box(rng, -hx, hx, -GAP, yb, z0, z1, planes, "fracture",
                       [((0, yb, rng.uniform(z0, z1)), (0.3, 0.12, 0.25)), ((hx, yb, z1), (0.15, 0.2, 0.15))],
                       voxel=0.02, amp=0.025, tris=2000, seed=101 + var)
    parts.append(lo)
    gap = rng.uniform(0.25, 0.45)
    if var == 1:
        hi = fractured_box(rng, -hx, hx, yb + gap, GAP, z0, z1, planes, "fracture",
                           [((0, yb + gap, rng.uniform(z0, z1)), (0.3, 0.12, 0.25))],
                           voxel=0.02, amp=0.025, tris=2000, seed=111)
        _xform(hi, (0.05, 0, 0.03), rz=0.04)
        parts.append(hi)
    for cx in (-hx + 0.05, hx - 0.05):                         # cage: 4 corner bars, buckled through the gap
        for cz in (z0 + 0.05, z1 - 0.05):
            mid = (cx * 1.6 + rng.normal(0, 0.04), yb + gap / 2, cz + rng.normal(0.06, 0.05))
            if var == 1:
                parts.append(tube([(cx, yb - 0.05, cz), mid, (cx + 0.05, yb + gap + 0.05, cz + 0.03)], 0.009, "rust"))
            else:
                parts.append(tube(bent_bar(rng, (cx, yb, cz), (0, 1, 0), rng.uniform(0.3, 0.8), 0.6), 0.009, "rust"))
    if var == 1:                                                # stirrups
        for yy in (yb + 0.08, yb + gap - 0.08):
            parts.append(tube([(-hx + 0.04, yy, z0 + 0.04), (hx - 0.04, yy, z0 + 0.04), (hx - 0.04, yy, z1 - 0.04),
                               (-hx + 0.04, yy, z1 - 0.04), (-hx + 0.04, yy, z0 + 0.04)], 0.005, "rust", sides=4))
    return parts


def beam_module(rng, var):
    """Broken beam end sticking out of a crushed storey at a slab line (y ~ 0), along N, with rebar."""
    hx, hy = 0.15, 0.3
    L = rng.uniform(0.9, 1.4)
    planes = [(0, hx, 1, "concrete"), (0, -hx, -1, "concrete"), (1, -hy, -1, "concrete"), (1, hy, 1, "concrete")]
    bites = [((rng.uniform(-hx, hx), rng.uniform(-hy, hy), L), (0.25, 0.3, 0.25)),
             ((hx, rng.uniform(-hy, hy), L * 0.8), (0.1, 0.15, 0.2))]
    parts = [fractured_box(rng, -hx, hx, -hy, hy, 0.0, L, planes, "fracture", bites, voxel=0.02, amp=0.025,
                           tris=1800, seed=120 + var)]
    for cx in (-hx + 0.04, hx - 0.04):
        for cy in (-hy + 0.04, hy - 0.04):
            parts.append(tube(bent_bar(rng, (cx, cy, L - 0.25), (0, 0, 1), rng.uniform(0.3, 0.7),
                                       0.4 if var else 0.8), 0.009, "rust"))
    if var == 1:
        for p in parts[0].data.vertices:                         # droops: hinged where it leaves the slab
            p.co.y -= 0.12 * (p.co.z / L) ** 2
    return parts


def debris_module(rng, var):
    sizes = [(0.6, 0.45, 0.16), (0.35, 0.3, 0.2), (0.9, 0.5, 0.18), (0.45, 0.35, 0.3), (0.5, 0.4, 0.12)]
    parts = [chunk(rng, sizes[var % 5], tile_face=var in (2, 4), seed=200 + var, tris=1200)]
    if var in (0, 2):
        sx = sizes[var][0]
        for k in range(int(rng.integers(2, 4))):
            parts.append(tube(bent_bar(rng, (sx / 2 - 0.08, rng.uniform(-0.15, 0.15), 0.08), (1, 0, 0.1),
                                       rng.uniform(0.2, 0.5), 0.5), 0.007, "rust"))
    return parts


KIT = {"slab": (slab_module, 5), "wall": (wall_module, 7), "void": (void_module, 4),
       "column": (column_module, 3), "beam": (beam_module, 2), "debris": (debris_module, 5)}


def collect(parts):
    """Merge Blender part objects into arrays: points, tris, corner normals, corner uvs, face material names."""
    P, F, Nn, UV, M = [], [], [], [], []
    o = 0
    for ob in parts:
        me = ob.data
        names = json.loads(ob["mats"])
        nv = len(me.vertices)
        co = np.empty(nv * 3, np.float32)
        me.vertices.foreach_get("co", co)
        nl = len(me.loops)
        vi = np.empty(nl, np.int32)
        me.loops.foreach_get("vertex_index", vi)
        cn = np.array([c.vector[:] for c in me.corner_normals], np.float32)
        uv = np.empty(nl * 2, np.float32)
        me.uv_layers["st"].data.foreach_get("uv", uv)
        npoly = len(me.polygons)
        mi = np.empty(npoly, np.int32)
        me.polygons.foreach_get("material_index", mi)
        ls = np.empty(npoly, np.int32)
        me.polygons.foreach_get("loop_total", ls)
        assert (ls == 3).all(), ob.name
        P.append(co.reshape(-1, 3))
        F.append(vi.reshape(-1, 3) + o)
        Nn.append(cn)
        UV.append(uv.reshape(-1, 2))
        M += [names[i] for i in mi]
        o += nv
    return (np.concatenate(P), np.concatenate(F), np.concatenate(Nn), np.concatenate(UV), M)


def _cache(out):
    """Intermediate geometry (npz) lives outside the kit folder so the repo only gets the USD + PNGs."""
    import hashlib
    import tempfile
    d = os.path.join(tempfile.gettempdir(), "facade_kit_build_" + hashlib.md5(out.encode()).hexdigest()[:8])
    os.makedirs(d, exist_ok=True)
    return os.path.join(d, "kit_geo.npz")


def build_kit(out, seed=7, log=print):
    bpy = _bpy()
    rng = np.random.default_rng(seed)
    mods = {}
    for kind, (fn, nvar) in KIT.items():
        for v in range(nvar):
            parts = fn(rng, v)
            P, F, Nn, UV, M = collect(parts)
            for ob in parts:
                me = ob.data
                bpy.data.objects.remove(ob)
                bpy.data.meshes.remove(me)
            name = f"{kind}_{v}"
            mods[name] = dict(P=P, F=F, N=Nn, UV=UV, M=M)
            lo, hi = P.min(0), P.max(0)
            log(f"[kit] {name}: {len(F)} tris, mats {sorted(set(M))}, bounds {lo.round(2)} .. {hi.round(2)}")
    os.makedirs(out, exist_ok=True)
    np.savez_compressed(_cache(out), **{f"{k}__{a}": (np.asarray(v[a]) if a != "M" else np.asarray(v[a]))
                                                              for k, v in mods.items() for a in ("P", "F", "N", "UV", "M")})
    return mods


# ================================================================================================ USD side
def write_usd(out, log=print):
    """facade_kit.usdc: /FacadeKit/<module> (Xform) / geo (Mesh, GeomSubsets per material) + Looks."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt
    meta = json.load(open(os.path.join(out, "kit.json")))
    Z = np.load(_cache(out))
    path = os.path.join(out, "facade_kit.usdc")
    st = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = UsdGeom.Xform.Define(st, "/FacadeKit")
    st.SetDefaultPrim(root.GetPrim())
    root.GetPrim().SetCustomDataByKey("frame", "x=U bay 3.0 m, y=V storey 2.9 m, z=N outward; origin bay centre "
                                               "on the face plane (see isaac/tools/blender_facade_kit.py)")

    def material(mpath, name):
        mat = UsdShade.Material.Define(st, mpath)
        sh = UsdShade.Shader.Define(st, mpath + "/PBR")
        sh.CreateIdAttr("UsdPreviewSurface")
        if name in CONST:
            rgb, r, m = CONST[name]
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(r)
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(m)
            if name == "glass":
                sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.35)
        else:
            files = meta["textures"][name]
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
                if kind == "albedo" and name in ALBEDO_SCALE:
                    k = ALBEDO_SCALE[name]
                    tx.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(k, k, k, 1))
                if kind == "normal":
                    tx.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(2, 2, 2, 1))
                    tx.CreateInput("bias", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(-1, -1, -1, 0))
                    o = tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
                elif kind == "rough":
                    o = tx.CreateOutput("r", Sdf.ValueTypeNames.Float)
                else:
                    o = tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
                sh.CreateInput(inp, typ).ConnectToSource(o)
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return mat

    n_tri = 0
    for name in meta["modules"]:
        P, F, Nn, UV = (Z[f"{name}__{a}"] for a in ("P", "F", "N", "UV"))
        M = [str(m) for m in Z[f"{name}__M"]]
        UsdGeom.Xform.Define(st, f"/FacadeKit/{name}")
        mesh = UsdGeom.Mesh.Define(st, f"/FacadeKit/{name}/geo")
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(P, np.float32)))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(F), 3, np.int32)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.ascontiguousarray(F.reshape(-1), np.int32)))
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(False)
        mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(Nn, np.float32)))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                                     UsdGeom.Tokens.faceVarying)
        pv.Set(Vt.Vec2fArray.FromNumpy(np.ascontiguousarray(UV, np.float32)))
        mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, P.min(0))), Gf.Vec3f(*map(float, P.max(0)))]))
        mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.3, 0.29, 0.27)]))
        names = sorted(set(M))
        Ma = np.array(M)
        for nm in names:
            mat = material(f"/FacadeKit/{name}/Looks/{nm}", nm)
            idx = np.nonzero(Ma == nm)[0].astype(np.int32)
            if len(names) == 1:
                UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
            else:
                sub = UsdGeom.Subset.CreateGeomSubset(mesh, nm, UsdGeom.Tokens.face, Vt.IntArray.FromNumpy(idx),
                                                      "materialBind", UsdGeom.Tokens.partition)
                UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mat)
        n_tri += len(F)
    st.GetRootLayer().Save()
    log(f"[kit] wrote {path}: {len(meta['modules'])} modules, {n_tri} triangles")
    return path


# ================================================================================================ preview
def _preview_material(name, meta, out, tint=None):
    bpy = _bpy()
    mat = bpy.data.materials.new("pv_" + name)
    mat.use_nodes = True
    T = Nodes(mat)
    bsdf = T.n("ShaderNodeBsdfPrincipled")
    outn = T.n("ShaderNodeOutputMaterial")
    T.t.links.new(bsdf.outputs[0], outn.inputs["Surface"])
    if name in CONST:
        rgb, r, m = CONST[name]
        bsdf.inputs["Base Color"].default_value = (*rgb, 1)
        bsdf.inputs["Roughness"].default_value = r
        bsdf.inputs["Metallic"].default_value = m
        return mat
    files = meta["textures"][name]
    uvn = T.n("ShaderNodeUVMap", uv_map="st")
    imgs = {}
    for kind in ("albedo", "rough", "normal"):
        im = bpy.data.images.load(os.path.join(out, files[kind]), check_existing=True)
        im.colorspace_settings.name = "sRGB" if kind == "albedo" else "Non-Color"
        nd = T.n("ShaderNodeTexImage", image=im)
        T.t.links.new(uvn.outputs[0], nd.inputs[0])
        imgs[kind] = nd
    k = ALBEDO_SCALE.get(name, 1.0)
    T._in(bsdf.inputs["Base Color"], T.mix(1.0, imgs["albedo"].outputs[0], (k, k, k), "MULTIPLY"))
    T.t.links.new(imgs["rough"].outputs[0], bsdf.inputs["Roughness"])
    nm = T.n("ShaderNodeNormalMap", uv_map="st")
    T.t.links.new(imgs["normal"].outputs[0], nm.inputs["Color"])
    T.t.links.new(nm.outputs[0], bsdf.inputs["Normal"])
    return mat


def _module_mesh(name, d, mats):
    bpy = _bpy()
    P, F, Nn, UV, M = d["P"], d["F"], d["N"], d["UV"], list(map(str, d["M"]))
    me = bpy.data.meshes.new(name)
    me.from_pydata(P.tolist(), [], F.tolist())
    uv = me.uv_layers.new(name="st")
    uv.data.foreach_set("uv", UV.reshape(-1))
    names = sorted(set(M))
    for nm in names:
        me.materials.append(mats[nm])
    me.polygons.foreach_set("material_index", np.array([names.index(m) for m in M], np.int32))
    me.polygons.foreach_set("use_smooth", np.ones(len(F), bool))
    me.normals_split_custom_set(Nn.tolist())
    me.update()
    return me


def preview(out, dest, log=print, samples=48, res=(1280, 720)):
    """Cycles CPU renders: (a) the kit laid out, (b) a fallen-tower face section (10 bays x 6 storeys) placed
    with isaac/facade_kit.plan_face, from 25 m and 60 m at 35 deg down, afternoon sun."""
    bpy = _bpy()
    from mathutils import Matrix, Vector
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
    from isaac import facade_kit as FK
    meta = json.load(open(os.path.join(out, "kit.json")))
    Z = np.load(_cache(out))
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o)
    names = set()
    for k in Z.files:
        if k.endswith("__M"):
            names |= set(map(str, Z[k]))
    mats = {n: _preview_material(n, meta, out) for n in names}
    meshes = {m: _module_mesh(m, {a: Z[f"{m}__{a}"] for a in ("P", "F", "N", "UV", "M")}, mats)
              for m in meta["modules"]}
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = samples
    sc.cycles.use_denoising = True
    sc.cycles.max_bounces = 6
    sc.render.resolution_x, sc.render.resolution_y = res
    sc.render.film_transparent = False
    sc.view_settings.view_transform = "AgX"
    sc.view_settings.look = "AgX - Medium High Contrast" if "AgX - Medium High Contrast" in [
        i.identifier for i in sc.view_settings.bl_rna.properties["look"].enum_items] else "None"
    # world: hazy afternoon sky
    w = bpy.data.worlds.new("w")
    sc.world = w
    w.use_nodes = True
    T = Nodes(w)
    tc = T.n("ShaderNodeTexCoord")
    sep = T.n("ShaderNodeSeparateXYZ")
    T.t.links.new(tc.outputs["Generated"], sep.inputs[0])
    sky = T.ramp(sep.outputs["Z"], [(0.0, (0.66, 0.62, 0.55)), (0.2, (0.62, 0.63, 0.64)), (1.0, (0.42, 0.50, 0.62))])
    bg = T.n("ShaderNodeBackground")
    T.t.links.new(sky, bg.inputs[0])
    bg.inputs[1].default_value = 0.45
    wo = T.n("ShaderNodeOutputWorld")
    T.t.links.new(bg.outputs[0], wo.inputs[0])
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sun.data.energy = 4.0
    sun.data.color = (1.0, 0.86, 0.70)                          # late-afternoon, through dust
    sun.data.angle = math.radians(1.0)
    sun.rotation_euler = (math.radians(52), 0, math.radians(225 - 180 + 90))
    sc.collection.objects.link(sun)
    ground_mat = mats.get("fracture")
    gme = bpy.data.meshes.new("ground")
    S = 120
    gme.from_pydata([(-S, -S, 0), (S, -S, 0), (S, S, 0), (-S, S, 0)], [], [(0, 1, 2, 3)])
    uv = gme.uv_layers.new(name="st")
    for i, (u, v) in enumerate([(0, 0), (S / 1.5, 0), (S / 1.5, S / 1.5), (0, S / 1.5)]):
        uv.data[i].uv = (u, v)
    gme.materials.append(ground_mat)
    ground = bpy.data.objects.new("ground", gme)
    sc.collection.objects.link(ground)
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    cam.data.lens = 35
    cam.data.clip_end = 2000
    sc.collection.objects.link(cam)
    sc.camera = cam

    def look(eye, tgt):
        d = Vector(tgt) - Vector(eye)
        cam.location = eye
        cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()

    def inst(mname, M4, coll="tmp"):
        ob = bpy.data.objects.new(mname, meshes[mname])
        ob.matrix_world = M4
        sc.collection.objects.link(ob)
        return ob

    os.makedirs(dest, exist_ok=True)
    outs = []
    # (a) kit layout
    tmp = []
    rows = list(KIT.keys())
    yrow = {"slab": -7.0, "wall": -3.6, "void": 0.2, "column": 3.6, "beam": 3.6, "debris": 6.2}
    for kind in rows:
        n = KIT[kind][1]
        x0 = {"column": -8.0, "beam": 4.0}.get(kind, -(n - 1) / 2 * 3.5)
        for v in range(n):
            dx = 2.0 if kind in ("column", "beam", "debris") else 3.5
            M4 = Matrix.Translation(Vector((x0 + v * dx, yrow[kind], 0.0)))
            tmp.append(inst(f"{kind}_{v}", M4))
    look((0.0, -16.5, 10.5), (0.0, 0.6, 0.0))
    cam.data.lens = 24
    sc.render.filepath = os.path.join(dest, "kit_layout.png")
    bpy.ops.render.render(write_still=True)
    outs.append(sc.render.filepath)
    log(f"[kit] preview {sc.render.filepath}")
    for o in tmp:
        bpy.data.objects.remove(o)
    # (b) fallen-tower section, 10 bays x 6 storeys: the upper side face of a tower lying on its side (30 deg
    # roll), dark body underneath, rubble at its foot; placed with isaac/facade_kit.plan_face like Isaac does
    rng = np.random.default_rng(5)
    variants = FK.default_variants(meta["modules"])

    def quad_mesh(name, P, mat):
        me = bpy.data.meshes.new(name)
        me.from_pydata([tuple(map(float, p)) for p in P], [], [tuple(range(len(P)))])
        me.uv_layers.new(name="st")
        me.materials.append(mats[mat])
        ob = bpy.data.objects.new(name, me)
        sc.collection.objects.link(ob)
        return ob

    def place(A0, A1, B0, B1, U, V, N, nst, nb, zmin):
        quad_mesh("face", [A0, A1, B1, B0], "interior")
        n = 0
        for kind, var, pos, R, s_ in FK.plan_face(A0, A1, B0, B1, U, V, N, nst, nb, rng, variants, zmin=zmin):
            M4 = (Matrix(R.tolist()) @ Matrix.Diagonal(Vector(s_))).to_4x4()
            M4.translation = Vector(pos)
            inst(f"{kind}_{var}", M4)
            n += 1
        return n

    def rubble(P0, P1, n, spread):
        for k in range(n):
            p = P0 + (P1 - P0) * rng.uniform(0, 1) + np.array([rng.normal(0, spread), rng.normal(0, 0.6), 0.0])
            p[2] = 0.0
            k_ = rng.uniform(0.6, 1.5)
            M4 = Matrix.Translation(Vector(p.tolist())) @ Matrix.Rotation(rng.uniform(0, 6.28), 4, "Z") @ \
                Matrix.Diagonal(Vector((k_, k_, k_, 1)))
            inst(f"debris_{rng.integers(variants['debris'])}", M4)

    def shoot(ctr, N, dist, lens, name, el_deg=35.0, az_off=-35.0):
        el = math.radians(el_deg)
        az = math.atan2(N[1], N[0]) + math.radians(az_off)
        d = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
        cam.data.lens = lens
        look(tuple(ctr + d * dist), tuple(ctr))
        sc.render.filepath = os.path.join(dest, name)
        bpy.ops.render.render(write_still=True)
        outs.append(sc.render.filepath)
        log(f"[kit] preview {sc.render.filepath}")

    nb, nst = 10, 6
    W, L = nb * 3.0, nst * 2.9
    roll = math.radians(30)
    U = np.array([math.cos(roll), 0.0, math.sin(roll)])
    V = np.array([0.0, 1.0, 0.0])
    N = np.cross(U, V)
    N = N if N[2] > 0 else -N
    V = np.cross(N, U)
    A0 = np.array([-W / 2 * math.cos(roll), -L / 2, 0.0])
    A1 = A0 + U * W
    B0, B1 = A0 + V * L, A1 + V * L
    before = set(bpy.data.objects)
    place(A0, A1, B0, B1, U, V, N, nst, nb, zmin=-1.0)
    G = [np.array([A1[0], A1[1], 0.0]), np.array([B1[0], B1[1], 0.0])]   # body: wedge down to the ground
    quad_mesh("body", [A1, G[0], G[1], B1], "fracture")
    rubble(A0, B0, 70, 1.2)
    rubble(G[0], G[1], 50, 1.5)
    ctr = (A0 + B1) / 2 + N * 0.6
    shoot(ctr, N, 25.0, 24, "tower_section_25m.png")
    shoot(ctr, N, 60.0, 30, "tower_section_60m.png")
    for o in set(bpy.data.objects) - before:
        bpy.data.objects.remove(o)
    # (c) the real west tower of the scanned map (its four faces as urban_look.facade frames them), 40 m
    try:
        from isaac import urban_look as UL
        Pw = np.array([[-37.82, -24.18, 1.2], [-29.45, -24.18, 9.57], [-28.83, 21.86, 9.57], [-37.2, 21.86, 1.2],
                       [-29.45, -24.18, 9.57], [-24.49, -24.18, 4.61], [-23.87, 21.86, 4.61], [-28.83, 21.86, 9.57],
                       [-24.49, -24.18, 4.61], [-32.86, -24.18, -3.76], [-32.24, 21.86, -3.76], [-23.87, 21.86, 4.61],
                       [-32.86, -24.18, -3.76], [-37.82, -24.18, 1.2], [-37.2, 21.86, 1.2], [-32.24, 21.86, -3.76]])
        quads = []
        for k in range(4):
            Q = Pw[4 * k:4 * k + 4]
            n = np.cross(Q[1] - Q[0], Q[2] - Q[0])
            quads.append((Q, n / np.linalg.norm(n)))
        centre = Pw.mean(0)
        for Q, n in quads:
            A0, A1, B0, B1, U, V, N = UL._frame(Q, n, centre)
            Wq, Lq = np.linalg.norm(A1 - A0), np.linalg.norm(B0 - A0)
            nst_, nb_ = max(2, int(round(Lq / UL.STOREY_M))), max(2, int(round(Wq / 3.0)))
            place(A0, A1, B0, B1, U, V, N, nst_, nb_, zmin=-0.4)
        rubble(np.array([-39.0, -24.0, 0]), np.array([-39.0, 22.0, 0]), 90, 2.0)
        rubble(np.array([-22.5, -24.0, 0]), np.array([-22.5, 22.0, 0]), 90, 2.0)
        top = np.array([-30.0, -6.0, 6.0])
        shoot(top, np.array([-0.7, -0.3, 0.6]), 45.0, 28, "real_W_tower_45m.png", el_deg=32.0, az_off=0.0)
    except Exception as e:
        log(f"[kit] real-tower preview skipped ({type(e).__name__}: {e})")
    return outs


# ================================================================================================ main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="kit folder (facade_kit.usdc + textures/)")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--stage", choices=["all", "build", "usd", "preview"], default="all")
    ap.add_argument("--pxr-python", default="", help="python with usd-core, if pxr is not importable here")
    ap.add_argument("--preview", default="", help="folder for Cycles preview renders (needs bpy)")
    ap.add_argument("--samples", type=int, default=48)
    ap.add_argument("--res", default="1280x720")
    a = ap.parse_args(argv)
    out = os.path.abspath(a.out)
    if a.stage in ("all", "build"):
        bpy = _bpy()
        bpy.ops.wm.read_factory_settings(use_empty=True)
        tex = bake_textures(out)
        mods = build_kit(out, a.seed)
        json.dump({"modules": list(mods.keys()), "textures": tex, "seed": a.seed,
                   "frame": "x=U (bay 3.0 m), y=V (storey 2.9 m), z=N outward; origin bay centre on face plane",
                   "bay": BAY, "storey": STOREY}, open(os.path.join(out, "kit.json"), "w"), indent=1)
    if a.stage in ("all", "usd"):
        try:
            import pxr  # noqa: F401
            ok = a.stage == "usd" or not a.pxr_python
        except Exception:
            ok = False
        if ok:
            write_usd(out)
        elif a.pxr_python:
            subprocess.check_call([a.pxr_python, os.path.abspath(__file__), "--out", out, "--stage", "usd"])
        else:
            print("[kit] pxr not importable: run again with --stage usd under a python with usd-core")
    if a.preview and a.stage in ("all", "preview"):
        w, h = map(int, a.res.split("x"))
        if a.stage == "preview":
            _bpy().ops.wm.read_factory_settings(use_empty=True)
        preview(out, a.preview, samples=a.samples, res=(w, h))


if __name__ == "__main__":
    main()
