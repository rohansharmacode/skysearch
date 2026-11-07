"""Shared layout + coordinate frames for the collapse-site build.

World frame (matches docs/ISAAC_HANDOFF.md): metres, x = east, y = north, z = up.
Origin = centre of the photogrammetry scan (top.png pixel (412, 384)).
Photo scale: one storey (3.1 m) = 20 px  ->  0.155 m / px.
"""
import numpy as np

MPP = 0.155                 # metres per photo pixel
U0, V0 = 412.0, 384.0       # photo pixel at world origin
EXTENT = 160.0              # world spans [-160, 160] in x and y  (core ~105 m -> ~3x)
TEX = 8192                  # global albedo canvas (3.9 cm / px)
TS = 2 * EXTENT / TEX       # metres per canvas px
HRES = 0.125                # heightfield spacing (m)
HN = int(round(2 * EXTENT / HRES)) + 1   # 2561 samples
NTILE = 4                   # 4 x 4 terrain tiles of 80 m


def px2w(u, v):
    return (np.asarray(u, float) - U0) * MPP, (V0 - np.asarray(v, float)) * MPP


def w2px(x, y):
    return np.asarray(x, float) / MPP + U0, V0 - np.asarray(y, float) / MPP


def w2c(x, y):
    """world -> global canvas pixel (col, row)"""
    return (np.asarray(x, float) + EXTENT) / TS - 0.5, (EXTENT - np.asarray(y, float)) / TS - 0.5


def w2h(x, y):
    """world -> heightfield index (col, row); row 0 = north edge"""
    return (np.asarray(x, float) + EXTENT) / HRES, (EXTENT - np.asarray(y, float)) / HRES


def poly_w(pts):
    u, v = zip(*pts)
    x, y = px2w(u, v)
    return np.stack([x, y], 1)


# --------------------------------------------------------------------------------------
# Core (photo) structures, in photo pixels
# --------------------------------------------------------------------------------------
CORE_BUILDINGS = [
    # name, polygon(px), height(m), wall texture key, roof style
    ("NW_warehouse", [(142,56),(276,80),(352,98),(350,162),(176,170),(140,192)], 7.0, "metal_white", "flat"),
    ("N_site_office", [(412,47),(476,47),(476,97),(412,97)], 4.0, "plaster_light", "flat"),
    ("N_damaged_hall", [(435,152),(518,152),(518,200),(435,200)], 5.5, "plaster_grey", "flat"),
    ("N_shed", [(521,146),(556,146),(556,206),(521,206)], 8.0, "plaster_grey", "flat"),
    ("NE_hut", [(686,166),(726,166),(726,208),(686,208)], 4.0, "plaster_light", "flat"),
    ("E_tank", [(706,232),(734,232),(734,284),(706,284)], 3.0, "metal_white", "flat"),
    ("Block6_remnant", [(246,158),(300,158),(300,242),(246,242)], 9.0, "facade_balcony", "flat"),
    ("W_long_shed", [(82,428),(136,430),(136,650),(78,650)], 5.0, "metal_white", "flat"),
    ("S_warehouse", [(238,622),(498,626),(498,692),(238,688)], 6.5, "metal_green", "flat"),
    ("SE_red_roof", [(606,458),(700,458),(700,548),(606,548)], 6.5, "plaster_light", "gable_x"),
    ("SE_white", [(698,458),(732,460),(732,546),(698,546)], 6.0, "plaster_light", "flat"),
    ("SE_tower", [(650,598),(746,598),(746,650),(650,650)], 24.0, "facade_tower", "flat"),
    ("SE_green_roof", [(608,656),(704,656),(704,728),(608,728)], 5.0, "plaster_grey", "flat"),
]

# Fallen 16-storey towers: tent-shaped prisms (west edge, ridge, east edge) along y.
# Each: y0, y1, (xw0, xw1), (xr0, xr1), (xe0, xe1), (hw, hr, he)
SLABS = [
    ("W_tower", 243, 540, (172, 168), (226, 222), (258, 254), (2.5, 11.0, 3.5)),
    ("E_tower", 248, 550, (456, 452), (522, 519), (560, 549), (2.5, 12.0, 4.0)),
]

# Broken floor-plates in the middle: polygon + (h_start, h_end) ramp along the first edge normal
PLATES = [
    ("P1", [(330,305),(372,300),(381,382),(344,386)], 6.0, 2.0),
    ("P2", [(318,392),(350,398),(330,548),(302,542)], 6.0, 3.5),
    ("P3", [(360,410),(388,412),(372,520),(346,516)], 4.5, 2.5),
    ("P4", [(352,515),(430,518),(428,545),(350,542)], 3.5, 2.5),
]

# Rubble zones: polygon(px), base height (m)
RUBBLE_ZONES = [
    ("main", [(250,245),(455,245),(455,560),(250,560)], 3.2),
    ("north", [(240,160),(430,200),(560,205),(585,250),(455,250),(250,250),(165,200)], 2.0),
    ("west", [(125,195),(172,205),(170,560),(132,585),(118,420)], 1.3),
    ("east", [(556,228),(600,248),(612,560),(552,560)], 1.6),
    ("south", [(150,540),(600,540),(600,618),(150,612)], 1.8),
    ("yard", [(560,250),(745,250),(745,450),(600,455)], 0.35),
    ("north_yard", [(355,100),(745,95),(745,240),(560,205),(430,195),(355,165)], 0.25),
]

# Small objects baked into the heightfield: polygon(px), height
CORE_BUMPS = [
    ([(560,70),(582,70),(582,108),(560,108)], 2.6),   # container
    ([(584,72),(603,72),(603,110),(584,110)], 2.6),   # container
    ([(518,74),(533,74),(533,100),(518,100)], 1.5),   # car
    ([(538,74),(553,74),(553,100),(538,100)], 1.5),   # car
    ([(612,50),(642,50),(642,100),(612,100)], 2.2),   # excavator
    ([(640,388),(716,392),(716,408),(640,404)], 1.0),  # pipe
    ([(544,650),(566,650),(566,700),(544,700)], 2.4),  # truck
]

# Regions where the photo shows a leaning facade on the ground (re-synthesise there)
LEAN_FIX = [
    [(612,548),(746,548),(746,598),(650,598),(650,652),(612,652)],
]

# --------------------------------------------------------------------------------------
# Outer (extrapolated) layout, world metres
# --------------------------------------------------------------------------------------
CORE_HALF = 58.0
STREETS_X = [(-112.5, 9.0), (-63.5, 11.0), (63.5, 11.0), (112.5, 9.0)]   # N-S streets at x
STREETS_Y = [(-112.5, 9.0), (-63.5, 11.0), (63.5, 11.0), (112.5, 9.0)]   # E-W streets at y
ALLEY = 7.0


def street_mask_w(x, y):
    """1 on streets/alleys (world coords arrays)."""
    m = np.zeros(np.broadcast(x, y).shape, bool)
    for c, w in STREETS_X:
        m |= np.abs(x - c) < w / 2
    for c, w in STREETS_Y:
        m |= np.abs(y - c) < w / 2
    m |= (np.abs(x) < ALLEY / 2) & (np.abs(y) > 69)
    m |= (np.abs(y) < ALLEY / 2) & (np.abs(x) > 69)
    return m


def blocks():
    """Axis-aligned city blocks between streets (excluding the core)."""
    xs = [-160, -117, -108, -69, -58, 58, 69, 108, 117, 160]
    edges = [(-160, -117), (-108, -69), (-58, 58), (69, 108), (117, 160)]
    out = []
    for (x0, x1) in edges:
        for (y0, y1) in edges:
            if (x0, x1) == (-58, 58) and (y0, y1) == (-58, 58):
                continue
            # split by alleys
            xsplits = [(x0, x1)]
            ysplits = [(y0, y1)]
            if (x0, x1) == (-58, 58):
                xsplits = [(-58, -ALLEY / 2), (ALLEY / 2, 58)]
            if (y0, y1) == (-58, 58):
                ysplits = [(-58, -ALLEY / 2), (ALLEY / 2, 58)]
            for a in xsplits:
                for b in ysplits:
                    out.append((a[0], b[0], a[1], b[1]))
    return out
