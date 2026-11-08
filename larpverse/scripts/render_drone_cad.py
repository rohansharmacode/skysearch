"""CAD-style renders of the heavy SAR drone USD (isaac/assets/drone/heavy_sar.usdc): software rasteriser (painter's
algorithm, flat Lambert shading, 2x supersampling), orthographic views + isometric hero, labelled component callouts."""
import sys, numpy as np
from pxr import Usd, UsdGeom, UsdShade
from PIL import Image, ImageDraw, ImageFont
USD, OUT = sys.argv[1], sys.argv[2]
st = Usd.Stage.Open(USD); cache = UsdGeom.XformCache()

def mat_color(prim):
    mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    if mat:
        for p in Usd.PrimRange(mat.GetPrim()):
            sh = UsdShade.Shader(p)
            if sh and sh.GetIdAttr().Get() == "UsdPreviewSurface":
                c = sh.GetInput("diffuseColor")
                if c and c.Get() is not None:
                    return np.array(c.Get(), float)
    dc = UsdGeom.Mesh(prim).GetDisplayColorAttr().Get()
    return np.array(dc[0], float) if dc else np.array([.5, .5, .5])

tris, cols, part_of = [], [], []
parts = {}
for p in st.Traverse():
    if not p.IsA(UsdGeom.Mesh) or "Blur" in p.GetPath().pathString:
        continue
    m = UsdGeom.Mesh(p)
    pts = np.array(m.GetPointsAttr().Get(), float)
    M = np.array(cache.GetLocalToWorldTransform(p))
    w = (np.c_[pts, np.ones(len(pts))] @ M)[:, :3]
    cnt = np.array(m.GetFaceVertexCountsAttr().Get()); idx = np.array(m.GetFaceVertexIndicesAttr().Get())
    part = p.GetPath().pathString.split("/")[2]
    part = "Rotor" if part.startswith("Rotor") else part
    col = np.clip(mat_color(p), 0, 1)
    o = 0
    for c in cnt:
        f = idx[o:o + c]; o += c
        for k in range(1, c - 1):
            tris.append(w[[f[0], f[k], f[k + 1]]]); cols.append(col); part_of.append(part)
    parts.setdefault(part, []).append(w)
T = np.array(tris); C = np.array(cols); P = np.array(part_of)
print("triangles", len(T))
lo, hi = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0)
centre = (lo + hi) / 2

def view_basis(name):
    if name == "top":   return np.array([1, 0, 0.]), np.array([0, 1, 0.]), np.array([0, 0, 1.])
    if name == "front": return np.array([0, -1, 0.]), np.array([0, 0, 1.]), np.array([1, 0, 0.])
    if name == "side":  return np.array([1, 0, 0.]), np.array([0, 0, 1.]), np.array([0, -1, 0.])
    az, el = np.radians(35), np.radians(28)                     # isometric-ish hero from front-right, above
    fwd = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])  # toward viewer
    right = np.cross(np.array([0, 0, 1.]), fwd); right /= np.linalg.norm(right)
    up = np.cross(fwd, right)
    return right, up, fwd

def render(name, W, H, bg, scale_pad=0.88, ss=2):
    r, u, f = view_basis(name)
    V = T - centre
    X, Y, Z = V @ r, V @ u, V @ f
    ext = max(np.ptp(X), np.ptp(Y) * W / H)
    s = scale_pad * W * ss / ext
    px = (X * s + W * ss / 2); py = (H * ss / 2 - Y * s)
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); nn = np.linalg.norm(n, axis=1, keepdims=True); nn[nn == 0] = 1; n = n / nn
    light = (0.35 * r + 0.55 * u + 0.75 * f); light /= np.linalg.norm(light)
    lam = np.abs(n @ light)                                       # two-sided
    shade = 0.32 + 0.68 * lam
    rgb = np.clip(C * shade[:, None] * 1.05 + 0.03, 0, 1)
    order = np.argsort(Z.mean(1))                                  # far first
    img = Image.new("RGB", (W * ss, H * ss), bg); d = ImageDraw.Draw(img)
    for i in order:
        poly = list(zip(px[i], py[i]))
        c = tuple(int(255 * v) for v in rgb[i])
        d.polygon(poly, fill=c, outline=c)
    img = img.resize((W, H), Image.LANCZOS)
    proj = lambda xyz: (((xyz - centre) @ r) * s / ss + W / 2, H / 2 - ((xyz - centre) @ u) * s / ss)
    return img, proj, s / ss

def font(sz, bold=False):
    for f in (["/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"] if bold else []) + ["/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"]:
        try: return ImageFont.truetype(f, sz)
        except OSError: pass
    return ImageFont.load_default()

LABELS = [("Lidar", "LiDAR (Ouster OS1-128 class), 45° mount"), ("GpsPuck", "RTK GNSS"), ("Antennas", "Mesh radio antennas (5.8 GHz)"),
          ("WifiAnt", "Rescue Wi-Fi hotspot (2.4 GHz)"), ("SpeakerHorn", "Loudspeaker (call-out)"), ("WinchDrum", "Winch + radar/mic pod tether"),
          ("MicCapsules", "8-mic array (perch-and-listen)"), ("ComputeBox", "Onboard compute (Jetson Orin class)"),
          ("Batteries", "Smart batteries (hot-swap)"), ("Motors", "8 motors, 4 coaxial pairs"), ("LandingGear", "Carbon landing gear"),
          ("Strobe", "Anti-collision strobe")]

def anchor(part):
    a = np.concatenate(parts[part]); return (a.min(0) + a.max(0)) / 2

# --- hero with callouts (light CAD sheet) ---
W, H = 2600, 1500
img, proj, s = render("iso", 1500, 1300, (246, 246, 243))
sheet = Image.new("RGB", (W, H), (246, 246, 243)); sheet.paste(img, (470, 120)); d = ImageDraw.Draw(sheet)
f_t, f_l, f_s = font(46, True), font(25), font(20)
d.text((60, 40), "LARPVERSE heavy SAR drone — component layout", fill=(25, 27, 30), font=f_t)
d.text((60, 100), "8 rotors (4 coaxial pairs) · LiDAR · winch-lowered radar + mic pod · loudspeaker · rescue Wi-Fi · mesh radio", fill=(90, 94, 100), font=f_s)
left = [l for l in LABELS if anchor(l[0]) @ np.array([0, -1, 0]) >= -0.02]
pts = []
for part, text in LABELS:
    x, y = proj(anchor(part)); pts.append((part, text, x + 470, y + 120))
pts.sort(key=lambda t: t[3])
L = [p for p in pts if p[2] < 470 + 750]; R = [p for p in pts if p[2] >= 470 + 750]
for side, group in (("L", L), ("R", R)):
    for k, (part, text, x, y) in enumerate(group):
        ty = 220 + k * (1150 / max(1, len(group)))
        bbox = d.textbbox((0, 0), text, font=f_l); tw = bbox[2] - bbox[0]
        if side == "L":
            tx = 50; ex = tx + tw + 14
        else:
            ex = 2010; tx = ex + 14
        d.line([(x, y), (ex, ty + 14)], fill=(217, 84, 30), width=3)
        d.ellipse([x - 7, y - 7, x + 7, y + 7], fill=(217, 84, 30))
        d.text((tx, ty), text, fill=(25, 27, 30), font=f_l)
span = hi[0] - lo[0]; height = hi[2] - lo[2]
d.text((60, H - 70), f"Model span {span:.2f} m × {hi[1]-lo[1]:.2f} m, height {height:.2f} m · {len(T):,} triangles · source isaac/assets/drone/heavy_sar.usdc (Blender-built, Isaac Sim asset)", fill=(90, 94, 100), font=f_s)
sheet.save(f"{OUT}/drone_cad_callouts.png")

# --- orthographic sheet: top / front / side ---
views = [("top", "TOP"), ("front", "FRONT"), ("side", "SIDE")]
sheet = Image.new("RGB", (2400, 900), (246, 246, 243)); d = ImageDraw.Draw(sheet)
for i, (v, lab) in enumerate(views):
    im, pj, sc = render(v, 760, 760, (246, 246, 243))
    sheet.paste(im, (20 + i * 790, 90))
    d.text((40 + i * 790, 30), lab, fill=(25, 27, 30), font=font(36, True))
    d.rectangle([20 + i * 790, 90, 780 + i * 790, 850], outline=(190, 190, 185), width=2)
d.text((40, 860), f"Orthographic views · span {span:.2f} m · height {height:.2f} m", fill=(90, 94, 100), font=font(22))
sheet.save(f"{OUT}/drone_cad_orthographic.png")

# --- hero renders, dark and transparent-ish ---
im, _, _ = render("iso", 1920, 1200, (15, 20, 27)); im.save(f"{OUT}/drone_hero_dark.png")
im, _, _ = render("iso", 1920, 1200, (255, 255, 255)); im.save(f"{OUT}/drone_hero_white.png")
print("done")
