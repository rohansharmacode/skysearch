"""LiDAR-style point renders of the Weiguan collapse-site map (assets/maps/collapse_site_heightmap.npz)."""
import sys, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
HM, OUT = sys.argv[1], sys.argv[2]
d = np.load(HM, allow_pickle=True)
z, g, res, (ox, oy) = d["z"].astype(float), d["ground"].astype(float), float(d["res"]), d["origin"]
H, W = z.shape
rng = np.random.default_rng(7)
BG, FG, MUTED, PURPLE = "#0f141b", "#e8ecf1", "#9aa4b2", "#9b6bff"
cmap = matplotlib.colors.LinearSegmentedColormap.from_list("lid", ["#241552", "#5b3bb8", PURPLE, "#d9c8ff", "#ffffff"])

def points(x0, x1, y0, y1, per_cell):
    j0, j1 = int((x0 - ox) / res), int((x1 - ox) / res); i0, i1 = int((y0 - oy) / res), int((y1 - oy) / res)
    j0, i0 = max(j0, 0), max(i0, 0); j1, i1 = min(j1, W - 1), min(i1, H - 1)
    I, J = np.mgrid[i0:i1, j0:j1]
    I, J = np.repeat(I.ravel(), per_cell), np.repeat(J.ravel(), per_cell)
    fy, fx = rng.random(I.size), rng.random(J.size)
    keep = rng.random(I.size) < 0.85                                # scan-line dropouts
    I, J, fx, fy = I[keep], J[keep], fx[keep], fy[keep]
    zz = z[I, J] + rng.normal(0, 0.03, I.size)                      # ~3 cm range noise
    hag = np.clip(z[I, J] - g[I, J], 0, None)                        # height above ground
    return ox + (J + fx) * res, oy + (I + fy) * res, zz, hag

def render(x, y, zz, hag, fname, title, sub, az=-38, el=34, size=0.5, crop=None):
    a, e = np.radians(az), np.radians(el)
    xr = x * np.cos(a) - y * np.sin(a); yr = x * np.sin(a) + y * np.cos(a)
    sx, sy = xr, yr * np.sin(e) + zz * np.cos(e)
    o = np.argsort(-(yr * np.cos(e) - zz * np.sin(e)))
    c = np.clip(0.18 + 0.82 * hag / max(np.percentile(hag, 99.3), 1e-6), 0, 1)
    fig = plt.figure(figsize=(16, 9), dpi=150, facecolor=BG)
    ax = fig.add_axes([0, 0, 1, 0.86] if title else [0, 0, 1, 1]); ax.set_facecolor(BG); ax.set_axis_off()
    ax.scatter(sx[o], sy[o], c=c[o], cmap=cmap, vmin=0, vmax=1, s=size, linewidths=0)
    ax.set_aspect("equal")
    lx, hx = np.percentile(sx, [1, 99]); cy = np.percentile(sy, 50); hh = (hx - lx) * ((0.86 if title else 1) * 9 / 16) / 2
    ax.set_xlim(lx, hx); ax.set_ylim(cy - hh, cy + hh)
    if title:
        fig.text(0.02, 0.945, title, color=FG, fontsize=26, weight="bold")
        fig.text(0.02, 0.90, sub, color=MUTED, fontsize=13)
    fig.savefig(f"{OUT}/{fname}", facecolor=BG); plt.close(fig)

xmin, ymin = ox, oy; xmax, ymax = ox + W * res, oy + H * res
x, y, zz, hag = points(xmin, xmax, ymin, ymax, 2)
n_full = x.size
render(x, y, zz, hag, "lidar_collapse_site.png", "LiDAR map of the Weiguan Jinlong collapse site",
       f"{n_full:,} laser points · 320 m × 320 m · colour = height above street · drones use this map to find safe spots for the radar pod")
render(x, y, zz, hag, "lidar_collapse_site_clean.png", None, None)
# close-up of the collapsed tower core (densest rubble)
hagmap = np.clip(z - g, 0, None)
iy, ix = np.unravel_index(np.argmax(np.convolve(hagmap.sum(1), np.ones(40), "same")[:, None] * np.convolve(hagmap.sum(0), np.ones(40), "same")[None, :]), hagmap.shape)
cx, cy = 0.0, 0.0   # collapse core is centred on the site origin (isaac/maps/collapse_core_0p5m.npz: origin -80,-80, 160 m)
x, y, zz, hag = points(cx - 45, cx + 45, cy - 45, cy + 45, 12)
render(x, y, zz, hag, "lidar_collapse_core.png", "LiDAR close-up: the collapsed towers",
       f"{x.size:,} laser points over 90 m × 90 m · each point is one laser return · white = highest rubble", az=-30, el=38, size=0.9)
render(x, y, zz, hag, "lidar_collapse_core_clean.png", None, None, az=-30, el=38, size=0.9)
print("ok", n_full)
