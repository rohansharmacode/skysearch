import sys, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle
PC, OUT = sys.argv[1], sys.argv[2]
BG, FG, MUTED = "#0f141b", "#e8ecf1", "#9aa4b2"
PURPLE, BLUE, YELLOW, TEAL, RED, ORANGE = "#9b6bff", "#4f8cff", "#f2c230", "#2bb3a3", "#ff4d5a", "#f07a3f"

# ---------- 1. LiDAR scan of the rubble (real points from a mission run) ----------
p = np.fromfile(PC, np.float32).reshape(-1, 3)
p = p[np.isfinite(p).all(1)]
z = p[:, 2]
az, el = np.radians(-50), np.radians(32)
x1 = p[:, 0] * np.cos(az) - p[:, 1] * np.sin(az)
y1 = p[:, 0] * np.sin(az) + p[:, 1] * np.cos(az)
sx, sy = x1, y1 * np.sin(el) + z * np.cos(el)
order = np.argsort(-(y1 * np.cos(el) - z * np.sin(el)))
lo_x, hi_x = np.percentile(sx, [3, 97]); cy = np.percentile(sy, 50); half_h = (hi_x - lo_x) * (0.86 * 9 / 16) / 2
lo_y, hi_y = cy - half_h, cy + half_h
fig = plt.figure(figsize=(16, 9), dpi=150, facecolor=BG)
ax = fig.add_axes([0, 0, 1, 0.86]); ax.set_facecolor(BG); ax.set_axis_off()
zn = np.clip(0.25 + 0.75*(z - np.percentile(z, 2)) / (np.percentile(z, 99.5) - np.percentile(z, 2)), 0, 1)
cmap = matplotlib.colors.LinearSegmentedColormap.from_list("lid", ["#2a1a55", PURPLE, "#d9c8ff", "#ffffff"])
ax.scatter(sx[order], sy[order], c=zn[order], cmap=cmap, s=2.2, linewidths=0, alpha=0.95)
ax.set_aspect("equal"); ax.set_xlim(lo_x, hi_x); ax.set_ylim(lo_y, hi_y)
fig.text(0.02, 0.945, "LiDAR scan of the rubble", color=FG, fontsize=26, weight="bold")
fig.text(0.02, 0.90, f"{len(p):,} laser points from the drones' 128-channel LiDAR · colour = height · used to find flat, safe spots for the radar pod",
        color=MUTED, fontsize=13)
fig.savefig(f"{OUT}/lidar_scan_rubble.png", facecolor=BG); plt.close(fig)
fig = plt.figure(figsize=(16, 9), dpi=150, facecolor=BG); ax = fig.add_axes([0, 0, 1, 1]); ax.set_facecolor(BG); ax.set_axis_off()
h2 = (hi_x - lo_x) * 9 / 16 / 2
ax.scatter(sx[order], sy[order], c=zn[order], cmap=cmap, s=2.2, linewidths=0, alpha=0.95); ax.set_aspect("equal"); ax.set_xlim(lo_x, hi_x); ax.set_ylim(cy - h2, cy + h2)
fig.savefig(f"{OUT}/lidar_scan_rubble_clean.png", facecolor=BG); plt.close(fig)

def box(ax, x, y, w, h, title, sub, col, fs=15):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.012,rounding_size=0.02", fc="#18202b", ec=col, lw=2.2))
    ax.text(x + w / 2, y + h * 0.64, title, ha="center", va="center", color=FG, fontsize=fs, weight="bold")
    ax.text(x + w / 2, y + h * 0.3, sub, ha="center", va="center", color=MUTED, fontsize=fs - 4.5, linespacing=1.3)

def arrow(ax, a, b, col, lab=None, off=(0, 0.025), lw=2.4, style="-|>", ls="-"):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle=style, mutation_scale=18, color=col, lw=lw, linestyle=ls))
    if lab:
        ax.text((a[0] + b[0]) / 2 + off[0], (a[1] + b[1]) / 2 + off[1], lab, ha="center", va="center", color=col, fontsize=11.5)

# ---------- 2. Rescue Wi-Fi riding on the existing drone mesh ----------
fig = plt.figure(figsize=(16, 9), dpi=150, facecolor=BG); ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.set_axis_off()
ax.text(0.03, 0.93, "Rescue Wi-Fi rides on the link the drones already use", color=FG, fontsize=26, weight="bold")
ax.text(0.03, 0.885, "No extra network: phone traffic shares the drones' 5.8 GHz mesh back to the van, alongside telemetry, maps and sensor alerts",
        color=MUTED, fontsize=13)
for i, (x, y) in enumerate([(0.30, 0.62), (0.47, 0.70), (0.64, 0.62)]):
    ax.add_patch(Circle((x, y), 0.135, fc=BLUE, alpha=0.08, ec=BLUE, lw=1.2, ls="--"))
    box(ax, x - 0.065, y - 0.045, 0.13, 0.09, f"Drone d{i+1}", "Wi-Fi AP 2.4 GHz\nmesh radio 5.8 GHz", BLUE, 13)
for i, (x, y, t) in enumerate([(0.17, 0.42, "Trapped phone\n“I'm trapped”"), (0.40, 0.40, "Phone under rubble"), (0.78, 0.43, "Rescuer phone")]):
    box(ax, x - 0.06, y - 0.04, 0.12, 0.08, "📱" if False else "Phone", t, BLUE, 12)
arrow(ax, (0.20, 0.465), (0.255, 0.585), BLUE, "2.4 GHz", (-0.04, 0))
arrow(ax, (0.41, 0.445), (0.45, 0.655), BLUE)
arrow(ax, (0.76, 0.475), (0.69, 0.585), BLUE)
arrow(ax, (0.365, 0.65), (0.405, 0.685), TEAL, style="<|-|>")
arrow(ax, (0.535, 0.685), (0.575, 0.65), TEAL, style="<|-|>")
ax.text(0.47, 0.80, "5.8 GHz mesh (multi-hop)", ha="center", color=TEAL, fontsize=12.5)
box(ax, 0.40, 0.14, 0.20, 0.13, "Command van", "mesh base station + mast\ncaptive portal · operator dashboard", ORANGE, 15)
arrow(ax, (0.64, 0.575), (0.56, 0.275), TEAL)
ax.text(0.615, 0.33, "same mesh link", color=TEAL, fontsize=12.5, rotation=64)
box(ax, 0.76, 0.15, 0.19, 0.11, "Internet", "satellite (OneWeb) or LTE\nmessages to family", MUTED, 14)
arrow(ax, (0.60, 0.205), (0.76, 0.205), MUTED, "uplink", (0, 0.03), style="<|-|>")
ax.text(0.03, 0.13, "Shared mesh link carries:", color=FG, fontsize=13, weight="bold")
for k, t in enumerate(["• flight telemetry + commands", "• LiDAR map updates", "• radar + audio alerts", "• phone portal traffic (rate-limited)"]):
    ax.text(0.03, 0.095 - k * 0.028, t, color=MUTED, fontsize=12)
fig.savefig(f"{OUT}/rescue_wifi_shared_mesh.png", facecolor=BG); plt.close(fig)

# ---------- 3. Audio pipeline: drone -> van local model (Argon 4) -> priority ----------
fig = plt.figure(figsize=(16, 9), dpi=150, facecolor=BG); ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.set_axis_off()
ax.text(0.03, 0.93, "How the drones' audio is understood", color=FG, fontsize=26, weight="bold")
ax.text(0.03, 0.885, "Filter on the drone, understand in the van: a local multilingual model (Argon 4) runs on the van's GPU, no internet needed",
        color=MUTED, fontsize=13)
ax.text(0.03, 0.80, "ON THE DRONE (rotors off)", color=YELLOW, fontsize=13, weight="bold")
box(ax, 0.03, 0.58, 0.17, 0.17, "8-mic array", "voices, knocks,\nbackground noise", YELLOW)
box(ax, 0.24, 0.58, 0.19, 0.17, "Noise filter", "rotor / generator / dog\nremoved or dismissed", YELLOW)
box(ax, 0.47, 0.58, 0.17, 0.17, "Direction finding", "bearing from each drone\n→ which pile", YELLOW)
arrow(ax, (0.20, 0.665), (0.24, 0.665), YELLOW); arrow(ax, (0.43, 0.665), (0.47, 0.665), YELLOW)
box(ax, 0.68, 0.58, 0.28, 0.17, "Only unusual sounds sent", "human-like clips forwarded over the mesh\n(seconds of audio, not a live stream)", YELLOW)
arrow(ax, (0.64, 0.665), (0.68, 0.665), YELLOW)
ax.text(0.03, 0.47, "IN THE VAN · Argon 4 (local)", color=TEAL, fontsize=13, weight="bold")
box(ax, 0.03, 0.22, 0.20, 0.20, "Language ID", "Mandarin · Taiwanese Hokkien\nEnglish · Indonesian\nVietnamese · Tagalog", TEAL, 14)
box(ax, 0.27, 0.22, 0.20, 0.20, "Transcribe + translate", "“Tolong! Kaki saya terjepit”\n→ “Help! My leg is trapped”", TEAL, 14)
box(ax, 0.51, 0.22, 0.20, 0.20, "Distress analysis", "call for help · pain · weak voice\nknock response", TEAL, 14)
box(ax, 0.75, 0.22, 0.21, 0.20, "Priority 0–100", "fused with the radar\nbreathing pattern\n→ dig order on dashboard", RED, 14)
arrow(ax, (0.82, 0.58), (0.13, 0.42), TEAL, "mesh link", (0.0, 0.03))
for a, b in ((0.23, 0.27), (0.47, 0.51), (0.71, 0.75)):
    arrow(ax, (a, 0.32), (b, 0.32), TEAL)
ax.text(0.03, 0.12, "Why local: the van keeps working with no internet after a quake, and the audio of trapped people never leaves the site. "
        "Rule-based fallback if the model is busy.", color=MUTED, fontsize=12.5)
ax.text(0.03, 0.08, "Every result is a place for rescuers to check, not a confirmed person.", color=MUTED, fontsize=12.5, style="italic")
fig.savefig(f"{OUT}/audio_pipeline_local_model.png", facecolor=BG); plt.close(fig)
print("ok")
