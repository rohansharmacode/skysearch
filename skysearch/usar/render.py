"""Mission-control view for the earthquake scenario (one 1920x1080 frame per call).

Left: what the fleet has mapped so far (fog of war), with every sensor drawn where it acts:
LiDAR beam ends, ultrasound rings, camera cones, microphone bearing cones and triangulated sound
fixes, breathing-radar pulses, the coarse radar-only map, smoke and fire, rooms and their search
status, victim pins with confidence, and the crew's safe route at handoff.
Faint dashed rings show where people really are (hidden from the drones) so viewers can judge.
Right: the system architecture, mission status, fleet table with live sensor health, what one
drone's sensors see right now, and the decision log (or the crew briefing at handoff).
"""
from __future__ import annotations
import math
import textwrap
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Circle, Ellipse, Wedge, RegularPolygon, FancyBboxPatch, Rectangle

from .building import OUT

BG = "#0E1117"
PANEL = "#151B24"
STATE_COL = {"MAP": "#4FC3F7", "VOID": "#F06292", "INVESTIGATE": "#FFD54F", "BEACON": "#66BB6A",
             "ASK HUMAN": "#FF8A65", "RELAY": "#B39DDB", "LISTEN": "#E1BEE7", "HOLD": "#EF5350",
             "RETURN": "#90A4AE", "CHARGE": "#90A4AE", "FAILED": "#E53935", "COORDINATE": "#FFD54F",
             "SURVEY": "#FFD54F", "IDLE": "#78909C", "TRAPPED": "#FF7043", "LISTENING (landed)": "#EA80FC",
             "TO POST": "#CE93D8", "PULL OUT": "#FF7043", "HOVER: mic + radar": "#00E5FF", "RADAR SWEEP": "#26C6DA",
             "CONFIRM": "#00E5FF"}
ROLE = {"coordinator": "coordinator · relay · speaker", "mapper": "LiDAR mapper + ultrasound",
        "radar": "UWB radar + LiDAR", "listener": "listener: mic array",
        "confirmer": "confirmer: RGB + mmWave"}
CONF_COL = {"HIGH": "#00E676", "MEDIUM": "#FFC400", "LOW": "#FF7043"}
ROOM_COL = {"unsearched": "#546E7A", "searching": "#FFCA28", "clear": "#66BB6A", "re-opened": "#FF7043",
            "victim": "#00E676"}
BREATH_HZ = {"A": 0.30, "B": 0.21, "C": 0.25}
SCENES = [("Mission", 1, "Launch"), ("listeners in position", 2, "Map + listen"), ("Knock heard", 3, "Knock → confirm"),
          ("Victim A", 3, "Knock → confirm"), ("Smoke", 4, "Chaos 1: smoke"), ("handed to radar", 5, "Radar takes over"),
          ("cleared room", 6, "Chaos 2: cleared room knocks"), ("Victim B", 6, "Chaos 2: cleared room knocks"),
          ("Victim C", 6, "Chaos 2: cleared room knocks"), ("Handing map", 7, "Hand map to crew")]


def map_rgba(clear, stale):
    """Colour each known cell by how much space there is to fly."""
    H, W = clear.shape
    img = np.zeros((H, W, 4))
    img[...] = (0.043, 0.055, 0.078, 1.0)                     # unknown: near black
    k = ~np.isnan(clear)
    c = np.nan_to_num(clear, nan=-1)

    def put(mask, rgb):
        img[mask & k, :3] = rgb
    put(c >= OUT, (0.10, 0.12, 0.15))                          # outside ground
    put((c >= 2.0) & (c < OUT), (0.20, 0.25, 0.31))            # open room
    put((c >= 0.9) & (c < 2.0), (0.27, 0.33, 0.40))            # under desks / low ceiling: scouts fit
    vm = (c >= 0.25) & (c < 0.9)                               # crevice void: only micro-drones fit
    t = np.clip((c - 0.25) / 0.65, 0, 1)
    img[vm & k, 0] = 0.55 + 0.35 * t[vm & k]
    img[vm & k, 1] = 0.33 + 0.25 * t[vm & k]
    img[vm & k, 2] = 0.08
    put((c >= 0.12) & (c < 0.25), (0.33, 0.24, 0.18))          # too tight for anything
    put((c >= 0) & (c < 0.12), (0.62, 0.65, 0.69))             # wall / solid rubble
    st = stale & k & (c < OUT)
    img[st, :3] = img[st, :3] * 0.55 + np.array([0.05, 0.08, 0.18])   # needs re-checking after aftershock
    return img


def health_col(v):
    if v is None:
        return None
    return "#66BB6A" if v >= 0.7 else "#FFCA28" if v >= 0.3 else "#EF5350"


class USARRenderer:
    def __init__(self, scen, show_truth=True, speed_label="", title="SkySearch USAR"):
        self.s = scen
        self.show_truth = show_truth
        self.speed_label = speed_label
        self.title = title
        self.fig = plt.figure(figsize=(19.2, 10.8), dpi=100, facecolor=BG)
        self.map_box = None
        self.pos = {}                 # optional interpolated drone positions for smooth video
        self.breath = []              # (t, drone, survivor name, solid cells, xy) radar detections
        self.focus = None
        self._last_note = None

    # ------------------------------------------------------------------ helpers
    def P(self, d):
        return self.pos.get(d.name, d.pos)

    def note_radar(self):
        f, b = self.s.fleet, self.s.b
        if self._last_note == f.t:
            return
        self._last_note = f.t
        for d in f.alive():
            for xy, solid in d.radar_dets:
                sv = min(b.survivors, key=lambda s: np.linalg.norm(s["pos"] - xy))
                self.breath.append((f.t, d.name, sv["name"], solid, np.array(xy)))
        self.breath = [x for x in self.breath if f.t - x[0] < 14]

    def latest_captions(self):
        caps = self.s.fleet.captions
        if not caps:
            return []
        t_last = caps[-1][0]
        return [c for c in caps if t_last - c[0] <= 4.0][-2:]

    def scene(self):
        sc = (1, "Launch")
        for t, m in self.s.fleet.captions:
            for key, n, lab in SCENES:
                if key in m and n >= sc[0]:
                    sc = (n, lab)
        return sc

    # ------------------------------------------------------------------ map
    def draw_map(self, ax):
        s, f, b, cfg = self.s, self.s.fleet, self.s.b, self.s.cfg
        cell = cfg.cell
        W, H = cfg.W * cell, cfg.H * cell
        ext = (0, W, H, 0)
        ax.imshow(map_rgba(f.cmap.clear, f.cmap.stale), extent=ext, interpolation="nearest", zorder=0)
        # coarse radar-only map: drawn as 0.8 m blocks (the radar's resolution)
        rc = f.radar_cells & ~f.seen
        if rc.any():
            Hc, Wc = cfg.H // 2, cfg.W // 2
            blk = rc[:Hc * 2, :Wc * 2].reshape(Hc, 2, Wc, 2).any(axis=(1, 3))
            ov = np.zeros((Hc, Wc, 4)); ov[..., 0], ov[..., 1], ov[..., 2] = 0.15, 0.78, 0.85
            ov[..., 3] = blk * 0.28
            ax.imshow(ov, extent=(0, Wc * 2 * cell, Hc * 2 * cell, 0), interpolation="nearest", zorder=1)
        # dust and smoke (physical, visible to everyone)
        dust = np.zeros((cfg.H, cfg.W, 4)); dust[..., 0], dust[..., 1], dust[..., 2] = 0.62, 0.52, 0.40
        dust[..., 3] = np.clip((b.dust - 0.18) * 0.9, 0, 0.6)
        ax.imshow(dust, extent=ext, interpolation="bilinear", zorder=1)
        if b.smoke.max() > 0.02:
            sm = np.zeros((cfg.H, cfg.W, 4)); sm[..., 0], sm[..., 1], sm[..., 2] = 0.55, 0.57, 0.60
            sm[..., 3] = np.clip(b.smoke * 0.85, 0, 0.72)
            ax.imshow(sm, extent=ext, interpolation="bilinear", zorder=2)
        # acoustic evidence
        A = f.acoustic
        if A.max() > 0.3:
            ac = np.zeros((cfg.H, cfg.W, 4)); ac[..., 0], ac[..., 1], ac[..., 2] = 0.85, 0.30, 1.0
            ac[..., 3] = np.clip(A / max(4.0, A.max()), 0, 1) * 0.5
            ax.imshow(ac, extent=ext, interpolation="bilinear", zorder=2)
        # building outline
        ax.add_patch(Rectangle((b.x0 * cell, b.y0 * cell), (b.x1 - b.x0) * cell, (b.y1 - b.y0) * cell,
                               fill=False, ec="#546E7A", lw=1.0, ls=":", zorder=3))
        # rooms and their search status
        handed = f.handoff_t is not None
        for rid, st in f.room_status.items():
            r = b.rooms[rid]
            (r0, r1), (c0, c1) = r["rows"], r["cols"]
            x, y, w, h = c0 * cell, r0 * cell, (c1 - c0) * cell, (r1 - r0) * cell
            col = ROOM_COL.get(st, "#546E7A")
            if st == "clear":
                ax.add_patch(Rectangle((x, y), w, h, fc="#66BB6A", alpha=0.07, ec="none", zorder=3))
            if st == "re-opened":
                pulse = 0.5 + 0.5 * math.sin(f.t * 4)
                ax.add_patch(Rectangle((x, y), w, h, fill=False, ec=col, lw=2 + 2 * pulse, zorder=3))
            elif st in ("searching", "victim"):
                ax.add_patch(Rectangle((x, y), w, h, fill=False, ec=col, lw=1.2, alpha=0.8, zorder=3))
            if handed and st in ("unsearched", "searching"):
                ax.add_patch(Rectangle((x, y), w, h, fill=False, ec="#FF8A65", hatch="///", lw=1.5, alpha=0.7, zorder=3))
                ax.text(x + w / 2, y + h / 2, "NOT\nSEARCHED", color="#FF8A65", fontsize=11, weight="bold",
                        ha="center", va="center", zorder=4, alpha=0.95)
            lab = {"unsearched": "", "searching": "searching", "clear": "clear", "re-opened": "RE-OPENED",
                   "victim": "victim"}[st]
            ax.text(x + 0.25, y + 0.25, f"R{rid}" + (f" · {lab}" if lab else ""), color=col, fontsize=9,
                    weight="bold", ha="left", va="top", zorder=4,
                    bbox=dict(fc=BG, ec="none", alpha=0.55, pad=0.8))
        # fire and the no-fly zone around it
        for fr in b.fires:
            x, y = fr["xy"]
            fl = 0.85 + 0.15 * math.sin(f.t * 9 + x)
            ax.add_patch(Circle((x, y), 0.9 * fl, color="#FF6D00", alpha=0.85, zorder=6))
            ax.add_patch(Circle((x, y), 0.45 * fl, color="#FFEA00", alpha=0.9, zorder=6))
            ax.add_patch(Circle((x, y), 3.0, fill=False, ec="#FF5252", lw=1.2, ls="--", alpha=0.8, zorder=6))
            ax.text(x, y + 3.35, "FIRE · no-fly 3 m", color="#FF8A65", fontsize=9, weight="bold", ha="center",
                    va="top", zorder=6)
        # aftershock zone
        if b.last_shock is not None and f.t - b.last_shock["t"] < 25:
            age = f.t - b.last_shock["t"]
            ax.add_patch(Circle(b.last_shock["xy"], b.last_shock["r"], fill=False, ec="#FF5252", lw=3 - age * 0.1,
                                alpha=max(0.2, 1 - age / 25), zorder=6))
            ax.text(b.last_shock["xy"][0], b.last_shock["xy"][1] - b.last_shock["r"] - 0.3, "AFTERSHOCK COLLAPSE",
                    color="#FF5252", fontsize=11, ha="center", weight="bold", alpha=max(0.3, 1 - age / 25), zorder=6)
        # radio links
        segs, cols = [], []
        for pa, pb, margin in f.links:
            segs.append([pa, pb])
            q = np.clip(margin / 20, 0, 1)
            cols.append((1 - q * 0.6, 0.85, 0.3 + 0.2 * q, 0.22))
        if segs:
            ax.add_collection(LineCollection(segs, colors=cols, lw=1.0, zorder=4))
        # command post
        ax.plot(*f.base, marker="s", ms=13, color="#ECEFF1", zorder=8)
        ax.text(f.base[0] + 0.6, f.base[1] + 0.2, "incident\ncommand", color="#ECEFF1", fontsize=8, va="center", zorder=8)
        # microphone bearing cones (each listener's direction estimate)
        by_name = {d.name: d for d in f.drones}
        for d in f.alive("micro"):
            for (t0, brg, sg) in d.bear_vis:
                age = f.t - t0
                if age > 6:
                    continue
                al = 1 - age / 6
                half = math.degrees(min(max(2 * sg, 0.12), 0.8))
                ax.add_patch(Wedge(tuple(self.P(d)), 9.0, math.degrees(brg) - half, math.degrees(brg) + half,
                                   color="#EA80FC", alpha=0.20 * al, lw=0, zorder=7))
                ax.add_patch(Wedge(tuple(self.P(d)), 9.0, math.degrees(brg) - half, math.degrees(brg) + half,
                                   fill=False, ec="#EA80FC", alpha=0.7 * al, lw=1, zorder=7))
                if age < 0.6:
                    comp = (math.degrees(brg) + 90) % 360
                    p = self.P(d) + 2.2 * np.array([math.cos(brg), math.sin(brg)])
                    ax.text(p[0], p[1], f"{comp:.0f}°", color="#EA80FC", fontsize=10, weight="bold", zorder=16,
                            ha="center", bbox=dict(fc=BG, ec="none", alpha=0.6, pad=0.8))
        # sound fixes (triangulation of several bearings)
        for fx in f.fixes:
            age = f.t - fx["t"]
            if age > 10:
                continue
            al = 1 - age / 10
            ev, evec = np.linalg.eigh(fx["cov"])
            ang = math.degrees(math.atan2(evec[1, 1], evec[0, 1]))
            w, h = 2 * 2 * np.sqrt(np.maximum(ev[::-1], 0.04))
            ax.add_patch(Ellipse(fx["xy"], w, h, angle=ang, fill=False, ec="#EA80FC", lw=2, alpha=al, zorder=7))
        # breathing radar: pulses where chest motion is detected
        for (t0, dn, svn, solid, xy) in self.breath:
            if f.t - t0 > 1.0:
                continue
            for k in range(3):
                ph = ((f.t * 0.8 + k / 3) % 1.0)
                ax.add_patch(Circle(tuple(xy), 0.3 + 1.1 * ph, fill=False, ec="#00E5FF", lw=2 * (1 - ph) + 0.3,
                                    alpha=0.9 * (1 - ph), zorder=9))
            if dn in by_name:
                ax.plot([self.P(by_name[dn])[0], xy[0]], [self.P(by_name[dn])[1], xy[1]], color="#00E5FF", lw=0.8,
                        ls=":", alpha=0.7, zorder=9)
        # candidates from cameras
        for c in f.candidates:
            x, y = c["xy"]
            if c["status"] == "rejected":
                ax.plot(x, y, marker="x", ms=10, mew=2, color="#9E9E9E", zorder=9)
                ax.text(x + 0.35, y - 0.35, "not a person", color="#9E9E9E", fontsize=8, zorder=9)
            elif c["status"] in ("open", "review"):
                ax.text(x, y, "?", color="#FFD54F", fontsize=15, weight="bold", ha="center", va="center", zorder=9)
        # truth (hidden from drones)
        if self.show_truth:
            for sv in b.survivors:
                if sv["found_t"] is None:
                    ax.add_patch(Circle(sv["pos"], 0.45, fill=False, ec="white", lw=1.0, ls=(0, (2, 2)), alpha=0.55, zorder=5))
        # crew route (after handoff: bold; before: faint)
        for fd in f.found:
            if fd.get("path"):
                pth = np.array(fd["path"])
                if handed:
                    ax.plot(pth[:, 0], pth[:, 1], color="#FFFFFF", lw=4.5, alpha=0.35, zorder=9, solid_capstyle="round")
                    ax.plot(pth[:, 0], pth[:, 1], color="#76FF03", lw=2.2, ls=(0, (4, 2)), zorder=10)
                else:
                    ax.plot(pth[:, 0], pth[:, 1], color="#76FF03", lw=1.0, ls=":", alpha=0.45, zorder=9)
        if handed and any(fd.get("path") for fd in f.found):
            ax.text(0.3, b.entrance[1] + 5.2, "SAFE ROUTE\ncrawl ≥ 0.6 m,\nno smoke", color="#76FF03",
                    fontsize=10, weight="bold", ha="left", va="top", zorder=11,
                    bbox=dict(fc=BG, ec="none", alpha=0.7, pad=1))
        # victim pins with confidence
        for fd in f.found:
            x, y = fd["xy"]
            col = CONF_COL.get(fd["conf"], "#00E676")
            ax.add_patch(Circle((x, y), 0.6, color=col, zorder=10))
            ax.plot(x, y, marker="P", ms=12, color="#0E1117", zorder=11)
            ev = fd["evidence"]
            ax.text(x + 0.8, y - 1.0, f"VICTIM {fd['label']} · {fd['conf']}\n{ev}", color=col, fontsize=10,
                    weight="bold", zorder=16, va="bottom", bbox=dict(fc="#0E1117", ec=col, lw=1, alpha=0.9, pad=2))
        # drones
        for d in f.drones:
            x, y = self.P(d)
            if len(d.trail) > 2 and d.alive:
                tr = np.array(list(d.trail[-60:]) + [self.P(d)])
                ax.plot(tr[:, 0], tr[:, 1], color=d.spec.color, lw=0.8, alpha=0.35, zorder=12)
            if not d.alive:
                ax.plot(x, y, marker="x", ms=11, mew=2.5, color="#E53935", zorder=13)
                ax.text(x + 0.4, y + 0.4, d.name + " lost", color="#E53935", fontsize=8, zorder=13)
                continue
            if d.kind == "scout":
                if d.ultra_mode:
                    ax.add_patch(Circle((x, y), cfg.ultra_range, fill=False, ec="#FFAB40", lw=1.2, ls="--", alpha=0.8, zorder=13))
                if d.lidar_hits is not None and len(d.lidar_hits):
                    E = d.lidar_hits
                    q = d.health.get("LiDAR") or 1.0
                    ax.add_collection(LineCollection([[(x, y), e] for e in E], colors=[(0.31, 0.76, 0.97, 0.10)], lw=0.6, zorder=6))
                    ax.scatter(E[:, 0], E[:, 1], s=3, color="#4FC3F7" if q > 0.5 else "#B0BEC5", alpha=0.8, lw=0, zorder=6)
                if d.radar_mode:
                    ax.add_patch(Circle((x, y), cfg.radar_range_scout, fill=False, ec="#26C6DA", lw=1.5, ls="-.", alpha=0.8, zorder=13))
                ax.add_patch(RegularPolygon((x, y), 4, radius=0.42, orientation=math.pi / 4,
                                            color="#26C6DA" if d.has_radar else d.spec.color, zorder=14))
            elif d.kind == "micro":
                if d.state not in ("CHARGE",):
                    q = d.health.get("RGB", 1.0) if d.health else 1.0
                    rng = cfg.cam_range * max(0.15, q)
                    hd = math.degrees(d.heading)
                    ax.add_patch(Wedge((x, y), rng, hd - cfg.cam_fov_deg / 2, hd + cfg.cam_fov_deg / 2,
                                       color=d.spec.color if q > 0.5 else "#9E9E9E", alpha=0.13, lw=0, zorder=6))
                col = "#CE93D8" if d.role == "listener" else "#00E5FF"
                ax.add_patch(Circle((x, y), 0.2, color=col, zorder=14))
                if d.state == "LISTENING (landed)":
                    ax.add_patch(Circle((x, y), 0.5, fill=False, ec="#EA80FC", lw=1.2, zorder=14))
                if d.state == "HOVER: mic + radar":
                    ax.add_patch(Circle((x, y), cfg.radar_range_micro, fill=False, ec="#00E5FF", lw=1, ls=":", alpha=0.7, zorder=13))
            else:
                if d.lidar_hits is not None and len(d.lidar_hits):
                    ax.scatter(d.lidar_hits[:, 0], d.lidar_hits[:, 1], s=2, color="#FFD54F", alpha=0.5, lw=0, zorder=6)
                ax.add_patch(RegularPolygon((x, y), 6, radius=0.75, color=d.spec.color, zorder=14))
                ax.add_patch(Circle((x, y), 1.2, fill=False, ec=d.spec.color, lw=1, alpha=0.6, zorder=14))
            ax.text(x, y - 0.55 if d.kind != "mother" else y - 1.0, d.name, color="white", fontsize=8 if d.kind == "micro" else 9,
                    ha="center", va="bottom", weight="bold", zorder=15)
        ax.set_xlim(0, W); ax.set_ylim(H, 0)
        ax.set_xticks([]); ax.set_yticks([])
        for sp in ax.spines.values():
            sp.set_color("#263238")
        ax.plot([W - 6.5, W - 1.5], [H - 0.8, H - 0.8], color="white", lw=2)
        ax.text(W - 4, H - 1.1, "5 m", color="white", fontsize=9, ha="center")
        # compass (bearings are compass degrees: 0° = north = up)
        ax.annotate("", xy=(W - 1.2, 1.0), xytext=(W - 1.2, 2.6), arrowprops=dict(arrowstyle="-|>", color="white", lw=1.5))
        ax.text(W - 1.2, 0.75, "N", color="white", fontsize=10, ha="center", weight="bold")
        if b.shaking(f.t):
            ax.text(W / 2, 1.2, "⚠ AFTERSHOCK: ALL DRONES HOLDING", color="#FF5252", fontsize=18, weight="bold", ha="center", zorder=20)
        elif f.quiet():
            ax.text(W / 2, 1.2, "QUIET PERIOD: rotors idle, listening for tapping", color="#EA80FC", fontsize=15,
                    weight="bold", ha="center", zorder=20)

    # ------------------------------------------------------------------ right panel
    def draw_arch(self, fig, x0, y):
        """Dashboard → Coordinator → drones → sensors, coloured by live link state."""
        f = self.s.fleet

        def box(x, yy, w, txt, col, sub=None):
            fig.patches.append(FancyBboxPatch((x, yy), w, 0.03, boxstyle="round,pad=0.002", fc="#1E2733", ec=col, lw=1.2,
                                              transform=fig.transFigure, figure=fig))
            fig.text(x + w / 2, yy + 0.019, txt, color=col, fontsize=9.5, weight="bold", ha="center", va="center")
            if sub:
                fig.text(x + w / 2, yy + 0.007, sub, color="#90A4AE", fontsize=7.2, ha="center", va="center")

        def arrow(xa, xb, yy):
            fig.text((xa + xb) / 2, yy + 0.014, "→", color="#78909C", fontsize=13, ha="center", va="center")
        fig.text(x0, y + 0.038, "SYSTEM", color="#B0BEC5", fontsize=10, weight="bold")
        box(x0, y, 0.054, "Dashboard", "#ECEFF1", "incident command")
        arrow(x0 + 0.054, x0 + 0.065, y)
        m = f.drones[0]
        box(x0 + 0.065, y, 0.064, "Coordinator M", "#FFD54F" if m.alive else "#E53935", "merge · auction · relay")
        arrow(x0 + 0.129, x0 + 0.140, y)
        groups = [("S1 S2", "LiDAR: SLAM", "#4FC3F7", ["S1", "S2"]),
                  ("S3", "radar: breathing", "#26C6DA", ["S3"]),
                  ("L1–L4", "acoustic: direction", "#CE93D8", ["L1", "L2", "L3", "L4"]),
                  ("C1 C2", "RGB + mmWave", "#00E5FF", ["C1", "C2"])]
        by = {d.name: d for d in f.drones}
        for i, (nm, sub, col, names) in enumerate(groups):
            xx = x0 + 0.140 + i * 0.0545
            n_up = sum(1 for n in names if n in by and by[n].alive and by[n].link)
            box(xx, y, 0.0515, f"{nm}  {n_up}/{len(names)}", col if n_up else "#E53935", sub)

    def draw_panel(self, fig, x0):
        s, f, b, cfg = self.s, self.s.fleet, self.s.b, self.s.cfg
        self.draw_arch(fig, x0, 0.872)
        n_found = len(f.found)
        inside = f.cmap.clear[b.y0:b.y1, b.x0:b.x1]
        mapped = float((~np.isnan(inside)).mean())
        searched = sum(1 for st in f.room_status.values() if st in ("clear", "victim"))
        linked = sum(1 for d in f.alive() if d.link)
        y = 0.818
        stats = [(f"{n_found}/{f.n_unaccounted}", "unaccounted located", "#00E676"),
                 (f"{mapped * 100:.0f}%", "building mapped", "#4FC3F7"),
                 (f"{searched}/{len(f.room_status)}", "rooms searched", "#FFCA28"),
                 (f"{linked}/{len(f.alive())}", "drones in radio contact", "#B39DDB")]
        for i, (v, lab, col) in enumerate(stats):
            xx = x0 + i * 0.092
            fig.text(xx, y, v, color=col, fontsize=22, weight="bold")
            fig.text(xx, y - 0.02, lab, color="#90A4AE", fontsize=9)
        # fleet table with live sensor health
        y = 0.77
        fig.text(x0, y, "FLEET", color="#B0BEC5", fontsize=10, weight="bold")
        hx = x0 + 0.272
        for j, lab in enumerate(["LiDAR", "RGB", "radar", "mic"]):
            fig.text(hx + j * 0.021, y, lab, color="#78909C", fontsize=7.5, ha="center")
        y -= 0.022
        for d in f.drones:
            col = d.spec.color if d.kind != "micro" else ("#CE93D8" if d.role == "listener" else "#00E5FF")
            if d.has_radar and d.kind == "scout":
                col = "#26C6DA"
            fig.text(x0, y, d.name, color=col, fontsize=10.5, weight="bold", family="monospace")
            fig.text(x0 + 0.022, y, ROLE.get(d.role, d.kind), color="#78909C", fontsize=8.5)
            st = d.state
            if d.kind == "scout" and d.radar_mode and d.alive:
                st = st + " · radar"
            fig.text(x0 + 0.128, y, st[:22], color=STATE_COL.get(d.state, "#CFD8DC"), fontsize=9, weight="bold")
            bx = x0 + 0.218
            fig.patches.append(FancyBboxPatch((bx, y + 0.002), 0.03, 0.01, boxstyle="square,pad=0", fc="#263238",
                                              ec="none", transform=fig.transFigure, figure=fig))
            bc = "#66BB6A" if d.battery > 0.4 else "#FFCA28" if d.battery > 0.2 else "#EF5350"
            fig.patches.append(FancyBboxPatch((bx, y + 0.002), 0.03 * max(d.battery, 0), 0.01, boxstyle="square,pad=0", fc=bc,
                                              ec="none", transform=fig.transFigure, figure=fig))
            fig.text(bx + 0.032, y, f"{d.battery * 100:3.0f}%", color="#90A4AE", fontsize=8, family="monospace")
            for j, k in enumerate(["LiDAR", "RGB", "radar", "mic"]):
                v = d.health.get(k) if (d.health and d.alive) else None
                hc = health_col(v)
                if hc is None:
                    fig.text(hx + j * 0.021, y, "·", color="#37474F", fontsize=9, ha="center")
                else:
                    fig.patches.append(Ellipse((hx + j * 0.021, y + 0.006), 0.0075, 0.0133, fc=hc, ec="none",
                                               transform=fig.transFigure, figure=fig))
            fig.text(x0 + 0.36, y, "((·))" if d.link and d.alive else "  ×  ", color="#66BB6A" if d.link and d.alive else "#EF5350",
                     fontsize=8, family="monospace")
            y -= 0.0215
        self.draw_sensors(fig, x0, 0.335)
        if f.briefing:
            self.draw_briefing(fig, x0, 0.30)
        else:
            self.draw_log(fig, x0, 0.30)

    def pick_focus(self):
        f = self.s.fleet
        if getattr(self, "focus_name", None):
            for d in f.drones:
                if d.name == self.focus_name:
                    return d
        smoky = [d for d in f.alive("scout") if d.health and (d.health.get("RGB") or 1) < 0.6]
        hov = [d for d in f.alive("micro") if d.state == "HOVER: mic + radar"]
        recent = [x for x in self.breath if f.t - x[0] < 3]
        by = {d.name: d for d in f.drones}
        if smoky:
            return smoky[0]
        if recent and recent[-1][1] in by:
            return by[recent[-1][1]]
        if hov:
            return hov[0]
        heard = [d for d in f.alive("micro") if d.bear_vis and f.t - d.bear_vis[-1][0] < 4]
        if heard:
            return heard[0]
        if self.focus is not None and self.focus.alive and self.focus.kind != "mother":
            return self.focus
        return by.get("S1", f.drones[1])

    def draw_sensors(self, fig, x0, y0):
        """What one drone senses right now: RGB view, LiDAR / radar / mic, breathing signal."""
        f, b, cfg = self.s.fleet, self.s.b, self.s.cfg
        d = self.focus = self.pick_focus()
        fig.text(x0, y0 + 0.17, f"LIVE SENSORS · {d.name} ({ROLE.get(d.role, d.kind)})", color="#B0BEC5",
                 fontsize=10, weight="bold")
        w, h = 0.112, 0.145
        # RGB: top-down crop of the true scene with the drone's real visibility applied
        ax = fig.add_axes([x0, y0, w, h], zorder=5); ax.set_xticks([]); ax.set_yticks([])
        R = 2.4 if d.kind == "micro" else 4.0
        r0, c0 = b.cell_of(self.P(d)); n = int(R / cfg.cell)
        rs, re_ = max(0, r0 - n), min(cfg.H, r0 + n); cs, ce = max(0, c0 - n), min(cfg.W, c0 + n)
        crop = np.clip(b.clear[rs:re_, cs:ce], 0, 3.0) / 3.0
        img = np.stack([0.25 + 0.45 * crop, 0.22 + 0.42 * crop, 0.18 + 0.38 * crop], -1)
        img[b.clear[rs:re_, cs:ce] < 0.12] = (0.55, 0.52, 0.48)
        ex = (cs * cfg.cell, ce * cfg.cell, re_ * cfg.cell, rs * cfg.cell)
        q = (d.health or {}).get("RGB", 1.0) if d.alive else 0.0
        q = 1.0 if q is None else q
        real = getattr(self, "rgb_img", None)
        if real is not None:                      # rendered by Isaac Sim from this drone's camera
            ax.imshow(real, extent=ex, interpolation="bilinear", aspect="auto")
            ax.set_xlim(ex[0], ex[1]); ax.set_ylim(ex[2], ex[3])
            fog = np.zeros((4, 4, 4)); fog[..., :3] = 0.62; fog[..., 3] = np.clip(1 - q, 0, 0.95)
            ax.imshow(fog, extent=ex, interpolation="bicubic", zorder=4, aspect="auto")
            ax.set_title(f"RGB camera (Isaac render) · visibility {q * 100:.0f}%", color="#FFCA28" if q < 0.6 else "#B0BEC5",
                         fontsize=8.5, pad=3)
            for sp in ax.spines.values():
                sp.set_color("#37474F")
            real = True
        if real is not True:
            ax.imshow(img, extent=ex, interpolation="bilinear")
            for sv in b.survivors:
                if ex[0] <= sv["pos"][0] <= ex[1] and ex[3] <= sv["pos"][1] <= ex[2]:
                    rr, cc = b.line_cells(self.P(d), sv["pos"])
                    if not (b.clear[rr[1:-1], cc[1:-1]] < 0.12).any():
                        ax.add_patch(Ellipse(sv["pos"], 0.5, 0.9, angle=30, color="#D7A27C", zorder=3))
            for dc in b.decoys:
                if ex[0] <= dc["pos"][0] <= ex[1] and ex[3] <= dc["pos"][1] <= ex[2]:
                    ax.add_patch(Ellipse(dc["pos"], 0.45, 0.8, angle=-20, color="#9E9E9E", zorder=3))
            fog = np.ones((8, 8, 4)) * np.array([0.62, 0.63, 0.65, 0.0])
            fog[..., 3] = np.clip(1 - q, 0, 0.97)
            rng = np.random.default_rng(int(f.t * 10))
            fog[..., :3] += rng.normal(0, 0.03, (8, 8, 1))
            ax.imshow(np.clip(fog, 0, 1), extent=ex, interpolation="bicubic", zorder=4)
            ax.set_xlim(ex[0], ex[1]); ax.set_ylim(ex[2], ex[3])
            ax.set_title(f"RGB camera · visibility {q * 100:.0f}%", color="#FFCA28" if q < 0.6 else "#B0BEC5", fontsize=8.5, pad=3)
            for sp in ax.spines.values():
                sp.set_color("#37474F")
        # LiDAR scan (scouts), radar map (S3 in smoke) or microphone bearings (micros)
        ax2 = fig.add_axes([x0 + w + 0.012, y0, w, h], facecolor="#0B0F15", zorder=5); ax2.set_xticks([]); ax2.set_yticks([])
        for sp in ax2.spines.values():
            sp.set_color("#37474F")
        if d.kind == "scout" and d.radar_mode:
            rr, cc, cv = b.radar_map(d.pos, cfg.radar_range_scout)
            pts = np.stack([(cc + 0.5) * cfg.cell, (rr + 0.5) * cfg.cell], 1) - d.pos
            solid = cv < 0.12
            ax2.scatter(pts[~solid, 0], pts[~solid, 1], s=16, marker="s", color="#0E4A55", lw=0)
            ax2.scatter(pts[solid, 0], pts[solid, 1], s=16, marker="s", color="#26C6DA", lw=0)
            ax2.plot(0, 0, marker="s", color="#26C6DA", ms=5)
            ax2.set_xlim(-5, 5); ax2.set_ylim(5, -5)
            ax2.set_title("UWB radar map · coarse, sees through smoke", color="#26C6DA", fontsize=8.5, pad=3)
        elif d.kind == "scout" and d.lidar_hits is not None and len(d.lidar_hits):
            E = d.lidar_hits - d.pos
            ql = (d.health or {}).get("LiDAR", 1.0) or 1.0
            ax2.scatter(E[:, 0], E[:, 1], s=4, color="#4FC3F7" if ql > 0.5 else "#B0BEC5", lw=0)
            ax2.plot(0, 0, marker="s", color="#4FC3F7", ms=5)
            ax2.set_xlim(-10, 10); ax2.set_ylim(10, -10)
            ax2.set_title(f"LiDAR scan · {len(E)} returns · quality {ql * 100:.0f}%",
                          color="#FFCA28" if ql < 0.6 else "#B0BEC5", fontsize=8.5, pad=3)
        else:
            th = np.linspace(0, 2 * math.pi, 100)
            ax2.plot(np.cos(th), np.sin(th), color="#37474F", lw=1)
            for (t0, brg, sg) in d.bear_vis:
                age = f.t - t0
                if age < 12:
                    al = 1 - age / 12
                    ax2.add_patch(Wedge((0, 0), 1.0, math.degrees(brg - 2 * sg), math.degrees(brg + 2 * sg),
                                        color="#EA80FC", alpha=0.5 * al))
            ax2.text(0, -1.1, "N", color="#78909C", fontsize=8, ha="center")
            ax2.set_xlim(-1.25, 1.25); ax2.set_ylim(1.25, -1.25); ax2.set_aspect("equal")
            ttl = f"mic array · knocks ({'landed: quiet' if d.state == 'LISTENING (landed)' else 'flying: rotor noise'})"
            ax2.set_title(ttl, color="#B0BEC5", fontsize=8.5, pad=3)
        # breathing signal from the radar (chest motion, 0.2-0.5 Hz)
        ax3 = fig.add_axes([x0 + 2 * (w + 0.012), y0, w, h], facecolor="#0B0F15", zorder=5); ax3.set_yticks([])
        ax3.tick_params(colors="#607D8B", labelsize=7)
        for sp in ax3.spines.values():
            sp.set_color("#37474F")
        mine = [x for x in self.breath if x[1] == d.name] or self.breath
        tt = np.linspace(f.t - 10, f.t, 200)
        if mine:
            t_last, dn, svn, solid, _ = mine[-1]
            hz = BREATH_HZ.get(svn, 0.25)
            have = np.array([any(abs(t - x[0]) < 0.6 for x in mine) for t in tt])
            sig = np.sin(2 * math.pi * hz * tt) * (0.8 - 0.15 * min(solid, 3))
            noise = np.random.default_rng(int(f.t * 10)).normal(0, 0.12 + 0.12 * solid, len(tt))
            y_ = np.where(have, sig + noise, noise * 0.6)
            ax3.plot(tt - f.t, y_, color="#00E5FF", lw=1.2)
            ax3.set_title(f"breathing radar · {dn} · {hz * 60:.0f}/min" + (" · thru rubble" if solid else ""),
                          color="#00E5FF", fontsize=8.5, pad=3)
        else:
            noise = np.random.default_rng(int(f.t * 10)).normal(0, 0.1, len(tt))
            ax3.plot(tt - f.t, noise, color="#37474F", lw=1)
            ax3.set_title("breathing radar · no chest motion", color="#607D8B", fontsize=8.5, pad=3)
        ax3.set_ylim(-1.4, 1.4); ax3.set_xlim(-10, 0)
        ax3.set_xticks([-10, -5, 0]); ax3.set_xticklabels(["-10 s", "-5 s", "now"])

    def draw_log(self, fig, x0, y):
        f = self.s.fleet
        fig.text(x0, y, "DECISION LOG  (decided on board; humans asked only when unsure)", color="#B0BEC5",
                 fontsize=10, weight="bold")
        y -= 0.024
        lines = []
        for t, m in f.log[-40:]:
            if "microphone:" in m:
                continue
            for j, w in enumerate(textwrap.wrap(m, 92)):
                lines.append((t if j == 0 else None, w, m))
        for t, w, m in lines[-10:]:
            col = "#00E676" if ("Victim" in m or "CONFIRMED" in m) else \
                "#FF5252" if ("AFTERSHOCK" in m or "lost" in m or "TRAPPED" in m or "moke" in m) else \
                "#EA80FC" if ("knock" in m.lower() or "Quiet" in m or "call-out" in m) else \
                "#26C6DA" if "radar" in m else \
                "#FF8A65" if ("UNSURE" in m or "COMMAND" in m) else "#CFD8DC"
            if t is not None:
                fig.text(x0, y, f"{t:6.1f}s", color="#607D8B", fontsize=8.8, family="monospace")
            fig.text(x0 + 0.038, y, w, color=col, fontsize=8.8)
            y -= 0.0198

    def draw_briefing(self, fig, x0, y):
        f = self.s.fleet
        fig.patches.append(FancyBboxPatch((x0 - 0.004, 0.08), 0.36, y - 0.065, boxstyle="round,pad=0.003", fc="#0F2A1A",
                                          ec="#76FF03", lw=1.2, transform=fig.transFigure, figure=fig))
        fig.text(x0, y - 0.006, f.briefing[0], color="#76FF03", fontsize=11, weight="bold")
        y -= 0.034
        for ln in f.briefing[1:]:
            for j, w in enumerate(textwrap.wrap(ln, 84)):
                col = "#00E676" if ln.startswith("Victim") and "HIGH" in ln else "#FFC400" if ln.startswith("Victim") else \
                    "#FF8A65" if ("Not searched" in ln or "unaccounted" in ln or "Hazards" in ln) else "#E0E0E0"
                fig.text(x0 + (0.008 if j else 0), y, w, color=col, fontsize=9.5)
                y -= 0.021
            y -= 0.006

    def legend(self, fig, x, y):
        items = [("#A0A8B0", "wall / rubble"), ("#334050", "open (scouts fit)"),
                 ("#E0A030", "crevice void 0.25–0.9 m (micro only)"), ("#0B0E14", "not yet mapped"),
                 ("#8C9199", "smoke"), ("#26C7D9", "radar-only map (coarse)"),
                 ("#D94DFF", "mic bearing / sound fix"), ("#00E5FF", "breathing radar pulse")]
        for i, (c, lab) in enumerate(items):
            xx = x + (i % 4) * 0.15
            yy = y - (i // 4) * 0.022
            fig.patches.append(FancyBboxPatch((xx, yy), 0.012, 0.014, boxstyle="square,pad=0", fc=c, ec="#455A64",
                                              transform=fig.transFigure, figure=fig))
            fig.text(xx + 0.016, yy + 0.002, lab, color="#B0BEC5", fontsize=9)

    def draw_caption(self, fig):
        caps = self.latest_captions()
        n, lab = self.scene()
        fig.patches.append(FancyBboxPatch((0.012, 0.852), 0.592, 0.062, boxstyle="round,pad=0.003", fc="#0A0D12",
                                          ec="#37474F", lw=1, transform=fig.transFigure, figure=fig))
        fig.text(0.02, 0.898, f"SCENE {n}/7 · {lab.upper()}", color="#78909C", fontsize=9.5, weight="bold")
        if caps:
            txt = "   ·   ".join(m for _, m in caps)
            age = self.s.fleet.t - caps[-1][0]
            col = "#FFFFFF" if age < 3 else "#CFD8DC"
            fs = 17 if len(txt) < 80 else 14
            fig.text(0.02, 0.864, txt, color=col, fontsize=fs, weight="bold")

    def frame(self):
        s, f, cfg = self.s, self.s.fleet, self.s.cfg
        fig = self.fig
        fig.clf()
        self.note_radar()
        fig.text(0.012, 0.962, "SkySearch: a drone team searching an earthquake-damaged building",
                 color="white", fontsize=19, weight="bold")
        sub = (f"1 coordinator · 2 LiDAR mappers + 1 radar scout · 4 listeners + 2 confirmers (crevice drones)   ·   "
               f"t = {f.t:5.1f} s{('   ·   ' + self.speed_label) if self.speed_label else ''}   ·   simulation, unscripted drone decisions")
        fig.text(0.012, 0.935, sub, color="#B0BEC5", fontsize=11.5)
        self.draw_caption(fig)
        ax = fig.add_axes([0.008, 0.075, 0.6, 0.77])
        ax.set_facecolor(BG)
        self.draw_map(ax)
        ax.apply_aspect()
        bb = ax.get_position()
        self.map_box = (bb.x0, bb.y0, bb.x1, bb.y1, cfg.W * cfg.cell, cfg.H * cfg.cell)
        self.legend(fig, 0.012, 0.05)
        if self.show_truth:
            fig.text(0.012, 0.012, "dashed white rings = where people really are (hidden from the drones)",
                     color="#78909C", fontsize=9)
        fig.patches.append(FancyBboxPatch((0.618, 0.07), 0.375, 0.86, boxstyle="round,pad=0.004", fc=PANEL, ec="#263238",
                                          transform=fig.transFigure, figure=fig))
        self.draw_panel(fig, 0.626)
        return fig

    def save(self, path, dpi=100):
        self.frame().savefig(path, dpi=dpi, facecolor=BG)
