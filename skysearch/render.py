"""Mission-control renderer: map with live search belief, drone regions, sensor footprints,
perception overlays, and a side panel with each drone's state and the decision log."""
from __future__ import annotations
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from matplotlib.colors import ListedColormap

COLORS = ["#4FC3F7", "#FFB74D", "#BA68C8", "#81C784", "#F06292", "#FFF176", "#4DB6AC", "#A1887F"]
BG = "#0E1117"
STATE_COL = {"SEARCH": "#81C784", "INSPECT": "#FFD54F", "CONFIRMED": "#4CAF50", "ASK HUMAN": "#FF8A65",
             "RETURN": "#90A4AE", "SWAP": "#90A4AE", "FAILED": "#E53935"}


class Renderer:
    def __init__(self, scen, show_truth=True, title="SkySearch: autonomous search-and-rescue swarm"):
        self.s = scen
        self.show_truth = show_truth
        self.fig = plt.figure(figsize=(19.2, 10.8), dpi=100, facecolor=BG)
        self.title = title

    def frame(self):
        s, sw, w, cfg = self.s, self.s.swarm, self.s.world, self.s.cfg
        fig = self.fig
        fig.clf()
        fig.text(0.015, 0.965, self.title, color="white", fontsize=20, weight="bold")
        fig.text(0.015, 0.94, f"Simulated forest 1 km²  ·  {len(sw.alive())}/{cfg.n_drones} drones flying  ·  "
                 f"all decisions on-board  ·  t = {sw.t:5.1f} s", color="#B0BEC5", fontsize=12)
        ax = fig.add_axes([0.01, 0.03, 0.52, 0.89])
        ax.set_facecolor(BG)
        A = cfg.area
        ext = (0, A, 0, A)
        ax.imshow(w.canopy, origin="lower", extent=ext, cmap="Greens", vmin=-0.3, vmax=1.1)
        ref = next((d for d in sw.alive() if d.link), sw.drones[0])
        post = ref.belief.posterior()
        lp = np.log(post + 1e-12)
        lo, hi = np.percentile(lp, 15), np.percentile(lp, 99.5)
        v = np.clip((lp - lo) / max(hi - lo, 1e-6), 0, 1)
        rgba = plt.get_cmap("inferno")(v)
        rgba[..., 3] = 0.75 * v ** 0.8
        ax.imshow(rgba, origin="lower", extent=ext)
        # region boundaries
        lab = sw.part.label
        for did in np.unique(lab):
            ax.contour(sw.part.X, sw.part.Y, (lab == did).astype(float), levels=[0.5],
                       colors=[COLORS[did % len(COLORS)]], linewidths=1.6, alpha=0.8)
        # known + true fire
        if w.fire.any():
            ax.imshow(np.ma.masked_where(~w.fire, w.fire), origin="lower", extent=ext, cmap=ListedColormap(["#FF3D00"]), alpha=0.85)
        if sw.hazard.any():
            hz = sw.hazard & ~w.fire
            ax.imshow(np.ma.masked_where(~hz, hz), origin="lower", extent=ext, cmap=ListedColormap(["#FF8A65"]), alpha=0.25)
        ax.add_patch(Rectangle((w.base[0] - 15, w.base[1] - 15), 30, 30, fc="none", ec="white", lw=1.5))
        ax.text(w.base[0] + 20, w.base[1] - 5, "BASE", color="white", fontsize=9)
        ax.plot(*w.lkp, marker="x", color="white", ms=10, mew=2)
        ax.text(w.lkp[0] + 12, w.lkp[1] + 8, "last known point", color="white", fontsize=9)
        # deer + truth
        for d_ in w.deer:
            ax.plot(*d_["pos"], "o", color="#A1887F", ms=4, alpha=0.8)
        if self.show_truth:
            for p in w.people:
                found = p["found_t"] is not None
                ax.plot(*p["pos"], marker="*", ms=16, mfc="#00E676" if found else "none", mec="#00E676" if found else "cyan", mew=1.5)
            ax.text(10, 10, "☆ true position of missing people (hidden from swarm)   ● deer (thermal decoys)",
                    color="#B0BEC5", fontsize=9)
        # candidates
        for c in sw.candidates:
            st = c["status"]
            sty = {"open": ("o", "#FFD54F"), "confirmed": ("P", "#00E676"), "rejected": ("x", "#9E9E9E"),
                   "review": ("D", "#FF8A65"), "handed off": ("o", "#FFD54F")}[st]
            ax.plot(*c["xy"], marker=sty[0], color=sty[1], ms=9, mfc="none" if st == "open" else sty[1], mew=2)
        # drones
        for d in sw.drones:
            col = COLORS[d.id % len(COLORS)]
            tr = np.array(d.trail)
            if len(tr) > 1:
                ax.plot(tr[:, 0], tr[:, 1], color=col, lw=1.2, alpha=0.6)
            if d.state == "FAILED":
                ax.plot(*d.pos[:2], marker="X", color="#E53935", ms=14)
                ax.text(d.pos[0] + 12, d.pos[1] + 12, f"D{d.id} DOWN", color="#E53935", fontsize=10, weight="bold")
                continue
            if d.pos[2] > 5:
                ax.add_patch(Circle(d.pos[:2], w.footprint_radius(d.pos[2]), fc=col, ec=col, alpha=0.18, lw=1))
            if d.state in ("SEARCH",) and d.target is not None:
                ax.plot([d.pos[0], d.target[0]], [d.pos[1], d.target[1]], ":", color=col, lw=1, alpha=0.7)
            if d.path:
                pth = np.array([d.pos[:2]] + d.path)
                ax.plot(pth[:, 0], pth[:, 1], "--", color="#FF8A65", lw=1.2)
            if d.state in ("INSPECT", "CONFIRMED", "ASK HUMAN") and d.cand is not None:
                ax.plot([d.pos[0], d.cand["xy"][0]], [d.pos[1], d.cand["xy"][1]], "-", color="#FFD54F", lw=1.5)
                ax.text(d.cand["xy"][0] + 10, d.cand["xy"][1] - 18, f"p(person)={d.cand['p']:.2f}", color="#FFD54F", fontsize=9)
            ax.plot(*d.pos[:2], marker="^", color=col, ms=12, mec="black")
            lbl = f"D{d.id} {d.state}" + ("" if d.link else " (no link)")
            ax.text(d.pos[0] + 12, d.pos[1] + 10, lbl, color=col, fontsize=9, weight="bold")
        ax.set_xlim(0, A); ax.set_ylim(0, A)
        ax.set_xticks([]); ax.set_yticks([])
        for sp in ax.spines.values():
            sp.set_color("#37474F")

        # ---------------- side panel ----------------
        x0 = 0.55
        fig.text(x0, 0.895, "FLEET", color="white", fontsize=13, weight="bold")
        y = 0.865
        for d in sw.drones:
            col = COLORS[d.id % len(COLORS)]
            fig.text(x0, y, f"D{d.id}", color=col, fontsize=12, weight="bold")
            fig.text(x0 + 0.025, y, d.state, color=STATE_COL.get(d.state, "white"), fontsize=11, weight="bold")
            fig.text(x0 + 0.095, y, f"alt {d.pos[2]:4.0f} m", color="#CFD8DC", fontsize=10, family="monospace")
            bax = fig.add_axes([x0 + 0.155, y - 0.002, 0.07, 0.014])
            bax.barh(0, d.battery, color="#66BB6A" if d.battery > 0.3 else "#EF5350"); bax.set_xlim(0, 1)
            bax.axis("off")
            bax.add_patch(Rectangle((0, -0.4), 1, 0.8, fc="none", ec="#607D8B", lw=0.8))
            fig.text(x0 + 0.23, y, f"{d.battery * 100:3.0f}%", color="#CFD8DC", fontsize=10, family="monospace")
            fig.text(x0 + 0.26, y, "link" if d.link else "offline", color="#81C784" if d.link else "#FF8A65", fontsize=10)
            fig.text(x0 + 0.295, y, f"{d.compute_ms:4.2f} ms", color="#90A4AE", fontsize=10, family="monospace")
            fig.text(x0 + 0.34, y, d.note, color="#FFD54F", fontsize=9)
            y -= 0.03
        # thermal view of the most interesting drone
        focus = next((d for d in sw.alive() if d.state in ("INSPECT", "ASK HUMAN", "CONFIRMED")), None) or (sw.alive()[0] if sw.alive() else sw.drones[0])
        tax = fig.add_axes([x0, 0.43, 0.17, 0.25])
        tax.imshow(self.thermal_image(focus), cmap="inferno", vmin=0, vmax=1, origin="lower")
        tax.set_xticks([]); tax.set_yticks([])
        for sp in tax.spines.values():
            sp.set_color(COLORS[focus.id % len(COLORS)]); sp.set_linewidth(2)
        tax.set_title(f"D{focus.id} thermal camera (on-board detector)", color="white", fontsize=10)
        # metrics
        mx = x0 + 0.2
        found = sum(p["found_t"] is not None for p in w.people)
        rows = [("People found", f"{found} / {len(w.people)}"),
                ("Prob. of success so far", f"{ref.belief.pos() * 100:4.1f}%"),
                ("Thermal decoys ruled out on-board", f"{sw.false_alarms_filtered}"),
                ("Questions sent to operator", f"{sw.reviews}"),
                ("Fire cells known to swarm", f"{int(ref.known_fire.sum())}")]
        fig.text(mx, 0.66, "MISSION", color="white", fontsize=13, weight="bold")
        for i, (k, v) in enumerate(rows):
            fig.text(mx, 0.625 - i * 0.035, k, color="#B0BEC5", fontsize=11)
            fig.text(mx + 0.2, 0.625 - i * 0.035, v, color="white", fontsize=12, weight="bold", ha="right")
        # log
        fig.text(x0, 0.39, "DECISION LOG (why each drone did what it did)", color="white", fontsize=13, weight="bold")
        for i, (t, m) in enumerate(sw.log[-11:][::-1]):
            fig.text(x0, 0.36 - i * 0.03, f"{t:6.1f}s  {m}"[:118], color="white" if i == 0 else "#90A4AE",
                     fontsize=10.5 if i == 0 else 10, family="monospace")
        return fig

    def thermal_image(self, d, n=96):
        w = self.s.world
        img = np.random.default_rng(int(self.s.swarm.t * 10)).normal(0.12, 0.03, (n, n))
        if d.pos[2] < 2:
            return img
        r = w.footprint_radius(d.pos[2])
        xs = np.linspace(-r, r, n)
        X, Y = np.meshgrid(xs + d.pos[0], xs + d.pos[1])
        rc_x = np.clip((X // w.cfg.cell).astype(int), 0, w.cfg.n - 1)
        rc_y = np.clip((Y // w.cfg.cell).astype(int), 0, w.cfg.n - 1)
        img += 0.12 * (1 - w.canopy[rc_y, rc_x])
        sig = max(1.2, 40 / max(d.pos[2], 8))
        for src, amp in [(p, 1.0) for p in w.people] + [(q, 0.9) for q in w.deer]:
            can = w.canopy[w.cell_of(src["pos"])]
            dd = (X - src["pos"][0]) ** 2 + (Y - src["pos"][1]) ** 2
            img += amp * (1 - 0.8 * can) * np.exp(-dd / (2 * sig ** 2))
        if w.fire.any():
            img += 0.9 * w.fire[rc_y, rc_x]
        return np.clip(img, 0, 1)
