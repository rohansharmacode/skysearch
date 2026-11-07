"""docs/slam.png: SLAM heightmap vs truth, height error, trajectory error over time."""
from __future__ import annotations

import numpy as np

INK, INK2, MUTED, SURF = "#0b0b0b", "#52514e", "#898781", "#fcfcfb"
SLAM_C, ODOM_C = "#eb6834", "#1baf7a"         # categorical slots 2, 3 (slot 1 blue = height ramp)
SEQ = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]
DIV = ["#1c5cab", "#6da7ec", "#f0efec", "#ec8a89", "#c23c3b"]


def plot_slam(r, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import patheffects as pe
    from matplotlib.colors import LinearSegmentedColormap

    seq = LinearSegmentedColormap.from_list("seq", SEQ)
    div = LinearSegmentedColormap.from_list("div", DIV)
    g, tg, m = r["grid"], r["truth_grid"], r["metrics"]
    ext = [g.bounds[0], g.bounds[2], g.bounds[1], g.bounds[3]]
    vmin, vmax = float(np.percentile(tg.z, 1)), float(np.percentile(tg.z, 99.5))
    halo = [pe.withStroke(linewidth=3.5, foreground=SURF)]
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK2,
                         "xtick.color": MUTED, "ytick.color": MUTED, "axes.titlecolor": INK,
                         "figure.facecolor": SURF, "axes.facecolor": SURF})
    fig, ax = plt.subplots(2, 2, figsize=(11, 10), constrained_layout=True)

    def traj(a, P, **kw):
        a.plot(P[:, 0, 3], P[:, 1, 3], path_effects=halo, **kw)

    def sites(a, land, key, label):
        c = [s for s in land[key] if s["status"] == "candidate"]
        xy = np.array([s["xy"] for s in c]).reshape(-1, 2)
        a.scatter(xy[:, 0], xy[:, 1], s=46, facecolor="white", edgecolor=INK, linewidth=1.3, zorder=5, label=label)

    a = ax[0, 0]
    im = a.imshow(g.z, origin="lower", extent=ext, cmap=seq, vmin=vmin, vmax=vmax)
    traj(a, r["gt_poses"], color=INK, lw=1.2, ls="--", label="ground-truth path")
    traj(a, r["odom_poses"], color=ODOM_C, lw=2, label=f"odometry only (ATE {m['odom_ate_rmse_m']:.2f} m)")
    traj(a, r["est_poses"], color=SLAM_C, lw=2, label=f"KISS-ICP SLAM (ATE {m['ate_rmse_m']:.2f} m)")
    sites(a, r["landing"], "slam_sites", "landing candidate")
    a.set_title(f"SLAM heightmap, pile {m['pile']} ({m['n_scans']} sweeps, {m['map_points']:,} pts)", loc="left")
    a.legend(loc="lower left", fontsize=8, framealpha=0.9)
    fig.colorbar(im, ax=a, shrink=0.8, label="height (m)")

    a = ax[0, 1]
    im = a.imshow(tg.z, origin="lower", extent=ext, cmap=seq, vmin=vmin, vmax=vmax)
    sites(a, r["landing"], "truth_sites", "landing candidate")
    a.set_title("True geometry heightmap (terrain + rubble pieces)", loc="left")
    a.legend(loc="lower left", fontsize=8, framealpha=0.9)
    fig.colorbar(im, ax=a, shrink=0.8, label="height (m)")

    a = ax[1, 0]
    he = m["height_error"]
    im = a.imshow(g.z - tg.z, origin="lower", extent=ext, cmap=div, vmin=-0.5, vmax=0.5)
    c = r["centre"]
    a.add_patch(plt.Circle(c, 14.0, fill=False, ec=INK2, lw=1, ls=":"))
    a.set_title(f"SLAM minus truth: MAE {he['mae_m'] * 100:.1f} cm, {he['frac_within_10cm'] * 100:.0f}% of cells within 10 cm",
                loc="left")
    fig.colorbar(im, ax=a, shrink=0.8, label="height error (m)")

    a = ax[1, 1]
    t = np.arange(len(r["gt_poses"])) / 10.0
    eo = np.linalg.norm(r["odom_poses"][:, :3, 3] - r["gt_poses"][:, :3, 3], axis=1)
    es = np.linalg.norm(r["est_poses"][:, :3, 3] - r["gt_poses"][:, :3, 3], axis=1)
    a.plot(t, eo, color=ODOM_C, lw=2)
    a.plot(t, es, color=SLAM_C, lw=2)
    a.text(t[-1], eo[-1], " odometry", color=INK2, va="center", ha="left")
    a.text(t[-1], es[-1], " SLAM", color=INK2, va="center", ha="left")
    a.set_xlim(0, t[-1] * 1.18)
    a.set_ylim(0, None)
    a.grid(axis="y", color="#e6e5e0", lw=0.8)
    for s in ("top", "right"):
        a.spines[s].set_visible(False)
    a.set_xlabel("time (s)")
    a.set_ylabel("position error vs ground truth (m)")
    L = m["landing"]
    a.set_title(f"Pose error over the orbit\nlanding cells IoU {L['feasible_cell_iou']:.2f}; "
                f"{L['slam_candidates_valid_on_truth']}/{L['slam_candidates']} SLAM sites valid on truth", loc="left")
    for a in (ax[0, 0], ax[0, 1], ax[1, 0]):
        a.set_xlabel("x east (m)")
        a.set_ylabel("y north (m)")
    fig.suptitle("LiDAR SLAM on synthetic sweeps (KISS-ICP + noisy odometry) vs true rubble geometry",
                 color=INK, fontsize=11, x=0.01, ha="left")
    fig.savefig(path, dpi=110)
    plt.close(fig)
