#!/usr/bin/env python3
"""[kv260-merge] Plot the reach map produced by tools/reachmap.cpp.

    python3 tools/reachmap_plot.py sweep.csv -o reachmap.png

One panel per scan height. Colour is SolveReadyIK's verdict at that cell:

    PASS         the solver accepts the point
    BRANCH       reachable only by flipping the arm branch (E27) — refused
    TILT         reachable only past the 60 deg tool-tilt cap — refused
    NO_SOLUTION  position unreachable even with orientation unconstrained
    LIMITS       solution outside the URDF joint limits

What is NOT in this picture: the table, the camera rig, the objects and the
robot's own links. RBDL carries no geometry, so nothing here is collision
checked — a PASS cell can still be a posture that drives through the table.
The base origin, the radius cap and any --mark points are drawn as annotation
only. Treat this as a map of the SOLVER, not of the workcell.
"""

import argparse
import csv
import sys
from collections import OrderedDict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

# Ordered worst-to-best so the legend reads as a severity scale.
VERDICTS = OrderedDict([
    ("PASS",        "#2e7d32"),
    ("BRANCH",      "#c62828"),
    ("TILT",        "#ef6c00"),
    ("NO_SOLUTION", "#455a64"),
    ("LIMITS",      "#6a1b9a"),
    ("NO_SEED",     "#000000"),
])


def read_map(path):
    rows, marks, meta = [], [], {}
    with open(path, newline="") as fh:
        header = None
        for line in fh:
            if line.startswith("#"):
                parts = line[1:].split()
                if parts and parts[0] == "mark" and len(parts) >= 4:
                    marks.append((float(parts[1]), float(parts[2]),
                                  float(parts[3]),
                                  parts[4] if len(parts) > 4 else ""))
                elif parts and parts[0] == "q_ready":
                    meta["q_ready"] = line[1:].strip()
                elif parts and parts[0] == "rmax=":
                    pass
                else:
                    meta.setdefault("head", []).append(line[1:].strip())
                continue
            if header is None:
                header = next(csv.reader([line]))
                continue
            rows.append(dict(zip(header, next(csv.reader([line])))))
    if not rows:
        sys.exit(f"{path}: no data rows")
    return rows, marks, meta


def rmax_from_meta(meta):
    for line in meta.get("head", []):
        for tok in line.split():
            if tok.startswith("rmax="):
                return float(tok[len("rmax="):])
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="output of reachmap.out")
    ap.add_argument("-o", "--out", default="reachmap.png")
    ap.add_argument("--metric", choices=["verdict", "tilt", "gap", "ms", "w"],
                    default="verdict",
                    help="verdict map (default) or a scalar field")
    args = ap.parse_args()

    rows, marks, meta = read_map(args.csv)
    rmax = rmax_from_meta(meta)
    zs = sorted({float(r["z"]) for r in rows})

    ncol = min(len(zs), 3)
    nrow = (len(zs) + ncol - 1) // ncol
    fig, axes = plt.subplots(nrow, ncol, figsize=(5.2 * ncol, 5.0 * nrow),
                             squeeze=False)

    for k, z in enumerate(zs):
        ax = axes[k // ncol][k % ncol]
        sub = [r for r in rows if abs(float(r["z"]) - z) < 1e-9]
        x = np.array([float(r["x"]) for r in sub])
        y = np.array([float(r["y"]) for r in sub])

        if args.metric == "verdict":
            for name, colour in VERDICTS.items():
                sel = [i for i, r in enumerate(sub) if r["verdict"] == name]
                if sel:
                    ax.scatter(x[sel], y[sel], s=6, c=colour, marker="s",
                               linewidths=0, label=name)
        else:
            col = {"tilt": "tilt_deg", "gap": "gap_rad",
                   "ms": "ms", "w": "w"}[args.metric]
            # Refused cells have no meaningful scalar — grey them out rather
            # than letting their numbers colour the field.
            ok = [i for i, r in enumerate(sub) if r["verdict"] == "PASS"]
            bad = [i for i, r in enumerate(sub) if r["verdict"] != "PASS"]
            if bad:
                ax.scatter(x[bad], y[bad], s=6, c="#dddddd", marker="s",
                           linewidths=0)
            v = np.array([float(sub[i][col]) for i in ok])
            sc = ax.scatter(x[ok], y[ok], s=6, c=v, marker="s", linewidths=0,
                            cmap="viridis")
            fig.colorbar(sc, ax=ax, shrink=0.8, label=col)

        # Frame the DATA, not the workspace: a zoomed sweep drawn on full-arm
        # axes shows nothing. Padded by one cell so edge points stay visible.
        pad = max(0.01, 0.02 * max(x.ptp(), y.ptp()))
        ax.set_xlim(x.min() - pad, x.max() + pad)
        ax.set_ylim(y.min() - pad, y.max() + pad)

        # Annotation only — none of this is a collision model.
        if x.min() - pad <= 0.0 <= x.max() + pad:
            ax.plot(0.0, 0.0, "k+", markersize=10)
            ax.annotate("base", (0.0, 0.0), textcoords="offset points",
                        xytext=(6, 4), fontsize=7)
        if rmax:
            th = np.linspace(-np.pi / 2, np.pi / 2, 400)
            ax.plot(rmax * np.cos(th), rmax * np.sin(th), "k--", lw=0.8,
                    alpha=0.5)
        # A mark belongs to the panel whose plane is nearest its own z —
        # drawing every object on every height would suggest the map says
        # something about it there, which it does not.
        for j, (mx, my, mz, label) in enumerate(marks):
            if min(range(len(zs)), key=lambda i: abs(zs[i] - mz)) != k:
                continue
            ax.plot(mx, my, "w*", markersize=11, markeredgecolor="k",
                    markeredgewidth=0.6)
            ax.annotate(f"{label} (z={mz:.2f})", (mx, my),
                        textcoords="offset points",
                        xytext=(9, 8 if j % 2 else -12), fontsize=7,
                        arrowprops=dict(arrowstyle="-", lw=0.5, color="k"),
                        bbox=dict(boxstyle="round,pad=0.15", fc="white",
                                  ec="none", alpha=0.75))

        npass = sum(1 for r in sub if r["verdict"] == "PASS")
        ax.set_title(f"z = {z:.2f} m   ({npass}/{len(sub)} accepted)",
                     fontsize=10)
        ax.set_xlabel("base x [m]")
        ax.set_ylabel("base y [m]")
        ax.set_aspect("equal")
        ax.grid(alpha=0.15, lw=0.5)

    for k in range(len(zs), nrow * ncol):
        axes[k // ncol][k % ncol].axis("off")

    if args.metric == "verdict":
        handles = [Line2D([], [], marker="s", linestyle="", color=c, label=n)
                   for n, c in VERDICTS.items()
                   if any(r["verdict"] == n for r in rows)]
        fig.legend(handles=handles, loc="lower center", ncol=len(handles),
                   frameon=False, fontsize=9)

    fig.suptitle("Ready-seed IK reach map — solver verdict per target\n"
                 "no collision model: PASS means the IK accepts the point, "
                 "not that the arm can safely reach it", fontsize=11)
    fig.tight_layout(rect=(0, 0.05, 1, 0.95))
    fig.savefig(args.out, dpi=140)
    print(f"wrote {args.out}")
    if "q_ready" in meta:
        print(f"  {meta['q_ready']}")


if __name__ == "__main__":
    main()
