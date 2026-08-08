#!/usr/bin/env python3
"""[kv260-merge] 3D view of the reach map produced by tools/reachmap.cpp.

    ./bin/reachmap.out --step 0.02 --z 0.10,0.125,...,0.50 --out vol.csv
    python3 tools/reachmap_plot3d.py vol.csv -o reachmap3d.png

The 2D plotter draws one panel per scan height; this stacks the same cells into
a volume. What it is for is the shape of the REFUSED region — the branch-flip
wedge grows with height, and that is invisible when the planes are separated.

Refused cells are drawn solid; accepted cells are a faint haze (subsampled)
that only exists to show the envelope they sit in. Drawing all 21k accepted
cells opaque would bury the thing worth looking at.

Same caveat as the 2D map, and it matters more here because a 3D picture of a
robot workspace invites the assumption that it is a model of the workcell: it
is NOT. RBDL carries no geometry, so there is no collision checking of any
kind. The table, the camera rig, the objects and the arm's own links are all
absent. A cell drawn green means the IK accepts that point, never that the arm
can safely reach it.
"""

import argparse
import csv
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

REFUSE_COLOUR = {
    "BRANCH":      "#c62828",
    "TILT":        "#ef6c00",
    "NO_SOLUTION": "#455a64",
    "LIMITS":      "#6a1b9a",
    "NO_SEED":     "#000000",
}


def read_map(path):
    rows, marks, head = [], [], []
    with open(path, newline="") as fh:
        header = None
        for line in fh:
            if line.startswith("#"):
                parts = line[1:].split()
                if parts and parts[0] == "mark" and len(parts) >= 4:
                    marks.append((float(parts[1]), float(parts[2]),
                                  float(parts[3]),
                                  parts[4] if len(parts) > 4 else ""))
                else:
                    head.append(line[1:].strip())
                continue
            if header is None:
                header = next(csv.reader([line]))
                continue
            rows.append(dict(zip(header, next(csv.reader([line])))))
    if not rows:
        sys.exit(f"{path}: no data rows")
    return rows, marks, head


def draw(ax, xyz, verdicts, marks, haze_every):
    ok = verdicts == "PASS"

    # Envelope first so the refused cells draw over it.
    hz = xyz[ok][::haze_every]
    ax.scatter(hz[:, 0], hz[:, 1], hz[:, 2], s=3, c="#2e7d32", alpha=0.05,
               linewidths=0, depthshade=False)

    for name, colour in REFUSE_COLOUR.items():
        sel = verdicts == name
        if not sel.any():
            continue
        p = xyz[sel]
        ax.scatter(p[:, 0], p[:, 1], p[:, 2], s=9, c=colour, alpha=0.35,
                   linewidths=0, depthshade=False)

    for j, (mx, my, mz, label) in enumerate(marks):
        ax.scatter([mx], [my], [mz], s=90, c="white", edgecolors="k",
                   linewidths=0.7, marker="*", depthshade=False)
        # Alternate the offset: the objects sit close together and the labels
        # collide on every projection otherwise.
        ax.text(mx, my, mz + (0.030 if j % 2 else -0.028), label, fontsize=7,
                ha="center")

    ax.set_xlabel("base x [m]", fontsize=8)
    ax.set_ylabel("base y [m]", fontsize=8)
    ax.set_zlabel("base z [m]", fontsize=8)
    ax.tick_params(labelsize=7)
    # Equal aspect in x/y, z left free: the sweep is a thin slab and a true
    # 1:1:1 box would squash it to a line.
    ax.set_box_aspect((xyz[:, 0].ptp(), xyz[:, 1].ptp(), 0.7 * xyz[:, 1].ptp()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="output of reachmap.out (several --z planes)")
    ap.add_argument("-o", "--out", default="reachmap3d.png")
    ap.add_argument("--haze-every", type=int, default=4,
                    help="draw every Nth accepted cell as envelope haze")
    args = ap.parse_args()

    rows, marks, _ = read_map(args.csv)
    xyz = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows])
    verdicts = np.array([r["verdict"] for r in rows])

    views = [(22, -60, "iso", None),
             (2, -90, "side (x–z): the refused region is a CEILING", "y"),
             (89, -90, "top (x–y)", "z")]
    fig = plt.figure(figsize=(6.0 * len(views), 6.4))
    for i, (elev, azim, title, flat) in enumerate(views):
        ax = fig.add_subplot(1, len(views), i + 1, projection="3d")
        draw(ax, xyz, verdicts, marks, args.haze_every)
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title, fontsize=9)
        # In an edge-on projection the collapsed axis's ticks pile up on the
        # frame and read as noise — drop the axis rather than the view.
        if flat == "y":
            ax.set_yticks([]); ax.set_ylabel("")
        elif flat == "z":
            ax.set_zticks([]); ax.set_zlabel("")

    handles = [Line2D([], [], marker="o", linestyle="", color="#2e7d32",
                      alpha=0.5, label="PASS (envelope haze)")]
    handles += [Line2D([], [], marker="o", linestyle="", color=c, label=n)
                for n, c in REFUSE_COLOUR.items() if (verdicts == n).any()]
    fig.legend(handles=handles, loc="lower center", ncol=len(handles),
               frameon=False, fontsize=9)

    npass = int((verdicts == "PASS").sum())
    fig.suptitle(
        f"Ready-seed IK reach map in 3D — {npass}/{len(rows)} cells accepted\n"
        "solid = refused by the solver;  no collision model, so PASS means "
        "the IK accepts the point, not that the arm can safely reach it",
        fontsize=11)
    fig.tight_layout(rect=(0, 0.06, 1, 0.93))
    fig.savefig(args.out, dpi=140)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
