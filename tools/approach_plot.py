#!/usr/bin/env python3
"""
Approach accuracy plots — "how close did the TCP actually get to the target?"

Reads the CSV the app appends one row per completed approach
(App/Indy7/approach_results/approach_log.csv, written by
CROS2PickBridge::TickApproachLog) and renders, per approach:

    approach_NNN_<class>.png   3-D target-vs-reached + per-axis bars + numbers
    approach_latest.png        a copy of the newest one (stable filename)
    approach_history.png       every approach so far, one chart

The per-approach figure shows exactly two points — where the arm was told to
go and where it ended up — inside 5 mm / 12 mm tolerance spheres.  It used to
also draw the pre-refine position across two flat panels (X-Y and X-Z); that
asked the reader to rebuild one 3-D fact from several 2-D pictures.  The
droop history is still recorded in the CSV (first_* columns).

Why a watcher instead of the app calling this itself: Indy7Ctrl runs
mlockall'd RT threads at 1 kHz.  fork()/system() marks every writable page
copy-on-write, so the next write from the RT thread takes a fault — a
textbook way to blow a 1 ms budget.  A separate process polling the CSV
costs the RT loop exactly nothing.

    python3 tools/approach_plot.py --watch          # auto, one PNG per approach
    python3 tools/approach_plot.py --all            # re-render everything
    python3 tools/approach_plot.py --csv <path>

run.sh starts `--watch --exit-with-parent` automatically unless
INDY7_NO_PLOT=1 is set.

All plot text is English on purpose: the board has no Korean matplotlib font
and missing glyphs render as boxes.
"""

import argparse
import csv
import math
import os
import shutil
import sys
import time

import matplotlib
matplotlib.use("Agg")            # headless board — must precede pyplot
import matplotlib.pyplot as plt
import numpy as np

# Mirrors CRobotIndy7::REFINE_TOL_M — refinement stops here, so it is the
# accuracy the control loop currently promises.  GOOD_MM is the grasp target
# from merge.md §9 (<5 mm), not something the stack achieves yet.
REFINE_TOL_MM = 12.0
GOOD_MM = 5.0

DEF_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "..", "App", "Indy7", "approach_results",
                       "approach_log.csv")


def _f(row, key, default=float("nan")):
    try:
        v = float(row.get(key, ""))
    except (TypeError, ValueError):
        return default
    return v


def _i(row, key, default=-1):
    try:
        return int(float(row.get(key, "")))
    except (TypeError, ValueError):
        return default


def load(path):
    """CSV -> list of dicts.  Tolerates a row being appended mid-read."""
    rows = []
    with open(path, newline="") as fh:
        for raw in csv.DictReader(fh):
            if raw.get("class") is None or raw.get("err_mm") in (None, ""):
                continue        # torn last line — it will be complete next tick
            rows.append({
                "when":  raw.get("iso_time", "?"),
                "cls":   raw.get("class", "?"),
                "goal":  np.array([_f(raw, "goal_x"), _f(raw, "goal_y"), _f(raw, "goal_z")]),
                "tcp":   np.array([_f(raw, "tcp_x"), _f(raw, "tcp_y"), _f(raw, "tcp_z")]),
                "first": np.array([_f(raw, "first_x"), _f(raw, "first_y"), _f(raw, "first_z")]),
                "err":   np.array([_f(raw, "err_x_mm"), _f(raw, "err_y_mm"), _f(raw, "err_z_mm")]),
                "err_mm":   _f(raw, "err_mm"),
                "first_mm": _f(raw, "first_err_mm"),
                "refine":   _i(raw, "refine_iter"),
                "zmargin":  _f(raw, "z_margin_m"),
                "std":   np.array([_f(raw, "vis_std_x_mm"), _f(raw, "vis_std_y_mm"),
                                   _f(raw, "vis_std_z_mm")]),
                "n":     _i(raw, "vis_n"),
                "seed":  _i(raw, "ik_seed"),
                "solves": _i(raw, "ik_solves"),
                "ik_ms":  _f(raw, "ik_ms"),
                "tilt":   _f(raw, "ik_tilt_deg"),
                "gap":    _f(raw, "ik_gap_rad"),
                "gap_ax": _i(raw, "ik_gap_axis"),
            })
    return rows


def grade(err_mm):
    if not np.isfinite(err_mm):
        return "#888888"
    if err_mm <= GOOD_MM:
        return "#1a9850"
    if err_mm <= REFINE_TOL_MM:
        return "#e08214"
    return "#d73027"


def _sphere(ax, r, color, label):
    """Tolerance ball around the target, drawn as three great circles.

    A full wireframe sphere was tried first and read as a ball of tangled
    string — it hid the one dot the figure exists to show.  Three orthogonal
    rings say "sphere of radius r" with six thin curves."""
    t = np.linspace(0, 2 * np.pi, 100)
    c, s, o = r * np.cos(t), r * np.sin(t), np.zeros_like(t)
    for k, (xs, ys, zs) in enumerate(((c, s, o), (c, o, s), (o, c, s))):
        ax.plot(xs, ys, zs, color=color, lw=1.1, alpha=0.55,
                label=label if k == 0 else None)


def plot_one(rec, idx, out_png):
    """One approach: where it was told to go vs where it ended up.

    Deliberately just those two points.  Earlier versions drew the pre-refine
    position and split the view into X-Y and X-Z panels; both made the reader
    reconstruct one 3-D fact from several 2-D pictures.  The droop history is
    still in the CSV (first_* columns) for anyone who wants it."""
    err = rec["err"]                       # mm, reached - target
    col = grade(rec["err_mm"])

    span = np.nanmax(np.abs(err))
    if not np.isfinite(span) or span <= 0:
        span = GOOD_MM
    lim = max(span * 1.6, REFINE_TOL_MM * 1.3)

    fig = plt.figure(figsize=(12.0, 6.4))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.35, 1.0],
                          height_ratios=[1.0, 1.0])
    fig.suptitle("Approach #%d  —  %s  |  miss %.1f mm"
                 % (idx, rec["cls"], rec["err_mm"]),
                 fontsize=14, y=0.97, color=col)

    # ---------------------------------------------------------------- 3-D
    ax = fig.add_subplot(gs[:, 0], projection="3d")
    _sphere(ax, GOOD_MM, "#1a9850", "grasp goal %g mm" % GOOD_MM)
    _sphere(ax, REFINE_TOL_MM, "#e08214", "refine tol %g mm" % REFINE_TOL_MM)

    # target at the origin
    ax.scatter([0], [0], [0], marker="+", s=190, c="k", linewidths=2.4,
               depthshade=False, label="target")
    # reached, with the miss drawn as a stick so the direction is obvious
    ax.plot([0, err[0]], [0, err[1]], [0, err[2]], "-", color=col, lw=1.8)
    ax.scatter([err[0]], [err[1]], [err[2]], s=110, c=col, depthshade=False,
               label="reached")

    # floor shadow + drop line: without them a 3-D scatter has no depth cue
    ax.plot([0, err[0]], [0, err[1]], [-lim, -lim], "-", color="#cccccc", lw=1.0)
    ax.scatter([err[0]], [err[1]], [-lim], s=45, c="#cccccc", depthshade=False)
    ax.plot([err[0], err[0]], [err[1], err[1]], [-lim, err[2]], ":",
            color="#bbbbbb", lw=0.9)

    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
    try:
        ax.set_box_aspect((1, 1, 1))       # matplotlib >= 3.3
    except AttributeError:
        pass
    ax.set_xlabel("\u0394X [mm]", fontsize=9, labelpad=1)
    ax.set_ylabel("\u0394Y [mm]", fontsize=9, labelpad=1)
    ax.set_zlabel("\u0394Z [mm]", fontsize=9, labelpad=1)
    ax.tick_params(labelsize=7.5)
    ax.view_init(elev=20, azim=-58)
    ax.set_title("target (+) vs reached  —  origin IS the target",
                 fontsize=10, pad=0)
    ax.legend(loc="upper left", fontsize=7.5, framealpha=0.85,
              bbox_to_anchor=(-0.06, 1.0))

    # ------------------------------------------------------- per-axis bars
    ax = fig.add_subplot(gs[0, 1])
    x = np.arange(4)
    vals = [err[0], err[1], err[2], rec["err_mm"]]
    ax.bar(x, vals, 0.55, color=col)
    for xi, v in zip(x, vals):
        ax.annotate("%.1f" % v, (xi, v), fontsize=8.5, ha="center",
                    va="bottom" if v >= 0 else "top")
    ax.axhline(0, color="k", lw=0.8)
    for tol, c in ((REFINE_TOL_MM, "#e08214"), (GOOD_MM, "#1a9850")):
        ax.axhline(tol, color=c, ls="--", lw=0.9)
        ax.axhline(-tol, color=c, ls="--", lw=0.9)
    ax.set_xlim(-0.6, 3.6)
    ax.set_xticks(x)
    ax.set_xticklabels(["\u0394X", "\u0394Y", "\u0394Z", "|\u0394|"])
    ax.set_ylabel("mm", fontsize=9)
    ax.set_title("per axis  (reached \u2212 target)", fontsize=10)
    ax.grid(axis="y", alpha=0.15)

    # ------------------------------------------------------------- numbers
    ax = fig.add_subplot(gs[1, 1])
    ax.axis("off")
    txt = [
        ("time",          rec["when"].replace("T", "  ")),
        ("target  [m]",   "%.4f, %.4f, %.4f" % tuple(rec["goal"])),
        ("reached [m]",   "%.4f, %.4f, %.4f" % tuple(rec["tcp"])),
        ("miss    [mm]",  "%.1f    (%+.1f, %+.1f, %+.1f)"
                          % (rec["err_mm"], err[0], err[1], err[2])),
        ("refine",        "%d pass(es)" % rec["refine"]),
        ("vision std",    "%.1f / %.1f / %.1f mm  (n=%d)"
                          % (rec["std"][0], rec["std"][1], rec["std"][2], rec["n"])),
        ("IK",            "seed %d, %d solve(s), %.1f ms, tilt %.0f\u00b0"
                          % (rec["seed"], rec["solves"], rec["ik_ms"], rec["tilt"])),
    ]
    y = 1.0
    for k, v in txt:
        ax.text(0.00, y, k, fontsize=8.5, va="top", color="#555555",
                family="monospace")
        ax.text(0.34, y, v, fontsize=8.5, va="top", family="monospace")
        y -= 0.128
    ax.text(0.00, y - 0.03,
            "NOTE: miss = TCP vs COMMANDED target.\n"
            "Camera-to-base calibration bias is NOT in it.",
            fontsize=7.5, va="top", color="#a05000")

    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out_png, dpi=140)
    plt.close(fig)
    return out_png


def plot_history(rows, out_png):
    n = len(rows)
    idx = np.arange(1, n + 1)
    err = np.array([r["err_mm"] for r in rows])

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(max(8.0, 0.55 * n + 4), 7.6),
                                   sharex=True)
    fig.suptitle("Approach accuracy history — %d approach(es)" % n,
                 fontsize=13)

    ax1.bar(idx, err, 0.55, color=[grade(e) for e in err], zorder=2)
    ax1.axhline(REFINE_TOL_MM, color="#e08214", ls="--", lw=1.0)
    ax1.axhline(GOOD_MM, color="#1a9850", ls="--", lw=1.0)
    # y-data / x-axes coords: pins the labels inside the right edge whatever
    # the bar count is (plain data coords got clipped at low n).
    for tol, c, lab in ((REFINE_TOL_MM, "#e08214", "refine tol %g mm" % REFINE_TOL_MM),
                        (GOOD_MM, "#1a9850", "grasp goal %g mm" % GOOD_MM)):
        ax1.text(0.995, tol, lab, color=c, fontsize=8, ha="right", va="bottom",
                 transform=ax1.get_yaxis_transform())
    for i, r in zip(idx, rows):
        ax1.annotate("%.1f" % r["err_mm"], (i, r["err_mm"]), fontsize=7,
                     ha="center", va="bottom")
    ax1.set_ylabel("miss [mm]")
    ax1.set_title("green ≤ %g mm, orange ≤ %g mm, red above"
                  % (GOOD_MM, REFINE_TOL_MM), fontsize=9)
    ax1.grid(axis="y", alpha=0.15)

    for k, (lab, mk) in enumerate((("ΔX", "o"), ("ΔY", "s"), ("ΔZ", "^"))):
        ax2.plot(idx, [r["err"][k] for r in rows], mk + "-", ms=5, lw=1.0,
                 label=lab)
    ax2.axhline(0, color="k", lw=0.8)
    ax2.set_ylabel("signed error [mm]")
    ax2.set_xlabel("approach #  (label = class)")
    ax2.set_xticks(idx)
    # Deliberately NOT colour-coded by class: the bars above use colour for the
    # pass/fail grade, and two colour languages in one figure read as one.
    ax2.set_xticklabels(["%d\n%s" % (i, r["cls"][:12]) for i, r in zip(idx, rows)],
                        fontsize=7)
    ax2.legend(loc="upper left", fontsize=8, ncol=3)
    ax2.grid(alpha=0.15)

    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out_png, dpi=140)
    plt.close(fig)
    return out_png


def render(csv_path, out_dir, redo_all, seen):
    rows = load(csv_path)
    if not rows:
        return 0
    os.makedirs(out_dir, exist_ok=True)
    made = 0
    latest = None
    for i, r in enumerate(rows, 1):
        safe = "".join(ch if ch.isalnum() else "_" for ch in r["cls"])[:24]
        png = os.path.join(out_dir, "approach_%03d_%s.png" % (i, safe))
        latest = png
        if not redo_all and (png in seen or os.path.exists(png)):
            continue
        plot_one(r, i, png)
        seen.add(png)
        made += 1
        print("  saved %s" % png)
    if made or redo_all:
        if latest:
            shutil.copyfile(latest, os.path.join(out_dir, "approach_latest.png"))
        plot_history(rows, os.path.join(out_dir, "approach_history.png"))
        print("  saved %s" % os.path.join(out_dir, "approach_history.png"))
    return made


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", default=os.environ.get("INDY7_APPROACH_LOG", DEF_CSV))
    ap.add_argument("--out", default=None, help="default: <csv dir>/plots")
    ap.add_argument("--all", action="store_true", help="re-render every row")
    ap.add_argument("--watch", action="store_true",
                    help="poll the CSV and plot each new approach")
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--exit-with-parent", action="store_true",
                    help="quit when the launching process is gone (run.sh)")
    a = ap.parse_args()

    csv_path = os.path.abspath(a.csv)
    out_dir = a.out or os.path.join(os.path.dirname(csv_path), "plots")

    if not a.watch:
        if not os.path.exists(csv_path):
            print("ERROR: no approach log at %s\n"
                  "       Run an approach first ('p' -> digit -> 'v')." % csv_path)
            return 1
        n = render(csv_path, out_dir, a.all, set())
        print("%d new plot(s) in %s" % (n, out_dir))
        return 0

    ppid0 = os.getppid()
    print("[approach-plot] watching %s -> %s" % (csv_path, out_dir))
    seen, last_sig = set(), None
    while True:
        if a.exit_with_parent and os.getppid() != ppid0:
            print("[approach-plot] launcher exited — stopping")
            return 0
        try:
            st = os.stat(csv_path)
            sig = (st.st_mtime, st.st_size)
        except OSError:
            sig = None
        if sig is not None and sig != last_sig:
            last_sig = sig
            try:
                if render(csv_path, out_dir, False, seen):
                    print("[approach-plot] updated %s" % out_dir)
            except Exception as exc:            # never kill the watcher
                print("[approach-plot] WARN: %s" % exc)
        time.sleep(a.interval)


if __name__ == "__main__":
    sys.exit(main())
