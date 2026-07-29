#!/usr/bin/env python3
"""
Approach accuracy plots — "how close did the TCP actually get to the target?"

Reads the CSV the app appends one row per completed approach
(App/Indy7/approach_results/approach_log.csv, written by
CROS2PickBridge::TickApproachLog) and renders, per approach:

    approach_NNN_<class>.png   bullseye + side view + per-axis bars + metadata
    approach_latest.png        a copy of the newest one (stable filename)
    approach_history.png       every approach so far, one chart

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


def _rings(ax, lim):
    """Tolerance rings + crosshair, target at the origin."""
    for r, style in ((GOOD_MM, "-"), (REFINE_TOL_MM, "--"), (25.0, ":"), (50.0, ":")):
        if r <= lim * 1.05:
            ax.add_patch(plt.Circle((0, 0), r, fill=False, ls=style,
                                    ec="#999999", lw=0.9, zorder=1))
            ax.annotate("%g" % r, (r * 0.707, r * 0.707), color="#999999",
                        fontsize=7, ha="left", va="bottom", zorder=1)
    ax.axhline(0, color="#cccccc", lw=0.8, zorder=0)
    ax.axvline(0, color="#cccccc", lw=0.8, zorder=0)
    ax.plot(0, 0, marker="+", ms=13, mew=2.0, color="k", zorder=5)


def _scatter_panel(ax, a_h, a_v, b_h, b_v, lim, title, xlabel, ylabel, col):
    """Bullseye zoomed on the FINAL error.  The pre-refine point is usually an
    order of magnitude further out (CTC droop is centimetres, the residual is
    millimetres); letting it set the scale would squash the one number the
    operator actually came to read.  So it gets clamped to the frame with its
    true magnitude printed next to it."""
    _rings(ax, lim)
    if np.isfinite(b_h) and np.isfinite(b_v):
        r = math.hypot(b_h, b_v)
        edge = r > lim * 0.94
        if edge and r > 0:
            k = lim * 0.94 / r
            p_h, p_v = b_h * k, b_v * k
        else:
            p_h, p_v = b_h, b_v
        ax.annotate("", xy=(a_h, a_v), xytext=(p_h, p_v), zorder=3,
                    arrowprops=dict(arrowstyle="->", color="#7f7f7f",
                                    lw=1.2, ls=":"))
        ax.plot(p_h, p_v, "o", ms=7, mfc="none", mec="#7f7f7f", mew=1.6,
                zorder=4,
                label="before refine" + (" (off scale)" if edge else ""))
        if edge:
            ax.annotate("%.0f mm" % r, (p_h, p_v), fontsize=7.5,
                        color="#7f7f7f", zorder=4,
                        textcoords="offset points",
                        xytext=(-6 if p_h > 0 else 6, -9 if p_v > 0 else 9),
                        ha="right" if p_h > 0 else "left",
                        va="top" if p_v > 0 else "bottom")
    ax.plot(a_h, a_v, "o", ms=10, color=col, zorder=6, label="reached")
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=10)
    ax.set_xlabel(xlabel, fontsize=9)
    ax.set_ylabel(ylabel, fontsize=9)
    ax.grid(alpha=0.15)


def plot_one(rec, idx, out_png):
    err, first = rec["err"], rec["first"] - rec["goal"]
    first_mm_axes = first * 1e3 if np.all(np.isfinite(first)) else np.full(3, np.nan)
    have_first = np.all(np.isfinite(first_mm_axes))
    col = grade(rec["err_mm"])

    span = np.nanmax(np.abs(err))
    if not np.isfinite(span) or span <= 0:
        span = GOOD_MM
    lim = max(span * 1.55, REFINE_TOL_MM * 1.35)

    fig = plt.figure(figsize=(11.5, 8.2))
    fig.suptitle("Approach #%d  —  %s  @ base (%.3f, %.3f, %.3f) m"
                 % (idx, rec["cls"], rec["goal"][0], rec["goal"][1], rec["goal"][2]),
                 fontsize=13, y=0.975)

    # --- top view: the bullseye.  Target at the crosshair, TCP where it landed
    ax = fig.add_subplot(2, 2, 1)
    _scatter_panel(ax, err[0], err[1],
                   first_mm_axes[0] if have_first else np.nan,
                   first_mm_axes[1] if have_first else np.nan,
                   lim, "Top view (X–Y)", "ΔX [mm]", "ΔY [mm]", col)
    ax.legend(loc="upper right", fontsize=7.5, framealpha=0.9)

    # --- side view: same units, so height error is read on the same scale
    ax = fig.add_subplot(2, 2, 2)
    _scatter_panel(ax, err[0], err[2],
                   first_mm_axes[0] if have_first else np.nan,
                   first_mm_axes[2] if have_first else np.nan,
                   lim, "Side view (X–Z)", "ΔX [mm]", "ΔZ [mm]", col)

    # --- per-axis bars, before/after
    ax = fig.add_subplot(2, 2, 3)
    labels = ["ΔX", "ΔY", "ΔZ", "|Δ|"]
    after = [err[0], err[1], err[2], rec["err_mm"]]
    x = np.arange(4)
    if have_first:
        before = [first_mm_axes[0], first_mm_axes[1], first_mm_axes[2],
                  rec["first_mm"]]
        ax.bar(x - 0.2, before, 0.38, color="#bbbbbb", label="before refine")
        ax.bar(x + 0.2, after, 0.38, color=col, label="after refine")
    else:
        ax.bar(x, after, 0.5, color=col, label="reached")
    for xi, v in zip(x + (0.2 if have_first else 0.0), after):
        ax.annotate("%.1f" % v, (xi, v), fontsize=8, ha="center",
                    va="bottom" if v >= 0 else "top")
    ax.axhline(0, color="k", lw=0.8)
    for tol, c, lab in ((REFINE_TOL_MM, "#e08214", "refine tol %g mm" % REFINE_TOL_MM),
                        (GOOD_MM, "#1a9850", "grasp goal %g mm" % GOOD_MM)):
        ax.axhline(tol, color=c, ls="--", lw=0.9)
        ax.axhline(-tol, color=c, ls="--", lw=0.9)
        ax.annotate(lab, (-0.55, -tol), color=c, fontsize=7, ha="left",
                    va="top")
    ax.set_xlim(-0.62, 3.62)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("error [mm]  (reached − target)", fontsize=9)
    ax.set_title("Per-axis error", fontsize=10)
    ax.legend(loc="upper left", fontsize=7.5)
    ax.grid(axis="y", alpha=0.15)

    # --- the numbers, so the PNG stands alone without the CSV
    ax = fig.add_subplot(2, 2, 4)
    ax.axis("off")
    refine_line = ("%.1f → %.1f mm in %d pass(es)"
                   % (rec["first_mm"], rec["err_mm"], rec["refine"])
                   if have_first else "not measured")
    txt = [
        ("time",            rec["when"]),
        ("target  [m]",     "%.4f, %.4f, %.4f" % tuple(rec["goal"])),
        ("reached [m]",     "%.4f, %.4f, %.4f" % tuple(rec["tcp"])),
        ("error   [mm]",    "%+.1f, %+.1f, %+.1f" % tuple(rec["err"])),
        ("|error| [mm]",    "%.1f" % rec["err_mm"]),
        ("refinement",      refine_line),
        ("hover margin",    "%.0f mm above the object" % (rec["zmargin"] * 1e3)),
        ("vision std [mm]", "%.1f / %.1f / %.1f  (n=%d)"
                            % (rec["std"][0], rec["std"][1], rec["std"][2], rec["n"])),
        ("IK",              "seed %d, %d solve(s), %.1f ms"
                            % (rec["seed"], rec["solves"], rec["ik_ms"])),
        ("IK tool tilt",    "%.1f deg from q_ready" % rec["tilt"]),
        ("IK branch gap",   "%.2f rad (J%d)" % (rec["gap"], rec["gap_ax"])),
    ]
    y = 0.97
    for k, v in txt:
        ax.text(0.00, y, k, fontsize=9, va="top", color="#555555",
                family="monospace")
        ax.text(0.42, y, v, fontsize=9, va="top", family="monospace")
        y -= 0.088
    ax.text(0.00, y - 0.02,
            "NOTE: |error| is TCP-vs-commanded-target only.\n"
            "Perception bias (camera-to-base calibration) is NOT in it.",
            fontsize=8, va="top", color="#a05000")

    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out_png, dpi=140)
    plt.close(fig)
    return out_png


def plot_history(rows, out_png):
    n = len(rows)
    idx = np.arange(1, n + 1)
    err = np.array([r["err_mm"] for r in rows])
    first = np.array([r["first_mm"] for r in rows])

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(max(8.0, 0.55 * n + 4), 7.6),
                                   sharex=True)
    fig.suptitle("Approach accuracy history — %d approach(es)" % n,
                 fontsize=13)

    ok = np.isfinite(first)
    if ok.any():
        ax1.bar(idx[ok], first[ok], 0.62, color="#dddddd", label="before refine",
                zorder=1)
    ax1.bar(idx, err, 0.42, color=[grade(e) for e in err], zorder=2,
            label="final |error|")
    ax1.axhline(REFINE_TOL_MM, color="#e08214", ls="--", lw=1.0)
    ax1.axhline(GOOD_MM, color="#1a9850", ls="--", lw=1.0)
    ax1.annotate("refine tol %g mm" % REFINE_TOL_MM, (0.52, REFINE_TOL_MM),
                 color="#e08214", fontsize=8, ha="left", va="bottom")
    ax1.annotate("grasp goal %g mm" % GOOD_MM, (0.52, GOOD_MM),
                 color="#1a9850", fontsize=8, ha="left", va="bottom")
    for i, r in zip(idx, rows):
        ax1.annotate("%.1f" % r["err_mm"], (i, r["err_mm"]), fontsize=7,
                     ha="center", va="bottom")
    ax1.set_ylabel("|error| [mm]")
    ax1.set_title("green ≤ %g mm, orange ≤ %g mm, red above"
                  % (GOOD_MM, REFINE_TOL_MM), fontsize=9)
    ax1.legend(loc="upper left", fontsize=8)
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
