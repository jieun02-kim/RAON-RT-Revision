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
import glob
import math
import os
import re
import shutil
import subprocess
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
                "unix":  _f(raw, "unix_s"),
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

    fig = plt.figure(figsize=(13.6, 6.4))
    gs = fig.add_gridspec(2, 3, width_ratios=[1.30, 0.42, 1.0],
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
    # Spell out that the origin is the HOVER point. Field feedback 2026-07-29:
    # with "target" alone the reader expects the reached dot to sit ~150 mm
    # high, because that is what the arm visibly does above the object. The
    # margin is already inside the commanded goal, so it is NOT an error and
    # must not look like one.
    ax.set_title("target (+) vs reached  —  origin = HOVER point\n"
                 "(%.0f mm above the object; the margin is already in the goal)"
                 % (rec["zmargin"] * 1e3), fontsize=9.5, pad=0)
    ax.legend(loc="upper left", fontsize=7.5, framealpha=0.85,
              bbox_to_anchor=(-0.06, 1.0))

    # -------------------------------------------- true-scale side elevation
    # The object cannot go in the 3-D panel: it is 150 mm below the target and
    # that panel is zoomed to ~15 mm, so it would either fall off the box or
    # force a 10x zoom-out that hides the millimetres. This narrow panel is
    # drawn at TRUE scale instead (equal aspect), which makes the honest point
    # by itself -- next to the hover height the miss is a hair.
    ax = fig.add_subplot(gs[:, 1])
    hov = rec["zmargin"] * 1e3                      # [mm]
    hmiss = math.hypot(err[0], err[1])              # horizontal miss [mm]
    xr = max(22.0, hmiss * 2.6)

    ax.plot([0, 0], [0, hov], "--", color="#bbbbbb", lw=1.0, zorder=1)
    ax.annotate("", xy=(0, hov), xytext=(0, 0), zorder=2,
                arrowprops=dict(arrowstyle="<->", color="#888888", lw=1.0))
    ax.annotate("%.0f mm\nhover" % hov, (0, hov * 0.5), fontsize=8,
                color="#666666", ha="left", va="center",
                textcoords="offset points", xytext=(5, 0))

    # object: a squat marker on the table line, drawn where the camera put it
    ax.axhline(0, color="#cccccc", lw=1.0, zorder=0)
    ax.plot(0, 0, marker="s", ms=11, color="#8c6d4f", zorder=4)
    ax.annotate("object", (0, 0), fontsize=8, color="#8c6d4f", ha="center",
                va="top", textcoords="offset points", xytext=(0, -10))
    ax.plot(0, hov, marker="+", ms=13, mew=2.2, color="k", zorder=5)
    ax.plot(hmiss, hov + err[2], "o", ms=9, color=col, zorder=6)

    ax.set_xlim(-xr, xr)
    ax.set_ylim(-0.16 * hov, hov * 1.30)
    ax.set_aspect("equal")
    ax.set_xlabel("horizontal\nmiss [mm]", fontsize=8)
    ax.set_ylabel("height above object [mm]", fontsize=8)
    ax.set_title("true scale\n(+ = target, dot = reached)", fontsize=9)
    ax.tick_params(labelsize=7)
    ax.grid(alpha=0.15)

    # ------------------------------------------------------- per-axis bars
    ax = fig.add_subplot(gs[0, 2])
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
    ax = fig.add_subplot(gs[1, 2])
    ax.axis("off")
    obj = rec["goal"].copy()
    obj[2] -= rec["zmargin"]           # what the camera actually saw
    txt = [
        ("time",          rec["when"].replace("T", "  ")),
        ("object  [m]",   "%.4f, %.4f, %.4f" % tuple(obj)),
        ("target  [m]",   "%.4f, %.4f, %.4f   (= object z +%.0f mm)"
                          % (rec["goal"][0], rec["goal"][1], rec["goal"][2],
                             rec["zmargin"] * 1e3)),
        ("reached [m]",   "%.4f, %.4f, %.4f" % tuple(rec["tcp"])),
        ("",              "\u2192 %.0f mm above the object"
                          % ((rec["tcp"][2] - obj[2]) * 1e3)),
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
        ax.text(0.30, y, v, fontsize=8.5, va="top", family="monospace",
                color="#1a6faf" if k == "" else "black")
        y -= 0.107
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


# ------------------------------------------------------------- trajectories
# Per-approach 3-D trajectory + three-way task-space comparison:
#   goal_*   what IK was asked to reach (incl. the refine bias steps)
#   ref_*    FK of the per-cycle joint REFERENCE (fk_replay output)
#   tcp_*    FK of the ACTUAL joints, computed by the RT loop itself
# The app arms its DataLog logger for exactly one approach (m_bLogUntilDone),
# so one DataLog file = one approach; pairing to approach_log rows is by
# wall-clock proximity (both stamps land within ~2 s of the done edge).
# FK happens OFFLINE via fk_replay.out, which links the robot's own
# FullDynControllerRT.o — the plots and the robot share one kinematics.

_fk_hint_shown = [False]


def tool_paths():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return (os.path.join(root, "App", "Indy7", "bin", "fk_replay.out"),
            os.path.join(root, "App", "Indy7", "indy7.urdf"))


def datalog_stamp(path):
    m = re.search(r"DataLog_(\d{8}_\d{6})\.csv$", os.path.basename(path))
    if not m:
        return None
    try:
        return time.mktime(time.strptime(m.group(1), "%Y%m%d_%H%M%S"))
    except ValueError:
        return None


def pair_datalogs(rows, rtlog_dir, tol_s=15.0):
    """row index -> DataLog path, nearest-stamp within tol, each file once.
    Manual 'l'/'a' logs have no matching row and stay unpaired; rows from
    before this feature have no file and stay unpaired — both are normal."""
    files = []
    for p in sorted(glob.glob(os.path.join(rtlog_dir, "DataLog_*.csv"))):
        if p.endswith("_fk.csv"):
            continue
        s = datalog_stamp(p)
        if s is not None:
            files.append([s, p, False])
    out = {}
    for i, r in enumerate(rows):
        if not np.isfinite(r["unix"]):
            continue
        best = None
        for f in files:
            if f[2]:
                continue
            d = abs(f[0] - r["unix"])
            if d <= tol_s and (best is None or d < best[0]):
                best = (d, f)
        if best:
            best[1][2] = True
            out[i] = best[1][1]
    return out


def ensure_fk(datalog, fk_bin, urdf):
    """Run fk_replay once per DataLog (cached by mtime). Returns aug path."""
    aug = datalog[:-4] + "_fk.csv"
    if (os.path.exists(aug) and
            os.path.getmtime(aug) >= os.path.getmtime(datalog)):
        return aug
    if not os.path.exists(fk_bin):
        if not _fk_hint_shown[0]:
            _fk_hint_shown[0] = True
            print("[approach-plot] fk_replay.out missing — trajectory plots "
                  "skipped. Build it: cd App/Indy7 && make fk_replay")
        return None
    r = subprocess.run([fk_bin, urdf, datalog, aug],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if r.returncode != 0 or not os.path.exists(aug):
        print("[approach-plot] WARN: fk_replay failed on %s" % datalog)
        return None
    return aug


def load_traj(aug_path):
    cols = ("timestamp_ns", "tcp_x", "tcp_y", "tcp_z", "goal_x", "goal_y",
            "goal_z", "ref_x", "ref_y", "ref_z", "act_fk_x", "act_fk_y",
            "act_fk_z")
    data = {k: [] for k in cols}
    with open(aug_path, newline="") as fh:
        for raw in csv.DictReader(fh):
            try:
                vals = [float(raw[k]) for k in cols]
            except (KeyError, TypeError, ValueError):
                continue
            for k, v in zip(cols, vals):
                data[k].append(v)
    if len(data["timestamp_ns"]) < 10:
        return None
    a = {k: np.array(v) for k, v in data.items()}
    a["t"] = (a["timestamp_ns"] - a["timestamp_ns"][0]) * 1e-9
    return a


def plot_traj(rec, idx, a, out_png):
    t = a["t"]
    n = len(t)
    col = grade(rec["err_mm"])
    goal = np.stack([a["goal_x"], a["goal_y"], a["goal_z"]])
    ref = np.stack([a["ref_x"], a["ref_y"], a["ref_z"]])
    act = np.stack([a["tcp_x"], a["tcp_y"], a["tcp_z"]])

    # The free cross-check: the logged tcp came from the RT loop's FK, the
    # act_fk columns from the offline tool. Anything above numerical noise
    # means the plot pipeline no longer shares the robot's kinematics.
    fk = np.stack([a["act_fk_x"], a["act_fk_y"], a["act_fk_z"]])
    xchk_mm = float(np.max(np.linalg.norm(act - fk, axis=0))) * 1e3

    # refine pass boundaries = the moments the commanded goal moved
    steps = np.where(np.linalg.norm(np.diff(goal, axis=1), axis=0) > 1e-9)[0]

    # The one-shot plan — what we WANTED: pass-1's reference held at its
    # endpoint. Not recomputed; each IK pass already plans straight to its
    # goal, so pass 1 of the log IS the one-shot trajectory. ideal and ref
    # coincide until the first refine re-target, then ref chases biased
    # goals while ideal keeps the original promise. Pass-1 boundary = first
    # goal step small enough to be a refine bias (0.65×miss ≈ ≤55 mm in the
    # field); staged leg-2 re-targets jump farther and are skipped, so a
    # staged plan counts whole as one shot.
    # orig = the goal as first commanded (bias 0) — the object's hover point.
    # In real data the one-shot endpoint sits ≤2 mm from it (IK Accept gate),
    # while the final aim drifts away by the accumulated bias.
    orig = goal[:, -1]
    ideal = ref
    for s in steps:
        if np.linalg.norm(goal[:, s + 1] - goal[:, s]) < 0.08:
            ideal = ref.copy()
            ideal[:, s + 1:] = ref[:, s:s + 1]
            orig = goal[:, s]
            break

    fig = plt.figure(figsize=(13.2, 6.4))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.25, 1.0])
    fig.suptitle("Approach #%d — %s trajectory  |  final miss %.1f mm, "
                 "refine %d pass(es)"
                 % (idx, rec["cls"], rec["err_mm"], rec["refine"]),
                 fontsize=13, y=0.985, color=col)

    # --- 3-D path (질문 2)
    ax = fig.add_subplot(gs[:, 0], projection="3d")
    d3 = max(1, n // 2500)
    ax.plot(act[0, ::d3], act[1, ::d3], act[2, ::d3], "-", color=col,
            lw=1.6, label="actual  FK(q_act)")
    # user spec (07-30): the 3-D shows exactly two trajectories — the
    # one-shot plan and the actual path — plus the goal and start markers,
    # nothing else. The plan ends ON the original-goal circle (within the
    # 2 mm IK residual), so its endpoint needs no marker of its own; the
    # refine re-plans stay visible through the actual path itself and the
    # re-target verticals on the right.
    ax.plot(ideal[0, ::d3], ideal[1, ::d3], ideal[2, ::d3], "-",
            color="#4575b4", lw=1.1, label="one-shot plan")
    ax.scatter([orig[0]], [orig[1]], [orig[2]], marker="o", s=90,
               facecolors="none", edgecolors="k", linewidths=1.4,
               depthshade=False, label="original goal (hover pt)")
    ax.scatter([act[0, 0]], [act[1, 0]], [act[2, 0]], s=45, c="#1a9850",
               depthshade=False, label="start")
    ax.set_xlabel("X [m]", fontsize=8, labelpad=1)
    ax.set_ylabel("Y [m]", fontsize=8, labelpad=1)
    ax.set_zlabel("Z [m]", fontsize=8, labelpad=1)
    ax.tick_params(labelsize=7)
    ax.view_init(elev=22, azim=-55)
    ax.legend(loc="upper left", fontsize=7.5, framealpha=0.85)

    # --- displacement from start: the three series of the user's spec in
    # ONE plot — the vision-given IK input (flat line: it is a point, not a
    # path), the planned FK(q_ref) and the measured FK(q_act). |P(t) − P(0)|
    # collapses direction, so equal displacement does NOT mean "arrived" (a
    # lateral miss is invisible here) — the title's final miss stays the
    # true meter, and which-axis detail lives in the 3-D path and the CSV.
    axd = fig.add_subplot(gs[0, 1])
    p0 = act[:, :1]
    ds_act = np.linalg.norm(act - p0, axis=0) * 1e3
    ds_ref = np.linalg.norm(ref - p0, axis=0) * 1e3
    ds_orig = float(np.linalg.norm(orig - act[:, 0])) * 1e3
    axd.axhline(ds_orig, color="#111111", lw=1.0,
                label="IK input (object hover pt)")
    axd.plot(t, ds_ref, "--", color="#666666", lw=0.9,
             label="reference FK(q_ref)")
    axd.plot(t, ds_act, color=col, lw=1.4, label="actual FK(q_act)")
    for s in steps:
        axd.axvline(t[s], color="#bbbbbb", lw=0.7, ls=":")
    axd.set_ylabel("displacement from start [mm]", fontsize=8)
    axd.set_xlabel("t [s]   (dotted verticals = refine re-targets)",
                   fontsize=8)
    axd.tick_params(labelsize=7)
    axd.grid(alpha=0.15)
    axd.legend(loc="lower right", fontsize=6.5, framealpha=0.85)

    fig.text(0.995, 0.005,
             "RT-FK vs offline-FK max Δ %.3f mm%s" %
             (xchk_mm, "" if xchk_mm < 1.0 else "  ⚠ MODEL MISMATCH"),
             fontsize=7, ha="right",
             color="#888888" if xchk_mm < 1.0 else "#d73027")
    fig.savefig(out_png, dpi=140, bbox_inches="tight")
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

    # trajectory pass — needs the paired DataLog, which lands a moment after
    # the row does; unpaired rows are retried on the next tick (the watcher
    # signature includes the rt_log_results dir for exactly this reason)
    rtlog_dir = os.path.abspath(os.path.join(os.path.dirname(csv_path), "..",
                                             "rt_log_results"))
    if os.environ.get("INDY7_RTLOG_DIR"):
        rtlog_dir = os.environ["INDY7_RTLOG_DIR"]
    if os.path.isdir(rtlog_dir):
        fk_bin, urdf = tool_paths()
        pairs = pair_datalogs(rows, rtlog_dir)
        latest_traj = None
        for i, r in enumerate(rows, 1):
            if (i - 1) not in pairs:
                continue
            safe = "".join(ch if ch.isalnum() else "_" for ch in r["cls"])[:24]
            png = os.path.join(out_dir,
                               "approach_%03d_%s_traj.png" % (i, safe))
            latest_traj = png
            if not redo_all and os.path.exists(png):
                continue
            aug = ensure_fk(pairs[i - 1], fk_bin, urdf)
            if aug is None:
                continue
            arrs = load_traj(aug)
            if arrs is None:
                continue
            plot_traj(r, i, arrs, png)
            made += 1
            print("  saved %s" % png)
        if latest_traj and os.path.exists(latest_traj):
            shutil.copyfile(latest_traj,
                            os.path.join(out_dir, "approach_latest_traj.png"))
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
        # The paired DataLog lands ~1-2 s AFTER the row (the logger task has
        # to notice the falling edge and write the file). Folding that dir
        # into the signature retries the trajectory pass when it appears.
        rtlog = os.environ.get("INDY7_RTLOG_DIR") or os.path.abspath(
            os.path.join(os.path.dirname(csv_path), "..", "rt_log_results"))
        try:
            sig = (sig, os.stat(rtlog).st_mtime)
        except OSError:
            pass
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
