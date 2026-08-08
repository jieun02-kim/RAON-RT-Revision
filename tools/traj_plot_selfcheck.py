#!/usr/bin/env python3
"""Render a synthetic approach through plot_traj — a semantics check for the
per-approach trajectory figure that needs no robot and no DataLog.

Why this exists: before the first real approach DataLog existed, the figure
was smoke-tested with a borrowed manual RECT log whose goal columns meant
nothing — every rendering question it raised was an artifact of that input,
not of the measurement (which records goal / q_ref / q at 1 kHz per
approach, exactly the three series the figure compares). This file replaces
that borrowed input with data that follows the real refine mechanics:

    pass-1 droop 37 mm  ->  0.65 re-aim  ->  final miss ~8 mm,
    IK residual <2 mm at every plan endpoint.

Expected reading of the output PNG:
  * 3-D: the blue one-shot plan ends ON the original-goal circle (its
    endpoint differs from the circle only by the <2 mm IK residual, so it
    carries no marker of its own); actual ends a few mm from the circle.
  * displacement: actual converges to the flat IK-input line after the
    re-target vertical.

Usage: python3 tools/traj_plot_selfcheck.py [out.png]
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import approach_plot as ap


def minjerk(n):
    tau = np.linspace(0.0, 1.0, n)
    return (10 * tau**3 - 15 * tau**4 + 6 * tau**5)[:, None]


P0 = np.array([0.45, -0.10, 0.35])           # start TCP
G = np.array([0.62, 0.12, 0.30])             # original goal (hover pt)
ikres = np.array([0.0012, -0.0006, 0.0008])  # IK Accept residual (<2 mm)
miss1 = np.array([-0.030, 0.018, 0.012])     # pass-1 droop (37 mm)

Gik1 = G + ikres                             # pass-1 plan endpoint
P1 = G + miss1                               # pass-1 reached
aim2 = G - 0.65 * miss1                      # re-aim: goal + bias
Gik2 = aim2 + ikres                          # pass-2 plan endpoint
P2 = G + 0.22 * miss1                        # final reached (~8 mm miss)

f = 100  # Hz is plenty for a rendering check
n_move, n_settle = 3 * f, int(1.5 * f)
mj = minjerk(n_move)
hold = np.ones((n_settle, 1))

ref = np.vstack([P0 + (Gik1 - P0) * mj, Gik1 * hold,
                 Gik1 + (Gik2 - Gik1) * mj, Gik2 * hold])
act = np.vstack([P0 + (P1 - P0) * mj, P1 * hold,
                 P1 + (P2 - P1) * mj, P2 * hold])
n = len(ref)
goal = np.vstack([np.tile(G, (n_move + n_settle, 1)),
                  np.tile(aim2, (n - n_move - n_settle, 1))])

a = {"t": np.arange(n) / f}
for i, k in enumerate("xyz"):
    a["goal_" + k] = goal[:, i]
    a["ref_" + k] = ref[:, i]
    a["tcp_" + k] = act[:, i]
    a["act_fk_" + k] = act[:, i]

rec = {"cls": "selfcheck", "err_mm": float(np.linalg.norm(P2 - G) * 1e3),
       "refine": 2}
out = sys.argv[1] if len(sys.argv) > 1 else "traj_selfcheck.png"
ap.plot_traj(rec, 0, a, out)
print("saved %s" % out)
print("expect: plan endpoint %.1f mm from the circle (overlapping), "
      "final miss %.1f mm"
      % (np.linalg.norm(Gik1 - G) * 1e3, rec["err_mm"]))
