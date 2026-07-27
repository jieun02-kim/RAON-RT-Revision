#!/usr/bin/env python3
"""E21 base-shift fit: recover the planar rigid motion (yaw + tx + ty) of the
robot base from hand-probe pairs, and compose it into the launch static TF.

Background (merge.md E21, 2026-07-27): the arm incidents shoved the robot
base on the optical table, so every camera-reported base_link coordinate is
systematically displaced. The camera and objects did NOT move — the base
frame did. A planar rigid correction C (yaw about z + 2D translation)
satisfies  p_probe = C(p_locked)  for every object, provided:
  * every (LOCKED, probe) pair is captured with NOTHING moving in between,
  * probes touch the object top near-vertically (tool-tip XY == TCP XY
    within ~3 mm; the 29 mm tip offset then lands almost entirely in z).

Sanity invariant checked first: rigid motion preserves inter-object
distances. If camera-frame and probe-frame distances disagree by more than
RESID_WARN_M the pairs are NOT simultaneous (an object moved) — refuse.

New TF: with T_old = base_link->camera_link (launch),  p_base = T_old(p_cam)
was calibrated for the OLD base pose. After the base moved, the true
coordinate in the NEW base frame is C(p_base), so
    R_new = Rz(theta) @ R_old        t_new = Rz(theta) @ t_old + [tx, ty, 0]
which this script prints in launch-file order (x y z qx qy qz qw).

Usage: edit PAIRS below, then  python3 base_shift_fit.py
"""
import math

import numpy as np

# --------------------------------------------------------------------------
# INPUT — one entry per object:  ((LOCKED x, y, z), (probe TCP x, y, z))
# Capture procedure per object (nothing may move in between):
#   'p' -> digit -> copy "TARGET LOCKED: <class> @ base(x, y, z)"
#   'g' grav-comp -> touch the object top near-vertically -> 's' -> copy t=[..]
PAIRS = [
    # (( 0.681, -0.235, 0.202), (0.7444, -0.1380, 0.2391)),   # example: apple
]

# Current launch TF (pick_place_vitis_ai.launch.py, calibrated 2026-07-27)
T_OLD = np.array([0.761822, -0.084713, 0.924974])
Q_OLD = np.array([0.72390457, -0.03237801, -0.68876782, 0.02264346])  # x y z w

RESID_WARN_M = 0.02   # pairwise-distance / per-point residual warning level


def quat_to_R(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, \
            (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, \
                (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, \
                0.25 * s, (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, \
                (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def main():
    if len(PAIRS) < 2:
        raise SystemExit("Need >= 2 pairs (3 recommended) — edit PAIRS.")

    cam = np.array([p[0] for p in PAIRS], dtype=float)
    rob = np.array([p[1] for p in PAIRS], dtype=float)

    # 1) rigidity sanity: inter-object distances must match between frames
    print("== pairwise-distance check (rigid ⇒ equal) ==")
    bad = False
    for i in range(len(PAIRS)):
        for j in range(i + 1, len(PAIRS)):
            dc = np.linalg.norm(cam[i, :2] - cam[j, :2])
            dr = np.linalg.norm(rob[i, :2] - rob[j, :2])
            flag = "" if abs(dc - dr) < RESID_WARN_M else "  <-- NOT rigid!"
            if flag:
                bad = True
            print(f"  {i}-{j}: cam {dc * 100:6.1f} cm | probe {dr * 100:6.1f}"
                  f" cm | diff {(dr - dc) * 100:+5.1f} cm{flag}")
    if bad:
        print("\nAn object moved between its LOCK and probe — recapture the "
              "flagged pairs simultaneously.")
        raise SystemExit(1)

    # 2) 2D Kabsch: rob_xy = Rz(theta) @ cam_xy + t
    cc, rc = cam[:, :2].mean(0), rob[:, :2].mean(0)
    A, B = cam[:, :2] - cc, rob[:, :2] - rc
    th = math.atan2((A[:, 0] * B[:, 1] - A[:, 1] * B[:, 0]).sum(),
                    (A * B).sum())
    c, s = math.cos(th), math.sin(th)
    R2 = np.array([[c, -s], [s, c]])
    t2 = rc - R2 @ cc

    print(f"\n== fit ==\n  yaw    = {math.degrees(th):+.3f} deg"
          f"\n  t      = ({t2[0] * 100:+.2f}, {t2[1] * 100:+.2f}) cm")
    print("  per-point XY residual / z delta (z: probe reads top + ~3 cm tip):")
    worst = 0.0
    for i in range(len(PAIRS)):
        pred = R2 @ cam[i, :2] + t2
        r = np.linalg.norm(pred - rob[i, :2])
        worst = max(worst, r)
        print(f"    #{i}: {r * 1000:6.1f} mm | dz {(rob[i, 2] - cam[i, 2]) * 1000:+6.1f} mm")
    if worst > RESID_WARN_M:
        print(f"  WARNING: worst residual {worst * 1000:.0f} mm > "
              f"{RESID_WARN_M * 1000:.0f} mm — motion may not be planar-rigid "
              f"(camera moved too?). Consider the full 16-pose recalib.")

    # 3) compose into the launch TF
    Rz3 = np.eye(3)
    Rz3[:2, :2] = R2
    R_new = Rz3 @ quat_to_R(Q_OLD)
    t_new = Rz3 @ T_OLD + np.array([t2[0], t2[1], 0.0])
    q_new = R_to_quat(R_new)
    print("\n== launch TF (base_link -> camera_link), drop-in ==")
    print(f"  x  {t_new[0]:.6f}  y {t_new[1]:.6f}  z {t_new[2]:.6f}")
    print(f"  qx {q_new[0]:.8f}  qy {q_new[1]:.8f}  qz {q_new[2]:.8f}"
          f"  qw {q_new[3]:.8f}")


if __name__ == "__main__":
    main()
