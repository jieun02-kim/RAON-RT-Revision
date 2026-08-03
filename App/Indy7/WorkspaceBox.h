/*****************************************************************************
*   Name: WorkspaceBox.h
*   Affiliation: RAIMLAB @ Myongji University
*   Description: [kv260-merge] The measured workspace box, ROS-free.
*
*   These two constants lived inside ROS2PickBridge.h, which drags in rclcpp
*   and my_interfaces. They are not a ROS concept — the app derives the
*   ready-seed IK probes from the same numbers, and the offline reach-map tool
*   needs them without any of ROS. Kept here so there is exactly one source of
*   truth; CROS2PickBridge::DEF_BOX / DEF_RMAX_XY still name these values.
*****************************************************************************/
#ifndef __KV260_WORKSPACE_BOX__
#define __KV260_WORKSPACE_BOX__

namespace kv260 {

/* Workspace box for the GOAL (z margin already added). Drawn from the
 * 2026-07-27 hand-eye calibration; E21 re-base (same evening): the robot
 * base was shoved (yaw +5.14 deg, t (+4.5, +10.4) cm), so the SAME
 * physical table region maps to the ranges below (old box corners run
 * through the fitted transform, AABB, rounded outward). x_max no longer
 * encodes "reach" — hover reachability is decided per-goal by the
 * deterministic ready-seed IK (RT refuses infeasible goals cleanly, no
 * motion); the box only rejects absurd goals (behind the robot / z band;
 * z_max 0.50 keeps clearance below the camera). */
static constexpr double WS_BOX[6] = {0.30, 0.94, -0.37, 0.63, 0.10, 0.50};

/* Coarse sanity net only: the old 0.80 was an empirical guess made in the
 * pre-shift frame and blocked objects the arm demonstrably reaches
 * (2026-07-27 field) — genuine reach is the IK solver's verdict. */
static constexpr double WS_RMAX_XY = 0.92;   // [m] from base axis

/* Tracking ('o') constants shared by the bridge (sample admission) and the
 * app (goal construction) — one source of truth, ROS-free. */

/* Hover margin while TRACKING a hand-held object. Larger than the 0.15
 * approach margin: at 0.3 m/s object speed the EMA + pipeline lag leaves
 * the commanded z ~7 cm behind a rising object, eating into the TCP-object
 * gap exactly when a hand is under the tool. */
static constexpr double TRACK_ZMARGIN_M = 0.20;

} // namespace kv260

#endif // __KV260_WORKSPACE_BOX__
