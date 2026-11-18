"""How high can the bottle-hold witness lift with the hand pointing straight down? (blog clip, three finger versions)

usage: python3 -I ik_reach.py <repo root> <Assets/Robots/x5/X5A.urdf>
Uses the witness's own IK (audit/probes/x5_ik.py: X5, down_grasp_R, its joint limits) on its own waypoints
(audit/harness/p32_gripper.py run_bottle_hold_trial): grasp 6 cm off-centre toward the base end, link6 0.1295 m above the
lying bottle's origin, closing across the bottle's axis, robot1's root from env_cfg/robot/dual_x5.yml. The bottle pose is
bottle2's settled pose in audit/out/d3/round2/d3r4f_w4 (runs[0].settled_poses). Prints the IK error per lift height,
straight up and with the hand moved along the radial direction from robot1's base.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(sys.argv[1], "audit/probes"))
from x5_ik import LIMITS, X5, down_grasp_R  # noqa: E402


def quat_to_mat(q):  # w, x, y, z (p32_gripper.quat_to_mat)
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


arm = X5(sys.argv[2])
bottle = [0.37056, -0.14477, 0.7963, 0.70818, 0.06203, 0.69881, -0.07935]  # bottle2, x y z qw qx qy qz
root, Rb = np.array([0.3, -0.45, 0.765]), quat_to_mat([0.707, 0, 0, 0.707])  # robot1 (grasp x > 0)
axis = quat_to_mat(bottle[3:])[:, 2]
axis_h = np.array([axis[0], axis[1], 0.0]) / np.linalg.norm(axis[:2])
grasp = np.array([*(np.array(bottle[:2]) - 0.06 * axis_h[:2]), bottle[2] + 0.1295])
R_grasp = down_grasp_R(close_axis_world=Rb.T @ np.array([-axis_h[1], axis_h[0], 0.0]))
print(f"joint limits used by the IK (deg): {np.degrees(LIMITS).round(1).tolist()}")
print(f"grasp target (link6, world) {grasp.round(4).tolist()}, bottle axis {axis.round(4).tolist()}")


def solve(p):
    q, ep, er = arm.ik_best(Rb.T @ (p - root), R_grasp, seeds=40)
    return q, ep * 1000.0, er


print("straight up: lift (m), position error (mm), orientation error (deg), joint 4 (deg)")
for h in np.round(np.arange(0.10, 0.2001, 0.01), 3):
    q, ep, er = solve(grasp + [0, 0, h])
    print(f"  {h:.2f}  {ep:7.2f}  {er:6.2f}  {np.degrees(q[3]):7.2f}")
rad = np.r_[(grasp - root)[:2] / np.linalg.norm((grasp - root)[:2]), 0.0]
print(f"moved along the radial direction {rad.round(3).tolist()} (dr > 0 away from the base): errors mm/deg")
for dr in (-0.10, -0.05, 0.05, 0.10):
    cells = []
    for h in (0.20, 0.25, 0.30):
        _, ep, er = solve(grasp + dr * rad + [0, 0, h])
        cells.append(f"lift {h:.2f}: {ep:5.1f}/{er:4.1f}")
    print(f"  dr {dr:+.2f}  " + "   ".join(cells))
