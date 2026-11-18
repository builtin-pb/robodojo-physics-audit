"""Forward kinematics and damped-least-squares IK for the ARX X5 arm, read from RoboDojo's X5A.urdf (our own code).

Frames: base_link at the robot root; returns link6 pose (the hand; fingers extend along link6 +x, close along link6 y).
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, q):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K


LIMITS = np.radians([[-180, 180], [0, 180], [0, 180], [-90, 90], [-90, 90], [-180, 180]])  # ARX_physics.usd


class X5:
    def __init__(self, urdf: Path):
        r = ET.parse(urdf).getroot()
        joints = {j.get("name"): j for j in r.findall("joint")}
        self.chain = []
        for name in ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]:
            j = joints[name]
            o = j.find("origin")
            xyz = np.array([float(v) for v in o.get("xyz").split()])
            rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
            axis = np.array([float(v) for v in j.find("axis").get("xyz").split()])
            self.chain.append((xyz, rpy_to_R(*rpy), axis))

    def fk(self, q):
        T = np.eye(4)
        for (xyz, R0, axis), qi in zip(self.chain, q):
            A = np.eye(4)
            A[:3, :3] = R0 @ axis_angle(axis, qi)
            A[:3, 3] = xyz
            T = T @ A
        return T

    def ik(self, p_target, R_target, q0, iters=300, lam=0.02, w_rot=0.3):
        q = np.array(q0, float)
        for _ in range(iters):
            T = self.fk(q)
            e_p = p_target - T[:3, 3]
            Re = R_target @ T[:3, :3].T
            e_r = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
            e = np.r_[e_p, w_rot * e_r]
            if np.linalg.norm(e_p) < 1e-5 and np.linalg.norm(e_r) < 1e-4:
                break
            J = np.zeros((6, 6))
            for i in range(6):
                dq = np.zeros(6)
                dq[i] = 1e-6
                T2 = self.fk(q + dq)
                Rd = T2[:3, :3] @ T[:3, :3].T
                J[:3, i] = (T2[:3, 3] - T[:3, 3]) / 1e-6
                J[3:, i] = w_rot * 0.5 * np.array([Rd[2, 1] - Rd[1, 2], Rd[0, 2] - Rd[2, 0], Rd[1, 0] - Rd[0, 1]]) / 1e-6
            q += J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(6), e)
            q = np.clip(q, LIMITS[:, 0], LIMITS[:, 1])
        T = self.fk(q)
        Re = R_target @ T[:3, :3].T
        rot_err = float(np.degrees(np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1))))
        return q, float(np.linalg.norm(p_target - T[:3, 3])), rot_err

    def ik_best(self, p_target, R_target, seeds=40, rng=0):
        g = np.random.default_rng(rng)
        best = None
        for k in range(seeds):
            q0 = g.uniform(LIMITS[:, 0], LIMITS[:, 1]) if k else np.array([0, 1.2, 1.2, 0, 0, 0])
            q, ep, er = self.ik(p_target, R_target, q0)
            score = ep + er / 1000
            if best is None or score < best[3]:
                best = (q, ep, er, score)
        return best[:3]


def down_grasp_R(close_axis_world=(0.0, 1.0, 0.0)):
    """link6 orientation in the base frame: fingers (+x) pointing down, closing along the given horizontal axis."""
    x = np.array([0.0, 0.0, -1.0])
    y = np.asarray(close_axis_world, float)
    y = y - x * (x @ y)
    y /= np.linalg.norm(y)
    return np.column_stack([x, y, np.cross(x, y)])


if __name__ == "__main__":
    import sys
    arm = X5(Path(sys.argv[1]))  # path to RoboDojo's Assets/Robots/x5/X5A.urdf
    print("fk(0) link6 position", np.round(arm.fk(np.zeros(6))[:3, 3], 4))
    R = down_grasp_R()
    for x in (0.25, 0.30, 0.35):
        for z in (0.16, 0.24, 0.31):
            q, ep, er = arm.ik_best(np.array([x, 0.0, z]), R)
            print([x, 0.0, z], "->", np.round(np.degrees(q), 1), f"pos err {ep*1000:.2f} mm, rot err {er:.2f} deg")
