"""CPU unit tests for `audit/harness/fast_ik.py` (`--fast_ik`): bit-identity with `audit/probes/x5_ik.py`.

Run: `python3 -I audit/harness/test_fast_ik.py [-v]` (numpy only; also inside the Isaac Sim container with
`/isaac-sim/python.sh`, whose numpy and OpenBLAS the harness uses). Every comparison is on the raw bytes of the
results, not on values: two results that print alike but differ in the last bit fail.
"""
from __future__ import annotations

import importlib.util
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "../.."))
X5_PY = os.path.join(REPO, "audit/probes/x5_ik.py")
URDF = os.path.join(REPO, "Assets/Robots/x5/X5A.urdf")
sys.path.insert(0, HERE)
import fast_ik  # noqa: E402

# The six joints of RoboDojo's X5A.urdf (origin xyz, rpy, axis), so the test runs without the Hugging Face Assets.
X5A_JOINTS = [
    ((0, 0, 0.0605), (0, 0, 0), (0, 0, 1)),
    ((0.02, 0, 0.04), (0, 0, 0), (0, 1, 0)),
    ((-0.264, 0, 0), (3.1416, 0, 0), (0, 1, 0)),
    ((0.245, 0, -0.056), (0, 0, 0), (0, 1, 0)),
    ((0.06775, 0.0005, -0.0865), (0, 0, 0), (0, 0, 1)),
    ((0.02895, 0, 0.0865), (-3.1416, 0, 0), (1, 0, 0)),
]


def _load(name):
    spec = importlib.util.spec_from_file_location(name, X5_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _arm(mod):
    if os.path.exists(URDF):
        return mod.X5(URDF)
    arm = mod.X5.__new__(mod.X5)
    arm.chain = [(np.array(xyz, float), mod.rpy_to_R(*rpy), np.array(ax, float)) for xyz, rpy, ax in X5A_JOINTS]
    return arm


def _same(a, b):
    a, b = np.asarray(a), np.asarray(b)
    return a.dtype == b.dtype and a.shape == b.shape and a.tobytes() == b.tobytes()


class FastIkBitIdentity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ref = _load("x5_ik_reference")
        cls.fast = _load("x5_ik_fast")
        fast_ik.install(cls.fast)
        cls.arm_ref = _arm(cls.ref)
        cls.arm_fast = _arm(cls.fast)
        cls.rng = np.random.default_rng(20261114)

    def test_install_is_idempotent_and_leaves_reference_alone(self):
        self.assertIs(fast_ik.install(self.fast), self.fast._FAST_IK_ORIG)
        self.assertIsNot(self.ref.X5.ik, self.fast.X5.ik)
        self.assertFalse(getattr(self.ref, "_FAST_IK_INSTALLED", False))

    def test_axis_angle(self):
        axes = [link[2] for link in self.arm_ref.chain] + [self.rng.normal(size=3) for _ in range(20)]
        axes += [np.array(v, float) for v in ((-1, 0, 0), (0, -1, 0), (0, 0, -1), (0, 0, 2.0))] + [(0, 1, 0), [1, 0, 0]]
        for axis in axes:
            for q in list(self.rng.uniform(-4, 4, size=50)) + [0.0, -0.0, np.pi, -np.pi, 1e-6, -1e-6]:
                q = np.float64(q)
                self.assertTrue(_same(self.ref.axis_angle(axis, q), self.fast.axis_angle(axis, q)), (axis, q))

    def test_coordinate_axes_take_the_single_product_path(self):
        for axis in ((1, 0, 0), (0, -1, 0), (0, 0, 3.0)):
            self.assertIsNotNone(fast_ik._K_of(np.array(axis, float))[1])
        self.assertIsNone(fast_ik._K_of(np.array([0.6, 0.8, 0.0]))[1])

    def test_fk(self):
        L = self.ref.LIMITS
        qs = [self.rng.uniform(L[:, 0], L[:, 1]) for _ in range(500)]
        qs += [np.zeros(6), -np.zeros(6), L[:, 0].copy(), L[:, 1].copy(), np.array([0.0, 1.2, 1.2, 0.0, 0.0, 0.0])]
        for q in qs:
            self.assertTrue(_same(self.arm_ref.fk(q), self.arm_fast.fk(q)), q)

    def _check_ik(self, p, R, q0, iters):
        a = self.arm_ref.ik(p, R, q0, iters=iters)
        b = self.arm_fast.ik(p, R, q0, iters=iters)
        self.assertTrue(_same(a[0], b[0]), (p, q0))
        self.assertTrue(_same(np.float64(a[1]), np.float64(b[1])) and _same(np.float64(a[2]), np.float64(b[2])), (a[1:], b[1:]))

    def test_ik_reachable_targets_script_starts(self):
        """Targets from fk of random joint states, started from the script's own candidates (solve_ik)."""
        L = self.ref.LIMITS
        for _ in range(12):
            T = self.arm_ref.fk(self.rng.uniform(L[:, 0], L[:, 1]))
            p, R = T[:3, 3].copy(), T[:3, :3].copy()
            g = np.random.default_rng(0)
            starts = [np.array([0.0, 1.2, 1.2, 0.0, 0.0, 0.0]), np.zeros(6)] + [g.uniform(L[:, 0], L[:, 1]) for _ in range(4)]
            for q0 in starts:
                self._check_ik(p, R, q0, 320)

    def test_ik_unreachable_and_edge_starts(self):
        """Targets out of reach (runs all iterations, hits the joint limits) and starts with -0.0 entries."""
        L = self.ref.LIMITS
        R = self.ref.down_grasp_R()
        for p in ([1.5, 0.0, 0.2], [0.0, 0.0, -0.5], [0.3, 0.0, 0.24]):
            for q0 in (-np.zeros(6), np.array([-0.0, 1.2, -0.0, 0.0, -0.0, 0.0]), L[:, 1].copy()):
                self._check_ik(np.array(p), R, q0, 60)

    def test_ik_default_iters(self):
        L = self.ref.LIMITS
        T = self.arm_ref.fk(self.rng.uniform(L[:, 0], L[:, 1]))
        a = self.arm_ref.ik(T[:3, 3], T[:3, :3], np.zeros(6))
        b = self.arm_fast.ik(T[:3, 3], T[:3, :3], np.zeros(6))
        self.assertTrue(_same(a[0], b[0]))


if __name__ == "__main__":
    unittest.main()
