"""Exact fast path for `audit/probes/x5_ik.py` (`--fast_ik`; harness speed, `audit/out/speed/`).

The bottle script spends about three quarters of an episode in `X5.ik` (16 starts x up to 320 damped-least-squares
iterations, each with 7 forward-kinematics calls). This module computes the same floating-point operations on the same
inputs, so every result is bit-identical to the original, and skips only work whose result is already known:

- `axis_angle(axis, q)`: the unit axis and its cross-product matrix K depend only on `axis`; they are computed once per
  axis value with the original expressions and reused. For a general axis the rotation is then formed with the original
  expression `np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K`. For a coordinate axis (the X5's six joints) every
  entry of K is 0 or +-1 and each row has at most one nonzero, so each entry of `((1 - cos q) * K) @ K` is a single
  exact product +-(1 - cos q) or a zero, and it equals `(1 - cos q) * (K @ K)` with `K @ K` computed once; a zero of
  either sign added to `eye + sin(q) K`, whose zeros are +0, changes nothing. The fast path uses that form.
- `X5.ik`: the finite-difference Jacobian perturbs one joint at a time, so `fk(q + dq_i)` shares its first i joint
  transforms and running products with `fk(q)`. They are reused when the perturbed joint values are bit-identical
  (checked on the int64 view, so a -0.0 that `q + 0.0` turns into +0.0 is recomputed); every other product is formed
  with the same matmul, in the same order, as `fk`. Constant arrays (`np.eye`, the joint transform's fixed entries,
  `lam**2 * np.eye(6)`) are built once with the original expressions and copied.

Nothing in `x5_ik.py` or the scripts changes on disk; `install()` replaces the module's `axis_angle` and the class's
`fk` and `ik` in memory before a script imports them. `audit/harness/test_fast_ik.py` checks bit-identity against the
original on random problems; the episode-level checks are in `audit/out/speed/README.md`.
"""
import numpy as np

_K_CACHE = {}
_K_BY_ID = {}
_I3 = np.eye(3)
_I3.setflags(write=False)


def _orig_K(axis):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])


def _single_product_rows(K):
    """True when every entry is 0 or +-1 and each row has at most one nonzero (K of a coordinate axis)."""
    vals = np.abs(K)
    return bool(np.all((vals == 0.0) | (vals == 1.0)) and np.all((vals != 0.0).sum(axis=1) <= 1))


def _K_of(axis):
    """(K, K @ K or None) for this axis value; looked up by object identity first (the chain's own arrays)."""
    hit = _K_BY_ID.get(id(axis))
    if hit is not None and hit[0] is axis:
        return hit[1]
    arr = np.asarray(axis, float)
    key = arr.tobytes() + str(arr.shape).encode()
    entry = _K_CACHE.get(key)
    if entry is None:
        K = _orig_K(axis)
        K.setflags(write=False)
        KK = None
        if _single_product_rows(K):
            KK = K @ K
            KK.setflags(write=False)
        entry = (K, KK)
        _K_CACHE[key] = entry
    if isinstance(axis, np.ndarray):
        _K_BY_ID[id(axis)] = (axis, entry)  # keeps a reference, so the id cannot be reused
    return entry


def axis_angle(axis, q):
    K, KK = _K_of(axis)
    if KK is None:
        return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K
    return _I3 + np.sin(q) * K + (1 - np.cos(q)) * KK


_A_TEMPLATE = {}


def _joint_A(link, qi):
    xyz, R0, axis = link
    tmpl = _A_TEMPLATE.get(id(xyz))
    if tmpl is None or tmpl[0] is not xyz:
        T0 = np.eye(4)
        T0[:3, 3] = xyz
        tmpl = (xyz, T0)
        _A_TEMPLATE[id(xyz)] = tmpl
    A = tmpl[1].copy()
    A[:3, :3] = R0 @ axis_angle(axis, qi)
    return A


def fk(self, q):
    T = np.eye(4)
    for link, qi in zip(self.chain, q):
        T = T @ _joint_A(link, qi)
    return T


def _fk_parts(chain, q):
    """Joint transforms and running products of `fk(q)`: prefix[j] is the product of the first j transforms."""
    As = []
    prefix = [np.eye(4)]
    T = prefix[0]
    for link, qi in zip(chain, q):
        A = _joint_A(link, qi)
        As.append(A)
        T = T @ A
        prefix.append(T)
    return As, prefix


def make_ik(LIMITS):
    def ik(self, p_target, R_target, q0, iters=300, lam=0.02, w_rot=0.3):
        chain = self.chain
        n = len(chain)
        q = np.array(q0, float)
        lam2I = lam**2 * np.eye(6)
        for _ in range(iters):
            As, prefix = _fk_parts(chain, q)
            T = prefix[n]
            e_p = p_target - T[:3, 3]
            Re = R_target @ T[:3, :3].T
            e_r = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
            e = np.r_[e_p, w_rot * e_r]
            if np.linalg.norm(e_p) < 1e-5 and np.linalg.norm(e_r) < 1e-4:
                break
            J = np.zeros((6, 6))
            qbits = q.view(np.int64)
            for i in range(6):
                dq = np.zeros(6)
                dq[i] = 1e-6
                qd = q + dq
                same = qd.view(np.int64) == qbits
                m = 0
                while m < n and same[m]:
                    m += 1
                T2 = prefix[m]
                for j in range(m, n):
                    T2 = T2 @ (As[j] if same[j] else _joint_A(chain[j], qd[j]))
                Rd = T2[:3, :3] @ T[:3, :3].T
                J[:3, i] = (T2[:3, 3] - T[:3, 3]) / 1e-6
                J[3:, i] = w_rot * 0.5 * np.array([Rd[2, 1] - Rd[1, 2], Rd[0, 2] - Rd[2, 0], Rd[1, 0] - Rd[0, 1]]) / 1e-6
            q += J.T @ np.linalg.solve(J @ J.T + lam2I, e)
            q = np.clip(q, LIMITS[:, 0], LIMITS[:, 1])
        T = self.fk(q)
        Re = R_target @ T[:3, :3].T
        rot_err = float(np.degrees(np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1))))
        return q, float(np.linalg.norm(p_target - T[:3, 3])), rot_err

    return ik


def install(x5_module):
    """Patch an imported `x5_ik` module in memory; returns the originals (for tests and the manifest)."""
    if getattr(x5_module, "_FAST_IK_INSTALLED", False):
        return x5_module._FAST_IK_ORIG
    orig = {"axis_angle": x5_module.axis_angle, "fk": x5_module.X5.fk, "ik": x5_module.X5.ik}
    x5_module.axis_angle = axis_angle
    x5_module.X5.fk = fk
    x5_module.X5.ik = make_ik(x5_module.LIMITS)
    x5_module._FAST_IK_INSTALLED = True
    x5_module._FAST_IK_ORIG = orig
    return orig
