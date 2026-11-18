"""Does numpy's OpenBLAS thread setting change the bits of small linear algebra? (audit/out/speed/)

usage (inside the Isaac Sim container): OPENBLAS_NUM_THREADS=<n> [OPENBLAS_THREAD_TIMEOUT=<t>] /isaac-sim/python.sh
       blas_determinism.py <repo> <label>   (label 'setN' calls openblas_set_num_threads64_(N) at runtime instead)
Prints SHA-256 prefixes of: 96 X5 IK solves (16 starts as the bottle script uses them), 3x3 and 4x4 matmuls, J @ J.T,
np.linalg.solve on 6x6 systems, 3-vector norms and 6x6 matrix-vector products, all on fixed random inputs.
"""
import sys, os, importlib.util, hashlib, numpy as np
R = sys.argv[1]; mode = sys.argv[2]
if mode.startswith("set"):
    import ctypes
    for line in open("/proc/self/maps"):
        p = line.split()[-1]
        if "openblas" in os.path.basename(p):
            lib = ctypes.CDLL(p); lib.openblas_set_num_threads64_(ctypes.c_int(int(mode[3:]))); print("set", mode[3:], lib.openblas_get_num_threads64_()); break
s = importlib.util.spec_from_file_location("x", R + "/audit/probes/x5_ik.py"); m = importlib.util.module_from_spec(s); s.loader.exec_module(m)
arm = m.X5(R + "/Assets/Robots/x5/X5A.urdf")
rng = np.random.default_rng(1); L = m.LIMITS
h = hashlib.sha256()
ops = {"mm33": hashlib.sha256(), "mm44": hashlib.sha256(), "mm66T": hashlib.sha256(), "solve6": hashlib.sha256(), "norm": hashlib.sha256(), "mv6": hashlib.sha256()}
for _ in range(300):
    A3, B3 = rng.normal(size=(3, 3)), rng.normal(size=(3, 3)); ops["mm33"].update((A3 @ B3).tobytes()); ops["mm33"].update((A3 @ B3.T).tobytes())
    A4, B4 = rng.normal(size=(4, 4)), rng.normal(size=(4, 4)); ops["mm44"].update((A4 @ B4).tobytes())
    J = rng.normal(size=(6, 6)); ops["mm66T"].update((J @ J.T).tobytes())
    e = rng.normal(size=6); ops["solve6"].update(np.linalg.solve(J @ J.T + 0.0004 * np.eye(6), e).tobytes())
    ops["norm"].update(np.float64(np.linalg.norm(e[:3])).tobytes()); ops["mv6"].update((J.T @ e).tobytes())
for _ in range(4):
    T = arm.fk(rng.uniform(L[:, 0], L[:, 1])); g = np.random.default_rng(0)
    for q0 in [np.array([0., 1.2, 1.2, 0, 0, 0]), np.zeros(6)] + [g.uniform(L[:, 0], L[:, 1]) for _ in range(4)]:
        q, ep, er = arm.ik(T[:3, 3].copy(), T[:3, :3].copy(), q0, iters=320); h.update(q.tobytes())
print(mode, "ik", h.hexdigest()[:16], " ".join(f"{k}={v.hexdigest()[:10]}" for k, v in ops.items()))
