"""Speed and bit-identity of audit/harness/fast_ik.py against audit/probes/x5_ik.py (audit/out/speed/).

usage (inside the Isaac Sim container): /isaac-sim/python.sh bench_ik.py <repo with the new fast_ik> <repo with an older one>
"""
import sys, time, importlib.util, numpy as np
R_new, R_old = sys.argv[1], sys.argv[2]
def load(n, R, fast_dir=None):
    s = importlib.util.spec_from_file_location(n, R + "/audit/probes/x5_ik.py"); m = importlib.util.module_from_spec(s); s.loader.exec_module(m)
    if fast_dir:
        fs = importlib.util.spec_from_file_location(n + "_f", fast_dir + "/audit/harness/fast_ik.py"); f = importlib.util.module_from_spec(fs); fs.loader.exec_module(f); f.install(m)
    return m
mods = {"orig": load("a", R_new), "fast_old": load("b", R_old, R_old), "fast_new": load("c", R_new, R_new)}
rng = np.random.default_rng(1)
arms = {k: m.X5(R_new + "/Assets/Robots/x5/X5A.urdf") for k, m in mods.items()}
L = mods["orig"].LIMITS
probs = []
for _ in range(4):
    T = arms["orig"].fk(rng.uniform(L[:, 0], L[:, 1])); g = np.random.default_rng(0)
    probs.append((T[:3, 3].copy(), T[:3, :3].copy(), [np.array([0., 1.2, 1.2, 0, 0, 0]), np.zeros(6)] + [g.uniform(L[:, 0], L[:, 1]) for _ in range(14)]))
outs = {}
for rep in range(2):
    for name, arm in arms.items():
        t = time.perf_counter(); out = []
        for p, Rt, starts in probs:
            for q0 in starts: out.append(arm.ik(p, Rt, q0, iters=320))
        print(rep, name, round(time.perf_counter() - t, 2), "s", flush=True); outs[name] = out
for name in ("fast_old", "fast_new"):
    print(name, "bit-identical to orig:", all(a[0].tobytes() == b[0].tobytes() and a[1] == b[1] and a[2] == b[2] for a, b in zip(outs["orig"], outs[name])))
