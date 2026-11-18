"""V1 static count: put_bottles_into_dustbin layouts whose bottle instances declare a mass of 10 kg or more (the 22 kg slip)."""
import glob, json, os, sys
root = sys.argv[1]
def masses(o, path=""):
    if isinstance(o, dict):
        for k, v in o.items():
            if k == "mass" and isinstance(v, (int, float)):
                yield path, v
            else:
                yield from masses(v, path + "/" + str(k))
    elif isinstance(o, list):
        for i, v in enumerate(o):
            yield from masses(v, path + f"[{i}]")
tot, hit, vals, per_seed = 0, 0, {}, {}
for s in (0, 1, 2):
    fs = sorted(glob.glob(os.path.join(root, str(s), "put_bottles_into_dustbin_*.json")))
    n = 0
    for f in fs:
        d = json.load(open(f))
        ms = list(masses(d))
        tot += 1
        big = [m for p, m in ms if m >= 10]
        for p, m in ms:
            vals[m] = vals.get(m, 0) + 1
        if big:
            hit += 1; n += 1
    per_seed[s] = (n, len(fs))
print(json.dumps({"layouts": tot, "layouts_with_mass_ge_10kg": hit, "per_seed": per_seed, "mass_value_counts": {str(k): v for k, v in sorted(vals.items())}}))
