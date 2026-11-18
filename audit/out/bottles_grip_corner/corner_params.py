"""Bottles G1 (#160, #200): copy docs/params with the corrected gripper at the adverse corner of its plausible ranges.

usage: python3 -I corner_params.py <docs/params dir> <out dir>

Transmission efficiency 0.7 moves finger_trip_n, finger_stiffness_n_per_m and finger_damping_n_s_per_m to the low ends
of their ranges together; finger_armature_kg goes to the high end of its range. Only those four leaves' `value` changes;
every other byte of the tables present is copied (`from_path` skips missing ones). Prints the directory-mode table SHA-256 the way
`PhysicsParamsTable.from_path` computes it, which each harness result records as `table_sha256`.
"""
from __future__ import annotations

import hashlib
import json
import os
import sys

FILES = ("classes.json", "class_values.json", "pairs.json", "assets.json", "robots.json")
CORNER = {  # leaf: (range end index, value expected there)
    "finger_trip_n": (0, 141.55),
    "finger_stiffness_n_per_m": (0, 22870.1),
    "finger_damping_n_s_per_m": (0, 914.81),
    "finger_armature_kg": (1, 14.062),
}


def main(src, out):
    os.makedirs(out, exist_ok=True)
    h = hashlib.sha256()
    for fname in FILES:
        if not os.path.isfile(os.path.join(src, fname)):
            continue
        with open(os.path.join(src, fname), "rb") as f:
            raw = f.read()
        if fname == "robots.json":
            tab = json.loads(raw)
            g = tab["robots"]["x5"]["gripper"]
            for leaf, (end, want) in CORNER.items():
                v = g[leaf]["range"][end]
                if v != want:
                    raise SystemExit(f"{leaf}: range end {v} != expected {want}")
                print(f"{leaf}: {g[leaf]['value']} -> {v}")
                g[leaf]["value"] = v
            raw = (json.dumps(tab, indent=2) + "\n").encode("utf-8")
        with open(os.path.join(out, fname), "wb") as f:
            f.write(raw)
        h.update(fname.encode("utf-8") + b"\0" + raw + b"\0")
    print("table_sha256", h.hexdigest())


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
