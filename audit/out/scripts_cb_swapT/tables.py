"""Markdown tables from the per-episode summaries written by summarize.py (read-only).

Usage: python3 -I tables.py <out dir>
cover_blocks: the 7 run layouts are expanded to all 20 development episodes by identical layout-file SHA-256
(layout_geometry_census.json); every inferred row says which run it copies.
"""
import gzip
import json
import sys
from pathlib import Path

out = Path(sys.argv[1])
eps = {}
for p in sorted((out / "episodes").glob("*.json.gz")):
    e = json.load(gzip.open(p, "rt"))
    if e["tag"].startswith(("scr_sT2_v", "scr_cb_v")):
        eps[(e["task"], e["evaluator"], e["seed"], e["layout"])] = e
census = json.load(open(out / "layout_geometry_census.json"))
file_of = {k: h for h, ks in census["cover_blocks"]["file_groups"].items() for k in ks}


def same_traj(a, b):
    la, lb = a["action_log"], b["action_log"]
    if len(la) != len(lb):
        return False
    return all(x["obj"] == y["obj"] and x["q7"] == y["q7"] for x, y in zip(la, lb))


for task in ("swap_T", "cover_blocks"):
    print(f"\n### {task}\n")
    if task == "swap_T":
        print("| seed | layout | v1 score | v1 success | v2 score | v2 success | actions (v1/v2) | worst final xy mm | worst final rot deg | held t0 rot at hover deg | v1 = v2 trajectory | failure cause |")
        print("|---|---|---|---|---|---|---|---|---|---|---|---|")
    else:
        print("| seed | layout | colour order (left to right) | run or copy of | v1 score | v1 success | v2 score | v2 success | actions (v1/v2) | max cube move mm | v1 = v2 trajectory | failure cause |")
        print("|---|---|---|---|---|---|---|---|---|---|---|---|")
    tot = {"v1": [0, 0.0], "v2": [0, 0.0]}
    for s in (0, 1):
        for l in range(10):
            src = (s, l)
            if task == "cover_blocks" and (task, "v1", s, l) not in eps:
                src = next((int(k.split("/")[0]), int(k.split("/")[1])) for k, h in file_of.items()
                           if h == file_of[f"{s}/{l}"] and (task, "v1", int(k.split("/")[0]), int(k.split("/")[1])) in eps)
            a, b = eps.get((task, "v1") + src), eps.get((task, "v2") + src)
            if a is None or b is None:
                print(f"| {s} | {l} | missing |")
                continue
            for ev, e in (("v1", a), ("v2", b)):
                tot[ev][0] += bool(e["success"])
                tot[ev][1] += float(e["score"])
            cause = (a["summary"] or {}).get("failure_cause") or (b["summary"] or {}).get("failure_cause") or "-"
            st = "yes" if same_traj(a, b) else "no"
            if task == "swap_T":
                fin = [a["summary"]["final"][t] for t in ("t0", "t1")]
                hov = next(c["t0"]["rot_deg"] for c in a["checkpoints"] if c["after"] == "hover" and c["T"] == "t0")
                print(f"| {s} | {l} | {a['score']} | {a['success']} | {b['score']} | {b['success']} | {a['take_action_cnt']}/{b['take_action_cnt']} | "
                      f"{max(f['xy_mm'] for f in fin)} | {max(f['rot_deg'] for f in fin)} | {hov} | {st} | {cause} |")
            else:
                ref = "run" if src == (s, l) else f"copy of s{src[0]} l{src[1]}"
                order = "".join(x[0].upper() for x in a["order_left_mid_right"])
                mx = max(a["summary"]["max_cube_disp_mm"].values())
                print(f"| {s} | {l} | {order} | {ref} | {a['score']} | {a['success']} | {b['score']} | {b['success']} | "
                      f"{a['take_action_cnt']}/{b['take_action_cnt']} | {mx} | {st} | {cause} |")
    for ev in ("v1", "v2"):
        print(f"\n{task} {ev}: {tot[ev][0]} / 20 success, mean score {tot[ev][1] / 20:.2f}")
