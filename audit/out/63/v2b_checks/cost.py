"""GPU cost of the v2b checks: union of process intervals on della-vis1's A100 (busy time) and the process-hours sum.

usage: python3 -I cost.py <times.tsv> <batch_summary.json> [...]   (prints JSON)
times.tsv rows: tag t0 t1 rc (run_one.sh); batch summaries: procs[].{tag,t_start,t_end,rc}.
"""
import json
import sys

rows = []
for p in sys.argv[1:]:
    if p.endswith(".tsv"):
        for line in open(p):
            t, a, b, rc = line.split()
            rows.append({"tag": t, "t0": float(a), "t1": float(b), "rc": int(rc)})
    else:
        d = json.load(open(p))
        for pr in d["procs"]:
            rows.append({"tag": pr["tag"], "t0": float(pr["t_start"]), "t1": float(pr["t_end"]), "rc": pr.get("rc")})
rows.sort(key=lambda r: r["t0"])
busy, cur = 0.0, None
for r in rows:
    if cur is None or r["t0"] > cur[1]:
        if cur is not None:
            busy += cur[1] - cur[0]
        cur = [r["t0"], r["t1"]]
    else:
        cur[1] = max(cur[1], r["t1"])
if cur is not None:
    busy += cur[1] - cur[0]
proc = sum(r["t1"] - r["t0"] for r in rows)
print(json.dumps({"processes": [dict(r, wall_s=round(r["t1"] - r["t0"], 1)) for r in rows],
                  "gpu_busy_s": round(busy, 1), "gpu_busy_h": round(busy / 3600, 3),
                  "process_s": round(proc, 1), "process_h": round(proc / 3600, 3)}, indent=1))
