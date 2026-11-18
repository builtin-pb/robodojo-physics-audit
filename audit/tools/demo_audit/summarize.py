"""Per-task summary of audit_parquet.py output. Usage: python3 -I summarize.py <parquet_audit.json>"""
import json, sys, collections
d = json.load(open(sys.argv[1])); R = d["per_episode"]; N = {int(k): v for k, v in d["names"].items()}
print("episodes", len(R), "frames", sum(r["len"] for r in R))
L = [r["len"] for r in R]; print("len min/med/max", min(L), sorted(L)[len(L)//2], max(L), "| >700:", sum(l > 700 for l in L), "| <50:", sum(l < 50 for l in L))
dig = collections.Counter(r["digest"] for r in R); print("duplicate digests", [(k, c) for k, c in dig.items() if c > 1][:10])
print("episodes with limit violation", sum(r["viol_frames"] > 0 for r in R), "speed>cap", sum(r["speed_frames"] > 0 for r in R), "jump>1rad", sum(r["jump_frames"] > 0 for r in R), "grip out", sum(r["grip_out"] > 0 for r in R))
print("frames: viol", sum(r["viol_frames"] for r in R), "speed", sum(r["speed_frames"] for r in R), "jump", sum(r["jump_frames"] for r in R))
jc = collections.Counter(); 
for r in R:
    for j in r["viol_joints"]: jc[j] += 1
print("episodes per violating joint", sorted(jc.items()))
print(f'{"task":4} {"n":>4} {"len med/max":>12} {">700":>5} {"viol%ep":>8} {"viol%fr":>8} {"spd%ep":>7} {"jump ep":>8}')
for t in sorted(N):
    rs = [r for r in R if r["task"] == t]
    if not rs: continue
    fr = sum(r["len"] for r in rs); Ls = sorted(r["len"] for r in rs)
    print(f'{t:4} {len(rs):4} {Ls[len(Ls)//2]:5}/{Ls[-1]:<6} {sum(l>700 for l in Ls):5} {100*sum(r["viol_frames"]>0 for r in rs)/len(rs):8.1f} {100*sum(r["viol_frames"] for r in rs)/fr:8.2f} {100*sum(r["speed_frames"]>0 for r in rs)/len(rs):7.1f} {sum(r["jump_frames"]>0 for r in rs):8}  {N[t][:50]}')
print("top jumps"); 
for r in sorted(R, key=lambda r: -r["jump_max"])[:8]: print(r["ep"], N[r["task"]][:40], round(r["jump_max"], 3), r["jump_at"], r["len"])
print("top speed ratio");
for r in sorted(R, key=lambda r: -r["speed_ratio_max"])[:8]: print(r["ep"], N[r["task"]][:40], round(r["speed_ratio_max"], 2), r["speed_frames"])
print("top violation depth")
for r in sorted(R, key=lambda r: -max([v[1] for v in r["viol_joints"].values()] or [0]))[:8]: print(r["ep"], N[r["task"]][:40], r["viol_joints"])
