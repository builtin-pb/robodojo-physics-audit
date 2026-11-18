"""List what the LeRobot metadata holds (episode table columns, v2.1 episodes.jsonl keys). Usage: python -I inspect_meta.py <hf_root>"""
import json, sys
import pyarrow.parquet as pq
root = sys.argv[1]
t = pq.read_table(f"{root}/data/RoboDojo_lerobot_v30_video/meta/episodes/chunk-000/file-000.parquet")
print("v3.0 episodes columns:", [c for c in t.column_names if not c.startswith("stats/")], "rows", t.num_rows)
print("stats cols:", len([c for c in t.column_names if c.startswith("stats/")]))
d = t.slice(0, 2).to_pylist()
for r in d: print({k: v for k, v in r.items() if not k.startswith("stats/")})
with open(f"{root}/data/RoboDojo_lerobot_v21_video/meta/episodes.jsonl") as f:
    L = [json.loads(l) for l in f]
print("v2.1 episodes.jsonl rows", len(L), "keys", list(L[0].keys()), L[0], L[-1])
print(open(f"{root}/data/RoboDojo_lerobot_v21_video/meta/tasks.jsonl").read()[:600])
