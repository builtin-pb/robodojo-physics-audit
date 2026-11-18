"""Distinct layout geometries per task (#59 lane 2): SHA-256 of each layout JSON's geometry part, colour labels aside.

Geometry part: every top-level section except `Rigid` verbatim, plus the `Rigid` objects as a sorted list of
(category, default_pos, default_ori, scale, physics) with `label` and `category_idx` dropped (the cube models
00004/00005/00006 differ only in colour; their colliders and masses are equal). Also reports whole-file SHA-256.
Usage: python3 -I layout_geometry_census.py <Eval_Layout/RoboDojo/arx_x5 dir> <out.json> <task> [<task> ...]
"""
import hashlib, json, sys
from pathlib import Path
from collections import defaultdict

root = Path(sys.argv[1])
out = {}
for task in sys.argv[3:]:
    files = sorted(root.glob(f"*/{task}_*.json"), key=lambda p: (int(p.parent.name), int(p.stem.rsplit('_', 1)[1])))
    geo, blob, labelled = defaultdict(list), defaultdict(list), defaultdict(list)
    for p in files:
        raw = p.read_bytes()
        d = json.loads(raw)
        objs, lab = [], []
        for cat, lst in d["Rigid"].items():
            for v in lst:
                objs.append([cat, [round(x, 6) for x in v["default_pos"]], [round(x, 6) for x in v["default_ori"]],
                             v.get("scale"), v.get("physics")])
                lab.append([v["label"], v.get("category_idx"), [round(x, 6) for x in v["default_pos"]]])
        objs.sort(key=json.dumps)
        rest = {k: d[k] for k in sorted(d) if k != "Rigid"}
        g = hashlib.sha256(json.dumps([rest, objs], sort_keys=True).encode()).hexdigest()
        key = f"{p.parent.name}/{p.stem.rsplit('_', 1)[1]}"
        geo[g].append(key)
        blob[hashlib.sha256(raw).hexdigest()].append(key)
        labelled[hashlib.sha256(json.dumps(sorted(lab, key=json.dumps)).encode()).hexdigest()].append(key)
    out[task] = {
        "files": len(files),
        "distinct_geometry_sha256": len(geo),
        "distinct_file_sha256": len(blob),
        "distinct_labelled_object_sets": len(labelled),
        "geometry_groups": {h: v for h, v in geo.items()},
        "file_groups": {h: v for h, v in blob.items()},
    }
    print(task, {k: out[task][k] for k in ("files", "distinct_geometry_sha256", "distinct_file_sha256", "distinct_labelled_object_sets")})
with open(sys.argv[2], "w") as f:
    json.dump(out, f, indent=1)
