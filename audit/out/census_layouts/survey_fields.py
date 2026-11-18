"""Survey the structure of all eval layout files: top-level keys, sections, object field names and types.

Usage: python3 -I survey_fields.py <manifest.json>
Prints, over all files: top-level keys with counts; for each top-level value type; for dict-of-list sections
(category -> [objects]) the object field names with counts and example values.
"""
import json, sys
from collections import Counter, defaultdict

man = json.load(open(sys.argv[1]))
top = Counter()
top_types = defaultdict(Counter)
obj_fields = defaultdict(Counter)
examples = {}
tasks_with = defaultdict(set)
for rel, v in man.items():
    name = rel.rsplit("/", 1)[1]
    task = name.rsplit("_", 1)[0]
    d = json.load(open(v["path"]))
    for k, val in d.items():
        top[k] += 1
        top_types[k][type(val).__name__] += 1
        tasks_with[k].add(task)
        if isinstance(val, dict):
            for cat, lst in val.items():
                if isinstance(lst, list):
                    for o in lst:
                        if isinstance(o, dict):
                            for f, fv in o.items():
                                obj_fields[k][f] += 1
                                examples.setdefault((k, f), (task, cat, json.dumps(fv)[:120]))
                        else:
                            obj_fields[k]["<non-dict item>"] += 1
                            examples.setdefault((k, "<non-dict item>"), (task, cat, json.dumps(o)[:120]))
                else:
                    obj_fields[k]["<non-list value:" + cat + ">"] += 1
                    examples.setdefault((k, "<non-list value:" + cat + ">"), (task, cat, json.dumps(lst)[:120]))
print("top-level keys (files containing):")
for k, n in top.most_common():
    ts = sorted(tasks_with[k])
    print(f"  {k}: {n} files, types {dict(top_types[k])}, tasks {len(ts)}: {ts if len(ts) < 12 else ts[:12] + ['...']}")
print()
for k in obj_fields:
    print(f"section {k}: object fields")
    for f, n in obj_fields[k].most_common():
        print(f"    {f}: {n}   e.g. {examples[(k, f)]}")
