"""Population census of RoboDojo eval layouts (#170 extended to all 54 configs).

Usage (della-vis1, CPU): python3 -I layout_census.py <manifest.json> <population.json> <out.json>
  manifest.json   from hf_list_fetch.py: Assets/... path -> {path, sha256}, every file verified at HF 1127bda
  population.json audit/out/31/population.json (config list, eval_num)

Files per config: every `<task>_<n>.json` in arx_x5/<seed>/ whose name fullmatches `re.escape(task) + r"_\\d+\\.json"`
(env/seed_manager/seed_manager.py:36-47), sorted by <n>.
Objects: every instance in the sections Rigid, Geometry, Articulation, Garment, Fluid, Dynamic; clutter objects sit
in Rigid with type == "cluttered". Model id = category_idx (the loader's asset folder, layout_manager.py:178-186).
Keys, from strict to loose (each is a function of the one before, so counts never increase down the list):
  file      SHA-256 of the bytes
  content   parsed JSON, canonical (sort_keys)
  nolabel   parsed JSON with every object's `label` removed
  strict    sorted (section, category, clutter?, category_idx, default_pos, default_ori, scale)
  arr       sorted (section, category, clutter?, default_pos, default_ori, scale)   = "arrangement"
  arr_task  arr without clutter objects
Diagnostic keys: nolabel_norm (nolabel, null `group` fields dropped, Room default_rot renamed default_ori; room.py:67
reads default_ori with identity fallback), arr_rawq (arr without quaternion sign canonicalisation), arr_pos (positions
only, no orientation or scale), arr_xy_mm (x, y on a 1 mm grid).
Floats rounded to 1e-6; -0.0 -> 0.0; quaternions sign-canonicalised (q and -q are one rotation).
near_*: connected components of files whose objects match one to one (same section and category) within
2 mm and 2 degrees (and a 10 mm / 10 degree variant); poses are compared as poses, not up to object symmetry.
"""
import hashlib, json, math, re, sys
from collections import Counter, defaultdict

man_path, pop_path, out_path = sys.argv[1:4]
man = json.load(open(man_path))
pop = json.load(open(pop_path))
BASE = "Assets/Eval_Layout/RoboDojo/arx_x5/"
OBJ = ("Rigid", "Geometry", "Articulation", "Garment", "Fluid", "Dynamic")
SCENE = ("Room", "Table", "Ground", "Background")
TOLS = {"near2": (0.002, 2.0), "near10": (0.010, 10.0)}
canon_flips = 0


def rnd(v):
    if isinstance(v, (int, float)):
        v = [v]
    return tuple(round(float(x), 6) + 0.0 for x in v)


def canon_q(q):
    global canon_flips
    r = rnd(q)
    lead = next((x for x in r if x != 0.0), 0.0)
    if lead < 0:
        canon_flips += 1
        r = rnd([-float(x) for x in q])
    return r


def h(x):
    return hashlib.sha256(json.dumps(x, sort_keys=True).encode()).hexdigest()[:16]


def objects(d):
    out = []
    for sec in OBJ:
        for cat, lst in (d.get(sec) or {}).items():
            for o in lst:
                out.append({"sec": sec, "cat": cat, "clutter": o.get("type") == "cluttered",
                            "model": o.get("category_idx"), "pos": rnd(o["default_pos"]),
                            "ori": canon_q(o["default_ori"]), "ori_raw": rnd(o["default_ori"]),
                            "scale": rnd(o.get("scale", [1.0, 1.0, 1.0])), "raw": o})
    return out


def pose_t(o):
    return [o["sec"], o["cat"], o["clutter"], o["pos"], o["ori"], o["scale"]]


def keys(raw_bytes, d, objs):
    nolab = json.loads(json.dumps(d))
    for sec in OBJ:
        for lst in (nolab.get(sec) or {}).values():
            for o in lst:
                o.pop("label", None)
    norm = json.loads(json.dumps(nolab))
    for sec in OBJ:
        for lst in (norm.get(sec) or {}).values():
            for o in lst:
                if o.get("group", 0) is None:
                    o.pop("group")
    if "default_rot" in norm.get("Room", {}):
        norm["Room"]["default_ori"] = norm["Room"].pop("default_rot")
    return {
        "content": h(d),
        "nolabel": h(nolab),
        "nolabel_norm": h(norm),
        "strict": h(sorted(([o["sec"], o["cat"], o["clutter"], o["model"], o["pos"], o["ori"], o["scale"]]
                            for o in objs), key=json.dumps)),
        "arr": h(sorted((pose_t(o) for o in objs), key=json.dumps)),
        "arr_task": h(sorted((pose_t(o) for o in objs if not o["clutter"]), key=json.dumps)),
        "arr_rawq": h(sorted(([o["sec"], o["cat"], o["clutter"], o["pos"], o["ori_raw"], o["scale"]] for o in objs),
                             key=json.dumps)),
        "arr_pos": h(sorted(([o["sec"], o["cat"], o["clutter"], o["pos"]] for o in objs), key=json.dumps)),
        "arr_xy_mm": h(sorted(([o["sec"], o["cat"], o["clutter"], round(o["pos"][0] * 1000), round(o["pos"][1] * 1000)]
                               for o in objs), key=json.dumps)),
    }


def angle_deg(q1, q2):
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    n = math.sqrt(sum(a * a for a in q1) * sum(b * b for b in q2)) or 1.0
    return math.degrees(2 * math.acos(min(1.0, dot / n)))


def perfect_match(A, B, tp, ta):
    """Kuhn bipartite matching: is there a one-to-one pairing with every pair within tolerance?"""
    if len(A) != len(B):
        return False
    adj = [[j for j, b in enumerate(B) if math.dist(a[0], b[0]) <= tp and angle_deg(a[1], b[1]) <= ta]
           for a in A]
    match = [-1] * len(B)

    def aug(i, seen):
        for j in adj[i]:
            if j not in seen:
                seen.add(j)
                if match[j] < 0 or aug(match[j], seen):
                    match[j] = i
                    return True
        return False

    return all(aug(i, set()) for i in range(len(A)))


def near_components(files, use_clutter, tp, ta):
    """Union-find over files whose objects match within (tp metres, ta degrees)."""
    n = len(files)
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    cls = []
    for f in files:
        c = defaultdict(list)
        for o in f["objs"]:
            if use_clutter or not o["clutter"]:
                c[(o["sec"], o["cat"], o["clutter"])].append((o["pos"], o["ori"]))
        cls.append(c)
    sig = [h(sorted(([list(k), len(v)] for k, v in c.items()), key=json.dumps)) for c in cls]
    for i in range(n):
        for j in range(i + 1, n):
            if sig[i] != sig[j] or find(i) == find(j):
                continue
            if all(perfect_match(cls[i][k], cls[j][k], tp, ta) for k in cls[i]):
                parent[find(i)] = find(j)
    comp = Counter(find(i) for i in range(n))
    return len(comp), max(comp.values())


def vary_within(group):
    """Fields that differ between files of one arrangement group (objects aligned by pose)."""
    varying = set()
    slots = []
    for f in group:
        s = defaultdict(list)
        for o in f["objs"]:
            s[json.dumps(pose_t(o))].append(o["raw"])
        for v in s.values():
            v.sort(key=lambda r: json.dumps([r.get("category_idx"), r.get("label")]))
        slots.append(s)
    ambiguous = any(len(v) > 1 for v in slots[0].values())
    for k in slots[0]:
        for idx in range(len(slots[0][k])):
            sec = json.loads(k)[0]
            objs = [s[k][idx] for s in slots]
            for field in set().union(*objs):
                if len({json.dumps(o.get(field), sort_keys=True) for o in objs}) > 1:
                    varying.add(f"obj.{field}")
    for sec in set().union(*(f["d"].keys() for f in group)) - set(OBJ):
        vals = [f["d"].get(sec) for f in group]
        if len({json.dumps(v, sort_keys=True) for v in vals}) > 1:
            sub = set()
            for v in vals:
                if isinstance(v, dict):
                    sub |= set(v)
            for fld in sub:
                if len({json.dumps(v.get(fld) if isinstance(v, dict) else None, sort_keys=True) for v in vals}) > 1:
                    varying.add(f"{sec}.{fld}")
    return varying, ambiguous


def class_poses(files, list_xy=False):
    """Per object class: distinct poses (pos+ori), models, instances per file; optionally the distinct x,y (mm)."""
    poses, models, xy, pers = defaultdict(set), defaultdict(set), defaultdict(set), []
    for f in files:
        per = Counter()
        for o in f["objs"]:
            k = f"{o['sec']}/{o['cat']}" + ("/clutter" if o["clutter"] else "")
            poses[k].add((o["pos"], o["ori"]))
            models[k].add(o["model"])
            xy[k].add((round(o["pos"][0] * 1000), round(o["pos"][1] * 1000)))
            per[k] += 1
        pers.append(per)
    out = {}
    for k in sorted(poses):
        out[k] = {"poses": len(poses[k]), "models": sorted(models[k], key=str),
                  "per_file": sorted({p.get(k, 0) for p in pers})}
        out[k]["n_xy_mm"] = len(xy[k])
        if list_xy:
            out[k]["xy_mm"] = sorted(xy[k])[:12] + (["..."] if len(xy[k]) > 12 else [])
    return out


rx_files = defaultdict(list)
all_names = {rel[len(BASE):] for rel in man}
matched = set()
results = {}
for c in pop["configs"]:
    task = c["task_name"]
    eval_num = c["layouts"]["eval_num"]
    pat = re.compile(rf"{re.escape(task)}_\d+\.json")
    files = []
    seeds = {}
    for seed in ("0", "1", "2"):
        names = sorted((n.split("/", 1)[1] for n in all_names if n.startswith(seed + "/") and
                        pat.fullmatch(n.split("/", 1)[1])), key=lambda n: int(n[:-5].rsplit("_", 1)[1]))
        ids = [int(n[:-5].rsplit("_", 1)[1]) for n in names]
        seeds[seed] = {"n": len(names), "contiguous_from_0": ids == list(range(len(ids)))}
        for n in names:
            rel = BASE + seed + "/" + n
            matched.add(rel)
            raw = open(man[rel]["path"], "rb").read()
            assert hashlib.sha256(raw).hexdigest() == man[rel]["sha256"], rel
            d = json.loads(raw)
            objs = objects(d)
            k = keys(raw, d, objs)
            k["file"] = man[rel]["sha256"][:16]
            files.append({"id": f"{seed}/{n[:-5].rsplit('_', 1)[1]}", "seed": seed,
                          "layout_id": int(n[:-5].rsplit("_", 1)[1]), "scored": int(n[:-5].rsplit("_", 1)[1]) < eval_num,
                          "d": d, "objs": objs, "k": k})
    r = {"eval_num": eval_num, "seeds": seeds, "files": len(files)}
    for key in ("file", "content", "nolabel", "nolabel_norm", "strict", "arr", "arr_task", "arr_rawq", "arr_pos",
                "arr_xy_mm"):
        groups = Counter(f["k"][key] for f in files)
        r[key] = {"distinct": len(groups), "largest": max(groups.values()),
                  "distinct_scored": len({f["k"][key] for f in files if f["scored"]}),
                  "distinct_per_seed": {s: len({f["k"][key] for f in files if f["seed"] == s}) for s in seeds},
                  "groups_spanning_seeds": sum(1 for g in groups
                                               if len({f["seed"] for f in files if f["k"][key] == g}) > 1)}
    for name, (tp, ta) in TOLS.items():
        r[name + "_task"] = dict(zip(("distinct", "largest"), near_components(files, False, tp, ta)))
        r[name + "_all"] = dict(zip(("distinct", "largest"), near_components(files, True, tp, ta)))
    r["objects_per_file"] = sorted({len(f["objs"]) for f in files})
    r["clutter_per_file"] = sorted({sum(o["clutter"] for o in f["objs"]) for f in files})
    r["classes"] = class_poses(files)
    # what varies inside arrangement groups (same poses)
    vcount, amb = Counter(), 0
    agroups = defaultdict(list)
    for f in files:
        agroups[f["k"]["arr"]].append(f)
    multi = [g for g in agroups.values() if len(g) > 1]
    for g in multi:
        v, a = vary_within(g)
        vcount.update(v)
        amb += a
    r["within_arr_groups"] = {"groups_with_2plus": len(multi), "field_varies_in_n_groups": dict(vcount.most_common()),
                              "groups_with_ambiguous_alignment": amb}
    # model multisets (which assets, regardless of where) and label sets
    r["distinct_model_multisets"] = len({h(sorted(([o["sec"], o["cat"], o["clutter"], o["model"]] for o in f["objs"]),
                                                  key=json.dumps)) for f in files})
    r["scene_distinct"] = {s: len({json.dumps(f["d"].get(s), sort_keys=True) for f in files}) for s in SCENE}
    if r["arr"]["distinct"] < 20:
        detail = []
        for g in sorted(agroups.values(), key=len, reverse=True):
            detail.append({"size": len(g), "scored": sum(f["scored"] for f in g),
                           "seeds": dict(Counter(f["seed"] for f in g)),
                           "files": len({f["k"]["file"] for f in g}), "strict": len({f["k"]["strict"] for f in g}),
                           "nolabel": len({f["k"]["nolabel"] for f in g}),
                           "model_multisets": len({h(sorted(([o["sec"], o["cat"], o["model"]] for o in f["objs"]),
                                                            key=json.dumps)) for f in g}),
                           "example": g[0]["id"], "varying": sorted(vary_within(g)[0]) if len(g) > 1 else []})
        r["arr_groups_detail"] = detail
        r["classes"] = class_poses(files, list_xy=True)
    results[task] = r
    print(f"{task:36s} files={len(files):3d} file={r['file']['distinct']:3d} nolabel={r['nolabel']['distinct']:3d} "
          f"strict={r['strict']['distinct']:3d} arr={r['arr']['distinct']:3d} (max {r['arr']['largest']:3d}) "
          f"task={r['arr_task']['distinct']:3d} near2={r['near2_task']['distinct']:3d} "
          f"near10={r['near10_task']['distinct']:3d} norm={r['nolabel_norm']['distinct']:3d} "
          f"rawq={r['arr_rawq']['distinct']:3d} pos={r['arr_pos']['distinct']:3d} xy_mm={r['arr_xy_mm']['distinct']:3d} "
          f"scored_arr={r['arr']['distinct_scored']:3d}", flush=True)

unmatched = sorted(set(man) - matched)
summary = {"configs": len(results), "files": sum(r["files"] for r in results.values()),
           "manifest_files": len(man), "unmatched_files": unmatched, "quaternion_sign_flips": canon_flips}
print(json.dumps(summary)[:2000])
json.dump({"summary": summary, "configs": results}, open(out_path, "w"), indent=1, default=list)
print("DONE")
