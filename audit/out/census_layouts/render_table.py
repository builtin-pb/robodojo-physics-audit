"""Render the census table and summary. Usage: python3 -I render_table.py <census.json> <population.json>"""
import json, sys
from collections import Counter

C = json.load(open(sys.argv[1]))["configs"]
P = {c["task_name"]: c for c in json.load(open(sys.argv[2]))["configs"]}

blob_mismatch = [t for t, r in C.items() if r["file"]["distinct"] != P[t]["layouts"]["unique_lfs_blobs_all_seeds"]]
print("P1 distinct files == population.json LFS blobs:", "all 54" if not blob_mismatch else blob_mismatch)
print("per-seed layout counts:", dict(Counter(r["seeds"]["0"]["n"] for r in C.values())),
      "| equal across seeds:", all(len({v["n"] for v in r["seeds"].values()}) == 1 for r in C.values()),
      "| contiguous from 0:", all(v["contiguous_from_0"] for r in C.values() for v in r["seeds"].values()))
print()
print("| config | seeds x layouts (scored) | files | strict | arrangements | largest group | scored arr. | pos only | near 2 mm/2 deg |")
print("|---|---|---|---|---|---|---|---|---|")
for t, r in C.items():
    n = r["seeds"]["0"]["n"]
    print(f"| {t} | 3 x {n} ({r['eval_num']}) | {r['file']['distinct']} | {r['strict']['distinct']} | "
          f"{r['arr']['distinct']} | {r['arr']['largest']} | {r['arr']['distinct_scored']} | {r['arr_pos']['distinct']} | "
          f"{r['near2_task']['distinct']} |")
print()
arr = {t: r["arr"]["distinct"] for t, r in C.items()}
alld = [t for t, r in C.items() if r["arr"]["distinct"] == r["files"]]
scored_alld = [t for t, r in C.items() if r["arr"]["distinct_scored"] == 3 * r["eval_num"]]
print("arrangements < 10:", sorted(t for t, a in arr.items() if a < 10))
print("arrangements < 55:", sorted(t for t, a in arr.items() if a < 55))
print("arrangements < layouts per seed:", sorted(t for t, r in C.items() if r["arr"]["distinct"] < r["seeds"]["0"]["n"]))
print("arrangements == files (all distinct):", len(alld), "| not:", sorted(set(C) - set(alld)))
print("scored arrangements all distinct:", len(scored_alld), "| not:", sorted(set(C) - set(scored_alld)))
print("strict < 55:", sorted(t for t, r in C.items() if r["strict"]["distinct"] < 55))
print("files < 55:", sorted(t for t, r in C.items() if r["file"]["distinct"] < 55))
print("pos-only < arrangements:", {t: (r["arr_pos"]["distinct"], r["arr"]["distinct"]) for t, r in C.items()
                                     if r["arr_pos"]["distinct"] < r["arr"]["distinct"]})
print("near2 < arrangements:", {t: (r["near2_task"]["distinct"], r["arr"]["distinct"]) for t, r in C.items()
                                if r["near2_task"]["distinct"] < r["arr"]["distinct"]})
print("near2_all vs near2_task differ:", {t: (r["near2_all"]["distinct"], r["near2_task"]["distinct"]) for t, r in C.items()
                                          if r["near2_all"]["distinct"] != r["near2_task"]["distinct"]})
print("configs with clutter:", sorted(t for t, r in C.items() if max(r["clutter_per_file"]) > 0))
print("cross-seed duplicate files:", {t: r["file"]["groups_spanning_seeds"] for t, r in C.items()
                                       if r["file"]["groups_spanning_seeds"]})
