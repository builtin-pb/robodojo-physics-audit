"""Solver sweep (#106): write one cells file per config, from the shipped population sweep, before any run.

Cells (layout 0, seed 0; each changes one thing from shipped):
  shipped         RoboDojo as shipped: GPU dynamics, dt 4 ms, stabilization as the task sets it, authored iterations
  dt2, dt1        time step 2 ms and 1 ms (control stays 25 Hz; the cell runner keeps simulated times equal)
  iters4x         scene minimum position iterations = 4 x the largest count any actor authors (16 for rigid objects,
                  32 for articulated scene objects), i.e. 64 or 128; on GPU dynamics the scene runs the largest
                  count, so this is 4 x the shipped effective count
  stab_off        scene stabilization off, only in the configs where RoboDojo turns it on (teleop tasks, #15)
  cpu             CPU dynamics instead of RoboDojo's forced GPU, except cloth and fluid configs (GPU only)
  shipped_repeat  shipped again, last: determinism and absence of order effects inside one process

usage: python3 -I make_cells.py <targets.json> <popsweep runs dir> <out dir>
"""
import glob
import json
import os
import sys

GPU_ONLY = {"fold_clothes", "fold_clothes_random", "pour_by_language", "pour_liquid_into_cup", "pour_liquid_into_cup_random"}


def main():
    targets = json.load(open(sys.argv[1]))
    runs_dir, out_dir = sys.argv[2], sys.argv[3]
    os.makedirs(out_dir, exist_ok=True)
    manifests = {}
    for f in sorted(glob.glob(os.path.join(runs_dir, "settle", "*.json"))) + sorted(glob.glob(os.path.join(runs_dir.rstrip("/") + "_slurm", "settle", "*.json"))):
        d = json.load(open(f))
        if d.get("runs") and d["runs"][0].get("manifest"):
            manifests[os.path.basename(f)[:-5]] = d["runs"][0]["manifest"]
    index = {}
    for task, man in sorted(manifests.items()):
        stab = bool(man["sim_cfg_physx"]["enable_stabilization"])
        authored = [b["pos_iters"] for b in man.get("rigid_bodies_authoring_iterations", []) if b.get("pos_iters")]
        authored += [a["pos_iters"] for a in man.get("articulations", []) if a.get("pos_iters")]
        it_max = int(max(authored)) if authored else 16
        tgt = (targets.get(task) or {}).get("target")
        st = {"slide_target": {k: tgt[k] for k in ("label", "inst", "labelled", "stem", "stem_rank", "dir_world_xy")}} if tgt else {}
        cells = [("shipped", {}), ("dt2", {"dt_s": 0.002}), ("dt1", {"dt_s": 0.001}), ("iters4x", {"min_pos_iters": 4 * it_max})]
        if stab:
            cells.append(("stab_off", {"enable_stabilization": False}))
        if task not in GPU_ONLY:
            cells.append(("cpu", {"gpu_dynamics": False}))
        cells.append(("shipped_repeat", {}))
        spec = [{"cell_name": f"{task}__{n}", "layout": 0, "overrides": dict(ov, **st)} for n, ov in cells]
        json.dump(spec, open(os.path.join(out_dir, f"{task}.json"), "w"), indent=1)
        index[task] = {"stabilization_shipped": stab, "max_authored_pos_iters": it_max, "cells": [n for n, _ in cells],
                       "slide_target": tgt and tgt["label"]}
    json.dump(index, open(os.path.join(out_dir, "index.json"), "w"), indent=1, sort_keys=True)
    print(len(index), "configs;", sum(len(v["cells"]) for v in index.values()), "cells")


if __name__ == "__main__":
    main()
