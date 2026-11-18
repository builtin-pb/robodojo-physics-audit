"""Write per-step logs for rendered replays (stress_hv): one gzipped JSON per harness run with, for each selected case,
the moving body's pose every physics step (launch cases) or the pressing arm's joints, the button joint and the
evaluator's counters (robot_press cases), plus the static poses a replay needs. stdlib only.
usage: python3 extract_clips.py <out_dir> <result.json>[:case,case,...] ...
"""
import gzip
import json
import os
import sys


def main():
    out_dir = sys.argv[1]
    os.makedirs(out_dir, exist_ok=True)
    for arg in sys.argv[2:]:
        path, _, sel = arg.partition(":")
        names = set(sel.split(",")) if sel else None
        d = json.load(open(path))
        run = d["runs"][0]
        pl = run["policy_log"]
        rec = {"source": os.path.basename(os.path.dirname(path)), "layout": run.get("layout"), "dt": pl.get("dt"),
               "env_origin": pl.get("env_origin"), "labels": pl.get("labels"), "settled": pl.get("settled"),
               "versions": d.get("versions"), "cases": []}
        for c in pl.get("stress", []):
            name = c.get("name") or c.get("label")
            if names is not None and name not in names:
                continue
            keep = {k: v for k, v in c.items() if k not in ("props", "rows", "extra_rays", "contacts")}
            rec["cases"].append(keep)
        out = os.path.join(out_dir, f"steps_{rec['source']}.json.gz")
        with gzip.open(out, "wt") as f:
            json.dump(rec, f, separators=(",", ":"))
        print(out, len(rec["cases"]), os.path.getsize(out))


if __name__ == "__main__":
    main()
