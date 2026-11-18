"""Run a G4 plan of `run_batch.py` calls on one vis node, at most `--slots` harness processes at a time.

usage (on della-vis2): python3 -I audit/harness/g4/run_plan.py <plan.json> --slots 5 --log <log> [--node della-vis2:1]

plan.json: a list of {"name": tag, "seed": s, "layouts": "0-19" or "0,3", "procs": k, "args": "<harness args>"}.
Items start in order as soon as `procs` slots are free; each is one `run_batch.py --name <tag> --nodes <node>:<procs>
--seed <s> --layouts <layouts> --state_digest -- <args>` call (which adds --strict_layout_files --timing --fast_ik and
HARNESS_OPENBLAS_TIMEOUT=4). Writes one line per start and end to the log and `DONE` when every item has ended.
"""
from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "../../.."))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("plan")
    ap.add_argument("--slots", type=int, default=5)
    ap.add_argument("--log", required=True)
    ap.add_argument("--node", default="della-vis2:1")
    ap.add_argument("--root", default=os.path.abspath(os.path.join(REPO, "..")))
    a = ap.parse_args()
    items = json.load(open(a.plan))
    free = [a.slots]
    cv = threading.Condition()
    logf = open(a.log, "a", buffering=1)

    def log(msg):
        logf.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {msg}\n")

    def run(item):
        cmd = ["python3", "-I", f"{REPO}/audit/harness/run_batch.py", "--root", a.root, "--repo", REPO,
               "--name", item["name"], "--nodes", f"{a.node}:{item['procs']}", "--seed", str(item["seed"]),
               "--layouts", str(item["layouts"]), "--state_digest", "--"] + shlex.split(item["args"])
        t0 = time.time()
        log(f"START {item['name']} procs={item['procs']} :: {' '.join(shlex.quote(c) for c in cmd)}")
        p = subprocess.run(cmd, capture_output=True, text=True)
        tail = " | ".join(x for x in p.stdout.splitlines() if x.startswith(("BATCH summary", "  s")))[-3000:]
        log(f"END {item['name']} rc={p.returncode} wall={time.time() - t0:.1f}s {tail}")
        if p.returncode != 0:
            log(f"STDERR {item['name']} {p.stderr[-1500:]!r}")
        with cv:
            free[0] += item["procs"]
            cv.notify_all()

    threads = []
    for item in items:
        with cv:
            while free[0] < item["procs"]:
                cv.wait()
            free[0] -= item["procs"]
        t = threading.Thread(target=run, args=(item,))
        t.start()
        threads.append(t)
        time.sleep(8)
    for t in threads:
        t.join()
    log("DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
