#!/usr/bin/env python3
"""Compact solver-sweep result.json files for the repository (#106): drop only the PhysX statistics-API docstrings
(identical in every manifest), write minified, gzipped JSON. Raw files stay on Della scratch, with their SHA-256 in
RAW_SHA256. Standard library only.

    python3 -I compact_solver.py <runs dir> <out dir>
"""
import gzip
import json
import os
import sys

RUNS, OUT = sys.argv[1:3]
os.makedirs(OUT, exist_ok=True)
n = 0
for task in sorted(os.listdir(RUNS)):
    p = os.path.join(RUNS, task, "result.json")
    if not os.path.isfile(p):
        continue
    d = json.load(open(p))
    for run in d.get("runs", []):
        (run.get("manifest") or {}).pop("statistics_api", None)
    raw = json.dumps(d, separators=(",", ":"), default=str).encode()
    with open(os.path.join(OUT, f"{task}.json.gz"), "wb") as fh, gzip.GzipFile(fileobj=fh, mode="wb", compresslevel=9, mtime=0) as f:
        f.write(raw)  # mtime 0: the same input gives the same bytes and checksum
    n += 1
print("compacted", n, "files")
