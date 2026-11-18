"""Merge chosen step frames of several clip_run.py state logs into one replay source (camera smokes; JSON only).

usage: python3 -I merge_states.py <out index.json> <tag>=<state log index.json>:<step>,<step>,... [...]
Keeps each chosen frame as logged, adds "src" (the tag) and "src_i" (its index in the source log) and renumbers "i".
"""
import json
import sys

out, frames, sources = sys.argv[1], [], []
for arg in sys.argv[2:]:
    tag, rest = arg.split("=", 1)
    path, steps = rest.rsplit(":", 1)
    want = [int(s) for s in steps.split(",")]
    log = json.load(open(path))
    by_step = {fr["phys"]: fr for fr in log["frames"] if fr.get("why") == "step"}
    for s in want:
        fr = dict(by_step[s])
        fr.update(src=tag, src_i=fr["i"], i=len(frames))
        frames.append(fr)
    sources.append({"tag": tag, "path": path, "steps": want})
json.dump({"merged_from": sources, "frames": frames}, open(out, "w"), indent=1)
print(f"{len(frames)} frames from {len(sources)} logs -> {out}")
