#!/usr/bin/env python3
"""Record the population sweep (#64) in audit/gates.json: gate 0 for every config that built and reset in the harness,
the sweep summary as evidence, and `delivered` from the engine readback. Never lowers a gate or drops evidence.

    python3 -I gates_from_sweep.py <summary.json> <gates.json> <evidence path> <date>

`delivered` is false when any rigid object shows #58's signature in the readback (a bound USD physics material whose
values the engine does not carry: `shipped_cpu_05_confirmed`), or when its engine mass differs from the declared mass
(the 0.5 kg cap, #5); unchanged (null) when the sweep had no rigid object to check. Standard library only.
"""
import json
import sys

SUMMARY, GATES, EVIDENCE, DATE = sys.argv[1:5]
summ = json.load(open(SUMMARY))
led = json.load(open(GATES))
changed = []
for task, c in summ["configs"].items():
    if c.get("status") != "built_and_reset":
        continue
    t = led["tasks"][task]
    before = dict(t)
    t["gate"] = max(t["gate"], 0)
    if EVIDENCE not in t["evidence"]:
        t["evidence"].append(EVIDENCE)
    d = c.get("delivery") or {}
    if d.get("checked"):
        t["delivered"] = bool(d.get("delivered")) and t.get("delivered") is not False
    note = "layout 0 built and reset in the population sweep (#64)"
    if d.get("checked") and not d.get("delivered"):
        note += f"; not delivered: {d.get('reason')}"
    if note not in (t.get("notes") or ""):
        t["notes"] = f"{t['notes']}; {note}" if t.get("notes") else note
    if t != before:
        changed.append(task)
led["updated"] = DATE
with open(GATES, "w") as f:
    json.dump(led, f, indent=1, ensure_ascii=False)
    f.write("\n")
print(len(changed), "configs updated")
