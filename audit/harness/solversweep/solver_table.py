"""Solver sweep (#106): Markdown tables from `analyze_solver.py`'s summary.

usage: python3 -I solver_table.py <summary.json> > tables.md
"""
import json
import sys

CELLS = ("dt2", "dt1", "iters4x", "stab_off", "cpu")


def fmt(x, k=2):
    return "-" if x is None else (f"{x:+.{k}f}" if isinstance(x, (int, float)) else str(x))


def obj_flags(cmp):
    out = {}
    for l, o in (cmp or {}).get("objects", {}).items():
        if o.get("flags"):
            out[l] = o
    return out


def main():
    s = json.load(open(sys.argv[1]))
    cfg = s["configs"]
    print("## Per config: what moved beyond the floor, against `shipped`\n")
    print("Counts of objects whose settled pose moved beyond 1 mm or 1 degree (`L`: beyond 5 mm or 5 degrees) or whose drop landing moved beyond 1 mm or 1 degree, in both cases also beyond twice the 1 um control's change where the control ran; resting contact pairs whose count changed by at least 2 and 25%, appeared or vanished, or whose separation moved over 1 mm; slide: centre-of-mass excess over Coulomb changed by more than 0.5 mm (`L` above 2 mm) and beyond twice the control's change, yaw over 2 degrees, or tipping. `UNSTABLE`: RoboDojo's reset rejected the layout. `-` = cell not run; `·` = nothing moved. `rep` = `shipped_repeat` identical to `shipped`.\n")
    print("| Config | stab | rep | " + " | ".join(CELLS) + " |")
    print("|---|---|---|" + "---|" * len(CELLS))
    for t, c in sorted(cfg.items()):
        if c.get("error"):
            print(f"| `{t}` | | | error: {c['error'][:80]} |")
            continue
        row = []
        for k in CELLS:
            v = c["vs_shipped"].get(k)
            if v is None:
                row.append("-")
                continue
            fb = {l: o.get("flags_beyond_sensitivity", o.get("flags", [])) for l, o in v["objects"].items()}
            npose = sum(1 for f in fb.values() if any(x.startswith("POSE") or x == "MOVER" for x in f))
            nl = sum(1 for f in fb.values() if "POSE_LARGE" in f)
            ndrop = sum(1 for f in fb.values() if any(x.startswith("DROP") for x in f))
            cf = sum(1 for o in v["contacts"].values() if o["flags"])
            sl = v.get("slide") or {}
            sf = [x for x in (sl.get("flags") or []) if sl.get("beyond_sensitivity", True) or x in ("SLIDE_TIP",)]
            parts = []
            if npose:
                parts.append(f"{npose} pose" + (f" ({nl} L)" if nl else ""))
            if ndrop:
                parts.append(f"{ndrop} drop")
            if cf:
                parts.append(f"{cf} pairs")
            if sf:
                parts.append("slide " + "/".join(x.replace("SLIDE_", "").replace("SLIDE", "Δ") for x in sf))
            err = (c["delivery"].get(k) or {}).get("errors") or []
            if err:
                parts.append("ERR " + ",".join(err))
            if (c["delivery"].get(k) or {}).get("unstable"):
                parts.append("UNSTABLE")
            row.append("; ".join(parts) or "·")
        stab = "on" if (c.get("index") or {}).get("stabilization_shipped") else "off"
        print(f"| `{t}` | {stab} | {'yes' if c.get('repeat_identical') else 'NO'} | " + " | ".join(row) + " |")
    print("\n## Slide witness: excess over discrete Coulomb (mm) by cell\n")
    print("| Config | target | shape | table contacts | shipped | dt2 | dt1 | R3 (excess) | iters4x | stab_off | cpu |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    for t, c in sorted(cfg.items()):
        sl = c.get("slide") or {}
        if not sl.get("shipped"):
            continue
        tg = ((c.get("index") or {}).get("slide_target"))
        so = c.get("shipped_objects", {})
        r3 = ((c.get("refinement") or {}).get("slide") or {})
        ex = r3.get("excess_mm") or {}
        shape = None
        for l, o in so.items():
            if l == tg or l.startswith(str(tg).rsplit("_", 1)[0] + "#"):
                shape = f"{','.join(o.get('approximations') or [])} ({o.get('colliders')})"
                break
        tc = sl["shipped"]["table_contacts_min_max"]
        cells = [fmt((sl.get(k) or {}).get("excess_mm")) for k in ("shipped", "dt2", "dt1")]
        grow = " grows" if r3.get("excess_abs_grows") else ""
        others = [fmt((sl.get(k) or {}).get("excess_mm")) for k in ("iters4x", "stab_off", "cpu")]
        print(f"| `{t}` | {tg} | {shape or '-'} | {tc[0]}–{tc[1]} | " + " | ".join(cells) + f" | {ex.get('class', '-')}{grow} | " + " | ".join(others) + " |")


if __name__ == "__main__":
    main()
