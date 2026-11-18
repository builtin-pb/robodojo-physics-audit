"""Compare harness runs field by field (audit/out/speed/): is a faster mode's episode the same episode?

usage: python3 -I audit/harness/speed_parity.py --ref A/result.json[,B/result.json] --cand C/result.json[,...]
           [--out report.json] [--max_diffs 20]

Runs are matched on (seed, layout, occurrence): the n-th run of a (seed, layout) in the reference files against the
n-th in the candidate files. Every key of a run is compared exactly (JSON values; floats by value), except:
- wall-clock fields (`WALL_KEYS`), which a speed change is meant to change;
- process-cumulative fields (`CUMULATIVE_KEYS`): counters that a process carries from one episode to the next
  (render counts) and the evaluator's running result, of which only this layout's own entries are compared
  (`eval_result.details[layout]` and its `ledger` rows), so a multi-episode process can be compared with lone runs;
- records that older harness versions do not write (`OPTIONAL_KEYS`), compared only when both runs have them.
Top-level fields are compared too, except provenance (`versions`, `speed`, `timing`, and the namespace-shadowing record,
which holds the checkout's path).
Instance names carry a process-wide creation counter (`bottle_60_1` in a lone run is `bottle_60_10` as the third
scene of a process; prim paths and contact-report actor names follow), so strings and dict keys are compared after
replacing each path component's trailing `_<digits>` with `_#` (`INSTANCE_COUNTER`); a field equal only after this is
reported with `raw_equal: false`. A dict whose keys would collide under this is compared with its raw keys.
A run passes when every compared field is equal; `settled_state` / `final_state` (`--state_digest`) make that a
bit-level check of every rigid pose and robot joint at the end of reset and of the episode.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from collections import defaultdict

WALL_KEYS = {"timing", "wall_seconds", "episode_seconds", "steps_per_second"}
CUMULATIVE_KEYS = {"renders_through_reset", "renders_total", "camera_reset_steps", "eval_result"}
OPTIONAL_KEYS = {"layout_files", "settled_state", "final_state"}  # newer records: compared only when both runs have them
TOP_SKIP = {"versions", "speed", "timing", "runs", "namespace_shadowing_before_pin"}  # provenance (commit, flags, host paths)


INSTANCE_COUNTER = re.compile(r"_\d+(?=/|$)")


def normalize_names(v):
    """Replace instance-name creation counters in every string and dict key (see the module docstring)."""
    if isinstance(v, str):
        return INSTANCE_COUNTER.sub("_#", v)
    if isinstance(v, list):
        return [normalize_names(x) for x in v]
    if isinstance(v, dict):
        keys = [normalize_names(k) if isinstance(k, str) else k for k in v]
        if len(set(keys)) < len(keys):  # two instances of one asset: keep raw keys rather than merge them
            return {k: normalize_names(x) for k, x in v.items()}
        return {nk: normalize_names(x) for nk, x in zip(keys, v.values())}
    return v


def load_runs(paths):
    tops, runs = [], []
    for p in paths:
        with open(p) as f:
            r = json.load(f)
        tops.append((p, r))
        for i, run in enumerate(r.get("runs", [])):
            runs.append((p, i, run))
    return tops, runs


def key_runs(runs):
    out = {}
    seen = defaultdict(int)
    for p, i, run in runs:
        k = (run.get("seed"), run.get("layout"))
        out[(k[0], k[1], seen[k])] = (p, i, run)
        seen[k] += 1
    return out


def deep_diff(a, b, path="", out=None, limit=50):
    """Exact structural comparison; returns a list of (path, ref, cand) for the first `limit` differences."""
    if out is None:
        out = []
    if len(out) >= limit:
        return out
    if isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b), key=str):
            if k not in a:
                out.append((f"{path}.{k}", "<missing>", _short(b[k])))
            elif k not in b:
                out.append((f"{path}.{k}", _short(a[k]), "<missing>"))
            else:
                deep_diff(a[k], b[k], f"{path}.{k}", out, limit)
            if len(out) >= limit:
                break
        return out
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            out.append((f"{path}[len]", len(a), len(b)))
            return out
        for i, (x, y) in enumerate(zip(a, b)):
            deep_diff(x, y, f"{path}[{i}]", out, limit)
            if len(out) >= limit:
                break
        return out
    if _equal_scalar(a, b):
        return out
    out.append((path, _short(a), _short(b)))
    return out


def _equal_scalar(a, b):
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b or (type(a) is type(b) and a == b)
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        if isinstance(a, float) and isinstance(b, float) and math.isnan(a) and math.isnan(b):
            return True
        return a == b
    return a == b


def _short(v, n=120):
    s = json.dumps(v, default=str)
    return s if len(s) <= n else s[:n] + "..."


def max_abs_numeric_diff(a, b):
    """Largest |a - b| over numbers at matching positions (None if the structures differ or hold no numbers)."""
    best = [None]

    def walk(x, y):
        if isinstance(x, dict) and isinstance(y, dict):
            for k in set(x) & set(y):
                walk(x[k], y[k])
        elif isinstance(x, list) and isinstance(y, list) and len(x) == len(y):
            for u, v in zip(x, y):
                walk(u, v)
        elif isinstance(x, (int, float)) and isinstance(y, (int, float)) and not isinstance(x, bool) and not isinstance(y, bool):
            d = abs(float(x) - float(y))
            if best[0] is None or d > best[0]:
                best[0] = d

    walk(a, b)
    return best[0]


def own_eval_entries(run):
    """This layout's own entries of the (process-cumulative) evaluator result: `details` values and `ledger` rows whose
    layout_id is the run's layout (details are keyed by episode index, not by layout id)."""
    ev = run.get("eval_result") or {}
    lay = run.get("layout")
    det = [v for v in (ev.get("details") or {}).values() if isinstance(v, dict) and v.get("layout_id") == lay]
    ledger = [row for row in (ev.get("ledger") or []) if isinstance(row, dict) and row.get("layout_id") == lay]
    return {"details": det, "ledger": ledger}


def compare_run(ref, cand, max_diffs=20):
    fields = {}
    for k in sorted(set(ref) | set(cand)):
        if k in WALL_KEYS or k in CUMULATIVE_KEYS:
            continue
        if k not in ref or k not in cand:
            if k in OPTIONAL_KEYS:
                continue
            fields[k] = {"equal": False, "note": "missing in " + ("ref" if k not in ref else "cand")}
            continue
        raw = deep_diff(ref[k], cand[k], limit=max_diffs)
        d = deep_diff(normalize_names(ref[k]), normalize_names(cand[k]), limit=max_diffs) if raw else raw
        entry = {"equal": not d, "raw_equal": not raw}
        if d:
            entry["diffs"] = d
            m = max_abs_numeric_diff(ref[k], cand[k])
            if m is not None:
                entry["max_abs_diff"] = m
        elif raw:
            entry["raw_diffs_example"] = raw[:3]
        fields[k] = entry
    d = deep_diff(own_eval_entries(ref), own_eval_entries(cand), limit=max_diffs)
    fields["eval_result[own layout]"] = {"equal": not d, "raw_equal": not d, **({"diffs": d} if d else {})}
    return {"equal": all(v["equal"] for v in fields.values()), "fields": fields}


def compare(ref_paths, cand_paths, max_diffs=20):
    rtops, rruns = load_runs(ref_paths)
    ctops, cruns = load_runs(cand_paths)
    rk, ck = key_runs(rruns), key_runs(cruns)
    report = {"ref": ref_paths, "cand": cand_paths, "runs": [], "unmatched_ref": [], "unmatched_cand": []}
    for k in sorted(set(rk) | set(ck), key=lambda t: tuple(-1 if v is None else v for v in t)):
        if k not in ck:
            report["unmatched_ref"].append(list(k))
            continue
        if k not in rk:
            report["unmatched_cand"].append(list(k))
            continue
        res = compare_run(rk[k][2], ck[k][2], max_diffs)
        res.update({"seed": k[0], "layout": k[1], "occurrence": k[2], "ref": f"{rk[k][0]}#{rk[k][1]}", "cand": f"{ck[k][0]}#{ck[k][1]}"})
        if "final_state" in rk[k][2] and "final_state" in ck[k][2]:
            res["final_state_sha256"] = [rk[k][2]["final_state"].get("sha256"), ck[k][2]["final_state"].get("sha256")]
        report["runs"].append(res)
    top = {}
    for key in sorted({k for _, t in rtops + ctops for k in t} - TOP_SKIP):
        rv = [t.get(key) for _, t in rtops]
        cv = [t.get(key) for _, t in ctops]
        diffs = [deep_diff(a, b, limit=max_diffs) for a in rv[:1] for b in cv]
        top[key] = {"equal": all(not d for d in diffs), **({"diffs": [d for d in diffs if d][0]} if any(diffs) else {})}
    report["top_level"] = top
    report["all_runs_equal"] = bool(report["runs"]) and all(r["equal"] for r in report["runs"]) and not report["unmatched_ref"]
    return report


def summary_lines(report):
    lines = []
    for r in report["runs"]:
        bad = [k for k, v in r["fields"].items() if not v["equal"]]
        names = [k for k, v in r["fields"].items() if v["equal"] and not v.get("raw_equal", True)]
        verdict = ("IDENTICAL" + (f" (instance-name counters differ in {', '.join(names)})" if names else "")) if r["equal"] else "DIFFERS in " + ", ".join(bad)
        lines.append(f"seed {r['seed']} layout {r['layout']} #{r['occurrence']}: {verdict}"
                     + (f" (final_state sha {r['final_state_sha256'][0][:12]} vs {r['final_state_sha256'][1][:12]})" if r.get("final_state_sha256") else ""))
    bad_top = [k for k, v in report["top_level"].items() if not v["equal"]]
    lines.append("top level: " + ("identical" if not bad_top else "differs in " + ", ".join(bad_top)))
    if report["unmatched_ref"] or report["unmatched_cand"]:
        lines.append(f"unmatched ref {report['unmatched_ref']} cand {report['unmatched_cand']}")
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", required=True)
    ap.add_argument("--cand", required=True)
    ap.add_argument("--out", default=None)
    ap.add_argument("--max_diffs", type=int, default=20)
    a = ap.parse_args(argv)
    rep = compare(a.ref.split(","), a.cand.split(","), a.max_diffs)
    for line in summary_lines(rep):
        print(line)
    if a.out:
        with open(a.out, "w") as f:
            json.dump(rep, f, indent=1, default=str)
    return 0 if rep["all_runs_equal"] else 1


if __name__ == "__main__":
    sys.exit(main())
