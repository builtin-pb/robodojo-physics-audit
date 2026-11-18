#!/usr/bin/env python3
"""Task x check x predicate x argument table for RoboDojo's scoring (#106 item 3, predicate audit).

Static: parses `task/RoboDojo/tasks/*.py` and `env/reward_manager/reward_manager.py` with `ast`; imports and runs
nothing from RoboDojo. For every call of a RewardManager predicate wrapper in a task file it records the task, the
enclosing method, the line, the structural call it flows into (check, query, trigger, score, repeat) and the
effective arguments (the call's own arguments plus the wrapper's defaults, as source text).

    python3 -I audit/tools/predicate_table.py --repo . --out audit/out/predicates
writes `task_checks.json` (one row per call site) and `task_checks.md` (grouped by predicate, then task).
"""

from __future__ import annotations

import argparse
import ast
from collections import defaultdict
import json
from pathlib import Path

STRUCTURAL = {
    "check": "success",
    "check_single_env": "success",
    "repeat": "success",
    "final_check": "success",
    "query": "query",
    "trigger_check": "success (triggered)",
    "trigger_query": "query (triggered)",
    "score": "score",
    "score_single_env": "score",
    "trigger_score": "score (triggered)",
    "final_score": "score",
}
SUCCESS_METHODS = {"run_reward"}
SCORE_METHODS = {"get_score"}
# Called by the env on the support-arm path, outside run_reward and get_score, and able to drop the episode.
EXCLUSION_METHODS = {"check_support_arm_stable"}


def wrapper_signatures(rm_path: Path) -> dict[str, list[tuple[str, str | None]]]:
    tree = ast.parse(rm_path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    sigs = {}
    for f in cls.body:
        if not isinstance(f, ast.FunctionDef):
            continue
        returns_tuple = any(isinstance(n, ast.Return) and isinstance(n.value, ast.Tuple) for n in ast.walk(f))
        if not returns_tuple:
            continue
        a = f.args
        names = [x.arg for x in a.args][1:]
        defaults = [None] * (len(names) - len(a.defaults)) + [ast.unparse(d) for d in a.defaults]
        sigs[f.name] = list(zip(names, defaults))
    return sigs


def parents(tree: ast.AST) -> dict[ast.AST, ast.AST]:
    par = {}
    for node in ast.walk(tree):
        for ch in ast.iter_child_nodes(node):
            par[ch] = node
    return par


def method_callgraph(cls: ast.ClassDef) -> dict[str, set[str]]:
    graph = {}
    for f in cls.body:
        if isinstance(f, ast.FunctionDef):
            called = set()
            for n in ast.walk(f):
                if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and isinstance(n.func.value, ast.Name):
                    if n.func.value.id == "self":
                        called.add(n.func.attr)
            graph[f.name] = called
    return graph


def roots_reaching(graph: dict[str, set[str]], target: str) -> set[str]:
    out = set()
    for root in SUCCESS_METHODS | SCORE_METHODS | EXCLUSION_METHODS:
        seen, stack = set(), [root]
        while stack:
            m = stack.pop()
            if m in seen:
                continue
            seen.add(m)
            if m == target:
                out.add(root)
                break
            stack.extend(graph.get(m, ()))
    return out


def task_rows(path: Path, sigs: dict) -> list[dict]:
    src = path.read_text()
    tree = ast.parse(src)
    par = parents(tree)
    rows = []
    for cls in [n for n in tree.body if isinstance(n, ast.ClassDef)]:
        graph = method_callgraph(cls)
        for f in [n for n in cls.body if isinstance(n, ast.FunctionDef)]:
            for n in ast.walk(f):
                if not (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr in sigs):
                    continue
                # Calls on func_parser (direct predicate calls, e.g. make_kong's support check) are recorded too.
                receiver = ast.unparse(n.func.value)
                structural = None
                p = par.get(n)
                while p is not None and p is not f:
                    if isinstance(p, ast.Call) and isinstance(p.func, ast.Attribute) and p.func.attr in STRUCTURAL:
                        structural = p.func.attr
                        break
                    p = par.get(p)
                if structural is not None:
                    role = STRUCTURAL[structural]
                elif f.name in EXCLUSION_METHODS:
                    role = "exclusion (support-arm validity)"
                elif receiver.endswith("func_parser"):
                    role = "support-arm gating"
                else:
                    reach = roots_reaching(graph, f.name)
                    role = " + ".join(
                        sorted(
                            {"success" if r in SUCCESS_METHODS else "score" if r in SCORE_METHODS else "exclusion" for r in reach}
                        )
                    ) or "unreached"
                names = [k for k, _ in sigs[n.func.attr]]
                given = {}
                # Direct Func_Parser calls take one dict of arguments (positional or `args=`).
                dict_arg = [a for a in n.args if isinstance(a, ast.Dict)] + [
                    kw.value for kw in n.keywords if kw.arg == "args" and isinstance(kw.value, ast.Dict)
                ]
                if receiver.endswith("func_parser") and dict_arg:
                    for k, v in zip(dict_arg[0].keys, dict_arg[0].values):
                        if isinstance(k, ast.Constant) and k.value != "env_idx":
                            given[str(k.value)] = ast.unparse(v)
                for i, a in enumerate([] if (receiver.endswith("func_parser") and dict_arg) else n.args):
                    if i < len(names):
                        given[names[i]] = ast.unparse(a)
                for kw in n.keywords:
                    if kw.arg is not None and kw.arg != "args":
                        given[kw.arg] = ast.unparse(kw.value)
                eff = {}
                for k, d in sigs[n.func.attr]:
                    if k in given:
                        eff[k] = given[k]
                    elif d is not None:
                        eff[k] = d + " (default)"
                rows.append(
                    {
                        "task": path.stem,
                        "method": f.name,
                        "line": n.lineno,
                        "role": role,
                        "structural_call": structural,
                        "predicate": n.func.attr,
                        "args": eff,
                    }
                )
    return rows


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", default=".")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    repo = Path(args.repo)
    sigs = wrapper_signatures(repo / "env/reward_manager/reward_manager.py")
    rows = []
    for t in sorted((repo / "task/RoboDojo/tasks").glob("*.py")):
        rows.extend(task_rows(t, sigs))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    used = sorted({r["predicate"] for r in rows})
    unused = sorted(set(sigs) - set(used))
    by_pred = defaultdict(lambda: defaultdict(list))
    for r in rows:
        by_pred[r["predicate"]][r["task"]].append(r)
    summary = {
        p: {"tasks": sorted(by_pred[p]), "n_tasks": len(by_pred[p]), "n_call_sites": sum(len(v) for v in by_pred[p].values())}
        for p in used
    }
    json.dump(
        {"source": "static parse of task/RoboDojo/tasks/*.py and env/reward_manager/reward_manager.py",
         "wrappers": sorted(sigs), "unused_wrappers": unused, "summary": summary, "rows": rows},
        open(out / "task_checks.json", "w"),
        indent=1,
    )

    def argtext(a: dict) -> str:
        skip = {"label_A_args", "label_B_args", "label_args"}
        return ", ".join(f"{k}={v}" for k, v in a.items() if k not in skip and v not in ("None (default)",))

    lines = [
        "# Task x check x predicate x arguments",
        "",
        "Generated by `python3 -I audit/tools/predicate_table.py --repo . --out audit/out/predicates` from the task files "
        "(static parse; `(default)` marks a wrapper default the call did not override). Role: `success` feeds `check` "
        "stages, `query` counts and fails the episode above its aim, `score` feeds partial credit, `exclusion` can drop "
        "the episode from the denominator. One row per distinct (role, arguments) per task; `n` is the number of call sites.",
        "",
        f"{len(rows)} call sites of {len(used)} predicates in {len({r['task'] for r in rows})} task files. "
        f"Wrappers never called by a task: {', '.join(unused) or 'none'}.",
        "",
    ]
    for p in sorted(used, key=lambda x: (-summary[x]["n_tasks"], x)):
        lines += [f"## `{p}` ({summary[p]['n_tasks']} tasks, {summary[p]['n_call_sites']} call sites)", ""]
        lines += ["| Task | Role | Arguments | n | Lines |", "|---|---|---|---|---|"]
        for t in sorted(by_pred[p]):
            groups = defaultdict(list)
            for r in by_pred[p][t]:
                groups[(r["role"], argtext(r["args"]))].append(r["line"])
            for (role, a), ls in sorted(groups.items()):
                a = a.replace("|", "\\|")
                lines.append(f"| {t} | {role} | {a} | {len(ls)} | {', '.join(map(str, sorted(set(ls))))} |")
        lines.append("")
    (out / "task_checks.md").write_text("\n".join(lines))
    print(f"{len(rows)} rows, {len(used)} predicates used, {len(unused)} unused: {unused}")


if __name__ == "__main__":
    main()
