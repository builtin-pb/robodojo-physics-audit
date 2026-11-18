"""Refuse a commit that leaves a governed file over its character limit (limits in audit/size_limits.json).

CI: `python3 audit/size_guard.py --all`. Install as a pre-commit hook: printf '#!/bin/sh\nexec python3 audit/size_guard.py\n' > .git/hooks/pre-commit && chmod +x .git/hooks/pre-commit
"""
import json
import subprocess
import sys
from pathlib import Path

root = Path(subprocess.run(["git", "rev-parse", "--show-toplevel"], capture_output=True, text=True, check=True).stdout.strip())
limits = json.loads((root / "audit/size_limits.json").read_text())
check_all = "--all" in sys.argv  # CI: check the files as checked out
staged = list(limits) if check_all else subprocess.run(["git", "diff", "--cached", "--name-only", "--diff-filter=ACMR"], capture_output=True, text=True, check=True, cwd=root).stdout.split()
bad = []
for path in staged:
    if path in limits:
        if check_all:
            text = (root / path).read_text(encoding="utf-8") if (root / path).exists() else ""
        else:
            text = subprocess.run(["git", "show", f":{path}"], capture_output=True, check=True, cwd=root).stdout.decode("utf-8")
        if len(text) > limits[path]:
            bad.append(f"  {path}: {len(text)} characters, limit {limits[path]}")
if bad:
    print("size_guard: condense before committing:\n" + "\n".join(bad))
    sys.exit(1)
