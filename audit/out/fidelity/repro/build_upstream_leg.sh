#!/bin/bash
# Build the upstream leg: RoboDojo f57e163's tree unchanged plus audit/ from a commit of this branch (default HEAD).
# usage: bash build_upstream_leg.sh [fork_commit]   (prints the new commit; makes branch bp/fidelity-upstream-leg)
set -e
SRC=${1:-HEAD}; IDX=$(mktemp -d)/idx
GIT_INDEX_FILE=$IDX git read-tree f57e163
GIT_INDEX_FILE=$IDX git read-tree --prefix=audit/ $SRC:audit
TREE=$(GIT_INDEX_FILE=$IDX git write-tree)
C=$(git commit-tree $TREE -p f57e163 -m "fidelity upstream leg: RoboDojo f57e163 unchanged plus audit/ from bp/fidelity $(git rev-parse --short $SRC)")
test -z "$(git diff --name-only f57e163 $C -- . ':!audit')" || { echo "non-audit difference" >&2; exit 1; }
git branch -f bp/fidelity-upstream-leg $C; echo $C
