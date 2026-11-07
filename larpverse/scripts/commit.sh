#!/usr/bin/env bash
# scripts/commit.sh "message" path [path...]: commit ONLY these paths, locally, under a lock,
# so parallel agents sharing one checkout never fight over the index. Never pushes.
set -euo pipefail
cd "$(dirname "$0")/.."
msg=$1; shift; [ $# -gt 0 ] || { echo "usage: scripts/commit.sh \"message\" path [path...]" >&2; exit 2; }
exec 9>.git/agent-commit.lock; flock 9
for p in "$@"; do git add -A -- "$p" 2>/dev/null || true; done
if git diff --cached --quiet -- "$@"; then echo "[commit] nothing to commit in: $*"; exit 0; fi
git commit -q -m "$msg" -- "$@"
echo "[commit] committed locally: $(git log -1 --format=%h) $msg"
