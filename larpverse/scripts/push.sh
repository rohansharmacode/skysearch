#!/usr/bin/env bash
# Orchestrator only: mirror committed HEAD into the team repo (github.com/rohansharmacode/skysearch,
# branch larpverse-sensing, folder larpverse/) and push. Only committed files ship (git archive),
# so uncommitted agent work and local env files never leave this machine.
set -euo pipefail
cd "$(dirname "$0")/.."
TEAM=${TEAM_REPO:-$HOME/Downloads/UCL/skysearch}; BR=larpverse-sensing
exec 9>.git/agent-commit.lock; flock 9
[ -d "$TEAM/.git" ] || git clone -q https://github.com/rohansharmacode/skysearch.git "$TEAM"
git -C "$TEAM" fetch -q origin
git -C "$TEAM" checkout -q "$BR" 2>/dev/null || git -C "$TEAM" checkout -q -b "$BR" "origin/$BR"
git -C "$TEAM" merge -q --ff-only "origin/$BR"
rm -rf "$TEAM/larpverse"; mkdir -p "$TEAM/larpverse"
git archive HEAD | tar -x -C "$TEAM/larpverse"
git -C "$TEAM" add -A larpverse
if git -C "$TEAM" diff --cached --quiet; then echo "[push] nothing new"; exit 0; fi
git -C "$TEAM" -c user.name=manocw -c user.email=manolochow@googlemail.com commit -q -m "larpverse: $(git log -1 --format=%s)

Synced from local HEAD $(git log -1 --format=%h).

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
git -C "$TEAM" push -q origin "$BR" && echo "[push] pushed $(git -C "$TEAM" log -1 --format=%h) to $BR"
