#!/usr/bin/env bash
# Run one heavy command politely: FIFO queue, 1 heavy slot + 1 browser slot machine-wide,
# memory cap per job, free-memory floor, low priority, browser timeout.
#   scripts/heavy.sh .venv/bin/python -m pytest -q tests/test_x.py
#   scripts/heavy.sh --browser .venv/bin/python scripts/record_animation.py ...
set -u
BROWSER=""; if [ "${1:-}" = "--browser" ]; then BROWSER=1; shift; fi
[ $# -gt 0 ] || { echo "usage: scripts/heavy.sh [--browser] <command> [args...]" >&2; exit 2; }
SLOTS=${HEAVY_SLOTS:-2}; MEM=${HEAVY_MEM:-4G}; SWAP=${HEAVY_SWAP:-512M}
MIN_FREE_MB=${HEAVY_MIN_FREE_MB:-1500}
LOCKDIR=${XDG_RUNTIME_DIR:-/tmp}/larpverse-heavy; QDIR="$LOCKDIR/queue"; mkdir -p "$QDIR"
export OMP_NUM_THREADS=${OMP_NUM_THREADS:-2} OPENBLAS_NUM_THREADS=${OPENBLAS_NUM_THREADS:-2} MKL_NUM_THREADS=${MKL_NUM_THREADS:-2}
avail_mb() { awk '/^MemAvailable:/ { print int($2/1024) }' /proc/meminfo; }
waited=""; note() { [ -z "$waited" ] && echo "[heavy] waiting: $1" >&2; waited=1; }
# Browser jobs first wait for the single browser slot OUTSIDE the FIFO, so a job waiting on the
# browser never blocks unrelated heavy jobs (head-of-line blocking fix, 19:55).
browser_fd=""
if [ -n "$BROWSER" ]; then
  until [ -n "$browser_fd" ]; do exec {b}>"$LOCKDIR/browser"
    if flock -n "$b"; then browser_fd=$b; else exec {b}>&-; note "browser slot busy"; sleep 3; fi; done
else exec {browser_fd}>/dev/null; fi
# FIFO: take a ticket; only the oldest live ticket may try the slot locks.
ticket="$QDIR/$(date +%s%N).$$"; : > "$ticket"; trap 'rm -f "$ticket"' EXIT INT TERM
my_turn() { local t; for t in $(ls "$QDIR" | sort); do
  if kill -0 "${t##*.}" 2>/dev/null; then [ "$QDIR/$t" = "$ticket" ]; return; fi; rm -f "$QDIR/$t"; done; }
until my_turn; do note "queued behind $(( $(ls "$QDIR" | wc -l) - 1 )) job(s)"; sleep 1; done
slot_fd=""
until [ -n "$slot_fd" ]; do for i in $(seq 1 "$SLOTS"); do exec {fd}>"$LOCKDIR/slot$i"
    if flock -n "$fd"; then slot_fd=$fd; break; fi; exec {fd}>&-; done
  [ -n "$slot_fd" ] || { note "all $SLOTS heavy slots busy"; sleep 3; }; done
while [ "$(avail_mb)" -lt "$MIN_FREE_MB" ]; do note "only $(avail_mb) MB free (need $MIN_FREE_MB)"; sleep 5; done
rm -f "$ticket"; [ -n "$waited" ] && echo "[heavy] starting: $*" >&2
run() { if systemd-run --user --scope -q true >/dev/null 2>&1; then
    systemctl --user set-property --runtime larpverse-heavy.slice MemoryMax="${HEAVY_TOTAL_MEM:-8G}" \
      MemorySwapMax=1G CPUQuota="${HEAVY_TOTAL_CPU:-1200%}" >/dev/null 2>&1 || true
    systemd-run --user --scope -q --slice=larpverse-heavy.slice -p MemoryMax="$MEM" -p MemorySwapMax="$SWAP" \
      nice -n 10 ionice -c2 -n7 "$@" {slot_fd}>&- {browser_fd}>&-
  else nice -n 10 ionice -c2 -n7 "$@" {slot_fd}>&- {browser_fd}>&-; fi; }
if [ -n "$BROWSER" ]; then run timeout --kill-after=30 "${HEAVY_BROWSER_TIMEOUT:-1200}" "$@"; else run "$@"; fi
status=$?
[ "$status" -eq 137 ] && echo "[heavy] killed: over HEAVY_MEM=$MEM (rerun with a higher HEAVY_MEM, don't remove the cap)" >&2
exit "$status"
