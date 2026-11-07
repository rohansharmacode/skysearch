#!/usr/bin/env bash
# One-command Larpverse demo: seed-7 autonomous grid mission in realtime-ish mode -> dashboard -> handover brief.
#
#   scripts/demo.sh                 # speed 4, pile cap 3 (~2 min wall with warm caches), best available backends
#   scripts/demo.sh --speed 6       # faster (full uncapped run measured 134 s at x6)
#   scripts/demo.sh --fast          # no realtime pacing (~1.5 min compute, cold caches), e.g. to regenerate frames for a replay
#   scripts/demo.sh --slam          # also build pile maps with LiDAR SLAM (cached; changes the run, see docs/DEMO.md)
#   scripts/demo.sh --stub          # force the synthetic acoustic stub
#   scripts/demo.sh --no-gemini     # force the argmax decider / template brief even if a key exists
#   scripts/demo.sh --direct        # on stage: run the mission directly, not through the shared heavy.sh queue
#   scripts/demo.sh --all-piles     # no pile cap (default --piles 3 until BL-3 lands: ends after B2-R1)
#
# Env: DEMO_SEED (7), DEMO_SPEED (4), DEMO_START_PILE, DEMO_PORT (8765), DEMO_PILES (3; 0 = no cap), DEMO_STAGE=1 (= --direct).
# Writes dashboard/data/{state.json,pointcloud.bin,frames/state_XXXX.json,brief.md}. Leaves no background processes.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd); cd "$ROOT"

SEED=${DEMO_SEED:-7}; SPEED=${DEMO_SPEED:-4}; PORT=${DEMO_PORT:-8765}; START=${DEMO_START_PILE:-}
REALTIME=1; SLAM=""; FORCE_STUB=""; NO_GEMINI=""; PILES=${DEMO_PILES:-3}
DIRECT=""; [ "${DEMO_STAGE:-}" = 1 ] && DIRECT=1
while [ $# -gt 0 ]; do case "$1" in
  --speed) SPEED=${2:?--speed needs a number}; shift 2 ;;
  --fast) REALTIME=""; shift ;;
  --slam) SLAM=1; shift ;;
  --stub) FORCE_STUB=1; shift ;;
  --no-gemini) NO_GEMINI=1; shift ;;
  --direct) DIRECT=1; shift ;;
  --all-piles) PILES=0; shift ;;
  --piles) PILES=${2:?}; shift 2 ;;
  --seed) SEED=${2:?}; shift 2 ;;
  --start-pile) START=${2:?}; shift 2 ;;
  -h|--help) sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
  *) echo "demo.sh: unknown option '$1' (try --help)" >&2; exit 2 ;;
esac; done

bold() { printf '\033[1m%s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m!! %s\033[0m\n' "$*" >&2; }
die()  { printf '\033[1;31mxx %s\033[0m\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------- (a) environment checks
bold "== Larpverse demo (seed $SEED) =="
PY=.venv/bin/python
[ -x "$PY" ] || die "missing .venv (python env). Create it: python3 -m venv .venv && .venv/bin/pip install numpy scipy matplotlib pytest"
"$PY" -c "import numpy, scipy, sim.grid_mission" 2>/dev/null || die ".venv cannot import numpy/scipy/sim.grid_mission; run from a clean checkout or reinstall .venv"
[ -n "$DIRECT" ] || [ -x scripts/heavy.sh ] || die "scripts/heavy.sh missing or not executable"

if [ -n "$FORCE_STUB" ]; then ACOUSTIC=stub; warn "ACOUSTIC: synthetic STUB forced (--stub): clip classes/priorities come from a labelled model, not the real audio models"
elif [ -x .venv-audio/bin/python ]; then ACOUSTIC=real; echo "acoustic : real models via .venv-audio (cached clips in dashboard/data/acoustic_cache when available)"
else ACOUSTIC=stub
  warn "ACOUSTIC: .venv-audio NOT FOUND -> using the SYNTHETIC STUB. Priorities are not from the real audio models."
  warn "          Say so on stage; to get real models back, restore .venv-audio (see acoustic/README)."
fi

has_key() {   # true if GEMINI_API_KEY is set in the env or in .env; never prints the key
  [ -n "${GEMINI_API_KEY:-}" ] && return 0
  [ -f .env ] && grep -Eq '^[[:space:]]*(export[[:space:]]+)?GEMINI_API_KEY[[:space:]]*=[[:space:]]*["'\'']?[^"'\''[:space:]]' .env
}
if [ -n "$NO_GEMINI" ]; then DECIDER=none; BRIEF_PROVIDER=(--provider none); echo "decider  : argmax (--no-gemini); brief: template"
elif has_key; then DECIDER=gemini; BRIEF_PROVIDER=(); echo "decider  : Gemini dispatch gate (key found; falls back to argmax on any error)"
else DECIDER=none; BRIEF_PROVIDER=(--provider none)
  warn "no GEMINI_API_KEY (env or .env): dispatch gate = deterministic argmax, brief = template (still fully working)"
fi

if [ -n "$SLAM" ]; then
  "$PY" -c "import kiss_icp" 2>/dev/null || die "--slam needs kiss-icp in .venv (not importable); rerun without --slam"
  n=$(find dashboard/data/slam_cache -maxdepth 1 -name "seed$SEED-*.npz" 2>/dev/null | wc -l || true)
  echo "slam     : on (KISS-ICP; $n pile map(s) cached for seed $SEED; uncached piles add ~60-90 s each)"
fi

URL="http://localhost:$PORT/dashboard/"
if curl -sf -o /dev/null --max-time 3 "$URL"; then echo "server   : dev server up on :$PORT"
else warn "dev server NOT reachable on :$PORT. Start it in another terminal (from the repo root):"
  warn "    python3 -m http.server $PORT --bind 127.0.0.1 --directory \"$ROOT\""
  warn "The mission still runs and writes dashboard/data; reload the page once the server is up."
fi

# ---------------------------------------------------------------- (b) mission
OUT=dashboard/data
mkdir -p "$OUT/frames"
find "$OUT/frames" -maxdepth 1 -name 'state_*.json' -delete   # stale frames from an earlier (longer) run
ARGS=(-m sim.grid_mission --seed "$SEED" --radar dsp --acoustic "$ACOUSTIC" --decider "$DECIDER" --frames --out "$OUT")
[ -n "$START" ] && ARGS+=(--start-pile "$START")
[ -n "$REALTIME" ] && ARGS+=(--realtime --speed "$SPEED")
[ -n "$SLAM" ] && ARGS+=(--slam)
# Pile cap: until BL-3 (acoustic attribution) lands, seed 7 spends its second half on empty C3 while the faint
# voice at B1 is missed; capping at 3 pile visits ends the run on B2-R1 with an honest budget summary.
[ "$PILES" != 0 ] && ARGS+=(--piles "$PILES")
if [ -n "$DIRECT" ]; then RUN=(nice -n 5); else RUN=(scripts/heavy.sh)
  q=${XDG_RUNTIME_DIR:-/tmp}/larpverse-heavy
  if [ -n "$(ls "$q/queue" 2>/dev/null)" ] || ! flock -n "$q/slot1" true 2>/dev/null; then
    warn "heavy.sh queue is busy: the mission waits for it. On stage use: scripts/demo.sh --direct (or DEMO_STAGE=1)"; fi
fi

echo
bold "Dashboard: $URL      (open it now; it polls state.json every second)"
if [ -n "$REALTIME" ]; then
  awk -v s="$SPEED" 'BEGIN { if (s + 0 <= 0) exit 1 }' || die "--speed must be a positive number"
  echo "Mission  : realtime x$SPEED (~$(awk -v s="$SPEED" -v m="$([ "$PILES" = 0 ] && echo 745 || echo 400)" 'BEGIN { printf "%.1f", (m / s + 15) / 60 }') min with warm caches). Ctrl-C stops it."
else echo "Mission  : --fast (no pacing): the dashboard jumps; use scripts/record_dashboard.py --live for a paced replay"; fi
echo "Command  : ${RUN[*]} $PY ${ARGS[*]}"
echo
LOG=$(mktemp "${TMPDIR:-/tmp}/larpverse-demo.XXXXXX.log")
trap 'rm -f "$LOG"' EXIT
set +e
"${RUN[@]}" "$PY" "${ARGS[@]}" 2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}
set -e
if [ "$rc" -ne 0 ]; then
  [ "$rc" -eq 130 ] && die "interrupted (Ctrl-C). The dashboard keeps the last state written."
  warn "mission exited with status $rc."
  [ "$ACOUSTIC" = real ] && warn "If the acoustic step failed, rerun with the stub:  scripts/demo.sh --stub"
  warn "Replay a previous run instead:  .venv/bin/python scripts/record_dashboard.py --live  (then open ${URL}?state=data/replay/state.json&poll=250)"
  exit "$rc"
fi

# ---------------------------------------------------------------- (c) result + (d) handover brief
echo
bold "== Result =="
"$PY" - "$OUT/state.json" <<'EOF'
import json, sys
s = json.load(open(sys.argv[1]))
m = s.get("mission", {})
pings = [c for c in s.get("radar_candidates", []) if c.get("status") == "candidate"]
weak = [c for c in s.get("radar_candidates", []) if c.get("status") != "candidate"]
decs = s.get("decisions", [])
by = sorted({d.get("by", "?") for d in decs})
print(f"piles visited : {' -> '.join(m.get('visited', []))}  ({len(m.get('visited', []))} visits)")
print(f"pings         : {len(pings)}  " + ", ".join(f"{c['id']} ({c['xy'][0]:.1f}, {c['xy'][1]:.1f})" for c in pings)
      + (f"   [+{len(weak)} weak/unresolved]" if weak else ""))
print(f"decisions     : {len(decs)} dispatch-gate decisions, by {', '.join(by) or '-'}")
print(f"acoustic      : {m.get('acoustic_source')}   radar scans: {m.get('scans_used')}/{m.get('scan_budget')}   sim time: {s.get('t', 0):.0f} s")
print(f"frames        : dashboard/data/frames/ (for scripts/record_dashboard.py)")
EOF
echo
bold "== Rescuer handover brief =="
"$PY" -m agents.brief --state "$OUT/state.json" --out "$OUT/brief.md" "${BRIEF_PROVIDER[@]}" \
  || warn "brief generation failed (mission output is still valid in $OUT/state.json)"
echo
echo "Brief saved to $OUT/brief.md. Dashboard: $URL"
