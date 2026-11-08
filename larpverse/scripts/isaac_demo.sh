#!/usr/bin/env bash
# One-command operator-driven Isaac demo (run ON the GPU box, from anywhere).
#   INPUT : the operator draws an area on the dashboard -> POST /api/tasking -> dashboard/data/tasking.json
#           -> run_mission --live polls it every 1 s -> drones leave the dock for that area.
#   OUTPUT: run_mission --live writes dashboard/data/{state,live,hud,motion}.json -> scripts/serve.py -> dashboard.
#
#   L=/workspace/skysearch/larpverse ISAACLAB=/workspace/isaaclab bash $L/scripts/isaac_demo.sh
#   DRY=1 bash scripts/isaac_demo.sh          # same loop without a GPU (CPU sim; pair with animation/live-view.html)
# Env: PORT (8765), SEED (7), SITE (grid), STUB_ACOUSTIC=1 (rehearsal only), EXTRA="..." (more run_mission flags)
set -euo pipefail
L=${L:-$(cd "$(dirname "$0")/.." && pwd)}
PORT=${PORT:-8765}; SEED=${SEED:-7}; SITE=${SITE:-grid}
cd "$L"
PY=${PY:-$( [ -x .venv/bin/python ] && echo .venv/bin/python || echo python3 )}

echo "== 1/4 fresh tasking (no zones: drones wait at the dock for the operator)"
"$PY" -c "import json,time,os; os.makedirs('dashboard/data',exist_ok=True); json.dump({'schema':'larpverse.tasking/v1','zones':[],'source':'operator','revision':1,'updated_at':time.strftime('%Y-%m-%dT%H:%M:%S')},open('dashboard/data/tasking.json','w'))"

echo "== 2/4 ops server on :$PORT (dashboard + /api/tasking, /api/broadcast, /api/portal)"
if ! curl -sf -o /dev/null --max-time 2 "http://localhost:$PORT/dashboard/"; then
  nohup python3 scripts/serve.py --port "$PORT" > /tmp/larpverse_serve.log 2>&1 &
  sleep 1.5
fi
curl -sf -o /dev/null --max-time 3 "http://localhost:$PORT/api/tasking" && echo "   server OK"

START=""; grep -q -- '"--start"' isaac/run_mission.py && START="--start idle"
ACO=""; [ "${STUB_ACOUSTIC:-0}" = 1 ] && ACO="--acoustic stub --allow-stub-acoustic"

if [ "${DRY:-0}" = 1 ]; then
  echo "== 3/4 (dry) skipping acoustic preflight"
  echo "== 4/4 CPU live run. Open: http://localhost:$PORT/animation/live-view.html  +  http://localhost:$PORT/dashboard/?layout=take"
  exec "$PY" isaac/run_mission.py --dry-run --live $START --camera director-take --out dashboard/data --seed "$SEED" --site "$SITE" \
       ${ACO:---acoustic stub} ${EXTRA:-}
fi

echo "== 3/4 acoustic preflight (must PASS for a real take; STUB_ACOUSTIC=1 only for rehearsals)"
[ -n "$ACO" ] || "$PY" isaac/live.py --preflight

echo "== 4/4 Isaac live run. Client: <brev URL>/viewer (ONE tab) + http://localhost:$PORT/dashboard/?layout=take (port-forwarded)"
cd "${ISAACLAB:-/workspace/isaaclab}"
exec ./isaaclab.sh -p "$L/isaac/run_mission.py" --live $START --livestream 2 --viz kit --enable_cameras \
     --site "$SITE" --seed "$SEED" --photoreal 1 --veg 0 --camera director-take --hud overlay --overlay usd --radar dsp \
     --out "$L/dashboard/data" $ACO ${EXTRA:-}
