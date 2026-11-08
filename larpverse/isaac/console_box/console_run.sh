# Operator console box side (isaac/hud/console.html): Isaac held on the pads until the first POST /api/goto.
# usage: SITE=grid|weiguan SEED=7 LOOK=day|dusk bash console_run.sh   (env: DECIDER=gemini|none, EXTRA="...",
#        KEEP_SERVE=1 when serve.py itself launched us via POST /api/site)
# Serves: http://127.0.0.1:8765/isaac/hud/console.html (+ /api/goto, /api/site, /stream.mjpg). Isaac log: ~/console.log
SITE=${SITE:-grid}; SEED=${SEED:-7}; LOOK=${LOOK:-day}
C=/home/ubuntu/live2; cd $C
. ~/isaac/bin/activate; export OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y LARPVERSE_WHISPER_DEVICE=cuda
export LV_OVERLAY_ONCE=1 LV_RPM_DIV=2 LV_PHYSX_DIV=2
export GEMINI_MODEL=${GEMINI_MODEL:-gemini-4-argon}
export GEMINI_AUDIO_MODEL=${GEMINI_AUDIO_MODEL:-$(cat /tmp/lv_audio_model 2>/dev/null || echo gemini-3.5-flash)}   # drones: audio analysis + transcription check   # live runs only (not a code default: replay caches are keyed by model)
export LV_VAN_STYLE=${VAN_STYLE:-police}            # generic police van; the drones launch from its roof
export LV_TOUCHDOWN=${TOUCHDOWN:-$(cat /tmp/lv_touchdown 2>/dev/null || echo 0)}   # opt-in landing settle (record_v5.sh)
export LV_AO=$(cat /tmp/lv_ao 2>/dev/null || echo 0)
export LV_SAFEPATH=$(cat /tmp/lv_safepath 2>/dev/null || echo 0)
export LV_EXTRA_ARGS_FILE=/tmp/lv_extra_args
[ -f /tmp/lv_env ] && . /tmp/lv_env                 # opt-in trims for a take (LV_CROP, LV_NODEBRIS, ...)
export LARPVERSE_PEOPLE=${PEOPLE:-3}          # simulated people, each under a different pile (sim/scenario.py)
L=~/lv/larpverse; D=$L/dashboard/data
mkdir -p $D
MYWALL=$(date +%s.%N)
echo "{\"site\": \"$SITE\", \"seed\": $SEED, \"look\": \"$LOOK\", \"status\": \"loading\", \"wall\": $MYWALL}" > $D/site.json.tmp \
  && mv $D/site.json.tmp $D/site.json
if [ "${KEEP_SERVE:-0}" = 1 ]; then                # site switch from the console: keep the display, kiosk and recorder
  pkill -f run_mission.py; sleep 3
else
  pkill -f "Xvfb :99"; pkill -f kiosk_take; pkill -f kiosk_console; pkill -f run_mission.py; pkill -f scripts/mjpeg.py; sleep 3
fi
pulseaudio --check || pulseaudio --start --exit-idle-time=-1
rm -rf $D/tasking.json $D/state.json $D/frames $D/live.json $D/hud.json $D/live_cmd.json $D/motion.json \
  $D/pc_tiles $D/pointcloud.bin $D/goto.json $D/console.json
case "$SITE" in
  grid)    SITEARGS="--site grid --grid-look 0 --town isaac/assets/town_kit" ;;
  weiguan) SITEARGS="--site weiguan" ;;
  collapse) SITEARGS="--site collapse --site-usd /home/ubuntu/cs3d/collapse_site/collapse_site.usda --no-scan-textures" ;;
  *) echo "unknown SITE $SITE"; exit 2 ;;
esac
case "$LOOK" in
  day)  LOOKARGS="--sky 600 --sun 2600 --fog-intensity 0.08 --fog-start 120" ;;
  dusk) LOOKARGS="--sky 250 --sun 1400 --sun-elev 8 --fog-intensity 0.22 --fog-start 50" ;;
  *) echo "unknown LOOK $LOOK"; exit 2 ;;
esac
(cd $L && python isaac/live.py --write-tasking default --out $D --seed $SEED --site $SITE 2>&1 | tail -1)
if [ "${KEEP_SERVE:-0}" != 1 ]; then
  pkill -f "scripts/serve.py"; sleep 1; (cd $L && setsid nohup python scripts/serve.py --port 8765 > ~/serve.log 2>&1 < /dev/null &)
fi
if ! pgrep -f "Xvfb :99" >/dev/null; then
  nohup Xvfb :99 -screen 0 1920x1080x24 -nolisten tcp > xvfb.log 2>&1 &
  sleep 2
fi
if ! pgrep -f "kiosk_console.py|kiosk_viewer.py" >/dev/null; then
  rm -rf /tmp/kiosk_console; DISPLAY=:99 setsid nohup python -u kiosk_console.py > console_kiosk.log 2>&1 < /dev/null &
fi
if ! pgrep -f "scripts/mjpeg.py" >/dev/null; then
  (cd $L && setsid nohup python scripts/mjpeg.py --display :99 --size 1920x1080 --out 1280x720 --fps 20 --port 8766 \
    > ~/mjpeg.log 2>&1 < /dev/null &)
fi
cd ~/IsaacLab && ./isaaclab.sh -p $L/isaac/run_mission.py --live --livestream 2 --viz kit --enable_cameras \
  $SITEARGS --seed $SEED --photoreal 1 --veg 0 --camera director-take --overlay usd --radar dsp --comms 0 \
  --live-pace clear --hud off --overlay-style simple --out $D --frames \
  --drone-style blender --take-open wide --live-asr multi $LOOKARGS \
  --delivery land --interactive --decider ${DECIDER:-gemini} $EXTRA $(cat /tmp/lv_extra_args 2>/dev/null)
RC=$?
python3 - "$D/site.json" "$RC" "$MYWALL" <<'PY'
import json, sys, time
p, rc, mine = sys.argv[1], int(sys.argv[2]), float(sys.argv[3])
try:
    d = json.load(open(p))
except Exception:
    d = {}
if abs(float(d.get("wall") or 0) - mine) < 1e-3:   # only our own launch's status (a newer one rewrote it)
    d.update(status="exited", exit_code=rc, status_wall=time.time())
    json.dump(d, open(p + ".tmp", "w")); __import__("os").replace(p + ".tmp", p)
PY
