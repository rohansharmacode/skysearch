# Operator console box side (isaac/hud/console.html): Isaac held on the pads until the first POST /api/goto.
# usage: SITE=grid|weiguan SEED=7 LOOK=day|dusk bash console_run.sh   (env: DECIDER=gemini|none, EXTRA="...",
#        KEEP_SERVE=1 when serve.py itself launched us via POST /api/site)
# Serves: http://127.0.0.1:8765/isaac/hud/console.html (+ /api/goto, /api/site, /stream.mjpg). Isaac log: ~/console.log
SITE=${SITE:-grid}; SEED=${SEED:-7}; LOOK=${LOOK:-day}
C=/home/ubuntu/live2; cd $C
. ~/isaac/bin/activate; export OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y LARPVERSE_WHISPER_DEVICE=cuda
export LV_OVERLAY_ONCE=1 LV_RPM_DIV=2 LV_PHYSX_DIV=2
L=~/lv/larpverse; D=$L/dashboard/data
mkdir -p $D
MYWALL=$(date +%s.%N)
echo "{\"site\": \"$SITE\", \"seed\": $SEED, \"look\": \"$LOOK\", \"status\": \"loading\", \"wall\": $MYWALL}" > $D/site.json.tmp \
  && mv $D/site.json.tmp $D/site.json
pkill -f "Xvfb :99"; pkill -f kiosk_take; pkill -f kiosk_console; pkill -f run_mission.py; pkill -f scripts/mjpeg.py; sleep 3
pulseaudio --check || pulseaudio --start --exit-idle-time=-1
rm -rf $D/tasking.json $D/state.json $D/frames $D/live.json $D/hud.json $D/live_cmd.json $D/motion.json \
  $D/pc_tiles $D/pointcloud.bin $D/goto.json $D/console.json
case "$SITE" in
  grid)    SITEARGS="--site grid --grid-look 0 --town isaac/assets/town_kit" ;;
  weiguan) SITEARGS="--site weiguan" ;;
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
nohup Xvfb :99 -screen 0 1920x1080x24 -nolisten tcp > xvfb.log 2>&1 &
sleep 2
rm -rf /tmp/kiosk_console; DISPLAY=:99 nohup python -u kiosk_console.py > console_kiosk.log 2>&1 &
(cd $L && setsid nohup python scripts/mjpeg.py --display :99 --size 1920x1080 --out 1280x720 --fps 20 --port 8766 \
  > ~/mjpeg.log 2>&1 < /dev/null &)
cd ~/IsaacLab && ./isaaclab.sh -p $L/isaac/run_mission.py --live --livestream 2 --viz kit --enable_cameras \
  $SITEARGS --seed $SEED --photoreal 1 --veg 0 --camera director-take --overlay usd --radar dsp --comms 0 \
  --live-pace clear --hud off --overlay-style simple --out $D --frames \
  --drone-style blender --take-open wide --live-asr multi $LOOKARGS \
  --delivery land --interactive --decider ${DECIDER:-gemini} $EXTRA
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
