# Final take v5: Rubble street (grid 7) loaded and held BEFORE recording; the kiosk drives the console from the start
# screen: card 0 -> 3D map -> hover B1 -> click A2; record until AFTER s past the first autonomous dispatch.
# usage: NAME=console_v5 TOUCHDOWN=1 bash record_v5.sh        (events with wall times -> ${NAME}_events.txt)
NAME=${NAME:-console_v5}; AFTER=${AFTER:-10}; MAXT=${MAXT:-300}
SITE=${SITE:-grid}; SEED=${SEED:-7}; LOOK=${LOOK:-day}; CARD=${CARD:-0}; HOVER=${HOVER:-B1}; PILE=${PILE:-A2}
C=/home/ubuntu/live2; D=/home/ubuntu/lv/larpverse/dashboard/data; cd $C
touch /tmp/recording.lock; echo '{"recording": true}' > $D/recording.json
trap "rm -f /tmp/recording.lock /tmp/lv_touchdown /tmp/lv_audio_model; echo '{\"recording\": false}' > $D/recording.json" EXIT
EV=${NAME}_events.txt; : > $EV
ev() { echo "$(date -u +%T.%3N) $*" | tee -a $EV; }
cmd() { python3 -c "import json,sys,time;d=json.loads(sys.argv[1]);d['seq']=time.time_ns();json.dump(d,open('/tmp/console_cmd.json','w'))" "$1"; }
[ "$(cat /tmp/box_mode 2>/dev/null)" = record-mode ] || bash record_mode.sh
echo ${TOUCHDOWN:-1} > /tmp/lv_touchdown
[ -n "$AUDIO_MODEL" ] && echo $AUDIO_MODEL > /tmp/lv_audio_model
ev "loading $SITE $SEED $LOOK (TOUCHDOWN=${TOUCHDOWN:-1}, audio model ${AUDIO_MODEL:-default})"
[ "${SKIP_SITE:-0}" = 1 ] || { curl -s -X POST -d "{\"site\":\"$SITE\",\"seed\":$SEED,\"look\":\"$LOOK\"}" http://127.0.0.1:8765/api/site >/dev/null; sleep 8; }
until grep -q '"ready"' $D/site.json; do grep -q '"exited"' $D/site.json && { ev "site exited"; exit 1; }; sleep 2; done
ev "site ready (held)"
until grep -q 'STREAM VIDEO UP' console_kiosk.log; do sleep 1; done
sleep 6                                              # let the viewer settle after the relaunch
cmd '{"reload": true}'; sleep 5                      # fresh console page: the start screen
rm -f $NAME.mkv
ffmpeg -nostdin -y -loglevel error -f x11grab -draw_mouse 0 -framerate 30 -video_size 1920x1080 -i :99.0 \
  -c:v libx264 -preset veryfast -crf 16 -pix_fmt yuv420p -t $MAXT $NAME.mkv > ${NAME}_ff.log 2>&1 &
FF=$!; ev "recording started (start screen)"
sleep 3
[ "$CARD" != none ] && { cmd "{\"at\": \"site:$CARD\", \"park\": false}"; ev "card $CARD glide+click sent"; }
sleep 4.5
cmd "{\"at\": \"$HOVER\", \"click\": false, \"glide_s\": 2.0, \"dwell_s\": 1.5}"; ev "hover $HOVER sent"
sleep ${HOVER_S:-4.5}
N0=$(grep -ac 'Dispatch d1-d3' ~/console.log)
cmd "{\"at\": \"$PILE\", \"park\": false, \"glide_s\": 1.5}"; ev "click $PILE sent"
until grep -aq 'first click' ~/console.log; do sleep 0.2; done; ev "isaac: first click received"
until [ $(grep -ac 'Dispatch d1-d3' ~/console.log) -gt $N0 ]; do
  kill -0 $FF 2>/dev/null || break; sleep 0.5; done
ev "first autonomous dispatch: $(grep -a 'Dispatch d1-d3' ~/console.log | sed -n "$((N0 + 1))p" | cut -c1-160)"
sleep $AFTER
kill -INT $FF; wait $FF 2>/dev/null
ev "stopped: $(ffprobe -v error -show_entries format=duration -of csv=p=0 $NAME.mkv) s, $(stat -c %s $NAME.mkv) B"
grep -aE 'first click|Deploy d1|phase |LIDAR FLASH|Decision \(|Dispatch d1|Gemini calls|listen [0-9]+ submitted' ~/console.log \
  | grep -v '\[ext' | cut -c1-200 > ${NAME}_isaac_events.txt
