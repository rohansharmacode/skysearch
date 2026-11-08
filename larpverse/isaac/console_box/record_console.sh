# Screen-record the operator console (Xvfb :99) for one site: load it, hold ~HOLD s, click PILE with the real
# mouse (kiosk_console.py), let the autonomous loop run until AFTER s past its first dispatch decision, stop.
# usage: NAME=site1_grid7 SITE=grid SEED=7 LOOK=day PILE=B1 bash record_console.sh
NAME=${NAME:?}; SITE=${SITE:-grid}; SEED=${SEED:-7}; LOOK=${LOOK:-day}; PILE=${PILE:?}
HOLD=${HOLD:-10}; AFTER=${AFTER:-15}; MAXT=${MAXT:-240}
C=/home/ubuntu/live2; D=/home/ubuntu/lv/larpverse/dashboard/data; cd $C
touch /tmp/recording.lock; echo '{"recording": true}' > $D/recording.json   # public clicks get 423; viewers see the take
trap "rm -f /tmp/recording.lock; echo {\\\"recording\\\": false} > $D/recording.json" EXIT
T0=$(date +%s)
[ "${SKIP_SITE:-0}" = 1 ] || curl -s -X POST -d "{\"site\":\"$SITE\",\"seed\":$SEED,\"look\":\"$LOOK\"}" http://127.0.0.1:8765/api/site; echo
sleep 8
until grep -q '"ready"' $D/site.json 2>/dev/null; do
  grep -q '"exited"' $D/site.json && { echo "site exited"; exit 1; }
  [ $(( $(date +%s) - T0 )) -gt 600 ] && { echo "timeout"; exit 1; }; sleep 2; done
echo "$(date +%T) ready after $(( $(date +%s) - T0 )) s"
until grep -q 'STREAM VIDEO UP' console_kiosk.log 2>/dev/null; do sleep 1; done
sleep 4                                            # the console has drawn the held state
rm -f $NAME.mkv
ffmpeg -nostdin -y -loglevel error -f x11grab -draw_mouse 0 -framerate 30 -video_size 1920x1080 -i :99.0 \
  -c:v libx264 -preset veryfast -crf 16 -pix_fmt yuv420p -t $MAXT $NAME.mkv > ${NAME}_ff.log 2>&1 &
FF=$!; R0=$(date +%s); echo "$(date +%T) recording"
sleep $HOLD
SEQ=$(date +%s%N); echo "{\"seq\": $SEQ, \"pile\": \"$PILE\"}" > /tmp/console_cmd.json
echo "$(date +%T) +$(( $(date +%s) - R0 ))s click $PILE"
N0=$(grep -ac 'Dispatch d1-d3' ~/console.log)
. /home/ubuntu/live2/wait_stop.sh                    # stop rule (first own dispatch, lead, cross, cap)
kill -INT $FF; wait $FF 2>/dev/null
echo "$(date +%T) stopped; $(ffprobe -v error -show_entries format=duration -of csv=p=0 $NAME.mkv) s, $(stat -c %s $NAME.mkv) B"
grep -aE 'Deploy d1|Decision \(|Dispatch d1|RE-PLAN|Gemini' ~/console.log | cut -c1-200 | tail -6
