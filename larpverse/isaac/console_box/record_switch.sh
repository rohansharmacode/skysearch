# Screen-record a site switch from the console (Xvfb :99): the kiosk clicks site chip CHIP on the current site's
# screen, the console shows the loading overlay, the new site comes up held, the kiosk clicks PILE, the autonomous
# loop runs until AFTER s past its first own dispatch. One continuous recording (the loading wait is cut later).
# usage: NAME=console_site2 CHIP=1 PILE=E bash record_switch.sh
NAME=${NAME:?}; CHIP=${CHIP:?}; PILE=${PILE:?}; HOLD=${HOLD:-6}; AFTER=${AFTER:-15}; MAXT=${MAXT:-420}
C=/home/ubuntu/live2; D=/home/ubuntu/lv/larpverse/dashboard/data; cd $C
touch /tmp/recording.lock; echo '{"recording": true}' > $D/recording.json   # public clicks get 423; viewers see the take
trap "rm -f /tmp/recording.lock; echo {\\\"recording\\\": false} > $D/recording.json" EXIT
rm -f $NAME.mkv
ffmpeg -nostdin -y -loglevel error -f x11grab -draw_mouse 0 -framerate 30 -video_size 1920x1080 -i :99.0 \
  -c:v libx264 -preset veryfast -crf 16 -pix_fmt yuv420p -t $MAXT $NAME.mkv > ${NAME}_ff.log 2>&1 &
FF=$!; R0=$(date +%s); el() { echo "+$(( $(date +%s) - R0 ))s"; }
echo "$(date +%T) recording"
sleep ${PRE:-2}
W0=$(python3 -c "import json;print(json.load(open('$D/site.json')).get('wall',0))" 2>/dev/null || echo 0)
echo "{\"seq\": $(date +%s%N), \"site_chip\": $CHIP}" > /tmp/console_cmd.json
echo "$(date +%T) $(el) chip $CHIP clicked"
until python3 -c "import json,sys;d=json.load(open('$D/site.json'));sys.exit(0 if float(d.get('wall',0))!=float($W0) else 1)" 2>/dev/null; do
  sleep 1; [ $(( $(date +%s) - R0 )) -gt 30 ] && { echo "chip click did not start a site load"; kill -INT $FF; exit 1; }; done
echo "$(date +%T) $(el) loading: $(cat $D/site.json)"
NUP=$(grep -c 'STREAM VIDEO UP' console_kiosk.log)
until grep -q '"ready"' $D/site.json; do
  grep -q '"exited"' $D/site.json && { echo "site exited"; kill -INT $FF; exit 1; }; sleep 2; done
echo "$(date +%T) $(el) ready"
until [ $(grep -c 'STREAM VIDEO UP' console_kiosk.log) -gt $NUP ] || [ $(( $(date +%s) - R0 )) -gt 400 ]; do sleep 1; done
echo "$(date +%T) $(el) stream back"
PID_=$(python3 -c "import json;ps=[p['id'] for p in json.load(open('$D/state.json'))['piles']];print(next((i for i in ps if i==\"$PILE\"), next((i for i in ps if i.startswith(\"$PILE\")), ps[0])))")
echo "$(date +%T) $(el) pile $PILE -> id '$PID_'"
sleep $HOLD
python3 -c "import json,sys,time;json.dump({'seq': time.time_ns(), 'pile': sys.argv[1]}, open('/tmp/console_cmd.json','w'))" "$PID_"
echo "$(date +%T) $(el) click $PILE"
N0=$(grep -ac 'Dispatch d1-d3' ~/console.log)
. /home/ubuntu/live2/wait_stop.sh                    # stop rule (first own dispatch, lead, cross, cap)
kill -INT $FF; wait $FF 2>/dev/null
echo "$(date +%T) stopped; $(ffprobe -v error -show_entries format=duration -of csv=p=0 $NAME.mkv) s, $(stat -c %s $NAME.mkv) B"
tail -3 console_kiosk.log
