# usage: bash run2.sh <name> ; expects /home/ubuntu/live2/<name>_mission.sh (container script)
N=${1:-live2}; cd /home/ubuntu/live2; rm -f $N.mp4 ${N}_kiosk.log ${N}_load.log
pkill -f "Xvfb :99"; pkill -f kiosk; docker exec vscode pkill -f run_mission.py; sleep 3
nohup Xvfb :99 -screen 0 1920x1080x24 -nolisten tcp > xvfb.log 2>&1 &
sleep 2
DISPLAY=:99 nohup /home/ubuntu/pwenv/bin/python -u kiosk3.py > ${N}_kiosk.log 2>&1 &
( while sleep 10; do echo "$(date +%T) gpu $(nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader) cpu $(top -bn1 | awk '/Cpu/{print 100-$8}')" ; done ) > ${N}_load.log 2>&1 &
LP=$!
nohup ffmpeg -nostdin -y -loglevel error -f x11grab -draw_mouse 0 -framerate 30 -video_size 1920x1080 -i :99.0 -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p -t 1500 $N.mp4 > ${N}_ff.log 2>&1 &
echo "$(date +%T) capture armed"
docker exec -i vscode bash -l < ${N}_mission.sh
echo "$(date +%T) mission launched"
for i in $(seq 1 300); do
  T=$(docker exec vscode sh -c "grep -a '\[isaac\] step' /workspace/$N.log | tail -1" | grep -oE 't=[0-9.]+' | cut -d= -f2)
  [ -n "$T" ] && awk -v t=$T 'BEGIN{exit !(t>=222)}' && { echo "$(date +%T) sim t=$T, stopping"; break; }
  sleep 3
done
sleep 3; pkill -INT -f "x11grab"; sleep 6
docker exec vscode grep -a "\[isaac\] step" /workspace/$N.log | head -3 | cut -c1-100
docker exec vscode sh -c "grep -aE '\[isaac\] step|first step|stream' /workspace/$N.log | head -2" | cut -c1-120
docker exec vscode pkill -f run_mission.py; docker exec vscode pkill -f add_zone.py
pkill -f kiosk3.py; pkill -f "Xvfb :99"; kill $LP
ls -la $N.mp4; echo CAPTURE_DONE
