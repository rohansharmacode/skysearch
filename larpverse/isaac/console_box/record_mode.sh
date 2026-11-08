# Back to recording: the console kiosk (kiosk_console.py) on Xvfb :99, MJPEG of the whole screen.
cd /home/ubuntu/live2; L=/home/ubuntu/lv/larpverse
for p in $(pgrep -f "python -u kiosk_viewer.py") $(pgrep -f "port 8766"); do kill $p; done; sleep 2
if ! pgrep -f "kiosk_console.py" >/dev/null; then
  rm -rf /tmp/kiosk_console; DISPLAY=:99 setsid nohup /home/ubuntu/isaac/bin/python -u kiosk_console.py > console_kiosk.log 2>&1 < /dev/null &
fi
(cd $L && setsid nohup /home/ubuntu/isaac/bin/python scripts/mjpeg.py --display :99 --size 1920x1080 --out 1280x720 --fps 20 \
  --port 8766 > ~/mjpeg.log 2>&1 < /dev/null &)
if ! pgrep -f "port 8767" >/dev/null; then          # /stream_full.mjpg: the whole screen (recording view for remote viewers)
  (cd $L && setsid nohup /home/ubuntu/isaac/bin/python scripts/mjpeg.py --display :99 --size 1920x1080 --out 1280x720 --fps 15 \
    --quality 7 --port 8767 > ~/mjpeg_full.log 2>&1 < /dev/null &)
fi
echo record-mode > /tmp/box_mode
