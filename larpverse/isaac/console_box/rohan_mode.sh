# Hand the box to a remote operator (Rohan) over `brev port-forward ... 8765`: the Xvfb Chromium shows ONLY the Isaac
# WebRTC viewer (kiosk_viewer.py), scripts/mjpeg.py serves its centre 1280x1080 crop as /stream.mjpg (WebRTC media is
# UDP and does not pass a TCP forward). Undo with record_mode.sh. Recordings are paused while in this mode.
cd /home/ubuntu/live2; L=/home/ubuntu/lv/larpverse
for p in $(pgrep -f "python -u kiosk_console.py") $(pgrep -f "scripts/mjpeg.py"); do kill $p; done; sleep 2
if ! pgrep -f "Xvfb :99" >/dev/null; then nohup Xvfb :99 -screen 0 1920x1080x24 -nolisten tcp > xvfb.log 2>&1 & sleep 2; fi
if ! pgrep -f "kiosk_viewer.py" >/dev/null; then
  rm -rf /tmp/kiosk_viewer; DISPLAY=:99 setsid nohup /home/ubuntu/isaac/bin/python -u kiosk_viewer.py > viewer_kiosk.log 2>&1 < /dev/null &
fi
(cd $L && setsid nohup /home/ubuntu/isaac/bin/python scripts/mjpeg.py --display :99 --size 1920x1080 --crop 1280:1080:320:0 \
  --out 1280x1080 --fps 15 --quality 7 --port 8766 > ~/mjpeg.log 2>&1 < /dev/null &)
echo rohan-mode > /tmp/box_mode
