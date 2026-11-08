cd /home/ubuntu/live2
echo 1 > /tmp/lv_safepath; rm -f /tmp/lv_extra_args
export CONSOLE_URL="http://localhost:8765/isaac/hud/console.html?state=../../dashboard/data/state.json&view=http://localhost:5173/&cursor=1&only=$(python3 -c "import urllib.parse;print(urllib.parse.quote('collapse:7:day:Tainan collapsed building'))")"
for p in $(pgrep -f "python -u kiosk_console.py"); do kill $p; done; sleep 1
bash record_mode.sh
NAME=${NAME:-console_v9b} SITE=collapse SEED=7 LOOK=day CARD=none HOVER=T1a HOVER_S=4.5 PILE=T1b AFTER=13 TOUCHDOWN=1 AUDIO_MODEL=gemini-3.5-flash-lite bash record_take.sh
