# Q&A watchdog (tmux "wd"): every 15 s, restart whatever died of serve.py (8765), public_gw.py (8770), cloudflared
# (tmux "pub"; a restart CHANGES the trycloudflare URL -> logged to ~/watchdog.log + ~/public_url.txt), and the Isaac
# runner (relaunch Rubble street held via /api/site) unless a recording lock or a site load is in progress.
L=/home/ubuntu/lv/larpverse; D=$L/dashboard/data; LOG=/home/ubuntu/watchdog.log
say() { echo "$(date -u +%FT%TZ) $*" >> $LOG; }
say "watchdog started"
while true; do
  if ! curl -s -o /dev/null -m 5 http://127.0.0.1:8765/api/site; then
    say "serve.py down -> restart"; (cd $L && setsid nohup python3 scripts/serve.py --port 8765 > ~/serve.log 2>&1 < /dev/null &); sleep 3
  fi
  if ! curl -s -o /dev/null -m 5 http://127.0.0.1:8770/dashboard/data/site.json; then
    say "gateway down -> restart"; (cd /home/ubuntu/live2 && setsid nohup python3 public_gw.py > ~/public_gw.log 2>&1 < /dev/null &); sleep 3
  fi
  if ! tmux has-session -t pub 2>/dev/null || ! pgrep -f "cloudflared.gh tunnel" >/dev/null; then
    say "cloudflared down -> restart (URL will change)"
    tmux kill-session -t pub 2>/dev/null
    tmux new-session -d -s pub "/home/ubuntu/bin/cloudflared.gh tunnel --no-autoupdate --url http://127.0.0.1:8770 2>&1 | tee /home/ubuntu/cloudflared_pub.log"
    sleep 15
    U=$(grep -oE 'https://[a-z0-9-]+\.trycloudflare\.com' /home/ubuntu/cloudflared_pub.log | tail -1); echo "$U" > /home/ubuntu/public_url.txt; say "new public URL $U"
  fi
  if [ -f /home/ubuntu/warm/pool_on ] && ! tmux has-session -t pool 2>/dev/null; then
    say "warm pool down -> restart"; tmux new-session -d -s pool "python3 /home/ubuntu/warm/warmpool.py 2>&1 | tee -a /home/ubuntu/warm/warmpool.out"
  fi
  st=$(python3 -c "import json;print(json.load(open('$D/site.json')).get('status',''))" 2>/dev/null)
  if [ ! -f /tmp/recording.lock ] && [ "$st" != loading ] && ! pgrep -f "isaac/run_mission.py" >/dev/null; then
    say "Isaac not running (site status '$st') -> relaunch Rubble street held"
    curl -s -X POST -d '{"site":"grid","seed":7,"look":"day"}' http://127.0.0.1:8765/api/site >/dev/null; sleep 60
  fi
  sleep 15
done
