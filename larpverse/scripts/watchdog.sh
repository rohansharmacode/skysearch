#!/usr/bin/env bash
# Memory guard, run under the Monitor tool filtered to WARN|HARD. Below HARD it kills headless
# browsers (the dev server here is a tiny python http.server, so it is left alone).
WARN_MB=${WARN_MB:-1500}; HARD_MB=${HARD_MB:-900}
last=0
while true; do
  a=$(awk '/^MemAvailable:/{print int($2/1024)}' /proc/meminfo)
  now=$(date +%s)
  if [ "$a" -lt "$HARD_MB" ]; then
    pkill -f '[c]hrome-headless-shell'; pkill -f '[g]oogle-chrome.*--headless'
    echo "HARD: ${a} MB available, killed headless browsers"; last=$now
  elif [ "$a" -lt "$WARN_MB" ] && [ $((now-last)) -gt 120 ]; then
    echo "WARN: ${a} MB available; top: $(ps -eo rss,comm --sort=-rss | awk 'NR>1&&NR<5{printf "%s=%dMB ",$2,$1/1024}')"; last=$now
  fi
  sleep 4
done
