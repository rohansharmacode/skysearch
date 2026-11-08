# Stop rule for console recordings (sourced by record_console.sh / record_switch.sh after the operator's click).
# Needs: FF (ffmpeg pid), R0 (recording start, epoch s), N0 (count of "Dispatch d1-d3" lines in ~/console.log at the click).
# First autonomous dispatch -> pile P at td. Stop at the later of td+30 and (PING at P)+10; if P yields no ping, stop
# 5 s after the drones leave P (next dispatch: the console's cross appears); always by td+75.
LOG=~/console.log
until [ $(grep -ac 'Dispatch d1-d3' $LOG) -gt $N0 ]; do kill -0 $FF 2>/dev/null || return 0; sleep 1; done
TD=$(date +%s)
DL=$(grep -a 'Dispatch d1-d3' $LOG | sed -n "$((N0 + 1))p")
P=$(echo "$DL" | grep -oE 'pile [A-Za-z0-9]+' | head -1 | cut -d' ' -f2)
echo "$(date +%T) +$((TD - R0))s autonomous dispatch to $P: $(echo "$DL" | cut -c1-170)"
TP=""; TL=""
while kill -0 $FF 2>/dev/null; do
  now=$(date +%s)
  if [ -z "$TP" ] && grep -aqE "PING [^ ]+ at pile $P\b" $LOG; then TP=$now; echo "$(date +%T) +$((now - R0))s radar lead at $P: $(grep -aE "PING [^ ]+ at pile $P\b" $LOG | head -1 | cut -c1-150)"; fi
  if [ -z "$TL" ] && [ $(grep -ac 'Dispatch d1-d3' $LOG) -gt $((N0 + 1)) ]; then TL=$now; echo "$(date +%T) +$((now - R0))s drones leave $P: $(grep -a 'Dispatch d1-d3' $LOG | sed -n "$((N0 + 2))p" | cut -c1-150)"; fi
  [ $now -ge $((TD + 75)) ] && { echo "cap td+75"; break; }
  if [ $now -ge $((TD + 30)) ]; then
    [ -n "$TP" ] && [ $now -ge $((TP + 10)) ] && { echo "stop: lead+10"; break; }
    [ -z "$TP" ] && [ -n "$TL" ] && [ $now -ge $((TL + 5)) ] && { echo "stop: left $P +5"; break; }
  fi
  sleep 1
done
