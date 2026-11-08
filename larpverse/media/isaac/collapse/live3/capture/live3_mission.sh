L=/workspace/lv/larpverse; D=$L/dashboard/data; PY=/workspace/isaaclab/_isaac_sim/python.sh
export OMNI_KIT_ACCEPT_EULA=YES
pkill -f run_mission.py; pkill -f add_zone.py; sleep 3
mkdir -p $D; rm -rf $D/tasking.json $D/state.json $D/frames
cd $L
$PY isaac/tools/add_zone.py --out $D --init T1a --site collapse --seed 7 > /workspace/live3_zone_init.log 2>&1; echo "init exit=$?"
nohup $PY isaac/tools/add_zone.py --out $D --at 116 --sector T1b > /workspace/live3_zone_at.log 2>&1 &
cd /workspace/isaaclab && nohup ./isaaclab.sh -p $L/isaac/run_mission.py --site collapse --site-usd /workspace/lv/collapse_site/collapse_site.usda --seed 7 --photoreal 1 --overlay usd --show-truth --fx 1 --tasking auto --out $D --platform heavy_sar --delivery pod --penetration literature --acoustic stub --scan-ai /workspace/scan_ai --rt-quality 1 --camera story --viz-scale 4 --lidar-profile heavy --speed 4 --livestream 2 --visualizer kit --frames --kit_args "--/app/window/hideUi=true --/app/window/width=1920 --/app/window/height=1080" > /workspace/live3.log 2>&1 &
echo started; date
