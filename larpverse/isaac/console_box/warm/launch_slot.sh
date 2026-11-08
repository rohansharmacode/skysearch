# usage: SLOT=1 SITE=grid SEED=7 LOOK=day bash launch_slot.sh   -> a held, warm Isaac instance with its own ports + data dir
SLOT=${SLOT:?}; SITE=${SITE:-grid}; SEED=${SEED:-7}; LOOK=${LOOK:-day}
L=/home/ubuntu/lv/larpverse; OUT=$L/dashboard/slot$SLOT; mkdir -p $OUT/sites; cp $L/dashboard/data/sites/*.json $OUT/sites/ 2>/dev/null
rm -rf $OUT/state.json $OUT/frames $OUT/live.json $OUT/hud.json $OUT/live_cmd.json $OUT/motion.json $OUT/pc_tiles $OUT/pointcloud.bin $OUT/goto.json $OUT/console.json
. ~/isaac/bin/activate; export OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y LARPVERSE_WHISPER_DEVICE=cuda LV_OVERLAY_ONCE=1 LV_RPM_DIV=2 LV_PHYSX_DIV=2
export GEMINI_MODEL=gemini-4-argon GEMINI_AUDIO_MODEL=gemini-3.5-flash-lite LV_VAN_STYLE=police LARPVERSE_PEOPLE=3 LV_TOUCHDOWN=1 LV_SAFEPATH=1 LV_STANDBY=1
echo "{\"site\": \"$SITE\", \"seed\": $SEED, \"look\": \"$LOOK\", \"status\": \"loading\", \"slot\": $SLOT, \"wall\": $(date +%s.%N)}" > $OUT/site.json
(cd $L && python isaac/live.py --write-tasking default --out $OUT --seed $SEED --site $SITE 2>&1 | tail -1)
case "$SITE" in grid) SA="--site grid --grid-look 0 --town isaac/assets/town_kit";; weiguan) SA="--site weiguan";;
  collapse) SA="--site collapse --site-usd /home/ubuntu/cs3d/collapse_site/collapse_site.usda --no-scan-textures";; esac
case "$LOOK" in day) LA="--sky 600 --sun 2600 --fog-intensity 0.08 --fog-start 120";; *) LA="--sky 250 --sun 1400 --sun-elev 8 --fog-intensity 0.22 --fog-start 50";; esac
SIG=$((49100 + SLOT)); STR=$((47998 + SLOT))
cd ~/IsaacLab && exec ./isaaclab.sh -p $L/isaac/run_mission.py --live --livestream 2 --viz kit --enable_cameras \
  --kit_args "--/exts/omni.kit.livestream.app/primaryStream/signalPort=$SIG --/exts/omni.kit.livestream.app/primaryStream/streamPort=$STR" \
  $SA --seed $SEED --photoreal 1 --veg 0 --camera director-take --overlay usd --radar dsp --comms 0 \
  --live-pace clear --hud off --overlay-style simple --out $OUT --frames --drone-style blender --take-open wide --live-asr multi $LA \
  --delivery land --interactive --decider gemini
