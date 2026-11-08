# live3: on-box capture of the live WebRTC stream (3d545d2)

What this is: the Isaac Lab route streaming over Kit WebRTC (`--livestream 2 --visualizer kit`), recorded on the L40S itself.
Xvfb :99 at 1920x1080 runs Chromium (Playwright, `--app` fullscreen). It loads `capture/compose.html`, which stacks the
web viewer (https://localhost/viewer/) and the judge HUD (http://localhost:8765/isaac/hud/judge.html?speed=4).
ffmpeg x11grab records it at 30 fps with the cursor hidden. No external sign-in was needed.

Flags: the take2 flags plus `--speed 4 --frames --kit_args "--/app/window/hideUi=true --/app/window/width=1920 --/app/window/height=1080"`.
Use `--visualizer kit`, not `--viz kit`: run_mission's parser reads `--viz` as an abbreviation of `--viz-scale` and exits.

- live3_720p.mp4: sim t 0 to 240 in 65.6 s (4x). The 1080p is on release larpverse-live3.
- stills/live3_tN.jpg: cut to match the HUD clock, within 1 s of sim t.
- timeline.csv: state file -> sim t -> second/frame in live3_720p.mp4 (calibrated on the HUD clock: video_s = 2.1 + t/4).
- live3_states.tgz: per-step state JSON with heightmap, pointcloud, slam, recon and landing_sites removed.
- live3_kiosk.log and live3_load.log: decoded stream frames, GPU and CPU load.
