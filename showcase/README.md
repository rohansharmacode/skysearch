# Larpverse showcase video

A presentation-style product video (100 s, 1920×1080, 60 fps) in the same visual language as the Orbit site: midnight palette, Geist, spring word reveals.

- `public/` is a static page. Open `index.html` (served over HTTP) to play it live; space bar plays and pauses, `?t=40` jumps to a time.
- Every frame is a pure function of time (`render(t)` in `app.js`), so the video is rendered frame by frame with Playwright: `window.__seek(t)` draws the frame and seeks the Isaac clips.
- Isaac footage in `public/clips/` is cut from our own grid-site live take (procedural town, our own assets): `final/final_h.mp4` on the `larpverse-rehearsals` branch. Source times: launch 3.8 s, lidar 11.6 s, pods 16.5 s, live 26.4 s, voice 42.2 s, replan 49.4 s, a1 58.5 s. Each clip is the Isaac viewport (left 1280 px of the take) cropped to 1280×720 and encoded as VP9 with a keyframe every 10 frames so seeks are exact.
- Nothing from the collapse-site render or the NCKU Weiguan scan may be used here: that scan has no licence. Scene 1 states facts about the real 2016 collapse (sources in `larpverse/docs/IMPACT_EVIDENCE.md` §4 on `larpverse-sensing`) over our simulated town, labelled as such.
- Scene 6 follows the one-take live change: a teammate switches on a hidden voice near A1, the drones' microphones hear it, and the plan changes from A2 to A1. Numbers on screen must match the take the clips are cut from.
- Scene 7 numbers come from `larpverse/docs/EVAL_GRID.md` (20 seeds, acoustic stub = best case) and `RADAR_VALIDATION.md` (0.39 bpm MAE on real recordings).

Render: serve `public/` with a server that supports range requests (`npx http-server public -p 8088`), then run `render.js out.mp4 60 <t0> <t1>` in 3–4 parallel segments (e.g. 0–25, 25–50, 50–75, 75–100) and concat them with ffmpeg (`-f concat -c copy`).
