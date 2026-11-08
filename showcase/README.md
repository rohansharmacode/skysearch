# Larpverse showcase video

A presentation-style product video (about 97 s, 1920×1080, 60 fps) in the same visual language as the Orbit site: midnight palette, Geist, spring word reveals.

- `public/` is a static page. Open `index.html` (served over HTTP) to play it live; space bar plays and pauses, `?t=40` jumps to a time.
- Every frame is a pure function of time (`render(t)` in `app.js`), so the video is rendered frame by frame with Playwright: `window.__seek(t)` draws the frame and seeks the Isaac clips.
- Isaac footage in `public/clips/` is cut from `larpverse_collapse_pathtraced_60s.mp4` (burned-in captions cropped off).

Render: serve `public/` with a server that supports range requests (`npx http-server public -p 8088`), then run `render.js out.mp4 60 0 97` (see the thread notes) and concat segments with ffmpeg.
