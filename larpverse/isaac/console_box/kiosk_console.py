"""Operator console in the box's Xvfb Chromium (DISPLAY=:99, 1920x1080), for screen recordings.

Loads isaac/hud/console.html (which embeds the Isaac WebRTC viewer from :5173), keeps the stream alive, and acts on
click commands so the recording shows a real mouse moving and clicking on the console's site map:

    echo '{"seq": 1, "pile": "B1"}' > /tmp/console_cmd.json     # smooth move to pileCentre('B1'), then click
    echo '{"seq": 2, "xy": [900, 400]}' > /tmp/console_cmd.json  # screen pixels
    echo '{"seq": 3, "reload": true}' > /tmp/console_cmd.json    # reload the console (after a site switch)
    echo '{"seq": 4, "at": "site:0", "park": false}' > ...        # start screen: click site card 0
    echo '{"seq": 5, "at": "C2", "click": false, "glide_s": 2.5}' # 3D map: hover over pile C2 (dots turn yellow)
    echo '{"seq": 6, "at": "B2", "park": false}'                  # 3D map: click pile B2 (sends the drones)

The click is a real DOM click: the page POSTs /api/goto itself.
"""
import asyncio
import json
import os
import time

from playwright.async_api import async_playwright

URL = os.environ.get("CONSOLE_URL", "http://localhost:8765/isaac/hud/console.html?state=../../dashboard/data/state.json"
                                    "&view=http://localhost:5173/&cursor=1")
CMD = "/tmp/console_cmd.json"
JS = "(()=>{const v=document.querySelector('video');if(!v)return null;return [v.videoWidth,v.readyState]})()"


def log(m):
    print(f"[console-kiosk] {time.strftime('%T')} {m}", flush=True)


async def main():
    async with async_playwright() as p:
        ctx = await p.chromium.launch_persistent_context(
            "/tmp/kiosk_console", headless=False, no_viewport=True, ignore_https_errors=True,
            args=[f"--app={URL}", "--start-fullscreen", "--window-position=0,0", "--window-size=1920,1080",
                  "--ignore-certificate-errors", "--autoplay-policy=no-user-gesture-required", "--no-first-run",
                  "--hide-scrollbars"])
        await asyncio.sleep(2)
        pg = ctx.pages[0]
        if "console.html" not in pg.url:
            await pg.goto(URL)
        await asyncio.sleep(1)
        log(f"page {pg.url} title={await pg.title()!r} pages={len(ctx.pages)}")
        ready, last_reload, seq, mx, my = False, 0.0, None, 960, 540
        try:
            seq = json.load(open(CMD)).get("seq")          # ignore a stale command from an earlier run
        except (OSError, ValueError):
            pass
        while True:
            await asyncio.sleep(0.25)
            vf = next((f for f in pg.frames if f != pg.main_frame and f.url.startswith("http://localhost:5173")), None)
            try:
                st = await vf.evaluate(JS) if vf else None
            except Exception:                              # noqa: BLE001
                st = None
            ok = bool(st and st[1] >= 3 and st[0] > 0)
            if ok and not ready:
                log(f"STREAM VIDEO UP {st}")
                ready = True
            if not ok and time.time() - last_reload > 10:
                last_reload = time.time()
                if ready:
                    log("stream lost, reloading the viewer iframe")
                try:
                    if vf:
                        await vf.goto("http://localhost:5173/?r=" + str(int(time.time())))
                except Exception:                          # noqa: BLE001
                    pass
                ready = False
            try:
                c = json.load(open(CMD))
            except (OSError, ValueError):
                continue
            if c.get("seq") == seq:
                continue
            seq = c.get("seq")
            try:
                if c.get("eval"):
                    log(f"eval -> {str(await pg.evaluate(c['eval']))[:600]}")
                    continue
                if c.get("reload"):
                    await pg.goto(URL)
                    log("console reloaded")
                    continue
                xy = c.get("xy")
                if c.get("site_chip") is not None:              # the console's site picker chip i
                    xy = await pg.evaluate(f"""(() => {{ const e = document.querySelector('.sc2[data-i="{int(c['site_chip'])}"]');
                        if (!e) return null; const r = e.getBoundingClientRect(); return [r.x + r.width / 2, r.y + r.height / 2]; }})()""")
                if c.get("pile"):
                    xy = await pg.evaluate(f"window.pileCentre && window.pileCentre({json.dumps(c['pile'])})")
                if c.get("at"):                                 # start screen card 'site:0..2' or a pile on the 3D map
                    xy = await pg.evaluate(f"window.pickerAt && window.pickerAt({json.dumps(c['at'])})")
                if not xy:
                    log(f"no screen position for {c}")
                    continue
                x, y = float(xy[0]), float(xy[1])

                async def glide(x0, y0, x1, y1, dur, n):     # eased, timed glide (the drawn ?cursor=1 pointer follows)
                    for i in range(1, n + 1):
                        u = i / n
                        e = u * u * (3 - 2 * u)
                        await pg.mouse.move(x0 + (x1 - x0) * e, y0 + (y1 - y0) * e)
                        await asyncio.sleep(dur / n)
                await glide(mx, my, x, y, float(c.get("glide_s", 1.5)), int(c.get("steps", 45)))
                if c.get("click") is False:                     # hover only (the 3D map lights up under the mouse)
                    mx, my = x, y
                    await asyncio.sleep(float(c.get("dwell_s", 0.5)))
                    log(f"hovered {c.get('at') or c.get('pile') or ''} at ({x:.0f}, {y:.0f})")
                    continue
                await asyncio.sleep(0.4)
                await pg.mouse.down()
                await asyncio.sleep(0.12)
                await pg.mouse.up()
                await asyncio.sleep(0.6)
                if c.get("park") is False:                      # stay put (the start screen / 3D map changes under it)
                    mx, my = x, y
                else:
                    ox, oy = c.get("park", [1600, 960])          # off the map: the right column's empty area
                    await glide(x, y, float(ox), float(oy), 1.0, 30)
                    mx, my = float(ox), float(oy)
                log(f"clicked {c.get('pile') or ''} at ({x:.0f}, {y:.0f})")
            except Exception as e:                         # noqa: BLE001
                log(f"command failed {c}: {e}")


asyncio.run(main())
