import asyncio, time
from playwright.async_api import async_playwright
JS = "(()=>{const v=document.querySelector('video');return v?[v.videoWidth,v.readyState,(v.getVideoPlaybackQuality?v.getVideoPlaybackQuality().totalVideoFrames:-1)]:null})()"
async def main():
    async with async_playwright() as p:
        ctx = await p.chromium.launch_persistent_context("/tmp/kiosk_profile", headless=False, no_viewport=True,
            ignore_https_errors=True, args=["--app=file:///home/ubuntu/live2/compose.html","--start-fullscreen",
            "--window-position=0,0","--window-size=1920,1080","--ignore-certificate-errors",
            "--autoplay-policy=no-user-gesture-required","--no-first-run","--hide-scrollbars"])
        await asyncio.sleep(2)
        pg = ctx.pages[0]
        if "compose" not in pg.url:
            await pg.goto("file:///home/ubuntu/live2/compose.html")
        print("[kiosk] inner", await pg.evaluate("[innerWidth,innerHeight,outerHeight]"), flush=True)
        t0 = time.time(); ready = False; last = 0
        while True:
            await asyncio.sleep(2)
            st = None
            try:
                f = pg.frames[1]
                st = await f.evaluate(JS)
            except Exception as e:
                st = None
            ok = bool(st and st[1] >= 3 and st[0] > 0)
            if ok and not ready:
                print(f"[kiosk] {time.strftime('%T')} STREAM VIDEO UP {st}", flush=True); ready = True
            if not ok and time.time() - last > 8:
                last = time.time()
                try:
                    await pg.evaluate("document.getElementById('v').src='https://localhost/viewer/?r='+Date.now()")
                except Exception: pass
                if ready: print(f"[kiosk] {time.strftime('%T')} stream lost, reloading", flush=True)
                ready = False
            if int(time.time() - t0) % 20 < 2:
                print(f"[kiosk] {time.strftime('%T')} {st}", flush=True)
asyncio.run(main())
