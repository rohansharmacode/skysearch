import asyncio, time, os
from playwright.async_api import async_playwright
JS = "(()=>{const v=document.querySelector('video');if(!v)return null;const s=v.srcObject;const a=document.querySelector('audio');const as=a&&a.srcObject;return [v.videoWidth,v.readyState,(v.getVideoPlaybackQuality?v.getVideoPlaybackQuality().totalVideoFrames:-1),s?s.getAudioTracks().length:-1,as?as.getAudioTracks().length:-1,a?a.muted:null]})()"
STATS = """(async()=>{const pcs=window.__pcs||[];let out=[];for(const pc of pcs){const r=await pc.getStats();r.forEach(x=>{if(x.type==='inbound-rtp')out.push([x.kind,x.bytesReceived,x.framesPerSecond||null,x.audioLevel||x.totalAudioEnergy||null])})}return out})()"""
HOOK = """(()=>{const O=window.RTCPeerConnection;window.__pcs=[];window.RTCPeerConnection=function(...a){const p=new O(...a);window.__pcs.push(p);return p};window.RTCPeerConnection.prototype=O.prototype;})()"""
FLAG = "/home/ubuntu/live2/pressY"
async def main():
    async with async_playwright() as p:
        ctx = await p.chromium.launch_persistent_context("/tmp/kiosk_viewer", headless=False, no_viewport=True,
            ignore_https_errors=True, args=["--app=file:///home/ubuntu/live2/compose_console.html","--start-fullscreen",
            "--window-position=0,0","--window-size=1920,1080","--ignore-certificate-errors",
            "--autoplay-policy=no-user-gesture-required","--no-first-run","--hide-scrollbars"])
        await ctx.add_init_script(HOOK)
        await asyncio.sleep(2)
        pg = ctx.pages[0]
        if "compose" not in pg.url:
            await pg.goto("file:///home/ubuntu/live2/compose_console.html")
        t0 = time.time(); ready = False; last = 0; pressed = False; lastlog = 0
        while True:
            await asyncio.sleep(1)
            vf = next((f for f in pg.frames if ":5173" in f.url), None)
            st = None
            try: st = await vf.evaluate(JS) if vf else None
            except Exception: st = None
            ok = bool(st and st[1] >= 3 and st[0] > 0)
            if ok and not ready:
                print(f"[kiosk] {time.strftime('%T')} STREAM VIDEO UP {st}", flush=True); ready = True
                try: await vf.evaluate("(()=>{const v=document.querySelector('video');v.muted=false;v.volume=1;v.play();const a=document.querySelector('audio');if(a){a.muted=false;a.volume=1;a.play();}})()")
                except Exception as e: print("[kiosk] unmute failed", e, flush=True)
            if not ok and time.time() - last > 8:
                last = time.time()
                try: await pg.evaluate("document.getElementById('v').src='http://localhost:5173/?r='+Date.now()")
                except Exception: pass
                if ready: print(f"[kiosk] {time.strftime('%T')} stream lost, reloading", flush=True)
                ready = False
            if ready and not pressed and os.path.exists(FLAG):
                try:
                    await vf.locator('video').focus()
                    await vf.locator('video').press('y')
                    print(f"[kiosk] {time.strftime('%T')} PRESSED Y in viewer", flush=True)
                except Exception as e:
                    print(f"[kiosk] key press failed {e}", flush=True)
                pressed = True
            if time.time() - lastlog > 10:
                lastlog = time.time()
                ss = None
                try: ss = await vf.evaluate(STATS) if vf else None
                except Exception as e: ss = repr(e)[:60]
                print(f"[kiosk] {time.strftime('%T')} {st} rtp={ss}", flush=True)
asyncio.run(main())
