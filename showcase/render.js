// usage: node render.js out.mp4 fps t0 t1
const { chromium } = require('playwright'); const { spawn } = require('child_process');
(async () => {
  const [out, fpsS, t0S, t1S] = process.argv.slice(2); const fps = +fpsS, t0 = +t0S, t1 = +t1S;
  const b = await chromium.launch({ args: ['--disable-gpu-vsync'] }); const p = await b.newPage({ viewport: { width: 1920, height: 1080 } });
  const errs = []; p.on('pageerror', e => errs.push(String(e))); p.on('console', m => { if (m.type() === 'error') errs.push(m.text()); });
  await p.goto('http://localhost:8088/?render', { waitUntil: 'load' }); await p.evaluate(() => window.__ready);
  const ff = spawn('ffmpeg', ['-v', 'error', '-y', '-f', 'image2pipe', '-framerate', String(fps), '-c:v', 'mjpeg', '-i', '-', '-c:v', 'libx264', '-preset', 'medium', '-crf', '16', '-pix_fmt', 'yuv420p', '-r', String(fps), out], { stdio: ['pipe', 'inherit', 'inherit'] });
  const n0 = Math.round(t0 * fps), n1 = Math.round(t1 * fps);
  for (let n = n0; n < n1; n++) {
    await p.evaluate(t => window.__seek(t), n / fps);
    const buf = await p.screenshot({ type: 'jpeg', quality: 95 });
    if (!ff.stdin.write(buf)) await new Promise(r => ff.stdin.once('drain', r));
    if (n % 300 === 0) console.log(out, n, '/', n1);
  }
  ff.stdin.end(); await new Promise(r => ff.on('close', r));
  console.log('done', out, 'errors:', errs.length ? errs.slice(0, 5).join(' | ') : 'none'); await b.close();
})();
