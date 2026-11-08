// Burn the judge HUD into a recorded take (backup for the live capture, or a preview of the OBS scene).
//   NODE_PATH=$(npm root -g) node isaac/hud/burn.js STATES_DIR TIMELINE.csv VIDEO.mp4 OUT.mp4 [speed=4]
// STATES_DIR: run_mission --frames states (state_XXXX.json, heavy keys may be stripped). The HUD is fed every
// state in order (so its live-change memory works), one transparent 1920x1080 PNG per mission step; ffmpeg
// then shows each PNG for exactly the recorded frames whose timeline t equals that step's t.
const { chromium } = require('playwright');
const fs = require('fs'), path = require('path'), { execFileSync } = require('child_process');
const [STATES, TL, VIDEO, OUT, SPEED = '4'] = process.argv.slice(2);
const HUD = path.join(__dirname, 'judge.html');
(async () => {
  const tmp = fs.mkdtempSync(path.join(path.dirname(OUT), 'hudburn-'));
  const files = fs.readdirSync(STATES).filter(f => /^state_\d+\.json$/.test(f)).sort();
  const b = await chromium.launch();
  const p = await b.newPage({ viewport: { width: 1920, height: 1080 }, deviceScaleFactor: 1 });
  await p.route('**/state.json*', r => r.fulfill({ status: 404, body: '' }));
  await p.goto('file://' + HUD + '?speed=' + SPEED);
  const steps = [];
  for (const f of files) {
    const s = JSON.parse(fs.readFileSync(path.join(STATES, f)));
    delete s.heightmap; delete s.pointcloud; delete s.slam;
    await p.evaluate(s => { lastOk = Date.now(); render(s); }, s);
    const png = path.join(tmp, f.replace('.json', '.png'));
    await p.screenshot({ path: png, omitBackground: true });
    steps.push([+s.t, png]);
  }
  await b.close();
  // timeline: frame,t,...  -> run lengths of t
  const rows = fs.readFileSync(TL, 'utf8').trim().split('\n').slice(1).map(l => +l.split(',')[1]);
  const fps = 30, list = [];
  let i = 0;
  const pick = t => { let best = steps[0][1]; for (const [st, png] of steps) { if (st <= t + 1e-6) best = png; else break; } return best; };
  while (i < rows.length) {
    let j = i; while (j < rows.length && rows[j] === rows[i]) j++;
    list.push(`file '${pick(rows[i])}'\nduration ${((j - i) / fps).toFixed(5)}`);
    i = j;
  }
  list.push(`file '${pick(rows[rows.length - 1])}'`);
  fs.writeFileSync(path.join(tmp, 'list.txt'), list.join('\n') + '\n');
  const [W, H] = execFileSync('ffprobe', ['-v', 'error', '-select_streams', 'v:0', '-show_entries', 'stream=width,height',
    '-of', 'csv=p=0', VIDEO]).toString().trim().split(',').map(Number);
  execFileSync('ffmpeg', ['-y', '-loglevel', 'error', '-i', VIDEO, '-f', 'concat', '-safe', '0', '-i', path.join(tmp, 'list.txt'),
    '-filter_complex', `[1:v]scale=${W}:${H}:flags=lanczos[h];[0:v][h]overlay=0:0:shortest=1,format=yuv420p`,
    '-c:v', 'libx264', '-crf', '20', '-preset', 'medium', '-movflags', '+faststart', '-an', OUT], { stdio: 'inherit' });
  console.log('wrote', OUT, 'from', steps.length, 'HUD frames over', rows.length, 'video frames');
})();
