// Larpverse operator console. Reads data/state.json (contracts/INTERFACES.md, v1 + v1.1 grid extensions) every second.
// Works with the single-pile v1 state (sim/coordinator.py) and the 3x3 grid state (sim/grid_mission.py).
import { demoState, demoCloud } from './demo.js';

const STATE_URL = 'data/state.json';
const POLL_MS = 1000;
const COL = { detect: '#ff4545', cone: '#ffe14a', coord: '#6fd36f', radar: '#ff9f43', teal: '#22c3d6', good: '#4fd18b', bad: '#ff6464', fg: '#e3eaef', muted: '#8a9caa', grid: 'rgba(150,180,200,0.10)' };
const STATUS = {
  unvisited: ['unvisited', COL.muted], surveying: ['surveying', COL.coord], scanning: ['radar scan', COL.radar],
  ping: ['ping', COL.detect], lead: ['acoustic lead', COL.teal], dismissed_nonhuman: ['non-human', '#6d7d89'],
  no_signal_not_cleared: ['not cleared', COL.muted],
};
const $ = id => document.getElementById(id);
const f1 = v => (v == null ? '—' : (+v).toFixed(1));
const f2 = v => (v == null ? '—' : (+v).toFixed(2));
const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

let S = null;                 // current state
let cloud = null, cloudKey = null;
let live = false;
const layers = { basemap: true, belief: true, rings: false, acoustic: true, sites: true, piles: true };
const isGrid = () => Array.isArray(S?.piles) && S.piles.length > 0;

// ---------------- data ----------------
async function poll() {
  try {
    const r = await fetch(STATE_URL + '?_=' + Date.now(), { cache: 'no-store' });
    if (!r.ok) throw new Error(r.status);
    const s = await r.json();
    if (s.schema !== 'larpverse.state/v1') throw new Error('schema ' + s.schema);
    live = true; setState(s);
  } catch (e) {
    if (!S || !live) { live = false; if (!S) setState(demoState()); }
    $('conn').textContent = live ? 'connection lost' : 'demo data'; $('conn').className = 'conn ' + (live ? '' : 'demo');
    return;
  }
  $('conn').textContent = 'live · state.json'; $('conn').className = 'conn live';
}
async function ensureCloud() {
  const pc = S.pointcloud || {}; const key = pc.url ? pc.url + ':' + pc.count : 'demo:' + (isGrid() ? 'grid' : 'pile');
  if (key === cloudKey) return; cloudKey = key;
  if (pc.url) {
    try { const r = await fetch('data/' + pc.url + '?_=' + Date.now(), { cache: 'no-store' }); if (!r.ok) throw new Error(r.status); cloud = new Float32Array(await r.arrayBuffer()); return; }
    catch { /* fall through */ }
  }
  cloud = demoCloud();
}
let hmKey = null;
function setState(s) {
  const first = !S || (S.demo && !s.demo) || (!!S.piles !== !!s.piles);
  const hk = s.heightmap ? `${s.heightmap.origin}|${s.heightmap.w}x${s.heightmap.h}|${s.heightmap.z.length}|${s.heightmap.z[(s.heightmap.z.length / 3) | 0]}` : null;
  if (hk !== hmKey) { baseImg = null; hmKey = hk; }
  S = s; beliefImg = null;
  ensureCloud();
  if (first) fit();
  renderSide(); draw();
  const m = !pingLinked && location.hash.match(/^#ping=(.+)$/);   // deep link: dashboard/#ping=A2-R1
  if (m) { const c = (S.radar_candidates || []).find(c => c.id === decodeURIComponent(m[1])); if (c) { pingLinked = true; openDetail(c); } }
}
let pingLinked = false;

// ---------------- map ----------------
const cv = $('map'), ctx = cv.getContext('2d');
const view = { cx: 0, cy: 0, scale: 14 };      // site metres at canvas centre; px per metre
let W = 0, Hh = 0, dpr = 1;
const toPx = (x, y) => [W / 2 + (x - view.cx) * view.scale, Hh / 2 - (y - view.cy) * view.scale];
const toM = (px, py) => [view.cx + (px - W / 2) / view.scale, view.cy - (py - Hh / 2) / view.scale];
function resize() { const r = cv.getBoundingClientRect(); dpr = Math.min(devicePixelRatio || 1, 2); W = r.width; Hh = r.height; cv.width = W * dpr; cv.height = Hh * dpr; draw(); }
function fit() {
  const b = S?.site?.bounds || [-25, -25, 25, 25], r = cv.getBoundingClientRect();
  view.cx = (b[0] + b[2]) / 2; view.cy = (b[1] + b[3]) / 2;
  view.scale = isGrid() ? Math.min((r.width - 24) / (b[2] - b[0]), (r.height - 120) / (b[3] - b[1]))
    : Math.min(r.width / (b[2] - b[0]), r.height / (b[3] - b[1])) * 1.25;
  if (isGrid()) view.cy -= 4 / view.scale;
  draw();
}

let baseImg = null, beliefImg = null;
function gridImage(g, fn) {
  const c = document.createElement('canvas'); c.width = g.w; c.height = g.h;
  const x = c.getContext('2d'), im = x.createImageData(g.w, g.h);
  for (let j = 0; j < g.h; j++) for (let i = 0; i < g.w; i++) {
    const o = ((g.h - 1 - j) * g.w + i) * 4, [r, gg, b, a] = fn(i, j); // flip: row j=0 is south
    im.data[o] = r; im.data[o + 1] = gg; im.data[o + 2] = b; im.data[o + 3] = a;
  }
  x.putImageData(im, 0, 0); return c;
}
function makeBase() {
  const g = S.heightmap; if (!g) return null;
  const z = (i, j) => g.z[Math.min(g.h - 1, Math.max(0, j)) * g.w + Math.min(g.w - 1, Math.max(0, i))];
  let zmax = 0.5; for (const v of g.z) zmax = Math.max(zmax, v);
  return gridImage(g, (i, j) => {
    const dx = (z(i + 1, j) - z(i - 1, j)) / (2 * g.res), dy = (z(i, j + 1) - z(i, j - 1)) / (2 * g.res);
    const shade = Math.max(0, Math.min(1, 0.62 + (-dx * 0.55 + dy * 0.55) / Math.sqrt(1 + dx * dx + dy * dy)));
    const e = z(i, j) / zmax, k = 26 + 70 * shade + 40 * e;
    return [k * 1.05, k, k * 0.92, 255];
  });
}
function makeBelief() {
  const g = S.belief; if (!g || !g.p) return null;
  let mx = 0; for (const v of g.p) mx = Math.max(mx, v);
  if (!(mx > 0)) return null;
  return gridImage(g, (i, j) => { const u = Math.pow(g.p[j * g.w + i] / mx, 0.6); return [255, 150 - 90 * u, 60, 200 * u]; });
}
function drawGridImage(img, g) {
  const [x0, y1] = toPx(g.origin[0], g.origin[1] + g.h * g.res); // cell centres at origin + (i + 0.5) * res
  const s = g.res * view.scale; ctx.imageSmoothingEnabled = true; ctx.drawImage(img, x0, y1, g.w * s, g.h * s);
}
function ellipse(c, k = 2.45) {
  const [[a, b], [, d]] = c.cov, tr = a + d, det = a * d - b * b, disc = Math.sqrt(Math.max(0, tr * tr / 4 - det));
  const l1 = tr / 2 + disc, l2 = Math.max(1e-4, tr / 2 - disc), ang = 0.5 * Math.atan2(2 * b, a - d);
  return { rx: k * Math.sqrt(l1), ry: k * Math.sqrt(l2), ang };
}
const pileById = id => (S.piles || []).find(p => p.id === id);

function cone(origin, bearing, width, len, fill, stroke, lw = 1.2) {
  const [ox, oy] = toPx(...origin), L = len * view.scale, b0 = (bearing - width / 2) * Math.PI / 180, b1 = (bearing + width / 2) * Math.PI / 180;
  ctx.save(); ctx.beginPath(); ctx.moveTo(ox, oy); ctx.arc(ox, oy, L, -b1, -b0); ctx.closePath(); ctx.fillStyle = fill; ctx.fill();
  ctx.strokeStyle = stroke; ctx.lineWidth = lw; ctx.beginPath();
  ctx.moveTo(ox + Math.cos(-b0) * L, oy + Math.sin(-b0) * L); ctx.lineTo(ox, oy); ctx.lineTo(ox + Math.cos(-b1) * L, oy + Math.sin(-b1) * L); ctx.stroke(); ctx.restore();
}
function arrow(a, b, color, alpha, text) {
  const [x0, y0] = toPx(...a), [x1, y1] = toPx(...b), ang = Math.atan2(y1 - y0, x1 - x0), r = 9 * view.scale;
  const sx = x0 + Math.cos(ang) * r, sy = y0 + Math.sin(ang) * r, ex = x1 - Math.cos(ang) * r, ey = y1 - Math.sin(ang) * r;
  ctx.save(); ctx.globalAlpha = alpha; ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2.5; ctx.setLineDash([9, 6]);
  ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(ex, ey); ctx.stroke(); ctx.setLineDash([]);
  ctx.beginPath(); ctx.moveTo(ex, ey); ctx.lineTo(ex - 13 * Math.cos(ang - 0.42), ey - 13 * Math.sin(ang - 0.42)); ctx.lineTo(ex - 13 * Math.cos(ang + 0.42), ey - 13 * Math.sin(ang + 0.42)); ctx.closePath(); ctx.fill();
  if (text) label(text, ex + 12, ey + 14, color);
  ctx.restore();
}

let hoverId = null;
function draw(now = performance.now()) {
  if (!S || !W) return;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, W, Hh);
  if (layers.basemap && S.heightmap) { baseImg ||= makeBase(); if (baseImg) drawGridImage(baseImg, S.heightmap); }
  // metric grid, 5 m (10 m on the grid site)
  const step = isGrid() ? 10 : 5;
  ctx.strokeStyle = COL.grid; ctx.lineWidth = 1; ctx.beginPath();
  const [mx0, my1] = toM(0, 0), [mx1, my0] = toM(W, Hh);
  for (let x = Math.ceil(mx0 / step) * step; x <= mx1; x += step) { const [px] = toPx(x, 0); ctx.moveTo(px, 0); ctx.lineTo(px, Hh); }
  for (let y = Math.ceil(my0 / step) * step; y <= my1; y += step) { const [, py] = toPx(0, y); ctx.moveTo(0, py); ctx.lineTo(W, py); }
  ctx.stroke();
  if (layers.belief && S.belief) { beliefImg ||= makeBelief(); if (beliefImg) { ctx.globalAlpha = 0.85; drawGridImage(beliefImg, S.belief); ctx.globalAlpha = 1; } }

  if (isGrid()) drawGridLayer(now);

  // acoustic cones
  if (layers.acoustic) {
    const clips = S.acoustic_clips;
    if (Array.isArray(clips) && clips.length) {
      const lastListen = Math.max(...clips.map(c => c.listen ?? 0));
      for (const c of clips) {
        if (c.bearing_deg == null || !c.origin) continue;
        const p = pileById(c.pile_id), d = p ? Math.hypot(p.xy[0] - c.origin[0], p.xy[1] - c.origin[1]) + 5 : 30;
        const human = (c.scores?.human ?? 0) >= 0.4, recent = (c.listen ?? 0) === lastListen;
        if (!recent) continue;                       // older listens: see the rail; keep the map readable
        if (human) cone(c.origin, c.bearing_deg, c.width_deg || 20, d, 'rgba(255,225,74,0.15)', 'rgba(255,225,74,0.85)');
        else cone(c.origin, c.bearing_deg, c.width_deg || 20, d * 0.55, 'rgba(150,170,185,0.05)', 'rgba(150,170,185,0.35)', 1);
      }
    } else for (const a of S.acoustic || []) cone(a.origin, a.bearing_deg, a.width_deg, 60, `rgba(255,225,74,${0.08 + 0.1 * (a.confidence ?? 0.5)})`, 'rgba(255,225,74,0.75)');
  }
  // range rings
  if (layers.rings) for (const s of S.scans || []) (s.range_candidates || []).forEach((r) => {
    const [x, y] = toPx(s.sensor_pose[0], s.sensor_pose[1]); const w = (s.used_weight ?? s.quality ?? 1);
    ctx.strokeStyle = `rgba(255,159,67,${0.25 + 0.6 * w})`; ctx.setLineDash(w < 0.5 ? [6, 5] : []); ctx.lineWidth = 1.3;
    ctx.beginPath(); ctx.arc(x, y, r * view.scale, 0, Math.PI * 2); ctx.stroke(); ctx.setLineDash([]);
  });
  // landing sites
  if (layers.sites) for (const l of S.landing_sites || []) {
    if (l.status === 'rejected' && !l.slope_deg) continue;
    const [x, y] = toPx(...l.xy), s = l.status === 'used' ? 9 : 5;
    ctx.strokeStyle = l.status === 'rejected' ? COL.bad : COL.good; ctx.lineWidth = l.status === 'used' ? 2 : 1.2; ctx.globalAlpha = l.status === 'candidate' ? 0.6 : 1;
    ctx.strokeRect(x - s / 2, y - s / 2, s, s); ctx.globalAlpha = 1;
    if (l.status === 'used' && view.scale > 10) label(l.id, x + 8, y + 12, COL.muted);
  }
  // next scan
  if (S.next_scan) {
    const site = (S.landing_sites || []).find(l => l.id === S.next_scan.site_id), dr = (S.drones || []).find(d => d.id === S.next_scan.drone_id);
    if (site) {
      const [x, y] = toPx(...site.xy); ctx.strokeStyle = COL.coord; ctx.setLineDash([5, 4]); ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(x, y, 12, 0, Math.PI * 2); ctx.stroke();
      if (dr) { const [dx, dy] = toPx(dr.pose[0], dr.pose[1]); ctx.beginPath(); ctx.moveTo(dx, dy); ctx.lineTo(x, y); ctx.stroke(); }
      ctx.setLineDash([]); label('next scan', x + 14, y - 10, COL.coord);
    }
  }
  // radar candidates (pings)
  const pulse = (now / 1200) % 1;
  for (const c of S.radar_candidates || []) {
    const [x, y] = toPx(...c.xy), weak = c.status !== 'candidate', moving = c.kind === 'moving';
    if (c.cov) {
      const e = ellipse(c); ctx.save(); ctx.translate(x, y); ctx.rotate(-e.ang);
      ctx.beginPath(); ctx.ellipse(0, 0, Math.max(3, e.rx * view.scale), Math.max(3, e.ry * view.scale), 0, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(255,69,69,0.14)'; ctx.fill(); ctx.strokeStyle = weak ? COL.radar : COL.detect; ctx.setLineDash(weak ? [5, 4] : []); ctx.lineWidth = 1.6; ctx.stroke(); ctx.restore(); ctx.setLineDash([]);
    }
    if (!weak) { ctx.strokeStyle = `rgba(255,69,69,${1 - pulse})`; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(x, y, 7 + pulse * 22, 0, Math.PI * 2); ctx.stroke(); }
    ctx.fillStyle = moving || weak ? COL.radar : COL.detect; ctx.beginPath();
    if (moving) { ctx.moveTo(x, y - 8); ctx.lineTo(x + 8, y); ctx.lineTo(x, y + 8); ctx.lineTo(x - 8, y); ctx.closePath(); } else ctx.arc(x, y, hoverId === c.id ? 9 : 7, 0, Math.PI * 2);
    ctx.fill(); ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5; ctx.stroke();
    label(`${c.id} · p ${f2(c.p_mass)}`, x + 12, y - 10, '#ffd0d0', 'rgba(60,8,8,0.85)');
  }
  // drones
  for (const d of S.drones || []) {
    const [x, y] = toPx(d.pose[0], d.pose[1]), yaw = (d.pose[3] ?? 0) * Math.PI / 180;
    ctx.save(); ctx.translate(x, y); ctx.rotate(-yaw); ctx.beginPath(); ctx.moveTo(10, 0); ctx.lineTo(-6, 6); ctx.lineTo(-3, 0); ctx.lineTo(-6, -6); ctx.closePath();
    ctx.fillStyle = d.state === 'listening' ? COL.cone : COL.fg; ctx.fill(); ctx.strokeStyle = '#0b1016'; ctx.lineWidth = 1.5; ctx.stroke(); ctx.restore();
    label(`${d.id} · ${d.state}`, x + 10, y + 16, COL.fg);
  }
  scaleBar();
}

function drawGridLayer(now) {
  // staging area
  const st = S.staging;
  if (st) {
    const [x0, y0] = toPx(st.xy[0] - 7, st.xy[1] + 4), [x1, y1] = toPx(st.xy[0] + 7, st.xy[1] - 4);
    ctx.save(); ctx.fillStyle = 'rgba(90,169,255,0.08)'; ctx.strokeStyle = 'rgba(90,169,255,0.7)'; ctx.setLineDash([4, 3]); ctx.lineWidth = 1.2;
    ctx.fillRect(x0, y0, x1 - x0, y1 - y0); ctx.strokeRect(x0, y0, x1 - x0, y1 - y0); ctx.setLineDash([]);
    for (const p of st.pads || []) { const [px, py] = toPx(...p); ctx.strokeStyle = 'rgba(90,169,255,0.9)'; ctx.strokeRect(px - 4, py - 4, 8, 8); }
    ctx.restore(); label('Staging · operator console', x0 + 4, y1 + 16, '#9cc8ff');
  }
  if (!layers.piles) return;
  const m = S.mission || {}, active = m.active_pile;
  // dispatch arrows: history faint, latest bright
  const ds = (S.dispatch || []).filter(d => d.to_pile && pileById(d.from_pile) && pileById(d.to_pile));
  ds.forEach((d, k) => {
    const latest = k === ds.length - 1, to = pileById(d.to_pile);
    arrow(pileById(d.from_pile).xy, to.xy, COL.coord, latest ? 0.95 : 0.35, latest ? `dispatch → ${d.to_pile}${to.priority != null ? ' · priority ' + to.priority : ''}` : null);
  });
  for (const p of S.piles) {
    const [x, y] = toPx(...p.xy), [txt, col] = STATUS[p.status] || [p.status, COL.muted], isActive = p.id === active;
    ctx.save();
    ctx.beginPath(); ctx.arc(x, y, 8.5 * view.scale, 0, Math.PI * 2);
    ctx.strokeStyle = col; ctx.globalAlpha = isActive ? 0.95 : 0.45; ctx.lineWidth = isActive ? 2.4 : 1.2;
    ctx.setLineDash(p.status === 'no_signal_not_cleared' || p.status === 'unvisited' ? [4, 5] : []); ctx.stroke(); ctx.setLineDash([]);
    if (isActive) { const k = (now / 1600) % 1; ctx.globalAlpha = 0.5 * (1 - k); ctx.beginPath(); ctx.arc(x, y, (8.5 + 3 * k) * view.scale, 0, Math.PI * 2); ctx.stroke(); }
    ctx.restore();
    // name, priority badge, status chip (above the pile)
    const ty = y - 8.5 * view.scale - 8;
    ctx.font = '600 15px "IBM Plex Sans Condensed", sans-serif'; const nw = ctx.measureText(p.id).width;
    ctx.fillStyle = 'rgba(11,16,22,0.85)'; ctx.fillRect(x - nw / 2 - 5, ty - 15, nw + 10, 20); ctx.fillStyle = COL.fg; ctx.textAlign = 'left'; ctx.fillText(p.id, x - nw / 2, ty);
    if (p.priority != null) {
      ctx.font = '600 12px "IBM Plex Mono", monospace'; const t = String(p.priority), bw = ctx.measureText(t).width + 10, bx = x + nw / 2 + 8;
      ctx.fillStyle = p.status === 'lead' || p.priority >= (m.priority_min ?? 20) ? COL.teal : 'rgba(34,195,214,0.35)';
      roundRect(bx, ty - 14, bw, 18, 9); ctx.fill(); ctx.fillStyle = '#04161a'; ctx.fillText(t, bx + 5, ty);
    }
    ctx.font = '500 10.5px "IBM Plex Mono", monospace'; const cw = ctx.measureText(txt.toUpperCase()).width;
    ctx.fillStyle = 'rgba(11,16,22,0.85)'; ctx.fillRect(x - cw / 2 - 5, ty + 7, cw + 10, 15); ctx.strokeStyle = col; ctx.lineWidth = 1; ctx.strokeRect(x - cw / 2 - 5, ty + 7, cw + 10, 15);
    ctx.fillStyle = col; ctx.fillText(txt.toUpperCase(), x - cw / 2, ty + 18);
  }
}
function roundRect(x, y, w, h, r) { ctx.beginPath(); ctx.moveTo(x + r, y); ctx.arcTo(x + w, y, x + w, y + h, r); ctx.arcTo(x + w, y + h, x, y + h, r); ctx.arcTo(x, y + h, x, y, r); ctx.arcTo(x, y, x + w, y, r); ctx.closePath(); }
function label(t, x, y, color, bg = 'rgba(11,16,22,0.8)') {
  ctx.font = '500 11px "IBM Plex Mono", monospace'; const w = ctx.measureText(t).width;
  ctx.fillStyle = bg; ctx.fillRect(x - 3, y - 11, w + 6, 15); ctx.fillStyle = color; ctx.fillText(t, x, y);
}
function scaleBar() {
  const nice = [1, 2, 5, 10, 20, 50], m = nice.find(v => v * view.scale > 70) || 50, px = m * view.scale, x = 14, y = Hh - 44;
  ctx.strokeStyle = COL.fg; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + px, y); ctx.moveTo(x, y - 4); ctx.lineTo(x, y + 4); ctx.moveTo(x + px, y - 4); ctx.lineTo(x + px, y + 4); ctx.stroke();
  ctx.fillStyle = COL.fg; ctx.font = '500 11px "IBM Plex Mono", monospace'; ctx.fillText(`${m} m`, x + px + 6, y + 4);
  ctx.fillText('N ↑', W - 40, 60);
}

// interaction
let drag = null;
cv.addEventListener('pointerdown', e => { drag = { x: e.clientX, y: e.clientY, cx: view.cx, cy: view.cy, moved: false }; cv.setPointerCapture(e.pointerId); });
cv.addEventListener('pointermove', e => {
  const r = cv.getBoundingClientRect(), [mx, my] = toM(e.clientX - r.left, e.clientY - r.top);
  $('cursor').textContent = `x ${mx.toFixed(1)} · y ${my.toFixed(1)} m`;
  if (drag) { const dx = e.clientX - drag.x, dy = e.clientY - drag.y; if (Math.hypot(dx, dy) > 3) drag.moved = true; view.cx = drag.cx - dx / view.scale; view.cy = drag.cy + dy / view.scale; cv.classList.add('drag'); draw(); return; }
  const hit = hitTest(e.clientX - r.left, e.clientY - r.top); hoverId = hit?.id ?? null; cv.classList.toggle('hot', !!hit);
});
cv.addEventListener('pointerup', e => {
  cv.classList.remove('drag'); const r = cv.getBoundingClientRect();
  if (drag && !drag.moved) { const hit = hitTest(e.clientX - r.left, e.clientY - r.top); if (hit) openDetail(hit); }
  drag = null;
});
cv.addEventListener('wheel', e => { e.preventDefault(); const r = cv.getBoundingClientRect(); zoomAt(e.clientX - r.left, e.clientY - r.top, Math.exp(-e.deltaY * 0.0015)); }, { passive: false });
function zoomAt(px, py, k) { const [mx, my] = toM(px, py); view.scale = Math.max(2, Math.min(120, view.scale * k)); const [nx, ny] = toM(px, py); view.cx += mx - nx; view.cy += my - ny; draw(); }
$('zin').onclick = () => zoomAt(W / 2, Hh / 2, 1.4); $('zout').onclick = () => zoomAt(W / 2, Hh / 2, 1 / 1.4); $('zfit').onclick = fit;
function hitTest(px, py) {
  for (const c of S?.radar_candidates || []) { const [x, y] = toPx(...c.xy); if (Math.hypot(px - x, py - y) < 16) return c; }
  return null;
}
document.querySelectorAll('.tog').forEach(b => b.onclick = () => { const k = b.dataset.k; layers[k] = !layers[k]; b.setAttribute('aria-pressed', layers[k]); draw(); });

// ---------------- side panel ----------------
const bar = v => `<div class="bar"><i style="width:${Math.round(100 * Math.max(0, Math.min(1, v ?? 0)))}%"></i></div>`;
function renderSide() {
  const grid = isGrid();
  $('siteName').textContent = S.site?.name || 'Site';
  $('phase').textContent = S.phase || '—';
  $('simT').textContent = `t = ${f1(S.t)} s`;
  document.body.classList.toggle('grid', grid);
  $('togPiles').hidden = !grid;
  $('legendGrid').hidden = !grid;
  const cands = S.radar_candidates || [];
  $('nCand').textContent = cands.length;
  $('cands').innerHTML = cands.length ? cands.map(c => {
    const e = c.cov ? ellipse(c) : null, weak = c.status !== 'candidate', mv = c.kind === 'moving';
    const depth = c.z_est != null && S.heightmap ? hmAt(c.xy[0], c.xy[1]) - c.z_est : null;
    return `<button class="cand ${weak ? 'weak' : ''} ${mv ? 'moving' : ''}" data-id="${esc(c.id)}">
      <div class="t"><span>${esc(c.id)}${c.pile_id ? ` <small class="pl">pile ${esc(c.pile_id)}</small>` : ''}</span><small>${mv ? 'moving track' : weak ? 'insufficient evidence' : 'candidate region'}</small></div>
      <div class="kv"><span>p<b>${f2(c.p_mass)}</b></span><span>breathing<b>${c.breathing_hz ? (c.breathing_hz * 60).toFixed(0) + '/min' : '—'}</b></span><span>depth<b>${depth != null ? '≈' + f1(depth) + ' m' : '—'}</b></span>
      <span>95% region<b>${e ? f1(2 * e.rx) + '×' + f1(2 * e.ry) : '—'}</b></span><span>evidence<b>${(c.evidence || []).length} scans</b></span><span>at<b>${f1(c.xy[0])}, ${f1(c.xy[1])}</b></span></div>
      <div class="go">Open LiDAR view →</div></button>`;
  }).join('') : '<p class="empty">No radar candidates yet. No detection is not proof of absence.</p>';
  $('cands').querySelectorAll('.cand').forEach(b => b.onclick = () => openDetail(cands.find(c => c.id === b.dataset.id)));

  // acoustic agent (grid) or plain acoustic directions (v1)
  const clips = S.acoustic_clips;
  $('secAgent').hidden = !grid; $('secPiles').hidden = !grid; $('secAc').hidden = grid;
  if (grid) {
    const ag = S.acoustic_agent, src = S.mission?.acoustic_source;
    $('agentSrc').textContent = src ? `${src}${ag?.mode && ag.mode !== src ? ' · ' + ag.mode : ''}` : 'waiting';
    const list = [...(clips || [])].sort((a, b) => (b.listen ?? 0) - (a.listen ?? 0) || (b.priority ?? 0) - (a.priority ?? 0));
    $('clips').innerHTML = list.length ? list.map(c => {
      const h = c.scores?.human ?? 0, nonhuman = h < 0.4, by = (c.by || 'fallback').toLowerCase().includes('llm') || c.by === 'claude' ? 'llm' : (c.by || 'fallback');
      return `<div class="clip ${nonhuman ? 'nonhuman' : ''}">
        <div class="hd"><span><b>${esc(c.pile_id)}</b> · heard by ${esc(c.drone_id)} · listen ${c.listen ?? '—'}</span><span><span class="cls">${esc(c.class)}</span> <span class="by ${by === 'llm' ? 'llm' : ''}">${esc(by)}</span></span></div>
        ${c.transcript ? `<q>${esc(c.transcript)}</q>` : ''}
        <div class="scores"><span>human${bar(h)}</span><span>urgency${bar(c.scores?.urgency)}</span><span>direction${bar(c.scores?.reliability)}</span><span class="pr" title="priority">${c.priority ?? '—'}</span></div>
        <div class="why">${esc(c.rationale)}</div></div>`;
    }).join('') : '<p class="empty">Listening starts once the drones have landed and the radar phase at the first pile is over.</p>';
    const vis = S.mission?.visited || [];
    const rows = [...S.piles].sort((a, b) => {
      const va = vis.indexOf(a.id), vb = vis.indexOf(b.id);
      if (va >= 0 || vb >= 0) return (va < 0 ? 99 : va) - (vb < 0 ? 99 : vb);
      return (b.priority ?? -1) - (a.priority ?? -1) || a.id.localeCompare(b.id);
    });
    const topUnvisited = rows.find(p => !vis.includes(p.id) && p.priority != null && p.priority >= (S.mission?.priority_min ?? 20));
    $('nPiles').textContent = `${vis.length} visited`;
    $('piles').innerHTML = rows.map(p => {
      const [txt, col] = STATUS[p.status] || [p.status, COL.muted];
      return `<div class="row ${p === topUnvisited ? 'top' : ''} ${vis.includes(p.id) ? 'done' : ''}">
        <b>${esc(p.id)}</b><span class="why"><span class="chip" style="color:${col};border-color:${col}">${esc(txt)}</span> ${esc(p.why)}</span>
        <span class="s">${p.priority ?? '—'}</span></div>`;
    }).join('');
    const d = (S.dispatch || []).slice(-1)[0];
    $('dispatch').textContent = d ? (d.to_pile ? `Dispatch ${d.from_pile} → ${d.to_pile}: ${d.reason}` : `${d.from_pile}: search loop ended (${d.reason})`) : '';
  } else {
    const ac = S.acoustic || [];
    $('nAc').textContent = ac.length;
    $('acList').innerHTML = ac.length ? ac.map(a => `<div class="ac"><span>${esc(a.drone_id)} · <span class="c">${esc(a.class)}</span></span><span>${a.bearing_deg.toFixed(0)}° ±${(a.width_deg / 2).toFixed(0)} · ${f2(a.confidence)}</span></div>`).join('') : '<p class="empty">Nothing heard yet.</p>';
  }
  $('fleet').innerHTML = (S.drones || []).map(d => `<div class="fl"><b>${esc(d.id)}</b><span class="st ${esc(d.state)}">${esc(d.state)}</span><span class="bat" title="battery ${Math.round((d.battery ?? 0) * 100)}%"><i style="width:${(d.battery ?? 0) * 100}%"></i></span></div>`).join('');
  $('next').textContent = S.next_scan ? `Next: ${S.next_scan.drone_id} → ${S.next_scan.site_id} · ${S.next_scan.reason}` : '';
  $('log').innerHTML = (S.log || []).slice(-14).reverse().map(l => `<div><span>${f1(l.t)}s</span><p style="margin:0">${esc(l.msg)}</p></div>`).join('');
}
function hmAt(x, y) {
  const g = S.heightmap; const i = (x - g.origin[0]) / g.res - 0.5, j = (y - g.origin[1]) / g.res - 0.5;
  const i0 = Math.max(0, Math.min(g.w - 2, Math.floor(i))), j0 = Math.max(0, Math.min(g.h - 2, Math.floor(j))), u = i - i0, v = j - j0, z = (a, b) => g.z[b * g.w + a];
  return (z(i0, j0) * (1 - u) + z(i0 + 1, j0) * u) * (1 - v) + (z(i0, j0 + 1) * (1 - u) + z(i0 + 1, j0 + 1) * u) * v;
}

// ---------------- detail: LiDAR view with depth ping ----------------
let viewer = null;
async function openDetail(c) {
  $('detail').hidden = false;
  const surf = S.heightmap ? hmAt(c.xy[0], c.xy[1]) : 0, z = c.z_est ?? surf - 1.5, depth = surf - z, e = c.cov ? ellipse(c) : null;
  $('dTitle').textContent = c.pile_id ? `${c.id}` : c.id;
  $('dStatus').textContent = c.kind === 'moving' ? 'Moving track · tracked separately' : c.status === 'candidate' ? 'Candidate search region · not a confirmed person' : 'Insufficient evidence · rescan suggested';
  const pile = c.pile_id ? pileById(c.pile_id) : null;
  const heard = (S.acoustic_clips || []).filter(a => a.pile_id === c.pile_id && (a.scores?.human ?? 0) >= 0.4).sort((a, b) => (b.priority ?? 0) - (a.priority ?? 0))[0];
  const rows = [
    ...(pile ? [['Pile', `${pile.id} · ${(STATUS[pile.status] || [pile.status])[0]}${pile.priority != null ? ' · acoustic priority ' + pile.priority : ''}`]] : []),
    ['Position (E, N)', `${f1(c.xy[0])}, ${f1(c.xy[1])} m`],
    ['Surface height', `${f1(surf)} m`],
    ['Estimated depth', `≈${f1(depth)} m below surface${c.z_est == null ? ' (default, no estimate)' : ''}`],
    ['95% region', e ? `${f1(2 * e.rx)} × ${f1(2 * e.ry)} m` : '—'],
    ['Probability mass', f2(c.p_mass)],
    ['Periodic signal', c.breathing_hz ? `${f2(c.breathing_hz)} Hz (${(c.breathing_hz * 60).toFixed(0)}/min), respiration-like` : '—'],
    ['Motion', c.kind === 'moving' ? 'Doppler shows bulk motion' : 'stationary'],
    ...(heard ? [['Heard here', `${esc(heard.class)}${heard.transcript ? ' · “' + esc(heard.transcript) + '”' : ''}`]] : []),
  ];
  $('dFacts').innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
  const ev = new Set(c.evidence || []);
  const scans = (S.scans || []).filter(s => !c.pile_id || !s.pile_id || s.pile_id === c.pile_id);
  $('dEv').querySelector('tbody').innerHTML = scans.map(s => `<tr class="${ev.has(s.scan_id) ? '' : 'low'}"><td>${esc(s.scan_id)}${ev.has(s.scan_id) ? '' : ' ·'}</td><td>${esc(s.drone_id)}</td><td>${(s.range_candidates || []).map(f1).join(', ') || 'none'}</td><td>${(s.confidence || []).map(f2).join(', ') || '—'}</td><td>${f2(s.quality)}</td></tr>`).join('');
  await ensureCloud();
  viewer ||= await import('./viewer3d.js').then(m => m.createViewer($('v3d')));
  viewer.show({ cloud, center: [c.xy[0], c.xy[1]], surf, z, ellipse: e, id: c.id, depth });
}
function closeDetail() { $('detail').hidden = true; viewer?.stop(); }
$('dClose').onclick = closeDetail;
$('detail').addEventListener('click', e => { if (e.target.id === 'detail') closeDetail(); });
addEventListener('keydown', e => { if (e.key === 'Escape') closeDetail(); });

// ---------------- boot ----------------
addEventListener('resize', resize);
resize();
poll(); setInterval(poll, POLL_MS);
(function tick(now) { draw(now); requestAnimationFrame(tick); })(performance.now());
