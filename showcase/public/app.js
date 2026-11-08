/* Larpverse showcase: a deterministic timeline. render(t) draws any instant, so the
   same page plays live in a browser and renders frame-perfect video (window.__seek). */
(() => {
'use strict';
const $ = (s, r = document) => r.querySelector(s);
const $$ = (s, r = document) => [...r.querySelectorAll(s)];
const clamp = (x, a = 0, b = 1) => Math.min(b, Math.max(a, x));
const mix = (a, b, p) => a + (b - a) * p;
const RENDER = new URLSearchParams(location.search).has('render');
if (RENDER) document.body.classList.add('render');

// ---- easing -------------------------------------------------------------
// critically damped spring, x in seconds since start; settles in ~0.7 s
const spring = (x, w = 8) => (x <= 0 ? 0 : 1 - (1 + w * x) * Math.exp(-w * x));
// slightly bouncy spring for "pop" moments
const bouncy = (x, w = 11, z = 0.55) => {
  if (x <= 0) return 0;
  const wd = w * Math.sqrt(1 - z * z);
  return 1 - Math.exp(-z * w * x) * (Math.cos(wd * x) + (z * w / wd) * Math.sin(wd * x));
};
const easeInOut = (p) => (p = clamp(p), p < 0.5 ? 4 * p * p * p : 1 - Math.pow(-2 * p + 2, 3) / 2);
const expoOut = (p) => (p = clamp(p), p === 1 ? 1 : 1 - Math.pow(2, -10 * p));
const ramp = (t, a, d) => clamp((t - a) / d);
const pm = (a, m) => ((a % m) + m) % m;

// ---- scenes ---------------------------------------------------------------
const SCENES = [
  ['s1', 0.0, 7.4],
  ['s2', 7.1, 13.4],
  ['s3', 13.1, 19.8],
  ['s4', 19.5, 46.0],
  ['s5', 45.7, 62.2],
  ['s6', 61.9, 72.2],
  ['s7', 71.9, 82.4],
  ['s8', 82.1, 89.4],
  ['s9', 89.1, 97.0],
];
const DURATION = 97.0;
const scene = {};
for (const [id, a, b] of SCENES) scene[id] = { el: document.getElementById(id), a, b };

// ---- text splitting -------------------------------------------------------
function splitWords(el) {
  const out = [];
  const walk = (node) => {
    for (const ch of [...node.childNodes]) {
      if (ch.nodeType === 3) {
        const frag = document.createDocumentFragment();
        ch.textContent.split(/(\s+)/).forEach((tok) => {
          if (!tok) return;
          if (/^\s+$/.test(tok)) { frag.appendChild(document.createTextNode(' ')); return; }
          const wrap = document.createElement('span'); wrap.className = 'wrap';
          const w = document.createElement('span'); w.className = 'w'; w.textContent = tok;
          wrap.appendChild(w); frag.appendChild(wrap); out.push(w);
        });
        node.replaceChild(frag, ch);
      } else if (ch.nodeType === 1) walk(ch);
    }
  };
  walk(el);
  return out;
}
function splitLetters(el) {
  const txt = el.textContent; el.textContent = '';
  return [...txt].map((c) => {
    const w = document.createElement('span'); w.className = 'w'; w.textContent = c; el.appendChild(w); return w;
  });
}

// reveal items: {el, scene, at, kind, parts}
const reveals = [];
for (const s of Object.values(scene)) {
  for (const el of $$('[data-in]', s.el)) {
    const at = parseFloat(el.dataset.in);
    if (el.classList.contains('words')) reveals.push({ s, at, kind: 'words', parts: splitWords(el) });
    else if (el.classList.contains('letters')) reveals.push({ s, at, kind: 'letters', parts: splitLetters(el) });
    else reveals.push({ s, at, kind: 'r', el });
  }
}
const rolls = $$('.roll').map((el) => ({
  el, s: scene[el.closest('.scene').id], at: parseFloat(el.dataset.at || 0), to: parseFloat(el.dataset.to),
  from: parseFloat(el.dataset.from || 0), dec: parseInt(el.dataset.dec || 0, 10), suf: el.dataset.suf || '', last: null,
}));
const bars = $$('.track i[data-to]').map((el) => ({ el, s: scene[el.closest('.scene').id], at: parseFloat(el.dataset.at), to: parseFloat(el.dataset.to) }));

// ---- stars ------------------------------------------------------------------
(() => {
  const c = $('#stars'), g = c.getContext('2d');
  let seed = 7; const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
  for (let i = 0; i < 160; i++) {
    g.globalAlpha = 0.15 + rnd() * 0.5; g.fillStyle = '#cfd8ee';
    g.beginPath(); g.arc(rnd() * 1920, rnd() * 1080, rnd() * 1.2 + 0.3, 0, 7); g.fill();
  }
})();

// ---- SVG helpers --------------------------------------------------------------
const NS = 'http://www.w3.org/2000/svg';
const el = (tag, attrs = {}, parent) => {
  const e = document.createElementNS(NS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(e);
  return e;
};
const set = (e, attrs) => { for (const k in attrs) e.setAttribute(k, attrs[k]); };
const fo = (parent, x, y, w, h, html, cls = 'chip') => {
  const f = el('foreignObject', { x, y, width: w, height: h }, parent);
  const d = document.createElement('div'); d.className = cls; d.innerHTML = html; f.appendChild(d);
  return f;
};
let rs = 11; const rnd = () => ((rs = (rs * 16807) % 2147483647) / 2147483647);
function blob(cx, cy, r, n = 22, jit = 0.18) {
  const pts = [];
  for (let i = 0; i < n; i++) {
    const a = (i / n) * Math.PI * 2, rr = r * (1 - jit / 2 + rnd() * jit);
    pts.push([cx + Math.cos(a) * rr, cy + Math.sin(a) * rr * 0.92]);
  }
  return 'M' + pts.map((p) => p.map((v) => v.toFixed(1)).join(',')).join('L') + 'Z';
}

// =====================================================================
// S4: the loop map
// =====================================================================
const STEPS = [
  ['Survey', 'LiDAR maps the rubble pile in 3D.', 0],
  ['Land', 'The drones land on flat spots and switch their rotors off.', 4.2],
  ['Radar', 'Radar looks for breathing under the rubble. Each reading is a distance: a ring.', 6.6],
  ['Ping', 'Where three rings cross, the drones mark a place to dig.', 10.0],
  ['Listen', 'Then they listen to the other piles: voices, knocking, a dog.', 12.2],
  ['Rank', 'An AI agent ranks the piles by how human and how urgent each sound is.', 16.2],
  ['Dispatch', 'The drones fly to the top pile on their own. The loop starts again.', 20.0],
];
const stepsEl = $('#steps');
STEPS.forEach(([name], i) => {
  const li = document.createElement('li');
  li.innerHTML = `<span class="n">0${i + 1}</span>${name}`;
  stepsEl.appendChild(li);
});
const stepLis = $$('#steps li');
const stepcap = $('#stepcap');

const map = $('#map');
const defs = el('defs', {}, map);
defs.innerHTML = `
  <radialGradient id="pileG" cx="45%" cy="40%" r="65%"><stop offset="0" stop-color="#3a4560"/><stop offset="1" stop-color="#1c2436"/></radialGradient>
  <radialGradient id="sweepG" cx="0" cy="0" r="1" gradientUnits="userSpaceOnUse"><stop offset="0" stop-color="#8e7bff" stop-opacity=".55"/><stop offset="1" stop-color="#8e7bff" stop-opacity="0"/></radialGradient>
  <filter id="glow" x="-50%" y="-50%" width="200%" height="200%"><feGaussianBlur stdDeviation="6"/></filter>`;
// grid
const gridG = el('g', { opacity: 0.5 }, map);
for (let x = 0; x <= 900; x += 45) el('line', { x1: x, y1: 0, x2: x, y2: 900, stroke: 'rgba(255,255,255,.035)' }, gridG);
for (let y = 0; y <= 900; y += 45) el('line', { x1: 0, y1: y, x2: 900, y2: y, stroke: 'rgba(255,255,255,.035)' }, gridG);

const COLS = [180, 450, 720], ROWS = [180, 440, 700];
const PILES = {};
const pileG = el('g', {}, map);
['A', 'B', 'C'].forEach((r, ri) => [1, 2, 3].forEach((c, ci) => {
  const id = r + c, x = COLS[ci], y = ROWS[ri];
  const g = el('g', {}, pileG);
  const halo = el('path', { d: blob(x, y, 100, 24, 0.12), fill: 'none', stroke: '#f0857c', 'stroke-width': 3, opacity: 0, filter: 'url(#glow)' }, g);
  const body = el('path', { d: blob(x, y, 88, 24, 0.2), fill: 'url(#pileG)', stroke: 'rgba(255,255,255,.12)', 'stroke-width': 1.5 }, g);
  for (let k = 0; k < 7; k++) {
    const a = rnd() * 6.28, d = rnd() * 50;
    el('path', { d: blob(x + Math.cos(a) * d, y + Math.sin(a) * d, 8 + rnd() * 10, 6, 0.5), fill: 'rgba(10,14,24,.35)' }, g);
  }
  const label = el('text', { x: x - 82, y: y - 78, fill: '#9aa5ba', 'font-family': 'GeistMono', 'font-size': 18, 'font-weight': 600 }, g);
  label.textContent = id;
  PILES[id] = { x, y, g, halo, body, label };
}));
// staging pad
const PAD = { x: 80, y: 845 };
el('rect', { x: PAD.x - 46, y: PAD.y - 26, width: 92, height: 52, rx: 10, fill: 'rgba(91,140,255,.08)', stroke: 'rgba(91,140,255,.4)', 'stroke-dasharray': '4 4' }, map);
const padT = el('text', { x: PAD.x, y: PAD.y + 46, fill: '#6b7690', 'font-family': 'GeistMono', 'font-size': 13, 'text-anchor': 'middle' }, map);
padT.textContent = 'staging';

// LiDAR point cloud on A2 and A3
function cloud(id, n) {
  const P = PILES[id], g = el('g', {}, map), pts = [];
  for (let i = 0; i < n; i++) {
    const a = rnd() * 6.28, d = Math.sqrt(rnd()) * 82;
    pts.push(el('circle', { cx: P.x + Math.cos(a) * d, cy: P.y + Math.sin(a) * d * 0.92, r: 1.8, fill: '#a597ff', opacity: 0 }, g));
  }
  return { g, pts };
}
const cloudA2 = cloud('A2', 260), cloudA3 = cloud('A3', 260);

const T = { x: PILES.A2.x + 14, y: PILES.A2.y + 8 }; // hidden breathing source (sim truth, drawn only via rings)
const LC = [[-62, 52], [66, 46], [4, -66]].map(([dx, dy]) => ({ x: PILES.A2.x + dx, y: PILES.A2.y + dy }));
const COLORS = ['#5b8cff', '#2cb7af', '#f5c77e'];

// landing cells
const cellsG = el('g', {}, map);
const cells = LC.map((p) => el('rect', { x: p.x - 16, y: p.y - 16, width: 32, height: 32, rx: 6, fill: 'rgba(61,181,127,.18)', stroke: '#3db57f', 'stroke-width': 2, opacity: 0 }, cellsG));

// radar rings
const ringsG = el('g', {}, map);
const ringR = LC.map((p) => Math.hypot(p.x - T.x, p.y - T.y));
const rings = LC.map((p, i) => el('circle', { cx: p.x, cy: p.y, r: ringR[i], fill: 'none', stroke: '#f5a35c', 'stroke-width': 2.5, opacity: 0, 'stroke-dasharray': 2 * Math.PI * ringR[i], 'stroke-dashoffset': 2 * Math.PI * ringR[i] }, ringsG));
const pulses = LC.map((p) => [0, 1].map(() => el('circle', { cx: p.x, cy: p.y, r: 0, fill: 'none', stroke: '#f5a35c', 'stroke-width': 2, opacity: 0 }, ringsG)));

// sound domes
const SOUNDS = [
  { id: 'A3', color: '#f0857c', html: '<span class="q">“Help… my leg is trapped”</span>', w: 300, dx: -150, dy: 108 },
  { id: 'B2', color: '#f5c77e', html: 'Knocking, 3 times', w: 190, dx: -95, dy: 108 },
  { id: 'C1', color: '#8a94ab', html: 'Dog bark', w: 120, dx: -60, dy: 108 },
];
const soundG = el('g', {}, map);
for (const s of SOUNDS) {
  const P = PILES[s.id];
  s.rings = [0, 1, 2].map(() => el('circle', { cx: P.x, cy: P.y, r: 0, fill: 'none', stroke: s.color, 'stroke-width': 2.5, opacity: 0 }, soundG));
  s.chip = fo(soundG, P.x + s.dx, P.y + s.dy - 4, s.w, 46, s.html, 'chip');
  s.chip.style.opacity = 0;
}
// bearing cones from landed drones toward A3
const conesG = el('g', {}, map);
const cones = LC.map(() => el('path', { fill: '#2cb7af', opacity: 0 }, conesG));

// dispatch arc
const A2 = PILES.A2, A3 = PILES.A3;
const arcD = `M${A2.x + 20},${A2.y - 60} Q${(A2.x + A3.x) / 2},${A2.y - 175} ${A3.x - 20},${A3.y - 60}`;
const arc = el('path', { d: arcD, fill: 'none', stroke: '#5b8cff', 'stroke-width': 3, 'stroke-dasharray': '10 9', opacity: 0 }, map);
const arcLen = arc.getTotalLength ? arc.getTotalLength() : 320;

// ping marker
const pingG = el('g', { opacity: 0 }, map);
const pingPulse = el('ellipse', { cx: T.x, cy: T.y, rx: 30, ry: 22, fill: 'none', stroke: '#ff5d5d', 'stroke-width': 2 }, pingG);
const pingE = el('ellipse', { cx: T.x, cy: T.y, rx: 30, ry: 22, fill: 'rgba(255,93,93,.22)', stroke: '#ff5d5d', 'stroke-width': 2.5 }, pingG);
el('circle', { cx: T.x, cy: T.y, r: 4.5, fill: '#ff8080' }, pingG);
const pingLbl = fo(pingG, T.x - 150, T.y + 92, 220, 70, '<b>Dig here first</b><span>A2 · candidate region</span>', 'chip ping');
const pingBadge = fo(map, A2.x - 64, A2.y + 96, 128, 40, '✓ pinged', 'chip ok');
pingBadge.style.opacity = 0;

// drones (on top)
const droneG = el('g', {}, map);
const drones = COLORS.map((c, i) => {
  const g = el('g', {}, droneG);
  const sweep = el('path', { fill: 'url(#sweepG)', opacity: 0 }, g);
  const glow = el('circle', { r: 20, fill: c, opacity: 0.25, filter: 'url(#glow)' }, g);
  const rotor = el('circle', { r: 17, fill: 'none', stroke: c, 'stroke-width': 2, opacity: 0.8, 'stroke-dasharray': '6 5' }, g);
  const core = el('circle', { r: 8, fill: c, stroke: '#090e18', 'stroke-width': 2 }, g);
  const lbl = el('text', { 'font-family': 'GeistMono', 'font-size': 14, fill: '#eef2f9', 'font-weight': 600 }, g);
  lbl.textContent = 'd' + (i + 1);
  return { g, sweep, glow, rotor, core, lbl, c };
});

// map chips CSS (inside foreignObject)
const st = document.createElement('style');
st.textContent = `
.chip{font:500 17px Geist,sans-serif;color:#eef2f9;background:rgba(9,14,24,.88);border:1px solid rgba(255,255,255,.16);border-radius:12px;padding:8px 14px;white-space:nowrap;display:inline-block;box-shadow:0 10px 30px -10px #000}
.chip .q{color:#ffb4ad}
.chip.ping{border-color:rgba(255,93,93,.6);display:flex;flex-direction:column;gap:2px}
.chip.ping b{font-weight:600;color:#ff9b9b;font-size:18px}.chip.ping span{font:13px GeistMono,monospace;color:#9aa5ba}
.chip.ok{color:#7fe0b0;border-color:rgba(61,181,127,.5);text-align:center}
.chip.zone{border-color:rgba(44,183,175,.6);color:#9ff0e9}`;
document.head.appendChild(st);

const orbitPos = (P, i, L, R = 118) => {
  const a = ((-90 + i * 120 + L * 42) * Math.PI) / 180;
  return { x: P.x + Math.cos(a) * R, y: P.y + Math.sin(a) * R * 0.92 };
};
const lerpP = (a, b, p) => ({ x: mix(a.x, b.x, p), y: mix(a.y, b.y, p) });
const quad = (a, c, b, p) => lerpP(lerpP(a, c, p), lerpP(c, b, p), p);

function dronePos(i, L) {
  const pad = { x: PAD.x + (i - 1) * 26, y: PAD.y };
  if (L < 1.6) return lerpP(pad, orbitPos(A2, i, L), easeInOut(L / 1.6));
  if (L < 4.2) return orbitPos(A2, i, L);
  if (L < 5.4) return lerpP(orbitPos(A2, i, 4.2), LC[i], easeInOut((L - 4.2) / 1.2));
  if (L < 20.4 + i * 0.15) return LC[i];
  const a = 20.4 + i * 0.15, d = 2.6;
  if (L < a + d) {
    const end = orbitPos(A3, i, a + d);
    const ctrl = { x: (LC[i].x + end.x) / 2, y: Math.min(LC[i].y, end.y) - 150 };
    return quad(LC[i], ctrl, end, easeInOut((L - a) / d));
  }
  return orbitPos(A3, i, L);
}
const wedge = (cx, cy, ang, span, r) => {
  const a0 = ang - span / 2, a1 = ang + span / 2;
  return `M${cx},${cy}L${cx + Math.cos(a0) * r},${cy + Math.sin(a0) * r}A${r},${r} 0 0 1 ${cx + Math.cos(a1) * r},${cy + Math.sin(a1) * r}Z`;
};

const rank = $('#rank');
const rrows = $$('.rrow', rank).map((r, i) => ({ r, bar: $('.bar i', r), v: $('.rv', r), to: [77, 39, 3][i] }));
const decision = $('.decision', rank);
let lastStep = -1;

function renderS4(u) {
  const L = u - 0.8;
  // steps highlight
  let idx = 0; STEPS.forEach((s, i) => { if (L >= s[2]) idx = i; });
  let hl = 0; for (let i = 1; i < STEPS.length; i++) hl += 62 * spring(L - STEPS[i][2], 9);
  $('#stepsHl').style.transform = `translateY(${hl}px)`;
  $('#stepsHl').style.opacity = clamp((u - 0.3) / 0.5);
  stepLis.forEach((li, i) => { li.className = i === idx ? 'on' : i < idx ? 'done' : ''; });
  if (idx !== lastStep) { stepcap.textContent = STEPS[idx][1]; lastStep = idx; }
  const cp = spring(L - STEPS[idx][2] + 0.0001, 9);
  stepcap.style.opacity = u < 0.5 ? 0 : cp; stepcap.style.transform = `translateY(${(1 - cp) * 12}px)`;

  // drones
  const flying = L < 5.2 || L > 20.3;
  drones.forEach((d, i) => {
    const p = dronePos(i, Math.max(L, 0));
    set(d.g, { transform: `translate(${p.x.toFixed(2)},${p.y.toFixed(2)})` });
    set(d.lbl, { x: 14, y: -14 });
    const rotOn = L < 5.0 ? 1 : L > 20.3 ? clamp((L - 20.3) / 0.4) : 0;
    set(d.rotor, { opacity: (0.25 + 0.65 * rotOn).toFixed(3), transform: `rotate(${(L * 900 * rotOn) % 360})` });
    set(d.glow, { opacity: (0.12 + 0.18 * rotOn).toFixed(3) });
    // LiDAR sweep while surveying
    let sw = 0;
    if (L > 1.0 && L < 4.3) sw = clamp((L - 1.0) / 0.4) * clamp((4.3 - L) / 0.3);
    if (L > 23.2) sw = clamp((L - 23.2) / 0.4) * 0.9;
    const ang = pm(L * 5.5 + i * 2.1, Math.PI * 2);
    set(d.sweep, { d: wedge(0, 0, ang, 1.1, 125), opacity: sw.toFixed(3) });
  });
  // LiDAR cloud
  const nA2 = cloudA2.pts.length;
  cloudA2.pts.forEach((c, k) => {
    const appear = 1.2 + (k / nA2) * 2.9;
    c.setAttribute('opacity', (clamp((L - appear) / 0.3) * (L > 4.4 ? mix(0.85, 0.35, clamp((L - 4.4) / 1)) : 0.85)).toFixed(3));
  });
  cloudA3.pts.forEach((c, k) => {
    const appear = 23.3 + (k / nA2) * 2.5;
    c.setAttribute('opacity', (clamp((L - appear) / 0.3) * 0.85).toFixed(3));
  });
  // landing cells
  cells.forEach((c, i) => {
    const p = spring(L - 4.4 - i * 0.12);
    set(c, { opacity: (p * (L > 20.5 ? clamp(1 - (L - 20.5) / 0.8) : 1)).toFixed(3) });
  });
  // radar rings + pulses
  rings.forEach((r, i) => {
    const C = 2 * Math.PI * ringR[i];
    const p = expoOut((L - 6.9 - i * 0.45) / 1.2);
    const fade = L > 12 ? mix(0.9, 0.3, clamp((L - 12) / 1)) : 0.9;
    set(r, { 'stroke-dashoffset': (C * (1 - p)).toFixed(1), opacity: (L > 6.9 + i * 0.45 ? fade : 0).toFixed(3) });
    if (L > 20.6) r.setAttribute('opacity', (0.3 * clamp(1 - (L - 20.6) / 0.8)).toFixed(3));
  });
  pulses.forEach((ps, i) => ps.forEach((c, k) => {
    const on = L > 6.6 && L < 10.4;
    const ph = pm(L - 6.6 + k * 0.6 + i * 0.2, 1.2) / 1.2;
    set(c, { r: (ph * ringR[i] * 1.15).toFixed(1), opacity: on ? ((1 - ph) * 0.7).toFixed(3) : 0 });
  }));
  // ping
  const pp = bouncy(L - 10.05);
  set(pingG, { opacity: clamp((L - 10.0) / 0.25).toFixed(3) });
  set(pingE, { transform: `translate(${T.x},${T.y}) scale(${Math.max(pp, 0.001)}) translate(${-T.x},${-T.y})` });
  const pph = pm(L - 10, 1.6) / 1.6;
  set(pingPulse, { transform: `translate(${T.x},${T.y}) scale(${1 + pph * 1.4}) translate(${-T.x},${-T.y})`, opacity: L > 10 ? (1 - pph) * 0.8 : 0 });
  pingLbl.style.opacity = spring(L - 10.4); pingLbl.style.transform = `translateY(${(1 - spring(L - 10.4)) * 10}px)`;
  if (L > 20.2) { set(pingG, { opacity: clamp(1 - (L - 20.2) / 0.6) }); }
  pingBadge.style.opacity = spring(L - 20.6);
  // listening
  for (const [si, s] of SOUNDS.entries()) {
    const on = L > 12.4 + si * 0.35 && L < 20.4;
    const strength = [1, 0.75, 0.5][si];
    s.rings.forEach((c, k) => {
      const ph = pm(L - 12.4 + k * 0.45, 1.35) / 1.35;
      set(c, { r: (30 + ph * 120).toFixed(1), opacity: on ? ((1 - ph) * 0.75 * strength).toFixed(3) : 0 });
    });
    const cp2 = spring(L - 12.9 - si * 0.4);
    const out = L > 20.0 ? clamp(1 - (L - 20.0) / 0.5) : 1;
    s.chip.style.opacity = (cp2 * out).toFixed(3);
  }
  cones.forEach((c, i) => {
    const p0 = LC[i], ang = Math.atan2(A3.y - p0.y, A3.x - p0.x);
    const on = clamp((L - 13.6) / 0.5) * clamp((19.8 - L) / 0.5);
    set(c, { d: wedge(p0.x, p0.y, ang, 0.28, Math.hypot(A3.x - p0.x, A3.y - p0.y) - 40), opacity: (on * 0.22).toFixed(3) });
  });
  // A3 highlight during rank and dispatch
  set(PILES.A3.halo, { opacity: (clamp((L - 16.6) / 0.5) * (0.65 + 0.25 * Math.sin(L * 5))).toFixed(3) });
  // rank panel
  const rp = spring(L - 16.3);
  rank.style.opacity = rp; rank.style.transform = `translateY(${(1 - rp) * 30}px) scale(${0.97 + 0.03 * rp})`;
  rrows.forEach((r, i) => {
    const p = spring(L - 16.7 - i * 0.18, 7);
    r.r.style.opacity = clamp((L - 16.6 - i * 0.18) / 0.3);
    r.bar.style.width = (r.to * p).toFixed(2) + '%';
    r.v.textContent = Math.round(r.to * expoOut((L - 16.7 - i * 0.18) / 1.2));
  });
  const dp = spring(L - 18.4);
  decision.style.opacity = dp; decision.style.transform = `translateY(${(1 - dp) * 8}px)`;
  // dispatch arc
  const ap = easeInOut((L - 20.0) / 0.9);
  set(arc, { opacity: L > 20 ? clamp(1 - (L - 23.6) / 0.6) : 0, 'stroke-dasharray': `${arcLen * ap} ${arcLen}` });
}

// =====================================================================
// S5: footage window
// =====================================================================
const WCAPS = [
  'A reconstruction of the real collapse site',
  'LiDAR maps one sector of the rubble',
  'Radar rings cross: a place to dig',
  'Each ping becomes a marked place to dig',
];
const wclips = $$('#s5 .wclip').map((v) => ({ v, at: parseFloat(v.dataset.start), dur: parseFloat(v.dataset.dur) }));
const wnum = $('#wnum'), wtxt = $('#wtxt'), wcap = $('.wcap');
let lastW = -1;
function renderS5(u) {
  let idx = 0;
  wclips.forEach((c, i) => {
    const local = u - c.at;
    const op = clamp(local / 0.5) * (i < wclips.length - 1 ? clamp((wclips[i + 1].at + 0.5 - u) / 0.5) : 1);
    c.v.style.opacity = op;
    c.v.style.transform = `scale(${1.0 + 0.045 * clamp(local / c.dur)})`;
    c.want = op > 0 ? clamp(local, 0, c.dur - 0.04) : null;
    if (u >= c.at + 0.25) idx = i;
  });
  if (idx !== lastW) { wnum.textContent = idx + 1; wtxt.textContent = WCAPS[idx]; lastW = idx; }
  const cp = spring(u - (wclips[idx].at + 0.25), 9);
  wcap.style.opacity = u < 0.6 ? 0 : (idx === 0 ? spring(u - 0.6) : cp);
  wcap.style.transform = `translateY(${(1 - (idx === 0 ? spring(u - 0.6) : cp)) * 12}px)`;
}

// =====================================================================
// S6: live change (operator adds a zone)
// =====================================================================
const dmap = $('#dmap');
dmap.innerHTML = `<defs><pattern id="hatch" width="10" height="10" patternUnits="userSpaceOnUse" patternTransform="rotate(45)"><line x1="0" y1="0" x2="0" y2="10" stroke="rgba(44,183,175,.35)" stroke-width="3"/></pattern></defs>`;
const dg = el('g', {}, dmap);
for (let x = 0; x <= 900; x += 40) el('line', { x1: x, y1: 0, x2: x, y2: 560, stroke: 'rgba(255,255,255,.03)' }, dg);
for (let y = 0; y <= 560; y += 40) el('line', { x1: 0, y1: y, x2: 900, y2: y, stroke: 'rgba(255,255,255,.03)' }, dg);
// buildings/rubble
[[60, 60, 150, 90], [250, 40, 120, 70], [700, 50, 150, 100], [60, 400, 130, 110], [720, 380, 140, 120], [400, 430, 180, 90]].forEach(([x, y, w, h]) =>
  el('rect', { x, y, width: w, height: h, rx: 8, fill: '#1a2233', stroke: 'rgba(255,255,255,.07)' }, dg));
rs = 3;
el('path', { d: blob(300, 260, 120, 26, 0.25), fill: '#242d42', stroke: 'rgba(255,255,255,.1)' }, dg);
el('path', { d: blob(610, 250, 110, 26, 0.25), fill: '#242d42', stroke: 'rgba(255,255,255,.1)' }, dg);
// zone T1a (existing)
const zA = el('path', { d: 'M180,150 L430,140 L440,370 L170,380 Z', fill: 'rgba(91,140,255,.08)', stroke: '#5b8cff', 'stroke-width': 2.5, 'stroke-dasharray': '8 6' }, dmap);
const zAl = fo(dmap, 182, 152, 140, 40, 'Zone T1a · done', 'chip');
// zone T1b (new)
const zBd = 'M500,150 L730,145 L735,360 L505,365 Z';
const zB = el('path', { d: zBd, fill: 'url(#hatch)', stroke: '#2cb7af', 'stroke-width': 3, opacity: 0 }, dmap);
const zBlen = 2 * (230 + 215);
const zBl = fo(dmap, 508, 152, 170, 40, 'New zone T1b', 'chip zone');
// voices near T1b
const vRings = [0, 1, 2].map(() => el('circle', { cx: 615, cy: 255, r: 0, fill: 'none', stroke: '#f0857c', 'stroke-width': 2.5, opacity: 0 }, dmap));
const vChip = fo(dmap, 540, 300, 200, 40, '<span class="q">Voices heard</span>', 'chip');
// drones in T1a
const dDr = COLORS.map((c, i) => {
  const g = el('g', {}, dmap);
  el('circle', { r: 16, fill: 'none', stroke: c, 'stroke-width': 2, 'stroke-dasharray': '5 4' }, g);
  el('circle', { r: 7, fill: c, stroke: '#090e18', 'stroke-width': 2 }, g);
  const t = el('text', { x: 13, y: -13, 'font-family': 'GeistMono', 'font-size': 13, fill: '#eef2f9', 'font-weight': 600 }, g); t.textContent = 'd' + (i + 1);
  return g;
});
const dStart = [{ x: 250, y: 220 }, { x: 360, y: 300 }, { x: 300, y: 330 }];
const dEnd = [{ x: 560, y: 200 }, { x: 680, y: 230 }, { x: 600, y: 320 }];
const dArcs = dStart.map((s, i) => el('path', { d: `M${s.x},${s.y} Q${(s.x + dEnd[i].x) / 2},${Math.min(s.y, dEnd[i].y) - 110} ${dEnd[i].x},${dEnd[i].y}`, fill: 'none', stroke: COLORS[i], 'stroke-width': 2, 'stroke-dasharray': '6 6', opacity: 0 }, dmap));
const cursor = $('#cursor'), addBtn = $('#addzone'), ban1 = $('#ban1'), ban2 = $('#ban2');
const logLis = $$('#log li').map((li) => ({ li, at: parseFloat(li.dataset.at) }));
function renderS6(u) {
  // voices pulse near T1b
  vRings.forEach((c, k) => {
    const ph = pm(u - 0.6 + k * 0.45, 1.35) / 1.35;
    set(c, { r: (20 + ph * 90).toFixed(1), opacity: u > 0.6 && u < 4.2 ? ((1 - ph) * 0.7).toFixed(3) : 0 });
  });
  vChip.style.opacity = spring(u - 0.9) * clamp((4.2 - u) / 0.4);
  // cursor path: enter, travel to button, click
  const btnX = 842 + 70, btnY = 32 + 2; // within dash
  const start = { x: 640, y: 560 };
  const cp = easeInOut((u - 1.3) / 1.5);
  const cpos = lerpP(start, { x: btnX, y: btnY }, cp);
  const leave = easeInOut((u - 3.6) / 1.0);
  const cfinal = lerpP(cpos, { x: 720, y: 520 }, leave);
  cursor.style.opacity = clamp((u - 1.1) / 0.3) * clamp((4.8 - u) / 0.3);
  const press = u > 3.0 && u < 3.25 ? 0.9 : 1;
  cursor.style.transform = `translate(${cfinal.x}px,${cfinal.y}px) scale(${press})`;
  addBtn.style.transform = `scale(${u > 3.0 && u < 3.25 ? 0.93 : 1})`;
  addBtn.style.boxShadow = u > 3.0 && u < 3.9 ? `0 0 0 ${8 * (u - 3) / 0.9}px rgba(44,183,175,${0.4 * (1 - (u - 3) / 0.9)})` : '';
  // zone draw
  const zp = easeInOut((u - 3.3) / 0.9);
  set(zB, { opacity: u > 3.3 ? 1 : 0, 'stroke-dasharray': `${zBlen * zp} ${zBlen}` });
  zB.style.fillOpacity = clamp((u - 3.8) / 0.5);
  zBl.style.opacity = spring(u - 3.9);
  // banners
  const b1 = spring(u - 3.35), b1o = clamp((5.0 - u) / 0.35);
  ban1.style.opacity = b1 * b1o; ban1.style.transform = `translate(-50%, ${(1 - b1) * -16}px)`;
  const b2 = spring(u - 5.0);
  ban2.style.opacity = b2; ban2.style.transform = `translate(-50%, ${(1 - b2) * -16}px)`;
  // drones fly to T1b
  dDr.forEach((g, i) => {
    const a = 5.5 + i * 0.15, p = easeInOut((u - a) / 2.2);
    const s = dStart[i], e = dEnd[i], c = { x: (s.x + e.x) / 2, y: Math.min(s.y, e.y) - 110 };
    const hover = { x: Math.sin(u * 2 + i) * 3, y: Math.cos(u * 2.3 + i) * 3 };
    const pos = quad(s, c, e, p);
    set(g, { transform: `translate(${(pos.x + hover.x).toFixed(1)},${(pos.y + hover.y).toFixed(1)})` });
  });
  dArcs.forEach((a, i) => {
    const len = 420, p = easeInOut((u - 5.1 - i * 0.1) / 0.6);
    set(a, { opacity: clamp(1 - (u - 8.2) / 0.5) * (u > 5.1 ? 0.8 : 0), 'stroke-dasharray': `${len * p} ${len}` });
  });
  zA.style.opacity = u > 7.5 ? mix(1, 0.5, clamp((u - 7.5) / 0.6)) : 1;
  logLis.forEach(({ li, at }) => { const p = spring(u - at); li.style.opacity = p; li.style.transform = `translateX(${(1 - p) * -16}px)`; });
}

// =====================================================================
// main render
// =====================================================================
const brand = $('#brand');
const blobs = $$('.blob');
const bgClip = { v: $('#s1 .bgclip'), at: 0, dur: 4.35 };
function render(t) {
  // aurora drift
  blobs[0].style.transform = `translate(${Math.sin(t * 0.21) * 90}px, ${Math.cos(t * 0.17) * 60}px)`;
  blobs[1].style.transform = `translate(${Math.cos(t * 0.19) * 100}px, ${Math.sin(t * 0.23) * 70}px)`;
  blobs[2].style.transform = `translate(${Math.sin(t * 0.13 + 1) * 140}px, ${Math.cos(t * 0.11) * 80}px)`;

  for (const s of Object.values(scene)) {
    const inP = spring(t - s.a, 6.5), outP = clamp((t - (s.b - 0.6)) / 0.6);
    const vis = t >= s.a && t <= s.b;
    const op = vis ? inP * (1 - easeInOut(outP)) : 0;
    s.el.style.opacity = op.toFixed(4);
    s.el.style.visibility = op > 0.001 ? 'visible' : 'hidden';
    s.el.style.transform = `scale(${(1.025 - 0.025 * inP - 0.02 * easeInOut(outP)).toFixed(4)})`;
    s.el.style.filter = (inP < 0.99 || outP > 0) ? `blur(${((1 - inP) * 10 + outP * 8).toFixed(2)}px)` : 'none';
    s.on = op > 0.001;
  }
  const b = clamp((t - 19.8) / 0.6) * clamp((89.3 - t) / 0.6);
  brand.style.opacity = b;

  for (const r of reveals) {
    if (!r.s.on) continue;
    const x = t - r.s.a - r.at;
    if (r.kind === 'r') {
      const p = spring(x, 7.5);
      r.el.style.opacity = clamp(p * 1.4).toFixed(3);
      r.el.style.transform = `translateY(${((1 - p) * 26).toFixed(2)}px)`;
      r.el.style.filter = p < 0.98 ? `blur(${((1 - p) * 6).toFixed(2)}px)` : 'none';
    } else {
      const step = r.kind === 'words' ? 0.065 : 0.045;
      r.parts.forEach((w, i) => {
        const p = spring(x - i * step, 7.5);
        w.style.opacity = clamp(p * 1.5).toFixed(3);
        w.style.transform = r.kind === 'words'
          ? `translateY(${((1 - p) * 0.75).toFixed(3)}em) rotateX(${((1 - p) * 40).toFixed(1)}deg)`
          : `translateY(${((1 - p) * 0.35).toFixed(3)}em) scale(${(0.9 + 0.1 * p).toFixed(3)})`;
      });
    }
  }
  for (const r of rolls) {
    if (!r.s.on) continue;
    const p = expoOut((t - r.s.a - r.at) / 1.6);
    const v = mix(r.from, r.to, p).toFixed(r.dec) + r.suf;
    if (v !== r.last) { r.el.textContent = v; r.last = v; }
  }
  for (const b2 of bars) {
    if (!b2.s.on) continue;
    b2.el.style.width = (b2.to * spring(t - b2.s.a - b2.at, 5)).toFixed(2) + '%';
  }
  if (scene.s4.on) renderS4(t - scene.s4.a);
  if (scene.s5.on) renderS5(t - scene.s5.a); else wclips.forEach((c) => (c.want = null));
  if (scene.s6.on) renderS6(t - scene.s6.a);
  bgClip.want = scene.s1.on ? clamp(t * 0.62, 0, bgClip.dur) : null; // slowed to fill the scene
  // final fade to black
  $('#stage').style.filter = t > DURATION - 1.0 ? `brightness(${clamp((DURATION - t) / 1.0).toFixed(3)})` : 'none';
}

// ---- video sync -------------------------------------------------------------
const vids = [bgClip, ...wclips];
function seekVideo(c) {
  return new Promise((res) => {
    if (c.want == null) return res();
    if (Math.abs(c.v.currentTime - c.want) < 0.004 && c.v.readyState >= 2) return res();
    const done = () => { c.v.removeEventListener('seeked', done); res(); };
    c.v.addEventListener('seeked', done);
    c.v.currentTime = c.want;
    setTimeout(done, 3000);
  });
}
window.__duration = DURATION;
window.__seek = async (t) => { render(t); await Promise.all(vids.map(seekVideo)); };
window.__ready = Promise.all([document.fonts.ready, ...vids.map((c) => new Promise((r) => (c.v.readyState >= 2 ? r() : c.v.addEventListener('loadeddata', r, { once: true }))))]);

// ---- live playback ---------------------------------------------------------
function fit() {
  const s = Math.min(innerWidth / 1920, innerHeight / 1080);
  $('#stage').style.transform = RENDER ? 'none' : `scale(${s})`;
}
addEventListener('resize', fit); fit();
let playing = false, t0 = 0, tNow = 0;
const btn = $('#play'), timeEl = $('#time');
function tick(now) {
  if (!playing) return;
  tNow = (now - t0) / 1000;
  if (tNow >= DURATION) { tNow = 0; t0 = now; }
  render(tNow);
  for (const c of vids) {
    if (c.want == null) { if (!c.v.paused) c.v.pause(); continue; }
    if (c === bgClip) { if (!c.v.paused) c.v.pause(); if (Math.abs(c.v.currentTime - c.want) > 0.08) c.v.currentTime = c.want; continue; }
    if (c.v.paused) { c.v.currentTime = c.want; c.v.play().catch(() => {}); }
    else if (Math.abs(c.v.currentTime - c.want) > 0.3) c.v.currentTime = c.want;
  }
  timeEl.textContent = tNow.toFixed(1) + 's';
  requestAnimationFrame(tick);
}
function play() { playing = true; t0 = performance.now() - tNow * 1000; btn.textContent = '❚❚ Pause'; requestAnimationFrame(tick); }
function pause() { playing = false; btn.textContent = '▶ Play'; vids.forEach((c) => c.v.pause()); }
btn.addEventListener('click', () => (playing ? pause() : play()));
addEventListener('keydown', (e) => { if (e.code === 'Space') { e.preventDefault(); playing ? pause() : play(); } });
const qt = new URLSearchParams(location.search).get('t');
if (qt) tNow = parseFloat(qt);
render(tNow);
if (!RENDER && !qt) window.__ready.then(() => setTimeout(play, 400));
})();
