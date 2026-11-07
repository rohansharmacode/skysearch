// Larpverse operator console. Reads data/state.json (contracts/INTERFACES.md, v1 + v1.1 grid extensions) every second.
// Works with the single-pile v1 state (sim/coordinator.py) and the 3x3 grid state (sim/grid_mission.py).
// ?state=data/<path>.json reads another state file (only relative paths under dashboard/data/; default data/state.json);
// the point cloud is resolved relative to that file's folder. ?poll=<ms> sets the poll interval (min 100 ms).
// #ping=<id> opens that ping's detail view. A missing state file shows the demo state.
import { demoState, demoCloud } from './demo.js';

const params = new URLSearchParams(location.search);
const STATE_URL = (() => {
  const q = params.get('state');
  return q && /^data\/(?:[\w-]+\/)*[\w-]+(?:\.[\w-]+)*\.json$/.test(q) && !q.split('/').includes('..') ? q : 'data/state.json';
})();
const DATA = STATE_URL.slice(0, STATE_URL.lastIndexOf('/'));
const POLL_MS = Math.max(100, +params.get('poll') || 1000), DEMO_POLL_MS = Math.max(POLL_MS, 5000);
const COL = { detect: '#ff4545', cone: '#ffe14a', coord: '#6fd36f', radar: '#ff9f43', teal: '#22c3d6', good: '#4fd18b', bad: '#ff6464', fg: '#e3eaef', muted: '#8a9caa', grid: 'rgba(150,180,200,0.10)' };
// status -> [long label, short label (small map scale), colour]
const STATUS = {
  unvisited: ['unvisited', 'unvisited', COL.muted], surveying: ['surveying', 'survey', COL.coord], scanning: ['radar scan', 'scan', COL.radar],
  ping: ['ping', 'ping', COL.detect], lead: ['acoustic lead', 'lead', COL.teal], radar_ping: ['acoustic lead', 'lead', COL.teal],
  dismissed_nonhuman: ['non-human', 'non-human', '#6d7d89'], no_signal_not_cleared: ['not cleared', 'not cleared', COL.muted],
};
const st = s => STATUS[s] || [s, s, COL.muted];
const $ = id => document.getElementById(id);
const f0 = v => (v == null ? '—' : (+v).toFixed(0));
const f1 = v => (v == null ? '—' : (+v).toFixed(1));
const f2 = v => (v == null ? '—' : (+v).toFixed(2));
const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

let S = null;                 // current state
let cloud = null, cloudKey = null;
let live = false, lastRaw = null;
const layers = { belief: true, rings: false, acoustic: true, sites: true, piles: true };
const isGrid = () => Array.isArray(S?.piles) && S.piles.length > 0;
// UI state that survives re-renders
let selListen = null;                                  // null = follow the latest listen
const openRows = new Set(), openWhy = new Set(), decOpen = new Map();

// ---------------- data ----------------
async function poll() {
  let ok = false;
  try {
    const r = await fetch(STATE_URL + '?_=' + Date.now(), { cache: 'no-store' });
    if (!r.ok) throw new Error(r.status);
    const raw = await r.text();
    if (raw !== lastRaw) {
      const s = JSON.parse(raw);
      if (s.schema !== 'larpverse.state/v1') throw new Error('schema ' + s.schema);
      lastRaw = raw; live = true; setState(s);
    }
    ok = true;
  } catch (e) {
    if (!S) { live = false; setState(demoState()); }
  }
  const c = $('conn');
  if (ok) { c.innerHTML = 'live<span class="src"> · state.json</span>'; c.className = 'conn live'; }
  else { c.textContent = live ? 'connection lost' : 'demo data'; c.className = 'conn ' + (live ? '' : 'demo'); }
  setTimeout(poll, ok || live ? POLL_MS : DEMO_POLL_MS);
}
async function ensureCloud() {
  const pc = S.pointcloud || {}; const key = pc.url ? pc.url + ':' + pc.count : 'demo:' + (isGrid() ? 'grid' : 'pile');
  if (key === cloudKey) return; cloudKey = key;
  if (pc.url) {
    try { const r = await fetch(DATA + '/' + pc.url + '?_=' + Date.now(), { cache: 'no-store' }); if (!r.ok) throw new Error(r.status); cloud = new Float32Array(await r.arrayBuffer()); return; }
    catch { /* fall through */ }
  }
  cloud = demoCloud();
}
let hmKey = null;
function setState(s) {
  const first = !S || (S.demo && !s.demo) || (!!S.piles !== !!s.piles);
  const hk = s.heightmap ? `${s.heightmap.origin}|${s.heightmap.w}x${s.heightmap.h}|${s.heightmap.z.length}|${s.heightmap.z[(s.heightmap.z.length / 3) | 0]}` : null;
  if (hk !== hmKey) { baseImg = null; rubbleImg = null; hmKey = hk; }
  S = s; beliefImg = null;
  ensureCloud();
  renderSide();
  if (first) fit();
  draw();
  linkPing();
}
let pingLinked = false;
function linkPing() {   // deep link: dashboard/#ping=A2-R1 (opens once the ping exists; re-armed by a hash change)
  const m = !pingLinked && S && location.hash.match(/^#ping=(.+)$/);
  if (m) { const c = (S.radar_candidates || []).find(c => c.id === decodeURIComponent(m[1])); if (c) { pingLinked = true; openDetail(c); } }
}
addEventListener('hashchange', () => { pingLinked = false; if (location.hash === '#key') setHelp(true); else linkPing(); });

// ---------------- honest numbers ----------------
// Depth: nominal (assumed constant) unless the coordinator says the radar measured it (depth_source / depth_sigma).
function depthInfo(c, surf) {
  if (surf === undefined) surf = S.heightmap ? hmAt(c.xy[0], c.xy[1]) : null;
  const src = String(c.depth_source || 'nominal').toLowerCase();
  const nominal = /nominal|assum|default|fixed|const/.test(src);
  let d = c.depth_m ?? c.depth ?? (c.z_est != null && surf != null ? surf - c.z_est : null);
  if (d == null) d = 1.5;
  const sig = c.depth_sigma ?? c.depth_sd ?? null, pm = sig != null ? ' ± ' + f1(sig) : '';
  return {
    d, sig, nominal,
    short: nominal ? `≈${f1(d)} m (nominal)` : `≈${f1(d)}${pm} m (radar estimate)`,
    long: nominal ? `≈${f1(d)} m below surface (nominal)` : `≈${f1(d)}${pm} m below surface (radar estimate)`,
    note: nominal ? 'assumed, not measured' : `measured from the radar slant ranges`,
  };
}
// Radar support for weak candidates: "2/3 ranges". Uses the coordinator's `support` field if present, else the evidence count.
function support(c) {
  const s = c.support;
  if (typeof s === 'number') return { n: s, of: c.support_needed ?? 3 };
  if (typeof s === 'string') { const m = s.match(/(\d+)\s*(?:\/|of)\s*(\d+)/); if (m) return { n: +m[1], of: +m[2] }; }
  if (s && typeof s === 'object') return { n: s.n_strong ?? s.n ?? s.strong ?? s.ranges ?? 0, of: s.needed ?? s.of ?? s.required ?? 3 };
  return { n: (c.evidence || []).length, of: 3 };
}
const isWeak = c => c.status !== 'candidate' && c.kind !== 'moving';
const supText = c => { const s = support(c); return `${s.n}/${s.of} ranges, rescan`; };
function byInfo(by) {
  const b = String(by || 'fallback').toLowerCase();
  if (b === 'fallback' || b === 'rules' || b === 'rule') return { cls: 'fb', txt: 'fallback', tip: 'Rule-based scorer: no LLM was used for this judgement' };
  if (b === 'argmax') return { cls: 'argmax', txt: 'argmax', tip: 'Deterministic rule: highest-priority valid option (no LLM decider configured)' };
  return { cls: 'llm', txt: b, tip: `Judged by an LLM (${b})` };
}
const vadOf = c => c.vad_speech_ratio ?? c.vad ?? (() => { const m = /VAD (\d(?:\.\d+)?)/.exec(c.rationale || ''); return m ? +m[1] : null; })();
function heardBy(c) {
  if (Array.isArray(c.heard_by) && c.heard_by.length) return c.heard_by.join(', ');
  if (c.per_drone && typeof c.per_drone === 'object') {
    const ids = Object.entries(c.per_drone).filter(([, v]) => v?.measured !== false).map(([k]) => k);
    if (ids.length) return ids.join(', ');
  }
  return null;
}
function provenance(c, withId = true) {
  const parts = [];
  if (withId) parts.push(`clip ${esc(c.id)}`);
  const hb = heardBy(c);
  parts.push(hb ? `heard by ${esc(hb)} (beam ${esc(c.drone_id)})` : `beam ${esc(c.drone_id)}`);
  if (c.doa_agree) parts.push(`${esc(c.doa_agree)} drones agree`);
  if (c.bearing_deg != null) parts.push(`${f0(c.bearing_deg)}° ±${f0((c.width_deg || 0) / 2)}°`);
  if (c.snr_db != null) parts.push(`SNR ${f0(c.snr_db)} dB`);
  const v = vadOf(c); if (v != null) parts.push(`VAD ${f2(v)}`);
  return parts.join(' · ');
}
const isAmb = c => c.association && c.association !== 'confident';
const ambChip = c => isAmb(c) ? `<span class="chip amb" title="The drones' bearings do not single out this pile${c.ambiguous_with?.length ? '; could be ' + esc(c.ambiguous_with.join(', ')) : ''}">ambiguous direction${c.ambiguous_with?.length ? ' · or ' + esc(c.ambiguous_with.join('/')) : ''}</span>` : '';
const SRC_TXT = { 'real-cached': 'Recorded audio · cached analysis', real: 'Live audio analysis', 'real-venv': 'Live audio analysis', custom: 'Custom acoustic source', demo: 'Demo data' };
const srcText = s => (/^stub/.test(s) ? 'Synthetic clips (from scenario)' : SRC_TXT[s] || s);

// ---------------- georeference + basemap tiles ----------------
// Site frame (metres, x east, y north) anchored at site.geo (default: the Weiguan Jinlong building, Tainan).
// bearing_deg = compass bearing of the site +y axis (0 = north). Local ENU approximation, fine over ~100 m.
const GEO_DEFAULT = { lat: 23.0051432, lon: 120.2608904, bearing_deg: 0, name: 'Weiguan Jinlong building (維冠金龍大樓), Yongkang District, Tainan' };
const geo = () => ({ ...GEO_DEFAULT, ...(S?.site?.geo || {}) });
const R_E = 6378137, RAD = Math.PI / 180;
function siteToLL(x, y) {
  const g = geo(), b = (g.bearing_deg || 0) * RAD, e = x * Math.cos(b) + y * Math.sin(b), n = -x * Math.sin(b) + y * Math.cos(b);
  return [g.lat + n / R_E / RAD, g.lon + e / (R_E * Math.cos(g.lat * RAD)) / RAD];
}
function llToG(lat, lon, z) { const n = 256 * 2 ** z, s = Math.sin(lat * RAD); return [(lon + 180) / 360 * n, (0.5 - Math.log((1 + s) / (1 - s)) / (4 * Math.PI)) * n]; }
const TILES = {
  map: { url: (z, x, y) => `https://tile.openstreetmap.org/${z}/${x}/${y}.png`, max: 19,
    filter: 'invert(0.92) hue-rotate(180deg) saturate(0.55) brightness(0.92) contrast(0.92)',
    attr: 'Map © <a href="https://www.openstreetmap.org/copyright" target="_blank" rel="noopener">OpenStreetMap</a> contributors' },
  sat: { url: (z, x, y) => `https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/${z}/${y}/${x}`, max: 19,
    filter: 'brightness(0.7) saturate(0.85)',
    attr: 'Imagery © <a href="https://www.esri.com/" target="_blank" rel="noopener">Esri</a>, Maxar, Earthstar Geographics, and the GIS User Community' },
};
let basemap = 'map';
const tileCache = new Map(), tileStat = { map: { ok: 0, fail: 0 }, sat: { ok: 0, fail: 0 } };
let tileVer = 0;
function tile(mode, z, x, y) {
  const k = `${mode}/${z}/${x}/${y}`; let t = tileCache.get(k);
  if (!t) {
    const img = new Image(); t = { img, ok: false, fail: false };
    img.onload = () => { t.ok = true; tileStat[mode].ok++; tileVer++; };
    img.onerror = () => { t.fail = true; tileStat[mode].fail++; tileVer++; if (tilesDead(mode)) renderBasemapUI(); };
    img.src = TILES[mode].url(z, x, y); tileCache.set(k, t);
  }
  return t;
}
const tilesDead = m => !!TILES[m] && tileStat[m].ok === 0 && tileStat[m].fail >= 3;   // offline: fall back to LiDAR relief
const tilesOn = () => !!TILES[basemap] && !tilesDead(basemap);
const bmCv = document.createElement('canvas'); let bmKey = '';
function renderTiles(mode) {
  bmCv.width = Math.max(1, W * dpr); bmCv.height = Math.max(1, Hh * dpr);
  const c = bmCv.getContext('2d'), T = TILES[mode], g = geo();
  const z = Math.max(1, Math.min(T.max, Math.ceil(Math.log2(156543.03392 * Math.cos(g.lat * RAD) * view.scale))));
  // site metres -> global pixel (relative to the site origin) at zoom z, as a local affine map
  const p0 = llToG(...siteToLL(0, 0), z), pe = llToG(...siteToLL(100, 0), z), pn = llToG(...siteToLL(0, 100), z);
  const a = (pe[0] - p0[0]) / 100, b = (pe[1] - p0[1]) / 100, cc = (pn[0] - p0[0]) / 100, d = (pn[1] - p0[1]) / 100, det = a * d - b * cc;
  const s = view.scale, ox = W / 2 - view.cx * s, oy = Hh / 2 + view.cy * s;
  const m11 = s * d / det, m12 = -s * cc / det, m21 = s * b / det, m22 = -s * a / det;    // screen = M (G - p0) + o
  const gs = [[0, 0], [W, 0], [0, Hh], [W, Hh]].map(([px, py]) => { const [x, y] = toM(px, py); return [a * x + cc * y + p0[0], b * x + d * y + p0[1]]; });
  const tx0 = Math.floor(Math.min(...gs.map(q => q[0])) / 256), tx1 = Math.floor(Math.max(...gs.map(q => q[0])) / 256);
  const ty0 = Math.floor(Math.min(...gs.map(q => q[1])) / 256), ty1 = Math.floor(Math.max(...gs.map(q => q[1])) / 256);
  if ((tx1 - tx0 + 1) * (ty1 - ty0 + 1) > 120) return;
  c.setTransform(dpr * m11, dpr * m21, dpr * m12, dpr * m22, dpr * ox, dpr * oy);
  c.filter = T.filter;
  for (let ty = ty0; ty <= ty1; ty++) for (let tx = tx0; tx <= tx1; tx++) {
    const t = tile(mode, z, tx, ty);
    if (t.ok) c.drawImage(t.img, tx * 256 - p0[0], ty * 256 - p0[1], 256.6, 256.6);
    else for (let k = 1; k <= 3; k++) {     // until it loads, stretch an already-loaded parent tile
      const pt = tileCache.get(`${mode}/${z - k}/${tx >> k}/${ty >> k}`); if (!pt?.ok) continue;
      const f = 256 >> k, sx = (tx - ((tx >> k) << k)) * f, sy = (ty - ((ty >> k) << k)) * f;
      c.drawImage(pt.img, sx, sy, f, f, tx * 256 - p0[0], ty * 256 - p0[1], 256.6, 256.6); break;
    }
  }
}
function drawBasemap() {
  if (!tilesOn()) return false;
  const key = [basemap, view.cx, view.cy, view.scale, W, Hh, dpr, tileVer, JSON.stringify(S?.site?.geo || null)].join('|');
  if (key !== bmKey) { bmKey = key; renderTiles(basemap); }
  ctx.drawImage(bmCv, 0, 0, W, Hh); return true;
}
function renderBasemapUI() {
  document.querySelectorAll('[data-bm]').forEach(b => { b.setAttribute('aria-pressed', b.dataset.bm === basemap); b.classList.toggle('dead', tilesDead(b.dataset.bm)); b.title = tilesDead(b.dataset.bm) ? 'Map tiles unavailable (offline): showing the LiDAR relief' : ''; });
  const on = tilesOn(); $('attrib').hidden = !on; if (on) $('attrib').innerHTML = TILES[basemap].attr;
  cv.classList.toggle('bmrelief', !on);
}

// ---------------- map ----------------
const cv = $('map'), ctx = cv.getContext('2d');
const view = { cx: 0, cy: 0, scale: 14 };      // site metres at canvas centre; px per metre
let W = 0, Hh = 0, dpr = 1;
const toPx = (x, y) => [W / 2 + (x - view.cx) * view.scale, Hh / 2 - (y - view.cy) * view.scale];
const toM = (px, py) => [view.cx + (px - W / 2) / view.scale, view.cy - (py - Hh / 2) / view.scale];
function resize() { const r = cv.getBoundingClientRect(); dpr = Math.min(devicePixelRatio || 1, 2); W = r.width; Hh = r.height; cv.width = W * dpr; cv.height = Hh * dpr; draw(); }
function fit() {
  resize(); const r = cv.getBoundingClientRect();
  const b = S?.site?.bounds || [-25, -25, 25, 25];
  view.cx = (b[0] + b[2]) / 2; view.cy = (b[1] + b[3]) / 2;
  if (isGrid()) {   // keep the whole site clear of the tool bar (top) and the legend + scale bar (bottom)
    const top = Math.max(50, $('tools').getBoundingClientRect().bottom - r.top + 10);
    const lg = $('legend').getBoundingClientRect(), bot = Math.max(70, (lg.height ? r.bottom - lg.top : 0) + 34);
    const avail = Math.max(100, r.height - top - bot);
    view.scale = Math.min((r.width - 24) / (b[2] - b[0]), avail / (b[3] - b[1]));
    const topPx = top + (avail - (b[3] - b[1]) * view.scale) / 2;
    view.cy = b[3] - (r.height / 2 - topPx) / view.scale;
  } else view.scale = Math.min(r.width / (b[2] - b[0]), r.height / (b[3] - b[1])) * 1.25;
  draw();
}

let baseImg = null, beliefImg = null, rubbleImg = null;
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
function makeRubble() {
  const g = S.heightmap; if (!g) return null;
  const z = (i, j) => g.z[Math.min(g.h - 1, Math.max(0, j)) * g.w + Math.min(g.w - 1, Math.max(0, i))];
  let zmax = 0.5; for (const v of g.z) zmax = Math.max(zmax, v);
  return gridImage(g, (i, j) => {
    const dx = (z(i + 1, j) - z(i - 1, j)) / (2 * g.res), dy = (z(i, j + 1) - z(i, j - 1)) / (2 * g.res);
    const shade = Math.max(0, Math.min(1, 0.62 + (-dx * 0.55 + dy * 0.55) / Math.sqrt(1 + dx * dx + dy * dy)));
    const h = z(i, j), k = 40 + 90 * shade + 40 * h / zmax, a = Math.max(0, Math.min(1, (h - 0.15) / 0.9));
    return [k * 1.06, k, k * 0.9, 225 * a];
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

// Labels are queued while drawing and placed at the end, highest priority first; movable labels slide
// vertically until they no longer overlap anything already placed (R1-021, R1-022).
const FONT_L = '500 11px "IBM Plex Mono", ui-monospace, monospace';
let LQ = [];
const overlaps = (a, b) => a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
function qLabel(t, x, y, color, bg = 'rgba(11,16,22,0.84)', prio = 5, stroke = null) {
  ctx.font = FONT_L; const w = ctx.measureText(t).width;
  LQ.push({ prio, r: { x: x - 3, y: y - 11, w: w + 6, h: 15 }, draw: dy => {
    ctx.font = FONT_L; ctx.textAlign = 'left'; ctx.fillStyle = bg; ctx.fillRect(x - 3, y - 11 + dy, w + 6, 15);
    if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = 1; ctx.strokeRect(x - 3.5, y - 11.5 + dy, w + 7, 16); }
    ctx.fillStyle = color; ctx.fillText(t, x, y + dy);
  } });
}
function flushLabels() {
  LQ.sort((a, b) => a.prio - b.prio);
  const placed = [], OFF = [0, 16, -16, 32, -32, 48, -48, 64];
  for (const L of LQ) {
    let dy = 0;
    if (!L.fixed) for (const o of OFF) { const r = { ...L.r, y: L.r.y + o }; if (!placed.some(p => overlaps(p, r))) { dy = o; break; } }
    placed.push({ ...L.r, y: L.r.y + dy }); L.draw(dy);
  }
  LQ = [];
}

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
  ctx.restore();
  if (text) {   // at the arrow's midpoint, offset perpendicular to it (R1-022)
    ctx.font = FONT_L; const w = ctx.measureText(text).width, mx = (sx + ex) / 2, my = (sy + ey) / 2;
    let nx = -Math.sin(ang), ny = Math.cos(ang); if (ny < 0) { nx = -nx; ny = -ny; }
    const off = 12 + Math.abs(nx) * w / 2;
    qLabel(text, mx + nx * off - w / 2, my + ny * off + 4, color, 'rgba(11,16,22,0.9)', 3);
  }
}

let hoverId = null;
function listensOf() { return [...new Set((S?.acoustic_clips || []).map(c => c.listen ?? 0))].sort((a, b) => a - b); }
function curListen() { const L = listensOf(); return selListen != null && L.includes(selListen) ? selListen : L[L.length - 1]; }

function draw(now = performance.now()) {
  if (!S || !W) return;
  LQ = [];
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, W, Hh);
  if (drawBasemap()) {   // real map underneath, then the drones' LiDAR rubble mounds on top (flat ground transparent)
    if (S.heightmap) { rubbleImg ||= makeRubble(); if (rubbleImg) drawGridImage(rubbleImg, S.heightmap); }
  } else if (S.heightmap) { baseImg ||= makeBase(); if (baseImg) drawGridImage(baseImg, S.heightmap); }
  // metric grid, 5 m (10 m on the grid site)
  const step = isGrid() ? 10 : 5;
  ctx.strokeStyle = COL.grid; ctx.lineWidth = 1; ctx.beginPath();
  const [mx0, my1] = toM(0, 0), [mx1, my0] = toM(W, Hh);
  for (let x = Math.ceil(mx0 / step) * step; x <= mx1; x += step) { const [px] = toPx(x, 0); ctx.moveTo(px, 0); ctx.lineTo(px, Hh); }
  for (let y = Math.ceil(my0 / step) * step; y <= my1; y += step) { const [, py] = toPx(0, y); ctx.moveTo(0, py); ctx.lineTo(W, py); }
  ctx.stroke();
  if (layers.belief && S.belief) { beliefImg ||= makeBelief(); if (beliefImg) { ctx.globalAlpha = 0.85; drawGridImage(beliefImg, S.belief); ctx.globalAlpha = 1; } }

  if (isGrid()) drawGridLayer(now);

  // acoustic cones: the selected listen (default: latest) (R1-025)
  if (layers.acoustic) {
    const clips = S.acoustic_clips;
    if (Array.isArray(clips) && clips.length) {
      const L = curListen();
      for (const c of clips) {
        if (c.bearing_deg == null || !c.origin || (c.listen ?? 0) !== L) continue;
        const p = pileById(c.pile_id), d = p ? Math.hypot(p.xy[0] - c.origin[0], p.xy[1] - c.origin[1]) + 5 : 30;
        const human = (c.scores?.human ?? 0) >= 0.4;
        if (human) cone(c.origin, c.bearing_deg, c.width_deg || 20, d, isAmb(c) ? 'rgba(255,225,74,0.08)' : 'rgba(255,225,74,0.15)', 'rgba(255,225,74,0.85)');
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
  // landing sites; skip squares that would sit under a pile's name/badge (R1-022)
  const pileRects = LQ.filter(l => l.pile).map(l => l.r);
  if (layers.sites) for (const l of S.landing_sites || []) {
    if (l.status === 'rejected' && !l.slope_deg) continue;
    const [x, y] = toPx(...l.xy), s = l.status === 'used' ? 9 : 5;
    const sq = { x: x - s / 2 - 2, y: y - s / 2 - 2, w: s + 4, h: s + 4 };
    if (pileRects.some(r => overlaps(r, sq))) continue;
    ctx.strokeStyle = l.status === 'rejected' ? COL.bad : COL.good; ctx.lineWidth = l.status === 'used' ? 2 : 1.2; ctx.globalAlpha = l.status === 'candidate' ? 0.6 : 1;
    ctx.strokeRect(x - s / 2, y - s / 2, s, s); ctx.globalAlpha = 1;
    if (l.status === 'used' && view.scale > 10) qLabel(l.id, x + 8, y + 12, COL.muted, 'rgba(11,16,22,0.7)', 9);
  }
  // next scan
  if (S.next_scan) {
    const site = (S.landing_sites || []).find(l => l.id === S.next_scan.site_id), dr = (S.drones || []).find(d => d.id === S.next_scan.drone_id);
    if (site) {
      const [x, y] = toPx(...site.xy); ctx.strokeStyle = COL.coord; ctx.setLineDash([5, 4]); ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(x, y, 12, 0, Math.PI * 2); ctx.stroke();
      if (dr) { const [dx, dy] = toPx(dr.pose[0], dr.pose[1]); ctx.beginPath(); ctx.moveTo(dx, dy); ctx.lineTo(x, y); ctx.stroke(); }
      ctx.setLineDash([]); qLabel('next scan', x + 14, y - 10, COL.coord, undefined, 4);
    }
  }
  // radar candidates (pings). Firm: solid red, pulsing. Insufficient evidence: hollow dashed orange, no pulse, no p (R1-018).
  const pulse = (now / 1200) % 1, small = W < 600;
  for (const c of S.radar_candidates || []) {
    const [x, y] = toPx(...c.xy), weak = isWeak(c), moving = c.kind === 'moving';
    if (c.cov) {
      const e = ellipse(c); ctx.save(); ctx.translate(x, y); ctx.rotate(-e.ang);
      ctx.beginPath(); ctx.ellipse(0, 0, Math.max(3, e.rx * view.scale), Math.max(3, e.ry * view.scale), 0, 0, Math.PI * 2);
      ctx.fillStyle = weak ? 'rgba(255,159,67,0.08)' : 'rgba(255,69,69,0.14)'; ctx.fill(); ctx.strokeStyle = weak ? COL.radar : COL.detect; ctx.setLineDash(weak ? [5, 4] : []); ctx.lineWidth = 1.6; ctx.stroke(); ctx.restore(); ctx.setLineDash([]);
    }
    const r = hoverId === c.id ? 9 : 7;
    if (moving) {
      ctx.fillStyle = COL.radar; ctx.beginPath(); ctx.moveTo(x, y - 8); ctx.lineTo(x + 8, y); ctx.lineTo(x, y + 8); ctx.lineTo(x - 8, y); ctx.closePath(); ctx.fill(); ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5; ctx.stroke();
    } else if (weak) {
      ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.fillStyle = 'rgba(11,16,22,0.75)'; ctx.fill();
      ctx.setLineDash([3, 2.5]); ctx.strokeStyle = COL.radar; ctx.lineWidth = 2.4; ctx.stroke(); ctx.setLineDash([]);
    } else {
      ctx.strokeStyle = `rgba(255,69,69,${1 - pulse})`; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(x, y, 7 + pulse * 22, 0, Math.PI * 2); ctx.stroke();
      ctx.fillStyle = COL.detect; ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.fill(); ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5; ctx.stroke();
    }
    const txt = weak ? `${c.id} · ${supText(c)}` : small ? c.id : `${c.id} · belief share ${f2(c.p_mass)}`;
    qLabel(txt, x + 12, y - 10, weak ? '#ffd9a6' : '#ffd0d0', weak ? 'rgba(58,35,6,0.92)' : 'rgba(60,8,8,0.88)', 1, weak ? 'rgba(255,159,67,0.7)' : null);
  }
  // drones; drones close together in the same state share one label ("d1–d3 · docked")
  const groups = [];
  for (const d of S.drones || []) {
    const [x, y] = toPx(d.pose[0], d.pose[1]), yaw = (d.pose[3] ?? 0) * Math.PI / 180;
    ctx.save(); ctx.translate(x, y); ctx.rotate(-yaw); ctx.beginPath(); ctx.moveTo(10, 0); ctx.lineTo(-6, 6); ctx.lineTo(-3, 0); ctx.lineTo(-6, -6); ctx.closePath();
    ctx.fillStyle = d.state === 'listening' ? COL.cone : COL.fg; ctx.fill(); ctx.strokeStyle = '#0b1016'; ctx.lineWidth = 1.5; ctx.stroke(); ctx.restore();
    const g = groups.find(g => g.state === d.state && Math.hypot(g.lx - x, g.ly - y) < 26);
    if (g) { g.ids.push(d.id); g.x = Math.min(g.x, x); g.lx = x; g.ly = y; } else groups.push({ state: d.state, ids: [d.id], x, y, lx: x, ly: y });
  }
  for (const g of groups) {
    const n = g.ids.map(i => +i.replace(/\D/g, '')), seq = g.ids.length > 2 && n.every((v, k) => k === 0 || v === n[k - 1] + 1);
    const ids = seq ? `${g.ids[0]}–${g.ids[g.ids.length - 1]}` : g.ids.join(' ');
    qLabel(`${ids} · ${g.state}`, g.x + 10, g.y + 18, COL.fg, undefined, 6);
  }
  flushLabels();
  scaleBar();
}

function drawGridLayer(now) {
  // staging area
  const stg = S.staging;
  if (stg) {
    const [x0, y0] = toPx(stg.xy[0] - 7, stg.xy[1] + 4), [x1, y1] = toPx(stg.xy[0] + 7, stg.xy[1] - 4);
    ctx.save(); ctx.fillStyle = 'rgba(90,169,255,0.08)'; ctx.strokeStyle = 'rgba(90,169,255,0.7)'; ctx.setLineDash([4, 3]); ctx.lineWidth = 1.2;
    ctx.fillRect(x0, y0, x1 - x0, y1 - y0); ctx.strokeRect(x0, y0, x1 - x0, y1 - y0); ctx.setLineDash([]);
    for (const p of stg.pads || []) { const [px, py] = toPx(...p); ctx.strokeStyle = 'rgba(90,169,255,0.9)'; ctx.strokeRect(px - 4, py - 4, 8, 8); }
    ctx.restore(); qLabel(W < 600 ? 'Staging' : 'Staging · operator console', x0 + 4, y1 + 16, '#9cc8ff', undefined, 7);
  }
  if (!layers.piles) return;
  const m = S.mission || {}, active = m.active_pile, short = view.scale < 6.5;
  // dispatch arrows: history faint, latest bright
  const ds = (S.dispatch || []).filter(d => d.to_pile && pileById(d.from_pile) && pileById(d.to_pile));
  ds.forEach((d, k) => {
    const latest = k === ds.length - 1, to = pileById(d.to_pile);
    arrow(pileById(d.from_pile).xy, to.xy, COL.coord, latest ? 0.95 : 0.35, latest ? `dispatch → ${d.to_pile}${to.priority != null && !short ? ' · priority ' + to.priority : ''}` : null);
  });
  for (const p of S.piles) {
    const [x, y] = toPx(...p.xy), [lng, sht, col] = st(p.status), txt = (short ? sht : lng).toUpperCase(), isActive = p.id === active;
    ctx.save();
    ctx.beginPath(); ctx.arc(x, y, 8.5 * view.scale, 0, Math.PI * 2);
    ctx.strokeStyle = col; ctx.globalAlpha = isActive ? 0.95 : 0.45; ctx.lineWidth = isActive ? 2.4 : 1.2;
    ctx.setLineDash(p.status === 'no_signal_not_cleared' || p.status === 'unvisited' ? [4, 5] : []); ctx.stroke(); ctx.setLineDash([]);
    if (isActive) { const k = (now / 1600) % 1; ctx.globalAlpha = 0.5 * (1 - k); ctx.beginPath(); ctx.arc(x, y, (8.5 + 3 * k) * view.scale, 0, Math.PI * 2); ctx.stroke(); }
    ctx.restore();
    // name, priority badge, status chip (above the pile), placed as one fixed block
    const ty = y - 8.5 * view.scale - 8;
    ctx.font = '600 15px "IBM Plex Sans Condensed", sans-serif'; const nw = ctx.measureText(p.id).width;
    ctx.font = '600 12px "IBM Plex Mono", monospace'; const pt = p.priority != null ? String(p.priority) : null, bw = pt ? ctx.measureText(pt).width + 10 : 0;
    ctx.font = '500 10.5px "IBM Plex Mono", monospace'; const cw = ctx.measureText(txt).width;
    const left = Math.min(x - nw / 2 - 5, x - cw / 2 - 5), right = Math.max(x + nw / 2 + 8 + bw, x + cw / 2 + 5);
    LQ.push({ prio: 0, fixed: true, pile: true, r: { x: left, y: ty - 15, w: right - left, h: 38 }, draw: () => {
      ctx.textAlign = 'left';
      ctx.font = '600 15px "IBM Plex Sans Condensed", sans-serif';
      ctx.fillStyle = 'rgba(11,16,22,0.85)'; ctx.fillRect(x - nw / 2 - 5, ty - 15, nw + 10, 20); ctx.fillStyle = COL.fg; ctx.fillText(p.id, x - nw / 2, ty);
      if (pt) {
        ctx.font = '600 12px "IBM Plex Mono", monospace'; const bx = x + nw / 2 + 8;
        ctx.fillStyle = p.status === 'lead' || p.priority >= (m.priority_min ?? 20) ? COL.teal : 'rgba(34,195,214,0.35)';
        roundRect(bx, ty - 14, bw, 18, 9); ctx.fill(); ctx.fillStyle = '#04161a'; ctx.fillText(pt, bx + 5, ty);
      }
      ctx.font = '500 10.5px "IBM Plex Mono", monospace';
      ctx.fillStyle = 'rgba(11,16,22,0.88)'; ctx.fillRect(x - cw / 2 - 5, ty + 7, cw + 10, 15); ctx.strokeStyle = col; ctx.lineWidth = 1; ctx.setLineDash(p.association === 'ambiguous' ? [3, 2] : []); ctx.strokeRect(x - cw / 2 - 5, ty + 7, cw + 10, 15); ctx.setLineDash([]);
      ctx.fillStyle = col; ctx.fillText(txt, x - cw / 2, ty + 18);
    } });
  }
}
function roundRect(x, y, w, h, r) { ctx.beginPath(); ctx.moveTo(x + r, y); ctx.arcTo(x + w, y, x + w, y + h, r); ctx.arcTo(x + w, y + h, x, y + h, r); ctx.arcTo(x, y + h, x, y, r); ctx.arcTo(x, y, x + w, y, r); ctx.closePath(); }
function scaleBar() {
  const nice = [1, 2, 5, 10, 20, 50], m = nice.find(v => v * view.scale > 70) || 50, px = m * view.scale, x = 14, y = Hh - (W < 900 ? (tilesOn() ? 62 : 40) : 44);
  ctx.strokeStyle = COL.fg; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + px, y); ctx.moveTo(x, y - 4); ctx.lineTo(x, y + 4); ctx.moveTo(x + px, y - 4); ctx.lineTo(x + px, y + 4); ctx.stroke();
  ctx.fillStyle = COL.fg; ctx.font = '500 11px "IBM Plex Mono", monospace'; ctx.textAlign = 'left'; ctx.fillText(`${m} m`, x + px + 6, y + 4);
  const b = (geo().bearing_deg || 0) * RAD, nx = W - 30, ny = 66;   // true north relative to the site frame
  ctx.save(); ctx.translate(nx, ny); ctx.rotate(-b); ctx.beginPath(); ctx.moveTo(0, -11); ctx.lineTo(5, 4); ctx.lineTo(0, 1); ctx.lineTo(-5, 4); ctx.closePath(); ctx.fill();
  ctx.textAlign = 'center'; ctx.fillText('N', 0, -14); ctx.restore(); ctx.textAlign = 'left';
}

// interaction
let drag = null;
cv.addEventListener('pointerdown', e => { drag = { x: e.clientX, y: e.clientY, cx: view.cx, cy: view.cy, moved: false }; cv.setPointerCapture(e.pointerId); });
cv.addEventListener('pointermove', e => {
  const r = cv.getBoundingClientRect(), [mx, my] = toM(e.clientX - r.left, e.clientY - r.top);
  const [la, lo] = siteToLL(mx, my); $('cursor').textContent = `x ${mx.toFixed(1)} · y ${my.toFixed(1)} m · ${la.toFixed(5)}° N ${lo.toFixed(5)}° E`;
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
document.querySelectorAll('[data-bm]').forEach(b => b.onclick = () => { basemap = b.dataset.bm; renderBasemapUI(); draw(); });
document.querySelectorAll('#layers .tog[data-k]').forEach(b => b.onclick = () => { const k = b.dataset.k; layers[k] = !layers[k]; b.setAttribute('aria-pressed', layers[k]); draw(); });
$('layersBtn').onclick = () => { const o = $('tools').classList.toggle('open'); $('layersBtn').setAttribute('aria-expanded', o); };
function setHelp(open) { if (open) { const t = $('tools'); $('help').style.top = (t.offsetTop + t.offsetHeight + 8) + 'px'; } $('help').hidden = !open; $('helpBtn').setAttribute('aria-expanded', open); if (open) { $('tools').classList.remove('open'); $('layersBtn').setAttribute('aria-expanded', false); } }
$('helpBtn').onclick = () => setHelp($('help').hidden);
$('helpClose').onclick = () => setHelp(false);
if (location.hash === '#key') setHelp(true);

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
  renderBanner();
  renderCands(grid);
  $('secAgent').hidden = !grid; $('secPiles').hidden = !grid; $('secAc').hidden = grid;
  if (grid) { renderPiles(); renderDecisions(); renderClips(); }
  else {
    $('secDec').hidden = true;
    const ac = S.acoustic || [];
    $('nAc').textContent = ac.length;
    $('acList').innerHTML = ac.length ? ac.map(a => `<div class="ac"><span>${esc(a.drone_id)} · <span class="c">${esc(a.class)}</span></span><span>${a.bearing_deg.toFixed(0)}° ±${(a.width_deg / 2).toFixed(0)} · ${f2(a.confidence)}</span></div>`).join('') : '<p class="empty">Nothing heard yet.</p>';
  }
  $('fleet').innerHTML = (S.drones || []).map(d => { const b = Math.round((d.battery ?? 0) * 100); return `<div class="fl"><b>${esc(d.id)}</b><span class="st ${esc(d.state)}">${esc(d.state)}</span><span class="bat" title="battery ${b}%"><i style="width:${b}%"></i></span><span class="pc">${b}%</span></div>`; }).join('');
  $('next').textContent = S.next_scan ? `Next: ${S.next_scan.drone_id} → ${S.next_scan.site_id} · ${S.next_scan.reason}` : '';
  $('log').innerHTML = (S.log || []).slice(-14).reverse().map(l => `<div><span>${f1(l.t)}s</span><p style="margin:0">${esc(l.msg)}</p></div>`).join('');
}

function renderBanner() {
  const b = $('banner'), src = String(S.mission?.acoustic_source || '');
  if (S.demo) {
    b.className = 'demo'; b.hidden = false;
    b.innerHTML = `<b>Demo data.</b> No mission is connected (<code>${esc(STATE_URL)}</code> not found), so everything on this screen is an illustrative example, not a real run.`;
  } else if (/^stub/.test(src)) {
    b.className = 'warn'; b.hidden = false;
    b.innerHTML = `<b>Synthetic acoustics.</b> Acoustic clips are synthetic, generated from the scenario, not sensed. Treat every acoustic lead and priority here as illustrative. <code>acoustic_source = ${esc(src)}</code>`;
  } else b.hidden = true;
}

function renderCands(grid) {
  const cands = S.radar_candidates || [];
  $('nCand').textContent = cands.length;
  $('candsTitle').textContent = grid ? 'Radar pings' : 'Radar candidates';
  const surf = c => (S.heightmap ? hmAt(c.xy[0], c.xy[1]) : null);
  if (!cands.length) { $('cands').innerHTML = '<p class="empty">No radar candidates yet. No detection is not proof of absence.</p>'; return; }
  $('cands').innerHTML = cands.map(c => {
    const e = c.cov ? ellipse(c) : null, weak = isWeak(c), mv = c.kind === 'moving', di = depthInfo(c, surf(c));
    const lab = mv ? 'moving track' : weak ? 'insufficient evidence' : 'candidate region';
    if (grid) return `<button class="prow ${weak ? 'weak' : ''}" data-id="${esc(c.id)}"><i class="mk"></i>
      <span class="l1"><b>${esc(c.id)}</b><small>${lab}</small></span><span class="go">3D view →</span>
      <span class="l2">${weak ? `<span class="sup">${esc(supText(c))}</span>` : `belief share <b>${f2(c.p_mass)}</b>`} · depth <b>${esc(di.short)}</b>${c.breathing_hz ? ` · ${(c.breathing_hz * 60).toFixed(0)}/min` : ''}</span></button>`;
    return `<button class="cand ${weak ? 'weak' : ''} ${mv ? 'moving' : ''}" data-id="${esc(c.id)}">
      <div class="t"><span>${esc(c.id)}${c.pile_id ? ` <small class="pl">pile ${esc(c.pile_id)}</small>` : ''}</span><small>${lab}</small></div>
      <div class="kv">${weak ? `<span>support<b class="sup">${esc(supText(c))}</b></span>` : `<span>belief share<b>${f2(c.p_mass)}</b></span>`}<span>breathing<b>${c.breathing_hz ? (c.breathing_hz * 60).toFixed(0) + '/min' : '—'}</b></span><span>depth<b>${esc(di.short)}</b></span>
      <span>95% region<b>${e ? f1(2 * e.rx) + '×' + f1(2 * e.ry) : '—'}</b></span><span>evidence<b>${(c.evidence || []).length} scans</b></span><span>at<b>${f1(c.xy[0])}, ${f1(c.xy[1])}</b></span></div>
      <div class="go">Open LiDAR view →</div></button>`;
  }).join('');
}
$('cands').addEventListener('click', e => { const b = e.target.closest('[data-id]'); if (b) openDetail((S.radar_candidates || []).find(c => c.id === b.dataset.id)); });

function renderPiles() {
  const m = S.mission || {}, vis = [...new Set(m.visited || [])], done = m.stage === 'done', pmin = m.priority_min ?? 20;
  const visited = vis.map(pileById).filter(Boolean);
  const rest = S.piles.filter(p => !vis.includes(p.id)).sort((a, b) => (b.priority ?? -1) - (a.priority ?? -1) || a.id.localeCompare(b.id));
  const topUnvisited = done ? null : rest.find(p => p.priority != null && p.priority >= pmin);
  $('pilesTitle').textContent = done ? 'Open leads for ground teams' : 'Pile priority';
  $('nPiles').textContent = `${vis.length} searched`;
  const row = p => {
    const [txt, , col] = st(p.status);
    return `<div class="row ${p === topUnvisited ? 'top' : ''} ${vis.includes(p.id) ? 'done' : ''} ${openRows.has(p.id) ? 'open' : ''}" data-pile="${esc(p.id)}" title="Click to expand">
      <b>${esc(p.id)}</b><span class="why"><span class="chip" style="color:${col};border-color:${col}">${esc(txt)}</span>${p.association === 'ambiguous' ? '<span class="chip amb">ambiguous direction</span>' : ''} ${esc(p.why)}</span>
      <span class="s">${p.priority ?? '—'}</span></div>`;
  };
  $('piles').innerHTML = (visited.length ? `<div class="sub">Searched by the drones</div>${visited.map(row).join('')}` : '')
    + `<div class="sub">${done ? 'Not searched: ranked by acoustic priority' : 'Not yet searched: ranked by acoustic priority'}</div>${rest.map(row).join('')}`;
  const d = (S.dispatch || []).slice(-1)[0];
  $('dispatch').classList.toggle('summary', !!(done && m.summary)); $('dispatch').title = done && m.summary ? 'Click to expand' : '';
  $('dispatch').textContent = done && m.summary ? m.summary.replace(/^MISSION SUMMARY:\s*/, 'Mission summary: ')
    : !(S.decisions || []).length && d ? (d.to_pile ? `Dispatch ${d.from_pile} → ${d.to_pile}: ${d.reason}` : `${d.from_pile}: search loop ended (${d.reason})`) : '';
}
$('dispatch').addEventListener('click', () => $('dispatch').classList.toggle('open'));
$('piles').addEventListener('click', e => { const r = e.target.closest('.row'); if (!r) return; const id = r.dataset.pile; openRows.has(id) ? openRows.delete(id) : openRows.add(id); r.classList.toggle('open'); });

// Dispatch decisions (R1-023): options with flags, the chosen one highlighted, who decided.
function renderDecisions() {
  const ds = S.decisions || [];
  $('secDec').hidden = !ds.length; $('nDec').textContent = ds.length;
  if (!ds.length) return;
  const act = o => o.action === 'dispatch' ? `dispatch → ${o.pile_id}` : o.action === 'stop' ? 'stop · return to pads' : o.action === 'rescan_current' ? `rescan ${o.pile_id ?? 'current pile'}` : o.action;
  $('decs').innerHTML = ds.map((d, k) => {
    const key = String(d.t), open = decOpen.has(key) ? decOpen.get(key) : k === ds.length - 1;
    const ch = (d.options || []).find(o => o.id === d.chosen), by = byInfo(d.by);
    const from = (S.dispatch || []).find(x => Math.abs(x.t - d.t) < 0.01)?.from_pile;
    const opts = (d.options || []).map(o => `<div class="opt ${o.id === d.chosen ? 'chosen' : ''}">
        <span>${esc(o.id)}</span><span class="oa">${esc(act(o))}</span>
        <span class="on">${[o.priority != null ? 'prio ' + o.priority : '', o.travel_s != null ? f0(o.travel_s) + ' s' : '', o.battery_after_min != null ? 'bat ≥' + Math.round(o.battery_after_min * 100) + '%' : ''].filter(Boolean).join(' · ')}</span>
        ${o.evidence ? `<span class="ev" title="${esc(o.evidence)}">${esc(o.evidence)}</span>` : ''}
        ${(o.flags || []).length ? `<span class="fg">${o.flags.map(f => `<span class="chip">${esc(f.replace(/_/g, ' '))}</span>`).join('')}</span>` : ''}</div>`).join('');
    return `<div class="dec">
      <button class="dh" data-dec="${esc(key)}" aria-expanded="${open}"><span><b>t ${f0(d.t)} s</b> · ${esc(d.gate || 'gate')}${from ? ' at ' + esc(from) : ''} · <span class="pick">${esc(ch ? act(ch) : d.chosen)}</span></span>
        <span><span class="by ${by.cls}" title="${esc(by.tip)}">${esc(by.txt)}</span>${d.latency_ms ? ` <span>${f0(d.latency_ms)} ms</span>` : ''} ${open ? '▴' : '▾'}</span></button>
      ${open ? `<div class="db"><div class="rat">${esc(d.rationale)}</div>${opts}</div>` : ''}</div>`;
  }).reverse().join('');
}
$('decs').addEventListener('click', e => {
  const b = e.target.closest('.dh'); if (!b) return;
  decOpen.set(b.dataset.dec, b.getAttribute('aria-expanded') !== 'true'); renderDecisions();
});

function renderClips() {
  const ag = S.acoustic_agent || {}, src = String(S.mission?.acoustic_source || '');
  $('agentSrc').textContent = src ? srcText(src) : 'waiting';
  $('agentSrc').title = src ? `acoustic_source = ${src}` : '';
  const calls = (ag.llm_calls || 0) + (ag.cache_hits || 0);
  $('agentMeta').innerHTML = !src ? '' : calls
    ? `Judge (last listen): <b>LLM${ag.model ? ' · ' + esc(ag.model) : ''}</b>, ${ag.llm_calls || 0} calls, ${ag.cache_hits || 0} replayed, ${ag.fallback || 0} rule-based${ag.error ? ` · <span class="err">error: ${esc(ag.error)}</span>` : ''}`
    : `Judge (last listen): <b>rule-based scorer</b>, no LLM calls${ag.fallback ? ` (${ag.fallback} clips)` : ''}${ag.error ? ` · <span class="err">LLM error: ${esc(ag.error)}</span>` : ''}`;
  const clips = S.acoustic_clips || [], L = listensOf(), cur = curListen();
  $('listens').innerHTML = L.length > 1 ? L.map(l => { const t = clips.find(c => (c.listen ?? 0) === l)?.t;
    return `<button role="tab" data-l="${l}" aria-selected="${l === cur}">Listen ${l}${t != null ? ' · t ' + f0(t) + ' s' : ''}${l === L[L.length - 1] ? ' (latest)' : ''}</button>`; }).join('') : '';
  const list = clips.filter(c => (c.listen ?? 0) === cur).sort((a, b) => (b.priority ?? 0) - (a.priority ?? 0));
  $('clips').innerHTML = list.length ? list.map(c => {
    const h = c.scores?.human ?? 0, nonhuman = h < 0.4, by = byInfo(c.by);
    return `<div class="clip ${nonhuman ? 'nonhuman' : ''} ${isAmb(c) ? 'amb' : ''}">
      <div class="hd"><span><b>${esc(c.pile_id)}</b> · listen ${c.listen ?? '—'}${c.t != null ? ' · t ' + f0(c.t) + ' s' : ''}</span><span><span class="cls">${esc(c.class)}</span> <span class="by ${by.cls}" title="${esc(by.tip)}">${esc(by.txt)}</span></span></div>
      ${c.transcript ? `<q>${esc(c.transcript)}</q>` : ''}
      <div class="prov">${provenance(c)}</div>${ambChip(c)}
      <div class="scores"><span>human${bar(h)}</span><span>urgency${bar(c.scores?.urgency)}</span><span>reliability${bar(c.scores?.reliability)}</span><span class="pr" title="acoustic priority 0-100">${c.priority ?? '—'}<small>priority</small></span></div>
      ${c.rationale ? `<div class="why ${openWhy.has(c.id) ? 'open' : ''}" data-clip="${esc(c.id)}" title="Click to expand">${esc(c.rationale)}</div>` : ''}</div>`;
  }).join('') : '<p class="empty">Listening starts once the drones have landed and the radar phase at the first pile is over.</p>';
}
$('listens').addEventListener('click', e => { const b = e.target.closest('[data-l]'); if (!b) return; const l = +b.dataset.l, L = listensOf(); selListen = l === L[L.length - 1] ? null : l; renderClips(); draw(); });
$('clips').addEventListener('click', e => { const w = e.target.closest('.why'); if (!w) return; const id = w.dataset.clip; openWhy.has(id) ? openWhy.delete(id) : openWhy.add(id); w.classList.toggle('open'); });

function hmAt(x, y) {
  const g = S.heightmap; const i = (x - g.origin[0]) / g.res - 0.5, j = (y - g.origin[1]) / g.res - 0.5;
  const i0 = Math.max(0, Math.min(g.w - 2, Math.floor(i))), j0 = Math.max(0, Math.min(g.h - 2, Math.floor(j))), u = i - i0, v = j - j0, z = (a, b) => g.z[b * g.w + a];
  return (z(i0, j0) * (1 - u) + z(i0 + 1, j0) * u) * (1 - v) + (z(i0, j0 + 1) * (1 - u) + z(i0 + 1, j0 + 1) * u) * v;
}

// ---------------- detail: LiDAR view with depth ping ----------------
let viewer = null;
async function openDetail(c) {
  if (!c) return;
  $('detail').hidden = false;
  const surf = S.heightmap ? hmAt(c.xy[0], c.xy[1]) : 0, di = depthInfo(c, surf), z = c.z_est ?? surf - di.d, e = c.cov ? ellipse(c) : null;
  const weak = isWeak(c), moving = c.kind === 'moving';
  $('dTitle').textContent = c.id;
  $('dStatus').textContent = moving ? 'Moving track · tracked separately' : weak ? `Insufficient evidence · ${supText(c)}` : 'Candidate search region · not a confirmed person';
  $('dStatus').className = 'status' + (weak || moving ? ' weak' : '');
  const pile = c.pile_id ? pileById(c.pile_id) : null;
  const rows = [
    ...(pile ? [['Pile', `${esc(pile.id)} · ${esc(st(pile.status)[0])}${pile.priority != null ? ' · acoustic priority ' + pile.priority : ''}`]] : []),
    ['Position (E, N)', `${f1(c.xy[0])}, ${f1(c.xy[1])} m`],
    ['Surface (LiDAR)', `${f1(surf)} m`],
    ['Depth', `${esc(di.long)}<small>${esc(di.note)}</small>`],
    ['95% region', e ? `${f1(2 * e.rx)} × ${f1(2 * e.ry)} m` : '—'],
    weak ? ['Radar support', `<span style="color:var(--radar)">${esc(supText(c))}</span><small>too few independent ranges agree for a fix; belief share not shown</small>`]
      : ['Belief share', `${f2(c.p_mass)}<small>share of this pile's radar belief at this spot; not the probability that a person is there</small>`],
    ['Periodic signal', c.breathing_hz ? `${f2(c.breathing_hz)} Hz (${(c.breathing_hz * 60).toFixed(0)}/min), respiration-like` : '—'],
    ['Motion', moving ? 'Doppler shows bulk motion' : 'stationary'],
  ];
  $('dFacts').innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
  // What was heard at this pile, with provenance (R1-029)
  const heard = (S.acoustic_clips || []).filter(a => c.pile_id && a.pile_id === c.pile_id && (a.scores?.human ?? 0) >= 0.4).sort((a, b) => (a.listen ?? 0) - (b.listen ?? 0));
  $('dHeard').innerHTML = !isGrid() ? '' : `<h4>Heard at this pile</h4>` + (heard.length ? heard.map(a => {
    const p = pileById(a.pile_id), dist = p && a.origin ? Math.hypot(p.xy[0] - a.origin[0], p.xy[1] - a.origin[1]) : null, by = byInfo(a.by);
    return `<div class="heard">${a.transcript ? `<q>${esc(a.transcript)}</q>` : ''}
      <span><span class="cls">${esc(a.class)}</span> · priority ${a.priority ?? '—'} · listen ${a.listen ?? '—'}${a.t != null ? ' (t ' + f0(a.t) + ' s)' : ''} · <span class="by ${by.cls}" title="${esc(by.tip)}">${esc(by.txt)}</span></span>
      <span class="prov">${provenance(a)}${dist != null ? ` · from ${f0(dist)} m away` : ''}</span>${ambChip(a)}</div>`;
  }).join('') : '<p class="empty">Nothing human-like was heard from this pile.</p>');
  $('dCaveat').textContent = (weak ? 'Insufficient evidence: rescan before sending a team to this exact spot. ' : '')
    + (di.nominal ? 'Depth is not measured: the ping is drawn at a nominal depth below the LiDAR surface. ' : 'Depth is the LiDAR surface height minus the radar depth estimate (± 1σ). ')
    + 'Geometry says nothing about structural safety. Confirm on site before any excavation.';
  const ev = new Set(c.evidence || []);
  const scans = (S.scans || []).filter(s => !c.pile_id || !s.pile_id || s.pile_id === c.pile_id);
  $('dEv').querySelector('tbody').innerHTML = scans.map(s => `<tr class="${ev.has(s.scan_id) ? '' : 'low'}"><td>${esc(s.scan_id)}${ev.has(s.scan_id) ? '' : ' ·'}</td><td>${esc(s.drone_id)}</td><td>${(s.range_candidates || []).map(f1).join(', ') || 'none'}</td><td>${(s.confidence || []).map(f2).join(', ') || '—'}</td><td>${f2(s.quality)}</td></tr>`).join('');
  await ensureCloud();
  try {
    viewer ||= await import('./viewer3d.js').then(m => m.createViewer($('v3d')));
    $('vhint').hidden = false;
    viewer.show({ cloud, center: [c.xy[0], c.xy[1]], surf, z, ellipse: e, id: c.id, depthLabel: di.long, weak: weak || moving });
  } catch (err) {
    $('v3d').innerHTML = '<div class="v3dmsg">3D view unavailable: three.js could not load (offline?). The facts on the right still apply.</div>';
    $('vhint').hidden = true;
  }
}
function closeDetail() { $('detail').hidden = true; viewer?.stop(); }
$('dClose').onclick = closeDetail;
$('detail').addEventListener('click', e => { if (e.target.id === 'detail') closeDetail(); });
addEventListener('keydown', e => { if (e.key === 'Escape') { if (!$('detail').hidden) closeDetail(); else { setHelp(false); setCase(false); } } });

// ---------------- case study ----------------
function setCase(open) {
  $('casePop').hidden = !open; $('caseBtn').setAttribute('aria-expanded', open);
  if (open) { const g = geo(); $('caseGeo').textContent = `Map anchor: ${g.name}, ${(+g.lat).toFixed(5)}° N, ${(+g.lon).toFixed(5)}° E (source: zh.wikipedia 維冠金龍大樓). Site frame: metres, x east, y north, origin at the building, bearing ${f0(g.bearing_deg || 0)}°${S?.site?.geo ? ' (from state.json)' : ' (default)'}.`; }
}
$('caseBtn').onclick = () => setCase($('casePop').hidden);
$('caseClose').onclick = () => setCase(false);

// ---------------- boot ----------------
new ResizeObserver(() => resize()).observe(cv);
renderBasemapUI();
resize();
poll();
(function tick(now) { draw(now); requestAnimationFrame(tick); })(performance.now());
