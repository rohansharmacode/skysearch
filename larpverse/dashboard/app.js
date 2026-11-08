// Larpverse operator console. Reads data/state.json (contracts/INTERFACES.md, v1 + v1.1 grid extensions) every second.
// Works with the single-pile v1 state (sim/coordinator.py) and the 3x3 grid state (sim/grid_mission.py).
// ?state=data/<path>.json reads another state file (only relative paths under dashboard/data/ or dashboard/fixtures/;
// default data/state.json); ?data=data/<dir> is an alias for ?state=data/<dir>/state.json (R2-001).
// The point cloud is resolved relative to that file's folder. ?poll=<ms> sets the poll interval (min 100 ms).
// #ping=<id> opens that ping's detail view. A missing state file shows the demo state.
import { demoState, demoCloud } from './demo.js';
import { createTasking, reconText } from './tasking.js';
import { createComms, linkChip, evidenceChip } from './comms.js';
import { createSpeaker, respChip } from './speaker.js';
import { createLive } from './live.js';
import { digScore, triageOf, conditionOf, conditionLabel, conditionShort, confMeter, rateOf, spark, caveatsOf, triageChip, drawTriageRing, ALWAYS, sourceOf, isRejected, signatureAsset } from './vitals.js';

const params = new URLSearchParams(location.search);
const STATE_URL = (() => {
  const d = params.get('data'), q = params.get('state') || (d ? d.replace(/\/+$/, '') + '/state.json' : null);
  return q && /^(?:data|fixtures)\/(?:[\w-]+\/)*[\w-]+(?:\.[\w-]+)*\.json$/.test(q) && !q.split('/').includes('..') ? q : 'data/state.json';
})();
const DATA = STATE_URL.slice(0, STATE_URL.lastIndexOf('/')).replace(/\/frames$/, '');   // frames/state_NNNN.json share the run's pointcloud.bin
const POLL_MS = Math.max(100, +params.get('poll') || 1000), DEMO_POLL_MS = Math.max(POLL_MS, 5000);
// ?layout=take: the video's right half (960x1080, docs/VIDEO_PLAN.md D1): big map, dig order, LIVE line, radar waveform,
// top clip; nothing else. live.json / hud.json (next to the state file) are polled in take mode or with ?live=1.
const TAKE = params.get('layout') === 'take', LIVE_FILES = TAKE || params.get('live') === '1';
document.body.classList.toggle('take', TAKE);
const COL = { detect: '#ff4545', cone: '#ffe14a', coord: '#6fd36f', radar: '#ff9f43', teal: '#22c3d6', good: '#4fd18b', bad: '#ff6464', fg: '#e3eaef', muted: '#8a9caa', grid: 'rgba(150,180,200,0.10)' };
// status -> [long label, short label (small map scale), colour]
const STATUS = {
  unvisited: ['unvisited', 'unvisited', COL.muted], surveying: ['surveying', 'survey', COL.coord], scanning: ['radar scan', 'scan', COL.radar],
  ping: ['ping', 'ping', COL.detect], lead: ['acoustic lead', 'lead', COL.teal], radar_ping: ['ping', 'ping', COL.detect],   // R2-017
  dismissed_nonhuman: ['non-human sound · not cleared', 'non-human', COL.muted, 'non-human · not cleared'], no_signal_not_cleared: ['not cleared', 'not cleared', COL.muted],   // R2-006
  uncertain_sound: ['uncertain sound · not cleared', 'uncertain', COL.muted, 'uncertain · not cleared'],
};
const st = s => STATUS[s] || [s, s, COL.muted];
// A lead whose evidence is only ambiguous / low-association clips reads "ambiguous lead" (pile.association, the
// "Lead (ambiguous" why prefix, or every human-like clip of that pile with association != confident or conf < 0.6).
function pileAmb(p) {
  if (p.association === 'ambiguous' || /^Lead \(ambiguous/.test(p.why || '')) return true;
  if (p.association === 'confident' || !['lead', 'radar_ping'].includes(p.status)) return false;
  const cl = (S?.acoustic_clips || []).filter(c => c.pile_id === p.id && (c.scores?.human ?? 0) >= 0.4 && c.forwarded !== false);
  return cl.length > 0 && cl.every(c => isAmb(c) || (c.association_conf != null && c.association_conf < 0.6));
}
const pst = p => { const v = st(p.status); return pileAmb(p) && ['lead', 'radar_ping'].includes(p.status) ? ['ambiguous lead', 'amb. lead', v[2], 'ambiguous lead'] : v; };
const $ = id => document.getElementById(id);
const f0 = v => (v == null ? '—' : (+v).toFixed(0));
const f1 = v => (v == null ? '—' : (+v).toFixed(1));
const f2 = v => (v == null ? '—' : (+v).toFixed(2));
const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

let S = null;                 // current state
let cloud = null, cloudKey = null;
let live = false, lastRaw = null;
const layers = { belief: true, rings: false, acoustic: true, sites: true, piles: true, reach: true, comms: true, speaker: true };
const isGrid = () => (Array.isArray(S?.piles) && S.piles.length > 0) || !!S?.recon || S?.mission?.kind === 'grid';
// UI state that survives re-renders
let selListen = null;                                  // null = follow the latest listen
const openRows = new Set(), openWhy = new Set(), decOpen = new Map();
let dismOpen = false, showAllPiles = false;
let DIG = [], DIG_BY = new Map(), REJ = [];   // REJ: pings the radar rejector says are not a person (kept out of the dig order)
let rejOpen = false;
let TK = null, CM = null, SP = null, LV = null;   // LV: live.json / hud.json (live.js)   // SP: loudspeaker panel + guidance overlay (speaker.js)   // CM: rescue hotspot / comms relay (comms.js)                          // operator tasking + recon overlay (tasking.js)       // dig order (ranked pings), recomputed per state

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
  const hk = s.heightmap ? `${s.heightmap.origin}|${s.heightmap.w}x${s.heightmap.h}|${s.heightmap.z.length}|${s.heightmap.z[(s.heightmap.z.length / 3) | 0]}|${s.heightmap.rev ?? ''}|${s.recon?.mapped_fraction ?? ''}` : null;
  if (hk !== hmKey) { baseImg = null; rubbleImg = null; hmKey = hk; }
  for (const p of s.piles || []) if (!p.xy && polyOf(p)) { const P = polyOf(p); p.xy = [P.reduce((a, q) => a + q[0], 0) / P.length, P.reduce((a, q) => a + q[1], 0) / P.length]; }
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
let basemap = 'relief', basemapUser = false;   // R2-016: the synthetic piles should not sit on real street labels by default
// During recon the fresh LiDAR relief is drawn over the (pre-collapse) satellite imagery unless the operator picked a basemap.
const bmMode = () => (!basemapUser && S?.recon && !tilesDead('sat') ? 'sat' : basemap);
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
const tilesOn = () => !!TILES[bmMode()] && !tilesDead(bmMode());
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
  const basemap = bmMode(), key = [basemap, view.cx, view.cy, view.scale, W, Hh, dpr, tileVer, JSON.stringify(S?.site?.geo || null)].join('|');
  if (key !== bmKey) { bmKey = key; renderTiles(basemap); }
  ctx.globalAlpha = basemap === 'map' ? 0.55 : 0.8; ctx.drawImage(bmCv, 0, 0, W, Hh); ctx.globalAlpha = 1; return true;
}
function renderBasemapUI() {
  const basemap = bmMode();
  document.querySelectorAll('[data-bm]').forEach(b => { b.setAttribute('aria-pressed', b.dataset.bm === basemap); b.classList.toggle('dead', tilesDead(b.dataset.bm)); b.title = tilesDead(b.dataset.bm) ? 'Map tiles unavailable (offline): showing the LiDAR relief' : ''; });
  const on = tilesOn(); $('attrib').hidden = !on; if (on) $('attrib').innerHTML = TILES[basemap].attr;
  cv.classList.toggle('bmrelief', !on);
  $('reconTag').hidden = !(S?.recon && on);
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
    const lg = $('legend').getBoundingClientRect();
    const top = TAKE ? 14 : Math.max(50, $('tools').getBoundingClientRect().bottom - r.top + 10);
    const bot = TAKE ? (lg.height ? r.bottom - lg.top : 0) + 34 : Math.max(70, (lg.height ? r.bottom - lg.top : 0) + 34);
    const avail = Math.max(100, r.height - top - bot);
    view.scale = Math.min((r.width - 24) / (b[2] - b[0]), avail / (b[3] - b[1]));
    const topPx = top + (avail - (b[3] - b[1]) * view.scale) / 2;
    view.cy = b[3] - (r.height / 2 - topPx) / view.scale;
  } else view.scale = Math.min(r.width / (b[2] - b[0]), r.height / (b[3] - b[1])) * 1.25;
  draw();
}

let baseImg = null, beliefImg = null, rubbleImg = null;
function gridImage(g, fn, maxCells = 200000) {   // large grids (640x640 collapse site) are sampled every `st` cells
  const st = Math.max(1, Math.ceil(Math.sqrt(g.w * g.h / maxCells))), w = Math.ceil(g.w / st), h = Math.ceil(g.h / st);
  const c = document.createElement('canvas'); c.width = w; c.height = h;
  const x = c.getContext('2d'), im = x.createImageData(w, h);
  for (let jj = 0; jj < h; jj++) for (let ii = 0; ii < w; ii++) {
    const o = ((h - 1 - jj) * w + ii) * 4, [r, gg, b, a] = fn(Math.min(g.w - 1, ii * st), Math.min(g.h - 1, jj * st)); // flip: row j=0 is south
    im.data[o] = r; im.data[o + 1] = gg; im.data[o + 2] = b; im.data[o + 3] = a;
  }
  x.putImageData(im, 0, 0); return c;
}
// Progressive (recon) maps: cells with z null / NaN, or heightmap.mask[k] == 0, are not mapped yet and stay transparent.
const hmKnown = (g, i, j) => { const k = j * g.w + i, v = g.z[k]; return v != null && Number.isFinite(+v) && !(g.mask && !g.mask[k]); };
const hmMax = g => { let m = 0.5; for (const v of g.z) if (v != null && +v > m) m = +v; return m; };
function makeBase() {
  const g = S.heightmap; if (!g) return null;
  const z = (i, j) => +g.z[Math.min(g.h - 1, Math.max(0, j)) * g.w + Math.min(g.w - 1, Math.max(0, i))] || 0;
  const zmax = hmMax(g);
  return gridImage(g, (i, j) => {
    if (!hmKnown(g, i, j)) return [0, 0, 0, 0];
    const dx = (z(i + 1, j) - z(i - 1, j)) / (2 * g.res), dy = (z(i, j + 1) - z(i, j - 1)) / (2 * g.res);
    const shade = Math.max(0, Math.min(1, 0.62 + (-dx * 0.55 + dy * 0.55) / Math.sqrt(1 + dx * dx + dy * dy)));
    const e = z(i, j) / zmax, k = 26 + 70 * shade + 40 * e;
    return [k * 1.05, k, k * 0.92, 255];
  });
}
function makeRubble() {
  const g = S.heightmap; if (!g) return null;
  const z = (i, j) => +g.z[Math.min(g.h - 1, Math.max(0, j)) * g.w + Math.min(g.w - 1, Math.max(0, i))] || 0;
  const zmax = hmMax(g), live = !!S.recon;
  return gridImage(g, (i, j) => {
    if (!hmKnown(g, i, j)) return [0, 0, 0, 0];
    const dx = (z(i + 1, j) - z(i - 1, j)) / (2 * g.res), dy = (z(i, j + 1) - z(i, j - 1)) / (2 * g.res);
    const shade = Math.max(0, Math.min(1, 0.62 + (-dx * 0.55 + dy * 0.55) / Math.sqrt(1 + dx * dx + dy * dy)));
    const h = z(i, j), k = 40 + 90 * shade + 40 * h / zmax, a = Math.max(0, Math.min(1, (h - 0.15) / 0.9));
    return live ? [k * 1.06, k, k * 0.9, 120 + 105 * a] : [k * 1.06, k, k * 0.9, 225 * a];   // recon: the whole fresh map shows over the stale imagery
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
const TS = TAKE ? 1.4 : 1;
const LEAN = () => TAKE && innerWidth <= 800;   // narrow take (640 px right third): names, priorities and ranked pings only   // take layout: map text 1.4x so it reads at 1080p video scale (11 px -> ~15 px)
const FONT_L = `500 ${11 * TS}px "IBM Plex Mono", ui-monospace, monospace`;
let LQ = [];
const overlaps = (a, b) => a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
function qLabel(t, x, y, color, bg = 'rgba(11,16,22,0.84)', prio = 5, stroke = null) {
  ctx.font = FONT_L; const w = ctx.measureText(t).width;
  const a = 3 * TS, up = 11 * TS, hh = 15 * TS;
  LQ.push({ prio, r: { x: x - a, y: y - up, w: w + 2 * a, h: hh }, draw: dy => {
    ctx.font = FONT_L; ctx.textAlign = 'left'; ctx.fillStyle = bg; ctx.fillRect(x - a, y - up + dy, w + 2 * a, hh);
    if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = 1; ctx.strokeRect(x - a - 0.5, y - up - 0.5 + dy, w + 2 * a + 1, hh + 1); }
    ctx.fillStyle = color; ctx.fillText(t, x, y + dy);
  } });
}
function flushLabels() {
  LQ.sort((a, b) => a.prio - b.prio);
  const placed = [], OFF = [0, 16, -16, 32, -32, 48, -48, 64].map(o => o * TS);
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
  TK?.draw(ctx, layers);
  CM?.draw(ctx, layers);
  SP?.draw(ctx, layers);

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
    const dg = DIG_BY.get(c.id), t = dg?.triage, rk = dg?.rank ? `#${dg.rank} ` : '';
    if (t) drawTriageRing(ctx, x, y, t);   // triage = ring style around the red ping (dig order), never a new hue
    const tri = t && c.vitals ? ` · ${t.label}${t.key === 'reject' && t.sub ? ': ' + t.sub : ''}` : '';
    const txt = small ? `${rk}${c.id}${tri}` : weak ? `${rk}${c.id}${tri} · ${supText(c)}` : `${rk}${c.id}${tri || ' · belief share ' + f2(c.p_mass)}`;
    if (LEAN() && (!dg?.rank)) continue;
    qLabel(LEAN() ? `#${dg.rank}` : txt, x + 12, y - 10, weak ? '#ffd9a6' : '#ffd0d0', weak ? 'rgba(58,35,6,0.92)' : 'rgba(60,8,8,0.88)', 1, weak ? 'rgba(255,159,67,0.7)' : null);
  }
  // drones; drones close together in the same state share one label ("d1–d3 · docked")
  const groups = [];
  for (const d of S.drones || []) {
    const [x, y] = toPx(d.pose[0], d.pose[1]), yaw = (d.pose[3] ?? 0) * Math.PI / 180;
    ctx.save(); ctx.translate(x, y); ctx.rotate(-yaw); ctx.beginPath(); ctx.moveTo(10, 0); ctx.lineTo(-6, 6); ctx.lineTo(-3, 0); ctx.lineTo(-6, -6); ctx.closePath();
    if (d.state === 'listening') {   // R2-022: white outline + a small "ear" ring, not the human-audio yellow
      ctx.fillStyle = '#0b1016'; ctx.fill(); ctx.strokeStyle = COL.fg; ctx.lineWidth = 1.6; ctx.stroke(); ctx.restore();
      ctx.save(); ctx.strokeStyle = 'rgba(227,234,239,0.8)'; ctx.lineWidth = 1.2; ctx.beginPath(); ctx.arc(x, y, 13, -0.35 * Math.PI, 0.35 * Math.PI); ctx.stroke();
      ctx.beginPath(); ctx.arc(x, y, 13, 0.65 * Math.PI, 1.35 * Math.PI); ctx.stroke(); ctx.restore();
    } else { ctx.fillStyle = COL.fg; ctx.fill(); ctx.strokeStyle = '#0b1016'; ctx.lineWidth = 1.5; ctx.stroke(); ctx.restore(); }
    const lk = d.link && d.link !== 'ok' ? `link ${d.link}${d.queued_kb ? ' · ' + Math.round(d.queued_kb) + ' kB queued' : ''}` : '';
    const g = !lk && groups.find(g => g.state === d.state && !g.lk && Math.hypot(g.lx - x, g.ly - y) < 26);
    if (g) { g.ids.push(d.id); g.x = Math.min(g.x, x); g.lx = x; g.ly = y; } else groups.push({ state: d.state, ids: [d.id], x, y, lx: x, ly: y, lk });
  }
  for (const g of groups) {
    const n = g.ids.map(i => +i.replace(/\D/g, '')), seq = g.ids.length > 2 && n.every((v, k) => k === 0 || v === n[k - 1] + 1);
    const ids = seq ? `${g.ids[0]}–${g.ids[g.ids.length - 1]}` : g.ids.join(' ');
    if (!LEAN()) qLabel(`${ids} · ${g.state}${g.lk ? ' · ' + g.lk : ''}`, g.x + 10, g.y + 18, g.lk ? '#a9c8ff' : COL.fg, undefined, 6, g.lk ? 'rgba(59,130,246,0.6)' : null);
  }
  flushLabels();
  scaleBar();
}

function drawGridLayer(now) {
  // staging area
  const stg = S.staging;
  if (stg) {
    const [x0, y0] = toPx(stg.xy[0] - 7, stg.xy[1] + 4), [x1, y1] = toPx(stg.xy[0] + 7, stg.xy[1] - 4);
    ctx.save(); ctx.fillStyle = 'rgba(200,210,220,0.06)'; ctx.strokeStyle = 'rgba(200,210,220,0.55)'; ctx.setLineDash([4, 3]); ctx.lineWidth = 1.2;
    ctx.fillRect(x0, y0, x1 - x0, y1 - y0); ctx.strokeRect(x0, y0, x1 - x0, y1 - y0); ctx.setLineDash([]);
    for (const p of stg.pads || []) { const [px, py] = toPx(...p); ctx.strokeStyle = 'rgba(200,210,220,0.85)'; ctx.strokeRect(px - 4, py - 4, 8, 8); }
    ctx.restore(); if (!LEAN()) qLabel(W < 600 ? 'Staging' : 'Staging · operator console', x0 + 4, y1 + 16, '#c3d0d9', undefined, 7);
  }
  if (!layers.piles) return;
  const m = S.mission || {}, active = m.active_pile, short = view.scale < 6.5;
  const vis = new Set(m.visited || []), pmin = m.priority_min ?? 20, many = (S.piles || []).length > 12, pings = new Set((S.radar_candidates || []).map(c => c.pile_id));
  // dispatch arrows: history faint, latest bright
  const ds = (S.dispatch || []).filter(d => d.to_pile && pileById(d.from_pile) && pileById(d.to_pile));
  ds.forEach((d, k) => {
    const latest = k === ds.length - 1, to = pileById(d.to_pile);
    arrow(pileById(d.from_pile).xy, to.xy, COL.coord, latest ? 0.95 : 0.35, latest && !LEAN() ? `dispatch → ${d.to_pile}${to.priority != null && !short ? ' · priority ' + to.priority : ''}` : null);
  });
  if (layers.reach) for (const p of S.piles || []) drawReach(p, short);
  for (const p of S.piles || []) {
    const [x, y] = toPx(...p.xy), [lng0, sht, col, mapLng] = pst(p), lng = mapLng || lng0, txt = (short ? sht : lng).toUpperCase(), isActive = p.id === active;
    const poly = !!polyOf(p), bx = shapeBox(p), [, topY] = toPx(p.xy[0], bx[3]), [lx0] = toPx(bx[0], 0), [lx1] = toPx(bx[2], 0);
    if (lx1 < -40 || lx0 > W + 40 || topY > Hh + 40) continue;
    ctx.save();
    if (poly) shapePath(ctx, p); else { ctx.beginPath(); ctx.arc(x, y, 8.5 * view.scale, 0, Math.PI * 2); }
    ctx.strokeStyle = col; ctx.globalAlpha = isActive ? 0.95 : 0.45; ctx.lineWidth = isActive ? 2.4 : poly ? 1 : 1.2;
    ctx.setLineDash(p.status === 'no_signal_not_cleared' || p.status === 'unvisited' ? [4, 5] : []); ctx.stroke(); ctx.setLineDash([]);
    if (isActive) { const k = (now / 1600) % 1; ctx.globalAlpha = 0.5 * (1 - k); poly ? shapePath(ctx, p, 0, 0, 3 * k * view.scale) : (ctx.beginPath(), ctx.arc(x, y, (8.5 + 3 * k) * view.scale, 0, Math.PI * 2)); ctx.stroke(); }
    ctx.restore();
    // Declutter (many sectors): name blocks only for notable sectors, or any sector once it is big on screen.
    const notable = isActive || vis.has(p.id) || ['ping', 'lead', 'radar_ping', 'scanning', 'surveying'].includes(p.status) || (p.priority ?? 0) >= pmin || pings.has(p.id);
    if (!notable && many && (lx1 - lx0) < 90) continue;
    // name, priority badge, status chip (above the pile), placed as one fixed block
    const ty = (poly ? topY : y - 8.5 * view.scale) - 8 * TS;
    ctx.font = '600 15px "IBM Plex Sans Condensed", sans-serif'; const nw = ctx.measureText(p.id).width;
    ctx.font = '600 12px "IBM Plex Mono", monospace'; const pt = p.priority != null ? String(p.priority) : null, bw = pt ? ctx.measureText(pt).width + 10 : 0;
    ctx.font = '500 10.5px "IBM Plex Mono", monospace'; const cw = ctx.measureText(txt).width;
    const left = Math.min(x - nw / 2 - 5, x - cw / 2 - 5), right = Math.max(x + nw / 2 + 8 + bw, x + cw / 2 + 5);
    LQ.push({ prio: 0, fixed: true, pile: true, r: { x: x + (left - x) * TS, y: ty - 15 * TS, w: (right - left) * TS, h: 38 * TS }, draw: () => {
      ctx.save(); if (TS !== 1) { ctx.translate(x, ty); ctx.scale(TS, TS); ctx.translate(-x, -ty); }   // whole block scaled about its anchor
      ctx.textAlign = 'left';
      ctx.font = '600 15px "IBM Plex Sans Condensed", sans-serif';
      ctx.fillStyle = 'rgba(11,16,22,0.85)'; ctx.fillRect(x - nw / 2 - 5, ty - 15, nw + 10, 20); ctx.fillStyle = COL.fg; ctx.fillText(p.id, x - nw / 2, ty);
      if (pt) {
        ctx.font = '600 12px "IBM Plex Mono", monospace'; const bx = x + nw / 2 + 8;
        ctx.fillStyle = p.status === 'lead' || p.priority >= (m.priority_min ?? 20) ? COL.teal : 'rgba(34,195,214,0.35)';
        roundRect(bx, ty - 14, bw, 18, 9); ctx.fill(); ctx.fillStyle = '#04161a'; ctx.fillText(pt, bx + 5, ty);
      }
      if (LEAN()) { ctx.restore(); return; }   // narrow take: the ring colour carries the status
      ctx.font = '500 10.5px "IBM Plex Mono", monospace';
      ctx.fillStyle = 'rgba(11,16,22,0.88)'; ctx.fillRect(x - cw / 2 - 5, ty + 7, cw + 10, 15); ctx.strokeStyle = col; ctx.lineWidth = 1; ctx.setLineDash(pileAmb(p) ? [3, 2] : []); ctx.strokeRect(x - cw / 2 - 5, ty + 7, cw + 10, 15); ctx.setLineDash([]);
      ctx.fillStyle = col; ctx.fillText(txt, x - cw / 2, ty + 18);
      ctx.restore();
    } });
  }
}
// Radar reach (piles[].radar_coverage): hatch the part of a searched pile the radar could not reach from the sites
// it scanned from. The in-reach area is drawn as discs around those sensor positions, sized so the hatched share of
// the pile matches 1 - radar_coverage; without radar_coverage the disc radius is the nominal reach (~3 m).
// Sectors: a pile is a circle (3x3 grid, r 9 m) or, on mapped sites (site="collapse", mesh sites), a polygon
// (piles[].polygon or piles[].outline, site metres). All map drawing goes through these helpers.
const PILE_R = 9, hatchCv = document.createElement('canvas'), reachCache = new Map();
const polyOf = p => (Array.isArray(p.polygon) && p.polygon.length >= 3 ? p.polygon : Array.isArray(p.outline) && p.outline.length >= 3 ? p.outline : null);
function shapeBox(p) {
  const P = polyOf(p); if (!P) return [p.xy[0] - PILE_R, p.xy[1] - PILE_R, p.xy[0] + PILE_R, p.xy[1] + PILE_R];
  let b = [Infinity, Infinity, -Infinity, -Infinity]; for (const [x, y] of P) b = [Math.min(b[0], x), Math.min(b[1], y), Math.max(b[2], x), Math.max(b[3], y)]; return b;
}
function shapePath(c2, p, ox = 0, oy = 0, grow = 0) {   // path in canvas px (minus an offset), optionally grown by `grow` px
  const P = polyOf(p); c2.beginPath();
  if (!P) { const [x, y] = toPx(...p.xy); c2.arc(x - ox, y - oy, PILE_R * view.scale + grow, 0, Math.PI * 2); return; }
  const [cx, cy] = toPx(...p.xy);
  P.forEach((q, k) => { let [x, y] = toPx(...q); if (grow) { const d = Math.hypot(x - cx, y - cy) || 1; x += (x - cx) / d * grow; y += (y - cy) / d * grow; } k ? c2.lineTo(x - ox, y - oy) : c2.moveTo(x - ox, y - oy); });
  c2.closePath();
}
const inPolyXY = (x, y, P) => { let c = false; for (let i = 0, j = P.length - 1; i < P.length; j = i++) { const [xi, yi] = P[i], [xj, yj] = P[j]; if ((yi > y) !== (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) c = !c; } return c; };
function shapeSamples(p) {
  const P = polyOf(p), b = shapeBox(p), st = Math.max(0.5, Math.max(b[2] - b[0], b[3] - b[1]) / 24), o = [];
  for (let x = b[0] + st / 2; x < b[2]; x += st) for (let y = b[1] + st / 2; y < b[3]; y += st)
    if (P ? inPolyXY(x, y, P) : (x - p.xy[0]) ** 2 + (y - p.xy[1]) ** 2 <= PILE_R * PILE_R) o.push([x, y]);
  return o;
}
function reachGeom(p) {
  const m = S.mission || {}, vis = m.visited || [];
  if (!vis.includes(p.id)) return null;
  const cov = p.radar_coverage;
  if (cov == null && p.id === m.active_pile && m.stage === 'pile') return null;          // still scanning
  let pts = (S.scans || []).filter(o => o.pile_id === p.id && o.sensor_pose && !o.doppler_moving).map(o => [o.sensor_pose[0], o.sensor_pose[1]]);
  if (!pts.length) pts = (S.landing_sites || []).filter(l => l.pile_id === p.id && l.status === 'used').map(l => l.xy);
  if (!pts.length || (cov != null && cov >= 0.98)) return null;
  const key = `${p.id}|${cov}|${pts.map(q => q.join(',')).join(';')}|${polyOf(p)?.length || 0}`;
  if (reachCache.has(key)) return reachCache.get(key);
  const smp = shapeSamples(p), b = shapeBox(p), span = Math.hypot(b[2] - b[0], b[3] - b[1]);
  const covered = R => smp.filter(([i, j]) => pts.some(([a, c]) => (i - a) ** 2 + (j - c) ** 2 <= R * R)).length / Math.max(1, smp.length);
  let R;
  if (cov != null) { let lo = 0, hi = span + 12; for (let k = 0; k < 22; k++) { const mid = (lo + hi) / 2; covered(mid) < cov ? lo = mid : hi = mid; } R = (lo + hi) / 2; }
  else R = Math.min(m.radar_reach_beneath_m ?? 3, 3.5);
  const g = covered(R) > 0.98 ? null : { R, pts, beyond: cov != null ? 1 - cov : 1 - covered(R) };
  reachCache.set(key, g); if (reachCache.size > 400) reachCache.clear();
  return g;
}
function drawReach(p, short) {
  const g = reachGeom(p); if (!g) return;
  const s = view.scale, bx = shapeBox(p), [x0, y0] = toPx(bx[0], bx[3]), [x1, y1] = toPx(bx[2], bx[1]);
  const ox = Math.floor(x0) - 2, oy = Math.floor(y0) - 2, sw = Math.ceil(x1 - ox) + 2, sh = Math.ceil(y1 - oy) + 2, size = Math.max(sw, sh);
  if (size < 8 || sw * sh > 9e6 || x1 < 0 || y1 < 0 || x0 > W || y0 > Hh) return;
  hatchCv.width = sw * dpr; hatchCv.height = sh * dpr;
  const h = hatchCv.getContext('2d'); h.setTransform(dpr, 0, 0, dpr, 0, 0); h.clearRect(0, 0, sw, sh);
  h.save(); shapePath(h, p, ox, oy); h.clip();
  h.fillStyle = 'rgba(11,16,22,0.28)'; h.fillRect(0, 0, size, size);
  h.strokeStyle = 'rgba(205,218,228,0.42)'; h.lineWidth = 1; h.beginPath();
  for (let k = -size; k < size; k += 7) { h.moveTo(k, size); h.lineTo(k + size, 0); }
  h.stroke(); h.restore();
  h.globalCompositeOperation = 'destination-out'; h.fillStyle = '#000';
  for (const q of g.pts) { const [px, py] = toPx(...q); h.beginPath(); h.arc(px - ox, py - oy, g.R * s, 0, Math.PI * 2); h.fill(); }
  h.globalCompositeOperation = 'source-over';
  ctx.drawImage(hatchCv, ox, oy, sw, sh);
  const [cx] = toPx(...p.xy);
  if ((S.zones || []).length) return;   // tasked zones carry the reach status in their own label
  const t = short ? 'beyond radar reach' : `beyond radar reach — acoustic leads only${p.radar_coverage != null ? ` (${f0(100 * g.beyond)}%)` : ''}`;
  ctx.font = FONT_L; const w = ctx.measureText(t).width;
  if (!LEAN()) qLabel(t, cx - w / 2, y1 + 15, '#c3d0d9', 'rgba(11,16,22,0.84)', 8, 'rgba(205,218,228,0.35)');
}
function roundRect(x, y, w, h, r) { ctx.beginPath(); ctx.moveTo(x + r, y); ctx.arcTo(x + w, y, x + w, y + h, r); ctx.arcTo(x + w, y + h, x, y + h, r); ctx.arcTo(x, y + h, x, y, r); ctx.arcTo(x, y, x + w, y, r); ctx.closePath(); }
function scaleBar() {
  const nice = [1, 2, 5, 10, 20, 50], m = nice.find(v => v * view.scale > 70) || 50, px = m * view.scale, x = 14, y = Hh - (W < 900 ? (tilesOn() ? 62 : 40) : 44);
  ctx.strokeStyle = COL.fg; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + px, y); ctx.moveTo(x, y - 4); ctx.lineTo(x, y + 4); ctx.moveTo(x + px, y - 4); ctx.lineTo(x + px, y + 4); ctx.stroke();
  ctx.fillStyle = COL.fg; ctx.font = `500 ${11 * TS}px "IBM Plex Mono", monospace`; ctx.textAlign = 'left'; ctx.fillText(`${m} m`, x + px + 6, y + 4);
  const b = (geo().bearing_deg || 0) * RAD, nx = W - 30, ny = 66;   // true north relative to the site frame
  ctx.save(); ctx.translate(nx, ny); ctx.rotate(-b); ctx.beginPath(); ctx.moveTo(0, -11); ctx.lineTo(5, 4); ctx.lineTo(0, 1); ctx.lineTo(-5, 4); ctx.closePath(); ctx.fill();
  ctx.textAlign = 'center'; ctx.fillText('N', 0, -14); ctx.restore(); ctx.textAlign = 'left';
}

// interaction
let drag = null;
let tkGrab = false;   // a tasking gesture (lasso, vertex drag) owns the pointer
const rel = e => { const r = cv.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; };
cv.addEventListener('pointerdown', e => {
  cv.setPointerCapture(e.pointerId);
  if (TK?.ptr.down(e, ...rel(e)) === true) { tkGrab = true; return; }
  drag = { x: e.clientX, y: e.clientY, cx: view.cx, cy: view.cy, moved: false };
});
cv.addEventListener('dblclick', e => { if (TK?.ptr.dbl()) e.preventDefault(); });
cv.addEventListener('pointermove', e => {
  if (TK?.ptr.move(e, ...rel(e)) && tkGrab) return;
  if (tkGrab) return;
  const r = cv.getBoundingClientRect(), [mx, my] = toM(e.clientX - r.left, e.clientY - r.top);
  const [la, lo] = siteToLL(mx, my); $('cursor').textContent = `x ${mx.toFixed(1)} · y ${my.toFixed(1)} m · ${la.toFixed(5)}° N ${lo.toFixed(5)}° E`;
  if (drag) { const dx = e.clientX - drag.x, dy = e.clientY - drag.y; if (Math.hypot(dx, dy) > 3) drag.moved = true; view.cx = drag.cx - dx / view.scale; view.cy = drag.cy + dy / view.scale; cv.classList.add('drag'); draw(); return; }
  const hit = hitTest(e.clientX - r.left, e.clientY - r.top); hoverId = hit?.id ?? null; cv.classList.toggle('hot', !!hit);
});
cv.addEventListener('pointerup', e => {
  cv.classList.remove('drag'); const r = cv.getBoundingClientRect();
  if (tkGrab) { tkGrab = false; TK.ptr.up(e, ...rel(e), false); return; }
  if (drag && TK?.planning && TK.ptr.up(e, ...rel(e), drag.moved)) { drag = null; return; }
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
document.querySelectorAll('[data-bm]').forEach(b => b.onclick = () => { basemap = b.dataset.bm; basemapUser = true; renderBasemapUI(); draw(); });
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
  const rc = S.recon; $('reconChip').hidden = !rc; if (rc) $('reconChip').textContent = reconText(rc);
  renderBasemapUI();
  TK?.renderRail();
  CM?.render(); if (S.comms) CM?.start();
  SP?.render(); if (grid && !TAKE) SP?.start();
  LV?.render();
  renderCands(grid);
  $('secAgent').hidden = !grid; $('secPiles').hidden = !grid || !(S.piles || []).length; $('secAc').hidden = grid;
  if (grid) { renderPiles(); renderDecisions(); renderClips(); }
  else {
    $('secDec').hidden = true;
    const ac = S.acoustic || [];
    $('nAc').textContent = ac.length;
    $('acList').innerHTML = ac.length ? ac.map(a => `<div class="ac"><span>${esc(a.drone_id)} · <span class="c">${esc(a.class)}</span></span><span>${a.bearing_deg.toFixed(0)}° ±${(a.width_deg / 2).toFixed(0)} · ${f2(a.confidence)}</span></div>`).join('') : '<p class="empty">Nothing heard yet.</p>';
  }
  $('fleet').innerHTML = (S.drones || []).map(d => { const b = Math.round((d.battery ?? 0) * 100); return `<div class="fl"><b>${esc(d.id)}</b><span class="st ${esc(d.state)}">${esc(d.state)}${linkChip(d)}</span><span class="bat" title="battery ${b}%"><i style="width:${b}%"></i></span><span class="pc">${b}%</span></div>`; }).join('');
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

// Dig order: radar pings ranked by breathing-pattern triage x localisation certainty x depth (vitals.js).
function computeDig() {
  const cands = S.radar_candidates || [];
  const all = cands.map(c => {
    const surf = S.heightmap ? hmAt(c.xy[0], c.xy[1]) : null, di = depthInfo(c, surf), e = c.cov ? ellipse(c) : null;
    return { c, di, e, ...digScore(c, di, e) };
  });
  DIG = all.filter(d => !isRejected(d.c)).sort((a, b) => b.score - a.score || (b.c.p_mass ?? 0) - (a.c.p_mass ?? 0) || a.c.id.localeCompare(b.c.id));
  REJ = all.filter(d => isRejected(d.c));
  DIG.forEach((d, k) => { d.rank = k + 1; }); REJ.forEach(d => { d.rank = null; });
  DIG_BY = new Map(all.map(d => [d.c.id, d]));
}
function renderCands(grid) {
  computeDig();
  $('nCand').textContent = DIG.length + (REJ.length ? ` + ${REJ.length} rejected` : '');
  $('candsTitle').textContent = 'Dig order';
  $('candsTitle').title = grid ? 'Radar pings, ranked' : 'Radar candidates, ranked';
  if (!DIG.length && !REJ.length) { $('cands').innerHTML = '<p class="empty">No radar pings yet. No detection is not proof of absence.</p>'; return; }
  const anyV = DIG.some(d => d.c.vitals);
  $('cands').innerHTML = DIG.map(d => {
    const { c, di, triage: t, weak } = d, v = c.vitals || null, mv = c.kind === 'moving';
    const lab = mv ? 'moving track' : weak ? 'insufficient evidence' : 'candidate region';
    const pl = conditionLabel(v, c), cond = conditionOf(v), rate = rateOf(v, c), sp = spark(c, { w: 92, h: 24 });
    const ci = v?.rate_ci?.length === 2 && v.rate_ci[0] != null ? ` (${f0(v.rate_ci[0])}–${f0(v.rate_ci[1])})` : '';
    const cav = caveatsOf(c);
    return `<button class="dig ${weak ? 'weak' : ''} t-${t.key}" data-id="${esc(c.id)}" title="${esc(cav.map(k => '• ' + k).join('\n'))}">
      <span class="rk">${d.rank}</span>
      <span class="l1"><i class="mk"></i><b>${esc(c.id)}</b><small>${lab}</small>${triageChip(t)}</span>
      <span class="sp${sp?.rateOnly ? ' ro' : ''}">${sp ? sp.svg + `<small>${esc(sp.label)}</small>` : '<small>no breathing signal</small>'}</span>
      <span class="l2 ${cond ? 'k-' + cond.key : 'none'}"><span class="cl-full">${esc(pl || 'No breathing signal at this ping')}</span><span class="cl-short">${esc(conditionShort(v, c) || pl || 'No breathing signal')}</span></span>
      <span class="l5">${v ? confMeter(v.pattern_conf) : '<span class="cm none">no pattern analysis</span>'}${respChip(c.responsiveness)}</span>
      <span class="l3">depth <b>${esc(di.short)}</b> · ${weak ? `<span class="sup">${esc(supText(c))}</span>` : `belief share <b>${f2(c.p_mass)}</b>`}${v?.rate_ci?.length === 2 && v.rate_ci[0] != null ? ` · rate 95% ${f0(v.rate_ci[0])}–${f0(v.rate_ci[1])}` : ''}</span>
      <span class="l4"><span class="aid">Aid, not a diagnosis</span><span class="cv">${cav.length} caveat${cav.length === 1 ? '' : 's'}</span><span class="go">3D view →</span></span></button>`;
  }).join('') + `<p class="takenote">Candidate regions, not confirmed people · aid for rescuers, not a diagnosis</p>` + (REJ.length ? `<details class="rej"${rejOpen ? ' open' : ''}><summary>Not a person, says the radar · ${REJ.length} (not ranked)</summary>${REJ.map(d => {
      const { c, triage: t } = d, src = sourceOf(c.vitals);
      return `<button class="dig rejd" data-id="${esc(c.id)}" title="${esc(c.vitals?.source_reason || '')}"><span class="rk">–</span>
        <span class="l1"><i class="mk"></i><b>${esc(c.id)}</b>${triageChip(t)}</span>
        <span class="l2 none">${esc(conditionLabel(c.vitals, c) || '')}</span>
        <span class="l3">source ${esc(src?.label || '')}${c.vitals?.source_conf != null ? ' · confidence ' + f2(c.vitals.source_conf) : ''} · still a radar return: a person cannot be fully ruled out</span>
        <span class="l4"><span class="go">3D view →</span></span></button>`;
    }).join('')}</details>` : '') + `<p class="dignote">Ranked by ${anyV ? 'the condition read from the breathing pattern (weighted by its confidence)' : 'condition (none analysed yet: all “assess”)'} × localisation certainty × depth. ${esc(ALWAYS)} Every ping is a candidate search region, not a confirmed person.</p>`;
}
$('cands').addEventListener('toggle', e => { if (e.target.matches?.('details.rej')) rejOpen = e.target.open; }, true);
$('cands').addEventListener('click', e => { const b = e.target.closest('[data-id]'); if (b) openDetail((S.radar_candidates || []).find(c => c.id === b.dataset.id)); });

function renderPiles() {
  const m = S.mission || {}, vis = [...new Set(m.visited || [])], done = m.stage === 'done', pmin = m.priority_min ?? 20;
  const visited = vis.map(pileById).filter(Boolean);
  const piles = S.piles || [], rest = piles.filter(p => !vis.includes(p.id)).sort((a, b) => (b.priority ?? -1) - (a.priority ?? -1) || a.id.localeCompare(b.id));
  const CAP = 12, restShown = showAllPiles || rest.length <= CAP + 2 ? rest : rest.filter((p, k) => k < CAP || openRows.has(p.id)), restHidden = rest.length > CAP + 2 ? rest.length - (showAllPiles ? 0 : restShown.length) : 0;
  const topUnvisited = done ? null : rest.find(p => p.priority != null && p.priority >= pmin);
  $('pilesTitle').textContent = done ? 'Open leads for ground teams' : 'Pile priority';
  $('nPiles').textContent = `${vis.length}/${piles.length}`; $('nPiles').title = `${vis.length} of ${piles.length} piles searched by the drones`;   // R2-018
  const row = p => {
    const [txt, , col] = pst(p), amb = pileAmb(p);
    return `<div class="row ${p === topUnvisited ? 'top' : ''} ${vis.includes(p.id) ? 'done' : ''} ${openRows.has(p.id) ? 'open' : ''}" data-pile="${esc(p.id)}" title="Click to expand">
      <b>${esc(p.id)}</b><span class="why"><span class="chip" style="color:${col};border-color:${col}${amb ? ';border-style:dashed' : ''}">${esc(txt)}</span>${amb ? '<span class="chip amb" title="The drones\' bearings do not single out this pile: the sound may come from a neighbour">ambiguous direction</span>' : ''}${respChip(p.responsiveness || S.responsiveness?.[p.id], true)}${evidenceChip(p)} ${esc(p.why)}</span>
      <span class="s">${p.priority ?? '—'}</span></div>`;
  };
  $('piles').innerHTML = (visited.length ? `<div class="sub">Searched by the drones</div>${visited.map(row).join('')}` : '')
    + `<div class="sub">${done ? 'Not searched: ranked by acoustic priority' : 'Not yet searched: ranked by acoustic priority'}</div>${restShown.map(row).join('')}`
    + (restHidden ? `<button class="more" id="pilesMore">${showAllPiles ? 'Show fewer' : `+ ${restHidden} more sectors (nothing ranked above them; silent ≠ cleared)`}</button>` : '');
  const d = (S.dispatch || []).slice(-1)[0];
  $('dispatch').classList.toggle('summary', !!(done && m.summary)); $('dispatch').title = done && m.summary ? 'Click to expand' : '';
  $('dispatch').textContent = done && m.summary ? m.summary.replace(/^MISSION SUMMARY:\s*/, 'Mission summary: ')
    : !(S.decisions || []).length && d ? (d.to_pile ? `Dispatch ${d.from_pile} → ${d.to_pile}: ${d.reason}` : `${d.from_pile}: search loop ended (${d.reason})`) : '';
}
$('dispatch').addEventListener('click', () => $('dispatch').classList.toggle('open'));
$('piles').addEventListener('click', e => { if (e.target.id === 'pilesMore') { showAllPiles = !showAllPiles; renderPiles(); return; } const r = e.target.closest('.row'); if (!r) return; const id = r.dataset.pile; openRows.has(id) ? openRows.delete(id) : openRows.add(id); r.classList.toggle('open'); });

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
  const keep = list.filter(c => !isDismissed(c)), dism = list.filter(isDismissed);
  $('clips').innerHTML = !list.length ? '<p class="empty">Listening starts once the drones have landed and the radar phase at the first pile is over.</p>'
    : keep.map(c => {
      const h = c.scores?.human ?? 0, nonhuman = h < 0.4, by = audioBy(c);
      return `<div class="clip ${nonhuman ? 'nonhuman' : ''} ${isAmb(c) ? 'amb' : ''}">
      <div class="hd"><span><b>${esc(c.pile_id)}</b> · listen ${c.listen ?? '—'}${c.t != null ? ' · t ' + f0(c.t) + ' s' : ''}</span><span><span class="cls">${esc(c.class)}</span> <span class="by ${by.cls}" title="${esc(by.tip)}">${esc(by.txt)}</span></span></div>
      ${clipSpeech(c)}
      <div class="prov">${provenance(c)}${c.novelty != null ? ` · novelty ${f1(c.novelty)}` : ''}${c.audio_analysis && c.by ? ` · judge ${esc(byInfo(c.by).txt)}` : ''}</div>${ambChip(c)}
      <div class="scores"><span>human${bar(h)}</span><span>urgency${bar(c.scores?.urgency)}</span><span>reliability${bar(c.scores?.reliability)}</span><span class="pr" title="acoustic priority 0-100">${c.priority ?? '—'}<small>priority</small></span></div>
      ${c.rationale ? `<div class="why ${openWhy.has(c.id) ? 'open' : ''}" data-clip="${esc(c.id)}" title="Click to expand">${esc(c.rationale)}</div>` : ''}</div>`;
    }).join('') + (dism.length ? `<details class="dism"${dismOpen ? ' open' : ''}><summary>Dismissed (non-human) · ${dism.length}</summary>
      ${dism.map(c => { const by = audioBy(c), why = (Array.isArray(c.dismissed) && c.dismissed.length ? c.dismissed.join(', ') : `${c.class} (human ${f2(c.scores?.human)})`);
        return `<div class="drow" title="${esc(c.rationale || '')}"><b>${esc(c.pile_id)}</b><span class="cls">${esc(c.class)}</span><span class="dw">${esc(why)}${c.novelty != null ? ` · novelty ${f1(c.novelty)}` : ''}</span><span class="by ${by.cls}" title="${esc(by.tip)}">${esc(by.txt)}</span></div>`; }).join('')}
      <p class="dnote">Dismissed as acoustic leads only: these piles are <b>not cleared</b>.</p></details>` : '');
}
// Clips dismissed before the analyser (stage 1 not forwarded) or judged non-human: collapsed, never "cleared".
const isDismissed = c => c.forwarded === false || (c.scores?.human ?? 0) < 0.4;
// Provenance of the audio analysis (language / transcript / translation / distress): Gemini or the local fallback.
function audioBy(c) {
  const b = String(c.audio_analysis?.by || '').toLowerCase();
  if (b === 'gemini') return { cls: 'llm', txt: 'Gemini', tip: `Audio analysed by Gemini${c.audio_analysis.model ? ' (' + c.audio_analysis.model + ')' : ''}: language, transcript, translation, distress` };
  if (b) return { cls: 'fb', txt: 'fallback', tip: 'Local fallback: Whisper + multilingual distress keywords (no Gemini call)' };
  return byInfo(c.by);
}
const LANG = { zh: 'Mandarin', 'zh-tw': 'Mandarin (Taiwan)', 'zh-hant': 'Mandarin (Traditional)', cmn: 'Mandarin', nan: 'Taiwanese Hokkien', 'nan-tw': 'Taiwanese Hokkien', hak: 'Hakka',
  en: 'English', id: 'Indonesian', vi: 'Vietnamese', tl: 'Tagalog', fil: 'Filipino', th: 'Thai', ja: 'Japanese', ko: 'Korean', es: 'Spanish' };
function langOf(c) {
  const l0 = Array.isArray(c.languages) && c.languages[0], code = (l0?.code || c.language || '').trim();
  if (!code) return null;
  const nm = l0?.name && l0.name !== code ? l0.name : LANG[code.toLowerCase()] || LANG[code.toLowerCase().split('-')[0]] || code;
  const more = Array.isArray(c.languages) && c.languages.length > 1 ? ` +${c.languages.length - 1}` : '';
  return { code, name: nm, txt: `${nm} · ${code}${more}`, en: /^en(\b|-)/i.test(code) };
}
const DISTRESS = { call_for_help: 'call for help', trapped: 'trapped', injured: 'injured', cant_breathe: "can't breathe", child: 'child', multiple_people: 'several people', bleeding: 'bleeding', pain: 'pain' };
// Language chip, original transcript, English translation and distress tags (all optional).
function clipSpeech(c) {
  const lg = langOf(c), orig = c.transcript_original || c.transcript || '', tr = (c.translation || '').trim();
  const showTr = tr && tr !== orig.trim() && !(lg?.en);
  const dist = Array.isArray(c.distress) ? c.distress : [];
  return (lg || dist.length ? `<div class="lang">${lg ? `<span class="lg" title="Language detected by ${esc(audioBy(c).txt)}">${esc(lg.txt)}</span>` : ''}${dist.map(d => `<span class="ds" title="Distress indicator reported by the audio analysis (${esc(audioBy(c).txt)}); not verified">${esc(DISTRESS[d] || String(d).replace(/_/g, ' '))}</span>`).join('')}</div>` : '')
    + (orig ? `<q${lg ? ` lang="${esc(lg.code)}"` : ''}>${esc(orig)}</q>` : '')
    + (showTr ? `<div class="tr"><span>EN</span>${esc(tr)}</div>` : '');
}
$('listens').addEventListener('click', e => { const b = e.target.closest('[data-l]'); if (!b) return; const l = +b.dataset.l, L = listensOf(); selListen = l === L[L.length - 1] ? null : l; renderClips(); draw(); });
$('clips').addEventListener('toggle', e => { if (e.target.matches?.('details.dism')) dismOpen = e.target.open; }, true);
$('clips').addEventListener('click', e => { const w = e.target.closest('.why'); if (!w) return; const id = w.dataset.clip; openWhy.has(id) ? openWhy.delete(id) : openWhy.add(id); w.classList.toggle('open'); });

function hmAt(x, y) {
  const g = S.heightmap; const i = (x - g.origin[0]) / g.res - 0.5, j = (y - g.origin[1]) / g.res - 0.5;
  const i0 = Math.max(0, Math.min(g.w - 2, Math.floor(i))), j0 = Math.max(0, Math.min(g.h - 2, Math.floor(j))), u = i - i0, v = j - j0, z = (a, b) => g.z[b * g.w + a];
  return (z(i0, j0) * (1 - u) + z(i0 + 1, j0) * u) * (1 - v) + (z(i0, j0 + 1) * (1 - u) + z(i0 + 1, j0 + 1) * u) * v;
}

// ---------------- detail: LiDAR view with depth ping ----------------
// Depth and breathing side by side, then the vitals numbers, the triage reason and every caveat.
function vitalsCard(c, di) {
  if (!DIG_BY.has(c.id)) computeDig();
  const dg = DIG_BY.get(c.id), v = c.vitals || null, t = dg?.triage || triageOf(v), rate = rateOf(v, c), sp = spark(c, { w: 300, h: 46, win: 45 });
  const pl = conditionLabel(v, c), cond = conditionOf(v), cav = caveatsOf(c);
  const kv = [];
  if (rate) kv.push(['Rate', `${f0(rate)}/min${v?.rate_ci?.[0] != null ? ` <small>95% ${f0(v.rate_ci[0])}–${f0(v.rate_ci[1])}</small>` : v ? '' : ' <small>periodic radar return</small>'}`]);
  if (v?.cv != null) kv.push(['Variability', `${f2(v.cv)} <small>breath-to-breath CV</small>`]);
  if (v?.apnea_events_per_min != null) kv.push(['Pauses > 10 s', `${f1(v.apnea_events_per_min)}/min`]);
  if (v?.movement != null) kv.push(['Movement', `${f0(100 * v.movement)}% <small>of the window</small>`]);
  if (v?.depth_rel != null) kv.push(['Chest excursion', `${f1(v.depth_rel)}× a calm resting breath <small>LOW confidence${v.depth_rel_conf != null ? ' (' + f2(v.depth_rel_conf) + ')' : ''}</small>`]);
  if (v?.sqi != null) kv.push(['Signal quality', f2(v.sqi)]);
  if (dg) kv.push(['Dig order', dg.rank ? `#${dg.rank} of ${DIG.length} <small>triage ${f2(dg.tri)} × localisation ${f2(dg.loc)} × depth ${f2(dg.acc)}</small>` : 'not ranked <small>the radar rejector says this is not a person</small>']);
  const src = sourceOf(v);
  if (src) kv.unshift(['Source', `${esc(src.label)}${v.source_conf != null ? ` <small>confidence ${f2(v.source_conf)}</small>` : ''}${v.source_reason ? `<small class="sr">${esc(v.source_reason)}</small>` : ''}`]);
  const sig = signatureAsset(v);
  return `<div class="vt">
      <div class="tile dtile"><h5>Depth</h5><b>${esc(di.short.replace(/\s*\((nominal|radar estimate)\)$/, ''))}</b><small>${esc(di.nominal ? 'nominal: assumed, not measured' : 'radar estimate, below the LiDAR surface')}</small></div>
      <div class="tile btile"><h5>Condition (breathing) ${triageChip(t)}</h5><b class="pat ${cond ? 'k-' + cond.key : ''}">${esc(pl || 'No breathing signal at this ping')}</b>
        ${v ? confMeter(v.pattern_conf) : '<span class="cm none">not analysed in this run: rate only</span>'}
        ${c.responsiveness ? `<div class="rsp">${respChip(c.responsiveness)}</div>` : ''}
        ${c.triage_fused?.changed_from ? `<small class="fz">Raised from “${esc(c.triage_fused.changed_from)}” after the call-out: ${esc(c.triage_fused.reason || '')}</small>` : ''}
        <small class="aidn">Aid for rescuers, not a diagnosis.</small>
        ${sp ? `<div class="bsp">${sp.svg}<small title="${esc(sp.tip)}">${esc(sp.label)}${sp.measured ? '' : sp.rateOnly ? ': periodic radar return, no pattern analysis; the shape is not a pattern' : ': shape from rate, variability and pauses, not the measured waveform'}</small></div>` : '<small>no periodic signal</small>'}</div>
    </div>
    ${kv.length ? `<dl class="vkv">${kv.map(([k, x]) => `<dt>${k}</dt><dd>${x}</dd>`).join('')}</dl>` : ''}
    ${sig ? `<figure class="sigth"><img src="/assets/radar_signatures/${esc(sig)}.png" alt="Reference radar signature: ${esc(sig.replace(/_/g, ' '))}" loading="lazy" onerror="this.closest('figure').hidden=true"><figcaption>Library signature for “${esc(sig.replace(/_/g, ' '))}” (synthetic reference, not this ping's data)</figcaption></figure>` : ''}
    ${v?.triage_reason || v?.pattern_reason ? `<p class="vwhy">${esc(v.triage_reason || '')}${v.pattern_reason ? `<small>${esc(v.pattern_reason)}</small>` : ''}</p>` : ''}
    <details class="vcav"${v ? '' : ' open'}><summary>Caveats (${cav.length})</summary><ul>${cav.map(k => `<li>${esc(k)}</li>`).join('')}</ul></details>`;
}

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
    ...(pile ? [['Pile', `${esc(pile.id)} · ${esc(pst(pile)[0])}${pile.priority != null ? ' · acoustic priority ' + pile.priority : ''}`]] : []),
    ['Position (E, N)', `${f1(c.xy[0])}, ${f1(c.xy[1])} m`],
    ['Surface (LiDAR)', `${f1(surf)} m`],
    ['95% region', e ? `${f1(2 * e.rx)} × ${f1(2 * e.ry)} m` : '—'],
    weak ? ['Radar support', `<span style="color:var(--radar)">${esc(supText(c))}</span><small>too few independent ranges agree for a fix; belief share not shown</small>`]
      : ['Belief share', `${f2(c.p_mass)}<small>share of this pile's radar belief at this spot; not the probability that a person is there</small>`],
    ['Periodic signal', c.breathing_hz ? `${f2(c.breathing_hz)} Hz (${(c.breathing_hz * 60).toFixed(0)}/min), respiration-like` : '—'],
    ['Motion', moving ? 'Doppler shows bulk motion' : 'stationary'],
  ];
  $('dFacts').innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
  $('dVitals').innerHTML = vitalsCard(c, di);
  // What was heard at this pile, with provenance (R1-029)
  const heard = (S.acoustic_clips || []).filter(a => c.pile_id && a.pile_id === c.pile_id && !isDismissed(a)).sort((a, b) => (a.listen ?? 0) - (b.listen ?? 0));
  $('dHeard').innerHTML = !isGrid() ? '' : `<h4>Heard at this pile</h4>` + (heard.length ? heard.map(a => {
    const p = pileById(a.pile_id), dist = p && a.origin ? Math.hypot(p.xy[0] - a.origin[0], p.xy[1] - a.origin[1]) : null, by = audioBy(a);
    return `<div class="heard">${clipSpeech(a)}
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
    const tv = c.vitals ? ' · ' + (DIG_BY.get(c.id)?.triage || triageOf(c.vitals)).label + ' (aid)' : '';
    $('vinfo').textContent = '';
    viewer.show({ cloud, tiles: S.pointcloud?.tiles, base: DATA, center: [c.xy[0], c.xy[1]], surf, z, ellipse: e, id: c.id, depthLabel: di.long + tv, weak: weak || moving, onInfo: t => { $('vinfo').textContent = t; } });
  } catch (err) {
    $('v3d').innerHTML = '<div class="v3dmsg">3D view unavailable: three.js could not load (offline?). The facts on the right still apply.</div>';
    $('vhint').hidden = true;
  }
}
function closeDetail() { $('detail').hidden = true; viewer?.stop(); }
$('dClose').onclick = closeDetail;
$('detail').addEventListener('click', e => { if (e.target.id === 'detail') closeDetail(); });
addEventListener('keydown', e => { if (TK?.ptr.key(e)) { e.preventDefault(); return; } if (e.key === 'Escape') { if (!$('detail').hidden) closeDetail(); else { setHelp(false); setCase(false); } } });

// ---------------- case study ----------------
function setCase(open) {
  $('casePop').hidden = !open; $('caseBtn').setAttribute('aria-expanded', open);
  if (open) { const g = geo(); $('caseGeo').textContent = `Map anchor: ${g.name}, ${(+g.lat).toFixed(5)}° N, ${(+g.lon).toFixed(5)}° E (source: zh.wikipedia 維冠金龍大樓). Site frame: metres, x east, y north, origin at the building, bearing ${f0(g.bearing_deg || 0)}°${S?.site?.geo ? ' (from state.json)' : ' (default)'}.`; }
}
$('caseBtn').onclick = () => setCase($('casePop').hidden);
$('caseClose').onclick = () => setCase(false);

// ---------------- operator tasking ----------------
LV = createLive({ S: () => S, DATA, $, take: TAKE });
if (LIVE_FILES) LV.start();
SP = createSpeaker({ S: () => S, toPx, scale: () => view.scale, ctx: () => ctx, draw: () => draw(), qLabel, font: FONT_L, $ });
CM = createComms({ S: () => S, toPx, scale: () => view.scale, ctx: () => ctx, draw: () => draw(), qLabel, font: FONT_L, $ });
TK = createTasking({ S: () => S, toPx, toM, scale: () => view.scale, ctx: () => ctx, draw: () => draw(), qLabel, font: FONT_L, $ });

// ---------------- boot ----------------
new ResizeObserver(() => (TAKE && S ? fit() : resize())).observe(cv);   // take: nobody pans, so keep the site fitted as panels load
renderBasemapUI();
resize();
poll();
(function tick(now) { if ($('detail').hidden) draw(now); requestAnimationFrame(tick); })(performance.now());   // map paused under the 3D view
