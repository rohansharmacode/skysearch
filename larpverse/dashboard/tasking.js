// Operator tasking (contracts/INTERFACES.md "Operator tasking (v1.2)") and recon display ("Reconnaissance mode (v1.2)").
// Plan search: draw zones (click vertices, double-click / Enter / tap the first vertex to close, or a freehand lasso),
// edit / rename / prioritise / delete, prefill from the rubble outlines, then POST /api/tasking (scripts/serve.py).
// Without the ops server (static hosting) zones stay in this browser's localStorage.
// Mission side: state.zones (status styling + coverage), state.zone_suggestions ("Add as zone"), recon path / perches,
// state.zone_proposals ("Accept", "Edit", "Accept all"). Nothing here is a confirmed person or a cleared area.

import { devicesIn } from './comms.js';

const API = '/api/tasking', LS = 'larpverse.tasking.draft.v1', PILE_R = 9;
const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const pct = v => (v == null ? '—' : Math.round(100 * v) + '%');
const lsGet = () => { try { return JSON.parse(localStorage.getItem(LS) || 'null'); } catch { return null; } };
const lsSet = v => { try { localStorage.setItem(LS, JSON.stringify(v)); } catch { /* private mode: in-memory only */ } };
export const circlePoly = (c, r, n = 16) => Array.from({ length: n }, (_, k) => [+(c[0] + r * Math.cos(2 * Math.PI * k / n)).toFixed(2), +(c[1] + r * Math.sin(2 * Math.PI * k / n)).toFixed(2)]);
function hull(pts) {   // monotone chain
  const p = [...pts].sort((a, b) => a[0] - b[0] || a[1] - b[1]); if (p.length < 3) return p;
  const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]), lo = [], up = [];
  for (const q of p) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], q) <= 0) lo.pop(); lo.push(q); }
  for (const q of p.reverse()) { while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], q) <= 0) up.pop(); up.push(q); }
  return lo.slice(0, -1).concat(up.slice(0, -1));
}
function grow(poly, m) {   // push each vertex out from the centroid by m metres (convex outlines)
  const c = centroid(poly); return poly.map(([x, y]) => { const d = Math.hypot(x - c[0], y - c[1]) || 1; return [+(x + (x - c[0]) / d * m).toFixed(2), +(y + (y - c[1]) / d * m).toFixed(2)]; });
}
export const centroid = P => [P.reduce((s, p) => s + p[0], 0) / P.length, P.reduce((s, p) => s + p[1], 0) / P.length];
function simplify(P, tol) {   // drop points closer than tol to the previous kept point, cap at 200
  const out = []; for (const p of P) if (!out.length || Math.hypot(p[0] - out[out.length - 1][0], p[1] - out[out.length - 1][1]) >= tol) out.push(p);
  while (out.length > 200) for (let k = out.length - 2; k > 0 && out.length > 200; k -= 2) out.splice(k, 1);
  return out;
}
const inPoly = (x, y, P) => { let c = false; for (let i = 0, j = P.length - 1; i < P.length; j = i++) { const [xi, yi] = P[i], [xj, yj] = P[j]; if ((yi > y) !== (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) c = !c; } return c; };

// zone status (mission side) -> [label, stroke, dash, fill/hatch]
const ZST = {
  unsearched: ['unsearched', 'rgba(226,234,240,0.85)', [6, 5], null],
  surveying: ['surveying', '#6fd36f', [], 'rgba(111,211,111,0.06)'],
  scanning: ['radar scan', '#ff9f43', [], 'rgba(255,159,67,0.06)'],
  searched_no_ping: ['searched — not cleared', '#8a9caa', [], 'hatch'],
  ping: ['ping', '#ff4545', [], 'rgba(255,69,69,0.07)'],
  lead: ['acoustic lead', '#22c3d6', [], 'rgba(34,195,214,0.06)'],
  beyond_radar_reach: ['beyond radar reach — acoustic leads only', '#b9c6cf', [3, 3], 'hatch'],
  cancelled: ['cancelled', 'rgba(138,156,170,0.5)', [1.5, 4], null],
};
const newEv = (S, z) => ['searched_no_ping', 'ping', 'beyond_radar_reach'].includes(z.status) && devicesIn(S, z).length > 0;
const ZSHORT = { searched_no_ping: 'searched, not cleared', beyond_radar_reach: 'beyond radar reach', lead: 'lead', scanning: 'scanning' };
export const zst = s => ZST[s] || [String(s || 'tasked').replace(/_/g, ' '), 'rgba(226,234,240,0.85)', [], null];

export function createTasking(app) {
  // app: { S(), toPx, toM, scale(), ctx(), draw(), qLabel, font, $ , onChange() }
  const T = {
    planning: false, tool: 'poly', zones: [], sel: null, drawing: null, lasso: null, dragV: null,
    server: null, revision: 0, dirty: false, msg: '', msgKind: '', hover: null,
  };
  const saved = lsGet();
  if (saved?.zones) { T.zones = saved.zones; T.revision = saved.revision || 0; T.dirty = !!saved.dirty; }
  const persist = () => lsSet({ zones: T.zones, revision: T.revision, dirty: T.dirty });
  const nextId = () => { let k = 1; const ids = new Set(T.zones.map(z => z.id)); while (ids.has('Z' + k)) k++; return 'Z' + k; };
  const say = (m, kind = '') => { T.msg = m; T.msgKind = kind; render(); };
  const NOSERVER = 'Tasking needs the ops server (python3 scripts/serve.py). Zones are kept in this browser until it is reachable.';

  async function load() {
    try {
      const r = await fetch(API, { cache: 'no-store' }); if (!r.ok) throw new Error(r.status);
      const doc = await r.json(); T.server = true; T.revision = doc.revision || 0;
      if (!T.dirty) T.zones = (doc.zones || []).map(z => ({ ...z })), persist();
      say(T.dirty ? `Unsent local changes (server is at revision ${T.revision}).` : doc.revision ? `Server tasking: revision ${doc.revision}, ${doc.zones.length} zone(s).` : 'No tasking sent yet: with no zones the drones run recon.');
    } catch { T.server = false; say(NOSERVER, 'warn'); }
  }
  async function post(zones, why) {
    const body = { schema: 'larpverse.tasking/v1', revision: T.revision, source: 'operator',
      zones: zones.map(z => ({ id: z.id, name: z.name || z.id, polygon: z.polygon, priority: z.priority ?? null, status: z.status || 'tasked', note: z.note || '' })) };
    try {
      const r = await fetch(API, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
      const doc = await r.json().catch(() => ({}));
      if (!r.ok) { T.server = true; say(`Server rejected the tasking: ${doc.error || r.status}`, 'err'); return false; }
      T.server = true; T.revision = doc.revision; T.zones = (doc.zones || []).map(z => ({ ...z })); T.dirty = false; persist();
      say(`${why} Sent as revision ${doc.revision} (${doc.zones.length} zone${doc.zones.length === 1 ? '' : 's'}); the mission re-reads it every step.`, 'ok');
      return true;
    } catch { T.server = false; T.dirty = true; persist(); say(NOSERVER, 'warn'); return false; }
  }
  // merge one or more zones into the server's current tasking (used by Add as zone / Accept)
  async function merge(newZones, why) {
    if (T.server !== false && !T.dirty) { try { const r = await fetch(API, { cache: 'no-store' }); if (r.ok) { const d = await r.json(); T.revision = d.revision || 0; T.zones = d.zones || []; } } catch { /* post() reports */ } }
    const have = new Set(T.zones.map(z => z.name));
    newZones = newZones.filter(z => !have.has(z.name));            // accepting twice never duplicates a zone
    if (!newZones.length) { say(`${why} Already in the tasking.`); return true; }
    for (const z of newZones) { z.id = nextId(); T.zones.push(z); }
    T.dirty = true; persist(); app.draw();
    return post(T.zones, why);
  }

  // ---------- prefill ----------
  function rubbleOutlines() {
    const S = app.S() || {}, out = [];
    for (const p of S.piles || []) {
      let poly = null;
      if (Array.isArray(p.polygon) && p.polygon.length >= 3) poly = p.polygon;
      else if (Array.isArray(p.outline) && p.outline.length >= 3) poly = p.outline;
      else if (Array.isArray(p.pieces) && p.pieces.length >= 3) poly = grow(hull(p.pieces.map(q => q.xy || q.center || q)), 2);
      else { const dz = (S.zones || []).find(z => z.pile_id === p.id || z.name?.startsWith(p.id)); poly = dz?.polygon || circlePoly(p.xy, PILE_R); }
      out.push({ name: `${p.id} rubble`, polygon: poly.map(q => [+(+q[0]).toFixed(2), +(+q[1]).toFixed(2)]), priority: null, status: 'tasked', note: p.polygon || p.outline ? 'sector outline' : p.pieces ? 'hull of the pile pieces + 2 m' : `pile centre + ${PILE_R} m` });
    }
    if (!out.length) for (const pr of S.zone_proposals || []) out.push({ name: `${pr.id} ${pr.kind === 'rubble_mound' ? 'mound' : 'proposal'}`, polygon: pr.polygon, priority: null, status: 'tasked', note: pr.why || '' });
    return out;
  }

  // ---------- pointer (map) ----------
  const hitV = (px, py) => { const z = T.zones.find(z => z.id === T.sel); if (!z) return -1; return z.polygon.findIndex(q => { const [x, y] = app.toPx(...q); return Math.hypot(x - px, y - py) < 11; }); };
  function finish() {
    const d = T.drawing; T.drawing = null;
    if (!d) return;
    const P = simplify(d, 0.3);
    if (P.length < 3) { say('A zone needs at least 3 points.', 'err'); return; }
    const z = { id: nextId(), name: '', polygon: P.map(q => [+q[0].toFixed(2), +q[1].toFixed(2)]), priority: null, status: 'tasked', note: '' };
    z.name = `Zone ${z.id.slice(1)}`; T.zones.push(z); T.sel = z.id; T.dirty = true; persist();
    say(`${z.id} added. Rename it, set a priority, then send the tasking to the drones.`); app.draw();
  }
  const ptr = {
    down(e, px, py) {
      if (!T.planning) return false;
      const vi = hitV(px, py); if (vi >= 0) { T.dragV = { i: vi }; return true; }
      if (T.tool === 'lasso') { T.lasso = [app.toM(px, py)]; return true; }
      return 'click';     // polygon tool: vertex added on a click (pointerup without a pan)
    },
    move(e, px, py) {
      if (!T.planning) return false;
      if (T.dragV) { const z = T.zones.find(z => z.id === T.sel); const [x, y] = app.toM(px, py); z.polygon[T.dragV.i] = [+x.toFixed(2), +y.toFixed(2)]; T.dirty = true; app.draw(); return true; }
      if (T.lasso) { T.lasso.push(app.toM(px, py)); app.draw(); return true; }
      T.hover = app.toM(px, py); return false;
    },
    up(e, px, py, moved) {
      if (!T.planning) return false;
      if (T.dragV) { T.dragV = null; persist(); render(); return true; }
      if (T.lasso) { T.drawing = T.lasso; T.lasso = null; finish(); return true; }
      if (moved) return false;
      if (T.tool === 'poly') {
        const m = app.toM(px, py);
        if (T.drawing?.length >= 3) { const [fx, fy] = app.toPx(...T.drawing[0]); if (Math.hypot(fx - px, fy - py) < 14) { finish(); return true; } }
        if (!T.drawing) {   // a click inside an existing zone selects it instead of starting a new one
          const z = [...T.zones].reverse().find(z => inPoly(m[0], m[1], z.polygon));
          if (z && T.sel !== z.id) { T.sel = z.id; render(); app.draw(); return true; }
          T.drawing = [];
        }
        const L = T.drawing[T.drawing.length - 1];
        if (!L || Math.hypot(L[0] - m[0], L[1] - m[1]) > 0.2) T.drawing.push(m);
        render(); app.draw(); return true;
      }
      return false;
    },
    dbl() { if (T.planning && T.drawing) { finish(); return true; } return false; },
    key(e) {
      if (!T.planning) return false;
      if (e.key === 'Enter' && T.drawing) { finish(); return true; }
      if (e.key === 'Escape' && T.drawing) { T.drawing = null; render(); app.draw(); return true; }
      if (e.key === 'Backspace' && T.drawing?.length && !/INPUT|TEXTAREA/.test(document.activeElement?.tagName)) { T.drawing.pop(); app.draw(); return true; }
      return false;
    },
  };

  // ---------- drawing ----------
  const trails = new Map(); let trailT = -1;
  function polyPath(ctx, P) { ctx.beginPath(); P.forEach((q, k) => { const [x, y] = app.toPx(...q); k ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.closePath(); }
  function hatch(ctx, P, color, gap = 7) {
    ctx.save(); polyPath(ctx, P); ctx.clip();
    const xs = P.map(q => app.toPx(...q)), x0 = Math.min(...xs.map(q => q[0])), x1 = Math.max(...xs.map(q => q[0])), y0 = Math.min(...xs.map(q => q[1])), y1 = Math.max(...xs.map(q => q[1]));
    ctx.strokeStyle = color; ctx.lineWidth = 1; ctx.beginPath();
    for (let k = x0 - (y1 - y0); k < x1; k += gap) { ctx.moveTo(k, y1); ctx.lineTo(k + (y1 - y0), y0); }
    ctx.stroke(); ctx.restore();
  }
  function topOf(P) { let b = P[0]; for (const q of P) if (q[1] > b[1]) b = q; return b; }
  function label(P, text, color, prio, bg, stroke) {
    const c = centroid(P), t = topOf(P), [x, y] = app.toPx(c[0], t[1]), ctx = app.ctx(); ctx.font = app.font;
    const w = ctx.measureText(text).width; app.qLabel(text, x - w / 2, y - 6, color, bg, prio, stroke);
  }
  function drawMission(ctx, layers) {
    const S = app.S(); if (!S) return;
    const small = app.scale() < 6.5;
    // recon: planned path, tracks, perches
    const rc = S.recon;
    if (S.t < trailT) trails.clear(); trailT = S.t ?? 0;
    for (const d of S.drones || []) {
      const tr = trails.get(d.id) || []; const L = tr[tr.length - 1];
      if (!L || Math.hypot(L[0] - d.pose[0], L[1] - d.pose[1]) > 0.8) tr.push([d.pose[0], d.pose[1]]);
      if (tr.length > 1500) tr.shift(); trails.set(d.id, tr);
    }
    if (rc) {
      if (Array.isArray(rc.path) && rc.path.length > 1) {
        ctx.save(); ctx.strokeStyle = 'rgba(226,234,240,0.45)'; ctx.lineWidth = 1; ctx.setLineDash([3, 4]); ctx.beginPath();
        rc.path.forEach((q, k) => { const [x, y] = app.toPx(q[0], q[1]); k ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke(); ctx.restore();
      }
      ctx.save(); ctx.lineWidth = 1.6; ctx.strokeStyle = 'rgba(226,234,240,0.75)';
      for (const d of S.drones || []) { const tr = (Array.isArray(d.track) && d.track.length > 1 ? d.track : trails.get(d.id)) || []; if (tr.length < 2) continue;
        ctx.beginPath(); tr.forEach((q, k) => { const [x, y] = app.toPx(q[0], q[1]); k ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke(); }
      ctx.restore();
      for (const p of rc.perches || []) {
        const [x, y] = app.toPx(...p.xy), n = (p.clips || []).length;
        ctx.save(); ctx.fillStyle = 'rgba(11,16,22,0.85)'; ctx.strokeStyle = '#e3eaef'; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.arc(x, y, 5, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
        ctx.beginPath(); ctx.arc(x, y, 9, -0.35 * Math.PI, 0.35 * Math.PI); ctx.stroke(); ctx.beginPath(); ctx.arc(x, y, 9, 0.65 * Math.PI, 1.35 * Math.PI); ctx.stroke(); ctx.restore();
        if (!small) app.qLabel(`perch ${p.drone_id}${p.t != null ? ' · t ' + Math.round(p.t) + ' s' : ''}${n ? ` · ${n} clip${n > 1 ? 's' : ''}` : ''}`, x + 11, y + 4, '#c3d0d9', undefined, 7);
      }
    }
    // proposals: neutral dashed with a "proposed" tag
    for (const pr of S.zone_proposals || []) {
      if (!Array.isArray(pr.polygon) || pr.polygon.length < 3) continue;
      ctx.save(); polyPath(ctx, pr.polygon); ctx.fillStyle = 'rgba(226,234,240,0.04)'; ctx.fill();
      ctx.strokeStyle = 'rgba(226,234,240,0.8)'; ctx.lineWidth = 1.4; ctx.setLineDash([7, 5]); ctx.stroke(); ctx.restore();
      label(pr.polygon, small ? `proposed ${pr.id}` : `proposed ${pr.id}${pr.score != null ? ' · score ' + (+pr.score).toFixed(2) : ''}`, '#e3eaef', 5, 'rgba(11,16,22,0.84)', 'rgba(226,234,240,0.5)');
    }
    // tasked zones as reported by the mission
    if (!T.planning) for (const z of S.zones || []) {
      if (!Array.isArray(z.polygon) || z.polygon.length < 3) continue;
      const [lab, col, dash, fill] = zst(z.status);
      ctx.save(); polyPath(ctx, z.polygon);
      if (fill && fill !== 'hatch') { ctx.fillStyle = fill; ctx.fill(); }
      ctx.strokeStyle = col; ctx.lineWidth = z.status === 'ping' ? 2.2 : 1.6; ctx.setLineDash(dash); ctx.stroke(); ctx.restore();
      if (fill === 'hatch') hatch(ctx, z.polygon, z.status === 'searched_no_ping' ? 'rgba(138,156,170,0.45)' : 'rgba(205,218,228,0.4)');
      const cv = z.coverage || {}, short = ZSHORT[z.status] || lab;
      const chip = !small && cv.radar_reached != null && !['unsearched', 'cancelled'].includes(z.status) ? ` · radar ${pct(cv.radar_reached)}` : '';
      const ev = newEv(S, z) ? ' · new evidence — re-search?' : '';
      label(z.polygon, small ? `${z.id} · ${short}${ev ? ' · new evidence' : ''}` : `${z.name || z.id} · ${short}${chip}${ev}`, col === 'rgba(226,234,240,0.85)' ? '#e3eaef' : col, 4);
    }
    // suggestions: dot-dash circles
    for (const sg of S.zone_suggestions || []) {
      const [x, y] = app.toPx(...sg.centre), r = (sg.radius_m || 6) * app.scale();
      ctx.save(); ctx.strokeStyle = 'rgba(226,234,240,0.8)'; ctx.lineWidth = 1.3; ctx.setLineDash([2, 4, 8, 4]);
      ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.stroke(); ctx.restore();
      app.qLabel(`suggested ${sg.id}`, x - 30, y - r - 6, '#e3eaef', undefined, 5, 'rgba(226,234,240,0.4)');
    }
  }
  function drawPlan(ctx) {
    if (!T.planning) return;
    for (const z of T.zones) {
      if (!z.polygon?.length) continue; const sel = z.id === T.sel;
      ctx.save(); polyPath(ctx, z.polygon); ctx.fillStyle = sel ? 'rgba(240,244,248,0.14)' : 'rgba(240,244,248,0.07)'; ctx.fill();
      ctx.strokeStyle = '#f0f4f8'; ctx.lineWidth = sel ? 2.4 : 1.6; ctx.setLineDash(z.status === 'cancelled' ? [2, 4] : []); ctx.stroke(); ctx.restore();
      if (sel) for (const q of z.polygon) { const [x, y] = app.toPx(...q); ctx.fillStyle = '#0b1016'; ctx.strokeStyle = '#f0f4f8'; ctx.lineWidth = 1.5; ctx.fillRect(x - 4, y - 4, 8, 8); ctx.strokeRect(x - 4, y - 4, 8, 8); }
      label(z.polygon, `${z.priority != null ? '#' + z.priority + ' ' : ''}${z.name || z.id}`, '#f0f4f8', 2);
    }
    const d = T.lasso || T.drawing;
    if (d?.length) {
      ctx.save(); ctx.strokeStyle = '#f0f4f8'; ctx.lineWidth = 2; ctx.setLineDash([5, 4]); ctx.beginPath();
      d.forEach((q, k) => { const [x, y] = app.toPx(...q); k ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
      if (T.drawing && T.hover) { const [x, y] = app.toPx(...T.hover); ctx.lineTo(x, y); }
      ctx.stroke(); ctx.setLineDash([]);
      if (T.drawing) d.forEach((q, k) => { const [x, y] = app.toPx(...q); ctx.fillStyle = k === 0 ? '#f0f4f8' : '#0b1016'; ctx.strokeStyle = '#f0f4f8'; ctx.beginPath(); ctx.arc(x, y, k === 0 ? 6 : 4, 0, Math.PI * 2); ctx.fill(); ctx.stroke(); });
      ctx.restore();
    }
  }

  // ---------- panels ----------
  const $ = app.$;
  function render() {
    const p = $('plan'); if (!p) return;
    p.hidden = !T.planning; $('planBtn').setAttribute('aria-pressed', T.planning); document.body.classList.toggle('planning', T.planning);
    if (T.planning) {
      const n = T.drawing?.length || 0;
      $('planTools').innerHTML = `<div class="seg" role="group" aria-label="Drawing tool"><span>Draw</span>
          <button class="tog" data-tool="poly" aria-pressed="${T.tool === 'poly'}" title="Click (tap) vertices; double-click, Enter or tap the first vertex to close">Polygon</button>
          <button class="tog" data-tool="lasso" aria-pressed="${T.tool === 'lasso'}" title="Drag a freehand outline">Lasso</button></div>
        ${T.drawing ? `<button class="btn" data-act="finish" ${n < 3 ? 'disabled' : ''}>Close shape (${n})</button><button class="btn" data-act="undo">Undo point</button><button class="btn" data-act="cancel">Cancel</button>` : ''}
        <button class="btn" data-act="prefill" title="Pre-fill zones from the rubble outlines (or a ${PILE_R} m circle around each pile centre)">Use rubble outlines</button>`;
      $('planHint').textContent = T.drawing ? (T.tool === 'poly' ? 'Click / tap each corner. Double-click, Enter, or tap the first point to close.' : '') : T.tool === 'lasso' ? 'Drag on the map to draw a freehand zone.' : 'Click / tap corners on the map to draw a zone; click a zone to select it and drag its corners.';
      $('planList').innerHTML = T.zones.length ? T.zones.map(z => `<div class="zrow ${z.id === T.sel ? 'sel' : ''}" data-z="${esc(z.id)}">
          <button class="zid" data-act="sel" title="Select to drag its corners">${esc(z.id)}</button>
          <input aria-label="Zone name" data-f="name" value="${esc(z.name || '')}" maxlength="60">
          <input aria-label="Priority (1 = first)" data-f="priority" type="number" min="1" max="99" placeholder="prio" value="${z.priority ?? ''}">
          <button class="btn x" data-act="del" aria-label="Delete ${esc(z.id)}">✕</button></div>`).join('')
        : '<p class="empty">No zones yet. With no zones tasked the drones run recon (mapping flight, perch-and-listen) and propose zones.</p>';
      $('planSend').disabled = !T.zones.length && !T.revision;
      $('planSend').textContent = T.server === false ? 'Send tasking to drones (ops server offline)' : `Send tasking to drones${T.dirty ? ' •' : ''}`;
    }
    $('planMsg').className = 'tmsg ' + T.msgKind; $('planMsg').textContent = T.msg; $('planMsg').hidden = !T.msg || !T.planning;
    renderRail();
  }
  // Operator flow on the map: [Select area to scan] -> draw -> [Send drones]; then suggested next areas, one click each.
  function renderCTA() {
    const el = $('cta'); if (!el) return;
    const S = app.S() || {}, tasked = (S.zones || []).filter(z => z.status !== 'cancelled').length + (T.revision && !T.dirty ? T.zones.length : 0);
    let h = '';
    if (!T.planning) h = tasked ? '<button class="cta" data-cta="start">+ Select another area</button>' : '<button class="cta big" data-cta="start">Select area to scan</button>';
    else if (T.drawing || T.lasso) h = `<span class="cta-hint">Click the corners of the area · double-click or tap the first corner to finish</span><button class="cta ghost" data-cta="cancel">Cancel</button>`;
    else if (T.dirty && T.zones.length) h = `<button class="cta big go" data-cta="send">Send drones → ${T.zones.length} area${T.zones.length > 1 ? 's' : ''}</button><button class="cta ghost" data-cta="clear">Redraw</button>`;
    else h = `<span class="cta-hint">Click the corners of the area to scan on the map</span><button class="cta ghost" data-cta="cancel">Cancel</button>`;
    el.classList.toggle('docked', !T.planning && tasked > 0);   // tasked: a small button out of the way; idle / drawing: top centre
    el.innerHTML = h + (T.msg && /Sent as revision|rejected|ops server/.test(T.msg) ? `<span class="cta-msg ${T.msgKind}">${esc(T.msg)}</span>` : '');
    // suggested next area (acoustics) as a map card, hidden while drawing
    const sg = $('suggest'); if (!sg) return;
    const tasknames = new Set([...T.zones, ...(S.zones || [])].map(z => z.name));
    const props = [...(S.zone_proposals || [])].filter(p => p.polygon?.length >= 3 && !tasknames.has(`${p.id} ${p.kind === 'rubble_mound' ? 'mound' : 'proposal'}`)).sort((a, b) => (b.score ?? 0) - (a.score ?? 0));
    const sugs = (S.zone_suggestions || []).filter(x => !tasknames.has(`${x.id} suggested`));
    const top = sugs[0] ? { kind: 's', id: sugs[0].id, why: sugs[0].why } : props[0] ? { kind: 'p', id: props[0].id, why: props[0].why } : null;
    sg.hidden = !top || T.planning;
    if (top) sg.innerHTML = `<div class="sh">Suggested next area · from acoustics</div><div class="sb"><b>${esc(top.id)}</b> ${esc(top.why || '')}</div>
      <button class="cta go" data-cta="${top.kind === 's' ? 'acceptS' : 'acceptP'}" data-id="${esc(top.id)}">Accept → send drones</button>${sugs.length + props.length > 1 ? `<span class="more">+${sugs.length + props.length - 1} more in the rail</span>` : ''}`;
  }
  function renderRail() {
    renderCTA();
    const S = app.S() || {}, sec = $('secZones'); if (!sec) return;
    const rc = S.recon, zs = S.zones || [], sg = S.zone_suggestions || [], pr = [...(S.zone_proposals || [])].sort((a, b) => (b.score ?? 0) - (a.score ?? 0));
    const any = rc || zs.length || sg.length || pr.length;
    sec.hidden = !any; if (!any) return;
    const tasked = new Set([...T.zones, ...zs].map(z => z.name));
    $('nZones').textContent = zs.length ? `${zs.filter(z => !['unsearched', 'cancelled'].includes(z.status)).length}/${zs.length}` : pr.length ? `${pr.length} proposed` : '—';
    const rcTxt = rc ? `<div class="recon"><b>${esc(reconText(rc))}</b>${rc.perches?.length ? ` · ${rc.perches.length} perch${rc.perches.length > 1 ? 'es' : ''}` : ''}${bar(rc.mapped_fraction)}</div>` : '';
    const prop = pr.length ? `<div class="sub">Proposed zones <button class="btn sm" data-act="acceptAll">Accept all</button></div>` + pr.map(p => `<div class="zp" data-p="${esc(p.id)}">
        <div class="zh"><span class="tag">proposed</span><b>${esc(p.id)}</b><span class="mut">${p.kind ? esc(p.kind.replace(/_/g, ' ')) : ''}${p.height_max_m != null ? ' · ' + (+p.height_max_m).toFixed(1) + ' m high' : ''}${p.area_m2 != null ? ' · ' + Math.round(p.area_m2) + ' m²' : ''}</span><span class="sc">${p.score != null ? (+p.score).toFixed(2) : ''}</span></div>
        ${p.why ? `<div class="zw">${esc(p.why)}</div>` : ''}
        <div class="za">${tasked.has(`${p.id} ${p.kind === 'rubble_mound' ? 'mound' : 'proposal'}`) ? '<span class="mut">accepted ✓</span>' : '<button class="btn sm" data-act="accept">Accept → send drones</button><button class="btn sm" data-act="edit">Edit</button>'}</div></div>`).join('') : '';
    const zl = zs.length ? `<div class="sub">Tasked zones</div>` + zs.map(z => { const [lab, col] = zst(z.status), cv = z.coverage || {};
        return `<div class="zt"><b>${esc(z.name || z.id)}</b><span class="chip" style="color:${col};border-color:${col}">${esc(lab)}</span>
          <span class="cov">${cv.lidar_mapped != null ? `<span>LiDAR ${pct(cv.lidar_mapped)}</span>` : ''}${cv.radar_reached != null ? `<span>radar ${pct(cv.radar_reached)}</span>` : ''}${cv.listened != null ? `<span>${cv.listened ? 'listened' : 'not listened'}</span>` : ''}${(z.pings || []).length ? `<span class="pg">${esc(z.pings.join(', '))}</span>` : ''}</span>
          ${z.retask_reason ? `<span class="zw">reopened: ${esc(z.retask_reason)}</span>` : ''}${newEv(S, z) ? `<span class="zev">phone on rescue Wi-Fi inside: new evidence — re-search?</span>` : ''}</div>`; }).join('') : '';
    const sl = sg.length ? `<div class="sub">Suggested next areas (from acoustics)</div>` + sg.map(s => `<div class="zp" data-s="${esc(s.id)}"><div class="zh"><span class="tag">suggested</span><b>${esc(s.id)}</b><span class="mut">${s.radius_m || 6} m radius</span></div>
        ${s.why ? `<div class="zw">${esc(s.why)}</div>` : ''}<div class="za">${tasked.has(`${s.id} suggested`) ? '<span class="mut">added ✓</span>' : '<button class="btn sm" data-act="addSug">Accept → send drones</button>'}</div></div>`).join('') : '';
    $('zones').innerHTML = rcTxt + prop + zl + sl + (T.msg && !T.planning ? `<p class="tmsg ${T.msgKind}">${esc(T.msg)}</p>` : '')
      + (zs.some(z => z.status === 'searched_no_ping') ? '<p class="caveat0">“Searched — not cleared”: no detection is not proof that nobody is there.</p>' : '');
  }
  const bar = v => (v == null ? '' : `<div class="bar"><i style="width:${Math.round(100 * Math.max(0, Math.min(1, v)))}%"></i></div>`);

  function bind() {
    $('planBtn').onclick = () => { T.planning = !T.planning; if (T.planning) load(); else { T.drawing = null; T.lasso = null; } render(); app.draw(); };
    $('planClose').onclick = () => { T.planning = false; T.drawing = null; render(); app.draw(); };
    $('plan').addEventListener('click', e => {
      const b = e.target.closest('[data-tool],[data-act]'); if (!b) return;
      if (b.dataset.tool) { T.tool = b.dataset.tool; T.drawing = null; render(); app.draw(); return; }
      const a = b.dataset.act, row = b.closest('[data-z]'), id = row?.dataset.z;
      if (a === 'finish') finish();
      else if (a === 'undo') { T.drawing?.pop(); app.draw(); }
      else if (a === 'cancel') { T.drawing = null; render(); app.draw(); }
      else if (a === 'prefill') { const add = rubbleOutlines(); for (const z of add) { z.id = nextId(); T.zones.push(z); } T.dirty = true; persist(); say(`Added ${add.length} zone(s) from the rubble outlines.`); app.draw(); }
      else if (a === 'sel') { T.sel = T.sel === id ? null : id; render(); app.draw(); }
      else if (a === 'del') { T.zones = T.zones.filter(z => z.id !== id); if (T.sel === id) T.sel = null; T.dirty = true; persist(); render(); app.draw(); }
    });
    $('plan').addEventListener('change', e => {
      const f = e.target.dataset.f, id = e.target.closest('[data-z]')?.dataset.z, z = T.zones.find(z => z.id === id); if (!f || !z) return;
      if (f === 'name') z.name = e.target.value.trim() || z.id;
      if (f === 'priority') { const v = parseInt(e.target.value, 10); z.priority = Number.isFinite(v) && v > 0 ? v : null; }
      T.dirty = true; persist(); render(); app.draw();
    });
    $('planSend').onclick = () => post(T.zones, 'Tasking accepted by the ops server.').then(ok => { if (ok) { T.planning = false; render(); app.draw(); } });
    const startPlan = () => { T.planning = true; T.tool = 'poly'; T.drawing = null; load(); render(); app.draw(); };
    const onCta = e => {
      const b = e.target.closest('[data-cta]'); if (!b) return; const S = app.S() || {}, a = b.dataset.cta;
      if (a === 'start') startPlan();
      else if (a === 'cancel') { T.planning = false; T.drawing = null; T.lasso = null; render(); app.draw(); }
      else if (a === 'clear') { T.dirty = false; T.drawing = null; persist(); load().then(() => { render(); app.draw(); }); }   // back to what the server has
      else if (a === 'send') post(T.zones, 'Drones dispatched.').then(ok => { if (ok) { T.planning = false; render(); app.draw(); } });
      else if (a === 'acceptS') { const x = (S.zone_suggestions || []).find(q => q.id === b.dataset.id); if (x) merge([{ name: `${x.id} suggested`, polygon: circlePoly(x.centre, x.radius_m || 6), priority: null, status: 'tasked', note: (x.why || '').slice(0, 200) }], `Accepted ${x.id}: drones dispatched.`); }
      else if (a === 'acceptP') { const p = (S.zone_proposals || []).find(q => q.id === b.dataset.id); if (p) merge([{ name: `${p.id} ${p.kind === 'rubble_mound' ? 'mound' : 'proposal'}`, polygon: p.polygon, priority: null, status: 'tasked', note: (p.why || '').slice(0, 200) }], `Accepted ${p.id}: drones dispatched.`); }
    };
    $('cta').addEventListener('click', onCta); $('suggest').addEventListener('click', onCta);
    $('zones').addEventListener('click', e => {
      const b = e.target.closest('[data-act]'); if (!b) return; const S = app.S() || {};
      const a = b.dataset.act, pid = b.closest('[data-p]')?.dataset.p, sid = b.closest('[data-s]')?.dataset.s;
      const fromP = p => ({ name: `${p.id} ${p.kind === 'rubble_mound' ? 'mound' : 'proposal'}`, polygon: p.polygon, priority: null, status: 'tasked', note: (p.why || '').slice(0, 200) });
      if (a === 'accept') { const p = (S.zone_proposals || []).find(q => q.id === pid); if (p) merge([fromP(p)], `Accepted ${p.id}.`); }
      else if (a === 'acceptAll') merge((S.zone_proposals || []).filter(p => p.polygon?.length >= 3).map(fromP), 'Accepted all proposals.');
      else if (a === 'edit') { const p = (S.zone_proposals || []).find(q => q.id === pid); if (!p) return; const z = { ...fromP(p), id: nextId() }; T.zones.push(z); T.sel = z.id; T.dirty = true; persist(); T.planning = true; say(`${p.id} copied into the plan as ${z.id}: drag its corners, then send.`); app.draw(); }
      else if (a === 'addSug') { const s = (S.zone_suggestions || []).find(q => q.id === sid); if (s) merge([{ name: `${s.id} suggested`, polygon: circlePoly(s.centre, s.radius_m || 6), priority: null, status: 'tasked', note: (s.why || '').slice(0, 200) }], `Added ${s.id} as a zone.`); }
    });
  }
  bind(); render();
  return { ptr, draw(ctx, layers) { drawMission(ctx, layers); drawPlan(ctx); }, render, renderRail, get planning() { return T.planning; }, _T: T };
}
export function reconText(rc) {
  const m = rc.mapped_fraction != null ? ` ${Math.round(100 * rc.mapped_fraction)}%` : '';
  return rc.status === 'mapping' ? `Recon: mapping${m}` : rc.status === 'listening' ? 'Recon: listening at perch' : rc.status === 'proposing' ? 'Recon: proposing zones' : rc.status === 'done' ? 'Recon done' : `Recon: ${rc.status || '…'}`;
}
