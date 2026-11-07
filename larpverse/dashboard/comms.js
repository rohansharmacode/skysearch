// Rescue hotspot + comms relay (contracts/INTERFACES.md "Rescue hotspot and comms relay (v1.4)"). Blue = comms only.
// Map: hotspot coverage per drone, mesh backhaul links (Mbps) to the GCS, connected-phone candidate regions (σ).
// Rail: "Connected" panel with each device's status, messages (original + translation + who translated) and replies
// from VETTED TEMPLATES only. Free text is held as "needs operator confirmation". Replies are recorded to a local log
// in this browser (no send endpoint yet). Live: polls GET /api/portal when the ops server offers it and toasts new
// submissions. A phone position is a consent-based candidate region from multi-drone RSSI, never a confirmed position.

export const BLUE = '#3b82f6';
const LS = 'larpverse.comms.replies.v1', POLL_MS = 3000, RETRY_MS = 60000;
const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const lsGet = () => { try { return JSON.parse(localStorage.getItem(LS) || '[]'); } catch { return []; } };
const lsSet = v => { try { localStorage.setItem(LS, JSON.stringify(v.slice(-200))); } catch { /* private mode */ } };
export const TEMPLATES = {
  help_coming: { label: 'Help is coming', en: 'Help is coming. Rescuers know where you are.' },
  stay_put_save_battery: { label: 'Stay put, save battery', en: 'Stay where you are if it is safe. Save battery: dim the screen and reply only when asked.' },
  tap_rhythm: { label: 'Tap a rhythm', en: 'If you can, tap 3 times on metal or concrete, then wait. Repeat every minute.' },
  move_to_light: { label: 'Move toward light', en: 'Only if you can move safely: move toward light or open air, then stay still.' },
};
const STATUS = { trapped: ['trapped', 'Says they are trapped'], safe: ['safe', 'Says they are safe'], unknown: ['unknown', 'No answer yet'] };
const LANG = { zh: 'Mandarin', 'zh-tw': 'Mandarin (Taiwan)', nan: 'Taiwanese Hokkien', en: 'English', id: 'Indonesian', vi: 'Vietnamese', tl: 'Tagalog', th: 'Thai', ja: 'Japanese' };
const lname = c => (c ? LANG[String(c).toLowerCase()] || LANG[String(c).toLowerCase().split('-')[0]] || c : '');
const inPoly = (x, y, P) => { let c = false; for (let i = 0, j = P.length - 1; i < P.length; j = i++) { const [xi, yi] = P[i], [xj, yj] = P[j]; if ((yi > y) !== (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) c = !c; } return c; };
const SEARCHED = new Set(['searched_no_ping', 'ping', 'beyond_radar_reach', 'lead']);

// Zones (or visited piles) a device sits in that were already searched: "new evidence — re-search?"
export function evidenceOf(S, d) {
  if (!d?.est_xy) return [];
  const out = [];
  for (const z of S?.zones || []) if (Array.isArray(z.polygon) && z.polygon.length >= 3 && SEARCHED.has(z.status) && inPoly(d.est_xy[0], d.est_xy[1], z.polygon)) out.push({ kind: 'zone', id: z.id, name: z.name || z.id });
  if (!out.length && d.pile && (S?.mission?.visited || []).includes(d.pile)) out.push({ kind: 'pile', id: d.pile, name: d.pile });
  return out;
}
export const devicesIn = (S, zone) => (S?.comms?.devices || []).filter(d => d.est_xy && Array.isArray(zone.polygon) && zone.polygon.length >= 3 && inPoly(d.est_xy[0], d.est_xy[1], zone.polygon));

export function createComms(app) {
  // app: { S(), toPx, scale(), ctx(), qLabel, font, $, draw() }
  const C = { log: lsGet(), live: null, liveSubs: [], seen: new Set(), first: true, pollT: null, open: new Set(), draft: {} };
  const $ = app.$;

  // ---------- map ----------
  function nodeXY(S, id) {
    const cm = S.comms || {};
    if (id === 'gcs') return cm.backhaul?.gcs_xy || S.staging?.xy || null;
    const dr = (S.drones || []).find(d => d.id === id); if (dr) return [dr.pose[0], dr.pose[1]];
    const h = (cm.hotspots || []).find(h => h.drone_id === id); return h?.xy || null;
  }
  function draw(ctx, layers) {
    const S = app.S(), cm = S?.comms; if (!cm || layers.comms === false) return;
    const sc = app.scale(), small = sc < 6.5;
    for (const h of cm.hotspots || []) {
      if (h.on === false) continue;
      const xy = nodeXY(S, h.drone_id) || h.xy; if (!xy) continue;
      const [x, y] = app.toPx(...xy), r = (h.radius_m_est || 30) * sc;
      ctx.save(); ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.fillStyle = 'rgba(59,130,246,0.07)'; ctx.fill();
      ctx.strokeStyle = 'rgba(59,130,246,0.45)'; ctx.lineWidth = 1; ctx.setLineDash([2, 5]); ctx.stroke(); ctx.restore();
    }
    for (const l of cm.backhaul?.links || []) {
      const a = nodeXY(S, l.from), b = nodeXY(S, l.to); if (!a || !b) continue;
      const [x0, y0] = app.toPx(...a), [x1, y1] = app.toPx(...b);
      ctx.save(); ctx.strokeStyle = 'rgba(96,150,255,0.85)'; ctx.lineWidth = 1.3; ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke(); ctx.restore();
      if (l.mbps != null && !small) { const t = `${(+l.mbps).toFixed(0)} Mbps`; ctx.font = app.font; const w = ctx.measureText(t).width; app.qLabel(t, (x0 + x1) / 2 - w / 2, (y0 + y1) / 2 - 4, '#a9c8ff', 'rgba(11,16,22,0.82)', 8); }
    }
    const g = cm.backhaul?.gcs_xy || S.staging?.xy;
    if (g) {
      const [x, y] = app.toPx(...g);
      ctx.save(); ctx.fillStyle = '#0b1016'; ctx.strokeStyle = BLUE; ctx.lineWidth = 1.6; ctx.fillRect(x - 7, y - 5, 14, 10); ctx.strokeRect(x - 7, y - 5, 14, 10);
      ctx.beginPath(); ctx.moveTo(x, y - 5); ctx.lineTo(x, y - 13); ctx.stroke(); ctx.beginPath(); ctx.arc(x, y - 13, 4, Math.PI * 1.15, Math.PI * 1.85); ctx.stroke(); ctx.restore();
      const net = cm.backhaul?.internet;
      app.qLabel(`GCS · internet ${net ? 'on' : 'off'}`, x + 11, y + 4, net ? '#a9c8ff' : '#8a9caa', 'rgba(11,16,22,0.88)', 3, net ? 'rgba(59,130,246,0.7)' : 'rgba(138,156,170,0.5)');
    }
    for (const d of cm.devices || []) {
      if (!d.est_xy) continue;
      const [x, y] = app.toPx(...d.est_xy), st = d.status || 'unknown', trapped = st === 'trapped';
      const r = (d.est_sigma_m || 5) * sc;
      ctx.save(); ctx.beginPath(); ctx.arc(x, y, Math.max(6, r), 0, Math.PI * 2); ctx.fillStyle = trapped ? 'rgba(59,130,246,0.14)' : 'rgba(59,130,246,0.06)'; ctx.fill();
      ctx.strokeStyle = trapped ? 'rgba(59,130,246,0.9)' : 'rgba(59,130,246,0.55)'; ctx.lineWidth = 1.2; ctx.setLineDash([4, 3]); ctx.stroke(); ctx.setLineDash([]);
      if (trapped) { ctx.strokeStyle = BLUE; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(x, y, 11, 0, Math.PI * 2); ctx.stroke(); }
      ctx.fillStyle = trapped ? BLUE : st === 'safe' ? 'rgba(147,197,253,0.9)' : 'rgba(96,150,255,0.6)'; ctx.strokeStyle = '#e8f0ff'; ctx.lineWidth = 1.2;
      const w = 7, h = 11; ctx.beginPath(); ctx.roundRect ? ctx.roundRect(x - w / 2, y - h / 2, w, h, 1.6) : ctx.rect(x - w / 2, y - h / 2, w, h); ctx.fill(); ctx.stroke();
      ctx.fillStyle = '#0b1016'; ctx.fillRect(x - 1.2, y + h / 2 - 2.6, 2.4, 1.2); ctx.restore();
      if (!small || trapped) app.qLabel(`${d.id} · ${st}`, x + 13, y - 8, trapped ? '#cfe0ff' : '#a9c8ff', trapped ? 'rgba(14,32,72,0.92)' : 'rgba(11,16,22,0.84)', trapped ? 2 : 6, trapped ? 'rgba(59,130,246,0.8)' : null);
    }
  }

  // ---------- rail ----------
  const ago = (S, t) => (t == null || S?.t == null ? '' : ` (${Math.max(0, Math.round(S.t - t))} s ago)`);
  function msgsOf(d) {
    const live = C.liveSubs.filter(s => s.device === d.id && s.text).map(s => ({ ...s, live: true }));
    const m = [...(d.messages || []), ...live]; const seen = new Set();
    return m.filter(x => { const k = `${x.t}|${x.text}`; if (seen.has(k)) return false; seen.add(k); return true; }).sort((a, b) => (a.t ?? 0) - (b.t ?? 0));
  }
  function devices(S) {
    const ds = [...(S?.comms?.devices || [])];
    for (const s of C.liveSubs) if (s.device && !ds.some(d => d.id === s.device)) ds.push({ id: s.device, status: s.status || 'unknown', battery_pct: s.battery_pct, first_seen: s.t, last_seen: s.t, portal_only: true });
    for (const d of ds) { const s = [...C.liveSubs].reverse().find(s => s.device === d.id && s.status); if (s) d.live_status = s.status; }
    const rank = d => ({ trapped: 0, unknown: 1, safe: 2 }[d.live_status || d.status] ?? 1);
    return ds.sort((a, b) => rank(a) - rank(b) || (b.last_seen ?? 0) - (a.last_seen ?? 0));
  }
  function render(force = false) {
    const S = app.S(), sec = $('secComms'); if (!sec) return;
    const host = $('comms'), ae = document.activeElement;
    if (host && host.contains(ae) && /INPUT|TEXTAREA|SELECT/.test(ae.tagName) && !force) { deferred = true; return; }
    deferred = false;
    const cm = S?.comms, ds = devices(S);
    sec.hidden = !cm && !C.liveSubs.length; if (sec.hidden) return;
    $('nComms').textContent = `${ds.length} device${ds.length === 1 ? '' : 's'}`;
    const bh = cm?.backhaul, links = (bh?.links || []).map(l => `${esc(l.from)}→${esc(l.to === 'gcs' ? 'GCS' : l.to)}${l.mbps != null ? ' ' + (+l.mbps).toFixed(0) + ' Mbps' : ''}`).join(' · ');
    const hs = (cm?.hotspots || []).filter(h => h.on !== false);
    $('commsMeta').innerHTML = `${hs.length ? `<span><b>${hs.length}</b> hotspot${hs.length > 1 ? 's' : ''} ${esc(hs[0].ssid || '')}</span>` : ''}${links ? `<span>mesh ${links}</span>` : ''}`
      + `${bh ? `<span class="net ${bh.internet ? 'on' : ''}">internet ${bh.internet ? 'on' : 'off'}</span>` : ''}`
      + `<span class="live ${C.live ? 'on' : ''}" title="${C.live ? 'Polling GET /api/portal' : 'No /api/portal on this server: showing state.json only'}">${C.live ? 'portal live' : 'portal feed off'}</span>`;
    $('comms').innerHTML = ds.length ? ds.map(d => card(S, d)).join('') : '<p class="empty">No phone has joined the rescue Wi-Fi yet.</p>';
  }
  function card(S, d) {
    const st = d.live_status || d.status || 'unknown', ev = evidenceOf(S, d), ms = msgsOf(d), lang = [...ms].reverse().find(m => m.lang)?.lang || null;
    const reps = [...(d.replies || []).map(r => ({ ...r, src: 'state' })), ...C.log.filter(r => r.device === d.id)].sort((a, b) => (a.t ?? 0) - (b.t ?? 0));
    const nDr = new Set((d.rssi || []).map(r => r.drone_id)).size;
    return `<div class="dev s-${esc(st)}" data-dev="${esc(d.id)}">
      <div class="dh"><i class="ph"></i><b>${esc(d.id)}</b><span class="chip st">${esc(STATUS[st]?.[0] || st)}</span>
        <span class="mut">seen t ${d.last_seen != null ? Math.round(d.last_seen) + ' s' : '—'}${esc(ago(S, d.last_seen))}${d.pile ? ' · sector ' + esc(d.pile) : ''}${d.battery_pct != null ? ' · battery ' + Math.round(d.battery_pct) + '%' : ''}</span></div>
      ${d.est_xy ? `<div class="pos">candidate region ±${(+(d.est_sigma_m ?? 0)).toFixed(1)} m from RSSI${nDr ? ` (${nDr} drone${nDr > 1 ? 's' : ''})` : ''} · not a confirmed position</div>` : d.portal_only ? '<div class="pos">portal submission only: no position yet</div>' : ''}
      ${ev.map(e => `<div class="ev">new evidence in searched ${e.kind} ${esc(e.name)} — re-search?</div>`).join('')}
      ${ms.map(m => `<div class="msg${m.live ? ' new' : ''}"><div class="mh"><span class="t">t ${m.t != null ? Math.round(m.t) : '—'} s</span>${m.lang ? `<span class="lg">${esc(lname(m.lang))} · ${esc(m.lang)}</span>` : ''}<span class="by ${m.by === 'gemini' ? 'llm' : m.by === 'fallback' ? 'fb' : ''}" title="translation by ${esc(m.by || 'none')}">${esc(m.by === 'gemini' ? 'Gemini' : m.by || 'none')}</span>${m.live ? '<span class="nw">live</span>' : ''}</div>
          <q${m.lang ? ` lang="${esc(m.lang)}"` : ''}>${esc(m.text)}</q>${m.translation && m.translation !== m.text ? `<div class="tr"><span>EN</span>${esc(m.translation)}</div>` : ''}</div>`).join('')}
      ${reps.length ? `<div class="reps">${reps.map(r => `<div class="rep ${r.pending ? 'pend' : ''}"><span>t ${r.t != null ? Math.round(r.t) : '—'} s</span> ${r.template ? esc(TEMPLATES[r.template]?.label || r.template) : `“${esc(r.text)}”`}${r.lang ? ` <small>(${esc(r.lang)})</small>` : ''} · ${r.pending ? '<b>needs operator confirmation</b> <button class="btn sm" data-act="confirm" data-k="' + esc(r.k) + '">Confirm</button>' : r.src === 'state' ? `sent${r.confirmed_by ? ', confirmed by ' + esc(r.confirmed_by) : ''}` : 'recorded locally (not sent: no send endpoint yet)'}</div>`).join('')}</div>` : ''}
      <div class="tpl">${Object.entries(TEMPLATES).map(([k, t]) => `<button class="btn sm" data-act="tpl" data-t="${k}" title="${esc(t.en)}${lang ? ' (sent in ' + esc(lname(lang)) + ')' : ''}">${esc(t.label)}</button>`).join('')}
        <button class="btn sm ghost" data-act="free">Free text…</button></div>
      ${C.open.has(d.id) ? `<div class="free"><input aria-label="Free-text reply" maxlength="200" placeholder="Free text: held until an operator confirms" value="${esc(C.draft[d.id] || '')}"><button class="btn sm" data-act="hold">Hold for confirmation</button></div>` : ''}
    </div>`;
  }
  function record(dev, r) {
    const S = app.S(); const lang = [...msgsOf((S?.comms?.devices || []).find(d => d.id === dev) || { id: dev })].reverse().find(m => m.lang)?.lang || null;
    C.log.push({ k: Math.random().toString(36).slice(2, 9), device: dev, t: S?.t ?? null, lang, ...r }); lsSet(C.log); render();
  }
  let deferred = false;
  function bind() {
    $('comms').addEventListener('focusout', () => setTimeout(() => { if (deferred) render(); }, 0));
    $('comms').addEventListener('click', e => {
      const b = e.target.closest('[data-act]'); if (!b) return; const dev = b.closest('[data-dev]')?.dataset.dev;
      const a = b.dataset.act;
      if (a === 'tpl') record(dev, { template: b.dataset.t, confirmed_by: 'operator' });
      else if (a === 'free') { C.open.has(dev) ? C.open.delete(dev) : C.open.add(dev); render(true); }
      else if (a === 'hold') { const v = (b.parentElement.querySelector('input')?.value || '').trim(); if (!v) return; C.draft[dev] = ''; C.open.delete(dev); record(dev, { text: v, pending: true }); render(true); }
      else if (a === 'confirm') { const r = C.log.find(r => r.k === b.dataset.k); if (r) { r.pending = false; r.confirmed_by = 'operator'; lsSet(C.log); render(); } }
    });
    $('comms').addEventListener('input', e => { const dev = e.target.closest('[data-dev]')?.dataset.dev; if (dev) C.draft[dev] = e.target.value; });
    $('toast').addEventListener('click', () => { $('toast').hidden = true; $('secComms').scrollIntoView({ behavior: 'smooth', block: 'start' }); });
  }

  // ---------- live portal feed ----------
  function toast(n, s) {
    const el = $('toast'); el.innerHTML = `<b>New message from rescue Wi-Fi</b>${s ? `<span>${esc(s.device || 'phone')}${s.status ? ' · ' + esc(s.status) : ''}${s.translation || s.text ? ': “' + esc((s.translation || s.text).slice(0, 80)) + '”' : ''}</span>` : ''}${n > 1 ? `<small>+${n - 1} more</small>` : ''}`;
    el.hidden = false; clearTimeout(C.toastT); C.toastT = setTimeout(() => { el.hidden = true; }, 7000);
  }
  async function poll() {
    let next = POLL_MS;
    try {
      const r = await fetch('/api/portal', { cache: 'no-store' }); if (!r.ok) throw new Error(r.status);
      const doc = await r.json(), arr = Array.isArray(doc) ? doc : doc.submissions || doc.devices?.flatMap(d => (d.messages || []).map(m => ({ ...m, device: d.id, status: d.status }))) || [];
      const subs = arr.map((s, i) => ({ ...s, device: s.device ?? s.device_id ?? s.token ?? s.id, key: s.id ?? `${s.t}|${s.device ?? s.token}|${s.text ?? s.status}|${i}` }));
      const fresh = subs.filter(s => !C.seen.has(s.key)); subs.forEach(s => C.seen.add(s.key));
      C.liveSubs = subs; const was = C.live; C.live = true;
      if (!C.first && fresh.length) toast(fresh.length, fresh[fresh.length - 1]);
      C.first = false; if (fresh.length || !was) { render(); app.draw(); }
    } catch { if (C.live !== false) { C.live = false; render(); } next = RETRY_MS; }
    C.pollT = setTimeout(poll, next);
  }
  bind();
  return { draw, render, start() { if (!C.pollT) poll(); }, _C: C };
}
