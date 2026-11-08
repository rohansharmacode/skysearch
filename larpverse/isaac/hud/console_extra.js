// console_extra.js: additive panels for the live operator console (isaac/hud/console.html), dashboard agent 2026-11-08.
// A floating stack on the right of the Isaac view: PRIORITY LIST (pings[].priority, Gemini), the latest clip with its
// Gemini analysis, SUGGESTED NEXT AREAS from acoustics (a click reuses the console's own gotoPile -> POST /api/goto),
// and a compact Rescue Wi-Fi & speaker strip. Reads only window._last (the state the console already polls) and the
// console's helpers; every panel hides itself when its fields are missing, so the console never breaks.
(function () {
  'use strict';
  const view = document.getElementById('view');
  if (!view) return;
  const css = document.createElement('style');
  css.textContent = `
  #xstack { right:24px; top:64px; width:380px; max-height:920px; display:flex; flex-direction:column; gap:10px;
            background:transparent; box-shadow:none; backdrop-filter:none; }
  #xstack > .xc { background:rgba(10,14,18,0.60); border-radius:16px; backdrop-filter:blur(6px); box-shadow:0 6px 28px rgba(0,0,0,0.35);
            padding:12px 16px; border:1px solid rgba(255,255,255,0.08); }
  #xstack .xh { font-size:12px; letter-spacing:.16em; text-transform:uppercase; color:#b8c3cc; display:flex; justify-content:space-between; gap:8px; align-items:baseline; }
  #xstack .xh i { font-style:normal; letter-spacing:0; text-transform:none; color:#8d98a1; font-size:12px; }
  .xp { display:grid; grid-template-columns:30px 1fr auto; gap:2px 10px; padding:7px 0; border-top:1px solid rgba(255,255,255,0.10); }
  .xp:first-of-type { border-top:0; }
  .xp .r { grid-column:1; grid-row:1 / span 3; font-size:24px; font-weight:800; color:#fff; line-height:1; padding-top:2px; }
  .xp .t { grid-column:2; grid-row:1; font-size:17px; font-weight:700; color:#fff; }
  .xp .t small { font-size:13px; font-weight:600; color:#ff7b7b; margin-left:6px; }
  .xp .s { grid-column:3; grid-row:1 / span 2; text-align:right; font-size:24px; font-weight:800; color:#ffb020; line-height:1; }
  .xp .s small { display:block; font-size:11px; font-weight:600; color:#8d98a1; letter-spacing:.06em; margin-top:3px; }
  .xp .b { grid-column:2; grid-row:2; font-size:15px; color:#ffd0d0; }
  .xp .w { grid-column:2 / span 2; grid-row:3; font-size:14px; color:#d5dde3; line-height:1.35; display:-webkit-box; -webkit-line-clamp:2; -webkit-box-orient:vertical; overflow:hidden; }
  .xp .w b { color:#8ab4ff; font-weight:700; }
  .xnote { font-size:12.5px; color:#9aa5ae; margin-top:6px; font-style:italic; }
  .xq { font-size:17px; font-style:italic; color:#e9f6f8; margin-top:8px; line-height:1.35; }
  .xq small { display:block; font-style:normal; color:#9aa5ae; font-size:13px; margin-top:3px; }
  .xtags { display:flex; flex-wrap:wrap; gap:6px; margin-top:7px; }
  .xtag { font-size:12px; font-weight:800; letter-spacing:.06em; padding:3px 8px; border-radius:6px; background:#ffb020; color:#0b0f13; text-transform:uppercase; }
  .xtag.l { background:#2fc6d9; }
  .xs { display:flex; align-items:center; gap:10px; padding:7px 0; border-top:1px solid rgba(255,255,255,0.10); }
  .xs:first-of-type { border-top:0; }
  .xs .txt { flex:1; font-size:14px; color:#d5dde3; line-height:1.35; }
  .xs .txt b { color:#fff; font-size:15px; }
  .xs button { font:700 14px Inter, "Segoe UI", system-ui, sans-serif; padding:8px 12px; border-radius:9px; border:1px solid #ffb020; background:#2a2f22;
               color:#fff; cursor:pointer; white-space:nowrap; }
  .xs button:hover { background:#3a4128; }
  .xw { display:flex; flex-wrap:wrap; gap:4px 14px; font-size:14px; color:#d5dde3; margin-top:6px; line-height:1.35; }
  .xw b { color:#8ab4ff; } .xw .tr { color:#ff9b9b; font-weight:700; }
  #xtog { position:absolute; right:24px; top:24px; z-index:5; font:700 13px Inter, "Segoe UI", system-ui, sans-serif; padding:6px 11px;
          border-radius:8px; border:1px solid #39424a; background:rgba(10,14,18,0.72); color:#cfd8de; cursor:pointer; user-select:none; }
  #xtog:hover { border-color:#ffb020; color:#fff; }
  #xstack.off { display:none !important; }`;
  document.head.appendChild(css);
  const box = document.createElement('div');
  box.className = 'hud'; box.id = 'xstack';
  const honest = document.getElementById('honest');
  view.insertBefore(box, honest || null);
  // hide / show toggle (button + key P) so the live camera view can be cleared during a take; default shown
  const tog = document.createElement('div'); tog.id = 'xtog'; tog.title = 'Hide or show the extra panels (key P)';
  view.insertBefore(tog, honest || null);
  let shownX = true;
  const setShown = v => { shownX = v; box.classList.toggle('off', !v); tog.textContent = v ? 'Hide panels (P)' : 'Show panels (P)'; };
  tog.addEventListener('click', () => setShown(!shownX));
  addEventListener('keydown', e => { if ((e.key === 'p' || e.key === 'P') && !/INPUT|TEXTAREA|SELECT/.test((document.activeElement || {}).tagName || '') && !e.ctrlKey && !e.metaKey && !e.altKey) setShown(!shownX); });
  setShown(true);

  const E = x => (typeof esc === 'function' ? esc(x) : String(x ?? '').replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c])));
  const name = id => (typeof pn === 'function' ? pn(id) : String(id ?? ''));
  const L = (typeof LANG === 'object' && LANG) || {};
  const tag = by => (typeof byTag === 'function' ? byTag(by) : '');
  const BREATH = { distressed_tachypnea: 'shallow, rapid', rapid_shallow: 'shallow, rapid', unconscious_slow: 'deep, slow', slow_regular: 'deep, slow',
                   irregular_apneic: 'irregular with pauses', irregular_with_pauses: 'irregular with pauses', calm: 'calm, regular', regular_calm: 'calm, regular' };

  // ---- 1. priority list
  function priority(s) {
    const cands = s.radar_candidates || [], byId = new Map(cands.map(c => [c.id, c]));
    const P = (s.pings && s.pings.length ? s.pings : cands.filter(c => c.status === 'candidate'))
      .map(p => ({ ...(byId.get(p.id) || {}), ...p, vitals: { ...((byId.get(p.id) || {}).vitals || {}), ...(p.vitals || {}) } }))
      .filter(p => String((p.vitals || {}).triage_hint || '') !== 'reject');
    if (!P.length) return '';
    const val = p => (Number.isFinite(+p.priority) && p.priority !== null ? +p.priority : null);
    const hasP = P.some(p => val(p) != null);
    P.sort((a, b) => hasP ? (val(b) ?? -1) - (val(a) ?? -1) : (+b.dig_score || 0) - (+a.dig_score || 0));
    const rows = P.slice(0, 4).map((p, k) => {
      const v = p.vitals || {}, g = v.gemini || {};
      const words = g.pattern || g.breathing || v.breathing_pattern || BREATH[v.condition] || BREATH[v.pattern] || '';
      const rate = v.rate_bpm ? `${Math.round(v.rate_bpm)}/min` : (p.breathing_hz ? `${Math.round(60 * p.breathing_hz)}/min` : '');
      const by = p.priority_by || g.by || null;
      const score = val(p) != null ? `${Math.round(val(p))}<small>PRIORITY</small>` : (p.dig_score != null ? `${(+p.dig_score).toFixed(2)}<small>DIG SCORE</small>` : '');
      const why = g.reason || v.condition_text || '';
      return `<div class="xp"><div class="r">${k + 1}</div><div class="t">${E(p.id)}<small>${E(name(p.pile_id || ''))}</small> ${by ? tag(by === 'gemini' ? 'gemini' : 'box') : ''}</div><div class="s">${score}</div>
        ${words || rate ? `<div class="b">breathing ${E([words, rate].filter(Boolean).join(', '))}</div>` : ''}
        ${why ? `<div class="w">${g.reason ? (by === 'gemini' ? '<b>Gemini:</b> ' : '<b>Why:</b> ') : ''}${E(String(why).replace(/\s*->\s*/g, ' — '))}</div>` : ''}</div>`;
    }).join('');
    return `<div class="xc"><div class="xh">Priority list · dig first <i>${hasP ? 'Gemini priority 0–100' : 'dig score'}</i></div>${rows}
      <div class="xnote">Aid for rescuers, not a diagnosis. Each is a place to check, not a confirmed person.</div></div>`;
  }

  // ---- 2. latest clip with its Gemini analysis
  function clip(s) {
    const C = s.acoustic_clips || []; if (!C.length) return '';
    const n = Math.max(...C.map(c => +c.listen || 0));
    const c = C.filter(x => (+x.listen || 0) === n && (((x.scores || {}).human ?? x.human ?? 0) >= 0.4))
      .sort((a, b) => (b.priority ?? 0) - (a.priority ?? 0))[0];
    if (!c) return '';
    const aa = c.audio_analysis || {}, by = aa.by || c.by || null;
    const lg = (c.languages && c.languages[0] && c.languages[0].code) || c.language || c.lang || null;
    const lname = lg ? (L[lg] || (c.languages && c.languages[0] && c.languages[0].name) || lg) : null;
    const orig = c.transcript_original || c.transcript || '';
    const tr = c.translation || (lg && /^en\b/.test(lg) ? orig : '');
    const theirs = typeof heard === 'function' ? (() => { try { return heard(c); } catch (e) { return null; } })() : null;
    const said = theirs ? theirs : lg && (tr || orig)
      ? `“${E(tr || orig)}”${orig && tr && orig !== tr ? `<small>spoken in ${E(lname)}: “${E(orig)}”</small>` : `<small>${E(lname)}</small>`}`
      : orig ? `“${E(orig)}”<small>language not identified</small>`
      : `${E(String(c.class || 'sound').replace(/_/g, ' ') === 'voice' ? 'voice-like call' : String(c.class || 'sound').replace(/_/g, ' '))} (language not identified)`;
    const dist = (c.distress || []).map(d => `<span class="xtag">${E(String(d).replace(/_/g, ' '))}</span>`).join('');
    return `<div class="xc"><div class="xh">Latest sound · ${E(name(c.pile_id))} ${tag(by === 'gemini' ? 'gemini' : by ? 'box' : null)}<i>listen ${n}</i></div>
      <div class="xq">${said}</div>${dist || lname ? `<div class="xtags">${lname ? `<span class="xtag l">${E(lname)}</span>` : ''}${dist}</div>` : ''}</div>`;
  }

  // ---- 3. suggested next areas (acoustics): a click = the console's own pile click (gotoPile -> POST /api/goto)
  function nearestPile(s, xy) {
    if (!xy) return null; let best = null, bd = 1e9;
    for (const p of s.piles || []) { if (!p.xy) continue; const d = Math.hypot(p.xy[0] - xy[0], p.xy[1] - xy[1]); if (d < bd) { bd = d; best = p.id; } }
    return bd <= 25 ? best : null;
  }
  const centroid = P => [P.reduce((a, q) => a + q[0], 0) / P.length, P.reduce((a, q) => a + q[1], 0) / P.length];
  function suggestions(s) {
    const visited = new Set(((s.mission || {}).visited) || []);
    const out = [];
    for (const g of s.zone_suggestions || []) out.push({ id: g.id, pile: g.pile_id || g.pile || nearestPile(s, g.centre), why: g.why });
    for (const p of [...(s.zone_proposals || [])].sort((a, b) => (b.score ?? 0) - (a.score ?? 0)))
      out.push({ id: p.id, pile: p.pile_id || (p.polygon && p.polygon.length >= 3 ? nearestPile(s, centroid(p.polygon)) : null), why: p.why });
    const rows = out.filter(o => o.pile && !visited.has(o.pile)).slice(0, 3);
    if (!rows.length) return '';
    return `<div class="xc"><div class="xh">Suggested next areas · from acoustics</div>${rows.map(o => `<div class="xs"><div class="txt"><b>${E(name(o.pile))}</b> ${E(o.why || '')}</div>
      <button data-goto="${E(o.pile)}" title="Same as clicking the pile on the map">Send drones →</button></div>`).join('')}</div>`;
  }

  // ---- 4. rescue Wi-Fi & speaker strip
  function strip(s) {
    const D = (s.comms && s.comms.devices) || [], B = (s.broadcasts || []).filter(b => b.status === 'played' || b.status === 'scheduled');
    if (!D.length && !B.length) return '';
    const tr = D.filter(d => d.status === 'trapped').length, last = B[B.length - 1];
    return `<div class="xc"><div class="xh">Rescue Wi-Fi &amp; speaker</div><div class="xw">
      ${D.length ? `<span><b>${D.length}</b> phone${D.length > 1 ? 's' : ''} connected${tr ? ` · <span class="tr">${tr} say trapped</span>` : ''}</span>` : ''}
      ${last ? `<span>speaker: ${E(last.kind || '')} ${E(last.lang || '')}${last.text_en ? ` “${E(String(last.text_en).slice(0, 70))}${String(last.text_en).length > 70 ? '…' : ''}”` : ''}</span>` : ''}</div></div>`;
  }

  let key = '';
  function tick() {
    const s = window._last; if (!s) return;
    let html = '';
    try { html = priority(s) + clip(s) + suggestions(s) + strip(s); } catch (e) { html = ''; }
    if (html !== key) { key = html; box.innerHTML = html; box.classList.toggle('hide', !html); }
  }
  box.addEventListener('click', e => {
    const b = e.target.closest('[data-goto]'); if (!b) return;
    if (typeof gotoPile === 'function') gotoPile(b.dataset.goto);
  });
  if (!window.__burn) setInterval(tick, 500); else window.__xtick = tick;
  tick();
})();
