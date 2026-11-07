// Loudspeaker (contracts/INTERFACES.md "Speaker (v1.3)", docs/SPEAKER.md): call-out, reassurance, guidance, operator
// messages. Every played string is a vetted template or text an operator typed; machine translations stay
// "pending_approval" until an operator approves them here. Nothing is auto-played or auto-broadcast by this page.
// Operator API (scripts/serve.py): GET/POST /api/broadcast, POST /api/broadcast/approve.

const API = '/api/broadcast', LS_OP = 'larpverse.operator.name', POLL_MS = 4000, RETRY_MS = 60000;
export const LANGS = [['zh-TW', 'Mandarin (Taiwan)'], ['nan-TW', 'Taiwanese Hokkien'], ['en', 'English'], ['id', 'Indonesian'], ['vi', 'Vietnamese']];
const DEFAULT_LANGS = ['zh-TW', 'nan-TW', 'en'];
const VETTED = {   // English of the fixed templates (sim/guidance.MESSAGES); other languages are rendered server-side
  callout: 'This is a rescue team. If you can hear me, knock three times or shout.',
  reassure: 'Help is coming. Stay still, save your energy, tap if you can hear me.',
};
const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const lsGet = k => { try { return localStorage.getItem(k) || ''; } catch { return ''; } };
const lsSet = (k, v) => { try { localStorage.setItem(k, v); } catch { /* private mode */ } };
const lname = c => (LANGS.find(l => l[0] === c) || [c, c])[1];

// Responsiveness chip (piles[] / radar_candidates[] .responsiveness). "No response" never clears anything.
const RESP = {
  responded: ['responded to call-out', 'r-yes', 'Responded to the loudspeaker call-out (a candidate, not a confirmed person)'],
  possible_response: ['possible response', 'r-maybe', 'A sound after the call-out that may be a response'],
  nonhuman_only: ['only non-human sounds after call-out', 'r-no', 'Only non-human sounds after the call-out: does not rule out an unconscious person'],
  no_response: ['no response — does not rule out unconscious', 'r-no', 'No response to the call-out. This does not rule out an unconscious person: not cleared.'],
};
export function respChip(r, short = false) {
  if (!r?.status) return '';
  const [lab, cls, tip] = RESP[r.status] || [String(r.status).replace(/_/g, ' '), 'r-maybe', ''];
  const kind = r.kind ? ` · ${String(r.kind).replace('_x3', ' ×3').replace(/_/g, ' ')}` : '';
  const extra = [r.text, r.range_m != null ? `call-out from ${Math.round(r.range_m)} m` : '', r.message_reached === false ? 'the message may not have reached this spot' : '', r.avpu ? `AVPU: ${r.avpu}` : ''].filter(Boolean).join(' · ');
  return `<span class="resp ${cls}" title="${esc(tip + (extra ? ' — ' + extra : ''))}">${esc(short && r.status === 'no_response' ? 'no response' : lab)}${r.status === 'responded' ? esc(kind) : ''}</span>`;
}

export function createSpeaker(app) {
  // app: { S(), toPx, scale(), ctx(), qLabel, font, $, draw(), FONT }
  const $ = app.$;
  const K = { q: null, server: null, msg: '', kind: '', pollT: null, playing: null, form: { kind: 'callout', target: '', langs: new Set(DEFAULT_LANGS), src: 'en', translate: true, text: '' } };

  // ---------- map ----------
  function arrowPath(ctx, P, color) {
    if (P.length < 2) return;
    ctx.save(); ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2; ctx.setLineDash([7, 5]);
    ctx.beginPath(); P.forEach((q, k) => { const [x, y] = app.toPx(q[0], q[1]); k ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke(); ctx.setLineDash([]);
    const [xa, ya] = app.toPx(...P[P.length - 2]), [xb, yb] = app.toPx(...P[P.length - 1]), an = Math.atan2(yb - ya, xb - xa);
    ctx.beginPath(); ctx.moveTo(xb, yb); ctx.lineTo(xb - 11 * Math.cos(an - 0.45), yb - 11 * Math.sin(an - 0.45)); ctx.lineTo(xb - 11 * Math.cos(an + 0.45), yb - 11 * Math.sin(an + 0.45)); ctx.closePath(); ctx.fill();
    ctx.restore();
  }
  function draw(ctx, layers) {
    const S = app.S(); if (!S || layers.speaker === false) return;
    const small = app.scale() < 6.5;
    for (const c of S.callouts || []) {
      for (const g of c.guided || []) {
        const P = Array.isArray(g.path) && g.path.length >= 2 ? g.path : null; if (!P) continue;
        arrowPath(ctx, P, 'rgba(240,244,248,0.9)');
        const [x, y] = app.toPx(...P[0]);
        if (!small) app.qLabel(`guided ${g.track} → rescue tent`, x + 10, y - 10, '#e3eaef', 'rgba(11,16,22,0.86)', 4, 'rgba(240,244,248,0.5)');
      }
      for (const t of c.tracks || []) {
        if (!t.xy || (c.guided || []).some(g => g.track === t.id || g.track?.endsWith('-' + t.id))) continue;
        const [x, y] = app.toPx(...t.xy); ctx.save(); ctx.strokeStyle = '#f0f4f8'; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(x, y, 6, 0, Math.PI * 2); ctx.stroke(); ctx.restore();
        if (!small) app.qLabel(`${t.id} · voice on open ground`, x + 9, y - 8, '#e3eaef', undefined, 6);
      }
    }
    for (const d of S.drones || []) {
      if (!d.speaker) continue;
      const [x, y] = app.toPx(d.pose[0], d.pose[1]);
      ctx.save(); ctx.translate(x - 15, y - 14); ctx.fillStyle = '#f0f4f8'; ctx.strokeStyle = '#f0f4f8'; ctx.lineWidth = 1.3;
      ctx.beginPath(); ctx.moveTo(0, 3); ctx.lineTo(3, 3); ctx.lineTo(7, 0); ctx.lineTo(7, 10); ctx.lineTo(3, 7); ctx.lineTo(0, 7); ctx.closePath(); ctx.fill();
      ctx.beginPath(); ctx.arc(8, 5, 4, -0.9, 0.9); ctx.stroke(); ctx.beginPath(); ctx.arc(8, 5, 7, -0.9, 0.9); ctx.stroke(); ctx.restore();
      if (!small) app.qLabel(`${d.id} · speaker ${d.speaker.kind || ''} ${d.speaker.lang || ''}`.trim(), x + 10, y - 24, '#f0f4f8', undefined, 3);
    }
  }

  // ---------- data ----------
  async function load() {
    let next = POLL_MS;
    try {
      const r = await fetch(API, { cache: 'no-store' }); if (!r.ok) throw new Error(r.status);
      K.q = await r.json(); K.server = true;
    } catch { K.server = false; next = RETRY_MS; }
    render(); K.pollT = setTimeout(load, next);
  }
  async function post(path, body) {
    try {
      const r = await fetch(path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
      const doc = await r.json().catch(() => ({}));
      if (!r.ok) { K.msg = `Server rejected it: ${doc.error || r.status}`; K.kind = 'err'; render(); return null; }
      K.server = true; return doc;
    } catch { K.server = false; K.msg = 'The speaker needs the ops server (python3 scripts/serve.py).'; K.kind = 'warn'; render(); return null; }
  }
  const items = () => {
    const S = app.S(), byId = new Map();
    for (const b of S?.broadcasts || []) byId.set(b.id, { ...b, from: 'state' });
    for (const b of K.q?.items || []) if (!byId.has(b.id) || b.status !== 'pending_approval') byId.set(b.id, { ...byId.get(b.id), ...b, from: byId.has(b.id) ? 'state' : 'queue' });
    return [...byId.values()];
  };

  // ---------- panel ----------
  function targets() {
    const S = app.S() || {};
    return [...(S.drones || []).map(d => [`d:${d.id}`, `drone ${d.id} (${d.state})`]), ...(S.zones || []).map(z => [`z:${z.id}`, `zone ${z.id} · ${z.name || ''}`])];
  }
  function render(force = false) {
    const S = app.S(), sec = $('secSpeaker'); if (!sec) return;
    const host = $('speaker'), ae = document.activeElement;
    if (host && host.contains(ae) && /INPUT|TEXTAREA|SELECT/.test(ae.tagName) && !force) { deferred = true; return; }
    deferred = false;
    const all = items(), pend = all.filter(b => b.status === 'pending_approval'), log = all.filter(b => b.status !== 'pending_approval')
      .sort((a, b) => (b.t ?? Infinity) - (a.t ?? Infinity));
    const co = (S?.callouts || []).slice(-1)[0];
    sec.hidden = !S || (!all.length && !(S.callouts || []).length && !(S.drones || []).length);
    if (sec.hidden) return;
    $('nSpk').textContent = pend.length ? `${pend.length} to approve` : `${log.length} played`;
    const F = K.form, tg = targets(); if (!F.target && tg.length) F.target = tg[0][0];
    const coTxt = co ? `<div class="co"><b>Call-out ${esc(co.id)}</b> by ${esc(co.speaker_drone)} at ${esc(co.target?.id || '')} · ${co.listen_ends != null && S.t < co.listen_ends ? `listening until t ${Math.round(co.listen_ends)} s` : 'results in'}${co.synthetic ? ' · <span class="syn">synthetic responses</span>' : ''}</div>` : '';
    const form = `<div class="cmp">
      <div class="r"><label>Target<select data-f="target">${tg.map(([v, l]) => `<option value="${esc(v)}"${v === F.target ? ' selected' : ''}>${esc(l)}</option>`).join('')}</select></label>
        <label>Message<select data-f="kind">${[['callout', 'Call-out (vetted)'], ['reassure', 'Reassure (vetted)'], ['operator', 'Free text']].map(([v, l]) => `<option value="${v}"${v === F.kind ? ' selected' : ''}>${l}</option>`).join('')}</select></label></div>
      <div class="langs" role="group" aria-label="Languages">${LANGS.map(([c, n]) => `<label class="lg${F.langs.has(c) ? ' on' : ''}" title="${esc(n)}"><input type="checkbox" data-lang="${c}"${F.langs.has(c) ? ' checked' : ''}>${c}</label>`).join('')}</div>
      ${F.kind === 'operator' ? `<textarea data-f="text" maxlength="300" rows="2" placeholder="Type the message (it is played under your name)">${esc(F.text)}</textarea>
        <div class="r"><label>Written in<select data-f="src">${LANGS.map(([c]) => `<option${c === F.src ? ' selected' : ''}>${c}</option>`).join('')}</select></label>
        <label class="ck"><input type="checkbox" data-f="translate"${F.translate ? ' checked' : ''}> machine-translate the rest</label></div>
        <p class="nc">Free text: your own words play as typed; every machine translation <b>needs confirmation</b> below before it can play.</p>`
        : `<p class="vt">Vetted template · <q>${esc(VETTED[F.kind])}</q> <small>(non-English lines pending native-speaker review)</small></p>`}
      <div class="r"><label>Operator<input data-f="op" value="${esc(lsGet(LS_OP))}" maxlength="40" placeholder="your name"></label>
        <button class="btn send" data-act="send">Queue for the speaker</button></div>
      ${K.msg ? `<p class="tmsg ${K.kind}">${esc(K.msg)}</p>` : K.server === false ? '<p class="tmsg warn">The speaker needs the ops server (python3 scripts/serve.py). Showing the mission\'s broadcast log only.</p>' : ''}
    </div>`;
    const pendH = pend.length ? `<div class="sub">Translations waiting for approval · never auto-played</div>` + pend.map(b => `<div class="pa" data-id="${esc(b.id)}">
        <div class="ph"><span class="lgc">${esc(b.lang)}</span><span class="mut">${esc(b.kind)} → ${esc(b.target?.type || '')} ${esc(b.target?.id || '')}${b.requested_by ? ' · by ' + esc(b.requested_by) : ''}</span><span class="by llm">machine translation</span></div>
        <div class="sbs"><q lang="${esc(b.lang)}">${esc(b.text)}</q><q lang="en">${esc(b.text_en)}</q></div>
        ${b.review ? `<small class="mut">${esc(b.review)}</small>` : ''}
        <div class="za"><button class="btn sm ok" data-act="approve">Approve</button><button class="btn sm" data-act="reject">Reject</button></div></div>`).join('') : '';
    const logH = log.length ? `<div class="sub">Broadcast log · newest first</div>` + log.slice(0, 40).map(b => {
      const has = !!b.audio_url, st = String(b.audio_status || '');
      return `<div class="bl s-${esc(b.status)}"><span class="t">${b.t != null ? 't ' + Math.round(b.t) + ' s' : 'queued'}</span>
        <span class="k">${esc(b.kind)}</span><span class="lgc">${esc(b.lang)}</span><span class="tg">${esc(b.drone_id || '')}${b.target?.id ? ' → ' + esc(b.target.id) : ''}</span>
        ${has ? `<button class="play" data-act="play" data-url="${esc(b.audio_url)}" aria-label="Play ${esc(b.lang)} audio" title="${esc(st)}">${K.playing === b.audio_url ? '■' : '▶'}</button>` : `<span class="noa" title="${esc(st || 'no audio')}">no audio</span>`}
        <span class="txt" title="${esc(b.text_en || '')}">${esc(b.text)}</span>
        <span class="meta">${esc(b.status)}${b.approved_by ? ' · ' + esc(b.approved_by) : ''}${b.source === 'llm_translation' ? ' · machine translation' : ''}${st && !/^rendered/.test(st) ? ' · ' + esc(st) : ''}</span></div>`;
    }).join('') : '<p class="empty">Nothing broadcast yet.</p>';
    $('speaker').innerHTML = coTxt + form + pendH + logH;
  }
  let deferred = false;
  function bind() {
    $('speaker').addEventListener('focusout', () => setTimeout(() => { if (deferred) render(); }, 0));
    const el = $('speaker');
    el.addEventListener('change', e => {
      const f = e.target.dataset.f, F = K.form;
      if (e.target.dataset.lang) { e.target.checked ? F.langs.add(e.target.dataset.lang) : F.langs.delete(e.target.dataset.lang); render(true); return; }
      if (f === 'target') F.target = e.target.value;
      if (f === 'kind') { F.kind = e.target.value; K.msg = ''; render(true); }
      if (f === 'src') F.src = e.target.value;
      if (f === 'translate') F.translate = e.target.checked;
      if (f === 'op') lsSet(LS_OP, e.target.value.trim());
    });
    el.addEventListener('input', e => { if (e.target.dataset.f === 'text') K.form.text = e.target.value; if (e.target.dataset.f === 'op') lsSet(LS_OP, e.target.value.trim()); });
    el.addEventListener('click', async e => {
      const b = e.target.closest('[data-act]'); if (!b) return; const a = b.dataset.act, op = lsGet(LS_OP), F = K.form;
      if (a === 'play') {
        const url = b.dataset.url;
        if (K.audio) { K.audio.pause(); if (K.playing === url) { K.playing = null; render(); return; } }
        K.audio = new Audio('/' + url.replace(/^\/+/, '')); K.playing = url;
        K.audio.onended = () => { K.playing = null; render(); };
        K.audio.onerror = () => { K.playing = null; K.msg = `Audio file not found (${url}): render it with acoustic.speaker_assets.`; K.kind = 'warn'; render(); };
        K.audio.play().catch(() => {}); render(); return;
      }
      if (!op) { K.msg = 'Enter your name: every broadcast and approval is attributed.'; K.kind = 'err'; render(true); return; }
      if (a === 'send') {
        if (!F.langs.size) { K.msg = 'Pick at least one language.'; K.kind = 'err'; render(); return; }
        if (F.kind === 'operator' && !F.text.trim()) { K.msg = 'Type a message first.'; K.kind = 'err'; render(); return; }
        const [tt, id] = [F.target.slice(0, 1), F.target.slice(2)];
        const body = { kind: F.kind, langs: LANGS.map(l => l[0]).filter(l => F.langs.has(l)), operator: op, ...(tt === 'z' ? { zone: id } : { drone_id: id }) };
        if (F.kind === 'operator') Object.assign(body, { text: F.text.trim(), src_lang: F.src, translate: F.translate });
        const out = await post(API, body); if (!out) return;
        const n = out.items?.length || 0, pend = (out.items || []).filter(i => i.status === 'pending_approval').length, probs = Object.entries(out.translation_problems || {});
        K.msg = `Queued ${n} item${n === 1 ? '' : 's'}${pend ? `; ${pend} machine translation${pend > 1 ? 's' : ''} need${pend > 1 ? '' : 's'} approval below` : ''}.` + (probs.length ? ' Translation problems: ' + probs.map(([l, p]) => `${l}: ${[].concat(p).join(', ')}`).join('; ') : '');
        K.kind = probs.length ? 'warn' : 'ok'; if (F.kind === 'operator') F.text = ''; render(true);
        clearTimeout(K.pollT); load();
      }
      if (a === 'approve' || a === 'reject') {
        const id = b.closest('[data-id]').dataset.id, out = await post(API + '/approve', { id, operator: op, approve: a === 'approve' }); if (!out) return;
        K.msg = `${id} ${a === 'approve' ? 'approved: the mission plays it at its next step' : 'rejected: it will not play'}.`; K.kind = 'ok';
        clearTimeout(K.pollT); load();
      }
    });
  }
  bind();
  return { draw, render, start() { if (!K.pollT) load(); }, _K: K };
}
