// Live take files next to state.json (docs/VIDEO_PLAN.md, Request D1), written every frame / step by isaac/live.py
// and isaac/hud.py while --live runs:
//   live.json  larpverse.live/v1: last_event (stage event), replans, quick_listens, last_listen, last_scan
//   hud.json   larpverse.hud/v1:  panels.radar {wave, range_db, verdict, elapsed_s, source, text}, panels.listen, ...
// Shows the LIVE event line, the radar breathing waveform ("analysing breathing… n s" until the 45 s verdict gate,
// then the condition) and the top clip. Missing files: probed once (in ?layout=take re-probed every 10 s), no noise.
import { conditionLabel } from './vitals.js';

const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const AID = 'aid for rescuers, not a diagnosis';

export function createLive(app) {
  // app: { S(), DATA, $, take, onChange() }
  const $ = app.$, L = { live: null, hud: null, has: { live: null, hud: null }, rawL: '', rawH: '' };
  const HUD_MS = app.take ? 250 : 1000, LIVE_MS = 1000, RETRY_MS = 10000;

  async function get(name) {
    try {
      const r = await fetch(`${app.DATA}/${name}?_=${Date.now()}`, { cache: 'no-store' });
      if (!r.ok) throw new Error(r.status);
      return await r.text();
    } catch { return null; }
  }
  function loop(name, key, ms) {
    const tick = async () => {
      const raw = await get(name);
      if (raw == null) { L.has[key] = false; render(); if (app.take) setTimeout(tick, RETRY_MS); return; }   // absent: stop (take: re-probe)
      L.has[key] = true;
      const rk = key === 'live' ? 'rawL' : 'rawH';
      if (raw !== L[rk]) { L[rk] = raw; try { L[key] = JSON.parse(raw); } catch { /* half-written: next tick */ } render(); }
      setTimeout(tick, ms);
    };
    tick();
  }

  // ---------- event line ----------
  function droneRange(S) {
    const ids = (S?.drones || []).map(d => d.id); if (!ids.length) return 'the drones';
    const n = ids.map(i => +i.replace(/\D/g, ''));
    return ids.length > 2 && n.every((v, k) => k === 0 || v === n[k - 1] + 1) ? `${ids[0]}–${ids[ids.length - 1]}` : ids.join(', ');
  }
  function clipKind(S, pile) {
    const cs = (S?.acoustic_clips || []).filter(c => c.pile_id === pile && (c.scores?.human ?? 0) >= 0.4);
    const c = cs.sort((a, b) => (b.t ?? 0) - (a.t ?? 0) || (b.priority ?? 0) - (a.priority ?? 0))[0];
    return c ? (c.class === 'knocking' ? 'knocking' : c.class === 'shout' ? 'shout' : 'voice') : 'sound';
  }
  // newest of: stage event, quick listen with a new lead, re-plan. A re-plan right after a new lead reads as one line.
  function eventLine(S, lj) {
    if (!lj) return null;
    const out = [];
    const ev = lj.last_event;
    if (ev) out.push({ t: ev.t, kind: 'stage', text: `stage: dummy ${ev.id} ${ev.what} near ${ev.sector || '—'} (${ev.cause || 'operator'}) — the drones must hear or scan it to know` });
    const ql = [...(lj.quick_listens || [])].reverse().find(q => (q.new || []).length);
    const rp = (lj.replans || []).slice(-1)[0];
    if (ql) {
      const top = [...ql.new].sort((a, b) => (b.priority ?? 0) - (a.priority ?? 0))[0];
      let text = `new ${clipKind(S, top.pile)} heard near ${top.pile}${top.priority != null ? ` (priority ${top.priority})` : ''}`;
      if (rp && rp.t >= ql.t && rp.t - ql.t <= 10 && (rp.to == null || rp.to === top.pile)) text += ` → redirecting ${droneRange(S)}${rp.to ? ' to ' + rp.to : ''}`;
      out.push({ t: ql.t, kind: 'listen', text });
    }
    if (rp && !(ql && rp.t >= ql.t && rp.t - ql.t <= 10)) out.push({ t: rp.t, kind: 'replan', text: rp.to ? `re-plan: ${rp.why} → redirecting ${droneRange(S)} to ${rp.to}` : `re-plan: ${rp.why}` });
    return out.sort((a, b) => (b.t ?? 0) - (a.t ?? 0))[0] || null;
  }

  // ---------- radar waveform ----------
  function waveSVG(w, W = 300, H = 64) {
    const y = (w || []).filter(Number.isFinite); if (y.length < 2) return '';
    let lo = Math.min(...y), hi = Math.max(...y); const pad = Math.max(1e-6, (hi - lo) * 0.1); lo -= pad; hi += pad;
    const d = y.map((v, k) => `${k ? 'L' : 'M'}${(k / (y.length - 1) * W).toFixed(1)} ${(H - (v - lo) / (hi - lo) * H).toFixed(1)}`).join('');
    return `<svg viewBox="0 0 ${W} ${H}" preserveAspectRatio="none" class="hw" role="img" aria-label="radar breathing waveform, last 10 s"><path d="${d}" fill="none" stroke="#ff8a8a" stroke-width="1.6" vector-effect="non-scaling-stroke"/></svg>`;
  }
  function rangeStrip(db) {
    const y = (db || []).filter(Number.isFinite); if (!y.length) return '';
    const lo = Math.min(...y), hi = Math.max(...y), k = Math.max(0, y.indexOf(hi));
    return `<div class="rb" title="range-bin intensity (dB)">${y.map((v, i) => `<i style="opacity:${(0.15 + 0.85 * (v - lo) / Math.max(1e-6, hi - lo)).toFixed(2)}"${i === k ? ' class="pk"' : ''}></i>`).join('')}</div>`;
  }
  function verdictLine(rd) {
    if (!rd) return '';
    const v = rd.verdict;
    if (!v) return rd.elapsed_s != null ? `<b class="an">analysing breathing… ${Math.round(rd.elapsed_s)} s</b> <small>verdict after 45 s of dwell (early reads flip)</small>` : '';
    if (v.source && v.source !== 'human_breathing') return `<b class="nh">${esc(String(v.source).replace(/_/g, ' '))}</b> <small>${esc(v.text || '')} · not dug for</small>`;
    const lab = conditionLabel({ condition: v.condition, condition_text: v.condition_text, rate_bpm: v.rate_bpm }, {}) || v.text || '';
    return `<b>${esc(lab)}</b>${v.conf != null ? ` <small>confidence ${(+v.conf).toFixed(2)}</small>` : ''}`;
  }

  // ---------- render ----------
  function render() {
    const S = app.S();
    // event line
    const e = eventLine(S, L.live), bar = $('liveBar');
    bar.hidden = !e;
    if (e) bar.innerHTML = `<b>LIVE</b><span class="lt">t ${Math.round(e.t ?? 0)} s</span><span class="lx k-${e.kind}">${esc(e.text)}</span>`;
    // radar panel
    const rd = L.hud?.panels?.radar, sec = $('secHud');
    sec.hidden = !rd;
    if (rd) {
      const head = String(rd.text || '').split('\n')[0];
      $('hud').innerHTML = `<div class="hh">${esc(head)}${rd.source === 'library' || rd.source === 'reference' ? ' <span class="ref">reference signature (no live scan)</span>' : ''}</div>
        ${waveSVG(rd.wave)}${rangeStrip(rd.range_db)}
        <div class="vl">${verdictLine(rd)}</div><div class="aid">${AID} · candidate region, not a confirmed person</div>`;
    }
    // top clip (take layout): from the HUD listen panel, else the latest listen in state
    const tc = $('secTopClip');
    if (tc) {
      const top = topClip(S, L.hud);
      tc.hidden = !top;
      if (top) $('topClip').innerHTML = `<div class="th"><b>${esc(top.pile || '—')}</b> <span class="cls">${esc(top.class || '')}</span>${top.lang ? ` <span class="lg">${esc(top.lang)}</span>` : ''}${top.human != null ? ` <span class="mut">human ${(+top.human).toFixed(2)}</span>` : ''}${top.priority != null ? ` <span class="pr">priority ${top.priority}</span>` : ''}${top.by ? ` <span class="mut">· ${esc(top.by)}</span>` : ''}</div>
        ${top.text ? `<q${top.lang ? ` lang="${esc(top.lang)}"` : ''}>${esc(top.text)}</q>` : ''}${top.en && top.en !== top.text ? `<div class="tr"><span>EN</span>${esc(top.en)}</div>` : ''}`;
    }
    app.onChange?.();
  }
  function topClip(S, hud) {
    const clips = S?.acoustic_clips || [];
    if (clips.length) {
      const n = Math.max(...clips.map(c => c.listen ?? 0)), last = clips.filter(c => (c.listen ?? 0) === n && (c.scores?.human ?? 0) >= 0.4);
      const c = last.sort((a, b) => (b.priority ?? 0) - (a.priority ?? 0))[0];
      if (c) return { pile: c.pile_id, class: c.class, lang: c.language || c.lang || null, human: c.scores?.human, priority: c.priority,
        text: c.transcript_original || c.transcript || '', en: c.translation || '', by: c.audio_analysis?.by === 'gemini' ? 'Gemini' : c.audio_analysis?.by || c.by };
    }
    const t = hud?.panels?.listen?.top;
    return t ? { pile: t.pile, class: t.class, lang: t.lang !== '-' ? t.lang : null, human: t.human, priority: t.priority, text: t.transcript || '' } : null;
  }
  return { start() { loop('live.json', 'live', LIVE_MS); loop('hud.json', 'hud', HUD_MS); }, render, _L: L };
}
