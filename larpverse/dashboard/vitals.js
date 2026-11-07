// Dig order + breathing-pattern helpers for radar pings (radar_candidates[].vitals, written by sim/vitals.py).
// Everything here is an AID for rescuers ordering the dig, never a diagnosis: radar sees chest-wall motion only.
// Every field is optional; a ping without vitals ranks as "assess" from localisation and depth alone.

const clamp = (v, a = 0, b = 1) => Math.max(a, Math.min(b, v));
const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

// Triage chips. rank = dig-order weight (sim/vitals.py TRIAGE_RANK); "assess" sits mid-table on purpose.
export const TRIAGE = {
  immediate: { key: 'immediate', label: 'immediate', rank: 1.0, tip: 'START respiratory criterion: breathing over 30/min. Dig first, confirm on site.' },
  urgent: { key: 'urgent', label: 'urgent', rank: 0.85, tip: 'Irregular, paused, slow or fast-irregular breathing: possible airway problem or reduced responsiveness. Check soon.' },
  assess: { key: 'assess', label: 'assess', rank: 0.55, tip: 'No usable breathing pattern from this scan: needs a longer or better scan.' },
  delayed: { key: 'delayed', label: 'delayed', rank: 0.35, tip: 'Regular breathing in the normal range (respiration only: perfusion and responsiveness are unknown).' },
};
export function triageOf(v) {
  const h = String(v?.triage_hint || '').toLowerCase().trim();
  if (!h || h === 'assess') return { ...TRIAGE.assess, sub: v ? '' : 'no pattern yet' };
  if (h === 'urgent' || /^immediate\s*\(check\)/.test(h)) return { ...TRIAGE.urgent, sub: 'check' };
  if (h.startsWith('immediate')) return { ...TRIAGE.immediate, sub: '' };
  if (/^delayed\s*\(re-?check\)/.test(h)) return { ...TRIAGE.delayed, rank: 0.45, sub: 're-check' };
  if (h.startsWith('delayed') || h === 'minor') return { ...TRIAGE.delayed, sub: '' };
  return { ...TRIAGE.assess, sub: '' };
}
export const PATTERN = {
  rapid_shallow: 'rapid, shallow — possible distress',
  irregular_with_pauses: 'irregular with pauses',
  regular_calm: 'calm, regular',
  fast_small: 'fast, small — possible child (low confidence)',
  slow_deep: 'slow — possible reduced responsiveness',
  unclear: 'pattern unclear — rescan',
};
export const patternLabel = v => (v?.pattern ? PATTERN[v.pattern] || String(v.pattern).replace(/_/g, ' ') : null);
export const rateOf = (v, c) => v?.rate_bpm ?? (c?.breathing_hz ? c.breathing_hz * 60 : null);

// Dig-order score = triage weight (shrunk towards "assess" by pattern_conf, like sim/vitals.priority_weight)
//                   x localisation certainty (firm fix, belief share, 95% region size) x accessibility (depth).
export function digScore(c, depth, ell) {
  const v = c.vitals || null, t = triageOf(v), conf = clamp(+(v?.pattern_conf ?? 0));
  const tri = v?.priority_weight ?? (TRIAGE.assess.rank + (t.rank - TRIAGE.assess.rank) * conf);
  const weak = c.status !== 'candidate' || c.kind === 'moving';
  let loc = weak ? 0.45 : 0.65 + 0.35 * clamp((c.p_mass ?? 0.5) / 0.8);
  if (ell) loc *= clamp(2.5 / Math.max(2.5, 2 * ell.rx), 0.6, 1);
  const acc = !depth || depth.nominal ? 0.8 : clamp(1.1 - 0.15 * depth.d, 0.5, 1);
  return { score: tri * loc * acc, tri, loc, acc, conf, triage: t, weak };
}

// ---- sparkline: the radar waveform if the mission ships one, else a pattern ILLUSTRATION synthesised from
// rate / variability / pauses / excursion (labelled as such wherever it is drawn).
function rng(seed) { let a = 0; for (const ch of String(seed)) a = (a * 31 + ch.charCodeAt(0)) | 0; return () => { a = (a + 0x6D2B79F5) | 0; let t = Math.imul(a ^ (a >>> 15), 1 | a); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }
function measuredWave(v) {
  const w = v?.waveform ?? v?.wave ?? null;
  const y = Array.isArray(w) ? w : Array.isArray(w?.y) ? w.y : Array.isArray(w?.displacement) ? w.displacement : null;
  return y && y.length >= 8 && y.every(Number.isFinite) ? y : null;
}
function synthWave(v, c, n, win) {
  const rate = rateOf(v, c); if (!rate) return null;
  const r = rng(c.id || 'x'), g = () => (r() + r() + r() - 1.5) * 1.15;
  const pat = v?.pattern, cv = clamp(+(v?.cv ?? (pat === 'irregular_with_pauses' ? 0.25 : 0.05)), 0, 0.6);
  let amp = v?.depth_rel != null ? clamp(+v.depth_rel, 0.3, 1.3) : pat === 'rapid_shallow' ? 0.6 : pat === 'fast_small' ? 0.5 : pat === 'slow_deep' ? 1.15 : 0.9;
  let nPause = Math.round((+(v?.apnea_events_per_min ?? 0)) * win / 60);
  if (!nPause && (+(v?.apnea_events_per_min ?? 0) > 0 || (pat === 'irregular_with_pauses' && v?.apnea_events_per_min == null))) nPause = 1;
  nPause = Math.min(nPause, 2);
  const pauses = Array.from({ length: nPause }, (_, k) => win * (k + 0.55) / (nPause + 0.3));
  const breaths = []; let t = 0, pi = 0;
  while (t < win) {
    if (pi < pauses.length && t >= pauses[pi]) { const L = 11 + 3 * r(); breaths.push({ t0: t, T: L, a: 0 }); t += L; pi++; continue; }
    const T = clamp((60 / rate) * (1 + cv * g()), 0.35 * 60 / rate, 2.5 * 60 / rate);
    breaths.push({ t0: t, T, a: amp * (1 + 0.12 * g()) }); t += T;
  }
  const mv = clamp(+(v?.movement ?? 0)), mvAt = win * (0.2 + 0.5 * r()), y = new Array(n);
  for (let k = 0, b = 0; k < n; k++) {
    const tt = k / (n - 1) * win; while (b < breaths.length - 1 && tt >= breaths[b].t0 + breaths[b].T) b++;
    const B = breaths[b], u = (tt - B.t0) / B.T;
    let val = u < 0.4 ? B.a * (1 - Math.cos(Math.PI * u / 0.4)) / 2 : B.a * (1 + Math.cos(Math.PI * (u - 0.4) / 0.6)) / 2;
    if (mv > 0.15 && Math.abs(tt - mvAt) < win * mv * 0.25) val += 0.35 * Math.sin(tt * 9.1) * (0.5 + r());
    y[k] = val + 0.02 * g();
  }
  return y;
}
// Returns { svg, measured, label, tip } or null when there is no rate at all.
export function spark(c, { w = 96, h = 24, win = 40, stroke = 'currentColor', cls = 'spk' } = {}) {
  const v = c.vitals || null, meas = measuredWave(v), n = Math.max(40, Math.round(w * 1.5));
  let y = meas ? meas.slice(-Math.max(n, 200)) : synthWave(v, c, n, win);
  if (!y) return null;
  let lo = Infinity, hi = -Infinity; for (const q of y) { lo = Math.min(lo, q); hi = Math.max(hi, q); }
  if (!meas) { lo = Math.min(lo, -0.05); hi = Math.max(hi, 1.35); }
  const sx = (w - 2) / (y.length - 1), sy = (h - 4) / Math.max(1e-9, hi - lo);
  const d = y.map((q, k) => `${k ? 'L' : 'M'}${(1 + k * sx).toFixed(1)} ${(h - 2 - (q - lo) * sy).toFixed(1)}`).join('');
  const rate = rateOf(v, c);
  const tip = meas ? 'Radar breathing waveform (displacement at the ping range)'
    : `Pattern illustration, synthesised from ${rate ? Math.round(rate) + '/min' : 'the rate'}` + (v?.cv != null ? `, variability ${(+v.cv).toFixed(2)}` : '')
      + (v?.apnea_events_per_min ? `, ${(+v.apnea_events_per_min).toFixed(1)} pauses/min` : '') + ': not the measured waveform';
  const rateOnly = !meas && !v?.pattern;
  return { measured: !!meas, rateOnly, label: meas ? 'radar waveform' : rateOnly ? 'rate only' : 'pattern illustration',
    tip: rateOnly ? `Rate only (${rate ? Math.round(rate) + '/min' : '—'}): no breathing-pattern analysis, so this regular shape is NOT a pattern` : tip,
    svg: `<svg class="${cls}" viewBox="0 0 ${w} ${h}" width="${w}" height="${h}" role="img" aria-label="${esc(tip)}"><title>${esc(tip)}</title><path d="${d}" fill="none" stroke="${stroke}" stroke-width="1.4"${!meas && !v?.pattern ? ' stroke-dasharray="2 2.5" opacity="0.7"' : ''} stroke-linejoin="round" stroke-linecap="round"/></svg>` };
}

export const ALWAYS = 'Aid for rescuers, not a diagnosis: radar sees chest-wall motion only (no perfusion, responsiveness or airway).';
export function caveatsOf(c) {
  const v = c.vitals; const out = [];
  if (Array.isArray(v?.caveats)) for (const k of v.caveats) if (k && !out.includes(String(k))) out.push(String(k));
  if (!out.some(k => /not a diagnosis/i.test(k))) out.unshift(ALWAYS);
  if (!v) out.push('No breathing-pattern analysis for this ping in this run: ranked on localisation and depth only.');
  return out;
}
export const triageChip = (t, extra = '') => `<span class="tri t-${t.key}" title="${esc(t.tip)}">${esc(t.label)}${t.sub ? `<small> · ${esc(t.sub)}</small>` : ''}</span>${extra}`;

// Map ring style per triage (shape, not a new hue: red = ping, yellow = cones, teal = priority stay unambiguous).
export function drawTriageRing(ctx, x, y, t) {
  ctx.save(); ctx.strokeStyle = 'rgba(240,244,248,0.92)';
  const ring = (r, lw, dash) => { ctx.lineWidth = lw; ctx.setLineDash(dash); ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.stroke(); };
  ctx.shadowColor = 'rgba(0,0,0,0.7)'; ctx.shadowBlur = 3;
  if (t.key === 'immediate') { ring(12, 2, []); ring(15.5, 2, []); }
  else if (t.key === 'urgent') ring(13, 2.2, []);
  else if (t.key === 'delayed') ring(13, 1.5, [5, 4]);
  else ring(13, 1.6, [1.5, 3.5]);
  ctx.restore();
}
