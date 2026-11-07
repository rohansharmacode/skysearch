// Demo state used only when data/state.json is not available (e.g. before the coordinator runs).
// A plausible mid-mission 3x3 grid state (contracts/INTERFACES.md v1 + v1.1 grid extensions), so the
// dashboard code path is the same as with live data from sim/grid_mission.py.

function hash2(x, y) { const s = Math.sin(x * 127.1 + y * 311.7) * 43758.5453; return s - Math.floor(s); }
function vnoise(x, y) { const xi = Math.floor(x), yi = Math.floor(y), xf = x - xi, yf = y - yi; const u = xf * xf * (3 - 2 * xf), v = yf * yf * (3 - 2 * yf); const a = hash2(xi, yi), b = hash2(xi + 1, yi), c = hash2(xi, yi + 1), d = hash2(xi + 1, yi + 1); return a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v; }
function mulberry32(a) { return () => { a |= 0; a = a + 0x6D2B79F5 | 0; let t = Math.imul(a ^ a >>> 15, 1 | a); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; }; }

const SP = 22, BOUNDS = [-36, -54, 36, 36], BASE = [0, -47], PADS = [[-2.6, -49], [0, -49], [2.6, -49]];
const rnd = mulberry32(7), R = (a, b) => a + (b - a) * rnd();
const PILES = [];
for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) PILES.push({ id: 'ABC'[c] + (r + 1), x: (c - 1) * SP + R(-1.5, 1.5), y: (1 - r) * SP + R(-1.5, 1.5), h: R(2.6, 3.7), sx: R(5, 6.2), sy: R(4.4, 5.6), rot: R(0, Math.PI), lx: R(-2, 2), ly: R(-2, 2) });
const P = Object.fromEntries(PILES.map(p => [p.id, p]));
function mound(p, x, y) {
  const dx = x - p.x, dy = y - p.y; if (dx * dx + dy * dy > 170) return 0;
  const cs = Math.cos(p.rot), sn = Math.sin(p.rot), u = dx * cs + dy * sn, v = -dx * sn + dy * cs;
  return p.h * Math.exp(-((u / p.sx) ** 2 + (v / p.sy) ** 2) * 1.4) + 0.8 * Math.exp(-((dx - p.lx) ** 2 + (dy - p.ly) ** 2) / 4.5);
}
function H(x, y) {
  let m = 0; for (const p of PILES) m += mound(p, x, y);
  const f = Math.min(1, m / 0.9);
  return m + f * ((vnoise(x * 0.9, y * 0.9) - 0.5) * 0.7 + (vnoise(x * 3.1, y * 3.1) - 0.5) * 0.45) + 0.03 * Math.sin(0.21 * x + 0.13 * y);
}
const at = (p, r, a) => [+(p.x + r * Math.cos(a)).toFixed(2), +(p.y + r * Math.sin(a)).toFixed(2)];

export function demoState() {
  const res = 0.5, origin = [BOUNDS[0], BOUNDS[1]], w = (BOUNDS[2] - BOUNDS[0]) / res, h = (BOUNDS[3] - BOUNDS[1]) / res, z = new Array(w * h);
  for (let j = 0; j < h; j++) for (let i = 0; i < w; i++) z[j * w + i] = +H(origin[0] + (i + 0.5) * res, origin[1] + (j + 0.5) * res).toFixed(3);
  const A2 = P.A2, A3 = P.A3;
  const R1 = at(A2, 1.6, 2.2), R2 = at(A3, 1.2, -0.9);
  // belief: 30 m window around the active pile (A3)
  const bo = [Math.round(A3.x - 15), Math.round(A3.y - 15)], bw = 60, p = new Array(bw * bw); let s = 0;
  for (let j = 0; j < bw; j++) for (let i = 0; i < bw; i++) {
    const x = bo[0] + (i + 0.5) * res, y = bo[1] + (j + 0.5) * res;
    const v = Math.exp(-(((x - R2[0]) / 0.7) ** 2 + ((y - R2[1]) / 0.55) ** 2) / 2) + 0.35 * Math.exp(-(((x - R2[0] - 3.4) / 0.8) ** 2 + ((y - R2[1] + 2.1) / 0.7) ** 2) / 2);
    p[j * bw + i] = v; s += v;
  }
  for (let k = 0; k < p.length; k++) p[k] = +(p[k] / s).toExponential(3);
  const siteA2 = [at(A2, 9.5, 0.4), at(A2, 10.5, 2.5), at(A2, 9.8, 4.4)], siteA3 = [at(A3, 10, 0.9), at(A3, 10.8, 3.0), at(A3, 9.6, 5.0)];
  const hz = (xy) => +(H(...xy)).toFixed(2);
  const L = (pid, k, xy, status) => ({ id: `${pid}-L${k}`, pile_id: pid, xy, z: hz(xy), slope_deg: 2.1, roughness: 0.02, clearance: 2.2, score: 0.88, status });
  const drones = siteA3.map((xy, k) => ({ id: 'd' + (k + 1), pose: [...xy, hz(xy) + 0.15, 120 * k + 40], state: k === 2 ? 'scanning' : 'landed', battery: +(0.74 - 0.03 * k).toFixed(2) }));
  const clips = [
    { id: '1-ac-A3', listen: 1, t: 122, pile_id: 'A3', drone_id: 'd3', origin: siteA2[2], class: 'voice', transcript: 'Help! Can anyone hear me? My leg is trapped.', scores: { human: 0.9, urgency: 0.92, reliability: 0.82 }, priority: 77, by: 'llm', rationale: 'Clear call for help with an injury; 3/3 drones agree on the bearing, SNR 14 dB.' },
    { id: '1-ac-B2', listen: 1, t: 122, pile_id: 'B2', drone_id: 'd1', origin: siteA2[0], class: 'knocking', transcript: '', scores: { human: 0.65, urgency: 0.43, reliability: 0.86 }, priority: 37, by: 'fallback', rationale: 'knocking 0.55; knock regularity 0.75; 3/3 drones agree; width 22 deg, SNR 12 dB' },
    { id: '1-ac-B3', listen: 1, t: 122, pile_id: 'B3', drone_id: 'd3', origin: siteA2[2], class: 'debris', transcript: '', scores: { human: 0.02, urgency: 0.02, reliability: 0.9 }, priority: 1, by: 'fallback', rationale: 'debris 0.55; 3/3 drones agree; width 24 deg, SNR 15 dB' },
    { id: '1-ac-A1', listen: 1, t: 122, pile_id: 'A1', drone_id: 'd1', origin: siteA2[0], class: 'engine', transcript: '', scores: { human: 0.08, urgency: 0.02, reliability: 0.8 }, priority: 3, by: 'fallback', rationale: 'engine 0.59; steady harmonic hum; 2/3 drones agree; SNR 11 dB' },
  ].map(c => { const pp = P[c.pile_id]; return { ...c, bearing_deg: +(Math.atan2(pp.y - c.origin[1], pp.x - c.origin[0]) * 180 / Math.PI + 3).toFixed(1), width_deg: 22 }; });
  const piles = PILES.map(pp => ({ id: pp.id, xy: [+pp.x.toFixed(2), +pp.y.toFixed(2)], status: 'no_signal_not_cleared', priority: 8, radar_candidate_ids: [], why: 'No sound assigned to this pile during the listen. Silence is not absence: not cleared.' }));
  const set = (id, o) => Object.assign(piles.find(q => q.id === id), o);
  set('A2', { status: 'ping', priority: null, radar_candidate_ids: ['A2-R1'], why: 'Radar ping A2-R1, p 0.94, ≈1.6 m below the surface. Candidate search region, not a confirmed person.' });
  set('A3', { status: 'scanning', priority: 77, radar_candidate_ids: [], why: 'Voice saying "Help! Can anyone hear me? My leg is trapped."; drones landed, radar scanning.' });
  set('B2', { status: 'lead', priority: 37, why: 'Knocking; human 0.65, urgency 0.43, reliability 0.86.' });
  set('B3', { status: 'dismissed_nonhuman', priority: 1, why: 'Heard debris (human 0.02); dismissed as an acoustic lead, but the pile is not cleared.' });
  set('A1', { status: 'dismissed_nonhuman', priority: 3, why: 'Heard engine (human 0.08); dismissed as an acoustic lead, but the pile is not cleared.' });
  const scans = [
    ...siteA2.map((xy, k) => ({ scan_id: 's' + (k + 1), pile_id: 'A2', drone_id: 'd' + (k + 1), sensor_pose: [...xy, hz(xy) + 0.25, 0], timestamp: 60 + k, range_candidates: [+(Math.hypot(xy[0] - R1[0], xy[1] - R1[1]) + 0.1).toFixed(2)], confidence: [0.9], quality: 0.9, breathing_hz: 0.27, doppler_moving: false, used_weight: 0.9 })),
    ...siteA3.slice(0, 2).map((xy, k) => ({ scan_id: 's' + (k + 4), pile_id: 'A3', drone_id: 'd' + (k + 1), sensor_pose: [...xy, hz(xy) + 0.25, 0], timestamp: 178 + k, range_candidates: [+(Math.hypot(xy[0] - R2[0], xy[1] - R2[1]) + 0.1).toFixed(2)], confidence: [0.85], quality: 0.88, breathing_hz: 0.28, doppler_moving: false, used_weight: 0.8 })),
  ];
  return {
    schema: 'larpverse.state/v1', t: 184, phase: 'radar', demo: true,
    site: { name: 'Demo rubble grid 3x3 (no coordinator connected)', bounds: BOUNDS },
    heightmap: { origin, res, w, h, z },
    pointcloud: { demo: true, count: 160000 },
    drones,
    landing_sites: [...siteA2.map((xy, k) => L('A2', k + 1, xy, 'used')), ...siteA3.map((xy, k) => L('A3', k + 1, xy, 'used')), L('A3', 4, at(A3, 11, 1.9), 'candidate'), L('A3', 5, at(A3, 10.4, 4.1), 'candidate')],
    scans,
    radar_candidates: [
      { id: 'A2-R1', pile_id: 'A2', xy: R1, z_est: +(H(...R1) - 1.6).toFixed(2), cov: [[0.09, 0.02], [0.02, 0.06]], p_mass: 0.94, breathing_hz: 0.27, kind: 'stationary', status: 'candidate', evidence: ['s1', 's2', 's3'] },
      { id: 'A3-R1', pile_id: 'A3', xy: R2, z_est: +(H(...R2) - 1.0).toFixed(2), cov: [[0.3, 0.05], [0.05, 0.22]], p_mass: 0.62, breathing_hz: 0.28, kind: 'stationary', status: 'insufficient_evidence', evidence: ['s4', 's5'] },
    ],
    acoustic: clips.filter(c => c.scores.human >= 0.4).map((c, k) => ({ obs_id: 'a' + (k + 1), drone_id: c.drone_id, origin: c.origin, timestamp: c.t, bearing_deg: c.bearing_deg, width_deg: c.width_deg, class: c.class === 'voice' ? 'voice' : 'knocking', confidence: c.scores.reliability, pile_id: c.pile_id })),
    belief: { origin: bo, res, w: bw, h: bw, pile_id: 'A3', p },
    next_scan: { drone_id: 'd3', site_id: 'A3-L3', pile_id: 'A3', reason: 'separates two modes 3.9 m apart', score: 0.7 },
    log: [
      { t: 0, msg: 'Grid mission start: 3 drones at the staging area, start pile A2 (operator choice)' },
      { t: 96, msg: 'PING A2-R1 at pile A2: depth ≈1.6 m, p 0.94 -> sent to the operator dashboard' },
      { t: 96, msg: 'Radar phase at A2 over; landed drones listen to the other piles for 26 s' },
      { t: 122, msg: 'Clip 1-ac-A3 (d3): voice "Help! Can anyone hear me? My leg is trapped." -> priority 77 for A3 [llm]' },
      { t: 122, msg: 'Pile priority: A3 77, B2 37, A1 3, B3 1' },
      { t: 122, msg: 'Dispatch d1-d3 -> pile A3: highest priority (77), a clear call for help' },
      { t: 182, msg: '[A3] Belief ambiguous: 2 significant modes, 3.9 m apart' },
    ],
    mission: { kind: 'grid', seed: 7, start_pile: 'A2', pile_budget: 3, priority_min: 20, visited: ['A2', 'A3'], active_pile: 'A3', stage: 'pile', acoustic_source: 'demo', listen_ends: null, in_transit: null },
    staging: { xy: BASE, pads: PADS },
    piles,
    acoustic_clips: clips,
    acoustic_agent: { mode: 'demo', llm_calls: 1, fallback: 3 },
    dispatch: [{ t: 122, from_pile: 'A2', to_pile: 'A3', reason: 'Send the radar drones to A3: highest priority (77) from "Help! Can anyone hear me?…", 3/3 drones agreeing on the bearing.' }],
  };
}

// Surface samples from the demo heightmap, dense on the piles, as float32 xyz.
export function demoCloud(n = 160000) {
  const a = new Float32Array(n * 3); let s = 12345;
  const r = () => ((s = (s * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff);
  for (let k = 0; k < n; k++) {
    let x, y;
    if (r() < 0.75) { const p = PILES[Math.floor(r() * 9)], rad = 10 * Math.sqrt(r()), th = r() * Math.PI * 2; x = p.x + rad * Math.cos(th); y = p.y + rad * Math.sin(th); }
    else { x = BOUNDS[0] + r() * (BOUNDS[2] - BOUNDS[0]); y = BOUNDS[1] + r() * (BOUNDS[3] - BOUNDS[1]); }
    const hz = H(x, y); a[k * 3] = x; a[k * 3 + 1] = y; a[k * 3 + 2] = hz + (hz > 0.4 ? (r() - 0.5) * 0.25 : 0.01);
  }
  return a;
}
