/* picker.js — site picker + 3D LiDAR-style topology map for the live operator console.
 *
 *   Picker.start({ stage, sites:[{key,label,site,seed,look,data}], onSite(siteObj) })
 *   Picker.topo({ stage, data, title, loading, onPick({pile, xy, click}), onBack() })
 *   Picker.setLoading(bool)
 *   Picker.close()
 *
 * `data` is the live state.json shape: { site:{name}, heightmap:{origin:[x0,y0],res,w,h,z:[w*h]},
 * piles:[{id,xy,polygon}], staging:{xy,pads} }. Heightmap is row-major: z[j*w+i] at x=x0+i*res, y=y0+j*res
 * (checked: every pile centre sits on a 3.5-4.6 m peak; column-major gives ~0 m). World is metres, z up.
 * Vanilla JS, no libraries. Dots are drawn with WebGL1 (Canvas 2D fallback); outlines/labels on a 2D overlay.
 * The overlay is position:absolute; inset:0; z-index:80 inside `stage`. CSS is injected from this file.
 */
(function () {
'use strict';

// ------------------------------------------------------------------------------------------------ settings
const HOVER_R = 10;         // m: yellow falloff radius around the mouse
const SNAP_R = 15;          // m: nearest pile centre accepted when the click is outside every outline
const PICK_DELAY = 1100;    // ms: the confirmation plays before onPick is called
const YEL = [1.0, 0.82, 0.16];
const D2R = Math.PI / 180;

const CSS = `
.pk-root { position:absolute; inset:0; z-index:80; background:#06080b; color:#eef2f5; overflow:hidden;
  font-family: Inter, "Segoe UI", system-ui, sans-serif; user-select:none; -webkit-user-select:none; }
.pk-root * { box-sizing:border-box; }
.pk-root .hide { display:none !important; }
.pk-fadein { animation: pkfade .35s ease both; } @keyframes pkfade { from { opacity:0; } to { opacity:1; } }
.pk-kick { font-size:15px; letter-spacing:.18em; text-transform:uppercase; color:#8d98a1; }
.pk-kick b { color:var(--amber, #ffb020); font-weight:700; }
/* start screen */
.pk-start { position:absolute; inset:0; display:flex; flex-direction:column; align-items:center; padding-top:110px;
  background: radial-gradient(1200px 700px at 50% 35%, #111820 0%, #06080b 70%); }
.pk-start h1 { font-size:58px; font-weight:800; margin:14px 0 10px; letter-spacing:-.01em; }
.pk-start .pk-sub { font-size:23px; color:#aab4bc; margin-bottom:64px; }
.pk-cards { display:flex; gap:40px; }
.pk-card { width:540px; border-radius:20px; background:rgba(255,255,255,0.045); border:2px solid rgba(255,255,255,0.10);
  padding:18px 18px 22px; cursor:pointer; transition: transform .18s ease, border-color .18s ease, background .18s ease; }
.pk-card:hover { transform: translateY(-6px); border-color:var(--amber, #ffb020); background:rgba(255,176,32,0.06); }
.pk-card.on { border-color:var(--amber, #ffb020); background:rgba(255,176,32,0.12); }
.pk-card canvas { width:500px; height:380px; display:block; border-radius:12px; background:#000; }
.pk-card .pk-nomap { width:500px; height:380px; border-radius:12px; background:#000; display:flex; align-items:center;
  justify-content:center; color:#6f7a83; font-size:18px; }
.pk-card .pk-cl { font-size:32px; font-weight:700; margin:18px 4px 6px; }
.pk-card .pk-cm { font-size:18px; color:#aab4bc; margin:0 4px; display:flex; justify-content:space-between; align-items:center; }
.pk-card .pk-go { color:var(--amber, #ffb020); font-weight:700; opacity:.0; transition:opacity .18s; }
.pk-card:hover .pk-go, .pk-card.on .pk-go { opacity:1; }
.pk-foot { position:absolute; bottom:44px; font-size:18px; color:#6f7a83; }
/* topo screen */
.pk-topo { position:absolute; inset:0; cursor:crosshair; }
.pk-topo.drag { cursor:grabbing; }
.pk-topo canvas { position:absolute; left:0; top:0; width:100%; height:100%; display:block; }
.pk-vig { position:absolute; inset:0; pointer-events:none;
  background: radial-gradient(ellipse 75% 70% at 50% 52%, rgba(0,0,0,0) 55%, rgba(0,0,0,0.55) 100%); }
.pk-top { position:absolute; left:0; right:0; top:0; height:150px; padding:26px 36px 0; display:flex; gap:28px; align-items:flex-start;
  pointer-events:none; background:linear-gradient(#06080bee 0%, #06080b99 60%, #06080b00 100%); }
.pk-top > * { pointer-events:auto; }
.pk-back { font:inherit; font-size:20px; font-weight:600; color:#e6ecf0; background:#151a1f; border:1px solid #39424a;
  border-radius:12px; padding:12px 20px; cursor:pointer; white-space:nowrap; }
.pk-back:hover { border-color:var(--amber, #ffb020); color:#fff; }
.pk-tt { flex:1; }
.pk-tt h2 { font-size:40px; font-weight:800; margin:6px 0 0; line-height:1.1; }
.pk-status { display:flex; align-items:center; gap:12px; padding:12px 20px; border-radius:999px; font-size:19px;
  background:rgba(10,14,18,0.8); border:1px solid #39424a; white-space:nowrap; margin-top:4px; }
.pk-status i { width:12px; height:12px; border-radius:50%; background:#4fd18b; box-shadow:0 0 10px #4fd18b; }
.pk-status.loading { border-color:rgba(255,176,32,0.55); }
.pk-status.bad { border-color:rgba(255,90,82,0.7); color:#ffb4ae; } .pk-status.bad i { background:#ff5a52; box-shadow:0 0 10px #ff5a52; animation:none; }
.pk-status.loading i { background:var(--amber, #ffb020); box-shadow:0 0 10px var(--amber, #ffb020); animation: pkblink 1.1s ease-in-out infinite; }
@keyframes pkblink { 50% { opacity:.25; } }
.pk-hint { position:absolute; left:50%; bottom:34px; transform:translateX(-50%); font-size:21px; color:#cfd8de; white-space:nowrap;
  padding:12px 24px; border-radius:999px; background:rgba(10,14,18,0.78); border:1px solid rgba(255,255,255,0.10); pointer-events:none; }
.pk-hint b { color:var(--amber, #ffb020); }
.pk-legend { position:absolute; left:36px; bottom:34px; font-size:16px; color:#aab4bc; line-height:1.75; pointer-events:none; }
.pk-legend span { display:inline-block; width:34px; text-align:center; margin-right:8px; }
.pk-tip { position:absolute; left:0; top:0; padding:8px 14px; border-radius:10px; font-size:18px; font-weight:600; white-space:nowrap;
  background:rgba(10,14,18,0.86); border:1px solid rgba(255,210,40,0.55); color:#fff; pointer-events:none;
  transform: translate(22px, -50%); transition: opacity .15s; }
.pk-tip.off { opacity:0; }
.pk-tip.no { border-color:rgba(255,255,255,0.18); color:#aab4bc; font-weight:500; }
.pk-tip.bad { border-color:#ff5252; color:#ffb3b3; }
.pk-conf { position:absolute; left:50%; top:170px; transform:translateX(-50%); padding:18px 34px; border-radius:18px;
  background:rgba(10,14,18,0.9); border:2px solid var(--amber, #ffb020); font-size:34px; font-weight:800; white-space:nowrap;
  box-shadow:0 10px 40px rgba(0,0,0,0.5), 0 0 30px rgba(255,176,32,0.25); pointer-events:none; animation: pkpop .35s ease-out both; }
.pk-conf small { display:block; font-size:18px; font-weight:500; color:#aab4bc; margin-top:4px; text-align:center; }
@keyframes pkpop { from { opacity:0; transform:translate(-50%, 10px) scale(.96); } to { opacity:1; transform:translate(-50%, 0) scale(1); } }
`;

// ------------------------------------------------------------------------------------------------ helpers
function injectCSS() {
  if (document.getElementById('pk-css')) return;
  const s = document.createElement('style'); s.id = 'pk-css'; s.textContent = CSS; document.head.appendChild(s);
}
function mk(tag, cls, html) { const e = document.createElement(tag); if (cls) e.className = cls; if (html != null) e.innerHTML = html; return e; }
const esc = s => String(s == null ? '' : s).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
function rng(seed) { let a = seed >>> 0; return () => { a |= 0; a = a + 0x6D2B79F5 | 0; let t = Math.imul(a ^ a >>> 15, 1 | a);
  t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; }; }
function inPoly(x, y, poly) {
  let c = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const xi = poly[i][0], yi = poly[i][1], xj = poly[j][0], yj = poly[j][1];
    if (((yi > y) !== (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) c = !c;
  }
  return c;
}
const ease3 = t => 1 - Math.pow(1 - Math.max(0, Math.min(1, t)), 3);

// ------------------------------------------------------------------------------------------------ data prep
function prep(data) {
  data = data || {};
  const hm = data.heightmap, rawPiles = (data.piles || []).filter(p => p && Array.isArray(p.xy));
  let H;
  if (hm && hm.z && hm.w && hm.h && hm.z.length >= hm.w * hm.h) {
    H = { x0: +hm.origin[0], y0: +hm.origin[1], res: +hm.res, w: hm.w | 0, h: hm.h | 0, z: Float32Array.from(hm.z) };
  } else {   // no heightmap: a flat ground around the piles so the map still works
    const pts = rawPiles.flatMap(p => p.polygon && p.polygon.length ? p.polygon : [p.xy]);
    if (data.staging && data.staging.xy) pts.push(data.staging.xy);
    if (!pts.length) pts.push([-30, -30], [30, 30]);
    const xs = pts.map(p => p[0]), ys = pts.map(p => p[1]), res = 0.5;
    const x0 = Math.min(...xs) - 10, y0 = Math.min(...ys) - 10;
    const w = Math.ceil((Math.max(...xs) + 10 - x0) / res), h = Math.ceil((Math.max(...ys) + 10 - y0) / res);
    H = { x0, y0, res, w, h, z: new Float32Array(w * h) };
  }
  H.x1 = H.x0 + (H.w - 1) * H.res; H.y1 = H.y0 + (H.h - 1) * H.res;
  const s = Float32Array.from(H.z).sort(), n = s.length;
  H.zmin = s[0]; H.zmax = s[n - 1]; H.zlo = s[Math.floor(n * 0.03)]; H.zhi = Math.max(H.zlo + 0.5, s[Math.floor(n * 0.995)]);
  H.at = (x, y) => {
    let fi = (x - H.x0) / H.res, fj = (y - H.y0) / H.res;
    fi = Math.max(0, Math.min(H.w - 1.001, fi)); fj = Math.max(0, Math.min(H.h - 1.001, fj));
    const i = fi | 0, j = fj | 0, u = fi - i, v = fj - j, k = j * H.w + i, z = H.z;
    return (z[k] * (1 - u) + z[k + 1] * u) * (1 - v) + (z[k + H.w] * (1 - u) + z[k + H.w + 1] * u) * v;
  };
  H.inside = (x, y) => x >= H.x0 && x <= H.x1 && y >= H.y0 && y <= H.y1;
  const piles = rawPiles.map(p => {
    let poly = p.polygon && p.polygon.length >= 3 ? p.polygon : null;
    if (!poly) { poly = []; for (let a = 0; a < 16; a++) poly.push([p.xy[0] + 7 * Math.cos(a / 16 * 2 * Math.PI), p.xy[1] + 7 * Math.sin(a / 16 * 2 * Math.PI)]); }
    // the top of the pile, for its label
    const xs = poly.map(q => q[0]), ys = poly.map(q => q[1]);
    let top = H.at(p.xy[0], p.xy[1]);
    for (let y = Math.min(...ys); y <= Math.max(...ys); y += H.res)
      for (let x = Math.min(...xs); x <= Math.max(...xs); x += H.res)
        if (H.inside(x, y) && inPoly(x, y, poly)) top = Math.max(top, H.at(x, y));
    // draped outline: the dashed line follows the rubble
    const drape = [];
    for (let i = 0; i < poly.length; i++) {
      const a = poly[i], b = poly[(i + 1) % poly.length], L = Math.hypot(b[0] - a[0], b[1] - a[1]), m = Math.max(1, Math.ceil(L / 0.8));
      for (let k = 0; k < m; k++) { const x = a[0] + (b[0] - a[0]) * k / m, y = a[1] + (b[1] - a[1]) * k / m; drape.push([x, y, H.at(x, y) + 0.15]); }
    }
    return { id: String(p.id), xy: [+p.xy[0], +p.xy[1]], poly, top, drape };
  });
  const st = data.staging && Array.isArray(data.staging.xy) ? data.staging : null;
  return { H, piles, staging: st, name: data.site && data.site.name };
}

// brighter = higher (ground dim grey, rubble rising to white)
function bright(H, z) {
  const g = H.zlo + 0.25;
  if (z < g) return 0.20 + 0.07 * Math.max(0, Math.min(1, (z - H.zlo) / 0.25));
  return 0.38 + 0.62 * Math.sqrt(Math.min(1, (z - g) / Math.max(0.5, H.zhi - g)));
}

// one LiDAR-ish dot per sub-cell, jittered so it reads as a scan, not a grid. stride 5: x y z brightness rnd
function makePoints(P, f) {
  const H = P.H, nw = H.w * f, nh = H.h * f, out = new Float32Array(nw * nh * 5), r = rng(7), step = H.res / f;
  let o = 0;
  for (let j = 0; j < nh; j++) for (let i = 0; i < nw; i++) {
    const x = H.x0 + (i + (r() - 0.5) * 0.9) * step, y = H.y0 + (j + (r() - 0.5) * 0.9) * step;
    const z = H.at(x, y) + (r() - 0.5) * 0.06;
    out[o++] = x; out[o++] = y; out[o++] = z; out[o++] = Math.min(1, bright(H, z) * (0.82 + 0.3 * r())); out[o++] = r();
  }
  return out;
}

// ------------------------------------------------------------------------------------------------ mat4 (column-major)
function persp(fovy, asp, n, f) { const t = 1 / Math.tan(fovy / 2), nf = 1 / (n - f);
  return [t / asp, 0, 0, 0, 0, t, 0, 0, 0, 0, (f + n) * nf, -1, 0, 0, 2 * f * n * nf, 0]; }
function lookAt(e, c, u) {
  let zx = e[0] - c[0], zy = e[1] - c[1], zz = e[2] - c[2], l = Math.hypot(zx, zy, zz); zx /= l; zy /= l; zz /= l;
  let xx = u[1] * zz - u[2] * zy, xy = u[2] * zx - u[0] * zz, xz = u[0] * zy - u[1] * zx; l = Math.hypot(xx, xy, xz); xx /= l; xy /= l; xz /= l;
  const yx = zy * xz - zz * xy, yy = zz * xx - zx * xz, yz = zx * xy - zy * xx;
  return [xx, yx, zx, 0, xy, yy, zy, 0, xz, yz, zz, 0,
    -(xx * e[0] + xy * e[1] + xz * e[2]), -(yx * e[0] + yy * e[1] + yz * e[2]), -(zx * e[0] + zy * e[1] + zz * e[2]), 1];
}
function mul(a, b) { const o = new Array(16);
  for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++)
    o[c * 4 + r] = a[r] * b[c * 4] + a[4 + r] * b[c * 4 + 1] + a[8 + r] * b[c * 4 + 2] + a[12 + r] * b[c * 4 + 3];
  return o; }
function inv(m) {
  const [a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23, a30, a31, a32, a33] = m;
  const b00 = a00 * a11 - a01 * a10, b01 = a00 * a12 - a02 * a10, b02 = a00 * a13 - a03 * a10, b03 = a01 * a12 - a02 * a11,
    b04 = a01 * a13 - a03 * a11, b05 = a02 * a13 - a03 * a12, b06 = a20 * a31 - a21 * a30, b07 = a20 * a32 - a22 * a30,
    b08 = a20 * a33 - a23 * a30, b09 = a21 * a32 - a22 * a31, b10 = a21 * a33 - a23 * a31, b11 = a22 * a33 - a23 * a32;
  const d = 1 / (b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06);
  return [(a11 * b11 - a12 * b10 + a13 * b09) * d, (a02 * b10 - a01 * b11 - a03 * b09) * d, (a31 * b05 - a32 * b04 + a33 * b03) * d,
    (a22 * b04 - a21 * b05 - a23 * b03) * d, (a12 * b08 - a10 * b11 - a13 * b07) * d, (a00 * b11 - a02 * b08 + a03 * b07) * d,
    (a32 * b02 - a30 * b05 - a33 * b01) * d, (a20 * b05 - a22 * b02 + a23 * b01) * d, (a10 * b10 - a11 * b08 + a13 * b06) * d,
    (a01 * b08 - a00 * b10 - a03 * b06) * d, (a30 * b04 - a31 * b02 + a33 * b00) * d, (a21 * b02 - a20 * b04 - a23 * b00) * d,
    (a11 * b07 - a10 * b09 - a12 * b06) * d, (a00 * b09 - a01 * b07 + a02 * b06) * d, (a31 * b01 - a30 * b03 - a32 * b00) * d,
    (a20 * b03 - a21 * b01 + a22 * b00) * d];
}
// world -> [ndcX, ndcY, w]
function proj(m, x, y, z) {
  const w = m[3] * x + m[7] * y + m[11] * z + m[15];
  return [(m[0] * x + m[4] * y + m[8] * z + m[12]) / w, (m[1] * x + m[5] * y + m[9] * z + m[13]) / w, w];
}

// ------------------------------------------------------------------------------------------------ dot renderers
const VS = `
attribute vec3 aPos; attribute float aB; attribute float aR;
uniform mat4 uVP; uniform vec2 uMouse; uniform float uHover; uniform float uRad; uniform float uSize; uniform float uRef;
uniform float uIntro; uniform float uTime; uniform vec3 uPulse; uniform float uPulseA; uniform vec3 uYel;
varying vec3 vC;
void main() {
  float k = clamp(uIntro * 1.4 - aR * 0.4, 0.0, 1.0); k = 1.0 - (1.0 - k) * (1.0 - k) * (1.0 - k);
  vec4 cp = uVP * vec4(aPos.xy, aPos.z * k, 1.0);
  gl_Position = cp;
  float h = 1.0 - smoothstep(0.0, uRad, distance(aPos.xy, uMouse));
  h = h * uHover;
  float e = (distance(aPos.xy, uPulse.xy) - uPulse.z) / 1.8;
  float p = exp(-e * e) * uPulseA;
  float t = clamp(max(h, p), 0.0, 1.0);
  float b = aB * (0.92 + 0.08 * sin(uTime * 1.7 + aR * 60.0));
  vec3 yel = uYel * (0.45 + 0.80 * aB);
  vC = mix(vec3(b), yel, t) * (0.35 + 0.65 * k) * (1.0 + 0.35 * t);
  gl_PointSize = max(1.0, uSize * uRef / cp.w * (1.0 + 0.3 * t));
}`;
const FS = `
precision mediump float; varying vec3 vC;
void main() { vec2 q = gl_PointCoord * 2.0 - 1.0; float r2 = dot(q, q); if (r2 > 1.0) discard;
  gl_FragColor = vec4(vC * (1.0 - r2 * r2) * 0.9, 1.0); }`;

function glDots(canvas, pts) {
  const gl = canvas.getContext('webgl', { alpha: false, antialias: false, premultipliedAlpha: false });
  if (!gl) return null;
  const sh = (t, s) => { const o = gl.createShader(t); gl.shaderSource(o, s); gl.compileShader(o);
    if (!gl.getShaderParameter(o, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(o)); return o; };
  const pr = gl.createProgram(); gl.attachShader(pr, sh(gl.VERTEX_SHADER, VS)); gl.attachShader(pr, sh(gl.FRAGMENT_SHADER, FS));
  gl.linkProgram(pr); if (!gl.getProgramParameter(pr, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(pr));
  gl.useProgram(pr);
  const buf = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, buf); gl.bufferData(gl.ARRAY_BUFFER, pts, gl.STATIC_DRAW);
  const at = n => gl.getAttribLocation(pr, n);
  [[at('aPos'), 3, 0], [at('aB'), 1, 12], [at('aR'), 1, 16]].forEach(([l, n, off]) => {
    gl.enableVertexAttribArray(l); gl.vertexAttribPointer(l, n, gl.FLOAT, false, 20, off); });
  const U = {}; ['uVP', 'uMouse', 'uHover', 'uRad', 'uSize', 'uRef', 'uIntro', 'uTime', 'uPulse', 'uPulseA', 'uYel']
    .forEach(n => { U[n] = gl.getUniformLocation(pr, n); });
  let N = pts.length / 5;
  gl.disable(gl.DEPTH_TEST); gl.enable(gl.BLEND); gl.blendFunc(gl.ONE, gl.ONE);
  return {
    kind: 'webgl',
    setPoints(p) { gl.bindBuffer(gl.ARRAY_BUFFER, buf); gl.bufferData(gl.ARRAY_BUFFER, p, gl.STATIC_DRAW); N = p.length / 5; },
    draw(u) {
      gl.viewport(0, 0, canvas.width, canvas.height);
      gl.clearColor(0.024, 0.031, 0.043, 1); gl.clear(gl.COLOR_BUFFER_BIT);
      gl.uniformMatrix4fv(U.uVP, false, new Float32Array(u.vp));
      gl.uniform2f(U.uMouse, u.mx, u.my); gl.uniform1f(U.uHover, u.hover); gl.uniform1f(U.uRad, HOVER_R);
      gl.uniform1f(U.uSize, u.size); gl.uniform1f(U.uRef, u.ref); gl.uniform1f(U.uIntro, u.intro); gl.uniform1f(U.uTime, u.time);
      gl.uniform3f(U.uPulse, u.px, u.py, u.pr); gl.uniform1f(U.uPulseA, u.pa); gl.uniform3f(U.uYel, YEL[0], YEL[1], YEL[2]);
      gl.drawArrays(gl.POINTS, 0, N);
    },
    destroy() { try { gl.deleteBuffer(buf); gl.deleteProgram(pr); const x = gl.getExtension('WEBGL_lose_context'); if (x) x.loseContext(); } catch (e) { } },
  };
}

// Canvas 2D fallback: same look, projected in JS, additive 2x2 dots into an ImageData
function cpuDots(canvas, pts) {
  const g = canvas.getContext('2d'), W = canvas.width, Hh = canvas.height, img = g.createImageData(W, Hh), d = img.data;
  const acc = new Float32Array(W * Hh * 3), N = pts.length / 5;
  return {
    kind: '2d',
    setPoints() { },
    draw(u) {
      acc.fill(0); const m = u.vp, R2 = HOVER_R * HOVER_R;
      for (let n = 0, o = 0; n < N; n++, o += 5) {
        const x = pts[o], y = pts[o + 1], aB = pts[o + 3], aR = pts[o + 4];
        let k = Math.max(0, Math.min(1, u.intro * 1.4 - aR * 0.4)); k = 1 - (1 - k) ** 3;
        const z = pts[o + 2] * k, w = m[3] * x + m[7] * y + m[11] * z + m[15];
        if (w <= 0) continue;
        const sx = ((m[0] * x + m[4] * y + m[8] * z + m[12]) / w + 1) * 0.5 * W, sy = (1 - (m[1] * x + m[5] * y + m[9] * z + m[13]) / w) * 0.5 * Hh;
        const ix = sx | 0, iy = sy | 0; if (ix < 0 || iy < 0 || ix >= W - 1 || iy >= Hh - 1) continue;
        const dx = x - u.mx, dy = y - u.my, dd = dx * dx + dy * dy;
        let h = 0; if (dd < R2) { const q = Math.sqrt(dd) / HOVER_R, s = q * q * (3 - 2 * q); h = (1 - s) * u.hover; }
        let p = 0; if (u.pa > 0) { const e = (Math.hypot(x - u.px, y - u.py) - u.pr) / 1.8; p = Math.exp(-e * e) * u.pa; }
        const t = Math.min(1, Math.max(h, p)), yl = 0.45 + 0.80 * aB, f = (0.35 + 0.65 * k) * (1 + 0.25 * t) * 0.6;
        const r = (aB + (YEL[0] * yl - aB) * t) * f, gg = (aB + (YEL[1] * yl - aB) * t) * f, b = (aB + (YEL[2] * yl - aB) * t) * f;
        for (const q of [0, 3, W * 3, W * 3 + 3]) { const j = (iy * W + ix) * 3 + q; acc[j] += r; acc[j + 1] += gg; acc[j + 2] += b; }
      }
      for (let i = 0, j = 0; i < acc.length; i += 3, j += 4) {
        d[j] = 6 + acc[i] * 255; d[j + 1] = 8 + acc[i + 1] * 255; d[j + 2] = 11 + acc[i + 2] * 255; d[j + 3] = 255; }
      g.putImageData(img, 0, 0);
    },
    destroy() { },
  };
}

// ------------------------------------------------------------------------------------------------ preview (top-down)
function drawPreview(canvas, P, look) {
  const g = canvas.getContext('2d'), W = canvas.width, Hc = canvas.height, H = P.H;
  g.fillStyle = '#000'; g.fillRect(0, 0, W, Hc);
  const pad = 14, k = Math.min((W - 2 * pad) / (H.x1 - H.x0), (Hc - 2 * pad) / (H.y1 - H.y0));
  const cx = (H.x0 + H.x1) / 2, cy = (H.y0 + H.y1) / 2;
  const px = (x, y) => [W / 2 + (x - cx) * k, Hc / 2 - (y - cy) * k];   // north up
  const img = g.getImageData(0, 0, W, Hc), d = img.data, r = rng(11);
  const tint = look === 'dusk' ? [1.0, 0.86, 0.72] : [1, 1, 1];
  for (let j = 0; j < H.h; j++) for (let i = 0; i < H.w; i++) {
    const z = H.z[j * H.w + i], [sx, sy] = px(H.x0 + (i + r() - 0.5) * H.res, H.y0 + (j + r() - 0.5) * H.res);
    const x = sx | 0, y = sy | 0; if (x < 0 || y < 0 || x >= W - 1 || y >= Hc - 1) continue;
    const c = 255 * bright(H, z);
    for (const o of [0, 4, W * 4, W * 4 + 4]) { const q = (y * W + x) * 4 + o;
      if (d[q] < c * tint[0]) { d[q] = c * tint[0]; d[q + 1] = c * tint[1]; d[q + 2] = c * tint[2]; } }
  }
  g.putImageData(img, 0, 0);
  g.setLineDash([6, 6]); g.lineWidth = 2; g.strokeStyle = 'rgba(255,255,255,0.35)';
  g.font = '600 22px Inter, system-ui, sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
  for (const p of P.piles) {
    g.beginPath(); p.poly.forEach((q, i) => { const [x, y] = px(q[0], q[1]); i ? g.lineTo(x, y) : g.moveTo(x, y); }); g.closePath(); g.stroke();
  }
  g.setLineDash([]);
  for (const p of P.piles) { const [x, y] = px(p.xy[0], p.xy[1]);
    g.lineWidth = 4; g.strokeStyle = 'rgba(0,0,0,0.8)'; g.strokeText(p.id, x, y); g.fillStyle = '#cfd8de'; g.fillText(p.id, x, y); }
  if (P.staging) { const [x, y] = px(P.staging.xy[0], P.staging.xy[1]);
    g.fillStyle = '#ffb020'; g.fillRect(x - 9, y - 6, 18, 12);
    g.font = '700 18px Inter, system-ui, sans-serif'; g.textAlign = 'left'; g.lineWidth = 4; g.strokeText('Van', x + 14, y); g.fillText('Van', x + 14, y); }
}

// ------------------------------------------------------------------------------------------------ module state
let S = null;   // { root, stage, ac, raf, timers, topo }
function ensureRoot(stage) {
  injectCSS();
  if (S && S.stage === stage && S.root.isConnected) { stopTopo(); S.root.innerHTML = ''; return S; }
  close();
  const root = mk('div', 'pk-root pk-fadein');
  stage.appendChild(root);
  S = { root, stage, timers: [], topo: null, loading: false };
  return S;
}
function stopTopo() {
  if (!S) return;
  S.timers.forEach(clearTimeout); S.timers = [];
  if (S.topo) { cancelAnimationFrame(S.topo.raf); S.topo.ac.abort(); S.topo.dots && S.topo.dots.destroy(); S.topo = null; }
}
function close() {
  if (!S) return;
  stopTopo(); S.root.remove(); S = null;
}

const LOOKS = { day: 'Daylight', dusk: 'Dusk, dusty air', night: 'Night' };

// ------------------------------------------------------------------------------------------------ start screen
function start(opts) {
  const { stage, sites = [], onSite } = opts || {};
  if (!stage) throw new Error('Picker.start: stage is required');
  const st = ensureRoot(stage);
  const box = mk('div', 'pk-start pk-fadein', `
    <div class="pk-kick">Step <b>1</b> of 2</div>
    <h1>Choose a disaster site</h1>
    <div class="pk-sub">Each map is a LiDAR scan of the ground, seen from above. Brighter dots are higher rubble.</div>
    <div class="pk-cards"></div>
    <div class="pk-foot">Next, you pick where the drones search first.</div>`);
  st.root.appendChild(box);
  const cards = box.querySelector('.pk-cards');
  sites.forEach(site => {
    const c = mk('div', 'pk-card');
    let P = null; try { P = site.data ? prep(site.data) : null; } catch (e) { P = null; }
    const nP = P ? P.piles.length : 0;
    const look = LOOKS[site.look] || (site.look ? String(site.look) : '');
    c.innerHTML = `${P ? '<canvas width="1000" height="760"></canvas>' : '<div class="pk-nomap">Map preview not ready yet</div>'}
      <div class="pk-cl">${esc(site.label || site.key || site.site)}</div>
      <div class="pk-cm"><span>${nP ? `${nP} rubble piles` : 'Rubble site'}${look ? ` · ${esc(look)}` : ''}</span><span class="pk-go">Open map →</span></div>`;
    if (P) drawPreview(c.querySelector('canvas'), P, site.look);
    c.addEventListener('click', () => {
      [...cards.children].forEach(x => x.classList.remove('on')); c.classList.add('on');
      if (onSite) onSite(site);
    });
    cards.appendChild(c);
  });
}

// ------------------------------------------------------------------------------------------------ topo screen
function topo(opts) {
  const { stage, data, title, loading, onPick, onBack, density = 'auto', view = {} } = opts || {};
  if (!stage) throw new Error('Picker.topo: stage is required');
  const st = ensureRoot(stage);
  const P = prep(data), H = P.H;
  const W = stage.offsetWidth || 1920, Hs = stage.offsetHeight || 1080;
  const nm = title || P.name || 'This site';

  const box = mk('div', 'pk-topo pk-fadein', `
    <canvas class="pk-dots"></canvas><div class="pk-vig"></div><canvas class="pk-ov"></canvas>
    <div class="pk-top">
      <button class="pk-back" type="button">← Sites</button>
      <div class="pk-tt"><div class="pk-kick">Step <b>2</b> of 2 · ${esc(nm)}</div><h2>Where should the drones search first?</h2></div>
      <div class="pk-status"><i></i><span></span></div>
    </div>
    <div class="pk-legend"><div><span style="color:#fff">●</span>Brighter dots = higher rubble</div>
      <div><span style="color:#cfd8de;letter-spacing:-1px">- - -</span>Rubble pile</div>
      <div><span style="color:#ffb020">■</span>Van: the rescue team's base</div></div>
    <div class="pk-hint">Point at the map. <b>Click a rubble pile</b> to send the drones there. Drag to turn the map.</div>
    <div class="pk-tip off"></div>`);
  st.root.appendChild(box);
  const $ = s => box.querySelector(s);
  const cvD = $('.pk-dots'), cvO = $('.pk-ov'), tip = $('.pk-tip'), hint = $('.pk-hint');
  cvD.width = cvO.width = W; cvD.height = cvO.height = Hs;
  const og = cvO.getContext('2d');

  let dots = null;
  try { dots = glDots(cvD, makePoints(P, density === 'low' ? 1 : 2)); } catch (e) { console.warn('Picker: WebGL failed, using 2D', e); dots = null; }
  if (!dots) { const c2 = cvD.cloneNode(); cvD.replaceWith(c2); dots = cpuDots(c2, makePoints(P, 1)); }

  // ---- camera: oblique 3/4 view from the south-west, gentle sway; drag to turn
  const T = { ac: new AbortController(), raf: 0, dots, P,
    az0: (view.az != null ? view.az : -35) * D2R, amp: 9 * D2R, el: (view.el != null ? view.el : 36) * D2R, fov: (view.fov || 30) * D2R, factor: density === 'low' ? 1 : 2, size: density === 'low' ? 3.6 : 2.7, perf: { n: 0, t: 0 }, userAz: 0, orbitT: 0, orbitV: 1,
    tgt: [(H.x0 + H.x1) / 2, (H.y0 + H.y1) / 2, (H.zlo + H.zhi) * 0.3], D: 100, shift: 0,
    mouse: null, lastMove: -1e9, hover: 0, mx: 0, my: 0, hw: null, intro: 0, t0: performance.now(), last: performance.now(),
    down: null, drag: false, busy: false, pulse: null, onPick, hoverPile: null, near: null, badUntil: 0 };
  st.topo = T;

  function mats(az, D, shift) {
    const ce = Math.cos(T.el), eye = [T.tgt[0] + D * ce * Math.sin(az), T.tgt[1] - D * ce * Math.cos(az), T.tgt[2] + D * Math.sin(T.el)];
    const pm = persp(T.fov, W / Hs, D * 0.05, D * 5); pm[9] = -shift;   // lens shift: moves the image by +shift in NDC y
    return mul(pm, lookAt(eye, T.tgt, [0, 0, 1]));
  }
  // frame the piles and the van (not the whole heightmap: its edges are bare ground and may run off-screen)
  const fr = P.piles.flatMap(p => p.poly);
  if (P.staging) fr.push(P.staging.xy, ...(P.staging.pads || []));
  if (fr.length < 2) fr.push([H.x0, H.y0], [H.x1, H.y1]);
  const fx = fr.map(q => q[0]), fy = fr.map(q => q[1]), M = 5;
  const bx0 = Math.max(H.x0, Math.min(...fx) - M), bx1 = Math.min(H.x1, Math.max(...fx) + M);
  const by0 = Math.max(H.y0, Math.min(...fy) - M), by1 = Math.min(H.y1, Math.max(...fy) + M);
  T.tgt = [(bx0 + bx1) / 2, (by0 + by1) / 2, (H.zlo + H.zhi) * 0.3];
  const corners = [];
  for (const x of [bx0, bx1]) for (const y of [by0, by1]) for (const z of [H.zlo, H.zhi]) corners.push([x, y, z]);
  function extent(az, D) {
    const m = mats(az, D, 0); let x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9;
    for (const c of corners) { const p = proj(m, c[0], c[1], c[2]); x0 = Math.min(x0, p[0]); x1 = Math.max(x1, p[0]); y0 = Math.min(y0, p[1]); y1 = Math.max(y1, p[1]); }
    return { x0, x1, y0, y1 };
  }
  // fit: map spans the screen below the title bar (ndc y from -0.86 to 0.70), for every angle of the sway
  {
    let lo = 5, hi = 2000;
    for (let it = 0; it < 40; it++) {
      const D = (lo + hi) / 2;
      const ok = [T.az0 - T.amp * 0.5, T.az0, T.az0 + T.amp * 0.5].every(a => { const e = extent(a, D); return e.x1 - e.x0 <= 1.88 && e.y1 - e.y0 <= 1.62; });
      ok ? hi = D : lo = D;
    }
    T.D = hi; const e = extent(T.az0, T.D); T.shift = -0.07 - (e.y0 + e.y1) / 2;
  }

  // ---- mouse -> world: march the ray down through the heightmap
  function toWorld(sx, sy, inverse) {
    const nx = sx / W * 2 - 1, ny = 1 - sy / Hs * 2;
    const un = z => { const m = inverse, x = m[0] * nx + m[4] * ny + m[8] * z + m[12], y = m[1] * nx + m[5] * ny + m[9] * z + m[13],
      zz = m[2] * nx + m[6] * ny + m[10] * z + m[14], w = m[3] * nx + m[7] * ny + m[11] * z + m[15]; return [x / w, y / w, zz / w]; };
    const a = un(-1), b = un(1), d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], L = Math.hypot(d[0], d[1], d[2]);
    d[0] /= L; d[1] /= L; d[2] /= L;
    if (d[2] >= -1e-4) return null;
    const zHi = H.zmax + 0.3, zLo = H.zlo;
    const t0 = Math.max(0, (zHi - a[2]) / d[2]), t1 = (zLo - a[2]) / d[2], step = 0.2;
    const at = t => [a[0] + d[0] * t, a[1] + d[1] * t, a[2] + d[2] * t];
    let prev = t0;
    for (let t = t0; t <= t1; t += step) {
      const p = at(t);
      if (H.inside(p[0], p[1]) && p[2] <= H.at(p[0], p[1])) {
        let lo = prev, hi = t;
        for (let k = 0; k < 8; k++) { const m = (lo + hi) / 2, q = at(m); (H.inside(q[0], q[1]) && q[2] <= H.at(q[0], q[1])) ? hi = m : lo = m; }
        return at(hi);
      }
      prev = t;
    }
    const p = at(t1); return H.inside(p[0], p[1]) ? p : null;
  }
  // click / hover -> pile: inside an outline wins, else the nearest centre within SNAP_R
  function pileAt(x, y) {
    let best = null, bd = 1e9;
    for (const p of P.piles) if (inPoly(x, y, p.poly)) { const d = Math.hypot(x - p.xy[0], y - p.xy[1]); if (d < bd) { bd = d; best = p; } }
    if (best) return { pile: best, inside: true, d: bd };
    for (const p of P.piles) { const d = Math.hypot(x - p.xy[0], y - p.xy[1]); if (d < bd) { bd = d; best = p; } }
    return best && bd <= SNAP_R ? { pile: best, inside: false, d: bd } : null;
  }

  const stageXY = e => { const r = cvO.getBoundingClientRect(); return [(e.clientX - r.left) * W / r.width, (e.clientY - r.top) * Hs / r.height]; };
  const sig = { signal: T.ac.signal };
  box.addEventListener('pointerdown', e => {
    if (e.button !== 0 || e.target.closest('.pk-top')) return;
    T.down = { xy: stageXY(e), az: T.userAz, el: T.el }; T.drag = false;
  }, sig);
  window.addEventListener('pointermove', e => {
    const xy = stageXY(e), t = e.target;
    if (!T.down && (!box.contains(t) || (t.closest && t.closest('.pk-top')))) { T.mouse = null; return; }
    T.mouse = xy; T.lastMove = performance.now();
    if (T.down) {
      const dx = xy[0] - T.down.xy[0], dy = xy[1] - T.down.xy[1];
      if (!T.drag && Math.hypot(dx, dy) > 6) { T.drag = true; box.classList.add('drag'); }
      if (T.drag) { T.userAz = T.down.az - dx * 0.25 * D2R; T.el = Math.max(22 * D2R, Math.min(78 * D2R, T.down.el + dy * 0.2 * D2R)); }
    }
  }, sig);
  window.addEventListener('pointerup', e => {
    if (!T.down) return;
    const wasDrag = T.drag; T.down = null; T.drag = false; box.classList.remove('drag');
    if (!wasDrag && !e.target.closest('.pk-top')) click(stageXY(e));
  }, sig);
  box.addEventListener('pointerleave', () => { T.mouse = null; }, sig);
  $('.pk-back').addEventListener('click', () => { if (onBack) onBack(); }, sig);

  function click(xy) {
    if (T.busy) return;
    const w = toWorld(xy[0], xy[1], T.inv);
    const hit = w && pileAt(w[0], w[1]);
    if (!hit) {
      if (!P.piles.length && w) { send(null, [w[0], w[1]], [w[0], w[1]]); return; }
      T.badUntil = performance.now() + 1600;
      tip.className = 'pk-tip bad'; tip.textContent = w ? 'No rubble pile here. Click inside a dashed outline.' : 'Click on the map.';
      return;
    }
    send(hit.pile, hit.pile.xy.slice(), [w[0], w[1]]);
  }
  function send(pile, xy, clickXY) {
    T.busy = true;
    T.pulse = { x: xy[0], y: xy[1], t0: performance.now(), pile };
    const nmP = pile ? pile.id : 'this spot';
    const conf = mk('div', 'pk-conf', `Sending the drones to ${esc(nmP)}<small>${S && S.loading ? 'They take off as soon as Isaac Sim is ready.' : 'They are on their way.'}</small>`);
    box.appendChild(conf);
    hint.innerHTML = `Target: <b>${esc(nmP)}</b>. Click another pile to change it.`;
    st.timers.push(setTimeout(() => { T.busy = false; if (T.onPick) T.onPick({ pile: pile ? pile.id : null, xy, click: clickXY }); }, PICK_DELAY));
    st.timers.push(setTimeout(() => { conf.style.transition = 'opacity .5s'; conf.style.opacity = '0'; }, 3200));
    st.timers.push(setTimeout(() => conf.remove(), 3800));
  }

  // ---- overlay drawing (outlines, labels, van, rings)
  function drawOverlay(m, now) {
    og.clearRect(0, 0, W, Hs);
    const S2 = (x, y, z) => { const p = proj(m, x, y, z); return [(p[0] + 1) / 2 * W, (1 - p[1]) / 2 * Hs, p[2]]; };
    const k = ease3((now - T.t0) / 1400 - 0.2);
    if (k <= 0) return;
    og.globalAlpha = k;
    const hp = T.hoverPile, tp = T.pulse && T.pulse.pile;
    og.lineJoin = 'round';
    for (const p of P.piles) {
      const on = p === hp, sent = p === tp;
      og.beginPath();
      p.drape.forEach((q, i) => { const s = S2(q[0], q[1], q[2] * k); i ? og.lineTo(s[0], s[1]) : og.moveTo(s[0], s[1]); });
      og.closePath();
      if (on || sent) { og.fillStyle = 'rgba(255,210,40,0.07)'; og.fill(); }
      og.setLineDash(on || sent ? [] : [9, 8]);
      og.lineWidth = on || sent ? 3 : 2;
      og.strokeStyle = sent ? 'rgba(255,176,32,0.95)' : on ? 'rgba(255,214,60,0.95)' : 'rgba(255,255,255,0.42)';
      og.stroke();
    }
    og.setLineDash([]);
    // van (staging) marker
    if (P.staging) {
      for (const pd of (P.staging.pads || [])) { const s = S2(pd[0], pd[1], H.at(pd[0], pd[1]) * k + 0.1);
        og.beginPath(); og.arc(s[0], s[1], 5, 0, 7); og.strokeStyle = 'rgba(255,176,32,0.8)'; og.lineWidth = 2; og.stroke(); }
      const sx = P.staging.xy[0], sy = P.staging.xy[1], s = S2(sx, sy, H.at(sx, sy) * k + 0.4);
      og.fillStyle = '#ffb020'; og.strokeStyle = '#000'; og.lineWidth = 2;
      og.beginPath(); og.roundRect ? og.roundRect(s[0] - 13, s[1] - 8, 26, 16, 4) : og.rect(s[0] - 13, s[1] - 8, 26, 16); og.fill(); og.stroke();
      og.font = '700 20px Inter, system-ui, sans-serif'; og.textAlign = 'left'; og.textBaseline = 'middle';
      og.lineWidth = 5; og.strokeStyle = 'rgba(0,0,0,0.85)'; og.strokeText('Van', s[0] + 20, s[1]); og.fillStyle = '#ffb020'; og.fillText('Van', s[0] + 20, s[1]);
    }
    // pick pulse rings, draped on the rubble
    if (T.pulse) {
      const a = (now - T.pulse.t0) / 1000;
      for (let r = 0; r < 3; r++) {
        const u = (a - r * 0.28) / 1.3; if (u < 0 || u > 1) continue;
        const R = 2 + 16 * ease3(u);
        og.beginPath();
        for (let i = 0; i <= 64; i++) { const t = i / 64 * 2 * Math.PI, x = T.pulse.x + R * Math.cos(t), y = T.pulse.y + R * Math.sin(t);
          const s = S2(x, y, H.at(x, y) + 0.2); i ? og.lineTo(s[0], s[1]) : og.moveTo(s[0], s[1]); }
        og.strokeStyle = `rgba(255,190,40,${(1 - u) * 0.95})`; og.lineWidth = 4 * (1 - u) + 1; og.stroke();
      }
    }
    // pile labels
    og.textAlign = 'center'; og.textBaseline = 'middle';
    for (const p of P.piles) {
      const on = p === hp || p === tp, s = S2(p.xy[0], p.xy[1], p.top * k + 1.6);
      og.font = on ? '800 30px Inter, system-ui, sans-serif' : '700 22px Inter, system-ui, sans-serif';
      og.lineWidth = 6; og.strokeStyle = 'rgba(0,0,0,0.85)'; og.strokeText(p.id, s[0], s[1]);
      og.fillStyle = p === tp ? '#ffb020' : on ? '#ffd23c' : '#e6ecf0'; og.fillText(p.id, s[0], s[1]);
    }
    og.globalAlpha = 1;
  }

  // ---- the frame loop
  function frame(now) {
    T.raf = requestAnimationFrame(frame);
    const dt = Math.min(0.05, (now - T.last) / 1000); T.last = now;
    const active = T.drag || (T.mouse && now - T.lastMove < 3000);
    T.orbitV += ((active ? 0 : 1) - T.orbitV) * (1 - Math.exp(-dt * 2.5));
    T.orbitT += dt * T.orbitV;
    const az = T.az0 + T.userAz + T.amp * Math.sin(T.orbitT * 0.14);
    const m = mats(az, T.D, T.shift); T.inv = inv(m); T.m = m;
    // slow machine (no GPU)? after ~2 s, halve the dot count and make the dots a bit bigger
    if (T.factor === 2 && density === 'auto' && T.dots.kind === 'webgl') {
      const pf = T.perf; pf.n++; if (pf.n > 20) pf.t += dt;
      if (pf.n === 80 && pf.t / 60 > 0.036) { T.dots.setPoints(makePoints(P, 1)); T.factor = 1; T.size = 3.6; }
    }
    T.intro = Math.min(1, (now - T.t0) / 1600);
    // hover point (re-marched every frame: the camera moves)
    const w = T.mouse && !T.drag ? toWorld(T.mouse[0], T.mouse[1], T.inv) : null;
    if (w) {
      if (!T.hw || T.hover < 0.02) { T.mx = w[0]; T.my = w[1]; }
      const f = 1 - Math.exp(-dt * 20); T.mx += (w[0] - T.mx) * f; T.my += (w[1] - T.my) * f;
    }
    T.hw = w;
    T.hover += ((w ? 1 : 0) - T.hover) * (1 - Math.exp(-dt * 9));
    const hit = w ? pileAt(w[0], w[1]) : null;
    T.hoverPile = hit ? hit.pile : null;
    // tooltip
    if (now < T.badUntil && T.mouse) { tip.style.left = T.mouse[0] + 'px'; tip.style.top = T.mouse[1] + 'px'; }
    else if (T.pulse && now - T.pulse.t0 < 2600) tip.className = 'pk-tip off';
    else if (T.mouse && !T.drag && w) {
      tip.style.left = T.mouse[0] + 'px'; tip.style.top = T.mouse[1] + 'px';
      if (hit) { tip.className = 'pk-tip'; tip.textContent = hit.inside ? `Pile ${hit.pile.id} · click to send the drones` : `Click to send the drones to ${hit.pile.id}`; }
      else { tip.className = 'pk-tip no'; tip.textContent = P.piles.length ? 'No pile here' : 'Click to send the drones here'; }
    } else tip.className = 'pk-tip off';
    // pulse through the dots
    let pr = 0, pa = 0, px = 0, py = 0;
    if (T.pulse) { const a = (now - T.pulse.t0) / 1000; px = T.pulse.x; py = T.pulse.y;
      if (a < 1.6) { pr = 2 + 18 * ease3(a / 1.6); pa = 1 - a / 1.6; } }
    T.dots.draw({ vp: m, mx: T.mx, my: T.my, hover: T.hover, size: T.size, ref: T.D, intro: T.intro, time: (now - T.t0) / 1000, px, py, pr, pa });
    drawOverlay(m, now);
  }
  T.raf = requestAnimationFrame(frame);
  setLoading(!!loading);
}

function setLoading(on) {
  if (!S) return;
  S.loading = !!on;
  const el = S.root.querySelector('.pk-status'); if (!el) return;
  el.classList.toggle('loading', !!on);
  el.querySelector('span').textContent = on ? 'Isaac Sim is loading this site… pick your spot now' : 'Ready';
}

// status pill with custom text: kind = 'loading' (amber, blinking) | 'ready' (green) | 'bad' (red)
function setStatus(text, kind) {
  if (!S) return;
  S.loading = kind === 'loading';
  const el = S.root.querySelector('.pk-status'); if (!el) return;
  el.classList.toggle('loading', kind === 'loading'); el.classList.toggle('bad', kind === 'bad');
  el.querySelector('span').textContent = text;
}

// test hook: stage-pixel position of a pile centre (or any world xy) in the current topo view
function screenOf(idOrXY) {
  const T = S && S.topo; if (!T || !T.m) return null;
  const H = T.P.H, p = typeof idOrXY === 'string' ? T.P.piles.find(q => q.id === idOrXY) : null;
  const xy = p ? p.xy : idOrXY; if (!xy) return null;
  const q = proj(T.m, xy[0], xy[1], H.at(xy[0], xy[1])), W = S.stage.offsetWidth || 1920, Hs = S.stage.offsetHeight || 1080;
  return [(q[0] + 1) / 2 * W, (1 - q[1]) / 2 * Hs];
}
window.Picker = { start, topo, setLoading, setStatus, close, _prep: prep, _screenOf: screenOf };
})();
