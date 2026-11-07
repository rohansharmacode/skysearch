// LiDAR detail view: point cloud cropped around a radar candidate, ping drawn at its depth (nominal or radar estimate).
// Weak (insufficient-evidence) candidates are orange and get no surface marker: they are not a search point yet.
// Point budget (R-PLAT-4, docs/PLATFORM.md): the crop is drawn at once from the LOD0 preview (pointcloud.bin, <= 300k),
// then, when state.pointcloud.tiles lists full-resolution 16 m tiles, only the tiles that intersect the crop are
// fetched (cached by URL) and replace the preview. No tiles index, or a tile that fails: the preview stays.
// Site frame is Z-up (x east, y north), so the camera uses z as up.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { CSS2DRenderer, CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';

const CROP = 9; // metres around the candidate
const MAX_PTS = 2_500_000, TILE_CACHE = 12;
const tileCache = new Map();          // url -> Promise<Float32Array>, LRU by insertion order
function loadTile(url) {
  if (tileCache.has(url)) { const p = tileCache.get(url); tileCache.delete(url); tileCache.set(url, p); return p; }
  const p = fetch(url, { cache: 'force-cache' }).then(r => { if (!r.ok) throw new Error(r.status); return r.arrayBuffer(); }).then(b => new Float32Array(b));
  p.catch(() => tileCache.delete(url));
  tileCache.set(url, p); while (tileCache.size > TILE_CACHE) tileCache.delete(tileCache.keys().next().value);
  return p;
}
// Crop several float32 xyz arrays to the square around (cx, cy); positions relative to the centre, MAKO colours.
function cropArrays(arrays, cx, cy) {
  let n = 0, zmin = Infinity, zmax = -Infinity;
  for (const a of arrays) for (let k = 0; k < a.length; k += 3) {
    if (Math.abs(a[k] - cx) > CROP || Math.abs(a[k + 1] - cy) > CROP) continue;
    n++; const z = a[k + 2]; if (z < zmin) zmin = z; if (z > zmax) zmax = z;
  }
  const step = Math.max(1, Math.ceil(n / MAX_PTS)), m = Math.ceil(n / step), pos = new Float32Array(m * 3), col = new Float32Array(m * 3);
  let i = 0, seen = 0; const span = Math.max(0.5, zmax - zmin);
  for (const a of arrays) for (let k = 0; k < a.length; k += 3) {
    const x = a[k], y = a[k + 1]; if (Math.abs(x - cx) > CROP || Math.abs(y - cy) > CROP) continue;
    if (seen++ % step || i >= m) continue;
    const h = a[k + 2], fade = Math.max(0.25, 1 - Math.hypot(x - cx, y - cy) / (CROP * 1.2));
    const f = Math.max(0, Math.min(0.999, (h - zmin) / span)) * 4, b = Math.floor(f), u = f - b, o = 3 * i++;
    pos[o] = x - cx; pos[o + 1] = y - cy; pos[o + 2] = h;
    for (let c = 0; c < 3; c++) col[o + c] = (MAKO[b][c] + (MAKO[b + 1][c] - MAKO[b][c]) * u) * fade;
  }
  return { pos: pos.subarray(0, 3 * i), col: col.subarray(0, 3 * i), n: i, total: n, step };
}
function pointsOf(c, size) {
  const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(c.pos, 3)); g.setAttribute('color', new THREE.BufferAttribute(c.col, 3));
  return new THREE.Points(g, new THREE.PointsMaterial({ size, vertexColors: true }));
}
const MAKO = [[0.17, 0.12, 0.28], [0.24, 0.27, 0.55], [0.21, 0.48, 0.64], [0.29, 0.76, 0.68], [0.87, 0.96, 0.9]];

export function createViewer(host) {
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); host.appendChild(renderer.domElement);
  const labels = new CSS2DRenderer(); Object.assign(labels.domElement.style, { position: 'absolute', inset: '0', pointerEvents: 'none' }); host.appendChild(labels.domElement);
  const scene = new THREE.Scene(); scene.background = new THREE.Color(0x070b0f);
  const camera = new THREE.PerspectiveCamera(45, 1, 0.05, 300); camera.up.set(0, 0, 1);
  const controls = new OrbitControls(camera, renderer.domElement); controls.enableDamping = true;
  const group = new THREE.Group(); scene.add(group);
  let raf = 0, t0 = performance.now(), halo = null, ring = null;

  function size() { const w = host.clientWidth, h = host.clientHeight; renderer.setSize(w, h); labels.setSize(w, h); camera.aspect = w / Math.max(1, h); camera.updateProjectionMatrix(); }
  new ResizeObserver(size).observe(host);

  let showTok = 0, pts = null;
  function setPoints(c, size) {
    if (pts) { group.remove(pts); pts.geometry.dispose(); pts.material.dispose(); }
    pts = pointsOf(c, size); group.add(pts);
  }
  function show({ cloud, tiles, base = '.', center, surf, z, ellipse, id, depthLabel, weak, onInfo }) {
    const tok = ++showTok;
    group.traverse(o => { if (o.element) o.element.remove(); o.geometry?.dispose?.(); o.material?.dispose?.(); });
    group.clear(); pts = null;
    const [cx, cy] = center, info = t => { if (tok === showTok) onInfo?.(t); };
    const lod0 = cropArrays([cloud], cx, cy); setPoints(lod0, 0.07);
    const sel = (tiles || []).filter(t => Array.isArray(t.bounds) && t.bounds[0] <= cx + CROP && t.bounds[2] >= cx - CROP && t.bounds[1] <= cy + CROP && t.bounds[3] >= cy - CROP);
    if (!sel.length) info(`LiDAR preview · ${lod0.n.toLocaleString()} points in view${tiles?.length ? ' (no full-resolution tile here)' : ''}`);
    else {
      info(`LiDAR preview · loading ${sel.length} full-resolution tile${sel.length > 1 ? 's' : ''}…`);
      Promise.allSettled(sel.map(t => loadTile(base + '/' + t.url))).then(rs => {
        if (tok !== showTok) return;
        const ok = rs.filter(r => r.status === 'fulfilled').map(r => r.value), bad = rs.length - ok.length;
        if (!ok.length) { info(`LiDAR preview · ${lod0.n.toLocaleString()} points (full-resolution tiles unavailable)`); return; }
        const full = cropArrays(bad ? [...ok, cloud] : ok, cx, cy);
        setPoints(full, full.n > 600000 ? 0.035 : 0.05);
        info(`LiDAR full resolution · ${full.n.toLocaleString()} points from ${ok.length} tile${ok.length > 1 ? 's' : ''}` + (full.step > 1 ? ` (every ${full.step}th of ${full.total.toLocaleString()})` : '') + (bad ? ` · ${bad} tile(s) failed, preview there` : ''));
      });
    }

    const red = weak ? 0xff9f43 : 0xff4545, onTop = { depthTest: false, transparent: true };
    const ping = new THREE.Mesh(new THREE.SphereGeometry(0.22, 20, 14), new THREE.MeshBasicMaterial({ color: red, ...onTop })); ping.position.set(0, 0, z); ping.renderOrder = 10; group.add(ping);
    halo = new THREE.Mesh(new THREE.SphereGeometry(0.22, 20, 14), new THREE.MeshBasicMaterial({ color: red, ...onTop, depthWrite: false })); halo.position.copy(ping.position); halo.renderOrder = 9; group.add(halo);
    const plumb = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, surf + 0.8), new THREE.Vector3(0, 0, z)]), new THREE.LineDashedMaterial({ color: red, dashSize: 0.18, gapSize: 0.12, ...onTop }));
    plumb.computeLineDistances(); plumb.renderOrder = 10; group.add(plumb);
    // 95% region drawn on the surface
    if (ellipse) {
      const pts = []; for (let i = 0; i <= 72; i++) { const u = i / 72 * Math.PI * 2, x = ellipse.rx * Math.cos(u), y = ellipse.ry * Math.sin(u); pts.push(new THREE.Vector3(x * Math.cos(ellipse.ang) - y * Math.sin(ellipse.ang), x * Math.sin(ellipse.ang) + y * Math.cos(ellipse.ang), surf + 0.12)); }
      ring = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: red, ...onTop })); ring.renderOrder = 10; group.add(ring);
    }
    const mk = (html, cls, p) => { const el = document.createElement('div'); el.className = 'lbl ' + cls; el.innerHTML = html; const o = new CSS2DObject(el); o.position.copy(p); group.add(o); };
    mk(`${id} · ${depthLabel}`, weak ? 'weak' : '', ping.position);
    if (!weak) mk(`Surface ${surf.toFixed(1)} m · candidate search point`, 'dig', new THREE.Vector3(0, 0, surf + 0.9));

    controls.target.set(0, 0, (surf + z) / 2);
    camera.position.set(7, -9, surf + 6);
    size(); start();
  }
  function frame() {
    const p = ((performance.now() - t0) / 1100) % 1;
    if (halo) { halo.scale.setScalar(1 + p * 2.6); halo.material.opacity = 0.55 * (1 - p); }
    controls.update(); renderer.render(scene, camera); labels.render(scene, camera); raf = requestAnimationFrame(frame);
  }
  function start() { cancelAnimationFrame(raf); raf = requestAnimationFrame(frame); }
  function stop() { cancelAnimationFrame(raf); }
  return { show, stop };
}
