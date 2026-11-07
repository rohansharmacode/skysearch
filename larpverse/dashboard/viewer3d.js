// LiDAR detail view: point cloud cropped around a radar candidate, ping drawn at its depth (nominal or radar estimate).
// Weak (insufficient-evidence) candidates are orange and get no surface marker: they are not a search point yet.
// Site frame is Z-up (x east, y north), so the camera uses z as up.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { CSS2DRenderer, CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';

const CROP = 9; // metres around the candidate
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

  function show({ cloud, center, surf, z, ellipse, id, depthLabel, weak }) {
    group.traverse(o => { if (o.element) o.element.remove(); });
    group.clear();
    const [cx, cy] = center, pos = [], col = [];
    let zmin = Infinity, zmax = -Infinity;
    for (let k = 0; k < cloud.length; k += 3) { const x = cloud[k], y = cloud[k + 1]; if (Math.abs(x - cx) > CROP || Math.abs(y - cy) > CROP) continue; zmin = Math.min(zmin, cloud[k + 2]); zmax = Math.max(zmax, cloud[k + 2]); }
    for (let k = 0; k < cloud.length; k += 3) {
      const x = cloud[k], y = cloud[k + 1], h = cloud[k + 2]; if (Math.abs(x - cx) > CROP || Math.abs(y - cy) > CROP) continue;
      const fade = Math.max(0.25, 1 - Math.hypot(x - cx, y - cy) / (CROP * 1.2));
      const f = Math.max(0, Math.min(0.999, (h - zmin) / Math.max(0.5, zmax - zmin))) * 4, a = Math.floor(f), u = f - a;
      pos.push(x - cx, y - cy, h); for (let c = 0; c < 3; c++) col.push((MAKO[a][c] + (MAKO[a + 1][c] - MAKO[a][c]) * u) * fade);
    }
    const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3)); g.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
    group.add(new THREE.Points(g, new THREE.PointsMaterial({ size: 0.07, vertexColors: true })));

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
