// Thin re-export so the dashboard resolves three.js through dashboard/vendor/ (index.html importmap).
// OFFLINE (venue Wi-Fi): replace this file and the two addon shims with the real three@0.160.0 files:
//   curl -o dashboard/vendor/three.module.js https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js
//   curl -o dashboard/vendor/addons/controls/OrbitControls.js https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/controls/OrbitControls.js
//   curl -o dashboard/vendor/addons/renderers/CSS2DRenderer.js https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/renderers/CSS2DRenderer.js
// (the real addon files import from "three", which the importmap maps back to this folder). No other change needed.
export * from "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/controls/OrbitControls.js";
