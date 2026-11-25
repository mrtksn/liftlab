'use strict';
// Onboard cameras. Any rigid mass can carry a camera (c.camera): a video board, an FPV camera, a Pi camera. The view
// shows what the selected drone's camera sees, in an inset at the camera's own resolution and frame rate, from where
// the part is on the airframe (its joints and the drone's attitude included). The camera looks along the part's +X
// (forward), turned by its tilt (+ up, −90° straight down) and pan (+ left); the picture's top is the part's +Z.
// Several cameras: the inset's switch button steps through them, as a two-camera video board's app does.
// The lens: a normal (rectilinear) one keeps straight lines straight, up to 170° across; a fisheye bends them, and
// sees as far as 240° across (a circle of picture with black around it past the lens's edge). Three fisheye
// projections, as lens makers specify them: equidistant (r = f·θ, most FPV and action cameras), equisolid
// (r = 2f·sin θ/2) and stereographic (r = 2f·tan θ/2, less squeezed at the edge). A fisheye picture is made from the
// six views round the camera, mapped through the lens.
// Not modelled: exposure, motion blur, compression and the video link's delay.

const CAMERA_DEFAULTS = Object.freeze({ fov: 90, w: 640, h: 360, fps: 25, tilt: 0, pan: 0, lens: 'rect' });
const CAMERA_LIMITS = { fov: [20, 170], w: [64, 1920], h: [48, 1440], fps: [1, 120], tilt: [-90, 90], pan: [-180, 180] };
const CAMERA_LENSES = { rect: 'Normal (rectilinear)', equidistant: 'Fisheye · equidistant', equisolid: 'Fisheye · equisolid', stereographic: 'Fisheye · stereographic' };
const CAMERA_FISHEYE_FOV = 240;
// The camera's settings, filled in and kept in range (a hand-edited design file may hold anything).
function cameraSpec(c) {
  const o = c && c.camera; if (!o || typeof o !== 'object') return null;
  const s = {}; for (const [k, [lo, hi]] of Object.entries(CAMERA_LIMITS)) s[k] = Number.isFinite(+o[k]) ? clamp(+o[k], lo, hi) : CAMERA_DEFAULTS[k];
  s.lens = CAMERA_LENSES[o.lens] ? o.lens : 'rect';
  if (s.lens !== 'rect' && Number.isFinite(+o.fov)) s.fov = clamp(+o.fov, CAMERA_LIMITS.fov[0], CAMERA_FISHEYE_FOV);
  s.w = Math.round(s.w); s.h = Math.round(s.h); return s;
}
const cameraParts = () => cfg.comps.filter(c => c.type === 'mass' && cameraSpec(c));

const onboard = { pick: null, pose: null, rt: null, rtKey: '', last: -Infinity, quad: null, cam: null, el: null, hidden: [] };

// Where the chosen camera is and which way it looks, for the selected drone: called from fleetScene while that drone's
// state is the current one.
function onboardCamCapture() {
  const list = cameraParts(); onboard.list = list.map(c => ({ id: c.id, name: c.name }));
  let c = list.find(x => x.id === onboard.pick) || list[0];
  if (!c || (typeof onBoard === 'function' && !onBoard(c))) { onboard.pose = null; return; }
  onboard.pick = c.id;
  const s = cameraSpec(c), R = editMode ? [1, 0, 0, 0, 1, 0, 0, 0, 1] : qmat(S.q);
  const P = poseOf(c, editMode && typeof previewAngle === 'function' ? previewAngle : angleTrue);
  const Rc = m3m(m3m(R, m3m(P.R, massRot(c))), eulerR(0, -s.tilt, s.pan));
  onboard.pose = { id: c.id, name: c.name, s, p: add(S.p, m3v(R, P.p)), R: Rc, t: S.t, edit: editMode };
}

function onboardSetup() {
  if (onboard.quad) return;
  onboard.cam = new THREE.PerspectiveCamera(60, 16 / 9, 0.01, 400); onboard.cam.up.set(0, 0, 1);
  const qs = new THREE.Scene(), qc = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), new THREE.MeshBasicMaterial({ depthTest: false, depthWrite: false }));
  qs.add(mesh); onboard.quad = { scene: qs, cam: qc, mesh };
  const box = el('div', { class: 'onboard-cam', hidden: true, 'aria-label': 'Onboard camera' });
  const name = el('span', { class: 'onboard-name' }), meta = el('span', { class: 'onboard-meta' });
  const next = UI.button({ class: 'btn onboard-btn', type: 'button', title: 'Next camera', 'aria-label': 'Next onboard camera', text: '⇄',
    onclick: () => { const L = onboard.list || []; if (L.length < 2) return; const i = L.findIndex(x => x.id === onboard.pick); onboard.pick = L[(i + 1) % L.length].id; onboard.last = -Infinity; } });
  const close = UI.button({ class: 'btn onboard-btn', type: 'button', title: 'Hide the onboard camera (Show → Onboard camera)', 'aria-label': 'Hide onboard camera', text: '×',
    onclick: () => setLayers({ onboard: false }) });
  box.append(el('div', { class: 'onboard-bar' }, name, meta, next, close));
  vpEl.appendChild(box); onboard.el = { box, name, meta, next };
}

// A fisheye picture: the cube of views round the camera, then a pass that turns each pixel into the direction the lens
// sends it (camera axes: x right, y up, looking along −z) and reads the cube there.
const FISHEYE_LENS = { equidistant: 0, equisolid: 1, stereographic: 2 };
function onboardFisheye(s) {
  const face = Math.min(1024, Math.max(256, 2 ** Math.ceil(Math.log2(Math.max(s.w, s.h) * 0.75))));
  if (!onboard.cube || onboard.cubeSize !== face) {
    if (onboard.cube) onboard.cube.rt.dispose();
    const rt = new THREE.WebGLCubeRenderTarget(face, { generateMipmaps: false, minFilter: THREE.LinearFilter, magFilter: THREE.LinearFilter });
    onboard.cube = { rt, cam: new THREE.CubeCamera(0.01, 400, rt) }; onboard.cubeSize = face;
  }
  if (!onboard.fish) {
    const mat = new THREE.ShaderMaterial({ depthTest: false, depthWrite: false,
      uniforms: { tCube: { value: null }, basis: { value: new THREE.Matrix3() }, halfFov: { value: 1 }, aspect: { value: 1 }, lens: { value: 0 } },
      vertexShader: 'varying vec2 vUv; void main(){ vUv = uv; gl_Position = vec4(position.xy, 0.0, 1.0); }',
      fragmentShader: [
        'uniform samplerCube tCube; uniform mat3 basis; uniform float halfFov; uniform float aspect; uniform int lens; varying vec2 vUv;',
        'void main(){',
        '  vec2 p = vec2(vUv.x * 2.0 - 1.0, (vUv.y * 2.0 - 1.0) / aspect); float r = length(p), th;',   // r = 1 at the picture's left and right edges
        '  if (lens == 0) th = r * halfFov;',
        '  else if (lens == 1) { float f = 0.5 / sin(halfFov * 0.5), q = r / (2.0 * f); if (q > 1.0) { gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0); return; } th = 2.0 * asin(q); }',
        '  else { float f = 0.5 / tan(halfFov * 0.5); th = 2.0 * atan(r / (2.0 * f)); }',
        '  if (th > ' + (CAMERA_FISHEYE_FOV / 2 * Math.PI / 180).toFixed(4) + ') { gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0); return; }',   // past the lens
        '  float ph = atan(p.y, p.x); vec3 d = vec3(sin(th) * cos(ph), sin(th) * sin(ph), -cos(th));',
        '  gl_FragColor = textureCube(tCube, basis * d);',
        '}'].join('\n') });
    const sc = new THREE.Scene(); sc.add(new THREE.Mesh(new THREE.PlaneGeometry(2, 2), mat));
    onboard.fish = { scene: sc, mat };
  }
  return onboard.fish;
}

// The inset's place on the canvas [CSS px, x from the left, y from the bottom as WebGL counts]: top right, under the
// view's own buttons (and its title bar), at most a third of the view's width.
function onboardRect(aspect) {
  const W = vpEl.clientWidth, H = vpEl.clientHeight, w = Math.round(clamp(W * 0.3, 160, 360)), h = Math.round(w / aspect);
  const tr = document.querySelector('.hud-tr'), box = vpEl.getBoundingClientRect();
  const top = clamp((tr ? tr.getBoundingClientRect().bottom - box.top : 40) + 26, 26, Math.max(26, H - h - 8));
  return { x: W - w - 8, y: Math.round(H - top - h), w, h, W, H };
}
// Hide the drawing aids (force arrows, markers, trails, rings) while the camera takes its picture: it sees the world
// and the airframe, not the overlays.
function onboardHideAids(hide) {
  if (!hide) { for (const o of onboard.hidden) o.visible = true; onboard.hidden.length = 0; return; }
  const off = o => { if (o && o.visible) { o.visible = false; onboard.hidden.push(o); } };
  off(worldFx); off(cogDot); off(modelRing);
  drone.traverse(o => { if (o !== drone && (o.type === 'ArrowHelper' || o.isLine || o.userData.noPick)) off(o); });
}

function onboardCamRender() {
  const P = onboard.pose, on = view.onboard && P && fleet.selected && !(typeof liveOn === 'function' && liveOn()) && !(typeof bench !== 'undefined' && bench.on);   // (the Computers bench hides it: don't draw it)
  if (onboard.el) onboard.el.box.hidden = !on;
  if (!on) return;
  onboardSetup();
  const s = P.s, aspect = s.w / s.h, key = s.w + 'x' + s.h;
  if (!onboard.rt || onboard.rtKey !== key) {
    if (onboard.rt) onboard.rt.dispose();
    onboard.rt = new THREE.WebGLRenderTarget(s.w, s.h, { minFilter: THREE.LinearFilter, magFilter: THREE.LinearFilter });
    onboard.rtKey = key; onboard.quad.mesh.material.map = onboard.rt.texture; onboard.quad.mesh.material.needsUpdate = true; onboard.last = -Infinity;
  }
  // A new frame when the camera's frame interval has passed in simulated time (or the world restarted, or it's
  // being edited): a slow camera's picture steps, as it does on a real video link.
  if (P.edit || P.t < onboard.last || P.t - onboard.last >= 1 / s.fps - 1e-9 || onboard.lastId !== P.id) {
    const cam = onboard.cam, R = P.R, col = j => new THREE.Vector3(R[j], R[3 + j], R[6 + j]);
    const back = col(0).negate(), up = col(2), right = col(1).negate();   // (camera axes: x right, y up, looking along −z)
    cam.position.set(...P.p); cam.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(right, up, back));
    cam.aspect = aspect; cam.fov = 2 * Math.atan(Math.tan(Math.min(s.fov, 170) * D2R / 2) / aspect) / D2R; cam.updateProjectionMatrix(); cam.updateMatrixWorld();
    onboardHideAids(true);
    try {
      if (s.lens === 'rect') { renderer.setRenderTarget(onboard.rt); renderer.clear(); renderer.render(scene, cam); }
      else {   // a fisheye: the six views round it, then through the lens
        const F = onboardFisheye(s); onboard.cube.cam.position.set(...P.p); onboard.cube.cam.update(renderer, scene);
        const u = F.mat.uniforms; u.tCube.value = onboard.cube.rt.texture; u.basis.value.setFromMatrix4(new THREE.Matrix4().makeBasis(right, up, back));
        u.halfFov.value = s.fov * D2R / 2; u.aspect.value = aspect; u.lens.value = FISHEYE_LENS[s.lens];
        renderer.setRenderTarget(onboard.rt); renderer.clear(); renderer.render(F.scene, onboard.quad.cam);
      }
    } finally { renderer.setRenderTarget(null); onboardHideAids(false); }
    onboard.last = P.t; onboard.lastId = P.id;
  }
  const r = onboardRect(aspect), auto = renderer.autoClear;
  renderer.setScissorTest(true); renderer.setViewport(r.x, r.y, r.w, r.h); renderer.setScissor(r.x, r.y, r.w, r.h);
  renderer.autoClear = false; renderer.render(onboard.quad.scene, onboard.quad.cam); renderer.autoClear = auto;
  renderer.setScissorTest(false); renderer.setViewport(0, 0, r.W, r.H);
  const E = onboard.el, n = (onboard.list || []).length;
  Object.assign(E.box.style, { right: (r.W - r.x - r.w) + 'px', bottom: r.y + 'px', width: r.w + 'px', height: r.h + 'px' });
  setText(E.name, P.name + (n > 1 ? ` (${(onboard.list.findIndex(x => x.id === P.id) + 1)}/${n})` : ''));
  setText(E.meta, `${s.w}×${s.h} · ${s.fps} fps${s.lens === 'rect' ? '' : ' · fisheye'}`); E.box.title = `${P.name}: ${s.w}×${s.h} at ${s.fps} fps, ${Math.round(s.fov)}° across, ${CAMERA_LENSES[s.lens].toLowerCase()} lens`;
  E.next.hidden = n < 2;
}
