'use strict';
// three.js scene: the airframe, force arrows, cable payloads, target marker and trail.

const view = { follow: true, chase: false, forces: true, trail: true, est: true };
const tok = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const vpEl = document.getElementById('viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2)); vpEl.appendChild(renderer.domElement);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(42, 1, 0.02, 200); camera.up.set(0, 0, 1);
scene.add(new THREE.HemisphereLight(0xffffff, 0x667788, 0.85));
const sun = new THREE.DirectionalLight(0xffffff, 0.75); sun.position.set(3, -4, 6); scene.add(sun);
let grid = null; const drone = new THREE.Group(); scene.add(drone);
const worldFx = new THREE.Group(); scene.add(worldFx);
let mats = {}, parts = new Map(), pickGroups = new Map(), pendVis = new Map(), ghost = null, cogDot, modelRing, gravArrow, windArrow, spMarker, trailLine;
const cam = { az: -2.2, el: 0.42, dist: 3.2, target: new THREE.Vector3(0, 0, 1.5) };
const Z = new THREE.Vector3(0, 0, 1);
const colorOf = n => new THREE.Color(tok(n));

function buildMaterials() {
  mats = {
    frame: new THREE.MeshStandardMaterial({ color: colorOf('--frame'), roughness: 0.6, metalness: 0.2 }),
    motor: new THREE.MeshStandardMaterial({ color: colorOf('--frame'), roughness: 0.4, metalness: 0.5 }),
    servo: new THREE.MeshStandardMaterial({ color: colorOf('--mass'), roughness: 0.6 }),
    prop: new THREE.MeshBasicMaterial({ color: colorOf('--accent'), transparent: true, opacity: 0.25, side: THREE.DoubleSide, depthWrite: false }),
    mass: new THREE.MeshStandardMaterial({ color: colorOf('--mass'), roughness: 0.7 }),
    massUnknown: new THREE.MeshStandardMaterial({ color: colorOf('--mass'), roughness: 0.7, transparent: true, opacity: 0.55 }),
    cable: new THREE.LineBasicMaterial({ color: colorOf('--cable') }),
    payload: new THREE.MeshStandardMaterial({ color: colorOf('--cable'), roughness: 0.5 }),
    ink: new THREE.MeshBasicMaterial({ color: colorOf('--ink') }),
    ring: new THREE.LineDashedMaterial({ color: colorOf('--ink-2'), dashSize: 0.012, gapSize: 0.01 }),
    sp: new THREE.LineBasicMaterial({ color: colorOf('--accent'), transparent: true, opacity: 0.7 }),
    trail: new THREE.LineBasicMaterial({ color: colorOf('--muted'), transparent: true, opacity: 0.6 }),
    sensor: new THREE.MeshStandardMaterial({ color: colorOf('--sensor'), roughness: 0.5 }),
    sensorAxis: new THREE.MeshBasicMaterial({ color: colorOf('--sensor') }),
    ghost: new THREE.LineDashedMaterial({ color: colorOf('--sensor'), dashSize: 0.02, gapSize: 0.015, transparent: true, opacity: 0.9 }),
  };
}
function applyTheme() {
  renderer.setClearColor(colorOf('--viewport'), 1);
  if (grid) { scene.remove(grid); grid.geometry.dispose(); }
  grid = new THREE.GridHelper(60, 240, colorOf('--grid-strong'), colorOf('--grid')); grid.rotation.x = Math.PI / 2; scene.add(grid);
  buildMaterials(); buildWorldFx(); rebuildDrone(); buildGizmo();
}
function rod(a, b, r, mat) {
  const va = new THREE.Vector3(...a), vb = new THREE.Vector3(...b); const len = va.distanceTo(vb); if (len < 1e-4) return null;
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, 8), mat); m.position.copy(va).add(vb).multiplyScalar(0.5);
  m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), vb.clone().sub(va).normalize()); return m;
}
function disposeGroup(g) { g.traverse(o => { if (o.geometry) o.geometry.dispose(); }); while (g.children.length) g.remove(g.children[0]); }

function rebuildDrone() {
  disposeGroup(drone); parts = new Map(); pickGroups = new Map();
  drone.add(new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.12, 0.04), mats.frame));
  const nose = rod([0.06, 0, 0], [0.1, 0, 0], 0.006, mats.ink); if (nose) drone.add(nose);
  for (const c of cfg.comps) {
    const r = rod([0, 0, 0], c.pos, 0.007, mats.frame); if (r) drone.add(r);
    if (c.type === 'motor' || c.type === 'tilt') {
      const mount = new THREE.Group(); mount.position.set(...c.pos); mount.userData.compId = c.id; pickGroups.set(c.id, mount); drone.add(mount);
      const axis = new THREE.Group(); mount.add(axis);
      if (c.type === 'tilt') {
        const sv = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.03, 0.022), mats.servo); sv.position.z = -0.018; mount.add(sv);
        const h = hingeAxis(c); const hinge = rod([-h[0] * 0.03, -h[1] * 0.03, -0.018], [h[0] * 0.03, h[1] * 0.03, -0.018], 0.004, mats.ink); if (hinge) mount.add(hinge);
      }
      axis.add(new THREE.Mesh(new THREE.CylinderGeometry(0.017, 0.017, 0.03, 14).rotateX(Math.PI / 2), mats.motor));
      const pr = clamp(0.035 * Math.sqrt(c.tmax), 0.05, 0.2);
      const disc = new THREE.Mesh(new THREE.CircleGeometry(pr, 32), mats.prop.clone()); disc.position.z = 0.02; axis.add(disc);
      const arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0.02), 0.1, colorOf('--accent'), 0.03, 0.018); axis.add(arrow);
      parts.set(c.id, { axis, disc, arrow });
    } else if (c.type === 'mass') {
      let g;
      if (c.shape === 'sphere') g = new THREE.SphereGeometry(c.radius, 20, 14);
      else if (c.shape === 'cylinder') g = new THREE.CylinderGeometry(c.radius, c.radius, c.length, 20).rotateX(Math.PI / 2);
      else g = new THREE.BoxGeometry(...c.size);
      const m = new THREE.Mesh(g, c.known ? mats.mass : mats.massUnknown); m.position.set(...c.pos); m.userData.compId = c.id; pickGroups.set(c.id, m); drone.add(m);
    } else if (c.type === 'hang') {
      const hk = new THREE.Mesh(new THREE.SphereGeometry(0.016, 12, 8), mats.payload); hk.position.set(...c.pos); hk.userData.compId = c.id; pickGroups.set(c.id, hk); drone.add(hk);
    } else if (c.type === 'sensor') {
      const g = new THREE.Group(); g.position.set(...c.pos);
      const Rm = eulerR(c.mount[0], c.mount[1], c.mount[2]);
      g.quaternion.setFromRotationMatrix(new THREE.Matrix4().set(Rm[0], Rm[1], Rm[2], 0, Rm[3], Rm[4], Rm[5], 0, Rm[6], Rm[7], Rm[8], 0, 0, 0, 0, 1));
      const size = { imu: [0.022, 0.022, 0.008], mag: [0.016, 0.016, 0.006], baro: [0.014, 0.014, 0.01], fix: [0.03, 0.03, 0.008] }[c.kind];
      g.add(new THREE.Mesh(new THREE.BoxGeometry(...size), mats.sensor));
      const ax = rod([0, 0, 0], [0.028, 0, 0], 0.0025, mats.sensorAxis); if (ax) g.add(ax);   // sensor X axis shows the mount
      g.userData.compId = c.id; pickGroups.set(c.id, g); drone.add(g);
    }
  }
  buildGhost();
  cogDot = new THREE.Mesh(new THREE.SphereGeometry(0.014, 14, 10), mats.ink); drone.add(cogDot);
  const rg = new THREE.BufferGeometry().setFromPoints(Array.from({ length: 41 }, (_, i) => { const a = i / 40 * Math.PI * 2; return new THREE.Vector3(Math.cos(a) * 0.03, Math.sin(a) * 0.03, 0); }));
  modelRing = new THREE.Line(rg, mats.ring); modelRing.computeLineDistances(); drone.add(modelRing);
  for (const v of pendVis.values()) { worldFx.remove(v.line); worldFx.remove(v.ball); v.line.geometry.dispose(); v.ball.geometry.dispose(); }
  pendVis = new Map();
  for (const c of cfg.comps) if (c.type === 'hang') {
    const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), mats.cable);
    const ball = new THREE.Mesh(new THREE.SphereGeometry(0.025 + 0.035 * Math.cbrt(c.mass), 18, 12), mats.payload);
    worldFx.add(line); worldFx.add(ball); pendVis.set(c.id, { line, ball });
  }
}
function buildGhost() {   // outline of where the flight software thinks the drone is
  if (ghost) { worldFx.remove(ghost); ghost.traverse(o => o.geometry && o.geometry.dispose()); }
  ghost = new THREE.Group(); const pts = [];
  const sq = [[0.06, 0.06], [-0.06, 0.06], [-0.06, -0.06], [0.06, -0.06]];
  for (let i = 0; i < 4; i++) { const a = sq[i], b = sq[(i + 1) % 4]; pts.push(new THREE.Vector3(a[0], a[1], 0), new THREE.Vector3(b[0], b[1], 0)); }
  pts.push(new THREE.Vector3(0.06, 0, 0), new THREE.Vector3(0.14, 0, 0));
  for (const c of actuators()) pts.push(new THREE.Vector3(0, 0, 0), new THREE.Vector3(...c.pos));
  const ls = new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(pts), mats.ghost); ls.computeLineDistances(); ghost.add(ls);
  worldFx.add(ghost);
}
function buildWorldFx() {
  for (const o of [gravArrow, windArrow, spMarker, trailLine]) if (o) { worldFx.remove(o); o.traverse(x => x.geometry && x.geometry.dispose()); }
  gravArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(), 0.2, colorOf('--grav'), 0.035, 0.02); worldFx.add(gravArrow);
  windArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.2, colorOf('--wind'), 0.04, 0.025); worldFx.add(windArrow);
  spMarker = new THREE.Group();
  spMarker.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(Array.from({ length: 49 }, (_, i) => { const a = i / 48 * Math.PI * 2; return new THREE.Vector3(Math.cos(a) * 0.09, Math.sin(a) * 0.09, 0); })), mats.sp));
  spMarker.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, -1)]), mats.sp));
  worldFx.add(spMarker);
  trailLine = new THREE.Line(new THREE.BufferGeometry(), mats.trail); worldFx.add(trailLine);
  for (const v of pendVis.values()) { v.line.material = mats.cable; v.ball.material = mats.payload; }
}
const tmpV = new THREE.Vector3();
function updateScene() {
  const R = qmat(S.q); const { hub } = hubState(R);
  drone.position.set(...hub);
  if (editMode) drone.quaternion.set(0, 0, 0, 1);            // edit in body axes: level, nose along +X
  else drone.quaternion.set(S.q[1], S.q[2], S.q[3], S.q[0]);
  const live = !editMode;
  for (const c of actuators()) {
    const p = parts.get(c.id); if (!p) continue; const st = act.get(c.id);
    p.axis.quaternion.setFromUnitVectors(Z, tmpV.set(...actDir(c, st.th)));   // follows the tilt law exactly
    const T = st.T * c.health / 100; p.disc.material.opacity = 0.12 + 0.4 * clamp(T / c.tmax, 0, 1);
    p.arrow.visible = live && view.forces && T > 0.02; if (p.arrow.visible) p.arrow.setLength(0.04 + T * 0.035, 0.03, 0.018);
  }
  cogDot.position.set(...truth.c); modelRing.position.set(...model.c); modelRing.visible = nrm(sub(truth.c, model.c)) > 0.004;
  gravArrow.visible = live && view.forces; gravArrow.position.set(S.p[0], S.p[1], S.p[2] - 0.02); gravArrow.setLength(0.06 + truth.m * G * 0.02, 0.035, 0.02);
  const wv = windVec(); windArrow.visible = live && view.forces && envr.wind > 0.05;
  if (windArrow.visible) { const u = unit(wv); windArrow.setDirection(new THREE.Vector3(...u)); windArrow.position.set(hub[0] - u[0] * 0.6, hub[1] - u[1] * 0.6, hub[2] + 0.25); windArrow.setLength(0.08 + envr.wind * 0.05, 0.04, 0.025); }
  for (const c of cfg.comps) {
    if (c.type !== 'hang') continue; const v = pendVis.get(c.id), st = pend.get(c.id); if (!v || !st) continue;
    v.line.visible = v.ball.visible = live;
    const aw = add(S.p, m3v(R, sub(c.pos, truth.c))); const pos = v.line.geometry.attributes.position;
    pos.setXYZ(0, ...aw); pos.setXYZ(1, ...st.p); pos.needsUpdate = true; v.line.geometry.computeBoundingSphere(); v.ball.position.set(...st.p);
  }
  ghost.visible = live && view.est; if (ghost.visible) { ghost.position.set(...est.p); ghost.quaternion.set(est.q[1], est.q[2], est.q[3], est.q[0]); }
  spMarker.visible = live; spMarker.position.set(setpoint.x, setpoint.y, setpoint.z); spMarker.children[1].scale.z = setpoint.z;
  trailLine.visible = live && view.trail;
  if (view.trail && trail.length > 1) { trailLine.geometry.dispose(); trailLine.geometry = new THREE.BufferGeometry().setFromPoints(trail.map(p => new THREE.Vector3(...p))); }
  const tgt = view.follow || editMode ? new THREE.Vector3(...hub) : new THREE.Vector3(setpoint.x, setpoint.y, setpoint.z);
  cam.target.lerp(tgt, view.follow ? 0.12 : 0.06);
  if (view.chase && live) {  // swing the camera behind the target heading
    let d = setpoint.yaw * D2R + Math.PI - cam.az; d = Math.atan2(Math.sin(d), Math.cos(d));
    cam.az += d * 0.06;
  }
  const ce = Math.cos(cam.el);
  camera.position.set(cam.target.x + cam.dist * ce * Math.cos(cam.az), cam.target.y + cam.dist * ce * Math.sin(cam.az), cam.target.z + cam.dist * Math.sin(cam.el));
  camera.lookAt(cam.target);
  updateEditView();
}

// Orbit and zoom: drag to rotate, wheel or pinch to zoom.
const ptrs = new Map(); let pinch0 = 0;
vpEl.addEventListener('pointerdown', e => { if (editPointerDown(e)) return; vpEl.setPointerCapture(e.pointerId); ptrs.set(e.pointerId, { x: e.clientX, y: e.clientY }); if (ptrs.size === 2) { const [a, b] = [...ptrs.values()]; pinch0 = Math.hypot(a.x - b.x, a.y - b.y); } });
vpEl.addEventListener('pointermove', e => {
  if (editPointerMove(e, ptrs.size > 0 && edit.down && Math.hypot(e.clientX - edit.down.x, e.clientY - edit.down.y) >= 5)) return;
  if (!ptrs.has(e.pointerId)) return; const p = ptrs.get(e.pointerId);
  if (ptrs.size === 1) { cam.az -= (e.clientX - p.x) * 0.008; cam.el = clamp(cam.el + (e.clientY - p.y) * 0.006, -0.2, 1.45); }
  p.x = e.clientX; p.y = e.clientY;
  if (ptrs.size === 2) { const [a, b] = [...ptrs.values()]; const d = Math.hypot(a.x - b.x, a.y - b.y); if (pinch0 > 0) cam.dist = clamp(cam.dist * pinch0 / d, 0.6, 20); pinch0 = d; }
});
const endPtr = e => { const handled = e.type === 'pointerup' && editPointerUp(e); ptrs.delete(e.pointerId); pinch0 = 0; return handled; };
vpEl.addEventListener('pointerleave', () => { if (!edit.drag) setHover(null); });
vpEl.addEventListener('pointerup', endPtr); vpEl.addEventListener('pointercancel', endPtr);
vpEl.addEventListener('wheel', e => { e.preventDefault(); cam.dist = clamp(cam.dist * Math.exp(e.deltaY * 0.001), 0.6, 20); }, { passive: false });
new ResizeObserver(() => { const w = vpEl.clientWidth, h = vpEl.clientHeight; if (!w || !h) return; renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix(); }).observe(vpEl);
