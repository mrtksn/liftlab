'use strict';
// three.js scene: the airframe, force arrows, torque arcs, cable payloads, target marker and trail.

const view = { follow: true, chase: false };
// What the view draws: each overlay on its own switch (the Show menu, ui.js). The defaults are what a new viewer
// sees; their choice is remembered in their browser.
const LAYERS = [
  { key: 'thrust', label: 'Thrust', group: 'Forces', on: true, tip: 'Each rotor\'s thrust, along its axis' },
  { key: 'weight', label: 'Weight', group: 'Forces', on: true, tip: 'Gravity at the true centre of mass' },
  { key: 'wind', label: 'Wind', group: 'Forces', on: true, tip: 'Wind direction and strength' },
  { key: 'lift', label: 'Wing lift', group: 'Forces', on: true, tip: 'Each wing\'s lift: across the air past it, from where it acts' },
  { key: 'drag', label: 'Wing drag', group: 'Forces', on: true, tip: 'Each wing\'s drag: along the air past it, from where it acts' },
  { key: 'rtorque', label: 'Rotor torque', group: 'Torque', on: false, tip: 'Each rotor\'s reaction torque on the frame, opposite to its spin: what makes the drone yaw (Q toggles torque)' },
  { key: 'ntorque', label: 'Net torque', group: 'Torque', on: false, tip: 'Everything turning the drone about its centre of mass, smoothed (Q toggles torque)' },
  { key: 'want', label: 'Wanted torque', group: 'Torque', on: false, tip: 'What the controller asked for, to compare with the net torque' },
  { key: 'spin', label: 'Prop spin', group: 'Airframe', on: true, tip: 'Which way each prop turns' },
  { key: 'servo', label: 'Servo range', group: 'Airframe', on: true, tip: 'The swing range of servos that steer a rotor' },
  { key: 'cog', label: 'Centre of mass', group: 'Airframe', on: true, tip: 'The true centre of mass (dot) and where the controller thinks it is (ring)' },
  { key: 'beam', label: 'Sensor beams', group: 'Airframe', on: true, tip: 'Rangefinder beams' },
  { key: 'air', label: 'Airflow', group: 'Airframe', on: false, tip: 'The rotor wakes' },
  { key: 'trail', label: 'Trail', group: 'Flight', on: true, tip: 'The path flown' },
  { key: 'est', label: 'Estimate', group: 'Flight', on: true, tip: 'Where the flight software thinks the drone is' },
  { key: 'target', label: 'Target', group: 'Flight', on: true, tip: 'The position the drone is flying to' },
  { key: 'heading', label: 'Forward', group: 'Flight', on: true, tip: 'A level arrow beside the drone: the way the forward key (and stick) moves it' },
  { key: 'grid', label: 'Ground grid', group: 'Scene', on: true, tip: 'The grid on the ground' },
  { key: 'shadow', label: 'Shadow', group: 'Scene', on: true, tip: 'A shadow on whatever is under the drone, to judge its height' },
  { key: 'viewctl', label: 'View controls', group: 'Scene', on: true, tip: 'The axes and the camera buttons, top right (Persp/Ortho, Top, Front, Side, Iso)' },
  { key: 'readouts', label: 'Readouts', group: 'Scene', on: true, tip: 'Position, speed and torque numbers, top left' },
  { key: 'legend', label: 'Legend', group: 'Scene', on: true, tip: 'The colour key, top left' },
];
for (const L of LAYERS) view[L.key] = L.on;
const tok = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const vpEl = document.getElementById('viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2)); vpEl.appendChild(renderer.domElement);
const scene = new THREE.Scene();
// Two cameras on the same orbit: perspective, and orthographic (no foreshortening, for lining parts up). The
// orthographic view is sized to frame what the perspective one shows at the orbit's centre.
const perspCam = new THREE.PerspectiveCamera(42, 1, 0.02, 200); perspCam.up.set(0, 0, 1);
const orthoCam = new THREE.OrthographicCamera(-1, 1, 1, -1, -60, 260); orthoCam.up.set(0, 0, 1);
let camera = perspCam;
scene.add(new THREE.HemisphereLight(0xffffff, 0x667788, 0.85));
const sun = new THREE.DirectionalLight(0xffffff, 0.75); sun.position.set(3, -4, 6); scene.add(sun);
let grid = null; let drone = new THREE.Group(); scene.add(drone);
let worldFx = new THREE.Group(); scene.add(worldFx);
let headArrow = null, mats = {}, rangeVis = new Map(), jointGroups = new Map(), parts = new Map(), pickGroups = new Map(), pendVis = new Map(), ghost = null, cogDot, modelRing, gravArrow, windArrow, spMarker, trailLine;
let tqNetGlyph = null, tqWantArrow = null, tqRotor = [];   // torque: net about the centre of mass, the controller's wish, per rotor
const cam = { az: -2.2, el: 0.42, dist: 3.2, target: new THREE.Vector3(0, 0, 1.5), pan: new THREE.Vector3(), anim: null };
const EL_MAX = Math.PI / 2 - 0.002;
// How far away something looks, for sizing handles to the screen (the orbit distance, in orthographic).
const viewDist = p => camera.isOrthographicCamera ? cam.dist : camera.position.distanceTo(p);
function setProjection(kind) {
  const next = kind === 'ortho' ? orthoCam : perspCam; if (next === camera) return;
  camera = next; syncViewUi();
}
// Named views, relative to the drone: its heading while flying, body axes while editing (the same thing).
const VIEWS = {
  front: { az: 0, el: 0, label: 'Front' }, back: { az: Math.PI, el: 0, label: 'Back' },
  left: { az: Math.PI / 2, el: 0, label: 'Left' }, right: { az: -Math.PI / 2, el: 0, label: 'Right' },
  top: { az: Math.PI, el: EL_MAX, label: 'Top' }, bottom: { az: Math.PI, el: -EL_MAX, label: 'Bottom' },
  iso: { az: -Math.PI / 4, el: Math.atan(1 / Math.SQRT2), label: 'Iso' },
};
function droneYaw() { if (editMode) return 0; const R = qmat(S.q); return Math.atan2(R[3], R[0]); }
function snapView(name) {   // glide to a named view over a quarter second
  const v = VIEWS[name]; if (!v) return;
  const az1 = v.az + droneYaw(); let d = az1 - cam.az; d = Math.atan2(Math.sin(d), Math.cos(d));
  cam.anim = { az0: cam.az, el0: cam.el, daz: d, el1: v.el, t0: performance.now(), dur: 280 };
  if (view.chase) { view.chase = false; const b = document.getElementById('tChase'); if (b) b.setAttribute('aria-pressed', 'false'); }
}
function stepCamAnim() {
  const a = cam.anim; if (!a) return;
  const u = clamp((performance.now() - a.t0) / a.dur, 0, 1), k = u * u * (3 - 2 * u);
  cam.az = a.az0 + a.daz * k; cam.el = a.el0 + (a.el1 - a.el0) * k;
  if (u >= 1) cam.anim = null;
}
const Z = new THREE.Vector3(0, 0, 1);
const colorOf = n => new THREE.Color(tok(n));

const heatCol = {};
function buildMaterials() {
  heatCol.warn = colorOf('--warn'); heatCol.bad = colorOf('--bad'); heatCol.swing = colorOf('--swing');
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
    beam: new THREE.LineDashedMaterial({ color: colorOf('--sensor'), dashSize: 0.03, gapSize: 0.02 }),
    wake: new THREE.MeshBasicMaterial({ color: colorOf('--wind'), transparent: true, opacity: 0.18, side: THREE.DoubleSide, depthWrite: false }),
    sensor: new THREE.MeshStandardMaterial({ color: colorOf('--sensor'), roughness: 0.5 }),
    sensorAxis: new THREE.MeshBasicMaterial({ color: colorOf('--sensor') }),
    spin: new THREE.LineBasicMaterial({ color: colorOf('--ink-2'), transparent: true, opacity: 0.4 }),
    spinHead: new THREE.MeshBasicMaterial({ color: colorOf('--ink-2'), transparent: true, opacity: 0.4, side: THREE.DoubleSide, depthWrite: false }),
    ghost: new THREE.LineDashedMaterial({ color: colorOf('--sensor'), dashSize: 0.02, gapSize: 0.015, transparent: true, opacity: 0.9 }),
    bldg: new THREE.MeshStandardMaterial({ color: colorOf('--bldg'), roughness: 0.95, polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1 }),
    bldgFade: new THREE.MeshStandardMaterial({ color: colorOf('--bldg'), roughness: 0.95, transparent: true, opacity: 0.14, depthWrite: false }),
    bldgEdge: new THREE.LineBasicMaterial({ color: colorOf('--bldg-edge') }),
    bldgEdgeFade: new THREE.LineBasicMaterial({ color: colorOf('--bldg-edge'), transparent: true, opacity: 0.3, depthWrite: false }),
    shadow: new THREE.MeshBasicMaterial({ color: colorOf('--ink'), transparent: true, opacity: 0.2, depthWrite: false }),
    nose: new THREE.MeshBasicMaterial({ color: colorOf('--ax-x'), side: THREE.DoubleSide }),
    heading: new THREE.MeshBasicMaterial({ color: colorOf('--ax-x'), transparent: true, opacity: 0.6, depthWrite: false, side: THREE.DoubleSide }),
    stub: new THREE.MeshBasicMaterial({ color: colorOf('--bad') }),
    cargo: new THREE.MeshStandardMaterial({ color: colorOf('--cargo'), roughness: 0.8 }),
    wing: new THREE.MeshStandardMaterial({ color: colorOf('--wing'), roughness: 0.5, metalness: 0.1 }),
    reachOk: new THREE.LineDashedMaterial({ color: colorOf('--good'), dashSize: 0.02, gapSize: 0.012, depthTest: false, transparent: true }),
    reachFar: new THREE.LineDashedMaterial({ color: colorOf('--muted'), dashSize: 0.02, gapSize: 0.012, depthTest: false, transparent: true, opacity: 0.8 }),
    ringOk: new THREE.MeshBasicMaterial({ color: colorOf('--good'), side: THREE.DoubleSide, transparent: true, depthTest: false }),
    ringFar: new THREE.MeshBasicMaterial({ color: colorOf('--muted'), side: THREE.DoubleSide, transparent: true, opacity: 0.8, depthTest: false }),
  };
}
function applyTheme() {
  renderer.setClearColor(colorOf('--viewport'), 1);
  buildMaterials(); buildWorldFx(); rebuildDrone(); buildGizmo(); buildCity();
}

/* ───────── the world: ground grid, buildings, shadow ───────── */
const city = new THREE.Group(); scene.add(city);
let cityVer = -1, bldg = [], shadowMesh = null;
const bigWorld = () => terrain.scale > 2;
const maxDist = () => bigWorld() ? 300 : 20;
function buildCity() {
  if (grid) { scene.remove(grid); grid.geometry.dispose(); }
  const big = bigWorld();   // a full-scale city gets a coarser, wider grid and a longer view
  grid = new THREE.GridHelper(big ? 1200 : 60, big ? 240 : 240, colorOf('--grid-strong'), colorOf('--grid')); grid.rotation.x = Math.PI / 2; scene.add(grid);
  perspCam.near = big ? 0.05 : 0.02; perspCam.far = big ? 5000 : 200; perspCam.updateProjectionMatrix();
  orthoCam.near = big ? -3000 : -60; orthoCam.far = big ? 5000 : 260; orthoCam.updateProjectionMatrix();
  cam.dist = Math.min(cam.dist, maxDist());
  city.traverse(o => { if (o.geometry) o.geometry.dispose(); }); while (city.children.length) city.remove(city.children[0]);
  bldg = [];
  for (const b of terrain.boxes) {
    const geo = new THREE.BoxGeometry(b.hi[0] - b.lo[0], b.hi[1] - b.lo[1], b.hi[2] - b.lo[2]);
    const m = new THREE.Mesh(geo, mats.bldg); m.position.set((b.lo[0] + b.hi[0]) / 2, (b.lo[1] + b.hi[1]) / 2, (b.lo[2] + b.hi[2]) / 2);
    const e = new THREE.LineSegments(new THREE.EdgesGeometry(geo), mats.bldgEdge); m.add(e);
    city.add(m); bldg.push({ m, e, b, faded: false });
  }
  shadowMesh = new THREE.Mesh(new THREE.CircleGeometry(1, 36), mats.shadow); shadowMesh.renderOrder = 1; city.add(shadowMesh);
  cityVer = terrain.ver;
}
// Does the segment a→b pass through box b? (slab test)
function segHitsBox(a, d, len, b) {
  let t0 = 0, t1 = len;
  for (let i = 0; i < 3; i++) {
    if (Math.abs(d[i]) < 1e-12) { if (a[i] < b.lo[i] || a[i] > b.hi[i]) return false; continue; }
    let u = (b.lo[i] - a[i]) / d[i], v = (b.hi[i] - a[i]) / d[i]; if (u > v) [u, v] = [v, u];
    t0 = Math.max(t0, u); t1 = Math.min(t1, v); if (t0 > t1) return false;
  }
  return true;
}
function updateCity(hub) {
  if (cityVer !== terrain.ver) buildCity();
  // Buildings between the camera and the drone (or the point the camera looks at) turn see-through.
  const c = camera.position.toArray(), tg = cam.target.toArray();
  const rays = [hub, tg].map(p => { const d = sub(p, c), L = nrm(d); return { d: scl(d, 1 / Math.max(L, 1e-9)), L }; });
  for (const B of bldg) {
    const fade = rays.some(r => segHitsBox(c, r.d, r.L, B.b));
    if (fade !== B.faded) { B.faded = fade; B.m.material = fade ? mats.bldgFade : mats.bldg; B.e.material = fade ? mats.bldgEdgeFade : mats.bldgEdge; }
  }
  // The shadow: a soft disc on whatever is right under the drone, fainter the higher it flies.
  const show = view.shadow && !editMode;
  shadowMesh.visible = show;
  if (show) {
    const z = surfaceBelow(hub), h = hub[2] - z, r = cReach * 0.8;
    shadowMesh.position.set(hub[0], hub[1], z + 0.004 * (bigWorld() ? 5 : 1)); shadowMesh.scale.setScalar(r * (1 + h * 0.04));
    shadowMesh.material.opacity = 0.28 * clamp(1 - h / (bigWorld() ? 60 : 8), 0.08, 1);
  }
}
// A wing as it looks: a cambered airfoil (NACA 2-4-xx: 2% camber at 40% of the chord, its thickness from the wing's),
// leading edge toward +X, stretched along the span (Y), centred on the part.
function airfoilGeo(chord, span, thick) {
  const t = clamp(thick / chord, 0.04, 0.24), m = 0.02, p = 0.4, N = 28, up = [], lo = [];
  for (let i = 0; i <= N; i++) {
    const x = (1 - Math.cos(Math.PI * i / N)) / 2;                // more points near the leading and trailing edges
    const yt = 5 * t * (0.2969 * Math.sqrt(x) - 0.126 * x - 0.3516 * x * x + 0.2843 * x ** 3 - 0.1036 * x ** 4);
    const yc = x < p ? m / (p * p) * (2 * p * x - x * x) : m / ((1 - p) ** 2) * (1 - 2 * p + 2 * p * x - x * x);
    const dy = x < p ? 2 * m / (p * p) * (p - x) : 2 * m / ((1 - p) ** 2) * (p - x), th = Math.atan(dy);
    up.push([x - yt * Math.sin(th), yc + yt * Math.cos(th)]); lo.push([x + yt * Math.sin(th), yc - yt * Math.cos(th)]);
  }
  const zMid = (Math.max(...up.map(q => q[1])) + Math.min(...lo.map(q => q[1]))) / 2;   // (centred on its thickness)
  const sh = new THREE.Shape(), P = q => [chord * (0.5 - q[0]), chord * (q[1] - zMid)];
  sh.moveTo(...P(up[N])); for (let i = N - 1; i >= 0; i--) sh.lineTo(...P(up[i])); for (let i = 1; i <= N; i++) sh.lineTo(...P(lo[i]));
  const g = new THREE.ExtrudeGeometry(sh, { depth: span, bevelEnabled: false, curveSegments: 1 });
  g.rotateX(Math.PI / 2); g.translate(0, span / 2, 0); g.computeVertexNormals();
  return g;
}
const m4of = R => new THREE.Matrix4().set(R[0], R[1], R[2], 0, R[3], R[4], R[5], 0, R[6], R[7], R[8], 0, 0, 0, 0, 1);
function rod(a, b, r, mat) {
  const va = new THREE.Vector3(...a), vb = new THREE.Vector3(...b); const len = va.distanceTo(vb); if (len < 1e-4) return null;
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, 8), mat); m.position.copy(va).add(vb).multiplyScalar(0.5);
  m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), vb.clone().sub(va).normalize()); return m;
}
function disposeGroup(g) { g.traverse(o => { if (o.geometry) o.geometry.dispose(); }); while (g.children.length) g.remove(g.children[0]); }

// What a servo can sweep, drawn while editing: a fan in its plane of motion out to the farthest part it
// carries, its limits either side of 0° (where the parts sit now), the path each carried part would trace,
// and the hinge axis. rest: the direction the fan is centred on (towards what it carries, or a carried
// rotor's thrust when that sits on the pivot).
function servoSweep(j) {
  const a = jointAxis(j), radial = q => { const r = sub(q, j.pos); return sub(r, scl(a, dot(r, a))); };
  const pts = [];
  for (const c of descendants(j)) { pts.push(c.pos); if (c.type === 'link') pts.push(linkTip(c)); }
  const arcs = pts.map(q => { const r = radial(q); return { r, h: sub(sub(q, j.pos), r) }; }).filter(x => nrm(x.r) > 0.008);
  const rs = arcs.map(x => x.r);
  const cd = carriedDir(j); let rest = sub(cd, scl(a, dot(cd, a)));
  if (nrm(rest) < 0.05) rest = crs(a, Math.abs(a[2]) > 0.9 ? [0, 1, 0] : [0, 0, 1]);
  const props = descendants(j).filter(c => c.type === 'motor').map(c => nrm(radial(c.pos)) + propR(c));   // a rotor's disc swings out to its rim
  const radius = clamp(Math.max(0, ...rs.map(nrm), ...props) || 0.08, 0.07, 0.45);
  return { a, rest: unit(rest), radius, arcs };
}
function buildRangeVis(j, p) {
  const { a, rest, radius, arcs } = servoSweep(j), v = crs(a, rest), R = j.range * D2R, N = 36;
  const at = (rad, th, off = [0, 0, 0]) => new THREE.Vector3(...add(off, add(scl(rest, rad * Math.cos(th)), scl(v, rad * Math.sin(th)))));
  const col = colorOf('--swing');   // servo travel has its own colour, apart from the rotors' blue
  const m = {
    fan: new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.1, side: THREE.DoubleSide, depthWrite: false }),
    line: new THREE.LineBasicMaterial({ color: col, transparent: true, opacity: 0.5 }),
    dash: new THREE.LineDashedMaterial({ color: col, dashSize: 0.012, gapSize: 0.008, transparent: true, opacity: 0.6 }),
  };
  const g = new THREE.Group(); g.position.set(...p); g.visible = false; g.userData.noPick = true;
  const fan = [], edge = [];
  for (let i = 0; i <= N; i++) edge.push(at(radius, -R + 2 * R * i / N));
  for (let i = 0; i < N; i++) fan.push(new THREE.Vector3(), edge[i], edge[i + 1]);
  g.add(new THREE.Mesh(new THREE.BufferGeometry().setFromPoints(fan), m.fan));
  g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([at(radius, -R), new THREE.Vector3(), at(radius, R)]), m.line));   // the two limits
  g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(edge), m.line));
  const mid = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), at(radius * 1.08, 0)]), m.dash); mid.computeLineDistances(); g.add(mid);   // 0°
  const axis = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(...scl(a, -0.06)), new THREE.Vector3(...scl(a, 0.06))]), m.dash); axis.computeLineDistances(); g.add(axis);
  for (const { r, h } of arcs) {   // the path each carried part would trace
    const n = nrm(r), u = unit(r), w = crs(a, u), path = [];
    for (let i = 0; i <= N; i++) { const th = -R + 2 * R * i / N; path.push(new THREE.Vector3(...add(h, add(scl(u, n * Math.cos(th)), scl(w, n * Math.sin(th)))))); }
    const l = new THREE.Line(new THREE.BufferGeometry().setFromPoints(path), m.dash); l.computeLineDistances(); g.add(l);
  }
  return { g, m };
}
// Which way a prop turns, seen facing the prop: two faint arcs on the disc, each with a small arrowhead.
function spinMarks(pr, spin) {
  const g = new THREE.Group(), r = 0.72 * pr, span = 1.6, N = 20, h = Math.min(0.012, 0.18 * pr);
  g.position.z = 0.0205; g.scale.x = spin >= 0 ? 1 : -1;   // mirrored for clockwise
  for (const a0 of [0, Math.PI]) {
    const pts = Array.from({ length: N + 1 }, (_, i) => { const a = a0 + span * i / N; return new THREE.Vector3(r * Math.cos(a), r * Math.sin(a), 0); });
    g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), mats.spin));
    const a = a0 + span, t = [-Math.sin(a), Math.cos(a)], n = [Math.cos(a), Math.sin(a)], tip = [r * n[0] + h * t[0], r * n[1] + h * t[1]];
    const head = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(tip[0], tip[1], 0),
      new THREE.Vector3(r * n[0] + 0.5 * h * n[0], r * n[1] + 0.5 * h * n[1], 0), new THREE.Vector3(r * n[0] - 0.5 * h * n[0], r * n[1] - 0.5 * h * n[1], 0)]);
    g.add(new THREE.Mesh(head, mats.spinHead));
  }
  return g;
}
// What the view draws on the drone: the design while editing, what's on board now while flying (a load dropped or
// picked up, cargo.js). Rebuilt when that changes.
let droneShown = '';
const viewComps = () => typeof editMode !== 'undefined' && editMode ? cfg.comps : liveComps();
function rebuildDrone() {
  droneShown = cargo.rev + ':' + (typeof editMode !== 'undefined' && editMode);
  const comps = viewComps();
  disposeGroup(drone); parts = new Map(); pickGroups = new Map(); jointGroups = new Map(); rangeVis = new Map();
  { const d = frameDims(), fm = new THREE.Mesh(frameWing() ? airfoilGeo(d[0], d[1], d[2]) : new THREE.BoxGeometry(...d), mats.frame); fm.setRotationFromMatrix(m4of(frameRot())); drone.add(fm); }   // the hub, or the body as a wing
  {   // the front: a red arrow on the hub (red as the X axis), pointing forward
    const d = frameDims(), hx = d[0] / 2, top = (frameWing() ? d[2] * 0.6 : d[2] / 2) + 0.002, L = Math.max(0.05, hx * 0.9), w = Math.min(0.03, Math.max(0.016, d[1] * 0.18));
    const sh = new THREE.Shape(); sh.moveTo(hx + 0.03, 0); sh.lineTo(hx + 0.03 - L, w); sh.lineTo(hx + 0.03 - L * 0.62, 0); sh.lineTo(hx + 0.03 - L, -w); sh.closePath();
    const nose = new THREE.Mesh(new THREE.ShapeGeometry(sh), mats.nose); nose.position.z = top; nose.userData.noPick = true; drone.add(nose);
    const tip = rod([hx, 0, 0], [hx + 0.035, 0, 0], 0.005, mats.nose); if (tip) { tip.userData.noPick = true; drone.add(tip); }
  }
  // Parts on a servo joint live inside that joint's group, which turns about the hinge; nested joints nest.
  const js = comps.filter(c => c.type === 'joint').sort((a, b) => chainOf(a).length - chainOf(b).length);
  const holder = c => { const j = parentJoint(c); return j ? { g: jointGroups.get(j.id), o: j.pos } : { g: drone, o: [0, 0, 0] }; };
  // A part's connector starts where it hangs from: a rod's tip, a joint's pivot, or the hub.
  const rel = c => { const h = holder(c), par = parentOf(c); return { g: h.g, p: sub(c.pos, h.o), from: par && par.type === 'link' ? sub(linkTip(par), h.o) : [0, 0, 0] }; };
  for (const j of js) {
    const { g, p, from } = rel(j);
    const r = rod(from, p, 0.007, mats.frame); if (r) { r.userData.compId = j.id; g.add(r); }   // clicking the arm a part hangs on picks the part
    // The servo case, fixed to what it's mounted on: its output shaft on the hinge axis, the case behind it
    // (away from the load). The horn on the output points at the load and turns with it.
    const a = jointAxis(j), rest = servoSweep(j).rest, basis = new THREE.Matrix4().makeBasis(new THREE.Vector3(...rest), new THREE.Vector3(...crs(a, rest)), new THREE.Vector3(...a));
    const body = new THREE.Group(); body.position.set(...p); body.quaternion.setFromRotationMatrix(basis);
    const box = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.02, 0.036), mats.servo); box.position.set(-0.01, 0, -0.022); body.add(box);
    const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.012, 12).rotateX(Math.PI / 2), mats.ink); shaft.position.z = -0.002; body.add(shaft);
    body.userData.compId = j.id; pickGroups.set(j.id, body); g.add(body);
    const jg = new THREE.Group(); jg.position.set(...p); g.add(jg); jointGroups.set(j.id, jg);   // the output: turns with the joint
    const rv = buildRangeVis(j, p); g.add(rv.g); rangeVis.set(j.id, rv);
    const horn = new THREE.Mesh(new THREE.BoxGeometry(0.036, 0.008, 0.003), mats.ink); horn.position.set(0.012, 0, 0.004);
    const hg = new THREE.Group(); hg.quaternion.setFromRotationMatrix(basis); hg.add(horn); horn.userData.compId = j.id; jg.add(hg);
  }
  for (const c of comps) {
    if (c.type === 'joint') continue;
    const { g, p, from } = rel(c);
    const r = rod(from, p, 0.007, mats.frame); if (r) { r.userData.compId = c.id; g.add(r); }   // clicking the arm a part hangs on picks the part
    if (c.type === 'link') {   // the rod itself, base to tip, with a knob at the tip where things attach
      const lg = new THREE.Group(); lg.userData.compId = c.id; pickGroups.set(c.id, lg); g.add(lg);
      const tip = add(p, scl(linkDir(c), c.length));
      const bar = rod(p, tip, 0.006, mats.link || mats.servo); if (bar) lg.add(bar);
      const knob = new THREE.Mesh(new THREE.SphereGeometry(0.009, 12, 8), mats.ink); knob.position.set(...tip); lg.add(knob);
      continue;
    }
    if (c.type === 'motor') {
      const mount = new THREE.Group(); mount.position.set(...p); mount.userData.compId = c.id; pickGroups.set(c.id, mount); g.add(mount);
      const axis = new THREE.Group(); mount.add(axis);
      const body = new THREE.Mesh(new THREE.CylinderGeometry(0.017, 0.017, 0.03, 14).rotateX(Math.PI / 2), mats.motor.clone()); axis.add(body);   // its own material: it glows as it heats
      const pr = propR(c);
      const disc = new THREE.Mesh(new THREE.CircleGeometry(pr, 32), mats.prop.clone()); disc.position.z = 0.02; axis.add(disc);
      const sm = spinMarks(pr, c.spin); sm.userData.noPick = true; axis.add(sm);
      // The group's Z runs along the shaft to the prop. A puller's thrust points that way, a pusher's back past the motor.
      const arrow = c.push ? new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(0, 0, -0.017), 0.1, colorOf('--accent'), 0.03, 0.018)
        : new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0.02), 0.1, colorOf('--accent'), 0.03, 0.018);
      arrow.userData.noPick = true; axis.add(arrow);
      const wake = new THREE.Mesh(new THREE.CylinderGeometry(0.71 * pr, pr, 3 * pr, 24, 1, true).rotateX(Math.PI / 2), mats.wake.clone());
      wake.position.z = 0.02 - 1.5 * pr; if (c.push) { wake.scale.z = -1; wake.position.z = 0.02 + 1.5 * pr; }   // the wake: behind the disc, past the motor for a puller, away from it for a pusher
      wake.visible = false; wake.userData.noPick = true; axis.add(wake);
      const stub = new THREE.Mesh(new THREE.BoxGeometry(pr * 0.7, 0.012, 0.004), mats.stub); stub.position.z = 0.02; stub.visible = false; axis.add(stub);   // what's left of a broken prop
      parts.set(c.id, { axis, disc, arrow, wake, body, spin: sm, stub });
    } else if (c.type === 'mass') {
      let geo;
      if (c.shape === 'sphere') geo = new THREE.SphereGeometry(c.radius, 20, 14);
      else if (c.shape === 'cylinder') geo = new THREE.CylinderGeometry(c.radius, c.radius, c.length, 20).rotateX(Math.PI / 2);
      else geo = isWing(c) ? airfoilGeo(...c.size) : new THREE.BoxGeometry(...c.size);
      const m = new THREE.Mesh(geo, c.cargo ? mats.cargo : isWing(c) ? mats.wing : c.known ? mats.mass : mats.massUnknown); m.position.set(...p); m.userData.compId = c.id; pickGroups.set(c.id, m); g.add(m);
      if (c.inc) m.setRotationFromMatrix(m4of(massRot(c)));
    } else if (c.type === 'latch') {   // a hook: its body, and a jaw that swings open (updateScene)
      const lg = new THREE.Group(); lg.position.set(...p); lg.userData.compId = c.id; pickGroups.set(c.id, lg); g.add(lg);
      lg.add(new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.022, 0.012), mats.frame));
      const jaw = new THREE.Group(); jaw.position.set(0.012, 0, -0.006); lg.add(jaw);
      const j1 = new THREE.Mesh(new THREE.BoxGeometry(0.004, 0.016, 0.016), mats.ink); j1.position.set(0, 0, -0.008); jaw.add(j1);
      const j2 = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.016, 0.004), mats.ink); j2.position.set(-0.01, 0, -0.016); jaw.add(j2);
      parts.set(c.id, { jaw });
    } else if (c.type === 'hang') {
      const hk = new THREE.Mesh(new THREE.SphereGeometry(0.016, 12, 8), mats.payload); hk.position.set(...p); hk.userData.compId = c.id; pickGroups.set(c.id, hk); g.add(hk);
    } else if (c.type === 'sensor') {
      const sg = new THREE.Group(); sg.position.set(...p);
      const Rm = eulerR(c.mount[0], c.mount[1], c.mount[2]);
      sg.quaternion.setFromRotationMatrix(new THREE.Matrix4().set(Rm[0], Rm[1], Rm[2], 0, Rm[3], Rm[4], Rm[5], 0, Rm[6], Rm[7], Rm[8], 0, 0, 0, 0, 1));
      const size = { imu: [0.022, 0.022, 0.008], mag: [0.016, 0.016, 0.006], baro: [0.014, 0.014, 0.01], fix: [0.03, 0.03, 0.008], flow: [0.02, 0.02, 0.012] }[c.kind];
      sg.add(new THREE.Mesh(new THREE.BoxGeometry(...size), mats.sensor));
      const ax = rod([0, 0, 0], [0.028, 0, 0], 0.0025, mats.sensorAxis); if (ax) sg.add(ax);   // sensor X axis shows the mount
      if (c.kind === 'flow') {   // rangefinder beam along the boresight
        const beam = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, -1)]), mats.beam);
        beam.visible = false; sg.add(beam); parts.set(c.id, { beam });
      }
      sg.userData.compId = c.id; pickGroups.set(c.id, sg); g.add(sg);
    }
  }
  buildGhost();
  cogDot = new THREE.Mesh(new THREE.SphereGeometry(0.014, 14, 10), mats.ink); drone.add(cogDot);
  const rg = new THREE.BufferGeometry().setFromPoints(Array.from({ length: 41 }, (_, i) => { const a = i / 40 * Math.PI * 2; return new THREE.Vector3(Math.cos(a) * 0.03, Math.sin(a) * 0.03, 0); }));
  modelRing = new THREE.Line(rg, mats.ring); modelRing.computeLineDistances(); drone.add(modelRing);
  for (const v of pendVis.values()) { worldFx.remove(v.line); worldFx.remove(v.ball); v.line.geometry.dispose(); v.ball.geometry.dispose(); }
  pendVis = new Map();
  for (const c of comps) if (c.type === 'hang') {
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
/* ───────── wings: each one's lift and drag, as arrows from where they act ───────── */
// Lift (across the air past the wing) and drag (along it), each smoothed over 0.1 s like the torques, and sized like
// the thrust arrows (a few cm per newton), so they compare with them. Seen through the airframe.
let aeroVis = { arrows: [], sm: [] };
function aeroArrow(col) {
  const a = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 0.2, col, 0.035, 0.022);
  for (const o of [a.line, a.cone]) { o.userData.noPick = true; o.material.depthTest = false; o.material.transparent = true; o.renderOrder = 19; }
  worldFx.add(a); return a;
}
function updateAeroVis(R, live) {
  const list = live && !S.crashed ? (S.aero || []) : [];
  while (aeroVis.arrows.length < list.length) aeroVis.arrows.push({ L: aeroArrow(colorOf('--lift')), D: aeroArrow(colorOf('--drag')) });
  aeroVis.arrows.forEach((ar, i) => {
    const e = list[i]; if (!e) { ar.L.visible = ar.D.visible = false; return; }
    const s = aeroVis.sm[i] = aeroVis.sm[i] || { L: e.L.slice(), D: e.D.slice() }, k = 0.15;
    s.L = add(s.L, scl(sub(e.L, s.L), k)); s.D = add(s.D, scl(sub(e.D, s.D), k));
    const p = add(S.p, m3v(R, e.P));
    for (const [key, a, f, on] of [['L', ar.L, s.L, view.lift], ['D', ar.D, s.D, view.drag]]) {
      const m = nrm(f); a.visible = on && m > 0.03; if (!a.visible) continue;
      const d = scl(m3v(R, f), 1 / m); a.position.set(p[0], p[1], p[2]); a.setDirection(tmpV.set(d[0], d[1], d[2])); a.setLength(0.05 + Math.min(0.8, m * 0.035), 0.035, 0.022);
    }
  });
  if (!list.length) aeroVis.sm.length = 0;
}

/* ───────── cargo: loose bodies, the latches' jaws, how far an open latch is from what it could grab ───────── */
let looseVis = new Map(), reachVis = [];   // loose body id -> group; per latch: { line, ring }
function looseGroup(L) {   // its parts, drawn at rest in the body's own axes (origin: its grab point)
  const g = new THREE.Group(), at = (m, p) => { m.position.set(...p); g.add(m); };
  for (const c of L.parts) {
    if (c.type === 'mass') {
      const geo = c.shape === 'sphere' ? new THREE.SphereGeometry(c.radius, 20, 14) : c.shape === 'cylinder' ? new THREE.CylinderGeometry(c.radius, c.radius, c.length, 20).rotateX(Math.PI / 2) : isWing(c) ? airfoilGeo(...c.size) : new THREE.BoxGeometry(...c.size);
      const mm = new THREE.Mesh(geo, c.origin == null ? mats.cargo : isWing(c) ? mats.wing : mats.mass); if (c.inc) mm.setRotationFromMatrix(m4of(massRot(c))); at(mm, c.pos);
    } else if (c.type === 'motor') {
      const mg = new THREE.Group(); mg.quaternion.setFromUnitVectors(Z, new THREE.Vector3(...mountDir(c)));
      mg.add(new THREE.Mesh(new THREE.CylinderGeometry(0.017, 0.017, 0.03, 14).rotateX(Math.PI / 2), mats.motor));
      const disc = new THREE.Mesh(new THREE.CircleGeometry(propR(c), 32), mats.prop); disc.position.z = 0.02; mg.add(disc);
      at(mg, c.pos);
    } else if (c.type === 'link') { const r = rod(c.pos, linkTip(c), 0.006, mats.servo); if (r) g.add(r); }
    else if (c.type === 'joint') at(new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.02, 0.036), mats.servo), c.pos);
    else if (c.type === 'hang') at(new THREE.Mesh(new THREE.SphereGeometry(payloadRad(c), 18, 12), mats.payload), c.pos);
    else if (c.type === 'sensor') at(new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.02, 0.008), mats.sensor), c.pos);
    else at(new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.022, 0.012), mats.frame), c.pos);
  }
  g.traverse(o => { o.userData.noPick = true; });
  return g;
}
function updateCargoVis(live) {
  const seen = new Set();
  for (const L of cargo.loose) {
    let g = looseVis.get(L.id); if (!g) { g = looseGroup(L); worldFx.add(g); looseVis.set(L.id, g); }
    seen.add(L.id); const o = looseGrab(L); g.position.set(o[0], o[1], o[2]); g.quaternion.set(L.q[1], L.q[2], L.q[3], L.q[0]);
  }
  for (const [id, g] of looseVis) if (!seen.has(id)) { worldFx.remove(g); g.traverse(x => x.geometry && x.geometry.dispose()); looseVis.delete(id); }
  for (const l of latches()) {   // the jaw: shut, or swung open
    const p = parts.get(l.id), st = cargo.lat.get(l.id); if (!p || !p.jaw) continue;
    p.jaw.rotation.y = -1.1 * (1 - (editMode ? (l.closed ? 1 : 0) : st ? st.pos : 1));
  }
  // From each open latch's hook to the nearest loose thing: green within reach, grey further (up to 3 m).
  const ls = live ? latches() : [];
  ls.forEach((l, i) => {
    let r = reachVis[i];
    if (!r) {
      r = { line: new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), mats.reachFar), ring: new THREE.Mesh(new THREE.RingGeometry(0.05, 0.062, 36), mats.ringFar) };
      for (const o of [r.line, r.ring]) { o.renderOrder = 20; o.userData.noPick = true; worldFx.add(o); }
      reachVis[i] = r;
    }
    const st = cargo.lat.get(l.id), n = st && st.pos < 0.999 ? cargoNearest(l) : null, show = !!n && n.d < 3;
    r.line.visible = r.ring.visible = show; if (!show) return;
    const b = looseGrab(n.L), pos = r.line.geometry.attributes.position;
    pos.setXYZ(0, ...n.hook); pos.setXYZ(1, ...b); pos.needsUpdate = true; r.line.geometry.computeBoundingSphere(); r.line.computeLineDistances();
    r.line.material = n.ok ? mats.reachOk : mats.reachFar; r.ring.material = n.ok ? mats.ringOk : mats.ringFar;
    r.ring.position.set(b[0], b[1], b[2] + 0.004);
  });
  for (let i = ls.length; i < reachVis.length; i++) reachVis[i].line.visible = reachVis[i].ring.visible = false;
}

/* A torque, drawn as a turning arrow: an arc round the torque's axis, turning the way the torque turns
 * (right-hand rule: thumb along the axis, fingers the way it turns). The arc grows with the torque: set(frac)
 * shows that fraction of 315°. Built at unit radius; scale the group to size it. */
const TQ_SEG = 48, TQ_SPAN = 1.75 * Math.PI, TQ_RAD = 6;
class TqArc extends THREE.Curve { getPoint(t, o = new THREE.Vector3()) { const a = t * TQ_SPAN; return o.set(Math.cos(a), Math.sin(a), 0); } }
function torqueGlyph(color, tube, opacity = 1, onTop = false) {   // onTop: seen through the airframe
  const g = new THREE.Group();
  const mat = new THREE.MeshBasicMaterial({ color, transparent: opacity < 1 || onTop, opacity, depthWrite: opacity >= 1 && !onTop, depthTest: !onTop });
  const arc = new THREE.Mesh(new THREE.TubeGeometry(new TqArc(), TQ_SEG, tube, TQ_RAD, false), mat);
  const head = new THREE.Mesh(new THREE.ConeGeometry(tube * 2.6, tube * 7, 12), mat);
  g.add(arc); g.add(head); g.userData = { arc, head };
  g.set = frac => {   // how much of the arc to show (0–1), with the arrowhead at its end
    const k = Math.max(2, Math.round(clamp(frac, 0, 1) * TQ_SEG)), a = k / TQ_SEG * TQ_SPAN;
    arc.geometry.setDrawRange(0, k * TQ_RAD * 6);
    head.position.set(Math.cos(a), Math.sin(a), 0); head.rotation.set(0, 0, a);
  };
  g.traverse(o => { o.userData.noPick = true; o.renderOrder = onTop ? 22 : 18; });
  return g;
}
function buildWorldFx() {
  for (const o of [gravArrow, windArrow, spMarker, headArrow, trailLine, tqNetGlyph, tqNetGlyph && tqNetGlyph.userData.axis, tqWantArrow, ...tqRotor]) if (o) { worldFx.remove(o); o.traverse(x => x.geometry && x.geometry.dispose()); }
  const tqCol = colorOf('--torque');
  tqNetGlyph = torqueGlyph(tqCol, 0.045, 0.95, true);
  worldFx.add(tqNetGlyph);
  const axisArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 0.2, tqCol, 0.035, 0.022);   // the torque's axis (right-hand rule)
  for (const o of [axisArrow.line, axisArrow.cone]) { o.userData.noPick = true; o.material.depthTest = false; o.material.transparent = true; o.renderOrder = 22; }
  worldFx.add(axisArrow); tqNetGlyph.userData.axis = axisArrow;
  tqWantArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 0.2, tqCol, 0.03, 0.02);   // what the controller asked for: faint
  for (const o of [tqWantArrow.line, tqWantArrow.cone]) { o.material.transparent = true; o.material.opacity = 0.35; o.material.depthTest = false; o.renderOrder = 21; o.userData.noPick = true; }
  worldFx.add(tqWantArrow);
  tqRotor = [];
  gravArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(), 0.2, colorOf('--grav'), 0.035, 0.02); worldFx.add(gravArrow);
  windArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.2, colorOf('--wind'), 0.04, 0.025); worldFx.add(windArrow);
  spMarker = new THREE.Group();
  spMarker.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(Array.from({ length: 49 }, (_, i) => { const a = i / 48 * Math.PI * 2; return new THREE.Vector3(Math.cos(a) * 0.09, Math.sin(a) * 0.09, 0); })), mats.sp));
  spMarker.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, -1)]), mats.sp));
  worldFx.add(spMarker);
  {   // level around the drone, the way forward moves it: a unit arrow from just outside the props, scaled to the airframe
    const sh = new THREE.Shape(); sh.moveTo(1, -0.07); sh.lineTo(1.45, -0.07); sh.lineTo(1.45, -0.2); sh.lineTo(1.8, 0); sh.lineTo(1.45, 0.2); sh.lineTo(1.45, 0.07); sh.lineTo(1, 0.07); sh.closePath();
    headArrow = new THREE.Mesh(new THREE.ShapeGeometry(sh), mats.heading); headArrow.renderOrder = 2; worldFx.add(headArrow);
  }
  trailLine = new THREE.Line(new THREE.BufferGeometry(), mats.trail); worldFx.add(trailLine);
  for (const v of pendVis.values()) { v.line.material = mats.cable; v.ball.material = mats.payload; }
}
const tmpV = new THREE.Vector3();
// Torque sizes: an arc's sweep and an axis arrow's length grow with the torque and level off for big ones.
const TQ_ROTOR = 0.03, TQ_NET = 0.08;   // N·m for about two thirds of the full sweep
function updateTorque(R, live) {
  const ok = live && !S.crashed, rs = S.rotors || [];
  // shown above `hi`, hidden below `lo`: no flicker on and off round one threshold
  const keep = (o, m, lo, hi) => (o.userData.shown = m > (o.userData.shown ? lo : hi));
  // Each rotor's reaction: the motor turning the prop pushes the frame the other way, about the rotor's axis.
  while (tqRotor.length < rs.length) { const g = torqueGlyph(colorOf('--torque'), 0.07, 0.8); worldFx.add(g); tqRotor.push(g); }
  tqRotor.forEach((g, i) => {
    const ro = rs[i], t = ro && ro.st && ro.st.tqR, m = t ? nrm(t) : 0;
    g.visible = ok && view.rtorque && i < rs.length && keep(g, m, 2e-4, 5e-4); if (!g.visible) return;
    const d = m3v(R, ro.d), pw = add(S.p, m3v(R, ro.p)), u = scl(m3v(R, t), 1 / m);
    g.position.set(pw[0] - d[0] * 0.018, pw[1] - d[1] * 0.018, pw[2] - d[2] * 0.018);   // just below the disc
    g.quaternion.setFromUnitVectors(Z, tmpV.set(u[0], u[1], u[2]));
    g.scale.setScalar(clamp((ro.R || 0.06) * 0.7, 0.03, 0.12));
    g.set(1 - Math.exp(-m / TQ_ROTOR));
  });
  // The net torque about the centre of mass: everything together (rotors, air, cables, ground; weight adds none).
  const tq = S.tq, m = tq ? nrm(tq) : 0, ax = tqNetGlyph.userData.axis;
  const cg = add(S.p, m3v(R, truth.c));
  tqNetGlyph.visible = ax.visible = ok && view.ntorque && keep(tqNetGlyph, m, 0.002, 0.005);
  if (tqNetGlyph.visible) {
    const u = scl(m3v(R, tq), 1 / m), f = 1 - Math.exp(-m / TQ_NET);
    tqNetGlyph.position.set(cg[0], cg[1], cg[2]); tqNetGlyph.quaternion.setFromUnitVectors(Z, tmpV.set(u[0], u[1], u[2]));
    tqNetGlyph.scale.setScalar(0.15); tqNetGlyph.set(f);
    ax.position.set(cg[0], cg[1], cg[2]); ax.setDirection(tmpV.set(u[0], u[1], u[2])); ax.setLength(0.06 + 0.3 * f, 0.035, 0.022);
  }
  // What the controller asked for, smoothed the same way (sim.js), to compare with what it gets: the motors take
  // a few tens of milliseconds to follow, so the two differ most while it's changing.
  const want = ok && view.want ? S.tqWant : null, mw = want ? nrm(want) : 0;
  tqWantArrow.visible = !!want && keep(tqWantArrow, mw, 0.002, 0.005);
  if (tqWantArrow.visible) {
    const u = scl(m3v(R, want), 1 / mw), f = 1 - Math.exp(-mw / TQ_NET);
    tqWantArrow.position.set(cg[0], cg[1], cg[2]); tqWantArrow.setDirection(tmpV.set(u[0], u[1], u[2])); tqWantArrow.setLength(0.06 + 0.3 * f, 0.03, 0.02);
  }
}
function updateScene(selected = true) {
  if (droneShown !== cargo.rev + ':' + editMode) rebuildDrone();
  if (grid) grid.visible = view.grid;
  const R = qmat(S.q); const { hub } = hubState(R);
  if (selected) updateCity(hub);
  drone.position.set(...hub);
  if (editMode) drone.quaternion.set(0, 0, 0, 1);            // edit in body axes: level, nose along +X
  else drone.quaternion.set(S.q[1], S.q[2], S.q[3], S.q[0]);
  const live = !editMode;
  for (const j of joints()) {   // each joint's group turns about its hinge (level and at rest while editing)
    const g = jointGroups.get(j.id); if (g) g.quaternion.setFromAxisAngle(tmpV.set(...jointAxis(j)), editMode ? previewAngle(j) : angleTrue(j));   // editing: at rest, or the preview
    const rv = rangeVis.get(j.id); if (!rv) continue;
    rv.g.visible = editMode || (view.servo && motorsUnder(j).length > 0);   // flying: a faint fan behind a servo that steers a rotor
    { const s = hs.get(j.id), broke = s && (s.limp || s.jam != null); for (const k of ['fan', 'line', 'dash']) rv.m[k].color.copy(broke ? heatCol.bad : heatCol.swing); if (broke && !editMode) rv.g.visible = true; }   // a jammed or limp servo shows red
    if (!editMode) { rv.m.fan.opacity = 0.08; rv.m.line.opacity = 0.3; rv.m.dash.opacity = 0; }
    else {   // brighter for the servo you're working on, or one carrying it
      const sel = compById(edit.sel), on = edit.sel === j.id || edit.hover === j.id || (sel && isUnder(sel, j));
      rv.m.fan.opacity = on ? 0.22 : 0.07; rv.m.line.opacity = on ? 0.95 : 0.35; rv.m.dash.opacity = on ? 0.9 : 0.3;
    }
  }
  const pj = previewing();
  for (const c of actuators()) {
    const p = parts.get(c.id); if (!p) continue; const st = act.get(c.id);
    p.axis.quaternion.setFromUnitVectors(Z, tmpV.set(...mountDir(c)));   // the motor's own mounting; its joints turn the group above
    const T = st.T * motorEff(c), shown = pj && isUnder(c, pj); p.disc.material.opacity = shown ? 0.45 : 0.12 + 0.4 * clamp(T / c.tmax, 0, 1);
    {   // heat: the motor warms toward amber from 30 °C below its limit, red past it; a stopped one's disc turns red
      const s = hs.get(c.id), lim = c.tmaxC ?? 120, f = s ? clamp((s.T - (lim - 30)) / 30, 0, 1.4) : 0;
      p.body.material.color.copy(mats.motor.color); if (f > 0) p.body.material.color.lerp(f < 1 ? heatCol.warn : heatCol.bad, Math.min(1, f) * 0.85);
      p.disc.material.color.copy(s && (s.dead || s.loss > 0.004) ? heatCol.bad : mats.prop.color);
      if (s && s.dead) p.disc.material.opacity = 0.18;
      const broke = !!(s && s.prop); p.disc.visible = !broke; p.stub.visible = broke;
      if (broke) p.stub.rotation.z += 0.9;   // the stub whirls
    }
    if (p.spin) p.spin.visible = (editMode || view.spin) && p.disc.visible;
    p.wake.visible = live && view.air && T > 0.02; if (p.wake.visible) p.wake.material.opacity = 0.05 + 0.3 * clamp(T / c.tmax, 0, 1);
    p.arrow.visible = live && view.thrust && T > 0.02; if (p.arrow.visible) p.arrow.setLength(0.04 + T * 0.035, 0.03, 0.018);
    else if (editMode) {   // editing: every motor shows which way its thrust points (pull or push); the selected one boldly
      const big = shown || edit.sel === c.id; p.arrow.visible = true;
      if (big) p.arrow.setLength(0.16, 0.035, 0.022); else p.arrow.setLength(0.075, 0.022, 0.014);
    }
    const over = editMode && edit.sel === c.id, faint = editMode && !over && !shown;   // the selected one drawn over the motor, so a pusher's shows
    for (const o of [p.arrow.line, p.arrow.cone]) { o.material.depthTest = !over; o.renderOrder = over ? 21 : 0; o.material.transparent = true; o.material.opacity = faint ? 0.55 : 1; }   // where its thrust points as the servo swings
  }
  for (const c of sensorsOf('flow')) {   // beam length: what the rangefinder reads, or its max range
    const p = parts.get(c.id), rt = sens.get(c.id); if (!p || !p.beam) continue;
    const L = rt && rt.latest; p.beam.visible = live && view.beam;
    if (live) { p.beam.scale.z = L && L.range > 0 ? L.range : c.maxRange; p.beam.computeLineDistances(); }
  }
  cogDot.position.set(...truth.c); modelRing.position.set(...model.c); cogDot.visible = editMode || view.cog; modelRing.visible = cogDot.visible && nrm(sub(truth.c, model.c)) > 0.004;
  gravArrow.visible = live && view.weight; { const cg = add(S.p, m3v(R, truth.c)); gravArrow.position.set(cg[0], cg[1], cg[2] - 0.02); } gravArrow.setLength(0.06 + truth.m * G * 0.02, 0.035, 0.02);
  const wv = windVec(); windArrow.visible = live && view.wind && envr.wind > 0.05;
  updateTorque(R, live);
  if (windArrow.visible) { const u = unit(wv); windArrow.setDirection(new THREE.Vector3(...u)); windArrow.position.set(hub[0] - u[0] * 0.6, hub[1] - u[1] * 0.6, hub[2] + 0.25); windArrow.setLength(0.08 + envr.wind * 0.05, 0.04, 0.025); }
  for (const c of liveComps()) {
    if (c.type !== 'hang') continue; const v = pendVis.get(c.id), st = pend.get(c.id); if (!v || !st) continue;
    v.line.visible = v.ball.visible = live;
    const aw = add(S.p, m3v(R, posNow(c))); const pos = v.line.geometry.attributes.position;
    pos.setXYZ(0, ...aw); pos.setXYZ(1, ...st.p); pos.needsUpdate = true; v.line.geometry.computeBoundingSphere(); v.ball.position.set(...st.p);
  }
  updateCargoVis(live); updateAeroVis(R, live);
  ghost.visible = live && view.est; if (ghost.visible) { ghost.position.set(...est.p); ghost.quaternion.set(est.q[1], est.q[2], est.q[3], est.q[0]); }
  headArrow.visible = live && view.heading && !S.crashed;
  if (headArrow.visible) {   // with navigation the keys move along the heading it holds; without (angle mode) they lean the body: its own heading
    const yaw = hasTask('nav') ? setpoint.yaw * D2R : droneYaw(), s = Math.max(0.12, cReach * 0.85);
    headArrow.position.set(...hub); headArrow.rotation.set(0, 0, yaw); headArrow.scale.set(s, s, 1);   // level, at the drone's height, whatever its tilt
  }
  spMarker.visible = live && view.target; spMarker.position.set(setpoint.x, setpoint.y, setpoint.z); spMarker.children[1].scale.z = setpoint.z;
  trailLine.visible = live && view.trail;
  if (view.trail && trail.length > 1) { trailLine.geometry.dispose(); trailLine.geometry = new THREE.BufferGeometry().setFromPoints(trail.map(p => new THREE.Vector3(...p))); }
  if (!selected) return;
  let tgt = view.follow || editMode ? new THREE.Vector3(...hub) : new THREE.Vector3(setpoint.x, setpoint.y, setpoint.z);
  const selC = editMode && compById(edit.sel);
  if (selC) {   // editing: orbit round the selected part; with the servo panel open, keep it clear of the panel
    // The centre is set when the part is picked (clicking it again re-centres), not followed: dragging a part
    // mustn't move the view under the pointer.
    if (edit.focusId !== selC.id || edit.refocus) {
      edit.focusId = selC.id; edit.refocus = false; edit.focusLocal = selC.pos.slice();
      edit.focusShift = selC.type === 'joint' ? new THREE.Vector3(0, 1, 0).applyQuaternion(camera.quaternion).multiplyScalar(-0.09 * cam.dist) : new THREE.Vector3();
    }
    tgt = drone.localToWorld(new THREE.Vector3(...edit.focusLocal)).add(edit.focusShift);
  } else if (editMode) edit.focusId = null;
  cam.target.lerp(tgt.add(cam.pan), view.follow ? 0.12 : 0.06);
  if (view.chase && live) {  // swing the camera behind the target heading
    let d = setpoint.yaw * D2R + Math.PI - cam.az; d = Math.atan2(Math.sin(d), Math.cos(d));
    cam.az += d * 0.06;
  }
  updateCamera();
}
// World view keeps its orbit centre instead of following an internal drone context.
function updateCamera() {
  stepCamAnim();
  const ce = Math.cos(cam.el);
  if (camera.isOrthographicCamera) {   // frame what the perspective camera shows at the orbit's centre
    const h = cam.dist * Math.tan(perspCam.fov * D2R / 2), w = h * perspCam.aspect;
    if (orthoCam.top !== h || orthoCam.right !== w) { orthoCam.left = -w; orthoCam.right = w; orthoCam.top = h; orthoCam.bottom = -h; orthoCam.updateProjectionMatrix(); }
  }
  camera.position.set(cam.target.x + cam.dist * ce * Math.cos(cam.az), cam.target.y + cam.dist * ce * Math.sin(cam.az), cam.target.z + cam.dist * Math.sin(cam.el));
  camera.lookAt(cam.target);
  updateEditView(); drawTriad();
}

// Screen-space pan has the same scale in perspective and the matching orthographic view.
const panDelta = new THREE.Vector3(), panRight = new THREE.Vector3(), panUp = new THREE.Vector3();
function panCamera(dx,dy) {
  const scale=2*cam.dist*Math.tan(perspCam.fov*D2R/2)/(Math.max(1,vpEl.clientHeight)*camera.zoom);
  camera.updateMatrixWorld();
  panRight.setFromMatrixColumn(camera.matrixWorld,0);panUp.setFromMatrixColumn(camera.matrixWorld,1);
  panDelta.copy(panRight).multiplyScalar(-dx*scale).addScaledVector(panUp,dy*scale);
  cam.target.add(panDelta);
  if(typeof fleet==='undefined' || !fleet.ready || fleet.selected)cam.pan.add(panDelta);
  cam.anim=null;
}
function centerCamera() {
  cam.target.sub(cam.pan);cam.pan.set(0,0,0);
  if(typeof fleet!=='undefined' && fleet.ready && !fleet.selected && fleet.drones.length) {
    const box=new THREE.Box3();for(const d of fleet.drones)box.expandByPoint(new THREE.Vector3(...d.state.S.p));
    box.getCenter(cam.target);
  }
  updateCamera();
}
// One finger or plain drag rotates; Ctrl/Command drag pans; two fingers pan and pinch to zoom.
const ptrs = new Map(); let pinch0 = 0;
vpEl.addEventListener('pointerdown', e => {
  if(e.button!==0 && !(e.button===2 && e.ctrlKey))return;
  const cameraOnly=e.ctrlKey||e.metaKey||ptrs.size>0;
  if(edit.drag || (!cameraOnly && editPointerDown(e)))return;
  vpEl.setPointerCapture(e.pointerId);
  ptrs.set(e.pointerId,{x:e.clientX,y:e.clientY,cameraOnly,multi:false});
  if(cameraOnly)edit.down=null;
  if(ptrs.size>=2){
    for(const p of ptrs.values()){p.cameraOnly=true;p.multi=true;}
    edit.down=null;const [a,b]=[...ptrs.values()];pinch0=Math.hypot(a.x-b.x,a.y-b.y);
  }
});
vpEl.addEventListener('pointermove', e => {
  const p=ptrs.get(e.pointerId),pan=p && (p.cameraOnly || e.ctrlKey || e.metaKey);
  if(pan){p.cameraOnly=true;edit.down=null;setHover(null);}
  else if (editPointerMove(e, ptrs.size > 0 && edit.down && Math.hypot(e.clientX - edit.down.x, e.clientY - edit.down.y) >= 5)) return;
  if(!p)return;
  const [a,b]=[...ptrs.values()],mx=b?(a.x+b.x)/2:0,my=b?(a.y+b.y)/2:0;
  if (ptrs.size === 1 && !p.multi) {
    if(pan)panCamera(e.clientX-p.x,e.clientY-p.y);
    else {cam.anim=null;cam.az-=(e.clientX-p.x)*.008;cam.el=clamp(cam.el+(e.clientY-p.y)*.006,-EL_MAX,EL_MAX);}
  }
  p.x = e.clientX; p.y = e.clientY;
  if(ptrs.size===2){
    const d=Math.hypot(a.x-b.x,a.y-b.y);
    if(pinch0>0 && d>0)cam.dist=clamp(cam.dist*pinch0/d,.6,maxDist());
    pinch0=d;panCamera((a.x+b.x)/2-mx,(a.y+b.y)/2-my);
  }
});
const endPtr = e => {
  const p=ptrs.get(e.pointerId);
  if(p?.cameraOnly || e.type!=='pointerup')edit.down=null;
  const handled=e.type==='pointerup' && !p?.cameraOnly && editPointerUp(e);
  if(e.type!=='pointerup' && edit.drag?.pointerId===e.pointerId)endDrag();
  ptrs.delete(e.pointerId);pinch0=0;return handled;
};
vpEl.addEventListener('pointerleave', () => { if (!edit.drag) setHover(null); });
vpEl.addEventListener('pointerup', endPtr); vpEl.addEventListener('pointercancel', endPtr);
vpEl.addEventListener('lostpointercapture',endPtr);
vpEl.addEventListener('contextmenu',e=>{if(e.ctrlKey||e.metaKey)e.preventDefault();});
vpEl.addEventListener('wheel', e => { e.preventDefault(); cam.anim = null; cam.dist = clamp(cam.dist * Math.exp(e.deltaY * 0.001), 0.6, maxDist()); }, { passive: false });
new ResizeObserver(() => { const w = vpEl.clientWidth, h = vpEl.clientHeight; if (!w || !h) return; renderer.setSize(w, h, false); perspCam.aspect = w / h; perspCam.updateProjectionMatrix(); orthoCam.top = NaN; }).observe(vpEl);

/* ───────── the view box: orientation triad, projection and named views ───────── */
// The triad shows the drone's axes as the camera sees them (X forward, Y left, Z up). Click an axis end to
// look from that side; keys: numpad 1 / 3 / 7 front, right, top (with Ctrl: back, left, bottom), 0 iso,
// 5 or O perspective/orthographic, V steps through top, front, right and iso.
const triad = { cv: document.getElementById('triad'), ends: [], hover: -1 };
const TRIAD_ENDS = [['x', 1, 'front'], ['x', -1, 'back'], ['y', 1, 'left'], ['y', -1, 'right'], ['z', 1, 'top'], ['z', -1, 'bottom']];
function drawTriad() {
  const cv = triad.cv; if (!cv) return;
  const dpr = Math.min(window.devicePixelRatio || 1, 2), W = cv.clientWidth, H = cv.clientHeight; if (!W) return;
  if (cv.width !== Math.round(W * dpr)) { cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
  const g = cv.getContext('2d'); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
  const yaw = droneYaw(), cy = Math.cos(yaw), sy = Math.sin(yaw), ax = { x: [cy, sy, 0], y: [-sy, cy, 0], z: [0, 0, 1] };
  const e = camera.matrixWorld.elements, right = [e[0], e[1], e[2]], up = [e[4], e[5], e[6]], back = [e[8], e[9], e[10]];
  const cx = W / 2, cyy = H / 2, R = Math.min(W, H) / 2 - 11, col = { x: tok('--ax-x'), y: tok('--ax-y'), z: tok('--ax-z') };
  triad.ends = TRIAD_ENDS.map(([k, sg, name], i) => { const d = ax[k].map(v => v * sg); return { i, k, sg, name, x: cx + R * dot(d, right), y: cyy - R * dot(d, up), z: dot(d, back) }; });
  for (const t of [...triad.ends].sort((a, b) => a.z - b.z)) {   // far ends first
    const hot = triad.hover === t.i, c = col[t.k];
    if (t.sg > 0) { g.strokeStyle = c; g.lineWidth = 2; g.beginPath(); g.moveTo(cx, cyy); g.lineTo(t.x, t.y); g.stroke(); }
    g.beginPath(); g.arc(t.x, t.y, hot ? 8.5 : 7, 0, Math.PI * 2);
    if (t.sg > 0) { g.fillStyle = c; g.fill(); g.fillStyle = tok('--panel'); g.font = '600 9px ' + tok('--f-mono'); g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText(t.k.toUpperCase(), t.x, t.y + 0.5); }
    else { g.fillStyle = tok('--panel'); g.fill(); g.strokeStyle = c; g.lineWidth = hot ? 2 : 1.3; g.stroke(); }
  }
}
function triadHit(ev) {
  const r = triad.cv.getBoundingClientRect(), x = ev.clientX - r.left, y = ev.clientY - r.top;
  let best = -1, bd = 11;
  for (const t of [...triad.ends].sort((a, b) => b.z - a.z)) { const d = Math.hypot(t.x - x, t.y - y); if (d < bd) { bd = d; best = t.i; } }
  return best;
}
if (triad.cv) {
  triad.cv.addEventListener('pointermove', e => { triad.hover = triadHit(e); triad.cv.style.cursor = triad.hover >= 0 ? 'pointer' : ''; const t = TRIAD_ENDS[triad.hover]; triad.cv.title = t ? `Look from the ${VIEWS[t[2]].label.toLowerCase()}` : 'Click an axis to look from that side'; });
  triad.cv.addEventListener('pointerleave', () => { triad.hover = -1; });
  triad.cv.addEventListener('click', e => { const i = triadHit(e); if (i >= 0) snapView(TRIAD_ENDS[i][2]); });
}
function syncViewUi() {
  const o = camera.isOrthographicCamera;
  const a = document.getElementById('projPersp'), b = document.getElementById('projOrtho');
  if (a) { a.setAttribute('aria-pressed', String(!o)); b.setAttribute('aria-pressed', String(o)); }
}
document.getElementById('projPersp')?.addEventListener('click', () => setProjection('persp'));
document.getElementById('projOrtho')?.addEventListener('click', () => setProjection('ortho'));
document.querySelectorAll('[data-view]').forEach(b => b.addEventListener('click', () => snapView(b.dataset.view)));
const V_CYCLE = ['top', 'front', 'right', 'iso']; let vCycle = -1;
window.addEventListener('keydown', e => {
  if (e.defaultPrevented || e.metaKey || e.altKey || typingIn(e.target)) return;
  const ctl = e.ctrlKey, map = { Numpad1: ctl ? 'back' : 'front', Numpad3: ctl ? 'left' : 'right', Numpad7: ctl ? 'bottom' : 'top', Numpad0: 'iso' };
  if (map[e.code]) { snapView(map[e.code]); e.preventDefault(); return; }
  if (ctl) return;
  if (e.code === 'Numpad5' || e.code === 'KeyO') { setProjection(camera.isOrthographicCamera ? 'persp' : 'ortho'); e.preventDefault(); }
  else if (e.code === 'KeyV' && !e.repeat) { vCycle = (vCycle + 1) % V_CYCLE.length; snapView(V_CYCLE[vCycle]); e.preventDefault(); }
});
