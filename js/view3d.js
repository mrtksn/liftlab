'use strict';
// three.js scene: the airframe, force arrows, cable payloads, target marker and trail.

const view = { follow: true, chase: false, forces: true, trail: true, est: true, air: false };
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
let mats = {}, rangeVis = new Map(), jointGroups = new Map(), parts = new Map(), pickGroups = new Map(), pendVis = new Map(), ghost = null, cogDot, modelRing, gravArrow, windArrow, spMarker, trailLine;
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
    beam: new THREE.LineDashedMaterial({ color: colorOf('--sensor'), dashSize: 0.03, gapSize: 0.02 }),
    wake: new THREE.MeshBasicMaterial({ color: colorOf('--wind'), transparent: true, opacity: 0.18, side: THREE.DoubleSide, depthWrite: false }),
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
  const col = colorOf('--accent');
  const m = {
    fan: new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.1, side: THREE.DoubleSide, depthWrite: false }),
    line: new THREE.LineBasicMaterial({ color: col, transparent: true, opacity: 0.5 }),
    dash: new THREE.LineDashedMaterial({ color: col, dashSize: 0.012, gapSize: 0.008, transparent: true, opacity: 0.6 }),
  };
  const g = new THREE.Group(); g.position.set(...p); g.visible = false;
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
function rebuildDrone() {
  disposeGroup(drone); parts = new Map(); pickGroups = new Map(); jointGroups = new Map(); rangeVis = new Map();
  drone.add(new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.12, 0.04), mats.frame));
  const nose = rod([0.06, 0, 0], [0.1, 0, 0], 0.006, mats.ink); if (nose) drone.add(nose);
  // Parts on a servo joint live inside that joint's group, which turns about the hinge; nested joints nest.
  const js = joints().slice().sort((a, b) => chainOf(a).length - chainOf(b).length);
  const holder = c => { const j = parentJoint(c); return j ? { g: jointGroups.get(j.id), o: j.pos } : { g: drone, o: [0, 0, 0] }; };
  // A part's connector starts where it hangs from: a rod's tip, a joint's pivot, or the hub.
  const rel = c => { const h = holder(c), par = parentOf(c); return { g: h.g, p: sub(c.pos, h.o), from: par && par.type === 'link' ? sub(linkTip(par), h.o) : [0, 0, 0] }; };
  for (const j of js) {
    const { g, p, from } = rel(j);
    const r = rod(from, p, 0.007, mats.frame); if (r) g.add(r);
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
    const hg = new THREE.Group(); hg.quaternion.setFromRotationMatrix(basis); hg.add(horn); jg.add(hg);
  }
  for (const c of cfg.comps) {
    if (c.type === 'joint') continue;
    const { g, p, from } = rel(c);
    const r = rod(from, p, 0.007, mats.frame); if (r) g.add(r);
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
      axis.add(new THREE.Mesh(new THREE.CylinderGeometry(0.017, 0.017, 0.03, 14).rotateX(Math.PI / 2), mats.motor));
      const pr = propR(c);
      const disc = new THREE.Mesh(new THREE.CircleGeometry(pr, 32), mats.prop.clone()); disc.position.z = 0.02; axis.add(disc);
      const arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0.02), 0.1, colorOf('--accent'), 0.03, 0.018); axis.add(arrow);
      const wake = new THREE.Mesh(new THREE.CylinderGeometry(0.71 * pr, pr, 3 * pr, 24, 1, true).rotateX(Math.PI / 2), mats.wake.clone());
      wake.position.z = 0.02 - 1.5 * pr; wake.visible = false; axis.add(wake);   // the wake column below the disc
      parts.set(c.id, { axis, disc, arrow, wake });
    } else if (c.type === 'mass') {
      let geo;
      if (c.shape === 'sphere') geo = new THREE.SphereGeometry(c.radius, 20, 14);
      else if (c.shape === 'cylinder') geo = new THREE.CylinderGeometry(c.radius, c.radius, c.length, 20).rotateX(Math.PI / 2);
      else geo = new THREE.BoxGeometry(...c.size);
      const m = new THREE.Mesh(geo, c.known ? mats.mass : mats.massUnknown); m.position.set(...p); m.userData.compId = c.id; pickGroups.set(c.id, m); g.add(m);
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
  for (const j of joints()) {   // each joint's group turns about its hinge (level and at rest while editing)
    const g = jointGroups.get(j.id); if (g) g.quaternion.setFromAxisAngle(tmpV.set(...jointAxis(j)), editMode ? previewAngle(j) : angleTrue(j));   // editing: at rest, or the preview
    const rv = rangeVis.get(j.id); if (!rv) continue;
    rv.g.visible = editMode || (view.forces && motorsUnder(j).length > 0);   // flying: a faint fan behind a servo that steers a rotor
    if (!editMode) { rv.m.fan.opacity = 0.08; rv.m.line.opacity = 0.3; rv.m.dash.opacity = 0; }
    else {   // brighter for the servo you're working on, or one carrying it
      const sel = compById(edit.sel), on = edit.sel === j.id || edit.hover === j.id || (sel && isUnder(sel, j));
      rv.m.fan.opacity = on ? 0.22 : 0.07; rv.m.line.opacity = on ? 0.95 : 0.35; rv.m.dash.opacity = on ? 0.9 : 0.3;
    }
  }
  const pj = previewing();
  for (const c of actuators()) {
    const p = parts.get(c.id); if (!p) continue; const st = act.get(c.id);
    p.axis.quaternion.setFromUnitVectors(Z, tmpV.set(...actDir(c)));   // the motor's own mounting; its joints turn the group above
    const T = st.T * c.health / 100, shown = pj && isUnder(c, pj); p.disc.material.opacity = shown ? 0.45 : 0.12 + 0.4 * clamp(T / c.tmax, 0, 1);
    p.wake.visible = live && view.air && T > 0.02; if (p.wake.visible) p.wake.material.opacity = 0.05 + 0.3 * clamp(T / c.tmax, 0, 1);
    p.arrow.visible = live && view.forces && T > 0.02; if (p.arrow.visible) p.arrow.setLength(0.04 + T * 0.035, 0.03, 0.018);
    else if (shown) { p.arrow.visible = true; p.arrow.setLength(0.16, 0.035, 0.022); }   // where its thrust points as the servo swings
  }
  for (const c of sensorsOf('flow')) {   // beam length: what the rangefinder reads, or its max range
    const p = parts.get(c.id), rt = sens.get(c.id); if (!p || !p.beam) continue;
    const L = rt && rt.latest; p.beam.visible = live;
    if (live) { p.beam.scale.z = L && L.range > 0 ? L.range : c.maxRange; p.beam.computeLineDistances(); }
  }
  cogDot.position.set(...truth.c); modelRing.position.set(...model.c); modelRing.visible = nrm(sub(truth.c, model.c)) > 0.004;
  gravArrow.visible = live && view.forces; { const cg = add(S.p, m3v(R, truth.c)); gravArrow.position.set(cg[0], cg[1], cg[2] - 0.02); } gravArrow.setLength(0.06 + truth.m * G * 0.02, 0.035, 0.02);
  const wv = windVec(); windArrow.visible = live && view.forces && envr.wind > 0.05;
  if (windArrow.visible) { const u = unit(wv); windArrow.setDirection(new THREE.Vector3(...u)); windArrow.position.set(hub[0] - u[0] * 0.6, hub[1] - u[1] * 0.6, hub[2] + 0.25); windArrow.setLength(0.08 + envr.wind * 0.05, 0.04, 0.025); }
  for (const c of cfg.comps) {
    if (c.type !== 'hang') continue; const v = pendVis.get(c.id), st = pend.get(c.id); if (!v || !st) continue;
    v.line.visible = v.ball.visible = live;
    const aw = add(S.p, m3v(R, posNow(c))); const pos = v.line.geometry.attributes.position;
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
