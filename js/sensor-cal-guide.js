'use strict';
// Tune → Sensor calibration, guided: the 3D view shows each pose. The drone is held where it was (the simulation and
// the real drone carry on underneath; only the picture stops), the camera comes close and turns to see the move
// best, and the drone acts it out, again and again, with an arrow for the way it goes. Meanwhile the guide asks the
// drone (imucal, as Measure does) about once a second: a reading that is still and points the pose's way
// (SensorCal.check), twice in a row and the same within 2°, is taken, and the guide goes on to the next pose. What's
// wrong is said as it goes: "lift the front more: 18° now", "hold it still", "the other way round".

const scalG = { on: false, done: false, i: 0, at: [0, 0, 0], yaw: 0, t0: 0, msg: '', tone: '', got: 0, last: null, run: 0, arrow: null, arrowFor: '', box: null };
// Each pose acted out: the turn from level (body axis, degrees) and where the camera looks from (azimuth from the
// drone's nose, elevation; radians): from its side for nose up and down, from behind for a roll, from above for level.
const GUIDE_MOVE = {
  flat: { cam: [-Math.PI / 4, 0.55] }, again: { cam: [-Math.PI / 4, 0.55] },
  nose: { axis: 'y', deg: -60, cam: [Math.PI / 2 - 0.35, 0.16] }, tail: { axis: 'y', deg: 60, cam: [Math.PI / 2 + 0.35, 0.16] },
  left: { axis: 'x', deg: -80, cam: [Math.PI + 0.3, 0.2] }, right: { axis: 'x', deg: 80, cam: [Math.PI - 0.3, 0.2] },
  upside: { axis: 'x', deg: 180, cam: [Math.PI + 0.7, 0.42] },
};
const scalGPose = () => SensorCal.POSES[scalG.i] || null;
const qTurn = (axis, deg) => { const h = deg * D2R / 2, s = Math.sin(h); return axis === 'x' ? [Math.cos(h), s, 0, 0] : axis === 'y' ? [Math.cos(h), 0, s, 0] : [Math.cos(h), 0, 0, s]; };
const scalGReach = () => Math.max(0.08, typeof cReach === 'number' && cReach > 0 ? cReach : 0.15);

// What the 3D view shows while guiding (view3d.js updateScene): { at, q, dist }, or null when not guiding.
function scalGuideView() {
  if (!scalG.on) { if (scalG.arrow) scalG.arrow.visible = false; if (scalG.front) scalG.front.visible = false; return null; }
  scalGFront();
  const p = scalGPose(), m = p && GUIDE_MOVE[p.id];
  let k = 0;
  if (m && m.axis) {
    if (scalG.got) k = 1;                                    // taken: it stays in the pose
    else {                                                   // the move, over and over: level, turn, hold, back
      const u = ((performance.now() - scalG.t0) / 1000) % 3.4, ease = x => x * x * (3 - 2 * x);
      k = u < 0.6 ? 0 : u < 1.9 ? ease((u - 0.6) / 1.3) : u < 2.9 ? 1 : 1 - ease((u - 2.9) / 0.5);
    }
  }
  const q = qnorm(qmul(qTurn('z', scalG.yaw * R2D), m && m.axis ? qTurn(m.axis, m.deg * k) : [1, 0, 0, 0]));
  scalGArrow(p, m);
  return { at: scalG.at, q, dist: scalGReach() * 5.2 };
}
// The arrow: an arc round the drone the way the pose turns it (from the point that moves most), or for level, one
// pointing down onto it. In the drone's heading, on top of everything.
function scalGArrow(p, m) {
  if (typeof THREE === 'undefined' || typeof scene === 'undefined') return;
  const key = p ? p.id : '';
  if (scalG.arrowFor !== key) {
    if (scalG.arrow) { scene.remove(scalG.arrow); scalG.arrow.traverse(o => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); }); }
    scalG.arrow = null; scalG.arrowFor = key;
    if (!p) return;
    const R = scalGReach() * 1.25, g = new THREE.Group(), col = new THREE.Color(tok('--accent') || '#2f6fed');
    const mat = new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.9, depthTest: false });
    if (m && m.axis) {
      const n = 40, pts = [], th = m.deg * D2R;
      for (let i = 0; i <= n; i++) {                         // the moving point: the nose (pitch) or the side going down (roll; over the top, upside down)
        const t = th * i / n;
        pts.push(m.axis === 'y' ? new THREE.Vector3(R * Math.cos(t), 0, -R * Math.sin(t)) : new THREE.Vector3(0, (m.deg < 0 || m.deg === 180 ? R : -R) * Math.cos(t), (m.deg < 0 || m.deg === 180 ? R : -R) * Math.sin(t)));
      }
      const curve = new THREE.CatmullRomCurve3(pts), tube = new THREE.Mesh(new THREE.TubeGeometry(curve, 48, R * 0.035, 8), mat);
      const tip = new THREE.Mesh(new THREE.ConeGeometry(R * 0.09, R * 0.24, 16), mat);
      const end = curve.getPoint(1), dir = curve.getTangent(1);
      tip.position.copy(end).addScaledVector(dir, R * 0.08); tip.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
      g.add(tube, tip);
    } else {
      const shaft = new THREE.Mesh(new THREE.CylinderGeometry(R * 0.035, R * 0.035, R * 0.5, 12), mat);
      shaft.rotation.x = Math.PI / 2; shaft.position.set(0, 0, R * 0.95);
      const tip = new THREE.Mesh(new THREE.ConeGeometry(R * 0.1, R * 0.24, 16), mat);
      tip.rotation.x = -Math.PI / 2; tip.position.set(0, 0, R * 0.6);
      g.add(shaft, tip);
    }
    g.traverse(o => { o.renderOrder = 40; });
    scene.add(g); scalG.arrow = g;
  }
  if (!scalG.arrow) return;
  scalG.arrow.visible = !scalG.got;
  scalG.arrow.position.set(...scalG.at); scalG.arrow.rotation.set(0, 0, scalG.yaw);
  const pulse = 0.55 + 0.4 * (0.5 + 0.5 * Math.sin(performance.now() / 260));
  scalG.arrow.traverse(o => { if (o.material) o.material.opacity = pulse; });
}
// Which way is the front: an arrow out past the nose, labelled, on the drone itself (it turns with each pose).
function scalGFront() {
  if (typeof THREE === 'undefined' || typeof drone === 'undefined') return;
  if (!scalG.front) {
    const R = scalGReach(), col = new THREE.Color(tok('--warn') || '#e07a1f'), g = new THREE.Group();
    const a = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(R * 0.55, 0, 0), R * 0.75, col, R * 0.22, R * 0.13);
    const cv = document.createElement('canvas'); cv.width = 256; cv.height = 96;
    const cx = cv.getContext('2d'); cx.font = '600 58px system-ui, sans-serif'; cx.textAlign = 'center'; cx.textBaseline = 'middle';
    cx.lineWidth = 10; cx.strokeStyle = 'rgba(255,255,255,0.9)'; cx.strokeText('Front', 128, 50); cx.fillStyle = '#' + col.getHexString(); cx.fillText('Front', 128, 50);
    const label = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(cv), depthTest: false, transparent: true }));
    label.scale.set(R * 0.62, R * 0.23, 1); label.position.set(R * 1.45, 0, R * 0.12);
    g.add(a, label);
    g.traverse(o => { if (o.material) { o.material.depthTest = false; o.material.transparent = true; } o.renderOrder = 41; });
    scalG.front = g;
  }
  if (scalG.front.parent !== drone) drone.add(scalG.front);   // (the drone is built again when the design changes)
  scalG.front.visible = true;
}
// The camera for a pose: close, from where the move shows best (it glides there, as the view buttons do).
function scalGCamera(p) {
  const m = GUIDE_MOVE[p.id]; if (!m || typeof cam === 'undefined') return;
  let d = m.cam[0] + scalG.yaw - cam.az; d = Math.atan2(Math.sin(d), Math.cos(d));
  cam.anim = { az0: cam.az, el0: cam.el, daz: d, el1: m.cam[1], t0: performance.now(), dur: 700 };
  if (typeof view !== 'undefined' && view.chase) view.chase = false;
}

/* ───────── the steps ───────── */
function scalGuideStart() {
  if (scal.busy) return;
  if (typeof editMode !== 'undefined' && editMode && typeof setEditMode === 'function') setEditMode(false);
  const R = qmat(S.q);
  scalG.cam0 = typeof cam !== 'undefined' ? { az: cam.az, el: cam.el, dist: cam.dist } : null;   // (where the camera goes back to after)
  Object.assign(scalG, { on: true, done: false, i: 0, at: S.p.slice(), yaw: Math.atan2(R[3], R[0]), msg: '', tone: '', got: 0, last: null });
  scal.poses = {}; scal.applied = false; scal.busy = 'guide';
  scalGStep(0); scalGRun(); renderSensorCal();
}
function scalGStep(i) {
  scalG.i = i; scalG.last = null; scalG.got = 0; scalG.t0 = performance.now();
  const p = scalGPose();
  if (!p) { scalGFinish(); return; }
  scalG.wait = performance.now() + (i ? 2200 : 1200);         // (time to take the pose before it's asked)
  scalG.msg = 'Do as the drone shows, then hold it still.'; scalG.tone = '';
  scalGCamera(p); scalGRender();
}
// The camera back where it was before the guide.
function scalGCameraBack() {
  const c = scalG.cam0; scalG.cam0 = null; if (!c || typeof cam === 'undefined') return;
  let d = c.az - cam.az; d = Math.atan2(Math.sin(d), Math.cos(d));
  cam.anim = { az0: cam.az, el0: cam.el, daz: d, el1: c.el, dist0: cam.dist, dist1: c.dist, t0: performance.now(), dur: 700 };
}
function scalGStop() {
  scalGCameraBack();
  scalG.on = false; scalG.done = false; scalG.run++; scal.busy = '';
  if (scalG.box) scalG.box.hidden = true;
  renderSensorCal();
}
function scalGFinish() {
  scalGCameraBack();
  scalG.on = false; scalG.done = true; scalG.run++; scal.busy = '';
  const r = scalResult();
  scalG.msg = !r ? 'Not enough poses.' : r.error ? r.error : r.ok ? 'All poses measured. Check the numbers, then Apply.' : `${r.worst.label} reads ${(r.worst.err * 100).toFixed(1)}% off 1 g: measure it again (Tune).`;
  scalG.tone = r && !r.error && r.ok ? 'ok' : 'warn';
  scalGRender(); renderSensorCal();
}
// The loop: ask, check, take two alike, go on.
async function scalGRun() {
  const run = ++scalG.run;
  while (scalG.on && run === scalG.run) {
    const p = scalGPose(); if (!p) break;
    if (performance.now() < scalG.wait) { await new Promise(r => setTimeout(r, 120)); continue; }
    const a = await scalAsk();
    if (!scalG.on || run !== scalG.run || p !== scalGPose()) continue;   // (stopped, or Back / Skip meanwhile)
    if (!a) { scalG.msg = scalWhere() === 'sim' ? 'No answer: is the simulation running?' : 'No answer from the drone: is the link up?'; scalG.tone = 'bad'; scalGRender(); await new Promise(r => setTimeout(r, 1000)); continue; }
    if (a.why) { scalG.msg = a.why; scalG.tone = 'bad'; scalGRender(); await new Promise(r => setTimeout(r, 1500)); continue; }
    if (a.error) { scalG.last = null; scalG.msg = /moving/.test(a.error) ? 'Hold it still…' : a.error; scalG.tone = /moving/.test(a.error) ? '' : 'bad'; scalGRender(); continue; }
    const c = SensorCal.check(scal.poses, p.id, a);
    if (!c.ok) { scalG.last = null; scalG.msg = c.msg.charAt(0).toUpperCase() + c.msg.slice(1); scalG.tone = 'warn'; scalGRender(); continue; }
    const L = scalG.last, same = L && Math.acos(Math.max(-1, Math.min(1, (L.acc[0] * a.acc[0] + L.acc[1] * a.acc[1] + L.acc[2] * a.acc[2]) / (Math.hypot(...L.acc) * Math.hypot(...a.acc))))) * R2D < 2;
    if (!same) { scalG.last = a; scalG.msg = 'That\'s it: hold it there…'; scalG.tone = ''; scalGRender(); continue; }
    scal.poses[p.id] = { acc: L.acc.map((x, i) => (x + a.acc[i]) / 2), gyro: L.gyro.map((x, i) => (x + a.gyro[i]) / 2) };
    scalG.got = performance.now(); scalG.msg = '✓ Got it.'; scalG.tone = 'ok'; scalGRender(); renderSensorCal();
    await new Promise(r => setTimeout(r, 1300));
    if (scalG.on && run === scalG.run && p === scalGPose()) scalGStep(scalG.i + 1);
  }
}

/* ───────── the card over the 3D view ───────── */
function scalGRender() {
  let box = scalG.box;
  if (!box) {
    const host = (typeof vpEl !== 'undefined' && vpEl.parentElement) || document.body;
    box = scalG.box = el('section', { class: 'scal-guide', id: 'scalGuide', 'aria-label': 'Guided sensor calibration', hidden: true });
    host.append(box);
  }
  const p = scalGPose(), n = SensorCal.POSES.length;
  box.hidden = !scalG.on && !scalG.done; if (box.hidden) return;
  box.textContent = '';
  const dots = el('ol', { class: 'scal-guide-dots', 'aria-hidden': 'true' }, ...SensorCal.POSES.map((q, i) => el('li', { class: scal.poses[q.id] ? 'is-done' : scalG.on && i === scalG.i ? 'is-now' : '' })));
  if (scalG.on && p) {
    box.append(el('div', { class: 'scal-guide-step', text: `Sensor calibration · step ${scalG.i + 1} of ${n}${p.need ? '' : ' (optional)'}` }), dots,
      el('h3', { text: p.label }), el('p', { class: 'scal-guide-how', text: p.how }),
      el('p', { class: 'ui-status' + (scalG.tone ? ' ' + scalG.tone : ''), role: 'status', 'aria-live': 'polite', text: scalG.msg }));
    const tools = el('div', { class: 'design-tools' });
    if (scalG.i > 0) tools.append(UI.button({ class: 'btn sm', type: 'button', text: 'Back', onclick: () => { delete scal.poses[SensorCal.POSES[scalG.i - 1].id]; scalGStep(scalG.i - 1); renderSensorCal(); } }));
    if (!p.need) tools.append(UI.button({ class: 'btn sm', type: 'button', text: 'Skip', title: 'Without it the offsets and gains of each axis can\'t all be measured', onclick: () => scalGStep(scalG.i + 1) }));
    tools.append(UI.button({ class: 'btn sm', type: 'button', text: 'Stop', onclick: scalGStop }));
    box.append(tools);
  } else {
    const r = scalResult();
    box.append(el('div', { class: 'scal-guide-step', text: 'Sensor calibration' }), dots, el('h3', { text: 'Done' }),
      el('p', { class: 'ui-status' + (scalG.tone ? ' ' + scalG.tone : ''), role: 'status', text: scalG.msg }));
    const tools = el('div', { class: 'design-tools' });
    if (r && !r.error && r.ok && !scal.applied) tools.append(UI.button({ class: 'btn sm', type: 'button', text: 'Apply', onclick: () => { scalApply(); scalGSync(); } }));
    if (scal.applied && scalWhere() === 'live' && typeof cardRadioPi === 'function' && cardRadioPi()) tools.append(UI.button({ class: 'btn sm', type: 'button', disabled: scalUploading() || undefined, text: scalUploading() ? 'Uploading…' : 'Upload design', onclick: scalUpload }));
    tools.append(UI.button({ class: 'btn sm', type: 'button', text: 'Close', title: scalUploading() ? 'The upload goes on; the drone card shows it' : null, onclick: () => { scalG.done = false; scalGRender(); } }));
    box.append(tools);
  }
}
