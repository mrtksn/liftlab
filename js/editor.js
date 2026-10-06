'use strict';
// Edit mode. The simulation pauses and the airframe is drawn level in its own body axes. Hover to
// highlight a part, click to select it, then drag the handles: arrows move along one axis, squares
// move within a plane, rings rotate. Moving or turning a part carries everything attached to it; what it's
// attached to stays put. A selected servo shows what it carries swinging through its travel. Positions snap
// to 5 mm and angles to 5°; hold Shift for 1 mm / 1°.

let editMode = false, editWasRunning = true;
const edit = { hover: null, sel: null, drag: null, down: null, focusId: null, refocus: false, focusLocal: null, focusShift: null };   // focus: where the view is centred while editing
const raycaster = new THREE.Raycaster(); raycaster.params.Line.threshold = 0.004; raycaster.params.Points.threshold = 0.004;
const AXES = [[1, 0, 0], [0, 1, 0], [0, 0, 1]], AXIS_NAME = ['X', 'Y', 'Z'];
let gizmo = null, travelG = null, hoverBox = null, selBox = null;
const handleMeshes = [];   // invisible, generous hit shapes with userData { kind, axis }
const handleVis = [];      // the visible shapes, to highlight the active one

const rotAxesFor = c => c.type === 'motor' || c.type === 'link' || c.type === 'joint' ? [0, 1, 2]
  : c.type === 'sensor' && (c.kind === 'imu' || c.kind === 'mag' || c.kind === 'flow') ? [0, 1, 2] : [];

function buildGizmo() {
  if (gizmo) { scene.remove(gizmo); gizmo.traverse(o => o.geometry && o.geometry.dispose()); }
  handleMeshes.length = 0; handleVis.length = 0;
  gizmo = new THREE.Group(); gizmo.visible = false; gizmo.renderOrder = 20;
  const cols = ['--ax-x', '--ax-y', '--ax-z'].map(colorOf);
  const vis = (geo, color, opacity = 1) => { const m = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color, transparent: true, opacity, depthTest: false, depthWrite: false, side: THREE.DoubleSide })); m.renderOrder = 20; return m; };
  const hit = (geo, data) => { const m = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ transparent: true, opacity: 0, depthTest: false, depthWrite: false, colorWrite: false, side: THREE.DoubleSide })); m.userData = data; handleMeshes.push(m); return m; };
  const toAxis = (obj, i) => { if (i === 0) obj.rotation.z = -Math.PI / 2; else if (i === 2) obj.rotation.x = Math.PI / 2; };  // geometry built along +Y
  for (let i = 0; i < 3; i++) {
    const g = new THREE.Group(); g.userData = { kind: 'move', axis: i, group: 'move' };
    const shaft = vis(new THREE.CylinderGeometry(0.012, 0.012, 0.72, 10).translate(0, 0.36, 0), cols[i]);
    const head = vis(new THREE.ConeGeometry(0.045, 0.16, 16).translate(0, 0.8, 0), cols[i]);
    g.add(shaft, head, hit(new THREE.CylinderGeometry(0.07, 0.07, 0.9, 8).translate(0, 0.45, 0), { kind: 'move', axis: i }));
    toAxis(g, i); gizmo.add(g); handleVis.push({ kind: 'move', axis: i, meshes: [shaft, head], base: 1 });
  }
  const planes = [[0, 1, 2], [1, 2, 0], [0, 2, 1]];   // [u, v, normal]
  for (const [u, v, n] of planes) {
    const sq = vis(new THREE.PlaneGeometry(0.16, 0.16), cols[n], 0.35);
    const o = [0, 0, 0]; o[u] = 0.26; o[v] = 0.26; sq.position.set(...o);
    if (n === 0) sq.rotation.y = Math.PI / 2; else if (n === 1) sq.rotation.x = Math.PI / 2;
    const h = hit(new THREE.PlaneGeometry(0.2, 0.2), { kind: 'plane', axis: n }); h.position.copy(sq.position); h.rotation.copy(sq.rotation);
    const g = new THREE.Group(); g.userData = { group: 'plane' }; g.add(sq, h); gizmo.add(g);
    handleVis.push({ kind: 'plane', axis: n, meshes: [sq], base: 0.35 });
  }
  for (let i = 0; i < 3; i++) {
    const ring = vis(new THREE.TorusGeometry(0.58, 0.009, 6, 72), cols[i], 0.9);
    const h = hit(new THREE.TorusGeometry(0.58, 0.05, 6, 48), { kind: 'rot', axis: i });
    const g = new THREE.Group(); g.userData = { group: 'rot', axis: i }; g.add(ring, h);
    if (i === 0) g.rotation.y = Math.PI / 2; else if (i === 1) g.rotation.x = Math.PI / 2;   // torus lies in XY (normal Z)
    gizmo.add(g); handleVis.push({ kind: 'rot', axis: i, meshes: [ring], base: 0.9 });
  }
  scene.add(gizmo);
  if (travelG) { scene.remove(travelG); travelG.traverse(o => o.geometry && o.geometry.dispose()); }
  travelG = new THREE.Group(); travelG.visible = false; travelG.renderOrder = 20;
  for (const side of [1, -1]) {   // the fan's two ends: drag either to change the travel
    const g = new THREE.Group(); g.userData = { side };
    const dot = vis(new THREE.SphereGeometry(1, 18, 12), colorOf('--swing'), 1);
    const ring = vis(new THREE.TorusGeometry(1.5, 0.22, 6, 24), colorOf('--swing'), 0.5);
    g.add(dot, ring, hit(new THREE.SphereGeometry(2.6, 10, 8), { kind: 'travel', axis: -1, side }));
    travelG.add(g); handleVis.push({ kind: 'travel', axis: -1, meshes: [dot, ring], base: 1 });
  }
  scene.add(travelG);
  if (!hoverBox) { hoverBox = new THREE.Box3Helper(new THREE.Box3(), 0xffffff); hoverBox.visible = false; scene.add(hoverBox); selBox = new THREE.Box3Helper(new THREE.Box3(), 0xffffff); selBox.visible = false; scene.add(selBox); }
  hoverBox.material.color = colorOf('--ink-2'); selBox.material.color = colorOf('--accent');
  hoverBox.material.depthTest = false; selBox.material.depthTest = false; hoverBox.renderOrder = selBox.renderOrder = 19;
}

function setEditMode(on) {
  if (on === editMode) return;
  editMode = on;
  if (on) { editWasRunning = running; running = false; releaseAll(); UI_PANELS.editor.select('air'); }
  else { running = editWasRunning; selectComp(null); setHover(null); }
  renderRun();
  $('#tEdit').setAttribute('aria-pressed', String(on));
  $('.view').classList.toggle('editing', on);
  $('#editBar').hidden = !on;
  updateEditMsg();
  renderDesignHud();
}

function selectComp(id) {
  const prev = edit.sel; edit.sel = id;
  document.querySelectorAll('.comp.sel').forEach(n => n.classList.remove('sel'));
  if (id != null) {
    openSet.add(id);
    const card = document.querySelector(`[data-id="${id}"]`);
    if (card) { const fresh = compCard(compById(id)); keepFocus(() => card.replaceWith(fresh)); fresh.classList.add('sel'); fresh.scrollIntoView({ block: 'nearest' }); }
  }
  if (prev !== id) updateEditMsg();
}
function setHover(id) {
  if (edit.hover === id) return; edit.hover = id;
  const tip = $('#pickTip'); if (id == null) tip.hidden = true;
}
function updateEditMsg() {
  const m = $('#editMsg'); if (!m) return;
  const c = compById(edit.sel);
  renderEditTools(c);
  $('#editBar').classList.toggle('servo', !!c && c.type === 'joint');
  if (!c) { m.textContent = 'Click a part to select it. The simulation is paused.'; return; }
  if (c.type === 'joint') {
    const kids = descendants(c);
    m.textContent = '';
    m.append(el('strong', { text: c.name }), ` on ${mountName(c)}`,
      el('span', { class: 'sub', text: kids.length ? ` · carries ${kids.map(k => k.name).join(', ')}. The rings turn it with everything on it; the dots at the fan's ends change how far it swings.`
        : ' · carries nothing yet. Set a part\'s "Attached to" to it, or drag the part onto it in the list.' }));
    return;
  }
  const rot = rotAxesFor(c).length ? ', rings rotate' : '';
  m.textContent = `${c.name}: drag the arrows or squares to move${rot}. Shift for fine steps, Esc to deselect.`;
}
// The swing preview: the selected servo moves through its travel, carrying its parts, so you can see what it
// does. It plays on its own; drag the slider to hold it at an angle.
const swingPrev = { play: true, th: 0, t0: 0, id: null };
function previewAngle(j) {
  if (!editMode || edit.sel !== j.id) return 0;
  const R = j.range * D2R;
  if (swingPrev.play) swingPrev.th = R * Math.sin((performance.now() - swingPrev.t0) / 1000 * 2 * Math.PI / 2.8);
  return clamp(swingPrev.th, -R, R);
}
const previewing = () => { const c = editMode && compById(edit.sel); return c && c.type === 'joint' ? c : null; };
// A selected servo, laid out in rows: which way it swings, how far, and the preview.
function renderEditTools(c) {
  const box = $('#editTools'); if (!box) return;
  box.textContent = ''; box.hidden = !(c && c.type === 'joint' && descendants(c).length); if (box.hidden) return;
  if (swingPrev.id !== c.id) { swingPrev.id = c.id; swingPrev.play = true; swingPrev.t0 = performance.now(); }
  const changed = key => { edited(c, key); refreshCard(c); updateEditMsg(); };
  const cur = swingPreset(c), w = swingOf(c);
  const btn = (text, title, on) => { const b = UI.button( { class: 'btn', type: 'button', title, 'aria-label': title, text }); b.addEventListener('click', on); return b; };
  const row = (label, ...kids) => el('div', { class: 'st-row' }, el('span', { class: 'st-lab', text: label }), el('div', { class: 'st-ctl' }, ...kids));

  const seg = el('div', { class: 'seg', role: 'group', 'aria-label': 'Swing direction' });
  for (const o of swingPresets(c)) {
    const b = UI.button( { type: 'button', 'aria-pressed': String(!!cur && cur.k === o.k), text: o.label });
    b.addEventListener('click', () => { setSwing(c, o.deg, 0); changed('swing'); });
    seg.append(b);
  }
  const ang = UI.input( { type: 'number', class: 'num', min: -180, max: 180, step: 5, value: String(Math.round(w.swing)), 'aria-label': 'Swing direction in degrees' });
  ang.addEventListener('change', () => { const v = parseFloat(ang.value); if (isFinite(v)) { setSwing(c, clamp(v, -180, 180)); changed('swing'); } });
  ang.addEventListener('keydown', e => { if (e.key === 'Enter') ang.blur(); });
  const nudge = d => { setSwing(c, ((Math.round(swingOf(c).swing) + d + 540) % 360) - 180); changed('swing'); };
  const angBox = el('span', { class: 'ang', title: `Relative to ${mountName(c)}; 0° swings toward ${swingRefName(c)}` },
    btn('⟲', 'Turn the swing 15° one way', () => nudge(-15)), ang, el('span', { class: 'unit', text: '°' }), btn('⟳', 'Turn the swing 15° the other way', () => nudge(15)));

  const step = d => { c.range = clamp(c.range + d, 5, 90); changed('range'); };
  const travel = el('span', { class: 'range-step' }, btn('−', 'Less travel (5°)', () => step(-5)), el('b', { class: 'rv', text: `±${c.range}°` }), btn('+', 'More travel (5°)', () => step(5)));

  const R = c.range;
  const play = btn(swingPrev.play ? '❚❚' : '▶', swingPrev.play ? 'Pause the preview' : 'Play the preview', () => {
    swingPrev.play = !swingPrev.play;
    if (swingPrev.play) swingPrev.t0 = performance.now() - 1000 * 2.8 / (2 * Math.PI) * Math.asin(clamp(swingPrev.th / (R * D2R), -1, 1));
    renderEditTools(c);
  });
  play.id = 'swingPlay';
  const scrub = UI.input( { type: 'range', id: 'swingScrub', min: -R, max: R, step: 1, value: String(Math.round(swingPrev.th * R2D)), 'aria-label': 'Preview angle' });
  scrub.addEventListener('input', () => { swingPrev.play = false; swingPrev.th = parseFloat(scrub.value) * D2R; play.textContent = '▶'; play.title = 'Play the preview'; play.setAttribute('aria-label', 'Play the preview'); });
  const at = el('span', { class: 'rv', id: 'swingAt' });
  const hold = (label, deg) => btn(label, `Hold at ${label}`, () => { swingPrev.play = false; swingPrev.th = deg * D2R; renderEditTools(c); });

  box.append(row('Swings', seg, angBox),
    row('Travel', travel, el('span', { class: 'st-note', text: `either side of where its parts sit now` })),
    row('Preview', play, hold(`−${R}°`, -R), scrub, hold(`+${R}°`, R), at));
}

/* ───────── picking ───────── */
function rayFrom(e) {
  const r = renderer.domElement.getBoundingClientRect();
  const ndc = new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
  raycaster.setFromCamera(ndc, camera); return raycaster.ray;
}
function pickHandle(e) {
  if (!gizmo || !gizmo.visible) return null;
  rayFrom(e);
  const live = handleMeshes.filter(h => { let o = h; while (o) { if (!o.visible) return false; o = o.parent; } return true; });
  const hits = raycaster.intersectObjects(live, false);
  const hit = hits.find(h => h.object.userData.kind === 'travel') || hits.find(h => h.object.userData.kind === 'plane') || hits[0];   // small targets (a fan's end dots, the squares) win over what's behind them
  return hit ? hit.object.userData : null;
}
// What a pixel shows: the nearest solid, visible surface of the airframe. Lines (arrows, arcs, paths),
// hidden shapes (a rotor's wake column) and overlays (a servo's fan) don't count, so only what's under the
// pointer is picked; the frame itself hides what's behind it.
function solidVisible(o) {
  if (!o.isMesh || !o.material || o.material.opacity === 0 || o.material.colorWrite === false) return false;
  for (let p = o; p; p = p.parent) { if (!p.visible || p.userData.noPick) return false; }
  return true;
}
function pickComp(e) {
  rayFrom(e);
  for (const h of raycaster.intersectObject(drone, true)) {
    if (!solidVisible(h.object)) continue;
    let o = h.object; while (o && o.userData.compId == null && o !== drone) o = o.parent;
    return o && o.userData.compId != null ? o.userData.compId : null;
  }
  return null;
}
// The box round a part: just its visible solid shapes (not its hidden wake or arrow).
const boxOf = (g, box) => { box.makeEmpty(); g.updateWorldMatrix(true, true); g.traverse(o => { if (solidVisible(o)) box.expandByObject(o); }); return box; };

/* ───────── dragging ───────── */
function closestOnAxis(ray, P0, a) {  // parameter t of the point on line P0 + t·a closest to the ray
  const w0 = new THREE.Vector3().subVectors(P0, ray.origin), b = a.dot(ray.direction), d = a.dot(w0), e = ray.direction.dot(w0);
  const den = 1 - b * b; if (den < 1e-6) return null;
  return (b * e - d) / den;
}
function onPlane(ray, P0, n) {
  const den = ray.direction.dot(n); if (Math.abs(den) < 1e-6) return null;
  const t = new THREE.Vector3().subVectors(P0, ray.origin).dot(n) / den; if (t < 0) return null;
  return ray.origin.clone().addScaledVector(ray.direction, t);
}
const snapTo = (v, s) => Math.round(v / s) * s;
function startDrag(h, e) {
  const c = compById(edit.sel); if (!c) return false;
  const P0 = gizmo.position.clone(), ray = rayFrom(e), a = h.axis >= 0 ? new THREE.Vector3(...AXES[h.axis]) : null;
  const d = { h, c, P0, pos0: c.pos.slice(), tilt0: c.tilt, az0: c.az, hinge0: c.hingeAz, mount0: c.mount ? c.mount.slice() : null,
    axis0: c.type === 'joint' ? jointAxis(c) : null, dir0: c.type === 'link' ? linkDir(c) : null };
  if (h.kind === 'travel') { const sw = servoSweep(c); d.n = new THREE.Vector3(...sw.a); d.rest = sw.rest; d.v = crs(sw.a, sw.rest); swingPrev.play = false; }
  else if (h.kind === 'move') { d.t0 = closestOnAxis(ray, P0, a); if (d.t0 == null) return false; }
  else { d.p0 = onPlane(ray, P0, a); if (!d.p0) return false; d.snap = poseSnap(c); }
  if (typeof undo !== 'undefined') undo.lastKey = null;   // a drag is its own undo step
  edit.drag = d; vpEl.setPointerCapture(e.pointerId); vpEl.style.cursor = 'grabbing';
  highlightHandle(h); return true;
}
function dragTo(e) {
  const d = edit.drag, c = d.c, fine = e.shiftKey, ray = rayFrom(e), a = d.h.axis >= 0 ? new THREE.Vector3(...AXES[d.h.axis]) : null;
  const step = fine ? 0.001 : 0.005, astep = (fine ? 1 : 5) * D2R;
  if (d.h.kind === 'travel') {   // drag a fan end round the hinge: the travel is how far it is from 0°
    const p = onPlane(ray, d.P0, d.n) || onPlane(ray, d.P0, d.n.clone().negate()); if (!p) return;
    const v = [p.x - d.P0.x, p.y - d.P0.y, p.z - d.P0.z], th = Math.atan2(dot(v, d.v), dot(v, d.rest)) * R2D;
    c.range = clamp(snapTo(Math.abs(th), fine ? 1 : 5), 5, 90); swingPrev.th = Math.sign(th || 1) * c.range * D2R;
    edited(c, 'range');
  } else if (d.h.kind === 'move') {
    const t = closestOnAxis(ray, d.P0, a); if (t == null) return;
    c.pos[d.h.axis] = +clamp(snapTo(d.pos0[d.h.axis] + t - d.t0, step), -2, 2).toFixed(4);
    edited(c, 'x');
  } else if (d.h.kind === 'plane') {
    const p = onPlane(ray, d.P0, a); if (!p) return;
    for (let i = 0; i < 3; i++) if (i !== d.h.axis) c.pos[i] = +clamp(snapTo(d.pos0[i] + (p.getComponent(i) - d.p0.getComponent(i)), step), -2, 2).toFixed(4);
    edited(c, 'x');
  } else {
    const p = onPlane(ray, d.P0, a); if (!p) return;
    const v0 = d.p0.clone().sub(d.P0), v1 = p.clone().sub(d.P0);
    const ang = snapTo(Math.atan2(new THREE.Vector3().crossVectors(v0, v1).dot(a), v0.dot(v1)), astep);
    const R = axisAngleR(AXES[d.h.axis], ang);
    // The whole turn from where the drag began: put the part and its load back, then turn them together.
    poseRestore(d.snap); for (const [x] of d.snap) snapHolder(x);
    turnPart(c, R);
    edited(c, { motor: 'tilt', joint: 'hingeAz', link: 'laz', sensor: 'mr' }[c.type]);
  }
  refreshCard(c); showDragReadout(c);
}
function endDrag() { if (edit.drag && edit.drag.h.kind === 'travel') { swingPrev.play = true; swingPrev.t0 = performance.now(); } edit.drag = null; vpEl.style.cursor = ''; highlightHandle(null); save(); updateEditMsg(); }
function highlightHandle(h) {
  for (const v of handleVis) {
    const on = h && v.kind === h.kind && v.axis === h.axis;
    for (const m of v.meshes) m.material.opacity = on ? 1 : h ? v.base * 0.35 : v.base;
  }
}
function showDragReadout(c) {
  const f = x => x.toFixed(3);
  let t = `${c.name}: position (${f(c.pos[0])}, ${f(c.pos[1])}, ${f(c.pos[2])}) m`;
  if (edit.drag && edit.drag.h.kind === 'travel') t = `${c.name}: travels ±${c.range}° either side of 0°. Shift for 1° steps.`;
  else if (edit.drag && edit.drag.h.kind === 'rot') {
    if (c.type === 'motor') t = `${c.name}: axis tilted ${c.tilt.toFixed(1)}° toward ${c.az.toFixed(1)}°`;
    else if (c.type === 'joint') { const p = swingPreset(c); t = `${c.name}: turned with ${descendants(c).length ? 'everything on it' : 'nothing on it yet'}; swings ${p ? p.label.toLowerCase() : 'at ' + swingOf(c).swing.toFixed(0) + '°'} (hinge ${c.hingeAz.toFixed(0)}°, ${c.hingeEl.toFixed(0)}° up). Shift for 1° steps.`; }
    else if (c.type === 'link') t = `${c.name}: pointing toward ${c.az.toFixed(1)}°, ${c.el.toFixed(1)}° up`;
    else t = `${c.name}: mount roll ${c.mount[0].toFixed(1)}°, pitch ${c.mount[1].toFixed(1)}°, yaw ${c.mount[2].toFixed(1)}°`;
  }
  $('#editMsg').textContent = t;
}

/* ───────── pointer hooks, called by the viewport's handlers ───────── */
function editPointerDown(e) {
  if (!editMode || e.button !== 0) return false;
  const h = pickHandle(e);
  if (h && startDrag(h, e)) return true;
  edit.down = { x: e.clientX, y: e.clientY };
  return false;                                          // let the camera orbit
}
function editPointerMove(e, orbiting) {
  if (!editMode) return false;
  if (edit.drag) { dragTo(e); return true; }
  if (orbiting) { setHover(null); return false; }
  const h = pickHandle(e);
  if (h) { vpEl.style.cursor = 'grab'; setHover(null); return false; }
  const id = pickComp(e); setHover(id);
  vpEl.style.cursor = id != null ? 'pointer' : '';
  if (id != null) {
    const c = compById(id), tip = $('#pickTip'), r = $('.view').getBoundingClientRect();
    tip.textContent = `${c.name} · ${tagOf(c)}` + (c.type === 'motor' ? ` · ${c.push ? 'pushes' : 'pulls'} · ${c.spin > 0 ? 'CCW' : 'CW'}` : ''); tip.hidden = false;
    tip.style.left = (e.clientX - r.left + 14) + 'px'; tip.style.top = (e.clientY - r.top + 12) + 'px';
  }
  return false;
}
function editPointerUp(e) {
  if (!editMode) return false;
  if (edit.drag) { endDrag(); return true; }
  if (edit.down && Math.hypot(e.clientX - edit.down.x, e.clientY - edit.down.y) < 5) { edit.refocus = true; selectComp(pickComp(e)); }   // a click (not a drag) centres the view on what it picks
  edit.down = null; return false;
}

/* ───────── per-frame ───────── */
function updateEditView() {
  if (!gizmo) return;
  if (!editMode) { gizmo.visible = travelG.visible = hoverBox.visible = selBox.visible = false; return; }
  if (edit.sel != null && !compById(edit.sel)) selectComp(null);
  const c = compById(edit.sel), g = c && pickGroups.get(c.id);
  selBox.visible = !!g; if (g) boxOf(g, selBox.box);
  const hg = edit.hover != null && edit.hover !== edit.sel ? pickGroups.get(edit.hover) : null;
  hoverBox.visible = !!hg; if (hg) boxOf(hg, hoverBox.box);
  gizmo.visible = !!c;
  travelG.visible = !!c && c.type === 'joint' && descendants(c).length > 0;
  if (travelG.visible) {   // on the fan's two ends
    const sw = servoSweep(c), v = crs(sw.a, sw.rest), R = c.range * D2R, k = viewDist(gizmo.position) * 0.16 * 0.045;
    for (const g of travelG.children) {
      const p = add(c.pos, add(scl(sw.rest, sw.radius * Math.cos(R)), scl(v, sw.radius * Math.sin(R) * g.userData.side)));
      g.position.copy(drone.localToWorld(new THREE.Vector3(...p))); g.scale.setScalar(k);
      g.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), new THREE.Vector3(...sw.a));
    }
  }
  if (c && c.type === 'joint') {
    const sc = $('#swingScrub'), at = $('#swingAt'), th = previewAngle(c) * R2D;
    if (sc && swingPrev.play) sc.value = String(Math.round(th)); if (at) at.textContent = `${th >= 0 ? '+' : '−'}${Math.abs(th).toFixed(0)}°`;
  }
  if (c) {
    gizmo.position.copy(drone.localToWorld(new THREE.Vector3(...c.pos)));
    gizmo.scale.setScalar(viewDist(gizmo.position) * 0.16);
    const rots = rotAxesFor(c);
    gizmo.children.forEach(ch => {
      if (ch.userData.group === 'rot') ch.visible = rots.includes(ch.userData.axis);
    });
  }
}

/* ───────── controls ───────── */
$('#tEdit').addEventListener('click', () => setEditMode(!editMode));
$('#editDone').addEventListener('click', () => setEditMode(false));
window.addEventListener('keydown', e => {
  if (e.defaultPrevented || e.metaKey || e.ctrlKey || e.altKey || typingIn(e.target)) return;
  if (e.code === 'KeyE' && !e.repeat) { setEditMode(!editMode); e.preventDefault(); }
  else if (e.code === 'Escape' && editMode) { if (edit.sel != null) selectComp(null); else setEditMode(false); }
});
