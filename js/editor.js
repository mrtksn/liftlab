'use strict';
// Edit mode. The simulation pauses and the airframe is drawn level in its own body axes. Hover to
// highlight a part, click to select it, then drag the handles: arrows move along one axis, squares
// move within a plane, rings rotate. Positions snap to 5 mm and angles to 5°; hold Shift for 1 mm / 1°.

let editMode = false, editWasRunning = true;
const edit = { hover: null, sel: null, drag: null, down: null };
const raycaster = new THREE.Raycaster();
const AXES = [[1, 0, 0], [0, 1, 0], [0, 0, 1]], AXIS_NAME = ['X', 'Y', 'Z'];
let gizmo = null, hoverBox = null, selBox = null;
const handleMeshes = [];   // invisible, generous hit shapes with userData { kind, axis }
const handleVis = [];      // the visible shapes, to highlight the active one

const rotAxesFor = c => c.type === 'motor' || c.type === 'link' ? [0, 1, 2]   // a servo turns by its axis knob instead
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
    const g = new THREE.Group(); g.userData = { group: 'move' }; g.add(sq, h); gizmo.add(g);
    handleVis.push({ kind: 'plane', axis: n, meshes: [sq], base: 0.35 });
  }
  for (let i = 0; i < 3; i++) {
    const ring = vis(new THREE.TorusGeometry(0.58, 0.009, 6, 72), cols[i], 0.9);
    const h = hit(new THREE.TorusGeometry(0.58, 0.05, 6, 48), { kind: 'rot', axis: i });
    const g = new THREE.Group(); g.userData = { group: 'rot', axis: i }; g.add(ring, h);
    if (i === 0) g.rotation.y = Math.PI / 2; else if (i === 1) g.rotation.x = Math.PI / 2;   // torus lies in XY (normal Z)
    gizmo.add(g); handleVis.push({ kind: 'rot', axis: i, meshes: [ring], base: 0.9 });
  }
  {   // a servo's hinge axis: an arrow from the pivot with a knob to drag it round to any direction
    const acol = colorOf('--accent');
    const shaft = vis(new THREE.CylinderGeometry(0.014, 0.014, 0.55, 10).translate(0, 0.275, 0), acol, 0.9);
    const knob = vis(new THREE.SphereGeometry(0.06, 20, 14).translate(0, 0.62, 0), acol, 1);
    const tail = vis(new THREE.CylinderGeometry(0.008, 0.008, 0.3, 8).translate(0, -0.15, 0), acol, 0.4);
    const g = new THREE.Group(); g.userData = { group: 'axis' };
    g.add(shaft, knob, tail, hit(new THREE.SphereGeometry(0.13, 12, 8).translate(0, 0.62, 0), { kind: 'axis', axis: -1 }));
    gizmo.add(g); handleVis.push({ kind: 'axis', axis: -1, meshes: [shaft, knob], base: 0.9 });
  }
  scene.add(gizmo);
  if (!hoverBox) { hoverBox = new THREE.BoxHelper(undefined, 0xffffff); hoverBox.visible = false; scene.add(hoverBox); selBox = new THREE.BoxHelper(undefined, 0xffffff); selBox.visible = false; scene.add(selBox); }
  hoverBox.material.color = colorOf('--ink-2'); selBox.material.color = colorOf('--accent');
  hoverBox.material.depthTest = false; selBox.material.depthTest = false; hoverBox.renderOrder = selBox.renderOrder = 19;
}

function setEditMode(on) {
  if (on === editMode) return;
  editMode = on;
  if (on) { editWasRunning = running; running = false; releaseAll(); showTab('air'); }
  else { running = editWasRunning; selectComp(null); setHover(null); }
  $('#runBtn').textContent = running ? 'Pause' : 'Run';
  $('#tEdit').setAttribute('aria-pressed', String(on));
  $('.view').classList.toggle('editing', on);
  $('#editBar').hidden = !on;
  updateEditMsg();
}

function selectComp(id) {
  const prev = edit.sel; edit.sel = id;
  document.querySelectorAll('.comp.sel').forEach(n => n.classList.remove('sel'));
  if (id != null) {
    openSet.add(id);
    const card = document.querySelector(`[data-id="${id}"]`);
    if (card) { const fresh = compCard(compById(id)); card.replaceWith(fresh); fresh.classList.add('sel'); fresh.scrollIntoView({ block: 'nearest' }); }
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
  if (!c) { m.textContent = 'Click a part to select it. The simulation is paused.'; return; }
  if (c.type === 'joint') { m.textContent = `${c.name}: drag the knob on its axis arrow to point it anywhere, or type the angles (relative to ${mountName(c)}).`; return; }
  const rot = rotAxesFor(c).length ? ', rings rotate' : '';
  m.textContent = `${c.name}: drag the arrows or squares to move${rot}. Shift for fine steps, Esc to deselect.`;
}
// A selected servo: its hinge axis relative to its mount, and how far it swings either way.
function renderEditTools(c) {
  const box = $('#editTools'); if (!box) return;
  box.textContent = ''; box.hidden = !(c && c.type === 'joint'); if (box.hidden) return;
  const changed = key => { edited(c, key); refreshCard(c); updateEditMsg(); };
  const r = hingeRel(c);
  const angle = (key, label, min, max) => {
    const i = el('input', { type: 'number', class: 'num', min, max, step: 5, value: String(+r[key].toFixed(1)), 'aria-label': label, title: label });
    i.addEventListener('change', () => { const v = parseFloat(i.value); if (!isFinite(v)) return; const n = hingeRel(c); setHingeRel(c, key === 'az' ? clamp(v, -180, 180) : n.az, key === 'el' ? clamp(v, -90, 90) : n.el); changed('hingeAz'); });
    i.addEventListener('keydown', e => { if (e.key === 'Enter') i.blur(); });
    return el('label', { class: 'ang' }, el('span', { text: label }), i, el('span', { class: 'unit', text: '°' }));
  };
  const step = d => { c.range = clamp(c.range + d, 5, 90); changed('range'); };
  const minus = el('button', { class: 'btn', type: 'button', title: 'Less travel (5°)', 'aria-label': 'Less travel', text: '−' }), plus = el('button', { class: 'btn', type: 'button', title: 'More travel (5°)', 'aria-label': 'More travel', text: '+' });
  minus.addEventListener('click', () => step(-5)); plus.addEventListener('click', () => step(5));
  box.append(angle('az', 'Heading', -180, 180), angle('el', 'Tilt', -90, 90), el('span', { class: 'range-step' }, minus, el('b', { class: 'rv', text: `±${c.range}°` }), plus));
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
  const hit = hits.find(h => h.object.userData.kind === 'plane') || hits[0];   // the small squares win over what's behind them
  return hit ? hit.object.userData : null;
}
function pickComp(e) {
  rayFrom(e);
  const hits = raycaster.intersectObjects([...pickGroups.values()], true);
  for (const h of hits) { let o = h.object; while (o && o.userData.compId == null) o = o.parent; if (o) return o.userData.compId; }
  return null;
}

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
  if (h.kind === 'axis') d.r = 0.62 * gizmo.scale.x;
  else if (h.kind === 'move') { d.t0 = closestOnAxis(ray, P0, a); if (d.t0 == null) return false; }
  else { d.p0 = onPlane(ray, P0, a); if (!d.p0) return false; }
  if (typeof undo !== 'undefined') undo.lastKey = null;   // a drag is its own undo step
  edit.drag = d; vpEl.setPointerCapture(e.pointerId); vpEl.style.cursor = 'grabbing';
  highlightHandle(h); return true;
}
// Where the pointer meets the sphere the axis knob moves on: of the two crossings, the one nearer the axis
// now, so the knob can go round behind the pivot; off the sphere, the nearest point on it.
function onSphere(ray, C, r, near) {
  const oc = ray.origin.clone().sub(C), b = oc.dot(ray.direction), disc = b * b - (oc.lengthSq() - r * r);
  if (disc < 0) return ray.origin.clone().addScaledVector(ray.direction, -b).sub(C).setLength(r);
  const s = Math.sqrt(disc), p1 = oc.clone().addScaledVector(ray.direction, -b - s), p2 = oc.clone().addScaledVector(ray.direction, -b + s);
  return p1.dot(near) >= p2.dot(near) ? p1 : p2;
}
function dragTo(e) {
  const d = edit.drag, c = d.c, fine = e.shiftKey, ray = rayFrom(e), a = d.h.axis >= 0 ? new THREE.Vector3(...AXES[d.h.axis]) : null;
  const step = fine ? 0.001 : 0.005, astep = (fine ? 1 : 5) * D2R;
  if (d.h.kind === 'axis') {   // point the hinge axis wherever the knob is dragged (in edit mode body axes are world axes)
    const v = onSphere(ray, d.P0, d.r, new THREE.Vector3(...jointAxis(c))).normalize();
    const rel = m3v(m3T(mountFrame(c)), [v.x, v.y, v.z]), st = fine ? 1 : 5;
    const el = snapTo(Math.asin(clamp(rel[2], -1, 1)) * R2D, st), az = Math.abs(el) > 89.9 ? hingeRel(c).az : snapTo(Math.atan2(rel[1], rel[0]) * R2D, st);
    setHingeRel(c, az, el); edited(c, 'hingeAz');
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
    if (c.type === 'motor') {
      const t = d.tilt0 * D2R, z = d.az0 * D2R;
      const dir = m3v(R, [Math.sin(t) * Math.cos(z), Math.sin(t) * Math.sin(z), Math.cos(t)]);
      c.tilt = +(Math.acos(clamp(dir[2], -1, 1)) * R2D).toFixed(1);
      if (c.tilt > 0.05) c.az = +(Math.atan2(dir[1], dir[0]) * R2D).toFixed(1);
      edited(c, 'tilt');
    } else if (c.type === 'joint') {   // turn the hinge axis itself: horizontal, vertical or anything between
      setDirAzEl(c, m3v(R, d.axis0), 'hingeAz', 'hingeEl');
      edited(c, 'hingeAz');
    } else if (c.type === 'link') {    // swing the rod about its base; what's on it swings along
      setDirAzEl(c, m3v(R, d.dir0), 'az', 'el');
      edited(c, 'laz');                  // edited() swings the carried parts along
    } else {
      c.mount = eulerFromR(m3m(R, eulerR(...d.mount0))).map(x => +x.toFixed(1));
      edited(c, 'mr');
    }
  }
  refreshCard(c); showDragReadout(c);
}
function endDrag() { edit.drag = null; vpEl.style.cursor = ''; highlightHandle(null); save(); updateEditMsg(); }
function highlightHandle(h) {
  for (const v of handleVis) {
    const on = h && v.kind === h.kind && v.axis === h.axis;
    for (const m of v.meshes) m.material.opacity = on ? 1 : h ? v.base * 0.35 : v.base;
  }
}
function showDragReadout(c) {
  const f = x => x.toFixed(3);
  let t = `${c.name}: position (${f(c.pos[0])}, ${f(c.pos[1])}, ${f(c.pos[2])}) m`;
  if (edit.drag && edit.drag.h.kind === 'axis') { const r = hingeRel(c); t = `${c.name}: hinge axis heading ${r.az.toFixed(0)}°, ${r.el.toFixed(0)}° up from ${mountName(c)}. Shift for 1° steps.`; }
  else if (edit.drag && edit.drag.h.kind === 'rot') {
    if (c.type === 'motor') t = `${c.name}: axis tilted ${c.tilt.toFixed(1)}° toward ${c.az.toFixed(1)}°`;
    else if (c.type === 'joint') t = `${c.name}: hinge axis toward ${c.hingeAz.toFixed(1)}°, tilted up ${c.hingeEl.toFixed(1)}°`;
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
    tip.textContent = `${c.name} · ${tagOf(c)}`; tip.hidden = false;
    tip.style.left = (e.clientX - r.left + 14) + 'px'; tip.style.top = (e.clientY - r.top + 12) + 'px';
  }
  return false;
}
function editPointerUp(e) {
  if (!editMode) return false;
  if (edit.drag) { endDrag(); return true; }
  if (edit.down && Math.hypot(e.clientX - edit.down.x, e.clientY - edit.down.y) < 5) selectComp(pickComp(e));
  edit.down = null; return false;
}

/* ───────── per-frame ───────── */
function updateEditView() {
  if (!gizmo) return;
  if (!editMode) { gizmo.visible = hoverBox.visible = selBox.visible = false; return; }
  if (edit.sel != null && !compById(edit.sel)) selectComp(null);
  const c = compById(edit.sel), g = c && pickGroups.get(c.id);
  selBox.visible = !!g; if (g) selBox.setFromObject(g);
  const hg = edit.hover != null && edit.hover !== edit.sel ? pickGroups.get(edit.hover) : null;
  hoverBox.visible = !!hg; if (hg) hoverBox.setFromObject(hg);
  gizmo.visible = !!c;
  if (c) {
    gizmo.position.copy(drone.localToWorld(new THREE.Vector3(...c.pos)));
    gizmo.scale.setScalar(camera.position.distanceTo(gizmo.position) * 0.16);
    const rots = rotAxesFor(c);
    gizmo.children.forEach(ch => {
      if (ch.userData.group === 'rot') ch.visible = rots.includes(ch.userData.axis);
      else if (ch.userData.group === 'axis') { ch.visible = c.type === 'joint'; if (ch.visible) ch.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(...jointAxis(c))); }
    });
  }
}

/* ───────── controls ───────── */
$('#tEdit').addEventListener('click', () => setEditMode(!editMode));
$('#editDone').addEventListener('click', () => setEditMode(false));
window.addEventListener('keydown', e => {
  if (e.metaKey || e.ctrlKey || e.altKey || typingIn(e.target)) return;
  if (e.code === 'KeyE' && !e.repeat) { setEditMode(!editMode); e.preventDefault(); }
  else if (e.code === 'Escape' && editMode) { if (edit.sel != null) selectComp(null); else setEditMode(false); }
});
