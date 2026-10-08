'use strict';
// Editing the world's objects: World settings → Edit objects. The simulation pauses (as in the drone's edit
// mode) and the objects can be picked and moved with the same handles: arrows move along one axis, squares
// within a plane, and the ring turns an object about the vertical. Positions snap to 5 cm (50 cm in the
// full-scale city) and turns to 5°; hold Shift for a tenth of that / 1°. New objects come from a 3D model
// file (world-objects.js), which is made solid as it's placed. Turning an object, resizing it or changing its
// detail makes it solid again; moving it just carries its solid shape along.

const wedit = { on: false, sel: null, hover: null, wasRunning: false, changed: false, showSolid: true, busy: false, say: '' };
const worldEditOn = () => wedit.on;

function worldEditSet(on) {
  if (on === wedit.on) return;
  if (on && (typeof fleet === 'undefined' || !fleet.ready || fleet.selected || (typeof liveOn === 'function' && liveOn()))) return;
  wedit.on = on;
  if (on) { wedit.wasRunning = running; running = false; wedit.changed = false; releaseAll(); }
  else {
    worldEditSelect(null); setHover(null);
    if (wedit.changed) worldEditClearDrones();
    running = wedit.wasRunning; fleetSave();
  }
  renderRun();
  $('.view').classList.toggle('editing', on);
  $('#worldEditBar').hidden = !on;
  $('#worldPanel').classList.toggle('editing-objects', on);
  if (on && worldCollapsed) { worldCollapsed = false; worldPanelRender(); }
  worldEditRender(); worldEditMsg();
}
// Drones the objects were moved onto start again (at their targets, or at the start point if that's blocked too).
function worldEditClearDrones() {
  for (const d of fleet.drones) withDrone(d, () => { if (terrainNear(S.p, cReach + 0.05).some(b => b.obj != null)) resetSim(); });
}
function worldEditSelect(id) {
  wedit.sel = id;
  worldObjShowSolid(wedit.showSolid ? worldObj(id) : null);
  worldEditRender(); worldEditMsg();
}
const worldEditChanged = () => { wedit.changed = true; fleetSave(); };
function worldEditMsg(text) {
  const m = $('#worldEditMsg'); if (!m) return;
  const o = worldObj(wedit.sel);
  m.textContent = text || (!o ? 'Click an object to select it, or import a 3D model. The simulation is paused.'
    : `${o.name}: drag the arrows or squares to move${o.base ? ', the ring to turn' : ''}. Shift for fine steps, Esc to deselect.`);
}
const moveStep = fine => (bigWorld() ? 0.5 : 0.05) / (fine ? 10 : 1);

/* ───────── the panel: World settings → Objects ───────── */
function worldEditRender() {
  const box = $('#worldObjBody'); if (!box) return;
  const n = worldObjects.list.length;
  $('#worldObjCount').textContent = n ? String(n) : '';
  if (!wedit.on) {
    box.replaceChildren(
      el('p', { class: 'hint', text: n ? `${n} object${n > 1 ? 's' : ''} from 3D models, solid for the drones.` : 'Bring in 3D models from Blender or any 3D tool; they are made solid for the drones.' }),
      UI.button({ class: 'btn', id: 'worldObjEdit', title: 'Pause and edit the objects: import models, move and turn them', onclick: () => worldEditSet(true) }, 'World editor'));
    return;
  }
  const keep = document.activeElement && box.contains(document.activeElement) ? document.activeElement.id : null;
  const file = UI.input({ type: 'file', id: 'worldObjFile', accept: '.glb,.gltf,.obj,.stl,.bin,.png,.jpg,.jpeg,.webp,.ktx2', multiple: true, hidden: true });
  file.addEventListener('change', () => { const f = file.files; if (f && f.length) worldEditImport(f); file.value = ''; });
  const kids = [
    el('div', { class: 'hrow' },
      UI.button({ class: 'btn primary', id: 'worldObjImport', disabled: wedit.busy || maps.busy || undefined, title: 'Read a 3D model and place it here, solid', onclick: () => file.click() }, wedit.busy ? 'Reading…' : 'Import 3D model…'),
      UI.button({class:'btn',id:'worldObjectLibrary',onclick:()=>objectLibraryOpen('world')},'Object library…'),
      UI.button({ class: 'btn', id: 'worldObjDone', disabled:wedit.busy||undefined, title: 'Finish editing; the simulation carries on', onclick: () => worldEditSet(false) }, 'Done'), file),
    el('p', { class: 'hint', text: 'glTF/GLB, OBJ or STL. With a .gltf, pick its .bin and textures too.' }),
    el('p', { class: 'world-obj-say', id: 'worldObjSay', role: 'status', text: wedit.say }),
  ];
  if (n) kids.push(el('ul', { class: 'world-obj-list', 'aria-label': 'Objects' }, ...worldObjects.list.map(o => el('li', {},
    UI.button({ class: 'world-obj-item', 'aria-pressed': String(o.id === wedit.sel), onclick: () => { worldEditSelect(o.id === wedit.sel ? null : o.id); if (wedit.sel) worldEditLookAt(o); } },
      el('span', { text: o.name }), el('small', { text: o.loading ? 'reading…' : o.collision==='mesh' ? 'mesh' : `${o.boxes.length} box${o.boxes.length === 1 ? '' : 'es'}` }))))));
  const o = worldObj(wedit.sel);
  if (o) kids.push(worldObjFields(o));
  box.replaceChildren(...kids);
  if (keep && document.getElementById(keep)) document.getElementById(keep).focus();
}
function solidDetailControl(value, onChange, disabled = false) {
  return el('div', { class: 'seg', role: 'group', 'aria-label': 'Solid shape detail' }, ...Object.entries(WORLD_OBJ_DETAIL).map(([key, d]) =>
    UI.button({ 'data-solid-detail': key, 'aria-pressed': String(key === value), disabled: disabled || undefined, onclick: () => onChange(key) }, d.label)));
}
// State stays with each editor; this UI block is identical for world and drone solids.
function solidShapeControls({ detail, shown, id, onDetail, onShown, disabled = false, description, collision = 'boxes', onCollision }) {
  const checkbox = UI.input({ type: 'checkbox', id }); checkbox.checked = shown;
  checkbox.addEventListener('change', () => onShown(checkbox.checked));
  return el('div', { class: 'solid-shape-controls' },
    onCollision ? UI.field({label:'Collision geometry'}, el('div',{class:'seg',role:'group','aria-label':'Collision geometry'}, ...[['mesh','Follows model'],['boxes','Filled boxes']].map(([key,label])=>UI.button({'data-collision':key,'aria-pressed':String(collision===key),disabled:disabled||undefined,onclick:()=>onCollision(key)},label)))) : null,
    collision === 'boxes' ? UI.field({ label: 'Box detail' }, solidDetailControl(detail, onDetail, disabled)) : el('p',{class:'hint',text:'Uses the model’s triangle surfaces. Closed meshes enclose a volume; open meshes act as surfaces.'}),
    description ? el('p', { class: 'hint', text: description }) : null,
    el('label', { class: 'world-obj-chk' }, checkbox, ' Show the solid shape'));
}
function worldObjFields(o) {
  const can = !!o.base, f2 = x => (+x).toFixed(2), num = (id, label, value, step, set, attrs = {}) => {
    const inp = UI.input({ type: 'number', class: 'num', id, step, value: String(value), 'aria-label': label, ...attrs });
    inp.addEventListener('change', () => { const v = parseFloat(inp.value); if (Number.isFinite(v)) { set(v); worldEditChanged(); worldEditRender(); } else inp.value = String(value); });
    inp.addEventListener('keydown', e => { if (e.key === 'Enter') inp.blur(); });
    return inp;
  };
  const row = (label, ...k) => el('div', { class: 'world-obj-row' }, el('span', { class: 'world-obj-lab', text: label }), el('div', { class: 'world-obj-ctl' }, ...k));
  const name = UI.input({ type: 'text', id: 'worldObjName', maxlength: 60, value: o.name, 'aria-label': 'Name' });
  name.addEventListener('change', () => { o.name = name.value.trim().slice(0, 60) || 'Object'; worldObjectsChanged(); worldEditChanged(); worldEditRender(); worldEditMsg(); });
  const at = [0, 1, 2].map(i => num('worldObjPos' + i, 'XYZ'[i] + ' position in metres', f2(o.pos[i]), moveStep(false), v => { o.pos[i] = clamp(v, -5000, 5000); worldEditMoved(o); }));
  const turn = num('worldObjYaw', 'Turn in degrees', Math.round(o.yaw * 10) / 10, 5, v => { o.yaw = ((v % 360) + 540) % 360 - 180; worldObjSolidify(o); worldEditSolidShown(o); }, { disabled: !can || undefined });
  const scale=objectScaleField({id:'worldObjSize',numberId:'worldObjSizeNumber',disabled:!can,get:()=>Math.max(...o.size)*o.scale,set:n=>{
    const ratio=n/(Math.max(...o.size)*o.scale);o.scale*=ratio;o.units='custom';o.h*=ratio;
    o.boxes=o.boxes.map(b=>({lo:b.lo.map(v=>v*ratio),hi:b.hi.map(v=>v*ratio)}));
    if(o.triangles)o.triangles=o.triangles.map(v=>v*ratio);
    placeGroup(o);worldObjectsChanged();worldEditSolidShown(o);worldEditChanged();
    $('#worldCopyDims').textContent=o.size.map(v=>(v*o.scale).toFixed(2)).join(' × ')+' m (width × depth × height)';
  }});
  const dims = o.size.map(x => x * o.scale);
  const voxel = o.h ? (o.h < 0.1 ? `${Math.round(o.h * 1000)} mm` : `${o.h.toFixed(2)} m`) : '';
  const notes = [];
  if (o.coarsened) notes.push(`${WORLD_OBJ_DETAIL[o.detail].label} would make more than ${WORLD_OBJ_MAX_BOXES} boxes: it's ${WORLD_OBJ_DETAIL[o.coarsened].label.toLowerCase()}.`);
  if (o.missing) notes.push('Its model file isn\'t in this browser, so it\'s drawn as its solid shape. It can be moved, not turned or resized.');
  if (o.unsaved) notes.push('This browser couldn\'t keep the model file: next time the object is drawn as its solid shape.');
  return el('div', { class: 'world-obj-fields' },
    row('Name', name),
    row('Position', ...at, el('span', { class: 'unit', text: 'm' })),
    row('Turn', turn, el('span', { class: 'unit', text: '°' })),
    scale.node,
    el('p', { class: 'hint world-obj-dims', id:'worldCopyDims', text: `${dims.map(f2).join(' × ')} m (width × depth × height)` }),
    solidShapeControls({ detail: o.detail, shown: wedit.showSolid, id: 'worldObjSolid', disabled: !can, collision:o.collision||'boxes',
      onCollision:key=>{o.collision=key;worldObjSolidify(o);worldEditSolidShown(o);worldEditChanged();worldEditRender();},
      onDetail: k => { if(k !== o.detail) { o.detail = k; worldObjSolidify(o); worldEditSolidShown(o); worldEditChanged(); worldEditRender(); } },
      onShown: shown => { wedit.showSolid = shown; worldEditSolidShown(o); },
      description: o.collision==='mesh' ? `${o.triangles.length/9} collision triangles; mass is fixed to the world.` : `${o.boxes.length} box${o.boxes.length === 1 ? '' : 'es'}${voxel ? ` from ${voxel} voxels` : ''}.` }),
    ...notes.map(t => el('p', { class: 'hint world-obj-note', text: t })),
    el('div', { class: 'hrow world-obj-acts' },
      UI.button({ class: 'btn btn-sm', title: 'Stand it on the ground', onclick: () => { o.pos[2] = -Math.min(0, ...o.boxes.map(b => b.lo[2])); worldEditMoved(o); worldEditChanged(); worldEditRender(); } }, 'Put on the ground'),
      UI.button({ class: 'btn btn-sm', title: 'Another one beside it', onclick: () => { const c = worldObjDuplicate(o); worldEditChanged(); worldEditSelect(c.id); } }, 'Duplicate'),
      UI.button({ class: 'btn btn-sm', id: 'worldObjRemove', title: 'Take it out of the world', onclick: () => { if (!confirm(`Remove ${o.name} from the world?`)) return; worldObjRemove(o); worldEditChanged(); worldEditSelect(null); } }, 'Remove')));
}
function worldEditMoved(o) { placeGroup(o); worldObjectsChanged(); if (solidVis) solidVis.position.set(...o.pos); }
function worldEditSolidShown(o) { worldObjShowSolid(wedit.showSolid && wedit.sel === o.id ? o : null); }
function worldEditSay(t) { wedit.say = t; const n = $('#worldObjSay'); if (n) n.textContent = t; }
// Centre the view on an object, far enough back to see it whole.
function worldEditLookAt(o) {
  const b = worldObjBounds(o), c = new THREE.Vector3((b.lo[0] + b.hi[0]) / 2, (b.lo[1] + b.hi[1]) / 2, (b.lo[2] + b.hi[2]) / 2);
  cam.target.copy(c); cam.pan.set(0, 0, 0); cam.dist = clamp(Math.max(...sub(b.hi, b.lo)) * 2.2, 0.6, maxDist()); cam.anim = null;
}
async function worldEditImport(files) { return objectImportStart(files,'world'); }

/* ───────── picking and dragging, through the drone editor's handles (editor.js) ───────── */
function pickWorldObj(e) {
  rayFrom(e);
  for (const h of raycaster.intersectObject(worldObjG, true)) {
    if (!h.object.isMesh) continue;
    let g = h.object; while (g && g.userData.worldObjId == null) g = g.parent;
    if (g) return g.userData.worldObjId;
  }
  return null;
}
function worldPointerDown(e) {
  if(partImport.active)return false;
  if (e.button !== 0) return false;
  const h = pickHandle(e);
  if (h && worldStartDrag(h, e)) return true;
  edit.down = { x: e.clientX, y: e.clientY };
  return false;   // let the camera orbit
}
function worldPointerMove(e, orbiting) {
  if(partImport.active)return false;
  if (edit.drag) { if (e.pointerId === edit.drag.pointerId) worldDragTo(e); return true; }
  const tip = $('#pickTip');
  if (orbiting) { wedit.hover = null; tip.hidden = true; return false; }
  if (pickHandle(e)) { vpEl.style.cursor = 'grab'; wedit.hover = null; tip.hidden = true; return false; }
  const id = pickWorldObj(e), o = worldObj(id); wedit.hover = id;
  vpEl.style.cursor = o ? 'pointer' : '';
  tip.hidden = !o;
  if (o) { const r = $('.view').getBoundingClientRect(); tip.textContent = `${o.name} · ${o.boxes.length} boxes`; tip.style.left = (e.clientX - r.left + 14) + 'px'; tip.style.top = (e.clientY - r.top + 12) + 'px'; }
  return false;
}
function worldPointerUp(e) {
  if(partImport.active)return true;
  if (edit.drag) { if (e.pointerId === edit.drag.pointerId) worldEndDrag(); return true; }
  if (edit.down && Math.hypot(e.clientX - edit.down.x, e.clientY - edit.down.y) < 5) worldEditSelect(pickWorldObj(e));
  edit.down = null; return false;
}
function worldStartDrag(h, e) {
  const o = worldObj(wedit.sel); if (!o || h.kind === 'travel') return false;
  const P0 = gizmo.position.clone(), ray = rayFrom(e), a = new THREE.Vector3(...AXES[h.axis]);
  const d = { world: true, h, o, P0, pointerId: e.pointerId, pos0: o.pos.slice(), yaw0: o.yaw };
  if (h.kind === 'move') { d.t0 = closestOnAxis(ray, P0, a); if (d.t0 == null) return false; }
  else { d.p0 = onPlane(ray, P0, a); if (!d.p0) return false; }
  if (h.kind === 'rot' && solidVis) solidVis.visible = false;   // (made solid again when it's let go)
  edit.drag = d; vpEl.setPointerCapture(e.pointerId); vpEl.style.cursor = 'grabbing';
  highlightHandle(h); return true;
}
function worldDragTo(e) {
  const d = edit.drag, o = d.o, fine = e.shiftKey, ray = rayFrom(e), a = new THREE.Vector3(...AXES[d.h.axis]), step = moveStep(fine);
  if (d.h.kind === 'move') {
    const t = closestOnAxis(ray, d.P0, a); if (t == null) return;
    o.pos[d.h.axis] = +clamp(snapTo(d.pos0[d.h.axis] + t - d.t0, step), -5000, 5000).toFixed(4);
  } else if (d.h.kind === 'plane') {
    const p = onPlane(ray, d.P0, a); if (!p) return;
    for (let i = 0; i < 3; i++) if (i !== d.h.axis) o.pos[i] = +clamp(snapTo(d.pos0[i] + p.getComponent(i) - d.p0.getComponent(i), step), -5000, 5000).toFixed(4);
  } else {
    const p = onPlane(ray, d.P0, a); if (!p) return;
    const v0 = d.p0.clone().sub(d.P0), v1 = p.clone().sub(d.P0);
    const ang = snapTo(Math.atan2(new THREE.Vector3().crossVectors(v0, v1).dot(a), v0.dot(v1)) * R2D, fine ? 1 : 5);
    o.yaw = ((d.yaw0 + ang) % 360 + 540) % 360 - 180;
    placeGroup(o);
    worldEditMsg(`${o.name}: turned to ${o.yaw.toFixed(0)}°. Its solid shape follows when you let go.`);
    return;
  }
  worldEditMoved(o);
  worldEditMsg(`${o.name}: at (${o.pos.map(x => x.toFixed(2)).join(', ')}) m`);
}
function worldEndDrag() {
  const d = edit.drag; edit.drag = null; vpEl.style.cursor = ''; highlightHandle(null);
  if (!d) return;
  if (d.h.kind === 'rot') { if (d.o.yaw !== d.yaw0) worldObjSolidify(d.o); worldEditSolidShown(d.o); }
  if (d.o.yaw !== d.yaw0 || d.o.pos.some((x, i) => x !== d.pos0[i])) worldEditChanged();
  worldEditRender(); worldEditMsg();
}
// Each frame (from editor.js updateEditView): the handles on the selected object, the outlines. True while editing.
function worldEditView() {
  if(wedit.on&&partImport.active){travelG.visible=gizmo.visible=selBox.visible=hoverBox.visible=false;return true;}
  if (!wedit.on) return false;
  travelG.visible = false;
  if (wedit.sel && !worldObj(wedit.sel)) worldEditSelect(null);
  const o = worldObj(wedit.sel), hv = wedit.hover !== wedit.sel ? worldObj(wedit.hover) : null;
  const outline = (helper, x) => { helper.visible = !!x; if (x) { const b = worldObjBounds(x); helper.box.min.set(...b.lo); helper.box.max.set(...b.hi); } };
  outline(selBox, o); outline(hoverBox, hv);
  gizmo.visible = !!o;
  if (o) {
    const b = worldObjBounds(o);
    gizmo.position.set(o.pos[0], o.pos[1], o.pos[2] + (b.hi[2] - b.lo[2]) / 2);
    gizmo.scale.setScalar(viewDist(gizmo.position) * 0.16);
    gizmo.children.forEach(ch => { if (ch.userData.group === 'rot') ch.visible = ch.userData.axis === 2 && !!o.base; });
  }
  return true;
}

window.addEventListener('keydown', e => {
  if (!wedit.on || partImport.active || e.defaultPrevented || e.metaKey || e.ctrlKey || e.altKey || typingIn(e.target)) return;
  if (e.code === 'Escape') { if (wedit.sel) worldEditSelect(null); else worldEditSet(false); e.preventDefault(); }
});
$('#worldEditDone').addEventListener('click', () => worldEditSet(false));
worldEditRender();
