'use strict';
let partEditPick = null;
const partMarks = new THREE.Group(); partMarks.userData.noPick = true; scene.add(partMarks);
let partMarksOwner = null;
function partMovePoint(c, id, pos, frame = false) {
  const before = partPointRest(c, id);
  c.points ||= partPoints(c).map(p => ({ ...p, pos: p.pos.slice() }));
  c.points.find(p => p.id === id).pos = pos;
  const d = sub(partPointRest(c, id), before), children = frame ? cfg.comps.filter(x => !parentOf(x)) : childrenOf(c);
  for (const x of children.filter(x => x.parentPoint === id)) { x.pos = add(x.pos, d); shiftSubtree(x, d); }
  if (!frame && c.selfPoint === id) { c.pos = sub(c.pos, d); shiftSubtree(c, scl(d, -1)); }
  if (!frame) { snapHolder(c); for (const x of descendants(c)) snapHolder(x); }
}
function partPickBegin(c, kind) {
  setEditMode(true); selectComp(c.id); partEditPick = { id: c.id, kind };
  $('#editMsg').textContent = 'Click a surface of ' + c.name + ' to place ' + (kind === 'cog' ? 'its center of mass' : 'the mounting point') + '. Use XYZ for interior points; Escape cancels.';
}
function partPickSurface(e) {
  const task = partEditPick, c = task && compById(task.id), group = c && pickGroups.get(c.id); if (!group) return;
  rayFrom(e); const hit = raycaster.intersectObject(group, true).find(h => solidVisible(h.object)); if (!hit) return;
  const body = drone.worldToLocal(hit.point.clone()).toArray(), pose = poseOf(c, previewAngle);
  let local = m3v(m3T(partRot(c)), m3v(m3T(pose.R), sub(body, pose.p)));
  if (task.kind === 'cog') { if (c.type === 'link') local[0] -= c.length / 2; c.cog = local; }
  else partMovePoint(c, task.kind, local);
  partEditPick = null; edited(c, task.kind === 'cog' ? 'cog' : 'points'); renderComps(); updateEditMsg();
}
function partPhysicalView() {
  const c = editMode && !partImport.active && compById(edit.sel);
  partMarks.visible = !!c; if (!c) { partEditPick = null; return; }
  const points = partPoints(c), count = points.length + (c.type === 'hang' ? 0 : 1);
  if (partMarksOwner !== c || partMarks.children.length !== count) {
    for (const m of partMarks.children) m.material.dispose(); disposeGroup(partMarks); partMarksOwner = c;
    for (let i = 0; i < count; i++) {
      const m = new THREE.Mesh(new THREE.SphereGeometry(.006, 12, 8), new THREE.MeshBasicMaterial({ color: colorOf(i === 0 && c.type !== 'hang' ? '--grav' : '--accent'), depthTest: false })); m.renderOrder = 35; partMarks.add(m);
    }
  }
  const places = [...(c.type === 'hang' ? [] : [partMassRest(c)]), ...points.map(p => partPointRest(c, p.id))];
  for (const [i, pos] of places.entries()) partMarks.children[i].position.copy(drone.localToWorld(new THREE.Vector3(...posePoint(c, pos, previewAngle).p)));
}
function partMountOptions(c) {
  const original=partImport.active&&partImport.editId?compById(partImport.editId):null;
  const out = [['', 'Frame']];
  for (const p of partPoints(cfg.frame)) out.push(['|' + p.id, 'Frame · ' + p.name]);
  for (const h of cfg.comps.filter(h => canAttach(original || c, h))) {
    out.push([String(h.id), h.name]);
    for (const p of partPoints(h)) out.push([h.id + '|' + p.id, h.name + ' · ' + p.name]);
  }
  return out;
}
function partChooseMount(c, value) {
  const [id, point] = value.split('|'); attachTo(c, id ? compById(+id) : null, point || null);
}
function partVecInputs(id, label, value, set, step = .001) {
  return el('div', { class: 'part-vector' }, ...value.map((v, k) => {
    const input = UI.input({ type: 'number', id: id + k, value: +v.toFixed(5), step, 'aria-label': label + ' ' + 'XYZ'[k] });
    input.addEventListener('change', () => { const n = Number(input.value); if (input.value.trim() && Number.isFinite(n) && Math.abs(n) <= 5000) set(k, n); else input.value = v; });
    return el('label', {}, 'XYZ'[k], input);
  }));
}
function partMassFields(c, frame = false) {
  const key = frame ? 'frame' : c.id, box = el('div', { class: 'part-physical' });
  const changed = k => {
    if (frame) { undoKey = 'frame:' + k; recomputeProps(); cPts = contactPoints(); rebuildDrone(); refreshEnvelope(); renderMass(); save(); }
    else edited(c, k);
  };
  if (c.type !== 'hang') {
    box.append(el('p', { class: 'hint', text: frame ? 'Center of mass: metres from the hub in the drone’s body axes. Frame mounting points use those same axes.' : c.type === 'link' ? 'Center of mass: offset from the rod’s middle in its own axes (X along the rod).' : 'Center of mass: offset from the part’s drawing origin, in metres in its own axes. Moving this does not move its shape.' }),
      partVecInputs('part-cog-' + key + '-', 'Center of mass', partCog(c), (k, v) => { c.cog = partCog(c).slice(); c.cog[k] = v; changed('cog'); }));
    if (!frame) box.append(UI.button({ class: 'btn btn-sm', onclick: () => partPickBegin(c, 'cog') }, 'Pick center on part'));
  } else return UI.details({ title: 'Center of mass', class: 'fold sub' }, el('p', { class: 'hint', text: 'A cable mass is a point mass: its center of mass is the hanging ball. Its position fields set the cable’s attachment. Mount other parts on rigid masses.' }));
  const points = el('div', { class: 'part-point-list' });
  const render = () => {
    points.replaceChildren(...partPoints(c).map(p => {
      const name = UI.input({ type: 'text', value: p.name, maxlength: 60, 'aria-label': 'Attachment point name' });
      const materialize = () => { c.points ||= partPoints(c).map(x => ({ ...x, pos: x.pos.slice() })); return c.points.find(x => x.id === p.id); };
      name.addEventListener('change', () => { materialize().name = name.value.trim() || 'Point'; changed('points'); });
      const used = !frame && (childrenOf(c).some(x => x.parentPoint === p.id || partPoints(c).length === 1) || c.selfPoint === p.id) || frame && cfg.comps.some(x => !parentOf(x) && x.parentPoint === p.id);
      return el('div', { class: 'part-point' }, name,
        partVecInputs('part-point-' + key + '-' + p.id + '-', 'Attachment ' + p.name, p.pos, (k, v) => {
          const pos = partPoints(c).find(x => x.id === p.id).pos.slice(); pos[k] = v; partMovePoint(c, p.id, pos, frame);
          changed('points');
        }),
        ...(!frame ? [UI.button({ class: 'btn btn-sm', onclick: () => partPickBegin(c, p.id) }, 'Pick on part')] : []),
        UI.button({ class: 'btn btn-sm', disabled: used || undefined, title: used ? 'This point is in use; choose another mounting point before removing it' : 'Remove this attachment point', onclick: () => { materialize(); c.points = c.points.filter(x => x.id !== p.id); changed('points'); render(); } }, 'Remove point'));
    }));
  };
  render();
  box.append(el('p', { class: 'hint', text: 'Named mounting points: other parts can attach here. Coordinates are local metres from this part’s drawing origin. Each part has one parent; multiple children may use its points.' }), points,
    UI.button({ class: 'btn btn-sm', id: 'part-point-add-' + key, onclick: () => {
      c.points = partPoints(c).map(p => ({ ...p, pos: p.pos.slice() }));
      c.points.push({ id: newObjId('p'), name: 'Point ' + (c.points.length + 1), pos: [0, 0, 0] }); changed('points'); render();
    } }, 'Add attachment point'));
  return UI.details({ title: 'Center of mass & attachment points', class: 'fold sub', open: !!c.model }, box);
}
window.addEventListener('keydown', e => { if (partEditPick && e.code === 'Escape') { partEditPick = null; updateEditMsg(); } });
function partModelFields(c) {
  const length=objectScaleField({id:'part-scale-'+c.id,numberId:'part-size-'+c.id,max:2,limit:10,get:()=>Math.max(...c.size),set:n=>{partScale(c,n);snapHolder(c);for(const x of descendants(c))snapHolder(x);edited(c,'size');}});
  const category = UI.select({ id: 'part-category-' + c.id, 'aria-label': 'Category' }, ...Object.entries(PART_CATEGORIES).map(([k, v]) => el('option', { value: k, text: v })));
  category.value = c.category || 'payload'; category.addEventListener('change', () => { c.category = category.value; edited(c, 'category'); });
  const rotation = partVecInputs('part-rotation-' + c.id + '-', 'Object rotation in degrees', c.rotation || [0, 0, 0], (k, v) => {
    const next = (c.rotation || [0, 0, 0]).slice(); next[k] = v;
    partOrientMass(c, eulerR(...next)); edited(c, 'rotation');
  }, 5);
  return el('div', { class: 'part-model-fields' }, UI.button({class:'btn primary',id:'objectEditCopy-'+c.id,onclick:()=>objectUseStart(objectAsset(c),'drone',c)},'Edit object copy…'), UI.field({ label: 'Category' }, category), length.node,
    el('p', { class: 'hint', text: c.size.map(v => v.toFixed(3)).join(' × ') + ' m · ' + c.model.boxes.length + ' solid boxes. Size changes keep the chosen weight.' }),
    el('p', { class: 'hint', text: 'Rotation about X, Y and Z (degrees)' }), rotation,
    UI.button({ class: 'btn btn-sm', onclick: async () => { designNote(await objectLibrarySaveCopy(c) ? 'Saved drone use defaults in Object library.' : 'This browser could not save the reusable part. Export the design to keep it.'); } }, 'Save drone use defaults'));
}
function partOwnMountField(c) {
  const points = partPoints(c); if (!points.length) return null;
  const sel = UI.select({ id: 'part-self-' + c.id }, el('option', { value: '', text: 'Drawing origin' }), ...points.map(p => el('option', { value: p.id, text: p.name })));
  sel.value = c.selfPoint || ''; sel.addEventListener('change', () => { attachTo(c, parentOf(c), c.parentPoint || null, sel.value || null); structural(); });
  return UI.field({ label: 'Mount this part by' }, sel);
}
