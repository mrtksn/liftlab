'use strict';
const partModels = new Map();
const partImport = { draft: null, preview: null, solid: null, showSolid: true, panel: null, busy: false, active: false, wasEditing: false, owner: null, pick: null, generation: 0 };
const PART_LIBRARY_LS = 'liftlab-imported-parts-v1';
let partLibrary = [];
try {
  const saved = JSON.parse(localStorage.getItem(PART_LIBRARY_LS) || '[]');
  if (Array.isArray(saved)) partLibrary = saved.filter(c => { try { validatePartGeometry(c); return !!c.model; } catch { return false; } });
} catch (_) {}
async function partModelBase(c) {
  const key = c.model.fileId + ':' + c.model.up;
  if (!partModels.has(key)) partModels.set(key, (async () => {
    const rec = worldObjects.files.get(c.model.fileId) || await worldFileGet(c.model.fileId);
    if (!rec) throw new Error('Original model file is unavailable');
    worldObjects.files.set(rec.id, rec);
    const { base, size } = standModel(await parseModel(rec), c.model.up);
    base.children[0].position.z -= size[2] / 2; return base;
  })().catch(e => { partModels.delete(key); throw e; }));
  return partModels.get(key);
}
function partCloneModel(base) {
  const clone = base.clone(true);
  clone.traverse(o => { if (o.geometry) o.geometry = o.geometry.clone(); });
  return clone;
}
function partModelVisual(c) {
  const group = new THREE.Group(); group.setRotationFromMatrix(m4of(massRot(c)));
  const fallback = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), mats.mass, c.model.boxes.length), M = new THREE.Matrix4();
  for (const [i, b] of partBoxes(c).entries()) { M.compose(new THREE.Vector3(...partBoxCenter(b)), new THREE.Quaternion(), new THREE.Vector3(...partBoxSize(b))); fallback.setMatrixAt(i, M); }
  group.add(fallback);
  partModelBase(c).then(base => {
    if (!group.parent) return;
    group.remove(fallback); fallback.geometry.dispose();
    const model = partCloneModel(base); model.scale.setScalar(c.model.scale); group.add(model);
  }).catch(() => { group.userData.missingModel = true; });
  return group;
}
function partAssetsRestore(design) {
  if(typeof objectLibraryAdopt === 'function')for(const c of design.comps)if(c.model)objectLibraryAdopt(c);
  for (const f of partAssetsDecode(design)) { worldObjects.files.set(f.id, f); worldFilePut(f).then(ok => { if (!ok) designNote('Model files could not be kept in this browser. Keep the design file.'); }); }
}
async function partDesignWithAssets(design) {
  const packed = [], existing = partAssetsDecode(design);
  for (const id of new Set(design.comps.map(c => c.model?.fileId).filter(Boolean))) {
    const f = worldObjects.files.get(id) || existing.find(f => f.id === id) || await worldFileGet(id);
    if (!f) throw new Error('An original part model is missing. Re-import it before saving or sharing a portable design. Its solid shape is still available.');
    packed.push({ id: f.id, name: f.name, files: f.files.map(p => ({ name: p.name, data: mapEncode(p.data) })) });
  }
  return { ...design, ...(packed.length ? { modelFiles: packed } : {}) };
}
function partSolidifyDraft(c, base) {
  const group = new THREE.Group(), model = partCloneModel(base); model.scale.setScalar(c.model.scale); group.add(model); group.updateMatrixWorld(true);
  const tri = [], v = new THREE.Vector3();
  group.traverse(m => { if (!m.isMesh || !m.geometry?.attributes.position) return;
    const a = m.geometry.attributes.position, ix = m.geometry.index, n = ix ? ix.count : a.count;
    for (let k = 0; k < n - n % 3; k++) { v.fromBufferAttribute(a, ix ? ix.getX(k) : k).applyMatrix4(m.matrixWorld); tri.push(v.x, v.y, v.z); }
  });
  const levels = Object.keys(WORLD_OBJ_DETAIL); let level = levels.indexOf(c.model.detail), result;
  do { result = voxelBoxes(Float32Array.from(tri), WORLD_OBJ_DETAIL[levels[level]].cells); if (result.boxes.length <= WORLD_OBJ_MAX_BOXES) break; } while (level-- > 0);
  disposeGroup(group);
  if (!result.boxes.length) throw new Error('The model has no solid surfaces');
  c.model.triangles=tri; validateCollisionGeometry(c.model.collision,tri);
  c.model.detail = levels[Math.max(0, level)]; c.model.boxes = result.boxes.map(b => [...b.lo, ...b.hi]);
  c.size = c.model.rawSize.map(v => Math.max(.0001, v * c.model.scale));
}
function partDraftRefresh() {
  const c = partImport.draft; if (!c) return;
  if (partImport.preview) { partImport.preview.parent?.remove(partImport.preview); disposeGroup(partImport.preview); }
  const g = partModelVisual(c); g.userData.noPick = true;
  c.pos = partImport.context === 'world' ? [cam.target.x,cam.target.y,c.size[2]/2] : [cReach + c.size[0] / 2 + .1, 0, 0]; g.position.set(...c.pos); (partImport.context === 'world' ? scene : drone).add(g); partImport.preview = g;
  partImport.solid = collisionVisual(c.model.collision,c.model.triangles,partBoxes(c)); partImport.solid.visible = partImport.phase !== 'import' && partImport.showSolid; g.add(partImport.solid);
  const mark = (pos, color) => { const m = new THREE.Mesh(new THREE.SphereGeometry(Math.max(.004, Math.max(...c.size) * .02), 12, 8), new THREE.MeshBasicMaterial({ color, depthTest: false })); m.userData.noPick = true; m.position.set(...pos); m.renderOrder = 30; g.add(m); };
  if(partImport.phase !== 'import'){mark(partCog(c), colorOf('--grav')); for (const p of partPoints(c)) mark(p.pos, colorOf('--accent'));}
  if ($('#partImportDims')) $('#partImportDims').textContent = c.size.map(v => v.toFixed(3)).join(' × ') + ' m · ' + (c.model.collision==='mesh' ? c.model.triangles.length/9+' collision triangles' : c.model.boxes.length + ' solid boxes · ' + WORLD_OBJ_DETAIL[c.model.detail].label);
  if ($('#partImportMass')) $('#partImportMass').value = +c.mass.toFixed(4);
  partImport.panel.querySelectorAll('[data-solid-detail]').forEach(b => b.setAttribute('aria-pressed', String(b.dataset.solidDetail === c.model.detail)));
}
async function partImportDetail(detail) {
  const c = partImport.draft; if (!c || partImport.busy || c.model.detail === detail) return;
  const generation = partImport.generation;
  partImport.busy = true; $('#partImportAdd').disabled = true;
  try {
    const base = await partModelBase(c);
    if (!partImport.active || partImport.generation !== generation) return;
    c.model.detail = detail; partSolidifyDraft(c, base);
    if (partImport.massAuto) c.mass = clamp(partSolidMass(c).volume * 200, .005, 20);
    partDraftRefresh();
  } catch (e) { if (partImport.generation === generation) $('#partImportPick').textContent = 'Could not change the solid shape: ' + (e.message || e); }
  finally { if (partImport.generation === generation) { partImport.busy = false; $('#partImportAdd').disabled = false; } }
}
function partImportPanel() {
  const c = partImport.draft;
  const panel = partImport.panel = el('aside', { class: 'view-overlay part-import-panel', 'aria-label': partImport.phase==='import'?'Import object model':'Object copy properties' });
  panel.append(el('div', { class: 'dialog-toolbar' }, el('h2', { text: partImport.phase === 'import' ? 'Import to object library' : partImport.editId ? 'Edit object copy' : 'Use object on drone' }), UI.button({ class: 'btn btn-sm', id: 'partImportCancel', onclick: partImportCancel }, 'Cancel')));
  if (!c) { panel.append(el('p', { class: 'hint', text: 'Reading and making the model solid…' })); $('.view').append(panel); return; }
  if(partImport.phase === 'import'){objectImportFields(panel,c);$('.view').append(panel);return;}
  const name = UI.input({ type: 'text', id: 'partImportName', value: c.name, maxlength: 60 }); name.addEventListener('input', () => { c.name = name.value.trim() || 'Object'; });
  const category = UI.select({ id: 'partImportCategory' }, ...Object.entries(PART_CATEGORIES).map(([k, v]) => el('option', { value: k, text: v }))); category.value = c.category; category.addEventListener('change', () => { c.category = category.value; });
  const size = objectScaleField({id:'partImportScale',numberId:'partImportSize',max:2,limit:10,get:()=>Math.max(...c.size),set:n=>{
    const span = 2 * cReach + Math.max(...c.size) + .1;
    partScale(c, n); if (partImport.massAuto) c.mass = clamp(partSolidMass(c).volume * 200, .005, 20);
    cam.dist *= (2 * cReach + Math.max(...c.size) + .1) / span;
    partDraftRefresh(); partImportRenderVectors();
  }});
  const mass = UI.input({ type: 'number', id: 'partImportMass', min: .0001, step: .01, value: c.mass }); mass.addEventListener('change', () => { const n = Number(mass.value); if (n > 0 && Number.isFinite(n)) { c.mass = n; partImport.massAuto = false; } else mass.value = c.mass; });
  const rotation=partVecInputs('objectCopyRotation','Object rotation in degrees',c.rotation||[0,0,0],(k,v)=>{c.rotation[k]=v;partDraftRefresh();},5);
  const mount = UI.select({ id: 'partImportMount' }, ...partMountOptions(c).map(([v, t]) => el('option', { value: v, text: t })));mount.value=partImport.mount||'';mount.addEventListener('change',()=>partImport.mount=mount.value);
  panel.append(el('p', { class: 'hint', text: 'Size the object beside the drone, then attach it. Blue surfaces show its collision shape; orange marks its center of mass and blue dots its mounting points.' }),
    UI.field({ label: 'Name' }, name), UI.field({ label: 'Category' }, category), objectRoleFields(c,()=>{partImport.panel.remove();partImportPanel();partDraftRefresh();}), size.node, el('p', { class: 'hint world-obj-dims', id: 'partImportDims' }),
    solidShapeControls({ detail: c.model.detail, shown: partImport.showSolid, id: 'partImportSolid', onDetail: partImportDetail, collision:c.model.collision||'boxes',
      onCollision:key=>{c.model.collision=key;partImport.panel.remove();partImportPanel();partDraftRefresh();},
      onShown: shown => { partImport.showSolid = shown; if (partImport.solid) partImport.solid.visible = shown; } }), UI.field({ label: 'Weight (kg)' }, mass),
    el('p', { class: 'hint', text: 'Starting weight estimates 200 kg/m³ over the solid shape. Adjust it to match the object.' }),
    UI.field({label:'Rotation (°)'},rotation),flightPhysicsFields(c,()=>{}), el('div', { id: 'partImportVectors' }), UI.field({ label: 'Attach to' }, mount),
    UI.button({ class: 'btn primary', id: 'partImportAdd', onclick: partImportCommit }, partImport.editId ? 'Save copy' : 'Attach object'));
  $('.view').append(panel); partImportRenderVectors();
}
function partImportRenderVectors() {
  const c = partImport.draft, box = $('#partImportVectors'); if (!c || !box) return;
  const own = UI.select({ id: 'partImportSelf' }, el('option', { value: '', text: 'Drawing origin' }), ...partPoints(c).map(p => el('option', { value: p.id, text: p.name })));
  own.value = c.selfPoint || ''; own.addEventListener('change', () => { c.selfPoint = own.value || null; });
  box.replaceChildren(el('span', { class: 'lbl', text: 'Center of mass (m)' }), partVecInputs('partImportCog', 'Center of mass', partCog(c), (k, v) => { c.cog[k] = v; partDraftRefresh(); }),
    UI.button({ class: 'btn btn-sm', onclick: () => { partImport.pick = 'cog'; $('#partImportPick').textContent = 'Click a surface of the preview to place its center of mass. Use XYZ for interior points.'; } }, 'Pick center on object'),
    ...partPoints(c).map(p => el('div', { class: 'part-point' },
      (() => { const n = UI.input({ type: 'text', value: p.name, maxlength: 60, 'aria-label': 'Attachment point name' }); n.addEventListener('change', () => { p.name = n.value.trim() || 'Point'; partImportRenderVectors(); }); return n; })(),
      partVecInputs('partImportPoint-' + p.id + '-', 'Attachment ' + p.name, p.pos, (k, v) => { p.pos[k] = v; partDraftRefresh(); }),
      el('div', { class: 'hrow' }, UI.button({ class: 'btn btn-sm', onclick: () => { partImport.pick = p.id; $('#partImportPick').textContent = 'Click a surface of the preview for ' + p.name + '.'; } }, 'Pick on object'), UI.button({ class: 'btn btn-sm', onclick: () => { c.points = c.points.filter(x => x !== p); if (c.selfPoint === p.id) c.selfPoint = null; partImportRenderVectors(); partDraftRefresh(); } }, 'Remove point')))),
    UI.button({ class: 'btn btn-sm', id: 'partImportPointAdd', onclick: () => { c.points.push({ id: newObjId('p'), name: 'Point ' + (c.points.length + 1), pos: [0, 0, 0] }); partImportRenderVectors(); partDraftRefresh(); } }, 'Add attachment point'),
    UI.field({ label: 'Mount object by' }, own), el('p', { class: 'hint', id: 'partImportPick', role: 'status' }));
}
async function partImportStart(files) { return objectImportStart(files,'drone'); }
function partImportEnd() {
  partImport.panel?.remove(); partImport.panel = null;
  $('.work').classList.remove('part-importing');
  if (partImport.preview) { partImport.preview.parent?.remove(partImport.preview); disposeGroup(partImport.preview); }
  partImport.preview = null; partImport.solid = null; partImport.draft = null; partImport.pick = null;
  perspCam.clearViewOffset(); orthoCam.clearViewOffset();
  for (const [e, inert] of partImport.locked || []) e.inert = inert;
  partImport.locked = null; partImport.generation++; partImport.busy = false;
  if (partImport.active) fleet.pendingEdits--; partImport.active = false;
}
function partImportCancel() {
  if (!partImport.active) return;
  const c = partImport.draft, wasEditing = partImport.wasEditing;
  partImportEnd(); if (c && partImport.phase === 'import' && !partImport.asset) { worldObjects.files.delete(c.model.fileId); partModels.delete(c.model.fileId + ':' + c.model.up); }
  if (partImport.context !== 'world' && !wasEditing) setEditMode(false);
  if(partImport.context === 'world')running=partImport.wasRunning;
  if (partImport.camera) { cam.dist = partImport.camera.dist; cam.pan.copy(partImport.camera.pan);cam.target.copy(partImport.camera.target); }
  $('#addPart').focus();
}
async function partImportCommit() {
  const c = partImport.draft; if (!c || partImport.busy || fleet.selected !== partImport.owner) return;
  const generation = partImport.generation, mount = $('#partImportMount').value, rec = worldObjects.files.get(c.model.fileId);
  try { validatePartGeometry(c); } catch (e) { $('#partImportPick').textContent = e.message || String(e); return; }
  partImport.busy = true; $('#partImportAdd').disabled = true;
  const kept = await worldFilePut(rec);
  if (!partImport.active || generation !== partImport.generation) return;
  const original=partImport.editId&&compById(partImport.editId);
  if(original){
    const used=childrenOf(original).map(x=>x.parentPoint).filter(Boolean);if(used.some(id=>!c.points.some(p=>p.id===id))){partImport.busy=false;$('#partImportAdd').disabled=false;$('#partImportPick').textContent='Keep mounting points that are used by child parts.';return;}
    partScale(original,Math.max(...c.size));partOrientMass(original,massRot(c));
    for(const point of c.points)if(partPoints(original).some(p=>p.id===point.id))partMovePoint(original,point.id,point.pos.slice());
    const id=original.id,pos=original.pos.slice(),parent=original.parent,parentPoint=original.parentPoint;Object.assign(original,c,{id,pos,parent,parentPoint});partChooseMount(original,mount);c.id=id;
  }else{cfg.comps.push(c); partChooseMount(c, mount);}
  partImportEnd(); partImport.busy = false; openSet.clear(); openSet.add(c.id); structural(); selectComp(c.id);
  designNote((original ? 'Updated ' : 'Added ') + c.name + (kept ? '. Edit its weight, center of mass and mounting points in Parts.' : '. The browser could not keep all its files or library entry; export the design before closing this page.'));
}
function partImportPick(e) {
  if (!partImport.active) return false;
  if (partImport.pick && partImport.preview) {
    rayFrom(e); const hit = raycaster.intersectObject(partImport.preview, true).find(h => h.object.isMesh && !h.object.userData.noPick);
    if (hit) { const p = partImport.preview.worldToLocal(hit.point.clone()).toArray();
      if (partImport.pick === 'cog') partImport.draft.cog = p; else { const a = partImport.draft.points.find(p => p.id === partImport.pick); if (a) a.pos = p; }
      partImport.pick = null; partImportRenderVectors(); partDraftRefresh();
    }
  }
  return true;
}
function partImportView() {
  if (partImport.active && partImport.preview && !partImport.preview.parent) (partImport.context==='world'?scene:drone).add(partImport.preview);
}
// Keep the drone and preview in the uncovered region while the properties remain in the same 3D view.
function partImportFrame() {
  if (!partImport.active || !partImport.panel) return null;
  return viewOverlayFrame(partImport.panel);
}
$('#partModelImport').addEventListener('click', () => { $('#addPartDlg').close(); $('#partModelFile').click(); });
$('#partModelFile').addEventListener('change', e => { const files = [...e.target.files]; e.target.value = ''; if (files.length) objectImportStart(files,objectImportContext); });
window.addEventListener('keydown', e => { if (partImport.active && e.code === 'Escape' && !typingIn(e.target)) { e.preventDefault(); partImportCancel(); } });
