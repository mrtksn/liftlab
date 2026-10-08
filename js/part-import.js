'use strict';
const partModels = new Map();
const partImport = { draft: null, preview: null, solid: null, showSolid: true, panel: null, busy: false, active: false, wasEditing: false, owner: null, pick: null, generation: 0 };
const PART_LIBRARY_LS = 'liftlab-imported-parts-v1';
let partLibrary = [];
try {
  const saved = JSON.parse(localStorage.getItem(PART_LIBRARY_LS) || '[]');
  if (Array.isArray(saved)) partLibrary = saved.filter(c => { try { validatePartGeometry(c); return !!c.model; } catch { return false; } });
} catch (_) {}
function partLibraryWrite() {
  try { localStorage.setItem(PART_LIBRARY_LS, JSON.stringify(partLibrary)); return true; } catch (_) { return false; }
}
function partLibrarySave(c) {
  const template = JSON.parse(JSON.stringify(c));
  delete template.id; delete template.parent; delete template.parentPoint; template.pos = [0, 0, 0];
  partLibrary = partLibrary.filter(p => p.model.fileId !== c.model.fileId); partLibrary.push(template);
  const kept = partLibraryWrite(); partLibraryRender(); return kept;
}
function partLibraryRender() {
  document.querySelectorAll('#addPartDlg .part-library-entry').forEach(e => e.remove());
  const categories = ['propulsion', 'power', 'structure', 'payload', 'sensors'];
  for (const c of partLibrary) {
    const category = categories.indexOf(c.category || 'payload'), row = el('div', { class: 'part-library-entry' });
    const add = UI.button({ class: 'btn', title: c.mass.toFixed(3) + ' kg · imported object', onclick: e => {
      e.stopPropagation(); $('#addPartDlg').close(); openPlace($('#addPart'), c.name, place => {
        const copy = JSON.parse(JSON.stringify(c)); copy.id = uid++; copy.pos = [0, 0, 0];
        cfg.comps.push(copy);
        if (place) placeNew(copy, place); else attachTo(copy, null, null, copy.selfPoint || null);
        openSet.clear(); openSet.add(copy.id); structural(); selectComp(copy.id);
      });
    } }, c.name);
    row.append(add, UI.button({ class: 'btn icon', 'aria-label': 'Remove saved part ' + c.name, title: 'Remove from Add a part; existing copies stay on their drones', onclick: () => { partLibrary = partLibrary.filter(p => p !== c); partLibraryWrite(); partLibraryRender(); } }, '×'));
    $('#addPartDlg').querySelectorAll('section .addrow')[category < 0 ? 3 : category].append(row);
  }
}
partLibraryRender();
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
function partAssetsDecode(design) {
  const files = design.modelFiles; if (!files) return [];
  if (!Array.isArray(files) || files.length > design.comps.length) throw new Error('Invalid part model files');
  const ids = new Set(); let bytes = 0;
  return files.map(f => {
    if (!f || typeof f.id !== 'string' || ids.has(f.id) || !design.comps.some(c => c.model?.fileId === f.id) || !Array.isArray(f.files) || !f.files.length || f.files.length > 500) throw new Error('Invalid part model file group');
    ids.add(f.id);
    const names = new Set(), parts = f.files.map(p => {
      if (!p || typeof p.name !== 'string' || !p.name || p.name.length > 256 || names.has(p.name) || typeof p.data !== 'string' || p.data.length % 4 || !/^[A-Za-z0-9+/]*={0,2}$/.test(p.data) || (bytes += p.data.length * .75) > 128 * 1024 * 1024) throw new Error('Invalid part model data');
      names.add(p.name); const text = atob(p.data), data = new Uint8Array(text.length);
      for (let i = 0; i < text.length; i++) data[i] = text.charCodeAt(i);
      return { name: p.name, data: data.buffer };
    });
    if (!parts.some(p => WORLD_OBJ_FORMATS[extOf(p.name)])) throw new Error('Missing part model');
    return { id: f.id, name: String(f.name || parts[0].name), files: parts };
  });
}
function partAssetsRestore(design) {
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
  c.model.detail = levels[Math.max(0, level)]; c.model.boxes = result.boxes.map(b => [...b.lo, ...b.hi]);
  c.size = c.model.rawSize.map(v => Math.max(.0001, v * c.model.scale));
}
function partDraftRefresh() {
  const c = partImport.draft; if (!c) return;
  if (partImport.preview) { partImport.preview.parent?.remove(partImport.preview); disposeGroup(partImport.preview); }
  const g = partModelVisual(c); g.userData.noPick = true;
  c.pos = [cReach + c.size[0] / 2 + .1, 0, 0]; g.position.set(...c.pos); drone.add(g); partImport.preview = g;
  partImport.solid = solidBoxesVisual(partBoxes(c)); partImport.solid.visible = partImport.showSolid; g.add(partImport.solid);
  const mark = (pos, color) => { const m = new THREE.Mesh(new THREE.SphereGeometry(Math.max(.004, Math.max(...c.size) * .02), 12, 8), new THREE.MeshBasicMaterial({ color, depthTest: false })); m.userData.noPick = true; m.position.set(...pos); m.renderOrder = 30; g.add(m); };
  mark(partCog(c), colorOf('--grav')); for (const p of partPoints(c)) mark(p.pos, colorOf('--accent'));
  $('#partImportDims').textContent = c.size.map(v => v.toFixed(3)).join(' × ') + ' m · ' + c.model.boxes.length + ' solid boxes · ' + WORLD_OBJ_DETAIL[c.model.detail].label;
  $('#partImportMass').value = +c.mass.toFixed(4);
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
  const panel = partImport.panel = el('aside', { class: 'view-overlay part-import-panel', 'aria-label': 'Import drone object' });
  panel.append(el('div', { class: 'dialog-toolbar' }, el('h2', { text: 'Import 3D object' }), UI.button({ class: 'btn btn-sm', id: 'partImportCancel', onclick: partImportCancel }, 'Cancel')));
  if (!c) { panel.append(el('p', { class: 'hint', text: 'Reading and making the model solid…' })); $('.view').append(panel); return; }
  const name = UI.input({ type: 'text', id: 'partImportName', value: c.name, maxlength: 60 }); name.addEventListener('input', () => { c.name = name.value.trim() || 'Object'; });
  const category = UI.select({ id: 'partImportCategory' }, ...Object.entries(PART_CATEGORIES).map(([k, v]) => el('option', { value: k, text: v }))); category.value = c.category; category.addEventListener('change', () => { c.category = category.value; });
  const size = numField('partImportScale', { label: 'Scale (longest side)', u: 'm', min: .005, max: 2, step: .005, dp: 3, hard: true, hmin: .005, hmax: 10, ends: ['5 mm', '2 m'] }, () => Math.max(...c.size), n => {
    const span = 2 * cReach + Math.max(...c.size) + .1;
    partScale(c, n); if (partImport.massAuto) c.mass = clamp(partSolidMass(c).volume * 200, .005, 20);
    cam.dist *= (2 * cReach + Math.max(...c.size) + .1) / span;
    partDraftRefresh(); partImportRenderVectors();
  });
  size.node.querySelector('input[type=number]').id = 'partImportSize';
  const mass = UI.input({ type: 'number', id: 'partImportMass', min: .0001, step: .01, value: c.mass }); mass.addEventListener('change', () => { const n = Number(mass.value); if (n > 0 && Number.isFinite(n)) { c.mass = n; partImport.massAuto = false; } else mass.value = c.mass; });
  const mount = UI.select({ id: 'partImportMount' }, ...partMountOptions(c).map(([v, t]) => el('option', { value: v, text: t })));
  panel.append(el('p', { class: 'hint', text: 'Size the object beside the drone, then attach it. Blue boxes show its collision shape; orange marks its center of mass and blue dots its mounting points.' }),
    UI.field({ label: 'Name' }, name), UI.field({ label: 'Category' }, category), size.node, el('p', { class: 'hint world-obj-dims', id: 'partImportDims' }),
    solidShapeControls({ detail: c.model.detail, shown: partImport.showSolid, id: 'partImportSolid', onDetail: partImportDetail,
      onShown: shown => { partImport.showSolid = shown; if (partImport.solid) partImport.solid.visible = shown; } }), UI.field({ label: 'Weight (kg)' }, mass),
    el('p', { class: 'hint', text: 'Starting weight estimates 200 kg/m³ over the solid shape. Adjust it to match the object.' }),
    el('div', { id: 'partImportVectors' }), UI.field({ label: 'Attach to' }, mount),
    UI.button({ class: 'btn primary', id: 'partImportAdd', onclick: partImportCommit }, 'Attach object'));
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
async function partImportStart(files) {
  if (partImport.active || !fleet.selected || liveOn() || !fleetCanSelect()) return;
  partImport.active = partImport.busy = true; partImport.owner = fleet.selected;
  const generation = ++partImport.generation;
  partImport.wasEditing = editMode; partImport.massAuto = true; partImport.pick = null; partImport.showSolid = true;
  partImport.camera = { dist: cam.dist, pan: cam.pan.clone() };
  setEditMode(true); fleet.pendingEdits++; partImportPanel();
  $('.work').classList.add('part-importing');
  partImport.locked = [$('#airframe'), $('.bar'), $('#editBar'), $('#worldPanel')].filter(Boolean).map(e => [e, e.inert]);
  for (const [e] of partImport.locked) e.inert = true;
  try {
    const parts = await Promise.all([...files].map(async f => ({ name: f.name, data: await f.arrayBuffer() })));
    if (parts.reduce((s, p) => s + p.data.byteLength, 0) > 128 * 1024 * 1024) throw new Error('Model files exceed 128 MB');
    const main = parts.find(p => WORLD_OBJ_FORMATS[extOf(p.name)]); if (!main) throw new Error('Choose a GLB, glTF, OBJ or STL model');
    const rec = { id: newObjId('part-file-'), name: main.name, files: parts };
    const up = extOf(main.name) === 'stl' ? 'z' : 'y', { base, size } = standModel(await parseModel(rec), up); base.children[0].position.z -= size[2] / 2;
    if (!partImport.active || generation !== partImport.generation) { disposeGroup(base); return; }
    const c = mkMass(main.name.replace(/\.[^.]+$/, '').slice(0, 60) || 'Object', 0, 0, 0, { shape: 'model', battery: false, category: 'payload', rotation: [0, 0, 0], model: { fileId: rec.id, rawSize: size, scale: .2 / Math.max(...size), up, detail: 'medium', boxes: [] } });
    partSolidifyDraft(c, base); c.cog = partSolidMass(c).center; c.mass = clamp(partSolidMass(c).volume * 200, .005, 20);
    c.points = [{ id: 'top', name: 'Top mount', pos: [0, 0, c.size[2] / 2] }]; c.selfPoint = 'top';
    worldObjects.files.set(rec.id, rec); partModels.set(rec.id + ':' + up, Promise.resolve(base));
    partImport.draft = c; partImport.panel.remove(); partImportPanel(); partDraftRefresh();
    cam.dist = Math.max(.6, (2 * cReach + Math.max(...c.size) + .1) * 1.2); cam.pan.set(0, 0, 0); edit.focusLocal = null; edit.focusId = null;
  } catch (e) { if (generation === partImport.generation) { partImportCancel(); designNote('Could not import object: ' + (e.message || e)); } }
  finally { if (generation === partImport.generation) partImport.busy = false; }
}
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
  partImportEnd(); if (c) { worldObjects.files.delete(c.model.fileId); partModels.delete(c.model.fileId + ':' + c.model.up); }
  if (!wasEditing) setEditMode(false);
  if (partImport.camera) { cam.dist = partImport.camera.dist; cam.pan.copy(partImport.camera.pan); }
  $('#addPart').focus();
}
async function partImportCommit() {
  const c = partImport.draft; if (!c || partImport.busy || fleet.selected !== partImport.owner) return;
  const generation = partImport.generation, mount = $('#partImportMount').value, rec = worldObjects.files.get(c.model.fileId);
  try { validatePartGeometry(c); } catch (e) { $('#partImportPick').textContent = e.message || String(e); return; }
  partImport.busy = true; $('#partImportAdd').disabled = true;
  const kept = await worldFilePut(rec);
  if (!partImport.active || generation !== partImport.generation) return;
  cfg.comps.push(c); partChooseMount(c, mount);
  const savedPart = partLibrarySave(c);
  partImportEnd(); partImport.busy = false; openSet.clear(); openSet.add(c.id); structural(); selectComp(c.id);
  designNote('Added ' + c.name + (kept && savedPart ? '. Edit its weight, center of mass and mounting points in Parts.' : '. The browser could not keep all its files or library entry; export the design before closing this page.'));
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
  if (partImport.active && partImport.preview && !partImport.preview.parent) drone.add(partImport.preview);
}
// Keep the drone and preview in the uncovered region while the properties remain in the same 3D view.
function partImportFrame() {
  if (!partImport.active || !partImport.panel) return null;
  const view = vpEl.getBoundingClientRect(), panel = partImport.panel.getBoundingClientRect(), w = view.width, h = view.height;
  const bottom = panel.width > w * .7;
  const x = 12, y = Math.min(130, h * .2), width = Math.max(100, bottom ? w - 24 : panel.left - view.left - 24), height = Math.max(100, (bottom ? panel.top - view.top : h - 12) - y);
  return { w, h, dx: w / 2 - (x + width / 2), dy: h / 2 - (y + height / 2), scale: Math.max(1, h / Math.min(width, height)) };
}
$('#partModelImport').addEventListener('click', () => { $('#addPartDlg').close(); $('#partModelFile').click(); });
$('#partModelFile').addEventListener('change', e => { const files = [...e.target.files]; e.target.value = ''; if (files.length) partImportStart(files); });
window.addEventListener('keydown', e => { if (partImport.active && e.code === 'Escape' && !typingIn(e.target)) { e.preventDefault(); partImportCancel(); } });
