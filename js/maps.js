'use strict';
// Named maps own a complete snapshot and copies of their model files in IndexedDB.
// Portable JSON embeds those files as base64; active fleet/drone data is never replaced.
const MAP_FORMAT = 'liftlab-map', MAP_MAX_BYTES = 128 * 1024 * 1024;
const maps = { list: [], current: null, ready: false, busy: false, db: null };
const mapSnapshot = () => ({ map: { kind: terrain.kind, seed: terrain.seed }, objects: worldObjectsSnapshot(), seeds: { ...worldSeeds }, environment: { ...envr } });
function mapSay(t) { $('#mapSay').textContent = t; }
function mapDbOpen() {
  if (!maps.db) maps.db = new Promise((ok, no) => {
    const r = indexedDB.open('liftlab-maps', 1);
    r.onupgradeneeded = () => r.result.createObjectStore('maps', { keyPath: 'id' });
    r.onsuccess = () => ok(r.result); r.onerror = () => no(r.error);
  }).catch(e => { maps.db = null; throw e; });
  return maps.db;
}
async function mapDbDo(mode, fn) {
  const db = await mapDbOpen();
  return new Promise((ok, no) => {
    const t = db.transaction('maps', mode), r = fn(t.objectStore('maps'));
    t.oncomplete = () => ok(r.result); t.onerror = () => no(t.error); t.onabort = () => no(t.error);
  });
}
async function mapsInit() {
  try { maps.list = await mapDbDo('readonly', s => s.getAll()); }
  catch (e) { mapSay('Saved maps are unavailable in this browser. You can still export and import map files.'); }
  maps.ready = true;
  if (!maps.list.some(m => m.id === maps.current)) maps.current = null;
  mapUiSync();
}
function mapUiSync() {
  const sel = $('#terrainSel'), current = maps.list.find(m => m.id === maps.current);
  const signature = JSON.stringify(maps.list.map(m => [m.id, m.name]));
  if (sel.dataset.maps !== signature) {
    sel.replaceChildren(el('optgroup', { label: 'Built-in maps' }, ...Object.entries(TERRAINS).map(([k, m]) => el('option', { value: k, text: m.label }))));
    if (maps.list.length) sel.append(el('optgroup', { label: 'Saved maps' }, ...maps.list.map(m => el('option', { value: m.id, text: m.name }))));
    sel.dataset.maps = signature;
  }
  sel.value = current ? current.id : terrain.kind;
  const busy = maps.busy || wedit.busy;
  sel.disabled = busy || !maps.ready;
  $('#terrainNew').disabled = busy || terrain.kind === 'open';
  for (const id of ['mapSave', 'mapExport', 'mapImport', 'mapDelete']) $('#' + id).disabled = busy || !maps.ready || (id === 'mapDelete' && !current);
  if (current && !$('#mapName').value) $('#mapName').value = current.name;
}
async function mapTask(fn) {
  if (maps.busy || wedit.busy || edit.drag || !maps.ready) { mapUiSync(); return false; }
  if (liveOn() || usbViewOn()) { mapSay('Stop the real-board view before changing maps.'); mapUiSync(); return false; }
  maps.busy = true; mapUiSync(); worldEditRender();
  try { await fn(); return true; }
  catch (e) { mapSay('Could not complete the map operation: ' + (e.message || e)); return false; }
  finally { maps.busy = false; mapUiSync(); worldEditRender(); }
}
async function mapCollect(name) {
  const world = mapSnapshot(), files = [];
  for (const id of new Set(world.objects.map(o => o.fileId))) {
    const rec = worldObjects.files.get(id) || await worldFileGet(id);
    if (rec) files.push(rec);
  }
  return { name, world, files };
}
const mapMissing = m => new Set(m.world.objects.filter(o => !m.files.some(f => f.id === o.fileId)).map(o => o.fileId)).size;
const mapModelNote = m => mapMissing(m) ? ' Some original model files are missing; those objects keep their solid shapes but cannot be turned or resized.' : '';
function mapName() { return $('#mapName').value.trim().slice(0, 60) || 'Map ' + (maps.list.length + 1); }
async function mapSave() {
  return mapTask(async () => {
    const name = mapName(), current = maps.list.find(m => m.id === maps.current && m.name === name);
    const m = { ...await mapCollect(name), id: current?.id || newObjId('map-'), at: Date.now() };
    await mapDbDo('readwrite', s => s.put(m));
    maps.list = [m, ...maps.list.filter(x => x.id !== m.id)]; maps.current = m.id;
    $('#mapName').value = name; fleetSave(); mapSay(`Saved “${name}” in this browser.` + mapModelNote(m));
  });
}
async function mapApply(m, id) {
  if (!fleetCanSelect()) throw new Error('Finish the current drone operation before switching maps');
  // Copies in each map outlive deletion of objects from the active world.
  let kept = true;
  for (const f of m.files) { worldObjects.files.set(f.id, f); if (!await worldFilePut(f)) kept = false; }
  worldEditSelect(null); wedit.hover = null;
  worldObjectsRestore(m.world.objects);
  if(typeof objectLibraryAdoptPlaced === "function")await objectLibraryAdoptPlaced();
  maps.current = id;
  replayWorldChanged();
  applyWorld({ ...m.world, terrain: m.world.map });
  userWorldReset(); fleetSave();
  if (wedit.on) wedit.changed = false;
  $('#mapName').value = m.name || ''; worldEditRender(); worldEditMsg();
  mapSay(`Loaded “${m.name || TERRAINS[m.world.map.kind].label}”: every flight started again.` + mapModelNote(m) + (kept ? '' : ' Model files could not be kept for reload; save or export the map.'));
}
async function mapSelect(value) {
  return mapTask(async () => {
    if (TERRAINS[value]) {
      await mapApply({ name: '', world: { ...mapSnapshot(), map: { kind: value, seed: terrain.seed }, objects: [] }, files: [] }, null);
    } else {
      const m = maps.list.find(m => m.id === value);
      if (!m) throw new Error('This saved map is unavailable');
      await mapApply(m, m.id);
    }
  });
}
function mapEncode(buffer) {
  const bytes = new Uint8Array(buffer); let s = '';
  for (let i = 0; i < bytes.length; i += 32768) s += String.fromCharCode(...bytes.subarray(i, i + 32768));
  return btoa(s);
}
async function mapExport() {
  return mapTask(async () => {
    const m = await mapCollect(mapName());
    const json = JSON.stringify({ format: MAP_FORMAT, version: 1, ...m, files: m.files.map(f => ({ id: f.id, name: f.name, files: f.files.map(a => ({ name: a.name, data: mapEncode(a.data) })) })) });
    const blob = new Blob([json], { type: 'application/json' });
    if (blob.size > MAP_MAX_BYTES) throw new Error('The map file exceeds the 128 MB import limit');
    const url = URL.createObjectURL(blob), a = document.createElement('a');
    a.href = url; a.download = (m.name.replace(/[^a-z0-9_-]+/gi, '-').replace(/^-|-$/g, '') || 'map') + '.liftlab-map.json';
    a.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
    mapSay(`Exported “${m.name}”.` + mapModelNote(m));
  });
}
// Validate the entire document before storing or changing the active world.
function mapDecode(doc) {
  const bad = t => { throw new Error('Invalid map file: ' + t); };
  if (!doc || doc.format !== MAP_FORMAT || doc.version !== 1) bad('expected a LiftLab map, version 1');
  const w = doc.world;
  const seed = v => Number.isInteger(v) && v >= 1 && v <= 0xFFFFFFFF;
  const vec = (a, limit, nonnegative = false) => Array.isArray(a) && a.length === 3 && a.every(v => Number.isFinite(v) && Math.abs(v) <= limit && (!nonnegative || v >= 0));
  if (!w || !Object.hasOwn(TERRAINS, w.map?.kind) || !seed(w.map.seed)) bad('map layout or seed');
  if (!w.seeds || !SEED_KEYS.every(k => seed(w.seeds[k]))) bad('random seeds');
  // Slider endpoints are hints: typed wind, heading, spread and temperature can exceed them.
  const ranges = { wind: [0, Infinity], windDir: [-Infinity, Infinity], turb: [0, 1], spread: [0, Infinity], texture: [0, 1], light: [0, 1], ambient: [-Infinity, Infinity], pressure: [20000, 120000], rotorSamples: [1, 5] }, environment = {};
  for (const k of Object.keys(DEFAULT_ENVIRONMENT)) {
    const v = w.environment?.[k];
    if (k === 'sensorEffects' ? typeof v !== 'boolean' : !Number.isFinite(v) || v < ranges[k][0] || v > ranges[k][1]) bad('environment: ' + k);
    if (k === 'rotorSamples' && ![1, 5].includes(v)) bad('rotor sampling');
    environment[k] = v;
  }
  if (!Array.isArray(w.objects) || w.objects.length > 1000) bad('object list');
  const ids = new Set(); let boxes = 0;
  const objects = w.objects.map(o => {
    if (!o || typeof o.id !== 'string' || !o.id || ids.has(o.id) || typeof o.fileId !== 'string' || !vec(o.pos, 5000) || !vec(o.size, 1e7, true) || !Number.isFinite(o.yaw) || Math.abs(o.yaw) > 360 || !Number.isFinite(o.scale) || o.scale <= 0 || o.scale > 1e5 || !Number.isFinite(o.h) || o.h < 0 || !Object.hasOwn(WORLD_OBJ_DETAIL, o.detail) || !['y', 'z'].includes(o.up)) bad('object transform');
    validateCollisionGeometry(o.collision,o.triangles);worldMotionValidate(o.animation);
    ids.add(o.id);
    if (!Array.isArray(o.boxes) || !o.boxes.length || o.boxes.length % 6 || o.boxes.length > 36000 || (boxes += o.boxes.length / 6) > 600000 || !o.boxes.every(x => Number.isFinite(x) && Math.abs(x) <= 1e6)) bad('solid shapes');
    for (let i = 0; i < o.boxes.length; i += 6) for (let k = 0; k < 3; k++) if (o.boxes[i + k] > o.boxes[i + k + 3]) bad('solid shape bounds');
    return { id: o.id, fileId: o.fileId, name: String(o.name || 'Object').slice(0, 60), pos: o.pos.slice(), size: o.size.slice(), yaw: o.yaw, scale: o.scale, h: o.h, up: o.up, units: Object.hasOwn(WORLD_OBJ_UNITS, o.units) ? o.units : 'custom', detail: o.detail, collision:o.collision || "boxes", ...(o.collision === "mesh" ? {triangles:o.triangles.slice()} : {}), boxes: o.boxes.slice(), ...(o.animation?{animation:worldMotionValidate(o.animation)}:{}) };
  });
  if (!Array.isArray(doc.files) || doc.files.length > objects.length) bad('model files');
  const fileIds = new Set(); let bytes = 0;
  const files = doc.files.map(f => {
    if (!f || typeof f.id !== 'string' || fileIds.has(f.id) || !objects.some(o => o.fileId === f.id) || !Array.isArray(f.files) || !f.files.length || f.files.length > 500) bad('model file group');
    fileIds.add(f.id); const names = new Set();
    const parts = f.files.map(p => {
      if (!p || typeof p.name !== 'string' || !p.name || p.name.length > 256 || names.has(p.name) || typeof p.data !== 'string' || p.data.length % 4 || !/^[A-Za-z0-9+/]*={0,2}$/.test(p.data)) bad('embedded model data');
      names.add(p.name); bytes += p.data.length * 3 / 4; if (bytes > MAP_MAX_BYTES) bad('model files exceed 128 MB');
      const s = atob(p.data), data = new Uint8Array(s.length); for (let i = 0; i < s.length; i++) data[i] = s.charCodeAt(i);
      return { name: p.name, data: data.buffer };
    });
    if (!parts.some(p => Object.hasOwn(WORLD_OBJ_FORMATS, extOf(p.name)))) bad('missing 3D model');
    // Imported identifiers cannot overwrite model files belonging to another local map.
    const id = newObjId('f'); for (const o of objects) if (o.fileId === f.id) o.fileId = id;
    return { id, name: String(f.name || parts[0].name).slice(0, 256), files: parts };
  });
  return { name: String(doc.name || 'Imported map').trim().slice(0, 60) || 'Imported map', world: { map: { kind: w.map.kind, seed: w.map.seed }, seeds: Object.fromEntries(SEED_KEYS.map(k => [k, w.seeds[k]])), environment, objects }, files };
}
async function mapImport(file) {
  return mapTask(async () => {
    if (file.size > MAP_MAX_BYTES) throw new Error('The map file exceeds 128 MB');
    const m = { ...mapDecode(JSON.parse(await file.text())), id: newObjId('map-'), at: Date.now() };
    // Import still works when browser storage is unavailable; report that saving failed.
    let saved = true;
    try { await mapDbDo('readwrite', s => s.put(m)); maps.list.unshift(m); }
    catch (e) { saved = false; }
    await mapApply(m, saved ? m.id : null);
    mapSay(`Imported “${m.name}”. ` + (saved ? 'Saved in this browser and available in the World selector.' : 'This browser could not save the map; keep the imported file.') + mapModelNote(m));
  });
}
$('#mapSave').addEventListener('click', mapSave);
$('#mapName').addEventListener('keydown', e => { if (e.key === 'Enter') { e.preventDefault(); $('#mapSave').click(); } });
$('#mapExport').addEventListener('click', mapExport);
$('#mapImport').addEventListener('click', () => $('#mapFile').click());
$('#mapFile').addEventListener('change', e => { const file = e.target.files[0]; if (file) mapImport(file); e.target.value = ''; });
$('#mapDelete').addEventListener('click', () => mapTask(async () => {
  const m = maps.list.find(m => m.id === maps.current); if (!m || !confirm(`Delete saved map “${m.name}”? The current world stays here.`)) return;
  await mapDbDo('readwrite', s => s.delete(m.id)); maps.list = maps.list.filter(x => x.id !== m.id); maps.current = null;
  fleetSave(); mapSay(`Deleted saved map “${m.name}”.`);
}));
