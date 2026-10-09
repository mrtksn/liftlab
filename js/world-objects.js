'use strict';
// Objects in the world: 3D models imported from a file (glTF/GLB, OBJ or STL) and placed where you like
// (world-edit.js moves them). Each is drawn as it is and made solid the simple way: the model is cut into
// voxels, a grid of small cubes (its surface, and inside it where the surface is closed), and runs of voxels
// are merged into as few boxes as they make. The boxes join the city's in terrain.js, so drones, props,
// cables, sensors and radios meet an object as they meet a building.
//
// An object: { id, name, fileId, pos (where the middle of its base sits), yaw (degrees about Z), scale (file
// units to metres), units, up ('y' or 'z': which way is up in the file), detail, boxes (relative to pos, turned
// and scaled as placed) }. The file itself is kept in this browser's IndexedDB; the boxes are saved with the
// world, so an object is solid at once on the next visit, and still solid (drawn as its boxes) if its file is gone.

const WORLD_OBJ_FORMATS = { glb: 'GLTFLoader', gltf: 'GLTFLoader', obj: 'OBJLoader', stl: 'STLLoader' };
const WORLD_OBJ_DETAIL = { coarse: { label: 'Coarse', cells: 16 }, medium: { label: 'Medium', cells: 32 }, fine: { label: 'Fine', cells: 64 } };
const WORLD_OBJ_UNITS = { m: { label: 'Metres', k: 1 }, cm: { label: 'Centimetres', k: 0.01 }, mm: { label: 'Millimetres', k: 0.001 }, in: { label: 'Inches', k: 0.0254 } };
const WORLD_OBJ_MAX_BOXES = 6000;   // more than this at the detail asked for: the next coarser detail
const worldObjects = { list: [], ver: 0, files: new Map() };   // files: fileId → { id, name, files: [{ name, data }] }, read this session
const worldObjG = new THREE.Group(); scene.add(worldObjG);
let worldObjMats = null;

/* ───────── reading a model file ───────── */
const extOf = name => (String(name).toLowerCase().match(/\.([a-z0-9]+)$/) || [])[1] || '';
const loaderScripts = new Map();
function needModelLoader(name) {   // the three.js readers load only when a model is first read
  if (THREE[name]) return Promise.resolve();
  if (!loaderScripts.has(name)) loaderScripts.set(name, new Promise((ok, no) => {
    const s = document.createElement('script'); s.src = `js/vendor/three-${name}-r128.js`;
    s.onload = ok; s.onerror = () => { loaderScripts.delete(name); no(new Error('The model reader could not be loaded')); };
    document.head.append(s);
  }));
  return loaderScripts.get(name);
}
// The model in a set of files: the first glb/gltf/obj/stl among them, with the others (a .gltf's buffers and
// textures) found by name. Resolves to a three.js object.
async function parseModel(rec) {
  const main = rec.files.find(f => WORLD_OBJ_FORMATS[extOf(f.name)]);
  if (!main) throw new Error('Pick a .glb, .gltf, .obj or .stl file');
  const ext = extOf(main.name); await needModelLoader(WORLD_OBJ_FORMATS[ext]);
  // STL's facet normals are often left zero, and OBJ may have none: shade them from their faces.
  if (ext === 'stl') { const geo = new THREE.STLLoader().parse(main.data); geo.computeVertexNormals(); return new THREE.Mesh(geo, worldObjMats.plain); }
  if (ext === 'obj') {
    const g = new THREE.OBJLoader().parse(new TextDecoder().decode(main.data));
    g.traverse(o => { if (o.isMesh) { o.material = worldObjMats.plain; if (!o.geometry.attributes.normal) o.geometry.computeVertexNormals(); } });   // (its .mtl isn't read)
    return g;
  }
  const urls = [], byName = new Map(rec.files.filter(f => f !== main).map(f => [f.name.split('/').pop(), f]));
  const manager = new THREE.LoadingManager(), missing = new Set();
  manager.setURLModifier(u => {
    if (/^(data|blob):/.test(u)) return u;
    const f = byName.get(decodeURIComponent(u.split(/[?#]/)[0]).split('/').pop());
    if (!f) { missing.add(u); return u; }
    const url = URL.createObjectURL(new Blob([f.data])); urls.push(url); return url;
  });
  try {
    const gltf = await new Promise((ok, no) => new THREE.GLTFLoader(manager).parse(ext === 'gltf' ? new TextDecoder().decode(main.data) : main.data, '', ok,
      e => no(missing.size ? new Error(`This .gltf needs ${[...missing].join(', ')}: pick ${missing.size > 1 ? 'them' : 'it'} together with the .gltf, or export a .glb`) : new Error(e && e.message ? e.message : 'The glTF file could not be read'))));
    return gltf.scene;
  } finally { setTimeout(() => urls.forEach(u => URL.revokeObjectURL(u)), 5000); }   // (textures may still be decoding)
}
// The model stood up (its up axis along Z) with the middle of its base at the origin: `base`, and its size in file units.
function standModel(model, up) {
  const turn = new THREE.Group(); turn.add(model); if (up === 'y') turn.rotation.x = Math.PI / 2;
  const base = new THREE.Group(); base.add(turn); base.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(base);
  if (box.isEmpty()) throw new Error('The file has no surfaces to make solid');
  turn.position.set(-(box.min.x + box.max.x) / 2, -(box.min.y + box.max.y) / 2, -box.min.z);
  return { base, size: box.getSize(new THREE.Vector3()).toArray() };
}
function guessUnits(ext, size) {   // CAD (and so STL) is mostly millimetres; anything kilometres big likely was too
  const big = Math.max(...size);
  return (ext === 'stl' && big > 20) || big > 400 ? 'mm' : 'm';
}

/* ───────── making it solid ───────── */
// Triangles (9 numbers each) → boxes covering every voxel the surface passes through and every voxel it closes in.
// cells: voxels along the longest side. Returns { boxes: [{ lo, hi }], h: the voxel size }.
function voxelBoxes(tri, cells) {
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < tri.length; i += 3) for (let k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], tri[i + k]); hi[k] = Math.max(hi[k], tri[i + k]); }
  if (!(hi[0] >= lo[0])) return { boxes: [], h: 0 };
  const h = Math.max(Math.max(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]) / cells, 0.005);
  // Along each axis a whole number of voxels spans the model, so its solid shape ends where it does (a voxel is
  // stretched a little to fit); a flat side (a wall with no thickness) is one voxel thick. A free voxel all
  // round lets the outside flow through.
  const hs = [], o = [], m = [];
  for (let k = 0; k < 3; k++) {
    const size = hi[k] - lo[k];
    if (size < h) { m[k] = 1; hs[k] = h; o[k] = (lo[k] + hi[k]) / 2 - 1.5 * h; }
    else { m[k] = Math.max(1, Math.round(size / h)); hs[k] = size / m[k]; o[k] = lo[k] - hs[k]; }
  }
  const n = m.map(x => x + 2), N0 = n[0], N01 = n[0] * n[1], solid = new Uint8Array(N01 * n[2]);
  const at = (x, y, z) => x + N0 * y + N01 * z;
  const cell = (v, k) => Math.min(m[k], Math.max(1, Math.floor((v - o[k]) / hs[k])));
  const mark = (x, y, z) => { solid[at(cell(x, 0), cell(y, 1), cell(z, 2))] = 1; };
  // The surface: points over each triangle no more than a third of a voxel apart.
  for (let i = 0; i < tri.length; i += 9) {
    const ax = tri[i], ay = tri[i + 1], az = tri[i + 2], ux = tri[i + 3] - ax, uy = tri[i + 4] - ay, uz = tri[i + 5] - az, vx = tri[i + 6] - ax, vy = tri[i + 7] - ay, vz = tri[i + 8] - az;
    const L = Math.sqrt(Math.max(ux * ux + uy * uy + uz * uz, vx * vx + vy * vy + vz * vz, (ux - vx) ** 2 + (uy - vy) ** 2 + (uz - vz) ** 2));
    const q = Math.min(4000, Math.max(1, Math.ceil(L / (Math.min(...hs) / 3))));
    for (let a = 0; a <= q; a++) for (let b = 0; a + b <= q; b++) { const s = a / q, t = b / q; mark(ax + ux * s + vx * t, ay + uy * s + vy * t, az + uz * s + vz * t); }
  }
  // The inside: whatever the outside can't reach, flowing face to face from a corner. (An open surface lets it
  // in, and stays a shell.)
  const out = new Uint8Array(solid.length), stack = new Int32Array(solid.length); let sp = 0;
  out[0] = 1; stack[sp++] = 0;
  while (sp) {
    const c = stack[--sp], x = c % N0, y = Math.floor(c / N0) % n[1], z = Math.floor(c / N01);
    const go = (ok, j) => { if (ok && !out[j] && !solid[j]) { out[j] = 1; stack[sp++] = j; } };
    go(x > 0, c - 1); go(x < N0 - 1, c + 1); go(y > 0, c - N0); go(y < n[1] - 1, c + N0); go(z > 0, c - N01); go(z < n[2] - 1, c + N01);
  }
  for (let j = 0; j < solid.length; j++) if (!out[j]) solid[j] = 1;
  // Runs of voxels into boxes: along X, then rows of those along Y, then slabs of those along Z.
  const used = new Uint8Array(solid.length), free = j => solid[j] && !used[j], boxes = [];
  for (let z = 0; z < n[2]; z++) for (let y = 0; y < n[1]; y++) for (let x = 0; x < N0; x++) {
    if (!free(at(x, y, z))) continue;
    let dx = 1, dy = 1, dz = 1;
    while (x + dx < N0 && free(at(x + dx, y, z))) dx++;
    const row = (yy, zz) => { for (let i = 0; i < dx; i++) if (!free(at(x + i, yy, zz))) return false; return true; };
    while (y + dy < n[1] && row(y + dy, z)) dy++;
    const slab = zz => { for (let j = 0; j < dy; j++) if (!row(y + j, zz)) return false; return true; };
    while (z + dz < n[2] && slab(z + dz)) dz++;
    for (let k = 0; k < dz; k++) for (let j = 0; j < dy; j++) for (let i = 0; i < dx; i++) used[at(x + i, y + j, z + k)] = 1;
    boxes.push({ lo: [o[0] + x * hs[0], o[1] + y * hs[1], o[2] + z * hs[2]], hi: [o[0] + (x + dx) * hs[0], o[1] + (y + dy) * hs[1], o[2] + (z + dz) * hs[2]] });
  }
  return { boxes, h };
}
// The model's triangles as it's placed (turned and scaled), around its base point.
function placedTriangles(o) {
  const out = [], v = new THREE.Vector3(), P = o.g.position;
  o.g.updateMatrixWorld(true);
  o.base.traverse(m => {
    if (!m.isMesh || !m.geometry || !m.geometry.attributes.position) return;
    const A = m.geometry.attributes.position, I = m.geometry.index, n = (I ? I.count : A.count) - (I ? I.count : A.count) % 3;
    for (let k = 0; k < n; k++) { v.fromBufferAttribute(A, I ? I.getX(k) : k).applyMatrix4(m.matrixWorld); out.push(v.x - P.x, v.y - P.y, v.z - P.z); }
  });
  return Float32Array.from(out);
}
// Make the object solid again (after it was turned, resized or its detail changed). Needs its model.
function worldObjSolidify(o) {
  if (!o.base) return;
  placeGroup(o);
  const tri = placedTriangles(o), keys = Object.keys(WORLD_OBJ_DETAIL);
  let k = Math.max(0, keys.indexOf(o.detail)), r;
  for (; ; k--) { r = voxelBoxes(tri, WORLD_OBJ_DETAIL[keys[k]].cells); if (r.boxes.length <= WORLD_OBJ_MAX_BOXES || k === 0) break; }
  o.coarsened = keys[k] !== o.detail ? keys[k] : null;
  o.boxes = r.boxes; o.h = r.h;
  o.triangles = Array.from(tri);
  validateCollisionGeometry(o.collision, o.triangles);
  worldObjectsChanged();
}

/* ───────── the objects ───────── */
const newObjId = p => p + Date.now().toString(36) + Math.random().toString(36).slice(2, 6);
const worldObj = id => worldObjects.list.find(o => o.id === id) || null;
function worldObjBounds(o) {   // in the world, from its boxes
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  if(o.animation&&typeof worldMotionShapes==='function'){for(const b of worldMotionShapes(o))for(let k=0;k<3;k++){lo[k]=Math.min(lo[k],b.lo[k]);hi[k]=Math.max(hi[k],b.hi[k]);}return {lo,hi};}
  for (const b of o.boxes) for (let k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], o.pos[k] + b.lo[k]); hi[k] = Math.max(hi[k], o.pos[k] + b.hi[k]); }
  return lo[0] <= hi[0] ? { lo, hi } : { lo: o.pos.slice(), hi: o.pos.slice() };
}
// The objects' boxes into the terrain, in the world. Called whenever an object moves, turns or comes or goes.
function worldObjectsChanged(structural = true) {
  const out = [];
  for (const o of worldObjects.list) {
    const what = o.name;
    if(o.animation&&structural){worldMotionApply(o,worldMotionState(o).t);}
    if(o.animation)worldMotionDraw(o);
    if(o.animation&&typeof worldMotionShapes==='function')out.push(...worldMotionShapes(o));
    else if (o.collision === 'mesh' && o.triangles?.length) {const mesh=collisionMesh(o.triangles);out.push({lo:add(o.pos,mesh.lo),hi:add(o.pos,mesh.hi),mesh,origin:o.pos.slice(),what,obj:o.id});}
    else for (const b of o.boxes) out.push({ lo: add(o.pos, b.lo), hi: add(o.pos, b.hi), what, obj: o.id });
    o.wb = null;   // (the fade's cached world boxes)
  }
  terrainSetObjectBoxes(out); worldObjects.ver++;
  if (structural && typeof replayWorldChanged === 'function') replayWorldChanged();   // a recording or a replay was of the world before
}
function placeGroup(o) { o.g.position.set(...o.pos); o.g.rotation.set(0, 0, o.yaw * D2R); o.g.scale.setScalar(o.scale); o.g.updateMatrixWorld(true); }
function objGroup(o) {
  const g = new THREE.Group(); g.userData.worldObjId = o.id; worldObjG.add(g); o.g = g; placeGroup(o); return g;
}
function disposeObj(o) {
  if(typeof worldSoundStopObject==='function')worldSoundStopObject(o.id);
  if(typeof worldMotionEditor!=='undefined'&&worldMotionEditor.id===o.id)worldMotionClose();
  if (!o.g) return;
  o.g.traverse(m => {
    if (m.geometry) m.geometry.dispose();
    for (const mat of [].concat(m.userData.mat0 || m.material || [])) if (mat && !Object.values(worldObjMats).includes(mat) && mat.dispose) { for (const v of Object.values(mat)) if (v && v.isTexture) v.dispose(); mat.dispose(); }
  });
  worldObjG.remove(o.g); o.g = null; o.base = null;
}
// Its model under its group, or its boxes when the model isn't here (the file was kept in another browser).
function showModel(o, base) {
  for (const c of [...o.g.children]) { o.g.remove(c); c.traverse(m => m.geometry && m.geometry.dispose()); }
  o.base = base; o.missing = !base;
  if (base) { base.traverse(m => { if (m.isMesh) { m.userData.mat0 = m.material; m.castShadow = false; } }); o.g.add(base); return; }
  const boxes = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), worldObjMats.plain, Math.max(1, o.boxes.length));
  boxes.count = o.boxes.length; boxes.userData.mat0 = worldObjMats.plain;
  const M = new THREE.Matrix4(), q = new THREE.Quaternion(), inv = 1 / o.scale, rz = new THREE.Matrix4().makeRotationZ(-o.yaw * D2R);
  o.boxes.forEach((b, i) => {   // (the group turns and scales; the boxes are already turned and scaled)
    const c = new THREE.Vector3((b.lo[0] + b.hi[0]) / 2, (b.lo[1] + b.hi[1]) / 2, (b.lo[2] + b.hi[2]) / 2).applyMatrix4(rz).multiplyScalar(inv);
    q.setFromRotationMatrix(rz); M.compose(c, q, new THREE.Vector3(b.hi[0] - b.lo[0], b.hi[1] - b.lo[1], b.hi[2] - b.lo[2]).multiplyScalar(inv)); boxes.setMatrixAt(i, M);
  });
  o.g.add(boxes);
}
async function readModel(o) {
  const rec = worldObjects.files.get(o.fileId) || await worldFileGet(o.fileId);
  if (!rec) return null;
  worldObjects.files.set(rec.id, rec);
  return standModel(await parseModel(rec), o.up).base;
}
function worldObjDuplicate(src) {
  const o = { ...src, id: newObjId('o'), name: (src.name + ' copy').slice(0, 60), pos: add(src.pos, [Math.max(0.5, (src.size[0] * src.scale) * 1.2), 0, 0]), boxes: src.boxes.map(b => ({ lo: b.lo.slice(), hi: b.hi.slice() })), g: null, base: null, wb: null, animation:src.animation?worldMotionValidate(src.animation):undefined, motionState:null, motionPose:null, motionPrevious:null };
  objGroup(o); worldObjects.list.push(o);
  if (src.base) showModel(o, src.base.clone(true)); else showModel(o, null);
  worldObjectsChanged();
  return o;
}
function worldObjRemove(o) {
  worldObjects.list = worldObjects.list.filter(x => x !== o); disposeObj(o); worldObjectsChanged();
  if (typeof objectLibrary !== 'undefined') return; // Library and portable copies own their source files independently.
  if (!worldObjects.list.some(x => x.fileId === o.fileId)) { worldObjects.files.delete(o.fileId); worldFileDelete(o.fileId); }
}
// Turn the model the other way up (it was drawn with Y up, or Z): it's stood up again and made solid again.
function worldObjSetUp(o, up) {
  if (up === o.up || !o.base) return;
  const model = o.base.children[0].children[0];
  o.base.children[0].remove(model); o.up = up;
  const { base, size } = standModel(model, up); o.size = size;
  o.g.remove(o.base); o.g.add(base); o.base = base; base.traverse(m => { if (m.isMesh && !m.userData.mat0) m.userData.mat0 = m.material; });
  worldObjSolidify(o);
}

/* ───────── the view ───────── */
function worldObjectsTheme() {
  const c = colorOf('--bldg');
  if (!worldObjMats) worldObjMats = {
    plain: new THREE.MeshStandardMaterial({ color: c, roughness: 0.85, metalness: 0.05 }),
    fade: new THREE.MeshStandardMaterial({ color: c, roughness: 0.95, transparent: true, opacity: 0.14, depthWrite: false }),
    solidWire: new THREE.LineBasicMaterial({color:colorOf('--accent'),transparent:true,opacity:.7,depthWrite:false}),
    solid: new THREE.MeshBasicMaterial({ color: colorOf('--accent'), transparent: true, opacity: 0.22, depthWrite: false, side: THREE.DoubleSide }),
  };
  worldObjMats.plain.color.copy(c); worldObjMats.fade.color.copy(c); worldObjMats.solid.color.copy(colorOf('--accent'));worldObjMats.solidWire.color.copy(colorOf('--accent'));
}
// Objects between the camera and what it looks at turn see-through, as buildings do (view3d.js updateCity).
function worldObjectsFade(c, rays) {
  for (const o of worldObjects.list) {
    if (!o.g) continue;
    if (!o.wb) { o.wb = worldObjBounds(o); o.wb.boxes = o.animation ? worldMotionShapes(o) : o.boxes.map(b => ({ lo: add(o.pos, b.lo), hi: add(o.pos, b.hi) })); }
    const fade = !(typeof worldEditOn === 'function' && worldEditOn()) && rays.some(r => segHitsBox(c, r.d, r.L, o.wb) && o.wb.boxes.some(b => segHitsBox(c, r.d, r.L, b)));
    if (fade === !!o.faded) continue;
    o.faded = fade;
    o.g.traverse(m => { if (m.isMesh && m.userData.mat0) m.material = fade ? worldObjMats.fade : m.userData.mat0; });
  }
}
// The selected object's solid shape, drawn over it (world-edit.js).
let solidVis = null;
// Shared collision display for world objects and the drone-part import preview. Boxes are local metres.
function solidBoxesVisual(boxes) {
  const mesh = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), worldObjMats.solid, boxes.length);
  const M = new THREE.Matrix4(), q = new THREE.Quaternion();
  boxes.forEach((b, i) => { M.compose(new THREE.Vector3((b.lo[0] + b.hi[0]) / 2, (b.lo[1] + b.hi[1]) / 2, (b.lo[2] + b.hi[2]) / 2), q, new THREE.Vector3(b.hi[0] - b.lo[0], b.hi[1] - b.lo[1], b.hi[2] - b.lo[2]).multiplyScalar(1.002)); mesh.setMatrixAt(i, M); });
  mesh.renderOrder = 18; mesh.userData.noPick = true;
  return mesh;
}
function collisionVisual(mode,triangles,boxes) {
  if(mode !== 'mesh' || !triangles?.length)return solidBoxesVisual(boxes);
  const geo=new THREE.BufferGeometry();geo.setAttribute('position',new THREE.Float32BufferAttribute(triangles,3));geo.computeVertexNormals();
  const mesh=new THREE.Mesh(geo,worldObjMats.solid);mesh.renderOrder=18;mesh.userData.noPick=true;const wire=new THREE.LineSegments(new THREE.WireframeGeometry(geo),worldObjMats.solidWire);wire.renderOrder=19;wire.userData.noPick=true;mesh.add(wire);return mesh;
}
function worldObjShowSolid(o) {
  if (solidVis) { scene.remove(solidVis); disposeGroup(solidVis); solidVis = null; }
  if (!o || !o.boxes.length) return;
  solidVis = collisionVisual(o.collision, o.triangles, o.boxes);
  solidVis.position.set(...o.pos);   // (moving the object moves this along)
  scene.add(solidVis);
  if(o.animation)worldMotionDraw(o);
}

/* ───────── saving ───────── */
function worldObjectsSnapshot() {
  return worldObjects.list.map(o => ({ id: o.id, name: o.name, fileId: o.fileId, pos: o.pos.map(x => +x.toFixed(4)), yaw: o.yaw, scale: o.scale, units: o.units, up: o.up, detail: o.detail, size: o.size.map(x => +x.toFixed(5)), h: +(o.h || 0).toFixed(5), collision:o.collision || "boxes", ...(o.collision === "mesh" ? {triangles:o.triangles} : {}),
    ...(o.animation?{animation:worldMotionValidate(o.animation)}:{}), boxes: o.boxes.flatMap(b => [...b.lo, ...b.hi]).map(x => +x.toFixed(4)) }));
}
// Objects from a saved world: solid at once, drawn when their files have been read.
function worldObjectsRestore(list) {
  for (const o of worldObjects.list) disposeObj(o);
  worldObjects.list = [];
  for (const s of Array.isArray(list) ? list : []) {
    if (!s || typeof s.id !== 'string' || !Array.isArray(s.pos) || s.pos.length !== 3 || !s.pos.every(Number.isFinite) || !Array.isArray(s.boxes) || s.boxes.length % 6) continue;
    try { validateCollisionGeometry(s.collision,s.triangles);worldMotionValidate(s.animation); } catch (_) { continue; }
    const boxes = []; for (let i = 0; i < s.boxes.length; i += 6) boxes.push({ lo: s.boxes.slice(i, i + 3).map(Number), hi: s.boxes.slice(i + 3, i + 6).map(Number) });
    const o = { id: s.id, name: String(s.name || 'Object').slice(0, 60), fileId: String(s.fileId || ''), pos: s.pos.map(Number), yaw: +s.yaw || 0, scale: +s.scale > 0 ? +s.scale : 1,
      units: WORLD_OBJ_UNITS[s.units] ? s.units : 'custom', up: s.up === 'z' ? 'z' : 'y', detail: WORLD_OBJ_DETAIL[s.detail] ? s.detail : 'medium', size: Array.isArray(s.size) && s.size.length === 3 ? s.size.map(Number) : [1, 1, 1], h: +s.h || 0, boxes, collision:s.collision || "boxes", triangles:s.triangles?.slice(), animation:worldMotionValidate(s.animation) };
    objGroup(o); showModel(o, null); o.missing = false; o.loading = true; worldObjects.list.push(o);
    readModel(o).then(base => { if (worldObj(o.id) !== o) return; o.loading = false; if (base) showModel(o, base); else o.missing = true;if(o.animation)worldMotionDraw(o); o.faded = false; if (typeof worldEditRender === 'function') worldEditRender(); })
      .catch(() => { o.loading = false; o.missing = true; if (typeof worldEditRender === 'function') worldEditRender(); });
  }
  worldObjectsChanged();
}

/* ───────── the files, in IndexedDB ───────── */
const WORLD_DB = 'liftlab-world-objects';
let worldDb = null;
function worldDbOpen() {
  if (!worldDb) worldDb = new Promise((ok, no) => {
    try {
      let blocked=false;const r=indexedDB.open(WORLD_DB,3);
      r.onupgradeneeded=()=>{for(const name of ['files','library','audio'])if(!r.result.objectStoreNames.contains(name))r.result.createObjectStore(name,{keyPath:'id'});};
      r.onblocked=()=>{blocked=true;no(new Error('Close other simulator tabs and reload to update object storage'));};
      r.onsuccess=()=>{if(blocked){r.result.close();return;}r.result.onversionchange=()=>{r.result.close();worldDb=null;};ok(r.result);};r.onerror=()=>no(r.error);
    }
    catch (e) { no(e); }
  }).catch(e => { worldDb = null; throw e; });
  return worldDb;
}
async function worldDbDo(mode, fn, store = 'files') {
  const db = await worldDbOpen();
  return new Promise((ok, no) => { const t = db.transaction(store, mode), r = fn(t.objectStore(store)); t.oncomplete = () => ok(r && r.result); t.onerror = () => no(t.error); t.onabort = () => no(t.error); });
}
const worldFilePut = rec => worldDbDo('readwrite', s => s.put(rec)).then(() => true, () => false);
const worldFileGet = id => worldDbDo('readonly', s => s.get(id)).catch(() => null);
const worldFileDelete = id => worldDbDo('readwrite', s => s.delete(id)).catch(() => {});

worldObjectsTheme();
