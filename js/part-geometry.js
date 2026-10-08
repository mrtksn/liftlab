'use strict';
// Geometry coordinates are metres in a part's own axes, centred on its drawing origin.
// CoM is independent of that origin. Each part has one structural parent and any number of mounting sites.
const PART_CATEGORIES = { payload: 'Payload', structure: 'Structure & motion', power: 'Power', propulsion: 'Propulsion', sensors: 'Sensors' };
const partCog = c => Array.isArray(c.cog) ? c.cog : [0, 0, 0];
function partRot(c) {
  if (c.type === 'mass') return massRot(c);
  if (c.type === 'link') return rodFrameOf(c);
  if (c.type === 'sensor') return eulerR(...(c.mount || [0, 0, 0]));
  if (c.type === 'motor') return frameFrom(mountDir(c), [1, 0, 0]);
  return [1, 0, 0, 0, 1, 0, 0, 0, 1];
}
function partMassRest(c) {
  const origin = c.type === 'link' ? add(c.pos, scl(linkDir(c), c.length / 2)) : c.pos || [0, 0, 0];
  return add(origin, m3v(partRot(c), partCog(c)));
}
function partPoints(c) {
  if (Array.isArray(c.points)) return c.points;
  if (c.type === 'link') return [{ id: 'base', name: 'Base', pos: [0, 0, 0] }, { id: 'tip', name: 'Tip', pos: [c.length, 0, 0] }];
  if (c.type === 'joint') return [{ id: 'output', name: 'Output', pos: [0, 0, 0] }];
  if (c.type === 'latch') return [{ id: 'hook', name: 'Hook', pos: LATCH_HOOK.slice() }];
  return [];
}
function partPointRest(c, id) {
  const q = partPoints(c).find(p => p.id === id);
  return add(c.pos || [0, 0, 0], m3v(partRot(c), q?.pos || [0, 0, 0]));
}
function partMountOrigin(c) {
  const p = parentOf(c);
  if (c.parentPoint) return partPointRest(p || cfg.frame, c.parentPoint);
  return p?.type === 'link' ? linkTip(p) : p?.pos || [0, 0, 0];
}
const partBoxCache = new WeakMap();
function partBoxes(c) {
  let list = partBoxCache.get(c.model.boxes);
  if (!list) { list = c.model.boxes.map(b => ({ lo: b.slice(0, 3), hi: b.slice(3, 6) })); partBoxCache.set(c.model.boxes, list); }
  return list;
}
const partBoxCenter = b => b.lo.map((v, k) => (v + b.hi[k]) / 2);
const partBoxSize = b => b.lo.map((v, k) => b.hi[k] - v);
function partSolidMass(c) {
  const boxes = partBoxes(c), vol = b => partBoxSize(b).reduce((a, v) => a * Math.max(v, 1e-9), 1);
  const volume = boxes.reduce((s, b) => s + vol(b), 0), center = [0, 0, 0];
  for (const b of boxes) { const f = vol(b) / volume, p = partBoxCenter(b); for (let k = 0; k < 3; k++) center[k] += f * p[k]; }
  return { volume, center };
}
const partInertiaCache = new WeakMap();
function partSolidInertia(c) {
  const old = partInertiaCache.get(c), key = c.mass + ':' + partCog(c).join() + ':' + (c.rotation || []).join() + ':' + (c.inc || 0);
  if (old?.boxes === c.model.boxes && old.key === key) return old.I;
  const { volume } = partSolidMass(c), J = Array(9).fill(0), cm = partCog(c);
  for (const b of partBoxes(c)) {
    const size = partBoxSize(b), m = c.mass * size.reduce((a, v) => a * Math.max(v, 1e-9), 1) / volume;
    const I = boxI(m, ...size), d = sub(partBoxCenter(b), cm), dd = dot(d, d);
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) J[i * 3 + j] += I[i * 3 + j] + m * ((i === j ? dd : 0) - d[i] * d[j]);
  }
  const R = massRot(c), I = m3m(m3m(R, J), m3T(R));
  partInertiaCache.set(c, { boxes: c.model.boxes, key, I }); return I;
}
function partSolidContacts(c) {
  const R = massRot(c), points = [], seen = new Set();
  for (const b of partBoxes(c)) for (const p of boxPoints(add(c.pos, m3v(R, partBoxCenter(b))), partBoxSize(b), R)) {
    const key = p.rest.map(v => v.toFixed(5)).join() + ':' + p.r;
    if (!seen.has(key)) { seen.add(key); points.push(p); }
  }
  return points;
}
function partScale(c, longest) {
  const ratio = longest / Math.max(...c.size);
  if (!(Number.isFinite(ratio) && ratio > 0)) return;
  const children = typeof cfg !== 'undefined' ? childrenOf(c).map(x => [x, partMountOrigin(x)]) : [];
  const anchor = c.selfPoint && cfg.comps.includes(c) ? partPointRest(c, c.selfPoint) : null;
  c.size = c.size.map(v => v * ratio); c.model.scale *= ratio;
  c.model.boxes = c.model.boxes.map(b => b.map(v => v * ratio));
  c.cog = partCog(c).map(v => v * ratio);
  if (c.points) for (const p of c.points) p.pos = p.pos.map(v => v * ratio);
  if (anchor) c.pos = add(c.pos, sub(anchor, partPointRest(c, c.selfPoint)));
  for (const [x, before] of children) { const d = sub(partMountOrigin(x), before); x.pos = add(x.pos, d); shiftSubtree(x, d); }
}
function partOrientMass(c, next) {
  const before = massRot(c), own = partPoints(c).find(p => p.id === c.selfPoint), pivot = own ? partPointRest(c, own.id) : c.pos.slice();
  setMassRot(c, next);
  if (own) c.pos = sub(pivot, m3v(next, own.pos));
  rotateSubtree(c, m3m(next, m3T(before)), pivot); snapHolder(c);
}
function validatePartGeometry(c) {
  const vec = a => Array.isArray(a) && a.length === 3 && a.every(v => Number.isFinite(v) && Math.abs(v) <= 5000);
  if (c.cog != null && !vec(c.cog)) throw new Error('Invalid center of mass');
  if (c.rotation != null && !vec(c.rotation)) throw new Error('Invalid object rotation');
  if (c.points != null) {
    if (!Array.isArray(c.points) || c.points.length > 1024) throw new Error('Invalid attachment points');
    const ids = new Set();
    for (const p of c.points) { if (!p || typeof p.id !== 'string' || !p.id || p.id.length > 80 || p.id.includes('|') || ids.has(p.id) || typeof p.name !== 'string' || p.name.length > 60 || !vec(p.pos)) throw new Error('Invalid attachment point'); ids.add(p.id); }
  }
  if (c.model != null) {
    const m = c.model;
    const raw = Array.isArray(m.rawSize) && m.rawSize.length === 3 && m.rawSize.every(v => Number.isFinite(v) && v >= 0) && Math.max(...m.rawSize) > 0;
    if (c.type !== 'mass' || c.shape !== 'model' || typeof m.fileId !== 'string' || !m.fileId || !Number.isFinite(c.mass) || c.mass <= 0 || !vec(c.size) || c.size.some(v => v <= 0) || !Number.isFinite(m.scale) || m.scale <= 0 || !raw || !['y', 'z'].includes(m.up) || !['coarse', 'medium', 'fine'].includes(m.detail) || !Array.isArray(m.boxes) || !m.boxes.length || m.boxes.length > 6000) throw new Error('Invalid imported part');
    for (const b of m.boxes) if (!Array.isArray(b) || b.length !== 6 || !b.every(Number.isFinite) || b.some(v => Math.abs(v) > 5000) || [0, 1, 2].some(k => b[k] >= b[k + 3])) throw new Error('Invalid imported solid shape');
  }
}

// Portable design validation is independent of the browser import UI and model readers.
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
    if (!parts.some(p => ['glb', 'gltf', 'obj', 'stl'].includes(p.name.split('.').pop().toLowerCase()))) throw new Error('Missing part model');
    return { id: f.id, name: String(f.name || parts[0].name), files: parts };
  });
}
