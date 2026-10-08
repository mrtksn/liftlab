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
  if (c.model.collision === "mesh") return partMeshContacts(c);
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
  if (c.model.triangles) c.model.triangles = c.model.triangles.map(v => v * ratio);
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
    validateCollisionGeometry(m.collision, m.triangles);
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

// Triangle surfaces are used directly for collision; boxes remain the volume/inertia estimate.
const MESH_MAX_TRIANGLES = 100000;
function validateCollisionGeometry(mode, triangles) {
  if (mode != null && !['boxes', 'mesh'].includes(mode)) throw new Error('Invalid collision geometry');
  if (triangles != null && (!Array.isArray(triangles) || !triangles.length || triangles.length % 9 || triangles.length > MESH_MAX_TRIANGLES * 9 || !triangles.every(v => Number.isFinite(v) && Math.abs(v) <= 1e6))) throw new Error('Invalid collision mesh');
  if (mode === 'mesh' && !triangles?.length) throw new Error('Collision mesh is missing');
}
const meshCache = new WeakMap();
function collisionMesh(triangles) {
  if (meshCache.has(triangles)) return meshCache.get(triangles);
  const tris = [];
  for (let i = 0; i < triangles.length; i += 9) {
    const v = [triangles.slice(i, i + 3), triangles.slice(i + 3, i + 6), triangles.slice(i + 6, i + 9)];
    if (nrm(crs(sub(v[1], v[0]), sub(v[2], v[0]))) < 1e-12) continue;
    tris.push({ v, lo: [0,1,2].map(k => Math.min(...v.map(p => p[k]))), hi: [0,1,2].map(k => Math.max(...v.map(p => p[k]))) });
  }
  const build = list => {
    const lo = [0,1,2].map(k => Math.min(...list.map(t => t.lo[k]))), hi = [0,1,2].map(k => Math.max(...list.map(t => t.hi[k])));
    if (list.length <= 12) return {lo,hi,tris:list};
    const span = sub(hi,lo), axis = span.indexOf(Math.max(...span)); list.sort((a,b) => a.lo[axis]+a.hi[axis]-b.lo[axis]-b.hi[axis]);
    const mid = list.length >> 1; return {lo,hi,left:build(list.slice(0,mid)),right:build(list.slice(mid))};
  };
  const edges=new Map();for(const t of tris){const keys=t.v.map(p=>p.map(v=>v.toPrecision(9)).join());for(let k=0;k<3;k++){const a=keys[k],b=keys[(k+1)%3],key=a<b?a+'|'+b:b+'|'+a;edges.set(key,(edges.get(key)||0)+1);}}
  const mesh = tris.length ? build(tris) : {lo:[0,0,0],hi:[0,0,0],tris:[]}; mesh.closed=tris.length>=4&&[...edges.values()].every(count=>count===2);meshCache.set(triangles,mesh); return mesh;
}
function meshCandidates(mesh, p, r, visit) {
  const distance = b => Math.hypot(...p.map((v,k) => Math.max(b.lo[k]-v,0,v-b.hi[k])));
  const walk = b => {if (distance(b)>r) return; if (b.tris) b.tris.forEach(visit); else {walk(b.left);walk(b.right);} }; walk(mesh);
}
function triangleClosest(p, v) {
  const [a,b,c]=v, ab=sub(b,a), ac=sub(c,a), ap=sub(p,a), d1=dot(ab,ap), d2=dot(ac,ap);
  if(d1<=0&&d2<=0)return a;
  const bp=sub(p,b),d3=dot(ab,bp),d4=dot(ac,bp);if(d3>=0&&d4<=d3)return b;
  const vc=d1*d4-d3*d2;if(vc<=0&&d1>=0&&d3<=0)return add(a,scl(ab,d1/(d1-d3)));
  const cp=sub(p,c),d5=dot(ab,cp),d6=dot(ac,cp);if(d6>=0&&d5<=d6)return c;
  const vb=d5*d2-d1*d6;if(vb<=0&&d2>=0&&d6<=0)return add(a,scl(ac,d2/(d2-d6)));
  const va=d3*d6-d5*d4;if(va<=0&&d4-d3>=0&&d5-d6>=0)return add(b,scl(sub(c,b),(d4-d3)/(d4-d3+d5-d6)));
  const den=1/(va+vb+vc);return add(a,add(scl(ab,vb*den),scl(ac,vc*den)));
}
function triangleRay(o,d,v) {
  const e1=sub(v[1],v[0]),e2=sub(v[2],v[0]),h=crs(d,e2),det=dot(e1,h);if(Math.abs(det)<1e-12)return Infinity;
  const s=sub(o,v[0]),u=dot(s,h)/det;if(u < -1e-9 || u > 1+1e-9)return Infinity;
  const q=crs(s,e1),w=dot(d,q)/det;if(w < -1e-9 || u+w > 1+1e-9)return Infinity;
  const t=dot(e2,q)/det;return t>=-1e-9?Math.max(0,t):Infinity;
}
function meshRayHits(mesh,o,d,max=Infinity) {
  const hits=[];
  const walk=b=>{
    let lo=0,hi=max;
    for(let k=0;k<3;k++){if(Math.abs(d[k])<1e-12){if(o[k]<b.lo[k]||o[k]>b.hi[k])return;}else{let a=(b.lo[k]-o[k])/d[k],z=(b.hi[k]-o[k])/d[k];if(a>z)[a,z]=[z,a];lo=Math.max(lo,a);hi=Math.min(hi,z);if(lo>hi)return;}}
    if(b.tris)for(const t of b.tris){const distance=triangleRay(o,d,t.v);if(Number.isFinite(distance)&&distance<=max)hits.push({distance,v:t.v});}else{walk(b.left);walk(b.right);}
  };walk(mesh);return hits.sort((a,b)=>a.distance-b.distance);
}
function meshInside(mesh,p) {
  if(!mesh.closed||p.some((v,k)=>v<=mesh.lo[k]||v>=mesh.hi[k]))return false;
  const hits=meshRayHits(mesh,p,unit([1,.37139,.21713]));let count=0,last=-Infinity;
  for(const h of hits)if(h.distance>1e-8&&h.distance-last>1e-7){count++;last=h.distance;}
  return !!(count%2);
}
function meshContact(p,r,mesh,prev) {
  const inside=meshInside(mesh,p);let best=null,distance=inside?Infinity:r;
  const nearest=b=>{
    const gap=Math.hypot(...p.map((v,k)=>Math.max(b.lo[k]-v,0,v-b.hi[k])));if(gap>distance)return;
    if(b.tris)for(const t of b.tris){const q=triangleClosest(p,t.v),d=nrm(sub(p,q));if(d<=distance){distance=d;best={q,v:t.v};}}
    else {const mid=b.left.lo.map((v,k)=>(v+b.left.hi[k])/2),first=nrm(sub(p,mid))<nrm(sub(p,b.right.lo));nearest(first?b.left:b.right);nearest(first?b.right:b.left);}
  };nearest(mesh);
  // Previous-position crossing handles thin/open surfaces and prevents escaping through the far face.
  if(prev){const travel=sub(p,prev),length=nrm(travel);if(length>1e-9){const hit=meshRayHits(mesh,prev,scl(travel,1/length),length)[0];if(hit&&hit.distance>1e-8){let n=unit(crs(sub(hit.v[1],hit.v[0]),sub(hit.v[2],hit.v[0])));if(dot(n,travel)>0)n=scl(n,-1);return {depth:Math.max(0,r-dot(sub(p,add(prev,scl(travel,hit.distance/length))),n)),n};}}}
  if(!best || (!inside && distance>=r))return null;
  let n=distance>1e-9?unit(sub(inside?best.q:p,inside?p:best.q)):unit(crs(sub(best.v[1],best.v[0]),sub(best.v[2],best.v[0])));
  if(prev&&dot(n,sub(prev,best.q))<0)n=scl(n,-1);
  return {depth:inside?r+distance:r-distance,n};
}
function partMeshContacts(c) {
  const out=[],seen=new Set(),R=massRot(c),tri=c.model.triangles,big=Math.max(...c.size),small=big<=BOX_SMALL,spacing=small?big/4:Math.max(BOX_STEP,big/16);
  const put=(p,key)=>{if(!seen.has(key)){seen.add(key);out.push({rest:add(c.pos,m3v(R,p)),r:0});}};
  // Match the simulator's existing surface-contact sampling, using the mesh rather than voxel faces.
  for(let i=0;i<tri.length;i+=9){const v=[tri.slice(i,i+3),tri.slice(i+3,i+6),tri.slice(i+6,i+9)];
    const steps=small?1:Math.max(1,Math.min(16,Math.ceil(Math.max(nrm(sub(v[1],v[0])),nrm(sub(v[2],v[0])),nrm(sub(v[2],v[1])))/spacing)));
    for(let a=0;a<=steps;a++)for(let b=0;b<=steps-a;b++){const p=add(v[0],add(scl(sub(v[1],v[0]),a/steps),scl(sub(v[2],v[0]),b/steps))),key=p.map(x=>Math.round(x/spacing)).join();put(p,key);}
  }
  // Keep extreme surface points even when a thin part's two faces share a sampling cell.
  for(let axis=0;axis<3;axis++)for(const sign of [-1,1]){let index=0;for(let i=3;i<tri.length;i+=3)if(sign*tri[i+axis]>sign*tri[index+axis])index=i;const p=tri.slice(index,index+3),rest=add(c.pos,m3v(R,p));if(!out.some(q=>nrm(sub(q.rest,rest))<1e-7))put(p,'extreme-'+axis+'-'+sign);}
  return out;
}
