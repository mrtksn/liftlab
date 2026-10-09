'use strict';
// The world around the drone: open ground, or a city of simple blocks to fly through.
//
// The city is generated from a seed, so the same seed always gives the same city. It's laid out on a grid of
// blocks around an open plaza at the start point. The ring of blocks next to the plaza has low obstacles
// (gates, platforms on stilts, tunnels, steps, walls); further out stand towers, stepped and L-shaped
// buildings, some joined by bridges. The parkour city is about 1:8, with streets 1.3 to 2.5 m wide and
// towers up to 10 m. The full-scale city is the same layout at real size.
//
// Everything is a box aligned with the axes. The physics asks: which boxes are near a point, how deep a
// point is inside the ground or a box (and which way is out), what a ray hits first, and how high the
// surface under a point is.
//
// Objects imported into the world (world-objects.js) are made solid as boxes too, from their voxels. Each of
// their boxes carries `obj` (the object's id). terrain.boxes is the city's and the objects' together; the
// queries go through an index that keeps each object's boxes behind its bounds, so a detailed object far away
// costs one box test.

const TERRAINS = {
  open: { label: 'Open field', scale: 0 },
  parkour: { label: 'Parkour city', scale: 1 },
  city: { label: 'Full-scale city', scale: 8 },
};
const terrain = { kind: 'parkour', seed: 1, scale: 1, boxes: [], city: [], objBoxes: [], extent: 0, ver: 0 };

function mulberry32(a) {
  return () => { a |= 0; a = a + 0x6D2B79F5 | 0; let t = Math.imul(a ^ a >>> 15, 1 | a); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; };
}

function setTerrain(kind, seed) {
  if (!TERRAINS[kind]) kind = 'parkour';
  terrain.kind = kind; terrain.seed = (seed >>> 0) || 1; terrain.scale = TERRAINS[kind].scale;
  terrain.city = kind === 'open' ? [] : genCity(terrain.seed, terrain.scale);
  terrain.ver++;
  terrainCompose();
}
// The objects' boxes changed (or the city did): put them together again.
function terrainSetObjectBoxes(boxes) { terrain.objBoxes = boxes; terrainCompose(); }
function terrainCompose() {
  terrain.boxes = terrain.objBoxes.length ? terrain.city.concat(terrain.objBoxes) : terrain.city;
  terrain.extent = terrain.boxes.reduce((m, b) => Math.max(m, Math.abs(b.lo[0]), Math.abs(b.hi[0]), Math.abs(b.lo[1]), Math.abs(b.hi[1])), 0);
}

// The city, in parkour units (metres at 1:1), scaled by S.
function genCity(seed, S) {
  const R = mulberry32(seed), rr = (a, b) => a + (b - a) * R(), pick = a => a[Math.floor(R() * a.length)];
  const out = [];
  const box = (x0, y0, z0, x1, y1, z1, what) => out.push({
    lo: [Math.min(x0, x1) * S, Math.min(y0, y1) * S, Math.min(z0, z1) * S],
    hi: [Math.max(x0, x1) * S, Math.max(y0, y1) * S, Math.max(z0, z1) * S], what });
  const weighted = list => { let t = R() * list.reduce((s, x) => s + x[1], 0); for (const [k, w] of list) { t -= w; if (t <= 0) return k; } return list[0][0]; };
  const P = 5.5, N = 4;                         // block pitch; blocks from −N to N each way, the middle one left open
  const towers = new Map();
  for (let i = -N; i <= N; i++) for (let j = -N; j <= N; j++) {
    const ring = Math.max(Math.abs(i), Math.abs(j)); if (!ring) continue;
    const cx = i * P, cy = j * P;
    const w = rr(3, 4.2), d = rr(3, 4.2);
    const x0 = cx - w / 2 + rr(-1, 1) * (P - 1.3 - w) / 2, y0 = cy - d / 2 + rr(-1, 1) * (P - 1.3 - d) / 2, x1 = x0 + w, y1 = y0 + d;   // streets stay at least 1.3 m wide
    const type = ring === 1 ? pick(['gates', 'platform', 'walls', 'tunnel', 'stairs', 'gates', 'park'])
      : weighted([['tower', 30], ['stepped', 14], ['L', 10], ['platform', 9], ['gates', 8], ['tunnel', 7], ['walls', 8], ['stairs', 6], ['park', 8]]);
    const alongX = R() < 0.5;
    if (type === 'tower') {
      const h = 2 + rr(1, 3 + 1.4 * ring);
      box(x0, y0, 0, x1, y1, h, 'a building'); towers.set(i + ',' + j, { x0, x1, y0, y1, h });
    } else if (type === 'stepped') {
      const h1 = rr(1.5, 3); box(x0, y0, 0, x1, y1, h1, 'a building');
      const k = rr(0.5, 0.7), mx = (x0 + x1) / 2 + rr(-0.4, 0.4), my = (y0 + y1) / 2 + rr(-0.4, 0.4), h2 = h1 + rr(1.2, 3);
      box(mx - k * w / 2, my - k * d / 2, h1, mx + k * w / 2, my + k * d / 2, h2, 'a building');
      if (R() < 0.5) box(mx - k * w / 4, my - k * d / 4, h2, mx + k * w / 4, my + k * d / 4, h2 + rr(1, 2.5), 'a building');
      towers.set(i + ',' + j, { x0, x1, y0, y1, h: h1 });
    } else if (type === 'L') {
      const h1 = rr(2, 4 + ring), h2 = rr(1.5, 3.5);
      if (alongX) { box(x0, y0, 0, x1, y0 + d * 0.45, h1, 'a building'); box(x0, y0, 0, x0 + w * 0.45, y1, h2, 'a building'); }
      else { box(x0, y1 - d * 0.45, 0, x1, y1, h1, 'a building'); box(x1 - w * 0.45, y0, 0, x1, y1, h2, 'a building'); }
    } else if (type === 'platform') {   // a slab on four stilts: fly under it or land on it
      const pw = rr(2, 3.4), pd = rr(2, 3.4), h = rr(1.1, 2.2), px = (x0 + x1) / 2 - pw / 2, py = (y0 + y1) / 2 - pd / 2, s = 0.15;
      for (const [a, b] of [[0, 0], [1, 0], [0, 1], [1, 1]]) box(px + a * (pw - s), py + b * (pd - s), 0, px + a * (pw - s) + s, py + b * (pd - s) + s, h, 'a pillar');
      box(px, py, h, px + pw, py + pd, h + 0.12, 'a platform');
      if (R() < 0.35) {   // a second deck
        const h2 = h + rr(1, 1.6), q = 0.6;
        for (const [a, b] of [[0, 0], [1, 0], [0, 1], [1, 1]]) box(px + a * (pw * q - s), py + b * (pd * q - s), h + 0.12, px + a * (pw * q - s) + s, py + b * (pd * q - s) + s, h2, 'a pillar');
        box(px, py, h2, px + pw * q, py + pd * q, h2 + 0.12, 'a platform');
      }
    } else if (type === 'gates') {   // two or three gates in a row to fly through
      const n = R() < 0.5 ? 2 : 3, s = 0.25;
      for (let k = 0; k < n; k++) {
        const t = (k + 0.5) / n, gap = rr(1.2, 2), h = rr(1.8, 3), off = rr(-0.5, 0.5);
        if (alongX) { const x = x0 + t * w, y = cy + off; box(x - s / 2, y - gap / 2 - s, 0, x + s / 2, y - gap / 2, h, 'a gate'); box(x - s / 2, y + gap / 2, 0, x + s / 2, y + gap / 2 + s, h, 'a gate'); box(x - s / 2, y - gap / 2 - s, h, x + s / 2, y + gap / 2 + s, h + s, 'a gate'); }
        else { const y = y0 + t * d, x = cx + off; box(x - gap / 2 - s, y - s / 2, 0, x - gap / 2, y + s / 2, h, 'a gate'); box(x + gap / 2, y - s / 2, 0, x + gap / 2 + s, y + s / 2, h, 'a gate'); box(x - gap / 2 - s, y - s / 2, h, x + gap / 2 + s, y + s / 2, h + s, 'a gate'); }
      }
    } else if (type === 'tunnel') {
      const L = rr(2.5, 4), iw = rr(1.1, 1.6), ih = rr(1.1, 1.6), t = 0.2;
      if (alongX) { const xa = cx - L / 2, xb = cx + L / 2; box(xa, cy - iw / 2 - t, 0, xb, cy - iw / 2, ih, 'a tunnel wall'); box(xa, cy + iw / 2, 0, xb, cy + iw / 2 + t, ih, 'a tunnel wall'); box(xa, cy - iw / 2 - t, ih, xb, cy + iw / 2 + t, ih + t, 'a tunnel roof'); }
      else { const ya = cy - L / 2, yb = cy + L / 2; box(cx - iw / 2 - t, ya, 0, cx - iw / 2, yb, ih, 'a tunnel wall'); box(cx + iw / 2, ya, 0, cx + iw / 2 + t, yb, ih, 'a tunnel wall'); box(cx - iw / 2 - t, ya, ih, cx + iw / 2 + t, yb, ih + t, 'a tunnel roof'); }
    } else if (type === 'walls') {   // low walls to hop over
      const n = 2 + Math.floor(R() * 2);
      for (let k = 0; k < n; k++) {
        const L = rr(1.5, 3.5), h = rr(0.5, 1.3), t = 0.2, x = cx + rr(-1.5, 1.5), y = cy + rr(-1.5, 1.5);
        if (R() < 0.5) box(x - L / 2, y - t / 2, 0, x + L / 2, y + t / 2, h, 'a wall'); else box(x - t / 2, y - L / 2, 0, x + t / 2, y + L / 2, h, 'a wall');
      }
    } else if (type === 'stairs') {   // blocks stepping up
      const n = 4, sh = rr(0.45, 0.7);
      for (let k = 0; k < n; k++) {
        if (alongX) box(x0 + k * w / n, y0, 0, x0 + (k + 1) * w / n, y0 + Math.min(d, 1.6), sh * (k + 1), 'a step');
        else box(x0, y0 + k * d / n, 0, x0 + Math.min(w, 1.6), y0 + (k + 1) * d / n, sh * (k + 1), 'a step');
      }
    }
  }
  // Bridges between neighbouring buildings, across the street.
  for (const [key, a] of towers) {
    const [i, j] = key.split(',').map(Number);
    const e = towers.get((i + 1) + ',' + j), n = towers.get(i + ',' + (j + 1));
    if (e && R() < 0.45) {
      const ya = Math.max(a.y0, e.y0), yb = Math.min(a.y1, e.y1), top = Math.min(a.h, e.h) - 0.5;
      if (yb - ya > 0.9 && top > 1.4) { const y = rr(ya + 0.4, yb - 0.4), z = rr(1.2, top); box(a.x1, y - 0.35, z, e.x0, y + 0.35, z + 0.15, 'a bridge'); }
    }
    if (n && R() < 0.45) {
      const xa = Math.max(a.x0, n.x0), xb = Math.min(a.x1, n.x1), top = Math.min(a.h, n.h) - 0.5;
      if (xb - xa > 0.9 && top > 1.4) { const x = rr(xa + 0.4, xb - 0.4), z = rr(1.2, top); box(x - 0.35, a.y1, z, x + 0.35, n.y0, z + 0.15, 'a bridge'); }
    }
  }
  return out;
}

/* ───────── queries ───────── */
// The index: a city box stands for itself; an object's boxes sit behind one entry with their bounds and `boxes`.
// It's rebuilt when terrain.boxes is replaced (a test may set it directly).
const tIndex = { src: null, items: [] };
function terrainIndex() {
  if (tIndex.src === terrain.boxes) return tIndex.items;
  const items = [], groups = new Map();
  for (const b of terrain.boxes) {
    if (b.obj == null || b.mesh || b.body?.mesh) { items.push(b); continue; }
    let g = groups.get(b.obj);
    if (!g) { g = { lo: b.lo.slice(), hi: b.hi.slice(), boxes: [], obj:b.obj }; groups.set(b.obj, g); items.push(g); }
    for (let i = 0; i < 3; i++) { g.lo[i] = Math.min(g.lo[i], b.lo[i]); g.hi[i] = Math.max(g.hi[i], b.hi[i]); }
    g.boxes.push(b);
  }
  tIndex.src = terrain.boxes; tIndex.items = items; return items;
}
const nearBox = (p, r, b) => p[0] > b.lo[0] - r && p[0] < b.hi[0] + r && p[1] > b.lo[1] - r && p[1] < b.hi[1] + r && p[2] > b.lo[2] - r && p[2] < b.hi[2] + r;
// Boxes that come within r of p.
function terrainNear(p, r) {
  const out = [];
  for (const b of terrainIndex()) if (nearBox(p, r, b)) { if (b.boxes) { for (const x of b.boxes) if (nearBox(p, r, x)) out.push(x); } else out.push(b); }
  return out;
}
// How many walls the straight line a→b passes through (sampled at n points): a building or an object counts once.
function terrainWalls(a, b, n) {
  const hit = new Set(), d = sub(b, a), items = terrainIndex(),length=nrm(d);
  if(length>1e-9)items.forEach((x,i)=>{if((x.boxes||[x]).some(b=>(b.mesh||b.body)&&terrainSegmentHit(b,a,scl(d,1/length),length)))hit.add(i);});
  for (let s = 1; s < n; s++) {
    const q = add(a, scl(d, s / n));
    items.forEach((x, i) => { if (nearBox(q, 0, x) && (!x.boxes ? insideBox(q,x) : x.boxes.some(y => insideBox(q,y)))) hit.add(i); });
  }
  return hit.size;
}
const insideBox = (p, b) => b.body ? insideBox(m3v(b.pose.RT,sub(p,b.pose.origin)),b.body) : b.mesh ? meshInside(b.mesh,sub(p,b.origin)) : p[0] > b.lo[0] && p[0] < b.hi[0] && p[1] > b.lo[1] && p[1] < b.hi[1] && p[2] > b.lo[2] && p[2] < b.hi[2];
// A sphere of radius r at p against box b: how deep it is and which way is out. Inside the box, the way out
// is back through the face it came in by (from `prev`, where it was a moment ago), so a fast part pushed
// deep into a thin slab still comes out the side it hit, never through the far side.
function boxContact(p, r, b, prev) {
  if(b.body){
    const local=m3v(b.pose.RT,sub(p,b.pose.origin)),old=b.previousPose||b.pose;
    const c=boxContact(local,r,b.body,prev&&m3v(old.RT,sub(prev,old.origin)));
    if(c){c.n=m3v(b.pose.R,c.n);c.velocity=add(b.pose.v,crs(b.pose.w,sub(p,b.pose.pivot)));c.obj=b.obj;}return c;
  }
  if (b.mesh) return meshContact(sub(p,b.origin), r, b.mesh, prev && sub(prev,b.origin));
  const q = [clamp(p[0], b.lo[0], b.hi[0]), clamp(p[1], b.lo[1], b.hi[1]), clamp(p[2], b.lo[2], b.hi[2])];
  const d = sub(p, q), dl = nrm(d);
  if (dl > 1e-9) return dl < r ? { depth: r - dl, n: scl(d, 1 / dl) } : null;
  let best = Infinity, n = null;
  for (let i = 0; i < 3; i++) {
    const lo = p[i] - b.lo[i], hi = b.hi[i] - p[i];
    const cameLo = prev && prev[i] <= b.lo[i], cameHi = prev && prev[i] >= b.hi[i];
    const pen = (x, came) => came ? x - 1e3 : x;   // a face it crossed wins
    if (pen(lo, cameLo) < best) { best = pen(lo, cameLo); n = [0, 0, 0]; n[i] = -1; }
    if (pen(hi, cameHi) < best) { best = pen(hi, cameHi); n = [0, 0, 0]; n[i] = 1; }
  }
  const i = n.findIndex(x => x !== 0);
  return { depth: (n[i] < 0 ? p[i] - b.lo[i] : b.hi[i] - p[i]) + r, n };
}
// Every surface a sphere at p touches: the ground and nearby boxes.
function terrainContacts(p, r, near, prev) {
  const out = [];
  if (p[2] < r) out.push({ depth: r - p[2], n: [0, 0, 1], what: 'the ground', ground: true });
  for (const b of near) { const c = boxContact(p, r, b, prev); if (c) { c.what = b.what; out.push(c); } }
  return out;
}
const solidAt = (p, near) => p[2] < 0 ? 'the ground' : (near.find(b => insideBox(p, b)) || {}).what || null;
// The first surface along a ray from o in unit direction d: its distance (Infinity if none within maxD).
// groundMinDown: the ground counts only for rays pointing at least this steeply down.
function terrainRay(o, d, maxD = 1e4, groundMinDown = 0) {
  let t = d[2] < -Math.max(1e-6, groundMinDown) ? o[2] / -d[2] : Infinity;
  for(const b of terrainIndex()) {
    if(terrainShapeRay({lo:b.lo,hi:b.hi},o,d,Math.min(t,maxD))>=t)continue;
    for(const x of b.boxes||[b])t=Math.min(t,terrainShapeRay(x,o,d,Math.min(t,maxD)));
  }
  return t <= maxD ? t : Infinity;
}
// Height of the surface under p (the ground, or the top of whatever stands below it).
function surfaceBelow(p) {
  let h = 0;
  const under = b => p[0] > b.lo[0] && p[0] < b.hi[0] && p[1] > b.lo[1] && p[1] < b.hi[1];
  for (const b of terrainIndex()) {
    if (!under(b) || b.lo[2] > p[2] + 0.01) continue;
    if(b.mesh||b.body){const distance=terrainShapeRay(b,p,[0,0,-1],p[2]);if(Number.isFinite(distance))h=Math.max(h,p[2]-distance);continue;}
    for (const x of b.boxes || [b]) if (under(x)) {if(x.body){const distance=terrainShapeRay(x,p,[0,0,-1],p[2]);if(Number.isFinite(distance))h=Math.max(h,p[2]-distance);}else if(x.hi[2]<=p[2]+.01&&x.hi[2]>h)h=x.hi[2];}
  }
  return h;
}

// Ray and surface queries share the same rigid transform as contact resolution.
function terrainShapeRay(b,o,d,far=1e4){
  if(b.body)return terrainShapeRay(b.body,m3v(b.pose.RT,sub(o,b.pose.origin)),m3v(b.pose.RT,d),far);
  if(b.mesh)return meshRayHits(b.mesh,sub(o,b.origin),d,far)[0]?.distance??Infinity;
  let t0=0,t1=far;
  for(let i=0;i<3;i++){
    if(Math.abs(d[i])<1e-12){if(o[i]<b.lo[i]||o[i]>b.hi[i])return Infinity;continue;}
    let a=(b.lo[i]-o[i])/d[i],c=(b.hi[i]-o[i])/d[i];if(a>c)[a,c]=[c,a];t0=Math.max(t0,a);t1=Math.min(t1,c);if(t0>t1)return Infinity;
  }return t0;
}

function terrainSegmentHit(b,o,d,length){
  if(b.body)return terrainSegmentHit(b.body,m3v(b.pose.RT,sub(o,b.pose.origin)),m3v(b.pose.RT,d),length);
  if(b.mesh)return meshRayHits(b.mesh,sub(o,b.origin),d,length).some(h=>h.distance>1e-8&&h.distance<length-1e-8);
  const t=terrainShapeRay(b,o,d,length);return t>1e-8&&t<length-1e-8;
}
