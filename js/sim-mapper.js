'use strict';
// The simulator's mapper (docs/map-navigation.md): what a computer on the ground running Room Survey does for the
// drone's navigation, played from the World. It makes a map of the World around where the drone starts (the city's
// boxes, imported objects, a survey brought in as a model), sends it to the drone as the file map.dmp with the guard's
// radius (mapguard), and then fixes: where the drone truly is (with the noise, rate, delay and dropouts set) and how
// its flight core's heading has turned from the World's (mapfix). Over the radio when the design has one, so a lost
// link loses them as it would the real mapper's; otherwise straight to the navigation's board (a mapper reaching it
// another way: Room Survey on the Pi's own Wi-Fi).
// The drone's side is the real C (nav_core.c, map_core.c) in its board's WebAssembly, as dfb_pi runs it.
//
// The map comes from one of two sources. The World as it is: every cell free or occupied at once. Or what the drone's
// cameras see: the map starts unknown, and each camera on the frame (a camera part, the downward flow camera) casts
// rays across its view a few times a second, marking cells free up to what each ray meets (within the camera's
// reach) and that cell occupied, as a reconstruction from the pictures would; where the drone has been is free too.
// That map goes up again as it grows. Its settings are this page's, not the design's: the mapper isn't part of the drone.

const MAPPER_LS = 'dfb.mapper';
const MAPPER_DEFAULTS = Object.freeze({ on: false, source: 'world', route: 'link', extent: 6, height: 4, margin: 0.15, rate: 5, noise: 0.05, latency: 0.2, drop: 0, reach: 6, every: 2 });
const MAPPER_FIELDS = [
  ['extent', 'Map reach around the start', 'm', 2, 30, 0.5, 'How far the map reaches each way from where the drone starts. Past it the drone sees unknown space: it slows and goes no further than half a metre in.'],
  ['height', 'Map height', 'm', 1, 20, 0.5, 'From the ground up.'],
  ['margin', 'Margin past the drone', 'm', 0, 1, 0.05, 'Added to the drone\'s own radius (its props): how close the guard lets it come to what\'s mapped.'],
  ['rate', 'Fixes a second', 'Hz', 0.2, 30, 0.2, 'How often the mapper says where the drone is.'],
  ['noise', 'Fix noise', 'm', 0, 1, 0.01, 'Each fix is off by this much (one standard deviation, each axis).'],
  ['latency', 'Fix delay', 's', 0, 3, 0.05, 'From the moment a fix describes to when the mapper sends it (the processing). The radio adds its own.'],
  ['drop', 'Fixes lost', '%', 0, 100, 1, 'The share of fixes the mapper never sends (a picture it couldn\'t place).'],
  ['reach', 'Cameras see as far as', 'm', 1, 30, 0.5, 'From the cameras: how far a camera\'s picture places what it sees (past it, nothing is marked).', 'cameras'],
  ['every', 'Map sent again every', 's', 0.5, 30, 0.5, 'From the cameras: how often the grown map goes up again (when it changed).', 'cameras'],
];
const MAPPER_SOURCES = { world: 'The World as it is', cameras: 'What the drone\'s cameras see' };
const MAPPER_ROUTES = { link: 'Its own link to the navigation\'s computer (fast; lost with the radio)', radio: 'The radio\'s text commands (slow: a few lines a second)' };
const mapper = { cfg: { ...MAPPER_DEFAULTS }, st: null, el: null };
try { const s = JSON.parse(localStorage.getItem(MAPPER_LS) || 'null'); if (s && typeof s === 'object') for (const k of Object.keys(MAPPER_DEFAULTS)) if (typeof s[k] === typeof MAPPER_DEFAULTS[k]) mapper.cfg[k] = s[k]; } catch (e) {}
function mapperSave() { try { localStorage.setItem(MAPPER_LS, JSON.stringify(mapper.cfg)); } catch (e) {} }

/* ───────── the map file (map_core.h): the same bytes map_encode writes ───────── */
function mapFileEncode(res, origin, n, cells) {
  const out = []; const dv = new DataView(new ArrayBuffer(36));
  [68, 70, 77, 80].forEach((c, i) => dv.setUint8(i, c)); dv.setUint32(4, 1, true); dv.setFloat32(8, res, true);
  for (let i = 0; i < 3; i++) { dv.setFloat32(12 + 4 * i, origin[i], true); dv.setUint32(24 + 4 * i, n[i], true); }
  for (const b of new Uint8Array(dv.buffer)) out.push(b);
  const row = n[0], lay = n[0] * n[1];
  for (let i = 0; i < cells.length;) {                                // the longest of: the same state; as a row back; as a layer back
    let j = i; while (j < cells.length && cells[j] === cells[i]) j++;
    let best = j - i, kind = cells[i];
    if (i >= row) { let r = i; while (r < cells.length && cells[r] === cells[r - row]) r++; if (r - i > best) { best = r - i; kind = 3; } }
    if (i >= lay) { let r = i; while (r < cells.length && cells[r] === cells[r - lay]) r++; if (r - i > best) { best = r - i; kind = 4; } }
    out.push(kind); let v = best; do { const c = v & 0x7f; v = Math.floor(v / 128); out.push(c | (v ? 0x80 : 0)); } while (v);
    i += best;
  }
  const b = Uint8Array.from(out), c = crc32(b), r = new Uint8Array(b.length + 4); r.set(b);
  new DataView(r.buffer).setUint32(b.length, c, true); return r;
}
// The World around p0 as cells (0 unknown, 1 free, 2 occupied), the finest that fits the drone's store and the file link.
function mapperBuild(p0, seen) {   // seen: start unknown (what the cameras see), else the World as it is
  const C = mapper.cfg, MAX = 450000, FILE = 32000;
  const size = [2 * C.extent, 2 * C.extent, C.height], vol = size[0] * size[1] * size[2];
  let res = Math.max(0.05, Math.ceil(Math.cbrt(vol / MAX) * 100) / 100);
  const boxes = terrainNear([p0[0], p0[1], C.height / 2], Math.hypot(C.extent, C.extent, C.height) + 1);
  for (let tries = 0; tries < 12; tries++, res = Math.ceil(res * 125) / 100) {
    const n = size.map(s => Math.max(1, Math.round(s / res))), origin = [p0[0] - n[0] * res / 2, p0[1] - n[1] * res / 2, 0];
    const cells = new Uint8Array(n[0] * n[1] * n[2]).fill(seen ? 0 : 1);
    if (seen) return { cells, res, n, origin, occ: 0, bytes: mapFileEncode(res, origin, n, cells) };
    const idx = (x, y, z) => x + n[0] * (y + n[1] * z), clampI = (v, k) => Math.max(0, Math.min(n[k] - 1, v));
    for (const b of boxes) {
      const lo = [0, 1, 2].map(k => clampI(Math.floor((b.lo[k] - origin[k]) / res), k)), hi = [0, 1, 2].map(k => clampI(Math.ceil((b.hi[k] - origin[k]) / res) - 1, k));
      if ([0, 1, 2].some(k => b.hi[k] <= origin[k] || b.lo[k] >= origin[k] + n[k] * res)) continue;
      const exact = b.body || b.mesh;                                   // (a turned or meshed object: its cells by where their centres are)
      for (let z = lo[2]; z <= hi[2]; z++) for (let y = lo[1]; y <= hi[1]; y++) for (let x = lo[0]; x <= hi[0]; x++) {
        if (exact && !insideBox([origin[0] + (x + 0.5) * res, origin[1] + (y + 0.5) * res, origin[2] + (z + 0.5) * res], b)) continue;
        cells[idx(x, y, z)] = 2;
      }
    }
    const bytes = mapFileEncode(res, origin, n, cells);
    if (bytes.length <= FILE) return { bytes, cells, res, n, origin, occ: cells.reduce((s, c) => s + (c === 2), 0) };
  }
  return null;
}
// The drone's radius: the farthest prop tip from the hub, across.
function mapperRadius() {
  let r = 0.1;
  for (const c of cfg.comps) if (c.type === 'motor' && Array.isArray(c.pos)) r = Math.max(r, Math.hypot(c.pos[0], c.pos[1]) + (c.prop || 0));
  return r + mapper.cfg.margin;
}

/* ───────── what the cameras see ───────── */
// (its own small vector helpers: the page's count operations for the flight software's budget, and this isn't it)
const mv = (M, v) => [M[0] * v[0] + M[1] * v[1] + M[2] * v[2], M[3] * v[0] + M[4] * v[1] + M[5] * v[2], M[6] * v[0] + M[7] * v[1] + M[8] * v[2]];
const mvAdd = (a, b, k = 1) => [a[0] + k * b[0], a[1] + k * b[1], a[2] + k * b[2]];
const mvUnit = a => { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
// Each camera's eye in the World and the directions of its rays: a camera part looks along its +X (turned by its tilt
// and pan), a flow camera along its −Z.
function mapperEyes() {
  const R = qmat(S.q), eyes = [];
  for (const c of cameraParts()) {
    if (typeof onBoard === 'function' && !onBoard(c)) continue;
    const s = cameraSpec(c), P = poseOf(c, angleTrue), Rc = m3m(m3m(R, m3m(P.R, massRot(c))), eulerR(0, -s.tilt, s.pan));
    const hf = Math.min(s.fov, 170) * D2R / 2, vf = Math.atan(Math.tan(hf) * s.h / s.w);
    eyes.push({ p: add(S.p, m3v(R, P.p)), R: Rc, hf, vf, axis: [1, 0, 0], right: [0, -1, 0], up: [0, 0, 1] });
  }
  for (const c of sensorsOf('flow')) {
    const P = poseOf(c, angleTrue), Rs = m3m(m3m(R, P.R), eulerR(c.mount[0], c.mount[1], c.mount[2])), hf = 30 * D2R;
    eyes.push({ p: add(S.p, m3v(R, P.p)), R: Rs, hf, vf: hf, axis: [0, 0, -1], right: [1, 0, 0], up: [0, 1, 0] });
  }
  return eyes;
}
// Mark what the eyes see in the cells: free along each ray up to what it meets, that cell occupied; and where the drone
// is, free. Returns how many cells changed.
function mapperObserve(m, reach, radius) {
  const { cells, res, n, origin } = m; let changed = 0;
  const at = q => { const x = Math.floor((q[0] - origin[0]) / res), y = Math.floor((q[1] - origin[1]) / res), z = Math.floor((q[2] - origin[2]) / res);
    return x < 0 || y < 0 || z < 0 || x >= n[0] || y >= n[1] || z >= n[2] ? -1 : x + n[0] * (y + n[1] * z); };
  const set = (i, v) => { if (i >= 0 && cells[i] !== v && !(v === 1 && cells[i] === 2)) { cells[i] = v; changed++; } };
  const k = Math.ceil(radius / res);                                    // where the drone is: free
  for (let dz = -k; dz <= k; dz++) for (let dy = -k; dy <= k; dy++) for (let dx = -k; dx <= k; dx++) set(at([S.p[0] + dx * res, S.p[1] + dy * res, S.p[2] + dz * res]), 1);
  const NU = 24, NV = 16;
  for (const e of mapperEyes()) {
    const ax = mv(e.R, e.axis), rt = mv(e.R, e.right), up = mv(e.R, e.up);
    for (let a = 0; a < NU; a++) for (let b = 0; b < NV; b++) {
      const u = Math.tan(e.hf) * (2 * (a + 0.5) / NU - 1), v = Math.tan(e.vf) * (2 * (b + 0.5) / NV - 1);
      const d = mvUnit(mvAdd(mvAdd(ax, rt, u), up, v)), t = terrainRay(e.p, d, reach);
      const far = Math.min(t, reach), step = res * 0.5;
      for (let s = 0; s < far - step; s += step) { const i = at(mvAdd(e.p, d, s)); if (i >= 0 && cells[i] === 0) { cells[i] = 1; changed++; } }   // (free: only what wasn't known)
      if (t <= reach) set(at(mvAdd(e.p, d, t + step * 0.5)), 2);     // what it met (just inside it)
    }
  }
  return changed;
}

/* ───────── each step ───────── */
// How the mapper reaches the drone. Its own link: straight to the navigation's board (as Room Survey reaches the Pi's
// command port over the Wi-Fi), at once, but not while the drone's radio link is down (a design with a radio: they
// share the air). The radio's text commands (when the radio lands on the navigation's board): at the link's pace, a few
// lines a second, so a map takes a while.
const mapperRadio = () => { const t = boardOf('tlm'), n = boardOf('nav'); return mapper.cfg.route === 'radio' && !!(t && n && t.id === n.id && brt.gnd); };
const mapperLinkDown = () => !mapperRadio() && hasTask('tlm') && !!brt.gnd && typeof radioConnected === 'function' && !radioConnected();
function mapperSendLine(line) {
  if (mapperRadio()) { const why = cardTextOut(line); return why ? why : ''; }
  if (mapperLinkDown()) return 'the link is down';
  const r = simMapText(line); return r == null ? 'no navigation on the drone' : '';
}
// The map up: its file's lines queued (a newer map replaces the lines of one still going), sent each step as the link
// takes them (mapperPump): the link's own pace, in the page and in a test alike. Straight to the board, at once.
function mapperSendMap(st) {
  const b = st.map.bytes, P = 36, lines = [`file map.dmp ${b.length} ${crc32(b).toString(16).padStart(8, '0')}`];
  for (let off = 0; off < b.length; off += P) lines.push(`fd ${off} ${cardUpB64(b.subarray(off, off + P))}`);
  lines.push('file end', `mapguard ${mapperRadius().toFixed(2)}`);
  st.lastSend = brt.t;
  if (!mapperRadio()) {
    if (mapperLinkDown()) { st.needSend = true; st.say = 'the link is down: the map waits'; return; }
    st.needSend = false; const r = simMapLoad(b); st.say = /^map: \d/.test(r) ? `map given to the navigation (${b.length} bytes)` : r; simMapText(lines[lines.length - 1]); st.sent = true; return;
  }
  st.out = lines; st.outN = lines.length; st.outBytes = b.length;
}
function mapperPump(st) {
  // the fixes first (each held until the link takes it, dropped a second old); the map's lines with the room left
  st.q = st.q.filter(f => f.at > brt.t - 1 - mapper.cfg.latency);
  while (st.sent && st.q.length && st.q[0].at <= brt.t) {
    const why = mapperSendLine(st.q[0].line);
    if (why && /too many/.test(why)) return;
    st.q.shift(); if (!why) { st.n++; st.lines = (st.lines || 0) + 1; }
  }
  while (st.out && st.out.length) {
    const why = cardTextOut(st.out[0]);
    if (why && /too many/.test(why)) break;
    if (why) { st.say = 'map not sent: ' + why; st.out = null; return; }
    st.out.shift(); st.lines = (st.lines || 0) + 1;
    st.say = st.out.length ? `sending the map: ${Math.round(100 * (1 - st.out.length / st.outN))}%` : `map sent up the radio (${st.outBytes} bytes)`;
    if (!st.out.length) { st.out = null; st.sent = true; }
  }
}
function mapperStep() {
  const C = mapper.cfg;
  if (!C.on || !brt.ready || !boardOf('nav') || (typeof liveOn === 'function' && liveOn())) { mapper.st = null; return; }
  let st = mapper.st;
  if (!st || brt.t < st.t0 || st.ver !== terrain.ver) {               // (started, a reset, or the World changed: the map again)
    const seen = C.source === 'cameras';
    st = mapper.st = { t0: brt.t, ver: terrain.ver, seen, map: mapperBuild(S.p, seen), sent: false, nextFix: brt.t + 0.5, nextLook: brt.t, q: [], n: 0, rng: 12345 + (terrain.ver | 0), say: '', dirty: 0, sends: 0 };
    if (!st.map) { st.say = 'the World is too big for one map file: make the reach smaller'; return; }
    if (seen) { mapperObserve(st.map, C.reach, mapperRadius()); st.map.bytes = mapFileEncode(st.map.res, st.map.origin, st.map.n, st.map.cells); }
    st.say = 'map made'; mapperSendMap(st);
  }
  if (!st.map) return;
  if (st.needSend && !mapperLinkDown()) mapperSendMap(st);
  mapperPump(st);
  if (st.seen && brt.t >= st.nextLook) {                              // the cameras, 4 times a second; the map up again when it grew
    st.nextLook = brt.t + 0.25;
    st.dirty += mapperObserve(st.map, C.reach, mapperRadius());
    if (st.dirty && !st.out && brt.t - st.lastSend >= C.every) {
      const bytes = mapFileEncode(st.map.res, st.map.origin, st.map.n, st.map.cells);
      if (bytes.length <= 32000) { st.map.bytes = bytes; st.dirty = 0; st.sends++; mapperSendMap(st); }
      else st.say = 'the map grew past the 32 KB a file may be: make its reach smaller';
    }
  }
  const rand = () => { st.rng = (st.rng * 1103515245 + 12345) >>> 0; return st.rng / 4294967296; };
  const gauss = () => Math.sqrt(-2 * Math.log(rand() + 1e-12)) * Math.cos(2 * Math.PI * rand());
  if (brt.t >= st.nextFix) {
    st.nextFix = Math.max(st.nextFix + 1 / C.rate, brt.t);
    if (rand() * 100 >= C.drop) {
      const R = qmat(S.q), E = qmat(est.q), psi = Math.atan2(E[3], E[0]) - Math.atan2(R[3], R[0]);
      const p = S.p.map(v => v + C.noise * gauss()), a = ((psi + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
      st.q.push({ at: brt.t + C.latency, line: `mapfix ${brt.t.toFixed(3)} ${p[0].toFixed(3)} ${p[1].toFixed(3)} ${p[2].toFixed(3)} ${a.toFixed(4)} ${Math.max(C.noise, 0.01).toFixed(2)}` });
    }
  }
  if (!st.sent) st.q = st.q.filter(f => f.at > brt.t - 1);          // (fixes wait for the map; old ones aren't worth sending)
}

/* ───────── the panel: World settings → The navigation's map ───────── */
function mapperRender() {
  const box = document.getElementById('mapperBody'); if (!box) return;
  if (!mapper.el) {
    box.textContent = '';
    const on = el('input', { type: 'checkbox', id: 'mapperOn' }); on.checked = mapper.cfg.on;
    on.addEventListener('change', () => { mapper.cfg.on = on.checked; mapperSave(); mapper.st = null; mapperRender(); });
    box.append(el('label', { class: 'hrow' }, on, el('span', { text: ' Give the drone\'s navigation a map of the World, and fixes' })));
    const src = el('select', { 'aria-label': 'Where the map comes from' });
    for (const [k, t] of Object.entries(MAPPER_SOURCES)) { const o = el('option', { value: k, text: t }); if (mapper.cfg.source === k) o.selected = true; src.append(o); }
    src.addEventListener('change', () => { mapper.cfg.source = src.value; mapperSave(); mapper.st = null; mapper.el = null; mapperRender(); });
    box.append(el('label', { class: 'hrow' }, el('span', { text: 'The map from ' }), src));
    const rt = el('select', { 'aria-label': 'How the mapper reaches the drone' });
    for (const [k, t] of Object.entries(MAPPER_ROUTES)) { const o = el('option', { value: k, text: t }); if (mapper.cfg.route === k) o.selected = true; rt.append(o); }
    rt.addEventListener('change', () => { mapper.cfg.route = rt.value; mapperSave(); mapper.st = null; });
    box.append(el('label', { class: 'hrow' }, el('span', { text: 'It reaches the drone by ' }), rt));
    const fields = el('div', { class: 'sp' });
    for (const [k, label, unit, lo, hi, step, hint, only] of MAPPER_FIELDS) {
      if (only && only !== mapper.cfg.source) continue;
      const inp = el('input', { type: 'number', min: String(lo), max: String(hi), step: String(step), value: String(mapper.cfg[k]), title: hint, 'aria-label': label });
      inp.addEventListener('change', () => { const v = +inp.value; if (Number.isFinite(v)) { mapper.cfg[k] = Math.min(hi, Math.max(lo, v)); mapperSave(); if (['extent', 'height', 'margin', 'reach'].includes(k)) mapper.st = null; } inp.value = String(mapper.cfg[k]); });
      fields.append(el('label', { class: 'hrow', title: hint }, el('span', { text: `${label} (${unit})` }), inp));
    }
    box.append(fields, el('p', { class: 'hint', id: 'mapperSay', role: 'status' }));
    mapper.el = true;
  }
  const say = document.getElementById('mapperSay'); if (!say) return;
  const st = mapper.st, o = brt.navOut;
  if (!mapper.cfg.on) say.textContent = 'Off. On: the mapper (a ground computer with Room Survey, played by the simulator) sends the drone a map of the World around its start and where it is in it. The navigation then stops short of what\'s mapped. Over the radio when the design has one.';
  else if (!boardOf('nav')) say.textContent = 'This design has no navigation: give a board the Navigation task (Computers tab).';
  else if (!st) say.textContent = 'Waiting for the simulation to start.';
  else say.textContent = `${st.say}${st.map ? ` · ${st.map.n.join(' × ')} cells at ${st.map.res.toFixed(2)} m, ${st.seen ? `${(100 * st.map.cells.reduce((a, c) => a + (c !== 0), 0) / st.map.cells.length).toFixed(0)}% seen, sent ${st.sends + 1} times` : `${st.map.occ} occupied`}` : ''} · ${st.n} fixes sent${mapperRadio() ? ' over the radio' : ' over its own link'}${mapperLinkDown() ? ' · the link is down' : ''}${o && o.guard ? ` · guard: ${o.guard & 2 ? 'stopping short' : 'slowing'}` : ''}`;
}
setInterval(() => { try { mapperRender(); } catch (e) {} }, 500);
