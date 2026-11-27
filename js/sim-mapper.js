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
// The map is as known: every cell is free or occupied, from the World as it is. (Mapping as the cameras see it, cell by
// cell, comes later.) Its settings are this page's, not the design's: the mapper isn't part of the drone.

const MAPPER_LS = 'dfb.mapper';
const MAPPER_DEFAULTS = Object.freeze({ on: false, extent: 6, height: 4, margin: 0.15, rate: 5, noise: 0.05, latency: 0.2, drop: 0 });
const MAPPER_FIELDS = [
  ['extent', 'Map reach around the start', 'm', 2, 30, 0.5, 'How far the map reaches each way from where the drone starts. Past it the drone sees unknown space: it slows and goes no further than half a metre in.'],
  ['height', 'Map height', 'm', 1, 20, 0.5, 'From the ground up.'],
  ['margin', 'Margin past the drone', 'm', 0, 1, 0.05, 'Added to the drone\'s own radius (its props): how close the guard lets it come to what\'s mapped.'],
  ['rate', 'Fixes a second', 'Hz', 0.2, 30, 0.2, 'How often the mapper says where the drone is.'],
  ['noise', 'Fix noise', 'm', 0, 1, 0.01, 'Each fix is off by this much (one standard deviation, each axis).'],
  ['latency', 'Fix delay', 's', 0, 3, 0.05, 'From the moment a fix describes to when the mapper sends it (the processing). The radio adds its own.'],
  ['drop', 'Fixes lost', '%', 0, 100, 1, 'The share of fixes the mapper never sends (a picture it couldn\'t place).'],
];
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
function mapperBuild(p0) {
  const C = mapper.cfg, MAX = 450000, FILE = 32000;
  const size = [2 * C.extent, 2 * C.extent, C.height], vol = size[0] * size[1] * size[2];
  let res = Math.max(0.05, Math.ceil(Math.cbrt(vol / MAX) * 100) / 100);
  const boxes = terrainNear([p0[0], p0[1], C.height / 2], Math.hypot(C.extent, C.extent, C.height) + 1);
  for (let tries = 0; tries < 12; tries++, res = Math.ceil(res * 125) / 100) {
    const n = size.map(s => Math.max(1, Math.round(s / res))), origin = [p0[0] - n[0] * res / 2, p0[1] - n[1] * res / 2, 0];
    const cells = new Uint8Array(n[0] * n[1] * n[2]).fill(1);
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
    if (bytes.length <= FILE) return { bytes, res, n, origin, occ: cells.reduce((s, c) => s + (c === 2), 0) };
  }
  return null;
}
// The drone's radius: the farthest prop tip from the hub, across.
function mapperRadius() {
  let r = 0.1;
  for (const c of cfg.comps) if (c.type === 'motor' && Array.isArray(c.pos)) r = Math.max(r, Math.hypot(c.pos[0], c.pos[1]) + (c.prop || 0));
  return r + mapper.cfg.margin;
}

/* ───────── each step ───────── */
// Where the mapper sends: up the radio (the command module's text) when the radio lands on the navigation's board, or to
// that board itself.
const mapperRadio = () => { const t = boardOf('tlm'), n = boardOf('nav'); return !!(t && n && t.id === n.id && brt.gnd); };   // (the radio lands where the navigation runs; else the mapper reaches that board itself, as Room Survey reaches the Pi over Wi-Fi)
function mapperSendLine(line) {
  if (mapperRadio()) { const why = cardTextOut(line); return why ? why : ''; }
  const r = simMapText(line); return r == null ? 'no navigation on the drone' : '';
}
async function mapperSendMap(st) {
  const b = st.map.bytes, P = 36, lines = [`file map.dmp ${b.length} ${crc32(b).toString(16).padStart(8, '0')}`];
  for (let off = 0; off < b.length; off += P) lines.push(`fd ${off} ${cardUpB64(b.subarray(off, off + P))}`);
  lines.push('file end', `mapguard ${mapperRadius().toFixed(2)}`);
  if (!mapperRadio()) { const r = simMapLoad(b); st.say = /^map: \d/.test(r) ? `map given to the navigation (${b.length} bytes)` : r; simMapText(lines[lines.length - 1]); st.sent = true; return; }
  for (let i = 0; i < lines.length; i++) {
    if (mapper.st !== st) return;                                     // (a reset, or the mapper switched off: this one is over)
    const why = cardTextOut(lines[i]);
    if (why && /too many/.test(why)) { i--; await new Promise(r => setTimeout(r, 30)); continue; }
    if (why) { st.say = 'map not sent: ' + why; return; }
    st.say = `sending the map: ${Math.round(100 * (i + 1) / lines.length)}%`;
  }
  st.sent = true; st.say = `map sent up the radio (${b.length} bytes)`;
}
function mapperStep() {
  const C = mapper.cfg;
  if (!C.on || !brt.ready || !boardOf('nav') || (typeof liveOn === 'function' && liveOn())) { mapper.st = null; return; }
  let st = mapper.st;
  if (!st || brt.t < st.t0 || st.ver !== terrain.ver) {               // (started, a reset, or the World changed: the map again)
    st = mapper.st = { t0: brt.t, ver: terrain.ver, map: mapperBuild(S.p), sent: false, nextFix: brt.t + 0.5, q: [], n: 0, rng: 12345 + (terrain.ver | 0), say: '' };
    if (!st.map) { st.say = 'the World is too big for one map file: make the reach smaller'; return; }
    st.say = 'map made'; mapperSendMap(st);
  }
  if (!st.map) return;
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
  while (st.sent && st.q.length && st.q[0].at <= brt.t) { if (!mapperSendLine(st.q.shift().line)) st.n++; }
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
    const fields = el('div', { class: 'sp' });
    for (const [k, label, unit, lo, hi, step, hint] of MAPPER_FIELDS) {
      const inp = el('input', { type: 'number', min: String(lo), max: String(hi), step: String(step), value: String(mapper.cfg[k]), title: hint, 'aria-label': label });
      inp.addEventListener('change', () => { const v = +inp.value; if (Number.isFinite(v)) { mapper.cfg[k] = Math.min(hi, Math.max(lo, v)); mapperSave(); if (['extent', 'height', 'margin'].includes(k)) mapper.st = null; } inp.value = String(mapper.cfg[k]); });
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
  else say.textContent = `${st.say}${st.map ? ` · ${st.map.n.join(' × ')} cells at ${st.map.res.toFixed(2)} m, ${st.map.occ} occupied` : ''} · ${st.n} fixes sent${mapperRadio() ? ' over the radio' : ' to the board'}${o && o.guard ? ` · guard: ${o.guard & 2 ? 'stopping short' : 'slowing'}` : ''}`;
}
setInterval(() => { try { mapperRender(); } catch (e) {} }, 500);
