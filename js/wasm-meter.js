'use strict';
// Apps' step limit (docs/apps.md): wasmMeter(bytes) returns the module with an exported mutable i32 global ll_fuel,
// taken down by one at the top of every loop; a loop that finds it at 0 traps (unreachable). The board's app host
// sets it before each call, so a loop that never ends stops instead of hanging the board (or the page). The same
// metered module runs in the simulator and on the boards.
function wasmMeter(src) {
  const u = src instanceof Uint8Array ? src : new Uint8Array(src);
  let p = 0;
  const fail = m => { throw new Error('can\'t meter this module: ' + m); };
  const leb = () => { let r = 0, s = 0, b; do { b = u[p++]; r |= (b & 0x7f) << s; s += 7; } while (b & 0x80 && s < 35); if (b & 0x80) fail('bad number'); return r >>> 0; };
  const skipLeb = () => { while (u[p++] & 0x80) if (p > u.length) fail('truncated'); };
  const enc = n => { const o = []; do { let b = n & 0x7f; n >>>= 7; if (n) b |= 0x80; o.push(b); } while (n); return o; };
  const skipName = () => { const n = leb(); p += n; };
  if (u.length < 8 || u[0] !== 0 || u[1] !== 0x61 || u[2] !== 0x73 || u[3] !== 0x6d) fail('not WebAssembly');
  p = 8;
  const secs = [];
  while (p < u.length) { const head = p, id = u[p++], n = leb(); secs.push({ id, head, start: p, end: p + n }); p += n; }
  const sec = id => secs.find(s => s.id === id);
  // the counter's index: after the imported and the defined globals
  let nGlobals = 0;
  if (sec(2)) {
    p = sec(2).start; const n = leb();
    for (let i = 0; i < n; i++) {
      skipName(); skipName(); const k = u[p++];
      if (k === 0) leb(); else if (k === 1) { p++; const f = u[p++]; leb(); if (f & 1) leb(); }
      else if (k === 2) { const f = u[p++]; leb(); if (f & 1) leb(); } else if (k === 3) { p += 2; nGlobals++; } else fail('import kind ' + k);
    }
  }
  const counter = [0x7f, 1, 0x41, 0, 0x0b];          // (mut i32) = 0
  let globalSec;
  if (sec(6)) { p = sec(6).start; const n = leb(); nGlobals += n; globalSec = [...enc(n + 1), ...u.subarray(p, sec(6).end), ...counter]; }
  else globalSec = [1, ...counter];
  const gIdx = enc(nGlobals);
  if (!sec(7)) fail('no exports');
  p = sec(7).start; const ne = leb();
  const exportSec = [...enc(ne + 1), ...u.subarray(p, sec(7).end), 7, ...new TextEncoder().encode('ll_fuel'), 3, ...gIdx];
  // the check at the top of every loop
  const check = [0x23, ...gIdx, 0x45, 0x04, 0x40, 0x00, 0x0b, 0x23, ...gIdx, 0x41, 1, 0x6b, 0x24, ...gIdx];
  if (!sec(10)) fail('no code');
  p = sec(10).start; const nf = leb(), code = [...enc(nf)];
  const blockType = () => { const b = u[p]; if (b === 0x40 || (b >= 0x6f && b <= 0x7f)) p++; else skipLeb(); };
  for (let f = 0; f < nf; f++) {
    const size = leb(), end = p + size, out = [];
    let from = p;
    const nl = leb(); for (let i = 0; i < nl; i++) { leb(); p++; }
    while (p < end) {
      const op = u[p++];
      if (op === 0x03) { blockType(); out.push(u.subarray(from, p), check); from = p; }
      else if (op === 0x02 || op === 0x04) blockType();
      else if (op === 0x0c || op === 0x0d || op === 0x10 || op === 0x12 || (op >= 0x20 && op <= 0x26) || op === 0xd2) leb();
      else if (op === 0x0e) { const n = leb(); for (let i = 0; i <= n; i++) leb(); }
      else if (op === 0x11 || op === 0x13) { leb(); leb(); }
      else if (op === 0x1c) { const n = leb(); p += n; }
      else if (op >= 0x28 && op <= 0x3e) { leb(); leb(); }
      else if (op === 0x3f || op === 0x40) leb();
      else if (op === 0x41 || op === 0x42) skipLeb();
      else if (op === 0x43) p += 4;
      else if (op === 0x44) p += 8;
      else if (op === 0xd0) p++;
      else if (op === 0xfc) {
        const s = leb();
        if (s <= 7) { }
        else if (s === 9 || s === 11 || s === 13 || (s >= 15 && s <= 17)) leb();
        else if (s === 8 || s === 10 || s === 12 || s === 14) { leb(); leb(); }
        else fail('instruction 0xfc ' + s);
      }
      else if (!(op <= 0x01 || op === 0x05 || op === 0x0b || op === 0x0f || op === 0x1a || op === 0x1b || (op >= 0x45 && op <= 0xc4) || op === 0xd1)) fail('instruction 0x' + op.toString(16));
    }
    if (p !== end) fail('a function\'s end');
    out.push(u.subarray(from, end));
    code.push(...enc(out.reduce((s, x) => s + x.length, 0))); for (const x of out) for (let i = 0; i < x.length; i++) code.push(x[i]);
  }
  // put it back together; a new global section goes before the first section that must follow it
  const parts = [u.subarray(0, 8)], put = (id, body) => parts.push(Uint8Array.of(id, ...enc(body.length)), Uint8Array.from(body));
  const AFTER_GLOBAL = new Set([7, 8, 9, 12, 10, 11]);
  let placed = !!sec(6);
  for (const s of secs) {
    if (!placed && AFTER_GLOBAL.has(s.id)) { put(6, globalSec); placed = true; }
    if (s.id === 6) put(6, globalSec); else if (s.id === 7) put(7, exportSec); else if (s.id === 10) put(10, code); else parts.push(u.subarray(s.head, s.end));
  }
  const r = new Uint8Array(parts.reduce((s, x) => s + x.length, 0)); let k = 0;
  for (const x of parts) { r.set(x, k); k += x.length; }
  return r;
}
if (typeof module !== 'undefined') module.exports = { wasmMeter };
