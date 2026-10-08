'use strict';
// The step runner in JavaScript: the reference the C runner (runner/rn.c) is checked against, and the fallback
// where WebAssembly isn't available. It works on the same arena layout, so the two are interchangeable.
// Also here: writing JavaScript values into the arena and reading them back, by type.

class RnTrap extends Error { constructor(msg, pc) { super(msg); this.pc = pc; } }

// Load a program: check it can't touch anything outside its arena or write its constants, then set up the arena.
function rnVerify(P) {
  const code = P.code, n = P.arenaSize, ce = P.constEnd;
  const fail = (m, pc) => { throw new RnTrap('Rejected: ' + m + ' at ' + pc, pc); };
  const starts = new Set();
  for (const f of Object.values(P.fns)) { let pc = f.entry; while (pc < f.end) { starts.add(pc); const op = code[pc]; if (!(op >= 0 && op < RN_OPS.length)) fail('unknown step ' + op, pc); pc += 1 + RN_OPS[op][1].length; } if (pc !== f.end) fail('a step runs past the end', pc); }
  for (const f of Object.values(P.fns)) {
    let pc = f.entry;
    while (pc < f.end) {
      const op = code[pc], [name, spec, blk] = RN_OPS[op], a = i => code[pc + 1 + i];
      spec.split('').forEach((k, i) => {
        const v = a(i);
        if (k === 'd' || k === 'r') { if (!(v >= ce && v < n)) fail(`${name} writes outside the arena`, pc); }
        else if (k === 'a' || k === 'R') { if (!(v >= 0 && v < n)) fail(`${name} reads outside the arena`, pc); }
        else if (k === 'o' || k === 'g') { if (!(v === -1 || (v >= 0 && v < n))) fail(`${name} reads outside the arena`, pc); }
        else if (k === 't') { if (!(starts.has(v) || v === f.end) || v < f.entry || v > f.end) fail(`${name} jumps outside its formula`, pc); }
      });
      for (const [oi, rw, sz] of blk || []) {
        const base = a(oi); if (base === -1 && spec[oi] === 'o') continue;
        let size;
        if (typeof sz === 'number') size = sz;
        else if (sz[0] === '$') size = a(+sz.slice(1));
        else if (sz[0] === 'N') size = 1 + a(+sz.slice(1));
        else { const [s, c] = sz.slice(1).split(',').map(Number); size = (sz[0] === 'R' ? 2 : 1) + a(c) * a(s); }
        if (!(size >= 0 && base >= 0 && base + size <= n)) fail(`${name} touches a block outside the arena`, pc);
        if (rw === 'w' && base < ce) fail(`${name} writes the constants`, pc);
      }
      if (RN_VIEWS[name]) {                                 // views without a register: inside the arena for cap elements
        const cap = a(spec.length - 1);
        for (const [ri, oi, si, w] of RN_VIEWS[name]) {
          if (a(ri) !== -1) continue;
          const off = a(oi), st = a(si), last = off + st * Math.max(0, cap - 1);
          if (st < 0 || off < 0 || last >= n) fail(`${name} touches a block outside the arena`, pc);
          if (w && off < ce) fail(`${name} writes the constants`, pc);
        }
      }
      pc += 1 + spec.length;
    }
  }
  return true;
}
function rnArena(P) { const A = new Float32Array(P.arenaSize); A.set(P.constData, 0); return A; }

const rnTruthy = x => x !== 0 && x === x;
const rnRound = x => Math.floor(x + 0.5);
// Run one formula. Returns the number of steps taken; throws RnTrap on a run-time error.
function rnRun(P, A, key, stats) {
  const f = P.fns[key]; const code = P.code, ce = P.constEnd, n = A.length;
  let pc = f.entry, steps = 0; const end = f.end, max = f.maxSteps;
  const trap = m => { throw new RnTrap(`${key}: ${m} (step ${pc - f.entry})`, pc); };
  const addr = v => { if (!(v >= 0 && v < n && Number.isInteger(v))) trap('address outside the arena'); return v; };
  const waddr = v => { if (!(v >= ce && v < n && Number.isInteger(v))) trap('write outside the arena'); return v; };
  while (pc < end) {
    if (++steps > max) trap('took more steps than its limit');
    if (stats) stats[pc] = (stats[pc] || 0) + 1;
    const op = code[pc];
    const o1 = code[pc + 1], o2 = code[pc + 2], o3 = code[pc + 3], o4 = code[pc + 4];
    switch (op) {
      case 0: pc += 1; break;                                              // NOP
      case 1: A[o1] = A[o2]; pc += 3; break;                               // MOV
      case 2: A[o1] = A[o2] + A[o3]; pc += 4; break;
      case 3: A[o1] = A[o2] - A[o3]; pc += 4; break;
      case 4: A[o1] = A[o2] * A[o3]; pc += 4; break;
      case 5: A[o1] = A[o2] / A[o3]; pc += 4; break;
      case 6: A[o1] = A[o2] % A[o3]; pc += 4; break;
      case 7: A[o1] = Math.pow(A[o2], A[o3]); pc += 4; break;
      case 8: A[o1] = Math.min(A[o2], A[o3]); pc += 4; break;
      case 9: A[o1] = Math.max(A[o2], A[o3]); pc += 4; break;
      case 10: A[o1] = Math.atan2(A[o2], A[o3]); pc += 4; break;
      case 11: A[o1] = A[o2] < A[o3] ? 1 : 0; pc += 4; break;
      case 12: A[o1] = A[o2] <= A[o3] ? 1 : 0; pc += 4; break;
      case 13: A[o1] = A[o2] === A[o3] ? 1 : 0; pc += 4; break;
      case 14: A[o1] = A[o2] !== A[o3] ? 1 : 0; pc += 4; break;
      case 15: A[o1] = -A[o2]; pc += 3; break;
      case 16: A[o1] = Math.abs(A[o2]); pc += 3; break;
      case 17: A[o1] = Math.sqrt(A[o2]); pc += 3; break;
      case 18: A[o1] = Math.sin(A[o2]); pc += 3; break;
      case 19: A[o1] = Math.cos(A[o2]); pc += 3; break;
      case 20: A[o1] = Math.tan(A[o2]); pc += 3; break;
      case 21: A[o1] = Math.asin(A[o2]); pc += 3; break;
      case 22: A[o1] = Math.acos(A[o2]); pc += 3; break;
      case 23: A[o1] = Math.atan(A[o2]); pc += 3; break;
      case 24: A[o1] = Math.exp(A[o2]); pc += 3; break;
      case 25: A[o1] = Math.log(A[o2]); pc += 3; break;
      case 26: A[o1] = Math.floor(A[o2]); pc += 3; break;
      case 27: A[o1] = Math.ceil(A[o2]); pc += 3; break;
      case 28: A[o1] = rnRound(A[o2]); pc += 3; break;
      case 29: A[o1] = Math.sign(A[o2]); pc += 3; break;
      case 30: A[o1] = rnTruthy(A[o2]) ? 0 : 1; pc += 3; break;          // NOT
      case 31: A[o1] = rnTruthy(A[o2]) ? 1 : 0; pc += 3; break;          // TRUTH
      case 32: A[o1] = rnTruthy(A[o2]) ? A[o3] : A[o4]; pc += 5; break;  // SEL
      case 33: { const x = A[o2], lo = A[o3], hi = A[o4]; A[o1] = x < lo ? lo : x > hi ? hi : x; pc += 5; break; }   // CLAMP (as math.js)
      case 34: A[o1] = A[o2] * A[o3] + A[o4]; pc += 5; break;            // FMA
      case 35: pc = o1; break;                                            // JMP
      case 36: pc = rnTruthy(A[o1]) ? pc + 3 : o2; break;                // JZ
      case 37: pc = rnTruthy(A[o1]) ? o2 : pc + 3; break;                // JNZ
      case 38: trap('stopped by the formula (code ' + o1 + ')'); break;
      case 39: A.copyWithin(o1, o2, o2 + o3); pc += 4; break;            // CPY
      case 40: A.fill(A[o2], o1, o1 + o3); pc += 4; break;               // FILL
      case 41: { const len = A[o2]; if (!(len >= 0 && len <= o3 && Number.isInteger(len))) trap(`list length ${len} is more than its ${o3} places`); A[o1] = len; pc += 5; break; }  // LLEN list n cap stride
      case 42: { const len = A[o1]; A.fill(A[o2], o1 + 1, o1 + 1 + len * o3); pc += 5; break; }          // LFILL list v stride cap
      case 43: { const len = A[o2]; if (!(len >= 0 && len <= o4)) trap(`a list of ${len} doesn't fit in ${o4} places`); A.copyWithin(o1 + 1, o2 + 1, o2 + 1 + len * o3); A[o1] = len; pc += 5; break; }   // CPYL dst src stride dcap
      case 44: { const len = A[o1]; if (len >= o4) trap(`list full (${o4} places)`); A.copyWithin(o1 + 1 + len * o3, o2, o2 + o3); A[o1] = len + 1; pc += 5; break; }   // PUSH list src stride cap
      case 45: { const len = A[o1], head = A[o1 + 1]; if (len >= o4) trap(`list full (${o4} places)`); const at = (head + len) % o4; A.copyWithin(o1 + 2 + at * o3, o2, o2 + o3); A[o1] = len + 1; pc += 5; break; }   // RPUSH
      case 46: { const len = A[o1]; if (len > 0) { A[o1] = len - 1; A[o1 + 1] = (A[o1 + 1] + 1) % o3; } pc += 4; break; }   // RSHIFT ring stride cap
      case 47: A[o1] = 0; A[o1 + 1] = 0; pc += 4; break;                  // RCLR
      case 48: { const i = A[o3], b = A[o4]; if (!(i >= 0 && i < b && Number.isInteger(i))) trap(`index ${i} outside 0…${b - 1}`); A[waddr(o1)] = o2 + i * code[pc + 5]; pc += 6; break; }   // IDX r base idx bound stride
      case 49: { const i = A[o4], b = A[code[pc + 5]]; if (!(i >= 0 && i < b && Number.isInteger(i))) trap(`index ${i} outside 0…${b - 1}`); A[o1] = A[o2] + o3 + i * code[pc + 6]; pc += 7; break; }   // IDXI r rb off idx bound stride
      case 50: { const i = A[o3], len = A[o2]; if (!(i >= 0 && i < len && Number.isInteger(i))) trap(`index ${i} outside 0…${len - 1}`); const cap = code[pc + 5]; A[o1] = o2 + 2 + ((A[o2 + 1] + i) % cap) * o4; pc += 6; break; }   // RIDX r ring idx stride cap
      case 51: A[o1] = A[addr(A[o2] + o3)]; pc += 4; break;               // LDI d r off
      case 52: A[waddr(A[o1] + o2)] = A[o3]; pc += 4; break;              // STI r off s
      case 53: { const s = addr(A[o2] + o3); addr(s + o4 - 1); A.copyWithin(o1, s, s + o4); pc += 5; break; }   // CPI dst r off n
      case 54: { const d = waddr(A[o1] + o2); waddr(d + o4 - 1); A.copyWithin(d, o3, o3 + o4); pc += 5; break; }   // CPO r off src n
      case 55: A[o1] = o2; pc += 3; break;                                 // AR
      case 56: { const M = o2, v = o3, x = A[v], y = A[v + 1], z = A[v + 2]; A[o1] = A[M] * x + A[M + 1] * y + A[M + 2] * z; A[o1 + 1] = A[M + 3] * x + A[M + 4] * y + A[M + 5] * z; A[o1 + 2] = A[M + 6] * x + A[M + 7] * y + A[M + 8] * z; pc += 4; break; }
      case 57: { const a = A.slice(o2, o2 + 9), b = A.slice(o3, o3 + 9); for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) A[o1 + 3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j]; pc += 4; break; }
      case 58: { const m = A.slice(o2, o2 + 9); for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) A[o1 + 3 * i + j] = m[3 * j + i]; pc += 3; break; }
      case 59: { const a0 = A[o2], a1 = A[o2 + 1], a2 = A[o2 + 2], b0 = A[o3], b1 = A[o3 + 1], b2 = A[o3 + 2]; A[o1] = a1 * b2 - a2 * b1; A[o1 + 1] = a2 * b0 - a0 * b2; A[o1 + 2] = a0 * b1 - a1 * b0; pc += 4; break; }
      case 60: { const a = A.slice(o2, o2 + 4), b = A.slice(o3, o3 + 4); A[o1] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3]; A[o1 + 1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2]; A[o1 + 2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1]; A[o1 + 3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]; pc += 4; break; }
      case 61: { const R = qmat(Array.from(A.slice(o2, o2 + 4))); A.set(R, o1); pc += 3; break; }
      case 62: { const q = A.slice(o2, o2 + 4), m = Math.hypot(q[0], q[1], q[2], q[3]); for (let i = 0; i < 4; i++) A[o1 + i] = q[i] / m; pc += 3; break; }
      case 63: { A.set(matToQuat(Array.from(A.slice(o2, o2 + 9))), o1); pc += 3; break; }
      case 64: { const x = A[o2], y = A[o2 + 1], z = A[o2 + 2], m = Math.hypot(x, y, z); if (m > 1e-12) { A[o1] = x / m; A[o1 + 1] = y / m; A[o1 + 2] = z / m; } else { A[o1] = 0; A[o1 + 1] = 0; A[o1 + 2] = 1; } pc += 3; break; }
      case 65: A[o1] = Math.hypot(A[o2], A[o2 + 1], A[o2 + 2]); pc += 3; break;
      case 66: A[o1] = A[o2] * A[o3] + A[o2 + 1] * A[o3 + 1] + A[o2 + 2] * A[o3 + 2]; pc += 4; break;
      case 67: { const a0 = A[o2] + A[o3], a1 = A[o2 + 1] + A[o3 + 1], a2 = A[o2 + 2] + A[o3 + 2]; A[o1] = a0; A[o1 + 1] = a1; A[o1 + 2] = a2; pc += 4; break; }
      case 68: { const a0 = A[o2] - A[o3], a1 = A[o2 + 1] - A[o3 + 1], a2 = A[o2 + 2] - A[o3 + 2]; A[o1] = a0; A[o1 + 1] = a1; A[o1 + 2] = a2; pc += 4; break; }
      case 69: { const s = A[o3]; A[o1] = A[o2] * s; A[o1 + 1] = A[o2 + 1] * s; A[o1 + 2] = A[o2 + 2] * s; pc += 4; break; }
      case 70: {                                                           // BLS
        const [d, cl, lo, hi, w, W, pq, pr, rel, K, cap] = code.subarray(pc + 1, pc + 12);
        const nc = A[cl], cols = [], L = [], H = [];
        for (let j = 0; j < nc; j++) { cols.push(Array.from(A.subarray(cl + 1 + j * K, cl + 1 + (j + 1) * K))); L.push(A[lo + 1 + j]); H.push(A[hi + 1 + j]); }
        const pull = pq >= 0 ? { q: Array.from(A.subarray(pq + 1, pq + 1 + A[pq])), r: pr >= 0 ? Array.from(A.subarray(pr + 1, pr + 1 + A[pr])) : [], rel: A[rel] } : undefined;
        const x = rnBls(cols, L, H, Array.from(A.subarray(w, w + K)), Array.from(A.subarray(W, W + K)), pull);
        A[d] = x.length; A.set(x, d + 1); pc += 12; break;
      }
      case 71: case 72: case 73: {                                       // VV, VS, VDOT: fused list steps
        const view = (i, w, cnt) => {
          const r = code[pc + 1 + i], off = code[pc + 2 + i], st = code[pc + 3 + i], b = (r >= 0 ? A[r] : 0) + off;
          if (r >= 0 && cnt > 0 && !(b >= (w ? ce : 0) && b + st * (cnt - 1) < n && Number.isInteger(b))) trap('list step outside the arena');
          return [b, st];
        };
        const f2 = (kind, u, v) => kind === 0 ? u + v : kind === 1 ? u - v : kind === 2 ? u * v : kind === 3 ? u / v : kind === 4 ? v - u : v / u;
        if (op === 71) {
          const kind = o1, cnt = A[code[pc + 11]], cap = code[pc + 12]; if (!(cnt >= 0 && cnt <= cap)) trap(`${cnt} elements, room for ${cap}`);
          const [d, ds] = view(1, 1, cnt), [x, xs] = view(4, 0, cnt), [y, ys] = view(7, 0, cnt);
          for (let k = 0; k < cnt; k++) A[d + k * ds] = f2(kind, A[x + k * xs], A[y + k * ys]);
          pc += 13; break;
        }
        if (op === 72) {
          const kind = o1, cnt = A[code[pc + 9]], cap = code[pc + 10]; if (!(cnt >= 0 && cnt <= cap)) trap(`${cnt} elements, room for ${cap}`);
          const [d, ds] = view(1, 1, cnt), [x, xs] = view(4, 0, cnt), v = A[code[pc + 8]];
          for (let k = 0; k < cnt; k++) A[d + k * ds] = f2(kind, A[x + k * xs], v);
          pc += 11; break;
        }
        const cnt = A[code[pc + 8]], cap = code[pc + 9]; if (!(cnt >= 0 && cnt <= cap)) trap(`${cnt} elements, room for ${cap}`);
        const [x, xs] = view(1, 0, cnt), [y, ys] = view(4, 0, cnt);
        let acc = 0; for (let k = 0; k < cnt; k++) acc = Math.fround(acc + Math.fround(A[x + k * xs] * A[y + k * ys]));
        A[o1] = acc; pc += 10; break;
      }
      default: trap('unknown step ' + op);
    }
  }
  if (stats) { stats.steps = (stats.steps || 0) + steps; stats.calls = (stats.calls || 0) + 1; stats.max = Math.max(stats.max || 0, steps); }
  return steps;
}
const rnBls = (...a) => (typeof bls === 'function' ? bls : globalThis.bls)(...a);

/* ───────── values in and out of the arena ───────── */
function rnWrite(A, at, t, v, path) {
  switch (t.k) {
    case 'num': A[at] = v === true ? 1 : v === false ? 0 : v; return;
    case 'enum': { const i = t.vals.indexOf(v); if (i < 0) throw new Error(`${path}: '${v}' isn't one of ${t.vals.join(', ')}`); A[at] = i; return; }
    case 'arr': { const es = tsize(t.el); if (!v || v.length < t.n) throw new Error(`${path}: expected ${t.n} elements`); if (t.el.k === 'num') { for (let i = 0; i < t.n; i++) A[at + i] = v[i]; return; } for (let i = 0; i < t.n; i++) rnWrite(A, at + i * es, t.el, v[i], path + '[' + i + ']'); return; }
    case 'list': { if (!v || v.length > t.cap) throw new Error(`${path}: ${v ? v.length : 'no'} elements, room for ${t.cap}`); A[at] = v.length; const es = tsize(t.el); if (t.el.k === 'num') { for (let i = 0; i < v.length; i++) A[at + 1 + i] = v[i]; return; } for (let i = 0; i < v.length; i++) rnWrite(A, at + 1 + i * es, t.el, v[i], path + '[' + i + ']'); return; }
    case 'ring': { if (v.length > t.cap) throw new Error(`${path}: too long`); A[at] = v.length; A[at + 1] = 0; const es = tsize(t.el); for (let i = 0; i < v.length; i++) rnWrite(A, at + 2 + i * es, t.el, v[i], path); return; }
    case 'rec': { let o = 0; for (const [k, ft] of Object.entries(t.f)) { const fv = v[k]; if (ft.k !== 'opt' && fv == null) throw new Error(`${path}.${k} is missing`); rnWrite(A, at + o, ft, fv, path + '.' + k); o += tsize(ft); } return; }
    case 'opt': if (v == null) { A[at] = 0; return; } A[at] = 1; rnWrite(A, at + 1, t.t, v, path); return;
  }
  throw new Error('internal: write ' + t.k);
}
function rnRead(A, at, t, into) {
  switch (t.k) {
    case 'num': return A[at];
    case 'enum': return t.vals[A[at]];
    case 'arr': { const es = tsize(t.el), o = Array.isArray(into) && into.length === t.n ? into : new Array(t.n); for (let i = 0; i < t.n; i++) o[i] = t.el.k === 'num' ? A[at + i] : rnRead(A, at + i * es, t.el, o[i]); return o; }
    case 'list': { const n = A[at], es = tsize(t.el), o = Array.isArray(into) && into.length === n ? into : new Array(n); for (let i = 0; i < n; i++) o[i] = t.el.k === 'num' ? A[at + 1 + i] : rnRead(A, at + 1 + i * es, t.el, o[i]); return o; }
    case 'ring': { const n = A[at], h = A[at + 1], es = tsize(t.el), o = []; for (let i = 0; i < n; i++) o.push(rnRead(A, at + 2 + ((h + i) % t.cap) * es, t.el)); return o; }
    case 'rec': { const o = into && typeof into === 'object' && !Array.isArray(into) ? into : {}; let off = 0; for (const [k, ft] of Object.entries(t.f)) { o[k] = rnRead(A, at + off, ft, o[k]); off += tsize(ft); } return o; }
    case 'opt': return A[at] ? rnRead(A, at + 1, t.t, into) : null;
  }
  throw new Error('internal: read ' + t.k);
}
// A formula's memory: the JavaScript object the simulator keeps (so its code can still read and adjust it)
// and the arena, kept in step. Ring lists live only in the arena; `fresh` says the object is new (reset).
function rnStateIn(A, f, st, fresh) {
  for (const s of f.state) {
    if (s.ring) { if (fresh) A[s.flag] = 0; continue; }
    const v = st[s.name];
    if (v == null || !s.t) { A[s.flag] = 0; continue; }
    A[s.flag] = 1; rnWrite(A, s.addr, s.t, v, 'st.' + s.name);
  }
}
function rnStateOut(A, f, st) {
  for (const s of f.state) {
    if (s.ring || !s.t) continue;
    if (A[s.flag]) st[s.name] = rnRead(A, s.addr, s.t, st[s.name]); else delete st[s.name];
  }
}

/* ───────── the program image: what the companion computer sends to the drone ───────── */
// Little-endian 32-bit words: magic, version, arena size, constants' size, code length, formulas, self-tests;
// the constants (floats); the code; per formula its name (32 bytes), entry, end, step limit, its inputs
// (address, size), its result (address, size) and its memory fields (flag, address, size, name in 16 bytes);
// then the self-tests; a CRC-32 of everything before it.
// A self-test is (formula, input regions, output regions), a region being (address, length, floats…) and each
// list ending with (0, 0): the drone loads the inputs, runs the formula and must get the outputs.
const rnFieldSize = t => t ? tsize(t) : 0;
function rnImage(P, opts = {}) {
  const keys = opts.keys || Object.keys(P.fns), tests = opts.tests || [];
  const w = [RN_MAGIC, RN_VERSION, P.arenaSize, P.constEnd, P.code.length, keys.length, tests.length];
  const words = [], floatAt = new Set(), bytes = [];      // bytes: [word index, Uint8Array of 32 or 16 bytes]
  const push = v => words.push(v | 0), pushF = v => { floatAt.add(words.length); words.push(v); };
  const name = (str, n) => { const b = new Uint8Array(n); b.set(new TextEncoder().encode(str).slice(0, n - 1)); bytes.push([words.length, b]); for (let i = 0; i < n / 4; i++) words.push(0); };
  w.forEach(push);
  for (let i = 0; i < P.constEnd; i++) pushF(P.constData[i]);
  for (let i = 0; i < P.code.length; i++) push(P.code[i]);
  for (const k of keys) {
    const f = P.fns[k];
    name(k, 32); push(f.entry); push(f.end); push(f.maxSteps);
    push(f.args.length); for (const a of f.args) { push(a.state ? -1 : a.addr); push(a.state ? 0 : tsize(a.t)); }
    push(f.ret.addr); push(tsize(f.ret.t));
    push(f.state.length); for (const st of f.state) { push(st.flag); push(st.t ? st.addr : -1); push(rnFieldSize(st.t)); name(st.name, 16); }
  }
  for (const t of tests) {
    push(keys.indexOf(t.key));
    for (const part of [t.ins, t.outs]) { for (const [a, vals] of part) { push(a); push(vals.length); for (const v of vals) pushF(v); } push(0); push(0); }
  }
  const buf = new ArrayBuffer((words.length + 1) * 4), dv = new DataView(buf);
  words.forEach((v, i) => floatAt.has(i) ? dv.setFloat32(i * 4, v, true) : dv.setInt32(i * 4, v, true));
  for (const [i, b] of bytes) new Uint8Array(buf, i * 4, b.length).set(b);
  dv.setUint32(words.length * 4, rnCrc32(new Uint8Array(buf, 0, words.length * 4)), true);
  return new Uint8Array(buf);
}
// Self-tests for a program: each sample { key, args } (the formula's arguments as the simulator passes them,
// memory objects included, and for ring fields the ring's contents under the field's name) is run on this
// program with the JavaScript runner, which gives the expected outputs.
// maxFloats: leave out tests bigger than this (a program sent over a serial link to a small board).
function rnMakeTests(P, samples, maxFloats = Infinity) {
  const A = rnArena(P), tests = [];
  const snap = (a, n) => Array.from(A.subarray(a, a + n));
  for (const s of samples) {
    const f = P.fns[s.key]; if (!f) continue;
    A.fill(0, P.constEnd);
    try {
      f.args.forEach((a, i) => {
        if (a.state) { rnStateIn(A, f, s.args[i] || {}, true); for (const st of f.state) if (st.ring && s.args[i] && s.args[i][st.name]) { A[st.flag] = 1; rnWrite(A, st.addr, st.t, s.args[i][st.name], st.name); } }
        else rnWrite(A, a.addr, a.t, s.args[i], a.name);
      });
    } catch (e) { continue; }                             // the sample doesn't fit this program's inputs
    const ins = [];
    for (const a of f.args) if (!a.state) ins.push([a.addr, snap(a.addr, tsize(a.t))]);
    for (const st of f.state) { ins.push([st.flag, snap(st.flag, 1)]); if (st.t) ins.push([st.addr, snap(st.addr, tsize(st.t))]); }
    try { rnRun(P, A, s.key); } catch (e) { continue; }
    const outs = [[f.ret.addr, snap(f.ret.addr, tsize(f.ret.t))]];
    for (const st of f.state) { outs.push([st.flag, snap(st.flag, 1)]); if (st.t) outs.push([st.addr, snap(st.addr, tsize(st.t))]); }
    if (ins.concat(outs).reduce((n, r) => n + 2 + r[1].length, 0) > maxFloats) continue;
    tests.push({ key: s.key, ins, outs });
  }
  return tests;
}
// Carry formula memory from one arena to another by formula and field name (as rn_transfer does on the drone).
function rnTransfer(Pd, Ad, Ps, As) {
  let n = 0;
  for (const [k, f] of Object.entries(Pd.fns)) {
    const g = Ps.fns[k]; if (!g) continue;
    for (const st of f.state) {
      Ad[st.flag] = 0;
      const o = g.state.find(x => x.name === st.name && rnFieldSize(x.t) === rnFieldSize(st.t));
      if (!o) continue;
      Ad[st.flag] = As[o.flag]; if (st.t) Ad.set(As.subarray(o.addr, o.addr + tsize(st.t)), st.addr); n++;
    }
  }
  return n;
}
let RN_CRC_T = null;
function rnCrc32(b) {
  if (!RN_CRC_T) { RN_CRC_T = new Uint32Array(256); for (let n = 0; n < 256; n++) { let c = n; for (let k = 0; k < 8; k++) c = c & 1 ? 0xEDB88320 ^ (c >>> 1) : c >>> 1; RN_CRC_T[n] = c >>> 0; } }
  let c = 0xFFFFFFFF; for (let i = 0; i < b.length; i++) c = RN_CRC_T[(c ^ b[i]) & 255] ^ (c >>> 8); return (c ^ 0xFFFFFFFF) >>> 0;
}

/* ───────── the C runner, built to WebAssembly ───────── */
const RN_WASM_ERR = { 12: 'bad self-test section', 13: 'a self-test gave a different result', 1: 'not a step program', 2: 'made for another runner version', 3: 'truncated', 4: 'checksum mismatch', 5: 'too big for the runner', 6: 'unknown step', 7: 'address outside the arena', 8: 'writes a constant', 9: 'jump outside the formula', 10: 'block outside the arena', 11: 'bad formula table',
  32: 'took more steps than its limit', 33: 'index outside the list', 34: 'computed address outside the arena', 35: 'list full', 36: 'list length beyond its capacity', 37: 'stopped by the formula', 38: 'unknown step', 39: 'no such formula', 40: 'kernel limits' };
class RnWasm {
  static async create(bytes) {
    const env = { sin: Math.sin, cos: Math.cos, tan: Math.tan, asin: Math.asin, acos: Math.acos, atan: Math.atan, atan2: Math.atan2, exp: Math.exp, log: Math.log, pow: Math.pow };
    const { instance } = await WebAssembly.instantiate(bytes, { env });
    return new RnWasm(instance);
  }
  // Another runner with its own memory, from a compiled module.
  static async fromModule(m) { return new RnWasm(await WebAssembly.instantiate(m, { env: RnWasm.env() })); }
  // (app_call: a board's apps, apps.js; an instance that runs none says so to any)
  static env() { return { sin: Math.sin, cos: Math.cos, tan: Math.tan, asin: Math.asin, acos: Math.acos, atan: Math.atan, atan2: Math.atan2, exp: Math.exp, log: Math.log, pow: Math.pow, app_call: () => -1 }; }
  constructor(inst) { this.x = inst.exports; this.P = null; this.index = {}; }
  // Load a compiled program. Returns null or the loader's reason for rejecting it.
  // With tests, the runner's self-tests run too (tol: largest relative difference); this.selfWorst holds the
  // worst difference found.
  load(P, tests, tol = 1e-2) {
    const img = rnImage(P, { tests }), keys = Object.keys(P.fns);
    if (img.length > this.x.img_cap()) return 'program too big';
    new Uint8Array(this.x.memory.buffer, this.x.img_ptr(), img.length).set(img);
    let e = this.x.load(img.length);
    if (e) return RN_WASM_ERR[e] || 'error ' + e;
    this.selfWorst = 0; this.selfTests = tests ? tests.length : 0;
    if (tests && tests.length) {
      e = this.x.selftest(img.length, tol); this.selfWorst = this.x.selftest_worst();
      if (e) { const t = tests[this.x.trap_pc()]; return `self-test ${this.x.trap_pc() + 1} of ${tests.length}${t ? ' (' + t.key + ')' : ''}: ${RN_WASM_ERR[e] || 'error ' + e}${e === 13 ? ` (off by ${(this.selfWorst * 100).toPrecision(2)}%)` : ''}`; }
    }
    this.P = P; this.index = {}; keys.forEach((k, i) => { this.index[k] = i; });
    this.A = new Float32Array(this.x.memory.buffer, this.x.arena_ptr(), P.arenaSize);
    return null;
  }
  // Run one formula on the arena as it is. Returns the step count; throws RnTrap on a run-time error.
  work() { return this.x.work(); }
  run(key) {
    const e = this.x.run(this.index[key]);
    if (e) throw new RnTrap(`${key}: ${RN_WASM_ERR[e] || 'error ' + e} (step ${this.x.trap_pc()})`, this.x.trap_pc());
    return this.x.steps();
  }
}

if (typeof module !== 'undefined') module.exports = { rnImage, rnMakeTests, rnTransfer, rnFieldSize, rnCrc32, RnWasm, rnRun, rnVerify, rnArena, rnWrite, rnRead, rnStateIn, rnStateOut, RnTrap };
