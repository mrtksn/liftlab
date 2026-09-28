'use strict';
// The step compiler: turns a formula written in the JavaScript subset (rn-parse.js) into a list of steps for the
// runner (rn-ops.js). Everything a step touches gets a fixed place in the arena when the program is linked, so
// running it needs no names, no types and no memory allocation.
//
// How values are kept:
//   num        one float                         arr(el, n)  n elements, fixed
//   list(el, c) [len, up to c elements]          ring(el, c)  [len, head, c elements] (push and shift)
//   rec{…}     its fields one after another       opt(t)     [present, t]
//   enum(…)    one float: which of the names      state{…}   a formula's memory between calls: every field is
//                                                            optional (formulas test `if (!st.q)`), and its type
//                                                            comes from the first assignment
// Lists have a capacity: set by the formula's signature for inputs, worked out from the code (an upper bound on
// a length) for lists it builds, or RN_LIST_CAP for a list built with push.
//
// Values while compiling: { t, s, off } static (the slot s, off floats in), { t, reg, off } dynamic (the address
// is in the scalar slot reg, set by an IDX step), { t, c } a constant, plus a few compile-time kinds: null, a
// string, a function (inlined at each call), a built-in helper.

const RN_LIST_CAP = 48;            // capacity of a list built with push when nothing says otherwise
const RN_UNROLL = 16;              // loops over fixed arrays up to this long are unrolled

const RT = {
  num: { k: 'num' },
  arr: (el, n) => ({ k: 'arr', el, n }),
  list: (el, cap) => ({ k: 'list', el, cap }),
  ring: (el, cap) => ({ k: 'ring', el, cap }),
  rec: fields => ({ k: 'rec', f: fields }),
  opt: t => ({ k: 'opt', t }),
  enm: (...vals) => ({ k: 'enum', vals }),
  state: (decl = {}) => ({ k: 'state', decl }),
};
const tsize = t => {
  if (!t) throw new Error('internal: size of an unknown type');
  switch (t.k) {
    case 'num': case 'enum': return 1;
    case 'arr': return t.n * tsize(t.el);
    case 'list': return 1 + t.cap * (t.el ? tsize(t.el) : 1);
    case 'ring': return 2 + t.cap * tsize(t.el);
    case 'rec': return Object.values(t.f).reduce((s, f) => s + tsize(f), 0);
    case 'opt': return 1 + tsize(t.t);
  }
  throw new Error('internal: size of ' + t.k);
};
const tstr = t => !t ? '?' : t.k === 'num' ? 'number' : t.k === 'enum' ? t.vals.map(v => `'${v}'`).join('|') : t.k === 'arr' ? `${tstr(t.el)}[${t.n}]` :
  t.k === 'list' ? `${tstr(t.el)}[≤${t.cap}]` : t.k === 'ring' ? `${tstr(t.el)}[ring ≤${t.cap}]` : t.k === 'opt' ? `${tstr(t.t)} or null` :
  t.k === 'rec' ? `{ ${Object.entries(t.f).map(([k, v]) => k + ': ' + tstr(v)).join(', ')} }` : t.k;
const fieldOff = (t, name) => { let o = 0; for (const [k, f] of Object.entries(t.f)) { if (k === name) return o; o += tsize(f); } return -1; };
function teq(a, b) {
  if (a === b) return true; if (!a || !b || a.k !== b.k) return false;
  switch (a.k) {
    case 'num': return true;
    case 'enum': return a.vals.join('|') === b.vals.join('|');
    case 'arr': return a.n === b.n && teq(a.el, b.el);
    case 'list': case 'ring': return a.cap === b.cap && teq(a.el, b.el);
    case 'opt': return teq(a.t, b.t);
    case 'rec': { const ka = Object.keys(a.f), kb = Object.keys(b.f); return ka.length === kb.length && ka.every((k, i) => k === kb[i] && teq(a.f[k], b.f[k])); }
  }
  return false;
}
// A type both values fit in (conditional arms, reassignments).
function tunify(a, b) {
  if (!a) return b; if (!b) return a;
  if (a.k === 'null') return b.k === 'opt' ? b : RT.opt(b);
  if (b.k === 'null') return a.k === 'opt' ? a : RT.opt(a);
  if (a.k === 'opt' || b.k === 'opt') { const u = tunify(a.k === 'opt' ? a.t : a, b.k === 'opt' ? b.t : b); return u && RT.opt(u); }
  if (a.k === 'num' && b.k === 'num') return a;
  if (a.k === 'enum' && b.k === 'enum' && teq(a, b)) return a;
  const seq = t => t.k === 'arr' || t.k === 'list';
  if (seq(a) && seq(b)) {
    const el = a.el && b.el ? tunify(a.el, b.el) : (a.el || b.el); if ((a.el && b.el) && !el) return null;
    if (a.k === 'arr' && b.k === 'arr' && a.n === b.n) return RT.arr(el, a.n);
    return RT.list(el, Math.max(a.k === 'arr' ? a.n : a.cap, b.k === 'arr' ? b.n : b.cap));
  }
  if (a.k === 'rec' && b.k === 'rec') {
    const f = {}; const keys = [...new Set([...Object.keys(a.f), ...Object.keys(b.f)])];
    for (const k of keys) {
      const x = a.f[k], y = b.f[k];
      const u = x && y ? tunify(x, y) : RT.opt((x || y).k === 'opt' ? (x || y).t : (x || y));
      if (!u) return null; f[k] = u;
    }
    return RT.rec(f);
  }
  return null;
}

class RnCompileError extends Error { constructor(msg, node, src) { super(src && node && node.pos != null ? `${msg} (line ${src.slice(0, node.pos).split('\n').length})` : msg); } }

/* ───────── slots, program ───────── */
let RN_SLOT_ID = 0;
class Slot {
  constructor(kind, size, name) { this.id = ++RN_SLOT_ID; this.kind = kind; this._size = size; this.name = name; this.first = Infinity; this.last = -1; this.addr = null; }
  get size() { return typeof this._size === 'function' ? this._size() : this._size; }
}

class RnProgram {
  constructor() { this.consts = new Map(); this.constBlocks = []; this.fns = new Map(); this.fixed = []; }
  constSlot(v) {
    const f = Math.fround(v), key = Object.is(f, -0) ? '-0' : String(f);
    if (!this.consts.has(key)) { const s = new Slot('const', 1, 'c' + key); s.init = [f]; this.consts.set(key, s); }
    return this.consts.get(key);
  }
  constBlock(vals) {
    const key = vals.map(v => Math.fround(v)).join(',');
    let s = this.constBlocks.find(b => b.key === key);
    if (!s) { s = new Slot('const', vals.length, 'cb'); s.init = vals.map(Math.fround); s.key = key; this.constBlocks.push(s); }
    return s;
  }
}

/* ───────── the compiler for one formula ───────── */
class RnFn {
  constructor(prog, key, src, sig, helpers) {
    this.prog = prog; this.key = key; this.src = src; this.sig = sig; this.helpers = helpers || RN_HELPERS;
    this.code = []; this.slots = []; this.scopes = []; this.loops = []; this.inlines = []; this.labelN = 0; this.depth = 0;
    this.weight = 1;                 // how many times an instruction emitted now can run per call (enclosing loops' bounds)
    this.stateSlots = [];
  }
  err(msg, node) { throw new RnCompileError(msg, node, this.src); }

  // ── slots and emission ──
  slot(kind, size, name) { const s = new Slot(kind, size, name); this.slots.push(s); return s; }
  tmp(t) { const s = this.slot('tmp', () => tsize(t), 'tmp'); return { t, s, off: 0, fresh: true }; }
  reg() { return this.slot('tmp', 1, 'reg'); }
  label() { return { label: ++this.labelN, at: null }; }
  place(l) { l.at = this.code.length; }
  touch(s) { const i = this.code.length; if (i < s.first) s.first = i; if (i > s.last) s.last = i; }
  emit(op, ...args) {
    if (!(op in RN_OP)) throw new Error('internal: unknown op ' + op);
    for (const a of args) if (a && a.s) this.touch(a.s);
    this.code.push({ op, args, w: this.weight, pos: this.curPos });
  }
  cref(v) { return { s: this.prog.constSlot(v), off: 0 }; }
  // An address operand for a static value.
  at(v, extra = 0) { if (v.c !== undefined) return this.cref(v.c); if (!v.s) this.err('internal: not a static value'); return { s: v.s, off: v.off + extra }; }
  // Read a scalar: an operand holding its value.
  sc(v, node) {
    v = this.unopt(v);
    if (v.k === 'fn' || v.k === 'str' || v.k === 'null' || v.k === 'kernel') this.err('Expected a number here', node);
    if (!v.t || (v.t.k !== 'num' && v.t.k !== 'enum')) this.err(`Expected a number here, not ${tstr(v.t)}`, node);
    if (v.c !== undefined) return this.cref(v.c);
    if (v.s) return { s: v.s, off: v.off };
    const t = this.reg(); this.emit('LDI', { s: t, off: 0 }, { s: v.reg, off: 0 }, v.off); return { s: t, off: 0 };
  }
  scV(ref, extra = {}) { return Object.assign({ t: RT.num, s: ref.s, off: ref.off }, extra); }
  unopt(v) { return v.t && v.t.k === 'opt' ? { ...v, t: v.t.t, off: v.off + 1 } : v; }
  // A static copy of a value that may live at a dynamic address.
  stat(v) {
    if (v.s || v.c !== undefined || !v.t) return v;
    if (v.k) return v;
    const out = this.tmp(v.t); this.emit('CPI', this.at(out), { s: v.reg, off: 0 }, v.off, tsize(v.t)); return out;
  }
  storeScalar(dst, src) {
    if (dst.s) { if (!(dst.s === src.s && dst.off === src.off)) this.emit('MOV', this.at(dst), src); }
    else this.emit('STI', { s: dst.reg, off: 0 }, dst.off, src);
  }
  // Copy a value into a destination of known type, converting where the types allow it.
  copyInto(dst, src, node) {
    let dt = dst.t;
    if (!dt) this.err('internal: destination without a type', node);
    if (src.k === 'null' || src.k === 'undef') {
      if (dt.k === 'opt') { this.storeScalar(dst, this.cref(0)); return; }
      if (dt.k === 'list') { this.storeScalar(dst, this.cref(0)); return; }
      if (dt.k === 'ring') { this.emit('RCLR', this.at(dst), tsize(dt.el), dt.cap); return; }
      this.err(`Can't store null in ${tstr(dt)}`, node);
    }
    if (src.k === 'str') {
      if (dt.k !== 'enum') this.err('Strings can only be compared or passed as a mode', node);
      const i = dt.vals.indexOf(src.v); if (i < 0) this.err(`'${src.v}' isn't one of ${tstr(dt)}`, node);
      this.storeScalar(dst, this.cref(i)); return;
    }
    if (src.k) this.err(`Can't store ${src.k === 'fn' ? 'a function' : 'this'} as a value`, node);
    const st = src.t;
    if (dt.k === 'opt') {
      if (st.k === 'opt') {
        if (teq(dt.t, st.t)) return this.copyRaw(dst, src, tsize(dt), node);
        const f = this.flagOf(src), skip = this.label();       // different layouts: the flag, then the value if there
        this.storeScalar(dst, f); this.emit('JZ', f, skip);
        this.copyInto({ ...dst, t: dt.t, off: dst.off + 1 }, this.unopt(src), node); this.place(skip); return;
      }
      this.storeScalar(dst, this.cref(1)); return this.copyInto({ ...dst, t: dt.t, off: dst.off + 1 }, src, node);
    }
    if (st.k === 'opt') return this.copyInto(dst, this.unopt(src), node);
    if (dt.k === 'num' || dt.k === 'enum') {
      if (st.k !== 'num' && st.k !== 'enum') this.err(`Expected a number, got ${tstr(st)}`, node);
      return this.storeScalar(dst, this.sc(src, node));
    }
    if (dt.k === 'arr') {
      if (st.k === 'arr') {
        if (st.n < dt.n) this.err(`Expected ${dt.n} elements, got ${st.n}`, node);
        if (teq(st.el, dt.el) && st.n === dt.n) return this.copyRaw(dst, src, tsize(dt), node);
        const ss = tsize(st.el), ds = tsize(dt.el);
        for (let i = 0; i < dt.n; i++) this.copyInto({ ...dst, t: dt.el, off: dst.off + i * ds }, { ...src, t: st.el, off: src.off + i * ss }, node);
        return;
      }
      if (st.k === 'list') {           // the first n elements
        const ss = tsize(st.el), ds = tsize(dt.el);
        for (let i = 0; i < dt.n; i++) this.copyInto({ ...dst, t: dt.el, off: dst.off + i * ds }, this.index(src, { t: RT.num, c: i }, node), node);
        return;
      }
    }
    if (dt.k === 'list') {
      if (!dt.el) dt.el = st.el || null;
      if (st.k === 'list') {
        if (!st.el && !dt.el) { this.storeScalar(dst, this.cref(0)); return; }
        if (!st.el) { this.storeScalar(dst, this.cref(0)); return; }
        if (!teq(st.el, dt.el)) {
          if (tunify(st.el, dt.el) && tsize(st.el) === tsize(dt.el) && st.el.k === dt.el.k) { /* same layout */ }
          else if (st.el.k === 'list' && dt.el.k === 'list') return this.loopCopyList(dst, src, node);
          else this.err(`Expected a list of ${tstr(dt.el)}, got ${tstr(st.el)}`, node);
        }
        const sv = this.stat(src);
        if (!dst.s) { const tmp = this.tmp(dt); this.emit('CPYL', this.at(tmp), this.at(sv), tsize(dt.el), dt.cap); this.emit('CPO', { s: dst.reg, off: 0 }, dst.off, this.at(tmp), tsize(dt)); return; }
        if (sv.s === dst.s && sv.off === dst.off) return;
        this.emit('CPYL', this.at(dst), this.at(sv), tsize(dt.el), Math.min(dt.cap, sv.t.cap)); return;
      }
      if (st.k === 'arr') {
        if (st.n > dt.cap) this.err(`${st.n} elements don't fit in a list of at most ${dt.cap}`, node);
        this.storeScalar({ ...dst, t: RT.num }, this.cref(st.n));
        const ss = tsize(st.el), ds = tsize(dt.el);
        if (teq(st.el, dt.el)) return this.copyRaw({ ...dst, off: dst.off + 1 }, src, st.n * ss, node);
        for (let i = 0; i < st.n; i++) this.copyInto({ ...dst, t: dt.el, off: dst.off + 1 + i * ds }, { ...src, t: st.el, off: src.off + i * ss }, node);
        return;
      }
    }
    if (dt.k === 'ring' && st.k === 'ring' && teq(dt, st)) return this.copyRaw(dst, src, tsize(dt), node);
    if (dt.k === 'ring' && st.k === 'list' && !st.el) { this.emit('RCLR', this.at(dst), tsize(dt.el), dt.cap); return; }
    if (dt.k === 'rec' && st.k === 'rec') {
      for (const [name, ft] of Object.entries(dt.f)) {
        const d = { ...dst, t: ft, off: dst.off + fieldOff(dt, name) };
        if (!(name in st.f)) { if (ft.k === 'opt') { this.storeScalar(d, this.cref(0)); continue; } this.err(`Missing field "${name}"`, node); }
        this.copyInto(d, { ...src, t: st.f[name], off: src.off + fieldOff(st, name) }, node);
      }
      return;
    }
    this.err(`Expected ${tstr(dt)}, got ${tstr(st)}`, node);
  }
  copyRaw(dst, src, n, node) {
    if (n === 0) return;
    if (dst.s && (src.s || src.c !== undefined)) {
      if (src.c !== undefined) { this.emit('MOV', this.at(dst), this.cref(src.c)); return; }
      if (dst.s === src.s && dst.off === src.off) return;
      if (n === 1) this.emit('MOV', this.at(dst), this.at(src)); else this.emit('CPY', this.at(dst), this.at(src), n);
    } else if (dst.s) this.emit('CPI', this.at(dst), { s: src.reg, off: 0 }, src.off, n);
    else if (src.s) this.emit('CPO', { s: dst.reg, off: 0 }, dst.off, this.at(src), n);
    else { const t = this.stat(src); this.emit('CPO', { s: dst.reg, off: 0 }, dst.off, this.at(t), n); }
  }
  // Copy a list of lists whose inner capacities differ, element by element.
  loopCopyList(dst, src, node) {
    const n = this.lenOf(src, node);
    this.setLen(dst, n, node);
    this.runLoop(n, dst.t.cap, i => this.copyInto(this.index(dst, i, node, true), this.index(src, i, node), node));
  }

  // ── scopes ──
  push() { this.scopes.push(new Map()); }
  pop() {
    const sc = this.scopes.pop(), end = this.code.length;
    for (const b of sc.values()) for (const s of b.pins || []) { if (s.last < end) s.last = end; }
  }
  bind(name, b, node) { const sc = this.scopes[this.scopes.length - 1]; if (sc.has(name) && !b.param) this.err(`"${name}" is declared twice`, node); sc.set(name, b); return b; }
  lookup(name) { for (let i = this.scopes.length - 1; i >= 0; i--) if (this.scopes[i].has(name)) return this.scopes[i].get(name); return null; }
  pinValue(v, b) {
    b.pins = b.pins || [];
    if (v.s && v.s.kind === 'tmp') { v.s.kind = 'var'; b.pins.push(v.s); this.touch(v.s); }
    if (v.reg) { b.pins.push(v.reg); this.touch(v.reg); }
    if (v.s && v.s.kind === 'var') b.pins.push(v.s);
  }

  // ── loops ──
  // A loop the runner will run: count is an operand with how many times, max its upper bound.
  runLoop(count, max, body, opts = {}) {
    if (!(max >= 0)) this.err('internal: loop without a bound');
    const i = this.slot('var', 1, 'i'), top = this.label(), cont = this.label(), end = this.label(), c = this.reg();
    this.emit('MOV', { s: i, off: 0 }, this.cref(opts.from || 0));
    const start = this.code.length; this.place(top);
    const w0 = this.weight; this.weight = w0 * Math.max(1, max);
    this.emit('LT', { s: c, off: 0 }, { s: i, off: 0 }, count); this.emit('JZ', { s: c, off: 0 }, end);
    this.loops.push({ brk: end, cont, start });
    body({ t: RT.num, s: i, off: 0, lb: 0, ub: max - 1 });
    this.loops.pop();
    this.place(cont); this.emit('ADD', { s: i, off: 0 }, { s: i, off: 0 }, this.cref(1)); this.emit('JMP', top);
    this.weight = w0; this.place(end);
    this.closeLoop(start);
  }
  // Anything first used before a loop and used inside it must survive every pass.
  closeLoop(start) { const end = this.code.length; for (const s of this.slots) if (s.first < start && s.last >= start && s.last < end) s.last = end; }

  // ── values ──
  lenOf(v, node) {
    v = this.unopt(v);
    if (!v.t) this.err('Expected a list', node);
    if (v.t.k === 'arr') return this.cref(v.t.n);
    if (v.t.k === 'list' || v.t.k === 'ring') return this.sc({ ...v, t: RT.num }, node);
    this.err(`${tstr(v.t)} has no length`, node);
  }
  lenV(v, node) {
    v = this.unopt(v);
    if (v.t.k === 'arr') return { t: RT.num, c: v.t.n, lb: v.t.n, ub: v.t.n };
    const r = this.lenOf(v, node); return this.scV(r, { lb: 0, ub: v.t.cap });
  }
  setLen(v, nRef, node) { this.emit('LLEN', this.at(this.stat(v)), nRef, v.t.cap, () => tsize(v.t.el || RT.num)); if (!v.s) this.err('internal: length of a dynamic list', node); }
  // Element i of an array or list, as a value (static when the index and the array are, else dynamic).
  index(v, iv, node, forWrite) {
    v = this.unopt(v);
    const t = v.t; if (!t) this.err('Can\'t index this', node);
    if (t.k !== 'arr' && t.k !== 'list' && t.k !== 'ring') this.err(`Can't index ${tstr(t)}`, node);
    if (!t.el) { if (forWrite) t.el = RT.num; else this.err('This list is empty here, so its elements have no type yet', node); }
    const es = tsize(t.el);
    if (t.k === 'arr' && iv.c !== undefined) {
      if (!(iv.c >= 0 && iv.c < t.n && Number.isInteger(iv.c))) this.err(`Index ${iv.c} is outside ${tstr(t)}`, node);
      if (v.cvals) return { t: RT.num, c: v.cvals[iv.c] };
      return { ...v, t: t.el, off: v.off + iv.c * es, cvals: undefined, fresh: false };
    }
    const r = this.reg(), ir = this.sc(iv, node);
    if (t.k === 'ring') { this.emit('RIDX', { s: r, off: 0 }, this.at(this.stat(v)), ir, es, t.cap); return { t: t.el, reg: r, off: 0 }; }
    const base = t.k === 'arr' ? 0 : 1;
    const bound = t.k === 'arr' ? this.cref(t.n) : null;
    if (v.s || v.c !== undefined) {
      this.emit('IDX', { s: r, off: 0 }, this.at(v, base), ir, bound || this.at(v), es);
    } else {
      let b = bound; if (!b) { const lt = this.reg(); this.emit('LDI', { s: lt, off: 0 }, { s: v.reg, off: 0 }, v.off); b = { s: lt, off: 0 }; }
      this.emit('IDXI', { s: r, off: 0 }, { s: v.reg, off: 0 }, v.off + base, ir, b, es);
    }
    return { t: t.el, reg: r, off: 0 };
  }
  field(v, name, node) {
    v = this.unopt(v);
    if (v.t && v.t.k === 'state') return this.stateField(v, name, node, false);
    if (!v.t || v.t.k !== 'rec') this.err(`"${name}": not a record (${tstr(v.t)})`, node);
    if (!(name in v.t.f)) this.err(`No field "${name}" in ${tstr(v.t)}`, node);
    return { ...v, t: v.t.f[name], off: v.off + fieldOff(v.t, name), fresh: false, cvals: undefined };
  }
  stateField(v, name, node, define) {
    const st = v.t; st.fields = st.fields || new Map();
    let f = st.fields.get(name);
    if (!f) {
      f = { name, t: st.decl[name] || null, flag: this.slot('state', 1, 'st.' + name + '?'), s: null, ring: st.decl[name] && st.decl[name].k === 'ring' };
      if (f.t) { f.s = this.slot('state', tsize(f.t), 'st.' + name); }
      st.fields.set(name, f); this.stateSlots.push(f);
    }
    return { t: f.t, s: f.s, off: 0, stField: f };
  }
  defineState(f, t, node) {
    if (f.t) return;
    if (t.k === 'list' && !t.el) this.err(`st.${f.name}: an empty list needs a type in the formula's signature`, node);
    f.t = t.k === 'arr' || t.k === 'list' ? JSON.parse(JSON.stringify(t)) : t;
    const ft = f.t; f.s = this.slot('state', () => tsize(ft), 'st.' + f.name);
  }

  // ── truth ──
  // Emit a jump to `to` when the condition is `when` (true or false).
  cond(node, to, when) {
    if (node.type === 'Logical' && node.op !== '??') {
      if ((node.op === '&&') === !when) {             // (a && b) false → a false or b false; (a || b) true → either true
        this.cond(node.left, to, when); this.cond(node.right, to, when);
      } else {
        const skip = this.label();
        this.cond(node.left, skip, !when); this.cond(node.right, to, when); this.place(skip);
      }
      return;
    }
    if (node.type === 'Unary' && node.op === '!') return this.cond(node.arg, to, !when);
    const sf = this.stateFlag(node);
    if (sf) return this.jumpOn(sf, to, when);
    if (node.type === 'Binary' && ['===', '==', '!==', '!='].includes(node.op)) {
      const nl = node.right.type === 'Null' ? node.left : node.left.type === 'Null' ? node.right : null;
      if (nl) { const pr = this.presence(nl); const eq = node.op === '===' || node.op === '=='; return this.jumpOn(pr, to, eq ? !when : when); }
    }
    const v = this.expr(node);
    return this.jumpOn(this.truth(v, node), to, when);
  }
  jumpOn(p, to, when) {
    if (p === true || p === false) { if (p === when) this.emit('JMP', to); return; }
    this.emit(when ? 'JNZ' : 'JZ', p, to);
  }
  // `st.x` on the formula's memory: its present flag.
  stateFlag(node) {
    if (node.type === 'Member' && !node.computed) { const o = this.peekState(node.obj); if (o) return { s: this.stateField(o, node.prop, node).stField.flag, off: 0 }; }
    return null;
  }
  // Whether something is there (for `x == null`): an operand holding the flag, or true / false when known.
  presence(node) {
    const sf = this.stateFlag(node); if (sf) return sf;
    if (node.type === 'Null') return false;
    const v = this.expr(node);
    if (v.k === 'null' || v.k === 'undef') return false;
    if (v.t && v.t.k === 'opt') return this.flagOf(v);
    return true;
  }
  peekState(node) { if (node.type !== 'Id') return null; const b = this.lookup(node.name); return b && b.v && b.v.t && b.v.t.k === 'state' ? b.v : null; }
  flagOf(v) { if (v.s) return { s: v.s, off: v.off }; return this.sc({ ...v, t: RT.num }); }
  truth(v, node) {
    if (v.k === 'null' || v.k === 'undef') return false;
    if (v.k === 'str') return v.v !== '';
    if (v.k === 'fn' || v.k === 'kernel') return true;
    if (v.c !== undefined) return v.c !== 0 && !Number.isNaN(v.c);
    if (v.t.k === 'opt') {
      if (v.t.t.k !== 'num' && v.t.t.k !== 'enum') return this.flagOf(v);
      const t = this.reg(); this.emit('TRUTH', { s: t, off: 0 }, this.sc(this.unopt(v), node)); this.emit('MUL', { s: t, off: 0 }, { s: t, off: 0 }, this.flagOf(v));
      return { s: t, off: 0 };
    }
    if (v.t.k === 'num' || v.t.k === 'enum') return this.sc(v, node);
    return true;                                       // arrays and records are always truthy
  }

  // ── expressions ──
  expr(node) {
    this.depth++; if (this.depth > 200) this.err('Too deeply nested', node);
    try { return this.expr_(node); } finally { this.depth--; }
  }
  expr_(node) {
    switch (node.type) {
      case 'Num': return { t: RT.num, c: node.value, lb: node.value, ub: node.value };
      case 'Str': return { k: 'str', v: node.value };
      case 'Null': return { k: 'null' };
      case 'Id': return this.ident(node);
      case 'Member': return this.member(node);
      case 'Call': return this.call(node);
      case 'ArrayLit': return this.arrayLit(node);
      case 'ObjectLit': return this.objectLit(node);
      case 'Binary': return this.binary(node);
      case 'Logical': return this.logical(node);
      case 'Unary': return this.unary(node);
      case 'Cond': return this.conditional(node);
      case 'Assign': return this.assign(node);
      case 'Update': return this.update(node);
      case 'Sequence': { let v; for (const e of node.list) v = this.expr(e); return v; }
      case 'Arrow': case 'Function': return { k: 'fn', node, scopes: this.scopes.slice() };
      case 'NewArray': {
        const n = this.expr(node.n), ub = n.ub;
        if (!(ub >= 0)) this.err('Can\'t tell how long this array can get: its length must come from a list\'s length or a number', node);
        const cap = Math.ceil(ub);
        const out = this.tmp(RT.list(RT.num, Math.max(1, cap))); out.t.el = null;
        this.emit('LLEN', this.at(out), this.sc(n, node), out.t.cap, () => tsize(out.t.el || RT.num));
        return out;
      }
      case 'Spread': this.err('... only works inside [ ]', node);
    }
    this.err(`${node.type} isn't supported in flight code`, node);
  }
  ident(node) {
    const b = this.lookup(node.name);
    if (b) {
      if (b.v) { if (b.v.pending && !b.v.t) this.err(`"${node.name}" is used before it's given a value`, node); return b.v; }
      return b;
    }
    const g = RN_GLOBALS[node.name]; if (g !== undefined) return { t: RT.num, c: g, lb: g, ub: g };
    if (this.helpers[node.name] || RN_KERNELS[node.name]) return { k: 'kernel', name: node.name };
    if (node.name === 'Math' || node.name === 'Array') return { k: 'ns', name: node.name };
    this.err(`Unknown name "${node.name}"`, node);
  }
  member(node) {
    if (!node.computed && node.obj.type === 'Id' && node.obj.name === 'Math' && !this.lookup('Math')) {
      const c = { PI: Math.PI, E: Math.E, SQRT2: Math.SQRT2, LN2: Math.LN2 }[node.prop];
      if (c === undefined) this.err(`Math.${node.prop} isn't a constant`, node); return { t: RT.num, c, lb: c, ub: c };
    }
    const o = this.expr(node.obj);
    if (o.k === 'null') this.err('Reading a field of null', node);
    if (!node.computed) {
      if (node.prop === 'length') { if (o.t && o.t.k === 'state') return this.field(o, 'length', node); return this.lenV(o, node); }
      return this.field(o, node.prop, node);
    }
    const iv = this.expr(node.index);
    if (iv.k === 'str') return this.field(o, iv.v, node);
    return this.index(o, iv, node);
  }
  arrayLit(node) {
    const els = node.elements;
    if (!els.length) { const out = this.tmp(RT.list(null, RN_LIST_CAP)); this.emit('LLEN', this.at(out), this.cref(0), RN_LIST_CAP, () => tsize(out.t.el || RT.num)); out.pendingList = true; return out; }
    const parts = els.map(e => e.type === 'Spread' ? { spread: this.expr(e.arg), node: e } : { v: this.expr(e), node: e });
    if (parts.every(p => !p.spread && p.v.c !== undefined)) {                 // all constants: a read-only block
      const vals = parts.map(p => p.v.c); return { t: RT.arr(RT.num, vals.length), s: this.prog.constBlock(vals), off: 0, cvals: vals, ro: true };
    }
    let el = null, fixed = 0, dyn = false, cap = 0;
    for (const p of parts) {
      if (p.spread) { const t = this.unopt(p.spread).t; if (t.k === 'arr') { fixed += t.n; cap += t.n; } else if (t.k === 'list') { dyn = true; cap += t.cap; } else this.err('Can only spread an array', p.node); el = tunify(el, t.el); }
      else { const t = p.v.k === 'str' ? null : p.v.t; if (!t) this.err('Arrays of strings aren\'t supported', p.node); el = tunify(el, t); fixed++; cap++; }
      if (!el) this.err('Array elements of different kinds', p.node);
    }
    if (el.k === 'opt') this.err('Array elements can\'t be null', node);
    const es = tsize(el);
    if (!dyn) {
      const out = this.tmp(RT.arr(el, fixed)); let k = 0;
      for (const p of parts) {
        if (p.spread) { const s = this.unopt(p.spread); for (let i = 0; i < s.t.n; i++) this.copyInto({ ...out, t: el, off: k++ * es }, { ...s, t: s.t.el, off: s.off + i * tsize(s.t.el), fresh: false, cvals: undefined }, p.node); }
        else this.copyInto({ ...out, t: el, off: k++ * es }, p.v, p.node);
      }
      return out;
    }
    const out = this.tmp(RT.list(el, cap)); this.emit('LLEN', this.at(out), this.cref(0), cap, es);
    for (const p of parts) {
      if (p.spread) this.forEach(p.spread, e => this.pushInto(out, e, p.node), p.node);
      else this.pushInto(out, p.v, p.node);
    }
    return out;
  }
  objectLit(node) {
    const vals = node.props.map(p => ({ key: p.key, v: this.expr(p.value), node: p.value }));
    const f = {};
    for (const { key, v, node: n } of vals) {
      if (v.k === 'null') f[key] = { k: 'null' };
      else if (v.k === 'str') this.err('String fields aren\'t supported', n);
      else if (v.k) this.err('Record fields must be values', n);
      else f[key] = v.t;
    }
    for (const k of Object.keys(f)) if (f[k].k === 'null') this.err(`"${k}: null" needs a type: return it where the formula's signature says what it is`, node);
    const out = this.tmp(RT.rec(f));
    for (const { key, v, node: n } of vals) this.copyInto(this.field(out, key, n), v, n);
    return out;
  }
  // Compile an expression straight into a destination of known type (return values, typed fields). Handles
  // literals with null fields, which have no type of their own.
  into(node, dst) {
    if (node.type === 'ObjectLit' && dst.t.k === 'rec') {
      const seen = new Set();
      for (const p of node.props) {
        if (!(p.key in dst.t.f)) this.err(`No field "${p.key}" in ${tstr(dst.t)}`, p.value);
        seen.add(p.key); this.into(p.value, this.field(dst, p.key, p.value));
      }
      for (const [k, ft] of Object.entries(dst.t.f)) if (!seen.has(k)) { if (ft.k === 'opt') this.storeScalar(this.field(dst, k), this.cref(0)); else this.err(`Missing field "${k}"`, node); }
      return;
    }
    if (node.type === 'ObjectLit' && dst.t.k === 'opt') { this.storeScalar({ ...dst, t: RT.num }, this.cref(1)); return this.into(node, { ...dst, t: dst.t.t, off: dst.off + 1 }); }
    if (node.type === 'Cond') {
      const f = this.label(), end = this.label();
      this.cond(node.test, f, false); this.into(node.cons, dst); this.emit('JMP', end); this.place(f); this.into(node.alt, dst); this.place(end); return;
    }
    if (node.type === 'Null') return this.copyInto(dst, { k: 'null' }, node);
    this.copyInto(dst, this.expr(node), node);
  }
  binary(node) {
    const op = node.op;
    if (['===', '==', '!==', '!='].includes(op)) {
      const neg = op[0] === '!';
      if (node.left.type === 'Null' || node.right.type === 'Null') { const p = this.presence(node.left.type === 'Null' ? node.right : node.left); return this.boolOf(p, !neg); }
      const l = this.expr(node.left), r = this.expr(node.right);
      if (node.left.type === 'Null' || node.right.type === 'Null') { const p = this.presence(node.left.type === 'Null' ? node.right : node.left); return this.boolOf(p, !neg); }
      if (l.k === 'str' || r.k === 'str') {
        const s = l.k === 'str' ? l : r, e = l.k === 'str' ? r : l;
        if (e.k === 'str') return { t: RT.num, c: (e.v === s.v) !== neg ? 1 : 0 };
        if (!e.t || e.t.k !== 'enum') this.err('A string can only be compared with a mode', node);
        const i = e.t.vals.indexOf(s.v);
        if (i < 0) return { t: RT.num, c: neg ? 1 : 0 };
        if (e.c !== undefined) return { t: RT.num, c: (e.c === i) !== neg ? 1 : 0 };
        return this.arith(neg ? 'NE' : 'EQ', e, { t: RT.num, c: i }, node);
      }
      return this.arith(neg ? 'NE' : 'EQ', l, r, node);
    }
    const l = this.expr(node.left), r = this.expr(node.right);
    switch (op) {
      case '+': return this.arith('ADD', l, r, node);
      case '-': return this.arith('SUB', l, r, node);
      case '*': return this.arith('MUL', l, r, node);
      case '/': return this.arith('DIV', l, r, node);
      case '%': return this.arith('MOD', l, r, node);
      case '**': if (r.c === 2) return this.arith('MUL', l, l, node); if (r.c === 0.5) return this.un('SQRT', l, node); return this.arith('POW', l, r, node);
      case '<': return this.arith('LT', l, r, node);
      case '<=': return this.arith('LE', l, r, node);
      case '>': return this.arith('LT', r, l, node);
      case '>=': return this.arith('LE', r, l, node);
    }
    this.err(`Operator ${op} isn't supported`, node);
  }
  boolOf(p, neg) {
    if (p === true || p === false) return { t: RT.num, c: (p !== neg) ? 1 : 0 };
    const o = this.tmp(RT.num); this.emit(neg ? 'NOT' : 'TRUTH', this.at(o), p); return o;
  }
  arith(op, l, r, node) {
    l = this.unopt(l); r = this.unopt(r);
    if (l.c !== undefined && r.c !== undefined) { const c = RN_FOLD[op](l.c, r.c); return { t: RT.num, c: Math.fround(c) === c || !Number.isFinite(c) ? c : c, lb: c, ub: c }; }
    const a = this.sc(l, node.left || node), b = this.sc(r, node.right || node);
    const o = this.tmp(RT.num); this.emit(op, this.at(o), a, b);
    const [ll, lu, rl, ru] = [l.lb, l.ub, r.lb, r.ub];
    if (op === 'ADD' && lu !== undefined && ru !== undefined) { o.ub = lu + ru; o.lb = ll + rl; }
    if (op === 'SUB' && lu !== undefined && rl !== undefined) { o.ub = lu - rl; if (ll !== undefined && ru !== undefined) o.lb = ll - ru; }
    if (op === 'MUL' && ll >= 0 && rl >= 0 && lu !== undefined && ru !== undefined) { o.ub = lu * ru; o.lb = ll * rl; }
    return o;
  }
  un(op, v, node) {
    v = this.unopt(v);
    if (v.c !== undefined) { const c = RN_FOLD[op](v.c); return { t: RT.num, c, lb: c, ub: c }; }
    const o = this.tmp(RT.num); this.emit(op, this.at(o), this.sc(v, node));
    if (op === 'NEG' && v.lb !== undefined && v.ub !== undefined) { o.lb = -v.ub; o.ub = -v.lb; }
    return o;
  }
  unary(node) {
    if (node.op === '!') {
      const sf = this.stateFlag(node.arg); if (sf) return this.boolOf(sf, true);
      const v = this.expr(node.arg); return this.boolOf(this.truth(v, node), true);
    }
    const v = this.expr(node.arg);
    if (node.op === '-') return this.un('NEG', v, node);
    return this.unopt(v);
  }
  logical(node) {
    const op = node.op;
    // list[i] || fallback: JavaScript gives undefined past the end; here, the fallback.
    if (op === '||' && node.left.type === 'Member' && node.left.computed) {
      const lv = this.expr(node.left.obj);
      if (lv.t && (lv.t.k === 'list' || lv.t.k === 'ring')) {
        const iv = this.expr(node.left.index), ir = this.sc(iv, node), len = this.lenOf(lv, node), c = this.reg(), c2 = this.reg();
        const fb = this.label(), end = this.label();
        this.emit('LT', { s: c, off: 0 }, ir, len); this.emit('JZ', { s: c, off: 0 }, fb);
        this.emit('LT', { s: c2, off: 0 }, ir, this.cref(0)); this.emit('JNZ', { s: c2, off: 0 }, fb);
        const e = this.index(lv, iv, node); const out = this.tmp(e.t);
        this.copyInto(out, e, node); this.emit('JMP', end); this.place(fb);
        this.into(node.right, out); this.place(end);
        return out;
      }
    }
    if (op === '??') {
      const l = this.expr(node.left);
      if (l.k === 'null' || l.k === 'undef') return this.expr(node.right);
      if (!l.t || l.t.k !== 'opt') return l;
      const inner = l.t.t, out = this.tmp(inner), none = this.label(), end = this.label();
      this.emit('JZ', this.flagOf(l), none); this.copyInto(out, this.unopt(l), node); this.emit('JMP', end);
      this.place(none); this.into(node.right, out); this.place(end);
      return out;
    }
    const l = this.expr(node.left);
    if (l.c !== undefined) {                            // known now
      const t = l.c !== 0 && !Number.isNaN(l.c);
      if (op === '||') return t ? l : this.expr(node.right);
      return t ? this.expr(node.right) : l;
    }
    if (l.t && (l.t.k === 'num' || l.t.k === 'enum')) {
      // Numbers: JavaScript's `a || b` is a if a is truthy, else b.
      const out = this.tmp(RT.num), end = this.label();
      this.storeScalar(out, this.sc(l, node));
      this.emit(op === '||' ? 'JNZ' : 'JZ', this.at(out), end);
      const r = this.expr(node.right); this.storeScalar(out, this.sc(r, node)); this.place(end);
      if (l.ub !== undefined && r.ub !== undefined) { out.ub = Math.max(l.ub, r.ub); out.lb = Math.min(l.lb ?? -Infinity, r.lb ?? -Infinity); }
      return out;
    }
    if (op === '&&') {
      // a && b with a record, list or something optional: b when a is there (and truthy), else null.
      const none = this.label(), end = this.label();
      this.jumpOn(this.truth(l, node), none, false);
      const r = this.expr(node.right);
      if (!r.k) {
        const out = this.tmp(RT.opt(this.ownType(r.t.k === 'opt' ? r.t.t : r.t)));
        this.copyInto(out, r, node); this.emit('JMP', end); this.place(none); this.storeScalar({ ...out, t: RT.num }, this.cref(0)); this.place(end);
        return out;
      }
      this.code.length = this.code.length;               // falls through to the true/false form below
    }
    if (op === '||' && l.t && l.t.k === 'opt') {
      // An optional value or a fallback.
      const useB = this.label(), end = this.label();
      const out = this.tmp(this.ownType(l.t.t));
      this.jumpOn(this.truth(l, node), useB, false);
      this.copyInto(out, this.unopt(l), node); this.emit('JMP', end); this.place(useB);
      this.into(node.right, out); this.place(end);
      return out;
    }
    // Anything else in a value position is used as true / false.
    const out = this.tmp(RT.num), f = this.label(), end = this.label();
    this.cond(node, f, false); this.storeScalar(out, this.cref(1)); this.emit('JMP', end); this.place(f); this.storeScalar(out, this.cref(0)); this.place(end);
    return out;
  }
  conditional(node) {
    // Known now: pick the arm (so `k === 5 ? … : …` inside an unrolled map costs nothing).
    const tv = this.tryConst(node.test);
    if (tv !== undefined) return this.expr(tv ? node.cons : node.alt);
    const f = this.label(), end = this.label();
    this.cond(node.test, f, false);
    if (node.cons.type === 'Null') {                     // c ? null : x
      this.emit('JMP', end); this.place(f);
      const b = this.expr(node.alt); if (b.k) this.err('Expected a value in the second arm', node.alt);
      const out = this.tmp(RT.opt(this.ownType(b.t))); this.copyInto(out, b, node);
      const skip = this.label(); this.emit('JMP', skip); this.place(end); this.storeScalar({ ...out, t: RT.num }, this.cref(0)); this.place(skip);
      return out;
    }
    const a = this.expr(node.cons);
    if (a.k) this.err('Expected a value in the first arm', node.cons);
    // The result has the first arm's type (made optional when the other arm is null); the second arm is
    // converted into it.
    const out = this.tmp(node.alt.type === 'Null' ? RT.opt(this.ownType(a.t)) : this.ownType(a.t));
    this.copyInto(out, a, node); this.emit('JMP', end); this.place(f);
    this.into(node.alt, out); this.place(end);
    if (a.ub !== undefined) { out.ub = a.ub; out.lb = a.lb; }
    const bu = this.ubOf(node.alt); if (out.ub !== undefined) { if (bu === undefined) { delete out.ub; delete out.lb; } else { out.ub = Math.max(out.ub, bu.ub); out.lb = Math.min(out.lb ?? -Infinity, bu.lb ?? -Infinity); } }
    return out;
  }
  // A type of its own (lists and records are copied so resolving a pending element type stays local).
  ownType(t) { return t && (t.k === 'arr' || t.k === 'list' || t.k === 'rec' || t.k === 'opt') ? JSON.parse(JSON.stringify(t)) : t; }
  // Bounds of a simple expression without compiling it (for lengths: `mot ? 2 * n : n`).
  ubOf(node) {
    const c = this.constOf(node); if (typeof c === 'number') return { lb: c, ub: c };
    if (node.type === 'Id') { const b = this.lookup(node.name); if (b && b.v && b.v.ub !== undefined) return { lb: b.v.lb, ub: b.v.ub }; }
    if (node.type === 'Member' && !node.computed && node.prop === 'length') { try { const saveN = this.code.length, saveS = this.slots.length; const o = this.expr(node.obj); this.code.length = saveN; this.slots.length = saveS; const t = this.unopt(o).t; if (t.k === 'arr') return { lb: t.n, ub: t.n }; if (t.k === 'list') return { lb: 0, ub: t.cap }; } catch (e) { } }
    if (node.type === 'Binary' && ['+', '-', '*'].includes(node.op)) {
      const a = this.ubOf(node.left), b = this.ubOf(node.right); if (!a || !b) return undefined;
      if (node.op === '+') return { lb: a.lb + b.lb, ub: a.ub + b.ub };
      if (node.op === '-') return { lb: a.lb - b.ub, ub: a.ub - b.lb };
      if (a.lb >= 0 && b.lb >= 0) return { lb: a.lb * b.lb, ub: a.ub * b.ub };
    }
    return undefined;
  }
  tryConst(node) {
    if (node.type === 'Num') return node.value;
    if (node.type === 'Binary' && ['===', '==', '!==', '!=', '<', '<=', '>', '>='].includes(node.op)) {
      const a = this.constOf(node.left), b = this.constOf(node.right);
      if (a === undefined || b === undefined) return undefined;
      switch (node.op) { case '===': case '==': return a === b; case '!==': case '!=': return a !== b; case '<': return a < b; case '<=': return a <= b; case '>': return a > b; case '>=': return a >= b; }
    }
    return undefined;
  }
  constOf(node) {
    if (node.type === 'Num') return node.value;
    if (node.type === 'Str') return node.value;
    if (node.type === 'Id') { const b = this.lookup(node.name); if (b && b.v && b.v.c !== undefined) return b.v.c; if (!b && RN_GLOBALS[node.name] !== undefined) return RN_GLOBALS[node.name]; }
    if (node.type === 'Binary' && ['+', '-', '*', '/'].includes(node.op)) { const a = this.constOf(node.left), b = this.constOf(node.right); if (typeof a === 'number' && typeof b === 'number') return RN_FOLD[{ '+': 'ADD', '-': 'SUB', '*': 'MUL', '/': 'DIV' }[node.op]](a, b); }
    return undefined;
  }
  // Where an assignment writes.
  lvalue(node) {
    if (node.type === 'Id') {
      const b = this.lookup(node.name);
      if (!b) this.err(`Unknown name "${node.name}"`, node);
      if (b.kind === 'const') this.err(`"${node.name}" is a const`, node);
      return { binding: b, v: b.v };
    }
    if (node.type === 'Member') {
      if (!node.computed) {
        const so = this.peekState(node.obj);
        if (so) { const f = this.stateField(so, node.prop, node).stField; return { stField: f, v: { t: f.t, s: f.s, off: 0 } }; }
      }
      return { v: this.member(node) };
    }
    this.err('Can\'t assign to this', node);
  }
  assign(node) {
    const L = this.lvalue(node.target);
    if (node.op === '=') {
      if (L.stField) {
        const f = L.stField;
        if (!f.t) {
          if (node.value.type === 'ArrayLit' && !node.value.elements.length) this.err(`st.${f.name} = []: give it a type in the formula's signature`, node);
          const v = this.expr(node.value);
          if (v.k === 'null') { this.storeScalar({ s: f.flag, off: 0 }, this.cref(0)); return v; }
          if (v.k) this.err('Can\'t store this in the formula\'s memory', node);
          this.defineState(f, v.t, node); this.copyInto({ t: f.t, s: f.s, off: 0 }, v, node);
        } else if (node.value.type === 'Null') { this.storeScalar({ s: f.flag, off: 0 }, this.cref(0)); return { k: 'null' }; }
        else this.into(node.value, { t: f.t, s: f.s, off: 0 });
        this.storeScalar({ s: f.flag, off: 0 }, this.cref(1));
        return { t: f.t, s: f.s, off: 0 };
      }
      if (L.binding && L.binding.v.pending && !L.binding.v.t) {      // `let x;` first given a value
        const v = this.expr(node.value); if (v.k) this.err('Can\'t store this in a variable', node);
        L.binding.v.t = v.t.k === 'arr' || v.t.k === 'list' || v.t.k === 'rec' ? JSON.parse(JSON.stringify(v.t)) : v.t;
        const tt = L.binding.v.t; L.binding.v.s._size = () => tsize(tt); delete L.binding.v.pending;
        this.copyInto(L.binding.v, v, node); return L.binding.v;
      }
      let dst = L.v;
      if (dst.ro || (dst.s && dst.s.kind === 'const')) this.err('This array is a constant: copy it first with .slice()', node);
      if (L.binding) { this.touch(dst.s); if (L.binding.noBounds !== false) { delete dst.ub; delete dst.lb; } }
      const v = this.expr(node.value);
      if (L.binding && dst.t && (dst.t.k === 'list') && v.t && (v.t.k === 'list' || v.t.k === 'arr')) {
        const cap = v.t.k === 'arr' ? v.t.n : v.t.cap; if (cap > dst.t.cap) this.err(`This list can be longer (${cap}) than the variable holds (${dst.t.cap})`, node);
      }
      this.copyInto(dst, v, node);
      return dst;
    }
    const opMap = { '+=': 'ADD', '-=': 'SUB', '*=': 'MUL', '/=': 'DIV', '**=': 'POW' };
    let dst = L.v;
    if (L.stField) { if (!L.stField.t) this.err(`st.${L.stField.name} is used before it's set`, node); }
    const r = this.expr(node.value);
    const cur = this.sc(dst, node.target);
    const o = this.reg(); this.emit(opMap[node.op], { s: o, off: 0 }, cur, this.sc(r, node.value));
    this.storeScalar(dst, { s: o, off: 0 });
    if (L.binding) { delete dst.ub; delete dst.lb; }
    return this.scV({ s: o, off: 0 });
  }
  update(node) {
    const L = this.lvalue(node.target);
    const dst = L.v; const cur = this.sc(dst, node.target);
    const old = node.prefix ? null : this.tmp(RT.num);
    if (old) this.emit('MOV', this.at(old), cur);
    const o = this.reg(); this.emit(node.op === '++' ? 'ADD' : 'SUB', { s: o, off: 0 }, cur, this.cref(1));
    this.storeScalar(dst, { s: o, off: 0 });
    if (L.binding) { delete dst.ub; delete dst.lb; }
    return node.prefix ? this.scV({ s: o, off: 0 }) : old;
  }

  // ── calls ──
  call(node) {
    const cal = node.callee;
    if (cal.type === 'Member' && !cal.computed) {
      if (cal.obj.type === 'Id' && cal.obj.name === 'Math' && !this.lookup('Math')) return this.mathCall(cal.prop, node);
      if (cal.obj.type === 'Id' && cal.obj.name === 'Array' && !this.lookup('Array')) this.err(`Array.${cal.prop} isn't supported; use new Array(n).fill(x) or .map`, node);
      const o = this.expr(cal.obj);
      return this.method(o, cal.prop, node);
    }
    const f = this.expr(cal);
    if (f.k === 'fn') return this.inline(f, node.args.map(a => a.type === 'Spread' ? this.err('... isn\'t supported in calls', a) : this.expr(a)), node, node.args);
    if (f.k === 'kernel') return this.kernel(f.name, node);
    this.err('This isn\'t a function', node);
  }
  mathCall(name, node) {
    const args = node.args.map(a => this.expr(a));
    const one = { abs: 'ABS', sqrt: 'SQRT', sin: 'SIN', cos: 'COS', tan: 'TAN', asin: 'ASIN', acos: 'ACOS', atan: 'ATAN', exp: 'EXP', log: 'LOG', floor: 'FLOOR', ceil: 'CEIL', round: 'ROUND', sign: 'SIGN' }[name];
    if (one) { if (args.length !== 1) this.err(`Math.${name} takes one number`, node); const v = this.un(one, args[0], node); if (name === 'abs' && args[0].ub !== undefined) { v.lb = 0; v.ub = Math.max(Math.abs(args[0].ub), Math.abs(args[0].lb ?? 0)); } if (['floor', 'ceil', 'round'].includes(name) && args[0].ub !== undefined) { v.ub = Math.ceil(args[0].ub); v.lb = args[0].lb !== undefined ? Math.floor(args[0].lb) : undefined; } return v; }
    if (name === 'atan2') return this.arith('ATAN2', args[0], args[1], node);
    if (name === 'pow') return this.arith('POW', args[0], args[1], node);
    if (name === 'min' || name === 'max') {
      if (!args.length) this.err(`Math.${name} needs numbers`, node);
      let v = args[0];
      for (let i = 1; i < args.length; i++) {
        const w = args[i], r = this.arith(name === 'min' ? 'MIN' : 'MAX', v, w, node);
        if (v.ub !== undefined && w.ub !== undefined) { r.ub = name === 'min' ? Math.min(v.ub, w.ub) : Math.max(v.ub, w.ub); }
        if (v.lb !== undefined && w.lb !== undefined) { r.lb = name === 'min' ? Math.min(v.lb, w.lb) : Math.max(v.lb, w.lb); }
        else if (name === 'max' && (v.lb !== undefined || w.lb !== undefined)) r.lb = Math.max(v.lb ?? -Infinity, w.lb ?? -Infinity);
        v = r;
      }
      return v;
    }
    if (name === 'hypot') {
      if (!args.length) return { t: RT.num, c: 0 };
      let s = null;
      for (const a of args) { const q = this.arith('MUL', a, a, node); s = s ? this.arith('ADD', s, q, node) : q; }
      return this.un('SQRT', s, node);
    }
    this.err(`Math.${name} isn't supported`, node);
  }
  // Inline a function at its call: its parameters become names for the arguments.
  inline(f, args, node, argNodes) {
    if (this.inlines.length > 12) this.err('Functions nest too deeply (recursion isn\'t supported)', node);
    const fn = f.node, saved = this.scopes; this.scopes = f.scopes.slice(); this.push();
    const assigned = assignedNames(fn.body);
    fn.params.forEach((p, i) => {
      const a = args[i] !== undefined ? args[i] : { k: 'undef' };
      this.bindPattern(p, a, assigned.has(p.name) ? 'let' : 'const', node, true);
    });
    let result;
    if (fn.body.type !== 'Block') result = this.expr(fn.body);
    else {
      const fr = { end: this.label(), out: null, type: null, retNode: null };
      this.inlines.push(fr);
      this.block(fn.body, true);
      this.inlines.pop();
      this.place(fr.end);
      result = fr.out || { k: 'undef' };
    }
    this.pop(); this.scopes = saved;
    return result;
  }
  bindPattern(p, v, kind, node, param) {
    if (p.type === 'Id') return this.bindName(p.name, v, kind, p, param);
    if (p.type === 'Default') return this.bindName(p.name, v.k === 'undef' ? this.expr(p.value) : v, kind, p, param);
    if (p.type === 'ArrayPattern') {
      p.elements.forEach((e, i) => { if (e) this.bindPattern(e, this.index(v, { t: RT.num, c: i }, node), kind, node, param); });
      return;
    }
    this.err('Unsupported pattern', node);
  }
  bindName(name, v, kind, node, param) {
    if (v.k === 'fn' || v.k === 'kernel' || v.k === 'str' || v.k === 'null' || v.k === 'undef') {
      if (kind === 'let' && (v.k === 'null' || v.k === 'undef')) this.err('A variable needs a starting value with a type', node);
      return this.bind(name, { kind: 'const', v, param }, node);
    }
    if (kind === 'const' || (v.fresh && v.s && v.s.kind === 'tmp' && !v.ro)) {
      // const: a name for the value where it is (as in JavaScript, no copy). let from a new value: take it over.
      const b = this.bind(name, { kind, v: kind === 'let' ? { ...v, fresh: false } : v, param }, node);
      this.pinValue(v, b);
      if (kind === 'let') { b.v = { ...v, fresh: false }; }
      return b;
    }
    // let from an existing place: a copy.
    const t = v.t.k === 'arr' || v.t.k === 'list' || v.t.k === 'rec' ? JSON.parse(JSON.stringify(v.t)) : v.t;
    const s = this.slot('var', () => tsize(t), name), nv = { t, s, off: 0 };
    this.copyInto(nv, v, node);
    const b = this.bind(name, { kind, v: nv, param }, node); b.pins = [s]; this.touch(s);
    return b;
  }
  method(o, name, node) {
    const args = node.args;
    const fnArg = i => { const f = this.expr(args[i]); if (f.k !== 'fn') this.err(`.${name} needs a function`, args[i]); return f; };
    if (o.k) this.err(`.${name} on ${o.k}`, node);
    const src = this.unopt(o), t = src.t;
    if (!t) this.err('internal: method on an untyped value', node);
    switch (name) {
      case 'map': {
        const f = fnArg(0);
        if (t.k === 'arr' && t.n <= RN_UNROLL) {
          const vals = []; for (let i = 0; i < t.n; i++) vals.push(this.inline(f, [this.index(src, { t: RT.num, c: i }, node), { t: RT.num, c: i, lb: i, ub: i }], node));
          let el = null; for (const v of vals) { if (v.k) this.err('.map must give numbers or arrays', node); el = tunify(el, v.t); }
          if (vals.every(v => v.c !== undefined)) return { t: RT.arr(RT.num, t.n), s: this.prog.constBlock(vals.map(v => v.c)), off: 0, cvals: vals.map(v => v.c), ro: true };
          const out = this.tmp(RT.arr(el, t.n)); vals.forEach((v, i) => this.copyInto({ ...out, t: el, off: i * tsize(el) }, v, node)); return out;
        }
        if (t.k !== 'list' && t.k !== 'arr') this.err(`.map on ${tstr(t)}`, node);
        const fused = this.vecMap(src, f, node); if (fused) return fused;
        const cap = t.k === 'arr' ? t.n : t.cap, holder = { el: null };
        const out = this.tmp(RT.list(null, cap)); out.t.el = null;
        const n = this.lenOf(src, node);
        this.emit('LLEN', this.at(out), n, cap, () => tsize(out.t.el));
        this.runLoop(n, cap, i => {
          const v = this.inline(f, [this.index(src, i, node), i], node);
          if (v.k) this.err('.map must give numbers or arrays', node);
          if (!out.t.el) out.t.el = v.t.k === 'arr' || v.t.k === 'list' ? JSON.parse(JSON.stringify(v.t)) : v.t;
          this.copyInto(this.index(out, i, node, true), v, node);
        });
        return out;
      }
      case 'forEach': {
        const f = fnArg(0); this.forEach(src, (e, i) => this.inline(f, [e, i], node), node); return { k: 'undef' };
      }
      case 'reduce': {
        const f = fnArg(0);
        if (args.length < 2) this.err('.reduce needs a starting value', node);
        const init = this.expr(args[1]); if (init.k) this.err('.reduce needs a number or array to start from', node);
        const fused = this.vecReduce(src, f, init, node); if (fused) return fused;
        const acc = this.tmp(init.t); this.copyInto(acc, init, node); acc.fresh = false;
        this.forEach(src, (e, i) => { const v = this.inline(f, [acc, e, i], node); this.copyInto(acc, v, node); }, node);
        return acc;
      }
      case 'slice': {
        const a = args[0] ? this.expr(args[0]) : { t: RT.num, c: 0 }, b = args[1] ? this.expr(args[1]) : null;
        if (t.k === 'arr' && a.c !== undefined && (!b || b.c !== undefined)) {
          const from = a.c < 0 ? t.n + a.c : a.c, to = b ? (b.c < 0 ? t.n + b.c : b.c) : t.n, n = Math.max(0, to - from);
          const out = this.tmp(RT.arr(t.el, n)); const es = tsize(t.el);
          if (n) this.copyRaw(out, { ...src, t: RT.arr(t.el, n), off: src.off + from * es }, n * es, node);
          return out;
        }
        if (t.k === 'list' && a.c === 0 && !b) { const out = this.tmp(JSON.parse(JSON.stringify(t))); this.copyInto(out, src, node); return out; }
        { const fast = this.sliceFlat(src, a, b, node); if (fast) return fast; }
        // A run-time slice: element by element.
        const cap = t.k === 'arr' ? t.n : t.cap;
        const out = this.tmp(RT.list(t.el, cap));
        const len = this.lenOf(src, node), from = this.sc(a, node);
        const to = b ? this.arith('MIN', this.scV(this.sc(b, node)), this.scV(len), node) : this.scV(len);
        const n = this.arith('MAX', this.arith('SUB', to, this.scV(from), node), { t: RT.num, c: 0 }, node);
        this.emit('LLEN', this.at(out), this.sc(n), cap, tsize(t.el));
        this.runLoop(this.sc(n), cap, i => {
          const j = this.arith('ADD', i, this.scV(from), node);
          this.copyInto(this.index(out, i, node, true), this.index(src, j, node), node);
        });
        return out;
      }
      case 'concat': {
        const others = args.map(a => this.unopt(this.expr(a)));
        const all = [src, ...others];
        if (all.every(v => v.t.k === 'arr')) {
          let el = null; for (const v of all) el = tunify(el, v.t.el);
          const n = all.reduce((s, v) => s + v.t.n, 0), out = this.tmp(RT.arr(el, n)); let k = 0;
          for (const v of all) for (let i = 0; i < v.t.n; i++) this.copyInto({ ...out, t: el, off: (k++) * tsize(el) }, this.index(v, { t: RT.num, c: i }, node), node);
          return out;
        }
        let el = null, cap = 0; for (const v of all) { if (v.t.el) el = tunify(el, v.t.el); cap += v.t.k === 'arr' ? v.t.n : v.t.cap; }
        if (!el) el = RT.num;
        const out = this.tmp(RT.list(el, cap)); this.emit('LLEN', this.at(out), this.cref(0), cap, tsize(el));
        for (const v of all) {
          if (v.t.k === 'list' && !v.t.el) v.t.el = el;
          this.forEach(v, e => this.pushInto(out, e, node), node);
        }
        return out;
      }
      case 'fill': {
        const v = this.expr(args[0]); if (!v.t || v.t.k !== 'num') this.err('.fill needs a number', node);
        if (t.k === 'list') { if (!t.el) t.el = RT.num; if (t.el.k !== 'num') this.err('.fill on a list of arrays', node); this.emit('LFILL', this.at(src), this.sc(v, node), 1, t.cap); return src; }
        if (t.k === 'arr' && t.el.k === 'num') { this.emit('FILL', this.at(this.stat(src)), this.sc(v, node), t.n); return src; }
        this.err('.fill on this', node);
      }
      case 'push': {
        for (const a of args) this.pushInto(src, this.expr(a), a);
        return { k: 'undef' };
      }
      case 'shift': {
        if (t.k !== 'ring') this.err('.shift only works on a list declared as a ring in the signature', node);
        this.emit('RSHIFT', this.at(src), tsize(t.el), t.cap); return { k: 'undef' };
      }
    }
    this.err(`.${name}() isn't supported in flight code`, node);
  }
  // A run-time slice as one fused copy of the elements' floats: out.length = max(0, min(b, len) − max(0, a)).
  sliceFlat(src, a, b, node) {
    const t = src.t; if (!t.el || (t.k !== 'list' && t.k !== 'arr')) return null;
    const es = tsize(t.el), cap = t.k === 'arr' ? t.n : t.cap, base = t.k === 'list' ? 1 : 0;
    if (!src.s && !src.reg) return null;
    const out = this.tmp(RT.list(this.ownType(t.el), cap));
    const len = this.lenOf(src, node);
    const from = a.c !== undefined ? { t: RT.num, c: Math.max(0, a.c) } : this.arith('MAX', a, { t: RT.num, c: 0 }, node);
    const to = b ? this.arith('MIN', b, this.scV(len), node) : this.scV(len);
    const n = this.arith('MAX', this.arith('SUB', to, from, node), { t: RT.num, c: 0 }, node);
    this.emit('LLEN', this.at(out), this.sc(n, node), cap, es);
    const cnt = es === 1 ? this.sc(n, node) : this.sc(this.arith('MUL', n, { t: RT.num, c: es }, node), node);
    let view;
    if (from.c !== undefined && src.s) view = { s: src.s, off: src.off + base + from.c * es, stride: 1 };
    else {
      const r = this.reg();
      if (src.s) this.emit('AR', { s: r, off: 0 }, { s: src.s, off: src.off + base }); else this.emit('ADD', { s: r, off: 0 }, { s: src.reg, off: 0 }, this.cref(src.off + base));
      if (from.c === undefined || from.c) this.emit('ADD', { s: r, off: 0 }, { s: r, off: 0 }, this.sc(es === 1 ? from : this.arith('MUL', from, { t: RT.num, c: es }, node), node));
      view = { reg: r, off: 0, stride: 1 };
    }
    const capF = (from.c !== undefined && view.s ? Math.max(0, cap - from.c) : cap) * es;   // a static view is checked at load for this many
    this.emit('VS', RN_VKIND.mul, ...this.vecOperands({ s: out.s, off: out.off + 1, stride: 1 }), ...this.vecOperands(view), this.cref(1), cnt, capF);
    return out;
  }
  pushInto(list, v, node) {
    const t = list.t;
    if (v.k) this.err('Can only push numbers, arrays or records', node);
    if (t.k !== 'list' && t.k !== 'ring') this.err(`.push on ${tstr(t)}`, node);
    if (!t.el) t.el = v.t.k === 'arr' || v.t.k === 'list' || v.t.k === 'rec' ? JSON.parse(JSON.stringify(v.t)) : v.t;
    let e = v;
    if (!teq(v.t, t.el)) { e = this.tmp(t.el); this.copyInto(e, v, node); }
    e = this.stat(e);
    if (!list.s) this.err('internal: push onto a list at a dynamic place', node);
    const tt = t;
    this.emit(t.k === 'ring' ? 'RPUSH' : 'PUSH', this.at(list), this.at(e), () => tsize(tt.el), t.cap);
  }
  // Run body(element, index) for every element: unrolled for short fixed arrays, a loop otherwise.
  forEach(v, body, node) {
    v = this.unopt(v); const t = v.t;
    if (t.k === 'arr' && t.n <= RN_UNROLL) { for (let i = 0; i < t.n; i++) body(this.index(v, { t: RT.num, c: i }, node), { t: RT.num, c: i, lb: i, ub: i }); return; }
    if (t.k !== 'arr' && t.k !== 'list' && t.k !== 'ring') this.err(`Can't loop over ${tstr(t)}`, node);
    if (t.k === 'list' && !t.el) return;                 // an empty list
    const n = this.lenOf(v, node), cap = t.k === 'arr' ? t.n : t.cap;
    this.runLoop(n, cap, i => body(this.index(v, i, node), i));
  }
  kernel(name, node) {
    const helper = this.helpers[name];
    if (helper && !RN_KERNELS[name]) return this.inline(this.helperFn(name), node.args.map(a => this.expr(a)), node);
    const K = RN_KERNELS[name];
    if (name === 'clamp') { const [x, a, b] = node.args.map(a => this.expr(a)); if (x.c !== undefined && a.c !== undefined && b.c !== undefined) return { t: RT.num, c: Math.min(Math.max(x.c, a.c), b.c) }; const o = this.tmp(RT.num); this.emit('CLAMP', this.at(o), this.sc(x, node), this.sc(a, node), this.sc(b, node)); if (a.lb !== undefined) o.lb = a.lb; if (b.ub !== undefined) o.ub = b.ub; return o; }
    if (name === 'bls') return this.blsCall(node);
    const args = node.args.map(a => this.expr(a));
    if (args.length !== K.args.length) this.err(`${name} takes ${K.args.length} arguments`, node);
    const ops = args.map((a, i) => {
      const want = K.args[i];
      if (want === 1) return this.sc(a, node.args[i]);
      a = this.unopt(a);
      if (!a.t || (a.t.k !== 'arr' && a.t.k !== 'list')) this.err(`${name}: argument ${i + 1} must be ${want} numbers`, node.args[i]);
      if (a.t.k === 'arr' && (a.t.n < want || a.t.el.k !== 'num')) this.err(`${name}: argument ${i + 1} must be ${want} numbers, got ${tstr(a.t)}`, node.args[i]);
      if (a.t.k === 'list') { const tmp = this.tmp(RT.arr(RT.num, want)); this.copyInto(tmp, a, node); return this.at(tmp); }
      return this.at(this.stat(a));
    });
    if (K.ret === 1) { const o = this.tmp(RT.num); this.emit(K.op, this.at(o), ...ops); return o; }
    const o = this.tmp(RT.arr(RT.num, K.ret)); this.emit(K.op, this.at(o), ...ops); return o;
  }
  helperFn(name) {
    this.helperCache = this.helperCache || {};
    if (!this.helperCache[name]) { const ast = rnParse(this.helpers[name]); this.helperCache[name] = { k: 'fn', node: ast, scopes: [] }; }
    return this.helperCache[name];
  }
  blsCall(node) {
    const a = node.args.map(x => x);
    if (a.length < 5) this.err('bls(cols, lo, hi, w, W, pull)', node);
    const cols = this.unopt(this.expr(a[0])), lo = this.unopt(this.expr(a[1])), hi = this.unopt(this.expr(a[2])), w = this.expr(a[3]), W = this.expr(a[4]);
    const asList = (v, cap, n) => { if (v.t.k === 'list' && v.t.cap === cap) return this.stat(v); const t = this.tmp(RT.list(v.t.el, cap)); this.copyInto(t, v, n); return t; };
    if (cols.t.k !== 'list' && cols.t.k !== 'arr') this.err('bls: cols must be a list of columns', a[0]);
    const cap = cols.t.k === 'arr' ? cols.t.n : cols.t.cap;
    const K = cols.t.el.k === 'arr' ? cols.t.el.n : this.err('bls: each column must have a fixed length', a[0]);
    const C = asList(cols, cap, a[0]), Lo = asList(lo, cap, a[1]), Hi = asList(hi, cap, a[2]);
    const wv = this.stat(this.unopt(w)), Wv = this.stat(this.unopt(W));
    if (wv.t.k !== 'arr' || wv.t.n !== K || Wv.t.k !== 'arr' || Wv.t.n !== K) this.err(`bls: w and W must have ${K} numbers`, node);
    let pq = -1, pr = -1, rel = this.cref(1);
    if (a[5]) {
      let q, r, relV;
      if (a[5].type === 'ObjectLit') {
        for (const p of a[5].props) { const v = this.expr(p.value); if (p.key === 'q') q = v; else if (p.key === 'r') r = v; else if (p.key === 'rel') relV = v; }
      } else { const pv = this.expr(a[5]); if (pv.k !== 'null') { q = this.field(pv, 'q', a[5]); r = this.field(pv, 'r', a[5]); if ('rel' in (this.unopt(pv).t.f || {})) relV = this.field(pv, 'rel', a[5]); } }
      if (q) pq = this.at(asList(this.unopt(q), cap, a[5]));
      if (r) pr = this.at(asList(this.unopt(r), cap, a[5]));
      if (relV) rel = this.sc(relV, a[5]);
    }
    const out = this.tmp(RT.list(RT.num, cap));
    this.emit('BLS', this.at(out), this.at(C), this.at(Lo), this.at(Hi), this.at(wv), this.at(Wv), pq, pr, rel, K, cap);
    return out;
  }


  // ── fused steps over whole lists ──
  // `list.map((v, j) => …)` and `list.reduce((s, v, j) => s + …, 0)` whose body is arithmetic on the element,
  // on other lists at the same index and on numbers that don't change along the list, become a few VV / VS /
  // VDOT steps instead of a loop of single steps. Anything else falls back to the loop.
  vecCx(src, f) {
    const fn = f.node; if (fn.body.type === 'Block') return null;
    const t = src.t; if (t.k !== 'list' && !(t.k === 'arr' && t.n > RN_UNROLL)) return null;
    if (!t.el) return null;
    return { src, t, es: tsize(t.el), cap: t.k === 'arr' ? t.n : t.cap, f, fn };
  }
  vecKind(node, cx) {
    const refs = n => rnRefs(n, cx.names);
    switch (node.type) {
      case 'Num': return 'S';
      case 'Id':
        if (node.name === cx.v) return cx.t.el.k === 'num' ? 'V' : null;
        if (node.name === cx.i || node.name === cx.acc) return null;
        return 'S';
      case 'Member':
        if (node.computed && node.index.type === 'Id' && node.index.name === cx.i && !refs(node.obj)) return 'V';
        if (node.obj.type === 'Id' && node.obj.name === cx.v) {
          if (node.computed) return typeof this.constOf(node.index) === 'number' ? 'V' : null;
          return node.prop === 'length' ? null : 'V';
        }
        return refs(node) ? null : 'S';
      case 'Binary': {
        if (!['+', '-', '*', '/'].includes(node.op)) return refs(node) ? null : 'S';
        const a = this.vecKind(node.left, cx), b = this.vecKind(node.right, cx);
        return !a || !b ? null : a === 'V' || b === 'V' ? 'V' : 'S';
      }
      case 'Unary': return node.op === '-' ? this.vecKind(node.arg, cx) : refs(node) ? null : 'S';
    }
    return refs(node) ? null : 'S';
  }
  // A view of a list's numbers: { s | reg, off, stride, cap }.
  listView(v) {
    v = this.unopt(v); const t = v.t;
    if (!t || (t.k !== 'list' && t.k !== 'arr') || !t.el || t.el.k !== 'num') return null;
    const base = t.k === 'list' ? 1 : 0, cap = t.k === 'list' ? t.cap : t.n;
    if (v.s) return { s: v.s, off: v.off + base, stride: 1, cap };
    if (v.reg) return { reg: v.reg, off: v.off + base, stride: 1, cap };
    return null;
  }
  vecOperands(w) { return w.reg ? [{ s: w.reg, off: 0 }, w.off, w.stride] : [-1, { s: w.s, off: w.off }, w.stride]; }
  vecEmit(node, cx) {
    const kind = this.vecKind(node, cx);
    if (kind === 'S') return { S: this.sc(this.expr(node), node) };
    switch (node.type) {
      case 'Id': return { V: cx.elemView(0) };
      case 'Member': {
        if (node.obj.type === 'Id' && node.obj.name === cx.v) {         // a number inside each element
          const et = cx.t.el; let off, ft;
          if (node.computed) { const k = this.constOf(node.index); if (et.k !== 'arr' || !(k >= 0 && k < et.n)) throw new RnNoFuse(); off = k * tsize(et.el); ft = et.el; }
          else { if (et.k !== 'rec' || !(node.prop in et.f)) throw new RnNoFuse(); off = fieldOff(et, node.prop); ft = et.f[node.prop]; }
          if (ft.k !== 'num') throw new RnNoFuse();
          return { V: cx.elemView(off) };
        }
        const o = this.expr(node.obj), w = this.listView(o);             // another list at the same index
        if (!w) throw new RnNoFuse();
        cx.capMin = Math.min(cx.capMin, w.cap);
        return { V: w };
      }
      case 'Unary': { const a = this.vecEmit(node.arg, cx); return this.vecOp('rsub', a, { S: this.cref(0) }, cx); }
      case 'Binary': {
        const a = this.vecEmit(node.left, cx), b = this.vecEmit(node.right, cx);
        return this.vecOp({ '+': 'add', '-': 'sub', '*': 'mul', '/': 'div' }[node.op], a, b, cx);
      }
    }
    throw new RnNoFuse();
  }
  vecTmp(cx) { const s = this.slot('tmp', cx.cap, 'vec'); return { s, off: 0, stride: 1, cap: cx.cap }; }
  // One fused step: a ∘ b into a new temporary list (or into `into`).
  vecOp(kind, a, b, cx, into) {
    const d = into || this.vecTmp(cx), cap = () => cx.capMin;
    if (a.V && b.V) this.emit('VV', RN_VKIND[kind], ...this.vecOperands(d), ...this.vecOperands(a.V), ...this.vecOperands(b.V), cx.count, cap);
    else if (a.V) this.emit('VS', RN_VKIND[kind], ...this.vecOperands(d), ...this.vecOperands(a.V), b.S, cx.count, cap);
    else {                                                   // number ∘ list
      const rk = { add: 'add', mul: 'mul', sub: 'rsub', div: 'rdiv', rsub: 'sub', rdiv: 'div' }[kind];
      this.emit('VS', RN_VKIND[rk], ...this.vecOperands(d), ...this.vecOperands(b.V), a.S, cx.count, cap);
    }
    return { V: d };
  }
  vecSetup(cx) {
    const src = cx.src;
    cx.capMin = cx.cap;
    cx.count = this.lenOf(src);
    const base = src.t.k === 'list' ? 1 : 0;
    cx.elemView = off => src.s ? { s: src.s, off: src.off + base + off, stride: cx.es } : { reg: src.reg, off: src.off + base + off, stride: cx.es };
  }
  vecMap(src, f, node) {
    const cx = this.vecCx(src, f); if (!cx) return null;
    if (cx.fn.params.some(p => p.type !== 'Id')) return null;
    const ps = cx.fn.params.map(p => p.name);
    cx.v = ps[0]; cx.i = ps[1]; cx.names = ps.filter(Boolean);
    const saved = this.scopes; this.scopes = f.scopes.slice(); this.push();
    try {
      if (this.vecKind(cx.fn.body, cx) !== 'V') return null;
      const mark = this.code.length, nSlots = this.slots.length;
      try {
        this.vecSetup(cx);
        const out = this.tmp(RT.list(RT.num, cx.cap));
        this.emit('LLEN', this.at(out), cx.count, cx.cap, 1);
        const r = this.vecEmit(cx.fn.body, cx);
        this.vecOp('mul', r, { S: this.cref(1) }, cx, { s: out.s, off: 1, stride: 1, cap: cx.cap });
        return out;
      } catch (e) { if (!(e instanceof RnNoFuse)) throw e; this.code.length = mark; return null; }
    } finally { this.pop(); this.scopes = saved; }
  }
  vecReduce(src, f, init, node) {
    const cx = this.vecCx(src, f); if (!cx) return null;
    if (cx.fn.params.some(p => p.type !== 'Id') || cx.fn.params.length < 2) return null;
    const ps = cx.fn.params.map(p => p.name), body = cx.fn.body;
    if (!init.t || init.t.k !== 'num') return null;
    if (body.type !== 'Binary' || body.op !== '+') return null;
    const accLeft = body.left.type === 'Id' && body.left.name === ps[0], accRight = body.right.type === 'Id' && body.right.name === ps[0];
    if (!accLeft && !accRight) return null;
    const term = accLeft ? body.right : body.left;
    cx.v = ps[1]; cx.i = ps[2]; cx.acc = ps[0]; cx.names = ps.filter(Boolean);
    const saved = this.scopes; this.scopes = f.scopes.slice(); this.push();
    try {
      if (this.vecKind(term, cx) !== 'V') return null;
      const mark = this.code.length, nSlots = this.slots.length;
      try {
        this.vecSetup(cx);
        let a, b;
        if (term.type === 'Binary' && term.op === '*' && this.vecKind(term.left, cx) === 'V' && this.vecKind(term.right, cx) === 'V') { a = this.vecEmit(term.left, cx); b = this.vecEmit(term.right, cx); }
        else { a = this.vecEmit(term, cx); b = { V: { s: this.prog.constSlot(1), off: 0, stride: 0 } }; }
        const d = this.tmp(RT.num);
        this.emit('VDOT', this.at(d), ...this.vecOperands(a.V), ...this.vecOperands(b.V), cx.count, () => cx.capMin);
        if (init.c === 0) return d;
        return this.arith('ADD', init, d, node);
      } catch (e) { if (!(e instanceof RnNoFuse)) throw e; this.code.length = mark; return null; }
    } finally { this.pop(); this.scopes = saved; }
  }

  // ── statements ──
  block(b, isFnBody) { this.push(); for (const s of b.body) this.stmt(s); this.pop(); }
  stmt(node) {
    if (node.pos != null && node.type !== 'Block') this.curPos = node.pos;
    switch (node.type) {
      case 'Block': return this.block(node);
      case 'Empty': return;
      case 'ExprStmt': this.expr(node.expr); return;
      case 'VarDecl':
        for (const d of node.decls) {
          if (!d.init) {
            if (node.kind === 'const') this.err('const needs a value', node);
            if (d.id.type !== 'Id') this.err('Unsupported declaration', node);
            const v = { t: null, s: null, off: 0, pending: true };
            v.s = this.slot('var', () => tsize(v.t), d.id.name);
            const b = this.bind(d.id.name, { kind: 'let', v }, node); b.pins = [v.s]; this.touch(v.s);
            continue;
          }
          const v = this.expr(d.init);
          this.bindPattern(d.id, v, node.kind, d.init);
          if (node.kind === 'let' && d.id.type === 'Id') {
            const b = this.lookup(d.id.name);
            if (this.assignedLater && this.assignedLater.has(d.id.name) && b.v) { b.v = { ...b.v }; delete b.v.ub; delete b.v.lb; }
          }
        }
        return;
      case 'If': {
        const tv = this.tryConst(node.test);
        if (tv !== undefined) { if (tv) this.stmt(node.cons); else if (node.alt) this.stmt(node.alt); return; }
        // `if (!pull)` on an argument that can't be null: never true.
        if (node.test.type === 'Unary' && node.test.op === '!' && (node.test.arg.type === 'Id') && !this.stateFlag(node.test.arg)) {
          const v = this.expr(node.test.arg);
          if (this.truth(v, node) === true) { if (node.alt) this.stmt(node.alt); return; }
        }
        const f = this.label(), end = this.label();
        this.cond(node.test, f, false);
        this.push(); this.stmt(node.cons); this.pop();
        if (node.alt) { this.emit('JMP', end); this.place(f); this.push(); this.stmt(node.alt); this.pop(); this.place(end); }
        else this.place(f);
        return;
      }
      case 'For': return this.forStmt(node);
      case 'ForOf': {
        const list = this.expr(node.list);
        this.forEach(list, e => { this.push(); this.bindPattern(node.id, e, node.kind === 'let' ? 'let' : 'const', node); this.stmt(node.body); this.pop(); }, node);
        return;
      }
      case 'Return': {
        const fr = this.inlines[this.inlines.length - 1];
        if (fr) {
          const v = node.arg ? this.expr(node.arg) : { k: 'undef' };
          if (!fr.out) {
            if (v.k) { fr.out = v; }
            else { fr.out = this.tmp(v.t.k === 'arr' || v.t.k === 'list' || v.t.k === 'rec' ? JSON.parse(JSON.stringify(v.t)) : v.t); fr.out.fresh = true; }
          }
          if (!fr.out.k) this.copyInto(fr.out, v, node);
          this.emit('JMP', fr.end); return;
        }
        if (!node.arg) this.err('The formula must return a value', node);
        this.into(node.arg, this.retV);
        this.emit('JMP', this.endLabel); return;
      }
      case 'Break': case 'Continue': {
        const l = this.loops[this.loops.length - 1]; if (!l) this.err(`${node.type.toLowerCase()} outside a loop`, node);
        this.emit('JMP', node.type === 'Break' ? l.brk : l.cont); return;
      }
    }
    this.err(`${node.type} isn't supported`, node);
  }
  forStmt(node) {
    // Recognize for (let i = A; i < B; i++) with a known bound.
    const init = node.init, test = node.test, upd = node.update;
    const simple = init && init.type === 'VarDecl' && init.decls.length === 1 && init.decls[0].id.type === 'Id' && test && test.type === 'Binary' && ['<', '<='].includes(test.op) &&
      test.left.type === 'Id' && test.left.name === init.decls[0].id.name && upd && upd.type === 'Update' && upd.op === '++' && upd.target.type === 'Id' && upd.target.name === test.left.name;
    if (!simple) this.err('Loops must look like for (let i = start; i < end; i++) or for (const x of list)', node);
    const name = test.left.name;
    if (this.vecFor(node, name)) return;
    this.push();
    const a = this.expr(init.decls[0].init), b = this.expr(test.right);
    const incl = test.op === '<=' ? 1 : 0;
    if (a.c !== undefined && b.c !== undefined && b.c + incl - a.c <= RN_UNROLL && !assignedNames(node.body).has(name)) {
      const brk = this.label();
      for (let i = a.c; i < b.c + incl; i++) {
        const cont = this.label(); this.push();
        this.bind(name, { kind: 'const', v: { t: RT.num, c: i, lb: i, ub: i } }, node);
        this.loops.push({ brk, cont, start: this.code.length }); this.stmt(node.body); this.loops.pop();
        this.pop(); this.place(cont);
      }
      this.place(brk); this.pop(); return;
    }
    if (b.ub === undefined) this.err(`Can't tell how many times this loop runs: "${srcOf(this.src, test.right)}" needs an upper limit (a list's length or a number)`, node);
    const max = Math.max(0, Math.ceil(b.ub + incl - (a.lb ?? a.c ?? 0)));
    const iv = this.slot('var', 1, name), top = this.label(), cont = this.label(), end = this.label(), c = this.reg();
    this.storeScalar({ t: RT.num, s: iv, off: 0 }, this.sc(a, node));
    const bref = this.sc(b, node);
    const start = this.code.length; this.place(top);
    const w0 = this.weight; this.weight = w0 * Math.max(1, max);
    this.emit(incl ? 'LE' : 'LT', { s: c, off: 0 }, { s: iv, off: 0 }, bref); this.emit('JZ', { s: c, off: 0 }, end);
    this.push(); const bnd = this.bind(name, { kind: 'var', v: { t: RT.num, s: iv, off: 0, lb: a.lb ?? a.c, ub: b.ub - 1 + incl } }, node); bnd.pins = [iv];
    this.loops.push({ brk: end, cont, start });
    this.stmt(node.body);
    this.loops.pop(); this.pop();
    this.place(cont); this.emit('ADD', { s: iv, off: 0 }, { s: iv, off: 0 }, this.cref(1)); this.emit('JMP', top);
    this.weight = w0; this.place(end); this.closeLoop(start);
    this.pop();
  }

  // for (let i = 0; i < N; i++) X[i] = expr, where expr is arithmetic on lists at index i and on numbers that
  // don't change in the loop: a few fused steps. The numbers may only be constants and plain variables, and X
  // is only read as X[i], so the result is the same as running the loop one element at a time.
  vecFor(node, name) {
    const init = node.init.decls[0].init, test = node.test;
    if (test.op !== '<' || !(init.type === 'Num' && init.value === 0)) return false;
    let body = node.body; if (body.type === 'Block') { if (body.body.length !== 1) return false; body = body.body[0]; }
    if (body.type !== 'ExprStmt' || body.expr.type !== 'Assign' || body.expr.op !== '=') return false;
    const tgt = body.expr.target, val = body.expr.value;
    if (tgt.type !== 'Member' || !tgt.computed || tgt.index.type !== 'Id' || tgt.index.name !== name) return false;
    const chain = n => n.type === 'Id' ? n.name : n.type === 'Member' && !n.computed ? chain(n.obj) : null;
    const root = chain(tgt.obj); if (!root || root === name) return false;
    const key = n => JSON.stringify(n, (k, v) => k === 'pos' ? undefined : v), tkey = key(tgt.obj);
    const plain = n => n.type === 'Num' || (n.type === 'Id' && n.name !== name && n.name !== root) ||
      (n.type === 'Binary' && ['+', '-', '*', '/'].includes(n.op) && plain(n.left) && plain(n.right)) || (n.type === 'Unary' && n.op === '-' && plain(n.arg));
    const ok = n => {                                    // every part: X[i] (X not the loop's own target unless it is X), or plain
      if (n.type === 'Member' && n.computed && n.index.type === 'Id' && n.index.name === name) { const r = chain(n.obj); return !!r && (r !== root || key(n.obj) === tkey) && !rnRefs(n.obj, [name]); }
      if (n.type === 'Binary' && ['+', '-', '*', '/'].includes(n.op)) return ok(n.left) && ok(n.right);
      if (n.type === 'Unary' && n.op === '-') return ok(n.arg);
      return plain(n);
    };
    if (!ok(val)) return false;
    const cx = { names: [name], i: name, v: null };
    if (this.vecKind(val, cx) !== 'V') return false;
    const mark = this.code.length;
    try {
      const b = this.expr(test.right); if (b.ub === undefined && b.c === undefined) return false;
      const dst = this.listView(this.expr(tgt.obj)); if (!dst) return false;
      const tv = this.unopt(this.expr(tgt.obj));
      cx.cap = cx.capMin = dst.cap; cx.count = this.sc(b, test.right);
      const len = this.lenOf(tv, node), okc = this.reg(), fine = this.label();
      this.emit('LE', { s: okc, off: 0 }, cx.count, len); this.emit('JNZ', { s: okc, off: 0 }, fine); this.emit('TRAP', 3); this.place(fine);
      const r = this.vecEmit(val, cx);
      this.vecOp('mul', r, { S: this.cref(1) }, cx, dst);
      return true;
    } catch (e) { if (!(e instanceof RnNoFuse)) throw e; this.code.length = mark; return false; }
  }

  // ── the whole formula ──
  // A formula's memory gets its fields' types from their first assignment, but code can read a field above the
  // line that first sets it (`if (dt <= 0) return st.th ? … : …`). So a first pass compiles what it can,
  // statement by statement, to learn the fields' types; the real pass then knows them all.
  discover() {
    const si = this.sig.args.findIndex(t => t.k === 'state'); if (si < 0) return this.sig;
    const probe = new RnFn(new RnProgram(), this.key, this.src, this.sig, this.helpers); probe.lenient = true;
    try { probe.compile(); } catch (e) { }
    const decl = { ...this.sig.args[si].decl };
    if (probe.stateT && probe.stateT.fields) for (const [name, f] of probe.stateT.fields) if (f.t && !decl[name]) decl[name] = f.t;
    const args = this.sig.args.slice(); args[si] = RT.state(decl);
    return { ...this.sig, args };
  }
  compile() {
    if (!this.lenient && !this.discovered) { this.sig = this.discover(); this.discovered = true; }
    const ast = rnParse(this.src);
    const fn = ast.type === 'Function' || ast.type === 'Arrow' ? ast : this.err('Expected a function');
    this.assignedLater = assignedNames(fn.body);
    const sig = this.sig;
    if (fn.params.length !== sig.args.length) this.err(`This formula takes ${sig.args.length} inputs (${sig.names.join(', ')}); the code has ${fn.params.length}`);
    this.push();
    this.args = fn.params.map((p, i) => {
      const t = sig.args[i];
      if (t.k === 'state') { const v = { t: { ...t, fields: new Map() }, stateRoot: true }; this.stateT = v.t; this.bindPattern(p, v, 'const', p); return { t, state: true }; }
      const s = this.slot('arg', tsize(t), sig.names[i]); const v = { t, s, off: 0 };
      this.bindPattern(p, v, assignedNames(fn.body).has(p.name) ? 'let' : 'const', p);
      return { t, s };
    });
    const retS = this.slot('ret', tsize(sig.ret), 'ret'); this.retV = { t: sig.ret, s: retS, off: 0 };
    this.endLabel = this.label();
    const body = fn.body.type === 'Block' ? fn.body : null;
    if (body && this.lenient) {                          // discovery: skip what can't be typed yet
      this.push();
      for (const st of body.body) { const depth = this.scopes.length, n = this.code.length; try { this.stmt(st); } catch (e) { this.scopes.length = depth; this.code.length = n; this.loops = []; this.inlines = []; this.weight = 1; } }
      this.pop();
    } else if (body) this.block(body, true); else { this.into(fn.body, this.retV); }
    this.place(this.endLabel);
    this.pop();
    return this;
  }
}
class RnNoFuse extends Error { }
// Whether a piece of code mentions any of these names.
function rnRefs(node, names) {
  if (!node || typeof node !== 'object') return false;
  if (Array.isArray(node)) return node.some(n => rnRefs(n, names));
  if (node.type === 'Id') return names.includes(node.name);
  for (const k in node) if (k !== 'pos' && node[k] && typeof node[k] === 'object' && rnRefs(node[k], names)) return true;
  return false;
}
// Names assigned anywhere inside a piece of code.
function assignedNames(node, out = new Set()) {
  if (!node || typeof node !== 'object') return out;
  if (Array.isArray(node)) { node.forEach(n => assignedNames(n, out)); return out; }
  if ((node.type === 'Assign' || node.type === 'Update') && node.target.type === 'Id') out.add(node.target.name);
  for (const k in node) if (k !== 'pos' && node[k] && typeof node[k] === 'object') assignedNames(node[k], out);
  return out;
}
const srcOf = (src, node) => { const s = src.slice(node.pos, node.pos + 40); return s.split(/[;)\n]/)[0]; };

const RN_GLOBALS = { G: 9.81, D2R: Math.PI / 180, R2D: 180 / Math.PI };
// Helpers from math.js that run as one native step. args: 1 = a number, k = k numbers.
const RN_KERNELS = {
  add: { op: 'ADD3', args: [3, 3], ret: 3 }, sub: { op: 'SUB3', args: [3, 3], ret: 3 }, scl: { op: 'SCL3', args: [3, 1], ret: 3 },
  dot: { op: 'DOT3', args: [3, 3], ret: 1 }, crs: { op: 'CRS', args: [3, 3], ret: 3 }, nrm: { op: 'NRM3', args: [3], ret: 1 },
  unit: { op: 'UNIT3', args: [3], ret: 3 }, m3v: { op: 'M3V', args: [9, 3], ret: 3 }, m3T: { op: 'M3T', args: [9], ret: 9 },
  m3m: { op: 'M3M', args: [9, 9], ret: 9 }, qmul: { op: 'QMUL', args: [4, 4], ret: 4 }, qmat: { op: 'QMAT', args: [4], ret: 9 },
  qnorm: { op: 'QNORM', args: [4], ret: 4 }, matToQuat: { op: 'M2Q', args: [9], ret: 4 },
  clamp: { special: true }, bls: { special: true },
};
// Helpers compiled from their JavaScript like any formula code.
const RN_HELPERS = {
  eye: 'n => { const M = []; for (let i = 0; i < n; i++) { const row = new Array(n).fill(0); row[i] = 1; M.push(row); } return M; }',
};
const RN_FOLD = {
  ADD: (a, b) => a + b, SUB: (a, b) => a - b, MUL: (a, b) => a * b, DIV: (a, b) => a / b, MOD: (a, b) => a % b, POW: (a, b) => a ** b,
  MIN: Math.min, MAX: Math.max, ATAN2: Math.atan2, LT: (a, b) => +(a < b), LE: (a, b) => +(a <= b), EQ: (a, b) => +(a === b), NE: (a, b) => +(a !== b),
  NEG: a => -a, ABS: Math.abs, SQRT: Math.sqrt, SIN: Math.sin, COS: Math.cos, TAN: Math.tan, ASIN: Math.asin, ACOS: Math.acos, ATAN: Math.atan,
  EXP: Math.exp, LOG: Math.log, FLOOR: Math.floor, CEIL: Math.ceil, ROUND: Math.round, SIGN: Math.sign, NOT: a => +!a, TRUTH: a => +!!a,
};


/* ───────── linking: places in the arena, jump offsets, the program ───────── */
// Constants first (read-only), then each formula's memory, inputs and result, then one pool of temporary
// space shared by all formulas (only one runs at a time). Inside the pool, a place is reused once nothing
// reads it any more.
function rnLink(prog, fns) {
  let addr = 0;
  for (const s of prog.consts.values()) { s.addr = addr; addr += 1; }
  for (const s of prog.constBlocks) { s.addr = addr; addr += s.size; }
  const constEnd = addr;
  const constData = new Float32Array(constEnd);
  for (const s of prog.consts.values()) constData[s.addr] = s.init[0];
  for (const s of prog.constBlocks) constData.set(s.init, s.addr);
  for (const f of fns) for (const s of f.slots) if (s.kind === 'state' || s.kind === 'arg' || s.kind === 'ret') { s.addr = addr; addr += s.size; }
  const poolBase = addr; let poolSize = 0;
  for (const f of fns) {
    const live = f.slots.filter(s => (s.kind === 'tmp' || s.kind === 'var') && s.last >= 0).sort((a, b) => a.first - b.first || b.size - a.size);
    const active = [];                                    // { s, a, e } places in use: [a, e)
    let top = 0;
    for (const s of live) {
      for (let i = active.length - 1; i >= 0; i--) if (active[i].s.last < s.first) active.splice(i, 1);
      active.sort((x, y) => x.a - y.a);
      const n = s.size; let at = 0;
      for (const b of active) { if (b.a - at >= n) break; at = Math.max(at, b.e); }
      s.addr = poolBase + at; active.push({ s, a: at, e: at + n }); top = Math.max(top, at + n);
    }
    for (const s of f.slots) if (s.addr === null) s.addr = poolBase;       // never used
    poolSize = Math.max(poolSize, top);
  }
  const arenaSize = poolBase + poolSize;
  // Code: word offsets, then operands.
  const words = []; const fnTable = {}; const posAt = new Map();
  const opArity = RN_OPS.map(o => o[1].length);
  for (const f of fns) {
    const entry = words.length; const offs = []; let w = entry;
    for (const ins of f.code) { offs.push(w); posAt.set(w, ins.pos); w += 1 + ins.args.length; }
    const end = w;
    const labelAt = l => l.at === null ? (() => { throw new Error('internal: unplaced label'); })() : (l.at < offs.length ? offs[l.at] : end);
    let maxSteps = 0;
    for (const ins of f.code) {
      const op = RN_OP[ins.op];
      if (ins.args.length !== opArity[op]) throw new Error(`internal: ${ins.op} takes ${opArity[op]} operands, got ${ins.args.length}`);
      words.push(op);
      for (const a of ins.args) {
        if (typeof a === 'number') words.push(a);
        else if (typeof a === 'function') words.push(a());
        else if (a.label) words.push(labelAt(a));
        else if (a.s) words.push(a.s.addr + a.off);
        else throw new Error('internal: bad operand');
      }
      maxSteps += ins.w;
    }
    const state = (f.stateSlots || []).map(fl => ({ name: fl.name, t: fl.t, addr: fl.s ? fl.s.addr : -1, flag: fl.flag.addr, ring: fl.t && fl.t.k === 'ring' }));
    fnTable[f.key] = { entry, end, maxSteps: Math.ceil(maxSteps * 1.1) + 16, args: f.args.map((a, i) => ({ name: f.sig.names[i], t: a.t, addr: a.s ? a.s.addr : -1, state: !!a.state })), ret: { t: f.sig.ret, addr: f.retV.s.addr }, state, nInstr: f.code.length };
  }
  return { posAt, code: Int32Array.from(words), constData, constEnd, arenaSize, fns: fnTable, poolBase, poolSize };
}

// A readable listing of a formula's steps (the Formulas tab shows it).
function rnListing(prog, key) {
  const f = prog.fns[key]; if (!f) return '';
  const names = new Map(), sizes = new Map();
  const put = (a, n, sz) => { names.set(a, n); sizes.set(a, sz); };
  for (const a of f.args) if (a.addr >= 0) put(a.addr, a.name, tsize(a.t));
  put(f.ret.addr, 'result', tsize(f.ret.t));
  for (const st of f.state) { if (st.addr >= 0 && st.t) put(st.addr, 'st.' + st.name, tsize(st.t)); put(st.flag, 'st.' + st.name + '?', 1); }
  const nm = a => {
    if (a < prog.constEnd) { const v = prog.constData[a]; return Number.isInteger(v) ? String(v) : (+v.toPrecision(5)).toString(); }
    if (names.has(a)) return names.get(a);
    for (const [base, n] of names) if (a > base && a < base + sizes.get(base)) return `${n}[${a - base}]`;
    return '@' + (a - prog.poolBase);
  };
  const lines = []; let pc = f.entry;
  const targets = new Set();
  for (let p = f.entry; p < f.end;) { const op = prog.code[p], spec = RN_OPS[op][1]; spec.split('').forEach((k, i) => { if (k === 't') targets.add(prog.code[p + 1 + i]); }); p += 1 + spec.length; }
  while (pc < f.end) {
    const op = prog.code[pc], [name, spec] = RN_OPS[op];
    const ops = spec.split('').map((k, i) => { const v = prog.code[pc + 1 + i]; return k === 't' ? `→${v - f.entry}` : k === 'i' ? (name === 'CPY' || name === 'CPI' || name === 'CPO' ? (i === spec.length - 1 ? v : nm(v)) : /^(M3V|M3M|M3T|CRS|QMUL|QMAT|QNORM|M2Q|UNIT3|NRM3|DOT3|ADD3|SUB3|SCL3)$/.test(name) || (name === 'BLS' && i < 9) || (/^(LLEN|LFILL|RSHIFT|RCLR|FILL|RIDX)$/.test(name) && i < 1) || (/^(CPYL|PUSH|RPUSH)$/.test(name) && i < 2) ? nm(v) : String(v)) : k === 'o' ? (v < 0 ? '—' : nm(v)) : k === 'r' || k === 'R' ? '%' + (v - prog.poolBase) : nm(v); });
    lines.push(`${String(pc - f.entry).padStart(5)}${targets.has(pc) ? ':' : ' '} ${name.padEnd(6)} ${ops.join(', ')}`);
    pc += 1 + spec.length;
  }
  return lines.join('\n');
}

function rnCompileAll(sources, sigs, opts = {}) {
  const prog = new RnProgram(), fns = [], errors = {};
  for (const [key, src] of Object.entries(sources)) {
    try { fns.push(new RnFn(prog, key, src, sigs[key], opts.helpers).compile()); }
    catch (e) { if (opts.throw) throw e; errors[key] = e.message; }
  }
  const P = rnLink(prog, fns); P.errors = errors; return P;
}

if (typeof module !== 'undefined') module.exports = { rnLink, rnListing, rnCompileAll, RT, tsize, tstr, teq, tunify, RnProgram, RnFn, RnCompileError, RN_KERNELS, RN_HELPERS, RN_GLOBALS, RN_LIST_CAP };
