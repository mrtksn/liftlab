'use strict';
// The simulator's flight code on the step runner. The flight formulas in use (edited or default) are compiled
// into one program (rn-compile.js) and loaded into the C runner built to WebAssembly (runner/rn.c), the same
// code the ESP32 runs; where WebAssembly isn't available, the JavaScript runner (rn-vm.js) runs the same steps.
// While the flight code runs (a control step), its formula calls are answered from there. The physics, the
// sensors and the supervisor stay in JavaScript: they aren't flight-controller code.
//
// An edit reaches the flying program the way it would reach the drone (runner/rn_host.c does the same there):
//   1. compile   the program with the edit, checked step by step (rnVerify);
//   2. load      into a second runner, whose loader checks every address again and runs the self-tests: real
//                inputs from this flight, with the outputs the JavaScript runner gets for them;
//   3. shadow    the new program runs beside the flying one on the same inputs for RN_SHADOW_S of flight,
//                starting from a copy of the flying program's memory. Its answers aren't used; a trap, a step
//                limit or a number that isn't finite rejects it;
//   4. blend     over RN_BLEND_S the answers used move from the old program's to the new one's;
//   5. swap      the new program flies, with the memory it built up in the background. The old one is kept.
//   6. fall back if the new program traps in flight, the old one takes over again with the memory carried
//                across by name.
// Nothing flying changes until step 5, so a bad edit never touches the flight.

const RN_STEP_OPS = 6;          // budget: one step ≈ 6 of the budget's operations (dispatch, operands, the math)
const RN_SHADOW_S = 1.0, RN_BLEND_S = 0.3, RN_TESTS_PER_FN = 2;
const RN = {
  want: true, engine: null, wasmErr: '', P: null, A: null, trapped: {}, buildErr: '',
  steps: {}, calls: {}, maxSteps: {}, lastWork: 0, compileMs: 0,
  act: null, prev: null, stage: null, pool: [], module: null, log: [], flying: {}, samples: {}, sampleN: {},
};
try { RN.want = localStorage.getItem('dfb-runner') !== 'off'; } catch (e) { }

/* ───────── engines: a loaded program and its arena ───────── */
// A C runner (its own WebAssembly instance and memory) or, without WebAssembly, the JavaScript runner.
class RnEngine {
  constructor(w) { this.w = w; this.kind = w ? 'wasm' : 'js'; this.P = null; this.A = null; this.seen = new WeakSet(); this.selfTests = 0; this.selfWorst = 0; }
  load(P, tests) {
    this.seen = new WeakSet();
    if (this.w) { const e = this.w.load(P, tests); if (e) return e; this.A = this.w.A; this.selfTests = this.w.selfTests; this.selfWorst = this.w.selfWorst; }
    else { this.A = rnArena(P); this.selfTests = 0; }
    this.P = P; return null;
  }
  run(key) { return this.w ? this.w.run(key) : rnRun(this.P, this.A, key); }
  work() { return this.w ? this.w.work() : 0; }
}
// Runners come from a small pool of WebAssembly instances (made ahead: making one takes a moment).
function rnTakeEngine() {
  if (RN.module) { const w = RN.pool.pop(); if (w) return new RnEngine(w); }
  return new RnEngine(null);
}
function rnGiveBack(E) { if (E && E.w && RN.pool.length < 3) RN.pool.push(E.w); }
function rnFillPool() {
  if (!RN.module) return;
  while (RN.pool.length + (RN.poolPending || 0) < 2) {
    RN.poolPending = (RN.poolPending || 0) + 1;
    RnWasm.fromModule(RN.module).then(w => { RN.poolPending--; RN.pool.push(w); }, e => { RN.poolPending--; RN.wasmErr = String(e && e.message || e); });
  }
}

/* ───────── which source flies ───────── */
// An edit that's applied flies; one that was rejected keeps whatever version was flying.
function rnSourceOf(key) {
  const L = LAWS[key];
  if (L.status === 'edited') return L.src;
  if (L.status === 'default') return L.defSrc;
  return RN.flying[key] || L.defSrc;
}
function rnSources() { const s = {}; for (const k of Object.keys(RN_SIGS)) s[k] = rnSourceOf(k); return s; }
function rnCompile(srcs) {
  const t0 = performance.now();
  const P = rnCompileAll(srcs, RN_SIGS); rnVerify(P);
  RN.compileMs = performance.now() - t0;
  return P;
}
function rnEvent(msg, tone) { RN.log.unshift({ t: typeof S !== 'undefined' ? S.t : 0, msg, tone: tone || '' }); RN.log.length = Math.min(RN.log.length, 12); }

// Load a program straight away (at start, or when the runner is switched on): nothing is flying on it yet.
function rnRebuild() {
  rnCancelStage();
  let P;
  try { P = rnCompile(RN.srcs = rnSources()); }
  catch (e) { RN.P = null; RN.buildErr = e.message; rnRender(); return; }
  const E = rnTakeEngine(), err = E.load(P);
  if (err) { RN.wasmErr = 'the C runner rejected the program: ' + err; rnGiveBack(E); return rnInstall(Object.assign(new RnEngine(null), {}), P); }
  rnInstall(E, P);
}
function rnInstall(E, P) {
  if (!E.P) E.load(P);
  E.srcs = { ...RN.srcs };
  rnGiveBack(RN.prev); RN.prev = RN.act; RN.act = E;
  RN.P = E.P; RN.A = E.A; RN.engine = E.kind; RN.buildErr = ''; RN.trapped = {};
  RN.flying = { ...RN.srcs };
  RN.steps = {}; RN.calls = {}; RN.maxSteps = {};
  rnFillPool(); rnRender();
}
const rnRender = () => { if (typeof renderRunner === 'function') renderRunner(); };

/* ───────── staged reload ───────── */
function rnCancelStage() { if (RN.stage) { rnGiveBack(RN.stage.E); RN.stage = null; } }
function rnStage() {
  if (!RN.want || !RN.act) return rnRebuild();
  rnCancelStage();
  const srcs = rnSources(), keys = Object.keys(srcs).filter(k => srcs[k] !== RN.flying[k]);
  if (!keys.length) { rnRender(); return; }
  const st = { keys, srcs, phase: 'compile', msg: '', t0: null, tStart: typeof S !== 'undefined' ? S.t : 0, diff: {}, pairs: [], clones: new WeakMap(), calls: 0 };
  RN.stage = st;
  const names = keys.map(k => LAWS[k].def.title).join(', ');
  let P;
  try { P = rnCompile(srcs); }
  catch (e) { return rnStageFail(st, 'it doesn\'t compile: ' + e.message); }
  for (const k of keys) if (P.errors[k]) return rnStageFail(st, P.errors[k]);
  // Self-tests: recent real inputs of every formula, expected outputs from the JavaScript runner on the new program.
  const samples = []; for (const k of Object.keys(RN.samples)) for (const s of RN.samples[k]) samples.push(s);
  const tests = rnMakeTests(P, samples);
  st.phase = 'load';
  const E = rnTakeEngine(), err = E.load(P, tests);
  st.E = E; st.P = P; st.tests = E.selfTests; st.selfWorst = E.selfWorst;
  if (err) return rnStageFail(st, (E.kind === 'wasm' ? 'the runner rejected it: ' : '') + err);
  // Its memory starts as a copy of the flying program's (rings live only in the arena; the rest is copied
  // from the flight's own memory objects on first use).
  rnTransfer(E.P, E.A, RN.act.P, RN.act.A);
  st.phase = 'shadow';
  rnEvent(`${names}: compiled, loaded${E.selfTests ? `, ${E.selfTests} self-tests passed` : ''}; flying it in the background.`);
  // Nothing is flying: no need to shadow.
  if (!RN.forceShadow && typeof running !== 'undefined' && !running) rnCommit(st);
  rnRender();
}
function rnStageFail(st, why) {
  st.phase = 'failed'; st.msg = why;
  for (const k of st.keys) {
    const L = LAWS[k];
    if (L.status === 'edited') { L.status = 'error'; L.err = `Not loaded: ${why}. The version that was flying keeps flying.`; if (L.fn !== L.def.fn) L.fn = L.def.fn; notifyLawQuiet(k); }
  }
  rnEvent(`${st.keys.map(k => LAWS[k].def.title).join(', ')}: not loaded (${why}).`, 'bad');
  rnGiveBack(st.E); st.E = null; RN.stage = null;
  rnRender();
}
function rnCommit(st) {
  for (const [real, clone] of st.pairs) {
    for (const k of Object.keys(real)) if (!(k in clone)) delete real[k];
    Object.assign(real, clone); st.E.seen.add(real);
  }
  RN.srcs = st.srcs; RN.stage = null;
  rnInstall(st.E, st.P);
  rnEvent(`${st.keys.map(k => LAWS[k].def.title).join(', ')}: flying${st.maxDiffTxt ? ` (${st.maxDiffTxt})` : ''}.`, 'good');
}
// Advance the stage with flight time.
function rnStageTick() {
  const st = RN.stage; if (!st || (st.phase !== 'shadow' && st.phase !== 'blend')) return;
  const t = S.t;
  if (st.t0 === null || t < st.t0) st.t0 = t;
  const age = t - st.t0;
  if (age >= RN_SHADOW_S + RN_BLEND_S) {
    const worst = Object.entries(st.diff).sort((a, b) => b[1] - a[1])[0];
    st.maxDiffTxt = worst && worst[1] > 0 ? `in the background its answers differed from the old version's by up to ${worst[1].toPrecision(2)}, in ${LAWS[worst[0]].def.title.toLowerCase()}` : 'in the background it gave the same answers as the old version';
    rnCommit(st);
  } else { const ph = age >= RN_SHADOW_S ? 'blend' : 'shadow'; if (ph !== st.phase) { st.phase = ph; rnRender(); } }
}
const notifyLawQuiet = key => { for (const f of lawListeners) if (f !== rnOnLaw) f(key); };

/* ───────── calls ───────── */
const rnActive = key => RN.want && RN.act && RN.P && RN.P.fns[key] && !RN.trapped[key];

// Run one formula call on an engine: inputs and memory in, run, result and memory out.
function rnCallOn(E, key, args) {
  const f = E.P.fns[key], A = E.A;
  for (let i = 0; i < f.args.length; i++) {
    const a = f.args[i], v = args[i];
    if (a.state) { const fresh = !E.seen.has(v); rnStateIn(A, f, v, fresh); if (fresh) E.seen.add(v); }
    else rnWrite(A, a.addr, a.t, v, a.name);
  }
  const steps = E.run(key);
  const out = rnRead(A, f.ret.addr, f.ret.t);
  for (let i = 0; i < f.args.length; i++) if (f.args[i].state) rnStateOut(A, f, args[i]);
  return { out, steps };
}
const rnClone = v => v == null || typeof v !== 'object' ? v : Array.isArray(v) ? v.map(rnClone) : Object.fromEntries(Object.entries(v).map(([k, x]) => [k, rnClone(x)]));
// Keep a few real inputs of every formula for the next program's self-tests.
function rnSample(key, args) {
  const n = RN.sampleN[key] = (RN.sampleN[key] || 0) + 1;
  if (n % 293 !== 7) return;
  const f = RN.P.fns[key], a = args.map(rnClone);
  f.args.forEach((x, i) => { if (x.state) for (const st of f.state) if (st.ring && RN.A[st.flag]) a[i][st.name] = rnRead(RN.A, st.addr, st.t); });
  const list = RN.samples[key] = RN.samples[key] || []; list.push({ key, args: a }); if (list.length > RN_TESTS_PER_FN) list.shift();
}
const rnFinite = v => typeof v === 'number' ? Number.isFinite(v) : v == null || typeof v !== 'object' ? true : Array.isArray(v) ? v.every(rnFinite) : Object.values(v).every(rnFinite);
function rnDiff(a, b) {
  if (typeof a === 'number' && typeof b === 'number') return Math.abs(a - b);
  if (!a || !b || typeof a !== 'object' || typeof b !== 'object') return 0;
  let d = 0; for (const k of Object.keys(a)) if (k in b) d = Math.max(d, rnDiff(a[k], b[k])); return d;
}
function rnBlend(a, b, w) {
  if (typeof a === 'number' && typeof b === 'number') return a + (b - a) * w;
  if (Array.isArray(a) && Array.isArray(b)) return a.length === b.length ? a.map((x, i) => rnBlend(x, b[i], w)) : (w < 0.5 ? a : b);
  if (a && b && typeof a === 'object' && typeof b === 'object') { const o = {}; for (const k of Object.keys(a)) o[k] = k in b ? rnBlend(a[k], b[k], w) : a[k]; return o; }
  return w < 0.5 ? a : b;
}

// Answer one formula call from the runner. Returns undefined if it couldn't (the caller then runs the formula
// in JavaScript).
function rnCall(key, args) {
  if (RN.stage) rnStageTick();
  rnSample(key, args);
  const st = RN.stage, shadow = st && (st.phase === 'shadow' || st.phase === 'blend') && st.E.P.fns[key];
  let sargs = null;
  if (shadow) {                                          // the shadow's own copies of the flight's memory
    sargs = args.slice();
    RN.P.fns[key].args.forEach((a, i) => {
      if (!a.state) return;
      let c = st.clones.get(args[i]); if (!c) { c = rnClone(args[i]); st.clones.set(args[i], c); st.pairs.push([args[i], c]); st.E.seen.add(c); }
      sargs[i] = c;
    });
  }
  let r;
  try { r = rnCallOn(RN.act, key, args); }
  catch (e) { rnTrapped(key, e.message); return undefined; }
  let out = r.out;
  RN.steps[key] = (RN.steps[key] || 0) + r.steps; RN.calls[key] = (RN.calls[key] || 0) + 1;
  if (r.steps > (RN.maxSteps[key] || 0)) RN.maxSteps[key] = r.steps;
  RN.lastWork = r.steps * RN_STEP_OPS + RN.act.work();
  if (shadow) {
    let s;
    try { s = rnCallOn(st.E, key, sargs); } catch (e) { rnStageFail(st, `in the background run, ${e.message}`); return out; }
    if (!rnFinite(s.out)) { rnStageFail(st, `in the background run, ${LAWS[key].def.title.toLowerCase()} gave a number that isn't finite`); return out; }
    st.diff[key] = Math.max(st.diff[key] || 0, rnDiff(out, s.out)); st.calls++;
    if (st.phase === 'blend') out = rnBlend(out, s.out, Math.min(1, (S.t - st.t0 - RN_SHADOW_S) / RN_BLEND_S));
  }
  return out;
}
// A step trapped in flight. An edit that just came in: back to the program before it. Otherwise that formula
// runs in JavaScript.
function rnTrapped(key, msg) {
  if (msg.startsWith(key + ': ')) msg = msg.slice(key.length + 2);
  const L = LAWS[key], at = (typeof S !== 'undefined' ? S.t : 0).toFixed(2);
  if (RN.prev && RN.act.P && RN.prev.P && rnFallback(key, msg, at)) return;
  if (L.status === 'edited') {
    L.fn = L.def.fn; L.status = 'error'; L.err = `Stopped at t = ${at} s in the step runner: ${msg}. The default is running until you apply a fix.`;
    notifyLaw(key);
  } else {
    RN.trapped[key] = `Stopped at t = ${at} s: ${msg}. Its JavaScript version is running instead.`;
    rnEvent(`${L.def.title}: ${msg}; running it in JavaScript.`, 'bad');
    rnRender();
  }
}
function rnFallback(key, msg, at) {
  const old = RN.prev, bad = RN.act;
  if (!old.P.fns[key] || !old.srcs || old.srcs[key] === RN.flying[key]) return false;   // only when this formula just changed
  rnTransfer(old.P, old.A, bad.P, bad.A);
  RN.act = old; RN.prev = null; RN.P = old.P; RN.A = old.A; RN.engine = old.kind;
  for (const k of Object.keys(RN.flying)) if (LAWS[k].status === 'edited' && RN.flying[k] === LAWS[k].src && k === key) {
    const L = LAWS[k]; L.status = 'error'; L.fn = L.def.fn; L.err = `Stopped at t = ${at} s in the step runner: ${msg}. The program before it took over again.`; notifyLawQuiet(k);
  }
  RN.flying = { ...old.srcs }; RN.srcs = { ...old.srcs };
  rnGiveBack(bad);
  rnEvent(`${LAWS[key].def.title}: ${msg} at t = ${at} s. The previous program took over.`, 'bad');
  rnRender();
  return true;
}

// Check an edited flight formula for the runner: compile it on its own, load it, and run it on the sample
// inputs. Returns '' or what's wrong.
function rnCheck(key, src) {
  if (!RN_SIGS[key]) return '';
  let P;
  try { P = rnCompileAll({ [key]: src }, RN_SIGS, { throw: true }); rnVerify(P); }
  catch (e) { return e.message; }
  const L = LAWS[key], f = P.fns[key], A = rnArena(P);
  let args; try { args = L.def.sample(); } catch (e) { return ''; }
  try {
    f.args.forEach((a, i) => { if (a.state) rnStateIn(A, f, args[i] || {}, true); else rnWrite(A, a.addr, a.t, args[i], a.name); });
  } catch (e) { return ''; }                            // the sample doesn't fit the signature: nothing to test with
  try { rnRun(P, A, key); } catch (e) { return 'Test run failed: ' + e.message; }
  const out = rnRead(A, f.ret.addr, f.ret.t);
  if (!shapeOk(out, L.def.shape, args)) return `Test run returned ${describe(out)}. It must return ${shapeText(L.def.shape)}.`;
  return '';
}

function rnSetWant(on) {
  RN.want = !!on; try { localStorage.setItem('dfb-runner', on ? 'on' : 'off'); } catch (e) { }
  if (RN.want) rnRebuild(); else rnCancelStage();
  rnRender();
}
// The program as the companion computer would send it to the drone, with self-tests from this flight (the
// small ones, so it fits a board's receive buffer: under 48 KB).
function rnDownload() {
  if (!RN.P) return;
  const samples = []; for (const k of Object.keys(RN.samples)) for (const s of RN.samples[k]) samples.push(s);
  const img = rnImage(RN.P, { tests: rnMakeTests(RN.P, samples, 600) }), blob = new Blob([img], { type: 'application/octet-stream' });
  const a = document.createElement('a'); a.href = URL.createObjectURL(blob); a.download = 'flight-formulas.rnp';
  document.body.append(a); a.click(); setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
}

// Start: compile now (the JavaScript runner can fly at once), then switch to the C runner when it's ready.
function rnOnLaw(key) { if (RN_SIGS[key]) { clearTimeout(RN.pending); RN.pending = setTimeout(() => { rnStage(); if (typeof boardsStageProgram === 'function') boardsStageProgram(); }, 30); } }
rnRebuild();
lawListeners.add(rnOnLaw);
(function startWasm() {
  if (typeof WebAssembly !== 'object' || typeof RN_WASM_B64 !== 'string') { RN.wasmErr = 'this browser has no WebAssembly'; return; }
  let bytes;
  try { bytes = Uint8Array.from(atob(RN_WASM_B64), c => c.charCodeAt(0)); } catch (e) { RN.wasmErr = e.message; return; }
  WebAssembly.compile(bytes).then(m => { RN.module = m; return Promise.all([RnWasm.fromModule(m), RnWasm.fromModule(m), RnWasm.fromModule(m)]); })
    .then(ws => { RN.pool.push(...ws); rnRebuild(); }, e => { RN.wasmErr = 'WebAssembly is blocked here (' + (e && e.message || e) + ')'; rnRender(); });
})();
