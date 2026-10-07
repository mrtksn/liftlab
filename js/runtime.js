'use strict';
// Law registry: holds the active version of every formula, compiles edits, validates results and
// falls back to the default when an edited formula throws or returns something unusable.

let LAWS = {};
for (const def of LAW_DEFS) {
  LAWS[def.key] = { def, fn: def.fn, src: def.fn.toString(), defSrc: def.fn.toString(), status: 'default', err: '' };
}
const lawListeners = new Set(); // UI callbacks: (key) => void
const notifyLaw = key => { for (const f of lawListeners) { if (typeof droneUiActive === 'function' && !droneUiActive() && f !== rnOnLaw) continue; f(key); } };

let evalAllowed = true;
try { evalAllowed = (new Function('return 1'))() === 1; } catch (e) { evalAllowed = false; }

const isNum = x => typeof x === 'number' && isFinite(x);
const isV3 = x => Array.isArray(x) && x.length === 3 && isNum(x[0]) && isNum(x[1]) && isNum(x[2]);
function shapeOk(out, shape, args) {
  if (shape === 'n') return isNum(out);
  if (shape === 3) return isV3(out);
  if (shape === 'alloc') return Array.isArray(out) && out.length === args[0].length && out.every(isNum);
  if (shape === 'vec6') return Array.isArray(out) && out.length === 6 && out.every(isNum);
  if (shape === 'mat3') return Array.isArray(out) && out.length === 9 && out.every(isNum);
  if (shape === 'pull') return !!out && ['q', 'r'].every(k => Array.isArray(out[k]) && out[k].length === args[0].length && out[k].every(isNum));
  if (shape === 'obj') return !!out && typeof out === 'object';
  if (!out || typeof out !== 'object') return false;
  for (const [k, n] of Object.entries(shape)) {
    if (n === 'n') { if (!(Array.isArray(out[k]) && out[k].every(isNum))) return false; continue; }
    if (n === 1) { if (!isNum(out[k])) return false; continue; }
    if (n === 'rows') { if (!(Array.isArray(out[k]) && out[k].length === 6 && out[k].every(r => Array.isArray(r) && r.length === args[1].length && r.every(isNum)))) return false; continue; }
    if (!(Array.isArray(out[k]) && out[k].length === n && out[k].every(isNum))) return false;
  }
  return true;
}
function shapeText(shape) {
  if (shape === 'n') return 'a finite number';
  if (shape === 3) return 'an array of 3 finite numbers';
  if (shape === 'alloc') return 'an array with one finite number per input';
  if (shape === 'vec6') return 'an array of 6 finite numbers [angular; linear]';
  if (shape === 'mat3') return 'an array of 9 finite numbers (a 3×3 matrix, row by row)';
  if (shape === 'pull') return '{ q, r }: two arrays with one finite number per input';
  if (shape === 'obj') return 'an object';
  return '{ ' + Object.entries(shape).map(([k, n]) => k + (n === 1 ? ': number' : n === 'n' ? ': [numbers]' : n === 'rows' ? ': 6 rows, one number per input' : n === 4 ? ': [w, x, y, z]' : ': [x, y, z]')).join(', ') + ' }';
}
function describe(v) {
  if (v === undefined) return 'undefined';
  try { const s = JSON.stringify(v, (k, x) => typeof x === 'number' && !isFinite(x) ? String(x) : x); return s.length > 90 ? s.slice(0, 87) + '…' : s; }
  catch (e) { return String(v); }
}

// Calls the active version of a law. If an edited version fails, it is switched off and the
// default answers instead, so the simulation keeps running.
// While the flight budget counts a control step, every call is counted (budget.js).
function run(key, ...args) {
  // Flight code (inside a control step) runs on the step runner when it's on (rn-bridge.js).
  if (OPS.on && typeof rnActive === 'function' && rnActive(key)) {
    let out; const n0 = OPS.n;
    out = budgetCall(key, args, () => rnCall(key, args), true);
    if (out !== undefined) return out;
    OPS.n = n0;                                          // it trapped: run the JavaScript version instead
  }
  return OPS.on ? budgetCall(key, args, () => runLaw(key, ...args)) : runLaw(key, ...args);
}
function runLaw(key, ...args) {
  const L = LAWS[key];
  if (L.fn !== L.def.fn) {
    try {
      const out = L.fn(...args);
      if (shapeOk(out, L.def.shape, args)) return out;
      throw new Error(`returned ${describe(out)}, expected ${shapeText(L.def.shape)}`);
    } catch (e) {
      L.fn = L.def.fn; L.status = 'error';
      L.err = `Stopped at t = ${(typeof S !== 'undefined' ? S.t : 0).toFixed(2)} s: ${e.message}. The default is running until you apply a fix.`;
      notifyLaw(key);
    }
  }
  return L.def.fn(...args);
}

function compileLaw(src) {
  if (!evalAllowed) throw new Error('This viewer does not allow running edited code. Open index.html from the repo to edit formulas.');
  let f;
  try { f = (new Function('"use strict";\nreturn (' + src + '\n);'))(); }
  catch (e) { throw new Error('Syntax error: ' + e.message); }
  if (typeof f !== 'function') throw new Error('The code must be a single function, like the default.');
  return f;
}

// Compiles, test-calls with sample inputs, and activates. Throws with a readable message on failure.
// A flight formula is also compiled for the step runner (rn-bridge.js); where this viewer can't run edited
// JavaScript, an edit that compiles still flies there.
function applyLaw(key, src) {
  const L = LAWS[key];
  if (src.trim() === L.defSrc.trim()) { resetLaw(key); return; }
  const flight = typeof RN_SIGS !== 'undefined' && !!RN_SIGS[key];
  let f = null;
  if (evalAllowed || !flight) f = compileLaw(src);
  const rnErr = flight ? rnCheck(key, src) : '';
  if (f) {
    const args = L.def.sample();
    let out;
    try { out = f(...args); } catch (e) { throw new Error('Test call failed: ' + e.message); }
    if (!shapeOk(out, L.def.shape, args)) throw new Error(`Test call returned ${describe(out)}. It must return ${shapeText(L.def.shape)}.`);
  } else if (rnErr) throw new Error(rnErr);
  L.fn = f || L.def.fn; L.src = src; L.status = 'edited'; L.err = ''; L.rnErr = rnErr; L.runnerOnly = !f;
  notifyLaw(key);
}
function resetLaw(key) {
  const L = LAWS[key];
  L.fn = L.def.fn; L.src = L.defSrc; L.status = 'default'; L.err = ''; L.rnErr = ''; L.runnerOnly = false;
  notifyLaw(key);
}
const editedLaws = () => Object.values(LAWS).filter(L => L.src.trim() !== L.defSrc.trim());
// The formulas are part of a design: its edited ones, by key (the rest at their defaults).
const lawSet = () => Object.fromEntries(editedLaws().map(L => [L.def.key, L.src]));
// Put a design's formulas in place: each it names, edited as it says (one that won't compile is kept, marked as an
// error, the default running); every other back to its default. Quietly: one design change is one undo step.
function setLaws(map) {
  map = map || {};
  const quiet = typeof undo !== 'undefined', was = quiet && undo.restoring; if (quiet) undo.restoring = true;
  try {
    for (const [key, L] of Object.entries(LAWS)) {
      const want = typeof map[key] === 'string' ? map[key] : null;
      if (want == null) { if (L.src.trim() !== L.defSrc.trim() || L.status !== 'default') resetLaw(key); continue; }
      if (want.trim() === L.src.trim() && L.status !== 'error') continue;
      try { applyLaw(key, want); } catch (e) { L.fn = L.def.fn; L.src = want; L.status = 'error'; L.err = e.message; notifyLaw(key); }
    }
  } finally { if (quiet) undo.restoring = was; }
  if (typeof refreshLawCard === 'function') for (const key of Object.keys(LAWS)) refreshLawCard(key);
}
