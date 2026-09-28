// Loads the simulator's formulas (js/math.js, js/laws.js) and the step compiler and runner (js/rn-*.js) into
// node, for the tools in this folder.
'use strict';
const vm = require('vm'), fs = require('fs'), path = require('path'), zlib = require('zlib');
const JS = path.join(__dirname, '..', 'js');
const ctx = vm.createContext({ console, Math, Float32Array, Float64Array, Int32Array, Uint8Array, Array, Object, Number, JSON, Map, Set, Error, structuredClone, performance: { now: () => 0 } });
for (const f of ['math.js', 'laws.js']) vm.runInContext(fs.readFileSync(path.join(JS, f), 'utf8') + '\n;', ctx, { filename: f });
for (const k of ['bls', 'qmat', 'matToQuat']) global[k] = vm.runInContext(k, ctx);
for (const f of ['rn-parse.js', 'rn-ops.js', 'rn-compile.js', 'rn-sigs.js', 'rn-vm.js']) Object.assign(global, require(path.join(JS, f)));
// The source of a formula as the simulator has it (laws.js), or null.
const lawSource = key => vm.runInContext(`(LAW_DEFS.find(d => d.key === ${JSON.stringify(key)}) || { fn: null }).fn`, ctx)?.toString() ?? null;
const lawSample = key => vm.runInContext(`(() => { const d = LAW_DEFS.find(d => d.key === ${JSON.stringify(key)}); return d && d.sample ? d.sample() : null; })()`, ctx);
const defaultSources = () => { const s = {}; for (const k of Object.keys(RN_SIGS)) s[k] = lawSource(k); return s; };
// Real calls recorded in simulated flights (every layout, a calibration, a throw, optical flow): { key: [{ args, ret, stAfter }] }.
const golden = () => JSON.parse(zlib.gunzipSync(fs.readFileSync(path.join(__dirname, 'golden.json.gz'))).toString());
// The C runner built to WebAssembly (runner/build_wasm.sh), if it's there.
async function wasmRunner() {
  const f = path.join(__dirname, '..', 'runner', 'runner.wasm');
  if (!fs.existsSync(f)) return null;
  return RnWasm.create(fs.readFileSync(f));
}
module.exports = { lawSource, lawSample, defaultSources, golden, wasmRunner };
