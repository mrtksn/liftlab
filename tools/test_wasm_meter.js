#!/usr/bin/env node
'use strict';
// The apps' step limit (js/wasm-meter.js) on hand-made modules: a counted loop runs to its end and uses one unit a
// turn; a loop that never ends traps when the fuel runs out; a module with no globals gets a global section in the
// right place; one that imports a global gets the counter after it; what's left of the module still works.
//   node tools/test_wasm_meter.js
const assert = require('assert');
const { wasmMeter } = require('../js/wasm-meter.js');

const leb = n => { const o = []; do { let b = n & 0x7f; n >>>= 7; if (n) b |= 0x80; o.push(b); } while (n); return o; };
const str = s => [...leb(s.length), ...Buffer.from(s)];
const section = (id, body) => [id, ...leb(body.length), ...body];
const vec = items => [...leb(items.length), ...items.flat()];
const func = (locals, code) => { const b = [...vec(locals), ...code, 0x0b]; return [...leb(b.length), ...b]; };
// (module (type (func (param i32) (result i32))) ... ) with: count(n) loops n times and returns n; forever() never ends
function module({ importGlobal = false, ownGlobal = false } = {}) {
  const types = section(1, vec([[0x60, 1, 0x7f, 1, 0x7f], [0x60, 0, 0]]));
  const imports = importGlobal ? section(2, vec([[...str('env'), ...str('g'), 3, 0x7f, 0]])) : [];
  const funcs = section(3, vec([[0], [1]]));
  const mem = section(5, vec([[0, 1]]));
  const globals = ownGlobal ? section(6, vec([[0x7f, 1, 0x41, 7, 0x0b]])) : [];
  const exports = section(7, vec([[...str('count'), 0, 0], [...str('forever'), 0, 1], [...str('memory'), 2, 0]]));
  // count: local i; loop: i++; br_if (i < n); return i — with a block, an if and a constant inside to be skipped over
  const count = func([[1, 0x7f]], [0x02, 0x40, 0x03, 0x40, 0x20, 1, 0x41, 1, 0x6a, 0x21, 1, 0x20, 1, 0x20, 0, 0x48, 0x04, 0x40, 0x44, 0, 0, 0, 0, 0, 0, 0xf0, 0x3f, 0x1a, 0x0b, 0x20, 1, 0x20, 0, 0x48, 0x0d, 0, 0x0b, 0x0b, 0x20, 1]);
  const forever = func([], [0x03, 0x40, 0x0c, 0, 0x0b]);
  const code = section(10, vec([count, forever]));
  return Uint8Array.from([0, 0x61, 0x73, 0x6d, 1, 0, 0, 0, ...types, ...imports, ...funcs, ...mem, ...globals, ...exports, ...code]);
}

for (const opts of [{}, { ownGlobal: true }, { importGlobal: true }, { importGlobal: true, ownGlobal: true }]) {
  const raw = module(opts), metered = wasmMeter(raw);
  assert.ok(WebAssembly.validate(raw), 'the hand-made module is valid');
  assert.ok(WebAssembly.validate(metered), 'the metered module is valid: ' + JSON.stringify(opts));
  const env = opts.importGlobal ? { env: { g: new WebAssembly.Global({ value: 'i32', mutable: false }, 3) } } : {};
  const x = new WebAssembly.Instance(new WebAssembly.Module(metered), env).exports;
  assert.ok(x.ll_fuel instanceof WebAssembly.Global, 'll_fuel exported');
  x.ll_fuel.value = 1000;
  assert.strictEqual(x.count(10), 10, 'the loop still counts');
  assert.strictEqual(x.ll_fuel.value, 990, 'one unit a turn');
  x.ll_fuel.value = 5;
  assert.throws(() => x.count(10), WebAssembly.RuntimeError, 'out of fuel: a trap');
  assert.strictEqual(x.ll_fuel.value, 0);
  x.ll_fuel.value = 100000;
  const t = Date.now();
  assert.throws(() => x.forever(), WebAssembly.RuntimeError, 'a loop that never ends traps');
  assert.ok(Date.now() - t < 1000, 'and soon');
  x.ll_fuel.value = 0;
  assert.throws(() => x.count(1), WebAssembly.RuntimeError, 'no fuel: the first turn traps');
}
assert.throws(() => wasmMeter(Uint8Array.from([1, 2, 3, 4, 5, 6, 7, 8])), /not WebAssembly/);
console.log('wasm meter: all passed');
