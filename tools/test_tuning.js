#!/usr/bin/env node
// The controller's tuning as compile-time constants (laws.js TUNE, rn-compile.js opts.consts) and the Airframe
// tab's model of it (tuning.js): the default tuning compiles to the program the formulas made with their numbers
// written in, a new tuning changes only constants, bad constants are refused, the gains map onto response, damping
// and integral and back, and the step prediction tells a settled loop from one that won't settle.
//   node tools/test_tuning.js
'use strict';
const fs = require('fs'), path = require('path'), vm = require('vm'), assert = require('assert');
const { defaultSources, defaultConsts } = require('./lib');
const img = (srcs, consts) => { const P = rnCompileAll(srcs, RN_SIGS, { consts, throw: true }); rnVerify(P); return Buffer.from(rnImage(P)); };

const srcs = defaultSources(), D = defaultConsts(), T = D.TUNE;
// The same formulas with the default numbers written in, as they were before the tuning.
const literal = { ...srcs,
  attitudeControl: srcs.attitudeControl.replace('kR = TUNE.att.kR, kW = TUNE.att.kW, kI = TUNE.att.kI', `kR = [${T.att.kR}], kW = [${T.att.kW}], kI = [${T.att.kI}]`),
  positionControl: srcs.positionControl.replace('kp = TUNE.pos.kp, kd = TUNE.pos.kd, ki = TUNE.pos.ki', `kp = ${T.pos.kp}, kd = ${T.pos.kd}, ki = ${T.pos.ki}`) };
assert(literal.attitudeControl !== srcs.attitudeControl && literal.positionControl !== srcs.positionControl, 'the formulas no longer read TUNE as this test expects');
const base = img(literal, {});
assert(img(srcs, D).equals(base), 'the default tuning compiles to a different program');
const hot = JSON.parse(JSON.stringify(D)); hot.TUNE.att.kR = [130, 130, 40]; hot.TUNE.pos.kd = 3;
const b2 = img(srcs, hot);
assert(!b2.equals(base), 'a new tuning should change the program');
assert(Math.abs(b2.length - base.length) <= 16, 'a new tuning should change constants, not code');   // (constants are pooled: one equal to another already there takes no place)
console.log('default tuning: the same program as the numbers written in; a new tuning changes its constants');

const refuse = (consts, src, re) => assert.throws(() => img({ positionControl: src || srcs.positionControl }, consts), re);
refuse({}, null, /Unknown name "TUNE"/);
refuse({ TUNE: { att: T.att, pos: { kp: 4, kd: 3.6 } } }, null, /TUNE\.pos has no "ki"/);
refuse({ TUNE: { att: T.att, pos: { kp: 4, kd: 'x', ki: 1 } } }, null, /TUNE\.pos\.kd isn't a number/);
refuse(D, srcs.positionControl.replace('TUNE.pos.kp', 'TUNE.pos[kp]'), /TUNE\.pos has no such field/);
console.log('missing, misspelt and non-numeric constants are refused with their name');

// tuning.js against a pretend design (it reads cfg and the formulas in use).
const J = path.join(__dirname, '..', 'js'), ctx = vm.createContext({ console, Math, JSON, Number, Array, Object });
for (const f of ['math.js', 'laws.js']) vm.runInContext(fs.readFileSync(path.join(J, f), 'utf8'), ctx, { filename: f });
vm.runInContext('var cfg = { comps: [{ type: "motor", tau: 0.03 }] }; var RN_SIGS = {}; var rnReadsTune = () => false; var rnSourceOf = () => "";', ctx);
vm.runInContext(fs.readFileSync(path.join(J, 'tuning.js'), 'utf8'), ctx, { filename: 'tuning.js' });
const r = vm.runInContext(`(() => {
  const d = tuneFix(null), out = { fixDefault: tuneSame(d, TUNE_DEFAULTS) };
  out.clamped = tuneFix({ att: { kR: [1e9, 'x', -5] }, pos: { kd: 0 } });
  const f = tuneFeel(d, 'att', 0), back = tuneWithFeel(d, 'att', [0, 1], f);
  out.feel = f; out.round = back.att.kR.map((k, i) => Math.abs(k - d.att.kR[i])).concat(back.att.kW.map((k, i) => Math.abs(k - d.att.kW[i])));
  out.posFeel = tuneFeel(d, 'pos');
  out.pred = ['att', 'pos'].map(l => { const p = tunePredict(d, l, 0, 0.03); return { stable: p.stable, os: p.overshoot, settle: p.settle, n: p.t.length }; });
  const wild = tuneWithFeel(d, 'att', [0, 1], { hz: 6, zeta: 0.3, integ: 0.8 }); out.wild = tunePredict(wild, 'att', 0, 0.03).stable;
  out.wildVerdict = tuneVerdict(tunePredict(wild, 'att', 0, 0.03), tuneFeel(wild, 'att', 0)).tone;
  out.slowMotors = tunePredict(d, 'att', 0, 0.2).stable;
  return JSON.parse(JSON.stringify(out)); })()`, ctx);
assert(r.fixDefault, 'tuneFix(null) is not the default');
assert.deepStrictEqual(r.clamped.att.kR, [2500, 100, 1], 'out-of-range or missing gains not fixed');
assert.strictEqual(r.clamped.pos.kd, 0.1, 'kd must stay above 0 (positionControl divides by it)');
assert(Math.abs(r.feel.hz - 10 / (2 * Math.PI)) < 1e-9 && Math.abs(r.feel.zeta - 0.8) < 1e-9 && Math.abs(r.feel.integ - 0.8) < 1e-9, 'default roll feel');
assert(Math.max(...r.round) < 1e-9, 'feel → gains does not give the gains back');
assert(Math.abs(r.posFeel.hz - 2 / (2 * Math.PI)) < 1e-9 && Math.abs(r.posFeel.zeta - 0.9) < 1e-9, 'default position feel');
for (const p of r.pred) assert(p.stable && p.os < 0.3 && p.n > 20, 'default tuning predicted unsettled: ' + JSON.stringify(p));
assert(!r.wild && r.wildVerdict === 'bad', '6 Hz with ζ 0.3 on 30 ms motors should be predicted unstable');
assert(!r.slowMotors, 'the default roll loop on 200 ms motors should be predicted unstable');
console.log('gains ↔ response, damping and integral round-trip; bounds hold; the prediction separates settled from unstable');
