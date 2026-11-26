#!/usr/bin/env node
'use strict';
// Safe gains from the motors (js/tune-suggest.js): slower motors and more uncertainty give slower, still safe gains;
// the default gains are flagged on slow motors; every suggestion keeps its margins over the whole strength range.
const assert = require('assert'), fs = require('fs'), vm = require('vm'), path = require('path');
const ctx = vm.createContext({ console, Math, Number, Array, Object, JSON, Error });
for (const f of ['autotune.js', 'tune-suggest.js']) vm.runInContext(fs.readFileSync(path.join(__dirname, '..', 'js', f), 'utf8'), ctx, { filename: f });
const { TuneSuggest: S, atMargins } = vm.runInContext('({ TuneSuggest, atMargins })', ctx);
const DEF = { rp: [100, 16, 80], yaw: [40, 10, 20] };
const model = (lag, delay, measured) => ({ lag, delay, strengths: S.strengthsFor(measured) });

// quick motors: the defaults keep their margin, and are kept (never sped up)
{ const m = model(0.025, 0.008, false), j = S.judge(DEF.rp, m); assert(j.safe, `defaults on quick motors are within the limits (${j.nominal}, ${j.margin})`);
  const k = S.suggest('rp', DEF.rp, m); assert(k.keep && k.gains[0] === 100, 'kept as they are'); }
// slow toy motors with a guessed strength: the defaults aren't, and the suggestion is slower but safe at every strength
const slow = model(0.08, 0.012, false), sNow = S.judge(DEF.rp, slow);
assert(!sNow.safe, 'defaults on slow, unmeasured motors are flagged');
assert(sNow.at === 1.5, `the worst case is motors stronger than described (${sNow.at})`);
const sug = S.suggest('rp', DEF.rp, slow);
assert(sug, 'a safe suggestion exists');
assert(sug.hz < S.feelOf(DEF.rp).hz, `slower than the default (${sug.hz.toFixed(2)} Hz)`);
for (const s of slow.strengths) { const m = atMargins(sug.gains, { gain: s, lag: slow.lag, delay: slow.delay, lead: 0 }); assert(m.margin >= (s === 1 ? 30 : 20), `margin at strength ${s}: ${m.margin}`); }
assert(!sug.keep && !sug.firm, 'a plain suggestion');
assert(sug.integ <= 0.8 && sug.zeta >= 0.8 && sug.zeta <= 1.1, 'integral and damping within their ranges');
// measuring the motors (a narrower strength range) allows a quicker response; slower motors a slower one
const meas = S.suggest('rp', DEF.rp, model(0.08, 0.012, true)), slower = S.suggest('rp', DEF.rp, model(0.12, 0.012, false));
assert(meas.hz >= sug.hz, `measured motors: at least as quick (${meas.hz} vs ${sug.hz})`);
assert(slower && slower.hz < sug.hz, `slower motors: slower gains (${slower && slower.hz} vs ${sug.hz})`);
// yaw: its own range
const y = S.suggest('yaw', DEF.yaw, slow); assert(y && y.hz >= 0.2 && y.hz <= 3, 'yaw suggestion within its range');
// a floor: a drone that needs a firmer roll and pitch than is fully safe gets the firm one, marked, with its margin
{ const floor = S.firmHz(4.5); assert(floor > 0.6 && floor < 0.8, `firmness for a 4.5 rad/s² push: ${floor}`);
  const m = { ...model(0.12, 0.012, false), firm: { rp: 0.85 } }, f = S.suggest('rp', DEF.rp, m), q0 = S.quickestSafe('rp', DEF.rp, m);
  assert(f.firm && f.hz >= 0.85 && f.safeHz === (q0 && q0.hz) && f.safeHz < 0.85, `marked firm at ${f.hz} (safe up to ${f.safeHz})`);
  assert(f.nominal < 30, `its margin is said (${f.nominal})`);
  const q = S.suggest('rp', DEF.rp, { ...model(0.025, 0.008, false), firm: { rp: floor } }); assert(!q.firm && q.hz > floor, 'quick motors: no compromise'); }
// the round trip of the feel
const f = S.feelOf(S.gainsOf(1.2, 0.9, 0.5)); assert(Math.abs(f.hz - 1.2) < 1e-9 && Math.abs(f.zeta - 0.9) < 1e-9 && Math.abs(f.integ - 0.5) < 1e-9);
console.log(`tune suggest: ok (slow guessed ${sug.hz.toFixed(2)} Hz ζ ${sug.zeta}, margin ${sug.margin.toFixed(0)}°; measured ${meas.hz.toFixed(2)} Hz; defaults' worst margin ${sNow.margin == null ? 'none' : sNow.margin.toFixed(0) + '°'})`);
