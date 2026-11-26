#!/usr/bin/env node
'use strict';
// The scale thrust test's maths (js/thrust-test.js): readings made from a known curve, with a scale's rounding and a
// sagging pack, give the curve back; the physical motor's prop coefficient found from readings makes it give them;
// the readings it refuses; typed readings.
const assert = require('assert'), fs = require('fs'), vm = require('vm'), path = require('path');
const ctx = vm.createContext({ console, Math, Number, Array, Object, JSON, Error });
for (const f of ['flight-physics.js', 'thrust-test.js']) vm.runInContext(fs.readFileSync(path.join(__dirname, '..', 'js', f), 'utf8'), ctx, { filename: f });
const { ThrustTest: T, FlightPhysics: P } = vm.runInContext('({ ThrustTest, FlightPhysics })', ctx);
const g = 9.80665;
let seed = 11; const rnd = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647 - 0.5; };
const close = (a, b, tol, what) => assert(Math.abs(a - b) <= tol, `${what}: ${a} vs ${b} (±${tol})`);

// typed readings
assert.strictEqual(T.parseGrams('312'), 312);
assert.strictEqual(T.parseGrams(' 312 g '), 312);
assert.strictEqual(T.parseGrams('0,312 kg'), 312);
close(T.parseGrams('1 oz'), 28.35, 0.01, 'ounces');
assert.strictEqual(T.parseGrams('-12'), -12);
assert.strictEqual(T.parseGrams('abc'), null);
assert.strictEqual(T.parseGrams(''), null);
assert.strictEqual(JSON.stringify(T.levelsUpTo(1)), JSON.stringify(T.LEVELS));
assert.strictEqual(JSON.stringify(T.levelsUpTo(0.7)), '[0.2,0.35,0.5,0.65,0.7]');

// A curve with a 1 g scale and a pack that sags as the throttle rises: the fit gives it back, at the reference voltage.
for (const [tmax, bend, sign] of [[0.9, 0.7, 1], [1.25, 0.4, -1], [0.5, 0.95, 1]]) {
  const vref = 4, pts = T.LEVELS.map(u => {
    const V = 4.1 - 0.35 * u, e = u * V / vref, N = tmax * ((1 - bend) * e + bend * e * e);
    return { u, V, grams: sign * Math.round(N / g * 1000 + 0.6 * rnd()) };
  });
  const f = T.fitCurve(pts, vref);
  assert(!f.error, f.error);
  close(f.tmax, tmax, 0.015 * tmax, `Tmax of ${tmax} N`);
  close(f.bend, bend, 0.06, `bend of ${bend}`);
  assert(!f.warn, `no warning for a clean curve (${f.warn})`);
  close(T.thrustAt(f, 0.5, 3.9), tmax * ((1 - bend) * 0.5 * 3.9 / 4 + bend * (0.5 * 3.9 / 4) ** 2), 0.01, 'thrust at a throttle');
  // hover: four such motors holding 290 g
  const W = 0.29 * g, h = T.hoverThrottle([f, f, f, f], W, 3.9);
  if (4 * tmax * (3.9 / 4) ** 2 > W) close(4 * T.thrustAt(f, h, 3.9), W, 1e-3, 'hover throttle holds the weight');
}
// a dead band: the lowest throttle doesn't spin it yet; not fitted, not a problem
{ const pts = [{ u: 0.2, V: 4, grams: 0 }, { u: 0.35, V: 4, grams: 25 }, { u: 0.5, V: 4, grams: 45 }, { u: 0.65, V: 4, grams: 68 }, { u: 0.8, V: 4, grams: 92 }];
  const f = T.fitCurve(pts, 4); assert(!f.error, f.error); assert.strictEqual(f.used, 4); }
// refusals
assert(T.fitCurve([{ u: 0.5, V: 4, grams: 40 }, { u: 0.6, V: 4, grams: 50 }], 4).error, 'two readings');
assert(T.fitCurve([{ u: 0.5, V: 4, grams: 40 }, { u: 0.55, V: 4, grams: 44 }, { u: 0.6, V: 4, grams: 50 }], 4).error, 'too little span');
assert(T.fitCurve(T.LEVELS.map(u => ({ u, V: 4, grams: 0 })), 4).error, 'nothing spun');
assert(T.fitCurve(T.LEVELS.map((u, i) => ({ u, V: 4, grams: 120 - 20 * i })), 4).error || T.fitCurve(T.LEVELS.map((u, i) => ({ u, V: 4, grams: 120 - 20 * i })), 4).warn, 'falling readings');
{ const pts = T.LEVELS.map(u => ({ u, V: 4, grams: Math.round(90 * u * u) })); pts[3].grams = 10;   // a mistyped one
  const f = T.fitCurve(pts, 4); assert(f.error || f.warn, 'a reading far off the curve is flagged'); }

// The physical motor: readings from one with a known prop coefficient give that coefficient back.
const card = { tmax: 1, prop: 0.065, fm: 0.6, mass: 0.006, motorPhysics: { kind: 'brushless', kv: 7000, resistance: 0.2, currentLimit: 8, escEfficiency: 0.9, idleW: 0.01, friction: 0.0003 }, propPhysics: { ct: 0.1 } };
const thrust = (ct, u, V) => P.equilibrium(P.motor({ ...card, propPhysics: { ct } }, 4), u, V, P.RHO).T;
for (const ct of [0.06, 0.1, 0.16]) {
  const pts = T.LEVELS.map(u => { const V = 4.05 - 0.3 * u; return { u, V, grams: Math.round(thrust(ct, u, V) / g * 1000) }; });
  const fit = T.fitPhysics(pts, thrust);
  assert(fit && !fit.atBound, `prop coefficient ${ct} found`);
  close(fit.ct, ct, 0.06 * ct, `prop coefficient ${ct}`);
  assert(fit.rel < 0.03, `it matches the readings (${(fit.rel * 100).toFixed(1)}%)`);
}
console.log('thrust test: ok');
