#!/usr/bin/env node
'use strict';
// The sensor calibration's maths (js/sensor-cal.js): poses made from a known mount, offsets and gains, with noise,
// give them back; the poses it refuses; the drone's answers.
const assert = require('assert'), fs = require('fs'), vm = require('vm'), path = require('path');
const ctx = vm.createContext({ console, Math, Number, Array, Object, JSON, Error });
for (const f of ['math.js', 'sensor-cal.js']) vm.runInContext(fs.readFileSync(path.join(__dirname, '..', 'js', f), 'utf8'), ctx, { filename: f });
const { SensorCal: C, eulerR, eulerFromR, G } = vm.runInContext('({ SensorCal, eulerR, eulerFromR, G })', ctx);

let seed = 7; const rnd = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647 - 0.5; };
const mul = (M, v) => [0, 1, 2].map(i => M[3 * i] * v[0] + M[3 * i + 1] * v[1] + M[3 * i + 2] * v[2]);
const T = M => [M[0], M[3], M[6], M[1], M[4], M[7], M[2], M[5], M[8]];
// The body's up in each pose (body axes). Level and nose up define the mount, so they're square (a tilted "level" is
// a tilted level, whatever the maths); the others as a person would hold them, not quite.
const UP = { flat: [0, 0, 1], nose: [0.8, 0, 0.6], tail: [-0.9, -0.04, 0.42], left: [0.03, -0.95, 0.3], right: [-0.05, 0.93, 0.35], upside: [0.04, 0.02, -1], again: [0.004, -0.006, 1] };
const unit = a => { const n = Math.hypot(...a); return a.map(x => x / n); };
// What the chip reads in a pose: the body's up turned into its axes (mount: sensor → body), then its error.
function poses(mount, bias, scale, gyro, ids = Object.keys(UP), noise = 0.004) {
  const out = {};
  for (const id of ids) {
    const truth = mul(T(mount), unit(UP[id]).map(x => x * G));
    out[id] = { acc: truth.map((x, i) => x / scale[i] + bias[i] + noise * rnd()), gyro: gyro.map(x => x + 1e-4 * rnd()) };
  }
  return out;
}
const angle = (A, B) => { let t = 0; for (let i = 0; i < 3; i++) t += A[i] * B[i] + A[3 + i] * B[3 + i] + A[6 + i] * B[6 + i]; return Math.acos(Math.max(-1, Math.min(1, (t - 1) / 2))) * 180 / Math.PI; };

// This drone's card: about face down (rolled over), turned a little; a Z offset of −0.11 g, gains off a few %.
for (const [name, mount] of [['face down', eulerR(178, -4, 3)], ['turned 90°', eulerR(0, 0, 90)], ['on its edge', eulerR(-102, -79, 30)], ['level', eulerR(0, 0, 0)]]) {
  const bias = [0.12, -0.2, -1.08], scale = [1.02, 0.97, 1.004], gyro = [-0.0576, -0.0101, 0.0173];
  const r = C.solve(poses(mount, bias, scale, gyro));
  assert(!r.error, `${name}: ${r.error}`);
  assert(r.fitted && r.ok, `${name}: fitted ${r.fitted}, worst ${r.worst.err}`);
  assert(angle(r.mount, mount) < 0.5, `${name}: mount off by ${angle(r.mount, mount)}°`);
  for (let i = 0; i < 3; i++) {
    assert(Math.abs(r.bias[i] - bias[i]) < 0.02, `${name}: offset ${i} ${r.bias[i]} vs ${bias[i]}`);   // (2 mg)
    assert(Math.abs(r.scale[i] - scale[i]) < 0.003, `${name}: gain ${i} ${r.scale[i]} vs ${scale[i]}`);
    assert(Math.abs(r.gyro[i] - gyro[i]) < 2e-4, `${name}: gyro ${i}`);
  }
  const e = eulerFromR(r.mount), back = eulerR(e[0], e[1], e[2]);
  assert(angle(back, r.mount) < 1e-4, `${name}: eulerFromR round trip`);
}
// Straight up or down in pitch: eulerFromR still round-trips.
for (const R of [eulerR(0, 90, 40), eulerR(0, -90, -120)]) { const e = eulerFromR(R); assert(angle(eulerR(e[0], e[1], e[2]), R) < 1e-4); }

// Only the two poses the mount needs: the mount, and one gain that makes level read 1 g.
{ const mount = eulerR(178, -4, 3), r = C.solve(poses(mount, [0, 0, -1.08], [1, 1, 1], [0, 0, 0], ['flat', 'nose']));
  assert(!r.error && !r.fitted && /one gain/.test(r.note) && angle(r.mount, mount) < 1.5);
  assert(Math.abs(r.scale[0] - r.scale[2]) < 1e-12 && Math.abs(r.bias[2]) === 0); }
// Refused: a missing pose, a nose barely lifted, a pose taken the wrong way round.
{ const mount = eulerR(0, 0, 0), P = poses(mount, [0, 0, 0], [1, 1, 1], [0, 0, 0]);
  assert(/Nose up/.test(C.solve({ flat: P.flat }).error));
  const low = { ...P, nose: { acc: mul(T(mount), unit([0.15, 0, 1]).map(x => x * G)), gyro: [0, 0, 0] } };
  assert(/only \d+°/.test(C.solve(low).error));
  assert(/the wrong way round/.test(C.solve({ ...P, left: P.right }).error)); }
// A level pose that moved (it reads 1.1 g, the other level pose 1 g) shows as not ok. (A side pose has no twin: the
// fit takes it as it is, and the drone's own stillness check is what refuses one that moved.)
{ const mount = eulerR(10, 5, 0), P = poses(mount, [0, 0, 0], [1, 1, 1], [0, 0, 0]);
  P.flat.acc = P.flat.acc.map(x => x * 1.1); const r = C.solve(P); assert(!r.error && !r.ok, `a moved pose isn't ok: ${r.error || r.worst.err}`);
  const Q = poses(mount, [0, 0, 0], [1, 1, 1], [0, 0, 0]); Q.again = { acc: mul(T(mount), unit([0.1, 0, 1]).map(x => x * G)), gyro: [0, 0, 0] };
  assert(/level poses differ/.test(C.solve(Q).error)); }
// The guide's check: level takes anything still; nose up wants 30°; the others their own way, from the mount so far.
{ const mount = eulerR(178, -4, 93), P = poses(mount, [0.12, -0.2, -1.08], [1.02, 0.97, 1.004], [0, 0, 0]);
  const at = (u, bias = [0.12, -0.2, -1.08]) => ({ acc: mul(T(mount), unit(u).map(x => x * G)).map((x, i) => x + bias[i]), gyro: [0, 0, 0] });
  assert(C.check({}, 'flat', P.flat).ok && !C.check({}, 'nose', P.nose).ok);
  const f = { flat: P.flat }, slight = C.check(f, 'nose', at([0.3, 0, 1]));
  assert(!slight.ok && /lift the front more: 1\d°/.test(slight.msg), slight.msg);
  assert(C.check(f, 'nose', P.nose).ok);
  const fn = { flat: P.flat, nose: P.nose };
  assert(C.check(fn, 'tail', P.tail).ok && C.check(fn, 'left', P.left).ok && C.check(fn, 'right', P.right).ok && C.check(fn, 'upside', P.upside).ok);
  assert(/other way/.test(C.check(fn, 'left', P.right).msg) && /further/.test(C.check(fn, 'upside', P.left).msg));
  assert(!C.check(fn, 'tail', at([-0.3, 0, 1])).ok && C.check(fn, 'again', P.flat).ok && !C.check(fn, 'again', at([0.1, 0, 1])).ok); }
// The drone's answers.
assert.strictEqual(JSON.stringify(C.parseAnswer('esp32: imucal -675 -1915 8612 -571 -101 173')), JSON.stringify({ acc: [-0.675, -1.915, 8.612], gyro: [-0.0571, -0.0101, 0.0173] }));
assert.strictEqual(JSON.stringify(C.parseAnswer('imucal: moving: keep it still and measure again')), JSON.stringify({ error: 'moving: keep it still and measure again' }));
assert.strictEqual(C.parseAnswer('to the ESP32: imucal (its answer: status, or the log)'), null);
assert.strictEqual(C.parseAnswer('esp32: disarm first').error, 'disarm it first');
console.log('Sensor calibration tests passed: mount face down, turned, on its edge and level within 0.5°, offsets within 2 mg, gains, the gyro offset, eulerFromR round trips, two poses alone, refusals, a pose that moved, the two level poses, the guide\'s check, the drone\'s answers.');
