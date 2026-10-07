'use strict';
// Learning the airframe: the simulator's side. The learning itself is a task on a flight computer (boards.js, the
// learning task: runner/fc/learn_core.c on a Pi): in-flight learning, the calibration and the throw start all run
// there, in C, on the formulas. This file has what the simulator needs around it:
//   - the inputs and the description's columns (what the airframe export and the learning task start from);
//   - how well what the task learned matches the real drone (only the simulator knows that);
//   - the throwing hand, which is physics (the task only finds out it was thrown when it starts to fall).
//
// Inputs are thrust fractions 0–1. A motor on the frame is one input. A motor on servo joints is several:
// its thrust times each product of (1, cos θ, sin θ) over the joints above it (3 for one joint, 9 for two).
// Turning a rigid part about a hinge is linear in cos θ and sin θ, so this is exact, and the motor's effect
// at any joint angles is a fixed sum of learned columns.

let learn = {
  index: new Map(), n: 0, sig: '',
  view: null,         // the learning task's status, as its board reports it (boards.js boardsLearnView)
  msg: '',
};

/* ───────── inputs ───────── */
// Basis over a motor's joints (nearest first): products of (1, cos θ, sin θ), first joint most significant.
function basisVals(angles) { let v = [1]; for (const a of angles) { const c = Math.cos(a), s = Math.sin(a); v = v.flatMap(x => [x, x * c, x * s]); } return v; }
// Turns a motor's effect, evaluated at joint angles {0, π/2, π} for each of its k joints, into its basis columns.
function decompose(evalAt, k) {
  const n = 3 ** k, A = [0, Math.PI / 2, Math.PI], V = [];
  for (let idx = 0; idx < n; idx++) {
    const ang = []; let r = idx; for (let i = k - 1; i >= 0; i--) { ang[i] = A[r % 3]; r = Math.floor(r / 3); }
    V.push(evalAt(ang));
  }
  for (let m = 0; m < k; m++) {   // per joint: v(0) = C0 + C1, v(π/2) = C0 + C2, v(π) = C0 − C1
    const stride = 3 ** (k - 1 - m);
    for (let b = 0; b < n; b++) {
      if (Math.floor(b / stride) % 3 !== 0) continue;
      const v0 = V[b], v1 = V[b + stride], v2 = V[b + 2 * stride];
      const c0 = v0.map((x, i) => (x + v2[i]) / 2), c1 = v0.map((x, i) => (x - v2[i]) / 2), c2 = v1.map((x, i) => x - c0[i]);
      V[b] = c0; V[b + stride] = c1; V[b + 2 * stride] = c2;
    }
  }
  return V;
}
function inputSig() { return actuators().map(c => c.id + ':' + chainOf(c).map(j => j.id).join('.')).join(','); }
function buildIndex() {
  learn.index = new Map(); let j = 0;
  for (const c of actuators()) { const k = chainOf(c).length, n = 3 ** k; learn.index.set(c.id, { cols: Array.from({ length: n }, (_, i) => j + i), k }); j += n; }
  learn.n = j; learn.sig = inputSig();
}
const seenAngles = c => chainOf(c).map(angleSeen);

/* ───────── the description's columns (acceleration per full thrust, body frame) ───────── */
const cfgToAccel = col => { const f = scl([col[0], col[1], col[2]], 1 / model.m); const a = m3v(model.Jinv, [col[3], col[4], col[5]]); return [...f, ...a]; };
function describedAt(c, angles) {   // from the airframe description, with the motor's joints at the given angles
  const ch = chainOf(c), n = rotorNow(c, j => angles[ch.indexOf(j)]);
  return cfgToAccel(scl6(wrenchCol(n.p, n.d, spinOf(c), kappaOf(c), model.c), c.tmax * hModel(c)));
}
// Cached per model: the description only changes when the believed mass properties do (a new `model`).
let descCache = new WeakMap();
const invalidateDesc = () => { descCache = new WeakMap(); };
function describedCols(c) {
  let m = descCache.get(model); if (!m) { m = new Map(); descCache.set(model, m); }
  let cols = m.get(c.id); if (!cols) { cols = decompose(a => describedAt(c, a), chainOf(c).length); m.set(c.id, cols); }
  return cols;
}
// The flight code is told the description; what it learns stays on its board.
const colsFor = describedCols;
function sumCols(cols, w) { const out = [0, 0, 0, 0, 0, 0]; cols.forEach((col, b) => { if (w[b]) for (let i = 0; i < 6; i++) out[i] += w[b] * col[i]; }); return out; }
const colAtAngles = (c, angles) => sumCols(colsFor(c), basisVals(angles));
const colAt = c => colAtAngles(c, seenAngles(c));
const add6 = (a, b) => a.map((x, i) => x + b[i]);
const ctlModel = () => model;
const ctlAxis = () => nb;
const flyingLearned = () => !!(learn.view && learn.view.useLearned);
// What the description says about the actuators' responses (the tests' results go to the flight core directly).
const believedThrust = (u, k) => (1 - k) * u + k * u * u;
const BEND_PRIOR = 0.7;   // a typical brushless prop curve (thrust grows faster than throttle); pitch control is close to linear
const curveHat = c => c.pitch === 'collective' ? 0 : BEND_PRIOR;
const motorLagHat = () => 0.035;
function servoModelHat(j) { const v = learn.view, k = joints().indexOf(j), r = v && v.joints[k]; return r && r.measured ? { rate: r.rate, lag: r.lag } : { rate: j.rate * D2R, lag: 0 }; }
function resetLearning() { buildIndex(); learn.view = null; learn.msg = ''; }
const calServo = () => null;

/* ───────── how well it matches the real drone ───────── */
function learnedCols(c) {
  const v = learn.view, ix = learn.index.get(c.id); if (!v || !v.B || !ix || v.n !== learn.n) return null;
  return ix.cols.map(jj => v.B.map(row => row[jj]));
}
// Each motor's true basis columns, from the real geometry, mass, inertia, health and battery (no airflow).
function trueAt(c, angles) {
  const ch = chainOf(c), n = rotorNow(c, j => angles[ch.indexOf(j)]);
  const col = scl6(wrenchCol(n.p, n.d, spinOf(c), kappaOf(c), truth.c), c.tmax * motorEff(c) * S.battK);
  const f = scl([col[0], col[1], col[2]], 1 / truth.m), a = m3v(truth.Jinv, [col[3], col[4], col[5]]);
  return [...f, ...a];
}
function trueB() { const out = new Map(); for (const c of actuators()) out.set(c.id, decompose(a => trueAt(c, a), chainOf(c).length)); return out; }
// How well each motor's learned effect matches the truth right now (1 = exact). Compared as the effect itself at
// joint angles across each steering joint's range (both ends and the middle), not column by column: over a ±30°
// range cos θ barely changes, so how the "1" and "cos θ" columns split is unknowable and doesn't matter.
function matchScores() {
  const acts = actuators(); if (!acts.length || !learn.view || !learn.view.B) return [];
  const sj = steerJoints();
  const samples = c => {
    const ch = chainOf(c); let sets = [ch.map(angleTrue)];
    ch.forEach((j, m) => { if (sj.includes(j)) sets = sets.flatMap(a => [-1, 0, 1].map(k => a.map((x, i) => i === m ? k * j.range * D2R : x))); });
    return sets;
  };
  const tb = trueB();
  const pairs = acts.map(c => { const T = tb.get(c.id), L = learnedCols(c); return L ? samples(c).map(a => { const phi = basisVals(a); return [sumCols(L, phi), sumCols(T, phi)]; }) : []; });
  const rowMax = i => { let m = 0; for (const P of pairs) for (const [, t] of P) m = Math.max(m, Math.abs(t[i])); return m; };
  const groups = [[0, 1, 2], [3, 4], [5]], scale = new Array(6);
  for (const g of groups) { const m = Math.max(1e-6, ...g.map(rowMax)); for (const i of g) scale[i] = 1 / m; }
  return acts.map((c, k) => {
    let e = 0, v = 0;
    for (const [l, t] of pairs[k]) for (let i = 0; i < 6; i++) { e += ((l[i] - t[i]) * scale[i]) ** 2; v += (t[i] * scale[i]) ** 2; }
    return { c, match: pairs[k].length ? clamp(1 - Math.sqrt(e / Math.max(v, 1e-9)), 0, 1) : 0 };
  });
}
function trueImuOffset() {   // mean IMU position relative to the true CoG, body frame (for the report only)
  const imus = sensorsOf('imu'); if (!imus.length) return [0, 0, 0];
  return sub(mean3(imus.map(posNow)), truth.c);
}

/* ───────── the throwing hand (physics) ───────── */
// The throw start (after Blaha, Smeur & Remes, TU Delft, 2024): the drone is held in the hand, armed with its motors
// off; the learning task is told a throw is coming. The hand then swings it up to speed and spin and lets go. From
// there the learning task flies it: it notices the free fall, pulses each motor, fits its model and catches itself.
let throwCfg = { height: 7, spin: 6, thenCalibrate: true, handH: 1.2 };   // apex height [m], tumble [rad/s], hand height [m]
let thr = null;          // { phase: 'hand' | 'toss' | 'free', t, toss } while the hand has it or it's in the air
let launchMode = 'hover';   // what Reset does: start on the ground and take off, or throw
function startThrow() { thr = { phase: 'hand', t: 0 }; }   // called right after resetSim(): held at hand height
function releaseThrow() {   // the hand swings it up to speed and spin over a moment (the IMU feels it), then lets go
  const dur = 0.12, h = Math.max(0.3, throwCfg.height - S.p[2]);   // apex = release height + v0·dur/2 + v0²/2g
  const v0 = G * (-dur / 2 + Math.sqrt(dur * dur / 4 + 2 * h / G));
  let ax = [randn(), randn(), randn()]; if (nrm(ax) < 1e-6) ax = [1, 0, 0];
  thr.phase = 'toss'; thr.toss = { t: 0, dur, dv: [0.3 * randn(), 0.3 * randn(), v0], dw: scl(unit(ax), throwCfg.spin) };
}
const throwBusy = () => !!thr;
function throwHandTick(dt) {   // after the toss, the drone is on its own (the physics' crash checks wait until it has caught itself)
  if (!thr) return;
  thr.t += dt;
  if (thr.phase === 'toss' && thr.toss.t >= thr.toss.dur) thr.phase = 'free';
  const v = learn.view;
  if (thr.phase === 'free' && v && v.thr === 0 && thr.t > 1) thr = null;   // caught (or given up): an ordinary flight from here
}
