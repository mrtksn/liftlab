'use strict';
// Learning the airframe. The controller's model of "what each actuator input does" (the effectiveness
// matrix B, in acceleration per full throttle) comes either from the airframe description or from
// identification on flight data. A calibration cycle excites the actuators in stages to learn B quickly;
// "keep learning" goes on correcting it through the flight.
//
// Inputs are thrust fractions 0–1. A motor on the frame is one input. A motor on servo joints is several:
// its thrust times each product of (1, cos θ, sin θ) over the joints above it (3 for one joint, 9 for two).
// Turning a rigid part about a hinge is linear in cos θ and sin θ, so this is exact, and the motor's effect
// at any joint angles is a fixed sum of learned columns.

const learn = {
  mode: 'config',     // 'config': fly on the airframe description, 'ident': fly on the learned B
  keep: true,         // keep learning during flight
  dither: 0.02,       // small excitation while learning in flight, so there's always something to learn from
  holdPulses: true,   // freeze the other motors while one is pulsed, so the controller's corrections can't blur the data
  applyCurve: false,  // linearize thrust with the measured throttle-curve bend (off: the estimate is too rough to trust yet)
  memCal: 4, memFlight: 30,   // forgetting time [s]: at least the whole calibration run, then 30 s in flight
  st: {}, B: null, prior: null, priorKind: 'desc', sig: '', index: new Map(), n: 0,
  resp: new Map(),    // actuator responses from the tests (lag, throttle curve, servo speed)
  cal: null,          // running calibration
  fit: null,          // { force, rot } from the last validation
  msg: '',
};

/* ───────── inputs ───────── */
// Basis over a motor's joints (nearest first): products of (1, cos θ, sin θ), first joint most significant.
function basisVals(angles) { opc(32 * angles.length + 2 * 3 ** angles.length); let v = [1]; for (const a of angles) { const c = Math.cos(a), s = Math.sin(a); v = v.flatMap(x => [x, x * c, x * s]); } return v; }
function dBasisVals(angles, m) {   // derivative of the basis with respect to joint m
  opc(32 * angles.length + 3 * 3 ** angles.length); let v = [1];
  angles.forEach((a, i) => { const f = i === m ? [0, -Math.sin(a), Math.cos(a)] : [1, Math.cos(a), Math.sin(a)]; v = v.flatMap(x => f.map(y => x * y)); });
  return v;
}
// Turns a motor's effect, evaluated at joint angles {0, π/2, π} for each of its k joints, into its basis columns.
function decompose(evalAt, k) {
  opc(18 * k * 3 ** k); const n = 3 ** k, A = [0, Math.PI / 2, Math.PI], V = [];
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
  learn.n = j;
}
const seenAngles = c => chainOf(c).map(angleSeen);
function motorInputs() {   // per input: its motor's believed thrust command and its basis factor
  const v = new Array(learn.n).fill(0), phi = new Array(learn.n).fill(0), m = new Array(learn.n).fill(0);
  actuators().forEach((c, i) => {
    const st = act.get(c.id), ix = learn.index.get(c.id); if (!st || !ix) return;
    const b = basisVals(seenAngles(c)); ix.cols.forEach((jj, k) => { v[jj] = st.v || 0; phi[jj] = b[k]; m[jj] = i; });
  });
  return { v, phi, m };
}
function inputVector() {   // what was sent, in the identification's input space
  const x = new Array(learn.n).fill(0);
  for (const c of actuators()) {
    const st = act.get(c.id), ix = learn.index.get(c.id); if (!st || !ix) continue;
    const phi = basisVals(seenAngles(c));
    ix.cols.forEach((jj, b) => { x[jj] = (st.v || 0) * phi[b]; });   // thrust fraction the controller believes it asked for
  }
  return x;
}

/* ───────── columns (acceleration per full thrust, body frame) ───────── */
const cfgToAccel = col => { const f = scl([col[0], col[1], col[2]], 1 / model.m); const a = m3v(model.Jinv, [col[3], col[4], col[5]]); return [...f, ...a]; };
function describedAt(c, angles) {   // from the airframe description, with the motor's joints at the given angles
  const ch = chainOf(c), n = rotorNow(c, j => angles[ch.indexOf(j)]);
  return cfgToAccel(scl6(wrenchCol(n.p, n.d, c.spin, c.kappa, model.c), c.tmax * hModel(c)));
}
// Cached per model: the description only changes when the believed mass properties do (a new `model`).
const descCache = new WeakMap();
function describedCols(c) {
  let m = descCache.get(model); if (!m) { m = new Map(); descCache.set(model, m); }
  let cols = m.get(c.id); if (!cols) { cols = decompose(a => describedAt(c, a), chainOf(c).length); m.set(c.id, cols); }
  return cols;
}
function learnedCols(c) {
  const ix = learn.index.get(c.id); if (!learn.B || !ix) return null;
  return ix.cols.map(jj => learn.B.map(row => row[jj]));
}
const flyingLearned = () => learn.mode === 'ident' && !!learn.B;
function frozenCols(c) {   // the learned model as it was when a calibration began: flown on until the calibration decides
  const ix = learn.index.get(c.id); if (!learn.flyB || !ix) return null;
  return ix.cols.map(jj => learn.flyB.map(row => row[jj]));
}
const colsFor = c => (flyingLearned() && ((learn.cal && frozenCols(c)) || learnedCols(c))) || describedCols(c);
function sumCols(cols, w) { opc(12 * cols.length); const out = [0, 0, 0, 0, 0, 0]; cols.forEach((col, b) => { if (w[b]) for (let i = 0; i < 6; i++) out[i] += w[b] * col[i]; }); return out; }
const colAtAngles = (c, angles) => sumCols(colsFor(c), basisVals(angles));
const colAt = c => colAtAngles(c, seenAngles(c));                 // effect per full thrust at the joint angles believed now
function dColAt(c, j) {                                            // how that effect changes as joint j turns
  const m = chainOf(c).indexOf(j); if (m < 0) return [0, 0, 0, 0, 0, 0];
  return sumCols(colsFor(c), dBasisVals(seenAngles(c), m));
}
const add6 = (a, b) => a.map((x, i) => x + b[i]);
// The model the controller computes forces and torques with. On the learned model it doesn't know
// its mass or inertia, and works directly in accelerations.
const UNIT = { m: 1, J: [1, 0, 0, 0, 1, 0, 0, 0, 1], Jinv: [1, 0, 0, 0, 1, 0, 0, 0, 1] };
const ctlModel = () => flyingLearned() ? UNIT : model;
function ctlAxis() {   // nominal thrust axis the controller believes in
  if (!flyingLearned()) return nb;
  let s = [0, 0, 0];
  for (const c of actuators()) s = add(s, colAtAngles(c, chainOf(c).map(restAngle)).slice(0, 3));
  return nrm(s) > 1e-6 ? unit(s) : nb;
}
/* ───────── what the actuator tests learned ───────── */
// learn.resp: per motor { tau, curve }, per servo joint { rate, lag }. Until measured, the
// controller assumes a straight throttle curve, 35 ms motor lag, the servo's rated speed and no servo lag.
const believedThrust = (u, k) => (1 - k) * u + k * u * u;
// Bend the thrust linearization uses: the learned one once applied, otherwise a typical brushless prop curve
// (thrust grows faster than throttle). The flight software isn't told the real motors' curve.
const BEND_PRIOR = 0.7;
const curveHat = c => (learn.resp.get(c.id) || {}).applied ?? BEND_PRIOR;
const motorLagHat = c => (learn.resp.get(c.id) || {}).tau ?? 0.035;
function servoModelHat(j) { const r = learn.resp.get(j.id) || {}; return { rate: r.rate ?? j.rate * D2R, lag: r.lag ?? 0 }; }
function inputLags() { const l = new Array(learn.n).fill(0.035); for (const c of actuators()) { const ix = learn.index.get(c.id); if (ix) for (const jj of ix.cols) l[jj] = motorLagHat(c); } return l; }
function priorRows() {   // the description, as 6 rows × inputs: identification starts from here
  const rows = [0, 1, 2, 3, 4, 5].map(() => new Array(learn.n).fill(0));
  for (const c of actuators()) {
    const k = describedCols(c), ix = learn.index.get(c.id);
    ix.cols.forEach((jj, b) => { for (let i = 0; i < 6; i++) rows[i][jj] = k[b][i]; });
  }
  return rows;
}
function resetLearning(keepResponses = false) {
  if (!keepResponses || learn.sig !== inputSig()) learn.resp = new Map();
  if (!model) model = massProps('model');
  buildIndex(); learn.sig = inputSig(); learn.st = {}; learn.prior = priorRows(); learn.priorKind = 'desc'; learn.B = learn.prior.map(r => r.slice());
  learn.cal = null; learn.imuR = null; learn.holdServos = false; learn.refine = null;
}
// Filtered accelerometer (lever-arm swing removed) and angular acceleration, for the actuator tests.
function measStep(dt) {
  if (!(dt > 0)) return;
  const m = learn.mf || (learn.mf = {}), k = dt / (dt + 1 / (2 * Math.PI * 25));
  const imus = sensorsOf('imu'), r = learn.imuR || (imus.length ? mean3(imus.map(knownPos)) : [0, 0, 0]);
  if (!m.w) { m.w = est.fGyro.slice(); m.f = est.fAccel.slice(); m.a = [0, 0, 0]; }
  const wp = m.w; m.w = m.w.map((v, i) => v + k * (est.fGyro[i] - v)); m.a = m.w.map((v, i) => (v - wp[i]) / dt);
  const fh = sub(sub(est.fAccel, crs(m.a, r)), crs(m.w, crs(m.w, r)));
  m.f = m.f.map((v, i) => v + k * (fh[i] - v));
}
// A measured 6-vector [f; α] projected onto an effect column: each half weighted by its own size.
function project(col, y6) {
  let num = 0, den = 0;
  for (const [a, b] of [[0, 3], [3, 6]]) {
    let cc = 0, cy = 0; for (let i = a; i < b; i++) { cc += col[i] * col[i]; cy += col[i] * y6[i]; }
    if (cc > 1e-6) { num += cy / cc; den += 1; }
  }
  return den ? num / den : 0;
}

/* ───────── each control step ───────── */
function learnStep(dt) {
  if (learn.sig !== inputSig()) { resetLearning(); learn.msg = 'The actuators changed, so learning restarted from the airframe description.'; }
  if (!est.haveImu || !learn.n || S.crashed) { if (learn.cal && S.crashed) endCalibration('Calibration stopped: the drone crashed.'); return; }
  measStep(dt);
  refineThrowStep(dt);
  if (thr && thr.phase !== 'recover') return;   // the throw runs its own identification while falling
  if (learn.keep || learn.cal) {
    const imus = sensorsOf('imu'); const r0 = learn.imuR || (imus.length ? mean3(imus.map(knownPos)) : [0, 0, 0]);
    const r = run('identifyEffectiveness', learn.st, inputVector(), est.fAccel, est.fGyro, r0, dt, learn.prior, learn.cal ? Math.max(learn.memCal, learn.cal.total) : learn.memFlight, inputLags(), motorInputs());
    learn.B = r.B;
  }
  if (learn.cal) calibrationTick(dt);
}

/* ───────── calibration cycle ───────── */
// Staged excitation while hovering: settle, pulse each actuator in turn, sweep the servos, excite
// everything at once, then validate on a fresh signal the model hasn't been fitted to yet.
function startCalibration(opts = {}) {
  if (S.crashed) return;
  learn.flyB = learn.mode === 'ident' && learn.B ? learn.B.map(r => r.slice()) : null;   // keep flying on this while the test runs
  // After a throw there is no description to fall back on: start from, and compete against, the throw model.
  if (opts.fromThrow && learn.mode === 'ident') { learn.st = {}; learn.prior = learn.B.map(r => r.slice()); learn.priorKind = 'throw'; learn.cal = null; }
  else resetLearning(true);
  const acts = actuators(), servos = steerJoints();
  const seg = [];
  seg.push({ stage: 'Settling', dur: 1, at: () => null });
  // Motor tests: each motor steps up then down (6%, then 15%) while everything else holds still.
  // Each test waits until the drone is calm, so it starts from a steady hover.
  for (const a of [0.06, 0.16]) acts.forEach(c => seg.push({ stage: 'Testing each motor', dur: 0.6, calm: true, rec: { kind: 'motor', c, until: 0.25 },
    at: t => ({ exc: new Map([[c.id, t < 0.02 ? 0 : t < 0.09 ? a : t < 0.16 ? -a : 0]]), hold: learn.holdPulses && t < 0.25, holdServos: learn.holdPulses && t < 0.25 }) }));
  if (seg.length > 1) seg[seg.length - 1].after = fitMotors;
  // Servo tests: each servo swings one way, then the other, while the motors and other servos hold.
  servos.forEach(c => { const d = 0.35 * c.range * D2R; seg.push({ stage: 'Testing each servo', dur: 0.8, calm: true, rec: { kind: 'servo', c, until: 0.4 },
    at: t => ({ step: { id: c.id, d: t < 0.02 ? 0 : t < 0.17 ? d : t < 0.32 ? -d : 0 }, hold: t < 0.4, holdServos: t < 0.4 }) }); });
  if (servos.length) seg[seg.length - 1].after = fitServos;
  servos.forEach(c => seg.push({ stage: 'Sweeping servos', dur: 2.5, at: t => ({ servo: new Map([[c.id, 0.3 * c.range * D2R * Math.sin(2 * Math.PI * 0.8 * t)]]) }) }));
  const multisine = (f0, df, amp) => t => {
    const exc = new Map(), servo = new Map();
    acts.forEach((c, i) => {
      exc.set(c.id, amp * Math.sin(2 * Math.PI * (f0 + df * i) * t + i * 1.3));
    });
    servos.forEach((j, i) => servo.set(j.id, 0.3 * j.range * D2R * Math.sin(2 * Math.PI * (0.6 + 0.35 * i) * t)));
    return { exc, servo };
  };
  seg.push({ stage: 'Exciting everything together', dur: 3, at: multisine(2.3, 1.7, 0.04) });
  seg.push({ stage: 'Validating', dur: 2, validate: true, at: multisine(3.1, 1.3, 0.04) });
  let t0 = 0; for (const s of seg) { s.t0 = t0; t0 += s.dur; }
  learn.cal = { seg, t: 0, total: t0, now: null, base: opts.fromThrow && learn.mode === 'ident' ? 'throw' : 'desc', sums: { e: [0, 0, 0, 0, 0, 0], d: [0, 0, 0, 0, 0, 0], y: [0, 0, 0, 0, 0, 0], y2: [0, 0, 0, 0, 0, 0], n: 0 } };
  learn.msg = '';
}
function calibrationTick(dt) {
  const cal = learn.cal;
  const tilt = Math.acos(clamp(dot(m3v(est.R, ctlAxis()), [0, 0, 1]), -1, 1));
  if (tilt > 0.52 || nrm(est.w) > 4) { cal.now = null; cal.held = true; if (cal.win) cal.win.bad = true; return; }   // tilted past 30° or spinning: pause the excitation
  cal.held = false;
  cal.t += dt;
  const s = cal.seg.find(x => cal.t >= x.t0 && cal.t < x.t0 + x.dur);
  if (!s) return finishCalibration();
  if (cal.cur && cal.cur !== s && cal.cur.after) cal.cur.after();   // a test stage just ended: fit it
  if (cal.cur !== s) { cal.cur = s; cal.win = null; cal.gate = s.calm ? 0 : null; }
  if (cal.gate != null) {   // wait (closed loop) until the drone is calm, at most 1.5 s
    const calm = nrm(est.w) < 0.25 && tilt < 0.14;
    if (!calm && cal.gate < 1.5) { cal.gate += dt; cal.t -= dt; cal.now = null; cal.stage = s.stage; return; }
    cal.gate = null;
  }
  cal.stage = s.stage; cal.now = s.at(cal.t - s.t0) || null;
  if (s.rec && cal.t - s.t0 < s.rec.until) recordTest(s, dt);
  if (s.validate && learn.st.e && learn.st.x) {   // score the learned model and the description on the same fresh data
    const S_ = cal.sums, x = learn.st.x; S_.n++;
    for (let i = 0; i < 6; i++) {
      const yd = learn.prior[i].reduce((a, v, j) => a + v * x[j], 0);
      S_.e[i] += learn.st.e[i] ** 2; S_.d[i] += (learn.st.y[i] - yd) ** 2; S_.y[i] += learn.st.y[i]; S_.y2[i] += learn.st.y[i] ** 2;
    }
  }
}
function finishCalibration() {
  const S_ = learn.cal.sums, fitOf = (key, rows) => {
    let e = 0, v = 0; for (const i of rows) { e += S_[key][i]; v += S_.y2[i] - S_.y[i] ** 2 / Math.max(1, S_.n); }
    return v > 1e-9 ? clamp(1 - e / v, 0, 1) : 0;
  };
  learn.fit = { force: fitOf('e', [0, 1, 2]), rot: fitOf('e', [3, 4, 5]) };
  const desc = { force: fitOf('d', [0, 1, 2]), rot: fitOf('d', [3, 4, 5]) };
  learn.fitDesc = desc;
  const pc = v => Math.round(v * 100) + '%';
  const better = learn.fit.rot + 0.5 * learn.fit.force > desc.rot + 0.5 * desc.force + 0.02;
  const good = learn.fit.force > 0.5 && learn.fit.rot > 0.6 && better;
  const fromThrow = learn.cal.base === 'throw', baseName = fromThrow ? 'the model from the throw' : 'the airframe description';
  if (good) { learn.mode = 'ident'; learn.keep = true; learn.holdServos = false; ctl.iAtt = [0, 0, 0]; ctl.iPos = [0, 0, 0]; }   // integrators were wound up for the old model
  else if (fromThrow) { learn.B = learn.prior.map(r => r.slice()); learn.st = {}; }   // keep the throw model
  else learn.mode = 'config';
  learn.flyB = null;
  endCalibration(good
    ? `Calibrated. On fresh test moves the learned model explains ${pc(learn.fit.rot)} of the rotation and ${pc(learn.fit.force)} of the force (${baseName}: ${pc(desc.rot)} and ${pc(desc.force)}). Flying on the learned model and still learning.`
    : `Calibration finished. The learned model explains ${pc(learn.fit.rot)} of the rotation and ${pc(learn.fit.force)} of the force, but ${baseName} does ${better ? 'nearly as well' : 'as well or better'} (${pc(desc.rot)}, ${pc(desc.force)}), so it keeps flying on ${fromThrow ? 'that' : 'the description'}.`);
}
function endCalibration(msg) { learn.cal = null; learn.flyB = null; learn.msg = msg; if (typeof renderLearn === 'function') renderLearn(true); }
const calExc = c => (learn.cal && learn.cal.now && learn.cal.now.exc && learn.cal.now.exc.get(c.id)) || 0;
function calServo(j) {   // servo angle the calibration is holding, or null
  const now = learn.cal && learn.cal.now, st = jst.get(j.id);
  if (!st) return null;
  if (now && now.step && now.step.id === j.id) {
    const cal = learn.cal; if (cal.stepBase == null || cal.stepFor !== cal.cur) { cal.stepBase = st.thCmd; cal.stepFor = cal.cur; }
    return clamp(cal.stepBase + now.step.d, -j.range * D2R, j.range * D2R);
  }
  if (!now || !now.servo || !now.servo.has(j.id)) return null;
  // Sweeps ride on top of what the controller asks for, so it keeps its servo authority (a tricopter's
  // tail servo is its only real yaw control).
  return clamp(st.thCmd + now.servo.get(j.id), -j.range * D2R, j.range * D2R);
}
function ditherFor(c, t) {   // tiny excitation while learning in flight
  if (!learn.keep || learn.cal || !learn.dither) return 0;
  const ix = learn.index.get(c.id); const i = ix ? ix.cols[0] : 0;
  return learn.dither * Math.sin(2 * Math.PI * (2.7 + 1.9 * i) * t + i);
}
// Each motor's true basis columns, from the real geometry, mass, inertia, health and battery (no airflow).
function trueAt(c, angles) {
  const ch = chainOf(c), n = rotorNow(c, j => angles[ch.indexOf(j)]);
  const col = scl6(wrenchCol(n.p, n.d, c.spin, c.kappa, truth.c), c.tmax * c.health / 100 * S.battK);
  const f = scl([col[0], col[1], col[2]], 1 / truth.m), a = m3v(truth.Jinv, [col[3], col[4], col[5]]);
  return [...f, ...a];
}
function trueB() { const out = new Map(); for (const c of actuators()) out.set(c.id, decompose(a => trueAt(c, a), chainOf(c).length)); return out; }
// How well each motor's learned effect matches the truth right now (1 = exact). Compared as the effect
// itself at joint angles across each steering joint's range (both ends and the middle), not column by
// column: over a ±30° range cos θ barely changes, so how the "1" and "cos θ" columns split is unknowable
// and doesn't matter.
function matchScores() {
  const acts = actuators(); if (!acts.length || !learn.B) return [];
  const sj = steerJoints();
  const samples = c => {   // angle sets to compare at
    const ch = chainOf(c); let sets = [ch.map(angleTrue)];
    ch.forEach((j, m) => { if (sj.includes(j)) sets = sets.flatMap(a => [-1, 0, 1].map(k => a.map((x, i) => i === m ? k * j.range * D2R : x))); });
    return sets;
  };
  const tb = trueB();
  const pairs = acts.map(c => { const T = tb.get(c.id), L = learnedCols(c); return samples(c).map(a => { const phi = basisVals(a); return [sumCols(L, phi), sumCols(T, phi)]; }); });
  // Rows are weighted by their typical size: force rows together, roll/pitch together, yaw on its own.
  const rowMax = i => { let m = 0; for (const P of pairs) for (const [, t] of P) m = Math.max(m, Math.abs(t[i])); return m; };
  const groups = [[0, 1, 2], [3, 4], [5]], scale = new Array(6);
  for (const g of groups) { const m = Math.max(1e-6, ...g.map(rowMax)); for (const i of g) scale[i] = 1 / m; }
  return acts.map((c, k) => {
    let e = 0, v = 0;
    for (const [l, t] of pairs[k]) for (let i = 0; i < 6; i++) { e += ((l[i] - t[i]) * scale[i]) ** 2; v += (t[i] * scale[i]) ** 2; }
    return { c, match: clamp(1 - Math.sqrt(e / Math.max(v, 1e-9)), 0, 1) };
  });
}

const servoHeld = () => !!(learn.cal && learn.cal.now && learn.cal.now.holdServos);

/* ───────── actuator tests ───────── */
// Records one test window: what was sent to the actuator under test, and the drone's response along
// that actuator's effect. The fits run when the stage ends and feed the controller straight away.
function recordTest(s, dt) {
  const cal = learn.cal, c = s.rec.c; if (!learn.mf || !learn.mf.f) return;
  if (!cal.win) {
    let col;
    if (s.rec.kind === 'motor') col = colAt(c);
    else { col = [0, 0, 0, 0, 0, 0]; for (const m of motorsUnder(c)) col = add6(col, scl6(dColAt(m, c), act.get(m.id).v || 0)); }   // what turning this joint does
    cal.win = { c, kind: s.rec.kind, col, u: [], cmd: [], y: [], dt, cmd0: s.rec.kind === 'servo' ? jointTarget(c) : 0 };
    (cal.tests || (cal.tests = [])).push(cal.win);
  }
  const w = cal.win, y6 = [...learn.mf.f, ...learn.mf.a];
  if (w.kind === 'motor') w.u.push(act.get(c.id).u); else w.cmd.push(jointTarget(c) - w.cmd0);
  w.y.push(project(w.col, y6));
}
const slopeAt = (k, u) => (1 - k) + 2 * k * u;
function fitMotors() {
  const cal = learn.cal; if (!cal || !learn.holdPulses) return;
  for (const c of actuators()) {
    const wins = (cal.tests || []).filter(w => w.c === c && w.kind === 'motor' && !w.bad && w.u.length > 50);
    if (!wins.length) continue;
    const r = run('identifyMotorResponse', wins, wins[0].dt);
    const old = learn.resp.get(c.id) || {};
    if (r.fit < 0.5 || !(r.gain > 0.3 && r.gain < 3)) { learn.resp.set(c.id, { ...old, fitM: r.fit }); continue; }   // poor fit, or a response nothing like the model's
    const u0 = wins.reduce((a, w) => a + w.u[0], 0) / wins.length;
    const next = { ...old, tau: r.tau, curve: r.curve, fitM: r.fit, u0 };
    if (learn.applyCurve) {   // props only bend upward, so a negative estimate is noise
      const k = clamp(r.curve, 0, 1);
      rescaleInput(c, slopeAt(k, u0) / slopeAt(curveHat(c), u0));   // same thrust, new units: rescale what was learned
      next.applied = k;
    }
    learn.resp.set(c.id, next);
  }
}
function fitServos() {
  const cal = learn.cal; if (!cal) return;
  for (const j of steerJoints()) {
    const wins = (cal.tests || []).filter(w => w.c === j && w.kind === 'servo' && !w.bad && w.cmd.length > 50);
    if (!wins.length) continue;
    const r = run('identifyServoResponse', wins, wins[0].dt);
    const old = learn.resp.get(j.id) || {};
    if (r.fit < 0.4 || !(r.gain > 0.3 && r.gain < 3)) { learn.resp.set(j.id, { ...old, fitS: r.fit }); continue; }
    learn.resp.set(j.id, { ...old, rate: r.rate, lag: r.lag, fitS: r.fit });
    const st = jst.get(j.id); if (st) st.pst = { h: st.thHat, th: st.thHat };
  }
}
// When the throttle curve changes, "one unit of input" changes size: x_new = ratio·x_old. Learned
// columns shrink by the same ratio so the model predicts the same thrust. The airframe description is
// already in true thrust units and stays as it is.
function rescaleInput(c, ratio) {
  if (!(ratio > 0.2 && ratio < 5) || Math.abs(ratio - 1) < 1e-6) return;
  const ix = learn.index.get(c.id); if (!ix) return; const js = ix.cols, s = 1 / ratio;
  const mats = new Set([learn.B, learn.flyB, learn.st.th, learn.priorKind !== 'desc' ? learn.prior : null].filter(Boolean));   // B and the RLS state can be the same array
  for (const M of mats) for (const row of M) for (const j of js) row[j] *= s;
  const st = learn.st;
  if (st.P) for (let a = 0; a < st.P.length; a++) for (let b = 0; b < st.P.length; b++) st.P[a][b] *= (js.includes(a) ? s : 1) * (js.includes(b) ? s : 1);
  for (const key of ['ua', 'xl', 'xs']) if (st[key]) for (const j of js) st[key][j] *= ratio;
}

/* ───────── hold during calibration pulses ───────── */
// While one motor is pulsed, the others keep the throttle they had when the pulse began. The controller
// would otherwise answer every pulse with the other motors, and inputs that always move together
// can't be told apart. The pulse is a doublet (up, then down), so the drone barely moves in 0.3 s.
function holdU(u) {
  const cal = learn.cal, now = cal && cal.now;
  if (!now || !now.hold) { if (cal) cal.holdU = null; return u; }
  if (!cal.holdU || cal.holdU.length !== u.length) cal.holdU = u.slice();
  return cal.holdU;
}

/* ───────── throw start ───────── */
// Launch option after Blaha, Smeur & Remes (TU Delft, 2024): the drone is thrown upward with its motors
// off and a random tumble, and knows nothing about its own geometry. While it falls it pulses each motor
// in turn (open loop, nothing fighting the pulses), fits its model with identifyThrow, then catches itself
// with the controller running on what it just learned.
const throwCfg = { height: 4, spin: 6, amp: 0.5, dwMax: 4, thenCalibrate: true };   // apex altitude [m], tumble rate [rad/s], pulse throttle, rate change allowed per pulse [rad/s]
let thr = null;
function throwPlan() {
  // Every motor, pulsed with each steering joint above it at three angles in turn (the middle, one end, the
  // other end), so its (1, cos θ, sin θ) columns can all be told apart. Passes go motor by motor, so a joint
  // can swing to its next angle while other motors are being pulsed.
  const sj = steerJoints(), variants = [];
  for (const c of actuators()) {
    const js = chainOf(c).filter(j => sj.includes(j)), v = [];
    if (!js.length) v.push(new Map());
    else {
      v.push(new Map(js.map(j => [j.id, 0])));   // the middle first: if it has to stop early, it knows the motors at rest
      v.push(new Map(js.map((j, i) => [j.id, i === 0 ? -0.9 * j.range * D2R : 0])));
      js.forEach((j, i) => {
        if (i === 0) v.push(new Map(js.map(x => [x.id, x === j ? 0.9 * j.range * D2R : 0])));
        else v.push(new Map(js.map(x => [x.id, x === j ? -0.9 * j.range * D2R : 0])), new Map(js.map(x => [x.id, x === j ? 0.9 * j.range * D2R : 0])));
      });
    }
    variants.push({ c, v });
  }
  const plan = [], most = Math.max(0, ...variants.map(x => x.v.length));
  for (let k = 0; k < most; k++) for (const { c, v } of variants) if (v[k]) plan.push({ c, angles: v[k] });
  return plan;
}
// Rough length of the pulse sequence [s]: each pulse and rest, plus time for a servo to swing between pulses.
const throwPlanTime = plan => plan.reduce((s, P, i) => s + 0.13 + (i > 0 && [...P.angles].some(([id, a]) => plan[i - 1].angles.get(id) !== a) ? 0.08 : 0), 0);
// Height it needs to catch itself, moving up at vz [m/s]: spin up and turn upright (~0.45 s, coasting), then
// brake at about 0.8 g.
function throwRoom(vz) { const t = 0.45, v1 = Math.min(0, vz - G * t); return 0.3 - (vz * t - 0.5 * G * t * t) + v1 * v1 / (2 * 0.8 * G); }
function startThrow() {                  // call right after resetSim()
  thr = { phase: 'hand', t: 0, plan: throwPlan(), i: 0, step: 'move', ts: 0, w0: null, st: {}, res: null, zMax: 0, tRel: 0 };
  learn.mode = 'config'; learn.msg = 'In the hand, motors off. The drone knows its sensors and how many actuators it has, nothing else.';
  for (const a of act.values()) { a.u = 0; a.Tcmd = 0; a.T = 0; a.Omega = 0; a.i = 0; }
}
function releaseThrow() {   // the hand swings it up to speed and spin over a moment (the IMU feels it), then lets go
  const dur = 0.12, h = Math.max(0.3, throwCfg.height - S.p[2]);   // apex = release height + v0·dur/2 + v0²/2g
  const v0 = G * (-dur / 2 + Math.sqrt(dur * dur / 4 + 2 * h / G));
  let ax = [randn(), randn(), randn()]; if (nrm(ax) < 1e-6) ax = [1, 0, 0];
  thr.phase = 'toss'; thr.toss = { t: 0, dur, dv: [0.3 * randn(), 0.3 * randn(), v0], dw: scl(unit(ax), throwCfg.spin) };
  learn.msg = 'Thrown. Near the top of the arc it pulses each motor on its own and fits what each one does from the gyro and accelerometer.';
}
const throwBusy = () => !!thr && thr.phase !== 'recover';
// Throttle and servo commands while the throw runs open loop. Returns true when it has set them.
function throwTick(dt) {
  if (!thr) return false;
  thr.t += dt; thr.zMax = Math.max(thr.zMax, S.p[2]);
  const acts = actuators(), cmd = new Map(acts.map(c => [c.id, 0]));
  const servo = new Map();   // each steering joint heads for the angle of the next pulse that needs it
  if (thr.plan) for (const j of steerJoints()) {
    const from = thr.phase === 'excite' && thr.step === 'rest' ? thr.i + 1 : thr.i;
    const P = thr.plan.slice(from).find(x => x.angles.has(j.id));
    servo.set(j.id, P ? P.angles.get(j.id) : 0);
  }
  if (thr.phase === 'hand') { if (thr.t > 0.8) releaseThrow(); }
  else if (thr.phase === 'toss') { if (thr.toss.t >= thr.toss.dur) { thr.phase = 'free'; thr.t = 0; thr.zRel = S.p[2]; } }
  else if (thr.phase === 'free' || thr.phase === 'excite') {
    // Pulse around the top of the throw: climbing or falling air through the props changes their thrust.
    const tPlan = throwPlanTime(thr.plan);
    // Start so the pulses finish just past the top: on the way down it soon needs its height to recover.
    if (thr.phase === 'free' && thr.t > 0.06 && est.v[2] < G * (tPlan - 0.1)) { thr.phase = 'excite'; thr.ts = thr.t; }
    const vb = m3v(m3T(est.R), est.v);
    if (thr.phase === 'free') run('identifyThrow', thr.st, inputVector(), est.fAccel, est.fGyro, vb, dt, false, motorInputs());
    if (thr.phase === 'excite') {
      const P = thr.plan[thr.i], el = thr.t - thr.ts;
      if (P) {
        if (thr.step === 'move') {   // only waits if a joint isn't at its pulse angle yet
          const ready = [...P.angles].every(([id, a]) => Math.abs(angleSeen(compById(id)) - a) < 2 * D2R);
          if (ready || el > 0.15) { thr.step = 'on'; thr.ts = thr.t; thr.w0 = est.fGyro.slice(); }
        } else if (thr.step === 'on') {
          // pulse until the rotation it causes reaches dwMax (stays well inside the gyro's range), 12–80 ms
          const dw = nrm(sub(est.fGyro, thr.w0));
          if ((el > 0.012 && dw > throwCfg.dwMax) || el > 0.08) { thr.step = 'rest'; thr.ts = thr.t; } else cmd.set(P.c.id, throwCfg.amp);
        } else if (el > 0.035) { thr.i++; thr.step = 'move'; thr.ts = thr.t; }
      }
      // Stop early if it has to start catching itself: room to spin the motors up, turn upright and brake.
      if (thr.i < thr.plan.length && est.p[2] < throwRoom(est.v[2])) thr.cut = thr.i;
      const done = thr.i >= thr.plan.length || thr.cut != null;
      const x = inputVector();
      thr.res = run('identifyThrow', thr.st, x, est.fAccel, est.fGyro, vb, dt, done, motorInputs());
      if (done) finishThrow();
    }
  }
  if (thr && thr.phase !== 'recover') {
    for (const c of acts) {
      const st = act.get(c.id); if (!st) continue;
      setThrottle(c, st, cmd.get(c.id));
    }
    for (const [id, a] of servo) jst.get(id).thCmd = a;
    return true;
  }
  return false;
}
const lagText = r => { const t = (r.taus && r.taus.length ? r.taus : [r.tau]).map(x => Math.round(x * 1000)), lo = Math.min(...t), hi = Math.max(...t);
  return lo === hi ? `≈ ${lo} ms` : `${lo}–${hi} ms`; };
// Use a throw fit as the flight model. Motors not pulsed at every servo angle (the pulses were cut short)
// keep what the angles tried can tell; returns true if some servo's effect is still unknown.
function adoptThrowModel(r, plan, cut) {
  const B = r.B.map(row => row.slice());
  let partial = false;
  if (cut != null) {
    const done = plan.slice(0, cut);
    for (const c of actuators()) {
      const ix = learn.index.get(c.id); if (!ix || ix.cols.length < 2) continue;
      if (done.filter(P => P.c === c).length >= plan.filter(P => P.c === c).length) continue;
      const val = (row, phi) => ix.cols.reduce((s, jj, b) => s + row[jj] * phi[b], 0);
      const j = chainOf(c)[0], tried = [...new Set(done.filter(P => P.c === c).map(P => P.angles.get(j.id) ?? 0))].filter(a => Math.abs(a) > 1e-6);
      for (const row of B) {
        const v0 = val(row, basisVals(new Array(ix.k).fill(0)));
        if (ix.k === 1 && tried.length) {   // the middle and one end: its effect and how it changes with the servo (cos θ ≈ 1 here)
          const a1 = tried[0], v1 = val(row, basisVals([a1]));
          row[ix.cols[0]] = v0; row[ix.cols[1]] = 0; row[ix.cols[2]] = (v1 - v0) / Math.sin(a1);
        } else ix.cols.forEach((jj, b) => { row[jj] = b === 0 ? v0 : 0; });   // only the middle
      }
      if (!(ix.k === 1 && tried.length)) partial = true;
    }
  }
  learn.B = B.map(row => row.slice()); learn.prior = B.map(row => row.slice()); learn.st = {};
  learn.mode = 'ident'; learn.keep = true; learn.imuR = r.r.slice();
  return partial;
}
function finishThrow() {
  const r = thr.res, ok = r && r.fitR > 0.6 && r.fitF > 0.4;
  thr.phase = 'recover'; thr.tRec = thr.t; ctl.iAtt = [0, 0, 0]; ctl.iPos = [0, 0, 0];
  const pc = v => Math.round(v * 100) + '%';
  let partial = false;
  learn.refine = null;
  if (ok) {
    partial = adoptThrowModel(r, thr.plan, thr.cut);
    if (!r.refined) learn.refine = { st: thr.st, plan: thr.plan, cut: thr.cut, fitR: r.fitR, fitF: r.fitF, t0: S.t };   // each motor's own lag, in the background
    thr.msg = `Identified in ${(thr.t).toFixed(2)} s of free fall: the fit explains ${pc(r.fitR)} of the rotation and ${pc(r.fitF)} of the force, motor lag ${lagText(r)}, IMU ${(nrm(r.r) * 100).toFixed(1)} cm from the balance point (true ${(nrm(trueImuOffset()) * 100).toFixed(1)} cm).`;
  } else {
    learn.mode = 'config';
    thr.msg = `The free-fall fit was poor (rotation ${pc(r ? r.fitR : 0)}, force ${pc(r ? r.fitF : 0)}), so it catches itself on the airframe description instead.`;
  }
  if (thr.cut != null) {
    const sj = steerJoints().length > 0;
    thr.msg += ` It stopped after ${thr.cut} of ${thr.plan.length} pulses to leave room to catch itself${sj && partial ? ', so it holds its servos in the middle until a calibration has measured them' : ''}.`;
    if (sj && ok && partial) learn.holdServos = true;
  }
  learn.msg = thr.msg + ' Recovering…';
}
// After a throw: work out each motor's own lag from the logged fall, with only the time the flight
// computer has spare (the ESP32's second core), and switch to it if it explains the fall better.
function refineThrowStep(dt) {
  const J = learn.refine; if (!J || !(dt > 0)) return;
  if (learn.sig !== inputSig()) { learn.refine = null; return; }
  const z = new Array(learn.n).fill(0), r = run('identifyThrow', J.st, z, [0, 0, 0], [0, 0, 0], [0, 0, 0], 0, 'refine', motorInputs(), budgetBackgroundOps(dt));
  J.spent = r.spent || 0; J.progress = r.progress ?? (r.refined ? 1 : 0);
  if (!r.refined) return;
  learn.refine = null;
  const secs = (S.t - J.t0).toFixed(1), differ = new Set(r.taus || []).size > 1;
  const better = differ && r.improved;   // lags that differ explained the fall's rotation better
  let note = ` Worked out each motor's own lag in the background (${lagText(r)}, ${secs} s on the spare core)`;
  if (better && !learn.cal && learn.mode === 'ident') {
    const partial = adoptThrowModel(r, J.plan, J.cut); learn.holdServos = learn.holdServos && partial;
    note += `; it now explains ${Math.round(r.fitR * 100)}% of the fall's rotation and flies on that.`;
  } else note += better ? '; the calibration running now supersedes it.' : differ ? '; it didn\'t fit the fall any better, so nothing changed.' : '; all the motors share one lag, so nothing changed.';
  learn.msg += note;
  if (typeof renderLearn === 'function') renderLearn(true);
}
function trueImuOffset() {   // mean IMU position relative to the true CoG, body frame (for the report only)
  const imus = sensorsOf('imu'); if (!imus.length) return [0, 0, 0];
  return sub(mean3(imus.map(posNow)), truth.c);
}
// Called every control step after the controller has taken over: ends the throw once upright and calm.
function throwRecoverCheck() {
  if (!thr || thr.phase !== 'recover') return;
  const up = dot(m3v(est.R, ctlAxis()), [0, 0, 1]);   // judged on the drone's own estimate
  thr.zMin = Math.min(thr.zMin ?? S.p[2], S.p[2]);
  thr.calmT = up > 0.9 && nrm(est.w) < 1.5 && Math.abs(est.v[2]) < 1 ? (thr.calmT || 0) + 2 * PDT : 0;   // upright and calm for half a second
  if (thr.calmT > 0.5) {
    const caught = `${thr.msg} Caught itself ${thr.t.toFixed(1)} s after release (thrown to ${thr.zMax.toFixed(1)} m, lowest point on the way down ${thr.zMin.toFixed(1)} m).`;
    const learned = learn.mode === 'ident';
    thr = null;
    if (learned && throwCfg.thenCalibrate) { startCalibration({ fromThrow: true }); learn.msg = caught + ' Now refining it with a hover calibration…'; }
    else learn.msg = caught + ' Still learning in flight.';
    if (typeof renderLearn === 'function') renderLearn(true);
  }
}
let launchMode = 'hover';   // what Reset does: start hovering, or throw
