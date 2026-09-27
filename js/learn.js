'use strict';
// Learning the airframe. The controller's model of "what each actuator input does" (the effectiveness
// matrix B, in acceleration per full throttle) comes either from the airframe description or from
// identification on flight data. A calibration cycle excites the actuators in stages to learn B quickly;
// "keep learning" goes on correcting it through the flight.
//
// Inputs are throttle fractions 0–1. A fixed motor is one input; a servo-mounted motor is two,
// a = u·cosθ and b = u·sinθ, so its effect at any servo angle is a·B_a + b·B_b.

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
function inputSig() { return actuators().map(c => c.id + (c.type === 'tilt' ? 't' : 'm')).join(','); }
function buildIndex() {
  learn.index = new Map(); let j = 0;
  for (const c of actuators()) { if (c.type === 'tilt') { learn.index.set(c.id, { a: j, b: j + 1 }); j += 2; } else { learn.index.set(c.id, { u: j }); j += 1; } }
  learn.n = j;
}
function inputVector() {   // what was sent, in the identification's input space
  const x = new Array(learn.n).fill(0);
  for (const c of actuators()) {
    const st = act.get(c.id), ix = learn.index.get(c.id); if (!st || !ix) continue;
    const u = st.v || 0, th = thSeen(c);   // thrust fraction the controller believes it asked for
    if (c.type === 'tilt') { x[ix.a] = u * Math.cos(th); x[ix.b] = u * Math.sin(th); } else x[ix.u] = u;
  }
  return x;
}

/* ───────── columns (acceleration per full throttle, body frame) ───────── */
const cfgToAccel = col => { const f = scl([col[0], col[1], col[2]], 1 / model.m); const a = m3v(model.Jinv, [col[3], col[4], col[5]]); return [...f, ...a]; };
function describedCols(c) {   // from the airframe description the controller was given
  const k = c.tmax * hModel(c);
  if (c.type === 'tilt') return { a: cfgToAccel(scl6(wrenchCol(c.pos, [0, 0, 1], c.spin, c.kappa, model.c), k)), b: cfgToAccel(scl6(wrenchCol(c.pos, hingeE(c), c.spin, c.kappa, model.c), k)) };
  return { u: cfgToAccel(scl6(wrenchCol(c.pos, actDir(c, 0), c.spin, c.kappa, model.c), k)) };
}
function learnedCols(c) {
  const ix = learn.index.get(c.id); if (!learn.B || !ix) return null;
  const col = j => learn.B.map(row => row[j]);
  return c.type === 'tilt' ? { a: col(ix.a), b: col(ix.b) } : { u: col(ix.u) };
}
const flyingLearned = () => learn.mode === 'ident' && !!learn.B;
function frozenCols(c) {   // the learned model as it was when a calibration began: flown on until the calibration decides
  const ix = learn.index.get(c.id); if (!learn.flyB || !ix) return null;
  const col = j => learn.flyB.map(row => row[j]);
  return c.type === 'tilt' ? { a: col(ix.a), b: col(ix.b) } : { u: col(ix.u) };
}
const colsFor = c => (flyingLearned() && ((learn.cal && frozenCols(c)) || learnedCols(c))) || describedCols(c);
function colAt(c, th) { const k = colsFor(c); return c.type === 'tilt' ? add6(scl6(k.a, Math.cos(th)), scl6(k.b, Math.sin(th))) : k.u; }
const add6 = (a, b) => a.map((x, i) => x + b[i]);
// The model the controller computes forces and torques with. On the learned model it doesn't know
// its mass or inertia, and works directly in accelerations.
const UNIT = { m: 1, J: [1, 0, 0, 0, 1, 0, 0, 0, 1], Jinv: [1, 0, 0, 0, 1, 0, 0, 0, 1] };
const ctlModel = () => flyingLearned() ? UNIT : model;
function ctlAxis() {   // nominal thrust axis the controller believes in
  if (!flyingLearned()) return nb;
  let s = [0, 0, 0];
  for (const c of actuators()) { const k = colsFor(c); const col = c.type === 'tilt' ? k.a : k.u; s = add(s, col.slice(0, 3)); }
  return nrm(s) > 1e-6 ? unit(s) : nb;
}
/* ───────── what the actuator tests learned ───────── */
// learn.resp: per actuator { tau, curve } for motors, { rate, lag } for servos. Until measured, the
// controller assumes a straight throttle curve, 35 ms motor lag, the servo's rated speed and no servo lag.
const believedThrust = (u, k) => (1 - k) * u + k * u * u;
const curveHat = c => (learn.resp.get(c.id) || {}).applied ?? 0;   // bend the thrust linearization uses
const motorLagHat = c => (learn.resp.get(c.id) || {}).tau ?? 0.035;
function servoModelHat(c) { const r = learn.resp.get(c.id) || {}; return { rate: r.rate ?? c.rate * D2R, lag: r.lag ?? 0 }; }
function inputLags() { const l = new Array(learn.n).fill(0.035); for (const c of actuators()) { const ix = learn.index.get(c.id); if (ix) for (const j of Object.values(ix)) l[j] = motorLagHat(c); } return l; }
function priorRows() {   // the description, as 6 rows × inputs: identification starts from here
  const rows = [0, 1, 2, 3, 4, 5].map(() => new Array(learn.n).fill(0));
  for (const c of actuators()) {
    const k = describedCols(c), ix = learn.index.get(c.id);
    for (const [key, j] of Object.entries(ix)) for (let i = 0; i < 6; i++) rows[i][j] = k[key][i];
  }
  return rows;
}
function resetLearning(keepResponses = false) {
  if (!keepResponses || learn.sig !== inputSig()) learn.resp = new Map();
  buildIndex(); learn.sig = inputSig(); learn.st = {}; learn.prior = priorRows(); learn.priorKind = 'desc'; learn.B = learn.prior.map(r => r.slice());
  learn.cal = null; learn.imuR = null;
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
  if (thr && thr.phase !== 'recover') return;   // the throw runs its own identification while falling
  if (learn.keep || learn.cal) {
    const imus = sensorsOf('imu'); const r0 = learn.imuR || (imus.length ? mean3(imus.map(knownPos)) : [0, 0, 0]);
    const r = run('identifyEffectiveness', learn.st, inputVector(), est.fAccel, est.fGyro, r0, dt, learn.prior, learn.cal ? Math.max(learn.memCal, learn.cal.total) : learn.memFlight, inputLags());
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
  const acts = actuators(), servos = acts.filter(c => c.type === 'tilt' && c.mode === 'auto');
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
      if (c.type === 'tilt' && c.mode === 'auto') servo.set(c.id, 0.3 * c.range * D2R * Math.sin(2 * Math.PI * (0.6 + 0.35 * i) * t));
    });
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
  if (good) { learn.mode = 'ident'; learn.keep = true; ctl.iAtt = [0, 0, 0]; ctl.iPos = [0, 0, 0]; }   // integrators were wound up for the old model
  else if (fromThrow) { learn.B = learn.prior.map(r => r.slice()); learn.st = {}; }   // keep the throw model
  else learn.mode = 'config';
  learn.flyB = null;
  endCalibration(good
    ? `Calibrated. On fresh test moves the learned model explains ${pc(learn.fit.rot)} of the rotation and ${pc(learn.fit.force)} of the force (${baseName}: ${pc(desc.rot)} and ${pc(desc.force)}). Flying on the learned model and still learning.`
    : `Calibration finished. The learned model explains ${pc(learn.fit.rot)} of the rotation and ${pc(learn.fit.force)} of the force, but ${baseName} does ${better ? 'nearly as well' : 'as well or better'} (${pc(desc.rot)}, ${pc(desc.force)}), so it keeps flying on ${fromThrow ? 'that' : 'the description'}.`);
}
function endCalibration(msg) { learn.cal = null; learn.flyB = null; learn.msg = msg; if (typeof renderLearn === 'function') renderLearn(true); }
const calExc = c => (learn.cal && learn.cal.now && learn.cal.now.exc && learn.cal.now.exc.get(c.id)) || 0;
function calServo(c) {   // servo angle the calibration is holding, or null
  const now = learn.cal && learn.cal.now;
  if (now && now.step && now.step.id === c.id) {
    const cal = learn.cal; if (cal.stepBase == null || cal.stepFor !== cal.cur) { cal.stepBase = act.get(c.id).thCmd; cal.stepFor = cal.cur; }
    return clamp(cal.stepBase + now.step.d, -c.range * D2R, c.range * D2R);
  }
  if (!now || !now.servo || !now.servo.has(c.id)) return null;
  // Sweeps ride on top of what the controller asks for, so it keeps its servo authority (a tricopter's
  // tail servo is its only real yaw control).
  return clamp(act.get(c.id).thCmd + learn.cal.now.servo.get(c.id), -c.range * D2R, c.range * D2R);
}
function ditherFor(c, t) {   // tiny excitation while learning in flight
  if (!learn.keep || learn.cal || !learn.dither) return 0;
  const j = learn.index.get(c.id); const i = j ? (j.u ?? j.a) : 0;
  return learn.dither * Math.sin(2 * Math.PI * (2.7 + 1.9 * i) * t + i);
}
// How well each actuator's learned effect matches the truth right now (1 = exact).
function matchScores() {
  const acts = actuators(); if (!acts.length || !learn.B) return [];
  const tb = trueB(), tc = acts.map(c => tb.get(c.id)), lc = acts.map(c => learnedCols(c));
  // Rows are weighted by their typical size: force rows together, roll/pitch together, yaw on its own.
  const rowMax = i => { let m = 0; for (const t of tc) for (const col of Object.values(t)) m = Math.max(m, Math.abs(col[i])); return m; };
  const groups = [[0, 1, 2], [3, 4], [5]], scale = new Array(6);
  for (const g of groups) { const m = Math.max(1e-6, ...g.map(rowMax)); for (const i of g) scale[i] = 1 / m; }
  return acts.map((c, k) => {
    let e = 0, v = 0;
    for (const key of Object.keys(tc[k])) for (let i = 0; i < 6; i++) { e += ((lc[k][key][i] - tc[k][key][i]) * scale[i]) ** 2; v += (tc[k][key][i] * scale[i]) ** 2; }
    return { c, match: clamp(1 - Math.sqrt(e / Math.max(v, 1e-9)), 0, 1) };
  });
}

const servoHeld = () => !!(learn.cal && learn.cal.now && learn.cal.now.holdServos);

/* ───────── actuator tests ───────── */
// Records one test window: what was sent to the actuator under test, and the drone's response along
// that actuator's effect. The fits run when the stage ends and feed the controller straight away.
function recordTest(s, dt) {
  const cal = learn.cal, c = s.rec.c, st = act.get(c.id); if (!st || !learn.mf || !learn.mf.f) return;
  if (!cal.win) {
    let col;
    if (s.rec.kind === 'motor') col = colAt(c, thSeen(c));
    else { const k = colsFor(c), th = thSeen(c); col = scl6(add6(scl6(k.a, -Math.sin(th)), scl6(k.b, Math.cos(th))), st.v || 0); }
    cal.win = { c, kind: s.rec.kind, col, u: [], cmd: [], y: [], dt, cmd0: servoTarget(c) };
    (cal.tests || (cal.tests = [])).push(cal.win);
  }
  const w = cal.win, y6 = [...learn.mf.f, ...learn.mf.a];
  w.u.push(st.u); w.cmd.push(servoTarget(c) - w.cmd0); w.y.push(project(w.col, y6));
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
  for (const c of actuators()) {
    if (c.type !== 'tilt') continue;
    const wins = (cal.tests || []).filter(w => w.c === c && w.kind === 'servo' && !w.bad && w.cmd.length > 50);
    if (!wins.length) continue;
    const r = run('identifyServoResponse', wins, wins[0].dt);
    const old = learn.resp.get(c.id) || {};
    if (r.fit < 0.4 || !(r.gain > 0.3 && r.gain < 3)) { learn.resp.set(c.id, { ...old, fitS: r.fit }); continue; }
    learn.resp.set(c.id, { ...old, rate: r.rate, lag: r.lag, fitS: r.fit });
    const st = act.get(c.id); if (st) st.pst = { h: st.thHat, th: st.thHat };
  }
}
// When the throttle curve changes, "one unit of input" changes size: x_new = ratio·x_old. Learned
// columns shrink by the same ratio so the model predicts the same thrust. The airframe description is
// already in true thrust units and stays as it is.
function rescaleInput(c, ratio) {
  if (!(ratio > 0.2 && ratio < 5) || Math.abs(ratio - 1) < 1e-6) return;
  const ix = learn.index.get(c.id); if (!ix) return; const js = Object.values(ix), s = 1 / ratio;
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
  // Every actuator once; servo rotors near one end of their range. Then each servo rotor again near the other end, so both halves
  // of a tilting rotor (u·cosθ and u·sinθ) are seen. A servo swings across right after its first
  // pulse, while the others are pulsed, so nobody waits for a servo.
  const acts = actuators(), plan = [];
  for (const c of acts) plan.push({ c, servo: c.type === 'tilt' && c.mode === 'auto' ? -0.9 * c.range * D2R : null });
  for (const c of acts) if (c.type === 'tilt' && c.mode === 'auto') plan.push({ c, servo: 0.9 * c.range * D2R, second: true });
  return plan;
}
const throwPlanTime = plan => plan.length * 0.1;   // rough length of the pulse sequence [s]
function startThrow() {                  // call right after resetSim()
  thr = { phase: 'hand', t: 0, plan: throwPlan(), i: 0, step: 'move', ts: 0, w0: null, st: {}, res: null, zMax: 0, tRel: 0 };
  learn.mode = 'config'; learn.msg = 'In the hand, motors off. The drone knows its sensors and how many actuators it has, nothing else.';
  for (const a of act.values()) { a.u = 0; a.Tcmd = 0; a.T = 0; }
}
function releaseThrow() {
  const v0 = Math.sqrt(2 * G * Math.max(0.3, throwCfg.height - S.p[2]));
  let ax = [randn(), randn(), randn()]; if (nrm(ax) < 1e-6) ax = [1, 0, 0];
  S.v = [0.3 * randn(), 0.3 * randn(), v0]; S.w = scl(unit(ax), throwCfg.spin);
  thr.phase = 'free'; thr.t = 0; thr.zRel = S.p[2];
  learn.msg = 'Thrown. Near the top of the arc it pulses each motor on its own and fits what each one does from the gyro and accelerometer.';
}
const throwBusy = () => !!thr && thr.phase !== 'recover';
// Throttle and servo commands while the throw runs open loop. Returns true when it has set them.
function throwTick(dt) {
  if (!thr) return false;
  thr.t += dt; thr.zMax = Math.max(thr.zMax, S.p[2]);
  const acts = actuators(), cmd = new Map(acts.map(c => [c.id, 0]));
  const servo = new Map();
  if (thr.plan) for (let k = 0; k < thr.plan.length; k++) {   // servo rotors: one end until their first pulse is over, then the other
    const P = thr.plan[k]; if (P.servo == null || P.second) continue;
    const firstDone = thr.phase === 'recover' || k < thr.i || (k === thr.i && thr.step === 'rest');
    servo.set(P.c.id, firstDone ? -P.servo : P.servo);
  }
  if (thr.phase === 'hand') { if (thr.t > 0.8) releaseThrow(); }
  else if (thr.phase === 'free' || thr.phase === 'excite') {
    // Pulse around the top of the throw: climbing or falling air through the props changes their thrust.
    const tPlan = throwPlanTime(thr.plan);
    if (thr.phase === 'free' && thr.t > 0.06 && est.v[2] < G * tPlan / 2) { thr.phase = 'excite'; thr.ts = thr.t; }
    const vb = m3v(m3T(est.R), est.v);
    if (thr.phase === 'free') run('identifyThrow', thr.st, inputVector(), est.fAccel, est.fGyro, vb, dt, false);
    if (thr.phase === 'excite') {
      const P = thr.plan[thr.i], el = thr.t - thr.ts;
      if (P) {
        if (thr.step === 'move') {   // only waits if the servo isn't at its pulse angle yet
          const th = P.servo != null ? thSeen(P.c) : 0;
          if (P.servo == null || Math.abs(th - P.servo) < 2 * D2R || el > 0.15) { thr.step = 'on'; thr.ts = thr.t; thr.w0 = est.fGyro.slice(); }
        } else if (thr.step === 'on') {
          // pulse until the rotation it causes reaches dwMax (stays well inside the gyro's range), 12–80 ms
          const dw = nrm(sub(est.fGyro, thr.w0));
          if ((el > 0.012 && dw > throwCfg.dwMax) || el > 0.08) { thr.step = 'rest'; thr.ts = thr.t; } else cmd.set(P.c.id, throwCfg.amp);
        } else if (el > 0.035) { thr.i++; thr.step = 'move'; thr.ts = thr.t; }
      }
      const done = thr.i >= thr.plan.length;
      const x = inputVector();
      thr.res = run('identifyThrow', thr.st, x, est.fAccel, est.fGyro, vb, dt, done);
      if (done) finishThrow();
    }
  }
  if (thr && thr.phase !== 'recover') {
    for (const c of acts) {
      const st = act.get(c.id); if (!st) continue;
      setThrottle(c, st, cmd.get(c.id));
      if (c.type === 'tilt') st.thCmd = servo.get(c.id) ?? 0;
    }
    return true;
  }
  return false;
}
function finishThrow() {
  const r = thr.res, ok = r && r.fitR > 0.6 && r.fitF > 0.4;
  thr.phase = 'recover'; thr.tRec = thr.t; ctl.iAtt = [0, 0, 0]; ctl.iPos = [0, 0, 0];
  const pc = v => Math.round(v * 100) + '%';
  if (ok) {
    learn.B = r.B.map(row => row.slice()); learn.prior = r.B.map(row => row.slice()); learn.st = {};
    learn.mode = 'ident'; learn.keep = true; learn.imuR = r.r.slice();
    thr.msg = `Identified in ${(thr.t).toFixed(2)} s of free fall: the fit explains ${pc(r.fitR)} of the rotation and ${pc(r.fitF)} of the force, motor lag ≈ ${Math.round(r.tau * 1000)} ms, IMU ${(nrm(r.r) * 100).toFixed(1)} cm from the balance point (true ${(nrm(trueImuOffset()) * 100).toFixed(1)} cm).`;
  } else {
    learn.mode = 'config';
    thr.msg = `The free-fall fit was poor (rotation ${pc(r ? r.fitR : 0)}, force ${pc(r ? r.fitF : 0)}), so it catches itself on the airframe description instead.`;
  }
  learn.msg = thr.msg + ' Recovering…';
}
function trueImuOffset() {   // mean IMU position relative to the true CoG, body frame (for the report only)
  const imus = sensorsOf('imu'); if (!imus.length) return [0, 0, 0];
  return sub(mean3(imus.map(c => c.pos)), truth.c);
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
