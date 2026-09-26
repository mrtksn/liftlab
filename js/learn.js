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
  memCal: 4, memFlight: 30,   // forgetting time [s]: at least the whole calibration run, then 30 s in flight
  st: {}, B: null, prior: null, sig: '', index: new Map(), n: 0,
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
    const u = st.u || 0;
    if (c.type === 'tilt') { x[ix.a] = u * Math.cos(st.th); x[ix.b] = u * Math.sin(st.th); } else x[ix.u] = u;
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
const colsFor = c => (flyingLearned() && learnedCols(c)) || describedCols(c);
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
function priorRows() {   // the description, as 6 rows × inputs: identification starts from here
  const rows = [0, 1, 2, 3, 4, 5].map(() => new Array(learn.n).fill(0));
  for (const c of actuators()) {
    const k = describedCols(c), ix = learn.index.get(c.id);
    for (const [key, j] of Object.entries(ix)) for (let i = 0; i < 6; i++) rows[i][j] = k[key][i];
  }
  return rows;
}
function resetLearning() {
  buildIndex(); learn.sig = inputSig(); learn.st = {}; learn.prior = priorRows(); learn.B = learn.prior.map(r => r.slice());
  learn.cal = null;
}

/* ───────── each control step ───────── */
function learnStep(dt) {
  if (learn.sig !== inputSig()) { resetLearning(); learn.msg = 'The actuators changed, so learning restarted from the airframe description.'; }
  if (!est.haveImu || !learn.n || S.crashed) { if (learn.cal && S.crashed) endCalibration('Calibration stopped: the drone crashed.'); return; }
  if (learn.keep || learn.cal) {
    const imus = sensorsOf('imu'); const r0 = imus.length ? mean3(imus.map(knownPos)) : [0, 0, 0];
    const r = run('identifyEffectiveness', learn.st, inputVector(), est.fAccel, est.fGyro, r0, dt, learn.prior, learn.cal ? Math.max(learn.memCal, learn.cal.total) : learn.memFlight);
    learn.B = r.B;
  }
  if (learn.cal) calibrationTick(dt);
}

/* ───────── calibration cycle ───────── */
// Staged excitation while hovering: settle, pulse each actuator in turn, sweep the servos, excite
// everything at once, then validate on a fresh signal the model hasn't been fitted to yet.
function startCalibration() {
  if (S.crashed) return;
  resetLearning();
  const acts = actuators(), servos = acts.filter(c => c.type === 'tilt' && c.mode === 'auto');
  const seg = [];
  seg.push({ stage: 'Settling', dur: 1, at: () => null });
  for (let rep = 0; rep < 2; rep++) acts.forEach(c => seg.push({ stage: 'Pulsing each motor', dur: 0.45, at: t => ({ exc: new Map([[c.id, t < 0.12 ? 0.07 : t < 0.24 ? -0.07 : 0]]) }) }));
  servos.forEach(c => seg.push({ stage: 'Sweeping servos', dur: 2.5, at: t => ({ servo: new Map([[c.id, 0.4 * c.range * D2R * Math.sin(2 * Math.PI * 0.8 * t)]]) }) }));
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
  learn.cal = { seg, t: 0, total: t0, now: null, sums: { e: [0, 0, 0, 0, 0, 0], d: [0, 0, 0, 0, 0, 0], y: [0, 0, 0, 0, 0, 0], y2: [0, 0, 0, 0, 0, 0], n: 0 } };
  learn.msg = '';
}
function calibrationTick(dt) {
  const cal = learn.cal;
  const tilt = Math.acos(clamp(dot(m3v(est.R, ctlAxis()), [0, 0, 1]), -1, 1));
  if (tilt > 0.52 || nrm(est.w) > 4) { cal.now = null; cal.held = true; return; }   // tilted past 30° or spinning: pause the excitation
  cal.held = false;
  cal.t += dt;
  const s = cal.seg.find(x => cal.t >= x.t0 && cal.t < x.t0 + x.dur);
  if (!s) return finishCalibration();
  cal.stage = s.stage; cal.now = s.at(cal.t - s.t0) || null;
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
  if (good) { learn.mode = 'ident'; learn.keep = true; ctl.iAtt = [0, 0, 0]; ctl.iPos = [0, 0, 0]; }   // integrators were wound up for the old model
  endCalibration(good
    ? `Calibrated. On fresh test moves the learned model explains ${pc(learn.fit.rot)} of the rotation and ${pc(learn.fit.force)} of the force (the airframe description: ${pc(desc.rot)} and ${pc(desc.force)}). Flying on the learned model and still learning.`
    : `Calibration finished. The learned model explains ${pc(learn.fit.rot)} of the rotation and ${pc(learn.fit.force)} of the force, but the airframe description does ${better ? 'nearly as well' : 'as well or better'} (${pc(desc.rot)}, ${pc(desc.force)}), so it keeps flying on the description.`);
}
function endCalibration(msg) { learn.cal = null; learn.msg = msg; if (typeof renderLearn === 'function') renderLearn(true); }
const calExc = c => (learn.cal && learn.cal.now && learn.cal.now.exc && learn.cal.now.exc.get(c.id)) || 0;
function calServo(c) {   // servo angle the calibration is holding, or null
  if (!learn.cal || !learn.cal.now || !learn.cal.now.servo || !learn.cal.now.servo.has(c.id)) return null;
  return clamp(learn.cal.now.servo.get(c.id), -c.range * D2R, c.range * D2R);
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
