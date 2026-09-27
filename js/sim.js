'use strict';
// Airframe configuration, simulation state, controller plumbing, physics stepping and the
// flight-envelope check. All physics and control formulas come from laws.js through run().

/* ───────── configuration ───────── */
let uid = 1;
const base = o => Object.assign({ id: uid++ }, o);
function mkMotor(name, x, y, z, o = {}) { return withProp(base(Object.assign({ type: 'motor', name, pos: [x, y, z], tilt: 0, az: 0, tmax: 6, kappa: 0.016, spin: 1, tau: 0.03, curve: 0.3, fm: 0.6, mass: 0.06, health: 100, healthKnown: true }, o))); }
function withProp(c) { if (!c.prop) c.prop = +clamp(0.035 * Math.sqrt(c.tmax), 0.05, 0.2).toFixed(3); return c; }
function mkMass(name, x, y, z, o = {}) { return base(Object.assign({ type: 'mass', name, pos: [x, y, z], shape: 'box', mass: 0.2, size: [0.08, 0.05, 0.03], radius: 0.04, length: 0.1, known: true }, o)); }
function mkHang(name, x, y, z, o = {}) { return base(Object.assign({ type: 'hang', name, pos: [x, y, z], length: 0.5, mass: 0.15, known: true }, o)); }
const r3 = v => +v.toFixed(3);
const PRESETS = {
  quadx: { label: 'Quad X', build() {
    const r = 0.2; const c = [45, 135, 225, 315].map((a, i) => mkMotor('M' + (i + 1), r3(r * cosd(a)), r3(r * sind(a)), 0.02, { spin: i % 2 ? -1 : 1 }));
    c.push(mkMass('Battery', 0, 0, -0.035, { mass: 0.18, size: [0.1, 0.04, 0.03] })); return { frame: 0.45, comps: c.concat(defaultSensors()), mode: 'tilt' }; } },
  hex: { label: 'Hexacopter', build() {
    const r = 0.25; const c = [0, 60, 120, 180, 240, 300].map((a, i) => mkMotor('M' + (i + 1), r3(r * cosd(a)), r3(r * sind(a)), 0.02, { spin: i % 2 ? -1 : 1, tmax: 5 }));
    c.push(mkMass('Battery', 0, 0, -0.04, { mass: 0.26, size: [0.13, 0.045, 0.035] })); return { frame: 0.55, comps: c.concat(defaultSensors()), mode: 'tilt' }; } },
  tri: { label: 'Tricopter (yaw servo)', build() {
    const r = 0.22; const c = [mkMotor('Left', r3(r * cosd(60)), r3(r * sind(60)), 0.02, { spin: 1, tmax: 7 }),
      mkMotor('Right', r3(r * cosd(-60)), r3(r * sind(-60)), 0.02, { spin: -1, tmax: 7 }),
      ...mkServoMotor('Tail', -r, 0, 0.02, { hingeAz: 0, range: 30, rate: 300 }, { spin: 1, tmax: 7 })];
    c.push(mkMass('Battery', 0.02, 0, -0.035, { mass: 0.18, size: [0.1, 0.04, 0.03] })); return { frame: 0.4, comps: c.concat(defaultSensors()), mode: 'tilt' }; } },
  heli: { label: 'Main lifter + 4 steering motors', build() {
    const c = [mkMotor('Main', 0, 0, 0.06, { tmax: 22, kappa: 0.03, mass: 0.22, spin: 1, tau: 0.06, prop: 0.2 })]; const r = 0.26;
    [0, 90, 180, 270].forEach((a, i) => c.push(...mkServoMotor('S' + (i + 1), r3(r * cosd(a)), r3(r * sind(a)), 0.02, { hingeAz: a, range: 45, rate: 300 }, { tmax: 4, kappa: 0.012, spin: i % 2 ? 1 : -1 })));
    c.push(mkMass('Battery', 0, 0, -0.04, { mass: 0.3, size: [0.12, 0.05, 0.035] }));
    const sn = defaultSensors(); sn[1].pos = [-0.12, -0.12, 0.08];   // compass on a boom, away from the big main motor
    return { frame: 0.5, comps: c.concat(sn), mode: 'tilt' }; } },
  indoor: { label: 'Indoor quad (optical flow, no GPS)', build() {
    const r = 0.2; const c = [45, 135, 225, 315].map((a, i) => mkMotor('M' + (i + 1), r3(r * cosd(a)), r3(r * sind(a)), 0.02, { spin: i % 2 ? -1 : 1 }));
    c.push(mkMass('Battery', 0, 0, -0.035, { mass: 0.18, size: [0.1, 0.04, 0.03] }));
    const sn = defaultSensors().filter(x => x.kind !== 'fix');
    sn.push(mkSensor('flow', 'Flow', 0, -0.03, -0.03));
    return { frame: 0.45, comps: c.concat(sn), mode: 'tilt' }; } },
  tiltquad: { label: 'Tilt-rotor quad (thrust vectoring)', build() {
    const r = 0.2; const c = [45, 135, 225, 315].flatMap((a, i) => mkServoMotor('T' + (i + 1), r3(r * cosd(a)), r3(r * sind(a)), 0.02, { hingeAz: a, range: 30, rate: 360 }, { spin: i % 2 ? -1 : 1, tmax: 6 }));
    c.push(mkMass('Battery', 0, 0, -0.035, { mass: 0.18, size: [0.1, 0.04, 0.03] })); return { frame: 0.45, comps: c.concat(defaultSensors()), mode: 'level' }; } },
};
const cfg = { frame: { mass: 0.45 }, comps: [] };
let mode = 'tilt';
const setpoint = { x: 0, y: 0, z: 1.5, yaw: 0 };
const envr = { wind: 0, windDir: 0, texture: 0.8, light: 1 };   // texture and light matter to optical flow

/* ───────── state ───────── */
const S = { p: [0, 0, 1.5], v: [0, 0, 0], q: [1, 0, 0, 0], w: [0, 0, 0], acc: [0, 0, 0], wdot: [0, 0, 0], batt: {}, battK: 1, rotors: [], crashed: null, t: 0, steps: 0 };
const act = new Map();   // id -> { T, Tcmd, th, thCmd }
const pend = new Map();  // id -> { p, v, Tn }
// Mixed steering: the servos take up to `share` of the sideways force; `rho` tracks how much of what they
// were asked for they actually made (low-passed), so the body leans more when they can't keep up.
const steerMix = { share: 0.5, rho: 1 };
const mixShare = () => steerMix.share * steerMix.rho;
const ctl = { iPos: [0, 0, 0], iAtt: [0, 0, 0], wDes: [0, 0, 0, 0, 0, 0], sat: false, eAtt: 0, vRef: [0, 0, 0] };  // vRef: pilot's commanded velocity
let truth = null, model = null, nb = [0, 0, 1];
let onCrash = () => {};

const actuators = () => cfg.comps.filter(c => c.type === 'motor');   // thrust inputs; servo joints are in joints.js
const hModel = c => c.healthKnown ? c.health / 100 : 1;
function actDir(c) { const t = c.tilt * D2R, a = c.az * D2R; return [Math.sin(t) * Math.cos(a), Math.sin(t) * Math.sin(a), Math.cos(t)]; }   // thrust axis at rest
function rotorNow(c, ang = angleTrue) { const P = poseOf(c, ang); return { p: P.p, d: m3v(P.R, actDir(c)) }; }   // where a rotor is and points
function wrenchCol(pos, d, spin, kappa, cog) { const w = run('rotorWrench', d, sub(pos, cog), 1, spin, kappa); return [w.F[0], w.F[1], w.F[2], w.tau[0], w.tau[1], w.tau[2]]; }
const scl6 = (c, s) => c.map(x => x * s);

/* ───────── mass properties ───────── */
function boxI(m, a, b, c) { return [m * (b * b + c * c) / 12, 0, 0, 0, m * (a * a + c * c) / 12, 0, 0, 0, m * (a * a + b * b) / 12]; }
function shapeI(c) {
  const m = c.mass;
  if (c.shape === 'sphere') { const I = 0.4 * m * c.radius * c.radius; return [I, 0, 0, 0, I, 0, 0, 0, I]; }
  if (c.shape === 'cylinder') { const r = c.radius, L = c.length, ix = m * (3 * r * r + L * L) / 12; return [ix, 0, 0, 0, ix, 0, 0, 0, m * r * r / 2]; }
  return boxI(m, c.size[0], c.size[1], c.size[2]);
}
// Mass, CoG and inertia with every part where its joints put it: the true angles for the physics, the
// angles the flight software believes for its model.
// A thin rod's inertia about its middle: m L²/12 across it, nothing along it.
function rodI(l) { const d = linkDir(l), k = l.mass * l.length * l.length / 12; return [0, 1, 2].flatMap(i => [0, 1, 2].map(j => k * ((i === j ? 1 : 0) - d[i] * d[j]))); }
function massProps(which) {
  const ang = which === 'truth' ? angleTrue : angleSeen;
  const items = [{ m: cfg.frame.mass, r: [0, 0, 0], I: boxI(cfg.frame.mass, 0.12, 0.12, 0.04) }];
  for (const c of cfg.comps) {
    const pose = () => poseOf(c, ang);
    if (c.type === 'motor' || c.type === 'joint') items.push({ m: c.mass, r: pose().p, I: null });
    else if (c.type === 'mass') { if (which === 'truth' || c.known) { const P = pose(); items.push({ m: c.mass, r: P.p, I: m3m(m3m(P.R, shapeI(c)), m3T(P.R)) }); } }
    else if (c.type === 'hang') { if (which === 'model' && c.known) items.push({ m: c.mass, r: pose().p, I: null }); }
    else if (c.type === 'link') { if (which === 'truth' || c.known) { const P = poseOf(c, ang); items.push({ m: c.mass, r: posePoint(c, add(c.pos, scl(linkDir(c), c.length / 2)), ang).p, I: m3m(m3m(P.R, rodI(c)), m3T(P.R)) }); } }
  }
  let m = 0, cm = [0, 0, 0]; for (const it of items) { m += it.m; cm = add(cm, scl(it.r, it.m)); } cm = scl(cm, 1 / m);
  const J = [0, 0, 0, 0, 0, 0, 0, 0, 0];
  for (const it of items) {
    if (it.I) for (let i = 0; i < 9; i++) J[i] += it.I[i];
    const d = sub(it.r, cm), dd = dot(d, d);
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) J[i * 3 + j] += it.m * ((i === j ? dd : 0) - d[i] * d[j]);
  }
  return { m, c: cm, J, Jinv: m3inv(J) };
}
function nominalAxis() {
  let s = [0, 0, 0];
  for (const c of actuators()) s = add(s, scl(rotorNow(c, restAngle).d, c.tmax * hModel(c)));   // at rest (manual joints at their set angle)
  return nrm(s) > 1e-9 ? unit(s) : [0, 0, 1];
}
function syncRuntime() {
  const ids = new Set(cfg.comps.map(c => c.id));
  for (const k of [...act.keys()]) if (!ids.has(k)) act.delete(k);
  for (const k of [...jst.keys()]) if (!ids.has(k)) jst.delete(k);
  for (const k of [...pend.keys()]) if (!ids.has(k)) pend.delete(k);
  const R = qmat(S.q);
  for (const c of cfg.comps) {
    if (c.type === 'motor' && !act.has(c.id)) act.set(c.id, { T: 0, Tcmd: 0, u: 0, v: 0, k: 1 });
    if (c.type === 'joint' && !jst.has(c.id)) { const t0 = restAngle(c), tr = t0 + (c.offset || 0) * D2R; jst.set(c.id, { th: tr, thR: tr, thCmd: t0, thHat: t0, pst: {}, rate: 0, acc: 0 }); }
    if (c.type === 'hang' && !pend.has(c.id)) { const a = add(S.p, m3v(R, sub(posNow(c), truth.c))); pend.set(c.id, { p: [a[0], a[1], a[2] - c.length], v: S.v.slice(), Tn: 0 }); }
  }
  syncSensors();
}
function reseatPend(c) {
  const st = pend.get(c.id); if (!st) return;
  const R = qmat(S.q); const a = add(S.p, m3v(R, sub(posNow(c), truth.c)));
  let d = sub(st.p, a); if (nrm(d) < 1e-6) d = [0, 0, -1];
  st.p = add(a, scl(unit(d), c.length)); st.v = S.v.slice();
}
function recomputeProps() {
  const R = qmat(S.q); const hub = truth ? sub(S.p, m3v(R, truth.c)) : [setpoint.x, setpoint.y, setpoint.z];
  truth = massProps('truth'); model = massProps('model');
  S.p = add(hub, m3v(R, truth.c)); nb = nominalAxis(); syncRuntime();
}

/* ───────── control ───────── */

// Weights for how the allocation breaks ties (see allocationPreferences). horizon: how far ahead a servo
// move is planned [s]; what it can reach in that time bounds each step's servo change.
const allocPrefs = { allowance: 0.02, efficiency: 0.02, servoMove: 0.01, horizon: 0.08 };
const powerFull = c => Math.pow(c.tmax, 1.5) / ((c.fm || 0.6) * Math.sqrt(2 * 1.225 * Math.PI * propR(c) ** 2));   // W at full thrust
// Each input's share of the steering: the size of its effect on rotation (over its whole range), relative to the largest.
function setAuthority(rows) {
  const a = rows.map(r => Math.hypot(r.col[3], r.col[4], r.col[5]) * (r.hi - r.lo)), mx = Math.max(1e-9, ...a);
  rows.forEach((r, i) => { r.inp.authority = a[i] / mx; });
}
function servoReach(j) { const m = servoModelHat(j); return m.rate * Math.max(0.005, allocPrefs.horizon - m.lag); }
function allocate(w, cm) {
  // w: wanted [force; torque] in the controller's model units. Columns are acceleration per full thrust.
  const wa = (() => { const f = scl([w[0], w[1], w[2]], 1 / cm.m); const a = m3v(cm.Jinv, [w[3], w[4], w[5]]); return [...f, ...a]; })();
  const acts = actuators();
  const thrustIn = c => { const st = act.get(c.id); return { col: colAt(c), lo: 0, hi: 1, inp: { kind: 'thrust', x: st.v || 0, lo: 0, hi: 1, power: powerFull(c) } }; };
  // Stage 1: servo joints. Each steering joint adds a small angle change δ as an input. Its effect is what
  // turning it does to every motor it carries, Σ thrust × d(column)/dθ, and δ is bounded by how far the
  // servo can get within the planning horizon.
  const sj = steerJoints();
  if (sj.length) {
    const rows = acts.map(thrustIn), who = acts.map(() => null);
    for (const j of sj) {
      const th = angleSeen(j), R = j.range * D2R, reach = servoReach(j);
      let d = [0, 0, 0, 0, 0, 0];
      for (const c of motorsUnder(j)) d = add6(d, scl6(dColAt(c, j), Math.max(act.get(c.id).v || 0, 0.02)));
      const lo = Math.min(0, Math.max(-R - th, -reach)), hi = Math.max(0, Math.min(R - th, reach));
      rows.push({ col: d, lo, hi, inp: { kind: 'servo', x: 0, lo, hi, th, range: R, reach } });
      who.push(j);
    }
    setAuthority(rows);
    const pull = run('allocationPreferences', rows.map(r => r.inp), allocPrefs);
    const x = run('allocation', rows.map(r => r.col), rows.map(r => r.lo), rows.map(r => r.hi), wa, mode, pull);
    if (!servoHeld()) who.forEach((j, i) => {   // servos stay put while a calibration step holds them
      if (j) jst.get(j.id).thCmd = clamp(angleSeen(j) + x[i], -j.range * D2R, j.range * D2R);
    });
  }
  // Stage 2: thrust for every motor at the joints' angles now (measured, or predicted without feedback),
  // so the motors cover whatever a moving servo hasn't reached yet.
  const rows = acts.map(thrustIn);
  setAuthority(rows);
  const pull = run('allocationPreferences', rows.map(r => r.inp), allocPrefs);
  const u = holdU(run('allocation', rows.map(r => r.col), rows.map(() => 0), rows.map(() => 1), wa, mode, pull));   // held during calibration pulses
  if (mode === 'mixed') {   // how much of the asked-for sideways force this step's thrusts actually make
    const dem = Math.hypot(wa[0], wa[1]);
    if (dem > 0.2) {
      const gx = rows.reduce((s, r, j) => s + r.col[0] * u[j], 0), gy = rows.reduce((s, r, j) => s + r.col[1] * u[j], 0);
      const ratio = clamp((gx * wa[0] + gy * wa[1]) / (dem * dem), 0, 1);
      steerMix.rho += (ratio - steerMix.rho) * Math.min(1, 0.001 / 0.3);   // about 0.3 s to settle
    } else steerMix.rho += (1 - steerMix.rho) * 0.001 / 2;                 // drift back when not asked
  }
  ctl.sat = false;
  acts.forEach((c, i) => {
    const st = act.get(c.id);
    // u[i] is thrust as a fraction of max; the learned throttle curve turns it into the throttle to send
    st.u = clamp(run('thrustLinearization', u[i], curveHat(c)) + calExc(c) + ditherFor(c, S.t), 0, 1);   // plus calibration or learning excitation
    setThrottle(c, st, st.u);
    if (u[i] >= 0.995) ctl.sat = true;
  });
}

// The throttle sent, what the controller believes it gives (thrust fraction), and what the motor will really make.
function setThrottle(c, st, u) { st.u = u; st.v = believedThrust(u, curveHat(c)); st.Tcmd = c.tmax * run('throttleCurve', u, c.curve || 0); }
// Servo angles the flight software uses: the feedback reading, or its own prediction from what it commanded.
function updateServoBelief(dt) {
  for (const j of joints()) {
    const st = jst.get(j.id); if (!st) continue;
    const m = servoModelHat(j), prev = angleSeen(j);
    st.thHat = run('servoPredictor', st.pst, jointTarget(j), m.rate, m.lag, dt);
    st.rateHat = dt > 0 ? (angleSeen(j) - prev) / dt : 0;
  }
  if (joints().length) model = massProps('model');   // parts on joints move the CoG the controller believes in
}
function hubState(R) { const hub = sub(S.p, m3v(R, truth.c)); const vh = sub(S.v, m3v(R, crs(S.w, truth.c))); return { hub, vh }; }
function control(dt) {
  senseAndEstimate(dt);
  updateServoBelief(dt);
  learnStep(dt);
  if (S.crashed) { for (const a of act.values()) { a.Tcmd = 0; a.u = 0; } return; }
  if (throwTick(dt)) return;             // throw start: open loop until it has identified itself
  throwRecoverCheck();
  // The flight software sees only the estimate, unless you hand it the ground truth.
  let R, w, hub, vh;
  if (sensing === 'truth') { R = qmat(S.q); w = S.w; ({ hub, vh } = hubState(R)); }
  else { R = est.R; w = est.w; hub = est.p; vh = est.v; }
  const cm = ctlModel(), axis = ctlAxis();
  const RT = m3T(R);
  const ep = sub([setpoint.x, setpoint.y, setpoint.z], hub);
  for (let i = 0; i < 3; i++) ctl.iPos[i] = clamp(ctl.iPos[i] + ep[i] * dt, i < 2 ? -2 : -5, i < 2 ? 2 : 5);   // vertical has room to trim out an unknown hover throttle
  const Fd = run('positionControl', ep, sub(vh, ctl.vRef), ctl.iPos, cm.m, G);
  const nd = unit(run('thrustAxisTarget', Fd, mode, mixShare()));
  let psi = setpoint.yaw * D2R;
  if (thr && thr.phase === 'recover') { const bx = m3v(R, [1, 0, 0]); psi = Math.atan2(bx[1], bx[0]); }   // catching a throw: get upright first, turn to the heading later
  const Rd = m3m(frameFrom(nd, [Math.cos(psi), Math.sin(psi), 0]), m3T(frameFrom(axis, [1, 0, 0])));
  const eR = run('attitudeError', R, Rd);
  ctl.eAtt = nrm(eR);
  for (let i = 0; i < 3; i++) ctl.iAtt[i] = clamp(ctl.iAtt[i] + eR[i] * dt, -0.5, 0.5);
  const tau = run('attitudeControl', eR, w, ctl.iAtt, cm.J);
  const f = run('forceDemand', m3v(RT, Fd), axis, mode);
  ctl.wDes = [f[0], f[1], f[2], tau[0], tau[1], tau[2]];
  allocate(ctl.wDes, cm);
}

/* ───────── physics ───────── */
const PDT = 0.0005;
function contactPoints() {   // where the airframe can touch the ground, with parts where their joints put them
  const pts = [[0, 0, -0.03]];
  for (const c of cfg.comps) {
    if (c.type === 'motor' || c.type === 'joint') { const p = posNow(c); pts.push([p[0], p[1], p[2] - 0.03]); }
    else if (c.type === 'mass') { const p = posNow(c), hz = c.shape === 'box' ? c.size[2] / 2 : c.shape === 'sphere' ? c.radius : c.length / 2; pts.push([p[0], p[1], p[2] - hz]); }
    else if (c.type === 'link') pts.push(posePoint(c, linkTip(c)).p, posNow(c));
  }
  return pts;
}
let cPts = [[0, 0, -0.03]];
const propR = c => c.prop || clamp(0.035 * Math.sqrt(c.tmax), 0.05, 0.2);   // prop radius [m]
const payloadR = c => 0.025 + 0.035 * Math.cbrt(c.mass);
function washParts() {   // parts the downwash can push: the hub plate and rigid masses (horizontal frontal area)
  const parts = [{ p: [0, 0, 0], area: 0.12 * 0.12 }];
  for (const c of cfg.comps) if (c.type === 'mass') parts.push({ p: posNow(c), area: c.shape === 'box' ? c.size[0] * c.size[1] : Math.PI * c.radius * c.radius });
  return parts;
}
function crash(why) { if (S.crashed) return; S.crashed = why; for (const a of act.values()) a.Tcmd = 0; onCrash(); }
function windVec() { return [envr.wind * cosd(envr.windDir), envr.wind * sind(envr.windDir), 0]; }

// Forces (world) and torques (body, about the CoG) from the rotors and their downwash on the frame.
// Used by the physics each step, and by trueB() to linearize the real airframe for comparison.
function rotorLoads(rotors, R, RT, wv, record) {
  let F = [0, 0, 0], tau = [0, 0, 0];
  for (const ro of rotors) {
    const r = sub(ro.p, truth.c);
    const vRotor = add(S.v, m3v(R, crs(S.w, r)));
    const u = add(m3v(RT, sub(wv, vRotor)), run('wakeVelocity', ro.p, rotors.filter(o => o !== ro)));   // air past this disc
    const ua = dot(u, ro.d);
    const h = add(S.p, m3v(R, r))[2];
    const ae = run('rotorAero', ro.T, ro.R, -ua, sub(u, scl(ro.d, ua)), h);
    if (record) { ro.st.k = ro.T > 1e-6 ? ae.T / ro.T : 1; ro.st.Teff = ae.T; }
    const rw = run('rotorWrench', ro.d, r, ae.T, ro.c.spin, ro.c.kappa);
    tau = add(tau, add(rw.tau, crs(r, ae.H))); F = add(F, m3v(R, add(rw.F, ae.H)));
  }
  for (const part of washParts()) {   // downwash on the hub and rigid masses
    const fw = run('wakeLoad', run('wakeVelocity', part.p, rotors), part.area);
    F = add(F, m3v(R, fw)); tau = add(tau, crs(sub(part.p, truth.c), fw));
  }
  return { F, tau };
}
function dynamics(dt) {
  if (thr && thr.phase === 'hand') { S.v = [0, 0, 0]; S.w = [0, 0, 0]; S.acc = [0, 0, 0]; S.wdot = [0, 0, 0]; return; }   // held still in the hand
  const R = qmat(S.q), RT = m3T(R);
  let F = run('gravity', truth.m, G), tau = [0, 0, 0];
  const wv = windVec(), acts = actuators();
  // Battery: the pack sags as it drains and under load.
  let load = 0, cap = 0; for (const c of acts) { load += act.get(c.id).T; cap += c.tmax; }
  S.battK = run('batteryModel', S.batt, cap ? load / cap : 0, dt);
  // Servo joints: the horn moves at the servo's speed, the part follows through the linkage. Moving parts
  // shift the CoG (the world CoG itself stays put: that's momentum), and turning them takes a torque the
  // frame feels the other way round.
  const js = joints();
  if (js.length) {
    let react = [0, 0, 0];
    for (const j of js) {
      const st = jst.get(j.id); if (!st) continue;
      const th0 = st.th, rate0 = st.rate || 0;
      st.thR = run('servoResponse', st.thR ?? st.th, jointTarget(j) + (j.offset || 0) * D2R, j.range * D2R, j.rate * D2R, dt);
      st.th = run('servoLinkage', st.th, st.thR, j.lag || 0, dt);
      st.rate = (st.th - th0) / dt; st.acc = (st.rate - rate0) / dt;
      if (Math.abs(st.acc) > 1e-6) {   // reaction: −I_axis·θ̈ about the joint's current axis
        const P = poseOf(j), a = m3v(P.R, jointAxis(j));
        let Ia = 0;
        for (const c of descendants(j)) {
          if (!(c.mass > 0) || c.type === 'hang') continue;
          const at = c.type === 'link' ? posePoint(c, add(c.pos, scl(linkDir(c), c.length / 2))).p : posNow(c);
          const r = sub(at, P.p), rp = sub(r, scl(a, dot(r, a)));
          const Iown = c.type === 'mass' ? dot(a, m3v(shapeI(c), a)) : c.type === 'link' ? dot(a, m3v(m3m(m3m(poseOf(c).R, rodI(c)), m3T(poseOf(c).R)), a)) : 0;
          Ia += c.mass * dot(rp, rp) + Iown;
        }
        react = add(react, scl(a, -Ia * st.acc));
      }
    }
    truth = massProps('truth'); cPts = contactPoints();
    tau = add(tau, react);
  }
  // Rotors: still-air thrust from the motors, then what the airflow does to it.
  const rotors = acts.map(c => {
    const st = act.get(c.id);
    st.T = run('motorResponse', st.T, st.Tcmd, c.tmax, c.tau, dt);
    const n = rotorNow(c);
    return { c, st, p: n.p, d: n.d, T: st.T * c.health / 100 * S.battK, R: propR(c) };
  });
  const ld = rotorLoads(rotors, R, RT, wv, true);
  F = add(F, ld.F); tau = add(tau, ld.tau);
  S.rotors = rotors;
  const dr = run('bodyDrag', S.v, wv, S.w); F = add(F, dr.F); tau = add(tau, dr.tau);
  for (const c of cfg.comps) {
    if (c.type !== 'hang') continue; const st = pend.get(c.id); if (!st) continue;
    const r = sub(posNow(c), truth.c); const aw = add(S.p, m3v(R, r)); const va = add(S.v, m3v(R, crs(S.w, r)));
    const dv = sub(st.p, aw), L = nrm(dv); let Fc = [0, 0, 0], Tn = 0;
    if (L > 1e-9) {
      const n = scl(dv, 1 / L);
      Tn = run('cableTension', L - c.length, dot(sub(st.v, va), n), c.mass);
      Fc = scl(n, Tn);
    }
    st.Tn = Tn; F = add(F, Fc); tau = add(tau, crs(r, m3v(RT, Fc)));
    const pb = add(m3v(RT, sub(st.p, S.p)), truth.c);                   // payload in the body frame, for the downwash
    const wash = m3v(R, run('wakeLoad', run('wakeVelocity', pb, rotors), Math.PI * payloadR(c) ** 2));
    const Fp = add(add(add(scl(Fc, -1), run('gravity', c.mass, G)), run('payloadDrag', st.v, wv)), wash);
    st.v = add(st.v, scl(Fp, dt / c.mass)); st.p = add(st.p, scl(st.v, dt));
    if (st.p[2] < 0.03) { st.p[2] = 0.03; if (st.v[2] < 0) st.v[2] = 0; st.v[0] *= 0.995; st.v[1] *= 0.995; }
  }
  for (const pt of cPts) {
    const r = sub(pt, truth.c); const pw = add(S.p, m3v(R, r)); if (pw[2] >= 0) continue;
    const vel = add(S.v, m3v(R, crs(S.w, r)));
    if (vel[2] < -3 && !S.crashed) crash(`Hit the ground at ${(-vel[2]).toFixed(1)} m/s.`);
    const fw = run('groundContact', -pw[2], vel); F = add(F, fw); tau = add(tau, crs(r, m3v(RT, fw)));
  }
  const rb = run('rigidBody', F, tau, truth.m, truth.J, truth.Jinv, S.w);
  S.acc = rb.a; S.wdot = rb.wdot;                        // what the accelerometers feel next
  S.v = add(S.v, scl(rb.a, dt)); S.p = add(S.p, scl(S.v, dt));
  S.w = add(S.w, scl(rb.wdot, dt));
  const dq = qmul(S.q, [0, S.w[0], S.w[1], S.w[2]]); S.q = qnorm(S.q.map((x, i) => x + 0.5 * dq[i] * dt));
  if (!S.crashed) {
    const up = dot(m3v(R, nb), [0, 0, 1]);
    if (!isFinite(S.p[0] + S.p[1] + S.p[2] + S.q[0])) { crash('The state became invalid (NaN). Check your edited formulas.'); S.p = [setpoint.x, setpoint.y, 0.2]; S.v = [0, 0, 0]; S.w = [0, 0, 0]; S.q = [1, 0, 0, 0]; }
    else if (up < -0.17 && !thr) crash('Flipped over. The actuators could not hold the attitude.');
    else if (nrm(S.w) > 35) crash('Spun out of control. Check yaw authority and spin directions.');
    else if (Math.abs(S.p[0]) > 40 || Math.abs(S.p[1]) > 40 || S.p[2] > 40) crash('Flew away from the target.');
  }
}
function physStep() { S.steps++; if (S.steps % 2 === 0) control(PDT * 2); dynamics(PDT); S.t += PDT; sampleSensors(PDT); if (S.steps % 40 === 0) pushHist(); }

function resetSim() {
  thr = null;
  // start with the nominal thrust axis pointing up at the target heading
  S.q = matToQuat(m3m(frameFrom([0, 0, 1], [cosd(setpoint.yaw), sind(setpoint.yaw), 0]), m3T(frameFrom(nb, [1, 0, 0]))));
  const R = qmat(S.q); S.p = add([setpoint.x, setpoint.y, setpoint.z], m3v(R, truth.c)); S.v = [0, 0, 0]; S.w = [0, 0, 0]; S.crashed = null; S.t = 0; S.steps = 0;
  ctl.iPos = [0, 0, 0]; ctl.iAtt = [0, 0, 0]; ctl.vRef = [0, 0, 0]; pend.clear(); act.clear(); jst.clear(); syncRuntime();
  S.batt = {}; S.battK = 1;
  resetEstimation(); resetLearning();
  for (let k = 0; k < 4; k++) {
    control(0);
    for (const c of actuators()) { const st = act.get(c.id); st.T = st.Tcmd; }
    for (const j of joints()) { const st = jst.get(j.id), t = jointTarget(j); st.th = st.thR = t + (j.offset || 0) * D2R; st.pst = {}; st.thHat = t; st.rate = 0; st.acc = 0; }
  }
  truth = massProps('truth'); model = massProps('model'); cPts = contactPoints();
  hist.t.length = hist.tilt.length = hist.err.length = hist.est.length = hist.util.length = 0; trail.length = 0;
  thr = null; if (launchMode === 'throw') startThrow();
}

/* ───────── flight envelope ───────── */
// Attainable accelerations form a zonotope: the sum of each input's segment [lo, hi]·column.
// Hover is feasible when the acceleration hover needs lies inside it. Headroom along each axis is
// the distance from that point to the zonotope boundary.
function envelopeCalc() {
  const k = mode === 'level' ? 6 : 4; const gens = [];   // mixed can always fall back on leaning, so it needs the 4 axes
  const sj = steerJoints();
  for (const c of actuators()) {
    const h = c.health / 100; if (h <= 0) continue;
    const toK = col => { const f = scl([col[0], col[1], col[2]], 1 / truth.m), al = m3v(truth.Jinv, [col[3], col[4], col[5]]); return k === 4 ? [dot(f, nb), al[0], al[1], al[2]] : [f[0], f[1], f[2], al[0], al[1], al[2]]; };
    const j = chainOf(c).find(x => sj.includes(x));
    if (j) {   // a rotor its nearest steering joint can swing: its thrust at the middle, plus the swing (linearized over the range)
      const at = th => { const n = rotorNow(c, x => x === j ? th : restAngle(x)); return wrenchCol(n.p, n.d, c.spin, c.kappa, truth.c); };
      const e = 1e-3, sb = Math.sin(j.range * D2R) * c.tmax * h;
      gens.push({ g: toK(at(0)), lo: 0, hi: c.tmax * h });
      gens.push({ g: toK(at(e).map((v, i) => (v - at(-e)[i]) / (2 * e))), lo: -sb, hi: sb });
    } else { const n = rotorNow(c); gens.push({ g: toK(wrenchCol(n.p, n.d, c.spin, c.kappa, truth.c)), lo: 0, hi: c.tmax * h }); }
  }
  let mp = 0, treq = [0, 0, 0];
  for (const c of cfg.comps) if (c.type === 'hang') { mp += c.mass; treq = add(treq, crs(sub(posNow(c), truth.c), scl(nb, c.mass * G))); }
  const fz = (truth.m + mp) * G / truth.m, areq = m3v(truth.Jinv, treq), freq = scl(nb, fz);
  const w = k === 4 ? [fz, areq[0], areq[1], areq[2]] : [freq[0], freq[1], freq[2], areq[0], areq[1], areq[2]];
  const labels = k === 4 ? ['Climb', 'Roll', 'Pitch', 'Yaw'] : ['Fwd/back', 'Left/right', 'Climb', 'Roll', 'Pitch', 'Yaw'];
  const units = k === 4 ? ['g', 'ang', 'ang', 'ang'] : ['lin', 'lin', 'g', 'ang', 'ang', 'ang'];
  const res = { k, labels, units, w, rank: rankOf(gens.map(x => x.g), k), n: gens.length };
  if (res.rank < k) {
    res.verdict = 'bad'; res.title = 'Not controllable';
    res.why = k === 6 ? `Stay-level mode needs thrust vectoring. The actuators control only ${res.rank} of 6 axes, so the craft cannot push sideways without tilting.`
      : `The actuators control only ${res.rank} of 4 axes (climb, roll, pitch, yaw). Add a motor, a servo or change spin directions.`;
    res.head = null; return res;
  }
  let cnt = 1; for (let i = 0; i < k - 1; i++) cnt = cnt * (gens.length - i) / (i + 1);
  if (cnt > 40000) { res.verdict = 'warn'; res.title = 'Too many actuators'; res.why = 'Envelope check skipped: too many actuator combinations.'; res.head = null; return res; }
  const H = []; const idx = [];
  (function rec(s, d) {
    if (d === k - 1) {
      const rows = idx.map(i => gens[i].g); const nv = new Array(k);
      for (let i = 0; i < k; i++) { const minor = rows.map(r => r.filter((_, j) => j !== i)); nv[i] = (i % 2 ? -1 : 1) * det(minor); }
      const nn = Math.hypot(...nv); if (nn > 1e-9) { const n = nv.map(x => x / nn); H.push(n, n.map(x => -x)); } return;
    }
    for (let i = s; i < gens.length; i++) { idx[d] = i; rec(i + 1, d + 1); }
  })(0, 0);
  const planes = H.map(n => {
    let h = 0; for (const gg of gens) { const p = n.reduce((s, x, i) => s + x * gg.g[i], 0); h += Math.max(p * gg.lo, p * gg.hi); }
    return { n, m: h - n.reduce((s, x, i) => s + x * w[i], 0) };
  });
  let minM = Infinity; for (const p of planes) minM = Math.min(minM, p.m);
  res.inside = minM > -1e-9;
  res.head = labels.map((_, i) => [-1, 1].map(sg => { let t = Infinity; for (const p of planes) { const ni = p.n[i] * sg; if (ni > 1e-9) t = Math.min(t, p.m / ni); } return t; }));
  const weak = [];
  res.head.forEach((hh, i) => {
    const u = units[i], lo = Math.min(hh[0], hh[1]);
    if (u === 'g') { if (hh[1] < 0.15 * G) weak.push(labels[i] + ' (climb)'); }
    else if (u === 'lin') { if (lo < 0.5) weak.push(labels[i]); }
    else { const lim = labels[i] === 'Yaw' ? 0.5 : 5; if (lo < lim) weak.push(labels[i]); }
  });
  res.weak = weak;
  if (!res.inside) { res.verdict = 'bad'; res.title = 'Cannot hover'; res.why = 'Hover needs more force or torque than the actuators can make on at least one axis. Reduce mass, move it toward the center, or add thrust.'; }
  else if (weak.length) { res.verdict = 'warn'; res.title = 'Marginal'; res.why = 'Hover is possible, but there is little authority left on: ' + weak.join(', ') + '.'; }
  else { res.verdict = 'good'; res.title = 'Flyable'; res.why = 'Hover sits inside the attainable set with margin on every axis.'; }
  return res;
}

/* ───────── history ───────── */
const hist = { t: [], tilt: [], err: [], est: [], util: [] }; const HMAX = 500;
const trail = [];
function pushHist() {
  const R = qmat(S.q); const { hub } = hubState(R); const up = clamp(dot(m3v(R, nb), [0, 0, 1]), -1, 1);
  let util = 0; for (const c of actuators()) { const st = act.get(c.id); util = Math.max(util, st.Tcmd / c.tmax); }
  hist.t.push(S.t); hist.tilt.push(Math.acos(up) * R2D); hist.err.push(nrm(sub(hub, [setpoint.x, setpoint.y, setpoint.z])) * 100); hist.util.push(util * 100);
  const E = m3m(m3T(R), est.R); hist.est.push(Math.acos(clamp((E[0] + E[4] + E[8] - 1) / 2, -1, 1)) * R2D);
  if (hist.t.length > HMAX) { for (const k in hist) hist[k].shift(); }
  if (S.steps % 200 === 0) { trail.push(hub); if (trail.length > 600) trail.shift(); }
}
