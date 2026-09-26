'use strict';
// Airframe configuration, simulation state, controller plumbing, physics stepping and the
// flight-envelope check. All physics and control formulas come from laws.js through run().

/* ───────── configuration ───────── */
let uid = 1;
const base = o => Object.assign({ id: uid++ }, o);
function mkMotor(name, x, y, z, o = {}) { return base(Object.assign({ type: 'motor', name, pos: [x, y, z], tilt: 0, az: 0, tmax: 6, kappa: 0.016, spin: 1, tau: 0.03, mass: 0.06, health: 100, healthKnown: true }, o)); }
function mkTilt(name, x, y, z, o = {}) { return base(Object.assign({ type: 'tilt', name, pos: [x, y, z], hingeAz: 0, mode: 'auto', manual: 0, range: 40, rate: 240, tmax: 6, kappa: 0.016, spin: 1, tau: 0.03, mass: 0.075, health: 100, healthKnown: true }, o)); }
function mkMass(name, x, y, z, o = {}) { return base(Object.assign({ type: 'mass', name, pos: [x, y, z], shape: 'box', mass: 0.2, size: [0.08, 0.05, 0.03], radius: 0.04, length: 0.1, known: true }, o)); }
function mkHang(name, x, y, z, o = {}) { return base(Object.assign({ type: 'hang', name, pos: [x, y, z], length: 0.5, mass: 0.15, known: true }, o)); }
const r3 = v => +v.toFixed(3);
const PRESETS = {
  quadx: { label: 'Quad X', build() {
    const r = 0.2; const c = [45, 135, 225, 315].map((a, i) => mkMotor('M' + (i + 1), r3(r * cosd(a)), r3(r * sind(a)), 0.02, { spin: i % 2 ? -1 : 1 }));
    c.push(mkMass('Battery', 0, 0, -0.035, { mass: 0.18, size: [0.1, 0.04, 0.03] })); return { frame: 0.45, comps: c, mode: 'tilt' }; } },
  hex: { label: 'Hexacopter', build() {
    const r = 0.25; const c = [0, 60, 120, 180, 240, 300].map((a, i) => mkMotor('M' + (i + 1), r3(r * cosd(a)), r3(r * sind(a)), 0.02, { spin: i % 2 ? -1 : 1, tmax: 5 }));
    c.push(mkMass('Battery', 0, 0, -0.04, { mass: 0.26, size: [0.13, 0.045, 0.035] })); return { frame: 0.55, comps: c, mode: 'tilt' }; } },
  tri: { label: 'Tricopter (yaw servo)', build() {
    const r = 0.22; const c = [mkMotor('Left', r3(r * cosd(60)), r3(r * sind(60)), 0.02, { spin: 1, tmax: 7 }),
      mkMotor('Right', r3(r * cosd(-60)), r3(r * sind(-60)), 0.02, { spin: -1, tmax: 7 }),
      mkTilt('Tail', -r, 0, 0.02, { hingeAz: 0, range: 30, rate: 300, spin: 1, tmax: 7 })];
    c.push(mkMass('Battery', 0.02, 0, -0.035, { mass: 0.18, size: [0.1, 0.04, 0.03] })); return { frame: 0.4, comps: c, mode: 'tilt' }; } },
  heli: { label: 'Main lifter + 4 steering motors', build() {
    const c = [mkMotor('Main', 0, 0, 0.06, { tmax: 22, kappa: 0.03, mass: 0.22, spin: 1, tau: 0.06 })]; const r = 0.26;
    [0, 90, 180, 270].forEach((a, i) => c.push(mkTilt('S' + (i + 1), r3(r * cosd(a)), r3(r * sind(a)), 0.02, { hingeAz: a, range: 45, rate: 300, tmax: 4, kappa: 0.012, spin: i % 2 ? 1 : -1 })));
    c.push(mkMass('Battery', 0, 0, -0.04, { mass: 0.3, size: [0.12, 0.05, 0.035] })); return { frame: 0.5, comps: c, mode: 'tilt' }; } },
  tiltquad: { label: 'Tilt-rotor quad (thrust vectoring)', build() {
    const r = 0.2; const c = [45, 135, 225, 315].map((a, i) => mkTilt('T' + (i + 1), r3(r * cosd(a)), r3(r * sind(a)), 0.02, { hingeAz: a, range: 30, rate: 360, spin: i % 2 ? -1 : 1, tmax: 6 }));
    c.push(mkMass('Battery', 0, 0, -0.035, { mass: 0.18, size: [0.1, 0.04, 0.03] })); return { frame: 0.45, comps: c, mode: 'level' }; } },
};
const cfg = { frame: { mass: 0.45 }, comps: [] };
let mode = 'tilt';
const setpoint = { x: 0, y: 0, z: 1.5, yaw: 0 };
const envr = { wind: 0, windDir: 0 };

/* ───────── state ───────── */
const S = { p: [0, 0, 1.5], v: [0, 0, 0], q: [1, 0, 0, 0], w: [0, 0, 0], crashed: null, t: 0, steps: 0 };
const act = new Map();   // id -> { T, Tcmd, th, thCmd }
const pend = new Map();  // id -> { p, v, Tn }
const ctl = { iPos: [0, 0, 0], iAtt: [0, 0, 0], wDes: [0, 0, 0, 0, 0, 0], sat: false, eAtt: 0 };
let truth = null, model = null, nb = [0, 0, 1];
let onCrash = () => {};

const actuators = () => cfg.comps.filter(c => c.type === 'motor' || c.type === 'tilt');
const hModel = c => c.healthKnown ? c.health / 100 : 1;
function hingeE(c) { const f = c.hingeAz * D2R; return [Math.sin(f), -Math.cos(f), 0]; }       // lean direction
function hingeAxis(c) { const f = c.hingeAz * D2R; return [Math.cos(f), Math.sin(f), 0]; }    // rotation axis
function actDir(c, th) {
  if (c.type === 'motor') { const t = c.tilt * D2R, a = c.az * D2R; return [Math.sin(t) * Math.cos(a), Math.sin(t) * Math.sin(a), Math.cos(t)]; }
  return unit(run('tiltAxis', th, hingeE(c)));
}
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
function massProps(which) {
  const items = [{ m: cfg.frame.mass, r: [0, 0, 0], I: boxI(cfg.frame.mass, 0.12, 0.12, 0.04) }];
  for (const c of cfg.comps) {
    if (c.type === 'motor' || c.type === 'tilt') items.push({ m: c.mass, r: c.pos, I: null });
    else if (c.type === 'mass') { if (which === 'truth' || c.known) items.push({ m: c.mass, r: c.pos, I: shapeI(c) }); }
    else if (c.type === 'hang') { if (which === 'model' && c.known) items.push({ m: c.mass, r: c.pos, I: null }); }
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
  for (const c of actuators()) { const th = c.type === 'tilt' && c.mode === 'manual' ? c.manual * D2R : 0; s = add(s, scl(actDir(c, th), c.tmax * hModel(c))); }
  return nrm(s) > 1e-9 ? unit(s) : [0, 0, 1];
}
function syncRuntime() {
  const ids = new Set(cfg.comps.map(c => c.id));
  for (const k of [...act.keys()]) if (!ids.has(k)) act.delete(k);
  for (const k of [...pend.keys()]) if (!ids.has(k)) pend.delete(k);
  const R = qmat(S.q);
  for (const c of cfg.comps) {
    if ((c.type === 'motor' || c.type === 'tilt') && !act.has(c.id)) act.set(c.id, { T: 0, Tcmd: 0, th: c.type === 'tilt' && c.mode === 'manual' ? c.manual * D2R : 0, thCmd: 0 });
    if (c.type === 'hang' && !pend.has(c.id)) { const a = add(S.p, m3v(R, sub(c.pos, truth.c))); pend.set(c.id, { p: [a[0], a[1], a[2] - c.length], v: S.v.slice(), Tn: 0 }); }
  }
}
function reseatPend(c) {
  const st = pend.get(c.id); if (!st) return;
  const R = qmat(S.q); const a = add(S.p, m3v(R, sub(c.pos, truth.c)));
  let d = sub(st.p, a); if (nrm(d) < 1e-6) d = [0, 0, -1];
  st.p = add(a, scl(unit(d), c.length)); st.v = S.v.slice();
}
function recomputeProps() {
  const R = qmat(S.q); const hub = truth ? sub(S.p, m3v(R, truth.c)) : [setpoint.x, setpoint.y, setpoint.z];
  truth = massProps('truth'); model = massProps('model');
  S.p = add(hub, m3v(R, truth.c)); nb = nominalAxis(); syncRuntime();
}

/* ───────── control ───────── */
const toAccel = col => { const f = scl([col[0], col[1], col[2]], 1 / model.m); const a = m3v(model.Jinv, [col[3], col[4], col[5]]); return [f[0], f[1], f[2], a[0], a[1], a[2]]; };

function allocate(w) {
  const wa = toAccel(w); const acts = actuators();
  // Stage 1: servo angles. Each auto servo rotor becomes two virtual inputs (T cos θ, T sin θ).
  if (acts.some(c => c.type === 'tilt' && c.mode === 'auto')) {
    const cols = [], lo = [], hi = [], who = [];
    for (const c of acts) {
      const hm = hModel(c), st = act.get(c.id);
      if (c.type === 'tilt' && c.mode === 'auto') {
        const sb = Math.sin(c.range * D2R) * c.tmax;
        cols.push(toAccel(scl6(wrenchCol(c.pos, [0, 0, 1], c.spin, c.kappa, model.c), hm))); lo.push(0); hi.push(c.tmax); who.push({ c, k: 'a' });
        cols.push(toAccel(scl6(wrenchCol(c.pos, hingeE(c), c.spin, c.kappa, model.c), hm))); lo.push(-sb); hi.push(sb); who.push({ c, k: 'b' });
      } else { cols.push(toAccel(scl6(wrenchCol(c.pos, actDir(c, st.th), c.spin, c.kappa, model.c), hm))); lo.push(0); hi.push(c.tmax); who.push({ c, k: 'u' }); }
    }
    const x = run('allocation', cols, lo, hi, wa, mode);
    for (let i = 0; i < who.length; i++) {
      if (who[i].k !== 'a') continue;
      const c = who[i].c, a = x[i], b = x[i + 1];
      if (Math.hypot(a, b) > 0.02 * c.tmax) act.get(c.id).thCmd = clamp(Math.atan2(b, a), -c.range * D2R, c.range * D2R);
    }
  }
  // Stage 2: motor thrusts at the servos' measured angles.
  const cols = acts.map(c => toAccel(scl6(wrenchCol(c.pos, actDir(c, act.get(c.id).th), c.spin, c.kappa, model.c), hModel(c))));
  const u = run('allocation', cols, acts.map(() => 0), acts.map(c => c.tmax), wa, mode);
  ctl.sat = false;
  acts.forEach((c, i) => { act.get(c.id).Tcmd = u[i]; if (u[i] >= c.tmax * 0.995) ctl.sat = true; });
}

function hubState(R) { const hub = sub(S.p, m3v(R, truth.c)); const vh = sub(S.v, m3v(R, crs(S.w, truth.c))); return { hub, vh }; }
function control(dt) {
  if (S.crashed) { for (const a of act.values()) a.Tcmd = 0; return; }
  const R = qmat(S.q), RT = m3T(R); const { hub, vh } = hubState(R);
  const ep = sub([setpoint.x, setpoint.y, setpoint.z], hub);
  for (let i = 0; i < 3; i++) ctl.iPos[i] = clamp(ctl.iPos[i] + ep[i] * dt, -2, 2);
  const Fd = run('positionControl', ep, vh, ctl.iPos, model.m, G);
  const nd = unit(run('thrustAxisTarget', Fd, mode));
  const psi = setpoint.yaw * D2R;
  const Rd = m3m(frameFrom(nd, [Math.cos(psi), Math.sin(psi), 0]), m3T(frameFrom(nb, [1, 0, 0])));
  const eR = run('attitudeError', R, Rd);
  ctl.eAtt = nrm(eR);
  for (let i = 0; i < 3; i++) ctl.iAtt[i] = clamp(ctl.iAtt[i] + eR[i] * dt, -0.5, 0.5);
  const tau = run('attitudeControl', eR, S.w, ctl.iAtt, model.J);
  const f = run('forceDemand', m3v(RT, Fd), nb, mode);
  ctl.wDes = [f[0], f[1], f[2], tau[0], tau[1], tau[2]];
  allocate(ctl.wDes);
}

/* ───────── physics ───────── */
const PDT = 0.0005;
function contactPoints() {
  const pts = [[0, 0, -0.03]];
  for (const c of cfg.comps) {
    if (c.type === 'motor' || c.type === 'tilt') pts.push([c.pos[0], c.pos[1], c.pos[2] - 0.03]);
    else if (c.type === 'mass') { const hz = c.shape === 'box' ? c.size[2] / 2 : c.shape === 'sphere' ? c.radius : c.length / 2; pts.push([c.pos[0], c.pos[1], c.pos[2] - hz]); }
  }
  return pts;
}
let cPts = [[0, 0, -0.03]];
function crash(why) { if (S.crashed) return; S.crashed = why; for (const a of act.values()) a.Tcmd = 0; onCrash(); }
function windVec() { return [envr.wind * cosd(envr.windDir), envr.wind * sind(envr.windDir), 0]; }

function dynamics(dt) {
  const R = qmat(S.q), RT = m3T(R);
  let F = run('gravity', truth.m, G), tau = [0, 0, 0];
  for (const c of actuators()) {
    const st = act.get(c.id);
    st.T = run('motorResponse', st.T, st.Tcmd, c.tmax, c.tau, dt);
    if (c.type === 'tilt') st.th = run('servoResponse', st.th, c.mode === 'manual' ? c.manual * D2R : st.thCmd, c.range * D2R, c.rate * D2R, dt);
    const rw = run('rotorWrench', actDir(c, st.th), sub(c.pos, truth.c), st.T * c.health / 100, c.spin, c.kappa);
    tau = add(tau, rw.tau); F = add(F, m3v(R, rw.F));
  }
  const wv = windVec();
  const dr = run('bodyDrag', S.v, wv, S.w); F = add(F, dr.F); tau = add(tau, dr.tau);
  for (const c of cfg.comps) {
    if (c.type !== 'hang') continue; const st = pend.get(c.id); if (!st) continue;
    const r = sub(c.pos, truth.c); const aw = add(S.p, m3v(R, r)); const va = add(S.v, m3v(R, crs(S.w, r)));
    const dv = sub(st.p, aw), L = nrm(dv); let Fc = [0, 0, 0], Tn = 0;
    if (L > 1e-9) {
      const n = scl(dv, 1 / L);
      Tn = run('cableTension', L - c.length, dot(sub(st.v, va), n), c.mass);
      Fc = scl(n, Tn);
    }
    st.Tn = Tn; F = add(F, Fc); tau = add(tau, crs(r, m3v(RT, Fc)));
    const Fp = add(add(scl(Fc, -1), run('gravity', c.mass, G)), run('payloadDrag', st.v, wv));
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
  S.v = add(S.v, scl(rb.a, dt)); S.p = add(S.p, scl(S.v, dt));
  S.w = add(S.w, scl(rb.wdot, dt));
  const dq = qmul(S.q, [0, S.w[0], S.w[1], S.w[2]]); S.q = qnorm(S.q.map((x, i) => x + 0.5 * dq[i] * dt));
  if (!S.crashed) {
    const up = dot(m3v(R, nb), [0, 0, 1]);
    if (!isFinite(S.p[0] + S.p[1] + S.p[2] + S.q[0])) { crash('The state became invalid (NaN). Check your edited formulas.'); S.p = [setpoint.x, setpoint.y, 0.2]; S.v = [0, 0, 0]; S.w = [0, 0, 0]; S.q = [1, 0, 0, 0]; }
    else if (up < -0.17) crash('Flipped over. The actuators could not hold the attitude.');
    else if (nrm(S.w) > 35) crash('Spun out of control. Check yaw authority and spin directions.');
    else if (Math.abs(S.p[0]) > 40 || Math.abs(S.p[1]) > 40 || S.p[2] > 40) crash('Flew away from the target.');
  }
}
function physStep() { S.steps++; if (S.steps % 2 === 0) control(PDT * 2); dynamics(PDT); S.t += PDT; if (S.steps % 40 === 0) pushHist(); }

function resetSim() {
  // start with the nominal thrust axis pointing up at the target heading
  S.q = matToQuat(m3m(frameFrom([0, 0, 1], [cosd(setpoint.yaw), sind(setpoint.yaw), 0]), m3T(frameFrom(nb, [1, 0, 0]))));
  const R = qmat(S.q); S.p = add([setpoint.x, setpoint.y, setpoint.z], m3v(R, truth.c)); S.v = [0, 0, 0]; S.w = [0, 0, 0]; S.crashed = null; S.t = 0; S.steps = 0;
  ctl.iPos = [0, 0, 0]; ctl.iAtt = [0, 0, 0]; pend.clear(); act.clear(); syncRuntime();
  for (let k = 0; k < 4; k++) { control(0); for (const c of actuators()) { const st = act.get(c.id); st.T = st.Tcmd; if (c.type === 'tilt') st.th = c.mode === 'manual' ? c.manual * D2R : st.thCmd; } }
  hist.t.length = hist.tilt.length = hist.err.length = hist.util.length = 0; trail.length = 0;
}

/* ───────── flight envelope ───────── */
// Attainable accelerations form a zonotope: the sum of each input's segment [lo, hi]·column.
// Hover is feasible when the acceleration hover needs lies inside it. Headroom along each axis is
// the distance from that point to the zonotope boundary.
function envelopeCalc() {
  const k = mode === 'level' ? 6 : 4; const gens = [];
  for (const c of actuators()) {
    const h = c.health / 100; if (h <= 0) continue; const st = act.get(c.id);
    const toK = col => { const f = scl([col[0], col[1], col[2]], 1 / truth.m), al = m3v(truth.Jinv, [col[3], col[4], col[5]]); return k === 4 ? [dot(f, nb), al[0], al[1], al[2]] : [f[0], f[1], f[2], al[0], al[1], al[2]]; };
    if (c.type === 'tilt' && c.mode === 'auto') {
      const sb = Math.sin(c.range * D2R) * c.tmax * h;
      gens.push({ g: toK(wrenchCol(c.pos, [0, 0, 1], c.spin, c.kappa, truth.c)), lo: 0, hi: c.tmax * h });
      gens.push({ g: toK(wrenchCol(c.pos, hingeE(c), c.spin, c.kappa, truth.c)), lo: -sb, hi: sb });
    } else gens.push({ g: toK(wrenchCol(c.pos, actDir(c, st ? st.th : 0), c.spin, c.kappa, truth.c)), lo: 0, hi: c.tmax * h });
  }
  let mp = 0, treq = [0, 0, 0];
  for (const c of cfg.comps) if (c.type === 'hang') { mp += c.mass; treq = add(treq, crs(sub(c.pos, truth.c), scl(nb, c.mass * G))); }
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
const hist = { t: [], tilt: [], err: [], util: [] }; const HMAX = 500;
const trail = [];
function pushHist() {
  const R = qmat(S.q); const { hub } = hubState(R); const up = clamp(dot(m3v(R, nb), [0, 0, 1]), -1, 1);
  let util = 0; for (const c of actuators()) { const st = act.get(c.id); util = Math.max(util, st.Tcmd / c.tmax); }
  hist.t.push(S.t); hist.tilt.push(Math.acos(up) * R2D); hist.err.push(nrm(sub(hub, [setpoint.x, setpoint.y, setpoint.z])) * 100); hist.util.push(util * 100);
  if (hist.t.length > HMAX) { for (const k in hist) hist[k].shift(); }
  if (S.steps % 200 === 0) { trail.push(hub); if (trail.length > 600) trail.shift(); }
}
