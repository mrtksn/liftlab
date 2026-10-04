'use strict';
// Airframe configuration, simulation state, controller plumbing, physics stepping and the
// flight-envelope check. All physics and control formulas come from laws.js through run().

/* ───────── configuration ───────── */
let uid = 1;
const base = o => Object.assign({ id: uid++ }, o);
function mkMotor(name, x, y, z, o = {}) { return withProp(base(Object.assign({ type: 'motor', name, pos: [x, y, z], tilt: 0, az: 0, tmax: 6, kappa: 0.016, spin: 1, push: false, tsens: false, telem: true, tmaxC: 120, cool: 1, failHeat: true, failMode: 'stop', failLoss: 50, tau: 0.03, pitch: 'fixed', fm: 0.6, mass: 0.06, health: 100, healthKnown: true }, o))); }
function withProp(c) { if (!c.prop) c.prop = +clamp(0.035 * Math.sqrt(c.tmax), 0.05, 0.2).toFixed(3); return c; }
function mkMass(name, x, y, z, o = {}) { return base(Object.assign({ type: 'mass', name, pos: [x, y, z], shape: 'box', mass: 0.2, size: [0.08, 0.05, 0.03], radius: 0.04, length: 0.1, known: true, aero: 'prism', inc: 0 }, o)); }
function mkHang(name, x, y, z, o = {}) { return base(Object.assign({ type: 'hang', name, pos: [x, y, z], length: 0.5, mass: 0.15, known: true }, o)); }
const r3 = v => +v.toFixed(3);
const PRESETS = {
  blank: { label: 'Blank (frame, battery, IMU)', blank: true, build() {   // a bare frame to build on: nothing to lift it yet
    return { frame: 0.12, comps: [mkMass('Battery', 0, 0, -0.03, { battery: true, mass: 0.1, size: [0.07, 0.035, 0.02] }), mkSensor('imu', 'IMU', 0, 0, 0.01)], mode: 'tilt' }; } },
  quadx: { label: 'Quad X', build() {
    const r = 0.2; const c = [45, 135, 225, 315].map((a, i) => mkMotor('M' + (i + 1), r3(r * cosd(a)), r3(r * sind(a)), 0.02, { spin: i % 2 ? -1 : 1 }));
    c.push(mkMass('Battery', 0, 0, -0.035, { battery: true, mass: 0.18, size: [0.1, 0.04, 0.03] })); return { frame: 0.45, comps: c.concat(defaultSensors()), mode: 'tilt' }; } },
  hex: { label: 'Hexacopter', build() {
    const r = 0.25; const c = [0, 60, 120, 180, 240, 300].map((a, i) => mkMotor('M' + (i + 1), r3(r * cosd(a)), r3(r * sind(a)), 0.02, { spin: i % 2 ? -1 : 1, tmax: 5 }));
    c.push(mkMass('Battery', 0, 0, -0.04, { battery: true, mass: 0.26, size: [0.13, 0.045, 0.035] })); return { frame: 0.55, comps: c.concat(defaultSensors()), mode: 'tilt' }; } },
  tri: { label: 'Tricopter (yaw servo)', build() {
    const r = 0.22; const c = [mkMotor('Left', r3(r * cosd(60)), r3(r * sind(60)), 0.02, { spin: 1, tmax: 7 }),
      mkMotor('Right', r3(r * cosd(-60)), r3(r * sind(-60)), 0.02, { spin: -1, tmax: 7 }),
      ...mkServoMotor('Tail', -r, 0, 0.02, { hingeAz: 0, range: 30, rate: 300 }, { spin: 1, tmax: 7 })];
    c.push(mkMass('Battery', 0.02, 0, -0.035, { battery: true, mass: 0.18, size: [0.1, 0.04, 0.03] })); return { frame: 0.4, comps: c.concat(defaultSensors()), mode: 'tilt' }; } },
  heli: { label: 'Main lifter + 4 steering motors', build() {
    const c = [mkMotor('Main', 0, 0, 0.06, { tmax: 22, kappa: 0.03, mass: 0.22, spin: 1, tau: 0.06, prop: 0.2 })]; const r = 0.26;
    [0, 90, 180, 270].forEach((a, i) => c.push(...mkServoMotor('S' + (i + 1), r3(r * cosd(a)), r3(r * sind(a)), 0.02, { hingeAz: a, range: 45, rate: 300 }, { tmax: 4, kappa: 0.012, spin: i % 2 ? 1 : -1 })));
    c.push(mkMass('Battery', 0, 0, -0.04, { battery: true, mass: 0.3, size: [0.12, 0.05, 0.035] }));
    const sn = defaultSensors(); sn[1].pos = [-0.12, -0.12, 0.08];   // compass on a boom, away from the big main motor
    return { frame: 0.5, comps: c.concat(sn), mode: 'tilt' }; } },
  helicopter: { label: 'Helicopter (main rotor + tail rotor)', build() {
    // The main rotor sits on a two-servo head that tilts it fore–aft and sideways (in place of a swashplate's
    // cyclic): tilting the lift off-centre is what turns the body. Its drag twists the body the other way,
    // which the tail rotor on the boom pushes against.
    const pitch = mkJoint('Head pitch servo', 0, 0, 0.14, { hingeAz: 90, range: 15, rate: 300, torque: 2 });
    const roll = mkJoint('Head roll servo', 0, 0, 0.14, { hingeAz: 0, range: 15, rate: 300, torque: 2, parent: pitch.id });
    const main = mkMotor('Main rotor', 0, 0, 0.14, { tmax: 24, prop: 0.3, kappa: 0.035, tau: 0.1, pitch: 'collective', mass: 0.2, spin: 1, parent: roll.id });
    const boom = mkLink('Tail boom', -0.03, 0, 0.04, { az: 180, el: 0, length: 0.45, mass: 0.05 });
    const tail = mkMotor('Tail rotor', -0.48, 0, 0.04, { tilt: 90, az: -90, tmax: 4, prop: 0.06, kappa: 0.012, tau: 0.02, mass: 0.04, spin: 1, parent: boom.id });
    const c = [pitch, roll, main, boom, tail, mkMass('Battery', 0.105, 0, -0.04, { battery: true, mass: 0.3, size: [0.12, 0.05, 0.035] }),   // battery forward, to balance the tail
      mkMass('Landing skids', 0.02, 0, -0.12, { mass: 0.06, size: [0.32, 0.2, 0.012] })];   // it stands on these, the tail rotor clear of the ground
    const sn = defaultSensors(); sn[1].pos = [-0.25, 0, 0.07];   // compass back along the boom, away from the main motor
    return { frame: 0.4, comps: c.concat(sn), mode: 'tilt' }; } },
  indoor: { label: 'Indoor quad (optical flow, no GPS)', build() {
    const r = 0.2; const c = [45, 135, 225, 315].map((a, i) => mkMotor('M' + (i + 1), r3(r * cosd(a)), r3(r * sind(a)), 0.02, { spin: i % 2 ? -1 : 1 }));
    c.push(mkMass('Battery', 0, 0, -0.035, { battery: true, mass: 0.18, size: [0.1, 0.04, 0.03] }));
    const sn = defaultSensors().filter(x => x.kind !== 'fix');
    sn.push(mkSensor('flow', 'Flow', 0, -0.03, -0.03));
    return { frame: 0.45, comps: c.concat(sn), mode: 'tilt' }; } },
  cargo: { label: 'Cargo quad (a hook, a bag on a line)', cargoTask: true, build() {   // a quad with a hook under the hub, carrying a bag on a line, that can drop it and pick things up
    const r = 0.2; const c = [45, 135, 225, 315].map((a, i) => mkMotor('M' + (i + 1), r3(r * cosd(a)), r3(r * sind(a)), 0.02, { spin: i % 2 ? -1 : 1 }));
    // The navigation holds 0.15 m above where it took off at the lowest: that leaves the hook about 7 cm over an
    // 8 cm parcel's top, within its 20 cm reach (with room to spare for hovering a little high or to one side). (A parcel hung straight on the hook would have it take off standing
    // on the parcel, too high to come back down to it once it's dropped: the bag hangs on a line instead.)
    // The hook sits beside the battery, not below it: it rests on the battery's corners, not on the hook.
    const hook = mkLatch('Hook', 0, 0.032, -0.035, { reach: 0.2 });
    c.push(mkMass('Battery', 0, 0, -0.035, { battery: true, mass: 0.18, size: [0.1, 0.04, 0.03] }), hook,
      mkHang('Bag', 0, 0.032, r3(-0.035 + LATCH_HOOK[2]), { length: 0.35, mass: 0.25, parent: hook.id }));
    const sn = defaultSensors(); Object.assign(sn[3], fixDefaults('rtk'), { quality: 'rtk', name: 'RTK GPS' });   // (to line the hook up on a parcel: plain GPS wanders half a metre)
    return { frame: 0.45, comps: c.concat(sn), mode: 'tilt' }; } },
  wingquad: { label: 'Quad with a wing', build() {   // a quad with a wing over it: it lifts in forward flight, which the flight computers aren't told about
    const r = 0.2; const c = [45, 135, 225, 315].map((a, i) => mkMotor('M' + (i + 1), r3(r * cosd(a)), r3(r * sind(a)), 0.02, { spin: i % 2 ? -1 : 1 }));
    c.push(mkMass('Battery', 0, 0, -0.035, { battery: true, mass: 0.18, size: [0.1, 0.04, 0.03] }),
      mkMass('Wing', 0, 0, 0.06, { shape: 'box', size: [0.1, 0.8, 0.01], mass: 0.07, aero: 'wing', inc: 15 }));   // (chord clear of the props; 15° of incidence: a quad leans 10–15° nose down to cruise, and the wing should still lift)
    return { frame: 0.45, comps: c.concat(defaultSensors()), mode: 'tilt' }; } },
  tiltquad: { label: 'Tilt-rotor quad (thrust vectoring)', build() {
    const r = 0.2; const c = [45, 135, 225, 315].flatMap((a, i) => mkServoMotor('T' + (i + 1), r3(r * cosd(a)), r3(r * sind(a)), 0.02, { hingeAz: a, range: 30, rate: 360 }, { spin: i % 2 ? -1 : 1, tmax: 6 }));
    c.push(mkMass('Battery', 0, 0, -0.035, { battery: true, mass: 0.18, size: [0.1, 0.04, 0.03] })); return { frame: 0.45, comps: c.concat(defaultSensors()), mode: 'level' }; } },
};
const cfg = { frame: { mass: 0.45 }, comps: [] };
let mode = 'tilt';
// Steering the controller uses right now: leaning, while its servos are held because it hasn't measured them yet.
const flyMode = () => learn.view && learn.view.holdServos ? 'tilt' : mode;
const setpoint = { x: 0, y: 0, z: 1.5, yaw: 0 };
const envr = { wind: 0, windDir: 0, turb: 0.3, spread: 1, texture: 0.8, light: 1, ambient: 25 };   // texture and light matter to optical flow
// No two motors and props are quite alike: each one's thrust, drag and spin-up differ a little from its card
// (a few percent, its own every time, fixed by its id). The controller and supervisor aren't told.
const spreadCache = new Map();
function spreadOf(c) {
  const k = envr.spread ?? 1, hit = spreadCache.get(c.id); if (hit && hit.k === k) return hit;
  const R = mulberry32(c.id * 7919 + 104729), n = () => clamp(Math.sqrt(-2 * Math.log(R() + 1e-12)) * Math.cos(2 * Math.PI * R()), -2.5, 2.5);
  const sp = { k, kT: 1 + 0.03 * k * n(), kQ: 1 + 0.05 * k * n(), J: 1 + 0.1 * k * n() }; spreadCache.set(c.id, sp); return sp;
}

/* ───────── state ───────── */
// p, v: the frame's centre (the hub) in the world; q, w: its attitude and body rates. Joints carry their own angles (jst).
const S = { p: [0, 0, 1.5], v: [0, 0, 0], q: [1, 0, 0, 0], w: [0, 0, 0], acc: [0, 0, 0], wdot: [0, 0, 0], batt: {}, battV: 16.8, battK: 1, rotors: [], mb: null, crashed: null, t: 0, steps: 0 };
const act = new Map();   // id -> { u: throttle, v: believed thrust fraction, Omega: prop speed, i: current, T: still-air thrust, Tcmd: steady thrust for u }
const pend = new Map();  // id -> { p, v, Tn }
// Mixed steering: the servos take up to `share` of the sideways force; `rho` tracks how much of what they
// were asked for they actually made (low-passed), so the body leans more when they can't keep up.
const steerMix = { share: 0.5, rho: 1 };
const mixShare = () => steerMix.share * steerMix.rho;
const ctl = { iPos: [0, 0, 0], iAtt: [0, 0, 0], wDes: [0, 0, 0, 0, 0, 0], sat: false, eAtt: 0, vRef: [0, 0, 0] };  // vRef: pilot's commanded velocity
let truth = null, model = null, nb = [0, 0, 1];
let onCrash = () => {};

const actuators = () => cfg.comps.filter(c => c.type === 'motor');   // thrust inputs; servo joints are in joints.js
// Thrust share the flight software believes a motor gives: its known health, and what the supervisor has told
// it (0 once a motor is taken out, the effectiveness it measured otherwise).
const hModel = c => c.healthKnown ? c.health / 100 : 1;   // what the description tells the flight code (the supervisor's corrections are on its board)
// A motor is mounted along its shaft (tilt, az: the way the shaft points, toward the prop). A puller's thrust
// points along the shaft, toward the prop (a tractor); a pusher's prop is pitched the other way, so its thrust
// points back along the shaft, toward the motor, and it blows air away past the prop. Spin is the prop's
// turning seen looking down the shaft at the prop, so a pusher's spin about its thrust axis is the reverse.
function mountDir(c) { const t = c.tilt * D2R, a = c.az * D2R; return [Math.sin(t) * Math.cos(a), Math.sin(t) * Math.sin(a), Math.cos(t)]; }
function actDir(c) { const m = mountDir(c); return c.push ? [-m[0], -m[1], -m[2]] : m; }   // thrust axis at rest
const spinOf = c => c.push ? -c.spin : c.spin;   // spin about the thrust axis, the way rotorWrench takes it
function rotorNow(c, ang = angleTrue) { const P = poseOf(c, ang); return { p: P.p, d: m3v(P.R, actDir(c)) }; }   // where a rotor is and points
function wrenchCol(pos, d, spin, kappa, cog) { const w = run('rotorWrench', d, sub(pos, cog), 1, spin, kappa); return [w.F[0], w.F[1], w.F[2], w.tau[0], w.tau[1], w.tau[2]]; }
const scl6 = (c, s) => c.map(x => x * s);

/* ───────── shapes in the air ───────── */
// Every solid part is a prism (drag on the face it shows the air: bluffDrag) or a wing (lift and drag: wingAero):
// the frame (cfg.frame.aero) and each rigid mass (c.aero). A wing is a box: chord along X, span along Y, thickness
// along Z, tipped by its incidence (inc, degrees, leading edge up). The flight computers aren't told: to them it's
// an oddly shaped body.
const FRAME_BOX = [0.12, 0.12, 0.04];
const frameWing = () => cfg.frame.aero === 'wing';
const frameDims = () => frameWing() ? [cfg.frame.chord || 0.25, cfg.frame.span || 0.8, cfg.frame.thick || 0.03] : FRAME_BOX;
function incR(deg) { const a = -(deg || 0) * D2R, c = Math.cos(a), s = Math.sin(a); return [c, 0, s, 0, 1, 0, -s, 0, c]; }   // leading edge (+X) up by deg
// The frame's shape, as a design keeps it.
const frameShapeOf = () => ({ aero: frameWing() ? 'wing' : 'prism', span: cfg.frame.span ?? 0.8, chord: cfg.frame.chord ?? 0.25, thick: cfg.frame.thick ?? 0.03, inc: cfg.frame.inc ?? 0 });
function setFrameShape(o) {
  o = o || {}; const n = (v, d, lo, hi) => isFinite(+v) ? clamp(+v, lo, hi) : d;
  Object.assign(cfg.frame, { aero: o.aero === 'wing' ? 'wing' : 'prism', span: n(o.span, 0.8, 0.1, 4), chord: n(o.chord, 0.25, 0.05, 1.5), thick: n(o.thick, 0.03, 0.005, 0.2), inc: n(o.inc, 0, -20, 20) });
}
const isWing = c => c.type === 'mass' && c.aero === 'wing' && c.shape === 'box';
const massRot = c => c.type === 'mass' && c.inc ? incR(c.inc) : [1, 0, 0, 0, 1, 0, 0, 0, 1];
const frameRot = () => frameWing() && cfg.frame.inc ? incR(cfg.frame.inc) : [1, 0, 0, 0, 1, 0, 0, 0, 1];
// The frontal area a prism shows along its X, Y and Z [m²].
function frontalAreas(c) {
  if (c.shape === 'sphere') { const a = Math.PI * c.radius ** 2; return [a, a, a]; }
  if (c.shape === 'cylinder') { const s = 2 * c.radius * c.length; return [s, s, Math.PI * c.radius ** 2]; }
  return [c.size[1] * c.size[2], c.size[0] * c.size[2], c.size[0] * c.size[1]];
}
// A box's eight corners (rest, body axes), turned by its incidence.
const boxCorners = (pos, size, R) => [-1, 1].flatMap(x => [-1, 1].flatMap(y => [-1, 1].map(z => add(pos, m3v(R, [x * size[0] / 2, y * size[1] / 2, z * size[2] / 2])))));

/* ───────── mass properties ───────── */
function boxI(m, a, b, c) { return [m * (b * b + c * c) / 12, 0, 0, 0, m * (a * a + c * c) / 12, 0, 0, 0, m * (a * a + b * b) / 12]; }
function shapeI(c) {
  const m = c.mass;
  if (c.shape === 'sphere') { const I = 0.4 * m * c.radius * c.radius; return [I, 0, 0, 0, I, 0, 0, 0, I]; }
  if (c.shape === 'cylinder') { const r = c.radius, L = c.length, ix = m * (3 * r * r + L * L) / 12; return [ix, 0, 0, 0, ix, 0, 0, 0, m * r * r / 2]; }
  const I = boxI(m, c.size[0], c.size[1], c.size[2]), R = massRot(c);
  return c.inc ? m3m(m3m(R, I), m3T(R)) : I;
}
const frameI = () => boxI(cfg.frame.mass, ...frameDims());
// Mass, CoG and inertia with every part where its joints put it: the true angles for the physics, the
// angles the flight software believes for its model.
// A thin rod's inertia about its middle: m L²/12 across it, nothing along it.
function rodI(l) { const d = linkDir(l), k = l.mass * l.length * l.length / 12; return [0, 1, 2].flatMap(i => [0, 1, 2].map(j => k * ((i === j ? 1 : 0) - d[i] * d[j]))); }
// The truth is what's on the drone now (a load dropped or picked up, cargo.js); the model is the design.
function massProps(which) {
  const ang = which === 'truth' ? angleTrue : angleSeen;
  const items = [{ m: cfg.frame.mass, r: [0, 0, 0], I: frameI() }];
  for (const c of which === 'truth' ? liveComps() : cfg.comps) {
    const pose = () => poseOf(c, ang);
    if (c.type === 'motor' || c.type === 'joint' || c.type === 'sensor' || c.type === 'latch') items.push({ m: c.mass, r: pose().p, I: null });
    else if (c.type === 'mass') { if (which === 'truth' || c.known) { const P = pose(); items.push({ m: c.mass, r: P.p, I: m3m(m3m(P.R, shapeI(c)), m3T(P.R)) }); } }
    else if (c.type === 'hang') { if (which === 'model' && c.known) items.push({ m: c.mass, r: pose().p, I: null }); }
    else if (c.type === 'link') { if (which === 'truth' || c.known) { const P = poseOf(c, ang); items.push({ m: c.mass, r: posePoint(c, add(c.pos, scl(linkDir(c), c.length / 2)), ang).p, I: m3m(m3m(P.R, rodI(c)), m3T(P.R)) }); } }
  }
  let m = 0, cm = [0, 0, 0]; for (const it of items) { m += it.m; cm = add(cm, scl(it.r, it.m)); } cm = scl(cm, 1 / m);
  opc(30 * items.length);
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
  // At rest (manual joints at their set angle), from the rotors that lift: a sideways rotor, like a
  // helicopter's tail rotor, steers but doesn't set which way is up.
  for (const c of actuators()) { const d = rotorNow(c, restAngle).d; s = add(s, scl(d, c.tmax * hModel(c) * Math.max(0, d[2]))); }
  return nrm(s) > 1e-9 ? unit(s) : [0, 0, 1];
}
function syncRuntime() {
  const live = liveComps(), ids = new Set(cfg.comps.map(c => c.id).concat(live.map(c => c.id))), here = new Set(live.map(c => c.id));
  for (const k of [...act.keys()]) if (!ids.has(k)) act.delete(k);
  for (const k of [...jst.keys()]) if (!ids.has(k)) jst.delete(k);
  for (const k of [...pend.keys()]) if (!here.has(k)) pend.delete(k);   // (a cable payload that fell is a loose body now)
  const R = qmat(S.q);
  for (const c of cfg.comps.concat(cargo.extra)) {
    if (c.type === 'hang' && !here.has(c.id)) continue;
    if (c.type === 'motor' && !act.has(c.id)) act.set(c.id, { T: 0, Tcmd: 0, u: 0, v: 0, k: 1, Omega: 0, i: 0 });
    if (c.type === 'joint' && !jst.has(c.id)) { const t0 = restAngle(c), tr = t0 + (c.offset || 0) * D2R; jst.set(c.id, { th: tr, thR: tr, thCmd: t0, thHat: t0, pst: {}, rate: 0, acc: 0, dq: [] }); }
    if (c.type === 'hang' && !pend.has(c.id)) { const a = add(S.p, m3v(R, posNow(c))); pend.set(c.id, { p: [a[0], a[1], a[2] - c.length], v: S.v.slice(), Tn: 0 }); }
  }
  syncSensors();
}
function reseatPend(c) {
  const st = pend.get(c.id); if (!st) return;
  const R = qmat(S.q); const a = add(S.p, m3v(R, posNow(c)));
  let d = sub(st.p, a); if (nrm(d) < 1e-6) d = [0, 0, -1];
  st.p = add(a, scl(unit(d), c.length)); st.v = S.v.slice();
}
function recomputeProps() {
  truth = massProps('truth'); model = massProps('model');
  buildBodies(); nb = nominalAxis(); syncRuntime();
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
// The throttle sent, what the controller believes it gives (thrust fraction), and what the motor will really make.
function setThrottle(c, st, u, sent = u) { st.u = sent; st.want = u; st.v = believedThrust(u, curveHat(c)); st.Tcmd = c.tmax * (isCollective(c) ? clamp(sent, 0, 1) : steadyX(sent, S.battV) ** 2); }
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
function hubState() { return { hub: S.p, vh: S.v }; }
// One step of the flight software: the sensor readings go to the flight computers, which fly the drone (boards.js).
function control(dt) { controlStep(dt); }
function controlStep(dt) {
  senseAndEstimate(dt);
  updateServoBelief(dt);
  boardsControl(dt);
  sensorsDone();
}

/* ───────── physics ───────── */
const PDT = 0.0005;
// Where the airframe can touch the ground or a building: small spheres (rest point, radius r) on the body
// that carries them, posed each step. The hub's corners, each motor and servo, the arms out to them, every
// corner of a box mass, rods along their length, and the sensors.
function contactPoints() {
  const on = c => (MB && MB.of.get(c.id)) || 0;
  const pts = [{ rest: [0, 0, -0.03], b: 0, r: 0 }];
  for (const p of boxCorners([0, 0, 0], frameDims(), frameRot())) pts.push({ rest: p, b: 0, r: 0 });   // the hub's corners (a wing's tips)
  for (const c of liveComps()) {
    const b = on(c);
    if (c.type === 'motor' || c.type === 'joint') {
      pts.push({ rest: add(c.pos, [0, 0, -0.03]), b, r: 0 }, { rest: c.pos.slice(), b, r: c.type === 'motor' ? 0.018 : 0.015 });
      if (!parentOf(c)) for (const k of [1 / 3, 2 / 3]) pts.push({ rest: scl(c.pos, k), b: 0, r: 0.008 });   // the arm from the hub
    } else if (c.type === 'mass') {
      const hz = c.shape === 'box' ? c.size[2] / 2 : c.shape === 'sphere' ? c.radius : c.length / 2;
      pts.push({ rest: add(c.pos, [0, 0, -hz]), b, r: 0 });
      if (c.shape === 'box') for (const p of boxCorners(c.pos, c.size, massRot(c))) pts.push({ rest: p, b, r: 0 });
      else if (c.shape === 'sphere') pts.push({ rest: c.pos.slice(), b, r: c.radius });
      else pts.push({ rest: add(c.pos, [0, 0, c.length / 2 - c.radius]), b, r: c.radius }, { rest: add(c.pos, [0, 0, -c.length / 2 + c.radius]), b, r: c.radius });
    } else if (c.type === 'link') pts.push({ rest: linkTip(c), b, r: 0 }, { rest: c.pos.slice(), b, r: 0 }, { rest: add(c.pos, scl(linkDir(c), c.length / 2)), b, r: 0.008 });
    else if (c.type === 'sensor') pts.push({ rest: add(c.pos, [0, 0, -0.005]), b, r: 0.006 });
    else if (c.type === 'latch') pts.push({ rest: add(c.pos, [0, 0, -0.01]), b, r: 0.008 });
  }
  cReach = Math.max(0.1, ...pts.map(p => nrm(p.rest) + p.r), ...liveMotors().map(c => nrm(c.pos) + propR(c)));
  return pts;
}
let cPts = [{ rest: [0, 0, -0.03], b: 0, r: 0 }], cReach = 0.3;   // how far from the hub any part (or prop tip) reaches
const propR = c => c.prop || clamp(0.035 * Math.sqrt(c.tmax), 0.05, 0.2);   // prop radius [m]
const payloadR = c => 0.025 + 0.035 * Math.cbrt(c.mass);
function washParts() {   // parts the downwash can push: the hub plate and rigid masses (horizontal frontal area); a wing meets it in wingAero
  const parts = frameWing() ? [] : [{ rest: [0, 0, 0], b: 0, area: 0.12 * 0.12 }];
  for (const c of liveComps()) if (c.type === 'mass' && !isWing(c)) parts.push({ rest: c.pos, b: MB.of.get(c.id) || 0, area: c.shape === 'box' ? c.size[0] * c.size[1] : Math.PI * c.radius * c.radius });
  return parts;
}
function crash(why) { if (S.crashed) return; S.crashed = why; for (const a of act.values()) { a.Tcmd = 0; a.u = 0; } onCrash(); }
function windVec() { const g = S.gust || [0, 0, 0]; return [envr.wind * cosd(envr.windDir) + g[0], envr.wind * sind(envr.windDir) + g[1], g[2]]; }
// Turbulence: gusts on top of the steady wind, random and correlated over about 2 s (stronger in stronger wind,
// weaker vertically).
function stepGusts(dt) {
  const tg = 2, sg = (envr.turb ?? 0) * (0.5 + 0.15 * envr.wind), k = sg * Math.sqrt(2 * dt / tg);
  S.gust = (S.gust || [0, 0, 0]).map((g, i) => g - g * dt / tg + k * (i === 2 ? 0.4 : 1) * randn());
}

/* ───────── motors ───────── */
// Every motor is a brushless motor, ESC and prop sized from its card: max thrust at V_NOM with the tips at
// 180 m/s, 80% of the voltage spent on back-EMF at full thrust (the rest across the windings), the prop's
// torque/thrust ratio, and a prop-plus-rotor inertia that gives the card's spin-up time near hover. The ESC
// limits the current to twice the full-thrust current.
const V_NOM = 16;   // pack voltage the max thrust is rated at (a charged 4S pack under load)
function motorParams(c) {
  const Om = 180 / propR(c), kT = c.tmax / (Om * Om), kap = Math.max(1e-4, c.kappa), kQ = kap * kT, Qm = kap * c.tmax;
  const Ke = 0.8 * V_NOM / Om, R = 0.2 * V_NOM * Ke / Qm, Oh = Math.sqrt(0.4) * Om;
  return { Om, kT, kQ, Ke, R, J: Math.max(0.005, c.tau || 0.03) * (Ke * Ke / R + 2 * kQ * Oh), iMax: 2 * Qm / Ke };
}
// Steady prop speed for throttle u at pack voltage V, as a fraction of the speed at max thrust. With the
// sizing above, k_QΩ² + (K_e²/R)Ω = K_e·uV/R becomes x² + 4x = 5uV/V_NOM for every motor.
const steadyX = (u, V) => -2 + Math.sqrt(4 + 5 * Math.max(0, u) * V / V_NOM);
// How much the real throttle-to-thrust curve bends, in the controller's terms (thrust ∝ (1−k)u + ku²).
const trueBend = c => c && isCollective(c) ? 0 : clamp((0.5 - (steadyX(0.5, S.battV) / steadyX(1, S.battV)) ** 2) / 0.25, 0, 1);

// A collective-pitch rotor (as on a helicopter): the ESC's governor holds the rotor at a set speed and the
// blade pitch sets the thrust, so thrust follows the command after the pitch servo's short lag, the rotor
// doesn't speed up or slow down (no spin-up twist), and it keeps its gyroscopic stiffness. More pitch means
// more drag torque, which the motor supplies and the frame feels the other way.
const isCollective = c => c.pitch === 'collective';
const GOV = 0.85;                                        // governed speed, as a fraction of the fixed-pitch full speed
function collectiveLoad(c, mp, col) {                    // thrust and drag coefficients at blade pitch col (0–1)
  const Og = GOV * mp.Om, kT = c.tmax / (Og * Og), kQf = Math.max(1e-4, c.kappa) * c.tmax / (Og * Og);
  return { Og, kT: kT * col, kQ: kQf * (0.3 + 0.7 * col ** 1.5) };
}
// One step of a motor: speed from the throttle (fixed pitch), or pitch from the command with the governor
// holding speed (collective). Returns motorDynamics' result; st.esc is the ESC duty (for the battery current).
function rotorStep(c, st, mp, V, dt) {
  if (!isCollective(c)) { st.esc = st.u || 0; return run('motorDynamics', st.Omega || 0, st.esc, V, mp, dt); }
  st.col = (st.col ?? 0) + ((st.u || 0) - (st.col ?? 0)) * Math.min(1, dt / 0.03);   // pitch servo
  const L = collectiveLoad(c, mp, clamp(st.col, 0, 1)), Om = st.Omega || 0, e = (L.Og - Om) / L.Og;
  st.gi = clamp((st.gi || 0) + 4 * e * dt, -0.3, 0.3);
  const ff = (mp.Ke * L.Og + mp.R * L.kQ * L.Og * L.Og / mp.Ke) / Math.max(1, V);   // duty that holds the speed under this load
  st.esc = clamp(ff + 3 * e + st.gi, 0, 1);
  return run('motorDynamics', Om, st.esc, V, { ...mp, kT: L.kT, kQ: L.kQ }, dt);
}

// The air each rotor meets (wind, its own motion, the other rotors' wash) and what that does to its thrust.
// Each disc also meets churned air of its own: small eddies (a tenth of a second) from turbulence and from its own
// wash curling back, much stronger close to the ground or a roof. That's what makes a real hover twitch.
function rotorAir(rotors, K, R, RT, wv, dt) {
  for (const ro of rotors) ro.va = sub(m3v(RT, wv), mbPointVel(K, ro.b, ro.p));   // oncoming air at each disc
  for (const ro of rotors) {
    const u = add(ro.va, run('wakeVelocity', ro.p, rotors.filter(o => o !== ro)));   // plus the other rotors' wash
    const pw = add(S.p, m3v(R, ro.p)), h = pw[2] - (terrain.boxes.length ? surfaceBelow(pw) : 0);   // height above the ground or the roof below
    if (dt) {
      const tr = 0.1, sr = (0.15 + (envr.turb ?? 0)) * 1.3 * (1 + 2.5 * Math.exp(-Math.max(0, h) / (6 * ro.R))) * Math.min(1, ro.T / 0.2);
      ro.st.gz = (ro.st.gz || 0) * (1 - dt / tr) + sr * Math.sqrt(2 * dt / tr) * randn();
    }
    const ua = dot(u, ro.d) + (ro.st.gz || 0);
    ro.ae = run('rotorAero', ro.T, ro.R, -ua, sub(u, scl(ro.d, ua)), h);
    ro.st.k = ro.T > 1e-6 ? ro.ae.T / ro.T : 1; ro.st.Teff = ro.ae.T;
  }
}

function dynamics(dt) {
  if (thr && thr.phase === 'hand') {   // held still in the hand
    S.v = [0, 0, 0]; S.w = [0, 0, 0]; S.acc = [0, 0, 0]; S.wdot = [0, 0, 0];
    for (const c of actuators()) { const a = act.get(c.id); a.Omega = isCollective(c) ? GOV * motorParams(c).Om : 0; a.T = 0; a.i = 0; a.col = 0; }   // a helicopter's rotor is spooled up before the throw
    S.mb = { K: mbKinematics([0, 0, 0, 0, 0, 0]), acc: MB.bodies.map(() => [0, 0, 0, 0, 0, 0]) };
    return;
  }
  if (thr && thr.phase === 'toss') {   // the throwing hand: a steady push up to speed and spin, joints held
    const T = thr.toss, a = scl(T.dv, 1 / T.dur), al = scl(T.dw, 1 / T.dur), R = qmat(S.q), RT = m3T(R);
    for (const c of actuators()) { const x = act.get(c.id); x.Omega = isCollective(c) ? GOV * motorParams(c).Om : 0; x.T = 0; x.i = 0; x.col = 0; }
    S.acc = a; S.wdot = al;
    S.v = add(S.v, scl(a, dt)); S.p = add(S.p, scl(S.v, dt)); S.w = add(S.w, scl(al, dt));
    const dq = qmul(S.q, [0, S.w[0], S.w[1], S.w[2]]); S.q = qnorm(S.q.map((x, i) => x + 0.5 * dq[i] * dt));
    T.t += dt;
    const vb = m3v(RT, S.v), K = mbKinematics(cat6(S.w, vb));
    S.mb = { K, acc: mbAccHeld(K, cat6(al, sub(m3v(RT, a), crs(S.w, vb)))) };
    return;
  }
  stepGusts(dt);
  const R = qmat(S.q), RT = m3T(R), wv = windVec(), acts = liveMotors(), N = MB.bodies.length;
  const K = mbKinematics(cat6(S.w, m3v(RT, S.v)));
  const posed = (b, rest) => add(K.ob[b], m3v(K.Rb[b], sub(rest, MB.bodies[b].pivot)));   // a rest point on body b, now (frame axes)
  const fext = MB.bodies.map(() => [0, 0, 0, 0, 0, 0]);
  // Every load also adds to the torque about the centre of mass (frame axes), for the view: gravity adds nothing
  // there, so it is what the rotors, the air, cables and the ground do to the drone's attitude.
  const cgF = truth.c; let tqNet = [0, 0, 0];
  const push = (b, F, P) => { fext[b] = add6v(fext[b], mbForce(K, b, F, P)); tqNet = add(tqNet, crs(sub(P, cgF), F)); };   // force F (frame axes) at P
  const pushT = (b, T) => { fext[b] = add6v(fext[b], mbTorque(K, b, T)); tqNet = add(tqNet, T); };
  const toWorld = P => add(S.p, m3v(R, P)), velW = (b, P) => m3v(R, mbPointVel(K, b, P));

  // Gravity on every body at its own centre of mass.
  MB.bodies.forEach((B, i) => { if (B.I.m > 0) push(i, m3v(RT, run('gravity', B.I.m, G)), posed(i, add(B.pivot, B.I.c))); });

  // Motors: throttle → current → torque → prop speed → thrust. The pack supplies the throttle-weighted current.
  let Ibatt = cargo.power ? 0.5 : 0;   // avionics (no battery on board: nothing at all, cargo.js)
  const lvc = escCutoffStep(dt, acts.some(c => (act.get(c.id) || {}).u > 0)) || !cargo.power;
  const rotors = acts.map(c => {
    const hsc = hsOf(c), st = act.get(c.id), mp0 = heatParams(c, motorParams(c)), dead = hsc.dead;   // the motor as it is at its temperature
    const sp = spreadOf(c), mpx = { ...mp0, kT: mp0.kT * sp.kT, kQ: mp0.kQ * sp.kQ, J: mp0.J * sp.J };   // this particular motor and prop
    const mp = hsc.prop ? { ...mpx, kT: 0, kQ: 0.03 * mpx.kQ, J: 0.4 * mpx.J } : mpx;                   // a broken prop: a stub, no thrust, almost no drag
    const off = (isCollective(c) && !(st.u > 0)) || lvc;   // a helicopter's ESC spools the rotor up only once it gets a throttle signal; the low-voltage cutoff stops them all
    const md = dead || off ? coastStep(st, mp, dt) : rotorStep(c, st, mp, S.battV, dt);
    if (dead || off) st.esc = 0;
    st.Omega = md.Omega; st.i = md.i; st.tauM = md.tau; st.T = md.T;
    Ibatt += st.esc * md.i;
    heatMotor(c, st, md, mp, dt);
    const b = MB.of.get(c.id) || 0;
    let d = m3v(K.Rb[b], actDir(c));
    if (isCollective(c)) {   // helicopter blades flap: the disc follows the mast a moment behind (world axes), instead of the whole airframe acting as a gyroscope
      const mast = m3v(R, d), tf = 16 / (4 * Math.max(50, md.Omega));   // flapping time constant 16/(γΩ), Lock number γ ≈ 4
      st.disc = st.disc ? unit(add(st.disc, scl(sub(mast, st.disc), Math.min(1, dt / tf)))) : mast;
      d = m3v(RT, st.disc);
    }
    return { c, st, b, p: posed(b, c.pos), d, T: md.T * motorEff(c), R: propR(c), Om: md.Omega, J: mp.J, tauM: md.tau };
  });
  // The pack and the motors pull on each other within a step (more current, more sag, less current), so the voltage
  // the ESCs see settles through their input capacitors (about 2 ms) rather than jumping each step, which would
  // ring once a nearly empty pack's resistance is high.
  if (cargo.power) {
    const Vpack = run('batteryModel', S.batt, Math.max(0, Ibatt), dt, battParams());
    S.battV += (Vpack - S.battV) * Math.min(1, dt / 0.002);
    S.battK = steadyX(1, S.battV) ** 2;
    S.battI = Ibatt;
    heatBattery(Math.max(0, Ibatt), dt);
  } else { S.battV = 0; S.battK = 0; S.battI = 0; }

  // Rotors in the air: inflow, wake interaction, ground effect, then the loads on whatever carries them.
  rotorAir(rotors, K, R, RT, wv, dt);
  for (const ro of rotors) {
    const Tw = Math.max(ro.ae.T, 1e-6);
    const rw = run('rotorWrench', ro.d, [0, 0, 0], Tw, spinOf(ro.c), ro.tauM / Tw);   // thrust, and the stator pushed back by the motor torque
    ro.tqReact = rw.tau;                                                               // (the reaction alone: r = 0 above)
    ro.st.tqR = ro.st.tqR ? add(ro.st.tqR, scl(sub(rw.tau, ro.st.tqR), Math.min(1, dt / 0.1))) : rw.tau.slice();   // smoothed, for the view
    push(ro.b, add(rw.F, ro.ae.H), ro.p);
    const h = isCollective(ro.c) ? [0, 0, 0] : scl(ro.d, spinOf(ro.c) * ro.J * ro.Om);   // the spinning prop's angular momentum (flapping blades don't pass it on)
    pushT(ro.b, sub(rw.tau, crs(mbOmega(K, ro.b), h)));                              // turning it takes a gyroscopic torque
  }
  S.rotors = rotors;
  for (const part of washParts()) { const P = posed(part.b, part.rest); push(part.b, run('wakeLoad', run('wakeVelocity', P, rotors), part.area), P); }
  const dr = run('bodyDrag', S.v, wv, S.w); push(0, m3v(RT, dr.F), [0, 0, 0]); pushT(0, dr.tau);
  // Shapes in the air: each wing's lift and drag (in the wind, its own motion and the rotors' wash), each blunt part's drag.
  S.aero = [];
  const aeroPart = (b, P, Rw, wing, dims, areas) => {
    const air = sub(m3v(RT, wv), mbPointVel(K, b, P)), u = m3v(m3T(Rw), wing ? add(air, run('wakeVelocity', P, rotors)) : air);
    let F, at = P;
    if (wing) { const w = run('wingAero', u, dims[0], dims[1]); F = m3v(Rw, w.F); at = add(P, m3v(Rw, [w.xcp, 0, 0])); }
    else F = m3v(Rw, run('bluffDrag', u, areas));
    push(b, F, at);
    if (wing) {   // for the view: drag is the part along the air past it, lift the rest (across it); its angle of attack and airspeed
      const a = m3v(Rw, u), V = nrm(a), dh = V > 1e-6 ? scl(a, 1 / V) : [0, 0, 0], D = scl(dh, dot(F, dh));
      S.aero.push({ b, P: at, F, L: sub(F, D), D, alpha: Math.atan2(u[2], -u[0]) * R2D, V: Math.hypot(u[0], u[2]) });
    }
  };
  if (frameWing()) aeroPart(0, [0, 0, 0], frameRot(), true, frameDims());
  for (const c of liveComps()) if (c.type === 'mass') { const b = MB.of.get(c.id) || 0; aeroPart(b, posed(b, c.pos), m3m(K.Rb[b], massRot(c)), isWing(c), c.size, isWing(c) ? null : frontalAreas(c)); }

  // Hanging payloads on cables.
  for (const c of liveComps()) {
    if (c.type !== 'hang') continue; const st = pend.get(c.id); if (!st) continue;
    const b = MB.of.get(c.id) || 0, P = posed(b, c.pos), aw = toWorld(P), va = velW(b, P);
    const dv = sub(st.p, aw), L = nrm(dv); let Fc = [0, 0, 0], Tn = 0;
    if (L > 1e-9) { const n = scl(dv, 1 / L); Tn = run('cableTension', L - c.length, dot(sub(st.v, va), n), c.mass); Fc = scl(n, Tn); }
    st.Tn = Tn; push(b, m3v(RT, Fc), P);
    const wash = m3v(R, run('wakeLoad', run('wakeVelocity', m3v(RT, sub(st.p, S.p)), rotors), Math.PI * payloadR(c) ** 2));
    const Fp = add(add(add(scl(Fc, -1), run('gravity', c.mass, G)), run('payloadDrag', st.v, wv)), wash);
    st.v = add(st.v, scl(Fp, dt / c.mass)); st.p = add(st.p, scl(st.v, dt));
    if (st.p[2] < 0.03) { st.p[2] = 0.03; if (st.v[2] < 0) st.v[2] = 0; st.v[0] *= 0.995; st.v[1] *= 0.995; }
    if (terrain.boxes.length) for (const h of terrainContacts(st.p, payloadR(c), terrainNear(st.p, payloadR(c) + 0.05), st.prev)) {   // a payload swung into a building
      st.p = add(st.p, scl(h.n, h.depth)); const vn = dot(st.v, h.n); if (vn < 0) st.v = sub(st.v, scl(h.n, vn)); st.v = scl(st.v, 0.995);
    }
    st.prev = st.p.slice();
  }
  // Ground and buildings: a contact spring at every point that's inside something, along the way out
  // (groundContact, turned to face that surface). Landing on something at more than 3 m/s is a crash;
  // bumping into a wall isn't, but the props may not survive it (below).
  const near = (terrain.boxes.length ? terrainNear(S.p, cReach + 0.2 + nrm(S.v) * 0.02) : []).concat(cargoSolids(S.p, cReach + 0.2 + nrm(S.v) * 0.02));   // (and loose things at rest, cargo.js)
  for (const pt of cPts) {
    if (pt.b >= N) continue;
    const P = posed(pt.b, pt.rest), pw = toWorld(P);
    if (pw[2] >= pt.r && !near.length) { pt.prev = pw; continue; }
    const hits = terrainContacts(pw, pt.r, near, pt.prev); pt.prev = pw; if (!hits.length) continue;
    const vel = velW(pt.b, P);
    for (const h of hits) {
      const n = h.n, t1 = Math.abs(n[2]) > 0.9 ? [1, 0, 0] : unit(crs([0, 0, 1], n)), t2 = crs(n, t1);   // the surface's own axes: on the ground, x, y, z
      const vl = [dot(vel, t1), dot(vel, t2), dot(vel, n)];
      if (n[2] > 0.7 && vl[2] < -3 && !S.crashed) crash(`Hit ${h.ground ? 'the ground' : 'the top of ' + h.what} at ${(-vl[2]).toFixed(1)} m/s.`);
      const f = run('groundContact', h.depth, vl);
      push(pt.b, m3v(RT, add(add(scl(t1, f[0]), scl(t2, f[1])), scl(n, f[2]))), P);
    }
  }
  propStrikes(rotors, toWorld, near);
  // Servos: each sees its command after its delay, drives toward it along its torque–speed line, and stops
  // hard a little past its travel.
  const tauJ = new Array(N).fill(0);
  for (let i = 1; i < N; i++) {
    const j = MB.bodies[i].j, st = jst.get(j.id); if (!st) continue;
    const Rg = j.range * D2R, off = (j.offset || 0) * D2R;
    if (!st.dq) st.dq = [];
    st.dq.push([S.t, clamp(jointTarget(j), -Rg, Rg)]);
    while (st.dq.length > 1 && st.dq[1][0] <= S.t - (j.lag || 0) + 1e-9) st.dq.shift();
    st.thR = st.dq[0][1] + off;
    let t = run('servoTorque', st.thR - st.th, st.rate || 0, { stall: j.torque ?? 0.8, speed: Math.max(1, j.rate) * D2R, band: 3 * D2R });
    t = servoFault(j, st, t);   // jammed or limp
    const x = st.th - off, over = Math.abs(x) - (Rg + 5 * D2R);
    if (over > 0) t += -Math.sign(x) * 50 * over - 0.5 * (st.rate || 0);
    tauJ[i] = t; st.tq = t;
  }
  // Torque about the centre of mass, smoothed over about 0.15 s for the view (step to step it jitters with the
  // controller's corrections).
  const TQS = 0.15;
  S.tqRaw = tqNet; S.tq = S.tq ? add(S.tq, scl(sub(tqNet, S.tq), Math.min(1, dt / TQS))) : tqNet.slice();
  {   // and what the flight core's attitude control asked for, in N·m
    const want = brt.out && brt.fcState === 1 ? brt.out.tau : null;
    S.tqWant = !want ? null : S.tqWant ? add(S.tqWant, scl(sub(want, S.tqWant), Math.min(1, dt / TQS))) : want.slice();
  }
  // Everything together: the frame and every joint, solved as one articulated body.
  const sol = mbSolve(K, fext, tauJ);
  const al = top3(sol.a0), acl = add(bot3(sol.a0), crs(S.w, m3v(RT, S.v)));   // spatial → ordinary acceleration of the hub
  S.acc = m3v(R, acl); S.wdot = al;
  S.v = add(S.v, scl(S.acc, dt)); S.p = add(S.p, scl(S.v, dt));
  S.w = add(S.w, scl(al, dt));
  const dq = qmul(S.q, [0, S.w[0], S.w[1], S.w[2]]); S.q = qnorm(S.q.map((x, i) => x + 0.5 * dq[i] * dt));
  for (let i = 1; i < N; i++) {
    const st = jst.get(MB.bodies[i].j.id); if (!st) continue;
    st.acc = sol.qdd[i]; st.rate = (st.rate || 0) + st.acc * dt; st.th += st.rate * dt;
  }
  if (N > 1) truth = massProps('truth');
  S.mb = { K: mbKinematics(cat6(S.w, m3v(m3T(qmat(S.q)), S.v))), acc: sol.acc };   // for the sensors
  if (!S.crashed) {
    const up = dot(m3v(R, nb), [0, 0, 1]);
    if (!isFinite(S.p[0] + S.p[1] + S.p[2] + S.q[0] + S.w[0])) { crash('The state became invalid (NaN). Check your edited formulas.'); S.p = [setpoint.x, setpoint.y, 0.2]; S.v = [0, 0, 0]; S.w = [0, 0, 0]; S.q = [1, 0, 0, 0]; S.mb = null; }
    else if (up < -0.17 && !thr && actuators().length) crash(actuators().some(c => hsOf(c).prop) ? 'Flipped over after losing a prop.' : 'Flipped over. The actuators could not hold the attitude.');
    else if (nrm(S.w) > 35) crash('Spun out of control. Check yaw authority and spin directions.');
    else if (Math.abs(S.p[0]) > 40 || Math.abs(S.p[1]) > 40 || S.p[2] > 40) crash('Flew away from the target.');
  }
}
function physStep() { S.steps++; if (S.steps % 2 === 0) control(PDT * 2); dynamics(PDT); cargoStep(PDT); S.t += PDT; sampleSensors(PDT); healthStep(PDT); if (S.steps % 40 === 0) pushHist(); }

// Every flight starts on the ground, motors stopped, under the target (or at the start point if a building is in
// the way); the flight computers then start as if just powered on, and the simulator arms and takes off for you.
let spawnAt = [0, 0, 0];
function resetSim() {
  thr = null;
  resetHealth(); nb = nominalAxis();   // parts repaired, the supervisor's settings cleared
  cargoReset();                        // every part back on board, the items to pick up where they're set
  buildBodies(); cPts = contactPoints();
  const blocked = (x, y) => { for (let z = 0.3; z <= Math.max(0.3, setpoint.z) + 0.01; z += 0.4) if (terrainNear([x, y, z], cReach + 0.4).length) return true; return false; };
  if (terrain.boxes.length && blocked(setpoint.x, setpoint.y)) {   // a building in the way: start again at the start point, in the open
    Object.assign(setpoint, { x: 0, y: 0, z: 1.5 }); pilot.vref = [0, 0, 0]; ctl.vRef = [0, 0, 0];
    if (typeof syncSp === 'function') syncSp();
    healthEvent('Started again at the start point: the target was against a building.', 'info');
  }
  // level, facing the target heading, resting on its lowest point
  S.q = matToQuat(m3m(frameFrom([0, 0, 1], [cosd(setpoint.yaw), sind(setpoint.yaw), 0]), m3T(frameFrom(nb, [1, 0, 0]))));
  const R0 = qmat(S.q); let low = 0;
  for (const pt of cPts) low = Math.min(low, m3v(R0, pt.rest)[2] - (pt.r || 0));
  const throwing = launchMode === 'throw' && hasTask('learn');   // a throw start: held in the hand at hand height
  S.p = [setpoint.x, setpoint.y, throwing ? throwCfg.handH : -low + 0.001]; spawnAt = [setpoint.x, setpoint.y, -low + 0.001];
  if (throwing) startThrow();
  S.v = [0, 0, 0]; S.w = [0, 0, 0]; S.gust = [0, 0, 0]; S.crashed = null; S.t = 0; S.steps = 0; S.tq = null; S.tqRaw = null; S.tqWant = null;
  ctl.iPos = [0, 0, 0]; ctl.iAtt = [0, 0, 0]; ctl.vRef = [0, 0, 0]; pend.clear(); act.clear(); jst.clear(); syncRuntime();
  S.batt = { soc: clamp(battCfg().startSoc ?? 1, 0.02, 1) }; S.battV = run('batteryModel', S.batt, 0.5, 0, battParams()); S.battK = steadyX(1, S.battV) ** 2;
  S.mb = { K: mbKinematics([0, 0, 0, 0, 0, 0]), acc: MB.bodies.map(() => [0, 0, 0, 0, 0, 0]) };
  resetEstimation(); resetLearning(); budgetReset();
  truth = massProps('truth'); model = massProps('model');
  hist.t.length = hist.tilt.length = hist.err.length = hist.est.length = hist.util.length = 0; trail.length = 0;
  boardsStart();
}

// A spinning prop that touches anything breaks. The disc's rim is checked at 16 points against the ground
// and nearby buildings; a stopped prop just rests against things.
function propStrikes(rotors, toWorld, near) {
  for (const ro of rotors) {
    const s = hsOf(ro.c); if (s.prop || ro.Om * ro.R < 12) continue;   // tips slower than 12 m/s: not spinning in earnest
    const d = ro.d, e1 = unit(crs(d, Math.abs(d[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0])), e2 = crs(d, e1);
    for (let k = 0; k < 16; k++) {
      const a = k * Math.PI / 8, P = add(ro.p, add(scl(e1, ro.R * Math.cos(a)), scl(e2, ro.R * Math.sin(a))));
      const what = solidAt(toWorld(P), near);
      if (what) { breakDevice(ro.c, 'prop', 'hit ' + what); break; }
    }
  }
}

/* ───────── flight envelope ───────── */
// Attainable accelerations form a zonotope: the sum of each input's segment [lo, hi]·column.
// Hover is feasible when the acceleration hover needs lies inside it. Headroom along each axis is
// the distance from that point to the zonotope boundary.
function envelopeCalc() {
  const k = mode === 'level' ? 6 : 4; const gens = [];   // mixed can always fall back on leaning, so it needs the 4 axes
  const sj = steerJoints();
  const sets = [];   // what each rotor can really make: its thrust (0 to full) along any direction its servos can swing it to
  for (const c of liveMotors()) {
    const h = motorEff(c); if (h <= 0) continue;   // what it really delivers: health, damage, or nothing if it has failed
    const toK = col => { const f = scl([col[0], col[1], col[2]], 1 / truth.m), al = m3v(truth.Jinv, [col[3], col[4], col[5]]); return k === 4 ? [dot(f, nb), al[0], al[1], al[2]] : [f[0], f[1], f[2], al[0], al[1], al[2]]; };
    const js = chainOf(c).filter(x => sj.includes(x));
    if (js.length) {   // a rotor its steering joints can swing: its thrust at the middle, plus each joint's swing (linearized over its range)
      gens.push({ g: toK((() => { const n = rotorNow(c, restAngle); return wrenchCol(n.p, n.d, spinOf(c), c.kappa, truth.c); })()), lo: 0, hi: c.tmax * h });
      let grid = [new Map()];   // every combination of its servos' angles, 7 steps across each range (49 at most)
      for (const j of js) { const R = j.range * D2R, st = js.length > 1 ? 4 : 7; grid = grid.flatMap(g => Array.from({ length: st }, (_, i) => new Map([...g, [j.id, -R + 2 * R * i / (st - 1)]]))); }
      sets.push(grid.map(g => { const n = rotorNow(c, x => g.has(x.id) ? g.get(x.id) : restAngle(x)); return toK(wrenchCol(n.p, n.d, spinOf(c), c.kappa, truth.c)).map(x => x * c.tmax * h); }));
      for (const j of js) {
        const at = th => { const n = rotorNow(c, x => x === j ? th : restAngle(x)); return wrenchCol(n.p, n.d, spinOf(c), c.kappa, truth.c); };
        const e = 1e-3, sb = Math.sin(j.range * D2R) * c.tmax * h;
        gens.push({ g: toK(at(e).map((v, i) => (v - at(-e)[i]) / (2 * e))), lo: -sb, hi: sb });
      }
    } else { const n = rotorNow(c); const g = toK(wrenchCol(n.p, n.d, spinOf(c), c.kappa, truth.c)); gens.push({ g, lo: 0, hi: c.tmax * h }); sets.push([g.map(x => x * c.tmax * h)]); }
  }
  let mp = 0, treq = [0, 0, 0];
  for (const c of liveComps()) if (c.type === 'hang') { mp += c.mass; treq = add(treq, crs(sub(posNow(c), truth.c), scl(nb, c.mass * G))); }
  const fz = (truth.m + mp) * G / truth.m, areq = m3v(truth.Jinv, treq), freq = scl(nb, fz);
  const w = k === 4 ? [fz, areq[0], areq[1], areq[2]] : [freq[0], freq[1], freq[2], areq[0], areq[1], areq[2]];
  const labels = k === 4 ? ['Climb', 'Roll', 'Pitch', 'Yaw'] : ['Fwd/back', 'Left/right', 'Climb', 'Roll', 'Pitch', 'Yaw'];
  const units = k === 4 ? ['g', 'ang', 'ang', 'ang'] : ['lin', 'lin', 'g', 'ang', 'ang', 'ang'];
  const res = { k, labels, units, w, rank: rankOf(gens.map(x => x.g), k), n: gens.length };
  if (res.rank < k) {
    res.verdict = 'bad'; res.title = actuators().length ? 'Not controllable' : 'Nothing lifts it';
    res.why = !actuators().length ? 'It has no motors yet, so it rests on the ground. Add motors (and servos, if you like) from Attach: a quad needs four, spinning in alternate directions.'
      : k === 6 ? `Stay-level mode needs thrust vectoring. The actuators control only ${res.rank} of 6 axes, so the craft cannot push sideways without tilting.`
      : `The actuators control only ${res.rank} of 4 axes (climb, roll, pitch, yaw). Add a motor, a servo or change spin directions.`;
    res.head = null; return res;
  }
  let cnt = 1; for (let i = 0; i < k - 1; i++) cnt = cnt * (gens.length - i) / (i + 1);
  if (cnt > 40000) { res.verdict = 'warn'; res.title = 'Too many actuators'; res.why = 'Envelope check skipped: too many actuator combinations.'; res.head = null; return res; }
  // The attainable set is the sum of every rotor's own set, so its faces lie across k−1 of the rotors' edges:
  // a thrust direction (0 to full), or the step between two directions a servo can swing it to. A swung
  // rotor's sideways push costs it lift (T cos θ up, T sin θ across), which a straight-line guess would miss.
  const H = [], normalsFrom = vecs => {
    const idx = [];
    (function rec(s, d) {
      if (d === k - 1) {
        const rows = idx.map(i => vecs[i]); const nv = new Array(k);
        for (let i = 0; i < k; i++) { const minor = rows.map(r => r.filter((_, j) => j !== i)); nv[i] = (i % 2 ? -1 : 1) * det(minor); }
        const nn = Math.hypot(...nv); if (nn > 1e-9) { const n = nv.map(x => x / nn); H.push(n, n.map(x => -x)); } return;
      }
      for (let i = s; i < vecs.length; i++) { idx[d] = i; rec(i + 1, d + 1); }
    })(0, 0);
  };
  normalsFrom(gens.map(g => g.g));
  const edges = [];
  for (const S of sets) { const sc = Math.max(1e-9, ...S.map(v => Math.hypot(...v))); for (let i = 0; i < S.length; i++) { edges.push(S[i]); for (let j = i + 1; j < S.length; j++) { const e = S[i].map((x, q) => x - S[j][q]); if (Math.hypot(...e) > 1e-3 * sc) edges.push(e); } } }
  let ce = 1; for (let i = 0; i < k - 1; i++) ce = ce * (edges.length - i) / (i + 1);
  if (ce <= 60000) normalsFrom(edges);
  const planes = H.map(n => {   // how far each face sits beyond hover: the sets' reach along n, less hover's
    let h = 0; for (const S of sets) { let b = 0; for (const v of S) b = Math.max(b, n.reduce((s, x, i) => s + x * v[i], 0)); h += b; }
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
