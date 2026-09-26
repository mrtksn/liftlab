'use strict';
// Sensors. The physical side works out what each sensor truly experiences at its position and
// mount (rotation, lever-arm acceleration, motor vibration, magnetic interference) and samples it
// through the imperfection models in laws.js at the sensor's rate, delivering readings after its delay.
// The flight-software side fuses the delivered readings and runs the estimators; the controller
// flies on that estimate unless you switch it to ground truth.

const MAG_EARTH = [0.5, 0, -0.866];        // Earth's field, strength 1, pointing north (+X) and 60° down
const VIB_FREQ = 380;                       // motor rotation frequency at full thrust [Hz]
const VIB_ACC = 4, VIB_GYRO = 0.15;         // vibration at a motor at full thrust [m/s², rad/s]
const VIB_REACH = 0.12;                     // vibration falls off over this distance [m]
const MAG_INTERF = 0.3, MAG_REACH = 0.05;   // motor field at the motor at full thrust (Earth = 1), fall-off [m]

const SENSOR_KINDS = { imu: 'IMU', mag: 'Compass', baro: 'Barometer', fix: 'Position fix' };
const FIX_QUALITY = {
  gps: { label: 'GPS', rate: 5, latency: 150, noise: 0.2, wander: 0.6, velNoise: 0.1 },
  rtk: { label: 'RTK GPS', rate: 10, latency: 80, noise: 0.02, wander: 0.02, velNoise: 0.03 },
  mocap: { label: 'Motion capture', rate: 100, latency: 10, noise: 0.002, wander: 0, velNoise: 0.01 },
};
function fixDefaults(q) { const { label, ...rest } = FIX_QUALITY[q]; return rest; }
function mkSensor(kind, name, x, y, z, o = {}) {
  const d = {
    imu: { rate: 1000, latency: 0, gyroNoise: 0.1, gyroBias: 0.5, gyroDrift: 0.02, gyroRange: 2000, accNoise: 0.05, accBias: 0.05, accRange: 16, vib: 1 },
    mag: { rate: 100, latency: 5, noise: 0.01, hardIron: 0.02, interference: 1 },
    baro: { rate: 50, latency: 20, noise: 0.15, drift: 0.01 },
    fix: Object.assign({ quality: 'gps', dropout: false }, fixDefaults('gps')),
  }[kind];
  return base(Object.assign({ type: 'sensor', kind, name, pos: [x, y, z], mount: [0, 0, 0], known: true }, d, o));
}
const defaultSensors = () => [
  mkSensor('imu', 'IMU', 0, 0, 0.01),
  mkSensor('mag', 'Compass', -0.03, 0, 0.07),
  mkSensor('baro', 'Baro', 0.03, 0.02, 0.005),
  mkSensor('fix', 'GPS', 0, 0, 0.09),
];

/* ───────── runtime ───────── */
const sens = new Map();   // sensor id -> { st, acc, queue, latest, fresh }
const vib = new Map();    // motor id -> { ph, u, um }
const est = { fGyro: [0, 0, 0], fAccel: [0, 0, 9.81], q: [1, 0, 0, 0], R: [1, 0, 0, 0, 1, 0, 0, 0, 1], w: [0, 0, 0], p: [0, 0, 0], v: [0, 0, 0], att: {}, pos: {}, haveImu: false };
let sensing = 'sensors';  // what the controller flies on: 'sensors' or 'truth'
const sensorsOf = kind => cfg.comps.filter(c => c.type === 'sensor' && c.kind === kind);
const allSensors = () => cfg.comps.filter(c => c.type === 'sensor');

function syncSensors() {
  const ids = new Set(cfg.comps.map(c => c.id));
  for (const k of [...sens.keys()]) if (!ids.has(k)) sens.delete(k);
  for (const k of [...vib.keys()]) if (!ids.has(k)) vib.delete(k);
  for (const c of allSensors()) if (!sens.has(c.id)) sens.set(c.id, { st: {}, acc: 0, queue: [], latest: null, fresh: false });
  for (const c of actuators()) if (!vib.has(c.id)) {
    const a = c.id * 2.39996;                                  // each motor shakes in its own direction
    vib.set(c.id, { ph: rand() * 6.283, u: unit([Math.cos(a), Math.sin(a), 0.35]), um: unit([Math.sin(a), Math.cos(a), 0.5]) });
  }
}

// What the sensor truly experiences, before imperfections.
function vibrationAt(pos) {
  let a = [0, 0, 0], w = [0, 0, 0];
  for (const c of actuators()) {
    const v = vib.get(c.id), st = act.get(c.id); if (!v || !st) continue;
    const k = (st.T * c.health / 100 / c.tmax) * Math.exp(-nrm(sub(pos, c.pos)) / VIB_REACH) * Math.sin(v.ph);
    a = add(a, scl(v.u, VIB_ACC * k)); w = add(w, scl(v.u, VIB_GYRO * k));
  }
  return { a, w };
}
function motorFieldAt(pos) {
  let b = [0, 0, 0];
  for (const c of actuators()) {
    const v = vib.get(c.id), st = act.get(c.id); if (!v || !st) continue;
    const d = nrm(sub(pos, c.pos));
    b = add(b, scl(v.um, MAG_INTERF * (st.T / c.tmax) / (1 + (d / MAG_REACH) ** 2)));
  }
  return b;
}
function advanceVibration(dt) {
  for (const c of actuators()) { const v = vib.get(c.id), st = act.get(c.id); if (v && st) v.ph += 2 * Math.PI * VIB_FREQ * Math.sqrt(clamp(st.T / c.tmax, 0, 1)) * dt; }
}
function measure(c, rt, dt) {
  const R = qmat(S.q), RT = m3T(R), Rm = eulerR(c.mount[0], c.mount[1], c.mount[2]), RmT = m3T(Rm);
  const r = sub(c.pos, truth.c);
  if (c.kind === 'imu') {
    const fb = add(m3v(RT, add(S.acc, [0, 0, G])), add(crs(S.wdot, r), crs(S.w, crs(S.w, r))));
    const vb = vibrationAt(c.pos);
    const p = { gyroNoise: c.gyroNoise * D2R, gyroBias: c.gyroBias * D2R, gyroDrift: c.gyroDrift * D2R, gyroRange: c.gyroRange * D2R, accNoise: c.accNoise, accBias: c.accBias, accRange: c.accRange * G };
    return run('imuModel', m3v(RmT, S.w), m3v(RmT, fb), { a: m3v(RmT, scl(vb.a, c.vib)), w: m3v(RmT, scl(vb.w, c.vib)) }, p, rt.st, dt);
  }
  if (c.kind === 'mag') return run('magModel', m3v(RmT, m3v(RT, MAG_EARTH)), m3v(RmT, scl(motorFieldAt(c.pos), c.interference)), { noise: c.noise, hardIron: c.hardIron }, rt.st);
  if (c.kind === 'baro') return run('baroModel', add(S.p, m3v(R, r))[2], { noise: c.noise, drift: c.drift }, rt.st, dt);
  return run('posFixModel', add(S.p, m3v(R, r)), add(S.v, m3v(R, crs(S.w, r))), { noise: c.noise, wander: c.wander, velNoise: c.velNoise }, rt.st, dt);
}
function sampleSensors(dt) {
  advanceVibration(dt);
  for (const c of allSensors()) {
    const rt = sens.get(c.id); if (!rt) continue;
    const period = 1 / Math.max(1, c.rate);
    rt.acc += dt; if (rt.acc + 1e-9 < period) continue;
    rt.acc = rt.acc % period;
    if (c.kind === 'fix' && c.dropout) continue;              // no fix while dropped out
    rt.queue.push({ t: S.t + c.latency / 1000, ts: S.t, m: measure(c, rt, period) });
    if (rt.queue.length > 400) rt.queue.shift();
  }
}
function primeSensors() { // one immediate reading from every sensor, so the estimators can align at reset
  for (const c of allSensors()) {
    const rt = sens.get(c.id); if (!rt || (c.kind === 'fix' && c.dropout)) continue;
    rt.latest = measure(c, rt, 1 / Math.max(1, c.rate)); rt.ts = S.t; rt.fresh = true;
  }
}

/* ───────── flight-software side ───────── */
// Readings are rotated into the body frame with the mount the controller knows, and position
// readings are shifted to the frame hub with the sensor position it knows. Several sensors of the
// same kind are averaged.
const knownMount = c => c.known ? eulerR(c.mount[0], c.mount[1], c.mount[2]) : [1, 0, 0, 0, 1, 0, 0, 0, 1];
const knownPos = c => c.known ? c.pos : [0, 0, 0];
function mean3(list) { const s = list.reduce((a, b) => add(a, b), [0, 0, 0]); return scl(s, 1 / list.length); }

function senseAndEstimate(dt) {
  for (const rt of sens.values()) while (rt.queue.length && rt.queue[0].t <= S.t + 1e-9) { const q = rt.queue.shift(); rt.latest = q.m; rt.ts = q.ts; rt.fresh = true; }
  const ready = kind => sensorsOf(kind).filter(c => sens.get(c.id) && sens.get(c.id).latest);
  const imus = ready('imu');
  est.haveImu = imus.length > 0;
  if (est.haveImu) {
    const gyro = mean3(imus.map(c => m3v(knownMount(c), sens.get(c.id).latest.gyro)));
    const accel = mean3(imus.map(c => m3v(knownMount(c), sens.get(c.id).latest.accel)));
    const mags = ready('mag');
    const mag = mags.length ? mean3(mags.map(c => m3v(knownMount(c), sens.get(c.id).latest))) : null;
    est.fGyro = gyro; est.fAccel = accel;               // fused readings, also used to learn the airframe
    const a = run('attitudeEstimator', est.att, gyro, accel, mag, dt);
    est.q = qnorm(a.q); est.R = qmat(est.q); est.w = a.w;
    const baros = ready('baro');
    const age = list => list.reduce((s, c) => s + (S.t - sens.get(c.id).ts), 0) / list.length;
    const baro = baros.length ? { alt: baros.reduce((s, c) => s + sens.get(c.id).latest - m3v(est.R, knownPos(c))[2], 0) / baros.length, age: age(baros) } : null;
    const fixes = ready('fix').filter(c => !c.dropout);
    const fix = fixes.length ? {
      p: mean3(fixes.map(c => sub(sens.get(c.id).latest.p, m3v(est.R, knownPos(c))))),
      v: mean3(fixes.map(c => sub(sens.get(c.id).latest.v, m3v(est.R, crs(est.w, knownPos(c)))))),
      age: age(fixes),
    } : null;
    const pv = run('positionEstimator', est.pos, est.R, accel, baro, fix, model.m, dt);
    est.p = pv.p; est.v = pv.v;
  }
  for (const rt of sens.values()) rt.fresh = false;
}
function resetEstimation() {
  seedRng(12345);
  sens.clear(); vib.clear(); syncSensors();
  est.att = {}; est.pos = {};
  const R = qmat(S.q); const hub = sub(S.p, m3v(R, truth.c));
  est.pos.p = hub.slice(); est.pos.v = [0, 0, 0];   // the drone starts where it thinks it is
  est.q = S.q.slice(); est.R = R; est.w = [0, 0, 0]; est.p = hub.slice(); est.v = [0, 0, 0];
  S.acc = [0, 0, 0]; S.wdot = [0, 0, 0];
  primeSensors();
}
// Estimation errors against the ground truth, for the telemetry.
function estimateErrors() {
  const R = qmat(S.q); const { hub, vh } = hubState(R);
  const E = m3m(m3T(R), est.R); const ang = Math.acos(clamp((E[0] + E[4] + E[8] - 1) / 2, -1, 1)) * R2D;
  const upT = m3v(R, nb), upE = m3v(est.R, nb); const tilt = Math.acos(clamp(dot(upT, upE), -1, 1)) * R2D;
  const hT = Math.atan2(R[3], R[0]), hE = Math.atan2(est.R[3], est.R[0]); let dh = (hE - hT) * R2D; dh = ((dh + 180) % 360 + 360) % 360 - 180;
  const dp = sub(est.p, hub);
  let gb = null, gl = null;
  const imu = sensorsOf('imu')[0]; const rt = imu && sens.get(imu.id);
  if (rt && rt.st.bg) { gb = nrm(rt.st.bg) * R2D; if (LAWS.attitudeEstimator.status === 'default' && est.att.ie) gl = nrm(est.att.ie) * (0.08 / 0.6) * R2D; }  // kI / kP of the default filter
  return { ang, tilt, head: dh, pos: Math.hypot(dp[0], dp[1]) * 100, alt: dp[2] * 100, vel: nrm(sub(est.v, vh)) * 100, gb, gl };
}
