'use strict';
// Sensors. The physical side works out what each sensor truly experiences at its position and
// mount (rotation, lever-arm acceleration, motor vibration, magnetic interference) and samples it
// through the imperfection models in laws.js at the sensor's rate, delivering readings after its delay.
// The flight computers (boards.js) get the delivered readings and do the estimating.

const MAG_EARTH = [0.5, 0, -0.866];        // Earth's field, strength 1, pointing north (+X) and 60° down
const VIB_FREQ = 380;                       // motor rotation frequency at full thrust [Hz]
const VIB_ACC = 4, VIB_GYRO = 0.15;         // vibration at a motor at full thrust [m/s², rad/s]
const VIB_REACH = 0.12;                     // vibration falls off over this distance [m]
const MAG_INTERF = 0.3, MAG_REACH = 0.05;   // motor field at the motor at full thrust (Earth = 1), fall-off [m]

const SENSOR_KINDS = { imu: 'IMU', mag: 'Compass', baro: 'Barometer', fix: 'Position fix', flow: 'Optical flow' };
const FIX_QUALITY = {
  gps: { label: 'GPS', rate: 5, latency: 150, noise: 0.2, wander: 0.6, velNoise: 0.1 },
  rtk: { label: 'RTK GPS', rate: 10, latency: 80, noise: 0.02, wander: 0.02, velNoise: 0.03 },
  mocap: { label: 'Motion capture', rate: 100, latency: 10, noise: 0.002, wander: 0, velNoise: 0.01 },
};
// What each sensor board weighs [kg]: a breakout like the MPU6050, QMC5883, BMP180; a GPS with its patch antenna; a flow camera with its rangefinder.
const SENSOR_MASS = { imu: 0.003, mag: 0.002, baro: 0.002, fix: 0.015, flow: 0.005 };
function fixDefaults(q) { const { label, ...rest } = FIX_QUALITY[q]; return rest; }
function mkSensor(kind, name, x, y, z, o = {}) {
  const d = {
    imu: { rate: 1000, latency: 0, gyroNoise: 0.1, gyroBias: 0.5, gyroDrift: 0.02, gyroRange: 2000, accNoise: 0.05, accBias: 0.05, accRange: 16, scaleErr: 0.005, misalign: 0.2, vib: 1 },
    mag: { rate: 100, latency: 5, noise: 0.01, hardIron: 0.02, softIron: 0.03, interference: 1 },
    baro: { rate: 50, latency: 20, noise: 0.15, drift: 0.01 },
    fix: Object.assign({ quality: 'gps', dropout: false }, fixDefaults('gps')),
    flow: { rate: 100, latency: 20, noise: 0.05, scale: 0.03, maxRate: 7, minRange: 0.05, maxRange: 4, rangeNoise: 0.01 },   // camera looks along its −Z
  }[kind];
  return base(Object.assign({ type: 'sensor', kind, name, pos: [x, y, z], mount: [0, 0, 0], known: true, mass: SENSOR_MASS[kind] }, d, o));
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
const est = { fGyro: [0, 0, 0], fAccel: [0, 0, 9.81], q: [1, 0, 0, 0], R: [1, 0, 0, 0, 1, 0, 0, 0, 1], w: [0, 0, 0], p: [0, 0, 0], v: [0, 0, 0], havePos: false, haveImu: false, drv: {} };
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
    const s = hs.get(c.id), stub = s && s.prop ? 0.8 * clamp((st.Omega || 0) * propR(c) / 180, 0, 1.3) : 0;   // a broken prop's stub is badly out of balance
    const k = (st.T * motorEff(c) / c.tmax + stub) * Math.exp(-nrm(sub(pos, posNow(c))) / VIB_REACH) * Math.sin(v.ph);
    a = add(a, scl(v.u, VIB_ACC * k)); w = add(w, scl(v.u, VIB_GYRO * k));
  }
  return { a, w };
}
function motorFieldAt(pos) {
  let b = [0, 0, 0];
  for (const c of actuators()) {
    const v = vib.get(c.id), st = act.get(c.id); if (!v || !st) continue;
    const d = nrm(sub(pos, posNow(c)));
    b = add(b, scl(v.um, MAG_INTERF * (st.T / c.tmax) / (1 + (d / MAG_REACH) ** 2)));
  }
  return b;
}
function advanceVibration(dt) {
  for (const c of actuators()) { const v = vib.get(c.id), st = act.get(c.id); if (v && st) v.ph += 2 * Math.PI * VIB_FREQ * clamp((st.Omega || 0) * propR(c) / 180, 0, 1.3) * dt; }   // follows the prop speed
}
function measure(c, rt, dt) {
  // Each sensor rides on a body of the airframe (the frame, or a servo's output and what's on it) and feels
  // that body's motion at its own spot: rotation of the frame plus every joint above it, and the
  // acceleration there including the joints swinging it.
  if (!S.mb || !MB) S.mb = { K: mbKinematics(cat6(S.w, m3v(m3T(qmat(S.q)), S.v))), acc: MB.bodies.map(() => [0, 0, 0, 0, 0, 0]) };
  const K = S.mb.K, b = Math.min(MB.of.get(c.id) || 0, K.v.length - 1), Pr = sub(c.pos, MB.bodies[b].pivot);   // spot on its body, body axes
  const R = qmat(S.q), Rbw = m3m(R, K.Rb[b]);                                    // body b → world
  const Mt = eulerR(c.mount[0], c.mount[1], c.mount[2]), MtT = m3T(Mt);           // sensor → body b
  const vb = K.v[b], wb = top3(vb);
  const P = add(K.ob[b], m3v(K.Rb[b], Pr));                                       // where it is now, frame axes
  const ps = add(S.p, m3v(R, P)), vs = m3v(Rbw, add(bot3(vb), crs(wb, Pr)));      // world position and velocity
  if (c.kind === 'imu') {
    const ab = S.mb.acc[b] || [0, 0, 0, 0, 0, 0];
    const acl = add(add(bot3(ab), crs(top3(ab), Pr)), crs(wb, add(bot3(vb), crs(wb, Pr))));   // ordinary acceleration of that spot
    const fb = add(acl, m3v(m3T(Rbw), [0, 0, G]));                                // minus gravity: what an accelerometer reads
    const vi = vibrationAt(P);
    const p = { gyroNoise: c.gyroNoise * D2R, gyroBias: c.gyroBias * D2R, gyroDrift: c.gyroDrift * D2R, gyroRange: c.gyroRange * D2R, accNoise: c.accNoise, accBias: c.accBias, accRange: c.accRange * G, scaleErr: c.scaleErr ?? 0.005, misalign: (c.misalign ?? 0.2) * D2R };
    const RbT = m3T(K.Rb[b]);   // vibration is described in frame axes
    return run('imuModel', m3v(MtT, wb), m3v(MtT, fb), { a: m3v(MtT, m3v(RbT, scl(vi.a, c.vib))), w: m3v(MtT, m3v(RbT, scl(vi.w, c.vib))) }, p, rt.st, dt);
  }
  const Rs = m3m(Rbw, Mt), RsT = m3T(Rs);                                         // sensor → world
  if (c.kind === 'mag') return run('magModel', m3v(RsT, MAG_EARTH), m3v(MtT, m3v(m3T(K.Rb[b]), scl(motorFieldAt(P), c.interference))), { noise: c.noise, hardIron: c.hardIron, softIron: c.softIron ?? 0.03 }, rt.st);
  if (c.kind === 'flow') {
    const down = m3v(Rs, [0, 0, -1]);
    const d = terrainRay(ps, down, 50, 0.2);                          // distance to the ground (or a roof, or a wall) along the boresight
    const v = m3v(RsT, vs), w = m3v(MtT, wb);
    const f = isFinite(d) && d > 0.01 ? [w[1] - v[0] / d, -w[0] - v[1] / d] : [w[1], -w[0]];
    const q = isFinite(d) ? envr.texture * envr.light * clamp(1.25 - d / 8, 0, 1) : 0;   // image quality: texture, light, height
    const fl = run('flowModel', f, q, { noise: c.noise, scale: c.scale, maxRate: c.maxRate }, rt.st, dt);
    return { flow: [fl[0], fl[1]], q: fl[2], range: run('rangeModel', d, { noise: c.rangeNoise, minRange: c.minRange, maxRange: c.maxRange }, rt.st) };
  }
  if (c.kind === 'baro') return run('baroModel', ps[2], { noise: c.noise, drift: c.drift }, rt.st, dt);
  return run('posFixModel', ps, vs, { noise: c.noise, wander: c.wander, velNoise: c.velNoise }, rt.st, dt);
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

/* ───────── what the boards receive ───────── */
// The sensor drivers' part: readings that have arrived (after each sensor's delay), turned into body axes with the
// mount the flight software knows, several of a kind averaged. The estimating is the boards' (boards.js): the flight
// core's attitude, the navigation's position. est holds what they believe, for the panels and the view.
// Where the flight software thinks a sensor is and how it's turned: its described mount, carried by the
// joints above it at the angles the software believes. An unknown sensor is assumed at the hub, unrotated.
const knownMount = c => c.known ? m3m(poseOf(c, angleSeen).R, eulerR(c.mount[0], c.mount[1], c.mount[2])) : [1, 0, 0, 0, 1, 0, 0, 0, 1];
const knownPos = c => c.known ? poseOf(c, angleSeen).p : [0, 0, 0];
function mean3(list) { const s = list.reduce((a, b) => add(a, b), [0, 0, 0]); return scl(s, 1 / list.length); }

function senseAndEstimate(dt) {
  for (const rt of sens.values()) while (rt.queue.length && rt.queue[0].t <= S.t + 1e-9) { const q = rt.queue.shift(); rt.latest = q.m; rt.ts = q.ts; rt.fresh = true; }
  const ready = kind => sensorsOf(kind).filter(c => sens.get(c.id) && sens.get(c.id).latest);
  const imus = ready('imu');
  est.haveImu = imus.length > 0;
  const drv = est.drv || (est.drv = {});
  if (est.haveImu) {
    est.fGyro = mean3(imus.map(c => sub(m3v(knownMount(c), sens.get(c.id).latest.gyro), c.known ? chainRateSeen(c) : [0, 0, 0])));   // minus its joints' own turning
    est.fAccel = mean3(imus.map(c => m3v(knownMount(c), sens.get(c.id).latest.accel)));
  }
  const mags = ready('mag');
  drv.mag = mags.length ? mean3(mags.map(c => m3v(knownMount(c), sens.get(c.id).latest))) : null;
  const baros = ready('baro');
  if (baros.length) { const fr = baros.filter(c => sens.get(c.id).fresh); if (fr.length) { drv.baro = { alt: fr.reduce((s, c) => s + sens.get(c.id).latest, 0) / fr.length }; drv.baroTs = Math.max(...fr.map(c => sens.get(c.id).ts)); } }
  else drv.baro = null;
  const fix = ready('fix').filter(c => !c.dropout)[0];
  drv.fix = fix ? { c: fix, p: sens.get(fix.id).latest.p, v: sens.get(fix.id).latest.v, ts: sens.get(fix.id).ts } : null;
  if (drv.fix && S.t - drv.fix.ts > 1) drv.fix = null;   // a fix that stopped coming
  const flow = ready('flow')[0];
  est.flowState = sensorsOf('flow').length ? 'none' : null;
  if (flow) { const L = sens.get(flow.id).latest; drv.flow = L.range > 0 ? { c: flow, flow: L.flow, range: L.range, q: L.q, ts: sens.get(flow.id).ts } : null; est.flowState = !(L.range > 0) ? 'out of range' : L.q > 0 ? 'tracking' : 'range only'; }
  else drv.flow = null;
}
function sensorsDone() { for (const rt of sens.values()) rt.fresh = false; }
function resetEstimation() {
  seedRng(12345);
  sens.clear(); vib.clear(); syncSensors();
  const R = qmat(S.q); const hub = S.p.slice();
  est.q = S.q.slice(); est.R = R; est.w = [0, 0, 0]; est.p = hub.slice(); est.v = [0, 0, 0]; est.havePos = false; est.drv = {};
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
  let gb = null;
  const imu = sensorsOf('imu')[0]; const rt = imu && sens.get(imu.id);
  if (rt && rt.st.bg) gb = nrm(rt.st.bg) * R2D;
  return { ang, tilt, head: dh, pos: est.havePos ? Math.hypot(dp[0], dp[1]) * 100 : null, alt: est.havePos ? dp[2] * 100 : null, vel: est.havePos ? nrm(sub(est.v, vh)) * 100 : null, gb, gl: null };
}
