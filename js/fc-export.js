'use strict';
// The airframe as the flight controller firmware needs it (runner/fc/fc_core.h): the model the simulator's
// controller flies on, at the joints' rest angles. "Export for the flight controller" saves it as a file for
// the drone (the Pi sends it over the link and the ESP32 keeps it); the firmware-in-the-loop mode hands the same
// bytes to the firmware built to WebAssembly.
const FC_MAX_MOTORS = 12, FC_MAX_JOINTS = 8, FC_MAX_CHAIN = 2;

function fcAirframe(opts = {}) {
  const acts = actuators(), js = joints(), steer = new Set(steerJoints());
  const problems = [];
  if (!acts.length) problems.push('no motors');
  if (acts.length > FC_MAX_MOTORS) problems.push(`${acts.length} motors: the firmware takes at most ${FC_MAX_MOTORS}`);
  if (js.length > FC_MAX_JOINTS) problems.push(`${js.length} joints: the firmware takes at most ${FC_MAX_JOINTS}`);
  if (acts.length + js.length > RN_IN) problems.push(`${acts.length + js.length} inputs: the formulas hold ${RN_IN}`);
  const cm = ctlModel();
  // The IMU's mount (the first known IMU, its joints at rest), so the firmware turns its readings into body axes.
  const imu = sensorsOf('imu').find(c => c.known);
  const imuR = opts.imuBody || !imu ? [1, 0, 0, 0, 1, 0, 0, 0, 1] : m3m(poseOf(imu, restAngle).R, eulerR(imu.mount[0], imu.mount[1], imu.mount[2]));
  const motors = acts.map(c => {
    const ch = chainOf(c);
    if (ch.length > FC_MAX_CHAIN) problems.push(`${c.name} rides on ${ch.length} joints: the firmware takes ${FC_MAX_CHAIN}`);
    return { name: c.name, chain: ch.map(j => js.indexOf(j)), cols: colsFor(c).map(col => col.slice()), bend: curveHat(c), lag: motorLagHat(c), power: powerFull(c) };
  });
  const jnt = js.map(j => { const m = servoModelHat(j); return { name: j.name, steer: steer.has(j), manual: restAngle(j), range: j.range * D2R, rate: m.rate, lag: m.lag }; });
  return {
    motors, joints: jnt, mode: ['tilt', 'mixed', 'level'].indexOf(mode), m: cm.m, J: cm.J.slice(), Jinv: cm.Jinv.slice(), axis: ctlAxis().slice(), imuR,
    allowance: allocPrefs.allowance, efficiency: allocPrefs.efficiency, servoMove: allocPrefs.servoMove, horizon: allocPrefs.horizon,
    leanMax: 35, mixShare: steerMix.share, learned: flyingLearned(), problems,
  };
}

function fcAirframeBlob(opts) {
  const A = fcAirframe(opts);
  if (A.problems.length) throw new Error(A.problems.join('; '));
  const w = [], f32 = new Float32Array(1), u32 = new Uint32Array(f32.buffer);
  const u = x => w.push(x >>> 0), f = x => { f32[0] = x; w.push(u32[0]); }, fs = a => a.forEach(f);
  const name = s => { const b = new Uint8Array(16); const e = new TextEncoder().encode(String(s || '')).slice(0, 15); b.set(e); for (let i = 0; i < 16; i += 4) w.push(b[i] | b[i + 1] << 8 | b[i + 2] << 16 | (b[i + 3] << 24 >>> 0)); };
  u(0x41424644); u(1); u(A.motors.length); u(A.joints.length); u(A.mode);
  f(A.m); fs(A.J); fs(A.Jinv); fs(A.axis); fs(A.imuR);
  f(A.allowance); f(A.efficiency); f(A.servoMove); f(A.horizon); f(A.leanMax); f(A.mixShare);
  for (const j of A.joints) { name(j.name); u(j.steer ? 1 : 0); f(j.manual); f(j.range); f(j.rate); f(j.lag); }
  for (const m of A.motors) {
    name(m.name); u(m.chain.length); for (let c = 0; c < FC_MAX_CHAIN; c++) u(m.chain[c] ?? 0);
    u(m.cols.length); for (const col of m.cols) fs(col);
    f(m.bend); f(m.lag); f(m.power);
  }
  const body = new Uint8Array(new Uint32Array(w).buffer), out = new Uint8Array(body.length + 4);
  out.set(body); new DataView(out.buffer).setUint32(body.length, rnCrc32(body), true);
  return out;
}

// Saves the airframe file (Airframe tab → Export for the flight controller).
function fcExportAirframe() {
  let blob;
  try { blob = fcAirframeBlob(); } catch (e) { FIL.err = 'Can\'t export for the flight controller: ' + e.message; filRender(); return; }
  FIL.err = ''; filRender();
  const nm = ((typeof designs !== 'undefined' && designs.name) || 'airframe').replace(/[^\w.-]+/g, '-');
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([blob], { type: 'application/octet-stream' }));
  a.download = nm + '.dfa'; document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}
