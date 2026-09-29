'use strict';
// Firmware in the loop: the simulator flies the drone's own flight code (runner/fc/fc_core.c, built to
// WebAssembly as js/fc-wasm.js) instead of its JavaScript controller. Each control step the firmware gets the
// fused IMU reading and the battery voltage and returns throttles and servo angles, which go to the simulated
// ESCs and servos. Commands come in at 50 Hz, as the Raspberry Pi sends them over the link; the flight
// formulas it runs are the ones compiled here, loaded through the firmware's own host (rn_host.c).
// The firmware flies in angle mode, with no horizontal position hold: the pilot keys lean it, turn it, and
// climb or sink (with a barometer it holds its height when the keys are let go).
const FIL = {
  on: false, noBaro: false, boost: 0, w: null, err: '', why: '', state: 0, tCmd: 0, linkDown: false, thr: 0, n: 0,
  cmd: { arm: 0, roll: 0, pitch: 0, yaw: 0, throttle: 0, testMotor: -1, testThrottle: 0 },
  keys: true,           // sticks from the pilot keys (a test script sets them itself)
  info: null,           // the firmware's attitude, rates and status from the last step
};
const FIL_STATE = ['disarmed', 'armed', 'failsafe', 'crashed', 'motor test'];
const FIL_IO = 11 + 12 + 8 + 16;

function filCstr(ptr) { const b = new Uint8Array(FIL.w.memory.buffer, ptr, 64); let n = 0; while (n < 64 && b[n]) n++; return new TextDecoder().decode(b.slice(0, n)); }
async function filLoad() {
  if (FIL.w) return FIL.w;
  if (typeof WebAssembly !== 'object' || typeof FC_WASM_B64 !== 'string') throw new Error('this browser has no WebAssembly');
  const bytes = Uint8Array.from(atob(FC_WASM_B64), c => c.charCodeAt(0));
  const { instance } = await WebAssembly.instantiate(bytes, { env: RnWasm.env() });
  return (FIL.w = instance.exports);
}
// The flight program as the Pi would send it: the formulas compiled here, with self-tests from this session.
function filImage() {
  if (!RN.P) throw new Error('the flight formulas don\'t compile' + (RN.buildErr ? ': ' + RN.buildErr : ''));
  const samples = []; for (const k of Object.keys(RN.samples || {})) for (const s of RN.samples[k]) samples.push(s);
  return rnImage(RN.P, { tests: rnMakeTests(RN.P, samples, 600) });
}
// Start: reset, put the drone on the ground, motors stopped, and hand the firmware the program and the airframe.
async function filStart() {
  const w = await filLoad();
  const img = filImage(), blob = fcAirframeBlob({ imuBody: true });
  if (img.length > w.img_cap() || blob.length > w.blob_cap()) throw new Error('the program or airframe is too big for the firmware');
  doReset(); filGround();
  new Uint8Array(w.memory.buffer, w.img_ptr(), img.length).set(img);
  new Uint8Array(w.memory.buffer, w.blob_ptr(), blob.length).set(blob);
  const e = w.setup(img.length, blob.length);
  FIL.why = filCstr(w.why_ptr());
  if (e) throw new Error(FIL.why || 'setup failed (' + e + ')');
  Object.assign(FIL.cmd, { arm: 0, roll: 0, pitch: 0, yaw: 0, throttle: 0, testMotor: -1, testThrottle: 0 });
  FIL.on = true; FIL.tCmd = 0; FIL.linkDown = false; FIL.thr = 0; FIL.n = 0; FIL.err = '';
  filRender();
  return FIL.why;
}
function filStop() { if (!FIL.on) return; FIL.on = false; filRender(); }
function filGround() {   // resting on its lowest contact point, level, still
  const R = qmat(S.q); let low = 0;
  for (const c of contactPoints()) low = Math.min(low, m3v(R, c.rest)[2]);
  S.p = [S.p[0], S.p[1], -low + 0.002]; S.v = [0, 0, 0]; S.w = [0, 0, 0];
  for (const a of act.values()) { a.T = 0; a.Tcmd = 0; a.u = 0; a.v = 0; a.Omega = 0; a.i = 0; }
  for (const p of pend.values()) { p.v = [0, 0, 0]; }
  S.mb = { K: mbKinematics([0, 0, 0, 0, 0, 0]), acc: MB.bodies.map(() => [0, 0, 0, 0, 0, 0]) };
  setpoint.z = S.p[2];
}
function filSticks(dt) {   // the pilot keys, as the sticks of a radio in angle mode
  const k = c => isHeld(c) ? 1 : 0, C = FIL.cmd;
  C.pitch = k('fwd') - k('back'); C.roll = k('right') - k('left'); C.yaw = k('yawL') - k('yawR');
  if (FIL.boost > 0) FIL.boost -= dt;   // Take off: climb for a moment
  C.throttle = FIL.thr > 0 ? clamp((FIL.boost > 0 ? 0.85 : FIL.thr) + 0.35 * (k('up') - k('down')), 0.06, 1) : 0;
}
// One control step (called from controlStep in place of the JavaScript controller).
function filStep(dt) {
  const w = FIL.w, io = new Float32Array(w.memory.buffer, w.io_ptr(), FIL_IO);
  if (FIL.keys) filSticks(dt);
  if ((FIL.tCmd -= dt) <= 0 && !FIL.linkDown) {   // a command frame from the Pi, 50 times a second
    FIL.tCmd += 0.02;
    const C = FIL.cmd;
    new Float32Array(w.memory.buffer, w.cmd_ptr(), 7).set([C.arm, C.roll, C.pitch, C.yaw, C.throttle, C.testMotor, C.testThrottle]);
    w.command();
  }
  const imu = est.haveImu;
  const baro = FIL.noBaro ? null : sensorsOf('baro').map(c => sens.get(c.id)).find(rt => rt && rt.latest != null);
  io.set([...(imu ? est.fGyro : [0, 0, 0]), ...(imu ? est.fAccel : [0, 0, 0]), imu ? 1 : 0, dt, fc.vComp && hread.b.V > 1 ? hread.b.V : 0,
    baro ? baro.latest : 0, baro ? 1 : 0], 0);
  w.tick();
  actuators().forEach((c, i) => { const st = act.get(c.id); if (st) { const u = S.crashed ? 0 : io[11 + i]; setThrottle(c, st, u, u); } });
  const steer = new Set(steerJoints());
  joints().forEach((j, k) => { const st = jst.get(j.id); if (st && steer.has(j)) st.thCmd = io[23 + k]; });
  const s = w.state(); if (s !== FIL.state || ++FIL.n % 100 === 0) { FIL.state = s; FIL.why = filCstr(w.why_ptr()); }
  FIL.info = { q: Array.from(io.subarray(31, 35)), w: Array.from(io.subarray(35, 38)), trap: io[38], rho: io[39], attOk: io[40] > 0.5, yawSp: io[41], az: io[42], trim: io[43], vz: io[44], alt: io[45], haveAlt: io[46] > 0.5 };
}

/* ───────── panel (Airframe tab → Flight controller) ───────── */
function filRender() {
  const $ = s => document.querySelector(s); if (!$('#filToggle')) return;
  $('#filToggle').textContent = FIL.on ? 'Back to the simulator\'s controller' : 'Fly the firmware';
  $('#filToggle').classList.toggle('primary', !FIL.on);
  for (const id of ['#filArm', '#filUp']) $(id).disabled = !FIL.on;
  $('#filArm').textContent = FIL.on && FIL.cmd.arm ? 'Disarm' : 'Arm';
  $('#filUp').textContent = FIL.thr > 0 ? 'Throttle down' : 'Take off';
  const st = $('#filState'); st.textContent = FIL.on ? FIL_STATE[FIL.state] || '' : '';
  const n = $('#filNote'); n.textContent = FIL.err || (FIL.on ? FIL.why : ''); n.hidden = !n.textContent;
}
(function filPanel() {
  const $ = s => document.querySelector(s); if (!$('#filToggle')) return;
  $('#fcExport').addEventListener('click', fcExportAirframe);
  $('#filToggle').addEventListener('click', () => {
    if (FIL.on) { filStop(); doReset(); return; }
    FIL.keys = true;
    filStart().catch(e => { FIL.err = 'Can\'t fly the firmware: ' + e.message; filRender(); });
  });
  $('#filArm').addEventListener('click', () => { FIL.cmd.arm = FIL.cmd.arm ? 0 : 1; if (!FIL.cmd.arm) FIL.thr = 0; filRender(); });
  $('#filUp').addEventListener('click', () => { FIL.thr = FIL.thr > 0 ? 0 : 0.5; FIL.boost = FIL.thr > 0 ? 1.2 : 0; filRender(); });
  setInterval(() => { if (FIL.on) filRender(); }, 250);
})();
