'use strict';
// Flight computers: the boards on the drone, which tasks each one runs, the sensors wired to it, and the links
// between them. Only these fly the drone. Each board is the real flight code (runner/fc: fc_core.c, nav_core.c,
// the step runner, the flight program compiled from the formulas) built to WebAssembly, one instance per board,
// the same code the ESP32 and the Pi run. The simulator only supplies what the hardware would: sensor readings,
// what a board sends another over their link (after the link's delay), and the pilot.
//
// Tasks:
//   core  the flight core (fc_core.c), 1 kHz: attitude, control, mixing, arming and the failsafes. It needs exact
//         timing, so it runs on a microcontroller (an ESP32). Every drone has exactly one.
//   nav   the navigation (nav_core.c), 100 Hz: position from GPS, optical flow and the barometer, and holding or
//         moving it. It sends the flight core guided commands (an acceleration and a heading). On a Pi it talks to
//         the ESP32 over the serial link (runner/pi/pi_nav.c); it can also run on the ESP32 itself.
// Without navigation the pilot flies the flight core directly, in angle mode (the sticks lean the drone).

const BOARD_KINDS = {
  esp32: { label: 'ESP32', mcu: true, mops: 60, cores: 2, ramKB: 300, note: 'microcontroller, 2 cores at 240 MHz' },
  s3: { label: 'ESP32-S3', mcu: true, mops: 80, cores: 2, ramKB: 300, note: 'microcontroller, 2 cores at 240 MHz, faster memory' },
  c3: { label: 'ESP32-C3', mcu: true, mops: 4, cores: 1, ramKB: 280, note: 'microcontroller, 1 core at 160 MHz, no float unit' },
  pizero: { label: 'Raspberry Pi Zero', mcu: false, mops: 150, cores: 1, ramKB: 512 * 1024, note: 'Linux computer, 1 core at 1 GHz' },
  pizero2: { label: 'Raspberry Pi Zero 2 W', mcu: false, mops: 600, cores: 4, ramKB: 512 * 1024, note: 'Linux computer, 4 cores at 1 GHz' },
  pi4: { label: 'Raspberry Pi 4', mcu: false, mops: 2000, cores: 4, ramKB: 4096 * 1024, note: 'Linux computer, 4 cores at 1.8 GHz' },
};
const TASKS = {
  core: { label: 'Flight core', hz: 1000, mcuOnly: true,
    formulas: ['attitudeEstimator', 'servoPredictor', 'thrustAxisTarget', 'attitudeError', 'attitudeControl', 'forceDemand', 'allocationPreferences', 'allocation', 'thrustLinearization', 'voltageCompensation'],
    what: 'Attitude, control and mixing 1000 times a second, arming and the failsafes. It needs exact timing, so it runs on a microcontroller.' },
  nav: { label: 'Navigation', hz: 100,
    formulas: ['positionEstimator', 'flowVelocity', 'positionControl'],
    what: 'Where the drone is (GPS, optical flow, barometer) and holding or moving its position. It tells the flight core which way to accelerate and where to face. Without it you fly in angle mode: the keys lean the drone.' },
};
// Tasks that come in the next stage, shown so you can see where their formulas will go.
const TASKS_LATER = {
  learn: { label: 'Learning', formulas: ['identifyEffectiveness', 'identifyMotorResponse', 'identifyServoResponse', 'identifyThrow'],
    what: 'Learns what each motor and servo really does (calibration, the throw start). Runs on a Pi; coming in the next stage.' },
  super: { label: 'Health supervisor', formulas: ['actuatorHealth', 'faultDecision', 'flightPolicy'],
    what: 'Watches for failing or overheating parts and changes how the drone flies. Runs on a Pi; coming in the next stage.' },
};
const BOARD_MAX = 4, LINK_DELAY = 0.006;   // serial link: frame time plus scheduling, each way [s]

/* ───────── the configuration (part of the design) ───────── */
const defaultComputers = () => ({ boards: [
  { id: 1, kind: 'esp32', name: 'Flight controller', tasks: ['core'] },
  { id: 2, kind: 'pizero', name: 'Pi Zero', tasks: ['nav'] },
] });
function computers() {
  if (!cfg.computers || !Array.isArray(cfg.computers.boards) || !cfg.computers.boards.length) cfg.computers = defaultComputers();
  return cfg.computers;
}
// Keep it valid: known kinds, one flight core on a microcontroller, each task on one board.
function fixComputers(C) {
  C = C && Array.isArray(C.boards) ? JSON.parse(JSON.stringify(C)) : defaultComputers();
  C.boards = C.boards.filter(b => BOARD_KINDS[b.kind]).slice(0, BOARD_MAX);
  if (!C.boards.some(b => BOARD_KINDS[b.kind].mcu)) C.boards.unshift({ id: 0, kind: 'esp32', name: 'Flight controller', tasks: [] });
  let id = 1; for (const b of C.boards) { b.id = id++; b.name = String(b.name || BOARD_KINDS[b.kind].label).slice(0, 24); b.tasks = (b.tasks || []).filter(t => TASKS[t]); }
  for (const t of Object.keys(TASKS)) { let seen = false; for (const b of C.boards) if (b.tasks.includes(t)) { if (seen || (TASKS[t].mcuOnly && !BOARD_KINDS[b.kind].mcu)) b.tasks = b.tasks.filter(x => x !== t); else seen = true; } }
  if (!C.boards.some(b => b.tasks.includes('core'))) C.boards.find(b => BOARD_KINDS[b.kind].mcu).tasks.unshift('core');
  return C;
}
const boardOf = task => computers().boards.find(b => b.tasks.includes(task)) || null;
const hasTask = task => !!boardOf(task);
const boardName = b => b ? `${b.name}${b.name === BOARD_KINDS[b.kind].label ? '' : ' (' + BOARD_KINDS[b.kind].label + ')'}` : '';
// Which board a sensor is wired to. The IMU, compass and barometer (the GY-87 is all three) go to the flight core's
// board; the GPS and the flow camera to the board that navigates (with no navigation they'd be unused).
function wiredTo(c) {
  const core = boardOf('core'), nav = boardOf('nav');
  if (c.kind === 'imu' || c.kind === 'mag' || c.kind === 'baro') return core;
  return nav;
}
function setComputers(C, why) {
  cfg.computers = fixComputers(C); undoKey = 'computers:' + (why || '');
  brt.sig = null; doReset(); save();
  if (typeof renderComputers === 'function') renderComputers();
  syncFlightUi();
}

/* ───────── what each board costs ───────── */
// Operations per step, from the formulas' costs (budget.js) and the code around them; and memory.
function taskCost(task) {
  const n = actuators().length + joints().length, m = actuators().length;
  const C = k => { const f = FLIGHT_COST[k]; try { return f ? f({}, null, null, {}, {}, {}) : 0; } catch (e) { return 0; } };
  if (task === 'core') return C('attitudeEstimator') + 12 * joints().length + C('thrustAxisTarget') + C('attitudeError') + C('attitudeControl') + C('forceDemand')
    + 2 * (30 * n + 60 * n * n + 200) + m * (C('thrustLinearization') + 10) + 1500;   // allocation (two stages), throttles, the code around
  if (task === 'nav') return FLIGHT_COST.positionEstimator({}, null, null, 1, 1, { v: 1 }) + C('flowVelocity') + C('positionControl') + 400;
  return 0;
}
function boardBudget(b) {
  const K = BOARD_KINDS[b.kind];
  const ops = b.tasks.reduce((s, t) => s + taskCost(t) * TASKS[t].hz, 0);
  const load = ops / (K.mops * 1e6) * (K.mcu ? 1 : 1.5);   // a Linux board loses some to the system
  const memKB = RN.P ? (RN.P.arenaSize * 4 + RN.P.code.length * 4) / 1024 * (b.tasks.length ? 1 : 0) : 0;
  return { ops, load, memKB, ramKB: K.ramKB };
}

/* ───────── the runtime: one instance of the flight code per board ───────── */
const brt = {
  module: null, err: '', inst: new Map(), sig: null, ready: false,
  toCore: [], toNav: [], tel: null, navOut: null, navReady: false, home: null,
  t: 0, nextTel: 0, nextNav: 0.005, nextStick: 0,
  pilot: { arm: 0, fly: 0, thr: 0, phase: 'ground', t: 0 },
  fcState: 0, fcWhy: '', navWhy: '', out: null, baroTs: null,
};
const FC_STATES = ['disarmed', 'armed', 'failsafe', 'crashed', 'motor test'];
function boardsLoad() {   // compile the module once (instances are then made at once)
  if (brt.module || brt.loading) return brt.loading;
  if (typeof WebAssembly !== 'object' || typeof BOARD_WASM_B64 !== 'string') { brt.err = 'this browser has no WebAssembly: the flight computers can\'t run'; return null; }
  const bytes = Uint8Array.from(atob(BOARD_WASM_B64), c => c.charCodeAt(0));
  brt.loading = WebAssembly.compile(bytes).then(m => { brt.module = m; brt.sig = null; doReset(); }, e => { brt.err = 'the flight computers didn\'t load: ' + e.message; });
  return brt.loading;
}
const cstr = (w, ptr) => { const b = new Uint8Array(w.memory.buffer, ptr, 64); let n = 0; while (n < 64 && b[n]) n++; return new TextDecoder().decode(b.slice(0, n)); };
// The flight program as a board loads it: the formulas compiled here.
function boardImage(P = RN.P) {
  if (!P) throw new Error('the flight formulas don\'t compile' + (RN.buildErr ? ': ' + RN.buildErr : ''));
  const samples = []; for (const k of Object.keys(RN.samples || {})) for (const s of RN.samples[k]) samples.push(s);
  return rnImage(P, { tests: rnMakeTests(P, samples, 600) });
}
// The navigation's config (nav_core.h): mass, where the barometer, GPS antenna and flow camera sit, which it has.
function navConfigBlob() {
  const nav = boardOf('nav'), first = k => sensorsOf(k).find(c => !(k === 'fix' && c.dropout && false));
  const baro = first('baro'), fix = sensorsOf('fix').find(c => wiredTo(c) === nav), flow = sensorsOf('flow').find(c => wiredTo(c) === nav);
  const pos = c => c && c.known ? poseOf(c, restAngle).p : [0, 0, 0];
  const flowR = flow && flow.known ? m3m(poseOf(flow, restAngle).R, eulerR(flow.mount[0], flow.mount[1], flow.mount[2])) : [1, 0, 0, 0, 1, 0, 0, 0, 1];
  const refs = (baro ? 1 : 0) | (fix ? 2 : 0) | (flow ? 4 : 0);
  const f = new Float32Array([ctlModel().m, ...pos(baro), ...pos(fix), ...pos(flow), ...flowR, PILOT_LEVELS.sport.h, refs]);
  const body = new Uint8Array(8 + f.length * 4), dv = new DataView(body.buffer);
  dv.setUint32(0, 0x434E4644, true); dv.setUint32(4, 1, true); body.set(new Uint8Array(f.buffer), 8);
  const out = new Uint8Array(body.length + 4); out.set(body); new DataView(out.buffer).setUint32(body.length, rnCrc32(body), true);
  return out;
}
// (Re)start every board: at each reset, as if powered on with the program, the airframe and the config.
function boardsStart() {
  brt.ready = false; brt.toCore = []; brt.toNav = []; brt.tel = null; brt.navOut = null; brt.navReady = false; brt.home = null; brt.baroTs = null;
  brt.t = 0; brt.nextTel = 0; brt.nextNav = 0.005; brt.nextStick = 0; brt.fcState = 0; brt.fcWhy = ''; brt.navWhy = ''; brt.out = null; brt.err = '';
  Object.assign(brt.pilot, { arm: 0, fly: 0, thr: 0, phase: 'ground', t: 0 });
  if (!brt.module) { boardsLoad(); brt.err = brt.err || 'starting the flight computers…'; return; }
  const C = computers();
  for (const id of [...brt.inst.keys()]) if (!C.boards.some(b => b.id === id)) brt.inst.delete(id);
  let img; try { img = boardImage(); } catch (e) { brt.err = e.message; return; }
  for (const b of C.boards) {
    let w = brt.inst.get(b.id);
    if (!w) { w = new WebAssembly.Instance(brt.module, { env: RnWasm.env() }).exports; brt.inst.set(b.id, w); }
    if (!b.tasks.length) continue;
    if (img.length > w.img_cap()) { brt.err = 'the flight program is too big for the boards'; return; }
    new Uint8Array(w.memory.buffer, w.img_ptr(), img.length).set(img);
    const e = w.host_setup(img.length); if (e) { brt.err = `${b.name}: ${cstr(w, w.why_ptr())}`; return; }
    if (b.tasks.includes('core')) {
      let blob; try { blob = fcAirframeBlob({ imuBody: true }); } catch (x) { brt.err = 'Flight core: ' + x.message; brt.fcWhy = brt.err; return; }
      new Uint8Array(w.memory.buffer, w.blob_ptr(), blob.length).set(blob);
      if (w.fc_setup(blob.length)) { brt.err = 'Flight core: ' + cstr(w, w.why_ptr()); brt.fcWhy = brt.err; return; }
    }
    if (b.tasks.includes('nav')) {
      const cb = navConfigBlob(); new Uint8Array(w.memory.buffer, w.ncfg_ptr(), cb.length).set(cb);
      if (w.nav_setup(cb.length)) { brt.err = 'Navigation: ' + cstr(w, w.nav_why_ptr()); return; }
    }
  }
  brt.ready = true;
}
// A formula edited in flight: every board stages the new program through its own loading steps, as on the drone.
function boardsStageProgram() {
  if (!brt.ready) return;
  let img; try { img = boardImage(rnCompile(rnSources())); } catch (e) { rnEvent('The edit didn\'t compile for the boards: ' + e.message, 'bad'); return; }
  for (const b of computers().boards) {
    const w = brt.inst.get(b.id); if (!w || !b.tasks.length) continue;
    new Uint8Array(w.memory.buffer, w.img_ptr(), img.length).set(img);
    const e = w.stage(img.length);
    rnEvent(`${b.name}: ${e ? 'rejected the new program (error ' + e + ')' : 'self-tests passed; flying the new program in the background first'}`, e ? 'bad' : '');
    brt.staging = true;
  }
}
// What the boards' loaders did since (shown in the Computers tab): swapped in, or fell back.
function boardsHostEvents() {
  for (const b of computers().boards) {
    const w = brt.inst.get(b.id); if (!w || !b.tasks.length) continue;
    const e = w.host_event(); if (!e) continue;
    const txt = { 1: 'loaded the new program, flying it in the background', 2: 'rejected the new program', 3: 'flies the new program now', 4: 'the new program stopped: back to the one before', 5: 'even the built-in program stopped' }[e] || 'event ' + e;
    rnEvent(`${b.name}: ${txt}`, e === 3 ? 'good' : e === 1 ? '' : 'bad');
  }
}

/* ───────── each control step (1 kHz) ───────── */
const io = w => new Float32Array(w.memory.buffer, w.io_ptr(), 15 + 12 + 8 + 19);
const IO_OUT = 15, IO_STATE = 15 + 12 + 8;
function fcSend(cmd, delay) { brt.toCore.push({ at: brt.t + delay, cmd }); }
function boardsControl(dt) {
  const coreB = boardOf('core'), navB = boardOf('nav');
  const acts = actuators();
  const idle = () => { for (const c of acts) { const st = act.get(c.id); if (st) setThrottle(c, st, 0, 0); } };
  if (!brt.ready || !coreB) { idle(); return; }
  brt.t += dt;
  const W = brt.inst.get(coreB.id), nav = navB && brt.inst.get(navB.id), same = navB && navB.id === coreB.id;
  autoPilot(dt, !!navB);

  // The flight core: commands that have arrived, then a step on the newest IMU sample.
  while (brt.toCore.length && brt.toCore[0].at <= brt.t + 1e-9) {
    const c = brt.toCore.shift().cmd;
    new Float32Array(W.memory.buffer, W.cmd_ptr(), 12).set([c.arm, c.roll || 0, c.pitch || 0, c.yaw || 0, c.throttle || 0, -1, 0, c.guided ? 1 : 0, ...(c.acc || [0, 0, 0]), c.heading || 0]);
    W.fc_command();
  }
  const b = io(W), drv = est.drv || {};
  const baro = drv.baro && drv.baroTs !== brt.baroTs; if (baro) brt.baroTs = drv.baroTs;
  b.set([...(est.haveImu ? est.fGyro : [0, 0, 0]), ...(est.haveImu ? est.fAccel : [0, 0, 0]), est.haveImu ? 1 : 0, dt, fc.vComp && hread.b.V > 1 ? hread.b.V : 0,
    baro ? drv.baro.alt : 0, baro ? 1 : 0, ...(drv.mag || [0, 0, 0]), drv.mag ? 1 : 0], 0);
  W.host_tick(dt); W.fc_tick();
  acts.forEach((c, i) => { const st = act.get(c.id); if (st) { const u = S.crashed ? 0 : b[IO_OUT + i]; setThrottle(c, st, u, u); } });
  const steer = new Set(steerJoints());
  joints().forEach((j, k) => { const st = jst.get(j.id); if (st && steer.has(j)) st.thCmd = b[IO_OUT + 12 + k]; });
  const s = b.subarray(IO_STATE);
  est.q = [s[0], s[1], s[2], s[3]]; est.R = qmat(est.q); est.w = [s[4], s[5], s[6]];
  brt.out = { attOk: s[9] > 0.5, alt: s[14], haveAlt: s[15] > 0.5, vz: s[13], tau: [s[16], s[17], s[18]], sat: acts.some((c, i) => b[IO_OUT + i] >= 0.995) };
  const st = W.state(); if (st !== brt.fcState || (S.steps % 400 === 0)) { brt.fcState = st; brt.fcWhy = cstr(W, W.why_ptr()); }
  if (brt.staging && S.steps % 100 === 0) boardsHostEvents();

  // Telemetry to the navigation, 100 times a second (over the link, or within the board).
  if (navB && brt.t >= brt.nextTel) {
    brt.nextTel += 0.01;
    const tel = { q: est.q.slice(), w: est.w.slice(), acc: est.haveImu ? est.fAccel.slice() : [0, 0, 9.81], att: brt.out.attOk, alt: brt.out.alt, haveAlt: brt.out.haveAlt, ts: brt.t };
    brt.toNav.push({ at: brt.t + (same ? 0 : LINK_DELAY), tel });
  }
  while (brt.toNav.length && brt.toNav[0].at <= brt.t + 1e-9) brt.tel = brt.toNav.shift().tel;

  // The navigation, 100 times a second, on the newest telemetry and its own sensors.
  if (nav && brt.t >= brt.nextNav) {
    const ndt = 0.01; brt.nextNav += ndt;
    if (!same) nav.host_tick(ndt);
    const T = brt.tel, a = new Float32Array(nav.memory.buffer, nav.nio_ptr(), 41 + 13);
    const home = brt.home || S.p, sp = [setpoint.x - home[0], setpoint.y - home[1], setpoint.z - home[2]];
    const fix = drv.fix, flow = drv.flow;
    const useFix = fix && wiredTo(fix.c) === navB, useFlow = flow && wiredTo(flow.c) === navB;
    const baroNav = T && T.haveAlt;
    a.set([...(T ? T.q : [1, 0, 0, 0]), ...(T ? T.w : [0, 0, 0]), ...(T ? T.acc : [0, 0, 9.81]), T && T.att ? 1 : 0,
      baroNav ? 1 : 0, baroNav ? T.alt : 0, T ? brt.t - T.ts : 0,
      useFix ? 1 : 0, ...(useFix ? fix.p : [0, 0, 0]), ...(useFix ? fix.v : [0, 0, 0]), useFix ? S.t - fix.ts : 0,
      useFlow ? 1 : 0, ...(useFlow ? flow.flow : [0, 0]), useFlow ? flow.range : 0, useFlow ? flow.q : 0, useFlow ? S.t - flow.ts : 0,
      ...sp, ...pilot.vref, setpoint.yaw * D2R, brt.pilot.fly ? 1 : 0, ndt], 0);
    const err = nav.nav_tick();
    const o = a.subarray(41);
    brt.navOut = { acc: [o[0], o[1], o[2]], heading: o[3], fly: o[4] > 0.5, p: [o[5], o[6], o[7]], v: [o[8], o[9], o[10]], haveHome: o[11] > 0.5, ready: o[12] > 0.5, err };
    if (brt.navOut.haveHome && !brt.home) brt.home = S.p.slice();   // where it took off: the nav's home, in the world
    brt.navReady = brt.navOut.ready;
    if (S.steps % 400 === 0 || err) brt.navWhy = cstr(nav, nav.nav_why_ptr());
    // the command it sends the flight core (as pi_nav.c does): arm, fly, the acceleration and heading
    if (!err && T) fcSend({ arm: brt.pilot.arm, throttle: brt.navOut.fly ? 1 : 0, guided: 1, acc: brt.navOut.acc, heading: brt.navOut.heading }, same ? 0 : LINK_DELAY);
    else if (!T) fcSend({ arm: 0, throttle: 0, guided: 1, acc: [0, 0, 0], heading: 0 }, same ? 0 : LINK_DELAY);   // announcing itself, disarmed
  }
  // Without navigation: the pilot's sticks, 50 times a second (a radio, or fly.py over USB).
  if (!navB && brt.t >= brt.nextStick) { brt.nextStick += 0.02; fcSend(stickCommand(), 0.002); }

  // What the flight software believes, for the view and the panels.
  if (brt.navOut && brt.navOut.haveHome) { est.p = add(brt.home, brt.navOut.p); est.v = brt.navOut.v.slice(); est.havePos = true; }
  else { est.p = S.p.slice(); est.v = [0, 0, brt.out.vz]; est.havePos = false; }
}

/* ───────── the pilot, as the simulator plays it ───────── */
// At each reset the drone sits on the ground; the simulator then does what you'd do: arm, and take off to the
// target height. With navigation it asks the navigation to fly (it takes off once its position has settled); without,
// it opens the throttle until the barometer shows the drone climbing past most of the target height.
function autoPilot(dt, navigated) {
  const P = brt.pilot; P.t += dt;
  const att = brt.out && brt.out.attOk;
  if (P.phase === 'ground' && P.t > 0.6 && att) { P.arm = 1; P.phase = 'arming'; P.t = 0; }
  if (P.phase === 'arming' && P.t > 0.2) {
    if (brt.fcState !== 1) { if (P.t > 2) { P.phase = 'ground'; P.arm = 0; P.t = 0; } return; }   // didn't arm (the reason is shown): switch off and try again
    P.phase = 'takeoff'; P.t = 0; P.fly = navigated ? 1 : 0; P.thr = navigated ? 0 : 0.85;
  }
  if (P.phase === 'takeoff') {
    if (navigated) { if (brt.navOut && brt.navOut.fly && est.havePos && est.p[2] > (brt.home ? brt.home[2] : 0) + 0.5 * (setpoint.z - (brt.home ? brt.home[2] : 0))) P.phase = 'flying'; return; }
    const h = brt.out && brt.out.haveAlt ? S.p[2] : null;   // (the barometer's height, as the flight core sees it, rises with the true one)
    if (h != null ? brt.out.alt - (P.alt0 ?? (P.alt0 = brt.out.alt)) > 0.6 * setpoint.z : P.t > 0.9) { P.thr = 0.5; P.phase = 'flying'; }
  } else if (P.phase !== 'takeoff') P.alt0 = null;
  if (brt.fcState === 0 && P.phase === 'flying') { P.phase = 'landed'; P.arm = 0; P.fly = 0; }
}
// The sticks from the pilot keys (angle mode): the arrows lean, A/D turn, W/S climb or sink around the middle.
function stickCommand() {
  const P = brt.pilot, k = c => isHeld(c) ? 1 : 0, s = { gentle: 0.4, normal: 0.7, sport: 1 }[pilot.level] || 0.7;
  const flying = P.phase === 'flying';
  return { arm: P.arm, pitch: s * (k('fwd') - k('back')), roll: s * (k('right') - k('left')), yaw: k('yawL') - k('yawR'),
    throttle: P.thr > 0 ? clamp((flying ? 0.5 : P.thr) + 0.35 * (k('up') - k('down')), 0.06, 1) : 0 };
}
const flightPhaseText = () => !brt.ready ? (brt.err || 'starting') : { ground: 'on the ground', arming: 'arming', takeoff: brt.navOut && !brt.navOut.ready ? 'waiting for its position to settle' : 'taking off', flying: 'flying', landed: 'landed' }[brt.pilot.phase] || '';
