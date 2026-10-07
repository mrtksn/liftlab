'use strict';
// Flight computers: the boards on the drone, which tasks each one runs, the sensors wired to it, and the links
// between them. Only these fly the drone. Each board is the real flight code (runner/fc: fc_core.c, nav_core.c,
// the step runner, the flight program compiled from the formulas) built to WebAssembly, one instance per board,
// the same code the ESP32 and the Pi run. The simulator only supplies what the hardware would: sensor readings,
// what a board sends another over their link (after the link's delay), and the pilot.
//
// Tasks:
//   core   the flight core (fc_core.c), 1 kHz: attitude, control, mixing, arming and the failsafes. It needs exact
//          timing, so it runs on a microcontroller (an ESP32). Every drone has exactly one.
//   nav    the navigation (nav_core.c), 100 Hz: position from GPS, optical flow and the barometer, and holding or
//          moving it. It sends the flight core guided commands (an acceleration and a heading). On a Pi it talks to
//          the ESP32 over the serial link (runner/pi/dfb_pi.c); it can also run on the ESP32 itself.
//   learn  the learning (learn_core.c), on the flight core's 200 Hz telemetry: what each motor and servo really
//          does, in flight, in a hover calibration, or from a throw. It needs a Linux computer (a Pi).
//   super  the health supervisor (super_core.c), 10 Hz: failing, weakened or hot parts, and how to fly on what's
//          left. On a Pi, with the health sensors wired to it.
//   tlm    telemetry and the pilot's radio (tlm_core.c, rc_core.c): the board the ExpressLRS receiver is wired to.
//          Its channels fly the drone (to the flight core, or to the navigation's board); every task's telemetry
//          comes to it and goes down the radio as CRSF frames, within the link's budget. Usually the flight
//          controller (where receivers go: the sticks then don't depend on the Pi), it can be on the Pi instead.
//          Without it the simulator's pilot reaches the boards directly, as over a cable.
//   cargo  the latches (cargo_core.c), 50 Hz: drives each latch open or closed on the pilot's command (the radio's
//          LATCH command, or the simulator's buttons as over a cable), reads their load switches, and reports them.
//          Any board: a third ESP32 just for the latches, the flight controller, or the Pi (dfb_pi --latch).
// Without navigation the pilot flies the flight core directly, in angle mode (the sticks lean the drone).
// Each board loads a program with the formulas of its tasks.
//
// On the ground: the command module (runner/ground/ground_core.c), the pilot's side of the radio, when the drone has
// one. An ESP32 with buttons, a Pi or a Mac, wired to the ExpressLRS transmitter module; here another instance of the
// same WebAssembly with the ground program (its formulas: stickInput, groundAlerts). Your keys are its buttons; it
// sends the channels and commands up, and decodes what comes down for the Ground station.

const BOARD_KINDS = {
  esp32: { label: 'ESP32', mcu: true, mops: 60, cores: 2, ramKB: 300, note: 'microcontroller, 2 cores at 240 MHz' },
  s3: { label: 'ESP32-S3', mcu: true, mops: 80, cores: 2, ramKB: 300, note: 'microcontroller, 2 cores at 240 MHz, faster memory' },
  c3: { label: 'ESP32-C3', mcu: true, mops: 4, cores: 1, ramKB: 280, note: 'microcontroller, 1 core at 160 MHz, no float unit' },
  pizero: { label: 'Raspberry Pi Zero', mcu: false, mops: 150, cores: 1, ramKB: 512 * 1024, note: 'Linux computer, 1 core at 1 GHz' },
  pizero2: { label: 'Raspberry Pi Zero 2 W', mcu: false, mops: 600, cores: 4, ramKB: 512 * 1024, note: 'Linux computer, 4 cores at 1 GHz' },
  pi4: { label: 'Raspberry Pi 4', mcu: false, mops: 2000, cores: 4, ramKB: 4096 * 1024, note: 'Linux computer, 4 cores at 1.8 GHz' },
  mac: { label: 'Mac or PC', mcu: false, mops: 5000, cores: 8, ramKB: 8192 * 1024, note: 'a laptop or desktop (macOS or Linux) with a USB serial adapter', groundOnly: true },
};
// The command module: the pilot's side of the radio (not on the drone, so not in TASKS: it has one board of its own).
const GROUND = { label: 'Command module', hz: 250, formulas: RN_TASK_FORMULAS.ground,
  what: 'The pilot\'s side of the radio, wired to the ExpressLRS transmitter module (or its own ESP-NOW or Wi-Fi): buttons, sticks or your own code in, channels and commands up (through its stickInput formula), the telemetry decoded and checked (groundAlerts).' };
const TASKS = {
  core: { label: 'Flight core', hz: 1000, mcuOnly: true, formulas: RN_TASK_FORMULAS.core,
    what: 'Attitude, control and mixing at the board\'s flight-loop rate, arming and the failsafes. It needs exact timing, so it runs on a microcontroller.' },
  nav: { label: 'Navigation', hz: 100, formulas: RN_TASK_FORMULAS.nav,
    what: 'Where the drone is (GPS, optical flow, barometer) and holding or moving its position. It tells the flight core which way to accelerate and where to face. Without it you fly in angle mode: the keys lean the drone.' },
  learn: { label: 'Learning', hz: 200, piOnly: true, formulas: RN_TASK_FORMULAS.learn,
    what: 'Learns what each motor and servo really does: in flight, in a hover calibration, or from a throw (the throw start). It asks the flight core for test moves and tells it which model to fly on. Needs a Linux computer: the throw\'s fit alone keeps about 300 KB.' },
  super: { label: 'Health supervisor', hz: 10, piOnly: true, formulas: RN_TASK_FORMULAS.super,
    what: 'Watches for failing, weakened or overheating parts (from the flight core\'s data stream and the health sensors wired to its board), takes them out of the flight core\'s table or caps them, and decides how to fly on what\'s left: carefully, home, or straight down.' },
};
TASKS.tlm = { label: 'Telemetry & radio', hz: 200, formulas: [],
  what: 'The pilot\'s radio is on this board: an ExpressLRS receiver wired to it, or its own ESP-NOW or Wi-Fi (the Ground tab picks the link). Its channels fly the drone; the other tasks\' telemetry comes here and goes down the radio, as much as the link has room for. About 16 KB of an ESP32\'s memory.' };
TASKS.cargo = { label: 'Cargo', hz: 50, formulas: [],
  what: 'The latches are wired to this board: it opens and closes them on the pilot\'s command (the radio, or the buttons on the view) and reports what they hold. Any board will do: the flight controller, the Pi, or an ESP32 of its own.' };
// Tasks that are plain code, without formulas: a board that runs only these loads no flight program.
const NO_PROGRAM = new Set(['tlm', 'cargo']);
const needsProgram = b => b.tasks.some(t => !NO_PROGRAM.has(t));
const BOARD_MAX = 4, LINK_DELAY = 0.006;   // serial link (921600 baud): frame time plus scheduling, each way [s]

/* ───────── the configuration (part of the design) ───────── */
const defaultComputers = () => ({ boards: [
  { id: 1, kind: 'esp32', name: 'Flight controller', tasks: ['core', 'tlm'] },
  { id: 2, kind: 'pizero', name: 'Pi Zero', tasks: ['nav', 'learn', 'super'] },
], radio: 1, ground: { kind: 'esp32', name: 'Command module' } });
function computers() {
  if (!cfg.computers || !Array.isArray(cfg.computers.boards) || !cfg.computers.boards.length) cfg.computers = defaultComputers();
  return cfg.computers;
}
// Keep it valid: known kinds, one flight core on a microcontroller, the Pi's tasks on Linux computers, each task on one board.
function fixComputers(C) {
  C = C && Array.isArray(C.boards) ? JSON.parse(JSON.stringify(C)) : defaultComputers();
  C.boards = C.boards.filter(b => BOARD_KINDS[b.kind] && !BOARD_KINDS[b.kind].groundOnly).slice(0, BOARD_MAX);
  const g = C.ground && BOARD_KINDS[C.ground.kind] ? C.ground : { kind: 'esp32' };
  C.ground = { kind: g.kind, name: String(g.name || 'Command module').slice(0, 24), ...(g.wiring?{wiring:g.wiring}:{}) };
  if (!C.boards.some(b => BOARD_KINDS[b.kind].mcu)) { C.boards = C.boards.slice(0, BOARD_MAX - 1); C.boards.unshift({ id: 0, kind: 'esp32', name: 'Flight controller', tasks: [] }); }   // (room made for it)
  const ids = new Set(); let id = Math.max(Number.isInteger(C.nextBoardId)?C.nextBoardId:1, ...C.boards.map(b => Number.isInteger(b.id)?b.id+1:1)); for (const b of C.boards) { if (!Number.isInteger(b.id) || b.id < 1 || ids.has(b.id)) { while (ids.has(id)) id++; b.id = id++; } ids.add(b.id); b.name = String(b.name || BOARD_KINDS[b.kind].label).slice(0, 24); b.tasks = (b.tasks || []).filter(t => TASKS[t]); }
  C.nextBoardId = Math.max(id,...C.boards.map(b=>b.id+1));
  for (const t of Object.keys(TASKS)) { let seen = false; for (const b of C.boards) if (b.tasks.includes(t)) { if (seen || (TASKS[t].mcuOnly && !BOARD_KINDS[b.kind].mcu) || (TASKS[t].piOnly && BOARD_KINDS[b.kind].mcu)) b.tasks = b.tasks.filter(x => x !== t); else seen = true; } }
  if (!C.boards.some(b => b.tasks.includes('core'))) C.boards.find(b => BOARD_KINDS[b.kind].mcu).tasks.unshift('core');
  C.radio = 1;
  return C;
}
// Computers saved before the drone had a radio (no `radio` mark) get its receiver where a new drone has it: on the
// flight controller, with the flight core.
function computersWithRadio(C) {
  if (!C || !Array.isArray(C.boards) || C.radio) return C;
  C = JSON.parse(JSON.stringify(C));
  const cb = C.boards.find(b => (b.tasks || []).includes('core'));
  if (cb && !C.boards.some(b => (b.tasks || []).includes('tlm'))) cb.tasks.push('tlm');
  return C;
}
const boardOf = task => computers().boards.find(b => b.tasks.includes(task)) || null;
const hasTask = task => !!boardOf(task);
const boardName = b => b ? `${b.name}${b.name === BOARD_KINDS[b.kind].label ? '' : ' (' + BOARD_KINDS[b.kind].label + ')'}` : '';
// Which board a sensor is wired to. The IMU, compass and barometer (the GY-87 is all three) go to the flight core's
// board; the GPS and the flow camera to the board that navigates (with no navigation they'd be unused).
function wiredTo(c) {
  return hardwareOwner(computers(), c);
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
  const n = actuators().length + joints().length, m = actuators().length, inputs = Math.max(1, learn.n || m);
  const C = k => { const f = FLIGHT_COST[k]; try { return f ? f({}, null, null, {}, {}, {}) : 0; } catch (e) { return 0; } };
  if (task === 'core') return C('attitudeEstimator') + 12 * joints().length + C('thrustAxisTarget') + C('attitudeError') + C('attitudeControl') + C('forceDemand')
    + 2 * (30 * n + 60 * n * n + 200) + m * (C('thrustLinearization') + 10) + 1500;   // allocation (two stages), throttles, the code around
  if (task === 'nav') return FLIGHT_COST.positionEstimator({}, null, null, 1, 1, { v: 1 }) + C('flowVelocity') + C('positionControl') + 400;
  if (task === 'learn') { const k = 2 * inputs; return 5 * k * k + 30 * k + 40 * inputs + 150 + 300; }   // the in-flight learning, every frame
  if (task === 'super') return 5 * m * 40 + 2 * (30 * n + 60 * n * n + 200) + 800;   // the health check, the margins (two solves), the decisions
  if (task === 'tlm') return 400;   // parsing the receiver's bytes, packing and scheduling the telemetry
  if (task === 'cargo') return 60 + 20 * latches().length;
  if (task === 'ground') return 4 * FLIGHT_COST.stickInput() + FLIGHT_COST.groundAlerts() / 25 + 300;   // the sticks every step, the alerts 10 times a second, the frames
  return 0;
}
function groundBudget() {
  const K = BOARD_KINDS[computers().ground.kind], ops = taskCost('ground') * GROUND.hz;
  let memKB = 0; try { const P = boardProgram(['ground']); memKB = (P.arenaSize * 4 + P.code.length * 4) / 1024; } catch (e) { }
  return { ops, load: ops / (K.mops * 1e6) * (K.mcu ? 1 : 1.5), memKB, ramKB: K.ramKB };
}
// A board's program: the formulas of its tasks (sizes from the compiled image, cached by task set).
const boardProgCache = new Map();
function boardProgram(tasks, srcs = rnSources()) {
  const keys = rnTaskFormulas(tasks), key = tasks.join(',') + '|' + keys.map(k => srcs[k]).join('\u0000');
  let P = boardProgCache.get(key);
  if (!P) {
    P = rnCompileAll(Object.fromEntries(keys.map(k => [k, srcs[k]])), RN_SIGS);
    const bad = Object.entries(P.errors); if (bad.length) throw new Error(`${bad[0][0]}: ${bad[0][1]}`);
    rnVerify(P);
    if (boardProgCache.size > 12) boardProgCache.clear();
    boardProgCache.set(key, P);
  }
  return P;
}
const boardTaskHz = (b, task) => task === 'core' && b.kind === 'c3' ? 250 : TASKS[task].hz;
function boardBudget(b) {
  const K = BOARD_KINDS[b.kind];
  const ops = b.tasks.reduce((s, t) => s + taskCost(t) * boardTaskHz(b, t), 0);
  const load = ops / (K.mops * 1e6) * (K.mcu ? 1 : 1.5);   // a Linux board loses some to the system
  let memKB = 0; if (needsProgram(b)) try { const P = boardProgram(b.tasks); memKB = (P.arenaSize * 4 + P.code.length * 4) / 1024; } catch (e) { }
  return { ops, load, memKB, ramKB: K.ramKB };
}

/* ───────── the runtime: one instance of the flight code per board ───────── */
let brt = {
  module: null, err: '', inst: new Map(), sig: null, ready: false,
  toCore: [], toNav: [], q: [], tel: null, navOut: null, navReady: false, home: null,
  coreElapsed: 0, t: 0, nextTel: 0, nextNav: 0.005, nextStick: 0, nextLtel: 0, nextHealth: 0, nextView: 0, nextRadio: 0, nextPub: 0, nextPack: 0, nextRc: 0, nextGnd: 0, nextGsRead: 0, nextCargo: 0, cargoN: 0,
  gnd: null, gndErr: '', gndOk: false,   // the command module's instance (while the drone has a radio); gndOk: its program runs (stickInput, groundAlerts)
  pilot: { arm: 0, fly: 0, thr: 0, phase: 'ground', t: 0 },
  fcState: 0, fcWhy: '', navWhy: '', out: null, pickup: 0, baroTs: null, superView: null, learnErr: '', srcs: new Map(),
};
const FC_STATES = ['disarmed', 'armed', 'failsafe', 'crashed', 'motor test'];
// learn_core.h commands
const LN_CMD = { calibrate: 1, stop: 2, useDesc: 3, useLearned: 4, keepOn: 5, keepOff: 6, throw: 7, holdOn: 8, holdOff: 9, thenCalOn: 10, thenCalOff: 11 };
let boardsModulePromise = null;
function boardsLoad() {   // compile the module once (instances are then made at once)
  if (brt.module || brt.loading) return brt.loading;
  if (typeof WebAssembly !== 'object' || typeof BOARD_WASM_B64 !== 'string') { brt.err = 'this browser has no WebAssembly: the flight computers can\'t run'; return null; }
  boardsModulePromise ||= WebAssembly.compile(Uint8Array.from(atob(BOARD_WASM_B64), c => c.charCodeAt(0)));
  const owner = brt, callback = fn => typeof window.runDroneCallback === 'function' ? window.runDroneCallback(owner,fn) : fn();
  brt.loading = boardsModulePromise.then(m => { owner.module = m; callback(() => { brt.sig = null; resetSim(); }); }, e => { owner.err = 'the flight computers didn\'t load: ' + e.message; });
  return brt.loading;
}
const cstr = (w, ptr, max = 64) => { const b = new Uint8Array(w.memory.buffer, ptr, max); let n = 0; while (n < max && b[n]) n++; return new TextDecoder().decode(b.slice(0, n)); };
const frIn = (w, data) => { new Float32Array(w.memory.buffer, w.fr_ptr(), data.length).set(data); return data.length; };
const frOut = (w, n) => Float32Array.from(new Float32Array(w.memory.buffer, w.fr_ptr(), n));
// A board's program as it loads it: its tasks' formulas, compiled here, with self-tests from this session's calls.
function boardImage(b, srcs) {
  if (!RN.P && !srcs) throw new Error('the flight formulas don\'t compile' + (RN.buildErr ? ': ' + RN.buildErr : ''));
  const P = boardProgram(b.tasks, srcs);
  const samples = []; for (const k of Object.keys(RN.samples || {})) if (P.fns[k]) for (const s of RN.samples[k]) samples.push(s);
  for (const k of Object.keys(P.fns)) if (!(RN.samples || {})[k]) { const d = LAW_DEFS.find(d => d.key === k); if (d && d.sample) samples.push({ key: k, args: d.sample() }); }   // (not called here yet: its own sample)
  return rnImage(P, { tests: rnMakeTests(P, samples, 600) });
}
// The navigation's config (nav_core.h): mass, where the barometer, GPS antenna and flow camera sit, which it has.
function navConfigBlob() {
  const nav = boardOf('nav');
  const baro = sensorsOf('baro')[0], fix = sensorsOf('fix').find(c => wiredTo(c) === nav), flow = sensorsOf('flow').find(c => wiredTo(c) === nav);
  const pos = c => c && c.known ? poseOf(c, restAngle).p : [0, 0, 0];
  const flowR = flow && flow.known ? m3m(poseOf(flow, restAngle).R, eulerR(flow.mount[0], flow.mount[1], flow.mount[2])) : [1, 0, 0, 0, 1, 0, 0, 0, 1];
  const refs = (baro ? 1 : 0) | (fix ? 2 : 0) | (flow ? 4 : 0);
  return f32Blob(0x434E4644, [ctlModel().m, ...pos(baro), ...pos(fix), ...pos(flow), ...flowR, PILOT_LEVELS.sport.h, refs]);
}
// The Pi's config (learn_core.h learn_config): the IMU's position, the throw's hand height, each motor's heat
// model, the battery.
function piConfigBlob() {
  const imus = sensorsOf('imu'), r = imus.length ? mean3(imus.map(knownPos)) : [0, 0, 0], b = battCfg(), acts = actuators();
  const f = [...r, throwCfg.handH, ambient(), acts.length];
  for (const c of acts) { const mp = motorParams(c), th = motorThermal(c); f.push(isCollective(c) ? 1 : 0, c.tmaxC ?? 120, mp.R, mp.Om, th.C, th.Gf); }
  f.push(b.cells, b.rInt, b.tmaxC);
  return f32Blob(0x434C4644, f);
}
function f32Blob(magic, vals) {
  const f = new Float32Array(vals), body = new Uint8Array(8 + f.length * 4), dv = new DataView(body.buffer);
  dv.setUint32(0, magic, true); dv.setUint32(4, 1, true); body.set(new Uint8Array(f.buffer), 8);
  const out = new Uint8Array(body.length + 4); out.set(body); new DataView(out.buffer).setUint32(body.length, rnCrc32(body), true);
  return out;
}
// (Re)start every board: at each reset, as if powered on with the program, the airframe and the configs.
function boardsStart() {
  brt.gnd = null; brt.gndErr = ''; brt.gndOk = false;               // (the last run's command module is no one's until groundStart: nothing is queued into it)
  brt.ready = false; brt.toCore = []; brt.toNav = []; brt.q = []; brt.tel = null; brt.navOut = null; brt.navReady = false; brt.home = null; brt.baroTs = null;
  brt.coreElapsed = 0; brt.t = 0; brt.nextTel = 0; brt.nextNav = 0.005; brt.nextStick = 0; brt.nextLtel = 0; brt.nextHealth = 0; brt.nextView = 0;
  brt.nextRadio = 0; brt.nextPub = 0; brt.nextPack = 0; brt.nextRc = 0; brt.nextGnd = 0; brt.nextGsRead = 0; brt.nextCargo = 0; brt.cargoN = 0; gsSet.x = null; gsSet.pending = null; radioReset();
  brt.fcState = 0; brt.fcWhy = ''; brt.navWhy = ''; brt.out = null; brt.pickup = 0; brt.err = ''; brt.superView = null; brt.learnErr = ''; brt.superLogSeq = 0;
  Object.assign(brt.pilot, { arm: 0, fly: 0, thr: 0, phase: 'ground', t: 0, downT: 0, flat: false });
  learn.view = null; learn.msg = '';
  if (!brt.module) { boardsLoad(); brt.err = brt.err || 'starting the flight computers…'; return; }
  const C = computers();
  for (const id of [...brt.inst.keys()]) if (!C.boards.some(b => b.id === id)) brt.inst.delete(id);
  let af = null, afErr = '';
  try { af = fcAirframeBlob({ imuBody: true }); } catch (x) { afErr = x.message; }
  for (const b of C.boards) {
    let w = brt.inst.get(b.id);
    if (!w) { w = new WebAssembly.Instance(brt.module, { env: RnWasm.env() }).exports; brt.inst.set(b.id, w); }
    w.tlm_setup(b.tasks.includes('tlm') ? 1 : 0); w.radio_link(...radioModel().wasm(radioCfg));
    if (b.tasks.includes('cargo')) {                                // the latches, as they were set up: closed or open
      const ls = latches().slice(0, 8); frIn(w, ls.map(l => l.travel ?? 0.15));
      w.cargo_setup(ls.length, ls.reduce((m, l, i) => m | (l.closed ? 1 << i : 0), 0)); brt.cargoN = w.cargo_nmsg();
    }
    if (!needsProgram(b)) continue;                                 // (the radio and the latches need no flight program)
    let img; try { img = boardImage(b); } catch (e) { brt.err = `${b.name}: ${e.message}`; return; }
    brt.srcs.set(b.id, boardSrcKey(b.tasks, rnSources()));
    if (img.length > w.img_cap()) { brt.err = `${b.name}: the flight program is too big for the board`; return; }
    new Uint8Array(w.memory.buffer, w.img_ptr(), img.length).set(img);
    const e = w.host_setup(img.length); if (e) { brt.err = `${b.name}: ${cstr(w, w.why_ptr())}`; return; }
    const needAf = b.tasks.some(t => t !== 'nav' && !NO_PROGRAM.has(t));
    if (needAf && !af) { brt.err = 'Flight core: ' + afErr; brt.fcWhy = brt.err; return; }
    if (needAf) new Uint8Array(w.memory.buffer, w.blob_ptr(), af.length).set(af);
    if (b.tasks.includes('core') && w.fc_setup(af.length)) { brt.err = 'Flight core: ' + cstr(w, w.why_ptr()); brt.fcWhy = brt.err; return; }
    if (b.tasks.includes('nav')) {
      const cb = navConfigBlob(); new Uint8Array(w.memory.buffer, w.ncfg_ptr(), cb.length).set(cb);
      if (w.nav_setup(cb.length)) { brt.err = 'Navigation: ' + cstr(w, w.nav_why_ptr()); return; }
    }
    if (b.tasks.includes('learn') || b.tasks.includes('super')) {
      const pc = piConfigBlob(); new Uint8Array(w.memory.buffer, w.lcfg_ptr(), pc.length).set(pc);
      if (b.tasks.includes('learn') && w.learn_setup(af.length, pc.length)) brt.learnErr = 'Learning: ' + cstr(w, w.learn_msg_ptr(), 200);
      if (b.tasks.includes('learn')) { w.learn_cmd(throwCfg.thenCalibrate ? LN_CMD.thenCalOn : LN_CMD.thenCalOff); w.learn_cmd(learnPrefs.keep ? LN_CMD.keepOn : LN_CMD.keepOff); w.learn_cmd(learnPrefs.holdPulses ? LN_CMD.holdOn : LN_CMD.holdOff); }
      if (b.tasks.includes('super') && w.super_setup(af.length, pc.length)) brt.err = 'Supervisor: ' + cstr(w, w.super_why_ptr(), 96);
    }
  }
  groundStart();
  radioLinkSetup();                                                  // (a packet link: both ends' packet layers, with the binding phrase)
  if (typeof peerSetup === 'function') peerSetup();                  // (the drone's own link to the others in the fleet: peer-air.js)
  brt.ready = true;
}
// The command module: its own instance with the ground program, started with the boards (when the drone has a radio).
// A program that doesn't compile, fit or load leaves it with none (not the last run's): it sends the raw sticks
// unshaped and groundAlerts doesn't run, and the message says so.
function groundStart() {
  brt.gndErr = ''; brt.gndOk = false;
  if (!hasTask('tlm')) { brt.gnd = null; return; }
  if (!brt.gndInst) brt.gndInst = new WebAssembly.Instance(brt.module, { env: RnWasm.env() }).exports;
  const g = brt.gndInst, G = { name: computers().ground.name, tasks: ['ground'] };
  let img = null, why = '';
  try { img = boardImage(G); } catch (e) { why = `its program didn't compile (${e.message})`; }
  if (img && img.length > g.img_cap()) { why = `its program (${img.length} bytes) is bigger than its memory (${g.img_cap()})`; img = null; }
  if (img) {
    new Uint8Array(g.memory.buffer, g.img_ptr(), img.length).set(img);
    const e = g.host_setup(img.length); if (e) why = `its program didn't load (error ${e})`;
  } else g.host_setup(0);                                            // (an empty image: no program at all)
  brt.srcs.set('ground', boardSrcKey(['ground'], rnSources()));      // (an edit is compared with what it was started with)
  const e = g.gnd_setup(0, 0); brt.gndOk = !e;
  if (why) brt.gndErr = `${G.name}: ${why}: the raw sticks go up unshaped, and groundAlerts doesn't run`;
  else if (e) brt.gndErr = `${G.name}: ${cstr(g, g.gnd_why_ptr(), 96)}`;   // (it still sends the raw sticks)
  brt.gnd = g;
}
let learnPrefs = { keep: true, holdPulses: true };
const boardSrcKey = (tasks, srcs) => rnTaskFormulas(tasks).map(k => srcs[k]).join('\u0000');
// A formula edited in flight: every board whose tasks use it stages its new program through its own loading steps.
function boardsStageProgram() {
  if (!brt.ready) return;
  const srcs = rnSources();
  for (const b of computers().boards) {
    const w = brt.inst.get(b.id); if (!w || !needsProgram(b)) continue;
    const key = boardSrcKey(b.tasks, srcs); if (brt.srcs.get(b.id) === key) continue;   // none of its formulas changed
    let img; try { img = boardImage(b, srcs); } catch (e) { rnEvent(`${b.name}: the edit didn't compile for it: ${e.message}`, 'bad'); continue; }
    brt.srcs.set(b.id, key);
    if (img.length > w.img_cap()) { rnEvent(`${b.name}: the edited program is too big for the board`, 'bad'); continue; }
    new Uint8Array(w.memory.buffer, w.img_ptr(), img.length).set(img);
    const e = w.stage(img.length);
    rnEvent(`${b.name}: ${e ? 'rejected the new program (error ' + e + ')' : 'self-tests passed; flying the new program in the background first'}`, e ? 'bad' : '');
    brt.staging = true;
  }
  const g = brt.gnd, key = boardSrcKey(['ground'], srcs);
  if (g && brt.srcs.get('ground') !== key && !brt.gndOk) {          // no program running there to hand over from: it takes it as it starts
    brt.srcs.set('ground', key); rnEvent(`${computers().ground.name}: its program isn't running; it loads the edited one at the next reset`, 'warn');
  } else if (g && brt.srcs.get('ground') !== key) {                // the command module, the same way
    const nm = computers().ground.name;
    let img; try { img = boardImage({ tasks: ['ground'] }, srcs); } catch (e) { rnEvent(`${nm}: the edit didn't compile for it: ${e.message}`, 'bad'); return; }
    brt.srcs.set('ground', key);
    if (img.length > g.img_cap()) { rnEvent(`${nm}: the edited program is too big for it`, 'bad'); return; }
    new Uint8Array(g.memory.buffer, g.img_ptr(), img.length).set(img);
    const e = g.stage(img.length);
    rnEvent(`${nm}: ${e ? 'rejected the new program (error ' + e + ')' : 'self-tests passed; running the new program in the background first'}`, e ? 'bad' : '');
    brt.staging = true;
  }
}
// What the boards' loaders did since (shown in the Computers tab): swapped in, or fell back.
function boardsHostEvents() {
  for (const b of computers().boards) {
    const w = brt.inst.get(b.id); if (!w || !needsProgram(b)) continue;
    const e = w.host_event(); if (!e) continue;
    rnEvent(`${b.name}: ${HOST_EVENTS[e] || 'event ' + e}`, e === 3 ? 'good' : e === 1 ? '' : 'bad');
  }
  const e = brt.gnd ? brt.gnd.host_event() : 0;
  if (e) rnEvent(`${computers().ground.name}: ${(HOST_EVENTS[e] || 'event ' + e).replace('flies', 'runs').replace('flying', 'running')}`, e === 3 ? 'good' : e === 1 ? '' : 'bad');
}
const HOST_EVENTS = { 1: 'loaded the new program, flying it in the background', 2: 'rejected the new program', 3: 'flies the new program now', 4: 'the new program stopped: back to the one before', 5: 'even the built-in program stopped' };
// What the pilot asks the learning (the Learning panel, the throw): with a radio and the navigation it goes up the
// link as a command (the navigation passes it on, as dfb_pi does); otherwise straight to the learning's board.
function pilotLearnCmd(name) {
  if (brt.gnd && hasTask('nav') && hasTask('learn')) return radioCommand(2, [LN_CMD[name]]);
  return boardsLearnCmd(name);
}
// The learning task's commands, on its board.
function boardsLearnCmd(name) {
  const b = boardOf('learn'), w = b && brt.ready && brt.inst.get(b.id); if (!w) return -1;
  const r = w.learn_cmd(LN_CMD[name]); boardsReadViews(true); return r;
}

// The cargo task's step, 50 times a second on its board: the load switches in (each latch with one fitted reads
// whether something hangs from it), the latches out (cargo.js moves them), and what it says.
function cargoTick() {
  const b = boardOf('cargo'), w = b && brt.inst.get(b.id);
  if (!w || brt.t < brt.nextCargo - 1e-9) return;
  brt.nextCargo += 0.02;
  const ls = latches().slice(0, 8);
  const loaded = ls.reduce((m, l, i) => m | (latchLoaded(l) ? 1 << i : 0), 0), sw = ls.reduce((m, l, i) => m | (l.sense !== false ? 1 << i : 0), 0);
  cargoDrive(w.cargo_tick(0.02, brt.t, loaded, sw));
  const n = w.cargo_nmsg(); if (n !== brt.cargoN) { brt.cargoN = n; cargoLog(`${b.name}: ${cstr(w, w.cargo_msg_ptr())}`, 'board'); }
}
// Stop a pickup: with a radio, as a pilot would (the hold switch); without, on the navigation's board.
function pickupStop() {
  if (brt.gnd) { radioHold(); return; }
  const b = boardOf('nav'), w = b && brt.ready && brt.inst.get(b.id); if (w) w.pickup_stop();
}
// A latch command from the pilot (the buttons on the view, G): with a radio it goes up the link as a LATCH command
// (the command module queues it), otherwise straight to the cargo task's board, as over a cable. latch: 0… or −1 all;
// action: 0 open, 1 close, 2 toggle. Returns '' or why it can't.
function pilotCargoCmd(latch, action) {
  const b = boardOf('cargo');
  if (!b) return 'no board runs the Cargo task (Computers tab)';
  if (!cargo.power) return 'the drone has no power';
  if (brt.gnd) return radioCommand(3, [latch, action]) ? 'the command module has too many commands waiting' : '';
  const w = brt.ready && brt.inst.get(b.id); if (!w) return 'the boards aren\'t running';
  return w.cargo_cmd(latch, action) ? 'no such latch' : '';
}

/* ───────── frames between the tasks ───────── */
// Each goes over the serial link (LINK_DELAY) unless both tasks are on the same board.
function sendFrame(fromB, toB, kind, data) { if (toB) brt.q.push({ at: brt.t + (fromB && toB.id === fromB.id ? 0 : LINK_DELAY), to: toB, kind, data }); }
function deliverFrames() {
  const coreB = boardOf('core'), navB = boardOf('nav'), learnB = boardOf('learn'), superB = boardOf('super');
  for (let k = 0; k < brt.q.length;) {
    const m = brt.q[k]; if (m.at > brt.t + 1e-9) { k++; continue; }
    brt.q.splice(k, 1);
    const w = brt.inst.get(m.to.id); if (!w) continue;
    const n = frIn(w, m.data);
    if (m.kind === 'exc') w.fc_exc(n);
    else if (m.kind === 'tlm') w.tlm_unpack(n, brt.t);
    else if (m.kind === 'rc') w.rc_unpack(n, brt.t);
    else if (m.kind === 'cargo') w.cargo_cmd(m.data[0], m.data[1]);   // a pickup's request: close the latch
    else if (m.kind === 'model') { if (coreB && m.to.id === coreB.id) w.fc_model(n); if (superB && m.to.id === superB.id) w.super_model(n); }
    else if (m.kind === 'set') {
      if (coreB && m.to.id === coreB.id) { flightRememberSettings(w, m.data); w.fc_set(n); setJointView(m.data); }
      if (navB && m.to.id === navB.id) w.nav_set(n);
      if (learnB && m.to.id === learnB.id) { frIn(w, m.data); w.learn_set(n); }
    } else if (m.kind === 'ltel') {
      if (learnB && m.to.id === learnB.id) {
        w.learn_ltel(n);
        const ne = w.learn_exc(); if (ne) sendFrame(learnB, coreB, 'exc', frOut(w, ne));
        const nm = w.learn_model(); if (nm) { const d = frOut(w, nm); sendFrame(learnB, coreB, 'model', d); if (superB) sendFrame(learnB, superB, 'model', d); }
      }
      if (superB && m.to.id === superB.id) {
        frIn(w, m.data); w.super_ltel(n);
        const ns = w.super_set(); if (ns) { const d = frOut(w, ns); sendFrame(superB, coreB, 'set', d); if (navB) sendFrame(superB, navB, 'set', d); if (learnB) sendFrame(superB, learnB, 'set', d); }
      }
    }
  }
}
function setJointView(d) {   // where the supervisor told the flight core a stuck servo is (the simulator's view of what it believes)
  const nm = d[4], js = joints(); fc.jAng.clear();
  js.forEach((j, k) => { if (d[6 + 3 * nm + 2 * k] > 0.5) fc.jAng.set(j.id, d[7 + 3 * nm + 2 * k]); });
}
// The learning's and the supervisor's state for the screen, a few times a second.
function boardsReadViews(now) {
  if (!now && brt.t < brt.nextView) return; brt.nextView = brt.t + 0.2;
  const lb = boardOf('learn'), lw = lb && brt.inst.get(lb.id);
  if (lw && brt.ready) {
    const n = lw.learn_status(), o = new Float32Array(lw.memory.buffer, lw.fr_ptr(), n), nm = actuators().length, nj = joints().length, ni = o[17];
    const motors = [], js = [], B = [];
    let k = 24;
    for (let i = 0; i < nm; i++, k += 4) motors.push({ measured: o[k] > 0.5, tau: o[k + 1], curve: o[k + 2], fit: o[k + 3] });
    for (let j = 0; j < nj; j++, k += 4) js.push({ measured: o[k] > 0.5, rate: o[k + 1], lag: o[k + 2], fit: o[k + 3] });
    for (let r = 0; r < 6; r++) { const row = []; for (let j = 0; j < ni; j++) row.push(o[k++]); B.push(row); }
    learn.view = { useLearned: o[0] > 0.5, keep: o[1] > 0.5, cal: o[2] > 0.5, calProg: o[3], held: o[4] > 0.5, segKind: o[5], segWho: o[6], left: o[7], thr: o[8], thrProg: o[9], pulseMotor: o[10],
      haveFit: o[11] > 0.5, fit: { rot: o[12], force: o[13], descRot: o[14], descForce: o[15] }, refining: o[16] > 0.5, n: ni, holdPulses: o[18] > 0.5, thenCal: o[19] > 0.5, holdServos: o[20] > 0.5,
      motors, joints: js, B };
    learn.msg = brt.learnErr || cstr(lw, lw.learn_msg_ptr(), 480);
  } else learn.view = null;
  const sb = boardOf('super'), sw = sb && brt.inst.get(sb.id);
  if (sw && brt.ready) {
    const n = sw.super_status(), o = new Float32Array(sw.memory.buffer, sw.fr_ptr(), n), nm = actuators().length, nj = joints().length;
    const v = { mode: o[0], why: o[1], margin: o[3], rpOk: o[4] > 0.5, yawOk: o[5] > 0.5, soc: o[6], cells: o[7], lim: { speed: o[8], lean: o[9], accel: o[10] }, seq: o[11], motors: [], joints: [] };
    let k = 12;
    for (let i = 0; i < nm; i++, k += 9) v.motors.push({ state: o[k], on: o[k + 1] > 0.5, eff: o[k + 2], cap: o[k + 3], why: o[k + 4], val: o[k + 5], temp: o[k + 6] > -900 ? o[k + 6] : null, eta: o[k + 7], conf: o[k + 8] });
    for (let j = 0; j < nj; j++, k += 4) v.joints.push({ off: o[k] > 0.5, angle: o[k + 1], why: o[k + 2], val: o[k + 3] });
    v.motors.forEach((m, i) => { m.whyText = m.why ? cstr(sw, sw.super_motor_why(i), 120) : ''; });
    v.modeWhy = cstr(sw, sw.super_mode_why(), 96);
    v.log = []; for (let i = 0, ln = sw.super_log_n(); i < ln; i++) v.log.push({ t: sw.super_log_t(i), tone: ['info', 'good', 'warn', 'bad'][sw.super_log_tone(i)] || 'info', msg: cstr(sw, sw.super_log_text(i), 160) });
    brt.superView = v;
  } else brt.superView = null;
}

/* ───────── each control step (1 kHz) ───────── */
const io = w => new Float32Array(w.memory.buffer, w.io_ptr(), 15 + 12 + 8 + 19);
const IO_OUT = 15, IO_STATE = 15 + 12 + 8;
function fcSend(cmd, delay) { brt.toCore.push({ at: brt.t + delay, cmd }); }
function boardsControl(dt) {
  const coreB = boardOf('core'), navB = boardOf('nav'), learnB = boardOf('learn'), superB = boardOf('super');
  const acts = actuators();
  const idle = () => { for (const c of acts) { const st = act.get(c.id); if (st) setThrottle(c, st, 0, 0); } };
  if (!brt.ready || !coreB) { idle(); return; }
  brt.t += dt;
  if (!cargo.power) {                                               // no battery on board: every board is dark (cargo.js)
    idle(); brt.fcState = 0; brt.fcWhy = 'no power: the battery fell off'; brt.navOut = null;
    const tlmB = boardOf('tlm'), tw = tlmB && brt.inst.get(tlmB.id);
    if (tw) radioTick(dt, tlmB, tw, coreB, navB, true);             // (the command module on the ground runs on, and hears nothing)
    return;
  }
  const W = brt.inst.get(coreB.id), nav = navB && brt.inst.get(navB.id), same = navB && navB.id === coreB.id;
  throwHandTick(dt);
  autoPilot(dt, !!navB);
  for (const b of computers().boards) { const w = brt.inst.get(b.id); if (w && needsProgram(b)) w.host_tick(dt); }   // the loaders' steps, on every board
  if (typeof peerStep === 'function') peerStep();                    // (the other drones: what its peer end sends, what reached it)
  deliverFrames();
  const tlmB = boardOf('tlm'), tw = tlmB && brt.inst.get(tlmB.id);
  if (tw) radioTick(dt, tlmB, tw, coreB, navB);
  cargoTick();

  // The flight core: commands that have arrived, then a step on the newest IMU sample.
  while (brt.toCore.length && brt.toCore[0].at <= brt.t + 1e-9) {
    const c = brt.toCore.shift().cmd;
    new Float32Array(W.memory.buffer, W.cmd_ptr(), 12).set([c.arm, c.roll || 0, c.pitch || 0, c.yaw || 0, c.throttle || 0, -1, 0, c.guided ? 1 : 0, ...(c.acc || [0, 0, 0]), c.heading || 0]);
    W.fc_command();
  }
  const drv = est.drv || {};
  brt.coreElapsed += dt;
  const coreDt = brt.coreElapsed;
  if (!brt.out || coreDt + 1e-9 >= 1 / boardTaskHz(coreB, 'core')) {
    brt.coreElapsed = 0;
    const b = io(W);
    const baro = drv.baro && drv.baroTs !== brt.baroTs; if (baro) brt.baroTs = drv.baroTs;
    b.set([...(est.haveImu ? est.fGyro : [0, 0, 0]), ...(est.haveImu ? est.fAccel : [0, 0, 0]), est.haveImu ? 1 : 0, coreDt, fc.vComp && hread.b.V > 1 ? hread.b.V : 0,
      baro ? drv.baro.alt : 0, baro ? 1 : 0, ...(drv.mag || [0, 0, 0]), drv.mag ? 1 : 0], 0);
    flightApplyLimits(W, fc.vComp && hread.b.V > 1 ? hread.b.V : 0, acts);
    W.fc_tick();
    acts.forEach((c, i) => { const st = act.get(c.id); if (st) { const u = S.crashed || wiredTo(c) !== coreB ? 0 : b[IO_OUT + i]; setThrottle(c, st, u, u); } });
    joints().forEach((j, k) => { const st = jst.get(j.id); if (st && wiredTo(j) === coreB) st.thCmd = b[IO_OUT + 12 + k]; });
    const s = b.subarray(IO_STATE);
    est.q = [s[0], s[1], s[2], s[3]]; est.R = qmat(est.q); est.w = [s[4], s[5], s[6]];
    brt.out = { attOk: s[9] > 0.5, alt: s[14], haveAlt: s[15] > 0.5, vz: s[13], tau: [s[16], s[17], s[18]], sat: acts.some((c, i) => b[IO_OUT + i] >= Math.max(.001, flightDuty(c) - .005)) };
    const st = W.state(); if (st !== brt.fcState || (S.steps % 400 === 0)) { brt.fcState = st; brt.fcWhy = cstr(W, W.why_ptr()); }
  }
  if (brt.fcState === 3 && thr) thr = null;   // crashed: the throw is over
  if (brt.staging && S.steps % 100 === 0) boardsHostEvents();

  // Telemetry to the learning and the supervisor, 200 times a second; the health sensors to the supervisor's board.
  if ((learnB || superB) && brt.t >= brt.nextLtel - 1e-9) {
    brt.nextLtel += 0.005;
    const d = frOut(W, W.fc_ltel());
    if (learnB) sendFrame(coreB, learnB, 'ltel', d);
    if (superB && (!learnB || superB.id !== learnB.id)) sendFrame(coreB, superB, 'ltel', d);   // (on one board, one frame serves both)
  }
  if (superB && brt.t >= brt.nextHealth) { brt.nextHealth = brt.t + 0.1; const sw = brt.inst.get(superB.id), h = healthReadings(); if (sw) { frIn(sw, h); sw.super_health(h.length); } }
  deliverFrames();
  boardsReadViews();

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
    const T = brt.tel, a = new Float32Array(nav.memory.buffer, nav.nio_ptr(), 41 + 14);
    const home = brt.home || S.p, sp = [setpoint.x - home[0], setpoint.y - home[1], setpoint.z - home[2]];
    const fix = drv.fix, flow = drv.flow;
    const useFix = fix && wiredTo(fix.c) === navB, useFlow = flow && wiredTo(flow.c) === navB;
    const baroNav = T && T.haveAlt;
    a.set([...(T ? T.q : [1, 0, 0, 0]), ...(T ? T.w : [0, 0, 0]), ...(T ? T.acc : [0, 0, 9.81]), T && T.att ? 1 : 0,
      baroNav ? 1 : 0, baroNav ? T.alt : 0, T ? brt.t - T.ts : 0,
      useFix ? 1 : 0, ...(useFix ? fix.p : [0, 0, 0]), ...(useFix ? fix.v : [0, 0, 0]), useFix ? S.t - fix.ts : 0,
      useFlow ? 1 : 0, ...(useFlow ? flow.flow : [0, 0]), useFlow ? flow.range : 0, useFlow ? flow.q : 0, useFlow ? S.t - flow.ts : 0,
      ...sp, ...pilot.vref, setpoint.yaw * D2R, brt.pilot.fly ? 1 : 0, ndt], 0);
    if (tw && useFix && fix.ts !== brt.gpsTs) {                     // the GPS, as the navigation's board reads it: into its telemetry
      brt.gpsTs = fix.ts; const lat = 41 + fix.p[0] / 6371000 * R2D, lon = 29 - fix.p[1] / (6371000 * Math.cos(41 * D2R)) * R2D;
      nav.tlm_gps(lat, lon, 100 + fix.p[2], Math.hypot(fix.v[0], fix.v[1]), (Math.atan2(-fix.v[1], fix.v[0]) * R2D + 360) % 360, 12, brt.t);
    }
    const err = tw ? nav.nav_tick_radio(brt.t) : nav.nav_tick();
    const o = new Float32Array(nav.memory.buffer, nav.nio_ptr(), 41 + 14 + 7).subarray(41);
    brt.navOut = { acc: [o[0], o[1], o[2]], heading: o[3], fly: o[4] > 0.5, p: [o[5], o[6], o[7]], v: [o[8], o[9], o[10]], haveHome: o[11] > 0.5, ready: o[12] > 0.5, landed: o[13] > 0.5, err };
    if (tw) {
      brt.navOut.radio = { target: [o[14], o[15], o[16]], heading: o[17], arm: o[18] > 0.5, link: o[19] > 0.5 };
      if (o[20] > 0.5) { const m = cstr(nav, nav.rc_msg_ptr()); if (m.startsWith('pickup')) cargoLog(`${navB.name}: ${m}`, 'board'); else rnEvent(`${navB.name}: ${m}`, 'warn'); }
      gsSyncTarget();
      const lr = nav.rc_learn_req(); if (lr) { const lname = Object.keys(LN_CMD).find(k => LN_CMD[k] === lr); if (lname) { boardsLearnCmd(lname); rnEvent(`${navB.name}: the radio asked the learning to ${lname === 'calibrate' ? 'calibrate' : lname}`, ''); } }   // (on the Pi, dfb_pi passes it on in-process)
    }
    if (brt.navOut.haveHome && !brt.home) brt.home = S.p.slice();   // where it took off: the nav's home, in the world
    {   // a pickup (pickup_core.c): its request to close the latch goes to the cargo task's board; its target is the one shown
      const pr = nav.pk_req(), cgB = boardOf('cargo');
      if (pr && cgB) sendFrame(navB, cgB, 'cargo', [(pr - 1) >> 2, (pr - 1) & 3]);
      if (!tw && nav.pk_said()) cargoLog(`${navB.name}: ${cstr(nav, nav.pk_msg_ptr())}`, 'board');
      brt.pickup = nav.pk_view(); const v = new Float32Array(nav.memory.buffer, nav.fr_ptr(), 4);
      if (brt.pickup && !tw && brt.home) { setpoint.x = brt.home[0] + v[0]; setpoint.y = brt.home[1] + v[1]; setpoint.z = brt.home[2] + v[2]; }
    }
    brt.navReady = brt.navOut.ready;
    if (S.steps % 400 === 0 || err) brt.navWhy = cstr(nav, nav.nav_why_ptr());
    // the command it sends the flight core (as dfb_pi.c does): arm, fly, the acceleration and heading
    const armNav = brt.navOut.landed ? 0 : tw ? (brt.navOut.radio.arm ? 1 : 0) : brt.pilot.arm;   // the supervisor landed it: it disarms
    if (!err && T) fcSend({ arm: armNav, throttle: brt.navOut.fly ? 1 : 0, guided: 1, acc: brt.navOut.acc, heading: brt.navOut.heading }, same ? 0 : LINK_DELAY);
    else if (!T) fcSend({ arm: 0, throttle: 0, guided: 1, acc: [0, 0, 0], heading: 0 }, same ? 0 : LINK_DELAY);   // announcing itself, disarmed
  }
  // Without navigation: the pilot's sticks, 50 times a second (a radio, or fly.py over USB).
  if (!navB && !tw && brt.t >= brt.nextStick) { brt.nextStick += 0.02; fcSend(stickCommand(), 0.002); }

  // What the flight software believes, for the view and the panels.
  if (brt.navOut && brt.navOut.haveHome) { est.p = add(brt.home, brt.navOut.p); est.v = brt.navOut.v.slice(); est.havePos = true; }
  else { est.p = S.p.slice(); est.v = [0, 0, brt.out.vz]; est.havePos = false; }
}

/* ───────── the radio: the receiver on the telemetry task's board ───────── */
// Each control step: the link's packets (link.js, and its model); what the receiver got goes to its board's UART; what the
// board wrote goes down. The tasks put their telemetry in every 10 ms; boards without the radio send theirs over
// the link every 50 ms; the channels go to the navigation's board 50 times a second, or (no navigation) the
// receiver's board makes the flight core's stick command.
function radioTick(dt, tlmB, tw, coreB, navB, droneOff = false) {
  radio.t = brt.t;
  if (radio.setup && (radio.setup.kind !== radioCfg.kind || (radio.setup.kind2 || '') !== (radioTwo() ? radioCfg2.kind : ''))) boardsRadioCfg();   // (the link changed without saying so: a saved one loaded after the start)
  const g = brt.gnd, two = radioTwo();
  const pk = two || !!radioModel().packets;                           // a packet link (or two links): each end's packet layer (plink.h) in its instance, no modules
  if (g) {                                                           // the command module: its step, every 4 ms
    g.host_tick(dt);
    if (two) radioModuleIn(g, 'gnd');                                // (two links: a module link's frames into the merger)
    if (pk) radioStackOut(g, 'gnd', brt.t);                          // what its packet layer has for it
    else if (radio.toGround.length) {                                // what the transmitter module handed it
      const rb = new Uint8Array(g.memory.buffer, g.rbuf_ptr(), 2048); let n = 0;
      let k = 0; for (; k < radio.toGround.length; k++) { const f = radio.toGround[k]; if (n + f.length > 2048) break; rb.set(f, n); n += f.length; }
      radio.toGround = radio.toGround.slice(k); g.gnd_from_radio(n, brt.t);   // (what didn't fit goes next step)
    }
    if (brt.t >= brt.nextGnd - 1e-9) {
      brt.nextGnd += 0.004;
      const I = groundInputs(brt.t), n = g.gnd_tick(I.held, I.has, I.ax[0], I.ax[1], I.ax[2], I.ax[3], brt.t, 0.004);
      if (n) { const b = Uint8Array.from(new Uint8Array(g.memory.buffer, g.rbuf_ptr(), n)); if (pk) g.plink_stack_in(n, brt.t); radioFromGround(b); }
    }
    if (brt.t >= brt.nextGsRead - 1e-9) { brt.nextGsRead += 0.1; gsRead(); }
  }
  radioStep(dt, brt.t, { gnd: g, drone: droneOff ? null : tw });
  if (droneOff) { radio.toBoard = []; if (radio2.parts.toBoard) radio2.parts.toBoard = []; return; }   // (nothing powers the receiver)
  if (two) radioModuleIn(tw, 'drone');
  if (pk) radioStackOut(tw, 'drone', brt.t);
  else if (radio.toBoard.length) {
    const rb = new Uint8Array(tw.memory.buffer, tw.rbuf_ptr(), 2048); let n = 0;
    let k = 0; for (; k < radio.toBoard.length; k++) { const f = radio.toBoard[k]; if (n + f.length > 2048) break; rb.set(f, n); n += f.length; }
    radio.toBoard = radio.toBoard.slice(k); tw.radio_in(n, brt.t);   // (what didn't fit goes next step)
  }
  if (brt.t >= brt.nextPub - 1e-9) {
    brt.nextPub += 0.01;
    const pack = brt.t >= brt.nextPack - 1e-9; if (pack) brt.nextPack += 0.05;
    for (const b of computers().boards) {
      const w = brt.inst.get(b.id); if (!w || !b.tasks.length) continue;
      w.tlm_publish((b.tasks.includes('core') ? 1 : 0) | (b.tasks.includes('nav') ? 2 : 0) | (b.tasks.includes('learn') ? 4 : 0) | (b.tasks.includes('super') ? 8 : 0) | (b.tasks.includes('cargo') ? 16 : 0), brt.t);
      if (pack && b.id !== tlmB.id) { const n = w.tlm_pack(); if (n) sendFrame(b, tlmB, 'tlm', frOut(w, n)); }
    }
  }
  if (brt.t >= brt.nextRadio - 1e-9) {
    brt.nextRadio += 0.005;
    const n = tw.radio_out(brt.t); if (n) { const b = Uint8Array.from(new Uint8Array(tw.memory.buffer, tw.rbuf_ptr(), n)); if (pk) tw.plink_stack_in(n, brt.t); radioFromDrone(b); }
  }
  if (brt.t >= brt.nextRc - 1e-9) {
    brt.nextRc += 0.02;
    if (navB && navB.id !== tlmB.id) sendFrame(tlmB, navB, 'rc', frOut(tw, tw.rc_pack(brt.t)));
    const cgB = boardOf('cargo');                                    // the latches' board takes the LATCH commands from these too
    if (cgB && cgB.id !== tlmB.id && (!navB || cgB.id !== navB.id)) sendFrame(tlmB, cgB, 'rc', frOut(tw, tw.rc_pack(brt.t)));
    if (!navB && coreB && !tw.radio_stick(brt.t)) {                 // angle mode: the sticks, while the channels come
      const c = new Float32Array(tw.memory.buffer, tw.cmd_ptr(), 5);
      fcSend({ arm: c[0], roll: c[1], pitch: c[2], yaw: c[3], throttle: c[4] }, coreB.id === tlmB.id ? 0 : LINK_DELAY);
    }
  }
}
// The ground station's target and the drone's: an edit of the target (the fields, Hold and Home buttons of the
// simulator) goes up as a "go to"; otherwise the target shown is the one the drone flies to (moved by the sticks).
let gsSet = { x: null, pending: null };
function gsSyncTarget() {
  const r = brt.navOut.radio, home = brt.home || spawnAt || [0, 0, 0];
  const tgt = [home[0] + r.target[0], home[1] + r.target[1], home[2] + r.target[2]], yaw = r.heading * R2D;
  const edited = gsSet.x == null || Math.abs(setpoint.x - gsSet.x) > 1e-6 || Math.abs(setpoint.y - gsSet.y) > 1e-6 || Math.abs(setpoint.z - gsSet.z) > 1e-6 || Math.abs(setpoint.yaw - gsSet.yaw) > 1e-6;
  if (edited) {
    radioCommand(1, [setpoint.x - home[0], setpoint.y - home[1], setpoint.z - home[2], setpoint.yaw * D2R]);
    gsSet.pending = { p: [setpoint.x, setpoint.y, setpoint.z], t: brt.t };
  } else if (gsSet.pending && (nrm(sub(tgt, gsSet.pending.p)) < 0.05 || brt.t - gsSet.pending.t > 3)) gsSet.pending = null;
  if (!gsSet.pending) { setpoint.x = tgt[0]; setpoint.y = tgt[1]; setpoint.z = tgt[2]; setpoint.yaw = ((yaw + 180) % 360 + 360) % 360 - 180; }
  Object.assign(gsSet, { x: setpoint.x, y: setpoint.y, z: setpoint.z, yaw: setpoint.yaw });
}

/* ───────── the pilot, as the simulator plays it ───────── */
// At each reset the drone sits on the ground; the simulator then does what you'd do: arm, and take off to the
// target height. With navigation it asks the navigation to fly (it takes off once its position has settled); without,
// it opens the throttle until the barometer shows the drone climbing past most of the target height.
// A throw start (with a learning task): held in the hand, armed with the motors off, the learning told a throw is
// coming; once the navigation (if any) has its position, the hand throws it and the learning flies it from there.
function autoPilot(dt, navigated) {
  const P = brt.pilot; P.t += dt;
  const att = brt.out && brt.out.attOk, throwing = !!thr;
  if (P.phase === 'ground' && P.t > 0.3 && att) { P.arm = 1; P.phase = 'arming'; P.t = 0; }
  if (P.phase === 'arming' && (P.t > 0.2 || (brt.fcState === 1 && P.t > 0.02))) {   // (on as soon as it's armed)
    if (brt.fcState !== 1) { if (P.t > 2) { P.phase = 'ground'; P.arm = 0; P.t = 0; } return; }   // didn't arm (the reason is shown): switch off and try again
    if (throwing) { P.phase = 'hand'; P.t = 0; P.fly = navigated ? 1 : 0; P.thr = navigated ? 0 : 0.5; pilotLearnCmd('throw'); return; }
    P.phase = 'takeoff'; P.t = 0; P.fly = navigated ? 1 : 0; P.thr = navigated ? 0 : 0.85;
  }
  if (P.phase === 'hand') {
    if (thr && thr.phase === 'hand' && P.t > 0.8 && (!navigated || (brt.navOut && brt.navOut.ready))) releaseThrow();
    if (!thr) P.phase = 'flying';   // caught itself (or crashed)
    return;
  }
  if (P.phase === 'takeoff') {
    if (navigated) { if (brt.navOut && brt.navOut.fly && est.havePos && est.p[2] > (brt.home ? brt.home[2] : 0) + 0.5 * (setpoint.z - (brt.home ? brt.home[2] : 0))) P.phase = 'flying'; return; }
    const h = brt.out && brt.out.haveAlt ? S.p[2] : null;   // (the barometer's height, as the flight core sees it, rises with the true one)
    if (h != null ? brt.out.alt - (P.alt0 ?? (P.alt0 = brt.out.alt)) > 0.6 * setpoint.z : P.t > 0.9) { P.thr = 0.5; P.phase = 'flying'; }
  } else if (P.phase !== 'takeoff') P.alt0 = null;
  if (brt.fcState === 0 && P.phase === 'flying') { P.phase = 'landed'; P.arm = 0; P.fly = 0; }
  // the ESCs cut out on a flat pack and it came down: switch off, as you would
  P.downT = P.phase === 'flying' && hb.lvc && S.p[2] - (terrain.boxes.length ? surfaceBelow(S.p) : 0) < 0.4 && nrm(S.v) < 0.2 ? (P.downT || 0) + dt : 0;
  if (P.downT > 2) { P.phase = 'landed'; P.arm = 0; P.fly = 0; P.thr = 0; P.flat = true; }
}
// The sticks from the pilot keys (angle mode): the arrows lean, A/D turn, W/S climb or sink around the middle.
function stickCommand() {
  const P = brt.pilot, k = c => isHeld(c) ? 1 : 0, s = { gentle: 0.4, normal: 0.7, sport: 1 }[pilot.level] || 0.7;
  const flying = P.phase === 'flying' || P.phase === 'hand';
  return { arm: P.arm, pitch: s * (k('fwd') - k('back')), roll: s * (k('right') - k('left')), yaw: k('yawL') - k('yawR'),
    throttle: P.thr > 0 ? clamp((flying ? 0.5 : P.thr) + 0.35 * (k('up') - k('down')), 0.06, 1) : 0 };
}
const flightPhaseText = () => !brt.ready ? (brt.err || 'starting') : { ground: 'on the ground', arming: 'arming', takeoff: brt.navOut && !brt.navOut.ready ? 'waiting for its position to settle' : 'taking off', flying: 'flying', landed: brt.pilot.flat ? 'down: the battery is flat' : 'landed',
  hand: !thr || thr.phase === 'free' ? 'thrown' : thr.phase === 'toss' ? 'being thrown' : brt.navOut && !brt.navOut.ready ? 'in the hand, waiting for its position' : 'in the hand, motors off' }[brt.pilot.phase] || '';
