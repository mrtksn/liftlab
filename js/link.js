'use strict';
// The pilot's radio link, simulated. Two parts, kept apart:
//
//   WHAT travels — the same over every link: CRSF frames (crsf.js). Up go the command module's channel frames (the
//   newest wins: they're state) and its commands (each one counts); down go the drone's telemetry frames and link
//   statistics. Both ends are real code (runner/ground/ground_core.c on the pilot's side, an instance of its own in
//   boards.js; runner/fc/tlm_core.h, rc_core.h on the drone board that runs the telemetry task). This file holds that
//   part: the plumbing between the two ends, what a frame says, the link log and its statistics, the command module's
//   inputs (you), and the Ground station's copy of what it decoded.
//
//   HOW they travel — the link itself: a model in RADIO_LINKS, chosen by radioCfg.kind. A model is what the two
//   radios and the air between them do: when a packet goes, whether it gets through, how long it takes, what a module
//   queues, drops or reports. ExpressLRS (link-elrs.js); ESP-NOW and Wi-Fi (link-packet.js). A model has:
//     label, receiver                         its name, and what sits on the drone's board
//     settings: [{ key, label, options, show(cfg) }]   its settings in radioCfg, as the Ground station shows them
//     wasm(cfg) → [kind, a, b]                the same for the boards (board_wasm.c radio_link, radio_link.h)
//     room(cfg) → bytes/s                     the telemetry's room on a good link (radio_link.c rlink_budget)
//     roomNote(cfg), signalNote(cfg)          what to say about the room (or the hardware), and about the signal now
//     reset(), step(dt, t, ends)              its state; each step: the packets due, what arrives. ends: { gnd, drone },
//                                             the command module's instance and the receiver board's (null while the
//                                             drone has no power)
//     resume(t)                               (optional) taking over in flight from another link: start from t
//     changed()                               (optional) a setting of its own changed in flight (both ends at once)
//     fromDrone(f, id)                        a frame the drone's board wrote (id: its record, linkNoteDown)
//     command(f)                              a frame from the command module other than the channels
//     connected()                             does the pilot's module count the link connected
//     stats() → { queued, upQueued }          what it holds now, for the statistics; extraStats(t) → more (optional)
//   A module link (ExpressLRS) delivers whole frames with radio.toBoard / radio.toGround. A packet link
//   (packets: true) has no modules: the packet layer (runner/fc/plink.h) runs at both ends, in the two instances,
//   set up by radioLinkSetup with the binding phrase. boards.js passes what each stack writes to its plink
//   (plink_stack_in) as well as to fromDrone/command (for the log); radioStackOut hands what each plink has for its
//   stack (plink_stack_out) to the stack, and each frame to the model's toStack(end, f, t); the model's step moves
//   the packets (plink_air_out at one end, plink_air_in at the other) and adds rebind() (new plinks, same link).
//   Both kinds log with linkDown, linkChannels, linkLog, linkEv.
//
// The command module's inputs are you: the keys and the simulator's pilot (arm, take off) are its buttons and sticks.

const RADIO_LINKS = {};
// kind; ExpressLRS: rate [Hz], ratio, power [mW]; ESP-NOW: channel, lr (long range); Wi-Fi: sta (the drone joins a
// network: 1; makes one: 0), channel; a serial line: baud, half (one way at a time), and the simulator's medium (0 a
// fibre or a wire, tether [m] long; 1 a laser; 2 infrared LEDs; 3 a radio modem); the packet links: bind (the binding
// phrase, the same at both ends); every link: extra (path loss [dB], the simulator's)
const radioCfg = { kind: 'elrs', rate: 250, ratio: 4, power: 100, extra: 0, channel: 1, lr: 0, sta: 0, baud: 115200, half: 0, medium: 1, tether: 50, bind: 'liftlab' };
const RADIO_KINDS = ['elrs', 'espnow', 'wifi', 'serial'];
// A binding phrase as both ends take it (the boards' bind=: 1–31 printable characters, no spaces at the ends;
// runner/esp_radio/radio_cfg.h), or null.
function radioPhraseOk(p) { if (typeof p !== 'string') return null; p = p.trim(); return p && p.length <= 31 && /^[\x20-\x7e]+$/.test(p) ? p : null; }
const radioModel = () => RADIO_LINKS[radioCfg.kind] || RADIO_LINKS.elrs;
const radio = {};
function radioReset() {
  Object.assign(radio, {
    t: 0, seed: 0x2545F491,
    downFrames: 0, fromGround: crsfParser(), fromDrone: crsfParser(),
    txCh: null, txChT: 0, toBoard: [], toGround: [],
    holdUntil: -1, homeUntil: -1,
    log: [], logN: 0, meta: new Map(), ev: {}, stickLogged: null, dropRun: null, lqUp: 0, lqDown: 0, rf: null, rfAt: -1, delivered: null, modeSeen: '',
    stackIn: { gnd: crsfParser(), drone: crsfParser() }, setup: null,
  });
  radioModel().reset();
  gsReset();
}
const radioActive = () => typeof hasTask === 'function' && hasTask('tlm') && brt.ready;
const radioRand = () => { radio.seed ^= radio.seed << 13; radio.seed >>>= 0; radio.seed ^= radio.seed >>> 17; radio.seed ^= radio.seed << 5; radio.seed >>>= 0; return radio.seed / 4294967296; };
const dbm = mw => 10 * Math.log10(mw);

// The path from the handset at the launch point (1.5 m up) to the drone, for any 2.4 GHz link: the distance, the
// buildings in the way (the city worlds), and the loss [dB]: free space with 2 dBi antennas each end, 18 dB a
// building, the "extra loss" setting (distance, walls and interference the simulated world is too small to have),
// and 6 dB more with the drone near the ground.
function linkPath() {
  const gsp = [spawnAt ? spawnAt[0] : 0, spawnAt ? spawnAt[1] : 0, 1.5];
  const d = Math.max(1, nrm(sub(S.p, gsp)));
  let walls = 0;
  if (terrain.boxes && terrain.boxes.length) {                       // buildings in the way: sample the line of sight
    const hit = new Set();
    for (let s = 1; s < 24; s++) {
      const q = add(gsp, scl(sub(S.p, gsp), s / 24));
      terrain.boxes.forEach((b, i) => { if (q[0] > b.lo[0] && q[0] < b.hi[0] && q[1] > b.lo[1] && q[1] < b.hi[1] && q[2] > b.lo[2] && q[2] < b.hi[2]) hit.add(i); });
    }
    walls = Math.min(3, hit.size);
  }
  return { d, walls, loss: 20 * Math.log10(d) + 40.2 + 18 * walls + radioCfg.extra + (S.p[2] < 0.3 ? 6 : 0) };
}

// Each 1 ms step: the link's packets due. Fills radio.toBoard with the bytes the receiver writes to the drone's UART,
// radio.toGround with what the pilot's module hands the command module.
function radioStep(dt, t, ends) { radioModel().step(dt, t, ends); }
// What the drone's board wrote to its receiver: each whole frame gets a record (for its latency and the log), then
// the link takes it.
function radioFromDrone(bytes) { radio.fromDrone.feed(bytes, f => radioModel().fromDrone(f, linkNoteDown(f))); }
function linkNoteDown(f) {
  const id = ++radio.downFrames;
  radio.meta.set(id, { id, t0: radio.t, desc: frameDesc(f), kind: frameKind(f), tries: 0, bytes: f });
  if (radio.meta.size > 600) radio.meta.delete(radio.meta.keys().next().value);   // (only waiting frames have one: never this many)
  linkEv('tlmIn', radio.t);
  return id;
}
// What the command module wrote to its module: its channel frames (the newest is what goes up) and its commands (the
// link takes them).
function radioFromGround(bytes) {
  radio.fromGround.feed(bytes, f => {
    if (f[2] === CRSF.RC) { radio.txCh = crsfRcRead(f.subarray(3, f.length - 1)); radio.txChT = radio.t; linkEv('chMade', radio.t); }
    else if (f[2] === CRSF.EXT) radioModel().command(f);
  });
}
const radioConnected = () => radioModel().connected();
// A command for the command module to send (go to, calibrate…). 0, or −1 if it has too many waiting (said in the
// log, not silently lost).
function radioCommand(cmd, values) {
  const g = brt.gnd; if (!g) return -1;
  let r; if (cmd === 1) r = g.gnd_goto(...values); else { frIn(g, values); r = g.gnd_command(cmd, values.length); }
  if (r) linkLog('↑', 'cmd', `${cmd === 1 ? 'GOTO' : cmd === 3 ? 'LATCH' : 'command ' + cmd} not sent`, r === -2 ? 'a value out of range' : 'the command module has too many waiting', 'bad');
  return r ? -1 : 0;
}
function radioHold() { radio.holdUntil = radio.t + 0.3; }
function radioHome() { radio.homeUntil = radio.t + 0.3; }
// The link's settings changed: the boards take the new ones (board_wasm.c radio_link: the telemetry budget follows,
// in flight too; nothing else resets). Another link, or another binding phrase: both ends switch at once (the
// packet layers set up again); the drone sees a short gap in its link.
function boardsRadioCfg() {
  if (!brt.ready) return;                                            // (starting: the boards take radioCfg as they start)
  const M = radioModel(), a = M.wasm(radioCfg), S = radio.setup || {};
  for (const b of computers().boards) { const w = brt.inst.get(b.id); if (w) w.radio_link(...a); }
  if (S.kind !== radioCfg.kind) {
    radio.toBoard = []; radio.toGround = []; radio.meta.clear();
    M.reset(); if (M.resume) M.resume(radio.t);
    radioLinkSetup();
    linkLog('↕', 'link', `now ${M.label}`, 'both ends switched', 'warn');
  } else if (M.packets && S.bind !== radioCfg.bind) {
    M.rebind(); radioLinkSetup();
    linkLog('↕', 'link', 'a new binding phrase', 'both ends set up again', 'warn');
  } else if (M.changed) M.changed();
}
// The two ends of a packet link: the command module's instance, the receiver board's.
function radioEnds() { const b = typeof boardOf === 'function' ? boardOf('tlm') : null; return { gnd: brt.gnd, drone: b ? brt.inst.get(b.id) || null : null }; }
// As the boards start (boardsStart), and when the link changes: a packet link's two ends set up, each with the binding
// phrase and a session number of its own (the simulator's random numbers: the same each run).
function radioLinkSetup() {
  radio.setup = { kind: radioCfg.kind, bind: radioCfg.bind };
  if (!radioModel().packets) return;
  const E = radioEnds();
  if (E.gnd) plinkSetup(E.gnd, 0, radioCfg.bind);
  if (E.drone) plinkSetup(E.drone, 1, radioCfg.bind);
}
function plinkSetup(w, role, phrase) {
  const b = new TextEncoder().encode(String(phrase)).slice(0, 63);
  new Uint8Array(w.memory.buffer, w.rbuf_ptr(), b.length).set(b);
  w.plink_setup(role, b.length, 1 + Math.floor(radioRand() * 0x7FFFFFFE), ...radioModel().wasm(radioCfg));   // (the link: a serial line's packet sizes follow its speed)
}
// One end given another phrase (as if its program had been installed with it): to try a mismatch. end 'gnd' or 'drone'.
function radioBindEnd(end, phrase) {
  const w = radioEnds()[end]; if (!w || !radioModel().packets) return false;
  plinkSetup(w, end === 'drone' ? 1 : 0, phrase); radioModel().rebind();
  linkLog('↕', 'link', `${end === 'drone' ? 'the drone' : 'the command module'}: binding phrase "${phrase}"`, 'that end set up again', 'warn');
  return true;
}
// A packet link: what one end's packet layer has for its stack (the frames that came, its link statistics) goes to
// the stack, as a module's UART would bring it; the model sees each frame (for the log).
function radioStackOut(w, end, t) {
  const n = w.plink_stack_out(t); if (!n) return;
  const b = Uint8Array.from(new Uint8Array(w.memory.buffer, w.rbuf_ptr(), n));
  if (end === 'gnd') w.gnd_from_radio(n, t); else w.radio_in(n, t);
  radio.stackIn[end].feed(b, f => radioModel().toStack(end, f, t));
}

// The command module's inputs (ground_core.h): the buttons held (GB bits) and the analog sticks. With navigation the
// keys are its stick buttons (stickInput eases them in); in angle mode the simulator's pilot moves the sticks
// (it opens the throttle for the take-off, then centres it). The arm and fly switches are the simulator's pilot's.
const GB = { right: 0, left: 1, fwd: 2, back: 3, up: 4, down: 5, yawR: 6, yawL: 7, arm: 8, fly: 9, hold: 10, home: 11, gentle: 12, normal: 13, sport: 14, cal: 15 };
function groundInputs(t) {
  const P = brt.pilot, bit = k => 1 << GB[k];
  let held = 0, has = 0, ax = [0, 0, 0, 0];
  if (hasTask('nav')) { for (const k of ['right', 'left', 'fwd', 'back', 'up', 'down', 'yawR', 'yawL']) if (isHeld(k)) held |= bit(k); }
  else { const s = stickCommand(); has = 15; ax = [s.roll, s.pitch, 2 * s.throttle - 1, -s.yaw]; }
  if (P.arm) held |= bit('arm'); if (P.fly) held |= bit('fly');
  if (t < radio.holdUntil) held |= bit('hold'); if (t < radio.homeUntil) held |= bit('home');
  held |= bit(pilot.level === 'gentle' ? 'gentle' : pilot.level === 'sport' ? 'sport' : 'normal');
  return { held, has, ax };
}

/* ───────── the link log: what passes through the simulated radio (only a simulator can watch this) ───────── */
let radioLogAll = false;   // every frame (busy), or only commands, switches, messages, the flight mode and link events
function linkLog(dir, kind, data, meta, tone, bytes) {
  radio.log.unshift({ id: ++radio.logN, t: radio.t, dir, kind, data, meta: meta || '', tone: tone || '', bytes: bytes ? Uint8Array.from(bytes) : null });
  if (radio.log.length > 300) radio.log.length = 300;
}
// What happened in the last 5 s, for the statistics: [time, value] per kind.
function linkEv(kind, t, v) {
  const a = radio.ev[kind] || (radio.ev[kind] = []); a.push([t, v ?? 0]);
  while (a.length && a[0][0] < t - 5) a.shift();
}
function linkStats(t) {
  const W = 5, get = k => (radio.ev[k] || []).filter(e => e[0] >= t - W), n = k => get(k).length;
  const lat = k => { const a = get(k).map(e => e[1]); return a.length ? { avg: a.reduce((s, x) => s + x, 0) / a.length, max: Math.max(...a) } : null; };
  const span = Math.min(W, Math.max(0.5, t)), pct = (a, b) => a + b ? 100 * a / (a + b) : 0, held = radioModel().stats();
  return {
    chMade: n('chMade') / span, chSent: n('chSent') / span, chLat: lat('chSent'), upLostPct: pct(n('upLost'), n('upOk')),
    cmds: n('cmdOut'), cmdLat: lat('cmdOut'),
    tlmIn: n('tlmIn') / span, tlmOut: n('tlmOut') / span, tlmLat: lat('tlmOut'), tlmDrop: n('tlmDrop'), tlmSuper: n('tlmSuper') / span, downLostPct: pct(n('downLost'), n('downOk')),
    queued: held.queued, upQueued: held.upQueued, cmdDrop: n('cmdDrop'),
    ...(radioModel().extraStats ? radioModel().extraStats(t) : {}),
  };
}
// A frame's bytes, field by field (the raw view of a log line).
const CRSF_ADDR_NAME = { 0xC8: 'flight controller', 0xEA: 'handset / command module', 0xEC: 'receiver', 0xEE: 'transmitter module' };
const CRSF_TYPE_NAME = { 0x02: 'GPS', 0x07: 'vario', 0x08: 'battery', 0x09: 'barometric altitude', 0x14: 'link statistics', 0x16: 'RC channels', 0x1E: 'attitude', 0x21: 'flight mode', 0x80: 'extended' };
function frameFields(f) {
  const p = f.subarray(3, f.length - 1), out = [];
  out.push(['address', `0x${f[0].toString(16).toUpperCase()} (${CRSF_ADDR_NAME[f[0]] || '?'})`], ['length', `${f[1]} (type, ${f[1] - 2} payload bytes, CRC)`],
    ['type', `0x${f[2].toString(16).toUpperCase()} (${CRSF_TYPE_NAME[f[2]] || '?'})`], ['CRC-8', `0x${f[f.length - 1].toString(16).toUpperCase()} ${crsfCrc8(f, 2, f.length - 1) === f[f.length - 1] ? '(good)' : '(BAD)'}`]);
  switch (f[2]) {
    case CRSF.RC: {
      let bit = 0; const raw = [];
      for (let i = 0; i < 16; i++) { let v = 0; for (let b = 0; b < 11; b++, bit++) if (p[bit >> 3] & (1 << (bit & 7))) v |= 1 << b; raw.push(v); }
      const names = ['roll', 'pitch', 'throttle', 'yaw', 'arm', 'speed', 'fly', 'hold', 'home'];
      const UNUSED = 173;   // −1 as both encoders write it: round(992 − 819)
      raw.forEach((v, i) => { if (i < 9 || v !== UNUSED) out.push([`ch${i + 1}${names[i] ? ' ' + names[i] : ''}`, `${v} → ${sgn((v - 992) / 819)}`]); });
      if (raw.slice(9).every(v => v === UNUSED)) out.push(['ch10–16', `${UNUSED} (unused: −1)`]);
      break;
    }
    case CRSF.ATTITUDE: out.push(['pitch', `${be16s(p, 0)} → ${(be16s(p, 0) / 1e4 * R2D).toFixed(1)}°`], ['roll', `${be16s(p, 2)} → ${(be16s(p, 2) / 1e4 * R2D).toFixed(1)}°`], ['yaw', `${be16s(p, 4)} → ${(be16s(p, 4) / 1e4 * R2D).toFixed(1)}°`]); break;
    case CRSF.BATTERY: out.push(['voltage', `${(p[0] << 8) | p[1]} → ${(((p[0] << 8) | p[1]) / 10).toFixed(1)} V`], ['current', `${(p[2] << 8) | p[3]} → ${(((p[2] << 8) | p[3]) / 10).toFixed(1)} A`],
      ['used', `${(p[4] << 16) | (p[5] << 8) | p[6]} mAh`], ['remaining', `${p[7]}%`]); break;
    case CRSF.GPS: { const la = (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3], lo = (p[4] << 24) | (p[5] << 16) | (p[6] << 8) | p[7];
      out.push(['latitude', `${la} → ${(la * 1e-7).toFixed(7)}°`], ['longitude', `${lo} → ${(lo * 1e-7).toFixed(7)}°`], ['speed', `${(p[8] << 8) | p[9]} → ${(((p[8] << 8) | p[9]) / 36).toFixed(1)} m/s`],
        ['course', `${(((p[10] << 8) | p[11]) / 100).toFixed(1)}°`], ['altitude', `${((p[12] << 8) | p[13]) - 1000} m`], ['satellites', `${p[14]}`]); break; }
    case CRSF.BARO_ALT: { const a = (p[0] << 8) | p[1]; out.push(['altitude', `${a} → ${(a & 0x8000 ? a & 0x7FFF : (a - 10000) / 10).toFixed(1)} m`]); if (p.length >= 4) out.push(['climb', `${be16s(p, 2)} → ${(be16s(p, 2) / 100).toFixed(2)} m/s`]); break; }
    case CRSF.FLIGHT_MODE: out.push(['mode', `"${cstrBytes(p, 0)}"`]); break;
    case CRSF.EXT:
      if (p[0] === CRSF.EXT_TEXT) out.push(['subtype', '0xF1 (status text)'], ['severity', `${p[1]} (${['emergency', 'alert', 'critical', 'error', 'warning', 'notice', 'info', 'debug'][p[1]] || '?'})`], ['text', `"${cstrBytes(p, 2)}"`]);
      else if (p[0] === CRSF.EXT_ITEM) { out.push(['subtype', '0xD0 (telemetry item)'], ['item', `${p[1]} (${(TLM_ITEMS[p[1]] || { key: '?' }).key})`], ['values', `${p[2]}`]);
        const d = TLM_ITEMS[p[1]]; for (let k = 0; k < p[2]; k++) out.push([d && d.fields ? d.fields[k] || `v${k}` : d && d.list ? (k ? `#${k}` : 'count') : `v${k}`, `${be16s(p, 3 + 2 * k)} (int16, × the item's scale)`]); }
      else if (p[0] === CRSF.EXT_CMD) { out.push(['subtype', '0xD1 (ground-station command)'], ['command', `${p[1]} (${{ 1: 'GOTO', 2: 'LEARN' }[p[1]] || '?'})`], ['sequence', `${p[2]}`]);
        for (let i = 3, k = 0; i + 1 < p.length; i += 2, k++) { const v = be16s(p, i); out.push([p[1] === 1 ? ['x', 'y', 'z', 'heading'][k] || `value ${k + 1}` : p[1] === 2 ? 'code' : `value ${k + 1}`, p[1] === 1 ? `${v} → ${k < 3 ? (v / 100).toFixed(2) + ' m' : (v / 1000 * R2D).toFixed(1) + '°'}` : p[1] === 2 ? `${v} (${Object.keys(LN_CMD).find(n => LN_CMD[n] === v) || '?'})` : `${v}`]); } }
      break;
  }
  return out;
}
const be16s = (p, i) => { const v = (p[i] << 8) | p[i + 1]; return v & 0x8000 ? v - 0x10000 : v; };
const cstrBytes = (p, i) => { let n = i; while (n < p.length && p[n]) n++; return new TextDecoder().decode(p.subarray(i, n)); };
const sgn = v => (v >= 0 ? '+' : '−') + Math.abs(v).toFixed(2);
function cmdDesc(f) {   // a command frame as data (the command module's scaling: rc_core.h rc_cmd_scale)
  const p = f.subarray(3, f.length - 1);
  if (p[0] !== CRSF.EXT_CMD) return `EXT 0x${p[0].toString(16)}`;
  const cmd = p[1], seq = p[2], v = []; for (let i = 3; i + 1 < p.length; i += 2) v.push(be16s(p, i));
  if (cmd === 1) return `GOTO x ${(v[0] / 100).toFixed(1)} y ${(v[1] / 100).toFixed(1)} z ${(v[2] / 100).toFixed(1)} hdg ${Math.round(v[3] / 1000 * R2D)}° #${seq}`;
  if (cmd === 2) return `LEARN ${Object.keys(LN_CMD).find(k => LN_CMD[k] === v[0]) || v[0]} #${seq}`;
  if (cmd === 3) return `LATCH ${v[0] < 0 ? 'all' : v[0] + 1} ${['open', 'close', 'toggle'][v[1]] || v[1]} #${seq}`;
  return `CMD ${cmd} ${v.join(' ')} #${seq}`;
}
function frameKind(f) { const p = f.subarray(3, f.length - 1); return f[2] === CRSF.FLIGHT_MODE ? 'mode' : f[2] === CRSF.EXT && p[0] === CRSF.EXT_TEXT ? 'msg' : 'frame'; }
function frameDesc(f) {   // a telemetry frame as data
  const p = f.subarray(3, f.length - 1);
  switch (f[2]) {
    case CRSF.ATTITUDE: return `ATT r ${(be16s(p, 2) / 1e4 * R2D).toFixed(0)}° p ${(be16s(p, 0) / 1e4 * R2D).toFixed(0)}° y ${(be16s(p, 4) / 1e4 * R2D).toFixed(0)}°`;
    case CRSF.BATTERY: return `BATT ${(((p[0] << 8) | p[1]) / 10).toFixed(1)} V ${(((p[2] << 8) | p[3]) / 10).toFixed(1)} A ${p[7]}%`;
    case CRSF.GPS: return `GPS ${p[14]} sats`;
    case CRSF.BARO_ALT: { const a = (p[0] << 8) | p[1]; return `ALT ${(a & 0x8000 ? a & 0x7FFF : (a - 10000) / 10).toFixed(1)} m`; }
    case CRSF.FLIGHT_MODE: return cstrBytes(p, 0);
    case CRSF.EXT:
      if (p[0] === CRSF.EXT_TEXT) return `"${cstrBytes(p, 2)}"`;
      if (p[0] === CRSF.EXT_ITEM) return `${((TLM_ITEMS[p[1]] || { key: 'item' + p[1] }).key).toUpperCase()} (${p[2]} values)`;
      return 'EXT';
  }
  return `0x${f[2].toString(16)}`;
}
// A whole frame out of the pilot's module, to the command module (id: the record of the frame the drone wrote).
// chunk: the link's piece size, to tell what was sent again (its pieces beyond the frame's own).
function linkDown(f, t, id, chunk = 5) {
  const m = radio.meta.get(id); if (!m) return;
  radio.meta.delete(id);
  const lat = `${Math.round((t - m.t0) * 1000)} ms`, resent = m.tries - Math.ceil(f.length / chunk), meta = resent > 0 ? `${lat} · ${resent} resent` : lat;
  if (m.kind === 'mode') { if (m.desc === radio.modeSeen && !radioLogAll) { linkEv('tlmOut', t, (t - m.t0) * 1000); return; } radio.modeSeen = m.desc; }
  linkEv('tlmOut', t, t - m.t0 > 0 ? (t - m.t0) * 1000 : 0);
  if (m.kind === 'msg' || m.kind === 'mode' || radioLogAll) linkLog('↓', m.kind, m.desc, meta, '', f);
}
// The channels as the drone's receiver passes them on: the sticks when they move (at most 10 times a second, and
// when they come back to the centre), the switches when they change.
const STICKS = ['roll', 'pitch', 'thr', 'yaw'];
function linkChannels(ch, t, fr, lat) {
  const d = radio.delivered; radio.delivered = ch.slice();
  if (!d) return;
  const L = radio.stickLogged || (radio.stickLogged = { v: [0, 0, 0, 0], t: -1 });
  const moved = ch.slice(0, 4).some((v, i) => Math.abs(v - L.v[i]) > 0.15), centred = ch.slice(0, 4).every(v => Math.abs(v) < 0.02), wasCentred = L.v.every(v => Math.abs(v) < 0.02);
  if ((moved && t - L.t >= 0.1) || (centred && !wasCentred)) {
    L.v = ch.slice(0, 4); L.t = t;
    const parts = STICKS.map((n, i) => Math.abs(ch[i]) >= 0.02 ? `${n} ${sgn(ch[i])}` : '').filter(Boolean);
    linkLog('↑', 'stick', parts.length ? parts.join('  ') : 'centred', `${Math.round(lat)} ms`, '', fr);
  }
  for (const [i, name] of [[4, 'ARM'], [5, 'SPEED'], [6, 'FLY'], [7, 'HOLD'], [8, 'HOME']]) {
    const a = Math.round(d[i] * 2) / 2, b = Math.round(ch[i] * 2) / 2; if (a === b) continue;
    linkLog('↑', 'switch', `${name} ${i === 5 ? ['gentle', 'normal', 'sport'][Math.round(b) + 1] : b > 0 ? 'on' : 'off'}`, `CH${i + 1} · ${Math.round(lat)} ms`, '', fr);
  }
}

/* ───────── the ground station: what the command module decoded ───────── */
// gs is the Ground station tab's copy of the command module's view (ground_core.c gnd_view_pack), read a few times a
// second: values by kind with when each came (gs.at, simulator time), the messages, the alert, the channels it sent.
// The battery voltage of each battery frame (vHist, vN of them so far) and the track of each position frame are kept
// here, on the data path, whether the tab is open or not.
const gs = { v: {}, at: {}, log: [], frames: 0, bytes: 0, trackXY: [], link: null, alert: null, sent: null, rate: [], vHist: [], vN: 0 };
function gsReset() {
  Object.assign(gs, { v: {}, at: {}, log: [], frames: 0, bytes: 0, lastAge: -1, trackXY: [], link: null, rate: [], alert: null, sent: null, nmsg: 0, vHist: [], vN: 0 });
  if (typeof GS_UI !== 'undefined') Object.assign(GS_UI, { built: false, paused: null, clearId: 0, logN: -1 });   // (the widgets hold the old run's values: built again; the log starts again)
  if (typeof GS_UI !== 'undefined') GS_UI.open.clear();
}
function gsRead() {
  const g = brt.gnd; if (!g) return;
  // when each value came: now − its age, to the millisecond (the age is a float32 from the C, so the same frame's time
  // would come out a little different at each read; frames arrive on whole 1 ms steps)
  const t = brt.t, n = g.gnd_view(t), o = new Float32Array(g.memory.buffer, g.fr_ptr(), n), at = a => a >= 0 ? Math.round((t - a) * 1000) / 1000 : null;
  const put = (kind, age, v) => { const a = at(age); if (a == null) return; gs.v[kind] = v; gs.at[kind] = a; };
  const battAt = gs.at.battery, posAt = gs.at.pos;
  gs.alert = { level: o[0], why: o[1], text: cstr(g, g.gnd_why_text(o[1]), 80) };
  if (gs.alert.level && !gs.alert.text) gs.alert.text = `alert ${o[1]}`;   // (a reason this build has no words for)
  gs.bytes = o[3]; gs.frames = o[4]; gs.lastAge = o[7];
  if (o[8] >= 0) gs.link = { upRssi: o[9], upLq: o[10], upSnr: o[11], downRssi: o[12], downLq: o[13], downSnr: o[14], power: o[15], t: at(o[8]) };
  put('attitude', o[16], { roll: o[17], pitch: o[18], yaw: o[19] });
  put('battery', o[20], { volts: o[21], amps: o[22], mah: o[23], pct: o[24] });
  put('gps', o[25], { lat: o[26] + o[27], lon: o[28] + o[29], speed: o[30], course: o[31], alt: o[32], sats: o[33] });
  put('baro', o[34], { alt: o[35], vz: o[36] });
  put('mode', o[37], { mode: cstr(g, g.gnd_mode_ptr(), 16) });
  gs.sent = Array.from(o.subarray(38, 54)); gs.shaped = o[54] > 0.5;
  for (let k = 55; k < n && o[k];) {
    const id = o[k], age = o[k + 1], m = o[k + 2], vals = Array.from(o.subarray(k + 3, k + 3 + m)); k += 3 + m;
    const d = TLM_ITEMS[id]; if (!d) continue;
    if (d.list) put(d.key, age, { n: vals[0], values: vals.slice(1) });
    else { const v = {}; d.fields.forEach((f, i) => { v[f] = vals[i]; }); put(d.key, age, v); }
  }
  if (gs.v.pos && gs.at.pos !== posAt) { gs.trackXY.push([gs.v.pos.x, gs.v.pos.y]); if (gs.trackXY.length > 400) gs.trackXY.shift(); }   // a new position frame: a point
  if (gs.v.battery && gs.at.battery !== battAt) { gs.vHist.push(gs.v.battery.volts); if (gs.vHist.length > 120) gs.vHist.shift(); gs.vN++; }   // a new battery frame
  const nm = o[6];
  for (let i = Math.max(gs.nmsg, nm - 16); i < nm; i++) { gs.log.unshift({ t: g.gnd_msg_t(i), sev: g.gnd_msg_sev(i), text: cstr(g, g.gnd_msg_text(i), 60) }); if (gs.log.length > 60) gs.log.length = 60; }
  gs.nmsg = nm;
}
