'use strict';
// The pilot's radio link, simulated: an ExpressLRS transmitter module wired by CRSF to the command module (the
// pilot's side: runner/ground/ground_core.c, an instance of its own, boards.js), and a receiver wired by CRSF to the
// drone board that runs the telemetry task (runner/fc/tlm_core.h, rc_core.h). Both ends are real code; this is only
// what the two modules and the air between them do. The drone gets what a real receiver gives: channel frames when
// uplink packets get through, link statistics ten times a second, the command module's commands; what the drone
// writes to its receiver goes down in the telemetry slots and comes out of the transmitter module to the command
// module, with the module's own link statistics.
//
// The air link, as ExpressLRS does it:
//   - a fixed packet rate (50, 150, 250 or 500 Hz); one packet in `ratio` goes down (telemetry), the rest go up
//     (the 16 channels, and now and then 5 bytes of a ground-station command instead);
//   - each telemetry packet carries 5 bytes of the receiver's queued CRSF frames; a "stubborn sender" repeats a lost
//     chunk until it gets through, so frames arrive whole and in order, just later on a bad link. The receiver
//     queues at most 512 bytes; past that it drops the oldest frames;
//   - each packet gets through with a probability set by the margin over the receiver's sensitivity for its rate
//     (LoRa at 2.4 GHz: −105 dBm at 500 Hz to −115 dBm at 50 Hz). The signal: transmit power and 2 dBi antennas, free-
//     space loss to the drone, 18 dB for each building in the way (the city worlds), and the "extra loss" setting,
//     which stands in for distance, walls and interference the simulated world is too small to have.
// The command module's inputs are you: the keys and the simulator's pilot (arm, take off) are its buttons and sticks.

const ELRS_RATES = { 50: -115, 150: -112, 250: -108, 500: -105 };   // receiver sensitivity [dBm]
const ELRS_RATIOS = [2, 4, 8, 16, 32, 64, 128];
const ELRS_POWERS = [10, 25, 100, 250, 500, 1000];   // [mW]
const radioCfg = { rate: 250, ratio: 4, power: 100, extra: 0 };
const radio = {};
function radioReset() {
  Object.assign(radio, {
    t: 0, nextPkt: 0, k: 0, seed: 0x2545F491,
    down: [], downBytes: 0, downFrames: 0, downDropped: 0, fromGround: crsfParser(), fromDrone: crsfParser(), txIn: crsfParser(),
    up: [], txCh: null, toBoard: [], toGround: [], nextStats: 0, ch: null,
    upHist: [], downHist: [], rssiUp: -50, rssiDown: -50, snrUp: 10, snrDown: 10, rfAt: -1,
    downChunks: 0, downGot: 0, upGot: 0, rxLost: false, holdUntil: -1, homeUntil: -1,
    log: [], logN: 0, downMeta: [], inFlight: -1, delivered: null, lastUpOk: 0, upGap: false, lastDownOk: 0, downGap: false, lqLow: false, modeSeen: '',
  });
  gsReset();
}
const radioActive = () => typeof hasTask === 'function' && hasTask('tlm') && brt.ready;
const radioRand = () => { radio.seed ^= radio.seed << 13; radio.seed >>>= 0; radio.seed ^= radio.seed >>> 17; radio.seed ^= radio.seed << 5; radio.seed >>>= 0; return radio.seed / 4294967296; };
const dbm = mw => 10 * Math.log10(mw);

// The signal at the drone, from the handset at the launch point (1.5 m up).
function radioRf() {
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
  const loss = 20 * Math.log10(d) + 40.2 + 18 * walls + radioCfg.extra + (S.p[2] < 0.3 ? 6 : 0);
  const rssi = dbm(radioCfg.power) + 4 - loss, sens = ELRS_RATES[radioCfg.rate] || -108;
  return { d, walls, rssi, margin: rssi - sens, snr: clamp(rssi - sens - 2, -18, 13) };
}
const radioP = margin => 1 / (1 + Math.exp(-(margin - 3) / 1.8));   // a packet gets through

// Each 1 ms step: the packets due. Fills radio.toBoard with the bytes the receiver writes to the drone's UART.
function radioStep(dt, t) {
  if (radio.rfAt < 0 || t - radio.rfAt > 0.02) { const r = radioRf(); radio.rf = r; radio.rfAt = t; }
  const r = radio.rf, pUp = radioP(r.margin), pDown = radioP(r.margin - 1);   // (the receiver's antenna is a little worse)
  const period = 1 / radioCfg.rate;
  while (radio.nextPkt <= t + 1e-9) {
    radio.nextPkt += period; radio.k++;
    const tlmSlot = radioCfg.ratio > 0 && radio.k % radioCfg.ratio === 0;
    if (tlmSlot) {
      const ok = radioRand() < pDown;
      radio.downHist.push(ok ? 1 : 0); if (radio.downHist.length > 100) radio.downHist.shift();
      if (radio.down.length) {
        radio.downChunks++; if (radio.down[0].frame === radio.downMeta[0]?.id) radio.downMeta[0].tries++;
        if (ok) {
          const c = radio.down.shift(); radio.downBytes -= c.length; radio.downGot++; radio.inFlight = radio.down.length && radio.down[0].frame === c.frame ? c.frame : -1;
          radio.txIn.feed(c, f => { radio.toGround.push(f); linkDown(f, t); });   // (the module hands on whole frames)
        }
      }
      if (ok) { if (radio.downGap) linkLog('↓', 'link', `telemetry packets get through again after ${(t - radio.lastDownOk).toFixed(1)} s`, 'good'); radio.lastDownOk = t; radio.downGap = false; }
      else if (!radio.downGap && t - radio.lastDownOk > 1) { radio.downGap = true; linkLog('↓', 'link', 'no telemetry packet has got through for 1 s', 'bad'); }
    } else {
      const ok = radioRand() < pUp;
      radio.upHist.push(ok ? 1 : 0); if (radio.upHist.length > 100) radio.upHist.shift();
      const cmdSlot = radio.up.length && radio.k % 2;
      if (!ok) {
        if (cmdSlot) radio.up[0].lost++;
        if (!radio.upGap && t - radio.lastUpOk > 0.5) { radio.upGap = true; linkLog('↑', 'link', 'no uplink packet has got through for 0.5 s: the drone gets no channels', 'bad'); }
        continue;
      }
      radio.upGot++;
      if (radio.upGap) linkLog('↑', 'link', `uplink packets get through again after ${(t - radio.lastUpOk).toFixed(1)} s`, 'good');
      radio.upGap = false; radio.lastUpOk = t;
      if (cmdSlot) {                                                 // this packet carries 5 bytes of a command
        const c = radio.up[0]; c.got.push(...c.bytes.subarray(c.at, c.at + 5)); c.at += 5; c.pk++;
        if (c.at >= c.bytes.length) {
          radio.toBoard.push(Uint8Array.from(c.got)); radio.up.shift();
          linkLog('↑', 'cmd', `${c.desc}: reached the drone's receiver after ${Math.round((t - c.t0) * 1000)} ms (${c.pk} packets${c.lost ? `, ${c.lost} lost and sent again` : ''})`, 'good');
        }
      } else if (radio.txCh) { radio.toBoard.push(crsfRcFrame(radio.txCh)); linkSwitches(radio.txCh, t); }   // (nothing from the command module yet: nothing to send)
      if (radioLogAll && !cmdSlot && radio.txCh && (radio.k % 50 === 1)) linkLog('↑', 'frame', `channels ${radio.txCh.slice(0, 4).map(v => v.toFixed(2)).join(' ')} … (one in 50 shown)`, '');
    }
  }
  const D = radio.dropRun;
  if (D && D.n && t - D.last > 0.5) { linkLog('↓', 'drop', `${D.n} frames dropped in ${(D.last - D.t0).toFixed(1)} s (${[...D.kinds].slice(0, 6).join(', ')}${D.kinds.size > 6 ? '…' : ''}); the newest values go down again`, 'warn'); radio.dropRun = null; }
  if (radio.lqUp < 50 !== radio.lqLow && radio.upHist.length >= 50) { radio.lqLow = radio.lqUp < 50; linkLog('↑', 'link', radio.lqLow ? `uplink quality down to ${Math.round(radio.lqUp)}%` : `uplink quality back up to ${Math.round(radio.lqUp)}%`, radio.lqLow ? 'warn' : 'good'); }
  const lq = h => h.length ? 100 * h.reduce((a, b) => a + b, 0) / h.length : 0;
  radio.lqUp = lq(radio.upHist); radio.lqDown = lq(radio.downHist);
  radio.rssiUp = r.rssi + (radioRand() - 0.5) * 2; radio.rssiDown = r.rssi - 1 + (radioRand() - 0.5) * 2;
  if (t >= radio.nextStats) {                                        // link statistics to the drone, 10 times a second
    radio.nextStats = t + 0.1;
    if (radio.lqUp > 0) radio.toBoard.push(crsfLinkStatsFrame({ upRssi: radio.rssiUp, upLq: radio.lqUp, upSnr: r.snr, downRssi: radio.rssiDown, downLq: radio.lqDown, downSnr: r.snr - 1, rfMode: [50, 150, 250, 500].indexOf(radioCfg.rate), power: radioCfg.power }));
    const ls = crsfLinkStatsFrame({ upRssi: radio.rssiUp, upLq: radio.lqUp, upSnr: r.snr, downRssi: radio.rssiDown, downLq: radio.lqDown, downSnr: r.snr - 1, rfMode: [50, 150, 250, 500].indexOf(radioCfg.rate), power: radioCfg.power });
    ls[0] = CRSF.ADDR_HANDSET; radio.toGround.push(ls);           // the transmitter module tells the command module too
  }
}
// What the drone's board wrote to the receiver: queued as 5-byte chunks for the telemetry slots.
function radioFromDrone(bytes) {
  radio.fromDrone.feed(bytes, f => {
    while (radio.downBytes + f.length > 512 && radio.down.length) {   // the receiver's queue is full: the oldest frame not being sent goes
      const first = radio.down.find(c => c.frame !== radio.inFlight); if (!first) break;
      const id = first.frame; let n = 0; radio.down = radio.down.filter(c => { if (c.frame !== id) return true; n += c.length; return false; });
      radio.downBytes -= n; radio.downDropped++;
      const k = radio.downMeta.findIndex(m => m.id === id), m = k >= 0 ? radio.downMeta.splice(k, 1)[0] : null;
      if (m && m.kind === 'msg') linkLog('↓', 'drop', `${m.desc}: dropped by the drone's receiver, its queue full`, 'bad');
      else if (m) {                                                  // the rest: one line when it starts, one when it ends
        const D = radio.dropRun || (radio.dropRun = { n: 0, t0: radio.t, kinds: new Set() });
        if (!D.n) linkLog('↓', 'drop', 'the drone\'s receiver is dropping frames: its queue is full (512 bytes waiting to go down)', 'warn');
        D.n++; D.last = radio.t; D.kinds.add(m.desc.replace(/ \(.*\)$/, ''));
      }
    }
    radio.downFrames++;
    radio.downMeta.push({ id: radio.downFrames, t0: radio.t, desc: frameDesc(f), kind: frameKind(f), tries: 0 });
    for (let i = 0; i < f.length; i += 5) { const c = f.subarray(i, i + 5); c.frame = radio.downFrames; radio.down.push(c); radio.downBytes += c.length; }
  });
}
// What the command module wrote to the transmitter module: its channel frames (the latest goes up in each uplink
// packet) and its commands (queued, and sent 5 bytes at a time in place of some channel packets).
function radioFromGround(bytes) {
  radio.fromGround.feed(bytes, f => {
    if (f[2] === CRSF.RC) radio.txCh = crsfRcRead(f.subarray(3, f.length - 1));
    else if (f[2] === CRSF.EXT) {
      const desc = cmdDesc(f); radio.up.push({ bytes: f, at: 0, got: [], t0: radio.t, pk: 0, lost: 0, desc });
      linkLog('↑', 'cmd', `${desc}: from the command module to the transmitter module (${f.length} bytes, ${Math.ceil(f.length / 5)} packets)`, '');
    }
  });
}
// A ground-station command (go to, calibrate…): the command module queues it and sends it up.
function radioCommand(cmd, values) { const g = brt.gnd; if (!g) return; if (cmd === 1) g.gnd_goto(...values); else { frIn(g, values); g.gnd_command(cmd, values.length); } }
function radioHold() { radio.holdUntil = radio.t + 0.3; }
function radioHome() { radio.homeUntil = radio.t + 0.3; }

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
function linkLog(dir, kind, text, tone) {
  radio.log.unshift({ t: radio.t, dir, kind, text, tone }); radio.logN++;
  if (radio.log.length > 300) radio.log.length = 300;
}
const be16s = (p, i) => { const v = (p[i] << 8) | p[i + 1]; return v & 0x8000 ? v - 0x10000 : v; };
const cstrBytes = (p, i) => { let s = ''; for (; i < p.length && p[i]; i++) s += String.fromCharCode(p[i]); return s; };
function cmdDesc(f) {   // a command frame, in words (the command module's scaling: rc_core.h rc_cmd_scale)
  const p = f.subarray(3, f.length - 1);
  if (p[0] !== CRSF.EXT_CMD) return `extended frame 0x${p[0].toString(16)}`;
  const cmd = p[1], seq = p[2], v = []; for (let i = 3; i + 1 < p.length; i += 2) v.push(be16s(p, i));
  if (cmd === 1) return `go to ${(v[0] / 100).toFixed(1)}, ${(v[1] / 100).toFixed(1)}, ${(v[2] / 100).toFixed(1)} m, heading ${Math.round(v[3] / 1000 * R2D)}° (#${seq})`;
  if (cmd === 2) return `learning: ${{ 1: 'calibrate', 2: 'stop', 3: 'fly on the description', 4: 'fly on the learned model' }[v[0]] || 'command ' + v[0]} (#${seq})`;
  return `command ${cmd} ${v.join(' ')} (#${seq})`;
}
function frameKind(f) { const p = f.subarray(3, f.length - 1); return f[2] === CRSF.FLIGHT_MODE ? 'mode' : f[2] === CRSF.EXT && p[0] === CRSF.EXT_TEXT ? 'msg' : 'frame'; }
function frameDesc(f) {   // a telemetry frame, in words
  const p = f.subarray(3, f.length - 1);
  switch (f[2]) {
    case CRSF.ATTITUDE: return `attitude (roll ${(be16s(p, 2) / 1e4 * R2D).toFixed(0)}°)`;
    case CRSF.BATTERY: return `battery (${(((p[0] << 8) | p[1]) / 10).toFixed(1)} V)`;
    case CRSF.GPS: return 'GPS';
    case CRSF.BARO_ALT: return 'height';
    case CRSF.FLIGHT_MODE: return `flight mode ${cstrBytes(p, 0)}`;
    case CRSF.EXT:
      if (p[0] === CRSF.EXT_TEXT) return `message "${cstrBytes(p, 2)}"`;
      if (p[0] === CRSF.EXT_ITEM) return `${(TLM_ITEMS[p[1]] || { key: 'item ' + p[1] }).key} item`;
      return 'extended frame';
  }
  return `frame 0x${f[2].toString(16)}`;
}
function linkDown(f, t) {   // a whole frame out of the transmitter module, to the command module
  const m = radio.downMeta.shift(); if (!m) return;
  const lat = Math.round((t - m.t0) * 1000), extra = m.tries > Math.ceil(f.length / 5) ? `, ${m.tries - Math.ceil(f.length / 5)} packets lost and sent again` : '';
  if (m.kind === 'mode') { const mode = cstrBytes(f.subarray(3, f.length - 1), 0); if (mode === radio.modeSeen && !radioLogAll) return; radio.modeSeen = mode; }
  if (m.kind === 'msg' || m.kind === 'mode' || radioLogAll) linkLog('↓', m.kind, `${m.desc}: from the drone to the command module in ${lat} ms${extra}`, '');
}
const SWITCHES = [[4, 'arm'], [5, 'speed level'], [6, 'fly (take off / land)'], [7, 'hold'], [8, 'home']];
function linkSwitches(ch, t) {   // a switch's new position, the first time the drone's receiver passes it on
  const d = radio.delivered; radio.delivered = ch.slice();
  if (!d) return;
  for (const [i, name] of SWITCHES) {
    const a = Math.round(d[i] * 2) / 2, b = Math.round(ch[i] * 2) / 2; if (a === b) continue;
    const pos = i === 5 ? ['gentle', 'normal', 'sport'][Math.round(b) + 1] : b > 0 ? 'on' : 'off';
    linkLog('↑', 'switch', `${name} ${pos} (channel ${i + 1}): reached the drone's receiver`, '');
  }
}

/* ───────── the ground station: what the command module decoded ───────── */
// gs is the Ground station tab's copy of the command module's view (ground_core.c gnd_view_pack), read a few times a
// second: values by kind with when each came (gs.at, simulator time), the messages, the alert, the channels it sent.
const gs = { v: {}, at: {}, log: [], frames: 0, bytes: 0, trackXY: [], link: null, alert: null, sent: null };
function gsReset() { Object.assign(gs, { v: {}, at: {}, log: [], frames: 0, bytes: 0, trackXY: [], link: null, rate: [], alert: null, sent: null, nmsg: 0, posAt: -1 }); }
function gsRead() {
  const g = brt.gnd; if (!g) return;
  const t = brt.t, n = g.gnd_view(t), o = new Float32Array(g.memory.buffer, g.fr_ptr(), n), at = a => a >= 0 ? t - a : null;
  const put = (kind, age, v) => { const a = at(age); if (a == null) return; gs.v[kind] = v; gs.at[kind] = a; };
  gs.alert = { level: o[0], why: o[1], text: cstr(g, g.gnd_why_text(o[1]), 80) };
  gs.bytes = o[3]; gs.frames = o[4];
  if (o[8] >= 0) gs.link = { upRssi: o[9], upLq: o[10], upSnr: o[11], downRssi: o[12], downLq: o[13], downSnr: o[14], power: o[15], t: t - o[8] };
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
  if (gs.v.pos && gs.at.pos !== gs.posAt) { gs.posAt = gs.at.pos; gs.trackXY.push([gs.v.pos.x, gs.v.pos.y]); if (gs.trackXY.length > 400) gs.trackXY.shift(); }
  const nm = o[6];
  for (let i = Math.max(gs.nmsg, nm - 16); i < nm; i++) { gs.log.unshift({ t: g.gnd_msg_t(i), sev: g.gnd_msg_sev(i), text: cstr(g, g.gnd_msg_text(i), 60) }); if (gs.log.length > 60) gs.log.length = 60; }
  gs.nmsg = nm;
}
