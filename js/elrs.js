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
//     chunk until it gets through, so frames arrive whole and in order, just later on a bad link. The receiver's
//     queue (RXOTAConnector) holds 512 bytes, a length byte and the frame each; a frame of a type below 0x28, or a
//     status text, replaces the one of its kind still waiting (newest wins, in its place); when the queue is full the
//     oldest frames go. The frame being sent has already left the queue;
//   - the transmitter module passes the command module's frames other than the channels (commands) only while its
//     link is connected: a telemetry packet heard within the last 5 telemetry slots (at least 0.512 s); else it
//     drops them (TXOTAConnector);
//   - each packet gets through with a probability set by the margin over the receiver's sensitivity for its rate
//     (LoRa at 2.4 GHz: −105 dBm at 500 Hz to −115 dBm at 50 Hz). The signal: transmit power and 2 dBi antennas, free-
//     space loss to the drone, 18 dB for each building in the way (the city worlds), and the "extra loss" setting,
//     which stands in for distance, walls and interference the simulated world is too small to have.
// The command module's inputs are you: the keys and the simulator's pilot (arm, take off) are its buttons and sticks.

const ELRS_RATES = { 50: -115, 150: -112, 250: -108, 500: -105 };   // receiver sensitivity [dBm]
const ELRS_AIR = { 50: 0.013, 150: 0.005, 250: 0.003, 500: 0.0016 };  // a packet's time on air, roughly [s]: it arrives that long after it's sent
const ELRS_RATIOS = [2, 4, 8, 16, 32, 64, 128];
const ELRS_POWERS = [10, 25, 100, 250, 500, 1000];   // [mW]
const radioCfg = { rate: 250, ratio: 4, power: 100, extra: 0 };
const radio = {};
function radioReset() {
  Object.assign(radio, {
    t: 0, nextPkt: 0, k: 0, upK: 0, seed: 0x2545F491,
    fifo: [], fifoBytes: 0, cur: null, downFrames: 0, downDropped: 0, fromGround: crsfParser(), fromDrone: crsfParser(), txIn: crsfParser(),
    up: [], txCh: null, toBoard: [], toGround: [], nextStats: 0, ch: null,
    upHist: [], downHist: [], rssiUp: -50, rssiDown: -50, snrUp: 10, snrDown: 10, rfAt: -1,
    downChunks: 0, downGot: 0, upGot: 0, rxLost: false, holdUntil: -1, homeUntil: -1,
    log: [], logN: 0, meta: new Map(), ev: {}, stickLogged: null, dropRun: null, lqUp: 0, lqDown: 0, rf: null, txChT: 0, air: [], delivered: null, lastUpOk: 0, upGap: false, lastDownOk: 0, downGap: false, lqLow: false, modeSeen: '',
    tlmHeard: -1,   // when the transmitter module last heard a telemetry packet (−1: never: not connected)
    txConn: false, txLostAt: -1, txLostLq: 0,   // its connection, and when and at what uplink LQ it was last lost
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
  const r = radio.rf, on = cargo.power, pUp = on ? radioP(r.margin) : 0, pDown = on ? radioP(r.margin - 1) : 0;   // (the receiver's antenna is a little worse; with no power on the drone, nothing at all)
  const period = 1 / radioCfg.rate, air = ELRS_AIR[radioCfg.rate] || 0.003;
  while (radio.nextPkt <= t + 1e-9) {
    const tp = radio.nextPkt;                                        // this packet goes now and arrives `air` later
    radio.nextPkt += period; radio.k++;
    const tlmSlot = radioCfg.ratio > 0 && radio.k % radioCfg.ratio === 0;
    if (tlmSlot) {
      const ok = radioRand() < pDown;
      radio.downHist.push(ok ? 1 : 0); if (radio.downHist.length > 100) radio.downHist.shift();
      linkEv(ok ? 'downOk' : 'downLost', t);
      if (!radio.cur) radio.cur = fifoPop();                        // the sender is free: the queue's oldest frame leaves it
      const c = radio.cur;
      if (c) {
        radio.downChunks++; { const m = radio.meta.get(c.id); if (m) m.tries++; }
        if (ok) {
          const chunk = c.f.subarray(c.at, c.at + 5); c.at += 5; radio.downGot++;
          if (c.at >= c.f.length) radio.cur = null;
          radio.air.push({ at: tp + air, down: chunk, id: c.id });  // (the module then hands on whole frames)
        }
      }
      if (ok) radio.tlmHeard = t;
      if (ok) { if (radio.downGap) linkLog('↓', 'link', 'telemetry back', `after ${(t - radio.lastDownOk).toFixed(1)} s`, 'good'); radio.lastDownOk = t; radio.downGap = false; }
      else if (!radio.downGap && t - radio.lastDownOk > 1) { radio.downGap = true; linkLog('↓', 'link', 'telemetry lost', 'nothing for 1 s', 'bad'); }
    } else {
      const ok = radioRand() < pUp;
      radio.upHist.push(ok ? 1 : 0); if (radio.upHist.length > 100) radio.upHist.shift();
      radio.upK++;
      const cmdSlot = radio.up.length && radio.upK % 2;             // (every other uplink packet: the uplink's own count, whatever the telemetry ratio)
      linkEv(ok ? 'upOk' : 'upLost', t);
      if (!ok) {
        if (cmdSlot) radio.up[0].lost++;
        if (!radio.upGap && t - radio.lastUpOk > 0.5) { radio.upGap = true; linkLog('↑', 'link', 'uplink lost', 'no channels for 0.5 s', 'bad'); }
        continue;
      }
      radio.upGot++;
      if (radio.upGap) linkLog('↑', 'link', 'uplink back', `after ${(t - radio.lastUpOk).toFixed(1)} s`, 'good');
      radio.upGap = false; radio.lastUpOk = t;
      if (cmdSlot) {                                                 // this packet carries 5 bytes of a command
        const c = radio.up[0]; c.got.push(...c.bytes.subarray(c.at, c.at + 5)); c.at += 5; c.pk++;
        if (c.at >= c.bytes.length) {
          radio.up.shift(); radio.air.push({ at: tp + air, cmd: c });
        }
      } else if (radio.txCh) {
        radio.air.push({ at: tp + air, ch: radio.txCh, made: radio.txChT });
      }   // (nothing from the command module yet: nothing to send)
    }
  }
  const D = radio.dropRun;
  if (D && D.n && t - D.last > 0.5) { linkLog('↓', 'drop', `${D.n} frames dropped`, `${(D.last - D.t0).toFixed(1)} s · receiver queue full`, 'warn'); radio.dropRun = null; }
  if (radio.lqUp < 50 !== radio.lqLow && radio.upHist.length >= 50) { radio.lqLow = radio.lqUp < 50; linkLog('↑', 'link', `LQ ${Math.round(radio.lqUp)}%`, radio.lqLow ? 'below 50%' : 'above 50% again', radio.lqLow ? 'warn' : 'good'); }
  // what has finished its time on air arrives
  while (radio.air.length && radio.air[0].at <= t + 1e-9) {
    const a = radio.air.shift();
    if (a.down) radio.txIn.feed(a.down, f => { radio.toGround.push(f); linkDown(f, t, a.id); });
    else if (a.cmd) {
      const c = a.cmd, got = Uint8Array.from(c.got); radio.toBoard.push(got); linkEv('cmdOut', t, (t - c.t0) * 1000);
      linkLog('↑', 'cmd', c.desc, `${Math.round((t - c.t0) * 1000)} ms · ${c.pk} pk${c.lost ? ` · ${c.lost} resent` : ''}`, '', got);
    } else if (a.ch) {
      const fr = crsfRcFrame(a.ch); radio.toBoard.push(fr); linkEv('chSent', t, (t - a.made) * 1000);
      linkChannels(a.ch, t, fr, (t - a.made) * 1000);
    }
  }
  const lq = h => h.length ? 100 * h.reduce((a, b) => a + b, 0) / h.length : 0;
  radio.lqUp = lq(radio.upHist); radio.lqDown = lq(radio.downHist);
  radio.rssiUp = r.rssi + (radioRand() - 0.5) * 2; radio.rssiDown = r.rssi - 1 + (radioRand() - 0.5) * 2;
  if (t >= radio.nextStats) {                                        // link statistics to the drone, 10 times a second
    radio.nextStats = t + 0.1;
    if (radio.lqUp > 0) radio.toBoard.push(crsfLinkStatsFrame({ upRssi: radio.rssiUp, upLq: radio.lqUp, upSnr: r.snr, downRssi: radio.rssiDown, downLq: radio.lqDown, downSnr: r.snr - 1, rfMode: [50, 150, 250, 500].indexOf(radioCfg.rate), power: radioCfg.power }));
    const ls = crsfLinkStatsFrame({ upRssi: radio.rssiUp, upLq: radioTxLq(t), upSnr: r.snr, downRssi: radio.rssiDown, downLq: radio.lqDown, downSnr: r.snr - 1, rfMode: [50, 150, 250, 500].indexOf(radioCfg.rate), power: radioCfg.power });
    ls[0] = CRSF.ADDR_HANDSET; radio.toGround.push(ls);   // the transmitter module tells the command module too, connected or not (LQ 0 until it connects, as a real one does)
  }
}
// What the drone's board wrote to the receiver: into its telemetry queue, as ExpressLRS's RXOTAConnector keeps it
// (512 bytes, a length byte and the frame each; entries {id, f, cap: the room it holds, del}). The telemetry slots
// take the frames from its head (fifoPop) and send them 5 bytes at a time.
const RX_FIFO = 512;
function radioFromDrone(bytes) {
  radio.fromDrone.feed(bytes, f => {
    const id = ++radio.downFrames, text = f[2] === CRSF.EXT && f[3] === CRSF.EXT_TEXT;
    radio.meta.set(id, { id, t0: radio.t, desc: frameDesc(f), kind: frameKind(f), tries: 0, bytes: f });
    if (radio.meta.size > 600) radio.meta.delete(radio.meta.keys().next().value);   // (only waiting frames have one: never this many)
    linkEv('tlmIn', radio.t);
    // a frame of a type below 0x28 (a "broadcast" one), or a status text, replaces the one of its kind still waiting
    const q = f[2] < 0x28 || text ? radio.fifo.find(e => !e.del && e.f[2] === f[2] && (!text || e.f[3] === CRSF.EXT_TEXT)) : null;
    if (q) {
      fifoGone(q.id, 'replaced');
      if (q.cap >= f.length) { q.f = f; q.id = id; return; }       // newest wins, in the old one's place
      q.del = true;                                                  // too big for its room: the old one is marked gone (its room comes free at the head), the new one goes to the end
    }
    while (radio.fifoBytes + f.length + 1 > RX_FIFO && radio.fifo.length) {   // full: the oldest go until there's room
      const e = radio.fifo.shift(); radio.fifoBytes -= e.cap + 1;
      if (!e.del) fifoGone(e.id, 'full');
    }
    radio.fifo.push({ id, f, cap: f.length, del: false }); radio.fifoBytes += f.length + 1;
  });
}
// The sender takes the next frame from the queue's head (skipping the ones marked gone): {id, f, at: bytes sent}.
function fifoPop() {
  while (radio.fifo.length) {
    const e = radio.fifo.shift(); radio.fifoBytes -= e.cap + 1;
    if (!e.del) return { id: e.id, f: e.f, at: 0 };
  }
  return null;
}
// A queued frame that won't go down: replaced by a newer one of its kind (normal for telemetry: superseded; a status
// text lost that way is a drop), or pushed out of a full queue (a drop).
function fifoGone(id, why) {
  const m = radio.meta.get(id); if (!m) return; radio.meta.delete(id);
  if (why === 'replaced' && m.kind !== 'msg') { linkEv('tlmSuper', radio.t); return; }
  radio.downDropped++; linkEv('tlmDrop', radio.t);
  if (m.kind === 'msg') linkLog('↓', 'drop', m.desc, why === 'replaced' ? 'replaced by a newer message · receiver queue' : 'dropped · receiver queue full', 'bad', m.bytes);
  else {                                                             // the rest: one line when it starts, one when it ends
    const D = radio.dropRun || (radio.dropRun = { n: 0, t0: radio.t, kinds: new Set() });
    if (!D.n) linkLog('↓', 'drop', 'dropping frames', `receiver queue full (${RX_FIFO} B)`, 'warn');
    D.n++; D.last = radio.t; D.kinds.add(m.desc.replace(/ \(.*\)$/, ''));
  }
}
// The uplink LQ the transmitter module reports to the command module (its link statistics go every 0.1 s, connected or
// not): the receiver's, which it learns from the telemetry (so 0 before the link first connects); when it's lost, the
// last one for a while (up to 3 s, shorter the worse it was), then 0 (tx_main.cpp checkSendLinkStatsToHandset).
function radioTxLq(t) {
  const c = radioConnected();
  if (c !== radio.txConn) { radio.txConn = c; if (!c) { radio.txLostAt = t; radio.txLostLq = radio.lqUp; } }
  if (c) return radio.lqUp;
  return radio.txLostAt >= 0 && t - radio.txLostAt <= (clamp(radio.txLostLq, 50, 100) - 50) / 50 * 3 ? radio.txLostLq : 0;
}
// The transmitter module's link is connected while it hears telemetry packets: within 5 telemetry slots, at least
// 0.512 s (ExpressLRS tx_main.cpp UpdateConnectDisconnectStatus).
const radioConnected = () => radio.tlmHeard >= 0 && radio.t - radio.tlmHeard <= Math.max(0.512, 5 * radioCfg.ratio / radioCfg.rate) + 0.002;
// What the command module wrote to the transmitter module: its channel frames (the latest goes up in each uplink
// packet) and its commands (queued, and sent 5 bytes at a time in place of some channel packets).
function radioFromGround(bytes) {
  radio.fromGround.feed(bytes, f => {
    if (f[2] === CRSF.RC) { radio.txCh = crsfRcRead(f.subarray(3, f.length - 1)); radio.txChT = radio.t; linkEv('chMade', radio.t); }
    else if (f[2] === CRSF.EXT) {
      if (!radioConnected()) { linkEv('cmdDrop', radio.t); linkLog('↑', 'cmd', cmdDesc(f), 'dropped · link down', 'bad', f); return; }   // (the module passes them on only while connected)
      radio.up.push({ bytes: f, at: 0, got: [], t0: radio.t, pk: 0, lost: 0, desc: cmdDesc(f) });   // (logged when it reaches the drone)
    }
  });
}
// A ground-station command (go to, calibrate…): the command module queues it and sends it up.
// A command for the command module to send. 0, or −1 if it has too many waiting (said in the log, not silently lost).
function radioCommand(cmd, values) {
  const g = brt.gnd; if (!g) return -1;
  let r; if (cmd === 1) r = g.gnd_goto(...values); else { frIn(g, values); r = g.gnd_command(cmd, values.length); }
  if (r) linkLog('↑', 'cmd', `${cmd === 1 ? 'GOTO' : cmd === 3 ? 'LATCH' : 'command ' + cmd} not sent`, r === -2 ? 'a value out of range' : 'the command module has too many waiting', 'bad');
  return r ? -1 : 0;
}
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
  const span = Math.min(W, Math.max(0.5, t)), pct = (a, b) => a + b ? 100 * a / (a + b) : 0;
  return {
    chMade: n('chMade') / span, chSent: n('chSent') / span, chLat: lat('chSent'), upLostPct: pct(n('upLost'), n('upOk')),
    cmds: n('cmdOut'), cmdLat: lat('cmdOut'),
    tlmIn: n('tlmIn') / span, tlmOut: n('tlmOut') / span, tlmLat: lat('tlmOut'), tlmDrop: n('tlmDrop'), tlmSuper: n('tlmSuper') / span, downLostPct: pct(n('downLost'), n('downOk')),
    queued: radio.fifoBytes, upQueued: radio.up.length, cmdDrop: n('cmdDrop'),
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
function linkDown(f, t, id) {   // a whole frame out of the transmitter module, to the command module (id: the frame the receiver sent)
  const m = radio.meta.get(id); if (!m) return;
  radio.meta.delete(id);
  const lat = `${Math.round((t - m.t0) * 1000)} ms`, resent = m.tries - Math.ceil(f.length / 5), meta = resent > 0 ? `${lat} · ${resent} resent` : lat;
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
  if (typeof GS_UI !== 'undefined') Object.assign(GS_UI, { built: false, paused: null, clearId: 0, logN: -1, cfgNote: '' });   // (the widgets hold the old run's values: built again; the log starts again)
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
