'use strict';
// The ExpressLRS link, simulated (a model for link.js): a transmitter module wired by CRSF to the command module, a
// receiver wired by CRSF to the drone board that runs the telemetry task, and the air between them. The drone gets
// what a real receiver gives: channel frames when uplink packets get through, link statistics ten times a second,
// the command module's commands; what the drone writes to its receiver goes down in the telemetry slots and comes out
// of the transmitter module to the command module, with the module's own link statistics.
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
//     (LoRa at 2.4 GHz: −105 dBm at 500 Hz to −115 dBm at 50 Hz), on the path link.js works out (linkPath).

const ELRS_RATES = { 50: -115, 150: -112, 250: -108, 500: -105 };   // receiver sensitivity [dBm]
const ELRS_AIR = { 50: 0.013, 150: 0.005, 250: 0.003, 500: 0.0016 };  // a packet's time on air, roughly [s]: it arrives that long after it's sent
const ELRS_RATIOS = [2, 4, 8, 16, 32, 64, 128];
const ELRS_POWERS = [10, 25, 100, 250, 500, 1000];   // [mW]
const elrs = {};   // the model's own state (radio, in link.js, holds what every link shares)

// The signal at the drone: transmit power and the path, against the receiver's sensitivity at this rate.
function elrsRf() {
  const P = linkPath();
  const rssi = dbm(radioCfg.power) + 4 - P.loss, sens = ELRS_RATES[radioCfg.rate] || -108;
  return { d: P.d, walls: P.walls, rssi, margin: rssi - sens, snr: clamp(rssi - sens - 2, -18, 13) };
}
const elrsP = margin => 1 / (1 + Math.exp(-(margin - 3) / 1.8));   // a packet gets through

RADIO_LINKS.elrs = {
  label: 'ExpressLRS 2.4 GHz', receiver: 'ExpressLRS receiver',
  settings: [
    { key: 'rate', label: 'Packet rate', options: Object.keys(ELRS_RATES).map(k => [k, k + ' Hz']) },
    { key: 'ratio', label: 'Telemetry', options: ELRS_RATIOS.map(k => [k, '1:' + k]) },
    { key: 'power', label: 'Power', options: ELRS_POWERS.map(k => [k, k + ' mW']) },
  ],
  wasm: c => [0, c.rate, c.ratio],                                   // radio_link.h RLINK_ELRS
  room: c => c.rate / c.ratio * 5,
  roomNote(c) {
    const room = this.room(c);
    return room < 40 ? `At ${c.rate} Hz with telemetry 1:${c.ratio}, only ${room < 10 ? room.toFixed(1) : Math.round(room)} bytes a second can come down (one packet in ${c.ratio}, 5 bytes each); a frame is 10–40 bytes, so values arrive seconds apart. The channels and commands go up in the other packets, so control isn't affected. 1:2 to 1:8 leaves the Ground station enough.` : '';
  },
  signalNote(c) {
    const rf = radio.rf; if (!rf) return '';
    return `Now ${rf.d.toFixed(0)} m from the handset${rf.walls ? `, ${rf.walls} building${rf.walls > 1 ? 's' : ''} in the way` : ''}; with the extra loss that is like ${fmtDist(rf.d * Math.pow(10, c.extra / 20))} in the open. Signal ${rf.rssi.toFixed(0)} dBm, the receiver needs ${ELRS_RATES[c.rate]} dBm at ${c.rate} Hz.`;
  },
  reset() {
    Object.assign(elrs, {
      nextPkt: 0, k: 0, upK: 0,
      fifo: [], fifoBytes: 0, cur: null, downDropped: 0, txIn: crsfParser(),
      up: [], nextStats: 0,
      upHist: [], downHist: [], downChunks: 0, downGot: 0, upGot: 0,
      air: [], lastUpOk: 0, upGap: false, lastDownOk: 0, downGap: false, lqLow: false,
      tlmHeard: -1,   // when the transmitter module last heard a telemetry packet (−1: never: not connected)
      txConn: false, txLostAt: -1, txLostLq: 0,   // its connection, and when and at what uplink LQ it was last lost
    });
  },
  resume(t) { Object.assign(elrs, { nextPkt: t, nextStats: t, lastUpOk: t, lastDownOk: t }); },   // (taking over in flight from another link)
  step: elrsStep,
  fromDrone: elrsFromDrone,
  command(f) {   // the transmitter module passes them on only while connected; 5 bytes at a time in place of some channel packets
    if (!elrsConnected()) { linkEv('cmdDrop', radio.t); linkLog('↑', 'cmd', cmdDesc(f), 'dropped · link down', 'bad', f); return; }
    elrs.up.push({ bytes: f, at: 0, got: [], t0: radio.t, pk: 0, lost: 0, desc: cmdDesc(f) });   // (logged when it reaches the drone)
  },
  connected: () => elrsConnected(),
  stats: () => ({ queued: elrs.fifoBytes, upQueued: elrs.up.length }),
};

// Each 1 ms step: the packets due.
function elrsStep(dt, t) {
  if (radio.rfAt < 0 || t - radio.rfAt > 0.02) { const r = elrsRf(); radio.rf = r; radio.rfAt = t; }
  const r = radio.rf, on = cargo.power, pUp = on ? elrsP(r.margin) : 0, pDown = on ? elrsP(r.margin - 1) : 0;   // (the receiver's antenna is a little worse; with no power on the drone, nothing at all)
  const period = 1 / radioCfg.rate, air = ELRS_AIR[radioCfg.rate] || 0.003;
  while (elrs.nextPkt <= t + 1e-9) {
    const tp = elrs.nextPkt;                                         // this packet goes now and arrives `air` later
    elrs.nextPkt += period; elrs.k++;
    const tlmSlot = radioCfg.ratio > 0 && elrs.k % radioCfg.ratio === 0;
    if (tlmSlot) {
      const ok = radioRand() < pDown;
      elrs.downHist.push(ok ? 1 : 0); if (elrs.downHist.length > 100) elrs.downHist.shift();
      linkEv(ok ? 'downOk' : 'downLost', t);
      if (!elrs.cur) elrs.cur = fifoPop();                           // the sender is free: the queue's oldest frame leaves it
      const c = elrs.cur;
      if (c) {
        elrs.downChunks++; { const m = radio.meta.get(c.id); if (m) m.tries++; }
        if (ok) {
          const chunk = c.f.subarray(c.at, c.at + 5); c.at += 5; elrs.downGot++;
          if (c.at >= c.f.length) elrs.cur = null;
          elrs.air.push({ at: tp + air, down: chunk, id: c.id });    // (the module then hands on whole frames)
        }
      }
      if (ok) elrs.tlmHeard = t;
      if (ok) { if (elrs.downGap) linkLog('↓', 'link', 'telemetry back', `after ${(t - elrs.lastDownOk).toFixed(1)} s`, 'good'); elrs.lastDownOk = t; elrs.downGap = false; }
      else if (!elrs.downGap && t - elrs.lastDownOk > 1) { elrs.downGap = true; linkLog('↓', 'link', 'telemetry lost', 'nothing for 1 s', 'bad'); }
    } else {
      const ok = radioRand() < pUp;
      elrs.upHist.push(ok ? 1 : 0); if (elrs.upHist.length > 100) elrs.upHist.shift();
      elrs.upK++;
      const cmdSlot = elrs.up.length && elrs.upK % 2;               // (every other uplink packet: the uplink's own count, whatever the telemetry ratio)
      linkEv(ok ? 'upOk' : 'upLost', t);
      if (!ok) {
        if (cmdSlot) elrs.up[0].lost++;
        if (!elrs.upGap && t - elrs.lastUpOk > 0.5) { elrs.upGap = true; linkLog('↑', 'link', 'uplink lost', 'no channels for 0.5 s', 'bad'); }
        continue;
      }
      elrs.upGot++;
      if (elrs.upGap) linkLog('↑', 'link', 'uplink back', `after ${(t - elrs.lastUpOk).toFixed(1)} s`, 'good');
      elrs.upGap = false; elrs.lastUpOk = t;
      if (cmdSlot) {                                                 // this packet carries 5 bytes of a command
        const c = elrs.up[0]; c.got.push(...c.bytes.subarray(c.at, c.at + 5)); c.at += 5; c.pk++;
        if (c.at >= c.bytes.length) {
          elrs.up.shift(); elrs.air.push({ at: tp + air, cmd: c });
        }
      } else if (radio.txCh) {
        elrs.air.push({ at: tp + air, ch: radio.txCh, made: radio.txChT });
      }   // (nothing from the command module yet: nothing to send)
    }
  }
  const D = radio.dropRun;
  if (D && D.n && t - D.last > 0.5) { linkLog('↓', 'drop', `${D.n} frames dropped`, `${(D.last - D.t0).toFixed(1)} s · receiver queue full`, 'warn'); radio.dropRun = null; }
  if (radio.lqUp < 50 !== elrs.lqLow && elrs.upHist.length >= 50) { elrs.lqLow = radio.lqUp < 50; linkLog('↑', 'link', `LQ ${Math.round(radio.lqUp)}%`, elrs.lqLow ? 'below 50%' : 'above 50% again', elrs.lqLow ? 'warn' : 'good'); }
  // what has finished its time on air arrives
  while (elrs.air.length && elrs.air[0].at <= t + 1e-9) {
    const a = elrs.air.shift();
    if (a.down) elrs.txIn.feed(a.down, f => { radio.toGround.push(f); linkDown(f, t, a.id); });
    else if (a.cmd) {
      const c = a.cmd, got = Uint8Array.from(c.got); radio.toBoard.push(got); linkEv('cmdOut', t, (t - c.t0) * 1000);
      linkLog('↑', 'cmd', c.desc, `${Math.round((t - c.t0) * 1000)} ms · ${c.pk} pk${c.lost ? ` · ${c.lost} resent` : ''}`, '', got);
    } else if (a.ch) {
      const fr = crsfRcFrame(a.ch); radio.toBoard.push(fr); linkEv('chSent', t, (t - a.made) * 1000);
      linkChannels(a.ch, t, fr, (t - a.made) * 1000);
    }
  }
  const lq = h => h.length ? 100 * h.reduce((a, b) => a + b, 0) / h.length : 0;
  radio.lqUp = lq(elrs.upHist); radio.lqDown = lq(elrs.downHist);
  const rssiUp = r.rssi + (radioRand() - 0.5) * 2, rssiDown = r.rssi - 1 + (radioRand() - 0.5) * 2;
  if (t >= elrs.nextStats) {                                         // link statistics to the drone, 10 times a second
    elrs.nextStats = t + 0.1;
    if (radio.lqUp > 0) radio.toBoard.push(crsfLinkStatsFrame({ upRssi: rssiUp, upLq: radio.lqUp, upSnr: r.snr, downRssi: rssiDown, downLq: radio.lqDown, downSnr: r.snr - 1, rfMode: [50, 150, 250, 500].indexOf(radioCfg.rate), power: radioCfg.power }));
    const ls = crsfLinkStatsFrame({ upRssi: rssiUp, upLq: elrsTxLq(t), upSnr: r.snr, downRssi: rssiDown, downLq: radio.lqDown, downSnr: r.snr - 1, rfMode: [50, 150, 250, 500].indexOf(radioCfg.rate), power: radioCfg.power });
    ls[0] = CRSF.ADDR_HANDSET; radio.toGround.push(ls);   // the transmitter module tells the command module too, connected or not (LQ 0 until it connects, as a real one does)
  }
}
// What the drone's board wrote to the receiver: into its telemetry queue, as ExpressLRS's RXOTAConnector keeps it
// (512 bytes, a length byte and the frame each; entries {id, f, cap: the room it holds, del}). The telemetry slots
// take the frames from its head (fifoPop) and send them 5 bytes at a time.
const RX_FIFO = 512;
function elrsFromDrone(f, id) {
  const text = f[2] === CRSF.EXT && f[3] === CRSF.EXT_TEXT;
  // a frame of a type below 0x28 (a "broadcast" one), or a status text, replaces the one of its kind still waiting
  const q = f[2] < 0x28 || text ? elrs.fifo.find(e => !e.del && e.f[2] === f[2] && (!text || e.f[3] === CRSF.EXT_TEXT)) : null;
  if (q) {
    fifoGone(q.id, 'replaced');
    if (q.cap >= f.length) { q.f = f; q.id = id; return; }           // newest wins, in the old one's place
    q.del = true;                                                    // too big for its room: the old one is marked gone (its room comes free at the head), the new one goes to the end
  }
  while (elrs.fifoBytes + f.length + 1 > RX_FIFO && elrs.fifo.length) {   // full: the oldest go until there's room
    const e = elrs.fifo.shift(); elrs.fifoBytes -= e.cap + 1;
    if (!e.del) fifoGone(e.id, 'full');
  }
  elrs.fifo.push({ id, f, cap: f.length, del: false }); elrs.fifoBytes += f.length + 1;
}
// The sender takes the next frame from the queue's head (skipping the ones marked gone): {id, f, at: bytes sent}.
function fifoPop() {
  while (elrs.fifo.length) {
    const e = elrs.fifo.shift(); elrs.fifoBytes -= e.cap + 1;
    if (!e.del) return { id: e.id, f: e.f, at: 0 };
  }
  return null;
}
// A queued frame that won't go down: replaced by a newer one of its kind (normal for telemetry: superseded; a status
// text lost that way is a drop), or pushed out of a full queue (a drop).
function fifoGone(id, why) {
  const m = radio.meta.get(id); if (!m) return; radio.meta.delete(id);
  if (why === 'replaced' && m.kind !== 'msg') { linkEv('tlmSuper', radio.t); return; }
  elrs.downDropped++; linkEv('tlmDrop', radio.t);
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
function elrsTxLq(t) {
  const c = elrsConnected();
  if (c !== elrs.txConn) { elrs.txConn = c; if (!c) { elrs.txLostAt = t; elrs.txLostLq = radio.lqUp; } }
  if (c) return radio.lqUp;
  return elrs.txLostAt >= 0 && t - elrs.txLostAt <= (clamp(elrs.txLostLq, 50, 100) - 50) / 50 * 3 ? elrs.txLostLq : 0;
}
// The transmitter module's link is connected while it hears telemetry packets: within 5 telemetry slots, at least
// 0.512 s (ExpressLRS tx_main.cpp UpdateConnectDisconnectStatus).
const elrsConnected = () => elrs.tlmHeard >= 0 && radio.t - elrs.tlmHeard <= Math.max(0.512, 5 * radioCfg.ratio / radioCfg.rate) + 0.002;
