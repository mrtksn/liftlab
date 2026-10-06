'use strict';
// The nRF24L01 link, simulated (a model for link.js, its bookkeeping beside link-packet.js's): a module with a printed
// antenna at each end, in Nordic's Enhanced ShockBurst. The packet layer is the real C at both ends (runner/fc/clink.h,
// through board_wasm.c's plink_* calls, which take the compact link when it is set up): it makes the ground's packet
// on its beat, the drone's answer, picks the channel each end is on (plink_channel: they hop over 8 from the binding
// phrase). This file is the radios and the air between:
//   - the ground's module sends its packet; if the drone's module is on that channel (and powered) and hears it, it
//     takes it (the first time: a retry of one taken is dropped) and acknowledges at once, with the answer loaded
//     for it (the drone's packet layer loads the next one each time the last has gone: its answer to the packet before);
//   - no acknowledgement heard: the ground's module tries again, up to 3 more times, then gives up (the packet lost);
//   - a try gets through with a chance set by the margin over the sensitivity at the data rate (−94 dBm at 250 kbit/s,
//     −85 at 1 Mbit/s, −82 at 2 Mbit/s; 0 dBm out, the printed antennas 3 dB worse than a dipole each, so about
//     100 m in the open at 1 Mbit/s and 300 m at 250 kbit/s), on the path link.js works
//     out (linkPath); the drone's transmissions 1 dB worse, as in the other models; a busy 2.4 GHz band (Wi-Fi) takes
//     2 dB more on a third of the channels.
// The time on air (the preamble, 5 address bytes, 9 bits of control, the payload, a 2-byte CRC; the acknowledgement
// the same with its payload; 130 µs to turn round; the retry wait) is the latency of each exchange.

const NRF_RATES = [[250, '250 kbit/s (furthest)'], [1000, '1 Mbit/s'], [2000, '2 Mbit/s']];
const NRF_SENS = { 250: -94, 1000: -85, 2000: -82 };
const nrfP = margin => 1 / (1 + Math.exp(-(margin - 2) / 1.5));
const nrfAir = (bytes, kbps) => (1 + 5 + 9 / 8 + bytes + 2) * 8 / (kbps * 1000);   // [s]
function nrfRf() {
  const P = linkPath(), c = radioCfg, rssi = 0 - 6 - P.loss, sens = NRF_SENS[c.kbps] || NRF_SENS[1000];
  return { d: P.d, walls: P.walls, rssi, sens, margin: rssi - sens };
}
function nrfStep(dt, t, E) {
  if (radio.rfAt < 0 || t - radio.rfAt > 0.02) { radio.rf = nrfRf(); radio.rfAt = t; }
  const drone = cargo.power ? E && E.drone : null, gnd = E && E.gnd;
  // the drone's module: its answer loaded once the last went
  if (drone && !pk.ackq) { const n = drone.plink_air_out(t); if (n) pk.ackq = { bytes: Uint8Array.from(new Uint8Array(drone.memory.buffer, drone.pbuf_ptr(), n)), t0: t }; }
  // the ground's module: its packet on its beat, on the channel for it
  if (gnd) {
    const ch = gnd.plink_channel(t), n = gnd.plink_air_out(t);
    if (n) nrfExchange(Uint8Array.from(new Uint8Array(gnd.memory.buffer, gnd.pbuf_ptr(), n)), ch, t, gnd, drone);
  }
  if (pk.air.some(a => a.at <= t + 1e-9)) {                          // (what got through, after its time on air)
    const due = pk.air.filter(a => a.at <= t + 1e-9).sort((a, b) => a.at - b.at || a.n - b.n);
    pk.air = pk.air.filter(a => a.at > t + 1e-9);
    for (const a of due) nrfArrive(a, a.from === 'gnd' ? drone : gnd, t);
  }
  pkEvents(t, gnd, drone);
}
// One packet's exchange: the tries, each heard or not, the acknowledgement (with the drone's answer) heard or not.
function nrfExchange(bytes, ch, t, gnd, drone) {
  const c = radioCfg, kb = c.kbps || 1000, rf = radio.rf, busy = ch % 3 === 0 ? 2 : 0;
  const up = { from: 'gnd', bytes, n: ++pkN, recs: [], cmds: [], made: bytes[0] >> 2 === 0x11 ? radio.txChT : null, at: 0 };   // (kind 1: channels)
  const ard = kb === 250 ? 0.0015 : 0.0005;
  let tt = t, taken = false;
  for (let k = 0; k < 4; k++) {
    tt += nrfAir(bytes.length, kb);
    const heard = drone && drone.plink_channel(t) === ch && radioRand() < nrfP(rf.margin - busy);
    if (heard && !taken) { taken = true; up.at = tt; pk.air.push(up); }
    if (heard) {
      const ack = pk.ackq, ackT = tt + 0.00013 + nrfAir(ack ? ack.bytes.length : 0, kb);
      if (radioRand() < nrfP(rf.margin - 1 - busy)) {                 // the acknowledgement heard: its payload goes with it
        if (ack) { pk.ackq = null; pk.air.push({ from: 'drone', bytes: ack.bytes, n: ++pkN, recs: [], cmds: [], at: ackT }); }
        return;
      }
      if (ack) { pk.ackq = null; linkEv('downLost', t); }          // (the answer went with the acknowledgement that wasn't heard)
    }
    tt += ard;
  }
  if (!taken) linkEv('upLost', t);
}
function nrfArrive(a, w, t) {
  if (!w) { linkEv(a.from === 'gnd' ? 'upLost' : 'downLost', t); return; }
  new Uint8Array(w.memory.buffer, w.pbuf_ptr(), a.bytes.length).set(a.bytes);
  if (!w.plink_air_in(a.bytes.length, 0, t)) { linkEv(a.from === 'gnd' ? 'upLost' : 'downLost', t); return; }
  if (a.from === 'gnd') {
    linkEv('upOk', t);
    if (pk.upGap) linkLog('↑', 'link', 'uplink back', `after ${(t - pk.lastUpOk).toFixed(1)} s`, 'good');
    pk.upGap = false; pk.lastUpOk = t; pk.upEver = true;
    if (a.made != null) pk.rcMade.push(a.made);
  } else {
    linkEv('downOk', t);
    if (pk.downGap) linkLog('↓', 'link', 'telemetry back', `after ${(t - pk.lastDownOk).toFixed(1)} s`, 'good');
    pk.downGap = false; pk.lastDownOk = t; pk.downEver = true;
  }
}
// A frame an end's packet layer gave its stack: the channels and commands reaching the drone, the telemetry and
// messages reaching the command module (the stream is in order and loses nothing: each is the oldest of its kind
// still waiting).
function nrfToStack(end, f, t) {
  if (f[2] === CRSF.LINK_STATS) return;
  if (end === 'drone') {
    if (f[2] === CRSF.RC) {
      const made = pk.rcMade.length ? pk.rcMade.shift() : null, lat = made != null ? (t - made) * 1000 : 0, ch = crsfRcRead(f.subarray(3, f.length - 1));
      while (pk.rcMade.length > 4) pk.rcMade.shift();
      linkEv('chSent', t, lat); linkChannels(ch, t, f, lat);
    } else if (f[2] === CRSF.EXT) {
      const key = pkKey(f), i = pk.cq.findIndex(c => c.key === key); if (i < 0) return;
      const c = pk.cq.splice(i, 1)[0], ms = Math.round((t - c.t0) * 1000);
      linkEv('cmdOut', t, (t - c.t0) * 1000); linkLog('↑', 'cmd', c.desc, `${ms} ms`, '', f);
    }
    return;
  }
  const key = pkKey(f), i = pk.dq.findIndex(x => x.key === key);
  if (i < 0) return;
  const e = pk.dq.splice(i, 1)[0]; linkDown(f, t, e.id, 1e9);
}

RADIO_LINKS.nrf24 = Object.assign(packetModel('espnow', []), {
  label: 'nRF24L01 2.4 GHz', receiver: 'nRF24L01 module',
  settings: [{ key: 'kbps', label: 'Data rate', options: NRF_RATES }],
  wasm: c => [4, [250, 1000, 2000].includes(+c.kbps) ? +c.kbps : 1000, 0],   // radio_link.h RLINK_NRF24
  room: c => (+c.kbps === 250 ? 50 : 100) * 21 * 0.6,                // radio_link.c rlink_budget
  roomNote: () => '',
  signalNote(c) {
    const rf = radio.rf; if (!rf || rf.sens == null) return '';
    return `Now ${rf.d.toFixed(0)} m from the handset${rf.walls ? `, ${rf.walls} building${rf.walls > 1 ? 's' : ''} in the way` : ''}; with the extra loss that is like ${fmtDist(rf.d * Math.pow(10, c.extra / 20))} in the open. Signal ${rf.rssi.toFixed(0)} dBm, the module needs ${rf.sens} dBm at ${c.kbps === 250 ? '250 kbit/s' : c.kbps === 2000 ? '2 Mbit/s' : '1 Mbit/s'}. ${c.kbps === 250 ? 50 : 100} packets a second, each answered; hopping over 8 channels from the binding phrase.`;
  },
  reset() { pkReset('nrf24'); pk.ackq = null; },
  rebind() { Object.assign(pk, { dq: [], cq: [], gIn: [], dIn: [], rcMade: [], air: [], lqSeen: false, lqLow: false, gPrev: null, dPrev: null, ackq: null }); },
  changed() { this.rebind(); radioLinkSetup(); linkLog('↕', 'link', `nRF24L01 now at ${radioCfg.kbps} kbit/s`, 'both ends set up again', 'warn'); },
  step: nrfStep,
  toStack: nrfToStack,
  stats: () => ({ queued: pk.dS ? pk.dS[10] : 0, upQueued: pk.gS ? pk.gS[10] : 0 }),
  extraStats: t => { const w = k => (radio.ev[k] || []).filter(e => e[0] >= t - 5); return { tlmLost: 0, resent: w('resent').reduce((s, e) => s + e[1], 0) }; },
});
