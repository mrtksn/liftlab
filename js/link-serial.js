'use strict';
// A serial line, simulated (a model for link.js, the bookkeeping shared with link-packet.js): whatever carries a UART's
// bytes from the command module to the drone and back. There is no module: the packet layer (runner/fc/plink.h) runs
// at both ends, the real C, and its packets go framed in the byte stream (runner/fc/pframe.h: 0, COBS, 0), the real C
// too (board_wasm.c plink_serial_out, plink_serial_in). This file is the line between: each byte takes 10 bits' time
// at the line's speed, one packet after another each way; some bytes come damaged (a bit flipped), the deframer and
// the signature catch them; one way at a time (half), the two ends' bytes collide if they overlap.
//
// What carries the bytes is the simulator's choice of medium (the boards don't know: radioCfg.medium):
//   0 fibre or a wire: clean, as long as the drone stays within the tether's length (beyond it, the simulator lets the
//     drone fly on and the line is broken: a real tether would hold it back, or snap);
//   1 a laser (a laser diode switched by the TX pin, a photodiode on the RX pin, each way): a narrow beam, so each end
//     must keep it on the other (a tracking mount on the ground, photodiodes all round the drone: taken as given
//     here); the signal falls with the distance squared (6 dB per doubling), 20 dB to spare at 30 m, out to about
//     300 m; a building in the way blocks it;
//   2 infrared LEDs and photodiodes (a remote control's parts): wide beams, no aiming, short: out to about 25 m;
//   3 a radio modem in transparent mode (an HC-12 class 433 MHz module: 100 mW, sensitivity from about −110 dBm at
//     9600 baud to −99 dBm at 115200): each packet goes whole or not at all (the modem's own packets), 6 ms later
//     (it takes the bytes in, sends them on air, hands them out), one way at a time at its best (set half).
// A byte comes damaged with a chance set by the margin: 1 in 1000 at none, 1 in 10 million with 10 dB to spare.

const SL_BAUDS = [19200, 38400, 57600, 115200, 230400, 460800, 921600];
const SL_MEDIA = [[0, 'Fibre or wire (tethered)'], [1, 'Laser and photodiode'], [2, 'Infrared LEDs'], [3, 'Radio modem (433 MHz)']];
// radio_link.c serial_sizing, the same numbers (the telemetry's room on a good line, the packets' rates)
function slSizing(c) {
  const B = c.baud / 10, cl = (x, lo, hi) => Math.min(hi, Math.max(lo, x));
  if (!c.half) {
    const up = cl(0.5 * B / 56, 1, 100), dmax = cl(0.15 * B / 30, 1, 50), room = 0.75 * B - dmax * 30;
    return { up, dmax, room: cl(room * 0.85, 0, 6000) };
  }
  const mtu = Math.floor(cl(B * 0.012, 64, 200)), up = cl(0.75 * B / (56 + mtu + 3), 1, 50);
  return { up, dmax: up, room: cl(up * (mtu - 30) * 0.85, 0, 6000) };
}
const slOk = c => SL_BAUDS.includes(+c.baud) && !(c.half && c.baud < 38400);
const slCfg = c => slOk(c) ? c : { ...c, baud: 115200 };   // (a stored speed the boards wouldn't take: the default)
function slRf() {
  const P = linkPath(), m = radioCfg.medium | 0;
  const rf = { d: P.d, walls: P.walls, margin: 0, cut: '', medium: m };
  if (m === 0) { const L = radioCfg.tether || 50; rf.margin = 30; if (P.d > L) rf.cut = `${P.d.toFixed(0)} m out on a ${L} m tether`; }
  else if (m === 1 || m === 2) {
    const range20 = m === 1 ? 30 : 2.5;                                // (where 20 dB are left)
    rf.margin = 20 + 20 * Math.log10(range20 / P.d) - radioCfg.extra;
    if (P.walls) rf.cut = 'a building in the way';
  } else {
    const sens = -120 + 10 * Math.log10(slCfg(radioCfg).baud / 1000), loss = P.loss - 14.9 - 8 * P.walls;   // (433 MHz: 15 dB less than 2.4 GHz in the open, walls 10 dB, not 18)
    rf.rssi = 20 + 2 - loss; rf.margin = rf.rssi - sens; rf.sens = sens;
  }
  rf.pb = Math.min(0.5, Math.pow(10, Math.max(-7, -3 - 0.4 * rf.margin)));   // a byte damaged
  return rf;
}

function slStep(dt, t, E) {
  if (radio.rfAt < 0 || t - radio.rfAt > 0.02) { radio.rf = slRf(); radio.rfAt = t; slNote(t); }
  const drone = cargo.power ? E && E.drone : null, gnd = E && E.gnd;
  for (const [from, w] of [['gnd', gnd], ['drone', drone]]) {
    if (!w) continue;
    const n = w.plink_serial_out(t); if (!n) continue;
    const raw = Uint8Array.from(new Uint8Array(w.memory.buffer, w.pbuf_ptr(), w.plink_pkt_n()));
    slSend(from, raw, Uint8Array.from(new Uint8Array(w.memory.buffer, w.sbuf_ptr(), n)), t);
  }
  if (pk.air.some(a => a.at <= t + 1e-9)) {
    const due = pk.air.filter(a => a.at <= t + 1e-9).sort((a, b) => a.at - b.at || a.n - b.n);
    pk.air = pk.air.filter(a => a.at > t + 1e-9);
    for (const a of due) pkArrive(a, a.from === 'gnd' ? drone : gnd, t);
  }
  pkEvents(t, gnd, drone);
}
// The line's state changing (cut, back): said once each way in the log.
function slNote(t) {
  const why = radio.rf.cut;
  if (why && !pk.cutSaid) { pk.cutSaid = why; linkLog('↕', 'link', 'the line is broken', why, 'bad'); }
  else if (!why && pk.cutSaid) { linkLog('↕', 'link', 'the line is whole again', '', 'good'); pk.cutSaid = ''; }
}
// A packet's bytes onto the line: after what went before them this way; damaged byte by byte, or lost whole (a radio
// modem's packet), or nothing at all (the line broken); one way at a time, garbled if the other end is talking too.
function slSend(from, raw, wire, t) {
  const c = slCfg(radioCfg), rf = radio.rf, a = pkCarry(from, raw, t), other = from === 'gnd' ? 'drone' : 'gnd';
  const bt = 10 / c.baud, modem = rf.medium === 3, delay = modem ? 0.006 : 0.0002;   // (a radio modem's turnaround; a UART driver's)
  const start = Math.max(t, pk.busy[from]), end = start + wire.length * bt;
  pk.busy[from] = end;
  if (rf.cut) { pkLost(a, t); return; }
  const bytes = Uint8Array.from(wire);
  if (modem && radioRand() >= 1 / (1 + Math.exp(-(rf.margin - 2) / 2))) { pkLost(a, t); return; }   // (the modem's packet lost whole)
  if (!modem) for (let i = 0; i < bytes.length; i++) if (radioRand() < rf.pb) bytes[i] ^= 1 << Math.floor(radioRand() * 8);
  if (c.half || modem) {                                              // one medium both ways (a modem's air): both talking at once, both garbled
    for (const b of pk.air) if (b.from === other && b.end > start && b.start < end) { slGarble(b.wire); slGarble(bytes); pk.collisions = (pk.collisions || 0) + 1; }
  }
  let at = end + delay; at = Math.max(at, pk.last[from]); pk.last[from] = at;
  Object.assign(a, { at, start, end, wire: bytes, rssi: modem ? Math.round(rf.rssi) : 0 });
  pk.air.push(a);
}
function slGarble(b) { for (let i = 1; i < b.length - 1; i += 3) b[i] ^= 0x55; }

RADIO_LINKS.serial = Object.assign(packetModel('espnow', []), {
  label: 'Serial line (laser, fibre, radio modem)', receiver: 'Serial line',
  settings: [
    { key: 'baud', label: 'Line speed', options: SL_BAUDS.map(b => [b, b + ' baud']) },
    { key: 'half', label: 'Direction', options: [[0, 'Both ways at once'], [1, 'One way at a time']] },
    { key: 'medium', label: 'Medium (simulated)', options: SL_MEDIA },
    { key: 'tether', label: 'Tether length', options: [[10, '10 m'], [25, '25 m'], [50, '50 m'], [100, '100 m'], [300, '300 m']], show: c => (c.medium | 0) === 0 },
  ],
  wasm: c => { c = slCfg(c); return [3, c.baud, c.half ? 1 : 0]; },  // radio_link.h RLINK_SERIAL
  room: c => slSizing(slCfg(c)).room,
  roomNote(c) {
    if (!slOk(c)) return `One way at a time needs 38400 baud or more (the channels, the answers and the turnarounds): the boards fly ${slCfg(c).baud} baud.`;
    if ((c.medium | 0) === 3 && !c.half) return 'Most radio modems (HC-12, LoRa serial modules) go one way at a time: set Direction to one way at a time, or the two ends\' packets collide on the air.';
    return '';
  },
  signalNote(c) {
    const rf = radio.rf; if (!rf) return '';
    const s = slSizing(slCfg(c)), pace = `${s.up.toFixed(0)} packets a second up, the telemetry ${Math.round(s.room)} B/s.`;
    const st = rf.cut ? ` Broken: ${rf.cut}.` : '';
    if (rf.medium === 0) return `A fibre or a wire, ${c.tether || 50} m long; the drone ${rf.d.toFixed(0)} m from the handset.${st} ${pace}`;
    if (rf.medium === 3) return `A radio modem: ${rf.d.toFixed(0)} m from the handset${rf.walls ? `, ${rf.walls} building${rf.walls > 1 ? 's' : ''} in the way` : ''}; signal ${rf.rssi.toFixed(0)} dBm, it needs ${rf.sens.toFixed(0)} dBm at ${slCfg(c).baud} baud. ${pace}`;
    const pb = rf.pb < 1e-6 ? 'next to none' : `1 in ${Math.round(1 / rf.pb).toLocaleString()}`;
    return `${rf.medium === 1 ? 'A laser (each end kept on the other)' : 'Infrared'}: ${rf.d.toFixed(0)} m from the handset; ${rf.margin.toFixed(0)} dB to spare, bytes damaged ${pb}.${st} ${pace}`;
  },
  reset() { pkReset('serial'); pk.sized = slKey(radioCfg); pk.cutSaid = ''; },
  changed() {                                                         // the speed or the direction: both ends' packets sized again
    if (pk.sized === slKey(radioCfg)) return;
    pk.sized = slKey(radioCfg); this.rebind(); radioLinkSetup();
    linkLog('↕', 'link', `the line now ${slCfg(radioCfg).baud} baud${radioCfg.half ? ', one way at a time' : ''}`, 'both ends set up again', 'warn');
  },
  step: slStep,
});
const slKey = c => slCfg(c).baud + '/' + (c.half ? 1 : 0);
