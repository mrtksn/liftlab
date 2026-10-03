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
      if (radio.down.length) { radio.downChunks++; if (ok) { const c = radio.down.shift(); radio.downBytes -= c.length; radio.downGot++; radio.txIn.feed(c, f => radio.toGround.push(f)); } }   // (the module hands on whole frames)
    } else {
      const ok = radioRand() < pUp;
      radio.upHist.push(ok ? 1 : 0); if (radio.upHist.length > 100) radio.upHist.shift();
      if (!ok) continue;
      radio.upGot++;
      if (radio.up.length && radio.k % 2) {                          // this packet carries 5 bytes of a command
        const c = radio.up[0]; c.got.push(...c.bytes.subarray(c.at, c.at + 5)); c.at += 5;
        if (c.at >= c.bytes.length) { radio.toBoard.push(Uint8Array.from(c.got)); radio.up.shift(); }
      } else if (radio.txCh) radio.toBoard.push(crsfRcFrame(radio.txCh));   // (nothing from the command module yet: nothing to send)
    }
  }
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
    while (radio.downBytes + f.length > 512 && radio.down.length) {   // the receiver's queue is full: the oldest frame goes
      let n = 0; const first = radio.down[0].frame; while (radio.down.length && radio.down[0].frame === first) { n += radio.down[0].length; radio.down.shift(); }
      radio.downBytes -= n; radio.downDropped++;
    }
    radio.downFrames++;
    for (let i = 0; i < f.length; i += 5) { const c = f.subarray(i, i + 5); c.frame = radio.downFrames; radio.down.push(c); radio.downBytes += c.length; }
  });
}
// What the command module wrote to the transmitter module: its channel frames (the latest goes up in each uplink
// packet) and its commands (queued, and sent 5 bytes at a time in place of some channel packets).
function radioFromGround(bytes) {
  radio.fromGround.feed(bytes, f => {
    if (f[2] === CRSF.RC) radio.txCh = crsfRcRead(f.subarray(3, f.length - 1));
    else if (f[2] === CRSF.EXT) radio.up.push({ bytes: f, at: 0, got: [] });
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
