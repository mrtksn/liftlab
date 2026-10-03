'use strict';
// The pilot's radio link, simulated: an ExpressLRS transmitter module on the handset and a receiver on the drone,
// wired by CRSF to the board that runs the telemetry task (runner/fc/tlm_core.h, rc_core.h). What the drone's code
// sees is what a real receiver gives it: channel frames when uplink packets get through, link statistics ten times a
// second, the ground station's commands; and what it sends the receiver goes down in the telemetry slots.
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
// The handset is you: the keys and the simulator's pilot (arm, take off) become stick positions and switches.

const ELRS_RATES = { 50: -115, 150: -112, 250: -108, 500: -105 };   // receiver sensitivity [dBm]
const ELRS_RATIOS = [2, 4, 8, 16, 32, 64, 128];
const ELRS_POWERS = [10, 25, 100, 250, 500, 1000];   // [mW]
const radioCfg = { rate: 250, ratio: 4, power: 100, extra: 0 };
const radio = {};
function radioReset() {
  Object.assign(radio, {
    t: 0, nextPkt: 0, k: 0, seed: 0x2545F491,
    down: [], downBytes: 0, downFrames: 0, downDropped: 0, groundIn: crsfParser(), fromDrone: crsfParser(),
    up: [], upSeq: 0, toBoard: [], nextStats: 0,
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
      if (radio.down.length) { radio.downChunks++; if (ok) { const c = radio.down.shift(); radio.downBytes -= c.length; radio.downGot++; gsBytes(c, t); } }
    } else {
      const ok = radioRand() < pUp;
      radio.upHist.push(ok ? 1 : 0); if (radio.upHist.length > 100) radio.upHist.shift();
      if (!ok) continue;
      radio.upGot++;
      if (radio.up.length && radio.k % 2) {                          // this packet carries 5 bytes of a command
        const c = radio.up[0]; c.got.push(...c.bytes.subarray(c.at, c.at + 5)); c.at += 5;
        if (c.at >= c.bytes.length) { radio.toBoard.push(Uint8Array.from(c.got)); radio.up.shift(); }
      } else radio.toBoard.push(crsfRcFrame(handsetChannels(t)));
    }
  }
  const lq = h => h.length ? 100 * h.reduce((a, b) => a + b, 0) / h.length : 0;
  radio.lqUp = lq(radio.upHist); radio.lqDown = lq(radio.downHist);
  radio.rssiUp = r.rssi + (radioRand() - 0.5) * 2; radio.rssiDown = r.rssi - 1 + (radioRand() - 0.5) * 2;
  if (t >= radio.nextStats) {                                        // link statistics to the drone, 10 times a second
    radio.nextStats = t + 0.1;
    if (radio.lqUp > 0) radio.toBoard.push(crsfLinkStatsFrame({ upRssi: radio.rssiUp, upLq: radio.lqUp, upSnr: r.snr, downRssi: radio.rssiDown, downLq: radio.lqDown, downSnr: r.snr - 1, rfMode: [50, 150, 250, 500].indexOf(radioCfg.rate), power: radioCfg.power }));
    gs.link = { upRssi: radio.rssiUp, upLq: radio.lqUp, upSnr: r.snr, downRssi: radio.rssiDown, downLq: radio.lqDown, d: r.d, walls: r.walls, t };
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
// A ground-station command, up the link.
function radioCommand(cmd, values) { radio.up.push({ bytes: crsfCmdFrame(cmd, values), at: 0, got: [] }); }
function radioHold() { radio.holdUntil = radio.t + 0.3; }
function radioHome() { radio.homeUntil = radio.t + 0.3; }

// The handset: 16 channels (rc_core.h's order), from the keys and the simulator's pilot.
function handsetChannels(t) {
  const P = brt.pilot, ch = new Array(16).fill(-1), k = c => isHeld(c) ? 1 : 0;
  if (hasTask('nav')) {                                              // position mode: the sticks ask for velocity, centred
    ch[0] = k('right') - k('left'); ch[1] = k('fwd') - k('back'); ch[2] = k('up') - k('down'); ch[3] = k('yawR') - k('yawL');
  } else {                                                           // angle mode: the sticks as they are
    const s = stickCommand(); ch[0] = s.roll; ch[1] = s.pitch; ch[2] = 2 * s.throttle - 1; ch[3] = -s.yaw;
  }
  ch[4] = P.arm ? 1 : -1; ch[5] = { gentle: -1, normal: 0, sport: 1 }[pilot.level] ?? 0; ch[6] = P.fly ? 1 : -1;
  ch[7] = t < radio.holdUntil ? 1 : -1; ch[8] = t < radio.homeUntil ? 1 : -1;
  radio.ch = ch;
  return ch;
}

/* ───────── the ground station: what came down ───────── */
const gs = { v: {}, at: {}, log: [], frames: 0, bytes: 0, track: [], link: null };
function gsReset() { Object.assign(gs, { v: {}, at: {}, log: [], frames: 0, bytes: 0, track: [], link: null, rate: [], lastRateT: 0, home: null }); }
function gsBytes(bytes, t) {
  gs.bytes += bytes.length;
  radio.groundIn.feed(bytes, f => {
    const d = crsfDecode(f); gs.frames++; if (!d) return;
    if (d.kind === 'text') { gs.log.unshift({ t, sev: d.sev, text: d.text }); if (gs.log.length > 60) gs.log.length = 60; return; }
    gs.v[d.kind] = d; gs.at[d.kind] = t;
    if (d.kind === 'gps' && d.sats > 0) { gs.track.push([d.lat, d.lon]); if (gs.track.length > 400) gs.track.shift(); }
    if (d.kind === 'pos') { gs.trackXY = gs.trackXY || []; gs.trackXY.push([d.x, d.y]); if (gs.trackXY.length > 400) gs.trackXY.shift(); }
  });
}
