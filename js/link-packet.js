'use strict';
// The packet links, simulated (models for link.js): ESP-NOW (an ESP32 at each end, talking directly) and Wi-Fi (UDP
// over 802.11n; the drone makes the network, an access point, or joins one, a station). There are no modules: the
// packet layer that does a module's part (runner/fc/plink.h) runs at both ends, the real C, in the command module's
// instance and in the receiver board's. The stacks' CRSF bytes go into the plinks and come out of them unchanged
// (boards.js radioTick, link.js radioStackOut); each 1 ms step, each end's plink makes the packet due
// (plink_air_out), and this file is the air between them: whether a packet gets through, when it arrives, and, for
// Wi-Fi, whether there is a network at all. What arrives goes to the other end's plink (plink_air_in), which checks
// its signature (the binding phrase) and takes it or not.
//
// The air, per kind:
//   - ESP-NOW: 802.11b frames at 1 Mbit/s (sensitivity about −98 dBm), or Espressif's long-range mode at 0.5 Mbit/s
//     (about −105 dBm, twice the time on air); 20 dBm (100 mW) out. A packet's time on air from its size (a 192 µs
//     preamble, 43 bytes of 802.11 and vendor framing at the rate); the MAC tries a unicast frame up to 3 times,
//     each acknowledged; 0.4–1.2 ms more through the driver and the callbacks: about 1–2 ms end to end. No
//     association: it works the moment the signal does.
//   - Wi-Fi: 802.11n, the rate picked from the signal (6.5 Mbit/s, sensitivity about −92 dBm, up to 65 Mbit/s at
//     −74 dBm); 4 tries; 2–5 ms through the two network stacks (1 ms more through a router, as a station), and now
//     and then (0.3% of packets) a stall of 15–80 ms (a busy channel, a background scan, power saving) that holds
//     back what follows. And association: the station hears the access point's beacons (every 102.4 ms, at the
//     lowest rate); after 3 s without one it disconnects, and once it hears one again it takes 1–3 s to join
//     (scan, authenticate, DHCP). Until it has joined, nothing goes through either way.
//   - Bluetooth LE: a connection between the two ESP32s (the drone advertises, the command module connects): the
//     packets wait for the next connection event (every 7.5 ms), each try is acknowledged by the link layer, one not
//     acknowledged goes again at the next event (up to 8); LE 1M PHY, sensitivity about −96 dBm, 9 dBm out. The
//     connection drops after 1 s with nothing through (the supervision timeout); the drone advertises again (every
//     40 ms), and once the command module hears it, it connects again in 0.15–0.5 s. The advertising carries a mark
//     from the drone's binding phrase: the command module connects only to its own phrase's.
// All: a packet gets through with a probability set by its margin over the sensitivity at its rate, on the path
// link.js works out (linkPath); the drone's transmissions are 1 dB worse (as in link-elrs.js); with no power on the
// drone, nothing at all. Each direction is one transmitter's queue: packets arrive in the order they went.

const PK_MCS = [[6.5, -92], [13, -89], [19.5, -87], [26, -84], [39, -80], [52, -76], [58.5, -75], [65, -74]];   // 802.11n HT20 rates [Mbit/s] and their sensitivity [dBm]
const PK_CHANNELS = Array.from({ length: 13 }, (_, i) => [i + 1, 'Channel ' + (i + 1)]);
const PK_AIR = {
  espnow: {
    label: 'ESP-NOW (ESP32 to ESP32)', short: 'ESP-NOW', code: 1, tx: 20, tries: 3, over: 43, difs: 50e-6, slot: 20e-6, cw: 31, ack: 314e-6, ackWait: 340e-6, assoc: false,
    phy: c => c.lr ? { mbps: 0.5, sens: -105, pre: 384e-6, name: '0.5 Mbit/s (long range)' } : { mbps: 1, sens: -98, pre: 192e-6, name: '1 Mbit/s' },
    lat: () => 0.0004 + 0.0008 * radioRand(), stall: 0,
  },
  wifi: {
    label: 'Wi-Fi (UDP)', short: 'Wi-Fi', code: 2, tx: 20, tries: 4, over: 66, difs: 34e-6, slot: 9e-6, cw: 15, ack: 60e-6, ackWait: 75e-6, assoc: true,
    phy: (c, margin) => {   // the fastest rate with 6 dB to spare; the lowest when none has (the margin is over the lowest's sensitivity)
      const rssi = margin + PK_MCS[0][1]; let m = PK_MCS[0];
      for (const r of PK_MCS) if (rssi - r[1] >= 6) m = r;
      return { mbps: m[0], sens: m[1], pre: 36e-6, name: m[0] + ' Mbit/s' };
    },
    lat: c => 0.002 + 0.003 * radioRand() + (c.sta ? 0.001 : 0), stall: 0.003, beacon: 0.1024, lose: 3, join: [1, 3],
  },
  ble: {   // a connection: the packets go at its events (every 7.5 ms), each acknowledged, again at the next if not
    label: 'Bluetooth LE (ESP32 to ESP32)', short: 'Bluetooth LE', code: 5, tx: 9, tries: 8, over: 17, difs: 0, slot: 0, cw: 0, ack: 150e-6, ackWait: 0.0075, assoc: true,
    phy: () => ({ mbps: 1, sens: -96, pre: 0, name: '1 Mbit/s (LE 1M)' }),
    lat: () => 0.0005 + 0.0075 * radioRand(), stall: 0, beacon: 0.04, lose: 1, join: [0.15, 0.5],   // (the next connection event; advertising every 40 ms, a 1 s supervision timeout, scan and connect)
  },
};
let pk = {};   // the packet model's own state (radio, in link.js, holds what every link shares)
const pkP = margin => 1 / (1 + Math.exp(-(margin - 2) / 1.3));   // one try gets through (802.11's error rate falls off faster than LoRa's)
const pkKey = f => String.fromCharCode.apply(null, f);
const pkReliable = f => f[2] === CRSF.EXT && f[1] >= 3 && (f[3] === CRSF.EXT_TEXT || f[3] === CRSF.EXT_CMD);   // what plink sends reliably (plink.c reliable)
// A packet's records (plink.h: a 16-byte header, then records, then an 8-byte tag): { rel, num, f } each.
function plinkRecords(p) {
  const out = [], end = p.length - 8; let k = 16;
  while (k + 1 < end) {
    const kind = p[k++]; let num = -1; if (kind === 2) num = p[k++];
    if (k + 2 > end) break;
    const len = p[k + 1] + 2; if (len < 4 || k + len > end) break;
    out.push({ rel: kind === 2, num, f: p.subarray(k, k + len) }); k += len;
  }
  return out;
}

function pkRf() {
  const P = linkPath(), A = PK_AIR[radioCfg.kind] || PK_AIR.espnow, rssi = A.tx + 4 - P.loss, low = radioCfg.kind === 'wifi' ? PK_MCS[0][1] : A.phy(radioCfg).sens;
  return { d: P.d, walls: P.walls, rssi, margin: rssi - low };
}

function packetModel(kind, settings) {
  const A = PK_AIR[kind];
  return {
    label: A.label, receiver: kind === 'wifi' ? 'Wi-Fi radio' : kind === 'ble' ? 'Bluetooth LE radio' : 'ESP-NOW radio', packets: true, settings,
    wasm: c => kind === 'wifi' ? [2, c.sta ? 1 : 0, c.channel] : kind === 'ble' ? [5, 0, 0] : [1, c.channel, c.lr ? 1 : 0],   // radio_link.h RLINK_ESPNOW, RLINK_WIFI, RLINK_BLE
    room: () => 6000,                                                 // radio_link.c rlink_budget for a packet link
    roomNote(c) {                                                     // what the hardware can't do (the simulator flies it anyway)
      const b = typeof boardOf === 'function' ? boardOf('tlm') : null, K = b && BOARD_KINDS[b.kind];
      if (kind === 'ble') {
        const g = computers().ground, gk = g && g.kind;
        if (b && b.kind !== 's3' && b.kind !== 'c3') return `Bluetooth LE needs an ESP32-S3 or C3 on the drone: ${b.name} is a ${K ? K.label : b.kind}, whose firmware here has no Bluetooth (the ESP32's controller takes memory the flight code needs; a Pi isn't supported). The simulator flies it anyway.`;
        if (gk && gk !== 's3' && gk !== 'c3' && gk !== 'mac') return 'Bluetooth LE: the command module needs an ESP32-S3 or C3 too (a Mac or PC flies through one on USB).';
        return '';
      }
      if (kind === 'espnow' && K && !K.mcu) return `ESP-NOW needs an ESP32 at each end: the receiver's board here, ${b.name}, is a ${K.label}, which has no ESP-NOW (the simulator flies it anyway). Put the Telemetry & radio task on an ESP32, or use Wi-Fi: it works with an ESP32 or a Pi.`;
      return '';
    },
    signalNote(c) {
      const rf = radio.rf; if (!rf) return '';
      const ph = A.phy(c, rf.margin), low = kind === 'wifi' ? `${PK_MCS[0][1]} dBm at ${PK_MCS[0][0]} Mbit/s, its lowest rate (now ${ph.name})` : `${ph.sens} dBm at ${ph.name}`;
      const S = pk.assoc && (kind === 'ble' ? { up: 'connected', join: 'connecting', down: pk.assoc.foreign > pk.assoc.heard ? 'not connecting: the drone advertising has another binding phrase' : 'disconnected: the drone advertising, not heard' } : { up: 'connected', join: 'joining the network (scan, authenticate, DHCP)', down: 'disconnected: no beacons' })[pk.assoc.state];
      const g = computers().ground, gk = BOARD_KINDS[g.kind];
      const who = kind === 'ble' ? ' The drone advertises; the command module connects (a 7.5 ms connection interval).' : kind === 'espnow' ? (gk && !gk.mcu ? ` The command module (${gk.label}) talks ESP-NOW through an ESP32 on USB.` : '') : ` The drone ${c.sta ? 'joins a network; the command module joins the same one' : 'makes the network (access point); the command module joins it'}.`;
      return `Now ${rf.d.toFixed(0)} m from the handset${rf.walls ? `, ${rf.walls} building${rf.walls > 1 ? 's' : ''} in the way` : ''}; with the extra loss that is like ${fmtDist(rf.d * Math.pow(10, c.extra / 20))} in the open. Signal ${rf.rssi.toFixed(0)} dBm, the ${kind === 'wifi' ? 'radio' : 'ESP32'} needs ${low}.${S ? ` ${A.short}: ${S}.` : ''}${who}`;
    },
    reset() { pkReset(kind); },
    resume() { },                                                     // (reset starts it at radio.t)
    rebind() { Object.assign(pk, { dq: [], cq: [], gIn: [], dIn: [], rcMade: [], air: [], lqSeen: false, lqLow: false, gPrev: null, dPrev: null }); },   // new plinks: what the old ones held is gone
    changed() {                                                       // a setting changed, both ends at once: a new network to join (Wi-Fi)
      if (!pk.assoc || pk.assoc.state === 'down') return;
      Object.assign(pk.assoc, { state: 'down', since: radio.t, heard: -1e9 });
      linkLog('↕', 'link', 'Wi-Fi: joining the network again', 'its settings changed', 'warn');
    },
    step: pkStep,
    fromDrone(f, id) { pk.dq.push({ id, key: pkKey(f), rel: pkReliable(f), got: false }); },   // (the plink has it: boards.js passed it the bytes)
    command(f) { pk.cq.push({ key: pkKey(f), t0: radio.t, desc: cmdDesc(f), tries: 0, lost: 0, got: false }); },   // (logged when it reaches the drone)
    toStack: pkToStack,
    connected: () => !!(pk.gS && pk.gS[12]),
    stats: () => ({ queued: pk.dS ? pk.dS[10] : 0, upQueued: pk.gS ? pk.gS[11] : 0 }),
    extraStats: t => { const w = k => (radio.ev[k] || []).filter(e => e[0] >= t - 5); return { tlmLost: w('tlmLost').length, resent: w('resent').reduce((s, e) => s + e[1], 0) }; },   // (frames lost with their packets; reliable frames sent again, both ends)
  };
}

RADIO_LINKS.espnow = packetModel('espnow', [
  { key: 'channel', label: 'Wi-Fi channel', options: PK_CHANNELS },
  { key: 'lr', label: 'Long range', options: [[0, 'Off (1 Mbit/s)'], [1, 'On (0.5 Mbit/s, further)']] },
]);
RADIO_LINKS.ble = packetModel('ble', []);
RADIO_LINKS.wifi = packetModel('wifi', [
  { key: 'sta', label: 'Network', options: [[0, 'Drone makes it (AP)'], [1, 'Drone joins one']] },
  { key: 'channel', label: 'Channel', options: PK_CHANNELS, show: c => !c.sta },
]);

function pkReset(kind) {
  const t = radio.t;
  Object.assign(pk, {
    kind, dq: [], cq: [], gIn: [], dIn: [], rcMade: [], air: [], busy: { gnd: t, drone: t }, last: { gnd: t, drone: t }, stallUntil: { gnd: -1, drone: -1 },
    nextStats: t, gS: null, dS: null, gPrev: null, dPrev: null, rej: { gnd: 0, drone: 0 }, rejLog: { gnd: -1e9, drone: -1e9 }, dropRun: null,
    lastUpOk: t, upGap: false, upEver: false, lastDownOk: t, downGap: false, downEver: false, lqLow: false, lqSeen: false,
    assoc: PK_AIR[kind] && PK_AIR[kind].assoc ? { state: 'down', since: t - 1, heard: -1e9, nextBeacon: t + 0.05, joinAt: 0, joinT0: 0, ever: false } : null,
  });
}

// Each 1 ms step: the network (Wi-Fi), the packets due at each end, what arrives, the ends' numbers.
function pkStep(dt, t, E) {
  if (radio.rfAt < 0 || t - radio.rfAt > 0.02) { radio.rf = pkRf(); radio.rfAt = t; }
  const drone = cargo.power ? E && E.drone : null, gnd = E && E.gnd;   // (no power on the drone: its radio is dark)
  if (pk.assoc) pkAssoc(t, drone);
  if (gnd) { const n = gnd.plink_air_out(t); if (n) pkSend('gnd', Uint8Array.from(new Uint8Array(gnd.memory.buffer, gnd.pbuf_ptr(), n)), t); }
  if (drone) { const n = drone.plink_air_out(t); if (n) pkSend('drone', Uint8Array.from(new Uint8Array(drone.memory.buffer, drone.pbuf_ptr(), n)), t); }
  // what has finished its time on air (each direction in order) arrives
  if (pk.air.some(a => a.at <= t + 1e-9)) {
    const due = pk.air.filter(a => a.at <= t + 1e-9).sort((a, b) => a.at - b.at || a.n - b.n);
    pk.air = pk.air.filter(a => a.at > t + 1e-9);
    for (const a of due) pkArrive(a, a.from === 'gnd' ? drone : gnd, t);
  }
  pkEvents(t, gnd, drone);
}
// Ten times a second the ends' numbers; the link's events: a direction quiet (after it first worked), the uplink's
// quality, frames dropped.
function pkEvents(t, gnd, drone) {
  if (t >= pk.nextStats - 1e-9) { pk.nextStats = t + 0.1; pkPoll(t, gnd, drone); }
  if (pk.upEver && !pk.upGap && t - pk.lastUpOk > 0.5) { pk.upGap = true; linkLog('↑', 'link', 'uplink lost', 'no packets for 0.5 s', 'bad'); }
  if (pk.downEver && !pk.downGap && t - pk.lastDownOk > 1) { pk.downGap = true; linkLog('↓', 'link', 'telemetry lost', 'nothing for 1 s', 'bad'); }
  const D = pk.dropRun;
  if (D && D.n && t - D.last > 0.5) { linkLog('↓', 'drop', `${D.n} frames dropped`, `${(D.last - D.t0).toFixed(1)} s · the drone's send queue full`, 'warn'); pk.dropRun = null; }
}

// A packet one end's plink made: through the air (or not), to arrive at the other end.
let pkN = 0;
function pkSend(from, bytes, t) {
  const A = PK_AIR[pk.kind], up = from === 'gnd', a = pkCarry(from, bytes, t);
  // the network (Wi-Fi): not joined, nothing goes
  if (pk.assoc && pk.assoc.state !== 'up') { pkLost(a, t); return; }
  const rf = radio.rf, margin = up ? rf.margin : rf.margin - 1, ph = A.phy(radioCfg, margin), rssi = margin + (pk.kind === 'wifi' ? PK_MCS[0][1] : ph.sens);
  let tt = Math.max(t, pk.busy[from]), ok = false;
  for (let k = 0; k < A.tries; k++) {                                 // the MAC: wait for the air, send, an ack or not
    tt += A.difs + A.slot * Math.floor(radioRand() * ((A.cw + 1) << Math.min(k, 3)));
    tt += ph.pre + (bytes.length + A.over) * 8 / (ph.mbps * 1e6);
    if (radioRand() < pkP(rssi - ph.sens)) { ok = true; tt += A.ack; break; }
    tt += A.ackWait;
  }
  pk.busy[from] = tt;                                                 // (the next waits for this one to be done)
  if (!ok) { pkLost(a, t); return; }
  let at = tt + A.lat(radioCfg);
  if (A.stall && radioRand() < A.stall) pk.stallUntil[from] = at + 0.015 + 0.065 * radioRand();   // a stall: this and what follows wait
  if (at < pk.stallUntil[from]) at = pk.stallUntil[from];
  at = Math.max(at, pk.last[from]); pk.last[from] = at;
  a.at = at; a.rssi = Math.round(rssi + (radioRand() - 0.5) * 2);
  pk.air.push(a);
}
// A packet one end's plink made, before the air has it: what it carries, for the log and the link's numbers.
function pkCarry(from, bytes, t) {
  const up = from === 'gnd', recs = plinkRecords(bytes), a = { from, bytes, n: ++pkN, recs: [], cmds: [], made: null, at: 0 };
  // what it carries, for the log: the channels (their age), the commands and messages (how many packets each took),
  // the telemetry frames (which of the drone's are in it; the ones before them that aren't were dropped from its queue)
  for (const r of recs) {
    const key = pkKey(r.f);
    if (up) {
      if (r.f[2] === CRSF.RC) a.made = radio.txChT;
      else if (r.rel) { const c = pk.cq.find(x => !x.got && x.key === key); if (c) { c.tries++; a.cmds.push(c); } }
    } else if (r.rel) {
      const e = pk.dq.find(x => x.rel && !x.got && x.key === key); if (e) { const m = radio.meta.get(e.id); if (m) m.tries++; a.recs.push(e); }
    } else {
      const i = pk.dq.findIndex(x => !x.rel && x.key === key); if (i < 0) continue;
      const gone = [];
      pk.dq = pk.dq.filter((x, j) => { if (j === i) { a.recs.push(x); return false; } if (j < i && !x.rel) { gone.push(x); return false; } return true; });
      for (const x of gone) pkGone(x, t);
    }
  }
  return a;
}
// A packet that didn't get through: its once-frames are gone; its commands and messages go again in the next ones.
function pkLost(a, t) {
  linkEv(a.from === 'gnd' ? 'upLost' : 'downLost', t);
  for (const c of a.cmds) c.lost++;
  for (const e of a.recs) if (!e.rel) { const m = radio.meta.get(e.id); radio.meta.delete(e.id); if (m) linkEv('tlmLost', t); }
}
// A packet at the other end: its plink checks it and takes it, or not.
function pkArrive(a, w, t) {
  if (!w) { pkLost(a, t); return; }                                   // (the drone's radio is dark)
  let took;
  if (a.wire) { new Uint8Array(w.memory.buffer, w.sbuf_ptr(), a.wire.length).set(a.wire); took = w.plink_serial_in(a.wire.length, t); }   // (a serial line: its bytes through the deframer)
  else { new Uint8Array(w.memory.buffer, w.pbuf_ptr(), a.bytes.length).set(a.bytes); took = w.plink_air_in(a.bytes.length, a.rssi, t); }
  if (!took) { pkLost(a, t); return; }                               // (a bad signature, a replay, damaged on the line: counted in its numbers; pkPoll says)
  if (a.from === 'gnd') {
    linkEv('upOk', t);
    if (pk.upGap) linkLog('↑', 'link', 'uplink back', `after ${(t - pk.lastUpOk).toFixed(1)} s`, 'good');
    pk.upGap = false; pk.lastUpOk = t; pk.upEver = true;
    if (a.made != null) pk.rcMade.push(a.made);
    for (const c of a.cmds) if (!c.got) { c.got = true; pk.dIn.push(c); }
  } else {
    linkEv('downOk', t);
    if (pk.downGap) linkLog('↓', 'link', 'telemetry back', `after ${(t - pk.lastDownOk).toFixed(1)} s`, 'good');
    pk.downGap = false; pk.lastDownOk = t; pk.downEver = true;
    for (const e of a.recs) { if (e.rel) { if (e.got) continue; e.got = true; const i = pk.dq.indexOf(e); if (i >= 0) pk.dq.splice(i, 1); } pk.gIn.push({ e, t }); }
  }
}
// A telemetry frame the drone's plink dropped (its queue full: the oldest go).
function pkGone(e, t) {
  const m = radio.meta.get(e.id); if (!m) return; radio.meta.delete(e.id);
  linkEv('tlmDrop', t);
  const D = pk.dropRun || (pk.dropRun = { n: 0, t0: t });
  if (!D.n) linkLog('↓', 'drop', 'dropping frames', 'the drone\'s send queue full (1024 B)', 'warn');
  D.n++; D.last = t;
}
// A frame an end's plink gave its stack (link.js radioStackOut): the channels and commands reaching the drone, the
// telemetry and messages reaching the command module. (The link statistics each plink makes are its own.)
function pkToStack(end, f, t) {
  if (f[2] === CRSF.LINK_STATS) return;
  if (end === 'drone') {
    if (f[2] === CRSF.RC) {
      const made = pk.rcMade.length ? pk.rcMade.shift() : null, lat = made != null ? (t - made) * 1000 : 0, ch = crsfRcRead(f.subarray(3, f.length - 1));
      linkEv('chSent', t, lat); linkChannels(ch, t, f, lat);
    } else if (f[2] === CRSF.EXT) {
      const key = pkKey(f), i = pk.dIn.findIndex(c => c.key === key); if (i < 0) return;
      const c = pk.dIn.splice(i, 1)[0], ms = Math.round((t - c.t0) * 1000), j = pk.cq.indexOf(c); if (j >= 0) pk.cq.splice(j, 1);
      linkEv('cmdOut', t, (t - c.t0) * 1000);
      linkLog('↑', 'cmd', c.desc, `${ms} ms · ${c.tries} pk${c.lost ? ` · ${c.lost} lost` : ''}`, '', f);
    }
    return;
  }
  const key = pkKey(f), i = pk.gIn.findIndex(x => x.e.key === key);
  if (i >= 0) { const x = pk.gIn.splice(i, 1)[0]; linkDown(f, t, x.e.id, 1e9); }   // (resent: the packets a message took, but its first)
  while (pk.gIn.length && t - pk.gIn[0].t > 0.2) pk.gIn.shift();     // (one the stack never got: plink's buffer for it was full)
}
// Ten times a second: each end's numbers (plink_stats), the link quality each hears, what they rejected.
function pkPoll(t, gnd, drone) {
  const rd = w => { const n = w.plink_stats(t); return Array.from(new Float32Array(w.memory.buffer, w.fr_ptr(), n)); };
  pk.gS = gnd ? rd(gnd) : null; pk.dS = drone ? rd(drone) : null;
  radio.lqUp = pk.dS ? pk.dS[7] : 0; radio.lqDown = pk.gS ? pk.gS[7] : 0;
  for (const [end, S, P] of [['drone', pk.dS, pk.dPrev], ['gnd', pk.gS, pk.gPrev]]) {
    if (!S || !P) continue;
    const bad = S[2] - P[2], old = S[3] - P[3] + S[4] - P[4], dropped = end === 'gnd' ? S[5] - P[5] : 0, again = S[6] - P[6];
    if (again > 0) linkEv('resent', t, again);
    if (bad > 0) {
      pk.rej[end] += bad;
      if (t - pk.rejLog[end] >= 5) { linkLog(end === 'drone' ? '↑' : '↓', 'drop', `${pk.rej[end]} packet${pk.rej[end] > 1 ? 's' : ''} rejected`, `by ${end === 'drone' ? 'the drone' : 'the command module'}: a bad signature · ${pk.kind === 'serial' ? 'damaged on the line, or a different binding phrase' : 'a different binding phrase?'}`, pk.kind === 'serial' ? 'warn' : 'bad'); pk.rej[end] = 0; pk.rejLog[end] = t; }
    }
    if (old > 0) linkLog(end === 'drone' ? '↑' : '↓', 'drop', `${old} packet${old > 1 ? 's' : ''} rejected`, 'a replay, or the other end\'s old session', 'warn');
    if (dropped > 0) { linkEv('cmdDrop', t); linkLog('↑', 'drop', `${dropped} command${dropped > 1 ? 's' : ''} dropped`, `the command module had ${PK_RQ} waiting`, 'bad'); }
  }
  pk.gPrev = pk.gS; pk.dPrev = pk.dS;
  if (radio.lqUp > 0) pk.lqSeen = true;
  if (pk.lqSeen && radio.lqUp < 50 !== pk.lqLow) { pk.lqLow = radio.lqUp < 50; linkLog('↑', 'link', `LQ ${Math.round(radio.lqUp)}%`, pk.lqLow ? 'below 50%' : 'above 50% again', pk.lqLow ? 'warn' : 'good'); }
}
const PK_RQ = 16;   // plink.h PLINK_RQ: reliable frames waiting
// Wi-Fi's network: the station hears the access point's beacons; it leaves after 3 s without one, and joins again
// (1–3 s) once it hears one. Bluetooth LE's connection the same way: its events (taken here every 40 ms) or, down, the
// drone's advertising; 1 s without, it drops; heard again, it connects in 0.15–0.5 s.
function pkAssoc(t, drone) {
  const S = pk.assoc, A = PK_AIR[pk.kind], W = A.short;
  while (S.nextBeacon <= t + 1e-9) {
    const tb = S.nextBeacon; S.nextBeacon += A.beacon;
    const margin = radioCfg.sta ? radio.rf.margin : radio.rf.margin - 1;   // (a station drone hears the router; an access point drone is heard by the command module)
    if (!drone || radioRand() >= pkP(margin)) continue;
    if (pk.kind === 'ble' && radio.phrase && radio.phrase.gnd !== radio.phrase.drone) {   // (its advertising carries its phrase's mark: another, not ours)
      S.foreign = tb;
      if (!(t - (S.foreignLog ?? -1e9) < 10)) { S.foreignLog = t; linkLog('↕', 'link', 'Bluetooth LE: a drone advertising, not ours', 'its mark is from another binding phrase: not connecting', 'bad'); }
      continue;
    }
    S.heard = tb;
  }
  if (S.state === 'up') {
    if (t - S.heard > A.lose) { S.state = 'down'; S.since = t; linkLog('↕', 'link', `${W} disconnected`, pk.kind === 'ble' ? 'nothing through for 1 s (the supervision timeout)' : 'no beacon for 3 s', 'bad'); }
  } else if (S.state === 'down') {
    if (S.heard > S.since) { S.state = 'join'; S.joinT0 = t; S.joinAt = t + A.join[0] + (A.join[1] - A.join[0]) * radioRand(); linkLog('↕', 'link', S.ever ? `${W} reconnecting` : `${W} connecting`, pk.kind === 'ble' ? 'the drone\'s advertising heard: connecting, the MTU, the service' : 'scan, authenticate, DHCP', 'warn'); }
  } else if (t - S.heard > 1) { S.state = 'down'; S.since = t; linkLog('↕', 'link', `${W} didn't connect`, 'lost the other end again', 'bad'); }
  else if (t >= S.joinAt) { S.state = 'up'; S.ever = true; linkLog('↕', 'link', `${W} connected`, `after ${(t - S.joinT0).toFixed(1)} s`, 'good'); }
}
