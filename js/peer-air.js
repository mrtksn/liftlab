'use strict';
// Drones talking to each other, simulated: the air between the fleet's drones for their peer links
// (runner/fc/peer.h: finding each other by beacons, a session with each one found). Each drone's end is the real C,
// in its flight controller's WebAssembly (board_wasm.c peer_*), as an ESP32 runs it beside its flight code; this file
// is ESP-NOW between them:
//   - a beacon goes to everyone; a packet to one drone goes to it alone, the radio trying up to 4 times;
//   - each try gets through with a chance set by the margin over the sensitivity (−98 dBm, 1 Mbit/s; 20 dBm out,
//     2 dBi antennas each end, free space for the first 10 m and then falling faster (3.3 decades: the ground, the
//     frames, small antennas), 18 dB a building in the way), so about 600 m in the open;
//   - 1–3 ms on the way; nothing for a drone without power, or with its peer link off, or in another fleet (its
//     packets don't check: peer.c drops them).
// The air is the world's (peerAir, not any drone's); each drone's step (boardsControl) sends what its end has due
// and takes what reached it. Its own state (radio.peer: who it's heard, for the link log) is the drone's.
const peerAir = { q: [], n: 0, seed: seedHash(worldSeeds.radio, 'peer'), starts: 0 };
function peerAirReset() { Object.assign(peerAir, { q: [], n: 0, seed: seedHash(worldSeeds.radio, 'peer'), starts: 0 }); }   // (the whole world starting again)
const peerRand = () => { let s = peerAir.seed; s ^= s << 13; s >>>= 0; s ^= s >>> 17; s ^= s << 5; s >>>= 0; peerAir.seed = s; return s / 4294967296; };   // (its own: the drones' radio models keep theirs)
const PEER_SENS = -98, PEER_TX = 20;
const peerOn = () => !!radioCfg.peers;
// The drone's node number (the same at every start, as an ESP32's from its MAC address) and its radio address.
function peerId(d) { let h = 2166136261; for (const ch of String(d.id)) { h ^= ch.charCodeAt(0); h = Math.imul(h, 16777619); } return (h >>> 0) || 1; }
const peerAddr = id => Uint8Array.of(0x02, 0x4C, id & 255, (id >>> 8) & 255, (id >>> 16) & 255, id >>> 24);
const peerBoard = () => typeof boardOf === 'function' ? boardOf('core') : null;
// As the boards start (boardsStart, each reset: a new session, as a power-on): the peer end on the flight controller.
function peerSetup() {
  radio.peer = { seen: new Map(), last: null };
  const b = peerBoard(), w = b && brt.inst.get(b.id), d = typeof fleet !== 'undefined' ? fleet.active : null;
  if (!w) return;
  if (!peerOn() || !d) { w.peer_setup(0, 0, 0); return; }
  const ph = new TextEncoder().encode(String(radioCfg.fleet || 'liftlab').slice(0, 31)), nm = new TextEncoder().encode(fleetName(d).slice(0, 15));
  const rb = new Uint8Array(w.memory.buffer, w.rbuf_ptr(), ph.length + nm.length + 2); rb.set(ph); rb[ph.length] = 0; rb.set(nm, ph.length + 1); rb[ph.length + 1 + nm.length] = 0;
  w.peer_setup(1, peerId(d) | 0, ((Math.imul(peerId(d) ^ ++peerAir.starts, 2654435761) >>> 1) || 1));
}
// The path between two drones: its loss [dB] (free space to 10 m, then 33 dB a decade; 2 dBi each end; buildings).
function peerLoss(a, b) {
  const d = Math.max(1, nrm(sub(a, b)));
  let walls = 0;
  if (terrain.boxes && terrain.boxes.length) {
    const hit = new Set();
    for (let s = 1; s < 16; s++) { const q = add(a, scl(sub(b, a), s / 16)); terrain.boxes.forEach((x, i) => { if (q[0] > x.lo[0] && q[0] < x.hi[0] && q[1] > x.lo[1] && q[1] < x.hi[1] && q[2] > x.lo[2] && q[2] < x.hi[2]) hit.add(i); }); }
    walls = Math.min(3, hit.size);
  }
  return 40.2 + 20 * Math.log10(Math.min(d, 10)) + 33 * Math.log10(Math.max(d, 10) / 10) - 4 + 18 * walls + (a[2] < 0.3 || b[2] < 0.3 ? 6 : 0);
}
// Each control step of a drone (boardsControl, in its scope): what its end has due goes on the air; what reached it
// comes in; what it publishes is its own state (the flight core's: armed, battery, height).
function peerStep() {
  if (typeof fleet === 'undefined' || !fleet.ready || !peerOn() || !radio.peer) return;
  const d = fleet.active, b = peerBoard(), w = b && brt.inst.get(b.id); if (!d || !w || !cargo.power) return;
  const t = brt.t, now = fleet.time;
  if (!radio.peer.next || t >= radio.peer.next) {                    // (what it publishes: 10 times a second is plenty)
    radio.peer.next = t + 0.1;
    const head = [brt.fcState, Math.round((S.batt ? S.batt.soc : 0) * 100), est.p ? est.p[2] - (spawnAt ? spawnAt[2] : 0) : 0];
    frIn(w, head); w.fleet_publish(t);                               // (then what the navigation's board and its fleet program said: fleet.h)
    const navB = boardOf('nav');                                     // the table, to the fleet program beside the navigation
    if (navB && w.fleet_pack) { frIn(w, head); const n = w.fleet_pack(t); if (n) sendFrame(b, navB, 'fleet', frOut(w, n)); }
  }
  for (let k = 0; k < 8; k++) {
    const n = w.peer_air_out(t); if (!n) break;
    const a = Uint8Array.from(new Uint8Array(w.memory.buffer, w.peer_addr_ptr(), 6));
    peerAir.q.push({ n: ++peerAir.n, from: d, src: peerAddr(peerId(d)), to: a.every(x => x === 255) ? null : a, bytes: Uint8Array.from(new Uint8Array(w.memory.buffer, w.pbuf_ptr(), n)), p: S.p.slice(), at: now + 0.001 + 0.002 * peerRand(), got: new Set() });
  }
  const mine = peerAddr(peerId(d));
  for (const pk of peerAir.q) {
    if (pk.at > now || pk.from === d || pk.got.has(d)) continue;
    pk.got.add(d);
    if (pk.to && !pk.to.every((x, i) => x === mine[i])) continue;
    const margin = PEER_TX - peerLoss(pk.p, S.p) - PEER_SENS, p1 = 1 / (1 + Math.exp(-(margin - 2) / 1.3));
    let heard = false; for (let r = 0; r < (pk.to ? 4 : 1) && !heard; r++) heard = peerRand() < p1;
    if (!heard) continue;
    new Uint8Array(w.memory.buffer, w.pbuf_ptr(), pk.bytes.length).set(pk.bytes);
    new Uint8Array(w.memory.buffer, w.peer_addr_ptr(), 6).set(pk.src);
    w.peer_air_in(pk.bytes.length, Math.round(PEER_TX - peerLoss(pk.p, S.p)), t);
  }
  if (peerAir.q.length > 64 || (peerAir.q.length && (peerAir.q[0].at < now - 0.02 || peerAir.q[0].at > now + 0.05))) peerAir.q = peerAir.q.filter(pk => pk.at >= now - 0.02 && pk.at <= now + 0.05);   // (late ones gone; and from before a world reset)
  if (!radio.peer.logAt || t >= radio.peer.logAt) { radio.peer.logAt = t + 0.2; peerLogChanges(w, t); }
}
// The drone's table, as its end has it (board_wasm.c peer_list, 8 + PEER_VALS a drone): [{ state, id, name, lq, heardUs, heard, valsAge, rtt, vals }].
const PEER_VALS = 20;
const PEER_STATES = ['lost', 'heard', 'stale', 'connected'];
function peerTable(w, t) {
  if (!w || !w.peer_list) return [];
  const n = w.peer_list(t), o = new Float32Array(w.memory.buffer, w.fr_ptr(), n), out = [];
  for (let i = 0; i < 8; i++) {
    const k = i * (8 + PEER_VALS); if (o[k] < 0) continue;
    out.push({ slot: i, state: o[k], id: w.peer_id(i) >>> 0, name: cstr(w, w.peer_name_ptr(i), 16), lq: o[k + 2], heardUs: o[k + 3], heard: o[k + 4], valsAge: o[k + 5], rtt: o[k + 6], vals: Array.from(o.subarray(k + 8, k + 8 + o[k + 7])) });
  }
  return out;
}
// Who came and went, into the link log (once each change).
function peerLogChanges(w, t) {
  for (const p of peerTable(w, t)) {
    const was = radio.peer.seen.get(p.id); if (was === p.state) continue;
    radio.peer.seen.set(p.id, p.state);
    if (was == null && p.state < 3) continue;                        // (first only heard: said when it connects)
    linkLog('↔', 'peer', `${p.name || 'a drone'}: ${PEER_STATES[p.state]}`, p.state === 3 ? (was == null ? 'found, a session with it' : 'back') : p.state === 2 ? 'nothing for a second' : p.state === 0 ? 'nothing for 3 s' : '', p.state === 3 ? 'good' : p.state === 0 ? 'bad' : 'warn');
  }
}
// The fleet program beside the navigation (fleet.h): what it is doing, and the pilot's switch for it. Values a drone
// publishes before its program's: the flight core's 3, the navigation's 8 (position, velocity, heading, flags).
const FLEET_HEAD_N = 11;
function fleetView() {
  const b = typeof boardOf === 'function' ? boardOf('nav') : null, w = b && brt.ready && brt.inst.get(b.id); if (!w || !w.fleet_view) return null;
  const n = w.fleet_view(), o = new Float32Array(w.memory.buffer, w.fr_ptr(), n);
  return { ok: o[0] > 0.5, engaged: o[1] > 0.5, go: [o[2], o[3], o[4]], heading: o[5], calls: o[6], fails: o[7], sent: o[8], got: o[9], pub: Array.from(o.subarray(11, 11 + o[10])) };
}
// On (1) or off: with a radio, the pilot's FLEET command up the link (rc_core.h), as the drone would get it; without,
// straight to the navigation's board. '' or why not.
function fleetEngage(on) {
  if (!hasTask('nav')) return 'No navigation: the fleet program runs beside it.';
  if (brt.gnd && hasTask('tlm')) return radioCommand(5, [on ? 1 : 0]) ? 'The command module has too many commands waiting.' : '';
  const b = boardOf('nav'), w = b && brt.inst.get(b.id); if (!w) return 'The boards aren\'t running.';
  return w.fleet_cmd(on ? 1 : 0) ? cstr(w, w.fleet_msg_ptr()) : '';
}
// A ping from this drone to another (the Ground tab's button): its round trip shows in the table.
function peerPing(id) { const b = peerBoard(), w = b && brt.inst.get(b.id); return w && peerOn() ? w.peer_ping(id | 0, brt.t) : -1; }
