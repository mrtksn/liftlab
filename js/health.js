'use strict';
// Heat, failures and the supervisor.
//
// The real drone (physics): every motor and the battery heat up from the current through their resistance
// and cool into the air (faster when the prop is spinning). Heat changes how they work: a hot winding has
// more resistance and a weaker magnet, a warm battery sags less. Past its limit a part is damaged for good
// (magnets lose strength, a battery loses capacity and gains resistance) and further past it, it fails the
// way its card says. Anything can also be broken on demand.
//
// What the drone can know about it: optional sensors (winding and battery temperature, battery voltage and
// current, ESC telemetry with rpm and current), each with its own rate, noise and delay.
//
// The health supervisor is a task on a flight computer (boards.js: runner/fc/super_core.c on a Pi). It reads these
// sensors (they're wired to its board) and the flight core's data stream, works out which parts have failed,
// weakened or are running hot, and sends the flight core new settings; then it decides how the drone should fly on
// what's left: normally, carefully, back home, or straight down.

/* ───────────────────────── the real drone ───────────────────────── */
const hs = new Map();   // id -> { T, loss, dead, jam, limp, cause, failT }, for motors and servos
const hb = { T: 25, fade: 0, cellsLost: 0, cut: false, cause: '', lvc: false, lvcT: 0 };
const ambient = () => (envr.ambient ?? 25);
function hsOf(c) {
  let s = hs.get(c.id);
  if (!s) { s = { T: ambient(), loss: 0, dead: false, prop: false, jam: null, limp: false, cause: '', failT: null }; hs.set(c.id, s); }
  return s;
}
// Thrust a motor really delivers, as a share of its card's max: its health, any damage, 0 if dead or its prop is broken.
const motorEff = c => { const s = hs.get(c.id); return c.health / 100 * (s ? (s.dead || s.prop ? 0 : 1 - s.loss) : 1); };
const defaultBattery = () => ({ cells: 4, capacity: 1.3, rInt: 0.06, tmaxC: 60, failHeat: true, failMode: 'cell', vsens: true, isens: true, tsens: true, startSoc: 1, escCut: 2.8 });
function battCfg() { if (!cfg.battery) cfg.battery = defaultBattery(); else for (const [k, v] of Object.entries(defaultBattery())) if (cfg.battery[k] === undefined) cfg.battery[k] = v; return cfg.battery; }
// What the battery model gets: cells still working, capacity after wear, resistance at this temperature.
function battParams() {
  const b = battCfg();
  return { cells: Math.max(0, b.cells - hb.cellsLost), capacity: b.capacity * 3600 * (1 - hb.fade), cut: hb.cut,
    rInt: b.rInt * clamp(Math.exp(0.015 * (25 - hb.T)), 0.6, 3) * (1 + 2 * hb.fade) };
}
// The ESCs' low-voltage cutoff (hardware, not the flight code): every ESC watches the pack voltage it runs on, and
// once it stays under the cutoff (per cell, under load) for 1.5 s, the ESCs stop their motors. They start
// again only after the throttle has been at zero (disarmed), as hobby ESCs do. 0 turns it off.
function escCutoffStep(dt, anyThrottle) {
  const b = battCfg(), cut = b.escCut ?? 2.8, cells = Math.max(1, b.cells - hb.cellsLost);
  if (hb.lvc) { if (!anyThrottle) { hb.lvc = false; hb.lvcT = 0; } return hb.lvc; }
  if (!(cut > 0) || !anyThrottle || hb.cut) { hb.lvcT = 0; return false; }
  hb.lvcT = S.battV / cells < cut ? hb.lvcT + dt : 0;
  if (hb.lvcT > 1.5) { hb.lvc = true; healthEvent(`ESCs: low-voltage cutoff, the pack is down to ${(S.battV / cells).toFixed(2)} V per cell under load. Motors stopped.`, 'bad'); }
  return hb.lvc;
}
// Thermal sizing: a motor's stator holds about 500 J/K per kg; its cooling is sized so that running at full
// throttle without a break would settle 25% above its limit (full throttle is for bursts), and it cools
// best with the prop at full speed (to 30% of that when stopped).
function motorThermal(c) {
  const mp = motorParams(c), iF = mp.kQ * mp.Om * mp.Om / mp.Ke, Pf = iF * iF * mp.R;
  return { C: 500 * Math.max(0.02, c.mass), Gf: (c.cool ?? 1) * Pf / (1.25 * Math.max(10, (c.tmaxC ?? 120) - 25)), Pf };
}
const battMass = () => { const b = battCfg(); return 0.038 * b.capacity * b.cells; };
// The motor as it is at its temperature: copper resistance +0.39 %/K, magnet strength −0.12 %/K.
function heatParams(c, mp) {
  const s = hs.get(c.id); if (!s) return mp;
  const d = s.T - 25;
  return { ...mp, R: mp.R * (1 + 0.0039 * d), Ke: mp.Ke * Math.max(0.5, 1 - 0.0012 * d) };
}
// A dead motor (burned out, or its ESC cut): no current, the prop just slows down in the air.
function coastStep(st, mp, dt) {
  const O = Math.max(0, (st.Omega || 0) - mp.kQ * (st.Omega || 0) ** 2 / mp.J * dt);
  return { Omega: O, i: 0, tau: 0, T: mp.kT * O * O };
}
function heatMotor(c, st, md, mp, dt) {
  const s = hsOf(c), th = motorThermal(c), x = clamp((st.Omega || 0) / mp.Om, 0, 1.2);
  const P = md.i * md.i * mp.R;
  s.T = run('thermalModel', s.T, P, th.Gf * (0.3 + 0.7 * x), th.C, ambient(), dt);
  s.P = P;
  if (c.failHeat === false) return;
  const lim = c.tmaxC ?? 120;
  if (s.T > lim && !s.dead) s.loss = Math.min(0.9, s.loss + dt * 0.0006 * (s.T - lim));   // magnets weaken for good
  if (s.T > lim + 35 && !s.dead && !s.failT) breakDevice(c, c.failMode || 'stop', 'burned out');
}
function heatBattery(I, dt) {
  const b = battCfg(), p = battParams(), m = battMass();
  hb.P = I * I * p.rInt;
  hb.T = run('thermalModel', hb.T, hb.P, 1.4 * (m / 0.2) ** (2 / 3), 900 * m, ambient(), dt);
  if (b.failHeat === false) return;
  if (hb.T > b.tmaxC) hb.fade = Math.min(0.8, hb.fade + dt * 0.0004 * (hb.T - b.tmaxC));   // capacity lost, resistance up
  if (hb.T > b.tmaxC + 25 && !hb.failT) breakBattery(b.failMode || 'cell', 'overheated');
}
// Break something on demand, or as a result of overheating.
function breakDevice(c, mode, cause = 'broken on demand') {
  const s = hsOf(c); s.cause = cause; s.failT = S.t;
  if (c.type === 'motor') {
    if (mode === 'prop') { s.prop = true; s.mode = 'prop'; healthEvent(`${c.name}: prop broke (${cause})`, 'bad'); if (typeof refreshEnvelope === 'function') refreshEnvelope(); return; }
    if (mode === 'loss') s.loss = Math.max(s.loss, (c.failLoss ?? 50) / 100); else s.dead = true; s.mode = mode === 'loss' ? 'loss' : 'stop';
  }
  else if (c.type === 'joint') { const st = jst.get(c.id); if (mode === 'limp') s.limp = true; else s.jam = st ? st.th : 0; s.mode = mode === 'limp' ? 'limp' : 'jam'; }
  healthEvent(`${c.name}: ${c.type === 'motor' ? (s.dead ? 'stopped' : `lost ${Math.round(s.loss * 100)}% of its thrust`) : (s.limp ? 'went limp' : 'jammed')} (${cause})`, 'bad');
  if (typeof refreshEnvelope === 'function') refreshEnvelope();
}
function breakBattery(mode, cause = 'broken on demand') {
  hb.cause = cause; hb.failT = S.t;
  if (mode === 'cut') hb.cut = true; else hb.cellsLost = Math.min(battCfg().cells, hb.cellsLost + 1);
  healthEvent(`Battery: ${hb.cut ? 'cut out' : 'lost a cell'} (${cause})`, 'bad');
}
// Servo failures, as torques: jammed holds its angle hard; limp has only friction.
function servoFault(j, st, t) {
  const s = hs.get(j.id); if (!s) return t;
  if (s.limp) return -0.004 * (st.rate || 0);
  if (s.jam != null) return -60 * (st.th - s.jam) - 1.5 * (st.rate || 0);
  return t;
}

/* ───────────────────────── sensors ───────────────────────── */
// What the drone's electronics report: each with its rate, noise and delay. A temperature sensor sits on
// the stator or the pack, so it trails the true temperature.
const hread = { m: new Map(), b: {}, next: {} };
// Their own noise, seeded apart from the flight sensors', so fitting health sensors doesn't change the rest.
let hSeed = 0x9e3779b9;
function hrandn() {
  const u = () => { hSeed = (hSeed + 0x6D2B79F5) | 0; let t = Math.imul(hSeed ^ (hSeed >>> 15), 1 | hSeed); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  return Math.sqrt(-2 * Math.log(u() + 1e-12)) * Math.cos(2 * Math.PI * u());
}
function readTick(key, rate) { const n = hread.next[key] ?? 0; if (S.t + 1e-9 < n) return false; hread.next[key] = n + 1 / rate; return true; }
function sampleHealth(dt) {
  for (const c of actuators()) {
    const s = hsOf(c), st = act.get(c.id), r = hread.m.get(c.id) || {}; hread.m.set(c.id, r);
    r.lagT = (r.lagT ?? s.T) + (s.T - (r.lagT ?? s.T)) * Math.min(1, dt / 1.5);
    if (c.tsens && readTick('t' + c.id, 10)) r.T = r.lagT + 0.3 * hrandn();
    if (!c.tsens) r.T = null;
    if (c.telem && st && readTick('e' + c.id, 50)) { r.rpm = (st.Omega || 0) * 60 / (2 * Math.PI) * (1 + 0.005 * hrandn()); r.I = (st.i || 0) + 0.1 * hrandn(); r.at = S.t; }
    if (!c.telem) { r.rpm = null; r.I = null; }
  }
  const b = battCfg(), r = hread.b;
  r.lagT = (r.lagT ?? hb.T) + (hb.T - (r.lagT ?? hb.T)) * Math.min(1, dt / 5);
  if (readTick('bv', 50)) { r.V = b.vsens ? S.battV + 0.02 * hrandn() : null; r.I = b.isens ? (S.battI || 0) + 0.2 * hrandn() : null; }
  if (readTick('bt', 2)) r.T = b.tsens ? r.lagT + 0.3 * hrandn() : null;
}

/* ───────────────────────── what the boards were told ───────────────────────── */
// The flight core corrects the throttle for the battery's voltage when it has a voltage reading (fc.vComp). jAng: the
// angle the supervisor says a stuck servo is really at (for the simulator's view of what the flight code believes).
const fc = { vComp: true, jAng: new Map() };
// The simulator's own events (a part breaking, a restart), shown with the supervisor's in the Health panel.
const sup = { log: [] };
function healthEvent(msg, tone = 'info') { sup.log.unshift({ t: S.t, msg, tone }); if (sup.log.length > 12) sup.log.pop(); }
function healthStep(dt) { sampleHealth(dt); }
// The health readings as the supervisor's board gets them (super_core.h super_health).
function healthReadings() {
  const acts = actuators(), out = [acts.length];
  for (const c of acts) { const r = hread.m.get(c.id) || {}; out.push(r.T != null ? 1 : 0, r.T ?? 0, r.rpm != null ? 1 : 0, r.rpm ?? 0, r.I != null ? 1 : 0, r.I ?? 0); }
  const b = hread.b; out.push(b.V != null ? 1 : 0, b.V ?? 0, b.I != null ? 1 : 0, b.I ?? 0, b.T != null ? 1 : 0, b.T ?? 0);
  return out;
}
function resetHealth() {
  hs.clear(); Object.assign(hb, { T: ambient(), fade: 0, cellsLost: 0, cut: false, cause: '', failT: null, P: 0, lvc: false, lvcT: 0 });
  hread.m.clear(); hread.b = {}; hread.next = {}; hSeed = 0x9e3779b9;
  fc.jAng.clear(); sup.log = [];
  if (typeof invalidateDesc === 'function') invalidateDesc();
}
