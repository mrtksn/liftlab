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
let hs = new Map();   // id -> { T, loss, dead, jam, limp, cause, failT }, for motors and servos
let hb = { T: 25, fade: 0, cellsLost: 0, cut: false, cause: '', lvc: false, lvcT: 0 };
const ambient = () => (envr.ambient ?? 25);
function hsOf(c) {
  let s = hs.get(c.id);
  if (!s) { s = { T: ambient(), loss: 0, dead: false, prop: false, jam: null, limp: false, cause: '', failT: null }; hs.set(c.id, s); }
  return s;
}
// Thrust a motor really delivers, as a share of its card's max: its health, any damage, 0 if dead or its prop is broken.
const motorEff = c => { const s = hs.get(c.id); return c.health / 100 * (s ? (s.dead || s.prop ? 0 : 1 - s.loss) : 1); };
const defaultBattery = () => ({ cells: 4, capacity: 1.3, rInt: 0.06, tmaxC: 60, failHeat: true, failMode: 'cell', vsens: true, isens: true, tsens: true, startSoc: 1, escCut: 2.8 });
const initializedBatteries = new WeakSet();
function battCfg() {
  if (!cfg.battery) cfg.battery = defaultBattery();
  if (!initializedBatteries.has(cfg.battery)) {
    for (const [k, v] of Object.entries(defaultBattery())) if (cfg.battery[k] === undefined) cfg.battery[k] = v;
    initializedBatteries.add(cfg.battery);
  }
  return cfg.battery;
}
// What the battery model gets: cells still working, capacity after wear, resistance at this temperature.
function battParams() {
  const b = battCfg();
  return { cells: Math.max(0, b.cells - hb.cellsLost), capacity: b.capacity * 3600 * (1 - hb.fade), cut: hb.cut,
    rInt: b.rInt * clamp(Math.exp(0.015 * (25 - hb.T)), 0.6, 3) * (1 + 2 * hb.fade) };
}
// Every source but the main pack (that one is S.batt, with its heat and wear): its charge, voltage and current, in
// S.packs by source id. A pack drains with what's on it and sags like the main pack (the battery law), with no heating
// or wear; a USB port or bench supply holds its voltage. A pack is flat at 3.0 V a cell under load; a USB port shuts
// off when it's asked for well over its rating for a moment (a bench supply limits its current: it sags instead).
// Whatever is on a source that's down has no power; on the flight controller's source, the drone is off.
function sourceInit(id, e) {
  const { src, spec } = e, st = { soc: src.kind === 'battery' ? clamp(spec.startSoc ?? 1, 0.02, 1) : 1, V: 0, I: 0, overT: 0 };
  st.V = src.kind === 'battery' ? run('batteryModel', st, 0, 0, { cells: spec.cells, capacity: spec.capacity * 3600, rInt: spec.rInt, cut: false }) : spec.volts;
  S.packs.set(id, st); return st;
}
function sourcesReset() { const m = cargo.pmap; if (m) for (const [id, e] of m.srcs) if (id !== m.main) sourceInit(id, e); }
function sourceStep(id, watts, dt) {
  const m = cargo.pmap, e = m && m.srcs.get(id); if (!e) return;
  const st = S.packs.get(id) || sourceInit(id, e);   // (a source that turned up mid-flight)
  const { src, spec } = e;
  if (st.cut || st.flat) { st.V = 0; st.I = 0; return; }
  st.I = watts / Math.max(1, st.V);
  if (src.kind === 'battery') {
    const V = run('batteryModel', st, st.I, dt, { cells: spec.cells, capacity: spec.capacity * 3600, rInt: spec.rInt, cut: false });
    st.V += (V - st.V) * Math.min(1, dt / 0.002);
    if (st.V / spec.cells < 3.0) { st.flat = true; sourceOff(id, 'is flat'); return; }
  } else if (src.kind === 'bench') st.V = st.I > spec.maxA ? spec.volts * spec.maxA / st.I : spec.volts;   // current limit: it sags
  else st.V = spec.volts;
  st.overT = st.I > spec.maxA * (src.kind === 'usb' ? 1.2 : 1) ? st.overT + dt : 0;
  if (st.overT > 0.3 && !st.overSaid) {
    st.overSaid = true;
    if (src.kind === 'usb') { st.cut = true; sourceOff(id, `shut off: asked for ${st.I.toFixed(2)} A, more than its ${spec.maxA} A`); }
    else healthEvent(`${src.name}: drawing ${st.I.toFixed(1)} A, more than its ${spec.maxA} A rating`, 'warn');
  }
}
// The volts a source gives now (the main pack's S.battV), 0 when it's down or there's none.
function sourceV(id) { const m = cargo.pmap; if (id == null || !m) return 0; if (id === m.main) return S.battV; const e = m.srcs.get(id); if (!e) return 0; const st = S.packs.get(id) || sourceInit(id, e); return !st.cut && !st.flat ? st.V : 0; }
function sourceOff(id, why) {
  const m = cargo.pmap, e = m && m.srcs.get(id), name = e ? e.src.name : 'A source', on = [...m.board.values()].filter(b => b.src === id).map(b => b.name);
  if (id === m.core) { cargo.power = false; cargo.powerT = S.t; }
  healthEvent(`${name} ${why}: ${on.length ? on.join(' and ') + (on.length > 1 ? ' are' : ' is') + ' off' : 'what it powers stops'}${id === m.core ? '. The motors stop.' : ''}`, 'bad');
  if (typeof cargoLog === 'function') cargoLog(`${name} ${why}.`, 'bad');
}
function breakSource(id) { const st = S.packs.get(id); if (!st || st.cut) return; st.cut = true; st.V = 0; sourceOff(id, 'cut out'); }
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
// Cached generic thermal sizing uses 500 J/K per kg and a fixed 95 K reference
// rise, independent of the failure threshold; explicit C/G override it.
function motorThermal(c) {
  const t = motorParams(c).thermal;
  return { C: t.C, Gf: (c.cool ?? 1) * t.G, Pf: t.referenceLoss };
}
const battMass = () => { const b = battCfg(); return flightBatteryThermalMass || 0.038 * b.capacity * b.cells; };
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
  const s = hsOf(c), th = mp.thermal, x = clamp((st.Omega || 0) / Math.max(1, mp.Om), 0, 1.2);
  const P = md.i * md.i * mp.R;
  s.T = run('thermalModel', s.T, P, th.G * (c.cool ?? 1) * (0.3 + 0.7 * x), th.C, ambient(), dt);
  s.P = P;
  if (c.failHeat === false) return;
  const lim = c.tmaxC ?? 120;
  if (s.T > lim && !s.dead) s.loss = Math.min(0.9, s.loss + dt * 0.0006 * (s.T - lim));   // magnets weaken for good
  if (s.T > lim + 35 && !s.dead && !s.failT) breakDevice(c, c.failMode || 'stop', 'burned out');
}
function heatBattery(I, dt) {
  const b = battCfg(), p = battParams(), m = battMass();
  hb.P = I * I * p.rInt;
  hb.T = run('thermalModel', hb.T, hb.P, b.cooling ?? 1.4 * (m / 0.2) ** (2 / 3), b.heatCapacity ?? 900 * m, ambient(), dt);
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
let hread = { m: new Map(), b: {}, next: {} };
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
let fc = { vComp: true, jAng: new Map() };
// The simulator's own events (a part breaking, a restart), shown with the supervisor's in the Health panel.
let sup = { log: [] };
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
  hread.m.clear(); hread.b = {}; hread.next = {}; hSeed = seedHash(worldSeeds.noise, 'health') | 0;
  fc.jAng.clear(); sup.log = [];
  if (typeof invalidateDesc === 'function') invalidateDesc();
}
