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
// The flight controller (ESP32) keeps, per actuator: whether it's available, how effective it is compared with
// its table, and a throttle ceiling. It corrects the throttle for the battery's voltage. It flies within
// speed and lean limits, and can return home and land by itself.
//
// The supervisor (a companion computer, like a Raspberry Pi) runs at 10 Hz over a link with a delay. It reads
// the sensors and the flight controller's data stream, works out which parts have failed, degraded or are
// running hot, and sends the flight controller new settings: a failed part is removed, a degraded one's
// column in the table is scaled down, a hot one is capped. Then it decides how the drone should fly on
// what's left: normally, carefully, back home, or straight down.

/* ───────────────────────── the real drone ───────────────────────── */
const hs = new Map();   // id -> { T, loss, dead, jam, limp, cause, failT }, for motors and servos
const hb = { T: 25, fade: 0, cellsLost: 0, cut: false, cause: '' };
const ambient = () => (envr.ambient ?? 25);
function hsOf(c) {
  let s = hs.get(c.id);
  if (!s) { s = { T: ambient(), loss: 0, dead: false, prop: false, jam: null, limp: false, cause: '', failT: null }; hs.set(c.id, s); }
  return s;
}
// Thrust a motor really delivers, as a share of its card's max: its health, any damage, 0 if dead or its prop is broken.
const motorEff = c => { const s = hs.get(c.id); return c.health / 100 * (s ? (s.dead || s.prop ? 0 : 1 - s.loss) : 1); };
const defaultBattery = () => ({ cells: 4, capacity: 1.3, rInt: 0.06, tmaxC: 60, failHeat: true, failMode: 'cell', vsens: true, isens: true, tsens: true });
function battCfg() { if (!cfg.battery) cfg.battery = defaultBattery(); return cfg.battery; }
// What the battery model gets: cells still working, capacity after wear, resistance at this temperature.
function battParams() {
  const b = battCfg();
  return { cells: Math.max(0, b.cells - hb.cellsLost), capacity: b.capacity * 3600 * (1 - hb.fade), cut: hb.cut,
    rInt: b.rInt * clamp(Math.exp(0.015 * (25 - hb.T)), 0.6, 3) * (1 + 2 * hb.fade) };
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

/* ───────────────────────── the flight controller's side ───────────────────────── */
// Per actuator: available, effectiveness relative to its table, throttle ceiling. Per servo: usable.
const fc = { act: new Map(), joint: new Map(), jAng: new Map(), lim: { speed: null, lean: 35, accel: 6 }, mode: 'normal', armed: true, vComp: true, landT: 0 };
const fcAct = c => { let e = fc.act.get(c.id); if (!e) { e = { on: true, eff: 1, cap: 1 }; fc.act.set(c.id, e); } return e; };
const fcOn = c => fcAct(c).on;
const fcHi = c => { const e = fcAct(c); return e.on ? e.cap : 0; };
const fcJointOk = j => (fc.joint.get(j.id) ?? true);
// The throttle the ESC gets: the voltage correction, so a sagging battery gives the thrust asked for.
// Returns [sent, what it amounts to at the reference voltage]; they differ when the correction runs out of throttle.
function fcThrottle(c, u) {
  if (!fc.vComp || isCollective(c)) return [u, u];
  const V = hread.b.V; if (!(V > 1)) return [u, u];
  const s = run('voltageCompensation', u, V, V_NOM);
  return [s, Math.min(u, s * V / V_NOM)];
}
// Applying a change of effectiveness means editing the table the allocation flies on: the learned columns
// are scaled (and what the learner has built up with them), the description through the modeled health.
function scaleActuatorColumns(c, s) {
  const ix = learn.index.get(c.id); if (!ix || !(s > 0) || Math.abs(s - 1) < 1e-6) return;
  const mats = new Set([learn.B, learn.flyB, learn.st.th, learn.priorKind !== 'desc' ? learn.prior : null].filter(Boolean));
  for (const M of mats) for (let r = 0; r < Math.min(6, M.length); r++) for (const j of ix.cols) M[r][j] *= s;
}
function fcApply(cmd) {
  let changed = false;
  for (const c of actuators()) {
    const k = cmd.acts && cmd.acts[c.id]; if (!k) continue;
    const e = fcAct(c);
    if (k.on === false && e.on) { e.on = false; changed = true; }
    if (k.eff > 0 && Math.abs(k.eff - e.eff) > 1e-3) { scaleActuatorColumns(c, k.eff / e.eff); e.eff = k.eff; changed = true; }
    if (k.cap != null) e.cap = clamp(k.cap, 0, 1);
  }
  for (const j of joints()) { const k = cmd.joints && cmd.joints[j.id]; if (!k) continue; if (fcJointOk(j)) { fc.joint.set(j.id, false); changed = true; } if (isFinite(k.angle)) fc.jAng.set(j.id, clamp(k.angle, -j.range * D2R * 1.2, j.range * D2R * 1.2)); }
  const RANK = { normal: 0, caution: 1, return: 2, land: 3, landed: 4 };
  if (cmd.mode && RANK[cmd.mode] > RANK[fc.mode]) { fc.mode = cmd.mode; if (cmd.mode === 'land' || cmd.mode === 'return') pilot.vref = [0, 0, 0]; }   // only ever steps up (the flight controller may already be landing)
  if (cmd.lim) fc.lim = { ...cmd.lim };
  if (changed) { invalidateDesc(); nb = nominalAxis(); }
}
// Flying home and landing by itself: moves the target like the pilot would, within the limits.
function autoPilotStep(dt) {
  if (fc.mode !== 'return' && fc.mode !== 'land') return false;
  if (!fc.spBefore) fc.spBefore = { ...setpoint };   // put back on reset
  const hub = sensing === 'truth' ? S.p : est.p, v = sensing === 'truth' ? S.v : est.v, sp = fc.lim.speed || 1;
  const want = [0, 0, 0];
  if (fc.mode === 'return') {
    const dx = -setpoint.x, dy = -setpoint.y, d = Math.hypot(dx, dy);
    if (d > 0.05) { want[0] = dx / d * Math.min(sp, d * 1.5); want[1] = dy / d * Math.min(sp, d * 1.5); }
    if (d < 0.15 && Math.hypot(hub[0], hub[1]) < 0.4) fc.mode = 'land';
  } else want[2] = hub[2] > 0.8 ? -0.6 : -0.3;
  for (let i = 0; i < 3; i++) pilot.vref[i] += clamp(want[i] - pilot.vref[i], -2 * dt, 2 * dt);
  setpoint.x += pilot.vref[0] * dt; setpoint.y += pilot.vref[1] * dt; setpoint.z = Math.max(-0.2, setpoint.z + pilot.vref[2] * dt);
  ctl.vRef = pilot.vref.slice();
  if (fc.mode === 'land') {   // touched down: settled low and slow for half a second, then the motors stop
    fc.landT = hub[2] < 0.2 && nrm(v) < 0.3 ? fc.landT + dt : 0;
    if (fc.landT > 0.5) { fc.mode = 'landed'; fc.armed = false; healthEvent('Landed. Motors stopped.', 'good'); }
  }
  return true;
}

/* ───────────────────────── the supervisor ───────────────────────── */
const SUP_HZ = 10, SUP_LINK = 0.04;   // runs at 10 Hz; each way over the link takes 40 ms
const sup = { on: true, cells: null, vHist: [], cellLost: false, next: 0, stream: [], out: [], st: {}, dec: {}, pol: { mode: 'normal' }, tEst: new Map(), log: [], view: {} };
function healthEvent(msg, tone = 'info') { sup.log.unshift({ t: S.t, msg, tone }); if (sup.log.length > 12) sup.log.pop(); }
// The flight controller streams what it commanded and what the drone did (50 Hz), for the supervisor to check.
function streamSample() {
  if (!learn.mf || !learn.mf.f || !learn.n || !fc.armed) return;
  const z = sensing === 'truth' ? S.p[2] : est.p[2]; if (z < 0.35 || S.t < 1.5) return;   // the ground pushes back near it; the first moments settle
  const acts = actuators();
  const phi = acts.map(c => {   // what the table says each motor did just now: its column at the believed angles × its thrust
    const st = act.get(c.id), v = (st && st.v) || 0; return colAt(c).map(x => x * v);
  });
  const psi = steerJoints().map(j => {   // what a little more turn on each steering servo would do, at the thrusts now
    let d = [0, 0, 0, 0, 0, 0]; for (const m of motorsUnder(j)) d = add6(d, scl6(dColAt(m, j), (act.get(m.id) || {}).v || 0)); return d;
  });
  sup.stream.push({ t: S.t, phi, psi, y: [...learn.mf.f, ...learn.mf.a], cmd: acts.map(c => (act.get(c.id) || {}).v || 0) });
  while (sup.stream.length && sup.stream[0].t < S.t - 2) sup.stream.shift();
}
// The supervisor's view of how much each motor could lift and whether roll and pitch can still be held,
// with the table as the flight controller now flies it (removed parts out, ceilings on).
function supMargins() {
  const acts = actuators().filter(fcOn), cols = acts.map(c => colAt(c)), hi = acts.map(fcHi), lo = acts.map(() => 0);
  // Steering servos count too: each can turn its rotors across what's left of its travel (a helicopter's head,
  // a tricopter's tail), at the thrust they carry now.
  for (const j of steerJoints().filter(fcJointOk)) {
    const on = motorsUnder(j).filter(fcOn); if (!on.length) continue;
    let d = [0, 0, 0, 0, 0, 0]; for (const m of on) d = add6(d, scl6(dColAt(m, j), Math.max((act.get(m.id) || {}).v || 0, 0.2)));
    const th = angleSeen(j), R = j.range * D2R;
    cols.push(d); lo.push(Math.min(0, -R - th)); hi.push(Math.max(0, R - th));
  }
  if (!acts.length) return { margin: 0, rpOk: false, yawOk: false };
  const W = [0.3, 0.3, 3, 10, 10, 1];
  const hov = bls(cols, lo, hi, [0, 0, G, 0, 0, 0], W);
  const made = k => cols.reduce((s, c, i) => s + c[k] * hov[i], 0);
  const rpOk = Math.abs(made(3)) < 2 && Math.abs(made(4)) < 2 && made(2) > 0.9 * G;
  const yawOk = Math.abs(made(5)) < 1;
  const up = bls(cols, lo, hi, [0, 0, 3 * G, 0, 0, 0], [0.01, 0.01, 1, 30, 30, 0.01]);
  const az = cols.reduce((s, c, i) => s + c[2] * up[i], 0);
  return { margin: az / G, rpOk, yawOk };
}
function supTick() {
  const now = S.t, seen = now - SUP_LINK;
  // 1. how well each motor still does what its column says, from the data stream (arrived after the link delay)
  const batch = sup.stream.filter(s => s.t <= seen && s.t > (sup.last ?? -1)); sup.last = seen;
  const acts = actuators();
  const h = run('actuatorHealth', sup.st, batch, 1 / SUP_HZ, 8);
  // 2. what each device reports, as the supervisor sees it; temperatures estimated from ESC current if there's no sensor
  const obs = acts.map((c, i) => {
    const r = hread.m.get(c.id) || {}, e = fcAct(c), mp = motorParams(c), cmd = batch.length ? batch[batch.length - 1].cmd[i] : 0;
    let T = r.T ?? null;
    if (T == null && r.I != null) {   // no sensor, but the ESC reports current: run the same heat model on the supervisor
      const th = motorThermal(c), x = clamp((r.rpm || 0) * 2 * Math.PI / 60 / mp.Om, 0, 1.2), t0 = sup.tEst.get(c.id) ?? ambient();
      T = run('thermalModel', t0, r.I * r.I * mp.R, th.Gf * (0.3 + 0.7 * x), th.C, ambient(), 1 / SUP_HZ); sup.tEst.set(c.id, T);
    }
    const rpmExp = Math.sqrt(Math.max(0, cmd)) * mp.Om * 60 / (2 * Math.PI);
    return { id: c.id, name: c.name, on: e.on, eff: e.eff, cmd, temp: T, tmax: c.tmaxC ?? 120, rpmRatio: r.rpm != null && rpmExp > 300 ? r.rpm / rpmExp : null,
      eta: h.eta[i] ?? 1, conf: h.conf[i] ?? 0 };
  });
  steerJoints().forEach((j, k) => {   // servos: where the controller believes each is, and (with feedback) how far off its command it reports
    const st = jst.get(j.id);
    obs.push({ kind: 'servo', id: j.id, name: j.name, angle: angleSeen(j), delta: (h.del || [])[k] ?? 0, conf: (h.sconf || [])[k] ?? 0,
      fbErr: j.feedback && st ? jointTarget(j) - st.th : null });
  });
  sup.dec = run('faultDecision', obs, sup.dec, 1 / SUP_HZ);
  // 3. how to fly on what's left
  const m = supMargins(), b = battCfg(), rb = hread.b;
  // The battery: its resting voltage (measured, plus the sag its current causes) per working cell gives the
  // charge. A drop of about a cell's worth within two seconds means a cell has failed: count one fewer.
  if (sup.cells == null) sup.cells = b.cells;
  const vRest = rb.V != null ? rb.V + b.rInt * (rb.I ?? 0) : null;
  if (vRest != null) {
    const per = vRest / Math.max(1, sup.cells);
    sup.vHist.push([now, per]); while (sup.vHist.length && sup.vHist[0][0] < now - 2) sup.vHist.shift();
    const before = Math.max(...sup.vHist.map(v => v[1]));
    if (per < 3.3 && before - per > 0.5 && sup.cells > 1) { sup.cells--; sup.vHist = []; sup.cellLost = true; healthEvent(`Battery: its voltage fell by about a cell's worth. Counting ${sup.cells} working cells.`, 'bad'); }
  }
  const soc = vRest != null ? clamp((vRest / Math.max(1, sup.cells) - 3.5) / 0.7, 0, 1) : null;
  const temps = obs.filter(o => o.temp != null).map(o => o.temp / o.tmax);
  const sum = { dt: 1 / SUP_HZ, margin: m.margin, rpOk: m.rpOk, yawOk: m.yawOk, anyFailed: Object.values(sup.dec.acts || {}).some(d => d.state === 'failed'),
    hot: temps.length ? Math.max(...temps) : null, soc, cellLost: !!sup.cellLost, vCell: rb.V != null ? rb.V / Math.max(1, sup.cells) : null, battT: rb.T ?? null, battMax: b.tmaxC };
  const pol = run('flightPolicy', sum, sup.pol);
  if (pol.mode !== sup.pol.mode) healthEvent(`Flight: ${POLICY_LABEL[pol.mode] || pol.mode}${pol.why ? ' (' + pol.why + ')' : ''}`, pol.mode === 'normal' ? 'good' : pol.mode === 'caution' ? 'warn' : 'bad');
  sup.pol = pol; sup.view = { sum, obs, m };
  // 4. send the new settings to the flight controller
  const acmd = {};
  for (const o of obs) { const d = (sup.dec.acts || {})[o.id]; if (!d) continue;
    const prev = sup.sent && sup.sent[o.id];
    if (d.on === false && (!prev || prev.on !== false)) healthEvent(`${o.name}: ${d.why}. Removed from the controller's table.`, 'bad');
    else if (d.eff && prev && Math.abs(d.eff - (prev.eff ?? 1)) > 0.02 && d.on !== false) healthEvent(`${o.name}: ${d.why}. Its column now ×${(d.eff / (prev.eff ?? 1)).toFixed(2)}.`, 'warn');
    else if (d.cap < 0.95 && !(sup.capLogged || (sup.capLogged = new Set())).has(o.id)) { sup.capLogged.add(o.id); healthEvent(`${o.name}: ${d.why}. Throttle capped at ${Math.round(d.cap * 100)}%, and lower as it heats.`, 'warn'); }
    acmd[o.id] = { on: d.on, eff: d.eff, cap: d.cap };
  }
  sup.sent = acmd;
  const jcmd = {};
  for (const [id, d] of Object.entries(sup.dec.joints || {})) {
    if (!(sup.jLogged || (sup.jLogged = new Set())).has(+id)) { sup.jLogged.add(+id); healthEvent(`${compById(+id) ? compById(+id).name : 'A servo'}: ${d.why}. Left out of the steering; the controller now uses where it really is.`, 'bad'); }
    jcmd[id] = { on: false, angle: d.angle };
  }
  sup.out.push({ at: now + SUP_LINK, cmd: { acts: acmd, joints: jcmd, mode: pol.mode, lim: pol.lim } });
}
const POLICY_LABEL = { normal: 'normal', caution: 'careful (lower speed and lean)', return: 'returning home to land', land: 'landing now', landed: 'landed' };
function healthStep(dt) {
  sampleHealth(dt);
  if (S.steps % 40 === 0) streamSample();                    // 50 Hz
  while (sup.out.length && sup.out[0].at <= S.t) fcApply(sup.out.shift().cmd);
  if (!sup.on || S.crashed || thr) return;   // a throw start runs on its own until it has caught itself
  if (S.t >= sup.next) { sup.next = S.t + 1 / SUP_HZ; supTick(); }
}
function resetHealth() {
  if (fc.spBefore) { Object.assign(setpoint, fc.spBefore); fc.spBefore = null; S.p = [setpoint.x, setpoint.y, setpoint.z]; }   // the autopilot moved the target: back to where it was
  hs.clear(); Object.assign(hb, { T: ambient(), fade: 0, cellsLost: 0, cut: false, cause: '', failT: null, P: 0 });
  hread.m.clear(); hread.b = {}; hread.next = {}; hSeed = 0x9e3779b9;
  fc.act.clear(); fc.joint.clear(); fc.jAng.clear(); fc.lim = { speed: null, lean: 35, accel: 6 }; fc.mode = 'normal'; fc.armed = true; fc.landT = 0;
  Object.assign(sup, { next: 0, stream: [], out: [], st: {}, dec: {}, pol: { mode: 'normal' }, log: [], view: {}, sent: null, last: -1, capLogged: new Set(), jLogged: new Set(), cells: null, vHist: [], cellLost: false });
  sup.tEst.clear();
  if (typeof invalidateDesc === 'function') invalidateDesc();
}
