'use strict';
// Integration of the physical models with the editor and simulated flight boards.
let flightMotorCache = new WeakMap(), flightLimitCache = new WeakMap();
let flightBatteryThermalMass = 0;
let flightAirTemp = NaN, flightAirPressure = NaN, flightAir = FlightPhysics.atmosphere();
function flightAtmosphere() {
  if (envr.ambient !== flightAirTemp || envr.pressure !== flightAirPressure) {
    flightAirTemp = envr.ambient; flightAirPressure = envr.pressure;
    flightAir = FlightPhysics.atmosphere(envr.ambient, envr.pressure);
  }
  return flightAir;
}
function flightMotor(c) {
  let mp = flightMotorCache.get(c);
  if (!mp) { mp = FlightPhysics.motor(c); flightMotorCache.set(c, mp); }
  return mp;
}
function flightPrepare() {
  flightMotorCache = new WeakMap();
  FlightPhysics.batteryMass(cfg.comps, battCfg());
  flightBatteryThermalMass = cfg.comps.reduce((sum, c) => sum + (c.type === 'mass' && c.battery ? c.mass : 0), 0);
  for (const c of actuators()) {
    if (c.propPhysics?.rows && c.propPhysics.radius && Math.abs(c.propPhysics.radius - propR(c)) > 1e-8) delete c.propPhysics.rows;
    const mp = flightMotor(c); if (mp.kind !== 'generic' || mp.table) c.tmax = mp.ratedThrust;
  }
}
function flightDuty(c) {
  const p = computers().wiring?.parts?.[c.id];
  return p?.driver === 'brushed' ? clamp((p.maxDuty ?? 100) / 100, 0, 1) : 1;
}
function flightAvailable(c, voltage = S.battV || 16) {
  const mp = heatParams(c, flightMotor(c)), air = flightAtmosphere();
  if (isCollective(c)) {
    const load = collectiveLoad(c, mp, 1);
    const point = FlightPhysics.equilibrium({...mp, table: null, kT: load.kT, kQ: load.kQ, rhoRef: 1.225}, flightDuty(c), voltage, air.rho, air.sound);
    return load.kT * Math.min(load.Og, point.Omega) ** 2 * air.rho / 1.225;
  }
  return FlightPhysics.equilibrium(mp, flightDuty(c), voltage, air.rho, air.sound).T;
}
function flightRememberSettings(w, data) { flightLimitCache.set(w, Float32Array.from(data)); }
function flightApplyLimits(w, voltage, acts = actuators()) {
  const n = 6 + 3 * acts.length + 2 * joints().length;
  let source = flightLimitCache.get(w);
  if (!source || source.length !== n) {
    source = new Float32Array(n); source[4] = acts.length; source[5] = joints().length;
    for (let i = 0; i < acts.length; i++) { source[6 + 3 * i] = 1; source[7 + 3 * i] = 1; source[8 + 3 * i] = 1; }
    flightLimitCache.set(w, source);
  }
  // The existing SET API bounds thrust fractions. Express the physical duty limit
  // in the controller's reference-voltage coordinates; retain all supervisor fields.
  if (!source.lastCaps) {
    source.lastCaps = Float32Array.from(acts, (_, i) => source[8 + 3 * i]);
    source.nextCaps = new Float32Array(acts.length);
  }
  let changed = false;
  for (let i = 0; i < acts.length; i++) {
    const c = acts[i], duty = flightDuty(c);
    const eq = clamp(duty * (voltage > 1 ? voltage / 16 : 1), 0, 1);
    // learn_core currently exports -1 for curve overrides: the core retains its prior.
    const bend = curveHat(c), cap = believedThrust(eq, bend);
    source.nextCaps[i] = Math.min(source[8 + 3 * i], cap);
    if (source.nextCaps[i] !== source.lastCaps[i]) changed = true;
  }
  if (!changed) return; // retain the core's state; no repeated SET/learned-axis solve
  const out = new Float32Array(w.memory.buffer, w.fr_ptr(), n); out.set(source);
  for (let i = 0; i < acts.length; i++) out[8 + 3 * i] = source.nextCaps[i];
  if (w.fc_set(n) === 0) source.lastCaps.set(source.nextCaps);
}
function flightDevicePower() {
  const b = battCfg(); let watts = b.avionicsW ?? 8, servoW = 0;
  for (const c of liveComps()) {
    watts += Math.max(0, c.deviceW || 0);
    if (c.type === 'joint') {
      const st = jst.get(c.id), p = c.servoPhysics || {};
      if (st && wiredTo(c) === boardOf('core')) {
        const load = clamp(Math.abs(st.tq || 0) / Math.max(.01, c.torque ?? .8), 0, 1);
        const mechanical = Math.abs((st.tq || 0) * (st.rate || 0));
        servoW += (p.idleW ?? .15) + (p.stallW ?? 5) * load * load + mechanical / (p.efficiency ?? .65);
      }
    } else if (c.type === 'latch') {
      const st = cargo.lat.get(c.id); if (st && st.drive != null && Math.abs(st.drive - st.pos) > .01) servoW += c.latchW ?? 2;
    }
  }
  S.deviceW = watts + servoW / (b.becEfficiency ?? .9);
  return S.deviceW;
}
function flightCoast(st, mp, dt) {
  const load = FlightPhysics.loadAt(mp, st.Omega || 0, flightAtmosphere().rho, st.airQ || 1, st.airT || 1);
  const old = st.Omega || 0, damping = 2 * load.Q / Math.max(1, old);
  const Omega = Math.max(0, old - (load.Q + mp.friction) * dt / (mp.J + dt * damping));
  return { Omega, i: 0, tau: 0, T: FlightPhysics.loadAt(mp, Omega, flightAtmosphere().rho, st.airQ || 1, st.airT || 1).T };
}
function flightEnvironment() { return { ambient: envr.ambient, pressure: envr.pressure ?? 101325, sensorEffects: !!envr.sensorEffects, rotorSamples: envr.rotorSamples }; }
function flightRotorWake(ro, rotors, rho) {
  const others = rotors.filter(o => o !== ro), center = run('wakeVelocity', ro.p, others, rho);
  if (envr.rotorSamples !== 5 || !others.length) return center;
  // Sample only where a nearby wake could intersect this disk. Crosswind drift
  // follows the same approximation as wakeVelocity; ordinary separated quads
  // retain their single query. Five fixed points bound work for coaxial designs.
  let overlap = false;
  for (const source of others) {
    if (!(source.T > 0)) continue;
    const rel = sub(ro.p, source.p), downstream = -dot(rel, source.d);
    if (downstream < -source.R) continue;
    const vi = Math.sqrt(source.T / (2 * rho * Math.PI * source.R * source.R));
    const flow = source.va || [0, 0, 0], drift = scl(sub(flow, scl(source.d, dot(flow, source.d))), Math.max(0, downstream) / (1.5 * vi));
    const radius = source.R * (downstream > 0 ? .71 + .29 * Math.exp(-downstream / source.R) : 1);
    if (nrm(sub(add(rel, scl(source.d, downstream)), drift)) < ro.R + 1.1 * radius) { overlap = true; break; }
  }
  if (!overlap) return center;
  const a = unit(crs(ro.d, Math.abs(ro.d[2]) > .9 ? [1, 0, 0] : [0, 0, 1])), b = crs(ro.d, a);
  let mean = center;
  for (const axis of [a, b]) for (const direction of [-1, 1]) mean = add(mean, run('wakeVelocity', add(ro.p, scl(axis, direction * .7 * ro.R)), others, rho));
  return scl(mean, .2);
}
const flightSkyDirections = [[0, 0, 1], [.8, 0, .6], [-.8, 0, .6], [0, .8, .6], [0, -.8, .6]];
function flightSkyVisibility(position, rt) {
  if (rt.skyTime == null || S.t < rt.skyTime || S.t - rt.skyTime >= .1) {
    let clear = 0;
    for (const direction of flightSkyDirections) if (!Number.isFinite(terrainRay(position, direction, 200))) clear++;
    rt.sky = clear / flightSkyDirections.length; rt.skyTime = S.t;
  }
  return rt.sky;
}
function flightRestoreEnvironment(e) {
  if (!e) return;
  envr.ambient = FlightPhysics.bounded(e.ambient, -50, 80, 25);
  envr.pressure = FlightPhysics.bounded(e.pressure, 20000, 120000, 101325);
  envr.sensorEffects = !!e.sensorEffects;
  envr.rotorSamples = e.rotorSamples === 1 ? 1 : 5;
  flightAirTemp = NaN; if (typeof syncSp === 'function') syncSp();
  const input = document.getElementById('flight-sensor-effects'); if (input) input.checked = envr.sensorEffects;
  const sampling = document.getElementById('flight-rotor-samples'); if (sampling) sampling.value = String(envr.rotorSamples);
}
// Frame CPU cost and actual rAF pacing, with fixed-size storage. GPU execution is
// asynchronous and is not represented by the CPU timer.
const flightPerf = { at: 0, count: 0, dt: new Float64Array(240), cpu: new Float64Array(240), physics: new Float64Array(240), sim: new Float64Array(240),
  record(dt, cpu, physics, sim) { const i = this.at++ % 240; this.count = Math.min(240, this.count + 1); this.dt[i] = dt; this.cpu[i] = cpu; this.physics[i] = physics; this.sim[i] = sim; },
  read() {
    const n = this.count; if (!n) return null;
    const q = a => { const s = Array.from(a.subarray(0, n)).sort((a, b) => a - b); return s[Math.min(n - 1, Math.floor(n * .95))]; };
    let wall = 0, sim = 0; for (let i = 0; i < n; i++) { wall += this.dt[i]; sim += this.sim[i]; }
    return { fps: wall > 0 ? n / wall : 0, realtime: wall > 0 ? sim / wall : 0, cpu95: q(this.cpu), physics95: q(this.physics), interval95: q(this.dt) * 1000 };
  } };
