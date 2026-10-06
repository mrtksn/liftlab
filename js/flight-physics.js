'use strict';
// Bounded, browser-side physical models. Static fits are prepared on design edits;
// per-step work is scalar arithmetic and at most a 64-row interpolation.
const FlightPhysics = (() => {
  const RHO = 1.225, TAU = 2 * Math.PI;
  const bounded = (x, lo, hi, fallback) => Number.isFinite(+x) ? Math.max(lo, Math.min(hi, +x)) : fallback;
  function atmosphere(temperature = 25, pressure = 101325) {
    const kelvin = bounded(temperature, -50, 80, 25) + 273.15;
    return { rho: bounded(pressure, 20000, 120000, 101325) / (287.05 * kelvin), sound: Math.sqrt(1.4 * 287.05 * kelvin) };
  }
  function propTable(rows) {
    if (!Array.isArray(rows) || rows.length < 2 || rows.length > 64) throw new Error('Use 2–64 rows: RPM, thrust N, torque Nm.');
    let previous = 0, thrust = 0;
    return rows.map((row, i) => {
      if (!Array.isArray(row) || row.length !== 3 || !row.every(x => typeof x === 'number' && Number.isFinite(x))) throw new Error(`Row ${i + 1}: three finite numbers required.`);
      const [rpm, t, q] = row;
      if (!(rpm > previous && rpm <= 200000 && t > 0 && t >= thrust && t <= 1000 && q > 0 && q <= 100)) throw new Error(`Row ${i + 1}: increasing positive RPM/thrust and positive torque required.`);
      previous = rpm; thrust = t;
      return [rpm * TAU / 60, t, q];
    });
  }
  function parsePropTable(text) {
    const rows = text.trim().split(/\r?\n/).filter(x => x.trim() && !x.trim().startsWith('#')).map((s, i) => {
      if (i === 0 && /^\s*rpm\b/i.test(s)) return null;
      return s.trim().split(/[\s,;]+/).map(Number);
    }).filter(Boolean);
    propTable(rows); return rows;
  }
  function loadAt(mp, omega, rho = RHO, torqueFactor = 1, thrustFactor = 1) {
    const o = Math.max(0, omega), scale = rho / mp.rhoRef;
    let t, q;
    if (!mp.table) { t = mp.kT * o * o; q = mp.kQ * o * o; }
    else {
      const a = mp.table, last = a.length - 1;
      if (o <= a[0][0] || o >= a[last][0]) {
        const r = o <= a[0][0] ? a[0] : a[last], f = (o / r[0]) ** 2; t = r[1] * f; q = r[2] * f;
      } else {
        let lo = 0, hi = last;
        while (hi - lo > 1) { const m = (hi + lo) >> 1; if (a[m][0] <= o) lo = m; else hi = m; }
        const f = (o - a[lo][0]) / (a[hi][0] - a[lo][0]);
        t = a[lo][1] + f * (a[hi][1] - a[lo][1]); q = a[lo][2] + f * (a[hi][2] - a[lo][2]);
      }
    }
    return { T: t * scale * thrustFactor, Q: q * scale * torqueFactor };
  }
  function equilibrium(mp, duty, voltage, rho = RHO, sound = 340.294) {
    const volts = Math.max(0, duty * voltage - mp.brushDrop), maximum = volts / mp.Ke;
    const fastTip = maximum * mp.radius / sound > .7;
    const load = omega => {
      const loss = fastTip ? propLoss(mp, omega, omega * mp.radius / sound, sound) : null;
      return loadAt(mp, omega, rho, loss?.torque ?? 1, loss?.thrust ?? 1);
    };
    let lo = 0, hi = maximum;
    for (let n = 0; n < 30; n++) {
      const o = (lo + hi) / 2, q = load(o).Q + mp.friction;
      const i = Math.min(mp.iMax, Math.max(0, (volts - mp.Ke * o) / mp.R));
      if (mp.Ke * i >= q) lo = o; else hi = o;
    }
    const Omega = (lo + hi) / 2, result = load(Omega);
    return { Omega, T: result.T, Q: result.Q, i: Math.min(mp.iMax, Math.max(0, (volts - mp.Ke * Omega) / mp.R)) };
  }
  function motor(c) {
    const nominalThrust = Math.max(.0001, c.tmax || .0001);
    const diameter = 2 * (c.prop || 0.035 * Math.sqrt(nominalThrust)), fm = bounded(c.fm, .2, .95, .6);
    const Om = TAU * Math.sqrt(nominalThrust / (.10 * RHO * diameter ** 4));
    const kap = Math.sqrt(.10) * diameter / (TAU * fm * Math.sqrt(Math.PI / 2));
    const kT = nominalThrust / (Om * Om), kQ = kap * kT, Qm = kap * nominalThrust;
    const genericKe = .8 * 16 / Om, genericR = .2 * 16 * genericKe / Qm;
    const p = c.motorPhysics || {}, explicit = p.kind === 'brushless' || p.kind === 'brushed';
    const Ke = explicit ? 60 / (TAU * bounded(p.kv, 10, 20000, 60 / (TAU * genericKe))) : genericKe;
    const R = explicit ? bounded(p.resistance, .005, 20, genericR) : genericR;
    const mp = { Om, Ke, R, kT, kQ, rhoRef: RHO, kind: explicit ? p.kind : 'generic',
      iMax: explicit ? bounded(p.currentLimit, .1, 500, 2 * Qm / genericKe) : 2 * Qm / genericKe,
      brushDrop: p.kind === 'brushed' ? bounded(p.brushDrop, 0, 3, .6) : 0,
      friction: explicit ? bounded(p.friction, 0, 1, 0) : 0,
      refVoltage: 16, // portable controller's fixed reference voltage
      escEfficiency: bounded(p.escEfficiency, .5, 1, .97), idleW: bounded(p.idleW, 0, 20, .1),
      radius: diameter / 2, maxRpm: bounded(p.maxRpm, 0, 200000, 0), table: null };
    const prop = c.propPhysics || {};
    if (prop.rows) { mp.rhoRef = bounded(prop.referenceDensity, .3, 2, RHO); mp.table = propTable(prop.rows); }
    else if (explicit) {
      const ct = bounded(prop.ct, .01, .5, .10);
      mp.kT = ct * RHO * diameter ** 4 / (TAU * TAU);
      // Infer torque from figure of merit only when no measured torque curve is supplied.
      mp.kQ = kap * Math.sqrt(ct / .1) * mp.kT;
    }
    const Oh = Math.sqrt(.4) * Om;
    mp.J = explicit ? bounded(p.inertia, 1e-8, .1, Math.max(.005, c.tau || .03) * (Ke * Ke / R + 2 * kQ * Oh)) : Math.max(.005, c.tau || .03) * (Ke * Ke / R + 2 * kQ * Oh);
    const rated = equilibrium(mp, 1, mp.refVoltage, RHO);
    if (explicit || mp.table) { mp.Om = rated.Omega; mp.ratedThrust = rated.T; }
    else mp.ratedThrust = c.tmax;
    mp.ratedQ = rated.Q; mp.ratedCurrent = rated.i;
    mp.response = Array.from({length: 33}, (_, i) => equilibrium(mp, i / 32, 16, RHO).T);
    mp.thermal = { C: bounded(p.heatCapacity, .1, 10000, 500 * Math.max(.02, c.mass)),
      G: bounded(p.cooling, .001, 100, rated.i ** 2 * R / (1.25 * 95)), referenceLoss: rated.i ** 2 * R };
    return mp;
  }
  function step(Omega, duty, voltage, mp, dt, rho = RHO, qFactor = 1, tFactor = 1) {
    const drop = duty > 0 ? mp.brushDrop : 0;
    const i = Math.max(-.5 * mp.iMax, Math.min(mp.iMax, (duty * voltage - drop - mp.Ke * Omega) / mp.R));
    const load = loadAt(mp, Omega, rho, qFactor, tFactor), tau = mp.Ke * i;
    const friction = Omega > 0 ? mp.friction : Math.min(mp.friction, Math.max(0, tau));
    const damping = mp.Ke * mp.Ke / mp.R + 2 * load.Q / Math.max(1, Omega);
    const o = Math.max(0, Omega + (tau - load.Q - friction) * dt / (mp.J + dt * damping));
    return { Omega: o, i, tau, T: loadAt(mp, o, rho, qFactor, tFactor).T };
  }
  function compressibility(mach) {
    const x = Math.max(0, Math.min(1, (mach - .7) / .3));
    // Generic bounded approximation; measured curves take precedence for calibration.
    return { thrust: 1 - .25 * x * x, torque: 1 + .8 * x * x };
  }
  function propLoss(mp, omega, mach, sound) {
    const loss = compressibility(mach);
    if (mp.table) {
      // Static measured curves already include their rotational tip losses.
      const referenceMach = Math.min(omega, mp.table[mp.table.length - 1][0]) * mp.radius / sound;
      const reference = compressibility(referenceMach);
      loss.thrust /= reference.thrust; loss.torque /= reference.torque;
    }
    return loss;
  }
  function commandThrust(mp, duty, voltage, rho = RHO) {
    const x = Math.max(0, duty * voltage / 16) * 32;
    if (x >= 32) return mp.response[32] * (x / 32) ** 2 * rho / RHO;
    const i = Math.floor(x), f = x - i;
    return (mp.response[i] + f * (mp.response[i + 1] - mp.response[i])) * rho / RHO;
  }
  function batteryMass(parts, b) {
    for (const c of parts) if (c.type === 'mass' && c.battery && c.batteryAutoMass !== false) {
      const r = c.batterySizing;
      if (!r || !(r.capacity > 0 && r.cells > 0 && r.mass > 0)) c.batterySizing = { capacity: b.capacity, cells: b.cells, mass: c.mass };
      const ref = c.batterySizing;
      c.mass = ref.mass * b.capacity * b.cells / (ref.capacity * ref.cells);
    }
  }
  function polar(rows) {
    if (!Array.isArray(rows) || rows.length < 2 || rows.length > 64) throw new Error('Use 2–64 rows: angle degrees, lift coefficient, drag coefficient.');
    let prev = -Infinity;
    return rows.map((r, i) => {
      if (!Array.isArray(r) || r.length !== 3 || !r.every(x => typeof x === 'number' && Number.isFinite(x)) || r[0] <= prev || Math.abs(r[0]) > 180 || Math.abs(r[1]) > 5 || r[2] < 0 || r[2] > 5) throw new Error(`Invalid polar row ${i + 1}.`);
      prev = r[0]; return r.slice();
    });
  }
  return { RHO, atmosphere, motor, equilibrium, loadAt, step, propTable, parsePropTable, compressibility, propLoss, commandThrust, batteryMass, polar, bounded };
})();
if (typeof module !== 'undefined') module.exports = FlightPhysics;
