'use strict';
// ─────────────────────────────────────────────────────────────────────────────
// Governing formulas
//
// Every physical law and every control law the simulator uses is one of the functions below.
// The simulator only ever calls them by name, so changing a function here changes the default
// model. You can also edit any of them live in the Formulas tab; edits there are kept in your
// browser, and "Copy edited formulas" gives you text to paste back into this file.
//
// Units are SI throughout: m, kg, s, N, N·m, rad.
// Helpers from math.js (add, sub, scl, dot, crs, nrm, unit, clamp, m3v, m3m, m3T, m3inv, bls, G …)
// are available inside every formula.
// ─────────────────────────────────────────────────────────────────────────────

// ═════════════ Physics (the plant) ═════════════

function rigidBody(F, tau, m, J, Jinv, w) {
  // F: total force on the rigid body, world frame [N]
  // tau: total torque about the center of gravity, body frame [N·m]
  const a = scl(F, 1 / m);                               // Newton: linear acceleration, world
  const wdot = m3v(Jinv, sub(tau, crs(w, m3v(J, w))));  // Euler: angular acceleration, body
  return { a, wdot };
}

function gravity(m, g) {
  return [0, 0, -m * g];                                 // weight, world frame
}

function rotorWrench(d, r, T, spin, kappa) {
  // d: rotor axis (unit), r: rotor position from the CoG, both in the body frame
  const F = scl(d, T);                                   // thrust along the axis
  const tau = sub(crs(r, F), scl(d, spin * kappa * T));  // lever-arm torque + drag reaction torque
  return { F, tau };
}

function tiltAxis(theta, e) {
  // e: horizontal unit vector the rotor leans toward as theta grows
  return [Math.sin(theta) * e[0], Math.sin(theta) * e[1], Math.cos(theta)];
}

function motorResponse(T, Tcmd, tmax, tau, dt) {
  const target = clamp(Tcmd, 0, tmax);                   // props can't push backwards or exceed max
  return T + (target - T) * Math.min(1, dt / tau);       // first-order spin-up lag
}

function servoResponse(theta, target, range, rate, dt) {
  const goal = clamp(target, -range, range);             // mechanical limit
  const step = rate * dt;                                // maximum servo speed
  return theta + clamp(goal - theta, -step, step);
}

function bodyDrag(v, wind, w) {
  const cd = 0.1;                                        // frame drag [N per m/s of airspeed]; the rotors add their own (rotorAero)
  const cw = 0.002;                                      // rotational damping [N·m per rad/s]
  return { F: scl(sub(wind, v), cd), tau: scl(w, -cw) };
}

function cableTension(stretch, stretchRate, m) {
  if (stretch <= 0) return 0;                            // a slack cable carries no load
  const k = m * 15800;                                   // stiffness: ~20 Hz bounce for this payload
  const c = 2 * 0.25 * Math.sqrt(k * m);                 // 25% of critical damping
  return Math.max(0, k * stretch + c * stretchRate);     // cables pull, never push
}

function payloadDrag(v, wind) {
  return scl(sub(wind, v), 0.04);                        // [N per m/s of airspeed]
}

function groundContact(depth, v) {
  // depth: how far the contact point is below the ground [m]; v: its velocity, world frame
  const k = 3000, c = 60, mu = 8;                        // stiffness, damping, sliding friction
  return [-mu * v[0], -mu * v[1], Math.max(0, k * depth - c * v[2])];
}

// ─── Airflow. The simulator uses these to decide what really happens; the controller never sees them.

function wakeVelocity(point, rotors) {
  // Air velocity that the rotors' wakes induce at `point`, body frame. rotors: [{ p, d, T, R }]
  // (p: disc center, d: thrust axis, T: thrust [N], R: prop radius [m]).
  let w = [0, 0, 0];
  for (const r of rotors) {
    if (r.T <= 0) continue;
    const vh = Math.sqrt(r.T / (2 * 1.225 * Math.PI * r.R * r.R));  // induced velocity at the disc (momentum theory)
    const rel = sub(point, r.p), s = -dot(rel, r.d);               // how far downstream of the disc
    if (s < -r.R) continue;                                        // well above the disc: no effect
    const radial = nrm(add(rel, scl(r.d, s)));
    const Rw = s > 0 ? r.R * (0.71 + 0.29 * Math.exp(-s / r.R)) : r.R;   // the wake contracts to ~0.71 R
    const edge = clamp((1.1 * Rw - radial) / (0.2 * Rw), 0, 1);          // soft wake boundary
    if (edge <= 0) continue;
    const speed = vh * (1 + s / Math.sqrt(s * s + r.R * r.R))            // speeds up to 2·vh below the disc…
      * Math.exp(-Math.max(0, s) / (12 * r.R));                          // …then mixes out far downstream
    w = add(w, scl(r.d, -speed * edge));
  }
  return w;
}

function rotorAero(T, R, vAxial, vInPlane, h) {
  // T: thrust this command would make in still air [N]; R: prop radius [m]
  // vAxial: air flowing into the disc from above along its axis [m/s] (climbing, or another rotor's wake)
  // vInPlane: air velocity across the disc, body frame [m/s]; h: height of the disc above the ground [m]
  if (T <= 0) return { T: 0, H: [0, 0, 0] };
  const vh = Math.sqrt(T / (2 * 1.225 * Math.PI * R * R));          // induced velocity in hover
  const ve = nrm(vInPlane);
  let vi = vh;                                                      // induced velocity now (Glauert): edgewise flow lowers it
  for (let n = 0; n < 6; n++) vi = 0.5 * vi + 0.5 * vh * vh / Math.sqrt(ve * ve + (vAxial + vi) ** 2 + 1e-9);
  let k = clamp(1 - 0.5 * (vAxial + vi - vh) / vh, 0.3, 1.3);       // more air through the disc than in hover costs thrust
  const hh = Math.max(h, R / 2);
  k *= Math.min(1.3, 1 / (1 - (R / (4 * hh)) ** 2));                 // ground effect (Cheeseman–Bennett)
  const cH = 0.03;                                                  // rotor drag from blade flapping [1/(m/s)]
  return { T: T * k, H: scl(vInPlane, cH * T) };
}

function wakeLoad(w, area) {
  // Force on a part with frontal area [m²] sitting in wake air moving at w [m/s]
  return scl(w, 0.5 * 1.225 * 1.1 * area * nrm(w));
}

function batteryModel(st, load, dt) {
  // load: share of the motors' combined maximum thrust in use (0–1). Returns the thrust factor.
  const endurance = 360;                                            // seconds of flight at a 40% load
  if (st.soc === undefined) st.soc = 1;
  st.soc = Math.max(0, st.soc - (load / 0.4) * dt / endurance);
  const v = 0.86 + 0.14 * st.soc - 0.06 * load;                     // pack voltage vs full, sagging under load
  return v * v;                                                     // the same command gives thrust ∝ voltage²
}

// ═════════════ Sensors (what the hardware reports) ═════════════
// Each sensor keeps its own state in `st` between samples (biases, drifts). The simulator supplies the
// true quantity at the sensor's position and mount, and vibration or interference where it applies.
// randn() gives a standard normal random number.

function imuModel(w, f, vib, p, st, dt) {
  // w: true angular rate, f: true specific force (what an ideal accelerometer feels), both in the sensor frame
  // vib: { a, w } motor vibration at the sensor; p: noise, bias and range settings in SI units
  if (!st.bg) {                                          // turn-on bias, different every power-up
    st.bg = [0, 1, 2].map(() => randn() * p.gyroBias);
    st.ba = [0, 1, 2].map(() => randn() * p.accBias);
  }
  for (let i = 0; i < 3; i++) st.bg[i] += randn() * p.gyroDrift * Math.sqrt(dt);   // gyro bias random walk
  const gyro = [0, 1, 2].map(i => clamp(w[i] + vib.w[i] + st.bg[i] + randn() * p.gyroNoise, -p.gyroRange, p.gyroRange));
  const accel = [0, 1, 2].map(i => clamp(f[i] + vib.a[i] + st.ba[i] + randn() * p.accNoise, -p.accRange, p.accRange));
  return { gyro, accel };
}

function magModel(b, interference, p, st) {
  // b: Earth's field in the sensor frame (strength 1); interference: field from nearby motor currents
  if (!st.hi) st.hi = [0, 1, 2].map(() => randn() * p.hardIron);   // hard-iron offset from the airframe
  return [0, 1, 2].map(i => b[i] + st.hi[i] + interference[i] + randn() * p.noise);
}

function baroModel(alt, p, st, dt) {
  // alt: true altitude of the sensor [m]
  if (st.drift === undefined) st.drift = 0;
  st.drift += randn() * p.drift * Math.sqrt(dt);         // weather and temperature drift
  return alt + st.drift + randn() * p.noise;
}

function posFixModel(pos, vel, p, st, dt) {
  // pos, vel: true position and velocity of the antenna or marker, world frame
  const tau = 30;                                        // how slowly the error wanders [s]
  if (!st.e) st.e = [0, 1, 2].map(() => randn() * p.wander);
  const a = Math.exp(-dt / tau), s = p.wander * Math.sqrt(1 - a * a);
  st.e = st.e.map(x => a * x + s * randn());             // slowly wandering error (Gauss–Markov)
  return {
    p: [0, 1, 2].map(i => pos[i] + st.e[i] * (i === 2 ? 1.5 : 1) + randn() * p.noise),
    v: [0, 1, 2].map(i => vel[i] + randn() * p.velNoise),
  };
}

// ═════════════ Estimation (what the flight software believes) ═════════════

function attitudeEstimator(st, gyro, accel, mag, dt) {
  // Mahony complementary filter. gyro, accel, mag are body-frame readings (mag is null without a compass).
  // Integrates the gyro and slowly pulls the estimate toward "up" from the accelerometer and "north"
  // from the compass. The integral term learns the gyro bias.
  // A multirotor's accelerometer feels thrust, not gravity, while it accelerates, so "up" is only
  // trusted when the (vibration-filtered) reading is very close to 1 g.
  const kP = 0.6, kI = 0.08, kMag = 0.4;                // [1/s], [1/s²], [1/s]
  const accCutoff = 8, rateCutoff = 60, gate = 0.05;     // [Hz], [Hz], fraction of g
  if (!st.q) {                                           // initial alignment from the first readings
    const up = unit(accel);
    let north = mag ? sub(mag, scl(up, dot(mag, up))) : sub([1, 0, 0], scl(up, up[0]));
    north = unit(north);
    const east = crs(up, north);                         // world axes expressed in the body frame
    st.q = matToQuat([north[0], north[1], north[2], east[0], east[1], east[2], up[0], up[1], up[2]]);
    st.ie = [0, 0, 0]; st.w = gyro.slice(); st.af = accel.slice();
  }
  const lp = fc => dt / (dt + 1 / (2 * Math.PI * fc));
  st.af = st.af.map((x, i) => x + lp(accCutoff) * (accel[i] - x));
  const R = qmat(st.q);
  let e = [0, 0, 0];
  const n = nrm(st.af);
  const trust = clamp(1 - Math.abs(n - G) / (gate * G), 0, 1);
  if (trust > 0) e = add(e, scl(crs(scl(st.af, 1 / n), [R[6], R[7], R[8]]), kP * trust));
  if (mag) {
    const mw = m3v(R, mag);                              // compass reading in the estimated world frame
    const yawErr = Math.atan2(mw[1], mw[0]);             // it should point north (+X)
    e = add(e, m3v(m3T(R), [0, 0, -kMag * yawErr]));
  }
  st.ie = st.ie.map((x, i) => clamp(x + e[i] * dt, -1, 1));
  const wb = add(gyro, scl(st.ie, kI / kP));            // bias-corrected rate
  const wc = add(wb, e);                                 // rate used to propagate the attitude
  const dq = qmul(st.q, [0, wc[0], wc[1], wc[2]]);
  st.q = qnorm(st.q.map((x, i) => x + 0.5 * dq[i] * dt));
  st.w = st.w.map((x, i) => x + lp(rateCutoff) * (wb[i] - x));   // low-passed rate for the D-term
  return { q: st.q.slice(), w: st.w.slice() };
}

function positionEstimator(st, R, accel, baro, fix, m, dt) {
  // Complementary filter. Integrates the accelerometer (rotated by the attitude estimate) and pulls the
  // result toward the position fix and the barometer. baro and fix are null when not available; their
  // `age` says how long ago they were measured, so they are compared with the estimate from that moment.
  // The simulator seeds st.p and st.v with the start point at reset.
  const kP = 0.8, kV = 0.3, kFixV = 0.6, kBaro = 1.5, kBaroV = 0.6;
  const cd = 0.25, kDrag = 1.0;                          // airframe drag coefficient [N per m/s] (see bodyDrag), drag-fusion gain
  if (!st.p) { st.p = fix ? fix.p.slice() : [0, 0, baro ? baro.alt : 0]; st.v = [0, 0, 0]; }
  if (!st.h) { st.h = []; st.af = accel.slice(); }       // recent estimates (newest last), filtered accel
  const a = add(m3v(R, accel), [0, 0, -G]);              // world acceleration from the IMU
  st.v = add(st.v, scl(a, dt)); st.p = add(st.p, scl(st.v, dt));
  const past = age => st.h[Math.max(0, st.h.length - 1 - Math.round(age / dt))] || { p: st.p, v: st.v };
  if (fix) {
    const then = past(fix.age);
    const e = sub(fix.p, then.p); if (baro) e[2] = 0;     // the barometer owns altitude when present
    st.p = add(st.p, scl(e, kP * dt)); st.v = add(st.v, scl(e, kV * dt));
    st.v = add(st.v, scl(sub(fix.v, then.v), kFixV * dt));
  } else {
    // No fix. Thrust only pushes along body Z, so the sideways accelerometer reading is air drag,
    // which reveals the body's airspeed: v_xy ≈ −(m / cd)·f_xy. Pull the estimate toward it.
    const k = dt / (dt + 1 / (2 * Math.PI * 2));
    st.af = st.af.map((x, i) => x + k * (accel[i] - x));
    const vb = m3v(m3T(R), st.v);
    const dv = [-(m / cd) * st.af[0] - vb[0], -(m / cd) * st.af[1] - vb[1], 0];
    st.v = add(st.v, scl(m3v(R, dv), kDrag * dt));
  }
  if (baro) { const e = baro.alt - past(baro.age).p[2]; st.p[2] += kBaro * e * dt; st.v[2] += kBaroV * e * dt; }
  st.h.push({ p: st.p.slice(), v: st.v.slice() }); if (st.h.length > 800) st.h.shift();
  return { p: st.p.slice(), v: st.v.slice() };
}

// ═════════════ Identification (learning what the actuators do) ═════════════

function identifyEffectiveness(st, u, f, w, r, dt, init, memory) {
  // Recursive least squares. Learns B, what each actuator input does to the drone, from flight data:
  //   Δ[f; dω/dt] ≈ B · Δu
  // Both sides are band-passed (0.3–12 Hz), so it learns from changes and steady offsets such as drag
  // or trim can't leak into B.
  // u: inputs the controller sent (throttle fractions; a tilting rotor counts twice, as u·cosθ and u·sinθ)
  // f: accelerometer (specific force) and w: gyro, body frame; r: IMU position from the hub [m]
  // init: starting guess, 6 rows × inputs; memory: how long past data counts [s].
  // Returns B: 6 rows (ax ay az αx αy αz) × inputs.
  const tauAct = 0.035, lpHz = 12, hpHz = 0.3, pMax = 2;    // assumed motor lag [s], band [Hz], covariance cap
  const n = u.length;
  const k = hz => dt / (dt + 1 / (2 * Math.PI * hz));
  if (dt <= 0) return { B: st.th || init };
  if (!st.P) {                                                   // start the filters from the first real reading
    st.th = init.map(row => row.slice());
    st.P = eye(n).map(row => row.map(x => x * 2));
    st.ua = u.slice(); st.xl = u.slice(); st.xs = u.slice(); st.yl = f.concat([0, 0, 0]); st.ys = st.yl.slice();
    st.wl = w.slice(); st.n = 0;
  }
  for (let j = 0; j < n; j++) st.ua[j] += (u[j] - st.ua[j]) * Math.min(1, dt / tauAct);   // what the motors are doing now
  const wPrev = st.wl.slice();
  st.wl = st.wl.map((v, i) => v + k(lpHz) * (w[i] - v));
  const alpha = st.wl.map((v, i) => (v - wPrev[i]) / dt);                     // angular acceleration
  const fHub = sub(sub(f, crs(alpha, r)), crs(st.wl, crs(st.wl, r)));         // remove the IMU's lever-arm swing
  const yRaw = [...fHub, ...alpha];
  st.xl = st.xl.map((v, j) => v + k(lpHz) * (st.ua[j] - v));
  st.yl = st.yl.map((v, i) => i < 3 ? v + k(lpHz) * (yRaw[i] - v) : alpha[i - 3]);   // alpha is already filtered once, like the inputs
  st.xs = st.xs.map((v, j) => v + k(hpHz) * (st.xl[j] - v));
  st.ys = st.ys.map((v, i) => v + k(hpHz) * (st.yl[i] - v));
  if (++st.n * dt < 1.5) return { B: st.th };                              // let the filters settle first
  const x = st.xl.map((v, j) => v - st.xs[j]), y = st.yl.map((v, i) => v - st.ys[i]);
  const P = st.P, Px = P.map(row => row.reduce((s, v, j) => s + v * x[j], 0));
  const lambda = Math.exp(-dt / memory), den = lambda + x.reduce((s, v, j) => s + v * Px[j], 0);
  const K = Px.map(v => v / den);
  st.e = st.th.map((row, i) => y[i] - row.reduce((s, v, j) => s + v * x[j], 0));   // prediction error before learning
  st.y = y; st.x = x;
  st.th = st.th.map((row, i) => row.map((v, j) => v + K[j] * st.e[i]));
  let trace = 0;
  for (let a = 0; a < n; a++) for (let b = 0; b < n; b++) { P[a][b] = (P[a][b] - K[a] * Px[b]) / lambda; if (a === b) trace += P[a][a]; }
  if (trace > pMax * n) for (const row of P) for (let b = 0; b < n; b++) row[b] *= pMax * n / trace;   // don't blow up without excitation
  return { B: st.th };
}

// ═════════════ Controller ═════════════

function positionControl(ep, v, ip, m, g) {
  // ep: position error, v: velocity error (hub velocity − commanded velocity), ip: integral of ep; world frame
  const kp = 4, kd = 3.6, ki = 1.0;                      // acceleration units, so they fit any mass
  const a = [0, 1, 2].map(i => kp * ep[i] - kd * v[i] + ki * ip[i]);
  const ah = Math.hypot(a[0], a[1]);
  if (ah > 6) { a[0] *= 6 / ah; a[1] *= 6 / ah; }        // limit the horizontal demand
  a[2] = clamp(a[2], -6, 8);
  return scl(add(a, [0, 0, g]), m);                      // desired total force, world frame
}

function thrustAxisTarget(Fd, mode) {
  if (mode === 'level') return [0, 0, 1];                // keep the thrust axis vertical
  const maxTilt = 35 * Math.PI / 180;
  let n = unit(Fd);                                      // point the thrust axis along the demand
  if (Math.acos(clamp(n[2], -1, 1)) > maxTilt) {
    const h = Math.hypot(Fd[0], Fd[1]) || 1;
    n = [Fd[0] / h * Math.sin(maxTilt), Fd[1] / h * Math.sin(maxTilt), Math.cos(maxTilt)];
  }
  return n;
}

function attitudeError(R, Rd) {
  // R: current attitude, Rd: desired attitude (body → world rotation matrices)
  const E = m3m(m3T(Rd), R), Et = m3T(E);
  return [0.5 * (E[7] - Et[7]), 0.5 * (E[2] - Et[2]), 0.5 * (E[3] - Et[3])];  // vee(½(E − Eᵀ))
}

function attitudeControl(eR, w, ia, J) {
  const kR = [100, 100, 40], kW = [16, 16, 10], kI = [80, 80, 20];
  const alpha = [0, 1, 2].map(i => -kR[i] * eR[i] - kW[i] * w[i] - kI[i] * ia[i]);
  return add(m3v(J, alpha), crs(w, m3v(J, w)));         // torque = J·α + ω × Jω
}

function forceDemand(Fb, n, mode) {
  // Fb: desired force in the body frame, n: nominal thrust axis
  if (mode === 'level') return Fb;                       // thrust vectoring handles sideways force
  return scl(n, dot(Fb, n));                             // only the thrust axis can push
}

function allocation(cols, lo, hi, wd, mode) {
  // cols[j]: what one unit of input j does, as [ax, ay, az, αx, αy, αz] (acceleration units)
  // wd: the 6 accelerations wanted; lo/hi: input limits
  const W = mode === 'level' ? [3, 3, 3, 10, 10, 1] : [0.3, 0.3, 3, 10, 10, 1];
  return bls(cols, lo, hi, wd, W);                       // bounded weighted least squares
}

// ─────────────────────────────────────────────────────────────────────────────
// Descriptions used by the Formulas tab. `sample` gives inputs for the test call made when you
// apply an edit; `shape` is what the function must return.
// ─────────────────────────────────────────────────────────────────────────────
const V = s => `<span class="v">${s}</span>`;
const LAW_DEFS = [
  { key: 'rigidBody', group: 'plant', fn: rigidBody, title: 'Rigid-body motion',
    math: [`${V('a')} = ${V('F')} / <i>m</i>`, `${V('ω̇')} = <i>J</i><sup>−1</sup>(${V('τ')} − ${V('ω')} × <i>J</i>${V('ω')})`],
    doc: 'Newton–Euler equations for one rigid body. The simulator integrates the result with a 0.5 ms step.',
    args: [['F', 'total force, world [N]'], ['tau', 'total torque about the CoG, body [N·m]'], ['m', 'rigid mass [kg]'], ['J', 'inertia about the CoG'], ['Jinv', 'J⁻¹'], ['w', 'angular velocity, body [rad/s]']],
    returns: '{ a: linear acceleration, world; wdot: angular acceleration, body }',
    shape: { a: 3, wdot: 3 },
    sample: () => [[0, 0, 9.81], [0.01, 0, 0], 1, [.01, 0, 0, 0, .01, 0, 0, 0, .02], [100, 0, 0, 0, 100, 0, 0, 0, 50], [0.1, 0, 0]] },
  { key: 'gravity', group: 'plant', fn: gravity, title: 'Weight',
    math: [`${V('F')}<sub>g</sub> = −<i>m g</i> ${V('ẑ')}`],
    doc: 'Applied at the center of gravity of the frame and of each cable payload.',
    args: [['m', 'mass [kg]'], ['g', 'gravitational acceleration, 9.81 m/s²']], returns: 'force, world [N]',
    shape: 3, sample: () => [1, 9.81] },
  { key: 'rotorWrench', group: 'plant', fn: rotorWrench, title: 'Rotor force and torque',
    math: [`${V('F')}<sub>i</sub> = <i>T</i><sub>i</sub> ${V('d')}<sub>i</sub>`, `${V('τ')}<sub>i</sub> = ${V('r')}<sub>i</sub> × ${V('F')}<sub>i</sub> − <i>s</i><sub>i</sub> κ<sub>i</sub> <i>T</i><sub>i</sub> ${V('d')}<sub>i</sub>`],
    doc: 'The contribution of one rotor. s is +1 for counter-clockwise spin seen from above, −1 for clockwise. κ is the drag-torque to thrust ratio.',
    used: 'Also builds the controller\'s effectiveness matrix and the flight envelope, evaluated at T = 1 N, so those assume the law is linear in T.',
    args: [['d', 'rotor axis (unit), body'], ['r', 'rotor position from the CoG, body [m]'], ['T', 'delivered thrust [N]'], ['spin', '+1 CCW, −1 CW'], ['kappa', 'drag torque ratio κ [m]']],
    returns: '{ F: force, body; tau: torque, body }',
    shape: { F: 3, tau: 3 }, sample: () => [[0, 0, 1], [0.2, 0.1, 0], 3, 1, 0.016] },
  { key: 'tiltAxis', group: 'plant', fn: tiltAxis, title: 'Servo tilt geometry',
    math: [`${V('d')}(θ) = sin θ ${V('e')} + cos θ ${V('ẑ')}`],
    doc: 'Direction of a servo-mounted rotor\'s axis at servo angle θ. e is set by the hinge direction.',
    used: 'Also used by the controller and envelope. Stage 1 of allocation picks servo angles assuming this sin/cos form.',
    args: [['theta', 'servo angle [rad]'], ['e', 'lean direction, body (unit)']], returns: 'rotor axis, body (unit)',
    shape: 3, sample: () => [0.2, [0, -1, 0]] },
  { key: 'motorResponse', group: 'plant', fn: motorResponse, title: 'Motor response',
    math: [`<i>T</i><sub>k+1</sub> = <i>T</i><sub>k</sub> + (sat(<i>T</i><sub>cmd</sub>) − <i>T</i><sub>k</sub>) · Δ<i>t</i> / τ<sub>m</sub>`],
    doc: 'How a motor\'s thrust follows its command. Health scales the thrust afterwards.',
    args: [['T', 'current thrust [N]'], ['Tcmd', 'commanded thrust [N]'], ['tmax', 'max thrust [N]'], ['tau', 'spin-up time constant [s]'], ['dt', 'time step [s]']], returns: 'next thrust [N]',
    shape: 'n', sample: () => [1, 2, 6, 0.03, 0.0005] },
  { key: 'servoResponse', group: 'plant', fn: servoResponse, title: 'Servo response',
    math: [`θ<sub>k+1</sub> = θ<sub>k</sub> + sat<sub>±ω<sub>max</sub>Δt</sub>(sat<sub>±θ<sub>max</sub></sub>(θ<sub>cmd</sub>) − θ<sub>k</sub>)`],
    doc: 'A rate-limited servo with a mechanical limit. No backlash or load sag yet.',
    args: [['theta', 'current angle [rad]'], ['target', 'commanded angle [rad]'], ['range', 'limit ± [rad]'], ['rate', 'max speed [rad/s]'], ['dt', 'time step [s]']], returns: 'next angle [rad]',
    shape: 'n', sample: () => [0, 0.3, 0.6, 4, 0.0005] },
  { key: 'bodyDrag', group: 'plant', fn: bodyDrag, title: 'Aerodynamic drag',
    math: [`${V('F')}<sub>d</sub> = <i>c</i><sub>d</sub>(${V('v')}<sub>wind</sub> − ${V('v')})`, `${V('τ')}<sub>d</sub> = −<i>c</i><sub>ω</sub> ${V('ω')}`],
    doc: 'Linear drag on the airframe and a little rotational damping.',
    args: [['v', 'velocity, world [m/s]'], ['wind', 'wind velocity, world [m/s]'], ['w', 'angular velocity, body [rad/s]']], returns: '{ F: force, world; tau: torque, body }',
    shape: { F: 3, tau: 3 }, sample: () => [[1, 0, 0], [0, 0, 0], [0, 0, 0.5]] },
  { key: 'cableTension', group: 'plant', fn: cableTension, title: 'Cable tension',
    math: [`<i>T</i><sub>c</sub> = max(0, <i>k</i>δ + <i>c</i>δ̇) &nbsp;if δ > 0, otherwise 0`, `<i>k</i> = 15800 <i>m</i><sub>p</sub>, &nbsp;<i>c</i> = 0.5 √(<i>k m</i><sub>p</sub>)`],
    doc: 'δ is how far the cable is stretched beyond its length. The tension pulls the payload toward the anchor and the anchor toward the payload.',
    args: [['stretch', 'δ [m]'], ['stretchRate', 'δ̇ [m/s]'], ['m', 'payload mass [kg]']], returns: 'tension [N]',
    shape: 'n', sample: () => [0.001, 0.01, 0.2] },
  { key: 'payloadDrag', group: 'plant', fn: payloadDrag, title: 'Payload drag',
    math: [`${V('F')} = 0.04 (${V('v')}<sub>wind</sub> − ${V('v')}<sub>p</sub>)`],
    doc: 'Drag on a cable payload. Its weight comes from the Weight law.',
    args: [['v', 'payload velocity, world [m/s]'], ['wind', 'wind velocity, world [m/s]']], returns: 'force, world [N]',
    shape: 3, sample: () => [[1, 0, 0], [0, 0, 0]] },
  { key: 'groundContact', group: 'plant', fn: groundContact, title: 'Ground contact',
    math: [`${V('F')} = (−μ<i>v</i><sub>x</sub>, −μ<i>v</i><sub>y</sub>, max(0, <i>k</i><sub>g</sub><i>h</i> − <i>c</i><sub>g</sub><i>v</i><sub>z</sub>))`],
    doc: 'A penalty spring at each contact point (hub, motors, masses) that is below the ground.',
    args: [['depth', 'h, depth below ground [m]'], ['v', 'point velocity, world [m/s]']], returns: 'force, world [N]',
    shape: 3, sample: () => [0.01, [0.1, 0, -0.5]] },

  { key: 'wakeVelocity', group: 'plant', fn: wakeVelocity, title: 'Rotor wakes',
    math: [`<i>v</i><sub>h</sub> = √(<i>T</i> / 2ρ<i>A</i>), &nbsp;${V('w')}(<i>s</i>) = −${V('d')} <i>v</i><sub>h</sub>(1 + <i>s</i>/√(<i>s</i>² + <i>R</i>²)) · e<sup>−<i>s</i>/12<i>R</i></sup> &nbsp;inside the wake`, `wake radius <i>R</i>(0.71 + 0.29 e<sup>−<i>s</i>/<i>R</i></sup>), &nbsp;<i>s</i> = distance downstream`],
    doc: 'Each rotor blows a column of air along −d that speeds up to twice its induced velocity and contracts. Another rotor inside that column loses thrust; parts and payloads inside it get pushed. Physics only: the controller never uses this.',
    args: [['point', 'where to evaluate, body frame [m]'], ['rotors', '[{ p, d, T, R }] disc center, axis, thrust, prop radius']],
    returns: 'induced air velocity, body frame [m/s]', shape: 3, sample: () => [[0, 0, -0.1], [{ p: [0, 0, 0], d: [0, 0, 1], T: 3, R: 0.08 }]] },
  { key: 'rotorAero', group: 'plant', fn: rotorAero, title: 'Rotor aerodynamics',
    math: [`<i>v</i><sub>i</sub> = <i>v</i><sub>h</sub>² / √(<i>V</i><sub>edge</sub>² + (<i>v</i><sub>ax</sub> + <i>v</i><sub>i</sub>)²) &nbsp;(Glauert)`, `<i>T</i><sub>eff</sub> = <i>T</i> · sat(1 − 0.5 (<i>v</i><sub>ax</sub> + <i>v</i><sub>i</sub> − <i>v</i><sub>h</sub>)/<i>v</i><sub>h</sub>) · 1/(1 − (<i>R</i>/4<i>h</i>)²)`, `${V('H')} = <i>c</i><sub>H</sub> <i>T</i> ${V('u')}<sub>in-plane</sub> &nbsp;(rotor drag)`],
    doc: 'Air coming down through the disc (from climbing or from another rotor\'s wake) costs thrust at the same command. Air crossing the disc edgewise, in forward flight, lowers the induced velocity and adds a little thrust (translational lift). The ground adds thrust within about a rotor diameter. Air moving across the disc tilts it back and makes rotor drag, which is most of a multirotor\'s drag.',
    args: [['T', 'still-air thrust for this command [N]'], ['R', 'prop radius [m]'], ['vAxial', 'inflow from above along the axis [m/s]'], ['vInPlane', 'air velocity across the disc, body frame [m/s]'], ['h', 'height above ground [m]']],
    returns: '{ T: effective thrust [N]; H: rotor drag force, body frame [N] }', shape: { T: 1, H: 3 }, sample: () => [3, 0.08, 0.5, [1, 0, 0], 2] },
  { key: 'wakeLoad', group: 'plant', fn: wakeLoad, title: 'Downwash on parts',
    math: [`${V('F')} = ½ ρ <i>C</i><sub>d</sub> <i>A</i> |${V('w')}| ${V('w')}`],
    doc: 'Rotor wash hitting the hub, rigid masses and cable payloads pushes them along the wake.',
    args: [['w', 'wake air velocity at the part [m/s]'], ['area', 'frontal area of the part [m²]']], returns: 'force [N]', shape: 3, sample: () => [[0, 0, -5], 0.01] },
  { key: 'batteryModel', group: 'plant', fn: batteryModel, title: 'Battery',
    math: [`<i>V</i>/<i>V</i><sub>full</sub> = 0.86 + 0.14·SoC − 0.06·load, &nbsp;thrust × (<i>V</i>/<i>V</i><sub>full</sub>)²`],
    doc: 'The pack drains with load and its voltage sags, so the same command gives less thrust as the flight goes on. Reset restores a full pack.',
    args: [['st', 'battery state (soc)'], ['load', 'share of combined max thrust in use'], ['dt', 'time step [s]']], returns: 'thrust factor', shape: 'n', sample: () => [{}, 0.4, 0.0005] },

  { key: 'imuModel', group: 'sensor', fn: imuModel, title: 'IMU (gyro + accelerometer)',
    math: [`${V('ω̃')} = sat(${V('ω')}<sub>s</sub> + ${V('ω')}<sub>vib</sub> + ${V('b')}<sub>g</sub> + ${V('n')}<sub>g</sub>), &nbsp;${V('ḃ')}<sub>g</sub> = random walk`, `${V('f̃')} = sat(<i>R</i><sub>s</sub><sup>T</sup>(<i>R</i><sup>T</sup>(${V('a')} − ${V('g')}) + ${V('ω̇')} × ${V('r')} + ${V('ω')} × (${V('ω')} × ${V('r')})) + ${V('a')}<sub>vib</sub> + ${V('b')}<sub>a</sub> + ${V('n')}<sub>a</sub>)`],
    doc: 'r is the IMU\'s offset from the center of gravity, so an off-center accelerometer also feels rotation. Vibration is a sum of sinusoids at each motor\'s rotation frequency, stronger near busy motors; a slow IMU rate aliases it into low frequencies.',
    args: [['w', 'true angular rate, sensor frame [rad/s]'], ['f', 'true specific force, sensor frame [m/s²]'], ['vib', '{ a, w } vibration at the sensor'], ['p', 'gyroNoise, gyroBias, gyroDrift, gyroRange, accNoise, accBias, accRange'], ['st', 'this sensor\'s state'], ['dt', 'sample period [s]']],
    returns: '{ gyro, accel }, sensor frame', shape: { gyro: 3, accel: 3 },
    sample: () => [[0.1, 0, 0], [0, 0, 9.81], { a: [0, 0, 0], w: [0, 0, 0] }, { gyroNoise: 0.002, gyroBias: 0.01, gyroDrift: 0.0003, gyroRange: 35, accNoise: 0.05, accBias: 0.05, accRange: 157 }, {}, 0.001] },
  { key: 'magModel', group: 'sensor', fn: magModel, title: 'Compass',
    math: [`${V('m̃')} = <i>R</i><sub>s</sub><sup>T</sup><i>R</i><sup>T</sup>${V('m')}<sub>earth</sub> + ${V('b')}<sub>hard iron</sub> + ${V('m')}<sub>motors</sub> + ${V('n')}`],
    doc: 'Earth\'s field has strength 1 and points north (+X) and 60° down. Motor currents add a field that grows with throttle and falls off quickly with distance, so where you mount the compass matters.',
    args: [['b', 'Earth field, sensor frame'], ['interference', 'motor field at the sensor, sensor frame'], ['p', 'noise, hardIron'], ['st', 'this sensor\'s state']],
    returns: 'field reading, sensor frame', shape: 3, sample: () => [[0.5, 0, -0.866], [0, 0, 0], { noise: 0.01, hardIron: 0.05 }, {}] },
  { key: 'baroModel', group: 'sensor', fn: baroModel, title: 'Barometer',
    math: [`<i>h̃</i> = <i>h</i> + <i>d</i> + <i>n</i>, &nbsp;<i>ḋ</i> = random walk`],
    doc: 'Altitude from air pressure, with white noise and a slow drift.',
    args: [['alt', 'true altitude of the sensor [m]'], ['p', 'noise, drift'], ['st', 'this sensor\'s state'], ['dt', 'sample period [s]']],
    returns: 'altitude reading [m]', shape: 'n', sample: () => [1.5, { noise: 0.15, drift: 0.01 }, {}, 0.02] },
  { key: 'posFixModel', group: 'sensor', fn: posFixModel, title: 'Position fix',
    math: [`${V('p̃')} = ${V('p')}<sub>s</sub> + ${V('e')} + ${V('n')}, &nbsp;${V('ė')} = −${V('e')}/τ + wander`, `${V('ṽ')} = ${V('v')}<sub>s</sub> + ${V('n')}<sub>v</sub>`],
    doc: 'GPS, RTK or motion capture, depending on the settings. The wandering error is what makes a GPS drone drift slowly while hovering; vertical error is 1.5× horizontal.',
    args: [['pos', 'true antenna position, world [m]'], ['vel', 'true antenna velocity, world [m/s]'], ['p', 'noise, wander, velNoise'], ['st', 'this sensor\'s state'], ['dt', 'sample period [s]']],
    returns: '{ p, v }, world', shape: { p: 3, v: 3 }, sample: () => [[0, 0, 1.5], [0, 0, 0], { noise: 0.2, wander: 0.6, velNoise: 0.1 }, {}, 0.2] },

  { key: 'attitudeEstimator', group: 'est', fn: attitudeEstimator, title: 'Attitude estimator',
    math: [`${V('e')} = ${V('ã')} × ${V('û')} + <i>R̂</i><sup>T</sup>(0, 0, −ψ<sub>err</sub>)`, `${V('ω')}<sub>c</sub> = ${V('ω̃')} + <i>K</i><sub>I</sub>∫${V('e')} d<i>t</i> + <i>K</i><sub>P</sub>${V('e')}, &nbsp; <i>q̂̇</i> = ½ <i>q̂</i> ⊗ ${V('ω')}<sub>c</sub>`],
    doc: 'Mahony complementary filter. A multirotor\'s accelerometer feels thrust rather than gravity whenever it accelerates, so the filter only trusts it for "up" when the low-passed reading is within 5% of 1 g. Without a compass, heading drifts with the gyro bias.',
    args: [['st', 'estimator state'], ['gyro', 'fused gyro reading, body [rad/s]'], ['accel', 'fused accelerometer reading, body [m/s²]'], ['mag', 'fused compass reading, body, or null'], ['dt', 'control period [s]']],
    returns: '{ q: attitude quaternion [w, x, y, z]; w: filtered rate, body }', shape: { q: 4, w: 3 },
    sample: () => [{}, [0, 0, 0], [0, 0, 9.81], [0.5, 0, -0.866], 0.001] },
  { key: 'positionEstimator', group: 'est', fn: positionEstimator, title: 'Position estimator',
    math: [`${V('v̂̇')} = <i>R̂</i>${V('f̃')} + ${V('g')} + <i>k</i><sub>V</sub>(${V('p̃')} − ${V('p̂')}) + <i>k</i><sub>fv</sub>(${V('ṽ')} − ${V('v̂')})`, `${V('p̂̇')} = ${V('v̂')} + <i>k</i><sub>P</sub>(${V('p̃')} − ${V('p̂')}), &nbsp;altitude from the barometer when present`, `no fix: ${V('v̂')}<sub>xy</sub> → −(<i>m</i>/<i>c</i><sub>d</sub>) ${V('f̃')}<sub>xy</sub> &nbsp;(drag fusion)`],
    doc: 'Fuses the accelerometer with the position fix and barometer. The simulator first shifts each reading to the frame hub using the sensor positions the controller knows, and reports how old it is so a delayed fix is compared with the estimate from when it was measured. Without a fix it falls back on drag fusion: a multirotor\'s accelerometer feels air drag sideways, which reveals airspeed, so wind and a wrong drag coefficient make it drift.',
    args: [['st', 'estimator state'], ['R', 'estimated attitude matrix'], ['accel', 'fused accelerometer reading, body'], ['baro', '{ alt, age } hub altitude, or null'], ['fix', '{ p, v, age } hub position and velocity, or null'], ['m', 'modeled mass [kg]'], ['dt', 'control period [s]']],
    returns: '{ p: hub position; v: hub velocity }, world', shape: { p: 3, v: 3 },
    sample: () => [{}, [1, 0, 0, 0, 1, 0, 0, 0, 1], [0, 0, 9.81], { alt: 1.5, age: 0.02 }, { p: [0, 0, 1.5], v: [0, 0, 0], age: 0.15 }, 1, 0.001] },

  { key: 'identifyEffectiveness', group: 'learn', fn: identifyEffectiveness, title: 'Effectiveness identification',
    math: [`Δ[${V('f̃')} − ${V('ω̇')}×${V('r')} − ${V('ω')}×(${V('ω')}×${V('r')}); ${V('ω̃̇')}] ≈ <i>B̂</i> Δ${V('u')}, &nbsp;both sides band-passed 0.3–12 Hz`, `<i>K</i> = <i>P</i>${V('x')} / (λ + ${V('x')}<sup>T</sup><i>P</i>${V('x')}), &nbsp;<i>B̂</i> += ${V('e')}<i>K</i><sup>T</sup>, &nbsp;<i>P</i> = (<i>P</i> − <i>K</i>${V('x')}<sup>T</sup><i>P</i>)/λ`],
    doc: 'Recursive least squares with forgetting (about 4 s of memory). It learns, straight from the accelerometer and gyro, how much acceleration and angular acceleration each actuator input produces. That covers mass, inertia, prop thrust, rotor wakes and battery sag without being told any of them. It learns from changes, so steady offsets like drag can\'t leak in, and it removes the accelerometer\'s lever-arm swing using the IMU position the controller knows. Runs during calibration and, if you leave learning on, all through the flight.',
    args: [['st', 'identification state'], ['u', 'inputs sent, throttle fractions (tilting rotors as u·cosθ, u·sinθ)'], ['f', 'accelerometer, body [m/s²]'], ['w', 'gyro, body [rad/s]'], ['r', 'IMU position from the hub [m]'], ['dt', 'control period [s]'], ['init', 'starting guess, 6 rows'], ['memory', 'forgetting time [s]: short while calibrating, long in flight']],
    returns: '{ B: 6 rows × inputs }', shape: { B: 'rows' },
    sample: () => [{}, [0.5, 0.5], [0, 0, 9.81], [0, 0, 0], [0, 0, 0.01], 0.001, [[0, 0], [0, 0], [10, 10], [100, -100], [0, 0], [1, -1]], 4] },

  { key: 'positionControl', group: 'ctrl', fn: positionControl, title: 'Position control',
    math: [`${V('a')}<sub>d</sub> = <i>K</i><sub>p</sub>${V('e')}<sub>p</sub> − <i>K</i><sub>d</sub>(${V('v')} − ${V('v')}<sub>cmd</sub>) + <i>K</i><sub>i</sub>∫${V('e')}<sub>p</sub> d<i>t</i>`, `${V('F')}<sub>d</sub> = <i>m</i>(${V('a')}<sub>d</sub> + <i>g</i>${V('ẑ')})`],
    doc: 'PID on the frame hub\'s position. On the learned model the controller doesn\'t know its mass, so m is 1 and the result is a desired specific force. When you fly with the keys or pads, the target moves at a commanded velocity and v arrives as the velocity error, so the damping term also feeds that velocity forward. The integral is kept by the simulator and clamped to ±2 m·s. m is the mass the controller believes in.',
    args: [['ep', 'position error, world [m]'], ['v', 'hub velocity − commanded velocity, world [m/s]'], ['ip', '∫ ep dt [m·s]'], ['m', 'modeled mass [kg]'], ['g', '9.81 m/s²']], returns: 'desired total force, world [N]',
    shape: 3, sample: () => [[0.1, 0, 0.1], [0, 0, 0], [0, 0, 0], 1, 9.81] },
  { key: 'thrustAxisTarget', group: 'ctrl', fn: thrustAxisTarget, title: 'Thrust-axis target',
    math: [`tilt body: ${V('n')}<sub>d</sub> = ${V('F')}<sub>d</sub> / ‖${V('F')}<sub>d</sub>‖, &nbsp;at most 35° from vertical`, `stay level: ${V('n')}<sub>d</sub> = ${V('ẑ')}`],
    doc: 'Where the craft\'s nominal thrust axis should point. The desired attitude is built from this and the target heading.',
    args: [['Fd', 'desired force, world [N]'], ['mode', '"tilt" or "level"']], returns: 'desired thrust axis, world (normalized afterwards)',
    shape: 3, sample: () => [[1, 0, 9.81], 'tilt'] },
  { key: 'attitudeError', group: 'ctrl', fn: attitudeError, title: 'Attitude error',
    math: [`${V('e')}<sub>R</sub> = ½ (<i>R</i><sub>d</sub><sup>T</sup><i>R</i> − <i>R</i><sup>T</sup><i>R</i><sub>d</sub>)<sup>∨</sup>`],
    doc: 'Geometric attitude error on SO(3). The simulator integrates it for the attitude integral, clamped to ±0.5 rad·s.',
    args: [['R', 'current attitude matrix'], ['Rd', 'desired attitude matrix']], returns: 'attitude error, body [rad]',
    shape: 3, sample: () => [qmat([1, 0, 0, 0]), qmat(qnorm([1, 0.05, 0, 0]))] },
  { key: 'attitudeControl', group: 'ctrl', fn: attitudeControl, title: 'Attitude control',
    math: [`${V('α')} = −<i>K</i><sub>R</sub>${V('e')}<sub>R</sub> − <i>K</i><sub>ω</sub>${V('ω')} − <i>K</i><sub>I</sub>∫${V('e')}<sub>R</sub> d<i>t</i>`, `${V('τ')}<sub>d</sub> = <i>J</i>${V('α')} + ${V('ω')} × <i>J</i>${V('ω')}`],
    doc: 'Gains are in angular-acceleration units and multiplied by the modeled inertia, so they carry over to new geometry. On the learned model J is the identity and the result is a desired angular acceleration.',
    args: [['eR', 'attitude error [rad]'], ['w', 'angular velocity, body [rad/s]'], ['ia', '∫ eR dt'], ['J', 'modeled inertia']], returns: 'desired torque, body [N·m]',
    shape: 3, sample: () => [[0.01, 0, 0], [0, 0, 0], [0, 0, 0], [.01, 0, 0, 0, .01, 0, 0, 0, .02]] },
  { key: 'forceDemand', group: 'ctrl', fn: forceDemand, title: 'Body force demand',
    math: [`tilt body: ${V('f')} = (${V('F')}<sub>b</sub> · ${V('n')}) ${V('n')}`, `stay level: ${V('f')} = ${V('F')}<sub>b</sub>`],
    doc: 'Which part of the desired force the actuators are asked to make directly, in the body frame.',
    args: [['Fb', 'desired force, body [N]'], ['n', 'nominal thrust axis, body'], ['mode', '"tilt" or "level"']], returns: 'force demand, body [N]',
    shape: 3, sample: () => [[0.5, 0, 9.81], [0, 0, 1], 'tilt'] },
  { key: 'allocation', group: 'ctrl', fn: allocation, title: 'Control allocation',
    math: [`${V('u')}* = argmin ‖<i>W</i><sup>½</sup>(<i>B</i>${V('u')} − ${V('w')}<sub>d</sub>)‖² &nbsp;subject to &nbsp;${V('u')}<sub>min</sub> ≤ ${V('u')} ≤ ${V('u')}<sub>max</sub>`],
    doc: 'Inputs are throttle fractions from 0 to 1, so B is in acceleration per full throttle; it comes either from the airframe description or from identification. Called twice per control step. Stage 1 includes servo-driven rotors as two virtual inputs each, (T cos θ, T sin θ), and the servo angle is taken from their ratio. Stage 2 solves every motor\'s thrust at the servos\' actual angles.',
    args: [['cols', 'columns of B, one 6-vector per input'], ['lo', 'lower limits'], ['hi', 'upper limits'], ['wd', 'wanted [ax, ay, az, αx, αy, αz]'], ['mode', '"tilt" or "level"']], returns: 'one value per input',
    shape: 'alloc', sample: () => [[[0, 0, 1, 1, 1, 0.1], [0, 0, 1, -1, 1, -0.1], [0, 0, 1, -1, -1, 0.1], [0, 0, 1, 1, -1, -0.1]], [0, 0, 0, 0], [6, 6, 6, 6], [0, 0, 9.81, 0, 0, 0], 'tilt'] },
];

// Wrench sum and control chain shown at the top of the Formulas tab.
const LAW_OVERVIEW = [
  `<i>m</i>${V('a')} = Σ<sub>i</sub> <i>R</i>${V('F')}<sub>i</sub> + ${V('F')}<sub>g</sub> + ${V('F')}<sub>d</sub> + Σ<sub>j</sub> <i>T</i><sub>c,j</sub>${V('n')}<sub>j</sub> + Σ ${V('F')}<sub>ground</sub>`,
  `<i>J</i>${V('ω̇')} + ${V('ω')} × <i>J</i>${V('ω')} = Σ<sub>i</sub> ${V('τ')}<sub>i</sub> + ${V('τ')}<sub>d</sub> + Σ<sub>j</sub> ${V('r')}<sub>j</sub> × <i>R</i><sup>T</sup><i>T</i><sub>c,j</sub>${V('n')}<sub>j</sub> + …`,
];
const LAW_CHAIN = {
  ctrl: ['attitudeEstimator', 'positionEstimator', 'identifyEffectiveness', 'positionControl', 'thrustAxisTarget', 'attitudeError', 'attitudeControl', 'forceDemand', 'allocation'],
  plant: ['batteryModel', 'servoResponse', 'motorResponse', 'tiltAxis', 'wakeVelocity', 'rotorAero', 'rotorWrench', 'wakeLoad', 'gravity', 'bodyDrag', 'cableTension', 'groundContact', 'rigidBody', 'imuModel', 'magModel', 'baroModel', 'posFixModel'],
};
