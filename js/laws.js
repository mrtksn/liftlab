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

function jointRotation(axis, theta) {
  // Servo joint: rotation by theta about its hinge axis (Rodrigues). Everything attached to the joint turns
  // with it about the joint's pivot: rotors, masses, sensors, further joints.
  const [x, y, z] = unit(axis), c = Math.cos(theta), s = Math.sin(theta), C = 1 - c;
  return [c + x * x * C, x * y * C - z * s, x * z * C + y * s,
          y * x * C + z * s, c + y * y * C, y * z * C - x * s,
          z * x * C - y * s, z * y * C + x * s, c + z * z * C];
}

function motorResponse(T, Tcmd, tmax, tau, dt) {
  const target = clamp(Tcmd, 0, tmax);                   // props can't push backwards or exceed max
  return T + (target - T) * Math.min(1, dt / tau);       // first-order spin-up lag
}

function throttleCurve(u, bend) {
  // Throttle (0–1) to thrust as a fraction of max. Props make thrust roughly with RPM², so real motors
  // bend upward: bend 0 is a straight line, 1 is thrust ∝ throttle². The controller isn't told this.
  const x = clamp(u, 0, 1);
  return (1 - bend) * x + bend * x * x;
}

function servoLinkage(theta, horn, lag, dt) {
  // The rotor follows the servo horn through the servo's own control loop and linkage: a first-order lag.
  return lag > 0 ? theta + (horn - theta) * Math.min(1, dt / lag) : horn;
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

function flowModel(f, q, p, st, dt) {
  // f: true angular flow of the ground under the camera, sensor frame [rad/s]
  // q: image quality from ground texture, light and height, 0–1. Returns [fx, fy, quality].
  if (!st.s) st.s = [1 + randn() * p.scale, 1 + randn() * p.scale];   // lens and calibration scale error
  if (q < 0.15) return [0, 0, 0];                                     // too dark or featureless: no reading
  if (Math.hypot(f[0], f[1]) > p.maxRate) return [0, 0, 0];           // ground moving too fast across the image
  const sd = p.noise / q;                                             // poorer images, noisier flow
  return [f[0] * st.s[0] + randn() * sd, f[1] * st.s[1] + randn() * sd, q];
}

function rangeModel(d, p, st) {
  // d: true distance to the ground along the sensor's boresight [m]. Returns −1 when out of range.
  if (!(d >= p.minRange && d <= p.maxRange)) return -1;
  return d + randn() * p.noise * (1 + d);                             // noise grows with distance
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

function flowVelocity(flow, range, w, Rs) {
  // Turns an optical-flow reading into velocity. flow: [fx, fy] [rad/s]; range: distance to the ground [m];
  // w: gyro in the sensor frame [rad/s]; Rs: sensor → world rotation (attitude estimate × known mount).
  // The camera looks along its −Z. Rotation also sweeps the image, so the gyro's share is removed first.
  const vs = [range * (w[1] - flow[0]), range * (-w[0] - flow[1]), 0];   // sensor velocity across the image plane
  const v = m3v(Rs, vs);                                                  // → world
  const down = m3v(Rs, [0, 0, -1]);
  return [v[0], v[1], range * -down[2]];                                  // [vx, vy, height above ground]
}

function servoPredictor(st, cmd, rate, lag, dt) {
  // Where a servo without angle feedback is, predicted from what was commanded: the horn moves at most
  // rate·dt per step, the rotor follows with a first-order lag. rate and lag come from the actuator tests.
  if (st.h == null) { st.h = cmd; st.th = cmd; }
  st.h += clamp(cmd - st.h, -rate * dt, rate * dt);
  st.th = lag > 0 ? st.th + (st.h - st.th) * Math.min(1, dt / lag) : st.h;
  return st.th;
}

function positionEstimator(st, R, accel, baro, fix, flow, m, dt) {
  // Complementary filter. Integrates the accelerometer (rotated by the attitude estimate) and pulls the
  // result toward whatever references exist: position fix, optical flow, rangefinder, barometer.
  // Each is null when not available; its `age` says how long ago it was measured, so it is compared
  // with the estimate from that moment. The simulator seeds st.p and st.v with the start point at reset.
  const kP = 0.8, kV = 0.3, kFixV = 0.6, kBaro = 1.5, kBaroV = 0.6;
  const kFlow = 2.0, kRange = 2.5, kRangeV = 1.2;        // optical flow velocity, rangefinder height
  const kBias = 0.4;                                     // learns the accelerometer's horizontal bias from velocity errors
  const cd = 0.25, kDrag = 1.0;                          // airframe drag coefficient [N per m/s] (see bodyDrag), drag-fusion gain
  if (!st.p) { st.p = fix ? fix.p.slice() : [0, 0, baro ? baro.alt : 0]; st.v = [0, 0, 0]; }
  if (!st.h) { st.h = []; st.af = accel.slice(); st.ab = [0, 0]; }   // recent estimates (newest last), filtered accel, accel bias
  const a = add(m3v(R, accel), [0, 0, -G]);              // world acceleration from the IMU
  a[0] -= st.ab[0]; a[1] -= st.ab[1];
  st.v = add(st.v, scl(a, dt)); st.p = add(st.p, scl(st.v, dt));
  const past = age => st.h[Math.max(0, st.h.length - 1 - Math.round(age / dt))] || { p: st.p, v: st.v };
  const range = flow && flow.h != null;                  // rangefinder height available
  if (fix) {
    const then = past(fix.age);
    const e = sub(fix.p, then.p); if (baro || range) e[2] = 0;   // barometer or rangefinder own altitude
    st.p = add(st.p, scl(e, kP * dt)); st.v = add(st.v, scl(e, kV * dt));
    st.v = add(st.v, scl(sub(fix.v, then.v), kFixV * dt));
    for (let i = 0; i < 2; i++) st.ab[i] -= kBias * kFixV * (fix.v[i] - then.v[i]) * dt;
  }
  if (flow && flow.v) {                                  // optical flow: horizontal velocity over the ground
    const then = past(flow.age);
    for (let i = 0; i < 2; i++) {
      const e = flow.v[i] - then.v[i];
      st.v[i] += kFlow * e * dt; st.ab[i] -= kBias * e * dt;   // a persistent velocity error means accelerometer bias
    }
  }
  st.ab = st.ab.map(b => clamp(b, -0.5, 0.5));
  if (!fix && !(flow && flow.v)) {
    // No fix or flow. Thrust only pushes along body Z, so the sideways accelerometer reading is air drag,
    // which reveals the body's airspeed: v_xy ≈ −(m / cd)·f_xy. Pull the estimate toward it.
    const k = dt / (dt + 1 / (2 * Math.PI * 2));
    st.af = st.af.map((x, i) => x + k * (accel[i] - x));
    const vb = m3v(m3T(R), st.v);
    const dv = [-(m / cd) * st.af[0] - vb[0], -(m / cd) * st.af[1] - vb[1], 0];
    st.v = add(st.v, scl(m3v(R, dv), kDrag * dt));
  }
  if (range) { const e = flow.h - past(flow.age).p[2]; st.p[2] += kRange * e * dt; st.v[2] += kRangeV * e * dt; }  // flat ground assumed
  if (baro) {
    const w = range ? 0.15 : 1;                          // near the ground the rangefinder is far better
    const e = baro.alt - past(baro.age).p[2]; st.p[2] += w * kBaro * e * dt; st.v[2] += w * kBaroV * e * dt;
  }
  st.h.push({ p: st.p.slice(), v: st.v.slice() }); if (st.h.length > 800) st.h.shift();
  return { p: st.p.slice(), v: st.v.slice() };
}

// ═════════════ Identification (learning what the actuators do) ═════════════

function identifyEffectiveness(st, u, f, w, r, dt, init, memory, lags) {
  // Recursive least squares. Learns B, what each actuator input does to the drone, from flight data:
  //   Δ[f; dω/dt] ≈ B · Δu
  // Both sides are band-passed (0.3–12 Hz), so it learns from changes and steady offsets such as drag
  // or trim can't leak into B.
  // u: inputs: thrust fractions, times (1, cos θ, sin θ) for every joint a motor sits on
  // f: accelerometer (specific force) and w: gyro, body frame; r: IMU position from the hub [m]
  // init: starting guess, 6 rows × inputs; memory: how long past data counts [s];
  // lags: each input's motor lag [s], from the actuator tests (35 ms assumed until then).
  // Returns B: 6 rows (ax ay az αx αy αz) × inputs.
  const lpHz = 12, hpHz = 0.3, pMax = 2;                    // band [Hz], covariance cap
  const n = u.length;
  const k = hz => dt / (dt + 1 / (2 * Math.PI * hz));
  if (dt <= 0) return { B: st.th || init };
  if (!st.P) {                                                   // start the filters from the first real reading
    st.th = init.map(row => row.slice());
    st.P = eye(n).map(row => row.map(x => x * 2));
    st.ua = u.slice(); st.xl = u.slice(); st.xs = u.slice(); st.yl = f.concat([0, 0, 0]); st.ys = st.yl.slice();
    st.wl = w.slice(); st.n = 0;
  }
  for (let j = 0; j < n; j++) st.ua[j] += (u[j] - st.ua[j]) * Math.min(1, dt / ((lags && lags[j]) || 0.035));   // what the motors are doing now
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

function identifyMotorResponse(wins, dt) {
  // One motor tested on its own while everything else holds still. Each window starts steady, then the
  // throttle steps up and down. y is the drone's response along this motor's known effect, in units of
  // "full thrust" (it's 1 when the motor goes from 0 to 100%). Model:
  //   y = g · lag_τ( (1−k)u + k u² ) + d·I + e·(lag_τ(u) − u₀)·I + c_w,   I = ∫(lag_τ(u) − u₀) dt
  //     = g·lag_τ(u) + g·k·lag_τ(u² − u) + d·I + e·(…)·I + c_w
  // Linear in g, g·k, d and the per-window offsets c_w, so each motor lag τ on the grid is one least-squares
  // fit; the best τ wins. A step up giving more than the same step down reveals the bend k. The I terms soak
  // up what the pulse's own motion does: the drone rising or rolling pushes air through the prop, and that
  // costs more thrust the harder the prop is pushing. Both would otherwise look like a bend.
  const taus = [0.01, 0.015, 0.02, 0.025, 0.03, 0.04, 0.05, 0.065, 0.08, 0.1], lpHz = 25;
  const kf = dt / (dt + 1 / (2 * Math.PI * lpHz)), nw = wins.length, np = 4 + nw;
  const dotn = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
  let sst = 0;
  for (const w of wins) { const m = w.y.reduce((a, v) => a + v, 0) / Math.max(1, w.y.length); for (const v of w.y) sst += (v - m) ** 2; }
  let best = null;
  for (const tau of taus) {
    const A = Array.from({ length: np }, () => new Array(np).fill(0)), b = new Array(np).fill(0); let yy = 0;
    wins.forEach((w, wi) => {
      const u0 = w.u[0]; let m1 = u0, m2 = u0 * u0 - u0, l1 = m1, l2 = m2, I = 0;
      for (let t = 0; t < w.u.length; t++) {
        const u = w.u[t], a = Math.min(1, dt / tau);
        m1 += (u - m1) * a; m2 += (u * u - u - m2) * a; l1 += kf * (m1 - l1); l2 += kf * (m2 - l2); I += (l1 - u0) * dt;
        const phi = new Array(np).fill(0); phi[0] = l1; phi[1] = l2; phi[2] = I; phi[3] = (l1 - u0) * I; phi[4 + wi] = 1;
        for (let p = 0; p < np; p++) { b[p] += phi[p] * w.y[t]; for (let q = 0; q < np; q++) A[p][q] += phi[p] * phi[q]; }
        yy += w.y[t] ** 2;
      }
    });
    const th = solveLin(A.map((r, i) => r.map((v, j) => v + (i === j ? 1e-9 : 0))), b);
    const sse = yy - 2 * dotn(th, b) + dotn(th, A.map(r => dotn(r, th)));
    if (!best || sse < best.sse) best = { tau, th, sse };
  }
  if (!best || Math.abs(best.th[0]) < 1e-6) return { tau: 0, curve: 0, gain: 0, fit: 0 };
  return { tau: best.tau, curve: clamp(best.th[1] / best.th[0], -0.5, 1.5), gain: best.th[0], fit: clamp(1 - best.sse / Math.max(1e-12, sst), 0, 1) };
}

function identifyServoResponse(wins, dt) {
  // One servo stepped on its own while everything else holds still. cmd: commanded angle change from the
  // start of the window [rad]; y: the drone's response along this servo's effect, which is sin(angle change)
  // for a rigid tilting rotor. Model: the horn moves at most `rate`, the rotor follows with lag λ,
  //   y = g · sin(θ) + c_w.
  // Rate and lag are found on a grid (each pair is a two-number least-squares fit); the best pair wins.
  const rates = [60, 80, 110, 150, 200, 260, 340, 450, 600, 800].map(d => d * Math.PI / 180);
  const lags = [0, 0.005, 0.01, 0.015, 0.02, 0.03, 0.045, 0.065, 0.09], lpHz = 25;
  const kf = dt / (dt + 1 / (2 * Math.PI * lpHz)), nw = wins.length, np = 1 + nw;
  const dotn = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
  let sst = 0;
  for (const w of wins) { const m = w.y.reduce((a, v) => a + v, 0) / Math.max(1, w.y.length); for (const v of w.y) sst += (v - m) ** 2; }
  let best = null;
  for (const rate of rates) for (const lag of lags) {
    const A = Array.from({ length: np }, () => new Array(np).fill(0)), b = new Array(np).fill(0); let yy = 0;
    wins.forEach((w, wi) => {
      let h = 0, th = 0, l = 0;
      for (let t = 0; t < w.cmd.length; t++) {
        h += clamp(w.cmd[t] - h, -rate * dt, rate * dt);
        th = lag > 0 ? th + (h - th) * Math.min(1, dt / lag) : h;
        l += kf * (Math.sin(th) - l);
        const phi = new Array(np).fill(0); phi[0] = l; phi[1 + wi] = 1;
        for (let p = 0; p < np; p++) { b[p] += phi[p] * w.y[t]; for (let q = 0; q < np; q++) A[p][q] += phi[p] * phi[q]; }
        yy += w.y[t] ** 2;
      }
    });
    const th = solveLin(A.map((r, i) => r.map((v, j) => v + (i === j ? 1e-9 : 0))), b);
    const sse = yy - 2 * dotn(th, b) + dotn(th, A.map(r => dotn(r, th)));
    if (!best || sse < best.sse) best = { rate, lag, th, sse };
  }
  if (!best) return { rate: 0, lag: 0, gain: 0, fit: 0 };
  return { rate: best.rate, lag: best.lag, gain: best.th[0], fit: clamp(1 - best.sse / Math.max(1e-12, sst), 0, 1) };
}

function identifyThrow(st, u, f, w, vb, dt, solve) {
  // Batch least squares over a free fall, for a drone that knows nothing about itself (after Blaha,
  // Smeur & Remes, TU Delft 2024). The motors start from zero and the drone is falling, so the
  // accelerometer feels only the rotors plus its own swing around the center of gravity (CoG):
  //   f = B_f · u_τ + ([α]× + [ω]×²) r − d v_b + c_f    r: IMU offset from the CoG, d: drag; both shared by the rows
  //   α = B_α · u_τ + K (ω_y ω_z, ω_z ω_x, ω_x ω_y) + c_α     K: gyroscopic coupling (inertia ratios)
  // u_τ is the command through a first-order motor lag τ. Several τ are fitted side by side; the best
  // fit wins, which also identifies the motor lag. Call every step while falling; solve = true to fit.
  // Returns B (6 rows × inputs), r [m], tau [s] and how much of the force and rotation it explains.
  const taus = [0.01, 0.02, 0.03, 0.045, 0.065, 0.09], lpHz = 25, skip = 0.03;
  const n = u.length, nf = 3 * n + 7, nr = n + 4;
  const dotn = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
  const k = dt > 0 ? dt / (dt + 1 / (2 * Math.PI * lpHz)) : 0;
  const zeros = (a, b) => b ? Array.from({ length: a }, () => new Array(b).fill(0)) : new Array(a).fill(0);
  if (!st.fits) {
    st.fits = taus.map(tau => ({ tau, um: zeros(n), ul: zeros(n),
      Af: zeros(nf, nf), bf: zeros(nf), Ar: zeros(nr, nr), br: zeros(3, nr) }));
    st.wl = w.slice(); st.fl = f.slice(); st.yy = zeros(6); st.ys = zeros(6); st.N = 0; st.t = 0;
  }
  if (dt > 0) {
    const wPrev = st.wl;
    st.wl = st.wl.map((v, i) => v + k * (w[i] - v));
    st.fl = st.fl.map((v, i) => v + k * (f[i] - v));
    const a = st.wl.map((v, i) => (v - wPrev[i]) / dt), W = st.wl;       // angular acceleration
    const L = [0, 1, 2].map(i => [0, 1, 2].map(j =>                         // [α]× + [ω]×²
      [[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]][i][j] + W[i] * W[j] - (i === j ? dot(W, W) : 0)));
    const gyro = [W[1] * W[2], W[2] * W[0], W[0] * W[1]];
    st.t += dt;
    for (const F of st.fits) {
      for (let j = 0; j < n; j++) { F.um[j] += (u[j] - F.um[j]) * Math.min(1, dt / F.tau); F.ul[j] += k * (F.um[j] - F.ul[j]); }
      if (st.t < skip) continue;                                             // let the filters start up
      for (let i = 0; i < 3; i++) {                                          // force rows share r
        const phi = zeros(nf);
        for (let j = 0; j < n; j++) phi[i * n + j] = F.ul[j];
        for (let j = 0; j < 3; j++) phi[3 * n + j] = L[i][j];
        phi[3 * n + 3 + i] = 1;
        phi[3 * n + 6] = -vb[i];
        for (let p = 0; p < nf; p++) { if (!phi[p]) continue; F.bf[p] += phi[p] * st.fl[i]; for (let q = 0; q < nf; q++) F.Af[p][q] += phi[p] * phi[q]; }
      }
      const phi = [...F.ul, ...gyro, 1];                                     // rotation rows share regressors
      for (let p = 0; p < nr; p++) { for (let q = 0; q < nr; q++) F.Ar[p][q] += phi[p] * phi[q]; for (let i = 0; i < 3; i++) F.br[i][p] += phi[p] * a[i]; }
    }
    if (st.t >= skip) { const y = [...st.fl, ...a]; st.N++; for (let i = 0; i < 6; i++) { st.yy[i] += y[i] * y[i]; st.ys[i] += y[i]; } }
  }
  if (!solve || st.N < 20) return st.out || { B: zeros(6, n), r: [0, 0, 0], tau: 0, fitF: 0, fitR: 0 };
  const ridge = A => A.map((row, i) => row.map((v, j) => v + (i === j ? 1e-9 + 1e-6 * A[i][i] : 0)));
  const sse = (A, b, th, yy) => yy - 2 * dotn(th, b) + dotn(th, A.map(row => dotn(row, th)));
  const sst = i => Math.max(1e-9, st.yy[i] - st.ys[i] ** 2 / st.N);
  let best = null;
  for (const F of st.fits) {
    const thF = solveLin(ridge(F.Af), F.bf), thR = [0, 1, 2].map(i => solveLin(ridge(F.Ar), F.br[i]));
    const fitF = clamp(1 - sse(F.Af, F.bf, thF, st.yy[0] + st.yy[1] + st.yy[2]) / (sst(0) + sst(1) + sst(2)), 0, 1);
    const fitR = clamp(1 - thR.reduce((s, th, i) => s + sse(F.Ar, F.br[i], th, st.yy[3 + i]), 0) / (sst(3) + sst(4) + sst(5)), 0, 1);
    if (!best || fitR + 0.5 * fitF > best.score) best = { F, thF, thR, fitF, fitR, score: fitR + 0.5 * fitF };
  }
  const B = [0, 1, 2].map(i => best.thF.slice(i * n, i * n + n)).concat(best.thR.map(th => th.slice(0, n)));
  st.out = { B, r: best.thF.slice(3 * n, 3 * n + 3), drag: best.thF[3 * n + 6], tau: best.F.tau, fitF: best.fitF, fitR: best.fitR };
  return st.out;
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

function thrustAxisTarget(Fd, mode, share) {
  // share (mixed mode): part of the sideways force the servos make, so the body only leans for the rest
  if (mode === 'level') return [0, 0, 1];                // keep the thrust axis vertical
  const maxTilt = 35 * Math.PI / 180;
  const s = mode === 'mixed' ? clamp(share || 0, 0, 1) : 0;
  let n = unit([Fd[0] * (1 - s), Fd[1] * (1 - s), Fd[2]]);   // point the thrust axis along the part the body makes
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
  if (mode === 'level' || mode === 'mixed') return Fb;   // thrust vectoring handles (its share of) the sideways force
  return scl(n, dot(Fb, n));                             // only the thrust axis can push
}

function thrustLinearization(v, bend) {
  // Inverse of the learned throttle curve: the throttle that gives thrust fraction v, if thrust = (1−b)u + b·u².
  // Keeps everything above it linear in thrust. bend is 0 until the actuator tests have measured it.
  if (Math.abs(bend) < 1e-4) return clamp(v, 0, 1);
  const a = 1 - bend, disc = Math.max(0, a * a + 4 * bend * clamp(v, 0, 1));
  return clamp((-a + Math.sqrt(disc)) / (2 * bend), 0, 1);
}

function allocation(cols, lo, hi, wd, mode, pull) {
  // cols[j]: what one unit of input j does, as [ax, ay, az, αx, αy, αz] (acceleration units)
  // wd: the 6 accelerations wanted; lo/hi: input limits
  // pull: { q, r } from allocationPreferences — how strongly each input is drawn toward a preferred value
  const W = mode === 'tilt' ? [0.3, 0.3, 3, 10, 10, 1] : [3, 3, 3, 10, 10, 1];   // sideways force only counts when servos are asked for it
  const x = bls(cols, lo, hi, wd, W);                    // 1. the best move the limits allow (bounded weighted least squares)
  if (!pull) return x;
  // 2. Of all the ways to make that same move, the preferred one. The pulls are tiny next to the move, so
  //    they only decide where there is a real choice: a hexacopter's spare motors, a servo vs. a motor.
  const got = [0, 1, 2, 3, 4, 5].map(k => cols.reduce((s, c, j) => s + c[k] * x[j], 0));
  return bls(cols, lo, hi, got, W, { q: pull.q, r: pull.r, rel: 1e-5 });
}

function allocationPreferences(inputs, prefs) {
  // What to prefer when a move can be made more than one way. Each preference becomes a quadratic pull
  // q·((x − r)/span)² on one input; the allocation only lets the pulls choose between equally good moves,
  // so only the ratios between them matter.
  // inputs[j]: { kind: 'thrust' | 'servo', x: value now, lo, hi,
  //              authority: 0–1, this input's share of the attitude control (how much it can turn the drone),
  //              power: W at full thrust (thrust inputs),
  //              th, range: servo angle now and its limit [rad], reach: how far it can get in time (servo inputs) }
  // prefs: { allowance, efficiency, servoMove } weights; 0 turns one off.
  const q = [], r = [];
  const Ptot = inputs.reduce((s, i) => s + (i.kind === 'thrust' ? i.power : 0), 0) || 1;
  for (const i of inputs) {
    let qs = 0, qr = 0;
    const add = (w, target) => { qs += w; qr += w * target; };
    if (i.kind === 'thrust') {
      // Allowance: room to move both ways. The pull toward the middle grows steeply near a limit
      // (1× in the middle, about 100× at the limit), and counts most for the inputs that steer: a big
      // lifting rotor near its limit costs less control than a steering motor near its limit.
      const m = clamp(Math.min(i.x - i.lo, i.hi - i.x) / (i.hi - i.lo), 0, 0.5);
      add(prefs.allowance * (0.2 + 0.8 * (i.authority ?? 1)) * (0.55 / (m + 0.05)) ** 2, (i.lo + i.hi) / 2);
      // Efficiency: rotor power grows as thrust^1.5 (momentum theory). Its quadratic model around the
      // thrust now is a pull toward less thrust, weighted by this rotor's share of the full power.
      const u = Math.max(0.05, i.x), e = prefs.efficiency * i.power / Ptot;
      add(0.375 * e / Math.sqrt(u), -u);
    } else {
      // Servo input is the angle change δ this step. Allowance pulls the angle toward the middle of its
      // range; moving costs in proportion to how much of its reach a move uses, so quick corrections go
      // to the motors and a slow or lagging servo takes the steady part.
      const m = clamp((i.range - Math.abs(i.th)) / (2 * i.range), 0, 0.5);
      add(prefs.allowance * (0.2 + 0.8 * (i.authority ?? 1)) * (0.55 / (m + 0.05)) ** 2 * ((i.hi - i.lo) / (2 * i.range)) ** 2, clamp(-i.th, i.lo, i.hi));
      add(prefs.servoMove * ((i.hi - i.lo) / (2 * Math.max(i.reach, 1e-3))) ** 2, 0);
    }
    q.push(qs); r.push(qs > 0 ? qr / qs : 0);
  }
  return { q, r };
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
  { key: 'jointRotation', group: 'plant', fn: jointRotation, title: 'Servo joint',
    math: [`<i>R</i>(θ) = cos θ <i>I</i> + sin θ [${V('a')}]<sub>×</sub> + (1 − cos θ) ${V('a')}${V('a')}<sup>T</sup>`, `${V('p')}<sub>now</sub> = ${V('q')} + <i>R</i>(θ)(${V('p')}<sub>rest</sub> − ${V('q')}), &nbsp;applied for each joint above a part, nearest first`],
    doc: 'A servo joint turns everything attached to it about its hinge axis a through its pivot q: motors, masses, cable attachments, sensors and further joints. Parts store where they are at rest (all angles zero). Positive θ turns by the right-hand rule about a. Because each rotor\'s effect is linear in cos θ and sin θ for every joint above it, the controller can learn it as a few fixed columns (see Effectiveness identification).',
    args: [['axis', 'hinge axis at rest, body frame'], ['theta', 'joint angle [rad]']], returns: '3×3 rotation, row by row',
    shape: 'mat3', sample: () => [[1, 0, 0], 0.3] },
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
  { key: 'servoLinkage', group: 'plant', fn: servoLinkage, title: 'Servo linkage lag',
    math: [`θ<sub>k+1</sub> = θ<sub>k</sub> + (θ<sub>horn</sub> − θ<sub>k</sub>) · Δ<i>t</i> / λ`],
    doc: 'The rotor follows the servo horn with a small lag from the servo\'s own control loop and the linkage. The servo\'s trim error (its zero being off by a few degrees) is added to the command before the horn. Neither the lag nor the trim error is told to the controller.',
    args: [['theta', 'rotor angle [rad]'], ['horn', 'servo horn angle [rad]'], ['lag', 'time constant [s]'], ['dt', 'time step [s]']], returns: 'next rotor angle [rad]',
    shape: 'n', sample: () => [0, 0.3, 0.02, 0.0005] },
  { key: 'throttleCurve', group: 'plant', fn: throttleCurve, title: 'Throttle curve',
    math: [`<i>T</i>/<i>T</i><sub>max</sub> = (1 − <i>b</i>) <i>u</i> + <i>b u</i>²`],
    doc: 'How throttle turns into thrust. Thrust grows roughly with RPM², so real motors bend upward: the same extra throttle gives more thrust near full power than near idle. The controller isn\'t told the bend; the actuator tests measure it.',
    args: [['u', 'throttle 0–1'], ['bend', '0 straight, 1 thrust ∝ throttle²']], returns: 'thrust as a fraction of max',
    shape: 'n', sample: () => [0.5, 0.3] },
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

  { key: 'flowModel', group: 'sensor', fn: flowModel, title: 'Optical flow camera',
    math: [`${V('f')} = (ω<sub>y</sub> − <i>v</i><sub>x</sub>/<i>d</i>, −ω<sub>x</sub> − <i>v</i><sub>y</sub>/<i>d</i>) &nbsp;(sensor frame, camera looking along −Z)`, `${V('f̃')} = <i>s</i>·${V('f')} + ${V('n')}/<i>q</i>, &nbsp;no reading when <i>q</i> < 0.15 or |${V('f')}| > <i>f</i><sub>max</sub>`],
    doc: 'How fast the ground slides across the image, as an angular rate. Both moving and rotating sweep the image. Image quality q comes from the ground texture, the light and the height; over calm water, snow or in the dark there\'s nothing to track.',
    args: [['f', 'true flow, sensor frame [rad/s]'], ['q', 'image quality 0–1'], ['p', 'noise, scale, maxRate'], ['st', 'this sensor\'s state'], ['dt', 'sample period [s]']],
    returns: '[fx, fy, quality]', shape: 3, sample: () => [[0.1, -0.2], 0.8, { noise: 0.05, scale: 0.03, maxRate: 7 }, {}, 0.01] },
  { key: 'rangeModel', group: 'sensor', fn: rangeModel, title: 'Rangefinder',
    math: [`<i>d̃</i> = <i>d</i> + <i>n</i>(1 + <i>d</i>) &nbsp;for <i>d</i><sub>min</sub> ≤ <i>d</i> ≤ <i>d</i><sub>max</sub>, otherwise no reading`],
    doc: 'Laser distance along the camera\'s boresight. Tilting the drone lengthens the beam; above its maximum range it reads nothing.',
    args: [['d', 'true distance to the ground [m]'], ['p', 'noise, minRange, maxRange'], ['st', 'this sensor\'s state']],
    returns: 'distance [m], or −1', shape: 'n', sample: () => [1.5, { noise: 0.01, minRange: 0.05, maxRange: 4 }, {}] },

  { key: 'attitudeEstimator', group: 'est', fn: attitudeEstimator, title: 'Attitude estimator',
    math: [`${V('e')} = ${V('ã')} × ${V('û')} + <i>R̂</i><sup>T</sup>(0, 0, −ψ<sub>err</sub>)`, `${V('ω')}<sub>c</sub> = ${V('ω̃')} + <i>K</i><sub>I</sub>∫${V('e')} d<i>t</i> + <i>K</i><sub>P</sub>${V('e')}, &nbsp; <i>q̂̇</i> = ½ <i>q̂</i> ⊗ ${V('ω')}<sub>c</sub>`],
    doc: 'Mahony complementary filter. A multirotor\'s accelerometer feels thrust rather than gravity whenever it accelerates, so the filter only trusts it for "up" when the low-passed reading is within 5% of 1 g. Without a compass, heading drifts with the gyro bias.',
    args: [['st', 'estimator state'], ['gyro', 'fused gyro reading, body [rad/s]'], ['accel', 'fused accelerometer reading, body [m/s²]'], ['mag', 'fused compass reading, body, or null'], ['dt', 'control period [s]']],
    returns: '{ q: attitude quaternion [w, x, y, z]; w: filtered rate, body }', shape: { q: 4, w: 3 },
    sample: () => [{}, [0, 0, 0], [0, 0, 9.81], [0.5, 0, -0.866], 0.001] },
  { key: 'flowVelocity', group: 'est', fn: flowVelocity, title: 'Flow to velocity',
    math: [`${V('v')}<sub>s</sub> = <i>d̃</i> (ω̃<sub>y</sub> − <i>f̃</i><sub>x</sub>, −ω̃<sub>x</sub> − <i>f̃</i><sub>y</sub>, 0), &nbsp;${V('v')} = <i>R̂</i><sub>s</sub>${V('v')}<sub>s</sub>, &nbsp;<i>h</i> = <i>d̃</i> · (−<i>R̂</i><sub>s</sub>ẑ)<sub>z</sub>`],
    doc: 'Removes the part of the image motion the gyro explains, scales the rest by the measured distance, and rotates it into the world. The result goes to the position estimator after being shifted to the hub.',
    args: [['flow', '[fx, fy] reading [rad/s]'], ['range', 'distance to the ground [m]'], ['w', 'gyro, sensor frame [rad/s]'], ['Rs', 'sensor → world rotation']],
    returns: '[vx, vy, height above ground]', shape: 3, sample: () => [[0.05, 0], 1.5, [0, 0, 0], [1, 0, 0, 0, 1, 0, 0, 0, 1]] },

  { key: 'servoPredictor', group: 'est', fn: servoPredictor, title: 'Servo angle predictor',
    math: [`<i>ĥ</i><sub>k+1</sub> = <i>ĥ</i><sub>k</sub> + sat<sub>±<i>ω̂</i>Δ<i>t</i></sub>(θ<sub>cmd</sub> − <i>ĥ</i><sub>k</sub>), &nbsp;θ̂<sub>k+1</sub> = θ̂<sub>k</sub> + (<i>ĥ</i> − θ̂)Δ<i>t</i>/λ̂`],
    doc: 'Hobby servos don\'t report their angle. Without feedback, the controller uses this prediction wherever it needs the servo\'s angle: in the allocation and in learning. Until the servo tests run, it assumes the rated speed from the description and no lag.',
    args: [['st', 'predictor state'], ['cmd', 'commanded angle [rad]'], ['rate', 'speed [rad/s]'], ['lag', 'lag [s]'], ['dt', 'control period [s]']], returns: 'predicted angle [rad]',
    shape: 'n', sample: () => [{}, 0.3, 5, 0.02, 0.001] },

  { key: 'positionEstimator', group: 'est', fn: positionEstimator, title: 'Position estimator',
    math: [`${V('v̂̇')} = <i>R̂</i>${V('f̃')} + ${V('g')} + <i>k</i><sub>V</sub>(${V('p̃')} − ${V('p̂')}) + <i>k</i><sub>fv</sub>(${V('ṽ')} − ${V('v̂')}) + <i>k</i><sub>flow</sub>(${V('v')}<sub>flow</sub> − ${V('v̂')})<sub>xy</sub>`, `altitude: rangefinder near the ground, barometer otherwise; &nbsp;${V('b̂')}<sub>a</sub> learned from velocity errors`, `${V('p̂̇')} = ${V('v̂')} + <i>k</i><sub>P</sub>(${V('p̃')} − ${V('p̂')}), &nbsp;altitude from the barometer when present`, `no fix: ${V('v̂')}<sub>xy</sub> → −(<i>m</i>/<i>c</i><sub>d</sub>) ${V('f̃')}<sub>xy</sub> &nbsp;(drag fusion)`],
    doc: 'Fuses the accelerometer with the position fix, optical flow, rangefinder and barometer. The simulator first shifts each reading to the frame hub using the sensor positions the controller knows, and reports how old it is so a delayed fix is compared with the estimate from when it was measured. Optical flow gives velocity over the ground, so position still drifts slowly without a fix. With neither, it falls back on drag fusion: a multirotor\'s accelerometer feels air drag sideways, which reveals airspeed, so wind and a wrong drag coefficient make it drift.',
    args: [['st', 'estimator state'], ['R', 'estimated attitude matrix'], ['accel', 'fused accelerometer reading, body'], ['baro', '{ alt, age } hub altitude, or null'], ['fix', '{ p, v, age } hub position and velocity, or null'], ['flow', '{ v: [vx, vy] or null, h: hub height or null, age }, or null'], ['m', 'modeled mass [kg]'], ['dt', 'control period [s]']],
    returns: '{ p: hub position; v: hub velocity }, world', shape: { p: 3, v: 3 },
    sample: () => [{}, [1, 0, 0, 0, 1, 0, 0, 0, 1], [0, 0, 9.81], { alt: 1.5, age: 0.02 }, { p: [0, 0, 1.5], v: [0, 0, 0], age: 0.15 }, { v: [0.1, 0], h: 1.5, age: 0.02 }, 1, 0.001] },

  { key: 'identifyEffectiveness', group: 'learn', fn: identifyEffectiveness, title: 'Effectiveness identification',
    math: [`Δ[${V('f̃')} − ${V('ω̇')}×${V('r')} − ${V('ω')}×(${V('ω')}×${V('r')}); ${V('ω̃̇')}] ≈ <i>B̂</i> Δ${V('u')}, &nbsp;both sides band-passed 0.3–12 Hz`, `<i>K</i> = <i>P</i>${V('x')} / (λ + ${V('x')}<sup>T</sup><i>P</i>${V('x')}), &nbsp;<i>B̂</i> += ${V('e')}<i>K</i><sup>T</sup>, &nbsp;<i>P</i> = (<i>P</i> − <i>K</i>${V('x')}<sup>T</sup><i>P</i>)/λ`],
    doc: 'Recursive least squares with forgetting (about 4 s of memory). A motor on servo joints is several inputs: its thrust times each product of (1, cos θ, sin θ) over the joints it sits on, so its effect at any joint angles is a fixed sum of learned columns (3 for one joint, 9 for two). It learns, straight from the accelerometer and gyro, how much acceleration and angular acceleration each actuator input produces. That covers mass, inertia, prop thrust, rotor wakes and battery sag without being told any of them. It learns from changes, so steady offsets like drag can\'t leak in, and it removes the accelerometer\'s lever-arm swing using the IMU position the controller knows. Runs during calibration and, if you leave learning on, all through the flight.',
    args: [['st', 'identification state'], ['u', 'inputs: thrust fractions, times (1, cos θ, sin θ) for each joint a motor sits on'], ['f', 'accelerometer, body [m/s²]'], ['w', 'gyro, body [rad/s]'], ['r', 'IMU position from the hub [m]'], ['dt', 'control period [s]'], ['init', 'starting guess, 6 rows'], ['memory', 'forgetting time [s]: short while calibrating, long in flight'], ['lags', 'each input\'s motor lag [s], learned by the actuator tests']],
    returns: '{ B: 6 rows × inputs }', shape: { B: 'rows' },
    sample: () => [{}, [0.5, 0.5], [0, 0, 9.81], [0, 0, 0], [0, 0, 0.01], 0.001, [[0, 0], [0, 0], [10, 10], [100, -100], [0, 0], [1, -1]], 4, [0.03, 0.03]] },
  { key: 'identifyMotorResponse', group: 'learn', fn: identifyMotorResponse, title: 'Motor response test',
    math: [`<i>y</i> = <i>g</i> · lag<sub>τ</sub>((1 − <i>k</i>)<i>u</i> + <i>k u</i>²) + <i>d I</i> + <i>e</i>(lag<sub>τ</sub>(<i>u</i>) − <i>u</i><sub>0</sub>)<i>I</i> + <i>c</i><sub>w</sub>, &nbsp;<i>I</i> = ∫(lag<sub>τ</sub>(<i>u</i>) − <i>u</i><sub>0</sub>)d<i>t</i>`, `least squares for each τ on a grid (10–100 ms); the best fit gives the motor lag τ and the throttle-curve bend <i>k</i>`],
    doc: 'Runs on the calibration\'s motor pulses. While one motor steps up and down, every other actuator holds still, so the drone\'s response along that motor\'s effect is that motor alone. How slowly it responds gives the lag; a step up giving more than the same step down gives the bend of its throttle curve. The two pulse sizes (6% and 16%) make the bend easier to see, and the I terms take out the air the pulse itself pushes through the prop as the drone moves. The result goes into the thrust linearization and into the effectiveness identification.',
    args: [['wins', '[{ u: throttle sent, y: response along the motor\'s effect }], one per pulse'], ['dt', 'sample period [s]']],
    returns: '{ tau: lag [s]; curve: bend k; gain; fit: share explained }', shape: { tau: 1, curve: 1, gain: 1, fit: 1 },
    sample: () => [[{ u: [0.4, 0.46, 0.46, 0.34, 0.34, 0.4], y: [0, 0.01, 0.04, 0.02, -0.03, -0.01] }], 0.001] },
  { key: 'identifyServoResponse', group: 'learn', fn: identifyServoResponse, title: 'Servo response test',
    math: [`horn: <i>h</i><sub>k+1</sub> = <i>h</i><sub>k</sub> + sat<sub>±<i>ω</i>Δ<i>t</i></sub>(θ<sub>cmd</sub> − <i>h</i><sub>k</sub>), &nbsp;rotor: θ̇ = (<i>h</i> − θ)/λ`, `<i>y</i> = <i>g</i> sin θ + <i>c</i><sub>w</sub>, &nbsp;best (<i>ω</i>, λ) on a grid`],
    doc: 'Runs on the calibration\'s servo steps. One servo swings one way, then the other, while the motors and every other servo hold still. The drone\'s response along that servo\'s effect traces out where the rotor really was, so the fit gives the servo\'s real speed and lag without any angle feedback. A servo\'s trim error needs no separate number: the learned effectiveness is measured against the commanded angle, so it already includes it.',
    args: [['wins', '[{ cmd: commanded change [rad], y: response along the servo\'s effect }], one per step'], ['dt', 'sample period [s]']],
    returns: '{ rate [rad/s]; lag [s]; gain; fit }', shape: { rate: 1, lag: 1, gain: 1, fit: 1 },
    sample: () => [[{ cmd: [0, 0.2, 0.2, 0.2, -0.2, -0.2], y: [0, 0.02, 0.1, 0.19, 0.1, -0.1] }], 0.001] },

  { key: 'identifyThrow', group: 'learn', fn: identifyThrow, title: 'Identification from a throw',
    math: [`${V('f̃')} = <i>B</i><sub>f</sub>${V('u')}<sub>τ</sub> + ([${V('ω̇')}]<sub>×</sub> + [${V('ω')}]<sub>×</sub>²)${V('r')} − <i>d</i>${V('v')}<sub>b</sub> + ${V('c')}<sub>f</sub> &nbsp;(free fall: no gravity in the accelerometer)`, `${V('ω̇')} = <i>B</i><sub>α</sub>${V('u')}<sub>τ</sub> + <i>K</i>(ω<sub>y</sub>ω<sub>z</sub>, ω<sub>z</sub>ω<sub>x</sub>, ω<sub>x</sub>ω<sub>y</sub>) + ${V('c')}<sub>α</sub>, &nbsp;<i>u</i><sub>τ</sub> = <i>u</i> / (1 + τ<i>s</i>)`, `least squares for each τ in {10 … 90 ms}; the best fit gives <i>B</i>, ${V('r')} and the motor lag τ`],
    doc: 'Used by the throw start. The drone is thrown with its motors off and a random spin, and it pulses each motor briefly while it falls. Because it is in free fall, the accelerometer feels only the rotors and the IMU\'s swing around the center of gravity, so a plain least-squares fit on less than a second of data gives the effectiveness matrix, where the IMU sits relative to the balance point, the gyroscopic coupling and the motor lag, all without any description of the airframe. It pulses near the top of the throw, where the air through the props is calmest. After Blaha, Smeur and Remes (TU Delft, 2024).',
    args: [['st', 'identification state'], ['u', 'inputs: thrust fractions, times (1, cos θ, sin θ) for each joint a motor sits on'], ['f', 'accelerometer, body [m/s²]'], ['w', 'gyro, body [rad/s]'], ['vb', 'estimated velocity, body [m/s] (for air drag)'], ['dt', 'control period [s]'], ['solve', 'true to fit and return the result']],
    returns: '{ B: 6 rows × inputs; r: IMU offset from the CoG [m]; tau: motor lag [s]; fitF, fitR: share of force and rotation explained }', shape: { B: 'rows', r: 3, tau: 1, fitF: 1, fitR: 1 },
    sample: () => [{}, [0.5, 0.2], [0.1, 0, 3], [1, 0.5, 0], [0, 0, 2], 0.001, true] },

  { key: 'positionControl', group: 'ctrl', fn: positionControl, title: 'Position control',
    math: [`${V('a')}<sub>d</sub> = <i>K</i><sub>p</sub>${V('e')}<sub>p</sub> − <i>K</i><sub>d</sub>(${V('v')} − ${V('v')}<sub>cmd</sub>) + <i>K</i><sub>i</sub>∫${V('e')}<sub>p</sub> d<i>t</i>`, `${V('F')}<sub>d</sub> = <i>m</i>(${V('a')}<sub>d</sub> + <i>g</i>${V('ẑ')})`],
    doc: 'PID on the frame hub\'s position. On the learned model the controller doesn\'t know its mass, so m is 1 and the result is a desired specific force. When you fly with the keys or pads, the target moves at a commanded velocity and v arrives as the velocity error, so the damping term also feeds that velocity forward. The integral is kept by the simulator and clamped to ±2 m·s sideways and ±5 m·s vertically, so it can trim out an unknown hover throttle. m is the mass the controller believes in.',
    args: [['ep', 'position error, world [m]'], ['v', 'hub velocity − commanded velocity, world [m/s]'], ['ip', '∫ ep dt [m·s]'], ['m', 'modeled mass [kg]'], ['g', '9.81 m/s²']], returns: 'desired total force, world [N]',
    shape: 3, sample: () => [[0.1, 0, 0.1], [0, 0, 0], [0, 0, 0], 1, 9.81] },
  { key: 'thrustAxisTarget', group: 'ctrl', fn: thrustAxisTarget, title: 'Thrust-axis target',
    math: [`tilt body: ${V('n')}<sub>d</sub> = ${V('F')}<sub>d</sub> / ‖${V('F')}<sub>d</sub>‖, &nbsp;at most 35° from vertical`, `mixed: ${V('n')}<sub>d</sub> ∝ ((1 − <i>s</i>)<i>F</i><sub>x</sub>, (1 − <i>s</i>)<i>F</i><sub>y</sub>, <i>F</i><sub>z</sub>), &nbsp;<i>s</i> = the servos' share of the sideways force`, `stay level: ${V('n')}<sub>d</sub> = ${V('ẑ')}`],
    doc: 'Where the craft\'s nominal thrust axis should point. The desired attitude is built from this and the target heading. In mixed steering the body leans only for the part of the sideways force the servos aren\'t making; the simulator lowers s automatically when the servos can\'t deliver their share.',
    args: [['Fd', 'desired force, world [N]'], ['mode', '"tilt", "mixed" or "level"'], ['share', 'servos\' share of the sideways force (mixed)']], returns: 'desired thrust axis, world (normalized afterwards)',
    shape: 3, sample: () => [[1, 0, 9.81], 'mixed', 0.5] },
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
    math: [`tilt body: ${V('f')} = (${V('F')}<sub>b</sub> · ${V('n')}) ${V('n')}`, `mixed and stay level: ${V('f')} = ${V('F')}<sub>b</sub> &nbsp;(whatever the lean doesn't cover is asked of the servos)`],
    doc: 'Which part of the desired force the actuators are asked to make directly, in the body frame.',
    args: [['Fb', 'desired force, body [N]'], ['n', 'nominal thrust axis, body'], ['mode', '"tilt", "mixed" or "level"']], returns: 'force demand, body [N]',
    shape: 3, sample: () => [[0.5, 0, 9.81], [0, 0, 1], 'tilt'] },
  { key: 'thrustLinearization', group: 'ctrl', fn: thrustLinearization, title: 'Thrust linearization',
    math: [`<i>u</i> = (−(1 − <i>k̂</i>) + √((1 − <i>k̂</i>)² + 4<i>k̂v</i>)) / 2<i>k̂</i> &nbsp;so that &nbsp;(1 − <i>k̂</i>)<i>u</i> + <i>k̂u</i>² = <i>v</i>`],
    doc: 'The allocation works in thrust fractions v; this turns each into the throttle to send, using the bend k̂ the motor tests measured. It makes a motor as predictable near idle as near full power, which matters when a hard correction drives it far from hover.',
    args: [['v', 'wanted thrust as a fraction of max'], ['bend', 'learned bend k̂ (0 until measured)']], returns: 'throttle 0–1',
    shape: 'n', sample: () => [0.4, 0.3] },
  { key: 'allocation', group: 'ctrl', fn: allocation, title: 'Control allocation',
    math: [`1. ${V('u')}₁ = argmin ‖<i>W</i><sup>½</sup>(<i>B</i>${V('u')} − ${V('w')}<sub>d</sub>)‖² &nbsp;subject to &nbsp;${V('u')}<sub>min</sub> ≤ ${V('u')} ≤ ${V('u')}<sub>max</sub>`, `2. ${V('u')}* = argmin ‖<i>W</i><sup>½</sup>(<i>B</i>${V('u')} − <i>B</i>${V('u')}₁)‖² + 10<sup>−5</sup><i>ē</i> Σ<sub>j</sub> <i>q</i><sub>j</sub>((<i>u</i><sub>j</sub> − <i>r</i><sub>j</sub>)/span<sub>j</sub>)², same limits`],
    doc: 'Inputs are thrust fractions from 0 to 1, so B is in acceleration per full thrust; it comes either from the airframe description or from identification. Called twice per control step. Stage 1 decides the servos: each servo rotor contributes its thrust and a small angle change δ, bounded by how far the servo can really get in the next moment (its learned speed and lag). Stage 2 solves every motor\'s thrust at the servos\' actual angles, so the motors cover whatever a moving servo hasn\'t reached yet. The first solve gets as close to the wanted move as the limits allow; the second keeps that move and, wherever there is more than one way to make it, picks by the q, r pulls from allocationPreferences. ē is the typical effect of one input.',
    args: [['cols', 'columns of B, one 6-vector per input'], ['lo', 'lower limits'], ['hi', 'upper limits'], ['wd', 'wanted [ax, ay, az, αx, αy, αz]'], ['mode', '"tilt", "mixed" or "level"'], ['pull', '{ q, r }: preferences per input']], returns: 'one value per input',
    shape: 'alloc', sample: () => [[[0, 0, 1, 1, 1, 0.1], [0, 0, 1, -1, 1, -0.1], [0, 0, 1, -1, -1, 0.1], [0, 0, 1, 1, -1, -0.1]], [0, 0, 0, 0], [6, 6, 6, 6], [0, 0, 9.81, 0, 0, 0], 'tilt', { q: [0.02, 0.02, 0.02, 0.02], r: [3, 3, 3, 3] }] },
  { key: 'allocationPreferences', group: 'ctrl', fn: allocationPreferences, title: 'Allocation preferences',
    math: [`allowance: <i>q</i> = <i>k</i><sub>a</sub>(0.2 + 0.8<i>a</i>)(0.55 / (<i>m</i> + 0.05))², pulling toward the middle; <i>m</i> = distance to the nearer limit as a share of the range, <i>a</i> = the input's share of the steering`, `efficiency: <i>P</i> ∝ <i>T</i><sup>1.5</sup> → <i>q</i> = 0.375 <i>k</i><sub>e</sub> (<i>P</i><sub>j</sub>/Σ<i>P</i>) / √<i>u</i>, pulling toward less thrust`, `servo moves: <i>q</i> = <i>k</i><sub>s</sub>(span / 2·reach)², pulling toward staying put; reach = speed × (horizon − lag)`],
    doc: 'The tie-breakers the allocation uses when the same move can be made in more than one way. Allowance keeps every device away from its limits, so there is room to react to the next surprise; near a limit it dominates, for example easing off a nearly maxed motor and tilting a servo to make up the difference. Efficiency prefers the move that costs the least rotor power. Servo moves cost more for a slow or laggy servo, so fast corrections go to the motors and the servos take the steady part. The weights are the three sliders under Allocation; the servo speed and lag come from the actuator tests.',
    args: [['inputs', '[{ kind, x, lo, hi, power | th, range, reach }] one per input'], ['prefs', '{ allowance, efficiency, servoMove }']], returns: '{ q, r } one pull per input',
    shape: 'pull', sample: () => [[{ kind: 'thrust', x: 0.5, lo: 0, hi: 1, power: 100, authority: 1 }, { kind: 'thrust', x: 0.95, lo: 0, hi: 1, power: 100, authority: 0.5 }, { kind: 'servo', x: 0, lo: -0.2, hi: 0.2, th: 0.3, range: 0.6, reach: 0.2, authority: 0.8 }], { allowance: 0.02, efficiency: 0.02, servoMove: 0.01 }] },
];

// Wrench sum and control chain shown at the top of the Formulas tab.
const LAW_OVERVIEW = [
  `<i>m</i>${V('a')} = Σ<sub>i</sub> <i>R</i>${V('F')}<sub>i</sub> + ${V('F')}<sub>g</sub> + ${V('F')}<sub>d</sub> + Σ<sub>j</sub> <i>T</i><sub>c,j</sub>${V('n')}<sub>j</sub> + Σ ${V('F')}<sub>ground</sub>`,
  `<i>J</i>${V('ω̇')} + ${V('ω')} × <i>J</i>${V('ω')} = Σ<sub>i</sub> ${V('τ')}<sub>i</sub> + ${V('τ')}<sub>d</sub> + Σ<sub>j</sub> ${V('r')}<sub>j</sub> × <i>R</i><sup>T</sup><i>T</i><sub>c,j</sub>${V('n')}<sub>j</sub> + …`,
];
const LAW_CHAIN = {
  ctrl: ['attitudeEstimator', 'flowVelocity', 'servoPredictor', 'positionEstimator', 'identifyThrow', 'identifyMotorResponse', 'identifyServoResponse', 'identifyEffectiveness', 'positionControl', 'thrustAxisTarget', 'attitudeError', 'attitudeControl', 'forceDemand', 'allocationPreferences', 'allocation', 'thrustLinearization'],
  plant: ['batteryModel', 'servoResponse', 'servoLinkage', 'throttleCurve', 'motorResponse', 'jointRotation', 'wakeVelocity', 'rotorAero', 'rotorWrench', 'wakeLoad', 'gravity', 'bodyDrag', 'cableTension', 'groundContact', 'rigidBody', 'imuModel', 'magModel', 'baroModel', 'posFixModel', 'flowModel', 'rangeModel'],
};
