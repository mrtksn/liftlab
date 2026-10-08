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

function rigidBody(I, a, v) {
  // Newton–Euler for one body of the airframe, in its own axes, as 6-vectors [angular; linear]:
  //   f = I·a + v ×* (I·v)
  // I: { m: mass, c: centre of mass, Ic: inertia about the centre of mass }; a: spatial acceleration;
  // v: spatial velocity. f is the net force and torque the body needs. The simulator chains this through
  // every servo joint (recursive Newton–Euler) and solves for the accelerations.
  const w = [v[0], v[1], v[2]], l = [v[3], v[4], v[5]], mc = scl(I.c, I.m);
  const Iv = (x) => { const xw = [x[0], x[1], x[2]], xl = [x[3], x[4], x[5]];
    return [...add(sub(m3v(I.Ic, xw), crs(mc, crs(I.c, xw))), crs(mc, xl)), ...sub(scl(xl, I.m), crs(mc, xw))]; };
  const Ia = Iv(a), h = Iv(v), hn = [h[0], h[1], h[2]], hf = [h[3], h[4], h[5]];
  const vx = [...add(crs(w, hn), crs(l, hf)), ...crs(w, hf)];     // v ×* (I v): gyroscopic and centripetal terms
  return Ia.map((x, i) => x + vx[i]);
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





function bodyDrag(v, wind, w, density) {
  const cd = 0.1;                                        // frame drag [N per m/s of airspeed]; the rotors add their own (rotorAero)
  const cw = 0.002;                                      // rotational damping [N·m per rad/s]
  const scale = (density || 1.225) / 1.225;
  return { F: scl(sub(wind, v), cd * scale), tau: scl(w, -cw * scale) };
}

function cableTension(stretch, stretchRate, m) {
  if (stretch <= 0) return 0;                            // a slack cable carries no load
  const k = m * 15800;                                   // stiffness: ~20 Hz bounce for this payload
  const c = 2 * 0.25 * Math.sqrt(k * m);                 // 25% of critical damping
  return Math.max(0, k * stretch + c * stretchRate);     // cables pull, never push
}

function payloadDrag(v, wind, density) {
  return scl(sub(wind, v), 0.04 * (density || 1.225) / 1.225);  // [N per m/s of airspeed]
}

function groundContact(depth, v) {
  // depth: how far the contact point is inside the surface [m]; v: its velocity in the surface's axes (z out of it)
  const k = 3000, c = 60, mu = 8;                        // stiffness, damping, sliding friction
  const normal = Math.max(0, k * depth - c * v[2]), slip = Math.hypot(v[0], v[1]);
  const drag = Math.min(mu, 0.6 * normal / Math.max(1e-9, slip));
  return [-drag * v[0], -drag * v[1], normal];
}

// ─── Airflow. The simulator uses these to decide what really happens; the controller never sees them.

function motorDynamics(Omega, u, V, p, dt) {
  // A brushless motor and its prop. The ESC puts u·V across the motor; the current is what that voltage
  // leaves after the back-EMF, through the winding resistance (it may go negative: active braking).
  //   i = (u·V − Ke·Ω)/R,   τ = Ke·i,   J·dΩ/dt = τ − kQ·Ω²,   thrust = kT·Ω²,   prop drag torque = kQ·Ω²
  // p: { Ke, R, J, kT, kQ, iMax }, optionally brushDrop/friction, density,
  // lookup table and airflow load factors. Returns speed, winding current, torque and thrust.
  const drop = u > 0 ? (p.brushDrop || 0) : 0;
  const i = clamp((u * V - drop - p.Ke * Omega) / p.R, -0.5 * p.iMax, p.iMax);
  const tau = p.Ke * i;
  const scale = (p.rho || 1.225) / (p.rhoRef || 1.225), qf = p.qFactor ?? 1, tf = p.tFactor ?? 1;
  const load = p.table ? FlightPhysics.loadAt(p, Omega, p.rho, qf, tf).Q : p.kQ * Omega * Omega * scale * qf;
  const friction = Omega > 0 ? (p.friction || 0) : Math.min(p.friction || 0, Math.max(0, tau));
  // Semi-implicit damping avoids stiff low-inertia motors oscillating at the fixed step.
  const damping = p.Ke * p.Ke / p.R + 2 * load / Math.max(1, Omega);
  const O = Math.max(0, Omega + (tau - load - friction) * dt / (p.J + dt * damping));
  const T = p.table ? FlightPhysics.loadAt(p, O, p.rho, qf, tf).T : p.kT * O * O * scale * tf;
  return { Omega: O, i, tau, T };
}

function servoTorque(err, rate, p) {
  // A hobby servo is a geared DC motor with a position loop. Its torque falls linearly with speed:
  //   τ = τ_stall · (sat(err / band) − θ̇ / ω_no-load)
  // So it can't beat its no-load speed, can't push harder than its stall torque, and a heavy or
  // aerodynamically loaded arm drags it off its target. err: target − angle [rad]; rate: angle rate [rad/s];
  // p: { stall [N·m], speed: no-load speed [rad/s], band: error that gives full power [rad] }.
  return p.stall * (clamp(err / p.band, -1, 1) - rate / p.speed);
}

function batteryModel(st, current, dt, p) {
  // A LiPo: its resting voltage follows the usual discharge curve (flat through the middle, falling away below
  // about 15%), and it sags under current through its internal resistance, which grows as the pack empties.
  // Past empty (over-discharge) the voltage collapses. current: total draw [A] (negative when braking motors push
  // charge back). p: { cells still working, capacity [C] after wear, rInt [Ω] at its temperature, cut: disconnected }
  const cells = p ? p.cells : 4, capacity = p ? p.capacity : 1.3 * 3600, rInt = p ? p.rInt : 0.06;
  if (st.soc === undefined) st.soc = 1;
  if (p && p.cut) return 0;
  st.soc = clamp(st.soc - current * dt / Math.max(1, capacity), -0.08, 1);
  const curve = [[-0.08, 2.0], [-0.04, 2.8], [0, 3.2], [0.05, 3.45], [0.1, 3.6], [0.15, 3.67], [0.2, 3.71], [0.3, 3.75], [0.4, 3.79],
    [0.5, 3.83], [0.6, 3.87], [0.7, 3.92], [0.8, 3.98], [0.9, 4.06], [0.95, 4.13], [1, 4.2]];   // [charge, resting volts per cell]
  let ocv = curve[curve.length - 1][1];
  for (let i = 1; i < curve.length; i++) {
    const a = curve[i - 1], b = curve[i];
    if (st.soc <= b[0]) { ocv = a[1] + (b[1] - a[1]) * (st.soc - a[0]) / (b[0] - a[0]); break; }
  }
  const r = rInt * (1 + 1.5 * clamp((0.2 - st.soc) / 0.2, 0, 1.4));   // a nearly empty pack sags more
  return Math.max(0, cells * ocv - r * current);   // terminal voltage [V]
}

function thermalModel(T, P, G, C, Tamb, dt) {
  // A part that heats up and cools down: heat capacity C [J/K], heated by P [W], losing G·(T − Tamb) to the
  // air (G [W/K] grows with airflow). Motors: P = i²R in the windings. Battery: P = I²R inside the pack.
  return T + dt * (P - G * (T - Tamb)) / C;
}

function wakeVelocity(point, rotors, density) {
  // Air velocity that the rotors' wakes induce at `point`, body frame. rotors: [{ p, d, T, R, va }]
  // (p: disc center, d: thrust axis, T: thrust [N], R: prop radius [m], va: oncoming air at the disc, body
  // frame). Wind and forward flight blow the wake sideways as it travels down, so in forward flight the
  // rear rotors fly into the front rotors' wash.
  let w = [0, 0, 0];
  for (const r of rotors) {
    if (r.T <= 0) continue;
    const vh = Math.sqrt(r.T / (2 * (density || 1.225) * Math.PI * r.R * r.R));  // induced velocity at the disc (momentum theory)
    const rel = sub(point, r.p), s = -dot(rel, r.d);               // how far downstream of the disc
    if (s < -r.R) continue;                                        // well above the disc: no effect
    const va = r.va || [0, 0, 0], vs = sub(va, scl(r.d, dot(va, r.d)));   // crosswind at the disc
    const drift = scl(vs, Math.max(0, s) / (1.5 * vh));            // how far the wake has been blown by this depth
    const radial = nrm(sub(add(rel, scl(r.d, s)), drift));
    const Rw = s > 0 ? r.R * (0.71 + 0.29 * Math.exp(-s / r.R)) : r.R;   // the wake contracts to ~0.71 R
    const edge = clamp((1.1 * Rw - radial) / (0.2 * Rw), 0, 1);          // soft wake boundary
    if (edge <= 0) continue;
    const speed = vh * (1 + s / Math.sqrt(s * s + r.R * r.R))            // speeds up to 2·vh below the disc…
      * Math.exp(-Math.max(0, s) / (12 * r.R));                          // …then mixes out far downstream
    w = add(w, scl(r.d, -speed * edge));
  }
  return w;
}

function rotorAero(T, R, vAxial, vInPlane, h, density) {
  // T: thrust this command would make in still air [N]; R: prop radius [m]
  // vAxial: air flowing into the disc from above along its axis [m/s] (climbing, or another rotor's wake)
  // vInPlane: air velocity across the disc, body frame [m/s]; h: height of the disc above the ground [m]
  if (T <= 0) return { T: 0, H: [0, 0, 0], Qfactor: 1 };
  const vh = Math.sqrt(T / (2 * (density || 1.225) * Math.PI * R * R));          // induced velocity in hover
  const ve = nrm(vInPlane);
  let vi = vh;                                                      // induced velocity now (Glauert): edgewise flow lowers it
  for (let n = 0; n < 6; n++) vi = 0.5 * vi + 0.5 * vh * vh / Math.sqrt(ve * ve + (vAxial + vi) ** 2 + 1e-9);
  let k = clamp(1 - 0.5 * (vAxial + vi - vh) / vh, 0.3, 1.3);       // more air through the disc than in hover costs thrust
  const hh = Math.max(h, R / 2);
  const ground = Math.min(1.3, 1 / (1 - (R / (4 * hh)) ** 2));
  k *= ground;                 // ground effect (Cheeseman–Bennett)
  const x = -vAxial / vh;                                           // descending into its own wake…
  k *= 1 - 0.3 * Math.exp(-(((x - 1.2) / 0.45) ** 2)) * Math.exp(-((ve / vh) ** 2));   // …vortex ring state costs up to 30%, less when moving sideways
  const cH = 0.03;                                                  // rotor drag from blade flapping [1/(m/s)]
  // Generic induced/profile power split. The previous step's correction also
  // loads the motor; forward flight and ground effect no longer change thrust alone.
  const Qfactor = clamp(k * (0.2 * (1 + 0.02 * ve * ve / (vh * vh)) + 0.8 * Math.max(0.05, (vAxial + vi) / vh) / Math.sqrt(ground)), 0.15, 2.5);
  return { T: T * k, H: scl(vInPlane, cH * T), Qfactor };
}

function wingAero(u, chord, span, density, polar) {
  // A wing in the air u [m/s] (the air's velocity past it, in the wing's own axes: x toward the leading edge, y along
  // the span, z up from its top). Lift rises with the angle of attack up to a stall at about 15°, then falls off to
  // what a flat plate gives; drag is a base drag, plus the lift's own (induced), plus a flat plate's once stalled.
  // Returns the force [N] in the wing's axes, and how far ahead of the wing's middle it acts (centre of pressure, m:
  // the quarter chord while the flow is attached, the middle once stalled).
  const rho = density || 1.225, S = chord * span, AR = span / Math.max(1e-3, chord);
  const V = Math.hypot(u[0], u[2]); if (V < 1e-3) return { F: [0, 0, 0], xcp: chord / 4 };
  const a = Math.atan2(u[2], -u[0]);                      // angle of attack: air from below and ahead is positive
  const cla = 2 * Math.PI / (1 + 2 / AR), stall = 15 * Math.PI / 180;
  const sa = Math.sin(a), ca = Math.cos(a);
  const att = 1 / (1 + Math.exp((Math.abs(Math.asin(sa)) - stall) / 0.035));   // 1 attached … 0 stalled (blended)
  let cl = att * cla * Math.asin(sa) * Math.sign(ca || 1) + (1 - att) * 2 * sa * ca;
  let cd = 0.02 + att * cl * cl / (Math.PI * 0.8 * AR) + (1 - att) * 1.9 * sa * sa;
  const degrees = a * 180 / Math.PI;
  if (polar && polar.length > 1 && degrees >= polar[0][0] && degrees <= polar[polar.length - 1][0]) {
    let lo = 0, hi = polar.length - 1;
    while (hi - lo > 1) { const m = (hi + lo) >> 1; if (polar[m][0] <= degrees) lo = m; else hi = m; }
    const f = (degrees - polar[lo][0]) / (polar[hi][0] - polar[lo][0]);
    cl = polar[lo][1] + f * (polar[hi][1] - polar[lo][1]); cd = polar[lo][2] + f * (polar[hi][2] - polar[lo][2]);
  } else if (polar && polar.length > 1) {
    const edge = degrees < polar[0][0] ? polar[0] : polar[polar.length - 1];
    const blend = clamp(Math.abs(degrees - edge[0]) / 10, 0, 1);
    cl = edge[1] * (1 - blend) + cl * blend; cd = edge[2] * (1 - blend) + cd * blend;
  }
  const q = 0.5 * rho * V * V * S, d = [u[0] / V, 0, u[2] / V], l = [d[2], 0, -d[0]];   // drag along the air, lift across it
  const F = [q * (cd * d[0] + cl * l[0]), 0.05 * 0.5 * rho * Math.abs(u[1]) * u[1] * S, q * (cd * d[2] + cl * l[2])];   // (and a little drag along the span)
  return { F, xcp: chord / 4 * att };
}

function bluffDrag(u, areas, density, coefficient) {
  // A blunt part (a box, a battery, a hub) in the air u [m/s], its own axes: each face's drag, as a box's
  // (Cd 1.05), on the frontal area it shows that way [m²].
  const rho = density || 1.225, cd = coefficient ?? 1.05, V = nrm(u);
  return [0, 1, 2].map(i => 0.5 * rho * cd * areas[i] * V * u[i]);
}

function wakeLoad(w, area, density) {
  // Force on a part with frontal area [m²] sitting in wake air moving at w [m/s]
  return scl(w, 0.5 * (density || 1.225) * 1.1 * area * nrm(w));
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
    // each axis has its own gain error, and the axes aren't quite square to each other or to the package
    const mis = () => { const e = [0, 1, 2].map(() => randn() * (p.misalign || 0)); return [1, -e[2], e[1], e[2], 1, -e[0], -e[1], e[0], 1]; };
    st.Mg = mis().map((v, k) => k % 4 === 0 ? 1 + randn() * (p.scaleErr || 0) : v);
    st.Ma = mis().map((v, k) => k % 4 === 0 ? 1 + randn() * (p.scaleErr || 0) : v);
  }
  w = m3v(st.Mg, w); f = m3v(st.Ma, f);
  for (let i = 0; i < 3; i++) st.bg[i] += randn() * p.gyroDrift * Math.sqrt(dt);   // gyro bias random walk
  const gyro = [0, 1, 2].map(i => clamp(w[i] + vib.w[i] + st.bg[i] + randn() * p.gyroNoise, -p.gyroRange, p.gyroRange));
  const accel = [0, 1, 2].map(i => clamp(f[i] + vib.a[i] + st.ba[i] + randn() * p.accNoise, -p.accRange, p.accRange));
  return { gyro, accel };
}

function magModel(b, interference, p, st) {
  // b: Earth's field in the sensor frame (strength 1); interference: field from nearby motor currents
  if (!st.hi) {
    st.hi = [0, 1, 2].map(() => randn() * p.hardIron);   // hard-iron offset from the airframe
    st.si = [0, 1, 2].flatMap(i => [0, 1, 2].map(j => (i === j ? 1 : 0) + randn() * (p.softIron || 0)));   // soft iron: nearby steel bends the field
  }
  const bb = m3v(st.si, b);
  return [0, 1, 2].map(i => bb[i] + st.hi[i] + interference[i] + randn() * p.noise);
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
  // with the estimate from that moment. Height starts from the rangefinder (height above the ground), else the
  // barometer, else the fix. The barometer reads pressure altitude, which has its own zero: while the
  // rangefinder sees the ground, the barometer's offset from it is learned (st.bo), so the two agree.
  const kP = 0.8, kV = 0.3, kFixV = 0.6, kBaro = 1.5, kBaroV = 0.6;
  const kFlow = 2.0, kRange = 2.5, kRangeV = 1.2;        // optical flow velocity, rangefinder height
  const kBias = 0.4;                                     // learns the accelerometer's horizontal bias from velocity errors
  const cd = 0.25, kDrag = 1.0;                          // airframe drag coefficient [N per m/s] (see bodyDrag), drag-fusion gain
  if (!st.p) {
    st.p = [0, 0, 0]; st.v = [0, 0, 0]; st.bo = 0;       // st.bo: the barometer's offset
    if (fix) { st.p[0] = fix.p[0]; st.p[1] = fix.p[1]; st.p[2] = fix.p[2]; }
    if (baro) st.p[2] = baro.alt;
    if (flow && flow.h != null) { st.p[2] = flow.h; if (baro) st.bo = baro.alt - flow.h; }
  }
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
    const e = baro.alt - st.bo - past(baro.age).p[2]; st.p[2] += w * kBaro * e * dt; st.v[2] += w * kBaroV * e * dt;
    if (range) st.bo += 0.5 * e * dt;                    // while the rangefinder sees the ground, learn the barometer's zero
  }
  st.h.push({ p: st.p.slice(), v: st.v.slice() }); if (st.h.length > 800) st.h.shift();
  return { p: st.p.slice(), v: st.v.slice() };
}

// ═════════════ Identification (learning what the actuators do) ═════════════

function identifyEffectiveness(st, u, f, w, r, dt, init, memory, lags, mot) {
  // Recursive least squares. Learns B, what each actuator input does to the drone, from flight data:
  //   Δ[f; dω/dt] ≈ B · Δu + [0; B₂] · Δ(dx/dt)
  // B₂: a rotor speeding up or slowing down twists the frame the other way (x ≈ √thrust, its speed). It is
  // learned alongside so those twists don't get mistaken for B. mot: each input's motor command and basis
  // factor (as for identifyThrow).
  // Both sides are band-passed (0.3–12 Hz), so it learns from changes and steady offsets such as drag
  // or trim can't leak into B.
  // u: inputs: thrust fractions, times (1, cos θ, sin θ) for every joint a motor sits on
  // f: accelerometer (specific force) and w: gyro, body frame; r: IMU position from the hub [m]
  // init: starting guess, 6 rows × inputs; memory: how long past data counts [s];
  // lags: each input's motor lag [s], from the actuator tests (35 ms assumed until then).
  // Returns B: 6 rows (ax ay az αx αy αz) × inputs.
  const lpHz = 12, hpHz = 0.3, pMax = 2;                    // band [Hz], covariance cap
  const n = u.length, m = mot ? 2 * n : n;
  const k = hz => dt / (dt + 1 / (2 * Math.PI * hz));
  const out = () => ({ B: st.th.map(row => row.slice(0, n)), B2: m > n ? st.th.slice(3).map(row => row.slice(n)) : null });
  if (dt <= 0) return st.th ? out() : { B: init };
  if (!st.P) {                                                   // start the filters from the first real reading
    st.th = init.map(row => row.concat(new Array(m - n).fill(0)));
    st.P = eye(m).map(row => row.map(x => x * 2));
    st.ua = u.slice(); st.um = mot ? mot.v.slice() : []; st.sp = new Array(n).fill(0);
    const x0 = u.concat(new Array(m - n).fill(0));
    st.xl = x0.slice(); st.xs = x0.slice(); st.yl = f.concat([0, 0, 0]); st.ys = st.yl.slice();
    st.wl = w.slice(); st.n = 0;
  }
  const raw = new Array(m).fill(0);
  for (let j = 0; j < n; j++) {
    const lag = Math.min(1, dt / ((lags && lags[j]) || 0.035));
    st.ua[j] += (u[j] - st.ua[j]) * lag; raw[j] = st.ua[j];                  // what the motors are doing now
    if (m > n) {
      st.um[j] += (mot.v[j] - st.um[j]) * lag;
      const sp = mot.coll && mot.coll[j] ? 0 : Math.sqrt(Math.max(0, st.um[j])) * mot.phi[j];   // its speed, as a fraction of full (a collective rotor's is held)
      raw[n + j] = (sp - st.sp[j]) / dt; st.sp[j] = sp;
    }
  }
  const wPrev = st.wl.slice();
  st.wl = st.wl.map((v, i) => v + k(lpHz) * (w[i] - v));
  const alpha = st.wl.map((v, i) => (v - wPrev[i]) / dt);                     // angular acceleration
  const fHub = sub(sub(f, crs(alpha, r)), crs(st.wl, crs(st.wl, r)));         // remove the IMU's lever-arm swing
  const yRaw = [...fHub, ...alpha];
  st.xl = st.xl.map((v, j) => v + k(lpHz) * (raw[j] - v));
  st.yl = st.yl.map((v, i) => i < 3 ? v + k(lpHz) * (yRaw[i] - v) : alpha[i - 3]);   // alpha is already filtered once, like the inputs
  st.xs = st.xs.map((v, j) => v + k(hpHz) * (st.xl[j] - v));
  st.ys = st.ys.map((v, i) => v + k(hpHz) * (st.yl[i] - v));
  if (++st.n * dt < 1.5) return out();                                     // let the filters settle first
  const x = st.xl.map((v, j) => v - st.xs[j]), y = st.yl.map((v, i) => v - st.ys[i]);
  const P = st.P, Px = P.map(row => row.reduce((s, v, j) => s + v * x[j], 0));
  const lambda = Math.exp(-dt / memory), den = lambda + x.reduce((s, v, j) => s + v * Px[j], 0);
  const K = Px.map(v => v / den);
  st.e = st.th.map((row, i) => y[i] - row.reduce((s, v, j) => s + v * x[j], 0));   // prediction error before learning
  st.y = y; st.x = x;
  st.th = st.th.map((row, i) => row.map((v, j) => v + K[j] * st.e[i]));
  let trace = 0;
  for (let a = 0; a < m; a++) {
    const row = P[a], ka = K[a];
    for (let b = 0; b < m; b++) row[b] = (row[b] - ka * Px[b]) / lambda;
    trace += row[a];
  }
  if (trace > pMax * m) for (const row of P) for (let b = 0; b < m; b++) row[b] *= pMax * m / trace;   // don't blow up without excitation
  return out();
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
  // Each window's mean response and starting throttle are taken out first (each window has its own offset c_w
  // anyway): the same fit, but the sums stay small, which the drone's 32-bit arithmetic needs.
  let sst = 0;
  const ym = new Array(nw).fill(0);
  for (let wi = 0; wi < nw; wi++) {
    const w = wins[wi]; let m = 0; for (const v of w.y) m += v;
    m /= Math.max(1, w.y.length); ym[wi] = m;
    for (const v of w.y) sst += (v - m) ** 2;
  }
  let bestSse = -1, bestTau = 0, bestG = 0, bestGk = 0;
  for (let it = 0; it < taus.length; it++) {
    const tau = taus[it], a = Math.min(1, dt / tau), A = [], b = new Array(np).fill(0);
    for (let p = 0; p < np; p++) A.push(new Array(np).fill(0));
    let yy = 0;
    for (let wi = 0; wi < nw; wi++) {
      const w = wins[wi], u0 = w.u[0], q0 = u0 * u0 - u0, phi = new Array(np).fill(0);
      let m1 = u0, m2 = q0, l1 = m1, l2 = m2, I = 0;
      phi[4 + wi] = 1;
      for (let t = 0; t < w.u.length; t++) {
        const u = w.u[t], y = w.y[t] - ym[wi];
        m1 += (u - m1) * a; m2 += (u * u - u - m2) * a; l1 += kf * (m1 - l1); l2 += kf * (m2 - l2); I += (l1 - u0) * dt;
        phi[0] = l1 - u0; phi[1] = l2 - q0; phi[2] = I; phi[3] = (l1 - u0) * I;
        for (let p = 0; p < np; p++) { b[p] += phi[p] * y; for (let q = 0; q < np; q++) A[p][q] += phi[p] * phi[q]; }
        yy += y * y;
      }
    }
    for (let p = 0; p < np; p++) A[p][p] += 1e-9;
    const th = solveLin(A, b);
    let sse = yy;
    for (let p = 0; p < np; p++) { let r = 0; for (let q = 0; q < np; q++) r += A[p][q] * th[q]; sse += th[p] * r - 2 * th[p] * b[p]; }
    if (bestSse < 0 || sse < bestSse) { bestSse = sse; bestTau = tau; bestG = th[0]; bestGk = th[1]; }
  }
  if (bestSse < 0 || Math.abs(bestG) < 1e-6) return { tau: 0, curve: 0, gain: 0, fit: 0 };
  return { tau: bestTau, curve: clamp(bestGk / bestG, -0.5, 1.5), gain: bestG, fit: clamp(1 - bestSse / Math.max(1e-12, sst), 0, 1) };
}

function identifyServoResponse(wins, dt) {
  // One servo stepped on its own while everything else holds still. cmd: commanded angle change from the
  // start of the window [rad]; y: the drone's response along this servo's effect, which is sin(angle change)
  // for a rigid tilting rotor. Model: the horn moves at most `rate`, the rotor follows with lag λ,
  //   y = g · sin(θ) + c_w.
  // Rate and lag are found on a grid (each pair is a two-number least-squares fit); the best pair wins.
  const rates = [60, 80, 110, 150, 200, 260, 340, 450, 600, 800], lags = [0, 0.005, 0.01, 0.015, 0.02, 0.03, 0.045, 0.065, 0.09], lpHz = 25;
  const kf = dt / (dt + 1 / (2 * Math.PI * lpHz)), nw = wins.length, np = 1 + nw;
  let sst = 0;
  const ym = new Array(nw).fill(0);   // each window's mean response, taken out first (as in identifyMotorResponse)
  for (let wi = 0; wi < nw; wi++) {
    const w = wins[wi]; let m = 0; for (const v of w.y) m += v;
    m /= Math.max(1, w.y.length); ym[wi] = m;
    for (const v of w.y) sst += (v - m) ** 2;
  }
  let bestSse = -1, bestRate = 0, bestLag = 0, bestG = 0;
  for (let ir = 0; ir < rates.length; ir++) {
    for (let il = 0; il < lags.length; il++) {
      const rate = rates[ir] * Math.PI / 180, lag = lags[il], A = new Array(np).fill(0).map(() => new Array(np).fill(0)), b = new Array(np).fill(0);
      let yy = 0;
      for (let wi = 0; wi < nw; wi++) {
        const w = wins[wi], phi = new Array(np).fill(0);
        let h = 0, th = 0, l = 0;
        phi[1 + wi] = 1;
        for (let t = 0; t < w.cmd.length; t++) {
          const y = w.y[t] - ym[wi];
          h += clamp(w.cmd[t] - h, -rate * dt, rate * dt);
          th = lag > 0 ? th + (h - th) * Math.min(1, dt / lag) : h;
          l += kf * (Math.sin(th) - l);
          phi[0] = l;
          for (let p = 0; p < np; p++) { b[p] += phi[p] * y; for (let q = 0; q < np; q++) A[p][q] += phi[p] * phi[q]; }
          yy += y * y;
        }
      }
      for (let p = 0; p < np; p++) A[p][p] += 1e-9;
      const x = solveLin(A, b);
      let sse = yy;
      for (let p = 0; p < np; p++) { let r = 0; for (let q = 0; q < np; q++) r += A[p][q] * x[q]; sse += x[p] * r - 2 * x[p] * b[p]; }
      if (bestSse < 0 || sse < bestSse) { bestSse = sse; bestRate = rate; bestLag = lag; bestG = x[0]; }
    }
  }
  if (bestSse < 0) return { rate: 0, lag: 0, gain: 0, fit: 0 };
  return { rate: bestRate, lag: bestLag, gain: bestG, fit: clamp(1 - bestSse / Math.max(1e-12, sst), 0, 1) };
}

function identifyThrow(st, u, f, w, vb, dt, solve, mot, budget) {
  // Batch least squares over a free fall, for a drone that knows nothing about itself (after Blaha,
  // Smeur & Remes, TU Delft 2024). The motors start from zero and the drone is falling, so the
  // accelerometer feels only the rotors plus its own swing around the center of gravity (CoG):
  //   f = B_f · u_τ + ([α]× + [ω]×²) r − d v_b + c_f    r: IMU offset from the CoG, d: drag; both shared by the rows
  //   α = B_α · u_τ + B₂ · dx/dt + K (ω_y ω_z, ω_z ω_x, ω_x ω_y) + c_α     K: gyroscopic coupling (inertia ratios)
  // B₂ is each rotor spinning up or down: the motor's torque pushes the frame back (their G₂). Without it,
  // pulses from standstill look like a huge yaw effect. This drone doesn't measure prop speed, so it runs a
  // generic brushless model for each motor (speed x as a fraction of full, back-EMF, a current limit, prop
  // drag; only its time constant τ unknown): u_τ = x² is the thrust. A collective-pitch rotor (mot.coll) holds its
  // speed, so its thrust just follows the command through a lag τ and it has no spin-up reaction.
  // Built to fit a small computer:
  //   solve 'fall' (while falling): one running fit per candidate τ (all motors the same), a fixed cost per
  //     step, and a compact 250 Hz record;
  //   solve 'catch': picks the best of those fits at once, so it can catch itself straight away;
  //   solve 'refine': afterwards, in the background, finds each motor's own τ from the record (one motor at
  //     a time, keeping what explains the rotation best), spending at most `budget` operations per call.
  //     A big slow rotor and small fast ones can then share a frame. out.refined is 1 when finished.
  // mot: per input, its motor's thrust command v, basis factor phi (inputs are thrust × (1, cos θ, sin θ)
  // products), motor number m and whether it is a collective-pitch rotor. Returns B (6 rows × inputs), B2 (3
  // rows), r [m], each motor's lag and how much of the force and rotation it explains.
  const taus = [0.01, 0.02, 0.03, 0.045, 0.065, 0.09, 0.13], nt = 7, lpHz = 25, skip = 0.03, logEvery = 0.004, logMax = 450;
  const n = u.length, nf = 3 * n + 7, nr = 2 * n + 4;
  // The motors: an input's group is its motor (inputs of one motor share its lag); first[g] is its first input.
  const gOf = new Array(n).fill(0), first = new Array(n).fill(0);
  let groups = 0;
  for (let j = 0; j < n; j++) {
    let g = -1;
    for (let k = 0; k < j; k++) if (k < groups && mot.m[first[k]] === mot.m[j]) g = k;
    if (g < 0) { g = groups; first[groups] = j; groups++; }
    gOf[j] = g;
  }
  const ng = Math.max(0, Math.min(groups, n));
  const kOf = h => h / (h + 1 / (2 * Math.PI * lpHz));
  const Lof = (a, W) => {                                                  // [α]× + [ω]×², by rows
    const ww = W[0] * W[0] + W[1] * W[1] + W[2] * W[2];
    return [W[0] * W[0] - ww, -a[2] + W[0] * W[1], a[1] + W[0] * W[2], a[2] + W[1] * W[0], W[1] * W[1] - ww, -a[0] + W[1] * W[2],
      -a[1] + W[2] * W[0], a[0] + W[2] * W[1], W[2] * W[2] - ww];
  };
  const gyroOf = W => [W[1] * W[2], W[2] * W[0], W[0] * W[1]];
  const zeros = (a, b) => new Array(a).fill(0).map(() => new Array(b).fill(0));
  const newZ = () => ({ x: new Array(n).fill(0), ul: new Array(n).fill(0), sq: new Array(n).fill(0), dq: new Array(n).fill(0) });
  // One step of the regressors with each motor group's lag tauG[g], updating the filter state z.
  const stepRegs = (z, v, ph, h, tauG) => {
    const k = kOf(h);
    for (let j = 0; j < n; j++) {
      const tau = tauG[gOf[j]];
      if (mot.coll[j]) {                                                   // collective pitch: speed held, thrust follows the pitch with a lag
        z.x[j] += (Math.max(0, v[j]) - z.x[j]) * Math.min(1, h / tau);
        z.ul[j] += k * (z.x[j] * ph[j] - z.ul[j]); z.dq[j] = 0;
      } else {
        const xt = Math.sqrt(Math.max(0, v[j]));
        const drive = clamp(xt * xt + 4 * xt - 4 * z.x[j], -1, 2) - z.x[j] * z.x[j];   // motor torque − prop drag, per full-thrust torque
        z.x[j] = Math.max(0, z.x[j] + drive / (5.26 * tau) * h);
        z.ul[j] += k * (z.x[j] * z.x[j] * ph[j] - z.ul[j]);              // thrust, through the same filter as the gyro
        const q = z.sq[j] + k * (z.x[j] * ph[j] - z.sq[j]);
        z.dq[j] = (q - z.sq[j]) / h; z.sq[j] = q;                          // prop acceleration
      }
    }
  };
  // The rotation rows share their regressors: [u_τ, dx/dt, gyroscopic terms, 1].
  const rotRegs = (z, W) => {
    const phi = new Array(nr).fill(0), gy = gyroOf(W);
    for (let j = 0; j < n; j++) { phi[j] = z.ul[j]; phi[n + j] = z.dq[j]; }
    phi[2 * n] = gy[0]; phi[2 * n + 1] = gy[1]; phi[2 * n + 2] = gy[2]; phi[2 * n + 3] = 1;
    return phi;
  };
  const addRot = (Ar, br, phi, a) => {
    for (let p = 0; p < nr; p++) {
      const c = phi[p];
      if (c !== 0) { for (let q = 0; q < nr; q++) Ar[p][q] += c * phi[q]; for (let i = 0; i < 3; i++) br[i][p] += c * a[i]; }
    }
  };
  const addForce = (Af, bf, z, fl, L, vbb) => {                           // force rows share r and drag
    const idx = new Array(n + 5).fill(0), val = new Array(n + 5).fill(0);
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < n; j++) { idx[j] = i * n + j; val[j] = z.ul[j]; }
      for (let j = 0; j < 3; j++) { idx[n + j] = 3 * n + j; val[n + j] = L[3 * i + j]; }
      idx[n + 3] = 3 * n + 3 + i; val[n + 3] = 1; idx[n + 4] = 3 * n + 6; val[n + 4] = -vbb[i];
      for (let p = 0; p < n + 5; p++) { bf[idx[p]] += val[p] * fl[i]; for (let q = 0; q < n + 5; q++) Af[idx[p]][idx[q]] += val[p] * val[q]; }
    }
  };
  const ridgeSolve = (A, b) => {
    const R = A.map((r, p) => { const row = r.slice(); row[p] += 1e-9 + 1e-6 * r[p]; return row; });
    return solveLin(R, b);
  };
  const sseOf = (A, b, th, y2) => {
    let s = y2;
    for (let p = 0; p < th.length; p++) { let r = 0; for (let q = 0; q < th.length; q++) r += A[p][q] * th[q]; s += th[p] * r - 2 * th[p] * b[p]; }
    return s;
  };
  const sst = (yy, ys, N, i) => Math.max(1e-9, yy[i] - ys[i] * ys[i] / N);
  const fitRot = (Ar, br, yy, ys, N) => {
    const thR = [ridgeSolve(Ar, br[0]), ridgeSolve(Ar, br[1]), ridgeSolve(Ar, br[2])];
    let e = 0; for (let i = 0; i < 3; i++) e += sseOf(Ar, br[i], thR[i], yy[3 + i]);
    return { thR, fitR: clamp(1 - e / (sst(yy, ys, N, 3) + sst(yy, ys, N, 4) + sst(yy, ys, N, 5)), 0, 1) };
  };
  const fitForce = (Af, bf, yy, ys, N) => {
    const thF = ridgeSolve(Af, bf);
    return { thF, fitF: clamp(1 - sseOf(Af, bf, thF, yy[0] + yy[1] + yy[2]) / (sst(yy, ys, N, 0) + sst(yy, ys, N, 1) + sst(yy, ys, N, 2)), 0, 1) };
  };
  const result = (Rr, Rf, tauG, refined, improved) => {
    const B = [], B2 = [], tg = new Array(ng).fill(0);
    for (let i = 0; i < 6; i++) { const row = new Array(n).fill(0); for (let j = 0; j < n; j++) row[j] = i < 3 ? Rf.thF[i * n + j] : Rr.thR[i - 3][j]; B.push(row); }
    for (let i = 0; i < 3; i++) { const row = new Array(n).fill(0); for (let j = 0; j < n; j++) row[j] = Rr.thR[i][n + j]; B2.push(row); }
    let tm = 0; for (let g = 0; g < ng; g++) { tg[g] = tauG[g]; tm += tauG[g]; }
    return { B, B2, r: [Rf.thF[3 * n], Rf.thF[3 * n + 1], Rf.thF[3 * n + 2]], drag: Rf.thF[3 * n + 6], tau: tm / Math.max(1, ng), taus: tg,
      fitF: Rf.fitF, fitR: Rr.fitR, refined, improved, progress: refined, spent: 0 };
  };
  const empty = () => ({ B: zeros(6, n), B2: zeros(3, n), r: [0, 0, 0], drag: 0, tau: 0, taus: new Array(ng).fill(0), fitF: 0, fitR: 0, refined: 0, improved: 0, progress: 0, spent: 0 });

  if (!st.fits && !st.done) {                                              // the start: a fit for each τ
    st.fits = taus.map(tau => ({ tau, z: newZ(), Ar: zeros(nr, nr), br: zeros(3, nr), Af: zeros(nf, nf), bf: new Array(nf).fill(0) }));
    st.wl = w.slice(); st.fl = f.slice(); st.t = 0; st.log = []; st.hLog = 0; st.yy = [0, 0, 0, 0, 0, 0]; st.ys = [0, 0, 0, 0, 0, 0]; st.N = 0;
  }
  if (dt > 0 && st.fits) {                                                 // falling: update the running fits
    const k = kOf(dt), a = [0, 0, 0];
    for (let i = 0; i < 3; i++) { const wn = st.wl[i] + k * (w[i] - st.wl[i]); a[i] = (wn - st.wl[i]) / dt; st.wl[i] = wn; st.fl[i] += k * (f[i] - st.fl[i]); }
    const W = st.wl, L = Lof(a, W), tauG = new Array(ng).fill(0);
    st.t += dt;
    for (const F of st.fits) {
      for (let g = 0; g < ng; g++) tauG[g] = F.tau;
      stepRegs(F.z, mot.v, mot.phi, dt, tauG);
      if (st.t >= skip) { addRot(F.Ar, F.br, rotRegs(F.z, W), a); addForce(F.Af, F.bf, F.z, st.fl, L, vb); }
    }
    if (st.t >= skip) { for (let i = 0; i < 3; i++) { st.yy[i] += st.fl[i] * st.fl[i]; st.ys[i] += st.fl[i]; st.yy[3 + i] += a[i] * a[i]; st.ys[3 + i] += a[i]; } st.N++; }
    st.hLog += dt;                                                         // the record: motor commands, basis factors, readings
    if (st.hLog >= logEvery - 1e-9 && st.log.length < logMax) {
      const vg = new Array(ng).fill(0); for (let g = 0; g < ng; g++) vg[g] = mot.v[first[g]];
      st.log.push({ v: vg, ph: mot.phi.slice(), fl: st.fl.slice(), a, W: W.slice(), vb: vb.slice(), h: st.hLog, use: st.t >= skip ? 1 : 0 });
      st.hLog = 0;
    }
  }
  if (solve === 'catch') {                                                 // catch: the best single lag, at once
    if (st.N < 20 || !st.fits) return st.out ? st.out : empty();
    let best = 0, bestFit = -1;
    for (let i = 0; i < nt; i++) { const R = fitRot(st.fits[i].Ar, st.fits[i].br, st.yy, st.ys, st.N); if (R.fitR > bestFit) { bestFit = R.fitR; best = i; } }   // the lag that explains the rotation best
    const F = st.fits[best], tauG = new Array(ng).fill(0);
    for (let g = 0; g < ng; g++) tauG[g] = F.tau;
    st.out = result(fitRot(F.Ar, F.br, st.yy, st.ys, st.N), fitForce(F.Af, F.bf, st.yy, st.ys, st.N), tauG, ng < 2 ? 1 : 0, 0);
    if (ng >= 2) { st.tauG = tauG; st.job = { k: 0, credit: 0, spent: 0, changed: 0, ready: 0, base: 0 }; }   // each motor's own lag, in the background
    st.fits = null; st.done = 1;                                           // the running fits aren't needed any more
    return st.out;
  }
  if (solve === 'refine' && st.job && st.out) {                            // background: each motor's own lag, from the record
    const J = st.job, S = st.log, nS = S.length, firstPass = ng * nt, total = 2 * firstPass;
    const yy = [0, 0, 0, 0, 0, 0], ys = [0, 0, 0, 0, 0, 0]; let N = 0;
    for (const s of S) if (s.use) { for (let i = 0; i < 3; i++) { yy[i] += s.fl[i] * s.fl[i]; ys[i] += s.fl[i]; yy[3 + i] += s.a[i] * s.a[i]; ys[3 + i] += s.a[i]; } N++; }
    const vOf = s => { const v = new Array(n).fill(0); for (let j = 0; j < n; j++) v[j] = s.v[gOf[j]]; return v; };
    const solveCost = 3 * (2 * nr * nr * nr / 3 + 4 * nr * nr);
    const groupSize = g => { let c = 0; for (let j = 0; j < n; j++) if (gOf[j] === g) c++; return c; };
    J.credit += budget;
    if (!J.ready) {                                                        // the fit's sums over the whole record with the lags now
      const cost = nS * (14 * n + 2 * nr * nr + 6 * nr) + solveCost;
      if (J.credit >= cost) {
        J.credit -= cost; J.spent += cost;
        const z = newZ(), Ar = zeros(nr, nr), br = zeros(3, nr);
        for (const s of S) { stepRegs(z, vOf(s), s.ph, s.h, st.tauG); if (s.use) addRot(Ar, br, rotRegs(z, s.W), s.a); }
        st.Ar = Ar; st.br = br; J.base = fitRot(Ar, br, yy, ys, N).fitR; J.ready = 1;
      }
    }
    for (let it = 0; it < total; it++) {
      if (!J.ready || J.k >= total) break;
      if (J.k === firstPass && !J.changed) { J.k = total; break; }         // nothing moved in the first pass: done
      const g = Math.floor((J.k % firstPass) / nt), tau = taus[J.k % nt];
      if (tau === st.tauG[g]) { J.k++; continue; }
      const kg = groupSize(g), cost = nS * (14 * n + 14 * kg + 4 * kg * nr + 12 * kg) + solveCost;
      if (J.credit < cost) break;
      J.credit -= cost; J.spent += cost; J.k++;
      // A trial changes one motor's lag, so only that motor's rows of the sums change: recompute just those.
      const inC = new Array(nr).fill(0);
      for (let j = 0; j < n; j++) if (gOf[j] === g) { inC[j] = 1; inC[n + j] = 1; }
      const Ar = zeros(nr, nr), br = zeros(3, nr);
      for (let p = 0; p < nr; p++) for (let q = 0; q < nr; q++) Ar[p][q] = inC[p] || inC[q] ? 0 : st.Ar[p][q];
      for (let i = 0; i < 3; i++) for (let p = 0; p < nr; p++) br[i][p] = inC[p] ? 0 : st.br[i][p];
      const trial = st.tauG.slice(); trial[g] = tau;
      const z = newZ(), zt = newZ();
      for (const s of S) {
        const v = vOf(s);
        stepRegs(z, v, s.ph, s.h, st.tauG); stepRegs(zt, v, s.ph, s.h, trial);
        if (s.use) {
          const phi = rotRegs(z, s.W);
          for (let j = 0; j < n; j++) if (inC[j]) { phi[j] = zt.ul[j]; phi[n + j] = zt.dq[j]; }
          for (let p = 0; p < nr; p++) {
            if (inC[p]) {
              const c = phi[p];
              for (let q = 0; q < nr; q++) if (!(inC[q] && q < p)) { const add = c * phi[q]; Ar[p][q] += add; if (q !== p) Ar[q][p] += add; }
              for (let i = 0; i < 3; i++) br[i][p] += c * s.a[i];
            }
          }
        }
      }
      const R = fitRot(Ar, br, yy, ys, N);
      if (R.fitR > J.base + 1e-4) { J.base = R.fitR; st.tauG = trial; st.Ar = Ar; st.br = br; J.changed = 1; }
    }
    const finalCost = nS * (14 * n + 2 * nr * nr + 6 * (n + 5) * (n + 5)) + 2 * nf * nf * nf / 3 + solveCost;
    if (J.ready && J.k >= total && J.credit >= finalCost) {                // done: the full fit with each motor's lag
      J.credit -= finalCost; J.spent += finalCost;
      const z = newZ(), Ar = zeros(nr, nr), br = zeros(3, nr), Af = zeros(nf, nf), bf = new Array(nf).fill(0);
      for (const s of S) {
        stepRegs(z, vOf(s), s.ph, s.h, st.tauG);
        if (s.use) { addRot(Ar, br, rotRegs(z, s.W), s.a); addForce(Af, bf, z, s.fl, Lof(s.a, s.W), s.vb); }
      }
      st.out = result(fitRot(Ar, br, yy, ys, N), fitForce(Af, bf, yy, ys, N), st.tauG, 1, J.changed);
      st.out.spent = J.spent;
      st.job = null; st.log = []; st.Ar = null; st.br = null;              // the record isn't needed any more
      return st.out;
    }
    st.out.progress = J.k / total; st.out.spent = J.spent;
    return st.out;
  }
  return st.out ? st.out : empty();
}

// ═════════════ The command module (on the ground) ═════════════

function stickInput(st, axis, analog, digital, dt) {
  // One stick of the command module (runner/ground/ground_core.c), called for each of the four, every step.
  // axis: 0 roll, 1 pitch, 2 throttle, 3 yaw. analog: a real stick's reading −1…1 (null when the axis has none);
  // digital: +1, −1 or 0 from buttons or keys (up/down, left/right…).
  // A real stick gets a small deadband (so a resting stick is exactly centred) and some expo on roll, pitch and yaw
  // (finer near the centre, full at the ends). Buttons can only be on or off, so they ease the stick toward full
  // deflection over a quarter of a second and let it back to the centre faster: a tap is a nudge, holding is full.
  const dead = 0.04, expo = axis === 2 ? 0 : 0.3;
  const rise = 4, fall = 10;                             // [full deflections per second]
  if (analog != null) {
    const m = Math.abs(analog);
    const a = m < dead ? 0 : Math.sign(analog) * Math.min(1, (m - dead) / (1 - dead));
    st.x = a;
    return (1 - expo) * a + expo * a * a * a;
  }
  const x = st.x == null ? 0 : st.x;
  const want = clamp(digital, -1, 1);
  const rate = Math.abs(want) > Math.abs(x) && want * x >= 0 ? rise : fall;
  const step = rate * dt;
  st.x = Math.abs(want - x) <= step ? want : x + Math.sign(want - x) * step;
  return st.x;
}

function groundAlerts(st, s, dt) {
  // What the command module warns the pilot about, from the telemetry that came down (and how long ago it did).
  // s: { age [s] since the last frame; lq: uplink quality [%]; downLq: telemetry link quality [%], from the
  //      transmitter module; soc: charge 0–1; vcell: cell voltage under load [V];
  //      failsafe, crashed, returning, landing, radioLost: what the drone reports (1/0) }. Unknown values are null.
  // level: 0 fine, 1 warning, 2 alarm (a command module can beep or light up); why: which (the list in the doc).
  // The most serious wins. A warning stays up 2 s after it clears, so a value hovering at a limit doesn't flicker.
  // Telemetry that comes seldom while the module still hears the drone's telemetry packets well is the link's
  // settings (a slow packet rate, a high ratio: little room down), not a lost drone: a warning, unless nothing at all
  // has come for 15 s.
  let level = 0, why = 0;
  if (s.crashed > 0.5) { level = 2; why = 8; }
  else if (s.age > 15 || (s.age > 1.5 && !(s.downLq != null && s.downLq >= 50))) { level = 2; why = 1; }
  else if (s.failsafe > 0.5) { level = 2; why = 7; }
  else if (s.radioLost > 0.5) { level = 2; why = 9; }
  else if ((s.soc != null && s.soc < 0.1) || (s.vcell != null && s.vcell < 3.3)) { level = 2; why = 4; }
  else if (s.landing > 0.5) { level = 1; why = 6; }
  else if (s.returning > 0.5) { level = 1; why = 5; }
  else if ((s.soc != null && s.soc < 0.25) || (s.vcell != null && s.vcell < 3.5)) { level = 1; why = 3; }
  else if (s.age > 1.5) { level = 1; why = 10; }
  else if (s.lq != null && s.lq < 60) { level = 1; why = 2; }
  if (st.level == null) { st.level = 0; st.why = 0; st.t = 0; }
  if (level >= st.level) { st.level = level; st.why = why; st.t = 0; }
  else { st.t = st.t + dt; if (st.t > 2) { st.level = level; st.why = why; st.t = 0; } }
  return { level: st.level, why: st.why };
}

// ═════════════ The fleet ═════════════

function fleetProgram(st, me, others, msg, dt) {
  // This drone's program for the fleet: it runs 10 times a second beside the navigation, on what the drone knows of
  // itself and what it hears from the others over its peer link (runner/fc/fleet_core.c, peer.c).
  // me: { id (its node number), p, v (where it is and how fast, from its home [m, m/s]), heading [rad], flying (1 in
  //       the air), battery [%], shared (1: its position is in the fleet's frame: GPS), engaged (1: the pilot lets
  //       this program fly it), t [s] }
  // others: the drones it hears, each { id, link (3 connected, 2 stale, 1 heard, 0 lost), lq [%], age [s] (of its
  //       values), flying, battery, p (in this drone's frame, from its home; null when either drone has no shared
  //       frame), v, heading, engaged, vals (the numbers its program publishes) }
  // msg: the messages that came since the last call, each { from, v (up to 8 numbers) }
  // Returns { publish: up to 8 numbers for the others (null: the same as before), go: { p, v, heading } where to
  // fly, from home (only while engaged; null: it holds the last place), send: [{ to (a node number; 0 every
  // drone), v }] }.
  //
  // This one flies a formation behind a leader: the drone the pilot flies (connected, flying, not engaged; the
  // lowest node number of those). Each engaged drone takes a place by its node number among the engaged ones: 2 m
  // behind the leader and 1.5 m to one side, the next to the other side, a row further back for each pair, at the
  // leader's height and facing its way. It keeps 1.5 m from every drone it can place. Joining, it tells the leader
  // (a message: 1, its place), and the leader counts the joins.
  // Published: role (0 alone, 1 leading, 2 following), place, the leader's node number (following) or the joins
  // counted (leading).
  if (st.joins == null) { st.joins = 0; st.told = 0; }
  for (let k = 0; k < msg.length; k++) if (msg[k].v.length > 0 && msg[k].v[0] === 1) st.joins = st.joins + 1;
  let lead = -1, leadId = 0, place = 0, followed = 0;
  for (let i = 0; i < others.length; i++) {
    const o = others[i];
    if (o.link >= 2 && o.flying > 0.5 && o.engaged < 0.5 && o.p != null && (lead < 0 || o.id < leadId)) { lead = i; leadId = o.id; }
    if (o.link >= 2 && o.engaged > 0.5 && o.id < me.id) place = place + 1;
    if (o.link >= 2 && o.engaged > 0.5 && o.vals.length > 2 && o.vals[0] === 2 && o.vals[2] === me.id) followed = followed + 1;
  }
  const steer = me.engaged > 0.5 && lead >= 0 && me.shared > 0.5;
  let tx = 0, ty = 0, tz = 0, h = 0, lv = [0, 0, 0], tell = 0;
  if (steer) {
    const L = others[lead], c = Math.cos(L.heading), s = Math.sin(L.heading);
    const row = Math.floor(place / 2) + 1, side = place % 2 === 0 ? 1 : -1, bx = -2 * row, by = 1.5 * row * side;
    tx = L.p[0] + c * bx - s * by; ty = L.p[1] + s * bx + c * by; tz = L.p[2]; h = L.heading; lv = L.v;
    for (let i = 0; i < others.length; i++) {                      // keep apart: pushed off any drone near it
      const o = others[i];
      if (o.p != null && o.link >= 2) {
        const dx = me.p[0] - o.p[0], dy = me.p[1] - o.p[1], d = Math.hypot(dx, dy);
        if (d < 1.5 && d > 0.01) { tx = tx + dx / d * 2 * (1.5 - d); ty = ty + dy / d * 2 * (1.5 - d); }
      }
    }
    if (st.told < 0.5) { tell = 1; st.told = 1; }
  } else if (me.engaged < 0.5) st.told = 0;
  const role = me.engaged > 0.5 && lead >= 0 ? 2 : followed > 0 ? 1 : 0;
  return { publish: [role, place, role === 2 ? leadId : st.joins], go: steer ? { p: [tx, ty, tz], v: lv, heading: h } : null,
    send: tell ? [{ to: leadId, v: [1, place] }] : null };
}

// ═════════════ Controller ═════════════

// The controller's gains are part of the design (Airframe tab → Tuning), not of the formulas: a formula reads them
// from TUNE. The step compiler fills them in as constants when it builds a board's program (rn-compile.js
// opts.consts), so a new tuning reaches the boards the way an edited formula does, and needs no new firmware.
// Attitude (per axis: roll, pitch, yaw) in angular-acceleration units: kR [1/s²] on the attitude error, kW [1/s]
// on the body rate, kI [1/s³] on the error's integral. Position (every axis) in acceleration units: kp [1/s²],
// kd [1/s], ki [1/s³].
const TUNE_DEFAULTS = Object.freeze({
  att: Object.freeze({ kR: Object.freeze([100, 100, 40]), kW: Object.freeze([16, 16, 10]), kI: Object.freeze([80, 80, 20]) }),
  pos: Object.freeze({ kp: 4, kd: 3.6, ki: 1.0 }),
});
// The gains the selected drone flies with, when a formula runs as JavaScript (the simulator points tuneSource at
// its design; tools use the defaults).
let tuneSource = () => TUNE_DEFAULTS;
const TUNE = { get att() { return (tuneSource() || TUNE_DEFAULTS).att; }, get pos() { return (tuneSource() || TUNE_DEFAULTS).pos; } };

function positionControl(ep, v, ip, m, g, lim) {
  // ep: position error, v: velocity error (hub velocity − commanded velocity), ip: integral of ep; world frame
  // lim: { accel, speed } the most horizontal acceleration [m/s²] and speed toward the target [m/s] (the supervisor
  // lowers them to fly gently)
  // A far target doesn't ask for any speed it likes: the position error asks for a velocity (kp/kd per metre),
  // capped at the speed limit sideways and at 3 m/s up, 1.5 m/s down. A drone sinking faster than that into its
  // own downwash can't brake (its rotors lose thrust in the turbulent air), so it would drop past the target.
  // Near the target it's the plain PID: kp·ep − kd·v + ki·∫ep.
  const kp = TUNE.pos.kp, kd = TUNE.pos.kd, ki = TUNE.pos.ki;   // acceleration units, so they fit any mass
  const vh = lim && lim.speed > 0 ? lim.speed : 6, k = kp / kd;
  const vx = k * ep[0], vy = k * ep[1], vxy = Math.hypot(vx, vy), sh = vxy > vh ? vh / vxy : 1;
  const want = [vx * sh, vy * sh, clamp(k * ep[2], -1.5, 3)];
  const a = [0, 1, 2].map(i => kd * (want[i] - v[i]) + ki * ip[i]);
  const ah = Math.hypot(a[0], a[1]), amax = lim && lim.accel > 0 ? lim.accel : 6;
  if (ah > amax) { a[0] *= amax / ah; a[1] *= amax / ah; }   // limit the horizontal demand
  a[2] = clamp(a[2], -6, 8);
  return scl(add(a, [0, 0, g]), m);                      // desired total force, world frame
}

function thrustAxisTarget(Fd, mode, share, leanMax) {
  // share (mixed mode): part of the sideways force the servos make, so the body only leans for the rest
  // leanMax: the most the body may lean [°] (35 unless the supervisor lowers it)
  if (mode === 'level') return [0, 0, 1];                // keep the thrust axis vertical
  const maxTilt = (leanMax > 0 ? leanMax : 35) * Math.PI / 180;
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
  const kR = TUNE.att.kR, kW = TUNE.att.kW, kI = TUNE.att.kI;   // roll, pitch, yaw (the design's: Airframe → Tuning)
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
  const made = x => [0, 1, 2, 3, 4, 5].map(k => cols.reduce((s, c, j) => s + c[k] * x[j], 0));
  // 1. The best move the limits allow (bounded weighted least squares), yaw aside: lift and tilt come first,
  //    as in most flight controllers. Otherwise, when yaw can't be had (a lone rotor, saturated motors), the
  //    cheapest way to cut the yaw error would be to cut the thrust, and the drone would drop.
  let x = bls(cols, lo, hi, wd, W.map((v, k) => k === 5 ? v * 1e-4 : v));
  //    Lift may give way to roll and pitch, but only so far: when a torque can't be had at all (a big rotor
  //    off the balance point with nothing able to cancel it), the cheapest fix would be to switch that rotor
  //    off, and the drone would drop as if it had no thrust. Keep at least LIFT_FLOOR of the lift asked for
  //    and let the attitude take the rest of the shortfall.
  const LIFT_FLOOR = 0.75;
  if (wd[2] > 0 && made(x)[2] < LIFT_FLOOR * wd[2]) {
    const t = wd.slice(); t[2] = LIFT_FLOOR * wd[2];   // the best attitude with the lift held at the floor (the optimum sits on it)
    x = bls(cols, lo, hi, t, W.map((v, k) => k === 5 ? v * 1e-4 : k === 2 ? v * 1e3 : v));
  }
  // 2. Then as much yaw as it can get without giving up any lift, roll or pitch (sideways force, which only
  //    counts where servos can make it, trades with yaw as usual).
  const kept = made(x); kept[0] = wd[0]; kept[1] = wd[1]; kept[5] = wd[5];
  x = bls(cols, lo, hi, kept, W.map((v, k) => k === 2 || k === 3 || k === 4 ? v * 1e3 : v));
  if (!pull) return x;
  // 3. Of all the ways to make that same move, the preferred one. The pulls are tiny next to the move, so
  //    they only decide where there is a real choice: a hexacopter's spare motors, a servo vs. a motor.
  return bls(cols, lo, hi, made(x), W, { q: pull.q, r: pull.r, rel: 1e-5 });
}

function voltageCompensation(u, vMeas, vRef) {
  // A motor's speed, and so its thrust, follows the voltage the ESC puts across it: throttle × pack voltage.
  // Scaling the throttle by vRef / vMeas gives the same thrust from a sagging or draining pack as from one at
  // vRef, so the controller's tables stay true through the flight.
  return clamp(u * vRef / Math.max(1, vMeas), 0, 1);
}

// ═════════════ Supervisor (the companion computer) ═════════════

function actuatorHealth(st, batch, dt, memory) {
  // What isn't doing what the flight controller's table says, from its data stream. Each sample:
  // phi[i] = motor i's column × its thrust (a 6-vector: what the table says it's doing); psi[k] = how the
  // drone would move if steering servo k were a little further round than the controller believes (its
  // rotors' column derivatives × their thrust); y = the drone's measured [acceleration; angular acceleration].
  // The gap r = y − Σ phi_i is steady while the table holds (small modelling errors, which a slow average
  // learns). When a motor loses a share κ of its effect the gap moves by −κ·phi_i; when a servo is really δ
  // further round than believed, by δ·psi_k; either way whatever the controller does about it, because it
  // still counts on its table. Each motor and each servo is tried as the one explanation for the change:
  //   κ_i = ⟨Δr, −phi_i⟩ / ‖phi_i‖²,  η_i = 1 − κ_i          δ_k = ⟨Δr, psi_k⟩ / ‖psi_k‖²
  // and only the best explanation is reported, with conf = how much of the change it explains × how
  // clearly the change stands out from the ordinary noise. memory: how long the normal gap is averaged [s].
  const w = [1, 1, 1, 0.05, 0.05, 1];                       // rows: rotation is ~20× force per unit, yaw ~1×
  const kl = Math.min(1, 0.02 / 0.035), kf = 0.02 / 0.25, ks = 0.02 / memory;
  for (const s of batch) {
    const n = s.phi.length, ns = s.psi.length;
    if (!st.eta || st.eta.length !== n || st.del.length !== ns) {         // the first sample, or the parts changed: start over
      st.eta = new Array(n).fill(1); st.conf = new Array(n).fill(0); st.del = new Array(ns).fill(0); st.sconf = new Array(ns).fill(0);
      st.lag = s.phi.slice(); st.lagS = s.psi.slice();                    // the motors' spin-up delay
      st.pp = new Array(n).fill(0); st.rp = new Array(n).fill(0); st.sp = new Array(ns).fill(0); st.sr = new Array(ns).fill(0);
      st.E = 0; st.N = 1e-2; st.k = 0; st.started = 0; st.base = [0, 0, 0, 0, 0, 0];
    } else {
      for (let i = 0; i < n; i++) for (let j = 0; j < 6; j++) st.lag[i][j] += kl * (s.phi[i][j] - st.lag[i][j]);
      for (let i = 0; i < ns; i++) for (let j = 0; j < 6; j++) st.lagS[i][j] += kl * (s.psi[i][j] - st.lagS[i][j]);
    }
    const r = [0, 0, 0, 0, 0, 0];
    for (let j = 0; j < 6; j++) { let m = 0; for (let i = 0; i < n; i++) m += st.lag[i][j]; r[j] = w[j] * (s.y[j] - m); }
    if (!st.started) { st.base = r; st.started = 1; continue; }
    const d = [0, 0, 0, 0, 0, 0]; let dd = 0;
    for (let j = 0; j < 6; j++) { d[j] = r[j] - st.base[j]; dd += d[j] * d[j]; }   // the change in the gap
    st.E += kf * (dd - st.E);
    for (let i = 0; i < n; i++) {                            // each motor, as the one explanation
      let dp = 0, q2 = 0; for (let j = 0; j < 6; j++) { const q = w[j] * st.lag[i][j]; dp += d[j] * q; q2 += q * q; }
      st.rp[i] += kf * (-dp - st.rp[i]); st.pp[i] += kf * (q2 - st.pp[i]);
    }
    for (let i = 0; i < ns; i++) {                           // each servo
      let dp = 0, q2 = 0; for (let j = 0; j < 6; j++) { const q = w[j] * st.lagS[i][j]; dp += d[j] * q; q2 += q * q; }
      st.sr[i] += kf * (dp - st.sr[i]); st.sp[i] += kf * (q2 - st.sp[i]);
    }
    // Learn the normal gap and the noise: quickly for the first 3 s, then only while nothing stands out (and
    // very slowly even then, so a lasting change that isn't a fault is eventually taken as the new normal).
    st.k++;
    const warm = st.k < 150, quiet = warm || st.E < 4 * st.N, kb = warm ? 0.05 : quiet ? ks : ks * 0.05;
    for (let j = 0; j < 6; j++) st.base[j] += kb * (r[j] - st.base[j]);
    if (quiet) st.N += (warm ? 0.05 : ks) * (Math.max(st.E, 1e-6) - st.N);
  }
  if (!st.started) return { eta: [], conf: [], del: [], sconf: [] };
  const n = st.eta.length, ns = st.del.length;
  const kap = st.rp.map((x, i) => st.pp[i] > 1e-6 ? x / st.pp[i] : 0), dl = st.sr.map((x, i) => st.sp[i] > 1e-6 ? x / st.sp[i] : 0);
  const fm = kap.map((k, i) => st.E > 1e-9 ? clamp(k * k * st.pp[i] / st.E, 0, 1) : 0), fs = dl.map((k, i) => st.E > 1e-9 ? clamp(k * k * st.sp[i] / st.E, 0, 1) : 0);
  const stand = st.k < 150 ? 0 : st.E / (st.E + 4 * st.N);
  let bm = 0, bs = -1;
  for (let i = 0; i < n; i++) if (fm[i] > fm[bm]) bm = i;
  for (let i = 0; i < ns; i++) if (bs < 0 || fs[i] > fs[bs]) bs = i;
  const servoWins = bs >= 0 && fs[bs] > (n ? fm[bm] : 0);
  for (let i = 0; i < n; i++) { const best = !servoWins && i === bm; st.eta[i] = best ? clamp(1 - kap[i], -0.5, 1.5) : 1; st.conf[i] = best ? fm[i] * stand : 0; }
  for (let i = 0; i < ns; i++) { const best = servoWins && i === bs; st.del[i] = best ? clamp(dl[i], -1.5, 1.5) : 0; st.sconf[i] = best ? fs[i] * stand : 0; }
  return { eta: st.eta.slice(), conf: st.conf.slice(), del: st.del.slice(), sconf: st.sconf.slice() };
}

function faultDecision(st, motors, servos, dt) {
  // What to do about each motor and servo, from what the supervisor sees. motors (one each): { on, eff (its
  // table scale now), cmd (thrust asked, 0–1), temp (°C, from a sensor or estimated from ESC current; null if
  // unknown), tmax, rpmRatio (ESC rpm ÷ what the command should give; null without telemetry), eta, conf }.
  // servos (each steering one): { angle (what the controller believes) [rad], delta, conf, fbErr (with
  // feedback: how far it is from its command [rad], else null) }. eta, delta and conf come from actuatorHealth.
  // st: its memory (timers, what it decided before).
  // Returns per motor { state: 0 ok, 1 degraded, 2 hot, 3 failed; on, eff, cap, why, val } and per servo
  // { stuck, angle, why, val }; why says what it saw, val the number that goes with it.
  const n = motors.length, ns = servos.length;
  if (!st.dead || st.dead.length !== n || st.off.length !== ns) {
    st.dead = new Array(n).fill(0); st.weak = new Array(n).fill(0); st.why = new Array(n).fill(0); st.val = new Array(n).fill(0); st.eff = new Array(n).fill(1);
    st.off = new Array(ns).fill(0); st.stuck = new Array(ns).fill(0); st.swhy = new Array(ns).fill(0); st.sval = new Array(ns).fill(0);
  }
  // why: 0 nothing, 1 the ESC reports it stopped, 2 it no longer moves the drone, 3 it delivers val of its table,
  // 4 running at val °C; servos: 1 it reports it isn't following its commands, 2 it's val rad from where it was told
  const servoOut = servos.map((o, k) => {
    // A servo that isn't where the controller believes: leave it out of the steering and tell the controller
    // where it really is (kept up to date while it stays out, since a limp one keeps moving).
    const fb = o.fbErr != null && Math.abs(o.fbErr) > 0.09;          // with feedback: it reports it isn't following its command
    st.off[k] = fb || (o.conf > 0.6 && Math.abs(o.delta) > 0.05) ? st.off[k] + dt : 0;
    if (!st.stuck[k] && st.off[k] > 0.5) { st.stuck[k] = 1; st.swhy[k] = fb ? 1 : 2; st.sval[k] = o.delta; }
    const angle = o.fbErr != null ? o.angle : o.angle + (o.conf > 0.4 ? o.delta : 0);
    return { stuck: st.stuck[k], angle, why: st.swhy[k], val: st.sval[k] };
  });
  const motorOut = motors.map((o, i) => {
    let on = o.on, eff = o.eff, lvl = 0;                    // 0 ok, 1 degraded, 2 hot, 3 failed
    // Failed: the ESC says it isn't spinning, or it has stopped doing anything while being asked to.
    const noSpin = o.rpmRatio != null && o.rpmRatio < 0.3, noEffect = o.conf > 0.7 && o.eta < 0.25;
    st.dead[i] = o.cmd > 0.15 && (noSpin || noEffect) ? st.dead[i] + dt : 0;
    if (!on || st.dead[i] >= (noSpin ? 0.2 : 0.5)) {
      if (on) { st.why[i] = noSpin ? 1 : 2; st.val[i] = 0; }
      on = false; lvl = 3;
    } else {
      // Degraded: its effect has settled well away from what the table says. Rescale the table to match.
      const off = o.conf > 0.3 && o.eta < 0.88;   // parts wear, they don't get stronger
      st.weak[i] = off ? st.weak[i] + dt : 0;
      if (st.weak[i] > 1.5) { eff = clamp(o.eff * o.eta, 0.15, 1.3); st.weak[i] = 0; st.why[i] = 3; st.val[i] = o.eta; }
      if (eff < 0.97) lvl = 1;
    }
    // Hot: ease it off before it's damaged. Full throttle up to 20 °C below its limit, 55% at the limit.
    let cap = 1;
    if (o.temp != null && on) {
      cap = clamp(1 - 0.45 * (o.temp - (o.tmax - 20)) / 20, 0.55, 1);
      if (cap < 0.999) { if (lvl === 0) lvl = 2; if (st.why[i] === 0 || st.why[i] === 4) { st.why[i] = 4; st.val[i] = o.temp; } }
    }
    return { state: lvl, on, eff, cap, why: st.why[i], val: st.val[i] };
  });
  return { motors: motorOut, servos: servoOut };
}

function flightPolicy(sum, prev) {
  // How to fly on what's left. sum: { margin: the most lift the working motors can make ÷ weight, cellLost: a battery cell failed, rpOk / yawOk:
  // whether roll and pitch / yaw can still be held at hover (dt: time since last call), anyFailed, hot: hottest motor as a share of its
  // limit (null if nothing is measured), soc: battery charge 0–1, vCell: V per cell, battT, battMax (°C) }.
  // Returns { mode: 'normal' | 'caution' | 'return' | 'land', lim: { speed [m/s] (0: no limit), lean [°], accel [m/s²] }, why, rpBad, vBad }.
  // why: 0 nothing, 1 a motor is running hot, 2 the battery is hot, 3 lift margin low, 4 a motor has failed, 5 lift margin very low,
  // 6 a battery cell has failed, 7 battery below 20%, 8 battery voltage low, 9 roll and pitch can no longer be held,
  // 10 not enough lift to stay up, 11 battery nearly empty, 12 battery overheating.
  // It never steps back down on its own: once it's heading home, it stays heading home.
  const rank = m => m === 'normal' ? 0 : m === 'caution' ? 1 : m === 'return' ? 2 : 3;
  let level = 0, why = 0;                                   // 0 normal, 1 caution, 2 return, 3 land
  const up = (l, w) => { if (l > level) { level = l; why = w; } };
  if (sum.hot != null && sum.hot > 0.85) up(1, 1);
  if (sum.battT != null && sum.battT > sum.battMax - 8) up(1, 2);
  if (sum.margin < 1.6) up(1, 3);
  if (sum.anyFailed) up(2, 4);
  if (sum.margin < 1.35) up(2, 5);
  if (sum.cellLost) up(2, 6);
  if (sum.soc != null && sum.soc < 0.2) up(2, 7);
  const vBad = sum.vCell != null && sum.vCell < 3.3 ? (prev ? prev.vBad : 0) + sum.dt : 0;   // sagging for a second (not a spool-up's dip)
  if (vBad > 1) up(2, 8);
  const rpBad = sum.rpOk ? 0 : (prev ? prev.rpBad : 0) + sum.dt;   // lost for a whole second, not one bad reading
  if (rpBad > 1) up(3, 9);
  if (sum.margin < 1.08) up(3, 10);
  if (sum.soc != null && sum.soc < 0.08) up(3, 11);
  if (sum.battT != null && sum.battT > sum.battMax + 5) up(3, 12);
  if (prev && rank(prev.mode) > level) { level = rank(prev.mode); why = prev.why; }
  const lim = level === 0 ? { speed: 0, lean: 35, accel: 6 } : level === 1 ? { speed: 2, lean: 20, accel: 3 } : level === 2 ? { speed: 1.5, lean: 15, accel: 2 } : { speed: 0.5, lean: 10, accel: 1.5 };
  return { mode: level === 0 ? 'normal' : level === 1 ? 'caution' : level === 2 ? 'return' : 'land', lim, why, rpBad, vBad };
}

function liftMargin(cols, lo, hi) {
  // What the drone can still do with the table as the flight controller now flies it (removed parts out,
  // ceilings on): cols, one per input (motors, then each steering servo's angle change at the thrust it
  // carries), with their limits. The hover solution says whether roll and pitch (and yaw) can be held at
  // all; the climb solution how much lift there is: margin = the most upward acceleration ÷ g, plus 1.
  const n = cols.length;
  if (!n) return { margin: 0, rpOk: false, yawOk: false };
  const hov = bls(cols, lo, hi, [0, 0, G, 0, 0, 0], [0.3, 0.3, 3, 10, 10, 1]);
  const made = [0, 0, 0, 0, 0, 0];
  for (let i = 0; i < n; i++) for (let k = 0; k < 6; k++) made[k] += cols[i][k] * hov[i];
  const rpOk = Math.abs(made[3]) < 2 && Math.abs(made[4]) < 2 && made[2] > 0.9 * G, yawOk = Math.abs(made[5]) < 1;
  const up = bls(cols, lo, hi, [0, 0, 3 * G, 0, 0, 0], [0.01, 0.01, 1, 30, 30, 0.01]);
  let az = 0;
  for (let i = 0; i < n; i++) az += cols[i][2] * up[i];
  return { margin: az / G, rpOk, yawOk };
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
  { key: 'rigidBody', group: 'plant', fn: rigidBody, title: 'Rigid-body motion (each body)',
    math: [`${V('f')} = <i>I</i>${V('a')} + ${V('v')} ×* <i>I</i>${V('v')} &nbsp;(spatial vectors [angular; linear], in the body's axes)`, `whole airframe: <i>M</i>(${V('q')}) ${V('q̈')} + ${V('c')}(${V('q')}, ${V('q̇')}, forces) = [0; ${V('τ')}<sub>servo</sub>], &nbsp;${V('q')} = frame pose + every joint angle`],
    doc: 'Newton–Euler for one rigid body. The frame is one body and every servo joint adds another (the servo output and everything rigidly on it). The simulator chains this formula through the joints (recursive Newton–Euler, Featherstone) to get the mass matrix and everything else, then solves for the frame\'s acceleration and every joint\'s angular acceleration each 0.5 ms step. That is why a swinging arm pushes the frame the other way, a load drags its servo, and a sensor on an arm feels the arm\'s own motion.',
    args: [['I', '{ m, c: centre of mass, Ic: inertia about it }, body axes'], ['a', 'spatial acceleration [α; a]'], ['v', 'spatial velocity [ω; v]']],
    returns: 'spatial force [torque; force] the body needs', shape: 'vec6',
    sample: () => [{ m: 1, c: [0, 0, 0], Ic: [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.02] }, [0, 0, 0, 0, 0, -9.81], [0.1, 0, 0, 0, 0, 0]] },
  { key: 'gravity', group: 'plant', fn: gravity, title: 'Weight',
    math: [`${V('F')}<sub>g</sub> = −<i>m g</i> ${V('ẑ')}`],
    doc: 'Applied at the center of gravity of the frame and of each cable payload.',
    args: [['m', 'mass [kg]'], ['g', 'gravitational acceleration, 9.81 m/s²']], returns: 'force, world [N]',
    shape: 3, sample: () => [1, 9.81] },
  { key: 'rotorWrench', group: 'plant', fn: rotorWrench, title: 'Rotor force and torque',
    math: [`${V('F')}<sub>i</sub> = <i>T</i><sub>i</sub> ${V('d')}<sub>i</sub>`, `${V('τ')}<sub>i</sub> = ${V('r')}<sub>i</sub> × ${V('F')}<sub>i</sub> − <i>s</i><sub>i</sub> κ<sub>i</sub> <i>T</i><sub>i</sub> ${V('d')}<sub>i</sub>`],
    doc: 'The contribution of one rotor. s is +1 when the prop turns counter-clockwise seen from the side the thrust points to, −1 for clockwise. For a puller that is the spin on its card (seen looking at the prop); a pusher\'s prop is pitched the other way, so its s is the reverse of its card. κ is the drag-torque to thrust ratio.',
    used: 'Also builds the controller\'s effectiveness matrix and the flight envelope, evaluated at T = 1 N, so those assume the law is linear in T.',
    args: [['d', 'rotor axis (unit), body'], ['r', 'rotor position from the CoG, body [m]'], ['T', 'delivered thrust [N]'], ['spin', '+1 CCW, −1 CW, about the thrust axis'], ['kappa', 'drag torque ratio κ [m]']],
    returns: '{ F: force, body; tau: torque, body }',
    shape: { F: 3, tau: 3 }, sample: () => [[0, 0, 1], [0.2, 0.1, 0], 3, 1, 0.016] },
  { key: 'jointRotation', group: 'plant', fn: jointRotation, title: 'Servo joint',
    math: [`<i>R</i>(θ) = cos θ <i>I</i> + sin θ [${V('a')}]<sub>×</sub> + (1 − cos θ) ${V('a')}${V('a')}<sup>T</sup>`, `${V('p')}<sub>now</sub> = ${V('q')} + <i>R</i>(θ)(${V('p')}<sub>rest</sub> − ${V('q')}), &nbsp;applied for each joint above a part, nearest first`],
    doc: 'A servo joint turns everything attached to it about its hinge axis a through its pivot q: motors, masses, cable attachments, sensors and further joints. Parts store where they are at rest (all angles zero). Positive θ turns by the right-hand rule about a. Because each rotor\'s effect is linear in cos θ and sin θ for every joint above it, the controller can learn it as a few fixed columns (see Effectiveness identification).',
    args: [['axis', 'hinge axis at rest, body frame'], ['theta', 'joint angle [rad]']], returns: '3×3 rotation, row by row',
    shape: 'mat3', sample: () => [[1, 0, 0], 0.3] },
  { key: 'motorDynamics', group: 'plant', fn: motorDynamics, title: 'Motor, ESC and prop',
    math: [`<i>i</i> = (<i>u V</i> − <i>K</i><sub>e</sub>Ω)/<i>R</i>, &nbsp;τ = <i>K</i><sub>e</sub><i>i</i>, &nbsp;<i>J</i>Ω̇ = τ − <i>k</i><sub>Q</sub>Ω²`, `<i>T</i> = <i>k</i><sub>T</sub>Ω², &nbsp;prop drag torque <i>k</i><sub>Q</sub>Ω², &nbsp;frame feels −τ about the motor axis and −ω × <i>J</i>Ω (gyroscopic)`],
    doc: 'Throttle sets voltage; back-EMF and winding resistance set winding current and torque. Rotor inertia determines spin-up; a semi-implicit damping step bounds stiff motor transients. Generic profiles infer electrical traits from the thrust rating; fixed profiles use KV, resistance, inertia and current limit, with brush voltage drop/friction for brushed motors. Static prop tables supply thrust and torque versus RPM. Density, bounded tip losses and the previous step’s inflow load affect the prop. Battery draw includes duty-weighted winding current, driver losses, avionics and servo power. Static tables do not validate forward flight.',
    args: [['Omega', 'prop speed [rad/s]'], ['u', 'throttle 0–1'], ['V', 'battery voltage [V]'], ['p', '{ Ke, R, J, kT, kQ, iMax; optional table, rho, rhoRef, qFactor, tFactor, brushDrop, friction }'], ['dt', 'time step [s]']],
    returns: '{ Omega, i: current [A], tau: motor torque [N·m], T: still-air thrust [N] }', shape: { Omega: 1, i: 1, tau: 1, T: 1 },
    sample: () => [1300, 0.5, 15.4, { Ke: 0.0064, R: 0.2, J: 7e-6, kT: 1.4e-6, kQ: 2.2e-8, iMax: 30 }, 0.0005] },
  { key: 'servoTorque', group: 'plant', fn: servoTorque, title: 'Servo',
    math: [`τ = τ<sub>stall</sub> (sat(<i>e</i>/<i>b</i>) − θ̇/ω<sub>no-load</sub>), &nbsp;<i>e</i> = command (delayed, plus trim error) − angle`, `plus the gearbox's reflected inertia on the joint and hard stops a little past the range`],
    doc: 'A hobby servo is a geared DC motor with a position loop, so it has a torque–speed line: full stall torque when stopped, none at its no-load speed. The joint then moves by the multibody dynamics: a light arm snaps to its target, a heavy one lags and overshoots, and thrust or weight on the arm can hold it off its target. The command reaches it after the PWM delay.',
    args: [['err', 'target − angle [rad]'], ['rate', 'angle rate [rad/s]'], ['p', '{ stall [N·m], speed: no-load [rad/s], band [rad] }']], returns: 'torque on the joint [N·m]',
    shape: 'n', sample: () => [0.1, 0, { stall: 0.8, speed: 5, band: 0.05 }] },
  { key: 'bodyDrag', group: 'plant', fn: bodyDrag, title: 'Aerodynamic drag',
    math: [`${V('F')}<sub>d</sub> = <i>c</i><sub>d</sub>(${V('v')}<sub>wind</sub> − ${V('v')})`, `${V('τ')}<sub>d</sub> = −<i>c</i><sub>ω</sub> ${V('ω')}`],
    doc: 'Linear drag on the airframe and a little rotational damping.',
    args: [['v', 'velocity, world [m/s]'], ['wind', 'wind velocity, world [m/s]'], ['w', 'angular velocity, body [rad/s]'], ['density', 'air density [kg/m³], default 1.225']], returns: '{ F: force, world; tau: torque, body }',
    shape: { F: 3, tau: 3 }, sample: () => [[1, 0, 0], [0, 0, 0], [0, 0, 0.5]] },
  { key: 'wingAero', group: 'plant', fn: wingAero, title: 'Wing lift and drag',
    math: [`α = atan2(<i>u</i><sub>z</sub>, −<i>u</i><sub>x</sub>), &nbsp;<i>C</i><sub>L</sub> = 2π α / (1 + 2/AR) &nbsp;below the stall (15°), then a flat plate's 2 sin α cos α`, `<i>C</i><sub>D</sub> = 0.02 + <i>C</i><sub>L</sub>²/(0.8 π AR) &nbsp;(then a plate's 1.9 sin²α), &nbsp;<i>L</i>, <i>D</i> = ½ρ<i>V</i>²<i>S C</i><sub>L</sub>, <i>C</i><sub>D</sub>`],
    doc: 'Every part shaped as a wing (the frame, a rigid mass) in the air it meets: the wind, its own motion and the rotors\' wash. Lift across the airflow, drag along it, acting at the quarter chord (the middle once stalled). Physics only: the flight computers don\'t know about wings; they meet them as an oddly shaped body.',
    args: [['u', 'air past the wing, its axes (x leading edge, y span, z up) [m/s]'], ['chord', 'chord [m]'], ['span', 'span [m]'], ['density', 'air density [kg/m³]'], ['polar', 'optional [angle degrees, Cl, Cd] rows; blend to generic stall outside coverage']], returns: '{ F: force, wing axes [N]; xcp: centre of pressure ahead of the middle [m] }',
    shape: { F: 3, xcp: 1 }, sample: () => [[-8, 0, 0.7], 0.2, 0.8] },
  { key: 'bluffDrag', group: 'plant', fn: bluffDrag, title: 'Drag on blunt parts',
    math: [`<i>F</i><sub><i>i</i></sub> = ½ρ <i>C</i><sub>d</sub> <i>A</i><sub><i>i</i></sub> |${V('u')}| <i>u</i><sub><i>i</i></sub>, &nbsp;<i>C</i><sub>d</sub> = 1.05`],
    doc: 'Every part shaped as a prism (rigid masses, and the frame unless it is a wing): drag on the face it shows the air, by direction. The frame also has its general drag (Aerodynamic drag).',
    args: [['u', 'air past the part, its axes [m/s]'], ['areas', 'frontal area seen along x, y, z [m²]'], ['density', 'air density [kg/m³]'], ['coefficient', 'drag coefficient, default 1.05']], returns: 'force, the part\'s axes [N]',
    shape: 3, sample: () => [[-5, 1, 0], [0.004, 0.008, 0.01]] },
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
    math: [`<i>N</i> = max(0, <i>k</i><sub>g</sub><i>h</i> − <i>c</i><sub>g</sub><i>v</i><sub>z</sub>), &nbsp;${V('F')}<sub>t</sub> = −min(8, 0.6<i>N</i>/|${V('v')}<sub>t</sub>|) ${V('v')}<sub>t</sub>`],
    doc: 'A penalty spring at each contact point (hub corners, motors, arms, masses, rods, sensors) that is inside the ground or a building. For a building the same law is turned to face its surface: z is the way out, x and y lie along the surface.',
    args: [['depth', 'h, depth below ground [m]'], ['v', 'point velocity, world [m/s]']], returns: 'force, world [N]',
    shape: 3, sample: () => [0.01, [0.1, 0, -0.5]] },

  { key: 'wakeVelocity', group: 'plant', fn: wakeVelocity, title: 'Rotor wakes',
    math: [`<i>v</i><sub>h</sub> = √(<i>T</i> / 2ρ<i>A</i>), &nbsp;${V('w')}(<i>s</i>) = −${V('d')} <i>v</i><sub>h</sub>(1 + <i>s</i>/√(<i>s</i>² + <i>R</i>²)) · e<sup>−<i>s</i>/12<i>R</i></sup> &nbsp;inside the wake`, `wake radius <i>R</i>(0.71 + 0.29 e<sup>−<i>s</i>/<i>R</i></sup>), &nbsp;<i>s</i> = distance downstream, centre blown sideways by ${V('v')}<sub>cross</sub>·<i>s</i>/1.5<i>v</i><sub>h</sub>`],
    doc: 'Each rotor blows a column of air along −d that speeds up to twice its induced velocity and contracts. Another rotor inside that column loses thrust; parts and payloads inside it get pushed. Physics only: the controller never uses this.',
    args: [['point', 'where to evaluate, body frame [m]'], ['rotors', '[{ p, d, T, R, va }] disc center, axis, thrust, prop radius, oncoming air'], ['density', 'air density [kg/m³]']],
    returns: 'induced air velocity, body frame [m/s]', shape: 3, sample: () => [[0, 0, -0.1], [{ p: [0, 0, 0], d: [0, 0, 1], T: 3, R: 0.08 }]] },
  { key: 'rotorAero', group: 'plant', fn: rotorAero, title: 'Rotor aerodynamics',
    math: [`<i>v</i><sub>i</sub> = <i>v</i><sub>h</sub>² / √(<i>V</i><sub>edge</sub>² + (<i>v</i><sub>ax</sub> + <i>v</i><sub>i</sub>)²) &nbsp;(Glauert)`, `<i>T</i><sub>eff</sub> = <i>T</i> · sat(1 − 0.5 (<i>v</i><sub>ax</sub> + <i>v</i><sub>i</sub> − <i>v</i><sub>h</sub>)/<i>v</i><sub>h</sub>) · 1/(1 − (<i>R</i>/4<i>h</i>)²) · (1 − 0.3 e<sup>−((<i>x</i> − 1.2)/0.45)²</sup>), &nbsp;<i>x</i> = descent rate/<i>v</i><sub>h</sub> (vortex ring state)`, `${V('H')} = <i>c</i><sub>H</sub> <i>T</i> ${V('u')}<sub>in-plane</sub> &nbsp;(rotor drag)`],
    doc: 'Air coming down through the disc (from climbing or from another rotor\'s wake) costs thrust at the same command. Air crossing the disc edgewise, in forward flight, lowers the induced velocity and adds a little thrust (translational lift). The ground adds thrust within about a rotor diameter. Descending straight into its own wake at around its induced velocity puts a rotor in vortex ring state and costs it up to 30% of its thrust. Air moving across the disc tilts it back and makes rotor drag, which is most of a multirotor\'s drag.',
    args: [['T', 'still-air thrust for this command [N]'], ['R', 'prop radius [m]'], ['vAxial', 'inflow from above along the axis [m/s]'], ['vInPlane', 'air velocity across the disc, body frame [m/s]'], ['h', 'height above ground [m]'], ['density', 'air density [kg/m³]']],
    returns: '{ T: effective thrust [N]; H: rotor drag force [N]; Qfactor: bounded torque correction for the next motor step }', shape: { T: 1, H: 3, Qfactor: 1 }, sample: () => [3, 0.08, 0.5, [1, 0, 0], 2] },
  { key: 'wakeLoad', group: 'plant', fn: wakeLoad, title: 'Downwash on parts',
    math: [`${V('F')} = ½ ρ <i>C</i><sub>d</sub> <i>A</i> |${V('w')}| ${V('w')}`],
    doc: 'Rotor wash hitting the hub, rigid masses and cable payloads pushes them along the wake.',
    args: [['w', 'wake air velocity at the part [m/s]'], ['area', 'frontal area of the part [m²]']], returns: 'force [N]', shape: 3, sample: () => [[0, 0, -5], 0.01] },
  { key: 'batteryModel', group: 'plant', fn: batteryModel, title: 'Battery',
    math: [`<i>V</i> = <i>n</i><sub>cells</sub> <i>V</i><sub>rest</sub>(SoC) − <i>R</i><sub>int</sub>(<i>T</i>, SoC) <i>I</i>, &nbsp;d(SoC)/d<i>t</i> = −<i>I</i> / capacity`, `<i>V</i><sub>rest</sub>: 4.2 V full, 3.83 V at half, 3.6 V at 10%, 3.2 V empty, collapsing past it`],
    doc: 'A LiPo, sized by the Battery settings (4 cells, 1.3 Ah and 60 mΩ by default). It drains with the current all the motors draw and sags under load, so the same throttle gives less thrust as the flight goes on and during hard manoeuvres, unless the flight controller measures the voltage and corrects for it. A warm pack sags less (its resistance falls about 1.5% per °C), overheating costs it capacity and adds resistance for good, and a failed cell takes 3.5–4.2 V away. Near empty its voltage falls away and it sags more, so the thrust it can give drops until the drone can no longer hover; over-discharged, it collapses (and the ESCs\' low-voltage cutoff, set in the Battery settings, stops the motors). Reset restores the pack at its take-off charge.',
    args: [['st', 'battery state (soc)'], ['current', 'total draw [A]'], ['dt', 'time step [s]'], ['p', '{ cells, capacity [C], rInt [Ω], cut }']], returns: 'terminal voltage [V]', shape: 'n', sample: () => [{}, 12, 0.0005, { cells: 4, capacity: 4680, rInt: 0.06, cut: false }] },
  { key: 'thermalModel', group: 'plant', fn: thermalModel, title: 'Heating and cooling',
    math: [`<i>C</i> d<i>T</i>/d<i>t</i> = <i>P</i> − <i>G</i> (<i>T</i> − <i>T</i><sub>air</sub>)`, `motor: <i>P</i> = <i>i</i>²<i>R</i>(<i>T</i>), &nbsp;<i>G</i> = <i>G</i><sub>full</sub>(0.3 + 0.7 Ω/Ω<sub>max</sub>), &nbsp;<i>R</i> +0.39%/K, magnet −0.12%/K`, `battery: <i>P</i> = <i>I</i>²<i>R</i><sub>int</sub>`],
    doc: 'Motor and battery I²R losses heat their lumped thermal masses. Motor cooling varies with speed; explicit heat capacity and conductance override generic estimates. Generic conductance uses a fixed reference temperature rise, independent of the failure threshold. Battery heat capacity uses the actual battery-part mass when available. Hot windings increase resistance and weaken the motor magnet; configured thresholds drive irreversible damage and failure. ESC and servo temperatures are not modeled.',
    args: [['T', 'temperature now [°C]'], ['P', 'heat made [W]'], ['G', 'cooling [W/K]'], ['C', 'heat capacity [J/K]'], ['Tamb', 'air temperature [°C]'], ['dt', 'time step [s]']], returns: 'temperature after dt [°C]', shape: 'n', sample: () => [40, 10, 0.2, 30, 25, 0.0005] },
  { key: 'imuModel', group: 'sensor', fn: imuModel, title: 'IMU (gyro + accelerometer)',
    math: [`${V('ω̃')} = sat(${V('ω')}<sub>s</sub> + ${V('ω')}<sub>vib</sub> + ${V('b')}<sub>g</sub> + ${V('n')}<sub>g</sub>), &nbsp;${V('ḃ')}<sub>g</sub> = random walk`, `${V('f̃')} = sat(<i>S</i><sub>a</sub>${V('f')} + ${V('a')}<sub>vib</sub> + ${V('b')}<sub>a</sub> + ${V('n')}<sub>a</sub>), &nbsp;${V('f')} = the acceleration of the body the IMU is on, at the IMU, minus gravity`, `<i>S</i> = scale errors on the diagonal, small axis misalignments off it`],
    doc: 'r is the IMU\'s offset from the center of gravity, so an off-center accelerometer also feels rotation. Vibration is a sum of sinusoids at each motor\'s rotation frequency, stronger near busy motors; a slow IMU rate aliases it into low frequencies.',
    args: [['w', 'true angular rate, sensor frame [rad/s]'], ['f', 'true specific force, sensor frame [m/s²]'], ['vib', '{ a, w } vibration at the sensor'], ['p', 'gyroNoise, gyroBias, gyroDrift, gyroRange, accNoise, accBias, accRange, scaleErr, misalign'], ['st', 'this sensor\'s state'], ['dt', 'sample period [s]']],
    returns: '{ gyro, accel }, sensor frame', shape: { gyro: 3, accel: 3 },
    sample: () => [[0.1, 0, 0], [0, 0, 9.81], { a: [0, 0, 0], w: [0, 0, 0] }, { gyroNoise: 0.002, gyroBias: 0.01, gyroDrift: 0.0003, gyroRange: 35, accNoise: 0.05, accBias: 0.05, accRange: 157, scaleErr: 0.005, misalign: 0.0035 }, {}, 0.001] },
  { key: 'magModel', group: 'sensor', fn: magModel, title: 'Compass',
    math: [`${V('m̃')} = <i>S</i><sub>soft iron</sub><i>R</i><sub>s</sub><sup>T</sup><i>R</i><sup>T</sup>${V('m')}<sub>earth</sub> + ${V('b')}<sub>hard iron</sub> + ${V('m')}<sub>motors</sub> + ${V('n')}`],
    doc: 'Earth\'s field has strength 1 and points north (+X) and 60° down. Motor currents add a field that grows with throttle and falls off quickly with distance, so where you mount the compass matters.',
    args: [['b', 'Earth field, sensor frame'], ['interference', 'motor field at the sensor, sensor frame'], ['p', 'noise, hardIron, softIron'], ['st', 'this sensor\'s state']],
    returns: 'field reading, sensor frame', shape: 3, sample: () => [[0.5, 0, -0.866], [0, 0, 0], { noise: 0.01, hardIron: 0.05, softIron: 0.03 }, {}] },
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
    math: [`Δ[${V('f̃')} − ${V('ω̇')}×${V('r')} − ${V('ω')}×(${V('ω')}×${V('r')}); ${V('ω̃̇')}] ≈ <i>B̂</i> Δ${V('u')} + [0; <i>B̂</i><sub>2</sub>] Δ<i>ẋ</i>, &nbsp;both sides band-passed 0.3–12 Hz, <i>x</i> = √thrust (rotor speed)`, `<i>K</i> = <i>P</i>${V('x')} / (λ + ${V('x')}<sup>T</sup><i>P</i>${V('x')}), &nbsp;<i>B̂</i> += ${V('e')}<i>K</i><sup>T</sup>, &nbsp;<i>P</i> = (<i>P</i> − <i>K</i>${V('x')}<sup>T</sup><i>P</i>)/λ`],
    doc: 'Recursive least squares with forgetting: short memory during calibration, 30 s during optional ordinary-flight adaptation. A motor on servo joints is several inputs: its thrust times each product of (1, cos θ, sin θ) over the joints it sits on, so its effect at any joint angles is a fixed sum of learned columns (3 for one joint, 9 for two). It estimates, straight from the accelerometer and gyro, how much acceleration and angular acceleration each actuator input produces. That covers mass, inertia, prop thrust, rotor wakes and battery sag without being told any of them. It learns from changes, so steady offsets like drag can\'t leak in, and it removes the accelerometer\'s lever-arm swing using the IMU position the controller knows. The learning task validates frozen candidates before changing the accepted flight model; optional adaptation adds no test pulses.',
    args: [['st', 'identification state'], ['u', 'inputs: thrust fractions, times (1, cos θ, sin θ) for each joint a motor sits on'], ['f', 'accelerometer, body [m/s²]'], ['w', 'gyro, body [rad/s]'], ['r', 'IMU position from the hub [m]'], ['dt', 'control period [s]'], ['init', 'starting guess, 6 rows'], ['memory', 'forgetting time [s]: short while calibrating, long in flight'], ['lags', 'each input\'s motor lag [s], learned by the actuator tests'], ['mot', '{ v, phi, m }: per input, its motor\'s thrust command, basis factor and motor number']],
    returns: '{ B: 6 rows × inputs; B2: 3 rows × inputs (rotation from rotors speeding up) }', shape: { B: 'rows' },
    sample: () => [{}, [0.5, 0.5], [0, 0, 9.81], [0, 0, 0], [0, 0, 0.01], 0.001, [[0, 0], [0, 0], [10, 10], [100, -100], [0, 0], [1, -1]], 4, [0.03, 0.03], { v: [0.5, 0.5], phi: [1, 1], m: [0, 1] }] },
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
    math: [`${V('f̃')} = <i>B</i><sub>f</sub>${V('u')}<sub>τ</sub> + ([${V('ω̇')}]<sub>×</sub> + [${V('ω')}]<sub>×</sub>²)${V('r')} − <i>d</i>${V('v')}<sub>b</sub> + ${V('c')}<sub>f</sub> &nbsp;(free fall: no gravity in the accelerometer)`, `${V('ω̇')} = <i>B</i><sub>α</sub>${V('u')}<sub>τ</sub> + <i>B</i><sub>2</sub> d<i>x</i>/d<i>t</i> + <i>K</i>(ω<sub>y</sub>ω<sub>z</sub>, ω<sub>z</sub>ω<sub>x</sub>, ω<sub>x</sub>ω<sub>y</sub>) + ${V('c')}<sub>α</sub>, &nbsp;<i>u</i><sub>τ</sub> = <i>u</i> / (1 + τ<i>s</i>)`, `least squares for each τ in {10 … 130 ms}; the best fit gives <i>B</i>, ${V('r')} and the motor lag τ`],
    doc: 'Used by the throw start (the learning task). The drone is thrown with its motors off and a random spin, and it pulses each motor briefly while it falls. Because it is in free fall, the accelerometer feels only the rotors and the IMU\'s swing around the center of gravity, so a plain least-squares fit on less than a second of data gives the effectiveness matrix, where the IMU sits relative to the balance point, the gyroscopic coupling, the spin-up reaction and the motor lag, all without any description of the airframe. It pulses over the top of the throw, where the air through the props is calmest. A fixed cost per step while falling, an instant fit to catch itself on, then each motor\'s own lag worked out in the background. Up to 12 inputs (a quadcopter with a tilting motor on each arm). After Blaha, Smeur and Remes (TU Delft, 2024).',
    args: [['st', 'identification state'], ['u', 'inputs: thrust fractions, times (1, cos θ, sin θ) for each joint a motor sits on'], ['f', 'accelerometer, body [m/s²]'], ['w', 'gyro, body [rad/s]'], ['vb', 'estimated velocity, body [m/s] (for air drag)'], ['dt', 'sample period [s]'], ['solve', '\'fall\' while falling, \'catch\' to fit at once, \'refine\' for the per-motor lags afterwards'], ['mot', '{ v, phi, m, coll }: per input, its motor\'s thrust command, its basis factor, the motor\'s number and whether it is a collective-pitch rotor (1)'], ['budget', 'refine only: operations it may spend this call']],
    returns: '{ B: 6 rows × inputs; B2: 3 rows × inputs, rotation from each rotor spinning up; r: IMU offset from the CoG [m]; taus: each motor\'s lag [s], tau: their mean; fitF, fitR: share of force and rotation explained; refined, progress }', shape: { B: 'rows', r: 3, tau: 1, fitF: 1, fitR: 1 },
    sample: () => [{}, [0.5, 0.2], [0.1, 0, 3], [1, 0.5, 0], [0, 0, 2], 0.004, 'catch', { v: [0.5, 0.2], phi: [1, 1], m: [0, 1], coll: [0, 0] }, 0] },

  { key: 'stickInput', group: 'ground', fn: stickInput, title: 'Stick shaping',
    math: [`stick: ${V('x')} = (1 − <i>e</i>)<i>a</i> + <i>e a</i>³, &nbsp;<i>a</i> = the reading past a 4% deadband, <i>e</i> = 0.3 (0 for the throttle)`, `buttons: ${V('x')} moves toward ±1 at 4 per second, back to 0 at 10 per second`],
    doc: 'Runs on the command module, the pilot\'s side of the radio (an ESP32 with buttons, a Pi, or a Mac), for each of the four sticks every step. A real stick gets a deadband and expo; buttons or keys, which are only on or off, ease the stick in so a tap nudges and holding gives full. What it returns goes up the radio as the channel, and the drone decides what a stick position makes it do.',
    args: [['st', 'the stick\'s memory'], ['axis', '0 roll, 1 pitch, 2 throttle, 3 yaw'], ['analog', 'a real stick −1…1, or null'], ['digital', '+1, −1 or 0 from buttons'], ['dt', 'step [s]']], returns: 'the channel −1…1',
    shape: 'n', sample: () => [{}, 0, null, 1, 0.004] },
  { key: 'groundAlerts', group: 'ground', fn: groundAlerts, title: 'Pilot alerts',
    math: [`alarm: crashed, no telemetry for 1.5 s (15 s while the telemetry link quality is ≥ 50%), failsafe, the drone hears no radio, battery &lt; 10% or &lt; 3.3 V/cell`, `warning: landing, returning home, battery &lt; 25% or &lt; 3.5 V/cell, telemetry slow (the link has little room), uplink quality &lt; 60%`],
    doc: 'Runs on the command module, on the telemetry that came down the radio: what to warn the pilot about. The most serious thing wins, and a warning stays up 2 s after it clears. Reasons: 1 no telemetry, 2 weak link, 3 battery low, 4 battery very low, 5 returning home, 6 landing, 7 failsafe, 8 crashed, 9 the drone hears no radio, 10 telemetry slow (frames seldom, but the module hears the drone\'s telemetry packets well: the packet rate and ratio leave little room). A command module with a buzzer or an LED beeps or lights for it; the Ground station shows it.',
    args: [['st', 'its memory'], ['s', '{ age, lq, downLq, soc, vcell, failsafe, crashed, returning, landing, radioLost }'], ['dt', 'step [s]']], returns: '{ level: 0 fine, 1 warning, 2 alarm; why }',
    shape: 'obj', sample: () => [{}, { age: 0.1, lq: 100, downLq: 100, soc: 0.8, vcell: 3.9, failsafe: 0, crashed: 0, returning: 0, landing: 0, radioLost: 0 }, 0.1] },
  { key: 'fleetProgram', group: 'ctrl', fn: fleetProgram, title: 'Fleet program',
    math: ['the leader: connected, flying, not engaged (the lowest node number)', `place <i>k</i> (by node number among the engaged): behind it 2(⌊<i>k</i>/2⌋ + 1) m, to the side ±1.5(⌊<i>k</i>/2⌋ + 1) m, at its height`, 'keep apart: pushed 2(1.5 − <i>d</i>) m off any drone nearer than 1.5 m'],
    doc: 'Each drone\'s own program for the fleet, beside the navigation, 10 times a second. It sees this drone and the others it hears over the peer link (ESP-NOW between drones: their state, where they are, what their programs publish) and their messages; it can publish numbers, send messages, and, while the pilot has engaged it (the Ground tab\'s Fleet program button, the radio\'s FLEET command), say where the drone flies. The navigation still flies it there, within the same box and limits as the pilot\'s target, and the sticks, hold, home, a go-to or the link lost take it back. Positions are shared only between drones with GPS (the fleet\'s frame); without it a drone can still talk, but not place the others. This default flies a formation behind the drone the pilot flies.',
    args: [['st', 'its memory'], ['me', '{ id, p, v, heading, flying, battery, shared, engaged, t }'], ['others', 'up to 8: { id, link, lq, age, flying, battery, p (or null), v, heading, engaged, vals }'], ['msg', 'up to 4: { from, v }'], ['dt', 'since the last call [s]']], returns: '{ publish (up to 8 numbers), go: { p, v, heading } or null, send: [{ to, v }] or null }',
    shape: 'obj', sample: () => [{}, { id: 3, p: [0, 0, 1.5], v: [0, 0, 0], heading: 0, flying: 1, battery: 80, shared: 1, engaged: 1, t: 10 },
      [{ id: 2, link: 3, lq: 100, age: 0.1, flying: 1, battery: 90, p: [4, 1, 2], v: [0.5, 0, 0], heading: 0.3, engaged: 0, vals: [1, 0, 1] },
       { id: 5, link: 3, lq: 96, age: 0.1, flying: 1, battery: 75, p: [1, 0.5, 1.5], v: [0, 0, 0], heading: 0, engaged: 1, vals: [2, 1, 2] }],
      [{ from: 5, v: [1, 1] }], 0.1] },
  { key: 'positionControl', group: 'ctrl', fn: positionControl, title: 'Position control',
    math: [`${V('a')}<sub>d</sub> = <i>K</i><sub>d</sub>(sat(<i>K</i><sub>p</sub>/<i>K</i><sub>d</sub> ${V('e')}<sub>p</sub>) − (${V('v')} − ${V('v')}<sub>cmd</sub>)) + <i>K</i><sub>i</sub>∫${V('e')}<sub>p</sub> d<i>t</i>`, `sat: sideways ≤ the speed limit, up ≤ 3 m/s, down ≤ 1.5 m/s`, `${V('F')}<sub>d</sub> = <i>m</i>(${V('a')}<sub>d</sub> + <i>g</i>${V('ẑ')})`],
    doc: 'PID on the frame hub\'s position. On the learned model the controller doesn\'t know its mass, so m is 1 and the result is a desired specific force. When you fly with the keys or pads, the target moves at a commanded velocity and v arrives as the velocity error, so the damping term also feeds that velocity forward. The integral is kept by the simulator and clamped to ±2 m·s sideways and ±5 m·s vertically, so it can trim out an unknown hover throttle. m is the mass the controller believes in. The gains kp, kd, ki are the design\'s (Airframe → Tuning), read from TUNE.pos.',
    args: [['ep', 'position error, world [m]'], ['v', 'hub velocity − commanded velocity, world [m/s]'], ['ip', '∫ ep dt [m·s]'], ['m', 'modeled mass [kg]'], ['g', '9.81 m/s²'], ['lim', '{ accel, speed }: most horizontal acceleration [m/s²] and speed [m/s], from the supervisor']], returns: 'desired total force, world [N]',
    shape: 3, sample: () => [[0.1, 0, 0.1], [0, 0, 0], [0, 0, 0], 1, 9.81, { accel: 6 }] },
  { key: 'thrustAxisTarget', group: 'ctrl', fn: thrustAxisTarget, title: 'Thrust-axis target',
    math: [`tilt body: ${V('n')}<sub>d</sub> = ${V('F')}<sub>d</sub> / ‖${V('F')}<sub>d</sub>‖, &nbsp;at most 35° from vertical`, `mixed: ${V('n')}<sub>d</sub> ∝ ((1 − <i>s</i>)<i>F</i><sub>x</sub>, (1 − <i>s</i>)<i>F</i><sub>y</sub>, <i>F</i><sub>z</sub>), &nbsp;<i>s</i> = the servos' share of the sideways force`, `stay level: ${V('n')}<sub>d</sub> = ${V('ẑ')}`],
    doc: 'Where the craft\'s nominal thrust axis should point. The desired attitude is built from this and the target heading. In mixed steering the body leans only for the part of the sideways force the servos aren\'t making; the simulator lowers s automatically when the servos can\'t deliver their share.',
    args: [['Fd', 'desired force, world [N]'], ['mode', '"tilt", "mixed" or "level"'], ['share', 'servos\' share of the sideways force (mixed)'], ['leanMax', 'most the body may lean [°], from the supervisor']], returns: 'desired thrust axis, world (normalized afterwards)',
    shape: 3, sample: () => [[1, 0, 9.81], 'mixed', 0.5, 35] },
  { key: 'attitudeError', group: 'ctrl', fn: attitudeError, title: 'Attitude error',
    math: [`${V('e')}<sub>R</sub> = ½ (<i>R</i><sub>d</sub><sup>T</sup><i>R</i> − <i>R</i><sup>T</sup><i>R</i><sub>d</sub>)<sup>∨</sup>`],
    doc: 'Geometric attitude error on SO(3). The simulator integrates it for the attitude integral, clamped to ±0.5 rad·s.',
    args: [['R', 'current attitude matrix'], ['Rd', 'desired attitude matrix']], returns: 'attitude error, body [rad]',
    shape: 3, sample: () => [qmat([1, 0, 0, 0]), qmat(qnorm([1, 0.05, 0, 0]))] },
  { key: 'attitudeControl', group: 'ctrl', fn: attitudeControl, title: 'Attitude control',
    math: [`${V('α')} = −<i>K</i><sub>R</sub>${V('e')}<sub>R</sub> − <i>K</i><sub>ω</sub>${V('ω')} − <i>K</i><sub>I</sub>∫${V('e')}<sub>R</sub> d<i>t</i>`, `${V('τ')}<sub>d</sub> = <i>J</i>${V('α')} + ${V('ω')} × <i>J</i>${V('ω')}`],
    doc: 'Gains are in angular-acceleration units and multiplied by the modeled inertia, so they carry over to new geometry. They are the design\'s (Airframe → Tuning), read from TUNE.att: kR, kW, kI for roll, pitch and yaw; replace them with numbers here and the tuning no longer reaches this formula. On the learned model J is the identity and the result is a desired angular acceleration.',
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
  { key: 'voltageCompensation', group: 'ctrl', fn: voltageCompensation, title: 'Battery voltage correction',
    math: [`<i>u</i><sub>sent</sub> = <i>u</i> · <i>V</i><sub>ref</sub> / <i>V</i><sub>measured</sub>`],
    doc: 'Runs when the battery voltage sensor is fitted. A motor\'s speed follows throttle × pack voltage, so this keeps the thrust for a given command the same as the pack drains or sags under a hard manoeuvre, and the controller\'s tables (described or learned) stay true. V_ref is the voltage the motors are rated at (16 V).',
    args: [['u', 'throttle the controller wants'], ['vMeas', 'measured pack voltage [V]'], ['vRef', 'reference voltage [V]']], returns: 'throttle sent to the ESC', shape: 'n', sample: () => [0.5, 14.8, 16] },
  { key: 'allocation', group: 'ctrl', fn: allocation, title: 'Control allocation',
    math: [`1. ${V('u')}₁ = argmin ‖<i>W</i><sub>−yaw</sub><sup>½</sup>(<i>B</i>${V('u')} − ${V('w')}<sub>d</sub>)‖² &nbsp;subject to &nbsp;${V('u')}<sub>min</sub> ≤ ${V('u')} ≤ ${V('u')}<sub>max</sub> &nbsp;(lift and tilt first)`, `2. ${V('u')}₂: as much yaw as it can get while keeping <i>B</i>${V('u')}₁'s lift, roll and pitch`, `3. ${V('u')}* = argmin ‖<i>W</i><sup>½</sup>(<i>B</i>${V('u')} − <i>B</i>${V('u')}₂)‖² + 10<sup>−5</sup><i>ē</i> Σ<sub>j</sub> <i>q</i><sub>j</sub>((<i>u</i><sub>j</sub> − <i>r</i><sub>j</sub>)/span<sub>j</sub>)², same limits`],
    doc: 'Inputs are thrust fractions from 0 to 1, so B is in acceleration per full thrust; it comes either from the airframe description or from identification. Called twice per control step. Stage 1 decides the servos: each servo rotor contributes its thrust and a small angle change δ, bounded by how far the servo can really get in the next moment (its learned speed and lag). Stage 2 solves every motor\'s thrust at the servos\' actual angles, so the motors cover whatever a moving servo hasn\'t reached yet. The first solve gets as close to the wanted lift and tilt as the limits allow, but never gives up more than a quarter of the lift for them: a torque nothing can cancel (a big rotor off the balance point) would otherwise be "fixed" by switching that rotor off. Then yaw gets what is left: when yaw can\'t be had, it gives way rather than the thrust. The last solve keeps that move and, wherever there is more than one way to make it, picks by the q, r pulls from allocationPreferences. ē is the typical effect of one input.',
    args: [['cols', 'columns of B, one 6-vector per input'], ['lo', 'lower limits'], ['hi', 'upper limits'], ['wd', 'wanted [ax, ay, az, αx, αy, αz]'], ['mode', '"tilt", "mixed" or "level"'], ['pull', '{ q, r }: preferences per input']], returns: 'one value per input',
    shape: 'alloc', sample: () => [[[0, 0, 1, 1, 1, 0.1], [0, 0, 1, -1, 1, -0.1], [0, 0, 1, -1, -1, 0.1], [0, 0, 1, 1, -1, -0.1]], [0, 0, 0, 0], [6, 6, 6, 6], [0, 0, 9.81, 0, 0, 0], 'tilt', { q: [0.02, 0.02, 0.02, 0.02], r: [3, 3, 3, 3] }] },
  { key: 'allocationPreferences', group: 'ctrl', fn: allocationPreferences, title: 'Allocation preferences',
    math: [`allowance: <i>q</i> = <i>k</i><sub>a</sub>(0.2 + 0.8<i>a</i>)(0.55 / (<i>m</i> + 0.05))², pulling toward the middle; <i>m</i> = distance to the nearer limit as a share of the range, <i>a</i> = the input's share of the steering`, `efficiency: <i>P</i> ∝ <i>T</i><sup>1.5</sup> → <i>q</i> = 0.375 <i>k</i><sub>e</sub> (<i>P</i><sub>j</sub>/Σ<i>P</i>) / √<i>u</i>, pulling toward less thrust`, `servo moves: <i>q</i> = <i>k</i><sub>s</sub>(span / 2·reach)², pulling toward staying put; reach = speed × (horizon − lag)`],
    doc: 'The tie-breakers the allocation uses when the same move can be made in more than one way. Allowance keeps every device away from its limits, so there is room to react to the next surprise; near a limit it dominates, for example easing off a nearly maxed motor and tilting a servo to make up the difference. Efficiency prefers the move that costs the least rotor power. Servo moves cost more for a slow or laggy servo, so fast corrections go to the motors and the servos take the steady part. The weights are the three sliders under Allocation; the servo speed and lag come from the actuator tests.',
    args: [['inputs', '[{ kind, x, lo, hi, power | th, range, reach }] one per input'], ['prefs', '{ allowance, efficiency, servoMove }']], returns: '{ q, r } one pull per input',
    shape: 'pull', sample: () => [[{ kind: 'thrust', x: 0.5, lo: 0, hi: 1, power: 100, authority: 1 }, { kind: 'thrust', x: 0.95, lo: 0, hi: 1, power: 100, authority: 0.5 }, { kind: 'servo', x: 0, lo: -0.2, hi: 0.2, th: 0.3, range: 0.6, reach: 0.2, authority: 0.8 }], { allowance: 0.02, efficiency: 0.02, servoMove: 0.01 }] },
  { key: 'actuatorHealth', group: 'super', fn: actuatorHealth, title: 'Actuator health check',
    math: [`gap ${V('r')} = ${V('y')} − Σ<sub>i</sub> ${V('φ')}<sub>i</sub>, &nbsp;${V('φ')}<sub>i</sub> = <i>B</i><sub>i</sub> <i>v</i><sub>i</sub> (motor <i>i</i>'s column × its thrust); &nbsp;Δ${V('r')} = ${V('r')} − its normal value`, `motor <i>i</i> alone: κ<sub>i</sub> = ⟨Δ${V('r')}, −${V('φ')}<sub>i</sub>⟩ / ‖${V('φ')}<sub>i</sub>‖², &nbsp;η<sub>i</sub> = 1 − κ<sub>i</sub> &nbsp;(1: as its table says, 0.6: 40% weaker, ≈ 0: not working)`],
    doc: 'Runs on the health supervisor\'s board at 10 Hz, on the flight core\'s data stream (it arrives over the link a few milliseconds late). The controller\'s table says what each motor is doing; the IMU says what the drone does. The gap between them stays put while the table holds (a slow average learns it, with the ordinary noise). When a motor weakens, the gap moves by the part of it that went missing, even while the controller fights it, because the table still counts on that motor. Each motor is tried as the single explanation; the best fit gives which motor and how much it lost. conf is that fit times how clearly the change stands out.',
    args: [['st', 'its own state'], ['batch', '[{ phi: one 6-vector per motor, psi: one per steering servo, y: measured [a; α] }] samples since last time'], ['dt', 'time since last call [s]'], ['memory', 'how long past data counts [s]']], returns: '{ eta, conf }: one per motor; { del, sconf }: one per servo',
    shape: { eta: 'n', conf: 'n', del: 'n', sconf: 'n' }, sample: () => [{}, [{ phi: [[0, 0, 5, 100, 100, 5], [0, 0, 5, -100, 100, -5]], psi: [], y: [0, 0, 10, 0, 200, 0] }, { phi: [[0, 0, 5, 100, 100, 5], [0, 0, 5, -100, 100, -5]], psi: [], y: [0, 0, 7, -80, 120, -4] }], 0.1, 8] },
  { key: 'faultDecision', group: 'super', fn: faultDecision, title: 'Fault decisions',
    math: [`failed: ESC rpm &lt; 30% of what the command should give for 0.2 s, or η &lt; 0.25 (conf &gt; 0.7) for 0.5 s while it's asked for thrust → removed (its limit set to 0)`, `degraded: η &lt; 0.88 held for 1.5 s (conf &gt; 0.3) → its column in the table × η`, `servo stuck: |δ| &gt; 3° (conf &gt; 0.6) for 0.5 s → left out of the steering, the controller told its real angle`, `hot: ceiling = 1 − 0.45 (<i>T</i> − (<i>T</i><sub>max</sub> − 20)) / 20, &nbsp;between 55% and 100%`],
    doc: 'Turns what the supervisor sees into settings for the flight core. A failed motor is taken out of the allocation, a degraded one has its column in the table (learned or described) scaled to what it really does, a stuck servo is taken out of the steering and the flight core is told where it really is (so its rotor\'s column is right again), and a hot one is capped so the others take more of the load before it\'s damaged. Temperatures come from a sensor, or, if the ESC reports current, from the same heating model run on the supervisor; without either, heat can\'t be seen. why says what it saw (1 the ESC reports it stopped, 2 it no longer moves the drone, 3 it delivers val of its table, 4 it runs at val °C; servos: 1 it reports it isn\'t following, 2 it is val rad off), so the board can say it in words.',
    args: [['st', 'its memory (timers, what it decided)'], ['motors', '[{ on, eff, cmd, temp, tmax, rpmRatio, eta, conf }] one per motor'], ['servos', '[{ angle, delta, conf, fbErr }] one per steering servo'], ['dt', 'time since last call [s]']], returns: '{ motors: [{ state (0 ok, 1 degraded, 2 hot, 3 failed), on, eff, cap, why, val }], servos: [{ stuck, angle, why, val }] }',
    shape: 'obj', sample: () => [{}, [{ on: 1, eff: 1, cmd: 0.4, temp: 110, tmax: 120, rpmRatio: 1, eta: 0.98, conf: 0.8 }], [{ angle: 0.1, delta: 0, conf: 0, fbErr: null }], 0.1] },
  { key: 'flightPolicy', group: 'super', fn: flightPolicy, title: 'Flight policy',
    math: [`land: roll/pitch lost, lift margin &lt; 1.08×, battery &lt; 8% or overheating`, `return home and land: a motor or a battery cell failed, margin &lt; 1.35×, battery &lt; 20% or &lt; 3.3 V/cell under load for a second`, `careful: a motor above 85% of its limit, a hot battery, or margin &lt; 1.6×`],
    doc: 'How the drone should fly on what\'s left. Each mode comes with limits the flight core and the navigation fly within: top speed, how far the body may lean and how hard it may accelerate sideways. Returning flies home at 1.5 m/s and lands (it needs the navigation task); landing comes straight down and stops the motors on the ground. It only ever steps up: once it has decided to go home, it goes home.',
    args: [['sum', '{ dt, margin, rpOk, yawOk, anyFailed, cellLost, hot, soc, vCell, battT, battMax }'], ['prev', 'its result last time']], returns: '{ mode, lim: { speed (0: none), lean, accel }, why, rpBad }',
    shape: 'obj', sample: () => [{ dt: 0.1, margin: 2, rpOk: true, yawOk: true, anyFailed: false, cellLost: false, hot: 0.5, soc: 0.8, vCell: 3.9, battT: 35, battMax: 60 }, { mode: 'normal', why: 0, rpBad: 0, vBad: 0 }] },
  { key: 'liftMargin', group: 'super', fn: liftMargin, title: 'Lift and control margin',
    math: [`hover: ${V('u')}<sub>h</sub> = argmin ‖<i>W</i><sup>½</sup>(<i>B</i>${V('u')} − (0, 0, <i>g</i>, 0, 0, 0))‖², &nbsp;roll and pitch held if what it makes is within 2 rad/s²`, `climb: ${V('u')}<sub>c</sub> for 3<i>g</i> up with the rotation held, &nbsp;margin = (<i>B</i>${V('u')}<sub>c</sub>)<sub>z</sub> / <i>g</i>`],
    doc: 'The supervisor\'s view of how much lift there is and whether the drone can still be held level, with the table as the flight core flies it now (removed motors out, hot ones capped). Steering servos count too: each can turn its rotors across what\'s left of its travel. The flight policy decides from these.',
    args: [['cols', 'one 6-vector per input (motors, then steering servos)'], ['lo', 'lower limits'], ['hi', 'upper limits']], returns: '{ margin; rpOk; yawOk }',
    shape: 'obj', sample: () => [[[0, 0, 5, 100, 100, 5], [0, 0, 5, -100, 100, -5], [0, 0, 5, -100, -100, 5], [0, 0, 5, 100, -100, -5]], [0, 0, 0, 0], [1, 1, 1, 1]] },
];

// Wrench sum and control chain shown at the top of the Formulas tab.
const LAW_OVERVIEW = [
  `<i>M</i>(${V('q')}) ${V('q̈')} + ${V('c')}(${V('q')}, ${V('q̇')}) = Σ<sub>bodies</sub> <i>J</i><sub>b</sub><sup>T</sup>(${V('f')}<sub>rotors</sub> + ${V('f')}<sub>gravity</sub> + ${V('f')}<sub>drag</sub> + ${V('f')}<sub>cables</sub> + ${V('f')}<sub>ground</sub>) + [0; ${V('τ')}<sub>servos</sub>]`,
  `${V('q')} = frame position and attitude + every servo joint angle; each body obeys ${V('f')} = <i>I</i>${V('a')} + ${V('v')} ×* <i>I</i>${V('v')}`,
];
const LAW_CHAIN = {
  ctrl: ['attitudeEstimator', 'flowVelocity', 'servoPredictor', 'positionEstimator', 'identifyThrow', 'identifyMotorResponse', 'identifyServoResponse', 'identifyEffectiveness', 'fleetProgram', 'positionControl', 'thrustAxisTarget', 'attitudeError', 'attitudeControl', 'forceDemand', 'allocationPreferences', 'allocation', 'thrustLinearization', 'voltageCompensation'],
  super: ['actuatorHealth', 'faultDecision', 'flightPolicy', 'liftMargin'],
  plant: ['batteryModel', 'thermalModel', 'motorDynamics', 'servoTorque', 'jointRotation', 'wakeVelocity', 'rotorAero', 'rotorWrench', 'wakeLoad', 'gravity', 'bodyDrag', 'wingAero', 'bluffDrag', 'cableTension', 'payloadDrag', 'groundContact', 'rigidBody', 'imuModel', 'magModel', 'baroModel', 'posFixModel', 'flowModel', 'rangeModel'],
};
