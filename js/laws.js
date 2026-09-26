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
  const cd = 0.25;                                       // linear drag [N per m/s of airspeed]
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

// ═════════════ Controller ═════════════

function positionControl(ep, v, ip, m, g) {
  // ep: position error, v: velocity, ip: integral of ep, all world frame
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

  { key: 'positionControl', group: 'ctrl', fn: positionControl, title: 'Position control',
    math: [`${V('a')}<sub>d</sub> = <i>K</i><sub>p</sub>${V('e')}<sub>p</sub> − <i>K</i><sub>d</sub>${V('v')} + <i>K</i><sub>i</sub>∫${V('e')}<sub>p</sub> d<i>t</i>`, `${V('F')}<sub>d</sub> = <i>m</i>(${V('a')}<sub>d</sub> + <i>g</i>${V('ẑ')})`],
    doc: 'PID on the frame hub\'s position. The integral is kept by the simulator and clamped to ±2 m·s. m is the mass the controller believes in.',
    args: [['ep', 'position error, world [m]'], ['v', 'hub velocity, world [m/s]'], ['ip', '∫ ep dt [m·s]'], ['m', 'modeled mass [kg]'], ['g', '9.81 m/s²']], returns: 'desired total force, world [N]',
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
    doc: 'Gains are in angular-acceleration units and multiplied by the modeled inertia, so they carry over to new geometry.',
    args: [['eR', 'attitude error [rad]'], ['w', 'angular velocity, body [rad/s]'], ['ia', '∫ eR dt'], ['J', 'modeled inertia']], returns: 'desired torque, body [N·m]',
    shape: 3, sample: () => [[0.01, 0, 0], [0, 0, 0], [0, 0, 0], [.01, 0, 0, 0, .01, 0, 0, 0, .02]] },
  { key: 'forceDemand', group: 'ctrl', fn: forceDemand, title: 'Body force demand',
    math: [`tilt body: ${V('f')} = (${V('F')}<sub>b</sub> · ${V('n')}) ${V('n')}`, `stay level: ${V('f')} = ${V('F')}<sub>b</sub>`],
    doc: 'Which part of the desired force the actuators are asked to make directly, in the body frame.',
    args: [['Fb', 'desired force, body [N]'], ['n', 'nominal thrust axis, body'], ['mode', '"tilt" or "level"']], returns: 'force demand, body [N]',
    shape: 3, sample: () => [[0.5, 0, 9.81], [0, 0, 1], 'tilt'] },
  { key: 'allocation', group: 'ctrl', fn: allocation, title: 'Control allocation',
    math: [`${V('u')}* = argmin ‖<i>W</i><sup>½</sup>(<i>B</i>${V('u')} − ${V('w')}<sub>d</sub>)‖² &nbsp;subject to &nbsp;${V('u')}<sub>min</sub> ≤ ${V('u')} ≤ ${V('u')}<sub>max</sub>`],
    doc: 'Called twice per control step. Stage 1 includes servo-driven rotors as two virtual inputs each, (T cos θ, T sin θ), and the servo angle is taken from their ratio. Stage 2 solves every motor\'s thrust at the servos\' actual angles.',
    args: [['cols', 'columns of B, one 6-vector per input'], ['lo', 'lower limits'], ['hi', 'upper limits'], ['wd', 'wanted [ax, ay, az, αx, αy, αz]'], ['mode', '"tilt" or "level"']], returns: 'one value per input',
    shape: 'alloc', sample: () => [[[0, 0, 1, 1, 1, 0.1], [0, 0, 1, -1, 1, -0.1], [0, 0, 1, -1, -1, 0.1], [0, 0, 1, 1, -1, -0.1]], [0, 0, 0, 0], [6, 6, 6, 6], [0, 0, 9.81, 0, 0, 0], 'tilt'] },
];

// Wrench sum and control chain shown at the top of the Formulas tab.
const LAW_OVERVIEW = [
  `<i>m</i>${V('a')} = Σ<sub>i</sub> <i>R</i>${V('F')}<sub>i</sub> + ${V('F')}<sub>g</sub> + ${V('F')}<sub>d</sub> + Σ<sub>j</sub> <i>T</i><sub>c,j</sub>${V('n')}<sub>j</sub> + Σ ${V('F')}<sub>ground</sub>`,
  `<i>J</i>${V('ω̇')} + ${V('ω')} × <i>J</i>${V('ω')} = Σ<sub>i</sub> ${V('τ')}<sub>i</sub> + ${V('τ')}<sub>d</sub> + Σ<sub>j</sub> ${V('r')}<sub>j</sub> × <i>R</i><sup>T</sup><i>T</i><sub>c,j</sub>${V('n')}<sub>j</sub> + …`,
];
const LAW_CHAIN = {
  ctrl: ['positionControl', 'thrustAxisTarget', 'attitudeError', 'attitudeControl', 'forceDemand', 'allocation'],
  plant: ['servoResponse', 'motorResponse', 'tiltAxis', 'rotorWrench', 'gravity', 'bodyDrag', 'cableTension', 'groundContact', 'rigidBody'],
};
