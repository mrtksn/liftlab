'use strict';
// The controller's tuning: the gains the control formulas read from TUNE (laws.js), kept with the design.
//
// cfg.tuning holds the raw gains, always complete and within bounds (tuneFix): { att: { kR, kW, kI } per axis
// (roll, pitch, yaw), pos: { kp, kd, ki } }. Both loops divide out the drone's own size first (attitudeControl
// multiplies by the inertia J, positionControl by the mass m), so each axis is a double integrator and its gains
// map exactly onto how fast and how damped it is:
//   kR = ωn², kW = 2ζωn, kI = ωn²·i      (attitude; position: kp, kd, ki the same way)
// with ωn the natural frequency [rad/s] (shown in Hz), ζ the damping ratio and i = kI/kR the integral rate [1/s]:
// how quickly it trims out a steady push (an off-centre weight, wind, a hover throttle it didn't know).
// The Airframe tab (tuning-ui.js) shows those three; Raw gains shows the numbers the formulas get.

const TUNE_BOUNDS = { kR: [1, 2500], kW: [0, 200], kI: [0, 2500], kp: [0.05, 50], kd: [0.1, 40], ki: [0, 25] };
const TUNE_AXES = ['roll', 'pitch', 'yaw'];
const tuneDefaults = () => JSON.parse(JSON.stringify(TUNE_DEFAULTS));
// A complete tuning from anything (a design file, an old save, typed values): every gain a finite number within
// its bounds; what's missing or broken is the default.
function tuneFix(t) {
  const D = TUNE_DEFAULTS, out = tuneDefaults();
  const one = (v, k, d) => { const [lo, hi] = TUNE_BOUNDS[k]; return typeof v === 'number' && Number.isFinite(v) ? clamp(v, lo, hi) : d; };
  const a = t && t.att, p = t && t.pos;
  for (const k of ['kR', 'kW', 'kI']) for (let i = 0; i < 3; i++) out.att[k][i] = one(a && Array.isArray(a[k]) ? a[k][i] : undefined, k, D.att[k][i]);
  for (const k of ['kp', 'kd', 'ki']) out.pos[k] = one(p ? p[k] : undefined, k, D.pos[k]);
  return out;
}
const tuneOf = () => cfg.tuning || TUNE_DEFAULTS;
tuneSource = tuneOf;                                     // (laws.js: formulas run as JavaScript read the selected drone's)
const tuneSame = (a, b) => JSON.stringify(tuneFix(a)) === JSON.stringify(tuneFix(b));
const tuneIsDefault = () => tuneSame(tuneOf(), TUNE_DEFAULTS);
// Formulas the tuning reaches: the flight formulas in use that read TUNE (an edited one may not).
const tuneReaders = () => Object.keys(RN_SIGS).filter(k => rnReadsTune(rnSourceOf(k)));

// Response, damping and integral rate ↔ the raw gains. Attitude: axis 0 roll, 1 pitch, 2 yaw; position: one set.
function tuneFeel(t, loop, axis = 0) {
  const [P, D, I] = loop === 'pos' ? [t.pos.kp, t.pos.kd, t.pos.ki] : [t.att.kR[axis], t.att.kW[axis], t.att.kI[axis]];
  const wn = Math.sqrt(P);
  return { hz: wn / (2 * Math.PI), zeta: D / (2 * wn), integ: I / P };
}
function tuneGains(hz, zeta, integ) { const wn = 2 * Math.PI * hz; return [wn * wn, 2 * zeta * wn, wn * wn * integ]; }
// Set one loop's feel: the attitude axes it names (roll and pitch together, or yaw), or position.
function tuneWithFeel(t, loop, axes, feel) {
  const n = tuneFix(t), [P, D, I] = tuneGains(feel.hz, feel.zeta, feel.integ);
  if (loop === 'pos') Object.assign(n.pos, { kp: P, kd: D, ki: I });
  else for (const i of axes) { n.att.kR[i] = P; n.att.kW[i] = D; n.att.kI[i] = I; }
  return tuneFix(n);
}

/* ───────── what to expect: a small-step prediction ───────── */
// The motors' thrust lag the prediction uses: the slowest motor's spin-up time on its card (a collective-pitch
// rotor answers through its pitch servo instead: about 20 ms).
function tuneMotorLag() {
  const m = cfg.comps.filter(c => c.type === 'motor');
  return m.length ? Math.max(...m.map(c => c.pitch === 'collective' ? 0.02 : Math.max(0.005, c.tau || 0.03))) : 0.03;
}
// One attitude axis as the controller sees it: a pure inertia (J divided out), the motors' first-order lag and a
// control step of delay, flown by kR, kW, kI toward a reference. Position: the drone accelerates by leaning, so
// its acceleration is g × the tilt the roll loop achieves. Linear and small: no limits, no integral clamp, no
// drag or wind. Returns the response to a unit step, sampled for plotting, and how it went.
function tunePredict(t, loop, axis = 0, lag = tuneMotorLag()) {
  const dt = 0.0005, delay = 0.002, nDel = Math.round(delay / dt), Tmax = loop === 'pos' ? 60 : 20, n = Math.ceil(Tmax / dt);
  const a = t.att, k = loop === 'pos' ? [a.kR[0], a.kW[0], a.kI[0]] : [a.kR[axis], a.kW[axis], a.kI[axis]];
  let th = 0, w = 0, ia = 0, u = 0, x = 0, v = 0, ip = 0, peak = 0, settle = 0, rise = null, bad = false;
  const buf = new Array(nDel).fill(0), ys = new Float32Array(n + 1);
  let s = 0;
  for (; s <= n; s++) {
    let ref = 1;
    if (loop === 'pos') { const ep = 1 - x; ref = (t.pos.kp * ep - t.pos.kd * v + t.pos.ki * ip) / G; ip += ep * dt; }   // the lean asked for [rad]
    const e = th - ref, cmd = -k[0] * e - k[1] * w - k[2] * ia; ia += e * dt;
    buf.push(cmd); const late = buf.shift();
    u += (late - u) * Math.min(1, dt / lag);
    w += u * dt; th += w * dt;
    if (loop === 'pos') { v += G * th * dt; x += v * dt; }
    const y = loop === 'pos' ? x : th, tt = s * dt;
    if (!Number.isFinite(y) || Math.abs(y) > 50) { bad = true; break; }
    ys[s] = y; if (y > peak) peak = y;
    if (rise === null && y >= 0.9) rise = tt;
    if (Math.abs(y - 1) > 0.05) settle = tt;
    else if (tt - settle > 3 && (rise ?? Infinity) < tt) break;     // within 5% for 3 s: settled
  }
  const stable = !bad && settle < Tmax * 0.8;
  // The window shown: until it has settled (or, if it doesn't, a few swings), sampled for plotting.
  const swing = rise ?? 0.2 / Math.max(0.05, tuneFeel(t, loop, axis).hz);
  const T = clamp(stable ? Math.max(settle * 1.25, swing * 3) : swing * 6, 0.3, Tmax), last = Math.min(s - 1, Math.round(T / dt));
  const every = Math.max(1, Math.floor(last / 160)), ts = [], out = [];
  for (let i = 0; i <= last; i += every) { ts.push(i * dt); out.push(clamp(ys[i], -1, 3)); }
  return { t: ts, y: out, T, lag, overshoot: Math.max(0, peak - 1), rise, settle: stable ? settle : null, stable };
}
// What the prediction says, in words, and how worried to be.
function tuneVerdict(p, f) {
  if (!p.stable) return { tone: 'bad', text: `Won't settle: too quick for motors that take ${Math.round(p.lag * 1000)} ms to change thrust, or too little damping. Expect it to shake or flip.` };
  const os = Math.round(p.overshoot * 100), parts = [`overshoots ${os}%`, `settles in ${p.settle < 1 ? Math.round(p.settle * 1000) + ' ms' : p.settle.toFixed(1) + ' s'}`];
  if (p.overshoot > 0.3) return { tone: 'warn', text: parts.join(' · ') + '. Bouncy: more damping, or a slower response.' };
  if (2 * Math.PI * f.hz * p.lag > 0.35) return { tone: 'warn', text: parts.join(' · ') + `. Close to what motors with ${Math.round(p.lag * 1000)} ms of lag can follow: a little wind or noise may set it wobbling.` };
  return { tone: '', text: parts.join(' · ') };
}
