'use strict';
// What each flight formula takes and returns, for the step compiler. The firmware passes exactly these, so an
// edited formula can change what it computes but not its inputs. Lists have a most-elements (their capacity):
// RN_IN is how many actuator inputs the runner is built for (motors, plus a motor's terms for each joint it
// rides on); a drone that grows parts mid-flight uses the spare places.
const RN_IN = 24;
const RN_MOT = 12, RN_JNT = 8;     // motors and servo joints (as the flight core: FC_MAX_MOTORS, FC_MAX_JOINTS)
const RN_THROW_IN = 12;            // inputs the throw start can identify (a quadcopter with a tilting motor on each arm)
const RN_WIN = 128;                // samples in one actuator-test window (0.6 s at 200 Hz)
const RN_SIGS = (() => {
  const { num, arr, list, ring, rec, opt, enm, state } = RT;
  const V2 = arr(num, 2), V3 = arr(num, 3), V4 = arr(num, 4), V6 = arr(num, 6), M3 = arr(num, 9);
  const MODE = enm('tilt', 'mixed', 'level'), L = list(num, RN_IN);
  const LT = list(num, RN_THROW_IN), LM = list(num, RN_MOT), LJ = list(num, RN_JNT), POLICY = enm('normal', 'caution', 'return', 'land');
  const THROW = rec({ B: arr(LT, 6), B2: arr(LT, 3), r: V3, drag: num, tau: num, taus: LT, fitF: num, fitR: num, refined: num, improved: num, progress: num, spent: num });
  const S = (names, args, ret, opts) => ({ names, args, ret, ...opts });   // ownPool: may run on another core
  return {
    attitudeEstimator: S(['st', 'gyro', 'accel', 'mag', 'dt'], [state(), V3, V3, opt(V3), num], rec({ q: V4, w: V3 })),
    flowVelocity: S(['flow', 'range', 'w', 'Rs'], [V2, num, V3, M3], V3),
    servoPredictor: S(['st', 'cmd', 'rate', 'lag', 'dt'], [state(), num, num, num, num], num),
    positionEstimator: S(['st', 'R', 'accel', 'baro', 'fix', 'flow', 'm', 'dt'],
      [state({ h: ring(rec({ p: V3, v: V3 }), 801) }), M3, V3, opt(rec({ age: num, alt: num })), opt(rec({ age: num, p: V3, v: V3 })),
        opt(rec({ age: num, h: opt(num), v: opt(V2) })), num, num], rec({ p: V3, v: V3 })),
    identifyEffectiveness: S(['st', 'u', 'f', 'w', 'r', 'dt', 'init', 'memory', 'lags', 'mot'],
      [state(), L, V3, V3, V3, num, arr(L, 6), num, opt(L), opt(rec({ coll: L, m: L, phi: L, v: L }))], rec({ B: arr(L, 6), B2: opt(arr(L, 3)) }), { ownPool: true }),
    positionControl: S(['ep', 'v', 'ip', 'm', 'g', 'lim'], [V3, V3, V3, num, num, opt(rec({ accel: opt(num), lean: opt(num), speed: opt(num) }))], V3),
    thrustAxisTarget: S(['Fd', 'mode', 'share', 'leanMax'], [V3, MODE, num, num], V3),
    attitudeError: S(['R', 'Rd'], [M3, M3], V3),
    attitudeControl: S(['eR', 'w', 'ia', 'J'], [V3, V3, V3, M3], V3),
    forceDemand: S(['Fb', 'n', 'mode'], [V3, V3, MODE], V3),
    allocationPreferences: S(['inputs', 'prefs'],
      [list(rec({ kind: enm('thrust', 'servo'), x: num, lo: num, hi: num, authority: opt(num), power: opt(num), th: opt(num), range: opt(num), reach: opt(num) }), RN_IN),
        rec({ allowance: num, efficiency: num, servoMove: num })], rec({ q: L, r: L })),
    allocation: S(['cols', 'lo', 'hi', 'wd', 'mode', 'pull'], [list(V6, RN_IN), L, L, V6, MODE, opt(rec({ q: L, r: L }))], L),
    thrustLinearization: S(['v', 'bend'], [num, num], num),
    voltageCompensation: S(['u', 'vMeas', 'vRef'], [num, num, num], num),
    // The learning task (on a Pi): actuator tests and the throw start.
    identifyMotorResponse: S(['wins', 'dt'], [list(rec({ u: list(num, RN_WIN), y: list(num, RN_WIN) }), 4), num], rec({ tau: num, curve: num, gain: num, fit: num })),
    identifyServoResponse: S(['wins', 'dt'], [list(rec({ cmd: list(num, RN_WIN), y: list(num, RN_WIN) }), 4), num], rec({ rate: num, lag: num, gain: num, fit: num })),
    identifyThrow: S(['st', 'u', 'f', 'w', 'vb', 'dt', 'solve', 'mot', 'budget'],
      [state({ log: list(rec({ v: LT, ph: LT, fl: V3, a: V3, W: V3, vb: V3, h: num, use: num }), 450), out: THROW }), LT, V3, V3, V3, num, enm('fall', 'catch', 'refine'),
        rec({ coll: LT, m: LT, phi: LT, v: LT }), num], THROW),
    // The health supervisor (on a Pi).
    actuatorHealth: S(['st', 'batch', 'dt', 'memory'],
      [state({ eta: LM, conf: LM, del: LJ, sconf: LJ, lag: list(V6, RN_MOT), lagS: list(V6, RN_JNT), pp: LM, rp: LM, sp: LJ, sr: LJ, base: V6 }),
        list(rec({ phi: list(V6, RN_MOT), psi: list(V6, RN_JNT), y: V6 }), 16), num, num], rec({ eta: LM, conf: LM, del: LJ, sconf: LJ })),
    faultDecision: S(['st', 'motors', 'servos', 'dt'],
      [state({ dead: LM, weak: LM, why: LM, val: LM, eff: LM, off: LJ, stuck: LJ, swhy: LJ, sval: LJ }),
        list(rec({ on: num, eff: num, cmd: num, temp: opt(num), tmax: num, rpmRatio: opt(num), eta: num, conf: num }), RN_MOT),
        list(rec({ angle: num, delta: num, conf: num, fbErr: opt(num) }), RN_JNT), num],
      rec({ motors: list(rec({ state: num, on: num, eff: num, cap: num, why: num, val: num }), RN_MOT), servos: list(rec({ stuck: num, angle: num, why: num, val: num }), RN_JNT) })),
    flightPolicy: S(['sum', 'prev'],
      [rec({ dt: num, margin: num, rpOk: num, yawOk: num, anyFailed: num, cellLost: num, hot: opt(num), soc: opt(num), vCell: opt(num), battT: opt(num), battMax: num }), opt(rec({ mode: POLICY, why: num, rpBad: num, vBad: num }))],
      rec({ mode: POLICY, lim: rec({ speed: num, lean: num, accel: num }), why: num, rpBad: num, vBad: num })),
    liftMargin: S(['cols', 'lo', 'hi'], [list(V6, RN_MOT + RN_JNT), list(num, RN_MOT + RN_JNT), list(num, RN_MOT + RN_JNT)], rec({ margin: num, rpOk: num, yawOk: num })),
    thermalModel: S(['T', 'P', 'G', 'C', 'Tamb', 'dt'], [num, num, num, num, num, num], num),
    rotorWrench: S(['d', 'r', 'T', 'spin', 'kappa'], [V3, V3, num, num, num], rec({ F: V3, tau: V3 })),
    jointRotation: S(['axis', 'theta'], [V3, num], M3),
  };
})();
// Which formulas each task runs (js/boards.js TASKS): a board's program has those of its tasks.
const RN_TASK_FORMULAS = {
  core: ['attitudeEstimator', 'servoPredictor', 'thrustAxisTarget', 'attitudeError', 'attitudeControl', 'forceDemand', 'allocationPreferences', 'allocation', 'thrustLinearization', 'voltageCompensation'],
  nav: ['positionEstimator', 'flowVelocity', 'positionControl'],
  learn: ['identifyEffectiveness', 'identifyMotorResponse', 'identifyServoResponse', 'identifyThrow'],
  super: ['actuatorHealth', 'faultDecision', 'flightPolicy', 'liftMargin', 'thermalModel'],
};
const rnTaskFormulas = tasks => { const out = []; for (const t of tasks) for (const k of RN_TASK_FORMULAS[t] || []) if (!out.includes(k)) out.push(k); return out; };
if (typeof module !== 'undefined') module.exports = { RN_SIGS, RN_IN, RN_MOT, RN_JNT, RN_THROW_IN, RN_WIN, RN_TASK_FORMULAS, rnTaskFormulas };
