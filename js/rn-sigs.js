'use strict';
// What each flight formula takes and returns, for the step compiler. The firmware passes exactly these, so an
// edited formula can change what it computes but not its inputs. Lists have a most-elements (their capacity):
// RN_IN is how many actuator inputs the runner is built for (motors, plus a motor's terms for each joint it
// rides on); a drone that grows parts mid-flight uses the spare places.
const RN_IN = 24;
const RN_SIGS = (() => {
  const { num, arr, list, ring, rec, opt, enm, state } = RT;
  const V2 = arr(num, 2), V3 = arr(num, 3), V4 = arr(num, 4), V6 = arr(num, 6), M3 = arr(num, 9);
  const MODE = enm('tilt', 'mixed', 'level'), L = list(num, RN_IN);
  const S = (names, args, ret) => ({ names, args, ret });
  return {
    attitudeEstimator: S(['st', 'gyro', 'accel', 'mag', 'dt'], [state(), V3, V3, opt(V3), num], rec({ q: V4, w: V3 })),
    flowVelocity: S(['flow', 'range', 'w', 'Rs'], [V2, num, V3, M3], V3),
    servoPredictor: S(['st', 'cmd', 'rate', 'lag', 'dt'], [state(), num, num, num, num], num),
    positionEstimator: S(['st', 'R', 'accel', 'baro', 'fix', 'flow', 'm', 'dt'],
      [state({ h: ring(rec({ p: V3, v: V3 }), 801) }), M3, V3, opt(rec({ age: num, alt: num })), opt(rec({ age: num, p: V3, v: V3 })),
        opt(rec({ age: num, h: opt(num), v: opt(V2) })), num, num], rec({ p: V3, v: V3 })),
    identifyEffectiveness: S(['st', 'u', 'f', 'w', 'r', 'dt', 'init', 'memory', 'lags', 'mot'],
      [state(), L, V3, V3, V3, num, arr(L, 6), num, opt(L), opt(rec({ coll: L, m: L, phi: L, v: L }))], rec({ B: arr(L, 6), B2: opt(arr(L, 3)) })),
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
    rotorWrench: S(['d', 'r', 'T', 'spin', 'kappa'], [V3, V3, num, num, num], rec({ F: V3, tau: V3 })),
    jointRotation: S(['axis', 'theta'], [V3, num], M3),
  };
})();
if (typeof module !== 'undefined') module.exports = { RN_SIGS, RN_IN };
