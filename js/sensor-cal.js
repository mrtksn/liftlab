'use strict';
// Sensor calibration: the IMU's mount and its accelerometer's offsets and gains, from a few still poses (Tune →
// Sensor calibration, sensor-cal-ui.js). Each pose is a second's average as the chip reads it, in its own axes, before
// any calibration (the firmware's "imucal", the simulator's the same): acceleration [m/s²] and rotation [rad/s].
//
// Lying still, an accelerometer reads 1 g straight up. A cheap one reads it a little off: an offset and a gain an axis
// (0.9 g one way up, 1.1 g the other). Poses that turn every axis up and down pin those six numbers: the fit makes
// every pose read exactly 1 g, |s ∘ (a − o)| = g (Gauss–Newton). With fewer poses, one gain for all three makes the
// level pose read 1 g, as the firmware does by itself at power-up.
// The mount: lying level, the reading is the body's up (Z) in the sensor's axes; nose up, what it gains besides up is
// the body's forward (X). Y = Z × X. That's the whole turn, the board's heading too (gravity alone can't tell it).
// The gyro's offset: its average over the still poses. Level again, last: six poses fit six numbers exactly whatever
// they read, so a seventh is what shows a pose that moved (it reads off 1 g after the fit); and it must agree with the
// first level pose, which the mount rests on.

const SensorCal = (() => {
  // The poses in the order asked. need: the two the mount comes from; up: the body axis each points up, roughly
  // (checked against what the poses measured, so a pose taken the wrong way round is caught).
  const POSES = [
    { id: 'flat', label: 'Level, right way up', how: 'Set it level on a table or hold it level, the right way up, and keep it still.', up: [0, 0, 1], need: true },
    { id: 'nose', label: 'Nose up', how: 'Lift the front: nose up, anywhere from 30° to straight up. Hold it still.', up: [1, 0, 0], need: true },
    { id: 'tail', label: 'Nose down', how: 'Now the other way: nose down (tail up), as steep as you can. Hold it still.', up: [-1, 0, 0] },
    { id: 'left', label: 'Left side down', how: 'Roll it onto its left side (the right side up). Hold it still.', up: [0, -1, 0] },
    { id: 'right', label: 'Right side down', how: 'Roll it onto its right side (the left side up). Hold it still.', up: [0, 1, 0] },
    { id: 'upside', label: 'Upside down', how: 'Turn it upside down. Hold it still.', up: [0, 0, -1] },
    { id: 'again', label: 'Level again', how: 'Back to level, the right way up, as in the first pose. Hold it still.', up: [0, 0, 1] },
  ];
  const MIN_TILT = 20;        // [°] the nose-up pose at least this far from level
  const MAX_RES = 0.01;       // a pose more than this far from 1 g after the fit [fraction of g]: something moved
  const SPAN = 1.2;           // each axis must swing this far (as a fraction of g, top to bottom) for the fit

  const norm = a => Math.hypot(a[0], a[1], a[2]);
  const dot3 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross3 = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const unit3 = a => { const n = norm(a); return [a[0] / n, a[1] / n, a[2] / n]; };

  // Parses the drone's answer "imucal AX AY AZ GX GY GZ" (mm/s², 0.1 mrad/s: whole numbers to fit a radio message).
  // A pose, or { error } for the drone's refusal ("imucal: moving ..."), or null for anything else.
  function parseAnswer(text) {
    const s = String(text || '').replace(/^esp32:\s*/, '').trim();
    const m = /^imucal((?:\s+-?\d+){6})\s*$/.exec(s);
    if (m) { const v = m[1].trim().split(/\s+/).map(Number); return { acc: v.slice(0, 3).map(x => x / 1000), gyro: v.slice(3).map(x => x / 1e4) }; }
    if (/^disarm first\b/.test(s)) return { error: 'disarm it first' };   // (the flight controller's answer to any setting while armed)
    const e = /^imucal:\s*(.+)$/.exec(s);
    return e ? { error: e[1] } : null;
  }

  // The six numbers that make every reading 1 g: offset o and gain s an axis. null when the poses don't swing every
  // axis both ways (too few, or too alike).
  function fitAccel(accs, g = G) {
    if (accs.length < 6) return null;
    for (let i = 0; i < 3; i++) {
      const u = accs.map(a => a[i] / norm(a));
      if (Math.max(...u) - Math.min(...u) < SPAN) return null;
    }
    const gain0 = g / (accs.reduce((t, a) => t + norm(a), 0) / accs.length);
    let o = [0, 0, 0], s = [gain0, gain0, gain0], lam = 1e-3, cost = Infinity;
    const resid = (o, s) => accs.map(a => norm([s[0] * (a[0] - o[0]), s[1] * (a[1] - o[1]), s[2] * (a[2] - o[2])]) - g);
    for (let it = 0; it < 60; it++) {
      const r = resid(o, s); cost = r.reduce((t, x) => t + x * x, 0);
      const H = Array.from({ length: 6 }, () => Array(6).fill(0)), b = Array(6).fill(0);
      accs.forEach((a, k) => {
        const v = [0, 1, 2].map(i => s[i] * (a[i] - o[i])), n = norm(v) || 1;
        const J = [0, 1, 2].map(i => -s[i] * v[i] / n).concat([0, 1, 2].map(i => (a[i] - o[i]) * v[i] / n));   // ∂r/∂o, ∂r/∂s
        for (let p = 0; p < 6; p++) { b[p] -= J[p] * r[k]; for (let q = 0; q < 6; q++) H[p][q] += J[p] * J[q]; }
      });
      for (let p = 0; p < 6; p++) H[p][p] *= 1 + lam;
      const d = solveLin(H, b);
      if (!d || d.some(x => !Number.isFinite(x))) return null;
      const o2 = o.map((x, i) => x + d[i]), s2 = s.map((x, i) => x + d[3 + i]);
      const c2 = resid(o2, s2).reduce((t, x) => t + x * x, 0);
      if (c2 < cost) { o = o2; s = s2; lam = Math.max(lam / 3, 1e-9); if (cost - c2 < 1e-14) break; } else lam *= 10;
    }
    if (s.some(x => !(x > 0.7 && x < 1.4)) || o.some(x => Math.abs(x) > 4)) return null;
    return { bias: o, scale: s };
  }

  // The poses taken ({ id: { acc, gyro } }) → the calibration, or { error } saying what's missing or wrong.
  // mount: sensor → body (a reading turned into body axes: R · v), rows the body's X, Y, Z in the sensor's axes.
  function solve(poses, g = G) {
    const got = POSES.filter(p => poses[p.id]);
    for (const p of POSES) if (p.need && !poses[p.id]) return { error: `measure "${p.label}" first` };
    const accs = got.map(p => poses[p.id].acc);
    const fit = fitAccel(accs, g);
    const uni = g / norm(poses.flat.acc);
    const bias = fit ? fit.bias : [0, 0, 0], scale = fit ? fit.scale : [uni, uni, uni];
    const cal = a => [0, 1, 2].map(i => scale[i] * (a[i] - bias[i]));
    const z = unit3(cal(poses.flat.acc)), n = cal(poses.nose.acc);
    const side = n.map((x, i) => x - dot3(n, z) * z[i]), tilt = Math.asin(Math.min(1, norm(side) / norm(n))) * R2D;
    if (!(tilt >= MIN_TILT)) return { error: `nose up is only ${tilt.toFixed(0)}° from level: lift the front at least ${MIN_TILT}° and measure it again` };
    const x = unit3(side), y = cross3(z, x), mount = [...x, ...y, ...z];
    const res = got.map(p => {
      const c = cal(poses[p.id].acc), b = [dot3(x, c), dot3(y, c), dot3(z, c)];
      return { id: p.id, label: p.label, err: norm(c) / g - 1, agrees: dot3(unit3(b), p.up) > 0.3 };
    });
    const wrong = res.find(r => !r.agrees);
    if (wrong) return { error: `"${wrong.label}" doesn't point the way its name says (was it taken the wrong way round?): measure it again` };
    if (poses.again) {
      const d = Math.acos(Math.max(-1, Math.min(1, dot3(z, unit3(cal(poses.again.acc)))))) * R2D;
      if (d > 2) return { error: `the two level poses differ by ${d.toFixed(1)}°: level the same way both times (on a table) and measure them again` };
    }
    const worst = res.reduce((m, r) => Math.abs(r.err) > Math.abs(m.err) ? r : m, res[0]);
    const gyro = [0, 1, 2].map(i => got.reduce((t, p) => t + poses[p.id].gyro[i], 0) / got.length);
    return {
      mount, bias, scale, gyro, fitted: !!fit, residuals: res, worst,
      ok: Math.abs(worst.err) <= MAX_RES,
      note: fit ? (poses.again ? '' : 'Level again, last, checks the others.') : got.length < 6 ? 'With all six poses the offset and gain of each axis are measured; with these only one gain for all three.' : 'The poses don\'t swing every axis up and down: one gain for all three.',
    };
  }

  return { POSES, MIN_TILT, MAX_RES, parseAnswer, fitAccel, solve };
})();
if (typeof module !== 'undefined') module.exports = SensorCal;
