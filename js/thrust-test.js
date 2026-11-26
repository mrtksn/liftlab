'use strict';
// Scale thrust test: each motor's real thrust against its throttle, read off a kitchen scale (Tune → Motor thrust,
// thrust-test-ui.js). The drone sits on the scale (upside down, so the thrust pushes it down; or the right way up and
// weighted, so it pulls up), the scale is read with the motors off, then one motor at a time spins at a few throttles
// and each reading is typed in. The change from the motors-off reading is that motor's thrust.
//
// The flight controller's picture of a motor: the thrust fraction v = (1 − k)·e + k·e² of its full thrust Tmax at the
// design's reference voltage, where e = u·V/Vref is the throttle u it sends scaled by the pack's voltage V (what its
// voltage compensation undoes). So the readings fit T = a·e + b·e² by least squares: Tmax = a + b, k = b / Tmax.
// A motor test sends the throttle straight to the ESC (no linearisation, no voltage compensation), so u is the ESC's.
//
// The simulator's motor is physical (KV, winding resistance, prop thrust coefficient); fitPhysics finds the prop
// thrust coefficient that makes it give the same thrust at the same throttles and voltages, so the simulated motor
// becomes this one.

const ThrustTest = (() => {
  const LEVELS = [0.2, 0.35, 0.5, 0.65, 0.8, 1];   // the throttles asked, lowest first
  const MIN_N = 0.01;           // [N] a reading below this (1 g) is "not spinning yet" (an ESC's dead band): not fitted
  const MAX_REL = 0.08;         // the fit's RMS miss beyond this fraction of Tmax: the readings disagree
  const BEND = [-0.3, 1.3];     // a prop's curve bends within this; outside, the readings are off (or the scale is)
  const levelsUpTo = top => LEVELS.filter(u => u <= top + 1e-9).concat(LEVELS.some(u => Math.abs(u - top) < 1e-9) ? [] : [top]).filter(u => u > 0).sort((a, b) => a - b);

  // A typed scale reading → grams (a number, "312", "312 g", "0.312 kg", "11 oz"), or null.
  function parseGrams(text) {
    const m = /^\s*(-?\d+(?:[.,]\d+)?)\s*(g|kg|oz|lb)?\s*$/i.exec(String(text ?? ''));
    if (!m) return null;
    const v = +m[1].replace(',', '.'), u = (m[2] || 'g').toLowerCase();
    return v * ({ g: 1, kg: 1000, oz: 28.349523, lb: 453.59237 })[u];
  }

  // The points of one motor ({ u, V, grams }: grams the reading minus the motors-off one, either sign) → the fit.
  // vref: the design's reference voltage; V missing or 0 counts as vref. { tmax [N], bend, rms [N], rel, used,
  // points: { u, e, T, fit } } or { error }.
  function fitCurve(points, vref, g = 9.80665) {
    const pts = points.filter(p => Number.isFinite(p.grams) && p.u > 0).map(p => {
      const V = p.V > 0.5 ? p.V : vref, e = p.u * V / vref;
      return { u: p.u, V, e, T: Math.abs(p.grams) * g / 1000 };
    });
    const used = pts.filter(p => p.T >= MIN_N);
    if (used.length < 3) return { error: used.length ? `only ${used.length} reading${used.length > 1 ? 's' : ''} with thrust: at least 3 are needed` : 'no reading shows thrust yet' };
    const span = Math.max(...used.map(p => p.e)) - Math.min(...used.map(p => p.e));
    if (span < 0.25) return { error: 'the readings span too little throttle: test up to a higher throttle' };
    // least squares for T = a e + b e² (through zero thrust at zero throttle)
    let s2 = 0, s3 = 0, s4 = 0, t1 = 0, t2 = 0;
    for (const p of used) { const e = p.e; s2 += e * e; s3 += e ** 3; s4 += e ** 4; t1 += p.T * e; t2 += p.T * e * e; }
    const det = s2 * s4 - s3 * s3;
    if (!(Math.abs(det) > 1e-12)) return { error: 'the readings can\'t be fitted (all at one throttle?)' };
    const a = (t1 * s4 - t2 * s3) / det, b = (s2 * t2 - s3 * t1) / det, tmax = a + b;
    if (!(tmax > 0)) return { error: 'the readings fall as the throttle rises: check the order, and the motors-off reading' };
    const bend = b / tmax, fit = e => a * e + b * e * e;
    const rms = Math.sqrt(used.reduce((t, p) => t + (fit(p.e) - p.T) ** 2, 0) / used.length);
    const out = { tmax, bend, rms, rel: rms / tmax, used: used.length, vref, points: pts.map(p => ({ ...p, fit: fit(p.e) })) };
    if (bend < BEND[0] || bend > BEND[1]) out.warn = `its curve bends ${bend.toFixed(2)} (a prop's is between ${BEND[0]} and ${BEND[1]}): a reading is probably off`;
    else if (out.rel > MAX_REL) out.warn = `the readings miss the curve by ${(out.rel * 100).toFixed(0)}% of full thrust: a reading is probably off (or the drone moved on the scale)`;
    return out;
  }

  // The thrust a fitted motor gives at throttle u and voltage V [N].
  const thrustAt = (f, u, V) => { const e = Math.max(0, u) * (V > 0.5 ? V : f.vref) / f.vref; return f.tmax * ((1 - f.bend) * e + f.bend * e * e); };

  // The throttle every motor needs together to hold weight W [N] at voltage V, from their fits; NaN past full throttle.
  function hoverThrottle(fits, W, V) {
    const total = u => fits.reduce((t, f) => t + thrustAt(f, u, V), 0);
    if (!fits.length || total(1) < W) return NaN;
    let lo = 0, hi = 1;
    for (let i = 0; i < 50; i++) { const m = (lo + hi) / 2; if (total(m) < W) lo = m; else hi = m; }
    return (lo + hi) / 2;
  }

  // The prop thrust coefficient that makes the physical motor match the readings: predict(ct, u, V) → thrust [N] (the
  // simulator's motor with that coefficient). Golden-section search on log ct within the card's bounds, least squares
  // over the readings with thrust. { ct, rms, rel, atBound } or null.
  function fitPhysics(points, predict, lo = 0.01, hi = 0.5, g = 9.80665) {
    const used = points.filter(p => Number.isFinite(p.grams) && p.u > 0 && Math.abs(p.grams) * g / 1000 >= MIN_N);
    if (used.length < 3) return null;
    const cost = ct => used.reduce((t, p) => t + (predict(ct, p.u, p.V) - Math.abs(p.grams) * g / 1000) ** 2, 0);
    let a = Math.log(lo), b = Math.log(hi); const r = (Math.sqrt(5) - 1) / 2;
    let c = b - r * (b - a), d = a + r * (b - a), fc = cost(Math.exp(c)), fd = cost(Math.exp(d));
    for (let i = 0; i < 60; i++) {
      if (fc < fd) { b = d; d = c; fd = fc; c = b - r * (b - a); fc = cost(Math.exp(c)); }
      else { a = c; c = d; fc = fd; d = a + r * (b - a); fd = cost(Math.exp(d)); }
    }
    const ct = Math.exp((a + b) / 2), rms = Math.sqrt(cost(ct) / used.length);
    const top = Math.max(...used.map(p => Math.abs(p.grams) * g / 1000));
    return { ct, rms, rel: rms / top, atBound: ct < lo * 1.02 || ct > hi / 1.02 };
  }

  return { LEVELS, MIN_N, MAX_REL, BEND, levelsUpTo, parseGrams, fitCurve, thrustAt, hoverThrottle, fitPhysics };
})();
if (typeof module !== 'undefined') module.exports = ThrustTest;
