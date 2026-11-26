'use strict';
// Tune → Gains from the motors: roll, pitch and yaw gains worked out from what the design knows about its motors,
// before any flight (tune-suggest-ui.js shows them). The same loop model and safety test as measured autotune
// (autotune.js: atPlant, atMargins): each attitude axis is an inertia with the controller's gains in units of angular
// acceleration, so what's left of the plant is
//   strength × e^(−s·delay) / (s² (1 + s·lag))
// lag: the slowest motor's spin-up time constant at hover (from its physical model: inertia over the motor's and the
// prop's damping), delay: the ESC and the control loop's own (a step of the loop, the ESC's pulse, the gyro's filter),
// strength: how much stronger the real motors are than the model says (1 when they match). A motor measured on the
// scale is known within a few percent; one only described on its card may be off by half (a 1.25 N guess for a
// 0.9 N motor, or the other way round), and too strong is what makes a loop overshoot and shake.
//
// The suggestion is the quickest response (damping 0.8–1.1; the integral no faster than now, at most 0.8 /s, and
// slower where it must be) that keeps a working margin: phase margin ≥ 30° with the motors as described (closed-loop
// peak ≤ 1.35, sensitivity ≤ 2.5) and still ≥ 20° at either end of the strength range, the integral well inside its
// bound. Not more: softer gains keep more margin but hold the drone less firmly against wind and motors that don't
// quite match (in the simulator, 80 ms motors fly better on 15° than on 35°), so it also has a floor (firmHz).
// It also says how the current gains do on the same model.

const TuneSuggest = (() => {
  const RANGE = { measured: [0.85, 1.15], guessed: [0.7, 1.5] };
  const LOOPS = { rp: { hz: [0.2, 4], axis: 0 }, yaw: { hz: [0.2, 3], axis: 2 } };
  const gainsOf = (hz, zeta, integ) => { const wn = 2 * Math.PI * hz; return [wn * wn, 2 * zeta * wn, wn * wn * integ]; };
  const feelOf = g => { const wn = Math.sqrt(g[0]); return { hz: wn / (2 * Math.PI), zeta: g[1] / (2 * wn), integ: g[2] / g[0] }; };

  const NOMINAL = { margin: 30, peak: 1.35, sensitivity: 2.5 }, EDGE = 20;
  // The gains [P, D, I] on a model { lag, delay, strengths }: { margin (the worst over the strengths, °; null: no
  // crossover), nominal (the margin with the motors as described), peak, sensitivity (as described), at (the strength
  // that was worst), integralOk, safe, score (how far inside the limits, ° : negative outside) }.
  function judge(gains, model) {
    const at = s => atMargins(gains, { gain: s, lag: model.lag, delay: model.delay, lead: 0 });
    const nom = at(1); let worst = { margin: nom.margin, at: 1 };
    for (const s of model.strengths) if (s !== 1) { const m = at(s); if ((m.margin ?? -90) < (worst.margin ?? -90)) worst = { margin: m.margin, at: s }; }
    const integralOk = gains[2] < gains[0] * gains[1] * 0.35, nm = nom.margin ?? -90, wm = worst.margin ?? -90;
    const safe = integralOk && nm >= NOMINAL.margin && nom.peak <= NOMINAL.peak && nom.sensitivity <= NOMINAL.sensitivity && wm >= EDGE;
    return { margin: worst.margin, at: worst.at, nominal: nom.margin, peak: nom.peak, sensitivity: nom.sensitivity, integralOk, safe,
      score: Math.min(nm - NOMINAL.margin, wm - EDGE, (NOMINAL.peak - nom.peak) * 100, (NOMINAL.sensitivity - nom.sensitivity) * 20) };
  }
  // The firmest response a loop needs [Hz]: stiff enough that a steady push of alpha [rad/s²] (motors that don't
  // quite match: one a few percent stronger than the others) tips it no more than tilt [rad] before the integral
  // trims it out. Too soft, and a lift-off's push tips the drone over before it catches it.
  const firmHz = (alpha, tilt = 15 * Math.PI / 180) => alpha > 0 ? Math.sqrt(alpha / tilt) / (2 * Math.PI) : 0;
  // The gains for one loop ('rp' or 'yaw'); now: its current gains [P, D, I]; model.firm: { rp, yaw } the firmest
  // response each needs [Hz]. The quickest safe gains; or, when those are softer than it needs (or none are safe),
  // the gains at that firmness with the most margin, marked firm (with safeHz: the quickest safe one, or null).
  // Gains that already keep the margin and are firm enough stay as they are (keep: true): this never speeds a loop up.
  function suggest(loop, now, model) {
    const floor = model.firm ? model.firm[loop] || 0 : 0, jn = judge(now, model), fn = feelOf(now);
    if (jn.safe && fn.hz >= floor - 1e-9) return { ...fn, gains: now.slice(), ...jn, keep: true };
    const best = quickestSafe(loop, now, model);
    if (!floor || (best && best.hz >= floor - 1e-9)) return best;
    const hz = Math.ceil(floor / 0.025 - 1e-9) * 0.025, top = Math.min(feelOf(now).integ, 0.8);
    let firm = null;
    for (const integ of [top, 0.5, 0.3, 0.15].filter(x => x <= top + 1e-9)) for (const zeta of [0.8, 0.9, 1, 1.1]) {
      const g = gainsOf(hz, zeta, integ), j = judge(g, model);
      if (!(g[2] < g[0] * g[1] * 0.35)) continue;                    // (its integral stays inside its bound)
      if (!firm || j.score > firm.score + 1e-9 || (Math.abs(j.score - firm.score) < 1e-9 && integ > firm.integ)) firm = { hz, zeta, integ, gains: g, ...j };
    }
    return firm ? { ...firm, firm: true, safeHz: best ? best.hz : null } : best;
  }
  function quickestSafe(loop, now, model) {
    const L = LOOPS[loop], top = Math.min(feelOf(now).integ, 0.8), integs = [top, 0.5, 0.3, 0.15, 0].filter((x, i, a) => x <= top + 1e-9 && a.indexOf(x) === i);
    let best = null;
    // the quickest response; at the same speed the faster integral, then the larger margin
    const top2 = Math.min(L.hz[1], feelOf(now).hz);                   // (never quicker than now)
    for (let hz = L.hz[0]; hz <= top2 + 1e-9; hz += 0.025) for (const integ of integs) for (const zeta of [0.8, 0.9, 1, 1.1]) {
      const g = gainsOf(hz, zeta, integ), j = judge(g, model);
      const better = !best || hz > best.hz + 1e-9 || (Math.abs(hz - best.hz) < 1e-9 && (integ > best.integ + 1e-9 || (Math.abs(integ - best.integ) < 1e-9 && j.margin > best.margin)));
      if (j.safe && better) best = { hz, zeta, integ, gains: g, ...j };
    }
    return best;
  }
  // The model's strength range: measured on the scale (every motor) or only described.
  const strengthsFor = measured => { const r = measured ? RANGE.measured : RANGE.guessed; return [r[0], 1, r[1]]; };
  return { RANGE, LOOPS, gainsOf, feelOf, judge, suggest, quickestSafe, firmHz, strengthsFor };
})();
if (typeof module !== 'undefined') module.exports = TuneSuggest;
