'use strict';
// Tune → Gains from the motors (the maths: tune-suggest.js). What the design says about its motors (the
// slowest one's spin-up near hover, from its physical model: motorSpinUp; whether they were measured on the scale),
// an assumed ESC and loop delay, and from those how safe the gains are now and the quickest ones that stay safe
// however strong the motors really turn out to be. Apply sets them as the design's tuning (one undo step); they fly
// at once in the simulator, and reach the drone with its program (Computers → Install, Send this design).

const tsg = { lag: null, delay: 0.012 };   // lag: null = the motors' model; delay [s]: assumed
const TSG_LOOPS = [{ id: 'rp', label: 'Roll & pitch', axes: [0, 1] }, { id: 'yaw', label: 'Yaw', axes: [2] }];

// The model the suggestion is for: { lag, lagFrom, delay, measured, strengths }.
function tsgModel() {
  const ms = actuators(), lags = ms.map(c => ({ c, lag: motorSpinUp(c) })), slow = lags.reduce((a, b) => b.lag > a.lag ? b : a, lags[0] || { lag: 0.03 });
  const measured = ms.length > 0 && ms.every(c => motorMeasured(c));
  const lag = tsg.lag ?? slow.lag, spread = measured ? 0.02 : 0.06, alpha = tsgPush(spread);
  return { lag, lagFrom: tsg.lag == null ? (slow.c ? slow.c.name : '') : 'typed', delay: tsg.delay, measured, strengths: TuneSuggest.strengthsFor(measured),
    spread, alpha, firm: { rp: TuneSuggest.firmHz(alpha) } };
}
// The steady push [rad/s²] of one motor lifting spread (a fraction) more than the model says, hovering: its share of
// the weight times its lever about the centre of mass, over the inertia, the worst motor about the worse axis.
function tsgPush(spread) {
  const ms = actuators(); if (!ms.length || typeof model === 'undefined' || !model) return 0;
  const Th = model.m * G / ms.length; let a = 0;
  for (const c of ms) { const r = sub(rotorNow(c, restAngle).p, model.c); a = Math.max(a, spread * Th * Math.abs(r[1]) / model.J[0], spread * Th * Math.abs(r[0]) / model.J[4]); }
  return a;
}
const tsgNow = (t, L) => { const a = L.axes[0]; return [t.att.kR[a], t.att.kW[a], t.att.kI[a]]; };
const tsgFeel = g => { const f = TuneSuggest.feelOf(g); return `${f.hz.toFixed(2)} Hz, damping ${f.zeta.toFixed(2)}, integral ${f.integ.toFixed(2)}`; };

function tsgApply(sugs) {
  const n = tuneFix(tuneOf());
  for (const { L, s } of sugs) if (s && !s.keep) for (const a of L.axes) { n.att.kR[a] = s.gains[0]; n.att.kW[a] = s.gains[1]; n.att.kI[a] = s.gains[2]; }
  setTuning(n, 'suggest:' + Date.now());
  renderTuneSuggest(typeof liveOn === 'function' && liveOn() ? 'Set. On the real drone they go with its program: Computers → Install, Send this design (until its board restarts).' : 'Set: the simulator flies on them now. On the real drone they go with its program (Computers → Install, Send this design).');
}

function renderTuneSuggest(msg = '') {
  const box = $('#tuneSuggest'); if (!box || !actuators().length) { if (box) box.textContent = ''; return; }
  const fold = $('#tuneSuggestFold'); if (fold && !fold.open) { tsgSum(); return; }
  const m = tsgModel(), t = tuneOf(), focus = document.activeElement && box.contains(document.activeElement) ? document.activeElement.dataset.focusKey : null;
  box.textContent = '';
  box.append(el('p', { class: 'hint', text: 'Gains worked out from the motors alone, before flying: how long they take to change thrust, the delay before a command reaches them, and how sure the design is of their strength. Too quick a response for slow motors is what overshoots and then shakes harder and harder.' }));
  // the model: spin-up (from the motors, or typed), delay (assumed), strength (measured or described)
  const lagIn = el('input', { type: 'number', class: 'num', min: 3, max: 500, step: 1, inputmode: 'decimal', 'aria-label': 'Motor spin-up time', 'data-focus-key': 'tsg-lag', value: Math.round(m.lag * 1000) });
  lagIn.addEventListener('change', () => { const v = parseFloat(lagIn.value); tsg.lag = Number.isFinite(v) && v >= 3 && v <= 500 ? v / 1000 : null; renderTuneSuggest(); });
  const delIn = el('input', { type: 'number', class: 'num', min: 0, max: 100, step: 1, inputmode: 'decimal', 'aria-label': 'ESC and loop delay', 'data-focus-key': 'tsg-delay', value: Math.round(m.delay * 1000) });
  delIn.addEventListener('change', () => { const v = parseFloat(delIn.value); if (Number.isFinite(v) && v >= 0 && v <= 100) tsg.delay = v / 1000; renderTuneSuggest(); });
  box.append(el('div', { class: 'tt-setup' },
    el('label', { class: 'field' }, el('span', { text: 'Motor spin-up (ms)' }), lagIn),
    el('label', { class: 'field' }, el('span', { text: 'ESC + loop delay (ms)' }), delIn),
    tsg.lag != null ? UI.button({ class: 'btn sm', type: 'button', 'data-focus-key': 'tsg-lag-reset', text: 'From the motors', onclick: () => { tsg.lag = null; renderTuneSuggest(); } }) : null));
  box.append(el('p', { class: 'hint', text: [
    tsg.lag == null ? `Spin-up: the slowest motor near hover (${m.lagFrom}), from its physical model (KV, resistance, rotor inertia, prop).` : 'Spin-up: as typed.',
    'Delay: a control step, the ESC\'s pulse and the gyro\'s filter, assumed.',
    m.measured ? `Strength: measured on the scale, so known within ${Math.round((1 - m.strengths[0]) * 100)}%.` : `Strength: only described on the motor cards, so the real motors may be anywhere from ${m.strengths[0]}× to ${m.strengths[2]}× as strong; the gains must keep a margin across all of it (Motor thrust measures it).`,
    m.firm.rp ? `Roll and pitch must also be firm enough (${m.firm.rp.toFixed(2)} Hz or more) to hold a motor lifting ${Math.round(m.spread * 100)}% more than the others within 15° until the integral trims it.` : '',
  ].join(' ') }));
  const sugs = TSG_LOOPS.map(L => { const now = tsgNow(t, L); return { L, now, j: TuneSuggest.judge(now, m), s: TuneSuggest.suggest(L.id, now, m) }; });
  const list = el('div', { class: 'tsg-loops' });
  for (const { L, now, j, s } of sugs) {
    const hold = g => L.id === 'rp' && m.alpha > 0 ? `; a ${Math.round(m.spread * 100)}% motor mismatch tips it ${Math.round(m.alpha / g[0] * 180 / Math.PI)}° until the integral trims it` : '';
    const deg = x => x == null ? 'no' : `${Math.round(x)}°`, margins = (x, g) => `${deg(x.nominal)} margin as described, ${deg(x.margin)} at worst${g ? hold(g) : ''}`;
    const nm = j.nominal ?? -90, nowTone = j.safe ? 'ok' : nm < 15 ? 'bad' : 'warn';
    const why = nm < 15 ? 'expect it to overshoot and shake' : nm < 30 || (j.margin ?? -90) < 20 ? 'bouncy, little room for noise, wind or motors stronger than described'
      : !j.integralOk ? 'its integral is too quick for this response: a slow swing after each push'
      : j.peak > 1.35 ? `it overshoots (peaks ${j.peak.toFixed(2)}× in its response)` : 'it magnifies disturbances near its own speed';
    const nowText = j.safe ? `within the limits: ${margins(j, now)}` : `${margins(j)} (${j.at}× strength)${hold(now)}: ${why}`;
    const sugText = !s ? 'Nothing keeps a margin: motors this slow (or this much delay) can\'t be held with these laws. Check the spin-up and the delay.'
      : s.keep ? 'Keep these: within the limits, and firm enough.' : `Suggested: ${tsgFeel(s.gains)} · ${margins(s, s.gains)}. Softer holds less firmly: in gusts it wanders more, so fly it gently at first.`;
    list.append(el('div', { class: 'tt-motor tsg-loop' },
      el('div', { class: 'scal-pose-h' }, el('b', { text: L.label })),
      el('p', { class: 'hint' }, el('span', { text: 'Now: ' + tsgFeel(now) + ' · ' }), el('span', { class: 'ui-status ' + nowTone, text: nowText })),
      el('p', { class: 'hint' + (s ? '' : ' warn'), text: sugText }),
      s && s.firm ? el('p', { class: 'hint warn', text: `A compromise: ${s.safeHz ? `gains up to ${s.safeHz.toFixed(2)} Hz would keep the full margin` : 'no gains keep the full margin'}, but that soft it can't hold motors that differ by ${Math.round(m.spread * 100)}%: a lift-off's push would tip it over (${m.firm.rp.toFixed(2)} Hz is the least that keeps it within 15°). ${m.measured ? 'Quicker motors or less delay would help.' : 'Measuring the motors (Motor thrust) narrows the strength range and buys margin.'}` }) : null));
  }
  box.append(list);
  if (msg) box.append(el('p', { class: 'ui-status ok', role: 'status', text: msg }));
  const can = sugs.some(x => x.s && !x.s.keep);
  box.append(el('div', { class: 'design-tools' }, UI.button({ class: 'btn sm primary', type: 'button', 'data-focus-key': 'tsg-apply', disabled: !can || undefined, text: 'Use the suggested gains', title: 'Set Roll & pitch and Yaw to the suggestion (one undo step)', onclick: () => tsgApply(sugs) })));
  tsgSum(sugs);
  if (focus) { const f = box.querySelector(`[data-focus-key="${focus}"]`); if (f && !f.disabled) f.focus(); }
}
// The fold's summary: how the current gains do.
function tsgSum(sugs) {
  const sum = $('#tuneSuggestSum'); if (!sum) return;
  if (!sugs) { const m = tsgModel(), t = tuneOf(); sugs = TSG_LOOPS.map(L => ({ j: TuneSuggest.judge(tsgNow(t, L), m) })); }
  const worst = sugs.reduce((a, b) => (b.j.nominal ?? -90) < (a.j.nominal ?? -90) ? b : a);
  setText(sum, sugs.every(x => x.j.safe) ? 'gains within the limits' : `now ${worst.j.nominal == null ? 'no' : Math.round(worst.j.nominal) + '°'} margin`);
}
document.addEventListener('toggle', e => { if (e.target && e.target.id === 'tuneSuggestFold' && e.target.open) renderTuneSuggest(); }, true);
