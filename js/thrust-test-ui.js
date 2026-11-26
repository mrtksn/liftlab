'use strict';
// Tune → Motor thrust: each motor's thrust against its throttle, read off a kitchen scale (the maths: thrust-test.js).
// One motor at a time spins at a few throttles, the lowest first; at each, the scale's reading is typed in and Next
// goes on. The motor spins only while this page keeps asking (a motor test lapses half a second after the last ask):
// on the drone as dfb_pi's "motors T1 T2 ..." text, sent up the radio like the drone card's commands; in the
// simulator straight to its flight core (simBenchSet), with its scale read for you (simScaleGrams).
// Apply keeps it in the design: each motor's record on its card (thrustTest: the readings, its full thrust and curve
// bend), the prop thrust coefficient that makes the simulated motor match (a generic motor: its max thrust), so the
// flight controller's model (its thrust and curve, Upload design for the drone) and the simulator both become these
// motors.

const tt = { zero: null, top: 1, runs: {}, cur: null, msg: '', tone: '', applied: false };
// tt.runs[i]: { points: [{ u, V, grams }] } (grams: reading − motors-off); tt.cur: { i, k, at } the motor spinning, the level
const ttWhere = () => typeof liveOn === 'function' && liveOn() ? (live.state === 'up' ? 'live' : 'down') : 'sim';
const ttMotors = () => actuators();
const ttLevels = () => ThrustTest.levelsUpTo(tt.top);
const ttNote = (msg, tone = '') => { tt.msg = msg; tt.tone = tone; renderThrustTest(); };
const ttFit = i => { const r = tt.runs[i]; return r && r.points.length ? ThrustTest.fitCurve(r.points, designVref()) : null; };
const TT_HOLD_S = 45;   // a level held this long without a reading: the motor stops (Spin again restarts it)

// The motors' pack voltage now: the power monitor's channel on it, else the battery reading (the drone); the pack (the
// simulator). 0: unknown (the fit takes the reference voltage).
function ttVolts() {
  if (ttWhere() === 'sim') return S.battV || 0;
  const v = gs.v || {}, p = v.power;
  if (p) for (const k of [1, 2, 3]) if (p['use' + k] === 1 && p['v' + k] > 0.5) return p['v' + k];
  return v.battery && v.battery.volts > 0.5 ? v.battery.volts : 0;
}
// The drone's state: 0 disarmed, 1 armed, 2 failsafe, 3 crashed, 4 motor test (fc_core.h), or -1 unknown.
const ttFcState = () => ttWhere() === 'sim' ? brt.fcState : gs.v && gs.v.state ? gs.v.state.state : -1;

// Ask for motor i at throttle u (or every motor off: i < 0). '' when asked, else why not.
function ttSpin(i, u) {
  if (ttWhere() === 'sim') return simBenchSet(i, u);
  if (ttWhere() === 'down') return 'not connected to the drone (Ground tab)';
  if (typeof cardRadioPi !== 'function' || !cardRadioPi()) return 'the motor test needs the Pi (dfb_pi) on the radio\'s end';
  const n = ttMotors().length, vals = Array.from({ length: n }, (_, k) => k === i ? u.toFixed(3) : '0');
  return cardTextOut(i < 0 ? 'motors off' : 'motors ' + vals.join(' '));
}
let ttTimer = 0, ttSeen = 0;
function ttTick() {
  const c = tt.cur; if (!c || !c.spinning) return;
  if ((performance.now() - c.at) / 1000 > TT_HOLD_S) { ttHalt(`No reading for ${TT_HOLD_S} s: the motor is stopped. Spin again to go on.`, 'warn'); return; }
  const why = ttSpin(c.i, ttLevels()[c.k]);
  if (why && !/too many commands/.test(why)) { ttHalt(why, 'bad'); return; }
  if (ttWhere() !== 'sim') {   // the drone's refusals (an older dfb_pi: "the radio's pilot has it"; armed)
    const k = Math.min(gs.log.length, (gs.nmsg || 0) - ttSeen); ttSeen = gs.nmsg || 0;
    const bad = gs.log.slice(0, Math.max(0, k)).find(m => /^motors?: /.test(String(m.text || '').trim()));
    if (bad) { ttHalt('The drone said: ' + bad.text.trim(), 'bad'); return; }
  }
}
function ttHalt(msg, tone) {
  if (tt.cur) tt.cur.spinning = false;
  clearInterval(ttTimer); ttTimer = 0; ttSpin(-1, 0);
  if (msg != null) ttNote(msg, tone); else renderThrustTest();
}
function ttStartLevel() {
  const c = tt.cur; if (!c) return;
  const st = ttFcState();
  if (st === 1 || st === 2) { ttNote('It\'s armed: disarm it first.', 'bad'); return; }
  const why = ttSpin(c.i, ttLevels()[c.k]);
  if (why) { ttNote(why, 'bad'); return; }
  c.spinning = true; c.at = performance.now(); ttSeen = gs.nmsg || 0;
  clearInterval(ttTimer); ttTimer = setInterval(ttTick, 250);
  ttNote(`${ttMotors()[c.i].name} at ${Math.round(ttLevels()[c.k] * 100)}%: wait for the reading to settle, then type it in.`);
}
function ttBegin(i) {
  if (tt.zero == null) { ttNote('Weigh it with the motors off first.', 'bad'); return; }
  if (tt.cur) ttHalt(null);
  tt.runs[i] = { points: [] }; tt.applied = false;
  tt.cur = { i, k: 0, spinning: false, at: 0 };
  ttStartLevel();
}
function ttRecord(grams) {
  const c = tt.cur; if (!c || !c.spinning) return;
  tt.runs[c.i].points.push({ u: ttLevels()[c.k], V: +ttVolts().toFixed(3), grams: grams - tt.zero });
  if (c.k + 1 < ttLevels().length) { c.k++; c.at = performance.now(); ttSpin(c.i, ttLevels()[c.k]); ttNote(`Recorded. ${ttMotors()[c.i].name} now at ${Math.round(ttLevels()[c.k] * 100)}%: wait for it to settle.`); return; }
  ttHalt(null); tt.cur = null;
  const f = ttFit(c.i), next = ttMotors().findIndex((m, j) => !tt.runs[j]);
  ttNote(f && !f.error ? `${ttMotors()[c.i].name} done${f.warn ? ' (' + f.warn + ')' : ''}.${next >= 0 ? ` Next: ${ttMotors()[next].name}.` : ' All motors measured: Apply keeps them.'}` : `${ttMotors()[c.i].name}: ${f ? f.error : 'no readings'}`, f && !f.error && !f.warn ? 'ok' : 'warn');
}
// The simulated scale, once the motor has settled (half a second of simulated time at this throttle: the simulator
// may run slower than the clock).
async function ttSimRead() {
  const t0 = performance.now();
  const want = tt.cur ? ttLevels()[tt.cur.k] : -1, settled = () => brt.bench && Math.abs(brt.bench.u - want) < 1e-6 && simBenchAge() >= 0.5;
  while (!settled() && performance.now() - t0 < 20000) await new Promise(r => setTimeout(r, 100));
  return simScaleGrams();
}
function ttStop() { ttHalt('Stopped: the motor is off.', ''); tt.cur = null; renderThrustTest(); }

// Apply: each measured motor's record, and the card fitted to it (one undo step).
function ttApply() {
  const ms = ttMotors(), vref = designVref(), rho = flightAtmosphere().rho, out = [];
  for (let i = 0; i < ms.length; i++) {
    const c = ms[i], f = ttFit(i); if (!f || f.error) continue;
    const rec = { at: Date.now(), vref, tmax: +f.tmax.toFixed(4), bend: +f.bend.toFixed(3), rms: +f.rms.toFixed(4), points: tt.runs[i].points.map(p => ({ ...p })), ct: null };
    const kind = c.motorPhysics?.kind;
    if (c.propPhysics?.rows) { out.push(`${c.name}: kept, but it has a measured prop table, which stays its thrust`); c.thrustTest = rec; continue; }
    if (kind === 'brushless' || kind === 'brushed') {
      const predict = (ct, u, V) => FlightPhysics.equilibrium(FlightPhysics.motor({ ...c, propPhysics: { ...c.propPhysics, ct } }, vref), u * flightDuty(c), V > 0.5 ? V : vref, rho).T;
      const ph = ThrustTest.fitPhysics(tt.runs[i].points, predict);
      if (!ph) continue;
      rec.ct = +ph.ct.toFixed(5); c.propPhysics = { ...c.propPhysics, ct: rec.ct };
      out.push(`${c.name}: prop coefficient ${rec.ct}${ph.atBound ? ' (at its limit: KV or resistance on the card may be off)' : ''}, matches within ${(ph.rel * 100).toFixed(0)}%`);
    } else { c.tmax = rec.tmax; out.push(`${c.name}: max thrust ${(rec.tmax / G * 1000).toFixed(0)} g`); }
    c.thrustTest = rec;
  }
  if (!out.length) { ttNote('Nothing to apply: measure a motor first.', 'bad'); return; }
  undoKey = 'thrust-test:' + Date.now();
  recomputeProps(); cPts = contactPoints(); rebuildDrone(); refreshEnvelope(); renderMass();
  for (const c of ms) refreshCard(c);
  buildActRows(); save();
  if (typeof flightRerenderParts === 'function') flightRerenderParts();
  tt.applied = true;
  ttNote(`Saved in the design. ${out.join('; ')}.` + (ttWhere() === 'live' ? ' Upload design puts the new model on the drone.' : ' The simulator flies on it now; on the real drone, Upload design (connected) puts it there.'), 'ok');
}
const ttUploading = () => typeof cardUp !== 'undefined' && cardUp.on;
async function ttUpload() {
  if (ttUploading()) return;
  const why = typeof cardUploadStart === 'function' ? cardUploadStart() : 'no drone card';
  if (why) { ttNote(why, 'bad'); return; }
  while (ttUploading()) { tt.msg = cardUp.said || 'Uploading the design…'; tt.tone = ''; renderThrustTest(); await new Promise(r => setTimeout(r, 300)); }
  const ok = card.msgTone !== 'bad' && card.msgTone !== 'warn';
  ttNote(card.msg ? (ok ? '✓ ' : '') + card.msg : 'Upload design finished.', ok ? 'ok' : 'bad');
}

const ttG = n => `${(n / G * 1000).toFixed(0)} g`;
function renderThrustTest() {
  const box = $('#thrustTest'); if (!box) return;
  const ms = ttMotors(), where = ttWhere(), c = tt.cur;
  const done = ms.filter((m, i) => { const f = ttFit(i); return f && !f.error; }).length;
  setText($('#thrustTestSum'), tt.applied ? 'applied' : c ? 'testing' : done ? `${done} of ${ms.length} motors` : ms.some(m => motorMeasured(m)) ? 'measured' : '');
  const focus = document.activeElement && box.contains(document.activeElement) ? document.activeElement.dataset.focusKey : null;
  box.textContent = '';
  box.append(el('p', { class: 'hint', text: 'Measures how much each motor really lifts at each throttle, on a kitchen scale, so the flight controller and the simulator know these motors instead of guessing. One motor at a time, the lowest throttle first.' }));
  if (!ms.length) { box.append(el('p', { class: 'hint warn', text: 'This design has no motors.' })); return; }
  box.append(el('p', { class: 'hint' + (where === 'live' ? ' warn' : where === 'down' ? ' warn' : ''), text: where === 'live'
    ? 'Props ON: this spins them. Upside down on the scale (the thrust presses it down) or the right way up and weighted down; either way tape it so it can\'t tip or lift, keep the props clear of the scale and everything else, hands away. Stop (or Esc) cuts the motor; it also stops by itself half a second after this page stops asking.'
    : where === 'down' ? 'Not connected to the drone yet (Ground tab). In the simulator (disconnected) it runs on the simulated drone and reads its scale for you.'
    : 'In the simulator: the drone sits on a simulated scale, the right way up, and Read the scale fills in its reading. On the real drone (connected) you read your own scale.' }));
  // setup: the highest throttle, the motors-off reading
  const top = UI.select({ 'aria-label': 'Highest throttle', 'data-focus-key': 'tt-top', disabled: !!c || undefined });
  for (const v of [0.5, 0.65, 0.8, 1]) top.append(el('option', { value: v, text: Math.round(v * 100) + '%' }));
  top.value = tt.top; top.addEventListener('change', () => { tt.top = +top.value; renderThrustTest(); });
  const zeroIn = el('input', { type: 'text', inputmode: 'decimal', 'aria-label': 'Scale reading with the motors off', 'data-focus-key': 'tt-zero', placeholder: 'grams', value: tt.zero ?? '', size: 7 });
  const setZero = v => { const g = ThrustTest.parseGrams(v); if (g == null) { ttNote('Type the reading in grams (e.g. 312).', 'bad'); return; } tt.zero = g; tt.runs = {}; tt.applied = false; ttNote(`Motors off: ${g.toFixed(0)} g. Now test each motor.`); };
  // the simulator: the drone put down on its scale (reset, if it's up) and kept there, disarmed, then weighed
  const simZero = async () => {
    const was = brt.benchHold; simBenchHold(true);
    if (!was) $('#resetBtn').click();   // (a fresh start on the ground; held from then on) ttNote('Putting it on the scale…');
    await new Promise(r => setTimeout(r, 1500));
    if (brt.fcState === 1 || brt.fcState === 2) { ttNote('It armed before it was held: try Read the scale again.', 'bad'); return; }
    setZero(String(simScaleGrams()));
  };
  zeroIn.addEventListener('change', () => setZero(zeroIn.value));
  box.append(el('div', { class: 'tt-setup' },
    el('label', { class: 'field' }, el('span', { text: 'Highest throttle' }), top),
    el('label', { class: 'field' }, el('span', { text: 'Motors off' }), zeroIn),
    where === 'sim' ? UI.button({ class: 'btn sm', type: 'button', 'data-focus-key': 'tt-zero-read', disabled: !!c || undefined, text: 'Read the scale', onclick: simZero }) : null));
  // the motors
  const list = el('ol', { class: 'tt-motors' });
  ms.forEach((m, i) => {
    const f = ttFit(i), on = c && c.i === i, meas = motorMeasured(m);
    const btn = UI.button({ class: 'btn sm', type: 'button', 'data-focus-key': 'tt-m' + i, disabled: !!c || tt.zero == null || where === 'down' || undefined, text: tt.runs[i] ? 'Again' : 'Test', onclick: () => ttBegin(i) });
    const li = el('li', { class: 'tt-motor' + (f && !f.error ? ' is-done' : '') + (on ? ' is-on' : '') },
      el('div', { class: 'scal-pose-h' }, el('b', { text: m.name }), el('small', { text: `card: ${ttG(m.tmax)} at ${designVref()} V${meas ? ', measured' : ''}` }), btn));
    if (f && f.error && !on) li.append(el('p', { class: 'hint warn', text: f.error }));
    else if (f && !f.error) {
      li.append(el('p', { class: 'hint scal-got', text: `✓ full thrust ${ttG(f.tmax)} at ${designVref()} V (card ${ttG(m.tmax)}), curve bend ${f.bend.toFixed(2)}, readings within ${ttG(f.rms)}` }));
      if (f.warn) li.append(el('p', { class: 'hint warn', text: f.warn }));
    }
    if (on) {
      const u = ttLevels()[c.k], inp = el('input', { type: 'text', inputmode: 'decimal', 'aria-label': 'Scale reading', 'data-focus-key': 'tt-read', placeholder: 'grams', size: 7 });
      const next = () => { const g = ThrustTest.parseGrams(inp.value); if (g == null) { ttNote('Type the reading in grams.', 'bad'); return; } ttRecord(g); };
      inp.addEventListener('keydown', e => { if (e.key === 'Enter') { e.preventDefault(); next(); } });
      li.append(el('div', { class: 'tt-now' },
        el('span', { text: `${c.spinning ? 'Spinning at' : 'Stopped at'} ${Math.round(u * 100)}% (${c.k + 1} of ${ttLevels().length})` }), inp,
        where === 'sim' ? UI.button({ class: 'btn sm', type: 'button', 'data-focus-key': 'tt-sim-read', text: 'Read the scale', disabled: !c.spinning || undefined, onclick: async () => { inp.value = String(await ttSimRead()); } }) : null,
        UI.button({ class: 'btn sm primary', type: 'button', 'data-focus-key': 'tt-next', disabled: !c.spinning || undefined, text: 'Next', onclick: next }),
        c.spinning ? null : UI.button({ class: 'btn sm', type: 'button', 'data-focus-key': 'tt-again', text: 'Spin again', onclick: ttStartLevel }),
        UI.button({ class: 'btn sm', type: 'button', 'data-focus-key': 'tt-stop', text: 'Stop', onclick: ttStop })));
      if (!focus || focus === 'tt-read' || focus === 'tt-next' || focus.startsWith('tt-m')) setTimeout(() => inp.isConnected && inp.focus(), 0);
    }
    list.append(li);
  });
  box.append(list);
  if (tt.msg) box.append(el('p', { class: 'ui-status' + (tt.tone ? ' ' + tt.tone : ''), role: 'status', text: tt.msg }));
  // what it means: the hover throttle, measured against what the card says
  const fits = ms.map((m, i) => ttFit(i));
  if (fits.every(f => f && !f.error)) {
    const W = truthOrModelWeight(), V = ttVolts() || designVref();
    const h = ThrustTest.hoverThrottle(fits, W, V);
    const hCard = ThrustTest.hoverThrottle(ms.map(m => ({ tmax: m.tmax, bend: curveHat(m), vref: designVref() })), W, V);
    box.append(el('p', { class: 'hint', text: `Hovering at ${(W / G * 1000).toFixed(0)} g on ${V.toFixed(2)} V takes ${Number.isFinite(h) ? Math.round(h * 100) + '% throttle' : 'more than full throttle'} by these readings; the card's model says ${Number.isFinite(hCard) ? Math.round(hCard * 100) + '%' : 'more than full throttle'}.${Number.isFinite(h) && Number.isFinite(hCard) && Math.abs(h - hCard) > 0.05 ? ' The difference is how far off the controller\'s corrections have been.' : ''}` }));
  }
  const tools = el('div', { class: 'design-tools' },
    UI.button({ class: 'btn sm', type: 'button', 'data-focus-key': 'tt-apply', disabled: !!c || !done || tt.applied || undefined, text: 'Apply', title: 'Keep the measured thrust in the design: the motor cards, the controller\'s model and the simulator', onclick: ttApply }),
    UI.button({ class: 'btn sm', type: 'button', 'data-focus-key': 'tt-clear', disabled: !!c || (!done && tt.zero == null) || undefined, text: 'Start over', onclick: () => { tt.runs = {}; tt.zero = null; tt.applied = false; if (ttWhere() === 'sim') simBenchHold(false); ttNote(''); } }));
  if (tt.applied && where === 'live' && typeof cardRadioPi === 'function' && cardRadioPi()) tools.append(UI.button({ class: 'btn sm', type: 'button', 'data-focus-key': 'tt-upload', disabled: ttUploading() || undefined, text: ttUploading() ? 'Uploading…' : 'Upload design', onclick: ttUpload }));
  box.append(tools);
  if (focus) { const f = box.querySelector(`[data-focus-key="${focus}"]`); if (f && !f.disabled) f.focus(); }
}
// The drone's weight as the design has it [N] (what it hovers with).
const truthOrModelWeight = () => (typeof model !== 'undefined' && model && model.m ? model.m : cfg.frame.mass) * G;

// Esc stops a spinning motor; so do closing the section, leaving the page and the link going down.
document.addEventListener('keydown', e => { if (e.key === 'Escape' && tt.cur && tt.cur.spinning) ttStop(); });
document.addEventListener('visibilitychange', () => { if (document.hidden && tt.cur && tt.cur.spinning) ttHalt('Stopped: the page was hidden.', 'warn'); });
document.addEventListener('toggle', e => { if (e.target && e.target.id === 'thrustTestFold') { if (e.target.open) renderThrustTest(); else { if (tt.cur) ttStop(); if (typeof simBenchHold === 'function') simBenchHold(false); } } }, true);
{ let seen = ''; setInterval(() => { const f = $('#thrustTestFold'); if (!f || !f.open) return; const w = ttWhere(); if (w !== seen) { if (seen && tt.cur && tt.cur.spinning) ttHalt('Stopped: the connection changed.', 'warn'); seen = w; renderThrustTest(); } }, 1000); }
