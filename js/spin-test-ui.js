'use strict';
// Tune → Rope spin test: how quickly the motors really answer, with the drone hanging on a rope, free to turn
// (runner/fc/spin_core.c: the sequence and the fit, on the Pi as dfb_pi's "spintest", in the simulator on the
// learning's board: simSpinStart). Every motor at a base throttle below lift; the two spin directions stepped against
// each other in doublets; the yaw rate fitted for the delay before the motors change and their spin-up time.
// Apply keeps them in the design (cfg.motorTiming), where Gains from the motors and the step predictions use them.

const spt = { base: 0.3, step: 0.05, half: 0.25, phase: 'idle', text: '', result: null, msg: '', tone: '', timer: 0, applied: false };
const sptWhere = () => typeof liveOn === 'function' && liveOn() ? (live.state === 'up' ? 'live' : 'down') : 'sim';
const sptNote = (msg, tone = '') => { spt.msg = msg; spt.tone = tone; renderSpinTest(); };

// What the test says ("spintest: ...", dfb_pi's or the simulator's own): { phase: run | done | failed | idle, text, result }.
function sptParse(text) {
  const s = String(text || '').replace(/^esp32:\s*/, '').trim(); if (!/^spintest:/.test(s)) return null;
  const m = /result (\d+) (\d+) (-?[\d.]+) (-?[\d.]+) (\d) ([\d.]+)(?: (-?[\d.]+))?/.exec(s);
  const result = m ? { delay: +m[1] / 1000, lag: +m[2] / 1000, gain: +m[3], r2: +m[4], ok: m[5] === '1', peak: +m[6], kick: m[7] != null ? +m[7] : null } : null;
  const phase = /settling|stepping/.test(s) ? 'run' : /\bdone\b/.test(s) ? 'done' : /stopped|needs|disarm|only in|no airframe|already|base 0\.05/.test(s) ? 'failed' : 'idle';
  return { phase, text: s.replace(/^spintest:\s*/, '').replace(/;\s*result .*$/, ''), result };
}
// The drone's newest "spintest:" line since n0 (live), or the simulator's.
function sptRead(n0) {
  if (sptWhere() === 'sim') { const st = simSpinStatus(); return st ? sptParse(st.text) : null; }
  const k = Math.min(gs.log.length, (gs.nmsg || 0) - n0);
  for (const m of gs.log.slice(0, Math.max(0, k))) { const p = sptParse(m.text); if (p) return p; }
  return null;
}
function sptPoll() {
  if (spt.phase !== 'run') { clearInterval(spt.timer); spt.timer = 0; return; }
  const n0 = spt.n0 ?? 0, p = sptRead(n0);
  if (p) { spt.text = p.text; if (p.phase !== 'run' && p.phase !== 'idle') { spt.phase = p.phase; spt.result = p.result; clearInterval(spt.timer); spt.timer = 0; spt.msg = ''; } }
  if (sptWhere() === 'live' && spt.phase === 'run') { spt.n0 = gs.nmsg || 0; cardTextOut('spintest'); }   // (ask again: the answer comes in the log)
  if (sptWhere() === 'down') { spt.phase = 'failed'; spt.text = 'the connection went down'; }
  renderSpinTest();
}
function sptStart() {
  const w = sptWhere(); spt.result = null; spt.applied = false; spt.text = ''; spt.msg = '';
  if (w === 'down') { sptNote('Not connected to the drone (Ground tab).', 'bad'); return; }
  if (w === 'sim') {
    const why = simSpinStart(spt.base, spt.step, spt.half); if (why) { sptNote(why, 'bad'); return; }
    if (!running) { running = true; if (typeof renderRun === 'function') renderRun(); }   // (the simulation runs the test)
  } else {
    spt.n0 = gs.nmsg || 0;
    const why = cardTextOut(`spintest ${spt.base.toFixed(3)} ${spt.step.toFixed(3)} ${spt.half.toFixed(2)}`); if (why) { sptNote(why, 'bad'); return; }
  }
  spt.phase = 'run'; spt.text = 'starting…';
  clearInterval(spt.timer); spt.timer = setInterval(sptPoll, w === 'sim' ? 300 : 1000);
  renderSpinTest();
}
function sptStop() {
  if (sptWhere() === 'sim') simSpinStop(); else cardTextOut('spintest stop');
  spt.phase = 'failed'; spt.text = 'stopped'; clearInterval(spt.timer); spt.timer = 0; renderSpinTest();
}
// Apply: the measured timing in the design (one undo step). The delay gains a control step (the flight core's own,
// which the test doesn't see: it times from the flight core's commands).
function sptApply() {
  const r = spt.result; if (!r || !r.ok) return;
  cfg.motorTiming = { lag: +r.lag.toFixed(4), delay: +(r.delay + 0.001).toFixed(4), measured: { delay: r.delay, lag: r.lag, gain: r.gain, r2: r.r2 }, at: Date.now() };
  undoKey = 'motor-timing:' + Date.now(); save(); refreshTuning();
  spt.applied = true;
  sptNote('Saved in the design: Gains from the motors and the step predictions use this spin-up and delay now.', 'ok');
}

function renderSpinTest() {
  const box = $('#spinTest'); if (!box) return;
  const fold = $('#spinTestFold'); const where = sptWhere(), mt = cfg.motorTiming;
  setText($('#spinTestSum'), spt.phase === 'run' || (typeof rlt !== 'undefined' && rlt.phase === 'run') ? 'running' : mt ? `spin-up ${Math.round(mt.lag * 1000)} ms` : '');
  if (typeof renderRopeLift === 'function' && typeof rlt !== 'undefined') renderRopeLift();
  if (fold && !fold.open) return;
  const focus = document.activeElement && box.contains(document.activeElement) ? document.activeElement.dataset.focusKey : null;
  box.textContent = '';
  box.append(el('p', { class: 'hint', text: 'Every motor at a base throttle below lift, then the ones that spin one way step up while the others step down: the torques no longer cancel and it starts to turn; the gyro shows how late, and how slowly, the motors followed. About 6 seconds.' }));
  box.append(el('p', { class: 'hint' + (where === 'down' ? ' warn' : ''), text: where === 'live' ? 'Connected: the Pi runs it (dfb_pi spintest), disarmed, timed by the flight core\'s own telemetry. Keep the rope clear of the props.' : where === 'down' ? 'Not connected to the drone yet (Ground tab).' : 'In the simulator the drone is held as on a rope: free to turn, nothing else.' }));
  const num = (key, label, min, max, step) => {
    const i = el('input', { type: 'number', class: 'num', min, max, step, inputmode: 'decimal', 'aria-label': label, 'data-focus-key': 'spt-' + key, value: spt[key], disabled: spt.phase === 'run' || undefined });
    i.addEventListener('change', () => { const v = parseFloat(i.value); if (Number.isFinite(v) && v >= min && v <= max) spt[key] = v; renderSpinTest(); });
    return el('label', { class: 'field' }, el('span', { text: label }), i);
  };
  box.append(el('div', { class: 'tt-setup' }, num('base', 'Base throttle', 0.05, 0.7, 0.05), num('step', 'Step', 0.01, 0.25, 0.01), num('half', 'Half period (s)', 0.08, 1, 0.05)));
  // below lift: the card's hover throttle, roughly
  const ms = actuators(), W = (model && model.m ? model.m : cfg.frame.mass) * G;
  const hover = ms.length ? ThrustTest.hoverThrottle(ms.map(c => motorMeasured(c) ? motorMeasured(c) : { tmax: c.tmax, bend: curveHat(c), vref: designVref() }), W, designVref()) : NaN;
  if (Number.isFinite(hover)) box.append(el('p', { class: 'hint' + (spt.base + spt.step > 0.85 * hover ? ' warn' : ''), text: `It hovers at about ${Math.round(hover * 100)}% by its model; the highest throttle here is ${Math.round((spt.base + spt.step) * 100)}%${spt.base + spt.step > 0.85 * hover ? ': that may lift it off the rope. Lower the base.' : ', well below.'}` }));
  box.append(el('div', { class: 'design-tools' },
    spt.phase === 'run' ? UI.button({ class: 'btn sm', type: 'button', 'data-focus-key': 'spt-stop', text: 'Stop', onclick: sptStop })
      : UI.button({ class: 'btn sm primary', type: 'button', 'data-focus-key': 'spt-start', disabled: where === 'down' || (typeof rlt !== 'undefined' && rlt.phase === 'run') || undefined, text: spt.result ? 'Run again' : 'Start', onclick: sptStart })));
  if (spt.text) box.append(el('p', { class: 'ui-status' + (spt.phase === 'failed' ? ' bad' : spt.phase === 'done' ? (spt.result && spt.result.ok ? ' ok' : ' warn') : ''), role: 'status', text: (spt.phase === 'run' ? 'Running: ' : '') + spt.text }));
  const r = spt.result;
  if (r) {
    const lagModel = ms.length ? Math.max(...ms.map(c => motorSpinUp(c))) : 0;
    box.append(el('div', { class: 'scal-result' },
      el('p', { text: `Delay ${Math.round(r.delay * 1000)} ms before the motors change (the ESC, the gyro's filter; Gains from the motors has used ${Math.round(tsgModel().delay * 1000)} ms).` }),
      el('p', { text: `Spin-up ${Math.round(r.lag * 1000)} ms (the motors' physical model says ${Math.round(lagModel * 1000)} ms).` }),
      el('p', { class: 'hint', text: `The fit explains ${Math.round(r.r2 * 100)}% of how the turn rate changed; it turned at most ${r.peak.toFixed(2)} rad/s.${r.ok ? '' : ' Not good enough to keep: a bigger step, or a steadier rope.'}` })));
  }
  if (spt.msg) box.append(el('p', { class: 'ui-status' + (spt.tone ? ' ' + spt.tone : ''), role: 'status', text: spt.msg }));
  box.append(el('div', { class: 'design-tools' },
    UI.button({ class: 'btn sm', type: 'button', 'data-focus-key': 'spt-apply', disabled: !r || !r.ok || spt.applied || spt.phase === 'run' || undefined, text: 'Apply', title: 'Keep this spin-up and delay in the design', onclick: sptApply }),
    mt ? UI.button({ class: 'btn sm', type: 'button', 'data-focus-key': 'spt-forget', disabled: spt.phase === 'run' || undefined, text: 'Forget the measurement', onclick: () => { delete cfg.motorTiming; undoKey = 'motor-timing:' + Date.now(); save(); refreshTuning(); spt.applied = false; renderSpinTest(); } }) : null));
  if (mt) box.append(el('p', { class: 'hint', text: `In the design: spin-up ${Math.round(mt.lag * 1000)} ms, delay ${Math.round(mt.delay * 1000)} ms (measured ${new Date(mt.at).toLocaleString()}).` }));
  if (focus) { const f = box.querySelector(`[data-focus-key="${focus}"]`); if (f && !f.disabled) f.focus(); }
}
document.addEventListener('toggle', e => { if (e.target && e.target.id === 'spinTestFold' && e.target.open) renderSpinTest(); }, true);
document.addEventListener('keydown', e => { if (e.key === 'Escape' && spt.phase === 'run') sptStop(); });

/* ───────── Rope hang → Lift test ───────── */
// While the rope is taut the accelerometer reads 1 g whatever the motors do; once they lift more than the weight the
// drone rises and it reads thrust over mass. runner/fc/spin_core.c's lift_* (dfb_pi "lifttest", the simulator's
// simLiftStart): every motor together, rising slowly until it lifts off the rope, then short pulses above that,
// each read as thrust / weight. Here: against what the flight controller believes the motors give at the same
// throttle and voltage, how strong they really are; Apply fits the motor cards to it (the controller's model, the
// simulator) and marks them measured (Gains from the motors then narrows its strength range).
const rlt = { base: 0.3, top: 0.9, rate: 0.04, step: 0.04, phase: 'idle', text: '', result: null, msg: '', tone: '', timer: 0, applied: false };
function rltParse(text) {
  const s = String(text || '').trim(); if (!/^lifttest:/.test(s)) return null;
  const m = /result ([\d.]+) ([\d.]+) (\d+)((?: [\d.]+)*)/.exec(s);
  let result = null;
  if (m) { const v = m[4].trim().split(/\s+/).filter(Boolean).map(Number), np = +m[3], per = np && v.length >= 3 * np ? 3 : 2, pulses = []; for (let i = 0; i + per - 1 < v.length; i += per) pulses.push({ u: v[i], f: v[i + 1], V: per === 3 ? v[i + 2] : +m[2] }); result = { liftU: +m[1], volts: +m[2], pulses }; }
  const head = s.replace(/;\s*result .*$/, '').replace(/^lifttest:\s*/, '');
  const phase = /^(settling|rising|lifted off at [\d.]+$|pulse \d)/.test(head) ? 'run' : /^done/.test(head) ? 'done' : /^stopped/.test(head) ? (result ? 'done' : 'failed') : /^idle/.test(head) ? 'idle' : 'failed';
  return { phase, text: s.replace(/^lifttest:\s*/, '').replace(/;\s*result .*$/, ''), result };
}
function rltRead(n0) {
  if (sptWhere() === 'sim') { const st = simLiftStatus(); return st ? rltParse(st.text) : null; }
  const k = Math.min(gs.log.length, (gs.nmsg || 0) - n0);
  for (const m of gs.log.slice(0, Math.max(0, k))) { const p = rltParse(m.text); if (p) return p; }
  return null;
}
function rltPoll() {
  if (rlt.phase !== 'run') { clearInterval(rlt.timer); rlt.timer = 0; return; }
  const p = rltRead(rlt.n0 ?? 0);
  if (p) { rlt.text = p.text; if (p.phase !== 'run' && p.phase !== 'idle') { rlt.phase = p.phase; rlt.result = p.result; clearInterval(rlt.timer); rlt.timer = 0; } }
  if (sptWhere() === 'live' && rlt.phase === 'run') { rlt.n0 = gs.nmsg || 0; cardTextOut('lifttest'); }
  if (sptWhere() === 'down') { rlt.phase = 'failed'; rlt.text = 'the connection went down'; }
  renderRopeLift();
}
function rltStart() {
  const w = sptWhere(); rlt.result = null; rlt.applied = false; rlt.text = ''; rlt.msg = '';
  if (w === 'down') { rlt.msg = 'Not connected to the drone (Ground tab).'; rlt.tone = 'bad'; renderRopeLift(); return; }
  if (w === 'sim') { const why = simLiftStart(rlt.base, rlt.top, rlt.rate, rlt.step); if (why) { rlt.msg = why; rlt.tone = 'bad'; renderRopeLift(); return; } if (!running) { running = true; if (typeof renderRun === 'function') renderRun(); } }
  else { rlt.n0 = gs.nmsg || 0; const why = cardTextOut(`lifttest ${rlt.base.toFixed(3)} ${rlt.top.toFixed(3)} ${rlt.rate.toFixed(3)} ${rlt.step.toFixed(3)}`); if (why) { rlt.msg = why; rlt.tone = 'bad'; renderRopeLift(); return; } }
  rlt.phase = 'run'; rlt.text = 'starting…';
  clearInterval(rlt.timer); rlt.timer = setInterval(rltPoll, w === 'sim' ? 300 : 1000);
  renderRopeLift();
}
function rltStop() { if (sptWhere() === 'sim') simSpinStop(); else cardTextOut('lifttest stop'); rlt.phase = 'failed'; rlt.text = 'stopped'; clearInterval(rlt.timer); rlt.timer = 0; renderRopeLift(); }

// What the flight controller believes all the motors lift together at throttle u on V volts [N] (their thrust at
// the reference voltage and curve, as its model has them).
const rltBelieved = (u, V) => actuators().reduce((t, c) => { const e = u * (V > 0.5 ? V : designVref()) / designVref(); return t + c.tmax * believedThrust(clamp(e, 0, 1.5), curveHat(c)); }, 0);
// The measured points: each pulse's thrust [N] (mass × its reading), and lift-off (about the weight: it triggers 4%
// over it, a little late). { pts: [{ u, V, T, model }], ratio: measured / believed, hoverU, hoverModel }.
function rltAnalyse(r) {
  const m = (model && model.m) || cfg.frame.mass, W = m * G, V = r.volts;
  const pts = r.pulses.map(p => ({ u: p.u, V: p.V || V, T: p.f * W, model: rltBelieved(p.u, p.V || V) }));
  if (!pts.length) return null;
  const ratio = pts.reduce((a, p) => a + p.T / p.model, 0) / pts.length;
  // hover: lift-off is a little over the weight (it triggers at 4% over): back down along thrust ∝ u^k, k from the
  // model's own curve there (about 1.5–2), the pack's voltage at lift-off
  const k = clamp(Math.log(rltBelieved(r.liftU * 1.05, V) / rltBelieved(r.liftU, V)) / Math.log(1.05), 1.1, 3);
  const hoverU = r.liftU * Math.pow(1.04, -1 / k);
  let lo = 0, hi = 1.2; for (let i = 0; i < 50; i++) { const mid = (lo + hi) / 2; if (rltBelieved(mid, V) < W) lo = mid; else hi = mid; }
  return { pts, ratio, hoverU, hoverModel: lo, mass: m, V };
}
// Apply: each motor's card fitted to its share of the measured thrust (one undo step), marked measured.
function rltApply() {
  const r = rlt.result, a = r && rltAnalyse(r); if (!a) return;
  const ms = actuators(), n = ms.length, vref = designVref(), rho = flightAtmosphere().rho, out = [];
  const pts = [{ u: r.liftU, V: a.V, grams: 1.04 * a.mass * 1000 / n }, ...a.pts.map(p => ({ u: p.u, V: p.V, grams: p.T / G * 1000 / n }))];
  for (const c of ms) {
    const kind = c.motorPhysics?.kind, bend = curveHat(c);
    if (c.propPhysics?.rows) { out.push(`${c.name}: has a measured prop table, kept`); continue; }
    const rec = { at: Date.now(), vref, bend: +bend.toFixed(3), points: pts.map(p => ({ ...p })), source: 'rope', ct: null, tmax: 0, rms: 0 };
    if (kind === 'brushless' || kind === 'brushed') {
      const ph = ThrustTest.fitPhysics(pts, (ct, u, V) => FlightPhysics.equilibrium(FlightPhysics.motor({ ...c, propPhysics: { ...c.propPhysics, ct } }, vref), u * flightDuty(c), V > 0.5 ? V : vref, rho).T);
      if (!ph) continue;
      rec.ct = +ph.ct.toFixed(5); c.propPhysics = { ...c.propPhysics, ct: rec.ct };
      rec.tmax = +FlightPhysics.motor(c, vref).ratedThrust.toFixed(4);
    } else { c.tmax = +(c.tmax * a.ratio).toFixed(4); rec.tmax = c.tmax; }
    c.thrustTest = rec;
  }
  undoKey = 'rope-lift:' + Date.now();
  recomputeProps(); cPts = contactPoints(); rebuildDrone(); refreshEnvelope(); renderMass();
  for (const c of ms) refreshCard(c);
  buildActRows(); save(); refreshTuning();
  if (typeof flightRerenderParts === 'function') flightRerenderParts();
  rlt.applied = true;
  rlt.msg = `Saved in the design: every motor's card now lifts ${a.ratio.toFixed(2)}× what it did${out.length ? ' (' + out.join('; ') + ')' : ''}. ${sptWhere() === 'live' ? 'Upload design puts the corrected model on the drone.' : 'On the real drone, Upload design puts it there.'}`;
  rlt.tone = 'ok'; renderRopeLift();
}

function renderRopeLift() {
  const box = $('#ropeLift'); if (!box) return;
  const fold = $('#spinTestFold'); if (fold && !fold.open) return;
  const where = sptWhere(), focus = document.activeElement && box.contains(document.activeElement) ? document.activeElement.dataset.focusKey : null;
  box.textContent = '';
  box.append(el('p', { class: 'hint', text: 'Every motor together rises slowly from the base throttle until the drone lifts off the rope (the accelerometer over 1 g, or the barometer up 8 cm): that throttle holds its weight. Then short pulses a little above it, each read as thrust = mass × acceleration. It rises a few centimetres each time and the rope catches it.' }));
  const num = (key, label, min, max, step) => {
    const i = el('input', { type: 'number', class: 'num', min, max, step, inputmode: 'decimal', 'aria-label': label, 'data-focus-key': 'rlt-' + key, value: rlt[key], disabled: rlt.phase === 'run' || undefined });
    i.addEventListener('change', () => { const v = parseFloat(i.value); if (Number.isFinite(v) && v >= min && v <= max) rlt[key] = v; renderRopeLift(); });
    return el('label', { class: 'field' }, el('span', { text: label }), i);
  };
  box.append(el('div', { class: 'tt-setup' }, num('base', 'Base throttle', 0.05, 0.7, 0.05), num('top', 'Highest', 0.3, 1, 0.05), num('rate', 'Rise (/s)', 0.005, 0.2, 0.005), num('step', 'Pulse step', 0.01, 0.2, 0.01)));
  box.append(el('p', { class: 'hint', text: `The drone weighs ${Math.round(((model && model.m) || cfg.frame.mass) * 1000)} g by the design: the thrust is read against that, so weigh it if you're not sure.` }));
  box.append(el('div', { class: 'design-tools' },
    rlt.phase === 'run' ? UI.button({ class: 'btn sm', type: 'button', 'data-focus-key': 'rlt-stop', text: 'Stop', onclick: rltStop })
      : UI.button({ class: 'btn sm primary', type: 'button', 'data-focus-key': 'rlt-start', disabled: where === 'down' || spt.phase === 'run' || undefined, text: rlt.result ? 'Run again' : 'Start', onclick: rltStart })));
  if (rlt.text) box.append(el('p', { class: 'ui-status' + (rlt.phase === 'failed' ? ' bad' : rlt.phase === 'done' ? (rlt.result && rlt.result.pulses.length ? ' ok' : ' warn') : ''), role: 'status', text: (rlt.phase === 'run' ? 'Running: ' : '') + rlt.text }));
  const a = rlt.result && rltAnalyse(rlt.result);
  if (a) {
    const pct = x => Math.round(x * 100) + '%';
    box.append(el('div', { class: 'scal-result' },
      el('p', { text: `It hovers at about ${pct(a.hoverU)} throttle on ${a.V.toFixed(2)} V; the flight controller's model says ${pct(a.hoverModel)}.` }),
      el('p', { text: `The motors lift ${a.ratio.toFixed(2)}× what the model says${Math.abs(a.ratio - 1) > 0.1 ? ` — every correction the controller makes is ${a.ratio > 1 ? 'that much too strong' : 'that much too weak'}` : ''}.` }),
      el('p', { class: 'hint', text: a.pts.map(p => `${pct(p.u)}: ${Math.round(p.T / G * 1000)} g (model ${Math.round(p.model / G * 1000)} g)`).join(' · ') })));
  }
  if (rlt.msg) box.append(el('p', { class: 'ui-status' + (rlt.tone ? ' ' + rlt.tone : ''), role: 'status', text: rlt.msg }));
  box.append(el('div', { class: 'design-tools' }, UI.button({ class: 'btn sm', type: 'button', 'data-focus-key': 'rlt-apply', disabled: !a || rlt.applied || rlt.phase === 'run' || undefined, text: 'Apply to the motor cards', title: 'Fit every motor card to the measured thrust (one undo step)', onclick: rltApply })));
  if (focus) { const f = box.querySelector(`[data-focus-key="${focus}"]`); if (f && !f.disabled) f.focus(); }
}
document.addEventListener('toggle', e => { if (e.target && e.target.id === 'spinTestFold' && e.target.open) renderRopeLift(); }, true);
document.addEventListener('keydown', e => { if (e.key === 'Escape' && rlt.phase === 'run') rltStop(); });
