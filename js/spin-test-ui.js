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
  setText($('#spinTestSum'), spt.phase === 'run' ? 'running' : spt.applied ? 'applied' : mt ? `measured: ${Math.round(mt.lag * 1000)} ms spin-up` : '');
  if (fold && !fold.open) return;
  const focus = document.activeElement && box.contains(document.activeElement) ? document.activeElement.dataset.focusKey : null;
  box.textContent = '';
  box.append(el('p', { class: 'hint', text: 'Measures how quickly the motors answer: the drone hangs on a rope, free to turn; every motor runs at a base throttle below lift, then the ones that spin one way step up while the others step down. The drag torques no longer cancel and it starts to turn; the gyro shows how late, and how slowly, the motors followed. Props on; about 8 seconds.' }));
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
      : UI.button({ class: 'btn sm primary', type: 'button', 'data-focus-key': 'spt-start', disabled: where === 'down' || undefined, text: spt.result ? 'Run again' : 'Start', onclick: sptStart })));
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
