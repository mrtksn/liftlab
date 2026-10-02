'use strict';
// The Battery settings (Airframe tab) and the Health panel: every motor, servo and the battery with its true
// state and temperature, what the drone's sensors say, what the supervisor has done about it, a way to break
// each one mid-flight, and the supervisor's log.

/* ───────── battery settings ───────── */
function battEdited() { undoKey = 'battery'; recomputeProps(); refreshEnvelope(); renderMass(); save(); renderBattSmall(); }
function renderBattSmall() { const b = battCfg(); $('#battSmall').textContent = `${b.cells}S · ${b.capacity.toFixed(1)} Ah · ${(0.038 * b.capacity * b.cells * 1000).toFixed(0)} g`; }
function renderBattery() {
  const box = $('#battFields'); if (!box) return; box.textContent = '';
  const b = battCfg();
  const num = (key, d) => numField('batt-' + key, d, () => battCfg()[key], v => { battCfg()[key] = key === 'cells' ? Math.round(v) : v; battEdited(); }).node;
  const chk = (key, label) => { const id = 'batt-' + key, i = el('input', { type: 'checkbox', id }); i.checked = !!b[key]; i.addEventListener('change', () => { battCfg()[key] = i.checked; battEdited(); }); return el('label', { class: 'check', for: id }, i, label); };
  const sel = el('select', { id: 'batt-failMode' });
  for (const [v, t] of [['cell', 'It loses a cell'], ['cut', 'It cuts out']]) { const o = el('option', { value: v, text: t }); if (b.failMode === v) o.selected = true; sel.append(o); }
  sel.addEventListener('change', () => { battCfg().failMode = sel.value; battEdited(); });
  box.append(
    num('cells', { label: 'Cells in series', min: 2, max: 8, step: 1, u: 'S', dp: 0 }),
    num('capacity', { label: 'Capacity', min: 0.3, max: 10, hmax: 50, step: 0.1, u: 'Ah', dp: 1 }),
    num('rInt', { label: 'Internal resistance (at 25 °C)', min: 0.005, max: 0.2, step: 0.005, u: 'mΩ', dp: 0, k: 1000 }),
    num('tmaxC', { label: 'Temperature limit', min: 40, max: 90, step: 1, u: '°C', dp: 0 }),
    chk('vsens', 'Voltage sensor (the flight controller corrects the throttle for sag)'), chk('isens', 'Current sensor'), chk('tsens', 'Temperature sensor'),
    chk('failHeat', 'Overheating damages it'),
    el('div', { class: 'field' }, el('label', { for: 'batt-failMode', text: 'When it fails' }), sel),
    el('p', { class: 'hint', text: 'Its weight isn\'t added for you: the layouts carry it as a mass part. It heats from the current through its internal resistance, which falls as it warms (so a warm pack sags less); past its limit it loses capacity and gains resistance for good, and 25 °C past it, it fails the way you set.' }));
  renderBattSmall();
}

/* ───────── health panel ───────── */
const HEALTH = { sig: '', rows: new Map() };
const MODE_TXT = { normal: ['Normal', 'good'], caution: ['Careful', 'warn'], return: ['Returning home', 'bad'], land: ['Landing', 'bad'], landed: ['Landed', 'warn'] };
function tempColor(f) { return f < 0.75 ? 'var(--good)' : f < 1 ? 'var(--warn)' : 'var(--bad)'; }
function breakMenu(options, onPick, label) {
  const s = el('select', { class: 'break', 'aria-label': label, title: label });
  s.append(el('option', { value: '', text: 'Break…' }));
  for (const [v, t] of options) s.append(el('option', { value: v, text: t }));
  s.addEventListener('change', () => { const v = s.value; s.value = ''; if (v) { onPick(v); renderHealth(true); } });
  return s;
}
function healthRow(key, name, menu) {
  const bar = el('i'), temp = el('span', { class: 'h-temp' }), state = el('span', { class: 'h-state' }), sub = el('span', { class: 'h-sub' });
  const row = el('div', { class: 'h-row' }, el('span', { class: 'h-name', text: name }), el('span', { class: 'h-bar' }, bar), temp, state, menu || el('span'), sub);
  HEALTH.rows.set(key, { row, bar, temp, state, sub });
  return row;
}
function buildHealth() {
  const box = $('#healthBody'); box.textContent = ''; HEALTH.rows.clear();
  const supId = 'supOn', cb = el('input', { type: 'checkbox', id: supId }); cb.checked = sup.on;
  cb.addEventListener('change', () => { sup.on = cb.checked; });
  const mode = el('span', { class: 'pill good', id: 'supMode' }, el('i'), el('span', { text: 'Normal' }));
  const repair = el('button', { class: 'btn', type: 'button', text: 'Repair all', title: 'Undo every failure and damage (the supervisor\'s settings too), without resetting the flight' });
  repair.addEventListener('click', () => { repairAll(); renderHealth(true); });
  box.append(el('div', { class: 'h-top' }, el('label', { class: 'check', for: supId }, cb, 'Supervisor on'), mode, repair),
    el('p', { class: 'hint', id: 'supWhy', text: 'Companion computer at 10 Hz, 40 ms each way over its link. Each part: its true temperature and state, then what the drone can sense and what the supervisor did.', title: 'The supervisor (like a Raspberry Pi next to the ESP32) reads the health sensors and the flight controller\'s data stream, and changes the controller\'s settings when a part fails, weakens or runs hot.' }));
  const list = el('div', { class: 'h-list' });
  for (const c of actuators()) list.append(healthRow(c.id, c.name, breakMenu([['stop', 'Stop it'], ['loss', `Lose ${c.failLoss ?? 50}% thrust`], ['prop', 'Break its prop']], v => breakDevice(c, v), 'Break ' + c.name)));
  for (const j of joints()) list.append(healthRow(j.id, j.name, breakMenu([['jam', 'Jam it'], ['limp', 'Make it go limp']], v => breakDevice(j, v), 'Break ' + j.name)));
  list.append(healthRow('batt', 'Battery', breakMenu([['cell', 'Lose a cell'], ['cut', 'Cut out']], v => breakBattery(v), 'Break the battery')));
  box.append(list, el('span', { class: 'lbl', text: 'What happened' }), el('ol', { class: 'h-log', id: 'supLog' }));
  HEALTH.sig = healthSig();
}
const healthSig = () => actuators().map(c => c.id + c.name + (c.failLoss ?? 50)).join(',') + '|' + joints().map(j => j.id + j.name).join(',');
function repairAll() {
  for (const c of actuators()) { const e = fcAct(c); if (e.eff !== 1) scaleActuatorColumns(c, 1 / e.eff); }
  hs.forEach(s => Object.assign(s, { loss: 0, dead: false, prop: false, jam: null, limp: false, cause: '', failT: null, mode: null }));
  Object.assign(hb, { fade: 0, cellsLost: 0, cut: false, cause: '', failT: null });
  fc.act.clear(); fc.joint.clear(); fc.jAng.clear(); if (fc.mode !== 'landed') { fc.mode = 'normal'; fc.lim = { speed: null, lean: 35, accel: 6 }; }
  Object.assign(sup, { dec: {}, pol: { mode: 'normal' }, st: {}, sent: null, capLogged: new Set(), jLogged: new Set(), cells: null, vHist: [], cellLost: false });
  invalidateDesc(); nb = nominalAxis(); refreshEnvelope();
  healthEvent('Everything repaired.', 'good');
}
function renderHealth(force) {
  if (!$('#healthBody')) return;
  if (force === true || HEALTH.sig !== healthSig() || !HEALTH.rows.size) buildHealth();
  const put = (key, frac, tempTxt, stateTxt, tone, subTxt) => {
    const r = HEALTH.rows.get(key); if (!r) return;
    r.bar.style.width = (clamp(frac, 0, 1.2) / 1.2 * 100).toFixed(1) + '%'; r.bar.style.background = tempColor(frac);
    r.temp.textContent = tempTxt; r.state.textContent = stateTxt; r.state.className = 'h-state ' + (tone || ''); r.sub.textContent = subTxt;
  };
  const dec = (sup.dec && sup.dec.acts) || {};
  for (const c of actuators()) {
    const s = hs.get(c.id) || { T: ambient(), loss: 0 }, lim = c.tmaxC ?? 120, r = hread.m.get(c.id) || {}, e = fcAct(c);
    const truth = s.prop ? ['prop broken', 'bad'] : s.dead ? ['stopped', 'bad'] : s.loss > 0.004 ? [`−${Math.round(s.loss * 100)}% thrust${s.cause === 'burned out' || s.T > lim ? ' (heat)' : ''}`, 'warn'] : ['working', ''];
    const seen = r.T != null ? `sensor ${Math.round(r.T)}°` : sup.tEst.has(c.id) && r.I != null ? `est. ${Math.round(sup.tEst.get(c.id))}° from ESC current` : 'temperature not measured';
    const did = !e.on ? 'removed from the table' : [e.eff < 0.99 || e.eff > 1.01 ? `table ×${e.eff.toFixed(2)}` : '', e.cap < 0.995 ? `capped at ${Math.round(e.cap * 100)}%` : ''].filter(Boolean).join(', ') || (dec[c.id] ? 'OK' : '');
    put(c.id, (s.T - ambient()) / Math.max(10, lim - ambient()), `${Math.round(s.T)} °C`, truth[0], truth[1], `${seen}${did ? ' · supervisor: ' + did : ''}`);
  }
  for (const j of joints()) {
    const s = hs.get(j.id) || {};
    put(j.id, 0, '', s.limp ? 'limp' : s.jam != null ? 'jammed' : 'working', s.limp || s.jam != null ? 'bad' : '', fcJointOk(j) ? (s.limp || s.jam != null ? 'supervisor: not spotted yet' : '') : `supervisor: left out of the steering, believed at ${Math.round((fc.jAng.get(j.id) ?? 0) * R2D)}° (really ${Math.round((jst.get(j.id) || {}).th * R2D || 0)}°)`);
  }
  {
    const b = battCfg(), r = hread.b, soc = S.batt && S.batt.soc != null ? S.batt.soc : 1;
    const truth = hb.cut ? ['cut out', 'bad'] : hb.cellsLost ? [`${hb.cellsLost} cell${hb.cellsLost > 1 ? 's' : ''} lost`, 'bad'] : hb.fade > 0.005 ? [`worn ${Math.round(hb.fade * 100)}%`, 'warn'] : ['working', ''];
    const seen = [r.V != null ? `${r.V.toFixed(1)} V` : 'no voltage sensor', r.I != null ? `${r.I.toFixed(0)} A` : '', r.T != null ? `${Math.round(r.T)}°` : ''].filter(Boolean).join(' · ');
    put('batt', (hb.T - ambient()) / Math.max(10, b.tmaxC - ambient()), `${Math.round(hb.T)} °C`, truth[0], truth[1], `${Math.round(soc * 100)}% charge · sensors: ${seen}${sup.cellLost ? ` · supervisor: counts ${sup.cells} cells, going home` : ''}`);
  }
  const m = MODE_TXT[fc.mode] || MODE_TXT.normal, pill = $('#supMode');
  pill.className = 'pill ' + m[1]; pill.querySelector('span').textContent = sup.on || fc.mode !== 'normal' ? m[0] : 'Off';
  const why = sup.pol && sup.pol.why && fc.mode !== 'normal' ? sup.pol.why : '';
  const hud = $('#hudSup'); hud.hidden = fc.mode === 'normal'; hud.textContent = `supervisor: ${m[0].toLowerCase()}${why ? ' · ' + why : ''}`;
  const log = $('#supLog'); log.textContent = '';
  for (const l of sup.log.slice(0, 8)) log.append(el('li', { class: l.tone }, el('b', { text: `${l.t.toFixed(1)} s` }), ' ' + l.msg));
  if (!sup.log.length) log.append(el('li', { class: 'muted', text: 'Nothing yet.' }));
  $('#healthSmall').textContent = sup.on ? (fc.mode === 'normal' ? 'all normal' : m[0].toLowerCase()) : 'supervisor off';
}
renderBattery();
