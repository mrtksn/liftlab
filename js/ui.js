'use strict';
// Panels: airframe editor, telemetry, traces, target & environment, header controls, persistence.

const $ = s => document.querySelector(s);
function el(tag, attrs = {}, ...kids) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (v == null) continue;
    if (k === 'class') e.className = v; else if (k === 'text') e.textContent = v; else if (k === 'html') e.innerHTML = v;
    else if (k.startsWith('on')) e.addEventListener(k.slice(2), v); else e.setAttribute(k, v);
  }
  for (const c of kids) if (c != null) e.append(c);
  return e;
}
// Readouts that tick: write only what changed, so a reading can be selected and copied while it runs.
function setText(n, t) {
  if (!n || n.textContent === t) return;
  const s = getSelection(); if (s && s.rangeCount && !s.isCollapsed && s.containsNode(n, true)) return;   // being selected: hold still until let go
  n.textContent = t;
}
function syncKv(dl, rows) {   // a <dl> of [name, value] rows, built once, values updated in place
  const sig = rows.map(r => r[0]).join('|');
  if (dl._sig !== sig) { dl.textContent = ''; dl._dd = rows.map(([k]) => { const dd = el('dd'); dl.append(el('dt', { text: k }), dd); return dd; }); dl._sig = sig; }
  rows.forEach((r, i) => { setText(dl._dd[i], r[1]); const c = r[2] || ''; if (dl._dd[i].className !== c) dl._dd[i].className = c; });
}
// Chips kept in place by key: only what changed is touched, so a chip can be clicked or keep the focus while the
// readouts tick. list: [{ key, src, text, tone, go }]; a chip with go (what clicking it does) is a button.
function syncChips(box, list) {
  const have = box._chips || (box._chips = new Map()), keys = new Set(list.map(c => c.key));
  for (const [k, n] of have) if (!keys.has(k) || n.tagName !== (list.find(c => c.key === k).go ? 'BUTTON' : 'SPAN')) { n.remove(); have.delete(k); }
  let at = box.firstChild;
  for (const c of list) {
    let n = have.get(c.key);
    if (!n) {
      n = el(c.go ? 'button' : 'span', { type: c.go ? 'button' : null, 'data-focus-key': 'chip-' + c.key }, srcDot(c.src), document.createTextNode(''));
      if (c.go) n.addEventListener('click', () => n._go && n._go());
      n._src = c.src; have.set(c.key, n);
    }
    n._go = c.go || null;
    if (n._src !== c.src) { n.firstChild.replaceWith(srcDot(c.src)); n._src = c.src; }
    const cls = 'chip' + (c.tone ? ' ' + c.tone : ''); if (n.className !== cls) n.className = cls;
    if (n.lastChild.data !== c.text) n.lastChild.data = c.text;
    if (n === at) at = at.nextSibling; else box.insertBefore(n, at);
  }
}
// Re-renders replace nodes: put the keyboard focus back on the same control afterwards (found by its id or its
// data-focus-key), instead of letting it drop to the page.
function keepFocus(fn) {
  const a = document.activeElement;
  const sel = a && a !== document.body ? (a.dataset && a.dataset.focusKey ? `[data-focus-key="${CSS.escape(a.dataset.focusKey)}"]` : a.id ? '#' + CSS.escape(a.id) : null) : null;
  try { return fn(); } finally {
    if (sel && !a.isConnected && (!document.activeElement || document.activeElement === document.body)) {
      const n = document.querySelector(sel); if (n) n.focus({ preventScroll: true });
    }
  }
}
// A button that opens a short list of actions. (A select used for actions acts on a single arrow key.)
// o: { text, label, title, key (data-focus-key), cls, align: 'left', items() -> [{ value, label, hint, group, disabled, cur }], onPick(value) }
let openMenuClose = null;
function menuButton(o) {
  const btn = el('button', { type: 'button', class: 'btn mb-btn' + (o.cls ? ' ' + o.cls : ''), 'aria-haspopup': 'menu', 'aria-expanded': 'false', 'aria-label': o.label || null, title: o.title || null, 'data-focus-key': o.key || null },
    el('span', { text: o.text }), el('span', { class: 'mb-caret', 'aria-hidden': 'true', text: '▾' }));
  const menu = el('div', { class: 'mb-menu' + (o.align === 'left' ? ' left' : ''), role: 'menu', 'aria-label': o.label || o.text });
  menu.hidden = true;
  const wrap = el('span', { class: 'mb' }, btn, menu);
  const live = () => [...menu.querySelectorAll('[role=menuitem]:not([aria-disabled="true"])')];
  const close = back => {
    if (menu.hidden) return; menu.hidden = true; btn.setAttribute('aria-expanded', 'false');
    if (openMenuClose === close) openMenuClose = null;
    if (back) btn.focus();
  };
  const open = which => {
    if (openMenuClose && openMenuClose !== close) openMenuClose(false);
    menu.textContent = ''; let grp = null;
    for (const it of o.items()) {
      if (it.group && it.group !== grp) { grp = it.group; menu.append(el('div', { class: 'mb-grp', role: 'presentation', text: grp })); }
      const b = el('button', { type: 'button', role: 'menuitem', tabindex: '-1', class: 'mb-item' + (it.cur ? ' cur' : '') }, el('span', { text: it.label }), it.hint ? el('span', { class: 'mb-hint', text: it.hint }) : null);
      if (it.disabled) b.setAttribute('aria-disabled', 'true');
      b.addEventListener('click', () => { if (it.disabled) return; close(true); o.onPick(it.value); });
      menu.append(b);
    }
    menu.hidden = false; btn.setAttribute('aria-expanded', 'true'); openMenuClose = close;
    const l = live(); const f = which === 'last' ? l[l.length - 1] : l[0]; if (f) f.focus();
  };
  btn.addEventListener('click', () => { if (menu.hidden) open('first'); else close(false); });
  btn.addEventListener('keydown', e => { if (e.key === 'ArrowDown' || e.key === 'ArrowUp') { e.preventDefault(); open(e.key === 'ArrowUp' ? 'last' : 'first'); } });
  menu.addEventListener('keydown', e => {
    const l = live(), i = l.indexOf(document.activeElement);
    let j = null;
    if (e.key === 'ArrowDown') j = (i + 1) % l.length; else if (e.key === 'ArrowUp') j = (i - 1 + l.length) % l.length;
    else if (e.key === 'Home') j = 0; else if (e.key === 'End') j = l.length - 1;
    else if (e.key === 'Escape') { e.preventDefault(); close(true); return; }
    else if (e.key === 'Tab') { close(false); return; }
    else return;
    e.preventDefault(); if (l[j]) l[j].focus();
  });
  wrap.addEventListener('focusout', e => { if (!menu.hidden && e.relatedTarget && !wrap.contains(e.relatedTarget)) close(false); });
  return { node: wrap, btn, close };
}
document.addEventListener('pointerdown', e => { if (openMenuClose && !e.target.closest('.mb')) openMenuClose(false); });
// A select whose change is a big step (it resets the flight): a pick with the mouse applies at once; stepping
// through it with the arrow keys only applies on Enter, or when you leave it.
function commitSelect(sel, apply, hint = 'Press Enter to apply') {
  let key = null, pending = false;
  const go = () => { if (!pending) return; pending = false; sel.classList.remove('pending'); if (sel.dataset.title != null) sel.title = sel.dataset.title; apply(sel.value); };
  sel.addEventListener('keydown', e => { if (e.key === 'Enter') { key = null; if (pending) { e.preventDefault(); go(); } } else key = e.key; });
  sel.addEventListener('pointerdown', () => { key = null; });
  sel.addEventListener('change', () => {
    const stepping = key != null && (/^(Arrow|Page)/.test(key) || key === 'Home' || key === 'End' || (key.length === 1 && key !== ' '));
    key = null; pending = true;
    if (!stepping) { go(); return; }
    sel.classList.add('pending'); if (sel.dataset.title == null) sel.dataset.title = sel.title || ''; sel.title = hint;
  });
  sel.addEventListener('blur', go);
}
let running = true, speed = 1;

/* ───────── components ───────── */
const TAG = { motor: 'Motor', joint: 'Servo', link: 'Rod', mass: 'Mass', hang: 'Cable' };
const SENSOR_TAG = { imu: 'IMU', mag: 'Compass', baro: 'Baro', fix: 'Fix', flow: 'Flow' };
const tagOf = c => c.type === 'sensor' ? SENSOR_TAG[c.kind] : TAG[c.type];
const FD = {
  x: { label: 'X', path: ['pos', 0], hmin: -2, hmax: 2,  min: -0.6, max: 0.6, step: 0.005, u: 'm', dp: 3 },
  y: { label: 'Y', path: ['pos', 1], hmin: -2, hmax: 2,  min: -0.6, max: 0.6, step: 0.005, u: 'm', dp: 3 },
  z: { label: 'Z', path: ['pos', 2], hmin: -2, hmax: 2,  min: -0.3, max: 0.3, step: 0.005, u: 'm', dp: 3 },
  tilt: { label: 'Shaft tilt from vertical', hmax: 180,  path: ['tilt'], min: 0, max: 90, step: 1, u: '°', dp: 0 },
  az: { label: 'Tilt toward (azimuth)', path: ['az'], min: -180, max: 180, step: 5, u: '°', dp: 0 },
  prop: { label: 'Prop radius', path: ['prop'], min: 0.03, max: 0.25, hmax: 0.6, step: 0.005, u: 'm', dp: 3 },
  tmax: { label: 'Max thrust', hmax: 200,  path: ['tmax'], min: 0.5, max: 30, step: 0.5, u: 'N', dp: 1 },
  kappa: { label: 'Drag torque ratio κ', path: ['kappa'], min: 0, max: 0.06, step: 0.001, u: 'm', dp: 3 },
  fm: { label: 'Prop efficiency (figure of merit)', path: ['fm'], min: 0.3, max: 0.85, step: 0.01, u: '', dp: 2 },
  storque: { label: 'Servo stall torque · hidden', path: ['torque'], min: 0.05, max: 5, hmax: 40, step: 0.05, u: 'N·m', dp: 2 },
  slag: { label: 'Servo command delay · hidden', path: ['lag'], min: 0, max: 0.15, step: 0.005, u: 'ms', dp: 0, k: 1000 },
  offset: { label: 'Servo trim error · hidden', path: ['offset'], min: -10, max: 10, step: 0.5, u: '°', dp: 1 },
  tau: { label: 'Spin-up time constant · hidden', hmin: 0.001, hmax: 1,  path: ['tau'], min: 0.01, max: 0.2, step: 0.005, u: 'ms', dp: 0, k: 1000 },
  mass: { label: 'Mass', hmax: 50,  path: ['mass'], min: 0.01, max: 2, step: 0.01, u: 'kg', dp: 2 },
  health: { label: 'Health (thrust delivered)', path: ['health'], min: 0, max: 100, step: 1, u: '%', dp: 0 },
  tmaxC: { label: 'Temperature limit', path: ['tmaxC'], min: 60, max: 180, step: 5, u: '°C', dp: 0 },
  cool: { label: 'Cooling (1 = typical)', path: ['cool'], min: 0.2, max: 2, step: 0.05, u: '×', dp: 2 },
  failLoss: { label: 'Thrust it loses when it fails', path: ['failLoss'], min: 10, max: 90, step: 5, u: '%', dp: 0 },
  hingeAz: { label: 'Hinge axis heading', path: ['hingeAz'], min: -180, max: 180, step: 5, u: '°', dp: 0 },
  manual: { label: 'Angle (set by you)', path: ['manual'], min: -90, max: 90, step: 1, u: '°', dp: 0 },
  hingeEl: { label: 'Hinge axis tilt up', path: ['hingeEl'], min: -90, max: 90, step: 5, u: '°', dp: 0 },
  laz: { label: 'Points toward (azimuth)', path: ['az'], min: -180, max: 180, step: 5, u: '°', dp: 0 },
  lel: { label: 'Points up or down (−90° straight down)', path: ['el'], min: -90, max: 90, step: 5, u: '°', dp: 0 },
  lroll: { label: 'Rolled about its length', path: ['roll'], min: -180, max: 180, step: 5, u: '°', dp: 0 },
  llen: { label: 'Length', path: ['length'], min: 0.02, max: 0.6, hmax: 3, step: 0.005, u: 'm', dp: 3 },
  smass: { label: 'Mass', path: ['mass'], min: 0.001, max: 0.05, hmin: 0.0001, hmax: 1, step: 0.001, u: 'g', dp: 1, k: 1000 },
  jmass: { label: 'Servo mass', path: ['mass'], min: 0.002, max: 0.2, step: 0.005, u: 'kg', dp: 3 },
  range: { label: 'Servo limit ±', path: ['range'], min: 5, max: 90, step: 1, u: '°', dp: 0 },
  rate: { label: 'Servo speed', hmax: 5000,  path: ['rate'], min: 20, max: 1000, step: 10, u: '°/s', dp: 0 },
  lx: { label: 'Size X', path: ['size', 0], min: 0.01, max: 0.5, step: 0.005, u: 'm', dp: 3 },
  ly: { label: 'Size Y', path: ['size', 1], min: 0.01, max: 0.5, step: 0.005, u: 'm', dp: 3 },
  lz: { label: 'Size Z', path: ['size', 2], min: 0.01, max: 0.5, step: 0.005, u: 'm', dp: 3 },
  radius: { label: 'Radius', path: ['radius'], min: 0.01, max: 0.25, step: 0.005, u: 'm', dp: 3 },
  length: { label: 'Length', path: ['length'], min: 0.02, max: 0.6, step: 0.005, u: 'm', dp: 3 },
  cable: { label: 'Cable length', hmax: 10,  path: ['length'], min: 0.05, max: 2, step: 0.01, u: 'm', dp: 2 },
  // sensors
  mr: { label: 'Mount roll', path: ['mount', 0], min: -180, max: 180, step: 5, u: '°', dp: 0 },
  mp: { label: 'Mount pitch', path: ['mount', 1], min: -90, max: 90, step: 5, u: '°', dp: 0 },
  my: { label: 'Mount yaw', path: ['mount', 2], min: -180, max: 180, step: 5, u: '°', dp: 0 },
  rateImu: { label: 'Sample rate', hmin: 10, hmax: 8000,  path: ['rate'], min: 50, max: 2000, step: 50, u: 'Hz', dp: 0 },
  rateMag: { label: 'Sample rate', path: ['rate'], min: 10, max: 200, step: 10, u: 'Hz', dp: 0 },
  rateBaro: { label: 'Sample rate', path: ['rate'], min: 5, max: 100, step: 5, u: 'Hz', dp: 0 },
  rateFix: { label: 'Update rate', path: ['rate'], min: 1, max: 200, step: 1, u: 'Hz', dp: 0 },
  latImu: { label: 'Delay', path: ['latency'], min: 0, max: 50, step: 0.5, u: 'ms', dp: 1 },
  lat: { label: 'Delay', hmax: 2000,  path: ['latency'], min: 0, max: 300, step: 1, u: 'ms', dp: 0 },
  gyroNoise: { label: 'Gyro noise', path: ['gyroNoise'], min: 0, max: 1, step: 0.01, u: '°/s', dp: 2 },
  gyroBias: { label: 'Gyro turn-on bias (σ)', path: ['gyroBias'], min: 0, max: 5, step: 0.05, u: '°/s', dp: 2 },
  scaleErr: { label: 'Scale error (σ)', path: ['scaleErr'], min: 0, max: 0.05, step: 0.001, u: '%', dp: 1, k: 100 },
  misalign: { label: 'Axis misalignment (σ)', path: ['misalign'], min: 0, max: 2, step: 0.05, u: '°', dp: 2 },
  gyroDrift: { label: 'Gyro bias drift', path: ['gyroDrift'], min: 0, max: 0.2, step: 0.005, u: '°/s/√s', dp: 3 },
  accNoise: { label: 'Accel noise', path: ['accNoise'], min: 0, max: 0.5, step: 0.01, u: 'm/s²', dp: 2 },
  accBias: { label: 'Accel bias (σ)', path: ['accBias'], min: 0, max: 0.5, step: 0.01, u: 'm/s²', dp: 2 },
  vib: { label: 'Vibration pickup (mounting)', path: ['vib'], min: 0, max: 3, step: 0.1, u: '×', dp: 1 },
  magNoise: { label: 'Noise', path: ['noise'], min: 0, max: 0.1, step: 0.002, u: '% of field', dp: 1, k: 100 },
  softIron: { label: 'Soft-iron distortion (σ)', path: ['softIron'], min: 0, max: 0.3, step: 0.005, u: '%', dp: 1, k: 100 },
  hardIron: { label: 'Hard-iron offset (σ)', path: ['hardIron'], min: 0, max: 0.5, step: 0.01, u: '% of field', dp: 0, k: 100 },
  interference: { label: 'Motor interference', path: ['interference'], min: 0, max: 3, step: 0.1, u: '×', dp: 1 },
  baroNoise: { label: 'Noise', path: ['noise'], min: 0, max: 0.5, step: 0.01, u: 'm', dp: 2 },
  baroDrift: { label: 'Drift', path: ['drift'], min: 0, max: 0.05, step: 0.001, u: 'm/√s', dp: 3 },
  fixNoise: { label: 'Noise', path: ['noise'], min: 0, max: 1, step: 0.001, u: 'm', dp: 3 },
  wander: { label: 'Wandering error (σ)', path: ['wander'], min: 0, max: 3, step: 0.01, u: 'm', dp: 2 },
  rateFlow: { label: 'Sample rate', path: ['rate'], min: 10, max: 400, step: 10, u: 'Hz', dp: 0 },
  flowNoise: { label: 'Flow noise (good texture)', path: ['noise'], min: 0, max: 0.3, step: 0.005, u: 'rad/s', dp: 3 },
  flowScale: { label: 'Scale error (σ)', path: ['scale'], min: 0, max: 0.1, step: 0.005, u: '%', dp: 1, k: 100 },
  flowMax: { label: 'Max flow rate', path: ['maxRate'], min: 1, max: 15, step: 0.5, u: 'rad/s', dp: 1 },
  rangeMax: { label: 'Rangefinder max range', path: ['maxRange'], min: 0.5, max: 40, step: 0.5, u: 'm', dp: 1 },
  rangeNoise: { label: 'Rangefinder noise', path: ['rangeNoise'], min: 0, max: 0.1, step: 0.002, u: 'm', dp: 3 },
  velNoise: { label: 'Velocity noise', path: ['velNoise'], min: 0, max: 0.5, step: 0.01, u: 'm/s', dp: 2 },
};
const FIX_TUNED = ['rateFix', 'lat', 'fixNoise', 'wander', 'velNoise'];
const getP = (o, p) => p.reduce((a, k) => a[k], o);
function setP(o, p, v) { const last = p[p.length - 1]; p.slice(0, -1).reduce((a, k) => a[k], o)[last] = v; }
const fmtV = (v, d) => (d.k ? v * d.k : v).toFixed(d.dp) + ' ' + d.u;
const openSet = new Set();
// How a servo is mounted: which way it swings what it carries (see swingOf), relative to what it's on.
const swingTag = j => { const p = swingPreset(j), w = swingOf(j); return 'swings ' + (p ? p.label.toLowerCase() : `${w.swing.toFixed(0)}°`) + (Math.abs(w.lean) > 0.5 ? `, lean ${w.lean.toFixed(0)}°` : ''); };
const ROD_PRESETS = [['down', 'Straight down', 0, -90], ['fwd', 'Forward', 0, 0], ['back', 'Back', 180, 0], ['left', 'Left', 90, 0], ['right', 'Right', -90, 0], ['up', 'Straight up', 0, 90], ['custom', 'Custom direction', null, null]];
const near = (a, b) => Math.abs(((a - b + 540) % 360) - 180) < 0.5;
function presetOf(list, az, el) { const p = list.find(([k, , a, e]) => k !== 'custom' && Math.abs(el - e) < 0.5 && (Math.abs(e) > 89.5 || near(az, a))); return p ? p[0] : 'custom'; }
function linkPointing(l) { const k = presetOf(ROD_PRESETS, l.az, l.el); return k === 'custom' ? `${l.az}° / ${l.el}°` : ROD_PRESETS.find(p => p[0] === k)[1].toLowerCase(); }
function summary(c) {
  const on = parentOf(c), p = `(${c.pos[0].toFixed(2)}, ${c.pos[1].toFixed(2)}, ${c.pos[2].toFixed(2)})` + (on ? ` · on ${on.name}` : '');
  if (c.type === 'link') { const n = descendants(c).length; return `${Math.round(c.length * 100)} cm · ${linkPointing(c)} · carries ${n}${on ? ' · on ' + on.name : ''}`; }
  if (c.type === 'motor') return `${c.tmax.toFixed(1)} N · ${c.push ? 'pusher · ' : ''}${c.spin > 0 ? 'CCW' : 'CW'} · ${p}${c.health < 100 ? ' · ' + c.health + '%' : ''}`;
  if (c.type === 'joint') { const n = descendants(c).length; return `${swingTag(c)} · ${steerJoints().includes(c) ? 'steering ±' + c.range + '°' : 'set to ' + c.manual + '°'} · carries ${n} part${n === 1 ? '' : 's'} · ${p}`; }
  if (c.type === 'mass') return `${c.mass.toFixed(2)} kg ${c.shape}${c.known ? '' : ' · unknown'} · ${p}`;
  if (c.type === 'sensor') {
    const u = (c.known ? '' : ' · mount unknown'), g = Math.round(c.mass * 1000) + ' g · ';
    if (c.kind === 'imu') return `${g}${c.rate} Hz · gyro ±${c.gyroNoise.toFixed(2)}°/s${u} · ${p}`;
    if (c.kind === 'mag') return `${g}${c.rate} Hz · ×${c.interference.toFixed(1)} interference${u} · ${p}`;
    if (c.kind === 'baro') return `${g}${c.rate} Hz · ±${c.noise.toFixed(2)} m${u} · ${p}`;
    if (c.kind === 'flow') return `${g}${c.rate} Hz · range ${c.maxRange} m${u} · ${p}`;
    return g + `${c.quality === 'custom' ? 'Custom' : FIX_QUALITY[c.quality].label} · ${c.rate} Hz · ${c.latency} ms${c.dropout ? ' · no fix' : ''}${u}`;
  }
  return `${c.mass.toFixed(2)} kg on ${c.length.toFixed(2)} m${c.known ? '' : ' · unknown'}`;
}
// A labelled value with a slider and a box you can type into. The slider covers the usual range;
// typed values may go further, up to hmin/hmax. `get`/`set` work in SI units, the box shows d.k × value.
// A typed value outside the limits is clamped, and a note beside the box says so for a moment. d.ends: words
// for the two ends of the slider (the scale), shown under it. d.int: whole numbers only.
function numField(id, d, get, set) {
  const k = d.k || 1, lo = d.hmin ?? d.min, hi = d.hmax ?? d.max, inK = x => +(x * k).toFixed(6);
  const show1 = x => String(+(x * k).toFixed(d.dp));
  const onGrid = x => Math.abs(x / d.step - Math.round(x / d.step)) < 1e-6;   // (a min off the step grid would move where the arrow keys step to)
  const num = el('input', { type: 'number', class: 'num', id: id + '-n', step: String(inK(d.step)), min: onGrid(lo) ? String(inK(lo)) : null, max: String(inK(hi)), 'aria-label': `${d.label}${d.u ? ' in ' + d.u : ''}`, inputmode: 'decimal' });
  const rng = el('input', { type: 'range', id, min: d.min, max: d.max, step: d.step });
  const note = el('span', { class: 'clampnote', role: 'status' }); let noteT = 0;
  const say = t => { clearTimeout(noteT); note.textContent = t; if (t) noteT = setTimeout(() => { note.textContent = ''; }, 2500); };
  const show = v => { num.value = show1(v); rng.value = String(v); };
  show(get());
  rng.addEventListener('input', () => { const v = parseFloat(rng.value); num.value = show1(v); set(v); });
  num.addEventListener('input', () => {
    const t = num.value; let v = parseFloat(t) / k; if (t === '' || !isFinite(v)) return;
    let why = '';
    if (v > hi) { v = hi; why = `max ${show1(hi)}`; } else if (v < lo) { v = lo; why = `min ${show1(lo)}`; }
    if (d.int && Math.round(v) !== v) { v = Math.round(v); why = why || 'whole numbers'; }
    say(why); rng.value = String(v); set(v);
  });
  num.addEventListener('change', () => show(get()));                       // tidy the box once typing is done
  num.addEventListener('keydown', e => {
    if (e.key === 'Enter') num.blur();
    else if (e.key === 'Escape') { show(get()); num.blur(); }
  });
  const refresh = () => { if (document.activeElement !== num) show(get()); };
  const why = el('span', { class: 'field-why' });
  const ends = d.ends ? el('div', { class: 'ends', id: id + '-ends' }, el('span', { text: d.ends[0] }), el('span', { text: d.ends[1] })) : null;
  if (ends) rng.setAttribute('aria-describedby', id + '-ends');
  const node = el('div', { class: 'field' }, el('label', { for: id, text: d.label }), el('span', { class: 'numwrap' }, note, num, el('span', { class: 'unit', text: d.u })), rng, ends, why);
  // switched off (not used here), with the reason under it
  const setOff = (off, txt = '') => { off = !!off; if (num.disabled === off && why.textContent === (off ? txt : '')) return; node.classList.toggle('off', off); num.disabled = rng.disabled = off; setText(why, off ? txt : ''); };
  return { node, refresh, setOff };
}
// A servo's swing, relative to what it's mounted on: quick picks, then the exact angle and lean.
function hingeFields(c, rerender) {
  const pre = swingPresets(c), cur = swingPreset(c);
  const seg = el('div', { class: 'seg seg-sm seg-fill', role: 'group', 'aria-label': 'Swings' });
  for (const o of pre) {
    const b = el('button', { type: 'button', 'aria-pressed': String(!!cur && cur.k === o.k), text: o.label, 'data-focus-key': `swing-${c.id}-${o.k}` });
    b.addEventListener('click', () => { setSwing(c, o.deg, 0); edited(c, 'swing'); rerender(); });
    seg.append(b);
  }
  const f = (key, label, min, max, get, set) => {
    const r = numField(`f-${c.id}-${key}`, { label, min, max, step: 1, u: '°', dp: 0 }, get, v => { set(v); edited(c, 'swing'); });
    if (!cardRefresh.has(c.id)) cardRefresh.set(c.id, []); cardRefresh.get(c.id).push(r.refresh);
    return r.node;
  };
  return [el('div', { class: 'field stack' }, el('span', { class: 'flab', text: 'Swings' }), seg),
    f('swingdeg', 'Swing direction', -180, 180, () => swingOf(c).swing, v => setSwing(c, v)),
    f('swinglean', 'Hinge lean (cone sweep)', -80, 80, () => swingOf(c).lean, v => setSwing(c, swingOf(c).swing, v)),
    el('p', { class: 'hint', text: `Which way it swings what it carries. Relative to ${mountName(c)}: 0° swings toward ${swingRefName(c)}, and it stays that way when ${mountName(c)} moves.` })];
}
const cardRefresh = new Map();   // component id -> functions that redraw its open card's values
function slider(c, key) {
  const d = FD[key];
  const f = numField(`f-${c.id}-${key}`, d, () => getP(c, d.path), v => { setP(c, d.path, v); edited(c, key); });
  if (!cardRefresh.has(c.id)) cardRefresh.set(c.id, []); cardRefresh.get(c.id).push(f.refresh);
  return f.node;
}
function refreshCard(c) {
  for (const f of cardRefresh.get(c.id) || []) f();
  const s = document.querySelector(`[data-id="${c.id}"] .comp-sum`); if (s) s.textContent = summary(c);
}
function selectF(c, key, label, opts, onchg) {
  const id = `f-${c.id}-${key}`; const s = el('select', { id });
  for (const [v, t] of opts) { const o = el('option', { value: v, text: t }); if (String(c[key]) === String(v)) o.selected = true; s.append(o); }
  s.addEventListener('change', () => { const v = s.value; c[key] = v === 'true' ? true : v === 'false' ? false : isNaN(+v) ? v : +v; edited(c, key); if (onchg) onchg(); });
  return el('div', { class: 'field' }, el('label', { for: id, text: label }), s);
}
function checkF(c, key, label) {
  const id = `f-${c.id}-${key}`; const i = el('input', { type: 'checkbox', id }); i.checked = !!c[key];
  i.addEventListener('change', () => { c[key] = i.checked; edited(c, key); });
  return el('label', { class: 'check', for: id }, i, label);
}
function compBody(c) {
  cardRefresh.set(c.id, []);
  const b = el('div', { class: 'comp-body' });
  const nid = `f-${c.id}-name`; const ni = el('input', { type: 'text', id: nid, value: c.name, maxlength: '18' });
  ni.addEventListener('input', () => { c.name = ni.value || tagOf(c); document.querySelector(`[data-id="${c.id}"] .comp-name`).textContent = c.name; buildActRows(); save(); });
  b.append(el('div', { class: 'field' }, el('label', { for: nid, text: 'Name' }), ni));
  // Attached to: the frame, or any servo joint that isn't this part or below it.
  const holders = [['', 'Frame']].concat(cfg.comps.filter(h => canAttach(c, h)).map(h => [String(h.id), `${h.name} (${h.type === 'link' ? 'rod end' : 'servo'})`]));
  const aid = `f-${c.id}-parent`, asel = el('select', { id: aid });
  for (const [v, t] of holders) { const o = el('option', { value: v, text: t }); if (String(c.parent ?? '') === v) o.selected = true; asel.append(o); }
  asel.addEventListener('change', () => { attachTo(c, asel.value ? compById(+asel.value) : null); if (c.type === 'hang') reseatPend(c); structural(); });
  b.append(el('div', { class: 'field' }, el('label', { for: aid, text: 'Attached to' }), asel));
  const pos = el('div', { class: 'subgrid' }, slider(c, 'x'), slider(c, 'y'), slider(c, 'z'));
  if (parentOf(c) || isHolder(c)) b.append(el('p', { class: 'hint', text: 'Positions are body axes with every servo at 0°; the servos above a part carry it from there. Moving or turning a servo or rod carries what\'s on it.' }));
  const presetSel = (list, kAz, kEl, label) => {   // quick directions, with the exact angles below
    const id = `f-${c.id}-${kAz}-preset`, sel = el('select', { id }), cur = presetOf(list, c[kAz], c[kEl]);
    for (const [k, t] of list) { const o = el('option', { value: k, text: t }); if (k === cur) o.selected = true; sel.append(o); }
    sel.addEventListener('change', () => { const p = list.find(x => x[0] === sel.value); if (p[2] == null) return; c[kAz] = p[2]; c[kEl] = p[3]; edited(c, kAz); rerender(); });
    return el('div', { class: 'field' }, el('label', { for: id, text: label }), sel);
  };
  const spinSel = () => selectF(c, 'spin', 'Spin, facing the prop', [[1, 'CCW'], [-1, 'CW']]);
  const pushSel = () => selectF(c, 'push', 'Prop', [['false', 'Pulls (tractor)'], ['true', 'Pushes (pusher)']], rerender);
  const rerender = () => keepFocus(() => { document.querySelector(`[data-id="${c.id}"]`).replaceWith(compCard(c)); });
  if (c.type === 'motor') {
    b.append(pos, slider(c, 'tilt'), slider(c, 'az'), slider(c, 'tmax'), slider(c, 'prop'), pushSel(), spinSel(), slider(c, 'kappa'), selectF(c, 'pitch', 'Blade pitch', [['fixed', 'Fixed: speed sets thrust'], ['collective', 'Collective: governed speed, pitch sets thrust']]), slider(c, 'tau'), slider(c, 'fm'), slider(c, 'mass'), slider(c, 'health'), checkF(c, 'healthKnown', 'Controller knows the health'),
      el('span', { class: 'lbl', text: 'Heat, sensing and failure' }),
      checkF(c, 'tsens', 'Temperature sensor on the motor'), checkF(c, 'telem', 'ESC telemetry (reports rpm and current)'),
      slider(c, 'tmaxC'), slider(c, 'cool'), checkF(c, 'failHeat', 'Overheating damages it'),
      selectF(c, 'failMode', 'When it fails', [['stop', 'It stops'], ['loss', 'It loses thrust']], rerender));
    if (c.failMode === 'loss') b.append(slider(c, 'failLoss'));
    b.append(el('p', { class: 'hint', text: 'It heats from the current in its windings and cools faster with the prop spinning. Past its limit its magnet weakens for good; 35 °C past it, it fails the way you set here. You can also break it from the Health panel while flying. The sensors are what the supervisor has to go on: without a temperature sensor it estimates the heat from the ESC\'s current, and without either it can\'t see it.' }),
      el('p', { class: 'hint', text: 'The shaft points from the motor to the prop. A puller\'s thrust points along it, toward the prop; a pusher\'s prop is pitched the other way, so its thrust points back toward the motor and it blows air away past the prop. The motor, ESC and prop are simulated from these: prop speed, current and torque, spin-up and spin-down, the throttle curve and the battery sag all follow. Hidden values are real hardware traits the controller isn\'t told. Calibrate measures them.' }));
  } else if (c.type === 'joint') {
    const carried = descendants(c), steer = motorsUnder(c).length > 0;
    b.append(el('p', { class: 'hint', text: carried.length ? 'Carries: ' + carried.map(x => x.name).join(', ') + '.' : 'Nothing is attached yet. Set a part\'s "Attached to" to this servo.' }));
    b.append(el('span', { class: 'lbl', text: 'Pivot' }), pos, el('span', { class: 'lbl', text: 'Swing' }), ...hingeFields(c, rerender),
      el('span', { class: 'lbl', text: 'Control' }),
      selectF(c, 'mode', 'Servo control', [['auto', steer ? 'Allocator steers it' : 'Allocator steers it (needs a motor on it)'], ['manual', 'Set by me']], rerender));
    if (c.mode === 'manual' || !steer) b.append(slider(c, 'manual'));
    b.append(slider(c, 'range'), slider(c, 'rate'), el('span', { class: 'lbl', text: 'Servo hardware' }),
      slider(c, 'storque'), slider(c, 'slag'), slider(c, 'offset'), checkF(c, 'feedback', 'Servo reports its angle (feedback)'), slider(c, 'jmass'),
      selectF(c, 'failMode', 'When it fails', [['jam', 'It jams where it is'], ['limp', 'It goes limp']]),
      el('p', { class: 'hint', text: 'Speed is no-load; under load it runs slower, and a heavy load or thrust on an arm can hold it off its target (stall torque). Hidden values are traits the controller isn\'t told. Calibrate measures them.' }));
  } else if (c.type === 'link') {
    const carried = descendants(c);
    b.append(el('p', { class: 'hint', text: carried.length ? 'At its far end: ' + carried.map(x => x.name).join(', ') + '.' : 'A stick or lever. Attach parts to it (drag them onto it in the list) and they ride at its far end.' }),
      el('span', { class: 'lbl', text: 'Base' }), pos, presetSel(ROD_PRESETS, 'az', 'el', 'Points'), slider(c, 'laz'), slider(c, 'lel'), slider(c, 'lroll'), slider(c, 'llen'), slider(c, 'mass'),
      checkF(c, 'known', 'Controller knows this rod\'s mass'));
  } else if (c.type === 'mass') {
    b.append(selectF(c, 'shape', 'Shape', [['box', 'Box'], ['sphere', 'Sphere'], ['cylinder', 'Cylinder (vertical)']], rerender), slider(c, 'mass'), pos);
    if (c.shape === 'box') b.append(el('div', { class: 'subgrid' }, slider(c, 'lx'), slider(c, 'ly'), slider(c, 'lz')));
    else if (c.shape === 'sphere') b.append(slider(c, 'radius')); else b.append(slider(c, 'radius'), slider(c, 'length'));
    b.append(checkF(c, 'known', 'Controller knows this mass'));
  } else if (c.type === 'sensor') {
    const mount = el('div', { class: 'subgrid' }, slider(c, 'mr'), slider(c, 'mp'), slider(c, 'my'));
    if (c.kind === 'imu') b.append(pos, mount, slider(c, 'rateImu'), slider(c, 'latImu'), slider(c, 'gyroNoise'), slider(c, 'gyroBias'), slider(c, 'gyroDrift'),
      selectF(c, 'gyroRange', 'Gyro range', [[250, '±250 °/s'], [500, '±500 °/s'], [1000, '±1000 °/s'], [2000, '±2000 °/s']]),
      slider(c, 'accNoise'), slider(c, 'accBias'), selectF(c, 'accRange', 'Accel range', [[2, '±2 g'], [4, '±4 g'], [8, '±8 g'], [16, '±16 g']]), slider(c, 'scaleErr'), slider(c, 'misalign'), slider(c, 'vib'));
    else if (c.kind === 'mag') b.append(pos, mount, slider(c, 'rateMag'), slider(c, 'lat'), slider(c, 'magNoise'), slider(c, 'hardIron'), slider(c, 'softIron'), slider(c, 'interference'));
    else if (c.kind === 'baro') b.append(pos, slider(c, 'rateBaro'), slider(c, 'lat'), slider(c, 'baroNoise'), slider(c, 'baroDrift'));
    else if (c.kind === 'flow') b.append(el('p', { class: 'hint', text: 'Looks along its own −Z (down, with no mount rotation).' }), pos, mount, slider(c, 'rateFlow'), slider(c, 'lat'), slider(c, 'flowNoise'), slider(c, 'flowScale'), slider(c, 'flowMax'), slider(c, 'rangeMax'), slider(c, 'rangeNoise'));
    else {
      const q = selectF(c, 'quality', 'Type', [['gps', 'GPS'], ['rtk', 'RTK GPS'], ['mocap', 'Motion capture'], ['custom', 'Custom']], () => {
        if (c.quality !== 'custom') Object.assign(c, fixDefaults(c.quality)); rerender(); save();
      });
      b.append(q, el('p', { class: 'hint', text: 'Antenna or marker position:' }), pos, slider(c, 'rateFix'), slider(c, 'lat'), slider(c, 'fixNoise'), slider(c, 'wander'), slider(c, 'velNoise'), checkF(c, 'dropout', 'Signal lost (no fix)'));
    }
    b.append(slider(c, 'smass'), checkF(c, 'known', c.kind === 'baro' ? 'Controller knows the position' : c.kind === 'fix' ? 'Controller knows the antenna position' : 'Controller knows the position and mount'));
  } else {
    b.append(el('p', { class: 'hint', text: 'Attachment point:' }), pos, slider(c, 'cable'), slider(c, 'mass'), checkF(c, 'known', 'Controller knows the static load'));
  }
  return b;
}
function compCard(c) {
  const open = openSet.has(c.id);
  const head = el('button', { class: 'comp-head', type: 'button', 'aria-expanded': String(open), 'data-focus-key': 'head-' + c.id }, el('span', { class: 'tag tag-' + c.type, text: tagOf(c) }), el('span', { class: 'comp-name', text: c.name }), el('span', { class: 'comp-sum', text: summary(c) }));
  head.addEventListener('click', () => {
    if (typeof editMode !== 'undefined' && editMode && edit.sel !== c.id) { edit.refocus = true; selectComp(c.id); return; }   // in edit mode a card click selects the part
    open ? openSet.delete(c.id) : openSet.add(c.id); keepFocus(() => document.querySelector(`[data-id="${c.id}"]`).replaceWith(compCard(c)));
  });
  const del = el('button', { class: 'icon-btn', type: 'button', 'aria-label': 'Remove ' + c.name, title: 'Remove', text: '×', 'data-focus-key': 'del-' + c.id });
  del.addEventListener('click', () => {   // parts on a removed joint move to what the joint was on
    for (const x of cfg.comps) if (x.parent === c.id) x.parent = c.parent ?? null;
    cfg.comps = cfg.comps.filter(x => x !== c); openSet.delete(c.id); structural();
  });
  const selected = typeof edit !== 'undefined' && edit.sel === c.id;
  const grip = el('span', { class: 'grip', title: 'Drag onto a servo or rod to attach', 'aria-hidden': 'true', text: '⠿' });
  const top = el('div', { class: 'comp-top', draggable: 'true' }, grip, head, del);
  top.addEventListener('dragstart', e => { dragId = c.id; e.dataTransfer.effectAllowed = 'move'; e.dataTransfer.setData('text/plain', String(c.id)); requestAnimationFrame(() => top.closest('.comp').classList.add('dragging')); });
  top.addEventListener('dragend', () => { dragId = null; document.querySelectorAll('.dragging,.drop-ok').forEach(x => x.classList.remove('dragging', 'drop-ok')); });
  const card = el('div', { class: 'comp' + (open ? ' open' : '') + (selected ? ' sel' : ''), 'data-id': c.id }, top, open ? compBody(c) : null);
  // a servo or rod: a drop target, and the twisty that folds what it carries (here, so a re-rendered card keeps them)
  if (isHolder(c)) dropTarget(card, c);
  const kids = childrenOf(c);
  if (kids.length) {
    const folded = foldSet.has(c.id);
    const tw = el('button', { class: 'twisty', type: 'button', 'aria-expanded': String(!folded), title: folded ? 'Show what it carries' : 'Hide what it carries', 'aria-label': (folded ? 'Show what ' : 'Hide what ') + c.name + ' carries', text: folded ? `▸ ${kids.length}` : '▾', 'data-focus-key': 'tw-' + c.id });
    tw.addEventListener('click', () => { folded ? foldSet.delete(c.id) : foldSet.add(c.id); renderComps(); });
    top.prepend(tw);
  }
  return card;
}
const foldSet = new Set();
let dragId = null;
function dropTarget(elm, holder) {   // accept a dragged part if it may be attached here
  const ok = () => { const c = compById(dragId); return c && (holder ? canAttach(c, holder) : !!parentOf(c)); };
  elm.addEventListener('dragover', e => { if (!ok()) return; e.preventDefault(); e.stopPropagation(); elm.classList.add('drop-ok'); });
  elm.addEventListener('dragleave', () => elm.classList.remove('drop-ok'));
  elm.addEventListener('drop', e => {
    elm.classList.remove('drop-ok'); if (!ok()) return; e.preventDefault(); e.stopPropagation();
    const c = compById(dragId); dragId = null; attachTo(c, holder); if (c.type === 'hang') reseatPend(c);
    if (holder) foldSet.delete(holder.id); structural();
  });
}
function renderComps() { keepFocus(renderComps1); }
function renderComps1() {
  const L = $('#compList'); L.textContent = '';
  // The parts as a tree: the frame at the root, each servo or rod followed by what it carries. Drag a part
  // onto a servo or rod to attach it there, or onto Frame to take it off.
  const frameRow = el('div', { class: 'tree-frame', 'data-drop': 'frame', text: 'Frame' });
  dropTarget(frameRow, null); L.append(frameRow);
  const put = (c, box) => {
    const node = el('div', { class: 'node' }), card = compCard(c), kids = childrenOf(c);
    node.append(card); box.append(node);
    if (kids.length && !foldSet.has(c.id)) { const kb = el('div', { class: 'kids' }); node.append(kb); for (const k of kids) put(k, kb); }
  };
  const top = el('div', { class: 'kids root' }); L.append(top);
  for (const c of cfg.comps.filter(x => !parentOf(x))) put(c, top);
  const na = actuators().length, nj = joints().length, ns = allSensors().length, np = cfg.comps.length - na - nj - ns;
  $('#compCount').textContent = `${na} motor${na === 1 ? '' : 's'} · ${nj} servo${nj === 1 ? '' : 's'} · ${ns} sensor${ns === 1 ? '' : 's'} · ${np} other`;
}
function edited(c, key) {
  undoKey = `${c.id}:${key}`;   // repeated edits to the same field (a slider or handle drag) are one undo step
  if (c.type === 'sensor' && c.kind === 'fix' && FIX_TUNED.includes(key) && c.quality !== 'custom') {
    c.quality = 'custom'; const q = document.getElementById(`f-${c.id}-quality`); if (q) q.value = 'custom';
  }
  if (isHolder(c)) carryAlong(c);
  const s = document.querySelector(`[data-id="${c.id}"] .comp-sum`); if (s) s.textContent = summary(c);
  recomputeProps(); if (c.type === 'hang' && (key === 'cable' || key === 'x' || key === 'y' || key === 'z')) reseatPend(c);
  cPts = contactPoints(); rebuildDrone(); refreshEnvelope(); renderMass(); save();
  if (typeof edit !== 'undefined' && editMode && edit.sel === c.id && c.type === 'joint') updateEditMsg();   // keep the edit bar's servo tools in step
}
// Last seen place of every servo and rod, so editing one carries what's on it along.
const holderSnap = new WeakMap();
function snapHolder(c) { if (isHolder(c)) holderSnap.set(c, { pos: c.pos.slice(), dir: c.type === 'link' ? linkDir(c) : null, F: c.type === 'link' ? rodFrameOf(c) : null, len: c.length }); }
function carryAlong(c) {
  const s0 = holderSnap.get(c); if (!s0) { snapHolder(c); return; }
  const d = sub(c.pos, s0.pos);
  if (nrm(d) > 1e-9) shiftSubtree(c, d);
  if (c.type === 'link') {
    const dir = linkDir(c);
    const F = rodFrameOf(c); if (F.some((v, i) => Math.abs(v - s0.F[i]) > 1e-9)) rotateSubtree(c, m3m(F, m3T(s0.F)), c.pos);   // what's on it keeps its place and angle relative to the rod
    if (Math.abs(c.length - s0.len) > 1e-9) shiftSubtree(c, scl(dir, c.length - s0.len));
  }
  snapHolder(c); for (const k of descendants(c)) snapHolder(k);
  for (const k of descendants(c)) { refreshCard(k); if (k.type === 'hang') reseatPend(k); }
}
function structural() {
  for (const c of cfg.comps) snapHolder(c); recomputeProps(); cPts = contactPoints(); rebuildDrone(); renderComps(); buildActRows(); refreshEnvelope(); renderMass(); save(); }
function addComp(type) {
  const n = cfg.comps.filter(c => c.type === type).length + 1; let c;
  if (type === 'motor') c = mkMotor('Motor ' + n, 0.3, 0, 0.02);
  else if (type === 'link') c = mkLink('Rod ' + (links().length + 1), 0, 0, -0.03);
  else if (type === 'joint') c = mkJoint('Servo ' + (joints().length + 1), -0.3, 0, 0.02, { hingeAz: 90 });
  else if (type === 'tilt') { const k = joints().length + 1, pr = mkServoMotor('Rotor ' + k, -0.3, 0, 0.02, { hingeAz: 90 }); cfg.comps.push(pr[0]); c = pr[1]; }
  else if (type === 'mass') c = mkMass('Mass ' + n, 0.1, 0, -0.04, { mass: 0.15 });
  else if (type === 'hang') c = mkHang('Cable ' + n, 0, 0, -0.03);
  else {
    const k = cfg.comps.filter(x => x.type === 'sensor' && x.kind === type).length + 1;
    const at = { imu: [0.05, 0, 0.01], mag: [0.1, 0, 0.05], baro: [-0.03, -0.02, 0.005], fix: [-0.05, 0, 0.09], flow: [0, -0.03, -0.03] }[type];
    c = mkSensor(type, SENSOR_KINDS[type] + ' ' + k, ...at);
  }
  cfg.comps.push(c); openSet.add(c.id); structural();
  requestAnimationFrame(() => { const card = document.querySelector(`[data-id="${c.id}"]`); if (card) card.scrollIntoView({ block: 'nearest' }); });
}
document.querySelectorAll('[data-add]').forEach(b => b.addEventListener('click', () => addComp(b.dataset.add)));

/* ───────── telemetry ───────── */
let actRows = new Map(), envRes = null;
function buildActRows() {
  const box = $('#acts'); box.textContent = ''; actRows = new Map();
  const acts = actuators(); if (!acts.length) { box.append(el('p', { class: 'hint', text: 'No actuators attached.' })); return; }
  for (const c of acts) {
    const fill = el('div', { class: 'fill' }), mk = el('div', { class: 'act' }); const val = el('span', { class: 'av' });
    box.append(el('div', { class: 'arow' }, el('span', { class: 'an', text: c.name, title: c.name }), el('div', { class: 'tbar' }, fill, mk), val));
    actRows.set(c.id, { fill, mk, val });
  }
  for (const j of joints()) {   // servo joints: where the angle sits in its range (middle = 0°)
    const mk = el('div', { class: 'act' }), cmd = el('div', { class: 'act cmd' }); const val = el('span', { class: 'av' });
    box.append(el('div', { class: 'arow jrow' }, el('span', { class: 'an', text: j.name, title: j.name }), el('div', { class: 'tbar jbar' }, cmd, mk), val));
    actRows.set(j.id, { mk, cmd, val, joint: true });
  }
}
const fmtSign = v => (v >= 0 ? '+' : '−') + Math.abs(v).toFixed(1);
function updateActs() {
  for (const c of actuators()) {
    const r = actRows.get(c.id), st = act.get(c.id); if (!r || !st) continue;
    const pc = clamp(st.u || 0, 0, 1), Te = st.Teff ?? st.T; r.fill.style.width = (pc * 100).toFixed(1) + '%'; r.fill.classList.toggle('sat', pc > 0.99);
    r.mk.style.left = `calc(${(clamp(Te / c.tmax, 0, 1) * 100).toFixed(1)}% - 1px)`;
    r.val.textContent = `${Math.round(pc * 100)}% · ${Te.toFixed(2)} N`;
    r.val.title = `Allowance: ${Math.round(Math.min(pc, 1 - pc) * 100)}% of the range left before a limit · prop ${Math.round((st.Omega || 0) * 60 / (2 * Math.PI))} rpm · ${(st.i || 0).toFixed(1)} A`;
  }
  for (const j of joints()) {
    const r = actRows.get(j.id), st = jst.get(j.id); if (!r || !st) continue;
    const R = j.range || 1, pos = a => `calc(${(clamp(0.5 + a * R2D / (2 * R), 0, 1) * 100).toFixed(1)}% - 1px)`, t = jointTarget(j);
    r.mk.style.left = pos(st.th); r.cmd.style.left = pos(t);
    r.val.textContent = `${fmtSign(st.th * R2D)}° → ${fmtSign(t * R2D)}°`;
    r.val.title = steerJoints().includes(j) ? 'Actual angle → commanded' : 'Actual angle → the angle you set';
  }
}
// Allocation preference sliders, and a live readout of rotor power and the tightest allowance.
const allocFieldRefs = [], allocOff = {};
function buildAllocFields() {
  const box = $('#allocFields'); box.textContent = '';
  const f = (key, label) => { const n = numField('ap-' + key, { label, min: 0, max: 0.2, step: 0.005, u: '', dp: 3, ends: ['0 off', '0.2 strongest'] }, () => allocPrefs[key], v => { allocPrefs[key] = v; save(); }); allocFieldRefs.push(n.refresh); allocOff[key] = n.setOff; return n.node; };
  box.append(f('allowance', 'Keep margin (allowance)'), f('efficiency', 'Save power (efficiency)'), f('servoMove', 'Servo move cost (uses speed and lag)'));
  const m = numField('ap-mix', { label: 'Mixed steering: servos\' share of sideways force', min: 0, max: 1, step: 0.05, u: '%', dp: 0, k: 100, ends: ['0: the body leans', '100: servos only'] }, () => steerMix.share, v => { steerMix.share = v; steerMix.rho = 1; save(); });
  allocFieldRefs.push(m.refresh); allocOff.mix = m.setOff; box.append(m.node);
}
function syncAllocFields() {   // what doesn't apply to this airframe or steering is greyed out, with why
  const nj = joints().length, steer = steerJoints().length;
  allocOff.servoMove(!nj, 'No servos on this airframe.');
  allocOff.mix(mode !== 'mixed' || !steer, mode !== 'mixed' ? 'Used only when Steering (top bar) is Mixed.' : 'No servos steer this airframe.');
}
function updateAllocInfo() {
  let P = (S.battV || 0) * (S.battI || 0), tight = null;   // electrical power from the pack
  for (const c of actuators()) {
    const st = act.get(c.id); if (!st) continue;
    const m = Math.min(st.u || 0, 1 - (st.u || 0)); if (!tight || m < tight.m) tight = { c, m };
  }
  for (const j of steerJoints()) { const st = jst.get(j.id); if (!st) continue; const ms = (j.range * D2R - Math.abs(st.th)) / (2 * j.range * D2R); if (!tight || ms < tight.m) tight = { c: j, m: ms, servo: true }; }
  setText($('#allocSmall'), (tight ? `≈ ${Math.round(P)} W · tightest ${tight.c.name} ${Math.round(tight.m * 100)}%` : '') +
    (mode === 'mixed' ? ` · servos take ${Math.round(mixShare() * 100)}% sideways` : ''));
}
// The check can take tens of milliseconds for a layout with several servo rotors, so a burst of edits (a
// slider or handle drag) runs it once, after the burst settles, instead of on every step.
let envTimer = null;
function refreshEnvelope() {
  clearTimeout(envTimer);
  envTimer = setTimeout(() => { envTimer = null; try { envRes = envelopeCalc(); } catch (e) { envRes = null; } renderEnvelope(); }, 60);
}
function renderEnvelope() {
  const r = envRes; if (!r) return;
  const p = $('#verdict'), pc = 'pill ' + r.verdict; if (p.className !== pc) p.className = pc; setText(p.querySelector('span'), r.title); setText($('#verdictWhy'), r.why);
  setText($('#envSpace'), r.k === 6 ? '6-axis (stay level)' : mode === 'mixed' ? '4-axis (mixed)' : '4-axis (tilt body)');
  const box = $('#env'), sig = r.head ? r.labels.join('|') : '-';
  if (box._sig !== sig) {   // the rows are built once per set of axes; the bars and numbers update in place
    box._sig = sig; box.textContent = ''; box._rows = [];
    if (!r.head) box.append(el('p', { class: 'hint', text: 'No headroom to show until every axis is controllable.' }));
    else for (const lab of r.labels) {
      const mk = cls => { const b = el('div', { class: 'b' }), t = el('span', { class: 't' }); return { n: el('div', { class: 'ebar ' + cls }, b, t), b, t }; };
      const neg = mk('eneg'), pos = mk('epos'); box._rows.push([neg, pos]);
      box.append(el('div', { class: 'erow' }, el('span', { class: 'en', text: lab }), neg.n, pos.n));
    }
  }
  if (!r.head) return;
  r.labels.forEach((lab, i) => {
    const u = r.units[i];
    const fmt = t => !isFinite(t) ? '—' : u === 'g' ? (t / G).toFixed(2) + ' g' : u === 'lin' ? t.toFixed(1) + ' m/s²' : t.toFixed(lab === 'Yaw' ? 1 : 0) + ' rad/s²';
    const scale = u === 'g' ? 2 * G : u === 'lin' ? 6 : lab === 'Yaw' ? 20 : 200;
    const lim = u === 'g' ? 0.15 * G : u === 'lin' ? 0.5 : lab === 'Yaw' ? 0.5 : 5;
    const put = (t, ref, isPos) => {
      const w = clamp(Math.abs(isFinite(t) ? t : 0) / scale, 0, 1) * 100; let st = '';
      if (t < 0) st = ' bad'; else if (t < lim && !(u === 'g' && !isPos)) st = ' warn';
      if (ref.b.className !== 'b' + st) ref.b.className = 'b' + st; ref.b.style.width = w.toFixed(1) + '%'; setText(ref.t, fmt(t));
    };
    put(r.head[i][0], box._rows[i][0], false); put(r.head[i][1], box._rows[i][1], true);
  });
}
function renderMass() {
  const mp = cfg.comps.filter(c => c.type === 'hang').reduce((s, c) => s + c.mass, 0);
  const tw = actuators().reduce((s, c) => s + c.tmax * motorEff(c), 0) / ((truth.m + mp) * G);
  const cm = truth.c.map(x => (x * 1000).toFixed(0)).join(', '); const dc = nrm(sub(truth.c, model.c)) * 1000;
  const rows = [['Rigid mass', truth.m.toFixed(3) + ' kg'], ['On cables', mp.toFixed(3) + ' kg'], ['Thrust / weight', tw.toFixed(2)], ['True CoG from hub', `(${cm}) mm`],
    ["Controller's CoG error", dc.toFixed(0) + ' mm'], ['Controller mass error', ((model.m - truth.m - mp) * 1000).toFixed(0) + ' g'],
    ['Inertia Ixx / Iyy / Izz', `${(truth.J[0] * 1000).toFixed(1)} / ${(truth.J[4] * 1000).toFixed(1)} / ${(truth.J[8] * 1000).toFixed(1)} g·m²`]];
  syncKv($('#massKv'), rows);
}
const goForm = () => showTab('form');
function updateLive() {
  const chips = [], chip = (key, src, text, tone, go) => chips.push({ key, src, text, tone, go });
  if (S.crashed) chip('crash', 'sim', 'Crashed', 'bad'); else {
    const last = hist.err.length ? hist.err[hist.err.length - 1] : 0;
    if (hasTask('nav') && brt.pilot.phase === 'flying') chip('hold', 'sim', last < 60 ? 'Holding target' : 'Getting there', last < 60 ? 'good' : 'warn');   // GPS alone is good to a few tens of cm
    if (brt.out && brt.out.sat) chip('sat', 'board', 'Motor at limit', 'warn');
    if (pend.size && [...pend.values()].some(p => p.Tn <= 0.01)) chip('slack', 'sim', 'Cable slack', 'warn');
  }
  const ed = editedLaws(), bad = ed.filter(L => L.status === 'error');
  if (bad.length) chip('laws', 'you', `${bad.length} formula error${bad.length > 1 ? 's' : ''}`, 'bad', goForm);
  else if (ed.length) chip('laws', 'you', `${ed.length} formula${ed.length > 1 ? 's' : ''} edited`, 'accent', goForm);
  const soc = Math.max(0, S.batt.soc ?? 1); chip('batt', 'sim', `Battery ${Math.round(soc * 100)}% · ${(S.battV || 0).toFixed(1)} V`, soc < 0.25 ? 'bad' : soc < 0.5 ? 'warn' : '');
  if (brt.err && !brt.ready) chip('board', 'board', brt.err, 'bad', goForm);
  else chip('board', 'board', `${flightPhaseText()}${brt.fcState !== 1 && brt.fcWhy ? ' · ' + brt.fcWhy : ''}`, brt.fcState === 3 ? 'bad' : '');
  syncChips($('#liveChips'), chips);
  const R = qmat(S.q); const { hub } = hubState(R);
  setText($('#hudTime'), `t ${S.t.toFixed(1)} s · ${running ? 'running' : 'paused'}`);
  setText($('#hudPos'), `hub (${hub.map(x => x.toFixed(2)).join(', ')}) m`);
  const vh = hubState(R).vh, gs = Math.hypot(vh[0], vh[1]);
  setText($('#hudCmd'), `speed ${gs.toFixed(1)} m/s · climb ${fmtSign(vh[2])} m/s · heading ${Math.round(setpoint.yaw)}°`);
  {   // the net torque on the drone about its centre of mass, in body axes (roll: X forward, pitch: Y left, yaw: Z up)
    const t = S.tq, f = x => (x < 0 ? '−' : '+') + Math.abs(x).toFixed(3);
    setText($('#hudTq'), view.readouts && (view.rtorque || view.ntorque || view.want) && t && !S.crashed ? `torque · roll ${f(t[0])} · pitch ${f(t[1])} · yaw ${f(t[2])} N·m` : '');
  }
  $('#kbdHint').hidden = document.hasFocus();
  syncSp(); syncAllocFields();
  updateActs(); updateAllocInfo(); renderEst(); if (hasTask('learn')) renderLearn();
}

/* ───────── state estimate ───────── */
function setSensing(m) { sensing = 'sensors'; }   // the flight computers always fly on their sensors
function renderEst() {
  const chips = []; const chip = (t, c, src) => chips.push({ key: t, src: src || 'calc', text: t, tone: c });
  const has = k => sensorsOf(k).length > 0, fixOk = sensorsOf('fix').some(c => !c.dropout);
  const core = boardOf('core'), nav = boardOf('nav');
  if (!has('imu')) chip('No IMU: attitude is unknown', 'bad');
  if (!has('mag')) chip('No compass: heading drifts', 'warn');
  const fs = est.flowState;
  if (fs === 'tracking') chip('Optical flow tracking', 'good', 'sensor');
  else if (fs === 'range only') chip('Optical flow: nothing to track (texture or light)', 'warn', 'sensor');
  else if (fs === 'out of range') chip('Optical flow: out of rangefinder range', 'warn', 'sensor');
  if (!nav) chip('No navigation: angle mode, no position hold', 'warn');
  else {
    if (!fixOk) chip(fs === 'tracking' ? 'No GPS: holding with optical flow, slow drift' : has('fix') ? 'Position fix lost: position drifts' : 'No position fix: position drifts', fs === 'tracking' ? '' : 'warn', has('fix') ? 'sensor' : 'calc');
    if (!has('baro') && !fixOk && !(fs === 'tracking' || fs === 'range only')) chip('No altitude reference', 'warn');
    if (has('imu') && has('mag') && fixOk) chip('All references present', 'good');
  }
  syncChips($('#senseChips'), chips);
  setText($('#estMode'), `attitude: ${core ? core.name : '—'}${nav ? ' · position: ' + nav.name : ''}`);
  const e = estimateErrors(); const f = (v, d, u) => (v == null ? '—' : v.toFixed(d) + ' ' + u);
  const rows = [['Attitude error', f(e.ang, 2, '°')], ['Tilt error', f(e.tilt, 2, '°')], ['Heading error', f(e.head, 1, '°')],
    ['Horizontal position error', f(e.pos, 1, 'cm')], ['Altitude error', f(e.alt, 1, 'cm')], ['Velocity error', f(e.vel, 1, 'cm/s')],
    ['Gyro turn-on bias (IMU 1)', e.gb == null ? '—' : `${e.gb.toFixed(2)} °/s`]];
  syncKv($('#estKv'), rows);
}

/* ───────── the learning (a task on a board: boards.js) ───────── */
$('#useDesc').addEventListener('click', () => { pilotLearnCmd('useDesc'); renderLearn(true); });
$('#useLearned').addEventListener('click', e => { if (e.currentTarget.getAttribute('aria-disabled') === 'true') return; pilotLearnCmd('useLearned'); renderLearn(true); });
$('#keepLearn').addEventListener('change', e => { learnPrefs.keep = e.target.checked; pilotLearnCmd(e.target.checked ? 'keepOn' : 'keepOff'); save(); });
$('#calBtn').addEventListener('click', () => {
  const v = learn.view;
  if (v && v.cal) { pilotLearnCmd('stop'); renderLearn(true); return; }
  if (S.crashed) return;
  if (typeof editMode !== 'undefined' && editMode) setEditMode(false);
  if (!running) { running = true; renderRun(); }
  pilotLearnCmd('calibrate'); renderLearn(true);
});
$('#holdPulses').addEventListener('change', e => { learnPrefs.holdPulses = e.target.checked; pilotLearnCmd(e.target.checked ? 'holdOn' : 'holdOff'); save(); });
$('#thenCal').addEventListener('change', e => { throwCfg.thenCalibrate = e.target.checked; pilotLearnCmd(e.target.checked ? 'thenCalOn' : 'thenCalOff'); save(); });
function setLaunch(m, go = true) {
  launchMode = m;
  $('#launchHover').setAttribute('aria-pressed', String(m === 'hover')); $('#launchThrow').setAttribute('aria-pressed', String(m === 'throw'));
  $('#crashResetLbl').textContent = m === 'throw' ? 'Throw again' : 'Reset';
  if (go) { if (!running) { running = true; renderRun(); } doReset(); renderLearn(true); save(); }
}
$('#launchHover').addEventListener('click', () => setLaunch('hover')); $('#launchThrow').addEventListener('click', () => setLaunch('throw'));
// What's on screen follows the flight computers: learning and the throw start need a learning task, Hold and Home
// need navigation, the supervisor's parts of the Health panel need a supervisor task.
function syncFlightUi() {
  const nav = hasTask('nav'), learnT = hasTask('learn');
  $('#learnSec').hidden = !learnT;
  $('#launchSeg').hidden = !learnT; if (!learnT && launchMode !== 'hover') setLaunch('hover', false);
  document.querySelectorAll('[data-act="hold"],[data-act="home"]').forEach(b => { b.hidden = !nav; });
  if (typeof renderHealth === 'function') renderHealth(true);
  if (typeof renderComputers === 'function') renderComputers();
}
const throwFieldRefs = [];
function buildThrowFields() {
  const box = $('#throwFields'); box.textContent = '';
  const f1 = numField('throwH', { label: 'Throw height (top of the arc)', min: 2.5, max: 15, step: 0.1, u: 'm', dp: 1 }, () => throwCfg.height, v => { throwCfg.height = v; save(); });
  const f2 = numField('throwS', { label: 'Tumble when thrown', min: 0, max: 15, step: 0.5, u: 'rad/s', dp: 1 }, () => throwCfg.spin, v => { throwCfg.spin = v; save(); });
  box.append(f1.node, f2.node); throwFieldRefs.push(f1.refresh, f2.refresh);
}
function throwHintText() {
  const b = boardOf('learn'), w = b && brt.ready && brt.inst.get(b.id);
  if (!actuators().length) return 'Add actuators to throw it.';
  if (!w) return '';
  const T = w.learn_plan(), n = new Float32Array(w.memory.buffer, w.fr_ptr(), 1)[0];
  if (learn.n > RN_THROW_IN) return `The throw start identifies at most ${RN_THROW_IN} inputs; this airframe has ${learn.n}.`;
  const v = G * T / 2, need = setpoint.z + 0.5 * G * (T / 2) ** 2 + v * v / 10 + 1.5;
  return `This airframe has ${n} pulses to fire, about ${T.toFixed(1)} s around the top of the throw. Throw it to at least ${need.toFixed(1)} m so there is room to catch it. Each pulse stops early once the drone turns 4 rad/s faster, well inside the gyro's range.`;
}
function throwStageText() {
  const v = learn.view;
  if (thr && thr.phase === 'hand') return brt.pilot.phase === 'hand' && brt.navOut && !brt.navOut.ready ? 'In the hand, motors off; the navigation is finding its position' : 'In the hand, motors off';
  if (thr && thr.phase === 'toss') return 'Being thrown';
  if (!v) return '';
  if (v.thr === 2) return 'Thrown, climbing with the motors off';
  if (v.thr === 3) { const c = actuators()[v.pulseMotor]; return c ? `Free fall: pulsing ${c.name}` : 'Fitting the model'; }
  if (v.thr === 4) return 'Catching itself on what it learned';
  return '';
}
const CAL_STAGE = ['Settling', 'Testing each motor', 'Testing each servo', 'Sweeping servos', 'Exciting everything together', 'Validating'];
let matchT = 0, matchCache = [];
function renderResponses() {
  const box = $('#respRows'); if (!box) return;
  const v = learn.view; let any = false;
  const tbl = el('table', { class: 'resp' });
  tbl.append(el('tr', {}, el('th', { text: '' }), el('th', { text: 'measured' }), el('th', { text: 'true' })));
  const row = (name, what, l, t) => tbl.append(el('tr', {}, el('td', { text: `${name} ${what}` }), el('td', { text: l }), el('td', { text: t })));
  if (v) {
    actuators().forEach((c, i) => { const r = v.motors[i]; if (r && r.measured) { any = true; row(c.name, 'lag', `${Math.round(r.tau * 1000)} ms`, `${Math.round(c.tau * 1000)} ms near hover`); row(c.name, 'curve bend', r.curve.toFixed(2), trueBend(c).toFixed(2)); } });
    joints().forEach((j, k) => { const r = v.joints[k]; if (r && r.measured) { any = true; row(j.name, 'speed', `${Math.round(r.rate * R2D)}°/s`, `${Math.round(j.rate)}°/s no-load`); row(j.name, 'lag', `${Math.round(r.lag * 1000)} ms`, `${Math.round((j.lag || 0) * 1000)} ms`); } });
  }
  const sig = any ? tbl.textContent : '-'; if (box._sig === sig) return; box._sig = sig; box.textContent = '';   // (rebuilt only when a number changed)
  if (any) box.append(tbl); else box.append(el('p', { class: 'hint', text: 'Calibrate to measure each motor\'s lag and throttle curve, and each servo\'s real speed and lag. The servos\' are sent to the flight core; the curve is only shown.' }));
}
function renderLearn(force) {
  const v = learn.view, b = boardOf('learn');
  setText($('#learnWhere'), b ? `On ${b.name}, on the flight core's telemetry (200 times a second). It asks the flight core for test moves and tells it which model to fly on.` : '');
  $('#useDesc').setAttribute('aria-pressed', String(!v || !v.useLearned)); $('#useLearned').setAttribute('aria-pressed', String(!!(v && v.useLearned)));
  {   // Learned needs something learned first
    const can = !!(v && (v.haveFit || v.useLearned)), ul = $('#useLearned');
    if (ul.getAttribute('aria-disabled') !== String(!can)) { ul.setAttribute('aria-disabled', String(!can)); ul.title = can ? 'Fly on what the calibration or a throw learned' : 'Nothing learned yet: calibrate (or throw it) first'; $('#learnedWhy').hidden = can; }
  }
  $('#keepLearn').checked = v ? v.keep : learnPrefs.keep;
  const cal = v && v.cal, busy = !!thr || (v && v.thr > 0 && v.thr < 4);
  setText($('#calBtn'), cal ? 'Stop' : v && v.haveFit ? 'Calibrate again' : 'Calibrate');
  $('#calBtn').disabled = !!S.crashed || !actuators().length || busy || !v;
  $('#holdPulses').checked = v ? v.holdPulses : learnPrefs.holdPulses; $('#thenCal').checked = throwCfg.thenCalibrate;
  if (force || performance.now() - (renderLearn.hintT || 0) > 2000) { renderLearn.hintT = performance.now(); setText($('#throwHint'), throwHintText()); }
  const throwing = busy || (v && v.thr === 4);
  $('#calProg').hidden = !cal && !throwing;
  if (throwing && !cal) { $('#calFill').style.width = (100 * (thr && thr.phase !== 'free' ? 0 : v ? v.thrProg : 0)).toFixed(1) + '%'; setText($('#calStage'), throwStageText()); }
  if (cal) {
    const who = v.segKind === 1 ? actuators()[v.segWho] : v.segKind === 2 || v.segKind === 3 ? joints()[v.segWho] : null;
    $('#calFill').style.width = (100 * v.calProg).toFixed(1) + '%';
    setText($('#calStage'), v.held ? 'Paused until the drone settles…' : `${CAL_STAGE[v.segKind] || 'Starting'}${who ? ': ' + who.name : ''} · ${v.left.toFixed(1)} s left`);
  }
  if (learn.msg) setText($('#learnMsg'), learn.msg);
  setText($('#learnSmall'), !v ? '' : v.useLearned ? (v.keep ? 'learned · learning' : 'learned') : (v.keep ? 'description · learning' : 'description'));
  if (force || performance.now() - matchT > 400) { matchT = performance.now(); matchCache = matchScores(); }
  const box = $('#matchRows'), sig = busy ? 'busy' : matchCache.map(m => m.c.id + m.c.name).join('|');
  if (box._sig !== sig) {   // rows built once per set of parts, the bars and numbers updated in place
    box._sig = sig; box.textContent = ''; box._rows = [];
    if (busy) box.append(el('p', { class: 'hint', text: 'Shown once it has caught itself.' }));
    else for (const m of matchCache) { const i = el('i'), mv = el('span', { class: 'mv' }); box._rows.push({ i, mv }); box.append(el('div', { class: 'mrow' }, el('span', { class: 'an', text: m.c.name }), el('div', { class: 'mbar' }, i), mv)); }
  }
  if (busy) return;
  matchCache.forEach((m, k) => {
    const pc = Math.round(m.match * 100), cls = pc >= 85 ? '' : pc >= 65 ? 'warn' : 'bad', r = box._rows[k];
    if (r.i.className !== cls) r.i.className = cls; r.i.style.width = pc + '%'; setText(r.mv, pc + '%');
  });
  renderResponses();
}

/* ───────── traces ───────── */
const cv = $('#chart'), cx = cv.getContext('2d'); let hoverX = null;
cv.addEventListener('pointermove', e => { const r = cv.getBoundingClientRect(); hoverX = e.clientX - r.left; }); cv.addEventListener('pointerleave', () => hoverX = null);
function drawChart() {
  const dpr = Math.min(window.devicePixelRatio || 1, 2); const W = cv.clientWidth, H = cv.clientHeight; if (!W) return;
  if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) { cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
  cx.setTransform(dpr, 0, 0, dpr, 0, 0); cx.clearRect(0, 0, W, H);
  const acc = tok('--accent'), muted = tok('--muted'), ink = tok('--ink'), line = tok('--line'), lineS = tok('--line-strong');
  const t1 = S.t, t0 = t1 - 10; const xOf = t => (t - t0) / 10 * W;
  const bands = [{ k: 'tilt', lab: 'Tilt', u: '°', min: 10, dp: 1, src: 'sim' }, { k: 'err', lab: 'Position error (from the target)', u: 'cm', min: 10, dp: 1, src: 'sim' },
    { k: 'est', lab: 'Attitude estimate error', u: '°', min: 2, dp: 2, src: 'cmp' }, { k: 'util', lab: 'Peak motor command', u: '%', min: 100, dp: 0, fix: true, src: 'board' }];
  const gap = 10, bh = (H - gap * (bands.length - 1)) / bands.length; let hi = -1;
  if (hoverX != null && hist.t.length) { const th = t0 + hoverX / W * 10; let best = 1e9; hist.t.forEach((t, i) => { const d = Math.abs(t - th); if (d < best) { best = d; hi = i; } }); }
  bands.forEach((b, bi) => {
    const y0 = bi * (bh + gap), top = y0 + 14, bot = y0 + bh; const data = hist[b.k];
    const mx = b.fix ? 100 : Math.max(b.min, ...data) * 1.1; const yOf = v => bot - (clamp(v, 0, mx) / mx) * (bot - top);
    cx.strokeStyle = line; cx.lineWidth = 1; cx.beginPath(); cx.moveTo(0, bot + .5); cx.lineTo(W, bot + .5); cx.moveTo(0, Math.round((top + bot) / 2) + .5); cx.lineTo(W, Math.round((top + bot) / 2) + .5); cx.stroke();
    cx.beginPath(); cx.arc(3.5, y0 + 6, 3.5, 0, Math.PI * 2); cx.fillStyle = tok('--src-' + b.src); cx.fill();   // where it comes from (sources.js)
    cx.font = '600 11px "Barlow Condensed", "Arial Narrow", sans-serif'; cx.fillStyle = muted; cx.textBaseline = 'top'; cx.fillText(b.lab.toUpperCase(), 11, y0);
    cx.font = '10px "JetBrains Mono", monospace'; cx.textAlign = 'right'; cx.fillText(`max ${mx.toFixed(0)} ${b.u}`, W, y0); cx.textAlign = 'left';
    if (data.length > 1) {
      cx.beginPath(); data.forEach((v, i) => { const x = xOf(hist.t[i]), y = yOf(v); i ? cx.lineTo(x, y) : cx.moveTo(x, y); });
      cx.lineTo(xOf(hist.t[data.length - 1]), bot); cx.lineTo(xOf(hist.t[0]), bot); cx.closePath(); cx.globalAlpha = 0.12; cx.fillStyle = acc; cx.fill(); cx.globalAlpha = 1;
      cx.beginPath(); data.forEach((v, i) => { const x = xOf(hist.t[i]), y = yOf(v); i ? cx.lineTo(x, y) : cx.moveTo(x, y); }); cx.strokeStyle = acc; cx.lineWidth = 2; cx.lineJoin = 'round'; cx.stroke();
      const li = data.length - 1; cx.beginPath(); cx.arc(xOf(hist.t[li]), yOf(data[li]), 3.5, 0, Math.PI * 2); cx.fillStyle = acc; cx.fill();
      cx.font = '500 11px "JetBrains Mono", monospace'; cx.fillStyle = ink; cx.textAlign = 'right'; if (hi < 0) cx.fillText(`${data[li].toFixed(b.dp)} ${b.u}`, W - 4, top + 2); cx.textAlign = 'left';
    }
    if (hi >= 0) {
      const x = xOf(hist.t[hi]); cx.strokeStyle = lineS; cx.lineWidth = 1; cx.beginPath(); cx.moveTo(x + .5, top); cx.lineTo(x + .5, bot); cx.stroke();
      cx.beginPath(); cx.arc(x, yOf(data[hi]), 4, 0, Math.PI * 2); cx.fillStyle = acc; cx.fill();
      const txt = `${data[hi].toFixed(b.dp)} ${b.u} @ ${(hist.t[hi] - t1).toFixed(1)} s`; cx.font = '500 11px "JetBrains Mono", monospace'; cx.fillStyle = ink;
      const tw = cx.measureText(txt).width; cx.fillText(txt, clamp(x + 6, 0, W - tw), top + 2);
    }
  });
}

/* ───────── target & environment ───────── */
const spRefs = [];
function spSlider(key, label, min, max, step, u, obj, ends) {
  const f = numField('sp-' + key, { label, min, max, step, u, dp: step < 1 ? (step < 0.1 ? 2 : 1) : 0, ends }, () => obj[key], v => {
    obj[key] = v; if (obj === setpoint) { pilot.vref = [0, 0, 0]; ctl.vRef = [0, 0, 0]; }
  });
  spRefs.push(f.refresh);
  return f.node;
}
function syncSp() { for (const r of spRefs) r(); }  // keep the target fields in step with flying
function buildSp() {
  const b = $('#spFields'); b.textContent = '';
  b.append(spSlider('x', 'Target X', -3, 3, 0.1, 'm', setpoint), spSlider('y', 'Target Y', -3, 3, 0.1, 'm', setpoint), spSlider('z', 'Target altitude', 0.3, 5, 0.1, 'm', setpoint),
    spSlider('yaw', 'Target heading', -180, 180, 5, '°', setpoint), spSlider('wind', 'Wind speed', 0, 10, 0.5, 'm/s', envr), spSlider('windDir', 'Wind toward', -180, 180, 5, '°', envr),
    spSlider('turb', 'Turbulence', 0, 1, 0.05, '', envr, ['0 still air', '1 gusty']), spSlider('spread', 'Motor and prop differences', 0, 3, 0.1, '× typical', envr, ['0 identical', '3× typical']),
    spSlider('texture', 'Ground texture', 0, 1, 0.05, '', envr, ['0 water', '1 gravel']), spSlider('light', 'Light', 0, 1, 0.05, '', envr, ['0 dark', '1 daylight']), spSlider('ambient', 'Air temperature', -10, 45, 1, '°C', envr));
}

/* ───────── header ───────── */
// Start from a layout or one of your saved designs: a menu, so nothing loads until you pick one.
const presetMenu = menuButton({ text: 'Layouts', key: 'presetMenu', align: 'left', title: 'Start from a layout (Blank is a bare frame) or one of your saved designs',
  items: () => [...Object.entries(PRESETS).map(([k, p]) => ({ value: 'p:' + k, label: p.label, group: 'Layouts', hint: p.blank ? 'opens the editor' : null })),
    ...(typeof designs !== 'undefined' ? designs.list : []).map(d => ({ value: 'd:' + d.id, label: d.name || 'Untitled design', group: 'My designs', cur: d.id === designs.cur }))],
  onPick: v => {
    if (v.startsWith('d:')) { const d = designs.list.find(x => x.id === v.slice(2)); if (d) askToSave(d.name || 'Untitled design', () => openDesign(d)); }
    else { const k = v.slice(2); if (PRESETS[k]) askToSave(PRESETS[k].label, () => loadPreset(k)); }
  } });
$('#presetSlot').replaceWith(presetMenu.node); presetMenu.node.id = 'presetSlot';
function loadPreset(key) { const p = PRESETS[key].build(); cfg.frame.mass = p.frame; cfg.comps = migrateComps(p.comps); cfg.battery = p.battery || defaultBattery(); setMode(p.mode, false); openSet.clear(); designLoaded(null, ''); afterLoad();
  if (PRESETS[key].blank && typeof setEditMode === 'function') setEditMode(true);   // a bare frame: straight to building
}
function afterLoad() {
  frameMassField.refresh();
  truth = null; recomputeProps(); cPts = contactPoints(); rebuildDrone(); renderComps(); buildActRows(); doReset(); refreshEnvelope(); renderMass(); save();
}
const frameMassField = numField('frameMass', { label: 'Frame hub mass', min: 0.1, max: 2, hmin: 0.02, hmax: 50, step: 0.01, u: 'kg', dp: 2 }, () => cfg.frame.mass,
  v => { cfg.frame.mass = v; undoKey = 'frame'; recomputeProps(); refreshEnvelope(); renderMass(); save(); });
$('#frameMassSlot').replaceWith(frameMassField.node);
const MODE_DESC = { tilt: 'Leans to move; servos help turn.', mixed: 'Servos push part, leaning does the rest.', level: 'Stays level; servos push sideways.' };
function setMode(m, recalc = true) {
  mode = m; steerMix.rho = 1;
  $('#modeTilt').setAttribute('aria-pressed', String(m === 'tilt')); $('#modeMixed').setAttribute('aria-pressed', String(m === 'mixed')); $('#modeLevel').setAttribute('aria-pressed', String(m === 'level'));
  setText($('#modeDesc'), MODE_DESC[m] || '');
  ctl.iAtt = [0, 0, 0]; if (recalc) { refreshEnvelope(); save(); }
}
$('#modeTilt').addEventListener('click', () => setMode('tilt')); $('#modeMixed').addEventListener('click', () => setMode('mixed')); $('#modeLevel').addEventListener('click', () => setMode('level'));
function renderRun() {   // one button: shows pause while running, play while paused
  const b = $('#runBtn'); b.classList.toggle('paused', !running);
  b.setAttribute('aria-label', running ? 'Pause' : 'Run'); b.title = running ? 'Pause (K)' : 'Run (K)';
}
$('#runBtn').addEventListener('click', () => {
  if (editMode) { editWasRunning = true; setEditMode(false); return; }   // Run leaves edit mode
  running = !running; renderRun();
});
function doReset() { pilot.vref = [0, 0, 0]; resetSim(); $('#crash').hidden = true; }
$('#resetBtn').addEventListener('click', doReset); $('#crashReset').addEventListener('click', doReset);
/* Poke: hold to charge, release to hit. Strength grows with hold time up to POKE_FULL seconds. */
const POKE_FULL = 1.5;
const poke = { t0: 0, src: null, raf: 0 };
const pokeBtn = $('#pokeBtn'), pokeLbl = pokeBtn.querySelector('.poke-lbl');
const pokeCharge = () => clamp((performance.now() - poke.t0) / 1000 / POKE_FULL, 0, 1);
function pokeStart(src) {
  if (poke.src || S.crashed) return;
  poke.src = src; poke.t0 = performance.now();
  pokeBtn.classList.add('charging'); pokeBtn.classList.remove('fired', 'full');
  const tick = () => {
    const c = pokeCharge();
    pokeBtn.style.setProperty('--charge', (c * 100).toFixed(1) + '%');
    pokeLbl.textContent = `Poke ${Math.round(c * 100)}%`;
    pokeBtn.classList.toggle('full', c >= 1);
    poke.raf = requestAnimationFrame(tick);
  };
  tick();
}
function pokeEnd(src, fire) {
  if (poke.src !== src) return;
  const c = pokeCharge(); cancelAnimationFrame(poke.raf); poke.src = null;
  pokeBtn.classList.remove('charging', 'full'); pokeBtn.style.setProperty('--charge', '0%');
  pokeLbl.textContent = 'Poke';
  if (!fire || S.crashed) return;
  const spin = 1.5 + 10.5 * c, push = 0.3 + 2.7 * c;          // rad/s and m/s added to the current motion
  const a = Math.random() * Math.PI * 2, b = Math.random() * Math.PI * 2;
  S.w = add(S.w, [Math.cos(a) * spin, Math.sin(a) * spin, (Math.random() - 0.5) * spin * 0.5]);
  S.v = add(S.v, [Math.cos(b) * push, Math.sin(b) * push, 0]);
  pokeBtn.title = `Last poke: ${Math.round(c * 100)}% · ${spin.toFixed(1)} rad/s spin, ${push.toFixed(1)} m/s shove. Hold to charge (P).`;
  void pokeBtn.offsetWidth; pokeBtn.classList.add('fired');
}
pokeBtn.addEventListener('pointerdown', e => { if (e.button !== 0) return; e.preventDefault(); try { pokeBtn.setPointerCapture(e.pointerId); } catch (x) {} pokeStart('ptr:' + e.pointerId); });
pokeBtn.addEventListener('pointerup', e => pokeEnd('ptr:' + e.pointerId, true));
pokeBtn.addEventListener('pointercancel', e => pokeEnd('ptr:' + e.pointerId, false));
pokeBtn.addEventListener('lostpointercapture', e => pokeEnd('ptr:' + e.pointerId, true));
pokeBtn.addEventListener('contextmenu', e => e.preventDefault());
pokeBtn.addEventListener('keydown', e => {   // Enter/Space on the focused button charges too
  if (e.code !== 'Space' && e.code !== 'Enter') return;
  e.preventDefault(); e.stopPropagation(); if (!e.repeat) pokeStart('key:btn');
});
pokeBtn.addEventListener('keyup', e => { if (e.code === 'Space' || e.code === 'Enter') { e.preventDefault(); e.stopPropagation(); pokeEnd('key:btn', true); } });
pokeBtn.addEventListener('blur', () => pokeEnd('key:btn', true));
window.addEventListener('keydown', e => {
  if (e.code !== 'KeyP' || e.metaKey || e.ctrlKey || e.altKey || typingIn(e.target)) return;
  e.preventDefault(); if (!e.repeat) pokeStart('key:P');
});
window.addEventListener('keyup', e => { if (e.code === 'KeyP') pokeEnd('key:P', true); });
window.addEventListener('blur', () => { if (poke.src) pokeEnd(poke.src, false); });
// The world: open ground or a city (terrain.js). Changing it starts the flight again in the open plaza.
function applyTerrain(kind, seed) {
  setTerrain(kind, seed); syncTerrainUi(); cPts = contactPoints(); doReset(); save();
}
function syncTerrainUi() { $('#terrainSel').value = terrain.kind; $('#terrainNew').disabled = terrain.kind === 'open'; }
commitSelect($('#terrainSel'), v => { if (v !== terrain.kind) applyTerrain(v, terrain.seed); }, 'Press Enter to switch: it starts the flight again');
$('#terrainNew').addEventListener('click', () => applyTerrain(terrain.kind, 1 + Math.floor(Math.random() * 1e9)));
function setSpeed(v) { speed = v; document.querySelectorAll('#speedSeg [data-speed]').forEach(b => b.setAttribute('aria-pressed', String(+b.dataset.speed === v))); }
document.querySelectorAll('#speedSeg [data-speed]').forEach(b => b.addEventListener('click', () => setSpeed(+b.dataset.speed)));
[['tFollow', 'follow'], ['tChase', 'chase']].forEach(([id, k]) => { const b = $('#' + id); b.addEventListener('click', () => { view[k] = !view[k]; b.setAttribute('aria-pressed', String(view[k])); }); });
onCrash = () => { $('#crashWhy').textContent = S.crashed; $('#crash').hidden = false; };

/* ───────── Show menu: what the view draws ───────── */
// One switch per overlay (LAYERS in view3d.js), grouped, with presets. The choice is kept in this browser.
const SHOW_LS = 'drone-force-bench-show';
const SHOW_PRESETS = {
  Defaults: () => Object.fromEntries(LAYERS.map(L => [L.key, L.on])),
  All: () => Object.fromEntries(LAYERS.map(L => [L.key, true])),
  'Drone only': () => Object.fromEntries(LAYERS.map(L => [L.key, L.key === 'grid'])),
};
function showSave() { try { localStorage.setItem(SHOW_LS, JSON.stringify(Object.fromEntries(LAYERS.map(L => [L.key, view[L.key]])))); } catch (e) {} }
function showApply() {   // buttons, legend keys and readouts follow the layers
  for (const b of document.querySelectorAll('#showMenu [data-key]')) b.setAttribute('aria-pressed', String(!!view[b.dataset.key]));
  const torque = view.rtorque || view.ntorque || view.want;
  for (const el of document.querySelectorAll('.hud-tl [data-layer]')) {
    const k = el.dataset.layer; el.hidden = !(k === 'torque' ? torque : view[k]);
  }
  $('#legend').hidden = !view.legend;
  const n = LAYERS.filter(L => view[L.key] !== L.on).length;
  $('#tShow').firstChild.textContent = n ? `Show (${n} changed) ` : 'Show ';
}
function setLayers(next) { for (const L of LAYERS) if (L.key in next) view[L.key] = !!next[L.key]; showApply(); showSave(); }
function toggleTorque() {   // Q: rotor and net torque together
  const on = !(view.rtorque || view.ntorque); setLayers({ rtorque: on, ntorque: on });
}
(function buildShowMenu() {
  try { const s = JSON.parse(localStorage.getItem(SHOW_LS) || 'null'); if (s) for (const L of LAYERS) if (typeof s[L.key] === 'boolean') view[L.key] = s[L.key]; } catch (e) {}
  const menu = $('#showMenu'), btn = $('#tShow');
  const groups = [...new Set(LAYERS.map(L => L.group))];
  menu.innerHTML = groups.map(g => `<div class="show-grp"><span class="lbl">${g}</span><div class="show-btns">${
    LAYERS.filter(L => L.group === g).map(L => `<button type="button" class="btn tog" data-key="${L.key}" aria-pressed="false" title="${L.tip.replace(/"/g, '&quot;')}">${L.label}</button>`).join('')}</div></div>`).join('')
    + `<div class="show-grp show-presets"><span class="lbl">Presets</span><div class="show-btns">${Object.keys(SHOW_PRESETS).map(p => `<button type="button" class="btn" data-preset="${p}">${p}</button>`).join('')}</div></div>`;
  menu.addEventListener('click', e => {
    const b = e.target.closest('button'); if (!b) return;
    if (b.dataset.key) setLayers({ [b.dataset.key]: !view[b.dataset.key] });
    else if (b.dataset.preset) setLayers(SHOW_PRESETS[b.dataset.preset]());
  });
  popover(btn, menu);
  showApply();
})();
// A button that shows a panel under it (the Show menu, the keys): Escape or a click elsewhere closes it.
function popover(btn, pop) {
  const open = on => { pop.hidden = !on; btn.setAttribute('aria-expanded', String(on)); };
  btn.addEventListener('click', e => { e.stopPropagation(); open(pop.hidden); });
  document.addEventListener('pointerdown', e => { if (!pop.hidden && !btn.parentElement.contains(e.target)) open(false); });
  document.addEventListener('keydown', e => { if (e.key === 'Escape' && !pop.hidden) { e.preventDefault(); open(false); btn.focus(); } });
}
popover($('#tKeys'), $('#keysPop'));

/* ───────── tabs ───────── */
const TABS = [['tabAir', 'air'], ['tabForm', 'form'], ['tabGs', 'gs']];
function showTab(which) {
  if (!['air', 'form', 'gs'].includes(which)) which = 'air';
  const form = which === 'form', gsT = which === 'gs';
  for (const [id, k] of TABS) { const t = $('#' + id); t.setAttribute('aria-selected', String(k === which)); t.tabIndex = k === which ? 0 : -1; }
  $('#paneAir').hidden = which !== 'air'; $('#paneForm').hidden = !form; $('#paneGs').hidden = !gsT;
  $('.work').classList.toggle('wide', form || gsT);
  if (gsT) renderGs(true);
  try { localStorage.setItem(LS + '-tab', which); } catch (e) {}
}
for (const [id, k] of TABS) $('#' + id).addEventListener('click', () => showTab(k));
$('.tabs').addEventListener('keydown', e => {   // arrow keys move between the tabs (one tab stop for the row)
  const i = TABS.findIndex(([id]) => id === e.target.id); if (i < 0) return;
  const j = e.key === 'ArrowRight' ? (i + 1) % TABS.length : e.key === 'ArrowLeft' ? (i - 1 + TABS.length) % TABS.length : e.key === 'Home' ? 0 : e.key === 'End' ? TABS.length - 1 : -1;
  if (j < 0) return; e.preventDefault(); showTab(TABS[j][1]); $('#' + TABS[j][0]).focus();
});

/* ───────── persistence (this browser only) ───────── */
const LS = 'drone-force-bench-v1';
function save() {
  if (typeof markDesign === 'function') markDesign();   // undo history and "unsaved changes" (designs.js)
  try {
    const laws = {}; for (const L of editedLaws()) laws[L.def.key] = L.src;
    localStorage.setItem(LS, JSON.stringify({ cfg, mode, laws, sensing, keepLearning: learnPrefs.keep, holdPulses: learnPrefs.holdPulses, allocPrefs: { allowance: allocPrefs.allowance, efficiency: allocPrefs.efficiency, servoMove: allocPrefs.servoMove }, mixShare: steerMix.share, designCur: typeof designs !== 'undefined' ? designs.cur : null, designName: typeof designs !== 'undefined' ? designs.name : '', designClean: typeof designs !== 'undefined' && !!designs.cur && designs.savedSnap === designSnap(), designEdited: typeof designs !== 'undefined' && designChanged(), terrain: { kind: terrain.kind, seed: terrain.seed }, launch: launchMode, throwCfg: { v: 2, height: throwCfg.height, spin: throwCfg.spin, thenCalibrate: throwCfg.thenCalibrate }, radio: { ...radioCfg }, tlmV: 1 }));
  } catch (e) {}
}
// Brings a design saved by an older version up to date.
function migrateComps(comps) {
  if (!comps.some(c => c.type === 'sensor')) comps.push(...defaultSensors());   // saved before sensors existed
  comps = migrateTiltParts(comps);   // saved before servo joints existed
  for (const c of comps) if (c.type === 'motor' && !c.prop) withProp(c);
  for (const c of comps) {   // saved before the hidden hardware traits existed
    if (c.type === 'motor') delete c.curve;   // the throttle curve now comes from the motor physics
    if (c.type === 'motor' && !c.pitch) c.pitch = 'fixed';
    if (c.type === 'motor') { c.push = !!c.push; for (const [k, v] of Object.entries({ tsens: false, telem: true, tmaxC: 120, cool: 1, failHeat: true, failMode: 'stop', failLoss: 50 })) if (c[k] == null) c[k] = v; }
    if (c.type === 'joint' && !c.failMode) c.failMode = 'jam';
    if (c.type === 'link' && c.roll == null) c.roll = 0;
    if (c.type === 'joint' && c.torque == null) c.torque = 0.8;
    if (c.type === 'sensor' && c.kind === 'imu') { if (c.scaleErr == null) c.scaleErr = 0.005; if (c.misalign == null) c.misalign = 0.2; }
    if (c.type === 'sensor' && c.kind === 'mag' && c.softIron == null) c.softIron = 0.03;
    if (c.type === 'sensor' && !(c.mass > 0)) c.mass = SENSOR_MASS[c.kind] || 0.003;   // saved when sensors weighed nothing
    if (c.type === 'joint' && !(c.mass > 0)) c.mass = 0.015;
    if (c.type === 'motor' && c.fm == null) c.fm = 0.6;
    if (c.type === 'joint' && c.hingeEl == null) c.hingeEl = 0;
  }
  return comps;
}
let bootDesign = null;   // which saved design the page was showing when it was last closed
function load() {
  let s = null; try { s = JSON.parse(localStorage.getItem(LS) || 'null'); } catch (e) {}
  if (!s) return false;
  if (s.laws) for (const [key, src] of Object.entries(s.laws)) {
    if (!LAWS[key]) continue;
    try { applyLaw(key, src); } catch (e) { const L = LAWS[key]; L.src = src; L.status = 'error'; L.err = e.message; }
  }
  if (s.terrain && TERRAINS[s.terrain.kind]) setTerrain(s.terrain.kind, s.terrain.seed);
  if (s.radio) for (const k of ['rate', 'ratio', 'power', 'extra']) if (isFinite(s.radio[k])) radioCfg[k] = +s.radio[k];
  if (s.cfg && Array.isArray(s.cfg.comps) && s.cfg.comps.length) {
    cfg.frame.mass = s.cfg.frame.mass; cfg.comps = s.cfg.comps; if (s.cfg.computers) cfg.computers = fixComputers(s.tlmV ? s.cfg.computers : computersWithRadio(s.cfg.computers)); uid = Math.max(0, ...cfg.comps.map(c => c.id)) + 1; mode = ['level', 'mixed'].includes(s.mode) ? s.mode : 'tilt';
    sensing = s.sensing === 'truth' ? 'truth' : 'sensors';
    if (s.keepLearning === false) learnPrefs.keep = false;
    if (s.holdPulses === false) learnPrefs.holdPulses = false;
    if (isFinite(s.mixShare)) steerMix.share = +s.mixShare;
    if (s.allocPrefs) for (const k of ['allowance', 'efficiency', 'servoMove']) if (isFinite(s.allocPrefs[k])) allocPrefs[k] = +s.allocPrefs[k];
    if (s.launch === 'throw') launchMode = 'throw';
    if (s.throwCfg) for (const k of ['height', 'spin']) if (isFinite(s.throwCfg[k]) && !(k === 'height' && !s.throwCfg.v && +s.throwCfg[k] === 4)) throwCfg[k] = +s.throwCfg[k];   // (4 m was the old default: now higher)
    if (s.throwCfg && s.throwCfg.thenCalibrate === false) throwCfg.thenCalibrate = false;
    cfg.comps = migrateComps(cfg.comps);
    cfg.battery = { ...defaultBattery(), ...(s.cfg.battery || {}) };
    bootDesign = { cur: s.designCur || null, name: s.designName || '', clean: !!s.designClean, edited: s.designEdited !== false && !(s.designCur && s.designClean) };
    return true;
  }
  return false;
}

/* ───────── theme ───────── */
const onTheme = () => { applyTheme(); renderEnvelope(); };
const mq = window.matchMedia('(prefers-color-scheme: dark)'); mq.addEventListener && mq.addEventListener('change', onTheme);
new MutationObserver(onTheme).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });

/* ───────── boot ───────── */
function boot() {
  buildSp(); buildThrowFields(); buildAllocFields(); buildComputers(); bindPads();
  const loaded = load(); if (!terrain.ver) setTerrain('parkour', 1); syncTerrainUi();
  if (loaded) setMode(mode, false); else { const p = PRESETS.quadx.build(); cfg.frame.mass = p.frame; cfg.comps = p.comps; setMode(p.mode, false); }
  setSensing(sensing); setLaunch(launchMode, false); for (const r of throwFieldRefs) r(); for (const r of allocFieldRefs) r(); buildMaterials(); applyTheme(); afterLoad(); refreshFormulaStatus();
  initDesigns(bootDesign); syncFlightUi();
  let tab = 'air'; try { tab = localStorage.getItem(LS + '-tab') || 'air'; } catch (e) {}
  showTab(tab);
  let lastT = performance.now(), envT = 0, uiT = 0;
  function frame(now) {
    const dt = Math.min(0.05, (now - lastT) / 1000); lastT = now;
    if (running) { const steps = Math.min(200, Math.round(dt * speed / PDT)); pilotStep(steps * PDT); for (let n = 0; n < steps; n++) physStep(); }
    envT += dt; if (envT > 1) { envT = 0; refreshEnvelope(); if (!$('#paneForm').hidden) renderComputers(); }
    renderGs();
    uiT += dt; if (uiT > 0.1) { uiT = 0; updateLive(); drawChart(); if (typeof renderHealth === 'function') renderHealth(); }
    updateScene(); renderer.render(scene, camera); requestAnimationFrame(frame);
  }
  requestAnimationFrame(frame);
}
