'use strict';
// Flight budget: what the flight software alone costs on the drone's own computer. The simulator's physics
// isn't counted; only what runs inside the control step (estimation, learning, control, allocation and the
// code around them) plus the throw's background refinement.
//
// How it's counted: the math helpers (math.js) add up the arithmetic they do while a control step runs;
// formulas with loops of their own are counted from their sizes (FLIGHT_COST below). A multiply-add counts
// 2 operations, a square root or trig call about 15. Memory is measured by counting the numbers the flight
// software keeps between steps, at 4 bytes each (32-bit floats). Time assumes plain C in 32-bit floats.

const CHIPS = {
  esp32: { label: 'ESP32', mops: 60, cores: 2, ramKB: 300, note: '2 cores at 240 MHz with a float unit' },
  s3: { label: 'ESP32-S3', mops: 80, cores: 2, ramKB: 300, note: '2 cores at 240 MHz with a float unit and faster memory' },
  c3: { label: 'ESP32-C3', mops: 4, cores: 1, ramKB: 280, note: '1 core at 160 MHz with no float unit, so floats are done in software' },
};
const CTRL_HZ = 1000;

// Operations per call for formulas with their own loops, from their argument sizes.
const FLIGHT_COST = {
  attitudeEstimator: () => 420,
  flowVelocity: () => 50,
  servoPredictor: () => 12,
  positionEstimator: (st, R, accel, baro, fix, flow) => 180 + (fix ? 60 : 0) + (flow ? 40 : 0) + (baro ? 15 : 0) + (!fix && !(flow && flow.v) ? 60 : 0),
  positionControl: () => 45,
  thrustAxisTarget: () => 60,
  attitudeError: () => 110,
  attitudeControl: () => 70,
  forceDemand: () => 12,
  thrustLinearization: () => 25,
  jointRotation: () => 60,
  allocationPreferences: inputs => 30 * inputs.length,
  identifyEffectiveness: (st, u, f, w, r, dt, init, memory, lags, mot) => {
    const n = u.length, m = mot ? 2 * n : n; return 5 * m * m + 30 * m + (mot ? 40 * n : 20 * n) + 150;
  },
  identifyThrow: (st, u, f, w, vb, dt, solve) => {
    const n = u.length, nr = 2 * n + 4, nf = 3 * n + 7, T = 7;
    if (solve === true) return T * (2 * nf ** 3 / 3 + 5 * nf * nf + 3 * (2 * nr ** 3 / 3 + 3 * nr * nr) + 6 * nr * nr);
    if (solve === 'refine' || !(dt > 0)) return 0;                              // background: counted from what it reports
    return T * (n * 45 + 2 * nr * nr + 6 * nr + 6 * (n + 5) ** 2) + 80;
  },
  identifyMotorResponse: wins => { const N = wins.reduce((s, w) => s + w.u.length, 0), np = 4 + wins.length; return 10 * (N * (2 * np * np + 25) + 2 * np ** 3 / 3 + 3 * np * np); },
  identifyServoResponse: wins => { const N = wins.reduce((s, w) => s + w.y.length, 0), np = 1 + wins.length; return 90 * (N * (2 * np * np + 40) + 2 * np ** 3 / 3); },
};
// Occasional fits that run once (at the end of a test, or at the moment a throw is solved), not every step.
const isOneOff = (key, args) => key === 'identifyMotorResponse' || key === 'identifyServoResponse' || (key === 'identifyThrow' && args[6] === true);

const budget = {
  chip: (() => { try { return localStorage.getItem('dfb-chip') || 'esp32'; } catch (e) { return 'esp32'; } })(),
  keys: {}, sum: {}, steps: 0, total: 0, peakWin: 0, peakKeys: null, peaks: [], flightMax: 0, flightMaxKeys: null,
  oneOffs: [], memPeak: 0, lastRender: 0,
};
const chipNow = () => CHIPS[budget.chip] || CHIPS.esp32;
// What the spare core can do in dt (on a single-core chip, a quarter of the only core).
const budgetBackgroundOps = dt => chipNow().mops * 1e6 * dt * (chipNow().cores > 1 ? 0.9 : 0.25);

function budgetBegin() { OPS.n = 0; OPS.on = true; budget.keys = {}; }
function budgetEnd(dt) {
  OPS.on = false;
  if (!(dt > 0)) return;
  let steady = OPS.n; const once = budget.keys.__once || 0; steady -= once; delete budget.keys.__once;
  const lawOps = Object.values(budget.keys).reduce((s, v) => s + v, 0);
  budget.keys.__glue = Math.max(0, steady - lawOps);
  budget.steps++; budget.total += steady;
  for (const [k, v] of Object.entries(budget.keys)) budget.sum[k] = (budget.sum[k] || 0) + v;
  if (steady > budget.peakWin) { budget.peakWin = steady; budget.peakKeys = { ...budget.keys }; }
  if (steady > budget.flightMax) { budget.flightMax = steady; budget.flightMaxKeys = { ...budget.keys }; }
}
// Called from run() for every formula call while a control step is being counted.
function budgetCall(key, args, fn) {
  const model = FLIGHT_COST[key], n0 = OPS.n;
  let out;
  if (model) { OPS.on = false; try { out = fn(); } finally { OPS.on = true; } OPS.n += model(...args); }
  else out = fn();
  const spent = OPS.n - n0;
  if (isOneOff(key, args)) {
    budget.keys.__once = (budget.keys.__once || 0) + spent;
    budget.oneOffs.unshift({ key, ops: spent, t: S.t }); budget.oneOffs.length = Math.min(budget.oneOffs.length, 6);
  } else budget.keys[key] = (budget.keys[key] || 0) + spent;
  return out;
}
function budgetReset() {
  Object.assign(budget, { sum: {}, steps: 0, total: 0, peakWin: 0, peakKeys: null, peaks: [], flightMax: 0, flightMaxKeys: null, oneOffs: [], memPeak: 0 });
}

// Numbers the flight software keeps between steps.
function countFloats(root) {
  const seen = new Set(), comps = new Set(cfg.comps);
  let n = 0;
  const walk = (x, depth) => {
    if (x == null || depth > 12) return;
    if (typeof x === 'number') { n++; return; }
    if (typeof x !== 'object' || seen.has(x) || comps.has(x)) return;
    seen.add(x);
    if (ArrayBuffer.isView(x)) { n += x.length; return; }
    if (x instanceof Map || x instanceof Set) { for (const v of x.values()) walk(v, depth + 1); return; }
    for (const v of Object.values(x)) walk(v, depth + 1);
  };
  walk(root, 0);
  return n;
}
function flightMemory() {
  const parts = [
    ['Attitude estimator', est.att],
    ['Position estimator (0.8 s of history for late readings)', est.pos],
    ['Learning in flight (RLS)', learn.st],
    ['Learned models', [learn.B, learn.prior, learn.flyB, learn.resp]],
    ['Calibration records', learn.cal],
    ['Throw identification (running fits and log)', [thr && thr.st, learn.refine && learn.refine.st]],
    ['Servo predictors', joints().map(j => (jst.get(j.id) || {}).pst)],
  ];
  const seenAll = new Set();
  const rows = parts.map(([name, obj]) => {
    const list = (Array.isArray(obj) ? obj : [obj]).filter(o => o && !seenAll.has(o)); list.forEach(o => seenAll.add(o));
    return { name, kb: countFloats(list) * 4 / 1024 };
  });
  const total = rows.reduce((s, r) => s + r.kb, 0);
  budget.memPeak = Math.max(budget.memPeak, total);
  return { rows, total };
}

const lawName = k => k === '__glue' ? 'Code around the formulas (models, columns, allocation set-up)' : (LAWS[k] ? LAWS[k].def.title : k);
function renderBudget(force) {
  const box = document.getElementById('budgetBody'); if (!box) return;
  const now = performance.now(); if (!force && now - budget.lastRender < 500) return; budget.lastRender = now;
  const chip = chipNow(), perStep = chip.mops * 1e6 / CTRL_HZ;   // operations one core can do per control period
  const avg = budget.steps ? budget.total / budget.steps : 0;
  budget.peaks.push(budget.peakWin); if (budget.peaks.length > 10) budget.peaks.shift();   // ~5 s of peaks
  const peak = Math.max(0, ...budget.peaks);
  const pct = x => `${Math.round(100 * x / perStep)}%`, kops = x => x >= 1e6 ? `${(x / 1e6).toFixed(1)} M` : x >= 1e3 ? `${(x / 1e3).toFixed(1)} k` : `${Math.round(x)}`;
  const ms = x => { const t = x / (chip.mops * 1e6) * 1000; return t >= 1000 ? `${(t / 1000).toFixed(1)} s` : t >= 10 ? `${Math.round(t)} ms` : `${t.toFixed(1)} ms`; };
  const top = Object.entries(budget.sum).map(([k, v]) => [k, v / Math.max(1, budget.steps)]).sort((a, b) => b[1] - a[1]).slice(0, 5);
  const mem = flightMemory();
  box.textContent = '';
  const kv = el('dl', { class: 'kv' });
  const row = (k, v, cls) => kv.append(el('dt', { text: k }), el('dd', { text: v, class: cls || '' }));
  const lvl = x => x > 0.9 ? 'bad' : x > 0.6 ? 'warn' : '';
  row(`Each ${1000 / CTRL_HZ} ms step`, avg ? `${kops(avg)} ops · ${pct(avg)} of a core` : '—', lvl(avg / perStep));
  row('Busiest, last 5 s', peak ? `${kops(peak)} ops · ${pct(peak)}` : '—', lvl(peak / perStep));
  row('Busiest this flight', budget.flightMax ? `${kops(budget.flightMax)} ops · ${pct(budget.flightMax)}` : '—', lvl(budget.flightMax / perStep));
  if (peak) row('Fastest loop it could keep', `${Math.min(99999, Math.floor(chip.mops * 1e6 / Math.max(peak, avg)))} Hz`, lvl(Math.max(peak, avg) / perStep));
  row('Memory kept', `${mem.total.toFixed(1)} KB · most ${budget.memPeak.toFixed(1)} of ~${chip.ramKB}`, lvl(budget.memPeak / chip.ramKB));
  box.append(kv);
  if (top.length) {
    const t = el('table', { class: 'resp budget' });
    t.append(el('tr', {}, el('th', { text: 'Each step, on average' }), el('th', { text: 'ops' }), el('th', { text: 'of core' })));
    for (const [k, v] of top) t.append(el('tr', {}, el('td', { text: lawName(k) }), el('td', { text: kops(v) }), el('td', { text: pct(v) })));
    box.append(t);
  }
  const once = budget.oneOffs.slice(0, 4);
  if (once.length) {
    const t = el('table', { class: 'resp budget' });
    t.append(el('tr', {}, el('th', { text: 'One-off fits' }), el('th', { text: 'ops' }), el('th', { text: 'time' })));
    for (const o of once) t.append(el('tr', {}, el('td', { text: `${lawName(o.key)} (t ${o.t.toFixed(1)} s)` }), el('td', { text: kops(o.ops) }), el('td', { text: ms(o.ops) })));
    box.append(t, el('p', { class: 'hint', text: 'Fits that run once. On the drone they belong on the second core, except the throw\'s first fit, which pauses control for that long while the motors are still off.' }));
  }
  const rf = learn.refine;
  if (rf) box.append(el('p', { class: 'hint', text: `Spare core: working out each motor's lag from the throw, ${Math.round((rf.progress || 0) * 100)}% done (${kops(rf.spent || 0)} ops so far).` }));
  const memT = el('table', { class: 'resp budget' });
  memT.append(el('tr', {}, el('th', { text: 'Memory' }), el('th', { text: 'KB' })));
  for (const r of mem.rows.filter(r => r.kb > 0.05).sort((a, b) => b.kb - a.kb)) memT.append(el('tr', {}, el('td', { text: r.name }), el('td', { text: r.kb.toFixed(1) })));
  box.append(memT);
  box.append(el('p', { class: 'hint', text: `Counted from the arithmetic the flight formulas do, not the simulator's. Time assumes plain C in 32-bit floats at about ${chip.mops} million operations per second on the ${chip.label} (${chip.note}). Careful C can beat that; a first port may not reach it. Memory counts the numbers the flight code keeps, at 4 bytes each.` }));
  budget.total = 0; budget.steps = 0; budget.sum = {}; budget.peakWin = 0;
}
function setChip(k) { budget.chip = k; try { localStorage.setItem('dfb-chip', k); } catch (e) {} renderBudget(true); }
