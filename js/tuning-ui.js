'use strict';
// Airframe tab → Tuning: the controller's gains (tuning.js), as how fast each loop responds, how damped it is and
// how quickly it trims out a steady push, with a prediction of a small step. A change flies at once: the formulas
// that read TUNE are compiled again and go through the boards' loading steps (check, fly in the background, swap).

const TUNE_LOOPS = [
  { id: 'rp', loop: 'att', axes: [0, 1], fold: 'Roll & pitch', law: 'attitudeControl', hz: [0.3, 4, 0.05], integ: [0, 3],
    what: 'Leaning: how quickly the drone tilts to the angle it wants. Everything else rides on this loop.' },
  { id: 'yaw', loop: 'att', axes: [2], fold: 'Yaw', law: 'attitudeControl', hz: [0.2, 3, 0.05], integ: [0, 3],
    what: 'Turning to the heading it wants. Yaw has the least authority (the props\' drag), so it is tuned softer.' },
  { id: 'pos', loop: 'pos', axes: null, fold: 'Position hold', law: 'positionControl', hz: [0.05, 1, 0.01], integ: [0, 1],
    what: 'Holding and moving to a target (the Navigation task). It leans to move, so it must be several times slower than Roll & pitch.' },
];
const tuneUi = { loops: new Map(), raw: [], refs: [], open: new Set(['rp']), defCache: new Map() };   // open: which loops are unfolded (on any drone)

function setTuning(t, key) {
  if (typeof atBusy === 'function' && (atBusy() || brt.autotune?.phase === 'review')) atStop('Manual tuning changed; test stopped.');
  if (tuneSame(t, tuneOf())) return;
  cfg.tuning = tuneFix(t); undoKey = 'tune:' + key;   // (a slider drag is one undo step)
  save(); rnRestage(250); refreshTuning();
}
function buildTuning() {
  const box = $('#tuneLoops'); box.textContent = ''; tuneUi.loops.clear(); tuneUi.refs.length = 0;
  for (const L of TUNE_LOOPS) {
    const feel = () => tuneFeel(tuneOf(), L.loop, L.axes ? L.axes[0] : 0);
    const put = (k, v) => setTuning(tuneWithFeel(tuneOf(), L.loop, L.axes, { ...feel(), [k]: v }), L.id + ':' + k);
    const f = [
      numField(`tune-${L.id}-hz`, { label: 'Response', min: L.hz[0], max: L.hz[1], step: L.hz[2], u: 'Hz', dp: 2, ends: ['gentle', 'snappy'] }, () => feel().hz, v => put('hz', v)),
      numField(`tune-${L.id}-zeta`, { label: 'Damping', min: 0.2, max: 1.5, step: 0.05, u: '', dp: 2, ends: ['bouncy', 'sluggish'] }, () => feel().zeta, v => put('zeta', v)),
      numField(`tune-${L.id}-integ`, { label: 'Integral (trims a steady push)', min: L.integ[0], max: L.integ[1], step: 0.05, u: '1/s', dp: 2, ends: ['0 off', 'fast'] }, () => feel().integ, v => put('integ', v)),
    ];
    const chart = tuneChart(L), verdict = el('p', { class: 'ui-status tune-verdict', role: 'status' }), note = el('p', { class: 'hint warn', hidden: true });
    const sum = el('small'), body = el('div', { class: 'tune-loop' }, el('p', { class: 'hint', text: L.what }), note, ...f.map(x => x.node), chart.node, verdict);
    const fold = UI.details({ class: 'fold', id: `tune-${L.id}-fold`, title: [el('span', { text: L.fold }), sum], open: tuneUi.open.has(L.id),
      onToggle: open => { open ? tuneUi.open.add(L.id) : tuneUi.open.delete(L.id); if (open) refreshTuning(); } }, body);
    box.append(fold);
    for (const x of f) tuneUi.refs.push(x.refresh);
    tuneUi.loops.set(L.id, { L, f, chart, verdict, note, sum, fold });
  }
  buildTuneRaw();
  refreshTuning();
}
// The numbers the formulas get, per axis.
function buildTuneRaw() {
  const box = $('#tuneRaw'); box.textContent = ''; tuneUi.raw.length = 0;
  const cell = (label, get, set, k) => {
    const inp = UI.input({ type: 'number', class: 'num', step: 'any', 'aria-label': label, inputmode: 'decimal' });
    inp.addEventListener('input', () => { const v = parseFloat(inp.value); if (inp.value !== '' && Number.isFinite(v)) { const n = tuneFix(tuneOf()); set(n, v); setTuning(n, 'raw:' + label); } });
    inp.addEventListener('change', () => show());
    const show = () => { if (document.activeElement !== inp) inp.value = String(+get(tuneOf()).toPrecision(4)); };
    tuneUi.raw.push(show); inp.title = `${k}: ${TUNE_BOUNDS[k][0]} to ${TUNE_BOUNDS[k][1]}`;
    return inp;
  };
  const rows = [['kR', 'attitude error', '1/s²'], ['kW', 'body rate', '1/s'], ['kI', 'error integral', '1/s³']];
  const att = el('div', { class: 'tune-raw' }, el('span'), ...TUNE_AXES.map(a => el('span', { class: 'tune-raw-h', text: a[0].toUpperCase() + a.slice(1) })));
  for (const [k, what, u] of rows) {
    att.append(el('span', { class: 'tune-raw-k', title: what }, el('b', { text: k }), ` ${u}`));
    TUNE_AXES.forEach((a, i) => att.append(cell(`${k} ${a}`, t => t.att[k][i], (n, v) => { n.att[k][i] = v; }, k)));
  }
  const pos = el('div', { class: 'tune-raw' }, el('span'), ...['kp 1/s²', 'kd 1/s', 'ki 1/s³'].map(t => el('span', { class: 'tune-raw-h', text: t })),
    el('span', { class: 'tune-raw-k', text: 'Position' }), ...['kp', 'kd', 'ki'].map(k => cell(`${k} position`, t => t.pos[k], (n, v) => { n.pos[k] = v; }, k)));
  box.append(el('p', { class: 'hint', text: 'Attitude gains are angular accelerations (attitudeControl multiplies them by the inertia), position gains accelerations (positionControl multiplies by the mass), so they carry over between airframes. The formulas read them as TUNE.att and TUNE.pos.' }), att, pos);
}

// A small step, predicted: this tuning, and the default for comparison when they differ. Hovering reads it off.
function tuneChart(L) {
  const W = 280, H = 104, pad = { l: 26, r: 8, t: 8, b: 18 }, NS = 'http://www.w3.org/2000/svg';
  const mk = (tag, a = {}) => { const n = document.createElementNS(NS, tag); for (const [k, v] of Object.entries(a)) n.setAttribute(k, v); return n; };
  const svg = mk('svg', { viewBox: `0 0 ${W} ${H}`, class: 'tune-chart', role: 'img' });
  const grid = mk('g', { class: 'tune-grid' }), def = mk('path', { class: 'tune-def' }), cur = mk('path', { class: 'tune-cur' });
  const cross = mk('line', { class: 'tune-cross', y1: pad.t, y2: H - pad.b, visibility: 'hidden' }), dot = mk('circle', { class: 'tune-dot', r: 3.5, visibility: 'hidden' });
  const xEnd = mk('text', { class: 'tune-ax', x: W - pad.r, y: H - 4, 'text-anchor': 'end' });
  svg.append(grid, def, cur, cross, dot, xEnd);
  const legend = el('div', { class: 'tune-legend' }, el('span', { class: 'tune-key cur', text: 'This tuning' }), el('span', { class: 'tune-key def', text: 'Default' }));
  const read = el('p', { class: 'hint tune-read', text: '' });
  const node = el('figure', { class: 'tune-fig' }, svg, legend, read);
  let last = null, sy = 1, T = 1;
  const X = t => pad.l + (t / T) * (W - pad.l - pad.r), Y = y => pad.t + (1 - y / sy) * (H - pad.t - pad.b);
  const path = p => p.t.map((t, i) => (i ? 'L' : 'M') + X(Math.min(t, T)).toFixed(1) + ' ' + Y(clamp(p.y[i], -0.1 * sy, sy)).toFixed(1)).join('');
  const what = L.loop === 'pos' ? 'a 1 m move' : `a small ${L.id === 'yaw' ? 'turn' : 'lean'}`;
  function draw(p, d) {
    last = p; T = p.T; sy = Math.max(1.3, Math.min(2, Math.max(...p.y, ...(d ? d.y : [0])) * 1.08));
    grid.textContent = '';
    for (const [y, lab] of [[0, '0'], [1, 'target']]) { grid.append(mk('line', { x1: pad.l, x2: W - pad.r, y1: Y(y), y2: Y(y), class: y ? 'tune-target' : 'tune-base' })); const tx = mk('text', { class: 'tune-ax', x: pad.l - 4, y: Y(y) + 3, 'text-anchor': 'end' }); tx.textContent = y ? '1' : '0'; grid.append(tx); }
    xEnd.textContent = `${T < 2 ? T.toFixed(2) : T.toFixed(1)} s`;
    cur.setAttribute('d', path(p));
    def.setAttribute('d', d ? path({ t: d.t.filter(t => t <= T), y: d.y }) : ''); legend.hidden = !d;
    svg.setAttribute('aria-label', `Predicted response to ${what}: ${tuneVerdict(p, tuneFeel(tuneOf(), L.loop, L.axes ? L.axes[0] : 0)).text}`);
    setText(read, `Predicted response to ${what}. Hover to read it.`);
  }
  const at = e => { const r = svg.getBoundingClientRect(); return clamp(((e.clientX - r.left) / r.width * W - pad.l) / (W - pad.l - pad.r), 0, 1) * T; };
  svg.addEventListener('pointermove', e => {
    if (!last) return; const t = at(e); let i = 0; while (i < last.t.length - 1 && last.t[i + 1] <= t) i++;
    const x = X(last.t[i]), y = Y(clamp(last.y[i], -0.1 * sy, sy));
    cross.setAttribute('x1', x); cross.setAttribute('x2', x); dot.setAttribute('cx', x); dot.setAttribute('cy', y);
    cross.setAttribute('visibility', 'visible'); dot.setAttribute('visibility', 'visible');
    setText(read, `${last.t[i].toFixed(2)} s: ${Math.round(last.y[i] * 100)}% of the way${L.loop === 'pos' ? ` (${last.y[i].toFixed(2)} m)` : ''}`);
  });
  svg.addEventListener('pointerleave', () => { cross.setAttribute('visibility', 'hidden'); dot.setAttribute('visibility', 'hidden'); setText(read, `Predicted response to ${what}. Hover to read it.`); });
  return { node, draw };
}

// Everything on the section from the selected drone's tuning (fields, summaries, charts, notes).
function refreshTuning() {
  if (typeof atRender === 'function') atRender();
  if (!tuneUi.loops.size) return;
  const t = tuneOf(), lag = tuneMotorLag(), readers = new Set(tuneReaders()), navOn = typeof boardOf === 'function' && !!boardOf('nav');
  for (const r of tuneUi.refs) r();
  for (const r of tuneUi.raw) r();
  setText($('#tuneState'), tuneIsDefault() ? 'defaults' : 'tuned');
  $('#tuneReset').disabled = tuneIsDefault();
  for (const { L, f, chart, verdict, note, sum, fold } of tuneUi.loops.values()) {
    const ax = L.axes ? L.axes[0] : 0, feel = tuneFeel(t, L.loop, ax);
    const split = L.id === 'rp' && (t.att.kR[0] !== t.att.kR[1] || t.att.kW[0] !== t.att.kW[1] || t.att.kI[0] !== t.att.kI[1]);
    setText(sum, `${feel.hz.toFixed(2)} Hz · ζ ${feel.zeta.toFixed(2)}`);
    const ignored = !readers.has(L.law), off = ignored ? `${LAWS[L.law].def.title} is edited and keeps its own gains (its code doesn't read TUNE).` : L.loop === 'pos' && !navOn ? 'No board runs the Navigation task (Computers tab), so nothing holds position: the sticks lean the drone.' : '';
    for (const x of f) x.setOff(!!off, '');
    note.hidden = !off && !split; setText(note, off || (split ? 'Roll and pitch differ (Raw gains); these show roll\'s, and a change here sets both.' : ''));
    if (!fold.open) continue;                                                // (the predictions only when it's open)
    const p = tunePredict(t, L.loop, ax, lag), v = tuneVerdict(p, feel);
    const dKey = L.id + '|' + lag, d = tuneSame(t, TUNE_DEFAULTS) ? null : (tuneUi.defCache.get(dKey) || (tuneUi.defCache.set(dKey, tunePredict(tuneFix(null), L.loop, ax, lag)), tuneUi.defCache.get(dKey)));
    chart.draw(p, d);
    verdict.className = 'ui-status tune-verdict' + (v.tone ? ' ' + v.tone : ''); setText(verdict, (v.tone === 'bad' ? '⚠ ' : v.tone === 'warn' ? '△ ' : '') + v.text);
  }
}
$('#tuneReset').addEventListener('click', () => setTuning(tuneDefaults(), 'reset'));
