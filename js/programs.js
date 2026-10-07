'use strict';
// Programs (docs/topic-bus.md): your own formulas, each on a board of your choice, run on the data bus.
//
// A program is part of the design (cfg.programs): a header (its name, its board, when it runs, the topics it reads,
// the topic it writes) and its code, a formula like the others: function name(st, inp, dt) { … return { … }; }.
// st is its own memory, inp the topics it reads (inp.baro.height: each topic by the last part of its name, its fields
// by its layout), dt the time since its last run; it returns the fields of the topic it writes (user.…), which is
// published on its board's bus, where other programs, boards and the Live data view can read it.
//
// It is compiled into its board's flight program (runner/fc/prog_core.c runs it there), so a board with only
// programs needs one too. Editing its code reloads it in flight like a formula; changing its header (what it reads or
// writes, when it runs, its board) restarts the flight, as wiring changes do.

const PROG_MAX = 16, PROG_READS = 8;
const programs = () => cfg.programs || (cfg.programs = []);
const programsOn = b => programs().filter(p => b && p.board === b.id && runsFormulas(b) && !programProblem(p));
const progKey = b => programsOn(b).map(p => p.name + '\u0001' + p.src).join('\u0000');   // (what a board's program has of them)

// The topics a design has, worked out from it (not from the running boards): the flight code's (as runner/fc/fc_core.c
// and nav_core.c publish them), the sensors' (on the board each is wired to), and every program's.
const BUS_BUILTIN = {
  core: [['fc.state', 'state settled hasHeight guided supMode learned formulaError'], ['fc.attitude', 'q[4] w[3]'], ['fc.imu', 'f[3] g[3]'], ['fc.height', 'height vz hasHeight'],
    ['fc.output', 'motor[12] servo[8]'], ['fc.torque', 'tau[3]'], ['cmd.pilot', 'arm roll pitch yaw throttle guided acc[3] heading']],
  nav: [['nav.estimate', 'p[3] v[3] hasHome ready landed'], ['nav.setpoint', 'target[3] vref[3] heading fly'], ['nav.command', 'acc[3] heading fly']],
};
function layoutFields(layout) {   // "q[4] w[3]" → [['q', 4], ['w', 3]], or null if it isn't a layout (as bus.h bus_layout_count)
  if (typeof layout !== 'string' || !/^[A-Za-z_]\w*(\[\d+\])?( [A-Za-z_]\w*(\[\d+\])?)*$/.test(layout)) return null;
  const f = layout.split(' ').map(x => { const m = /^(\w+)(?:\[(\d+)\])?$/.exec(x); return [m[1], m[2] ? +m[2] : 1, !!m[2]]; });
  const n = f.reduce((s, x) => s + x[1], 0), names = new Set(f.map(x => x[0]));
  return f.every(x => x[1] >= 1 && x[1] <= 32) && n <= 32 && names.size === f.length ? f : null;
}
const layoutSize = layout => { const f = layoutFields(layout); return f ? f.reduce((s, x) => s + x[1], 0) : 0; };
function busCatalog(except) {
  const out = [], C = computers(), core = boardOf('core'), nav = boardOf('nav');
  if (core) for (const [name, layout] of BUS_BUILTIN.core) out.push({ name, layout, board: core, from: 'flight core' });
  if (nav) for (const [name, layout] of BUS_BUILTIN.nav) out.push({ name, layout, board: nav, from: 'navigation' });
  for (const c of cfg.comps.filter(c => c.type === 'sensor')) { const b = wiredTo(c), S = BUS_SENSOR[c.kind]; if (b && S) out.push({ name: busSensorName(c), layout: S[1], board: b, from: c.name }); }
  for (const p of programs()) { if (p === except || p.id === (except && except.id)) continue; const b = C.boards.find(x => x.id === p.board); if (b && p.writes && layoutFields(p.writes.layout)) out.push({ name: p.writes.topic, layout: p.writes.layout, board: b, from: 'program ' + p.name }); }
  if (typeof apps === 'function') for (const a of apps()) { if (a === except || a.id === (except && except.id)) continue; const b = appBoard(a); if (b && a.writes && layoutFields(a.writes.layout)) out.push({ name: a.writes.topic, layout: a.writes.layout, board: b, from: 'app ' + a.name + (APP_KINDS[a.kind].runtime === 'native' ? ' (not simulated)' : '') }); }
  return out.map(t => ({ ...t, n: layoutSize(t.layout) }));
}
// A program's names for what it reads: each topic by the last part of its name (two alike: the whole name, . as _).
function progAliases(reads) {
  const last = n => n.split('.').pop(), seen = new Map(); for (const r of reads) seen.set(last(r), (seen.get(last(r)) || 0) + 1);
  return reads.map(r => seen.get(last(r)) > 1 ? r.replace(/\./g, '_') : last(r));
}
const fieldsType = layout => RT.rec(Object.fromEntries(layoutFields(layout).map(([f, k, isArr]) => [f, isArr ? RT.arr(RT.num, k) : RT.num])));
// Its signature for the step compiler: (st, inp, dt) → the fields of the topic it writes.
function progSig(p, cat = busCatalog(p)) {
  const al = progAliases(p.reads);
  return { names: ['st', 'inp', 'dt'], args: [RT.state(), RT.rec(Object.fromEntries(p.reads.map((r, i) => [al[i], fieldsType(cat.find(t => t.name === r).layout)]))), RT.num], ret: fieldsType(p.writes.layout) };
}
const zeroOf = t => t.k === 'num' ? 0 : t.k === 'arr' ? Array.from({ length: t.n }, () => zeroOf(t.el)) : t.k === 'rec' ? Object.fromEntries(Object.entries(t.f).map(([k, v]) => [k, zeroOf(v)])) : {};
// What's wrong with a program's header, or ''.
function programProblem(p, list = programs()) {
  if (!p || typeof p.name !== 'string' || !/^[A-Za-z_]\w{0,30}$/.test(p.name)) return 'Its name must be a word of letters, digits and _ (31 at most), starting with a letter.';
  if (LAWS[p.name] || RN_SIGS[p.name] || RN_KERNELS[p.name] || RN_HELPERS[p.name] || ['st', 'inp', 'dt', 'Math'].includes(p.name)) return `“${p.name}” is taken by a formula or a helper.`;
  if (list.some(q => q !== p && q.id !== p.id && q.name === p.name) || (typeof apps === 'function' && apps().some(a => a.name === p.name))) return `Another program or app is called “${p.name}”.`;
  const b = computers().boards.find(x => x.id === p.board); if (!b) return 'Choose the board it runs on.';
  if (!runsFormulas(b)) return `${b.name} runs apps, not formulas: choose a board that runs formulas (a board's settings say what it runs).`;
  if (!p.writes || !/^user\.[A-Za-z0-9_][A-Za-z0-9_.]{0,17}$/.test(p.writes.topic)) return 'It writes a topic under user. (up to 23 characters: letters, digits, _ and .).';
  if (!layoutFields(p.writes.layout)) return 'Its topic\'s fields: names separated by spaces, a count in brackets for a list (range rate ok, or v[3]); 32 numbers at most.';
  if (list.some(q => q !== p && q.id !== p.id && q.writes && q.writes.topic === p.writes.topic) || (typeof apps === 'function' && apps().some(a => a.writes.topic === p.writes.topic))) return `Another program or app writes ${p.writes.topic}.`;
  if (!Array.isArray(p.reads) || p.reads.length > PROG_READS) return 'It reads 8 topics at most.';
  const cat = busCatalog(p);
  for (const r of p.reads) { if (r === p.writes.topic) return 'It can\'t read the topic it writes.'; if (!cat.some(t => t.name === r)) return `Nothing on this drone publishes ${r}.`; }
  if (p.reads.reduce((s, r) => s + cat.find(t => t.name === r).n, 0) > 256) return 'Its inputs are too big (256 numbers at most).';
  if (!(p.every > 0) && !p.reads.includes(p.on)) return 'It runs when a topic it reads changes: choose which, or run it every so often.';
  if (p.every > 0 && !(p.every >= 0.001 && p.every <= 60)) return 'Every 1 ms to 60 s.';
  if (list.filter(q => q.board === b.id && q.id !== p.id).length >= PROG_MAX) return 'A board runs 16 programs at most.';
  return '';
}
function fixPrograms(list) {
  if (!Array.isArray(list)) return [];
  return list.slice(0, 64).filter(p => p && typeof p === 'object').map((p, i) => ({
    id: typeof p.id === 'string' && p.id ? p.id.slice(0, 16) : 'p' + i, name: String(p.name || 'program' + (i + 1)).slice(0, 31), board: Number.isInteger(p.board) ? p.board : null,
    every: Number.isFinite(+p.every) && +p.every > 0 ? +p.every : 0, on: typeof p.on === 'string' ? p.on : '', reads: Array.isArray(p.reads) ? p.reads.filter(r => typeof r === 'string').slice(0, PROG_READS) : [],
    writes: { topic: String(p.writes && p.writes.topic || 'user.out'), layout: String(p.writes && p.writes.layout || 'v') }, src: String(p.src || ''),
  }));
}
const progHeader = p => JSON.stringify([p.name, p.board, p.every, p.every > 0 ? '' : p.on, p.reads, p.writes]);

/* ───────── on the boards ───────── */
// The board's flight program, with its programs (boards.js boardProgram, boardImage).
function progSources(b) { const cat = busCatalog(), ps = programsOn(b); return { srcs: Object.fromEntries(ps.map(p => [p.name, p.src])), sigs: Object.fromEntries(ps.map(p => [p.name, progSig(p, cat.filter(t => t.name !== p.writes.topic))])) }; }
// At each start: every board's programs registered (their topics), then (once boards.js has routed every read) checked.
function progRegister() {
  brt.progAt = new Map(); brt.progErr = new Map();
  for (const b of computers().boards) {
    const w = brt.inst.get(b.id), ps = programsOn(b); if (!w) continue;
    w.prog_reset();
    for (const p of ps) {
      busTxt3(w, p.name, p.writes.topic, p.writes.layout);
      const i = w.prog_add(layoutSize(p.writes.layout), p.every > 0 ? p.every : 0);
      if (i < 0) brt.progErr.set(p.id, cstr(w, w.prog_why_ptr(), 96)); else brt.progAt.set(p.id, { b, w, i });
    }
  }
  for (const p of programs()) if (programProblem(p)) brt.progErr.set(p.id, programProblem(p));
}
// (and the apps': apps.js)
function progConnect(list = programs(), atOf = brt.progAt, errOf = brt.progErr) {
  for (const p of list) {
    const at = atOf.get(p.id); if (!at) continue;
    const { w, i } = at, fail = () => errOf.set(p.id, cstr(w, w.prog_why_ptr(), 96));
    let ok = true;
    for (const r of p.reads) { busTxt(w, r); if (w.prog_read(i) < 0) { fail(); ok = false; break; } }
    if (ok && !(p.every > 0)) { busTxt(w, p.on); if (w.prog_on(i) < 0) { fail(); ok = false; } }
    if (ok && w.prog_check(i) < 0) fail();
  }
}
const busTxt3 = (w, a, b, c) => { const x = new TextEncoder().encode(a + '\0' + b + '\0' + c + '\0'); new Uint8Array(w.memory.buffer, w.txt_ptr(), x.length).set(x); };
// How each program is doing on its board: { runs, fails, waits, err, ok }.
function progStats(atOf = brt.progAt) {
  const out = new Map(), byBoard = new Map();
  for (const [id, at] of atOf || []) {
    if (!byBoard.has(at.w)) { const n = at.w.prog_list(); byBoard.set(at.w, new Float32Array(at.w.memory.buffer, at.w.fr_ptr(), n * 5).slice()); }
    const v = byBoard.get(at.w), k = at.i * 5; out.set(id, { ok: v[k] > 0.5, runs: v[k + 1], fails: v[k + 2], waits: v[k + 3], err: v[k + 4] });
  }
  return out;
}

/* ───────── the editor (Formula editor → Programs) ───────── */
const progCards = new Map();   // id → { card, draft, … } (drafts kept while you look at other formulas)
const PROG_TEMPLATE = name => `function ${name}(st, inp, dt) {
  // st: this program's own memory between runs (starts as {}); inp: the topics it reads; dt: seconds since its last run.
  // Return the fields of the topic it writes.
  return { v: 0 };
}`;
function newProgram() {
  const C = computers(), taken = new Set(programs().map(p => p.name));
  let k = 1; while (taken.has('program' + k)) k++;
  const name = 'program' + k, b = C.boards.find(x => !BOARD_KINDS[x.kind].mcu && runsFormulas(x)) || boardOf('core') || C.boards.find(runsFormulas);
  const p = { id: 'p' + Date.now().toString(36), name, board: b ? b.id : null, every: 0.05, on: '', reads: [], writes: { topic: 'user.' + name, layout: 'v' }, src: PROG_TEMPLATE(name), draft: true };
  progCards.delete(p.id); showProgram(p);
}
function showProgram(pOrId) {
  const p = typeof pOrId === 'string' ? programs().find(x => x.id === pOrId) || (progCards.get(pOrId) || {}).p : pOrId; if (!p) return;
  COMP.formula = 'prog:' + p.id; renderFormulaChoices();
  for (const old of [...$('#formulaActive').children]) $('#formulaStore').append(old);
  let c = progCards.get(p.id); if (!c) { c = progCard(p); progCards.set(p.id, c); }
  $('#formulaActive').append(c.card); setText($('#formulaTitle'), c.d.name || 'New program');
  const b = computers().boards.find(x => x.id === c.d.board); $('#formulaOwner').textContent = 'Program · ' + (b ? b.name : 'not assigned');
  c.sync(); fitTa(c.ta);
}
function progCard(p) {
  const d = JSON.parse(JSON.stringify({ ...p, draft: undefined })), isNew = !!p.draft;   // the draft being edited
  const name = UI.input({ type: 'text', 'aria-label': 'Program name', value: d.name, maxlength: 31, spellcheck: 'false' });
  const board = UI.select({ 'aria-label': 'Board' }), mode = UI.select({ 'aria-label': 'When it runs' }), on = UI.select({ 'aria-label': 'The topic whose change runs it' });
  const every = UI.input({ type: 'number', 'aria-label': 'Every (ms)', min: 1, max: 60000, step: 1, value: String(Math.round((d.every || 0.05) * 1000)) });
  const topic = UI.input({ type: 'text', 'aria-label': 'Topic it writes (after user.)', value: d.writes.topic.replace(/^user\./, ''), maxlength: 18, spellcheck: 'false' });
  const layout = UI.input({ type: 'text', 'aria-label': 'Its fields', value: d.writes.layout, spellcheck: 'false' });
  const reads = el('div', { class: 'prog-reads' }), shape = el('pre', { class: 'prog-shape' }), stats = UI.status({ class: 'hint prog-stats', role: 'status' });
  const ta = UI.textarea({ class: 'code', spellcheck: 'false', autocapitalize: 'off', autocomplete: 'off', 'aria-label': 'Program code' }); ta.value = d.src;
  const err = UI.status({ class: 'law-err', role: 'status' }), apply = UI.button({ class: 'btn primary', text: isNew ? 'Add program' : 'Apply' }), del = UI.button({ class: 'btn', text: isNew ? 'Discard' : 'Delete' });
  mode.append(el('option', { value: 'change', text: 'when a topic it reads changes' }), el('option', { value: 'every', text: 'every' }));
  const c = { p, d, ta, card: null, isNew };
  const readHeader = () => {
    d.name = name.value.trim(); d.board = +board.value; d.writes = { topic: 'user.' + topic.value.trim(), layout: layout.value.trim().replace(/\s+/g, ' ') };
    d.every = mode.value === 'every' ? Math.max(1, +every.value || 50) / 1000 : 0; d.on = d.every > 0 ? '' : on.value;
    d.reads = [...reads.querySelectorAll('input:checked')].map(x => x.value); d.src = ta.value;
  };
  c.sync = () => {   // the controls from the draft, and what its code gets and returns
    const C = computers(); board.replaceChildren(...C.boards.filter(runsFormulas).map(b => el('option', { value: b.id, text: b.name + ' · ' + BOARD_KINDS[b.kind].label })));
    const ok = C.boards.some(b => b.id === d.board && runsFormulas(b));
    if (!ok) board.prepend(el('option', { value: '', text: 'not assigned' }));
    board.value = ok ? String(d.board) : '';
    mode.value = d.every > 0 ? 'every' : 'change'; every.hidden = !(d.every > 0); on.hidden = d.every > 0;
    const cat = busCatalog(d), byBoard = new Map(); for (const t of cat) { if (!byBoard.has(t.board)) byBoard.set(t.board, []); byBoard.get(t.board).push(t); }
    reads.replaceChildren(...[...byBoard].map(([b, ts]) => el('fieldset', {}, el('legend', { text: b.name }), ...ts.map(t => el('label', { class: 'check', title: t.layout + ' · from ' + t.from },
      Object.assign(UI.input({ type: 'checkbox', value: t.name }), { checked: d.reads.includes(t.name), onchange: () => { readHeader(); c.sync(); } }), el('code', { text: t.name }))))));
    if (!cat.length) reads.append(el('p', { class: 'hint', text: 'No topics yet.' }));
    on.replaceChildren(...(d.reads.length ? d.reads : ['']).map(r => el('option', { value: r, text: r || 'choose topics it reads first' })));
    if (d.reads.includes(d.on)) on.value = d.on; else if (d.reads.length) { d.on = d.reads[0]; on.value = d.on; }
    const al = progAliases(d.reads.filter(r => cat.some(t => t.name === r)));
    const inp = d.reads.filter(r => cat.some(t => t.name === r)).map((r, i) => (layoutFields(cat.find(t => t.name === r).layout) || []).map(([f, k, a]) => `inp.${al[i]}.${f}${a ? `[0…${k - 1}]` : ''}`).join(', ')).join('\n') || '(nothing: it reads no topics)';
    const out = layoutFields(d.writes.layout);
    shape.textContent = `function ${d.name || 'name'}(st, inp, dt)\n\nreads:\n${inp}\n\nreturns { ${out ? out.map(([f, k, a]) => a ? `${f}: [${k} numbers]` : f).join(', ') : '…'} }  → ${d.writes.topic}`;
    const why = programProblem(d, c.isNew ? [...programs(), d] : programs().map(q => q.id === d.id ? d : q)); err.textContent = why; err.className = 'ui-status law-err' + (why ? ' on' : '');
    progStatsLine(c);
  };
  const doApply = () => {
    readHeader();
    const list = c.isNew || !programs().some(q => q.id === d.id) ? [...programs(), d] : programs().map(q => q.id === d.id ? d : q);
    let why = programProblem(d, list);
    if (!why) try { const sig = progSig(d); const P = rnCompileAll({ [d.name]: d.src }, { [d.name]: sig }, { throw: true, consts: rnConsts() }); rnVerify(P); } catch (e) { why = 'It doesn\'t compile: ' + e.message; }
    err.textContent = why; err.className = 'ui-status law-err' + (why ? ' on' : ''); if (why) return;
    const old = programs().find(q => q.id === d.id), header = !old || progHeader(old) !== progHeader(d);
    cfg.programs = list.map(q => q.id === d.id ? JSON.parse(JSON.stringify(d)) : q); c.isNew = false; c.p = cfg.programs.find(q => q.id === d.id);
    apply.textContent = 'Apply'; del.textContent = 'Delete'; undoKey = 'program:' + d.id; flash(ta); save();
    if (header) { brt.sig = null; doReset(); renderComputers(true); } else if (typeof boardsStageProgram === 'function') boardsStageProgram();
    showProgram(d.id);
  };
  apply.addEventListener('click', doApply);
  del.addEventListener('click', () => {
    if (c.isNew) { progCards.delete(d.id); openFormulaEditor(formulaGroups()[0].keys[0]); return; }
    if (del.dataset.armed !== '1') { del.dataset.armed = '1'; del.textContent = 'Click again to delete'; setTimeout(() => { del.dataset.armed = ''; del.textContent = 'Delete'; }, 3000); return; }
    cfg.programs = programs().filter(q => q.id !== d.id); progCards.delete(d.id); undoKey = 'program:' + d.id; save(); brt.sig = null; doReset(); renderComputers(true);
    openFormulaEditor(formulaGroups()[0].keys[0]);
  });
  name.addEventListener('input', () => {   // the code's function name follows, while it is the old one
    const was = d.name, now = name.value.trim(), m = new RegExp('^(\\s*function\\s+)' + was + '(\\s*\\()');
    if (was && /^[A-Za-z_]\w*$/.test(now) && m.test(ta.value)) { ta.value = ta.value.replace(m, '$1' + now + '$2'); }
    if (!topic.dataset.edited && topic.value === was) topic.value = now;                       // (and its topic, until you set one)
    readHeader(); c.sync();
  });
  topic.addEventListener('input', () => { topic.dataset.edited = '1'; });
  for (const x of [topic, layout, every]) x.addEventListener('input', () => { readHeader(); c.sync(); });
  for (const x of [board, mode, on]) x.addEventListener('change', () => { if (x === mode && mode.value === 'every' && !(d.every > 0)) every.value = '50'; readHeader(); c.sync(); });
  ta.addEventListener('input', () => { fitTa(ta); d.src = ta.value; });
  ta.addEventListener('keydown', e => {
    if (e.key === 'Enter' && (e.metaKey || e.ctrlKey)) { e.preventDefault(); doApply(); return; }
    if (e.key === 'Tab' && !e.shiftKey) { e.preventDefault(); const s = ta.selectionStart, en = ta.selectionEnd; ta.setRangeText('  ', s, en, 'end'); fitTa(ta); }
  });
  c.stats = stats;
  c.card = UI.card({ class: 'law prog-card', 'data-program': d.id },
    el('div', { class: 'prog-head' }, hardwareField('Name', name), hardwareField('Board', board)),
    el('div', { class: 'prog-head' }, hardwareField('Runs', mode), el('div', { class: 'prog-when' }, on, el('label', { class: 'prog-every' }, every, el('span', { text: 'ms' })))),
    UI.details({ class: 'prog-reads-box', title: 'Reads', open: true }, reads),
    el('div', { class: 'prog-head' }, hardwareField('Writes', el('span', { class: 'prog-topic' }, el('code', { text: 'user.' }), topic)), hardwareField('Its fields', layout)),
    el('p', { class: 'hint', text: 'Fields: names separated by spaces, a count in brackets for a list (range rate ok, or v[3]).' }),
    shape, ta, el('div', { class: 'law-actions' }, apply, del, el('span', { class: 'kbd', text: '⌘/Ctrl + Enter applies' })), err, stats,
    el('p', { class: 'hint', text: 'Changing what it reads or writes, when it runs or its board restarts the flight; editing its code reloads it in flight, checked first as any formula is.' }));
  return c;
}
function progStatsLine(c) {
  if (!c.stats) return;
  if (c.isNew) { setText(c.stats, 'Not added yet.'); return; }
  const e = brt.progErr && brt.progErr.get(c.d.id), s = progStats().get(c.d.id);
  setText(c.stats, e ? 'Not running: ' + e : !s ? 'Not running (the flight computers aren\'t running).' : `${s.ok ? 'Running' : 'Not running'} on ${(computers().boards.find(b => b.id === c.d.board) || {}).name || '?'}: ${s.runs} runs${s.waits ? `, ${s.waits} waited for its inputs` : ''}${s.fails ? `, ${s.fails} failed (error ${s.err})` : ''}.`);
}
setInterval(() => { const d = document.getElementById('formulaDlg'); if (!d || !d.open || !String(COMP.formula || '').startsWith('prog:')) return; const c = progCards.get(COMP.formula.slice(5)); if (c && (typeof fleet === 'undefined' || !fleet.ready || fleet.selected)) (typeof fleet !== 'undefined' && fleet.ready ? withDrone(fleet.selected, () => progStatsLine(c)) : progStatsLine(c)); }, 500);
