'use strict';
// Computers tab: the boards that fly the drone and the tasks each one runs, then every formula, under the task (and
// board) that runs it, as math plus its live code, editable. The physics and sensor models are the world's, not any
// board's: they come last.

const lawOpen = new Set();
const lawCards = new Map(); // key -> { card, status, ta, err, applyBtn, resetBtn }

function statusText(L) { return L.status === 'error' ? 'Error' : L.status === 'edited' ? 'Edited' : 'Default'; }
function mathBlock(lines) { const m = el('div', { class: 'math' }); for (const l of lines) m.append(el('div', { html: l })); return m; }

function lawCard(key) {
  const L = LAWS[key], d = L.def, open = lawOpen.has(key);
  const status = el('span', { class: 'lst' });
  const head = el('button', { class: 'law-head', type: 'button', 'aria-expanded': String(open), 'aria-controls': 'law-' + key },
    el('span', { class: 'law-title', text: d.title }), el('code', { class: 'law-key', text: d.key + '()' }), status);
  const body = el('div', { class: 'law-body', id: 'law-' + key }); body.hidden = !open;
  head.addEventListener('click', () => { const o = body.hidden; body.hidden = !o; head.setAttribute('aria-expanded', String(o)); o ? lawOpen.add(key) : lawOpen.delete(key); if (o) fitTa(ta); });

  body.append(mathBlock(d.math), el('p', { class: 'law-doc', text: d.doc }));
  if (d.used) body.append(el('p', { class: 'law-used', text: d.used }));
  const io = el('dl', { class: 'law-io' });
  for (const [n, t] of d.args) io.append(el('dt', { text: n }), el('dd', { text: t }));
  io.append(el('dt', { class: 'ret', text: 'returns' }), el('dd', { text: d.returns }));
  body.append(io);

  const taId = 'code-' + key;
  const ta = el('textarea', { class: 'code', id: taId, spellcheck: 'false', autocapitalize: 'off', autocomplete: 'off', 'aria-label': d.title + ' code' });
  ta.value = L.src;
  const err = el('p', { class: 'law-err', role: 'status' });
  const applyBtn = el('button', { class: 'btn primary', type: 'button', text: 'Apply' });
  const resetBtn = el('button', { class: 'btn', type: 'button', text: 'Revert to default' });
  const hint = el('span', { class: 'kbd', text: '⌘/Ctrl + Enter applies' });
  const doApply = () => {
    try { applyLaw(key, ta.value); err.textContent = ''; err.className = 'law-err'; flash(ta); }
    catch (e) { err.textContent = e.message; err.className = 'law-err on'; }
    save(); refreshLaw(key);
  };
  applyBtn.addEventListener('click', doApply);
  resetBtn.addEventListener('click', () => { resetLaw(key); ta.value = LAWS[key].src; err.textContent = ''; err.className = 'law-err'; save(); refreshLaw(key); fitTa(ta); });
  ta.addEventListener('input', () => { fitTa(ta); refreshLaw(key); });
  ta.addEventListener('keydown', e => {
    if (e.key === 'Enter' && (e.metaKey || e.ctrlKey)) { e.preventDefault(); doApply(); return; }
    if (e.key === 'Tab' && !e.shiftKey) { e.preventDefault(); const s = ta.selectionStart, en = ta.selectionEnd; ta.setRangeText('  ', s, en, 'end'); fitTa(ta); refreshLaw(key); }
  });
  body.append(ta, el('div', { class: 'law-actions' }, applyBtn, resetBtn, hint), err);
  // Flight formulas: what the step runner makes of them.
  let rn = null;
  if (typeof RN_SIGS !== 'undefined' && RN_SIGS[key]) {
    const note = el('p', { class: 'law-rn', role: 'status' });
    const steps = el('details', { class: 'steps' });
    const sum = el('summary', { text: 'Compiled steps' }), pre = el('pre', { class: 'listing' });
    steps.append(sum, pre);
    steps.addEventListener('toggle', () => { if (steps.open) pre.textContent = RN.P && RN.P.fns[key] ? rnListing(RN.P, key) : ''; });
    body.append(note, steps);
    rn = { note, steps, sum, pre };
  }
  const card = el('div', { class: 'law', 'data-law': key }, head, body);
  lawCards.set(key, { card, status, ta, err, applyBtn, resetBtn, rn });
  return card;
}
// A flight formula's line about the step runner, and its steps.
function refreshLawRunner(key) {
  const c = lawCards.get(key); if (!c || !c.rn) return;
  const L = LAWS[key], f = RN.P && RN.P.fns[key], r = c.rn;
  let msg = '', tone = '';
  if (RN.P && RN.P.errors[key]) { msg = `Not compiled for the drone: ${RN.P.errors[key]}. The boards fly the last version that compiled.`; tone = 'bad'; }
  else if (L.rnErr) { msg = `Not compiled for the drone: ${L.rnErr}. The boards fly the last version that compiled.`; tone = 'bad'; }
  else if (RN.trapped[key]) { msg = RN.trapped[key]; tone = 'bad'; }
  else if (RN.stage && RN.stage.keys.includes(key)) { msg = rnStageText(RN.stage); tone = 'warn'; }

  r.note.textContent = msg; r.note.className = 'law-rn' + (tone ? ' ' + tone : ''); r.note.hidden = !msg;
  if (f) {
    const calls = RN.calls[key] || 0, avg = calls ? Math.round(RN.steps[key] / calls) : null;
    r.sum.textContent = `Compiled steps: ${f.nInstr}` + (avg != null ? ` · ${avg} run per call in this flight` : '') + ` · at most ${f.maxSteps.toLocaleString()}`;
    if (r.steps.open) r.pre.textContent = rnListing(RN.P, key);
  } else r.sum.textContent = 'Compiled steps: none';
  r.steps.hidden = !f;
}
// The step runner's panel at the top of the tab.
function renderRunner() {
  const box = $('#rnBox'); if (!box) return;
  const P = RN.P, st = $('#rnStatus');
  if (!P) st.textContent = 'The flight formulas didn\'t compile: ' + RN.buildErr;
  else {
    const n = Object.keys(P.fns).length, kb = x => (x * 4 / 1024).toFixed(0) + ' KB';
    st.textContent = `${n} flight formulas compiled into one program: ${kb(P.code.length + P.constEnd)} of steps and constants, ${kb(P.arenaSize)} of working memory. Every board loads this same program; each runs the formulas of its tasks.`;
  }
  const log = $('#rnLog'); log.textContent = '';
  for (const l of RN.log.slice(0, 5)) log.append(el('li', { class: l.tone }, el('b', { text: `${l.t.toFixed(1)} s` }), ' ' + l.msg));
  log.hidden = !RN.log.length;
  const probs = $('#rnProblems'); probs.textContent = '';
  if (P) for (const [k, e] of Object.entries(P.errors)) probs.append(el('li', { class: 'bad', html: `<b>${LAWS[k].def.title}</b>: not compiled (${escapeHtml(e)})` }));
  probs.hidden = !probs.childElementCount;
  $('#rnDownload').disabled = !P;
  for (const k of Object.keys(RN_SIGS)) refreshLawRunner(k);
}
// Where an edit is on its way to the flying program.
function rnStageText(st, names) {
  const who = names ? st.keys.map(k => LAWS[k].def.title).join(', ') + ': ' : '';
  const t = st.t0 == null || typeof S === 'undefined' ? 0 : Math.max(0, S.t - st.t0);
  if (st.phase === 'shadow') return `${who}loaded${st.tests ? ` (${st.tests} self-tests passed)` : ''}, flying in the background beside the current version: ${t.toFixed(1)} of ${RN_SHADOW_S} s. It takes over if nothing goes wrong.`;
  if (st.phase === 'blend') return `${who}blending in: ${Math.round(Math.min(1, (t - RN_SHADOW_S) / RN_BLEND_S) * 100)}% of the answers from the new version.`;
  return `${who}${st.phase}…`;
}
const escapeHtml = s => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
function fitTa(ta) { ta.style.height = 'auto'; ta.style.height = Math.min(ta.scrollHeight + 2, 520) + 'px'; }
function flash(ta) { ta.classList.remove('ok'); void ta.offsetWidth; ta.classList.add('ok'); }

function refreshLaw(key) {
  const L = LAWS[key], c = lawCards.get(key); if (!c) return;
  const dirty = c.ta.value.trim() !== L.src.trim();
  c.status.textContent = statusText(L) + (dirty ? ' · unapplied' : '');
  c.status.className = 'lst ' + L.status + (dirty ? ' dirty' : '');
  c.ta.classList.toggle('dirty', dirty);
  c.resetBtn.disabled = L.status === 'default' && !dirty;
  if (L.status === 'error' && L.err && !dirty) { c.err.textContent = L.err; c.err.className = 'law-err on'; }
  const chip = document.querySelector(`.chain [data-go="${key}"]`); if (chip) chip.className = 'lchip ' + L.status;
  refreshFormulaStatus();
  refreshLawRunner(key);
}
function refreshFormulaStatus() {
  const ed = editedLaws(), bad = ed.filter(L => L.status === 'error').length;
  const badge = $('#editedCount'); badge.textContent = ed.length ? String(ed.length) : ''; badge.hidden = !ed.length; badge.className = 'count' + (bad ? ' bad' : '');
  $('#copyEdited').disabled = !ed.length; $('#revertAll').disabled = !ed.length;
}
lawListeners.add(key => {
  const c = lawCards.get(key);
  if (c && LAWS[key].status === 'error') { c.err.textContent = LAWS[key].err; c.err.className = 'law-err on'; }
  refreshLaw(key); save();
});

function goToLaw(key) {
  const c = lawCards.get(key); if (!c) return;
  const body = c.card.querySelector('.law-body');
  if (body.hidden) c.card.querySelector('.law-head').click();
  c.card.scrollIntoView({ block: 'start', behavior: window.matchMedia('(prefers-reduced-motion: reduce)').matches ? 'auto' : 'smooth' });
}

/* ───────── the tab ───────── */
const taskOfLaw = k => Object.keys(TASKS).find(t => TASKS[t].formulas.includes(k)) || null;
const COMP = { built: false };
function lawSection(id, title, blurb, keys, small) {
  const sec = el('section', { class: 'sec', id }, el('h2', {}, document.createTextNode(title + ' '), el('small', { text: small || '' })), el('p', { class: 'hint', text: blurb }));
  const list = el('div', { class: 'laws' }); for (const k of keys) list.append(lawCards.has(k) ? lawCards.get(k).card : lawCard(k));
  sec.append(list); return sec;
}
function buildComputers() {
  const pane = $('#paneForm'); pane.textContent = '';
  for (const d of LAW_DEFS) if (!lawCards.has(d.key)) lawCard(d.key);
  pane.append(el('section', { class: 'sec' },
    el('h2', { text: 'Flight computers' }),
    el('p', { class: 'hint', text: 'The boards on the drone and what each one runs. They are what flies: each is the drone\'s real flight code (the same C as on the ESP32 and the Pi) running here, fed by the simulated sensors, with the link between boards delaying what they send each other.' }),
    el('div', { class: 'boards', id: 'boardList' }),
    el('div', { class: 'board-add' }, el('label', { class: 'lbl', for: 'boardKind', text: 'Add a board' }),
      el('select', { id: 'boardKind' }, ...Object.entries(BOARD_KINDS).map(([k, b]) => el('option', { value: k, text: b.label }))),
      el('button', { class: 'btn', type: 'button', id: 'boardAdd', text: 'Add', onclick: () => {
        const C = JSON.parse(JSON.stringify(computers())), kind = $('#boardKind').value;
        if (C.boards.length >= BOARD_MAX) return;
        C.boards.push({ id: 99, kind, name: BOARD_KINDS[kind].label, tasks: [] }); setComputers(C, 'add');
      } }))),
    el('section', { class: 'sec' }, el('h2', { text: 'Tasks' }), el('p', { class: 'hint', text: 'Which board runs each part of the flight code. Its formulas are listed under it, below.' }), el('div', { class: 'tasks', id: 'taskRows' })),
    el('div', { id: 'taskLaws' }),
    el('section', { class: 'sec', id: 'rnBox' },
      el('h2', { text: 'The flight program' }),
      el('p', { class: 'hint', text: 'The flight formulas are compiled into steps for a small runner: the heavy math (matrices, quaternions, the least-squares allocation) is built into the runner in C, and the formulas are the steps between. Every board has the same runner and loads a program with the formulas of its tasks, so a formula edited here flies unchanged on the drone. An edit applied in flight reaches each board the way it would on the drone: the loader\'s checks, self-tests, a second of flying in the background beside the current version, then a short blend; if the new version stops in flight, the one before takes over again.' }),
      el('p', { class: 'rn-status', id: 'rnStatus', role: 'status' }),
      el('ol', { class: 'rn-log', id: 'rnLog', hidden: true }),
      el('ul', { class: 'rn-problems', id: 'rnProblems' }),
      el('div', { class: 'law-actions' }, el('button', { class: 'btn', type: 'button', id: 'rnDownload', text: 'Download the program (.rnp)', onclick: rnDownload }))));
  // the world's formulas, then the edit tools
  pane.append(lawSection('worldLaws', 'The world', 'Not flight code: what actually happens to the airframe (physics) and what the hardware reports (sensor models). Edits here change the world the flight computers have to cope with.', LAW_DEFS.filter(d => d.group === 'plant' || d.group === 'sensor').map(d => d.key), 'simulator only'));
  const copyBtn = el('button', { class: 'btn', type: 'button', id: 'copyEdited', text: 'Copy edited formulas' });
  const revertAll = el('button', { class: 'btn', type: 'button', id: 'revertAll', text: 'Revert all' });
  const copyOut = el('textarea', { class: 'code', id: 'copyOut', readonly: 'readonly', 'aria-label': 'Edited formulas' }); copyOut.hidden = true;
  const copyMsg = el('span', { class: 'kbd', role: 'status' });
  copyBtn.addEventListener('click', () => {
    const text = '// Edited formulas from Drone Force Bench. Paste over the matching functions in js/laws.js.\n\n' + editedLaws().map(L => L.src.trim()).join('\n\n') + '\n';
    const fallback = () => { copyOut.hidden = false; copyOut.value = text; fitTa(copyOut); copyOut.focus(); copyOut.select(); copyMsg.textContent = 'Select and copy the text below.'; };
    try { navigator.clipboard.writeText(text).then(() => { copyOut.hidden = true; copyMsg.textContent = 'Copied.'; }, fallback); } catch (e) { fallback(); }
  });
  revertAll.addEventListener('click', () => {
    if (revertAll.dataset.armed !== '1') { revertAll.dataset.armed = '1'; revertAll.textContent = 'Click again to revert all'; setTimeout(() => { revertAll.dataset.armed = ''; revertAll.textContent = 'Revert all'; }, 3000); return; }
    revertAll.dataset.armed = ''; revertAll.textContent = 'Revert all';
    for (const L of editedLaws()) { resetLaw(L.def.key); const c = lawCards.get(L.def.key); c.ta.value = LAWS[L.def.key].src; c.err.textContent = ''; c.err.className = 'law-err'; fitTa(c.ta); refreshLaw(L.def.key); }
    save();
  });
  const tools = el('section', { class: 'sec' }, el('h2', { text: 'Your edits' }),
    el('p', { class: 'hint', text: 'Every formula above is the code that runs. Edit one and apply it; it takes effect mid-flight. The math shows the default form.' }),
    el('div', { class: 'law-actions' }, copyBtn, revertAll, copyMsg), copyOut);
  if (!evalAllowed) tools.append(el('p', { class: 'note', text: 'This viewer blocks running edited JavaScript. The flight formulas still take edits: they are compiled for the boards. The world\'s formulas can be read here; open index.html from the repo to edit them.' }));
  pane.append(tools, el('section', { class: 'sec' }, el('h2', { text: 'Available inside formulas' }),
    el('p', { class: 'hint', html: 'Vectors are <code>[x, y, z]</code> arrays; 3×3 matrices are 9 numbers, row by row. Helpers: <code>add sub scl dot crs nrm unit clamp</code>, <code>m3v m3m m3T m3inv</code>, <code>qmat qmul qnorm matToQuat</code>, <code>randn</code> (standard normal, seeded so each reset replays the same noise), <code>bls(cols, lo, hi, w, W)</code>, and constants <code>G D2R R2D</code>. Anything in <code>Math</code> works too. A formula that throws or returns a wrong shape is switched off and the default takes over.' })));
  COMP.built = true;
  renderComputers(true);
  for (const d of LAW_DEFS) refreshLaw(d.key);
}
const pct = x => x < 0.01 ? '<1%' : Math.round(x * 100) + '%';
function renderComputers(full) {
  if (!COMP.built) return;
  const C = computers(), core = boardOf('core');
  const sig = JSON.stringify(C) + '|' + actuators().length + '|' + allSensors().map(c => c.kind).join(',');
  if (full || COMP.sig !== sig) {
    COMP.sig = sig;
    const list = $('#boardList'); list.textContent = '';
    for (const b of C.boards) {
      const K = BOARD_KINDS[b.kind], bud = boardBudget(b), wired = allSensors().filter(c => wiredTo(c) === b);
      const name = el('input', { type: 'text', class: 'board-name', value: b.name, maxlength: 24, 'aria-label': 'Board name' });
      name.addEventListener('change', () => { const C2 = JSON.parse(JSON.stringify(C)); C2.boards.find(x => x.id === b.id).name = name.value.trim() || K.label; setComputers(C2, 'name'); });
      const kind = el('select', { 'aria-label': 'Board' }, ...Object.entries(BOARD_KINDS).map(([k, x]) => el('option', { value: k, text: x.label, selected: k === b.kind ? 'selected' : null })));
      kind.addEventListener('change', () => { const C2 = JSON.parse(JSON.stringify(C)); C2.boards.find(x => x.id === b.id).kind = kind.value; setComputers(C2, 'kind'); });
      const only = b.tasks.includes('core') && C.boards.filter(x => BOARD_KINDS[x.kind].mcu).length < 2;
      const del = el('button', { class: 'icon-btn', type: 'button', text: '×', title: only ? 'The flight core needs a microcontroller: add another before removing this one' : 'Remove this board', 'aria-label': 'Remove ' + b.name, disabled: only || C.boards.length < 2 ? 'disabled' : null });
      del.addEventListener('click', () => {
        const C2 = JSON.parse(JSON.stringify(C)); C2.boards = C2.boards.filter(x => x.id !== b.id);
        if (b.tasks.includes('core')) C2.boards.find(x => BOARD_KINDS[x.kind].mcu).tasks.unshift('core');   // the flight core moves to another microcontroller
        setComputers(C2, 'remove');
      });
      const runs = b.tasks.length ? b.tasks.map(t => TASKS[t].label).join(' · ') : 'nothing yet (give it a task below)';
      const link = core && core.id !== b.id ? `Serial link to ${core.name}: ${(LINK_DELAY * 1000).toFixed(0)} ms each way` : C.boards.length > 1 ? 'The other boards talk to it over serial links' : '';
      const load = b.tasks.length ? `${pct(bud.load)} of ${K.cores > 1 ? 'one core' : 'its core'}${K.mcu ? ` · program ${bud.memKB.toFixed(0)} KB of ${K.ramKB} KB` : ''}` : '';
      const ex = el('div', { class: 'board-ex' });
      if (b.tasks.includes('core')) ex.append(el('button', { class: 'btn', type: 'button', text: 'Export the airframe (.dfa)', title: 'What the flight core flies on: send it with fly.py airframe FILE.dfa', onclick: () => boardsExport('airframe') }));
      if (b.tasks.includes('nav')) ex.append(el('button', { class: 'btn', type: 'button', text: 'Export the navigation config (.dnc)', title: K.mcu ? 'For the navigation on this board' : 'For dfb_pi on this Pi: ./dfb_pi --nav FILE.dnc', onclick: () => boardsExport('nav') }));
      if (!K.mcu && (b.tasks.includes('learn') || b.tasks.includes('super'))) ex.append(
        el('button', { class: 'btn', type: 'button', text: 'Export the airframe (.dfa)', title: 'The learning and the supervisor start from the same airframe as the flight core: ./dfb_pi --airframe FILE.dfa', onclick: () => boardsExport('airframe') }),
        el('button', { class: 'btn', type: 'button', text: 'Export the Pi config (.dlc)', title: 'Where the IMU sits, the motors\' heat and the battery, for the learning and the supervisor: ./dfb_pi --pi FILE.dlc', onclick: () => boardsExport('pi') }));
      list.append(el('div', { class: 'board' + (bud.load > 0.8 ? ' over' : '') },
        el('div', { class: 'board-head' }, name, kind, del),
        el('p', { class: 'board-note', text: K.note }),
        el('dl', { class: 'kv board-kv' },
          el('dt', { text: 'Runs' }), el('dd', { text: runs }),
          el('dt', { text: 'Wired to it' }), el('dd', { text: wired.length ? wired.map(c => c.name).join(', ') : '—' }),
          ...(link ? [el('dt', { text: 'Link' }), el('dd', { text: link })] : []),
          ...(load ? [el('dt', { text: 'Load' }), el('dd', { class: bud.load > 0.8 ? 'bad' : '', text: load + (bud.load > 1 ? ': too much for this board' : '') })] : [])),
        ex.childElementCount ? ex : el('span')));
    }
    $('#boardAdd').disabled = C.boards.length >= BOARD_MAX;
    // tasks: which board
    const rows = $('#taskRows'); rows.textContent = '';
    for (const [t, T] of Object.entries(TASKS)) {
      const cur = boardOf(t), sel = el('select', { 'aria-label': T.label + ' runs on' });
      if (!T.mcuOnly) sel.append(el('option', { value: '', text: 'No board (off)' }));
      for (const b of C.boards) if ((!T.mcuOnly || BOARD_KINDS[b.kind].mcu) && (!T.piOnly || !BOARD_KINDS[b.kind].mcu)) sel.append(el('option', { value: String(b.id), text: b.name, selected: cur && cur.id === b.id ? 'selected' : null }));
      if (!cur) sel.value = '';
      sel.addEventListener('change', () => {
        const C2 = JSON.parse(JSON.stringify(C)); for (const b of C2.boards) b.tasks = b.tasks.filter(x => x !== t);
        if (sel.value) C2.boards.find(b => String(b.id) === sel.value).tasks.push(t);
        setComputers(C2, 'task');
      });
      const noPi = T.piOnly && !C.boards.some(b => !BOARD_KINDS[b.kind].mcu);
      if (noPi) sel.disabled = true;
      rows.append(el('div', { class: 'task-row' }, el('div', {}, el('b', { text: T.label }), el('span', { class: 'hint', text: ' ' + T.what + (noPi ? ' Add a Raspberry Pi to run it.' : '') })), sel));
    }
    // the formulas, under the task that runs them
    const box = $('#taskLaws'); box.textContent = '';
    for (const [t, T] of Object.entries(TASKS)) {
      const b = boardOf(t); if (!b) continue;
      box.append(lawSection('laws-' + t, T.label, `${T.hz} times a second.`, T.formulas, 'on ' + b.name));
    }
    const off = Object.entries(TASKS).filter(([t]) => !boardOf(t));
    if (off.length) box.append(lawSection('laws-off', 'Not on any board', 'These formulas belong to tasks no board runs: they don\'t fly. ' + off.map(([, T]) => T.label).join(', ') + '.', off.flatMap(([, T]) => T.formulas)));
  }
  renderRunner();
}
// Files for the drone: the airframe for the flight core, the config for the navigation.
function boardsExport(what) {
  let data, ext;
  try { if (what === 'airframe') { data = fcAirframeBlob(); ext = '.dfa'; } else if (what === 'pi') { data = piConfigBlob(); ext = '.dlc'; } else { data = navConfigBlob(); ext = '.dnc'; } }
  catch (e) { rnEvent('Can\'t export: ' + e.message, 'bad'); renderRunner(); return; }
  const nm = ((typeof designs !== 'undefined' && designs.name) || 'drone').replace(/[^\w.-]+/g, '-');
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([data], { type: 'application/octet-stream' }));
  a.download = nm + ext; document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}
