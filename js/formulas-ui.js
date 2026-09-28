'use strict';
// Formulas tab: shows every governing law as math plus its live code, and lets you edit it.

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
  const card = el('div', { class: 'law', 'data-law': key }, head, body);
  lawCards.set(key, { card, status, ta, err, applyBtn, resetBtn });
  return card;
}
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

function buildFormulas() {
  const pane = $('#paneForm'); pane.textContent = '';
  const intro = el('section', { class: 'sec' },
    el('h2', { text: 'Governing equations' }),
    el('p', { class: 'hint', text: 'Every formula below is the code the simulator runs. Edit one and apply it; the change takes effect on the next time step, mid-flight. The math shows the default form.' }),
    mathBlock(LAW_OVERVIEW));
  const chain = (label, keys) => {
    const row = el('div', { class: 'chain' });
    keys.forEach((k, i) => {
      if (i) row.append(el('span', { class: 'arr', 'aria-hidden': 'true', text: '→' }));
      row.append(el('button', { class: 'lchip ' + LAWS[k].status, type: 'button', 'data-go': k, text: LAWS[k].def.title, onclick: () => goToLaw(k) }));
    });
    return el('div', { class: 'chain-wrap' }, el('span', { class: 'lbl', text: label }), row);
  };
  intro.append(chain('Each control step (1 kHz)', LAW_CHAIN.ctrl), chain('Each physics step (2 kHz), sensors at their own rates', LAW_CHAIN.plant), chain('Supervisor (10 Hz, over a link)', LAW_CHAIN.super));
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
  intro.append(el('div', { class: 'law-actions' }, copyBtn, revertAll, copyMsg), copyOut);
  if (!evalAllowed) intro.append(el('p', { class: 'note', text: 'This viewer blocks running edited code, so you can read the formulas here but not apply changes. Open index.html from the repo to edit them.' }));
  pane.append(intro);

  const groups = [['plant', 'Physics (the plant)', 'What actually happens to the airframe. Edits here change the world the controller has to cope with.'],
    ['sensor', 'Sensors', 'What the hardware reports. Each model turns the true quantity at the sensor into a reading with noise, bias, drift and limits. Each sensor keeps its own state between samples.'],
    ['est', 'Estimation', 'What the flight software believes. These turn the sensor readings into the attitude, rate, position and velocity the controller flies on.'],
    ['learn', 'Identification', 'What the flight software learns about its own airframe from flight data, so it doesn\'t need prop, mass or inertia figures.'],
    ['ctrl', 'Controller', 'What the flight software decides. It works from the estimate, and only knows the modeled mass, inertia and actuator health.'],
    ['super', 'Supervisor (companion computer)', 'A separate, slower computer (like a Raspberry Pi) that watches the flight controller and the health sensors over a link, and changes the controller\'s settings when a part fails, weakens or overheats.']];
  for (const [g, title, blurb] of groups) {
    const sec = el('section', { class: 'sec' }, el('h2', { text: title }), el('p', { class: 'hint', text: blurb }));
    const list = el('div', { class: 'laws' });
    for (const d of LAW_DEFS) if (d.group === g) list.append(lawCard(d.key));
    sec.append(list); pane.append(sec);
  }
  pane.append(el('section', { class: 'sec' }, el('h2', { text: 'Available inside formulas' }),
    el('p', { class: 'hint', html: 'Vectors are <code>[x, y, z]</code> arrays; 3×3 matrices are 9 numbers, row by row. Helpers: <code>add sub scl dot crs nrm unit clamp</code>, <code>m3v m3m m3T m3inv</code>, <code>qmat qmul qnorm matToQuat</code>, <code>randn</code> (standard normal, seeded so each reset replays the same noise), <code>bls(cols, lo, hi, w, W)</code>, and constants <code>G D2R R2D</code>. Anything in <code>Math</code> works too. A formula that throws or returns a wrong shape is switched off and the default takes over.' })));
  for (const d of LAW_DEFS) refreshLaw(d.key);
}
