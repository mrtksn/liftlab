'use strict';
// Designs: undo and redo of airframe edits, a library of saved designs, and design files.
//
// A design is the airframe alone: frame mass, every part, and the steering mode. Flight state, learning and
// edited formulas aren't part of it.
//
// Saved designs live in your account when this page runs as a published artifact (private to you, on any
// device), otherwise in this browser. "Save to file" and "Open file…" move a design anywhere, such as into
// the project's repository.

/* ───────── undo / redo ───────── */
let undoKey = null;   // set by edited(): repeated edits to one field within a moment are one step
const undo = { stack: [], i: -1, lastKey: null, lastT: 0, restoring: false };
const designOf = () => ({ frame: cfg.frame.mass, frameShape: frameShapeOf(), comps: cfg.comps, mode, battery: cfg.battery, computers: computers() });
const designSnap = () => JSON.stringify(designOf());

// Called from save() after every change. Records a step when the design itself changed.
function markDesign() {
  const key = undoKey; undoKey = null;
  if (undo.restoring || !undo.stack.length) return;
  const s = designSnap();
  if (undo.stack[undo.i] === s) { renderDesignState(); return; }
  const now = performance.now();
  if (key && key === undo.lastKey && now - undo.lastT < 1000 && undo.i > 0 && undo.i === undo.stack.length - 1) undo.stack[undo.i] = s;
  else {
    undo.stack.length = undo.i + 1; undo.stack.push(s); undo.i++;
    if (undo.stack.length > 300) { undo.stack.shift(); undo.i--; }
  }
  undo.lastKey = key; undo.lastT = now;
  renderUndo(); renderDesignState();
}
function restoreSnap(s) {
  const d = JSON.parse(s);
  let restart = false;
  undo.restoring = true;
  try {
    cfg.frame.mass = d.frame; setFrameShape(d.frameShape); cfg.comps = d.comps; cfg.battery = { ...defaultBattery(), ...(d.battery || {}) }; uid = Math.max(uid, ...cfg.comps.map(c => c.id + 1));
    if (d.computers) {   // (other boards or tasks: they start again, as when you change them yourself)
      const was = JSON.stringify(computers()); cfg.computers = fixComputers(d.computers); if (typeof syncFlightUi === 'function') syncFlightUi();
      if (JSON.stringify(cfg.computers) !== was) { brt.sig = null; restart = true; }
    }
    if (typeof renderBattery === 'function') renderBattery();
    setMode(d.mode, false); frameMassField.refresh(); renderFrameShape(); structural();
    if (typeof edit !== 'undefined' && edit.sel != null) selectComp(compById(edit.sel) ? edit.sel : null);
  } finally { undo.restoring = false; }
  if (restart) doReset();
  undo.lastKey = null; renderUndo(); renderDesignState();
}
function undoStep() { if (undo.i > 0) { undo.i--; restoreSnap(undo.stack[undo.i]); } }
function redoStep() { if (undo.i < undo.stack.length - 1) { undo.i++; restoreSnap(undo.stack[undo.i]); } }
function renderUndo() {
  for (const id of ['undoBtn', 'undoBtn2']) { const b = document.getElementById(id); if (b) b.disabled = undo.i <= 0; }
  for (const id of ['redoBtn', 'redoBtn2']) { const b = document.getElementById(id); if (b) b.disabled = undo.i >= undo.stack.length - 1; }
}
window.addEventListener('keydown', e => {
  if (!(e.ctrlKey || e.metaKey) || e.altKey || typingIn(e.target)) return;   // text fields keep their own undo
  const k = e.key.toLowerCase();
  if (k === 'z' && !e.shiftKey) { e.preventDefault(); undoStep(); }
  else if ((k === 'z' && e.shiftKey) || k === 'y') { e.preventDefault(); redoStep(); }
});

/* ───────── saved designs ───────── */
const designs = { list: [], cur: null, name: '', savedSnap: null, baseSnap: null, col: null, where: 'browser' };
// Changed and not saved: different from how it was loaded (a layout, a design, a file) and from its last save.
const designChanged = () => { const s = designSnap(); return s !== designs.baseSnap && !(designs.cur && s === designs.savedSnap); };
const LSD = 'drone-force-bench-v1-designs';
const claudeUse = name => (window.claude && typeof window.claude.use === 'function') ? window.claude.use(name).catch(() => null) : Promise.resolve(null);
const localDesigns = () => { try { const l = JSON.parse(localStorage.getItem(LSD) || '[]'); return Array.isArray(l) ? l : []; } catch (e) { return []; } };
const writeLocal = list => { try { localStorage.setItem(LSD, JSON.stringify(list)); return true; } catch (e) { return false; } };
const newId = () => 'd' + Date.now().toString(36) + Math.random().toString(36).slice(2, 7);
const sortList = l => l.sort((a, b) => (b.savedAt || 0) - (a.savedAt || 0));

// The library: in your account when the page can reach it, else this browser.
async function initDesignStore() {
  designs.list = sortList(localDesigns()); renderDesigns();
  const [db, user] = await Promise.all([claudeUse('db'), claudeUse('user')]);
  const id = user ? await user.id().catch(() => null) : null;
  if (!db || !id) return;
  try { designs.col = db.collection('data/users/' + id); } catch (e) { return; }
  designs.where = 'account';
  designs.col.onSnapshot(snap => {
    designs.list = sortList(snap.docs.map(d => ({ id: d.id, ...d.data() })).filter(d => d.design && Array.isArray(d.design.comps)));
    renderDesigns();
  }, () => { designs.col = null; designs.where = 'browser'; designs.list = sortList(localDesigns()); renderDesigns(); });
  renderDesigns();
}
async function storeDesign(rec) {
  if (designs.col) {
    try { await designs.col.doc(rec.id).set({ name: rec.name, savedAt: rec.savedAt, design: rec.design }); return true; }
    catch (e) {
      if (e && e.code === 'quota_exceeded') { designNote('Your saved designs are full. Delete some to save more.'); return false; }
      designs.col = null; designs.where = 'browser';   // can't write here: keep them in this browser instead
    }
  }
  const l = localDesigns().filter(d => d.id !== rec.id); l.push(rec);
  if (!writeLocal(l)) { designNote('This browser won\'t store it. Use Save to file instead.'); return false; }
  designs.list = sortList(l); return true;
}
async function removeDesign(id) {
  if (designs.col) { try { await designs.col.doc(id).delete(); return; } catch (e) { designNote('Couldn\'t delete it just now.'); return; } }
  const l = localDesigns().filter(d => d.id !== id); writeLocal(l); designs.list = sortList(l); renderDesigns();
}

function designLoaded(id, name) {   // after a design is opened, or a preset loaded (id null)
  designs.cur = id; designs.name = name || ''; designs.pendingClean = true;
  const inp = document.getElementById('designName'); if (inp) inp.value = designs.name;
}
async function saveDesign() {
  const inp = $('#designName'), name = (inp.value || '').trim() || 'Untitled design';
  const same = designs.list.find(d => d.name === name);
  const rec = { id: same ? same.id : (designs.cur && designs.list.find(d => d.id === designs.cur && d.name === name) ? designs.cur : newId()), name, savedAt: Date.now(), design: JSON.parse(designSnap()) };
  const btn = $('#designSave'); btn.disabled = true;
  const ok = await storeDesign(rec);
  btn.disabled = false;
  if (!ok) return;
  designs.cur = rec.id; designs.name = name; inp.value = name; designs.savedSnap = designs.baseSnap = designSnap();
  designNote(same ? `Updated “${name}”.` : `Saved “${name}”.`);
  renderDesigns();
}
function applyDesign(d) {
  cfg.frame.mass = +d.frame || 0.45; setFrameShape(d.frameShape);
  cfg.comps = migrateComps(JSON.parse(JSON.stringify(d.comps)));
  cfg.battery = { ...defaultBattery(), ...(d.battery || {}) };
  if (d.computers) cfg.computers = fixComputers(computersWithRadio(d.computers));   // (a design saved before boards keeps the ones you have)
  if (typeof syncFlightUi === 'function') setTimeout(syncFlightUi);
  uid = Math.max(uid, ...cfg.comps.map(c => c.id + 1));
  setMode(['level', 'mixed'].includes(d.mode) ? d.mode : 'tilt', false); openSet.clear();
  if (typeof edit !== 'undefined' && edit.sel != null) selectComp(null);
}
function openDesign(rec) {
  applyDesign(rec.design); designLoaded(rec.id, rec.name); afterLoad();
}

/* ───────── design files ───────── */
const FILE_FORMAT = 'drone-force-bench-design';
async function exportDesign(rec) {
  const name = rec ? rec.name : (($('#designName').value || '').trim() || 'Untitled design');
  const body = JSON.stringify({ format: FILE_FORMAT, version: 1, name, savedAt: new Date().toISOString(), design: rec ? rec.design : JSON.parse(designSnap()) }, null, 1);
  const filename = (name.replace(/[^\w\- ]+/g, '').trim().replace(/\s+/g, '-') || 'design') + '.json';
  const dl = await claudeUse('downloads');
  if (dl) {
    try { await dl.save({ filename, data: body }); designNote(`Saved ${filename}.`); return; }
    catch (e) { if (e && e.code === 'declined') return; if (e && e.code === 'rate_limited') { designNote('A save is already waiting for you.'); return; } }
  }
  try {   // outside the artifact viewer: an ordinary browser download
    const a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([body], { type: 'application/json' })); a.download = filename;
    document.body.append(a); a.click(); a.remove(); setTimeout(() => URL.revokeObjectURL(a.href), 5000);
    designNote(`Saved ${filename}.`);
  } catch (e) { designNote('This page can\'t save files here.'); }
}
function readDesignFile(text) {   // a design file, or a design copied from the page's own storage
  const o = JSON.parse(text);
  const d = o.format === FILE_FORMAT ? o.design : o.cfg ? { frame: o.cfg.frame && o.cfg.frame.mass, frameShape: o.cfg.frame, comps: o.cfg.comps, mode: o.mode } : o;
  if (!d || !Array.isArray(d.comps) || !d.comps.every(c => c && typeof c.type === 'string' && Array.isArray(c.pos))) throw new Error('not a design');
  return { name: (o.name || '').toString().slice(0, 60), design: { frame: d.frame, frameShape: d.frameShape, comps: d.comps, mode: d.mode } };
}
async function importDesign(file) {
  try {
    const got = readDesignFile(await file.text()), name = got.name || file.name.replace(/\.json$/i, '');
    if (!await askToSave(name)) return;
    applyDesign(got.design); designLoaded(null, name); afterLoad();
    designNote(`Opened ${file.name}. Save to keep it in your designs.`);
  } catch (e) { designNote(`${file.name} isn't a design file.`); }
}

/* ───────── unsaved changes ───────── */
// Before something replaces the airframe on screen: if it has changes nobody saved, ask to save them,
// throw them away, or stay. Resolves true to go ahead (and runs `then`), false to stay.
function askToSave(what, then) {
  return new Promise(resolve => {
    const go = () => { if (then) then(); resolve(true); };
    const dlg = document.getElementById('saveAsk');
    if (!designChanged() || !dlg || typeof dlg.showModal !== 'function') { go(); return; }
    const cur = designs.cur && designs.list.find(d => d.id === designs.cur), name = $('#saveAskName');
    $('#saveAskWhy').textContent = (cur ? `You changed “${cur.name}” since you last saved it.` : 'You changed this airframe and haven’t saved it.') + ` Opening “${what}” replaces it.`;
    name.value = cur ? cur.name : (($('#designName').value || '').trim());
    dlg.returnValue = '';
    dlg.addEventListener('close', async () => {
      const r = dlg.returnValue;
      if (r === 'discard') { go(); return; }
      if (r !== 'save') { resolve(false); return; }
      let n = (name.value || '').trim() || 'Untitled design';   // never write over a different saved design from here
      const taken = x => designs.list.some(d => d.name === x && d.id !== designs.cur);
      if (taken(n)) { const b = n; for (let k = 2; taken(n); k++) n = `${b} (${k})`; }
      $('#designName').value = n;
      await saveDesign();
      if (designs.savedSnap === designSnap()) go(); else { designNote('Couldn’t save the design, so nothing was replaced.'); resolve(false); }
    }, { once: true });
    dlg.showModal(); name.focus(); name.select();
  });
}

/* ───────── panel ───────── */
let noteTimer = 0;
function designNote(t) {
  const n = document.getElementById('designNote'); if (!n) return;
  n.textContent = t; n.hidden = !t; clearTimeout(noteTimer); if (t) noteTimer = setTimeout(() => { n.hidden = true; }, 6000);
}
function renderDesignState() {
  const st = document.getElementById('designState'); if (!st) return;
  if (designs.pendingClean) { designs.pendingClean = false; designs.savedSnap = designs.cur ? designSnap() : null; designs.baseSnap = designSnap(); }
  const dirty = !designs.cur || designs.savedSnap !== designSnap();
  st.textContent = designs.cur ? (dirty ? 'unsaved changes' : 'saved') : '';
}
const designSummary = d => {
  const cs = d.design.comps, n = t => cs.filter(c => c.type === t).length, parts = [];
  const m = n('motor'), j = n('joint');
  parts.push(`${m} motor${m === 1 ? '' : 's'}`); if (j) parts.push(`${j} servo${j === 1 ? '' : 's'}`);
  if (d.savedAt) parts.push(new Date(d.savedAt).toLocaleString([], { month: 'short', day: 'numeric', hour: '2-digit', minute: '2-digit' }));
  return parts.join(' · ');
};
function renderDesigns() {
  const box = document.getElementById('designList'); if (!box) return;
  box.textContent = '';
  for (const d of designs.list) {
    const open = el('button', { class: 'dname', type: 'button', title: 'Open this design', text: d.name || 'Untitled design' });
    open.addEventListener('click', () => askToSave(d.name || 'Untitled design', () => openDesign(d)));
    const exp = el('button', { class: 'icon-btn dexp', type: 'button', title: 'Save to file', 'aria-label': 'Save ' + d.name + ' to a file', text: '⤓' });
    exp.addEventListener('click', () => exportDesign(d));
    const del = el('button', { class: 'icon-btn', type: 'button', title: 'Delete', 'aria-label': 'Delete ' + d.name, text: '×' });
    del.addEventListener('click', () => {   // two clicks: the first asks
      if (del.dataset.armed) { removeDesign(d.id); return; }
      del.dataset.armed = '1'; del.textContent = 'Delete?'; del.classList.add('armed');
      setTimeout(() => { if (del.isConnected) { delete del.dataset.armed; del.textContent = '×'; del.classList.remove('armed'); } }, 3000);
    });
    box.append(el('div', { class: 'drow' + (d.id === designs.cur ? ' cur' : '') }, el('div', { class: 'dtext' }, open, el('span', { class: 'dmeta', text: designSummary(d) })), exp, del));
  }
  if (!designs.list.length) box.append(el('p', { class: 'hint', text: 'No saved designs yet.' }));
  const where = document.getElementById('designWhere');
  if (where) where.textContent = designs.where === 'account' ? 'Saved designs are kept in your account, private to you.' : 'Saved designs are kept in this browser only. Save to file for a copy you can keep anywhere.';
  // (the Layouts menu in the top bar lists these when it opens: ui.js presetMenu)
  renderDesignState();
}
function initDesigns(boot) {
  if (boot) { designs.cur = boot.cur; designs.name = boot.name; $('#designName').value = boot.name; designs.savedSnap = boot.cur && boot.clean ? designSnap() : null; }
  if (!boot || !boot.edited) designs.baseSnap = designSnap();   // the airframe on screen is as it was loaded
  $('#designSave').addEventListener('click', saveDesign);
  $('#designName').addEventListener('keydown', e => { if (e.key === 'Enter') { e.preventDefault(); saveDesign(); } });
  $('#designExport').addEventListener('click', () => exportDesign(null));
  $('#designImport').addEventListener('click', () => $('#designFile').click());
  $('#designFile').addEventListener('change', e => { const f = e.target.files[0]; e.target.value = ''; if (f) importDesign(f); });
  for (const id of ['undoBtn', 'undoBtn2']) $('#' + id).addEventListener('click', undoStep);
  for (const id of ['redoBtn', 'redoBtn2']) $('#' + id).addEventListener('click', redoStep);
  undo.stack = [designSnap()]; undo.i = 0; renderUndo();
  initDesignStore(); save();
}
