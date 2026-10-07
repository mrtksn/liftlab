'use strict';
// Computers inventory and assignments stay compact. Board/device wiring and installation
// open in detail dialogs; the dedicated formula editor retains drafts across navigation.

let lawOpen = new Set();
const lawCards = new Map(); // key -> { card, status, ta, err, applyBtn, resetBtn }

function statusText(L) { return L.status === 'error' ? 'Error' : L.status === 'edited' ? 'Edited' : 'Default'; }
function mathBlock(lines) { const m = el('div', { class: 'math' }); for (const l of lines) m.append(el('div', { html: l })); return m; }

function lawCard(key) {
  const L = LAWS[key], d = L.def, open = lawOpen.has(key);
  const status = el('span', { class: 'lst' });
  const head = UI.button( { class: 'law-head', type: 'button', 'aria-expanded': String(open), 'aria-controls': 'law-' + key, 'data-focus-key': 'lawhead-' + key },
    el('span', { class: 'law-title', text: d.title }), el('code', { class: 'law-key', text: d.key + '()' }), status);
  const body = el('div', { class: 'law-body', id: 'law-' + key }); body.hidden = !open;
  UI.bindDisclosure(head,body,{open,onToggle:o=>{o?lawOpen.add(key):lawOpen.delete(key);if(o)fitTa(ta);}});

  body.append(mathBlock(d.math), el('p', { class: 'law-doc', text: d.doc }));
  if (d.used) body.append(el('p', { class: 'law-used', text: d.used }));
  const io = el('dl', { class: 'law-io' });
  for (const [n, t] of d.args) io.append(el('dt', { text: n }), el('dd', { text: t }));
  io.append(el('dt', { class: 'ret', text: 'returns' }), el('dd', { text: d.returns }));
  body.append(io);
  const reference=UI.details({class:'formula-reference',title:'Math, inputs & documentation'},...body.children);body.replaceChildren();

  const taId = 'code-' + key;
  const ta = UI.textarea( { class: 'code', id: taId, spellcheck: 'false', autocapitalize: 'off', autocomplete: 'off', 'aria-label': d.title + ' code' });
  ta.value = L.src;
  const err = UI.status( { class: 'law-err', role: 'status' });
  const applyBtn = UI.button( { class: 'btn primary', type: 'button', text: 'Apply' });
  const resetBtn = UI.button( { class: 'btn', type: 'button', text: 'Revert to default' });
  const hint = el('span', { class: 'kbd', text: '⌘/Ctrl + Enter applies' });
  const doApply = () => {
    try { applyLaw(key, ta.value); err.textContent = ''; err.className = 'ui-status law-err'; flash(ta); }
    catch (e) { err.textContent = e.message; err.className = 'ui-status law-err on'; }
    save(); refreshLaw(key);
  };
  applyBtn.addEventListener('click', doApply);
  resetBtn.addEventListener('click', () => { resetLaw(key); ta.value = LAWS[key].src; err.textContent = ''; err.className = 'ui-status law-err'; save(); refreshLaw(key); fitTa(ta); });
  ta.addEventListener('input', () => { fitTa(ta); refreshLaw(key); });
  ta.addEventListener('keydown', e => {
    if (e.key === 'Enter' && (e.metaKey || e.ctrlKey)) { e.preventDefault(); doApply(); return; }
    if (e.key === 'Tab' && !e.shiftKey) { e.preventDefault(); const s = ta.selectionStart, en = ta.selectionEnd; ta.setRangeText('  ', s, en, 'end'); fitTa(ta); refreshLaw(key); }
  });
  body.append(ta, el('div', { class: 'law-actions' }, applyBtn, resetBtn, hint), err);
  // Flight formulas: what the step runner makes of them.
  let rn = null;
  if (typeof RN_SIGS !== 'undefined' && RN_SIGS[key]) {
    const note = UI.status( { class: 'law-rn', role: 'status' });
    const steps = UI.details({ class: 'steps', title: 'Compiled steps' });
    const sum = steps.querySelector('summary'), pre = el('pre', { class: 'listing' });
    steps.append(pre);
    steps.addEventListener('toggle', () => { if (steps.open) pre.textContent = RN.P && RN.P.fns[key] ? rnListing(RN.P, key) : ''; });
    body.append(note, steps);
    rn = { note, steps, sum, pre };
  }
  body.append(reference);
  const card = UI.card( { class: 'law', 'data-law': key }, head, body);
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

  r.note.textContent = msg; r.note.className = 'ui-status law-rn' + (tone ? ' ' + tone : ''); r.note.hidden = !msg;
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
  const P = RN.P, st = $('#rnStatus'), metrics = $('#rnMetrics');
  if (!P) { setText(st, 'Compilation failed: ' + RN.buildErr); metrics.hidden = true; }
  else {
    const n = Object.keys(P.fns).length, errors = Object.keys(P.errors).length, kb = x => (x * 4 / 1024).toFixed(0) + ' KB';
    setText(st, `${n} formulas compiled` + (errors ? ` · ${errors} failed` : ''));
    metrics.hidden = false; syncKv(metrics, [['Program', kb(P.code.length + P.constEnd)], ['Working memory', kb(P.arenaSize)], ['Edited formulas', String(editedLaws().length)]]);
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
  if (L.status === 'error' && L.err && !dirty) { c.err.textContent = L.err; c.err.className = 'ui-status law-err on'; }
  const chip = document.querySelector(`.chain [data-go="${key}"]`); if (chip) chip.className = 'ui-button lchip ' + L.status;
  refreshFormulaStatus();
  refreshLawRunner(key);
}
// A formula's card after its code changed from elsewhere (a design opened, an undo, the AI agent).
function refreshLawCard(key) { const c = lawCards.get(key); if (c) { c.ta.value = LAWS[key].src; fitTa(c.ta); refreshLaw(key); } }
function refreshFormulaStatus() {
  const ed = editedLaws(), bad = ed.filter(L => L.status === 'error').length;
  const badge = $('#editedCount'); badge.textContent = ed.length ? String(ed.length) : ''; badge.hidden = !ed.length; badge.className = 'count' + (bad ? ' bad' : '');
  $('#copyEdited').disabled = !ed.length; $('#revertAll').disabled = !ed.length;
}
lawListeners.add(key => {
  const c = lawCards.get(key);
  if (c && LAWS[key].status === 'error') { c.err.textContent = LAWS[key].err; c.err.className = 'ui-status law-err on'; }
  refreshLaw(key); save();
});

function goToLaw(key) {
  openFormulaEditor(key);
  const c = lawCards.get(key); if (!c) return;
  const body = c.card.querySelector('.law-body');
  if (body.hidden) c.card.querySelector('.law-head').click();
  c.card.scrollIntoView({ block: 'start', behavior: window.matchMedia('(prefers-reduced-motion: reduce)').matches ? 'auto' : 'smooth' });
}

/* ───────── the tab ───────── */
const taskOfLaw = k => Object.keys(TASKS).find(t => TASKS[t].formulas.includes(k)) || null;
const COMP = { built: false };
function lawSection(id, title, blurb, keys, small) {
  const sec = UI.section( { class: 'sec', id }, el('h2', {}, document.createTextNode(title + ' '), el('small', { text: small || '' })), el('p', { class: 'hint', text: blurb }));
  const list = el('div', { class: 'laws' }); for (const k of keys) list.append(lawCards.has(k) ? lawCards.get(k).card : lawCard(k));
  sec.append(list); return sec;
}
function computerDialog(id, title) {
  const body=el('div',{class:'computer-detail',id:id+'Body'}),close=UI.button({class:'btn',text:'Close',onclick:()=>$('#'+id).close()});
  const dialog=el('dialog',{class:'ask computer-dialog',id,'aria-labelledby':id+'Title'},el('div',{class:'dialog-toolbar'},el('h2',{id:id+'Title',text:title}),el('div',{class:'dialog-actions'},UI.button({class:'btn sm',id:id+'Undo',text:'↶ Undo','data-design-history':'undo'}),UI.button({class:'btn sm',id:id+'Redo',text:'↷ Redo','data-design-history':'redo'}),close)),body);
  return {dialog,body};
}
function computerToolButton(id,text,svg,onclick){
  return UI.button({class:'btn computer-tool',id,onclick,'aria-haspopup':'dialog'},el('span',{'aria-hidden':'true',html:'<svg viewBox="0 0 20 20" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round">'+svg+'</svg>'}),el('span',{text}));
}
function formulaGroups(){
  return [...Object.entries(TASKS).map(([,T])=>({label:T.label,keys:T.formulas})),{label:GROUND.label,keys:GROUND.formulas},{label:'World & sensor models',keys:LAW_DEFS.filter(d=>['plant','sensor'].includes(d.group)).map(d=>d.key)}];
}
function renderFormulaChoices(){
  const select=$('#formulaSelect');if(!select)return;
  const tokens=$('#formulaSearch').value.trim().toLocaleLowerCase().split(/\s+/).filter(Boolean),groups=formulaGroups();
  const matches=text=>tokens.every(t=>text.toLocaleLowerCase().includes(t));
  let results=groups;
  if(tokens.length){
    const title=[],content=[];
    for(const key of groups.flatMap(g=>g.keys)){
      const L=LAWS[key],d=L.def;
      if(matches(d.title))title.push(key);
      else if(matches([d.title,key,lawCards.get(key)?.ta.value||L.src,d.doc,d.used,d.returns,...d.args.flat()].join(' ')))content.push(key);
    }
    results=[{label:'Title matches',keys:title},{label:'Content matches',keys:content}];
  }
  const keys=results.flatMap(g=>g.keys),current=COMP.formula||select.value||keys[0];select.replaceChildren();
  if(!keys.includes(current))select.append(el('option',{value:'',text:keys.length?'Choose a matching formula…':'No matching formulas',disabled:'disabled'}));
  for(const group of results)if(group.keys.length)select.append(el('optgroup',{label:group.label},...group.keys.map(key=>el('option',{value:key,text:LAWS[key].def.title}))));
  select.value=keys.includes(current)?current:'';select.disabled=!keys.length;
  setText($('#formulaSearchStatus'),tokens.length?`${keys.length} ${keys.length===1?'match':'matches'} · titles first`:'Search titles, code and documentation.');
}
function buildComputers() {
  const pane=$('#paneForm');pane.textContent='';COMP.view=null;COMP.confirm=null;
  for(const d of LAW_DEFS)if(!lawCards.has(d.key))lawCard(d.key);
  const section=(id,title,content)=>UI.section({class:'sec',id},el('h2',{text:title}),content);
  pane.append(section('computerBoardsSec','Boards',el('div',{},UI.button({class:'btn',id:'boardAdd',text:'+ Add a board',onclick:()=>$('#boardAddDlg').showModal()}),el('div',{class:'computer-grid',id:'boardList'}))),
    section('computerAssignmentsSec','Assignments',el('div',{class:'computer-grid',id:'taskRows'})),
    section('computerOutputsSec','Motors & servos',el('div',{class:'computer-grid',id:'computerOutputs'})),
    section('computerSensorsSec','Sensors',el('div',{class:'computer-grid',id:'computerSensors'})),
    section('computerCargoSec','Cargo outputs',el('div',{class:'computer-grid',id:'computerCargo'})),
    section('computerRadioSec','Radio',el('div',{class:'computer-grid',id:'computerRadio'})),
    section('computerToolsSec','Tools',el('div',{class:'computer-tools'},computerToolButton('formulasOpen','Formula editor…','<path d="M7 4L2 10l5 6m6-12 5 6-5 6M11 3l-2 14"/>',()=>openFormulaEditor()),computerToolButton('wiringOpen','Wiring overview…','<rect x="2" y="2" width="5" height="5" rx="1"/><rect x="13" y="13" width="5" height="5" rx="1"/><path d="M4.5 7v8.5H13M7 4.5h8.5V13"/>',()=>openComputerView({kind:'wiring'})))));
  pane.prepend($('#computerToolsSec'));
  pane.append(el('div',{id:'hardwareRows',hidden:true}));
  const detail=computerDialog('computerDlg','Computer details');pane.append(detail.dialog);
  detail.dialog.addEventListener('close',()=>{COMP.view=null;COMP.confirm=null;if(COMP.returnFocus)document.querySelector(COMP.returnFocus)?.focus({preventScroll:true});});
  const add=computerDialog('boardAddDlg','Add a board');
  for(const mcu of [true,false]){
    const group=el('section',{class:'computer-add-group'},el('h3',{text:mcu?'Microcontrollers':'Linux computers'}));
    for(const [kind,K]of Object.entries(BOARD_KINDS).filter(([,K])=>!K.groundOnly&&K.mcu===mcu))group.append(UI.button({class:'board-choice', 'data-board-kind':kind},el('b',{text:K.label}),el('span',{text:K.note})));
    add.body.append(group);
  }
  add.body.querySelectorAll('[data-board-kind]').forEach(btn=>btn.addEventListener('click',()=>{
    const C=JSON.parse(JSON.stringify(computers()));if(C.boards.length>=BOARD_MAX)return;
    const kind=btn.dataset.boardKind;C.boards.push({id:C.nextBoardId||Math.max(0,...C.boards.map(b=>b.id))+1,kind,name:BOARD_KINDS[kind].label,tasks:[]});
    add.dialog.close();setComputers(C,'add');
  }));pane.append(add.dialog);
  const editor=computerDialog('formulaDlg','Formula editor'),formulaDialog=editor.body;
  editor.dialog.classList.add('formula-dialog');
  const select=UI.select({id:'formulaSelect','aria-label':'Formula'});
  const search=UI.input({type:'search',id:'formulaSearch',placeholder:'Title or code…',autocomplete:'off','aria-label':'Search formulas','aria-controls':'formulaSelect','aria-describedby':'formulaSearchStatus'});
  search.addEventListener('input',()=>renderFormulaChoices());
  search.addEventListener('keydown',e=>{if(e.key==='Enter'){e.preventDefault();const first=select.querySelector('option:not([disabled])');if(first){showFormula(first.value);select.focus();}}});
  select.addEventListener('change',()=>{if(select.value)showFormula(select.value);});
  formulaDialog.append(el('div',{class:'formula-finder'},hardwareField('Search formulas',search),hardwareField('Formula',select)),UI.status({class:'hint formula-search-status',id:'formulaSearchStatus',role:'status'}),el('h3',{id:'formulaTitle'}),el('p',{class:'hint',id:'formulaOwner'}),el('div',{id:'formulaActive'}));
  renderFormulaChoices();
  const store=el('div',{id:'formulaStore',hidden:true},el('div',{id:'taskLaws'}));
  store.append(lawSection('worldLaws','The world','Physics and sensor models used by the simulator.',LAW_DEFS.filter(d=>['plant','sensor'].includes(d.group)).map(d=>d.key),'simulator only'));pane.append(store);
  formulaDialog.append(UI.details({class:'sec',id:'rnBox',title:'Program status & exports'},
      UI.status( { class: 'rn-status', id: 'rnStatus', role: 'status' }),
      el('dl',{class:'program-metrics',id:'rnMetrics'}),
      UI.details({title:'Recent activity',class:'program-activity'},el('ol', { class: 'rn-log', id: 'rnLog', hidden: true })),
      el('ul', { class: 'rn-problems', id: 'rnProblems' }),
      el('div', { class: 'law-actions' }, UI.button( { class: 'btn', type: 'button', id: 'rnDownload', text: 'Full program (.rnp)', onclick: rnDownload })),el('p',{class:'hint',text:'Totals include all duties. Per-board files are in Install / export.'}),UI.details({class:'program-help',title:'How flight updates work'},el('p',{class:'hint',text:'Applied edits are compiled and checked before activation. During flight, the new program is tested beside the current one, then blended in. If it fails, the previous version resumes.'}))));
  const copyBtn = UI.button( { class: 'btn', type: 'button', id: 'copyEdited', text: 'Copy edited formulas' });
  const revertAll = UI.button( { class: 'btn', type: 'button', id: 'revertAll', text: 'Revert all' });
  const copyOut = UI.textarea( { class: 'code', id: 'copyOut', readonly: 'readonly', 'aria-label': 'Edited formulas' }); copyOut.hidden = true;
  const copyMsg = el('span', { class: 'kbd', role: 'status' });
  copyBtn.addEventListener('click', () => {
    const text = '// Edited formulas from LiftLab. Paste over the matching functions in js/laws.js.\n\n' + editedLaws().map(L => L.src.trim()).join('\n\n') + '\n';
    const fallback = () => { copyOut.hidden = false; copyOut.value = text; fitTa(copyOut); copyOut.focus(); copyOut.select(); copyMsg.textContent = 'Select and copy the text below.'; };
    try { navigator.clipboard.writeText(text).then(() => { copyOut.hidden = true; copyMsg.textContent = 'Copied.'; }, fallback); } catch (e) { fallback(); }
  });
  revertAll.addEventListener('click', () => {
    if (revertAll.dataset.armed !== '1') { revertAll.dataset.armed = '1'; revertAll.textContent = 'Click again to revert all'; setTimeout(() => { revertAll.dataset.armed = ''; revertAll.textContent = 'Revert all'; }, 3000); return; }
    revertAll.dataset.armed = ''; revertAll.textContent = 'Revert all';
    for (const L of editedLaws()) { resetLaw(L.def.key); const c = lawCards.get(L.def.key); c.ta.value = LAWS[L.def.key].src; c.err.textContent = ''; c.err.className = 'ui-status law-err'; fitTa(c.ta); refreshLaw(L.def.key); }
    save();
  });
  const tools = UI.section( { class: 'sec' }, el('h2', { text: 'Your edits' }),
    el('p', { class: 'hint', text: 'Apply changes to the selected formula to use them in flight. The math reference describes its default form.' }),
    el('div', { class: 'law-actions' }, copyBtn, revertAll, copyMsg), copyOut);
  if (!evalAllowed) tools.append(el('p', { class: 'note', text: 'This viewer blocks running edited JavaScript. The flight formulas still take edits: they are compiled for the boards. The world\'s formulas can be read here; open index.html from the repo to edit them.' }));
  formulaDialog.append(tools, UI.details({class:'sec formula-help',title:'Available inside formulas'},
    el('p', { class: 'hint', html: 'Vectors are <code>[x, y, z]</code> arrays; 3×3 matrices are 9 numbers, row by row. Helpers: <code>add sub scl dot crs nrm unit clamp</code>, <code>m3v m3m m3T m3inv</code>, <code>qmat qmul qnorm matToQuat</code>, <code>randn</code> (standard normal, seeded so each reset replays the same noise), <code>bls(cols, lo, hi, w, W)</code>, and constants <code>G D2R R2D</code>. Anything in <code>Math</code> works too. A formula that throws or returns a wrong shape is switched off and the default takes over.' })));
  pane.append(editor.dialog);
  COMP.built=true;applySrcTags(pane);renderComputers(true);for(const d of LAW_DEFS)refreshLaw(d.key);
}

const pct = x => x < 0.01 ? '<1%' : Math.round(x * 100) + '%';
// A name typed in: saved as it is, without resetting the flight (nothing that flies depends on it).
function renameComputer(apply) {
  apply(computers()); undoKey = 'computers:name'; save();
  renderComputers(true); if (typeof renderHealth === 'function') renderHealth(true);
}
// A name you can type in, with a pencil to say so.
function nameBox(value, label, id, onName) {
  const i = UI.input( { type: 'text', class: 'board-name', value, maxlength: 24, 'aria-label': label, id, title: 'Click to rename', autocomplete: 'off', spellcheck: 'false' });
  i.addEventListener('change', () => onName(i.value.trim()));
  i.addEventListener('keydown', e => { if (e.key === 'Enter') i.blur(); else if (e.key === 'Escape') { i.value = value; i.blur(); } });
  const pen = document.createElementNS('http://www.w3.org/2000/svg', 'svg'); pen.setAttribute('viewBox', '0 0 16 16'); pen.setAttribute('aria-hidden', 'true');
  const pth = document.createElementNS('http://www.w3.org/2000/svg', 'path'); pth.setAttribute('d', 'M10.5 2.5l3 3L6 13H3v-3zM9 4l3 3'); pth.setAttribute('stroke-linejoin', 'round'); pen.append(pth);
  return el('span', { class: 'name-edit' }, i, pen);
}
function renderComputers(full) { keepFocus(() => renderComputers1(full)); }
function renderComputers1(full) {
  if(!COMP.built)return;
  const C=computers(),sig=JSON.stringify(C)+'|'+cfg.comps.map(c=>[c.id,c.type,c.kind,c.name,c.mount]).join(';')+'|'+JSON.stringify([radioCfg,radioCfg2]);
  if(full||COMP.sig!==sig){
    COMP.sig=sig;COMP.rendering=true;
    try{
      $('#boardList').replaceChildren(...C.boards.map(b=>computerBoardCard(b)),computerBoardCard(C.ground,true));
      $('#boardAdd').disabled=C.boards.length>=BOARD_MAX;
      $('#taskRows').replaceChildren(...Object.entries(TASKS).map(([id,T])=>assignmentCard(T.label,boardOf(id),T.what,()=>openComputerView({kind:'task',id}),{'data-task':id})));
      for(const [id,filter]of [['computerOutputs',c=>['motor','joint'].includes(c.type)],['computerSensors',c=>c.type==='sensor'],['computerCargo',c=>c.type==='latch']]){
        const parts=cfg.comps.filter(filter);$('#'+id).replaceChildren(...parts.map(c=>assignmentCard(c.name,hardwareOwner(C,c),deviceDescription(c),()=>openComputerView({kind:'device',id:c.id}),{'data-device':c.id})));
        $('#'+id).closest('.sec').hidden=!parts.length;
      }
      $('#computerRadio').replaceChildren(assignmentCard(RADIO_LINKS[radioCfg.kind]?.label||'Radio',boardOf('tlm'),'The pilot link and telemetry use the Telemetry & radio board.',()=>openComputerView({kind:'radio'}),{'data-radio':'primary'}));
      if(radioTwo())$('#computerRadio').append(assignmentCard('Second link · '+(RADIO_LINKS[radioCfg2.kind]?.label||radioCfg2.kind),boardOf('tlm'),'Both links use the same Telemetry & radio board.',()=>openComputerView({kind:'radio'}),{'data-radio':'secondary'}));
      renderHardware();
      const box=$('#taskLaws');box.textContent='';
      for(const [t,T]of Object.entries(TASKS))box.append(lawSection('laws-'+t,T.label,T.what,T.formulas,boardOf(t)?.name||'unassigned'));
      box.append(lawSection('laws-ground',GROUND.label,GROUND.what,GROUND.formulas,C.ground.name));
      if($('#formulaDlg').open)showFormula(COMP.formula||$('#formulaSelect').value);
      if($('#computerDlg').open)renderComputerDetail();
    }finally{COMP.rendering=false;}
  }
  renderRunner();
}
function interactiveCard(attrs,open,...children){
  const card=UI.card({class:'computer-card',role:'button',tabindex:0,...attrs},...children);
  card.addEventListener('click',e=>{if(!e.target.closest('button,input,select,a'))open();});
  card.addEventListener('keydown',e=>{if(e.target===card&&['Enter',' '].includes(e.key)){e.preventDefault();open();}});return card;
}
function computerBoardCard(b,ground=false){
  const key=ground?'ground':b.id,K=BOARD_KINDS[b.kind];
  return interactiveCard({'data-board':key,'aria-label':b.name+' details'},()=>openComputerView({kind:'board',id:key}),
    el('div',{class:'computer-card-heading'},el('b',{text:b.name}),el('span',{class:'computer-location',text:ground?'Ground':'Onboard'})),
    el('span',{class:'computer-kind',text:K.label}),el('p',{class:'computer-meta',text:K.note}),
    UI.button({class:'btn sm',text:'Install / export…',id:ground?'ginst':'binst-'+b.id,onclick:e=>{e.stopPropagation();openInstall(ground?'ground':b);}}));
}
function assignmentCard(name,board,description,open,attrs={}){
  return interactiveCard({...attrs,class:'computer-card assignment-card'+(!board?' empty':'')},open,
    el('b',{text:name}),board?el('div',{class:'assigned-board'},el('b',{text:board.name}),el('span',{text:BOARD_KINDS[board.kind].label})):el('p',{class:'computer-meta',text:description}),
    ...(!board?[UI.button({class:'btn sm',text:'Assign',onclick:e=>{e.stopPropagation();open();}})]:[]));
}
function deviceDescription(c){return c.type==='sensor'?(SENSOR_KINDS[c.kind]+' sensor: choose the board that receives its measurements.'):c.type==='joint'?'Servo: choose the board that controls its position.':c.type==='latch'?'Cargo latch: choose the board that opens and closes it.':'Motor: choose the board that sends its throttle signal.';}
function openComputerView(view){
  const dlg=$('#computerDlg');
  if(!dlg.open){
    const active=document.activeElement,card=active.closest('[data-board],[data-task],[data-device],[data-radio]');
    const key=card&&['board','task','device','radio'].find(k=>Object.hasOwn(card.dataset,k));
    COMP.returnFocus=key?'#paneForm [data-'+key+'="'+CSS.escape(card.dataset[key])+'"]':active.id?'#'+CSS.escape(active.id):null;
  }
  COMP.view=view;COMP.confirm=null;renderHardware();renderComputerDetail();
  if(!dlg.open)dlg.showModal();dlg.scrollTop=0;
}
function eligibleTaskBoards(task){const T=TASKS[task];return computers().boards.filter(b=>(!T.mcuOnly||BOARD_KINDS[b.kind].mcu)&&(!T.piOnly||!BOARD_KINDS[b.kind].mcu));}
function assignDuty(task,id){
  const C=JSON.parse(JSON.stringify(computers()));for(const b of C.boards)b.tasks=b.tasks.filter(t=>t!==task);
  if(id!=null){const b=C.boards.find(b=>b.id===id);if(!b||!eligibleTaskBoards(task).some(x=>x.id===id))return;b.tasks.push(task);}
  if(task==='core')C.unassignedCore=id==null;setComputers(C,'task');
}
function assignmentChoices(box,current,boards,apply){
  box.append(el('h3',{text:current?'Assigned board':'Choose a board'}));
  for(const b of boards)box.append(UI.card({class:'board-assignment'},el('div',{},el('b',{text:b.name}),el('span',{class:'computer-meta',text:BOARD_KINDS[b.kind].note})),UI.button({class:'btn sm',text:current?.id===b.id?'Assigned':'Assign',disabled:current?.id===b.id?'disabled':null, 'data-assign-board':b.id,onclick:()=>apply(b.id)})));
  if(!boards.length)box.append(el('p',{class:'hint',text:'No compatible board yet. Add a board or assign the required duty first.'}));
  if(current)box.append(UI.button({class:'btn sm',text:'Remove assignment','data-remove-assignment':'',onclick:()=>apply(null)}));
}
function mountHardware(box,selector){for(const n of [...$('#hardwareRows').querySelectorAll(selector)])box.append(n);}
function computerWiringIssues(box,b){
  if(!b)return;const plan=boardWiringPlan(b);
  for(const msg of plan.errors)box.append(el('p',{class:'bad',text:msg}));
  for(const msg of plan.warnings)box.append(el('p',{class:'hint',text:msg}));
}
function renderComputerDetail(){
  const view=COMP.view,box=$('#computerDlgBody');if(!view||!box)return;box.textContent='';box.classList.toggle('device-detail',view.kind==='device');const C=computers();
  renderUndo();
  if(view.kind==='wiring'){$('#computerDlgTitle').textContent='Wiring overview';mountHardware(box,'[data-hw-role="ground"]');box.append(renderWiringOverview(C));mountHardware(box,'#hardwareReport');return;}
  if(view.kind==='task'||view.kind==='radio'){
    const t=view.kind==='radio'?'tlm':view.id,T=TASKS[t];$('#computerDlgTitle').textContent=view.kind==='radio'?'Radio assignment & wiring':T.label;
    box.append(el('p',{text:T.what}));assignmentChoices(box,boardOf(t),eligibleTaskBoards(t),id=>assignDuty(t,id));
    if(view.kind==='radio'){mountHardware(box,'[data-hw-role="radio"]');computerWiringIssues(box,boardOf(t));}
    else if(T.formulas.length)box.append(UI.button({class:'btn',text:'Open formulas…',onclick:()=>{$('#computerDlg').close();openFormulaEditor(T.formulas[0]);}}));
    return;
  }
  if(view.kind==='device'){
    const c=compById(view.id);if(!c){$('#computerDlg').close();return;}$('#computerDlgTitle').textContent=c.name;
    box.append(el('p',{text:deviceDescription(c)}));const current=hardwareOwner(C,c),task=c.type==='latch'?'cargo':c.type==='sensor'&&['fix','flow'].includes(c.kind)?'nav':'core';
    const boards=eligibleTaskBoards(task);assignmentChoices(box,current,boards,id=>editWiring(w=>{const same=hardwareOwner(computers(),c)?.id===id;w.parts[c.id]={...w.parts[c.id],board:id,...(c.type==='sensor'?{}:{pin:id==null?-1:same?partWiring(c).pin:-1})};},c.id));
    if(Object.hasOwn(C.wiring?.parts?.[c.id]||{},'board'))box.append(UI.button({class:'btn sm',text:'Follow '+TASKS[task].label+' automatically',onclick:()=>editWiring(w=>{const p={...w.parts[c.id]};delete p.board;delete p.pin;w.parts[c.id]=p;},c.id)}));
    else box.append(el('p',{class:'hint',text:'Follows the '+TASKS[task].label+' board automatically.'}));
    if(current&&['motor','joint'].includes(c.type)&&current.id!==boardOf('core')?.id)box.append(el('p',{class:'hint',text:'Assign Flight core to this board to drive this output. Distributed flight outputs are not supported.'}));
    mountHardware(box,'[data-hw-part="'+c.id+'"]');computerWiringIssues(box,current);return;
  }
  box.classList.remove('device-detail');
  const ground=view.id==='ground',b=ground?C.ground:C.boards.find(b=>b.id===view.id);if(!b){$('#computerDlg').close();return;}
  const K=BOARD_KINDS[b.kind],budget=ground?groundBudget():boardBudget(b);$('#computerDlgTitle').textContent=b.name;
  box.append(nameBox(b.name,'Board name',ground?'gname':'bname-'+b.id,v=>renameComputer(D=>{const x=ground?D.ground:D.boards.find(x=>x.id===b.id);if(x)x.name=(v||K.label).slice(0,24);})),
    hardwareField('Computer',hardwareSelect('Board kind',ground?'gkind':'bkind-'+b.id,Object.entries(BOARD_KINDS).filter(([,K])=>ground||!K.groundOnly).map(([id,K])=>[id,K.label]),b.kind,v=>{const D=JSON.parse(JSON.stringify(computers()));(ground?D.ground:D.boards.find(x=>x.id===b.id)).kind=v;setComputers(D,'kind');})),
    el('p',{text:K.note}),el('p',{class:'hint',text:ground?'Command module':b.tasks.map(t=>TASKS[t].label+' · '+boardTaskHz(b,t)+' Hz').join(' / ')||'No duties assigned'}),
    el('p',{class:budget.load>0.8?'bad':'hint',text:'Estimated load '+pct(budget.load)+' of '+(K.cores>1?'one core':'its core')+(K.mcu?' · program '+budget.memKB.toFixed(1)+' KB of '+K.ramKB+' KB':'')+(budget.load>1?' · over capacity':'')}),
    UI.button({class:'btn primary',text:'Install / export…',onclick:()=>{$('#computerDlg').close();openInstall(ground?'ground':b);}}));
  const only=!ground&&K.mcu&&C.boards.filter(x=>BOARD_KINDS[x.kind].mcu).length===1;
  const blocked=ground?'The ground command module is part of the pilot link.':C.boards.length<2?'Keep at least one onboard board.':only?'Add another microcontroller before deleting the last one.':'';
  const del=UI.button({class:'btn',id:ground?'gdel':'bdel-'+b.id,text:'Delete board',disabled:blocked?'disabled':null,title:blocked,onclick:()=>{COMP.confirm=b.id;renderHardware();renderComputerDetail();}});box.append(del);
  if(blocked)box.append(el('p',{class:'hint',text:blocked}));
  if(COMP.confirm===b.id&&!ground){
    box.append(el('div',{class:'computer-confirm',role:'alert'},el('p',{text:'Delete “'+b.name+'”? Its duties and device connections will be removed.'}),UI.button({class:'btn',text:'Cancel',onclick:()=>{COMP.confirm=null;renderHardware();renderComputerDetail();}}),UI.button({class:'btn primary',id:'boardDeleteConfirm',text:'Delete board',onclick:()=>{
      const D=JSON.parse(JSON.stringify(computers()));D.boards=D.boards.filter(x=>x.id!==b.id);if(b.tasks.includes('core'))D.unassignedCore=true;
      if(D.wiring){delete D.wiring.boards?.[b.id];for(const p of Object.values(D.wiring.parts||{}))if(p.board===b.id){p.board=null;p.pin=-1;}}
      $('#computerDlg').close();setComputers(D,'remove');
    }})));
  }
  box.append(el('h3',{text:ground?'Command wiring':'Wiring & devices'}));
  if(ground){mountHardware(box,'[data-hw-role="ground"]');const group=hardwareOverview(C,cfg.comps).groups.find(g=>g.id==='ground');if(!groundHardware(C))box.append(el('p',{class:'hint',text:group?.rows.map(r=>r.connection+' · '+r.note).join(' ')||'Host inputs and USB serial.'}));}
  else{
    for(const c of cfg.comps.filter(c=>hardwareOwner(C,c)?.id===b.id))mountHardware(box,'[data-hw-part="'+c.id+'"]');
    mountHardware(box,'[data-hw-board="'+b.id+'"]');
    if(boardOf('tlm')?.id===b.id)mountHardware(box,'[data-hw-role="radio"]');
    computerWiringIssues(box,b);
  }
}
function openFormulaEditor(key){const dlg=$('#formulaDlg');if(key)$('#formulaSearch').value='';renderUndo();if(!dlg.open)dlg.showModal();showFormula(key||COMP.formula||formulaGroups()[0].keys[0]);}
function showFormula(key){
  const c=lawCards.get(key);if(!c)return;COMP.formula=key;renderFormulaChoices();setText($('#formulaTitle'),LAWS[key].def.title);
  for(const old of [...$('#formulaActive').children])$('#formulaStore').append(old);$('#formulaActive').append(c.card);
  const task=taskOfLaw(key),board=task&&boardOf(task);$('#formulaOwner').textContent=task?(TASKS[task].label+' · '+(board?board.name:'not assigned')):GROUND.formulas.includes(key)?'Command module · '+computers().ground.name:'Simulator world model';
  c.card.querySelector('.law-body').hidden=false;c.card.querySelector('.law-head').setAttribute('aria-expanded','true');lawOpen.add(key);fitTa(c.ta);refreshLaw(key);
}
// An export that failed says why under its button (kept across a re-render of the cards).
const errLine = key => { const e = COMP.exErr && String(COMP.exErr.key) === String(key) ? COMP.exErr.msg : ''; return UI.status( { class: 'law-err board-err' + (e ? ' on' : ''), role: 'status', text: e }); };
function exportErr(btn, msg) {
  const card = btn && btn.closest('.board'), p = card && card.querySelector('.board-err');
  if (!p) { if (msg) { rnEvent(msg, 'bad'); renderRunner(); } return; }
  COMP.exErr = msg ? { key: card.dataset.board, msg } : null;
  p.textContent = msg; p.classList.toggle('on', !!msg);
}
function groundDownload(btn) {
  let img; try { img = boardImage({ tasks: ['ground'] }); } catch (e) { exportErr(btn, 'Can\'t make the command module\'s program: ' + e.message); return; }
  exportErr(btn, '');
  const a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([img], { type: 'application/octet-stream' })); a.download = 'command-module.rnp';
  document.body.append(a); a.click(); setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
}
// Files for the drone: the airframe for the flight core, the config for the navigation.
function boardsExport(what, btn) {
  let data, ext;
  try { if (what === 'airframe') { data = fcAirframeBlob(); ext = '.dfa'; } else if (what === 'pi') { data = piConfigBlob(); ext = '.dlc'; } else { data = navConfigBlob(); ext = '.dnc'; } }
  catch (e) { exportErr(btn, 'Can\'t export: ' + e.message); return; }
  exportErr(btn, '');
  const nm = ((typeof designs !== 'undefined' && designs.name) || 'drone').replace(/[^\w.-]+/g, '-');
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([data], { type: 'application/octet-stream' }));
  a.download = nm + ext; document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}
