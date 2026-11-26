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
  const progs=typeof programs==='function'?programs().filter(p=>!tokens.length||matches(p.name+' '+p.writes.topic+' '+p.src)):[];   // your programs (programs.js), after the formulas
  const draft=String(COMP.formula||'').startsWith('prog:')&&!progs.some(p=>'prog:'+p.id===COMP.formula)&&progCards.get(COMP.formula.slice(5));
  const keys=[...results.flatMap(g=>g.keys),...progs.map(p=>'prog:'+p.id),...(draft?[COMP.formula]:[])],current=COMP.formula||select.value||keys[0];select.replaceChildren();
  if(!keys.includes(current))select.append(el('option',{value:'',text:keys.length?'Choose a matching formula…':'No matching formulas',disabled:'disabled'}));
  for(const group of results)if(group.keys.length)select.append(el('optgroup',{label:group.label},...group.keys.map(key=>el('option',{value:key,text:LAWS[key].def.title}))));
  if(progs.length||draft)select.append(el('optgroup',{label:'Your programs'},...progs.map(p=>el('option',{value:'prog:'+p.id,text:p.name+' · '+((computers().boards.find(b=>b.id===p.board)||{}).name||'not assigned')})),...(draft?[el('option',{value:COMP.formula,text:draft.d.name+' (not added yet)'})]:[])));
  select.value=keys.includes(current)?current:'';select.disabled=!keys.length;
  setText($('#formulaSearchStatus'),tokens.length?`${keys.length} ${keys.length===1?'match':'matches'} · titles first`:'Search titles, code and documentation.');
}
function buildComputers() {
  const pane=$('#paneForm');pane.textContent='';COMP.view=null;COMP.confirm=null;
  for(const d of LAW_DEFS)if(!lawCards.has(d.key))lawCard(d.key);
  const section=(id,title,hint,...content)=>UI.section({class:'sec',id},el('h2',{},document.createTextNode(title+' '),el('small',{id:id+'Count'})),...(hint?[el('p',{class:'hint',text:hint})]:[]),...content);
  const list=id=>el('div',{class:'complist clist',id});
  const addMenu=menuButton({text:'+ Add',key:'compAdd',align:'left',title:'Add a board, a power source, a sensor or another electronic part',items:compAddItems,onPick:compAddPick});addMenu.node.id='compAddMenu';
  const viewSeg=el('div',{class:'seg',role:'group','aria-label':'Show',id:'compViewSeg'},
    UI.button({type:'button','data-comp-view':'parts','aria-pressed':'true',text:'Parts',title:'The electronics as parts: pick one to close in on it',onclick:()=>{if(COMP.focus&&COMP.focus.kind==='wiring')compFocus(null);else compFocus(COMP.focus);}}),
    UI.button({type:'button','data-comp-view':'wiring','aria-pressed':'false',text:'Wiring',title:'Every wire, from the pin it leaves to the part it reaches',onclick:()=>compFocus({kind:'wiring'})}));
  const inspect=UI.button({class:'btn sm',id:'benchInspect',type:'button','aria-pressed':'false',text:'Inspect in 3D',title:'Show the electronics pulled apart in the view, or go back to watching the flight',onclick:()=>{bench.want=!bench.on;benchSync();renderComputers(true);}});
  pane.append(section('computerToolsSec','Tools','',el('div',{class:'computer-tools'},computerToolButton('formulasOpen','Formula editor…','<path d="M7 4L2 10l5 6m6-12 5 6-5 6M11 3l-2 14"/>',()=>openFormulaEditor()),computerToolButton('appsOpen','App manager…','<rect x="2.5" y="2.5" width="6" height="6" rx="1"/><rect x="11.5" y="2.5" width="6" height="6" rx="1"/><rect x="2.5" y="11.5" width="6" height="6" rx="1"/><path d="M14.5 11.5v6M11.5 14.5h6"/>',()=>openAppManager()),computerToolButton('busOpen','Live data…','<path d="M2 15h3l3-9 4 12 3-7h3"/>',()=>openComputerView({kind:'bus'}))),
      el('div',{class:'computer-tools comp-viewbar'},addMenu.node,viewSeg,inspect)),
    section('computerBoardsSec','Boards','',list('boardList')),
    section('computerPowerSec','Power','',list('computerPower')),
    section('computerSensorsSec','Sensors','',list('computerSensors')),
    section('computerElecSec','Other electronics','',list('computerElec')),
    section('computerWiringSec','Wiring','',list('computerWiring'),list('computerLinks'),list('computerRadio')),
    section('computerOutputsSec','Motors & servos','',list('computerOutputs'),list('computerCargo')),
    section('computerAssignmentsSec','Duties','',list('taskRows')));
  COMP.detail=el('div',{class:'cdetail',id:'compDetail'},el('h3',{class:'sr-only',id:'compDetailTitle'}),el('div',{class:'computer-detail',id:'compDetailBody'}));
  pane.append(el('div',{id:'hardwareRows',hidden:true}));
  const detail=computerDialog('computerDlg','Computer details');pane.append(detail.dialog);
  detail.dialog.addEventListener('close',()=>{COMP.view=null;COMP.confirm=null;if(COMP.returnFocus)document.querySelector(COMP.returnFocus)?.focus({preventScroll:true});});
  buildAppManager(pane);
  const editor=computerDialog('formulaDlg','Formula editor'),formulaDialog=editor.body;
  editor.dialog.classList.add('formula-dialog');
  const select=UI.select({id:'formulaSelect','aria-label':'Formula'});
  const search=UI.input({type:'search',id:'formulaSearch',placeholder:'Title or code…',autocomplete:'off','aria-label':'Search formulas','aria-controls':'formulaSelect','aria-describedby':'formulaSearchStatus'});
  search.addEventListener('input',()=>renderFormulaChoices());
  search.addEventListener('keydown',e=>{if(e.key==='Enter'){e.preventDefault();const first=select.querySelector('option:not([disabled])');if(first){showFormula(first.value);select.focus();}}});
  select.addEventListener('change',()=>{if(select.value)showFormula(select.value);});
  formulaDialog.append(el('div',{class:'formula-finder'},hardwareField('Search formulas',search),hardwareField('Formula',select),UI.button({class:'btn',id:'progNew',text:'+ New program',title:'A program of your own: a formula on a board you choose, reading and writing topics on the data bus',onclick:()=>newProgram()})),UI.status({class:'hint formula-search-status',id:'formulaSearchStatus',role:'status'}),el('h3',{id:'formulaTitle'}),el('p',{class:'hint',id:'formulaOwner'}),el('div',{id:'formulaActive'}));
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
  const C=computers(),sig=JSON.stringify(C)+'|'+cfg.comps.map(c=>[c.id,c.type,c.kind,c.name,c.battery,c.pack?JSON.stringify(c.pack):'',c.deviceW||0,c.elec,c.boardId,c.type==='mass'?Math.round(c.mass*1000):0]).join(';')+'|'+JSON.stringify([COMP.focus,bench.on])+'|'+JSON.stringify([radioCfg,radioCfg2,battCfg(),typeof programs==='function'?programs():[],typeof apps==='function'?apps().map(a=>[a.id,a.name,a.kind]):[]]);
  const wsig=sig.slice(0,sig.lastIndexOf('|'));
  if(wsig!==COMP.wsig){COMP.wsig=wsig;if(typeof bench!=='undefined'&&bench.on)droneShown='';}   // (the bench redraws its wires when the wiring changes)
  if(full||COMP.sig!==sig){
    COMP.sig=sig;COMP.rendering=true;
    try{
      renderComputerList(C);
      renderHardware();
      const box=$('#taskLaws');box.textContent='';
      for(const [t,T]of Object.entries(TASKS))box.append(lawSection('laws-'+t,T.label,T.what,T.formulas,boardOf(t)?.name||'unassigned'));
      box.append(lawSection('laws-ground',GROUND.label,GROUND.what,GROUND.formulas,C.ground.name));
      if($('#formulaDlg').open)showFormula(COMP.formula||$('#formulaSelect').value);
      renderComputerDetail();
    }finally{COMP.rendering=false;}
  }
  renderRunner();
}
function interactiveCard(attrs,open,...children){
  const card=UI.card({class:'computer-card',role:'button',tabindex:0,...attrs},...children);
  card.addEventListener('click',e=>{if(!e.target.closest('button,input,select,a'))open();});
  card.addEventListener('keydown',e=>{if(e.target===card&&['Enter',' '].includes(e.key)){e.preventDefault();open();}});return card;
}
// The list: a row per thing (a board, a power source, a sensor, another electronic part, a wire run, a duty), like
// Airframe's parts. Opening a row opens its settings under it and turns the 3D view to it (bench3d.js).
const viewKey=v=>v?v.kind+':'+(v.id??''):'';
const sameView=(a,b)=>viewKey(a)===viewKey(b);
function crow(view,attrs,tag,tagCls,name,sum,flag){
  const open=sameView(COMP.focus,view);
  const head=UI.button({class:'comp-head',type:'button','aria-expanded':String(open),'data-focus-key':'crow-'+viewKey(view)},el('span',{class:'tag '+tagCls,text:tag}),el('span',{class:'comp-name',text:name}),el('span',{class:'comp-sum',text:sum}));
  head.addEventListener('click',()=>compFocus(open?null:view));
  return UI.card({class:'comp crow'+(open?' open sel':'')+(flag?' '+flag:''),'data-ckey':viewKey(view),...attrs},el('div',{class:'comp-top'},head));
}
const gramsOf=c=>c&&c.mass!=null?(c.mass<0.0095?(c.mass*1000).toFixed(1):Math.round(c.mass*1000))+' g':'';
function renderComputerList(C){
  const comps=cfg.comps,count=(id,n)=>setText($('#'+id+'Count'),n?String(n):'');
  // boards
  const boards=C.boards.map(b=>{const K=BOARD_KINDS[b.kind],body=boardBodyOf(b),duty=runsFormulas(b)?b.tasks.map(t=>TASKS[t].label).join(', '):'',apps=appsOn(b).map(a=>a.name),progs=typeof programs==='function'?programs().filter(p=>p.board===b.id).map(p=>p.name):[];
    return crow({kind:'board',id:b.id},{'data-board':b.id},K.mcu?'MCU':'Linux','tag-board',b.name,[K.label,duty||(apps.length||progs.length?'':runsFormulas(b)?'no duties':'no apps'),progs.length?'programs: '+progs.join(', '):'',apps.length?'apps: '+apps.join(', '):'',body?gramsOf(body):'not on the airframe'].filter(Boolean).join(' · '),body?'':'virtual');});
  boards.push(crow({kind:'board',id:'ground'},{'data-board':'ground'},'Ground','tag-ground',C.ground.name,BOARD_KINDS[C.ground.kind].label+' · the pilot\'s side of the radio'));
  $('#boardList').replaceChildren(...boards);count('computerBoardsSec',C.boards.length+1);
  // power
  const srcs=powerSources(C,comps),ks=powerConsumers(C,comps);
  $('#computerPower').replaceChildren(...srcs.map(s=>{
    const on=sourceOn(C,s),mine=ks.filter(k=>(consumerSource(C,comps,k.key)||{}).id===s.id),esc=mine.filter(k=>k.kind==='motor').length;
    const what=[...mine.filter(k=>k.kind!=='motor').map(k=>k.name),...(esc?[esc>1&&esc===ks.filter(k=>k.kind==='motor').length?'all '+esc+' ESCs':esc+' ESC'+(esc>1?'s':'')]:[])];
    return crow({kind:'power',id:s.id},{'data-power':s.id},s.kind==='battery'?'Batt':s.kind==='usb'?'USB':'PSU','tag-power',s.name,sourceLabel(C,s,comps)+' · '+(on?(what.length?'powers '+what.join(', '):'powers nothing yet'):'not connected'),on&&what.length?'':'empty');
  }),...(srcs.length?[]:[crow({kind:'addpower'},{'data-power':'none'},'None','tag-power','No power','Nothing powers the drone: add a battery, or on the bench a USB port or supply','empty')]));
  count('computerPowerSec',srcs.length);
  // sensors
  const sens=comps.filter(c=>c.type==='sensor');
  const SHORT={imu:'IMU',mag:'Mag',baro:'Baro',fix:'GPS',flow:'Flow',tof:'ToF'};
  $('#computerSensors').replaceChildren(...sens.map(c=>{const b=hardwareOwner(C,c),p=hardwarePart(C,c,comps),i2c=I2C_SENSOR_KINDS.includes(c.kind);
    const how=c.kind==='flow'?(p.driver==='picam'?'Pi camera':'simulated only'):p.driver==='none'?'simulated only':i2c?'I²C '+(p.address?'0x'+p.address.toString(16):'auto'):c.kind==='fix'?'serial '+(p.port||'/dev/ttyUSB0'):'';
    return crow({kind:'device',id:c.id},{'data-device':c.id},SHORT[c.kind]||tagOf(c),'tag-sensor',c.name,[b?b.name:'no board',how].filter(Boolean).join(' · '),b?'':'empty');}));
  $('#computerSensorsSec').hidden=!sens.length;count('computerSensorsSec',sens.length);
  // other electronics
  const bodies=new Set(C.boards.map(b=>boardBodyOf(b)).filter(Boolean)),other=comps.filter(c=>c.type==='mass'&&partIsElec(c)&&!c.battery&&!bodies.has(c));
  $('#computerElec').replaceChildren(...other.map(c=>crow({kind:'part',id:c.id},{'data-part':c.id},c.camera?'Cam':'Part','tag-mass',c.name,[gramsOf(c),c.deviceW>0?'draws '+c.deviceW+' W':'',c.camera?'camera':''].filter(Boolean).join(' · '))),
    ...(other.length?[]:[el('p',{class:'hint',text:'Converters, cables, a camera board: add one with + Add, or mark a rigid mass in Airframe as electronics.'})]));
  count('computerElecSec',other.length);
  // wiring
  const wires=benchWires(C,comps),issues=C.boards.reduce((n,b)=>{const P=boardWiringPlan(b);return n+P.errors.length;},0)+powerIssues(C,comps).errors.length;
  $('#computerWiring').replaceChildren(crow({kind:'wiring'},{'data-wiring':''},'Wires','tag-wire','Wiring overview',wires.length+' wire'+(wires.length===1?'':'s')+(issues?' · '+issues+' problem'+(issues>1?'s':''):' · no problems'),issues?'bad':''));
  const links=boardLinks(C);$('#computerLinks').replaceChildren(...links.map(L=>crow({kind:'link',id:L.b.id},{'data-link':L.b.id},'Link','tag-wire',L.a.name+' ↔ '+L.b.name,!L.pi?'simulated link · no wiring recipe yet':L.transport==='mqtt'?'MQTT · Wi-Fi · '+L.port:(L.port==='/dev/serial0'?'UART wires':'USB serial adapter')+' · '+L.baud+' baud',L.supported?'':'empty')));
  for(const [id,filter]of [['computerOutputs',c=>['motor','joint'].includes(c.type)],['computerCargo',c=>c.type==='latch']]){
    $('#'+id).replaceChildren(...comps.filter(filter).map(c=>{const b=hardwareOwner(C,c),p=hardwarePart(C,c,comps);
      return crow({kind:'device',id:c.id},{'data-device':c.id},c.type==='motor'?(p.driver==='brushed'?'FET':'ESC'):c.type==='joint'?'Servo':'Latch','tag-'+c.type,c.name,b?b.name+' · '+(p.pin>=0?pinShort(C,b,p.pin):'choose a pin'):'no board',b&&p.pin>=0?'':'empty');}));
  }
  $('#computerRadio').replaceChildren(crow({kind:'radio'},{'data-radio':'primary'},'Radio','tag-wire',RADIO_LINKS[radioCfg.kind]?.label||'Radio',(boardOf('tlm')?.name||'no board')+(radioTwo()?' · and '+(RADIO_LINKS[radioCfg2.kind]?.label||radioCfg2.kind):''),boardOf('tlm')?'':'empty'));
  count('computerWiringSec',wires.length);const outs=comps.filter(c=>['motor','joint','latch'].includes(c.type)).length;count('computerOutputsSec',outs);$('#computerOutputsSec').hidden=!outs;
  // duties
  $('#taskRows').replaceChildren(...Object.entries(TASKS).map(([id,T])=>{const b=boardOf(id);return crow({kind:'task',id},{'data-task':id},'Duty','tag-duty',T.label,b?b.name+(runsFormulas(b)?' · '+boardTaskHz(b,id)+' Hz':''):'not assigned',b?'':'empty');}));
  // the open row's settings, under it
  const card=COMP.focus&&$('#paneForm').querySelector('[data-ckey="'+CSS.escape(viewKey(COMP.focus))+'"]');
  if(card)card.append(COMP.detail);else{COMP.detail.remove();if(COMP.focus&&!['addpower'].includes(COMP.focus.kind)&&!COMP.rendering)COMP.focus=null;}
  $('#compViewSeg')?.querySelectorAll('[data-comp-view]').forEach(b=>b.setAttribute('aria-pressed',String((b.dataset.compView==='wiring')===(COMP.focus?.kind==='wiring'))));
  if(typeof benchBarSync==='function')benchBarSync();
}
// Open a row (null: close it). The 3D view follows; a click on a part in the view comes here too.
function compFocus(view,opt={}){
  if(view&&['bus','addpower'].includes(view.kind)){openComputerView(view);return;}
  const dlg=$('#computerDlg');if(dlg&&dlg.open&&COMP.view&&COMP.view.kind==='addpower')dlg.close();
  const same=sameView(COMP.focus,view);COMP.focus=view||null;COMP.confirm=null;
  if(view&&$('#paneForm').hidden)UI_PANELS.editor.select('form');
  renderComputers(true);
  if(typeof benchFocus==='function')benchFocus(COMP.focus);
  if(view&&(opt.from3d||!same)){const card=$('#paneForm').querySelector('[data-ckey="'+CSS.escape(viewKey(view))+'"]');if(card)requestAnimationFrame(()=>card.scrollIntoView({block:'nearest'}));}
}
// + Add: a board, a power source, a sensor or another part.
function compAddItems(){
  const full=computers().boards.length>=BOARD_MAX;
  return [...Object.entries(BOARD_KINDS).filter(([,K])=>!K.groundOnly).map(([k,K])=>({value:'board:'+k,label:K.label,hint:K.mcu?'microcontroller':'Linux computer',group:'Board',disabled:full})),
    ...Object.entries(POWER_KINDS).map(([k,P])=>({value:'power:'+k,label:P.label,group:'Power'})),
    ...Object.entries(SENSOR_KINDS).map(([k,n])=>({value:'sensor:'+k,label:n,group:'Sensor'})),
    {value:'part',label:'Electronic part',hint:'a converter, cable, power monitor…',group:'Other'}];
}
function compAddPick(v){
  const [what,k]=v.split(':');
  if(what==='board'){const C=JSON.parse(JSON.stringify(computers()));if(C.boards.length>=BOARD_MAX)return;const id=C.nextBoardId||Math.max(0,...C.boards.map(b=>b.id))+1;C.boards.push({id,kind:k,name:BOARD_KINDS[k].label,tasks:[]});setComputers(C,'add');compFocus({kind:'board',id});return;}
  if(what==='power'){COMP.addPower={kind:k,preset:Object.entries(POWER_PRESETS).find(([,p])=>p.kind===k)[0],name:''};openComputerView({kind:'addpower'});return;}
  if(what==='sensor'){const c=addComp(k);compFocus({kind:'device',id:c.id});return;}
  const c=mkMass('Electronic part',0.03,0,frameDims()[2]/2+0.008,{mass:0.005,size:[0.03,0.02,0.006]});c.elec=true;c.known=true;cfg.comps.push(c);undoKey='addpart';structural();compFocus({kind:'part',id:c.id});
}
// A part's own settings (where it sits, what it weighs, its shape), as in Airframe.
// The inspector: every row's settings are built from the same few parts, always in the same order:
//   problems (only when there are some) → key facts → the main action → sections (what it is, how it's connected,
//   what it does, where it sits) → folded advanced settings → removing it, last.
// Each control appears once: a board is chosen in one place, a part's physical properties in one place.
const dfacts=(...pairs)=>el('dl',{class:'dfacts'},...pairs.filter(Boolean).map(([k,v,cls])=>el('div',{class:'dfact'+(cls?' '+cls:'')},el('dt',{text:k}),el('dd',{text:v}))));
const dlead=text=>el('p',{class:'dlead',text});
const dsec=(title,...kids)=>el('section',{class:'dsec'},el('h3',{class:'dsec-h',text:title}),...kids.filter(Boolean));
const dfold=(key,title,...kids)=>UI.details({class:'dsec dfold',title,open:HW_UI.open.has('d:'+key),onToggle:o=>{o?HW_UI.open.add('d:'+key):HW_UI.open.delete('d:'+key);}},...kids.filter(Boolean));
const dactions=(...b)=>el('div',{class:'dactions'},...b.filter(Boolean));
const dissues=(errors=[],warnings=[])=>errors.length||warnings.length?el('div',{class:'dissues',role:'status'},...errors.map(m=>el('p',{class:'dissue bad',text:m})),...warnings.map(m=>el('p',{class:'dissue',text:m}))):null;
const ddanger=(...kids)=>el('div',{class:'ddanger'},...kids.filter(Boolean));
const mm=v=>Math.round(v*1000);
const put=(box,...kids)=>box.append(...kids.filter(Boolean));
function nameInput(value,label,id,onName){
  const i=UI.input({type:'text',value,maxlength:24,'aria-label':label,id,autocomplete:'off',spellcheck:'false'});
  i.addEventListener('change',()=>onName(i.value.trim()));
  i.addEventListener('keydown',e=>{if(e.key==='Enter')i.blur();else if(e.key==='Escape'){i.value=value;i.blur();}});
  return i;
}
// Hardware cards (hardware-ui.js) in the inspector: alone in a section they lose their title (the section names them);
// several (a board's own settings) each fold under their title, closed until opened.
function cardFolds(nodes,key,drop=''){return nodes.map(n=>{if(n.tagName==='DETAILS'){n.classList.add('dsub');const s=n.querySelector(':scope>summary');if(s&&drop)s.textContent=s.textContent.replace(drop+' · ','').replace(/^./,ch=>ch.toUpperCase());return n;}if(!n.classList||!n.classList.contains('hw-device'))return n;const t=n.querySelector(':scope>.hw-device-title'),title=t?[...t.children].map(x=>x.textContent).filter(x=>x&&x!==drop).join(' · '):'Settings',k='d:'+key+':'+title;return UI.details({class:'dsub',title,open:HW_UI.open.has(k),onToggle:o=>{o?HW_UI.open.add(k):HW_UI.open.delete(k);}},n);});}
function mounted(selector){const tmp=el('div',{});mountHardware(tmp,selector);return [...tmp.children];}
// One choice of board, as a list of buttons (the duties, the radio): the one in use pressed, Not assigned last.
function choiceList(current,boards,apply,none='Not assigned'){
  const list=el('div',{class:'choice-list',role:'group'});
  for(const b of boards)list.append(UI.button({class:'choice','aria-pressed':String(current?.id===b.id),'data-assign-board':b.id,onclick:()=>{if(current?.id!==b.id)apply(b.id);}},el('b',{text:b.name}),el('span',{text:BOARD_KINDS[b.kind].label})));
  list.append(UI.button({class:'choice','aria-pressed':String(!current),'data-remove-assignment':'',onclick:()=>{if(current)apply(null);}},el('b',{text:none}),el('span',{text:'nothing runs it'})));
  return boards.length?list:el('div',{},list,el('p',{class:'hint',text:'No board can run it yet: add one with + Add.'}));
}
// Wires as rows: a colour for what it carries, where it goes; hover picks it out in the view, a click opens its end.
function wireRows(wires){
  return wires.map(w=>{
    const row=UI.button({class:'wire-row','data-wire':w.id,type:'button',title:WIRE_KINDS[w.kind]||w.kind},el('i',{class:'pd-swatch pd-'+(w.kind==='power'?'v5':w.kind==='radio'?'signal':w.kind)}),el('span',{text:w.text}),el('small',{text:WIRE_KINDS[w.kind]||''}));
    row.addEventListener('mouseenter',()=>benchHiWire(w.id));row.addEventListener('mouseleave',()=>benchHiWire(null));
    row.addEventListener('focus',()=>benchHiWire(w.id));row.addEventListener('blur',()=>benchHiWire(null));
    row.addEventListener('click',()=>{if(!sameView(w.view,COMP.focus))compFocus(w.view);});
    return row;
  });
}
function wireList(C){
  const wires=benchWires(C,cfg.comps),groups=new Map();
  for(const w of wires){const k=w.kind==='power'?'Power':w.kind==='radio'?'Radio':(w.a.key.startsWith('board:')?(C.boards.find(b=>'board:'+b.id===w.a.key)||C.ground).name:'Other');if(!groups.has(k))groups.set(k,[]);groups.get(k).push(w);}
  const box=el('div',{class:'wire-list'});
  for(const [k,list]of groups)box.append(el('h4',{text:k}),...wireRows(list));
  if(!wires.length)box.append(el('p',{class:'hint',text:'No wires yet: give the motors and sensors a board.'}));
  if(typeof pinoutLegend==='function')box.append(pinoutLegend());
  return box;
}
// A part's own settings (its name, a camera's picture, a sensor's rate and noise, its draw): always in view.
function partSettings(c){return typeof compCard==='function'?compCard(c,{only:'part'}):null;}
// Where it sits: a line, and Edit physical properties, which puts the view back together and gives the part
// Airframe's handles, its position, mass and size under them.
function physSection(c,title='On the airframe'){
  if(!c)return null;
  const on=bench.on&&bench.phys===c.id,size=c.type==='mass'&&!c.model&&c.shape==='box'?' · '+c.size.map(mm).join('×')+' mm':'';
  const btn=UI.button({class:'btn sm'+(on?' primary':''),'data-phys':c.id,text:on?'Done':'Edit physical properties…',title:on?'Back to the exploded view':'Place it on the assembled drone with handles, and edit its mass and size',onclick:()=>{if(on)benchPhys(null);else{bench.want=true;benchSync();benchPhys(c.id);}}});
  const s=dsec(title,el('div',{class:'cpart-head'},el('span',{class:'dmeta',text:gramsOf(c)+' · at '+c.pos.map(mm).join(', ')+' mm'+size}),btn));
  s.classList.add('cpart');
  if(on){s.classList.add('phys-on');s.append(el('p',{class:'hint',text:'Drag the arrows or squares in the view to move it, the rings to turn it (Shift: fine steps).'}),compCard(c,{only:'phys'}));}
  return s;
}
function removePart(c,what='part'){
  return UI.button({class:'btn sm danger','data-remove-part':c.id,text:'Remove '+what,onclick:()=>{for(const x of cfg.comps)if(x.parent===c.id)x.parent=c.parent??null;cfg.comps=cfg.comps.filter(x=>x!==c);openSet.delete(c.id);benchPhysEnd();COMP.focus=null;undoKey='remove'+c.id;structural();compFocus(null);}});
}
function renderPartDetail(box,C,id){
  const c=compById(id);if(!c){detClose();return;}detTitle(c.name);
  const b=boardOfBody(c,C);
  put(box,dfacts(['Mass',gramsOf(c)],c.deviceW>0&&['Draws',c.deviceW+' W'],['Board',b?b.name:'none']),
    dsec('Part',partSettings(c),hardwareField('This part is the board',hardwareSelect('Board this part is','part-board-'+c.id,[['','No board'],...C.boards.map(x=>[x.id,x.name+' · '+BOARD_KINDS[x.kind].label])],b?b.id:'',v=>{if(v==='')linkBoardBody(b||{id:-2},'');else linkBoardBody(C.boards.find(x=>x.id===Number(v)),c.id);}))),
    physSection(c),ddanger(removePart(c)));
}
// A board's body: the part that is it on the airframe, or Put it on the airframe.
function boardBodySection(b){
  const body=boardBodyOf(b),B=benchBody(b.kind),masses=cfg.comps.filter(c=>c.type==='mass'&&!c.battery&&!isWing(c));
  const pick=hardwareField('Its part',hardwareSelect(b.name+' part','bbody-'+b.id,[['','None: not on the airframe'],...masses.map(c=>[c.id,c.name+' · '+gramsOf(c)])],body?body.id:'',v=>linkBoardBody(b,v)));
  if(body){const s=physSection(body);s.insertBefore(pick,s.children[1]);return s;}
  return dsec('On the airframe',el('p',{class:'hint',text:'Not on the airframe: the simulation flies without its '+gramsOf({mass:B.mass})+', and the view shows it above the hub, dashed.'}),
    dactions(UI.button({class:'btn sm','data-place-board':b.id,text:'Put it on the airframe',onclick:()=>{placeBoardBody(b);renderComputers(true);}})),pick);
}
// The 5 V a board runs on: what makes it from its source, and what it draws.
function boardSupplyFields(C,x){
  const comps=cfg.comps,S=boardSupplyOf(C,x,comps),others=C.boards.filter(o=>o!==x),sx=hardwareBus(C,x);
  const pick=v=>editWiring(w=>{w.boards[x.id]={...hardwareBus(computers(),x),supply:v,...(v==='board'&&!others.some(o=>o.id===sx.supplyFrom)&&others[0]?{supplyFrom:others[0].id}:{})};},x.id+'supply');
  const kinds=Object.entries(POWER_SUPPLIES).filter(([k])=>(k!=='board'||others.length)&&(k!=='usb'||sx.supply==='usb')&&(k!=='direct'||sx.supply==='direct'||ESP_PROFILES[x.kind]));
  const autoKind=S.spec.vmin>=4.7&&S.spec.vmax<=5.3?'direct5':S.spec.vmax<5?'boost5':'reg5';
  const f=el('div',{class:'hw-fields'},hardwareField('5 V from',hardwareSelect(x.name+' supply','pw-supply-'+x.id,[['auto','Automatic: '+POWER_SUPPLIES[autoKind].short],...kinds.map(([k,d])=>[k,d.label])],S.auto?'auto':S.kind,pick)));
  if(S.kind==='board')f.append(hardwareField('Fed from',hardwareSelect(x.name+' fed from','pw-from-'+x.id,others.map(o=>[o.id,o.name+' · '+supplyPinName(C,o)]),S.from?.id??'',v=>editWiring(w=>{w.boards[x.id]={...hardwareBus(computers(),x),supplyFrom:Number(v)};},x.id+'supplyFrom'))));
  f.append(hardwareNumber('Draw at 5 V (W)',boardWatts(C,x),v=>v>=0&&v<=50&&editWiring(w=>{w.boards[x.id]={...hardwareBus(computers(),x),watts:v};},x.id+'watts'),0,50));
  return f;
}
// Power: a card per source (a battery on the airframe, a USB port or charger, a bench supply) with what it powers, and
// Add power. A source's detail: its specs (from a preset, then any edited), what it powers (each board, each motor's
// ESC, the servos, each part with a draw), the boards' 5 V supplies and draws, the drawing and the problems.
function setBatteryWired(c,on){editWiring(w=>{w.parts[c.id]={...w.parts[c.id],power:!!on};},'power'+c.id);}
const sourceOf=id=>powerSources(computers(),cfg.comps).find(s=>String(s.id)===String(id))||null;
// Every consumer's source written down, as it is now: from then on the power list decides, not the old rules.
function materializePower(w,C,comps){
  w.power||={};
  for(const k of powerConsumers(C,comps)){
    if(Object.prototype.hasOwnProperty.call(w.power,k.key)||(k.kind==='board'&&hardwareBus(C,k.board).supply==='board'))continue;
    const s=consumerSource(C,comps,k.key);if(s&&s.own)continue;   // (a board on its own USB port in an old design: left as it is)
    w.power[k.key]=s?s.id:'';
  }
}
function setPowered(key,id){
  editWiring(w=>{const C=computers();materializePower(w,C,cfg.comps);w.power[key]=id;
    const b=key.startsWith('board:')&&C.boards.find(x=>'board:'+x.id===key);if(b&&id!==''&&(w.boards[b.id]||{}).supply==='usb')w.boards[b.id]={...w.boards[b.id],supply:'auto'};},'power'+key);
}
// A source's spec: the main pack's are the Battery settings; another pack's are its part's; a USB port's or supply's
// are in the wiring.
function setSourceSpec(s,changes){
  if(s.kind==='battery'){
    const C=computers(),P=packSpec(C,s.part,cfg.comps);   // (every pack keeps its own specs; the main pack's are copied into the Battery settings)
    s.part.pack={cells:P.cells,capacity:P.capacity,rInt:P.rInt,maxA:P.maxA,startSoc:P.startSoc,...changes};undoKey='pack'+s.id;syncMainPack(computers(),cfg.comps);
    recomputeProps();if(typeof refreshEnvelope==='function')refreshEnvelope();if(typeof renderMass==='function')renderMass();if(typeof renderBattery==='function')renderBattery();doReset();save();renderComputers(true);
  }else editWiring(w=>{w.sources=(w.sources||[]).map(x=>x.id===s.id?{...x,...changes}:x);},'source'+s.id);
}
function renameSource(s,name){
  if(!name)return;
  if(s.kind==='battery'){s.part.name=name.slice(0,40);undoKey='name'+s.id;structural();renderComputers(true);}
  else editWiring(w=>{w.sources=(w.sources||[]).map(x=>x.id===s.id?{...x,name:name.slice(0,40)}:x);},'source'+s.id);
}
function removeSource(s){COMP.focus=null;editWiring(w=>{w.sources=(w.sources||[]).filter(x=>x.id!==s.id);for(const k of Object.keys(w.power||{}))if(w.power[k]===s.id)w.power[k]='';},'source'+s.id);benchFocus(null);}
// Add power: a battery (a part on the airframe, sized and weighed from the preset) or a source off the drone. With
// other sources there already, the new one powers nothing until ticked; the first powers everything.
function addPower(kind,presetKey,name){
  const P=POWER_PRESETS[presetKey]||Object.values(POWER_PRESETS).find(p=>p.kind===kind),C0=computers(),first=!powerSources(C0,cfg.comps).length;
  if(kind==='battery'){
    const was=batteryParts(cfg.comps).length,c=addComp('battery');c.name=(name||P.label).slice(0,40);
    const spec={cells:P.cells,capacity:P.capacity,maxA:P.maxA,rInt:P.rInt,startSoc:1},mass=lipoMass(P.cells,P.capacity);
    c.mass=+mass.toFixed(4);c.batterySizing={capacity:P.capacity,cells:P.cells,mass:c.mass};
    c.pack=spec;
    if(!first)editWiring(w=>{materializePower(w,C0,cfg.comps.filter(x=>x!==c));},'addpower');
    structural();openComputerView({kind:'power',id:c.id});
  }else{
    const ids=(C0.wiring&&C0.wiring.sources||[]).map(s=>s.id),id='s'+(ids.reduce((n,x)=>Math.max(n,+String(x).slice(1)||0),0)+1);
    editWiring(w=>{if(!first)materializePower(w,C0,cfg.comps);w.sources=[...(w.sources||[]),{id,kind,name:(name||P.label).slice(0,40),volts:P.volts,maxA:P.maxA,preset:presetKey}];},'addpower');
    openComputerView({kind:'power',id});
  }
}
// The Add power detail: the kind, a preset of it, a name.
function renderAddPower(box){
  detTitle('Add power');
  const st=COMP.addPower||(COMP.addPower={kind:'battery',preset:'lipo1s1800',name:''});
  const presets=Object.entries(POWER_PRESETS).filter(([,p])=>p.kind===st.kind);
  if(!presets.some(([k])=>k===st.preset))st.preset=presets[0][0];
  box.append(el('p',{text:'Where the power comes from. A battery goes on the airframe (its mass and place are a part you can move in Airframe); a USB port or bench supply stays off the drone. Then tick what it powers.'}),
    el('div',{class:'hw-fields'},
      hardwareField('Kind',hardwareSelect('Kind','ap-kind',Object.entries(POWER_KINDS).map(([k,d])=>[k,d.label]),st.kind,v=>{st.kind=v;renderComputerDetail();})),
      hardwareField('Preset',hardwareSelect('Preset','ap-preset',presets.map(([k,p])=>[k,p.label]),st.preset,v=>{st.preset=v;renderComputerDetail();})),
      hardwareText('Name',st.name||POWER_PRESETS[st.preset].label,v=>{st.name=v;})),
    el('p',{class:'hint',text:'Every spec can be changed afterwards: cells, capacity, the most it gives, internal resistance.'}),
    UI.button({class:'btn primary','data-add-power-go':'',text:'Add',onclick:()=>{const s={...st};COMP.addPower=null;addPower(s.kind,s.preset,s.name);}}));
}
function renderPowerSource(box,C,id){
  const s=sourceOf(id);if(!s){detClose();return;}
  const comps=cfg.comps,on=sourceOn(C,s),P=sourceSpec(C,s,comps),bat=s.kind==='battery';
  detTitle(s.name);
  // what it powers, and what that draws
  const ks=powerConsumers(C,comps),list=el('div',{class:'pw-powers'}),groups=[['Boards',k=>k.kind==='board'],['ESCs',k=>k.kind==='motor'],['Other',k=>k.kind!=='board'&&k.kind!=='motor']];
  let mineA=0,motorsA=0;
  for(const [title,f]of groups){
    const g=ks.filter(f);if(!g.length)continue;
    const head=el('div',{class:'pw-powers-head'},el('b',{text:title}));
    if(title==='ESCs'&&g.length>1)head.append(UI.button({class:'btn sm','data-all-escs':'',text:'All ESCs',onclick:()=>editWiring(w=>{materializePower(w,C,comps);for(const k of g)w.power[k.key]=s.id;},'escs'+s.id)}));
    list.append(head);
    for(const k of g){
      const cur=consumerSource(C,comps,k.key),viaPin=k.kind==='board'&&hardwareBus(C,k.board).supply==='board',mine=cur&&cur.id===s.id,a=consumerAmps(C,comps,k,P);
      if(mine&&!viaPin){if(k.kind==='motor')motorsA+=a||0;else mineA+=a;}
      const box2=UI.input({type:'checkbox',id:'pw-on-'+s.id+'-'+k.key.replace(':','-'),'data-consumer':k.key});box2.checked=!!mine;box2.disabled=viaPin||!on;
      box2.addEventListener('change',()=>setPowered(k.key,box2.checked?s.id:''));
      const note=viaPin?'from '+(boardSupply(C,k.board,1).from||{}).name+'\'s 5 V pin':cur&&!mine?'now on '+cur.name:!cur?'no power':'';
      const draw=k.kind==='motor'?(a?'up to '+a+' A':''):'≈'+(a<1?Math.round(a*1000)+' mA':a.toFixed(2)+' A');
      list.append(el('label',{class:'check pw-consumer',for:box2.id},box2,el('span',{text:k.name}),el('span',{class:'computer-meta',text:[draw,note].filter(Boolean).join(' · ')})));
    }
  }
  const amps=a=>a<1?Math.round(a*1000)+' mA':a.toFixed(2)+' A',I=powerIssues(C,comps);
  put(box,dissues([...(!cargo.power?['No power now: '+powerWhy()+'.']:[]),...I.errors],[]),
    dfacts(['Kind',POWER_KINDS[s.kind].short],['Voltage',bat?P.vmin.toFixed(1)+'–'+P.vmax.toFixed(1)+' V':P.volts+' V'],['Gives up to',P.maxA+' A'],['Draw',amps(mineA)+(motorsA?' · '+(mineA+motorsA).toFixed(1)+' A flat out':'')],['State',on?'connected':'not connected',on?'':'bad']));
  if(bat)put(box,dactions(UI.button({class:'btn sm'+(on?'':' primary'),'data-power-toggle':'',text:on?'Disconnect':'Connect',onclick:()=>setBatteryWired(s.part,!on)})));
  // specs
  const presets=Object.entries(POWER_PRESETS).filter(([,p])=>p.kind===s.kind);
  const spec=el('div',{class:'hw-fields'},...(bat?[]:[hardwareText('Name',s.name,v=>renameSource(s,v))]),
    hardwareField('Preset',hardwareSelect(s.name+' preset','pw-preset-'+s.id,[['','Choose a preset…'],...presets.map(([k,p])=>[k,p.label])],'',v=>{const p=POWER_PRESETS[v];if(!p)return;const {kind,label,...x}=p;void kind;void label;setSourceSpec(s,bat?x:{...x,preset:v});})));
  if(bat)spec.append(hardwareNumber('Cells in series',P.cells,v=>Number.isInteger(v)&&v>=1&&v<=8&&setSourceSpec(s,{cells:v}),1,8),hardwareNumber('Capacity (Ah)',P.capacity,v=>v>0&&v<=50&&setSourceSpec(s,{capacity:v}),0.05,50),
    hardwareNumber('Most it gives (A)',+P.maxA.toFixed(1),v=>v>0&&v<=1000&&setSourceSpec(s,{maxA:v}),0.1,1000),hardwareNumber('Internal resistance (mΩ)',Math.round(P.rInt*1000),v=>v>0&&v<1000&&setSourceSpec(s,{rInt:v/1000}),1,999),
    hardwareNumber('Charge at start (%)',Math.round((P.startSoc??1)*100),v=>v>=5&&v<=100&&setSourceSpec(s,{startSoc:v/100}),5,100));
  else spec.append(hardwareNumber('Volts',P.volts,v=>v>0&&v<=60&&setSourceSpec(s,{volts:v}),0.5,60),hardwareNumber('Most it gives (A)',P.maxA,v=>v>0&&v<=100&&setSourceSpec(s,{maxA:v}),0.05,100));
  put(box,dsec('Specs',spec,el('p',{class:'hint',text:bat?'Most it gives: its C rating × its capacity (20C × 1.8 Ah: 36 A). '+(P.main?'The main pack: the motors run on it, the flight controller reads it, and its specs are also Tune → Battery.':'Its mass follows the cells and capacity.'):'Off the drone, through a cable. A USB port shuts off when asked for well over its rating; a bench supply holds its current limit and the voltage sags.'})));
  put(box,dsec('Powers',list,el('p',{class:'hint',text:'Each source\'s − joins one common ground. A board\'s 5 V converter is set in the board\'s own settings.'}),dissues([],I.warnings)));
  if(bat){
    const sense=el('div',{});mountHardware(sense,'[data-hw-role="battery"],[data-hw-role="pmon"]');
    put(box,dfold('sense'+s.id,'Voltage and current sensing',el('p',{class:'hint',text:'Optional: lets the flight controller see this pack (low-battery warnings, thrust that stays true as it drains): a resistor divider on an ADC pin, or a power monitor (INA219, INA226, INA3221) for voltage and current.'}),sense));
  }
  if(typeof powerDiagramView==='function')put(box,dfold('diagram','Power diagram',powerDiagramView(C,comps)));
  if(bat)put(box,dsec('Pack',partSettings(s.part)),physSection(s.part));
  put(box,ddanger(bat?removePart(s.part,'battery'):UI.button({class:'btn sm danger',text:'Remove this source',onclick:()=>removeSource(s)})));
}
function openComputerView(view){
  if(!['bus','addpower'].includes(view.kind)){compFocus(view);return;}   // (everything else opens in the list, with the 3D view on it)
  const dlg=$('#computerDlg');
  if(!dlg.open){
    const active=document.activeElement,card=active.closest('[data-board],[data-task],[data-device],[data-radio],[data-power],[data-link]');
    const key=card&&['board','task','device','radio','power','link'].find(k=>Object.hasOwn(card.dataset,k));
    COMP.returnFocus=key?'#paneForm [data-'+key+'="'+CSS.escape(card.dataset[key])+'"]':active.id?'#'+CSS.escape(active.id):null;
  }
  COMP.view=view;COMP.confirm=null;if(!dlg.open)dlg.showModal();renderHardware();renderComputerDetail();dlg.scrollTop=0;
}
// (a duty needs a board that runs formulas; a device's wiring doesn't: forDevice)
function eligibleTaskBoards(task,forDevice=false){const T=TASKS[task];return computers().boards.filter(b=>(!T.mcuOnly||BOARD_KINDS[b.kind].mcu)&&(!T.piOnly||!BOARD_KINDS[b.kind].mcu)&&(forDevice||runsFormulas(b)));}
function assignDuty(task,id){
  const C=JSON.parse(JSON.stringify(computers()));for(const b of C.boards)b.tasks=b.tasks.filter(t=>t!==task);
  if(id!=null){const b=C.boards.find(b=>b.id===id);if(!b||!eligibleTaskBoards(task).some(x=>x.id===id))return;b.tasks.push(task);}
  if(task==='core')C.unassignedCore=id==null;setComputers(C,'task');
}
function mountHardware(box,selector){for(const n of [...$('#hardwareRows').querySelectorAll(selector)])box.append(n);}
function renderComputerDetail(){
  const dlg=$('#computerDlg');
  if(dlg&&dlg.open&&COMP.view)renderDetailInto(COMP.view,$('#computerDlgBody'),t=>{$('#computerDlgTitle').textContent=t;},()=>dlg.close());
  const host=$('#compDetailBody');
  if(COMP.focus&&host&&host.isConnected)renderDetailInto(COMP.focus,host,t=>{$('#compDetailTitle').textContent=t;},()=>{COMP.focus=null;setTimeout(()=>compFocus(null));});
}
const detTitle=t=>COMP.det&&COMP.det.title(t),detClose=()=>COMP.det&&COMP.det.close();
const planIssues=b=>{if(!b)return null;const P=boardWiringPlan(b);return dissues(P.errors,P.warnings);};
const pinShort=(C,b,g)=>{const s=boardGpioShort(C,b,g);return /^\d+$/.test(s)?'GPIO '+s:s;};
function renderDetailInto(view,box,setTitle,close){
  if(!view||!box)return;COMP.det={title:setTitle,close};box.textContent='';box.className='computer-detail';const C=computers();
  renderUndo();
  if(view.kind==='bus'){renderBusView(box);return;}
  if(view.kind==='addpower'){renderAddPower(box);return;}
  if(view.kind==='power'){renderPowerSource(box,C,view.id);return;}
  if(view.kind==='part'){renderPartDetail(box,C,view.id);return;}
  if(view.kind==='wiring'){
    detTitle('Wiring');const wires=benchWires(C,cfg.comps),bad=C.boards.reduce((n,b)=>n+boardWiringPlan(b).errors.length,0)+powerIssues(C,cfg.comps).errors.length;
    put(box,dfacts(['Wires',String(wires.length)],['Boards',String(C.boards.length+1)],['Errors',bad?String(bad):'none',bad?'bad':'']));
    mountHardware(box,'#hardwareReport');   // (the checks first: problems lead, as everywhere)
    put(box,dsec('Every wire',el('p',{class:'hint',text:'What the install guide asks for. Hover a wire to pick it out in the view; click it to open what it goes to.'}),wireList(C)));
    mountHardware(box,'[data-hw-role="ground"]');
    put(box,dfold('overview','Board diagrams & the full connection list',renderWiringOverview(C)));return;
  }
  if(view.kind==='task'||view.kind==='radio'){
    const t=view.kind==='radio'?'tlm':view.id,T=TASKS[t],b=boardOf(t);
    detTitle(view.kind==='radio'?'Radio':T.label);
    put(box,dlead(T.what),dfacts(b&&runsFormulas(b)&&['Rate',boardTaskHz(b,t)+' Hz'],view.kind==='radio'&&['Link',(RADIO_LINKS[radioCfg.kind]?.label||radioCfg.kind)+(radioTwo()?' + '+(RADIO_LINKS[radioCfg2.kind]?.label||radioCfg2.kind):'')],T.formulas.length&&['Formulas',String(T.formulas.length)]),
      dsec('Runs on',choiceList(b,eligibleTaskBoards(t),id=>assignDuty(t,id))));
    if(view.kind==='radio'){const w=el('div',{class:'dsolo'});mountHardware(w,'[data-hw-role="radio"]');const P=b&&boardWiringPlan(b),rf=m=>/radio|receiver|crsf|nrf|link|elrs|espnow|esp-now|wi-?fi|bluetooth|ble/i.test(m);put(box,dsec('Wiring',P&&dissues(P.errors.filter(rf),P.warnings.filter(rf)),w));}
    else if(T.formulas.length)put(box,dactions(UI.button({class:'btn sm',text:'Open its formulas…',onclick:()=>openFormulaEditor(T.formulas[0])})));
    return;
  }
  if(view.kind==='link'){
    const L=boardLinks(C).find(L=>L.b.id===view.id);if(!L){detClose();return;}detTitle(L.a.name+' ↔ '+L.b.name);
    const wires=benchWires(C,cfg.comps).filter(w=>w.view.kind==='link'&&w.view.id===L.b.id),set=el('div',{class:'dsolo'});mountHardware(set,'[data-hw-role="link-'+L.b.id+'"]');
    put(box,planIssues(L.b),dlead(L.transport==='mqtt'?L.b.name+' and '+L.a.name+' (the flight core) talk through an MQTT broker '+(L.broker==='here'?'on '+L.b.name+': one Wi-Fi hop':'elsewhere on the network: two hops')+'. Every frame is checked; if the flight core hears nothing for half a second it levels and lands.':L.b.name+' and '+L.a.name+' (the flight core) talk over a serial line. Every frame is checked; if the flight core hears nothing for half a second it levels and lands.'),
      dfacts(['Transport',!L.pi?'simulated':L.transport==='mqtt'?'MQTT · Wi-Fi':L.port==='/dev/serial0'?'UART wires':'USB serial adapter'],L.baud&&['Speed',L.baud+' baud'],L.pi&&L.transport!=='mqtt'&&['Ports',L.port]),
      L.wires.length?dsec('Wires',...wireRows(wires),el('p',{class:'hint',text:'TX goes to RX both ways, and the grounds join. 3.3 V logic on both ends: no level shifter.'})):null,
      L.carries.length?dsec('What it carries',el('ul',{class:'link-wires'},...L.carries.map(t=>el('li',{text:t})))):null,
      L.note?el('p',{class:'hint',text:L.note}):null,dsec('Settings',set));
    const draw=typeof boardLinkView==='function'&&boardLinkView(C,L);if(draw)put(box,dfold('linkdraw','On the boards',draw,pinoutLegend()));
    return;
  }
  if(view.kind==='device'){
    const c=compById(view.id);if(!c){detClose();return;}detTitle(c.name);
    const b=hardwareOwner(C,c),p=hardwarePart(C,c,cfg.comps),out=['motor','joint','latch'].includes(c.type),i2c=c.type==='sensor'&&I2C_SENSOR_KINDS.includes(c.kind),bus=b&&hardwareBus(C,b);
    const drv=c.type==='sensor'&&DEVICE_PROFILES[c.kind]&&DEVICE_PROFILES[c.kind][p.driver];
    const conn=el('div',{});mountHardware(conn,'[data-hw-part="'+c.id+'"]');
    const errs=[...(out&&b&&c.type!=='latch'&&b.id!==boardOf('core')?.id?['Only the board running the flight core drives motors and servos: move the flight core to '+b.name+', or choose that board here.']:[])];
    const own=m=>m.includes(c.name),P=b&&boardWiringPlan(b);
    put(box,dissues([...errs,...(P?P.errors.filter(own):[])],P?P.warnings.filter(own):[]),
      dfacts(['Board',b?b.name:'none',b?'':'bad'],
        out?['Signal',b&&p.pin>=0?pinShort(C,b,p.pin):'no pin',b&&p.pin>=0?'':'bad']:i2c?['I²C',(p.address?'0x'+p.address.toString(16):'auto address')]:c.kind==='fix'?['Port',p.port||'/dev/ttyUSB0']:c.kind==='flow'&&p.driver==='picam'?['Camera','CSI (dfb_flow)']:['Bus','simulation only'],
        i2c&&b&&['Pins','SDA '+pinShort(C,b,bus.sda).replace('GPIO ','')+' · SCL '+pinShort(C,b,bus.scl).replace('GPIO ','')],
        out?['Driver',c.type==='motor'?(p.driver==='brushed'?'MOSFET':'PWM ESC'):c.type==='joint'?'PWM servo':p.driver==='pwm'?'PWM servo':'digital']:drv&&['Driver',drv.label.replace(/ \(.*/,'')]),
      dsec('Connection',conn));
    if(c.type==='sensor')put(box,dsec(SENSOR_KINDS[c.kind],partSettings(c)),physSection(c),ddanger(removePart(c,'sensor')));
    else put(box,dsec('On the airframe',el('div',{class:'cpart-head'},el('span',{class:'dmeta',text:'Its place, thrust and physics are set in Airframe.'}),UI.button({class:'btn sm',text:'Open in Airframe',onclick:()=>{bench.want=false;UI_PANELS.editor.select('air');setEditMode(true);selectComp(c.id);}}))));
    return;
  }
  // a board
  const ground=view.id==='ground',b=ground?C.ground:C.boards.find(b=>b.id===view.id);if(!b){detClose();return;}
  const K=BOARD_KINDS[b.kind],budget=ground?groundBudget():boardBudget(b),body=!ground&&boardBodyOf(b);detTitle(b.name);
  const duties=ground?'command module':runsFormulas(b)?b.tasks.map(t=>TASKS[t].label).join(', ')||'none':appsOn(b).map(a=>a.name).join(', ')||'no apps';
  put(box,ground?null:planIssues(b),
    dfacts(['Chip',K.label],['Runs',duties],...(ground||runsFormulas(b)?[['Load',pct(budget.load)+' of '+(K.cores>1?'one core':'its core'),budget.load>0.8?'bad':''],K.mcu&&['Program',budget.memKB.toFixed(1)+' of '+K.ramKB+' KB']]:[]),!ground&&['On the airframe',body?gramsOf(body):'no',body?'':'muted']),
    dactions(UI.button({class:'btn sm primary',id:ground?'ginst':'binst-'+b.id,text:'Install / export…',onclick:()=>openInstall(ground?'ground':b)})),
    dsec('Board',hardwareField('Name',nameInput(b.name,'Board name',ground?'gname':'bname-'+b.id,v=>renameComputer(D=>{const x=ground?D.ground:D.boards.find(x=>x.id===b.id);if(x)x.name=(v||K.label).slice(0,24);}))),
      hardwareField('Computer',hardwareSelect('Board kind',ground?'gkind':'bkind-'+b.id,Object.entries(BOARD_KINDS).filter(([,K])=>ground||!K.groundOnly).map(([id,K])=>[id,K.label]),b.kind,v=>{const D=JSON.parse(JSON.stringify(computers()));(ground?D.ground:D.boards.find(x=>x.id===b.id)).kind=v;setComputers(D,'kind');})),
      el('p',{class:'hint',text:K.note.charAt(0).toUpperCase()+K.note.slice(1)+'.'})));
  if(ground){
    const w=el('div',{});mountHardware(w,'[data-hw-role="ground"]');
    if(!groundHardware(C)){const g=hardwareOverview(C,cfg.comps).groups.find(g=>g.id==='ground');w.append(el('p',{class:'hint',text:g?.rows.map(r=>r.connection+' · '+r.note).join(' ')||'Host inputs and USB serial.'}));}
    put(box,dsec('Wiring',w),ddanger(el('p',{class:'hint',text:'The command module is part of the pilot link: it stays.'})));
    return;
  }
  put(box,boardRunsSection(b));
  {   // its wires (each opens its device), then its own hardware settings
    const mine=benchWires(C,cfg.comps).filter(w=>w.a.key==='board:'+b.id||w.b.key==='board:'+b.id);
    const own=[...mounted('[data-hw-board="'+b.id+'"]'),...(boardOf('tlm')?.id===b.id?mounted('[data-hw-role="radio"]'):[])],presets=own.filter(n=>n.classList.contains('hw-preset-btn'));
    for(const n of presets)n.classList.add('sm');
    put(box,dsec('Wiring',mine.length?null:el('p',{class:'hint',text:'Nothing wired to it yet.'}),
      mine.length?UI.details({class:'dsub',title:'Its wires ('+mine.length+')',open:HW_UI.open.has('d:wires'+b.id),onToggle:o=>{o?HW_UI.open.add('d:wires'+b.id):HW_UI.open.delete('d:wires'+b.id);}},...wireRows(mine)):null,
      ...cardFolds(own.filter(n=>!presets.includes(n)),'b'+b.id,b.name),presets.length?dactions(...presets):null));
  }
  put(box,dsec('Power',boardSupplyFields(C,b),el('p',{class:'hint',text:'Below 5 V (a 1S pack) a boost converter steps it up, above it a regulator (BEC) steps it down. Draw: what it takes at 5 V; the defaults are typical, put your own measurement in.'})),
    boardBodySection(b));
  if(typeof boardModelSection==='function'&&(ESP_PROFILES[b.kind]||b.kind.startsWith('pi')))put(box,dfold('pinout'+b.id,'Pinout & board model',boardModelSection(C,b,editWiring)));
  const only=K.mcu&&C.boards.filter(x=>BOARD_KINDS[x.kind].mcu).length===1;
  const blocked=C.boards.length<2?'Keep at least one onboard board.':only?'Add another microcontroller before deleting the last one.':'';
  const danger=ddanger(UI.button({class:'btn sm danger',id:'bdel-'+b.id,text:'Delete board',disabled:blocked?'disabled':null,title:blocked,onclick:()=>{COMP.confirm=b.id;renderHardware();renderComputerDetail();}}),blocked?el('span',{class:'hint',text:blocked}):null);
  if(COMP.confirm===b.id)danger.append(el('div',{class:'computer-confirm',role:'alert'},el('p',{text:'Delete “'+b.name+'”? Its duties and device connections go with it.'}),UI.button({class:'btn sm',text:'Cancel',onclick:()=>{COMP.confirm=null;renderHardware();renderComputerDetail();}}),UI.button({class:'btn sm danger',id:'boardDeleteConfirm',text:'Delete board',onclick:()=>{
    const D=JSON.parse(JSON.stringify(computers()));D.boards=D.boards.filter(x=>x.id!==b.id);if(b.tasks.includes('core'))D.unassignedCore=true;
    if(D.wiring){delete D.wiring.boards?.[b.id];for(const p of Object.values(D.wiring.parts||{}))if(p.board===b.id){p.board=null;p.pin=-1;}}
    COMP.focus=null;setComputers(D,'remove');benchFocus(null);
  }})));
  put(box,danger);
}
function openFormulaEditor(key){const dlg=$('#formulaDlg');if(key)$('#formulaSearch').value='';renderUndo();if(!dlg.open)dlg.showModal();showFormula(key||COMP.formula||formulaGroups()[0].keys[0]);}
function showFormula(key){
  if(String(key).startsWith('prog:')){showProgram(key.slice(5));return;}
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
// A board's settings: what it runs (formulas, WebAssembly apps, native apps), and its apps.
function boardRunsSection(b){
  const K=BOARD_KINDS[b.kind],runs=boardRuns(b),box=el('section',{class:'dsec board-runs'},el('h3',{class:'dsec-h',text:'Software'}));
  if(runsFormulas(b))box.append(el('div',{class:'chips'},...(b.tasks.length?b.tasks.map(t=>UI.button({class:'chip',title:'Open the '+TASKS[t].label+' duty',text:TASKS[t].label+' · '+boardTaskHz(b,t)+' Hz',onclick:()=>compFocus({kind:'task',id:t})})):[el('span',{class:'hint',text:'No duties: give it one under Duties.'})])));
  const msg=UI.status({class:'law-err board-runs-err',role:'status'});
  const set=next=>{
    const lost=runs.filter(r=>!next.includes(r));
    if(lost.includes('formulas')&&(b.tasks.length||programs().some(p=>p.board===b.id))){msg.textContent='Move its duties'+(programs().some(p=>p.board===b.id)?' and programs':'')+' to another board first: '+[...b.tasks.map(t=>TASKS[t].label),...programs().filter(p=>p.board===b.id).map(p=>p.name)].join(', ')+'.';msg.className='ui-status law-err board-runs-err on';renderRunsChoice();return;}
    const gone=appsOn(b).filter(a=>!next.includes(APP_KINDS[a.kind].runtime));
    if(gone.length){msg.textContent='Take its '+gone.map(a=>a.name).join(', ')+' off first (Apps on this board, below).';msg.className='ui-status law-err board-runs-err on';renderRunsChoice();return;}
    const D=JSON.parse(JSON.stringify(computers())),x=D.boards.find(x=>x.id===b.id);x.runs=next;setComputers(D,'runs');
  };
  const choice=el('div',{class:'board-runs-choice'});
  const renderRunsChoice=()=>{
    if(K.mcu){const sel=UI.select({'aria-label':'What the board runs',id:'bruns-'+b.id},...['formulas','wasm'].map(r=>el('option',{value:r,text:RUNTIMES[r].label})));sel.value=runs[0];sel.addEventListener('change',()=>set([sel.value]));choice.replaceChildren(sel,el('p',{class:'hint',text:RUNTIMES[runs[0]].what.charAt(0).toUpperCase()+RUNTIMES[runs[0]].what.slice(1)+'. A microcontroller runs one or the other: it decides the firmware it gets.'}));}
    else choice.replaceChildren(...Object.entries(RUNTIMES).map(([r,R])=>el('label',{class:'check'},Object.assign(UI.input({type:'checkbox',value:r,'data-runs':r}),{checked:runs.includes(r),onchange:e=>{const next=Object.keys(RUNTIMES).filter(k=>k===r?e.target.checked:runs.includes(k));if(!next.length){e.target.checked=true;return;}set(next);}}),el('span',{text:R.label+' · '+R.what}))));
  };
  renderRunsChoice();box.append(choice,msg);
  const appRt=runs.filter(r=>r!=='formulas');
  if(appRt.length){
    const mine=appsOn(b),list=el('div',{class:'board-app-list'});
    for(const a of mine)list.append(UI.card({class:'board-assignment','data-board-app':a.id},el('div',{},el('b',{text:a.name}),el('span',{class:'computer-meta',text:APP_KINDS[a.kind].label+' · writes '+a.writes.topic})),
      el('div',{class:'law-actions'},UI.button({class:'btn sm',text:'Open',onclick:()=>{$('#computerDlg').close();openAppManager(a.id);}}),UI.button({class:'btn sm',text:'Take off','data-app-off':a.id,onclick:()=>boardApp(b,a.id,false)}))));
    if(!mine.length)list.append(el('p',{class:'hint',text:'No apps on it yet.'}));
    const fits=apps().filter(a=>appRt.includes(APP_KINDS[a.kind].runtime)&&!mine.includes(a));
    const pick=UI.select({'aria-label':'An app to put on this board',id:'bapp-'+b.id},el('option',{value:'',text:fits.length?'Put an app on it…':'No other apps for it'}),...fits.map(a=>{const at=appBoard(a);return el('option',{value:a.id,text:a.name+' · '+APP_KINDS[a.kind].label+(at?' (now on '+at.name+')':'')});}));
    pick.disabled=!fits.length;pick.addEventListener('change',()=>{if(pick.value)boardApp(b,pick.value,true);});
    const kind=appRt.includes('wasm')?'wasm':'python';
    box.append(el('h4',{text:'Apps on this board'}),list,el('div',{class:'law-actions'},pick,UI.button({class:'btn sm',text:'+ New app here','data-new-app':b.id,onclick:()=>{$('#computerDlg').close();newApp(kind,b.id);}})));
    if(K.mcu)box.append(el('p',{class:'hint',text:'On the drone it needs the app-board firmware (the WebAssembly runtime); see Install / export. Until then its apps run in the simulator.'}));
  }
  return box;
}
// Put an app on a board (off any other: an app runs on one board), or take it off.
function boardApp(b,id,on){
  const D=JSON.parse(JSON.stringify(computers()));
  for(const x of D.boards)if(on||x.id===b.id)x.apps=(x.apps||[]).filter(k=>k!==id);
  if(on){const x=D.boards.find(x=>x.id===b.id);x.apps=[...(x.apps||[]),id];}
  setComputers(D,'apps');
}
