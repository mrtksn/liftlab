'use strict';
// Shared UI primitives. Keep simulator/domain state in the feature modules.

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
    const cls = 'chip' + (c.go ? ' ui-button' : '') + (c.tone ? ' ' + c.tone : ''); if (n.className !== cls) n.className = cls;
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
  const btn = UI.button( { type: 'button', class: 'btn mb-btn' + (o.cls ? ' ' + o.cls : ''), 'aria-haspopup': 'menu', 'aria-expanded': 'false', 'aria-label': o.label || null, title: o.title || null, 'data-focus-key': o.key || null },
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
      const b = UI.button( { type: 'button', role: 'menuitem', tabindex: '-1', class: 'mb-item' + (it.cur ? ' cur' : '') }, el('span', { text: it.label }), it.hint ? el('span', { class: 'mb-hint', text: it.hint }) : null);
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

function numField(id, d, get, set) {
  const frac = d.hard || d.u === '%' || (d.min === 0 && d.max === 1);
  const k = d.k || 1, lo = frac ? (d.hmin ?? d.min) : d.min >= 0 ? 0 : -Infinity, hi = frac ? (d.hmax ?? d.max) : Infinity, pos = !frac && d.min > 0, inK = x => +(x * k).toFixed(6);
  const show1 = x => String(+(x * k).toFixed(d.dp));
  const onGrid = x => Math.abs(x / d.step - Math.round(x / d.step)) < 1e-6;   // (a min off the step grid would move where the arrow keys step to)
  const num = UI.input( { type: 'number', class: 'num', id: id + '-n', step: String(inK(d.step)), min: isFinite(lo) && onGrid(lo) ? String(inK(lo)) : null, max: isFinite(hi) ? String(inK(hi)) : null, 'aria-label': `${d.label}${d.u ? ' in ' + d.u : ''}`, inputmode: 'decimal' });
  const rng = UI.input( { type: 'range', id, min: d.min, max: d.max, step: d.step });
  const note = el('span', { class: 'clampnote', role: 'status' }); let noteT = 0;
  const say = t => { clearTimeout(noteT); note.textContent = t; if (t) noteT = setTimeout(() => { note.textContent = ''; }, 2500); };
  const show = v => { num.value = show1(v); rng.value = String(v); };
  show(get());
  rng.addEventListener('input', () => { const v = parseFloat(rng.value); num.value = show1(v); set(v); });
  num.addEventListener('input', () => {
    const t = num.value; let v = parseFloat(t) / k; if (t === '' || !isFinite(v)) return;
    let why = '';
    if (pos && v <= 0) { say('must be above 0'); return; }
    if (v > hi) { v = hi; why = `max ${show1(hi)}`; } else if (v < lo) { v = lo; why = `min ${show1(lo)}`; }
    if (d.int && Math.round(v) !== v) { v = Math.round(v); why = why || 'whole numbers'; }
    if (!why && (v > d.max || v < d.min)) why = 'past the slider';
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

// Factories return native elements, so feature modules can update readouts without rerendering a tree.
// Content and callbacks belong to callers; appearance and interaction belong here.
const UI = (() => {
  let nextId = 0;
  const classes = (...values) => values.filter(Boolean).join(' ');
  const control = (tag, attrs = {}, ...children) => el(tag, {...attrs, class:classes('ui-control', attrs.class)}, ...children);
  const button = (attrs = {}, ...children) => {
    const {variant, size, ...native} = attrs;
    return el('button', {type:'button', ...native, class:classes('ui-button', native.class, variant && 'btn '+variant, size && 'btn-'+size)}, ...children);
  };
  const field = ({label, hint, class:cls, layout='stack', ...attrs}, input, ...children) => {
    const target = /^(INPUT|SELECT|TEXTAREA)$/.test(input.tagName)?input:input.querySelector('input,select,textarea');
    let id = target?.id;
    const isInput = !!target;
    if (isInput && !id) target.id = id = 'ui-field-'+(++nextId);
    const caption = el(isInput?'label':'span', {class:'ui-field__label', for:isInput?id:null}, label);
    const help = hint ? el('p', {class:'hint ui-field__hint', id:'ui-help-'+(++nextId)}, hint) : null;
    if (help && isInput) target.setAttribute('aria-describedby', classes(target.getAttribute('aria-describedby'), help.id));
    return el('div', {...attrs, class:classes('ui-field','ui-field--'+layout,cls)}, caption,input,...children,help);
  };
  const choice = ({label,id,options,value,onChange,commit=false,...attrs}) => {
    const sel = control('select', {id,'aria-label':label,...attrs});
    for (const option of options) {
      const [v,text,disabled] = Array.isArray(option)?option:[option.value,option.label,option.disabled];
      sel.append(el('option',{value:String(v),text,disabled:disabled?'disabled':null}));
    }
    sel.value = String(value);
    if(commit) commitSelect(sel,onChange);
    else if(onChange) sel.addEventListener('change',()=>onChange(sel.value));
    return sel;
  };
  const card = (attrs={},...children) => el('article',{...attrs,class:classes('ui-card',attrs.class)},...children);
  const section = (attrs={},...children) => el('section',{...attrs,class:classes('ui-section',attrs.class)},...children);
  const status = (attrs={},...children) => el('p',{role:'status',...attrs,class:classes('ui-status',attrs.class)},...children);
  const details = ({title,open=false,onToggle,class:cls,...attrs},...children) => {
    const node = el('details',{...attrs,class:classes('ui-disclosure',cls)},el('summary',{class:'ui-disclosure__trigger'},...(Array.isArray(title)?title:[title])),...children);
    node.open=!!open;
    if(onToggle) node.addEventListener('toggle',()=>onToggle(node.open));
    return node;
  };
  const bindDisclosure = (trigger,body,{open=false,onToggle,beforeToggle}={}) => {
    trigger.classList.add('ui-disclosure__trigger');
    if(body) {
      body.classList.add('ui-disclosure__body');
      if(!body.id) body.id='ui-disclosure-'+(++nextId);
      trigger.setAttribute('aria-controls',body.id);
    }
    const setOpen = (value,notify=true) => {
      trigger.setAttribute('aria-expanded',String(!!value));
      if(body) body.hidden=!value;
      if(notify && onToggle) onToggle(!!value);
    };
    setOpen(open,false);
    trigger.addEventListener('click',()=>{
      if(beforeToggle && beforeToggle()===false)return;
      setOpen(trigger.getAttribute('aria-expanded')!=='true');
    });
    return {setOpen};
  };
  const tabs = ({root,label,items,initial,storageKey,onChange}) => {
    if(!root || !items.length)throw new Error('Tabs require a root and items');
    const enabled=items.filter(i=>!i.disabled);
    let active=enabled.some(i=>i.key===initial)?initial:enabled[0]?.key;
    if(!active)throw new Error('Tabs require an enabled item');
    root.classList.add('ui-tabs');root.setAttribute('role','tablist');root.setAttribute('aria-label',label);
    root.replaceChildren();
    const entries=items.map(item=>{
      const pane=document.getElementById(item.panel);
      if(!pane)throw new Error('Missing tab panel '+item.panel);
      const text=el('span',{},item.label,item.badge?el('span',{class:'count',id:item.badge.id,hidden:''}):null);
      const icon=item.icon?el('span',{class:'ui-tab__icon',html:item.icon,'aria-hidden':'true'}):null;
      const trigger=button({class:'ui-tab',role:'tab',id:item.id,'aria-controls':item.panel,'aria-label':item['aria-label'],title:item.title,disabled:item.disabled?'disabled':null},icon,text);
      pane.setAttribute('role','tabpanel');pane.setAttribute('aria-labelledby',item.id);pane.classList.add('ui-tabpanel');
      trigger.addEventListener('click',()=>select(item.key));root.append(trigger);
      return {item,trigger,pane};
    });
    function select(key,notify=true) {
      active=enabled.some(i=>i.key===key)?key:enabled[0].key;
      for(const {item,trigger,pane} of entries){const selected=item.key===active;trigger.setAttribute('aria-selected',String(selected));trigger.tabIndex=selected?0:-1;pane.hidden=!selected;}
      if(storageKey)try{localStorage.setItem(storageKey,active);}catch(e){}
      if(notify && onChange)onChange(active);
      return active;
    }
    root.addEventListener('keydown',e=>{
      const index=enabled.findIndex(i=>i.id===e.target.id);if(index<0 || e.altKey || e.ctrlKey || e.metaKey)return;
      const rtl=getComputedStyle(root).direction==='rtl';
      const delta=e.key==='ArrowRight'?(rtl?-1:1):e.key==='ArrowLeft'?(rtl?1:-1):0;
      const next=delta?(index+delta+enabled.length)%enabled.length:e.key==='Home'?0:e.key==='End'?enabled.length-1:-1;
      if(next<0)return;e.preventDefault();select(enabled[next].key);entries.find(x=>x.item.key===active).trigger.focus();
    });
    // Reading saved state must happen before select() persists an initial choice.
    const restore=()=>{let saved=initial;try{if(storageKey)saved=localStorage.getItem(storageKey)||initial;}catch(e){}return select(saved);};
    // Initial DOM only; bootstrap invokes callbacks after all feature modules are loaded.
    for(const {item,trigger,pane} of entries){const selected=item.key===active;trigger.setAttribute('aria-selected',String(selected));trigger.tabIndex=selected?0:-1;pane.hidden=!selected;}
    return {root,select,restore,get active(){return active;}};
  };
  const hydrate = root => {
    for(const node of root.querySelectorAll('button,input,select,textarea,details,.sec')) {
      if(node.tagName==='BUTTON')node.classList.add('ui-button');
      else if(/^(INPUT|SELECT|TEXTAREA)$/.test(node.tagName))node.classList.add('ui-control');
      else if(node.tagName==='DETAILS')node.classList.add('ui-disclosure');
      else node.classList.add('ui-section');
    }
  };
  return Object.freeze({button,field,choice,card,section,status,details,bindDisclosure,tabs,hydrate,
    input:(attrs,...kids)=>control('input',attrs,...kids),select:(attrs,...kids)=>control('select',attrs,...kids),textarea:(attrs,...kids)=>control('textarea',attrs,...kids)});
})();
