'use strict';
// The AI tab (left panel). Until a model is connected it's one card: connect a model. Then it's the chat: a list of
// threads (your chats, and a Triggers group, each trigger with its own thread), a thread with the model and the tokens
// it used in its header, and Settings (the connection, to change or remove, and how the agent behaves).


const aiUi = { built: false, view: 'list', editConn: false, draft: null, msg: '', msgTone: '', models: [] };
const kfmt = n => n >= 1e6 ? (n / 1e6).toFixed(1) + 'M' : n >= 1000 ? (n / 1000).toFixed(1) + 'k' : String(n);
const ago = ms => { const s = (Date.now() - ms) / 1000; return s < 60 ? 'just now' : s < 3600 ? Math.round(s / 60) + ' min ago' : s < 86400 ? Math.round(s / 3600) + ' h ago' : Math.round(s / 86400) + ' d ago'; };
const epLabel = () => (AGENT_ENDPOINTS[agent.cfg.endpoint] || {}).label || 'Custom';
const iconBtn = (label, text, onclick, extra = {}) => el('button', { class: 'ai-icon', type: 'button', 'aria-label': label, title: label, text, onclick, ...extra });

function agentRender() {   // the whole tab, for the view it's on
  const P = $('#paneAi'); if (!P) return;
  aiUi.built = true; P.textContent = '';
  if (!agent.cfg.connected || aiUi.editConn) { P.append(connView()); agentUi(); return; }
  const v = aiUi.view, th = threadOf(agent.cur);
  P.append(v === 'settings' ? settingsView() : v === 'chat' && th ? chatView(th) : listView());
  agentUi();
}

/* ───────── connecting a model (first run, or editing the connection) ───────── */
function connView() {
  const C = agent.cfg, D = aiUi.draft || (aiUi.draft = { endpoint: C.endpoint, url: C.url, model: C.model, key: agent.key, remember: C.remember });
  const E = () => AGENT_ENDPOINTS[D.endpoint] || AGENT_ENDPOINTS.custom;
  const box = el('div', { class: 'ai-view ai-setup' });
  box.append(el('div', { class: 'ai-hero' },
    el('div', { class: 'ai-mark', 'aria-hidden': 'true', text: '✦' }),
    el('h2', { text: aiUi.editConn ? 'Change the connection' : 'Connect an AI model' }),
    el('p', { text: 'An agent that can fly the simulated drone, rebuild it, tune its formulas and test it, and that triggers can call when something happens. Any OpenAI-compatible chat API works; pick a model that can call tools.' })));
  const ep = el('div', { class: 'ai-eps', role: 'radiogroup', 'aria-label': 'Provider' });
  for (const [k, X] of Object.entries(AGENT_ENDPOINTS)) {
    const b = el('button', { type: 'button', class: 'ai-ep', role: 'radio', 'aria-checked': String(k === D.endpoint), text: X.label.replace(' (this computer)', ''), title: X.label });
    if (X.noKey) b.append(el('small', { text: 'on this computer' }));
    b.addEventListener('click', () => { D.endpoint = k; if (X.url) D.url = X.url; else if (k === 'custom') D.url = ''; D.model = X.model || ''; aiUi.models = []; aiUi.msg = ''; agentRender(); });
    ep.append(b);
  }
  const field = (label, input, note) => el('label', { class: 'ai-field' }, el('span', { class: 'lbl', text: label }), input, note ? el('span', { class: 'ai-fnote', text: note }) : null);
  const url = el('input', { type: 'url', id: 'aiUrl', value: D.url, placeholder: 'https://…/v1', autocomplete: 'off', spellcheck: 'false' }); url.addEventListener('input', () => { D.url = url.value.trim(); });
  const key = el('input', { type: 'password', id: 'aiKey', value: D.key, placeholder: E().noKey ? 'not needed' : 'sk-…', autocomplete: 'off', spellcheck: 'false' }); key.addEventListener('input', () => { D.key = key.value.trim(); });
  const model = el('input', { type: 'text', id: 'aiModel', value: D.model, list: 'aiModels', autocomplete: 'off', spellcheck: 'false', placeholder: 'model name' }); model.addEventListener('input', () => { D.model = model.value.trim(); });
  const dl = el('datalist', { id: 'aiModels' }); for (const m of aiUi.models.slice(0, 500)) dl.append(el('option', { value: m }));
  const rem = el('input', { type: 'checkbox', id: 'aiRemember' }); rem.checked = D.remember; rem.addEventListener('change', () => { D.remember = rem.checked; });
  const msg = el('p', { class: 'hint ' + (aiUi.msgTone || ''), id: 'aiConnMsg', role: 'status', text: aiUi.msg });
  const test = async () => {
    const save = { url: agent.cfg.url, key: agent.key }; agent.cfg.url = D.url; agent.key = D.key;
    try { aiUi.models = await agentModels(); return ''; } catch (e) { return e.message; } finally { agent.cfg.url = save.url; agent.key = save.key; }
  };
  const load = el('button', { class: 'btn', type: 'button', text: 'Load models', id: 'aiLoad', onclick: async () => {
    aiUi.msg = 'Asking…'; aiUi.msgTone = ''; msg.textContent = aiUi.msg; const err = await test();
    aiUi.msg = err ? 'Couldn\'t list the models: ' + err : `${aiUi.models.length} models: pick one in the Model field.`; aiUi.msgTone = err ? 'bad' : 'good'; agentRender(); } });
  const connect = el('button', { class: 'btn primary', type: 'button', id: 'aiConnect', text: aiUi.editConn ? 'Save' : 'Connect', onclick: async () => {
    if (!D.url) { aiUi.msg = 'The base URL is missing.'; aiUi.msgTone = 'bad'; return agentRender(); }
    if (!D.model) { aiUi.msg = 'Pick a model (Load models lists them).'; aiUi.msgTone = 'bad'; return agentRender(); }
    if (!D.key && !E().noKey && D.endpoint !== 'custom') { aiUi.msg = 'The API key is missing.'; aiUi.msgTone = 'bad'; return agentRender(); }
    connect.disabled = true; aiUi.msg = 'Checking the connection…'; aiUi.msgTone = ''; msg.textContent = aiUi.msg; msg.className = 'hint';
    const err = await test();
    if (err && !aiUi.forceOk) { aiUi.msg = `Couldn't reach it: ${err}. Check the URL and key, or save it anyway (some servers don't list their models).`; aiUi.msgTone = 'bad'; aiUi.forceOk = true; connect.disabled = false; connect.textContent = 'Save anyway'; msg.textContent = aiUi.msg; msg.className = 'hint bad'; return; }
    Object.assign(agent.cfg, { connected: true, endpoint: D.endpoint, url: D.url, model: D.model, remember: D.remember }); agent.key = D.key; agentSave();
    aiUi.draft = null; aiUi.msg = ''; aiUi.forceOk = false; aiUi.editConn = false; aiUi.view = agent.threads.some(t => t.kind === 'chat') ? 'list' : 'chat';
    if (aiUi.view === 'chat') agent.cur = threadNew().id;
    agentRender(); } });
  const form = el('div', { class: 'ai-card' }, el('span', { class: 'lbl', text: 'Provider' }), ep,
    field('Base URL', url, 'OpenAI-compatible: the chat API lives at …/chat/completions'),
    E().noKey ? null : field('API key', key, 'Sent only to this URL, from this page.'),
    E().noKey ? null : el('label', { class: 'check', for: 'aiRemember' }, rem, 'Remember the key in this browser (otherwise until the tab closes)'),
    field('Model', el('div', { class: 'ai-row' }, model, load, dl)),
    msg, el('div', { class: 'ai-row' }, connect, aiUi.editConn ? el('button', { class: 'btn', type: 'button', text: 'Cancel', onclick: () => { aiUi.editConn = false; aiUi.draft = null; aiUi.msg = ''; agentRender(); } }) : null),
    E().noKey ? el('p', { class: 'hint', text: 'A server on this computer must allow requests from a web page (CORS). For Ollama, start it with OLLAMA_ORIGINS=*.' }) : null);
  box.append(form, el('p', { class: 'hint ai-foot', text: 'It works on the simulator only: this page has no link to a real drone.' }));
  return box;
}

/* ───────── the thread list ───────── */
function topBar(left, title, right) { return el('div', { class: 'ai-top' }, left, el('div', { class: 'ai-top-t' }, title), right); }
function modelChip() { return el('span', { class: 'ai-chip', title: `${epLabel()} · ${agent.cfg.url}`, text: agent.cfg.model }); }
function listView() {
  const box = el('div', { class: 'ai-view' });
  box.append(topBar(null, el('div', {}, el('b', { text: 'AI agent' }), el('div', { class: 'ai-sub' }, modelChip(), el('span', { id: 'aiUse' }))),
    iconBtn('Settings', '⚙', () => { aiUi.view = 'settings'; agentRender(); }, { id: 'aiSettings' })));
  const list = el('div', { class: 'ai-list' });
  const newChat = el('button', { class: 'btn primary ai-new', type: 'button', id: 'aiNewChat', text: '+ New chat', onclick: () => { agent.cur = threadNew().id; aiUi.view = 'chat'; agentRender(); setTimeout(() => { const i = $('#aiInput'); if (i) i.focus(); }); } });
  list.append(newChat);
  const chats = agent.threads.filter(t => t.kind === 'chat' && (t.msgs.length || t.feed.length)).sort((a, b) => b.updated - a.updated);
  list.append(el('div', { class: 'ai-group' }, el('span', { class: 'lbl', text: 'Chats' })));
  if (!chats.length) list.append(el('p', { class: 'hint ai-none', text: 'No chats yet.' }));
  for (const t of chats) list.append(threadRow(t, t.title, `${ago(t.updated)}${t.tokens.in ? ' · ' + kfmt(t.tokens.in + t.tokens.out) + ' tokens' : ''}`));
  list.append(el('div', { class: 'ai-group' }, el('span', { class: 'lbl', text: 'Triggers' }),
    el('button', { class: 'ai-link', type: 'button', id: 'aiNewTrig', text: '+ New trigger', onclick: () => {
      if (agent.triggers.length >= 12) return;
      const T = { id: 't' + Date.now().toString(36), kind: 'crash', value: null, expr: TRIGGER_KINDS.expr.expr, msg: 'Find out why and suggest a fix.', gap: 10, keepFlying: false, on: true, fired: 0, last: -1e9, was: false };
      agent.triggers.push(T); agentSave(); const th = threadForTrigger(T); agentSaveThreads(); agent.cur = th.id; aiUi.view = 'chat'; aiUi.trigOpen = true; agentRender(); } })));
  if (!agent.triggers.length) list.append(el('p', { class: 'hint ai-none', text: 'Triggers ask the AI by themselves when something happens: a crash, a low battery, off target, or a condition of yours.' }));
  for (const T of agent.triggers) {
    const th = threadForTrigger(T);
    const on = el('input', { type: 'checkbox', class: 'ai-switch', 'aria-label': 'On', title: T.on ? 'On' : 'Off' }); on.checked = T.on;
    on.addEventListener('click', e => e.stopPropagation()); on.addEventListener('change', () => { T.on = on.checked; T.was = false; agentSave(); agentRender(); });
    list.append(threadRow(th, triggerText(T), `${T.on ? 'on' : 'off'}${T.fired ? ` · fired ${T.fired}×` : ''}${th.feed.length ? ' · ' + ago(th.updated) : ''}`, on, () => { agent.triggers = agent.triggers.filter(x => x !== T); agentSave(); }));
  }
  box.append(list);
  return box;
}
function threadRow(t, title, meta, lead, onDelete) {
  const row = el('div', { class: 'ai-thread' + (t.kind === 'trigger' ? ' trig' : '') + (agent.turn === t ? ' busy' : ''), 'data-thread': t.id });
  const open = el('button', { type: 'button', class: 'ai-thread-open', onclick: () => { agent.cur = t.id; aiUi.view = 'chat'; aiUi.trigOpen = false; agentRender(); } },
    t.kind === 'trigger' ? el('span', { class: 'ai-bolt', 'aria-hidden': 'true', text: '⚡' }) : null,
    el('span', { class: 'ai-thread-t', text: title }), el('span', { class: 'ai-thread-m', text: meta }));
  const del = iconBtn(t.kind === 'trigger' ? 'Delete this trigger' : 'Delete this chat', '×', () => { if (onDelete) onDelete(); threadDelete(t.id); agentRender(); }, { class: 'ai-icon del' });
  if (lead) row.append(lead); row.append(open, del);
  return row;
}

/* ───────── a thread ───────── */
const AI_SUGGEST = ['Fly a 2 m square at 2 m height and tell me how well it held the corners', 'Add a wing and see how it flies in 5 m/s wind', 'Make the position control softer', 'Find out why it crashes when M2 stops'];
function chatView(th) {
  const box = el('div', { class: 'ai-view ai-chatv' });
  const T = th.kind === 'trigger' ? agent.triggers.find(x => x.id === th.triggerId) : null;
  box.append(topBar(iconBtn('All chats', '‹', () => { aiUi.view = 'list'; agentRender(); }, { id: 'aiBack' }),
    el('div', {}, el('b', { class: 'ai-title', text: T ? '⚡ ' + triggerText(T) : th.title }),
      el('div', { class: 'ai-sub' }, modelChip(), el('span', { id: 'aiThreadUse' }))),
    iconBtn('Settings', '⚙', () => { aiUi.view = 'settings'; agentRender(); })));
  if (T) {
    const d = el('details', { class: 'ai-trigcard' }); d.open = !!aiUi.trigOpen || !th.feed.length;
    d.addEventListener('toggle', () => { aiUi.trigOpen = d.open; });
    d.append(el('summary', {}, el('span', { text: 'When it fires' }), el('span', { class: 'ai-thread-m', id: 'aiFired', text: T.fired ? `fired ${T.fired}×` : 'not fired yet' })), triggerEditor(T, th));
    box.append(d);
  }
  box.append(el('div', { class: 'ai-feed', id: 'aiFeed', role: 'log', 'aria-live': 'polite', 'aria-label': 'Conversation with the AI agent' }));
  const input = el('textarea', { id: 'aiInput', class: 'ai-input', rows: '2', placeholder: T ? 'Ask about what it found, or tell it what to do next…' : 'Ask it to fly, build, tune or test…', 'aria-label': 'Message to the AI agent' });
  const send = el('button', { class: 'btn primary', type: 'button', id: 'aiSend', text: 'Send' });
  const stop = el('button', { class: 'btn', type: 'button', id: 'aiStop', text: 'Stop', onclick: agentStop });
  const go = () => { const t = input.value.trim(); if (!t || agent.busy) return; input.value = ''; agentAsk(t, { thread: th }); };
  send.addEventListener('click', go);
  input.addEventListener('keydown', e => { if (e.key === 'Enter' && !e.shiftKey && !e.isComposing) { e.preventDefault(); go(); } });
  box.append(el('div', { class: 'ai-compose' }, input, el('div', { class: 'ai-row' }, el('span', { class: 'ai-status', id: 'aiHint' }), stop, send)));
  setTimeout(() => agentRenderFeed(th));
  return box;
}
function triggerEditor(T, th) {
  const K = TRIGGER_KINDS[T.kind] || TRIGGER_KINDS.crash, ch = () => { agentSave(); th.title = triggerText(T); const t = document.querySelector('.ai-title'); if (t) t.textContent = '⚡ ' + th.title; };
  const box = el('div', { class: 'ai-trig' });
  const kind = el('select', { 'aria-label': 'When' });
  for (const [k, D] of Object.entries(TRIGGER_KINDS)) { const o = el('option', { value: k, text: D.label }); if (k === T.kind) o.selected = true; kind.append(o); }
  kind.addEventListener('change', () => { T.kind = kind.value; T.value = TRIGGER_KINDS[T.kind].value ?? null; T.was = false; ch(); aiUi.trigOpen = true; agentRender(); });
  const val = K.value != null ? el('input', { type: 'number', class: 'ai-num', value: T.value ?? K.value, 'aria-label': 'Value' }) : null;
  if (val) val.addEventListener('change', () => { T.value = +val.value; ch(); });
  const expr = T.kind === 'expr' ? el('input', { type: 'text', class: 'ai-expr', value: T.expr || '', spellcheck: 'false', 'aria-label': 'Expression' }) : null;
  const err = el('span', { class: 'ai-err' });
  if (expr) { const chk = () => { const f = triggerTest(T); err.textContent = f ? '' : triggerFn.get(T.id).err; }; expr.addEventListener('change', () => { T.expr = expr.value; ch(); chk(); }); chk(); }
  const msg = el('input', { type: 'text', class: 'ai-tmsg', value: T.msg || '', placeholder: 'What to tell the AI', 'aria-label': 'Message to the AI' });
  msg.addEventListener('change', () => { T.msg = msg.value; ch(); });
  const on = el('input', { type: 'checkbox' }); on.checked = T.on; on.addEventListener('change', () => { T.on = on.checked; T.was = false; ch(); });
  const gap = el('input', { type: 'number', class: 'ai-num', value: T.gap ?? 10, min: '0', 'aria-label': 'Gap, seconds' }); gap.addEventListener('change', () => { T.gap = Math.max(0, +gap.value || 0); ch(); });
  const kf = el('input', { type: 'checkbox' }); kf.checked = !!T.keepFlying; kf.addEventListener('change', () => { T.keepFlying = kf.checked; ch(); });
  box.append(...[el('div', { class: 'ai-row' }, kind, val), expr ? el('div', { class: 'ai-row' }, expr) : null, err,
    el('label', { class: 'ai-field' }, el('span', { class: 'lbl', text: 'Tell the AI' }), msg),
    el('div', { class: 'ai-row small' }, el('label', { class: 'check' }, on, 'on'), el('label', {}, 'again after ', gap, ' s'), el('label', { class: 'check', title: 'Let the simulation run while the AI thinks, as a real drone would, instead of pausing it' }, kf, 'keep flying')),
    T.kind === 'expr' ? el('p', { class: 'hint', text: 'Reads: ' + Object.keys(agentSample()).join(', ') + '.' }) : null].filter(Boolean));
  return box;
}

const AI_DESC = {   // a tool call, in a few words, for the chat
  get_state: () => 'read the state', get_history: a => `read the last ${a.seconds || 10} s`, get_airframe: () => 'read the airframe', get_part: a => `read part ${a.id}`,
  set_part: a => `set ${a.id}: ${Object.keys(a.fields || {}).join(', ')}`, add_part: a => `add a ${a.type}${a.on != null ? ' on ' + a.on : a.between != null ? ' above ' + a.between : ''}`,
  remove_part: a => `remove ${a.id}`, attach_part: a => `attach ${a.id} to ${a.to}`, set_frame: a => 'set ' + Object.keys(a).join(', '), load_layout: a => 'load ' + a.layout,
  get_computers: () => 'read the computers', set_computers: () => 'set the computers', list_formulas: () => 'list the formulas', get_formula: a => 'read ' + a.key,
  set_formula: a => 'change ' + a.key, reset_formula: a => 'reset ' + a.key, simulation: a => a.action + (a.speed ? ' ' + a.speed : ''),
  wait: a => `run ${a.seconds} s${a.fast ? ', fast' : ''}${a.until ? ' until ' + a.until : ''}`, fly: a => a.action === 'goto' ? `go to ${['x', 'y', 'z'].map(k => a[k] ?? '·').join(', ')}${a.heading != null ? ' facing ' + a.heading + '°' : ''}` : a.action + (a.level ? ' ' + a.level : ''),
  latch: a => `${a.action} latch ${a.latch}`, environment: a => 'set ' + Object.entries(a).map(([k, v]) => `${k} ${v}`).join(', '), break_part: a => `break ${a.id}${a.mode ? ' (' + a.mode + ')' : ''}`, repair_all: () => 'repair all',
};
function agentRenderFeed(th) {
  const F = $('#aiFeed'); if (!F) return;
  const cur = threadOf(agent.cur); if (th && th !== cur) { agentUi(); return; } th = cur; if (!th) return;
  const atEnd = F.scrollHeight - F.scrollTop - F.clientHeight < 40;
  F.textContent = '';
  if (!th.feed.length) {
    if (th.kind === 'trigger') F.append(el('p', { class: 'ai-empty', text: 'Nothing yet. When it fires, what it sent and what the AI did shows here, and you can carry on the conversation.' }));
    else F.append(el('div', { class: 'ai-empty' }, el('p', { text: 'What should it do? For example:' }),
      ...AI_SUGGEST.map(s => el('button', { type: 'button', class: 'ai-suggest', text: s, onclick: () => { const i = $('#aiInput'); if (i) { i.value = s; i.focus(); } } }))));
  }
  for (const it of th.feed) {
    if (it.who === 'tool') {
      let a = {}; try { a = JSON.parse(it.args || '{}'); } catch (e) {}
      const d = el('details', { class: 'ai-tool' + (it.bad ? ' bad' : '') });
      d.append(el('summary', {}, el('span', { class: 'ai-gear', 'aria-hidden': 'true', text: it.text === '…' ? '◌' : it.bad ? '✕' : '✓' }), (AI_DESC[it.name] ? AI_DESC[it.name](a) : it.name)));
      if (it.name === 'set_formula' && a.code) d.append(el('pre', { class: 'ai-pre', text: a.code }));
      else if (it.args && it.args !== '{}') d.append(el('pre', { class: 'ai-pre', text: it.args }));
      d.append(el('pre', { class: 'ai-pre out', text: it.text }));
      if (it.img) d.append(el('img', { class: 'ai-img', src: it.img, alt: 'What it saw: the 3D view' }));
      F.append(d);
      if (it.confirm) F.append(el('div', { class: 'ai-ask' }, el('span', { text: it.confirm.text }),
        el('button', { class: 'btn primary', type: 'button', text: 'Apply', onclick: () => it.confirm.resolve(true) }),
        el('button', { class: 'btn', type: 'button', text: 'Don\'t', onclick: () => it.confirm.resolve(false) })));
      if (it.undo && !it.undone) F.append(el('div', { class: 'ai-ask' }, el('button', { class: 'btn', type: 'button', text: '↶ ' + it.undo.label + ' this formula change',
        onclick: () => { try { it.undone = it.undo.run(); } catch (e) { it.undone = 'Couldn\'t undo: ' + e.message; } agentFeed({ who: 'note', text: it.undone }, th); } })));
      continue;
    }
    if (it.who === 'cont') {   // stopped at the limit (or the budget): carry on, or leave it
      const box = el('div', { class: 'ai-cont' }, el('span', { text: it.text + (it.used ? '' : it.budget ? ' Allow more and carry on?' : ' Carry on, or stop here?') }));
      if (!it.used) box.append(
        el('button', { class: 'btn primary', type: 'button', text: it.budget ? `Allow ${agent.cfg.budget} more and continue` : 'Continue', disabled: agent.busy ? '' : null, onclick: () => {
          if (agent.busy) return; it.used = true; if (it.budget) { agent.cfg.budget += agent.cfg.budget; agentSave(); }
          agentTurn(null, { thread: th, resume: true }); } }),
        el('button', { class: 'btn', type: 'button', text: 'Stop here', onclick: () => { it.used = true; agentRenderFeed(th); } }));
      F.append(box); continue;
    }
    F.append(el('div', { class: 'ai-msg w-' + it.who + (it.tone ? ' t-' + it.tone : '') },
      it.who === 'trigger' ? el('b', { text: `⚡ ${it.t != null ? (+it.t).toFixed(1) + ' s · ' : ''}` }) : null, el('span', { text: it.text })));
  }
  if (agent.turn === th) F.append(el('div', { class: 'ai-typing', 'aria-label': 'Thinking' }, el('i'), el('i'), el('i')));
  if (atEnd || agent.turn === th) F.scrollTop = F.scrollHeight;
  agentUi();
}

/* ───────── settings ───────── */
function settingsView() {
  const C = agent.cfg, box = el('div', { class: 'ai-view' });
  box.append(topBar(iconBtn('Back', '‹', () => { aiUi.view = threadOf(agent.cur) ? 'chat' : 'list'; agentRender(); }, { id: 'aiBack' }), el('b', { text: 'Settings' }), null));
  const kv = (k, v) => [el('dt', { text: k }), el('dd', { text: v })];
  const conn = el('div', { class: 'ai-card' }, el('span', { class: 'lbl', text: 'Connection' }),
    el('dl', { class: 'kv' }, ...kv('Provider', epLabel()), ...kv('URL', C.url), ...kv('Model', C.model), ...kv('Key', agent.key ? '••••' + agent.key.slice(-4) + (C.remember ? ' (remembered)' : ' (until the tab closes)') : 'none')),
    el('div', { class: 'ai-row' },
      el('button', { class: 'btn', type: 'button', id: 'aiEditConn', text: 'Change…', onclick: () => { aiUi.editConn = true; aiUi.draft = null; aiUi.models = []; agentRender(); } }),
      el('button', { class: 'btn danger', type: 'button', id: 'aiDelConn', text: 'Remove the connection', onclick: () => {
        if (!confirm('Remove the connection and forget the key? Your chats and triggers stay.')) return;
        agentStop(); C.connected = false; agent.key = ''; agentSave(); aiUi.view = 'list'; agentRender(); } })));
  const chk = (id, label, k) => { const i = el('input', { type: 'checkbox', id }); i.checked = !!C[k]; i.addEventListener('change', () => { C[k] = i.checked; agentSave(); }); return el('label', { class: 'check', for: id }, i, label); };
  const budget = numField('aiBudget', { label: 'Requests this session, at most', min: 1, max: 1000, step: 1, u: '', dp: 0, int: true }, () => C.budget, v => { C.budget = Math.round(v); agentSave(); agentUi(); });
  const beh = el('div', { class: 'ai-card' }, el('span', { class: 'lbl', text: 'While it works' }),
    chk('aiPause', 'Pause the simulation while the AI thinks (a trigger can keep it flying)', 'pauseThinking'),
    chk('aiAsk', 'Ask me before it changes a formula', 'askFormulas'),
    numField('aiRounds', { label: 'Ask to continue after this many requests in one turn', min: 5, max: 200, step: 5, u: '', dp: 0, int: true }, () => C.roundLimit, v => { C.roundLimit = Math.round(v); agentSave(); }).node,
    chk('aiSee', 'The model can see images: let it look at the 3D view', 'canSee'),
    chk('aiJs', 'Let it run its own JavaScript in the page, for anything the tools don\'t reach', 'allowJs'),
    el('p', { class: 'hint', text: 'Its own JavaScript has the whole page: it could change anything, and read your API key from it. Turn it on for a model and endpoint you trust.' }),
    chk('aiAskJs', 'Ask me before it runs JavaScript', 'askJs'), budget.node,
    el('p', { class: 'hint', id: 'aiUse' }),
    el('div', { class: 'ai-row' }, el('button', { class: 'btn', type: 'button', text: 'Reset the count', onclick: () => { agent.used = 0; agent.tokens = { in: 0, out: 0 }; agentUi(); } })),
    el('p', { class: 'hint', text: 'Each request to the model counts; a turn with tools takes several. Past the number above it stops with a Continue button, in case it\'s going round in circles. Airframe and computer changes go into Undo; a formula change has its own Undo in the chat.' }));
  const data = el('div', { class: 'ai-card' }, el('span', { class: 'lbl', text: 'Chats' }),
    el('p', { class: 'hint', text: 'Kept in this browser (the newest 30, and each trigger\'s).' }),
    el('div', { class: 'ai-row' }, el('button', { class: 'btn danger', type: 'button', text: 'Delete all chats', onclick: () => {
      if (!confirm('Delete every chat? Triggers stay.')) return; for (const t of agent.threads.filter(x => x.kind === 'chat')) threadDelete(t.id); agentRender(); } })));
  box.append(el('div', { class: 'ai-list' }, conn, beh, data));
  return box;
}

/* ───────── what changes as it works ───────── */
function agentUi() {
  if (!aiUi.built) return;
  const C = agent.cfg, th = threadOf(agent.cur);
  const use = $('#aiUse'); if (use) setText(use, `${agent.used}/${C.budget} requests${agent.tokens.in ? ` · ${kfmt(agent.tokens.in)} in, ${kfmt(agent.tokens.out)} out` : ''} this session`);
  const tt = document.querySelector('.ai-title'); if (tt && th && th.kind === 'chat') setText(tt, th.title);
  const tu = $('#aiThreadUse'); if (tu && th) setText(tu, th.tokens.in ? `${kfmt(th.tokens.in)} in · ${kfmt(th.tokens.out)} out · ${th.requests} req` : 'no tokens yet');
  const send = $('#aiSend'), stop = $('#aiStop');
  if (send) send.disabled = agent.busy; if (stop) stop.hidden = !agent.busy;
  const hint = $('#aiHint');
  if (hint) setText(hint, agent.busy ? (agent.turn === th ? 'Thinking…' : 'Busy in another thread…') + (agent.queue.length ? ` (${agent.queue.length} waiting)` : '') : `${agent.used}/${C.budget} requests`);
  const fired = $('#aiFired'); if (fired && th && th.kind === 'trigger') { const T = agent.triggers.find(x => x.id === th.triggerId); if (T) setText(fired, T.fired ? `fired ${T.fired}×` : 'not fired yet'); }
  document.querySelectorAll('.ai-thread').forEach(r => r.classList.toggle('busy', !!agent.turn && r.dataset.thread === agent.turn.id));
  $('#tabAi').classList.toggle('busy', agent.busy);
}
setInterval(() => { if (aiUi.built && !$('#paneAi').hidden && aiUi.view === 'list' && agent.cfg.connected && !aiUi.editConn) {   // (fired counts, times)
  const L = $('.ai-list'); if (L && !L.contains(document.activeElement)) agentRender(); } }, 5000);
if (!agent.cfg.connected) aiUi.view = 'list';
else if (agent.threads.some(t => t.kind === 'chat')) aiUi.view = 'list';
agentRender();
