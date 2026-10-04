'use strict';
// The AI tab (left panel): the chat with the agent (agent.js), its triggers, the connection and the settings.

function refreshLawCard(key) { const c = lawCards.get(key); if (c) { c.ta.value = LAWS[key].src; fitTa(c.ta); refreshLaw(key); } }

const aiUi = { built: false };
function agentBuild() {
  const P = $('#paneAi'); P.textContent = '';
  // the chat
  const chat = el('section', { class: 'sec ai-chat' }, el('h2', {}, 'AI agent', el('small', { id: 'aiUse' })));
  const feed = el('div', { class: 'ai-feed', id: 'aiFeed', role: 'log', 'aria-live': 'polite', 'aria-label': 'Conversation with the AI agent' });
  const input = el('textarea', { id: 'aiInput', class: 'ai-input', rows: '3', placeholder: 'Ask it to fly, build, tune or test… (Enter sends, Shift+Enter: a new line)', 'aria-label': 'Message to the AI agent' });
  const send = el('button', { class: 'btn primary', type: 'button', id: 'aiSend', text: 'Send' });
  const stop = el('button', { class: 'btn', type: 'button', id: 'aiStop', text: 'Stop' });
  const clear = el('button', { class: 'btn', type: 'button', id: 'aiClear', text: 'New chat', title: 'Forget the conversation (the airframe and formulas stay as they are)' });
  const go = () => { const t = input.value.trim(); if (!t || agent.busy) return; input.value = ''; agentAsk(t); };
  send.addEventListener('click', go); stop.addEventListener('click', agentStop); clear.addEventListener('click', agentClear);
  input.addEventListener('keydown', e => { if (e.key === 'Enter' && !e.shiftKey && !e.isComposing) { e.preventDefault(); go(); } });
  chat.append(feed, input, el('div', { class: 'ai-row' }, send, stop, clear), el('p', { class: 'hint', id: 'aiHint' }));
  // triggers
  const trig = el('section', { class: 'sec' }, el('h2', {}, 'Triggers', el('small', { text: 'ask the AI when…' })),
    el('div', { id: 'aiTrig' }),
    el('button', { class: 'btn', type: 'button', id: 'aiTrigAdd', text: '+ Trigger', onclick: () => { if (agent.triggers.length >= 12) return; agent.triggers.push({ id: 't' + Date.now().toString(36), kind: 'crash', value: null, expr: TRIGGER_KINDS.expr.expr, msg: 'Find out why and suggest a fix.', gap: 10, keepFlying: false, on: true, fired: 0, last: -1e9, was: false }); agentSave(); agentRenderTriggers(); } }),
    el('p', { class: 'hint', text: 'Checked 10 times a simulated second. When one turns true it sends its message and the state to the AI; it waits its gap before it fires again. "Keep flying" lets the simulation run while the AI thinks (as a real drone would), instead of pausing it. Expressions can read: ' + Object.keys(agentSample()).join(', ') + '.' }));
  // connection and settings
  const C = agent.cfg, conn = el('details', { class: 'sec ai-conn', id: 'aiConn' });
  if (!C.url || (!agent.key && !(AGENT_ENDPOINTS[C.endpoint] || {}).noKey)) conn.open = true;
  const ep = el('select', { id: 'aiEp', 'aria-label': 'Endpoint' });
  for (const [k, E] of Object.entries(AGENT_ENDPOINTS)) { const o = el('option', { value: k, text: E.label }); if (k === C.endpoint) o.selected = true; ep.append(o); }
  const url = el('input', { type: 'url', id: 'aiUrl', value: C.url, placeholder: 'https://…/v1', autocomplete: 'off', spellcheck: 'false' });
  const key = el('input', { type: 'password', id: 'aiKey', value: agent.key, placeholder: 'sk-…', autocomplete: 'off', spellcheck: 'false' });
  const rem = el('input', { type: 'checkbox', id: 'aiRemember' }); rem.checked = C.remember;
  const model = el('input', { type: 'text', id: 'aiModel', value: C.model, list: 'aiModels', autocomplete: 'off', spellcheck: 'false', placeholder: 'model name' });
  const models = el('datalist', { id: 'aiModels' });
  const load = el('button', { class: 'btn', type: 'button', text: 'Load models', title: 'Ask the endpoint which models it has (also checks the key)' });
  const status = el('p', { class: 'hint', id: 'aiConnMsg', role: 'status' });
  ep.addEventListener('change', () => { C.endpoint = ep.value; const E = AGENT_ENDPOINTS[ep.value]; if (E.url) url.value = C.url = E.url; if (E.model) model.value = C.model = E.model; agentSave(); agentUi(); });
  url.addEventListener('change', () => { C.url = url.value.trim(); agentSave(); agentUi(); });
  key.addEventListener('change', () => { agent.key = key.value.trim(); agentSave(); agentUi(); });
  rem.addEventListener('change', () => { C.remember = rem.checked; agentSave(); });
  model.addEventListener('change', () => { C.model = model.value.trim(); agentSave(); agentUi(); });
  load.addEventListener('click', async () => {
    status.textContent = 'Asking…'; status.className = 'hint';
    try { const L = await agentModels(); models.textContent = ''; for (const m of L.slice(0, 400)) models.append(el('option', { value: m })); status.textContent = `${L.length} models. Pick one in the Model field.`; status.className = 'hint good'; }
    catch (e) { status.textContent = 'Couldn\'t list the models: ' + e.message; status.className = 'hint bad'; }
  });
  const row = (label, node, id) => el('label', { class: 'ai-field', for: id }, el('span', { class: 'lbl', text: label }), node);
  conn.append(el('summary', {}, el('h2', {}, 'Connection', el('small', { id: 'aiConnSmall' }))),
    row('Endpoint', ep, 'aiEp'), row('Base URL (OpenAI-compatible)', url, 'aiUrl'), row('API key', key, 'aiKey'),
    el('label', { class: 'check', for: 'aiRemember' }, rem, 'Remember the key in this browser (otherwise: until this tab closes)'),
    row('Model', el('div', { class: 'ai-row' }, model, load, models), 'aiModel'), status,
    el('p', { class: 'hint', text: 'The key goes only to this endpoint, from this page. A server on this computer (Ollama, LM Studio) must allow requests from a web page: CORS (for Ollama: OLLAMA_ORIGINS=*). Use a model that can call tools.' }));
  const chk = (id, label, k) => { const i = el('input', { type: 'checkbox', id }); i.checked = !!C[k]; i.addEventListener('change', () => { C[k] = i.checked; agentSave(); }); return el('label', { class: 'check', for: id }, i, label); };
  const budget = numField('aiBudget', { label: 'Requests this session, at most', min: 1, max: 1000, step: 1, u: '', dp: 0, int: true }, () => C.budget, v => { C.budget = Math.round(v); agentSave(); agentUi(); });
  const sets = el('details', { class: 'sec ai-conn' }, el('summary', {}, el('h2', {}, 'Settings')),
    chk('aiPause', 'Pause the simulation while the AI thinks (triggers can keep it flying)', 'pauseThinking'),
    chk('aiAsk', 'Ask me before it changes a formula', 'askFormulas'),
    budget.node,
    el('div', { class: 'ai-row' }, el('button', { class: 'btn', type: 'button', text: 'Reset the count', onclick: () => { agent.used = 0; agent.tokens = { in: 0, out: 0 }; agentUi(); } })),
    el('p', { class: 'hint', text: 'Each request to the model counts, a turn with tools takes several (at most 16). Airframe and computer changes go into Undo; a formula change has its own Undo in the chat. It works on the simulator only.' }));
  P.append(chat, trig, conn, sets);
  aiUi.built = true; agentRenderFeed(); agentRenderTriggers(); agentUi();
}

function agentUi() {
  if (!aiUi.built) return;
  const C = agent.cfg, E = AGENT_ENDPOINTS[C.endpoint] || {}, ready = !!C.url && !!C.model && (!!agent.key || !!E.noKey || C.endpoint === 'custom');
  setText($('#aiUse'), `${agent.used} / ${C.budget} requests${agent.tokens.in ? ` · ${(agent.tokens.in / 1000).toFixed(1)}k in, ${(agent.tokens.out / 1000).toFixed(1)}k out` : ''}`);
  setText($('#aiConnSmall'), ready ? `${E.label || 'custom'} · ${C.model}` : 'not set');
  $('#aiSend').disabled = agent.busy || !ready; $('#aiStop').disabled = !agent.busy;
  setText($('#aiHint'), agent.busy ? `Thinking…${agent.queue.length ? ` (${agent.queue.length} waiting)` : ''}` : !ready ? 'Set the endpoint, key and model under Connection, below.' : '');
  $('#tabAi').classList.toggle('busy', agent.busy);
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
function agentRenderFeed() {
  const F = $('#aiFeed'); if (!F) return;
  const atEnd = F.scrollHeight - F.scrollTop - F.clientHeight < 40;
  F.textContent = '';
  if (!agent.feed.length) F.append(el('p', { class: 'ai-empty', text: 'Try: "Fly a 2 m square at 2 m height and tell me how well it held the corners", "Add a wing and see how it flies in 5 m/s wind", "Make the position control softer", or "Find out why it crashes when motor 2 stops".' }));
  for (const it of agent.feed) {
    if (it.who === 'tool') {
      let a = {}; try { a = JSON.parse(it.args || '{}'); } catch (e) {}
      const d = el('details', { class: 'ai-tool' + (it.bad ? ' bad' : '') });
      d.append(el('summary', {}, el('span', { class: 'ai-gear', 'aria-hidden': 'true', text: it.text === '…' ? '◌' : it.bad ? '✕' : '✓' }), (AI_DESC[it.name] ? AI_DESC[it.name](a) : it.name)));
      if (it.name === 'set_formula' && a.code) d.append(el('pre', { class: 'ai-pre', text: a.code }));
      else if (it.args && it.args !== '{}') d.append(el('pre', { class: 'ai-pre', text: it.args }));
      d.append(el('pre', { class: 'ai-pre out', text: it.text }));
      F.append(d);
      if (it.confirm) F.append(el('div', { class: 'ai-ask' }, el('span', { text: it.confirm.text }),
        el('button', { class: 'btn primary', type: 'button', text: 'Apply', onclick: () => it.confirm.resolve(true) }),
        el('button', { class: 'btn', type: 'button', text: 'Don\'t', onclick: () => it.confirm.resolve(false) })));
      if (it.undo && !it.undone) F.append(el('div', { class: 'ai-ask' }, el('button', { class: 'btn', type: 'button', text: '↶ ' + it.undo.label + ' this formula change',
        onclick: () => { try { it.undone = it.undo.run(); } catch (e) { it.undone = 'Couldn\'t undo: ' + e.message; } agentFeed({ who: 'note', text: it.undone }); } })));
      continue;
    }
    F.append(el('div', { class: 'ai-msg w-' + it.who + (it.tone ? ' t-' + it.tone : '') },
      it.who === 'trigger' ? el('b', { text: '⚡ ' }) : null, el('span', { text: it.text })));
  }
  if (atEnd) F.scrollTop = F.scrollHeight;
}

function agentRenderTriggers() {
  const box = $('#aiTrig'); if (!box) return; box.textContent = '';
  if (!agent.triggers.length) box.append(el('p', { class: 'hint', text: 'None yet.' }));
  for (const T of agent.triggers) {
    const K = TRIGGER_KINDS[T.kind] || TRIGGER_KINDS.crash, ch = () => { agentSave(); };
    const on = el('input', { type: 'checkbox', 'aria-label': 'On' }); on.checked = T.on; on.addEventListener('change', () => { T.on = on.checked; T.was = false; ch(); });
    const kind = el('select', { 'aria-label': 'When' });
    for (const [k, D] of Object.entries(TRIGGER_KINDS)) { const o = el('option', { value: k, text: D.label }); if (k === T.kind) o.selected = true; kind.append(o); }
    kind.addEventListener('change', () => { T.kind = kind.value; T.value = TRIGGER_KINDS[T.kind].value ?? null; T.was = false; ch(); agentRenderTriggers(); });
    const val = K.value != null ? el('input', { type: 'number', class: 'ai-num', value: T.value ?? K.value, 'aria-label': 'Value' }) : null;
    if (val) val.addEventListener('change', () => { T.value = +val.value; ch(); });
    const expr = T.kind === 'expr' ? el('input', { type: 'text', class: 'ai-expr', value: T.expr || '', spellcheck: 'false', 'aria-label': 'Expression' }) : null;
    const err = el('span', { class: 'ai-err' });
    if (expr) { const chk = () => { const f = triggerTest(T); err.textContent = f ? '' : triggerFn.get(T.id).err; }; expr.addEventListener('change', () => { T.expr = expr.value; ch(); chk(); }); chk(); }
    const msg = el('input', { type: 'text', class: 'ai-tmsg', value: T.msg || '', placeholder: 'What to tell the AI', 'aria-label': 'Message' });
    msg.addEventListener('change', () => { T.msg = msg.value; ch(); });
    const gap = el('input', { type: 'number', class: 'ai-num', value: T.gap ?? 10, min: '0', 'aria-label': 'Gap, seconds' }); gap.addEventListener('change', () => { T.gap = Math.max(0, +gap.value || 0); ch(); });
    const kf = el('input', { type: 'checkbox' }); kf.checked = !!T.keepFlying; kf.addEventListener('change', () => { T.keepFlying = kf.checked; ch(); });
    const del = el('button', { class: 'icon-btn', type: 'button', text: '×', 'aria-label': 'Remove this trigger', onclick: () => { agent.triggers = agent.triggers.filter(x => x !== T); ch(); agentRenderTriggers(); } });
    box.append(el('div', { class: 'ai-trig' + (T.on ? '' : ' off') },
      el('div', { class: 'ai-row' }, on, kind, val, del), expr ? el('div', { class: 'ai-row' }, expr) : null, err,
      el('div', { class: 'ai-row' }, msg),
      el('div', { class: 'ai-row small' }, el('label', {}, 'gap ', gap, ' s'), el('label', { class: 'check' }, kf, 'keep flying'), el('span', { class: 'ai-fired', text: T.fired ? `fired ${T.fired}×` : '' }))));
  }
}
setInterval(() => { if (aiUi.built && !$('#paneAi').hidden) for (const [i, T] of agent.triggers.entries()) { const s = document.querySelectorAll('#aiTrig .ai-fired')[i]; if (s) setText(s, T.fired ? `fired ${T.fired}×` : ''); } }, 1000);
agentBuild();
