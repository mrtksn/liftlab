'use strict';
// The AI agent: a model behind an OpenAI-compatible chat API (OpenAI, OpenRouter, a local server) that can read the
// simulator and change it through tools: the airframe, computers, wiring, driver source, formulas and flight.
// Agent tools operate on the saved design and simulation; physical connections are handled separately by Install.
//
// A turn: what you typed (or a trigger's message, with the state at that moment) goes to the model with the tools;
// the model answers, or asks for tools; the page runs them and sends the results back; until it answers in words, or
// it has made cfg.roundLimit requests in this turn, when it shows a Continue button (in case it's going round in circles). Each request counts against the session's budget, so nothing can loop away with
// your API credit. The key stays in this browser and goes only to the endpoint you set.
//
// Triggers (agent-ui.js sets them up): conditions checked 10 times a simulated second (a crash, the battery low, far
// from the target, a formula stopped, every N seconds while flying, or an expression of your own). When one turns
// true it sends the agent its message and the state; it doesn't fire again within its gap.

const AGENT_LS = 'dfb-agent', AGENT_KEY_LS = 'dfb-agent-key', AGENT_THREADS_LS = 'dfb-agent-threads';
const AGENT_THREADS_MAX = 30;         // chats kept (the oldest go first); each trigger has its own as well
const AGENT_RESULT_MAX = 6000;        // characters of one tool result sent back
const AGENT_CONTEXT_MAX = 90000;      // characters of conversation kept (the oldest turns are dropped)
const AGENT_ENDPOINTS = {
  openai: { label: 'OpenAI', url: 'https://api.openai.com/v1', model: 'gpt-4.1-mini' },
  openrouter: { label: 'OpenRouter', url: 'https://openrouter.ai/api/v1', model: 'openai/gpt-4.1-mini' },
  ollama: { label: 'Ollama (this computer)', url: 'http://localhost:11434/v1', model: 'qwen2.5:7b', noKey: true },
  lmstudio: { label: 'LM Studio (this computer)', url: 'http://localhost:1234/v1', model: '', noKey: true },
  custom: { label: 'Other (OpenAI-compatible)', url: '', model: '' },
};

const agent = {
  cfg: { connected: false, endpoint: 'openai', url: AGENT_ENDPOINTS.openai.url, model: AGENT_ENDPOINTS.openai.model, remember: false,
    pauseThinking: true, askFormulas: false, budget: 200, roundLimit: 40, temperature: 0.2, canSee: false, allowJs: false, askJs: true },
  key: '',
  triggers: [],          // { id, kind, value, expr, msg, gap, keepFlying, on, fired, last }
  // Threads: each chat, and each trigger's (kind 'trigger', triggerId). msgs: the conversation as the API takes it
  // (without the system message); feed: what the chat shows ({ who: 'you'|'ai'|'tool'|'trigger'|'note', text, … }).
  threads: [], cur: null,
  busy: false, abort: null, queue: [], turn: null,   // turn: the thread a turn is running in
  used: 0, tokens: { in: 0, out: 0 },
  rec: [], recNext: 0, recLast: 0,   // telemetry, 10 per simulated second, the last 120 s
  wasRunning: null,
};

function agentLoad() {
  try {
    const s = JSON.parse(localStorage.getItem(AGENT_LS) || 'null');
    if (s && s.cfg) for (const k of Object.keys(agent.cfg)) if (s.cfg[k] != null && typeof s.cfg[k] === typeof agent.cfg[k]) agent.cfg[k] = s.cfg[k];
    if (s && Array.isArray(s.triggers)) agent.triggers = s.triggers.slice(0, 12).map(t => ({ ...t, fired: 0, last: -1e9, was: false }));
    agent.key = (agent.cfg.remember ? localStorage.getItem(AGENT_KEY_LS) : sessionStorage.getItem(AGENT_KEY_LS)) || '';
    if (s && s.cfg && s.cfg.connected == null) agent.cfg.connected = !!(agent.cfg.url && agent.cfg.model);   // (set up before connections were a step)
    const T = JSON.parse(localStorage.getItem(AGENT_THREADS_LS) || '[]');
    if (Array.isArray(T)) agent.threads = T.filter(t => t && t.id && Array.isArray(t.msgs) && Array.isArray(t.feed));
  } catch (e) {}
}
// The threads, without what can't be kept (a pending question, an undo's code): the newest chats, every trigger's.
function agentSaveThreads() {
  const keep = t => ({ ...t, feed: t.feed.map(({ confirm, undo, img, ...f }) => f) });
  const chats = agent.threads.filter(t => t.kind === 'chat').sort((a, b) => b.updated - a.updated).slice(0, AGENT_THREADS_MAX);
  const out = [...chats, ...agent.threads.filter(t => t.kind === 'trigger')].map(keep);
  try { localStorage.setItem(AGENT_THREADS_LS, JSON.stringify(out)); }
  catch (e) { try { localStorage.setItem(AGENT_THREADS_LS, JSON.stringify(out.map(t => ({ ...t, feed: t.feed.slice(-40), msgs: t.msgs.slice(-20) })))); } catch (x) {} }   // (over the browser's quota: keep less)
}
function agentSave() {
  try {
    localStorage.setItem(AGENT_LS, JSON.stringify({ cfg: agent.cfg, triggers: agent.triggers.map(({ fired, last, was, ...t }) => t) }));
    if (agent.cfg.remember) { localStorage.setItem(AGENT_KEY_LS, agent.key); sessionStorage.removeItem(AGENT_KEY_LS); }
    else { localStorage.removeItem(AGENT_KEY_LS); sessionStorage.setItem(AGENT_KEY_LS, agent.key); }   // (this tab only, until it closes)
  } catch (e) {}
}
agentLoad();

/* ───────── what the agent sees: the state, and its recent past ───────── */
const r2 = x => Math.round(x * 100) / 100, r1 = x => Math.round(x * 10) / 10;
function agentAttitude(q) {
  const R = qmat(q);
  return { roll: r1(Math.atan2(R[7], R[8]) * R2D), pitch: r1(Math.asin(clamp(-R[6], -1, 1)) * R2D), yaw: r1(Math.atan2(R[3], R[0]) * R2D),
    tilt: r1(Math.acos(clamp(R[8], -1, 1)) * R2D) };
}
// One flat sample: what the recorder keeps and what trigger expressions read.
function agentSample() {
  const a = agentAttitude(S.q), home = brt.home || [0, 0, 0];
  const err = Math.hypot(S.p[0] - setpoint.x, S.p[1] - setpoint.y, S.p[2] - setpoint.z);
  const F = S.aero && S.aero.length ? S.aero.reduce((s, e) => add(s, e.F), [0, 0, 0]) : [0, 0, 0];
  return { t: r2(S.t), x: r2(S.p[0]), y: r2(S.p[1]), alt: r2(S.p[2]),
    vx: r2(S.v[0]), vy: r2(S.v[1]), climb: r2(S.v[2]), speed: r2(Math.hypot(S.v[0], S.v[1])),
    roll: a.roll, pitch: a.pitch, yaw: a.yaw, tilt: a.tilt,
    tx: r2(setpoint.x), ty: r2(setpoint.y), tz: r2(setpoint.z), heading: Math.round(setpoint.yaw), err: r2(err),
    batt: Math.round(100 * Math.max(0, S.batt.soc ?? 1)), volts: r2(S.battV || 0),
    crashed: !!S.crashed, phase: brt.pilot.phase, flying: brt.pilot.phase === 'flying' && !S.crashed,
    aero: r2(Math.hypot(...F)), home: home.map(r2), power: cargo.power,
    formulaErrors: editedLaws().filter(L => L.status === 'error').length };
}
function agentRecord() {
  if (S.t < agent.recLast) { agent.rec.length = 0; agent.recNext = 0; }   // (a reset: the clock went back)
  agent.recLast = S.t;
  if (S.t < agent.recNext - 1e-9) return;
  agent.recNext = S.t + 0.1;
  const s = agentSample(); agent.rec.push(s); if (agent.rec.length > 1200) agent.rec.shift();
  agentTriggers(s);
}
{ const step = physStep; physStep = function () { step(); if (S.steps % 10 === 0) agentRecord(); }; }   // (every 5 ms of simulated time)

// What happened lately: the boards', the supervisor's and the cargo's logs, newest first.
function agentEvents(n = 8) {
  const ev = [...(RN.log || []).map(e => ({ t: e.t, msg: e.msg })), ...(sup.log || []).map(e => ({ t: e.t, msg: e.msg })), ...(cargo.log || []).map(e => ({ t: e.t, msg: e.msg }))];
  return ev.filter(e => e && e.msg).sort((a, b) => b.t - a.t).slice(0, n).map(e => `${(+e.t || 0).toFixed(1)} s: ${e.msg}`);
}
function agentState() {
  const s = agentSample();
  return {
    time: s.t, simulation: running ? 'running' : 'paused', speed, crashed: S.crashed || null, phase: flightPhaseText(),
    position: [s.x, s.y, s.alt], velocity: [s.vx, s.vy, s.climb], attitude_deg: { roll: s.roll, pitch: s.pitch, yaw: s.yaw, tilt: s.tilt },
    target: { x: s.tx, y: s.ty, z: s.tz, heading: s.heading }, distance_to_target: s.err,
    believes_position: est.havePos ? est.p.map(r2) : null,
    battery: { percent: s.batt, volts: s.volts }, power: cargo.power,
    wind: { speed: envr.wind, toward_deg: envr.windDir, turbulence: envr.turb }, wing_force_N: s.aero || undefined,
    airframe_check: envRes ? { verdict: envRes.verdict, title: envRes.title, why: envRes.why } : null,
    formulas: { edited: editedLaws().map(L => L.def.key), stopped: editedLaws().filter(L => L.status === 'error').map(L => L.def.key + ': ' + L.err) },
    latches: latches().map((l, i) => { const v = typeof latchView === 'function' ? latchView(l) : null; return { index: i, name: l.name, closed: v ? v.closed : l.closed, holds: liveUnder(l).map(c => c.name) }; }),
    loose_things: cargo.loose.map(L => ({ name: L.name, at: L.p ? L.p.map(r2) : undefined })),
    parts_in_trouble: [...actuators(), ...joints()].map(c => { const h = hs.get(c.id) || {}; const st = h.prop ? 'prop broken' : h.dead ? 'stopped' : h.loss > 0.004 ? `lost ${Math.round(h.loss * 100)}% thrust` : h.limp ? 'limp' : h.jam != null ? 'jammed' : ''; return st ? `${c.name}: ${st}` : ''; }).filter(Boolean)
      .concat(hb.cut ? ['battery: cut out'] : hb.cellsLost ? [`battery: ${hb.cellsLost} cells lost`] : []),
    supervisor: brt.superView ? (MODE_TXT[brt.superView.mode | 0] || MODE_TXT[0])[0] : null,
    recent_events: agentEvents(),
  };
}

/* ───────── triggers ───────── */
const TRIGGER_KINDS = {
  crash: { label: 'It crashes', test: s => s.crashed },
  battery: { label: 'Battery below (%)', value: 25, test: (s, v) => s.batt < v },
  far: { label: 'Further from the target than (m)', value: 1, test: (s, v) => s.flying && s.err > v },
  tilt: { label: 'Tilts more than (°)', value: 35, test: (s, v) => !s.crashed && s.tilt > v },
  formula: { label: 'A formula is stopped (an error)', test: s => s.formulaErrors > 0 },
  every: { label: 'Every N seconds while flying', value: 20, periodic: true },
  expr: { label: 'Expression', expr: 'alt < 0.5 && flying' },
};
const triggerFn = new Map();   // trigger id -> compiled expression (or an error message)
function triggerTest(T) {
  if (T.kind !== 'expr') return TRIGGER_KINDS[T.kind].test;
  let f = triggerFn.get(T.id);
  if (!f || f.src !== T.expr) {
    try { const fn = new Function('s', `"use strict"; const { ${Object.keys(agentSample()).join(', ')} } = s; return (${T.expr || 'false'});`); f = { src: T.expr, fn, err: '' }; }
    catch (e) { f = { src: T.expr, fn: null, err: e.message }; }
    triggerFn.set(T.id, f);
  }
  return f.fn;
}
function agentTriggers(s) {
  for (const T of agent.triggers) {
    if (!T.on) continue;
    if (T.last > s.t) T.last = -1e9;   // (the clock went back: a reset)
    let fire = false;
    if (TRIGGER_KINDS[T.kind].periodic) fire = s.flying && s.t - T.last >= Math.max(2, +T.value || 20);
    else {
      const f = triggerTest(T); let now = false;
      try { now = !!(f && f(s, +T.value)); } catch (e) { now = false; }
      fire = now && !T.was && s.t - T.last >= (+T.gap || 0); T.was = now;
    }
    if (!fire) continue;
    T.last = s.t; T.fired++;
    agentAsk(`[trigger: ${triggerText(T)}] ${T.msg || 'React to this.'}\nState now:\n${JSON.stringify(agentState())}`,
      { trigger: T, keepFlying: !!T.keepFlying });
  }
}
const triggerText = T => T.kind === 'expr' ? T.expr : TRIGGER_KINDS[T.kind].label.replace(/\((.*)\)/, '') + (TRIGGER_KINDS[T.kind].value != null ? ' ' + T.value : '');

/* ───────── the conversation ───────── */
function agentSystem() {
  return [
    'You are the AI agent inside LiftLab, a browser simulator of a multirotor drone you can rebuild while it flies. You work on the SIMULATOR only.',
    'Axes: X forward, Y left, Z up, metres, from the world origin (the start point). Body axes on the airframe: the same, from the frame hub. Angles in degrees.',
    'Flight computers: boards (ESP32 microcontrollers, Raspberry Pi) run tasks: core (attitude, control, mixing), nav (position), learn, super (health), tlm (radio), cargo (latches). Each task runs formulas: JavaScript functions you can read and replace (same arguments, same kind of return value).',
    'Work in small steps and check: after a change, run the simulation for a few seconds (wait) and read the state. Read a formula (get_formula) before you replace it, and keep its signature.',
    'Use get_computers before set_computers and keep existing board IDs when renaming, reordering or moving tasks. Removing a board leaves its explicit wiring disconnected. get_hardware lists boards, device drivers and C presets; query a board id (or "ground") for its pins, resolved device assignments and wiring checks. get_wiring_overview gives the board connection list. Use set_hardware for atomic GPIO, board, driver/address, PWM/MOSFET duty, battery/receiver/serial and ground-input changes. It rejects new wiring errors unless draft=true; draft wiring may be saved but is not ready to install. Follow-up get_hardware/get_install_settings checks the result. apply_hardware_preset configures a combined 10DOF sensor module; set_driver_code saves a C preset or custom source, get_driver_code reads it in chunks.',
    'Hardware tools edit the simulated design and its installation settings only; they do not connect to, flash or operate physical boards. Only Install can send settings. Custom sensor C is saved/exported source: it is not compiled or executed in this browser and needs an ESP-IDF rebuild, custom firmware installation and driver="custom" on the relevant sensors. MOSFET outputs are active-high, one direction, zero duty when stopped; frequency is shared on the board. Duty ceilings affect simulated actuation, but the allocator/learning model does not yet account for reduced headroom. Distributed motor outputs, Pi I2C sensor drivers and physical optical-flow drivers are unsupported.',
    'Other tools: get_health, get_actuators, get_estimate, get_learning, get_boards, get_envelope, get_radio, get_events; stick and poke to fly by hand; set_view and look (if allowed) to see; designs, triggers, cargo_items, set_control, set_learning, set_radio, set_frame_shape. Prefer dedicated tools over run_js (if allowed).',
    'The person sees each tool you call. Be brief in words: say what you did, what you saw and what you suggest.',
    `Now: ${new Date().toISOString().slice(0, 10)}. Simulated time ${S.t.toFixed(1)} s.`,
  ].join('\n');
}
/* threads */
const threadOf = id => agent.threads.find(t => t.id === id) || null;
function threadNew(kind = 'chat', o = {}) {
  const t = { id: (kind === 'trigger' ? 'g' : 'c') + Date.now().toString(36) + Math.random().toString(36).slice(2, 5), kind, title: o.title || 'New chat',
    triggerId: o.triggerId || null, msgs: [], feed: [], tokens: { in: 0, out: 0 }, requests: 0, created: Date.now(), updated: Date.now() };
  agent.threads.push(t); return t;
}
function threadForTrigger(T) {
  let t = agent.threads.find(x => x.kind === 'trigger' && x.triggerId === T.id);
  if (!t) t = threadNew('trigger', { triggerId: T.id, title: triggerText(T) });
  return t;
}
function threadDelete(id) {
  const t = threadOf(id); if (!t) return;
  if (agent.turn === t) agentStop();
  agent.queue = agent.queue.filter(q => q.o.thread !== t);
  agent.threads = agent.threads.filter(x => x !== t); if (agent.cur === id) agent.cur = null;
  agentSaveThreads();
}
function agentFeed(item, th) {
  th = th || agent.turn || threadOf(agent.cur); if (!th) return item;
  th.feed.push(item); if (th.feed.length > 300) th.feed.shift(); th.updated = Date.now();
  if (typeof agentRenderFeed === 'function') agentRenderFeed(th);
  return item;
}
// Ask the agent something: what you typed (in a thread), or a trigger (in its own). Waits its turn if one is running.
function agentAsk(text, o = {}) {
  o.thread = o.thread || (o.trigger ? threadForTrigger(o.trigger) : threadOf(agent.cur) || threadNew());
  if (agent.busy) {
    if (o.trigger && agent.queue.some(q => q.o.trigger === o.trigger)) return;   // (that trigger is already waiting)
    if (agent.queue.length < 4) agent.queue.push({ text, o }); if (typeof agentUi === 'function') agentUi(); return;
  }
  agentTurn(text, o);
}
function agentTrim(th) {   // keep the conversation under AGENT_CONTEXT_MAX, dropping whole turns from the start
  let n = JSON.stringify(th.msgs).length;
  while (n > AGENT_CONTEXT_MAX && th.msgs.length > 2) {
    let i = 1; while (i < th.msgs.length && th.msgs[i].role !== 'user') i++;
    if (i >= th.msgs.length) break;
    th.msgs.splice(0, i); n = JSON.stringify(th.msgs).length;
  }
}
async function agentTurn(text, o = {}) {
  const th = o.thread || threadOf(agent.cur) || threadNew();
  if (!agent.cfg.url || !agent.cfg.connected) { agentFeed({ who: 'note', tone: 'bad', text: 'Connect a model first.' }, th); return; }
  agent.busy = true; agent.abort = new AbortController(); agent.turn = th; agentUi();
  if (th.kind === 'chat' && th.title === 'New chat' && !o.trigger) th.title = text.replace(/\s+/g, ' ').slice(0, 60) + (text.length > 60 ? '…' : '');
  if (!o.resume) {   // (a Continue picks the same turn up where it stopped: nothing new to say)
    agentFeed(o.trigger ? { who: 'trigger', text: triggerText(o.trigger) + (o.trigger.msg ? ': ' + o.trigger.msg : ''), t: S.t } : { who: 'you', text }, th);
    th.msgs.push({ role: 'user', content: text });
  }
  const limit = clamp(Math.round(agent.cfg.roundLimit || 40), 1, 1000);
  const pause = agent.cfg.pauseThinking && !o.keepFlying;
  agent.wasRunning = running;
  try {
    for (let round = 0; ; round++) {
      if (agent.used >= agent.cfg.budget) { agentFeed({ who: 'cont', budget: true, text: `The session's budget of ${agent.cfg.budget} requests is used up.` }); break; }
      if (round >= limit) { agentFeed({ who: 'cont', text: `${limit} requests in this turn and it hasn't finished.` }); break; }
      if (pause && running) { running = false; renderRun(); }
      agentTrim(th);
      const m = await agentRequest(th);
      const calls = m.tool_calls || [];
      th.msgs.push({ role: 'assistant', content: m.content || null, ...(calls.length ? { tool_calls: calls } : {}) });
      if (m.content && String(m.content).trim()) agentFeed({ who: 'ai', text: String(m.content).trim() });
      if (!calls.length) break;
      const images = [];   // (a look: the picture goes after the tool answers, as a user message: tool answers are text)
      for (const c of calls) {
        let args = {}, out;
        const item = agentFeed({ who: 'tool', name: c.function.name, args: c.function.arguments, text: '…' });
        try { args = c.function.arguments ? JSON.parse(c.function.arguments) : {}; } catch (e) { out = { error: 'the arguments are not valid JSON: ' + e.message }; }
        if (!out) {
          const T = AGENT_TOOLS[c.function.name];
          if (!T) out = { error: 'no such tool: ' + c.function.name };
          else try { out = await T.run(args, item); } catch (e) { out = { error: e.message || String(e) }; }
        }
        let s = typeof out === 'string' ? out : JSON.stringify(out);
        if (s.length > AGENT_RESULT_MAX) s = s.slice(0, AGENT_RESULT_MAX) + '… (cut: ask for less)';
        let img = null; if (out && out.__image) { img = out.__image; delete out.__image; }
        let s2 = img ? JSON.stringify(out) : s; if (img) item.img = img;
        item.text = s2; item.bad = !!(out && out.error); agentRenderFeed();
        th.msgs.push({ role: 'tool', tool_call_id: c.id, content: s2 });
        if (img) images.push(img);
        if (agent.abort.signal.aborted) throw new DOMException('stopped', 'AbortError');
      }
      if (images.length) th.msgs.push({ role: 'user', content: [{ type: 'text', text: 'The view you asked to look at:' }, ...images.map(u => ({ type: 'image_url', image_url: { url: u } }))] });
    }
  } catch (e) {
    if (e.name === 'AbortError') agentFeed({ who: 'note', text: 'Stopped.' });
    else agentFeed({ who: 'note', tone: 'bad', text: e.message || String(e) });
    // (the conversation must not end on tool calls without their answers)
    const last = th.msgs[th.msgs.length - 1];
    if (last && last.role === 'assistant' && last.tool_calls) for (const c of last.tool_calls) th.msgs.push({ role: 'tool', tool_call_id: c.id, content: '{"error":"stopped"}' });
  } finally {
    agent.busy = false; agent.abort = null; agent.turn = null; th.updated = Date.now(); agentSaveThreads();
    if (typeof agentRenderFeed === 'function') agentRenderFeed(th);   // (without the thinking dots)
    if (pause && agent.wasRunning && !running && !S.crashed) { running = true; renderRun(); }
    agentUi();
    const next = agent.queue.shift(); if (next) setTimeout(() => agentTurn(next.text, next.o), 0);
  }
}
function agentStop() { if (agent.abort) agent.abort.abort(); agent.queue.length = 0; }

async function agentRequest(th) {
  const url = agent.cfg.url.replace(/\/+$/, '') + '/chat/completions';
  const body = { model: agent.cfg.model, messages: [{ role: 'system', content: agentSystem() }, ...th.msgs],
    tools: Object.entries(AGENT_TOOLS).map(([name, T]) => ({ type: 'function', function: { name, description: T.desc, parameters: T.params } })),
    tool_choice: 'auto', temperature: agent.cfg.temperature };
  agent.used++; th.requests++; agentUi();
  let r;
  try { r = await fetch(url, { method: 'POST', headers: agentHeaders(), body: JSON.stringify(body), signal: agent.abort.signal }); }
  catch (e) {
    if (e.name === 'AbortError') throw e;
    throw new Error(`Couldn't reach ${url}: ${e.message}. A server on this computer must allow requests from this page (CORS: for Ollama, OLLAMA_ORIGINS=*).`);
  }
  let d = null; try { d = await r.json(); } catch (e) {}
  if (!r.ok) throw new Error(`The endpoint said ${r.status}${d && d.error ? ': ' + (d.error.message || JSON.stringify(d.error)) : ''}`);
  if (d && d.usage) for (const T of [agent.tokens, th.tokens]) { T.in += d.usage.prompt_tokens || 0; T.out += d.usage.completion_tokens || 0; }
  for (const x of th.msgs) if (Array.isArray(x.content) && x.content.some(p => p.type === 'image_url'))   // (seen once: keep the conversation small)
    x.content = x.content.map(p => p.type === 'image_url' ? { type: 'text', text: '[a picture of the view, already seen]' } : p);
  const m = d && d.choices && d.choices[0] && d.choices[0].message;
  if (!m) throw new Error('The endpoint sent no answer' + (d && d.error ? ': ' + (d.error.message || '') : ''));
  return m;
}
function agentHeaders() {
  const h = { 'Content-Type': 'application/json' };
  if (agent.key) h.Authorization = 'Bearer ' + agent.key;
  if (/openrouter\.ai/.test(agent.cfg.url)) h['X-Title'] = 'LiftLab';
  return h;
}
async function agentModels() {
  const r = await fetch(agent.cfg.url.replace(/\/+$/, '') + '/models', { headers: agentHeaders() });
  let d = null; try { d = await r.json(); } catch (e) {}
  if (!r.ok) throw new Error(`${r.status}${d && d.error ? ': ' + (d.error.message || '') : ''}`);
  return ((d && d.data) || []).map(m => m.id).filter(Boolean).sort();
}
