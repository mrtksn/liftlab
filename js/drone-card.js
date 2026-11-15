'use strict';
// The drone card: a small card over the 3D view, the same in the simulator and connected to the real drone (live.js):
// what the drone says (its link, battery, height, attitude, state), its buttons, a line to type a command, and what
// the drone answered. The buttons are the design's, the command module's (computers().ground.buttons; the defaults,
// CARD_BUTTONS_DEFAULT in boards.js, until you edit them). Each does one of:
//   arm, fly     the command module's arm and fly switches (a toggle: its label, and its label while on)
//   hold, home   momentary: hold where it is, fly home (with navigation)
//   send         a line of text up the radio (rc_core.h RC_CMD_TEXT). On the real drone the Pi's dfb_pi takes it as
//                its text commands (level, level none, safety off, esp32 show …: docs/pi-suite.md), an ESP32 that is
//                the radio as its settings lines; in the simulator the boards answer the same way (boards.js
//                droneText). The answer comes back down as a message: the card shows it under the line.
//   disconnect   leave the real drone (shown only while connected)
// confirm: the first press asks, a second within 3 s does it. In the simulator the switches take over from the
// simulator's pilot (who arms and takes off by itself after a reset) until the next reset.
// The card moves (drag its title) and folds; where it is and whether it's folded are this browser's.

const card = { sent: null, ask: null, msg: '', msgTone: '', msgT: 0, pos: null, fold: false, edit: false, next: 0 };
const cardButtons = () => computers().ground.buttons || CARD_BUTTONS_DEFAULT;
const cardPref = (k, d) => { try { const v = localStorage.getItem('dfb.card.' + k); return v == null ? d : JSON.parse(v); } catch (e) { return d; } };
const cardSavePref = (k, v) => { try { localStorage.setItem('dfb.card.' + k, JSON.stringify(v)); } catch (e) {} };
const cardLive = () => typeof liveOn === 'function' && liveOn();
const cardClock = () => cardLive() ? live.t : brt.t;

/* ───────── what the buttons do ───────── */
function cardSwitchOn(act) {
  if (cardLive()) return act === 'arm' ? live.arm : live.fly;
  const P = brt.pilot; return act === 'arm' ? !!P.arm : !!P.fly || P.phase === 'takeoff' || P.phase === 'flying' || P.phase === 'hand';
}
function cardShown(b) {
  const nav = hasTask('nav');
  if (b.act === 'home') return nav;
  if (b.act === 'disconnect') return cardLive();
  if (b.act === 'fly') return !(cardLive() && !nav && live.gp);       // (angle mode with a gamepad: its throttle stick takes off)
  return true;
}
// The simulator's switches: the card has them from now until the next reset.
function cardSimArm() {
  const P = brt.pilot; P.manual = true;
  if (P.arm) { Object.assign(P, { arm: 0, fly: 0, thr: 0, phase: 'ground', t: 0 }); linkLog('↑', 'switch', 'ARM off', 'the drone card', ''); return; }
  Object.assign(P, { arm: 1, fly: 0, thr: 0, phase: 'armed', t: 0 }); linkLog('↑', 'switch', 'ARM on', 'the drone card', 'warn');
}
function cardSimFly() {
  const P = brt.pilot, nav = hasTask('nav'); P.manual = true;
  if (!P.arm) return 'arm first';
  if (cardSwitchOn('fly')) { P.fly = 0; P.phase = 'landing'; P.t = 0; P.stillT = 0; if (!nav) P.thr = 0.3; linkLog('↑', 'switch', 'FLY off', 'the drone card: landing', ''); return; }
  Object.assign(P, { phase: 'takeoff', t: 0, fly: nav ? 1 : 0, thr: nav ? 0 : 0.85, alt0: null }); linkLog('↑', 'switch', 'FLY on', 'the drone card: taking off', '');
}
// A line for the drone: up through this page's command module (the simulator's, or live.js's own).
function cardSend(line) {
  line = String(line || '').trim(); if (!line) return 'type a command first';
  const g = cardLive() ? live.g : brt.gnd;
  if (!g) return hasTask('tlm') ? 'the command module isn\'t running (see the Ground tab)' : 'this design has no radio: give a board the Telemetry & radio task (Computers tab)';
  if (line.length > CARD_TEXT_MAX || /[^\x20-\x7e]/.test(line)) return `a command is up to ${CARD_TEXT_MAX} plain characters`;
  if (typeof g.gnd_text !== 'function') return 'this page\'s flight computers are older than the card: reload the page';
  txtIn(g, line); const r = g.gnd_text();
  if (r) return r === -1 ? 'too many commands waiting for the link: try again in a moment' : 'not sent: 1 to 57 plain characters';
  card.sent = { text: line, t: cardClock(), n: gs.nmsg || 0 };
  linkLog('↑', 'cmd', `text: ${line}`, 'the drone card', '');
  return '';
}
function cardPress(i, el) {
  const b = cardButtons()[i]; if (!b) return;
  const now = performance.now();
  if (b.confirm && !(card.ask && card.ask.i === i && now < card.ask.until)) { card.ask = { i, until: now + 3000 }; cardRender(true); return; }
  card.ask = null;
  let why = '';
  const L = cardLive();
  if (b.act === 'arm') why = L ? liveArmToggle() : cardSimArm();
  else if (b.act === 'fly') why = L ? liveFlyToggle() : cardSimFly();
  else if (b.act === 'hold') radioHold();
  else if (b.act === 'home') radioHome();
  else if (b.act === 'disconnect') { if (L) why = liveStopPress(); }
  else if (b.act === 'send') why = cardSend(b.cmd);
  cardNote(why || '', why ? 'bad' : '');
  if (why && el) flashEl(el);
  cardRender(true);
}
function cardNote(s, tone) { card.msg = s; card.msgTone = tone || ''; card.msgT = performance.now(); }

/* ───────── each moment: the card ───────── */
const cardDeg = r => `${(r * 57.2958).toFixed(0)}°`;
function cardRender(force) {
  const bar = $('#liveBar'); if (!bar) return;
  const now = performance.now(); if (!force && now < card.next) return; card.next = now + 200;
  const usb = typeof usbViewOn === 'function' && usbViewOn();
  bar.hidden = usb; if (usb) return;
  const L = cardLive(), v = gs.v || {}, lk = gs.link, a = gs.alert;
  bar.classList.toggle('is-live', L); bar.classList.toggle('is-folded', card.fold);
  setText($('#cardKind'), L ? 'Live' : 'Sim');
  setText($('#liveName'), L ? live.name : 'the simulated drone');
  const st = $('#liveState');
  if (L) {
    const conn = LIVE_MODEL.connected();
    setText(st, live.state === 'up' ? (conn ? 'connected' : 'connected, nothing coming') : live.state === 'lost' ? 'reconnecting…' : 'connecting…');
    st.dataset.tone = live.state === 'up' && conn ? 'good' : live.state === 'up' ? 'warn' : 'bad';
  } else { setText(st, brt.ready ? flightPhaseText() : (brt.err || 'starting')); st.dataset.tone = brt.fcState === 1 ? 'warn' : ''; }
  if (card.fold) { if (card.pos) cardPlace(); return; }
  const bits = [];
  if (lk) bits.push(`LQ ↑${Math.round(lk.upLq)}% ↓${Math.round(lk.downLq)}%`);
  if (v.battery) bits.push(`${v.battery.volts.toFixed(1)} V${v.battery.pct ? ` ${Math.round(v.battery.pct)}%` : ''}`);
  if (v.pos) bits.push(`height ${v.pos.z.toFixed(1)} m`); else if (v.baro) bits.push(`height ${v.baro.alt.toFixed(1)} m`);
  if (v.attitude) bits.push(`R ${cardDeg(v.attitude.roll)} P ${cardDeg(v.attitude.pitch)}`);
  if (v.mode) bits.push(v.mode.mode);
  if (!L && !gs.link && brt.ready) bits.push(fcStateWord());           // (no radio in the design: the flight core's own)
  setText($('#liveRead'), bits.join(' · ') || (L ? 'no telemetry yet' : 'no telemetry: the design has no radio'));
  const al = $('#liveAlert'), why = !L && brt.fcState !== 1 && /won't arm|not level|refus/i.test(brt.fcWhy || '') ? brt.fcWhy : '';
  const showA = a && a.level > 0 && !(L && live.state === 'connecting');
  al.hidden = !showA && !why;
  if (showA) { setText(al, a.text.charAt(0).toUpperCase() + a.text.slice(1)); al.dataset.tone = a.level >= 2 ? 'bad' : 'warn'; }
  else if (why) { setText(al, why.charAt(0).toUpperCase() + why.slice(1)); al.dataset.tone = 'warn'; }
  cardRenderButtons();
  cardRenderSaid();
  if (card.edit && $('#cardEditor').dataset.sig !== JSON.stringify(cardButtons())) cardRenderEditor();   // (an undo, another tab's edit)
  const nav = hasTask('nav'), pad = L && !!live.gp, slow = L && live.slow ? ` This page stood still ${live.slow}×: keep this tab in front.` : '';
  let note = card.msg && now - card.msgT < 6000 ? card.msg.charAt(0).toUpperCase() + card.msg.slice(1) + '.' : '';
  if (!note) note = L ? (!nav && !pad ? `Angle mode: ${live.landing ? 'landing (it sinks until the height stops changing).' : live.fly ? 'W/S climb and sink (let go: it holds the height), A/D turn, the arrows lean.' : 'Arm, then Take off: the keys and pads fly it, W/S climb.'}`
    : `${pad ? `Gamepad: ${live.gp.replace(/\s*\(.*$/, '')}. ` : ''}${nav ? 'The keys and pads move it; Space holds, H flies home.' : ''}`) + slow
    : brt.pilot.manual ? 'The card\'s switches fly it until the next reset.' : 'The simulator\'s pilot arms and takes off by itself; the switches here take over.';
  const n = $('#liveNote'); setText(n, note); n.dataset.tone = card.msg && now - card.msgT < 6000 ? card.msgTone : '';
  if (card.pos) cardPlace();                                           // (its height changes with what it shows)
}
const fcStateWord = () => ['disarmed', 'armed', 'failsafe', 'crashed', 'motor test'][brt.fcState] || '';
function cardRenderButtons() {
  const box = $('#cardBtns'), list = cardButtons(), now = performance.now();
  if (card.ask && now > card.ask.until) card.ask = null;
  const sig = JSON.stringify([list, cardLive()]);
  if (box.dataset.sig !== sig) {
    box.dataset.sig = sig; box.textContent = '';
    list.forEach((b, i) => {
      const x = el('button', { class: 'btn' + (b.act === 'arm' || b.act === 'fly' ? ' tog' : '') + (b.act === 'arm' ? ' live-arm' : ''), type: 'button', 'data-i': i,
        title: b.act === 'send' ? `Sends "${b.cmd}" to the drone${b.confirm ? ' (press twice)' : ''}` : { arm: 'The arm switch', fly: 'The fly switch: take off, and land when off', hold: 'Hold where it is (Space)', home: 'Fly home (H)', disconnect: 'Leave the real drone: the simulation comes back' }[b.act] });
      x.addEventListener('mousedown', e => e.preventDefault());       // (a click leaves the keyboard where it was: Space mustn't press it again)
      x.addEventListener('click', () => cardPress(i, x));
      box.append(x);
    });
  }
  box.querySelectorAll('button').forEach(x => {
    const i = +x.dataset.i, b = list[i]; if (!b) return;
    x.hidden = !cardShown(b);
    const on = (b.act === 'arm' || b.act === 'fly') && cardSwitchOn(b.act);
    if (b.act === 'arm' || b.act === 'fly') x.setAttribute('aria-pressed', String(on));
    const asking = card.ask && card.ask.i === i;
    x.classList.toggle('is-asking', !!asking);
    setText(x, asking ? `${b.label}? Again` : b.act === 'disconnect' && cardLive() && live.stopAsk && now < live.stopAsk ? 'Really disconnect?' : on && b.on ? b.on : b.label);
  });
  if (cardLive() && live.stopAsk && now > live.stopAsk) live.stopAsk = 0;
}
// What the drone said since the last command: the messages that came after it (the newest last).
function cardRenderSaid() {
  const box = $('#cardSaid'), s = card.sent;
  if (s && (gs.nmsg || 0) < s.n) card.sent = null;                    // (the ground station started again)
  const items = [];
  if (card.sent) {
    const k = Math.min(4, (gs.nmsg || 0) - s.n), got = gs.log.slice(0, Math.max(0, k)).reverse();
    items.push({ text: `› ${s.text}`, cls: 'card-sent' });
    for (const m of got) items.push({ text: m.text, cls: m.sev <= 3 ? 'is-bad' : m.sev === 4 ? 'is-warn' : '' });
    if (!got.length) items.push({ text: cardClock() - s.t > 5 ? 'no answer yet: is the link up, and the drone\'s software new enough to take text commands?' : 'sent; waiting for the answer…', cls: 'card-wait' });
  }
  const sig = JSON.stringify(items); if (box.dataset.sig === sig) return; box.dataset.sig = sig;
  box.textContent = ''; box.hidden = !items.length;
  for (const it of items) box.append(el('li', { class: it.cls, text: it.text }));
}
function cardFocus() { const b = $('#cardBtns button:not([hidden])'); if (card.fold) cardFoldSet(false); if (b) b.focus(); }

/* ───────── moving and folding ───────── */
function cardPlace() {
  const bar = $('#liveBar'), view = bar && bar.parentElement; if (!bar || !view) return;
  const p = card.pos;
  if (!p) { bar.style.left = bar.style.top = bar.style.right = bar.style.bottom = ''; return; }
  const W = view.clientWidth, H = view.clientHeight, w = bar.offsetWidth, h = bar.offsetHeight;
  const x = clamp(p.x, 0, Math.max(0, W - w)), y = clamp(p.y, 0, Math.max(0, H - h));   // (all of it inside the view, as far as it fits)
  bar.style.left = x + 'px'; bar.style.top = y + 'px'; bar.style.right = 'auto'; bar.style.bottom = 'auto';
}
function cardFoldSet(f) {
  card.fold = !!f; cardSavePref('fold', card.fold);
  const b = $('#cardFold'); b.setAttribute('aria-expanded', String(!card.fold)); b.title = card.fold ? 'Unfold the card' : 'Fold the card'; b.setAttribute('aria-label', b.title);
  $('#cardBody').hidden = card.fold; cardRender(true); cardPlace();
}
function cardDrag(e) {
  if (e.button !== 0 || e.target.closest('button')) return;
  const bar = $('#liveBar'), view = bar.parentElement, r0 = bar.getBoundingClientRect(), v0 = view.getBoundingClientRect();
  const dx = e.clientX - r0.left, dy = e.clientY - r0.top, head = e.currentTarget;
  head.setPointerCapture(e.pointerId); bar.classList.add('is-dragging');
  const move = ev => { card.pos = { x: ev.clientX - v0.left - dx, y: ev.clientY - v0.top - dy }; cardPlace(); };
  const up = () => { head.removeEventListener('pointermove', move); head.removeEventListener('pointerup', up); head.removeEventListener('pointercancel', up); bar.classList.remove('is-dragging'); if (card.pos) cardSavePref('pos', card.pos); };
  head.addEventListener('pointermove', move); head.addEventListener('pointerup', up); head.addEventListener('pointercancel', up);
  e.preventDefault();
}

/* ───────── editing the buttons ───────── */
const CARD_ACT_LABEL = { arm: 'Arm switch', fly: 'Take-off switch', hold: 'Hold', home: 'Fly home', send: 'Send a command', disconnect: 'Disconnect (live)' };
function cardSetButtons(list) {
  const C = computers(); C.ground.buttons = fixCardButtons(list);
  undoKey = 'cardButtons'; save(); $('#cardEditor').dataset.sig = JSON.stringify(cardButtons()); cardRender(true);
}
function cardRenderEditor() {
  const box = $('#cardEditor'); box.hidden = !card.edit; if (!card.edit) return;
  box.textContent = ''; box.dataset.sig = JSON.stringify(cardButtons());
  const list = cardButtons().map(b => ({ ...b }));
  const commit = () => cardSetButtons(list);
  box.append(el('p', { class: 'hint', text: 'The buttons are the design\'s (its command module\'s). A command goes up the radio as text: on the drone the Pi takes it (level, level none, level ROLL,PITCH,YAW, safety off, safety on, esp32 SETTING …); the simulator answers the same.' }));
  list.forEach((b, i) => {
    const row = el('div', { class: 'card-row' });
    const lab = el('input', { type: 'text', value: b.label, maxlength: 24, 'aria-label': `Button ${i + 1}: its label` });
    lab.addEventListener('change', () => { b.label = lab.value; commit(); });
    const act = el('select', { 'aria-label': `Button ${i + 1}: what it does` });
    for (const k of CARD_ACTS) act.append(el('option', { value: k, text: CARD_ACT_LABEL[k] }));
    act.value = b.act; act.addEventListener('change', () => { b.act = act.value; if (b.act === 'send' && !b.cmd) b.cmd = 'status'; commit(); cardRenderEditor(); });
    row.append(lab, act);
    if (b.act === 'send') { const c = el('input', { type: 'text', class: 'mono', value: b.cmd || '', maxlength: CARD_TEXT_MAX, spellcheck: 'false', placeholder: 'level', 'aria-label': `Button ${i + 1}: the command it sends` }); c.addEventListener('change', () => { b.cmd = c.value; commit(); }); row.append(c); }
    else if (b.act === 'arm' || b.act === 'fly') { const o = el('input', { type: 'text', value: b.on || '', maxlength: 24, placeholder: 'label while on', 'aria-label': `Button ${i + 1}: its label while on` }); o.addEventListener('change', () => { b.on = o.value; commit(); }); row.append(o); }
    else row.append(el('span'));
    const cf = el('label', { class: 'card-confirm', title: 'Press twice: the first asks' }); const cb = el('input', { type: 'checkbox' }); cb.checked = !!b.confirm;
    cb.addEventListener('change', () => { b.confirm = cb.checked; commit(); }); cf.append(cb, ' twice');
    const mv = (d, t) => { const x = el('button', { type: 'button', class: 'card-ic', title: t, 'aria-label': `Button ${i + 1}: ${t}`, text: d < 0 ? '↑' : '↓' }); x.disabled = d < 0 ? i === 0 : i === list.length - 1;
      x.addEventListener('click', () => { const j = i + d; [list[i], list[j]] = [list[j], list[i]]; commit(); cardRenderEditor(); }); return x; };
    const del = el('button', { type: 'button', class: 'card-ic', title: 'Remove it', 'aria-label': `Button ${i + 1}: remove it`, text: '✕' });
    del.addEventListener('click', () => { list.splice(i, 1); commit(); cardRenderEditor(); });
    row.append(cf, mv(-1, 'Move up'), mv(1, 'Move down'), del);
    box.append(row);
  });
  const add = el('button', { type: 'button', class: 'btn', text: 'Add a button' });
  add.addEventListener('click', () => { list.push({ label: 'Status', act: 'send', cmd: 'status' }); commit(); cardRenderEditor(); const last = box.querySelectorAll('.card-row'); const r = last[last.length - 1]; if (r) r.querySelector('input').select(); });
  const def = el('button', { type: 'button', class: 'btn', text: 'Defaults' });
  def.addEventListener('click', () => { const C = computers(); delete C.ground.buttons; undoKey = 'cardButtons'; save(); cardRenderEditor(); cardRender(true); });
  const done = el('button', { type: 'button', class: 'btn', text: 'Done' }); done.addEventListener('click', () => cardEditSet(false));
  box.append(el('div', { class: 'card-edit-acts' }, add, def, done));
}
function cardEditSet(on) {
  card.edit = !!on; const b = $('#cardEdit'); b.setAttribute('aria-pressed', String(card.edit)); $('#liveBar').classList.toggle('is-editing', card.edit);
  if (card.edit && card.fold) cardFoldSet(false);
  cardRenderEditor(); cardPlace();
}

(function cardBind() {
  const bar = $('#liveBar'); if (!bar) return;
  card.pos = cardPref('pos', null); card.fold = !!cardPref('fold', false);
  if (card.pos && !(isFinite(card.pos.x) && isFinite(card.pos.y))) card.pos = null;
  $('#cardHead').addEventListener('pointerdown', cardDrag);
  $('#cardHead').addEventListener('dblclick', e => { if (!e.target.closest('button')) { card.pos = null; cardSavePref('pos', null); cardPlace(); } });   // (back where it started)
  $('#cardFold').addEventListener('click', () => cardFoldSet(!card.fold));
  $('#cardEdit').addEventListener('click', () => cardEditSet(!card.edit));
  $('#cardCmd').addEventListener('submit', e => { e.preventDefault(); const i = $('#cardCmdIn'), why = cardSend(i.value); cardNote(why, why ? 'bad' : ''); if (!why) i.value = ''; cardRender(true); });
  $('#cardCmdIn').addEventListener('keydown', e => { if (e.key === 'Escape') e.target.blur(); e.stopPropagation(); });
  for (const x of bar.querySelectorAll('.card-head button')) x.addEventListener('mousedown', e => e.preventDefault());
  window.addEventListener('resize', cardPlace);
  if (typeof ResizeObserver === 'function') new ResizeObserver(() => cardPlace()).observe(bar);   // (it grew: unfolded, the editor, an answer)
  cardFoldSet(card.fold);
  setInterval(() => { if (!cardLive()) cardRender(); }, 200);         // (connected: live.js renders it with each frame)
})();
