'use strict';
// The drone card: a small card over the 3D view, the same in the simulator and connected to the real drone (live.js):
// what the drone says (its link, battery, height, attitude, state), its buttons, a line to type a command, and what
// the drone answered. The buttons are the design's, the command module's (computers().ground.buttons; the defaults,
// CARD_BUTTONS_DEFAULT in boards.js, until you edit them), in a group for each kind (switch, momentary, command, page
// action: CARD_KINDS below), each saying which of the design's boards takes it. Each does one of:
//   arm, fly     the command module's arm and fly switches (a toggle: its label, and its label while on)
//   hold, home   momentary: hold where it is, fly home (with navigation)
//   send         a line of text up the radio (rc_core.h RC_CMD_TEXT). On the real drone the Pi's dfb_pi takes it as
//                its text commands (level, level none, safety off, esp32 show …: docs/pi-suite.md), an ESP32 that is
//                the radio as its settings lines; in the simulator the boards answer the same way (boards.js
//                droneText). The answer comes back down as a message: the card shows it under the line.
//   upload       Upload design: the design's files to the drone's Pi, its wiring and airframe on to the ESP32 (below:
//                cardUpload), up the radio; shown when the radio is on a Pi. Pressed again while it runs, it stops
//   disconnect   leave the real drone (shown only while connected)
// confirm: the first press asks, a second within 3 s does it. In the simulator the switches take over from the
// simulator's pilot (who arms and takes off by itself after a reset) until the next reset.
// The card moves (drag its title) and folds; where it is and whether it's folded are this browser's.

const card = { sent: null, ask: null, msg: '', msgTone: '', msgT: 0, pos: null, fold: false, edit: false, next: 0, hover: null };
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
  if (b.act === 'upload') return !!cardRadioPi();
  if (b.act === 'fly') return !(cardLive() && !nav && live.gp);       // (angle mode with a gamepad: its throttle stick takes off)
  return true;
}
/* ───────── what each button is: its kind, and who takes it ───────── */
// Four kinds, and the card and its editor say which each button is (a group each) and which of the design's boards
// takes it (the names on the Computers tab):
//   switch     a radio channel in every frame the command module sends (about 250 a second): it stays as you set it,
//              and if the link drops the drone stops hearing it (the failsafe). Arm, take off.
//   momentary  a radio channel switched on for a moment: hold, fly home (the navigation's).
//   command    a line of text up the radio, once (sent again until it arrives), answered under the command line.
//   page       done by this page: Upload design (a script of commands, below), Disconnect (nothing sent).
const CARD_KINDS = {
  switch: { name: 'Switches', one: 'switch', how: 'held on the radio', hint: 'A radio channel in every frame (about 250 a second): it stays as you set it, and if the link drops the drone lets go of it (its failsafe).' },
  pulse: { name: 'Momentary', one: 'momentary switch', how: 'on the radio for a moment', hint: 'A radio channel switched on for a moment: the drone acts on it once.' },
  text: { name: 'Commands', one: 'command', how: 'one-off text', hint: 'A line of text up the radio, sent once (again until it arrives); the answer shows under the command line.' },
  page: { name: 'Actions', one: 'page action', how: 'run by this page', hint: 'This page does it: Upload design runs a series of commands; Disconnect sends nothing.' },
};
const CARD_KIND_ORDER = ['switch', 'pulse', 'text', 'page'];
const CARD_ACT_KIND = { arm: 'switch', fly: 'switch', hold: 'pulse', home: 'pulse', send: 'text', upload: 'page', disconnect: 'page' };
// The commands the drone's Pi takes (dfb_pi: docs/pi-suite.md), for the command line's suggestions and the editor's menu.
const CARD_CMDS = [
  { cmd: 'level', label: 'Level here', what: 'as it sits now is level (disarmed, still)', confirm: true },
  { cmd: 'level none', label: 'Forget level', what: 'back to the sensor\'s own level' },
  { cmd: 'safety off', label: 'Safety off', what: 'no level check when arming, no crash cut-off (bench work)', confirm: true },
  { cmd: 'safety on', label: 'Safety on', what: 'the checks back on' },
  { cmd: 'status', label: 'Status', what: 'what the Pi\'s program and the ESP32 are doing' },
  { cmd: 'health', label: 'Health', what: 'the Pi\'s health checks' },
  { cmd: 'esp32 show', label: 'ESP32 settings', what: 'the flight controller\'s settings lines' },
  { cmd: 'esp32 reboot', label: 'Restart ESP32', what: 'the flight controller starts again (disarmed only)', confirm: true },
  { cmd: 'restart', label: 'Restart dfb_pi', what: 'the Pi\'s program starts again with its folder\'s files', confirm: true },
];
const cardKind = b => CARD_ACT_KIND[b.act] || 'text';
const cardCoreName = () => { const b = boardOf('core'); return b ? b.name : 'the flight controller'; };
// Who takes a line of text: the radio's board, and on a Pi, the lines it passes on to the ESP32 (dfb_pi: esp32 …,
// level, safety; wiring and airframe send its files there). { to, via, what }
function cardTextRoute(line) {
  const r = boardOf('tlm'), w = String(line || '').trim(), word = w.split(/\s+/)[0] || '';
  if (!r) return { to: '', what: 'this design has no radio' };
  if (BOARD_KINDS[r.kind].mcu) return { to: r.name, what: 'a settings line' };
  const known = CARD_CMDS.find(c => c.cmd === w);
  if (/^(esp32|level|safety)$/.test(word)) return { to: cardCoreName(), via: r.name, what: known ? known.what : word === 'esp32' ? 'a settings line, passed on' : '' };
  if (/^(wiring|airframe)$/.test(word)) return { to: cardCoreName(), via: r.name, what: `the Pi's ${word === 'wiring' ? 'wiring.txt' : 'drone.dfa'}, sent on` };
  return { to: r.name, what: known ? known.what : '' };
}
// A button's route: { kind, to (who takes it), via, what (in a few words) }
function cardRoute(b) {
  const kind = cardKind(b), nav = boardOf('nav'), pi = cardRadioPi();
  if (b.act === 'arm') return { kind, to: cardCoreName(), what: 'armed while it\'s on' };
  if (b.act === 'fly') return nav ? { kind, to: nav.name, what: 'the navigation takes off while it\'s on, lands when off' } : { kind, to: cardCoreName(), what: 'the throttle up while it\'s on (angle mode), down when off' };
  if (b.act === 'hold') return { kind, to: nav ? nav.name : '', what: 'hold where it is' };
  if (b.act === 'home') return { kind, to: nav ? nav.name : '', what: 'fly back to where it took off' };
  if (b.act === 'send') { const r = cardTextRoute(b.cmd); return { kind, ...r, what: `sends "${b.cmd || ''}"${r.what ? ': ' + r.what : ''}` }; }
  if (b.act === 'upload') return { kind, to: pi ? `${pi.name}, then ${cardCoreName()}` : '', what: 'the design\'s files, wiring and airframe onto the drone' };
  return { kind, to: 'this page', what: 'leave the real drone (nothing is sent)' };
}
const cardRouteTo = r => r.to ? `→ ${r.to}${r.via ? ` (through ${r.via})` : ''}` : '';
// Why a button isn't on the card now ('' when it is).
function cardHiddenWhy(b) {
  if (cardShown(b)) return '';
  return { home: 'not shown: this design has no navigation', disconnect: 'shown while connected to the real drone', upload: 'shown when the radio is on a Pi',
    fly: 'not shown: the gamepad\'s throttle takes off in angle mode' }[b.act] || 'not shown now';
}
function cardExplain(b) {
  const r = cardRoute(b), k = CARD_KINDS[r.kind];
  return `${b.label}: ${k.one}, ${k.how} ${cardRouteTo(r)}. ${r.what.charAt(0).toUpperCase() + r.what.slice(1)}.${b.confirm ? ' Press twice.' : ''}`;
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
// A line for the drone: up through this page's command module (the simulator's, or live.js's own). '' when it went.
function cardTextOut(line) {
  const g = cardLive() ? live.g : brt.gnd;
  if (!g) return hasTask('tlm') ? 'the command module isn\'t running (see the Ground tab)' : 'this design has no radio: give a board the Telemetry & radio task (Computers tab)';
  if (line.length > CARD_TEXT_MAX || /[^\x20-\x7e]/.test(line)) return `a command is up to ${CARD_TEXT_MAX} plain characters`;
  if (typeof g.gnd_text !== 'function') return 'this page\'s flight computers are older than the card: reload the page';
  txtIn(g, line); const r = g.gnd_text();
  return r ? r === -1 ? 'too many commands waiting for the link: try again in a moment' : 'not sent: 1 to 57 plain characters' : '';
}
function cardSend(line) {
  line = String(line || '').trim(); if (!line) return 'type a command first';
  const why = cardTextOut(line); if (why) return why;
  card.sent = { text: line, t: cardClock(), n: gs.nmsg || 0, to: cardRouteTo(cardTextRoute(line)) };
  linkLog('↑', 'cmd', `text: ${line}`, 'the drone card', '');
  return '';
}
function cardPress(i, el) {
  const b = cardButtons()[i]; if (!b) return;
  const now = performance.now();
  if (b.confirm && !(b.act === 'upload' && cardUp.on) && !(card.ask && card.ask.i === i && now < card.ask.until)) { card.ask = { i, until: now + 3000 }; cardRender(true); return; }
  card.ask = null;
  let why = '';
  const L = cardLive();
  if (b.act === 'arm') why = L ? liveArmToggle() : cardSimArm();
  else if (b.act === 'fly') why = L ? liveFlyToggle() : cardSimFly();
  else if (b.act === 'hold') radioHold();
  else if (b.act === 'home') radioHome();
  else if (b.act === 'disconnect') { if (L) why = liveStopPress(); }
  else if (b.act === 'send') why = cardSend(b.cmd);
  else if (b.act === 'upload') { if (cardUp.on) { cardUp.stop = true; why = ''; } else why = cardUploadStart(); }
  cardNote(why || '', why ? 'bad' : '');
  if (why && el) flashEl(el);
  cardRender(true);
}
/* ───────── Upload design: the design onto the drone, through its Pi ───────── */
// The design's files go up the radio as text commands (runner/fc/file_rx.h: file NAME SIZE CRC, fd OFF BASE64, file
// end; the Pi lists the pieces that didn't come, and they go again) into the Pi's folder (dfb_pi's design/): wiring.txt
// (the flight controller's settings lines, as Install gives them), drone.dfa (the airframe), drone.dnc (the navigation's
// config) and, with the learning or the supervisor, drone.dlc. A file the Pi has already, the same, isn't sent. Then
// `wiring` puts the settings on the ESP32 and saves them (the lines that carry the Pi's own link to it stay as they
// are: a wrong one there would cut the drone off until USB), `esp32 reboot` starts it with them, and `airframe` gives
// it the airframe, which it checks against its wiring and keeps in flash. The Pi's own files are used from dfb_pi's
// next start (`restart`). Disarmed only. The simulator's Pi answers the same (boards.js piText).
const cardUp = { on: false, stop: false, said: '' };
const cardRadioPi = () => { const b = boardOf('tlm'); return b && BOARD_KINDS[b.kind] && !BOARD_KINDS[b.kind].mcu ? b : null; };
function cardUploadFiles() {
  const core = boardOf('core'), files = [];
  if (core && BOARD_KINDS[core.kind].mcu && typeof boardWiringPlan === 'function') {
    const plan = boardWiringPlan(core);
    if (plan.errors.length) throw new Error('the wiring has problems (Computers tab): ' + plan.errors.join('; '));
    files.push({ name: 'wiring.txt', data: new TextEncoder().encode(hardwareSettings(plan).join('\n') + '\n') });
  }
  files.push({ name: 'drone.dfa', data: fcAirframeBlob() }, { name: 'drone.dnc', data: navConfigBlob() });
  if (hasTask('learn') || hasTask('super')) files.push({ name: 'drone.dlc', data: piConfigBlob() });
  return files.map(f => ({ ...f, data: f.data instanceof Uint8Array ? f.data : new Uint8Array(f.data) }));
}
function cardUploadStart() {
  if (!cardRadioPi()) return 'Upload design goes through the drone\'s Pi, and this design\'s radio isn\'t on one (with no Pi: Computers → Install, by USB)';
  if (cardLive() ? live.state !== 'up' : !brt.ready) return cardLive() ? 'not connected to the drone yet' : 'the simulator isn\'t running yet';
  if (cardLive() ? live.arm : simArmed()) return 'disarm first';
  let files; try { files = cardUploadFiles(); } catch (e) { return 'can\'t make the files: ' + e.message; }
  Object.assign(cardUp, { on: true, stop: false, said: 'Upload design: starting' });
  linkLog('↑', 'cmd', 'upload design', files.map(f => `${f.name} ${f.data.length} B`).join(', '), '');
  cardUpload(files).then(r => { cardUp.on = false; cardNote(r.text, r.tone); linkLog('↓', 'cmd', 'upload design', r.text, r.tone === 'bad' ? 'bad' : ''); cardRender(true); });
  return '';
}
// (time as the drone's clock goes: the simulator's stops with the page, and so do the waits)
const cardUpTick = () => new Promise(r => setTimeout(r, 40));
async function cardUpSleep(s) { const t0 = cardClock(); while (cardClock() - t0 < s) { if (cardUp.stop) throw new Error('stopped'); await cardUpTick(); } }
function cardUpSay(s) { cardUp.said = s; cardRender(true); }
async function cardUpSend(line) {
  const t0 = cardClock();
  for (;;) {
    if (cardUp.stop) throw new Error('stopped');
    if (cardLive() && live.state !== 'up') throw new Error('the link to the drone is gone');
    const why = cardTextOut(line);
    if (!why) return;
    if (!/too many commands/.test(why) || cardClock() - t0 > 15) throw new Error(why);
    await cardUpTick();
  }
}
// The first message (oldest first) since n0 that matches re: its text, or '' after s seconds.
async function cardUpWait(n0, re, s) {
  const t0 = cardClock();
  for (;;) {
    const k = Math.min(gs.log.length, (gs.nmsg || 0) - n0), got = gs.log.slice(0, Math.max(0, k)).reverse().find(m => re.test(m.text));
    if (got) return got.text;
    if (cardClock() - t0 > s) return '';
    if (cardUp.stop) throw new Error('stopped');
    await cardUpTick();
  }
}
const cardUpB64 = b => { let s = ''; for (const x of b) s += String.fromCharCode(x); return btoa(s); };
const cardUpEsc = s => s.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
async function cardUpFile(f) {
  const PIECE = 36, all = [], crc = crc32(f.data).toString(16), name = cardUpEsc(f.name);
  for (let o = 0; o < f.data.length; o += PIECE) all.push(o);
  let n0 = gs.nmsg || 0; await cardUpSend(`file ${f.name} ${f.data.length} ${crc}`);
  const first = await cardUpWait(n0, new RegExp(`^file ${name}: |^file: `), 8);
  if (!first) throw new Error(`no answer from the Pi about ${f.name}: is its dfb_pi new enough (it takes files since this LiftLab)?`);
  if (/the same is here/.test(first)) return 'same';
  if (!/send them/.test(first)) throw new Error(first);
  let todo = all;
  for (let round = 1; round <= 8; round++) {
    for (let i = 0; i < todo.length; i++) {
      cardUpSay(`Upload design: ${f.name}, ${round > 1 ? 'again, ' : ''}${i + 1} of ${todo.length} pieces`);
      await cardUpSend(`fd ${todo[i]} ${cardUpB64(f.data.subarray(todo[i], todo[i] + PIECE))}`);
    }
    n0 = gs.nmsg || 0; await cardUpSend('file end');
    const r = await cardUpWait(n0, new RegExp(`^file ${name}: (saved|missing|damaged|couldn't|done)|^file end: `), 10);
    if (/: saved/.test(r)) return 'saved';
    if (/: missing/.test(r)) { todo = (r.split(':').pop().match(/\d+/g) || []).map(Number).filter(o => all.includes(o)); if (!todo.length) todo = all; continue; }
    if (/: damaged/.test(r)) { todo = all; continue; }
    throw new Error(r || `no answer when ${f.name} was all sent`);
  }
  throw new Error(`${f.name}: pieces kept going missing; is the link weak?`);
}
async function cardUpload(files) {
  const sim = !cardLive(), saved = {};
  try {
    for (const f of files) saved[f.name] = await cardUpFile(f);
    if (files.some(f => f.name === 'wiring.txt')) {
      cardUpSay('Upload design: the wiring to the ESP32');
      let n0 = gs.nmsg || 0; await cardUpSend('wiring wiring.txt');
      const w = await cardUpWait(n0, /^wiring: /, 8);
      if (!/settings lines/.test(w)) throw new Error(w || 'no answer to wiring from the Pi');
      const e = await cardUpWait(n0, /^esp32: (saved|couldn't save)/, 10);
      if (!/: saved/.test(e)) throw new Error(e || 'the ESP32 didn\'t say it saved the wiring: is it connected to the Pi?');
      const k = Math.min(gs.log.length, (gs.nmsg || 0) - n0), bad = gs.log.slice(0, Math.max(0, k)).reverse().find(m => /^esp32: /.test(m.text) && !/^esp32: (saved|set )/.test(m.text));
      if (bad) throw new Error(`the ESP32 refused a wiring line (it saved the rest; nothing restarted): ${bad.text.replace(/^esp32: /, '')}`);   // (its answers but "set …": dfb_pi holds those back)
      if (cardLive() ? live.arm : simArmed()) throw new Error('it was armed: the ESP32 wasn\'t restarted (its new wiring is saved for its next start)');
      cardUpSay('Upload design: restarting the ESP32 with it');
      await cardUpSend('esp32 reboot'); await cardUpSleep(sim ? 0.5 : 6);
    }
    let a = '';
    for (let k = 0; k < 3 && !/loaded/.test(a); k++) {
      cardUpSay('Upload design: the airframe to the ESP32');
      const n0 = gs.nmsg || 0; await cardUpSend('airframe drone.dfa');
      const p = await cardUpWait(n0, /^airframe /, 8);
      if (!/motors/.test(p)) throw new Error(p || 'no answer to airframe from the Pi');
      a = await cardUpWait(n0, /^esp32: airframe/, 6);
      if (!/loaded/.test(a) && k < 2) await cardUpSleep(3);            // (the ESP32 may still be starting)
    }
    if (!/loaded and saved/.test(a)) throw new Error(a ? a.replace(/^esp32: /, 'the ESP32: ') : 'the ESP32 didn\'t say it took the airframe');
    const pi = files.filter(f => f.name !== 'wiring.txt' && f.name !== 'drone.dfa' && saved[f.name] === 'saved').map(f => f.name);
    const same = files.filter(f => saved[f.name] === 'same').length;
    return { text: `Uploaded: the ESP32 has the wiring and the airframe${same ? ` (${same} of the files were on the Pi already)` : ''}${pi.length ? `. The Pi's ${pi.join(' and ')}: used from dfb_pi's next start (send restart)` : ''}`, tone: 'good' };
  } catch (e) {
    return { text: `Upload design ${e.message === 'stopped' ? 'stopped' : 'failed: ' + e.message}`, tone: e.message === 'stopped' ? '' : 'bad' };
  }
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
  cardRenderCmdTo();
  cardRenderSaid();
  if (card.edit && $('#cardEditor').dataset.sig !== JSON.stringify(cardButtons())) cardRenderEditor();   // (an undo, another tab's edit)
  const nav = hasTask('nav'), pad = L && !!live.gp, slow = L && live.slow ? ` This page stood still ${live.slow}×: keep this tab in front.` : '';
  if (cardUp.on) cardNote(cardUp.said, '');                           // (its progress stays while it runs)
  let note = card.msg && now - card.msgT < 6000 ? card.msg.charAt(0).toUpperCase() + card.msg.slice(1) + '.' : '';
  const hb = card.hover != null && !note && cardButtons()[card.hover];
  if (hb) note = cardExplain(hb);                                     // (the button under the pointer, or in focus: what it is)
  if (!note) note = L ? (!nav && !pad ? `Angle mode: ${live.landing ? 'landing (it sinks until the height stops changing).' : live.fly ? 'W/S climb and sink (let go: it holds the height), A/D turn, the arrows lean.' : 'Arm, then Take off: the keys and pads fly it, W/S climb.'}`
    : `${pad ? `Gamepad: ${live.gp.replace(/\s*\(.*$/, '')}. ` : ''}${nav ? 'The keys and pads move it; Space holds, H flies home.' : ''}`) + slow
    : brt.pilot.manual ? 'The card\'s switches fly it until the next reset.' : 'The simulator\'s pilot arms and takes off by itself; the switches here take over.';
  const n = $('#liveNote'); setText(n, note); n.dataset.tone = card.msg && now - card.msgT < 6000 ? card.msgTone : '';
  if (card.pos) cardPlace();                                           // (its height changes with what it shows)
}
const fcStateWord = () => ['disarmed', 'armed', 'failsafe', 'crashed', 'motor test'][brt.fcState] || '';
// The buttons, a group for each kind (CARD_KINDS: the switches, then the momentary ones, the commands, the page's own),
// each group saying how it goes and to which board; a button in focus or under the pointer says what it does, below.
function cardGroups(list) {
  const g = new Map();
  list.forEach((b, i) => { const k = cardKind(b); if (!g.has(k)) g.set(k, []); g.get(k).push(i); });
  return CARD_KIND_ORDER.filter(k => g.has(k)).map(k => ({ kind: k, items: g.get(k) }));
}
function cardGroupTo(list, items) {
  const to = [...new Set(items.filter(i => cardShown(list[i])).map(i => cardRoute(list[i])).filter(r => r.to).map(r => r.via ? `${r.to} (through ${r.via})` : r.to))];
  return to.length ? '→ ' + to.join(', ') : '';
}
function cardRenderButtons() {
  const box = $('#cardBtns'), list = cardButtons(), now = performance.now();
  if (card.ask && now > card.ask.until) card.ask = null;
  const groups = cardGroups(list);
  const sig = JSON.stringify([list, cardLive(), groups.map(g => cardGroupTo(list, g.items)), list.map(b => cardShown(b))]);
  if (box.dataset.sig !== sig) {
    box.dataset.sig = sig; box.textContent = '';
    for (const g of groups) {
      const K = CARD_KINDS[g.kind], row = el('div', { class: 'card-btns' }), to = cardGroupTo(list, g.items);
      for (const i of g.items) {
        const b = list[i];
        const x = el('button', { class: 'btn' + (b.act === 'arm' || b.act === 'fly' ? ' tog' : '') + (b.act === 'arm' ? ' live-arm' : '') + (b.act === 'send' ? ' is-cmd' : ''), type: 'button', 'data-i': i, title: cardExplain(b) });
        x.addEventListener('mousedown', e => e.preventDefault());       // (a click leaves the keyboard where it was: Space mustn't press it again)
        x.addEventListener('click', () => cardPress(i, x));
        for (const ev of ['pointerenter', 'focus']) x.addEventListener(ev, () => { card.hover = i; cardRender(true); });
        for (const ev of ['pointerleave', 'blur']) x.addEventListener(ev, () => { if (card.hover === i) { card.hover = null; cardRender(true); } });
        row.append(x);
      }
      const sec = el('div', { class: 'card-grp', 'data-kind': g.kind, role: 'group', 'aria-label': `${K.name}: ${K.how} ${to}` },
        el('div', { class: 'card-grp-h', title: K.hint }, el('b', { text: K.name }), el('span', { text: K.how }), to ? el('span', { class: 'card-to', text: to }) : null), row);
      sec.hidden = !g.items.some(i => cardShown(list[i]));
      box.append(sec);
    }
  }
  box.querySelectorAll('button').forEach(x => {
    const i = +x.dataset.i, b = list[i]; if (!b) return;
    x.hidden = !cardShown(b);
    const on = (b.act === 'arm' || b.act === 'fly') && cardSwitchOn(b.act);
    if (b.act === 'arm' || b.act === 'fly') x.setAttribute('aria-pressed', String(on));
    const asking = card.ask && card.ask.i === i;
    x.classList.toggle('is-asking', !!asking);
    setText(x, asking ? `${b.label}? Again` : b.act === 'upload' && cardUp.on ? 'Stop upload' : b.act === 'disconnect' && cardLive() && live.stopAsk && now < live.stopAsk ? 'Really disconnect?' : on && b.on ? b.on : b.label);
  });
  if (cardLive() && live.stopAsk && now > live.stopAsk) live.stopAsk = 0;
}
// The command line: who takes what's typed, and the Pi's commands as suggestions.
function cardRenderCmdTo() {
  const inp = $('#cardCmdIn'), t = $('#cardCmdTo'), v = inp.value, r = cardTextRoute(v), r0 = cardTextRoute('');
  const ph = r0.to ? `A command for ${r0.to}: level, safety off, ${cardRadioPi() ? 'status' : 'show'}…` : 'A command (this design has no radio)';
  if (inp.placeholder !== ph) inp.placeholder = ph;
  setText(t, r.to && v.trim() ? `→ ${r.to}` : '');                   // (what's typed: who takes it, as it's typed) t.title = r.to ? `Typed text is a one-off command for ${r.to}${r.via ? `, passed on by ${r.via}` : ''}${r.what ? `: ${r.what}` : ''}. Its answer shows below.` : r.what;
  const dl = $('#cardCmdList'), sig = cardRadioPi() ? 'pi' : 'mcu';
  if (dl.dataset.sig !== sig) { dl.dataset.sig = sig; dl.textContent = ''; if (sig === 'pi') for (const c of CARD_CMDS) dl.append(el('option', { value: c.cmd, label: c.what })); }
}
// What the drone said since the last command: the messages that came after it (the newest last).
function cardRenderSaid() {
  const box = $('#cardSaid'), s = card.sent;
  if (s && (gs.nmsg || 0) < s.n) card.sent = null;                    // (the ground station started again)
  const items = [];
  if (card.sent) {
    const k = Math.min(4, (gs.nmsg || 0) - s.n), got = gs.log.slice(0, Math.max(0, k)).reverse();
    items.push({ text: `› ${s.text}${s.to ? `   ${s.to}` : ''}`, cls: 'card-sent' });
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
// In the same groups as the card, each row saying what it does and who takes it; a button moves within its group (a
// change of what it does moves it to that one's group). Add a button from a menu: the switches, the Pi's commands, your
// own command, the page's actions (the ones your buttons haven't, marked).
const CARD_ACT_LABEL = { arm: 'Arm switch', fly: 'Take-off switch', hold: 'Hold', home: 'Fly home', send: 'Command (text)', upload: 'Upload design', disconnect: 'Disconnect' };
const CARD_ACT_NEW = { arm: { label: 'Arm', on: 'Armed', act: 'arm' }, fly: { label: 'Take off', on: 'Land', act: 'fly' }, hold: { label: 'Hold', act: 'hold' }, home: { label: 'Home', act: 'home' },
  upload: { label: 'Upload design', act: 'upload', confirm: true }, disconnect: { label: 'Disconnect', act: 'disconnect' } };
function cardSetButtons(list) {
  const C = computers(); C.ground.buttons = fixCardButtons(list);
  undoKey = 'cardButtons'; save(); $('#cardEditor').dataset.sig = JSON.stringify(cardButtons()); cardRender(true);
}
function cardRenderEditor() {
  const box = $('#cardEditor'); box.hidden = !card.edit; if (!card.edit) return;
  box.textContent = ''; box.dataset.sig = JSON.stringify(cardButtons());
  const list = cardButtons().map(b => ({ ...b }));
  const commit = () => cardSetButtons(list), redo = () => { commit(); cardRenderEditor(); };
  box.append(el('p', { class: 'hint', text: 'The design\'s buttons (its command module\'s). Each is one of four kinds, and goes to one of the design\'s boards (Computers tab):' }));
  const legend = el('dl', { class: 'card-kinds' });
  for (const k of CARD_KIND_ORDER) legend.append(el('dt', { 'data-kind': k, text: CARD_KINDS[k].name }), el('dd', { text: CARD_KINDS[k].hint }));
  box.append(legend);
  const missing = Object.keys(CARD_ACT_NEW).filter(a => !list.some(b => b.act === a) && CARD_BUTTONS_DEFAULT.some(b => b.act === a));
  if (missing.length) box.append(el('p', { class: 'hint card-missing', text: `Not among your buttons: ${missing.map(a => CARD_ACT_NEW[a].label).join(', ')} (add from the menu below).` }));
  for (const g of cardGroups(list)) {
    const K = CARD_KINDS[g.kind];
    box.append(el('div', { class: 'card-grp-h', 'data-kind': g.kind }, el('b', { text: K.name }), el('span', { text: K.how })));
    g.items.forEach((i, j) => {
      const b = list[i], n = `Button "${b.label}"`;
      const item = el('div', { class: 'card-item', 'data-kind': g.kind }), row = el('div', { class: 'card-row' });
      const lab = el('input', { type: 'text', value: b.label, maxlength: 24, 'aria-label': `${n}: its label` });
      lab.addEventListener('change', () => { b.label = lab.value; commit(); });
      const act = el('select', { 'aria-label': `${n}: what it does` });
      for (const k of CARD_KIND_ORDER) {
        const og = el('optgroup', { label: CARD_KINDS[k].name });
        for (const a of CARD_ACTS) if (CARD_ACT_KIND[a] === k) og.append(el('option', { value: a, text: CARD_ACT_LABEL[a] }));
        act.append(og);
      }
      act.value = b.act; act.addEventListener('change', () => { b.act = act.value; if (b.act === 'send' && !b.cmd) b.cmd = 'status'; redo(); });
      row.append(lab, act);
      const route = el('div', { class: 'card-route' });
      const showRoute = () => {
        const r = cardRoute(b), why = cardHiddenWhy(b);
        route.textContent = '';
        route.append(el('span', { class: 'card-chip', 'data-kind': r.kind, text: CARD_KINDS[r.kind].one }), el('span', { class: 'card-to', text: cardRouteTo(r) || '→ (no board for it in this design)' }), el('span', { text: r.what }));
        if (why) route.append(el('span', { class: 'card-off', text: why }));
      };
      if (b.act === 'send') {
        const c = el('input', { type: 'text', class: 'mono', value: b.cmd || '', maxlength: CARD_TEXT_MAX, spellcheck: 'false', placeholder: 'level', list: 'cardCmdList', 'aria-label': `${n}: the text it sends` });
        c.addEventListener('input', () => { b.cmd = c.value; showRoute(); });
        c.addEventListener('change', () => { b.cmd = c.value; commit(); });
        row.append(c);
      } else if (b.act === 'arm' || b.act === 'fly') {
        const o = el('input', { type: 'text', value: b.on || '', maxlength: 24, placeholder: 'label while on', 'aria-label': `${n}: its label while on` });
        o.addEventListener('change', () => { b.on = o.value; commit(); }); row.append(o);
      } else row.append(el('span'));
      const cf = el('label', { class: 'card-confirm', title: 'Press twice: the first press asks, a second within 3 s does it' }); const cb = el('input', { type: 'checkbox' }); cb.checked = !!b.confirm;
      cb.addEventListener('change', () => { b.confirm = cb.checked; commit(); }); cf.append(cb, ' confirm');
      const mv = (d, t) => {
        const k = g.items[j + d], x = el('button', { type: 'button', class: 'card-ic', title: t, 'aria-label': `${n}: ${t}`, text: d < 0 ? '↑' : '↓' });
        x.disabled = k == null;
        x.addEventListener('click', () => { [list[i], list[k]] = [list[k], list[i]]; redo(); }); return x;
      };
      const del = el('button', { type: 'button', class: 'card-ic', title: 'Remove it', 'aria-label': `${n}: remove it`, text: '✕' });
      del.addEventListener('click', () => { list.splice(i, 1); redo(); });
      row.append(cf, mv(-1, 'Move up'), mv(1, 'Move down'), del);
      showRoute();
      item.append(row, route); box.append(item);
    });
  }
  const add = el('select', { class: 'card-add', 'aria-label': 'Add a button' });
  add.append(el('option', { value: '', text: 'Add a button…' }));
  const has = a => list.some(b => b.act === a), opt = (v, t) => el('option', { value: v, text: t });
  const og = (k, ...o) => { const x = el('optgroup', { label: `${CARD_KINDS[k].name}: ${CARD_KINDS[k].how}` }); x.append(...o); return x; };
  const actOpt = a => opt('act:' + a, CARD_ACT_NEW[a].label + (has(a) ? '' : ' (not on the card)'));
  add.append(og('switch', actOpt('arm'), actOpt('fly')), og('pulse', actOpt('hold'), actOpt('home')),
    og('text', ...(cardRadioPi() ? CARD_CMDS.map(c => opt('cmd:' + c.cmd, `${c.label} (${c.cmd})`)) : []), opt('cmd:', 'Your own command…')),
    og('page', actOpt('upload'), actOpt('disconnect')));
  add.addEventListener('change', () => {
    const v = add.value; if (!v) return;
    if (v.startsWith('act:')) list.push({ ...CARD_ACT_NEW[v.slice(4)] });
    else { const c = CARD_CMDS.find(x => x.cmd === v.slice(4)); list.push(c ? { label: c.label, act: 'send', cmd: c.cmd, ...(c.confirm ? { confirm: true } : {}) } : { label: 'Command', act: 'send', cmd: '' }); }
    redo();
    const nb = list[list.length - 1], last = [...box.querySelectorAll(`.card-item[data-kind="${cardKind(nb)}"]`)].pop();   // (the new one: last in its group)
    const inp = last && (nb.act === 'send' && !nb.cmd ? last.querySelector('input.mono') : last.querySelector('input'));
    if (inp) { inp.focus(); inp.select(); }
  });
  const def = el('button', { type: 'button', class: 'btn', text: 'Reset to defaults' });
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
  $('#cardCmdIn').addEventListener('input', cardRenderCmdTo);
  for (const x of bar.querySelectorAll('.card-head button')) x.addEventListener('mousedown', e => e.preventDefault());
  window.addEventListener('resize', cardPlace);
  if (typeof ResizeObserver === 'function') new ResizeObserver(() => cardPlace()).observe(bar);   // (it grew: unfolded, the editor, an answer)
  cardFoldSet(card.fold);
  setInterval(() => { if (!cardLive()) cardRender(); }, 200);         // (connected: live.js renders it with each frame)
})();
