'use strict';
// The Ground station tab: what came down the radio (elrs.js decodes it into gs), shown with the data displays of
// gs-widgets.js, and the radio's settings. Nothing here reads the simulator's truth or the boards directly: if the
// link can't carry it, it isn't here.

const SUP_WHY = ['', 'a motor is running hot', 'the battery is hot', 'lift margin low', 'a motor has failed', 'lift margin very low', 'a battery cell has failed',
  'battery below 20%', 'battery voltage low', 'roll and pitch can no longer be held', 'not enough lift to stay up', 'battery nearly empty', 'battery overheating'];
const SUP_MODE = ['normal', 'careful', 'returning home', 'landing'];
// paused: the log as it was when paused. items: the link log's lines on screen, by entry id ({ li, btn, raw }), kept
// as they are while new ones come (so a line keeps its focus, its open bytes and a press in progress).
const GS_UI = { built: false, w: {}, next: 0, logShow: {}, logN: -1, logKey: '', items: new Map(), open: new Set(), paused: null, clearId: 0, cfgNote: '' };

function buildGs() {
  const pane = $('#paneGs'); pane.textContent = ''; const W = GS_UI.w = {};
  const sec = (title, small, src) => { const s = el('section', { class: 'sec gs-sec' }); s.append(el('h2', { 'data-src': src || 'tlm' }, title, small ? el('small', { text: small }) : '')); pane.append(s); return s; };
  pane.append(srcLegend(['tlm', 'gnd', 'sim', 'you']));
  const grid = (...kids) => el('div', { class: 'gs-grid' }, ...kids);

  // the command module's alerts (groundAlerts)
  const al = sec('Alerts', 'from the command module', 'gnd');
  W.alert = GSW.badge(); al.append(el('div', { class: 'gs-moderow' }, W.alert.el), el('p', { class: 'hint', id: 'gsCmdNote' }));

  // the radio
  const r = sec('Radio link', 'ExpressLRS 2.4 GHz', 'tlm gnd sim you');
  const sel = (id, label, opts, get, set) => {
    const s = el('select', { id }); for (const [v, t] of opts) { const o = el('option', { value: String(v), text: t }); if (String(get()) === String(v)) o.selected = true; s.append(o); }
    s.addEventListener('change', () => { set(s.value); save(); boardsRadioCfg(); });
    return el('label', { class: 'gs-sel', for: id }, el('span', { class: 'lbl', text: label }), s);
  };
  r.append(el('div', { class: 'gs-row' },
    sel('gsRate', 'Packet rate', Object.keys(ELRS_RATES).map(k => [k, k + ' Hz']), () => radioCfg.rate, v => { radioCfg.rate = +v; }),
    sel('gsRatio', 'Telemetry', ELRS_RATIOS.map(k => [k, '1:' + k]), () => radioCfg.ratio, v => { radioCfg.ratio = +v; }),
    sel('gsPower', 'Power', ELRS_POWERS.map(k => [k, k + ' mW']), () => radioCfg.power, v => { radioCfg.power = +v; })),
    el('p', { class: 'hint warn', id: 'gsCfgNote', hidden: '' }));
  const extra = numField('gsExtra', { label: 'Extra path loss (distance, walls, interference)', min: 0, max: 120, step: 1, u: 'dB', dp: 0 }, () => radioCfg.extra, v => { radioCfg.extra = v; save(); });
  r.append(el('p', { class: 'hint warn' }, srcDot('calc'), el('span', { id: 'gsRoom' })), extra.node, el('p', { class: 'hint' }, srcDot('sim'), el('span', { id: 'gsEquiv' })));
  W.linkUp = GSW.bar('Uplink LQ', { min: 0, max: 100, unit: '%', tone: v => v < 50 ? 'bad' : v < 80 ? 'warn' : '' });
  W.linkDown = GSW.bar('Telemetry LQ', { min: 0, max: 100, unit: '%', tone: v => v < 50 ? 'bad' : v < 80 ? 'warn' : '' });
  W.rssi = GSW.value('RSSI', { unit: 'dBm', dp: 0 }); W.snr = GSW.value('SNR', { unit: 'dB', dp: 0 });
  W.thru = GSW.value('Telemetry', { unit: 'B/s', dp: 0 }); W.budget = GSW.value('Room', { unit: 'B/s', dp: 0 });
  W.age = GSW.value('Last frame', { unit: 's ago', dp: 1, tone: v => v > 1.5 ? 'bad' : v > 0.5 ? 'warn' : '' });
  W.budget.el.firstChild.prepend(srcDot('calc'));   // (what the settings allow, worked out on the ground)
  r.append(el('p', { class: 'hint' }, srcDot('tlm'), 'Link quality, RSSI and SNR as the transmitter module reports them to the command module; the rest decoded by it from the frames that came down.'),
    W.linkUp.el, W.linkDown.el, grid(W.rssi.el, W.snr.el, W.thru.el, W.budget.el, W.age.el));
  W.chans = GSW.columns('Channels the command module sends (1–9)'); W.chans.el.firstChild.prepend(srcDot('gnd'));
  r.append(W.chans.el, el('p', { class: 'hint', id: 'gsRadioNote' }));

  // inside the link (the simulator's view: a real ground station can't see this)
  const lg = sec('Link log', 'inside the simulated radio', 'sim');
  GS_UI.stats = el('table', { class: 'gs-stats', title: 'Channels: the command module makes a frame every 4 ms; each uplink packet carries the newest, so the rest are superseded, not lost. Latency: from when it was made to when the other end got it. Lost: packets that didn\'t get through (sent again for commands and telemetry). Superseded: a newer frame of the same kind replaced one still waiting in the drone\'s receiver (it keeps only the newest of each). Dropped: frames the receiver threw away, its queue full, and status texts replaced by newer ones; commands the transmitter module threw away while its link was down.' });
  GS_UI.statCells = [['', 'per second', 'latency avg / max', 'lost'], ['↑ channels', '', '', ''], ['↑ commands', '', '', ''], ['↓ telemetry', '', '', '']].map((r, i) => {
    const cells = r.map(c => el(i ? 'td' : 'th', { text: c })); GS_UI.stats.append(el('tr', {}, ...cells)); return cells;
  });
  GS_UI.stats.hidden = true;
  lg.append(GS_UI.stats, el('p', { class: 'hint', text: '↑ reached the drone\'s receiver, ↓ reached the command module, with how long it took. The sticks and switches are channels, sent in every uplink packet; commands are one-off frames. Click a line, or press Enter on it, for its bytes.' }));
  const kinds = [['stick', 'Sticks & switches'], ['cmd', 'Commands'], ['msg', 'Messages & mode'], ['link', 'Link & drops'], ['all', 'Every frame']];
  const row = el('div', { class: 'gs-logbar' }), chips = el('div', { class: 'gs-filters', role: 'group', 'aria-label': 'Show in the log' });
  for (const [k, label] of kinds) {
    const on = () => k === 'all' ? radioLogAll : GS_UI.logShow[k] !== false;
    const b = el('button', { type: 'button', class: 'gs-filter', 'aria-pressed': String(on()), text: label });
    if (k === 'all') b.title = 'Log every telemetry frame too (busy). Off: only commands, switches, messages, the flight mode and link events.';
    b.addEventListener('click', () => { if (k === 'all') radioLogAll = !radioLogAll; else GS_UI.logShow[k] = !on(); b.setAttribute('aria-pressed', String(on())); renderGs(true); });
    chips.append(b);
  }
  // Pause / Resume: built once; a press changes its icon and label, the count of what came since changes only its own
  // text (so a long press isn't lost to a button rebuilt under it). The label says what it does: no aria-pressed.
  const PAUSE = 'M4 3h3v10H4zM9 3h3v10H9z', PLAY = 'M5 3l8 5-8 5z', svgNs = 'http://www.w3.org/2000/svg';
  const ico = document.createElementNS(svgNs, 'svg'), path = document.createElementNS(svgNs, 'path'); ico.setAttribute('viewBox', '0 0 16 16'); ico.setAttribute('aria-hidden', 'true'); ico.append(path);
  const pzLabel = el('span'), pzNew = el('span', { class: 'gs-new', hidden: '' });
  const pz = GS_UI.pauseBtn = el('button', { type: 'button', class: 'btn btn-sm gs-pause' }, ico, pzLabel, pzNew);
  let pzWas;
  GS_UI.setPause = () => {
    const p = GS_UI.paused;
    if (pzWas !== p) {
      pzWas = p; path.setAttribute('d', p ? PLAY : PAUSE); pzLabel.textContent = p ? 'Resume' : 'Pause'; pz.classList.toggle('is-paused', !!p);
      pz.title = p ? `Paused at ${p.t.toFixed(2)} s: the link keeps running, the log stands still` : 'Hold the log still to read it (the link keeps running)';
    }
    let fresh = 0; if (p) { const want = logWant(); for (const e of radio.log) { if (e.id <= p.n) break; if (want(e)) fresh++; } }   // (newest first: what came since, as the filters show it)
    const s = fresh > 0 ? `${fresh > 99 ? '99+' : fresh} new` : '';
    if (pzNew.textContent !== s) pzNew.textContent = s; pzNew.hidden = !s;
  };
  pz.addEventListener('click', () => { GS_UI.paused = GS_UI.paused ? null : { log: radio.log.slice(), n: radio.logN, t: radio.t }; GS_UI.setPause(); renderGs(true); });
  const clr = el('button', { type: 'button', class: 'btn btn-sm', text: 'Clear', title: 'Start the log afresh from here' });
  clr.addEventListener('click', () => { GS_UI.clearId = radio.logN; GS_UI.paused = null; GS_UI.open.clear(); GS_UI.setPause(); renderGs(true); });
  GS_UI.setPause();
  row.append(chips, el('div', { class: 'gs-logbtns' }, pz, clr));
  GS_UI.linkLog = el('ol', { class: 'w-log gs-linklog' }); GS_UI.logN = -1; GS_UI.logKey = ''; GS_UI.items = new Map(); GS_UI.emptyLi = null; GS_UI.press = false;
  // while a button is held down on the list, new lines wait (they would slide the line under the pointer away before it's let go)
  GS_UI.linkLog.addEventListener('pointerdown', () => { GS_UI.press = true; });
  if (!GS_UI.pressWatch) { GS_UI.pressWatch = true; for (const k of ['pointerup', 'pointercancel', 'blur']) window.addEventListener(k, () => { if (GS_UI.press) { GS_UI.press = false; setTimeout(() => renderGs(true)); } }); }
  lg.append(row, GS_UI.linkLog);

  // flight
  const f = sec('Flight', 'from the drone');
  W.mode = GSW.badge(); W.horizon = GSW.horizon();
  W.alt = GSW.value('Height', { unit: 'm', dp: 1 }); W.vz = GSW.value('Climb', { unit: 'm/s', dp: 1 });
  W.speed = GSW.value('Speed', { unit: 'm/s', dp: 1 }); W.home = GSW.value('From home', { unit: 'm', dp: 1 });
  W.sats = GSW.value('Satellites', { dp: 0 }); W.gps = GSW.value('Position', { fmt: v => v, text: true });
  f.append(el('div', { class: 'gs-moderow' }, W.mode.el), W.horizon.el, grid(W.alt.el, W.vz.el, W.speed.el, W.home.el, W.sats.el), W.gps.el);

  // battery
  const b = sec('Battery');
  W.batPct = GSW.bar('Charge', { min: 0, max: 100, unit: '%', tone: v => v < 20 ? 'bad' : v < 35 ? 'warn' : '' });
  W.volts = GSW.value('Voltage', { unit: 'V', dp: 1 }); W.amps = GSW.value('Current', { unit: 'A', dp: 1 }); W.mah = GSW.value('Used', { unit: 'mAh', dp: 0 });
  const cells = () => (gs.v.super && gs.v.super.cells) || battCfg().cells;   // (the pack the pilot put in)
  GS_UI.vN = -1;
  W.vSpark = GSW.sparkline('Voltage, last minute (empty to full)', { dp: 1, n: 120, min: () => 3.2 * cells(), max: () => 4.2 * cells() });
  b.append(W.batPct.el, grid(W.volts.el, W.amps.el, W.mah.el), W.vSpark.el);

  // map
  const m = sec('Map', 'from home, north up');
  W.map = GSW.map(); m.append(W.map.el);

  // motors and the tasks
  const t = sec('Motors and the Pi\'s tasks');
  W.motors = GSW.columns('Throttle (marked: the supervisor scaled or removed it)');
  W.sup = GSW.value('Supervisor', { fmt: v => v, text: true }); W.margin = GSW.value('Lift margin', { unit: '×', dp: 2, tone: v => v < 1.35 ? 'bad' : v < 1.6 ? 'warn' : '' });
  W.learn = GSW.value('Learning', { fmt: v => v, text: true }); W.cal = GSW.bar('Calibration', { min: 0, max: 100, unit: '%' });
  t.append(W.motors.el, grid(W.sup.el, W.margin.el), W.learn.el, W.cal.el);

  // the latches: what the drone's cargo item says, and commands up
  const cg = GS_UI.cargoSec = sec('Cargo', 'the latches', 'tlm you');
  W.cargo = GSW.value('Latches', { fmt: v => v, text: true });
  GS_UI.cargoBtns = el('div', { class: 'gs-row gs-cargo' }); GS_UI.cargoN = -1;
  cg.append(W.cargo.el, GS_UI.cargoBtns, el('p', { class: 'hint', text: 'The buttons send LATCH commands up the radio: the command module queues them and they go once the link is up. Each latch\'s state comes back down in the drone\'s cargo item (closed or open, and, with a load switch, whether something hangs from it), on a change and every 2 s.' }));

  // messages
  const l = sec('Messages');
  W.log = GSW.log(40); l.append(W.log.el);
  applySrcTags(pane);
  GS_UI.built = true;
}
// The radio's settings changed: the boards' telemetry budget follows at once (as if set on both ends); nothing else
// resets. A build without tlm_link can only set them with the full tlm_setup, which starts the telemetry and the
// radio's pilot state afresh: done only disarmed on the ground; in flight they wait for the next reset (and it says so).
function boardsRadioCfg() {
  if (!brt.ready) return;                                            // (starting: the boards take radioCfg as they start)
  const ws = computers().boards.map(b => [b, brt.inst.get(b.id)]).filter(([, w]) => w);
  const say = (s, tone) => { GS_UI.cfgNote = tone ? s : ''; rnEvent(s, tone || ''); };
  if (ws.every(([, w]) => w.tlm_link)) { for (const [, w] of ws) w.tlm_link(radioCfg.rate, radioCfg.ratio); GS_UI.cfgNote = ''; return; }
  if (brt.fcState === 0 && (brt.pilot.phase === 'ground' || brt.pilot.phase === 'landed')) {
    for (const [b, w] of ws) w.tlm_setup(b.tasks.includes('tlm') ? 1 : 0, radioCfg.rate, radioCfg.ratio);
    say(`Radio: the boards took ${radioCfg.rate} Hz, telemetry 1:${radioCfg.ratio} (their telemetry started afresh, on the ground)`);
  } else say(`Radio: the boards take ${radioCfg.rate} Hz, telemetry 1:${radioCfg.ratio} at the next reset (this build can't change them in flight); until then the drone's telemetry keeps to the old budget`, 'warn');
}
// What the link log shows: by the filters, after the last Clear.
const LOG_GROUP = { stick: 'stick', switch: 'stick', cmd: 'cmd', msg: 'msg', mode: 'msg', link: 'link', drop: 'link' };
const LOG_TAG = { stick: 'STICK', switch: 'SW', cmd: 'CMD', msg: 'MSG', mode: 'MODE', link: 'LINK', drop: 'DROP', frame: 'TLM' };
function logWant() { const show = GS_UI.logShow, c = GS_UI.clearId; return e => e.id > c && (e.kind === 'frame' ? radioLogAll : show[LOG_GROUP[e.kind]] !== false); }
// One line: its summary row (a button when it has bytes: Enter, Space or a click open them below it), the bytes once opened.
function logLine(e) {
  const kids = [el('b', { text: e.t.toFixed(2) }), ' ', el('span', { class: 'gs-dir', text: e.dir }), ' ', el('span', { class: 'gs-kind', text: LOG_TAG[e.kind] || e.kind }), ' ',
    el('span', { class: 'gs-data', text: e.data }), ' ', el('span', { class: 'gs-meta', text: e.meta })];
  const li = el('li', { 'data-tone': e.tone || '' });
  if (!e.bytes) { li.append(el('div', { class: 'gs-line' }, ...kids)); return { li }; }
  li.className = 'has-raw';
  const it = { li, btn: el('button', { type: 'button', class: 'gs-line', 'aria-expanded': 'false', title: 'Show the bytes' }, ...kids), raw: null };
  li.append(it.btn);
  it.btn.addEventListener('click', () => { const open = !GS_UI.open.has(e.id); if (open) GS_UI.open.add(e.id); else GS_UI.open.delete(e.id); logLineOpen(e, it, open); });
  if (GS_UI.open.has(e.id)) logLineOpen(e, it, true);
  return it;
}
function logLineOpen(e, it, open) {
  if (open && !it.raw) { it.raw = rawView(e.bytes); it.raw.id = 'gsraw-' + e.id; it.li.append(it.raw); it.btn.setAttribute('aria-controls', it.raw.id); }
  if (it.raw) it.raw.hidden = !open;
  it.li.classList.toggle('open', open); it.btn.setAttribute('aria-expanded', String(open)); it.btn.title = open ? 'Hide the bytes' : 'Show the bytes';
}
// The list brought up to date: lines no longer shown go, new ones go in at their place, the rest stay as they are.
function renderLinkLog(has) {
  const box = GS_UI.linkLog, items = GS_UI.items;
  const list = (GS_UI.paused ? GS_UI.paused.log : radio.log).filter(logWant()).slice(0, 100), ids = new Set(list.map(e => e.id));
  for (const [id, it] of items) if (!ids.has(id)) { it.li.remove(); items.delete(id); }
  if (list.length && GS_UI.emptyLi) { GS_UI.emptyLi.remove(); GS_UI.emptyLi = null; }
  let ref = box.firstChild;
  for (const e of list) {                                            // (both newest first)
    let it = items.get(e.id); if (!it) items.set(e.id, it = logLine(e));
    if (it.li === ref) ref = ref.nextSibling; else box.insertBefore(it.li, ref);
  }
  if (!list.length) {
    if (!GS_UI.emptyLi) box.append(GS_UI.emptyLi = el('li', { class: 'muted' }));
    setText(GS_UI.emptyLi, !has ? 'The drone has no radio.' : GS_UI.clearId ? 'Cleared. New entries show here.' : 'Nothing yet.');
  }
}

function renderGs(force) {
  if ($('#paneGs').hidden) return;
  const now = performance.now(); if (!force && now < GS_UI.next) return; GS_UI.next = now + 200;
  if (!GS_UI.built) buildGs();
  const W = GS_UI.w, t = brt.t, has = hasTask('tlm');
  const gk = computers().ground, gname = `${gk.name === 'Command module' ? '' : gk.name + ', '}${BOARD_KINDS[gk.kind].label}`;
  setText($('#gsRadioNote'), has ? `The receiver is on ${boardOf('tlm').name}. Its channels fly the drone${hasTask('nav') ? ' (the sticks move its target; arm, take off, hold and home are switches)' : ' (angle mode)'}; everything below came down the link.`
    : 'No board runs the Telemetry & radio task (Computers tab): the drone has no radio, so nothing comes down and the simulator\'s pilot reaches the boards directly.');
  setText($('#gsCmdNote'), !has ? '' : !brt.gnd ? (brt.err ? `Not running: ${brt.err}` : 'Starting…') : brt.gndErr ? brt.gndErr : `Your keys and the simulator's pilot are the buttons of the command module (${gname}): it shapes them (stickInput${gs.shaped === false ? ', not answering: the raw sticks go up' : ''}), sends the channels and commands to the transmitter module, decodes what comes back and warns (groundAlerts).`);
  setText($('#gsCfgNote'), GS_UI.cfgNote); $('#gsCfgNote').hidden = !GS_UI.cfgNote;
  if (has && gs.alert) {
    const a = gs.alert, cap = s => s.charAt(0).toUpperCase() + s.slice(1);
    if (a.level) W.alert.set(cap(a.text), t, a.level >= 2 ? 'bad' : 'warn');
    else if (!brt.gndOk) W.alert.set('Not checked: groundAlerts isn\'t running', t, 'warn');   // (its program didn't load: no alerts can come)
    else W.alert.set('All fine', t, 'good');
  }
  const rf = radio.rf;
  const room = radioCfg.rate / radioCfg.ratio * 5;
  setText($('#gsRoom'), room < 40 ? `At ${radioCfg.rate} Hz with telemetry 1:${radioCfg.ratio}, only ${room < 10 ? room.toFixed(1) : Math.round(room)} bytes a second can come down (one packet in ${radioCfg.ratio}, 5 bytes each); a frame is 10–40 bytes, so values arrive seconds apart. The channels and commands go up in the other packets, so control isn't affected. 1:2 to 1:8 leaves the Ground station enough.` : '');
  $('#gsRoom').parentNode.hidden = room >= 40;
  setText($('#gsEquiv'), rf ? `Now ${rf.d.toFixed(0)} m from the handset${rf.walls ? `, ${rf.walls} building${rf.walls > 1 ? 's' : ''} in the way` : ''}; with the extra loss that is like ${fmtDist(rf.d * Math.pow(10, radioCfg.extra / 20))} in the open. Signal ${rf.rssi.toFixed(0)} dBm, the receiver needs ${ELRS_RATES[radioCfg.rate]} dBm at ${radioCfg.rate} Hz.` : '');
  const L = gs.link;
  if (has && L) { W.linkUp.set(L.upLq, L.t); W.linkDown.set(L.downLq, L.t); W.rssi.set(L.upRssi, L.t); W.snr.set(L.upSnr, L.t); }
  // throughput over the last second
  gs.rate.push([t, gs.bytes]); while (gs.rate.length > 2 && t - gs.rate[0][0] > 1) gs.rate.shift();
  const dt = gs.rate.length > 1 ? t - gs.rate[0][0] : 0;
  if (has) { W.thru.set(dt > 0 ? (gs.bytes - gs.rate[0][1]) / dt : 0, t); W.budget.set(radioCfg.rate / radioCfg.ratio * 5, t); }
  if (has) W.age.set(gs.lastAge >= 0 ? gs.lastAge : null, gs.lastAge >= 0 ? t : null);   // (the command module's own count: −1 until a frame comes)
  if (has && gs.sent) W.chans.set(gs.sent.slice(0, 9).map(v => (v + 1) / 2), t, ['Ail', 'Ele', 'Thr', 'Rud', 'Arm', 'Spd', 'Fly', 'Hold', 'Home']);

  const v = gs.v, at = gs.at;
  if (v.mode) W.mode.set(v.mode.mode, at.mode, /FS|CRASH|LOST/.test(v.mode.mode) ? 'bad' : /RTH|LAND|\*/.test(v.mode.mode) ? 'warn' : v.mode.mode === 'DISARMED' ? '' : 'good');
  if (v.attitude) W.horizon.set(v.attitude, at.attitude);
  if (v.baro) { W.alt.set(v.baro.alt, at.baro); if (v.baro.vz != null) W.vz.set(v.baro.vz, at.baro); }
  if (v.gps) { W.speed.set(v.gps.speed, at.gps); W.sats.set(v.gps.sats, at.gps); W.gps.set(`${v.gps.lat.toFixed(6)}°, ${v.gps.lon.toFixed(6)}°, ${v.gps.alt.toFixed(0)} m`, at.gps); }
  if (v.pos) { W.home.set(Math.hypot(v.pos.x, v.pos.y, v.pos.z), at.pos); if (!v.gps) W.speed.set(Math.hypot(v.pos.vx, v.pos.vy), at.pos); }
  if (v.battery) {
    W.volts.set(v.battery.volts, at.battery); W.amps.set(v.battery.amps, at.battery); W.mah.set(v.battery.mah, at.battery);
    if (v.super && v.super.soc >= 0) W.batPct.set(v.battery.pct, at.battery);
    if (gs.vN !== GS_UI.vN) { GS_UI.vN = gs.vN; W.vSpark.set(gs.vHist, at.battery); }   // (one point per battery frame, kept by gsRead)
  }
  if (v.pos || v.nav) W.map.set({ track: gs.trackXY || [], pos: v.pos ? [v.pos.x, v.pos.y] : null, target: v.nav ? [v.nav.tx, v.nav.ty] : null, heading: v.attitude ? v.attitude.yaw : 0 }, at.pos ?? at.nav);
  if (v.motors) W.motors.set(v.motors.values, at.motors, actuators().map(c => c.name), v.parts ? v.parts.values : null);
  if (v.super) { W.sup.set((SUP_MODE[v.super.mode] || `mode ${v.super.mode}`) + (v.super.why ? ': ' + (SUP_WHY[v.super.why] || `reason ${v.super.why}`) : ''), at.super); W.margin.set(v.super.margin, at.super); }
  if (v.learn) {
    const lv = v.learn;
    W.learn.set(lv.cal ? 'calibrating' : lv.throw ? ['', 'in the hand', 'thrown', 'pulsing in free fall', 'catching itself'][lv.throw] || 'throw' : `${lv.learned ? 'on the learned model' : 'on the description'}${lv.keep ? ', learning in flight' : ''}${lv.fitRot ? ` (fit ${Math.round(lv.fitRot * 100)}% rotation, ${Math.round(lv.fitForce * 100)}% force)` : ''}`, at.learn);
    W.cal.set(lv.cal ? lv.progress * 100 : 0, at.learn);
  }
  {                                                                  // the latches (names from the design, as the motors')
    const ls = latches(); GS_UI.cargoSec.hidden = !ls.length && !v.cargo;
    if (GS_UI.cargoN !== ls.length) {
      GS_UI.cargoN = ls.length; const box = GS_UI.cargoBtns; box.textContent = '';
      const b = (text, title, latch, action) => { const x = el('button', { type: 'button', class: 'btn btn-sm', text, title }); x.addEventListener('click', () => radioCommand(3, [latch, action])); return x; };
      ls.forEach((l, i) => box.append(b(`Open ${l.name}`, `LATCH ${i + 1} open: drop what it holds`, i, 0), b(`Close ${l.name}`, `LATCH ${i + 1} close: grab what's in reach`, i, 1)));
      if (ls.length > 1) box.append(b('Open all', 'LATCH all open', -1, 0));
    }
    GS_UI.cargoBtns.querySelectorAll('button').forEach(x => { x.disabled = !has || !brt.gnd; });
    if (v.cargo) W.cargo.set(v.cargo.values.map((bits, i) => `${(ls[i] || {}).name || 'latch ' + (i + 1)} ${bits & 1 ? 'closed' : 'open'}${bits & 4 ? ' (moving)' : ''}${bits & 8 ? (bits & 2 ? ', loaded' : ', empty') : ''}`).join(' · ') || 'none', at.cargo);
  }
  W.log.set(gs.log);
  if (has) {                                                         // the link's numbers, last 5 s
    const S = linkStats(t), ms = L => L ? `${Math.round(L.avg)} / ${Math.round(L.max)} ms` : '—', ps = x => x < 10 ? x.toFixed(1) : Math.round(x);
    const rows = [null,
      [`${ps(S.chSent)} carried of ${ps(S.chMade)} made`, ms(S.chLat), `${S.upLostPct.toFixed(0)}% of packets`],
      [`${S.cmds} in 5 s${S.upQueued ? ` · ${S.upQueued} waiting` : ''}`, ms(S.cmdLat), S.cmdDrop ? `${S.cmdDrop} dropped · link down` : ''],
      [`${ps(S.tlmOut)} delivered of ${ps(S.tlmIn)} written${S.tlmSuper ? ` · ${ps(S.tlmSuper)} superseded` : ''}`, ms(S.tlmLat), `${S.downLostPct.toFixed(0)}% of packets · ${S.tlmDrop} frames dropped · ${S.queued} B queued`]];
    rows.forEach((r, i) => { if (r) r.forEach((c, k) => setText(GS_UI.statCells[i][k + 1], c)); });   // (cells built once: only what changed is written)
  }
  GS_UI.stats.hidden = !has;
  if (GS_UI.paused) GS_UI.setPause();                                  // (its count of what came since)
  const key = [radioLogAll, ...['stick', 'cmd', 'msg', 'link'].map(k => GS_UI.logShow[k] !== false), GS_UI.clearId, GS_UI.paused ? GS_UI.paused.n : -1, has].join();
  if (key !== GS_UI.logKey || (radio.logN !== GS_UI.logN && !GS_UI.paused && !GS_UI.press)) { GS_UI.logKey = key; GS_UI.logN = radio.logN; renderLinkLog(has); }   // the link log: when something new came, or what it shows changed
  for (const w of Object.values(W)) w.age(t);
}
// A frame's bytes: hex with offsets, then each field.
function rawView(b) {
  const box = el('div', { class: 'gs-raw' }), hex = [];
  for (let i = 0; i < b.length; i += 16) hex.push(i.toString().padStart(3, ' ') + '  ' + Array.from(b.subarray(i, i + 16), x => x.toString(16).toUpperCase().padStart(2, '0')).join(' '));
  box.append(el('pre', { text: hex.join('\n') }));
  const dl = el('dl'); for (const [k, v] of frameFields(b)) dl.append(el('dt', { text: k }), el('dd', { text: v }));
  box.append(dl);
  return box;
}
const fmtDist = m => m >= 1000 ? `${(m / 1000).toFixed(m >= 10000 ? 0 : 1)} km` : `${Math.round(m)} m`;
