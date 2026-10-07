'use strict';
// The Ground station tab: what came down the radio (link.js decodes it into gs), shown with the data displays of
// gs-widgets.js, and the radio's settings. Nothing here reads the simulator's truth or the boards directly: if the
// link can't carry it, it isn't here.

const SUP_WHY = ['', 'a motor is running hot', 'the battery is hot', 'lift margin low', 'a motor has failed', 'lift margin very low', 'a battery cell has failed',
  'battery below 20%', 'battery voltage low', 'roll and pitch can no longer be held', 'not enough lift to stay up', 'battery nearly empty', 'battery overheating'];
const SUP_MODE = ['normal', 'careful', 'returning home', 'landing'];
// paused: the log as it was when paused. items: the link log's lines on screen, by entry id ({ li, btn, raw }), kept
// as they are while new ones come (so a line keeps its focus, its open bytes and a press in progress).
const GS_UI = { built: false, w: {}, next: 0, logShow: {}, logN: -1, logKey: '', items: new Map(), open: new Set(), paused: null, clearId: 0 };

function buildGs() {
  const pane = $('#paneGs'); pane.textContent = ''; const W = GS_UI.w = {};
  const sec = (title, small, src) => { const s = UI.section( { class: 'sec gs-sec' }); s.append(el('h2', { 'data-src': src || 'tlm' }, title, small ? el('small', { text: small }) : '')); pane.append(s); return s; };
  pane.append(srcLegend(['tlm', 'gnd', 'sim', 'you']));
  const grid = (...kids) => el('div', { class: 'gs-grid' }, ...kids);

  // the command module's alerts (groundAlerts)
  const al = sec('Alerts', 'from the command module', 'gnd');
  W.alert = GSW.badge(); al.append(el('div', { class: 'gs-moderow' }, W.alert.el), el('p', { class: 'hint', id: 'gsCmdNote' }));

  // the radio: which link, then its own settings (its model in link-*.js says which)
  const r = sec('Radio link', radioModel().label, 'tlm gnd sim you');
  GS_UI.radioSmall = r.querySelector('h2 small');
  GS_UI.radioCfg = el('div', { class: 'gs-radiocfg' }); buildGsRadio();
  r.append(GS_UI.radioCfg);
  const extra = numField('gsExtra', { label: 'Extra path loss (distance, walls, interference)', min: 0, max: 120, step: 1, u: 'dB', dp: 0 }, () => radioCfg.extra, v => { radioCfg.extra = v; save(); });
  r.append(el('p', { class: 'hint warn' }, srcDot('calc'), el('span', { id: 'gsRoom' })), extra.node, el('p', { class: 'hint' }, srcDot('sim'), el('span', { id: 'gsEquiv' })));
  W.linkUp = GSW.bar('Uplink LQ', { min: 0, max: 100, unit: '%', tone: v => v < 50 ? 'bad' : v < 80 ? 'warn' : '' });
  W.linkDown = GSW.bar('Telemetry LQ', { min: 0, max: 100, unit: '%', tone: v => v < 50 ? 'bad' : v < 80 ? 'warn' : '' });
  W.rssi = GSW.value('RSSI', { unit: 'dBm', dp: 0 }); W.snr = GSW.value('SNR', { unit: 'dB', dp: 0 });
  W.thru = GSW.value('Telemetry', { unit: 'B/s', dp: 0 }); W.budget = GSW.value('Room', { unit: 'B/s', dp: 0 });
  W.age = GSW.value('Last frame', { unit: 's ago', dp: 1, tone: v => v > 1.5 ? 'bad' : v > 0.5 ? 'warn' : '' });
  W.budget.el.firstChild.prepend(srcDot('calc'));   // (what the settings allow, worked out on the ground)
  GS_UI.kindShown = null;                                              // (renderGs says who reports the link quality, by the link)
  r.append(el('p', { class: 'hint' }, srcDot('tlm'), el('span', { id: 'gsLinkWho' })),
    W.linkUp.el, W.linkDown.el, grid(W.rssi.el, W.snr.el, W.thru.el, W.budget.el, W.age.el));
  W.chans = GSW.columns('Channels the command module sends (1–9)'); W.chans.el.firstChild.prepend(srcDot('gnd'));
  r.append(W.chans.el, el('p', { class: 'hint', id: 'gsRadioNote' }));

  // the other drones (the drone's own peer link, runner/fc/peer.h: on board, not sent down the link)
  const pe = sec('Other drones', 'the drone\'s own link to them', 'board you');
  const on = UI.input({ type: 'checkbox', id: 'gsPeers' }); on.checked = !!radioCfg.peers;
  on.addEventListener('change', () => { radioCfg.peers = on.checked ? 1 : 0; save(); peerSetup(); renderGs(true); });
  const fl = UI.input({ type: 'text', id: 'gsFleet', value: radioCfg.fleet || 'liftlab', maxlength: '31', spellcheck: 'false', autocomplete: 'off', title: 'Every drone of the fleet the same: it signs their packets. Enter or leaving the box applies it.' });
  const takeFleet = () => { const f = radioPhraseOk(fl.value); if (!f) { fl.setAttribute('aria-invalid', 'true'); return; } fl.removeAttribute('aria-invalid'); if (f !== radioCfg.fleet) { radioCfg.fleet = f; save(); peerSetup(); } };
  fl.addEventListener('change', takeFleet); fl.addEventListener('keydown', e => { if (e.key === 'Enter') { e.preventDefault(); takeFleet(); } });
  const ch = UI.select({ id: 'gsPeerCh', title: 'The Wi-Fi channel the drones meet on (the same on each). With an ESP-NOW link to the drone, that link\'s channel.' }, ...Array.from({ length: 13 }, (_, i) => el('option', { value: String(i + 1), text: 'Channel ' + (i + 1) })));
  ch.value = String(radioCfg.peerCh || 1); ch.addEventListener('change', () => { radioCfg.peerCh = +ch.value; save(); renderGs(true); });
  pe.append(el('label', { class: 'gs-peer-on' }, on, ' Talk to other drones (ESP-NOW, beside the pilot\'s link)'), UI.field({ label: 'Fleet phrase', class: 'gs-sel gs-bind', hint: 'The same on every drone of the fleet: only those hear each other.' }, fl),
    UI.field({ label: 'Wi-Fi channel', class: 'gs-sel', hint: 'On the drones: the one they meet on (an ESP-NOW link\'s own). The simulator doesn\'t model channels.' }, ch));
  GS_UI.peerBox = el('div', { class: 'gs-peers', role: 'list' }); GS_UI.peerRows = new Map();
  pe.append(GS_UI.peerBox, el('p', { class: 'hint', id: 'gsPeerNote' }));
  // this drone's fleet program (fleet.h): the pilot lets it fly the drone, or not
  GS_UI.fleetBtn = UI.button({ class: 'btn', type: 'button', id: 'gsFleetGo', 'aria-pressed': 'false', text: 'Let the fleet program fly it', title: 'Its fleet program (Computers tab, beside the navigation) then says where it flies. The sticks, hold, home or a go-to take it back.' });
  GS_UI.fleetBtn.addEventListener('click', () => { const v = fleetView(), why = fleetEngage(!(v && v.engaged)); setText($('#gsFleetNote'), why || ''); renderGs(true); });
  pe.append(el('div', { class: 'gs-fleet' }, GS_UI.fleetBtn, el('span', { class: 'gs-peer-vals', id: 'gsFleetState' })), el('p', { class: 'hint', id: 'gsFleetNote' }));

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
    const b = UI.button( { type: 'button', class: 'gs-filter', 'aria-pressed': String(on()), text: label });
    if (k === 'all') b.title = 'Log every telemetry frame too (busy). Off: only commands, switches, messages, the flight mode and link events.';
    b.addEventListener('click', () => { if (k === 'all') radioLogAll = !radioLogAll; else GS_UI.logShow[k] = !on(); b.setAttribute('aria-pressed', String(on())); renderGs(true); });
    chips.append(b);
  }
  // Pause / Resume: built once; a press changes its icon and label, the count of what came since changes only its own
  // text (so a long press isn't lost to a button rebuilt under it). The label says what it does: no aria-pressed.
  const PAUSE = 'M4 3h3v10H4zM9 3h3v10H9z', PLAY = 'M5 3l8 5-8 5z', svgNs = 'http://www.w3.org/2000/svg';
  const ico = document.createElementNS(svgNs, 'svg'), path = document.createElementNS(svgNs, 'path'); ico.setAttribute('viewBox', '0 0 16 16'); ico.setAttribute('aria-hidden', 'true'); ico.append(path);
  const pzLabel = el('span'), pzNew = el('span', { class: 'gs-new', hidden: '' });
  const pz = GS_UI.pauseBtn = UI.button( { type: 'button', class: 'btn btn-sm gs-pause' }, ico, pzLabel, pzNew);
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
  const clr = UI.button( { type: 'button', class: 'btn btn-sm', text: 'Clear', title: 'Start the log afresh from here' });
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
// The radio's settings: which link (both ends switch at once, in flight too), then its own settings, and for a packet
// link the binding phrase. Built again when the link changes, or a setting that shows or hides another (the keyboard
// focus stays on the control it was on).
function buildGsRadio() {
  const box = GS_UI.radioCfg; if (!box) return;
  if (typeof liveOn === 'function' && liveOn()) {                    // the real drone: its link is set on it
    box.textContent = ''; box.append(el('p', { class: 'hint', text: `This page talks Bluetooth LE to the drone, with the binding phrase "${radioCfg.bind}" (the drone set to radio=ble and the same bind=). The simulator's link settings come back when you disconnect.` }));
    if (GS_UI.radioSmall) GS_UI.radioSmall.textContent = radioModel().label;
    return;
  }
  keepFocus(() => {
    box.textContent = '';
    const M = radioModel(), apply = () => { save(); boardsRadioCfg(); };
    const pick = UI.choice({ id: 'gs-kind', label: 'Link', commit: true, value: radioCfg.kind,
      options: RADIO_KINDS.filter(k => RADIO_LINKS[k]).map(k => [k, RADIO_LINKS[k].label]),
      onChange: v => { if (v === radioCfg.kind || !RADIO_LINKS[v]) return; radioCfg.kind = v; apply(); buildGsRadio(); renderGs(true); } });
    box.append(el('div', { class: 'gs-row' }, UI.field({ label: 'Link', class: 'gs-sel gs-linkpick', hint: 'Both ends switch at once, in flight too.' }, pick)));
    const own = el('div', { class: 'gs-row' });
    for (const s of M.settings) {
      if (s.show && !s.show(radioCfg)) continue;
      const c = UI.choice({ id: 'gs-' + s.key, label: s.label, options: s.options, value: radioCfg[s.key],
        onChange: v => { radioCfg[s.key] = +v; apply(); if (M.settings.some(x => x.show)) buildGsRadio(); } });
      own.append(UI.field({ label: s.label, class: 'gs-sel' }, c));
    }
    if (M.packets) {                                                 // the binding phrase: taken when you leave the box or press Enter
      const say = UI.status({ class: 'hint gs-bindmsg' });
      const inp = UI.input({ type: 'text', id: 'gs-bind', value: radioCfg.bind, maxlength: '31', spellcheck: 'false', autocomplete: 'off', title: 'Enter or leaving the box applies it; Escape puts it back' });
      const take = () => {
        if (inp.value === radioCfg.bind) { say.textContent = ''; return; }
        const p = radioPhraseOk(inp.value);
        if (!p) { say.textContent = 'A phrase of 1–31 plain characters (letters, digits, punctuation, spaces inside).'; say.dataset.tone = 'bad'; inp.setAttribute('aria-invalid', 'true'); return; }
        inp.value = p; inp.removeAttribute('aria-invalid'); say.textContent = 'Both ends set up again with it.'; say.dataset.tone = '';
        if (p !== radioCfg.bind) { radioCfg.bind = p; apply(); }
      };
      inp.addEventListener('change', take);
      inp.addEventListener('keydown', e => { if (e.key === 'Enter') { e.preventDefault(); take(); } else if (e.key === 'Escape') { inp.value = radioCfg.bind; inp.removeAttribute('aria-invalid'); say.textContent = ''; } });
      own.append(UI.field({ label: 'Binding phrase', class: 'gs-sel gs-bind', hint: 'The same at both ends: it signs the packets.' }, inp, say));
    }
    box.append(own);
    // a second link at once (runner/fc/lmux.h): any other kind; both carry, the channels by the first while it has them
    const two = el('div', { class: 'gs-row' }), M2 = RADIO_LINKS[radioCfg2.kind], apply2 = () => { save(); boardsRadioCfg2(); };
    const pick2 = UI.choice({ id: 'gs-kind2', label: 'Second link', commit: true, value: radioTwo() ? radioCfg2.kind : '',
      options: [['', 'None'], ...RADIO_KINDS.filter(k => RADIO_LINKS[k] && k !== radioCfg.kind).map(k => [k, RADIO_LINKS[k].label])],
      onChange: v => { if (v === radioCfg2.kind) return; radioCfg2.kind = v; apply2(); buildGsRadio(); renderGs(true); } });
    two.append(UI.field({ label: 'Second link', class: 'gs-sel gs-linkpick', hint: 'Both carry everything: the drone takes the channels from the first while it has them, the second fills in; commands and messages arrive once. A line that goes one way only belongs here.' }, pick2));
    if (radioTwo()) for (const s2 of M2.settings) {
      if (s2.show && !s2.show(radioCfg2)) continue;
      const c = UI.choice({ id: 'gs2-' + s2.key, label: s2.label, options: s2.options, value: radioCfg2[s2.key],
        onChange: v => { radioCfg2[s2.key] = +v; apply2(); if (M2.settings.some(x => x.show)) buildGsRadio(); } });
      two.append(UI.field({ label: s2.label, class: 'gs-sel' }, c));
    }
    box.append(two);
  });
  if (GS_UI.radioSmall) GS_UI.radioSmall.textContent = radioModel().label + (radioTwo() ? ' + ' + RADIO_LINKS[radioCfg2.kind].label : '');
}
// The other drones, as this drone's peer end has them (peer-air.js peerTable): a row each, kept while it's in the
// table (so a press on its Ping button isn't lost to a row built again under it).
const PEER_TONE = ['bad', 'warn', 'warn', 'good'];
function renderPeers() {
  const box = GS_UI.peerBox; if (!box) return;
  const b = peerBoard(), w = b && brt.inst.get(b.id), list = peerOn() && w ? peerTable(w, brt.t) : [], ids = new Set(list.map(p => p.id));
  for (const [id, r] of GS_UI.peerRows) if (!ids.has(id)) { r.el.remove(); GS_UI.peerRows.delete(id); }
  for (const p of list) {
    let r = GS_UI.peerRows.get(p.id);
    if (!r) {
      r = { name: el('b'), state: el('span', { class: 'gs-peer-state' }), info: el('span', { class: 'gs-peer-info' }), vals: el('span', { class: 'gs-peer-vals' }), prog: el('span', { class: 'gs-peer-vals' }), ping: UI.button({ class: 'btn gs-peer-ping', text: 'Ping', title: 'Send it a ping: the round trip shows here' }) };
      r.ping.addEventListener('click', () => peerPing(p.id));
      r.el = el('div', { class: 'gs-peer', role: 'listitem' }, el('div', { class: 'gs-peer-head' }, r.name, r.state, r.ping), r.info, r.vals, r.prog);
      GS_UI.peerRows.set(p.id, r); box.append(r.el);
    }
    setText(r.name, p.name || 'drone ' + p.id.toString(16));
    setText(r.state, PEER_STATES[p.state]); r.state.dataset.tone = PEER_TONE[p.state];
    setText(r.info, `LQ ${Math.round(p.lq)}% · it hears us ${Math.round(p.heardUs)}% · heard ${p.heard < 1 ? 'now' : p.heard.toFixed(0) + ' s ago'}${p.rtt > 0 ? ` · ping ${(p.rtt * 1000).toFixed(0)} ms` : ''}`);
    const fv = p.vals.length >= FLEET_HEAD_N ? p.vals : null, fl = fv ? fv[10] : 0;   // (with a fleet program: where it is, its flags, what it publishes)
    setText(r.prog, !fv ? '' : `${fl & 2 ? 'its fleet program flies it' : 'its pilot flies it'}${fl & 1 ? '' : ' · no shared position (no GPS)'}${fv.length > FLEET_HEAD_N ? ' · program: ' + fv.slice(FLEET_HEAD_N).map(x => +x.toFixed(2)).join(', ') : ''}`);
    const v = p.vals; setText(r.vals, v.length >= 3 ? `${['disarmed', 'armed', 'failsafe', 'crashed'][v[0]] || 'state ' + v[0]} · battery ${v[1].toFixed(0)}% · height ${v[2].toFixed(1)} m${p.valsAge > 1 ? ` (${p.valsAge.toFixed(0)} s old)` : ''}` : 'nothing published yet');
  }
  const fvw = fleetView(), fbtn = GS_UI.fleetBtn;
  if (fbtn) {
    fbtn.disabled = !fvw || !fvw.ok; fbtn.setAttribute('aria-pressed', String(!!(fvw && fvw.engaged)));
    setText(fbtn, fvw && fvw.engaged ? 'Take it back from the fleet program' : 'Let the fleet program fly it');
    setText($('#gsFleetState'), !fvw ? 'The fleet program runs beside the navigation: this drone has none (Computers tab).' : !fvw.ok ? 'No fleet program in the navigation\'s program.'
      : `Its fleet program: ${fvw.engaged ? `flying it to ${fvw.go.map(x => x.toFixed(1)).join(' ')} m from home` : 'not flying it'} · publishes ${fvw.pub.length ? fvw.pub.map(x => +x.toFixed(2)).join(', ') : 'nothing'} · messages ${fvw.sent} sent, ${fvw.got} in${fvw.fails ? ` · failed ${fvw.fails} times` : ''}`);
  }
  const others = typeof fleet !== 'undefined' ? fleet.drones.length - 1 : 0, chSel = $('#gsPeerCh');
  if (chSel) { const en = [radioCfg, radioTwo() ? radioCfg2 : null].find(l => l && l.kind === 'espnow'); chSel.disabled = !!en || !peerOn(); if (en) chSel.value = String(en.channel || 1); }
  const hwNote = peerOn() && [radioCfg.kind, radioTwo() ? radioCfg2.kind : ''].some(k => k === 'wifi' || k === 'ble') ? ' On a drone not beside a Wi-Fi or Bluetooth link yet: Install sends peers=off.' : '';
  setText($('#gsPeerNote'), (!peerOn() ? 'Off: this drone neither sends beacons nor listens for other drones.' : list.length ? 'The drone\'s own table: who its beacons found, each one\'s link quality (ours of it, and its of us, as it says), and what it publishes (its state, battery and height). A drone out of range turns stale after a second and lost after three, and comes back by itself.'
    : others > 0 ? 'Listening: no other drone of this fleet heard yet (the others need the same fleet phrase and their peer link on).' : 'Listening. Add another airframe to the world (Add airframe) to have a drone to find.') + hwNote);
}
// What the link log shows: by the filters, after the last Clear.
const LOG_GROUP = { stick: 'stick', switch: 'stick', cmd: 'cmd', msg: 'msg', mode: 'msg', link: 'link', drop: 'link', peer: 'link' };
const LOG_TAG = { stick: 'STICK', switch: 'SW', cmd: 'CMD', msg: 'MSG', mode: 'MODE', link: 'LINK', drop: 'DROP', frame: 'TLM', peer: 'PEER' };
function logWant() { const show = GS_UI.logShow, c = GS_UI.clearId; return e => e.id > c && (e.kind === 'frame' ? radioLogAll : show[LOG_GROUP[e.kind]] !== false); }
// One line: its summary row (a button when it has bytes: Enter, Space or a click open them below it), the bytes once opened.
function logLine(e) {
  const kids = [el('b', { text: e.t.toFixed(2) }), ' ', el('span', { class: 'gs-dir', text: e.dir }), ' ', el('span', { class: 'gs-kind', text: LOG_TAG[e.kind] || e.kind }), ' ',
    el('span', { class: 'gs-data', text: e.data }), ' ', el('span', { class: 'gs-meta', text: e.meta })];
  const li = el('li', { 'data-tone': e.tone || '' });
  if (!e.bytes) { li.append(el('div', { class: 'gs-line' }, ...kids)); return { li }; }
  li.className = 'has-raw';
  const it = { li, btn: UI.button( { type: 'button', class: 'gs-line', 'aria-expanded': 'false', title: 'Show the bytes' }, ...kids), raw: null };
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
  setText($('#gsCmdNote'), typeof liveOn === 'function' && liveOn() ? `The real drone: this page is the command module (its program here, over Bluetooth LE to ${live.name}). Your keys, the pads, a gamepad and the bar's switches are its buttons; everything below came from the drone.` : !has ? '' : !brt.gnd ? (brt.err ? `Not running: ${brt.err}` : 'Starting…') : brt.gndErr ? brt.gndErr : `Your keys and the simulator's pilot are the buttons of the command module (${gname}): it shapes them (stickInput${gs.shaped === false ? ', not answering: the raw sticks go up' : ''}), sends the channels and commands to the transmitter module, decodes what comes back and warns (groundAlerts).`);
  if (has && gs.alert) {
    const a = gs.alert, cap = s => s.charAt(0).toUpperCase() + s.slice(1);
    if (a.level) W.alert.set(cap(a.text), t, a.level >= 2 ? 'bad' : 'warn');
    else if (!brt.gndOk) W.alert.set('Not checked: groundAlerts isn\'t running', t, 'warn');   // (its program didn't load: no alerts can come)
    else W.alert.set('All fine', t, 'good');
  }
  renderPeers();
  const LM = radioModel(), roomNote = LM.roomNote(radioCfg);         // what the link's model says of its room and its signal
  if (GS_UI.kindShown !== radioCfg.kind) { GS_UI.kindShown = radioCfg.kind; buildGsRadio(); setText($('#gsLinkWho'), LM.packets ? 'Link quality and RSSI as the command module\'s packet layer reports them (a packet link has no SNR); the rest decoded by it from the frames that came down.' : 'Link quality, RSSI and SNR as the transmitter module reports them to the command module; the rest decoded by it from the frames that came down.'); }   // (another link, set elsewhere: the agent, a loaded design)
  setText($('#gsRoom'), roomNote);
  $('#gsRoom').parentNode.hidden = !roomNote;
  setText($('#gsEquiv'), LM.signalNote(radioCfg));
  const L = gs.link;
  if (has && L) { W.linkUp.set(L.upLq, L.t); W.linkDown.set(L.downLq, L.t); W.rssi.set(L.upRssi, L.t); W.snr.set(LM.packets ? null : L.upSnr, LM.packets ? null : L.t); }
  // throughput over the last second
  gs.rate.push([t, gs.bytes]); while (gs.rate.length > 2 && t - gs.rate[0][0] > 1) gs.rate.shift();
  const dt = gs.rate.length > 1 ? t - gs.rate[0][0] : 0;
  if (has) { W.thru.set(dt > 0 ? (gs.bytes - gs.rate[0][1]) / dt : 0, t); W.budget.set(LM.room(radioCfg), t); }
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
      const b = (text, title, latch, action) => { const x = UI.button( { type: 'button', class: 'btn btn-sm', text, title }); x.addEventListener('click', () => radioCommand(3, [latch, action])); return x; };
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
      [`${S.cmds} in 5 s${S.upQueued ? ` · ${S.upQueued} waiting` : ''}`, ms(S.cmdLat), S.cmdDrop ? `${S.cmdDrop} dropped · ${S.tlmLost != null ? 'too many waiting' : 'link down'}` : S.resent ? `${S.resent} sent again` : ''],
      [`${ps(S.tlmOut)} delivered of ${ps(S.tlmIn)} written${S.tlmSuper ? ` · ${ps(S.tlmSuper)} superseded` : ''}`, ms(S.tlmLat), `${S.downLostPct.toFixed(0)}% of packets${S.tlmLost != null ? ` · ${S.tlmLost} frames lost` : ''} · ${S.tlmDrop} frames dropped · ${S.queued} B queued`]];
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
