'use strict';
// The Ground station tab: what came down the radio (elrs.js decodes it into gs), shown with the data displays of
// gs-widgets.js, and the radio's settings. Nothing here reads the simulator's truth or the boards directly: if the
// link can't carry it, it isn't here.

const SUP_WHY = ['', 'a motor is running hot', 'the battery is hot', 'lift margin low', 'a motor has failed', 'lift margin very low', 'a battery cell has failed',
  'battery below 20%', 'battery voltage low', 'roll and pitch can no longer be held', 'not enough lift to stay up', 'battery nearly empty', 'battery overheating'];
const SUP_MODE = ['normal', 'careful', 'returning home', 'landing'];
const GS_UI = { built: false, w: {}, next: 0, logShow: {}, logN: -1, open: new Set(), paused: false };

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
    sel('gsPower', 'Power', ELRS_POWERS.map(k => [k, k + ' mW']), () => radioCfg.power, v => { radioCfg.power = +v; })));
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
  GS_UI.stats = el('table', { class: 'gs-stats' });
  lg.append(GS_UI.stats, el('p', { class: 'hint', text: '↑ reached the drone\'s receiver, ↓ reached the command module, with how long it took. The sticks and switches are channels, sent in every uplink packet; commands are one-off frames. Click a line for its bytes.' }));
  const kinds = [['stick', 'Sticks and switches'], ['cmd', 'Commands'], ['msg', 'Messages, mode'], ['link', 'Link, drops'], ['all', 'Every frame']];
  const row = el('div', { class: 'gs-filters' });
  for (const [k, label] of kinds) {
    const id = 'gsLog-' + k, c = el('input', { type: 'checkbox', id }); c.checked = k === 'all' ? radioLogAll : GS_UI.logShow[k] !== false;
    c.addEventListener('change', () => { if (k === 'all') radioLogAll = c.checked; else GS_UI.logShow[k] = c.checked; GS_UI.logN = -1; renderGs(true); });
    row.append(el('label', { class: 'check', for: id }, c, label));
  }
  const pz = el('input', { type: 'checkbox', id: 'gsLogPause' }); pz.addEventListener('change', () => { GS_UI.paused = pz.checked; GS_UI.logN = -1; renderGs(true); });
  row.append(el('label', { class: 'check gs-pause', for: 'gsLogPause' }, pz, 'Pause'));
  GS_UI.linkLog = el('ol', { class: 'w-log gs-linklog' }); GS_UI.logN = -1;
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

  // messages
  const l = sec('Messages');
  W.log = GSW.log(40); l.append(W.log.el);
  applySrcTags(pane);
  GS_UI.built = true;
}
function boardsRadioCfg() {   // the radio's settings changed: the boards hear of it at the next reset (as on a real drone, set on both ends)
  for (const b of computers().boards) { const w = brt.inst.get(b.id); if (w) w.tlm_setup(b.tasks.includes('tlm') ? 1 : 0, radioCfg.rate, radioCfg.ratio); }
}

function renderGs(force) {
  if ($('#paneGs').hidden) return;
  const now = performance.now(); if (!force && now < GS_UI.next) return; GS_UI.next = now + 200;
  if (!GS_UI.built) buildGs();
  const W = GS_UI.w, t = brt.t, has = hasTask('tlm');
  const gk = computers().ground, gname = `${gk.name === 'Command module' ? '' : gk.name + ', '}${BOARD_KINDS[gk.kind].label}`;
  $('#gsRadioNote').textContent = has ? `The receiver is on ${boardOf('tlm').name}. Its channels fly the drone${hasTask('nav') ? ' (the sticks move its target; arm, take off, hold and home are switches)' : ' (angle mode)'}; everything below came down the link.`
    : 'No board runs the Telemetry & radio task (Computers tab): the drone has no radio, so nothing comes down and the simulator\'s pilot reaches the boards directly.';
  $('#gsCmdNote').textContent = !has ? '' : brt.gndErr ? brt.gndErr : `Your keys and the simulator's pilot are the buttons of the command module (${gname}): it shapes them (stickInput${gs.shaped === false ? ', not answering: the raw sticks go up' : ''}), sends the channels and commands to the transmitter module, decodes what comes back and warns (groundAlerts).`;
  if (has && gs.alert) W.alert.set(gs.alert.level ? gs.alert.text.charAt(0).toUpperCase() + gs.alert.text.slice(1) : 'All fine', t, gs.alert.level >= 2 ? 'bad' : gs.alert.level === 1 ? 'warn' : 'good');
  const rf = radio.rf;
  const room = radioCfg.rate / radioCfg.ratio * 5;
  $('#gsRoom').textContent = room < 40 ? `At ${radioCfg.rate} Hz with telemetry 1:${radioCfg.ratio}, only ${room < 10 ? room.toFixed(1) : Math.round(room)} bytes a second can come down (one packet in ${radioCfg.ratio}, 5 bytes each); a frame is 10–40 bytes, so values arrive seconds apart. The channels and commands go up in the other packets, so control isn't affected. 1:2 to 1:8 leaves the Ground station enough.` : '';
  $('#gsRoom').parentNode.hidden = room >= 40;
  $('#gsEquiv').textContent = rf ? `Now ${rf.d.toFixed(0)} m from the handset${rf.walls ? `, ${rf.walls} building${rf.walls > 1 ? 's' : ''} in the way` : ''}; with the extra loss that is like ${fmtDist(rf.d * Math.pow(10, radioCfg.extra / 20))} in the open. Signal ${rf.rssi.toFixed(0)} dBm, the receiver needs ${ELRS_RATES[radioCfg.rate]} dBm at ${radioCfg.rate} Hz.` : '';
  const L = gs.link;
  if (has && L) { W.linkUp.set(L.upLq, L.t); W.linkDown.set(L.downLq, L.t); W.rssi.set(L.upRssi, L.t); W.snr.set(L.upSnr, L.t); }
  // throughput over the last second
  gs.rate.push([t, gs.bytes]); while (gs.rate.length > 2 && t - gs.rate[0][0] > 1) gs.rate.shift();
  const dt = gs.rate.length > 1 ? t - gs.rate[0][0] : 0;
  if (has) { W.thru.set(dt > 0 ? (gs.bytes - gs.rate[0][1]) / dt : 0, t); W.budget.set(radioCfg.rate / radioCfg.ratio * 5, t); }
  const last = Math.max(-1e9, ...Object.values(gs.at)); if (has) W.age.set(isFinite(last) ? Math.max(0, t - last) : null, isFinite(last) ? t : null);
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
    if (at.battery !== GS_UI.vAt) { GS_UI.vAt = at.battery; W.vSpark.push(v.battery.volts, at.battery); }
  }
  if (v.pos || v.nav) W.map.set({ track: gs.trackXY || [], pos: v.pos ? [v.pos.x, v.pos.y] : null, target: v.nav ? [v.nav.tx, v.nav.ty] : null, heading: v.attitude ? v.attitude.yaw : 0 }, at.pos ?? at.nav);
  if (v.motors) W.motors.set(v.motors.values, at.motors, actuators().map(c => c.name), v.parts ? v.parts.values : null);
  if (v.super) { W.sup.set(SUP_MODE[v.super.mode] + (v.super.why ? ': ' + (SUP_WHY[v.super.why] || '') : ''), at.super); W.margin.set(v.super.margin, at.super); }
  if (v.learn) {
    const lv = v.learn;
    W.learn.set(lv.cal ? 'calibrating' : lv.throw ? ['', 'in the hand', 'thrown', 'pulsing in free fall', 'catching itself'][lv.throw] || 'throw' : `${lv.learned ? 'on the learned model' : 'on the description'}${lv.keep ? ', learning in flight' : ''}${lv.fitRot ? ` (fit ${Math.round(lv.fitRot * 100)}% rotation, ${Math.round(lv.fitForce * 100)}% force)` : ''}`, at.learn);
    W.cal.set(lv.cal ? lv.progress * 100 : 0, at.learn);
  }
  W.log.set(gs.log);
  if (has) {                                                         // the link's numbers, last 5 s
    const S = linkStats(t), ms = L => L ? `${Math.round(L.avg)} / ${Math.round(L.max)} ms` : '—', ps = x => x < 10 ? x.toFixed(1) : Math.round(x);
    const rows = [['', 'per second', 'latency avg / max', 'lost'],
      ['↑ channels', `${ps(S.chSent)} carried of ${ps(S.chMade)} made`, ms(S.chLat), `${S.upLostPct.toFixed(0)}% of packets`],
      ['↑ commands', `${S.cmds} in 5 s${S.upQueued ? ` · ${S.upQueued} waiting` : ''}`, ms(S.cmdLat), ''],
      ['↓ telemetry', `${ps(S.tlmOut)} delivered of ${ps(S.tlmIn)} written`, ms(S.tlmLat), `${S.downLostPct.toFixed(0)}% of packets · ${S.tlmDrop} frames dropped · ${S.queued} B queued`]];
    const tb = GS_UI.stats; tb.textContent = '';
    rows.forEach((r, i) => tb.append(el('tr', {}, ...r.map(c => el(i ? 'td' : 'th', { text: c })))));
    tb.title = 'Channels: the command module makes a frame every 4 ms; each uplink packet carries the newest, so the rest are superseded, not lost. Latency: from when it was made to when the other end got it. Lost: packets that didn\'t get through (sent again for commands and telemetry). Dropped: frames the drone\'s receiver threw away, its queue full.';
  }
  if (radio.logN !== GS_UI.logN && !GS_UI.paused) {                  // the link log, when something new came
    GS_UI.logN = radio.logN; const show = GS_UI.logShow, box = GS_UI.linkLog; box.textContent = '';
    const group = { stick: 'stick', switch: 'stick', cmd: 'cmd', msg: 'msg', mode: 'msg', link: 'link', drop: 'link' };
    const want = e => e.kind === 'frame' ? radioLogAll : show[group[e.kind]] !== false;
    const TAG = { stick: 'STICK', switch: 'SW', cmd: 'CMD', msg: 'MSG', mode: 'MODE', link: 'LINK', drop: 'DROP', frame: 'TLM' };
    const list = radio.log.filter(want).slice(0, 100);
    for (const e of list) {
      const li = el('li', { 'data-tone': e.tone || '', class: e.bytes ? 'has-raw' + (GS_UI.open.has(e.id) ? ' open' : '') : '' }, el('b', { text: e.t.toFixed(2) }), el('span', { class: 'gs-dir', text: e.dir }),
        el('span', { class: 'gs-kind', text: TAG[e.kind] || e.kind }), el('span', { class: 'gs-data', text: e.data }), el('span', { class: 'gs-meta', text: e.meta }));
      if (e.bytes) {
        li.title = 'Click for the bytes';
        li.addEventListener('click', () => { if (GS_UI.open.has(e.id)) GS_UI.open.delete(e.id); else GS_UI.open.add(e.id); GS_UI.logN = -1; const p = GS_UI.paused; GS_UI.paused = false; renderGs(true); GS_UI.paused = p; });
        if (GS_UI.open.has(e.id)) li.append(rawView(e.bytes));
      }
      box.append(li);
    }
    if (!list.length) box.append(el('li', { class: 'muted', text: has ? 'Nothing yet.' : 'The drone has no radio.' }));
  }
  for (const w of Object.values(W)) w.age(t);
}
// A frame's bytes: hex with offsets, then each field.
function rawView(b) {
  const box = el('div', { class: 'gs-raw' }), hex = [];
  for (let i = 0; i < b.length; i += 16) hex.push(i.toString().padStart(3, ' ') + '  ' + Array.from(b.subarray(i, i + 16), x => x.toString(16).toUpperCase().padStart(2, '0')).join(' '));
  box.append(el('pre', { text: hex.join('\n') }));
  const dl = el('dl'); for (const [k, v] of frameFields(b)) dl.append(el('dt', { text: k }), el('dd', { text: v }));
  box.append(dl);
  box.addEventListener('click', e => e.stopPropagation());
  return box;
}
const fmtDist = m => m >= 1000 ? `${(m / 1000).toFixed(m >= 10000 ? 0 : 1)} km` : `${Math.round(m)} m`;
