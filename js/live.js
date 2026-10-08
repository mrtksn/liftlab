'use strict';
// The real drone, from this page: Bluetooth LE straight from the browser (Web Bluetooth: Chrome or Edge on a computer
// or Android; not Safari, Firefox or iOS). The page becomes the command module: the ground program runs here in an
// instance of its own (the same WebAssembly as the simulator's boards: runner/ground/ground_core.c, the design's
// stickInput and groundAlerts), its packet layer (runner/fc/plink.c) talks to the drone's ESP32-S3 or C3 set to
// radio=ble (runner/esp_radio/radio_ble.c): the packets go to the service's UP characteristic (write without response)
// and come back as notifications of DOWN; INFO says the connection's MTU, which a page can't ask its own Bluetooth.
// While connected the simulation stands still and everything that shows the simulated drone shows the real one: the
// 3D view (its attitude and position from the telemetry), the Ground tab (the command module's view, alerts, link log),
// and the keys, the on-screen pads and a gamepad are its sticks. Arm, take off, hold and home are the bar's switches.
//
// The ground program steps every 4 ms on a timer. A hidden tab's timers slow to once a second: the channels then stop
// and the drone's failsafe takes over (as when a command module's program stalls). Lost, the page reconnects by
// itself; the switches stay as they were (as a handset's).

const LIVE_UUID = n => `6c1f000${n}-6b3d-4e8b-9a7e-6c6966746c62`;   // radio_ble.c SVC, CHR_UP, CHR_DOWN, CHR_INFO
const LIVE_SVC = LIVE_UUID(1), LIVE_UP = LIVE_UUID(2), LIVE_DOWN = LIVE_UUID(3), LIVE_INFO = LIVE_UUID(4);
const live = {
  on: false, state: 'off', dev: null, up: null, g: null, t0: 0, t: 0, lastT: 0, rx: [], writing: false, pend: null,
  timer: 0, mtu: 0, name: '', arm: false, fly: false, nextGnd: 0, nextGs: 0, lastRx: -1, tries: 0, sent: 0, got: 0,
  bad: 0, wErr: 0, wasRunning: true, stopAsk: 0, slow: 0, retry: 0, gp: null,
};
const liveOn = () => live.on;
const liveSupported = () => !!(navigator.bluetooth && window.isSecureContext);
// While connected, the link the rest of the page sees (radioModel): this one.
const LIVE_MODEL = {
  label: 'Bluetooth LE, from this browser', receiver: 'the drone\'s ESP32 (radio=ble)', packets: true, settings: [],
  wasm: () => [5, 0, 0], room: () => 6000, roomNote: () => '',
  signalNote: () => live.state === 'up' ? `Connected to ${live.name} from this browser, packets up to ${live.mtu - 3} bytes (the connection's MTU ${live.mtu}). The browser doesn't say the signal: the drone's own reading is its RSSI here.` : live.state === 'lost' ? `${live.name}: the connection dropped; reconnecting.` : 'Connecting…',
  reset() {}, rebind() {}, step() {}, toStack() {}, fromDrone() {}, command() {},
  connected: () => live.state === 'up' && live.t - live.lastRx < 1,
  stats: () => ({ queued: 0, upQueued: 0 }),
  extraStats: () => ({ resent: 0 }),
};
const liveModel = () => live.on ? LIVE_MODEL : null;

// The ground program in an instance of its own, as boards.js groundStart starts the simulator's.
function liveGround() {
  if (!brt.module) throw new Error('the flight computers are still loading');
  const g = new WebAssembly.Instance(brt.module, { env: RnWasm.env() }).exports;
  let img = null, why = '';
  try { img = boardImage({ name: computers().ground.name, tasks: ['ground'] }); } catch (e) { why = `its program didn't compile (${e.message})`; }
  if (img && img.length > g.img_cap()) { why = 'its program is bigger than its memory'; img = null; }
  if (img) { new Uint8Array(g.memory.buffer, g.img_ptr(), img.length).set(img); if (g.host_setup(img.length)) why = 'its program didn\'t load'; }
  else g.host_setup(0);
  const e = g.gnd_setup(0, 0);
  g.radio_link(5, 0, 0);
  return { g, why: why || (e ? cstr(g, g.gnd_why_ptr(), 96) : '') };
}
function liveMark(g, phrase) {
  const b = new TextEncoder().encode(phrase).slice(0, 63);
  new Uint8Array(g.memory.buffer, g.rbuf_ptr(), b.length).set(b);
  const m = g.ble_mark(b.length) >>> 0;
  return Uint8Array.of(m & 255, (m >>> 8) & 255, (m >>> 16) & 255, m >>> 24);
}

/* ───────── connecting ───────── */
async function liveFind() {
  const msg = $('#liveDlgMsg'), say = (s, tone) => { msg.textContent = s; msg.dataset.tone = tone || ''; };
  if (!liveSupported()) return say('This browser has no Web Bluetooth here.', 'bad');
  if (!hasTask('tlm')) return say('This design has no radio: give a board the Telemetry & radio task (Computers tab).', 'bad');
  let G; try { G = liveGround(); } catch (e) { return say(e.message, 'bad'); }
  const phrase = radioCfg.bind, mark = liveMark(G.g, phrase);
  let dev = null;
  try {
    dev = await navigator.bluetooth.requestDevice({ filters: [{ services: [LIVE_SVC], manufacturerData: [{ companyIdentifier: 0xFFFF, dataPrefix: mark }] }] });
  } catch (e) {
    if (e.name === 'TypeError') {                                    // (a browser without the manufacturer-data filter: every LiftLab drone is listed)
      try { dev = await navigator.bluetooth.requestDevice({ filters: [{ services: [LIVE_SVC] }] }); } catch (e2) { e = e2; }
    }
    if (!dev) return say(e.name === 'NotFoundError' ? 'No drone chosen. Is it powered, set to radio=ble with this binding phrase, and not connected to a command module already?' : `Bluetooth: ${e.message}`, 'bad');
  }
  $('#liveDlg').close();
  liveStart(dev, G);
}
function liveStart(dev, G) {
  if (typeof usbViewOn === 'function' && usbViewOn()) usbViewStop(false);   // (the USB board's view ends: one real drone at a time)
  live.wasRunning = running;
  doReset();                                                         // (the simulated drone back on the ground, its boards started again)
  running = false; renderRun();
  radioReset();
  Object.assign(live, { on: true, state: 'connecting', dev, g: G.g, name: dev.name || 'the drone', t0: performance.now() / 1000 - 1, rx: [], writing: false, pend: null,
    arm: false, fly: false, lastRx: -1, sent: 0, got: 0, bad: 0, wErr: 0, mtu: 0, tries: 0, stopAsk: 0, slow: 0 });
  live.t = live.lastT = 1; live.nextGnd = 1; live.nextGs = 1;
  const ph = new TextEncoder().encode(String(radioCfg.bind)).slice(0, 63);
  new Uint8Array(G.g.memory.buffer, G.g.rbuf_ptr(), ph.length).set(ph);
  const ses = crypto.getRandomValues(new Uint32Array(1))[0] & 0x7FFFFFFF || 1;
  G.g.plink_setup(0, ph.length, ses, 5, 0, 0); G.g.plink_mtu(64);
  brt.gnd = G.g; brt.gndOk = !G.why; brt.gndErr = G.why ? `This page's command module: ${G.why}` : ''; brt.t = 1; radio.t = 1;
  dev.addEventListener('gattserverdisconnected', liveDropped);
  document.querySelector('.view').classList.add('live'); document.body.classList.add('live-on'); $('#livePaused').hidden = false;
  $('#liveBar').hidden = false; liveRenderBar(true); liveRenderBtn();
  linkLog('↕', 'link', `connecting to ${live.name}`, 'Bluetooth LE from this browser', 'warn');
  live.timer = setInterval(liveTick, 4);
  liveGatt();
}
async function liveGatt() {
  const d = live.dev; if (!live.on || !d) return;
  live.state = live.tries ? 'lost' : 'connecting'; live.tries++;
  try {
    const s = await d.gatt.connect(), svc = await s.getPrimaryService(LIVE_SVC);
    const [up, down, info] = await Promise.all([svc.getCharacteristic(LIVE_UP), svc.getCharacteristic(LIVE_DOWN), svc.getCharacteristic(LIVE_INFO)]);
    if (!live.on || live.dev !== d) return;
    down.addEventListener('characteristicvaluechanged', e => { const v = e.target.value; live.rx.push(new Uint8Array(v.buffer, v.byteOffset, v.byteLength).slice()); });
    await down.startNotifications();
    let mtu = 23;                                                    // (the drone asks for 256 when it connects: wait for it a little)
    for (let k = 0; k < 8; k++) { const v = await info.readValue(); mtu = v.getUint16(0, true); if (mtu >= 67) break; await new Promise(r => setTimeout(r, 150)); }
    if (!live.on || live.dev !== d) return;
    live.up = up; live.mtu = mtu; live.g.plink_mtu(Math.min(250, mtu - 3));
    live.state = 'up'; live.tries = 0;
    linkLog('↕', 'link', `connected to ${live.name}`, mtu >= 67 ? `packets up to ${Math.min(250, mtu - 3)} bytes` : `this computer's Bluetooth kept the MTU at ${mtu}: too small for a packet; try another computer or browser`, mtu >= 67 ? 'good' : 'bad');
  } catch (e) {
    if (!live.on || live.dev !== d) return;
    live.state = 'lost';
    if (live.tries === 1 || live.tries % 10 === 0) linkLog('↕', 'link', `couldn't connect to ${live.name}`, `${e.message}; trying again`, 'bad');
    clearTimeout(live.retry); live.retry = setTimeout(liveGatt, 1000);
  }
}
function liveDropped() {
  if (!live.on) return;
  live.up = null; live.state = 'lost';
  linkLog('↕', 'link', `${live.name}: disconnected`, 'reconnecting; the drone\'s failsafe flies it meanwhile', 'bad');
  clearTimeout(live.retry); live.retry = setTimeout(liveGatt, 300);
}
function liveStop() {
  if (!live.on) return;
  live.on = false; live.state = 'off';
  clearInterval(live.timer); clearTimeout(live.retry);
  const d = live.dev; live.dev = null; live.up = null;
  if (d) { d.removeEventListener('gattserverdisconnected', liveDropped); try { if (d.gatt.connected) d.gatt.disconnect(); } catch (e) {} }
  live.g = null;
  document.querySelector('.view').classList.remove('live'); document.body.classList.remove('live-on'); $('#livePaused').hidden = true;
  $('#liveBar').hidden = true;
  doReset();                                                         // (the simulator's command module and link back)
  running = live.wasRunning; renderRun();
  liveRenderBtn();
}

/* ───────── each 4 ms: the command module ───────── */
function liveSend(b) {
  if (!live.up) return;
  if (live.writing) { live.pend = b; return; }                       // (one write at a time: the newest waits; plink sends again what counts)
  live.writing = true; live.sent++;
  live.up.writeValueWithoutResponse(b).catch(() => { live.wErr++; }).finally(() => {
    live.writing = false; if (live.pend) { const p = live.pend; live.pend = null; liveSend(p); }
  });
}
function liveInputs(t) {
  const bit = k => 1 << GB[k];
  let held = 0, has = 0, ax = [0, 0, 0, 0];
  if (hasTask('nav')) { for (const k of ['right', 'left', 'fwd', 'back', 'up', 'down', 'yawR', 'yawL']) if (isHeld(k)) held |= bit(k); }
  const p = livePad(); if (p) { has = 15; ax = p; }
  if (live.arm) held |= bit('arm'); if (live.fly) held |= bit('fly');
  if (t < radio.holdUntil) held |= bit('hold'); if (t < radio.homeUntil) held |= bit('home');
  held |= bit(pilot.level === 'gentle' ? 'gentle' : pilot.level === 'sport' ? 'sport' : 'normal');
  return { held, has, ax };
}
// A gamepad's sticks, Mode 2 (the left stick throttle and yaw, the right stick roll and pitch): [roll, pitch, throttle,
// yaw], −1…1, or null with none.
function livePad() {
  const pads = navigator.getGamepads ? Array.from(navigator.getGamepads()).filter(Boolean) : [];
  const gp = pads.find(p => p.connected && p.axes.length >= 4); live.gp = gp ? gp.id : null;
  if (!gp) return null;
  const a = i => { const v = gp.axes[i] || 0; return Math.abs(v) < 0.05 ? 0 : v; };
  return [a(2), -a(3), -a(1), a(0)];
}
function liveTick() {
  if (!live.on) return;
  const g = live.g, t = performance.now() / 1000 - live.t0;
  if (t - live.lastT > 0.25 && live.state === 'up') { live.slow++; linkLog('↕', 'link', `this page stood still for ${(t - live.lastT).toFixed(1)} s`, 'a hidden tab or a busy computer: nothing went up meanwhile, and the drone may have failed safe', 'bad'); live.nextGnd = t; }
  // what came
  while (live.rx.length) {
    const p = live.rx.shift(); if (p.length > 250) continue;
    new Uint8Array(g.memory.buffer, g.pbuf_ptr(), p.length).set(p);
    if (g.plink_air_in(p.length, 0, t)) { live.got++; live.lastRx = t; linkEv('downOk', t); } else { live.bad++; linkEv('downLost', t); }
  }
  g.host_tick(Math.min(0.1, t - live.lastT));
  const n = g.plink_stack_out(t);
  if (n) {
    const b = Uint8Array.from(new Uint8Array(g.memory.buffer, g.rbuf_ptr(), n));
    g.gnd_from_radio(n, t);
    radio.stackIn.gnd.feed(b, f => liveFrame(f, t));
  }
  // the ground program's steps
  for (let k = 0; live.nextGnd <= t && k < 8; k++) {
    live.nextGnd += 0.004;
    const I = liveInputs(t), m = g.gnd_tick(I.held, I.has, I.ax[0], I.ax[1], I.ax[2], I.ax[3], t, 0.004);
    if (m) { const b = Uint8Array.from(new Uint8Array(g.memory.buffer, g.rbuf_ptr(), m)); g.plink_stack_in(m, t); radioFromGround(b); }
  }
  if (live.nextGnd < t - 0.05) live.nextGnd = t;
  // the packets due
  for (let k = 0; k < 4; k++) { const m = g.plink_air_out(t); if (!m) break; if (live.state === 'up') liveSend(Uint8Array.from(new Uint8Array(g.memory.buffer, g.pbuf_ptr(), m))); }
  live.lastT = live.t = brt.t = radio.t = t;
  if (t >= live.nextGs) { live.nextGs = t + 0.1; gsRead(); }
}
// A frame the packet layer gave the command module: the messages and the flight mode go in the link log.
function liveFrame(f, t) {
  if (f[2] === CRSF.LINK_STATS) return;
  linkEv('tlmOut', t, 0);
  const k = frameKind(f);
  if (k === 'msg') linkLog('↓', 'msg', frameDesc(f), '', '', f);
  else if (k === 'mode') { const d = frameDesc(f); if (d !== radio.modeSeen) { radio.modeSeen = d; linkLog('↓', 'mode', d, '', '', f); } }
  else if (radioLogAll) linkLog('↓', 'frame', frameDesc(f), '', '', f);
}

/* ───────── each frame: the 3D view and the bar, from the telemetry ───────── */
// The attitude as a quaternion (w, x, y, z), from the angles as the drone sends them (tlm_crsf.c: roll and pitch as
// the flight core's, the heading its yaw).
function liveQuat(a) {
  const h = v => [Math.cos(v / 2), Math.sin(v / 2)], [cr, sr] = h(a.roll), [cp, sp] = h(a.pitch), [cy, sy] = h(a.yaw);
  return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy];
}
function liveView() {
  if (!live.on) return;
  const v = gs.v, home = spawnAt || [0, 0, 0];
  if (v.attitude) S.q = qnorm(liveQuat(v.attitude));
  if (v.pos) S.p = [home[0] + v.pos.x, home[1] + v.pos.y, home[2] + Math.max(0, v.pos.z)];
  else if (v.baro) S.p = [home[0], home[1], home[2] + Math.max(0, v.baro.alt)];
  S.v = v.pos ? [v.pos.vx, v.pos.vy, v.pos.vz] : [0, 0, v.baro ? v.baro.vz || 0 : 0];
  S.t = live.t;
  liveRenderBar();
}
function liveRenderBar(force) {
  if (!live.on) return;
  const now = performance.now(); if (!force && now < (live.barNext || 0)) return; live.barNext = now + 200;
  const v = gs.v, L = gs.link, a = gs.alert, conn = LIVE_MODEL.connected();
  setText($('#liveName'), live.name);
  const st = $('#liveState');
  setText(st, live.state === 'up' ? (conn ? 'connected' : 'connected, nothing coming') : live.state === 'lost' ? 'reconnecting…' : 'connecting…');
  st.dataset.tone = live.state === 'up' && conn ? 'good' : live.state === 'up' ? 'warn' : 'bad';
  const bits = [];
  if (L) bits.push(`LQ ↑${Math.round(L.upLq)}% ↓${Math.round(L.downLq)}%`);
  if (v.battery) bits.push(`${v.battery.volts.toFixed(1)} V${v.battery.pct ? ` ${Math.round(v.battery.pct)}%` : ''}`);
  if (v.pos) bits.push(`height ${v.pos.z.toFixed(1)} m`); else if (v.baro) bits.push(`height ${v.baro.alt.toFixed(1)} m`);
  if (v.mode) bits.push(v.mode.mode);
  setText($('#liveRead'), bits.join(' · ') || 'no telemetry yet');
  const al = $('#liveAlert'), show = a && a.level > 0 && live.state !== 'connecting';
  al.hidden = !show; if (show) { setText(al, a.text.charAt(0).toUpperCase() + a.text.slice(1)); al.dataset.tone = a.level >= 2 ? 'bad' : 'warn'; }
  const nav = hasTask('nav'), pad = !!live.gp;
  $('#liveFly').hidden = !nav;
  $('#liveHome').hidden = !nav;
  $('#liveArm').setAttribute('aria-pressed', String(live.arm)); $('#liveArm').textContent = live.arm ? 'Armed' : 'Arm';
  $('#liveFly').setAttribute('aria-pressed', String(live.fly)); $('#liveFly').textContent = live.fly ? 'Land' : 'Take off';
  const stop = $('#liveStop'); if (live.stopAsk && now > live.stopAsk) live.stopAsk = 0;
  stop.textContent = live.stopAsk ? 'Really disconnect?' : 'Disconnect';
  setText($('#liveNote'), !nav && !pad ? 'Angle mode: plug in a gamepad for the sticks (the keys move only a navigated drone).'
    : `${pad ? `Gamepad: ${live.gp.replace(/\s*\(.*$/, '')}. ` : ''}${nav ? 'The keys and pads move it; Space holds, H flies home.' : ''}${live.slow ? ` This page stood still ${live.slow}×: keep this tab in front.` : ''}`);
}
function liveRenderBtn() {
  const b = $('#liveBtn'); if (!b) return;
  b.classList.toggle('on', live.on); b.setAttribute('aria-pressed', String(live.on));
  setText(b.querySelector('.live-lbl'), live.on ? 'Live' : 'Connect to drone');
  b.title = live.on ? 'Flying the real drone over Bluetooth: the bar over the view disconnects' : 'Fly the real drone from this page over Bluetooth LE (its ESP32-S3 or C3 set to radio=ble)';
}
function liveArmToggle() {
  if (!live.arm) {
    if (!hasTask('nav') && !live.gp) { linkLog('↕', 'link', 'not armed', 'angle mode needs a gamepad for the throttle', 'bad'); return; }
    const p = livePad(); if (!hasTask('nav') && p && p[2] > -0.9) { linkLog('↕', 'link', 'not armed', 'throttle stick down first', 'bad'); flashEl($('#liveNote')); return; }
  }
  live.arm = !live.arm; if (!live.arm) live.fly = false;
  linkLog('↑', 'switch', `ARM ${live.arm ? 'on' : 'off'}`, 'this page\'s switch', live.arm ? 'warn' : '');
  liveRenderBar(true);
}
function flashEl(e) { if (!e) return; e.classList.remove('flash'); void e.offsetWidth; e.classList.add('flash'); }

/* ───────── the page ───────── */
function liveOpen() {
  if (live.on) { $('#liveStop').focus(); return; }
  const ok = liveSupported(), dlg = $('#liveDlg');
  setText($('#livePhrase'), radioCfg.bind);
  $('#liveNoBt').hidden = ok; $('#liveFind').disabled = !ok;
  setText($('#liveDlgMsg'), ''); setText($('#liveMode'), hasTask('nav') ? 'This design navigates: the keys, the pads or a gamepad move it, the switches arm it and take it off.' : 'This design flies in angle mode (no navigation): plug in a gamepad for the sticks before you arm.');
  dlg.showModal();
}
(function liveBind() {
  const b = $('#liveBtn'); if (!b) return;
  b.addEventListener('click', liveOpen);
  $('#liveFind').addEventListener('click', liveFind);
  $('#liveClose').addEventListener('click', () => $('#liveDlg').close());
  $('#liveArm').addEventListener('click', liveArmToggle);
  $('#liveFly').addEventListener('click', () => { if (!live.arm && !live.fly) { linkLog('↕', 'link', 'not taking off', 'arm first', 'bad'); return; } live.fly = !live.fly; linkLog('↑', 'switch', `FLY ${live.fly ? 'on' : 'off'}`, 'this page\'s switch', ''); liveRenderBar(true); });
  $('#liveHold').addEventListener('click', () => radioHold());
  $('#liveHome').addEventListener('click', () => radioHome());
  $('#liveStop').addEventListener('click', () => {
    const flying = gs.v.state && gs.v.state.state >= 1 && LIVE_MODEL.connected();
    if (flying && !live.stopAsk) { live.stopAsk = performance.now() + 4000; linkLog('↕', 'link', 'the drone is armed', 'disconnecting leaves it to its failsafe (it lands): press again to disconnect', 'warn'); liveRenderBar(true); return; }
    liveStop();
  });
  document.querySelectorAll('#liveBar button').forEach(x => x.addEventListener('mousedown', e => e.preventDefault()));   // (a click leaves the keyboard where it was: Space mustn't press Arm again)
  liveRenderBtn();
})();
