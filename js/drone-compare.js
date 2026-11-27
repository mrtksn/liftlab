'use strict';
// Compare with the drone: what the connected drone has, beside the design. Through the drone's Pi, down the radio
// (runner/fc/file_rx.h, files down; dfb_pi.c readout and fget; the simulator's Pi answers the same: boards.js piText):
//   readout        the Pi asks the ESP32 for its settings (show) and keeps its whole answer as esp32.txt
//   fget NAME      a file's size and CRC-32; fget NAME OFF N: its pieces, "fg:OFF:BASE64" a message (link.js puts
//                  them in gs.fg, not the log), asked for again until all have come
// Then, beside what the design would install (its wiring lines: hardwareSettings, and its files: cardUploadFiles):
//   - the ESP32's settings, a row each: the same, different, only on the drone, or only in the design. A setting the
//     design has a place for (pins, ESC pulses, level, the IMU's calibration, the sensors' drivers…) can be taken
//     into the design (Take: one undo step, as an edit of the wiring); the radio's and the network's are compared only;
//   - the Pi's files (wiring.txt, drone.dfa, drone.dnc, drone.dlc): the same or not, by the CRC-32 each ends with
//     (the .dfa, .dnc, .dlc) or of the whole (wiring.txt). A different airframe is read: its mass, inertia, IMU mount
//     and each motor's effect against the design's. It isn't taken: an airframe holds totals, not the parts that
//     make them, so the design is what to change (or Upload design, to give the drone the design's).
// Disarmed or not, it only reads: nothing on the drone changes but esp32.txt in the Pi's folder.

const DC = { on: false, stop: false, said: '', res: null, err: '', at: 0 };
const DC_FILES = ['wiring.txt', 'drone.dfa', 'drone.dnc', 'drone.dlc'];
const DC_PIECE = 33;                                                      // (file_rx.h FILE_TX_PIECE)
const DC_SECRET = ['bind', 'fleet', 'wifi'];                              // (show masks them)
const DC_INFO = ['radio', 'radio2', 'peers', 'rate', 'telemetry', 'baud'];   // compared only as text: the radio's own words
const DC_MINUS1 = ['battery', 'battery2', 'crsf', 'link_uart', 'imu_power', 'tof', 'nrf24', 'pmon'];   // (−1 and anything after it: not used)
const DC_LABEL = { motors: 'Motor pins', motor_driver: 'Motor drivers (0 ESC, 1 brushed)', motor_max: 'Motor duty limits', brushed_hz: 'Brushed PWM', servos: 'Servo pins',
  esc_hz: 'ESC signal', esc_us: 'ESC pulses (start, full, stop)', i2c: 'I²C pins (SDA, SCL)', battery: 'Battery ADC (pin, divider)', battery2: 'Second pack ADC', vref: 'Airframe voltage',
  link_uart: 'Link to the Pi (TX, RX)', crsf: 'Receiver (RX, TX)', level: 'Level', level_yaw: 'Level yaw', imu_power: 'IMU power switch', acc_bias: 'Accelerometer offsets',
  acc_scale: 'Accelerometer gains', gyro_bias: 'Gyro offsets', imu: 'IMU (driver, address)', baro: 'Barometer (driver, address)', mag: 'Compass (driver, address)', tof: 'Rangefinder address',
  pmon: 'Power monitor', servo_center: 'Servo centres', servo_us_per_rad: 'Servo scales', nrf24: 'nRF24 pins', link: 'Link', link_topic: 'Link topics', radio: 'Radio', radio2: 'Second radio',
  rate: 'Loop rate', telemetry: 'Telemetry rate', baud: 'Link baud', peers: 'Peers', mag_matrix: 'Compass matrix', mag_bias: 'Compass offsets', mag_scale: 'Compass gains' };

/* ───────── the settings, the design's and the drone's ───────── */
// What the design would put on its flight controller: KEY → VALUE, the last line of each (the lines clear some first).
function dcDesignSettings() {
  const core = boardOf('core'), S = new Map();
  if (!core || !BOARD_KINDS[core.kind] || !BOARD_KINDS[core.kind].mcu) return S;
  for (const l of hardwareSettings(boardWiringPlan(core))) { const k = l.indexOf('='); if (k > 0) S.set(l.slice(0, k), l.slice(k + 1)); }
  return S;
}
// The ESP32's answer to show (flight.c: hw_describe's KEY=VALUE line, its sensors' line, what it found): { now, next
// (what it takes after a reboot, if that differs), info: its other lines }.
function dcParseShow(text) {
  const now = new Map(), info = []; let next = null;
  const tokens = (s, M) => { for (const m of s.matchAll(/([a-z][a-z0-9_]*)=(\([^)]*\)|[^\s(]*(?:\([^)]*\))?)/g)) M.set(m[1], m[2]); };
  for (const raw of String(text).split('\n')) {
    const l = raw.trim(); if (!l) continue;
    if (l.startsWith('after a reboot: ')) { next = new Map(); tokens(l.slice(16), next); }
    else if (l.startsWith('sensor profiles: ')) tokens(l.slice(17), now);
    else if (/^[a-z][a-z0-9_]*=/.test(l)) tokens(l, now);               // (hw_describe's line: KEY=VALUE, a space between)
    else info.push(l);
  }
  return { now, next, info };
}
// A value to compare: { nums } or { text }, or { secret }.
function dcNorm(key, v) {
  v = String(v ?? '').trim();
  if (DC_SECRET.includes(key) || /^\(/.test(v)) return { secret: true, text: v };
  const bare = v.replace(/\(.*\)$/, ''), parts = bare === '' ? [] : bare.split(',');
  if (DC_INFO.includes(key) || !parts.every(p => p.trim() !== '' && Number.isFinite(Number(p)))) return { text: bare.toLowerCase() };
  let nums = parts.map(Number);
  if (DC_MINUS1.includes(key) && nums[0] === -1) nums = [-1];
  if (key === 'imu_power' && nums.length === 1 && nums[0] >= 0) nums.push(1);
  if (key === 'esc_us' && nums.length === 3 && nums[2] === 0) nums.pop();
  return { nums };
}
function dcTol(key, y) {
  if (key === 'level') return 0.051;
  if (key === 'level_yaw' || key === 'servo_us_per_rad') return 0.51;
  if (key === 'battery' || key === 'battery2' || key === 'vref') return 0.051;
  return 1e-3 + 1e-3 * Math.abs(y);
}
function dcSame(key, a, b) {
  if (a.secret || b.secret) return null;                                // (can't say)
  if (a.nums && b.nums) return a.nums.length === b.nums.length && a.nums.every((x, i) => Math.abs(x - b.nums[i]) <= dcTol(key, b.nums[i]));
  return (a.text ?? (a.nums || []).join(',')) === (b.text ?? (b.nums || []).join(','));
}
// The rows: { key, label, design, drone, state: same | differ | drone | design | secret, take }.
function dcSettingRows(design, drone) {
  const keys = [...new Set([...design.keys(), ...drone.keys()])], rows = [];
  for (const key of keys) {
    const d = design.get(key), e = drone.get(key), row = { key, label: DC_LABEL[key] || key, design: d, drone: e };
    if (d == null) row.state = 'drone';
    else if (e == null) row.state = 'design';
    else { const s = dcSame(key, dcNorm(key, d), dcNorm(key, e)); row.state = s == null ? 'secret' : s ? 'same' : 'differ'; }
    row.take = row.state === 'differ' && !!DC_TAKE[key];
    rows.push(row);
  }
  const order = { differ: 0, design: 1, drone: 2, secret: 3, same: 4 };
  return rows.sort((a, b) => order[a.state] - order[b.state]);
}

/* ───────── taking the drone's into the design ───────── */
// Each: (wiring, the drone's numbers, its text, ctx) → '' or why it can't. ctx: { core, motors, joints, sensor(kind), drone }.
const dcByKindOn = (kind, core) => sensorsOf(kind).find(c => { const o = wiredTo(c); return o && o.id === core.id; });
const dcPins = (list, what, prop) => (w, n) => {
  const comps = list(); if (n.length !== comps.length) return `the drone has ${n.length} ${what}, the design ${comps.length}`;
  comps.forEach((c, i) => { (w.parts[c.id] ||= {})[prop] = n[i]; }); return '';
};
const DC_TAKE = {
  esc_hz: (w, n, t, x) => { x.B(w).escHz = n[0]; return ''; },
  esc_us: (w, n, t, x) => { const B = x.B(w); B.escMin = n[0]; B.escMax = n[1]; B.escStop = n[2] || 0; return ''; },
  brushed_hz: (w, n, t, x) => { x.B(w).brushedHz = n[0]; return ''; },
  i2c: (w, n, t, x) => { const B = x.B(w); B.sda = n[0]; B.scl = n[1]; return ''; },
  battery: (w, n, t, x) => { const B = x.B(w); B.batteryPin = n[0]; if (n[0] >= 0) B.batteryDivider = n[1]; return ''; },
  battery2: (w, n, t, x) => { const B = x.B(w); B.batteryPin2 = n[0]; if (n[0] >= 0) B.batteryDivider2 = n[1]; return ''; },
  crsf: (w, n, t, x) => { const B = x.B(w); B.crsfRx = n[0]; B.crsfTx = n[0] >= 0 ? n[1] : -1; return ''; },
  link_uart: (w, n, t, x) => { const B = x.B(w); B.fcLinkTx = n[0]; B.fcLinkRx = n[0] >= 0 ? n[1] : -1; return ''; },
  motors: dcPins(() => actuators(), 'motors', 'pin'),
  servos: dcPins(() => joints(), 'servos', 'pin'),
  motor_driver: (w, n) => dcPins(() => actuators(), 'motors', 'driver')(w, n.map(v => v === 1 ? 'brushed' : 'pwm')),
  motor_max: dcPins(() => actuators(), 'motors', 'maxDuty'),
  servo_center: dcPins(() => joints(), 'servos', 'center'),
  servo_us_per_rad: dcPins(() => joints(), 'servos', 'usPerRad'),
  level: (w, n, t, x) => { const c = x.sensor('imu'); if (!c) return 'no IMU on the flight controller in the design'; (w.parts[c.id] ||= {}).level = n ? n.join(',') : t; return ''; },
  level_yaw: (w, n, t, x) => { const c = x.sensor('imu'); if (!c) return 'no IMU on the flight controller in the design'; (w.parts[c.id] ||= {}).levelYaw = n[0]; return ''; },
  imu_power: (w, n, t, x) => { const c = x.sensor('imu'); if (!c) return 'no IMU on the flight controller in the design'; const p = (w.parts[c.id] ||= {}); p.powerPin = n[0]; if (n[0] >= 0) p.powerLevel = n[1] ?? 1; return ''; },
  // the calibration: all three at once (sensorCalOk takes them together), as the drone has them
  ...Object.fromEntries(['acc_bias', 'acc_scale', 'gyro_bias'].map(k => [k, (w, n, t, x) => {
    const c = x.sensor('imu'); if (!c) return 'no IMU on the flight controller in the design';
    const g = key => { const v = dcNorm(key, x.drone.get(key)); return v.nums && v.nums.length === 3 ? v.nums : null; };
    const cal = { accBias: g('acc_bias'), accScale: g('acc_scale'), gyroBias: g('gyro_bias'), fitted: true, at: Date.now(), from: 'the drone' };
    if (!cal.accBias || !cal.accScale || !cal.gyroBias || !sensorCalOk(cal)) return 'the drone\'s calibration isn\'t a whole one LiftLab takes';
    (w.parts[c.id] ||= {}).cal = cal; return '';
  }])),
  ...Object.fromEntries(['imu', 'baro', 'mag'].map(k => [k, (w, n, t, x) => {
    const c = x.sensor(k); if (!c) return `no ${SENSOR_KINDS[k].toLowerCase()} on the flight controller in the design`;
    const drv = Object.entries(DEVICE_PROFILES[k]).find(([, d]) => d.id === n[0]); if (!drv) return `driver ${n[0]} isn't one LiftLab knows for it`;
    const p = (w.parts[c.id] ||= {}); p.driver = drv[0]; p.address = n[1]; return '';
  }])),
  tof: (w, n, t, x) => { const c = x.sensor('tof'); if (!c) return 'no rangefinder on the flight controller in the design'; if (n[0] < 0) return 'the drone has none: remove it from the design instead'; (w.parts[c.id] ||= {}).address = n[0]; return ''; },
};
// Take these rows' drone values into the design (one undo step). '' or why not.
function dcTake(rows) {
  const core = boardOf('core'); if (!core) return 'the design has no flight controller';
  const drone = DC.res && DC.res.show ? DC.res.show.now : new Map(), why = [];
  editWiring(w => {
    w.parts ||= {}; w.boards ||= {};
    const x = { B: W => (W.boards[core.id] ||= {}), sensor: k => dcByKindOn(k, core), drone };
    for (const r of rows) {
      const v = dcNorm(r.key, r.drone), f = DC_TAKE[r.key];
      const e = f ? f(w, v.nums || null, v.text, x) : 'not a setting LiftLab keeps';
      if (e) why.push(`${r.label}: ${e}`);
    }
  }, 'from the drone');
  if (DC.res) DC.res.settings = dcSettingRows(dcDesignSettings(), drone);
  return why.join('; ');
}

/* ───────── the airframe, read ───────── */
function dcAirframe(b) {
  const v = new DataView(b.buffer, b.byteOffset, b.byteLength); let o = 0;
  const u = () => { const x = v.getUint32(o, true); o += 4; return x; }, f = () => { const x = v.getFloat32(o, true); o += 4; return x; }, fs = n => Array.from({ length: n }, f);
  const name = () => { const s = new TextDecoder().decode(b.subarray(o, o + 16)).replace(/\0.*$/s, ''); o += 16; return s; };
  if (b.length < 24 || u() !== 0x41424644) return null;
  const ver = u(), nm = u(), nj = u(), mode = u(), A = { ver, mode, m: f(), J: fs(9) }; fs(9); fs(3); A.imuR = fs(9); A.imuPos = ver >= 2 ? fs(3) : [0, 0, 0];
  fs(6); A.joints = []; A.motors = [];
  for (let j = 0; j < nj; j++) { A.joints.push(name()); u(); fs(4); }
  for (let i = 0; i < nm; i++) { const n = name(); u(); u(); u(); const nb = u(), cols = []; for (let k = 0; k < nb; k++) cols.push(fs(6)); fs(3); A.motors.push({ name: n, cols }); }
  return A;
}
// What differs between two airframes, a line each.
function dcAirframeDiff(D, E) {
  if (!D || !E) return ['not an airframe file'];
  const out = [], pct = (a, b) => Math.abs(b) > 1e-9 ? Math.abs(a - b) / Math.abs(b) : Math.abs(a - b);
  if (pct(E.m, D.m) > 0.005) out.push(`mass: the drone's ${(E.m * 1000).toFixed(0)} g, the design's ${(D.m * 1000).toFixed(0)} g`);
  const Jd = [D.J[0], D.J[4], D.J[8]], Je = [E.J[0], E.J[4], E.J[8]];
  if (Jd.some((x, i) => pct(Je[i], x) > 0.02)) out.push(`inertia (roll, pitch, yaw): the drone's ${Je.map(x => (x * 1e4).toFixed(1)).join(', ')}, the design's ${Jd.map(x => (x * 1e4).toFixed(1)).join(', ')} ×10⁻⁴ kg·m²`);
  { const R = D.imuR, Q = E.imuR, tr = R[0] * Q[0] + R[1] * Q[1] + R[2] * Q[2] + R[3] * Q[3] + R[4] * Q[4] + R[5] * Q[5] + R[6] * Q[6] + R[7] * Q[7] + R[8] * Q[8];
    const ang = Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2))) * 180 / Math.PI; if (ang > 0.5) out.push(`IMU mount: turned ${ang.toFixed(1)}° from the design's`); }
  if (Math.hypot(...E.imuPos.map((x, i) => x - D.imuPos[i])) > 0.003) out.push(`IMU position: ${(Math.hypot(...E.imuPos.map((x, i) => x - D.imuPos[i])) * 1000).toFixed(0)} mm from the design's`);
  if (E.motors.length !== D.motors.length) out.push(`motors: the drone's ${E.motors.length}, the design's ${D.motors.length}`);
  else D.motors.forEach((m, i) => {
    const e = E.motors[i], a = m.cols[0], c = e.cols[0], mag = Math.max(...a.map(Math.abs)) || 1, d = Math.max(...a.map((x, k) => Math.abs(x - c[k]))) / mag;
    if (e.name !== m.name) out.push(`motor ${i + 1}: the drone's "${e.name}", the design's "${m.name}"`);
    if (d > 0.02) out.push(`${m.name}: its effect differs by up to ${(d * 100).toFixed(0)}% (thrust, position or spin)`);
  });
  if (E.joints.length !== D.joints.length) out.push(`servos: the drone's ${E.joints.length}, the design's ${D.joints.length}`);
  return out;
}

/* ───────── reading it from the drone ───────── */
const dcTick = () => new Promise(r => setTimeout(r, 40));
function dcSay(s) { DC.said = s; dcRender(); }
async function dcSend(line) {
  const t0 = cardClock();
  for (;;) {
    if (DC.stop) throw new Error('stopped');
    if (cardLive() && live.state !== 'up') throw new Error('the link to the drone is gone');
    const why = cardTextOut(line); if (!why) return;
    if (!/too many commands/.test(why) || cardClock() - t0 > 15) throw new Error(why);
    await dcTick();
  }
}
async function dcWait(n0, re, s) {
  const t0 = cardClock();
  for (;;) {
    if ((gs.nmsg || 0) < n0) n0 = 0;                                     // (the command module started again: its count did too)
    const k = Math.min(gs.log.length, (gs.nmsg || 0) - n0), got = gs.log.slice(0, Math.max(0, k)).reverse().find(m => re.test(m.text));
    if (got) return got.text;
    if (cardClock() - t0 > s) return '';
    if (DC.stop) throw new Error('stopped');
    await dcTick();
  }
}
const dcEsc = s => s.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
// A file's size and CRC-32 on the drone, or null if it hasn't it.
async function dcHead(name) {
  const n0 = gs.nmsg || 0; await dcSend(`fget ${name}`);
  const r = await dcWait(n0, new RegExp(`^fget ${dcEsc(name)}: |^fget NAME`), 8);
  if (!r) throw new Error(`no answer from the Pi about ${name}: is its dfb_pi new enough (it sends files since this LiftLab)?`);
  const m = /: (\d+) ([0-9a-f]{8})$/.exec(r); if (!m) { if (/no such file/.test(r)) return null; throw new Error(r); }
  return { size: +m[1], crc: parseInt(m[2], 16) };
}
// Bytes [from, to) of a file on the drone, asked for three pieces at a time, again for those that don't come.
async function dcRange(name, size, from, to, what) {
  const buf = new Uint8Array(size), want = [];
  for (let o = Math.floor(from / DC_PIECE) * DC_PIECE; o < Math.min(to, size); o += DC_PIECE) want.push(o);
  const got = new Set();
  for (let round = 0; round < 8 && got.size < want.length; round++) {
    const todo = want.filter(o => !got.has(o));
    for (let i = 0; i < todo.length; i += 3) {
      const batch = todo.slice(i, i + 3); if (what) dcSay(`${what}: ${got.size} of ${want.length} pieces`);
      const f0 = (gs.fg ||= []).length; await dcSend(`fget ${name} ${batch[0]} ${batch.length}`);
      const t0 = cardClock();
      while (cardClock() - t0 < 4 && !batch.every(o => got.has(o))) {
        if (DC.stop) throw new Error('stopped');
        for (const t of gs.fg.splice(Math.min(f0, gs.fg.length))) {
          const m = /^fg:(\d+):([A-Za-z0-9+/=]+)$/.exec(t); if (!m) continue;
          const o = +m[1]; if (!want.includes(o)) continue;
          const d = Uint8Array.from(atob(m[2]), c => c.charCodeAt(0)); buf.set(d.subarray(0, Math.max(0, size - o)), o); got.add(o);
        }
        await dcTick();
      }
      if (batch.length === 3 && batch.every(o => !got.has(o)) && round > 0) await dcTick();
    }
  }
  if (got.size < want.length) throw new Error(`${name}: pieces kept going missing; is the link weak?`);
  return buf;
}
async function dcFile(name, head, what) {
  const b = await dcRange(name, head.size, 0, head.size, what);
  if (crc32(b) !== head.crc) throw new Error(`${name} came down damaged; try again`);
  return b;
}
const dcTail = b => b.length >= 4 ? new DataView(b.buffer, b.byteOffset + b.length - 4, 4).getUint32(0, true) : null;
async function dcRun() {
  const res = { at: Date.now(), live: cardLive(), files: [], show: null, settings: [], next: [] };
  // the ESP32's settings
  dcSay('Asking the ESP32 for its settings');
  let n0 = gs.nmsg || 0; await dcSend('readout');
  const r = await dcWait(n0, /^readout: (esp32\.txt|no answer|couldn't)/, 10);
  if (!r) throw new Error('no answer to readout from the Pi: is its dfb_pi new enough (readout since this LiftLab)?');
  if (!/esp32\.txt/.test(r)) throw new Error(r.replace(/^readout: /, ''));
  const h = await dcHead('esp32.txt'); if (!h) throw new Error('the Pi has no esp32.txt after all');
  const text = new TextDecoder().decode(await dcFile('esp32.txt', h, 'The ESP32\'s settings'));
  res.show = dcParseShow(text); res.showText = text;
  const design = dcDesignSettings();
  res.settings = dcSettingRows(design, res.show.now);
  if (res.show.next) res.next = dcSettingRows(res.show.now, res.show.next).filter(x => x.state === 'differ');
  // the Pi's files
  let mine = []; try { mine = cardUploadFiles(); } catch (e) { res.filesWhy = e.message; }
  for (const name of DC_FILES) {
    const d = mine.find(f => f.name === name), row = { name, design: d ? d.data.length : null };
    dcSay(`The Pi's ${name}`);
    const hd = await dcHead(name); row.drone = hd ? hd.size : null;
    if (!hd) row.state = d ? 'missing' : 'none';
    else if (!d) row.state = 'drone';
    else if (name === 'wiring.txt') row.state = hd.size === d.data.length && hd.crc === crc32(d.data) ? 'same' : 'differ';
    else {
      // these end in their own CRC-32: the last 4 bytes say what's in them, without bringing the whole file
      const tail = hd.size >= 4 ? (await dcRange(name, hd.size, hd.size - 4, hd.size)).subarray(hd.size - 4) : null;
      const theirs = tail ? new DataView(tail.buffer, tail.byteOffset, 4).getUint32(0, true) : null;
      row.state = hd.size === d.data.length && theirs === dcTail(d.data) ? 'same' : 'differ';
      if (row.state === 'differ' && name === 'drone.dfa') row.diff = dcAirframeDiff(dcAirframe(d.data), dcAirframe(await dcFile(name, hd, 'The Pi\'s airframe')));
    }
    res.files.push(row);
  }
  return res;
}
function dcStart() {
  if (!cardRadioPi()) return 'Compare with the drone goes through the drone\'s Pi, and this design\'s radio isn\'t on one';
  if (cardLive() ? live.state !== 'up' : !brt.ready) return cardLive() ? 'not connected to the drone yet' : 'the simulator isn\'t running yet';
  if (DC.on || (typeof cardUp !== 'undefined' && cardUp.on)) return 'busy: wait for it to finish';
  Object.assign(DC, { on: true, stop: false, err: '', said: 'Starting' });
  linkLog('↑', 'cmd', 'compare with the drone', 'readout, fget', '');
  dcRun().then(res => { DC.res = res; DC.err = ''; }, e => { DC.err = e.message === 'stopped' ? 'Stopped.' : e.message; })
    .finally(() => { DC.on = false; DC.said = ''; linkLog('↓', 'cmd', 'compare with the drone', DC.err || 'done', DC.err ? 'bad' : ''); dcRender(); });
  return '';
}

/* ───────── the dialog ───────── */
function dcOpen() { const d = $('#compareDlg'); if (!d) return; if (!d.open) d.showModal(); dcRender(); }
function dcClose() { const d = $('#compareDlg'); if (d && d.open) d.close(); }
function dcRender() {
  const body = $('#compareBody'); if (!body) return;
  if (typeof cardRender === 'function') cardRender(true);
  const dlg = $('#compareDlg'); if (!dlg || !dlg.open) return;
  const R = DC.res, where = cardLive() ? `the drone (${live.name})` : 'the simulated drone';
  setText($('#compareSub'), DC.on ? DC.said + '…' : R ? `What ${R.live ? 'the drone' : 'the simulated drone'} had at ${new Date(R.at).toLocaleTimeString()}, beside the design.` : `Reads ${where}'s settings and files through its Pi, and lists what differs from the design. It only reads.`);
  body.textContent = '';
  body.append(el('div', { class: 'design-tools' },
    UI.button({ class: 'btn sm primary', type: 'button', text: DC.on ? 'Stop' : R ? 'Read it again' : 'Read the drone', onclick: () => { if (DC.on) DC.stop = true; else { const w = dcStart(); if (w) DC.err = w; } dcRender(); } })));
  if (DC.err) body.append(el('p', { class: 'ui-status bad', role: 'status', text: DC.err }));
  if (!R) return;
  const rows = R.settings, differ = rows.filter(r => r.state === 'differ'), takeable = differ.filter(r => r.take);
  const val = v => v == null ? '—' : String(v);
  const table = list => {
    const t = el('table', { class: 'dc-table' }); t.append(el('tr', {}, el('th', { text: 'Setting' }), el('th', { text: 'Design' }), el('th', { text: 'Drone' }), el('th', { text: '' })));
    for (const r of list) {
      const tr = el('tr', { class: 'dc-' + r.state }, el('td', { text: r.label, title: r.key }), el('td', { text: val(r.design) }), el('td', { text: val(r.drone) }));
      const td = el('td');
      if (r.take) td.append(UI.button({ class: 'btn sm', type: 'button', text: 'Take', title: 'Set the design to the drone\'s (one undo step)', onclick: () => { DC.err = dcTake([r]); dcRender(); } }));
      else if (r.state === 'differ') td.append(el('span', { class: 'hint', text: 'compare only' }));
      tr.append(td); t.append(tr);
    }
    return t;
  };
  body.append(el('h3', { text: 'The ESP32\'s settings' }));
  body.append(el('p', { class: 'ui-status ' + (differ.length ? 'warn' : 'ok'), text: differ.length ? `${differ.length} ${differ.length === 1 ? 'differs' : 'differ'} from the design${takeable.length ? `; ${takeable.length} can be taken into it` : ''}.` : 'Every setting both have is the same.' }));
  if (differ.length) body.append(table(differ));
  if (takeable.length > 1) body.append(el('div', { class: 'design-tools' }, UI.button({ class: 'btn sm', type: 'button', text: `Take all ${takeable.length}`, onclick: () => { DC.err = dcTake(takeable); dcRender(); } })));
  if (R.next && R.next.length) body.append(el('p', { class: 'hint warn', text: `Saved on the ESP32 for its next start, not used yet: ${R.next.map(r => `${r.label} ${r.drone}`).join('; ')} (esp32 reboot uses them).` }));
  const rest = (s, title) => { const l = rows.filter(r => r.state === s); if (l.length) body.append(UI.details({ class: 'dc-more', title: `${title} (${l.length})` }, table(l))); };
  rest('design', 'Only in the design: the drone doesn\'t report them');
  rest('drone', 'Only on the drone: the design doesn\'t set them');
  rest('secret', 'Not shown by the drone (passwords and phrases)');
  rest('same', 'The same');
  body.append(el('h3', { text: 'The Pi\'s files' }));
  const ft = el('table', { class: 'dc-table' }); ft.append(el('tr', {}, el('th', { text: 'File' }), el('th', { text: 'Design' }), el('th', { text: 'Drone' }), el('th', { text: '' })));
  const fs = { same: 'the same', differ: 'different', missing: 'not on the Pi', none: '—', drone: 'only on the Pi' };
  for (const f of R.files) {
    ft.append(el('tr', { class: 'dc-' + (f.state === 'same' || f.state === 'none' ? 'same' : 'differ') }, el('td', { text: f.name }), el('td', { text: f.design == null ? '—' : `${f.design} B` }), el('td', { text: f.drone == null ? '—' : `${f.drone} B` }), el('td', { text: fs[f.state] })));
    if (f.diff) for (const d of f.diff.length ? f.diff : ['the same values, written differently']) ft.append(el('tr', { class: 'dc-note' }, el('td', { colspan: 4, text: '· ' + d })));
  }
  body.append(ft);
  if (R.files.some(f => f.state !== 'same' && f.state !== 'none')) body.append(el('p', { class: 'hint', text: 'Files that differ were put there by an older upload, or by hand. The airframe holds totals, not the parts: change the design until it matches (or Upload design to give the drone the design\'s).' }));
  if (R.filesWhy) body.append(el('p', { class: 'hint warn', text: 'The design\'s own files couldn\'t be made: ' + R.filesWhy }));
  if (R.show && R.show.info.length) body.append(UI.details({ class: 'dc-more', title: 'What else the ESP32 said' }, ...R.show.info.map(l => el('p', { class: 'hint', text: l }))));
}
{ const c = $('#compareClose'); if (c) c.addEventListener('click', dcClose); }
