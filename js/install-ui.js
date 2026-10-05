'use strict';
// Install: putting a board's part of the design on a real board (Computers tab → a board → Install).
// An ESP32 is flashed from the page over USB (Web Serial and esptool-js, js/vendor/esptool.js, loaded when first
// needed) with the firmware in firmware/ (tools/build_firmware.sh), then sent the design over the same cable: the
// airframe, the wiring, edited formulas, as fly.py sends them (the link's frames, rn_link.h). A Raspberry Pi gets a
// step-by-step guide with the commands to paste into its terminal (over SSH): the design's files travel inside them.

const FW_DIR = 'firmware/';
const ESP_MOTOR_PINS = [25, 26, 27, 14, 32, 33, 4, 13], ESP_SERVO_PINS = [16, 17, 18, 19, 23];   // the firmware's defaults (hw.c)
const ESP_OUT_PINS = [4, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33];                   // what it lets drive an output
const LK = { PROGRAM: 1, STATUS: 2, AIRFRAME: 4, SETTING: 5, EVENT: 0x81, REPORT: 0x82, TELEM: 0x83 };
const INST_STATES = ['disarmed', 'ARMED', 'FAILSAFE', 'CRASHED', 'motor test'];
const INST = { target: null, conn: null, busy: false, fw: null, files: null, log: [] };
const instPref = (k, d) => { try { return localStorage.getItem('dfb.install.' + k) ?? d; } catch (e) { return d; } };
const instSave = (k, v) => { try { localStorage.setItem('dfb.install.' + k, v); } catch (e) {} };

/* ───────── MD5 (esptool-js checks what it wrote against the board's own MD5 of the flash) ───────── */
function md5hex(bytes) {
  const K = new Uint32Array(64), S = [7, 12, 17, 22, 5, 9, 14, 20, 4, 11, 16, 23, 6, 10, 15, 21];
  for (let i = 0; i < 64; i++) K[i] = Math.floor(Math.abs(Math.sin(i + 1)) * 2 ** 32);
  const n = bytes.length, len = ((n + 8) >> 6) + 1 << 6, m = new Uint8Array(len); m.set(bytes); m[n] = 0x80;
  const dv = new DataView(m.buffer); dv.setUint32(len - 8, n * 8 >>> 0, true); dv.setUint32(len - 4, Math.floor(n / 0x20000000), true);
  let a0 = 0x67452301, b0 = 0xefcdab89, c0 = 0x98badcfe, d0 = 0x10325476;
  for (let o = 0; o < len; o += 64) {
    let a = a0, b = b0, c = c0, d = d0;
    for (let i = 0; i < 64; i++) {
      let f, g;
      if (i < 16) { f = (b & c) | (~b & d); g = i; } else if (i < 32) { f = (d & b) | (~d & c); g = (5 * i + 1) % 16; }
      else if (i < 48) { f = b ^ c ^ d; g = (3 * i + 5) % 16; } else { f = c ^ (b | ~d); g = (7 * i) % 16; }
      const t = d; d = c; c = b;
      const x = (a + f + K[i] + dv.getUint32(o + g * 4, true)) | 0, s = S[(i >> 4) * 4 + (i & 3)];
      b = (b + ((x << s) | (x >>> (32 - s)))) | 0; a = t;
    }
    a0 = (a0 + a) | 0; b0 = (b0 + b) | 0; c0 = (c0 + c) | 0; d0 = (d0 + d) | 0;
  }
  return [a0, b0, c0, d0].map(v => [0, 8, 16, 24].map(s => ((v >>> s) & 255).toString(16).padStart(2, '0')).join('')).join('');
}

/* ───────── the link's frames: DF, type, length (u32), payload, CRC-32 of type+length+payload ───────── */
function dfFrame(type, payload = new Uint8Array(0)) {
  const p = payload instanceof Uint8Array ? payload : new TextEncoder().encode(String(payload));
  const out = new Uint8Array(11 + p.length), dv = new DataView(out.buffer);
  out[0] = 0x44; out[1] = 0x46; out[2] = type; dv.setUint32(3, p.length, true); out.set(p, 7);
  dv.setUint32(7 + p.length, rnCrc32(out.subarray(2, 7 + p.length)), true);
  return out;
}
// Bytes from the board → frames and lines of text (the boot log, printf), as fly.py reads them.
function dfParser(onFrame, onText) {
  let buf = new Uint8Array(0), text = '';
  const line = s => { for (const l of s.split('\n')) if (l.trim()) onText(l.replace(/\r$/, '')); };
  return bytes => {
    const nb = new Uint8Array(buf.length + bytes.length); nb.set(buf); nb.set(bytes, buf.length); buf = nb;
    for (;;) {
      let i = -1; for (let k = 0; k + 1 < buf.length; k++) if (buf[k] === 0x44 && buf[k + 1] === 0x46) { i = k; break; }
      if (i !== 0) {
        const cut = i < 0 ? buf.length - (buf.length && buf[buf.length - 1] === 0x44 ? 1 : 0) : i;
        text += new TextDecoder().decode(buf.subarray(0, cut)); buf = buf.slice(cut);
        const nl = text.lastIndexOf('\n'); if (nl >= 0) { line(text.slice(0, nl)); text = text.slice(nl + 1); }
        if (text.length > 400) { line(text); text = ''; }
        if (i < 0) return;
        continue;
      }
      if (buf.length < 7) return;
      const dv = new DataView(buf.buffer, buf.byteOffset), type = buf[2], n = dv.getUint32(3, true);
      if (n > 4096) { text += 'DF'; buf = buf.slice(2); continue; }
      if (buf.length < 11 + n) return;
      if (rnCrc32(buf.subarray(2, 7 + n)) !== dv.getUint32(7 + n, true)) { text += 'DF'; buf = buf.slice(2); continue; }
      const payload = buf.slice(7, 7 + n); buf = buf.slice(11 + n); onFrame(type, payload);
    }
  };
}

/* ───────── a board on a USB serial port ───────── */
class BoardConn {
  constructor(port, opts) { this.port = port; this.o = opts; this.waits = []; this.closing = false; }
  async open(baud) {
    this.baud = baud;
    await this.port.open({ baudRate: baud, bufferSize: 1 << 16 });
    // DTR and RTS both off: on most ESP32 boards that's "run" (one without the other holds it in reset or the bootloader)
    try { await this.port.setSignals({ dataTerminalReady: false, requestToSend: false }); } catch (e) {}
    const feed = dfParser((t, p) => this.frame(t, p), s => { this.o.text(s); this.match(s); });
    this.reading = (async () => {
      while (this.port.readable && !this.closing) {
        this.reader = this.port.readable.getReader();
        try { for (;;) { const { value, done } = await this.reader.read(); if (done) break; if (value) feed(value); } }
        catch (e) { if (!this.closing) this.o.text('(the port stopped: ' + e.message + ')'); }
        finally { try { this.reader.releaseLock(); } catch (e) {} }
        if (!this.closing) await new Promise(r => setTimeout(r, 50));
      }
    })();
  }
  frame(type, p) {
    if (type === LK.TELEM && p.length === 144) { this.o.telem(new Float32Array(p.buffer, p.byteOffset, 36)); return; }
    if (type === LK.EVENT || type === LK.REPORT) { const s = new TextDecoder().decode(p).replace(/\s+$/, ''); this.o.text(s, type === LK.EVENT ? 'ev' : 'rep'); this.match(s); }
  }
  match(s) { for (const w of this.waits.slice()) if (w.test(s)) { this.waits.splice(this.waits.indexOf(w), 1); w.done(s); } }
  waitFor(test, ms) {
    return new Promise(res => { const w = { test, done: s => { clearTimeout(w.t); res(s); } }; w.t = setTimeout(() => { this.waits.splice(this.waits.indexOf(w), 1); res(null); }, ms); this.waits.push(w); });
  }
  async write(bytes) { const w = this.port.writable.getWriter(); try { await w.write(bytes); } finally { w.releaseLock(); } }
  async close() {
    this.closing = true;
    try { if (this.reader) await this.reader.cancel(); } catch (e) {}
    try { await this.reading; } catch (e) {}
    try { await this.port.close(); } catch (e) {}
  }
}

/* ───────── which board, what it needs ───────── */
const instKind = t => t === 'ground' ? computers().ground.kind : t.kind;
const instName = t => t === 'ground' ? computers().ground.name : t.name;
const instTasks = t => t === 'ground' ? ['ground'] : t.tasks;
const editedFor = tasks => rnTaskFormulas(tasks).filter(k => LAWS[k] && LAWS[k].status === 'edited');
const designBase = () => ((typeof designs !== 'undefined' && designs.name) || 'drone').replace(/[^\w.-]+/g, '-');
// A file the design makes for a board: the airframe (.dfa), the navigation config (.dnc), the Pi config (.dlc), a program (.rnp).
function instFile(what, t) {
  if (what === 'airframe') return { data: fcAirframeBlob(), ext: '.dfa', pi: 'drone.dfa' };
  if (what === 'nav') return { data: navConfigBlob(), ext: '.dnc', pi: 'drone.dnc' };
  if (what === 'pi') return { data: piConfigBlob(), ext: '.dlc', pi: 'drone.dlc' };
  if (what === 'program') return { data: boardImage(t === 'ground' ? { tasks: ['ground'] } : t), ext: '.rnp', pi: t === 'ground' ? 'command-module.rnp' : 'drone.rnp' };
  throw new Error('no such file');
}
function instDownload(what, t, msgEl) {
  let f; try { f = instFile(what, t); } catch (e) { instMsg(msgEl, 'Can\'t make it: ' + e.message, 'bad'); return; }
  const a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([f.data], { type: 'application/octet-stream' }));
  a.download = (what === 'program' && t === 'ground' ? 'command-module' : designBase()) + f.ext; document.body.append(a); a.click();
  setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
}
const instMsg = (n, text, cls) => { if (!n) return; n.textContent = text; n.className = 'hint inst-msg' + (cls ? ' ' + cls : ''); };
const b64 = bytes => { let s = ''; for (let i = 0; i < bytes.length; i += 0x8000) s += String.fromCharCode.apply(null, bytes.subarray(i, i + 0x8000)); return btoa(s); };
async function sha256hex(bytes) { const h = await crypto.subtle.digest('SHA-256', bytes); return [...new Uint8Array(h)].map(x => x.toString(16).padStart(2, '0')).join(''); }

/* ───────── the dialog ───────── */
function cmdBox(text, label) {
  const code = el('code', { text });
  const copy = el('button', { class: 'btn', type: 'button', text: 'Copy', 'aria-label': 'Copy ' + (label || 'the command') });
  copy.addEventListener('click', async () => {
    let ok = false; try { await navigator.clipboard.writeText(code.textContent); ok = true; } catch (e) {
      const r = document.createRange(); r.selectNodeContents(code); const s = getSelection(); s.removeAllRanges(); s.addRange(r); try { ok = document.execCommand('copy'); } catch (x) {}
    }
    copy.textContent = ok ? 'Copied' : 'Select and copy'; setTimeout(() => { copy.textContent = 'Copy'; }, 1500);
  });
  const box = el('div', { class: 'cmd' }, el('pre', {}, code), copy); box.code = code; return box;
}
const instStep = (n, title, ...kids) => el('section', { class: 'inst-step' }, el('h3', {}, el('span', { class: 'inst-n', text: String(n) }), title), ...kids);
const instPara = (html, cls) => el('p', { class: cls || '', html });

function openInstall(target) {
  INST.target = target;
  const dlg = $('#installDlg'), kind = instKind(target), K = BOARD_KINDS[kind];
  setText($('#installTitle'), `Install on a real board: ${instName(target)}`);
  setText($('#installSub'), `${K.label} · runs ${instTasks(target).map(t => t === 'ground' ? 'the command module' : TASKS[t].label).join(', ') || 'nothing yet'}`);
  const body = $('#installBody'); body.textContent = '';
  if (kind === 'esp32') body.append(...espGuide(target));
  else if (K.mcu) body.append(...otherMcu(target, K));
  else if (target === 'ground') body.append(...groundPiGuide(K));
  else body.append(...piGuide(target, K));
  if (!dlg.open) dlg.showModal();
  dlg.scrollTop = 0; $('#installTitle').focus();   // start at the top, not at its first input
}
function closeInstall() {
  if (INST.busy) { instMsg($('#installBusy'), 'Wait until it finishes: unplugging or closing now leaves the board half written (it can be flashed again).', 'bad'); return; }
  if (INST.conn) { INST.conn.close(); INST.conn = null; }
  $('#installDlg').close();
}

/* ───────── ESP32 ───────── */
function espGuide(t) {
  const ground = t === 'ground', fwName = ground ? 'ground' : 'flight';
  const out = [], tasks = instTasks(t), edited = editedFor(tasks);
  const notes = [];
  if (!ground && !tasks.includes('core')) notes.push('The firmware is the flight controller\'s: on a board that doesn\'t run the flight core, it would fly nothing. Give this board the flight core, or use it for the command module.');
  if (!ground && tasks.includes('nav')) notes.push('The firmware doesn\'t run the navigation on the ESP32 yet: on the real drone, put the navigation on a Raspberry Pi (until then it flies in angle mode).');
  if (!ground && tasks.includes('cargo')) notes.push('The firmware has no latch outputs yet: on the real drone, the latches are driven from a Pi (dfb_pi --latch).');
  if (ground && edited.length) notes.push(`Your edits to ${edited.join(', ')} aren't loaded by the ESP32 command module yet: it runs its built-in formulas.`);
  out.push(instPara(`What goes on it: the ${ground ? 'command module' : 'flight controller'} firmware (the same C the simulator runs for this board), then ${ground ? 'its wiring, typed in below' : 'this design: the airframe, which board pins the motors and servos are on, and any formulas you edited'}.`));
  for (const n of notes) out.push(el('p', { class: 'inst-note', text: n }));
  out.push(el('div', { class: 'inst-safety' }, el('b', { text: 'Before you plug it in' }), el('ul', {},
    el('li', { text: 'Props off. A freshly flashed board, or a wrong pin, can twitch an ESC.' }),
    el('li', { text: 'Battery unplugged (or the ESCs\' 5 V wire off the board) while the USB cable is in: two supplies can push current back into your computer.' }),
    el('li', { text: 'A USB cable that carries data (some only charge). Chrome or Edge on a computer.' }))));
  const serialOk = 'serial' in navigator;
  // 1: firmware
  const prog = el('progress', { max: 1, value: 0, hidden: '' }), msg = el('p', { class: 'hint inst-msg', id: 'installBusy', role: 'status' });
  const erase = el('label', { class: 'check' }, el('input', { type: 'checkbox', id: 'instErase' }), 'Erase the whole board first (forgets the wiring and airframe it saved)');
  const go = el('button', { class: 'btn primary', type: 'button', id: 'instFlash', text: 'Install now over USB' });
  const pick = el('input', { type: 'file', multiple: '', accept: '.bin', hidden: '', id: 'instFiles' });
  const pickBtn = el('button', { class: 'btn', type: 'button', text: 'Use firmware files from this computer…', title: `The three .bin files from firmware/esp32-${fwName}/` });
  pickBtn.addEventListener('click', () => pick.click());
  pick.addEventListener('change', () => { INST.files = [...pick.files]; instMsg(msg, INST.files.length ? `Using ${INST.files.map(f => f.name).join(', ')}.` : ''); });
  go.addEventListener('click', () => espFlash(fwName, prog, msg));
  if (!serialOk) { go.disabled = true; instMsg(msg, 'This browser can\'t reach USB ports: use Chrome or Edge on a computer (not a phone or tablet, not Safari or Firefox), or install by hand (below).', 'bad'); }
  out.push(instStep(1, 'Install the firmware', instPara(`Plug the board in, press <b>Install now</b> and pick its port (<i>CP2102</i>, <i>CH340</i>, <i>USB Serial</i> or <i>USB JTAG</i>). It takes about half a minute. If it can't connect, hold the board's <b>BOOT</b> button while you press Install, and let go once it starts writing.`),
    el('div', { class: 'inst-row' }, go, pickBtn, pick), erase, prog, msg));
  // 2: the design (or wiring for the command module)
  out.push(ground ? espGroundWiring() : espDesign(t, edited));
  // the board's messages
  const con = el('pre', { class: 'inst-console', id: 'instConsole', 'aria-live': 'polite' });
  const live = el('p', { class: 'hint inst-live', id: 'instLive' });
  const line = el('input', { type: 'text', id: 'instLine', placeholder: ground ? 'A command: show, set tx=17,16, save, reboot, status…' : 'A setting: show, set baud=921600, save, reboot…', 'aria-label': 'Send a line to the board', spellcheck: 'false', autocomplete: 'off' });
  const send = el('button', { class: 'btn', type: 'button', text: 'Send' });
  const baud = el('select', { id: 'instBaud', 'aria-label': 'Speed' }, ...(ground ? [115200] : [921600, 460800, 230400, 115200]).map(b => el('option', { value: b, text: b + ' baud' })));
  const conn = el('button', { class: 'btn', type: 'button', id: 'instConn', text: 'Connect' });
  conn.addEventListener('click', async () => { if (INST.conn) { await INST.conn.close(); INST.conn = null; instConnUi(); } else await espConnect(); });
  const sendLine = async () => {
    const s = line.value.trim(); if (!s) return;
    if (!await espConnect()) return;
    instLog('› ' + s, 'me');
    if (ground) await INST.conn.write(new TextEncoder().encode(s + '\n'));
    else if (s === 'status') await INST.conn.write(dfFrame(LK.STATUS));
    else await INST.conn.write(dfFrame(LK.SETTING, s.replace(/^set\s+/, '')));
    line.value = '';
  };
  send.addEventListener('click', sendLine); line.addEventListener('keydown', e => { if (e.key === 'Enter') { e.preventDefault(); sendLine(); } });
  if (!serialOk) [conn, send, line].forEach(x => { x.disabled = true; });
  out.push(instStep(3, 'Its messages', instPara(ground ? 'What it prints over USB (115200 baud). Type its commands here as you would in a terminal.' : 'What it says over USB: its events, and twice a second what it\'s doing. Settings are typed without <code>set</code> or with it: <code>motors=25,26,27,14</code>, then <code>save</code> and <code>reboot</code>; <code>status</code> and <code>show</code> say where it is.'),
    el('div', { class: 'inst-row' }, conn, baud), live, con, el('div', { class: 'inst-row inst-line' }, line, send)));
  out.push(espManual(t, ground, fwName, edited));
  INST.log = []; instConnUi();
  return out;
}
function espDesign(t, edited) {
  const acts = actuators(), js = joints(), msg = el('p', { class: 'hint inst-msg', role: 'status' });
  const mp = el('input', { type: 'text', id: 'instMotors', value: ESP_MOTOR_PINS.slice(0, acts.length).join(','), spellcheck: 'false', 'aria-label': 'Motor pins' });
  const sp = el('input', { type: 'text', id: 'instServos', value: ESP_SERVO_PINS.slice(0, js.length).join(','), spellcheck: 'false', 'aria-label': 'Servo pins' });
  const map = el('ol', { class: 'inst-map' });
  const redraw = () => {
    map.textContent = ''; const m = mp.value.split(/[,\s]+/).filter(Boolean), s = sp.value.split(/[,\s]+/).filter(Boolean);
    acts.forEach((c, i) => map.append(el('li', {}, el('b', { text: c.name }), ` → GPIO ${m[i] ?? '?'}`, m[i] == null ? el('span', { class: 'bad', text: ' (no pin: it won\'t arm)' }) : null)));
    js.forEach((j, i) => map.append(el('li', {}, el('b', { text: j.name }), ` (servo) → GPIO ${s[i] ?? '?'}`, s[i] == null ? el('span', { class: 'bad', text: ' (no pin: it won\'t arm)' }) : null)));
  };
  mp.addEventListener('input', redraw); sp.addEventListener('input', redraw); redraw();
  const sendAf = el('button', { class: 'btn primary', type: 'button', id: 'instSendAf', text: 'Send the airframe' });
  const sendWire = el('button', { class: 'btn', type: 'button', id: 'instSendWire', text: 'Send the wiring (it restarts)' });
  sendAf.addEventListener('click', () => espSendAirframe(msg));
  sendWire.addEventListener('click', () => espSendWiring(mp.value, sp.value, msg));
  const kids = [
    instPara('Once the firmware is on, send it the design over the same cable. Both are kept on the board: send them again only when the design or the wiring changes.'),
    el('div', { class: 'inst-row' }, sendAf),
    instPara(`<b>Wiring:</b> which GPIO each ESC signal and servo is on, in the airframe's order. The defaults avoid the pins that upset booting; the ones it can drive are ${ESP_OUT_PINS.join(', ')}.`),
    el('div', { class: 'inst-pins' }, el('label', {}, el('span', { text: 'Motors' }), mp), js.length ? el('label', {}, el('span', { text: 'Servos' }), sp) : null), map,
    el('div', { class: 'inst-row' }, sendWire),
  ];
  if (edited.length) {
    const sendP = el('button', { class: 'btn', type: 'button', id: 'instSendProg', text: 'Send the edited formulas' });
    sendP.addEventListener('click', () => espSendProgram(msg));
    kids.push(instPara(`You edited ${edited.join(', ')}. The board checks the new formulas, runs them beside its own for a second, then swaps; they last until it restarts (it always starts on its built-in ones), so send them after each power-up, or before a flight.`), el('div', { class: 'inst-row' }, sendP));
  }
  kids.push(msg);
  return instStep(2, 'Send this design', ...kids);
}
function espGroundWiring() {
  return instStep(2, 'Wire it up', instPara('The command module is set up by typing its settings (below, once connected): which pins go to the ExpressLRS transmitter module, the buttons, the sticks, a buzzer. Each takes effect after <code>save</code> and <code>reboot</code>.'),
    cmdBox('set tx=17,16\nset arm=25\nset fly=26\nset roll=34\nset pitch=35\nset throttle=32\nset yaw=33\nset buzzer=27\nset latch=arm,fly\nsave\nreboot', 'the settings'),
    instPara('<code>tx=TX,RX</code>: the ESP32 pin to the module\'s CRSF input, then the one its replies come in on (one pin for a module bay\'s single wire: <code>tx=17,17</code>). A button goes from its pin to ground; a stick\'s wiper to an ADC pin (32–39), centred at power-on (<code>34i</code> inverts it). Not pins 1 and 3 (the USB serial) or 6–11 (the flash); on a WROVER board 16 and 17 are the PSRAM\'s, so pick others for <code>tx</code>.', 'hint'));
}
function espManual(t, ground, fwName, edited) {
  const dir = FW_DIR + 'esp32-' + fwName + '/', app = ground ? 'dfb_ground.bin' : 'dfb_flight.bin', src = ground ? 'runner/ground/esp32' : 'runner/fc/esp32';
  const files = el('table', { class: 'inst-files' }, el('tbody', {},
    ...[['bootloader.bin', '0x1000', 'Espressif\'s second-stage bootloader: starts the firmware.'],
      ['partition-table.bin', '0x8000', 'How the flash is divided: settings storage (nvs, at 0x9000, where the wiring and airframe are kept), and the app.'],
      [app, '0x10000', ground ? 'The command module itself: ground_core.c with the buttons, sticks and CRSF.' : 'The flight controller itself: fc_core.c, the step runner and its built-in formulas, the sensors, ESCs, servos and the link.']]
      .map(([f, at, what]) => el('tr', {}, el('td', {}, el('a', { href: dir + f, download: f, text: f })), el('td', { class: 'mono', text: at }), el('td', { text: what })))));
  const kids = [instPara(`The firmware is three files, each written at its own place in the board's flash (<code>firmware/esp32-${fwName}/</code>):`), files];
  const port = 'PORT';
  kids.push(instPara(`With Espressif's esptool (<code>pip install esptool</code>), in that folder. <code>${port}</code> is the board's port: <code>/dev/ttyUSB0</code> on Linux, <code>/dev/cu.usbserial-…</code> or <code>/dev/cu.SLAB_USBtoUART</code> on a Mac, <code>COM3</code> or similar on Windows:`),
    cmdBox(`esptool.py --chip esp32 -p ${port} -b 460800 write_flash @flash_args`, 'the flash command'),
    instPara('That keeps what the board saved. For a clean start, first <code>esptool.py -p PORT erase_flash</code>. To build it yourself instead (ESP-IDF 5.x):', 'hint'),
    cmdBox(`cd ${src}\nidf.py set-target esp32\nidf.py -p ${port} build flash monitor`, 'the build command'));
  if (!ground) {
    const msg = el('p', { class: 'hint inst-msg', role: 'status' });
    const dl = (what, text) => el('button', { class: 'btn', type: 'button', text, onclick: () => instDownload(what, t, msg) });
    kids.push(instPara('The design, sent with <code>runner/pi/fly.py</code> (<code>pip install pyserial</code>) from a computer or the Pi:'),
      el('div', { class: 'inst-row' }, dl('airframe', `${designBase()}.dfa: the airframe`), ...(edited.length ? [dl('program', `${designBase()}.rnp: the edited formulas`)] : [])), msg,
      cmdBox(`python3 runner/pi/fly.py ${port} airframe ${designBase()}.dfa\npython3 runner/pi/fly.py ${port} set motors=${ESP_MOTOR_PINS.slice(0, actuators().length).join(',')}${joints().length ? ' servos=' + ESP_SERVO_PINS.slice(0, joints().length).join(',') : ''}\npython3 runner/pi/fly.py ${port} save\npython3 runner/pi/fly.py ${port} reboot${edited.length ? `\npython3 runner/pi/fly.py ${port} program ${designBase()}.rnp` : ''}\npython3 runner/pi/fly.py ${port} status`, 'the fly.py commands'),
      instPara('<code>.dfa</code>: the airframe the flight core flies on (each motor\'s force and torque, the servos, the mass and inertia, where the IMU sits), checked by the board and kept in its flash. <code>.rnp</code>: a program, the formulas compiled into the steps the board runs, with self-tests; it lasts until a restart. <code>fly.py PORT test 1 0.1</code> spins motor 1 at 10% for 2 s, props off.', 'hint'));
  } else kids.push(instPara('Its settings can also be typed in any serial terminal at 115200 baud (<code>screen PORT 115200</code>, the Arduino serial monitor).', 'hint'));
  return el('details', { class: 'inst-manual' }, el('summary', { text: 'What the files are, and doing it by hand' }), ...kids);
}
function otherMcu(t, K) {
  return [instPara(`The ready firmware is for the original ESP32 (an ESP32-WROOM or WROVER DevKit). On an ${K.label} the pins and peripherals differ, so it isn't built for it yet: pick <b>ESP32</b> for this board to install it, or port <code>runner/fc/esp32</code> (its pins are in <code>hw.c</code>).`),
    el('p', { class: 'inst-note', text: 'The ESP32\'s firmware can\'t go on it by mistake: the flasher checks which chip it is and stops.' })];
}

function instLog(s, cls) {
  const con = $('#instConsole'); if (!con) return;
  INST.log.push([s, cls]); if (INST.log.length > 400) INST.log.splice(0, INST.log.length - 400);
  const near = con.scrollHeight - con.scrollTop - con.clientHeight < 30;
  con.append(el('span', { class: cls || '', text: s + '\n' }));
  while (con.childNodes.length > 400) con.firstChild.remove();
  if (near) con.scrollTop = con.scrollHeight;
}
function instConnUi() {
  const c = $('#instConn'); if (!c) return;
  c.textContent = INST.conn ? 'Disconnect' : 'Connect'; const b = $('#instBaud'); if (b) b.disabled = !!INST.conn;
  if (!INST.conn) setText($('#instLive'), '');
}
function instTelem(f) {
  const fl = f[13], st = INST_STATES[f[1]] || '?';
  const bits = [st, `roll ${f[2].toFixed(0)}° pitch ${f[3].toFixed(0)}°`, `loop ${f[11].toFixed(0)} µs`];
  if (f[10] > 0) bits.push(f[10].toFixed(2) + ' V');
  bits.push(fl & 1 ? (fl & 4 ? 'gyro ok' : 'settling') : 'NO GYRO', fl & 2 ? 'barometer' : 'no barometer', fl & 16 ? 'airframe loaded' : 'NO AIRFRAME');
  setText($('#instLive'), bits.join(' · '));
}
// Opens the board's port for the link (asking which one, the first time). Returns false if it couldn't.
async function espConnect() {
  if (INST.conn) return true;
  if (!('serial' in navigator)) return false;
  let port = INST.port;
  try { if (!port) port = INST.port = await navigator.serial.requestPort(); } catch (e) { return false; }
  const baud = +($('#instBaud') ? $('#instBaud').value : 921600);
  const c = new BoardConn(port, { text: (s, k) => instLog(s, k), telem: instTelem });
  try { await c.open(baud); } catch (e) { instLog('Couldn\'t open the port: ' + e.message + (/already open|in use/i.test(e.message) ? ' (another program has it: close it there)' : ''), 'bad'); INST.port = null; return false; }
  INST.conn = c; instConnUi(); instLog(`connected at ${baud} baud`, 'me');
  return true;
}
async function espSend(bytes, until, ms, msg, what) {
  if (INST.busy) return null;
  if (!await espConnect()) { instMsg(msg, 'Not connected: plug the board in and pick its port.', 'bad'); return null; }
  INST.busy = true; instMsg(msg, what + '…');
  try { const w = INST.conn.waitFor(until, ms); await INST.conn.write(bytes); return await w; }
  catch (e) { instMsg(msg, 'Couldn\'t send it: ' + e.message, 'bad'); return null; }
  finally { INST.busy = false; }
}
async function espSendAirframe(msg) {
  let blob; try { blob = fcAirframeBlob(); } catch (e) { instMsg(msg, 'This design can\'t be flown by the firmware: ' + e.message, 'bad'); return; }
  const r = await espSend(dfFrame(LK.AIRFRAME, blob), s => /^airframe/.test(s), 4000, msg, `Sending the airframe (${blob.length} bytes)`);
  if (r === null) { if (!/Couldn|Not conn/.test(msg.textContent)) instMsg(msg, 'No answer from the board. Is the firmware on it, and the speed right (921600, or 115200 for a board set up before firmware v3)?', 'bad'); return; }
  instMsg(msg, r === 'airframe loaded and saved' ? 'The board took the airframe and saved it.' : 'The board says: ' + r, r === 'airframe loaded and saved' ? 'good' : 'bad');
}
async function espSendWiring(motors, servos, msg) {
  const lines = ['motors=' + motors.replace(/\s+/g, ''), ...(servos.trim() ? ['servos=' + servos.replace(/\s+/g, '')] : [])];
  for (const l of lines) {
    const r = await espSend(dfFrame(LK.SETTING, l), s => /^set |can't|expected|not a list|at most|unknown|disarm first|GPIO/.test(s), 2000, msg, 'Sending ' + l);
    if (r === null) { if (!/Couldn|Not conn/.test(msg.textContent)) instMsg(msg, 'No answer from the board.', 'bad'); return; }
    if (!/^set /.test(r)) { instMsg(msg, 'The board refused ' + l + ': ' + r, 'bad'); return; }
  }
  const sv = await espSend(dfFrame(LK.SETTING, 'save'), s => /saved|couldn't save|disarm first/.test(s), 2000, msg, 'Saving');
  if (!sv || !/saved/.test(sv)) { instMsg(msg, 'It didn\'t save: ' + (sv || 'no answer'), 'bad'); return; }
  await espSend(dfFrame(LK.SETTING, 'reboot'), s => /rebooting|disarm first/.test(s), 2000, msg, 'Restarting it');
  instMsg(msg, 'Saved: it restarted on the new wiring. Check it with show (below).', 'good');
}
async function espSendProgram(msg) {
  let img; try { img = boardImage(INST.target); } catch (e) { instMsg(msg, 'The formulas don\'t compile: ' + e.message, 'bad'); return; }
  const r = await espSend(dfFrame(LK.PROGRAM, img), s => /^program (swapped|rejected)|didn't|rejected|too big/.test(s), 15000 + img.length / 5, msg, `Sending the formulas (${(img.length / 1024).toFixed(1)} KB)`);
  instMsg(msg, r === null ? 'No answer from the board.' : 'The board says: ' + r, r && /swapped/.test(r) ? 'good' : 'bad');
}

/* ───────── flashing ───────── */
function loadEsptool() {
  if (window.esptool) return Promise.resolve(window.esptool);
  return new Promise((res, rej) => {
    const s = document.createElement('script'); s.src = 'js/vendor/esptool.js';
    s.onload = () => window.esptool ? res(window.esptool) : rej(new Error('the flasher didn\'t load'));
    s.onerror = () => rej(new Error('the flasher (js/vendor/esptool.js) didn\'t load')); document.head.append(s);
  });
}
// The firmware's parts: from the page's firmware/ folder, or the files picked on this computer.
async function firmwareParts(name) {
  if (INST.files && INST.files.length) {
    const want = [[0x1000, /bootloader/i], [0x8000, /partition/i], [0x10000, name === 'ground' ? /dfb_ground/i : /dfb_flight/i]];
    const parts = [];
    for (const [addr, re] of want) {
      const f = INST.files.find(x => re.test(x.name)); if (!f) throw new Error(`pick bootloader.bin, partition-table.bin and ${name === 'ground' ? 'dfb_ground.bin' : 'dfb_flight.bin'} together`);
      parts.push({ address: addr, data: new Uint8Array(await f.arrayBuffer()), name: f.name });
    }
    return { parts, about: 'files from this computer' };
  }
  let man;
  try { const r = await fetch(FW_DIR + 'manifest.json', { cache: 'no-cache' }); if (!r.ok) throw new Error(r.status); man = await r.json(); }
  catch (e) { throw new Error(location.protocol === 'file:' ? 'the page is opened as a file, so it can\'t read its firmware folder: open it from a web address (python3 -m http.server in its folder, then http://localhost:8000), or use “firmware files from this computer”' : 'couldn\'t load firmware/manifest.json'); }
  const fw = man.firmware[name], parts = [];
  for (const [addr, f] of fw.parts) {
    const r = await fetch(FW_DIR + fw.dir + '/' + f, { cache: 'no-cache' }); if (!r.ok) throw new Error('couldn\'t load ' + f);
    const data = new Uint8Array(await r.arrayBuffer()), want = fw.files[f];
    if (want && (data.length !== want.size || await sha256hex(data) !== want.sha256)) throw new Error(f + ' arrived damaged (its checksum doesn\'t match): reload the page and try again');
    parts.push({ address: addr, data, name: f });
  }
  return { parts, about: `built ${man.built} from ${man.commit}` };
}
async function espFlash(name, prog, msg) {
  if (INST.busy) return;
  const erase = $('#instErase') && $('#instErase').checked;
  let fw; try { instMsg(msg, 'Loading the firmware…'); fw = await firmwareParts(name); } catch (e) { instMsg(msg, 'Can\'t install: ' + e.message + '.', 'bad'); return; }
  let E; try { E = await loadEsptool(); } catch (e) { instMsg(msg, 'Can\'t install: ' + e.message + '.', 'bad'); return; }
  if (INST.conn) { await INST.conn.close(); INST.conn = null; instConnUi(); }
  let port = INST.port;
  try { if (!port) port = await navigator.serial.requestPort(); } catch (e) { instMsg(msg, 'No port picked.', ''); return; }
  INST.port = port; INST.busy = true; prog.hidden = false; prog.value = 0; $('#instFlash').disabled = true;
  const say = s => { instMsg(msg, s); instLog(s, 'me'); };
  const term = { clean() {}, writeLine: s => instLog(s), write: s => instLog(s) };
  let transport = null;
  try {
    transport = new E.Transport(port, false);
    const loader = new E.ESPLoader({ transport, baudrate: 460800, terminal: term });
    say('Connecting to the board (it restarts into its bootloader)…');
    await loader.main();
    const chip = loader.chip.CHIP_NAME;
    if (chip !== 'ESP32') throw new Error(`this is an ${chip}; the firmware is for the original ESP32, so nothing was written`);
    say(`${chip} found. ${erase ? 'Erasing it, then writing' : 'Writing'} the firmware (${fw.about})…`);
    const total = fw.parts.reduce((a, p) => a + p.data.length, 0), done = fw.parts.map(() => 0);
    await loader.writeFlash({
      fileArray: fw.parts.map(p => ({ data: p.data, address: p.address })), flashMode: 'keep', flashFreq: 'keep', flashSize: 'keep',
      eraseAll: !!erase, compress: true, calculateMD5Hash: img => md5hex(img instanceof Uint8Array ? img : Uint8Array.from(img, c => c.charCodeAt(0))),
      reportProgress: (i, written, size) => { done[i] = written / size * fw.parts[i].data.length; prog.value = done.reduce((a, b) => a + b, 0) / total; },
    });
    prog.value = 1;
    await loader.after('hard_reset');
    say('Installed and checked: the board restarted on the new firmware.' + (name === 'flight' ? ' Now send it this design (step 2).' : ' Now connect and set up its wiring (step 2).'));
    msg.className = 'hint inst-msg good';
  } catch (e) {
    const m = String(e && e.message || e);
    instMsg(msg, 'It didn\'t install: ' + m + (/timed? ?out|Failed to connect|sync/i.test(m) ? '. Hold the BOOT button while you press Install, try another cable, or close anything else using the port (Arduino, a serial monitor).' : '.'), 'bad');
    instLog('error: ' + m, 'bad');
  } finally {
    try { if (transport) await transport.disconnect(); } catch (e) {}
    INST.busy = false; $('#instFlash').disabled = false;
  }
}

/* ───────── Raspberry Pi ───────── */
function piInputs(fields, onChange) {
  const box = el('div', { class: 'inst-pins' });
  for (const [k, label, def, hint] of fields) {
    const inp = el('input', { type: 'text', value: instPref(k, def), spellcheck: 'false', autocomplete: 'off', 'aria-label': label, title: hint || '' });
    inp.addEventListener('input', () => { instSave(k, inp.value.trim()); onChange(); });
    box.append(el('label', {}, el('span', { text: label }), inp));
  }
  return box;
}
// The design's files as one block to paste into the Pi's terminal: each file in base64, checked with its SHA-256.
async function pastePack(files, after) {
  const lines = ['mkdir -p ~/dfb && cd ~/dfb'];
  for (const f of files) {
    lines.push(`base64 -d > ${f.pi} <<'EOF'`, ...b64(f.data).match(/.{1,76}/g), 'EOF');
  }
  lines.push(`sha256sum -c <<'EOF'`, ...await Promise.all(files.map(async f => `${await sha256hex(f.data)}  ${f.pi}`)), 'EOF');
  if (after) lines.push(after);
  return lines.join('\n');
}
function piGuide(t, K) {
  const tasks = t.tasks, learnOrSuper = tasks.includes('learn') || tasks.includes('super');
  const fix = sensorsOf('fix').some(c => wiredTo(c) === t) && tasks.includes('nav'), nL = latches().length, cargo = tasks.includes('cargo') && nL > 0, radio = tasks.includes('tlm');
  const core = boardOf('core'), bt = t.kind !== 'pizero';
  const fields = [['host', 'Pi (user@address)', 'pi@raspberrypi.local', 'What you type after ssh']];
  if (fix) fields.push(['gps', 'GPS port', '/dev/ttyUSB0', 'A USB GPS: /dev/ttyUSB0 or /dev/ttyACM0']);
  fields.push(['link', 'Link to the ESP32', '/dev/serial0', '/dev/serial0 for the GPIO pins, /dev/ttyUSB0 for a USB cable to the ESP32']);
  if (cargo) fields.push(['latch', 'Latch outputs', ['pwm0', 'pwm1', 'gpio17', 'gpio27', 'gpio22', 'gpio23', 'gpio24', 'gpio25'].slice(0, nL).join(','), 'pwmN: a servo on hardware PWM channel N; gpioN: an on/off line']);
  if (radio) fields.push(['crsf', 'Receiver port', K.label === 'Raspberry Pi 4' ? '/dev/ttyAMA1' : '/dev/ttyUSB1', 'The ExpressLRS receiver\'s serial port on the Pi']);
  const need = ['nav', ...(learnOrSuper ? ['airframe', 'pi'] : [])];
  const cmds = {};
  const out = [];
  const notes = [];
  if (!tasks.includes('nav')) notes.push('dfb_pi always runs the navigation (it needs the .dnc): with this board\'s navigation off in the simulator, the real Pi still runs it.');
  if (editedFor(tasks).length) notes.push(`Your edits to ${editedFor(tasks).join(', ')} aren't loaded by dfb_pi yet: it runs its built-in formulas.`);
  if (tasks.includes('cargo') && !nL) notes.push('This design has no latches, so the cargo task has nothing to drive.');
  out.push(instPara(`The Pi runs <code>dfb_pi</code>: ${tasks.map(x => TASKS[x].label.toLowerCase()).join(', ') || 'the navigation'}, the same C as this simulator's ${t.name}. It's built on the Pi from the source, and this design's files go in beside it. You need Raspberry Pi OS (Lite is fine) with its network and SSH working: everything below is typed into <code>ssh ${'<span class="inst-host"></span>'}</code>, except where it says your computer.`));
  for (const n of notes) out.push(el('p', { class: 'inst-note', text: n }));
  const inputs = piInputs(fields, () => refresh());
  out.push(el('div', { class: 'inst-safety' }, el('b', { text: 'Your Pi' }), inputs));
  const v = k => instPref(k, (fields.find(f => f[0] === k) || [])[2] || '');
  // the steps
  out.push(instStep(1, 'Wire it to the flight controller', instPara(core ? `To <b>${core.name}</b> (${BOARD_KINDS[core.kind].label}), on its USB serial port's pins. Both sides are 3.3 V: no level shifter.` : 'To the flight controller, on its USB serial port\'s pins.'),
    el('table', { class: 'inst-files' }, el('tbody', {},
      el('tr', {}, el('td', { text: 'Pi pin 8 (GPIO 14, TX)' }), el('td', { text: '→' }), el('td', { text: 'ESP32 GPIO 3 (RX0)' })),
      el('tr', {}, el('td', { text: 'Pi pin 10 (GPIO 15, RX)' }), el('td', { text: '←' }), el('td', { text: 'ESP32 GPIO 1 (TX0)' })),
      el('tr', {}, el('td', { text: 'Pi pin 6 (GND)' }), el('td', { text: '—' }), el('td', { text: 'ESP32 GND' })))),
    instPara('Those are the same pins as the ESP32\'s USB port, so unplug its USB cable while the Pi is connected. With a free USB port on the Pi, a USB cable to the ESP32 does the same: set the link above to <code>/dev/ttyUSB0</code>.', 'hint')));
  cmds.serial = cmdBox('', 'the serial port commands');
  out.push(instStep(2, 'Turn on its serial port', instPara(`It frees the Pi's main serial port for the link: no login console on it, and Bluetooth moved off it (on a Zero W, Zero 2 W or Pi 4 it has that port, and the other one is too unsteady at 921600 baud). Then it restarts.`), cmds.serial));
  cmds.copy = cmdBox('', 'the copy command'); cmds.clone = cmdBox('', 'the clone command');
  out.push(instStep(3, 'Copy the source code to it', instPara('<b>On your computer</b>, in this repository\'s folder (where <code>index.html</code> is):'), cmds.copy,
    instPara('Or, if the Pi can reach the repository itself (a private one needs your GitHub login on the Pi):', 'hint'), cmds.clone));
  cmds.build = cmdBox('', 'the build commands');
  out.push(instStep(4, 'Build it', instPara('The compiler first (it may already be there), then dfb_pi: a single compile, about a minute on a Pi Zero. It ends with <code>built dfb_pi</code>.'), cmds.build));
  cmds.files = cmdBox('', 'the design files block');
  const dmsg = el('p', { class: 'hint inst-msg', role: 'status' });
  const dl = el('div', { class: 'inst-row' }, ...need.map(w => el('button', { class: 'btn', type: 'button', text: `${designBase()}${w === 'nav' ? '.dnc' : w === 'airframe' ? '.dfa' : '.dlc'}`, title: 'Download it', onclick: () => instDownload(w, t, dmsg) })));
  out.push(instStep(5, 'Put this design on it', instPara(`One block with the files inside it: paste it into the Pi's terminal. It writes them to <code>~/dfb</code> and checks each arrived whole (<code>OK</code>). ${need.length > 1 ? '<code>drone.dnc</code>: where its sensors sit, for the navigation; <code>drone.dfa</code>: the airframe, as the flight controller has it, for the learning and the supervisor to start from; <code>drone.dlc</code>: where the IMU sits, each motor\'s heat model and the battery.' : '<code>drone.dnc</code>: the mass and where its sensors sit, for the navigation.'}`), cmds.files,
    instPara('Do this again whenever the design changes. Or download them and copy them across with <code>scp</code>, renamed to <code>drone.dnc</code>…:', 'hint'), dl, dmsg));
  cmds.run = cmdBox('', 'the run command');
  out.push(instStep(6, 'Try it', instPara('With the flight controller on (props off). It says what it found (the link, the GPS) and takes commands as you type: <code>status</code>, <code>health</code>, <code>learning</code>. Ctrl+C stops it.'), cmds.run));
  cmds.svc = cmdBox('', 'the service commands');
  out.push(instStep(7, 'Start it at power-up', instPara('As a service, it starts when the Pi boots and again if it ever stops. Its commands then come over UDP (port 14560) instead of the keyboard.'), cmds.svc,
    instPara('Its output: <code>journalctl -u dfb -f</code>. A command: <code>echo status | nc -u -w1 127.0.0.1 14560</code>. After a new design (step 5): <code>sudo systemctl restart dfb</code>. After new code (steps 3 and 4): the same.', 'hint')));
  out.push(el('details', { class: 'inst-manual' }, el('summary', { text: 'Good to know' }),
    instPara('<b>Power.</b> Cutting the drone\'s battery cuts the Pi without warning, which can damage what it was writing on the SD card. Its files are written once and only read in flight, so the risk is small; for more, turn on the read-only overlay (<code>sudo raspi-config</code> → Performance → Overlay file system) once it\'s all set up, and off again to change the design.'),
    instPara('<b>Boot time.</b> A Pi Zero takes 20–30 s to start. The ESP32 flies without it meanwhile (angle mode), and the navigation, learning and supervisor join in when it\'s up.'),
    instPara('<b>Where things are.</b> <code>~/dfb/runner</code>: the source and <code>runner/pi/dfb_pi</code>; <code>~/dfb/drone.*</code>: the design; <code>/etc/systemd/system/dfb.service</code>: the service. <code>runner/pi/fly.py</code> talks to the ESP32 from the Pi too (stop the service first: one program on the port at a time).')));
  const refresh = async () => {
    const host = v('host') || 'pi@raspberrypi.local';
    out[0].querySelectorAll('.inst-host').forEach(n => { n.textContent = host; });
    cmds.serial.code.textContent = [
      'sudo raspi-config nonint do_serial_cons 1    # no login console on it',
      'sudo raspi-config nonint do_serial_hw 0      # the port itself on',
      'CFG=/boot/firmware/config.txt; [ -f $CFG ] || CFG=/boot/config.txt',
      'grep -q "^dtoverlay=disable-bt" $CFG || echo "dtoverlay=disable-bt" | sudo tee -a $CFG    # Bluetooth off the port' + (bt ? '' : ' (nothing to do on a Zero without W)'),
      'sudo systemctl disable hciuart 2>/dev/null; true',
      ...(cargo && /pwm/.test(v('latch')) ? ['grep -q "^dtoverlay=pwm-2chan" $CFG || echo "dtoverlay=pwm-2chan" | sudo tee -a $CFG    # hardware PWM for the latches (GPIO 18, 19)'] : []),
      'sudo usermod -aG dialout,gpio $USER',
      'sudo reboot'].join('\n');
    cmds.copy.code.textContent = `ssh ${host} "mkdir -p ~/dfb"\nscp -r runner ${host}:~/dfb/`;
    cmds.clone.code.textContent = `git clone --depth 1 https://github.com/mrtksn/drone-force-bench.git ~/dfb`;
    cmds.build.code.textContent = 'sudo apt update && sudo apt install -y build-essential\ncd ~/dfb && sh runner/pi/build.sh';
    let files = []; try { files = need.map(w => instFile(w, t)); } catch (e) { cmds.files.code.textContent = '# This design can\'t be exported: ' + e.message; }
    if (files.length) cmds.files.code.textContent = await pastePack(files, 'sudo systemctl restart dfb 2>/dev/null; true');
    const args = ['--nav drone.dnc', ...(learnOrSuper ? ['--airframe drone.dfa --pi drone.dlc'] : []), ...(v('link') && v('link') !== '/dev/serial0' ? ['--link ' + v('link')] : []),
      ...(!tasks.includes('learn') && learnOrSuper ? ['--no-learning'] : []), ...(!tasks.includes('super') && learnOrSuper ? ['--no-supervisor'] : []),
      ...(fix ? ['--gps ' + v('gps')] : []), ...(cargo ? ['--latch ' + v('latch')] : []), ...(radio ? ['--crsf ' + v('crsf')] : [])].join(' ');
    cmds.run.code.textContent = `cd ~/dfb && ./runner/pi/dfb_pi ${args}`;
    cmds.svc.code.textContent = `sudo tee /etc/systemd/system/dfb.service > /dev/null <<EOF\n[Unit]\nDescription=Drone Force Bench: dfb_pi\nAfter=network.target\n\n[Service]\nUser=$USER\nWorkingDirectory=$HOME/dfb\nExecStart=$HOME/dfb/runner/pi/dfb_pi ${args}\nRestart=always\nRestartSec=2\n\n[Install]\nWantedBy=multi-user.target\nEOF\nsudo systemctl daemon-reload\nsudo systemctl enable --now dfb`;
  };
  refresh();
  return out;
}
// The command module on a Pi or a Mac: dfb_ground, built from the source.
function groundPiGuide(K) {
  const mac = K.groundOnly, edited = editedFor(['ground']);
  const fields = mac ? [] : [['host', 'Pi (user@address)', 'pi@raspberrypi.local', 'What you type after ssh']];
  fields.push(['tx', 'Transmitter module port', mac ? '/dev/cu.usbserial-0001' : '/dev/ttyUSB0', 'The ExpressLRS transmitter module\'s USB serial port']);
  const v = k => instPref(k, (fields.find(f => f[0] === k) || [])[2] || '');
  const out = [], cmds = { copy: cmdBox('', 'the copy command'), build: cmdBox('', 'the build commands'), run: cmdBox('', 'the run command') };
  out.push(instPara(`It runs <code>dfb_ground</code>, the command module's C (the same this simulator runs on the ground), wired to the ExpressLRS transmitter module over USB serial, with the terminal's keys, a gamepad or your own code over UDP.${mac ? ' Everything below is typed into a terminal on this computer.' : ''}`));
  out.push(el('div', { class: 'inst-safety' }, el('b', { text: mac ? 'This computer' : 'Your Pi' }), piInputs(fields, () => refresh())));
  if (!mac) out.push(instStep(1, 'Copy the source code to it', instPara('<b>On your computer</b>, in this repository\'s folder:'), cmds.copy));
  out.push(instStep(mac ? 1 : 2, 'Build it', instPara(mac ? 'In this repository\'s folder (it needs the Xcode command line tools: <code>xcode-select --install</code>):' : 'The compiler first, then dfb_ground:'), cmds.build));
  const steps = [cmds.run, instPara('The keys: W/S climb and sink, A/D turn, the arrows move, Space holds, H flies home, R arms, T takes off and lands. <code>--joystick /dev/input/js0</code> reads a gamepad on Linux; UDP port 14561 takes text commands from scripts.', 'hint')];
  if (edited.length) {
    cmds.prog = cmdBox('', 'the program block');
    steps.push(instPara(`You edited ${edited.join(', ')}: put the program next to it (paste this${mac ? ' in the repository\'s folder' : ''}), and it's in the command above.`), cmds.prog);
  }
  out.push(instStep(mac ? 2 : 3, 'Run it', ...steps));
  const refresh = async () => {
    const host = v('host') || 'pi@raspberrypi.local';
    cmds.copy.code.textContent = `ssh ${host} "mkdir -p ~/dfb"\nscp -r runner ${host}:~/dfb/`;
    cmds.build.code.textContent = mac ? 'sh runner/ground/build.sh' : 'sudo apt update && sudo apt install -y build-essential\ncd ~/dfb && sh runner/ground/build.sh';
    cmds.run.code.textContent = `${mac ? '' : 'cd ~/dfb && '}./runner/ground/dfb_ground --tx ${v('tx')} --keys${edited.length ? ' --program command-module.rnp' : ''}`;
    if (cmds.prog) { try { const f = instFile('program', 'ground'); cmds.prog.code.textContent = (await pastePack([f])).replace('mkdir -p ~/dfb && cd ~/dfb', mac ? '# (in the repository\'s folder)' : 'mkdir -p ~/dfb && cd ~/dfb'); } catch (e) { cmds.prog.code.textContent = '# The formulas don\'t compile: ' + e.message; } }
  };
  refresh();
  return out;
}

$('#installClose').addEventListener('click', closeInstall);
$('#installDlg').addEventListener('cancel', e => { e.preventDefault(); closeInstall(); });
if ('serial' in navigator) navigator.serial.addEventListener('disconnect', e => {
  if (INST.port && e.target === INST.port) { if (INST.conn) { INST.conn.close(); INST.conn = null; } INST.port = null; instConnUi(); instLog('the board was unplugged', 'bad'); }
});
