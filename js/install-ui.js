'use strict';
// Install: putting a board's part of the design on a real board (Computers tab → a board → Install).
// An ESP32 is flashed from the page over USB (Web Serial and esptool-js, js/vendor/esptool.js, loaded when first
// needed) with the firmware in firmware/ (tools/build_firmware.sh), then sent the design over the same cable: the
// airframe, the wiring, edited formulas, as fly.py sends them (the link's frames, rn_link.h). A Raspberry Pi gets a
// step-by-step guide with the commands to paste into its terminal (over SSH): the design's files travel inside them.

const FW_DIR = 'firmware/';
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

const shq = s => /^[A-Za-z0-9_.,:@%+=\/-]+$/.test(s) ? s : `'${String(s).replace(/'/g, `'\\''`)}'`;   // a word for the shell
/* ───────── which board, what it needs ───────── */
const instKind = t => t === 'ground' ? computers().ground.kind : t.kind;
const instProfile = t => ESP_PROFILES[instKind(t)];
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
const instMsg = (n, text, cls) => { if (!n) return; n.textContent = text; n.className = 'ui-status hint inst-msg' + (cls ? ' ' + cls : ''); };
const b64 = bytes => { let s = ''; for (let i = 0; i < bytes.length; i += 0x8000) s += String.fromCharCode.apply(null, bytes.subarray(i, i + 0x8000)); return btoa(s); };
async function sha256hex(bytes) { const h = await crypto.subtle.digest('SHA-256', bytes); return [...new Uint8Array(h)].map(x => x.toString(16).padStart(2, '0')).join(''); }

/* ───────── the dialog ───────── */
function cmdBox(text, label) {
  const code = el('code', { text });
  const copy = UI.button( { class: 'btn', type: 'button', text: 'Copy', 'aria-label': 'Copy ' + (label || 'the command') });
  copy.addEventListener('click', async () => {
    let ok = false; try { await navigator.clipboard.writeText(code.textContent); ok = true; } catch (e) {
      const r = document.createRange(); r.selectNodeContents(code); const s = getSelection(); s.removeAllRanges(); s.addRange(r); try { ok = document.execCommand('copy'); } catch (x) {}
    }
    copy.textContent = ok ? 'Copied' : 'Select and copy'; setTimeout(() => { copy.textContent = 'Copy'; }, 1500);
  });
  const box = el('div', { class: 'cmd' }, el('pre', {}, code), copy); box.code = code; return box;
}
const instStep = (n, title, ...kids) => UI.section( { class: 'inst-step' }, el('h3', {}, el('span', { class: 'inst-n', text: String(n) }), title), ...kids);
const instPara = (html, cls) => el('p', { class: cls || '', html });

function openInstall(target) {
  INST.target = target;
  const dlg = $('#installDlg'), kind = instKind(target), K = BOARD_KINDS[kind];
  setText($('#installTitle'), `Install on a real board: ${instName(target)}`);
  setText($('#installSub'), `${K.label} · runs ${instTasks(target).map(t => t === 'ground' ? 'the command module' : TASKS[t].label).join(', ') || 'nothing yet'}`);
  const body = $('#installBody'); body.textContent = '';
  if (K.mcu) body.append(...espGuide(target));
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
  const ground = t === 'ground', fwName = ground ? 'ground' : 'flight', profile = instProfile(t);
  INST.files = null; INST.customFirmware = false;
  const out = [], tasks = instTasks(t), edited = editedFor(tasks);
  const notes = [];
  if (!ground && !tasks.includes('core')) notes.push('The firmware is the flight controller\'s: on a board that doesn\'t run the flight core, it would fly nothing. Give this board the flight core, or use it for the command module.');
  if (!ground && tasks.includes('nav')) notes.push('The firmware doesn\'t run the navigation on the ESP32 yet: on the real drone, put the navigation on a Raspberry Pi (until then it flies in angle mode).');
  if (!ground && tasks.includes('cargo')) notes.push('The firmware has no latch outputs yet: on the real drone, the latches are driven from a Pi (dfb_pi --latch).');
  if (ground && edited.length) notes.push(`Your edits to ${edited.join(', ')} aren't loaded by the ESP32 command module yet: it runs its built-in formulas.`);
  out.push(el('p', { class: 'inst-note', text: `${profile.label}: ${profile.outputs} total motor/servo PWM outputs; flight loop defaults to ${profile.rate} Hz. The running firmware communicates on UART0; use a USB-to-UART connector or adapter. Native USB can flash the chip, but does not carry this firmware’s design/console link. Check the module schematic before wiring.` }));
  out.push(instPara(`What goes on it: the ${ground ? 'command module' : 'flight controller'} firmware (the same C the simulator runs for this board), then ${ground ? 'its wiring, typed in below' : 'this design: the airframe, which board pins the motors and servos are on, and any formulas you edited'}.`));
  for (const n of notes) out.push(el('p', { class: 'inst-note', text: n }));
  out.push(el('div', { class: 'inst-safety' }, el('b', { text: 'Before you plug it in' }), el('ul', {},
    el('li', { text: 'Props off. A freshly flashed board, or a wrong pin, can twitch an ESC.' }),
    el('li', { text: 'Battery unplugged (or the ESCs\' 5 V wire off the board) while the USB cable is in: two supplies can push current back into your computer.' }),
    el('li', { text: 'A USB cable that carries data (some only charge). Chrome or Edge on a computer.' }))));
  const serialOk = 'serial' in navigator;
  // 1: firmware
  const prog = el('progress', { max: 1, value: 0, hidden: '' }), msg = UI.status( { class: 'hint inst-msg', id: 'installBusy', role: 'status' });
  const erase = el('label', { class: 'check' }, UI.input( { type: 'checkbox', id: 'instErase' }), 'Erase the whole board first (forgets the wiring and airframe it saved)');
  const go = UI.button( { class: 'btn primary', type: 'button', id: 'instFlash', text: 'Install now over USB' });
  const pick = UI.input( { type: 'file', multiple: '', accept: '.bin', hidden: '', id: 'instFiles' });
  const pickBtn = UI.button( { class: 'btn', type: 'button', text: 'Use firmware files from this computer…', title: `The three .bin files from firmware/${profile.chip}-${fwName}/` });
  pickBtn.addEventListener('click', () => pick.click());
  pick.addEventListener('change', () => { INST.files = [...pick.files]; instMsg(msg, INST.files.length ? `Using ${INST.files.map(f => f.name).join(', ')}.` : ''); });
  go.addEventListener('click', () => espFlash(fwName, profile, prog, msg));
  if (!serialOk) { go.disabled = true; instMsg(msg, 'This browser can\'t reach USB ports: use Chrome or Edge on a computer (not a phone or tablet, not Safari or Firefox), or install by hand (below).', 'bad'); }
  out.push(instStep(1, 'Install the firmware', instPara(`Plug the board in, press <b>Install now</b> and pick its port (<i>CP2102</i>, <i>CH340</i> or <i>USB Serial</i>). It takes about half a minute. If it can't connect, hold the board's <b>BOOT</b> button while you press Install, and let go once it starts writing.`),
    el('div', { class: 'inst-row' }, go, pickBtn, pick), erase, prog, msg));
  // 2: the design (or wiring for the command module)
  out.push(ground ? espGroundWiring(profile) : espDesign(t, edited));
  // the board's messages
  const con = el('pre', { class: 'inst-console', id: 'instConsole', 'aria-live': 'polite' });
  const live = el('p', { class: 'hint inst-live', id: 'instLive' });
  const line = UI.input( { type: 'text', id: 'instLine', placeholder: ground ? 'A command: show, set tx=17,16, save, reboot, status…' : 'A setting: show, set baud=921600, save, reboot…', 'aria-label': 'Send a line to the board', spellcheck: 'false', autocomplete: 'off' });
  const send = UI.button( { class: 'btn', type: 'button', text: 'Send' });
  const baud = UI.select( { id: 'instBaud', 'aria-label': 'Speed' }, ...(ground ? [115200] : [921600, 460800, 230400, 115200]).map(b => el('option', { value: b, text: b + ' baud' })));
  const conn = UI.button( { class: 'btn', type: 'button', id: 'instConn', text: 'Connect' });
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
  out.push(instStep(3, 'Its messages', instPara(ground ? 'What it prints over USB (115200 baud). Type its commands here as you would in a terminal.' : `What it says over USB: its events, and twice a second what it's doing. Settings are typed without <code>set</code> or with it: <code>motors=${profile.motors.slice(0,4).join(',')}</code>, then <code>save</code> and <code>reboot</code>; <code>status</code> and <code>show</code> say where it is.`),
    el('div', { class: 'inst-row' }, conn, baud), live, con, el('div', { class: 'inst-row inst-line' }, line, send)));
  out.push(espManual(t, ground, fwName, edited));
  INST.log = []; instConnUi();
  return out;
}
function espDesign(t, edited) {
  const profile = instProfile(t), wiring = boardWiringPlan(t);
  const acts = actuators(), js = joints(), msg = UI.status( { class: 'hint inst-msg', role: 'status' });
  const mp = UI.input( { type: 'text', id: 'instMotors', value: wiring.motors.join(','), readonly: 'readonly', spellcheck: 'false', 'aria-label': 'Motor pins' });
  const sp = UI.input( { type: 'text', id: 'instServos', value: wiring.servos.join(','), readonly: 'readonly', spellcheck: 'false', 'aria-label': 'Servo pins' });
  const map = el('ol', { class: 'inst-map' });
  const redraw = () => {
    map.textContent = ''; const m = mp.value.split(/[,\s]+/).filter(Boolean), s = sp.value.split(/[,\s]+/).filter(Boolean);
    acts.forEach((c, i) => map.append(el('li', {}, el('b', { text: c.name }), ` → GPIO ${m[i] ?? '?'} · ${wiring.motorConfigs[i]?.driver==='brushed'?'MOSFET, '+wiring.bus.brushedHz+' Hz, max '+wiring.motorConfigs[i].maxDuty+'%':'PWM ESC'}`, m[i] == null ? el('span', { class: 'bad', text: ' (no pin: it won\'t arm)' }) : null)));
    js.forEach((j, i) => map.append(el('li', {}, el('b', { text: j.name }), ` (servo) → GPIO ${s[i] ?? '?'}`, s[i] == null ? el('span', { class: 'bad', text: ' (no pin: it won\'t arm)' }) : null)));
  };
  mp.addEventListener('input', redraw); sp.addEventListener('input', redraw); redraw();
  const sendAf = UI.button( { class: 'btn primary', type: 'button', id: 'instSendAf', text: 'Send the airframe' });
  const sendWire = UI.button( { class: 'btn', type: 'button', id: 'instSendWire', text: 'Send hardware settings (it restarts)' });
  sendAf.addEventListener('click', () => espSendAirframe(msg));
  sendWire.addEventListener('click', () => espSendWiring(mp.value, sp.value, msg));
  const kids = [
    ...(wiring.motorConfigs.some(m=>m.driver==='brushed')?[el('p',{class:'inst-note',text:'For MOSFET motors, keep motor power disconnected until these saved driver settings have been sent and the board has restarted. Factory/default wiring uses ESC pulses, which are not a stopped MOSFET signal.'})]:[]),
    instPara('Once the firmware is on, send the wiring first and let it restart, then send the airframe. Use the UART0 USB-to-serial connection. Both are kept on the board: send them again when the design or wiring changes.'),
    instPara(`Battery ADC: ${wiring.bus.batteryPin<0?'not connected':'GPIO '+wiring.bus.batteryPin+' · divider '+wiring.bus.batteryDivider}. ${!wiring.radioBoard?'':radioCfg.kind==='elrs'||radioCfg.kind==='serial'?`${radioCfg.kind==='serial'?'Serial line':'Receiver'} UART: ${wiring.bus.crsfRx<0?'not connected':'board RX GPIO '+wiring.bus.crsfRx+' / TX GPIO '+wiring.bus.crsfTx}. `:''}${wiring.radioBoard&&radioCfg.kind==='nrf24'?`nRF24L01 SPI: ${(wiring.bus.nrfPins||[]).every(p=>p>=0)?'GPIO '+wiring.bus.nrfPins.join(', '):'not connected'} (SCK, MOSI, MISO, CSN, CE). `:''}${wiring.radioBoard?`Radio: <code>${radioSettingLines(radioCfg).join(' ')}</code> (the link and binding phrase from the Ground tab, sent with the hardware settings)${radioCfg.kind==='elrs'||radioCfg.kind==='serial'?'':'; built in, nothing to wire'}.`:''}`),
    instPara(`<b>Wiring:</b> which GPIO each ESC signal and servo is on, in the airframe's order. I²C is GPIO ${wiring.bus.sda}, ${wiring.bus.scl}. Change these assignments in <b>Hardware wiring</b> in Computers. The defaults avoid the pins that upset booting; the ones it can drive are ${profile.pins.join(', ')}.`),
    el('div', { class: 'inst-pins' }, el('label', {}, el('span', { text: 'Motors' }), mp), js.length ? el('label', {}, el('span', { text: 'Servos' }), sp) : null), map,
    el('div', { class: 'inst-row' }, sendWire),
    el('div', { class: 'inst-row' }, sendAf),
  ];
  if (edited.length) {
    const sendP = UI.button( { class: 'btn', type: 'button', id: 'instSendProg', text: 'Send the edited formulas' });
    sendP.addEventListener('click', () => espSendProgram(msg));
    kids.push(instPara(`You edited ${edited.join(', ')}. The board checks the new formulas, runs them beside its own for a second, then swaps; they last until it restarts (it always starts on its built-in ones), so send them after each power-up, or before a flight.`), el('div', { class: 'inst-row' }, sendP));
  }
  if(Object.values(wiring.sensors).some(s=>s.driver==='custom')) { const ack=UI.input({type:'checkbox',id:'instCustomBuilt'});ack.addEventListener('change',()=>{INST.customFirmware=ack.checked;});kids.push(el('label',{class:'check'},ack,'This board already runs firmware rebuilt with this design’s custom C driver')); }
  for(const text of [...wiring.errors,...wiring.warnings]) kids.push(el('p',{class:'inst-note',text}));
  kids.push(msg);
  return instStep(2, 'Send this design', ...kids);
}
function espGroundWiring(profile) {
  const errors=groundHardwareErrors(computers());
  return instStep(2, 'Wire it up', instPara('Choose transmitter, button, stick and buzzer GPIOs in <b>Hardware wiring</b> in Computers. These saved settings take effect after <code>save</code> and <code>reboot</code>.'),
    ...(errors.length?errors.map(text=>el('p',{class:'bad',text})):[cmdBox(groundHardwareSettings(computers()), 'the settings')]),
    instPara(`<code>tx=TX,RX</code>: to the module's CRSF input, then from its output; one pin for a module bay's single wire. Buttons connect to ground; sticks use free ADC1 pins (${profile.adc.join(', ')}). Keep all assignments distinct. ${profile.chip === 'esp32' ? 'On WROVER modules 16/17 belong to PSRAM; choose other pins.' : 'Native USB, flash/PSRAM and boot strap pins are reserved.'}`, 'hint'));

}
function espManual(t, ground, fwName, edited) {
  const profile = instProfile(t);
  const dir = FW_DIR + profile.chip + '-' + fwName + '/', app = ground ? 'dfb_ground.bin' : 'dfb_flight.bin', src = ground ? 'runner/ground/esp32' : 'runner/fc/esp32';
  const files = el('table', { class: 'inst-files' }, el('tbody', {},
    ...[['bootloader.bin', '0x' + profile.boot.toString(16), 'Espressif\'s second-stage bootloader: starts the firmware.'],
      ['partition-table.bin', '0x8000', 'How the flash is divided: settings storage (nvs, at 0x9000, where the wiring and airframe are kept), and the app.'],
      [app, '0x10000', ground ? 'The command module itself: ground_core.c with the buttons, sticks and CRSF.' : 'The flight controller itself: fc_core.c, the step runner and its built-in formulas, the sensors, ESCs, servos and the link.']]
      .map(([f, at, what]) => el('tr', {}, el('td', {}, el('a', { href: dir + f, download: f, text: f })), el('td', { class: 'mono', text: at }), el('td', { text: what })))));
  const kids = [instPara(`The firmware is three files, each written at its own place in the board's flash (<code>firmware/${profile.chip}-${fwName}/</code>):`), files];
  const port = 'PORT';
  kids.push(instPara(`With Espressif's esptool (<code>pip install esptool</code>), in that folder. <code>${port}</code> is the board's port: <code>/dev/ttyUSB0</code> on Linux, <code>/dev/cu.usbserial-…</code> or <code>/dev/cu.SLAB_USBtoUART</code> on a Mac, <code>COM3</code> or similar on Windows:`),
    cmdBox(`esptool.py --chip ${profile.chip} -p ${port} -b 460800 write_flash @flash_args`, 'the flash command'),
    instPara('That keeps what the board saved. For a clean start, first <code>esptool.py -p PORT erase_flash</code>. To build it yourself instead (ESP-IDF 5.x):', 'hint'),
    cmdBox(`cd ${src}\nidf.py set-target ${profile.chip}\nidf.py -p ${port} build flash monitor`, 'the build command'));
  if (!ground) {
    const msg = UI.status( { class: 'hint inst-msg', role: 'status' });
    const dl = (what, text) => UI.button( { class: 'btn', type: 'button', text, onclick: () => instDownload(what, t, msg) });
    kids.push(instPara('The design, sent with <code>runner/pi/fly.py</code> (<code>pip install pyserial</code>) from a computer or the Pi:'),
      el('div', { class: 'inst-row' }, dl('airframe', `${designBase()}.dfa: the airframe`), ...(edited.length ? [dl('program', `${designBase()}.rnp: the edited formulas`)] : [])), msg,
      cmdBox(`${hardwareSettings(boardWiringPlan(t)).map(line=>'python3 runner/pi/fly.py '+port+' set '+line).join('\n')}\npython3 runner/pi/fly.py ${port} save\npython3 runner/pi/fly.py ${port} reboot\npython3 runner/pi/fly.py ${port} airframe ${designBase()}.dfa${edited.length ? `\npython3 runner/pi/fly.py ${port} program ${designBase()}.rnp` : ''}\npython3 runner/pi/fly.py ${port} status`, 'the fly.py commands'),
      instPara('<code>.dfa</code>: the airframe the flight core flies on (each motor\'s force and torque, the servos, the mass and inertia, where the IMU sits), checked by the board and kept in its flash. <code>.rnp</code>: a program, the formulas compiled into the steps the board runs, with self-tests; it lasts until a restart. <code>fly.py PORT test 1 0.1</code> spins motor 1 at 10% for 2 s, props off.', 'hint'));
  } else kids.push(instPara('Its settings can also be typed in any serial terminal at 115200 baud (<code>screen PORT 115200</code>, the Arduino serial monitor).', 'hint'));
  return UI.details({ class: 'inst-manual', title: 'What the files are, and doing it by hand' }, ...kids);
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
  const plan=boardWiringPlan(INST.target);
  if(plan.errors.length) { instMsg(msg,plan.errors.join('; '),'bad');return; }
  const custom=Object.values(plan.sensors).some(s=>s.driver==='custom');
  if(custom && !INST.customFirmware) { instMsg(msg,'Custom C drivers require rebuilt firmware. Flash your custom .bin files first, then send these settings.','bad');return; }
  const lines=hardwareSettings(plan);
  for (const l of lines) {
    const r = await espSend(dfFrame(LK.SETTING, l), s => /^set |can't|expected|not a list|at most|unknown|disarm first|GPIO|reserved|invalid|I2C|compass|sensor|servo_|esc_|radio|bind|phrase|firmware/.test(s), 2000, msg, 'Sending ' + l);   // (radio=, bind=: the board's answer, or why it can't)
    if (r === null) { if (!/Couldn|Not conn/.test(msg.textContent)) instMsg(msg, 'No answer from the board.', 'bad'); return; }
    if (!/^set /.test(r)) { instMsg(msg, 'The board refused ' + l + ': ' + r, 'bad'); return; }
  }
  const sv = await espSend(dfFrame(LK.SETTING, 'save'), s => /saved|couldn't save|disarm first/.test(s), 2000, msg, 'Saving');
  if (!sv || !/saved/.test(sv)) { instMsg(msg, 'It didn\'t save: ' + (sv || 'no answer'), 'bad'); return; }
  const reboot = await espSend(dfFrame(LK.SETTING, 'reboot'), s => /rebooting|disarm first/.test(s), 2000, msg, 'Restarting it');
  if (reboot !== 'rebooting') { instMsg(msg, 'Wiring saved, but restart was not confirmed: ' + (reboot || 'no answer'), 'bad'); return; }
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
async function firmwareParts(name, profile) {
  if (INST.files && INST.files.length) {
    const want = [[profile.boot, /bootloader/i], [0x8000, /partition/i], [0x10000, name === 'ground' ? /dfb_ground/i : /dfb_flight/i]];
    const parts = [];
    for (const [addr, re] of want) {
      const f = INST.files.find(x => re.test(x.name)); if (!f) throw new Error(`pick bootloader.bin, partition-table.bin and ${name === 'ground' ? 'dfb_ground.bin' : 'dfb_flight.bin'} together`);
      const data = new Uint8Array(await f.arrayBuffer());
      if (addr !== 0x8000) checkFirmwareImage(data, profile, f.name);
      parts.push({ address: addr, data, name: f.name });
    }
    return { parts, about: 'files from this computer' };
  }
  let man;
  try { const r = await fetch(FW_DIR + 'manifest.json', { cache: 'no-cache' }); if (!r.ok) throw new Error(r.status); man = await r.json(); }
  catch (e) { throw new Error(location.protocol === 'file:' ? 'the page is opened as a file, so it can\'t read its firmware folder: open it from a web address (python3 -m http.server in its folder, then http://localhost:8000), or use “firmware files from this computer”' : 'couldn\'t load firmware/manifest.json'); }
  const fw = man.targets && man.targets[profile.chip] && man.targets[profile.chip].firmware[name], parts = [];
  if (!fw) throw new Error('No firmware bundle for ' + profile.label + ': rebuild tools/build_firmware.sh');
  for (const [addr, f] of fw.parts) {
    const r = await fetch(FW_DIR + fw.dir + '/' + f, { cache: 'no-cache' }); if (!r.ok) throw new Error('couldn\'t load ' + f);
    const data = new Uint8Array(await r.arrayBuffer()), want = fw.files[f];
    if (want && (data.length !== want.size || await sha256hex(data) !== want.sha256)) throw new Error(f + ' arrived damaged (its checksum doesn\'t match): reload the page and try again');
    if (addr !== 0x8000) checkFirmwareImage(data, profile, f);
    parts.push({ address: addr, data, name: f });
  }
  return { parts, about: `built ${man.built} from ${man.commit}` };
}
async function espFlash(name, profile, prog, msg) {
  if (INST.busy) return;
  const erase = $('#instErase') && $('#instErase').checked;
  let fw; try { instMsg(msg, 'Loading the firmware…'); fw = await firmwareParts(name, profile); } catch (e) { instMsg(msg, 'Can\'t install: ' + e.message + '.', 'bad'); return; }
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
    if (chip !== profile.label) throw new Error(`this is an ${chip}; you selected ${profile.label}, so nothing was written`);
    say(`${chip} found. ${erase ? 'Erasing it, then writing' : 'Writing'} the firmware (${fw.about})…`);
    const total = fw.parts.reduce((a, p) => a + p.data.length, 0), done = fw.parts.map(() => 0);
    await loader.writeFlash({
      fileArray: fw.parts.map(p => ({ data: p.data, address: p.address })), flashMode: 'keep', flashFreq: 'keep', flashSize: 'keep',
      eraseAll: !!erase, compress: true, calculateMD5Hash: img => md5hex(img instanceof Uint8Array ? img : Uint8Array.from(img, c => c.charCodeAt(0))),
      reportProgress: (i, written, size) => { done[i] = written / size * fw.parts[i].data.length; prog.value = done.reduce((a, b) => a + b, 0) / total; },
    });
    prog.value = 1;
    INST.customFirmware = !!(INST.files && INST.files.length);
    await loader.after('hard_reset');
    say('Installed and checked: the board restarted on the new firmware.' + (name === 'flight' ? ' Now send it this design (step 2).' : ' Now connect and set up its wiring (step 2).'));
    msg.className = 'ui-status hint inst-msg good';
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
  for (const [k, label, def, hint, owned] of fields) {
    const inp = UI.input( { type: 'text', value: owned?def:instPref(k, def), readonly:owned?'readonly':null, spellcheck: 'false', autocomplete: 'off', 'aria-label': label, title: hint || '' });
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
  const plan=boardWiringPlan(t);
  if(plan.errors.length)return [instStep(1,'Fix hardware connections',instPara('Change the connections in Hardware wiring before installing.'),...plan.errors.map(text=>el('p',{class:'bad',text})))];
  const tasks = t.tasks, learnOrSuper = tasks.includes('learn') || tasks.includes('super');
  const gpsSensor=sensorsOf('fix').find(c=>wiredTo(c)===t);
  const fix = sensorsOf('fix').some(c => wiredTo(c) === t) && tasks.includes('nav'), nL = latches().length, cargo = tasks.includes('cargo') && nL > 0, radio = tasks.includes('tlm');
  const core = boardOf('core'), coreProfile = core && ESP_PROFILES[core.kind], linkPins = coreProfile ? coreProfile.link : [1,3], bt = t.kind !== 'pizero';
  const fields = [['host', 'Pi (user@address)', 'pi@raspberrypi.local', 'What you type after ssh']];
  if (fix) fields.push(['gps', 'GPS port', gpsSensor && partWiring(gpsSensor).port || '/dev/ttyUSB0', 'Change this saved connection in Hardware wiring',true]);
  fields.push(['link', 'Link to the ESP32', hardwareBus(computers(),t).linkPort||'/dev/serial0', 'Change the serial connection in Hardware wiring',true]);
  if (cargo) fields.push(['latch', 'Latch outputs', piLatchSettings(computers(),cfg.comps,t), 'Saved GPIO/PWM connections from Hardware wiring',true]);
  const rk = radioCfg.kind, rlines = radioSettingLines(radioCfg);   // the link (Ground tab): a receiver on a serial port, or the Pi's own Wi-Fi
  if (radio && (rk === 'elrs' || rk === 'serial')) fields.push(['crsf', rk === 'serial' ? 'Serial line port' : 'Receiver port', hardwareBus(computers(),t).receiverPort || '/dev/ttyUSB1', 'Change this saved connection in Hardware wiring',true]);
  const need = ['nav', ...(learnOrSuper ? ['airframe', 'pi'] : [])];
  const cmds = {};
  const out = [];
  const notes = [];
  if (!tasks.includes('nav')) notes.push('dfb_pi always runs the navigation (it needs the .dnc): with this board\'s navigation off in the simulator, the real Pi still runs it.');
  if (editedFor(tasks).length) notes.push(`Your edits to ${editedFor(tasks).join(', ')} aren't loaded by dfb_pi yet: it runs its built-in formulas.`);
  if (tasks.includes('cargo') && !nL) notes.push('This design has no latches, so the cargo task has nothing to drive.');
  if (radio && rk === 'espnow') notes.push('ESP-NOW needs an ESP32: a Pi can\'t be its drone end (dfb_pi refuses it). Put the Telemetry & radio task on the ESP32, or pick Wi-Fi in the Ground tab: the Pi\'s own Wi-Fi works.');
  if (radio && rk === 'wifi') notes.push(radioCfg.sta ? 'Wi-Fi, the Pi joining a network: set its network up as usual (nmcli device wifi connect …); the command module joins the same one and talks to this Pi\'s address.' : `Wi-Fi, the Pi making the network: set up a hotspot on channel ${radioCfg.channel} (hostapd, or NetworkManager: nmcli device wifi hotspot ssid liftlab password … channel ${radioCfg.channel} band bg); the command module joins it.`);
  out.push(instPara(`The Pi runs <code>dfb_pi</code>: ${tasks.map(x => TASKS[x].label.toLowerCase()).join(', ') || 'the navigation'}, the same C as this simulator's ${t.name}. It's built on the Pi from the source, and this design's files go in beside it. You need Raspberry Pi OS (Lite is fine) with its network and SSH working: everything below is typed into <code>ssh ${'<span class="inst-host"></span>'}</code>, except where it says your computer.`));
  for (const n of notes) out.push(el('p', { class: 'inst-note', text: n }));
  const inputs = piInputs(fields, () => refresh());
  out.push(el('div', { class: 'inst-safety' }, el('b', { text: 'Your Pi' }), inputs));
  const v = k => {const f=fields.find(f=>f[0]===k)||[];return f[4]?f[2]:instPref(k,f[2]||'');};
  // The saved connection determines the wiring and which Pi interfaces need enabling.
  const gpioLink=v('link')==='/dev/serial0',gpioSerial=gpioLink || (fix && v('gps')==='/dev/serial0') || (radio && v('crsf')==='/dev/serial0');
  const pwmCargo=cargo && /pwm/.test(v('latch'));
  out.push(instStep(1, 'Wire it to the flight controller',
    gpioLink ? instPara(core ? `To <b>${core.name}</b> (${BOARD_KINDS[core.kind].label}), on its UART0 pins. Both sides are 3.3 V: no level shifter.` : 'To the flight controller’s UART0 pins.') : instPara(`Connect the flight controller to the Pi with a USB serial cable. The saved link is <code>${v('link')}</code>; this connection uses no Pi GPIO.`),
    ...(gpioLink ? [el('table', { class: 'inst-files' }, el('tbody', {},
      el('tr', {}, el('td', { text: 'Pi pin 8 (GPIO 14, TX)' }), el('td', { text: '→' }), el('td', { text: `Flight controller GPIO ${linkPins[1]} (RX0)` })),
      el('tr', {}, el('td', { text: 'Pi pin 10 (GPIO 15, RX)' }), el('td', { text: '←' }), el('td', { text: `Flight controller GPIO ${linkPins[0]} (TX0)` })),
      el('tr', {}, el('td', { text: 'Pi pin 6 (GND)' }), el('td', { text: '—' }), el('td', { text: 'ESP32 GND' })))),
      instPara('These are also connected to the ESP32’s USB serial adapter: unplug that USB cable while the Pi is wired here. A USB serial connection instead can be selected in Hardware wiring.', 'hint')] : [])));
  cmds.serial = cmdBox('', 'the interface setup commands');
  out.push(instStep(2, 'Prepare its interfaces', instPara(gpioSerial ? 'Free the Pi GPIO UART from the login console and Bluetooth, grant GPIO/serial access, then restart.' : 'Grant GPIO/USB serial access, then restart. The saved USB connections do not need the Pi GPIO UART.'), cmds.serial));
  cmds.copy = cmdBox('', 'the copy command'); cmds.clone = cmdBox('', 'the clone command');
  out.push(instStep(3, 'Copy the source code to it', instPara('<b>On your computer</b>, in this repository\'s folder (where <code>index.html</code> is):'), cmds.copy,
    instPara('Or, if the Pi can reach the repository itself (a private one needs your GitHub login on the Pi):', 'hint'), cmds.clone));
  cmds.build = cmdBox('', 'the build commands');
  out.push(instStep(4, 'Build it', instPara('The compiler first (it may already be there), then dfb_pi: a single compile, about a minute on a Pi Zero. It ends with <code>built dfb_pi</code>.'), cmds.build));
  cmds.files = cmdBox('', 'the design files block');
  const dmsg = UI.status( { class: 'hint inst-msg', role: 'status' });
  const dl = el('div', { class: 'inst-row' }, ...need.map(w => UI.button( { class: 'btn', type: 'button', text: `${designBase()}${w === 'nav' ? '.dnc' : w === 'airframe' ? '.dfa' : '.dlc'}`, title: 'Download it', onclick: () => instDownload(w, t, dmsg) })));
  out.push(instStep(5, 'Put this design on it', instPara(`One block with the files inside it: paste it into the Pi's terminal. It writes them to <code>~/dfb</code> and checks each arrived whole (<code>OK</code>). ${need.length > 1 ? '<code>drone.dnc</code>: where its sensors sit, for the navigation; <code>drone.dfa</code>: the airframe, as the flight controller has it, for the learning and the supervisor to start from; <code>drone.dlc</code>: where the IMU sits, each motor\'s heat model and the battery.' : '<code>drone.dnc</code>: the mass and where its sensors sit, for the navigation.'}`), cmds.files,
    instPara('Do this again whenever the design changes. Or download them and copy them across with <code>scp</code>, renamed to <code>drone.dnc</code>…:', 'hint'), dl, dmsg));
  cmds.run = cmdBox('', 'the run command');
  out.push(instStep(6, 'Try it', instPara('With the flight controller on (props off). It says what it found (the link, the GPS) and takes commands as you type: <code>status</code>, <code>health</code>, <code>learning</code>. Ctrl+C stops it.'), cmds.run));
  cmds.svc = cmdBox('', 'the service commands');
  out.push(instStep(7, 'Start it at power-up', instPara('As a service, it starts when the Pi boots and again if it ever stops. Its commands then come over UDP (port 14560) instead of the keyboard.'), cmds.svc,
    instPara('Its output: <code>journalctl -u dfb -f</code>. A command: <code>echo status | nc -u -w1 127.0.0.1 14560</code>. After a new design (step 5): <code>sudo systemctl restart dfb</code>. After new code (steps 3 and 4): the same.', 'hint')));
  out.push(UI.details({ class: 'inst-manual', title: 'Good to know' },
    instPara('<b>Power.</b> Cutting the drone\'s battery cuts the Pi without warning, which can damage what it was writing on the SD card. Its files are written once and only read in flight, so the risk is small; for more, turn on the read-only overlay (<code>sudo raspi-config</code> → Performance → Overlay file system) once it\'s all set up, and off again to change the design.'),
    instPara('<b>Boot time.</b> A Pi Zero takes 20–30 s to start. The ESP32 flies without it meanwhile (angle mode), and the navigation, learning and supervisor join in when it\'s up.'),
    instPara('<b>Where things are.</b> <code>~/dfb/runner</code>: the source and <code>runner/pi/dfb_pi</code>; <code>~/dfb/drone.*</code>: the design; <code>/etc/systemd/system/dfb.service</code>: the service. <code>runner/pi/fly.py</code> talks to the ESP32 from the Pi too (stop the service first: one program on the port at a time).')));
  const refresh = async () => {
    const host = v('host') || 'pi@raspberrypi.local';
    out[0].querySelectorAll('.inst-host').forEach(n => { n.textContent = host; });
    cmds.serial.code.textContent = [
      ...(gpioSerial ? [
        'sudo raspi-config nonint do_serial_cons 1    # no login console on it',
        'sudo raspi-config nonint do_serial_hw 0      # the port itself on',
        'CFG=/boot/firmware/config.txt; [ -f $CFG ] || CFG=/boot/config.txt',
        'grep -q "^dtoverlay=disable-bt" $CFG || echo "dtoverlay=disable-bt" | sudo tee -a $CFG    # Bluetooth off the port' + (bt ? '' : ' (nothing to do on a Zero without W)'),
        'sudo systemctl disable hciuart 2>/dev/null; true'] : []),
      ...(pwmCargo ? [
        ...(!gpioSerial ? ['CFG=/boot/firmware/config.txt; [ -f $CFG ] || CFG=/boot/config.txt'] : []),
        'grep -q "^dtoverlay=pwm-2chan" $CFG || echo "dtoverlay=pwm-2chan" | sudo tee -a $CFG    # hardware PWM for the latches (GPIO 18, 19)'] : []),
      'sudo usermod -aG dialout,gpio $USER',
      'sudo reboot'].join('\n');
    cmds.copy.code.textContent = `ssh ${host} "mkdir -p ~/dfb"\nscp -r runner ${host}:~/dfb/`;
    cmds.clone.code.textContent = `git clone --depth 1 https://github.com/mrtksn/liftlab.git ~/dfb`;
    cmds.build.code.textContent = 'sudo apt update && sudo apt install -y build-essential\ncd ~/dfb && sh runner/pi/build.sh';
    let files = []; try { files = need.map(w => instFile(w, t)); } catch (e) { cmds.files.code.textContent = '# This design can\'t be exported: ' + e.message; }
    if (files.length) cmds.files.code.textContent = await pastePack(files, 'sudo systemctl restart dfb 2>/dev/null; true');
    const args = ['--nav drone.dnc', ...(learnOrSuper ? ['--airframe drone.dfa --pi drone.dlc'] : []), ...(v('link') && v('link') !== '/dev/serial0' ? ['--link ' + v('link')] : []),
      ...(!tasks.includes('learn') && learnOrSuper ? ['--no-learning'] : []), ...(!tasks.includes('super') && learnOrSuper ? ['--no-supervisor'] : []),
      ...(fix ? ['--gps ' + v('gps')] : []), ...(cargo ? ['--latch ' + v('latch')] : []), ...(radio && rk === 'elrs' ? ['--crsf ' + v('crsf') + ' --radio ' + rlines[0].slice(6)] : []), ...(radio && rk === 'wifi' ? [`--radio ${rlines[0].slice(6)} --bind ${shq(radioCfg.bind)}`] : []), ...(radio && rk === 'serial' ? [`--radio ${rlines[0].slice(6)} --radio-dev ${v('crsf')} --bind ${shq(radioCfg.bind)}`] : []), ...(radio && rk === 'nrf24' ? [`--radio ${rlines[0].slice(6)} --nrf-spi ${hardwareBus(computers(),t).nrfSpi || '/dev/spidev0.0'} --nrf-ce ${hardwareBus(computers(),t).nrfCe ?? 25} --bind ${shq(radioCfg.bind)}`] : [])].join(' ');
    cmds.run.code.textContent = `cd ~/dfb && ./runner/pi/dfb_pi ${args}`;
    cmds.svc.code.textContent = `sudo tee /etc/systemd/system/dfb.service > /dev/null <<EOF\n[Unit]\nDescription=LiftLab: dfb_pi\nAfter=network.target\n\n[Service]\nUser=$USER\nWorkingDirectory=$HOME/dfb\nExecStart=$HOME/dfb/runner/pi/dfb_pi ${args}\nRestart=always\nRestartSec=2\n\n[Install]\nWantedBy=multi-user.target\nEOF\nsudo systemctl daemon-reload\nsudo systemctl enable --now dfb`;
  };
  refresh();
  return out;
}
// The command module on a Pi or a Mac: dfb_ground, built from the source.
function groundPiGuide(K) {
  const desktop = K.groundOnly, macOS = /Mac/.test(navigator.platform || ''), edited = editedFor(['ground']), rk = radioCfg.kind;
  const fields = desktop ? [] : [['host', 'Pi (user@address)', 'pi@raspberrypi.local', 'What you type after ssh']];
  if (rk === 'nrf24') fields.push(['nrfspi', 'nRF24L01 SPI device', '/dev/spidev0.0', 'SPI on (raspi-config); CSN on CE0 (pin 24)'], ['nrfce', 'nRF24L01 CE GPIO', '25', 'Any free GPIO (25: pin 22)']);
  else if (rk === 'wifi') fields.push(['drone', 'The drone\'s address', radioCfg.sta ? 'liftlab-drone.local' : '192.168.4.1', 'Its Pi or ESP32 on the Wi-Fi network']);
  else fields.push(['tx', rk === 'espnow' ? 'ESP-NOW bridge port' : rk === 'serial' ? 'Serial line port' : 'Transmitter module port', desktop && macOS ? '/dev/cu.usbserial-0001' : '/dev/ttyUSB0', rk === 'espnow' ? 'The USB serial port of the ESP32 that is its ESP-NOW radio' : rk === 'serial' ? 'The serial port the line is on (a USB serial adapter wired to the laser, fibre or modem)' : 'The ExpressLRS transmitter module\'s USB serial port']);
  const v = k => instPref(k, (fields.find(f => f[0] === k) || [])[2] || '');
  const out = [], cmds = { copy: cmdBox('', 'the copy command'), build: cmdBox('', 'the build commands'), run: cmdBox('', 'the run command') };
  out.push(instPara(`It runs <code>dfb_ground</code>, the command module's C (the same this simulator runs on the ground), ${rk === 'nrf24' ? (desktop ? 'with an nRF24L01 on SPI, which only a Pi (or another Linux board with SPI) has: a Mac or PC can\'t drive one, so use an ESP32 command module, or this ESP32 on USB as its radio' : 'with an nRF24L01 on the Pi\'s SPI (the same data rate and binding phrase as the drone)') : rk === 'wifi' ? 'on this computer\'s Wi-Fi, in UDP to the drone (the same binding phrase at both ends)' : rk === 'espnow' ? 'with an ESP32 on USB serial as its ESP-NOW radio' : rk === 'serial' ? `on a serial line to the drone (${radioSettingLines(radioCfg)[0].slice(6)}: a laser, fibre, infrared, a radio modem or a wire on a serial port; the same binding phrase at both ends)` : 'wired to the ExpressLRS transmitter module over USB serial'}, with the terminal's keys, a gamepad or your own code over UDP.${desktop ? ' Everything below is typed into a terminal on this computer.' : ''}`));
  out.push(el('div', { class: 'inst-safety' }, el('b', { text: desktop ? 'This computer' : 'Your Pi' }), piInputs(fields, () => refresh())));
  if (!desktop) out.push(instStep(1, 'Copy the source code to it', instPara('<b>On your computer</b>, in this repository\'s folder:'), cmds.copy));
  out.push(instStep(desktop ? 1 : 2, 'Build it', instPara(desktop ? `In this repository's folder. ${macOS ? 'Install Xcode command line tools with <code>xcode-select --install</code> if needed.' : 'On Linux, install the C compiler with your package manager (Debian/Ubuntu: <code>sudo apt install build-essential</code>).'}` : 'The compiler first, then dfb_ground:'), cmds.build));
  const steps = [cmds.run, instPara('The keys: W/S climb and sink, A/D turn, the arrows move, Space holds, H flies home, R arms, T takes off and lands. <code>--joystick /dev/input/js0</code> reads a gamepad on Linux; UDP port 14561 takes text commands from scripts.', 'hint')];
  if (edited.length) {
    cmds.prog = cmdBox('', 'the program block');
    steps.push(instPara(`You edited ${edited.join(', ')}: put the program next to it (paste this${desktop ? ' in the repository\'s folder' : ''}), and it's in the command above.`), cmds.prog);
  }
  out.push(instStep(desktop ? 2 : 3, 'Run it', ...steps));
  const refresh = async () => {
    const host = v('host') || 'pi@raspberrypi.local';
    cmds.copy.code.textContent = `ssh ${host} "mkdir -p ~/dfb"\nscp -r runner ${host}:~/dfb/`;
    cmds.build.code.textContent = desktop ? 'sh runner/ground/build.sh' : 'sudo apt update && sudo apt install -y build-essential\ncd ~/dfb && sh runner/ground/build.sh';
    const link = rk === 'nrf24' ? `--radio ${radioSettingLines(radioCfg)[0].slice(6)} --nrf-spi ${v('nrfspi')} --nrf-ce ${v('nrfce')} --bind ${shq(radioCfg.bind)}` : rk === 'wifi' ? `--radio wifi --drone ${v('drone')} --bind ${shq(radioCfg.bind)}` : rk === 'espnow' ? `--radio espnow --tx ${v('tx')}` : rk === 'serial' ? `--radio ${radioSettingLines(radioCfg)[0].slice(6)} --tx ${v('tx')} --bind ${shq(radioCfg.bind)}` : `--tx ${v('tx')}`;
    cmds.run.code.textContent = `${desktop ? '' : 'cd ~/dfb && '}./runner/ground/dfb_ground ${link} --keys${edited.length ? ' --program command-module.rnp' : ''}`;
    if (cmds.prog) { try { const f = instFile('program', 'ground'); cmds.prog.code.textContent = (await pastePack([f])).replace('mkdir -p ~/dfb && cd ~/dfb', desktop ? '# (in the repository\'s folder)' : 'mkdir -p ~/dfb && cd ~/dfb'); } catch (e) { cmds.prog.code.textContent = '# The formulas don\'t compile: ' + e.message; } }
  };
  refresh();
  return out;
}

$('#installClose').addEventListener('click', closeInstall);
$('#installDlg').addEventListener('cancel', e => { e.preventDefault(); closeInstall(); });
if ('serial' in navigator) navigator.serial.addEventListener('disconnect', e => {
  if (INST.port && e.target === INST.port) { if (INST.conn) { INST.conn.close(); INST.conn = null; } INST.port = null; instConnUi(); instLog('the board was unplugged', 'bad'); }
});
