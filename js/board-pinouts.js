'use strict';
// Board models: which pins a board brings out, in the order they sit on its headers. The chip (BOARD_KINDS, firmware)
// decides what a GPIO can do; the model says where it is on the board you hold. Built-in models cover common ESP32
// DevKits and the Pi's 40-pin header; a design can edit any of them or add its own (C.wiring.models, saved with the
// design), and each board picks one (C.wiring.boards[id].model).
//
// A model: { label, kinds: chip kinds it fits, layout: 'sides' (two rows on the board's long edges, a DevKit) or
// 'header' (one 2-row header, a Pi: odd pins in the first column; pin 1 drawn square, as its pad is), top: what the
// top of the drawing is, hold: how to hold the board to match the drawing, rows: the two columns' names, cols: two
// columns, top to bottom, each pin a line: a label, '=GPIO' for a GPIO (D15=15, RX2=16), or a supply as it's
// printed (3V3, 5V, VIN, GND, EN) }.
const BUILTIN_BOARD_MODELS = {
  'esp32-devkit30': { label: 'ESP32 DevKit V1 (30 pins)', kinds: ['esp32'], layout: 'sides', top: 'USB end',
    hold: 'Components facing you (the ESP32 module\'s metal can and the chips on top), the USB connector at the top.',
    cols: [['3V3','GND','D15=15','D2=2','D4=4','RX2=16','TX2=17','D5=5','D18=18','D19=19','D21=21','RX0=3','TX0=1','D22=22','D23=23'],
           ['VIN','GND','D13=13','D12=12','D14=14','D27=27','D26=26','D25=25','D33=33','D32=32','D35=35','D34=34','VN=39','VP=36','EN']] },
  'esp32-devkitc38': { label: 'ESP32-DevKitC (38 pins)', kinds: ['esp32'], layout: 'sides', top: 'USB end',
    hold: 'Components facing you (the ESP32 module\'s metal can on top), the USB connector at the top.',
    cols: [['CLK=6','SD0=7','SD1=8','15=15','2=2','0=0','4=4','16=16','17=17','5=5','18=18','19=19','GND','21=21','RX=3','TX=1','22=22','23=23','GND'],
           ['5V','CMD=11','SD3=10','SD2=9','13=13','GND','12=12','14=14','27=27','26=26','25=25','33=33','32=32','35=35','34=34','SVN=39','SVP=36','EN','3V3']] },
  'esp32s3-devkitc1': { label: 'ESP32-S3-DevKitC-1 (44 pins)', kinds: ['s3'], layout: 'sides', top: 'antenna end (USB at the bottom)',
    hold: 'Components facing you, the module\'s antenna end at the top and the two USB connectors at the bottom.',
    cols: [['3V3','3V3','RST','4=4','5=5','6=6','7=7','15=15','16=16','17=17','18=18','8=8','3=3','46=46','9=9','10=10','11=11','12=12','13=13','14=14','5V','GND'],
           ['GND','TX=43','RX=44','1=1','2=2','42=42','41=41','40=40','39=39','38=38','37=37','36=36','35=35','0=0','45=45','48=48','47=47','21=21','20=20','19=19','GND','GND']] },
  'esp32c3-supermini': { label: 'ESP32-C3 SuperMini (16 pins)', kinds: ['c3'], layout: 'sides', top: 'USB end',
    hold: 'Components facing you (the chip and the BOOT/RESET buttons on top), the USB-C connector at the top.',
    cols: [['5V','GND','3V3','4=4','3=3','2=2','1=1','0=0'], ['5=5','6=6','7=7','8=8','9=9','10=10','RX=20','TX=21']] },
  'pi-header40': { label: 'Raspberry Pi 40-pin header', kinds: ['pizero','pizero2','pi4'], layout: 'header', top: 'pin 1 end: the microSD card end on a Zero',
    hold: 'Components facing you, the header on the right, the microSD card end up (a Pi 4: the end away from the USB ports). Pin 1 is the square pad; odd pins are on the inner row, even pins on the row along the board\'s edge.',
    rows: ['odd · inner row', 'even · edge row'],
    cols: [['3V3','SDA=2','SCL=3','GPIO4=4','GND','GPIO17=17','GPIO27=27','GPIO22=22','3V3','MOSI=10','MISO=9','SCLK=11','GND','ID_SD=0','GPIO5=5','GPIO6=6','GPIO13=13','GPIO19=19','GPIO26=26','GND'],
           ['5V','5V','GND','TXD=14','RXD=15','GPIO18=18','GND','GPIO23=23','GPIO24=24','GND','GPIO25=25','CE0=8','CE1=7','ID_SC=1','GND','GPIO12=12','GND','GPIO16=16','GPIO20=20','GPIO21=21']] },
};
const BOARD_MODEL_MAX_PINS = 60;
function parsePinLine(s) {
  s = String(s).trim(); const i = s.lastIndexOf('='), label = (i >= 0 ? s.slice(0, i) : s).trim(), g = i >= 0 ? s.slice(i + 1).trim() : '';
  const gpio = g === '' ? null : /^\d{1,2}$/.test(g) ? Number(g) : NaN;
  const up = label.toUpperCase().replace(/\s/g, '');
  const fn = gpio !== null ? 'gpio' : /^(3V3|3\.3V?|3V)$/.test(up) ? '3v3' : /^(5V|VIN|VBUS|VCC5)$/.test(up) ? '5v' : /^(GND|G|VSS)$/.test(up) ? 'gnd' : 'other';
  return { label: label || (gpio !== null ? String(gpio) : '?'), gpio, fn };
}
// The models a design can use: built-ins with its edits on top, and its own.
function boardModels(C) { return { ...BUILTIN_BOARD_MODELS, ...((C && C.wiring && C.wiring.models) || {}) }; }
const modelsForKind = (C, kind) => Object.entries(boardModels(C)).filter(([, m]) => Array.isArray(m.kinds) && m.kinds.includes(kind));
function boardModelId(C, b) {
  if (!b) return null;
  const want = C && C.wiring && C.wiring.boards && C.wiring.boards[b.id] && C.wiring.boards[b.id].model, M = boardModels(C);
  if (want && M[want] && M[want].kinds.includes(b.kind)) return want;
  const first = modelsForKind(C, b.kind)[0]; return first ? first[0] : null;
}
// The board's model with its pins parsed: { id, label, layout, top, cols: [[{label, gpio, fn, num}]] } or null.
function boardModel(C, b) {
  const id = boardModelId(C, b); if (!id) return null; const m = boardModels(C)[id];
  const cols = (m.cols || [[], []]).slice(0, 2).map((col, c) => col.map((s, i) => ({ ...parsePinLine(s), num: m.layout === 'header' ? 2 * i + 1 + c : null, col: c, row: i })));
  return { id, label: m.label, layout: m.layout === 'header' ? 'header' : 'sides', top: m.top || '', hold: m.hold || '', rows: Array.isArray(m.rows) ? m.rows : m.layout === 'header' ? ['odd pins', 'even pins'] : ['left row', 'right row'], cols, builtin: !!BUILTIN_BOARD_MODELS[id], edited: !!(BUILTIN_BOARD_MODELS[id] && C.wiring && C.wiring.models && C.wiring.models[id]) };
}
const modelPins = M => M ? [...M.cols[0], ...M.cols[1]] : [];
// A model's problems: a GPIO listed twice, a bad line.
function boardModelErrors(m) {
  const e = [], seen = new Set();
  if (!m.label || !String(m.label).trim()) e.push('a name');
  const cols = m.cols || [];
  if (cols.length !== 2) e.push('two columns of pins');
  if (cols.flat().length > BOARD_MODEL_MAX_PINS) e.push('at most ' + BOARD_MODEL_MAX_PINS + ' pins');
  for (const s of cols.flat()) { const p = parsePinLine(s); if (Number.isNaN(p.gpio) || (p.gpio !== null && p.gpio > 48)) e.push('“' + s + '”: after = comes a GPIO number (0–48)'); else if (p.gpio !== null) { if (seen.has(p.gpio)) e.push('GPIO ' + p.gpio + ' is listed twice'); seen.add(p.gpio); } }
  return e;
}
// How a GPIO is named on the board ("D15", "pin 10 · RXD"), or null when it isn't on the board's headers.
function boardGpioName(C, b, gpio) {
  const M = boardModel(C, b); if (!M) return 'GPIO ' + gpio;
  const p = modelPins(M).find(p => p.gpio === gpio); if (!p) return null;
  return M.layout === 'header' ? 'pin ' + p.num + ' (GPIO ' + gpio + ')' : p.label + (String(p.label) === String(gpio) ? '' : ' (GPIO ' + gpio + ')');
}
// The same, short for a drawing's labels: D35 rather than D35 (GPIO 35) when the silkscreen already says the number.
function boardGpioShort(C, b, gpio) {
  const M = boardModel(C, b), p = M && modelPins(M).find(p => p.gpio === gpio);
  if (p && M.layout !== 'header' && new RegExp('(^|\\D)' + gpio + '$').test(p.label)) return p.label;
  return boardGpioName(C, b, gpio) || 'GPIO ' + gpio;
}

// What each pin of a board is wired to: Map key (a GPIO number, or '3v3' / '5v' / 'gnd') → [{text, kind}].
function boardPinUses(C, comps, b) {
  const uses = new Map(), add = (k, text, kind) => { if (!uses.has(k)) uses.set(k, []); uses.get(k).push({ text, kind }); };
  const supply = { '3v3': new Set(), '5v': new Set(), gnd: new Set() };
  const sensors = comps.filter(c => c.type === 'sensor' && ['imu', 'baro', 'mag'].includes(c.kind) && hardwareOwner(C, c) === b).map(c => c.name);
  const bus = hardwareBus(C, b), esp = !!ESP_PROFILES[b.kind];
  for (const cl of hardwarePinClaims(C, comps, b)) {
    const k = cl.key, part = /^part(\d+)$/.exec(k), gps = /^gps(Tx|Rx)(\d+)$/.exec(k), nrf = /^nrf(\d)$/.exec(k);
    if (k === 'sda' || k === 'scl') { add(cl.pin, k.toUpperCase() + ' → ' + (sensors.join(', ') || 'I²C'), k); sensors.forEach(n => { supply['3v3'].add(n); supply.gnd.add(n); }); }
    else if (k === 'battery') { add(cl.pin, 'Battery divider midpoint', 'signal'); supply.gnd.add('battery divider'); }
    else if (k === 'rx' || k === 'tx') { const name = radioWiringRow(b, bus, esp, typeof hardwareRadio === 'function' ? hardwareRadio() : null).device; add(cl.pin, k === 'rx' ? 'RX ← ' + name + ' TX' : 'TX → ' + name + ' RX', k === 'rx' ? 'rx' : 'tx'); supply['5v'].add(name); supply.gnd.add(name); }
    else if (nrf) { add(cl.pin, 'nRF24L01 ' + ['SCK', 'MOSI', 'MISO', 'CSN', 'CE'][nrf[1]], 'signal'); supply['3v3'].add('nRF24L01'); supply.gnd.add('nRF24L01'); }
    else if (part) {
      const c = comps.find(c => c.id === +part[1]); if (!c) continue; const p = hardwarePart(C, c, comps);
      add(cl.pin, (c.type === 'motor' ? (p.driver === 'brushed' ? 'MOSFET gate' : 'ESC signal') : c.type === 'joint' ? 'Servo signal' : 'Latch') + ' → ' + c.name, 'pwm');
      supply.gnd.add(c.type === 'motor' ? 'ESC signal ground' : c.name); if (c.type !== 'motor') supply['5v'].add(c.name);
    }
    else if (gps) { const c = comps.find(c => c.id === +gps[2]); add(cl.pin, gps[1] === 'Tx' ? 'TX → ' + c.name + ' RX' : 'RX ← ' + c.name + ' TX', gps[1] === 'Tx' ? 'tx' : 'rx'); supply['3v3'].add(c.name); supply.gnd.add(c.name); }
    else if (k === 'rxTx' || k === 'rxRx') { add(cl.pin, k === 'rxTx' ? 'TX → receiver RX' : 'RX ← receiver TX', k === 'rxTx' ? 'tx' : 'rx'); supply.gnd.add('receiver'); }
  }
  for (const L of boardLinks(C, comps)) linkPinUses(C, L, b, add, supply);   // the companion's serial link, from either end
  for (const [k, set] of Object.entries(supply)) if (set.size) add(k, (k === 'gnd' ? 'GND → ' : k === '5v' ? '5 V → ' : '3.3 V → ') + [...set].join(', '), k === 'gnd' ? 'gnd' : k === '5v' ? 'v5' : 'v33');
  return uses;
}

// A board link's pins on board b: its TX, RX and the shared ground (a Pi's GPIO 14/15, or the flight
// controller's link UART, wired to the Pi or to a USB serial adapter).
function linkPinUses(C, L, b, add, supply) {
  if (L.transport !== 'serial' || !L.pi) return;
  const fl = fcLinkPins(C, L.a); if (!fl) return;
  const gpio = L.port === '/dev/serial0', far = gpio ? L.b.name + ' ' : 'USB adapter ';
  if (b === L.a) { add(fl.tx, 'TX → ' + far + (gpio ? boardGpioShort(C, L.b, 15) : 'RX'), 'tx'); add(fl.rx, 'RX ← ' + far + (gpio ? boardGpioShort(C, L.b, 14) : 'TX'), 'rx'); supply.gnd.add(gpio ? L.b.name : 'USB adapter'); }
  if (b === L.b && gpio) { add(15, 'RX ← ' + L.a.name + ' ' + boardGpioShort(C, L.a, fl.tx), 'rx'); add(14, 'TX → ' + L.a.name + ' ' + boardGpioShort(C, L.a, fl.rx), 'tx'); supply.gnd.add(L.a.name); }
}
// Only the link's pins, for its own drawing.
function linkOnlyUses(C, L, b) {
  const uses = new Map(), add = (k, text, kind) => { if (!uses.has(k)) uses.set(k, []); uses.get(k).push({ text, kind }); }, supply = { gnd: new Set() };
  linkPinUses(C, L, b, add, supply);
  if (supply.gnd.size) add('gnd', 'GND → ' + [...supply.gnd].join(', '), 'gnd');
  return uses;
}
// The link drawn: each end's board with just the link's pins labelled ('' when there are no wires: MQTT).
function boardLinkView(C, L) {
  const ends = [L.a, L.b].filter(b => boardModel(C, b) && linkOnlyUses(C, L, b).size);
  if (!ends.length) return null;
  const box = el('div', { class: 'pd-link' });
  for (const b of ends) box.append(boardPinoutView(C, b, { uses: linkOnlyUses(C, L, b) }));
  if (L.port !== '/dev/serial0') box.append(el('p', { class: 'hint', text: 'The other end is a USB-to-UART adapter plugged into ' + L.b.name + ' (' + L.port + '): its RX, TX and GND go to the pins above.' }));
  return box;
}

// One board drawn with its headers, each pin in use labelled on its own line beside it (no wires to cross).
const PD = { pitch: 21, labelW: 310, sidesW: 118, headerW: 96, top: 52, chars: 44 };
const pdEsc = s => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
function boardPinoutSvg(C, comps, b, opt = {}) {
  const M = boardModel(C, b); if (!M) return '';
  const uses = opt.uses || boardPinUses(C, comps, b), header = M.layout === 'header', bw = header ? PD.headerW : PD.sidesW;
  const rows = Math.max(M.cols[0].length, M.cols[1].length), bx = PD.labelW + 20, H = PD.top + rows * PD.pitch + 44, W = bx + bw + PD.labelW + 20;
  const out = [];
  // a supply is labelled once, at its pin nearest the signals in use (the GND beside the link's TX and RX, say)
  const used = modelPins(M).filter(p => p.fn === 'gpio' && uses.has(p.gpio)), mid = used.length ? used.reduce((a, p) => a + p.row, 0) / used.length : 0;
  const supplyPin = {}; for (const k of ['3v3', '5v', 'gnd']) { const c = modelPins(M).filter(p => p.fn === k); if (c.length) supplyPin[k] = c.reduce((a, p) => Math.abs(p.row - mid) < Math.abs(a.row - mid) ? p : a); }
  out.push('<rect class="pd-board" x="' + bx + '" y="' + (PD.top - 14) + '" width="' + bw + '" height="' + (rows * PD.pitch + 16) + '" rx="8"/>');
  out.push('<rect class="pd-tab" x="' + (bx + bw * 0.25) + '" y="' + (PD.top - 20) + '" width="' + (bw * 0.5) + '" height="10" rx="2"><title>' + pdEsc(M.top || 'top') + '</title></rect>');   // (the end that's up: its connector)
  out.push('<text class="pd-title" x="' + (bx + bw / 2) + '" y="16" text-anchor="middle">' + pdEsc(b.name + ' · ' + M.label) + '</text>');
  out.push('<text class="pd-top" x="' + (bx + bw / 2) + '" y="' + (PD.top - 25) + '" text-anchor="middle">↑ ' + pdEsc(M.top || 'top') + '</text>');
  const by = PD.top + rows * PD.pitch + 16;   // the rows' names under the board
  out.push('<text class="pd-rows" x="' + (bx + (header ? 16 : 10)) + '" y="' + by + '" text-anchor="end">' + pdEsc(M.rows[0]) + ' ↑</text><text class="pd-rows" x="' + (bx + bw - (header ? 16 : 10)) + '" y="' + by + '" text-anchor="start">↑ ' + pdEsc(M.rows[1]) + '</text>');
  M.cols.forEach((col, c) => col.forEach((p, i) => {
    const y = PD.top + i * PD.pitch, x = header ? bx + (c ? bw - 16 : 16) : bx + (c ? bw - 10 : 10), side = c ? 1 : -1;
    const key = p.fn === 'gpio' ? p.gpio : p.fn;
    let list = uses.get(key) || [];
    if (p.fn !== 'gpio' && supplyPin[key] !== p) list = [];
    const on = list.length > 0, inner = header ? (c ? bx + bw - 28 : bx + 28) : x - side * 10;
    const tip = '<title>' + pdEsc(p.label + (p.num ? ' · header pin ' + p.num : '') + (p.num === 1 ? ' (the square pad)' : '') + (p.gpio !== null ? ' · GPIO ' + p.gpio : '')) + '</title>', r = on ? 5 : 3.6;
    out.push(p.num === 1 ? '<rect class="pd-pin pd-pin1' + (on ? ' on' : '') + '" x="' + (x - r - 0.5) + '" y="' + (y - r - 0.5) + '" width="' + (2 * r + 1) + '" height="' + (2 * r + 1) + '">' + tip + '</rect>'
      : '<circle class="pd-pin pd-' + p.fn + (on ? ' on' : '') + '" cx="' + x + '" cy="' + y + '" r="' + r + '">' + tip + '</circle>');
    const silk = header ? String(p.num) : p.label;
    out.push('<text class="pd-silk' + (on ? ' on' : '') + '" x="' + inner + '" y="' + (y + 4) + '" text-anchor="' + (c ? 'end' : 'start') + '">' + pdEsc(silk) + '</text>');
    if (on) {
      const kind = list[0].kind, text = (p.gpio !== null && String(p.gpio) !== p.label ? 'GPIO ' + p.gpio + ': ' : '') + list.map(u => u.text).join('; ');
      const lx = c ? bx + bw + 16 : bx - 16, short = text.length > PD.chars ? text.slice(0, PD.chars - 1) + '…' : text;
      out.push('<line class="pd-lead pd-' + kind + '" x1="' + x + '" y1="' + y + '" x2="' + lx + '" y2="' + y + '"/>');
      out.push('<g class="pd-tag pd-' + kind + '"><title>' + pdEsc(text) + '</title><rect x="' + (c ? lx : lx - PD.labelW + 4) + '" y="' + (y - 9) + '" width="' + (PD.labelW - 4) + '" height="18" rx="4"/><text x="' + (c ? lx + 7 : lx - 7) + '" y="' + (y + 4) + '" text-anchor="' + (c ? 'start' : 'end') + '">' + pdEsc(short) + '</text></g>');
    }
  }));
  return '<svg class="pinout-diagram" viewBox="0 0 ' + W + ' ' + H + '" width="' + W + '" height="' + H + '" role="img" aria-label="' + pdEsc(b.name + ' pinout and wiring') + '">' + out.join('') + '</svg>';
}
const PD_LEGEND = { sda: 'I²C SDA', scl: 'I²C SCL', pwm: 'PWM (ESC, servo)', tx: 'UART out (TX)', rx: 'UART in (RX)', signal: 'Other signal', v33: '3.3 V', v5: '5 V', gnd: 'GND' };
function pinoutLegend() { return el('div', { class: 'pd-legend' }, ...Object.entries(PD_LEGEND).map(([k, t]) => el('span', {}, el('i', { class: 'pd-swatch pd-' + k }), t))); }
function boardPinoutView(C, b, opt = {}) {
  const M = boardModel(C, b), box = el('div', { class: 'pd-scroll' }); box.innerHTML = boardPinoutSvg(C, cfg.comps, b, opt) || '';
  if (M) box.append(el('p', { class: 'pd-hold', text: 'Hold it: ' + (M.hold || M.top + ' at the top, components facing you.') + (M.layout === 'header' ? ' Pin 1 is drawn square.' : '') }));
  return box;
}

// The board's model picker and the pinout editor (in the board's dialog).
function boardModelSection(C, b, editWiringFn) {
  const M = boardModel(C, b), wrap = el('div', { class: 'pd-section' });
  if (!M) { wrap.append(el('p', { class: 'hint', text: 'No pinout for ' + (BOARD_KINDS[b.kind]?.label || b.kind) + ' yet: add a board model below.' })); }
  const choices = modelsForKind(C, b.kind).map(([id, m]) => [id, m.label + (BUILTIN_BOARD_MODELS[id] ? (C.wiring?.models?.[id] ? ' (edited)' : '') : ' (yours)')]);
  const setModel = id => editWiringFn(w => { w.boards[b.id] = { ...(w.boards[b.id] || {}), model: id }; }, b.id + 'model');
  if (choices.length) wrap.append(hardwareField('Board model', hardwareSelect(b.name + ' board model', 'pd-model-' + b.id, choices, M ? M.id : '', setModel)));
  if (M) wrap.append(boardPinoutView(C, b), pinoutLegend(), el('p', { class: 'hint', text: 'Each pin in use is labelled with what it goes to. Boards vary: check the silkscreen, and edit the pinout below if yours differs.' }));
  // the editor: a model's pins as two columns of lines
  const src = M ? boardModels(C)[M.id] : { label: (BOARD_KINDS[b.kind]?.label || b.kind) + ' board', kinds: [b.kind], layout: 'sides', top: 'USB end', cols: [[], []] };
  const draft = { label: src.label, layout: src.layout || 'sides', top: src.top || '', hold: src.hold || '', rows: (src.rows || []).join(', '), cols: [(src.cols?.[0] || []).join('\n'), (src.cols?.[1] || []).join('\n')] };
  const err = el('p', { class: 'bad', hidden: true });
  const save = id => {
    const m = { label: draft.label.trim().slice(0, 48), kinds: id && boardModels(C)[id] ? boardModels(C)[id].kinds : [b.kind], layout: draft.layout, top: draft.top.trim().slice(0, 64), hold: draft.hold.trim().slice(0, 300), ...(draft.rows.trim() ? { rows: draft.rows.split(',').map(s => s.trim().slice(0, 24)).slice(0, 2) } : {}), cols: draft.cols.map(t => t.split('\n').map(s => s.trim()).filter(Boolean)) };
    const e = boardModelErrors(m); if (e.length) { err.hidden = false; err.textContent = 'Not saved: ' + e.join('; '); return; }
    const newId = id || 'custom-' + Date.now().toString(36);
    editWiringFn(w => { w.models = { ...(w.models || {}), [newId]: m }; if (!id) w.boards[b.id] = { ...(w.boards[b.id] || {}), model: newId }; }, b.id + 'pinout');
  };
  const name = UI.input({ type: 'text', value: draft.label, 'aria-label': 'Model name' }); name.addEventListener('input', () => { draft.label = name.value; });
  const top = UI.input({ type: 'text', value: draft.top, 'aria-label': 'Top of the drawing' }); top.addEventListener('input', () => { draft.top = top.value; });
  const hold = UI.input({ type: 'text', value: draft.hold, 'aria-label': 'How to hold it' }); hold.addEventListener('input', () => { draft.hold = hold.value; });
  const rowNames = UI.input({ type: 'text', value: draft.rows, 'aria-label': 'Row names', placeholder: 'odd · inner row, even · edge row' }); rowNames.addEventListener('input', () => { draft.rows = rowNames.value; });
  const layout = hardwareSelect('Layout', 'pd-layout-' + b.id, [['sides', 'Two rows on the long edges (a DevKit)'], ['header', 'One 2-row header, odd pins first (a Pi)']], draft.layout, v => { draft.layout = v; });
  const col = i => { const t = el('textarea', { class: 'pd-col', rows: 16, spellcheck: 'false', 'aria-label': (i ? 'Right' : 'Left') + ' column of pins' }); t.value = draft.cols[i]; t.addEventListener('input', () => { draft.cols[i] = t.value; }); return t; };
  const buttons = el('div', { class: 'pd-buttons' });
  if (M) buttons.append(UI.button({ class: 'btn primary', text: M.builtin ? 'Save my edits to this model' : 'Save the pinout', onclick: () => save(M.id) }));
  buttons.append(UI.button({ class: 'btn', text: M ? 'Save as a new board model' : 'Add this board model', onclick: () => save(null) }));
  if (M && M.edited) buttons.append(UI.button({ class: 'btn', text: 'Back to the built-in pinout', onclick: () => editWiringFn(w => { const x = { ...(w.models || {}) }; delete x[M.id]; w.models = x; }, b.id + 'pinout') }));
  if (M && !M.builtin) buttons.append(UI.button({ class: 'btn', text: 'Delete this model', onclick: () => editWiringFn(w => { const x = { ...(w.models || {}) }; delete x[M.id]; w.models = x; for (const s of Object.values(w.boards || {})) if (s.model === M.id) delete s.model; }, b.id + 'pinout') }));
  wrap.append(hardwareDetails('pinout' + b.id, M ? 'Edit the pinout, or add a board model' : 'Add a board model',
    el('p', { class: 'hint', text: 'One pin per line, top to bottom, as printed on the board: a GPIO as LABEL=GPIO (D15=15, RX2=16, TXD=14), a supply or other pin as its label (3V3, 5V, VIN, GND, EN). Edits are saved with this design. A new model fits ' + (BOARD_KINDS[b.kind]?.label || b.kind) + ' boards: its firmware and GPIO rules stay the chip\'s.' }),
    el('div', { class: 'hw-fields' }, hardwareField('Model name', name), hardwareField('Top of the drawing (the end that\'s up)', top), hardwareField('Layout', layout), hardwareField('Row names (left, right)', rowNames)),
    hardwareField('How to hold it to match the drawing (which side faces you, where pin 1 is)', hold),
    el('div', { class: 'pd-cols' }, hardwareField('Left column', col(0)), hardwareField('Right column', col(1))), err, buttons));
  return wrap;
}
if (typeof module !== 'undefined') module.exports = { BUILTIN_BOARD_MODELS, parsePinLine, boardModel, boardModelErrors, boardPinUses, boardPinoutSvg };
