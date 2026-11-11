'use strict';
// The wiring as a picture: each board drawn with its real header (the pin order on the board you hold), every wire
// from the pin to the device it goes to, coloured by what it carries. Built from the same data as the wiring list
// (hardwarePinClaims, boardLinks), so the two never disagree.
//
// Pinouts are data, chosen per board (C.wiring.boards[id].pinout): a DevKit's two rows of pins as they run from
// the USB end, a Pi's 40-pin header, or 'schematic' (only the pins in use) for a board whose header isn't listed.
// A pin is [gpio, silkscreen label] (gpio null: power or other).
const BOARD_PINOUTS = {
  devkit30: { label: 'ESP32 DevKit V1 · 30 pins', kinds: ['esp32'],
    left: [[null,'3V3'],[null,'GND'],[15,'D15'],[2,'D2'],[4,'D4'],[16,'RX2'],[17,'TX2'],[5,'D5'],[18,'D18'],[19,'D19'],[21,'D21'],[3,'RX0'],[1,'TX0'],[22,'D22'],[23,'D23']],
    right: [[null,'VIN'],[null,'GND'],[13,'D13'],[12,'D12'],[14,'D14'],[27,'D27'],[26,'D26'],[25,'D25'],[33,'D33'],[32,'D32'],[35,'D35'],[34,'D34'],[39,'VN'],[36,'VP'],[null,'EN']] },
  devkitc38: { label: 'ESP32-DevKitC · 38 pins', kinds: ['esp32'],
    left: [[6,'CLK'],[7,'SD0'],[8,'SD1'],[15,'15'],[2,'2'],[0,'0'],[4,'4'],[16,'16'],[17,'17'],[5,'5'],[18,'18'],[19,'19'],[null,'GND'],[21,'21'],[3,'RX'],[1,'TX'],[22,'22'],[23,'23'],[null,'GND']],
    right: [[null,'5V'],[11,'CMD'],[10,'SD3'],[9,'SD2'],[13,'13'],[null,'GND'],[12,'12'],[14,'14'],[27,'27'],[26,'26'],[25,'25'],[33,'33'],[32,'32'],[35,'35'],[34,'34'],[39,'SVN'],[36,'SVP'],[null,'EN'],[null,'3V3']] },
  header40: { label: 'Raspberry Pi 40-pin header', kinds: ['pizero','pizero2','pi4'], header: true,
    // [odd pin, even pin] from pin 1/2; drawn with the even row (the board's edge) towards the flight controller
    rows: [['3V3','5V'],[2,'5V'],[3,'GND'],[4,14],['GND',15],[17,18],[27,'GND'],[22,23],['3V3',24],[10,'GND'],[9,25],[11,8],['GND',7],[0,1],[5,'GND'],[6,12],[13,'GND'],[19,16],[26,20],['GND',21]] },
  schematic: { label: 'Only the pins in use', kinds: null },
};
function boardPinoutKey(C, b) {
  const want = hardwareBus(C, b).pinout, P = BOARD_PINOUTS[want];
  if (P && (!P.kinds || P.kinds.includes(b.kind))) return want;
  return Object.keys(BOARD_PINOUTS).find(k => BOARD_PINOUTS[k].kinds && BOARD_PINOUTS[k].kinds.includes(b.kind)) || 'schematic';
}
const pinoutChoices = b => Object.entries(BOARD_PINOUTS).filter(([, P]) => !P.kinds || P.kinds.includes(b.kind)).map(([k, P]) => [k, P.label]);

// What is wired: devices with terminals, and wires {board, pin, dev, term, kind}; board-to-board wires {a, apin, b, bpin, kind}.
function wiringNetlist(C, comps) {
  const devices = [], wires = [], links = [], devOf = new Map();
  const dev = (key, name, sub, board, power) => { if (!devOf.has(key)) { const d = { key, name, sub, board, terms: [], power }; devOf.set(key, d); devices.push(d); } return devOf.get(key); };
  const term = (d, t) => { if (!d.terms.includes(t)) d.terms.push(t); return t; };
  const wire = (b, pin, d, t, kind) => wires.push({ board: b, pin, dev: d, term: term(d, t), kind });
  const radio = C.boards.find(b => b.tasks.includes('tlm')), sensorsOf = b => comps.filter(c => c.type === 'sensor' && ['imu', 'baro', 'mag'].includes(c.kind) && hardwareOwner(C, c) === b);
  for (const b of C.boards) {
    const bus = hardwareBus(C, b), esp = !!ESP_PROFILES[b.kind];
    for (const cl of hardwarePinClaims(C, comps, b)) {
      const k = cl.key, part = /^part(\d+)$/.exec(k), gps = /^gps(Tx|Rx)(\d+)$/.exec(k), nrf = /^nrf(\d)$/.exec(k);
      if (k === 'sda' || k === 'scl') for (const c of sensorsOf(b)) { const p = hardwarePart(C, c, comps); wire(b, cl.pin, dev('c' + c.id, c.name, DEVICE_PROFILES[c.kind][p.driver]?.label || '', b, '3V3'), k.toUpperCase(), k); }
      else if (k === 'battery') wire(b, cl.pin, dev('batt' + b.id, 'Battery divider', 'midpoint to the pin; ratio ' + bus.batteryDivider, b, null), 'MID', 'signal');
      else if (k === 'rx' || k === 'tx') { const row = radioWiringRow(b, bus, esp, typeof hardwareRadio === 'function' ? hardwareRadio() : null); wire(b, cl.pin, dev('rx' + b.id, row.device, row.note.split(' · ')[0], b, '5V'), k === 'rx' ? 'TX' : 'RX', k === 'rx' ? 'rx' : 'tx'); }
      else if (nrf) wire(b, cl.pin, dev('nrf' + b.id, 'nRF24L01', '3.3 V, 10 µF at the module', b, '3V3'), ['SCK','MOSI','MISO','CSN','CE'][nrf[1]], 'signal');
      else if (gps) { const c = comps.find(c => c.id === +gps[2]); wire(b, cl.pin, dev('c' + c.id, c.name, 'GPS · NMEA serial', b, '3V3'), gps[1] === 'Tx' ? 'RX' : 'TX', gps[1] === 'Tx' ? 'tx' : 'rx'); }
      else if (k === 'rxTx' || k === 'rxRx') wire(b, cl.pin, dev('rx' + b.id, 'Receiver', 'CRSF serial', b, '5V'), k === 'rxTx' ? 'RX' : 'TX', k === 'rxTx' ? 'tx' : 'rx');
      else if (part) {
        const c = comps.find(c => c.id === +part[1]), p = hardwarePart(C, c, comps);
        const sub = c.type === 'motor' ? (p.driver === 'brushed' ? 'MOSFET gate' : 'ESC · ' + escSummary(bus)) : c.type === 'joint' ? 'servo · 50 Hz' : 'latch';
        wire(b, cl.pin, dev('c' + c.id, c.name, sub, b, c.type === 'motor' ? null : '5V'), c.type === 'motor' ? (p.driver === 'brushed' ? 'GATE' : 'SIG') : 'SIG', 'pwm');
      }
    }
  }
  for (const L of boardLinks(C, comps)) {
    if (L.transport !== 'serial' || !L.pi || L.port !== '/dev/serial0') continue;
    const fl = fcLinkPins(C, L.a); if (!fl) continue;
    links.push({ a: L.a, apin: fl.tx, b: L.b, bpin: 15, kind: 'tx', label: L.a.name + ' TX → ' + L.b.name + ' RX' }, { a: L.a, apin: fl.rx, b: L.b, bpin: 14, kind: 'rx', label: L.b.name + ' TX → ' + L.a.name + ' RX' }, { a: L.a, apin: 'GND', b: L.b, bpin: 'GND', kind: 'gnd', label: 'GND ↔ GND' });
  }
  return { devices, wires, links };
}

const WD = { top: 76, pitch: 22, boardW: 150, headerW: 74, devW: 196, gap: 70 };
const WD_KIND = { sda: 'SDA (I²C data)', scl: 'SCL (I²C clock)', pwm: 'PWM signal (ESC, servo)', tx: 'UART: flight controller → other end', rx: 'UART: other end → flight controller', signal: 'Other signal', v33: '3.3 V', v5: '5 V', gnd: 'GND' };
const wdEsc = s => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

// The board's pins with positions: [{gpio, label, x, y, side}] (side: −1 pins face left, +1 right).
function wdBoardPins(C, b, used, x0, y0) {
  const key = boardPinoutKey(C, b), P = BOARD_PINOUTS[key], pins = [], p = WD.pitch;
  if (P.header) {   // the Pi header: even row (the board's edge) on the left, towards the flight controller
    P.rows.forEach(([odd, even], i) => {
      const y = y0 + 34 + i * p, lab = v => typeof v === 'number' ? 'GPIO ' + v : v;
      pins.push({ gpio: typeof even === 'number' ? even : null, label: lab(even), num: 2 * i + 2, x: x0, y, side: -1 });
      pins.push({ gpio: typeof odd === 'number' ? odd : null, label: lab(odd), num: 2 * i + 1, x: x0 + WD.headerW, y, side: 1 });
    });
    return { key, P, pins, w: WD.headerW, h: 34 + P.rows.length * p + 10 };
  }
  if (key === 'schematic') {
    const list = [[null, '3V3'], [null, 'GND'], ...[...used].filter(n => typeof n === 'number').sort((a, b) => a - b).map(n => [n, 'GPIO ' + n])];
    const half = Math.ceil(list.length / 2), cols = [list.slice(0, half), list.slice(half)];
    cols.forEach((col, s) => col.forEach(([g, l], i) => pins.push({ gpio: g, label: l, x: x0 + s * WD.boardW, y: y0 + 34 + i * p, side: s ? 1 : -1 })));
    return { key, P, pins, w: WD.boardW, h: 34 + half * p + 10 };
  }
  P.left.forEach(([g, l], i) => pins.push({ gpio: g, label: l, x: x0, y: y0 + 34 + i * p, side: -1 }));
  P.right.forEach(([g, l], i) => pins.push({ gpio: g, label: l, x: x0 + WD.boardW, y: y0 + 34 + i * p, side: 1 }));
  return { key, P, pins, w: WD.boardW, h: 34 + Math.max(P.left.length, P.right.length) * p + 10 };
}

// The diagram as SVG markup. opt.linkTo: a board id (only its link to the flight controller), opt.power: supply wires.
function wiringDiagramSvg(C, comps, opt = {}) {
  const N = wiringNetlist(C, comps);
  if (opt.linkTo) { N.wires = []; N.devices = []; N.links = N.links.filter(l => l.b.id === opt.linkTo); }   // (a link's own dialog: just its wires)
  const core = C.boards.find(b => b.tasks.includes('core'));
  const piB = C.boards.filter(b => b !== core && N.links.some(l => l.b === b));
  const shown = C.boards.filter(b => (b === core || piB.includes(b) || N.wires.some(w => w.board === b)) && (!opt.linkTo || b === core || b.id === opt.linkTo));
  if (!shown.length) return '';
  // boards left to right: the flight controller, then the boards linked to it, then any other wired board
  shown.sort((a, b) => (a === core ? -1 : b === core ? 1 : piB.includes(a) ? -1 : piB.includes(b) ? 1 : 0));
  const pinPos = new Map(), layout = [];
  let x = WD.devW + WD.gap + 40;
  for (const b of shown) {
    const used = new Set([...N.wires.filter(w => w.board === b).map(w => w.pin), ...N.links.filter(l => l.a === b).map(l => l.apin), ...N.links.filter(l => l.b === b).map(l => l.bpin)]);
    const L = wdBoardPins(C, b, used, x, WD.top); L.b = b; L.x = x; layout.push(L);
    pinPos.set(b.id, L);
    x += L.w + WD.devW + 2 * WD.gap;
  }
  const findPin = (b, pin, near) => {   // a GPIO, or the nearest supply pin of that name
    const L = pinPos.get(b.id); if (!L) return null;
    if (typeof pin === 'number') return L.pins.find(p => p.gpio === pin) || null;
    const c = L.pins.filter(p => p.gpio === null && p.label === pin); if (!c.length) return null;
    return near ? c.reduce((a, p) => Math.abs(p.y - near.y) + (p.side === near.side ? 0 : 400) < Math.abs(a.y - near.y) + (a.side === near.side ? 0 : 400) ? p : a) : c[0];
  };
  // devices: beside their board, on the side most of their pins face, at the height of those pins
  const placed = [];
  for (const d of N.devices) {
    if (!pinPos.has(d.board.id)) continue;
    const ps = N.wires.filter(w => w.dev === d).map(w => findPin(d.board, w.pin)).filter(Boolean); if (!ps.length) continue;
    const L = pinPos.get(d.board.id), side = ps.reduce((s, p) => s + p.side, 0) >= 0 ? 1 : -1;
    const s = side;
    const terms = [...d.terms, ...(opt.power !== false && d.power ? [d.power] : []), ...(opt.power !== false ? ['GND'] : [])];
    const h = 30 + terms.length * 18, want = ps.reduce((a, p) => a + p.y, 0) / ps.length - h / 2;
    placed.push({ d, side: s, L, terms, h, want, x: s < 0 ? L.x - WD.gap - WD.devW : L.x + L.w + WD.gap });
  }
  for (const L of layout) for (const s of [-1, 1]) {   // no overlaps: down the column in the order they want
    let y = 0; for (const p of placed.filter(p => p.L === L && p.side === s).sort((a, b) => a.want - b.want)) { p.y = Math.max(p.want, y, 8); y = p.y + p.h + 12; }
  }
  const out = [], wiresOut = [];
  const lanes = new Map();   // per board end (above/below): wires routed round it, one lane each
  // A wire from a pin to a point: straight out to the side; a pin on the board's far row goes round its nearer end
  // (above or below), not under the board, so each wire visibly leaves its own pin.
  const curve = (x1, y1, x2, y2, s1, s2, cls, title, L) => {
    let d = 'M' + x1 + ' ' + y1;
    if (L && ((s1 < 0 && x2 > L.x + L.w) || (s1 > 0 && x2 < L.x))) {
      const below = y1 > WD.top + L.h / 2, key = L.b.id + (below ? 'b' : 'a'), lane = lanes.get(key) || 0; lanes.set(key, lane + 1);
      const ox = x1 + s1 * (14 + lane * 5), ey = below ? WD.top + L.h + 12 + lane * 6 : WD.top - 44 - lane * 6, fx = s1 < 0 ? L.x + L.w + 14 + lane * 5 : L.x - 14 - lane * 5;
      d += ' H' + ox + ' V' + ey + ' H' + fx; x1 = fx; y1 = ey; s1 = -s1;
    }
    const dx = Math.max(40, Math.abs(x2 - x1) * 0.45);
    wiresOut.push('<path class="wd-wire wd-' + cls + '" d="' + d + ' C' + (x1 + s1 * dx) + ' ' + y1 + ' ' + (x2 + s2 * dx) + ' ' + y2 + ' ' + x2 + ' ' + y2 + '"><title>' + wdEsc(title) + '</title></path>');
  };
  const supplyCls = t => t === 'GND' ? 'gnd' : t === '5V' ? 'v5' : 'v33';
  for (const p of placed) {
    const tx = p.side < 0 ? p.x + WD.devW : p.x, faces = -p.side;
    out.push('<g class="wd-dev"><rect x="' + p.x + '" y="' + p.y + '" width="' + WD.devW + '" height="' + p.h + '" rx="6"/><text class="wd-dev-name" x="' + (p.x + 10) + '" y="' + (p.y + 17) + '">' + wdEsc(p.d.name) + '</text>' + (p.d.sub ? '<text class="wd-dev-sub" x="' + (p.x + 10) + '" y="' + (p.y + 29) + '">' + wdEsc(p.d.sub.length > 34 ? p.d.sub.slice(0, 33) + '…' : p.d.sub) + '</text>' : '') + '</g>');
    p.terms.forEach((t, i) => {
      const ty = p.y + 40 + i * 18; p.d['_' + t] = { x: tx, y: ty };
      out.push('<circle class="wd-term" cx="' + tx + '" cy="' + ty + '" r="3.5"/><text class="wd-term-label" x="' + (tx - faces * 8) + '" y="' + (ty + 4) + '" text-anchor="' + (faces > 0 ? 'end' : 'start') + '">' + wdEsc(t) + '</text>');
    });
    for (const w of N.wires.filter(w => w.dev === p.d)) { const pin = findPin(p.d.board, w.pin), t = p.d['_' + w.term]; if (pin && t) curve(pin.x, pin.y, t.x, t.y, pin.side, faces, w.kind, p.d.board.name + ' ' + pin.label + ' → ' + p.d.name + ' ' + w.term, pinPos.get(p.d.board.id)); }
    if (opt.power !== false) for (const t of [p.d.power, 'GND'].filter(Boolean)) {
      const pos = p.d['_' + t], pin = findPin(p.d.board, t === '5V' && !pinPos.get(p.d.board.id).pins.some(q => q.label === '5V') ? 'VIN' : t, { y: pos.y, side: p.side }) || findPin(p.d.board, t);
      if (pin && pos) curve(pin.x, pin.y, pos.x, pos.y, pin.side, faces, supplyCls(t), p.d.board.name + ' ' + pin.label + ' → ' + p.d.name + ' ' + t, pinPos.get(p.d.board.id));
    }
  }
  for (const l of N.links) {
    const a = findPin(l.a, l.apin, l.apin === 'GND' ? { y: 9999, side: 1 } : null), b = findPin(l.b, l.bpin, l.bpin === 'GND' ? { y: 0, side: -1 } : null);
    if (a && b) {   // a pin facing away from the other board goes round its own board's end
      const La = pinPos.get(l.a.id), Lb = pinPos.get(l.b.id), cls = l.kind === 'gnd' ? 'gnd' : l.kind;
      const away = (p, L, q) => (p.side < 0 && q.x > L.x + L.w) || (p.side > 0 && q.x < L.x);
      if (away(a, La, b)) curve(a.x, a.y, b.x, b.y, a.side, b.side, cls, l.label, La);
      else if (away(b, Lb, a)) curve(b.x, b.y, a.x, a.y, b.side, a.side, cls, l.label, Lb);
      else curve(a.x, a.y, b.x, b.y, a.side, b.side, cls, l.label);
    }
  }
  // boards on top of the wires (a wire to a pin on the far row passes under the board)
  const usedPins = new Set();
  for (const w of N.wires) { const p = findPin(w.board, w.pin); if (p) usedPins.add(p); }
  for (const l of N.links) { for (const [b, pin] of [[l.a, l.apin], [l.b, l.bpin]]) { const p = findPin(b, pin, pin === 'GND' ? (b === l.a ? { y: 9999, side: 1 } : { y: 0, side: -1 }) : null); if (p) usedPins.add(p); } }
  if (opt.power !== false) for (const p of placed) for (const t of [p.d.power, 'GND'].filter(Boolean)) { const pos = p.d['_' + t], q = findPin(p.d.board, t === '5V' && !pinPos.get(p.d.board.id).pins.some(z => z.label === '5V') ? 'VIN' : t, { y: pos.y, side: p.side }) || findPin(p.d.board, t); if (q) usedPins.add(q); }
  const boardsOut = [];
  for (const L of layout) {
    const hx = L.x, w = L.w, top = WD.top;
    boardsOut.push('<g class="wd-board"><rect x="' + hx + '" y="' + top + '" width="' + w + '" height="' + L.h + '" rx="8"/>' +
      '<text class="wd-board-name" x="' + (hx + w / 2) + '" y="' + (top - 22) + '" text-anchor="middle">' + wdEsc(L.b.name) + '</text>' +
      '<text class="wd-board-sub" x="' + (hx + w / 2) + '" y="' + (top - 8) + '" text-anchor="middle">' + wdEsc(L.P.header ? 'header · pin 1 at the top' : L.key === 'schematic' ? (BOARD_KINDS[L.b.kind]?.label || L.b.kind) + ' · pins in use' : 'USB end ↑') + '</text>');
    for (const p of L.pins) {
      const on = usedPins.has(p), inside = p.side < 0 ? p.x + 9 : p.x - 9;
      boardsOut.push('<circle class="wd-pin' + (on ? ' on' : '') + '" cx="' + p.x + '" cy="' + p.y + '" r="' + (on ? 5 : 3.5) + '"><title>' + wdEsc(p.label + (p.num ? ' · header pin ' + p.num : '') + (p.gpio !== null && !/\d/.test(p.label) ? ' · GPIO ' + p.gpio : '')) + '</title></circle>');
      if (!L.P.header || on) boardsOut.push('<text class="wd-pin-label' + (on ? ' on' : '') + '" x="' + inside + '" y="' + (p.y + 4) + '" text-anchor="' + (p.side < 0 ? 'start' : 'end') + '">' + wdEsc(L.P.header ? p.num + ' ' + p.label : p.label + (p.gpio !== null && !/^\d+$/.test(p.label) && !/^GPIO/.test(p.label) && !/^D\d+$/.test(p.label) ? ' ·' + p.gpio : '')) + '</text>');
    }
    boardsOut.push('</g>');
  }
  let minX = Infinity, maxX = -Infinity, maxY = 0;
  for (const L of layout) { minX = Math.min(minX, L.x - 60); maxX = Math.max(maxX, L.x + L.w + 60); maxY = Math.max(maxY, WD.top + L.h + 12 + 6 * (Math.max(lanes.get(L.b.id + 'b') || 0, 0))); }
  for (const p of placed) { minX = Math.min(minX, p.x - 10); maxX = Math.max(maxX, p.x + WD.devW + 10); maxY = Math.max(maxY, p.y + p.h); }
  const W = maxX - minX, H = maxY + 20;
  return '<svg class="wiring-diagram" viewBox="' + minX + ' 0 ' + W + ' ' + H + '" width="' + W + '" height="' + H + '" role="img" aria-label="Wiring diagram">' + wiresOut.join('') + boardsOut.join('') + out.join('') + '</svg>';
}

// The diagram with its controls: a pinout per board, power wires on or off, a legend.
function renderWiringDiagram(C, opt = {}) {
  const wrap = el('div', { class: 'wd-wrap' }), box = el('div', { class: 'wd-scroll' }), controls = el('div', { class: 'wd-controls' });
  let power = opt.power !== false;
  const draw = () => { box.innerHTML = wiringDiagramSvg(computers(), cfg.comps, { ...opt, power }) || '<p class="hint">Nothing wired yet.</p>'; };
  const shownBoards = C.boards.filter(b => ESP_PROFILES[b.kind] || b.kind.startsWith('pi')).filter(b => !opt.linkTo || b.id === opt.linkTo || b.tasks.includes('core'));
  for (const b of shownBoards) {
    controls.append(hardwareField(b.name + ' pinout', hardwareSelect(b.name + ' pinout', 'wd-pinout-' + b.id + (opt.linkTo ? '-l' + opt.linkTo : ''), pinoutChoices(b), boardPinoutKey(C, b),
      v => editWiring(w => { w.boards[b.id] = { ...hardwareBus(computers(), b), pinout: v }; }, b.id + 'pinout'))));
  }
  const id = 'wd-power-' + (opt.linkTo || 'all'), chk = UI.input({ type: 'checkbox', id }); chk.checked = power;
  chk.addEventListener('change', () => { power = chk.checked; draw(); });
  controls.append(el('label', { class: 'check', for: id }, chk, 'Power wires (3.3 V, 5 V, GND)'));
  const legend = el('div', { class: 'wd-legend' }, ...Object.entries(WD_KIND).map(([k, t]) => el('span', {}, el('i', { class: 'wd-swatch wd-' + k }), t)));
  draw();
  wrap.append(controls, box, legend, el('p', { class: 'hint', text: 'Boards are drawn as you hold them with the USB end up (a Pi: its header, the edge row towards the flight controller). Hover a wire or pin for its name. Check against your board’s silkscreen: DevKits vary.' }));
  return wrap;
}
if (typeof module !== 'undefined') module.exports = { BOARD_PINOUTS, wiringNetlist, wiringDiagramSvg };
