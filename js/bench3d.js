'use strict';
// The Computers tab's 3D bench. With the tab open, the view draws the design level (as Edit does), its
// electronics pulled apart (an exploded view) and the rest of the airframe faded behind them. Each board is drawn as a
// board: its plate and its header pins where its model (board-pinouts.js) puts them, the pins in use coloured by what
// they carry. What is picked in the list (or clicked in the view) is closed in on, and its wires are drawn from the pin
// they leave to the part they reach; the wiring view draws all of them. Nothing here is saved: it only reads the design
// and its wiring (hardware.js), so every wire shown is one the install guide asks for.

const bench = { on: false, want: true, wasEdit: false, view: null, phys: null, hoverKey: null, hiWire: null, ex: 0, exT: 0, t0: 0,
  items: new Map(), wires: [], group: null, ghosts: [], layer: null, labels: [], zoom: null, dirty: true, hoverBox: null, sig: '' };

// What each kind of board looks like when it has no part of its own on the airframe [m], what Put it on the airframe
// weighs [kg], and its plate's colour.
const BENCH_BODY = {
  esp32: { size: [0.052, 0.028, 0.012], mass: 0.009, plate: '#22303f' },
  s3: { size: [0.063, 0.026, 0.012], mass: 0.011, plate: '#22303f' },
  c3: { size: [0.023, 0.018, 0.006], mass: 0.002, plate: '#262c34' },
  pizero: { size: [0.065, 0.030, 0.005], mass: 0.009, plate: '#2e7a45' },
  pizero2: { size: [0.065, 0.030, 0.005], mass: 0.010, plate: '#2e7a45' },
  pi4: { size: [0.085, 0.056, 0.017], mass: 0.046, plate: '#2e7a45' },
  mac: { size: [0.12, 0.08, 0.008], mass: 0, plate: '#5b6470' },
};
const benchBody = kind => BENCH_BODY[kind] || BENCH_BODY.esp32;

// Electronics: what the Computers tab manages rather than the Airframe tab. Sensors always; a rigid mass when it says so
// (c.elec), or by default when it powers something, draws power, carries a camera, is a board's body or is named like
// electronics (a converter, a cable, a power monitor). The part's own Electronics switch overrides the guess.
const ELEC_NAME = /\b(boards?|pcb|perfboard|esp\d*|esp32\S*|pi|raspberry|controller|fc|converter|regulator|bec|boost|buck|monitor|ina\d+|xt\d+|leads?|wires?|wiring|cables?|usb|receiver|camera|vtx|module|headers?|battery|lipo)\b/i;
function partIsElec(c) {
  if (!c) return false;
  if (c.type === 'sensor') return true;
  if (c.type !== 'mass' || c.cargo || (typeof isWing === 'function' && isWing(c))) return false;
  if (typeof c.elec === 'boolean') return c.elec;
  return !!(c.battery || (c.boardId != null && c.boardId !== -1) || c.deviceW > 0 || c.camera || (!c.model && ELEC_NAME.test(c.name || '')));   // (an imported object only when it says so)
}
// A board's body: the rigid mass that is that board (c.boardId), or one named like it that no board claims yet.
// boardId -1 on a mass says it's no board's.
function boardBodyOf(b, comps = cfg.comps) {
  if (!b || typeof b.id !== 'number') return null;
  const own = comps.find(c => c.type === 'mass' && c.boardId === b.id); if (own) return own;
  const norm = s => String(s || '').trim().toLowerCase(), names = [b.name, BOARD_KINDS[b.kind] && BOARD_KINDS[b.kind].label].map(norm).filter(Boolean);
  const boards = computers().boards, taken = new Set(boards.map(x => x !== b && comps.find(c => c.type === 'mass' && c.boardId === x.id)).filter(Boolean));
  for (const x of boards) { if (x === b) break; const g = comps.find(c => c.type === 'mass' && c.boardId == null && !c.battery && [x.name, BOARD_KINDS[x.kind] && BOARD_KINDS[x.kind].label].map(norm).includes(norm(c.name))); if (g) taken.add(g); }   // (an earlier board's guess wins)
  return comps.find(c => c.type === 'mass' && c.boardId == null && !c.battery && !taken.has(c) && names.includes(norm(c.name))) || null;
}
const boardOfBody = (c, C = computers()) => c && c.type === 'mass' ? C.boards.find(b => boardBodyOf(b) === c) || null : null;
// Put a board on the airframe: a part the size and weight of the board, on the frame above the hub.
function placeBoardBody(b) {
  const B = benchBody(b.kind), n = computers().boards.indexOf(b), c = mkMass(b.name, 0, 0, frameDims()[2] / 2 + B.size[2] / 2 + 0.004 + 0.014 * Math.max(0, n), { mass: B.mass, size: B.size.slice() });
  c.boardId = b.id; c.elec = true; c.known = true; cfg.comps.push(c); undoKey = 'boardbody' + b.id; structural(); return c;
}
function linkBoardBody(b, id) {   // this part is that board ('' : no part)
  for (const c of cfg.comps) if (c.type === 'mass' && c.boardId === b.id) c.boardId = -1;
  const c = compById(Number(id)); if (c) { c.boardId = b.id; c.elec = true; }
  undoKey = 'boardbody' + b.id; structural(); if (typeof renderComputers === 'function') renderComputers(true);
}
const benchSizeOf = c => {
  if (c.type === 'sensor') return ({ imu: [0.022, 0.022, 0.008], mag: [0.016, 0.016, 0.006], baro: [0.014, 0.014, 0.01], fix: [0.03, 0.03, 0.008], flow: [0.02, 0.02, 0.012], tof: [0.02, 0.013, 0.00665] })[c.kind] || [0.02, 0.02, 0.01];
  if (c.type === 'mass' && !c.model) return c.shape === 'sphere' ? [2 * c.radius, 2 * c.radius, 2 * c.radius] : c.shape === 'cylinder' ? [2 * c.radius, 2 * c.radius, c.length] : c.size.slice();
  if (c.type === 'motor') { const r = propR(c); return [2 * r, 2 * r, 0.03]; }
  return [0.04, 0.03, 0.03];
};

/* ───────── wires: what connects to what, from the saved wiring ───────── */
const WIRE_KINDS = { sda: 'I²C SDA', scl: 'I²C SCL', pwm: 'PWM signal', tx: 'UART TX', rx: 'UART RX', signal: 'Signal', v33: '3.3 V', v5: '5 V', power: 'Power', gnd: 'GND', radio: 'Radio link' };
const benchColors = {};
function benchColor(kind) {
  if (benchColors[kind]) return benchColors[kind];
  const probe = el('i', { class: 'pd-' + (kind === 'power' ? 'v5' : kind === 'radio' ? 'signal' : kind) }); probe.style.display = 'none'; document.body.append(probe);
  const v = getComputedStyle(probe).getPropertyValue('--pd-c').trim() || '#888'; probe.remove();
  return (benchColors[kind] = v);
}
// Every wire of the design: { id, a: {key, pin}, b: {key, pin}, kind, text, view } — a and b are bench item keys, pin a
// GPIO number or a supply ('3v3', '5v', 'gnd') on a board, or null for a part (the wire goes to the part itself).
function benchWires(C = computers(), comps = cfg.comps) {
  const out = [], seen = new Set();
  const put = (a, b, kind, text, view, extra = {}) => { const id = a.key + '|' + a.pin + '>' + b.key + '|' + b.pin + ':' + kind; if (seen.has(id)) return; seen.add(id); out.push({ id, a, b, kind, text, view, ...extra }); };
  const bk = b => 'board:' + b.id, pk = c => 'part:' + c.id;
  const pinName = (b, g) => { const s = typeof boardGpioShort === 'function' ? boardGpioShort(C, b, g) : 'GPIO ' + g; return /^\d+$/.test(s) ? 'GPIO ' + s : s; };
  for (const b of C.boards) {
    const claims = typeof hardwarePinClaims === 'function' ? hardwarePinClaims(C, comps, b) : [];
    const i2c = comps.filter(c => c.type === 'sensor' && I2C_SENSOR_KINDS.includes(c.kind) && hardwareOwner(C, c) === b);
    for (const cl of claims) {
      const k = cl.key, part = /^part(\d+)$/.exec(k), gps = /^gps(Tx|Rx)(\d+)$/.exec(k), pow = /^imupow(\d+)$/.exec(k);
      if (k === 'sda' || k === 'scl') for (const c of i2c) put({ key: bk(b), pin: cl.pin }, { key: pk(c), pin: null }, k, pinName(b, cl.pin) + ' → ' + c.name + ' ' + k.toUpperCase(), { kind: 'device', id: c.id });
      else if (k === 'battery' || k === 'battery2') { const P = k === 'battery' ? mainPack(C, comps) : sensedPack2(C, comps); if (P) put({ key: bk(b), pin: cl.pin }, { key: pk(P), pin: null }, 'signal', pinName(b, cl.pin) + ' ← ' + P.name + ' voltage divider', { kind: 'power', id: P.id }); }
      else if (k === 'rx' || k === 'tx' || /^nrf\d$/.test(k)) put({ key: bk(b), pin: cl.pin }, { key: 'radio', pin: null }, k === 'rx' ? 'rx' : k === 'tx' ? 'tx' : 'signal', pinName(b, cl.pin) + ' · ' + cl.name, { kind: 'radio' });
      else if (k === 'rxTx' || k === 'rxRx') put({ key: bk(b), pin: cl.pin }, { key: 'radio', pin: null }, k === 'rxTx' ? 'tx' : 'rx', pinName(b, cl.pin) + ' · ' + cl.name, { kind: 'radio' });
      else if (part) { const c = comps.find(x => x.id === +part[1]); if (c) { const p = hardwarePart(C, c, comps); put({ key: bk(b), pin: cl.pin }, { key: pk(c), pin: null }, 'pwm', pinName(b, cl.pin) + ' → ' + c.name + ' ' + (c.type === 'motor' ? (p.driver === 'brushed' ? 'MOSFET gate' : 'ESC signal') : c.type === 'joint' ? 'servo signal' : 'latch'), { kind: 'device', id: c.id }); } }
      else if (gps) { const c = comps.find(x => x.id === +gps[2]); if (c) put({ key: bk(b), pin: cl.pin }, { key: pk(c), pin: null }, gps[1] === 'Tx' ? 'tx' : 'rx', pinName(b, cl.pin) + ' · ' + cl.name, { kind: 'device', id: c.id }); }
      else if (pow) { const c = comps.find(x => x.id === +pow[1]); if (c) put({ key: bk(b), pin: cl.pin }, { key: pk(c), pin: null }, 'v33', pinName(b, cl.pin) + ' → ' + c.name + ' power switch', { kind: 'device', id: c.id }); }
    }
  }
  for (const c of comps.filter(c => c.type === 'sensor' && c.kind === 'fix')) {   // a GPS on a USB serial adapter
    const b = hardwareOwner(C, c), p = b && hardwarePart(C, c, comps);
    if (b && (p.port || '/dev/ttyUSB0') !== '/dev/serial0') put({ key: bk(b), pin: null }, { key: pk(c), pin: null }, 'signal', b.name + ' USB (' + (p.port || '/dev/ttyUSB0') + ') ← ' + c.name + ' through a USB serial adapter', { kind: 'device', id: c.id }, { dashed: true });
  }
  for (const L of (typeof boardLinks === 'function' ? boardLinks(C, comps) : [])) {
    const view = { kind: 'link', id: L.b.id };
    if (L.transport === 'mqtt' || !L.pi) { put({ key: bk(L.a), pin: null }, { key: bk(L.b), pin: null }, 'radio', L.a.name + ' ↔ ' + L.b.name + (L.transport === 'mqtt' ? ' · MQTT over Wi-Fi' : ' · simulated link'), view, { dashed: true }); continue; }
    const fl = fcLinkPins(C, L.a); if (!fl) continue;
    if (L.port === '/dev/serial0') {
      put({ key: bk(L.a), pin: fl.tx }, { key: bk(L.b), pin: 15 }, 'tx', L.a.name + ' ' + pinName(L.a, fl.tx) + ' TX → ' + L.b.name + ' ' + pinName(L.b, 15) + ' RX', view);
      put({ key: bk(L.b), pin: 14 }, { key: bk(L.a), pin: fl.rx }, 'rx', L.b.name + ' ' + pinName(L.b, 14) + ' TX → ' + L.a.name + ' ' + pinName(L.a, fl.rx) + ' RX', view);
      put({ key: bk(L.a), pin: 'gnd' }, { key: bk(L.b), pin: 'gnd' }, 'gnd', L.a.name + ' GND ↔ ' + L.b.name + ' GND', view);
    } else put({ key: bk(L.a), pin: fl.tx }, { key: bk(L.b), pin: null }, 'signal', L.a.name + ' ' + pinName(L.a, fl.tx) + '/' + pinName(L.a, fl.rx) + ' ↔ USB serial adapter → ' + L.b.name + ' ' + L.port, view, { dashed: true });
  }
  // power: each source to what it powers (a board through its 5 V input, a motor's ESC, a part with a draw of its own)
  const srcKey = s => s.kind === 'battery' ? pk(s.part) : 'source:' + s.id;
  for (const k of powerConsumers(C, comps)) {
    const s = consumerSource(C, comps, k.key); if (!s) continue;
    if (k.kind === 'board') {
      const S = boardSupplyOf(C, k.board, comps);
      if (S.kind === 'board' && S.from) { put({ key: bk(S.from), pin: '5v' }, { key: bk(k.board), pin: '5v' }, 'v5', S.from.name + ' 5 V pin → ' + k.board.name + ' 5 V', { kind: 'board', id: k.board.id }); continue; }
      put({ key: srcKey(s), pin: null }, { key: bk(k.board), pin: '5v' }, 'power', s.name + ' → ' + k.board.name + ' (' + (POWER_SUPPLIES[S.kind] ? POWER_SUPPLIES[S.kind].short : '5 V') + ')', { kind: 'power', id: s.id });
    } else if (k.kind === 'motor') put({ key: srcKey(s), pin: null }, { key: pk(k.part), pin: null }, 'power', s.name + ' → ' + k.name, { kind: 'power', id: s.id });
    else if (k.kind === 'part') put({ key: srcKey(s), pin: null }, { key: pk(k.part), pin: null }, 'power', s.name + ' → ' + k.name, { kind: 'power', id: s.id });
  }
  const tlm = C.boards.find(b => b.tasks.includes('tlm'));
  if (tlm && C.ground) put({ key: 'board:ground', pin: null }, { key: out.some(w => w.b.key === 'radio') ? 'radio' : bk(tlm), pin: null }, 'radio', C.ground.name + ' ⇢ ' + (typeof RADIO_LINKS !== 'undefined' && RADIO_LINKS[radioCfg.kind] ? RADIO_LINKS[radioCfg.kind].label : 'radio') + ' ⇢ ' + tlm.name, { kind: 'radio' }, { dashed: true });
  return out;
}

/* ───────── the board drawn as a board ───────── */
// Its plate, a module or chip on top and its headers, the pins where the board model puts them. pins: key → local
// position of the pin's tip ('g' + GPIO, and the first 3v3 / 5v / gnd), with which column it's in.
function benchPcb(C, b, dims) {
  const g = new THREE.Group(), M = typeof boardModel === 'function' ? boardModel(C, b) : null, B = benchBody(b.kind);
  const alongX = dims[0] >= dims[1], L = Math.max(dims[0], dims[1]), W = Math.min(dims[0], dims[1]), T = 0.0016;
  if (!alongX) g.rotation.z = Math.PI / 2;   // build along X, turn to the part's long side
  const mat = c => new THREE.MeshLambertMaterial({ color: c });
  const plate = new THREE.Mesh(new THREE.BoxGeometry(L, W, T), mat(B.plate)); g.add(plate);
  const top = T / 2, pi = !BOARD_KINDS[b.kind] || !BOARD_KINDS[b.kind].mcu;
  if (pi) {   // a Pi: the SoC in the middle, the SD card at one end
    const soc = new THREE.Mesh(new THREE.BoxGeometry(Math.min(0.012, L * 0.2), Math.min(0.012, W * 0.45), 0.0012), mat('#30343a')); soc.position.set(-L * 0.08, 0, top + 0.0006); g.add(soc);
    const sd = new THREE.Mesh(new THREE.BoxGeometry(0.011, Math.min(0.012, W * 0.4), 0.0012), mat('#b9bec5')); sd.position.set(-L / 2 + 0.0055, 0, top + 0.0006); g.add(sd);
  } else {     // an ESP32: its module (the metal can and the antenna) at one end, the USB socket at the other
    const mw = Math.min(0.018, W * 0.7), ml = Math.min(0.025, L * 0.42);
    const can = new THREE.Mesh(new THREE.BoxGeometry(ml * 0.7, mw, 0.0028), mat('#aeb4bb')); can.position.set(L / 2 - ml * 0.65, 0, top + 0.0014); g.add(can);
    const ant = new THREE.Mesh(new THREE.BoxGeometry(ml * 0.3, mw, 0.0008), mat('#1b1f24')); ant.position.set(L / 2 - ml * 0.15, 0, top + 0.0004); g.add(ant);
    const usb = new THREE.Mesh(new THREE.BoxGeometry(0.006, Math.min(0.009, W * 0.4), 0.0028), mat('#9aa0a8')); usb.position.set(-L / 2 + 0.003, 0, top + 0.0014); g.add(usb);
  }
  const pins = new Map(), cols = M ? M.cols : [[], []], pitch = 0.00254;
  const used = typeof boardPinUses === 'function' && typeof b.id === 'number' ? boardPinUses(C, cfg.comps, b) : new Map();
  const n = Math.max(cols[0].length, cols[1].length, 1), step = Math.min(pitch, (L - 0.006) / n), x0 = (n - 1) * step / 2;
  const header = (y, len) => { const h = new THREE.Mesh(new THREE.BoxGeometry(len, 0.0025, 0.0025), mat('#15181c')); h.position.set(0, y, top + 0.00125); g.add(h); };
  const gold = mat('#d6b25e'), rowY = M && M.layout === 'header' ? [-(W / 2 - 0.0045), -(W / 2 - 0.002)] : [W / 2 - 0.0016, -(W / 2 - 0.0016)];
  if (M) for (let ci = 0; ci < 2; ci++) {
    if (!cols[ci].length) continue;
    header(rowY[ci], cols[ci].length * step);
    cols[ci].forEach((p, i) => {
      const x = (M.layout === 'header' ? x0 - i * step : x0 - i * step), key = p.gpio !== null ? 'g' + p.gpio : p.fn;
      const u = used.get(p.gpio !== null ? p.gpio : p.fn === '3v3' ? '3v3' : p.fn === '5v' ? '5v' : p.fn === 'gnd' ? 'gnd' : null);
      const pinM = new THREE.Mesh(new THREE.BoxGeometry(0.0007, 0.0007, 0.006), u && p.gpio !== null ? new THREE.MeshBasicMaterial({ color: benchColor(u[0].kind === 'pwm' ? 'pwm' : u[0].kind) }) : gold);
      pinM.position.set(x, rowY[ci], top + 0.003); g.add(pinM);
      const at = new THREE.Vector3(x, rowY[ci], top + 0.006);
      if (!pins.has(key)) pins.set(key, { at, col: ci, label: p.label, num: p.num });
    });
  }
  for (const [k, fn] of [['3v3', '3v3'], ['5v', '5v'], ['gnd', 'gnd']]) if (!pins.has(k)) pins.set(k, { at: new THREE.Vector3(-L / 2 + 0.004, 0, top + 0.002), col: 0, label: fn.toUpperCase() });
  g.userData.pinTargets = pins; g.userData.size = [L, W]; g.userData.alongX = alongX;
  return { g, pins };
}

/* ───────── entering and leaving ───────── */
const benchTabOpen = () => typeof UI_PANELS !== 'undefined' && !$('#paneForm').hidden;
function benchSync() {
  const can = bench.want && benchTabOpen() && typeof fleet !== 'undefined' && fleet.ready && !!fleet.selected
    && !(typeof partImport !== 'undefined' && partImport.active) && !(typeof worldEditOn === 'function' && worldEditOn());
  if (can && !bench.on) benchEnter(); else if (!can && bench.on) benchLeave();
  benchBarSync();
}
function benchEnter() {
  bench.on = true; bench.wasEdit = editMode; bench.ex = 0; bench.exT = 0; bench.phys = null; bench.t0 = performance.now(); bench.dirty = true; bench.view = typeof COMP !== 'undefined' ? COMP.focus : null;
  const r = running;   // (only the view changes: the flight, or the pause, carries on, so Live data and apps keep running)
  if (!editMode) { setEditMode(true); running = r; editWasRunning = r; renderRun(); } else { selectComp(null); setHover(null); $('#editBar').hidden = true; }
  if (cam.el < 0.35) cam.el = 0.55;
  bench.zoom = { fit: true, to: 0.8, t0: performance.now() };
  droneShown = '';   // redraw for the bench
}
function benchLeave() {
  benchPhysEnd();
  bench.on = false; benchClearLabels(); bench.items = new Map(); bench.wires = []; bench.group = null; bench.ghosts = [];
  if (bench.hoverBox) bench.hoverBox.visible = false;
  if (editMode) { if (bench.wasEdit && !$('#paneAir').hidden) { $('#editBar').hidden = false; updateEditMsg(); } else { const r = bench.wasEdit ? editWasRunning : running; editWasRunning = r; setEditMode(false); } }
  droneShown = '';
}
// Called by setEditMode(false) from elsewhere (Run, a flight starting): the bench closes with it.
function benchEditEnded() { if (!bench.on) return; bench.phys = null; bench.on = false; benchClearLabels(); bench.items = new Map(); bench.wires = []; bench.group = null; bench.ghosts = []; droneShown = ''; benchBarSync(); }
function benchBarSync() {
  const bar = $('#benchBar'); if (!bar) return; bar.hidden = !bench.on;
  $('.view').classList.toggle('benching', bench.on && !(typeof liveOn === 'function' && liveOn()));   // (the drone card steps aside while the sim is paused for the bench)
  const wiring = !!(bench.view && bench.view.kind === 'wiring');
  bar.querySelectorAll('[data-bench-mode]').forEach(b => b.setAttribute('aria-pressed', String((b.dataset.benchMode === 'wiring') === wiring)));
  const msg = $('#benchMsg'); if (msg) msg.textContent = bench.on ? benchMessage() : '';
  const inspect = $('#benchInspect'); if (inspect) { inspect.setAttribute('aria-pressed', String(bench.on)); inspect.textContent = bench.on ? 'Back to flying' : 'Inspect in 3D'; }
}
function benchMessage() {
  const v = bench.view;
  if (!v) return 'Exploded view · click a part or pick one in the list to close in on it.' + (running ? ' The drone flies on: Fly shows it.' : '');
  if (bench.phys != null) { const c = compById(bench.phys); return (c ? c.name : 'The part') + ': drag the arrows or squares to move it, the rings to turn it. Shift for fine steps; Esc or Done when it sits right.'; }
  if (v.kind === 'wiring') return 'Every wire, from the pin it leaves to the part it reaches. Hover a wire to name it.';
  const it = benchFocusKeys(v).map(k => bench.items.get(k)).filter(Boolean)[0];
  return (it ? it.name : 'Selected') + ' · its wires are drawn; Esc or click empty space to step back.';
}
// The bench follows what the Computers list has open.
function benchFocus(view) {
  if (bench.phys != null && !(view && benchFocusKeys(view).includes('part:' + bench.phys) || view && view.kind === 'board' && boardBodyOf(computers().boards.find(b => b.id === view.id)) === compById(bench.phys))) benchPhysEnd();
  bench.view = view || null; bench.hiWire = null; bench.dirty = true;
  if (view) bench.want = true;
  benchSync();
  if (!bench.on) return;
  const keys = benchFocusKeys(bench.view), its = keys.map(k => bench.items.get(k)).filter(Boolean);
  const wiring = bench.view && bench.view.kind === 'wiring';
  const r = its.length ? Math.max(...its.map(i => Math.max(i.size[0], i.size[1]))) : 0, board = its.length === 1 && its[0].kind === 'board';
  bench.zoom = { fit: wiring || !its.length, to: wiring || !its.length ? 0.9 : its.length > 1 ? 0.55 : board ? Math.max(0.15, r * 3.4) : Math.min(0.6, Math.max(0.17, r * 4.5)), t0: performance.now() };
  const el1 = board ? 1.0 : its.length ? 0.75 : 0.6;   // a board from above, so its pin rows show
  if (Math.abs(cam.el - el1) > 0.05) cam.anim = { az0: cam.az, el0: cam.el, daz: 0, el1, t0: performance.now(), dur: 450 };
  benchBarSync();
}
function benchFocusKeys(v) {
  if (!v) return [];
  const C = computers();
  if (v.kind === 'board') return ['board:' + v.id];
  if (v.kind === 'power') return [bench.items.has('part:' + v.id) ? 'part:' + v.id : 'source:' + v.id];
  if (v.kind === 'device' || v.kind === 'part') return ['part:' + v.id];
  if (v.kind === 'link') { const L = boardLinks(C).find(L => L.b.id === v.id); return L ? ['board:' + L.a.id, 'board:' + L.b.id] : []; }
  if (v.kind === 'task') { const b = boardOf(v.id); return b ? ['board:' + b.id] : []; }
  if (v.kind === 'radio') { const b = boardOf('tlm'); return [...(b ? ['board:' + b.id] : []), 'radio']; }
  return [];
}
const benchViewOf = key => {
  const it = bench.items.get(key); if (!it) return null;
  if (it.kind === 'board') return { kind: 'board', id: it.board === computers().ground ? 'ground' : it.board.id };
  if (it.kind === 'source') return { kind: 'power', id: it.source.id };
  if (it.kind === 'radio') return { kind: 'radio' };
  const c = it.comp; if (!c) return null;
  if (c.type === 'mass' && c.battery) return { kind: 'power', id: c.id };
  if (c.type === 'sensor' || ['motor', 'joint', 'latch'].includes(c.type)) return { kind: 'device', id: c.id };
  return { kind: 'part', id: c.id };
};

/* ───────── building the bench (after the drone is drawn) ───────── */
function benchRebuild() {
  if (!bench.on || !editMode) return;
  const C = computers(), comps = cfg.comps;
  bench.items = new Map(); bench.ghosts = []; bench.dirty = true;
  const owner = o => { let p = o; while (p && p !== drone && p.userData.compId == null) p = p.parent; return p && p !== drone ? p.userData.compId : null; };
  const inside = (o, g) => { for (let p = o; p; p = p.parent) if (p === g) return true; return false; };
  const elec = new Set(comps.filter(partIsElec).map(c => c.id));
  const wired = new Set(comps.filter(c => ['motor', 'joint', 'latch'].includes(c.type)).map(c => c.id));
  const meshes = []; drone.traverse(o => { if (o.isMesh || o.isLine) meshes.push(o); });
  for (const o of meshes) {
    const id = owner(o), pg = id != null ? pickGroups.get(id) : null;
    if (id != null && elec.has(id)) { if (!pg || !inside(o, pg)) o.visible = false; else if (o.material && !Array.isArray(o.material)) { o.userData.benchOrig = o.material; o.material = o.material.clone(); o.material.transparent = true; } continue; }
    if (o.isLine || (o.material && o.material.isLineBasicMaterial)) { o.visible = false; continue; }
    const orig = o.material; if (!orig || Array.isArray(orig)) continue;
    const ghost = new THREE.MeshBasicMaterial({ color: orig.color ? orig.color.clone().lerp(colorOf('--panel'), 0.35) : colorOf('--muted'), transparent: true, opacity: 0.14, depthWrite: false, side: THREE.DoubleSide });
    const pp = id != null && parts.get(id), prop = pp && (o === pp.disc || o === pp.stub || o === pp.wake);
    o.material = ghost; o.userData.benchGhost = true; bench.ghosts.push({ o, orig, ghost, id: id != null && wired.has(id) && inside(o, pg) && !prop ? id : null });
  }
  bench.group = new THREE.Group(); bench.group.userData.noPick = true; drone.add(bench.group);
  const add = (key, it) => bench.items.set(key, { key, off: new THREE.Vector3(), dim: false, ...it });
  // boards: their own part drawn as a board, or (not on the airframe yet) a board standing in above the hub
  const fz = frameDims()[2] / 2;
  let dock = 0;
  for (const b of C.boards) {
    const body = boardBodyOf(b, comps), pg = body && pickGroups.get(body.id), boxy = body && !body.model && body.shape === 'box';
    const dims = boxy ? body.size : benchBody(b.kind).size, pcb = benchPcb(C, b, dims);
    let obj;
    if (pg) {
      obj = pg;
      if (boxy && obj.isMesh) { obj.material = new THREE.MeshBasicMaterial({ transparent: true, opacity: 0, depthWrite: false }); obj.userData.benchHidden = true; }
      else pcb.g.position.z = benchSizeOf(body)[2] / 2 + 0.003;
      obj.add(pcb.g);
    } else {
      obj = new THREE.Group(); obj.position.set(0, 0, fz + 0.045 + 0.02 * dock++); obj.add(pcb.g); bench.group.add(obj);
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(dims[0] + 0.006, dims[1] + 0.006, 0.01)), new THREE.LineDashedMaterial({ color: colorOf('--muted'), dashSize: 0.004, gapSize: 0.003 }));
      edges.computeLineDistances(); obj.add(edges);
    }
    add('board:' + b.id, { kind: 'board', board: b, comp: body || null, obj, pcb, pins: pcb.pins, base: obj.position.clone(), p: body ? body.pos.slice() : obj.position.toArray(), size: dims.slice(), virtual: !body, explode: true, name: b.name });
  }
  {   // the command module, off the drone
    const g = C.ground, dims = benchBody(g.kind).size.map(v => v * 0.6), pcb = benchPcb(C, { ...g, id: 'ground' }, benchBody(g.kind).size), obj = new THREE.Group();
    obj.add(pcb.g); obj.position.set(-0.42, 0.28, -0.02); bench.group.add(obj);
    add('board:ground', { kind: 'board', board: g, obj, pcb, pins: pcb.pins, base: obj.position.clone(), p: obj.position.toArray(), size: dims, virtual: true, offboard: true, explode: false, name: g.name + ' (ground)' });
  }
  for (const s of powerSources(C, comps).filter(s => s.kind !== 'battery')) {   // a USB port or a bench supply, off the drone
    const obj = new THREE.Group(), n = [...bench.items.values()].filter(i => i.kind === 'source').length;
    const box = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.035, 0.02), new THREE.MeshLambertMaterial({ color: s.kind === 'usb' ? '#5b6470' : '#3d4450', transparent: true })); obj.add(box);
    obj.position.set(-0.42, -0.25 - 0.08 * n, -0.02); bench.group.add(obj);
    add('source:' + s.id, { kind: 'source', source: s, obj, base: obj.position.clone(), p: obj.position.toArray(), size: [0.05, 0.035, 0.02], virtual: true, offboard: true, explode: false, name: s.name });
  }
  // the radio module, when the telemetry board wires one
  const wires = benchWires(C, comps);
  if (wires.some(w => w.b.key === 'radio' || w.a.key === 'radio')) {
    const tb = boardOf('tlm'), at = tb && bench.items.get('board:' + tb.id), obj = new THREE.Group();
    obj.add(new THREE.Mesh(new THREE.BoxGeometry(0.018, 0.012, 0.004), new THREE.MeshLambertMaterial({ color: '#2b3442', transparent: true })));
    const ant = rod([0, 0.004, 0.002], [0, 0.004, 0.03], 0.0007, new THREE.MeshLambertMaterial({ color: '#c9ccd1' })); if (ant) obj.add(ant);
    obj.position.copy(at ? at.base : new THREE.Vector3()).add(new THREE.Vector3(0, 0.05, 0.02)); bench.group.add(obj);
    const r = typeof hardwareRadio === 'function' && tb ? radioWiringRow(tb, hardwareBus(C, tb), !!ESP_PROFILES[tb.kind], hardwareRadio()) : null;
    add('radio', { kind: 'radio', obj, base: obj.position.clone(), p: obj.position.toArray(), size: [0.018, 0.012, 0.004], virtual: true, explode: true, name: r ? r.device : 'Radio module' });
  }
  // the rest of the electronics, and the parts a board drives (they stay where they are)
  for (const c of comps) {
    const pg = pickGroups.get(c.id); if (!pg) continue;
    if (elec.has(c.id)) { if ([...bench.items.values()].some(i => i.comp === c)) continue; add('part:' + c.id, { kind: c.battery ? 'battery' : c.type === 'sensor' ? 'sensor' : 'part', comp: c, obj: pg, base: pg.position.clone(), p: c.pos.slice(), size: benchSizeOf(c), explode: true, name: c.name }); }
    else if (wired.has(c.id)) add('part:' + c.id, { kind: 'output', comp: c, obj: pg, base: pg.position.clone(), p: c.pos.slice(), size: benchSizeOf(c), explode: false, name: c.name });
  }
  for (const it of bench.items.values()) {   // its meshes, for dimming
    it.meshes = []; it.obj.traverse(o => { if (o.isMesh && o.material && !Array.isArray(o.material) && !o.userData.benchHidden && !o.userData.benchGhost) { if (!o.material.transparent) { o.material = o.material.clone(); o.material.transparent = true; } it.meshes.push(o); } });
  }
  benchLayout();
  bench.wires = wires.filter(w => bench.items.has(w.a.key) && bench.items.has(w.b.key));
  for (const w of bench.wires) { w.mesh = null; }
  if (!bench.hoverBox) { bench.hoverBox = new THREE.Box3Helper(new THREE.Box3(), 0xffffff); bench.hoverBox.material.depthTest = false; bench.hoverBox.renderOrder = 19; bench.hoverBox.visible = false; scene.add(bench.hoverBox); }
  bench.hoverBox.material.color = colorOf('--accent');
  if (bench.layer) bench.layer.textContent = ''; bench.labels = [];
}
// The exploded places: each electronic part moved out from the hub, then pushed apart until none overlaps another or
// the hub, so a stack (a board on a converter on the battery) fans out and every part can be seen and picked.
function benchLayout() {
  const its = [...bench.items.values()].filter(i => i.explode);
  const hub = frameDims(), q = its.map(i => new THREE.Vector3(...i.p)), r = its.map(i => 0.5 * Math.hypot(i.size[0], i.size[1], Math.min(i.size[2], 0.03)) + 0.006);
  for (let i = 0; i < its.length; i++) {
    const p = q[i], h = Math.hypot(p.x, p.y);
    if (h > 1e-4) { p.x *= 1.35; p.y *= 1.35; }
    p.z += (p.z >= -0.005 ? 1 : -1) * (0.035 + 0.01 * (i % 3));
  }
  for (let iter = 0; iter < 120; iter++) {
    let moved = false;
    for (let i = 0; i < its.length; i++) for (let j = i + 1; j < its.length; j++) {
      const d = q[j].clone().sub(q[i]), min = (r[i] + r[j]) * 1.05; let L = d.length();
      if (L >= min) continue;
      if (L < 1e-5) { const a = (i * 2.399 + j * 0.7); d.set(Math.cos(a), Math.sin(a), 0.35); L = d.length(); }
      d.multiplyScalar((min - L) / L * 0.5); d.z *= 0.6; q[i].sub(d); q[j].add(d); moved = true;
    }
    for (let i = 0; i < its.length; i++) {   // keep out of the hub
      const p = q[i], inX = Math.abs(p.x) < hub[0] / 2 + r[i] * 0.7, inY = Math.abs(p.y) < hub[1] / 2 + r[i] * 0.7, gap = hub[2] / 2 + r[i] * 0.6;
      if (inX && inY && Math.abs(p.z) < gap) { p.z = (p.z >= 0 ? 1 : -1) * gap; moved = true; }
    }
    if (!moved) break;
  }
  its.forEach((it, i) => it.off.copy(q[i]).sub(new THREE.Vector3(...it.p)));
}

/* ───────── every frame ───────── */
const benchEase = u => u * u * (3 - 2 * u);
const benchTmp = new THREE.Vector3();
function benchWorldPin(it, pin, toward) {
  if (pin != null && it.pins) {
    const P = it.pins.get(typeof pin === 'number' ? 'g' + pin : pin);
    if (P) return it.pcb.g.localToWorld(P.at.clone());
  }
  const c = new THREE.Vector3(); it.obj.getWorldPosition(c);
  if (toward) { const d = toward.clone().sub(c); const L = d.length(); if (L > 1e-6) c.addScaledVector(d, Math.min(0.45 * Math.max(...it.size.slice(0, 2)), L * 0.3) / L); }
  return c;
}
function benchFrame(hub) {
  if (!bench.on || !bench.group) return null;
  const now = performance.now();
  const dt = Math.min(0.1, (now - (bench.lastT || now)) / 1000); bench.lastT = now;
  const goal = bench.phys != null ? 0 : 1, was = bench.ex;   // (put back together while a part is placed: its handles act where it really is)
  bench.exT = goal > bench.exT ? Math.min(goal, bench.exT + dt / 0.55) : Math.max(goal, bench.exT - dt / 0.4); bench.ex = benchEase(bench.exT);
  const moving = bench.ex !== was;
  const v = bench.view, wiring = !!(v && v.kind === 'wiring'), focus = new Set(benchFocusKeys(v));
  const related = new Set(focus), hiWires = new Set();
  for (const w of bench.wires) if (wiring ? (!bench.hiWire || bench.hiWire === w.id) : focus.has(w.a.key) || focus.has(w.b.key)) { hiWires.add(w.id); if (!wiring || bench.hiWire) { related.add(w.a.key); related.add(w.b.key); } }
  if (wiring && !bench.hiWire) for (const w of bench.wires) { related.add(w.a.key); related.add(w.b.key); }
  // places and fading
  for (const it of bench.items.values()) {
    if (it.explode) it.obj.position.copy(it.base).addScaledVector(it.off, bench.ex);
    const on = !v || related.has(it.key), op = on ? 1 : 0.22;
    for (const m of it.meshes) { m.material.opacity = op; m.material.depthWrite = on; }
  }
  for (const g of bench.ghosts) {   // the airframe: faded, but a part a board drives keeps its colour while its wire shows
    const show = g.id != null && related.has('part:' + g.id) && (wiring || focus.size);
    g.o.material = show ? g.orig : g.ghost;
  }
  for (const p of parts.values()) { if (p.arrow) p.arrow.visible = false; if (p.spin) p.spin.visible = false; if (p.wake) p.wake.visible = false; if (p.beam) p.beam.visible = false; }
  for (const rv of rangeVis.values()) rv.g.visible = false;
  if (cogDot) cogDot.visible = false; if (modelRing) modelRing.visible = false;
  // wires
  if (bench.dirty) benchBarSync();
  if (moving || bench.dirty || bench.wiresDirty) benchDrawWires(hiWires, wiring);
  bench.dirty = false; bench.wiresDirty = false;
  // labels
  benchLabels(focus, related, hiWires, wiring);
  // camera: closes in on what's open (once, as it opens: after that the wheel is yours)
  if (bench.zoom) { cam.dist += (bench.zoom.to - cam.dist) * (1 - Math.exp(-dt * 9)); if (Math.abs(bench.zoom.to - cam.dist) < 0.004 || now - bench.zoom.t0 > 3000) bench.zoom = null; }
  const keys = focus.size ? [...focus] : wiring ? bench.wires.flatMap(w => [w.a.key, w.b.key]) : [...bench.items.values()].filter(i => i.explode || !v).map(i => i.key);
  const box = new THREE.Box3();
  for (const k of keys) { const it = bench.items.get(k); if (it && (!it.offboard || focus.has(k))) box.expandByPoint(it.obj.getWorldPosition(benchTmp)); }
  if (box.isEmpty()) return null;
  if (bench.zoom && bench.zoom.fit) { const s = box.getSize(new THREE.Vector3()); bench.zoom.to = Math.min(1.4, Math.max(0.35, Math.hypot(s.x, s.y, s.z) * 1.35)); }
  return box.getCenter(new THREE.Vector3());
}
function benchDrawWires(hi, wiring) {
  for (const w of bench.wires) { if (w.mesh) { bench.group.remove(w.mesh); w.mesh.geometry.dispose(); w.mesh.material.dispose(); w.mesh = null; } }
  const show = wiring || bench.view;
  if (!show) return;
  drone.updateWorldMatrix(true, true);
  const inv = new THREE.Matrix4().copy(drone.matrixWorld).invert();
  for (const w of bench.wires) {
    const on = hi.has(w.id); if (!wiring && !on) continue;
    const A = bench.items.get(w.a.key), B = bench.items.get(w.b.key);
    let pa = A.pins && w.a.pin != null ? benchWorldPin(A, w.a.pin) : null, pb = B.pins && w.b.pin != null ? benchWorldPin(B, w.b.pin) : null;
    if (!pa) pa = benchWorldPin(A, null, pb || B.obj.getWorldPosition(new THREE.Vector3()));
    if (!pb) pb = benchWorldPin(B, null, pa);
    pa.applyMatrix4(inv); pb.applyMatrix4(inv);
    const d = pa.distanceTo(pb), h = Math.min(0.06, 0.18 * d + 0.008), up = new THREE.Vector3(0, 0, 1);
    const ua = A.pins && w.a.pin != null ? up : pb.clone().sub(pa).setZ(0).normalize().multiplyScalar(0.5).add(up).normalize();
    const ub = B.pins && w.b.pin != null ? up : pa.clone().sub(pb).setZ(0).normalize().multiplyScalar(0.5).add(up).normalize();
    const curve = new THREE.CubicBezierCurve3(pa, pa.clone().addScaledVector(ua, h), pb.clone().addScaledVector(ub, h), pb);
    const thick = w.kind === 'power' ? 0.0011 : w.kind === 'radio' ? 0.0005 : 0.0006, strong = on && (!wiring || bench.hiWire === w.id || !bench.hiWire);
    const mat = new THREE.MeshBasicMaterial({ color: benchColor(w.kind), transparent: true, opacity: strong ? (w.dashed ? 0.55 : 0.95) : 0.16, depthWrite: strong });
    w.mesh = new THREE.Mesh(new THREE.TubeGeometry(curve, 28, thick * (bench.hiWire === w.id ? 1.8 : 1), 6, false), mat);
    w.mesh.userData.wire = w.id; bench.group.add(w.mesh);
  }
}

/* ───────── labels over the view ───────── */
function benchClearLabels() { if (bench.layer) bench.layer.textContent = ''; bench.labels = []; bench.labelSig = ''; }
function benchLabels(focus, related, hiWires, wiring) {
  if (!bench.layer) { bench.layer = el('div', { class: 'bench-layer', 'aria-hidden': 'true' }); $('.view').append(bench.layer); }
  const C = computers(), want = [];
  for (const it of bench.items.values()) {
    const f = focus.has(it.key), on = f || (bench.view ? related.has(it.key) : it.explode || it.offboard);
    if (!on || (it.kind === 'output' && !related.has(it.key))) continue;
    want.push({ id: 'n' + it.key, it, text: it.name, cls: f ? 'bench-tag focus' : 'bench-tag' + (it.virtual && !it.offboard && it.kind === 'board' ? ' virtual' : '') });
    if (f) for (const line of benchInfo(it, C)) want.push({ id: 'i' + it.key + line, it, text: line, cls: 'bench-info', info: true });
  }
  // pins in use: on the open board, or on every board in the wiring view
  for (const it of bench.items.values()) {
    if (it.kind !== 'board' || !it.pins || !(focus.has(it.key) || wiring)) continue;
    const uses = typeof boardPinUses === 'function' && typeof it.board.id === 'number' ? boardPinUses(C, cfg.comps, it.board) : new Map();
    for (const [k, list] of uses) {
      const P = it.pins.get(typeof k === 'number' ? 'g' + k : k); if (!P) continue;
      let short = typeof k === 'number' ? (typeof boardGpioShort === 'function' ? boardGpioShort(C, it.board, k) : 'GPIO ' + k) : P.label; if (/^\d+$/.test(short)) short = 'GPIO ' + short;
      const text = focus.has(it.key) ? short + ' · ' + list.map(u => u.text).join('; ') : short;
      if (wiring && !focus.has(it.key) && typeof k !== 'number') continue;
      want.push({ id: 'p' + it.key + ':' + k, it, pin: P, text, cls: 'bench-pin pd-' + list[0].kind });
    }
  }
  if (bench.hiWire) { const w = bench.wires.find(w => w.id === bench.hiWire); if (w) want.push({ id: 'w' + w.id, wire: w, text: w.text, cls: 'bench-wire pd-' + (w.kind === 'power' ? 'v5' : w.kind === 'radio' ? 'signal' : w.kind) }); }
  const sig = want.map(w => w.id + w.text).join('|');
  if (sig !== bench.labelSig) {
    bench.labelSig = sig; bench.layer.textContent = '';
    bench.labels = want.map(w => { const n = el('div', { class: w.cls, text: w.text }); bench.layer.append(n); return { ...w, n }; });
  }
  const r = vpEl.getBoundingClientRect(), vr = $('.view').getBoundingClientRect(), ox = r.left - vr.left, oy = r.top - vr.top, W = r.width, H = r.height;
  const proj = p => { const q = p.clone().project(camera); return { x: ox + (q.x + 1) / 2 * W, y: oy + (1 - q.y) / 2 * H, ok: q.z < 1 && q.z > -1 }; };
  const count = new Map(), stack = new Map(), placed = [];
  for (const L of bench.labels) if (!L.pin && !L.wire) count.set(L.it.key, (count.get(L.it.key) || 0) + 1);
  for (const L of bench.labels) {
    let p, cls = '';
    if (L.wire) { const m = L.wire.mesh; if (!m) { L.n.hidden = true; continue; } const g = m.geometry.parameters.path.getPoint(0.5); p = proj(drone.localToWorld(g.clone())); }
    else if (L.pin) { const w = L.it.pcb.g.localToWorld(L.pin.at.clone()), c = L.it.obj.getWorldPosition(new THREE.Vector3()); p = proj(w); const pc = proj(c); cls = p.x < pc.x ? 'left' : 'right'; }
    else {
      const c = L.it.obj.getWorldPosition(new THREE.Vector3()); c.z += Math.max(0.008, L.it.size[2] / 2 + 0.006); p = proj(c);
      const k = L.it.key, n = stack.get(k) || 0; stack.set(k, n + 1); p.y -= 6 + (count.get(k) - 1 - n) * 17;   // the name, its facts under it, all above the part
    }
    L.p = p; L.side = cls;
  }
  // Nothing on top of anything: the most important first (the wire under the pointer, the open part, its pins), each
  // later one moved up or down until it's clear, or left out (a far part's name).
  const rank = L => L.wire ? 0 : L.it && focus.has(L.it.key) ? (L.pin ? 2 : 1) : L.pin ? 3 : 4;
  for (const L of [...bench.labels].sort((a, b) => rank(a) - rank(b))) {
    const p = L.p; if (!p || !p.ok || p.x < -40 || p.y < -20 || p.x > vr.width + 40 || p.y > vr.height + 20) { L.n.hidden = true; continue; }
    if (L.pin) { L.n.classList.toggle('left', L.side === 'left'); L.n.classList.toggle('right', L.side !== 'left'); }
    if (!L.w) { L.n.hidden = false; L.w = L.n.offsetWidth; L.h = L.n.offsetHeight; }
    const w = L.w + 2, h = L.h + 1;
    const x0 = L.pin ? (L.side === 'left' ? p.x - 6 - w : p.x + 6) : p.x - w / 2, y0 = L.pin ? p.y - h / 2 : L.wire ? p.y - 1.3 * h : p.y - h;
    const tries = L.pin ? [0, 1, -1, 2, -2, 3, -3, 4, -4, 5, -5] : L.info || rank(L) <= 1 ? [0, -1, 1, -2, 2] : [0, -1, -2, 1];
    let dy = null;
    for (const t of tries) { const y = y0 + t * h; if (!placed.some(r => x0 < r.x + r.w && x0 + w > r.x && y < r.y + r.h && y + h > r.y)) { dy = t * h; break; } }
    if (dy === null && rank(L) >= 3) { L.n.hidden = true; continue; }
    dy = dy || 0; placed.push({ x: x0, y: y0 + dy, w, h });
    L.n.hidden = false; L.n.style.transform = 'translate(' + Math.round(p.x) + 'px,' + Math.round(p.y + dy) + 'px)';
  }
}
// The open item's few facts, drawn beside it.
function benchInfo(it, C) {
  const out = [], g = x => Math.round(x * 1000) + ' g';
  if (it.kind === 'board') {
    const K = BOARD_KINDS[it.board.kind]; out.push(K.label + (it.offboard ? ' · on the ground' : it.virtual ? ' · not on the airframe yet' : it.comp ? ' · ' + g(it.comp.mass) : ''));
    if (typeof it.board.id === 'number') { const t = it.board.tasks.map(t => TASKS[t].label).join(', '); if (t) out.push(t); }
    const M = typeof boardModel === 'function' ? boardModel(C, it.board) : null; if (M) out.push(M.label);
  } else if (it.kind === 'source') out.push(sourceLabel(C, it.source, cfg.comps), 'off the drone');
  else if (it.kind === 'radio') out.push('wired to ' + (boardOf('tlm') || {}).name);
  else if (it.comp) {
    const c = it.comp;
    if (c.type === 'mass' && c.battery) { const s = powerSources(C, cfg.comps).find(s => s.id === c.id); if (s) out.push(sourceLabel(C, s, cfg.comps)); }
    else if (c.type === 'sensor') { const b = hardwareOwner(C, c); out.push(SENSOR_KINDS[c.kind] + (b ? ' → ' + b.name : ' · no board')); }
    else if (['motor', 'joint', 'latch'].includes(c.type)) { const b = hardwareOwner(C, c), p = hardwarePart(C, c, cfg.comps); out.push(b ? b.name + ' · ' + (p.pin >= 0 ? (typeof boardGpioShort === 'function' ? boardGpioShort(C, b, p.pin) : 'GPIO ' + p.pin) : 'no pin') : 'no board'); }
    if (c.mass != null) out.push(g(c.mass) + (c.type === 'mass' && c.size && c.shape === 'box' ? ' · ' + c.size.map(v => Math.round(v * 1000)).join('×') + ' mm' : ''));
  }
  return out;
}

/* ───────── pointing at things ───────── */
const benchRay = new THREE.Raycaster();
function benchPick(e) {
  const r = renderer.domElement.getBoundingClientRect();
  benchRay.setFromCamera(new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1), camera);
  benchRay.params.Line.threshold = 0.002;
  const targets = [...bench.items.values()].filter(i => i.kind !== 'output' || (bench.view && bench.view.kind === 'wiring')).map(i => i.obj);
  const wires = bench.wires.filter(w => w.mesh).map(w => w.mesh);
  let best = null;
  for (const h of benchRay.intersectObjects([...targets, ...wires], true)) {
    if (h.object.isLine || h.object.visible === false) continue;
    if (h.object.userData.wire) { best = best || { wire: h.object.userData.wire, d: h.distance }; continue; }
    for (const it of bench.items.values()) { let o = h.object; while (o && o !== it.obj) o = o.parent; if (o) { if (!best || best.wire || h.distance < best.d) return { key: it.key, d: h.distance }; } }
  }
  return best;
}
function benchPointerMove(e, orbiting) {
  if (orbiting) { benchHover(null); return false; }
  const h = benchPick(e); benchHover(h);
  const tip = $('#pickTip');
  if (h) {
    const r = $('.view').getBoundingClientRect(), it = h.key && bench.items.get(h.key), w = h.wire && bench.wires.find(x => x.id === h.wire);
    tip.textContent = it ? it.name + ' · click to open' : w ? w.text : ''; tip.hidden = !tip.textContent;
    tip.style.left = (e.clientX - r.left + 14) + 'px'; tip.style.top = (e.clientY - r.top + 12) + 'px';
  } else tip.hidden = true;
  vpEl.style.cursor = h ? 'pointer' : '';
  return false;
}
function benchHover(h) {
  const key = h && h.key || null, it = key && bench.items.get(key);
  bench.hoverKey = key;
  if (bench.hoverBox) { bench.hoverBox.visible = !!it; if (it) bench.hoverBox.box.setFromObject(it.obj); }
  const wire = h && h.wire || null;
  if (bench.view && bench.view.kind === 'wiring' && wire !== bench.hoverWire) { bench.hoverWire = wire; benchHiWire(wire); }
}
function benchHiWire(id) { if (bench.hiWire === id) return; bench.hiWire = id; bench.wiresDirty = true; document.querySelectorAll('[data-wire]').forEach(n => n.classList.toggle('hi', n.dataset.wire === id)); }
function benchClick(e) {
  const h = benchPick(e);
  let view = null;
  if (h && h.key) view = benchViewOf(h.key);
  else if (h && h.wire) { const w = bench.wires.find(x => x.id === h.wire); if (bench.view && bench.view.kind === 'wiring') { benchHiWire(h.wire); const row = document.querySelector('[data-wire="' + CSS.escape(h.wire) + '"]'); if (row) row.scrollIntoView({ block: 'nearest' }); return; } view = w && w.view; }
  if (typeof compFocus === 'function') compFocus(view, { from3d: true });
}
if (typeof vpEl !== 'undefined') for (const t of ['wheel', 'pointerdown', 'touchstart']) vpEl.addEventListener(t, () => { bench.zoom = null; }, { passive: true });   // (your own zoom wins)
window.addEventListener('keydown', e => {
  if (!bench.on || e.defaultPrevented || typingIn(e.target)) return;
  if (e.code === 'Escape' && !document.querySelector('dialog[open]')) { e.preventDefault(); if (bench.phys != null) { benchPhys(null); return; } if (bench.view && typeof compFocus === 'function') compFocus(null); }
});

/* ───────── the bar over the view ───────── */
document.querySelectorAll('#benchBar [data-bench-mode]').forEach(b => b.addEventListener('click', () => {
  if (typeof compFocus !== 'function') return;
  if (b.dataset.benchMode === 'wiring') compFocus({ kind: 'wiring' }); else compFocus(bench.view && bench.view.kind === 'wiring' ? null : bench.view);
}));
$('#benchDone')?.addEventListener('click', () => { bench.want = false; benchSync(); if (typeof renderComputers === 'function') renderComputers(true); });
$('#addElecGo')?.addEventListener('click', () => { $('#addPartDlg').close(); bench.want = true; UI_PANELS.editor.select('form'); requestAnimationFrame(() => $('#compAddMenu button')?.click()); });

/* ───────── placing a part: Airframe's handles, on the assembled design ───────── */
// Edit physical properties puts the exploded view back together and gives the part Edit's handles (arrows move along
// an axis, squares within a plane, rings turn it), with its fields in the list. Done, another row or Esc ends it.
function benchPhys(id) {
  const c = id != null ? compById(id) : null;
  if (!c || !bench.on) { benchPhysEnd(); if (typeof renderComputers === 'function') renderComputers(true); return; }
  bench.phys = c.id; edit.sel = c.id; edit.hover = null; edit.refocus = true; bench.dirty = true;
  bench.zoom = { to: Math.min(0.6, Math.max(0.3, cam.dist)), t0: performance.now() };
  if (typeof renderComputers === 'function') renderComputers(true);
  benchBarSync();
}
function benchPhysEnd() {
  if (bench.phys == null) return;
  if (edit.drag) endDrag();
  bench.phys = null; edit.sel = null; bench.dirty = true; benchBarSync();
}
