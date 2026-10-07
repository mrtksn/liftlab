#!/usr/bin/env node
// "Connect to drone" (js/live.js) against a stand-in for Web Bluetooth: the page's Bluetooth is a mock whose far end
// is a drone board of the page's own (its WebAssembly, the packet layer as the drone's: plink.c, as radio_ble.c runs
// it), so the page's command module, the switches, the telemetry into the Ground tab and the 3D view, a dropped
// connection and the way back to the simulation are tried without hardware. Requires Playwright and Chrome
// (PLAYWRIGHT_PATH / CHROME_PATH as for test_flight_browser.cjs; THREE_PATH: a local three.min.js r128, if the page
// can't fetch it).
'use strict';
const fs = require('fs'), path = require('path'), http = require('http'), assert = require('assert');
const { chromium } = require(process.env.PLAYWRIGHT_PATH || 'playwright');
const root = path.resolve(__dirname, '..');
const server = http.createServer((req, res) => {
  const url = new URL(req.url, 'http://localhost'), file = path.resolve(root, '.' + decodeURIComponent(url.pathname === '/' ? '/index.html' : url.pathname));
  if (!file.startsWith(root + path.sep)) { res.writeHead(403).end(); return; }
  fs.readFile(file, (err, data) => { if (err) { res.writeHead(404).end(); return; } res.setHeader('Cache-Control', 'no-store');
    res.setHeader('Content-Type', file.endsWith('.js') ? 'application/javascript' : file.endsWith('.css') ? 'text/css' : file.endsWith('.html') ? 'text/html' : 'application/octet-stream'); res.end(data); });
});
// The mock: navigator.bluetooth with one device, LiftLab's service, its three characteristics. __bt.toPage(bytes)
// notifies DOWN; what the page writes to UP goes to __bt.fromPage; __bt.drop() drops the connection.
const MOCK = () => {
  const U = n => `6c1f000${n}-6b3d-4e8b-9a7e-6c6966746c62`;
  const bt = window.__bt = { reqs: [], fromPage: [], mtu: 185, writes: 0, refuse: 0 };
  const ch = u => Object.assign(new EventTarget(), { uuid: u });
  const up = ch(U(2)), down = ch(U(3)), info = ch(U(4));
  up.writeValueWithoutResponse = async b => { if (!dev.gatt.connected) throw new Error('not connected'); bt.writes++; bt.fromPage.push(new Uint8Array(b.buffer ? b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) : b)); };
  down.startNotifications = async () => down;
  info.readValue = async () => { const d = new DataView(new ArrayBuffer(3)); d.setUint16(0, bt.mtu, true); d.setUint8(2, 1); return d; };
  const svc = { getCharacteristic: async u => ({ [U(2)]: up, [U(3)]: down, [U(4)]: info })[u] };
  const dev = Object.assign(new EventTarget(), { name: 'LiftLab-TEST', id: 'x' });
  dev.gatt = { connected: false, async connect() { if (bt.refuse > 0) { bt.refuse--; throw new Error('out of range'); } this.connected = true; return { getPrimaryService: async u => { if (u !== U(1)) throw new Error('no service'); return svc; } }; }, disconnect() { this.connected = false; } };
  bt.toPage = b => { if (!dev.gatt.connected) return; Object.defineProperty(down, 'value', { value: new DataView(b.buffer.slice(0)), configurable: true }); down.dispatchEvent(new Event('characteristicvaluechanged')); };
  bt.drop = () => { dev.gatt.connected = false; dev.dispatchEvent(new Event('gattserverdisconnected')); };
  bt.dev = dev;
  Object.defineProperty(navigator, 'bluetooth', { value: { requestDevice: async o => { bt.reqs.push(o); return dev; } }, configurable: true });
};
(async () => {
  await new Promise(r => server.listen(0, '127.0.0.1', r)); let browser;
  try {
    browser = await chromium.launch({ executablePath: process.env.CHROME_PATH || undefined, headless: true, args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
    const [vw, vh] = (process.env.LIVE_VIEWPORT || '1440x1000').split('x').map(Number), page = await browser.newPage({ viewport: { width: vw, height: vh } }), errors = [];
    page.on('pageerror', e => errors.push(e.message));
    await page.route(/fonts\.google|goatcounter|gc\.zgo/, r => r.abort());
    if (process.env.THREE_PATH) await page.route(/three\.min\.js/, r => r.fulfill({ path: process.env.THREE_PATH, contentType: 'application/javascript' }));
    await page.addInitScript(MOCK);
    await page.goto('http://127.0.0.1:' + server.address().port, { waitUntil: 'networkidle' });
    await page.waitForFunction(() => typeof brt !== 'undefined' && brt.ready);
    await page.evaluate(async () => { loadPreset('quadx'); while (!brt.ready) await new Promise(r => setTimeout(r, 20)); radioCfg.bind = 'live-test'; });
    // Connect: the button, the dialog, "Find the drone"
    await page.click('#liveBtn'); await page.waitForSelector('#liveDlg[open]');
    assert.strictEqual(await page.textContent('#livePhrase'), 'live-test');
    await page.click('#liveFind');
    await page.waitForFunction(() => liveOn() && live.state === 'up', null, { timeout: 5000 });
    // The drone: the receiver board's own instance, its packet layer the drone's end of Bluetooth LE
    const r1 = await page.evaluate(async () => {
      const tw = brt.inst.get(boardOf('tlm').id), ph = new TextEncoder().encode(radioCfg.bind);
      new Uint8Array(tw.memory.buffer, tw.rbuf_ptr(), ph.length).set(ph);
      const mark = tw.ble_mark(ph.length) >>> 0, want = [mark & 255, (mark >>> 8) & 255, (mark >>> 16) & 255, mark >>> 24];
      const f = __bt.reqs[0].filters[0], got = Array.from(f.manufacturerData[0].dataPrefix);
      new Uint8Array(tw.memory.buffer, tw.rbuf_ptr(), ph.length).set(ph);
      tw.plink_setup(1, ph.length, 12345, 5, 0, 0); tw.plink_mtu(182);
      const D = window.__drone = { tw, nextPub: 0, armSeen: false, chOk: 0 };
      D.timer = setInterval(() => {
        const t = performance.now() / 1000;
        while (__bt.fromPage.length) { const p = __bt.fromPage.shift(); new Uint8Array(tw.memory.buffer, tw.pbuf_ptr(), p.length).set(p); tw.plink_air_in(p.length, -50, t); }
        const n = tw.plink_stack_out(t); if (n) tw.radio_in(n, t);
        if (t >= D.nextPub) { D.nextPub = t + 0.01; tw.tlm_publish(1 | 2, t); }
        const m = tw.radio_out(t); if (m) tw.plink_stack_in(m, t);
        for (let k = 0; k < 3; k++) { const q = tw.plink_air_out(t); if (!q) break; __bt.toPage(Uint8Array.from(new Uint8Array(tw.memory.buffer, tw.pbuf_ptr(), q))); }
        if (tw.rc_link_ok(t)) D.chOk++;
      }, 4);
      await new Promise(r => setTimeout(r, 2500));
      UI_PANELS.editor.select('gs'); renderGs(true);
      return { want, got, svc: f.services[0], lq: gs.link, att: !!gs.v.attitude, batt: !!gs.v.battery, writes: __bt.writes, chOk: D.chOk, note: $('#gsCmdNote').textContent, bar: $('#liveRead').textContent, view: document.querySelector('.view').classList.contains('live'), running };
    });
    assert.deepStrictEqual(r1.got, r1.want, 'the filter carries the phrase\'s mark');
    assert.strictEqual(r1.svc, '6c1f0001-6b3d-4e8b-9a7e-6c6966746c62');
    assert(r1.lq && r1.lq.upLq > 90 && r1.lq.downLq > 90, 'link quality both ways: ' + JSON.stringify(r1.lq));
    assert(r1.att && r1.batt, 'the drone\'s attitude and battery reached the Ground tab');
    assert(r1.writes > 200 && r1.chOk > 100, `packets up (${r1.writes}) and channels at the drone (${r1.chOk})`);
    assert(r1.view && !r1.running && /real drone/.test(r1.note) && /LQ/.test(r1.bar), 'live mode shown: ' + r1.note + ' | ' + r1.bar);
    if (process.env.LIVE_SCREENSHOT) await page.screenshot({ path: process.env.LIVE_SCREENSHOT });
    console.log('connect: the phrase\'s mark in the filter, packets both ways, LQ', Math.round(r1.lq.upLq), Math.round(r1.lq.downLq), '%, telemetry in the Ground tab');
    // The switches: arm reaches the drone's channels; the 3D view follows the telemetry
    const r2 = await page.evaluate(async () => {
      const tw = __drone.tw, ch = () => { const n = tw.rc_pack(performance.now() / 1000); return Array.from(new Float32Array(tw.memory.buffer, tw.fr_ptr(), n)); };
      const before = ch(); $('#liveArm').click(); await new Promise(r => setTimeout(r, 400)); const after = ch();
      $('#liveArm').click(); await new Promise(r => setTimeout(r, 300));
      const off = ch(), v = gs.v.attitude, q = liveQuat(v);
      return { before: before[4], after: after[4], off: off[4], q, Sq: S.q };
    });
    assert(r2.before < -0.5 && r2.after > 0.5 && r2.off < -0.5, `the arm channel at the drone: ${r2.before} → ${r2.after} → ${r2.off}`);
    assert(r2.Sq.every((c, i) => Math.abs(c - r2.q[i]) < 0.05), 'the view shows the telemetry\'s attitude');
    console.log('switches: arm reached the drone; the 3D view follows its attitude');
    // A dropped connection: reconnects by itself
    const r3 = await page.evaluate(async () => {
      __bt.refuse = 2; __bt.drop(); await new Promise(r => setTimeout(r, 200)); const lost = live.state;
      await new Promise(r => setTimeout(r, 3500));
      return { lost, back: live.state, ok: LIVE_MODEL.connected(), log: radio.log.slice(0, 12).map(e => e.data) };
    });
    assert.strictEqual(r3.lost, 'lost'); assert.strictEqual(r3.back, 'up'); assert(r3.ok, 'packets again after the reconnect: ' + r3.log.join(' / '));
    console.log('a dropped connection: reconnected by itself after two refusals');
    // Disconnect: the simulation is back
    const r4 = await page.evaluate(async () => {
      $('#liveStop').click(); await new Promise(r => setTimeout(r, 300)); clearInterval(__drone.timer);
      return { on: liveOn(), gnd: brt.gnd === brt.gndInst, running, bar: $('#liveBar').hidden, conn: __bt.dev.gatt.connected, model: radioModel() === RADIO_LINKS[radioCfg.kind] };
    });
    assert(!r4.on && r4.gnd && r4.running && r4.bar && !r4.conn && r4.model, 'back to the simulation: ' + JSON.stringify(r4));
    await page.waitForFunction(() => brt.ready && brt.fcState === 1, null, { timeout: 15000 });
    console.log('disconnect: the simulation runs again and flies');
    assert.deepStrictEqual(errors, []);
    console.log('live Bluetooth (mock): ok');
  } finally { if (browser) await browser.close(); server.close(); }
})().catch(e => { console.error(e); server.close(); process.exitCode = 1; });
