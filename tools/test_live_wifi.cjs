#!/usr/bin/env node
// "Connect to drone" (js/live.js) over Wi-Fi, against a stand-in for the bridge (runner/ground/wifi_bridge.py, tried by
// tools/test_wifi_bridge.py): the page's WebSocket is a mock whose far end is a drone board of the page's own (its
// WebAssembly, the packet layer as the drone's with the Wi-Fi link's sizes: plink.c, as dfb_pi's radio_udp.c runs it),
// so the dialog's bridge command, the command module, the switches, the telemetry into the Ground tab and the 3D
// view, the bridge going away and coming back, and the way back to the simulation are tried without hardware. Requires Playwright and Chrome
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
// The mock: window.WebSocket for the bridge's address. __ws.toPage(bytes) is a datagram from the drone; what the page
// sends goes to __ws.fromPage; __ws.drop() closes it as a bridge stopping; __ws.refuse > 0 refuses that many.
const MOCK = () => {
  const ws = window.__ws = { urls: [], fromPage: [], refuse: 0, cur: null, sent: 0 };
  const Real = window.WebSocket;
  class Mock extends EventTarget {
    constructor(url) {
      super(); ws.urls.push(url); this.url = url; this.readyState = 0; this.binaryType = 'blob'; this.bufferedAmount = 0;
      setTimeout(() => {
        if (ws.refuse > 0) { ws.refuse--; this.readyState = 3; this.onclose && this.onclose({ code: 1006 }); return; }
        this.readyState = 1; ws.cur = this; this.onopen && this.onopen({});
        this.onmessage && this.onmessage({ data: JSON.stringify({ drone: '10.9.8.7:14570' }) });
      }, 20);
    }
    send(b) { if (this.readyState !== 1) throw new Error('not open'); ws.sent++; ws.fromPage.push(new Uint8Array(b.buffer ? b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) : b)); }
    close() { if (this.readyState === 3) return; this.readyState = 3; if (ws.cur === this) ws.cur = null; this.onclose && this.onclose({ code: 1000 }); }
  }
  ws.toPage = b => { const c = ws.cur; if (c && c.readyState === 1) c.onmessage && c.onmessage({ data: b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) }); };
  ws.drop = () => { const c = ws.cur; if (c) { c.readyState = 3; ws.cur = null; c.onclose && c.onclose({ code: 1006 }); } };
  window.WebSocket = Mock; window.WebSocket.Real = Real;
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
    await page.evaluate(async () => { loadPreset('quadx'); while (!brt.ready) await new Promise(r => setTimeout(r, 20)); radioCfg.bind = 'live-test'; Object.assign(radioCfg, { kind: 'wifi', sta: 1, channel: 1 }); try { localStorage.removeItem('dfb.install.drone'); localStorage.removeItem('dfb.install.liveBridge'); } catch (e) {} });
    // Connect: the button, the dialog, "Find the drone"
    assert.strictEqual(await page.locator('.bar #liveBtn').count(),0);
    await page.click('#tabGs');await page.waitForSelector('#paneGs:not([hidden]) #liveBtn');
    assert.strictEqual(await page.locator('#liveBtn').count(),1);
    assert(await page.evaluate(()=>{const button=$('#liveBtn');button.focus();buildGs();return $('#liveBtn')===button&&document.activeElement===button;}),'Ground rebuild replaced the connection action');
    await page.click('#liveBtn'); await page.waitForSelector('#liveDlg[open]');
    assert.strictEqual(await page.textContent('#livePhrase'), 'live-test');
    // The dialog: Wi-Fi, the drone's address and the bridge's, the command to start it
    assert(await page.isVisible('#liveWifi') && await page.textContent('#liveFind') === 'Connect', 'the Wi-Fi part of the dialog');
    assert(/wifi,sta/.test(await page.textContent('#liveHow')));
    await page.fill('#liveDrone', '10.9.8.7'); await page.fill('#liveBridge', 'ws://127.0.0.1:14599');
    assert.strictEqual(await page.textContent('#liveBridgeCmd'), 'python3 runner/ground/wifi_bridge.py --drone 10.9.8.7 --port 14599');
    await page.click('#liveFind');
    await page.waitForFunction(() => liveOn() && live.state === 'up', null, { timeout: 5000 });
    // The drone: the receiver board's own instance, its packet layer the drone's end of Bluetooth LE
    const r1 = await page.evaluate(async () => {
      const tw = brt.inst.get(boardOf('tlm').id), ph = new TextEncoder().encode(radioCfg.bind);
      new Uint8Array(tw.memory.buffer, tw.rbuf_ptr(), ph.length).set(ph);
      tw.plink_setup(1, ph.length, 12345, 2, 1, 1);
      let big = 0;
      const D = window.__drone = { tw, nextPub: 0, armSeen: false, chOk: 0 };
      D.timer = setInterval(() => {
        const t = performance.now() / 1000;
        while (__ws.fromPage.length) { const p = __ws.fromPage.shift(); big = Math.max(big, p.length); new Uint8Array(tw.memory.buffer, tw.pbuf_ptr(), p.length).set(p); tw.plink_air_in(p.length, -50, t); }
        const n = tw.plink_stack_out(t); if (n) tw.radio_in(n, t);
        if (t >= D.nextPub) { D.nextPub = t + 0.01; tw.tlm_publish(1 | 2, t); }
        const m = tw.radio_out(t); if (m) tw.plink_stack_in(m, t);
        for (let k = 0; k < 3; k++) { const q = tw.plink_air_out(t); if (!q) break; __ws.toPage(Uint8Array.from(new Uint8Array(tw.memory.buffer, tw.pbuf_ptr(), q))); }
        if (tw.rc_link_ok(t)) D.chOk++;
      }, 4);
      await new Promise(r => setTimeout(r, 2500));
      UI_PANELS.editor.select('gs'); renderGs(true);
      return { urls: __ws.urls, target: live.target, signal: LIVE_MODEL.signalNote(), lq: gs.link, att: !!gs.v.attitude, batt: !!gs.v.battery, writes: __ws.sent, big, chOk: D.chOk, note: $('#gsCmdNote').textContent, bar: $('#liveRead').textContent, view: document.querySelector('.view').classList.contains('live'), running };
    });
    assert.deepStrictEqual(r1.urls, ['ws://127.0.0.1:14599'], 'the bridge\'s address as set');
    assert(r1.target === '10.9.8.7:14570' && /bridge/.test(r1.signal) && /Wi-Fi/.test(r1.note), 'the bridge said where the packets go: ' + r1.signal + ' | ' + r1.note);
    assert(r1.lq && r1.lq.upLq > 90 && r1.lq.downLq > 90, 'link quality both ways: ' + JSON.stringify(r1.lq));
    assert(r1.att && r1.batt, 'the drone\'s attitude and battery reached the Ground tab');
    assert(r1.big > 0 && r1.big <= 250, 'packets the Wi-Fi link\'s size: ' + r1.big);
    assert(r1.writes > 200 && r1.chOk > 100, `packets up (${r1.writes}) and channels at the drone (${r1.chOk})`);
    assert(r1.view && !r1.running && /real drone/.test(r1.note) && /LQ/.test(r1.bar), 'live mode shown: ' + r1.note + ' | ' + r1.bar);
    if (process.env.LIVE_SCREENSHOT) await page.screenshot({ path: process.env.LIVE_SCREENSHOT });
    console.log('connect: the dialog\'s bridge address, packets both ways, LQ', Math.round(r1.lq.upLq), Math.round(r1.lq.downLq), '%, telemetry in the Ground tab');
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
      __ws.refuse = 2; __ws.drop(); await new Promise(r => setTimeout(r, 200)); const lost = live.state;
      await new Promise(r => setTimeout(r, 4500));
      return { lost, back: live.state, ok: LIVE_MODEL.connected(), log: radio.log.slice(0, 12).map(e => e.data) };
    });
    assert.strictEqual(r3.lost, 'lost'); assert.strictEqual(r3.back, 'up'); assert(r3.ok, 'packets again after the reconnect: ' + r3.log.join(' / '));
    console.log('the bridge gone: reconnected by itself after two refusals');
    // Disconnect: the simulation is back
    const r4 = await page.evaluate(async () => {
      clearInterval(__drone.timer); $('#liveStop').click(); await new Promise(r => setTimeout(r, 300));   // (the stand-in drone is the simulator's own receiver board: it stops first)
      return { on: liveOn(), gnd: brt.gnd === brt.gndInst, running, bar: $('#liveBar').hidden, conn: !!__ws.cur, model: radioModel() === RADIO_LINKS[radioCfg.kind] };
    });
    assert(!r4.on && r4.gnd && r4.running && r4.bar && !r4.conn && r4.model, 'back to the simulation: ' + JSON.stringify(r4));
    await page.waitForFunction(() => brt.ready && brt.fcState === 1, null, { timeout: 15000 });
    console.log('disconnect: the simulation runs again and flies');
    assert.deepStrictEqual(errors, []);
    console.log('live Wi-Fi (mock bridge): ok');
  } finally { if (browser) await browser.close(); server.close(); }
})().catch(e => { console.error(e); server.close(); process.exitCode = 1; });
