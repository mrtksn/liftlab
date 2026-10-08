#!/usr/bin/env node
// The board on the install dialog's USB cable (js/install-ui.js, js/usb-view.js) against a stand-in for Web Serial: a
// port whose far end answers "version" as the flight firmware does (a REPORT frame) and sends its telemetry
// (RN_LINK_TELEM). Tries the build comparison with firmware/manifest.json, old firmware's answer, "Show it in the 3D
// view" (the drone follows the board's attitude and height, the simulation paused), the bar's Console and Stop, Run
// and Disconnect ending the view. Requires Playwright and Chrome (PLAYWRIGHT_PATH / CHROME_PATH as for
// test_flight_browser.cjs; THREE_PATH: a local three.min.js r128, if the page can't fetch it).
'use strict';
const fs = require('fs'), path = require('path'), http = require('http'), assert = require('assert');
const { chromium } = require(process.env.PLAYWRIGHT_PATH || 'playwright');
const root = path.resolve(__dirname, '..');
const server = http.createServer((req, res) => {
  const url = new URL(req.url, 'http://localhost'), file = path.resolve(root, '.' + decodeURIComponent(url.pathname === '/' ? '/index.html' : url.pathname));
  if (!file.startsWith(root + path.sep)) { res.writeHead(403).end(); return; }
  fs.readFile(file, (err, data) => { if (err) { res.writeHead(404).end(); return; } res.setHeader('Cache-Control', 'no-store');
    res.setHeader('Content-Type', file.endsWith('.js') ? 'application/javascript' : file.endsWith('.css') ? 'text/css' : file.endsWith('.html') ? 'text/html' : file.endsWith('.json') ? 'application/json' : 'application/octet-stream'); res.end(data); });
});
// The mock: navigator.serial.requestPort gives one port. __usb.reply is what the board answers to "version" (a REPORT
// frame's text); __usb.telem(roll, pitch, yaw, height) sends one telemetry frame; __usb.lines is what the page wrote.
const MOCK = () => {
  const usb = window.__usb = { reply: '', lines: [], ctrl: null };
  const frame = (type, p) => { const out = new Uint8Array(11 + p.length), dv = new DataView(out.buffer); out[0] = 0x44; out[1] = 0x46; out[2] = type; dv.setUint32(3, p.length, true); out.set(p, 7); dv.setUint32(7 + p.length, rnCrc32(out.subarray(2, 7 + p.length)), true); return out; };
  const port = {
    async open() { this.readable = new ReadableStream({ start(c) { usb.ctrl = c; } }); this.writable = new WritableStream({ write(b) {
      const s = new TextDecoder().decode(b); usb.lines.push(s);
      if (s.includes('version') && usb.reply) setTimeout(() => usb.ctrl.enqueue(frame(0x82, new TextEncoder().encode(usb.reply))), 20);
    } }); },
    async setSignals() {}, async close() {},
  };
  usb.telem = (roll, pitch, yaw, h) => { const t = new Float32Array(36); t[2] = roll; t[3] = pitch; t[4] = yaw; t[8] = h; t[11] = 450; t[13] = 1 | 2 | 4; usb.ctrl.enqueue(frame(0x83, new Uint8Array(t.buffer))); };
  usb.port = port;
  const serial = Object.assign(new EventTarget(), { requestPort: async () => port });
  Object.defineProperty(navigator, 'serial', { value: serial, configurable: true });
};
(async () => {
  await new Promise(r => server.listen(0, '127.0.0.1', r)); let browser;
  try {
    browser = await chromium.launch({ executablePath: process.env.CHROME_PATH || undefined, headless: true, args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
    const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } }), errors = [];
    page.on('pageerror', e => errors.push(e.message));
    await page.route(/fonts\.google|goatcounter|gc\.zgo/, r => r.abort());
    if (process.env.THREE_PATH) await page.route(/three\.min\.js/, r => r.fulfill({ path: process.env.THREE_PATH, contentType: 'application/javascript' }));
    await page.addInitScript(MOCK);
    await page.goto('http://127.0.0.1:' + server.address().port, { waitUntil: 'networkidle' });
    await page.waitForFunction(() => typeof brt !== 'undefined' && brt.ready && typeof fleet !== 'undefined' && fleet.ready);
    const commit = JSON.parse(fs.readFileSync(path.join(root, 'firmware/manifest.json'))).commit;
    // The install dialog for the flight core's board; Connect: the board says its build, the same as the page's
    await page.evaluate(c => { running = true; renderRun(); __usb.reply = `firmware ${c} ${boardOf('core').kind === 'esp32' ? 'esp32' : ESP_PROFILES[boardOf('core').kind].chip} flight`; openInstall(boardOf('core')); }, commit);
    await page.click('#instConn');
    await page.waitForFunction(() => /same build/.test(document.querySelector('#instFw').textContent));
    assert.ok(await page.evaluate(() => __usb.lines.some(s => s.includes('version'))), 'it asks the board which build it runs');
    console.log('version: the board\'s build matches the page\'s firmware/manifest.json');
    // Another build, then firmware from before "version"
    const other = await page.evaluate(async () => { __usb.reply = 'firmware 1234567 esp32 flight'; INST.board = null; await instAskVersion(INST.conn); await new Promise(r => setTimeout(r, 100)); return document.querySelector('#instFw').textContent; });
    assert.match(other, /a different build/);
    const old = await page.evaluate(async () => { __usb.reply = 'expected key=value'; INST.board = null; await instAskVersion(INST.conn); await new Promise(r => setTimeout(r, 100)); return document.querySelector('#instFw').textContent; });
    assert.match(old, /before version reporting/);
    console.log('version: another build, and old firmware, ask for an install');
    // The 3D view: the dialog closes, the cable stays, the simulation pauses, the drone takes the board's attitude
    await page.evaluate(() => __usb.telem(0, 0, 0, 0));
    await page.click('#instView');
    await page.waitForFunction(() => usbViewOn() && !document.querySelector('#installDlg').open && !document.querySelector('#usbBar').hidden);
    assert.strictEqual(await page.evaluate(() => running), false, 'the simulation pauses');
    await page.evaluate(() => __usb.telem(30, -10, 45, 0.5));
    await page.waitForFunction(() => { const [w, x, y, z] = S.q; return Math.abs(Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * 180 / Math.PI - 30) < 0.5; });
    await page.waitForFunction(() => /roll 30°/.test(document.querySelector('#usbRead').textContent));   // (the bar: five times a second)
    const seen = await page.evaluate(() => { const [w, x, y, z] = S.q; return { pitch: Math.asin(2 * (w * y - z * x)) * 180 / Math.PI, h: S.p[2] - spawnAt[2], read: document.querySelector('#usbRead').textContent, state: document.querySelector('#usbState').textContent }; });
    assert.ok(Math.abs(seen.pitch + 10) < 0.5 && Math.abs(seen.h - 0.5) < 1e-3, JSON.stringify(seen));
    assert.match(seen.read, /roll 30° pitch -10° yaw 45° · height 0\.50 m/); assert.strictEqual(seen.state, 'connected');
    console.log('view: the drone follows the board\'s roll, pitch and height; the bar reads its telemetry');
    // Console: the same cable and its messages; closing it leaves the view on
    await page.click('#usbConsole'); await page.waitForSelector('#installDlg[open]');
    await page.waitForFunction(() => usbViewOn() && !!INST.conn && document.querySelector('#instConsole').childNodes.length > 0 && /before version reporting/.test(document.querySelector('#instFw').textContent));   // (still the old build's answer)
    await page.click('#installClose');
    assert.ok(await page.evaluate(() => usbViewOn() && !!INST.conn && !document.querySelector('#installDlg').open), 'closing the dialog keeps the view and the cable');
    // Stop: the simulation as it was, the cable still connected
    await page.click('#usbStop');
    assert.deepStrictEqual(await page.evaluate(() => [usbViewOn(), document.querySelector('#usbBar').hidden, running, !!INST.conn]), [false, true, true, true]);
    // Run ends the view and runs
    await page.evaluate(() => { usbViewStart(); running = false; renderRun(); });
    await page.click('#runBtn');
    assert.deepStrictEqual(await page.evaluate(() => [usbViewOn(), running]), [false, true]);
    // Disconnect in the dialog ends it too
    await page.evaluate(() => { usbViewStart(); openInstall(boardOf('core')); });
    await page.click('#instConn');
    assert.deepStrictEqual(await page.evaluate(() => [usbViewOn(), !!INST.conn, document.querySelector('#usbBar').hidden]), [false, false, true]);
    console.log('view: Console, closing the dialog, Stop, Run and Disconnect');
    assert.deepStrictEqual(errors, []);
    console.log('USB board view and firmware version (mock): ok');
  } catch (e) { console.error(e); process.exitCode = 1; }
  finally { if (browser) await browser.close(); server.close(); }
})();
