#!/usr/bin/env node
// Drones finding each other and talking (runner/fc/peer.c in each drone's flight controller, js/peer-air.js between
// them): three drones in one world, each sees the other two connected with their published state; a ping; one flown
// out of range turns stale, then lost, and comes back by itself; a drone in another fleet is never heard; the
// Ground tab lists them. Requires Playwright and Chrome (PLAYWRIGHT_PATH / CHROME_PATH; THREE_PATH: a local
// three.min.js r128, if the page can't fetch it).
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
(async () => {
  await new Promise(r => server.listen(0, '127.0.0.1', r)); let browser;
  try {
    browser = await chromium.launch({ executablePath: process.env.CHROME_PATH || undefined, headless: true, args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
    const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } }), errors = [];
    page.on('pageerror', e => errors.push(e.stack || e.message));
    await page.route(/fonts\.google|goatcounter|gc\.zgo/, r => r.abort());
    if (process.env.THREE_PATH) await page.route(/three\.min\.js/, r => r.fulfill({ path: process.env.THREE_PATH, contentType: 'application/javascript' }));
    await page.goto('http://127.0.0.1:' + server.address().port, { waitUntil: 'networkidle' });
    await page.waitForFunction(() => typeof fleet !== 'undefined' && fleet.ready && brt.ready);
    const r = await page.evaluate(async () => {
      running = false; setTerrain('open', 1);
      const check = (v, m) => { if (!v) throw Error(m); };
      const a = fleet.selected, b = fleetCreate('quadx'), c = fleetCreate('quadx');
      check(a && b && c && fleet.drones.length === 3, 'three drones');
      const fly = async s => { for (let n = 0; n < s * 4; n++) { fleetStep(500); await new Promise(r => setTimeout(r, 0)); } };
      const table = d => withDrone(d, () => { const w = brt.inst.get(peerBoard().id); return peerTable(w, brt.t); });
      const st = (d, e) => { const p = table(d).find(p => p.id === peerId(e)); return p ? PEER_STATES[p.state] : 'none'; };
      await fly(4);
      const tA = table(a), names = tA.map(p => p.name).sort();
      check(tA.length === 2 && tA.every(p => p.state === 3), 'A sees the other two connected: ' + JSON.stringify(tA.map(p => [p.name, p.state])));
      check(st(b, a) === 'connected' && st(c, b) === 'connected', 'every drone sees the others: ' + JSON.stringify([a, b, c].map(d => [d.name, peerId(d), table(d).map(p => [p.name, p.id, p.state])])));
      const pb = tA.find(p => p.id === peerId(b));
      check(pb.vals.length >= 3 && pb.vals[0] === 1 && pb.vals[1] > 50 && pb.vals[2] > 0.5 && pb.lq > 90 && pb.heardUs > 90, 'B publishes armed, battery, height: ' + JSON.stringify(pb));
      withDrone(a, () => peerPing(peerId(c))); await fly(0.5);
      const rtt = table(a).find(p => p.id === peerId(c)).rtt;
      check(rtt > 0 && rtt < 0.05, 'a ping A → C: ' + rtt);
      // C flown away (its target 20 km off: it flies out of range), then home
      const far = () => { c.state.S.p = [3000, 0, 50]; c.state.S.v = [0, 0, 0]; };
      far(); await fly(1.5); const s15 = st(a, c); far(); await fly(2.5); const s4 = st(a, c);
      c.state.S.p = [c.state.setpoint.x, c.state.setpoint.y, c.state.setpoint.z]; await fly(1.5); const back = st(a, c);
      check(s15 === 'stale' && s4 === 'lost' && back === 'connected', `out of range and back: ${s15}, ${s4}, ${back}`);
      // another fleet
      withDrone(c, () => { radioCfg.fleet = 'another fleet'; peerSetup(); }); await fly(4);
      check(st(a, c) !== 'connected' && st(c, a) === 'none', 'another fleet phrase: not connected (' + st(a, c) + ', ' + st(c, a) + ')');
      withDrone(c, () => { radioCfg.fleet = 'liftlab'; peerSetup(); }); await fly(1.5);
      check(st(a, c) === 'connected', 'the same phrase again: connected');
      // the Ground tab's list
      fleetSelect(a.id); UI_PANELS.editor.select('gs'); GS_UI.built = false; renderGs(true);
      const rows = [...document.querySelectorAll('#paneGs .gs-peer')].map(e => e.textContent);
      check(rows.length === 2 && rows.every(t => /connected/.test(t) && /battery/.test(t)), 'Ground tab rows: ' + JSON.stringify(rows));
      const log = radio.log.filter(e => e.kind === 'peer').map(e => e.data);
      check(log.some(x => /connected/.test(x)) && log.some(x => /lost/.test(x)), 'the link log says who came and went: ' + JSON.stringify(log));
      return { names, rtt, rows };
    });
    console.log('three drones found each other:', r.names.join(', '), '· ping', (r.rtt * 1000).toFixed(1), 'ms');
    console.log('out of range: stale, lost, back by itself; another fleet unheard; the Ground tab lists them');
    assert.deepStrictEqual(errors, []);
    console.log('drones talking (simulated): ok');
  } finally { if (browser) await browser.close(); server.close(); }
})().catch(e => { console.error(e); server.close(); process.exitCode = 1; });
