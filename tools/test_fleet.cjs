#!/usr/bin/env node
// The fleet program in the simulator (runner/fc/fleet_core.c beside each drone's navigation, fleet_link.c at its peer
// end, js/peer-air.js between them): three drones; the pilot flies A; B and C are handed to their fleet programs (the
// default: a formation behind the leader) by the radio's FLEET command, and follow A as it flies, in their places,
// apart; A's program counts their join messages; hold on B gives it back to its pilot; the Ground tab shows it.
// Requires Playwright and Chrome (PLAYWRIGHT_PATH / CHROME_PATH; THREE_PATH: a local three.min.js r128).
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
      const fly = async (s, each) => { for (let n = 0; n < s * 4; n++) { if (each) each(n / 4); fleetStep(500); await new Promise(r => setTimeout(r, 0)); } };
      const view = d => withDrone(d, () => fleetView());
      const pos = d => d.state.S.p.slice();
      await fly(6);
      check([a, b, c].every(d => view(d) && view(d).ok), 'each drone has its fleet program: ' + JSON.stringify([a, b, c].map(view)));
      check(withDrone(b, () => fleetEngage(true)) === '' && withDrone(c, () => fleetEngage(true)) === '', 'the FLEET command goes up');
      await fly(1.5);
      check(view(b).engaged && view(c).engaged && !view(a).engaged, 'B and C engaged by the radio\'s command: ' + JSON.stringify([view(a), view(b), view(c)].map(v => v.engaged)));
      // the pilot flies A: 16 m along x, then 8 along y, in steps of its target
      const a0 = withDrone(a, () => ({ x: setpoint.x, y: setpoint.y }));
      let closest = 1e9, worst = 0;
      const ids = [b, c].map(d => peerId(d) >>> 8), order = ids[0] < ids[1] ? [b, c] : [c, b];
      await fly(36, t => {
        withDrone(a, () => { const s = Math.min(t, 16); setpoint.x = a0.x + s * 0.8; setpoint.y = a0.y + Math.max(0, Math.min(t - 20, 10)) * 0.6; });
        const P = [a, b, c].map(pos);
        for (let i = 0; i < 3; i++) for (let j = i + 1; j < 3; j++) closest = Math.min(closest, Math.hypot(P[i][0] - P[j][0], P[i][1] - P[j][1]));
      });
      // where their places are: behind A (its heading) and to each side
      const hA = withDrone(a, () => brt.navOut.heading), PA = pos(a), places = order.map((d, k) => { const row = Math.floor(k / 2) + 1, side = k % 2 ? -1 : 1, bx = -2 * row, by = 1.5 * row * side;
        return [PA[0] + Math.cos(hA) * bx - Math.sin(hA) * by, PA[1] + Math.sin(hA) * bx + Math.cos(hA) * by]; });
      order.forEach((d, k) => { worst = Math.max(worst, Math.hypot(pos(d)[0] - places[k][0], pos(d)[1] - places[k][1])); });
      check(Math.hypot(PA[0] - a0.x, PA[1] - a0.y) > 12, 'A flew: ' + JSON.stringify([PA, a0]));
      check(worst < 2.5, `B and C in their places behind A after it flew 14 m (worst ${worst.toFixed(2)} m)`);
      check(closest > 1.0, `never nearer each other than ${closest.toFixed(2)} m`);
      const tA = withDrone(b, () => peerTable(brt.inst.get(peerBoard().id), brt.t)).find(p => p.id === peerId(a));
      check(tA && tA.vals[FLEET_HEAD_N] === 1 && tA.vals[FLEET_HEAD_N + 2] === 2, 'A leads, 2 joined (their messages): ' + JSON.stringify(tA && tA.vals));
      // the Ground tab: A's view lists B and C as flown by their programs
      fleetSelect(a.id); UI_PANELS.editor.select('gs'); GS_UI.built = false; renderGs(true);
      const rows = [...document.querySelectorAll('#paneGs .gs-peer')].map(e => e.textContent);
      check(rows.length === 2 && rows.every(t => /fleet program flies it/.test(t) && /program: 2/.test(t)), 'Ground tab rows: ' + JSON.stringify(rows));
      const st = document.querySelector('#gsFleetState').textContent;
      check(/not flying it/.test(st) && /publishes 1, 0, 2/.test(st), 'A\'s own fleet program: ' + st);
      // hold on B: back to its pilot, where it is
      withDrone(b, () => radioHold()); await fly(1.5);
      check(!view(b).engaged && view(c).engaged, 'hold on B gives it back to its pilot; C flies on');
      const pb = pos(b); await fly(3); const moved = Math.hypot(pos(b)[0] - pb[0], pos(b)[1] - pb[1]);
      check(moved < 0.6, `B holds where it is (moved ${moved.toFixed(2)} m)`);
      return { worst, closest, moved, st };
    });
    console.log(`formation: worst ${r.worst.toFixed(2)} m from its place; closest ${r.closest.toFixed(2)} m; after hold B moved ${r.moved.toFixed(2)} m`);
    console.log('A\'s program:', r.st);
    assert.deepStrictEqual(errors, []);
    console.log('fleet program (simulated): ok');
  } finally { if (browser) await browser.close(); server.close(); }
})().catch(e => { console.error(e); server.close(); process.exitCode = 1; });
