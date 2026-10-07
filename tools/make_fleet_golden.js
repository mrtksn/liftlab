#!/usr/bin/env node
// Adds calls of the fleet program (js/laws.js fleetProgram) to tools/golden.json.gz, for check_formulas.js: the
// fleet isn't flown in the recorded flights, so these are made here, scenario by scenario (a drone alone, leading,
// following in each place, too close to another, without a shared frame, messages), each call on the memory the
// last left, as the drone runs it.   node tools/make_fleet_golden.js
'use strict';
const fs = require('fs'), path = require('path'), zlib = require('zlib'), vm = require('vm');
const JS = path.join(__dirname, '..', 'js');
const ctx = vm.createContext({ Math, Array, Object, Number, JSON });
for (const f of ['math.js', 'laws.js']) vm.runInContext(fs.readFileSync(path.join(JS, f), 'utf8') + '\n;', ctx, { filename: f });
const fleetProgram = vm.runInContext('fleetProgram', ctx);
const file = path.join(__dirname, 'golden.json.gz'), G = JSON.parse(zlib.gunzipSync(fs.readFileSync(file)).toString());
const clone = x => JSON.parse(JSON.stringify(x));
let seed = 7; const rnd = () => { seed = (seed * 1103515245 + 12345) % 2147483648; return seed / 2147483648; };
const drone = (id, o) => ({ id, link: 3, lq: 100, age: 0.1, flying: 1, battery: 80, p: [0, 0, 1.5], v: [0, 0, 0], heading: 0, engaged: 0, vals: [], ...o });
const calls = [];
const run = (st, me, others, msg) => { const args = clone([st, me, others, msg, 0.1]); const ret = fleetProgram(st, me, others, msg, 0.1); calls.push({ args, ret: clone(ret), stAfter: clone(st) }); return ret; };
// alone, then a leader appears, then this drone is engaged and follows for a while (the leader turning), another
// follower comes close, the link to the leader goes stale, then lost
{ const st = {}; let me = { id: 9, p: [0, 0, 0], v: [0, 0, 0], heading: 0, flying: 0, battery: 95, shared: 1, engaged: 0, t: 0 };
  run(st, me, [], []);
  run(st, { ...me, flying: 1, p: [0, 0, 1.5] }, [drone(4, { p: [3, 2, 2] })], []);
  for (let k = 0; k < 12; k++) {
    const h = 0.25 * k, lp = [3 + k * 0.3, 2 + Math.sin(k) * 0.5, 2 + 0.05 * k];
    me = { ...me, flying: 1, engaged: 1, t: 1 + 0.1 * k, p: [lp[0] - 2 + rnd(), lp[1] + 1 + rnd(), 1.8], v: [0.3, rnd() - 0.5, 0], heading: h };
    const others = [drone(4, { p: lp, v: [0.3, 0.1, 0], heading: h, vals: [1, 0, k] }), drone(12, { engaged: 1, p: [me.p[0] + (k > 6 ? 0.4 : 3), me.p[1] + 0.3, 1.8], vals: [2, 1, 4] })];
    if (k === 10) others[0].link = 2;
    run(st, me, others, k === 3 ? [{ from: 12, v: [1, 1] }] : []);
  }
  run(st, { ...me, t: 3 }, [drone(4, { link: 0, p: [5, 2, 2] })], []);
  run(st, { ...me, engaged: 0, t: 3.1 }, [drone(4, { p: [5, 2, 2] })], []);
}
// the leader: followers publish that they follow it, and tell it when they join
{ const st = {}, me = { id: 4, p: [1, 1, 2], v: [0.5, 0, 0], heading: 0.4, flying: 1, battery: 70, shared: 1, engaged: 0, t: 5 };
  for (let k = 0; k < 4; k++) {
    const others = [drone(9, { engaged: 1, vals: [2, 0, 4], p: [-1, 2, 2] }), drone(12, { engaged: 1, vals: [2, 1, 4], p: [-1, 0, 2] }), drone(20, { flying: 0, p: null })];
    run(st, { ...me, t: 5 + 0.1 * k }, others.slice(0, 1 + (k % 3)), k < 2 ? [{ from: 9 + 3 * k, v: [1, k] }] : []);
  }
}
// places 0–5 behind a leader heading west; and without a shared frame
for (let place = 0; place < 6; place++) {
  const others = [drone(2, { p: [10, -3, 3], heading: Math.PI, v: [-1, 0, 0] })];
  for (let j = 0; j < place; j++) others.push(drone(3 + j, { engaged: 1, p: [12 + j, -3 + j, 3] }));
  run({ joins: 2, told: 1 }, { id: 50, p: [11, -2, 3], v: [0, 0, 0], heading: 3, flying: 1, battery: 60, shared: 1, engaged: 1, t: 20 }, others, []);
}
run({}, { id: 50, p: [0, 0, 1], v: [0, 0, 0], heading: 0, flying: 1, battery: 60, shared: 0, engaged: 1, t: 1 }, [drone(2, { p: null })], []);
G.fleetProgram = calls;
fs.writeFileSync(file, zlib.gzipSync(JSON.stringify(G)));
console.log(`golden.json.gz: ${calls.length} fleet program calls`);
