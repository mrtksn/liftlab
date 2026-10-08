#!/usr/bin/env node
// A program image for the native test of programs on the data bus (runner/fc/test_prog.c), compiled as the simulator
// compiles a board's programs (js/programs.js: signature (st, inp, dt), inp a record of the topics read, the result
// the layout of the topic written):
//   node tools/prog_test_data.js /tmp/prog   (writes prog.rnp there)
'use strict';
const fs = require('fs'), path = require('path');
require('./lib');                                   // (the step compiler and runner, as globals)
const dir = process.argv[2] || '.'; fs.mkdirSync(dir, { recursive: true });
const { num, arr, rec, state } = RT;
const S = (inp, ret) => ({ names: ['st', 'inp', 'dt'], args: [state(), inp, num], ret });
const baro = rec({ height: num }), imu = rec({ gyro: arr(num, 3), accel: arr(num, 3) });
const sigs = {
  // on a change of sensor.baro: smooth it, its rate, how many runs
  smooth: S(rec({ baro, imu }), rec({ h: num, rate: num, n: num })),
  // every 100 ms: the time it has run for
  ticker: S(rec({ baro }), rec({ t: num })),
  // gives a number that isn't finite: its runs fail
  broken: S(rec({ baro }), rec({ x: num })),
};
const srcs = {
  smooth: `function smooth(st, inp, dt) {
  if (st.h == null) { st.h = inp.baro.height; st.n = 0; }
  const h = st.h + 0.5 * (inp.baro.height - st.h), rate = (h - st.h) / dt;
  st.h = h; st.n = st.n + 1 + 0 * inp.imu.gyro[0];
  return { h, rate, n: st.n };
}`,
  ticker: `function ticker(st, inp, dt) { if (st.t == null) st.t = 0; st.t = st.t + dt; return { t: st.t }; }`,
  broken: `function broken(st, inp, dt) { return { x: inp.baro.height / 0 }; }`,
};
const P = rnCompileAll(srcs, sigs, { throw: true }); rnVerify(P);
fs.writeFileSync(path.join(dir, 'prog.rnp'), rnImage(P, { tests: [] }));
console.log('programs image in ' + dir);
