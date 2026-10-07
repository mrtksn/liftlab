#!/usr/bin/env node
// Images and a call sequence for the native test of the drone's loading steps (runner/test_rnhost.c):
//   node tools/host_test_data.js /tmp/rn   (writes builtin.rnp edit.rnp nan.rnp trap.rnp sig.rnp calls.bin there)
'use strict';
const fs = require('fs'), path = require('path');
const { defaultSources, defaultConsts, golden } = require('./lib');
const dir = process.argv[2] || '.'; fs.mkdirSync(dir, { recursive: true });
const G = golden(), all = defaultSources(), def = Object.fromEntries(rnTaskFormulas(['core', 'nav']).map(k => [k, all[k]]));   // the ESP32's program
const samples = []; for (const [key, ss] of Object.entries(G)) samples.push(...ss.slice(0, 2).map(s => ({ key, args: s.args })));
function image(srcs, sigs = RN_SIGS, tests = true) {
  const P = rnCompileAll(srcs, sigs, { throw: true, consts: defaultConsts() }); rnVerify(P);
  return { P, img: rnImage(P, { tests: tests ? rnMakeTests(P, samples) : [] }) };
}
const w = (f, b) => fs.writeFileSync(path.join(dir, f), b);
const B = image(def); w('builtin.rnp', B.img);
const ac = def.attitudeControl;
w('edit.rnp', image({ ...def, attitudeControl: ac.replace('kR = TUNE.att.kR', 'kR = [130, 130, 40]') }).img);
w('nan.rnp', image({ ...def, attitudeControl: ac.replace('return add(', 'if (Math.abs(w[0]) + Math.abs(w[1]) + Math.abs(w[2]) > 0) return [0 / 0, 0, 0];\n  return add(') }, RN_SIGS, false).img);
w('trap.rnp', image({ ...def, servoPredictor: def.servoPredictor.replace('if (st.h == null)', 'st.n = (st.n || 0) + 1; if (st.n > 1600) { const a = [1, 2]; return a[Math.round(st.n)]; }\n  if (st.h == null)') }).img);
const sig2 = { ...RN_SIGS, thrustLinearization: { ...RN_SIGS.thrustLinearization, args: [RT.num, RT.arr(RT.num, 2)] } };
w('sig.rnp', image({ ...def, thrustLinearization: '(v, bend) => clamp(v, 0, 1)' }, sig2, false).img);
// Every formula once per control step (servoPredictor for 4 servos), recorded inputs in turn. The learning
// formula keeps one layout's inputs (its memory is sized by the first call).
const P = B.P, keys = Object.keys(P.fns), A = rnArena(P), out = [];
for (let step = 0; step < 5000; step++) {
  for (const [fi, key] of keys.entries()) {
    const f = P.fns[key], ss = key === 'identifyEffectiveness' ? G[key].filter(s => s.args[1].length === G[key][0].args[1].length && !!s.args[9] === !!G[key][0].args[9]) : G[key];
    if (!ss || !ss.length) continue;
    const s = ss[step % ss.length];
    for (let inst = 0; inst < (key === 'servoPredictor' ? 4 : 1); inst++) {
      const flat = [];
      f.args.forEach((a, i) => { if (a.state) return; A.fill(0, a.addr, a.addr + tsize(a.t)); rnWrite(A, a.addr, a.t, s.args[i], a.name); flat.push(...A.subarray(a.addr, a.addr + tsize(a.t))); });
      out.push(fi, inst, flat.length, ...new Int32Array(Float32Array.from(flat).buffer));
    }
  }
  out.push(-1, -1, -1);
}
w('calls.bin', Buffer.from(Int32Array.from(out).buffer));
console.log(`test data in ${dir}`);
