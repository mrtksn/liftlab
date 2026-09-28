#!/usr/bin/env node
// Checks the step compiler and both runners against the formulas themselves: every flight formula is compiled,
// then each recorded call (golden.json.gz) runs on the JavaScript runner and on the C runner (WebAssembly, if
// runner/build_wasm.sh has built it). Their results and memory must match the formula's within 32-bit float
// rounding. Also prints each formula's steps per call.
//   node tools/check_formulas.js
'use strict';
const { defaultSources, golden, wasmRunner } = require('./lib');
(async () => {
  const P = rnCompileAll(defaultSources(), RN_SIGS);
  let bad = 0;
  for (const [k, e] of Object.entries(P.errors)) { console.log('does not compile:', k, e); bad++; }
  rnVerify(P);
  const W = await wasmRunner(); if (W) { const e = W.load(P); if (e) { console.log('C runner rejected the program:', e); bad++; } }
  const J = rnArena(P), G = golden();
  // Relative difference with a floor, so tiny values don't dominate; allocation picks between equally good
  // moves, so compare what they make, not each input.
  const diff = (a, b) => {
    if (a == null || b == null) return a == null && b == null ? 0 : Infinity;
    if (typeof a === 'number') return typeof b === 'number' ? (Number.isNaN(a) && Number.isNaN(b) ? 0 : Math.abs(a - b) / Math.max(1e-2, Math.abs(a))) : Infinity;
    if (Array.isArray(a)) return Array.isArray(b) && a.length === b.length ? Math.max(0, ...a.map((x, i) => diff(x, b[i]))) : Infinity;
    return Math.max(0, ...Object.keys(a).filter(k => a[k] !== null).map(k => diff(a[k], b[k])));
  };
  const made = (cols, x) => [0, 1, 2, 3, 4, 5].map(r => cols.reduce((s, c, j) => s + c[r] * x[j], 0));
  const TOL = { identifyEffectiveness: 0.05, positionEstimator: 0.01, allocation: 0.02 };
  console.log('formula                   calls  JS runner  C runner   steps/call');
  for (const [key, samples] of Object.entries(G)) {
    const f = P.fns[key]; if (!f) continue;
    const worst = [0, 0]; let steps = 0;
    for (const s of samples) {
      [J, W && W.A].forEach((A, e) => {
        if (!A) return;
        A.fill(0, P.constEnd);
        f.args.forEach((a, i) => {
          if (a.state) { rnStateIn(A, f, s.args[i], true); for (const st of f.state) if (st.ring && s.args[i][st.name]) { A[st.flag] = 1; rnWrite(A, st.addr, st.t, s.args[i][st.name]); } }
          else rnWrite(A, a.addr, a.t, s.args[i], a.name);
        });
        let n; try { n = e ? W.run(key) : rnRun(P, A, key); } catch (err) { worst[e] = Infinity; return; }
        if (!e) steps += n;
        let ret = rnRead(A, f.ret.addr, f.ret.t), want = s.ret;
        if (key === 'allocation') { ret = made(s.args[0], ret); want = made(s.args[0], want); }
        let d = diff(want, ret);
        if (s.stAfter) { const st = {}; rnStateOut(A, f, st); for (const k of Object.keys(s.stAfter)) if (k in st) d = Math.max(d, diff(s.stAfter[k], st[k])); }
        worst[e] = Math.max(worst[e], d);
      });
    }
    const tol = TOL[key] || 2e-3, ok = worst[0] <= tol && (!W || worst[1] <= tol);
    if (!ok) bad++;
    console.log(`${key.padEnd(24)} ${String(samples.length).padStart(6)}  ${worst[0].toExponential(1).padStart(9)}  ${W ? worst[1].toExponential(1).padStart(8) : '       —'}  ${String(Math.round(steps / samples.length)).padStart(9)}${ok ? '' : '   MISMATCH'}`);
  }
  console.log(bad ? `${bad} problems` : 'all formulas match');
  process.exit(bad ? 1 : 0);
})();
