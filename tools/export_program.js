#!/usr/bin/env node
// Builds a program image for the drone from the flight formulas: the defaults in js/laws.js, with any edits.
// It carries self-tests (real inputs recorded in simulated flights, with the outputs this program gives) that the
// drone runs when it loads it.
//   node tools/export_program.js --out flight.rnp [--tasks core,nav] [--edit attitudeControl=my_attitude.js …] [--tune gains.json] [--c runner/rn_builtin.c] [--link]
// --tasks: the board's tasks (core, nav, learn, super, or ground for the command module); its program has their formulas. Default core,nav (the ESP32's
// built-in program); the Pi's is nav,learn,super (runner/pi/rn_builtin_pi.c).
// --c also writes the image as C, to compile into the firmware as its built-in program (--sym NAME: its C names,
// NAME_img and NAME_len; rn_builtin by default).
// --tune: the controller's gains, as a design has them (its "tuning": { att: { kR, kW, kI }, pos: { kp, kd, ki } });
// the defaults otherwise (js/laws.js TUNE_DEFAULTS). Fields left out keep their defaults.
// --link keeps the image small (under 48 KB) for sending over the serial link: only the small self-tests.
// The simulator's Formulas tab makes the same file ("Download program for the drone"), with your edits.
'use strict';
const fs = require('fs'), path = require('path');
const { defaultSources, defaultConsts, golden, lawSample } = require('./lib');
const args = process.argv.slice(2), opt = (n) => { const i = args.indexOf(n); return i >= 0 ? args[i + 1] : null; };
const all = defaultSources(), keys = rnTaskFormulas((opt('--tasks') || 'core,nav').split(','));
const srcs = Object.fromEntries(keys.map(k => [k, all[k]]));
args.forEach((a, i) => { if (a === '--edit') { const [k, f] = args[i + 1].split('='); if (!(k in srcs)) throw new Error('no flight formula ' + k + ' in these tasks'); srcs[k] = fs.readFileSync(f, 'utf8'); } });
const consts = defaultConsts();
if (opt('--tune')) { const t = JSON.parse(fs.readFileSync(opt('--tune'), 'utf8')); for (const g of ['att', 'pos']) Object.assign(consts.TUNE[g], (t.tuning || t)[g] || {}); }
const P = rnCompileAll(srcs, RN_SIGS, { consts });
for (const [k, e] of Object.entries(P.errors)) { console.error(`${k}: ${e}`); process.exit(1); }
rnVerify(P);
// Two self-tests per formula, from the smallest recorded calls (the image goes over a serial link).
const G = golden(), samples = [];
for (const [key, ss] of Object.entries(G)) if (keys.includes(key)) samples.push(...ss.map(s => ({ key, args: s.args, n: JSON.stringify(s.args).length })).sort((a, b) => a.n - b.n).slice(0, 2));
for (const key of keys) if (!G[key]) { const a = lawSample(key); if (a) samples.push({ key, args: a }); }   // (no recorded calls: the formula's own sample)
const img = rnImage(P, { tests: rnMakeTests(P, samples, args.includes('--link') ? 600 : Infinity) });
const out = opt('--out') || 'flight.rnp';
fs.writeFileSync(out, img);
console.log(`${out}: ${img.length} bytes, ${Object.keys(P.fns).length} formulas, arena ${P.arenaSize * 4} bytes, ${P.code.length * 4} bytes of steps`);
const c = opt('--c');
const sym = opt('--sym') || 'rn_builtin';   // the C names (the command module's: rn_builtin_ground)
if (c) {
  const words = [], dv = new DataView(img.buffer, img.byteOffset, img.length);
  for (let i = 0; i < img.length; i += 4) words.push('0x' + dv.getUint32(i, true).toString(16).padStart(8, '0') + 'u');
  const lines = []; for (let i = 0; i < words.length; i += 8) lines.push('  ' + words.slice(i, i + 8).join(', ') + ',');
  fs.writeFileSync(c, `/* The built-in program: the default formulas of its tasks, compiled by tools/export_program.js. Generated; do not edit. */
#include <stdint.h>
/* 32-bit words, so the image is aligned and its steps can run straight from flash (rn_load with code = NULL).
 * Stored little-endian, as the ESP32 is. */
const uint32_t ${sym}_words[] = {
${lines.join('\n')}
};
const uint8_t *const ${sym}_img = (const uint8_t *)${sym}_words;
const uint32_t ${sym}_len = ${img.length}u;
`);
  console.log(`${c}: the same image as C`);
}
