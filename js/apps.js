'use strict';
// Apps (docs/apps.md): your own code in C or Python, on the boards that run apps.
//
// An app is part of the design (cfg.apps): a header like a program's (its name, when it runs, the topics it reads, the
// topic it writes) and its code. A board says what it runs (its settings: formulas, WebAssembly apps, native apps) and
// which apps are on it (board.apps). Its kind decides where it can go:
//   wasm    C, compiled here into WebAssembly (clang, in the browser) and run by the board's app host: an ESP32 or a Pi
//           that runs WebAssembly apps, and the simulator, the same module;
//   native  C compiled on the Pi itself, so it can use Linux's libraries; the same calls as a WebAssembly app;
//   python  Python on the Pi.
// Native and Python apps aren't simulated (their topics stay empty here).
//
// The code: liftlab.h (made from the header) gives struct inputs (the topics it reads, each by the last part of its
// name, its fields by its layout) and struct output (the fields of the topic it writes). You write
//   void setup(void)                                                  once at the start (optional)
//   int step(const struct inputs *in, struct output *out, float dt)  when it's due; 1 publishes out, 0 doesn't
// printf goes to the board's log; board_time() is the board's clock [s]. The board's app host calls step as
// runner/fc/prog_core.c calls a program: when a topic it reads changes, or every period, once every topic it reads has
// a value. A loop is limited (wasm-meter.js): one that never ends stops the run. A run that traps or runs too long
// publishes nothing and is counted, and the app starts again (setup, its memory fresh).

const APP_MAX = 32, APP_FUEL = 5e6;   // apps in a design; loop turns one step may take
const APP_KINDS = {
  wasm: { label: 'C · WebAssembly', runtime: 'wasm', lang: 'c', note: 'Compiled here; runs on an ESP32 or a Pi that runs WebAssembly apps, and in the simulator.' },
  native: { label: 'C · native (Pi)', runtime: 'native', lang: 'c', note: 'Compiled on the Pi, so it can use Linux\'s libraries. Not simulated.' },
  python: { label: 'Python (Pi)', runtime: 'native', lang: 'python', note: 'Runs on the Pi. Not simulated.' },
};
const APP_CLANG = 'https://cdn.jsdelivr.net/npm/@yowasp/clang@22.0.0-git20542-10/gen/bundle.js';
const apps = () => cfg.apps || (cfg.apps = []);
const appById = id => apps().find(a => a.id === id) || null;
const appBoard = a => a ? computers().boards.find(b => (b.apps || []).includes(a.id)) || null : null;
const appsOn = b => (b && b.apps || []).map(appById).filter(a => a && boardRuns(b).includes(APP_KINDS[a.kind].runtime));
const appKey = a => [a.kind, a.src, a.name, a.every, a.on, a.reads, a.writes];

// What's wrong with an app's header, or ''.
function appProblem(a, list = apps()) {
  if (!a || !APP_KINDS[a.kind]) return 'Choose what it\'s written in.';
  if (typeof a.name !== 'string' || !/^[A-Za-z_]\w{0,30}$/.test(a.name)) return 'Its name must be a word of letters, digits and _ (31 at most), starting with a letter.';
  if (list.some(q => q !== a && q.id !== a.id && q.name === a.name) || programs().some(p => p.name === a.name)) return `Another app or program is called “${a.name}”.`;
  if (!a.writes || !/^user\.[A-Za-z0-9_][A-Za-z0-9_.]{0,17}$/.test(a.writes.topic)) return 'It writes a topic under user. (up to 23 characters: letters, digits, _ and .).';
  if (!layoutFields(a.writes.layout)) return 'Its topic\'s fields: names separated by spaces, a count in brackets for a list (range rate ok, or v[3]); 32 numbers at most.';
  if (list.some(q => q !== a && q.id !== a.id && q.writes && q.writes.topic === a.writes.topic) || programs().some(p => p.writes.topic === a.writes.topic)) return `Another app or program writes ${a.writes.topic}.`;
  if (!Array.isArray(a.reads) || a.reads.length > PROG_READS) return 'It reads 8 topics at most.';
  const cat = busCatalog(a);
  for (const r of a.reads) { if (r === a.writes.topic) return 'It can\'t read the topic it writes.'; if (!cat.some(t => t.name === r)) return `Nothing on this drone publishes ${r}.`; }
  if (a.reads.reduce((s, r) => s + cat.find(t => t.name === r).n, 0) > 256) return 'Its inputs are too big (256 numbers at most).';
  if (!(a.every > 0) && !a.reads.includes(a.on)) return 'It runs when a topic it reads changes: choose which, or run it every so often.';
  if (a.every > 0 && !(a.every >= 0.001 && a.every <= 60)) return 'Every 1 ms to 60 s.';
  return '';
}
function fixApps(list) {
  if (!Array.isArray(list)) return [];
  return list.slice(0, APP_MAX).filter(a => a && typeof a === 'object').map((a, i) => ({
    id: typeof a.id === 'string' && a.id ? a.id.slice(0, 16) : 'a' + i, kind: APP_KINDS[a.kind] ? a.kind : 'wasm', name: String(a.name || 'app' + (i + 1)).slice(0, 31),
    every: Number.isFinite(+a.every) && +a.every > 0 ? +a.every : 0, on: typeof a.on === 'string' ? a.on : '', reads: Array.isArray(a.reads) ? a.reads.filter(r => typeof r === 'string').slice(0, PROG_READS) : [],
    writes: { topic: String(a.writes && a.writes.topic || 'user.out'), layout: String(a.writes && a.writes.layout || 'v') }, src: String(a.src || ''),
    ...(typeof a.bin === 'string' && typeof a.binOf === 'string' ? { bin: a.bin, binOf: a.binOf } : {}),
  }));
}
// The header's part that the board's wiring depends on: changing it starts the flight again.
const appHeader = a => JSON.stringify([a.kind, a.name, a.every, a.every > 0 ? '' : a.on, a.reads, a.writes]);

/* ───────── liftlab.h ───────── */
const C_WORDS = new Set('auto break case char const continue default do double else enum extern float for goto if inline int long register restrict return short signed sizeof static struct switch typedef union unsigned void volatile while in out dt'.split(' '));
// liftlab.h for a C app, from its header: what its code gets and returns, and the glue its board's app host calls.
function appHeaderC(a, cat = busCatalog(a)) {
  const al = progAliases(a.reads), fields = layout => (layoutFields(layout) || []).map(([f, k, arr]) => `float ${f}${arr ? `[${k}]` : ''};`).join(' ');
  const L = [`/* liftlab.h: made by LiftLab from ${a.name}'s header. Don't edit it: change the header in the app manager. */`,
    '#ifndef LIFTLAB_H', '#define LIFTLAB_H', '#include <math.h>', '#include <stdio.h>', '#include <string.h>', '',
    `/* What it reads (in the order of its header), as its board last heard each: */`, 'struct inputs {'];
  a.reads.forEach((r, i) => { const t = cat.find(t => t.name === r); if (t) L.push(`  struct { ${fields(t.layout)} } ${al[i]};   /* ${r}: ${t.board.name}, ${t.from} */`); });
  L.push('};', `/* What it writes: ${a.writes.topic} */`, `struct output { ${fields(a.writes.layout)} };`, '',
    a.every > 0 ? `/* step runs every ${+(a.every * 1000).toFixed(3)} ms, once everything it reads has a value. */` : `/* step runs when ${a.on || 'a topic it reads'} changes, once everything it reads has a value. */`,
    'void setup(void);                                                 /* yours: once at the start (optional) */',
    'int step(const struct inputs *in, struct output *out, float dt);  /* yours: 1 publishes out, 0 doesn\'t */',
    '', '/* From the board: its clock [s] (printf goes to its log). */',
    '__attribute__((import_module("env"), import_name("ll_time"))) double board_time(void);',
    '', '/* The glue its app host calls. */',
    'static struct inputs ll_in_; static struct output ll_out_;',
    '__attribute__((weak)) void setup(void);',
    '__attribute__((export_name("ll_setup"))) void ll_setup(void) { if (setup) setup(); }',
    '__attribute__((export_name("ll_step"))) int ll_step(float dt) { return step(&ll_in_, &ll_out_, dt); }',
    '__attribute__((export_name("ll_in"))) void *ll_in(void) { return &ll_in_; }',
    '__attribute__((export_name("ll_out"))) void *ll_out(void) { return &ll_out_; }',
    '__attribute__((export_name("ll_in_n"))) int ll_in_n(void) { return (int)(sizeof ll_in_ / sizeof(float)); }',
    '__attribute__((export_name("ll_out_n"))) int ll_out_n(void) { return (int)(sizeof ll_out_ / sizeof(float)); }',
    '#endif', '');
  return L.join('\n');
}
// What a C field name can't be (it would not compile): a C word.
function appFieldsProblem(a, cat = busCatalog(a)) {
  if (APP_KINDS[a.kind].lang !== 'c') return '';
  const al = progAliases(a.reads), names = [...(layoutFields(a.writes.layout) || []).map(f => f[0]), ...al];
  for (const r of a.reads) { const t = cat.find(t => t.name === r); if (t) names.push(...(layoutFields(t.layout) || []).map(f => f[0])); }
  const bad = names.find(n => C_WORDS.has(n) || /^\d/.test(n));
  return bad ? `“${bad}” can't be a name in C: rename the field or the topic.` : '';
}
// What a Python app's step gets and returns (its header, as the Pi's app host passes it).
function appShapePython(a, cat = busCatalog(a)) {
  const al = progAliases(a.reads), out = layoutFields(a.writes.layout);
  const inp = a.reads.map((r, i) => { const t = cat.find(t => t.name === r); return t ? (layoutFields(t.layout) || []).map(([f, k, arr]) => `inp.${al[i]}.${f}${arr ? `[0…${k - 1}]` : ''}`).join(', ') : ''; }).filter(Boolean).join('\n') || '(nothing: it reads no topics)';
  return `def step(inp, out, dt)\n\nreads:\n${inp}\n\nsets: ${out ? out.map(([f, k, arr]) => arr ? `out.${f}[0…${k - 1}]` : 'out.' + f).join(', ') : '…'}  → ${a.writes.topic}\nreturns True to publish`;
}
const APP_TEMPLATE = {
  c: n => `#include "liftlab.h"

// Once, when its board starts. (You can leave it out.)
void setup(void) {
  printf("${n} started\\n");
}

// When it's due: read in->…, fill out->…; return 1 to publish out, 0 not to.
int step(const struct inputs *in, struct output *out, float dt) {
  out->v = 0;
  return 1;
}
`,
  python: n => `# Runs on the Pi, in its own process beside the board's data bus.

def setup():
    print("${n} started")

# When it's due: read inp.…, set out.…; return True to publish out.
def step(inp, out, dt):
    out.v = 0
    return True
`,
};

/* ───────── compiling (clang in the browser, in a worker) ───────── */
const appCompiler = { worker: null, next: 1, wait: new Map(), progress: null };
const APP_WORKER = `let clang = null;
onmessage = async e => {
  const { id, url, args, files } = e.data;
  let err = '';
  const text = b => typeof b === 'string' ? b : b ? new TextDecoder().decode(b) : '';
  try {
    clang = clang || await import(url);
    const out = await clang.runClang(args, files, { stderr: b => { err += text(b); }, stdout: b => { err += text(b); },
      fetchProgress: ev => postMessage({ id, progress: [ev.doneLength, ev.totalLength] }) });
    postMessage({ id, ok: true, wasm: out['app.wasm'], err });
  } catch (x) { postMessage({ id, ok: false, err: err || String(x && x.message || x) }); }
};`;
function appClang(args, files, onProgress) {
  if (!appCompiler.worker) {
    appCompiler.worker = new Worker(URL.createObjectURL(new Blob([APP_WORKER], { type: 'text/javascript' })), { type: 'module' });
    appCompiler.worker.onmessage = e => { const w = appCompiler.wait.get(e.data.id); if (!w) return; if (e.data.progress) { w.onProgress && w.onProgress(e.data.progress); return; } appCompiler.wait.delete(e.data.id); w.resolve(e.data); };
    appCompiler.worker.onerror = e => { for (const w of appCompiler.wait.values()) w.resolve({ ok: false, err: 'the compiler didn\'t start: ' + (e.message || 'no network?') }); appCompiler.wait.clear(); appCompiler.worker = null; };
  }
  const id = appCompiler.next++;
  return new Promise(resolve => { appCompiler.wait.set(id, { resolve, onProgress }); appCompiler.worker.postMessage({ id, url: window.APP_CLANG_URL || APP_CLANG, args, files }); });
}
// What an app's module may ask its board for: the board's clock, and what printf needs (wasi-libc's stdio).
const APP_IMPORTS = { env: ['ll_time'], wasi_snapshot_preview1: ['fd_write', 'fd_close', 'fd_seek', 'fd_fdstat_get', 'clock_time_get', 'random_get', 'proc_exit', 'environ_get', 'environ_sizes_get', 'args_get', 'args_sizes_get'] };
const appBinKey = (a, h) => String(rnCrc32(new TextEncoder().encode(a.src + '\u0000' + h)));
const appB64 = u => { let s = ''; for (let i = 0; i < u.length; i += 0x8000) s += String.fromCharCode.apply(null, u.subarray(i, i + 0x8000)); return btoa(s); };
const unb64 = s => Uint8Array.from(atob(s), c => c.charCodeAt(0));
// Compile a WebAssembly app: { bin, binOf, size } or { err } (what clang said, or why the module won't do).
async function appCompile(a, onProgress) {
  const h = appHeaderC(a);
  const r = await appClang(['clang', '--target=wasm32-wasip1', '-mexec-model=reactor', '-O2', '-std=c11', '-Wall', '-Wextra', '-Wno-unused-parameter', '-o', 'app.wasm', 'app.c',
    '-Wl,-z,stack-size=8192', '-Wl,--initial-memory=131072', '-Wl,--max-memory=262144', '-Wl,--strip-all'], { 'app.c': a.src, 'liftlab.h': h }, onProgress);
  const warn = (r.err || '').trim();
  if (!r.ok || !r.wasm) return { err: warn || 'it didn\'t compile' };
  let bin; try { bin = wasmMeter(r.wasm); } catch (e) { return { err: e.message } }
  let m; try { m = await WebAssembly.compile(bin); } catch (e) { return { err: 'not a module the boards can run: ' + e.message }; }
  for (const i of WebAssembly.Module.imports(m)) if (!(APP_IMPORTS[i.module] || []).includes(i.name)) return { err: `It uses ${i.module === 'env' ? i.name : i.name + ' (' + i.module + ')'}, which the boards don't offer.` };
  const ex = new Set(WebAssembly.Module.exports(m).map(e => e.name));
  for (const n of ['memory', 'll_step', 'll_setup', 'll_in', 'll_out', 'll_fuel']) if (!ex.has(n)) return { err: 'it has no ' + n + ' (is liftlab.h included?)' };
  return { bin: appB64(bin), binOf: appBinKey(a, h), size: bin.length, warn };
}
// Its compiled module, if it's current (compiled from this code and this header).
const appModules = new Map();
function appModule(a) {
  if (a.kind !== 'wasm' || !a.bin || a.binOf !== appBinKey(a, appHeaderC(a))) return null;
  let m = appModules.get(a.binOf);
  if (!m) { m = new WebAssembly.Module(unb64(a.bin)); if (appModules.size > 32) appModules.clear(); appModules.set(a.binOf, m); }
  return m;
}

/* ───────── in the simulator: the boards' app hosts ───────── */
// An app's instance: its module with what it asks the board for. Throws if setup fails.
function appInstance(a, m, log) {
  let x = null, seed = 0x2545f491;
  const mem = () => x.memory.buffer, line = { s: '' };
  const say = s => { line.s += s; let k; while ((k = line.s.indexOf('\n')) >= 0) { log(line.s.slice(0, k)); line.s = line.s.slice(k + 1); } if (line.s.length > 200) { log(line.s); line.s = ''; } };
  const wasi = {
    fd_write: (fd, iov, n, nw) => { const dv = new DataView(mem()); let t = 0; for (let i = 0; i < n; i++) { const p = dv.getUint32(iov + 8 * i, true), l = dv.getUint32(iov + 8 * i + 4, true); say(new TextDecoder().decode(new Uint8Array(mem(), p, l))); t += l; } dv.setUint32(nw, t, true); return 0; },
    fd_close: () => 8, fd_seek: () => 8, fd_fdstat_get: (fd, p) => { new Uint8Array(mem(), p, 24).fill(0); new Uint8Array(mem())[p] = 2; return 0; },
    clock_time_get: (id, prec, p) => { new DataView(mem()).setBigUint64(p, BigInt(Math.round(brt.t * 1e9)), true); return 0; },
    random_get: (p, n) => { const u = new Uint8Array(mem(), p, n); for (let i = 0; i < n; i++) { seed ^= seed << 13; seed ^= seed >>> 17; seed ^= seed << 5; u[i] = seed & 255; } return 0; },
    proc_exit: c => { throw new Error('it exited (' + c + ')'); },
    environ_get: () => 0, environ_sizes_get: (a, b) => { new DataView(mem()).setUint32(a, 0, true); new DataView(mem()).setUint32(b, 0, true); return 0; },
    args_get: () => 0, args_sizes_get: (a, b) => { new DataView(mem()).setUint32(a, 0, true); new DataView(mem()).setUint32(b, 0, true); return 0; },
  };
  x = new WebAssembly.Instance(m, { env: { ll_time: () => brt.t }, wasi_snapshot_preview1: wasi }).exports;
  x.ll_fuel.value = APP_FUEL; if (x._initialize) x._initialize();
  x.ll_fuel.value = APP_FUEL; x.ll_setup();
  return x;
}
// At each start, after the programs (boards.js busSetup): every board's apps, their topics on its bus.
function appRegister() {
  brt.appAt = new Map(); brt.appErr = new Map(); brt.appHost = new Map(); brt.appLog = brt.appLog || new Map();
  for (const a of apps()) { const why = appProblem(a); if (why) brt.appErr.set(a.id, why); }
  for (const b of computers().boards) {
    const w = brt.inst.get(b.id); if (!w) continue;
    const host = [];
    for (const a of appsOn(b)) {
      if (brt.appErr.has(a.id)) continue;
      if (a.kind !== 'wasm') { brt.appErr.set(a.id, 'not simulated: it runs on the Pi'); continue; }
      let m; try { m = appModule(a); } catch (e) { brt.appErr.set(a.id, 'its module doesn\'t load: ' + e.message); continue; }
      if (!m) { brt.appErr.set(a.id, 'not compiled: apply it in the app manager'); continue; }
      const log = s => appLogLine(a.id, s);
      let x; try { x = appInstance(a, m, log); } catch (e) { brt.appErr.set(a.id, 'its setup stopped: ' + appFault(e)); continue; }
      busTxt3(w, a.name, a.writes.topic, a.writes.layout);
      const i = w.prog_add_app(layoutSize(a.writes.layout), a.every > 0 ? a.every : 0, x.ll_in_n() + 1);
      if (i < 0) { brt.appErr.set(a.id, cstr(w, w.prog_why_ptr(), 96)); continue; }
      host[i] = { a, m, x, log, restarts: 0, why: '' };   // (i: its index among the board's programs, which prog_core passes back)
      brt.appAt.set(a.id, { b, w, i });
    }
    if (host.length) brt.appHost.set(b.id, host);
  }
}
const appFault = e => e instanceof WebAssembly.RuntimeError ? (/unreachable/.test(e.message) ? 'a trap (or a loop that ran too long)' : e.message) : e instanceof RangeError ? 'it ran out of stack' : (e && e.message) || String(e);
function appLogLine(id, s) { const L = brt.appLog.get(id) || []; L.push({ t: brt.t, s }); if (L.length > 40) L.splice(0, L.length - 40); brt.appLog.set(id, L); }
// prog_core calls an app (board_wasm.c app_call): its inputs into its memory, its step, its result back. 1 publish,
// 0 don't, −1 a trap, −2 it ran too long; after either it starts again.
function appCall(boardId, i, inP, nIn, outP, nOut) {
  const host = brt.appHost && brt.appHost.get(boardId), h = host && host[i], w = brt.inst.get(boardId);
  if (!h || !w) return -1;
  const x = h.x;
  try {
    const src = new Float32Array(w.memory.buffer, inP, nIn);
    new Float32Array(x.memory.buffer, x.ll_in(), nIn - 1).set(src.subarray(0, nIn - 1));
    x.ll_fuel.value = APP_FUEL;
    if (!x.ll_step(src[nIn - 1])) return 0;
    new Float32Array(w.memory.buffer, outP, nOut).set(new Float32Array(x.memory.buffer, x.ll_out(), nOut));
    return 1;
  } catch (e) {
    const long = x.ll_fuel.value === 0;
    h.why = long ? 'a loop ran too long' : appFault(e); h.log('stopped: ' + h.why + '; starting it again');
    h.restarts++;
    try { h.x = appInstance(h.a, h.m, h.log); } catch (e2) { h.why = 'its setup stopped: ' + appFault(e2); }
    return long ? -2 : -1;
  }
}
// An app's new code, in flight: the same header (so the same inputs and topic), a new module, started fresh.
function appReload(a) {
  const at = brt.appAt && brt.appAt.get(a.id), host = at && brt.appHost.get(at.b.id), h = host && host[at.i];
  if (!h) return false;
  let m; try { m = appModule(a); } catch (e) { return false; }
  if (!m) return false;
  try { const x = appInstance(a, m, h.log); if (x.ll_in_n() !== h.x.ll_in_n()) return false; h.x = x; h.m = m; h.a = a; h.why = ''; h.log('loaded the new code'); return true; } catch (e) { return false; }
}
// How each app is doing: { ok, runs, fails, waits, err, restarts, why }.
function appStats() {
  const out = progStats(brt.appAt);
  for (const [id, s] of out) { const at = brt.appAt.get(id), h = (brt.appHost.get(at.b.id) || [])[at.i]; if (h) { s.restarts = h.restarts; s.why = h.why; } }
  return out;
}

/* ───────── the app manager ───────── */
const appCards = new Map();   // id → card (drafts kept while you look at other apps)
const APPS_UI = { current: null };
function newApp(kind = 'wasm', board = null) {
  const taken = new Set([...apps(), ...programs()].map(p => p.name));
  let k = 1; while (taken.has('app' + k)) k++;
  const name = 'app' + k, lang = APP_KINDS[kind].lang;
  const a = { id: 'a' + Date.now().toString(36), kind, name, every: 0.1, on: '', reads: [], writes: { topic: 'user.' + name, layout: 'v' }, src: APP_TEMPLATE[lang](name), draft: true, board };
  appCards.delete(a.id); openAppManager(); showApp(a);
}
function openAppManager(id) {
  const dlg = $('#appDlg'); renderUndo(); if (!dlg.open) dlg.showModal();
  if (id) showApp(id); else if (APPS_UI.current && (appById(APPS_UI.current) || appCards.has(APPS_UI.current))) showApp(APPS_UI.current); else if (apps().length) showApp(apps()[0].id); else renderAppChoices();
}
function renderAppChoices() {
  const s = $('#appSelect'); if (!s) return;
  const draft = APPS_UI.current && !appById(APPS_UI.current) && appCards.get(APPS_UI.current);
  s.replaceChildren(...apps().map(a => el('option', { value: a.id, text: `${a.name} · ${APP_KINDS[a.kind].label} · ${(appBoard(a) || { name: 'on no board' }).name}` })), ...(draft ? [el('option', { value: APPS_UI.current, text: draft.d.name + ' (not added yet)' })] : []));
  if (!s.options.length) s.append(el('option', { value: '', text: 'No apps yet', disabled: 'disabled' }));
  s.value = APPS_UI.current || ''; s.disabled = !s.options.length || !!s.options[0].disabled;
  if (!APPS_UI.current) { $('#appActive').replaceChildren(el('p', { class: 'hint', text: 'An app is code of your own in C or Python, on a board that runs apps (a board\'s settings say what it runs). + New app starts one.' })); setText($('#appTitle'), ''); }
}
function showApp(aOrId) {
  const a = typeof aOrId === 'string' ? appById(aOrId) || (appCards.get(aOrId) || {}).a : aOrId; if (!a) return;
  APPS_UI.current = a.id;
  let c = appCards.get(a.id); if (!c) { c = appCard(a); appCards.set(a.id, c); }
  renderAppChoices();
  $('#appActive').replaceChildren(c.card); setText($('#appTitle'), c.d.name);
  c.sync(); fitTa(c.ta);
}
function appCard(a) {
  const d = JSON.parse(JSON.stringify({ ...a, draft: undefined, board: undefined })), c = { a, d, isNew: !!a.draft, board: a.board };
  const kind = UI.select({ 'aria-label': 'Written in' }, ...Object.entries(APP_KINDS).map(([k, K]) => el('option', { value: k, text: K.label })));
  const name = UI.input({ type: 'text', 'aria-label': 'App name', value: d.name, maxlength: 31, spellcheck: 'false' });
  const mode = UI.select({ 'aria-label': 'When it runs' }, el('option', { value: 'change', text: 'when a topic it reads changes' }), el('option', { value: 'every', text: 'every' }));
  const on = UI.select({ 'aria-label': 'The topic whose change runs it' });
  const every = UI.input({ type: 'number', 'aria-label': 'Every (ms)', min: 1, max: 60000, step: 1, value: String(Math.round((d.every || 0.1) * 1000)) });
  const topic = UI.input({ type: 'text', 'aria-label': 'Topic it writes (after user.)', value: d.writes.topic.replace(/^user\./, ''), maxlength: 18, spellcheck: 'false' });
  const layout = UI.input({ type: 'text', 'aria-label': 'Its fields', value: d.writes.layout, spellcheck: 'false' });
  const reads = el('div', { class: 'prog-reads' }), where = el('p', { class: 'hint app-where' }), note = el('p', { class: 'hint' });
  const header = el('pre', { class: 'prog-shape app-header' }), headerBox = UI.details({ class: 'app-header-box', title: 'liftlab.h (made from the header)' }, header);
  const ta = UI.textarea({ class: 'code', spellcheck: 'false', autocapitalize: 'off', autocomplete: 'off', 'aria-label': 'App code' }); ta.value = d.src;
  const err = UI.status({ class: 'law-err', role: 'status' }), out = el('pre', { class: 'app-compiler', hidden: true }), stats = UI.status({ class: 'hint prog-stats', role: 'status' });
  const logBox = el('pre', { class: 'app-log', hidden: true });
  const apply = UI.button({ class: 'btn primary', text: c.isNew ? 'Add app' : 'Apply' }), del = UI.button({ class: 'btn', text: c.isNew ? 'Discard' : 'Delete' }), dl = UI.button({ class: 'btn', text: 'Download' });
  kind.value = d.kind;
  const readHeader = () => {
    d.kind = kind.value; d.name = name.value.trim(); d.writes = { topic: 'user.' + topic.value.trim(), layout: layout.value.trim().replace(/\s+/g, ' ') };
    d.every = mode.value === 'every' ? Math.max(1, +every.value || 100) / 1000 : 0; d.on = d.every > 0 ? '' : on.value;
    d.reads = [...reads.querySelectorAll('input:checked')].map(x => x.value); d.src = ta.value;
  };
  const listWith = () => c.isNew || !apps().some(q => q.id === d.id) ? [...apps(), d] : apps().map(q => q.id === d.id ? d : q);
  c.sync = () => {
    mode.value = d.every > 0 ? 'every' : 'change'; every.hidden = !(d.every > 0); on.hidden = d.every > 0;
    const cat = busCatalog(d), byBoard = new Map(); for (const t of cat) { if (!byBoard.has(t.board)) byBoard.set(t.board, []); byBoard.get(t.board).push(t); }
    reads.replaceChildren(...[...byBoard].map(([b, ts]) => el('fieldset', {}, el('legend', { text: b.name }), ...ts.map(t => el('label', { class: 'check', title: t.layout + ' · from ' + t.from },
      Object.assign(UI.input({ type: 'checkbox', value: t.name }), { checked: d.reads.includes(t.name), onchange: () => { readHeader(); c.sync(); } }), el('code', { text: t.name }))))));
    if (!cat.length) reads.append(el('p', { class: 'hint', text: 'No topics yet.' }));
    on.replaceChildren(...(d.reads.length ? d.reads : ['']).map(r => el('option', { value: r, text: r || 'choose topics it reads first' })));
    if (d.reads.includes(d.on)) on.value = d.on; else if (d.reads.length) { d.on = d.reads[0]; on.value = d.on; }
    const K = APP_KINDS[d.kind], b = c.isNew ? computers().boards.find(x => x.id === c.board) : appBoard(d);
    note.textContent = K.note;
    where.textContent = b ? `On ${b.name} (${BOARD_KINDS[b.kind].label}).` : 'On no board yet: a board that runs ' + (K.runtime === 'wasm' ? 'WebAssembly' : 'native') + ' apps takes it in its settings.';
    const known = d.reads.filter(r => cat.some(t => t.name === r)), dd = { ...d, reads: known };
    header.textContent = K.lang === 'c' ? appHeaderC(dd, cat) : appShapePython(dd, cat);
    headerBox.querySelector('summary').textContent = K.lang === 'c' ? 'liftlab.h (made from the header)' : 'What step gets and returns';
    const why = appProblem(d, listWith()) || appFieldsProblem(d, cat); err.textContent = why; err.className = 'ui-status law-err' + (why ? ' on' : '');
    appStatsLine(c);
  };
  c.log = logBox;
  const doApply = async () => {
    readHeader();
    let why = appProblem(d, listWith()) || appFieldsProblem(d);
    err.textContent = why; err.className = 'ui-status law-err' + (why ? ' on' : ''); if (why) return;
    const compiled = { ...d };
    if (d.kind === 'wasm') {
      apply.disabled = true; out.hidden = false; out.textContent = 'Compiling…';
      const r = await appCompile(d, ([n, t]) => { out.textContent = `Fetching the C compiler (once): ${Math.round(n / Math.max(1, t) * 100)}%`; });
      apply.disabled = false;
      if (r.err) { out.textContent = r.err; err.textContent = 'It doesn\'t compile: see what the compiler said.'; err.className = 'ui-status law-err on'; return; }
      out.textContent = `Compiled: ${(r.size / 1024).toFixed(1)} KB${r.warn ? '\n\n' + r.warn : ''}`;
      Object.assign(compiled, { bin: r.bin, binOf: r.binOf });
    } else { delete compiled.bin; delete compiled.binOf; out.hidden = true; }
    const old = appById(d.id), restart = !old || appHeader(old) !== appHeader(compiled);
    cfg.apps = listWith().map(q => q.id === d.id ? JSON.parse(JSON.stringify(compiled)) : q); c.a = appById(d.id);
    if (c.isNew && c.board != null) {   // (started from a board's settings: it goes on that board)
      const C = JSON.parse(JSON.stringify(computers())), b = C.boards.find(x => x.id === c.board);
      if (b && boardRuns(b).includes(APP_KINDS[d.kind].runtime)) { for (const x of C.boards) x.apps = (x.apps || []).filter(id => id !== d.id); b.apps = [...(b.apps || []), d.id]; cfg.computers = fixComputers(C); }
    }
    c.isNew = false; apply.textContent = 'Apply'; del.textContent = 'Delete'; undoKey = 'app:' + d.id; flash(ta); save();
    if (restart || !appReload(c.a)) { brt.sig = null; doReset(); }
    renderComputers(true); showApp(d.id);
  };
  apply.addEventListener('click', doApply);
  del.addEventListener('click', () => {
    if (c.isNew) { appCards.delete(d.id); APPS_UI.current = null; openAppManager(); return; }
    if (del.dataset.armed !== '1') { del.dataset.armed = '1'; del.textContent = 'Click again to delete'; setTimeout(() => { del.dataset.armed = ''; del.textContent = 'Delete'; }, 3000); return; }
    cfg.apps = apps().filter(q => q.id !== d.id); appCards.delete(d.id); APPS_UI.current = null;
    const C = JSON.parse(JSON.stringify(computers())); for (const b of C.boards) b.apps = (b.apps || []).filter(id => id !== d.id); cfg.computers = fixComputers(C);
    undoKey = 'app:' + d.id; save(); brt.sig = null; doReset(); renderComputers(true); openAppManager();
  });
  dl.addEventListener('click', () => appDownload(c.a && !c.isNew ? c.a : d));
  name.addEventListener('input', () => { if (!topic.dataset.edited && topic.value === d.name) topic.value = name.value.trim(); readHeader(); c.sync(); });
  topic.addEventListener('input', () => { topic.dataset.edited = '1'; });
  kind.addEventListener('change', () => {   // a new app's template follows its language, while it's untouched
    const was = APP_KINDS[d.kind].lang, now = APP_KINDS[kind.value].lang;
    if (was !== now && ta.value === APP_TEMPLATE[was](d.name)) ta.value = APP_TEMPLATE[now](d.name);
    readHeader(); c.sync(); fitTa(ta);
  });
  for (const x of [topic, layout, every]) x.addEventListener('input', () => { readHeader(); c.sync(); });
  for (const x of [mode, on]) x.addEventListener('change', () => { if (x === mode && mode.value === 'every' && !(d.every > 0)) every.value = '100'; readHeader(); c.sync(); });
  ta.addEventListener('input', () => { fitTa(ta); d.src = ta.value; });
  ta.addEventListener('keydown', e => {
    if (e.key === 'Enter' && (e.metaKey || e.ctrlKey)) { e.preventDefault(); doApply(); return; }
    if (e.key === 'Tab' && !e.shiftKey) { e.preventDefault(); const s = ta.selectionStart, en = ta.selectionEnd; ta.setRangeText(APP_KINDS[d.kind].lang === 'python' ? '    ' : '  ', s, en, 'end'); fitTa(ta); }
  });
  c.stats = stats; c.ta = ta;
  c.card = UI.card({ class: 'law prog-card app-card', 'data-app': d.id },
    el('div', { class: 'prog-head' }, hardwareField('Name', name), hardwareField('Written in', kind)), note, where,
    el('div', { class: 'prog-head' }, hardwareField('Runs', mode), el('div', { class: 'prog-when' }, on, el('label', { class: 'prog-every' }, every, el('span', { text: 'ms' })))),
    UI.details({ class: 'prog-reads-box', title: 'Reads', open: true }, reads),
    el('div', { class: 'prog-head' }, hardwareField('Writes', el('span', { class: 'prog-topic' }, el('code', { text: 'user.' }), topic)), hardwareField('Its fields', layout)),
    el('p', { class: 'hint', text: 'Fields: names separated by spaces, a count in brackets for a list (range rate ok, or v[3]).' }),
    headerBox, ta, el('div', { class: 'law-actions' }, apply, del, dl, el('span', { class: 'kbd', text: '⌘/Ctrl + Enter applies' })), err, out, stats,
    UI.details({ class: 'app-log-box', title: 'Its log (printf)' }, logBox),
    el('p', { class: 'hint', text: 'Changing what it reads or writes, when it runs or what it\'s written in restarts the flight; new code for a WebAssembly app is compiled and loaded in flight.' }));
  return c;
}
function appStatsLine(c) {
  if (!c.stats) return;
  if (c.isNew) { setText(c.stats, 'Not added yet.'); return; }
  const e = brt.appErr && brt.appErr.get(c.d.id), s = brt.appAt && appStats().get(c.d.id), b = appBoard(c.d);
  setText(c.stats, !b ? 'Not running: it\'s on no board.' : e ? 'Not running: ' + e : !s ? 'Not running (the flight computers aren\'t running).'
    : `${s.ok ? 'Running' : 'Not running'} on ${b.name}: ${s.runs} runs${s.waits ? `, ${s.waits} waited for its inputs` : ''}${s.fails ? `, ${s.fails} stopped (${s.why || 'error ' + s.err}) and started again` : ''}.`);
  const L = (brt.appLog && brt.appLog.get(c.d.id)) || [];
  c.log.hidden = !L.length; const text = L.map(l => `${l.t.toFixed(2)} s  ${l.s}`).join('\n'); if (c.log.textContent !== text) c.log.textContent = text;
}
// The app's files: a WebAssembly app's module (what its board loads) and its source with liftlab.h; another's source.
function appDownload(a) {
  const save1 = (name, data, type) => { const x = document.createElement('a'); x.href = URL.createObjectURL(new Blob([data], { type })); x.download = name; document.body.append(x); x.click(); setTimeout(() => { URL.revokeObjectURL(x.href); x.remove(); }, 1000); };
  const lang = APP_KINDS[a.kind].lang;
  if (lang === 'c') { save1(a.name + '.c', a.src, 'text/x-c'); save1('liftlab.h', appHeaderC(a), 'text/x-c'); }
  else save1(a.name + '.py', a.src, 'text/x-python');
  if (a.kind === 'wasm' && a.bin && a.binOf === appBinKey(a, appHeaderC(a))) save1(a.name + '.wasm', unb64(a.bin), 'application/wasm');
}
setInterval(() => {
  const d = document.getElementById('appDlg'); if (!d || !d.open || !APPS_UI.current) return;
  const c = appCards.get(APPS_UI.current); if (!c) return;
  if (typeof fleet !== 'undefined' && fleet.ready) { if (fleet.selected) withDrone(fleet.selected, () => appStatsLine(c)); } else appStatsLine(c);
}, 500);
function buildAppManager(pane) {
  const dlg = computerDialog('appDlg', 'App manager'); dlg.dialog.classList.add('formula-dialog', 'app-dialog');
  const select = UI.select({ id: 'appSelect', 'aria-label': 'App' }); select.addEventListener('change', () => { if (select.value) showApp(select.value); });
  const kind = UI.select({ id: 'appNewKind', 'aria-label': 'New app written in' }, ...Object.entries(APP_KINDS).map(([k, K]) => el('option', { value: k, text: K.label })));
  dlg.body.append(el('div', { class: 'formula-finder' }, hardwareField('App', select), hardwareField('New app in', kind), UI.button({ class: 'btn', id: 'appNew', text: '+ New app', onclick: () => newApp(kind.value) })),
    el('h3', { id: 'appTitle' }), el('div', { id: 'appActive' }));
  pane.append(dlg.dialog);
}
