'use strict';
// Repeatable runs. Everything random comes from the world's seeds (math.js worldSeeds; the city's is terrain.seed),
// so starting the world again with the same seeds is the same run. Seeds can be kept as snapshots (with the terrain
// and the environment) and brought back.
// A recording is the pilot's inputs, each on the physics step it was made on: the keys and pads held, and the
// actions (speed, hold, home, poke, a latch, a reset, the target and environment fields). Recording starts the
// world again. A replay restores the recording's world (seeds, terrain, environment, each drone's target, speed and
// launch), starts it again and makes the same inputs on the same steps, on whatever design flies now: the same
// flight, gusts and sensor noise, so a change to the design or its code can be judged against the last one.
// Inputs of your own during a replay end it, and you fly on from there.

const SEEDS_LS = 'liftlab-seed-snapshots-v1', REC_LS = 'liftlab-recordings-v1';
const REPLAY_CTRLS = ['fwd', 'back', 'left', 'right', 'up', 'down', 'yawL', 'yawR'];
const SEED_FIELDS = [['city', 'City layout'], ['noise', 'Sensor noise'], ['air', 'Gusts, eddies and pokes'], ['parts', 'Motor and prop differences'], ['radio', 'Radio links']];
const loadList = k => { try { const v = JSON.parse(localStorage.getItem(k) || '[]'); return Array.isArray(v) ? v : []; } catch (e) { return []; } };
const saveList = (k, v) => { try { localStorage.setItem(k, JSON.stringify(v)); return true; } catch (e) { fleetNotice('This browser could not keep it: ' + e.message); return false; } };
const newSeed = () => 1 + Math.floor(Math.random() * 0xFFFFFFFE);
const fmtT = s => { const m = Math.floor(s / 60); return (m ? m + ':' + String(Math.floor(s % 60)).padStart(2, '0') : Math.floor(s)) + '.' + Math.floor((s % 1) * 10); };

/* ───────── seeds ───────── */
function setWorldSeeds(s) {
  for (const k of SEED_KEYS) { const v = Math.floor(Number(s[k])); worldSeeds[k] = Number.isFinite(v) && v > 0 && v <= 0xFFFFFFFF ? v : DEFAULT_SEEDS[k]; }
}
function worldSnapshot() { return { terrain: { kind: terrain.kind, seed: terrain.seed }, seeds: { ...worldSeeds }, environment: { ...envr } }; }
// The world as a snapshot had it. (Starting the flights again is the caller's.)
function applyWorld(w) {
  setWorldSeeds(w.seeds || {});
  if (w.environment) Object.assign(envr, w.environment);
  if (w.terrain && TERRAINS[w.terrain.kind] && (w.terrain.kind !== terrain.kind || w.terrain.seed !== terrain.seed)) { setTerrain(w.terrain.kind, w.terrain.seed); for (const d of fleet.drones) withDrone(d, () => { cPts = contactPoints(); }); }
  syncTerrainUi(); syncSp(); refreshEnvelope(); renderMass(); seedsUiSync(); save(); fleetSave();
}
let seedSnaps = loadList(SEEDS_LS);
const seedUi = { fields: new Map() };
function seedsUiBuild() {
  const box = $('#seedFields'); if (!box) return;
  box.replaceChildren();
  for (const [k, label] of SEED_FIELDS) {
    const inp = UI.input({ type: 'number', class: 'num seed-num', id: 'seed-' + k, min: '1', max: '4294967295', step: '1', 'aria-label': label + ' seed' });
    inp.addEventListener('change', () => {
      const v = Math.floor(Number(inp.value)); if (!(v >= 1 && v <= 0xFFFFFFFF)) { seedsUiSync(); return; }
      if (k === 'city') { if (v !== terrain.seed) applyTerrain(terrain.kind, v); return; }   // a new city: the flights start again
      worldSeeds[k] = v; replayWorldChanged(); fleetSave();   // (the rest take hold from the next reading, gust or reset)
    });
    seedUi.fields.set(k, inp);
    box.append(el('label', { class: 'seed-row', for: 'seed-' + k }, el('span', { text: label }), inp));
  }
  seedsUiSync(); seedSnapsRender();
}
function seedsUiSync() {
  for (const [k, inp] of seedUi.fields) if (document.activeElement !== inp) inp.value = String(k === 'city' ? terrain.seed : worldSeeds[k]);
  const c = seedUi.fields.get('city'); if (c) c.disabled = terrain.kind === 'open';
}
function seedSnapsRender() {
  const box = $('#seedSnaps'); if (!box) return;
  box.replaceChildren(...seedSnaps.map((s, i) => {
    const what = `${({ open: 'Open field', parkour: 'Parkour city', city: 'Full-scale city' })[s.terrain?.kind] || ''} · wind ${(+s.environment?.wind || 0).toFixed(1)} m/s · turbulence ${(+s.environment?.turb || 0).toFixed(2)}`;
    return el('li', { class: 'seed-snap' },
      el('span', { class: 'seed-snap-name' }, el('b', { text: s.name }), el('small', { text: what })),
      UI.button({ class: 'btn btn-sm', title: 'Use these seeds, map layout and environment, and start every flight again', onclick: () => seedSnapLoad(i) }, 'Load'),
      UI.button({ class: 'btn icon btn-sm', 'aria-label': 'Delete ' + s.name, title: 'Delete', onclick: () => { seedSnaps.splice(i, 1); saveList(SEEDS_LS, seedSnaps); seedSnapsRender(); } }, '×'));
  }));
  $('#seedSnapsEmpty').hidden = seedSnaps.length > 0;
}
function seedSnapLoad(i) {
  const s = seedSnaps[i]; if (!s || liveOn()) return;
  replayWorldChanged(); applyWorld(s); userWorldReset(); seedSay(`Loaded “${s.name}”: every flight started again.`);
}
function seedSay(t) { const n = $('#seedSay'); if (!n) return; n.textContent = t; clearTimeout(seedSay.t); seedSay.t = setTimeout(() => { n.textContent = ''; }, 4000); }
$('#seedSave').addEventListener('click', () => {
  const name = $('#seedName').value.trim().slice(0, 60) || 'World ' + (seedSnaps.length + 1);
  seedSnaps.unshift({ name, at: Date.now(), ...worldSnapshot() }); seedSnaps = seedSnaps.slice(0, 40);
  if (saveList(SEEDS_LS, seedSnaps)) { $('#seedName').value = ''; seedSnapsRender(); seedSay(`Saved “${name}”.`); }
});
$('#seedName').addEventListener('keydown', e => { if (e.key === 'Enter') { e.preventDefault(); $('#seedSave').click(); } });
$('#seedRandom').addEventListener('click', () => {   // a new world: every seed new, and the flights start again in it
  if (liveOn()) return;
  for (const k of SEED_KEYS) worldSeeds[k] = newSeed();
  applyTerrain(terrain.kind, newSeed()); seedsUiSync(); fleetSave(); seedSay('A new random world: every flight started again.');
});

/* ───────── inputs: recorded and replayed ───────── */
const rec = { on: false, take: null, mask: new Map() };
const replay = { rec: null, play: false, open: false, ended: false, applying: false, i: 0, shift: 0, end: 0, map: new Map(), seek: null, seekRun: false };
let recordings = loadList(REC_LS);
replay.rec = recordings[0] || null;
const USER_ACTIONS = {
  hold: () => pilotHold(), home: () => pilotHome(), level: k => setPilotLevel(k), cargo: i => cargoAct(i),
  poke: c => { if (!S.crashed) pokeHit(c); }, reset: () => doReset(), launch: m => setLaunch(m), set: a => setTargetOrWorld(a),
};
// What a person does at the controls: done now, and recorded on this step. One of their own ends a replay.
function userAction(kind, arg) {
  if (!USER_ACTIONS[kind]) return;
  if (replay.play && !replay.applying) replayStop(true);
  if (rec.on && !liveOn()) rec.take.ev.push([fleet.steps, fleet.selected ? fleet.selected.id : null, kind, arg ?? null]);
  return USER_ACTIONS[kind](arg);
}
const heldMask = () => REPLAY_CTRLS.reduce((m, c, i) => isHeld(c) ? m | 1 << i : m, 0);
// pilot.js: what's held changed (src: who pressed or let go).
function replayInput(src) {
  if (src === 'replay' || !fleet.ready) return;
  if (replay.play && !replay.applying && src !== 'release' && src !== 'agent') replayStop(true);
  if (!rec.on || !fleet.active) return;
  const d = fleet.active, m = heldMask();
  if (m === (rec.mask.get(d.id) || 0)) return;
  rec.mask.set(d.id, m); rec.take.ev.push([fleet.steps, d.id, 'k', m]);
}
function userWorldReset() { if (rec.on) recordStop(); if (replay.play) replayStop(false); return fleetResetAll(); }
function replayWorldChanged() { if (rec.on) recordStop(); if (replay.play) replayStop(false); }

function recordStart() {
  if (!fleet.ready || liveOn() || replay.play || !fleetCanSelect()) return;
  if (!fleetResetAll()) return;
  const drones = fleet.drones.map(d => withDrone(d, () => ({ id: d.id, name: fleetName(d), setpoint: { ...setpoint }, level: pilot.level, launch: launchMode })));   // (where each starts: as the reset left it)
  rec.take = { v: 1, id: 'r' + Date.now().toString(36), name: '', at: Date.now(), world: worldSnapshot(), drones, sel: fleet.selected?.id || null, ev: [], end: 0 };
  rec.on = true; rec.mask = new Map();
  if (!running) { running = true; renderRun(); }
  replayUi(true);
}
function recordStop() {
  if (!rec.on) return;
  const t = rec.take; rec.on = false; rec.take = null; t.end = fleet.steps;
  if (t.end < 1 / PDT) { replaySay('Too short to keep: record at least a second.'); replayUi(true); return; }
  const n = 1 + recordings.reduce((m, r) => Math.max(m, +(/^Take (\d+)/.exec(r.name) || [0, 0])[1]), 0);
  t.name = `Take ${n} · ${fleetName(fleet.drones.find(d => d.id === recMain(t)) || fleet.selected || fleet.drones[0])}`;
  recordings.unshift(t); recordings = recordings.slice(0, 30); saveList(REC_LS, recordings);
  replay.rec = t; replaySay(`Recorded ${t.name.split(' · ')[0]}, ${fmtT(t.end * PDT)} s. Replay plays it over the same world.`); replayUi(true);
}
const recLead = r => r.ev.length ? r.ev[0][0] : r.end;
const recDelay = r => r.delay ?? recLead(r) * PDT;   // before the first input: as recorded, unless changed
function recMain(r) {   // the drone the pilot flew most
  const n = new Map(); for (const e of r.ev) if (e[1]) n.set(e[1], (n.get(e[1]) || 0) + 1);
  return [...n].sort((a, b) => b[1] - a[1])[0]?.[0] || r.sel || r.drones[0]?.id;
}
// Which drones fly whose inputs: as recorded (a drone that's gone: the selected one, if only one was flown), the
// selected drone flies the main one's, or every drone does.
function replayMap(r, to) {
  const m = new Map(), main = recMain(r), byId = id => fleet.drones.find(d => d.id === id);
  if (to === 'all') m.set(main, [...fleet.drones]);
  else if (to === 'selected') { if (fleet.selected) m.set(main, [fleet.selected]); }
  else {
    const flown = [...new Set(r.ev.map(e => e[1]).filter(Boolean))];
    for (const id of flown) { const d = byId(id) || (flown.length === 1 ? fleet.selected || fleet.drones[0] : null); if (d) m.set(id, [d]); }
  }
  return m;
}
function replayStart(r = replay.rec) {
  if (!r || !fleet.ready || liveOn() || rec.on || !fleetCanSelect()) return false;
  if (replay.play) replayStop(false);
  // Where each drone starts (the drone's target moves as it flies, so it's put back every time): a recorded drone
  // where it started, a drone standing in for one that's gone where that one did, any other where it was when the
  // replay opened. Drones flying a recorded drone's inputs take its speed and launch too.
  if (!replay.open || !replay.homes) replay.homes = new Map(fleet.drones.map(d => [d.id, { ...d.state.setpoint }]));
  replay.rec = r; replay.map = replayMap(r, r.to || 'recorded');
  applyWorld(r.world);
  const recd = new Map(r.drones.map(x => [x.id, x]));
  for (const d of fleet.drones) {
    const from = [...replay.map].find(([, ds]) => ds.includes(d))?.[0], own = recd.get(d.id), stand = !own && from && !fleet.drones.some(x => x.id === from) ? recd.get(from) : null;
    const start = own || stand, how = own || recd.get(from);
    withDrone(d, () => {
      Object.assign(setpoint, start ? start.setpoint : replay.homes.get(d.id) || setpoint);
      if (how) { pilot.level = PILOT_LEVELS[how.level] ? how.level : 'normal'; if (how.launch !== launchMode && (how.launch !== 'throw' || hasTask('learn'))) launchMode = how.launch; }
    });
  }
  if (!fleetResetAll()) return false;
  setPilotLevel(pilot.level); setLaunch(launchMode, false); syncSp();
  Object.assign(replay, { play: true, open: true, ended: false, i: 0, seek: null });
  replay.shift = Math.round(recDelay(r) / PDT) - recLead(r); replay.end = Math.max(1, r.end + replay.shift);
  if (!running) { running = true; renderRun(); }
  replayUi(true); return true;
}
function replayLetGo() { for (const d of fleet.drones) withDrone(d, () => { for (const c of REPLAY_CTRLS) release(c, 'replay'); }); }
// took: the person took the controls (the bar goes; they fly on).
function replayStop(took) {
  if (!replay.play && !replay.open) return;
  replay.play = false; replay.seek = null; replayLetGo();
  if (took) { replay.open = false; replay.homes = null; replaySay('Replay stopped: you have the controls.'); }
  replayUi(true);
}
function replayClose() { if (replay.play) replayStop(false); replay.open = false; replay.homes = null; replayUi(true); }
function replayApply(e) {
  const [, id, kind, arg] = e, run = () => {
    if (kind === 'k') { for (let i = 0; i < REPLAY_CTRLS.length; i++) (arg >> i & 1 ? press : release)(REPLAY_CTRLS[i], 'replay'); }
    else if (USER_ACTIONS[kind]) USER_ACTIONS[kind](arg);
  };
  if (id == null) { run(); return; }
  for (const d of replay.map.get(id) || []) withDrone(d, run);
}
// fleet.js, before every physics step: the inputs due on it. True: stop stepping (the recording ended).
function replayStep() {
  if (!replay.play) return false;
  const ev = replay.rec.ev, k = fleet.steps - replay.shift;
  if (replay.i < ev.length && ev[replay.i][0] <= k) {
    replay.applying = true;
    try { while (replay.i < ev.length && ev[replay.i][0] <= k) replayApply(ev[replay.i++]); } finally { replay.applying = false; }
  }
  if (fleet.steps >= replay.end) {   // the end of the recording: it stops there, to be looked at
    replay.play = false; replay.ended = true; replay.seek = null; replayLetGo();
    running = false; renderRun(); replayUi(true); return true;
  }
  return false;
}
// ui.js frame: while seeking, as many steps as fit in a frame (the rest of the time, the usual pace).
function replaySeekSteps(budget) {
  if (replay.seek == null) return null;
  const left = replay.seek - fleet.steps;
  if (left <= 0 || !replay.play) { replay.seek = null; if (!replay.seekRun && replay.play) { running = false; renderRun(); } replayUi(true); return 0; }
  return Math.min(left, budget);
}
function replaySeek(step) {
  if (!replay.rec) return;
  const run = running || replay.play;
  if (!replay.play || step < fleet.steps) { if (!replayStart()) return; }
  replay.seek = Math.min(step, replay.end); replay.seekRun = run && !replay.ended;
  if (!running) { running = true; renderRun(); }
  replayUi(true);
}

/* ───────── the bar ───────── */
function replaySay(t) { const n = $('#rpSay'); if (!n) return; n.textContent = t; clearTimeout(replaySay.t); replaySay.t = setTimeout(() => { n.textContent = ''; }, 5000); }
const rpMenu = menuButton({ text: 'Recording', key: 'rpMenu', label: 'Recordings', title: 'Pick a recording, or rename, save, open or delete one',
  items: () => [
    ...recordings.map(r => ({ value: 'r:' + r.id, label: r.name, hint: fmtT(r.end * PDT) + ' s', group: 'Recordings', cur: r === replay.rec })),
    ...(replay.rec ? [{ value: 'rename', label: 'Rename…', group: 'This recording' }, { value: 'export', label: 'Save as a file', group: 'This recording' }, { value: 'delete', label: 'Delete', group: 'This recording' }] : []),
    { value: 'import', label: 'Open a recording file…', group: 'Files' }],
  onPick: v => {
    const r = replay.rec;
    if (v.startsWith('r:')) { const x = recordings.find(x => x.id === v.slice(2)); if (x) replayStart(x); }
    else if (v === 'rename' && r) { const n = prompt('Name this recording', r.name); if (n && n.trim()) { r.name = n.trim().slice(0, 80); saveList(REC_LS, recordings); replayUi(true); } }
    else if (v === 'export' && r) {
      const a = el('a', { href: URL.createObjectURL(new Blob([JSON.stringify(r)], { type: 'application/json' })), download: r.name.replace(/[^\w .-]+/g, '_') + '.liftlab-replay.json' });
      document.body.append(a); a.click(); a.remove(); setTimeout(() => URL.revokeObjectURL(a.href), 1000);
    }
    else if (v === 'delete' && r && confirm(`Delete ${r.name}?`)) {
      recordings = recordings.filter(x => x !== r); saveList(REC_LS, recordings);
      replay.rec = recordings[0] || null; replayClose();
    }
    else if (v === 'import') $('#rpFile').click();
  } });
rpMenu.node.classList.add('rp-menu');
$('#rpMenuSlot').replaceWith(rpMenu.node);
$('#rpFile').addEventListener('change', async e => {
  const f = e.target.files[0]; e.target.value = ''; if (!f) return;
  try {
    const r = JSON.parse(await f.text());
    if (!r || r.v !== 1 || !Array.isArray(r.ev) || !Array.isArray(r.drones) || !r.world || !(r.end > 0)) throw new Error('not a recording');
    r.id = 'r' + Date.now().toString(36); recordings.unshift(r); recordings = recordings.slice(0, 30); saveList(REC_LS, recordings); replayStart(r);
  } catch (x) { replaySay('Could not open it: ' + x.message); }
});
$('#recBtn').addEventListener('click', () => { if (rec.on) recordStop(); else recordStart(); });
$('#replayBtn').addEventListener('click', () => { if (replay.rec) replayStart(); });
$('#rpRestart').addEventListener('click', () => replayStart());
$('#rpPlay').addEventListener('click', () => {
  if (replay.ended || !replay.play) { replayStart(); return; }
  running = !running; renderRun(); replayUi(true);
});
$('#rpClose').addEventListener('click', replayClose);
const rpTime = $('#rpTime');
rpTime.addEventListener('input', () => { replay.drag = true; replayUi(true); });
rpTime.addEventListener('change', () => { replay.drag = false; replaySeek(+rpTime.value); });
$('#rpDelay').addEventListener('change', e => {
  const r = replay.rec, v = Number(e.target.value); if (!r) return;
  if (!(v >= 0 && v <= 600)) { replayUi(true); return; }
  r.delay = Math.round(v * 10) / 10; saveList(REC_LS, recordings); replayStart(r);
});
$('#rpTo').addEventListener('change', e => { const r = replay.rec; if (!r) return; r.to = e.target.value; saveList(REC_LS, recordings); replayStart(r); });

let rpKey = '';
function replayUi(force = false) {
  if (rec.on && liveOn()) recordStop();
  const r = replay.rec, open = replay.open && !!r, steps = fleet.steps;
  const key = [rec.on, open, replay.play, replay.ended, running, r?.id, r?.name, recordings.length, fleet.drones.length, replay.seek != null].join('|');
  if (force || key !== rpKey) {
    rpKey = key;
    $('.pilot-mid').classList.toggle('replaying', open);
    $('#rpBar').hidden = !open;
    const rb = $('#recBtn'); rb.classList.toggle('on', rec.on); rb.setAttribute('aria-pressed', String(rec.on));
    rb.title = rec.on ? 'Stop recording and keep it' : 'Start every flight again and record your inputs, to replay them later over the same world';
    $('#replayBtn').disabled = rec.on || !r; $('#replayBtn').title = r ? `Replay ${r.name} over the same world (its seeds, map and environment)` : 'Record a flight first';
    if (r) {
      rpMenu.btn.firstChild.textContent = r.name.split(' · ')[0];
      rpMenu.btn.title = r.name + ' — pick another recording, or rename, save or delete this one';
      rpTime.max = String(replay.end || r.end);
      const dl = $('#rpDelay'); if (document.activeElement !== dl) dl.value = recDelay(r).toFixed(1);
      $('#rpTo').value = r.to || 'recorded'; $('#rpToWrap').hidden = fleet.drones.length < 2 && (r.to || 'recorded') === 'recorded';
      const end = replay.end || r.end, lead = Math.round(recDelay(r) / PDT);
      rpTime.style.setProperty('--lead', (100 * Math.min(1, lead / end)).toFixed(2) + '%');
    }
    const pb = $('#rpPlay'), playing = replay.play && running;
    pb.classList.toggle('paused', !playing); pb.setAttribute('aria-label', playing ? 'Pause' : 'Play'); pb.title = playing ? 'Pause' : replay.ended ? 'Play again from the start' : 'Play';
  }
  if (rec.on) setText($('#recLbl'), 'Stop ' + fmtT(steps * PDT));
  else setText($('#recLbl'), 'Record');
  if (open) {
    const end = replay.end || r.end, at = replay.drag ? +rpTime.value : Math.min(steps, end);
    if (!replay.drag) rpTime.value = String(at);
    rpTime.style.setProperty('--at', (100 * at / end).toFixed(2) + '%');
    setText($('#rpClock'), `${fmtT(at * PDT)} / ${fmtT(end * PDT)}`);
    setText($('#rpState'), replay.seek != null ? 'seeking…' : replay.ended ? 'ended' : replay.play && at < Math.round(recDelay(r) / PDT) ? 'waiting' : '');
  }
}
seedsUiBuild(); replayUi(true);
