'use strict';
// The cargo controls: a button per latch on the view (G works the chosen one), how far the nearest loose thing is
// from an open latch, the Cargo section (what each latch holds, the things in the world to pick up, what happened).
// The buttons are the pilot's: with a radio they go up it as LATCH commands, otherwise to the cargo task's board.

const cargoUi = { cur: 0, sig: null, say: '', sayT: 0, logN: -1 };

// What a latch is doing, as the simulator sees it (the truth: the Ground tab has what the telemetry says).
function latchView(l) {
  const st = cargo.lat.get(l.id), on = onBoard(l), pos = st ? st.pos : l.closed ? 1 : 0;
  const under = on ? liveUnder(l) : [], mass = under.reduce((s, c) => s + (c.mass || 0), 0);
  const moving = st && st.drive != null && Math.abs(st.drive - pos) > 0.001;
  const near = on && pos < 0.999 ? cargoNearest(l) : null;
  return { on, pos, closed: pos >= 0.5, under, mass, moving, near };
}
function cargoSay(text) { cargoUi.say = text; cargoUi.sayT = performance.now(); }
// A press: drop what it holds; open an empty closed one; close on what's in reach; or, with the nearest thing out of
// reach, fetch it (the navigation's pickup: it flies over it, comes down, closes the latch and climbs); during a
// pickup, stop it.
const FETCH_MAX = 15;   // [m] the farthest thing it fetches
const PK_PHASE = ['', 'flying over it', 'coming down', 'closing', 'climbing'];
function cargoAct(i) {
  const l = latches()[i]; if (!l) return;
  cargoUi.cur = i;
  if (!hasTask('cargo')) { latchNeedsBoard(); cargoBarSync(); return; }   // (a latch no board drives: give it one; the flight restarts with it)
  if (brt.pickup) { pickupStop(); cargoSay(`${l.name}: pickup stopped`); cargoBarSync(); return; }
  const v = latchView(l), fetch = v.on && !v.closed && !v.moving && v.near && !v.near.ok && v.near.d < FETCH_MAX;
  const why = fetch ? cargoFetch(l, i, v.near.L) : pilotCargoCmd(i, 2);   // (toggle: the cargo task knows which way it's driving it)
  cargoSay(why ? `${l.name}: ${why}` : fetch ? `${l.name}: fetching ${v.near.L.name}` : '');
  cargoBarSync();
}
// Fetch a loose thing: the simulator knows where it is (on the drone, the pilot says: dfb_ground's "pickup X Y Z").
// With a radio the command module works out where the hub goes from the hook's place and the drone's heading
// (gnd_pickup) and sends it up; without one, it goes straight to the navigation's board.
function cargoFetch(l, i, L) {
  if (!hasTask('nav')) return 'fetching needs the navigation (Computers tab)';
  if (brt.pilot.phase !== 'flying' || !brt.home) return 'take off first';
  const top = sub(looseGrab(L), brt.home), P = poseOf(l), hk = add(P.p, m3v(P.R, LATCH_HOOK));   // the thing's top from home; the hook from the hub (body axes)
  if (brt.gnd) {
    const r = brt.gnd.gnd_pickup(hk[0], hk[1], hk[2], top[0], top[1], top[2], i, brt.t);
    return r === -3 ? 'no attitude from the drone yet' : r === -1 ? 'the command module has too many commands waiting' : r ? 'too far' : '';
  }
  const b = boardOf('nav'), w = b && brt.inst.get(b.id); if (!w) return 'the navigation isn\'t running';
  const h = Math.atan2(est.R[3], est.R[0]), c = Math.cos(h), s = Math.sin(h);
  const spot = [top[0] - (c * hk[0] - s * hk[1]), top[1] - (s * hk[0] + c * hk[1]), top[2] - hk[2] + 0.03];
  return w.pickup_cmd(spot[0], spot[1], spot[2], h, i) ? cstr(w, w.pk_msg_ptr()) : '';
}
const cargoKey = () => { if (latches().length) cargoAct(Math.min(cargoUi.cur, latches().length - 1)); };

// The buttons on the view: one per latch, rebuilt when the latches change.
function cargoBarBuild() {
  const box = $('#cargoBar'); if (!box) return;
  const ls = latches(); box.textContent = ''; box.hidden = !ls.length;
  ls.forEach((l, i) => {
    const b = UI.button( { type: 'button', class: 'btn cargo-btn', 'data-latch': String(i) },
      el('span', { class: 'cg-name', text: l.name }), el('span', { class: 'cg-state' }),
      el('span', { class: 'cg-meter', 'aria-hidden': 'true' }, el('i')), el('kbd', { text: 'G' }));
    b.addEventListener('mousedown', e => e.preventDefault());   // (the keyboard stays with the view)
    b.addEventListener('click', () => cargoAct(i));
    box.append(b);
  });
  box.append(el('span', { class: 'cargo-say', id: 'cargoSay', role: 'status' }));
}
function cargoBarSync() {
  const box = $('#cargoBar'); if (!box) return;
  const ls = latches(), sig = ls.map(l => l.id + l.name).join(',');
  if (sig !== cargoUi.sig) { cargoUi.sig = sig; cargoBuildRest(); }
  if (cargoUi.cur >= ls.length) cargoUi.cur = 0;
  ls.forEach((l, i) => {
    const b = box.querySelector(`[data-latch="${i}"]`); if (!b) return;
    const v = latchView(l), st = b.querySelector('.cg-state'), meter = b.querySelector('.cg-meter'), bar = meter.firstChild;
    let text, tone = '', title;
    if (!v.on) { text = 'fell off'; tone = 'off'; title = `${l.name} came off the drone`; }
    else if (v.moving) { text = v.closed ? 'opening…' : 'closing…'; title = `${l.name} is moving`; }
    else if (v.closed && v.under.length) { text = `drop ${(v.mass * 1000).toFixed(0)} g`; tone = 'loaded'; title = `${l.name} holds ${v.under.filter(c => parentOf(c) === l).map(c => c.name).join(', ')}: press to let go`; }
    else if (v.closed) { text = 'empty · open'; title = `${l.name} is closed with nothing in it: press to open it, ready to grab`; }
    else if (brt.pickup && i === cargoUi.cur) { text = `fetching · ${PK_PHASE[brt.pickup] || ''}`; tone = 'reach'; title = 'The drone is picking it up by itself: press to stop (or move the sticks)'; }
    else if (v.near && v.near.ok) { text = `grab · ${(v.near.d * 100).toFixed(0)} cm`; tone = 'reach'; title = `${v.near.L.name} is within reach: press to close on it`; }
    else if (v.near && v.near.d < FETCH_MAX) { text = `fetch · ${v.near.d < 1 ? (v.near.d * 100).toFixed(0) + ' cm' : v.near.d.toFixed(1) + ' m'}`; title = `${v.near.L.name} is ${v.near.d.toFixed(2)} m away (in reach: ${((l.reach ?? 0.08) * 100).toFixed(0)} cm). Press and the drone fetches it: flies over it, comes down, closes the latch and climbs`; }
    else if (v.near) { text = `${v.near.d.toFixed(0)} m away`; title = `The nearest loose thing, ${v.near.L.name}, is too far to fetch`; }
    else { text = 'open · close'; title = `${l.name} is open, nothing loose nearby`; }
    setText(st, text); b.dataset.tone = tone; b.title = title + (i === cargoUi.cur ? ' (G)' : '');
    meter.hidden = !(v.on && !v.closed && v.near);
    if (!meter.hidden) bar.style.width = `${Math.round(100 * clamp(1 - (v.near.d - (l.reach ?? 0.08)) / 1.0, 0, 1))}%`;
    b.querySelector('kbd').hidden = i !== cargoUi.cur;
    b.disabled = !v.on || !cargo.power;
  });
  const say = $('#cargoSay');
  if (say) setText(say, !cargo.power ? 'no power: ' + powerWhy() : !hasTask('cargo') && ls.length ? 'no board drives the latches: press to put the Cargo task on the flight controller' : performance.now() - cargoUi.sayT < 4000 ? cargoUi.say : '');
}

/* ───────── the Cargo section (right panel) ───────── */
function cargoBuildRest() { cargoBarBuild(); cargoSecBuild(); }
function cargoSecBuild() {
  const box = $('#cargoBody'); if (!box) return;
  box.textContent = '';
  box.append(el('dl', { id: 'cargoLatches', class: 'kv' }));
  box.append(el('span', { class: 'lbl', text: 'To pick up' }), el('div', { id: 'cargoItems' }));
  const add = UI.button( { class: 'btn', type: 'button', text: '+ Thing to pick up' });
  add.addEventListener('click', () => { if (cargoWorld.items.length >= 8) return; cargoWorld.items.push({ name: 'Parcel ' + (cargoWorld.items.length + 1), mass: 0.2, size: [0.1, 0.1, 0.08], at: [1 + 0.4 * cargoWorld.items.length, -0.6] }); cargoWorldSave(); cargoItemsBuild(); });
  box.append(add, el('p', { class: 'hint', text: 'Boxes resting on whatever is under them, from the start point (X forward, Y left at the start). They move to where you set them at the next reset. Fly an open latch over one, within its reach, and close it.' }));
  box.append(el('span', { class: 'lbl', text: 'What happened' }), el('ul', { class: 'cargo-log', id: 'cargoLog' }));
  cargoItemsBuild(); cargoUi.logN = -1;
}
function cargoItemsBuild() {
  const box = $('#cargoItems'); if (!box) return; box.textContent = '';
  cargoWorld.items.forEach((it, k) => {
    const nm = UI.input( { type: 'text', value: it.name || '', maxlength: '18', 'aria-label': 'Name' });
    nm.addEventListener('change', () => { it.name = nm.value || 'Parcel'; cargoWorldSave(); });
    const f = (label, get, set, d) => numField(`cgi-${k}-${label}`, { label, ...d }, get, v => { set(v); cargoWorldSave(); }).node;
    const del = UI.button( { class: 'icon-btn', type: 'button', text: '×', title: 'Remove', 'aria-label': 'Remove ' + it.name });
    del.addEventListener('click', () => { cargoWorld.items.splice(k, 1); cargoWorldSave(); cargoItemsBuild(); });
    box.append(el('div', { class: 'cargo-item' }, el('div', { class: 'cargo-item-head' }, nm, del),
      f('Mass', () => it.mass, v => { it.mass = v; }, { min: 0.02, max: 1.5, hmax: 5, step: 0.01, u: 'kg', dp: 2 }),
      el('div', { class: 'subgrid' },
        f('X', () => it.at[0], v => { it.at[0] = v; }, { min: -5, max: 5, hmin: -25, hmax: 25, step: 0.1, u: 'm', dp: 1 }),
        f('Y', () => it.at[1], v => { it.at[1] = v; }, { min: -5, max: 5, hmin: -25, hmax: 25, step: 0.1, u: 'm', dp: 1 }),
        f('Size', () => it.size[0], v => { it.size = [v, v, +(v * 0.8).toFixed(3)]; }, { min: 0.03, max: 0.4, step: 0.01, u: 'm', dp: 2 }))));
  });
}
function cargoSecSync() {
  const sec = $('#cargoSec'); if (!sec) return;
  const ls = latches(); sec.hidden = !ls.length && !cargo.loose.length;
  const box = $('#cargoLatches'); if (!box) return;
  const rows = ls.map(l => {
    const v = latchView(l), held = v.under.filter(c => parentOf(c) === l).map(c => c.name);
    return [l.name, !v.on ? 'fell off' : `${v.moving ? (v.closed ? 'opening' : 'closing') : v.closed ? 'closed' : 'open'}${held.length ? ' · holds ' + held.join(', ') + ` (${(v.mass * 1000).toFixed(0)} g)` : v.closed ? ' · empty' : ''}${v.near && !v.closed ? ` · nearest ${v.near.L.name} ${(v.near.d * 100).toFixed(0)} cm` : ''}`];
  });
  rows.push(['Power', cargo.power ? 'on' : cargo.powerT != null ? `off since ${cargo.powerT.toFixed(1)} s: ${powerWhy()}` : `off: ${powerWhy()}`]);
  rows.push(['Loose', cargo.loose.length ? cargo.loose.map(L => `${L.name} (${(L.m * 1000).toFixed(0)} g${L.asleep ? '' : ', moving'})`).join(', ') : 'nothing']);
  syncKv(box, rows);
  const log = $('#cargoLog'); if (!log) return;
  const key = cargo.log.length ? cargo.log[0].t + cargo.log[0].msg + cargo.log.length : '';
  if (key === cargoUi.logN) return; cargoUi.logN = key;
  log.textContent = '';
  for (const e of cargo.log) log.append(el('li', { class: 'tone-' + e.tone }, el('span', { class: 't', text: e.t.toFixed(1) + ' s' }), ' ' + e.msg));
  if (!cargo.log.length) log.append(el('li', { class: 'muted', text: 'Nothing yet.' }));
}
