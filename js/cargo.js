'use strict';
// Cargo: latches that hold parts and let them go, loose parts in the world, and picking them up again.
//
// A latch is a part like any other (a hook, a gripper, an electromagnet), mounted on the frame, a rod or a servo,
// and anything can hang from it: a parcel, a cable payload, a whole arm, a motor, the battery. A board runs the
// Cargo task (runner/fc/cargo_core.c) and drives each latch: closed, it holds what's under it; opened, everything
// under it falls away as one loose body. Closed again, it holds whatever loose body is within its reach at that
// moment: the body snaps under the hook (in the drone's axes), and the drone carries it from then on.
//
// The design (cfg.comps) is what you built and stays as it is. What's on the drone now is an overlay on it:
// design parts that fell off (cargo.off) and copies of the parts picked up (cargo.extra, each with its own id).
// The physics flies liveComps(); the editor, the boards' wiring and the exports keep the design.
//
// Power: a rigid mass marked as a battery powers the drone. Drop the last one and the motors stop, the boards go
// dark (their latches stay where they are) and the radio goes quiet, until the next reset. A design with no mass
// marked as a battery is always powered (designs from before this existed).

function mkLatch(name, x, y, z, o = {}) {
  return base(Object.assign({ type: 'latch', name, pos: [x, y, z], mass: 0.02, closed: true, reach: 0.08, travel: 0.15, sense: true, known: true }, o));
}
const latches = () => cfg.comps.filter(c => c.type === 'latch');
const LATCH_HOOK = [0, 0, -0.018];                 // where a load hangs from, below the latch's pivot
const hookOf = l => add(l.pos, LATCH_HOOK);        // at rest, body axes
// How far a part's top sits above its own position, so it hangs under a hook rather than through it.
function topOf(c) {
  if (c.type === 'mass') return c.shape === 'box' ? c.size[2] / 2 : c.shape === 'sphere' ? c.radius : c.length / 2;
  if (c.type === 'hang') return 0;
  if (c.type === 'motor' || c.type === 'link') return 0;
  return 0.012;
}

/* ───────── what's on the drone now ───────── */
let cargo = {
  off: new Set(),        // design parts not on the drone now
  extra: [],             // parts picked up: copies with their own ids, attached to a latch
  loose: [],             // loose bodies in the world
  lat: new Map(),        // latch id -> { pos: 0 open … 1 closed, drive: what its board drives (1 closed, 0 open, null none), holds }
  rev: 0,                // counts changes to what's on board (the view rebuilds the drone)
  power: true,
  log: [],
  powerT: null,          // when the power went
};
const liveComps = () => cargo.off.size || cargo.extra.length ? cfg.comps.filter(c => !cargo.off.has(c.id)).concat(cargo.extra) : cfg.comps;
const onBoard = c => !cargo.off.has(c.id);
const liveMotors = () => liveComps().filter(c => c.type === 'motor' && !c.cargo);   // (a motor picked up isn't wired to anything)
const liveJoints = () => liveComps().filter(c => c.type === 'joint');
const liveUnder = a => liveComps().filter(c => c !== a && isUnder(c, a));
const isBattery = c => c.type === 'mass' && !!c.battery;
const designPowered = () => !cfg.comps.some(isBattery) || liveComps().some(isBattery);
function cargoLog(msg, tone = 'info') { cargo.log.unshift({ t: S.t, msg, tone }); if (cargo.log.length > 10) cargo.log.pop(); }

/* ───────── things in the world to pick up ───────── */
// Set in the Cargo section (right panel), kept in this browser. Positions are from the start point; each starts
// resting on whatever is under it. They're there only when the airframe has a latch.
const CARGO_LS = 'drone-force-bench-cargo';
const cargoWorld = { items: [{ name: 'Parcel', mass: 0.25, size: [0.1, 0.1, 0.08], at: [1.2, 0.6] }] };
try { const s = JSON.parse(localStorage.getItem(CARGO_LS) || 'null'); if (s && Array.isArray(s.items)) cargoWorld.items = s.items.slice(0, 8); } catch (e) {}
function cargoWorldSave() { try { localStorage.setItem(CARGO_LS, JSON.stringify(cargoWorld)); } catch (e) {} }

/* ───────── loose bodies ───────── */
// A loose body is rigid: its parts' positions at rest, relative to its grab point (a dropped load: the hook it hung
// from; an item: the middle of its top). Its state is its centre of mass p, v, attitude q (world from its own
// axes) and spin w (world axes). A cable payload that falls is a ball of its own.
const payloadRad = c => 0.025 + 0.035 * Math.cbrt(c.mass);
function looseBody(name, parts, kind = 'rigid') {
  const items = [], pts = [];
  for (const c of parts) {
    const ball = 0.4 * c.mass * 0.015 * 0.015, I0 = [ball, 0, 0, 0, ball, 0, 0, 0, ball];   // (a point mass still has some size)
    if (c.type === 'mass') items.push({ m: c.mass, r: c.pos, I: shapeI(c) });
    else if (c.type === 'link') items.push({ m: c.mass, r: add(c.pos, scl(linkDir(c), c.length / 2)), I: rodI(c) });
    else items.push({ m: c.mass || 0.01, r: c.pos, I: I0 });
    if (c.type === 'mass') {
      if (c.shape === 'box') for (const p of boxPoints(c.pos, c.size, massRot(c))) pts.push({ r: p.rest, rad: p.r });
      else if (c.shape === 'sphere') pts.push({ r: c.pos.slice(), rad: c.radius });
      else pts.push({ r: add(c.pos, [0, 0, c.length / 2 - c.radius]), rad: c.radius }, { r: add(c.pos, [0, 0, -c.length / 2 + c.radius]), rad: c.radius });
    } else if (c.type === 'link') pts.push({ r: c.pos.slice(), rad: 0.006 }, { r: linkTip(c), rad: 0.009 });
    else if (c.type === 'hang') pts.push({ r: c.pos.slice(), rad: payloadRad(c) });
    else if (c.type === 'motor') pts.push({ r: c.pos.slice(), rad: 0.018 }, { r: add(c.pos, [0, 0, 0.02]), rad: Math.min(0.03, propR(c) * 0.3) });
    else pts.push({ r: c.pos.slice(), rad: c.type === 'joint' ? 0.015 : 0.01 });
  }
  let m = 0, cm = [0, 0, 0]; for (const it of items) { m += it.m; cm = add(cm, scl(it.r, it.m)); } cm = scl(cm, 1 / Math.max(1e-6, m));
  const J = [0, 0, 0, 0, 0, 0, 0, 0, 0];
  for (const it of items) { for (let i = 0; i < 9; i++) J[i] += it.I[i]; const d = sub(it.r, cm), dd = dot(d, d); for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) J[i * 3 + j] += it.m * ((i === j ? dd : 0) - d[i] * d[j]); }
  const reach = Math.max(0.05, ...pts.map(p => nrm(sub(p.r, cm)) + p.rad));
  return { id: uid++, name, parts, kind, m: Math.max(1e-3, m), cm, Ic: J, pts, reach, p: [0, 0, 0], v: [0, 0, 0], q: [1, 0, 0, 0], w: [0, 0, 0],
    still: 0, asleep: false, near: [], nearT: 0, battery: parts.some(isBattery) };
}
const looseR = L => qmat(L.q);
const looseAt = (L, r) => add(L.p, m3v(looseR(L), sub(r, L.cm)));   // a point of the body (its own rest axes) in the world
const looseGrab = L => looseAt(L, [0, 0, 0]);
// The world items, as loose bodies resting where they're set.
function spawnItems() {
  for (const it of cargoWorld.items) {
    const sz = (it.size || [0.1, 0.1, 0.08]).map(v => Math.max(0.005, +v || 0.05));   // (any size or mass you type; above zero)
    const box = mkMass(it.name || 'Parcel', 0, 0, -sz[2] / 2, { mass: Math.max(0.001, +it.mass || 0.2), size: sz, known: false, cargo: true });
    const L = looseBody(box.name, [box]), x = it.at ? +it.at[0] || 0 : 1, y = it.at ? +it.at[1] || 0 : 0;
    const ground = terrain.boxes.length ? surfaceBelow([x, y, 100]) : 0;
    L.p = [x, y, ground + sz[2] / 2 + 0.002]; L.asleep = true; L.box = null;
    cargo.loose.push(L);
  }
}

/* ───────── reset ───────── */
function cargoReset() {
  cargo.off.clear(); cargo.extra = []; cargo.loose = []; cargo.lat.clear(); cargo.log = []; cargo.rev++; cargo.powerT = null;
  for (const l of latches()) cargo.lat.set(l.id, { pos: l.closed ? 1 : 0, drive: null, holds: true });
  cargo.power = designPowered();
  if (latches().length) spawnItems();   // (only for an airframe that could pick them up: the others' world stays as it was)
}
// Something came off or went on: the drone's bodies, mass and contacts again, and its power.
function cargoChanged() {
  const acc0 = S.mb && S.mb.acc ? S.mb.acc[0] : [0, 0, 0, 0, 0, 0];
  recomputeProps(); cPts = contactPoints(); truth = massProps('truth');
  S.mb = { K: mbKinematics(cat6(S.w, m3v(m3T(qmat(S.q)), S.v))), acc: MB.bodies.map((_, i) => i ? [0, 0, 0, 0, 0, 0] : acc0) };
  cargo.rev++;
  const was = cargo.power; cargo.power = designPowered();
  if (was && !cargo.power) { cargo.powerT = S.t; cargoLog('No battery on board: the power is off. The motors stop and the boards go dark.', 'bad'); }
}

/* ───────── letting go ───────── */
function cargoRelease(l) {
  const under = liveUnder(l); if (!under.length) return 0;
  const R = qmat(S.q), P = poseOf(l), hookB = add(P.p, m3v(P.R, LATCH_HOOK)), Rw = m3m(R, P.R);
  const vAt = rb => add(S.v, m3v(R, crs(S.w, rb))), wW = m3v(R, S.w);
  const hookRest = hookOf(l), ids = new Map();
  for (const c of under) ids.set(c.id, uid++);
  const rigid = [], dropped = [];
  for (const c of under) {
    const k = JSON.parse(JSON.stringify(c));
    k.id = ids.get(c.id); k.origin = c.origin ?? c.id; k.cargo = true;
    k.parent = c.parent === l.id || !ids.has(c.parent) ? null : ids.get(c.parent);
    if (c.type === 'hang') {   // a cable payload falls as a ball of its own, from where it swung
      const st = pend.get(c.id); k.pos = [0, 0, 0]; k.parent = null;
      const L = looseBody(c.name, [k], 'hang');
      L.p = st ? st.p.slice() : add(S.p, m3v(R, posNow(c))); L.v = st ? st.v.slice() : S.v.slice();
      cargo.loose.push(L); dropped.push(c.name);
    } else { k.pos = sub(c.pos, hookRest); rigid.push(k); }
  }
  if (rigid.length) {
    const L = looseBody(rigid.filter(k => k.parent == null).map(k => k.name).join(' + ') || rigid[0].name, rigid);
    L.q = matToQuat(Rw); L.p = add(add(S.p, m3v(R, hookB)), m3v(Rw, L.cm));
    L.w = wW; L.v = vAt(add(hookB, m3v(P.R, L.cm)));
    cargo.loose.push(L); dropped.push(L.name);
  }
  for (const c of under) {
    if (c.cargo) cargo.extra = cargo.extra.filter(x => x !== c); else cargo.off.add(c.id);
    pend.delete(c.id);
    const st = c.type === 'motor' && act.get(c.id); if (st) Object.assign(st, { Omega: 0, T: 0, i: 0, esc: 0 });   // (its spin no longer shakes the frame)
  }
  cargoLog(`${l.name} opened: ${dropped.join(', ')} fell away.`, 'warn');
  cargoChanged();
  return under.length;
}

/* ───────── picking up ───────── */
// The loose body nearest a latch's hook, and how far its grab point is (a cable payload: from the ball's surface).
function cargoNearest(l) {
  if (!onBoard(l)) return null;
  const R = qmat(S.q), P = poseOf(l), hook = add(S.p, m3v(R, add(P.p, m3v(P.R, LATCH_HOOK))));
  let best = null;
  for (const L of cargo.loose) {
    const d = Math.max(0, nrm(sub(looseGrab(L), hook)) - (L.kind === 'hang' ? payloadRad(L.parts[0]) : 0));
    if (!best || d < best.d) best = { L, d, hook };
  }
  if (best) best.ok = best.d <= (l.reach ?? 0.08);
  return best;
}
function cargoGrab(l) {
  if (liveUnder(l).length) return false;                   // (it still holds something)
  const n = cargoNearest(l); if (!n || !n.ok) return false;
  const L = n.L, hook = hookOf(l);
  for (const k of L.parts) { if (k.parent == null) k.parent = l.id; k.pos = add(hook, L.kind === 'hang' ? [0, 0, 0] : k.pos); cargo.extra.push(k); }
  // momentum: the drone and its new load move on together
  const M = truth.m; S.v = scl(add(scl(S.v, M), scl(L.v, L.m)), 1 / (M + L.m));
  cargo.loose = cargo.loose.filter(x => x !== L);
  cargoChanged();
  if (L.kind === 'hang') { const k = L.parts[0], st = pend.get(k.id); if (st) { st.p = L.p.slice(); st.v = L.v.slice(); } }
  cargoLog(`${l.name} closed on ${L.name} (${(L.m * 1000).toFixed(0)} g).`, 'good');
  return true;
}

/* ───────── each physics step ───────── */
// A latch follows what its board drives, at its own speed (travel: open to closed). Half open, what it holds falls;
// fully closed, it holds whatever is within its reach then.
function cargoStep(dt) {
  for (const l of latches()) {
    const st = cargo.lat.get(l.id); if (!st || !onBoard(l)) continue;
    if (st.drive == null) continue;
    const was = st.pos, k = dt / Math.max(0.01, l.travel ?? 0.15);
    st.pos = clamp(st.pos + clamp(st.drive - st.pos, -k, k), 0, 1);
    if (was >= 0.5 && st.pos < 0.5) { st.holds = false; cargoRelease(l); }
    if (st.pos >= 0.999 && !st.holds) { st.holds = true; if (!cargoGrab(l) && was < 0.999) cargoLog(`${l.name} closed with nothing in reach.`); }
  }
  for (const L of cargo.loose) looseStep(L, dt);
}
// What each latch's load switch says: something is under it.
const latchLoaded = l => onBoard(l) && liveUnder(l).length > 0;
// The boards drive the latches (boards.js): bit i of mask closed, for the design's latches in order.
function cargoDrive(mask) { const b=boardOf('cargo');latches().forEach((l, i) => { const st = cargo.lat.get(l.id); if (st) st.drive = b && wiredTo(l)===b ? (mask & (1 << i) ? 1 : 0) : null; }); }

// A loose thing at rest is solid to the drone and to other falling things, as a box round it (the drone can stand on
// a parcel or knock its props on it; it doesn't push it about). terrainContacts takes these like the buildings.
function looseBox(L) {
  if (L.box) return L.box;
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (const pt of L.pts) { const p = looseAt(L, pt.r); for (let i = 0; i < 3; i++) { lo[i] = Math.min(lo[i], p[i] - pt.rad); hi[i] = Math.max(hi[i], p[i] + pt.rad); } }
  return (L.box = { lo, hi, what: L.name });
}
function cargoSolids(p, r, except = null) {
  const out = [];
  for (const L of cargo.loose) {
    if (!L.asleep || L === except) continue;
    const b = looseBox(L);
    if (p[0] > b.lo[0] - r && p[0] < b.hi[0] + r && p[1] > b.lo[1] - r && p[1] < b.hi[1] + r && p[2] > b.lo[2] - r && p[2] < b.hi[2] + r) out.push(b);
  }
  return out;
}
// A rigid body under gravity and air drag, resting on the ground, the buildings and things already at rest through
// contact springs at its corners (stiff enough that a drop from 10 m doesn't sink in). It falls asleep once it has stopped.
function looseStep(L, dt) {
  if (L.asleep) return;
  const R = looseR(L), m = L.m;
  let F = add([0, 0, -m * G], scl(sub(windVec(), L.v), 0.03)), T = scl(L.w, -0.0005);
  if (--L.nearT <= 0) { const r = L.reach + 0.3 + nrm(L.v) * 0.05; L.near = (terrain.boxes.length ? terrainNear(L.p, r) : []).concat(cargoSolids(L.p, r, L)); L.nearT = 20; }
  const k = 15000 * m, c = 120 * m, mu = 60 * m;
  let touching = false;
  for (const pt of L.pts) {
    const rw = m3v(R, sub(pt.r, L.cm)), pw = add(L.p, rw);
    if (pw[2] > pt.rad && !L.near.length) { pt.prev = pw; continue; }
    const hits = terrainContacts(pw, pt.rad, L.near, pt.prev); pt.prev = pw; if (!hits.length) continue;
    touching = true;
    const vel = add(L.v, crs(L.w, rw));
    for (const h of hits) {
      const n = h.n, vn = dot(vel, n), vt = sub(vel, scl(n, vn)), fn = Math.max(0, k * h.depth - c * vn);
      const vtn = nrm(vt), ft = vtn > 1e-6 ? scl(vt, -Math.min(mu * vtn, 0.6 * fn) / vtn) : [0, 0, 0];
      const f = add(scl(n, fn), ft); F = add(F, f); T = add(T, crs(rw, f));
    }
  }
  L.v = add(L.v, scl(F, dt / m)); L.p = add(L.p, scl(L.v, dt));
  const Iw = m3m(m3m(R, L.Ic), m3T(R)), Iinv = m3inv(Iw);
  L.w = add(L.w, scl(m3v(Iinv, sub(T, crs(L.w, m3v(Iw, L.w)))), dt));
  const dq = qmul([0, L.w[0], L.w[1], L.w[2]], L.q); L.q = qnorm(L.q.map((x, i) => x + 0.5 * dq[i] * dt));
  if (L.p[2] < -0.5) { L.p[2] = 0.05; L.v = [0, 0, 0]; }   // (never lost under the ground)
  L.still = touching && nrm(L.v) < 0.02 && nrm(L.w) < 0.08 ? L.still + dt : 0;
  if (L.still > 0.4) { L.asleep = true; L.v = [0, 0, 0]; L.w = [0, 0, 0]; L.box = null; }
}
