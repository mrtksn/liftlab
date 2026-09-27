'use strict';
// Articulated rigid-body dynamics. The frame is a free-floating body; every servo joint adds a body (the
// servo's output and everything rigidly on it) that turns about its hinge relative to the body it's mounted
// on. The equations of motion M(q)·q̈ + C(q, q̇, forces) = [0; τ_servo] are built with the recursive
// Newton–Euler algorithm (Featherstone) in each body's own axes and solved every physics step, so swinging
// an arm pushes the frame the other way, rotor thrust on a tilted rotor loads its servo, and a sensor on an
// arm feels the arm's own acceleration.
//
// Spatial vectors are [angular; linear]. Body frames sit at the joint pivot, aligned with the frame's axes
// when every joint is at 0°. The per-body Newton–Euler equation is the 'rigidBody' formula.

let MB = null;   // body structure, rebuilt when the airframe changes

/* ───────── spatial algebra ───────── */
const top3 = v => [v[0], v[1], v[2]], bot3 = v => [v[3], v[4], v[5]];
const cat6 = (a, b) => [a[0], a[1], a[2], b[0], b[1], b[2]];
const add6v = (a, b) => a.map((x, i) => x + b[i]);
function crossM(v, u) { const w = top3(v), l = bot3(v), uw = top3(u), ul = bot3(u); return cat6(crs(w, uw), add(crs(w, ul), crs(l, uw))); }   // v ×m u
function crossF(v, f) { const w = top3(v), l = bot3(v), n = top3(f), ff = bot3(f); return cat6(add(crs(w, n), crs(l, ff)), crs(w, ff)); }      // v ×f f
// A child body's coordinates are E·(parent coordinates), its origin at r (parent coordinates).
function Xmotion(E, r, v) { const w = top3(v); return cat6(m3v(E, w), m3v(E, sub(bot3(v), crs(r, w)))); }   // parent → child
function XforceUp(E, r, f) { const n = m3v(m3T(E), top3(f)), ff = m3v(m3T(E), bot3(f)); return cat6(add(n, crs(r, ff)), ff); }   // child → parent
// Spatial inertia {m, c, Ic} (mass, CoM, inertia about the CoM) times a spatial vector.
function inertiaTimes(I, v) {
  const w = top3(v), l = bot3(v), mc = scl(I.c, I.m);
  return cat6(add(sub(m3v(I.Ic, w), crs(mc, crs(I.c, w))), crs(mc, l)), sub(scl(l, I.m), crs(mc, w)));
}
const spatialForce = (F, P) => cat6(crs(P, F), F);   // force F acting at point P, both in body axes

/* ───────── structure ───────── */
// Which body a part rides on: its nearest joint's, or the frame (0).
const bodyIndexOf = c => { const j = chainOf(c)[0]; return j ? MB.index.get(j.id) : 0; };
function buildBodies() {
  const js = joints().slice().sort((a, b) => chainOf(a).length - chainOf(b).length);
  MB = { bodies: [{ j: null, parent: -1, pivot: [0, 0, 0] }], index: new Map(), of: new Map() };
  for (const j of js) {
    const pj = chainOf(j)[0], parent = pj ? MB.index.get(pj.id) : 0;
    MB.index.set(j.id, MB.bodies.length);
    MB.bodies.push({ j, parent, pivot: j.pos.slice(), axis: jointAxis(j), r: sub(j.pos, MB.bodies[parent].pivot) });
  }
  // Mass items per body, in that body's axes (rest positions relative to its pivot).
  const items = MB.bodies.map(() => []);
  items[0].push({ m: cfg.frame.mass, r: [0, 0, 0], I: boxI(cfg.frame.mass, 0.12, 0.12, 0.04) });
  for (const c of cfg.comps) {
    const b = bodyIndexOf(c), o = MB.bodies[b].pivot;
    MB.of.set(c.id, b);
    if (c.type === 'motor' || c.type === 'joint') items[b].push({ m: c.mass, r: sub(c.pos, o), I: null });
    else if (c.type === 'mass') items[b].push({ m: c.mass, r: sub(c.pos, o), I: shapeI(c) });
    else if (c.type === 'link') items[b].push({ m: c.mass, r: sub(add(c.pos, scl(linkDir(c), c.length / 2)), o), I: rodI(c) });
  }
  MB.bodies.forEach((B, i) => {
    let m = 0, cm = [0, 0, 0]; for (const it of items[i]) { m += it.m; cm = add(cm, scl(it.r, it.m)); }
    cm = m > 0 ? scl(cm, 1 / m) : [0, 0, 0];
    const J = [0, 0, 0, 0, 0, 0, 0, 0, 0];
    for (const it of items[i]) {
      if (it.I) for (let k = 0; k < 9; k++) J[k] += it.I[k];
      const d = sub(it.r, cm), dd = dot(d, d);
      for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) J[a * 3 + b] += it.m * ((a === b ? dd : 0) - d[a] * d[b]);
    }
    B.I = { m, c: cm, Ic: J };
    // A servo's gearbox makes its output feel heavier to turn: reflected motor inertia, bigger for stronger servos.
    if (B.j) B.armature = 0.0005 * Math.max(0.1, B.j.torque ?? 0.8);
  });
}

/* ───────── kinematics ───────── */
// Joint transforms, body velocities and each body's pose relative to the frame (rotation Rb: body → frame axes,
// origin ob in frame axes). v0 is the frame's spatial velocity in its own axes.
function mbKinematics(v0) {
  const K = { E: [], r: [], v: [], Rb: [], ob: [], S: [] };
  MB.bodies.forEach((B, i) => {
    if (i === 0) { K.v[0] = v0; K.Rb[0] = [1, 0, 0, 0, 1, 0, 0, 0, 1]; K.ob[0] = [0, 0, 0]; return; }
    const s = jst.get(B.j.id) || { th: 0, rate: 0 };
    const Rj = run('jointRotation', B.axis, s.th);
    K.E[i] = m3T(Rj); K.r[i] = B.r; K.S[i] = cat6(B.axis, [0, 0, 0]);
    K.v[i] = add6v(Xmotion(K.E[i], K.r[i], K.v[B.parent]), scl6(K.S[i], s.rate));
    K.Rb[i] = m3m(K.Rb[B.parent], Rj);
    K.ob[i] = add(K.ob[B.parent], m3v(K.Rb[B.parent], B.r));
  });
  return K;
}
// Velocity (frame axes) of a point given in frame axes that rides on body b.
function mbPointVel(K, b, P) {
  const Pb = m3v(m3T(K.Rb[b]), sub(P, K.ob[b])), v = K.v[b];
  return m3v(K.Rb[b], add(bot3(v), crs(top3(v), Pb)));
}
const mbOmega = (K, b) => m3v(K.Rb[b], top3(K.v[b]));   // body b's angular velocity, frame axes
// Turn a force (or torque) given in frame axes into a spatial force on body b.
function mbForce(K, b, F, P) { const Rt = m3T(K.Rb[b]); return spatialForce(m3v(Rt, F), m3v(Rt, sub(P, K.ob[b]))); }
function mbTorque(K, b, T) { return cat6(m3v(m3T(K.Rb[b]), T), [0, 0, 0]); }

/* ───────── dynamics ───────── */
// fext[i]: spatial force on body i (its own axes); tauJ[i]: servo torque at joint i. Returns the frame's
// spatial acceleration (own axes), joint accelerations, and every body's spatial acceleration.
function mbSolve(K, fext, tauJ) {
  const N = MB.bodies.length, n = N - 1;
  const rnea = (acc0, qdd, withVel, withExt) => {
    const a = [], f = [];
    for (let i = 0; i < N; i++) {
      const B = MB.bodies[i];
      if (i === 0) a[0] = acc0;
      else {
        a[i] = add6v(Xmotion(K.E[i], K.r[i], a[B.parent]), scl6(K.S[i], qdd[i]));
        if (withVel) a[i] = add6v(a[i], crossM(K.v[i], scl6(K.S[i], (jst.get(B.j.id) || {}).rate || 0)));
      }
      f[i] = withVel ? run('rigidBody', B.I, a[i], K.v[i]) : inertiaTimes(B.I, a[i]);
      if (withExt && fext[i]) f[i] = sub6(f[i], fext[i]);
    }
    const tau = new Array(N).fill(0);
    for (let i = N - 1; i > 0; i--) {
      tau[i] = dot6(K.S[i], f[i]);
      const p = MB.bodies[i].parent; f[p] = add6v(f[p], XforceUp(K.E[i], K.r[i], f[i]));
    }
    return { gen: [...f[0], ...tau.slice(1)], a };
  };
  const zero6 = [0, 0, 0, 0, 0, 0], zq = new Array(N).fill(0);
  const bias = rnea(zero6, zq, true, true).gen;                   // everything but the accelerations
  const dim = 6 + n, M = [];
  for (let k = 0; k < dim; k++) {                                 // mass matrix, one column per unit acceleration
    const a0 = zero6.slice(), qdd = zq.slice();
    if (k < 6) a0[k] = 1; else qdd[k - 5] = 1;
    M.push(rnea(a0, qdd, false, false).gen);
  }
  const A = [...Array(dim)].map((_, r) => [...Array(dim)].map((_, c) => M[c][r]));   // columns → rows
  for (let i = 1; i < N; i++) A[5 + i][5 + i] += MB.bodies[i].armature;
  const rhs = [...Array(dim)].map((_, r) => (r < 6 ? 0 : tauJ[r - 5] || 0) - bias[r]);
  const qdd = solveLin(A, rhs);
  const a0 = qdd.slice(0, 6), jq = [0, ...qdd.slice(6)];
  const acc = rnea(a0, jq, true, false).a;                        // each body's spatial acceleration, for sensors
  return { a0, qdd: jq, acc };
}
// Every body's spatial acceleration when the frame accelerates by a0 and the joints are held still.
function mbAccHeld(K, a0) {
  const acc = [a0];
  for (let i = 1; i < MB.bodies.length; i++) acc[i] = Xmotion(K.E[i], K.r[i], acc[MB.bodies[i].parent]);
  return acc;
}
const sub6 = (a, b) => a.map((x, i) => x - b[i]);
const dot6 = (a, b) => a.reduce((s, x, i) => s + x * b[i], 0);
