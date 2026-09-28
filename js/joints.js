'use strict';
// Servo joints and rods. A joint is a hinge mounted on the frame, a rod or another joint. A rod (a stick or
// lever) is a rigid bar with a mass; whatever is attached to it rides at its far end. Anything can be
// attached to either: motors, rigid masses, cable payloads, sensors, rods, further joints. Every part keeps its position and
// orientation "at rest" (all joint angles zero, in body axes); where it is now comes from rotating it
// about each joint above it, nearest first:  p_now = M_root(… M_parent(p_rest)),  M_j(p) = q_j + R_j(p − q_j).
//
// The physics uses the true joint angles. The flight software uses the angles it believes: the servo's
// feedback reading, or its own prediction from what it commanded (servoPredictor).

function mkJoint(name, x, y, z, o = {}) {
  return base(Object.assign({ type: 'joint', name, pos: [x, y, z], hingeAz: 0, hingeEl: 0, mode: 'auto', manual: 0,
    range: 40, rate: 240, lag: 0.02, torque: 0.8, offset: 0, feedback: false, mass: 0.015 }, o));
}
// A motor on its own servo joint, both at the same point, the way a tilt-rotor is usually built.
function mkServoMotor(name, x, y, z, jo = {}, mo = {}) {
  const j = mkJoint(name + ' servo', x, y, z, jo);
  const m = mkMotor(name, x, y, z, Object.assign({ parent: j.id }, mo));
  return [j, m];
}

// A rod: from its base (pos) along a direction (azimuth, elevation) for `length`. Its far end is the tip.
function mkLink(name, x, y, z, o = {}) {
  return base(Object.assign({ type: 'link', name, pos: [x, y, z], az: 0, el: -90, roll: 0, length: 0.15, mass: 0.02, known: true }, o));
}
function linkDir(l) { const a = l.az * D2R, e = l.el * D2R; return [Math.cos(e) * Math.cos(a), Math.cos(e) * Math.sin(a), Math.sin(e)]; }
const linkTip = l => add(l.pos, scl(linkDir(l), l.length));

const joints = () => cfg.comps.filter(c => c.type === 'joint');
const links = () => cfg.comps.filter(c => c.type === 'link');
const compById = id => id == null ? null : cfg.comps.find(c => c.id === id) || null;
const isHolder = c => !!c && (c.type === 'joint' || c.type === 'link');
function parentOf(c) { const p = compById(c.parent); return isHolder(p) ? p : null; }   // what a part is attached to (null: the frame)
function ancestorsOf(c) {   // what a part hangs from, nearest first
  const out = [], seen = new Set([c.id]);
  for (let p = parentOf(c); p && !seen.has(p.id); p = parentOf(p)) { seen.add(p.id); out.push(p); }
  return out;
}
function parentJoint(c) { return chainOf(c)[0] || null; }   // nearest joint above a part
function jointAxis(j) {   // hinge axis at rest, body frame
  const a = j.hingeAz * D2R, e = (j.hingeEl || 0) * D2R;
  return [Math.cos(e) * Math.cos(a), Math.cos(e) * Math.sin(a), Math.sin(e)];
}
// The axes of the surface a part is mounted on, in body axes at rest (columns: its X, Y, Z). On the frame,
// or on a servo's output (which sits at 0° at rest), they're the body axes. On a rod: X runs along the rod,
// Z is as close to up as the rod allows (forward, for a rod pointing straight up or down), Y completes them.
// A rod can also be rolled about its own length (roll, degrees), which turns what's on it round the rod.
function rodFrame(x, roll = 0) {
  const up = Math.abs(x[2]) > 0.95 ? [1, 0, 0] : [0, 0, 1];
  let z = unit(sub(up, scl(x, dot(up, x)))), y = crs(z, x);
  if (roll) { const r = roll * D2R, c = Math.cos(r), s = Math.sin(r); [y, z] = [add(scl(y, c), scl(z, s)), sub(scl(z, c), scl(y, s))]; }
  return [x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]];
}
const rodFrameOf = l => rodFrame(linkDir(l), l.roll || 0);
function setRodFrame(l, F) {   // point and roll a rod so its frame is F (as near as the stored angles allow)
  setDirAzEl(l, [F[0], F[3], F[6]], 'az', 'el');
  const B = rodFrame(linkDir(l), 0), fz = [F[2], F[5], F[8]];
  l.roll = +(Math.atan2(-dot(fz, [B[1], B[4], B[7]]), dot(fz, [B[2], B[5], B[8]])) * R2D).toFixed(1);
  if (Math.abs(l.roll) < 0.05) l.roll = 0;
}
function mountFrame(c) {
  const p = parentOf(c);
  return p && p.type === 'link' ? rodFrameOf(p) : [1, 0, 0, 0, 1, 0, 0, 0, 1];
}
const mountName = c => { const p = parentOf(c); return p && p.type === 'link' ? p.name : p ? p.name + '\'s output' : 'the frame'; };
// A servo's hinge axis as a heading and tilt relative to what it's mounted on (degrees), and back.
function hingeRel(j) {
  const a = m3v(m3T(mountFrame(j)), jointAxis(j));
  return { az: Math.hypot(a[0], a[1]) > 1e-6 ? Math.atan2(a[1], a[0]) * R2D : 0, el: Math.asin(clamp(a[2], -1, 1)) * R2D };
}
function setHingeRel(j, az, el) {
  const A = az * D2R, E = el * D2R;
  setDirAzEl(j, m3v(mountFrame(j), [Math.cos(E) * Math.cos(A), Math.cos(E) * Math.sin(A), Math.sin(E)]), 'hingeAz', 'hingeEl');
}

// The way people think about a servo: what it carries sticks out from it one way (d), and the servo swings
// that load toward some direction s, at right angles to d; the hinge axis is d × s. `swing` is the angle of
// s around d, measured from the mount's own X (its Z when X runs along d), so it's relative to the mount and
// stays put when the mount turns. `lean` tips the hinge toward d, for a load that sweeps a cone (rare).
// d: toward the parts it carries, or a carried rotor's thrust when that sits on the pivot, else the mount's Z.
function carriedDir(j) {
  let s = [0, 0, 0];
  for (const c of descendants(j)) for (const q of c.type === 'link' ? [c.pos, linkTip(c)] : [c.pos]) { const r = sub(q, j.pos); if (nrm(r) > 0.008) s = add(s, unit(r)); }
  if (nrm(s) < 1e-3) { const m = descendants(j).find(c => c.type === 'motor'); if (m) s = mountDir(m); }
  if (nrm(s) < 1e-3) { const M = mountFrame(j); s = [M[2], M[5], M[8]]; }
  return unit(s);
}
function swingBasis(j) {
  const d = carriedDir(j), M = mountFrame(j);
  let r = [0, 0, 0];
  for (const k of [0, 2, 1]) { const x = [M[k], M[3 + k], M[6 + k]]; r = sub(x, scl(d, dot(x, d))); if (nrm(r) > 0.3) break; }
  const e1 = unit(r); return { d, e1, e2: crs(d, e1) };
}
const swingRefName = j => {   // what 0° swings toward, in words: where that points on the drone at rest
  const e = swingBasis(j).e1, names = [['the front', [1, 0, 0]], ['the back', [-1, 0, 0]], ['the left', [0, 1, 0]], ['the right', [0, -1, 0]], ['the top', [0, 0, 1]], ['the bottom', [0, 0, -1]]];
  const best = names.reduce((b, n) => dot(n[1], e) > dot(b[1], e) ? n : b);
  return dot(best[1], e) > 0.94 ? best[0] : `${mountName(j)}'s ${(parentOf(j) || {}).type === 'link' ? 'far end' : 'front'}`;
};
const swingVec = (b, deg) => add(scl(b.e1, Math.cos(deg * D2R)), scl(b.e2, Math.sin(deg * D2R)));
function swingOf(j) {   // { swing, lean } in degrees, and the swing direction s at rest (body axes)
  const b = swingBasis(j), a = jointAxis(j), ad = clamp(dot(a, b.d), -1, 1), ap = sub(a, scl(b.d, ad));
  if (nrm(ap) < 1e-6) return { swing: 0, lean: ad > 0 ? 90 : -90, s: b.e1 };
  const s = unit(crs(ap, b.d));
  return { swing: Math.atan2(dot(s, b.e2), dot(s, b.e1)) * R2D, lean: Math.asin(ad) * R2D, s };
}
function setSwing(j, swing, lean = swingOf(j).lean) {
  const b = swingBasis(j), s = swingVec(b, swing), L = lean * D2R;
  setDirAzEl(j, add(scl(crs(b.d, s), Math.cos(L)), scl(b.d, Math.sin(L))), 'hingeAz', 'hingeEl');
}
// Quick picks: swing the load toward the drone's forward, left or up (whichever aren't along the load),
// and, on a rod, along the rod. Each is a swing angle; none is fixed to the body once set.
function swingPresets(j) {
  const b = swingBasis(j), out = [];
  const add1 = (k, label, v) => {
    const p = sub(v, scl(b.d, dot(v, b.d))); if (nrm(p) < 0.35) return;
    const deg = Math.atan2(dot(p, b.e2), dot(p, b.e1)) * R2D;
    if (out.some(o => Math.abs(Math.sin((o.deg - deg) * D2R)) < 0.17)) return;   // same plane as one already listed
    out.push({ k, label, deg: +deg.toFixed(1) });
  };
  add1('fb', 'Forward–back', [1, 0, 0]); add1('lr', 'Left–right', [0, 1, 0]); add1('ud', 'Up–down', [0, 0, 1]);
  const p = parentOf(j); if (p && p.type === 'link') add1('rod', 'Along the rod', linkDir(p));
  return out;
}
function swingPreset(j) {   // the quick pick the servo matches now (either way round), or null
  const w = swingOf(j); if (Math.abs(w.lean) > 0.5) return null;
  return swingPresets(j).find(o => Math.abs(Math.sin((o.deg - w.swing) * D2R)) < 0.01) || null;
}
const chainOf = c => ancestorsOf(c).filter(a => a.type === 'joint');   // joints above a part, nearest first
const isUnder = (c, a) => ancestorsOf(c).includes(a);
const descendants = a => cfg.comps.filter(c => c !== a && isUnder(c, a));
const childrenOf = a => cfg.comps.filter(c => parentOf(c) === a);
const canAttach = (c, a) => isHolder(a) && a !== c && !isUnder(a, c);
const motorsUnder = j => actuators().filter(c => isUnder(c, j));
// Joints the allocation steers: in auto mode and carrying at least one motor. Others are positioned by you.
const steerJoints = () => joints().filter(j => j.mode === 'auto' && motorsUnder(j).length);

/* ───────── joint state ───────── */
const jst = new Map();   // id -> { th: true angle, thR: horn angle, thCmd, thHat: believed angle, pst, rate, acc }
const restAngle = j => j.mode === 'manual' ? j.manual * D2R : 0;
const angleTrue = j => { const s = jst.get(j.id); return s ? s.th : restAngle(j); };
function angleSeen(j) {   // what the flight software uses
  const s = jst.get(j.id); if (!s) return restAngle(j);
  return j.feedback ? s.th : s.thHat;
}
const angleZero = () => 0;
function jointTarget(j) {   // what the servo is told
  const s = jst.get(j.id); const cal = calServo(j);
  if (cal != null) return cal;
  if (j.mode === 'manual' || !steerJoints().includes(j)) return j.manual * D2R;
  return s ? s.thCmd : 0;
}

/* ───────── poses ───────── */
function poseOf(c, ang = angleTrue) { return posePoint(c, c.pos, ang); }
function posePoint(c, pRest, ang = angleTrue) {   // any rest point that moves with part c (rods: their tip)
  let R = [1, 0, 0, 0, 1, 0, 0, 0, 1], p = pRest.slice();
  for (const j of chainOf(c)) {
    const Rj = run('jointRotation', jointAxis(j), ang(j));
    p = add(j.pos, m3v(Rj, sub(p, j.pos)));
    R = m3m(Rj, R);
  }
  return { R, p };
}
const posNow = c => poseOf(c).p;
// Angular velocity of a part relative to the frame, from the joints above it moving (body frame).
function chainRate(c) {
  let w = [0, 0, 0];
  for (const j of chainOf(c)) { const s = jst.get(j.id); if (!s || !s.rate) continue; w = add(w, scl(m3v(poseOf(j).R, jointAxis(j)), s.rate)); }
  return w;
}

function chainRateSeen(c) {   // the same, from the joint angles the flight software believes
  let w = [0, 0, 0];
  for (const j of chainOf(c)) { const s = jst.get(j.id); if (!s || !s.rateHat) continue; w = add(w, scl(m3v(poseOf(j, angleSeen).R, jointAxis(j)), s.rateHat)); }
  return w;
}

// Velocity of a part relative to the frame from its joints turning (body frame): true, and as believed.
function chainVel(c, seen = false) {
  const ang = seen ? angleSeen : angleTrue, p = poseOf(c, ang).p; let v = [0, 0, 0];
  for (const j of chainOf(c)) {
    const s = jst.get(j.id), r = s && (seen ? s.rateHat : s.rate); if (!r) continue;
    const P = poseOf(j, ang); v = add(v, scl(crs(m3v(P.R, jointAxis(j)), sub(p, P.p)), r));
  }
  return v;
}

/* ───────── carrying parts along when a holder is edited ───────── */
// Moving a joint or rod moves everything on it; turning a rod swings everything on it about its base.
function shiftSubtree(a, d) { for (const c of descendants(a)) c.pos = c.pos.map((v, i) => +(v + d[i]).toFixed(4)); }
function rotateSubtree(a, R, pivot) {
  for (const c of descendants(a)) {
    c.pos = add(pivot, m3v(R, sub(c.pos, pivot))).map(v => +v.toFixed(4));
    if (c.type === 'motor') { const d = m3v(R, mountDir(c)); c.tilt = +(Math.acos(clamp(d[2], -1, 1)) * R2D).toFixed(1); if (c.tilt > 0.05) c.az = +(Math.atan2(d[1], d[0]) * R2D).toFixed(1); }
    else if (c.type === 'sensor') c.mount = eulerFromR(m3m(R, eulerR(...c.mount))).map(x => +x.toFixed(1));
    else if (c.type === 'joint') setDirAzEl(c, m3v(R, jointAxis(c)), 'hingeAz', 'hingeEl');
    else if (c.type === 'link') setRodFrame(c, m3m(R, rodFrameOf(c)));
  }
}
// Turn a part by R about its own pivot, carrying everything attached to it (what it's on stays put).
// Motors and sensors turn in place; a servo's hinge turns and its load swings round the pivot; a rod
// turns about its base (edited() then swings its load, from the rod's old and new frames).
function turnPart(c, R) {
  if (c.type === 'joint') { setDirAzEl(c, m3v(R, jointAxis(c)), 'hingeAz', 'hingeEl'); rotateSubtree(c, R, c.pos); }
  else if (c.type === 'link') setRodFrame(c, m3m(R, rodFrameOf(c)));
  else if (c.type === 'motor') { const d = m3v(R, mountDir(c)); c.tilt = +(Math.acos(clamp(d[2], -1, 1)) * R2D).toFixed(1); if (c.tilt > 0.05) c.az = +(Math.atan2(d[1], d[0]) * R2D).toFixed(1); }
  else if (c.type === 'sensor') c.mount = eulerFromR(m3m(R, eulerR(...c.mount))).map(x => +x.toFixed(1));
}
// What a part and everything on it look like now, to put back (a drag applies its whole turn from the start).
const POSE_KEYS = ['pos', 'tilt', 'az', 'el', 'roll', 'hingeAz', 'hingeEl', 'mount'];
const poseSnap = c => [c, ...descendants(c)].map(x => [x, Object.fromEntries(POSE_KEYS.filter(k => k in x).map(k => [k, Array.isArray(x[k]) ? x[k].slice() : x[k]]))]);
const poseRestore = snap => { for (const [x, v] of snap) for (const k in v) x[k] = Array.isArray(v[k]) ? v[k].slice() : v[k]; };
function setDirAzEl(c, d, kAz, kEl) {
  const u = unit(d); c[kEl] = +(Math.asin(clamp(u[2], -1, 1)) * R2D).toFixed(1);
  if (Math.hypot(u[0], u[1]) > 1e-4) c[kAz] = +(Math.atan2(u[1], u[0]) * R2D).toFixed(1);
}
function rotationBetween(a, b) {   // smallest rotation taking unit a to unit b
  const u = unit(a), v = unit(b), ax = crs(u, v), s = nrm(ax), c = clamp(dot(u, v), -1, 1);
  if (s < 1e-9) return c > 0 ? [1, 0, 0, 0, 1, 0, 0, 0, 1] : axisAngleR(Math.abs(u[0]) < 0.9 ? crs(u, [1, 0, 0]) : crs(u, [0, 1, 0]), Math.PI);
  return axisAngleR(ax, Math.atan2(s, c));
}
// Attach a part to a holder (or to the frame with null). On a rod a part goes to the rod's far end; on a
// servo it goes onto the servo's output: a motor or rod right on the pivot (a tilt-rotor, an arm), anything
// else just below it. A servo given its first motor is there to steer it, so it's handed to the allocator.
function attachTo(c, a) {
  const firstMotor = a && a.type === 'joint' && c.type === 'motor' && !motorsUnder(a).length;
  c.parent = a ? a.id : null;
  const to = !a ? null : a.type === 'link' ? linkTip(a) : c.type === 'motor' || c.type === 'link' ? a.pos : add(a.pos, [0, 0, -0.04]);
  if (to) { const d = sub(to, c.pos); c.pos = to.map(v => +v.toFixed(4)); shiftSubtree(c, d); }
  if (firstMotor && a.mode === 'manual') a.mode = 'auto';
}

/* ───────── saved airframes from before joints existed ───────── */
// A "motor on servo" part becomes a servo joint with a motor attached at the same point.
function migrateTiltParts(comps) {
  const out = [];
  for (const c of comps) {
    if (c.type !== 'tilt') { out.push(c); continue; }
    const j = { id: uid++, type: 'joint', name: c.name + ' servo', pos: c.pos.slice(), hingeAz: c.hingeAz || 0, hingeEl: 0, mode: c.mode || 'auto', manual: c.manual || 0,
      range: c.range ?? 40, rate: c.rate ?? 240, lag: c.lag ?? 0.02, torque: 0.8, offset: c.offset || 0, feedback: !!c.feedback, mass: 0.015, parent: c.parent };
    const m = Object.assign({}, c, { type: 'motor', tilt: 0, az: 0, parent: j.id, mass: Math.max(0.01, (c.mass ?? 0.075) - 0.015) });
    for (const k of ['hingeAz', 'mode', 'manual', 'range', 'rate', 'lag', 'offset', 'feedback']) delete m[k];
    out.push(j, m);
  }
  return out;
}
