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
  return base(Object.assign({ type: 'link', name, pos: [x, y, z], az: 0, el: -90, length: 0.15, mass: 0.02, known: true }, o));
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
    if (c.type === 'motor') { const d = m3v(R, actDir(c)); c.tilt = +(Math.acos(clamp(d[2], -1, 1)) * R2D).toFixed(1); if (c.tilt > 0.05) c.az = +(Math.atan2(d[1], d[0]) * R2D).toFixed(1); }
    else if (c.type === 'sensor') c.mount = eulerFromR(m3m(R, eulerR(...c.mount))).map(x => +x.toFixed(1));
    else if (c.type === 'joint') setDirAzEl(c, m3v(R, jointAxis(c)), 'hingeAz', 'hingeEl');
    else if (c.type === 'link') setDirAzEl(c, m3v(R, linkDir(c)), 'az', 'el');
  }
}
function setDirAzEl(c, d, kAz, kEl) {
  const u = unit(d); c[kEl] = +(Math.asin(clamp(u[2], -1, 1)) * R2D).toFixed(1);
  if (Math.hypot(u[0], u[1]) > 1e-4) c[kAz] = +(Math.atan2(u[1], u[0]) * R2D).toFixed(1);
}
function rotationBetween(a, b) {   // smallest rotation taking unit a to unit b
  const u = unit(a), v = unit(b), ax = crs(u, v), s = nrm(ax), c = clamp(dot(u, v), -1, 1);
  if (s < 1e-9) return c > 0 ? [1, 0, 0, 0, 1, 0, 0, 0, 1] : axisAngleR(Math.abs(u[0]) < 0.9 ? crs(u, [1, 0, 0]) : crs(u, [0, 1, 0]), Math.PI);
  return axisAngleR(ax, Math.atan2(s, c));
}
// Attach a part to a holder (or to the frame with null).
function attachTo(c, a) {
  c.parent = a ? a.id : null;
  // On a rod, a part goes to the rod's far end; a rod put on a servo starts at the servo's pivot.
  const to = a && a.type === 'link' ? linkTip(a) : a && a.type === 'joint' && c.type === 'link' ? a.pos : null;
  if (to) { const d = sub(to, c.pos); c.pos = to.map(v => +v.toFixed(4)); shiftSubtree(c, d); }
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
