'use strict';
// Servo joints. A joint is a hinge mounted on the frame or on another joint. Anything can hang off it:
// motors, rigid masses, cable payloads, sensors, further joints. Every part keeps its position and
// orientation "at rest" (all joint angles zero, in body axes); where it is now comes from rotating it
// about each joint above it, nearest first:  p_now = M_root(… M_parent(p_rest)),  M_j(p) = q_j + R_j(p − q_j).
//
// The physics uses the true joint angles. The flight software uses the angles it believes: the servo's
// feedback reading, or its own prediction from what it commanded (servoPredictor).

function mkJoint(name, x, y, z, o = {}) {
  return base(Object.assign({ type: 'joint', name, pos: [x, y, z], hingeAz: 0, hingeEl: 0, mode: 'auto', manual: 0,
    range: 40, rate: 240, lag: 0.02, offset: 0, feedback: false, mass: 0.015 }, o));
}
// A motor on its own servo joint, both at the same point, the way a tilt-rotor is usually built.
function mkServoMotor(name, x, y, z, jo = {}, mo = {}) {
  const j = mkJoint(name + ' servo', x, y, z, jo);
  const m = mkMotor(name, x, y, z, Object.assign({ parent: j.id }, mo));
  return [j, m];
}

const joints = () => cfg.comps.filter(c => c.type === 'joint');
const compById = id => id == null ? null : cfg.comps.find(c => c.id === id) || null;
function parentJoint(c) { const p = compById(c.parent); return p && p.type === 'joint' ? p : null; }
function jointAxis(j) {   // hinge axis at rest, body frame
  const a = j.hingeAz * D2R, e = (j.hingeEl || 0) * D2R;
  return [Math.cos(e) * Math.cos(a), Math.cos(e) * Math.sin(a), Math.sin(e)];
}
function chainOf(c) {     // joints above a part, nearest first
  const out = [], seen = new Set();
  for (let j = parentJoint(c); j && !seen.has(j.id); j = parentJoint(j)) { seen.add(j.id); out.push(j); }
  return out;
}
const isUnder = (c, j) => chainOf(c).includes(j);
const descendants = j => cfg.comps.filter(c => c !== j && isUnder(c, j));
const canAttach = (c, j) => !!j && j.type === 'joint' && j !== c && !(c.type === 'joint' && isUnder(j, c));
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
function poseOf(c, ang = angleTrue) {
  let R = [1, 0, 0, 0, 1, 0, 0, 0, 1], p = c.pos.slice();
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

/* ───────── saved airframes from before joints existed ───────── */
// A "motor on servo" part becomes a servo joint with a motor attached at the same point.
function migrateTiltParts(comps) {
  const out = [];
  for (const c of comps) {
    if (c.type !== 'tilt') { out.push(c); continue; }
    const j = { id: uid++, type: 'joint', name: c.name + ' servo', pos: c.pos.slice(), hingeAz: c.hingeAz || 0, hingeEl: 0, mode: c.mode || 'auto', manual: c.manual || 0,
      range: c.range ?? 40, rate: c.rate ?? 240, lag: c.lag ?? 0.02, offset: c.offset || 0, feedback: !!c.feedback, mass: 0.015, parent: c.parent };
    const m = Object.assign({}, c, { type: 'motor', tilt: 0, az: 0, parent: j.id, mass: Math.max(0.01, (c.mass ?? 0.075) - 0.015) });
    for (const k of ['hingeAz', 'mode', 'manual', 'range', 'rate', 'lag', 'offset', 'feedback']) delete m[k];
    out.push(j, m);
  }
  return out;
}
