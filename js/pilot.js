'use strict';
// Manual flight. Keys and the on-screen pads don't drive the motors directly: they move the target
// the controller is holding, at a commanded velocity that is also fed forward to the position law.
//
//   W / S       climb / descend          ↑ / ↓   forward / back   (relative to the heading)
//   A / D       turn left / right        ← / →   left / right
//   Space       hold here                H       fly home
//   K           pause / run              R       reset
//   1 / 2 / 3   gentle / normal / sport  C       chase camera

const PILOT_LEVELS = {
  gentle: { label: 'Gentle', h: 1, v: 0.6, yaw: 45 },   // max horizontal m/s, vertical m/s, turn °/s
  normal: { label: 'Normal', h: 3, v: 1.5, yaw: 90 },
  sport: { label: 'Sport', h: 6, v: 3, yaw: 150 },
};
const PILOT_ACCEL = 3;                  // how fast the commanded velocity ramps [m/s²]
const PILOT_BOX = { xy: 25, zMin: 0.3, zMax: 15 };
const KEYMAP = { KeyW: 'up', KeyS: 'down', KeyA: 'yawL', KeyD: 'yawR', ArrowUp: 'fwd', ArrowDown: 'back', ArrowLeft: 'left', ArrowRight: 'right' };
const pilot = { held: new Map(), level: 'normal', vref: [0, 0, 0] };   // held: control -> set of sources

const isHeld = c => pilot.held.has(c);
function markPad(c) { const b = document.querySelector(`[data-ctrl="${c}"]`); if (b) b.setAttribute('aria-pressed', String(isHeld(c))); }
function press(c, src) { if (!pilot.held.has(c)) pilot.held.set(c, new Set()); pilot.held.get(c).add(src); markPad(c); }
function release(c, src) { const s = pilot.held.get(c); if (s) { s.delete(src); if (!s.size) pilot.held.delete(c); } markPad(c); }
function releaseAll() { const cs = [...pilot.held.keys()]; pilot.held.clear(); cs.forEach(markPad); }

function pilotStep(dt) {
  if (S.crashed || dt <= 0) { pilot.vref = [0, 0, 0]; ctl.vRef = [0, 0, 0]; return; }
  if (typeof autoPilotStep === 'function' && autoPilotStep(dt)) return;   // the supervisor is bringing it home or down
  if (fc.mode === 'landed') { pilot.vref = [0, 0, 0]; ctl.vRef = [0, 0, 0]; return; }
  const L0 = PILOT_LEVELS[pilot.level], L = fc.lim.speed ? { ...L0, h: Math.min(L0.h, fc.lim.speed) } : L0;   // the supervisor's speed limit
  const f = (isHeld('fwd') ? 1 : 0) - (isHeld('back') ? 1 : 0);
  const l = (isHeld('left') ? 1 : 0) - (isHeld('right') ? 1 : 0);
  const u = (isHeld('up') ? 1 : 0) - (isHeld('down') ? 1 : 0);
  const y = (isHeld('yawL') ? 1 : 0) - (isHeld('yawR') ? 1 : 0);
  const psi = setpoint.yaw * D2R;
  let hx = f * Math.cos(psi) - l * Math.sin(psi), hy = f * Math.sin(psi) + l * Math.cos(psi);
  const hn = Math.hypot(hx, hy); if (hn > 1) { hx /= hn; hy /= hn; }
  const want = [hx * L.h, hy * L.h, u * L.v];
  const dv = PILOT_ACCEL * dt;
  for (let i = 0; i < 3; i++) pilot.vref[i] += clamp(want[i] - pilot.vref[i], -dv, dv);

  const nx = setpoint.x + pilot.vref[0] * dt, ny = setpoint.y + pilot.vref[1] * dt, nz = setpoint.z + pilot.vref[2] * dt;
  setpoint.x = clamp(nx, -PILOT_BOX.xy, PILOT_BOX.xy); if (setpoint.x !== nx) pilot.vref[0] = 0;
  setpoint.y = clamp(ny, -PILOT_BOX.xy, PILOT_BOX.xy); if (setpoint.y !== ny) pilot.vref[1] = 0;
  setpoint.z = clamp(nz, PILOT_BOX.zMin, PILOT_BOX.zMax); if (setpoint.z !== nz) pilot.vref[2] = 0;
  if (y) { let a = setpoint.yaw + y * L.yaw * dt; a = ((a + 180) % 360 + 360) % 360 - 180; setpoint.yaw = a; }
  ctl.vRef = pilot.vref.slice();
}
function pilotHold() {   // hold where the flight software believes it is
  const hub = sensing === 'truth' ? hubState(qmat(S.q)).hub : est.p;
  setpoint.x = clamp(hub[0], -PILOT_BOX.xy, PILOT_BOX.xy); setpoint.y = clamp(hub[1], -PILOT_BOX.xy, PILOT_BOX.xy);
  setpoint.z = clamp(hub[2], PILOT_BOX.zMin, PILOT_BOX.zMax);
  pilot.vref = [0, 0, 0]; ctl.vRef = [0, 0, 0];
  flashCtl('hold');
}
function pilotHome() { setpoint.x = 0; setpoint.y = 0; setpoint.z = 1.5; setpoint.yaw = 0; pilot.vref = [0, 0, 0]; ctl.vRef = [0, 0, 0]; flashCtl('home'); }
function setPilotLevel(k) {
  pilot.level = k;
  document.querySelectorAll('[data-level]').forEach(b => b.setAttribute('aria-pressed', String(b.dataset.level === k)));
}
function flashCtl(id) { const b = document.querySelector(`[data-act="${id}"]`); if (!b) return; b.classList.remove('flash'); void b.offsetWidth; b.classList.add('flash'); }

// Keys. Ignored while typing in a text field or the formula editor, and when ⌘/Ctrl/Alt is held.
const typingIn = t => t && (t.isContentEditable || t.tagName === 'TEXTAREA' || t.tagName === 'SELECT' ||
  (t.tagName === 'INPUT' && !['range', 'checkbox', 'radio', 'button'].includes(t.type)));
window.addEventListener('keydown', e => {
  if (e.metaKey || e.ctrlKey || e.altKey || typingIn(e.target) || document.querySelector('dialog[open]')) return;
  const c = KEYMAP[e.code];
  if (c) { e.preventDefault(); if (!e.repeat) press(c, 'key:' + e.code); return; }
  if (e.repeat) { if (e.code === 'Space') e.preventDefault(); return; }
  if (e.code === 'Space') { e.preventDefault(); pilotHold(); }
  else if (e.code === 'KeyH') pilotHome();
  else if (e.code === 'KeyC') document.getElementById('tChase').click();
  else if (e.code === 'KeyQ' && typeof toggleTorque === 'function') toggleTorque();
  else if (e.code === 'KeyT' && typeof setLaunch === 'function') setLaunch('throw');
  else if (e.code === 'KeyK') document.getElementById('runBtn').click();
  else if (e.code === 'KeyR') document.getElementById('resetBtn').click();
  else if (e.code === 'Digit1') setPilotLevel('gentle');
  else if (e.code === 'Digit2') setPilotLevel('normal');
  else if (e.code === 'Digit3') setPilotLevel('sport');
});
window.addEventListener('keyup', e => {
  const c = KEYMAP[e.code]; if (c) release(c, 'key:' + e.code);
  if (e.code === 'Space' && !typingIn(e.target)) e.preventDefault();   // stop a focused button from clicking
});
window.addEventListener('blur', releaseAll);
document.addEventListener('visibilitychange', () => { if (document.hidden) releaseAll(); });

// On-screen pads: press and hold with mouse or touch.
function bindPads() {
  document.querySelectorAll('[data-ctrl]').forEach(b => {
    const c = b.dataset.ctrl;
    b.addEventListener('pointerdown', e => { e.preventDefault(); try { b.setPointerCapture(e.pointerId); } catch (x) {} press(c, 'ptr:' + e.pointerId); });
    const up = e => release(c, 'ptr:' + e.pointerId);
    b.addEventListener('pointerup', up); b.addEventListener('pointercancel', up); b.addEventListener('lostpointercapture', up);
    b.addEventListener('contextmenu', e => e.preventDefault());
  });
  document.querySelector('[data-act="hold"]').addEventListener('click', pilotHold);
  document.querySelector('[data-act="home"]').addEventListener('click', pilotHome);
  document.querySelectorAll('[data-level]').forEach(b => b.addEventListener('click', () => setPilotLevel(b.dataset.level)));
  setPilotLevel(pilot.level);
}
