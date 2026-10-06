'use strict';
// Manual flight. Keys and the on-screen pads don't drive the motors directly: they move the target
// the controller is holding, at a commanded velocity that is also fed forward to the position law.
// With a radio (a board runs the telemetry & radio task) they are the handset's sticks and switches instead
// (link.js): the drone moves its own target from them (runner/fc/rc_core.c).
//
//   W / S       climb / descend          ↑ / ↓   forward / back   (relative to the heading)
//   A / D       turn left / right        ← / →   left / right
//   Space       hold here                H       fly home
//   K           pause / run              R       reset
//   1 / 2 / 3   gentle / normal / sport  C       chase camera
//   G           the chosen latch: drop what it holds, or close on what's in reach (cargo-ui.js)

const PILOT_LEVELS = {
  gentle: { label: 'Gentle', h: 1, v: 0.6, yaw: 45 },   // max horizontal m/s, vertical m/s, turn °/s
  normal: { label: 'Normal', h: 3, v: 1.5, yaw: 90 },
  sport: { label: 'Sport', h: 6, v: 3, yaw: 150 },
};
const PILOT_ACCEL = 3;                  // how fast the commanded velocity ramps [m/s²]
const PILOT_BOX = { xy: 25, zMin: 0.15, zMax: 15 };   // (as the drone's own box, rc_core.c: low enough to reach a parcel)
// Two layouts. Handset: as a Mode 2 radio, the left hand climbs and turns (W A S D, the left stick), the right moves
// (the arrows, the right stick); the command module's keys and most drone simulators do this. Game: W A S D move, as
// in games, and the arrows climb and turn.
const KEY_LAYOUTS = {
  handset: { KeyW: 'up', KeyS: 'down', KeyA: 'yawL', KeyD: 'yawR', ArrowUp: 'fwd', ArrowDown: 'back', ArrowLeft: 'left', ArrowRight: 'right' },
  game: { KeyW: 'fwd', KeyS: 'back', KeyA: 'left', KeyD: 'right', ArrowUp: 'up', ArrowDown: 'down', ArrowLeft: 'yawL', ArrowRight: 'yawR' },
};
const KEY_NAMES = { KeyW: 'W', KeyS: 'S', KeyA: 'A', KeyD: 'D', ArrowUp: '↑', ArrowDown: '↓', ArrowLeft: '←', ArrowRight: '→' };
const CTRL_NAMES = { up: 'Climb', down: 'Descend', yawL: 'Turn left', yawR: 'Turn right', fwd: 'Forward', back: 'Back', left: 'Left', right: 'Right' };
let keyLayout = 'handset'; try { if (localStorage.getItem('dfb-keys') === 'game') keyLayout = 'game'; } catch (e) {}
const KEYMAP = { ...KEY_LAYOUTS[keyLayout] };
const keyOf = c => KEY_NAMES[Object.keys(KEYMAP).find(k => KEYMAP[k] === c)];
function setKeyLayout(k, store = true) {
  if (!KEY_LAYOUTS[k]) return;
  releaseAll(); keyLayout = k;
  for (const x of Object.keys(KEYMAP)) delete KEYMAP[x]; Object.assign(KEYMAP, KEY_LAYOUTS[k]);
  if (store) try { localStorage.setItem('dfb-keys', k); } catch (e) {}
  // the pads show their keys; in the game layout the move pad goes on the left, under the hand on W A S D
  document.querySelectorAll('[data-ctrl]').forEach(b => { const c = b.dataset.ctrl, kb = b.querySelector('kbd'); if (kb) kb.textContent = keyOf(c); b.setAttribute('aria-label', `${CTRL_NAMES[c]} (${keyOf(c)})`); });
  const pl = document.querySelector('.pilot'); if (pl) pl.classList.toggle('game', k === 'game');
  document.querySelectorAll('[data-keys]').forEach(b => b.setAttribute('aria-pressed', String(b.dataset.keys === k)));
  const kb = (...cs) => cs.map(c => `<kbd>${keyOf(c)}</kbd>`).join(' ');
  const set = (id, h) => { const e = document.getElementById(id); if (e) e.innerHTML = h; };
  set('keysMove', kb('fwd', 'back', 'left', 'right')); set('keysAlt', kb('up', 'down')); set('keysTurn', kb('yawL', 'yawR'));
  const f = document.getElementById('fwdKey'); if (f) f.textContent = keyOf('fwd');
}
document.querySelectorAll('[data-keys]').forEach(b => b.addEventListener('click', () => setKeyLayout(b.dataset.keys)));
const pilot = { held: new Map(), level: 'normal', vref: [0, 0, 0] };   // held: control -> set of sources

const isHeld = c => pilot.held.has(c);
function markPad(c) { const b = document.querySelector(`[data-ctrl="${c}"]`); if (b) b.setAttribute('aria-pressed', String(isHeld(c))); }
function press(c, src) { if (!pilot.held.has(c)) pilot.held.set(c, new Set()); pilot.held.get(c).add(src); markPad(c); }
function release(c, src) { const s = pilot.held.get(c); if (s) { s.delete(src); if (!s.size) pilot.held.delete(c); } markPad(c); }
function releaseAll() { const cs = [...pilot.held.keys()]; pilot.held.clear(); cs.forEach(markPad); }

function pilotStep(dt) {
  if (S.crashed || dt <= 0) { pilot.vref = [0, 0, 0]; ctl.vRef = [0, 0, 0]; return; }
  if (radioActive()) { pilot.vref = [0, 0, 0]; ctl.vRef = [0, 0, 0]; return; }   // the keys are the handset's sticks: the drone moves its own target
  const sv = brt.superView;   // the supervisor bringing it home or down: the navigation flies that by itself
  if ((sv && sv.mode >= 2) || (brt.navOut && brt.navOut.landed)) { pilot.vref = [0, 0, 0]; ctl.vRef = [0, 0, 0]; return; }
  const L0 = PILOT_LEVELS[pilot.level], L = sv && sv.lim.speed > 0 ? { ...L0, h: Math.min(L0.h, sv.lim.speed) } : L0;   // the supervisor's speed limit
  const f = (isHeld('fwd') ? 1 : 0) - (isHeld('back') ? 1 : 0);
  const l = (isHeld('left') ? 1 : 0) - (isHeld('right') ? 1 : 0);
  const u = (isHeld('up') ? 1 : 0) - (isHeld('down') ? 1 : 0);
  const y = (isHeld('yawL') ? 1 : 0) - (isHeld('yawR') ? 1 : 0);
  if (brt.pickup && (f || l || u || y)) pickupStop();   // the keys take it back from a pickup (with a radio, the drone sees the sticks)
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
  if (!est.havePos) return;   // no position estimate (no navigation): nothing to hold
  if (radioActive()) { radioHold(); flashCtl('hold'); return; }   // the hold switch on the handset
  const hub = est.p;
  setpoint.x = clamp(hub[0], -PILOT_BOX.xy, PILOT_BOX.xy); setpoint.y = clamp(hub[1], -PILOT_BOX.xy, PILOT_BOX.xy);
  setpoint.z = clamp(hub[2], PILOT_BOX.zMin, PILOT_BOX.zMax);
  pilot.vref = [0, 0, 0]; ctl.vRef = [0, 0, 0];
  flashCtl('hold');
}
function pilotHome() { if (!hasTask("nav")) return; if (radioActive()) { radioHome(); flashCtl('home'); return; } const h = brt.home || spawnAt; setpoint.x = h[0]; setpoint.y = h[1]; setpoint.z = h[2] + 1.5; pilot.vref = [0, 0, 0]; ctl.vRef = [0, 0, 0]; flashCtl('home'); }
function setPilotLevel(k) {
  pilot.level = k;
  document.querySelectorAll('[data-level]').forEach(b => b.setAttribute('aria-pressed', String(b.dataset.level === k)));
}
function flashCtl(id) { const b = document.querySelector(`[data-act="${id}"]`); if (!b) return; b.classList.remove('flash'); void b.offsetWidth; b.classList.add('flash'); }

// Keys. Ignored while typing in a text field or the formula editor, and when ⌘/Ctrl/Alt is held.
const typingIn = t => t && (t.isContentEditable || t.tagName === 'TEXTAREA' || t.tagName === 'SELECT' ||
  (t.tagName === 'INPUT' && !['range', 'checkbox', 'radio', 'button'].includes(t.type)));
// Does the focused control use this key itself? Then the flight keys leave it alone: the arrows move a slider or
// the tabs, Space and Enter press a button or tick a box. Letters still fly from a button, so a click on one
// doesn't take the keyboard away.
function ownsKey(t, e) {
  if (!t || t === document.body || t === document.documentElement || !t.tagName) return false;
  if (typingIn(t)) return true;
  const k = e.key, nav = /^(Arrow|Page)/.test(k) || k === 'Home' || k === 'End', act = k === ' ' || k === 'Enter' || k === 'Spacebar';
  const role = t.getAttribute('role');
  if (t.tagName === 'INPUT') return t.type === 'range' ? nav : t.type === 'radio' ? nav || act : act;
  if (role === 'tab' || role === 'menuitem' || role === 'option' || role === 'slider' || role === 'radio') return nav || act;
  if (t.tagName === 'BUTTON' || t.tagName === 'SUMMARY' || role === 'button' || (t.tagName === 'A' && t.hasAttribute('href'))) return act;
  if (t.hasAttribute('tabindex') && t.tabIndex >= 0 && t.tagName !== 'CANVAS') return act || nav;
  return false;
}
let spaceHold = false;   // Space went to Hold, not to a control: its keyup mustn't click anything
window.addEventListener('keydown', e => {
  if (e.defaultPrevented || e.metaKey || e.ctrlKey || e.altKey || ownsKey(e.target, e) || document.querySelector('dialog[open]')) return;
  const c = KEYMAP[e.code];
  if (c) { e.preventDefault(); if (!e.repeat) press(c, 'key:' + e.code); return; }
  if (e.repeat) { if (e.code === 'Space') e.preventDefault(); return; }
  if (e.code === 'Space') { e.preventDefault(); spaceHold = true; pilotHold(); }
  else if (e.code === 'KeyH') pilotHome();
  else if (e.code === 'KeyG' && typeof cargoKey === 'function') cargoKey();   // the chosen latch: drop, or grab
  else if (e.code === 'KeyC') document.getElementById('tChase').click();
  else if (e.code === 'KeyQ' && typeof toggleTorque === 'function') toggleTorque();
  else if (e.code === 'KeyT' && typeof setLaunch === 'function' && hasTask('learn')) setLaunch('throw');   // the throw start needs the learning task
  else if (e.code === 'KeyK') document.getElementById('runBtn').click();
  else if (e.code === 'KeyR') document.getElementById('resetBtn').click();
  else if (e.code === 'Digit1') setPilotLevel('gentle');
  else if (e.code === 'Digit2') setPilotLevel('normal');
  else if (e.code === 'Digit3') setPilotLevel('sport');
});
window.addEventListener('keyup', e => {
  const c = KEYMAP[e.code]; if (c) release(c, 'key:' + e.code);
  if (e.code === 'Space' && spaceHold) { spaceHold = false; e.preventDefault(); }
});
window.addEventListener('blur', releaseAll);
document.addEventListener('visibilitychange', () => { if (document.hidden) releaseAll(); });

// On-screen pads: press and hold with mouse or touch.
function bindPads() {
  setKeyLayout(keyLayout, false);
  document.querySelectorAll('[data-ctrl]').forEach(b => {
    const c = b.dataset.ctrl;
    b.addEventListener('pointerdown', e => { e.preventDefault(); try { b.setPointerCapture(e.pointerId); } catch (x) {} press(c, 'ptr:' + e.pointerId); });
    const up = e => release(c, 'ptr:' + e.pointerId);
    b.addEventListener('pointerup', up); b.addEventListener('pointercancel', up); b.addEventListener('lostpointercapture', up);
    b.addEventListener('contextmenu', e => e.preventDefault());
    // from the keyboard: Enter or Space holds it down like a press
    const isAct = e => e.key === 'Enter' || e.key === ' ';
    b.addEventListener('keydown', e => { if (!isAct(e)) return; e.preventDefault(); if (!e.repeat) press(c, 'kbd'); });
    b.addEventListener('keyup', e => { if (!isAct(e)) return; e.preventDefault(); release(c, 'kbd'); });
    b.addEventListener('blur', () => release(c, 'kbd'));
  });
  // A mouse click on the flight buttons leaves the keyboard where it was (Space still holds, not presses them again).
  document.querySelectorAll('.pilot-mid button').forEach(b => b.addEventListener('mousedown', e => e.preventDefault()));
  document.querySelector('[data-act="hold"]').addEventListener('click', pilotHold);
  document.querySelector('[data-act="home"]').addEventListener('click', pilotHome);
  document.querySelectorAll('[data-level]').forEach(b => b.addEventListener('click', () => setPilotLevel(b.dataset.level)));
  setPilotLevel(pilot.level);
}
