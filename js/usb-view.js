'use strict';
// The board on the install dialog's USB cable, in the 3D view (Computers → a board → Install / export → Its messages →
// Show it in the 3D view): the drone takes the attitude and height in the flight firmware's telemetry (RN_LINK_TELEM,
// flight.c; as fly.py watch reads it), so tilting the board on the bench tilts the drone. It only watches: nothing goes
// to the board, no sticks, no arming. Any ESP32 (the WROOM too: no Bluetooth needed). The simulation pauses meanwhile;
// Stop, Run or a Bluetooth connection ends it, and so does unplugging the board or disconnecting it in the dialog.

const usbView = { on: false, f: null, at: 0, name: '', target: null, wasRunning: true, barNext: 0 };
const usbViewOn = () => usbView.on;
function usbViewTelem(f) { usbView.f = Float32Array.from(f); usbView.at = performance.now(); }
function usbViewStart() {
  if (usbView.on || !INST.conn || INST.connTarget === 'ground' || (typeof liveOn === 'function' && liveOn())) return;
  Object.assign(usbView, { on: true, target: INST.connTarget, name: instName(INST.connTarget), wasRunning: running, barNext: 0 });
  if (editMode) setEditMode(false);
  running = false; renderRun();
  document.querySelector('.view').classList.add('live');
  $('#usbBar').hidden = false; usbViewBar(true);
}
// resume: false leaves the simulation paused (Run, which starts it itself; a Bluetooth connection).
function usbViewStop(resume) {
  if (!usbView.on) return;
  usbView.on = false; usbView.f = null;
  $('#usbBar').hidden = true; document.querySelector('.view').classList.remove('live');
  doReset();                                                         // (the simulated drone back where it starts)
  if (resume !== false) { running = usbView.wasRunning; renderRun(); }
}
// Each frame: the drone's attitude and height from the newest telemetry (the angles as flight.c sends them: roll and
// pitch as the flight core's, the heading its yaw; the barometer's height above where it was switched on).
function usbViewFrame() {
  const f = usbView.f, home = spawnAt || [0, 0, 0], d = Math.PI / 180;
  if (f) {
    S.q = qnorm(liveQuat({ roll: f[2] * d, pitch: f[3] * d, yaw: f[4] * d }));
    const baro = (f[13] & 2) !== 0;
    S.p = [home[0], home[1], home[2] + (baro ? Math.max(0, f[8]) : 0)];
    S.v = [0, 0, baro ? f[9] : 0];
  }
  usbViewBar();
}
function usbViewBar(force) {
  const now = performance.now(); if (!force && now < usbView.barNext) return; usbView.barNext = now + 200;
  const fresh = usbView.f && now - usbView.at < 1000, st = $('#usbState');
  setText($('#usbName'), usbView.name);
  setText(st, !INST.conn ? 'disconnected' : fresh ? 'connected' : 'connected, no telemetry');
  st.dataset.tone = !INST.conn ? 'bad' : fresh ? 'good' : 'warn';
  setText($('#usbRead'), usbView.f ? instTelemText(usbView.f) : 'no telemetry yet: is it the flight firmware, at this speed?');
}
(function usbViewBind() {
  if (!$('#usbBar')) return;
  $('#usbStop').addEventListener('click', () => usbViewStop());
  $('#usbConsole').addEventListener('click', () => { if (usbView.target) openInstall(usbView.target); });
  document.querySelectorAll('#usbBar button').forEach(x => x.addEventListener('mousedown', e => e.preventDefault()));
})();
