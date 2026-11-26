'use strict';
// Tune → Sensor calibration: the IMU's mount and its accelerometer's offsets and gains, from still poses (the maths:
// sensor-cal.js). Each Measure sends "imucal" up the radio, as the drone card's typed commands go (cardTextOut): the
// drone's flight controller (the ESP32's flight.c, the simulator's board_wasm.c) averages a second of its IMU as the
// chip reads it and answers down the radio. Apply keeps the result in the design: the mount on the IMU part (Airframe:
// the airframe's imu_R), the offsets and gains with its wiring (acc_bias=, acc_scale=, gyro_bias=), and Level set to
// the mount (level=off), so whichever way the drone sits at power-up (on a rope, on a slope) it knows which way is up.
// On the real drone, Upload design puts it there.

const scal = { poses: {}, busy: '', msg: '', tone: '', applied: false };
const scalImu = () => sensorsOf('imu').find(c => c.known) || sensorsOf('imu')[0] || null;
const scalWhere = () => typeof liveOn === 'function' && liveOn() ? (live.state === 'up' ? 'live' : 'down') : 'sim';
const scalResult = () => scal.poses.flat && scal.poses.nose ? SensorCal.solve(scal.poses) : null;
const scalNote = (msg, tone = '') => { scal.msg = msg; scal.tone = tone; renderSensorCal(); };

// Ask the drone for one pose: send imucal, wait for its answer (wall time: a paused simulator answers nothing).
// The pose, { error } (the drone's refusal), { why } (not sent), or null (no answer).
async function scalAsk() {
  const why = cardTextOut('imucal'); if (why) return { why };
  const n0 = gs.nmsg || 0, t0 = performance.now();
  while (performance.now() - t0 < 8000) {
    await new Promise(r => setTimeout(r, 80));
    const k = Math.min(gs.log.length, (gs.nmsg || 0) - n0);
    for (const m of gs.log.slice(0, Math.max(0, k)).reverse()) { const a = SensorCal.parseAnswer(m.text); if (a) return a; }
  }
  return null;
}
async function scalMeasure(id) {
  if (scal.busy) return;
  scal.busy = id; scal.applied = false; scalNote('Measuring: keep it still…');
  const a = await scalAsk(); scal.busy = '';
  if (!a) { scalNote(scalWhere() === 'sim' ? 'No answer: is the simulation running (not paused)?' : 'No answer from the drone: is the link up, and its firmware new enough for imucal?', 'bad'); return; }
  if (a.why) { scalNote(a.why, 'bad'); return; }
  if (a.error) { scalNote(`${SensorCal.POSES.find(p => p.id === id).label}: ${a.error}`, 'bad'); return; }
  scal.poses[id] = a;
  const next = SensorCal.POSES.find(p => !scal.poses[p.id]);
  scalNote(next ? `Measured. Next: ${next.label}.` : 'All poses measured.', '');
}
function scalApply() {
  const r = scalResult(), c = scalImu();
  if (!r || r.error || !c) return;
  // the part's mount, so that its pose on the drone times the mount is the measured turn (sensor → body)
  const P = poseOf(c, restAngle).R;
  c.mount = eulerFromR(m3m(m3T(P), r.mount)).map(x => +x.toFixed(2)); c.known = true;
  edited(c, 'mount');
  const k = { accBias: r.bias.map(x => +x.toFixed(5)), accScale: r.scale.map(x => +x.toFixed(5)), gyroBias: r.gyro.map(x => +x.toFixed(6)), fitted: r.fitted, at: Date.now() };
  editWiring(w => { w.parts[c.id] = { ...w.parts[c.id], cal: k, level: 'off', levelYaw: 0 }; }, c.id);
  scal.applied = true;
  scalNote(scalWhere() === 'live'
    ? 'Saved in the design: the IMU\'s mount (Airframe), its offsets and gains, and Level as the mount. Upload design puts it on the drone (its ESP32 restarts).'
    : 'Saved in the design: the IMU\'s mount (Airframe), its offsets and gains, and Level as the mount. The simulator flies it now; on the real drone, Upload design (connected) or Install puts it there.', 'ok');
}
function scalUpload() {
  const why = typeof cardUploadStart === 'function' ? cardUploadStart() : 'no drone card';
  scalNote(why || 'Uploading the design: the drone card shows how it goes.', why ? 'bad' : 'ok');
}

function renderSensorCal() {
  const box = $('#sensorCal'); if (!box) return;
  const c = scalImu(), where = scalWhere(), r = scalResult();
  const n = SensorCal.POSES.filter(p => scal.poses[p.id]).length;
  setText($('#sensorCalSum'), !c ? 'no IMU' : scal.applied ? 'applied' : n ? `${n} of ${SensorCal.POSES.length} poses` : computers().wiring?.parts?.[c.id]?.cal ? 'calibrated' : '');
  const focus = document.activeElement && box.contains(document.activeElement) ? document.activeElement.dataset.focusKey : null;
  box.textContent = '';
  box.append(el('p', { class: 'hint', text: 'Measures which way the IMU board really sits (its mount, the heading too) and its accelerometer\'s offset and gain on every axis, from a few still poses. Props off. Hold each pose still for a second while it measures.' }));
  if (!c) { box.append(el('p', { class: 'hint warn', text: 'This design has no IMU (add one in Airframe).' })); return; }
  box.append(el('p', { class: 'hint' + (where === 'down' ? ' warn' : ''), text: where === 'live' ? 'Connected to the real drone: the poses are its IMU\'s.' : where === 'down' ? 'Not connected to the drone yet (Ground tab).' : 'In the simulator the drone sits level on the ground, so only the level poses can be measured here. Connect to the real drone (Ground tab) to calibrate it.' }));
  const guiding = typeof scalG !== 'undefined' && scalG.on;
  box.append(el('div', { class: 'design-tools' }, guiding
    ? UI.button({ class: 'btn sm', type: 'button', 'data-focus-key': 'scal-guide', text: 'Stop the guide', onclick: scalGStop })
    : UI.button({ class: 'btn sm primary', type: 'button', 'data-focus-key': 'scal-guide', disabled: !!scal.busy || where === 'down' || undefined, text: 'Guided calibration',
      title: 'The 3D view shows each pose; the drone is asked as you go, and each pose is taken once it\'s right and still', onclick: scalGuideStart })),
    el('p', { class: 'hint', text: guiding ? 'Guiding: follow the drone in the 3D view.' : 'Or measure the poses one by one:' }));
  const list = el('ol', { class: 'scal-poses' });
  for (const p of SensorCal.POSES) {
    const got = scal.poses[p.id], busy = scal.busy === p.id;
    const g = got ? Math.hypot(...got.acc) / G : 0;
    const btn = UI.button({ class: 'btn sm', type: 'button', 'data-focus-key': 'scal-' + p.id, disabled: !!scal.busy || where === 'down' || undefined,
      text: busy ? 'Measuring…' : guiding && SensorCal.POSES[scalG.i] === p ? 'Guiding…' : got ? 'Again' : 'Measure', onclick: () => scalMeasure(p.id) });
    list.append(el('li', { class: 'scal-pose' + (got ? ' is-done' : '') },
      el('div', { class: 'scal-pose-h' }, el('b', { text: p.label }), el('small', { text: p.need ? 'needed' : p.id === 'again' ? 'checks the others' : 'for the offsets and gains' }), btn),
      el('p', { class: 'hint', text: p.how }),
      got ? el('p', { class: 'hint scal-got', text: `✓ reads ${g.toFixed(3)} g` }) : null));
  }
  box.append(list);
  if (scal.msg) box.append(el('p', { class: 'ui-status' + (scal.tone ? ' ' + scal.tone : ''), role: 'status', text: scal.msg }));
  if (r) {
    const res = el('div', { class: 'scal-result' });
    if (r.error) res.append(el('p', { class: 'ui-status warn', text: r.error }));
    else {
      const mount = eulerFromR(m3m(m3T(poseOf(c, restAngle).R), r.mount)), f = (a, m, d) => a.map(x => (x * m).toFixed(d)).join(', ');
      res.append(
        el('p', { text: `Mount: roll ${mount[0].toFixed(1)}°, pitch ${mount[1].toFixed(1)}°, yaw ${mount[2].toFixed(1)}° (Airframe has ${c.mount.map(x => (+x).toFixed(1) + '°').join(', ')}).` }),
        el('p', { text: `Accelerometer: offsets ${f(r.bias, 1000 / G, 0)} mg, gains ${f(r.scale, 1, 4)}${r.fitted ? '' : ' (one gain for all three)'}. Gyro offset ${f(r.gyro, R2D, 2)} °/s.` }),
        el('p', { class: 'hint', text: `After it, the poses read 1 g within ${(Math.abs(r.worst.err) * 100).toFixed(2)}% (${r.worst.label} the furthest).${r.note ? ' ' + r.note : ''}` }));
      if (!r.ok) res.append(el('p', { class: 'ui-status warn', text: `${r.worst.label} reads ${(r.worst.err * 100).toFixed(1)}% off 1 g: it moved, or the poses disagree. Measure it again.` }));
    }
    box.append(res);
  }
  const tools = el('div', { class: 'design-tools' },
    UI.button({ class: 'btn sm', type: 'button', 'data-focus-key': 'scal-apply', disabled: !r || !!r.error || !r.ok || scal.applied || undefined, text: 'Apply', title: 'Keep the mount, offsets and gains in the design', onclick: scalApply }),
    UI.button({ class: 'btn sm', type: 'button', 'data-focus-key': 'scal-clear', disabled: !n || !!scal.busy || undefined, text: 'Start over', onclick: () => { scal.poses = {}; scal.applied = false; scalNote(''); if (typeof scalG !== 'undefined' && scalG.done) { scalG.done = false; scalGRender(); } } }));
  if (scal.applied && where === 'live' && typeof cardRadioPi === 'function' && cardRadioPi()) tools.append(UI.button({ class: 'btn sm', type: 'button', 'data-focus-key': 'scal-upload', text: 'Upload design', onclick: scalUpload }));
  box.append(tools);
  if (focus) { const f = box.querySelector(`[data-focus-key="${focus}"]`); if (f && !f.disabled) f.focus(); }
}
// While it's open: the connection's note and buttons follow the link (connected, lost, the simulator).
{ let seen = ''; setInterval(() => { const f = $('#sensorCalFold'); if (!f || !f.open) return; const w = scalWhere() + (scal.busy ? 1 : 0); if (w !== seen) { seen = w; renderSensorCal(); } }, 1000);
  document.addEventListener('toggle', e => { if (e.target && e.target.id === 'sensorCalFold' && e.target.open) renderSensorCal(); }, true); }
