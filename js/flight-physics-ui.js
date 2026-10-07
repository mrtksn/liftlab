'use strict';
// Optional physical profiles and compact live diagnostics; refreshed at the UI rate.
let flightBatteryFieldRefs = [];
function flightPhysicsEdited(c) {
  undoKey = `physics:${c?.id ?? 'battery'}`;
  recomputeProps(); cPts = contactPoints(); rebuildDrone(); refreshEnvelope(); renderMass(); save();
  if (c) refreshCard(c); buildActRows();
}
function flightRerenderParts() {
  const open = [...document.querySelectorAll('#compList details[open][id]')].map(node => node.id);
  keepFocus(() => { renderComps1(); for (const id of open) { const node = document.getElementById(id); if (node) node.open = true; } });
}
function flightNumber(owner, key, label, min, max, step, unit, fallback, changed, prefix = 'physics') {
  const field = numField(`${prefix}-${key}`, {label, min, max, hard: true, step, u: unit, dp: key === 'inertia' ? 8 : step < .001 ? 6 : step < .01 ? 4 : step < 1 ? 2 : 0},
    () => owner[key] ?? (typeof fallback === 'function' ? fallback() : fallback), v => { owner[key] = v; changed(); });
  if (owner === envr) spRefs.push(field.refresh);
  if (prefix === 'battery-physics') flightBatteryFieldRefs.push(field.refresh);
  return field.node;
}
function flightCurveEditor(owner, key, title, header, validate, changed, prefix) {
  const area = el('textarea', {rows: 5, class: 'flight-curve', 'aria-label': title, placeholder: header});
  area.value = owner[key] ? owner[key].map(r => r.join(', ')).join('\n') : '';
  const status = el('p', {class: 'hint', role: 'status', text: owner[key] ? 'Imported data; check its units and test conditions.' : 'No imported data. Generic model in use.'});
  const apply = UI.button({type: 'button', class: 'btn sm', text: 'Apply data'});
  apply.addEventListener('click', () => {
    try {
      const rows = validate(area.value); owner[key] = rows;
      status.textContent = `${rows.length} rows applied.`; changed(true);
    } catch (e) { status.textContent = e.message; }
  });
  const clear = UI.button({type: 'button', class: 'btn sm', text: 'Use generic model'});
  clear.addEventListener('click', () => { delete owner[key]; area.value = ''; status.textContent = 'No imported data. Generic model in use.'; changed(false); });
  const source = el('input', {type: 'text', id: `${prefix}-source`, maxlength: 160, value: owner.source || ''});
  source.addEventListener('change', () => { owner.source = source.value; changed(false); });
  return UI.details({title, id: `${prefix}-${key}-fold`, class: 'fold sub'}, el('p', {class: 'hint', text: header}), area,
    el('div', {class: 'addrow'}, apply, clear), status,
    el('div', {class: 'field'}, el('label', {for: source.id, text: 'Data source / test conditions'}), source));
}
function flightFixedMotor(c, kind) {
  const mp = flightMotor(c);
  c.motorPhysics = {...c.motorPhysics, kind, kv: 60 / (2 * Math.PI * mp.Ke), resistance: mp.R,
    currentLimit: mp.iMax, inertia: mp.J, friction: mp.friction, brushDrop: kind === 'brushed' ? .6 : 0};
}
function flightPhysicsFields(c) {
  let changed = () => flightPhysicsEdited(c); const prefix = `physics-${c.id}`;
  const box = UI.details({title: 'Physical model', id: `${prefix}-fold`, class: 'fold sub'});
  const num = (owner, key, label, min, max, step, unit, fallback) => flightNumber(owner, key, label, min, max, step, unit, fallback, changed, prefix);
  if (c.type === 'motor') {
    const mp = flightMotor(c), prop = c.propPhysics || {}; let p = c.motorPhysics || {};
    changed = () => { c.motorPhysics = p; c.propPhysics = prop; flightPhysicsEdited(c); };
    const select = UI.select({id: `${prefix}-kind`, 'aria-label': 'Motor model'});
    for (const [v, label] of [['generic', 'Generic motor (estimated)'], ['brushless', 'Fixed brushless motor'], ['brushed', 'Fixed brushed motor']]) select.append(el('option', {value: v, text: label}));
    select.value = mp.kind;
    select.addEventListener('change', () => { if (select.value === 'generic') { p.kind = 'generic'; delete prop.rows; } else { flightFixedMotor(c, select.value); p = c.motorPhysics; } changed(); flightRerenderParts(); });
    box.append(el('div', {class: 'field'}, el('label', {for: select.id, text: 'Motor model'}), select));
    box.append(el('p', {class: 'hint', text: mp.kind === 'generic' ? 'KV, resistance and inertia are estimated from thrust and prop size. Select a fixed motor to keep its electrical traits when changing props.' : 'Max thrust is derived at 16 V and 1.225 kg/m³. Changing the prop changes load on this motor. Initial values are estimates; enter measured motor specifications. Reset to load the new controller description.'}));
    if (mp.kind !== 'generic') box.append(
      num(p, 'kv', 'Motor KV', 10, 20000, 10, 'RPM/V', 60 / (2 * Math.PI * mp.Ke)),
      num(p, 'resistance', 'Equivalent winding resistance', .005, 20, .005, 'Ω', mp.R),
      num(p, 'currentLimit', 'Winding current limit', .1, 500, .5, 'A', mp.iMax),
      num(p, 'inertia', 'Motor + prop inertia', 1e-8, .1, .000001, 'kg·m²', mp.J),
      num(p, 'friction', 'Mechanical friction torque', 0, 1, .001, 'N·m', 0),
      num(prop, 'ct', 'Generic prop thrust coefficient', .01, .5, .005, '', .1));
    if (mp.kind === 'brushed') box.append(num(p, 'brushDrop', 'Brush voltage drop', 0, 3, .05, 'V', .6));
    box.append(num(p, 'escEfficiency', 'ESC / driver efficiency', .5, 1, .01, '', .97),
      num(p, 'idleW', 'ESC idle draw', 0, 20, .05, 'W', .1),
      num(p, 'maxRpm', 'Manufacturer RPM limit (0: unknown)', 0, 200000, 100, 'RPM', 0),
      num(p, 'heatCapacity', 'Motor heat capacity', .1, 10000, .1, 'J/K', mp.thermal.C),
      num(p, 'cooling', 'Full-speed thermal conductance', .001, 100, .01, 'W/K', mp.thermal.G));
    box.append(flightCurveEditor(prop, 'rows', 'Measured fixed-pitch prop data', 'RPM, thrust N, torque N·m; 2–64 increasing RPM rows. Data applies to this prop radius. Changing radius clears it. Static data does not identify forward-flight or compressibility behavior.', text => {
      if (isCollective(c)) throw new Error('Use fixed pitch for static prop data. Collective pitch needs a pitch-dependent map.');
      return FlightPhysics.parsePropTable(text);
    }, applied => { const switchModel = applied && mp.kind === 'generic'; if (applied) { if (switchModel) { flightFixedMotor(c, 'brushless'); p = c.motorPhysics; } prop.radius = propR(c); } changed(); if (switchModel) flightRerenderParts(); }, prefix));
    box.append(num(prop, 'referenceDensity', 'Prop test air density', .3, 2, .005, 'kg/m³', 1.225));
    const summary = el('p', {class: 'hint'}), refresh = () => {
      const current = flightMotor(c);
      setText(summary, `Reference RPM: ${Math.round(current.Om * 60 / (2 * Math.PI))}; ${current.kind === 'generic' ? 'estimated ' : ''}KV: ${Math.round(60 / (2 * Math.PI * current.Ke))}. Sonic tip RPM here: ${Math.round(flightAtmosphere().sound * 60 / (2 * Math.PI * propR(c)))}. Sonic speed is not a safe operating limit. RPM limits generate a warning.`);
    };
    refresh(); cardRefresh.get(c.id)?.push(refresh); box.append(summary);
  } else if (c.type === 'joint') {
    const p = c.servoPhysics || {};
    changed = () => { c.servoPhysics = p; flightPhysicsEdited(c); };
    box.append(num(p, 'idleW', 'Servo idle draw', 0, 20, .05, 'W', .15),
      num(p, 'stallW', 'Additional electrical loss at stall', 0, 100, .5, 'W', 5),
      num(p, 'efficiency', 'Servo mechanical efficiency', .1, 1, .05, '', .65));
  } else if (c.type === 'mass') {
    if (c.battery) {
      const check = UI.input({type: 'checkbox', id: `${prefix}-auto`}); check.checked = c.batteryAutoMass !== false;
      check.addEventListener('change', () => { c.batteryAutoMass = check.checked; delete c.batterySizing; changed(); });
      box.append(el('label', {class: 'check', for: check.id}, check, 'Scale battery mass with capacity and cell count'),
        el('p', {class: 'hint', text: 'Scaling starts from this pack’s saved mass, capacity and cell count. Editing mass switches to a manual weight.'}));
    }
    if (isWing(c)) box.append(flightPolarEditor(c, changed, prefix));
    else box.append(num(c, 'dragCd', 'Body drag coefficient', .05, 3, .05, '', 1.05));
  }
  box.append(num(c, 'deviceW', 'Additional powered-device draw', 0, 200, .1, 'W', 0));
  return box;
}
function flightPolarEditor(c, changed, prefix) {
  return flightCurveEditor(c, 'polar', 'Wing coefficient data', 'Angle degrees, Cl, Cd; 2–64 increasing angles. Outside the supplied range, blend to the generic stall model over 10°.', text => {
    const rows = text.trim().split(/\r?\n/).filter(s => s.trim() && !s.trim().startsWith('#'));
    if (/^\s*(alpha|angle)\b/i.test(rows[0] || '')) rows.shift();
    return FlightPhysics.polar(rows.map(s => s.trim().split(/[\s,;]+/).map(Number)));
  }, changed, prefix);
}
function flightBatteryFields(b) {
  flightBatteryFieldRefs = [];
  const f = (key, label, lo, hi, step, unit, fallback) => flightNumber(b, key, label, lo, hi, step, unit, fallback, battEdited, 'battery-physics');
  return UI.details({title: 'Electrical and thermal model', class: 'fold sub'},
    f('avionicsW', 'Total board / avionics draw', 0, 200, .5, 'W', 8),
    f('becEfficiency', 'Servo BEC efficiency', .5, 1, .01, '', .9),
    f('heatCapacity', 'Battery heat capacity', 1, 10000, 1, 'J/K', () => 900 * battMass()),
    f('cooling', 'Battery thermal conductance', .01, 100, .05, 'W/K', () => 1.4 * (battMass() / .2) ** (2 / 3)));
}
function buildFlightPhysicsUI() {
  const readout = el('dl', {id: 'flightPhysicsKv', class: 'kv'}), performance = el('dl', {id: 'flightPerformanceKv', class: 'kv'});
  $('#paneAir').append(el('section', {class: 'sec'}, el('h2', {text: 'Flight physics'}), readout,
    el('p', {class: 'hint', text: 'Thrust per watt is a lifting metric, not propulsive efficiency. Disk loading uses weight and summed rotor area; overlapping/tilted disks are not corrected. Generic tip losses start above Mach 0.7; Mach 0.6 is not a universal optimum.'}),
    UI.details({title: 'Frame timing', id: 'flightTimingFold', class: 'fold'}, performance, el('p', {class: 'hint', text: 'Rolling 240 frames. CPU includes scene submission and UI; GPU execution is asynchronous. Real-time rate is simulated seconds per wall second. Playback slows when estimated physics work exceeds an 11 ms frame budget; integration accuracy stays at the same fixed step.'}))));
  buildFlightEnvironmentFields();
}
function buildFlightEnvironmentFields() {
  $('#spFields').append(flightNumber(envr, 'pressure', 'Air pressure', 20000, 120000, 100, 'Pa', 101325, () => { refreshEnvelope(); renderMass(); save(); }, 'air'));
  const check = UI.input({type: 'checkbox', id: 'flight-sensor-effects'}); check.checked = envr.sensorEffects;
  spRefs.push(() => { check.checked = !!envr.sensorEffects; });
  check.addEventListener('change', () => { envr.sensorEffects = check.checked; save(); });
  $('#spFields').append(el('label', {class: 'check', for: check.id}, check, 'Estimated GPS sky blockage and barometer downwash bias'));
  const sample = UI.choice({label: 'Overlapping rotor wake sampling', options: [[1, 'One disk point'], [5, 'Five points where wakes overlap']], value: envr.rotorSamples,
    onChange: value => { envr.rotorSamples = Number(value); save(); }});
  spRefs.push(() => { sample.value = String(envr.rotorSamples); });
  sample.id = 'flight-rotor-samples'; $('#spFields').append(el('div', {class: 'field'}, el('label', {for: sample.id, text: 'Overlapping rotor wakes'}), sample));
}
function renderFlightPhysics() {
  const box = $('#flightPhysicsKv'); if (!box || !truth) return;
  const acts = actuators().filter(onBoard), air = flightAtmosphere();
  let area = 0, thrust = 0, mach = 0, rpm = 0, warnings = [];
  for (const c of acts) {
    const st = act.get(c.id) || {}; area += Math.PI * propR(c) ** 2; thrust += st.Teff || 0;
    const r = (st.Omega || 0) * 60 / (2 * Math.PI); rpm = Math.max(rpm, r); mach = Math.max(mach, st.mach || 0);
    if (st.overspeed) warnings.push(`${c.name}: manufacturer RPM exceeded`);
    if (st.outsidePropData) warnings.push(`${c.name}: extrapolating prop data`);
  }
  const watts = Math.max(0, (S.battI || 0) * (S.battV || 0));
  const mass = truth.m + liveComps().filter(c => c.type === 'hang').reduce((s, c) => s + c.mass, 0);
  syncKv(box, [['Battery current', `${(S.battI || 0).toFixed(2)} A`], ['Battery power', `${watts.toFixed(1)} W`],
    ['Motor electrical / shaft power', `${(S.motorW || 0).toFixed(1)} / ${(S.shaftW || 0).toFixed(1)} W`], ['Devices + servo input', `${(S.deviceW || 0).toFixed(1)} W`],
    ['Rotor thrust per battery watt', watts > 1 ? `${(thrust / watts).toFixed(3)} N/W` : '—'],
    ['Weight / summed disk area', area > 0 ? `${(mass * G / area).toFixed(1)} N/m²` : '—'], ['Air density', `${air.rho.toFixed(3)} kg/m³`],
    ['Highest prop RPM / tip Mach', `${Math.round(rpm)} / ${mach.toFixed(3)}`], ['Prop warnings', warnings.join('; ') || (mach >= .7 ? 'Generic compressibility loss active' : 'None')]]);
  const p = $('#flightTimingFold')?.open ? flightPerf.read() : null; if (p) syncKv($('#flightPerformanceKv'), [['FPS / real-time rate', `${p.fps.toFixed(1)} / ${p.realtime.toFixed(2)}×`],
    ['Frame CPU p95', `${p.cpu95.toFixed(2)} ms`], ['Physics CPU p95', `${p.physics95.toFixed(2)} ms`], ['Frame interval p95', `${p.interval95.toFixed(2)} ms`]]);
}
