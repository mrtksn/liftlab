'use strict';
// The agent's tools (agent.js): each a JSON-schema description for the model and a function over the simulator's own
// code, the same calls the buttons and fields make. Airframe and computer changes go into the undo history; a formula
// change can be undone from its line in the chat. Nothing here reaches outside the simulator.

const PART_TYPES = ['motor', 'tilt', 'joint', 'link', 'mass', 'hang', 'wing', 'latch', 'imu', 'mag', 'baro', 'fix', 'flow'];
const ENV_RANGE = { wind: [0, 15], windDir: [-180, 180], turb: [0, 1], spread: [0, 3], texture: [0, 1], light: [0, 1], ambient: [-10, 45] };
const num = (x, d) => (typeof x === 'number' && isFinite(x) ? x : d);
const obj = (props, req = []) => ({ type: 'object', properties: props, required: req, additionalProperties: false });
const kindOf = c => c.type === 'sensor' ? c.kind : c.type === 'mass' && isWing(c) ? 'wing' : c.type;
const partBrief = c => ({ id: c.id, type: kindOf(c), name: c.name, on: parentOf(c) ? parentOf(c).id : 'frame', pos: c.pos.map(v => Math.round(v * 1000) / 1000), mass: c.mass,
  ...(c.type === 'motor' ? { max_thrust_N: c.tmax, spin: c.spin > 0 ? 'CCW' : 'CW', pusher: !!c.push, tilt: c.tilt || 0, az: c.az || 0, prop_m: propR(c) * 2 } : {}),
  ...(c.type === 'joint' ? { mode: c.mode, range: c.range, hinge_az: c.hingeAz, hinge_el: c.hingeEl } : {}),
  ...(c.type === 'mass' ? { shape: c.shape, size: c.size, aero: c.aero, battery: c.battery || undefined } : {}), ...(c.type === 'sensor' ? { kind: c.kind } : {}) });
function partOf(id) { const c = cfg.comps.find(x => x.id === +id || x.name === id); if (!c) throw new Error(`no part ${JSON.stringify(id)} (get_airframe lists them)`); return c; }

// Run the simulation for a while, at its speed (or as fast as it goes), stopping early on a crash, a condition, or Stop.
async function agentRun(seconds, o = {}) {
  const t0 = S.t, until = S.t + clamp(num(seconds, 2), 0.05, 120);
  let test = null;
  if (o.until) { test = triggerTest({ id: 'until:' + o.until, kind: 'expr', expr: o.until }); if (!test) throw new Error('until: ' + triggerFn.get('until:' + o.until).err); }
  const win = { altMin: Infinity, altMax: -Infinity, errMax: 0, tiltMax: 0 }, note = s => {
    win.altMin = Math.min(win.altMin, s.alt); win.altMax = Math.max(win.altMax, s.alt); win.errMax = Math.max(win.errMax, s.err); win.tiltMax = Math.max(win.tiltMax, s.tilt); };
  let why = 'time', last = performance.now();
  const sig = agent.abort && agent.abort.signal;
  while (S.t < until - 1e-9) {
    if (sig && sig.aborted) { why = 'stopped by the person'; break; }
    if (S.crashed) { why = 'crashed: ' + S.crashed; break; }
    if (editMode) setEditMode(false);
    const s = agentSample(); note(s);
    if (test) { let hit = false; try { hit = !!test(s); } catch (e) {} if (hit) { why = 'until: ' + o.until; break; } }
    if (o.fast || document.hidden) {   // (a hidden page draws no frames: it's stepped here)
      if (running) { running = false; renderRun(); }
      const now = performance.now(), dt = o.fast ? 0.25 : Math.min(0.25, (now - last) / 1000 * speed); last = now;
      const n = Math.max(1, Math.round(Math.min(dt, until - S.t) / PDT));
      for (let i = 0; i < n && !S.crashed; i++) { if (i % 20 === 0) pilotStep(20 * PDT); physStep(); }
    } else if (!running) { running = true; renderRun(); }
    await new Promise(r => setTimeout(r, o.fast ? 0 : 40));
  }
  if (S.t >= until - 1e-9 && running && agent.cfg.pauseThinking) { running = false; renderRun(); }
  const s = agentSample(); note(s);
  return { ran_s: r2(S.t - t0), ended: why, now: s,
    over_that_time: { alt_min: r2(win.altMin), alt_max: r2(win.altMax), max_distance_to_target: r2(win.errMax), max_tilt_deg: r1(win.tiltMax) },
    events: agentEvents(6).filter(e => parseFloat(e) >= t0 - 1e-6) };
}

// Ask the person in the chat before doing something (a formula change, when Settings say so).
function agentConfirm(item, text) {
  return new Promise(resolve => {
    item.confirm = { text, resolve: ok => { item.confirm = null; item.answered = ok ? 'applied' : 'declined'; agentRenderFeed(); resolve(ok); } };
    agentRenderFeed();
    const sig = agent.abort && agent.abort.signal; if (sig) sig.addEventListener('abort', () => item.confirm && item.confirm.resolve(false), { once: true });
  });
}

const AGENT_TOOLS = {
  get_state: { desc: 'The simulation now: time, flight phase, position, velocity, attitude, target, battery, wind, the airframe check, edited or stopped formulas, latches, recent events.',
    params: obj({}), run: () => agentState() },
  get_history: { desc: 'Recorded telemetry of the last seconds (up to 120 s), sampled every `every` seconds. Fields: t x y alt vx vy climb speed roll pitch yaw tilt tx ty tz heading err (distance to target) batt volts crashed phase flying aero.',
    params: obj({ seconds: { type: 'number' }, every: { type: 'number' }, fields: { type: 'array', items: { type: 'string' } } }),
    run: a => {
      const from = S.t - clamp(num(a.seconds, 10), 0.1, 120), every = Math.max(0.1, num(a.every, 0.5)), f = a.fields && a.fields.length ? ['t', ...a.fields.filter(k => k !== 't')] : null;
      const out = []; let next = -Infinity;
      for (const s of agent.rec) if (s.t >= from && s.t <= S.t + 1e-6 && s.t >= next - 1e-6) { next = s.t + every; out.push(f ? f.map(k => s[k]) : s); }
      return f ? { columns: f, rows: out } : { samples: out };
    } },

  get_airframe: { desc: 'The airframe: frame mass and shape, steering mode, battery, and every part (id, type, name, what it is on, position in body axes [m], mass [kg]). Layouts that can be loaded too.',
    params: obj({}),
    run: () => ({ frame_mass: cfg.frame.mass, frame_shape: frameShapeOf(), steering: mode, battery: battCfg(), parts: cfg.comps.map(partBrief),
      layouts: Object.fromEntries(Object.entries(PRESETS).map(([k, p]) => [k, p.label])) }) },
  get_part: { desc: 'Every field of one part (by id or name).', params: obj({ id: { type: ['integer', 'string'] } }, ['id']), run: a => partOf(a.id) },
  set_part: { desc: 'Change fields of a part (any field get_part shows except id, type, parent; same kind of value). `pos` is [x,y,z] in body axes. Moving or turning a servo or rod carries what is on it.',
    params: obj({ id: { type: ['integer', 'string'] }, fields: { type: 'object' } }, ['id', 'fields']),
    run: a => {
      const c = partOf(a.id), F = a.fields || {}, bad = [];
      for (const [k, v] of Object.entries(F)) {
        if (['id', 'type', 'parent'].includes(k)) { bad.push(`${k}: can't be set (attach_part moves a part)`); continue; }
        if (!(k in c)) { bad.push(`${k}: no such field`); continue; }
        const o = c[k];
        if (Array.isArray(o) ? !(Array.isArray(v) && v.length === o.length && v.every(x => typeof x === typeof o[0] && (typeof x !== 'number' || isFinite(x)))) : typeof v !== typeof o || (typeof v === 'number' && !isFinite(v))) { bad.push(`${k}: expected ${Array.isArray(o) ? 'an array of ' + o.length : typeof o}`); continue; }
        c[k] = Array.isArray(v) ? v.slice() : v;
      }
      if (isHolder(c)) carryAlong(c); if (c.type === 'hang') reseatPend(c);
      undoKey = null; structural();
      return { part: partOf(c.id), ...(bad.length ? { not_set: bad } : {}) };
    } },
  add_part: { desc: 'Add a part: on the frame (default), on a servo/rod/latch (`on`: its id), or, for joint, link or latch, `between` a part (its id) and what it hangs on, which keeps that part in place. Then `fields` as set_part. Types: motor, tilt (a motor on a servo), joint (servo), link (rod), mass, hang (mass on a cable), wing, latch, imu, mag, baro, fix (position fix), flow.',
    params: obj({ type: { type: 'string', enum: PART_TYPES }, on: { type: ['integer', 'string'] }, between: { type: ['integer', 'string'] }, fields: { type: 'object' } }, ['type']),
    run: a => {
      let place = null;
      if (a.on != null && a.on !== 'frame') { const h = partOf(a.on); if (!isHolder(h)) throw new Error(`${h.name} can't carry parts: only a servo, rod or latch`); place = { on: h }; }
      if (a.between != null) { if (!INSERTABLE.has(a.type)) throw new Error('only a joint, link or latch goes between'); place = { above: partOf(a.between) }; }
      const hadCargo = hasTask('cargo'), c = addComp(a.type, place);
      const note = !hadCargo && hasTask('cargo') ? { note: 'no board ran the Cargo task, so it was put on the flight controller: the flight restarted' } : {};
      return { ...(a.fields ? AGENT_TOOLS.set_part.run({ id: c.id, fields: a.fields }) : { part: c }), ...note };
    } },
  remove_part: { desc: 'Remove a part. What was on it moves to what it was on.', params: obj({ id: { type: ['integer', 'string'] } }, ['id']),
    run: a => { const c = partOf(a.id); for (const x of cfg.comps) if (x.parent === c.id) x.parent = c.parent ?? null; cfg.comps = cfg.comps.filter(x => x !== c); openSet.delete(c.id); structural(); return { removed: c.name }; } },
  attach_part: { desc: 'Attach a part to a servo, rod or latch (`to`: its id), or to the frame (`to`: "frame").', params: obj({ id: { type: ['integer', 'string'] }, to: { type: ['integer', 'string'] } }, ['id', 'to']),
    run: a => {
      const c = partOf(a.id), h = a.to === 'frame' || a.to == null ? null : partOf(a.to);
      if (h && !canAttach(c, h)) throw new Error(`${c.name} can't go on ${h.name}`);
      attachTo(c, h); if (c.type === 'hang') reseatPend(c); structural();
      return { part: partBrief(c) };
    } },
  set_frame: { desc: 'The frame and the drone as a whole: frame_mass [kg], steering ("tilt" body, "mixed", "level"), battery fields (cells, capacity [Ah], rInt [ohm], startSoc 0…1…).',
    params: obj({ frame_mass: { type: 'number' }, steering: { type: 'string', enum: ['tilt', 'mixed', 'level'] }, battery: { type: 'object' } }),
    run: a => {
      if (a.frame_mass != null) { cfg.frame.mass = clamp(num(a.frame_mass, cfg.frame.mass), 0.02, 50); frameMassField.refresh(); }
      if (a.steering && a.steering !== mode) { setMode(a.steering, false); doReset(); }   // (the flight controller reads it at start)
      if (a.battery) { const b = battCfg(); for (const [k, v] of Object.entries(a.battery)) if (k in b && typeof v === typeof b[k]) b[k] = v; renderBattery(); renderBattSmall(); }
      undoKey = null; recomputeProps(); refreshEnvelope(); renderMass(); structural();
      return { frame_mass: cfg.frame.mass, steering: mode, battery: battCfg() };
    } },
  load_layout: { desc: 'Replace the airframe with a layout (get_airframe lists them). The person can undo it.', params: obj({ layout: { type: 'string', enum: Object.keys(PRESETS) } }, ['layout']),
    run: a => { if (!PRESETS[a.layout]) throw new Error('no such layout'); loadPreset(a.layout); return { loaded: PRESETS[a.layout].label, parts: cfg.comps.map(partBrief) }; } },

  get_computers: { desc: 'The flight computers: each board, its kind and the tasks it runs; what each task and board kind is.',
    params: obj({}),
    run: () => ({ boards: computers().boards.map(b => ({ id:b.id,name: b.name, kind: b.kind, tasks: b.tasks, ...(b.tasks.includes('core')?{flight_loop_hz:boardTaskHz(b,'core')}:{}) })), ground:computers().ground,
      tasks: Object.fromEntries(Object.entries(TASKS).map(([k, T]) => [k, `${T.label}, ${T.hz} Hz${T.mcuOnly ? ', microcontroller only' : ''}${T.piOnly ? ', Linux only' : ''}: ${T.what}`])),
      board_kinds: Object.fromEntries(Object.entries(BOARD_KINDS).filter(([, k]) => !k.groundOnly).map(([k, B]) => [k, `${B.label}: ${B.note}`])),
      ground_board_kinds:Object.fromEntries(Object.entries(BOARD_KINDS).map(([k,B])=>[k,B.label])) }) },
  set_computers: { desc: 'Replace the drone board list (1–4). Include each existing board id from get_computers to preserve wiring, even when renaming/reordering. Omit id for a new board; an exact unique existing name also preserves its id. Removed boards leave explicit connections disconnected. Assign core exactly once to a microcontroller; other tasks at most once. Optional ground changes the command-module kind/name. Restarts flight.',
    params: obj({ boards: { type: 'array', items: obj({ id:{type:'integer'},name: { type: 'string' }, kind: { type: 'string' }, tasks: { type: 'array', items: { type: 'string' } } }, ['kind', 'tasks']) },ground:obj({kind:{type:'string'},name:{type:'string'}}) }, ['boards']),
    run: a => agentSetComputers(a) },

  list_formulas: { desc: 'Every formula: key, title, group (plant: physics; sensor; est: estimators; ctrl: control; learn; super: health; ground), which task runs it, and whether it is edited or stopped.',
    params: obj({}),
    run: () => Object.values(LAWS).map(L => ({ key: L.def.key, title: L.def.title, group: L.def.group, task: taskOfLaw(L.def.key), status: L.status })) },
  get_formula: { desc: 'A formula: what it does, its arguments and return value, its current code and (if edited) the default code.',
    params: obj({ key: { type: 'string' }, with_default: { type: 'boolean' } }, ['key']),
    run: a => {
      const L = LAWS[a.key]; if (!L) throw new Error('no such formula (list_formulas)');
      return { key: a.key, title: L.def.title, doc: L.def.doc, used: L.def.used, args: L.def.args, returns: L.def.returns, status: L.status, error: L.err || L.rnErr || undefined,
        code: L.src, ...(a.with_default && L.status !== 'default' ? { default_code: L.defSrc } : {}) };
    } },
  set_formula: { desc: 'Replace a formula\'s code: one JavaScript function with the same arguments, returning the same kind of value. It is test-called before it goes in; an error comes back and nothing changes. Flight formulas also compile for the boards.',
    params: obj({ key: { type: 'string' }, code: { type: 'string' } }, ['key', 'code']),
    run: async (a, item) => {
      const L = LAWS[a.key]; if (!L) throw new Error('no such formula (list_formulas)');
      if (agent.cfg.askFormulas && !(await agentConfirm(item, `Apply this change to ${L.def.title}?`))) return { error: 'the person declined this change' };
      const prev = L.src;
      applyLaw(a.key, a.code); save(); if (typeof refreshLawCard === 'function') refreshLawCard(a.key);
      item.undo = { label: 'Undo', run: () => { applyLaw(a.key, prev); save(); if (typeof refreshLawCard === 'function') refreshLawCard(a.key); return `${L.def.title}: back to the code before this change`; } };
      return { applied: a.key, status: LAWS[a.key].status, compiled_for_boards: !LAWS[a.key].rnErr, ...(LAWS[a.key].rnErr ? { board_note: LAWS[a.key].rnErr } : {}) };
    } },
  reset_formula: { desc: 'Put a formula back to its default code.', params: obj({ key: { type: 'string' } }, ['key']),
    run: (a, item) => {
      const L = LAWS[a.key]; if (!L) throw new Error('no such formula'); const prev = L.src;
      resetLaw(a.key); save(); if (typeof refreshLawCard === 'function') refreshLawCard(a.key);
      if (prev !== L.defSrc) item.undo = { label: 'Undo', run: () => { applyLaw(a.key, prev); save(); if (typeof refreshLawCard === 'function') refreshLawCard(a.key); return `${L.def.title}: the edited code is back`; } };
      return { reset: a.key };
    } },

  simulation: { desc: 'Control the simulation: reset (start the flight again), pause, run, speed (0.25, 0.5, 1), throw (reset into a throw: needs the learning task), hover (resets start hovering), calibrate (a hover calibration: needs the learning task), stop_calibration.',
    params: obj({ action: { type: 'string', enum: ['reset', 'pause', 'run', 'speed', 'throw', 'hover', 'calibrate', 'stop_calibration'] }, speed: { type: 'number', enum: [0.25, 0.5, 1] } }, ['action']),
    run: a => {
      if (editMode) setEditMode(false);
      switch (a.action) {
        case 'reset': doReset(); break;
        case 'pause': running = false; renderRun(); agent.wasRunning = false; break;
        case 'run': running = true; renderRun(); agent.wasRunning = true; break;
        case 'speed': setSpeed([0.25, 0.5, 1].includes(a.speed) ? a.speed : 1); break;
        case 'throw': if (!hasTask('learn')) throw new Error('the throw start needs a board running the learning task'); setLaunch('throw'); break;
        case 'hover': setLaunch('hover'); break;
        case 'calibrate': if (!hasTask('learn')) throw new Error('calibration needs a board running the learning task'); if (S.crashed) throw new Error('crashed: reset first'); pilotLearnCmd('calibrate'); renderLearn(true); break;
        case 'stop_calibration': pilotLearnCmd('stop'); renderLearn(true); break;
      }
      return { done: a.action, time: r2(S.t), simulation: running ? 'running' : 'paused', speed };
    } },
  wait: { desc: 'Run the simulation for some seconds (up to 120) and report: the state at the end, the range of height, distance to target and tilt over that time, and events. `until`: stop early when this expression is true (fields as get_history, e.g. "err < 0.1"). `fast`: as fast as the computer goes instead of real time. Stops on a crash.',
    params: obj({ seconds: { type: 'number' }, until: { type: 'string' }, fast: { type: 'boolean' } }, ['seconds']),
    run: a => agentRun(a.seconds, a) },
  fly: { desc: 'Fly: goto (a target x, y, z [m] from the origin, and heading [deg], each optional: unchanged when left out), hold (where it is), home, or level (the speed of the keys: gentle, normal, sport). Needs the navigation task for go-to, hold and home.',
    params: obj({ action: { type: 'string', enum: ['goto', 'hold', 'home', 'level'] }, x: { type: 'number' }, y: { type: 'number' }, z: { type: 'number' }, heading: { type: 'number' }, level: { type: 'string', enum: ['gentle', 'normal', 'sport'] } }, ['action']),
    run: a => {
      if (a.action === 'level') { setPilotLevel(a.level || 'normal'); return { level: pilot.level }; }
      if (!hasTask('nav')) throw new Error('no board runs the navigation: it flies in angle mode (keys lean it). Add the nav task with set_computers.');
      if (brt.pickup) pickupStop();
      if (a.action === 'hold') pilotHold(); else if (a.action === 'home') pilotHome();
      else {
        setpoint.x = clamp(num(a.x, setpoint.x), -PILOT_BOX.xy, PILOT_BOX.xy); setpoint.y = clamp(num(a.y, setpoint.y), -PILOT_BOX.xy, PILOT_BOX.xy);
        setpoint.z = clamp(num(a.z, setpoint.z), PILOT_BOX.zMin, PILOT_BOX.zMax);
        if (a.heading != null) setpoint.yaw = ((num(a.heading, 0) + 180) % 360 + 360) % 360 - 180;
        pilot.vref = [0, 0, 0]; ctl.vRef = [0, 0, 0];
      }
      syncSp();
      return { target: { x: r2(setpoint.x), y: r2(setpoint.y), z: r2(setpoint.z), heading: Math.round(setpoint.yaw) }, phase: flightPhaseText() };
    } },
  latch: { desc: 'Work a latch (index from get_state): open (drop what it holds), close (grab what is within its reach), or fetch (fly to the nearest loose thing and pick it up: needs the navigation and the latch open).',
    params: obj({ latch: { type: 'integer' }, action: { type: 'string', enum: ['open', 'close', 'fetch'] } }, ['latch', 'action']),
    run: a => {
      const l = latches()[a.latch]; if (!l) throw new Error('no such latch');
      if (!hasTask('cargo')) latchNeedsBoard();
      let why;
      if (a.action === 'fetch') { const v = latchView(l); if (!v.near) throw new Error('nothing loose to fetch'); why = cargoFetch(l, a.latch, v.near.L); }
      else why = pilotCargoCmd(a.latch, a.action === 'open' ? 0 : 1);
      if (why) throw new Error(why);
      return { done: a.action, latch: l.name };
    } },
  environment: { desc: 'The world: wind [m/s], windDir (where it blows toward, deg), turb (turbulence 0…1), spread (motor and prop differences, × typical), texture (ground, for optical flow 0…1), light 0…1, ambient [°C]; terrain: open, parkour or city (changing it restarts the flight).',
    params: obj({ wind: { type: 'number' }, windDir: { type: 'number' }, turb: { type: 'number' }, spread: { type: 'number' }, texture: { type: 'number' }, light: { type: 'number' }, ambient: { type: 'number' }, terrain: { type: 'string', enum: ['open', 'parkour', 'city'] } }),
    run: a => {
      for (const [k, [lo, hi]] of Object.entries(ENV_RANGE)) if (a[k] != null) envr[k] = clamp(num(a[k], envr[k]), lo, hi);
      if (a.terrain && a.terrain !== terrain.kind) applyTerrain(a.terrain, terrain.seed);
      syncSp();
      return { ...Object.fromEntries(Object.keys(ENV_RANGE).map(k => [k, envr[k]])), terrain: terrain.kind };
    } },
  break_part: { desc: 'Make a part fail in flight, to test the drone: a motor (mode: stop, loss, prop), a servo (jam, limp), or the battery (id "battery"; mode: cell, cut). Until the next reset.',
    params: obj({ id: { type: ['integer', 'string'] }, mode: { type: 'string' } }, ['id']),
    run: a => {
      if (a.id === 'battery') { breakBattery(a.mode === 'cut' ? 'cut' : 'cell', 'broken by the AI agent'); return { broke: 'battery' }; }
      const c = partOf(a.id); if (c.type !== 'motor' && c.type !== 'joint') throw new Error('only a motor, a servo or the battery');
      breakDevice(c, a.mode || c.failMode, 'broken by the AI agent'); return { broke: c.name, mode: a.mode || c.failMode };
    } },
  repair_all: { desc: 'Repair every broken part (as the Health panel\'s Repair all).', params: obj({}), run: () => { repairAll(); renderHealth(true); refreshEnvelope(); return { repaired: true }; } },
};
