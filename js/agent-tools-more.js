'use strict';
// The rest of the agent's tools (agent-tools.js has the core ones): everything else the page shows or lets you do, so
// the agent sees and works what you do. The health of every part, the actuators, the sensors and the estimate, the
// learning, the boards, the airframe check and mass, the radio and the command module, the logs; the sticks and the
// poke, the camera and what the view draws, the learning and allocation settings, the radio's link and settings, saved
// designs, undo, the things to pick up, its own triggers. Two more are off until Settings turn them on: looking at
// the view (a picture, for a model that can see) and running its own JavaScript in the page.

const rnd = (x, d = 3) => (typeof x === 'number' && isFinite(x) ? +x.toFixed(d) : x);
const rndA = (a, d = 3) => (Array.isArray(a) ? a.map(x => rnd(x, d)) : a);
const CTRLS = ['fwd', 'back', 'left', 'right', 'up', 'down', 'yawL', 'yawR'];

Object.assign(AGENT_TOOLS, {
  get_health: { desc: 'Every motor, servo and the battery: the true state (working, stopped, prop broken, thrust lost, jammed, limp; cut, cells lost), the temperature and its limit, what the drone\'s sensors read, and what the health supervisor decided (its mode and why, each part kept, scaled or removed). Also the supervisor\'s log.',
    params: obj({}),
    run: () => {
      const sv = brt.superView, b = battCfg();
      const motors = actuators().map((c, i) => {
        const s = hs.get(c.id) || {}, r = hread.m.get(c.id) || {}, e = sv && sv.motors[i];
        return { id: c.id, name: c.name, state: s.prop ? 'prop broken' : s.dead ? 'stopped' : s.loss > 0.004 ? `lost ${Math.round(s.loss * 100)}% thrust` : 'working', cause: s.cause || undefined,
          temp_C: rnd(s.T, 1), limit_C: c.tmaxC ?? 120, health_pct: c.health, sensor_temp_C: rnd(r.T, 1), esc_current_A: rnd(r.I, 1),
          supervisor: e ? { in_table: !!e.on, scale: rnd(e.eff), cap: rnd(e.cap), est_temp_C: rnd(e.temp, 1) } : undefined };
      });
      const servos = joints().map((j, k) => { const s = hs.get(j.id) || {}, d = sv && sv.joints[k], st = jst.get(j.id) || {};
        return { id: j.id, name: j.name, state: s.limp ? 'limp' : s.jam != null ? 'jammed' : 'working', angle_deg: rnd((st.th || 0) * R2D, 1),
          supervisor: d ? { left_out: !!d.off, believed_deg: rnd(d.angle * R2D, 1) } : undefined }; });
      return { supervisor: sv ? { board: (boardOf('super') || {}).name, mode: (MODE_TXT[sv.mode | 0] || MODE_TXT[0])[0], why: sv.modeWhy || undefined, cells_counted: sv.cells, limits: sv.lim, log: (sv.log || []).slice(0, 10).map(l => `${rnd(l.t, 1)} s: ${l.msg}`) } : 'no board runs the supervisor',
        motors, servos,
        battery: { state: hb.cut ? 'cut out' : hb.lvc ? 'ESCs cut (low voltage)' : hb.cellsLost ? `${hb.cellsLost} cells lost` : hb.fade > 0.005 ? `worn ${Math.round(hb.fade * 100)}%` : 'working', charge_pct: Math.round(100 * Math.max(0, S.batt.soc ?? 1)), volts: rnd(S.battV, 2), temp_C: rnd(hb.T, 1), limit_C: b.tmaxC, cells: b.cells,
          sensors: { volts: rnd(hread.b.V, 2), amps: rnd(hread.b.I, 1), temp_C: rnd(hread.b.T, 1) } },
        events: (sup.log || []).slice(0, 10).map(l => `${rnd(l.t, 1)} s: ${l.msg}`) };
    } },
  get_actuators: { desc: 'What each motor and servo is doing now: a motor\'s commanded and real thrust [N], throttle, prop speed [rad/s], current [A], and whether it is at its limit; a servo\'s commanded, believed and real angle [deg] and torque. Also what the flight core is asking for (torque, saturation).',
    params: obj({}),
    run: () => ({
      motors: actuators().map(c => { const st = act.get(c.id) || {}; return { id: c.id, name: c.name, thrust_cmd_N: rnd(st.Tcmd, 2), thrust_N: rnd(st.T * motorEff(c), 2), max_N: c.tmax, throttle: rnd(st.u), rpm: rnd((st.Omega || 0) * 60 / (2 * Math.PI), 0), current_A: rnd(st.i, 1), at_limit: (st.Tcmd || 0) >= c.tmax * 0.98, pusher: !!c.push, spin: c.spin }; }),
      servos: joints().map(j => { const st = jst.get(j.id) || {}; return { id: j.id, name: j.name, mode: j.mode, cmd_deg: rnd((st.thCmd || 0) * R2D, 1), believed_deg: rnd((st.thHat || 0) * R2D, 1), real_deg: rnd((st.th || 0) * R2D, 1), torque_Nm: rnd(st.tq), range_deg: j.range }; }),
      flight_core: brt.out ? { state: FC_STATES[brt.fcState] || brt.fcState, why: brt.fcWhy || undefined, saturated: !!brt.out.sat, torque_wanted_Nm: rndA(brt.out.tau) } : null,
    }) },
  get_estimate: { desc: 'What the drone believes against the truth: attitude, tilt, heading, position, altitude and velocity errors, the gyro bias; its estimated position and velocity; and each sensor\'s latest reading.',
    params: obj({}),
    run: () => {
      const e = estimateErrors();
      return { errors: { attitude_deg: rnd(e.ang, 2), tilt_deg: rnd(e.tilt, 2), heading_deg: rnd(e.head, 1), position_cm: rnd(e.pos, 1), altitude_cm: rnd(e.alt, 1), velocity_cm_s: rnd(e.vel, 1), gyro_bias_deg_s: rnd(e.gb, 2) },
        estimate: { position: est.havePos ? rndA(est.p) : null, velocity: est.havePos ? rndA(est.v) : null, has_position: est.havePos },
        sensors: allSensors().map(c => { const rt = sens.get(c.id), L = rt && rt.latest; const o = { id: c.id, kind: c.kind, name: c.name };
          if (L) for (const [k, v] of Object.entries(L)) if (typeof v === 'number' || Array.isArray(v)) o[k] = Array.isArray(v) ? rndA(v) : rnd(v); return o; }) };
    } },
  get_learning: { desc: 'The learning task: what the flight core flies on (description or learned), the calibration\'s progress, what it learned against the truth, the settings (keep learning in flight, freeze other motors during pulses) and the throw start\'s settings.',
    params: obj({}),
    run: () => { const v = learn.view; if (!v) return { learning: hasTask('learn') ? 'not running yet' : 'no board runs the learning task', throw: throwCfg };
      const { B, motors, joints: js, ...rest } = v;
      return { ...rest, message: learn.msg, motors: (motors || []).slice(0, 12), servos: (js || []).slice(0, 8), throw: throwCfg, launch: launchMode }; } },
  set_learning: { desc: 'Learning settings: fly_on ("description" or "learned"), keep_learning (in flight), freeze_pulses (other motors during the tests), and the throw start: throw_height [m], throw_spin [rad/s], calibrate_after_throw.',
    params: obj({ fly_on: { type: 'string', enum: ['description', 'learned'] }, keep_learning: { type: 'boolean' }, freeze_pulses: { type: 'boolean' }, throw_height: { type: 'number' }, throw_spin: { type: 'number' }, calibrate_after_throw: { type: 'boolean' } }),
    run: a => {
      if (a.fly_on) pilotLearnCmd(a.fly_on === 'learned' ? 'useLearned' : 'useDesc');
      if (a.keep_learning != null) { learnPrefs.keep = a.keep_learning; pilotLearnCmd(a.keep_learning ? 'keepOn' : 'keepOff'); }
      if (a.freeze_pulses != null) { learnPrefs.holdPulses = a.freeze_pulses; pilotLearnCmd(a.freeze_pulses ? 'holdOn' : 'holdOff'); }
      if (a.throw_height != null) throwCfg.height = Math.max(1, num(a.throw_height, throwCfg.height));
      if (a.throw_spin != null) throwCfg.spin = Math.max(0, num(a.throw_spin, throwCfg.spin));
      if (a.calibrate_after_throw != null) { throwCfg.thenCalibrate = a.calibrate_after_throw; pilotLearnCmd(a.calibrate_after_throw ? 'thenCalOn' : 'thenCalOff'); }
      for (const r of throwFieldRefs) r(); renderLearn(true); save();
      return { learn_prefs: learnPrefs, throw: throwCfg };
    } },
  get_boards: { desc: 'The flight computers running: each board\'s load (share of its processor), memory, its tasks; the flight core\'s state; the program being loaded (staged) and the boards\' messages; errors.',
    params: obj({}),
    run: () => ({ boards: computers().boards.map(b => { const B = boardBudget(b); return { name: b.name, kind: b.kind, tasks: b.tasks, load_pct: rnd(B.load * 100, 1), memory_KB: rnd(B.memKB, 1), ram_KB: B.ramKB }; }),
      running: brt.ready, error: brt.err || undefined, flight_core: FC_STATES[brt.fcState] || brt.fcState, why: brt.fcWhy || undefined, navigation: brt.navWhy || undefined, learning_error: brt.learnErr || undefined,
      program: RN.stage ? rnStageText(RN.stage) : 'flying the loaded program', messages: (RN.log || []).slice(0, 12).map(l => `${rnd(l.t, 1)} s: ${l.msg}`) }) },
  get_envelope: { desc: 'The airframe check in full: the verdict and why, the control headroom beyond hover on each axis (both sides), and the mass properties (true mass, mass on cables, thrust to weight, centre of mass, what the controller believes, inertia).',
    params: obj({}),
    run: () => {
      const r = envRes || {}, mp = cfg.comps.filter(c => c.type === 'hang').reduce((s, c) => s + c.mass, 0);
      return { verdict: r.verdict, title: r.title, why: r.why, axes: r.k, headroom: r.head && r.labels ? Object.fromEntries(r.labels.map((l, i) => [l, { minus: rnd(r.head[i][0]), plus: rnd(r.head[i][1]) }])) : null, weak: r.weak,
        mass: { rigid_kg: rnd(truth.m), on_cables_kg: rnd(mp), thrust_to_weight: rnd(actuators().reduce((s, c) => s + c.tmax * motorEff(c), 0) / ((truth.m + mp) * G), 2),
          cog_from_hub_m: rndA(truth.c), controller_cog_error_mm: rnd(nrm(sub(truth.c, model.c)) * 1000, 0), controller_mass_error_g: rnd((model.m - truth.m - mp) * 1000, 0), inertia_kgm2: [truth.J[0], truth.J[4], truth.J[8]].map(x => rnd(x, 5)) } };
    } },
  get_radio: { desc: 'The radio and the command module: whether the drone has a radio, the link (kind: elrs ExpressLRS 2.4 GHz, espnow ESP-NOW ESP32 to ESP32, wifi Wi-Fi UDP) and its settings, the link statistics of the last 5 s, the telemetry the command module decoded, its alert, and its log.',
    params: obj({}),
    run: () => JSON.parse(JSON.stringify({ has_radio: hasTask('tlm'), active: radioActive(), link_kind: radioCfg.kind, link_label: radioModel().label, settings: radioCfg, second_link: radioTwo() ? { ...radioCfg2, label: RADIO_LINKS[radioCfg2.kind].label } : 'none', setting_lines: radioSettingLines(radioCfg),
      wifi: pk.assoc && radioModel().packets ? pk.assoc.state : undefined, link: hasTask('tlm') ? linkStats(radio.t) : null, note: radioModel().roomNote(radioCfg) || undefined,
      ground: { alert: gs.alert, link: gs.link, telemetry: gs.v, frames: gs.frames, log: (gs.log || []).slice(-10).map(l => typeof l === 'string' ? l : (l.msg || l.text || JSON.stringify(l)).slice(0, 160)) } },
      (k, v) => (typeof v === 'number' ? +v.toFixed(2) : v))) },
  set_radio: { desc: 'The radio\'s settings; both ends take them at once, in flight too. kind: elrs, espnow, ble, wifi, serial or nrf24 (another link: the drone sees a short gap; Wi-Fi then joins its network, 1–3 s). ExpressLRS: rate (packets a second: 50, 150, 250, 500), ratio (telemetry every Nth packet: 2…128), power [mW: 10, 25, 100, 250, 500, 1000]. ESP-NOW: channel (1–13), lr (long range, true/false). Wi-Fi: sta (true: the drone joins a network; false: it makes one, an access point), channel (1–13, the access point\'s). Serial line (a laser, fibre, infrared, a radio modem or a wire carrying a UART\'s bytes): baud (19200, 38400, 57600, 115200, 230400, 460800, 921600), half (true: one way at a time, as radio modems; 38400 and up), dir (up or down: a line that goes one way only, for beside a second link; both as it was), medium (the simulator\'s: 0 fibre or wire, 1 laser, 2 infrared, 3 radio modem), tether [m: 10, 25, 50, 100, 300] for fibre or wire. nRF24L01: kbps (250, 1000 or 2000: 250 reaches furthest). ESP-NOW, Wi-Fi, serial and nRF24L01: bind (the binding phrase, the same at both ends, 1–31 characters). Any link: extra (extra path loss [dB]: distance, walls).',
    params: obj({ kind: { type: 'string', enum: ['elrs', 'espnow', 'ble', 'wifi', 'serial', 'nrf24'] }, kbps: { type: 'integer' }, baud: { type: 'integer' }, half: { type: 'boolean' }, dir: { type: 'string', enum: ['both', 'up', 'down'] }, medium: { type: 'integer' }, tether: { type: 'number' }, rate: { type: 'number' }, ratio: { type: 'number' }, power: { type: 'number' }, extra: { type: 'number' },
      channel: { type: 'integer' }, lr: { type: 'boolean' }, sta: { type: 'boolean' }, bind: { type: 'string' } }),
    run: a => {
      const bad = [], ok = (k, test, why) => { if (a[k] != null && !test(a[k])) bad.push(`${k}: ${why}`); };
      ok('kind', v => !!RADIO_LINKS[v], 'elrs, espnow, ble, wifi, serial or nrf24'); ok('kbps', v => [250, 1000, 2000].includes(+v), '250, 1000 or 2000');
      ok('baud', v => SL_BAUDS.includes(+v), SL_BAUDS.join(', ')); ok('medium', v => [0, 1, 2, 3].includes(+v), '0 fibre or wire, 1 laser, 2 infrared, 3 radio modem'); ok('tether', v => [10, 25, 50, 100, 300].includes(+v), '10, 25, 50, 100 or 300');
      if ((a.half ?? radioCfg.half) && +(a.baud ?? radioCfg.baud) < 38400 && (a.kind || radioCfg.kind) === 'serial') bad.push('half: one way at a time needs 38400 baud or more');
      ok('rate', v => ELRS_RATES[v] != null, '50, 150, 250 or 500'); ok('ratio', v => ELRS_RATIOS.includes(+v), 'one of ' + ELRS_RATIOS.join(', '));
      ok('power', v => ELRS_POWERS.includes(+v), 'one of ' + ELRS_POWERS.join(', ')); ok('extra', v => isFinite(v) && v >= 0 && v <= 200, '0 to 200 dB');
      ok('channel', v => Number.isInteger(+v) && v >= 1 && v <= 13, '1 to 13'); ok('bind', v => !!radioPhraseOk(v), '1–31 plain characters');
      if (bad.length) throw new Error('Radio not changed: ' + bad.join('; '));
      for (const k of ['rate', 'ratio', 'power', 'extra', 'channel', 'baud', 'medium', 'tether', 'kbps']) if (a[k] != null) radioCfg[k] = +a[k];
      for (const k of ['lr', 'sta', 'half']) if (a[k] != null) radioCfg[k] = a[k] ? 1 : 0;
      if (a.dir) radioCfg.half = a.dir === 'up' ? 2 : a.dir === 'down' ? 3 : radioCfg.half > 1 ? 0 : radioCfg.half;
      if (a.bind != null) radioCfg.bind = radioPhraseOk(a.bind);
      if (a.kind) radioCfg.kind = a.kind;
      save(); boardsRadioCfg(); if (typeof renderGs === 'function') renderGs(true);
      return { settings: radioCfg, setting_lines: radioSettingLines(radioCfg), note: radioModel().roomNote(radioCfg) || 'applied at both ends now' };
    } },
  set_second_link: { desc: 'A second link at once beside the first (runner/fc/lmux.h): both carry everything; the drone takes the channels from the first while it has them and the second fills in; commands and messages arrive once. kind: none, or a kind other than the first\'s (elrs, espnow, ble, wifi, serial, nrf24), with its settings as set_radio\'s (rate, ratio, power, channel, lr, sta, baud, half, dir, medium, tether, kbps, extra). The binding phrase is the first link\'s. Applied at both ends now.',
    params: obj({ kind: { type: 'string', enum: ['none', 'elrs', 'espnow', 'ble', 'wifi', 'serial', 'nrf24'] }, kbps: { type: 'integer' }, baud: { type: 'integer' }, half: { type: 'boolean' }, dir: { type: 'string', enum: ['both', 'up', 'down'] }, medium: { type: 'integer' }, tether: { type: 'number' }, rate: { type: 'number' }, ratio: { type: 'number' }, power: { type: 'number' }, extra: { type: 'number' }, channel: { type: 'integer' }, lr: { type: 'boolean' }, sta: { type: 'boolean' } }),
    run: a => {
      if (a.kind && a.kind !== 'none' && (!RADIO_LINKS[a.kind] || a.kind === radioCfg.kind)) throw new Error('Second link not changed: kind: none, or a kind other than the first link\'s (' + radioCfg.kind + ')');
      if (a.baud != null && !SL_BAUDS.includes(+a.baud)) throw new Error('Second link not changed: baud: ' + SL_BAUDS.join(', '));
      for (const k of ['rate', 'ratio', 'power', 'extra', 'channel', 'baud', 'medium', 'tether', 'kbps']) if (a[k] != null && isFinite(a[k])) radioCfg2[k] = +a[k];
      for (const k of ['lr', 'sta', 'half']) if (a[k] != null) radioCfg2[k] = a[k] ? 1 : 0;
      if (a.dir) radioCfg2.half = a.dir === 'up' ? 2 : a.dir === 'down' ? 3 : radioCfg2.half > 1 ? 0 : radioCfg2.half;
      if (a.kind) radioCfg2.kind = a.kind === 'none' ? '' : a.kind;
      save(); boardsRadioCfg2(); if (typeof renderGs === 'function') { if (typeof GS_UI !== 'undefined') GS_UI.kindShown = null; renderGs(true); }
      return { second: radioTwo() ? { ...radioCfg2 } : 'none', setting_lines: radioSettingLines(radioCfg) };
    } },
  get_events: { desc: 'Everything logged lately, newest first: the boards, the supervisor, the cargo (up to 40 lines).', params: obj({ n: { type: 'integer' } }), run: a => agentEvents(clamp(num(a.n, 20), 1, 40)) },

  stick: { desc: 'Hold flight keys for some seconds, as you would on the keyboard: fwd, back, left, right (from the heading), up, down, yawL, yawR. With navigation they move the target; without, they lean the drone (angle mode). Runs the simulation for that time and reports.',
    params: obj({ controls: { type: 'array', items: { type: 'string', enum: CTRLS } }, seconds: { type: 'number' }, level: { type: 'string', enum: ['gentle', 'normal', 'sport'] } }, ['controls', 'seconds']),
    run: async a => {
      if (a.level) setPilotLevel(a.level);
      const cs = (a.controls || []).filter(c => CTRLS.includes(c)); for (const c of cs) press(c, 'agent');
      try { return await agentRun(clamp(num(a.seconds, 1), 0.1, 30), {}); } finally { for (const c of cs) release(c, 'agent'); }
    } },
  poke: { desc: 'Poke the drone: a random spin and shove, as the Poke button (strength 0..1: up to 12 rad/s and 3 m/s).', params: obj({ strength: { type: 'number' } }),
    run: a => { if (S.crashed) throw new Error('crashed: reset first'); const c = clamp(num(a.strength, 0.5), 0, 1), spin = 1.5 + 10.5 * c, push = 0.3 + 2.7 * c, A = Math.random() * 2 * Math.PI, B2 = Math.random() * 2 * Math.PI;
      S.w = add(S.w, [Math.cos(A) * spin, Math.sin(A) * spin, (Math.random() - 0.5) * spin * 0.5]); S.v = add(S.v, [Math.cos(B2) * push, Math.sin(B2) * push, 0]); return { spin_rad_s: rnd(spin, 1), shove_m_s: rnd(push, 1) }; } },
  set_view: { desc: 'The 3D view: camera (top, front, right, back, left, bottom, iso), projection (persp, ortho), follow the drone, chase camera, and what it draws (layers: thrust, weight, wind, lift, drag, torque, rtorque, ntorque, want, spin, servo, cog, beam, air, trail, est, target, heading, grid, shadow, readouts, legend).',
    params: obj({ camera: { type: 'string', enum: ['top', 'front', 'right', 'back', 'left', 'bottom', 'iso'] }, projection: { type: 'string', enum: ['persp', 'ortho'] }, follow: { type: 'boolean' }, chase: { type: 'boolean' }, layers: { type: 'object' } }),
    run: a => {
      if (a.camera) snapView(a.camera); if (a.projection) setProjection(a.projection);
      if (a.follow != null && a.follow !== view.follow) $('#tFollow').click(); if (a.chase != null && a.chase !== view.chase) $('#tChase').click();
      if (a.layers) setLayers(Object.fromEntries(Object.entries(a.layers).filter(([k]) => LAYERS.some(L => L.key === k))));
      return { follow: view.follow, chase: view.chase, layers: Object.fromEntries(LAYERS.map(L => [L.key, view[L.key]])) };
    } },
  set_control: { desc: 'Allocation preferences (each 0..0.2: allowance keeps margin, efficiency saves power, servoMove is the cost of moving servos) and mix_share (Mixed steering: the servos\' share of sideways force, 0..1).',
    params: obj({ allowance: { type: 'number' }, efficiency: { type: 'number' }, servoMove: { type: 'number' }, mix_share: { type: 'number' } }),
    run: a => { for (const k of ['allowance', 'efficiency', 'servoMove']) if (a[k] != null) allocPrefs[k] = clamp(num(a[k], allocPrefs[k]), 0, 0.2);
      if (a.mix_share != null) { steerMix.share = clamp(num(a.mix_share, steerMix.share), 0, 1); steerMix.rho = 1; }
      for (const r of allocFieldRefs) r(); save(); return { allowance: allocPrefs.allowance, efficiency: allocPrefs.efficiency, servoMove: allocPrefs.servoMove, mix_share: steerMix.share }; } },
  set_frame_shape: { desc: 'The frame\'s shape in the air: aero "prism" (a box hub: drag only) or "wing" (lift and drag), and for a wing its span, chord, thick(ness) [m] and inc(idence) [deg].',
    params: obj({ aero: { type: 'string', enum: ['prism', 'wing'] }, span: { type: 'number' }, chord: { type: 'number' }, thick: { type: 'number' }, inc: { type: 'number' } }),
    run: a => { setFrameShape({ ...frameShapeOf(), ...Object.fromEntries(Object.entries(a).filter(([, v]) => v != null)) }); renderFrameShape(); undoKey = null; recomputeProps(); cPts = contactPoints(); rebuildDrone(); refreshEnvelope(); renderMass(); save(); return frameShapeOf(); } },

  designs: { desc: 'Saved designs: list them, save the airframe under a name, open one (by name or id; the airframe on screen is replaced, and it can be undone), undo or redo an airframe change, share (a link with the whole design in it), or open_shared (a link or code someone shared).',
    params: obj({ action: { type: 'string', enum: ['list', 'save', 'open', 'undo', 'redo', 'share', 'open_shared'] }, name: { type: 'string' }, id: { type: 'string' }, code: { type: 'string' } }, ['action']),
    run: async a => {
      if (a.action === 'list') return { current: designs.name || null, designs: designs.list.map(d => ({ id: d.id, name: d.name, saved: new Date(d.savedAt).toISOString().slice(0, 16) })) };
      if (a.action === 'share') { const code = await designCode(a.name || designs.name || 'Untitled design'); return { link: shareBase() + '#' + SHARE_KEY + code, code_length: code.length }; }
      if (a.action === 'open_shared') { const ok = await openDesignCode(a.code || '', 'a shared code'); return ok ? { opened: designs.name, parts: cfg.comps.length } : { error: 'the person chose to keep the airframe on screen' }; }
      if (a.action === 'undo') { undoStep(); return { undone: true }; } if (a.action === 'redo') { redoStep(); return { redone: true }; }
      if (a.action === 'save') { $('#designName').value = a.name || designs.name || 'Untitled design'; await saveDesign(); return { saved: designs.name }; }
      const d = designs.list.find(x => x.id === a.id || x.name === a.name); if (!d) throw new Error('no such design (list them)');
      openDesign(d); return { opened: d.name, parts: cfg.comps.map(partBrief) };
    } },
  cargo_items: { desc: 'The things to pick up in the world (when the airframe has a latch): list them, or set the whole list (up to 8): each { name, mass [kg], size [x,y,z m], at [x,y] from the start point }. They move there at the next reset.',
    params: obj({ items: { type: 'array', items: { type: 'object' } } }),
    run: a => { if (a.items) { cargoWorld.items = a.items.slice(0, 8).map((it, i) => ({ name: String(it.name || 'Parcel ' + (i + 1)), mass: Math.max(0.001, +it.mass || 0.2), size: (it.size || [0.1, 0.1, 0.08]).map(v => Math.max(0.005, +v || 0.05)), at: (it.at || [1, 0]).map(v => +v || 0) })); cargoWorldSave(); if (typeof cargoItemsBuild === 'function') cargoItemsBuild(); }
      return { items: cargoWorld.items, loose_now: cargo.loose.map(L => ({ name: L.name, at: rndA(L.p), mass: rnd(L.m) })) }; } },
  triggers: { desc: 'Your triggers (they ask you when something happens): list them, add one, change one (by id), or delete one. Kinds: crash, battery (value %), far (value m), tilt (value deg), formula, every (value s), expr (expr: a condition over the telemetry fields, e.g. "alt < 0.5 && flying"). Fields: msg (what to tell you), gap [s], keepFlying, on.',
    params: obj({ action: { type: 'string', enum: ['list', 'add', 'change', 'delete'] }, id: { type: 'string' }, kind: { type: 'string', enum: Object.keys(TRIGGER_KINDS) }, value: { type: 'number' }, expr: { type: 'string' }, msg: { type: 'string' }, gap: { type: 'number' }, keepFlying: { type: 'boolean' }, on: { type: 'boolean' } }, ['action']),
    run: a => {
      const pub = T => ({ id: T.id, kind: T.kind, value: T.value, expr: T.kind === 'expr' ? T.expr : undefined, msg: T.msg, gap: T.gap, keepFlying: !!T.keepFlying, on: T.on, fired: T.fired });
      if (a.action === 'list') return agent.triggers.map(pub);
      if (a.action === 'delete') { const T = agent.triggers.find(x => x.id === a.id); if (!T) throw new Error('no such trigger'); agent.triggers = agent.triggers.filter(x => x !== T); const th = agent.threads.find(t => t.kind === 'trigger' && t.triggerId === T.id); if (th && th !== agent.turn) threadDelete(th.id); agentSave(); if (typeof agentRender === 'function' && aiUi.view === 'list') agentRender(); return { deleted: T.id }; }
      let T = a.action === 'add' ? null : agent.triggers.find(x => x.id === a.id);
      if (a.action === 'change' && !T) throw new Error('no such trigger');
      if (!T) { if (agent.triggers.length >= 12) throw new Error('12 triggers at most'); T = { id: 't' + Date.now().toString(36), kind: 'crash', value: null, expr: '', msg: '', gap: 10, keepFlying: false, on: true, fired: 0, last: -1e9, was: false }; agent.triggers.push(T); }
      for (const k of ['kind', 'value', 'expr', 'msg', 'gap', 'keepFlying', 'on']) if (a[k] != null) T[k] = a[k];
      if (T.value == null && TRIGGER_KINDS[T.kind].value != null) T.value = TRIGGER_KINDS[T.kind].value;
      if (T.kind === 'expr' && !triggerTest(T)) { const err = triggerFn.get(T.id).err; if (a.action === 'add') agent.triggers = agent.triggers.filter(x => x !== T); throw new Error('the expression: ' + err); }
      T.was = false; agentSave(); const th = threadForTrigger(T); th.title = triggerText(T); agentSaveThreads(); return pub(T);
    } },

  look: { desc: 'Look at the 3D view as it is now: a picture of the simulator\'s view (only when Settings say the model can see images). Set the view first (set_view) to look from a side or from above.',
    params: obj({}),
    run: () => {
      if (!agent.cfg.canSee) throw new Error('the person hasn\'t said this model can see images (AI tab, Settings)');
      renderer.render(scene, camera);
      const src = renderer.domElement, w = Math.min(768, src.width), h = Math.round(src.height * w / src.width), cv = document.createElement('canvas'); cv.width = w; cv.height = h;
      cv.getContext('2d').drawImage(src, 0, 0, w, h);
      return { text: 'The view is attached as an image after this.', __image: cv.toDataURL('image/jpeg', 0.75) };
    } },
  run_js: { desc: 'Run JavaScript in the page, for anything the other tools don\'t reach (only when Settings allow it). The code is a function body; return a value (kept short). All of the simulator\'s globals are in scope (cfg, S, brt, LAWS…).',
    params: obj({ code: { type: 'string' } }, ['code']),
    run: async (a, item) => {
      if (!agent.cfg.allowJs) throw new Error('the person hasn\'t allowed running JavaScript (AI tab, Settings)');
      if (agent.cfg.askJs && !(await agentConfirm(item, 'Run this JavaScript in the page?'))) return { error: 'the person declined to run it' };
      let out = await (new Function('"use strict";\nreturn (async () => {\n' + a.code + '\n})();'))();
      try { out = JSON.parse(JSON.stringify(out ?? null, (k, v) => (typeof v === 'number' && !isFinite(v) ? String(v) : v))); } catch (e) { out = String(out); }
      return { result: out };
    } },
});
