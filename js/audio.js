'use strict';
// Sound, made from the simulation as it runs (Web Audio, nothing recorded): off until the speaker button turns it on.
//
// Each motor: the blade-pass tone (rpm/60 × 2 blades, with its harmonics: the hum), the motor's electrical whine
// (rpm/60 × 7 pole pairs, louder with current) and the rush of its wash (noise around the blade tone, with thrust).
// Each is as loud as the motor works, panned to where it is from the camera, quieter the further the camera is.
// Motors at slightly different speeds beat against each other, as real ones do.
// Each servo: a gear whine while it moves, higher and louder the faster. A latch: its servo's whirr while it moves,
// and a clunk when it closes or a click when it opens. A prop strike: a clatter. A crash: a thud. Arming: the ESCs'
// three rising beeps. A battery under 20%: a beeper every few seconds.

const SND_BLADES = 2, SND_POLES = 7, SND_MAX_MOTORS = 8, SND_MAX_SERVOS = 6;
const snd = { on: false, ctx: null, master: null, noise: null, wave: null, motors: new Map(), servos: new Map(), events: new Map() };
// Playback changes pitch and stretches simulated effects; real-device telemetry stays in real time.
function sndRate() { return typeof liveOn === 'function' && liveOn() ? 1 : Number.isFinite(speed) && speed > 0 ? speed : 1; }

function sndNoiseBuffer(ctx) {
  const b = ctx.createBuffer(1, ctx.sampleRate * 2, ctx.sampleRate), d = b.getChannelData(0);
  for (let i = 0; i < d.length; i++) d[i] = Math.random() * 2 - 1;
  return b;
}
function sndStart() {
  if (snd.ctx) { snd.ctx.resume(); return; }
  const ctx = snd.ctx = new (window.AudioContext || window.webkitAudioContext)();
  const comp = ctx.createDynamicsCompressor(); comp.threshold.value = -18; comp.ratio.value = 4; comp.connect(ctx.destination);
  snd.master = ctx.createGain(); snd.master.gain.value = 0; snd.master.connect(comp);
  snd.noise = sndNoiseBuffer(ctx);
  // a prop's tone: the blade-pass fundamental and its harmonics, falling off (a buzzy hum, not a pure whistle)
  const n = 12, re = new Float32Array(n), im = new Float32Array(n); for (let k = 1; k < n; k++) im[k] = 1 / Math.pow(k, 1.3);
  snd.wave = ctx.createPeriodicWave(re, im);
}
function sndNoiseSrc() { const s = snd.ctx.createBufferSource(); s.buffer = snd.noise; s.loop = true; s.loopStart = Math.random(); s.start(); return s; }

/* ───────── voices that last: one per motor, one per servo ───────── */
function motorVoice() {
  const ctx = snd.ctx, out = ctx.createStereoPanner(), g = ctx.createGain(); g.gain.value = 0; g.connect(out); out.connect(snd.master);
  const hum = ctx.createOscillator(); hum.setPeriodicWave(snd.wave); const hg = ctx.createGain(); hg.gain.value = 0; hum.connect(hg); hg.connect(g); hum.start();
  const whine = ctx.createOscillator(); whine.type = 'triangle'; const wg = ctx.createGain(); wg.gain.value = 0; whine.connect(wg); wg.connect(g); whine.start();
  const ns = sndNoiseSrc(), bp = ctx.createBiquadFilter(); bp.type = 'bandpass'; bp.Q.value = 0.7; const ng = ctx.createGain(); ng.gain.value = 0; ns.connect(bp); bp.connect(ng); ng.connect(g);
  return { out, g, hum, hg, whine, wg, bp, ng, ns, stop: () => { for (const o of [hum, whine, ns]) try { o.stop(); } catch (e) {} out.disconnect(); } };
}
function servoVoice() {
  const ctx = snd.ctx, out = ctx.createStereoPanner(), g = ctx.createGain(); g.gain.value = 0; g.connect(out); out.connect(snd.master);
  const o = ctx.createOscillator(); o.type = 'square'; const f = ctx.createBiquadFilter(); f.type = 'bandpass'; f.Q.value = 3; o.connect(f); f.connect(g); o.start();
  return { out, g, o, f, stop: () => { try { o.stop(); } catch (e) {} out.disconnect(); } };
}

/* ───────── one-off sounds ───────── */
function sndBurst({ dur = 0.2, f = 800, q = 1, gain = 0.5, type = 'bandpass', pan = 0, at = 0, drop = 0 }) {   // filtered noise
  const rate = sndRate(); dur /= rate;
  const ctx = snd.ctx, t = ctx.currentTime + at / rate, s = ctx.createBufferSource(); s.buffer = snd.noise; s.playbackRate.value = rate;
  const fl = ctx.createBiquadFilter(); fl.type = type; fl.frequency.setValueAtTime(f * rate, t); fl.Q.value = q; if (drop) fl.frequency.exponentialRampToValueAtTime(Math.max(40, f * drop) * rate, t + dur);
  const g = ctx.createGain(), p = ctx.createStereoPanner(); p.pan.value = clamp(pan, -1, 1);
  g.gain.setValueAtTime(0.0001, t); g.gain.exponentialRampToValueAtTime(gain, t + 0.004 / rate); g.gain.exponentialRampToValueAtTime(0.0001, t + dur);
  s.connect(fl); fl.connect(g); g.connect(p); p.connect(snd.master); s.start(t, Math.random()); s.stop(t + dur + 0.05 / rate);
}
function sndTone({ f = 1000, f2 = 0, dur = 0.12, gain = 0.25, type = 'sine', at = 0, pan = 0 }) {
  const rate = sndRate(); dur /= rate;
  const ctx = snd.ctx, t = ctx.currentTime + at / rate, o = ctx.createOscillator(), g = ctx.createGain(), p = ctx.createStereoPanner(); p.pan.value = clamp(pan, -1, 1);
  o.type = type; o.frequency.setValueAtTime(f * rate, t); if (f2) o.frequency.exponentialRampToValueAtTime(f2 * rate, t + dur);
  g.gain.setValueAtTime(0.0001, t); g.gain.exponentialRampToValueAtTime(gain, t + 0.01 / rate); g.gain.exponentialRampToValueAtTime(0.0001, t + dur);
  o.connect(g); g.connect(p); p.connect(snd.master); o.start(t); o.stop(t + dur + 0.05 / rate);
}
const sndCrash = (hard, pan) => { sndBurst({ dur: 0.5, f: 300, type: 'lowpass', gain: 0.9 * hard, drop: 0.25, pan }); sndTone({ f: 110, f2: 40, dur: 0.35, gain: 0.6 * hard, pan }); sndBurst({ dur: 0.25, f: 2500, q: 0.8, gain: 0.25 * hard, at: 0.02, pan }); };
const sndStrike = pan => { for (let k = 0; k < 7; k++) sndBurst({ dur: 0.035, f: 2500 + Math.random() * 2500, q: 2, gain: 0.45 * (1 - k / 8), at: k * 0.025 + Math.random() * 0.01, pan }); };
const sndClunk = pan => { sndBurst({ dur: 0.08, f: 900, q: 4, gain: 0.5, pan }); sndTone({ f: 180, f2: 90, dur: 0.08, gain: 0.35, pan }); };
const sndClick = pan => { sndBurst({ dur: 0.03, f: 3200, q: 5, gain: 0.35, pan }); };
const sndArm = () => [523, 659, 784].forEach((f, i) => sndTone({ f, dur: 0.16, gain: 0.12, type: 'square', at: i * 0.18 }));
const sndLowBatt = () => [0, 0.22].forEach(at => sndTone({ f: 2700, dur: 0.12, gain: 0.09, type: 'square', at }));

/* ───────── each frame: the voices follow the simulation ───────── */
const sndTmp = new THREE.Vector3();
function sndPan(world) {   // left–right from the camera, and how far
  sndTmp.set(...world).applyMatrix4(camera.matrixWorldInverse);
  const d = sndTmp.length(); return { pan: clamp(sndTmp.x / Math.max(0.3, d), -0.9, 0.9), d };
}
function sndResetScope() {
  for (const v of [...snd.motors.values(),...snd.servos.values()]) v.stop();
  snd.motors.clear();snd.servos.clear();snd.events.clear();
}
// Changing selection is silent: preserve matching voices and seed event history anew.
function sndSelectScope() { snd.events.clear(); }
function sndRetireVoice(v) {
  v.g.gain.cancelScheduledValues(snd.ctx.currentTime);
  v.g.gain.setTargetAtTime(0,snd.ctx.currentTime,.02);
  setTimeout(v.stop,100);
}
function sndTick() {
  if (!snd.on || !snd.ctx) return;
  const live = running && !editMode && !document.hidden;
  snd.master.gain.setTargetAtTime(live ? .9 : 0,snd.ctx.currentTime,.05);
  const seen=new Set(), sj=new Set();
  if (typeof fleet !== 'undefined' && fleet.ready) {
    const drones=fleet.selected ? [fleet.selected] : fleet.drones;
    for(const d of drones)withDrone(d,()=>sndDroneTick(d.id,live,seen,sj));
    for(const id of snd.events.keys())if(!fleet.drones.some(d=>d.id===id))snd.events.delete(id);
  } else sndDroneTick('boot',live,seen,sj);
  for(const [id,v] of snd.motors)if(!seen.has(id)){sndRetireVoice(v);snd.motors.delete(id);}
  for(const [id,v] of snd.servos)if(!sj.has(id)){sndRetireVoice(v);snd.servos.delete(id);}
}
function sndDroneTick(owner,live,seen,sj) {
  const ctx=snd.ctx,t=ctx.currentTime,k=.04,rate=sndRate();
  let last=snd.events.get(owner);
  if(!last){
    last={fc:brt.fcState,crash:!!S.crashed,lowT:S.t,clock:S.t};
    for(const c of actuators())last['p'+c.id]=!!hs.get(c.id)?.prop;
    snd.events.set(owner,last);
  }
  // Resetting a flight also restarts its reminder interval; pause cannot advance this clock.
  if (S.t < last.clock) last.lowT = S.t;
  last.clock = S.t;
  const R = qmat(S.q), at = p => add(S.p, m3v(R, p));
  // motors
  const ms = actuators().slice(0, SND_MAX_MOTORS);
  for (const c of ms) {
    const key=owner+':'+c.id;seen.add(key);
    let v = snd.motors.get(key); const fresh = !v; if (fresh) { v = motorVoice(); snd.motors.set(key, v); }
    const st = act.get(c.id) || {}, rps = Math.max(0, (st.Omega || 0) / (2 * Math.PI)), hsc = hs.get(c.id) || {};
    const full = propOmega(c) / (2 * Math.PI), frac = clamp(rps / Math.max(1, full), 0, 1.3), on = cargo.power && !cargo.off.has(c.id);
    const { pan, d } = sndPan(at(c.pos)), near = 1 / (1 + Math.max(0, d - 1) * 0.35);
    const loud = on ? Math.pow(frac, 1.6) * near * (hsc.prop ? 0.25 : 1) : 0;
    if (fresh) { // Begin at the actual rotor frequency, without a 440 Hz oscillator startup sweep.
      v.hum.frequency.value=Math.max(1,rps*SND_BLADES)*rate; v.whine.frequency.value=Math.max(1,rps*SND_POLES)*rate;
      v.bp.frequency.value=Math.max(80,rps*SND_BLADES*5)*rate; v.ns.playbackRate.value=rate;
    }
    v.hum.frequency.setTargetAtTime(Math.max(1, rps * SND_BLADES) * rate, t, k);
    v.whine.frequency.setTargetAtTime(Math.max(1, rps * SND_POLES) * rate, t, k);
    v.bp.frequency.setTargetAtTime(Math.max(80, rps * SND_BLADES * 5) * rate, t, k);
    v.ns.playbackRate.setTargetAtTime(rate, t, k);
    v.hg.gain.setTargetAtTime(0.16 * loud, t, k);
    v.wg.gain.setTargetAtTime(0.03 * clamp(Math.abs(st.i || 0) / 20, 0, 1) * near, t, k);
    v.ng.gain.setTargetAtTime(0.2 * clamp((st.T || 0) / Math.max(0.5, c.tmax), 0, 1) * near, t, k);
    v.g.gain.setTargetAtTime(1, t, k); v.out.pan.setTargetAtTime(pan, t, k);
    // a prop strike: once, when the prop breaks
    const was = last['p' + c.id]; if (hsc.prop && !was) sndStrike(pan); last['p' + c.id] = !!hsc.prop;
  }
  // servos: a gear whine while they move
  const js = joints().slice(0, SND_MAX_SERVOS);
  for (const j of js) {
    const key=owner+':'+j.id;sj.add(key);
    let v = snd.servos.get(key); if (!v) { v = servoVoice(); snd.servos.set(key, v); }
    const st = jst.get(j.id) || {}, w = Math.abs(st.rate || 0), { pan, d } = sndPan(at(j.pos)), near = 1 / (1 + Math.max(0, d - 1) * 0.35);
    const g = clamp(w / 4, 0, 1);
    v.o.frequency.setTargetAtTime((120 + 260 * clamp(w / 6, 0, 1.5)) * rate, t, 0.02); v.f.frequency.setTargetAtTime((900 + 1400 * clamp(w / 6, 0, 1.5)) * rate, t, 0.02);
    v.g.gain.setTargetAtTime(0.07 * g * near * (g > 0.03 ? 1 : 0), t, 0.02); v.out.pan.setTargetAtTime(pan, t, 0.05);
  }
  // latches: a click opening, a clunk closing (and a short whirr while they move, as their servo)
  for (const l of latches()) {
    const st = cargo.lat.get(l.id); if (!st) continue;
    const prev = last['l' + l.id], { pan } = sndPan(at(l.pos));
    if (prev != null && st.pos !== prev) {
      if (Math.abs(st.pos - prev) > 0.001) sndBurst({ dur: 0.05, f: 1800, q: 6, gain: 0.05, pan });
      if (st.pos >= 1 && prev < 1) sndClunk(pan); else if (st.pos <= 0 && prev > 0) sndClick(pan);
    }
    last['l' + l.id] = st.pos;
  }
  // a crash: a thud as hard as the hit; arming: the ESCs' beeps; a low battery: the beeper
  if (S.crashed && !last.crash) { const m = /([\d.]+) m\/s/.exec(S.crashed); sndCrash(clamp(m ? +m[1] / 6 : 0.7, 0.3, 1), sndPan(S.p).pan); }
  last.crash = !!S.crashed;
  if (brt.fcState === 1 && last.fc !== 1 && last.fc != null) sndArm();
  last.fc = brt.fcState;
  const soc = S.batt && S.batt.soc != null ? S.batt.soc : 1;
  if (live && cargo.power && soc < 0.2 && brt.fcState === 1 && S.t - last.lowT > 3) { last.lowT = S.t; sndLowBatt(); }
}
(function sndLoop() { try { sndTick(); } catch (e) {} requestAnimationFrame(sndLoop); })();

/* ───────── the button ───────── */
function setSound(on) {
  snd.on = !!on;
  if (snd.on) { sndStart(); sndResetScope(); }
  else if (snd.ctx) { snd.master.gain.setTargetAtTime(0, snd.ctx.currentTime, 0.03); setTimeout(() => { if (!snd.on && snd.ctx) snd.ctx.suspend(); }, 200); }
  const b = $('#tSound'); b.setAttribute('aria-pressed', String(snd.on)); b.classList.toggle('on', snd.on); b.title = snd.on ? 'Sound is on: click to mute (M)' : 'Sound is off: turn it on to hear the motors, servos, latches and crashes (M)';
}
$('#tSound').addEventListener('click', () => setSound(!snd.on));
window.addEventListener('keydown', e => { if (e.code === 'KeyM' && !e.repeat && !e.metaKey && !e.ctrlKey && !e.altKey && !typingIn(e.target) && !document.querySelector('dialog[open]')) setSound(!snd.on); });
