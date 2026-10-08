'use strict';
// Dedicated flight tuning tools share the UI's validation, staging, cancellation and undo paths.
function agentAutotune() {
  const a = brt.autotune;
  return { available: atAvailable() || true, phase: a?.phase || 'idle', loop: a?.loop || null,
    message: a?.message || '', progress: a?.tests ? { completed: a.index, total: a.tests.length } : null,
    saved_gains: tuneFix(tuneOf()), provisional_gains: brt.tuneTrial || null,
    recommendation: a?.recommendation || null, abort_metrics: a?.abortMetrics || null,
    attitude_verified: a?.attVerified === atAttKey() };
}
Object.assign(AGENT_TOOLS, {
  get_tuning: { desc: 'Saved PID gains, raw gain bounds, response/damping/integral settings and the formulas that read them. Attitude arrays are roll, pitch, yaw. These response settings are a simplified prediction, not measured stability.',
    params: obj({}), run: () => ({ gains: tuneFix(tuneOf()), bounds: TUNE_BOUNDS,
      response: { attitude: TUNE_AXES.map((axis, i) => ({ axis, ...tuneFeel(tuneOf(), 'att', i) })), position: tuneFeel(tuneOf(), 'pos') },
      formulas: tuneReaders(), autotune: agentAutotune() }) },
  set_tuning: { desc: 'Patch raw PID gains within get_tuning bounds. Each attitude gain is three numbers (roll, pitch, yaw); position gains are scalars. Shares manual Tune behavior: cancels any pending autotune, saves one undoable edit and reloads formulas. Does not prove flight stability.',
    params: obj({ att: obj(Object.fromEntries(['kR','kW','kI'].map(k => [k,{type:'array',items:{type:'number'},minItems:3,maxItems:3}]))),
      pos: obj(Object.fromEntries(['kp','kd','ki'].map(k => [k,{type:'number'}]))) }),
    run: a => {
      agentFields(a, {att:{type:'object'},pos:{type:'object'}}, 'tuning');
      if (!Object.values(a).some(loop => Object.keys(loop).length)) throw Error('Provide at least one PID gain.');
      const t = tuneFix(tuneOf());
      for (const loop of ['att','pos']) if (a[loop] != null) {
        const keys = loop === 'att' ? ['kR','kW','kI'] : ['kp','kd','ki'];
        agentFields(a[loop], Object.fromEntries(keys.map(k => [k,{type:loop==='att'?'array':'number'}])), loop);
        for (const [k,v] of Object.entries(a[loop])) {
          const values = loop === 'att' ? v : [v], [lo,hi] = TUNE_BOUNDS[k];
          if (loop === 'att' && values.length !== 3 || values.some(x => typeof x !== 'number' || !Number.isFinite(x) || x < lo || x > hi)) throw Error(k + ': use finite gains within ' + lo + '…' + hi);
          t[loop][k] = loop === 'att' ? v.slice() : v;
        }
      }
      setTuning(t, 'agent'); return AGENT_TOOLS.get_tuning.run();
    } },
  autotune: { desc: 'Simulator measured autotune: status, measure_attitude, measure_position, apply_verify, stop. Calibrate the airframe, take off and hover with controls released first; attitude must be verified before position. apply_verify stages provisional gains and repeats flight tests; saved gains change only on success. Use wait then status to follow progress. Failed signal/fit/motion tests keep the prior gains. Supports fixed motors, tilt mode and standard control formulas only.',
    params: obj({ action:{type:'string',enum:['status','measure_attitude','measure_position','apply_verify','stop']} }, ['action']),
    run: a => {
      if (!['status','measure_attitude','measure_position','apply_verify','stop'].includes(a.action)) throw Error('Unknown autotune action');
      if (a.action.startsWith('measure_')) {
        if (atBusy()) throw Error('An autotune test is already running; wait or stop it first.');
        const why = atAvailable(); if (why) throw Error(why);
        if (a.action === 'measure_position' && brt.autotune?.attVerified !== atAttKey()) throw Error('Verify attitude tuning before measuring position hold.');
        atStart(a.action === 'measure_position' ? 'pos' : 'att');
      } else if (a.action === 'apply_verify') {
        if (brt.autotune?.phase !== 'review' || !brt.autotune.recommendation) throw Error('Measure successfully before applying and verifying.');
        atApply();
      } else if (a.action === 'stop') atStop();
      return agentAutotune();
    } },
  get_runtime: { desc: 'Selected drone board runtimes, custom formula programs and C/WebAssembly/native/Python apps, with status and logs. Optional kind and id read one saved source/header (chunked); native/Python apps are not simulated. Built-in control formulas use get_formula.',
    params: obj({kind:{type:'string',enum:['program','app']},id:{type:'string'},offset:{type:'integer',minimum:0},limit:{type:'integer',minimum:1,maximum:3000}}),
    run: a => {
      if (a.id) {
        if (!['program','app'].includes(a.kind)) throw Error('Choose program or app');
        const p = (a.kind === 'program' ? programs() : apps()).find(p => p.id === a.id); if (!p) throw Error('Unknown ' + a.kind + ' id');
        const {src,bin,binOf,...header} = p, offset = Math.max(0,num(a.offset,0)), limit = clamp(num(a.limit,2000),1,3000);
        return {header,source:src.slice(offset,offset+limit),offset,total:src.length,next:offset+limit<src.length?offset+limit:null,
          interface: a.kind === 'app' ? (p.kind === 'python' ? appShapePython(p) : appHeaderC(p)) : 'function '+p.name+'(st, inp, dt) returns fields of writes.layout; inp topic aliases: '+progAliases(p.reads).join(', ')};
      }
      const ps = progStats(), as = appStats();
      return {boards:computers().boards.map(b=>({id:b.id,name:b.name,runs:boardRuns(b),tasks:b.tasks,apps:b.apps||[]})),
        programs:programs().map(({src,...p})=>({...p,status:ps.get(p.id)||null,error:brt.progErr?.get(p.id)||programProblem(p)||null})),
        apps:apps().map(({src,bin,binOf,...p})=>({...p,status:as.get(p.id)||null,board:appBoard(p)?.id??null,simulated:p.kind==='wasm',error:brt.appErr?.get(p.id)||appProblem(p)||null,log:(brt.appLog?.get(p.id)||[]).slice(-4)}))};
    } },
  get_bus: { desc: 'Selected drone topic catalog and live per-board values, freshness, mirror status and rejected writes. Filter by board id and/or topic to inspect a sensor, program or app without truncating the full bus.',
    params: obj({board:{type:'integer'},topic:{type:'string'}}),
    run: a => ({catalog:busCatalog().filter(t=>(a.board==null||t.board.id===a.board)&&(!a.topic||t.name===a.topic)).map(t=>({name:t.name,layout:t.layout,board:t.board.id,from:t.from})),
      boards:busSnapshot().filter(x=>a.board==null||x.b.id===a.board).map(x=>({id:x.b.id,name:x.b.name,topics:x.topics.filter(t=>!a.topic||t.name===a.topic)}))}) },
});
