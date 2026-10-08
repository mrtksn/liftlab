'use strict';
// Measured tuning. The browser coordinates bounded tests over the board link and analyses board telemetry;
// no physics state, true inertia/mass or configured motor lag participates in the recommendations.
// The core's mode-3 EXC is closed loop and expires in 100 ms. Position tests offset only nav's input target.
const AT_FREQ = { att: [0.8, 1.6, 3.2, 6.4], pos: [0.18, 0.32, 0.55] };
const atMul = (a, b) => [a[0]*b[0]-a[1]*b[1], a[0]*b[1]+a[1]*b[0]];
const atDiv = (a, b) => { const d=b[0]*b[0]+b[1]*b[1]; return [(a[0]*b[0]+a[1]*b[1])/d,(a[1]*b[0]-a[0]*b[1])/d]; };
const atAbs = a => Math.hypot(...a);
function atPlant(hz, model) {
  const w=2*Math.PI*hz, a=w*model.delay;
  return atMul(atDiv([-model.gain*Math.cos(a)/(w*w), model.gain*Math.sin(a)/(w*w)], [1,w*model.lag]),[1,w*(model.lead||0)]);
}
function atResponse(hz, gains, model, inner) {
  const w=2*Math.PI*hz, [P,D,I]=gains;
  let G=atPlant(hz,model);
  if (inner) G=atMul(G,atResponse(hz,inner.gains,{...inner.model,outer:null}));
  const feedback=[P,D*w-I/w], reference=[P,-I/w];
  if(model.outer) { const [p,d,i]=model.outer, nav=atMul(reference,[-p/(w*w),-(d*w-i/w)/(w*w)]);feedback[0]+=nav[0];feedback[1]+=nav[1]; }
  const L=atMul(G,feedback), ref=atMul(G,reference);
  return atDiv(ref,[1+L[0],L[1]]);
}
// Sin/cos + mean + drift regression, using actual telemetry timestamps. The residual measures noise and
// off-frequency motion; it is not a confidence claim based on a perfect simulated sensor.
function atBin(samples,hz,amp,rate=true) {
  const w=2*Math.PI*hz, M=Array.from({length:4},()=>Array(5).fill(0)), t0=samples[0]?.[0] || 0;
  for (const [t,y] of samples) {
    const x=[Math.sin(w*t),Math.cos(w*t),1,t-t0];
    for(let a=0;a<4;a++){ for(let b=0;b<4;b++) M[a][b]+=x[a]*x[b]; M[a][4]+=x[a]*y; }
  }
  for(let a=0;a<4;a++) {
    let pivot=a; for(let b=a+1;b<4;b++) if(Math.abs(M[b][a])>Math.abs(M[pivot][a])) pivot=b;
    [M[a],M[pivot]]=[M[pivot],M[a]];
    const d=M[a][a]; if(Math.abs(d)<1e-10) return {hz,quality:0,T:[0,0],noise:Infinity};
    for(let b=a;b<5;b++) M[a][b]/=d;
    for(let b=0;b<4;b++) if(b!==a){ const k=M[b][a]; for(let j=a;j<5;j++) M[b][j]-=k*M[a][j]; }
  }
  const coef=M.map(r=>r[4]); let err=0;
  for(const [t,y] of samples) err+=(y-coef[0]*Math.sin(w*t)-coef[1]*Math.cos(w*t)-coef[2]-coef[3]*(t-t0))**2;
  const noise=Math.sqrt(err/Math.max(1,samples.length)), signal=Math.hypot(coef[0],coef[1]);
  const H=[coef[0]/amp,coef[1]/amp], T=rate ? [H[1]/w,-H[0]/w] : H;
  return {hz,T,noise,quality:signal>0?signal*signal/(signal*signal+2*noise*noise):0,samples:samples.length};
}
function atFit(bins,gains,inner,outer,diagnostic=false) {
  const usable=bins.filter(b=>b.quality>0.65 && b.samples>=60 && atAbs(b.T)>0.015);
  if(usable.length<3) return null;
  let best=null;
  const plant=usable.every(b=>b.plant);
  for(let lag=0;lag<=0.1601;lag+=0.01) for(let delay=0;delay<=0.0401;delay+=0.004) for(let gain=0.6;gain<=1.4001;gain+=0.05) for(const lead of plant?[0,0.02,0.04,0.08,0.12,0.2]:[0]) {
    const model={lag,delay,gain,lead,outer:plant?null:outer}; let err=0;
    for(const b of usable) { const t=plant?atPlant(b.hz,model):atResponse(b.hz,gains,model,inner),want=plant?b.plant:b.T; err+=b.quality*((t[0]-want[0])**2+(t[1]-want[1])**2)/Math.max(plant?1e-8:0.04,atAbs(want)**2); }
    err/=usable.length; if(!best || err<best.error) best={...model,error:err,quality:Math.min(...usable.map(b=>b.quality))};
  }
  return best.error<0.15 || diagnostic ? best : null;
}
function atMargins(gains,model,inner) {
  let last=Infinity, margin=null, peak=0, sensitivity=0;
  for(let k=0;k<220;k++) {
    const hz=0.02*Math.pow(2500,k/219), w=2*Math.PI*hz;
    let G=atPlant(hz,model); if(inner) G=atMul(G,atResponse(hz,inner.gains,{...inner.model,outer:null}));
    const feedback=[gains[0],gains[1]*w-gains[2]/w];
    if(model.outer) {const [p,d,i]=model.outer, nav=atMul([gains[0],-gains[2]/w],[-p/(w*w),-(d*w-i/w)/(w*w)]);feedback[0]+=nav[0];feedback[1]+=nav[1];}
    const L=atMul(G,feedback), mag=atAbs(L);
    if(last>=1 && mag<1) {
      let phase=-Math.PI-Math.atan(w*model.lag)-w*model.delay+Math.atan(w*(model.lead||0))+Math.atan2(feedback[1],feedback[0]);
      if(inner) { const T=atResponse(hz,inner.gains,{...inner.model,outer:null}); phase+=Math.atan2(T[1],T[0]); }
      margin=(Math.PI+phase)*180/Math.PI;
    }
    last=mag; peak=Math.max(peak,atAbs(atResponse(hz,gains,model,inner))); sensitivity=Math.max(sensitivity,1/atAbs([1+L[0],L[1]]));
  }
  // Conservative integral bound; avoid the low-frequency unstable mode hidden by a high crossover margin.
  const integralOk=gains[2]<gains[0]*gains[1]*0.35;
  return {margin,peak,sensitivity,safe:integralOk && margin>=45 && peak<=1.25 && sensitivity<=2};
}
function atRecommend(t,loop,models,inner) {
  const out=JSON.parse(JSON.stringify(t)), report=[];
  for(let axis=0;axis<(loop==='att'?3:1);axis++) {
    const model=models[axis]; if(!model) return null;
    const feel=tuneFeel(t,loop,axis), ceiling=loop==='att' ? Math.min(4,feel.hz*1.25) : Math.min(1,feel.hz*1.25,Math.sqrt(Math.min(...t.att.kR.slice(0,2)))/(2*Math.PI*4));
    let best=null;
    for(let hz=loop==='att'?0.3:0.08;hz<=ceiling+1e-8;hz+=loop==='att'?0.025:0.01) for(const zeta of [0.8,0.9,1,1.1]) {
      const integ=Math.min(feel.integ,loop==='att'?0.8:0.3), gains=tuneGains(hz,zeta,integ), m=atMargins(gains,model,inner);
      if(m.safe && (!best || hz>best.hz+1e-8 || (Math.abs(hz-best.hz)<1e-8 && zeta<best.zeta))) best={hz,zeta,integ,gains,...m};
    }
    if(!best) return null;
    if(loop==='att') ['kR','kW','kI'].forEach((k,i)=>out.att[k][axis]=best.gains[i]);
    else ['kp','kd','ki'].forEach((k,i)=>out.pos[k]=best.gains[i]);
    report.push(best);
  }
  return {tuning:out,report};
}

const atBusy = () => !!brt.autotune && ['measure','staging','verify'].includes(brt.autotune.phase);
const atAttKey = () => JSON.stringify([tuneOf().att,brt.modelSig]);
function atAvailable() {
  if(typeof live!=='undefined' && live.state!=='off') return 'Disconnect the real drone to run simulated tuning tests.';
  if(!brt.ready || !boardOf('learn') || !boardOf('nav') || !brt.navOut?.ready) return 'Needs Learning and Navigation tasks with a position estimate.';
  if(computers().boards.some(b=>brt.inst.get(b.id)?.host_phase()%10)) return 'Wait for the boards to finish the program change.';
  if(brt.pilot.phase!=='flying' || brt.fcState!==1) return 'Take off and hold still first.';
  if(typeof cableUnderLoad === 'function' && cableUnderLoad()) return 'Unload the cable before measured autotune; payload swing is outside its response model.';
  if(learn.view?.cal || learn.view?.thr || !learn.view?.haveFit) return 'Finish an airframe calibration first.';
  if(!rnReadsTune(rnSourceOf('attitudeControl')) || !rnReadsTune(rnSourceOf('positionControl'))) return 'The flight formulas must read the tuning gains.';
  // The fit assumes these control laws; a custom formula requires a matching identification model.
  if(LAWS.attitudeControl.src!==LAWS.attitudeControl.defSrc || LAWS.positionControl.src!==LAWS.positionControl.defSrc) return 'Restore the standard control formulas before autotuning.';
  if(pilot.vref.some(v=>Math.abs(v)>0.01) || brt.pickup) return 'Release the controls and finish cargo moves first.';
  if(steerJoints().length || mode!=='tilt') return 'Measured autotune currently supports fixed-motor aircraft in tilt mode.';
  return '';
}
function atExc(a,angle=0) {
  const nm=actuators().length,nj=joints().length,p=new Float32Array(8+nm+nj);
  p[0]=angle ? 3 : 0; p[4]=nm;p[5]=nj;p[6+nm+nj]=a.axis||0;p[7+nm+nj]=angle;
  sendFrame(boardOf('learn'),boardOf('core'),'exc',p);
  if(!angle) sendFrame(boardOf('learn'),boardOf('nav'),'navexc',Float32Array.of(0,0));
}
function atStop(reason='Test stopped; kept the previous tuning.') {
  const a=brt.autotune;if(!a || (!atBusy() && a.phase!=='review')) return;
  atExc(a);
  if(a.phase==='staging' || a.phase==='verify') { brt.tuneTrial=null; boardsStageProgram(); refreshTuning(); }
  boardsLearnCmd(a.keep?'keepOn':'keepOff'); a.phase='stopped';a.message=reason;
  atRender();
}
function atPlan(a) {
  a.tests=(a.loop==='att'?[0,1,2]:[0]).flatMap(axis=>AT_FREQ[a.loop].map(hz=>({axis,hz})));
  a.index=0;a.bins=[];a.samples=[];a.alpha=null;a.settleStart=null;a.start=brt.t;a.last=brt.t;a.maxMotor=0;a.maxRate=0;
}
function atStart(loop='att') {
  if(atBusy()) return;
  if(brt.autotune?.phase==='review') atStop('Starting a fresh measurement.');
  const why=atAvailable();if(why){ brt.autotune={phase:'stopped',message:why};atRender();return; }
  const old=brt.autotune, verified=old?.attVerified;
  if(loop==='pos' && verified!==atAttKey()) { if(old) old.message='Verify attitude tuning before measuring position hold.';atRender();return; }
  const a=brt.autotune={loop,phase:'measure',before:JSON.parse(JSON.stringify(tuneOf())),keep:learn.view.keep,attVerified:verified,
    attModels:old?.attModels,model:brt.modelSig,modelFlag:learn.view.useLearned,target:JSON.stringify(setpoint),sources:rnSources(),message:'Measuring response from board telemetry…'};
  if(loop==='pos') { const axis=a.before.att.kR[0]<=a.before.att.kR[1]?0:1; a.inner={gains:['kR','kW','kI'].map(k=>a.before.att[k][axis]),model:a.attModels[axis]}; }
  boardsLearnCmd('keepOff'); atPlan(a); atRender();
}
function atApply() {
  const a=brt.autotune;if(!a || a.phase!=='review' || !a.recommendation) return;
  if(atAvailable() || !tuneSame(a.before,tuneOf()) || a.model!==brt.modelSig || a.target!==JSON.stringify(setpoint) || Object.entries(a.sources).some(([k,src])=>src!==rnSourceOf(k))) {a.message='The flight changed; measure again before applying.';atRender();return;}
  a.phase='staging';a.stageStart=brt.t;a.stageSlots=Object.fromEntries(computers().boards.map(b=>[b.id,Math.floor(brt.inst.get(b.id).host_phase()/10)]));boardsLearnCmd('keepOff');brt.tuneTrial=JSON.parse(JSON.stringify(a.recommendation.tuning));
  boardsStageProgram();refreshTuning();a.message='Checking the recommended gains on the boards, then repeating the flight tests…';atRender();
}
function atTelemetry(p) {
  const a=brt.autotune;if(!a || !atBusy()) return;
  const bad=a.phase==='staging'?atAvailable().replace(/^Wait for the boards to finish the program change\.$/,''):atAvailable();
  if(bad || a.target!==JSON.stringify(setpoint) || Object.entries(a.sources).some(([k,src])=>src!==rnSourceOf(k)) || pilot.vref.some(v=>Math.abs(v)>0.01)) {atStop(bad || 'Controls or flight configuration changed; test stopped.');return;}
  if(!tuneSame(a.before,tuneOf()) || a.model!==brt.modelSig || !!(p[2]&4)!==a.modelFlag){atStop('Tuning or the flown model changed; measure again.');return;}
  const nm=p[17], load=Math.max(...p.slice(19,19+nm)), rate=Math.hypot(...p.slice(10,13));
  const q=p.slice(3,7), up=1-2*(q[1]*q[1]+q[2]*q[2]), n=brt.navOut;
  const home=brt.home, position=n.p.map((v,i)=>v+(home?.[i]||0)), target=[setpoint.x,setpoint.y,setpoint.z];
  if(p[1]!==1 || !(p[2]&1) || load>0.93 || rate>3 || up<0.94 || Math.hypot(...n.v)>1.5 || Math.hypot(...position.map((v,i)=>v-target[i]))>0.75 || brt.superView?.mode>0) {a.abortMetrics={load,rate,up,velocity:Math.hypot(...n.v),error:Math.hypot(...position.map((v,i)=>v-target[i])),supervisor:brt.superView?.mode};atStop('Test exceeded motion or motor headroom limits; kept the previous tuning.');return;}
  a.maxMotor=Math.max(a.maxMotor,load);a.maxRate=Math.max(a.maxRate,rate);
  if(a.phase==='staging') {
    const boards=computers().boards.filter(b=>b.tasks.includes('core') || b.tasks.includes('nav'));
    if(brt.t-a.stageStart>8){atStop('The boards did not finish applying the recommendation; restored the previous gains.');return;}
    if(brt.t-a.stageStart>2 && boards.every(b=>brt.inst.get(b.id).host_phase()%10===0 && Math.floor(brt.inst.get(b.id).host_phase()/10)!==a.stageSlots[b.id])) {
      a.trialSlots=boards.map(b=>[b.id,Math.floor(brt.inst.get(b.id).host_phase()/10)]);a.phase='verify';atPlan(a);
    }
    return;
  }
  if(a.phase==='verify' && a.trialSlots.some(([id,slot])=>Math.floor(brt.inst.get(id).host_phase()/10)!==slot)) {
    atStop('A board fell back from the trial program; restored the previous gains.');return;
  }
  const test=a.tests[a.index], dt=brt.t-a.last;a.last=brt.t;
  if(dt>0.04){atStop('Telemetry interrupted; test stopped.');return;}
  if(a.settleStart!=null) {
    if(Math.hypot(...n.v)<0.12 && Math.hypot(...position.map((v,i)=>v-target[i]))<0.2 && rate<0.4) {a.settleStart=null;a.start=brt.t;}
    else {if(brt.t-a.settleStart>20)atStop('The aircraft did not settle between tests; kept the previous gains.');return;}
  }
  const t=brt.t-a.start, warm=Math.max(2,Math.ceil(3*test.hz))/test.hz, duration=warm+Math.max(4,Math.ceil(2*test.hz))/test.hz,amp=a.loop==='att'?(test.hz<1.2?0.034907:0.05236):0.18;a.axis=test.axis;
  if(a.loop==='att') atExc(a,amp*Math.sin(2*Math.PI*test.hz*t));
  else sendFrame(boardOf('learn'),boardOf('nav'),'navexc',Float32Array.of(test.axis,0.18*Math.sin(2*Math.PI*test.hz*t)));
  if(t>warm) {
    if(a.loop==='att' && (!a.alpha || Math.abs(a.alpha[0]-p[0])>0.001)) {atStop('The command and gyro samples did not match; test stopped.');return;}
    a.samples.push([t,a.loop==='att'?p[10+test.axis]:n.v[test.axis],a.alpha?.[1+test.axis]||0]);
  }
  if(t<duration) return;
  atExc(a);const bin=atBin(a.samples,test.hz,amp,true);bin.axis=test.axis;a.bins.push(bin);
  if(a.loop==='att') {const command=atBin(a.samples.map(s=>[s[0],s[2]]),test.hz,amp,false);bin.plant=atDiv(bin.T,command.T);bin.quality=Math.min(bin.quality,command.quality);}
  a.samples=[];a.index++;a.start=brt.t;
  if(a.index<a.tests.length) {a.settleStart=brt.t;return;}
  if(a.phase==='measure') {
    const models=(a.loop==='att'?[0,1,2]:[0]).map(axis=>atFit(a.bins.filter(b=>b.axis===axis),a.loop==='att'?['kR','kW','kI'].map(k=>a.before.att[k][axis]):['kp','kd','ki'].map(k=>a.before.pos[k]),a.inner,a.loop==='att'&&axis<2?['kp','kd','ki'].map(k=>a.before.pos[k]):null));
    const recommendation=models.every(Boolean)?atRecommend(a.before,a.loop,models,a.inner):null;
    a.models=models;a.recommendation=recommendation;a.baseline=a.bins;a.baselineMotor=a.maxMotor;
    a.phase=recommendation?'review':'stopped';a.message=recommendation?'Measured a recommendation. Apply & verify repeats the tests before saving it.':'Insufficient signal or a poor response fit; kept the current gains. Calibrate or measure again in calmer conditions.';
    if(!recommendation) boardsLearnCmd(a.keep?'keepOn':'keepOff');
  } else {
    const verified=a.bins.every((b,i)=>b.quality>0.65 && atAbs(b.T)<=Math.max(1.25,atAbs(a.baseline[i].T)*1.02) && (b.hz>=(a.loop==='att'?1.2:0.4) || atAbs([b.T[0]-1,b.T[1]])<=Math.max(1,atAbs([a.baseline[i].T[0]-1,a.baseline[i].T[1]])*1.1))) && a.maxMotor<0.9;
    if(!verified){atStop('Verification found noise, oscillation or insufficient motor headroom; restored the previous gains.');return;}
    const candidate=a.recommendation.tuning;brt.tuneTrial=null;a.phase='done';
    setTuning(candidate,'autotune-'+a.loop);
    if(a.loop==='att'){a.attVerified=atAttKey();a.attModels=a.models;}
    boardsLearnCmd(a.keep?'keepOn':'keepOff');a.message='Verified in flight and saved. Undo restores the previous tuning.';
  }
  atRender();
}
function atRender() {
  if(typeof document==='undefined' || !document.getElementById('autotuneStatus')) return;
  const a=brt.autotune,busy=atBusy(),why=atAvailable();
  $('#autotuneAtt').disabled=busy || !!why;$('#autotunePos').disabled=busy || !!why || a?.attVerified!==atAttKey();
  $('#autotuneApply').disabled=a?.phase!=='review';$('#autotuneStop').hidden=!busy && a?.phase!=='review';
  $('#autotuneStatus').textContent=a?.message || why || 'Measure attitude first, then position hold. Each recommendation is verified before it is saved.';
  const progress=$('#autotuneProgress');progress.hidden=!busy;
  if(busy && a.tests) {const test=a.tests[a.index];progress.textContent=`${a.phase==='verify'?'Verifying':'Measuring'} ${a.loop==='att'?TUNE_AXES[test?.axis||0]:'position'} · ${test?.hz||0} Hz · ${Math.min(a.index+1,a.tests.length)}/${a.tests.length}`;}
  const results=$('#autotuneResults'),key=JSON.stringify(a?.recommendation || null);
  if(results._key===key) return;results._key=key;results.replaceChildren();
  if(a?.recommendation) {
    a.recommendation.report.forEach((r,i)=>{const p=document.createElement('p'),m=a.models[i];p.className='hint';p.textContent=`${a.loop==='att'?TUNE_AXES[i]:'Position'}: ${r.hz.toFixed(2)} Hz, damping ${r.zeta.toFixed(2)}, integral ${r.integ.toFixed(2)}/s · estimated lag ${Math.round(m.lag*1000)} ms, delay ${Math.round(m.delay*1000)} ms · predicted margin ${Math.round(r.margin)}°`;results.append(p);});
  }
}
if(typeof document!=='undefined') {
  $('#autotuneAtt').addEventListener('click',()=>atStart('att'));$('#autotunePos').addEventListener('click',()=>atStart('pos'));
  $('#autotuneApply').addEventListener('click',atApply);$('#autotuneStop').addEventListener('click',()=>atStop());
}
if(typeof module!=='undefined') module.exports={atBin,atFit,atResponse,atMargins,atRecommend};
