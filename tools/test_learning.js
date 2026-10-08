#!/usr/bin/env node
'use strict';
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const {lawSource,wasmRunner}=require('./lib');
const ctx=vm.createContext({Math,console});
for(const f of ['math.js','laws.js','tuning.js','autotune.js']) vm.runInContext(fs.readFileSync('js/'+f,'utf8'),ctx,{filename:f});
const rls=vm.runInContext('identifyEffectiveness',ctx), clone=x=>JSON.parse(JSON.stringify(x));
function inverse(a) {
 const n=a.length,M=a.map((r,i)=>r.concat(Array.from({length:n},(_,j)=>i===j?1:0)));
 for(let i=0;i<n;i++){let p=i;for(let j=i+1;j<n;j++)if(Math.abs(M[j][i])>Math.abs(M[p][i]))p=j;[M[i],M[p]]=[M[p],M[i]];
  const d=M[i][i];assert(Math.abs(d)>1e-12);M[i]=M[i].map(v=>v/d);
  for(let j=0;j<n;j++)if(j!==i){const k=M[j][i];M[j]=M[j].map((v,b)=>v-k*M[i][b]);}}
 return M.map(r=>r.slice(n));
}
(async()=>{
 const n=3,m=2*n,st={},dt=.005,memory=30,init=Array.from({length:6},()=>Array(n).fill(0));
 let info=Array.from({length:m},(_,i)=>Array.from({length:m},(_,j)=>i===j?.5:0)),maxErr=0;
 const P=rnCompileAll({identifyEffectiveness:lawSource('identifyEffectiveness')},RN_SIGS,{throw:true}),f=P.fns.identifyEffectiveness,A=rnArena(P);
 const W=await wasmRunner() || await RnWasm.create(Buffer.from(vm.runInNewContext(fs.readFileSync('js/rn-wasm.js','utf8')+'\nRN_WASM_B64'), 'base64'));
 assert(!W.load(P));
 for(let k=0;k<900;k++) {
  const t=k*dt,u=Array.from({length:n},(_,j)=>.45+.09*Math.sin((3+j*2)*t+j));
  const args=[st,u,[.05*Math.sin(t*3),.1*Math.cos(t*2),9.81+.3*Math.sin(t*7)],[.05*Math.sin(t*3),.1*Math.cos(t*4),.02*Math.sin(t*7)],[0,0,0],dt,init,memory,Array(n).fill(.035),{v:u,phi:Array(n).fill(1),m:[0,1,2],coll:Array(n).fill(0)}];
  const ret=rls(...args);
  if(st.x) {
   const lambda=Math.exp(-dt/memory);
   info=info.map((row,i)=>row.map((v,j)=>lambda*v+st.x[i]*st.x[j]));
   let cov=inverse(info),trace=cov.reduce((s,r,i)=>s+r[i],0);
   if(trace>2*m){const scale=2*m/trace;cov=cov.map(row=>row.map(v=>v*scale));info=info.map(row=>row.map(v=>v/scale));}
   for(let i=0;i<m;i++)for(let j=0;j<m;j++)maxErr=Math.max(maxErr,Math.abs(cov[i][j]-st.P[i][j]));
  }
  for(const arena of [A,W.A]) f.args.forEach((a,i)=>{if(!a.state)rnWrite(arena,a.addr,a.t,args[i]);});
  rnRun(P,A,'identifyEffectiveness');W.run('identifyEffectiveness');
  const actual=rnRead(A,f.ret.addr,f.ret.t),native=rnRead(W.A,f.ret.addr,f.ret.t);
  for(const [want,got] of [[ret,actual],[ret,native]]) for(const key of ['B','B2'])for(let i=0;i<want[key].length;i++)for(let j=0;j<n;j++)assert(Math.abs(want[key][i][j]-got[key][i][j])<.004,'RLS runner parity');
 }
 assert(maxErr<1e-8,'Full covariance differs from independent information-matrix reference: '+maxErr);
 assert(st.P.slice(n).some((row,i)=>Math.abs(row[n+i]-2)>.01),'Transient covariance never changed');
 assert(st.P.slice(0,n).some(row=>row.slice(n).some(v=>Math.abs(v)>.001)),'Correlated regressors never acquired cross covariance');
 console.log('Full-regressor covariance matches information-matrix reference; JS and compiled C/JS runners agree');
 const result=vm.runInContext(`(() => {
  const gains=[100,16,80],model={gain:1.05,lag:.04,delay:.012};
  const bins=AT_FREQ.att.map(hz=>{const T=atResponse(hz,gains,model),w=2*Math.PI*hz,amp=.05236;
   const samples=Array.from({length:1600},(_,k)=>{const t=k*.005;return[t,amp*w*(-T[1]*Math.sin(w*t)+T[0]*Math.cos(w*t))+.001*Math.sin(71*t)+.02+.001*t];});
   return atBin(samples,hz,amp,true);});
  const fit=atFit(bins,gains),recommendation=atRecommend(tuneDefaults(),'att',[fit,fit,fit]);
  const plantModel={gain:1.1,lag:.06,delay:.016,lead:.04};
  const plantFit=atFit(AT_FREQ.att.map(hz=>({hz,T:[1,0],plant:atPlant(hz,plantModel),quality:.99,samples:500})),gains);
  const noise=AT_FREQ.att.map(hz=>atBin(Array.from({length:1600},(_,k)=>[k*.005,.1*Math.sin(71*k*.005)]),hz,.05236,true));
  return JSON.stringify({fit,plantFit,recommendation,noiseRejected:atFit(noise,gains)===null});
 })()`,ctx);
 const r=JSON.parse(result);assert(r.fit && Math.abs(r.fit.lag-.04)<.012 && Math.abs(r.fit.delay-.012)<.006,'Measured model fit');
 assert(r.recommendation && r.recommendation.report.every(x=>x.safe),'No stable recommendation');assert(r.noiseRejected,'Noise accepted as a response');
 assert(r.plantFit && Math.abs(r.plantFit.lag-.06)<.001 && Math.abs(r.plantFit.delay-.016)<.001 && r.plantFit.lead===.04 && Math.abs(r.plantFit.gain-1.1)<.001,'Command/gyro plant fit with rotor-transient lead');
 console.log('Frequency analysis recovers gain/lag/delay, rejects poor signal and recommends gains with margin/headroom');
})().catch(e=>{console.error(e);process.exitCode=1;});
