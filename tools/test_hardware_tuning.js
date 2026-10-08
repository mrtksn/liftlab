'use strict';
const fs=require('fs'),vm=require('vm'),assert=require('assert'),cp=require('child_process');
const ctx=vm.createContext({Math,console});
for(const file of ['math.js','laws.js','tuning.js','autotune.js'])vm.runInContext(fs.readFileSync('js/'+file,'utf8'),ctx);
const {atBin,atFit,atResponse,atRecommend}=vm.runInContext('({atBin,atFit,atResponse,atRecommend})',ctx);
const model={gain:1.05,lag:.03,delay:.012,lead:.02},ig=[100,16,80],pg=[4,3.6,1];
function plant(hz){const w=2*Math.PI*hz,a=w*model.delay,G=[-model.gain*Math.cos(a)/(w*w),model.gain*Math.sin(a)/(w*w)],den=1+(w*model.lag)**2;const q=[(G[0]+G[1]*w*model.lag)/den,(G[1]-G[0]*w*model.lag)/den];return [q[0]-q[1]*w*model.lead,q[1]+q[0]*w*model.lead];}
const bins=[.8,1.6,3.2,6.4].map(hz=>({hz,quality:.99,samples:500,T:atResponse(hz,ig,model),plant:plant(hz)}));
const expectedAtt=atFit(bins,ig),inner={gains:ig,model:expectedAtt};
const pos=[.18,.32,.55].map(hz=>({hz,quality:.99,samples:500,T:atResponse(hz,pg,model,inner)}));
const expectedPos=atFit(pos,pg,inner);
const samples=Array.from({length:500},(_,k)=>{const t=k*.005+.0002*Math.sin(k);return [t,.13*Math.sin(2*Math.PI*1.6*t)+.07*Math.cos(2*Math.PI*1.6*t)+.4+.03*t+.001*Math.sin(17*t)];});
const bin=atBin(samples,1.6,.05),native=JSON.parse(cp.execFileSync(process.argv[2],{encoding:'utf8'}));
const tuning=JSON.parse(vm.runInContext('JSON.stringify(TUNE_DEFAULTS)',ctx)),flatten=t=>[...t.att.kR,...t.att.kW,...t.att.kI,t.pos.kp,t.pos.kd,t.pos.ki];
function compare(a,b,epsilon,label){assert.equal(a.length,b.length);a.forEach((n,i)=>assert(Math.abs(n-b[i])<epsilon,`${label}[${i}]: ${n} vs ${b[i]}`));}
compare(native.bin,[...bin.T,bin.quality],1e-9,'regression');
for(const [key,m] of [['attModel',expectedAtt],['posModel',expectedPos]])compare(native[key],[m.gain,m.lag,m.delay,m.lead,m.error],1e-9,key);
compare(native.att,flatten(atRecommend(tuning,'att',[expectedAtt,expectedAtt,expectedAtt]).tuning),2e-5,'attitude gains');
const tuned=atRecommend(tuning,'att',[expectedAtt,expectedAtt,expectedAtt]).tuning,verifiedInner={gains:[tuned.att.kR[0],tuned.att.kW[0],tuned.att.kI[0]],model:expectedAtt};
compare(native.pos,flatten(atRecommend(tuned,'pos',[expectedPos],verifiedInner).tuning),2e-5,'position gains');
console.log('Native/simulator timestamp regression, attitude/position fits, bounded recommendations and poor-signal rejection passed');
