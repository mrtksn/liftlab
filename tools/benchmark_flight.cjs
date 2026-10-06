#!/usr/bin/env node
// Isolated production-loop benchmark. --baseline serves tracked files from HEAD.
// Requires Playwright and Chrome; override paths as in test_flight_browser.cjs.
'use strict';
const fs=require('fs'),path=require('path'),http=require('http'),{spawnSync}=require('child_process');
const {chromium}=require(process.env.PLAYWRIGHT_PATH || '/Users/mrtksn/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const root=path.resolve(__dirname,'..'),baseline=process.argv.includes('--baseline'),cache=new Map();
const server=http.createServer((req,res)=>{
  const url=new URL(req.url,'http://localhost'),file=path.resolve(root,'.'+decodeURIComponent(url.pathname==='/'?'/index.html':url.pathname));
  if(!file.startsWith(root+path.sep)){res.writeHead(403).end();return;}
  try{let data=cache.get(file);if(!data){if(baseline){const git=spawnSync('git',['show','HEAD:'+path.relative(root,file)],{cwd:root,maxBuffer:20*1024*1024});if(git.status===0)data=git.stdout;}if(!data)data=fs.readFileSync(file);cache.set(file,data);}
    res.setHeader('Cache-Control','no-store');res.setHeader('Content-Type',file.endsWith('.js')?'application/javascript':file.endsWith('.css')?'text/css':file.endsWith('.html')?'text/html':'application/octet-stream');res.end(data);
  }catch(e){res.writeHead(404).end();}
});
(async()=>{await new Promise(r=>server.listen(0,'127.0.0.1',r));let browser;
try{
  browser=await chromium.launch({executablePath:process.env.CHROME_PATH || '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',headless:true,args:process.argv.includes('--hardware-gpu')?['--enable-gpu']:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
  const page=await browser.newPage({viewport:{width:1440,height:1000}}),errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());
  await page.addInitScript(()=>{
    const original=window.requestAnimationFrame;window.frameMeasurements=[];let last=0;
    window.requestAnimationFrame=function(callback){if(callback.name!=='frame')return original.call(window,callback);
      return original.call(window,now=>{const t=performance.now(),sim=typeof S==='undefined'?0:S.t;
        callback(now);const after=typeof S==='undefined'?0:S.t;
        if(last)window.frameMeasurements.push({dt:(now-last)/1000,cpu:performance.now()-t,sim:Math.max(0,after-sim)});last=now;
      });};
  });
  await page.goto('http://127.0.0.1:'+server.address().port,{waitUntil:'networkidle'});await page.waitForFunction(()=>brt.ready);
  const gpu=await page.evaluate(()=>{const gl=renderer.getContext(),debug=gl.getExtension('WEBGL_debug_renderer_info');return debug?gl.getParameter(debug.UNMASKED_RENDERER_WEBGL):gl.getParameter(gl.RENDERER);});console.log('WebGL renderer: '+gpu);
  const summary=values=>{const a=values.slice().sort((a,b)=>a-b);return {median:a[Math.floor(a.length/2)],p95:a[Math.floor(a.length*.95)],p99:a[Math.floor(a.length*.99)]};};
  const batches=await page.evaluate(async({baseline})=>{
    running=false;const results=[],summ=a=>{a.sort((a,b)=>a-b);return {median:a[Math.floor(a.length/2)],p95:a[Math.floor(a.length*.95)],p99:a[Math.floor(a.length*.99)]};};
    const keys=['quadx','hex','tiltquad','wingquad',...(!baseline?['quad-prop-table','overlap-octo']:[])];
    for(const key of keys){loadPreset(key==='quad-prop-table'||key==='overlap-octo'?'quadx':key);running=false;
      if(key==='quad-prop-table')for(const c of actuators()){
        const p=flightMotor(c);c.motorPhysics={kind:'brushless',kv:60/(2*Math.PI*p.Ke),resistance:p.R,inertia:p.J,currentLimit:p.iMax};
        c.propPhysics={radius:c.prop,referenceDensity:1.225,rows:Array.from({length:64},(_,i)=>{const o=p.Om*(i+1)/64*1.3;return[o*60/(2*Math.PI),p.kT*o*o,p.kQ*o*o];})};
      }
      if(key==='overlap-octo'){for(const c of actuators().slice()){const copy=structuredClone(c);copy.id=uid++;copy.name+=' lower';copy.pos[2]-=.16;copy.spin=-copy.spin;cfg.comps.push(copy);}envr.rotorSamples=5;}
      if(key==='quad-prop-table'||key==='overlap-octo'){recomputeProps();cPts=contactPoints();doReset();}
      while(!brt.ready)await new Promise(r=>setTimeout(r,20));
      for(let i=0;i<6000;i++){if(i%20===0)pilotStep(.01);physStep();}
      const values=[];for(let j=0;j<180;j++){const t=performance.now();pilotStep(34*PDT);for(let i=0;i<34;i++)physStep();values.push(performance.now()-t);if(j%30===0)await new Promise(r=>setTimeout(r,0));}
      results.push({key,batchMs:summ(values),motors:actuators().length,joints:joints().length,crashed:S.crashed,fcState:brt.fcState,altitude:S.p[2]});
    }return results;
  },{baseline});console.log(JSON.stringify({baseline,batches},null,2));
  const frames=[];
  for(const key of ['quadx','tiltquad'])for(const rate of [1,2,4]){
    await page.evaluate(({key,rate})=>{loadPreset(key);speed=rate;running=true;}, {key,rate});
    await page.waitForFunction(()=>brt.ready && S.t>2);await page.evaluate(()=>{window.frameMeasurements=[];});
    await page.waitForFunction(()=>window.frameMeasurements.length>=120,{},{timeout:40000});
    const data=await page.evaluate(()=>({records:window.frameMeasurements.slice(-120),crashed:S.crashed}));
    let wall=0,sim=0;for(const f of data.records){wall+=f.dt;sim+=f.sim;}
    const record={key,speed:rate,cpuMs:summary(data.records.map(x=>x.cpu)),intervalMs:summary(data.records.map(x=>x.dt*1000)),fps:data.records.length/wall,realtime:sim/wall,crashed:data.crashed};frames.push(record);console.log(JSON.stringify(record));
  }
  const out={baseline,gpu,errors,batches,frames};if(process.env.BENCH_OUTPUT)fs.writeFileSync(process.env.BENCH_OUTPUT,JSON.stringify(out,null,2));
  if(errors.length || batches.some(b=>b.crashed) || frames.some(f=>f.crashed))throw new Error('Flight/console failures: '+JSON.stringify(out));
}finally{if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
