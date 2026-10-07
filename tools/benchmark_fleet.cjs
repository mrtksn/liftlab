#!/usr/bin/env node
// Production-loop fleet benchmark: --payloads compares contact handling off/on in 1/2/4 loaded quads.
// Requires Playwright and Chrome; override paths as in test_flight_browser.cjs.
'use strict';
const fs=require('fs'),path=require('path'),http=require('http');
const {chromium}=require(process.env.PLAYWRIGHT_PATH || '/Users/mrtksn/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const root=path.resolve(__dirname,'..'),cache=new Map(),worldAudio=process.argv.includes('--world-audio'),payloads=process.argv.includes('--payloads');
const server=http.createServer((req,res)=>{
  const url=new URL(req.url,'http://localhost'),file=path.resolve(root,'.'+decodeURIComponent(url.pathname==='/'?'/index.html':url.pathname));
  if(!file.startsWith(root+path.sep)){res.writeHead(403).end();return;}
  try{let data=cache.get(file);if(!data){if(!data)data=fs.readFileSync(file);cache.set(file,data);}
    res.setHeader('Cache-Control','no-store');res.setHeader('Content-Type',file.endsWith('.js')?'application/javascript':file.endsWith('.css')?'text/css':file.endsWith('.html')?'text/html':'application/octet-stream');res.end(data);
  }catch(e){res.writeHead(404).end();}
});
(async()=>{await new Promise(r=>server.listen(0,'127.0.0.1',r));let browser;
try{
  browser=await chromium.launch({executablePath:process.env.CHROME_PATH || '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',headless:true,args:process.argv.includes('--hardware-gpu')?['--enable-gpu']:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
  const page=await browser.newPage({viewport:{width:1440,height:1000}}),errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());
  await page.addInitScript(()=>{
    const original=window.requestAnimationFrame;window.frameMeasurements=[];window.audioMeasurements=[];let last=0;
    window.requestAnimationFrame=function(callback){if(callback.name==='sndLoop')return original.call(window,now=>{const t=performance.now();callback(now);window.audioMeasurements.push(performance.now()-t);if(window.audioMeasurements.length>240)window.audioMeasurements.shift();});if(callback.name!=='frame')return original.call(window,callback);
      return original.call(window,now=>{const t=performance.now(),sim=typeof S==='undefined'?0:S.t;
        callback(now);const after=typeof S==='undefined'?0:S.t;
        if(last)window.frameMeasurements.push({dt:(now-last)/1000,cpu:performance.now()-t,sim:Math.max(0,after-sim)});last=now;
      });};
  });
  await page.goto('http://127.0.0.1:'+server.address().port,{waitUntil:'networkidle'});await page.waitForFunction(()=>brt.ready);await page.evaluate(()=>{window.payloadContactsOriginal=fleetHasPayloads;});
  const gpu=await page.evaluate(()=>{const gl=renderer.getContext(),debug=gl.getExtension('WEBGL_debug_renderer_info');return debug?gl.getParameter(debug.UNMASKED_RENDERER_WEBGL):gl.getParameter(gl.RENDERER);});console.log('WebGL renderer: '+gpu);
  const summary=values=>{const a=values.slice().sort((a,b)=>a-b);return {median:a[Math.floor(a.length/2)],p95:a[Math.floor(a.length*.95)],p99:a[Math.floor(a.length*.99)]};};
  const frames=[];
  for(const [key,count,sound] of (payloads ? [1,2,4].flatMap(n=>[['quadx',n,false],['quadx',n,true]]) : worldAudio ? [1,4,8].flatMap(n=>[['quadx',n,false],['quadx',n,true]]) : [['quadx',1],['quadx',2],['quadx',4],['quadx',8],['quadx',16],['tiltquad',2]])){
    await page.evaluate(({key,count,worldAudio,payloads,load})=>{
      running=false;fleetSelect(fleet.drones[0].id);setTerrain('open',1);setEditMode(false);
      while(fleet.drones.length>1)fleetRemove(fleet.drones[fleet.drones.length-1].id);
      loadPreset(key);setpoint.x=0;setpoint.y=0;doReset();
      for(let n=1;n<count;n++)fleetCreate(key);
      if(payloads){fleetHasPayloads=load?window.payloadContactsOriginal:()=>false;}
      if(payloads)for(const d of fleet.drones)withDrone(d,()=>{cfg.comps.push(mkHang('Benchmark load',0,0,-.06,{mass:.25,length:.35}));recomputeProps();resetSim();});
      if(worldAudio){fleetSelect(null);cam.target.set(0,0,1.5);cam.dist=8;}speed=1;running=true;
    },{key,count,worldAudio,payloads,load:sound});
    if(worldAudio && await page.evaluate(()=>snd.on)!==sound)await page.locator('#tSound').click();
    await page.waitForFunction(()=>fleet.drones.every(d=>d.state.brt.ready&&d.state.S.t>3),{},{timeout:60000});
    await page.evaluate(()=>{window.frameMeasurements=[];window.audioMeasurements=[];});
    await page.waitForFunction(()=>window.frameMeasurements.length>=240,{},{timeout:30000});
    const data=await page.evaluate(()=>({audio:window.audioMeasurements.slice(-240),voices:snd.motors.size,records:window.frameMeasurements.slice(-240),flight:flightPerf.read(),drones:fleet.drones.map(d=>({crashed:d.state.S.crashed,altitude:d.state.S.p[2],t:d.state.S.t,contacts:d.contacts||0}))}));
    let wall=0,sim=0;for(const f of data.records){wall+=f.dt;sim+=f.sim;}
    const record={key,count,...(payloads?{payloadContacts:sound}:{}),...(worldAudio?{sound,voices:data.voices,audioMs:summary(data.audio)}:{}),cpuMs:summary(data.records.map(x=>x.cpu)),intervalMs:summary(data.records.map(x=>x.dt*1000)),fps:data.records.length/wall,realtime:sim/wall,physics95:data.flight.physics95,drones:data.drones};frames.push(record);console.log(JSON.stringify(record));
  }
  const collisions=await page.evaluate(()=>{
    running=false;while(fleet.drones.length>1)fleetRemove(fleet.drones.at(-1).id);loadPreset('quadx');running=false;
    const result=[],q=a=>{a.sort((a,b)=>a-b);return {median:a[Math.floor(a.length/2)],p95:a[Math.floor(a.length*.95)]};};
    for(const count of [2,4,8]){
      while(fleet.drones.length<count)fleetCreate('quadx');
      const samples=[];
      for(let k=0;k<40;k++){
        for(const [i,d] of fleet.drones.entries())withDrone(d,()=>{S.p=[(i%2)*.1,Math.floor(i/2)*.06,3];S.v=[0,0,0];S.w=[0,0,0];S.q=[1,0,0,0];for(const c of actuators()){hsOf(c).prop=false;act.get(c.id).Omega=600;}});
        const t=performance.now();fleetCollisions();samples.push(performance.now()-t);
      }
      result.push({count,collisionTickMs:q(samples)});
    }return result;
  });console.log(JSON.stringify({collisions}));
  const out={gpu,errors,frames,collisions};if(process.env.BENCH_OUTPUT)fs.writeFileSync(process.env.BENCH_OUTPUT,JSON.stringify(out,null,2));
  if(payloads && frames.some(f=>f.drones.some(d=>d.altitude<.5)))throw Error('Loaded quad did not become airborne');
  if(errors.length||frames.some(f=>f.drones.some(d=>d.crashed)))throw Error('Fleet/console failure: '+JSON.stringify(out));
}finally{if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
