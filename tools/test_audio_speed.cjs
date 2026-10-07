#!/usr/bin/env node
'use strict';
// Inspect actual Web Audio nodes and scheduling through the production speed controls.
const fs=require('fs'),path=require('path'),http=require('http'),assert=require('assert');
const {chromium}=require(process.env.PLAYWRIGHT_PATH || '/Users/mrtksn/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const root=path.resolve(__dirname,'..');
const server=http.createServer((req,res)=>{
 const url=new URL(req.url,'http://localhost'),file=path.resolve(root,'.'+decodeURIComponent(url.pathname==='/'?'/index.html':url.pathname));
 if(!file.startsWith(root+path.sep)){res.writeHead(403).end();return;}
 fs.readFile(file,(err,data)=>{if(err){res.writeHead(404).end();return;}
  res.setHeader('Cache-Control','no-store');res.setHeader('Content-Type',file.endsWith('.js')?'application/javascript':file.endsWith('.css')?'text/css':file.endsWith('.html')?'text/html':'application/octet-stream');res.end(data);});
});
(async()=>{await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));let browser;
try{
 browser=await chromium.launch({executablePath:process.env.CHROME_PATH || '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',headless:true,args:['--enable-gpu']});
 const page=await browser.newPage({viewport:{width:1600,height:1000}}),errors=[];
 page.on('pageerror',e=>errors.push(e.stack));await page.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());
 await page.goto(process.env.LIVE_URL || 'http://127.0.0.1:'+server.address().port,{waitUntil:'networkidle'});
 await page.waitForFunction(()=>fleet.ready&&brt.ready);
 await page.evaluate(()=>{running=false;fleetCreate('tiltquad');fleetCreate('tiltquad');fleetSelect(null);running=false;});
 await page.locator('#tSound').click();
 // Install recording around native methods: the real graph still receives every call.
 await page.evaluate(()=>{
  window.audioProbe={params:new Map(),nodes:[],originals:[],voices:null,baseline:null};const p=audioProbe;
  for(const name of ['setValueAtTime','setTargetAtTime','exponentialRampToValueAtTime']){
   const orig=AudioParam.prototype[name];p.originals.push(()=>AudioParam.prototype[name]=orig);
   AudioParam.prototype[name]=function(...args){let calls=p.params.get(this);if(!calls)p.params.set(this,calls=[]);calls.push({name,args});return orig.apply(this,args);};
  }
  for(const name of ['createOscillator','createBufferSource','createBiquadFilter','createGain']){
   const orig=snd.ctx[name];p.originals.push(()=>snd.ctx[name]=orig);
   snd.ctx[name]=function(...args){const node=orig.apply(this,args),record={name,node,start:[],stop:[]};p.nodes.push(record);
    if(node.start)for(const method of ['start','stop']){const fn=node[method];node[method]=function(...args){record[method].push(args);return fn.apply(this,args);};}return node;};
  }
  for(const d of fleet.drones)withDrone(d,()=>{
   S.t=20;S.crashed=null;S.batt.soc=1;S.p=[fleet.drones.indexOf(d)*3,0,2];cargo.power=true;
   for(const c of actuators()){act.set(c.id,{...act.get(c.id),Omega:600,i:10,T:2});hs.set(c.id,{...hs.get(c.id),prop:false});}
   for(const j of joints())jst.set(j.id,{...jst.get(j.id),rate:2});
  });
  sndResetScope();sndTick();p.voices=new Map([...snd.motors,...snd.servos]);
 });
 for(const rate of [1,.25,.5,1,2,4]){
  if(rate<=1)await page.locator('#speedSeg [data-speed="'+rate+'"]').click();else await page.evaluate(rate=>setSpeed(rate),rate);
  const result=await page.evaluate(rate=>{
   const p=audioProbe,check=(v,m)=>{if(!v)throw Error(m);},near=(a,b,m)=>check(Math.abs(a-b)<1e-5,m+': '+a+' != '+b);
   const target=param=>p.params.get(param).filter(c=>c.name==='setTargetAtTime').at(-1).args[0];
   p.params.clear();sndTick();check(speed===rate&&sndRate()===rate,'Speed control not applied');
   for(const [key,v] of snd.motors){
    check(p.voices.get(key)===v,'Speed restarted motor voice');near(target(v.hum.frequency),600/(2*Math.PI)*2*rate,'Blade pitch');
    near(target(v.whine.frequency),600/(2*Math.PI)*7*rate,'Motor pitch');near(target(v.bp.frequency),600/(2*Math.PI)*10*rate,'Wash filter');near(target(v.ns.playbackRate),rate,'Wash playback');
   }
   check(snd.servos.size===8,'Both moving-servo craft must be audible');
   for(const [key,v] of snd.servos){check(p.voices.get(key)===v,'Speed restarted servo');near(target(v.o.frequency),(120+260/3)*rate,'Servo pitch');near(target(v.f.frequency),(900+1400/3)*rate,'Servo filter');}
   const mix=[...snd.motors.values()].flatMap(v=>[target(v.hg.gain),target(v.wg.gain),target(v.ng.gain),target(v.out.pan)]).concat([...snd.servos.values()].flatMap(v=>[target(v.g.gain),target(v.out.pan)]));
   if(!p.baseline)p.baseline=mix;else mix.forEach((v,i)=>near(v,p.baseline[i],'Speed changed volume/pan'));
   // One-offs must stretch the entire envelope, pitch sweep, tail and onset delay.
   p.nodes=[];p.params.clear();const before=snd.ctx.currentTime;sndTone({f:1000,f2:500,dur:.2,at:.1});
   let source=p.nodes.find(n=>n.name==='createOscillator'),gain=p.nodes.find(n=>n.name==='createGain');const onset=source.start[0][0];
   check(onset>=before+.1/rate-.001&&onset<=snd.ctx.currentTime+.1/rate+.001,'Tone onset spacing');near(source.stop[0][0]-onset,.25/rate,'Tone duration/tail');
   let calls=p.params.get(source.node.frequency);near(calls[0].args[0],1000*rate,'Tone pitch');near(calls[1].args[0],500*rate,'Tone sweep');near(calls[1].args[1]-onset,.2/rate,'Tone sweep duration');
   calls=p.params.get(gain.node.gain);near(calls[1].args[1]-onset,.01/rate,'Tone attack');near(calls[2].args[1]-onset,.2/rate,'Tone envelope');
   p.nodes=[];p.params.clear();sndBurst({f:300,drop:.1,dur:.5,at:.02});source=p.nodes.find(n=>n.name==='createBufferSource');gain=p.nodes.find(n=>n.name==='createGain');
   const filter=p.nodes.find(n=>n.name==='createBiquadFilter'),start=source.start[0][0];near(source.node.playbackRate.value,rate,'Burst playback');near(source.stop[0][0]-start,.55/rate,'Burst duration/tail');
   calls=p.params.get(filter.node.frequency);near(calls[0].args[0],300*rate,'Burst filter');near(calls[1].args[0],40*rate,'Burst floor scales');near(calls[1].args[1]-start,.5/rate,'Burst sweep duration');
   calls=p.params.get(gain.node.gain);near(calls[1].args[1]-start,.004/rate,'Burst attack');near(calls[2].args[1]-start,.5/rate,'Burst envelope');
   p.nodes=[];sndArm();const beeps=p.nodes.filter(n=>n.name==='createOscillator');check(beeps.length===3,'Arming sequence');near(beeps[1].start[0][0]-beeps[0].start[0][0],.18/rate,'Arming spacing');
   p.nodes=[];sndLowBatt();const low=p.nodes.filter(n=>n.name==='createOscillator');near(low[1].start[0][0]-low[0].start[0][0],.22/rate,'Battery pair spacing');
   return rate+'x: native motor/servo/noise pitch, stable voices/mix, one-shot envelope and beeps';
  },rate);console.log(result);
 }
 console.log(await page.evaluate(()=>{
  const check=(v,m)=>{if(!v)throw Error(m);},tone=sndTone;let count=0;sndTone=()=>count++;
  try{
   setSpeed(.25);const a=fleet.drones[0],b=fleet.drones[1];sndSelectScope();
   for(const d of [a,b])withDrone(d,()=>{S.t=0;S.batt.soc=.1;brt.fcState=1;sndDroneTick(d.id,true,new Set(),new Set());});check(count===0,'Initial battery reminder');
   withDrone(a,()=>{S.t=2.99;sndDroneTick(a.id,true,new Set(),new Set());});check(count===0,'Wall clock used for battery cadence');
   withDrone(a,()=>{S.t=3.01;sndDroneTick(a.id,true,new Set(),new Set());});check(count===2,'Battery reminder missing after three simulation seconds');
   withDrone(b,()=>{S.t=1;sndDroneTick(b.id,true,new Set(),new Set());});check(count===2,'Battery clock shared across drones');
   withDrone(a,()=>{S.t=6.1;sndDroneTick(a.id,false,new Set(),new Set());});check(count===2,'Pause emitted beep');
   withDrone(a,()=>{S.t=0;sndDroneTick(a.id,true,new Set(),new Set());S.t=2.99;sndDroneTick(a.id,true,new Set(),new Set());});check(count===2,'Reset reminder clock not restarted');
   withDrone(a,()=>{S.t=3.01;sndDroneTick(a.id,true,new Set(),new Set());});check(count===4,'Reset suppressed future reminder');
   fleetSelect(a.id);sndTick();check(count===4,'Selection added alert');
  }finally{sndTone=tone;}
  // Physical telemetry must remain 1x even if a slow simulation rate was selected earlier.
  const on=live.on;try{live.on=true;check(sndRate()===1,'Real-device sound slowed');audioProbe.nodes=[];sndTone({f:1000});const n=audioProbe.nodes.find(n=>n.name==='createOscillator');check(audioProbe.params.get(n.node.frequency).at(-1).args[0]===1000,'Real-device effect pitch');}finally{live.on=on;}
  for(const restore of audioProbe.originals.reverse())restore();setSpeed(1);running=false;sndTick();
  return 'Independent simulation-clock reminders, pause/reset/silent selection and real-device 1x';
 }));
 await page.waitForTimeout(300);assert(await page.evaluate(()=>snd.master.gain.value<.01),'Pause did not silence output');
 await page.evaluate(()=>{setEditMode(true);running=true;sndTick();running=false;});await page.waitForTimeout(300);assert(await page.evaluate(()=>snd.master.gain.value<.01),'Edit did not silence output');
 await page.locator('#tSound').click();await page.waitForTimeout(300);assert(await page.evaluate(()=>!snd.on&&snd.ctx.state==='suspended'),'Mute did not suspend audio');
 assert.deepStrictEqual(errors,[]);console.log('Pause/edit/mute passed; no page errors');
}finally{if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
