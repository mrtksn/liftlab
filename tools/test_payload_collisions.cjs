#!/usr/bin/env node
'use strict';
// Exercise cable-ball geometry, dynamic contacts, sensor timing and shared flight in Chrome.
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
 console.log(await page.evaluate(()=>{
  const check=(v,m)=>{if(!v)throw Error(m);},close=(a,b,m)=>check(nrm(sub(a,b))<1e-8,m),zero=()=>[0,0,0];
  running=false;applyTerrain('open',1);loadPreset('quadx');running=false;
  const owner=fleet.selected;let part;
  withDrone(owner,()=>{part=mkHang('Test cable ball',0,0,-.06,{mass:.2,length:5});cfg.comps.push(part);recomputeProps();cPts=contactPoints();S.p=[-4,0,3];S.q=[1,0,0,0];S.v=zero();S.w=zero();});
  const ball={part,st:owner.state.pend.get(part.id),mass:part.mass,radius:payloadRad(part)};
  // True sphere contact, not a proxy box: corners must not create phantom hits.
  const box=fleetBox(zero(),qmat([1,0,0,0]),[2,2,2]);
  ball.st.p=[1+ball.radius*.5,0,0];let hit=fleetSphereBox(ball,box);check(hit&&hit.normal[0]===1,'Sphere/box face missing');
  ball.st.p=[1+ball.radius*.8,1+ball.radius*.8,0];check(!fleetSphereBox(ball,box),'Sphere corner has a false box contact');
  ball.st.p=zero();hit=fleetSphereBox(ball,box);check(hit&&hit.depth>0&&hit.normal.every(Number.isFinite),'Inside sphere failed');
  const R=qmat(matToQuat(incR(45))),rotated=fleetBox([0,0,2],R,[2,.4,.4]),edge=m3v(R,[1+ball.radius*.5,0,0]);ball.st.p=add([0,0,2],edge);check(fleetSphereBox(ball,rotated),'Rotated contact failed');
  // A cable can hit its own craft even in a single-drone world.
  withDrone(owner,()=>{S.p=[0,0,3];S.v=zero();S.w=zero();});ball.st.p=[.06+ball.radius*.8,0,3];ball.st.v=[-1,0,0];owner.contacts=0;const ownBefore=ball.st.v.slice();fleetCollisions();check(owner.contacts&&ball.st.v[0]>ownBefore[0]&&owner.state.S.v[0]<0,'Own-drone response missing');
  const target=fleetCreate('quadx');running=false;
  const setup=()=>{for(const [d,p] of [[owner,[-4,0,3]],[target,[0,0,3]]])withDrone(d,()=>{S.p=p;S.q=[1,0,0,0];S.v=zero();S.w=zero();S.crashed=null;resetHealth();for(const c of actuators())act.get(c.id).Omega=0;});};
  const linear=()=>fleet.drones.reduce((v,d)=>add(v,scl(add(d.state.S.v,m3v(qmat(d.state.S.q),crs(d.state.S.w,d.state.truth.c))),d.state.truth.m)),scl(ball.st.v,ball.mass));
  const angular=()=>fleet.drones.reduce((v,d)=>{const s=d.state,R=qmat(s.S.q),vc=add(s.S.v,m3v(R,crs(s.S.w,s.truth.c))),cm=add(s.S.p,m3v(R,s.truth.c));return add(v,add(crs(cm,scl(vc,s.truth.m)),m3v(R,m3v(s.truth.J,s.S.w))));},crs(ball.st.p,scl(ball.st.v,ball.mass)));
  setup();ball.st.p=[.06+ball.radius-.0005,.025,3];ball.st.v=[-1,.4,0];let before=linear(),spin=angular();const selection=fleet.selected;
  fleetCollisions();check(target.contacts&&target.state.S.v[0]<0&&ball.st.v[0]>-1,'Long-cable contact missed');close(linear(),before,'Linear momentum lost');close(angular(),spin,'Angular momentum lost');check(nrm(target.state.S.w)>0,'Off-centre impact lacked angular response');check(owner.state.S.p[0]===-4&&owner.state.S.v[0]===0,'Impulse wrongly applied to distant carrier');check(fleet.selected===selection&&cfg===target.state.cfg,'Collision changed active selection');
  // Deep overlaps and hard impacts remain bounded and finite.
  setup();ball.st.p=[0,0,3];ball.st.v=[-5,0,0];fleetCollisions();check([...ball.st.p,...ball.st.v,...target.state.S.v,...target.state.S.w].every(Number.isFinite),'Deep overlap became non-finite');
  setup();ball.st.p=[.06+ball.radius-.0005,0,3];ball.st.v=[-4,0,0];fleetCollisions();check(target.state.S.crashed,'Hard payload impact did not crash struck craft');
  // Spinning disks use sphere/disk overlap; stopped props remain intact.
  setup();const prop=fleetShapes(target).boxes.find(b=>b.part?.type==='motor').part,ro=withDrone(target,()=>rotorNow(prop)),centre=add(target.state.S.p,ro.p);
  ball.st.p=add(centre,[0,0,ball.radius*.5]);ball.st.v=zero();fleetCollisions();check(!target.state.hs.get(prop.id)?.prop,'Stopped prop damaged');
  setup();target.state.act.get(prop.id).Omega=600;ball.st.p=add(centre,[0,0,ball.radius*.5]);ball.st.v=zero();fleetCollisions();check(target.state.hs.get(prop.id)?.prop,'Cable ball passed through spinning prop');
  const disk={center:zero(),axis:[0,0,1],radius:.1};ball.st.p=[.1+ball.radius+.004,0,0];check(!fleetSphereProp(ball,disk),'Disk struck distant ball');ball.st.p=[.1+ball.radius*.5,0,0];check(fleetSphereProp(ball,disk),'Disk edge missed sphere');
  // The same ball stays dynamic after release, including a resting/sleeping ball.
  setup();owner.state.pend.delete(part.id);owner.state.cargo.off.add(part.id);
  const loose=withDrone(owner,()=>looseBody('Dropped ball',[{...part,pos:zero(),parent:null}],'hang'));loose.p=[.06+payloadRad(part)-.0005,0,3];loose.v=[-1,0,0];loose.asleep=true;loose.still=1;owner.state.cargo.loose.push(loose);target.contacts=0;fleetCollisions();check(target.contacts&&loose.v[0]>-1&&!loose.asleep&&loose.still===0&&loose.nearT===0,'Dropped ball collision/wake missing');
  withDrone(owner,()=>{loose.asleep=true;loose.box=null;check(cargoSolids(loose.p,1).some(b=>b.what==='Dropped ball'),'Legacy static cargo missing');check(!cargoSolids(loose.p,1,null,true).some(b=>b.what==='Dropped ball'),'Ball got duplicate static contact');});
  withDrone(owner,()=>resetSim());check(owner.state.pend.has(part.id)&&!owner.state.cargo.off.has(part.id)&&!owner.state.cargo.loose.some(l=>l.kind==='hang'),'Reset failed to restore cable payload');
  // A 1 kHz IMU integrates both 2 kHz contact ticks and the angular lever arm.
  withDrone(owner,()=>{
    S.q=[1,0,0,0];S.v=zero();S.w=zero();S.mb={K:mbKinematics([0,0,0,0,0,0]),acc:MB.bodies.map(()=>[0,0,0,0,0,0])};
    for(const st of act.values()){st.Omega=0;st.T=0;}
    const imu=allSensors().find(c=>c.kind==='imu');Object.assign(imu,{pos:[0,.1,0],mount:zero(),rate:1000,gyroNoise:0,gyroBias:0,gyroDrift:0,accNoise:0,accBias:0,scaleErr:0,misalign:0});
    const rt=sens.get(imu.id);rt.acc=0;rt.st={};rt.queue=[];
    for(let n=0;n<2;n++){S.contactImpulse={dv:[.001,0,0],dw:[0,0,.0005]};sampleSensors(PDT);}
    check(rt.queue.length===1,'Contact changed sensor rate');check(Math.abs(rt.queue[0].m.accel[0]-1.9)<1e-8,'IMU missed contact ticks/angular lever');
    S.contactImpulse=null;sampleSensors(PDT);sampleSensors(PDT);check(Math.abs(rt.queue[1].m.accel[0])<1e-8,'Contact impulse replayed');
  });
  return 'Sphere geometry, own/distant drone, linear/angular momentum, off-centre reaction, overlap/crash, prop strike, dropped/woken ball, reset and IMU contact averaging passed';
 }));
 await page.evaluate(()=>{
  running=false;fleetSelect(fleet.drones[0].id);loadPreset('cargo');
  const c=liveComps().find(c=>c.type==='hang'),st=pend.get(c.id),ball={radius:payloadR(c),st};
  if(st.p[2]<ball.radius || fleetShapesCurrent().boxes.some(box=>fleetSphereBox(ball,box)))throw Error('Cargo ball spawns inside airframe/ground');
  fleetSelect(fleet.drones[1].id);loadPreset('quadx');running=false;
  for(const [i,d] of fleet.drones.entries())withDrone(d,()=>{Object.assign(setpoint,{x:i?1:-1,y:0,z:1.5});resetSim();d.contacts=0;});
 });
 await page.waitForFunction(()=>fleet.drones.every(d=>d.state.brt.ready&&d.state.RN.act?.kind==='wasm'&&d.state.brt.gndOk));
 console.log(await page.evaluate(()=>{
  const finite=()=>{for(const d of fleet.drones)if(![...d.state.S.p,...d.state.S.v,...d.state.S.w,...[...d.state.pend.values()].flatMap(st=>[...st.p,...st.v])].every(Number.isFinite))throw Error('Non-finite payload dynamics');};
  // Cold-start stepping must not create the old ball trapped inside the battery.
  // The AI recorder must forward deferred sampling; every sensor advances once per physics tick.
  const owner=fleet.drones[0],target=fleet.drones[1];
  const imu=owner.state.cfg.comps.find(c=>c.kind==='imu'),rt=owner.state.sens.get(imu.id);rt.acc=0;rt.queue=[];
  fleetStep(1);if(Math.abs(rt.acc-PDT)>1e-10 || rt.queue.length)throw Error('Fleet/AI wrapper sampled sensors twice per tick');
  for(let i=0;i<700;i++)fleetStep(20);finite();
  if(fleet.drones.some(d=>d.state.S.p[2]<.5))throw Error('Cable/quad fleet failed to take off');
  if(fleet.drones.some(d=>d.contacts||d.state.S.crashed))throw Error('Spurious reset/startup impact');
  const part=owner.state.cfg.comps.find(c=>c.type==='hang'),ball=owner.state.pend.get(part.id);
  for(const [d,p] of [[owner,[-.4,0,3]],[target,[0,0,3]]])withDrone(d,()=>{S.p=p;S.v=[0,0,0];S.w=[0,0,0];S.q=[1,0,0,0];});
  // Near-side frame contact with a slack cable: neither distant carrier nor prop is the obstacle.
  ball.p=[-.06-payloadRad(part)+.001,0,3];ball.v=[1,0,0];target.contacts=0;
  fleetStep(4);finite();if(!target.contacts||target.state.S.v[0]<=0)throw Error('Integrated contact did not recoil the struck drone');
  for(let i=0;i<100;i++)fleetStep(20);finite();
  // Keep the cost fixture separated with its load below the carrier.
  for(const [d,p] of [[owner,[-1,0,3]],[target,[1,0,3]]])withDrone(d,()=>{S.p=p;S.v=[0,0,0];S.w=[0,0,0];S.q=[1,0,0,0];});
  ball.p=[-1,0,2.6];ball.v=[0,0,0];
  return 'Single sampling cadence, seven-second cargo/quad airborne flight without false startup impacts and one-second integrated contact passed';
 }));
 const timing=await page.evaluate(()=>{
  running=false;const enabled=fleetPayloadCollisions,results=[],summary=a=>{a.sort((a,b)=>a-b);return{median:a[Math.floor(a.length/2)],p95:a[Math.floor(a.length*.95)]};};
  for(const mode of ['without payload contacts','payload contacts','payload body contact']){
   fleetPayloadCollisions=mode==='without payload contacts'?()=>{}:enabled;
   const samples=[];for(let n=0;n<120;n++){const t=performance.now();for(let k=0;k<100;k++){if(mode==='payload body contact'){const target=fleet.drones[1],ball=[...fleet.drones[0].state.pend.values()][0],part=fleet.drones[0].state.cfg.comps.find(c=>c.type==='hang');target.state.S.p=[1,0,3];target.state.S.v=[0,0,0];target.state.S.w=[0,0,0];ball.p=[1-.06-payloadRad(part)+.005,0,3];ball.v=[1,0,0];}fleetCollisions();}samples.push((performance.now()-t)/100);}results.push({mode,tickMs:summary(samples)});
  }fleetPayloadCollisions=enabled;return results;
 });console.log('Collision microbenchmark (two separated craft, one cable): '+JSON.stringify(timing));
 assert.deepStrictEqual(errors,[]);console.log('No page errors');
}finally{if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
