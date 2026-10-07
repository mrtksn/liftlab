#!/usr/bin/env node
'use strict';
// Shared-world flight, runtime/UI isolation, selection, collisions and persistence.
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
 page.on('pageerror',e=>{errors.push(e.stack);console.error(e.stack);});
 await page.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());
 await page.goto(process.env.LIVE_URL || 'http://127.0.0.1:'+server.address().port,{waitUntil:'networkidle'});
 await page.waitForFunction(()=>fleet.ready && brt.ready);
 console.log(await page.evaluate(async()=>{
  running=false;setTerrain('open',1);const check=(v,m)=>{if(!v)throw Error(m);};
  const a=fleet.selected,config=JSON.stringify(cfg),time=S.t,board=brt.inst.get(boardOf('core').id);
  const b=fleetCreate('hex');check(b&&fleet.drones.length===2,'Add drone failed');
  check(JSON.stringify(a.state.cfg)===config&&a.state.S.t===time,'Adding replaced/reset the old drone');
  check(b.state.cfg.comps.filter(c=>c.type==='motor').length===6,'Added preset wrong');
  check(a.state.brt.inst.get(a.state.cfg.computers.boards.find(x=>x.tasks.includes('core')).id)===board,'Old board restarted');
  check(brt.inst.get(boardOf('core').id)!==board&&a.state.hs!==hs&&a.state.est!==est&&a.state.act!==act&&a.state.radio!==radio,'Runtime leaked across drones');
  cfg.frame.mass+=.2;recomputeProps();save();const bMass=cfg.frame.mass;undoStep();check(cfg.frame.mass!==bMass,'Selected undo failed');redoStep();check(cfg.frame.mass===bMass,'Selected redo failed');
  check(fleetSelect(a.id),'Select first failed');check(actuators().length===4&&cfg.frame.mass!==bMass,'Sidebar context wrong');
  const target=b.state.setpoint.x, x=document.getElementById('sp-x-n');x.value='0.4';x.dispatchEvent(new Event('input',{bubbles:true}));check(setpoint.x===0.4&&b.state.setpoint.x===target,'Target field retained another drone');
  const draftKey='positionControl';const draft=LAWS[draftKey].src+'\n// unsaved editor draft';lawCards.get(draftKey).ta.value=draft;HW_UI.drafts.set('test','owner A');
  const ca=threadNew();ca.title='A only';ca.msgs.push({role:'user',content:'one'});agent.cur=ca.id;const trigger={id:'same-id',kind:'crash'};const ta=threadForTrigger(trigger);
  fleetSelect(b.id);const tb=threadForTrigger(trigger);check(ta!==tb,'Trigger chat shared across drones');check(!HW_UI.drafts.has('test'),'Hardware draft leaked');
  const request=agentRequest,connected=agent.cfg.connected;let wrongTurn=false;agent.cfg.connected=true;agentRequest=async()=>{wrongTurn=true;throw Error('wrong drone');};await agentTurn('queued',{thread:ca});agentRequest=request;agent.cfg.connected=connected;check(!wrongTurn&&!agent.busy,'Queued AI turn ran on a different drone');
  fleetSelect(a.id);check(lawCards.get(draftKey).ta.value===draft&&HW_UI.drafts.get('test')==='owner A'&&agent.cur===ca.id,'Draft/chat selection lost');
  let failed=false;try{withDrone(b,()=>{throw Error('scope test');});}catch(e){failed=true;}check(failed&&fleet.active===a&&cfg===a.state.cfg,'Scope exception lost selected context');
  agent.busy=true;check(!fleetSelect(b.id),'AI context changed during a turn');agent.busy=false;fleet.pendingEdits=1;check(!fleetSelect(b.id),'Async design edit switched owner');fleet.pendingEdits=0;
  const node=document.getElementById('batt-capacity-n'),capacity=b.state.cfg.battery.capacity;node.value='2.1';node.dispatchEvent(new Event('input',{bubbles:true}));check(battCfg().capacity===2.1&&b.state.cfg.battery.capacity===capacity,'Battery edit leaked');
  radioCfg.kind='wifi';radioCfg.bind='alpha';radioReset();brt.sig=null;doReset();save();
  fleetSelect(b.id);radioCfg.kind='espnow';radioCfg.bind='bravo';radioReset();brt.sig=null;doReset();save();
  press('fwd','test');pokeStart('key:P');fleetSelect(a.id);check(!poke.src&&!b.state.pilot.held.size&&!pilot.held.size,'Held controls/poke migrated or stuck');
  const key='positionControl',src=LAWS[key].defSrc.replace('function positionControl','function positionControl');
  applyLaw(key,src+'\n// drone A only');fleetSelect(b.id);
  await new Promise(r=>setTimeout(r,100));check(LAWS[key].status==='default'&&a.state.LAWS[key].status==='edited','Formula/async timer migrated');
  check(a.state.RN!==RN&&a.state.RN.P!==RN.P,'Formula arenas shared');check(RN.act.kind==='wasm'&&a.state.RN.act.kind==='wasm'&&a.state.RN.module===RN.module,'WASM compilation/instance ownership failed');
  for(let n=0;n<12;n++){fleetStep(500);await new Promise(r=>setTimeout(r,0));}
  check(Math.abs(a.state.S.t-b.state.S.t)<1e-7&&a.state.S.t>2.9,'Drones not stepping in the same world');
  check(!a.state.S.crashed&&!b.state.S.crashed,'Normal flight crashed: '+a.state.S.crashed+' / '+b.state.S.crashed);
  check(a.state.radio.t>1&&b.state.radio.t>1&&a.state.gs!==b.state.gs,'Independent radios did not run');
  check(fleet.selected===b&&cfg===b.state.cfg,'Stepping changed selection');
  const before=a.state.S.t;doReset();check(a.state.S.t===before&&S.t===0,'Selected reset reset another drone');
  setEditMode(true);fleetSelect(a.id);check(editMode&&document.getElementById('hudTime').textContent.startsWith('mass '),'Edit selection HUD wrong');
  fleetSelect(b.id);setEditMode(false);running=false;fleetScene();renderer.render(scene,camera);
  return 'Independent configs, undo, battery, boards/WASM, sensors, radio, formulas/timers, controls and flight';
 }));
 // Select by real viewport tap, then verify a camera drag does not change selection.
 const pick=await page.evaluate(()=>{
  const a=fleet.drones[0],b=fleet.drones[1];cam.target.set(b.state.setpoint.x,b.state.setpoint.y,b.state.setpoint.z);cam.dist=5;cam.anim=null;view.follow=false;fleetScene();
  camera.lookAt(cam.target);renderer.render(scene,camera);const p=new THREE.Vector3(...a.state.S.p).project(camera),r=renderer.domElement.getBoundingClientRect();
  return {x:r.left+(p.x+1)*r.width/2,y:r.top+(1-p.y)*r.height/2,id:a.id};
 });
 await page.mouse.click(pick.x,pick.y);assert(await page.evaluate(id=>fleet.selected.id===id,pick.id),'Click did not select drone');
 const selected=await page.evaluate(()=>fleet.selected.id);await page.mouse.move(pick.x,pick.y);await page.mouse.down();await page.mouse.move(pick.x+35,pick.y+10);await page.mouse.up();assert(await page.evaluate(id=>fleet.selected.id===id,selected),'Orbit drag changed selection');
 // Drop-down and actual Add airframe UI.
 const other=await page.evaluate(()=>fleet.drones.find(d=>d!==fleet.selected).id);await page.selectOption('#droneSelect',other);assert(await page.evaluate(id=>fleet.selected.id===id,other),'Dropdown selection failed');
 await page.locator('#presetSlot .mb-btn').click();await page.getByRole('menuitem',{name:/Blank/}).click();
 assert(await page.evaluate(()=>fleet.drones.length===3&&editMode&&actuators().length===0),'Airframe menu did not add');
 console.log('Viewport tap, orbit drag, drone dropdown and Add airframe menu');
 console.log(await page.evaluate(()=>{
  const check=(v,m)=>{if(!v)throw Error(m);};running=false;setEditMode(false);running=false;
  // Bodies meet away from all props: test impulse momentum and collision detection.
  const [a,b]=fleet.drones;fleetSelect(a.id);loadPreset('quadx');withDrone(a,()=>{S.p=[0,0,3];S.q=[1,0,0,0];S.v=[1,0,0];S.w=[0,0,0];});
  fleetSelect(b.id);loadPreset('quadx');withDrone(b,()=>{S.p=[.105,0,3];S.q=[1,0,0,0];S.v=[-1,0,0];S.w=[0,0,0];});
  const momentum=()=>[a,b].reduce((v,d)=>add(v,scl(add(d.state.S.v,m3v(qmat(d.state.S.q),crs(d.state.S.w,d.state.truth.c))),d.state.truth.m)),[0,0,0]),before=momentum();fleetCollisions();
  check(a.contacts&&b.contacts,'Drone contacts absent');check(nrm(sub(momentum(),before))<1e-5,'Collision lost linear momentum');check(a.state.S.v[0]<1&&b.state.S.v[0]>-1,'Collision had no response');
  // Coincident rotating props produce damage on both crafts.
  withDrone(a,()=>{S.p=[0,0,3];for(const c of actuators())act.get(c.id).Omega=600;});
  withDrone(b,()=>{S.p=[0,0,3];for(const c of actuators())act.get(c.id).Omega=600;});fleetCollisions();
  check([...a.state.hs.values()].some(h=>h.prop)&&[...b.state.hs.values()].some(h=>h.prop),'Inter-drone prop strikes absent');
  const count=fleet.drones.length;fleetRemove(fleet.drones[2].id);check(fleet.drones.length===count-1,'Remove failed');
  fleetSelect(a.id);doReset();fleetSelect(b.id);doReset();running=false;
  fleet.selected.name='Test hex';fleetRenderSelector();fleetSave();
  return 'Component collision response/momentum, spinning-prop damage and drone removal';
 }));
 for(const theme of ['light','dark']){
  await page.setViewportSize({width:390,height:844});await page.evaluate(theme=>{document.documentElement.dataset.theme=theme;fleetScene();},theme);await page.waitForTimeout(200);
  assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1),'Phone overflow');
  if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-'+theme+'.png'});
 }
 const expected=await page.evaluate(()=>({ids:fleet.drones.map(d=>d.id),selected:fleet.selected.id,name:fleet.selected.name}));
 await page.reload({waitUntil:'networkidle'});await page.waitForFunction(()=>fleet.ready&&brt.ready);
 assert.deepStrictEqual(await page.evaluate(()=>({ids:fleet.drones.map(d=>d.id),selected:fleet.selected.id,name:fleet.selected.name})),expected,'Saved world not restored');
 await page.waitForFunction(()=>fleet.drones.every(d=>d.state.RN.act?.kind==='wasm'&&d.state.brt.ready));
 assert(await page.evaluate(()=>new Set(fleet.drones.map(d=>d.graphics.drone)).size===fleet.drones.length),'Render groups shared');
 await page.evaluate(()=>{running=false;view.follow=true;cam.dist=6;fleetSelect(fleet.drones[0].id);fleetScene();});
 await page.setViewportSize({width:1600,height:1000});await page.waitForTimeout(400);
 if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-desktop.png'});
 assert.deepStrictEqual(errors,[]);console.log('Saved-world reload, light/dark phone layout, separate render groups; no page errors');
}finally{if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
