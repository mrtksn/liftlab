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
 const ids=await page.evaluate(()=>{
  running=false;const a=fleet.selected,b=fleetCreate('quadx');running=false;
  withDrone(a,()=>{S.p=[-1,0,2];S.q=[1,0,0,0];});withDrone(b,()=>{S.p=[1,0,2];S.q=[1,0,0,0];});
  a.name='First craft';b.name='Second craft';fleetRenderSelector();fleetSave();return[a.id,b.id];
 });
 await page.selectOption('#droneSelect','');await page.waitForTimeout(250);
 assert(await page.evaluate(()=>!fleet.selected&&$('#airframe').hidden&&$('#telemetry').hidden&&!$('#worldPanel').hidden&&!$('#rtab-world')),'World selection/sidebar state wrong');
 assert(await page.evaluate(()=>$('.view').clientWidth>innerWidth*.9&&$('#resetBtn').disabled&&$('#tEdit').disabled),'World canvas/controls wrong');
 const before=await page.evaluate(()=>({targets:fleet.drones.map(d=>({...d.state.setpoint})),undo:undo.i}));
 for(const key of ['ArrowUp','Space','h','g','e','p','t','r','3'])await page.keyboard.press(key);
 await page.keyboard.press('Meta+z');await page.keyboard.press('Control+z');
 assert.deepStrictEqual(await page.evaluate(()=>({targets:fleet.drones.map(d=>({...d.state.setpoint})),undo:undo.i})),before,'World keyboard controlled retained drone');
 assert(await page.evaluate(()=>!editMode&&!poke.src&&fleet.drones.every(d=>!d.state.pilot.held.size)),'World controls left held state');
 await page.locator('#sp-wind-n').fill('2');await page.locator('#sp-wind-n').dispatchEvent('input');
 assert(await page.evaluate(()=>envr.wind===2&&JSON.parse(localStorage.getItem('liftlab-world-v1')).environment.wind===2),'World setting did not save');
 assert.deepStrictEqual(await page.evaluate(()=>fleet.drones.map(d=>({...d.state.setpoint}))),before.targets,'World settings changed drone targets');
 assert(await page.evaluate(i=>undo.i===i,before.undo),'World settings added retained-drone undo');
 await page.locator('#worldSettingsToggle').click();assert(await page.locator('#worldSettingsBody').isHidden(),'Collapse failed');
 await page.locator('#worldSettingsToggle').click();
 await page.evaluate(()=>{envr.wind=0;save();cam.target.set(0,0,2);cam.dist=4;cam.az=.7;cam.el=1.3;fleetScene();renderer.render(scene,camera);});await page.waitForTimeout(100);
 const dronePoint=async id=>page.evaluate(id=>{
  fleetScene();renderer.render(scene,camera);const r=vpEl.getBoundingClientRect(),d=fleet.drones.find(d=>d.id===id),candidates=[];
  d.graphics.drone.traverse(o=>{if(solidVisible(o)){const p=new THREE.Box3().setFromObject(o).getCenter(new THREE.Vector3()).project(camera);candidates.push({x:r.left+(p.x+1)*r.width/2,y:r.top+(1-p.y)*r.height/2});}});
  return candidates.find(p=>p.x>r.left&&p.x<r.right&&p.y>r.top&&p.y<r.bottom&&fleetHit({clientX:p.x,clientY:p.y})===d);
 },id);
 let point=await dronePoint(ids[0]);assert(point,'No visible clickable craft');await page.mouse.move(point.x,point.y);
 await page.waitForFunction(()=>!$('#droneHover').hidden&&$('#droneHover').textContent.includes('First craft'));
 assert(await page.evaluate(()=>fleetHoverOutline.visible&&vpEl.classList.contains('selectable')),'Hover outline/cursor missing');
 await page.mouse.click(point.x,point.y);await page.waitForTimeout(200);
 assert(await page.evaluate(id=>fleet.selected?.id===id&&!$('#airframe').hidden&&$('#worldPanel').hidden,ids[0]),'Hover craft click failed');
 assert(await page.locator('#targetFields').count()===1&&!await page.locator('#spFields').isVisible(),'Target/world ownership wrong');
 // Empty bottom-middle avoids HUD, controls and the settings overlay.
 const empty=async()=>page.evaluate(()=>{const r=vpEl.getBoundingClientRect();for(const y of [.55,.65,.45])for(const x of [.15,.85,.25,.75]){const p={x:r.left+x*r.width,y:r.top+y*r.height};if(!fleetHit({clientX:p.x,clientY:p.y})&&document.elementFromPoint(p.x,p.y)===renderer.domElement)return p;}throw Error('No empty viewport point');});
 let blank=await empty();await page.mouse.move(blank.x,blank.y);await page.waitForTimeout(100);assert(await page.locator('#droneHover').isHidden(),'Hover did not clear');
 // A drag that comes back to its original empty position must not deselect.
 await page.mouse.down();await page.mouse.move(blank.x+35,blank.y+10);await page.mouse.move(blank.x,blank.y);await page.mouse.up();
 assert(await page.evaluate(id=>fleet.selected?.id===id,ids[0]),'Return-to-origin orbit deselected');
 await page.mouse.click(blank.x,blank.y);await page.waitForTimeout(200);assert(await page.evaluate(()=>fleet.selected===null),'Empty click did not deselect');
 const target=await page.evaluate(()=>cam.target.toArray());await page.evaluate(()=>{fleet.drones[0].state.S.p[0]+=2;fleetScene();});
 assert.deepStrictEqual(await page.evaluate(()=>cam.target.toArray()),target,'World camera followed retained drone');
 await page.selectOption('#droneSelect',ids[1]);await page.waitForTimeout(200);
 // Pinch and cancellation retain selection.
 await page.evaluate(()=>{
  const r=vpEl.getBoundingClientRect(),send=(type,id,x,y)=>vpEl.dispatchEvent(new PointerEvent(type,{pointerId:id,pointerType:'touch',clientX:r.left+x,clientY:r.top+y,bubbles:true}));
  // Synthetic pointer capture is unavailable; let normal handlers ignore it while testing capture bookkeeping.
  const capture=vpEl.setPointerCapture;vpEl.setPointerCapture=()=>{};
  try{send('pointerdown',41,300,300);send('pointerdown',42,320,300);send('pointerup',42,320,300);send('pointerup',41,300,300);send('pointerdown',43,300,300);send('pointercancel',43,300,300);}finally{vpEl.setPointerCapture=capture;}
 });assert(await page.evaluate(id=>fleet.selected?.id===id,ids[1]),'Pinch/cancel deselected');
 // Leaving Edit saves/rebuilds the same drone and releases controls.
 await page.locator('#tEdit').click();await page.waitForTimeout(100);
 const handle=await page.evaluate(()=>{
  selectComp(actuators()[0].id);fleetScene();renderer.render(scene,camera);const r=vpEl.getBoundingClientRect();
  for(const o of handleMeshes){let visible=true;for(let p=o;p;p=p.parent)if(!p.visible)visible=false;if(!visible)continue;
    const p=new THREE.Box3().setFromObject(o).getCenter(new THREE.Vector3()).project(camera),e={clientX:r.left+(p.x+1)*r.width/2,clientY:r.top+(1-p.y)*r.height/2};
    if(pickHandle(e)&&document.elementFromPoint(e.clientX,e.clientY)===renderer.domElement)return {x:e.clientX,y:e.clientY};
  }return null;
 });assert(handle,'No clickable editor handle');await page.mouse.click(handle.x,handle.y);assert(await page.evaluate(id=>fleet.selected?.id===id&&editMode&&!edit.drag,ids[1]),'Editor handle click deselected');
 blank=await empty();await page.mouse.click(blank.x,blank.y);await page.waitForTimeout(200);assert(await page.evaluate(()=>!fleet.selected&&!editMode),'Empty Edit click did not enter world');
 await page.selectOption('#droneSelect',ids[1]);await page.locator('#tEdit').click();await page.locator('#worldView').click();await page.waitForTimeout(200);
 assert(await page.evaluate(()=>!fleet.selected&&!editMode&&!edit.drag),'Edit-to-world transition failed');
 const shared=await page.evaluate(async()=>{fleetSelect(fleet.drones[0].id);const code=await designCode('Shared craft');fleetSelect(null);return code;});const unchanged=await page.evaluate(()=>fleet.drones.map(d=>JSON.stringify(d.state.cfg)));
 await page.locator('#presetSlot .mb-btn').click();await page.getByRole('menuitem',{name:/Open a shared design/}).click();await page.locator('#shareIn').fill(shared);await page.locator('#shareOpen').click();await page.waitForFunction(()=>fleet.drones.length===3&&fleet.selected?.name==='Shared craft');assert.deepStrictEqual(await page.evaluate(()=>fleet.drones.slice(0,2).map(d=>JSON.stringify(d.state.cfg))),unchanged,'Shared design replaced a retained drone');await page.evaluate(()=>{fleetRemove();fleetSelect(null);});
 console.log('World/dropdown/empty clicks, hover/cursor, orbit/pinch/cancel, camera, hidden controls and environment ownership');
 // Web Audio uses actual node graphs; each drone has duplicate part IDs but independent voices/events.
 await page.locator('#tSound').click();
 console.log(await page.evaluate(()=>{
  const check=(v,m)=>{if(!v)throw Error(m);};running=false;for(const [i,d] of fleet.drones.entries())withDrone(d,()=>{S.p=[i*2,0,2];S.crashed=null;for(const c of actuators())hs.set(c.id,{...hs.get(c.id),prop:false});});running=true;sndResetScope();sndTick();
  const counts=()=>fleet.drones.map(d=>[...snd.motors.keys()].filter(k=>k.startsWith(d.id+':')).length);
  check(counts().every(n=>n===4)&&snd.motors.size===8,'World audio missed a craft or collided on part IDs');
  check(snd.events.size===2&&snd.events.get(fleet.drones[0].id)!==snd.events.get(fleet.drones[1].id),'Audio events shared');
  const previous=fleet.active;check(fleetSelect(fleet.drones[0].id),'Audio selection failed');sndTick();check(snd.motors.size===4&&counts()[1]===0,'Selected audio includes others');
  fleetSelect(fleet.drones[1].id);sndTick();check(snd.motors.size===4&&counts()[0]===0,'Audio did not switch craft');
  fleetSelect(null);sndTick();check(snd.motors.size===8&&fleet.active===fleet.drones[1],'Audio altered active context');
  const voices=[...snd.motors.values()];check(new Set(voices).size===8,'Voice objects shared');
  const burst=sndBurst,tone=sndTone;let crashes=0,strikes=0;sndBurst=o=>{if(o.dur===.5)crashes++;if(o.dur===.035)strikes++;};sndTone=()=>{};
  try{for(const d of fleet.drones)withDrone(d,()=>{S.crashed='test 2 m/s';hs.set(actuators()[0].id,{...hs.get(actuators()[0].id),prop:true});});sndTick();sndTick();check(crashes===2&&strikes===14,'Per-craft one-shot event detection wrong');}finally{sndBurst=burst;sndTone=tone;}
  running=false;sndTick();
  return 'Actual Web Audio graphs: all/selected scope, independent voices and crash/strike events; pause and mute';
 }));
 await page.waitForTimeout(300);assert(await page.evaluate(()=>snd.master.gain.value<.01),'Paused output is audible');await page.locator('#tSound').click();await page.waitForTimeout(300);assert(await page.evaluate(()=>!snd.on&&snd.ctx.state==='suspended'),'Mute did not suspend audio');
 await page.evaluate(()=>{fleetSelect(fleet.drones[0].id);doReset();fleetSelect(fleet.drones[1].id);doReset();running=false;fleetSelect(null);fleetStep(20);fleetScene();fleetSave();});
 const expected=await page.evaluate(()=>({ids:fleet.drones.map(d=>d.id),selected:fleet.selected,wind:envr.wind}));
 await page.locator('#worldSettingsToggle').click();
 await page.reload({waitUntil:'networkidle'});await page.waitForFunction(()=>fleet.ready&&fleet.drones.every(d=>d.state.brt.ready));
 await page.evaluate(()=>running=false);
 assert.deepStrictEqual(await page.evaluate(()=>({ids:fleet.drones.map(d=>d.id),selected:fleet.selected,wind:envr.wind})),expected,'Null selection did not persist');
 assert(await page.locator('#worldSettingsBody').isHidden(),'Collapsed world settings did not persist');
 await page.locator('#worldSettingsToggle').click();
 for(const theme of ['light','dark']){
  await page.setViewportSize({width:390,height:844});await page.evaluate(theme=>{document.documentElement.dataset.theme=theme;fleetTheme();},theme);await page.waitForTimeout(200);
  assert(await page.evaluate(()=>{const a=$('#worldPanel').getBoundingClientRect(),r=$('.view').getBoundingClientRect();return document.documentElement.scrollWidth<=innerWidth+1&&a.right<=r.right&&a.left>=r.left&&a.bottom<=r.bottom+1;}),'World overlay escapes phone viewport');
  if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-'+theme+'.png'});
 }
 await page.setViewportSize({width:1600,height:1000});await page.waitForTimeout(200);
 if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-desktop.png'});
 await page.locator('#presetSlot .mb-btn').click();await page.getByRole('menuitem',{name:/Blank/}).click();
 assert(await page.evaluate(()=>fleet.selected&&fleet.drones.length===3&&editMode&&!$('#airframe').hidden&&$('#worldPanel').hidden),'Add from world failed');
 const fixtures=await browser.newPage();await fixtures.goto('http://127.0.0.1:'+server.address().port+'/tools/test_ui_components.html');await fixtures.waitForFunction(()=>document.getElementById('testResults').textContent.includes('PASSED'));assert(await fixtures.evaluate(()=>!document.getElementById('testResults').textContent.includes('FAIL')),'UI component fixtures failed');await fixtures.close();
 assert.deepStrictEqual(errors,[]);console.log('World reload, physics/WASM, collapsed preference, add from world and phone themes; no page errors');
}finally{if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
