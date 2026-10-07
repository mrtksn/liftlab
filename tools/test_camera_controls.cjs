#!/usr/bin/env node
'use strict';
const fs=require('fs'),path=require('path'),http=require('http'),assert=require('assert');
const {chromium}=require(process.env.PLAYWRIGHT_PATH || '/Users/mrtksn/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const root=path.resolve(__dirname,'..');
const server=http.createServer((req,res)=>{
 const url=new URL(req.url,'http://localhost'),file=path.resolve(root,'.'+decodeURIComponent(url.pathname==='/'?'/index.html':url.pathname));
 if(!file.startsWith(root+path.sep)){res.writeHead(403).end();return;}
 fs.readFile(file,(err,data)=>{if(err){res.writeHead(404).end();return;}res.setHeader('Cache-Control','no-store');res.setHeader('Content-Type',file.endsWith('.js')?'application/javascript':file.endsWith('.css')?'text/css':file.endsWith('.html')?'text/html':'application/octet-stream');res.end(data);});
});
(async()=>{await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));let browser;
try{
 browser=await chromium.launch({executablePath:process.env.CHROME_PATH || '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',headless:true,args:['--enable-gpu']});
 const page=await browser.newPage({viewport:{width:1600,height:1000}}),errors=[];
 page.on('pageerror',e=>errors.push(e.stack));await page.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());
 await page.goto(process.env.LIVE_URL || 'http://127.0.0.1:'+server.address().port,{waitUntil:'networkidle'});await page.waitForFunction(()=>fleet.ready&&brt.ready);
 await page.evaluate(()=>{running=false;view.chase=false;view.follow=true;S.p=[0,0,2];Object.assign(setpoint,{x:0,y:0,z:2});cam.pan.set(0,0,0);cam.target.set(0,0,2);cam.az=-.8;cam.el=.55;cam.dist=5;fleetScene();renderer.render(scene,camera);});
 const empty=()=>page.evaluate(()=>{const r=vpEl.getBoundingClientRect();for(const y of [.7,.6,.8])for(const x of [.2,.8,.3,.7]){const p={x:r.left+x*r.width,y:r.top+y*r.height};if(!fleetHit({clientX:p.x,clientY:p.y})&&!pickHandle({clientX:p.x,clientY:p.y})&&document.elementFromPoint(p.x,p.y)===renderer.domElement)return p;}throw Error('No empty camera point');});
 const state=()=>page.evaluate(()=>{updateCamera();renderer.render(scene,camera);const p=new THREE.Vector3(0,0,2).project(camera);return {az:cam.az,el:cam.el,dist:cam.dist,target:cam.target.toArray(),pan:cam.pan.toArray(),selected:fleet.selected?.id,point:[(p.x+1)*vpEl.clientWidth/2,(1-p.y)*vpEl.clientHeight/2]};});
 for(const projection of ['persp','ortho'])for(const modifier of ['Control','Meta']){
  await page.evaluate(projection=>{centerCamera();setProjection(projection);fleetScene();renderer.render(scene,camera);},projection);
  const before=await state(),p=await empty();await page.keyboard.down(modifier);await page.mouse.move(p.x,p.y);await page.mouse.down();await page.mouse.move(p.x+40,p.y+20,{steps:5});await page.mouse.up();await page.keyboard.up(modifier);
  await page.waitForTimeout(250);const after=await state();
  assert.strictEqual(after.selected,before.selected,'Pan changed drone selection');assert.strictEqual(after.az,before.az,'Pan rotated azimuth');assert.strictEqual(after.el,before.el,'Pan rotated elevation');assert.strictEqual(after.dist,before.dist,'Pan zoomed');
  assert(Math.abs(after.point[0]-before.point[0]-40)<.5&&Math.abs(after.point[1]-before.point[1]-20)<.5,projection+' pan does not track the pointer');
  await page.waitForTimeout(250);assert.deepStrictEqual((await state()).target,after.target,'Follow snapped pan back');
  await page.keyboard.down(modifier);await page.mouse.click(p.x,p.y);await page.keyboard.up(modifier);assert.strictEqual((await state()).selected,before.selected,'Modifier click deselected drone');
 }
 let p=await empty(),before=await state();await page.mouse.move(p.x,p.y);await page.mouse.down();await page.mouse.move(p.x+35,p.y+20);await page.mouse.up();let after=await state();assert(after.az!==before.az&&after.el!==before.el,'Plain drag no longer orbits');
 await page.locator('#tShow').click();await page.locator('#centerView').click();assert(await page.evaluate(()=>cam.pan.length()===0),'Center view did not clear pan');await page.locator('#tShow').click();
 const second=await page.evaluate(()=>{const d=fleetCreate('quadx');running=false;withDrone(d,()=>{S.p=[4,0,2];});fleetSelect(null);cam.target.set(0,0,2);cam.az=-.8;cam.el=.55;cam.dist=5;fleetScene();renderer.render(scene,camera);return d.id;});
 p=await empty();before=await state();await page.keyboard.down('Control');await page.mouse.move(p.x,p.y);await page.mouse.down();await page.mouse.move(p.x+30,p.y);await page.mouse.up();await page.keyboard.up('Control');after=await state();assert.notDeepStrictEqual(after.target,before.target,'World pan did not move');assert.strictEqual(after.selected,undefined,'World pan selected a craft');
 await page.evaluate(()=>fleetScene());assert.deepStrictEqual((await state()).target,after.target,'World pan followed retained drone');
 await page.evaluate(()=>centerCamera());assert.deepStrictEqual((await state()).target,[2,0,2],'World center did not center fleet');
 await page.selectOption('#droneSelect',second);await page.evaluate(()=>{running=false;setProjection('persp');cam.target.set(...S.p);cam.pan.set(0,0,0);fleetScene();renderer.render(scene,camera);});
 // Dispatch simultaneous contacts through the production handlers, including lift/cancel transitions.
 console.log(await page.evaluate(()=>{
  const check=(v,m)=>{if(!v)throw Error(m);},r=vpEl.getBoundingClientRect(),capture=vpEl.setPointerCapture;
  const send=(type,id,x,y)=>vpEl.dispatchEvent(new PointerEvent(type,{pointerId:id,pointerType:'touch',button:0,clientX:r.left+x,clientY:r.top+y,bubbles:true}));
  vpEl.setPointerCapture=()=>{};
  try{
   const selected=fleet.selected,az=cam.az,el=cam.el,dist=cam.dist,target=cam.target.clone();
   send('pointerdown',51,100,350);send('pointerdown',52,200,350);send('pointermove',51,120,365);send('pointermove',52,220,365);
   check(cam.az===az&&cam.el===el,'Two fingers rotated');check(Math.abs(cam.dist-dist)<1e-9,'Two-finger translation zoomed');check(cam.target.distanceTo(target)>0,'Two fingers failed to pan');
   send('pointermove',52,270,365);check(cam.dist<dist,'Pinch did not zoom');send('pointerup',52,270,365);
   send('pointermove',51,150,380);check(cam.az===az&&cam.el===el,'Pinch tail rotated');send('pointerup',51,150,380);
   check(fleet.selected===selected&&ptrs.size===0&&!edit.down,'Pinch selected drone or left pointer state');
   send('pointerdown',53,100,350);send('pointermove',53,125,365);check(cam.az!==az,'One finger did not orbit');send('pointercancel',53,125,365);check(ptrs.size===0&&!edit.down,'Cancel retained camera gesture');
   send('pointerdown',54,100,350);send('lostpointercapture',54,100,350);check(ptrs.size===0&&!edit.down&&!fleetPointer,'Lost capture retained gesture');
   send('pointerdown',55,100,350);send('pointerdown',56,100,350);send('pointermove',56,100,350);check(Number.isFinite(cam.dist),'Coincident fingers produced invalid zoom');send('pointercancel',55,100,350);send('pointercancel',56,100,350);
  }finally{vpEl.setPointerCapture=capture;}
  return 'Touch orbit/pan/pinch, remaining-finger tail, cancel/lost capture and coincident contacts passed';
 }));
 await page.locator('#tEdit').click();await page.evaluate(()=>{selectComp(actuators()[0].id);fleetScene();renderer.render(scene,camera);});await page.waitForTimeout(300);
 const handle=await page.evaluate(()=>{fleetScene();renderer.render(scene,camera);const r=vpEl.getBoundingClientRect();for(const o of handleMeshes){let visible=true;for(let p=o;p;p=p.parent)if(!p.visible)visible=false;if(!visible)continue;const p=new THREE.Box3().setFromObject(o).getCenter(new THREE.Vector3()).project(camera),e={clientX:r.left+(p.x+1)*r.width/2,clientY:r.top+(1-p.y)*r.height/2};if(pickHandle(e)&&document.elementFromPoint(e.clientX,e.clientY)===renderer.domElement)return{x:e.clientX,y:e.clientY};}throw Error('No editor handle');});
 const design=await page.evaluate(()=>JSON.stringify(designSnap()));await page.keyboard.down('Control');await page.mouse.move(handle.x,handle.y);await page.mouse.down();assert(await page.evaluate(()=>!edit.drag),'Ctrl started part edit');await page.mouse.move(handle.x+30,handle.y+15);await page.mouse.up();await page.keyboard.up('Control');assert.strictEqual(await page.evaluate(()=>JSON.stringify(designSnap())),design,'Camera pan changed airframe');
 await page.evaluate(()=>{centerCamera();fleetScene();renderer.render(scene,camera);});await page.waitForTimeout(300);
 console.log(await page.evaluate(()=>{
  const check=(v,m)=>{if(!v)throw Error(m);},r=vpEl.getBoundingClientRect(),capture=vpEl.setPointerCapture;
  let point;for(const o of handleMeshes){const p=new THREE.Box3().setFromObject(o).getCenter(new THREE.Vector3()).project(camera),e={clientX:r.left+(p.x+1)*r.width/2,clientY:r.top+(1-p.y)*r.height/2};if(pickHandle(e)){point=e;break;}}
  check(point,'No handle after recenter');vpEl.setPointerCapture=()=>{};
  const send=(type,id,x=point.clientX,y=point.clientY)=>vpEl.dispatchEvent(new PointerEvent(type,{pointerId:id,pointerType:'touch',button:0,clientX:x,clientY:y,bubbles:true}));
  try{send('pointerdown',71);check(edit.drag?.pointerId===71,'Plain handle no longer edits');const snap=JSON.stringify(designSnap());send('pointerdown',72);send('pointermove',72,point.clientX+40,point.clientY+30);send('pointercancel',72);check(edit.drag?.pointerId===71&&JSON.stringify(designSnap())===snap,'Other touch moved or cancelled edited part');send('pointercancel',71);check(!edit.drag,'Handle cancel left drag active');}finally{vpEl.setPointerCapture=capture;}
  return 'Editor handle retains its owning touch and releases on cancel';
 }));
 // Actual mobile context retains gesture instructions and recenter access in the Show sheet.
 const mobile=await browser.newContext({viewport:{width:390,height:844},hasTouch:true,isMobile:true,deviceScaleFactor:2}),phone=await mobile.newPage();await phone.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());phone.on('pageerror',e=>errors.push(e.stack));
 await phone.goto(process.env.LIVE_URL || 'http://127.0.0.1:'+server.address().port,{waitUntil:'networkidle'});await phone.waitForFunction(()=>fleet.ready&&brt.ready);await phone.locator('#tShow').tap();assert(await phone.locator('#centerView').isVisible(),'Mobile has no recenter control');assert((await phone.locator('#showMenu').innerText()).includes('two fingers pan'),'Mobile gesture instructions missing');await phone.locator('#centerView').tap();
 assert(await phone.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1),'Mobile Show menu overflows');await phone.locator('#tShow').tap();
 const phonePoint=await phone.evaluate(()=>{running=false;cam.anim=null;view.chase=false;const r=vpEl.getBoundingClientRect();return{x:r.left+r.width*.3,y:r.top+r.height*.45,az:cam.az,el:cam.el,dist:cam.dist,target:cam.target.toArray(),selected:fleet.selected.id};});
 const cdp=await mobile.newCDPSession(phone),finger=(id,dx,dy=0)=>({id,x:phonePoint.x+dx,y:phonePoint.y+dy,radiusX:3,radiusY:3,force:1});
 await cdp.send('Input.dispatchTouchEvent',{type:'touchStart',touchPoints:[finger(1,0),finger(2,70)]});
 await cdp.send('Input.dispatchTouchEvent',{type:'touchMove',touchPoints:[finger(1,20,15),finger(2,90,15)]});
 await cdp.send('Input.dispatchTouchEvent',{type:'touchMove',touchPoints:[finger(1,10,15),finger(2,110,15)]});
 await cdp.send('Input.dispatchTouchEvent',{type:'touchEnd',touchPoints:[]});await phone.waitForTimeout(250);
 const phoneAfter=await phone.evaluate(()=>({az:cam.az,el:cam.el,dist:cam.dist,target:cam.target.toArray(),selected:fleet.selected.id,pointers:ptrs.size}));assert.strictEqual(phoneAfter.az,phonePoint.az,'Native mobile pan rotated');assert.strictEqual(phoneAfter.el,phonePoint.el,'Native mobile pan tipped');assert(phoneAfter.dist<phonePoint.dist,'Native mobile pinch failed');assert.notDeepStrictEqual(phoneAfter.target,phonePoint.target,'Native mobile pan failed');assert.strictEqual(phoneAfter.selected,phonePoint.selected,'Native mobile pan selected craft');assert.strictEqual(phoneAfter.pointers,0,'Native touch release retained pointers');await mobile.close();
 assert.deepStrictEqual(errors,[]);console.log('Ctrl/Command pan, orbit, perspective/orthographic pointer tracking, stable follow/world pan, recenter, editor protection and mobile Show passed');
}finally{if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
