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
 browser=await chromium.launch({executablePath:process.env.CHROME_PATH || '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const page=await browser.newPage({viewport:{width:1600,height:1000}}),errors=[];
 page.on('pageerror',e=>errors.push(e.stack));await page.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());
 await page.goto(process.env.LIVE_URL || 'http://127.0.0.1:'+server.address().port,{waitUntil:'networkidle'});await page.waitForFunction(()=>typeof fleet!=='undefined'&&fleet.ready&&brt.ready);
 await page.evaluate(()=>{running=false;setTerrain('open',1);});assert.strictEqual(await page.evaluate(()=>keyLayout),'handset');
 await page.locator('#tKeys').click();await page.locator('[data-keys="intuitive"]').click();
 assert.strictEqual(await page.locator('[data-keys="intuitive"]').getAttribute('aria-pressed'),'true');
 assert.deepStrictEqual(await page.locator('#keysMove kbd').allTextContents(),['↑','↓','A','D']);
 assert.deepStrictEqual(await page.locator('#keysAlt kbd').allTextContents(),['W','S']);assert.deepStrictEqual(await page.locator('#keysTurn kbd').allTextContents(),['←','→']);assert.strictEqual(await page.locator('#fwdKey').innerText(),'↑');
 if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-keys.png'});
 await page.locator('#tKeys').click();await page.evaluate(()=>document.activeElement.blur());
 const keys=[['w','up','z',1],['s','down','z',-1],['a','left','y',1],['d','right','y',-1],['ArrowUp','fwd','x',1],['ArrowDown','back','x',-1],['ArrowLeft','yawL','yaw',1],['ArrowRight','yawR','yaw',-1]];
 for(const [key,action,axis,sign] of keys){
   await page.evaluate(()=>{releaseAll();pilot.vref=[0,0,0];Object.assign(setpoint,{x:0,y:0,z:2,yaw:0});S.crashed=null;brt.superView=null;brt.navOut=null;});
   await page.keyboard.down(key);assert.deepStrictEqual(await page.evaluate(()=>[...pilot.held.keys()]),[action]);
   assert.strictEqual(await page.locator('[data-ctrl="'+action+'"]').getAttribute('aria-pressed'),'true');
   const radioBits=await page.evaluate(action=>({actual:groundInputs(0).held&255,expected:1<<GB[action]}),action);
   assert.strictEqual(radioBits.actual,radioBits.expected,key+' drove the wrong radio stick');
   // Also exercise the direct-target path used by drones without a running radio board.
   const moved=await page.evaluate(axis=>{const before=setpoint[axis],ready=brt.ready;brt.ready=false;try{pilotStep(.1);return setpoint[axis]-before;}finally{brt.ready=ready;}},axis);assert(moved*sign>0,key+' drove the wrong target axis');
   await page.keyboard.up(key);assert.strictEqual(await page.evaluate(()=>pilot.held.size),0);
 }
 // Both simultaneous hands and all remapping transitions release the old sources.
 await page.keyboard.down('w');await page.keyboard.down('ArrowUp');assert.deepStrictEqual((await page.evaluate(()=>[...pilot.held.keys()])).sort(),['fwd','up']);
 await page.evaluate(()=>setKeyLayout('game'));assert.strictEqual(await page.evaluate(()=>pilot.held.size),0);await page.keyboard.up('w');await page.keyboard.up('ArrowUp');
 for(const layout of ['game','intuitive','handset','intuitive','game','intuitive']){
   await page.evaluate(layout=>setKeyLayout(layout),layout);
   const expected=layout==='intuitive'?['up','down','left','right']:layout==='game'?['fwd','back','left','right']:['up','down','yawL','yawR'];
   const pads=await page.evaluate(()=>[...document.querySelectorAll('.pilot > .pad')].map(e=>({x:e.getBoundingClientRect().x,actions:[...e.querySelectorAll('[data-ctrl]')].map(b=>b.dataset.ctrl),label:e.getAttribute('aria-label')})).sort((a,b)=>a.x-b.x));
   assert.deepStrictEqual(pads[0].actions.sort(),expected.sort());
   for(const action of ['up','down','fwd','back','left','right','yawL','yawR']){
     const b=page.locator('[data-ctrl="'+action+'"]'),bounds=await b.boundingBox();await page.mouse.move(bounds.x+bounds.width/2,bounds.y+bounds.height/2);await page.mouse.down();
     assert.deepStrictEqual(await page.evaluate(()=>[...pilot.held.keys()]),[action],layout+' pad retained a stale handler');await page.mouse.up();assert.strictEqual(await page.evaluate(()=>pilot.held.size),0);
   }
 }
 // Field focus keeps normal typing and slider navigation; control focus still supports pad activation.
 await page.locator('#droneName').focus();await page.keyboard.press('w');assert.strictEqual(await page.evaluate(()=>pilot.held.size),0);
 await page.evaluate(()=>{openSet.add(actuators()[0].id);renderComps();const i=$('#compList input[type=range]');i.focus();});await page.keyboard.down('ArrowUp');assert.strictEqual(await page.evaluate(()=>pilot.held.size),0);await page.keyboard.up('ArrowUp');
 await page.locator('[data-ctrl="up"]').focus();await page.keyboard.down('Space');assert.deepStrictEqual(await page.evaluate(()=>[...pilot.held.keys()]),['up']);await page.keyboard.up('Space');assert.strictEqual(await page.evaluate(()=>pilot.held.size),0);
 await page.evaluate(()=>document.activeElement.blur());
 for(const width of [1600,390]){
   await page.setViewportSize({width,height:width===390?844:1000});await page.waitForTimeout(350);
   for(const layout of ['handset','game','intuitive']){
     await page.evaluate(layout=>setKeyLayout(layout),layout);
     const bounds=await page.evaluate(()=>[...document.querySelectorAll('.pilot > .pad')].map(e=>({x:e.getBoundingClientRect().x,y:e.getBoundingClientRect().y,actions:[...e.querySelectorAll('[data-ctrl]')].map(b=>b.dataset.ctrl)})).sort((a,b)=>a.x-b.x));
     const expected=layout==='intuitive'?['up','down','left','right']:layout==='game'?['fwd','back','left','right']:['up','down','yawL','yawR'];
     assert.deepStrictEqual(bounds[0].actions.sort(),expected.sort(),layout+' '+width+' left pad actions');
     assert(Math.abs(bounds[0].y-bounds[1].y)<1&&bounds[1].x>bounds[0].x,layout+' '+width+' pads must sit side by side');
   }
   const pads=await page.evaluate(()=>[...document.querySelectorAll('.pilot > .pad')].map(e=>({x:e.getBoundingClientRect().x,label:e.getAttribute('aria-label'),top:e.querySelector('.p-up').dataset.ctrl,bottom:e.querySelector('.p-down').dataset.ctrl,left:e.querySelector('.p-left').dataset.ctrl,right:e.querySelector('.p-right').dataset.ctrl})).sort((a,b)=>a.x-b.x));
   assert.deepStrictEqual(pads.map(p=>[p.top,p.bottom,p.left,p.right]),[['up','down','left','right'],['fwd','back','yawL','yawR']]);
   if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-'+width+'.png'});
 }
 await page.reload({waitUntil:'networkidle'});await page.waitForFunction(()=>typeof fleet!=='undefined'&&fleet.ready&&brt.ready);assert.strictEqual(await page.evaluate(()=>keyLayout),'intuitive');
 const saved=await page.evaluate(()=>localStorage.getItem('dfb-keys'));assert.strictEqual(saved,'intuitive');
 assert.deepStrictEqual(errors,[]);console.log('Intuitive keyboard target motion, both pads, layout switching, held-input release, focused controls, help/HUD, persistence and desktop/phone placement passed; no page errors');
}finally{if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
