#!/usr/bin/env node
'use strict';
// Ground grouping, live radio cards, scoped settings, peers and stable log controls.
const fs=require('fs'),path=require('path'),http=require('http'),assert=require('assert');
const {chromium}=require(process.env.PLAYWRIGHT_PATH||'/Users/mrtksn/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const root=path.resolve(__dirname,'..'),server=http.createServer((req,res)=>{
 const file=path.resolve(root,'.'+decodeURIComponent(new URL(req.url,'http://localhost').pathname.replace(/^\/$/,'/index.html')));
 if(!file.startsWith(root+path.sep)){res.writeHead(403).end();return;}
 fs.readFile(file,(e,b)=>{if(e){res.writeHead(404).end();return;}res.setHeader('Cache-Control','no-store');res.setHeader('Content-Type',file.endsWith('.js')?'application/javascript':file.endsWith('.css')?'text/css':file.endsWith('.html')?'text/html':'application/octet-stream');res.end(b);});
});
(async()=>{await new Promise(r=>server.listen(0,'127.0.0.1',r));let browser;
try{
 browser=await chromium.launch({executablePath:process.env.CHROME_PATH||'/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const page=await browser.newPage({viewport:{width:1700,height:1100}}),errors=[];
 await page.emulateMedia({reducedMotion:'reduce'});page.on('pageerror',e=>{errors.push(e.stack);console.error(e.stack);});await page.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());
 await page.goto(process.env.LIVE_URL||'http://127.0.0.1:'+server.address().port,{waitUntil:'networkidle'});await page.waitForFunction(()=>typeof fleet!=='undefined'&&fleet.ready&&brt.ready);
 await page.evaluate(()=>{running=false;setTerrain('open',1);UI_PANELS.editor.select('gs');renderGs(true);});
 const pick=async(selector,value)=>{await page.locator(selector).dispatchEvent('pointerdown');await page.locator(selector).selectOption(String(value));};
 const close=async()=>{await page.keyboard.press('Escape');await page.waitForFunction(()=>!document.querySelector('dialog[open]'));};
 assert.deepStrictEqual(await page.locator('#paneGs > .sec > h2').evaluateAll(nodes=>nodes.map(n=>{const c=n.cloneNode(true);c.querySelectorAll('.srcs,.src-tag').forEach(e=>e.remove());return c.textContent;})),['Real drone','Telemetryreceived from the drone','Radio link','Other dronesthe drone\'s own link to them','Cargothe latches','Link loginside the simulated radio','Telemetry messages']);
 assert.strictEqual(await page.locator('#gsTelemetrySec canvas').count(),3);
 assert.strictEqual(await page.locator('#paneGs input,#paneGs select,#paneGs textarea').count(),0);
 assert.strictEqual(await page.locator('#gsRadioCard .w-bar').count(),2);
 await page.evaluate(()=>{const a=fleet.selected;fleetCreate('quadx');fleetSelect(a.id);renderGs(true);return a.id;});
 await page.locator('#gsRadioCard').focus();await page.keyboard.press('Enter');assert(await page.locator('#gsRadioDlg').evaluate(d=>d.open));
 const owner=await page.evaluate(()=>fleet.selected.id);
 await page.evaluate(()=>{const other=fleet.drones.find(d=>d.id!==fleet.selected.id);if(fleetSelect(other.id))throw Error('settings allowed drone switch');});
 assert.strictEqual(await page.evaluate(()=>fleet.selected.id),owner);
 await pick('#gs-kind','espnow');await page.waitForFunction(()=>radioCfg.kind==='espnow');
 await pick('#gs-channel','6');assert.strictEqual(await page.evaluate(()=>radioCfg.channel),6);
 await page.locator('#gs-bind').fill('ground cards');await page.locator('#gs-bind').press('Enter');assert.strictEqual(await page.evaluate(()=>radioCfg.bind),'ground cards');
 await page.locator('#gsExtra').fill('17');await page.locator('#gsExtra').dispatchEvent('change');assert.strictEqual(await page.evaluate(()=>radioCfg.extra),17);
 await pick('#gs-kind2','serial');await pick('#gs2-medium','0');assert.strictEqual(await page.evaluate(()=>radioCfg2.medium),0);
 await page.locator('#gs-bind').fill('pending draft');await page.locator('#gs-bind').evaluate(n=>n.setSelectionRange(3,5));
 await page.evaluate(()=>{doReset();renderGs(true);});assert.strictEqual(await page.locator('#gs-bind').inputValue(),'pending draft');assert.deepStrictEqual(await page.locator('#gs-bind').evaluate(n=>[n.selectionStart,n.selectionEnd]),[3,5]);
 await page.locator('#gs-bind').fill('ground cards');await page.locator('#gs-bind').press('Enter');
 await close();await page.evaluate(()=>renderGs(true));assert((await page.locator('#gsRadioCard').innerText()).includes('17 dB extra loss'));
 await page.locator('#gsPeerSettings').click();assert(await page.locator('#gsPeerDlg').evaluate(d=>d.open));
 await page.locator('#gsFleet').fill('test fleet');await page.locator('#gsFleet').press('Enter');assert.strictEqual(await page.evaluate(()=>radioCfg.fleet),'test fleet');
 assert(await page.locator('#gsPeerCh').isDisabled());assert.strictEqual(await page.locator('#gsPeerCh').inputValue(),'6');
 await close();await page.locator('#gsPeers').click();assert.strictEqual(await page.locator('#gsPeers').getAttribute('aria-pressed'),'false');
 await page.locator('#gsPeers').click();assert.strictEqual(await page.locator('#gsPeers').getAttribute('aria-pressed'),'true');
 // Distinct configurations must refresh on selection and persist through reload.
 const other=await page.evaluate(()=>{const d=fleet.drones.find(d=>d.id!==fleet.selected.id);fleetSelect(d.id);UI_PANELS.editor.select('gs');renderGs(true);return d.id;});
 await page.locator('#gsRadioCard').click();assert.strictEqual(await page.locator('#gsExtra').inputValue(),'0');await pick('#gs-kind','wifi');await close();
 await page.evaluate(id=>{fleetSelect(id);renderGs(true);},owner);assert((await page.locator('#gsRadioCard').innerText()).includes('ESP-NOW'));
 await page.locator('#gsPeerSettings').click();assert.strictEqual(await page.locator('#gsFleet').inputValue(),'test fleet');await close();
 await page.reload({waitUntil:'networkidle'});await page.waitForFunction(()=>fleet.ready&&brt.ready);await page.evaluate(()=>{running=false;UI_PANELS.editor.select('gs');renderGs(true);});
 assert.strictEqual(await page.evaluate(()=>radioCfg.extra),17);assert.strictEqual(await page.evaluate(()=>radioCfg2.kind),'serial');
 console.log('Grouping, native keyboard/dialog ownership, both link settings, peer controls and per-drone reload passed');
 // Exercise real transport and telemetry, then loss/staleness without fabricated readings.
 await page.locator('#gsRadioCard').click();await pick('#gs-kind2','');await pick('#gs-kind','elrs');await page.locator('#gsExtra').fill('0');await page.locator('#gsExtra').dispatchEvent('change');await close();
 await page.evaluate(async()=>{
  radioCfg.fleet='liftlab';radioCfg.peers=1;peerSetup();
  const d=fleet.drones.find(d=>d.id!==fleet.selected.id);withDrone(d,()=>{radioCfg.fleet='liftlab';radioCfg.peers=1;peerSetup();});
  for(let n=0;n<20;n++){fleetStep(500);await new Promise(r=>setTimeout(r,0));}renderGs(true);
 });
 assert((await page.locator('#gsRadioCard').innerText()).includes('Receiving telemetry'));
 assert.strictEqual(await page.locator('#gsRadioCard .w-stale').count(),0);
 assert.strictEqual(await page.locator('#gsTelemetrySec .w-num').filter({hasText:/^—$/}).count(),0);
 assert.strictEqual(await page.locator('#paneGs .gs-peer').count(),1);
 await page.locator('.gs-peer-ping').click();await page.evaluate(async()=>{for(let n=0;n<3;n++){fleetStep(500);await new Promise(r=>setTimeout(r,0));}renderGs(true);});
 assert((await page.locator('.gs-peer-info').innerText()).includes('ping'));
 assert(await page.locator('#gsFleetGo').isEnabled());await page.locator('#gsFleetGo').click();
 await page.evaluate(async()=>{for(let n=0;n<6;n++){fleetStep(500);await new Promise(r=>setTimeout(r,0));}renderGs(true);});
 assert.strictEqual(await page.locator('#gsFleetGo').getAttribute('aria-pressed'),'true');await page.locator('#gsFleetGo').click();
 await page.evaluate(async()=>{for(let n=0;n<6;n++){fleetStep(500);await new Promise(r=>setTimeout(r,0));}renderGs(true);});assert.strictEqual(await page.locator('#gsFleetGo').getAttribute('aria-pressed'),'false');
 // Pause holds rows/bytes while the link continues; filters and clear remain native controls.
 await page.locator('.gs-pause').click();const paused=await page.locator('.gs-linklog').innerText();
 await page.evaluate(()=>{for(let n=0;n<3;n++)fleetStep(500);renderGs(true);});assert.strictEqual(await page.locator('.gs-linklog').innerText(),paused);
 const raw=page.locator('.gs-linklog button.gs-line').first();if(await raw.count()){await raw.click();assert(await page.locator('.gs-raw').first().isVisible());}
 await page.getByRole('button',{name:'Resume',exact:true}).click();await page.getByRole('button',{name:'Clear',exact:true}).click();assert((await page.locator('.gs-linklog').innerText()).includes('Cleared'));
 await page.getByRole('button',{name:'Every frame',exact:true}).click();assert.strictEqual(await page.getByRole('button',{name:'Every frame',exact:true}).getAttribute('aria-pressed'),'true');
 await page.evaluate(async()=>{radioCfg.extra=120;save();for(let n=0;n<28;n++){fleetStep(500);await new Promise(r=>setTimeout(r,0));}renderGs(true);});
 assert.deepStrictEqual(await page.locator('#gsRadioCard .w-num').allTextContents(),['0%','0%']);assert((await page.locator('#gsRadioCard').innerText()).includes('Telemetry stale'));assert((await page.locator('#gsTelemetrySec .w-num').allTextContents()).includes('—'));
 console.log('Actual telemetry/peer ping/fleet engagement, log pause/raw/resume/clear/filter and lost-link staleness passed');
 // Reset while a detail is open: the same native dialog survives and restores keyboard focus.
 await page.locator('#gsRadioCard').click();await page.locator('#gs-rate').focus();
 await page.evaluate(()=>{doReset();renderGs(true);});assert.strictEqual(await page.locator('#gsRadioDlg[open]').count(),1);assert.strictEqual(await page.locator('#gsRadioDlg').count(),1);assert.strictEqual(await page.evaluate(()=>document.activeElement.id),'gs-rate');await close();
 await page.evaluate(async()=>{radioCfg.extra=0;save();for(let n=0;n<20;n++){fleetStep(500);await new Promise(r=>setTimeout(r,0));}renderGs(true);$('#airframe').scrollTop=0;});
 for(const width of [1700,390])for(const theme of ['light','dark']){
  await page.setViewportSize({width,height:1100});await page.evaluate(t=>{document.documentElement.dataset.theme=t;},theme);await page.evaluate(()=>renderGs(true));
  assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));
  await page.locator('#gsTelemetrySec').scrollIntoViewIfNeeded();
  if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-'+width+'-'+theme+'.png',fullPage:true});
  for(const trigger of ['#gsRadioCard','#gsPeerSettings']){await page.locator(trigger).click();
   if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-'+trigger.slice(1)+'-'+width+'-'+theme+'.png'});
   assert(await page.evaluate(()=>{const d=document.querySelector('dialog[open]'),r=d.getBoundingClientRect();return r.left>=0&&r.right<=innerWidth+1&&d.scrollWidth<=d.clientWidth+1;}));await close();}
 }
 // No telemetry task: summary and instruments must show absence, with settings still inspectable.
 await page.evaluate(()=>{cfg.computers.boards.forEach(b=>b.tasks=b.tasks.filter(t=>t!=='tlm'));structural();doReset();renderGs(true);});
 assert((await page.locator('#gsRadioCard').innerText()).includes('No radio assigned'));
 assert.deepStrictEqual(await page.locator('#gsRadioCard .w-num').allTextContents(),['—','—']);
 await page.locator('#gsRadioCard').click();assert.strictEqual(await page.locator('#gsRadioDlg[open]').count(),1);await close();
 assert.deepStrictEqual(errors,[]);console.log('Reset/dialog lifetime, desktop/phone light/dark layouts, no page errors passed');
}finally{if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
