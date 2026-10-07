#!/usr/bin/env node
'use strict';
// Compact Computers inventory, scoped native dialogs, assignments, hardware and formula workflows.
const fs=require('fs'),path=require('path'),http=require('http'),assert=require('assert');
const {chromium}=require(process.env.PLAYWRIGHT_PATH || '/Users/mrtksn/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const root=path.resolve(__dirname,'..'),server=http.createServer((req,res)=>{
 const file=path.resolve(root,'.'+decodeURIComponent(new URL(req.url,'http://localhost').pathname.replace(/^\/$/,'/index.html')));
 if(!file.startsWith(root+path.sep)){res.writeHead(403).end();return;}
 fs.readFile(file,(e,b)=>{if(e){res.writeHead(404).end();return;}res.setHeader('Cache-Control','no-store');res.setHeader('Content-Type',file.endsWith('.js')?'application/javascript':file.endsWith('.css')?'text/css':file.endsWith('.html')?'text/html':'application/octet-stream');res.end(b);});
});
(async()=>{await new Promise(r=>server.listen(0,'127.0.0.1',r));let browser;
try{
 browser=await chromium.launch({executablePath:process.env.CHROME_PATH || '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const page=await browser.newPage({viewport:{width:1700,height:1100}}),errors=[];
 page.on('pageerror',e=>{errors.push(e.stack);console.error(e.stack);});await page.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());
 await page.goto(process.env.LIVE_URL||'http://127.0.0.1:'+server.address().port,{waitUntil:'networkidle'});
 await page.waitForFunction(()=>typeof fleet!=='undefined'&&fleet.ready&&brt.ready);
 await page.evaluate(()=>{running=false;setTerrain('open',1);UI_PANELS.editor.select('form');});
 const pick=async(selector,value)=>{await page.locator(selector).dispatchEvent('pointerdown');await page.locator(selector).selectOption(String(value));};
 const close=async()=>{await page.keyboard.press('Escape');await page.waitForFunction(()=>!document.querySelector('dialog[open]'));};
 const duty=id=>page.locator('#taskRows [data-task="'+id+'"]');
 const board=id=>page.locator('#boardList [data-board="'+id+'"]');
 const device=id=>page.locator('#paneForm > .sec [data-device="'+id+'"]');
 const core=await page.evaluate(()=>boardOf('core').id),pi=await page.evaluate(()=>boardOf('nav').id);
 const motor=await page.evaluate(()=>actuators()[0].id),imu=await page.evaluate(()=>sensorsOf('imu')[0].id),mag=await page.evaluate(()=>sensorsOf('mag')[0].id);
 assert.strictEqual(await page.locator('#boardList > .computer-card').count(),3);
 assert.strictEqual(await page.locator('#paneForm > .sec input,#paneForm > .sec select,#paneForm > .sec textarea').count(),0);
 for(const id of [core,pi,'ground'])assert.strictEqual(await board(id).locator('button').count(),1);
 assert.strictEqual(await duty('cargo').locator('button').innerText(),'Assign');assert.strictEqual(await duty('core').locator('button').count(),0);
 assert((await board(pi).innerText()).includes('Linux computer, 1 core at 1 GHz'));
 await page.waitForTimeout(200);
 if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-desktop.png'});
 await board(core).focus();await page.keyboard.press('Enter');assert(await page.locator('#bname-'+core).isVisible());await close();assert.strictEqual(await page.evaluate(()=>document.activeElement.dataset.board),String(core));
 await page.locator('#boardAdd').click();await close();assert.strictEqual(await page.evaluate(()=>document.activeElement.id),'boardAdd');
 await page.locator('#boardAdd').click();await page.locator('[data-board-kind="s3"]').click();const extra=await page.evaluate(()=>computers().boards.at(-1).id);
 await board(extra).click();assert(await page.locator('#computerDlg').isVisible());await page.locator('#bname-'+extra).fill('Auxiliary');await page.locator('#bname-'+extra).press('Enter');
 await page.locator('#bdel-'+extra).click();assert(await page.locator('#boardDeleteConfirm').isVisible());await page.locator('.computer-confirm .btn').first().click();assert.strictEqual(await page.evaluate(()=>computers().boards.length),3);
 await page.locator('#bdel-'+extra).click();await page.locator('#boardDeleteConfirm').click();assert.strictEqual(await page.evaluate(()=>computers().boards.length),2);
 await page.locator('#undoBtn').click();assert(await board(extra).isVisible());assert((await board(extra).innerText()).includes('Auxiliary'));
 // Duties list only compatible boards; removal stays empty through normalization and undo.
 await duty('learn').click();assert.strictEqual(await page.locator('#computerDlg [data-assign-board]').count(),1);await page.locator('[data-remove-assignment]').click();await close();assert(await duty('learn').locator('button').isVisible());
 await duty('learn').locator('button').click();await page.locator('[data-assign-board="'+pi+'"]').click();await close();assert.strictEqual(await page.evaluate(()=>boardOf('learn').id),pi);
 await duty('core').click();assert.strictEqual(await page.locator('#computerDlg [data-assign-board]').count(),2);await page.locator('[data-remove-assignment]').click();
 assert(await page.evaluate(()=>!boardOf('core')&&computers().unassignedCore));await page.evaluate(()=>{doReset();fleetStep(50);});assert(await page.evaluate(()=>!boardOf('core')&&actuators().every(c=>act.get(c.id).u===0)));
 await page.locator('#computerDlgUndo').click();assert.strictEqual(await page.evaluate(()=>boardOf('core').id),core);await close();
 // Device assignment/removal and saved GPIO edits; ID uniqueness across staged/mounted hardware.
 await device(motor).click();await page.locator('[data-remove-assignment]').click();await close();assert(await device(motor).locator('button').isVisible());
 await device(motor).locator('button').click();await page.locator('[data-assign-board="'+core+'"]').click();
 const pin=await page.locator('#hw-pin-'+motor+' option:not([disabled])').evaluateAll(nodes=>nodes.map(n=>n.value).find(v=>v!=='-1'&&v!==nodes[0].parentNode.value));
 await pick('#hw-pin-'+motor,pin);assert.strictEqual(await page.evaluate(id=>partWiring(compById(id)).pin,motor),+pin);
 assert(await page.evaluate(()=>{const ids=[...document.querySelectorAll('[id]')].map(n=>n.id);return ids.length===new Set(ids).size;}),'Duplicate DOM IDs after mounting wiring');await close();
 // Deleting the core board leaves its duties empty; undo restores its devices and duty.
 await board(core).click();await page.locator('#bdel-'+core).click();await page.locator('#boardDeleteConfirm').click();assert(await page.evaluate(()=>!boardOf('core')&&computers().unassignedCore));await page.locator('#undoBtn').click();assert.strictEqual(await page.evaluate(()=>boardOf('core').id),core);assert.strictEqual(await page.evaluate(id=>partWiring(compById(id)).pin,motor),+pin);
 // Sensor drivers and shared I2C bus live in device and owning board views, not the sidebar.
 await device(imu).click();await pick('#hw-driver-'+imu,'custom');await page.locator('#computerDlg button').filter({hasText:'Edit C driver'}).click();
 assert(await page.locator('#hw-code-'+core).isVisible());const draft='// owner-specific driver draft';await page.locator('#hw-code-'+core).fill(draft);
 await close();await board(core).click();assert.strictEqual(await page.locator('#hw-code-'+core).inputValue(),draft);assert(await page.locator('#hw-pin-'+motor).isVisible());
 const sda=await page.locator('#hw-sda-'+imu+' option:not([disabled])').evaluateAll(nodes=>nodes.map(n=>n.value).find(v=>v!==nodes[0].parentNode.value));await pick('#hw-sda-'+imu,sda);
 assert.strictEqual(await page.locator('#hw-sda-'+mag).inputValue(),sda);await close();
 // Radio and its second link share the telemetry duty; settings and card summary follow edits.
 await page.locator('[data-radio="primary"]').click();await pick('#hw-link-'+core,'wifi');await pick('#hw-link2-'+core,'serial');await close();assert.strictEqual(await page.locator('#computerRadio .computer-card').count(),2);
 await page.locator('[data-radio="secondary"]').click();await page.locator('[data-assign-board="'+extra+'"]').click();assert.strictEqual(await page.evaluate(()=>boardOf('tlm').id),extra);assert(await page.locator('#hw-link-'+extra).isVisible());await close();
 // Wiring overview includes command module and its editable inputs.
 await page.locator('#wiringOpen').click();assert((await page.locator('#wiringOverview').innerText()).includes('Command module'));assert((await page.locator('#wiringOverview').innerText()).includes('Arm button'));
 await page.locator('[data-hw-role="ground"] summary').click();const arm=await page.locator('#hw-ground-arm-0 option:not([disabled])').evaluateAll(nodes=>nodes.map(n=>n.value).find(v=>v!=='-1'&&v!==nodes[0].parentNode.value));await pick('#hw-ground-arm-0',arm);
 assert.strictEqual(await page.evaluate(()=>groundHardware(computers()).arm[0]),+arm);assert((await page.locator('#wiringOverview').innerText()).includes('GPIO '+arm));await close();
 await board('ground').click();assert(await page.locator('[data-hw-role="ground"]').isVisible());assert(await page.locator('#gdel').isDisabled());await close();
 // Install/export remains the full existing guide, with actual per-board config/program files.
 await page.locator('#binst-'+pi).click();assert((await page.locator('#installTitle').innerText()).includes('Install / export'));
 const expected=await page.evaluate(async id=>Array.from(new Uint8Array(await new Blob([instFile('nav',computers().boards.find(b=>b.id===id)).data]).arrayBuffer())),pi);
 const downloadEvent=page.waitForEvent('download');await page.locator('[data-export-file="nav"]').click();const download=await downloadEvent;assert.deepStrictEqual([...fs.readFileSync(await download.path())],expected);
 await page.locator('#installClose').click();
 // A dedicated editor shows one formula/code first; Apply, native keyboard shortcut, errors and drafts survive.
 await page.locator('#formulasOpen').click();await pick('#formulaSelect','positionControl');assert.strictEqual(await page.locator('#formulaActive .law').count(),1);assert(await page.locator('#code-positionControl').isVisible());assert(!await page.locator('#formulaActive .formula-reference').evaluate(n=>n.open));
 const source=await page.evaluate(()=>LAWS.positionControl.src),edited=source+'\n// applied in the editor';
 await page.locator('#code-positionControl').fill('function positionControl() { return null; }');await page.locator('#code-positionControl').press('Control+Enter');assert(await page.locator('#formulaActive .law-err').isVisible());assert.strictEqual(await page.evaluate(()=>LAWS.positionControl.src),source); await page.locator('#code-positionControl').fill(edited);await page.locator('#code-positionControl').press('Control+Enter');assert.strictEqual(await page.evaluate(()=>LAWS.positionControl.src),edited);
 await page.locator('#formulaDlgUndo').click();assert.strictEqual(await page.evaluate(()=>LAWS.positionControl.src),source);assert.strictEqual(await page.locator('#code-positionControl').inputValue(),source);await page.locator('#formulaDlgRedo').click();assert.strictEqual(await page.locator('#code-positionControl').inputValue(),edited);
 const formulaDraft=edited+'\n// unsaved draft';await page.locator('#code-positionControl').fill(formulaDraft);await pick('#formulaSelect','attitudeControl');await pick('#formulaSelect','positionControl');assert.strictEqual(await page.locator('#code-positionControl').inputValue(),formulaDraft);
 assert(await page.evaluate(()=>!fleetSelect(null)),'Editor permitted drone deselection');await close();
 // Board deletion disconnects explicit device routes, never silently assigns them to a reused ID.
 await device(motor).click();await page.locator('[data-assign-board="'+extra+'"]').click();await close();await board(extra).click();await page.locator('#bdel-'+extra).click();await page.locator('#boardDeleteConfirm').click();
 assert(await page.evaluate(id=>hardwareOwner(computers(),compById(id))===null,motor));assert(await page.evaluate(()=>!boardOf('tlm')));
 await page.locator('#undoBtn').click();assert.strictEqual(await page.evaluate(id=>hardwareOwner(computers(),compById(id)).id,motor),extra);
 const owner=await page.evaluate(()=>fleet.selected.id);await page.evaluate(()=>fleetCreate('hex'));await page.evaluate(id=>fleetSelect(id),owner);
 await page.locator('#formulasOpen').click();await pick('#formulaSelect','positionControl');assert.strictEqual(await page.locator('#code-positionControl').inputValue(),formulaDraft);await close();
 for(const width of [1700,390])for(const theme of ['light','dark']){
   await page.setViewportSize({width,height:width===390?844:1100});await page.evaluate(t=>{document.documentElement.dataset.theme=t;UI_PANELS.editor.select('form');},theme);
   await page.locator('#boardAdd').scrollIntoViewIfNeeded();assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));
   if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-'+width+'-'+theme+'.png'});
   for(const view of ['board','formulas','wiring']){
     if(view==='board')await board(core).click();else await page.locator(view==='formulas'?'#formulasOpen':'#wiringOpen').click();
     const selector=view==='formulas'?'#formulaDlg':'#computerDlg',bounds=await page.locator(selector).boundingBox();assert(bounds.x>=0&&bounds.x+bounds.width<=width+1&&bounds.y>=0&&bounds.y+bounds.height<=(width===390?844:1100)+1,'Modal exceeds viewport: '+JSON.stringify(bounds));
     assert(await page.locator(selector).evaluate(n=>n.scrollWidth<=n.clientWidth+1),'Horizontal dialog overflow');
     if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-'+view+'-'+width+'-'+theme+'.png'});await close();
   }
 }
 await page.evaluate(()=>fleetSave());await page.reload({waitUntil:'networkidle'});await page.waitForFunction(()=>fleet.ready&&brt.ready);
 assert.strictEqual(await page.evaluate(id=>partWiring(compById(id)).board,motor),extra);assert.strictEqual(await page.evaluate(()=>groundHardware(computers()).arm[0]),+arm);
 assert.deepStrictEqual(errors,[]);console.log('Computers inventory, board add/delete/undo, duties and no-core idle, device/GPIO/driver/radio assignments, command wiring, actual install exports, formula editor/drafts, ownership/reload and desktop/phone themes passed.');
}finally{if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
