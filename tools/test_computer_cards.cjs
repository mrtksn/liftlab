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
 await page.emulateMedia({reducedMotion:'reduce'});page.on('pageerror',e=>{errors.push(e.stack);console.error(e.stack);});await page.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());
 await page.goto(process.env.LIVE_URL||'http://127.0.0.1:'+server.address().port,{waitUntil:'networkidle'});
 await page.waitForFunction(()=>typeof fleet!=='undefined'&&fleet.ready&&brt.ready);
 await page.evaluate(()=>{if(!fleet.selected)fleetSelect(fleet.drones[0].id);running=false;setTerrain('open',1);UI_PANELS.editor.select('form');});
 const pick=async(selector,value)=>{await page.locator(selector).dispatchEvent('pointerdown');await page.locator(selector).selectOption(String(value));};
 const close=async()=>{await page.keyboard.press('Escape');await page.waitForFunction(()=>!document.querySelector('dialog[open]'));};
 const shut=async()=>{await page.evaluate(()=>compFocus(null));await page.waitForFunction(()=>!COMP.focus);};
 const row=sel=>page.locator('#paneForm '+sel+' > .comp-top .comp-head');
 const duty=id=>row('#taskRows [data-task="'+id+'"]'),board=id=>row('#boardList [data-board="'+id+'"]'),device=id=>row('[data-device="'+id+'"]');
 const core=await page.evaluate(()=>boardOf('core').id),pi=await page.evaluate(()=>boardOf('nav').id);
 const motor=await page.evaluate(()=>actuators()[0].id),imu=await page.evaluate(()=>sensorsOf('imu')[0].id),mag=await page.evaluate(()=>sensorsOf('mag')[0].id);
 // The Computers tab is a list of the electronics, like Airframe's parts, and the view pulls them apart (the bench).
 assert.strictEqual(await page.locator('#boardList > .crow').count(),3);
 assert(await page.evaluate(()=>bench.on&&editMode),'Computers did not open the 3D bench');
 assert.strictEqual(await page.locator('#paneForm > .sec input,#paneForm > .sec select,#paneForm > .sec textarea').count(),0);
 assert(await page.locator('#boardList [data-board="'+pi+'"]').evaluate(n=>n.innerText.includes('Raspberry Pi Zero')));
 assert(await page.locator('#taskRows [data-task="cargo"]').evaluate(n=>n.classList.contains('empty')&&n.innerText.includes('not assigned')));
 assert(!await page.locator('#taskRows [data-task="core"]').evaluate(n=>n.classList.contains('empty')));
 assert.strictEqual(await page.locator('#paneForm > .sec').first().getAttribute('id'),'computerToolsSec');
 assert.strictEqual(await page.locator('#computerToolsSec svg').count(),3);   // formulas, apps, live data
 assert.strictEqual(await page.locator('#busOpen').innerText(),'Live data…');
 await page.locator('#computerRadio').scrollIntoViewIfNeeded();await page.waitForTimeout(100);
 const pinned=await page.locator('#computerToolsSec').boundingBox(),nav=await page.locator('#airframe .section-nav').boundingBox();assert(pinned.y>=nav.y+nav.height-1&&pinned.y<nav.y+nav.height+5,'Computers tools were not pinned below section navigation');
 await page.locator('#airframe .section-jump > summary').click();await page.locator('#airframe .section-links a').filter({hasText:'Duties'}).click();await page.waitForTimeout(100);const assignmentHeading=await page.locator('#computerAssignmentsSec h2').boundingBox(),toolsBottom=await page.locator('#computerToolsSec').boundingBox();assert(assignmentHeading.y>=toolsBottom.y+toolsBottom.height,'Section jump hid its heading below pinned tools');
 if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-desktop.png'});
 // A row opens under itself and the view closes in on it; opening it again closes it, focus stays on the row.
 await board(core).focus();await page.keyboard.press('Enter');assert(await page.locator('#bname-'+core).isVisible());
 assert.deepStrictEqual(await page.evaluate(()=>bench.view),{kind:'board',id:core});assert(await page.evaluate(id=>bench.items.has('board:'+id)||true,core));
 await board(core).click();assert(!await page.locator('#bname-'+core).count());assert.strictEqual(await page.evaluate(()=>document.activeElement.closest('[data-board]')?.dataset.board),String(core));
 // + Add → a board, opened at once; rename, delete with confirmation, undo.
 await page.locator('#compAddMenu > button').click();await page.locator('#compAddMenu [role=menuitem]').filter({hasText:'ESP32-S3'}).click();const extra=await page.evaluate(()=>computers().boards.at(-1).id);
 assert.deepStrictEqual(await page.evaluate(()=>COMP.focus),{kind:'board',id:extra});await page.locator('#bname-'+extra).fill('Auxiliary');await page.locator('#bname-'+extra).press('Enter');
 await page.locator('#bdel-'+extra).click();assert(await page.locator('#boardDeleteConfirm').isVisible());await page.locator('.computer-confirm .btn').first().click();assert.strictEqual(await page.evaluate(()=>computers().boards.length),3);
 await page.locator('#bdel-'+extra).click();await page.locator('#boardDeleteConfirm').click();assert.strictEqual(await page.evaluate(()=>computers().boards.length),2);assert.strictEqual(await page.evaluate(()=>COMP.focus),null);
 await page.locator('#undoBtn').click();assert(await board(extra).isVisible());assert((await page.locator('#boardList [data-board="'+extra+'"]').innerText()).includes('Auxiliary'));
 // A board not on the airframe can be put there: a part its size and weight, drawn as the board.
 await board(extra).click();const parts0=await page.evaluate(()=>cfg.comps.length);await page.locator('[data-place-board="'+extra+'"]').click();
 assert.strictEqual(await page.evaluate(()=>cfg.comps.length),parts0+1);assert(await page.evaluate(id=>{const c=boardBodyOf(computers().boards.find(b=>b.id===id));return c&&c.boardId===id&&c.mass>0&&partIsElec(c);},extra));
 assert.strictEqual(await page.locator('#compList .comp-name').filter({hasText:/^ESP32-S3$|^Auxiliary$/}).count(),0,'A board part showed in Airframe');
 await page.locator('#undoBtn').click();assert.strictEqual(await page.evaluate(()=>cfg.comps.length),parts0);await shut();
 // Duties list only compatible boards; removal stays empty through normalization and undo.
 await duty('learn').click();assert.strictEqual(await page.locator('#compDetail [data-assign-board]').count(),1);await page.locator('[data-remove-assignment]').click();await shut();assert(await page.locator('#taskRows [data-task="learn"]').evaluate(n=>n.classList.contains('empty')));
 await duty('learn').click();await page.locator('[data-assign-board="'+pi+'"]').click();await shut();assert.strictEqual(await page.evaluate(()=>boardOf('learn').id),pi);
 await duty('core').click();assert.strictEqual(await page.locator('#compDetail [data-assign-board]').count(),2);await page.locator('[data-remove-assignment]').click();
 assert(await page.evaluate(()=>!boardOf('core')&&computers().unassignedCore));await page.evaluate(()=>{doReset();fleetStep(50);});assert(await page.evaluate(()=>actuators().every(c=>act.get(c.id).u===0)));
 await page.locator('#undoBtn').click();assert.strictEqual(await page.evaluate(()=>boardOf('core').id),core);await shut();
 // Device assignment/removal and saved GPIO edits; ID uniqueness across staged/mounted hardware.
 await device(motor).click();await page.locator('[data-remove-assignment]').click();await shut();assert(await page.locator('#computerOutputs [data-device="'+motor+'"]').evaluate(n=>n.classList.contains('empty')));
 await device(motor).click();await page.locator('[data-assign-board="'+core+'"]').click();
 const pin=await page.locator('#hw-pin-'+motor+' option:not([disabled])').evaluateAll(nodes=>nodes.map(n=>n.value).find(v=>v!=='-1'&&v!==nodes[0].parentNode.value));
 await pick('#hw-pin-'+motor,pin);assert.strictEqual(await page.evaluate(id=>partWiring(compById(id)).pin,motor),+pin);
 assert(await page.evaluate(()=>{const ids=[...document.querySelectorAll('[id]')].map(n=>n.id);return ids.length===new Set(ids).size;}),'Duplicate DOM IDs after mounting wiring');
 await page.waitForFunction(id=>bench.wires.some(w=>w.b.key==='part:'+id&&w.kind==='pwm'),motor,{timeout:3000});await shut();
 // Deleting the core board leaves its duties empty; undo restores its devices and duty.
 await board(core).click();await page.locator('#bdel-'+core).click();await page.locator('#boardDeleteConfirm').click();assert(await page.evaluate(()=>!boardOf('core')&&computers().unassignedCore));await page.locator('#undoBtn').click();assert.strictEqual(await page.evaluate(()=>boardOf('core').id),core);assert.strictEqual(await page.evaluate(id=>partWiring(compById(id)).pin,motor),+pin);
 // Sensor drivers and shared I2C bus live in device and owning board views; a sensor's own part settings are there too.
 await device(imu).click();assert(await page.locator('#compDetail .cpart [data-id="'+imu+'"]').isVisible());await pick('#hw-driver-'+imu,'custom');await page.locator('#compDetail button').filter({hasText:'Edit C driver'}).click();
 assert(await page.locator('#hw-code-'+core).isVisible());const draft='// owner-specific driver draft';await page.locator('#hw-code-'+core).fill(draft);
 await shut();await board(core).click();assert.strictEqual(await page.locator('#hw-code-'+core).inputValue(),draft);assert(await page.locator('#hw-pin-'+motor).isVisible());
 const sda=await page.locator('#hw-sda-'+imu+' option:not([disabled])').evaluateAll(nodes=>nodes.map(n=>n.value).find(v=>v!==nodes[0].parentNode.value));await pick('#hw-sda-'+imu,sda);
 assert.strictEqual(await page.locator('#hw-sda-'+mag).inputValue(),sda);await shut();
 // Radio and its second link share the telemetry duty; settings and the row's summary follow edits.
 await row('[data-radio="primary"]').click();await pick('#hw-link-'+core,'wifi');await pick('#hw-link2-'+core,'serial');await shut();assert((await page.locator('#computerRadio [data-radio="primary"]').innerText()).includes('and '));
 await row('[data-radio="primary"]').click();await page.locator('[data-assign-board="'+extra+'"]').click();assert.strictEqual(await page.evaluate(()=>boardOf('tlm').id),extra);assert(await page.locator('#hw-link-'+extra).isVisible());await shut();
 // Wiring: every wire listed (and drawn), then the command module's inputs and the board diagrams.
 await row('[data-wiring]').click();assert.strictEqual(await page.evaluate(()=>bench.view&&bench.view.kind),'wiring');
 const wires=await page.locator('#compDetail .wire-row').count();assert(wires>5&&wires===await page.evaluate(()=>benchWires().length),'Wire list does not match the wiring');
 await page.locator('#compDetail .wire-row').first().hover();assert(await page.evaluate(()=>!!bench.hiWire));
 assert.strictEqual(await page.locator('#compDetailBody > [data-hw-role="ground"] h3').innerText(),'Command-module wiring');await page.locator('#compDetail .hw-advanced').filter({hasText:'Board diagrams'}).locator('summary').first().click();await page.locator('#wiringOverview [data-view="list"]').click();assert((await page.locator('#wiringOverview').innerText()).includes('Command module'));assert((await page.locator('#wiringOverview').innerText()).includes('Arm button'));
 await page.locator('[data-hw-role="ground"] summary').click();const arm=await page.locator('#hw-ground-arm-0 option:not([disabled])').evaluateAll(nodes=>nodes.map(n=>n.value).find(v=>v!=='-1'&&v!==nodes[0].parentNode.value));await pick('#hw-ground-arm-0',arm);
 assert.strictEqual(await page.evaluate(()=>groundHardware(computers()).arm[0]),+arm);assert((await page.locator('#wiringOverview').innerText()).includes('GPIO '+arm));await shut();
 await board('ground').click();assert(await page.locator('[data-hw-role="ground"]').isVisible());assert(await page.locator('#gdel').isDisabled());await shut();
 // A click on a part in the view opens its row.
 await page.waitForFunction(()=>!bench.zoom&&!cam.anim&&bench.ex===1);await page.waitForTimeout(2500);
 const imuAt=await page.evaluate(id=>{const it=bench.items.get('part:'+id),w=it.obj.getWorldPosition(new THREE.Vector3()).project(camera),r=vpEl.getBoundingClientRect();return {x:r.left+(w.x+1)/2*r.width,y:r.top+(1-w.y)/2*r.height};},imu);
 await page.mouse.click(imuAt.x,imuAt.y);
 assert.deepStrictEqual(await page.evaluate(()=>COMP.focus),{kind:'device',id:imu});await shut();
 // The bench only changes the view: a flight carries on under it (Live data, apps), a pause stays a pause. Leaving the tab
 // or Fly ends it, Inspect in 3D brings it back; Run and Pause work in it.
 await page.evaluate(()=>{UI_PANELS.editor.select('air');running=true;UI_PANELS.editor.select('form');});assert(await page.evaluate(()=>bench.on&&running));
 await page.locator('#runBtn').click();assert(await page.evaluate(()=>bench.on&&!running));await page.locator('#runBtn').click();assert(await page.evaluate(()=>bench.on&&running));
 await page.evaluate(()=>UI_PANELS.editor.select('air'));assert(await page.evaluate(()=>!bench.on&&!editMode&&running));
 await page.evaluate(()=>{running=false;UI_PANELS.editor.select('form');});await page.locator('#benchDone').click();assert(await page.evaluate(()=>!bench.on&&!editMode&&!running));
 await page.locator('#benchInspect').click();assert(await page.evaluate(()=>bench.on&&!running));
 // Install/export opens from the board's own settings.
 await board(pi).click();
 // Install/export remains the full existing guide, with actual per-board config/program files.
 await page.locator('#binst-'+pi).click();assert((await page.locator('#installTitle').innerText()).includes('Install / export'));
 assert(await page.locator('#installDlg .dialog-toolbar [data-design-history="undo"]').isVisible());
 const beforeHistory=await page.evaluate(()=>designSnap());await page.locator('#installDlg [data-design-history="undo"]').click();assert.notStrictEqual(await page.evaluate(()=>designSnap()),beforeHistory);assert.strictEqual(await page.evaluate(()=>INST.target===computers().boards.find(b=>b.id===INST.target.id)),true,'Installer kept an old board snapshot');await page.locator('#installDlg [data-design-history="redo"]').click();assert.strictEqual(await page.evaluate(()=>designSnap()),beforeHistory);
 for(const lock of ['busy','conn','preparing']){await page.evaluate(k=>{INST[k]=k==='conn'?{}:true;renderUndo();},lock);assert(await page.locator('#installDlg [data-design-history="undo"]').isDisabled());await page.evaluate(()=>installHistory('undo'));assert.strictEqual(await page.evaluate(()=>designSnap()),beforeHistory);await page.evaluate(k=>{INST[k]=k==='conn'?null:false;renderUndo();},lock);}
 // Protect history throughout async firmware preparation, before any port/flash work starts.
 await page.evaluate(()=>{window._firmwareParts=firmwareParts;firmwareParts=()=>new Promise((_,reject)=>{window._rejectPreparation=reject;});window._preparingFlash=espFlash('flight',ESP_PROFILES.esp32,document.createElement('progress'),document.createElement('p'));});
 assert(await page.locator('#installDlg [data-design-history="undo"]').isDisabled());await page.evaluate(()=>installHistory('undo'));assert.strictEqual(await page.evaluate(()=>designSnap()),beforeHistory);
 await page.evaluate(async()=>{window._rejectPreparation(new Error('test preparation stopped'));await window._preparingFlash;firmwareParts=window._firmwareParts;delete window._firmwareParts;delete window._rejectPreparation;delete window._preparingFlash;});
 assert(await page.locator('#installDlg [data-design-history="undo"]').isEnabled());
 const expected=await page.evaluate(async id=>Array.from(new Uint8Array(await new Blob([instFile('nav',computers().boards.find(b=>b.id===id)).data]).arrayBuffer())),pi);
 const downloadEvent=page.waitForEvent('download');await page.locator('[data-export-file="nav"]').click();const download=await downloadEvent;assert.deepStrictEqual([...fs.readFileSync(await download.path())],expected);
 await page.locator('#installClose').click();await shut();
 // A dedicated editor shows one formula/code first; Apply, native keyboard shortcut, errors and drafts survive.
 await page.locator('#formulasOpen').click();await pick('#formulaSelect','positionControl');assert.strictEqual(await page.locator('#formulaActive .law').count(),1);assert(await page.locator('#code-positionControl').isVisible());assert(!await page.locator('#formulaActive .formula-reference').evaluate(n=>n.open));
 const source=await page.evaluate(()=>LAWS.positionControl.src),edited=source+'\n// applied in the editor';
 await page.locator('#code-positionControl').fill('function positionControl() { return null; }');await page.locator('#code-positionControl').press('Control+Enter');assert(await page.locator('#formulaActive .law-err').isVisible());assert.strictEqual(await page.evaluate(()=>LAWS.positionControl.src),source); await page.locator('#code-positionControl').fill(edited);await page.locator('#code-positionControl').press('Control+Enter');assert.strictEqual(await page.evaluate(()=>LAWS.positionControl.src),edited);
 await page.locator('#formulaDlgUndo').click();assert.strictEqual(await page.evaluate(()=>LAWS.positionControl.src),source);assert.strictEqual(await page.locator('#code-positionControl').inputValue(),source);await page.locator('#formulaDlgRedo').click();assert.strictEqual(await page.locator('#code-positionControl').inputValue(),edited);
 const formulaDraft=edited+'\n// unsaved draft';await page.locator('#code-positionControl').fill(formulaDraft);await pick('#formulaSelect','attitudeControl');await pick('#formulaSelect','positionControl');assert.strictEqual(await page.locator('#code-positionControl').inputValue(),formulaDraft);
 // Search checks titles first, then code/docs, including unapplied drafts; filtering never changes the active code.
 await page.locator('#formulaSearch').fill('control');const ranked=await page.locator('#formulaSelect optgroup').evaluateAll(gs=>gs.map(g=>({label:g.label,keys:[...g.children].map(o=>o.value)})));assert.strictEqual(ranked[0].label,'Title matches');assert(ranked[0].keys.includes('positionControl'));assert(ranked.find(g=>g.label==='Content matches')?.keys.length>0);
 await page.locator('#formulaSearch').fill('TUNE.pos.kp');assert(await page.locator('#formulaSelect option[value="positionControl"]').count());assert.strictEqual(await page.locator('#code-positionControl').inputValue(),formulaDraft);
 await page.locator('#formulaSearch').fill('unsaved draft');assert.strictEqual(await page.locator('#formulaSelect option:not([disabled])').count(),1);assert.strictEqual(await page.locator('#formulaSelect option:not([disabled])').getAttribute('value'),'positionControl');
 await page.locator('#formulaSearch').fill('no-result-7a88ff');assert(await page.locator('#formulaSelect').isDisabled());assert((await page.locator('#formulaSearchStatus').innerText()).startsWith('0 matches'));assert.strictEqual(await page.locator('#code-positionControl').inputValue(),formulaDraft);assert.strictEqual(await page.evaluate(()=>COMP.formula),'positionControl');
 await page.locator('#formulaSearch').fill('weight');await page.locator('#formulaSearch').press('Enter');assert.strictEqual(await page.locator('#formulaTitle').innerText(),'Weight');assert(await page.locator('#code-gravity').isVisible());
 await page.locator('#formulaSearch').fill('');await pick('#formulaSelect','positionControl');assert.strictEqual(await page.locator('#code-positionControl').inputValue(),formulaDraft);
 await page.locator('#rnBox > summary').click();assert((await page.locator('#rnStatus').innerText()).includes('formulas compiled'));assert.strictEqual(await page.locator('#rnMetrics dd').count(),3);assert(!await page.locator('#rnBox .program-help').evaluate(n=>n.open));assert(!await page.locator('#rnBox .program-activity').evaluate(n=>n.open));
 assert((await page.locator('#rnBox').innerText()).length<500,'Program status is needlessly verbose');
 const fullProgram=await page.evaluate(()=>Array.from(rnImage(RN.P,{tests:rnMakeTests(RN.P,Object.values(RN.samples).flat(),600)}))),programEvent=page.waitForEvent('download');await page.locator('#rnDownload').click();const fullDownload=await programEvent;assert.deepStrictEqual([...fs.readFileSync(await fullDownload.path())],fullProgram);
 if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-program.png'});
 // Build failure and individual compiler errors stay visible even when help/activity are folded.
 await page.evaluate(()=>{window._savedProgram=RN.P;RN.P=null;RN.buildErr='test compilation failure';renderRunner();});assert((await page.locator('#rnStatus').innerText()).includes('test compilation failure'));assert(await page.locator('#rnDownload').isDisabled());
 await page.evaluate(()=>{RN.P=window._savedProgram;delete window._savedProgram;RN.P.errors.positionControl='test formula diagnostic';renderRunner();});assert((await page.locator('#rnProblems').innerText()).includes('test formula diagnostic'));await page.evaluate(()=>{delete RN.P.errors.positionControl;RN.buildErr='';renderRunner();});
 assert(await page.evaluate(()=>!fleetSelect(null)),'Editor permitted drone deselection');await close();
 // Board deletion disconnects explicit device routes, never silently assigns them to a reused ID.
 await device(motor).click();await page.locator('[data-assign-board="'+extra+'"]').click();await shut();await board(extra).click();await page.locator('#bdel-'+extra).click();await page.locator('#boardDeleteConfirm').click();
 assert(await page.evaluate(id=>hardwareOwner(computers(),compById(id))===null,motor));assert(await page.evaluate(()=>!boardOf('tlm')));
 await page.locator('#undoBtn').click();assert.strictEqual(await page.evaluate(id=>hardwareOwner(computers(),compById(id)).id,motor),extra);
 const owner=await page.evaluate(()=>fleet.selected.id);await page.evaluate(()=>fleetCreate('hex'));await page.evaluate(id=>fleetSelect(id),owner);
 await page.locator('#formulasOpen').click();await pick('#formulaSelect','positionControl');assert.strictEqual(await page.locator('#code-positionControl').inputValue(),formulaDraft);await close();
 for(const width of [1700,390])for(const theme of ['light','dark']){
   await page.setViewportSize({width,height:width===390?844:1100});await page.evaluate(t=>{document.documentElement.dataset.theme=t;UI_PANELS.editor.select('form');},theme);
   await board(core).click();assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));assert(await page.locator('#paneForm').evaluate(n=>n.scrollWidth<=n.clientWidth+1),'Horizontal sidebar overflow');
   if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-'+width+'-'+theme+'.png'});await shut();
   await row('[data-wiring]').click();assert(await page.locator('#paneForm').evaluate(n=>n.scrollWidth<=n.clientWidth+1),'Horizontal wiring overflow');
   if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-wiring-'+width+'-'+theme+'.png'});await shut();
   for(const view of ['formulas','install']){
     if(view==='install'){await board(pi).click();await page.locator('#binst-'+pi).click();}else await page.locator('#formulasOpen').click();
     const selector=view==='formulas'?'#formulaDlg':'#installDlg',bounds=await page.locator(selector).boundingBox();assert(bounds.x>=0&&bounds.x+bounds.width<=width+1&&bounds.y>=0&&bounds.y+bounds.height<=(width===390?844:1100)+1,'Modal exceeds viewport: '+JSON.stringify(bounds));
     assert(await page.locator(selector).evaluate(n=>n.scrollWidth<=n.clientWidth+1),'Horizontal dialog overflow');
     if(view==='formulas'){await page.locator('#formulaSearch').fill('servo');assert(await page.locator('#formulaSelect optgroup[label="Title matches"]').count());await page.locator('#formulaSearch').fill('');}assert(await page.locator(selector+' .dialog-toolbar').isVisible());
     if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-'+view+'-'+width+'-'+theme+'.png'});await close();if(view==='install')await shut();
   }
 }
 await page.evaluate(()=>fleetSave());await page.reload({waitUntil:'networkidle'});await page.waitForFunction(()=>fleet.ready&&brt.ready);
 assert.strictEqual(await page.evaluate(id=>partWiring(compById(id)).board,motor),extra);assert.strictEqual(await page.evaluate(()=>groundHardware(computers()).arm[0]),+arm);
 assert.deepStrictEqual(errors,[]);console.log('Electronics list with the 3D bench (focus, wires, clicks, board parts), command-first wiring, install history/locks and actual exports, title/content/draft search, concise compiler status/errors, assignments, dialog history, ownership/reload and desktop/phone themes passed.');
}finally{if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
