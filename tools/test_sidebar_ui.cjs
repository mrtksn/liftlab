#!/usr/bin/env node
'use strict';
// Native sidebar workflows, retained design data, selected-drone ownership and responsive scrolling.
const fs=require('fs'),path=require('path'),http=require('http'),assert=require('assert');
const {chromium}=require(process.env.PLAYWRIGHT_PATH || '/Users/mrtksn/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const root=path.resolve(__dirname,'..');
const comparable=d=>{d=JSON.parse(JSON.stringify(d));delete d.computers.nextBoardId;return d;};
const server=http.createServer((req,res)=>{
  const file=path.resolve(root,'.'+new URL(req.url,'http://localhost').pathname.replace(/^\/$/,'/index.html'));
  if(!file.startsWith(root+path.sep)){res.writeHead(403).end();return;}
  fs.readFile(file,(err,data)=>{if(err){res.writeHead(404).end();return;}res.setHeader('Cache-Control','no-store');res.setHeader('Content-Type',file.endsWith('.js')?'application/javascript':file.endsWith('.css')?'text/css':file.endsWith('.html')?'text/html':'application/octet-stream');res.end(data);});
});
(async()=>{await new Promise(r=>server.listen(0,'127.0.0.1',r));let browser;
try{
  browser=await chromium.launch({executablePath:process.env.CHROME_PATH || '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
  const page=await browser.newPage({viewport:{width:1600,height:1000}}),errors=[];
  page.on('pageerror',e=>{errors.push(e.stack);console.error(e.stack);});await page.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());
  await page.goto(process.env.LIVE_URL || 'http://127.0.0.1:'+server.address().port,{waitUntil:'networkidle'});
  await page.waitForFunction(()=>typeof fleet!=='undefined'&&fleet.ready&&brt.ready);
  await page.evaluate(()=>{running=false;setTerrain('open',1);UI_PANELS.editor.select('air');});
  assert(await page.locator('#designSaveAs').isVisible());assert(!await page.locator('#designName').isVisible());assert(!await page.locator('[data-add="motor"]').isVisible());
  assert(await page.locator('#droneName').isVisible());
  await page.locator('#droneName').fill('Workshop quad');await page.locator('#droneName').press('Tab');
  assert.strictEqual(await page.evaluate(()=>fleet.selected.name),'Workshop quad');
  for(const [button,dialog] of [['designSaveAs','designSaveDlg'],['designFiles','designFilesDlg'],['designManage','designManageDlg'],['addPart','addPartDlg']]){
    await page.locator('#'+button).click();assert(await page.locator('#'+dialog).isVisible());
    const id=await page.evaluate(()=>fleet.selected.id);
    assert(await page.evaluate(()=>!fleetSelect(null)),'Modal allowed deselection');
    await page.keyboard.press('Escape');await page.waitForFunction(id=>!document.getElementById(id).open,dialog);
    assert.strictEqual(await page.evaluate(()=>document.activeElement.id),button,'Escape did not restore focus');
    assert.strictEqual(await page.evaluate(()=>fleet.selected.id),id);
  }
  await page.locator('#designFiles').click();const beforeModalUndo=await page.evaluate(()=>designSnap());await page.locator('#designFilesDlg [data-close-dialog]').focus();await page.keyboard.press('Control+z');assert.strictEqual(await page.evaluate(()=>designSnap()),beforeModalUndo);await page.keyboard.press('Escape');
  // Save as keeps earlier copies and reports naming/storage errors inside the dialog.
  await page.locator('#designSaveAs').click();await page.locator('#designName').fill('Original');await page.locator('#designSave').click();
  await page.waitForFunction(()=>!$('#designSaveDlg').open&&designs.name==='Original');
  const original=await page.evaluate(()=>({id:designs.cur,snapshot:designSnap()}));
  await page.evaluate(()=>{cfg.frame.mass+=.2;recomputeProps();save();});
  await page.locator('#designSaveAs').click();await page.locator('#designName').fill('Original');await page.locator('#designSave').click();
  assert(await page.locator('#designSaveDlg').isVisible());assert((await page.locator('#designSaveDlg [data-design-note]').innerText()).includes('already saved'));
  assert.strictEqual(await page.evaluate(()=>designs.list.length),1);
  await page.locator('#designName').fill('Copy');await page.locator('#designSave').click();await page.waitForFunction(()=>!$('#designSaveDlg').open);
  assert(await page.evaluate(({id,snapshot})=>designs.cur!==id&&JSON.stringify(designs.list.find(d=>d.id===id).design)===snapshot,original),'Save as replaced original');
  await page.locator('#designSaveAs').click();await page.locator('#designName').fill('Storage failure');
  await page.evaluate(()=>{window.sidebarSetItem=Storage.prototype.setItem;Storage.prototype.setItem=function(k,v){if(k===LSD)throw Error('quota');return window.sidebarSetItem.call(this,k,v);};});await page.locator('#designSave').click();
  await page.waitForFunction(()=>!$('#designSave').disabled);assert(await page.locator('#designSaveDlg').isVisible());assert((await page.locator('#designSaveDlg [data-design-note]').innerText()).includes("won't store"));
  await page.evaluate(()=>{Storage.prototype.setItem=window.sidebarSetItem;});await page.keyboard.press('Escape');
  // Actual export download includes tuning, boards, parts; a cancelled Save as draft does not rename it.
  const expected=await page.evaluate(()=>JSON.parse(designSnap()));
  await page.locator('#designFiles').click();const downloadEvent=page.waitForEvent('download');await page.locator('#designExport').click();const download=await downloadEvent;
  assert.strictEqual(download.suggestedFilename(),'Copy.json');const exported=JSON.parse(fs.readFileSync(await download.path(),'utf8'));assert.deepStrictEqual(exported.design,expected);await page.keyboard.press('Escape');
  // Library opens the selected craft only, with unsaved-change protection.
  await page.evaluate(()=>{cfg.frame.mass+=.3;recomputeProps();save();});const dirty=await page.evaluate(()=>designSnap());
  await page.locator('#designManage').click();await page.locator('#designList .dname').filter({hasText:'Original'}).click();
  assert(await page.locator('#saveAsk').isVisible());await page.keyboard.press('Escape');assert.strictEqual(await page.evaluate(()=>designSnap()),dirty);
  await page.locator('#designManage').click();await page.locator('#designList .dname').filter({hasText:'Original'}).click();
  await page.locator('#saveAsk button[value="discard"]').click();await page.waitForFunction(()=>designs.name==='Original');assert.deepStrictEqual(comparable(await page.evaluate(()=>JSON.parse(designSnap()))),comparable(JSON.parse(original.snapshot)));
  // Import bad JSON stays in the file dialog with a message; good JSON requires confirmation when dirty.
  await page.locator('#designFiles').click();await page.locator('#designFile').setInputFiles({name:'bad.json',mimeType:'application/json',buffer:Buffer.from('{bad')});
  await page.waitForFunction(()=>$('#designFilesDlg').querySelector('[data-design-note]').textContent.includes("isn't a design"));assert(await page.locator('#designFilesDlg').isVisible());
  await page.keyboard.press('Escape');await page.evaluate(()=>{cfg.frame.mass+=.1;recomputeProps();save();});
  await page.locator('#designFiles').click();await page.locator('#designFile').setInputFiles({name:'Copy.json',mimeType:'application/json',buffer:Buffer.from(JSON.stringify(exported))});
  await page.locator('#saveAsk button[value="save"]').click();await page.waitForFunction(()=>designs.name==='Copy'&&!$('#saveAsk').open&&brt.ready);assert.deepStrictEqual(comparable(await page.evaluate(()=>JSON.parse(designSnap()))),comparable(expected));
  // Grouped palette retains placement and exactly one expanded editor.
  const count=await page.evaluate(()=>cfg.comps.length);await page.locator('#addPart').click();assert.strictEqual(await page.locator('#addPartDlg h3').count(),5);   // power, propulsion, structure & motion, payloads, sensors
  await page.locator('[data-add="joint"]').click();assert(!await page.locator('#addPartDlg').isVisible());await page.locator('.place-opt').first().click();assert.strictEqual(await page.evaluate(()=>cfg.comps.length),count+1);
  assert.strictEqual(await page.locator('#compList .comp.open').count(),1);
  for(const n of [0,1,0]){await page.locator('#compList .comp-head').nth(n).click();assert.strictEqual(await page.locator('#compList .comp.open').count(),1);}
  await page.locator('#compList .comp-head').first().click();assert.strictEqual(await page.locator('#compList .comp.open').count(),0);
  await page.evaluate(()=>{setEditMode(true);selectComp(cfg.comps[0].id);selectComp(cfg.comps[1].id);});assert.strictEqual(await page.locator('#compList .comp.open').count(),1);
  await page.evaluate(()=>setEditMode(false));
  // Tune uses the same existing inputs; its edits and history retain controller data.
  await page.locator('#tabTune').click();assert(await page.locator('#bodySec').isVisible());assert(await page.locator('#tuneSec').isVisible());assert(!await page.locator('#partsSec').isVisible());
  await page.locator('#frameFold > summary').click();const mass=await page.evaluate(()=>cfg.frame.mass);await page.locator('#frameMass-n').fill(String(mass+.25));await page.locator('#frameMass-n').dispatchEvent('input');
  await page.locator('#undoBtn').click();assert.strictEqual(await page.evaluate(()=>cfg.frame.mass),mass);assert(await page.locator('#paneTune').isVisible());await page.locator('#redoBtn').click();assert.strictEqual(await page.evaluate(()=>cfg.frame.mass),mass+.25);
  const tuning=await page.evaluate(()=>JSON.stringify(tuneOf()));await page.locator('#tune-rp-fold summary').click();
  await page.evaluate(()=>{const input=$('#tune-rp-fold input[type="range"]');input.value=String(+input.value+.1);input.dispatchEvent(new Event('input',{bubbles:true}));});
  assert.notStrictEqual(await page.evaluate(()=>JSON.stringify(tuneOf())),tuning);await page.locator('#undoBtn').click();assert.strictEqual(await page.evaluate(()=>JSON.stringify(tuneOf())),tuning);
  // Delayed save captures the submitted snapshot, retains later dirty edits and owns its drone.
  assert(await page.evaluate(async()=>{
    const other=fleetCreate('hex');fleetSelect(fleet.drones[0].id);const before=designSnap(),store=storeDesign;let finish;
    storeDesign=rec=>new Promise(resolve=>{finish=()=>{store(rec).then(resolve);};});$('#designName').value='Delayed copy';
    const saving=saveDesign(true);const locked=!fleetSelect(other.id);cfg.frame.mass+=.15;recomputeProps();save();finish();await saving;storeDesign=store;
    return locked&&designs.savedSnap===before&&designChanged()&&designs.list.find(d=>d.name==='Delayed copy').design.frame===JSON.parse(before).frame;
  }));
  for(const tab of ['gs','ai']){await page.evaluate(tab=>UI_PANELS.editor.select(tab),tab);assert(!await page.locator('#sidebarHistory').isVisible());}
  // Persistent toolbar and modal bounds on large desktop and narrow touch layout in both themes.
  for(const width of [1600,390])for(const theme of ['light','dark']){
    await page.setViewportSize({width,height:width===390?844:1000});await page.evaluate(theme=>{document.documentElement.dataset.theme=theme;UI_PANELS.editor.select('air');},theme);
    for(const bottom of [false,true]){
      await page.evaluate(bottom=>{const p=$('#airframe');if(getComputedStyle(p).overflowY==='visible'){window.scrollTo(0,bottom?p.offsetTop+p.offsetHeight-innerHeight:p.offsetTop);}else p.scrollTop=bottom?p.scrollHeight:0;},bottom);
      await page.waitForTimeout(100);
      if(process.env.TEST_SCREENSHOTS&&!bottom)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-top-'+width+'-'+theme+'.png'});
      const bounds=await page.locator('#sidebarHistory').boundingBox();assert(bounds&&bounds.y>=0&&bounds.y+bounds.height<= (width===390?844:1000)+1,'History left the viewport: '+JSON.stringify(bounds));
    }
    if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-'+width+'-'+theme+'.png'});
    await page.locator('#addPart').click();const b=await page.locator('#addPartDlg').boundingBox();assert(b.x>=0&&b.x+b.width<=width+1&&b.y>=0&&b.y+b.height<= (width===390?844:1000)+1,'Part dialog overflow');
    if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-picker-'+width+'-'+theme+'.png'});
    await page.keyboard.press('Escape');
  }
  await page.evaluate(()=>{UI_PANELS.editor.select('tune');fleetSave();});await page.reload({waitUntil:'networkidle'});await page.waitForFunction(()=>typeof fleet!=='undefined'&&fleet.ready&&brt.ready);
  assert(await page.locator('#paneTune').isVisible(),'Tune tab not restored');assert.strictEqual(await page.evaluate(()=>fleet.selected.name),'Workshop quad');
  assert.deepStrictEqual(errors,[]);
  const base=process.env.LIVE_URL || 'http://127.0.0.1:'+server.address().port;await page.goto(base.replace(/\/$/,'')+'/tools/test_ui_components.html',{waitUntil:'networkidle'});await page.waitForFunction(()=>document.getElementById('testResults').textContent.includes('PASSED'));assert((await page.locator('#testResults').innerText()).includes('ALL 9 PASSED'));
  console.log('Sidebar dialogs, copy/error/import/export/library, parts accordion/placement, Tune/PID/history, async ownership, desktop/mobile themes and reload passed.');
}finally{if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
