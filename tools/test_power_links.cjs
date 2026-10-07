#!/usr/bin/env node
'use strict';
// Computers → Power and Board links: a drone with no battery, or none connected, doesn't turn on; Add a part →
// Battery and Connect bring the power back; the choice is part of the design (undo, files, reload); the
// flight controller ↔ Pi link card shows both ends and the wires.
//   node tools/test_power_links.cjs
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
 browser=await chromium.launch({executablePath:process.env.CHROME_PATH || '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',headless:true});
 const page=await browser.newPage({viewport:{width:1400,height:900}}),errors=[];
 page.on('pageerror',e=>{errors.push(e.stack);console.error(e.stack);});
 await page.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());
 await page.goto('http://127.0.0.1:'+server.address().port,{waitUntil:'networkidle'});
 await page.waitForFunction(()=>fleet.ready&&brt.ready);
 console.log(await page.evaluate(async()=>{
  const check=(v,m)=>{if(!v)throw Error(m);},wait=ms=>new Promise(r=>setTimeout(r,ms));
  UI_PANELS.editor.select('form');renderComputers(true);
  const bat=batteryParts(cfg.comps)[0];check(bat&&cargo.power,'The layout should start powered by its battery');
  check($('[data-power="'+bat.id+'"]').innerText.includes('Connected'),'Battery card not shown as connected');
  // disconnect: no power, and why
  $('[data-power="'+bat.id+'"]').click();await wait(50);$('[data-power-toggle]').click();await wait(50);
  check(!cargo.power&&powerWhy()==="the battery isn't connected"&&!batteryWired(computers(),bat),'Disconnecting did not cut the power');
  running=true;await wait(800);check([...act.values()].every(s=>!(s.u>0))&&brt.fcState===0,'Motors or flight core ran with no power');
  check(/isn't connected/.test(brt.fcWhy),'Flight core status does not say why: '+brt.fcWhy);
  check(hardwareOverview(computers(),cfg.comps).groups[0].rows.some(r=>r.connection==='Not connected'),'Overview does not list the unplugged battery');
  // it's design data: in the snapshot and a design file
  check(JSON.parse(designSnap()).computers.wiring.parts[bat.id].power===false,'Disconnection not in the design');
  const file=JSON.stringify({format:FILE_FORMAT,version:1,name:'t',design:JSON.parse(designSnap())});
  undoStep();check(cargo.power&&batteryWired(computers(),bat),'Undo did not reconnect');
  applyDesign(readDesignFile(file).design);afterLoad();check(!cargo.power,'A design file lost the disconnection');
  $('#computerDlg').close();renderComputers(true);$('[data-power="'+bat.id+'"] button').click();check(cargo.power,'Connect on the card did not restore power');
  // no battery at all: no power; Add a part → Battery fixes it
  cfg.comps=cfg.comps.filter(c=>!c.battery);structural();doReset();save();renderComputers(true);
  check(!cargo.power&&powerWhy()==='no battery on the drone','A drone with no battery still had power');
  check($('[data-power="none"]').innerText.includes('No battery'),'No-battery card missing');
  $('#addPart').click();check(!!$('#addPartDlg [data-add="battery"]'),'Add a part has no Battery');
  $('#addPartDlg [data-add="battery"]').click();await wait(50);$('.place-opt').click();doReset();
  check(cargo.power&&batteryParts(cfg.comps).length===1,'Adding a battery did not power the drone');
  return 'Power: disconnect cuts it (motors, boards, status), undo and design files keep it, no battery = no power, Add a part → Battery restores it';
 }));
 console.log(await page.evaluate(async()=>{
  const check=(v,m)=>{if(!v)throw Error(m);};
  const L=boardLinks(computers());check(L.length===1&&L[0].pi,'Expected one flight controller ↔ Pi link');
  const card=$('[data-link="'+L[0].b.id+'"]');check(card&&card.innerText.includes('↔'),'Link card missing');card.click();
  const t=$('#computerDlg').innerText;check(/GPIO 1 \(TX\) → .*GPIO 15 \(RX\)/.test(t)&&/GPIO 14 \(TX\) → .*GPIO 3 \(RX\)/.test(t)&&/navigation:/.test(t),'Link detail lacks the wires or what it carries');
  check(!!$('#computerDlg [aria-label$="flight link serial port"]'),'Pi serial port setting not in the link detail');
  $('#computerDlg').close();return 'Board links: the flight controller ↔ Pi card shows both ends, the wires, what it carries and the Pi port';
 }));
 await page.reload({waitUntil:'networkidle'});await page.waitForFunction(()=>fleet.ready&&brt.ready);
 assert(await page.evaluate(()=>cargo.power&&batteryParts(cfg.comps).length===1),'Reload lost the added battery');
 assert.deepStrictEqual(errors,[]);console.log('Reload keeps it; no page errors');
}finally{if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
