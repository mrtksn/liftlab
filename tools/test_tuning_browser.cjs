#!/usr/bin/env node
'use strict';
// Airframe → Tuning in the page: a slider moved in flight reaches the flight computers through their loading
// steps (and the drone keeps flying), the gains really fly (a tuning the prediction calls unstable flips it), undo,
// reset, design files, reload and each drone keeping its own tuning.
//   node tools/test_tuning_browser.cjs
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
 await page.waitForFunction(()=>fleet.ready && brt.ready && RN.act);
 // Hover in flight, then move the roll & pitch response: every board stages and swaps, nothing crashes.
 console.log(await page.evaluate(async()=>{
  const check=(v,m)=>{if(!v)throw Error(m);},wait=ms=>new Promise(r=>setTimeout(r,ms)),set=(id,v)=>{const n=document.getElementById(id);n.value=String(v);n.dispatchEvent(new Event('input',{bubbles:true}));};
  running=true;for(let i=0;i<100&&S.t<4;i++)await wait(50);
  check(tuneIsDefault()&&$('#tuneState').textContent==='defaults','Not on the default tuning at start');
  check(brt.srcs.get(boardOf('core').id).includes(JSON.stringify(TUNE_DEFAULTS)),'Flight controller not flying the default tuning');
  const swapsBefore=RN.log.filter(e=>/flies the new program now/.test(e.msg)).length;
  set('tune-rp-hz-n',2.2);
  check(Math.abs(cfg.tuning.att.kR[0]-(2*Math.PI*2.2)**2)<1e-6&&cfg.tuning.att.kR[1]===cfg.tuning.att.kR[0]&&cfg.tuning.att.kR[2]===40,'Response did not set roll and pitch kR only');
  for(let i=0;i<120&&RN.log.filter(e=>/flies the new program now/.test(e.msg)).length<swapsBefore+2;i++)await wait(50);
  const swapped=RN.log.filter(e=>/flies the new program now/.test(e.msg)).length-swapsBefore;
  check(swapped>=2,'Boards did not swap in the new program: '+RN.log.slice(0,6).map(e=>e.msg).join(' | '));
  for(const b of computers().boards.filter(b=>b.tasks.includes('core')||b.tasks.includes('nav')))check(brt.srcs.get(b.id)===boardSrcKey(b.tasks,rnSources())+'\u0002'+progKey(b)&&brt.srcs.get(b.id).includes(JSON.stringify(cfg.tuning)),b.name+' is not on the new tuning');
  check(!S.crashed&&FC_STATES[brt.fcState]==='armed','A stable retune upset the flight: '+(S.crashed||FC_STATES[brt.fcState]));
  check(editedFor(boardOf('core').tasks).some(x=>/tuning/.test(x)),'A real board would not be sent the tuning');
  return 'In flight: the new gains staged, checked and swapped on every board ('+swapped+' swaps); still flying';
 }));
 // A tuning predicted unstable really flips the simulated drone: the gains fly.
 console.log(await page.evaluate(async()=>{
  const check=(v,m)=>{if(!v)throw Error(m);},wait=ms=>new Promise(r=>setTimeout(r,ms)),set=(id,v)=>{const n=document.getElementById(id);n.value=String(v);n.dispatchEvent(new Event('input',{bubbles:true}));};
  $('#tune-rp-fold').open=true;set('tune-rp-hz-n',4);set('tune-rp-zeta-n',0.2);
  check(/Won't settle/.test($('#tune-rp-fold .tune-verdict').textContent),'Prediction did not warn: '+$('#tune-rp-fold .tune-verdict').textContent);
  for(let i=0;i<200&&!S.crashed;i++)await wait(50);
  check(S.crashed,'The unstable tuning flew without trouble: the gains may not reach the controller');
  // undo steps back through the changes; reset restores the defaults exactly
  const k=()=>cfg.tuning.att.kR[0];undoStep();check(k()===(2*Math.PI*4)**2,'Undo did not step back damping first');undoStep();check(Math.abs(k()-(2*Math.PI*2.2)**2)<1e-6,'Undo did not step back the response');
  redoStep();check(k()===(2*Math.PI*4)**2,'Redo');
  $('#tuneReset').click();check(tuneIsDefault()&&$('#tuneReset').disabled,'Reset to defaults');undoStep();check(!tuneIsDefault(),'Undo of reset');
  // design files keep it; a file from before tuning flies on the defaults
  const file=JSON.stringify({format:FILE_FORMAT,version:1,name:'t',design:JSON.parse(designSnap())}),old=JSON.parse(file);delete old.design.tuning;
  applyDesign(readDesignFile(JSON.stringify(old)).design);afterLoad();check(tuneIsDefault(),'An old design file kept a tuning');
  applyDesign(readDesignFile(file).design);afterLoad();check(k()===(2*Math.PI*4)**2&&$('#tune-rp-hz-n').value==='4','A design file lost its tuning');
  set('tune-yaw-hz-n',1.5);running=false;doReset();
  return 'Unstable gains flip the drone as predicted; undo/redo/reset; design files keep the tuning, old ones get the defaults';
 }));
 const kept=await page.evaluate(()=>JSON.stringify(cfg.tuning));
 await page.reload({waitUntil:'networkidle'});await page.waitForFunction(()=>fleet.ready&&brt.ready&&RN.act);
 console.log(await page.evaluate(kept=>{
  const check=(v,m)=>{if(!v)throw Error(m);};
  check(JSON.stringify(cfg.tuning)===kept&&$('#tune-yaw-hz-n').value==='1.5'&&brt.srcs.get(boardOf('core').id).includes(kept),'Reload lost the tuning');
  // another drone starts on the defaults; each keeps its own, and the fields follow selection
  const a=fleet.selected,b=fleetCreate('quadx');check(tuneIsDefault()&&$('#tune-yaw-hz-n').value==='1.01','A new drone did not start on the defaults');
  const n=$('#tune-pos-hz-n');n.value='0.5';n.dispatchEvent(new Event('input',{bubbles:true}));
  check(Math.abs(b.state.cfg.tuning.pos.kp-(Math.PI)**2)<1e-9&&a.state.cfg.tuning.pos.kp===4,'Position tuning leaked between drones');
  fleetSelect(a.id);check($('#tune-yaw-hz-n').value==='1.5'&&$('#tune-pos-hz-n').value==='0.32','Fields did not follow selection');
  return 'Reload keeps it and flies it; each drone keeps its own tuning and the fields follow selection';
 },kept));
 assert.deepStrictEqual(errors,[]);console.log('No page errors');
}finally{if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
