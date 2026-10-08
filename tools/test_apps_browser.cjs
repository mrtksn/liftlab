#!/usr/bin/env node
'use strict';
// Apps (js/apps.js, docs/apps.md) in the page: a second ESP32 set to run WebAssembly apps, with the barometer wired to
// it; a C app on it, written in the app manager, compiled in the browser (clang as WebAssembly) and run by its board's
// app host through prog_core.c; the Pi running formulas and apps together, its app reading the first one's topic
// through the flight controller with the flight core's state; printf in the app's log; a loop that never ends
// stopped, counted and the app started again while the drone flies on; code edits reloading in flight, header edits
// restarting; what's refused and why; a Python app on the Pi (not simulated); undo, delete, design files, reload.
//   node tools/test_apps_browser.cjs
// The compiler comes from the CDN (about 25 MB, once); APP_CLANG_DIR=<the @yowasp/clang package's directory> serves
// it from disk instead.
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
 const context=await browser.newContext({viewport:{width:1400,height:950}});
 if(process.env.APP_CLANG_DIR)await context.route(/cdn\.jsdelivr\.net\/npm\/@yowasp\/clang@[^/]+\/(.*)$/,r=>{const f=path.join(process.env.APP_CLANG_DIR,new URL(r.request().url()).pathname.replace(/^.*?@yowasp\/clang@[^/]+\//,''));
   r.fulfill({status:200,headers:{'Access-Control-Allow-Origin':'*','Content-Type':f.endsWith('.js')?'application/javascript':f.endsWith('.wasm')?'application/wasm':'application/octet-stream'},body:fs.readFileSync(f)});});
 await context.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());
 const page=await context.newPage(),errors=[];
 page.on('pageerror',e=>{errors.push(e.stack);console.error(e.stack);});
 await page.goto('http://127.0.0.1:'+server.address().port,{waitUntil:'networkidle'});
 await page.waitForFunction(()=>fleet.ready&&brt.ready);
 await page.evaluate(()=>{
  window.T={wait:ms=>new Promise(r=>setTimeout(r,ms)),check:(v,m)=>{if(!v)throw Error(m);},
   card:()=>document.querySelector('#appActive .app-card'),
   set:(label,v)=>{const x=T.card().querySelector(`[aria-label="${label}"]`);x.value=v;x.dispatchEvent(new Event(x.tagName==='SELECT'?'change':'input',{bubbles:true}));},
   read:(topic,on=true)=>{const x=T.card().querySelector(`input[type=checkbox][value="${topic}"]`);T.check(x,'No checkbox for '+topic);x.checked=on;x.dispatchEvent(new Event('change',{bubbles:true}));},
   code:src=>{const ta=T.card().querySelector('textarea');ta.value=src;ta.dispatchEvent(new Event('input',{bubbles:true}));},
   press:re=>{const b=[...T.card().querySelectorAll('button')].find(b=>re.test(b.textContent));T.check(b,'No button '+re);b.click();},
   applied:async(re=/Add app|^Apply$/)=>{T.press(re);for(let i=0;i<1200&&T.card().querySelector('button.primary').disabled;i++)await T.wait(100);await T.wait(100);},
   err:()=>T.card().querySelector('.law-err').textContent,out:()=>T.card().querySelector('.app-compiler').textContent,
   topic:(b,n)=>busTopics(brt.inst.get(b.id)).find(t=>t.name===n),
   board:id=>{openComputerView({kind:'board',id});return $('#computerDlgBody');}};
 });
 console.log(await page.evaluate(async()=>{
  const {wait,check,set,read,code,applied,err,out,topic,board}=T;
  // a second ESP32, the barometer wired to it; in its settings it is set to run WebAssembly apps
  const D=JSON.parse(JSON.stringify(computers()));D.boards.push({id:9,kind:'esp32',name:'App board',tasks:[]});setComputers(D,'test');
  const baro=sensorsOf('baro')[0];editWiring(w=>{w.parts[baro.id]={...w.parts[baro.id],board:9};},baro.id);
  running=true;
  let box=board(9),runs=box.querySelector('#bruns-9');check(runs&&runs.value==='formulas','The board\'s settings have no Runs choice');
  runs.value='wasm';runs.dispatchEvent(new Event('change'));await wait(50);
  check(boardRuns(computers().boards.find(b=>b.id===9))[0]==='wasm','Runs not saved');
  box=$('#computerDlgBody');check(/Apps on this board/.test(box.textContent),'An app board has no apps section');
  // the flight core's board can't stop running formulas while it has duties
  board(boardOf('core').id).querySelector('[id^=bruns-]').value='wasm';$('#computerDlgBody [id^=bruns-]').dispatchEvent(new Event('change'));await wait(50);
  check(/Move its duties/.test($('#computerDlgBody .board-runs-err').textContent)&&runsFormulas(boardOf('core')),'The flight controller was switched to apps with its duties on it');
  // + New app here: the app manager, a C app on this board
  board(9).querySelector('[data-new-app="9"]').click();await wait(50);
  check($('#appDlg').open&&T.card(),'The app manager did not open');
  check(/#include "liftlab.h"/.test(T.card().querySelector('textarea').value),'No C template');
  set('App name','altC');set('When it runs','change');read('sensor.baro');set('Topic it writes (after user.)','altc');set('Its fields','height rate n');
  const h=T.card().querySelector('.app-header').textContent;
  check(h.includes('struct { float height; } baro;')&&h.includes('struct output { float height; float rate; float n; };'),'liftlab.h does not say what the code gets: '+h);
  code('#include "liftlab.h"\nstatic float h0; static int n;\nvoid setup(void) { printf("altC up\\n"); }\nint step(const struct inputs *in, struct output *out, float dt) {\n  if (!n) h0 = in->baro.height;\n  float h = h0 + 0.2f * (in->baro.height - h0);\n  out->rate = (h - h0) / dt; h0 = h; out->height = h; out->n = ++n;\n  if (n % 50 == 0) printf("n=%d h=%.2f\\n", n, h);\n  return 1;\n}\n');
  await applied();check(err()==='','Adding refused: '+err()+' / '+out());
  check(/Compiled: [\d.]+ KB/.test(out()),'Not compiled: '+out());
  check(apps().length===1&&apps()[0].bin&&appBoard(apps()[0]).id===9,'Not added to the design on the board');
  check(document.querySelector('[data-board="9"]').textContent.includes('Apps: altC'),'The board card does not list its app');
  // the Pi runs formulas and WebAssembly apps; its app reads altC's topic (relayed) and the flight core's state
  const pi=boardOf('nav');box=board(pi.id);const wasmBox=box.querySelector('[data-runs="wasm"]');wasmBox.checked=true;wasmBox.dispatchEvent(new Event('change'));await wait(50);
  check(boardRuns(computers().boards.find(b=>b.id===pi.id)).join()==='formulas,wasm'&&boardOf('nav').id===pi.id,'The Pi does not run both');
  $('#computerDlgBody [data-new-app="'+pi.id+'"]').click();await wait(50);
  set('App name','climbC');set('When it runs','every');set('Every (ms)','100');read('user.altc');read('fc.state');set('Topic it writes (after user.)','climbc');set('Its fields','climbing armed maxRate');
  code('#include "liftlab.h"\nstatic float mx;\nint step(const struct inputs *in, struct output *out, float dt) {\n  if (in->altc.rate > mx) mx = in->altc.rate;\n  out->climbing = in->altc.rate > 0.2f; out->armed = in->state.state == 1; out->maxRate = mx;\n  return 1;\n}\n');
  await applied();check(err()==='','Adding the Pi app refused: '+err()+' / '+out());
  for(let i=0;i<120&&!(brt.navOut&&brt.navOut.fly&&S.p[2]>1);i++)await wait(50);
  await wait(1500);
  check(!brt.appErr.size,'Apps not running: '+JSON.stringify([...brt.appErr]));
  const ab=computers().boards.find(b=>b.id===9),alt=topic(ab,'user.altc'),altP=topic(pi,'user.altc'),climb=topic(pi,'user.climbc'),st=appStats();
  check(alt&&!alt.mirror&&alt.got>20&&alt.vals[2]===st.get(apps()[0].id).runs,'altC does not publish on its board: '+JSON.stringify(alt));
  check(Math.abs(alt.vals[0]-topic(ab,'sensor.baro').vals[0])<0.6,'altC\'s height is not near the barometer\'s');
  check(altP&&altP.mirror&&climb&&!climb.mirror&&climb.got>=10&&st.get(apps()[1].id).runs===climb.got,'climbC does not publish every 100 ms from the relayed topic: '+JSON.stringify(climb));
  check(climb.vals[1]===(topic(boardOf('core'),'fc.state').vals[0]===1?1:0)&&climb.vals[2]>=altP.vals[1]-1e-6,'climbC computed wrong: '+climb.vals);
  const L=brt.appLog.get(apps()[0].id)||[];check(L[0]&&L[0].s==='altC up'&&L.some(l=>/^n=50 h=/.test(l.s)),'printf did not reach the log: '+JSON.stringify(L.slice(0,3)));
  return 'A C app on an ESP32 app board and one on the Pi beside its duties: compiled here, run by prog_core, relayed through the flight controller, printf in the log';
 }));
 console.log(await page.evaluate(async()=>{
  const {wait,check,set,read,code,applied,err,out,topic}=T;
  // a loop that never ends: stopped, counted, started again; the drone flies on
  showApp(apps()[1].id);await wait(50);const t0=S.t,z0=S.p[2];
  code(T.card().querySelector('textarea').value.replace('return 1;\n}','if (in->altc.n > 0) { volatile int k = 0; while (in->altc.n > 0) k++; }\n  return 1;\n}'));
  await applied(/^Apply$/);check(err()==='','Apply refused: '+err());
  check(S.t>t0,'A code edit should load in flight, not restart');
  const t1=performance.now();await wait(1200);check(performance.now()-t1<4000,'The page hung');
  const s=appStats().get(apps()[1].id);check(s.fails>=5&&/ran too long/.test(s.why)&&s.restarts===s.fails,'The runaway loop was not stopped and counted: '+JSON.stringify(s));
  check(/a loop ran too long; starting it again/.test((brt.appLog.get(apps()[1].id)||[]).map(l=>l.s).join('\n')),'Not in its log');
  check(brt.pilot.phase!=='landed'&&S.p[2]>0.5,'The drone did not fly on: '+S.p[2]);
  code(T.card().querySelector('textarea').value.replace(/if \(in->altc\.n > 0\) \{[^\n]*\}\n  /,''));await applied(/^Apply$/);
  const f0=appStats().get(apps()[1].id).fails;await wait(600);check(appStats().get(apps()[1].id).fails===f0,'The fix did not load');
  // the header restarts the flight
  const t2=S.t;set('Every (ms)','200');await applied(/^Apply$/);check(S.t<t2&&apps()[1].every===0.2,'A header change should restart the flight');
  // what's refused, with why
  $('#appNew').click();await wait(50);
  set('App name','altC');check(/Another app or program is called/.test(err()),'A taken name: '+err());
  set('App name','p3');set('Topic it writes (after user.)','altc');check(/writes user\.altc/.test(err()),'A taken topic: '+err());
  set('Topic it writes (after user.)','p3');set('Its fields','int x');check(/can't be a name in C/.test(err()),'A C word as a field: '+err());
  set('Its fields','x');set('When it runs','every');code('#include "liftlab.h"\nint step(const struct inputs *in, struct output *out, float dt) {\n  out->y = 1;\n  return 1;\n}\n');
  await applied(/Add app/);check(/doesn't compile/.test(err())&&/app\.c:3:\d+: error: no member named 'y'/.test(out())&&apps().length===2,'A compile error not shown with its line: '+out());
  code('#include "liftlab.h"\nint step(const struct inputs *in, struct output *out, float dt) {\n  FILE *f = fopen("x", "r"); out->x = f != 0;\n  return 1;\n}\n');
  await applied(/Add app/);check(/which the boards don't offer/.test(out())&&apps().length===2,'A module asking for what boards lack was added: '+out());
  T.press(/Discard/);await wait(50);
  // a formula program can't go on a board that runs apps only
  const p={id:'px',name:'px',board:9,every:0.1,on:'',reads:[],writes:{topic:'user.px',layout:'v'},src:''};check(/runs apps, not formulas/.test(programProblem(p,[p])),'A program on an app board');
  // a Python app on the Pi (native apps): not simulated
  const pi=boardOf('nav');let box=T.board(pi.id),nat=box.querySelector('[data-runs="native"]');nat.checked=true;nat.dispatchEvent(new Event('change'));await wait(50);
  $('#appNewKind').value='python';newApp('python',pi.id);await wait(50);
  check(/def step\(inp, out, dt\)/.test(T.card().querySelector('textarea').value),'No Python template');
  set('App name','camPy');set('When it runs','every');set('Topic it writes (after user.)','cam');await applied(/Add app/);check(err()==='','Python app refused: '+err());
  await wait(300);check(/not simulated/.test(brt.appErr.get(apps()[2].id)),'A Python app should say it is not simulated: '+brt.appErr.get(apps()[2].id));
  check(appBoard(apps()[2]).id===pi.id,'Not on the Pi');
  // the install dialog lists the app board's apps
  openInstall(computers().boards.find(b=>b.id===9));await wait(50);check($('#installBody [data-app-files]')&&/app-board firmware/.test($('#installBody').textContent),'Install does not show the apps');$('#installDlg').close();
  // undo, delete, files
  undoStep();check(apps().length===2,'Undo did not remove the Python app');redoStep();check(apps().length===3,'Redo');
  const file=JSON.stringify({format:FILE_FORMAT,version:1,name:'t',design:JSON.parse(designSnap())});
  openAppManager(apps()[2].id);await wait(50);T.press(/Delete/);T.press(/Click again/);await wait(100);
  check(apps().length===2&&!(computers().boards.find(b=>b.id===pi.id).apps||[]).includes('camPy'),'Delete');
  applyDesign(readDesignFile(file).design);afterLoad();check(apps().length===3&&apps()[0].bin,'A design file lost its apps');
  return 'Runaway loops stopped and restarted while it flies; code reloads in flight, the header restarts; refused with why (names, topics, C words, compile errors with lines, what boards lack, programs on app boards); Python not simulated; install, undo, delete, files';
 }));
 await page.reload({waitUntil:'networkidle'});await page.waitForFunction(()=>fleet.ready&&brt.ready);
 console.log(await page.evaluate(async()=>{
  const wait=ms=>new Promise(r=>setTimeout(r,ms)),check=(v,m)=>{if(!v)throw Error(m);};
  check(apps().length===3&&apps()[1].every===0.2,'Reload lost the apps');
  running=true;await wait(1500);
  const s=appStats();check(s.get(apps()[0].id).ok&&s.get(apps()[0].id).runs>0&&s.get(apps()[1].id).runs>0,'Apps not running after a reload (no compiling needed): '+JSON.stringify([...brt.appErr]));
  return 'After a reload they are there and running, from their saved modules';
 }));
 assert.deepStrictEqual(errors,[]);console.log('No page errors');
}finally{if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
