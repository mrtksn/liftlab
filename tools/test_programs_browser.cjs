#!/usr/bin/env node
'use strict';
// Programs (js/programs.js, runner/fc/prog_core.c; docs/topic-bus.md) in the page: written in the Formula editor
// (+ New program), each on a board of its own; a sensor board's program filters its barometer, a Pi's program reads
// that through the flight controller with the flight core's state; what they publish is what their code says;
// a header that doesn't fit, or code that doesn't compile, is refused with why; editing the code reloads it in flight,
// changing the header restarts; undo, delete, design files and reload.
//   node tools/test_programs_browser.cjs
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
 const page=await browser.newPage({viewport:{width:1400,height:950}}),errors=[];
 page.on('pageerror',e=>{errors.push(e.stack);console.error(e.stack);});
 await page.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());
 await page.goto('http://127.0.0.1:'+server.address().port,{waitUntil:'networkidle'});
 await page.waitForFunction(()=>fleet.ready&&brt.ready);
 // the page's helpers for this test: fill a program card and press a button on it
 await page.evaluate(()=>{
  window.T={wait:ms=>new Promise(r=>setTimeout(r,ms)),check:(v,m)=>{if(!v)throw Error(m);},
   card:()=>document.querySelector('#formulaActive .prog-card'),
   set:(label,v)=>{const x=T.card().querySelector(`[aria-label="${label}"]`);x.value=v;x.dispatchEvent(new Event(x.tagName==='SELECT'?'change':'input',{bubbles:true}));},
   read:(topic,on=true)=>{const x=T.card().querySelector(`input[type=checkbox][value="${topic}"]`);T.check(x,'No checkbox for '+topic);x.checked=on;x.dispatchEvent(new Event('change',{bubbles:true}));},
   code:src=>{const ta=T.card().querySelector('textarea');ta.value=src;ta.dispatchEvent(new Event('input',{bubbles:true}));},
   press:re=>{const b=[...T.card().querySelectorAll('button')].find(b=>re.test(b.textContent));T.check(b,'No button '+re);b.click();},
   err:()=>T.card().querySelector('.law-err').textContent,
   topic:(b,n)=>busTopics(brt.inst.get(b.id)).find(t=>t.name===n)};
 });
 console.log(await page.evaluate(async()=>{
  const {wait,check,set,read,code,press,err,topic}=T;
  // a sensor board: a second ESP32 with no duties and the barometer wired to it
  const D=JSON.parse(JSON.stringify(computers()));D.boards.push({id:9,kind:'esp32',name:'Sensor board',tasks:[]});setComputers(D,'test');
  const baro=sensorsOf('baro')[0];editWiring(w=>{w.parts[baro.id]={...w.parts[baro.id],board:9};},baro.id);
  running=true;UI_PANELS.editor.select('form');openFormulaEditor();await wait(100);
  // its program, from + New program
  $('#progNew').click();await wait(50);
  check(/function program1\(st, inp, dt\)/.test(T.card().querySelector('textarea').value),'No template code');
  set('Program name','altFilter');set('Board','9');set('When it runs','change');read('sensor.baro');set('Topic it writes (after user.)','alt');set('Its fields','height rate');
  check(T.card().querySelector('.prog-shape').textContent.includes('inp.baro.height')&&T.card().querySelector('.prog-shape').textContent.includes('returns { height, rate }'),'The shape box does not say what the code gets and returns');
  code('function altFilter(st, inp, dt) {\n  if (st.h == null) st.h = inp.baro.height;\n  const h = st.h + 0.2 * (inp.baro.height - st.h), rate = (h - st.h) / dt;\n  st.h = h;\n  return { height: h, rate };\n}');
  press(/Add program/);await wait(50);check(err()==='','Adding refused: '+err());
  check(programs().length===1&&programs()[0].board===9,'Not added to the design');
  check(document.querySelector('[data-board="9"]').textContent.includes('Programs: altFilter'),'The board card does not list its program');
  // the Pi's program, every 100 ms, on the sensor board's topic (relayed) and the flight core's state
  $('#progNew').click();await wait(50);const pi=boardOf('nav');
  set('Program name','climbWatch');set('Board',String(pi.id));set('When it runs','every');set('Every (ms)','100');read('user.alt');read('fc.state');
  set('Topic it writes (after user.)','climb');set('Its fields','climbing armed maxRate');
  code('function climbWatch(st, inp, dt) {\n  if (st.max == null) st.max = 0;\n  st.max = Math.max(st.max, inp.alt.rate);\n  return { climbing: inp.alt.rate > 0.2 ? 1 : 0, armed: inp.state.state === 1 ? 1 : 0, maxRate: st.max };\n}');
  press(/Add program/);await wait(50);check(err()==='','Adding the Pi program refused: '+err());
  for(let i=0;i<120&&!(brt.navOut&&brt.navOut.fly&&S.p[2]>1);i++)await wait(50);
  await wait(1500);
  check(!brt.progErr.size,'Programs not running: '+JSON.stringify([...brt.progErr]));
  const sb=computers().boards.find(b=>b.id===9),alt=topic(sb,'user.alt'),altC=topic(boardOf('core'),'user.alt'),altP=topic(pi,'user.alt'),climb=topic(pi,'user.climb');
  check(alt&&!alt.mirror&&alt.got>20,'altFilter does not publish user.alt on the sensor board');
  check(altC&&altC.mirror&&altP&&altP.mirror&&altP.got>5,'user.alt is not relayed to the Pi');
  check(climb&&!climb.mirror&&climb.got>=10,'climbWatch does not publish every 100 ms: '+JSON.stringify(climb));
  const st=progStats();check(st.get(programs()[0].id).runs===alt.got&&st.get(programs()[1].id).runs===climb.got,'Run counts disagree with publishes');
  const fcs=topic(boardOf('core'),'fc.state');check(climb.vals[1]===(fcs.vals[0]===1?1:0),'climbWatch read the flight core\'s state wrong: '+climb.vals+' / '+fcs.vals[0]);
  check(climb.vals[2]>=altP.vals[1]-1e-6,'Its memory (the largest rate) was not kept: '+climb.vals);
  check(Math.abs(alt.vals[0]-topic(sb,'sensor.baro').vals[0])<0.6,'altFilter\'s height is not near the barometer\'s');
  return 'Two programs on two boards: the sensor board filters its barometer, the Pi watches it through the flight controller with the flight core\'s state';
 }));
 console.log(await page.evaluate(async()=>{
  const {wait,check,set,read,code,press,err}=T;
  // what's refused, with why
  $('#progNew').click();await wait(50);
  set('Program name','altFilter');check(/Another program or app is called/.test(err()),'A taken name not refused: '+err());
  set('Program name','attitudeControl');check(/taken by a formula/.test(err()),'A formula\'s name not refused: '+err());
  set('Program name','bad name');check(/must be a word/.test(err()),'A bad name not refused');
  set('Program name','p3');set('Topic it writes (after user.)','alt');check(/Another program or app writes user\.alt/.test(err()),'A topic another program writes not refused: '+err());
  set('Topic it writes (after user.)','p3');set('Its fields','a[0]');check(/fields/.test(err()),'A bad layout not refused');
  set('Its fields','x');set('When it runs','change');check(/runs when a topic it reads changes/.test(err()),'On a change with nothing read not refused');
  read('fc.height');code('function p3(st, inp, dt) { return { y: 1 }; }');press(/Add program/);await wait(50);
  check(/doesn't compile/.test(err())&&programs().length===2,'Code returning the wrong fields was added: '+err());
  press(/Discard/);await wait(50);check(programs().length===2&&!String(COMP.formula).startsWith('prog:p'),'Discard');
  // editing the code reloads it in flight; the header restarts the flight
  showProgram(programs()[1].id);await wait(50);const t0=S.t;
  code(T.card().querySelector('textarea').value.replace('> 0.2','> 0.5'));press(/^Apply$/);await wait(400);
  check(S.t>t0&&RN.log.some(e=>/Pi Zero: (self-tests passed|loaded the new program)/.test(e.msg)),'A code edit should stage the Pi\'s program in flight: '+RN.log.slice(0,3).map(e=>e.msg));
  check(programs()[1].src.includes('> 0.5'),'Code edit not applied');
  const t1=S.t;set('Every (ms)','200');press(/^Apply$/);await wait(100);check(S.t<t1,'A header change should restart the flight');
  check(programs()[1].every===0.2,'Header not applied');
  // undo, delete, files, reload
  undoStep();check(programs()[1].every===0.1,'Undo did not restore the header');redoStep();check(programs()[1].every===0.2,'Redo');
  const file=JSON.stringify({format:FILE_FORMAT,version:1,name:'t',design:JSON.parse(designSnap())});
  showProgram(programs()[0].id);await wait(50);press(/Delete/);press(/Click again/);await wait(100);
  check(programs().length===1,'Delete: '+programs().map(p=>p.name));check(!document.querySelector('[data-board="9"]').textContent.includes('Programs:'),'Delete: the board card still lists it: '+document.querySelector('[data-board="9"]').textContent);
  applyDesign(readDesignFile(file).design);afterLoad();check(programs().length===2&&programs()[0].name==='altFilter','A design file lost its programs');
  return 'Refused with why (names, topics, layouts, triggers, code); code edits reload in flight, header edits restart; undo, delete and design files';
 }));
 await page.reload({waitUntil:'networkidle'});await page.waitForFunction(()=>fleet.ready&&brt.ready);
 console.log(await page.evaluate(async()=>{
  const wait=ms=>new Promise(r=>setTimeout(r,ms)),check=(v,m)=>{if(!v)throw Error(m);};   // (a fresh page: no T)
  check(programs().length===2&&programs()[1].every===0.2,'Reload lost the programs');
  running=true;await wait(1500);
  check(!brt.progErr.size&&[...progStats().values()].every(s=>s.ok&&s.runs>0),'Programs not running after a reload: '+JSON.stringify([...brt.progErr]));
  return 'After a reload they are there and running';
 }));
 assert.deepStrictEqual(errors,[]);console.log('No page errors');
}finally{if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
