#!/usr/bin/env node
'use strict';
// The data bus in the simulator (docs/topic-bus.md): the simulated boards publish while flying, the Pi's copies follow
// the flight core's topics at the rates asked for and as old as the link makes them (and the other way for the
// navigation), the state travels on change, a program's own topic can be published and copied, and Computers →
// Live data shows it all and keeps it current.
//   node tools/test_bus_browser.cjs
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
  running=true;for(let i=0;i<200&&!(brt.navOut&&brt.navOut.fly&&S.p[2]>1);i++)await wait(50);
  check(brt.navOut&&brt.navOut.fly,'Did not take off');
  const core=boardOf('core'),pi=boardOf('nav'),cw=brt.inst.get(core.id),pw=brt.inst.get(pi.id);
  const topics=w=>Object.fromEntries(busTopics(w).map(t=>[(t.mirror?'↓':'')+t.name,t]));
  const c0=topics(cw),p0=topics(pw);
  for(const n of ['fc.state','fc.attitude','fc.imu','fc.height','fc.output','fc.torque','cmd.pilot'])check(c0[n]&&c0[n].age>=0&&c0[n].age<0.01,'Flight core does not publish '+n);
  for(const n of ['nav.estimate','nav.setpoint','nav.command'])check(p0[n]&&p0[n].age>=0&&p0[n].age<0.02,'Navigation does not publish '+n);
  check(Math.abs(c0['fc.attitude'].vals[0]-est.q[0])<1e-6&&Math.abs(c0['fc.attitude'].vals[3]-est.q[3])<1e-6,'fc.attitude is not the flight core\'s attitude');
  check(c0['fc.state'].vals[0]===1,'fc.state not armed while flying');
  // copies: values as published a moment earlier, aged by the link, at the rates asked for
  const g0={att:p0['↓fc.attitude'].got,h:p0['↓fc.height'].got,st:p0['↓fc.state'].got,ne:c0['↓nav.estimate'].got},t0=brt.t;
  await wait(1500);
  const c1=topics(cw),p1=topics(pw),dt=brt.t-t0;
  check(dt>0.8,'Simulation barely advanced: '+dt);
  const rate=(a,b)=>(a-b)/dt;
  check(Math.abs(rate(p1['↓fc.attitude'].got,g0.att)-50)<8,'fc.attitude copies not ~50/s: '+rate(p1['↓fc.attitude'].got,g0.att));
  check(Math.abs(rate(p1['↓fc.height'].got,g0.h)-20)<5,'fc.height copies not ~20/s: '+rate(p1['↓fc.height'].got,g0.h));
  check(rate(p1['↓fc.state'].got,g0.st)<10,'fc.state is not on change: '+rate(p1['↓fc.state'].got,g0.st)+'/s');
  check(Math.abs(rate(c1['↓nav.estimate'].got,g0.ne)-20)<5,'nav.estimate copies to the flight core not ~20/s');
  check(p1['↓fc.attitude'].age>=0&&p1['↓fc.attitude'].age<0.03,'A copy is as old as when it was sent plus since it came (under its 20 ms period and the link): '+p1['↓fc.attitude'].age);
  check(p1['↓fc.state'].vals.every((v,i)=>v===c1['fc.state'].vals[i]),'fc.state copy differs from the flight core\'s');
  // a program's own topic: published on the Pi, asked for by the flight controller's board, copied over
  const put=(w,name,vals)=>{const b=new TextEncoder().encode(name+'\0');new Uint8Array(w.memory.buffer,w.txt_ptr(),b.length).set(b);new Float32Array(w.memory.buffer,w.fr_ptr(),vals.length).set(vals);return w.bus_put(vals.length);};
  check(put(pw,'user.camera.photo',[7,1.5,2])===0,'Could not publish a program\'s topic');
  check(busWant(cw,'user.camera.photo',3,0)>=0,'Could not ask for it');
  await wait(800);
  const ph=topics(cw)['↓user.camera.photo'];check(ph&&ph.vals.join()==='7,1.5,2','The program\'s topic did not reach the other board: '+JSON.stringify(ph));
  check(put(cw,'fc.attitude',[1,2,3,4,5,6,7])===-1&&put(cw,'fc.attitude',[1,2,3])===-1,'A program could overwrite the flight core\'s topic');
  return 'Flight core and navigation publish; copies follow at 50/s and 20/s, on change for the state, aged by the link; a program\'s own topic travels';
 }));
 console.log(await page.evaluate(async()=>{
  const check=(v,m)=>{if(!v)throw Error(m);},wait=ms=>new Promise(r=>setTimeout(r,ms));
  UI_PANELS.editor.select('form');renderComputers(true);$('#busOpen').click();await wait(700);
  const cards=[...document.querySelectorAll('[data-bus-board]')];check(cards.length===2,'Live data: expected two board cards, got '+cards.length);
  const row=(b,name,mirror)=>[...document.querySelectorAll(`[data-bus-board="${b}"] tr[data-topic="${name}"]`)].find(r=>r.textContent.includes(mirror?'↓':'●'));
  const core=boardOf('core'),pi=boardOf('nav'),r=row(pi.id,'fc.attitude',true);
  check(r&&r.children[1].textContent===core.name,'The Pi\'s copy does not say where it is from');
  const before=r.children[4].textContent;await wait(800);check(r.children[4].textContent!==before,'Live data does not refresh');
  check(/\/s$/.test(r.children[3].textContent),'No rate shown: '+r.children[3].textContent);
  check(r.children[4].title.includes('qw:'),'Values have no labels on hover');
  $('#computerDlg').close();
  return 'Live data: a card per board, copies say where they come from, refreshes, rates and labelled values';
 }));
 assert.deepStrictEqual(errors,[]);console.log('No page errors');
}finally{if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
