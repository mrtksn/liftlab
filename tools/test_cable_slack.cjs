#!/usr/bin/env node
'use strict';
const fs=require('fs'),path=require('path'),http=require('http'),assert=require('assert');
const {chromium}=require(process.env.PLAYWRIGHT_PATH || '/Users/mrtksn/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const root=path.resolve(__dirname,'..');
const server=http.createServer((req,res)=>{
 const file=path.resolve(root,'.'+new URL(req.url,'http://localhost').pathname.replace(/^\/$/,'/index.html'));
 if(!file.startsWith(root+path.sep)){res.writeHead(403).end();return;}
 fs.readFile(file,(e,data)=>{if(e){res.writeHead(404).end();return;}res.setHeader('Content-Type',file.endsWith('.js')?'application/javascript':file.endsWith('.css')?'text/css':file.endsWith('.html')?'text/html':'application/octet-stream');res.end(data);});
});
(async()=>{await new Promise(r=>server.listen(0,'127.0.0.1',r));let browser;
 try {
  browser=await chromium.launch({executablePath:process.env.CHROME_PATH || '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
  const page=await browser.newPage(),errors=[];page.on('pageerror',e=>errors.push(e.message));
  page.on('console',m=>{if(m.type()==='log')console.log(m.text());});
  await page.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());
  await page.goto('http://127.0.0.1:'+server.address().port,{waitUntil:'networkidle'});if(errors.length)throw Error(errors.join('\n'));await page.waitForFunction(()=>typeof fleet!=='undefined'&&fleet.ready&&brt.ready);
  const result=await page.evaluate(async()=>{
   running=false;loadPreset('quadx');running=false;
   // Reliable position feedback for the successful workflow; poor-signal/motion cases must reject below.
   Object.assign(sensorsOf('fix')[0],fixDefaults('rtk'),{quality:'rtk'});envr.turb=0;doReset();running=false;
   while(!brt.ready) await new Promise(r=>setTimeout(r,20));
   const check=(x,m)=>{if(!x)throw Error(m);};
   const drive=async(s,until=()=>false)=>{const end=S.t+s;let i=0;while(S.t<end&&!until()) {if(i%20===0)pilotStep(.01);physStep();i++;if(i%4000===0)await new Promise(r=>setTimeout(r,0));}boardsReadViews(true);};
   loadPreset('cargo');running=false;const c=cfg.comps.find(c=>c.type==='hang');c.length=1.83;c.mass=1.06;doReset();running=false;
   while(!brt.ready)await new Promise(r=>setTimeout(r,20));
   check(Math.abs(model.m-truth.m)<1e-9,'Ground-supported bag changes rigid model');
   const physical=fcAirframe({cableLoads:true}),exported=fcAirframe();
   check(Math.abs(physical.m-truth.m)<1e-9&&Math.abs(exported.m-truth.m-c.mass)<1e-9,'Simulator/export load boundary');
   let st=pend.get(c.id),before=st.p.slice();c.length=2.5;reseatPend(c);
   check(nrm(sub(st.p,before))<1e-9&&st.p[2]>=payloadR(c),'Longer cable forced ball taut/underground');c.length=1.83;
   const original=JSON.stringify(model);
   // Reported case: fully known heavy bag remains on ground below the target height.
   await drive(8);check(S.p[2]>1&&brt.superView.mode===0&&!S.crashed,'Premature payload landing: '+JSON.stringify(agentState()));
   check(st.Tn===0&&knownCableLoad().every(x=>x===0),'Slack cable loads aircraft');
   check(JSON.stringify(model)===original,'Cable altered rigid inertia/CoG');
   check(Math.abs(st.p[2]-payloadR(c))<.005,'Bag left ground with slack cable');
   console.log('Heavy 1.06 kg / 1.83 m cable: airborne at',S.p[2].toFixed(2),'m with zero tension and normal supervisor');
   Object.assign(setpoint,{z:3});gsSet.x=null;
   let first=null;await drive(10,()=>{if(st.Tn>0){first={p:S.p.slice(),L:nrm(sub(st.p,add(S.p,m3v(qmat(S.q),posNow(c))))),load:knownCableLoad()};return true;}return false;});
   check(first&&first.L>=c.length-.005,'Cable failed to engage at full separation');
   await drive(.5);check(st.Tn>0&&knownCableLoad()[2]<0,'Taut supported cable missing weight compensation');
   check(/Unload the cable/.test(atAvailable()),'Loaded autotune admitted');
   check(AGENT_TOOLS.get_learning.run({}).paused_for_cable&&!learn.view.cal,'Learning load guard missing');
   const loadedStatus=AGENT_TOOLS.get_envelope.run({});check(loadedStatus.mass.known_cable_load[2]<0,'Agent cannot see supported load');
   // Fixed-input supervisor tests the external force/torque without counting payload twice.
   check(brt.superView.mode>0,'Heavy supported bag failed to reduce supervisor margin');
   c.known=false;check(knownCableLoad().every(x=>x===0),'Unknown load leaks into controller feedforward');c.known=true;
   cargo.off.add(c.id);check(knownCableLoad().every(x=>x===0),'Detached cable keeps controller load');cargo.off.delete(c.id);
   // A lighter bag should be picked up and remain airborne with the supported-load path.
   c.mass=.25;doReset();running=false;while(!brt.ready)await new Promise(r=>setTimeout(r,20));Object.assign(setpoint,{z:2.5});gsSet.x=null;
   await drive(15);st=pend.get(c.id);
   check(!S.crashed&&S.p[2]>2&&st.p[2]>payloadR(c)+.1&&st.Tn>0&&brt.superView.mode===0,'Normal bag pickup failed: '+JSON.stringify(agentState()));
   // Geometry, not just hub altitude: a sideways displacement can tighten the rope below its vertical length.
   S.p=[0,0,1];S.q=[1,0,0,0];const anchor=add(S.p,posNow(c));st.p=add(anchor,[c.length+.001,0,-.05]);st.Tn=c.mass*G;
   check(knownCableLoad()[2]<0&&S.p[2]<c.length,'Diagonal taut cable wrongly depends on altitude');
   st.p=add(anchor,[0,0,-.2]);st.Tn=c.mass*G;check(knownCableLoad().every(x=>x===0),'Slack geometry retains stale tension load');
   c.length=4;before=st.p.slice();reseatPend(c);check(nrm(sub(st.p,before))<1e-9,'Editing a slack rope moved its mass');
   doReset();running=false;check(knownCableLoad().every(x=>x===0),'Reset retained previous tension/load');
   return 'Slack/taut threshold, reported heavy bag, safe pickup, known/unknown/detached load, rigid inertia/CoG, static hardware export, diagonal tightening, edits and reset passed';

  });
  console.log(result);assert.deepStrictEqual(errors,[]);
 } finally {if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
