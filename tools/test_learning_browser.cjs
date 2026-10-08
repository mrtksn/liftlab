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
   await drive(5);check(!learn.view.keep,'Adaptation should start off');check(!S.crashed,'Takeoff');
   boardsLearnCmd('keepOn');await drive(4);
   const lw=brt.inst.get(boardOf('learn').id);check(lw.learn_exc()===0,'Passive adaptation injected motor test signals');
   boardsLearnCmd('keepOff');boardsLearnCmd('calibrate');await drive(35,()=>learn.view?.haveFit&&!learn.view.cal);
   check(learn.view.haveFit&&!learn.view.cal,'Calibration did not finish: '+learn.msg);check(!learn.view.keep,'Calibration forced adaptation on');
   console.log('Calibration validated; adaptation preference retained');
   const before=JSON.stringify(cfg.tuning);AGENT_TOOLS.autotune.run({action:'measure_attitude'});check(atBusy(),'Autotune did not start: '+brt.autotune.message);
   await drive(180,()=>!atBusy());
   console.log('Attitude measurement:',brt.autotune.phase,brt.autotune.message);
   check(brt.autotune.phase==='review','No measured recommendation: '+brt.autotune.message);
   const measured=JSON.parse(JSON.stringify(brt.autotune.recommendation));
   const measuredModels=JSON.parse(JSON.stringify(brt.autotune.models));
   check(JSON.stringify(cfg.tuning)===before,'Measurement changed saved gains');
   AGENT_TOOLS.autotune.run({action:'apply_verify'});check(brt.tuneTrial,'No provisional tuning');save();check(JSON.stringify(cfg.tuning)===before,'Trial gains leaked into the design');
   await drive(180,()=>!atBusy());
   console.log('Attitude verification:',brt.autotune.phase,brt.autotune.message);
   check(brt.autotune.phase==='done','Verification failed: '+brt.autotune.message);check(!brt.tuneTrial,'Trial override retained');
   const after=JSON.stringify(cfg.tuning);check(after!==before,'Autotune made no tuning change');
   AGENT_TOOLS.autotune.run({action:'measure_position'});check(atBusy(),'Position tuning did not start: '+brt.autotune.message);
   await drive(150,()=>!atBusy());console.log('Position measurement:',brt.autotune.phase,brt.autotune.message);
   check(brt.autotune.phase==='review','No position recommendation: '+brt.autotune.message);
   AGENT_TOOLS.autotune.run({action:'apply_verify'});await drive(150,()=>!atBusy());check(brt.autotune.phase==='done','Position verification failed: '+brt.autotune.message);
   check(!S.crashed,'Flight crashed during tuning');
   undoStep();check(JSON.stringify(cfg.tuning)===after,'Undo did not restore attitude-only tuning');
   undoStep();check(JSON.stringify(cfg.tuning)===before,'Undo did not restore original tuning');
   await drive(5);boardsLearnCmd('keepOn');atStart('att');check(atBusy()&&!learn.view.keep,'Measurement did not pause adaptation: '+brt.autotune.message+' / '+atAvailable());
   await drive(.1);atStop();check(learn.view.keep&&!atBusy()&&!brt.tuneTrial&&JSON.stringify(cfg.tuning)===before,'Stop did not restore preference and gains');
   atStart('att');await drive(.1);brt.autotune.last-=.1;await drive(.01);check(!atBusy()&&/Telemetry interrupted/.test(brt.autotune.message),'Telemetry gap did not cancel');
   // Reuse the already measured original-gain proposal to isolate loader fallback from signal fitting.
   atStart('att');check(atBusy(),'Fallback fixture did not start');brt.autotune.phase='review';brt.autotune.recommendation=measured;brt.autotune.models=measuredModels;atApply();
   await drive(8,()=>brt.autotune.phase==='verify');check(brt.autotune.phase==='verify','Trial program did not become active');
   const coreId=boardOf('core').id,core=brt.inst.get(coreId),prior=brt.autotune.stageSlots[coreId];
   // WebAssembly exports are immutable; use a facade reporting the predecessor slot to the coordinator.
   brt.inst.set(coreId,{...core,host_phase:()=>prior*10});await drive(.02);brt.inst.set(coreId,core);
   check(!atBusy()&&!brt.tuneTrial&&/fell back/.test(brt.autotune.message)&&JSON.stringify(cfg.tuning)===before,'Fallback was approved as a verified tuning: '+brt.autotune.message);await drive(5);
   atStart('att');await drive(.1);const manual=tuneWithFeel(tuneOf(),'att',[0,1],{hz:1.55,zeta:1,integ:.4});setTuning(manual,'manual-regression');
   check(!atBusy()&&tuneSame(tuneOf(),manual),'Manual tuning was overwritten');undoStep();await drive(3);
   atStart('att');await drive(.1);const own=brt.autotune,owner=fleet.selected,other=fleetCreate('quadx');
   check(!brt.autotune,'Autotune state leaked to a new drone');fleetSelect(owner.id);check(brt.autotune===own,'Measurement lost its drone ownership');atStop();
   boardsLearnCmd('keepOff');const use=learn.view.useLearned;boardsLearnCmd('calibrate');await drive(1);boardsLearnCmd('stop');
   check(learn.view.useLearned===use&&!learn.view.cal,'Stopping calibration replaced the flown model');
   await drive(3);atStart('att');check(atBusy(),'Model-change fixture did not start');
   boardsLearnCmd(use?'useDesc':'useLearned');await drive(.05);
   check(!atBusy()&&/flown model changed/.test(brt.autotune.message),'Changing the accepted flight model did not cancel measurement');
   return 'Passive adaptation, calibration preference, measured attitude/position tuning, runtime trials, verification, undo, Stop, telemetry gaps, board fallback, model changes, manual edits, drone ownership and calibration cancellation passed';
  });
  console.log(result);assert.deepStrictEqual(errors,[]);
 } finally {if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
