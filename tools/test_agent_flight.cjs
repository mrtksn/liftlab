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
   const tool=(name,args={})=>AGENT_TOOLS[name].run(args,{});
   const initial=JSON.stringify(cfg.tuning);
   for(const patch of [{att:{kR:[1,2]}},{pos:{kp:Infinity}},{pos:{kd:-1}},{att:{wrong:[1,2,3]}}]) {
    let rejected=false;try{tool('set_tuning',patch);}catch(e){rejected=true;}
    check(rejected&&JSON.stringify(cfg.tuning)===initial,'Invalid gains changed design');
   }
   check(tool('get_tuning').gains.att.kR.length===3,'Gains missing');
   tool('set_tuning',{pos:{kp:4}});check(cfg.tuning.pos.kp===4,'Gain edit failed');undoStep();check(JSON.stringify(cfg.tuning)===initial,'Gain undo failed');
   let blocked=false;try{tool('autotune',{action:'measure_attitude'});}catch(e){blocked=true;}check(blocked,'Uncalibrated measurement accepted');
   blocked=false;try{tool('set_learning',{fly_on:'learned'});}catch(e){blocked=true;}check(blocked,'Unaccepted model accepted');
   tool('simulation',{action:'reset'});running=false;while(!brt.ready)await new Promise(r=>setTimeout(r,20));await drive(15,()=>brt.pilot.phase==='flying'&&S.t>5);check(brt.pilot.phase==='flying','Agent fixture failed to take off: '+JSON.stringify(agentState()));tool('simulation',{action:'calibrate'});await drive(35,()=>learn.view?.haveFit&&!learn.view.cal);
   check(learn.view.accepted,'Calibration not accepted: '+learn.msg+' '+JSON.stringify(learn.view));
   tool('autotune',{action:'measure_attitude'});check(atBusy(),'Agent measurement failed');await drive(.1);
   tool('set_tuning',{pos:{kp:4}});check(!atBusy()&&!brt.tuneTrial,'Manual edit failed to cancel');undoStep();await drive(3);
   tool('autotune',{action:'measure_attitude'});await drive(.1);tool('set_learning',{keep_learning:true});check(!atBusy()&&learnPrefs.keep,'Learning edit failed to cancel');
   tool('autotune',{action:'measure_attitude'});await drive(.1);tool('simulation',{action:'calibrate'});check(!atBusy(),'Calibration failed to cancel');tool('simulation',{action:'stop_calibration'});
   const runtime=tool('get_runtime');check(runtime.boards[0].runs.includes('formulas'),'Runtimes missing');
   const bus=tool('get_bus',{topic:'fc.attitude'});check(bus.catalog.length===1&&bus.boards.some(b=>b.topics.some(t=>t.name==='fc.attitude'&&t.vals.length===7)),'Live bus missing');
   const savedPrograms=cfg.programs,savedApps=cfg.apps;
   cfg.programs=[{id:'probe',name:'probe',board:boardOf('core').id,every:.1,on:'',reads:[],writes:{topic:'user.probe',layout:'v'},src:PROG_TEMPLATE('probe')}];
   cfg.apps=[{id:'pyprobe',kind:'python',name:'pyprobe',every:.1,on:'',reads:[],writes:{topic:'user.pyprobe',layout:'v'},src:APP_TEMPLATE.python('pyprobe')}];
   const source=tool('get_runtime',{kind:'program',id:'probe',offset:0,limit:20});check(source.source===cfg.programs[0].src.slice(0,20)&&source.next===20,'Source chunks failed');
   const python=tool('get_runtime',{kind:'app',id:'pyprobe'});check(python.interface.includes('def step')&&tool('get_runtime').apps[0].simulated===false,'Python boundary missing');
   cfg.programs=savedPrograms;cfg.apps=savedApps;
   check(agentSystem().includes('Measured autotune')&&agentState().tuning.gains,'Prompt/state stale');
   const owner=fleet.selected,other=fleetCreate('quadx');check(tool('autotune',{action:'status'}).phase==='idle','Autotune leaked');fleetSelect(owner.id);
   return 'Agent PID bounds/edit/undo, calibration prerequisites, autotune/learning cancellation, runtime/bus visibility and drone isolation passed';
  });
  console.log(result);assert.deepStrictEqual(errors,[]);
 } finally {if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
