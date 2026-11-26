#!/usr/bin/env node
// Requires Playwright and Chrome. PLAYWRIGHT_PATH / CHROME_PATH may override local defaults.
// Tune → Rope hang on the simulator (the toy-drone rebuild held as on a rope; runner/fc/spin_core.c on the learning's
// board, as dfb_pi runs it):
//  - the spin test: its spin-up against a thrust step measured directly, stock motors and rotors 3.3× heavier; Apply
//    keeps it in the design (undo too), and Gains from the motors uses it;
//  - the lift test with props 30% grippier than their cards say (a speed-limited motor lifts less than 30% more): its hover throttle against the simulated truth,
//    the strength it finds, and Apply fitting the cards so that the controller's model lifts what they do.
'use strict';
const fs=require('fs'),path=require('path'),http=require('http'),assert=require('assert');
const {chromium}=require(process.env.PLAYWRIGHT_PATH || '/Users/mrtksn/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const root=path.resolve(__dirname,'..');
const server=http.createServer((req,res)=>{
  const url=new URL(req.url,'http://localhost'),file=path.resolve(root,'.'+decodeURIComponent(url.pathname==='/'?'/index.html':url.pathname));
  if(!file.startsWith(root+path.sep)){res.writeHead(403).end();return;}
  fs.readFile(file,(err,data)=>{if(err){res.writeHead(404).end();return;}res.setHeader('Cache-Control','no-store');
    res.setHeader('Content-Type',file.endsWith('.js')?'application/javascript':file.endsWith('.css')?'text/css':file.endsWith('.html')?'text/html':'application/octet-stream');res.end(data);});
});
(async()=>{await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));let browser;
try{
  browser=await chromium.launch({executablePath:process.env.CHROME_PATH || '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',headless:true,args:['--use-angle=swiftshader']});
  const page=await browser.newPage({viewport:{width:1440,height:1000}}),errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.route(/fonts\.google|goatcounter|gc\.zgo/,route=>route.abort());
  await page.goto('http://127.0.0.1:'+server.address().port,{waitUntil:'networkidle'});
  await page.waitForFunction(()=>typeof brt!=='undefined' && brt.ready && typeof renderRopeLift==='function');
  const result=await page.evaluate(async()=>{
    running=false;const out={};
    const assert=(ok,message)=>{if(!ok)throw new Error(message);};
    const step=s=>{for(let i=0;i<s/PDT;i++){if(i%20===0)pilotStep(.01);physStep();if(i%100===0){if(spt.phase==='run')sptPoll();if(rlt.phase==='run')rltPoll();}}};
    const fresh=async(inertia=1,strong=1)=>{loadPreset('toy1s');running=false;while(!brt.ready)await new Promise(r=>setTimeout(r,20));
      for(const c of actuators()){const mp=flightMotor(c);c.motorPhysics={...c.motorPhysics,inertia:mp.J*inertia};c.propPhysics={...c.propPhysics,ct:(c.propPhysics?.ct??0.1)*strong};}
      recomputeProps();await new Promise(r=>setTimeout(r,200));step(0.5);};
    const fold=document.getElementById('spinTestFold');fold.open=true;
    // a direct thrust step, for the truth: 63% of the way
    const stepLag=()=>{const c=actuators()[0];simBenchHold(true);step(0.5);for(let i=0;i<1/PDT;i++){if(i%5===0)simBenchSet(0,0.55);physStep();}
      const T0=act.get(c.id).T,tr=[];for(let i=0;i<0.6/PDT;i++){if(i%5===0)simBenchSet(0,0.65);physStep();tr.push([i*PDT,act.get(c.id).T]);}
      const T1=tr[tr.length-1][1],hit=tr.find(([,T])=>T>=T0+0.632*(T1-T0));simBenchSet(-1,0);simBenchHold(false);step(0.2);return hit[0];};
    out.spin=[];
    for(const k of [1,3.3]){
      await fresh(k);const truth=stepLag();await fresh(k);
      renderSpinTest();document.querySelector('[data-focus-key="spt-start"]').click();running=false;
      step(8);sptPoll();
      assert(spt.phase==='done'&&spt.result&&spt.result.ok,'spin test done: '+spt.text);
      assert(Math.abs(spt.result.lag/truth-1)<0.15,`spin-up ${spt.result.lag} against a thrust step's ${truth}`);
      assert(spt.result.delay<0.006,'the simulator\'s delay is small: '+spt.result.delay);
      assert(brt.benchHold===false&&!brt.spin,'off the rope after it');
      out.spin.push([k,+spt.result.lag.toFixed(3),+truth.toFixed(3),+spt.result.delay.toFixed(3)]);
    }
    document.querySelector('[data-focus-key="spt-apply"]').click();
    assert(cfg.motorTiming&&Math.abs(cfg.motorTiming.lag-spt.result.lag)<1e-3,'Apply keeps the spin-up');
    assert(tsgModel().lagFrom==='rope'&&Math.abs(tuneMotorLag()-cfg.motorTiming.lag)<1e-9,'the gains suggestion and the predictions use it');
    undoStep();assert(!cfg.motorTiming,'undo takes it back');
    // the lift test: motors 30% stronger than their cards say. The run is the motors as they are (a motor test: the
    // cards don't fly it); then the cards are made to say 1/1.3 of that, and the test must find it and put it right.
    await fresh(1,1.3);
    const ms=actuators(),rho=flightAtmosphere().rho,W=truth.m*G,real=new Map(ms.map(c=>[c.id,FlightPhysics.motor(c,designVref())]));
    renderRopeLift();document.querySelector('[data-focus-key="rlt-start"]').click();running=false;
    step(25);rltPoll();
    assert(rlt.phase==='done'&&rlt.result&&rlt.result.pulses.length>=2,'lift test done: '+rlt.text);
    for(const c of ms)c.propPhysics={...c.propPhysics,ct:c.propPhysics.ct/1.3};
    recomputeProps();
    const a=rltAnalyse(rlt.result);
    const Ttrue=u=>ms.reduce((t,c)=>{const mp=real.get(c.id);return t+FlightPhysics.equilibrium({...mp,kT:mp.kT*spreadOf(c).kT},u,a.V,rho).T;},0);
    let lo=0,hi=1;for(let i=0;i<40;i++){const m=(lo+hi)/2;if(Ttrue(m)<W)lo=m;else hi=m;}
    out.lift={hover:+a.hoverU.toFixed(3),trueHover:+lo.toFixed(3),model:+a.hoverModel.toFixed(3),ratio:+a.ratio.toFixed(3),pulses:rlt.result.pulses};
    assert(Math.abs(a.hoverU-lo)<0.02,`hover ${a.hoverU} against the truth's ${lo}`);
    const ratioTrue=a.pts.reduce((t,p)=>t+Ttrue(p.u)/p.model,0)/a.pts.length;out.lift.ratioTrue=+ratioTrue.toFixed(3);
    assert(ratioTrue>1.04,'the cards do say less than the motors give ('+ratioTrue+')');
    assert(Math.abs(a.ratio/ratioTrue-1)<0.06,`it finds how much stronger they are: ${a.ratio} against ${ratioTrue}`);
    document.querySelector('[data-focus-key="rlt-apply"]').click();
    assert(rlt.applied&&ms.every(c=>motorMeasured(c)),'Apply marks the cards measured: '+rlt.msg);
    const after=rltBelieved(a.pts[0].u,a.V)/a.pts[0].T;out.lift.after=+after.toFixed(3);
    assert(Math.abs(after-1)<0.08,`after Apply the controller's model lifts what they do (${after})`);
    assert(tsgModel().measured,'Gains from the motors now counts them measured');
    return out;
  });
  console.log(JSON.stringify(result));
  assert.deepStrictEqual(errors,[]);
  console.log('Rope hang tests in the simulator: ok');
}finally{if(browser)await browser.close();server.close();}
})().catch(error=>{console.error(error);server.close();process.exitCode=1;});
