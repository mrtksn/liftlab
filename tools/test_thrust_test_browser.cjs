#!/usr/bin/env node
// Requires Playwright and Chrome. PLAYWRIGHT_PATH / CHROME_PATH may override local defaults.
// Tune → Motor thrust on the simulator: the toy-drone rebuild held on its simulated scale, each motor tested alone,
// the fit against the simulated motors' real thrust (their hidden spread included), Apply (the prop coefficient on the
// cards, the measured bend for the flight controller, no hidden spread left), a second test of a motor agreeing with
// its new card, undo, and the drone flying again once the scale lets go.
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
  await page.waitForFunction(()=>typeof brt!=='undefined' && brt.ready && typeof renderThrustTest==='function');
  const result=await page.evaluate(async()=>{
    running=false;const out={};
    const assert=(ok,message)=>{if(!ok)throw new Error(message);};
    const step=(seconds,tick)=>{for(let i=0;i<seconds/PDT;i++){if(i%20===0){pilotStep(.01);if(tick)ttTick();}physStep();}};
    loadPreset('toy1s');running=false;while(!brt.ready)await new Promise(r=>setTimeout(r,20));
    simBenchHold(true);step(1.5);
    assert(brt.fcState===0,'held on the scale it stays disarmed ('+brt.fcState+')');
    const fold=document.getElementById('thrustTestFold');fold.open=true;renderThrustTest();
    tt.zero=simScaleGrams();out.zero=tt.zero;
    assert(Math.abs(tt.zero-truth.m*1000)<=2,'the motors-off reading is the weight');
    const ms=actuators(),vref=designVref(),rho=flightAtmosphere().rho;
    // what each simulated motor really gives at full throttle, at the reference voltage (its hidden spread included)
    const real=ms.map(c=>FlightPhysics.equilibrium(flightMotor(c),1,vref,rho).T*spreadOf(c).kT);
    const test=i=>{ttBegin(i);assert(tt.cur&&tt.cur.spinning,'motor '+i+' spins: '+tt.msg);
      for(let k=0;k<ttLevels().length;k++){step(0.6,true);assert(brt.fcState===4,'the flight core runs the motor test ('+brt.fcState+')');
        const others=ms.filter((c,j)=>j!==i).map(c=>act.get(c.id).u||0);assert(others.every(u=>u===0),'only the motor tested spins');
        ttRecord(simScaleGrams());}
      return ttFit(i);};
    out.fits=ms.map((c,i)=>{const f=test(i);assert(f&&!f.error,'fit '+i+': '+(f&&f.error));assert(!f.warn,'clean fit '+i+': '+f.warn);
      assert(Math.abs(f.tmax/real[i]-1)<0.08,`motor ${i}: measured ${f.tmax.toFixed(3)} N against the simulated ${real[i].toFixed(3)} N`);
      assert(f.bend>-0.3&&f.bend<1.3,'bend '+f.bend);return [+f.tmax.toFixed(3),+f.bend.toFixed(3),+real[i].toFixed(3)];});
    assert(!tt.cur&&!brt.bench,'after the last level the motor is off');
    const before=designSnap(),ct0=ms.map(c=>c.propPhysics?.ct);
    ttApply();assert(tt.applied,'applied: '+tt.msg);
    out.cts=ms.map(c=>c.propPhysics.ct);
    ms.forEach((c,i)=>{assert(motorMeasured(c),'motor '+i+' counts as measured');assert(curveHat(c)===c.thrustTest.bend,'the controller is told its bend');
      assert(spreadOf(c).kT===1,'no hidden thrust spread left');assert(Math.abs(c.tmax/out.fits[i][0]-1)<0.08,`card ${c.tmax} against measured ${out.fits[i][0]}`);});
    assert(fcAirframe().motors.every((m,i)=>m.bend===ms[i].thrustTest.bend),'the exported airframe carries the measured bends');
    // measure motor 0 again: the simulated motor now is its card
    const again=test(0);out.again=[+again.tmax.toFixed(3),+ms[0].tmax.toFixed(3)];
    assert(Math.abs(again.tmax/ms[0].tmax-1)<0.06,`second test ${again.tmax} against the new card ${ms[0].tmax}`);
    // one undo step takes it all back
    undoStep();const ms2=actuators();
    assert(ms2.every((c,i)=>c.propPhysics?.ct===ct0[i]&&!c.thrustTest),'undo restores the cards');assert(designSnap()===before,'undo restores the design');
    // the scale lets go: it flies as before
    fold.open=false;fold.dispatchEvent(new Event('toggle'));simBenchHold(false);step(4);
    out.after=brt.fcState;assert(brt.fcState===1,'it arms and flies again ('+brt.fcState+')');
    return out;
  });
  console.log(JSON.stringify(result));
  assert.deepStrictEqual(errors,[]);
  console.log('Motor thrust test in the simulator: ok');
}finally{if(browser)await browser.close();server.close();}
})().catch(error=>{console.error(error);server.close();process.exitCode=1;});
