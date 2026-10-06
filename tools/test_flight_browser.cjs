#!/usr/bin/env node
// Requires Playwright and Chrome. PLAYWRIGHT_PATH / CHROME_PATH may override local defaults.
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
  browser=await chromium.launch({executablePath:process.env.CHROME_PATH || '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
  const page=await browser.newPage({viewport:{width:1440,height:1000}}),errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.route(/fonts\.google|goatcounter|gc\.zgo/,route=>route.abort());
  await page.goto('http://127.0.0.1:'+server.address().port,{waitUntil:'networkidle'});
  await page.waitForFunction(()=>typeof brt!=='undefined' && brt.ready && typeof flightPerf!=='undefined');
  const result=await page.evaluate(async()=>{
    running=false;const passed=[];
    const assert=(ok,message)=>{if(!ok)throw new Error(message);};
    const close=(a,b,tol=1e-6)=>assert(Math.abs(a-b)<=tol*Math.max(1,Math.abs(b)),`${a} != ${b}`);
    const editNumber=(id,value)=>{const input=document.getElementById(id+'-n');assert(input,'Missing input '+id);input.value=String(value);input.dispatchEvent(new Event('input',{bubbles:true}));};
    const settle=async(key,seconds=3)=>{loadPreset(key);running=false;while(!brt.ready)await new Promise(r=>setTimeout(r,20));for(let i=0;i<seconds/PDT;i++){if(i%20===0)pilotStep(.01);physStep();}};
    await settle('quadx');assert(!S.crashed && brt.fcState===1,'Quad did not fly');
    const initialI=S.battI;assert(initialI>0 && S.deviceW>=8 && S.motorW>0,'Power accounting not active');
    passed.push('Quad flight and battery/motor/device power');
    let battery=cfg.comps.find(c=>c.type==='mass'&&c.battery),oldMass=battery.mass,oldCapacity=battCfg().capacity;
    editNumber('batt-capacity',oldCapacity*2);close(battery.mass,oldMass*2);assert(document.getElementById('battSmall').textContent.includes('360 g'),'Battery label not using actual mass');
    close(Number(document.getElementById('battery-physics-heatCapacity-n').value),900*battery.mass,1e-2);
    undoStep();battery=cfg.comps.find(c=>c.type==='mass'&&c.battery);close(battery.mass,oldMass);close(battCfg().capacity,oldCapacity);
    redoStep();battery=cfg.comps.find(c=>c.type==='mass'&&c.battery);close(battery.mass,oldMass*2);
    openSet.add(battery.id);renderComps();editNumber(`f-${battery.id}-mass`,.25);assert(battery.batteryAutoMass===false && !document.getElementById(`physics-${battery.id}-auto`).checked,'Manual battery override missing');
    editNumber('batt-capacity',oldCapacity);close(battery.mass,.25);passed.push('Capacity/weight, undo/redo and manual battery override');
    await settle('quadx');let c=actuators()[0];const unchanged=designSnap();openSet.add(c.id);renderComps();assert(designSnap()===unchanged,'Inspecting motor changed design');
    const select=document.getElementById(`physics-${c.id}-kind`);select.value='brushless';select.dispatchEvent(new Event('change',{bubbles:true}));
    const physicalFold=document.getElementById(`physics-${c.id}-fold`);physicalFold.open=true;
    const kvInput=document.getElementById(`physics-${c.id}-kv-n`);kvInput.focus();editNumber(`physics-${c.id}-kv`,1200);assert(document.activeElement===kvInput && physicalFold.open,'Editing model loses focus/disclosure');
    const before=flightMotor(c),oldRadius=c.prop;editNumber(`f-${c.id}-prop`,oldRadius*1.1);const after=flightMotor(c);
    close(before.Ke,after.Ke);close(before.R,after.R);assert(after.Om<before.Om,'Prop change resized fixed motor');
    passed.push('Fixed motor UI and independent prop loading');
    const editor=document.querySelector(`[data-id="${c.id}"] textarea[aria-label="Measured fixed-pitch prop data"]`);
    editor.value='RPM, thrust, torque\n6000, 1, .01\n12000, 4, .04';editor.parentElement.querySelector('button').click();
    assert(flightMotor(c).table?.length===2,'Prop table not applied');
    const design=readDesignFile(JSON.stringify({format:FILE_FORMAT,design:designOf()})).design;
    assert(design.comps.find(p=>p.id===c.id).propPhysics.rows.length===2,'Prop table lost on export');
    let rejected=false;try{readDesignFile(JSON.stringify({comps:[{...c,propPhysics:{rows:[[1,1,-1],[2,2,1]]}}]}));}catch(e){rejected=true;}assert(rejected,'Bad imported prop accepted');
    editNumber(`f-${c.id}-prop`,oldRadius);assert(!c.propPhysics.rows,'Prop table survived a diameter change');passed.push('Prop import, export validation and diameter binding');
    envr.pressure=80000;envr.sensorEffects=true;save();const snap=designSnap();envr.pressure=101325;envr.sensorEffects=false;restoreSnap(snap);
    close(envr.pressure,80000);assert(envr.sensorEffects && document.getElementById('flight-sensor-effects').checked,'Environment not restored');
    passed.push('Density/sensor environment persistence');
    await settle('quadx');envr.pressure=101325;envr.sensorEffects=false;
    const full=envelopeCalc();assert(full.inside,'Full-duty quad cannot hover');
    computers().wiring ||= {boards:{},parts:{}};
    for(const motor of actuators()) computers().wiring.parts[motor.id]={...(computers().wiring.parts[motor.id]||{}),driver:'brushed',maxDuty:20};
    const limited=envelopeCalc();assert(!limited.inside,'Duty-limited envelope still predicts hover');
    const core=brt.inst.get(boardOf('core').id),nm=actuators().length,nj=joints().length,settings=new Float32Array(6+3*nm+2*nj);settings[0]=1;settings[1]=.3;settings[4]=nm;settings[5]=nj;
    for(let i=0;i<nm;i++){settings[6+3*i]=1;settings[7+3*i]=.8;settings[8+3*i]=.9;}settings[6]=0;
    flightRememberSettings(core,settings);flightApplyLimits(core,14);
    const out=new Float32Array(core.memory.buffer,core.fr_ptr(),settings.length);close(out[0],1);close(out[1],.3,1e-5);close(out[6],0);close(out[7],.8,1e-5);
    assert(out[8]<.1 && flightLimitCache.get(core)[8]>.89,'Driver limit lost supervisor settings');
    passed.push('Duty/voltage allocation ceilings and preserved supervisor settings');
    await settle('tiltquad');const device=flightDevicePower(),servo=joints()[0],st=jst.get(servo.id);st.tq=servo.torque;st.rate=1;
    assert(flightDevicePower()>device,'Loaded servo draws no additional power');
    passed.push('Tilt flight and load-dependent servo power');
    await settle('wingquad');const wing=cfg.comps.find(isWing);wing.polar=[[-20,-.5,.2],[0,.5,.1],[20,1,.2]];recomputeProps();for(let i=0;i<1000;i++)physStep();assert(S.aero.every(a=>a.F.every(Number.isFinite)),'Polar produced nonfinite force');
    cfg.frame.polar=wing.polar;const shape=frameShapeOf();setFrameShape(shape);assert(cfg.frame.polar.length===3,'Frame polar lost');
    const rt={},oldBoxes=terrain.boxes;terrain.boxes=[{lo:[-10,-10,1],hi:[10,10,2]}];close(flightSkyVisibility([0,0,0],rt),0);terrain.boxes=[];rt.skyTime=-1;close(flightSkyVisibility([0,0,0],rt),1);terrain.boxes=oldBoxes;
    envr.sensorEffects=true;const gps=allSensors().find(c=>c.kind==='fix');
    terrain.boxes=[{lo:[S.p[0]-100,S.p[1]-100,S.p[2]+1],hi:[S.p[0]+100,S.p[1]+100,S.p[2]+2]}];assert(measure(gps,{st:{}},.2)===null,'Blocked GPS still provides a fix');terrain.boxes=oldBoxes;
    const baro={...allSensors().find(c=>c.kind==='baro'),noise:0,drift:0},oldRotors=S.rotors;
    S.rotors=[{p:[0,0,.1],d:[0,0,1],T:3,R:.08}];const biased=measure(baro,{st:{}},.02);envr.sensorEffects=false;const clean=measure(baro,{st:{}},.02);assert(biased<clean,'Barometer wash has no effect');S.rotors=oldRotors;
    const top={p:[0,0,.2],d:[0,0,1],T:3,R:.08},bottom={p:[0,0,0],d:[0,0,1],T:3,R:.08};envr.rotorSamples=1;const centerWake=nrm(flightRotorWake(bottom,[top,bottom],1.225));envr.rotorSamples=5;const diskWake=nrm(flightRotorWake(bottom,[top,bottom],1.225));assert(diskWake<centerWake,'Overlapping wake not averaged over disk');
    passed.push('Wing forces, frame polar persistence and cached sky blockage');
    await settle('quadx');envr.spread=0;
    for(const motor of actuators()){
      const p=flightMotor(motor);motor.motorPhysics={kind:'brushed',kv:60/(2*Math.PI*p.Ke),resistance:p.R,inertia:p.J,currentLimit:p.iMax,brushDrop:.6};
      motor.propPhysics={radius:motor.prop,rows:Array.from({length:16},(_,i)=>{const o=p.Om*(i+1)/16*1.2;return[o*60/(2*Math.PI),p.kT*o*o,p.kQ*o*o];})};
    }
    recomputeProps();doReset();while(!brt.ready)await new Promise(r=>setTimeout(r,20));for(let i=0;i<10000;i++){if(i%20===0)pilotStep(.01);physStep();}
    assert(!S.crashed && brt.fcState===1 && S.p.every(Number.isFinite),'Brushed/table flight failed');passed.push('Brushed motor and measured-prop flight');
    const heliKey=Object.keys(PRESETS).find(key=>PRESETS[key].label.startsWith('Helicopter'));
    await settle(heliKey,5);assert(!S.crashed && brt.fcState===1,'Collective helicopter flight failed');
    const main=actuators().find(isCollective),fullCollective=flightAvailable(main);computers().wiring ||= {parts:{},boards:{}};computers().wiring.parts[main.id]={driver:'brushed',maxDuty:10};
    assert(flightAvailable(main)<fullCollective*.5,'Collective ceiling ignores motor load');passed.push('Collective flight and governor-aware feasibility');
    // Persist an explicit profile, then verify it again after the storage reload.
    await settle('quadx');c=actuators()[0];flightFixedMotor(c,'brushless');c.propPhysics={radius:c.prop,rows:[[6000,1,.01],[12000,4,.04]]};envr.pressure=80000;envr.sensorEffects=true;recomputeProps();save();
    renderFlightPhysics();assert(document.getElementById('flightPhysicsKv').textContent.includes('Battery current'),'Live physics diagnostics missing');
    assert(flightPerf.read()?.cpu95>=0,'Frame profiler not running');passed.push('Live diagnostics and frame profiler');
    return {passed,initialI};
  });
  assert.deepStrictEqual(errors,[]);console.log(JSON.stringify(result,null,2));
  // Verify browser-storage reload, including profiles and environment.
  await page.reload({waitUntil:'networkidle'});await page.waitForFunction(()=>brt.ready);assert.deepStrictEqual(errors,[]);
  assert(await page.evaluate(()=>actuators()[0].motorPhysics?.kind==='brushless' && actuators()[0].propPhysics?.rows.length===2 && envr.pressure===80000 && envr.sensorEffects),'Saved profile/environment did not survive reload');
  await page.evaluate(()=>{running=false;UI_PANELS.editor.select('air');document.getElementById('flightTimingFold').open=true;renderFlightPhysics();document.getElementById('flightPhysicsKv').scrollIntoView();});
  if(process.env.FLIGHT_SCREENSHOT)await page.screenshot({path:process.env.FLIGHT_SCREENSHOT});
  console.log('Cold startup and saved-design reload: no page errors');
}finally{if(browser)await browser.close();server.close();}
})().catch(error=>{console.error(error);server.close();process.exitCode=1;});
