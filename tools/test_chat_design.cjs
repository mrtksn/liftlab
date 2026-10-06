#!/usr/bin/env node
'use strict';
// Browser checks for untrusted Markdown, saved-design physics and responsive edit HUD.
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
  browser=await chromium.launch({executablePath:process.env.CHROME_PATH || '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',headless:true,args:process.argv.includes('--hardware-gpu')?['--enable-gpu']:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
  const page=await browser.newPage({viewport:{width:1440,height:1000}}),errors=[];
  page.on('pageerror',e=>errors.push(e.message));await page.route(/fonts\.google|goatcounter|gc\.zgo/,route=>route.abort());
  await page.goto(process.env.LIVE_URL || 'http://127.0.0.1:'+server.address().port,{waitUntil:'networkidle'});
  await page.waitForFunction(()=>brt.ready && typeof ChatMarkdown!=='undefined');
  const results=await page.evaluate(async()=>{
    const passed=[],check=(ok,msg)=>{if(!ok)throw new Error(msg);},close=(a,b)=>check(Math.abs(a-b)<1e-7*Math.max(1,Math.abs(b)),`${a} != ${b}`);
    running=false;agent.cfg.connected=true;agent.cfg.model='Preview';const th=threadNew();agent.cur=th.id;aiUi.view='chat';
    const sample='# Link options\n\n- **ESP-NOW** for normal flying.\n- **ExpressLRS** for long range.\n  - Tune *power* and `rate`.\n\n| Link | Medium |\n| --- | --- |\n| Serial | Laser or fibre |\n| Wi-Fi | UDP |\n\n> Match the binding phrase.\n\n```c\nradio=serial,115200\n// <script> is plain code\n```\n\n[Documentation](https://example.com/guide)\n\n- [x] Configured\n- [ ] Hardware test';
    th.feed=[{who:'you',text:'**Show the links**'},{who:'ai',text:sample},{who:'tool',name:'get_state',args:'{}',text:'**plain tool output**'}];
    agentRender();agentRenderFeed(th);UI_PANELS.editor.select('ai');
    const box=document.querySelector('.w-ai .ai-markdown');
    check(box.querySelector('h1')&&box.querySelectorAll('strong').length===2,'Headings/emphasis not rendered');
    check(box.querySelector('ul ul')&&box.querySelector('em')&&box.querySelector('code'),'Nested/inline Markdown missing');
    check(box.querySelectorAll('table tbody tr').length===2&&box.querySelector('.ai-table[tabindex="0"]'),'Table not rendered/scrollable');
    check(box.querySelector('pre code').textContent.includes('<script>'),'Code was interpreted/lost');
    check([...box.querySelectorAll('input')].every(i=>i.type==='checkbox'&&i.disabled),'Task checks are interactive');
    check(box.querySelector('a').rel.includes('noopener')&&box.querySelector('a').target==='_blank','Link isolation missing');
    check(document.querySelector('.w-you').textContent==='**Show the links**'&&document.querySelector('.ai-pre.out').textContent==='**plain tool output**','Non-assistant messages changed');
    const raw=ChatMarkdown.render('<img src=x onerror="window.markdownExecuted=1">\n\n<script>window.markdownExecuted=1</script>\n\n[bad](javascript:alert%281%29)\n\n[bad](data:text/html,bad)\n\n[bad](jav&#x61;script:alert%281%29)\n\n![No remote image](https://example.com/image.png)');
    document.body.append(raw);
    check(!raw.querySelector('img,script,iframe,svg,style,form')&&!window.markdownExecuted,'Raw HTML executed');
    check([...raw.querySelectorAll('a[href]')].every(a=>/^https?:|^mailto:/.test(a.href)),'Unsafe link survived');raw.remove();
    const item=th.feed[1],original=ChatMarkdown.message(item);item.text='**Updated reply**';check(ChatMarkdown.message(item).querySelector('strong').textContent==='Updated reply','Edited reply cache stale');item.text=sample;check(original.querySelector('h1'),'Cached nodes were mutated');agentRenderFeed(th);
    passed.push('Markdown formatting, nested lists, tables, inert HTML/code, safe links and cached reply updates');
    loadPreset('quadx');running=false;while(!brt.ready)await new Promise(r=>setTimeout(r,20));view.readouts=true;showApply();setEditMode(true);updateLive();
    const s=droneDesignStats(),mass=cfg.frame.mass+cfg.comps.reduce((n,c)=>n+c.mass,0),area=actuators().reduce((n,c)=>n+Math.PI*propR(c)**2,0);
    close(s.mass,mass);close(s.area,area);close(s.diskLoading,mass*G/area);close(s.energy,battCfg().cells*3.7*battCfg().capacity);check(s.tw>1&&s.rpm>0,'Thrust/RPM estimate missing');
    check(!document.getElementById('hudDesign').hidden,'Edit HUD hidden');for(const id of ['hudTime','hudPos','hudCmd','hudTq'])check(getComputedStyle(document.getElementById(id)).display==='none','Flight readout visible in edit mode: '+id);
    const hudStyle=getComputedStyle(document.getElementById('hudDesign'));
    check(hudStyle.pointerEvents==='none'&&hudStyle.backgroundColor==='rgba(0, 0, 0, 0)'&&parseFloat(hudStyle.borderTopWidth)===0,'Design HUD blocks the view');
    check(document.querySelectorAll('#hudDesignStats .hud-design-row').length===4,'Design HUD is not compact');
    const baseline={mass:s.mass,tw:s.tw,rpm:s.rpm};S.battV=1;S.batt.soc=0;hb.cellsLost=3;hb.cut=true;
    for(const c of actuators())hsOf(c).dead=true;const battery=cfg.comps.find(c=>c.battery);cargo.off.add(battery.id);
    designStatsCache=null;const independent=droneDesignStats();close(independent.mass,baseline.mass);close(independent.tw,baseline.tw);close(independent.rpm,baseline.rpm);
    const added=mkMass('Unknown payload',.3,0,0,{mass:.5,known:false});cfg.comps.push(added);recomputeProps();const heavy=droneDesignStats();close(heavy.mass,baseline.mass+.5);check(heavy.rigid.c[0]>s.rigid.c[0]&&heavy.tw<s.tw,'Unknown design mass excluded');
    const c=actuators()[0],small=heavy.diskLoading;c.prop*=1.2;recomputeProps();check(droneDesignStats().diskLoading<small,'Prop change did not refresh disk loading');
    computers().wiring ||= {parts:{},boards:{}};for(const motor of actuators())computers().wiring.parts[motor.id]={driver:'brushed',maxDuty:20};check(droneDesignStats().tw<heavy.tw*.5,'Output limits missing from stats');
    cfg.comps=cfg.comps.filter(c=>c.type!=='motor');recomputeProps();renderDesignHud();check(droneDesignStats().diskLoading===null&&!document.getElementById('hudDesignStats').textContent.includes('NaN'),'No-motor design is invalid');
    setEditMode(false);updateLive();check(document.getElementById('hudDesign').hidden&&getComputedStyle(document.getElementById('hudTime')).display!=='none','Flight HUD did not return');
    loadPreset('quadx');running=false;while(!brt.ready)await new Promise(r=>setTimeout(r,20));setEditMode(true);view.readouts=false;showApply();check(document.getElementById('hudDesign').hidden,'Show readouts off ignored');view.readouts=true;showApply();check(!document.getElementById('hudDesign').hidden,'Show readouts on ignored');
    resetHealth();doReset();while(!brt.ready)await new Promise(r=>setTimeout(r,20));updateLive();
    const snap=designSnap();renderDesignHud();check(designSnap()===snap,'Reading design stats changed the design');
    const beforeMass=droneDesignStats().mass,input=document.getElementById('frameMass-n');input.value=String(cfg.frame.mass+.2);input.dispatchEvent(new Event('input',{bubbles:true}));close(droneDesignStats().mass,beforeMass+.2);undoStep();close(droneDesignStats().mass,beforeMass);
    passed.push('Design equations, saved-vs-live state, unknown mass, prop/output changes, empty motors and edit/fly/Show transitions');
    agentSave();agentSaveThreads();
    return passed;
  });console.log(JSON.stringify(results,null,2));assert.deepStrictEqual(errors,[]);
  if(process.env.TEST_SCREENSHOTS){await page.evaluate(()=>{UI_PANELS.editor.select('ai');document.getElementById('aiFeed').scrollTop=0;});await page.waitForTimeout(500);await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-desktop.png'});}
  for(const theme of ['light','dark']){
    await page.setViewportSize({width:390,height:844});await page.evaluate(theme=>{document.documentElement.dataset.theme=theme;renderDesignHud();},theme);
    await page.waitForTimeout(250);
    await page.evaluate(()=>{view.readouts=true;showApply();renderDesignHud();});
    assert(await page.evaluate(()=>{
      const hud=document.getElementById('hudDesign'),r=hud.getBoundingClientRect(),v=document.querySelector('.view').getBoundingClientRect();
      return r.width>0&&r.right<=v.right+1&&r.left>=v.left&&hud.scrollWidth<=hud.clientWidth+1&&r.bottom<document.getElementById('editBar').getBoundingClientRect().top;
    }),'Mobile design HUD overflows');
    if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-'+theme+'-mobile.png'});
  }
  await page.reload({waitUntil:'networkidle'});await page.waitForFunction(()=>brt.ready);
  assert(await page.evaluate(()=>{
    running=false;const th=agent.threads.find(t=>t.feed.some(i=>i.who==='ai'&&i.text.includes('# Link options')));if(!th)return false;
    agent.cur=th.id;aiUi.view='chat';agentRender();agentRenderFeed(th);
    return !!document.querySelector('.w-ai .ai-markdown strong')&&!!document.querySelector('.w-ai table');
  }),'Saved Markdown reply did not render after reload');
  assert.deepStrictEqual(errors,[]);console.log('Desktop/mobile, light/dark: no page errors or HUD overflow');
}finally{if(browser)await browser.close();server.close();}
})().catch(error=>{console.error(error);server.close();process.exitCode=1;});
