#!/usr/bin/env node
'use strict';
// Room Survey → World: a surface the survey page posts from LiftLab's Room Survey dialog becomes an object library
// import in the World editor (js/room-survey.js), faces turned toward the walk, and placed copies collide with it.
const fs=require('fs'),path=require('path'),http=require('http'),assert=require('assert');
const {chromium}=require(process.env.PLAYWRIGHT_PATH || '/Users/mrtksn/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const root=path.resolve(__dirname,'..');
const types={'.js':'application/javascript','.css':'text/css','.html':'text/html','.json':'application/json','.wasm':'application/wasm'};
const server=http.createServer((req,res)=>{
  const url=new URL(req.url,'http://localhost'),file=path.resolve(root,'.'+decodeURIComponent(url.pathname==='/'?'/index.html':url.pathname));
  if(!file.startsWith(root+path.sep)){res.writeHead(403).end();return;}
  fs.readFile(file,(err,data)=>{if(err){res.writeHead(404).end();return;}
    res.setHeader('Cache-Control','no-store');res.setHeader('Content-Type',types[path.extname(file)]||'application/octet-stream');res.end(data);});
});

// A 4 × 3 m room, 2.5 m high, Z up: floor, four walls (the front one with a doorway), no ceiling; the walk in the middle.
const W=4,D=3,H=2.5;
const room={type:'liftlab-survey-scene',id:'t1',name:'Gaussian surface mesh · 20261122-100049-c7a0f',units:'m',levelled:true,
  polygons:[
    {outer:[[0,0,0],[0,D,0],[W,D,0],[W,0,0]]},                                     // floor, wound to face down
    {outer:[[0,0,0],[W,0,0],[W,0,H],[0,0,H]],holes:[[[1.5,0,0.01],[2.5,0,0.01],[2.5,0,2],[1.5,0,2]]]},   // front, with a door
    {outer:[[0,D,0],[0,D,H],[W,D,H],[W,D,0]]},
    {outer:[[0,0,0],[0,0,H],[0,D,H],[0,D,0]]},
    {outer:[[W,0,0],[W,D,0],[W,D,H],[W,0,H]]},
    {outer:[[0,0,H],[W,0,H],[W,D,H]]},                                             // a triangle (a Triangle list output)
  ],
  points:[[1,1,1,255,0,0],[2,2,1,0,255,0],[3,1,2,0,0,255]],paths:[[[1,1.5,1.2],[2,1.5,1.2],[3,1.5,1.2]]],
  viewpoints:[[1,1.5,1.2],[2,1.5,1.2],[3,1.5,1.2]]};

(async()=>{await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));let browser;
try{
  browser=await chromium.launch({executablePath:process.env.CHROME_PATH||'/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
  const page=await (await browser.newContext({viewport:{width:1600,height:1000}})).newPage(),errors=[];
  page.on('pageerror',e=>{errors.push(e.stack);console.error(e.stack);});
  await page.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());
  await page.goto('http://127.0.0.1:'+server.address().port,{waitUntil:'networkidle'});
  await page.waitForFunction(()=>typeof fleet!=='undefined'&&fleet.ready&&objectLibrary.ready);

  // The frame posts, as the survey page does (survey/graph.js sendToWorld); replies come back to it.
  const send=data=>page.evaluate(data=>new Promise(resolve=>{
    const w=document.getElementById('surveyFrame').contentWindow;
    w.addEventListener('message',e=>{if(e.data?.type==='liftlab-survey-reply')resolve(e.data);},{once:true});
    w.eval('parent.postMessage('+JSON.stringify(data)+',parent.location.origin)');
  }),data);

  // Malformed surfaces are refused and nothing opens.
  let reply=await send({...room,polygons:[{outer:[[0,0,0],[1,0]]}]});
  assert(!reply.ok&&/malformed/.test(reply.text),JSON.stringify(reply));
  assert(await page.evaluate(()=>!partImport.active));
  // Another window's message is ignored.
  await page.evaluate(r=>postMessage(r,location.origin),room);await page.waitForTimeout(200);
  assert(await page.evaluate(()=>!partImport.active),'a message from the page itself must not import');

  reply=await send(room);
  assert(reply.ok,reply.text);
  await page.waitForFunction(()=>partImport.active&&partImport.draft&&!partImport.busy);
  let r=await page.evaluate(()=>{const c=partImport.draft;return {world:wedit.on,selected:!!fleet.selected,context:partImport.context,up:c.model.up,units:c.model.units,
    size:c.model.rawSize.map(v=>v*c.model.scale),say:$('#objectImportSay').textContent,name:c.name,dlg:$('#surveyDlg').open,
    obj:new TextDecoder().decode(worldObjects.files.get(c.model.fileId).files[0].data)};});
  assert(r.world&&!r.selected&&r.context==='world'&&!r.dlg,'opens in the World editor: '+JSON.stringify({...r,obj:''}));
  assert.strictEqual(r.up,'z');assert.strictEqual(r.units,'m');
  [W,D,H].forEach((v,i)=>assert(Math.abs(r.size[i]-v)<1e-6,'size '+r.size));
  assert(/triangles, 3 points and 1 path from the survey\./.test(r.say)&&/metres/.test(r.say),r.say);
  const drawn=await page.evaluate(()=>{const kinds={};partImport.preview?.traverse?.(o=>{if(o.isPoints)kinds.points=[o.geometry.attributes.position.count,o.material.size,!!o.geometry.attributes.color];if(o.isLine)kinds.line=o.geometry.attributes.position.count;});return kinds;});
  assert(drawn.points&&drawn.points[0]===3&&drawn.points[1]>=3&&drawn.points[2]&&drawn.line===4,'points and path drawn '+JSON.stringify(drawn));
  assert(r.name.startsWith('Gaussian surface mesh'),r.name);

  // Every face points into the room (toward the walk), and the doorway is left open.
  const v=[],faces=[];
  for(const line of r.obj.split('\n')){const t=line.split(' ');if(t[0]==='v')v.push(t.slice(1,4).map(Number));if(t[0]==='f')faces.push(t.slice(1).map(i=>v[i-1]));}
  assert(faces.length>=12,'faces '+faces.length);
  for(const [a,b,c] of faces){
    const n=[(b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1]),(b[2]-a[2])*(c[0]-a[0])-(b[0]-a[0])*(c[2]-a[2]),(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])];
    const m=[0,1,2].map(k=>(a[k]+b[k]+c[k])/3),to=[2-m[0],1.5-m[1],1.2-m[2]];
    assert(n[0]*to[0]+n[1]*to[1]+n[2]*to[2]>0,'face turned away from the walk at '+m);
  }
  const inside=([a,b,c],[x,z])=>{const s=(p,q)=>(x-q[0])*(p[2]-q[2])-(p[0]-q[0])*(z-q[2]),d=[s(a,b),s(b,c),s(c,a)];return d.every(v=>v>=0)||d.every(v=>v<=0);};
  assert(!faces.some(f=>f.every(q=>q[1]===0)&&inside(f,[2,1])),'doorway filled');
  assert(faces.some(f=>f.every(q=>q[1]===0)&&inside(f,[.5,1])),'front wall missing');

  // A second scene while this import is open: refused, saying why.
  const busy=await send({...room,id:'t1b'});
  assert(!busy.ok&&/Another import or edit is open/.test(busy.text),JSON.stringify(busy));

  // Save to the library, place it: the copy collides with its triangles.
  await page.locator('#objectImportSave').click();
  await page.locator('#objectLibraryDlg [data-object-use]').first().click();
  await page.waitForFunction(()=>worldObjects.list.length===1&&!wedit.busy);
  r=await page.evaluate(()=>({collision:worldObjects.list[0].collision,asset:objectLibrary.list[0].worldCollision}));
  assert.strictEqual(r.collision,'mesh');assert.strictEqual(r.asset,'mesh');

  // Only points and a path: drawn, nothing to collide with.
  await page.evaluate(()=>worldEditSet(false));
  reply=await send({...room,id:'t5',polygons:[]});assert(reply.ok,reply.text);
  await page.waitForFunction(()=>partImport.active&&partImport.draft&&!partImport.busy);
  r=await page.evaluate(()=>({say:$('#objectImportSay').textContent,hint:partImport.hints.worldCollision}));
  assert(/^3 points and 1 path from the survey \(no surfaces/.test(r.say)&&r.hint==='boxes',JSON.stringify(r));
  await page.locator('#objectImportSave').click();
  await page.locator('#objectLibraryDlg [data-object-use]').first().click();
  await page.waitForFunction(()=>worldObjects.list.length===2&&!wedit.busy);
  r=await page.evaluate(()=>{const o=worldObjects.list[1];let pts=0;o.base.traverse(m=>{if(m.isPoints)pts+=m.geometry.attributes.position.count;});return {boxes:o.boxes.length,pts};});
  assert(r.boxes===0&&r.pts===3,'points-only copy '+JSON.stringify(r));
  await page.evaluate(()=>worldEditSet(false));
  reply=await send({...room,id:'t6',polygons:[],points:[],paths:[]});
  assert(!reply.ok&&/nothing to send/.test(reply.text),reply.text);

  // A map without metres, sized from the ToF estimate the output carried; a measured map ignores one.
  await page.evaluate(()=>worldEditSet(false));
  const half=v=>v.map(x=>x/2),relative={...room,id:'t3',units:'relative',viewpoints:room.viewpoints.map(half),
    polygons:room.polygons.map(p=>({outer:p.outer.map(half),holes:(p.holes||[]).map(h=>h.map(half))})),points:[],paths:[],
    scale:{metres_per_unit:2,median_error_m:.157,p80_error_m:.435,hits:36,eligible:46}};
  reply=await send(relative);assert(reply.ok,reply.text);
  await page.waitForFunction(()=>partImport.active&&partImport.draft&&!partImport.busy);
  r=await page.evaluate(()=>({units:partImport.draft.model.units,size:partImport.draft.model.rawSize.map(v=>v*partImport.draft.model.scale),say:$('#objectImportSay').textContent}));
  assert.strictEqual(r.units,'custom');[W,D,H].forEach((v,i)=>assert(Math.abs(r.size[i]-v)<1e-6,'ToF size '+r.size));
  assert(/2 m per survey unit, readings within 16 cm \(80% within 44 cm\) on 36 of 46/.test(r.say),r.say);
  await page.evaluate(()=>{partImportCancel();worldEditSet(false);});
  reply=await send({...relative,id:'t4',scale:{metres_per_unit:-1}});assert(reply.ok,reply.text);
  await page.waitForFunction(()=>partImport.active&&partImport.draft&&!partImport.busy);
  r=await page.evaluate(()=>({scale:partImport.draft.model.scale,say:$('#objectImportSay').textContent}));
  assert(r.scale===1&&/set the room's longest side/.test(r.say),JSON.stringify(r));
  await page.evaluate(()=>{partImportCancel();worldEditSet(false);});

  // Too many triangles for a world object: refused with the reason.
  const many={...room,id:'t2',polygons:Array.from({length:100001},(_,i)=>({outer:[[i,0,0],[i+1,0,0],[i,1,0]]}))};
  reply=await send(many);
  assert(!reply.ok&&/100,000/.test(reply.text),reply.text);

  assert.deepStrictEqual(errors,[]);
  console.log('Room survey to World: surfaces, points and paths, orientation, doorway, mesh collision, ToF size and refusals passed');
}catch(e){console.error(e);process.exitCode=1;}finally{await browser?.close();server.close();}})();
