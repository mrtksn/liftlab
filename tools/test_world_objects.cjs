#!/usr/bin/env node
'use strict';
// World objects: importing GLB, STL and OBJ models, their solid shapes in the terrain, moving and turning them
// with the edit handles, the drones they're dropped on, and keeping them across a reload.
const fs=require('fs'),path=require('path'),http=require('http'),assert=require('assert');
const {chromium}=require(process.env.PLAYWRIGHT_PATH || '/Users/mrtksn/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const root=path.resolve(__dirname,'..');
const server=http.createServer((req,res)=>{
  const url=new URL(req.url,'http://localhost'),file=path.resolve(root,'.'+decodeURIComponent(url.pathname==='/'?'/index.html':url.pathname));
  if(!file.startsWith(root+path.sep)){res.writeHead(403).end();return;}
  fs.readFile(file,(err,data)=>{if(err){res.writeHead(404).end();return;}
    res.setHeader('Cache-Control','no-store');res.setHeader('Content-Type',file.endsWith('.js')?'application/javascript':file.endsWith('.css')?'text/css':file.endsWith('.html')?'text/html':'application/octet-stream');res.end(data);});
});
// A box as a binary glTF, Y up (as Blender exports it): w × h × d with its height along Y.
function glbBox(w,h,d){
  const V=[[0,0,0],[w,0,0],[w,h,0],[0,h,0],[0,0,d],[w,0,d],[w,h,d],[0,h,d]],F=[0,2,1,0,3,2,4,5,6,4,6,7,0,1,5,0,5,4,1,2,6,1,6,5,2,3,7,2,7,6,3,0,4,3,4,7];
  const pos=Buffer.from(new Float32Array(V.flat()).buffer),idx=Buffer.from(new Uint16Array(F).buffer),bin=Buffer.concat([pos,idx,Buffer.alloc((4-(pos.length+idx.length)%4)%4)]);
  const json={asset:{version:'2.0'},buffers:[{byteLength:bin.length}],bufferViews:[{buffer:0,byteOffset:0,byteLength:pos.length,target:34962},{buffer:0,byteOffset:pos.length,byteLength:idx.length,target:34963}],
    accessors:[{bufferView:0,componentType:5126,count:8,type:'VEC3',min:[0,0,0],max:[w,h,d]},{bufferView:1,componentType:5123,count:36,type:'SCALAR'}],
    meshes:[{primitives:[{attributes:{POSITION:0},indices:1}]}],nodes:[{mesh:0}],scenes:[{nodes:[0]}],scene:0};
  let js=Buffer.from(JSON.stringify(json));js=Buffer.concat([js,Buffer.alloc((4-js.length%4)%4,0x20)]);
  const head=Buffer.alloc(12),jh=Buffer.alloc(8),bh=Buffer.alloc(8);
  head.write('glTF',0);head.writeUInt32LE(2,4);head.writeUInt32LE(12+8+js.length+8+bin.length,8);
  jh.writeUInt32LE(js.length,0);jh.write('JSON',4);bh.writeUInt32LE(bin.length,0);bh.write('BIN\0',4);
  return Buffer.concat([head,jh,js,bh,bin]);
}
// A cube in millimetres as ASCII STL (CAD style, Z up).
function stlCube(s){
  const V=[[0,0,0],[s,0,0],[s,s,0],[0,s,0],[0,0,s],[s,0,s],[s,s,s],[0,s,s]],F=[[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]];
  return Buffer.from('solid cube\n'+F.map(t=>' facet normal 0 0 0\n  outer loop\n'+t.map(i=>'   vertex '+V[i].join(' ')+'\n').join('')+'  endloop\n endfacet\n').join('')+'endsolid cube\n');
}
// A thin wall with no thickness: one quad, 4 m wide and 2 m tall (Y up).
const objWall=Buffer.from('o wall\nv 0 0 0\nv 4 0 0\nv 4 2 0\nv 0 2 0\nf 1 2 3\nf 1 3 4\n');
(async()=>{await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));let browser;
try{
 browser=await chromium.launch({executablePath:process.env.CHROME_PATH || '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',headless:true,args:['--enable-gpu']});
 const context=await browser.newContext({viewport:{width:1600,height:1000}}),page=await context.newPage(),errors=[];
 page.on('pageerror',e=>{errors.push(e.stack);console.error(e.stack);});
 page.on('dialog',d=>d.accept());
 await page.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());
 const url=process.env.LIVE_URL || 'http://127.0.0.1:'+server.address().port;
 await page.goto(url,{waitUntil:'networkidle'});
 await page.waitForFunction(()=>fleet.ready && brt.ready);
 await page.evaluate(()=>{applyTerrain('open',1);running=true;renderRun();});
 await page.selectOption('#droneSelect','');await page.waitForTimeout(150);
 await page.locator('#worldObjEdit').click();
 assert(await page.evaluate(()=>wedit.on&&!running&&!$('#worldEditBar').hidden&&$('#terrainSel').closest('.bar')&&$('#terrainSel').offsetParent!==null&&$('#mapSave').offsetParent!==null&&$('#worldPanel').classList.contains('editing-objects')),'World editor did not pause with maps available in the top bar and panel');

 // A Y-up GLB stands up: its 3 m along Y becomes its height.
 await page.evaluate(()=>{cam.target.set(4,0,0);});
 await page.locator('#worldObjFile').setInputFiles({name:'tower.glb',mimeType:'model/gltf-binary',buffer:glbBox(1,3,1)});
 await page.waitForFunction(()=>worldObjects.list.length===1&&!wedit.busy);
 let o=await page.evaluate(()=>{const o=worldObjects.list[0];return {name:o.name,size:o.size,units:o.units,pos:o.pos,boxes:o.boxes.length,sel:wedit.sel===o.id,top:terrainRay([4,0,10],[0,0,-1]),inside:solidAt([4,0,1.5],terrainNear([4,0,1.5],0)),objBoxes:terrain.boxes.filter(b=>b.obj===o.id).length};});
 assert(o.name==='tower'&&o.units==='m'&&o.sel&&o.boxes>0&&o.objBoxes===o.boxes,'GLB import wrong: '+JSON.stringify(o));
 assert(Math.abs(o.size[2]-3)<1e-6&&Math.abs(o.size[0]-1)<1e-6&&Math.abs(o.size[1]-1)<1e-6,'GLB not stood up (Y up to Z up): '+o.size);
 assert(Math.abs(o.top-(10-3))<0.005&&o.inside==='tower','GLB solid shape wrong (ray from above '+o.top+', inside '+o.inside+')');
 console.log(`GLB: stood up, ${o.boxes} boxes, top at ${(10-o.top).toFixed(3)} m, solid inside`);

 // A CAD cube in millimetres: read as millimetres, 2 m across.
 await page.evaluate(()=>{cam.target.set(-4,0,0);});
 await page.locator('#worldObjFile').setInputFiles({name:'crate.stl',mimeType:'model/stl',buffer:stlCube(2000)});
 await page.waitForFunction(()=>worldObjects.list.length===2&&!wedit.busy);
 o=await page.evaluate(()=>{const o=worldObjects.list[1];return {units:o.units,scale:o.scale,boxes:o.boxes.length,dims:o.size.map(x=>x*o.scale),side:terrainRay([-10,0,1],[1,0,0])};});
 assert(o.units==='mm'&&o.scale===0.001&&o.dims.every(x=>Math.abs(x-2)<1e-6)&&o.boxes===1,'STL mm cube wrong: '+JSON.stringify(o));
 assert(Math.abs(o.side-5)<0.005,'STL cube side not where expected: '+o.side);
 console.log('STL: millimetres guessed, 2 m cube as 1 box');

 // A thin wall (an open surface) is still solid: rays and the radio's line of sight meet it.
 await page.evaluate(()=>{cam.target.set(0,6,0);});
 await page.locator('#worldObjFile').setInputFiles({name:'wall.obj',mimeType:'text/plain',buffer:objWall});
 await page.waitForFunction(()=>worldObjects.list.length===3&&!wedit.busy);
 o=await page.evaluate(()=>{const o=worldObjects.list[2];return {boxes:o.boxes.length,size:o.size,hit:terrainRay([0,0,1],[0,1,0]),walls:terrainWalls([0,0,1],[0,12,1],24)};});
 assert(o.boxes>0&&Math.abs(o.hit-6)<0.15&&o.walls===1,'OBJ wall not solid: '+JSON.stringify(o));
 console.log(`OBJ: open wall solid as ${o.boxes} boxes, the radio counts it as one wall`);

 // Click the tower to select it, drag its X arrow, then turn it with the field.
 const centre=await page.evaluate(()=>{fleetScene();renderer.render(scene,camera);worldEditLookAt(worldObjects.list[0]);fleetScene();renderer.render(scene,camera);
   const r=vpEl.getBoundingClientRect(),p=new THREE.Vector3(4,0,1.2).project(camera);return {x:r.left+(p.x+1)*r.width/2,y:r.top+(1-p.y)*r.height/2};});
 await page.evaluate(()=>worldEditSelect(null));
 await page.mouse.move(centre.x+40,centre.y+40);await page.mouse.move(centre.x,centre.y);await page.mouse.down();await page.mouse.up();
 assert(await page.evaluate(()=>wedit.sel===worldObjects.list[0].id&&!$('#worldObjName').disabled),'Clicking an object did not select it');
 const arrow=await page.evaluate(()=>{fleetScene();renderer.render(scene,camera);const r=vpEl.getBoundingClientRect(),g=gizmo.position.clone(),k=gizmo.scale.x,
   pt=v=>{const p=v.clone().project(camera);return {x:r.left+(p.x+1)*r.width/2,y:r.top+(1-p.y)*r.height/2};};
   return {a:pt(g.clone().add(new THREE.Vector3(.55*k,0,0))),b:pt(g.clone().add(new THREE.Vector3(1.55*k,0,0))),k};});
 const x0=await page.evaluate(()=>worldObjects.list[0].pos[0]);
 await page.mouse.move(arrow.a.x,arrow.a.y);await page.mouse.down();
 for(let s=1;s<=8;s++)await page.mouse.move(arrow.a.x+(arrow.b.x-arrow.a.x)*s/8,arrow.a.y+(arrow.b.y-arrow.a.y)*s/8);
 await page.mouse.up();
 o=await page.evaluate(()=>{const o=worldObjects.list[0];return {pos:o.pos,ray:terrainRay([o.pos[0],0,10],[0,0,-1]),dragging:!!edit.drag};});
 assert(!o.dragging&&o.pos[0]>x0+0.3&&Math.abs(o.pos[0]-x0-arrow.k)<0.2&&o.pos[1]===0&&o.pos[2]===0,'X arrow drag wrong: '+JSON.stringify(o)+' from '+x0+' expected +'+arrow.k);
 assert(Math.abs(o.ray-7)<0.1,'Solid shape did not move with the object');
 console.log(`Drag: X arrow moved it ${(o.pos[0]-x0).toFixed(2)} m, its solid shape with it`);
 await page.locator('#worldObjYaw').fill('45');await page.locator('#worldObjYaw').press('Enter');
 o=await page.evaluate(()=>{const o=worldObjects.list[0],b=worldObjBounds(o);return {yaw:o.yaw,w:b.hi[0]-b.lo[0],gizmoRing:gizmo.children.filter(c=>c.userData.group==='rot'&&c.visible).map(c=>c.userData.axis)};});
 assert(o.yaw===45&&o.w>1.35&&o.w<1.55,'Turning did not make it solid again: '+JSON.stringify(o));
 await page.evaluate(()=>{fleetScene();});
 assert.deepStrictEqual(await page.evaluate(()=>gizmo.children.filter(c=>c.userData.group==='rot'&&c.visible).map(c=>c.userData.axis)),[2],'Only the vertical ring should show');
 console.log(`Turn: 45° makes it ${o.w.toFixed(2)} m across`);

 // Dropped on a drone: Done starts that drone again, and the simulation runs on.
 await page.evaluate(()=>{const d=fleet.drones[0];withDrone(d,()=>{S.t=5;});const p=d.state.S.p,o=worldObjects.list[1];o.pos=[+p[0].toFixed(2)+0,+p[1].toFixed(2)+0,0];worldEditMoved(o);worldEditChanged();});
 await page.locator('#worldEditDone').click();
 o=await page.evaluate(()=>({on:wedit.on,running,t:fleet.drones[0].state.S.t,bar:$('#worldEditBar').hidden,inside:withDrone(fleet.drones[0],()=>terrainNear(S.p,cReach).some(b=>b.obj!=null))}));
 assert(!o.on&&o.running&&o.bar&&o.t<5,'Done did not end editing, resume, and move the drone out: '+JSON.stringify(o));
 console.log('Done: simulation resumed; the drone under the crate started again');

 // Kept: the boxes at once, the models from IndexedDB.
 await page.evaluate(()=>fleetSave());
 const saved=await page.evaluate(()=>worldObjects.list.map(o=>({id:o.id,pos:o.pos,yaw:o.yaw,n:o.boxes.length})));
 await page.reload({waitUntil:'networkidle'});await page.waitForFunction(()=>fleet.ready && brt.ready);
 {const got=await page.evaluate(()=>worldObjects.list.map(o=>({id:o.id,pos:o.pos,yaw:o.yaw,n:o.boxes.length})));assert.deepStrictEqual(got,saved,'Objects not restored: '+JSON.stringify(got)+' vs '+JSON.stringify(saved));}
 assert(await page.evaluate(()=>terrain.boxes.filter(b=>b.obj!=null).length===worldObjects.list.reduce((s,o)=>s+o.boxes.length,0)),'Restored boxes not in the terrain');
 await page.waitForFunction(()=>worldObjects.list.every(o=>o.base&&!o.loading&&!o.missing),null,{timeout:10000});
 console.log('Reload: objects solid at once, models read back from IndexedDB');

 // Save complete maps, switch away, and restore after the active model files are deleted.
 await page.waitForFunction(()=>maps.ready);
 await page.locator('#worldView').click();await page.locator('#worldObjEdit').click();
 await page.evaluate(()=>{envr.wind=20;envr.pressure=60000;worldSeeds.noise=7654321;for(const d of fleet.drones)Object.assign(d.state.setpoint,{x:20,y:20,z:2});syncSp();});
 await page.locator('#mapName').fill('My flight map');await page.locator('#mapSave').click();
 await page.waitForFunction(()=>maps.list.length===1&&!maps.busy);
 const mapId=await page.evaluate(()=>maps.current),snapshot=await page.evaluate(()=>mapSnapshot());
 const drones=await page.evaluate(()=>({selected:fleet.selected?.id||null,running,designs:fleet.drones.map(d=>withDrone(d,designSnap)),targets:fleet.drones.map(d=>({...d.state.setpoint}))}));
 assert(await page.evaluate(id=>$('#terrainSel').value===id&&maps.list[0].files.length===3,mapId),'Saved map not available or model files missing');
 const downloadEvent=page.waitForEvent('download');await page.locator('#mapExport').click();const download=await downloadEvent;
 const portable=fs.readFileSync(await download.path()),document=JSON.parse(portable);
 assert(document.format==='liftlab-map'&&document.version===1&&document.files.length===3,'Portable export missing original models');
 assert(Buffer.from(document.files.find(f=>f.name==='tower.glb').files[0].data,'base64').equals(glbBox(1,3,1)),'Export changed original GLB bytes');
 await page.selectOption('#terrainSel','open');await page.waitForFunction(()=>!maps.busy&&worldObjects.list.length===0);
 await page.evaluate(async()=>{for(const f of [...worldObjects.files.values()]){await worldFileDelete(f.id);}worldObjects.files.clear();});
 await page.selectOption('#terrainSel',mapId);await page.waitForFunction(()=>!maps.busy&&worldObjects.list.length===3&&worldObjects.list.every(o=>o.base&&!o.loading));
 assert.deepStrictEqual(await page.evaluate(()=>mapSnapshot()),snapshot,'Saved map did not restore layout/objects/seeds/environment');
 assert.deepStrictEqual(await page.evaluate(()=>({selected:fleet.selected?.id||null,running,designs:fleet.drones.map(d=>withDrone(d,designSnap)),targets:fleet.drones.map(d=>({...d.state.setpoint}))})),drones,'Map switch replaced drone settings or pause/selection');
 await page.locator('#mapSave').click();await page.waitForFunction(()=>!maps.busy);
 assert(await page.evaluate(()=>maps.list.length===1),'Saving current map under its name should update it');
 await page.locator('#mapName').fill('Second map');await page.locator('#mapSave').click();await page.waitForFunction(()=>!maps.busy);
 assert(await page.evaluate(()=>maps.list.length===2),'A new name should save a separate map');
 await page.reload({waitUntil:'networkidle'});await page.waitForFunction(()=>fleet.ready&&maps.ready&&worldObjects.list.every(o=>o.base&&!o.loading));
 assert(await page.evaluate(()=>maps.list.length===2&&$('#terrainSel').value===maps.current&&$('#mapName').value==='Second map'),'Map library/current choice not kept after reload');
 console.log('Maps: save/update/copy, selector, model retention, fleet preservation and reload');

 // A fresh browser has no model files: the exported file must supply everything.
 const fresh=await browser.newContext({viewport:{width:1600,height:1000}}),cold=await fresh.newPage();
 cold.on('pageerror',e=>errors.push(e.stack));cold.on('dialog',d=>d.accept());
 await cold.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());
 await cold.goto(url,{waitUntil:'networkidle'});await cold.waitForFunction(()=>fleet.ready&&maps.ready);
 await cold.locator('#worldView').click();
 await cold.locator('#mapFile').setInputFiles({name:download.suggestedFilename(),mimeType:'application/json',buffer:portable});
 await cold.waitForFunction(()=>maps.list.length===1&&!maps.busy&&worldObjects.list.length===3&&worldObjects.list.every(o=>o.base&&!o.loading));
 const coldSnapshot=await cold.evaluate(()=>mapSnapshot());
 assert.deepStrictEqual({...coldSnapshot,objects:coldSnapshot.objects.map(({fileId,...o})=>o)},{...snapshot,objects:snapshot.objects.map(({fileId,...o})=>o)},'Portable map changed layout or transforms');
 assert(await cold.evaluate(()=>worldObjects.list.every(o=>!!o.base&&!o.missing)&&terrainRay([worldObjects.list[0].pos[0],0,10],[0,0,-1])<10),'Imported models missing graphics or collision');
 const unchanged=await cold.evaluate(()=>JSON.stringify(mapSnapshot()));
 for(const mutate of [d=>{d.version=2;},d=>{d.world.objects[0].boxes[0]='bad';},d=>{d.files[0].files[0].data='@@@@';},d=>{d.world.environment.wind=-1;}]){
   const invalid=JSON.parse(portable);mutate(invalid);
   await cold.locator('#mapFile').setInputFiles({name:'invalid.json',mimeType:'application/json',buffer:Buffer.from(JSON.stringify(invalid))});
   await cold.waitForFunction(()=>!maps.busy&&$('#mapSay').textContent.startsWith('Could not complete'));
   assert.strictEqual(await cold.evaluate(()=>JSON.stringify(mapSnapshot())),unchanged,'Rejected import changed the world');
   assert(await cold.evaluate(()=>maps.list.length===1),'Rejected import changed map library');
 }
 await cold.reload({waitUntil:'networkidle'});await cold.waitForFunction(()=>fleet.ready&&maps.ready&&worldObjects.list.length===3&&worldObjects.list.every(o=>o.base&&!o.loading));
 assert(await cold.evaluate(()=>maps.list.length===1&&$('#terrainSel').value===maps.current),'Imported map not saved for reload');
 await cold.locator('#mapDelete').click();await cold.waitForFunction(()=>!maps.busy&&maps.list.length===0);
 assert.strictEqual(await cold.evaluate(()=>JSON.stringify(mapSnapshot())),unchanged,'Deleting a saved map changed the active world');
 await cold.setViewportSize({width:390,height:844});await cold.evaluate(()=>{document.documentElement.classList.add('phone');$('.bar').classList.add('open');});
 assert(await cold.evaluate(()=>$('#terrainSel').offsetParent!==null&&document.documentElement.scrollWidth<=innerWidth+1),'Phone map selector hidden or overflowing');
 await cold.screenshot({path:'/tmp/liftlab-maps-phone.png'});
 await page.screenshot({path:'/tmp/liftlab-maps-desktop.png'});
 await fresh.close();
 console.log('Maps: portable import in a fresh browser, graphics/collisions, atomic validation, delete and phone layout');

 // Multi-file glTF: buffers and textures travel together, even without any source IndexedDB.
 const model=glbBox(1,3,1),jsonLength=model.readUInt32LE(12),gltf=JSON.parse(model.subarray(20,20+jsonLength));
 gltf.buffers[0].uri='mesh.bin';gltf.images=[{uri:'color.png'}];gltf.textures=[{source:0}];
 gltf.materials=[{pbrMetallicRoughness:{baseColorTexture:{index:0}}}];gltf.meshes[0].primitives[0].material=0;
 const texturedDoc=JSON.parse(portable),fileId=texturedDoc.world.objects[0].fileId;
 texturedDoc.files.find(f=>f.id===fileId).files=[
   {name:'mesh.gltf',data:Buffer.from(JSON.stringify(gltf)).toString('base64')},
   {name:'mesh.bin',data:model.subarray(20+jsonLength+8).toString('base64')},
   {name:'color.png',data:''}
 ];
 const textureContext=await browser.newContext(),texturePage=await textureContext.newPage();
 texturePage.on('pageerror',e=>errors.push(e.stack));await texturePage.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());
 await texturePage.goto(url,{waitUntil:'networkidle'});await texturePage.waitForFunction(()=>fleet.ready&&maps.ready);
 texturedDoc.files.find(f=>f.id===fileId).files[2].data=await texturePage.evaluate(()=>{const c=document.createElement('canvas');c.width=c.height=1;c.getContext('2d').fillRect(0,0,1,1);return c.toDataURL('image/png').split(',')[1];});
 assert(await texturePage.evaluate(async d=>mapImport(new File([JSON.stringify(d)],'textured-map.json')),texturedDoc),'Textured map import failed');
 await texturePage.waitForFunction(()=>worldObjects.list.every(o=>o.base&&!o.loading));
 assert(await texturePage.evaluate(()=>{let found=false;worldObjects.list[0].base.traverse(o=>{if(o.material?.map?.image?.width===1)found=true;});return found;}),'Multi-file glTF texture missing');
 const packed=await texturePage.evaluate(async()=>{const m=await mapCollect('Texture check');return m.files.flatMap(f=>f.files.map(a=>({name:a.name,data:mapEncode(a.data)})));});
 assert.deepStrictEqual(packed.filter(f=>['mesh.gltf','mesh.bin','color.png'].includes(f.name)),texturedDoc.files.find(f=>f.id===fileId).files,'Texture or buffer bytes changed during re-export');
 const guarded=await texturePage.evaluate(()=>JSON.stringify(mapSnapshot()));
 await texturePage.evaluate(async()=>{agent.busy=true;try{await mapSelect('open');}finally{agent.busy=false;}});
 assert.strictEqual(await texturePage.evaluate(()=>JSON.stringify(mapSnapshot())),guarded,'Map switch ignored an active drone operation');
 // Browser quota failure: import applies but reports that it could not save; saving never pretends to succeed.
 await texturePage.evaluate(()=>{window.originalMapDbDo=mapDbDo;mapDbDo=async()=>{throw new Error('Quota exceeded');};});
 const nMaps=await texturePage.evaluate(()=>maps.list.length);
 assert(!await texturePage.evaluate(()=>mapSave()),'Failed map save reported success');
 assert.strictEqual(await texturePage.evaluate(()=>maps.list.length),nMaps,'Failed save altered the library');
 assert(await texturePage.evaluate(async d=>mapImport(new File([JSON.stringify(d)],'map.json')),document),'Import should still apply when saving is unavailable');
 assert(await texturePage.evaluate(()=>$('#mapSay').textContent.includes('could not save')&&maps.current===null),'Storage failure was not reported');
 await texturePage.evaluate(()=>{mapDbDo=window.originalMapDbDo;});await textureContext.close();
 console.log('Maps: multi-file glTF buffers/textures, re-export bytes, active-operation guard and quota-failure reporting');

 // Missing file (another browser): drawn as its boxes, can be moved but not turned.
 await page.evaluate(async()=>{for(const o of worldObjects.list){worldObjects.files.delete(o.fileId);await worldFileDelete(o.fileId);}worldObjectsRestore(worldObjectsSnapshot());});
 await page.waitForFunction(()=>worldObjects.list.every(o=>!o.loading));
 assert(await page.evaluate(()=>worldObjects.list.every(o=>o.missing&&!o.base&&o.g.children[0].isInstancedMesh)),'Missing files should draw the solid shape');
 await page.selectOption('#droneSelect','');await page.locator('#worldObjEdit').click();
 await page.evaluate(()=>worldEditSelect(worldObjects.list[0].id));
 assert(await page.evaluate(()=>$('#worldObjYaw').disabled&&!$('#worldObjPos0').disabled&&!!$('.world-obj-note')),'Missing model should allow moving only, with a note');
 // Duplicate and remove.
 await page.getByRole('button',{name:'Duplicate'}).click();
 assert(await page.evaluate(()=>worldObjects.list.length===4&&wedit.sel===worldObjects.list[3].id),'Duplicate failed');
 await page.locator('#worldObjRemove').click();
 assert(await page.evaluate(()=>worldObjects.list.length===3&&!wedit.sel),'Remove failed');
 // Picking a drone ends the edit.
 await page.selectOption('#droneSelect',await page.evaluate(()=>fleet.drones[0].id));
 assert(await page.evaluate(()=>!wedit.on&&fleet.selected),'Selecting a drone should end editing objects');
 assert(!errors.length,'Page errors:\n'+errors.join('\n'));
 console.log('World objects: all checks passed');
}finally{if(browser)await browser.close();server.close();}})().catch(e=>{console.error(e);process.exit(1);});
