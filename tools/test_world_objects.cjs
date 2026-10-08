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
 assert(await page.evaluate(()=>wedit.on&&!running&&!$('#worldEditBar').hidden&&$('#terrainSel').offsetParent===null&&$('#worldPanel').classList.contains('editing-objects')),'Edit objects did not pause and open the object editor');

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
