#!/usr/bin/env node
'use strict';
// Imported drone solids, editing, mounting, mass properties and portable designs.
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
const comparable=d=>{delete d.computers.nextBoardId;return d;};
const near=(a,b,eps=1e-7)=>assert(Math.abs(a-b)<eps,`${a} != ${b}`);
const vector=(a,b,eps)=>a.forEach((v,k)=>near(v,b[k],eps));
(async()=>{await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));let browser;
try{
 browser=await chromium.launch({executablePath:process.env.CHROME_PATH||'/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const context=await browser.newContext({viewport:{width:1600,height:1000}}),page=await context.newPage(),errors=[];page.on('pageerror',e=>{errors.push(e.stack);console.error(e.stack);});
 await page.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());await page.goto('http://127.0.0.1:'+server.address().port,{waitUntil:'networkidle'});await page.waitForFunction(()=>typeof fleet!=='undefined'&&fleet.ready&&brt.ready&&objectLibrary.ready);
 await page.evaluate(()=>{setTerrain('open',1);running=false;renderRun();});
 await page.locator('#addPart').click();await page.locator('#partModelImport').click();await page.locator('#partModelFile').setInputFiles({name:'reusable.glb',mimeType:'application/octet-stream',buffer:glbBox(2,1,1)});await page.waitForFunction(()=>partImport.draft&&!partImport.busy);
 assert.strictEqual(await page.locator('#partImportMass').count(),0,'Import should not edit physical mass');assert.strictEqual(await page.locator('#objectRole').count(),0);
 await page.locator('#partImportName').fill('Shared object');await page.locator('#partImportSize').fill('.2');await page.locator('#partImportSize').dispatchEvent('change');
 await page.locator('#objectImportSave').click();assert(await page.evaluate(()=>cfg.comps.every(c=>!c.model)&&worldObjects.list.length===0));await page.locator('#objectLibraryDlg [data-object-use]').first().click();await page.waitForFunction(()=>partImport.phase==='use'&&partImport.draft&&!partImport.busy);
 const assetId=await page.evaluate(()=>objectLibrary.list[0].id);assert(await page.evaluate(()=>objectLibrary.list.length===1&&cfg.comps.every(c=>!c.model)));
 await page.locator('#objectRole').selectOption('battery');await page.locator('[data-collision=mesh]').click();assert(await page.evaluate(()=>partImport.solid.geometry.attributes.position.count===partImport.draft.model.triangles.length/3&&partImport.solid.material===worldObjMats.solid&&collisionMesh(partImport.draft.model.triangles).closed));
 await page.locator('#partImportMass').fill('.31');await page.locator('#partImportMass').dispatchEvent('change');
 if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-drone.png'});
 const physical=page.locator('.part-import-panel details').filter({hasText:'Physical model'});await physical.locator('summary').click();
 await page.evaluate(()=>{partImport.draft.dragCd=1.7;});await page.locator('#partImportPointAdd').click();
 await page.locator('#partImportAdd').click();await page.waitForFunction(()=>!partImport.active);
 let r=await page.evaluate(()=>{const c=cfg.comps.find(c=>c.model);return {battery:isBattery(c),fileId:c.model.fileId,mass:c.mass,drag:c.dragCd,points:c.points.length,collision:c.model.collision,shape:fleetShapesCurrent().boxes.some(b=>b.mesh&&b.part===c)};});assert(r.battery&&r.fileId===assetId&&r.mass===.31&&r.drag===1.7&&r.points===2&&r.collision==='mesh'&&r.shape);
 const original=await page.evaluate(()=>designSnap());
 await page.locator('#objectEditCopy-'+await page.evaluate(()=>cfg.comps.find(c=>c.model).id)).click();await page.waitForFunction(()=>partImport.draft&&!partImport.busy);await page.locator('#partImportMass').fill('9');await page.locator('#partImportMass').dispatchEvent('change');await page.locator('#partImportCancel').click();assert.strictEqual(await page.evaluate(()=>designSnap()),original,'Canceled edits mutated the drone');
 // The copy editor preserves mounting relationships and commits only on Save.
 const copyId=await page.evaluate(()=>cfg.comps.find(c=>c.model).id);
 await page.evaluate(id=>{const c=compById(id),child=mkMass('Mounted child',0,0,0,{mass:.01});cfg.comps.push(child);attachTo(child,c,c.points[1].id);structural();window.libraryChild=child.id;},copyId);
 const withChild=await page.evaluate(()=>designSnap());await page.locator('#objectEditCopy-'+copyId).click();await page.waitForFunction(()=>partImport.draft&&!partImport.busy);
 assert(await page.evaluate(()=>!partMountOptions(partImport.draft).some(([value])=>value===String(partImport.editId)||value===String(window.libraryChild))),'Copy editor offered a cyclic parent');
 await page.locator('#partImportSize').fill('.4');await page.locator('#partImportSize').dispatchEvent('change');await page.locator('#objectCopyRotation2').fill('45');await page.locator('#objectCopyRotation2').dispatchEvent('change');
 assert.strictEqual(await page.evaluate(()=>designSnap()),withChild,'Draft size/rotation moved real children');
 await page.locator('#partImportAdd').click();await page.waitForFunction(()=>!partImport.active);
 assert(await page.evaluate(id=>{const c=compById(id),child=compById(window.libraryChild);return c.mass===.31&&Math.abs(Math.max(...c.size)-.4)<1e-8&&nrm(sub(child.pos,partPointRest(c,c.points[1].id)))<1e-8&&nrm(partPointRest(c,c.selfPoint))<1e-8;},copyId));
 await page.locator('[data-id="'+copyId+'"] button').filter({hasText:'Save drone use defaults'}).click();await page.waitForFunction(()=>objectLibrary.list[0].droneCollision==='mesh');
 await page.locator('#addPart').click();await page.locator('#objectLibraryOpen').click();await page.locator('[data-object-use="'+assetId+'"]').click();await page.waitForFunction(()=>partImport.draft&&!partImport.busy);
 assert(await page.evaluate(()=>partImport.draft.battery&&partImport.draft.mass===.31&&partImport.draft.model.collision==='mesh'&&Math.abs(Math.max(...partImport.draft.size)-.4)<1e-8));
 await page.locator('#objectRole').selectOption('wing');assert(await page.evaluate(()=>isWing(partImport.draft)&&!isBattery(partImport.draft)));await page.locator('#partImportCancel').click();
 await page.selectOption('#droneSelect','');await page.locator('#worldObjEdit').click();await page.locator('#worldObjectLibrary').click();await page.locator('[data-object-use="'+assetId+'"]').click();await page.waitForFunction(()=>worldObjects.list.length===1&&!wedit.busy);
 await page.locator('#worldObjBody [data-collision=mesh]').click();assert(await page.evaluate(()=>{const o=worldObjects.list[0];return o.fileId===objectLibrary.list[0].fileId&&o.collision==='mesh'&&terrainNear([0,0,.05],2).some(b=>b.mesh)&&solidVis.geometry.attributes.position.count===o.triangles.length/3;}));
 assert(await page.evaluate(()=>{const d=fleet.drones[0],c=d.state.cfg.comps.find(c=>c.model);return c.mass===.31&&c.battery&&c.model.collision==='mesh'&&objectLibrary.list[0].scale===.1;}),'World use changed drone or shared import settings');
 // Real collision/ray tests: a tetrahedron leaves its bounding-box corner empty and has an oblique normal.
 r=await page.evaluate(()=>{
   const tri=[0,0,0, 0,1,0, 1,0,0, 0,0,0, 1,0,0, 0,0,1, 0,0,0, 0,0,1, 0,1,0, 1,0,0, 0,1,0, 0,0,1],mesh=collisionMesh(tri),origin=[4,4,1],shape={mesh,origin,lo:origin,hi:add(origin,[1,1,1]),what:'tetra'};
   const contact=boxContact(add(origin,[.4,.4,.25]),.08,shape),empty=boxContact(add(origin,[.9,.9,.9]),.02,shape);
   const R=eulerR(0,0,37),target=fleetMesh(origin,R,tri,null),ball={radius:.08,st:{p:add(origin,m3v(R,[.4,.4,.25]))}},f=fleetSphereBox(ball,target);
   const no=fleetBoxContact(fleetBox(add(origin,m3v(R,[.9,.9,.9])),R,[.02,.02,.02]),target),yes=fleetBoxContact(fleetBox(add(origin,m3v(R,[.3,.3,.4])),R,[.1,.1,.1]),target);
   terrainSetObjectBoxes([{...shape,obj:'tetra-test'}]);
   const worldRay=terrainRay(add(origin,[.25,.25,2]),[0,0,-1]),below=surfaceBelow(add(origin,[.25,.25,2])),walls=terrainWalls(add(origin,[.25,.25,2]),add(origin,[.25,.25,-1]),2),insideName=solidAt(add(origin,[.1,.1,.1]),terrainNear(add(origin,[.1,.1,.1]),.1));worldObjectsChanged();
   const c=mkMass('Tetra',0,0,0,{shape:'model',size:[1,1,1],model:{fileId:'tetra-test',up:'z',scale:1,rawSize:[1,1,1],detail:'medium',collision:'mesh',triangles:tri,boxes:[[0,0,0,1,1,1]]}}),sibling=mkMass('Sibling',2,0,0,{size:[.1,.1,.1]}),L=looseBody('Mixed released mesh',[c,sibling]);L.p=L.cm.slice();L.asleep=true;const cargoBefore=cargo.loose;cargo.loose=[L];
   const near=cargoSolids([.9,.9,.9],.01),looseEmpty=near.some(b=>boxContact([.9,.9,.9],.01,b));cargo.loose=cargoBefore;
   const crossing=meshContact([.4,.4,.15],0,mesh,[.4,.4,.4]);
   const openShell=collisionMesh(tri.slice(9)),openInside=meshInside(openShell,[.1,.1,.1]);
   const open=collisionMesh([0,0,0,1,0,0,0,1,0]),thin=meshContact([.2,.2,-.02],0,open,[.2,.2,.02]);
   return {openInside,worldRay,below,walls,insideName,looseEmpty,looseMeshes:near.filter(b=>b.mesh).length,inside:meshInside(mesh,[.1,.1,.1]),emptyInside:meshInside(mesh,[.9,.9,.9]),contact,empty,f,no,yes,crossing,thin,ray:meshRayHits(mesh,[.25,.25,2],[0,0,-1])[0].distance};
 });assert(!r.openInside&&r.inside&&!r.emptyInside&&r.contact&&!r.empty&&r.f&&!r.no&&r.yes&&r.crossing&&r.thin,JSON.stringify(r));near(r.ray,1.5);near(r.worldRay,1.5);near(r.below,1.5);assert(r.walls===1&&r.insideName==='tetra'&&!r.looseEmpty&&r.looseMeshes===1);vector(r.contact.n,[1/Math.sqrt(3),1/Math.sqrt(3),1/Math.sqrt(3)],1e-5);assert(r.thin.n[2]>0&&r.crossing.n[2]>0);console.log('Triangle sphere/box contacts, rotated dynamic mesh, empty corners, sloped normals, thin-surface crossing and ray passed');
 if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-world.png'});
 const portable=await page.evaluate(async()=>{const m=await mapCollect('Mesh world');return {format:MAP_FORMAT,version:1,...m,files:m.files.map(f=>({...f,files:f.files.map(p=>({name:p.name,data:mapEncode(p.data)}))}))};});
 assert(portable.world.objects[0].collision==='mesh'&&portable.world.objects[0].triangles.length>0);assert(await page.evaluate(doc=>mapDecode(doc).world.objects[0].collision==='mesh',portable));
 assert(await page.evaluate(doc=>{doc.world.objects[0].triangles[0]=NaN;try{mapDecode(doc);return false;}catch{return true;}},objectCloneForTest(portable)));
 // Remove from library, preserve both copies and their original source bytes across reload.
 await page.locator('#worldObjectLibrary').click();await page.locator('[aria-label="Remove library object Shared object"]').click();await page.waitForFunction(()=>objectLibrary.list.length===0);assert.strictEqual(await page.locator('[data-object-use]').count(),0);await page.locator('#objectLibraryDlg button').filter({hasText:'Close'}).click();
 assert(await page.evaluate(async id=>!!await worldFileGet(id),assetId));await page.reload({waitUntil:'networkidle'});await page.waitForFunction(()=>typeof fleet!=='undefined'&&fleet.ready&&objectLibrary.ready);assert(await page.evaluate(()=>objectLibrary.list.length===0&&worldObjects.list[0].collision==='mesh'&&fleet.drones[0].state.cfg.comps.find(c=>c.model).model.collision==='mesh'));
 const stable=await page.evaluate(()=>{running=false;return withDrone(fleet.drones[0],()=>{resetSim();let finite=true;for(let i=0;i<2/PDT;i++){physStep();if(![...S.p,...S.v,...S.w,...S.q,...S.acc].every(Number.isFinite)||String(S.crashed).includes('NaN')){finite=false;break;}}return {finite,crashed:S.crashed,points:cPts.length};});});assert(stable.finite,'Reloaded mesh contacts became invalid '+JSON.stringify(stable));
 // Legacy templates migrate with source files, mounting data and use defaults.
 await page.evaluate(async id=>{const c=objectClone(fleet.drones[0].state.cfg.comps.find(c=>c.model)),rec=await worldFileGet(id);rec.id='legacy-file';await worldFilePut(rec);c.model.fileId=rec.id;c.name='Legacy model';localStorage.setItem(PART_LIBRARY_LS,JSON.stringify([c]));},assetId);
 await page.reload({waitUntil:'networkidle'});await page.waitForFunction(()=>objectLibrary.list.some(a=>a.name==='Legacy model'));
 assert(await page.evaluate(()=>{const a=objectLibrary.list.find(a=>a.name==='Legacy model');return a.droneDefaults.mass===.31&&a.droneDefaults.points.length===2;}));
 await page.setViewportSize({width:390,height:844});await page.selectOption('#droneSelect',await page.evaluate(()=>fleet.drones[0].id));await page.locator('#addPart').click();await page.locator('#objectLibraryOpen').click();await page.locator('[data-object-use=legacy-file]').click();await page.waitForFunction(()=>partImport.draft&&!partImport.busy);
 await page.evaluate(()=>{for(let i=0;i<60;i++)fleetScene();renderer.render(scene,camera);});
 const bounds=await page.locator('.part-import-panel').boundingBox();assert(bounds.x>=0&&bounds.x+bounds.width<=391&&bounds.y+bounds.height<=845);
 if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-phone-check.png'});
 const visible=await page.evaluate(()=>{const view=vpEl.getBoundingClientRect(),panel=partImport.panel.getBoundingClientRect();return [[0,0,0],partImport.draft.pos].map(p=>{const q=drone.localToWorld(new THREE.Vector3(...p)).project(camera),x=view.left+(q.x+1)*view.width/2,y=view.top+(1-q.y)*view.height/2;return {x,y,view:{left:view.left,right:view.right,top:view.top},panel:panel.top,element:Number.isFinite(x)&&Number.isFinite(y)?document.elementFromPoint(x,y)?.className:'nonfinite',ok:x>view.left&&x<view.right&&y>view.top&&y<panel.top&&document.elementFromPoint(x,y)===renderer.domElement,target:cam.target.toArray(),dist:cam.dist,reach:cReach,projection:camera.type,hub:S.p,preview:partImport.draft.pos,scale:partImport.draft.model.scale,size:partImport.draft.size};});});assert(visible.every(p=>p.ok),'Phone drone/preview must remain uncovered '+JSON.stringify(visible));
 if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-phone.png'});
 await page.locator('#partImportCancel').click();
 assert.deepStrictEqual(errors,[]);console.log('One shared import, battery/drag/mounts, independent world/drone copies, cancel, portable mesh validation and deletion/reload retention passed');
}finally{if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
function objectCloneForTest(v){return JSON.parse(JSON.stringify(v));}
