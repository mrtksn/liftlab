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
 browser=await chromium.launch({executablePath:process.env.CHROME_PATH || '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 const context=await browser.newContext({viewport:{width:1600,height:1000}}),page=await context.newPage(),errors=[];
 page.on('pageerror',e=>{errors.push(e.stack);console.error(e.stack);});
 await page.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());
 const url=process.env.LIVE_URL || 'http://127.0.0.1:'+server.address().port;
 await page.goto(url,{waitUntil:'networkidle'});await page.waitForFunction(()=>typeof fleet!=='undefined' && fleet.ready && brt.ready);
 await page.evaluate(()=>{setTerrain('open',1);running=true;renderRun();});
 const importModel=async(name,buffer)=>{
   await page.locator('#addPart').click();
   assert.strictEqual(await page.locator('#partModelImport').evaluate(e=>!!e.closest('#addPartDlg section')),false,'Import belongs above the categories');
   await page.locator('#partModelImport').click();
   await page.locator('#partModelFile').setInputFiles({name,mimeType:'application/octet-stream',buffer});
   await page.waitForFunction(()=>partImport.draft&&!partImport.busy);
   await page.locator('#partImportSize').fill('.2');await page.locator('#partImportSize').dispatchEvent('change');
   await page.locator('#objectImportSave').click();await page.locator('#objectLibraryDlg [data-object-use]').first().click();await page.waitForFunction(()=>partImport.phase==='use'&&partImport.draft&&!partImport.busy);
 };
 const before=await page.evaluate(()=>designSnap());
 await importModel('camera.glb',glbBox(2,1,1));
 let r=await page.evaluate(()=>({size:partImport.draft.size,mass:partImport.draft.mass,cog:partImport.draft.cog,category:partImport.draft.category,unchanged:designSnap(),paused:!running,locked:!fleetSelect(null),preview:partImport.preview.position.toArray(),reach:cReach}));
 vector(r.size,[.2,.1,.1]);vector(r.cog,[0,0,0],.007);assert(r.mass>0&&r.mass<2&&r.category==='payload'&&r.paused&&r.locked&&r.preview[0]>r.reach);assert.strictEqual(r.unchanged,before);
 assert(await page.evaluate(()=>partImport.panel.parentElement===$('.view')&&partImport.solid.visible&&partImport.solid.material===worldObjMats.solid));
 assert(await page.evaluate(()=>{
   const M=new THREE.Matrix4(),p=new THREE.Vector3(),q=new THREE.Quaternion(),s=new THREE.Vector3();
   return partBoxes(partImport.draft).every((b,i)=>{partImport.solid.getMatrixAt(i,M);M.decompose(p,q,s);return p.distanceTo(new THREE.Vector3(...partBoxCenter(b)))<1e-6&&s.distanceTo(new THREE.Vector3(...partBoxSize(b)).multiplyScalar(1.002))<1e-6;});
 }),'Displayed boxes do not match physics');
 await page.locator('#partImportSolid').uncheck();assert(await page.evaluate(()=>!partImport.solid.visible&&partImport.draft.model.boxes.length>0));await page.locator('#partImportSolid').check();
 // The shared slider changes geometry live and synchronizes the number.
 await page.locator('#partImportScale').evaluate(e=>{e.value='.4';e.dispatchEvent(new Event('input',{bubbles:true}));});
 vector(await page.evaluate(()=>partImport.draft.size),[.4,.2,.2]);near(+await page.locator('#partImportSize').inputValue(),.4);
 await page.locator('#partImportSize').fill('.2');await page.locator('#partImportSize').dispatchEvent('change');near(+await page.locator('#partImportScale').inputValue(),.2);
 const geometry=await page.evaluate(()=>({cog:partImport.draft.cog,points:partImport.draft.points}));
 for(const detail of ['coarse','fine','medium']){
   await page.locator('.part-import-panel [data-solid-detail="'+detail+'"]').click();await page.waitForFunction(()=>!partImport.busy);
   assert(await page.evaluate(detail=>partImport.draft.model.detail===detail&&partImport.solid.count===partImport.draft.model.boxes.length,detail));
   assert.deepStrictEqual(await page.evaluate(()=>({cog:partImport.draft.cog,points:partImport.draft.points})),geometry);
 }
 await page.locator('#partImportCancel').click();assert(await page.evaluate(()=>running&&!editMode&&fleet.pendingEdits===0&&!$('#airframe').inert));assert.strictEqual(await page.evaluate(()=>designSnap()),before);
 assert(await page.evaluate(()=>!perspCam.view?.enabled&&!orthoCam.view?.enabled),'Import camera offset survived cancel');
 // A canceled asynchronous read cannot overwrite a newer draft or unlock its owner.
 assert(await page.evaluate(async()=>{
   let finish;const slow=partImportStart([{name:'old.glb',arrayBuffer:()=>new Promise(resolve=>finish=resolve)}]);partImportCancel();
   const newer=partImportStart([{name:'new.obj',arrayBuffer:async()=>new TextEncoder().encode('v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3').buffer}]);
   await newer;finish(new Uint8Array([1,2,3]).buffer);await slow;
   const ok=partImport.draft?.name==='new'&&partImport.active&&fleet.pendingEdits===1&&!partImport.busy;partImportCancel();return ok;
 }));
 await importModel('camera.glb',glbBox(2,1,1));
 await page.locator('#partImportName').fill('Survey camera');await page.locator('#partImportCategory').selectOption('sensors');
 await page.locator('#partImportSize').fill('.3');await page.locator('#partImportSize').dispatchEvent('change');
 await page.locator('#partImportMass').fill('.25');await page.locator('#partImportMass').dispatchEvent('change');
 await page.locator('#partImportCog0').fill('.025');await page.locator('#partImportCog0').dispatchEvent('change');
 for(let i=0;i<2;i++)await page.locator('#partImportPointAdd').click();
 await page.evaluate(()=>{partImport.draft.points[1].name='Left mount';partImport.draft.points[1].pos=[0,.075,0];partImport.draft.points[2].name='Right mount';partImport.draft.points[2].pos=[0,-.075,0];partImportRenderVectors();partDraftRefresh();});
 // Surface picking uses visible model triangles, excluding the CoM/mount markers.
 await page.locator('#partImportVectors button').filter({hasText:'Pick center on object'}).click();
 const click=await page.evaluate(()=>{fleetScene();renderer.render(scene,camera);const r=vpEl.getBoundingClientRect(),c=partImport.draft,p=drone.localToWorld(new THREE.Vector3(...c.pos)).project(camera);return {x:r.left+(p.x+1)*r.width/2,y:r.top+(1-p.y)*r.height/2};});
 if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-before-pick.png'});
 
 await page.mouse.click(click.x,click.y);assert(await page.evaluate(()=>partImport.pick===null),'Surface click missed');
 await page.locator('#partImportCog0').fill('.025');await page.locator('#partImportCog0').dispatchEvent('change');
 for(const k of [1,2]){await page.locator('#partImportCog'+k).fill('0');await page.locator('#partImportCog'+k).dispatchEvent('change');}
 if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-preview.png'});
 await page.locator('#partImportAdd').click();await page.waitForFunction(()=>!partImport.active);
 const id=await page.evaluate(()=>cfg.comps.find(c=>c.model).id);
 r=await page.evaluate(id=>{const c=compById(id);return {name:c.name,category:c.category,mass:c.mass,points:c.points.length,anchor:partPointRest(c,c.selfPoint),cog:c.cog,contacts:partSolidContacts(c).length,boxes:c.model.boxes.length,geometry:pickGroups.get(c.id).children.length};},id);
 assert(r.name==='Survey camera'&&r.category==='sensors'&&r.mass===.25&&r.points===3&&r.contacts>0&&r.boxes>0&&r.geometry>0);vector(r.anchor,[0,0,0]);vector(r.cog,[.025,0,0]);
 // Mounting, resize and moving a point preserve child and own anchors once, including grandchildren.
 await page.evaluate(id=>{const c=compById(id),m=mkMotor('Mounted motor',0,0,0);cfg.comps.push(m);attachTo(m,c,c.points[2].id);structural();window.mountMotor=m.id;selectComp(c.id);},id);
 await page.locator('#part-size-'+id).fill('.6');await page.locator('#part-size-'+id).dispatchEvent('change');
 r=await page.evaluate(id=>{const c=compById(id),m=compById(window.mountMotor);return {anchor:partPointRest(c,c.selfPoint),child:m.pos,target:partPointRest(c,c.points[2].id),mass:c.mass,cog:c.cog};},id);
 vector(r.anchor,[0,0,0]);vector(r.child,r.target);near(r.mass,.25);near(r.cog[0],.05);
 const point=await page.evaluate(id=>compById(id).points[2].id,id);
 await page.locator('#part-point-'+id+'-'+point+'-1').fill('-.17');await page.locator('#part-point-'+id+'-'+point+'-1').dispatchEvent('change');
 r=await page.evaluate(id=>{const c=compById(id);return {child:compById(window.mountMotor).pos,target:partPointRest(c,c.points[2].id)};},id);vector(r.child,r.target);
 await page.locator('#part-point-'+id+'-top-2').fill('.12');await page.locator('#part-point-'+id+'-top-2').dispatchEvent('change');
 r=await page.evaluate(id=>{const c=compById(id);return {anchor:partPointRest(c,c.selfPoint),child:compById(window.mountMotor).pos,target:partPointRest(c,c.points[2].id)};},id);vector(r.anchor,[0,0,0]);vector(r.child,r.target);
 await page.locator('#part-rotation-'+id+'-2').fill('90');await page.locator('#part-rotation-'+id+'-2').dispatchEvent('change');
 r=await page.evaluate(id=>{const c=compById(id);return {child:compById(window.mountMotor).pos,target:partPointRest(c,c.points[2].id),cycle:canAttach(c,compById(window.mountMotor))};},id);vector(r.child,r.target);assert(!r.cycle);
 await page.locator('#part-rotation-'+id+'-0').fill('45');await page.locator('#part-rotation-'+id+'-0').dispatchEvent('change');
 r=await page.evaluate(id=>{const c=compById(id);return {anchor:partPointRest(c,c.selfPoint),child:compById(window.mountMotor).pos,target:partPointRest(c,c.points[2].id)};},id);vector(r.anchor,[0,0,0]);vector(r.child,r.target);
 const snap=await page.evaluate(()=>designSnap());await page.evaluate(()=>undoStep());assert.notStrictEqual(await page.evaluate(()=>designSnap()),snap);await page.evaluate(()=>redoStep());assert.deepStrictEqual(comparable(await page.evaluate(()=>JSON.parse(designSnap()))),comparable(JSON.parse(snap)));
 // The selected-part picker remains available after import, and its library entry is reusable.
 await page.evaluate(id=>partPickBegin(compById(id),'cog'),id);await page.waitForTimeout(700);
 const partClick=await page.evaluate(id=>{fleetScene();renderer.render(scene,camera);const r=vpEl.getBoundingClientRect(),p=drone.localToWorld(new THREE.Vector3(...compById(id).pos)).project(camera);return {x:r.left+(p.x+1)*r.width/2,y:r.top+(1-p.y)*r.height/2};},id);
 await page.mouse.click(partClick.x,partClick.y);assert(await page.evaluate(()=>!partEditPick));await page.evaluate(()=>undoStep());
 await page.locator('[data-id="'+id+'"] button').filter({hasText:'Save drone use defaults'}).click();
 await page.locator('#addPart').click();await page.locator('#objectLibraryOpen').click();await page.locator('#objectLibraryDlg [data-object-use]').filter({hasText:'Use on drone'}).first().click();await page.waitForFunction(()=>partImport.draft&&!partImport.busy);await page.locator('#partImportAdd').click();await page.waitForFunction(()=>!partImport.active);
 assert(await page.evaluate(()=>{const copies=cfg.comps.filter(c=>c.model);return copies.length===2&&copies[1].name===copies[0].name&&copies[1].id!==copies[0].id&&nrm(partPointRest(copies[1],copies[1].selfPoint))<1e-6;}));await page.evaluate(()=>undoStep());
 console.log('GLB preview/defaults, surface picking, cancel races, multiple mounts, CoM, resize, rotation and undo passed');
 // Independent analytic rigid-box properties, translated geometry, articulated bodies and rod endpoints.
 r=await page.evaluate(()=>{
   const original=JSON.parse(designSnap()),c=mkMass('Analytic box',1,2,3,{mass:2,shape:'model',size:[.4,.2,.1],cog:[.03,0,0],rotation:[0,0,90],model:{fileId:'analytic',rawSize:[.4,.2,.1],scale:1,up:'z',detail:'medium',boxes:[[-.2,-.1,-.05,.2,.1,.05]]}});
   cfg.frame.mass=1;cfg.frame.cog=[.1,0,0];cfg.comps=[c];cargo.off.clear();cargo.extra=[];recomputeProps();
   const p=massProps('truth'),J=shapeI(c),cm=partMassRest(c),body=MB.bodies[0].I.c,contact=partSolidContacts(c)[0].rest;
   const joint=mkJoint('Hinge',0,0,0,{mass:0,hingeAz:0,hingeEl:90,mode:'manual',manual:90}),rot=JSON.parse(JSON.stringify(c));rot.parent=joint.id;cfg.frame.cog=[0,0,0];cfg.comps=[joint,rot];recomputeProps();const articulated=massProps('truth').c;
   const l=mkLink('Rod',0,0,0,{length:.2,az:0,el:0}),a=mkMass('Base',0,0,0),b=mkMass('Tip',0,0,0);cfg.comps=[l,a,b];attachTo(a,l,'base');attachTo(b,l,'tip');structural();l.length=.4;edited(l,'length');const rod=[a.pos,b.pos];
   const cpts=JSON.parse(JSON.stringify(c));cpts.rotation=[0,0,0];const loose=looseBody('Dropped box',[cpts]);
   applyDesign(original);structural();return {p,J,cm,body,contact,articulated,rod,loose:{mass:loose.m,cm:loose.cm,pts:loose.pts.length}};
 });
 vector(r.cm,[1,2.03,3]);vector(r.p.c,[(.1+2)/3,4.06/3,2]);vector(r.body,r.p.c);
 near(r.J[0],2*(.4*.4+.1*.1)/12+2*.03*.03);near(r.J[4],2*(.2*.2+.1*.1)/12);near(r.J[8],2*(.4*.4+.2*.2)/12+2*.03*.03);
 vector(r.articulated,[-4.06/3,2/3,2]);vector(r.rod[0],[0,0,0]);vector(r.rod[1],[.4,0,0]);vector(r.loose.cm,[1.03,2,3]);assert(r.loose.pts>0);near(r.loose.mass,2);
 console.log('Analytic mass/inertia, frame and rigid CoM, servo poses, rod Base/Tip and loose solid passed');
 r=await page.evaluate(async id=>{
   const original=JSON.parse(designSnap()),source=compById(id),c=JSON.parse(JSON.stringify(source));
   // Two separated solid blocks must leave a real gap in the collision shape.
   c.pos=[2,0,0];c.rotation=[0,0,0];c.cog=[0,0,0];c.selfPoint=null;c.parent=null;c.model.boxes=[[-.2,-.1,-.1,-.1,.1,.1],[.1,-.1,-.1,.2,.1,.1]];cfg.comps=[c];recomputeProps();
   const solids=fleetShapesCurrent().boxes.filter(b=>b.part===c&&b.half[0]>.03),gap=add(S.p,[2,0,0]),inside=add(S.p,[2.15,0,0]);
   const goodGap=solids.length===2&&!solids.some(b=>fleetPointInBox(gap,b))&&solids.some(b=>fleetPointInBox(inside,b));
   const latch=mkLatch('Test latch',0,0,0),payload=JSON.parse(JSON.stringify(source));payload.pos=[0,0,-.1];payload.parent=latch.id;cfg.comps=[latch,payload];cargo.loose=[];cargo.off.clear();structural();
   const expected=partMassRest(payload),m=payload.mass,n=cargoRelease(latch),L=cargo.loose[0],drop=n===1&&L.m===m&&nrm(sub(L.cm,sub(expected,hookOf(latch))))<1e-8&&L.pts.length>0&&L.parts[0].model.boxes.length>0;
   // A small imported solid is carried by the normal compiled flight controller.
   loadPreset('quadx');running=false;setEditMode(false);running=false;
   const flight=JSON.parse(JSON.stringify(source));flight.id=uid++;flight.parent=null;flight.parentPoint=null;flight.selfPoint=null;flight.rotation=[0,0,0];flight.pos=[0,0,-.07];partScale(flight,.04);flight.mass=.02;flight.cog=[.001,0,0];flight.model.collision='mesh';cfg.comps.push(flight);afterLoad();running=false;
   while(!brt.ready)await new Promise(r=>setTimeout(r,20));
   for(let i=0;i<5/PDT;i++){if(i%20===0)pilotStep(.01);physStep();}
   const airborne=!S.crashed&&brt.fcState===1&&S.p.every(Number.isFinite)&&S.p[2]>.1;
   const result={goodGap,drop,airborne,flight:{p:S.p,crashed:S.crashed,fcState:brt.fcState}};applyDesign(original);afterLoad();running=false;setEditMode(true);selectComp(id);return result;
 },id);assert(r.goodGap&&r.drop&&r.airborne,JSON.stringify(r));
 console.log('Separated solid collision gap, latch release geometry/CoM and five-second compiled-controller flight with mesh contacts passed');
 // Library/file/share assets survive a fresh origin context, rather than only the warm model cache.
 await page.evaluate(()=>{$('#designName').value='Imported camera';});assert(await page.evaluate(()=>saveDesign(true)));
 const downloaded=page.waitForEvent('download');await page.evaluate(()=>exportDesign());const download=await downloaded;
 const exported=JSON.parse(fs.readFileSync(await download.path(),'utf8'));assert(exported.design.modelFiles.length===1);assert.strictEqual(Buffer.from(exported.design.modelFiles[0].files[0].data,'base64').compare(glbBox(2,1,1)),0);
 const code=await page.evaluate(()=>designCode('Camera link'));
 const cold=await browser.newContext({viewport:{width:1600,height:1000}}),other=await cold.newPage();other.on('pageerror',e=>errors.push(e.stack));await other.route(/fonts\.google|goatcounter|gc\.zgo/,r=>r.abort());await other.goto(url,{waitUntil:'networkidle'});await other.waitForFunction(()=>typeof fleet!=='undefined'&&fleet.ready&&brt.ready);
 assert(await other.evaluate(async code=>{const d=await readDesignCode(code);applyDesign(d.design);structural();const c=cfg.comps.find(c=>c.model),g=await partModelBase(c);return !!g&&c.name==='Survey camera'&&c.points.length===3&&c.cog[0]===.05&&c.category==='sensors'&&partSolidContacts(c).length>0;},code));
 await page.reload({waitUntil:'networkidle'});await page.waitForFunction(()=>typeof fleet!=='undefined'&&fleet.ready&&brt.ready);
 assert(await page.evaluate(async()=>{const c=cfg.comps.find(c=>c.model);return !!(await partModelBase(c))&&c.points.length===3;}));
 assert(await page.evaluate(()=>{const saved=designs.list.find(d=>d.name==='Imported camera');return saved.design.modelFiles.length===1;}));
 const same=await page.evaluate(()=>designSnap());assert(await page.evaluate(text=>{const o=JSON.parse(text);o.design.comps.find(c=>c.model).points[0].pos=[NaN,0,0];try{readDesignFile(JSON.stringify(o));return false;}catch{return true;}},JSON.stringify(exported)));assert.strictEqual(await page.evaluate(()=>designSnap()),same);
 console.log('Library, real download, cold-browser shared design, asset bytes, reload and invalid geometry rejection passed');
 const binary=glbBox(2,1,1),jsonLength=binary.readUInt32LE(12),gltf=JSON.parse(binary.subarray(20,20+jsonLength));
 gltf.buffers[0].uri='mesh.bin';gltf.images=[{uri:'color.png'}];gltf.textures=[{source:0}];gltf.materials=[{pbrMetallicRoughness:{baseColorTexture:{index:0}}}];gltf.meshes[0].primitives[0].material=0;
 const png=Buffer.from(await page.evaluate(()=>{const c=document.createElement('canvas');c.width=c.height=1;c.getContext('2d').fillRect(0,0,1,1);return c.toDataURL('image/png').split(',')[1];}),'base64');
 const files=[{name:'mesh.gltf',mimeType:'model/gltf+json',buffer:Buffer.from(JSON.stringify(gltf))},{name:'mesh.bin',mimeType:'application/octet-stream',buffer:binary.subarray(20+jsonLength+8)},{name:'color.png',mimeType:'image/png',buffer:png}];
 await page.locator('#addPart').click();await page.locator('#partModelImport').click();await page.locator('#partModelFile').setInputFiles(files);await page.waitForFunction(()=>partImport.draft&&!partImport.busy);await page.locator('#objectImportSave').click();await page.locator('#objectLibraryDlg [data-object-use]').first().click();await page.waitForFunction(()=>partImport.phase==='use'&&partImport.draft&&!partImport.busy);
 assert(await page.evaluate(async()=>{let textured=false;(await partModelBase(partImport.draft)).traverse(m=>{if(m.material?.map?.image?.width===1)textured=true;});return textured;}));
 await page.locator('#partImportAdd').click();await page.waitForFunction(()=>!partImport.active);
 const packed=await page.evaluate(()=>partDesignWithAssets(JSON.parse(designSnap())));
 assert.deepStrictEqual(packed.modelFiles.find(f=>f.files.length===3).files.map(f=>({name:f.name,data:f.data})),files.map(f=>({name:f.name,data:f.buffer.toString('base64')})));
 assert(await other.evaluate(async design=>{const got=readDesignFile(JSON.stringify({format:FILE_FORMAT,design}));applyDesign(got.design);structural();let found=false;(await partModelBase(cfg.comps.find(c=>c.name==='mesh'))).traverse(m=>{if(m.material?.map?.image?.width===1)found=true;});return found;},packed));
 console.log('glTF multi-file import, textures and exact portable model bytes passed');
 // Other source formats, phone/desktop viewport bounds, and independent fleet model ownership.
 for(const [name,buffer] of [['cad.stl',stlCube(1000)],['panel.obj',objWall]]){
   await importModel(name,buffer);assert(await page.evaluate(()=>partImport.draft.model.boxes.length>0&&Math.abs(Math.max(...partImport.draft.size)-.2)<1e-6));
   for(const width of [1600,390]){
     await page.setViewportSize({width,height:width===390?844:1000});await page.waitForTimeout(350);
     if(process.env.TEST_SCREENSHOTS)await page.screenshot({path:process.env.TEST_SCREENSHOTS+'-'+name+'-'+width+'.png'});
     const bounds=await page.locator('.part-import-panel').boundingBox();assert(bounds.x>=0&&bounds.x+bounds.width<=width+1&&bounds.y>=0&&bounds.y+bounds.height<=(width===390?844:1000)+1,'Import panel overflow '+JSON.stringify(bounds));
     assert(await page.evaluate(()=>{
       for(let i=0;i<60;i++)fleetScene();renderer.render(scene,camera);
       const view=vpEl.getBoundingClientRect(),panel=partImport.panel.getBoundingClientRect(),bottom=panel.width>view.width*.7;
       const visible=p=>{const q=drone.localToWorld(new THREE.Vector3(...p)).project(camera),x=view.left+(q.x+1)*view.width/2,y=view.top+(1-q.y)*view.height/2;return x>view.left+4&&x<view.right-4&&y>view.top+4&&y<view.bottom-4&&(bottom?y<panel.top-4:x<panel.left-4)&&document.elementFromPoint(x,y)===renderer.domElement;};
       return visible([0,0,0])&&visible(partImport.draft.pos);
     }),'Drone/preview are covered by the overlay '+width);
     for(const projection of ['ortho','persp']){
       const delta=await page.evaluate(projection=>{
         setProjection(projection);fleetScene();renderer.render(scene,camera);const p=cam.target.clone(),before=p.clone().project(camera);
         panCamera(20,10);updateCamera();camera.updateMatrixWorld();const after=p.clone().project(camera);
         panCamera(-20,-10);updateCamera();
         return [(after.x-before.x)*vpEl.clientWidth/2,-(after.y-before.y)*vpEl.clientHeight/2,camera.view?.enabled];
       },projection);
       near(delta[0],20,.1);near(delta[1],10,.1);assert(delta[2],'Import projection offset missing');
     }
   }
   await page.locator('#partImportCancel').click();await page.setViewportSize({width:1600,height:1000});
 }
 assert(await page.evaluate(()=>{const original=fleet.selected,extra=fleetCreate('hex');fleetSelect(extra.id);const clear=!cfg.comps.some(c=>c.model);fleetSelect(original.id);return clear&&cfg.comps.some(c=>c.model)&&fleetShapes(original).boxes.length>0;}));
 assert.deepStrictEqual(errors,[]);console.log('STL/OBJ solidity and size, responsive preview and independent fleet ownership passed; no page errors');
 await cold.close();
}finally{if(browser)await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
