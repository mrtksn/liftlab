'use strict';
// A library asset owns source files and import settings. Placed copies own all physical settings.
const objectLibrary={list:[],ready:false,busy:false,deleted:new Set()};
let objectImportContext='drone';
try {objectLibrary.deleted=new Set(JSON.parse(localStorage.getItem('liftlab-object-library-removed')||'[]'));} catch (_) {}
const objectClone=value=>JSON.parse(JSON.stringify(value));
async function objectLibraryDb(mode,fn) {
  const db=await worldDbOpen();
  return new Promise((ok,no)=>{const tx=db.transaction('library',mode),req=fn(tx.objectStore('library'));tx.oncomplete=()=>ok(req?.result);tx.onerror=()=>no(tx.error);tx.onabort=()=>no(tx.error);});
}
function objectAsset(c) {
  return {id:c.model.fileId,fileId:c.model.fileId,name:c.name,up:c.model.up,units:c.model.units||'custom',scale:c.model.scale,rawSize:c.model.rawSize.slice()};
}
async function objectLibraryKeep(asset,rec) {
  if(rec&&!await worldFilePut(rec))throw new Error('This browser could not store the model files. Keep the original files and export placed copies.');
  await objectLibraryDb('readwrite',s=>s.put(asset));
  objectLibrary.list=[asset,...objectLibrary.list.filter(a=>a.id!==asset.id)];objectLibrary.deleted.delete(asset.id);
  try {localStorage.setItem('liftlab-object-library-removed',JSON.stringify([...objectLibrary.deleted]));}catch(_){}
  return asset;
}
async function objectLibraryAdopt(c) {
  const asset=objectAsset(c);if(objectLibrary.deleted.has(asset.id)||objectLibrary.list.some(a=>a.id===asset.id))return;
  try {await objectLibraryKeep(asset); }catch(_){} // Existing placed copies remain usable without library storage.
}
async function objectLibraryInit() {
  try {const records=await objectLibraryDb('readonly',s=>s.getAll());for(const a of records)if(a.deleted)objectLibrary.deleted.add(a.id);objectLibrary.list=records.filter(a=>!a.deleted);}catch(e){designNote('Object library storage is unavailable: '+e.message);}
  objectLibrary.ready=true;
  // One-time migration retains the old templates as a backup and preserves their physical defaults.
  for(const c of partLibrary){if(objectLibrary.deleted.has(c.model.fileId)||objectLibrary.list.some(a=>a.id===c.model.fileId))continue;
    const asset=objectAsset(c),defaults=objectClone(c);delete defaults.model;delete defaults.id;delete defaults.size;asset.droneDefaults=defaults;asset.droneScale=c.model.scale;asset.droneCollision=c.model.collision||'boxes';asset.droneDetail=c.model.detail;
    try {await objectLibraryKeep(asset);}catch(_){}
  }
  await objectLibraryAdoptPlaced();
}
async function objectLibraryAdoptPlaced() {
  if(!objectLibrary.ready)return;
  for(const d of fleet.drones)for(const c of d.state.cfg.comps)if(c.model)await objectLibraryAdopt(c);
  for(const o of worldObjects.list){if(objectLibrary.deleted.has(o.fileId)||objectLibrary.list.some(a=>a.id===o.fileId))continue;
    try {await objectLibraryKeep({id:o.fileId,fileId:o.fileId,name:o.name,up:o.up,units:o.units,scale:o.scale,rawSize:o.size.slice()});}catch(_){}
  }
}
async function objectLibrarySaveCopy(c) {
  const asset=objectLibrary.list.find(a=>a.id===c.model.fileId)||objectAsset(c),defaults=objectClone(c);delete defaults.id;delete defaults.parent;delete defaults.parentPoint;delete defaults.model;defaults.pos=[0,0,0];
  // Explicitly saved use defaults never replace the shared import scale/orientation.
  delete defaults.size;asset.droneDefaults=defaults;asset.droneScale=c.model.scale;asset.droneCollision=c.model.collision||'boxes';asset.droneDetail=c.model.detail;
  try{await objectLibraryKeep(asset);return true;}catch(e){designNote('Could not save use defaults: '+e.message);return false;}
}
function objectLibraryOpen(context) {
  if(partImport.active||!objectLibrary.ready)return;objectImportContext=context;
  $('#objectLibraryDlg')?.remove();
  const dlg=el('dialog',{id:'objectLibraryDlg',class:'ask parts-dialog','aria-labelledby':'objectLibraryTitle'});
  const status=el('p',{class:'hint',role:'status'}),list=el('div',{class:'object-library-list'});
  const render=()=>list.replaceChildren(...objectLibrary.list.map(asset=>el('div',{class:'object-library-entry'},
    el('div',{},el('strong',{text:asset.name}),el('p',{class:'hint',text:asset.rawSize.map(v=>(v*asset.scale).toFixed(3)).join(' × ')+' m · shared model'})),
    el('div',{class:'hrow'},UI.button({class:'btn primary','data-object-use':asset.id,onclick:()=>{dlg.close();objectUseStart(asset,context);}},context==='world'?'Place in world':'Use on drone'),
      UI.button({class:'btn btn-sm','data-object-edit':asset.id,onclick:()=>{dlg.close();objectAssetEdit(asset,context);}},'Import settings'),
      UI.button({class:'btn icon','aria-label':'Remove library object '+asset.name,onclick:async()=>{try{await objectLibraryDb('readwrite',s=>s.put({id:asset.id,deleted:true}));objectLibrary.list=objectLibrary.list.filter(a=>a!==asset);objectLibrary.deleted.add(asset.id);try{localStorage.setItem('liftlab-object-library-removed',JSON.stringify([...objectLibrary.deleted]));}catch(_){}render();}catch(e){status.textContent='Could not remove entry: '+e.message;}}},'×')))));
  dlg.append(el('div',{class:'dialog-toolbar'},el('h2',{id:'objectLibraryTitle',text:'Object library'}),UI.button({class:'btn',onclick:()=>dlg.close()},'Close')),
    el('p',{class:'hint',text:'Import a model once, then place independent copies in the world or on a drone. Removing a library entry keeps existing copies and their files.'}),
    UI.button({class:'btn primary',id:'objectLibraryImport',onclick:()=>{dlg.close();$('#partModelFile').click();}},'Import 3D object…'),list,status);
  if(!objectLibrary.list.length)status.textContent='Your imported objects will appear here.';
  document.body.append(dlg);render();dlg.showModal();
}
function objectSessionBegin(context,phase) {
  if(partImport.active||maps.busy||!fleetCanSelect()||(context==='drone'&&(!fleet.selected||liveOn()))||(context==='world'&&!wedit.on))return false;
  partImport.active=partImport.busy=true;partImport.context=context;partImport.phase=phase;partImport.editId=null;partImport.asset=null;partImport.mount='';partImport.owner=fleet.selected;
  partImport.wasEditing=editMode;partImport.wasRunning=running;partImport.showSolid=true;partImport.massAuto=true;partImport.pick=null;partImport.generation++;
  partImport.camera={dist:cam.dist,pan:cam.pan.clone(),target:cam.target.clone()};
  if(context==='drone')setEditMode(true);else running=false;
  fleet.pendingEdits++;$('.work').classList.add('part-importing');
  partImport.locked=[$('#airframe'),$('.bar'),$('#editBar'),$('#worldPanel')].filter(Boolean).map(e=>[e,e.inert]);for(const [e]of partImport.locked)e.inert=true;
  partImportPanel();return true;
}
async function objectImportStart(files,context) {
  if(!objectSessionBegin(context,'import'))return;const generation=partImport.generation;
  try {
    const parts=await Promise.all([...files].map(async f=>({name:f.name,data:await f.arrayBuffer()})));
    if(parts.reduce((s,p)=>s+p.data.byteLength,0)>128*1024*1024)throw new Error('Model files exceed 128 MB');
    const main=parts.find(p=>WORLD_OBJ_FORMATS[extOf(p.name)]);if(!main)throw new Error('Choose a GLB, glTF, OBJ or STL model');
    const rec={id:newObjId('asset-'),name:main.name,files:parts},up=extOf(main.name)==='stl'?'z':'y';
    const {base,size}=standModel(await parseModel(rec),up);base.children[0].position.z-=size[2]/2;
    if(!partImport.active||generation!==partImport.generation){disposeGroup(base);return;}
    const units=guessUnits(extOf(main.name),size);
    const c=mkMass(main.name.replace(/\.[^.]+$/,'').slice(0,60)||'Object',0,0,0,{shape:'model',battery:false,category:'payload',rotation:[0,0,0],model:{fileId:rec.id,rawSize:size,scale:WORLD_OBJ_UNITS[units].k,units,up,detail:'medium',collision:'boxes',boxes:[]}});
    worldObjects.files.set(rec.id,rec);partModels.set(rec.id+':'+up,Promise.resolve(base));partImport.draft=c;
    partSolidifyDraft(c,base);c.cog=partSolidMass(c).center;c.mass=clamp(partSolidMass(c).volume*200,.005,20);c.points=[];
    partImport.panel.remove();partImportPanel();partDraftRefresh();objectPreviewFrame(c);
  } catch(e){if(generation===partImport.generation){partImportCancel();designNote('Could not import object: '+e.message);worldEditSay('Could not import object: '+e.message);}}
  finally {if(generation===partImport.generation)partImport.busy=false;}
}
function objectPreviewFrame(c) {
  cam.dist=Math.max(.6,((partImport.context==='drone'?2*cReach:0)+Math.max(...c.size)+.1)*1.2);cam.pan.set(0,0,0);edit.focusLocal=null;edit.focusId=null;
}
function objectScaleField({id,get,set,max=10,limit=1000,numberId,disabled=false}) {
  const field=numField(id,{label:'Scale (longest side)',u:'m',min:.005,max,step:.005,dp:3,hard:true,hmin:.005,hmax:limit,ends:['5 mm',max+' m']},get,set);
  if(numberId)field.node.querySelector('input[type=number]').id=numberId;
  if(disabled)field.node.querySelectorAll('input').forEach(input=>input.disabled=true);
  return field;
}
function objectImportFields(panel,c) {
  const name=UI.input({id:'partImportName',type:'text',value:c.name,maxlength:60});name.addEventListener('input',()=>c.name=name.value.trim()||'Object');
  const units=UI.select({id:'objectImportUnits'},...Object.entries(WORLD_OBJ_UNITS).map(([key,u])=>el('option',{value:key,text:u.label})),el('option',{value:'custom',text:'Custom size'}));units.value=c.model.units||'custom';
  const resize=n=>{partScale(c,n);c.mass=clamp(partSolidMass(c).volume*200,.005,20);partDraftRefresh();objectPreviewFrame(c);};
  const size=objectScaleField({id:'partImportScale',numberId:'partImportSize',get:()=>Math.max(...c.size),set:n=>{c.model.units='custom';units.value='custom';resize(n);}});
  units.addEventListener('change',()=>{if(WORLD_OBJ_UNITS[units.value]){c.model.units=units.value;resize(Math.max(...c.model.rawSize)*WORLD_OBJ_UNITS[units.value].k);size.refresh();}});
  const up=UI.select({id:'objectImportUp'},el('option',{value:'y',text:'Y up'}),el('option',{value:'z',text:'Z up'}));up.value=c.model.up;
  up.addEventListener('change',async()=>{if(partImport.busy)return;const generation=partImport.generation;partImport.busy=true;
    try{c.model.up=up.value;const base=await partModelBase(c);if(generation!==partImport.generation)return;const bounds=new THREE.Box3().setFromObject(base),v=new THREE.Vector3();bounds.getSize(v);c.model.rawSize=v.toArray();partSolidifyDraft(c,base);partDraftRefresh();objectPreviewFrame(c);size.refresh();}catch(e){$('#objectImportSay').textContent=e.message;}finally{if(generation===partImport.generation)partImport.busy=false;}});
  panel.append(el('p',{class:'hint',text:'Set the model name, file units, orientation and default size. Physical properties and collision geometry belong to each copy when you use it.'}),
    UI.field({label:'Name'},name),UI.field({label:'File units'},units),UI.field({label:'Up in file'},up),size.node,el('p',{id:'partImportDims',class:'hint'}),
    el('p',{id:'objectImportSay',role:'status',class:'hint'}),UI.button({class:'btn primary',id:'objectImportSave',onclick:objectImportSave},'Save to object library'));
}
async function objectImportSave() {
  if(partImport.busy||!partImport.draft)return;const c=partImport.draft,generation=partImport.generation,context=partImport.context;
  partImport.busy=true;$('#objectImportSave').disabled=true;
  try {const old=partImport.asset,asset={...old,...objectAsset(c)};await objectLibraryKeep(asset,worldObjects.files.get(c.model.fileId));if(generation!==partImport.generation)return;
    // Library changes do not alter copies already placed in the world or on drones.
    const wasEditing=partImport.wasEditing,wasRunning=partImport.wasRunning;partImportEnd();if(context==='drone'&&!wasEditing)setEditMode(false);if(context==='world')running=wasRunning;
    objectLibraryOpen(context);
  }catch(e){if(generation===partImport.generation){$('#objectImportSay').textContent=e.message;partImport.busy=false;$('#objectImportSave').disabled=false;}}
}
async function objectAssetDraft(asset) {
  const c=mkMass(asset.name,0,0,0,{shape:'model',battery:false,category:'payload',rotation:[0,0,0],model:{fileId:asset.fileId,rawSize:asset.rawSize.slice(),scale:asset.scale,units:asset.units,up:asset.up,detail:'medium',collision:'boxes',boxes:[]}});
  const base=await partModelBase(c);partSolidifyDraft(c,base);c.cog=partSolidMass(c).center;c.mass=clamp(partSolidMass(c).volume*200,.005,20);c.points=[{id:'top',name:'Top mount',pos:[0,0,c.size[2]/2]}];c.selfPoint='top';return c;
}
async function objectAssetEdit(asset,context) {
  if(!objectSessionBegin(context,'import'))return;const generation=partImport.generation;partImport.asset=asset;
  try{const c=await objectAssetDraft(asset);if(generation!==partImport.generation)return;partImport.draft=c;partImport.panel.remove();partImportPanel();partDraftRefresh();objectPreviewFrame(c);}
  catch(e){if(generation===partImport.generation){partImportCancel();designNote(e.message);}}finally{if(generation===partImport.generation)partImport.busy=false;}
}
async function objectUseStart(asset,context,existing=null) {
  if(context==='world')return objectWorldUse(asset);
  if(!objectSessionBegin(context,'use'))return;const generation=partImport.generation;
  try{const c=existing?objectClone(existing):await objectAssetDraft(asset);if(generation!==partImport.generation)return;
    if(existing){partImport.editId=existing.id;partImport.mount=(existing.parent??'')+(existing.parentPoint?'|'+existing.parentPoint:'');c.id=uid++;partImport.massAuto=false;if(!c.model.triangles)partSolidifyDraft(c,await partModelBase(c));}else if(asset.droneDefaults){if(asset.droneScale)partScale(c,Math.max(...c.model.rawSize)*asset.droneScale);Object.assign(c,objectClone(asset.droneDefaults));delete c.parent;delete c.parentPoint;c.id=uid++;c.pos=[0,0,0];c.model.collision=asset.droneCollision||'boxes';if(asset.droneDetail&&asset.droneDetail!==c.model.detail){c.model.detail=asset.droneDetail;partSolidifyDraft(c,await partModelBase(c));}partImport.massAuto=false;}
    if(generation!==partImport.generation)return;
    partImport.draft=c;partImport.panel.remove();partImportPanel();const mount=$('#partImportMount');if(existing)mount.value=(existing.parent??'')+(existing.parentPoint?'|'+existing.parentPoint:'');partDraftRefresh();objectPreviewFrame(c);
  }catch(e){if(generation===partImport.generation){partImportCancel();designNote('Could not use object: '+e.message);}}finally{if(generation===partImport.generation)partImport.busy=false;}
}
async function objectWorldUse(asset) {
  if(!wedit.on||wedit.busy||maps.busy||partImport.active)return;wedit.busy=true;fleet.pendingEdits++;worldEditSay('Preparing object copy…');worldEditRender();mapUiSync();
  try{const rec=worldObjects.files.get(asset.fileId)||await worldFileGet(asset.fileId);if(!rec)throw new Error('Original model file unavailable');
    const {base,size}=standModel(await parseModel(rec),asset.up);if(!wedit.on){disposeGroup(base);throw new Error('World editing ended before placement');}const o={id:newObjId('o'),fileId:asset.fileId,name:asset.name,pos:[cam.target.x,cam.target.y,0],yaw:0,scale:asset.scale,units:asset.units,up:asset.up,detail:'medium',collision:'boxes',size,boxes:[]};
    objGroup(o);showModel(o,base);try{worldObjSolidify(o);}catch(e){disposeObj(o);throw e;}worldObjects.list.push(o);worldObjectsChanged();worldEditChanged();wedit.busy=false;worldEditSay('Placed an independent fixed copy. Edit its placement, size and collision geometry below.');worldEditSelect(o.id);worldEditLookAt(o);
  }catch(e){worldEditSay('Could not place object: '+e.message);}finally{wedit.busy=false;fleet.pendingEdits--;worldEditRender();mapUiSync();}
}
function objectRoleFields(c,changed) {
  const role=UI.select({id:'objectRole'},...Object.entries({payload:'Payload',battery:'Battery',wing:'Wing'}).map(([value,text])=>el('option',{value,text})));role.value=c.battery?'battery':isWing(c)?'wing':'payload';
  role.addEventListener('change',()=>{c.battery=role.value==='battery';c.batteryAutoMass=false;c.aero=role.value==='wing'?'wing':'prism';if(c.battery)c.category='power';else if(isWing(c))c.category='structure';changed();});
  return UI.field({label:'Use as'},role);
}
$('#objectLibraryOpen').addEventListener('click',()=>{$('#addPartDlg').close();objectLibraryOpen('drone');});
$('#partModelImport').addEventListener('click',()=>{objectImportContext='drone';});
objectLibraryInit();
