'use strict';
// One shared world/clock/renderer. Each drone keeps its own mutable runtime and
// WASM instances; legacy feature modules operate on the synchronously bound drone.
const FLEET_LS = 'liftlab-world-v1';
const fleet = { drones: [], selected: null, active: null, visual: null, nextId: 1, time: 0, steps: 0, ready: false, restoring: false, pendingEdits:0 };   // steps: the shared clock in physics steps, since the world last started
function droneCopy(x, seen = new Map()) {
  if (!x || typeof x !== 'object') return x;
  if (seen.has(x)) return seen.get(x);
  if (x instanceof WeakMap) return new WeakMap();
  if (x instanceof WeakSet) return new WeakSet();
  if (ArrayBuffer.isView(x)) return new x.constructor(x);
  if (x instanceof WebAssembly.Module) return x;
  const y = x instanceof Map ? new Map() : x instanceof Set ? new Set() : Array.isArray(x) ? [] : Object.create(Object.getPrototypeOf(x));
  seen.set(x,y);
  if (x instanceof Map) for (const [k,v] of x) y.set(droneCopy(k,seen),droneCopy(v,seen));
  else if (x instanceof Set) for (const v of x) y.add(droneCopy(v,seen));
  else for (const [k,v] of Object.entries(x)) y[k] = droneCopy(v,seen);
  return y;
}
const droneSeed = captureDroneState();
// Template references are captured before boot mutates the first design.
const droneTemplate = droneCopy({...droneSeed, LAWS:null, RN:null});
const droneLawDefaults = Object.fromEntries(Object.entries(LAWS).map(([k,L]) => [k,{...L}]));
const droneRunnerDefaults = {...RN, pool:[], act:null, prev:null, stage:null, samples:{}, sampleN:{}, log:[]};
function droneUiActive() { return !fleet.ready || fleet.active === fleet.selected; }
function droneSwitch(d) {
  if (d === fleet.active) return;
  if (fleet.active) captureDroneState(fleet.active.state);
  installDroneState(d.state); fleet.active = d;
}
function withDrone(d, fn) {
  const previous = fleet.active; droneSwitch(d);
  try { const out = fn(); if (out && typeof out.then === 'function') throw new Error('Drone scope must be synchronous'); return out; }
  finally { if (previous) droneSwitch(previous); }
}
function droneGraphicsSwitch(d) {
  if (fleet.visual === d) return;
  if (fleet.visual) captureDroneGraphics(fleet.visual.graphics);
  installDroneGraphics(d.graphics); fleet.visual = d;
}
// Async loaders/timers carry the owning runtime, never whichever drone is selected later.
window.runDroneCallback = (owner, fn) => {
  const d = fleet.drones.find(d => d.state.brt === owner || d.state.RN === owner);
  if (d) return withDrone(d,fn);
  if (!fleet.ready) return fn();
};
function fleetNotice(text) { designNote(text); }
function fleetCanSelect(ownedEdit = false) {
  if (agent.busy) { fleetNotice('Stop the current AI task before changing drones.'); return false; }
  if (fleet.pendingEdits && !(ownedEdit && fleet.pendingEdits === 1)) { fleetNotice('Finish the current design operation before changing drones.'); return false; }
  if (live.state !== 'off' || INST.busy || INST.conn) { fleetNotice('Disconnect the real drone before changing simulated drones.'); return false; }
  if (document.querySelector('dialog[open]') || edit.drag) return false;
  return true;
}
function fleetRefresh() {
  INST.target = null;
  cardRefresh.clear(); holderSnap.clear?.(); lawCards.clear(); progCards.clear(); appCards.clear(); APPS_UI.current = null; COMP.built = false; COMP.exErr = null;
  HW_UI.open ||= new Set();
  buildComputers(); renderComps(); buildActRows(); frameMassField.refresh(); renderFrameShape();
  spRefs.length=0;buildSp();buildFlightEnvironmentFields();
  throwFieldRefs.length=0;buildThrowFields();allocFieldRefs.length=0;$('#mixSlot').replaceChildren();buildAllocFields();buildTuning();
  renderBattery(); syncFlightUi(); syncSp(); syncAllocFields(); setPilotLevel(pilot.level);
  $('#keepLearn').checked=learnPrefs.keep;$('#holdPulses').checked=learnPrefs.holdPulses;$('#thenCal').checked=throwCfg.thenCalibrate;
  setLaunch(launchMode,false); for (const f of throwFieldRefs) f(); for (const f of allocFieldRefs) f();
  matchT = 0; matchCache = []; launchUi.key = ''; GS_UI.built = false; GS_UI.next = 0; GS_UI.logKey = ''; GS_UI.logN = -1; GS_UI.items.clear(); GS_UI.cargoN = -1;
  $('#designName').value = designs.name; renderUndo(); renderDesigns(); refreshEnvelope(); renderMass(); refreshFormulaStatus();
  for (const [k,src] of fleet.selected.formulaDrafts) { const card = lawCards.get(k); if (card) card.ta.value = src; }
  agent.rec = fleet.selected.agentMemory.rec; agent.recNext = fleet.selected.agentMemory.recNext; agent.recLast = fleet.selected.agentMemory.recLast;
  agent.cur = fleet.selected.chat; aiUi.draft = null; agentRender(); updateLive(); drawChart(); renderHealth(); cargoBarSync(); cargoSecSync();
  $('#crash').hidden = !S.crashed; if (S.crashed) $('#crashWhy').textContent = S.crashed;
  Object.assign(edit,{sel:null,hover:null,drag:null,down:null,focusId:null,refocus:false}); updateEditMsg();
  // Switch the audible scope without replaying alerts or restarting matching voices.
  sndSelectScope();
  fleetRenderSelector();
}
function fleetSelect(id) {
  const d = id ? fleet.drones.find(d => d.id === id) : null;
  if (id && !d) return false;
  if (d === fleet.selected) return true;
  if (!fleetCanSelect()) { fleetRenderSelector(); return false; }
  if (d && worldEditOn()) worldEditSet(false);   // (picking a drone ends editing the world's objects)
  fleetRememberUi(); fleetReleaseControls();
  if (!d) {
    if (editMode) setEditMode(false);
    fleet.selected = null; fleetClearHover(); sndSelectScope();
    fleetRenderSelector(); updateLive(); fleetSave(); return true;
  }
  const library = {list:designs.list,col:designs.col,where:designs.where};
  droneSwitch(d); droneGraphicsSwitch(d); fleet.selected = d; cam.pan.set(0,0,0); Object.assign(designs,library);
  fleetRefresh(); fleetSave(); return true;
}
function fleetRememberUi() {
  const d = fleet.selected; if (!d) return;
  d.formulaDrafts.clear(); for (const [k,c] of lawCards) if (c.ta.value !== LAWS[k].src) d.formulaDrafts.set(k,c.ta.value);
  d.chat = agent.cur; Object.assign(d.agentMemory,{rec:agent.rec,recNext:agent.recNext,recLast:agent.recLast});
}
function fleetReleaseControls() { cancelResetHold(); releaseAll(); if (poke.src) pokeEnd(poke.src,false); }
function fleetName(d) { return d.name || d.state.designs.name || 'Drone'; }
function fleetRenderSelector() {
  const select = $('#droneSelect'); if (!select || !fleet.ready) return;
  const signature = fleet.drones.map(d => d.id+':'+fleetName(d)).join('|');
  if (select.dataset.signature !== signature) { select.replaceChildren(el('option',{value:'',text:'World — no selection'}),...fleet.drones.map(d => el('option',{value:d.id,text:fleetName(d)}))); select.dataset.signature = signature; }
  select.value = fleet.selected?.id || ''; $('#droneRemove').disabled = !fleet.selected || fleet.drones.length < 2;
  if (fleet.selected) $('#droneName').value = fleetName(fleet.selected);
  fleetSelectionUi();
}
function fleetSpawn() {
  const origin = fleet.selected ? fleet.selected.state.setpoint : {x:0,y:0};
  for (let n = 1; n < 400; n++) {
    const ring = Math.ceil(Math.sqrt(n)), angle = n*2.399963, x = origin.x+ring*1.5*Math.cos(angle), y = origin.y+ring*1.5*Math.sin(angle);
    if (terrainNear([x,y,.6],.8).length) continue;
    if (fleet.drones.some(d => Math.hypot(d.state.S.p[0]-x,d.state.S.p[1]-y) < d.state.cReach+1)) continue;
    return {x,y,z:1.5,yaw:0};
  }
  return {x:origin.x+fleet.drones.length*2,y:origin.y,z:1.5,yaw:0};
}
function fleetCreate(key = 'quadx', saved = null, ownedEdit = false) {
  if (!fleet.restoring && !fleetCanSelect(ownedEdit)) return null;
  fleetRememberUi(); fleetReleaseControls();
  const library = {list:designs.list,col:designs.col,where:designs.where}, module = brt.module, runnerModule = RN.module;
  const state = droneCopy(droneTemplate);
  state.LAWS = Object.fromEntries(Object.entries(droneLawDefaults).map(([k,L]) => [k,{...L}]));
  state.RN = {...droneRunnerDefaults,module:runnerModule,loading:null,poolPending:0,pool:[],P:null,A:null,act:null,prev:null,stage:null,log:[],samples:{},sampleN:{},trapped:{},steps:{},calls:{},maxSteps:{}};
  state.brt.module = module; Object.assign(state.designs,library);
  state.agentTriggers = saved?.triggers || []; state.agentChat = saved?.chat || null;
  state.setpoint = saved?.setpoint ? {...saved.setpoint} : fleetSpawn();
  const number = fleet.nextId++, d = {id:saved?.id || 'drone-'+number,name:saved?.name || (PRESETS[key]?.label.replace(/\s*\(.*\)\s*$/,'') || 'Drone')+' '+number,state,graphics:null,formulaDrafts:new Map(),agentMemory:{rec:[],recNext:0,recLast:0},chat:null};
  fleet.drones.push(d); droneSwitch(d); fleet.selected = d;
  d.agentMemory = state.agentMemory; d.chat = state.agentChat;
  const g = new THREE.Group(), fx = new THREE.Group(); scene.add(g,fx); g.userData.droneId = d.id;
  d.graphics = {drone:g,worldFx:fx,headArrow:null,mats:{},rangeVis:new Map(),jointGroups:new Map(),parts:new Map(),pickGroups:new Map(),pendVis:new Map(),ghost:null,cogDot:null,modelRing:null,gravArrow:null,windArrow:null,spMarker:null,trailLine:null,tqNetGlyph:null,tqWantArrow:null,tqRotor:[],droneShown:'',aeroVis:{arrows:[],sm:[]},looseVis:new Map(),reachVis:[]};
  droneGraphicsSwitch(d); buildMaterials(); buildWorldFx();
  if (saved) {
    applyDesign({...saved.design,environment:null}); Object.assign(radioCfg,saved.radio || {}); Object.assign(radioCfg2,saved.radio2 || {});
    Object.assign(allocPrefs,saved.allocPrefs || {}); Object.assign(throwCfg,saved.throwCfg || {}); Object.assign(learnPrefs,saved.learnPrefs || {});
    launchMode = saved.launch === 'throw' ? 'throw' : 'hover'; steerMix.share = saved.mixShare ?? .5;
    designLoaded(saved.designId || null,saved.designName || ''); designs.preset = PRESETS[saved.preset] ? saved.preset : null;
  } else {
    const p = PRESETS[key].build(); cfg.frame.mass = p.frame; setFrameShape(p.frameShape); cfg.comps = migrateComps(p.comps); cfg.battery = p.battery || defaultBattery(); cfg.tuning = tuneDefaults(); cfg.programs = []; cfg.apps = [];
    const C = PRESETS[key].computers ? PRESETS[key].computers() : defaultComputers(); if (PRESETS[key].cargoTask) C.boards.find(b=>b.tasks.includes('core')).tasks.push('cargo');
    cfg.computers = fixComputers(C); mode = p.mode; designLoaded(null,''); designs.preset = key;
  }
  rnLoadWasm(); rnRebuild(); afterLoad(); undo.stack = [designSnap()]; undo.i = 0;
  captureDroneState(d.state); captureDroneGraphics(d.graphics); fleetRefresh();
  if (!saved && PRESETS[key].blank && !fleet.restoring) setEditMode(true);
  fleetSave(); return d;
}
function fleetRemove(id = fleet.selected?.id) {
  if (fleet.drones.length < 2 || !fleetCanSelect()) return false;
  const d = fleet.drones.find(d=>d.id===id); if (!d) return false;
  if (d === fleet.selected) fleetSelect(fleet.drones.find(x=>x!==d).id);
  if (fleet.active === d) droneSwitch(fleet.drones.find(x=>x!==d));
  if (fleet.visual === d) droneGraphicsSwitch(fleet.drones.find(x=>x!==d));
  clearTimeout(d.state.RN.pending);
  for (const group of [d.graphics.drone,d.graphics.worldFx]) { scene.remove(group); disposeGroup(group); }
  for (const material of Object.values(d.graphics.mats)) material.dispose?.();
  fleet.drones = fleet.drones.filter(x=>x!==d); fleetRenderSelector(); fleetSave(); return true;
}
function fleetResetAll() {
  if (!fleet.ready || !fleetCanSelect()) return false;
  fleetReleaseControls();
  peerAirReset(); fleet.time=0; fleet.steps=0;worldMotionReset();
  for (const d of fleet.drones) withDrone(d,()=>{releaseAll();pilot.vref=[0,0,0];resetSim();d.contacts=0;});
  sndSelectScope(); GS_UI.next=0; launchUi.key='';
  $('#crash').hidden=true;$('#liftoff').hidden=true;updateLive();
  if (fleet.selected) {drawChart();renderHealth();cargoBarSync();cargoSecSync();}
  fleetSave(); return true;
}
$('#fleetReset').addEventListener('click',()=>userWorldReset());
// Every step is the same whatever the frame rate: the pilot's target moves on a fixed tick, and a replay's inputs
// land on the steps they were made on (replay.js). That's what makes a run repeatable.
function fleetStep(steps) {
  if (!fleet.ready) { for(let n=0;n<steps;n++){ if(S.steps%PILOT_TICK===0)pilotStep(PILOT_TICK*PDT); physStep(); } return; }
  // Equal steps for every craft; if CPU-limited the shared clock slows together.
  for (let n=0;n<steps;n++) {
    if (replayStep()) break;
    if (fleet.steps%PILOT_TICK===0) for (const d of fleet.drones) withDrone(d,()=>pilotStep(PILOT_TICK*PDT));
    worldMotionStep(PDT);
    const payloadContacts=fleet.drones.some(fleetHasPayloads);
    for (const d of fleet.drones) withDrone(d,()=>physStep(payloadContacts));
    fleetCollisions(); fleet.time += PDT; fleet.steps++;
    if(payloadContacts)for(const d of fleet.drones)withDrone(d,finishPhysStep);
  }
}
function fleetScene() {
  const selected = fleet.selected, restoreVisual = fleet.visual, wasEditing = editMode;
  try {
    for (const d of fleet.drones) {
      droneGraphicsSwitch(d);
      withDrone(d,()=>{editMode = wasEditing && d===selected; updateScene(d===selected);});
    }
  } finally { editMode = wasEditing; droneGraphicsSwitch(selected || restoreVisual); }
  if (!selected) { updateCity(cam.target.toArray()); shadowMesh.visible=false; updateCamera(); }
  fleetHoverScene();
}
function fleetTheme() {
  const restoreVisual = fleet.visual;
  try {
    for (const d of fleet.drones) {
      droneGraphicsSwitch(d);
      withDrone(d,()=>{buildMaterials();buildWorldFx();rebuildDrone();});
    }
  } finally { droneGraphicsSwitch(fleet.selected || restoreVisual); }
  renderer.setClearColor(colorOf('--viewport'),1);buildGizmo();buildCity();
}
function fleetSnapshot() {
  if (fleet.active) captureDroneState(fleet.active.state);
  return {v:1,selected:fleet.selected?.id || null,nextId:fleet.nextId,mapId:maps.current,terrain:{kind:terrain.kind,seed:terrain.seed},objects:worldObjectsSnapshot(),seeds:{...worldSeeds},environment:{...envr},drones:fleet.drones.map(d=>{
    const s=d.state;
    return {id:d.id,name:d.name,setpoint:{...s.setpoint},design:{frame:s.cfg.frame.mass,frameShape:{...s.cfg.frame},comps:s.cfg.comps,mode:s.mode,battery:s.cfg.battery,computers:s.cfg.computers,tuning:s.cfg.tuning,programs:s.cfg.programs||[],apps:s.cfg.apps||[],laws:Object.fromEntries(Object.entries(s.LAWS).filter(([,L])=>L.src!==L.defSrc).map(([k,L])=>[k,L.src]))},radio:{...s.radioCfg},radio2:{...s.radioCfg2},allocPrefs:{...s.allocPrefs},throwCfg:{...s.throwCfg},learnPrefs:{...s.learnPrefs},launch:s.launchMode,mixShare:s.steerMix.share,designId:s.designs.cur,designName:s.designs.name,preset:s.designs.preset,triggers:s.agentTriggers.map(({fired,last,was,...t})=>t),chat:s.agentChat};
  })};
}
function fleetSave() {
  if (!fleet.ready || fleet.restoring) return;
  try { localStorage.setItem(FLEET_LS,JSON.stringify(fleetSnapshot())); } catch (_) { fleetNotice('World could not be saved in this browser. Save important designs to files.'); }
}
function fleetInit() {
  const d = {id:'drone-1',name:designs.name || PRESETS[designs.preset]?.label.replace(/\s*\(.*\)\s*$/,'') || 'Drone 1',state:captureDroneState(),graphics:captureDroneGraphics(),formulaDrafts:new Map(),agentMemory:{rec:agent.rec,recNext:agent.recNext,recLast:agent.recLast},chat:agent.cur};
  drone.userData.droneId = d.id; fleet.drones=[d]; fleet.selected=fleet.active=fleet.visual=d; fleet.nextId=2; fleet.ready=true;
  d.agentMemory=d.state.agentMemory;
  for (const t of agent.threads) if (!t.droneId) t.droneId=d.id;
  let saved; try { saved=JSON.parse(localStorage.getItem(FLEET_LS)); } catch (_) {}
  if (saved?.v===1 && Array.isArray(saved.drones) && saved.drones.length && new Set(saved.drones.map(x=>x?.id)).size===saved.drones.length && saved.drones.every(x=>x.id && x.design && Array.isArray(x.design.comps) && x.setpoint && ['x','y','z','yaw'].every(k=>Number.isFinite(x.setpoint[k])))) {
    fleet.restoring=true;
    try {
      d.id='boot-placeholder'; d.graphics.drone.userData.droneId=d.id;
      if (TERRAINS[saved.terrain?.kind]) setTerrain(saved.terrain.kind,saved.terrain.seed);
      maps.current = typeof saved.mapId === 'string' ? saved.mapId : null;
      worldObjectsRestore(saved.objects);
      setWorldSeeds(saved.seeds || {});
      for (const rec of saved.drones) fleetCreate('quadx',rec);
      fleetRemove(d.id); fleet.nextId=Math.max(fleet.nextId,saved.nextId || 1); Object.assign(envr,saved.environment || {});
      fleetSelect(saved.selected === null ? null : saved.selected || fleet.drones[0].id);
    } catch(e) { fleetNotice('Could not restore the whole world: '+e.message); }
    finally { fleet.restoring=false; }
  }
  fleetRenderSelector(); fleetSave();
}
$('#droneSelect').addEventListener('change',e=>fleetSelect(e.target.value));
window.addEventListener('pagehide',fleetSave);
$('#droneName').addEventListener('change',e=>{if (!fleet.selected) return; fleet.selected.name=e.target.value.trim().slice(0,80) || 'Drone';fleetRenderSelector();fleetSave();});
$('#droneRemove').addEventListener('click',()=>{if(!fleet.selected)return;if(confirm('Remove '+fleetName(fleet.selected)+' from this simulation? Saved designs are kept.'))fleetRemove();});
// Keep delayed file/account edits on their originating drone. Selection remains
// available during read-only loading, but not halfway through a design mutation.
function droneEditTask(fn) { return async function(...args) { fleet.pendingEdits++; try {return await fn(...args);}finally{fleet.pendingEdits--;} }; }
saveDesign=droneEditTask(saveDesign);importDesign=droneEditTask(importDesign);openDesignCode=droneEditTask(openDesignCode);
// Selection is a UI concern; the retained active/visual context is internal only.
function fleetSelectionUi() {
  const world = !fleet.selected;
  $('.app').classList.toggle('world-view',world); $('.work').classList.toggle('world-view',world);
  $('#airframe').hidden=world; $('#telemetry').hidden=world; $('#worldPanel').hidden=!world;
  for (const id of ['tEdit','tFollow','tChase','resetBtn','designShare','liveBtn','launchHover','launchThrow']) $('#'+id).disabled=world;
  if (world) for (const id of ['crash','liftoff']) $('#'+id).hidden=true;
}
let worldCollapsed=false;
try { worldCollapsed=localStorage.getItem('liftlab-world-collapsed')==='true'; } catch (_) {}
function worldPanelRender() {
  $('#worldSettingsBody').hidden=worldCollapsed;
  $('#worldSettingsToggle').setAttribute('aria-expanded',String(!worldCollapsed));
  $('#worldSettingsToggle span').textContent=worldCollapsed ? '▸' : '▾';
}
worldPanelRender();
$('#worldSettingsToggle').addEventListener('click',()=>{
  worldCollapsed=!worldCollapsed;worldPanelRender();
  try {localStorage.setItem('liftlab-world-collapsed',String(worldCollapsed));} catch (_) {}
});
$('#worldView').addEventListener('click',()=>{if(fleetSelect(null)){worldCollapsed=false;worldPanelRender();}});
new ResizeObserver(()=>$('.app').style.setProperty('--world-header-height',$('#topBar').getBoundingClientRect().height+'px')).observe($('#topBar'));

// A short click selects a frontmost craft or clears selection on empty ground.
// Track the whole gesture so a drag returning to its origin or a pinch is never a click.
let fleetPointer=null, fleetHoverPoint=null, fleetHoverNext=0, fleetHovered=null;
const fleetHoverBox=new THREE.Box3(), fleetHoverOutline=new THREE.Box3Helper(fleetHoverBox);
fleetHoverOutline.userData.noPick=true;fleetHoverOutline.visible=false;
fleetHoverOutline.material.depthTest=false;fleetHoverOutline.material.transparent=true;
fleetHoverOutline.material.opacity=.75;fleetHoverOutline.renderOrder=30;scene.add(fleetHoverOutline);
function fleetHit(e) {
  camera.updateMatrixWorld(); rayFrom(e);
  const hit=raycaster.intersectObjects(fleet.drones.map(d=>d.graphics.drone),true).find(h=>solidVisible(h.object));
  let g=hit?.object;while(g && !g.userData.droneId)g=g.parent;
  return g ? fleet.drones.find(d=>d.id===g.userData.droneId) : null;
}
function fleetClearHover() {
  fleetHovered=null;fleetHoverOutline.visible=false;$('#droneHover').hidden=true;
  vpEl.classList.remove('selectable');
}
function fleetHoverScene() {
  if (!fleetHoverPoint || fleetPointer || ptrs.size || edit.drag || worldEditOn()) {fleetClearHover();return;}
  if (performance.now()>=fleetHoverNext) {
    fleetHoverNext=performance.now()+50;
    fleetHovered=fleetHit(fleetHoverPoint);
    if(editMode && fleetHovered===fleet.selected)fleetHovered=null;
  }
  if(!fleetHovered){fleetClearHover();return;}
  boxOf(fleetHovered.graphics.drone,fleetHoverBox);fleetHoverBox.expandByScalar(.025);
  fleetHoverOutline.material.color.set(colorOf('--accent'));fleetHoverOutline.visible=true;
  const tip=$('#droneHover'),r=vpEl.getBoundingClientRect();
  tip.textContent=fleetName(fleetHovered)+(fleetHovered===fleet.selected ? ' · selected' : ' · click to select');tip.hidden=false;
  tip.style.left=Math.max(8,Math.min(r.width-tip.offsetWidth-8,fleetHoverPoint.clientX-r.left+14))+'px';
  tip.style.top=Math.max(8,Math.min(r.height-tip.offsetHeight-8,fleetHoverPoint.clientY-r.top+14))+'px';
  vpEl.classList.add('selectable');
}
vpEl.addEventListener('pointerdown',e=>{
  if (e.button!==0 || e.ctrlKey || e.metaKey) {fleetPointer=null;return;}
  if(fleetPointer){fleetPointer.dragged=true;return;}
  fleetPointer={id:e.pointerId,x:e.clientX,y:e.clientY,dragged:false};fleetClearHover();
},true);
vpEl.addEventListener('pointermove',e=>{
  if(fleetPointer && (e.ctrlKey || e.metaKey || Math.hypot(e.clientX-fleetPointer.x,e.clientY-fleetPointer.y)>=5))fleetPointer.dragged=true;
  fleetHoverPoint=e.pointerType==='touch' ? null : {clientX:e.clientX,clientY:e.clientY};
},true);
vpEl.addEventListener('pointerleave',()=>{fleetHoverPoint=null;fleetClearHover();});
vpEl.addEventListener('pointercancel',()=>{fleetPointer=null;fleetClearHover();},true);
vpEl.addEventListener('lostpointercapture',e=>{if(fleetPointer?.id===e.pointerId)fleetPointer=null;fleetClearHover();},true);
vpEl.addEventListener('pointerup',e=>{
  const p=fleetPointer; fleetPointer=null;
  if(!p || p.id!==e.pointerId || p.dragged || Math.hypot(e.clientX-p.x,e.clientY-p.y)>=5 || edit.drag || ptrs.size>1)return;
  // Editor handles have precedence over world picking.
  if((editMode && pickHandle(e)) || worldEditOn())return;   // (editing the world, a click picks an object)
  const hit=fleetHit(e);
  if(hit!==fleet.selected){
    const changed=fleetSelect(hit?.id || null);
    if(changed){e.stopImmediatePropagation();ptrs.delete(e.pointerId);edit.down=null;}
  }
},true);
