'use strict';
// Prescribed motion belongs only to world copies. Source geometry is never regenerated during playback.
const WORLD_MOTION_DEFAULT = {type:'hinge',axis:2,pivot:[0,0,0],offset:[0,0,1],angle:90,duration:1.5,ease:'smooth',mode:'button',waitOpen:2,waitClosed:2,zone:[0,0,0],radius:2,delay:1,blocked:'stop'};
const worldMotionEditor={id:null,playing:false,pick:false};
function worldMotionValidate(a) {
  if(a==null)return null;
  const vec=v=>Array.isArray(v)&&v.length===3&&v.every(x=>Number.isFinite(x)&&Math.abs(x)<=100);
  if(!a||!['hinge','slide'].includes(a.type)||![0,1,2].includes(a.axis)||!vec(a.pivot)||!vec(a.offset)||!vec(a.zone)||!Number.isFinite(a.angle)||Math.abs(a.angle)>360||!Number.isFinite(a.duration)||a.duration<.1||a.duration>120||!['linear','smooth'].includes(a.ease)||!['loop','button','proximity'].includes(a.mode)||!['stop','reverse','push'].includes(a.blocked)||!['waitOpen','waitClosed','delay'].every(k=>Number.isFinite(a[k])&&a[k]>=0&&a[k]<=120)||!Number.isFinite(a.radius)||a.radius<.05||a.radius>100)throw new Error('Invalid world animation');
  return Object.fromEntries(Object.keys(WORLD_MOTION_DEFAULT).map(k=>[k,Array.isArray(a[k])?a[k].slice():a[k]]));
}
function worldMotionState(o){return o.motionState||(o.motionState={t:0,target:0,wait:0,empty:0,blocked:false});}
function worldMotionPose(o,t,oldT=t,dt=0){
  const a=o.animation,Y=eulerR(0,0,o.yaw),f=x=>a.ease==='smooth'?x*x*(3-2*x):x,u=f(clamp(t,0,1)),prev=f(clamp(oldT,0,1));
  const axis=m3v(Y,[0,1,2].map(k=>k===a.axis?1:0)),pivot=m3v(Y,a.pivot),off=a.type==='slide'?m3v(Y,scl(a.offset,u)):[0,0,0],angle=a.type==='hinge'?a.angle*D2R*u:0;
  const q=[Math.cos(angle/2),...scl(axis,Math.sin(angle/2))],R=qmat(q),RT=m3T(R),origin=add(add(o.pos,off),sub(pivot,m3v(R,pivot)));
  return {R,RT,q,origin,pivot:add(add(o.pos,pivot),off),v:dt&&a.type==='slide'?m3v(Y,scl(a.offset,(u-prev)/dt)):[0,0,0],w:dt&&a.type==='hinge'?scl(axis,a.angle*D2R*(u-prev)/dt):[0,0,0]};
}
function worldMotionBounds(body,pose){
  const lo=[Infinity,Infinity,Infinity],hi=[-Infinity,-Infinity,-Infinity],b=body.mesh?body.mesh:body;
  for(let i=0;i<8;i++){const p=add(pose.origin,m3v(pose.R,[0,1,2].map(k=>i&(1<<k)?b.hi[k]:b.lo[k])));for(let k=0;k<3;k++){lo[k]=Math.min(lo[k],p[k]);hi[k]=Math.max(hi[k],p[k]);}}
  return {lo,hi};
}
function worldMotionShapes(o,pose=o.motionPose||worldMotionPose(o,0),prev=o.motionPrevious||pose){
  const bodies=o.collision==='mesh'&&o.triangles?.length?[{mesh:collisionMesh(o.triangles),origin:[0,0,0]}]:o.boxes;
  return bodies.map(body=>({...worldMotionBounds(body,pose),body,pose,previousPose:prev,obj:o.id,what:o.name}));
}
function worldMotionDraw(o){
  if(!o.animation||!o.g)return;const p=o.motionPose||worldMotionPose(o,0),q=qmul(p.q,[Math.cos(o.yaw*D2R/2),0,0,Math.sin(o.yaw*D2R/2)]);
  o.g.position.set(...p.origin);o.g.quaternion.set(q[1],q[2],q[3],q[0]);o.g.updateMatrixWorld(true);
  if(typeof solidVis!=='undefined'&&solidVis&&wedit.sel===o.id){solidVis.position.set(...p.origin);solidVis.quaternion.set(p.q[1],p.q[2],p.q[3],p.q[0]);}
}
function worldMotionApply(o,t,dt=0){const st=worldMotionState(o),prev=o.motionPose||worldMotionPose(o,st.t);o.motionPrevious=prev;o.motionPose=worldMotionPose(o,t,st.t,dt);st.t=t;worldMotionDraw(o);}
function worldMotionReset(){
  for(const o of worldObjects.list){delete o.motionState;delete o.motionPose;delete o.motionPrevious;if(o.animation)worldMotionApply(o,0);else placeGroup(o);}worldObjectsChanged(false);worldMotionControls();
}
// The same articulated contact points used by the drone solver; proximity zones instead use each hub.
function worldMotionObstacles(){
  const out=[];for(const d of fleet.drones)withDrone(d,()=>{
    const R=qmat(S.q),K=mbKinematics(cat6(S.w,m3v(m3T(R),S.v)));
    for(const pt of cPts){if(pt.b>=MB.bodies.length)continue;const P=add(K.ob[pt.b],m3v(K.Rb[pt.b],sub(pt.rest,MB.bodies[pt.b].pivot)));out.push({p:add(S.p,m3v(R,P)),r:pt.r});}
    for(const c of liveComps())if(c.type==='hang'){const s=pend.get(c.id);if(s)out.push({p:s.p,r:payloadR(c)});}
    for(const L of cargo.loose)for(const pt of L.pts)out.push({p:add(L.p,m3v(looseR(L),sub(pt.r,L.cm))),r:pt.rad});
  });return out;
}
const worldMotionSamples=new WeakMap();
function worldMotionRestPoints(o){
  const key=o.collision==='mesh'?o.triangles:o.boxes;let points=worldMotionSamples.get(key);if(points)return points;
  if(o.collision==='mesh')points=partMeshContacts({size:o.size.map(x=>x*o.scale),pos:[0,0,0],rotation:[0,0,0],model:{triangles:o.triangles}}).map(p=>p.rest);
  else{const seen=new Set();points=[];for(const b of o.boxes){const put=p=>{const k=p.map(x=>Math.round(x/.005)).join(',');if(!seen.has(k)){seen.add(k);points.push(p);}};for(let i=0;i<8;i++)put([0,1,2].map(k=>i&(1<<k)?b.hi[k]:b.lo[k]));put(b.lo.map((x,k)=>(x+b.hi[k])/2));}}
  worldMotionSamples.set(key,points);return points;
}
function worldMotionBlocked(o,next,points){
  const st=worldMotionState(o),steps=Math.min(32,Math.max(1,Math.ceil(Math.abs(next-st.t)*(o.animation.type==='slide'?nrm(o.animation.offset):Math.abs(o.animation.angle)*D2R*Math.max(...o.size)*o.scale)/.02)));
  for(let i=1;i<=steps;i++){
    const pose=worldMotionPose(o,st.t+(next-st.t)*i/steps),shapes=worldMotionShapes(o,pose);
    for(const p of points)for(const b of shapes)if(nearBox(p.p,p.r+.002,b)){
      const hit=boxContact(p.p,p.r,b);if(hit&&hit.depth>1e-6){const old=boxContact(p.p,p.r,{...b,pose:o.motionPose||worldMotionPose(o,st.t)});if(!old||hit.depth>old.depth+1e-7)return true;}
    }
    // Sample the moving surface against scenery too, ignoring pre-existing overlap unless it worsens.
    const oldPose=o.motionPose||worldMotionPose(o,st.t),fixed=terrainIndex().filter(b=>b.obj!==o.id&&shapes.some(s=>s.lo.every((v,k)=>v<b.hi[k]&&s.hi[k]>b.lo[k])));
    if(fixed.length||shapes.some(b=>b.lo[2]<0))for(const rest of worldMotionRestPoints(o)){
      const p=add(pose.origin,m3v(pose.R,rest)),prev=add(oldPose.origin,m3v(oldPose.R,rest));
      if(p[2]<-1e-6&&p[2]<prev[2]-1e-7)return true;
      for(const group of fixed)for(const b of group.boxes||[group])if(nearBox(p,.002,b)){
        const h=boxContact(p,.002,b),old=boxContact(prev,.002,b);if(h&&h.depth>1e-6&&(!old||h.depth>old.depth+1e-7))return true;
      }
    }
  }return false;
}
function worldMotionCommand(arg){const o=worldObj(arg?.[0]);if(!o?.animation||o.animation.mode!=='button')return;const s=worldMotionState(o);s.target=arg[1]?1:0;s.reversing=false;s.cooldown=0;worldMotionControls();}
function worldMotionStep(dt){
  const objects=worldObjects.list.filter(o=>o.animation);if(!objects.length||wedit.on)return;
  let moved=false,points=null;
  for(const o of objects){const a=o.animation,s=worldMotionState(o);s.blocked=false;s.cooldown=Math.max(0,(s.cooldown||0)-dt);
    if(a.mode==='loop'&&!s.cooldown&&Math.abs(s.t-s.target)<1e-9){s.wait+=dt;if(s.wait>=(s.target?a.waitOpen:a.waitClosed)){s.target=1-s.target;s.wait=0;}}
    if(a.mode==='proximity'){
      const center=add(o.pos,m3v(eulerR(0,0,o.yaw),a.zone)),inside=fleet.drones.some(d=>nrm(sub(d.state.S.p,center))<=a.radius+(s.target?.1:0));
      if(!s.reversing&&!(s.cooldown>0)){if(inside){s.empty=0;s.target=1;}else{s.empty+=dt;if(s.empty>=a.delay)s.target=0;}}
    }
    const next=s.t+clamp(s.target-s.t,-dt/a.duration,dt/a.duration);
    if(Math.abs(next-s.t)>1e-12&&a.blocked!=='push'){
      points ||=worldMotionObstacles();if(worldMotionBlocked(o,next,points)){s.blocked=true;if(a.blocked==='reverse'){s.target=1-s.target;s.reversing=true;s.wait=0;}worldMotionApply(o,s.t);moved=true;continue;}
    }
    if(s.reversing&&Math.abs(next-s.target)<1e-9){s.reversing=false;s.cooldown=1;s.empty=0;}
    if(next!==s.t){worldMotionApply(o,next,dt);moved=true;}else if(o.motionPose&&(nrm(o.motionPose.v)||nrm(o.motionPose.w))){worldMotionApply(o,s.t);moved=true;}
  }
  if(moved){worldObjectsChanged(false);terrain.motionVer=(terrain.motionVer||0)+1;}
}
function worldMotionOpen(o){
  if(worldMotionEditor.id!==o.id)worldMotionEditor.camera={target:cam.target.clone(),pan:cam.pan.clone(),dist:cam.dist};
  $('#worldPanel').classList.add('animation-panel');
  worldMotionEditor.id=o.id;worldMotionEditor.playing=false;worldMotionEditor.pick=false;
  if(!o.animation)o.animation=worldMotionValidate(WORLD_MOTION_DEFAULT);
  worldMotionApply(o,0);worldObjectsChanged(false);worldEditChanged();worldEditRender();worldMotionGuides();worldMotionFit(o);worldEditMsg();
}
function worldMotionClose(){
  $('#worldPanel').classList.remove('animation-panel');
  if(worldMotionEditor.camera){cam.target.copy(worldMotionEditor.camera.target);cam.pan.copy(worldMotionEditor.camera.pan);cam.dist=worldMotionEditor.camera.dist;worldMotionEditor.camera=null;}
  const o=worldObj(worldMotionEditor.id);worldMotionEditor.id=null;worldMotionEditor.playing=false;worldMotionEditor.pick=false;
  if(o){worldMotionApply(o,0);delete o.motionState;}worldObjectsChanged(false);worldMotionGuides();worldEditMsg();
}
function worldMotionEdit(o){worldMotionEditor.playing=false;worldMotionApply(o,worldMotionState(o).t);worldObjectsChanged();worldEditChanged();worldMotionGuides();worldMotionFit(o);}
function worldMotionFit(o){
  if(!o?.animation)return;const body={lo:[Infinity,Infinity,Infinity],hi:[-Infinity,-Infinity,-Infinity]},bounds={lo:[Infinity,Infinity,Infinity],hi:[-Infinity,-Infinity,-Infinity]};
  for(const b of o.boxes)for(let k=0;k<3;k++){body.lo[k]=Math.min(body.lo[k],b.lo[k]);body.hi[k]=Math.max(body.hi[k],b.hi[k]);}
  for(let i=0;i<=16;i++){const b=worldMotionBounds(body,worldMotionPose(o,i/16));for(let k=0;k<3;k++){bounds.lo[k]=Math.min(bounds.lo[k],b.lo[k]);bounds.hi[k]=Math.max(bounds.hi[k],b.hi[k]);}}
  cam.target.set(...bounds.lo.map((v,k)=>(v+bounds.hi[k])/2));cam.pan.set(0,0,0);cam.dist=Math.max(.6,nrm(sub(bounds.hi,bounds.lo))*1.8);cam.anim=null;
}
function worldMotionPick(e){
  if(!worldMotionEditor.pick)return false;const o=worldObj(worldMotionEditor.id);if(!o)return false;rayFrom(e);const h=raycaster.intersectObject(o.g,true).find(h=>h.object.isMesh);
  if(h){o.animation.pivot=m3v(m3T(eulerR(0,0,o.yaw)),sub(h.point.toArray(),o.pos));worldMotionEditor.pick=false;worldMotionEdit(o);worldEditRender();}return true;
}
let worldMotionGuide=null;
function worldMotionGuides(){
  if(worldMotionGuide){scene.remove(worldMotionGuide);worldMotionGuide.traverse(m=>m.material?.dispose());disposeGroup(worldMotionGuide);worldMotionGuide=null;}
  const o=worldObj(worldMotionEditor.id);if(!o?.animation||!wedit.on)return;
  const a=o.animation,Y=eulerR(0,0,o.yaw),g=worldMotionGuide=new THREE.Group();scene.add(g);g.userData.noPick=true;
  const line=(pts,color)=>{const geo=new THREE.BufferGeometry().setFromPoints(pts.map(p=>new THREE.Vector3(...p))),m=new THREE.Line(geo,new THREE.LineBasicMaterial({color,depthTest:false}));m.renderOrder=30;g.add(m);};
  if(a.type==='hinge'){const at=add(o.pos,m3v(Y,a.pivot)),axis=m3v(Y,[0,1,2].map(k=>k===a.axis?1:0));line([sub(at,scl(axis,.5)),add(at,scl(axis,.5))],0xe98232);const ball=new THREE.Mesh(new THREE.SphereGeometry(.025,12,8),new THREE.MeshBasicMaterial({color:0xe98232,depthTest:false}));ball.position.set(...at);ball.renderOrder=30;g.add(ball);}
  else line([o.pos,add(o.pos,m3v(Y,a.offset))],0x2b55df);
  if(a.mode==='proximity'){const zone=new THREE.Mesh(new THREE.SphereGeometry(a.radius,20,12),new THREE.MeshBasicMaterial({color:0x24a86b,wireframe:true,transparent:true,opacity:.3,depthWrite:false}));zone.position.set(...add(o.pos,m3v(Y,a.zone)));g.add(zone);}
}
function worldMotionFrame(dt){
  const o=worldObj(worldMotionEditor.id);if(o&&wedit.on&&worldMotionEditor.playing){const t=worldMotionState(o).t+dt/o.animation.duration;worldMotionApply(o,Math.min(1,t));worldObjectsChanged(false);if(t>=1)worldMotionEditor.playing=false;const n=$('#worldMotionTimeline');if(n)n.value=String(worldMotionState(o).t);const l=$('#worldMotionTime');if(l)l.textContent=(worldMotionState(o).t*o.animation.duration).toFixed(2)+' s';}
  const play=$('#worldMotionPlay');if(play)play.setAttribute('aria-pressed',String(worldMotionEditor.playing));
  worldMotionControls();
}
let worldMotionControlKey='';
function worldMotionControls(){
  let bar=$('#worldMotionControls');if(!bar){bar=el('div',{id:'worldMotionControls',class:'world-motion-controls','aria-label':'World object controls'});$('.view').append(bar);}
  const objects=worldObjects.list.filter(o=>o.animation?.mode==='button'),key=objects.map(o=>o.id+o.name+worldMotionState(o).target+worldMotionState(o).blocked).join('|')+'|'+wedit.on+'|'+liveOn();if(key===worldMotionControlKey)return;worldMotionControlKey=key;
  bar.hidden=!objects.length||wedit.on||liveOn();bar.replaceChildren(...objects.map(o=>UI.button({class:'btn btn-sm',disabled:liveOn()||undefined,'data-world-motion':o.id,onclick:()=>userAction('worldMotion',[o.id,1-worldMotionState(o).target])},(worldMotionState(o).blocked?'Blocked · ':'')+(worldMotionState(o).target?'Close ':'Open ')+o.name)));
}
function worldMotionFields(o){
  const a=o.animation,open=worldMotionEditor.id===o.id;
  if(!open)return el('div',{class:'world-motion-summary'},UI.button({class:'btn btn-sm',id:'worldMotionEdit',onclick:()=>worldMotionOpen(o)},a?'Edit animation…':'Add animation…'),a?el('p',{class:'hint',text:`${a.type==='hinge'?'Hinge':'Slide'} · ${a.mode} · ${a.duration} s`}):null);
  const field=(key,label,min,max,step,u='',set)=>numField('worldMotion-'+key,{label,min,max,step,u,dp:step<1?2:0,hard:true},()=>a[key],v=>{a[key]=v;(set||(()=>worldMotionEdit(o)))();}).node;
  const select=(key,label,opts)=>{const n=UI.select({id:'worldMotion-'+key,'aria-label':label},...opts.map(([v,t])=>el('option',{value:v,text:t})));n.value=String(a[key]);n.onchange=()=>{a[key]=key==='axis'?Number(n.value):n.value;worldMotionEdit(o);worldEditRender();};return UI.field({label},n);};
  const vector=(key,label)=>UI.field({label},el('div',{class:'hrow world-motion-vector'},...[0,1,2].map(i=>{const n=UI.input({type:'number',class:'num',id:`worldMotion-${key}-${i}`,step:.01,min:-100,max:100,value:+a[key][i].toFixed(3),'aria-label':label+' '+'XYZ'[i]+' in metres'});n.onchange=()=>{const v=Number(n.value);if(n.value!==''&&Number.isFinite(v)){a[key][i]=clamp(v,-100,100);worldMotionEdit(o);}else n.value=a[key][i];};return el('label',{},'XYZ'[i],n);})),el('span',{class:'hint',text:'Local metres from the object’s base.'}));
  const timeline=UI.input({type:'range',id:'worldMotionTimeline',min:0,max:1,step:.001,value:worldMotionState(o).t,'aria-label':'Animation preview timeline'});timeline.oninput=()=>{worldMotionEditor.playing=false;worldMotionApply(o,+timeline.value);worldObjectsChanged(false);$('#worldMotionTime').textContent=(+timeline.value*a.duration).toFixed(2)+' s';};
  return el('section',{class:'world-motion-editor','aria-label':'World animation editor'},
    el('div',{class:'hrow'},el('strong',{text:o.name+' · Animation'}),UI.button({class:'btn btn-sm',id:'worldMotionDone',onclick:()=>{worldMotionClose();worldEditRender();}},'Done')),
    el('p',{class:'hint',text:'The placed object is the closed pose. Open defines a relative second keyframe. Motion affects this world copy and its collision shape.'}),
    select('type','Motion',[['hinge','Hinge'],['slide','Slide']]),
    ...(a.type==='hinge'?[select('axis','Hinge axis',[[0,'X'],[1,'Y'],[2,'Z']]),vector('pivot','Hinge point'),UI.button({class:'btn btn-sm',id:'worldMotionPick',onclick:()=>{worldMotionApply(o,0);worldObjectsChanged(false);worldMotionEditor.pick=!worldMotionEditor.pick;worldEditRender();}},worldMotionEditor.pick?'Click the hinge on the object…':'Pick hinge on object'),field('angle','Open angle',-360,360,1,'°')]:[vector('offset','Open slide offset')]),
    field('duration','Travel time',.1,120,.1,'s'),select('ease','Speed',[['smooth','Smooth start / stop'],['linear','Constant']]),
    UI.field({label:'Timeline: Closed → Open'},timeline),el('span',{id:'worldMotionTime',class:'hint',text:(worldMotionState(o).t*a.duration).toFixed(2)+' s'}),
    el('div',{class:'hrow'},UI.button({class:'btn btn-sm',id:'worldMotionPlay',onclick:()=>{if(worldMotionState(o).t>=1)worldMotionApply(o,0);worldMotionEditor.playing=!worldMotionEditor.playing;}},'Play / pause'),UI.button({class:'btn btn-sm',id:'worldMotionFit',onclick:()=>worldMotionFit(o)},'Frame motion'),UI.button({class:'btn btn-sm',id:'worldMotionClosed',onclick:()=>{worldMotionEditor.playing=false;worldMotionApply(o,0);worldObjectsChanged(false);worldEditRender();}},'Closed'),UI.button({class:'btn btn-sm',id:'worldMotionOpen',onclick:()=>{worldMotionEditor.playing=false;worldMotionApply(o,1);worldObjectsChanged(false);worldEditRender();}},'Open')),
    select('mode','Activate',[['button','Button'],['loop','Loop'],['proximity','Drone proximity']]),
    ...(a.mode==='loop'?[field('waitOpen','Wait open',0,120,.1,'s'),field('waitClosed','Wait closed',0,120,.1,'s')]:a.mode==='proximity'?[vector('zone','Detection zone center'),field('radius','Detection radius',.05,100,.05,'m'),field('delay','Close after zone empty',0,120,.1,'s')]:[el('p',{class:'hint',text:'An Open / Close button appears over the simulation view.'})]),
    select('blocked','When blocked',[['stop','Stop until clear'],['reverse','Reverse'],['push','Continue and push']]),
    UI.button({class:'btn btn-sm',id:'worldMotionRemove',onclick:()=>{worldMotionClose();delete o.animation;delete o.motionPose;delete o.motionPrevious;placeGroup(o);worldObjectsChanged();worldEditChanged();worldEditRender();}},'Remove animation'));
}
