'use strict';
const $ = id => document.getElementById(id);
let state, version = -1, model, active = false, yaw = -.38, pitch = .42, zoom = 1;
let timeline=[], timelineRun, replay=false, cursor=0, animation=0, displayedFrame;
let sampleOrder=new Map();
let mapRun, gapRanges=[], gapRequest=0;
let estimate=null, tracking=false, estimating=false, generation=0;
let centre = [0,0,0], extent = 1, drag;
const canvas = $('map'), ctx = canvas.getContext('2d');
function draw() {
  const w=canvas.clientWidth, h=canvas.clientHeight, d=devicePixelRatio;
  canvas.width=w*d; canvas.height=h*d; ctx.setTransform(d,0,0,d,0,0);
  if (!model) return;
  const project = p => {
    const [x,y,z]=p.slice(0,3).map((v,i)=>v-centre[i]);
    const u=x*Math.cos(yaw)-y*Math.sin(yaw), v=x*Math.sin(yaw)+y*Math.cos(yaw);
    const scale=Math.min(w,h)*zoom/extent;
    return [w/2+u*scale,h/2-(z*Math.cos(pitch)+v*Math.sin(pitch))*scale];
  };
  if($('surfaceLayer').checked&&model.surfaces?.patches.length){
    const depth=([x,y,z])=>(x*Math.sin(yaw)+y*Math.cos(yaw))*Math.cos(pitch)-z*Math.sin(pitch);
    const patches=model.surfaces.patches.map(p=>({p,z:p.vertices.reduce((sum,v)=>sum+depth(v),0)/4}));
    patches.sort((a,b)=>b.z-a.z);
    for(const {p} of patches){
      const [a,b,c]=p.vertices;
      const u=b.map((n,i)=>n-a[i]),v=c.map((n,i)=>n-a[i]);
      const n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];
      const shade=.55+.4*Math.abs(depth(n))/Math.max(1e-9,Math.hypot(...n));
      const rgb=(p.tof_supported?[130,235,196]:p.inferred?[182,160,126]:p.kind==='floor'?[142,167,234]:[127,184,216]).map(n=>Math.round(n*shade));
      ctx.fillStyle='rgba('+rgb.join(',')+','+(p.inferred?.62:.9)+')';
      ctx.beginPath();
      p.vertices.forEach((vertex,i)=>i?ctx.lineTo(...project(vertex)):ctx.moveTo(...project(vertex)));
      ctx.closePath();ctx.fill();
    }
  }
  for (const p of ($('cloud').checked ? model.points : [])) {
    const [x,y]=project(p);
    ctx.fillStyle='rgb('+p.slice(3).map(v=>Math.round(55+.78*v)).join(',')+')'; ctx.fillRect(x,y,2,2);
  }
  if(estimate?.xyz){
    const a=project(estimate.xyz), v=estimate.rotation.map(row=>row[2]);
    const b=project(estimate.xyz.map((x,i)=>x+v[i]*extent*.10));
    ctx.strokeStyle='#58baff';ctx.fillStyle='#58baff';ctx.lineWidth=3;
    ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();
    ctx.beginPath();ctx.arc(...a,6,0,Math.PI*2);ctx.fill();
  }
  if(!$('route').checked)return;
  ctx.strokeStyle='#a5efd1'; ctx.lineWidth=2; ctx.beginPath();
  model.poses.forEach((p,i)=>{const [x,y]=project(p.xyz); if(i && (!sampleOrder.size || sampleOrder.get(p.frame)===sampleOrder.get(model.poses[i-1].frame)+1)) ctx.lineTo(x,y); else ctx.moveTo(x,y);});
  ctx.stroke();
  for(const p of model.poses){
    const [x,y]=project(p.xyz);
    const fraction=parseInt(p.frame)/Math.max(1,(state?.frames||1)-1);
    ctx.fillStyle='hsl('+(167-130*fraction)+' 78% 71%)';
    ctx.fillRect(x-2,y-2,4,4);
  }
  const first=model.poses[0], last=model.poses.at(-1);
  const marker=(pose,label,color,below=false)=>{
    const [x,y]=project(pose.xyz);
    ctx.fillStyle=color; ctx.beginPath(); ctx.arc(x,y,5,0,Math.PI*2); ctx.fill();
    ctx.font='600 12px system-ui';
    const tx=Math.min(w-ctx.measureText(label).width-8,Math.max(5,x+9));
    const ty=Math.min(h-5,Math.max(13,y+(below?17:-10)));
    ctx.strokeStyle='#0b1012';ctx.lineWidth=4;ctx.strokeText(label,tx,ty);ctx.fillText(label,tx,ty);
  };
  marker(first,parseInt(first.frame)===0?'Start':'First mapped pose','#a5efd1');
  marker(last,parseInt(last.frame)===state.frames-1?'Finish':state.recording?'Latest pose':'Last mapped pose','#ffcf80',true);
  const selected=replay&&model.poses.find(p=>p.frame===timeline[cursor]?.id);
  if(selected){
    const [x,y]=project(selected.xyz);
    ctx.strokeStyle='#72caff';ctx.lineWidth=3;ctx.beginPath();ctx.arc(x,y,9,0,Math.PI*2);ctx.stroke();
  }
}
function showMap(m) {
  model=m;
  updateSurfaceInfo();
  updateGaps();
  updateStreamControls();
  sampleOrder=new Map((m?.path_frames||m?.sampled_frames||[]).map((f,i)=>[f,i]));
  $('estimate').disabled=$('continuous').disabled=!m;
  const label=m ? 'Frames '+parseInt(m.poses[0].frame)+'–'+parseInt(m.poses.at(-1).frame)+' · '+state.frames+' saved' : '';
  $('coverage').textContent=label; $('mapempty').hidden=!!m;
  $('poses').textContent=m ? (m.registered_views??m.poses.length)+' / '+m.views : '—';
  $('points').textContent=m ? m.points.length.toLocaleString() : '—';
  $('residual').textContent=m ? m.mean_error_px+' px' : '—';
  $('units').textContent=m?.units==='m'?(m.reference?'Measured baseline · m':m.tof_scale?'ToF scale estimate · m':'Measured scale · m'):'Relative scale';
  if(m?.gravity_aligned) $('units').textContent+=' · IMU aligned';
  if(m) {
    const axes=[0,1,2].map(i=>m.points.map(p=>p[i]).sort((a,b)=>a-b));
    const bounds=axes.map(v=>[v[Math.floor(v.length*.03)],v[Math.floor(v.length*.97)]]);
    for(const p of m.poses) p.xyz.forEach((v,i)=>{bounds[i][0]=Math.min(v,bounds[i][0]);bounds[i][1]=Math.max(v,bounds[i][1]);});
    centre=bounds.map(([a,b])=>(a+b)/2); extent=Math.max(1e-5,...bounds.map(([a,b])=>b-a))*1.4;
  }
  draw();
}
canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);};
canvas.onpointermove=e=>{if(drag){yaw+=(e.clientX-drag[0])*.008;pitch=Math.max(-1.5,Math.min(1.5,pitch+(e.clientY-drag[1])*.008));drag=[e.clientX,e.clientY];draw();}};
canvas.onpointerup=canvas.onpointercancel=()=>drag=null;
canvas.onwheel=e=>{e.preventDefault();zoom=Math.max(.15,Math.min(12,zoom*Math.exp(-e.deltaY*.002)));draw();};
new ResizeObserver(draw).observe(canvas);
async function update() {
  const s=await (await fetch('/state')).json(); state=s;
  $('connection').textContent=s.connection; $('connection').classList.toggle('live',s.fresh);
  const connected=['Live','Connecting'].includes(s.connection);
  $('connect').textContent=connected?'Disconnect':'Connect camera'; $('connect').disabled=s.recording;
  $('record').textContent=s.recording?'Finish walk':'Start walk'; $('record').disabled=!s.recording&&!s.fresh;
  $('record').classList.toggle('recording',s.recording);
  let recordStatus = 'Ready to record';
  if(s.recording) recordStatus = 'Recording in progress';
  else if(!s.fresh) recordStatus = connected?'Waiting for live camera frames':'Connect the camera to record';
  if(s.busy) recordStatus += s.fresh&&!s.recording
    ? ' · You can start a walk while the map rebuilds'
    : ' · Reconstruction running in the background';
  if($('recordStatus').textContent!==recordStatus) $('recordStatus').textContent=recordStatus;
  updateCamera();

  if(!replay) $('rate').textContent=s.fresh?s.fps+' fps · 640 × 480 · '+(s.tofRaw==null?'ToF unavailable':'ToF '+s.tofRaw+' raw (~cm)')+(s.tofHz==null?'':' · '+s.tofHz+' ToF reads/s ('+s.tofValidPercent+'% valid)')+' · IMU '+(s.motionFresh?'live':'unavailable'):'OV5647 · 640 × 480';
  $('frames').textContent=s.frames.toLocaleString(); $('walk').textContent=s.run||'No walk selected';
  $('captureWarning').hidden=!s.fresh||s.motionFresh;
  $('message').textContent=s.message; $('samples').textContent=s.samples+' views'; $('lens').textContent=s.calibrated?'Calibrated':'Self-estimate available';
  $('download').hidden=!s.hasMap;
  updateStreamControls();
  $('load').disabled=s.busy||s.recording; $('sample').disabled=!s.fresh||s.recording;
  $('calibrate').disabled=s.samples<12||s.recording;
  $('markA').disabled=!s.recording; $('markB').disabled=!s.recording||!s.refs.A;
  $('refs').textContent=[s.refs.A&&'A saved',s.refs.B&&'B saved'].filter(Boolean).join(' · ');
  const run=s.recording?null:s.run;
  if(run!==timelineRun){
    pauseReplay();
    clearGapEvidence();
    const response=run&&await fetch('/timeline?'+new URLSearchParams({run}));
    timeline=response?.ok?await response.json():[];
    timelineRun=run; displayedFrame=null; cursor=0;
    updateGaps();
    $('timeline').max=timeline.at(-1)?.ms||0;
    $('timeline').value=0;
    replay=timeline.length>0;
    $('cameraMode').disabled=!timeline.length;
    refreshReplay();
  }
  if(s.version!==version) {
    const m=s.hasMap?await(await fetch('/survey.json')).json():null;
    if(mapRun!==s.run){
      setStreams(m?.streams||streamChoices.map(choice=>choice.key));
      mapRun=s.run;
    }
    resetTracking(); showMap(m); version=s.version;
    refreshReplay();
    const list=await(await fetch('/runs')).json(), old=$('runs').value;
    $('runs').replaceChildren(...list.map(n=>new Option(n,n)));
    $('runs').value=list.includes(old)?old:(s.run||list[0]||'');
  }
  updateWalkActions();
}
async function act(key,body={}) {
  if(active)return; active=true;
  try{
    const r=await fetch('/'+key,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    const v=await r.json(); if(!r.ok)throw Error(v.error);
    $('error').textContent=''; await update();
  }catch(e){$('error').textContent=e.message;}finally{active=false;}
}
$('connect').onclick=()=>act(['Live','Connecting'].includes(state.connection)?'disconnect':'connect');
$('record').onclick=()=>act(state.recording?'stop':'start');
$('markA').onclick=()=>act('A'); $('markB').onclick=()=>act('B',{metres:$('metres').value});
$('load').onclick=()=>act('load',{name:$('runs').value});
function updateWalkActions(){
  const name=$('runs').value;
  const unfinished=(name===state?.run&&state.recording)||state?.buildingWalks?.includes(name);
  $('archive').disabled=$('deleteWalk').disabled=!name||unfinished;
  $('load').disabled=!name||state?.recording||state?.busy;
}
$('runs').onchange=updateWalkActions;
$('archive').onclick=()=>{
  const link=document.createElement('a');
  link.href='/walk.zip?'+new URLSearchParams({run:$('runs').value});
  link.download=$('runs').value+'.zip';
  link.click();
};
$('deleteWalk').onclick=()=>{
  const name=$('runs').value;
  if(confirm('Move walk '+name+' and all its files to survey trash?')){
    act('delete',{name});
  }
};
for(const id of ['sample','calibrate']) $(id).onclick=()=>act(id);
$('reset').onclick=()=>act('reset-samples');
const preview=$('preview');
let previewPending=false;
preview.onload=preview.onerror=()=>{previewPending=false;};
setInterval(()=>{
  if(!replay && state?.fresh && !document.hidden && !previewPending) {
    previewPending=true; preview.src='/preview.jpg?t='+Date.now();
  }
},125);
async function poll(){try{if(!active)await update();}catch(e){$('connection').textContent='Dashboard offline';$('record').disabled=true;$('recordStatus').textContent='Dashboard offline';}finally{setTimeout(poll,450);}}
function resetTracking(){
  generation++;tracking=false;estimate=null;$('continuous').textContent='Start tracking';
  $('matches').hidden=true;$('tracking').textContent='Choose live tracking or inspect a recorded frame.';draw();
}
async function locate(){
  if(estimating||!state||!model)return;
  estimating=true;const token=generation,source=$('source').value;
  try{
    const r=await fetch('/locate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({
      run:state.run,source,frame:parseInt(timeline[cursor]?.id),minimum:+$('minimum').value})});
    const q=await r.json();if(token!==generation)return;
    if(!r.ok||q.version!==version)throw Error(q.error||'Map changed');
    const img=new Image();img.src='data:image/jpeg;base64,'+q.image;await img.decode();if(token!==generation)return;
    estimate=q;draw();const el=$('matches'),c=el.getContext('2d');el.hidden=false;c.drawImage(img,0,0);
    c.fillStyle='#38ff9e';for(const [x,y] of q.pixels||[])c.fillRect(x-2,y-2,4,4);
    $('tracking').textContent=q.status+' · '+q.inliers+' inliers / '+q.matches+' matches · '+(source==='live'?'camera '+(q.stamp/1e6).toFixed(2)+' s':'frame '+q.stamp);
  }catch(e){if(token===generation){resetTracking();$('tracking').textContent=e.message;}}
  finally{estimating=false;}
}
$('estimate').onclick=locate;$('continuous').onclick=()=>{tracking=!tracking;$('continuous').textContent=tracking?'Pause tracking':'Start tracking';if(tracking)locate();};
setInterval(()=>{if(tracking&&!document.hidden)locate();},750);
$('source').onchange=resetTracking;
function updateSurfaceInfo(){
  const patches=model?.surfaces?.patches||[];
  const toggle=$('surfaceLayer'), info=$('surfaceInfo');
  toggle.disabled=!patches.length;
  info.hidden=!(model&&(!patches.length||toggle.checked));
  if(!patches.length){
    info.textContent=model?.surfaces?'Too little planar evidence for surfaces.':'Rebuild the walk to generate surfaces.';
    return;
  }
  const supported=patches.filter(p=>p.tof_supported).length;
  const range=supported
    ? supported+' ToF-corroborated (green)'
    : model.units!=='m'?'ToF overlay needs metric scale':'No patches corroborated by ToF';
  const inferred=patches.filter(p=>p.inferred).length;
  info.textContent=patches.length+' surface polygons · '+range+
    (inferred?' · '+inferred+' interpolated polygons (tan)':'')+(model.surfaces.floor_plane!=null?' · Floor candidate (blue-violet)':'')+' · Open areas are unknown';
}
$('surfaceLayer').onchange=()=>{updateSurfaceInfo();draw();};
$('cloud').onchange=$('route').onchange=draw;
$('top').onclick=()=>{yaw=0;pitch=Math.PI/2;draw();};$('iso').onclick=()=>{yaw=-.38;pitch=.42;zoom=1;draw();};
$('range').onclick=async()=>{try{const r=await fetch('/range',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({address:Number($('address').value)})});
const q=await r.json();$('reading').textContent=r.ok?'Raw reading '+q.raw:q.error;}catch(e){$('reading').textContent=e.message;}};
const sensorTypes=[
  ['tof','ToF','m',true], ['gyro','Gyroscope','rad/s',true],
  ['accel','Acceleration','m/s²',true], ['height','Barometric height','m',true],
  ['attitude','Attitude quaternion','',false],
];
const sensorFields=sensorTypes.map(([key,name,unit,on])=>{
  const toggle=document.createElement('input'); toggle.type='checkbox';toggle.checked=on;
  const label=document.createElement('label');label.append(toggle,' '+name);$('sensorToggles').append(label);
  const field=document.createElement('div'), title=document.createElement('small'), output=document.createElement('output');
  title.textContent=name;field.append(title,output);$('sensorValues').append(field);
  toggle.onchange=()=>{field.hidden=!toggle.checked;};field.hidden=!on;
  return {key,unit,output};
});
function updateCamera(){
  $('cameraTitle').textContent=replay?'Recorded camera':'Live camera';
  $('cameraMode').textContent=replay?'Show live camera':'Review walk';
  $('replayPanel').hidden=!replay;
  $('replayImage').hidden=!replay;
  $('preview').hidden=replay||!state?.fresh;
  $('cameraempty').hidden=replay||!!state?.fresh;
}
function drawTimeline(){
  const plot=$('timelinePlot'), width=plot.clientWidth;
  if(!width||!timeline.length)return;
  const d=devicePixelRatio;
  plot.width=width*d;plot.height=13*d;
  const g=plot.getContext('2d');g.scale(d,d);
  const duration=Math.max(1,timeline.at(-1).ms);
  const x=ms=>2+ms/duration*(width-4);
  const registered=new Map(model?.poses.map(p=>[p.frame,p.method]));
  for(const frame of timeline){
    g.fillStyle=registered.has(frame.id)?(registered.get(frame.id)?'#72caff':'#a5efd1'):'#556469';
    g.fillRect(x(frame.ms),3,2,7);
  }
  for(const gap of selectedStreams().includes('camera')?gapRanges:[]){
    g.fillStyle='#c28453';
    g.fillRect(x(gap.first.ms),3,Math.max(2,x(gap.last.ms)-x(gap.first.ms)),7);
  }
  g.fillStyle='#72caff';g.fillRect(x(timeline[cursor].ms)-1,0,3,13);
}
new ResizeObserver(drawTimeline).observe($('timelinePlot'));
function refreshReplay(){
  updateCamera();
  const frame=timeline[cursor];
  if(!replay||!frame){draw();return;}
  const key=timelineRun+'/'+frame.id;
  if(displayedFrame!==key){
    displayedFrame=key;
    $('replayImage').src='/recorded.jpg?'+new URLSearchParams({run:timelineRun,frame:frame.id});
  }
  const seconds=ms=>(ms/1000).toFixed(2)+' s';
  $('frameNo').textContent=seconds(frame.ms)+' / '+seconds(timeline.at(-1).ms)+' · Frame '+parseInt(frame.id);
  $('rate').textContent='Recorded · '+(frame.tof>0&&frame.tof<250?'ToF '+(frame.tof/104).toFixed(2)+' m':'ToF unavailable')+
    ' · '+(frame.gyro?'IMU paired':'No paired IMU');
  const hasPose=model?.poses.find(p=>p.frame===frame.id);
  $('timelineStatus').textContent=hasPose
    ? (hasPose.method==='aligned_window'?'Pose aligned from overlapping views':hasPose.method?'Pose recovered from nearby frames':'Pose registered in the map')
    : model?.sampled_frames?.includes(frame.id)
      ? 'Unmapped · sampled but pose unresolved in the main map'
      : 'Unmapped · outside the initial sample and no recovery passed';
  const visual=selectedStreams().includes('camera');
  $('gapPicker').hidden=!visual||!gapRanges.length;
  $('inspectGap').hidden=!visual||!!hasPose||!model?.feature_index;
  $('inspectGap').disabled=!!state?.busy;
  if(!visual) $('timelineStatus').textContent='Sensor-only view';
  drawTraces();
  for(const {key,unit,output} of sensorFields){
    const v=frame[key];
    if(key==='tof'){
      output.textContent=v==null?'Unavailable':v>0&&v<250?(v/104).toFixed(2)+' m · '+v+' raw':'Invalid · '+v+' raw';
    }else if(Array.isArray(v)){
      const axes=key==='attitude'?['w','x','y','z']:['x','y','z'];
      output.textContent=v.map((n,i)=>axes[i]+' '+(n==null?'—':n.toFixed(3))).join(' · ')+' '+unit;
    }else{
      output.textContent=v==null?'Unavailable':v.toFixed(2)+' '+unit;
    }
  }
  $('sampleTiming').textContent=frame.lag_ms==null?'No paired motion sample.':'Motion receipt offset '+frame.lag_ms+' ms from camera callback.';
  drawTimeline();draw();
}
function seek(ms){
  let left=0,right=timeline.length-1;
  while(left<right){
    const mid=Math.floor((left+right)/2);
    if(timeline[mid].ms<ms)left=mid+1;else right=mid;
  }
  const previous=timeline[left-1];
  const next=previous&&ms-previous.ms<timeline[left].ms-ms?left-1:left;
  if(cursor!==next){cursor=next;resetTracking();clearGapEvidence();}
  $('timeline').value=ms;
  refreshReplay();
}
function pauseReplay(){
  cancelAnimationFrame(animation);animation=0;$('play').textContent='Play';
}
$('timeline').oninput=()=>{pauseReplay();seek(+$('timeline').value);};
function jumpToEndpoint(frame){
  pauseReplay();
  seek(frame.ms);
}
$('first').onclick=()=>jumpToEndpoint(timeline[0]);
$('last').onclick=()=>jumpToEndpoint(timeline.at(-1));
$('play').onclick=()=>{
  if(animation){pauseReplay();return;}
  if(cursor===timeline.length-1)seek(0);
  const begin=performance.now()-timeline[cursor].ms;
  $('play').textContent='Pause';
  const tick=now=>{
    const end=timeline.at(-1).ms;
    const ms=Math.min(end,now-begin);
    if(ms>=end){seek(end);pauseReplay();return;}
    if(ms>=timeline[cursor].ms+40)seek(ms);
    animation=requestAnimationFrame(tick);
  };
  animation=requestAnimationFrame(tick);
};
$('cameraMode').onclick=()=>{pauseReplay();replay=!replay;refreshReplay();};
const streamNames={camera:'Camera',tof:'ToF',gyro:'Gyroscope',accel:'Accelerometer',barometer:'Barometer'};
const traceConfig={
  tof:['Range along beam · m', f=>f.tof>0&&f.tof<250?[f.tof/104]:[]],
  gyro:['Angular rate XYZ · rad/s', f=>f.gyro||[]],
  accel:['Specific force XYZ · m/s²', f=>f.accel||[]],
  barometer:['Estimated height · m', f=>[f.height]],
};
const streamChoices=Object.entries(streamNames).map(([key,name])=>{
  const input=document.createElement('input');
  input.type='checkbox';input.checked=true;
  const label=document.createElement('label');label.append(input,' '+name);
  $('inputStreams').append(label);
  input.onchange=()=>{clearGapEvidence();updateStreamControls();refreshReplay();};
  return {key,input};
});
function selectedStreams(){return streamChoices.filter(({input})=>input.checked).map(({key})=>key);}
function setStreams(keys){for(const {key,input} of streamChoices) input.checked=keys.includes(key);}
function updateStreamControls(){
  const selected=selectedStreams(), visual=selected.includes('camera');
  $('reconstructionInputs').hidden=!state?.run||!!state?.recording;
  $('rebuild').hidden=!visual;
  $('rebuild').disabled=!state?.run||state.busy||state.recording;
  $('sensorOnlyNote').hidden=visual;
  $('mapPanel').hidden=!visual;
  $('posePanel').hidden=!visual;
  const used=model?.streams;
  const saved=used?'Saved map inputs · '+used.map(key=>streamNames[key]).join(', '):
    model?'Saved map inputs · legacy reconstruction':'No saved reconstruction';
  $('streamSummary').textContent=visual?((used||Object.keys(streamNames)).join(',')!==selected.join(',')?'Selection changed · rebuild to apply. ':'')+saved:
    selected.length?'Selected traces · '+selected.map(key=>streamNames[key]).join(', '):'Select a sensor to inspect its trace.';
  const traceKeys=selected.filter(key=>key!=='camera');
  $('sensorTraces').hidden=!traceKeys.length;
  if($('sensorTraces').dataset.keys!==traceKeys.join(',')){
    $('sensorTraces').dataset.keys=traceKeys.join(',');
    $('sensorTraces').replaceChildren(...traceKeys.map(key=>{
      const wrapper=document.createElement('div'), title=document.createElement('p'), plot=document.createElement('canvas');
      title.textContent=traceConfig[key][0];plot.dataset.sensor=key;
      plot.setAttribute('aria-label',title.textContent);
      wrapper.append(title,plot);return wrapper;
    }));
  }
}
$('rebuild').onclick=()=>act('rebuild',{streams:selectedStreams()});
function drawTraces(){
  if($('sensorTraces').hidden||!timeline.length)return;
  for(const plot of $('sensorTraces').querySelectorAll('canvas')){
    const width=plot.clientWidth, height=plot.clientHeight, d=devicePixelRatio;
    if(!width||!height)continue;
    const points=timeline.map(traceConfig[plot.dataset.sensor][1]);
    const values=points.flat().filter(Number.isFinite);
    plot.width=width*d;plot.height=height*d;
    const g=plot.getContext('2d');g.scale(d,d);
    if(!values.length){g.fillStyle='#a2b3b6';g.font='12px system-ui';g.fillText('No valid readings',12,20);continue;}
    const [lo,hi]=values.reduce(([a,b],v)=>[Math.min(a,v),Math.max(b,v)],[Infinity,-Infinity]);
    const x=ms=>54+(width-60)*ms/Math.max(1,timeline.at(-1).ms);
    const y=v=>height-9-(height-18)*(hi===lo?.5:(v-lo)/(hi-lo));
    g.fillStyle='#a2b3b6';g.font='11px monospace';g.fillText(hi.toFixed(2),5,12);g.fillText(lo.toFixed(2),5,height-5);
    const colors=['#a5efd1','#ffba7c','#c3a9ff'];
    colors.slice(0,points.reduce((n,p)=>Math.max(n,p.length),0)).forEach((color,axis)=>{
      g.strokeStyle=color;g.beginPath();
      let previous=false;
      points.forEach((values,i)=>{
        const value=values[axis];
        if(Number.isFinite(value)){
          const px=x(timeline[i].ms), py=y(value);
          if(previous)g.lineTo(px,py);else g.moveTo(px,py);
        }
        previous=Number.isFinite(value);
      });
      g.stroke();
    });
    g.strokeStyle='#72caff';g.beginPath();g.moveTo(x(timeline[cursor].ms),0);g.lineTo(x(timeline[cursor].ms),height);g.stroke();
  }
}
new ResizeObserver(drawTraces).observe($('sensorTraces'));
function updateGaps(){
  const mapped=new Set(model?.poses.map(p=>p.frame));
  gapRanges=[];
  if(model){
    let start=null;
    for(let i=0;i<=timeline.length;i++){
      if(i===timeline.length||mapped.has(timeline[i].id)){
        if(start!==null&&i-start>=2)
          gapRanges.push({first:timeline[start],last:timeline[i-1],count:i-start});
        start=null;
      }else if(start===null)start=i;
    }
  }
  $('gaps').replaceChildren(new Option('Jump to a gap',''),...gapRanges.map(gap=>
    new Option((gap.first.ms/1000).toFixed(1)+'–'+(gap.last.ms/1000).toFixed(1)+' s · '+gap.count+' frames',gap.first.ms)));
}
$('gaps').onchange=()=>{
  if($('gaps').value==='')return;
  pauseReplay();seek(+$('gaps').value);$('gaps').value='';
};
function clearGapEvidence(){
  gapRequest++;
  $('gapEvidence').hidden=true;
}
$('inspectGap').onclick=async()=>{
  const token=++gapRequest, frame=parseInt(timeline[cursor].id);
  const output=$('gapEvidence');output.hidden=false;output.textContent='Checking this frame against the saved map…';
  try{
    const response=await fetch('/locate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({
      run:state.run,source:'replay',frame,minimum:12})});
    const data=await response.json();
    if(!response.ok)throw Error(data.error);
    if(token!==gapRequest)return;
    const detail=data.matches<12
      ? data.matches+' candidate matches · need 12 to try a pose'
      : data.inliers+' geometric inliers / '+data.matches+' candidate matches · '+
        (data.status==='Located'?'Independent pose check passed':'Pose check failed');
    output.textContent='Frame '+frame+' · '+detail;
  }catch(e){if(token===gapRequest)output.textContent=e.message;}
};
poll();
