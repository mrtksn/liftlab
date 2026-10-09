'use strict';
// World cues share the existing Web Audio output, spatial pan and simulation playback rate.
const WORLD_SOUND_EVENTS={start:'Motion starts',moving:'While moving',stop:'Motion stops',opened:'Reached Open',closed:'Reached Closed',blocked:'Blocked'};
const WORLD_SOUND_PRESETS={motor:'Motor whirr',clunk:'Metal clunk',click:'Click',beep:'Beep',whoosh:'Whoosh'};
const WORLD_SOUND_MAX_BYTES=16*1024*1024,WORLD_SOUND_MAX_DURATION=120;
const worldSounds={assets:new Map(),files:new Map(),buffers:new Map(),loads:new Map(),states:new Map(),voices:new Set(),loops:new Map(),pending:new Map(),failed:new Set(),queue:[],epoch:0,serial:0,uploads:0,ready:false,say:''};
const worldSoundDefault=key=>({source:'off',preset:({moving:'motor',start:'click',stop:'clunk',opened:'beep',closed:'clunk',blocked:'beep'})[key],volume:.65,pitch:1,duration:key==='moving'?1:key==='start'?.08:.2});
function worldSoundValidate(config){
  if(config==null)return undefined;if(typeof config!=='object'||Array.isArray(config))throw new Error('Invalid animation sounds');const out={};
  for(const key of Object.keys(config)){
    const c=config[key];if(!Object.hasOwn(WORLD_SOUND_EVENTS,key)||!c||!['off','generated','file'].includes(c.source)||!Object.hasOwn(WORLD_SOUND_PRESETS,c.preset)||!Number.isFinite(c.volume)||c.volume<0||c.volume>1||!Number.isFinite(c.pitch)||c.pitch<.25||c.pitch>4||!Number.isFinite(c.duration)||c.duration<.02||c.duration>WORLD_SOUND_MAX_DURATION||c.fileId!=null&&(typeof c.fileId!=='string'||!c.fileId||c.fileId.length>100)||c.source==='file'&&!c.fileId)throw new Error('Invalid animation sound cue');
    out[key]={source:c.source,preset:c.preset,volume:c.volume,pitch:c.pitch,duration:c.duration,...(c.fileId?{fileId:c.fileId,name:String(c.name||'Audio file').slice(0,256)}:{})};
  }return out;
}
function worldSoundRefs(objects){return [...new Set(objects.flatMap(o=>Object.values(o.animation?.sounds||{}).filter(c=>c.fileId).map(c=>c.fileId)))];}
function worldSoundAssetValidate(a){if(!a||typeof a.id!=='string'||!a.id||a.id.length>100||typeof a.name!=='string'||!a.name||a.name.length>256||typeof a.type!=='string'||a.type.length>100||!(a.data instanceof ArrayBuffer)||!a.data.byteLength||a.data.byteLength>WORLD_SOUND_MAX_BYTES||!Number.isFinite(a.duration)||a.duration<=0||a.duration>WORLD_SOUND_MAX_DURATION)throw new Error('Invalid sound file');return a;}
async function worldSoundInit(){
  try{const list=await worldDbDo('readonly',s=>s.getAll(),'audio');for(const a of list)worldSounds.assets.set(a.id,{id:a.id,name:a.name,type:a.type,duration:a.duration});}
  catch(e){worldSounds.say='Sound file storage is unavailable: '+e.message;}worldSounds.ready=true;if(worldMotionEditor.id)worldEditRender();
}
async function worldSoundAsset(id){if(worldSounds.files.has(id))return worldSounds.files.get(id);const a=await worldDbDo('readonly',s=>s.get(id),'audio');if(a){worldSoundAssetValidate(a);worldSounds.files.set(id,a);worldSounds.assets.set(id,{id:a.id,name:a.name,type:a.type,duration:a.duration});}return a;}
async function worldSoundDecode(data){
  try{const ctx=new (window.OfflineAudioContext||window.webkitOfflineAudioContext)(1,1,44100),b=await ctx.decodeAudioData(data.slice(0));if(b.duration>WORLD_SOUND_MAX_DURATION)throw new Error('Sound must be at most 120 seconds');return b;}
  catch(e){throw new Error(e.message.includes('120 seconds')?e.message:'This sound cannot be decoded by this browser. Try WAV, MP3 or Ogg.');}
}
function worldSoundBuffer(id){
  if(worldSounds.buffers.has(id))return Promise.resolve(worldSounds.buffers.get(id));if(worldSounds.loads.has(id))return worldSounds.loads.get(id);
  const task=(async()=>{const a=await worldSoundAsset(id);if(!a)throw new Error('The sound file is missing. Upload a replacement.');const b=await worldSoundDecode(a.data);worldSounds.buffers.set(id,b);return b;})();worldSounds.loads.set(id,task);task.finally(()=>worldSounds.loads.delete(id)).catch(()=>{});return task;
}
async function worldSoundPrepare(files){const buffers=await Promise.all(files.map(a=>{worldSoundAssetValidate(a);return worldSoundDecode(a.data);}));for(let i=0;i<files.length;i++){files[i].duration=buffers[i].duration;worldSounds.buffers.set(files[i].id,buffers[i]);worldSounds.failed.delete(files[i].id);}}
async function worldSoundImport(o,key,file){
  const animation=o.animation;if(!animation||worldSounds.uploads||maps.busy||wedit.busy)return;worldSounds.uploads++;worldSounds.say='Reading sound…';mapUiSync();const status=$('#worldSoundSay');if(status)status.textContent=worldSounds.say;worldEditRender();
  try{
    if(!file.size||file.size>WORLD_SOUND_MAX_BYTES)throw new Error('Sound files must be between 1 byte and 16 MB');const data=await file.arrayBuffer(),buffer=await worldSoundDecode(data),a={id:newObjId('snd-'),name:file.name.slice(0,256),type:file.type||'audio/*',duration:buffer.duration,data};
    await worldDbDo('readwrite',s=>s.put(a),'audio');worldSounds.files.set(a.id,a);worldSounds.buffers.set(a.id,buffer);worldSounds.assets.set(a.id,{id:a.id,name:a.name,type:a.type,duration:a.duration});
    if(worldObj(o.id)!==o||o.animation!==animation)return;animation.sounds ||= {};animation.sounds[key]={...(animation.sounds[key]||worldSoundDefault(key)),source:'file',fileId:a.id,name:a.name,duration:Math.max(.02,a.duration)};worldSoundStopObject(o.id);worldEditChanged();worldSounds.say='Sound file saved in this browser.';
  }catch(e){worldSounds.say='Could not import sound: '+e.message;}
  finally{worldSounds.uploads--;mapUiSync();if(worldMotionEditor.id===o.id)worldEditRender();}
}
function worldSoundAllowed(preview=false){return snd.on&&snd.ctx&&!document.hidden&&!liveOn()&&!usbViewOn()&&(preview?worldMotionEditor.id!=null:running&&!wedit.on);}
function worldSoundState(o,preview=false){let s=worldSounds.states.get(o.id);if(!s||s.o!==o||s.preview!==preview){worldSoundStopObject(o.id);s={o,preview,moving:false,dir:0,blocked:false,elapsed:0};worldSounds.states.set(o.id,s);}return s;}
function worldSoundEvent(o,key,preview){if(worldSoundAllowed(preview)&&o.animation?.sounds?.[key]?.source!=='off'&&o.animation?.sounds?.[key]){if(worldSounds.queue.length<128)worldSounds.queue.push({o,key,preview,at:snd.ctx.currentTime});}}
function worldSoundObserve(o,from,to,dt=0,blocked=false,preview=false){
  const s=worldSoundState(o,preview),dir=Math.sign(to-from),travel=Math.abs(to-from)>1e-12;
  if(travel){if(!s.moving||dir!==s.dir){if(s.moving)worldSoundEvent(o,'stop',preview);worldSoundEvent(o,'start',preview);s.elapsed=0;}s.elapsed+=dt;}
  if(blocked&&!s.blocked)worldSoundEvent(o,'blocked',preview);
  const ended=travel&&(to===0||to===1);if((s.moving||travel)&&(!travel||ended||blocked))worldSoundEvent(o,'stop',preview);
  if(ended)worldSoundEvent(o,to===1?'opened':'closed',preview);
  s.moving=travel&&!ended&&!blocked;s.dir=dir||s.dir;s.blocked=blocked;
}
function worldSoundStopObject(id){
  worldSounds.queue=worldSounds.queue.filter(e=>e.o.id!==id);for(const v of [...worldSounds.voices])if(v.owner===id)v.stop();for(const [key,p]of worldSounds.pending)if(p.owner===id)worldSounds.pending.delete(key);worldSounds.states.delete(id);
}
function worldSoundSilence(){worldSounds.epoch++;for(const v of [...worldSounds.voices])v.stop();worldSounds.pending.clear();worldSounds.queue=[];worldSounds.previewUntil=0;}
function worldSoundReset(){worldSoundSilence();worldSounds.states.clear();}
function worldSoundOutput(o){const p=o.motionPose?add(o.motionPose.origin,m3v(o.motionPose.R,[0,0,o.size[2]*o.scale/2])):o.pos,{pan,d}=sndPan(p);return {pan,near:1/(1+Math.max(0,d-1)*.35)};}
function worldSoundMakeVoice(o,key,cue,buffer,loop,preview,elapsed){
  const ctx=snd.ctx,rate=preview?1:sndRate(),{pan,near}=worldSoundOutput(o),out=ctx.createStereoPanner(),g=ctx.createGain(),nodes=[g,out],sources=[],rateParams=[];
  out.pan.value=pan;g.gain.value=0;g.connect(out);out.connect(preview?snd.previewMaster:snd.master);
  const addSource=s=>{sources.push(s);nodes.push(s);return s;};
  let fileSource=null;
  if(buffer){fileSource=addSource(ctx.createBufferSource());fileSource.buffer=buffer;fileSource.loop=loop;fileSource.playbackRate.value=cue.pitch*rate;fileSource.connect(g);fileSource.start(0,loop?(elapsed*cue.pitch)%buffer.duration:0);}
  else{
    const specs={motor:{f:150,type:'square',noise:.25,filter:1800},clunk:{f:180,type:'sine',noise:.8,filter:850},click:{f:2800,type:'sine',noise:1,filter:3200},beep:{f:720,type:'sine',noise:0,filter:720},whoosh:{f:90,type:'sine',noise:1,filter:1200}},p=specs[cue.preset];
    const tone=addSource(ctx.createOscillator()),tg=ctx.createGain();nodes.push(tg);tone.type=p.type;tone.frequency.value=p.f*cue.pitch*rate;tg.gain.value=p.noise===1?.05:.55;tone.connect(tg);tg.connect(g);tone.start();if(loop)rateParams.push([tone.frequency,p.f*cue.pitch]);
    if(!loop&&cue.preset==='clunk')tone.frequency.exponentialRampToValueAtTime(60*cue.pitch*rate,ctx.currentTime+cue.duration/rate);
    if(p.noise){const ns=addSource(ctx.createBufferSource()),f=ctx.createBiquadFilter(),ng=ctx.createGain();nodes.push(f,ng);ns.buffer=snd.noise;ns.loop=true;ns.playbackRate.value=rate;f.type=cue.preset==='whoosh'?'lowpass':'bandpass';f.frequency.value=p.filter*cue.pitch*rate;f.Q.value=cue.preset==='motor'?3:1;ng.gain.value=p.noise*.35;ns.connect(f);f.connect(ng);ng.connect(g);ns.start();if(loop)rateParams.push([ns.playbackRate,1],[f.frequency,p.filter*cue.pitch]);}
  }
  const v={owner:o.id,key,cue:{...cue},o,out,g,sources,nodes,loop,preview,fileSource,rateParams,ended:false,created:ctx.currentTime,stop(){if(v.ended)return;v.ended=true;worldSounds.voices.delete(v);if(worldSounds.loops.get(o.id)===v)worldSounds.loops.delete(o.id);const at=ctx.currentTime;g.gain.cancelScheduledValues(at);g.gain.setTargetAtTime(0,at,.008);for(const s of sources)try{s.stop(at+.04);}catch{}setTimeout(()=>nodes.forEach(n=>{try{n.disconnect();}catch{}}),60);}};
  worldSounds.voices.add(v);if(loop)worldSounds.loops.set(o.id,v);
  const level=cue.volume*.3*near,at=ctx.currentTime,duration=buffer?Math.min(buffer.duration,cue.duration)/(cue.pitch*rate):cue.duration/rate;
  if(loop)g.gain.setTargetAtTime(level,at,.015);
  else{
    const attack=Math.min(.005,duration/4);g.gain.setValueAtTime(.0001,at);g.gain.exponentialRampToValueAtTime(Math.max(.0001,level),at+attack);
    // Uploaded clips retain their natural envelope; only the cut edges get a short fade.
    if(buffer)g.gain.setValueAtTime(Math.max(.0001,level),at+Math.max(attack,duration-Math.min(.015,duration/4)));
    g.gain.exponentialRampToValueAtTime(.0001,at+Math.max(.006,duration));for(const s of sources)try{s.stop(at+duration+.04);}catch{}sources[0].onended=()=>v.stop();
  }
  return v;
}
function worldSoundPlay(o,key,{loop=false,preview=false,elapsed=0,limit=Infinity}={}){
  const cue=o.animation?.sounds?.[key];if(!cue||cue.source==='off'||cue.volume===0||!worldSoundAllowed(preview))return;
  const epoch=worldSounds.epoch,spec=JSON.stringify(cue),token={owner:o.id,preview,at:snd.ctx.currentTime,serial:++worldSounds.serial},pendingKey=o.id+':'+key+':'+preview;worldSounds.pending.set(pendingKey,token);
  const go=buffer=>{
    if(worldSounds.pending.get(pendingKey)!==token)return;worldSounds.pending.delete(pendingKey);
    if(epoch!==worldSounds.epoch||worldObj(o.id)!==o||!worldSoundAllowed(preview)||(!loop&&snd.ctx.currentTime-token.at>.5)||JSON.stringify(o.animation?.sounds?.[key])!==spec)return;
    const st=worldSounds.states.get(o.id);if(loop&&!st?.moving)return;
    return worldSoundMakeVoice(o,key,{...cue,duration:Math.min(cue.duration,limit*(buffer?cue.pitch:1))},buffer,loop,preview,st?.elapsed??elapsed);
  };
  if(cue.source==='file'){worldSoundBuffer(cue.fileId).then(go).catch(e=>{if(worldSounds.pending.get(pendingKey)===token){worldSounds.pending.delete(pendingKey);worldSounds.failed.add(cue.fileId);worldSounds.say=e.message;const n=$('#worldSoundSay');if(n)n.textContent=e.message;}});}
  else return go(null);
}
function worldSoundAudition(o,key){
  if(!snd.on)setSound(true);else sndStart();worldSoundStopObject(o.id);worldSounds.previewUntil=snd.ctx.currentTime+Math.min(5,o.animation.sounds[key].duration)+.1;
  // Auditions are capped at five seconds; they never advance world or drone physics.
  return worldSoundPlay(o,key,{preview:true,limit:5});
}
function worldSoundTick(live){
  const ctx=snd.ctx;if(!ctx)return;const previewAllowed=snd.on&&!document.hidden&&worldMotionEditor.id!=null&&!liveOn()&&!usbViewOn();
  for(const [key,p]of worldSounds.pending)if(p.preview?!previewAllowed:!live)worldSounds.pending.delete(key);
  if(!live)worldSounds.queue=worldSounds.queue.filter(e=>e.preview&&previewAllowed);
  for(const v of [...worldSounds.voices])if(worldObj(v.owner)!==v.o||(!v.preview&&!live)||(v.preview&&!previewAllowed))v.stop();
  const events=worldSounds.queue.splice(0);for(const e of events)if(ctx.currentTime-e.at<.5)worldSoundPlay(e.o,e.key,{preview:e.preview});
  for(const [id,s]of worldSounds.states){const allowed=s.preview?previewAllowed&&worldMotionEditor.playing:live,loop=worldSounds.loops.get(id),cue=s.o.animation?.sounds?.moving;
    if(!s.moving||!allowed||!cue||cue.source==='off'||cue.source==='file'&&worldSounds.failed.has(cue.fileId)){if(loop)loop.stop();continue;}
    if(loop&&JSON.stringify(loop.cue)!==JSON.stringify(cue))loop.stop();
    if(!worldSounds.loops.has(id)&&!worldSounds.pending.has(id+':moving:'+s.preview))worldSoundPlay(s.o,'moving',{loop:true,preview:s.preview,elapsed:s.elapsed});
  }
  for(const v of worldSounds.voices)if(v.loop){const {pan,near}=worldSoundOutput(v.o);v.out.pan.setTargetAtTime(pan,ctx.currentTime,.03);v.g.gain.setTargetAtTime(v.cue.volume*.3*near,ctx.currentTime,.03);const rate=v.preview?1:sndRate();if(v.fileSource)v.fileSource.playbackRate.setTargetAtTime(v.cue.pitch*rate,ctx.currentTime,.03);for(const [param,mult]of v.rateParams)param.setTargetAtTime(mult*rate,ctx.currentTime,.03);}
  const previewActive=previewAllowed&&(worldMotionEditor.playing||ctx.currentTime<(worldSounds.previewUntil||0)||[...worldSounds.voices].some(v=>v.preview));snd.previewMaster.gain.setTargetAtTime(previewActive?.9:0,ctx.currentTime,.02);
}
function worldSoundCueFields(o,key){
  const a=o.animation,c=a.sounds?.[key]||worldSoundDefault(key),changed=()=>{a.sounds ||= {};a.sounds[key]=c;worldSoundStopObject(o.id);worldEditChanged();},choice=(id,label,value,options,set)=>{const n=UI.select({id,'aria-label':label},...options.map(([v,t])=>el('option',{value:v,text:t})));n.value=value;n.onchange=()=>{if(set(n.value)===false){n.value=c.source;return;}changed();worldEditRender();};return UI.field({label},n);},prefix='worldSound-'+key;
  const file=UI.input({id:prefix+'-file',type:'file',accept:'audio/*,.wav,.mp3,.ogg,.m4a,.aac,.flac',hidden:true});file.onchange=()=>{const f=file.files[0];file.value='';if(f)worldSoundImport(o,key,f);};
  const n=(prop,label,min,max,step,u='')=>numField(prefix+'-'+prop,{label,min,max,step,u,hard:true,dp:2},()=>c[prop],v=>{c[prop]=v;changed();}).node;
  const assets=[...worldSounds.assets.values()].map(x=>[x.id,x.name]);if(c.fileId&&!worldSounds.assets.has(c.fileId))assets.unshift([c.fileId,(c.name||'Audio file')+' (missing)']);
  const nodes=[choice(prefix+'-source','Sound source',c.source,[['off','No sound'],['generated','Generated'],['file','Audio file']],v=>{c.source=v;if(v==='file'&&!c.fileId){const first=worldSounds.assets.values().next().value;if(first){c.fileId=first.id;c.name=first.name;c.duration=first.duration;}else{c.source='off';file.click();return false;}}})];
  if(c.source==='generated')nodes.push(choice(prefix+'-preset','Preset',c.preset,Object.entries(WORLD_SOUND_PRESETS),v=>c.preset=v));
  nodes.push(el('div',{class:'hrow world-sound-actions'},UI.button({class:'btn btn-sm',id:prefix+'-upload',disabled:worldSounds.uploads>0||undefined,onclick:()=>file.click()},'Upload sound…'),file));
  if(c.source==='file')nodes.push(choice(prefix+'-asset','Audio file',c.fileId,assets,v=>{c.fileId=v;c.name=worldSounds.assets.get(v)?.name||'Audio file';c.duration=worldSounds.assets.get(v)?.duration||c.duration;}));
  if(c.source!=='off')nodes.push(n('volume','Volume',0,1,.05),n('pitch',c.source==='file'?'Pitch / playback rate':'Pitch',.25,4,.05,'×'),...(key==='moving'?[]:[n('duration','Sound length',.02,c.source==='file'?Math.max(.02,worldSounds.assets.get(c.fileId)?.duration||c.duration):5,.02,'s')]),UI.button({class:'btn btn-sm',id:prefix+'-preview',onclick:()=>worldSoundAudition(o,key)},'Listen'));
  return UI.details({title:WORLD_SOUND_EVENTS[key],open:worldMotionEditor.soundCues?.has(key),class:'world-sound-cue',id:prefix},...nodes);
}
function worldSoundFields(o){return UI.details({title:'Sound cues',open:!!worldMotionEditor.soundsOpen,'data-owner':o.id,class:'world-sound-fields'},el('p',{class:'hint',text:'Generate a motor, clunk, click, beep or whoosh, or upload audio. Moving sounds loop until motion stops. Sounds follow the object and the speaker’s mute setting.'}),...Object.keys(WORLD_SOUND_EVENTS).map(key=>worldSoundCueFields(o,key)),el('p',{id:'worldSoundSay',class:'hint',role:'status',text:worldSounds.say}));}

// Read the native disclosure state before a properties rebuild; toggle events may still be queued.
function worldSoundCapture(){const box=$('.world-sound-fields');if(!box||box.dataset.owner!==worldMotionEditor.id)return;worldMotionEditor.soundsOpen=box.open;worldMotionEditor.soundCues=new Set(Object.keys(WORLD_SOUND_EVENTS).filter(k=>$('#worldSound-'+k)?.open));}
