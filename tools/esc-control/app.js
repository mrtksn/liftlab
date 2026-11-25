'use strict';
const B=SignalBytes,Q=SignalSequence,M=SignalModel,C=SignalConfig,$=id=>document.getElementById(id),colors=C.board.colors;
const esc=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const num=(n,d=2)=>Number.isFinite(n)?n.toLocaleString(undefined,{maximumFractionDigits:d}):'—';
let presets,selectedId,connected=false,connecting=false,demo=false,armed=false,busy=false,watching=false,watchTimer=null;
let captures=[],currentCapture=null,incoming=null,sequence=0,runEpoch=0,editorDraft=null;
let analogCaptures=[],currentAnalog=null,incomingAnalog=null,analogWatching=false,analogTimer=null;
const states=new Map(),logs=[],updates=new Map(),tickets=new Map(),owners=new Map(),runningSequences=new Map(),activeOutputs=new Set(),payloadErrors=new Map(),editorByteErrors=new Map();let enabling=null;let updateTimer=null,toastTimer;
try{const saved=localStorage.getItem('signal-workbench.presets.v1');presets=saved?JSON.parse(saved):M.defaults();if(!Array.isArray(presets)||!presets.length)throw Error('No presets');presets=presets.map(M.migratePreset);presets.forEach(M.validatePreset);selectedId=localStorage.getItem('signal-workbench.selected')||presets[0].id;}catch{presets=M.defaults();selectedId=presets[0].id;}
try{for(const {id,marker}of C.defaults.legacy.seedPresets){if(!localStorage.getItem(marker)){if(!presets.some(p=>p.id===id))presets.push(M.defaults().find(p=>p.id===id));localStorage.setItem(marker,'1');}}}catch{}
const preset=()=>presets.find(p=>p.id===selectedId)||presets[0];
const ready=()=>connected||demo;
function toast(message){$('toast').textContent=message;$('toast').hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').hidden=true,5000);}
function notice(message,type=''){$('notice').textContent=message;$('notice').className='notice '+type;}
function log(level,message){logs.push({date:new Date().toISOString(),level,message});if(logs.length>500)logs.shift();$('logCount').textContent=logs.length;renderLog();}
function renderLog(){const search=$('logSearch').value.toLowerCase(),level=$('logLevel').value;const found=logs.filter(l=>(level==='all'||l.level===level)&&l.message.toLowerCase().includes(search));$('log').innerHTML=found.length?found.map(l=>`<div class="log-row ${l.level}"><time>${esc(new Date(l.date).toLocaleTimeString())}</time><span class="log-level">${esc(l.level)}</span><span class="log-message">${esc(l.message)}</span></div>`).join(''):'<div class="log-empty">No matching activity yet.</div>';$('log').scrollTop=$('log').scrollHeight;}
function persist(){try{localStorage.setItem('signal-workbench.presets.v1',JSON.stringify(presets));localStorage.setItem('signal-workbench.selected',selectedId);}catch{toast('Browser storage is unavailable. Export your presets to keep a copy.');}}
function download(name,data,type='application/json'){const url=URL.createObjectURL(new Blob([data],{type})),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
const transport=new SignalTransport(handleDevice,error=>{connected=false;connecting=false;armed=false;clearRuntime();stopWatch();incoming=null;busy=false;notice(error.message+' Outputs return to their configured idle state when the device lease expires.','error');log('error',error.message);drawState();});
function drawState(){
 $('connectionState').textContent=demo?'Demo · simulated':connected?'Connected':connecting?'Connecting…':'Disconnected';$('connectionState').className='badge '+(demo?'demo':connected?'connected':'');
 $('connect').disabled=connecting||connected;$('connect').hidden=connected;$('disconnect').hidden=!connected&&!connecting;$('demoButton').textContent=demo?'Exit demo':'Try demo';$('demoButton').disabled=connecting;
 $('outputState').textContent=activeOutputs.size?activeOutputs.size+' output'+(activeOutputs.size===1?'':'s')+' active':ready()?'Ready':'Offline';$('outputState').className='badge '+(activeOutputs.size?'armed':'');
 const inputs=preset().channels.some(c=>c.direction==='in'||($('captureSources').value==='all'&&c.direction==='out'));$('capture').disabled=!ready()||busy||!inputs;$('watch').disabled=!ready()||!inputs||(!watching&&busy);$('watch').textContent=watching?'■ Stop live view':'Start live view';
 $('capture').textContent=busy?'Capturing…':'● Capture once';
 const analogs=preset().channels.some(c=>c.direction==='analog');$('analogCapture').disabled=!ready()||busy||!analogs;$('analogWatch').disabled=!ready()||!analogs||(!analogWatching&&busy);$('analogWatch').textContent=analogWatching?'Stop live readings':'Start live readings';
 for(const c of preset().controls){const button=$('button-'+c.id),state=states.get(c.id);if(!button)continue;const on=!!state?.on,hold=['hold','holdEdges'].includes(c.mode);button.disabled=!ready()||(!on&&payloadErrors.has(c.id));button.classList.toggle('active',on);button.setAttribute('aria-pressed',String(on));button.dataset.hold=String(hold);
  const toolbar=c.placement==='toolbar';
  const fallback=c.mode==='sequence'?(on?'Cancel sequence':'Run sequence'):c.mode==='stop'?'■ '+c.name:c.mode==='once'?'Send once':hold?(on?'RELEASE · Stop':'HOLD · Run'):on?'ON · Turn off':'OFF · Turn on';
  button.querySelector('strong').textContent=(on?c.labelOn:c.labelOff)||(toolbar?(on&&c.mode!=='once'&&c.mode!=='stop'?'Cancel / stop · '+c.name:c.name):fallback);
  button.querySelector('small').textContent=c.mode==='sequence'&&on&&state.progress?state.progress:c.name;
  button.querySelector('.power-icon').textContent=c.mode==='sequence'?'▸':c.mode==='stop'?'■':c.mode==='once'?'↗':hold?'◉':'⏻';
  const color=on?c.activeColor:c.color;button.style.background=color||M.PALETTE.mint;button.style.borderColor=color||M.PALETTE.mint;button.style.color=buttonTextColor(color||M.PALETTE.mint);

 }
}
function clearRuntime(){runEpoch++;for(const run of runningSequences.values())run.controller.abort();tickets.clear();owners.clear();activeOutputs.clear();enabling=null;updates.clear();clearTimeout(updateTimer);updateTimer=null;states.clear();drawState();}
async function stopAll(reason='Stopped all outputs'){
 clearRuntime();armed=false;drawState();
 if(connected){try{await transport.stop();}catch(e){log('error',e.message);}}
 if(ready())log('output',reason);
}
function stopWatch(){analogWatching=false;clearTimeout(analogTimer);analogTimer=null;watching=false;clearTimeout(watchTimer);watchTimer=null;drawState();}
async function configure(){
 if(connected){const reply=await transport.command('configure',{channels:preset().channels});if(reply.armed)throw Error('Device failed to disarm during configuration.');}
 armed=false;clearRuntime();drawState();
}
async function changePreset(id){stopWatch();await stopAll('Preset changed; all outputs stopped');selectedId=id;currentCapture=null;incoming=null;currentAnalog=null;incomingAnalog=null;persist();render();if(connected)try{await configure();notice('Preset configured. Use any preset control to send a signal, or capture inputs.');}catch(e){notice(e.message,'error');}}
function unit(kind){return kind==='pwm'?'µs':kind==='duty'?'%':kind==='dshot'?'value':'level';}
function summary(s){if(['pwm','duty'].includes(s.kind))return `${num(s.hz)} Hz · ${num(s.value)} ${unit(s.kind)}`;if(s.kind==='uart')return `${s.baud} baud · 8N1 · ${s.data}`;if(s.kind==='dshot')return `DShot${s.rate} · value ${s.value} · checksum`;if(s.kind==='pulse')return `${s.pulses?.length||0} pulse segments · 1 µs steps`;if(s.kind==='highz')return 'High impedance';return `Digital ${s.value?'HIGH':'LOW'} · ${s.duration||1000} µs pulse when sent once`;}
function buttonTextColor(hex){const rgb=hex.match(/\w\w/g)?.map(v=>parseInt(v,16))||[180,243,207];return rgb[0]*.299+rgb[1]*.587+rgb[2]*.114>150?'#13251C':'#FFFFFF';}
function byteEditor(s,key){
 const format=s.encoding||'hex',endian=s.endian||'little';let value='',error='';try{value=B.decode(s.data,format,endian);}catch(e){error=e.message;}
 return `<div class="byte-editor" data-uart-key="${esc(key)}"><div class="byte-options"><label>Display / edit as<select data-byte-format>${Object.entries(B.FORMATS).map(([v,n])=>`<option value="${v}" ${v===format?'selected':''}>${n}</option>`).join('')}</select></label><label data-endian-label ${!B.TYPES[format]||B.TYPES[format][0]===1?'hidden':''}>Byte order<select data-byte-endian><option value="little" ${endian==='little'?'selected':''}>Little endian</option><option value="big" ${endian==='big'?'selected':''}>Big endian</option></select></label></div><label>Payload<textarea data-byte-value rows="2" spellcheck="false" aria-label="UART payload">${esc(value)}</textarea></label><div class="byte-status ${error?'invalid':''}" data-byte-status>${esc(error||B.fromHex(s.data).length+' / 12 bytes · changes apply when you leave the field')}</div><div class="byte-preview"><span>WIRE BYTES</span><code data-byte-hex>${esc(B.decode(s.data))}</code></div><p class="help">Text accepts \\n, \\r, \\t and \\xNN escapes. Numbers are packed as binary values; use Text to send digits such as 123.</p></div>`;
}
function bindByteEditors(root,getSignal,onCommit){
 root.querySelectorAll('[data-uart-key]').forEach(widget=>{
  const key=widget.dataset.uartKey,s=getSignal(key),format=widget.querySelector('[data-byte-format]'),endian=widget.querySelector('[data-byte-endian]'),value=widget.querySelector('[data-byte-value]'),status=widget.querySelector('[data-byte-status]'),hex=widget.querySelector('[data-byte-hex]');
  const message=(text,error=false)=>{status.textContent=text;status.classList.toggle('invalid',error);};
  const preview=()=>{try{const data=B.encode(value.value,format.value,endian.value);hex.textContent=B.decode(data);message(B.fromHex(data).length+' / 12 bytes · leave the field to apply');return data;}catch(e){message(e.message,true);return null;}};
  const commit=()=>{const data=preview();if(data===null){hex.textContent=B.decode(s.data);const error=status.textContent;message(error+' · not applied; previous bytes retained',true);onCommit(key,error,false);return false;}const changed=data!==s.data;s.data=data;s.encoding=format.value;s.endian=endian.value;value.dataset.dirty='';onCommit(key,null,changed);message(B.fromHex(data).length+' / 12 bytes · '+(changed?'updated':'ready'));return true;};
  const changeView=()=>{
   if(value.dataset.dirty){const oldFormat=s.encoding||'hex',oldEndian=s.endian||'little',nextFormat=format.value,nextEndian=endian.value;format.value=oldFormat;endian.value=oldEndian;if(!commit())return;format.value=nextFormat;endian.value=nextEndian;}
   s.encoding=format.value;s.endian=endian.value;widget.querySelector('[data-endian-label]').hidden=!B.TYPES[format.value]||B.TYPES[format.value][0]===1;
   try{value.value=B.decode(s.data,s.encoding,s.endian);message(B.fromHex(s.data).length+' / 12 bytes · ready');onCommit(key,null,false);}catch(e){value.value='';message(e.message,true);onCommit(key,e.message,false);}
  };
  value.oninput=()=>{value.dataset.dirty='true';preview();};value.onchange=commit;value.onblur=()=>{if(value.dataset.dirty)commit();};widget.commitPayload=commit;format.onchange=endian.onchange=changeView;
 });
}
function renderControl(c,toolbar){
 const ch=preset().channels[c.channel],slider=c.source==='slider'&&c.mode!=='stop';
 let fields=slider?`<div class="value-display"><input type="number" id="value-${esc(c.id)}" min="${c.min}" max="${c.max}" step="${c.step}" value="${c.value}" aria-label="${esc(c.name)} value"><span>${c.mode==='sequence'?'value':unit(c.on.kind)}</span></div><input class="control-slider" type="range" id="slider-${esc(c.id)}" min="${c.min}" max="${c.max}" step="${c.step}" value="${c.value}" aria-label="${esc(c.name)} slider"><div class="slider-bounds"><span>${num(c.min)}</span><span>${num(c.max)}</span></div>`:'';
 if(c.mode==='sequence')fields+=`<ol class="sequence-summary">${c.steps.map(step=>`<li>${step.action==='wait'?'Wait '+num(step.ms)+' ms':step.action==='stop'?'Stop all outputs':step.action==='idle'?'CH'+(step.channel+1)+' → idle':'CH'+(step.channel+1)+' · '+esc(summary(step.signal))+' · '+step.mode}</li>`).join('')}</ol>`;
 else if(c.mode!=='stop'&&c.on.kind==='uart')fields+=byteEditor(c.on,c.id);
 else if(!slider&&c.mode!=='stop')fields+=`<div class="signal-summary">${esc(summary(c.on))}</div>`;
 const badge=c.mode==='stop'?'All outputs':c.mode==='sequence'?M.controlChannels(c).map(i=>'CH'+(i+1)).join(' / '):`CH${c.channel+1} · GPIO${ch?.pin}`;
 const button=`<button id="button-${esc(c.id)}" class="control-button" aria-pressed="false" disabled><span class="power-icon" aria-hidden="true">⏻</span><span><strong>${esc(c.name)}</strong><small>${esc(c.name)}</small></span></button>`;
 return toolbar?`<div class="toolbar-control">${button}${fields?`<details class="toolbar-settings"><summary>Values / steps</summary><div>${fields}</div></details>`:''}</div>`:`<article class="control-card"><div class="control-heading"><h3>${esc(c.name)}</h3><span class="control-channel" style="--channel-color:${colors[c.channel]||colors[0]}">${esc(badge)}</span></div><p class="control-type">${esc(M.MODES[c.mode])}</p>${fields}${button}<p class="control-hint">${c.mode==='stop'?'Applies the preset’s channel idle signals and stops pending sequences.':c.mode==='sequence'?'Ordered signals and host-timed waits. Press again to cancel and idle its channels.':slider?'The slider and numeric input update repeating output live.':'Values are editable; sending uses the current payload.'}</p></article>`;
}

function render(){
 const p=preset();const boardContext={board:p.wiring.board};$('boardName').textContent=p.wiring.board.name;$('boardSummary').textContent=p.wiring.board.appCopy?.summary||'';for(const [id,key]of [['generatorFootnote','generator'],['analogVoltageNotice','analog'],['analogFootnote','analogFootnote'],['instrumentFooter','footer']])$(id).textContent=SignalWiring.template(p.wiring.board.appCopy?.[key]||'',boardContext);$('presetTitle').textContent=p.name;$('presetDescription').textContent=p.description||'';
 $('presets').innerHTML=presets.map(s=>`<button class="preset-item ${s.id===p.id?'active':''}" data-preset="${esc(s.id)}">${esc(s.name)}<small>${s.channels.filter(c=>c.direction==='out').length} OUT / ${s.channels.filter(c=>c.direction==='in').length} IN${s.channels.some(c=>c.direction==='analog')?' / '+s.channels.filter(c=>c.direction==='analog').length+' ADC':''} · ${s.controls.length} controls</small></button>`).join('');
 $('presets').querySelectorAll('button').forEach(b=>b.onclick=()=>changePreset(b.dataset.preset));
 $('channelStrip').innerHTML=p.channels.map((c,i)=>`<div class="channel-chip" style="--channel-color:${colors[i]}"><b>CH${i+1}</b><span>${esc(c.name)} <small>· GPIO${c.pin} · ${c.direction==='in'?'IN':c.direction==='analog'?'ADC':'OUT'}</small></span></div>`).join('');
 $('outputCount').textContent=p.channels.filter(c=>c.direction==='out').length;$('inputCount').textContent=p.channels.filter(c=>c.direction==='in').length;$('analogCount').textContent=p.channels.filter(c=>c.direction==='analog').length;
 $('toolbarControls').innerHTML=p.controls.filter(c=>c.placement==='toolbar').map(c=>renderControl(c,true)).join('');
 $('controls').innerHTML=p.controls.filter(c=>c.placement!=='toolbar').map(c=>renderControl(c,false)).join('')||`<div class="empty-card"><h3>${p.controls.length?'Your controls live in the toolbar.':'This preset listens to inputs.'}</h3><p>Add controls in Edit preset, or open an input instrument.</p><button id="openAnalyzer" class="primary">${p.channels.some(c=>c.direction==='analog')?'Open analog inputs':'Open logic analyzer'}</button></div>`;
 for(const c of p.controls){
  const button=$('button-'+c.id);
  if(['hold','holdEdges'].includes(c.mode)){
   const start=e=>{e.preventDefault();if(e.type==='keydown'&&e.repeat)return;activate(c).catch(showError);if(e.pointerId!==undefined)try{button.setPointerCapture(e.pointerId);}catch{}};
   const release=()=>{if(states.get(c.id)?.on)deactivate(c).catch(showError);};
   button.addEventListener('pointerdown',start);for(const type of ['pointerup','pointercancel','lostpointercapture'])button.addEventListener(type,release);
   button.addEventListener('keydown',e=>{if([' ','Enter'].includes(e.key))start(e);});button.addEventListener('keyup',e=>{if([' ','Enter'].includes(e.key)){e.preventDefault();release();}});
  }else button.onclick=()=>activate(c).catch(showError);
  if(c.source==='slider'&&c.mode!=='stop'){
   const input=$('value-'+c.id),slider=$('slider-'+c.id);
   slider.oninput=()=>changeValue(c,slider.value,input,slider);
   input.onchange=()=>{if(!input.checkValidity()){input.reportValidity();return;}changeValue(c,input.value,input,slider);};
  }
 }
 bindByteEditors($('generatorPanel'),key=>preset().controls.find(c=>c.id===key).on,(key,error,changed)=>{const c=preset().controls.find(c=>c.id===key);if(error){payloadErrors.set(key,error);drawState();return;}payloadErrors.delete(key);drawState();persist();if(changed&&states.get(key)?.on&&['continuous','hold'].includes(c.mode))scheduleSignal(c);});
 if($('openAnalyzer'))$('openAnalyzer').onclick=()=>selectTab(p.channels.some(c=>c.direction==='analog')?'analog':'analyzer');
 const options=p.channels.map((c,i)=>c.direction==='in'?`<option value="${i}">CH${i+1} · ${esc(c.name)}</option>`:'').join('');
 $('trigger').innerHTML='<option value="-1">Immediate</option>'+options;
 for(const id of ['roleA','roleB','roleC'])$(id).innerHTML=(id==='roleC'?'<option value="-1">Not used</option>':'')+options;
 const inputs=p.channels.flatMap((c,i)=>c.direction==='in'?[i]:[]);if(inputs.length>1)$('roleB').value=inputs[1];
 renderCapture();renderAnalog();drawState();
}
function showError(e){if(e.message==='Cancelled')return;toast(e.message);log('error',e.message);}
function changeValue(c,value,input,slider){
 const n=Number(value);if(!Number.isFinite(n)||n<c.min||n>c.max)return;
 try{M.validateSignal(M.resolveSignal(c,n));}catch(e){showError(e);return;}
 c.value=n;input.value=slider.value=n;persist();
 if(states.get(c.id)?.on&&['continuous','hold'].includes(c.mode))scheduleSignal(c);
}
function scheduleSignal(c){updates.set(c.channel,c);if(updateTimer)return;updateTimer=setTimeout(async()=>{updateTimer=null;const batch=[...updates.values()];updates.clear();for(const control of batch)if(states.get(control.id)?.on&&armed)try{await sendSignal(control,M.resolveSignal(control,control.value),'continuous');}catch(e){showError(e);}},40);}
function makeTicket(c,independent=false){const ticket={epoch:runEpoch,id:c.id,independent};tickets.set(c.id,ticket);return ticket;}
const validTicket=t=>t&&!t.signal?.aborted&&t.epoch===runEpoch&&(t.independent||tickets.get(t.id)===t);
async function ensureEnabled(ticket){
 if(!ready())throw Error('Connect the board first.');if(!validTicket(ticket))throw Error('Cancelled');
 if(!armed){
  if(!enabling){const epoch=runEpoch;const promise=(async()=>{if(connected)await transport.command('enable',{},8000,()=>epoch===runEpoch);if(epoch!==runEpoch)throw Error('Cancelled');armed=true;drawState();})();enabling=promise;promise.finally(()=>{if(enabling===promise)enabling=null;}).catch(()=>{});}
  await enabling;
 }
 if(!validTicket(ticket))throw Error('Cancelled');
}
async function sendSignal(c,signal,mode,ch=c.channel,ticket=tickets.get(c.id),repeatMs=c.repeatMs){
 M.validateSignal(signal);await ensureEnabled(ticket);const accept=()=>validTicket(ticket)&&owners.get(ch)===c.id;
 if(!accept())throw Error('Cancelled');
 if(!demo){if(signal.kind==='idle')await transport.command('idle',{ch},8000,accept);else await transport.command('signal',{ch,signal,mode,repeatMs},8000,accept);}
 if(signal.kind==='idle'||signal.kind==='highz'||mode==='once')activeOutputs.delete(ch);else activeOutputs.add(ch);drawState();
}
async function claimChannels(c,channels,ticket){
 for(const [id,run] of [...runningSequences])if(id!==c.id&&run.channels.some(ch=>channels.includes(ch))){run.controller.abort();await run.finished;}
 if(!validTicket(ticket))throw Error('Cancelled');
 for(const ch of channels){for(const other of preset().controls)if(other.id!==c.id&&M.controlChannels(other).includes(ch)){states.delete(other.id);tickets.delete(other.id);}updates.delete(ch);owners.set(ch,c.id);}
 drawState();
}
async function idleOwned(c,epoch){
 if(epoch!==runEpoch)return;
 for(const [ch,id] of [...owners])if(id===c.id){owners.delete(ch);activeOutputs.delete(ch);if(connected&&armed)try{await transport.command('idle',{ch});}catch(e){showError(e);}}
 drawState();
}
async function runSequence(c){
 const old=runningSequences.get(c.id);if(old){old.controller.abort();await old.finished;return;}
 const ticket=makeTicket(c),controller=new AbortController(),channels=M.controlChannels(c);ticket.signal=controller.signal;const run={controller,channels,finished:null};runningSequences.set(c.id,run);states.set(c.id,{on:true,progress:'Starting…'});drawState();
 run.finished=(async()=>{
  try{
   await claimChannels(c,channels,ticket);Q.check(controller.signal);
   await Q.execute(c.steps,{
    send:async(ch,s,mode,repeatMs)=>{Q.check(controller.signal);await sendSignal(c,s,mode,ch,ticket,repeatMs);},
    idle:async ch=>{Q.check(controller.signal);await sendSignal(c,{kind:'idle'},'once',ch,ticket);},
    stop:()=>stopAll(c.name+': stop step'),resolveSignal:step=>M.resolveStep(c,step),
    onStep:(i,n)=>{if(states.has(c.id)){states.get(c.id).progress=`Step ${i+1} / ${n}`;drawState();}}
   },controller.signal);
   if(validTicket(ticket))log('output',c.name+': sequence complete'+(demo?' [simulated]':''));
  }catch(e){controller.abort();if(e.message!=='Cancelled')showError(e);log('output',c.name+': sequence cancelled');await idleOwned(c,ticket.epoch);}
  finally{if(runningSequences.get(c.id)===run){runningSequences.delete(c.id);states.delete(c.id);if(tickets.get(c.id)===ticket)tickets.delete(c.id);}drawState();}
 })();
 await run.finished;
}
async function activate(c){
 if(!ready())return;if(c.mode==='stop'){await stopAll(c.name);return;}if(c.mode==='sequence'){await runSequence(c);return;}
 const state=states.get(c.id)||{on:false};
 if(['hold','holdEdges'].includes(c.mode)&&state.on)return;
 if(c.mode!=='once'&&state.on){await deactivate(c);return;}
 const widget=[...$('generatorPanel').querySelectorAll('[data-uart-key]')].find(w=>w.dataset.uartKey===c.id);if(widget&&!widget.commitPayload())throw Error(payloadErrors.get(c.id)||'Check the UART payload.');
 if(payloadErrors.has(c.id))throw Error(payloadErrors.get(c.id));
 const ticket=makeTicket(c,c.mode==='once'),continuous=['continuous','hold'].includes(c.mode);states.set(c.id,{on:c.mode!=='once'});drawState();
 try{await claimChannels(c,[c.channel],ticket);await sendSignal(c,M.resolveSignal(c,c.value),continuous?'continuous':'once',c.channel,ticket);if(!validTicket(ticket))return;log('output',`${c.name}: ${continuous?'started':'sent once'} · ${summary(M.resolveSignal(c,c.value))}${demo?' [simulated]':''}`);}catch(e){if(tickets.get(c.id)===ticket)states.delete(c.id);drawState();throw e;}
}
async function deactivate(c){
 const ticket=makeTicket(c);states.delete(c.id);updates.delete(c.channel);drawState();
 await claimChannels(c,[c.channel],ticket);await sendSignal(c,c.off,'once',c.channel,ticket);log('output',c.name+': off'+(demo?' [simulated]':''));
}
function handleDevice(event){
 if(event.type==='analog'){incomingAnalog={...event,signals:event.channels,presetChannels:M.clone(preset().channels),points:[],demo:false,date:new Date().toISOString()};}
 else if(event.type==='analogPoints'&&incomingAnalog?.id===event.id)incomingAnalog.points.push(...event.points);
 else if(event.type==='analogEnd'&&incomingAnalog?.id===event.id){const cap=incomingAnalog;incomingAnalog=null;if(cap.points.length!==cap.count){showError(Error('Incomplete analog capture.'));return;}finishAnalog(cap);}
 if(['ack','error'].includes(event.type)&&event.armed===false&&armed){armed=false;clearRuntime();}
 if(event.type==='timeout'){armed=false;clearRuntime();notice(event.message,'error');log('error',event.message);}
 else if(event.type==='boot'&&connected){connected=false;armed=false;clearRuntime();stopWatch();notice('Device restarted. Disconnect and reconnect before using outputs.','error');log('error','Device restarted.');transport.close().catch(()=>{});drawState();}
 else if(event.type==='error'&&!event.id){log('error',event.message);}
 else if(event.type==='capture'){incoming={...event,edges:[],channels:M.clone(preset().channels),demo:false,date:new Date().toISOString()};}
 else if(event.type==='edges'&&incoming?.id===event.id)incoming.edges.push(...event.edges);
 else if(event.type==='captureEnd'&&incoming?.id===event.id){const cap=incoming;incoming=null;if(cap.count!==cap.edges.length){showError(Error('Incomplete capture; edge count does not match.'));return;}finishCapture(cap);}
}
$('connect').onclick=async()=>{
 if(connecting)return;demo=false;connecting=true;clearRuntime();drawState();
 try{await transport.close();await transport.open();connected=true;connecting=false;await configure();notice('Board ready. Controls send only their configured signals; connecting does not start an idle waveform.');log('connection','Signal Workbench firmware connected at 921600 baud.');}
 catch(e){connected=false;connecting=false;notice(e.message,'error');log('error',e.message);}drawState();
};
$('disconnect').onclick=async()=>{stopWatch();await stopAll('Disconnected; outputs stopped');connected=false;connecting=false;await transport.close();notice('Disconnected.');drawState();};
$('demoButton').onclick=async()=>{
 stopWatch();if(connected){await stopAll('Entered demo; hardware outputs stopped');connected=false;await transport.close();}demo=!demo;armed=false;clearRuntime();notice(demo?'DEMO · All signals, captures, and output actions are simulated. No GPIO is driven.':'Demo closed. Connect your ESP32 to use hardware.',demo?'demo':'');log('connection',demo?'Entered simulated demo.':'Exited demo.');drawState();
};
window.addEventListener('blur',()=>{if(armed)stopAll('Window lost focus; outputs stopped');stopWatch();});
document.addEventListener('visibilitychange',()=>{if(document.hidden){if(armed)stopAll('Tab hidden; outputs stopped');stopWatch();}});
window.addEventListener('pagehide',()=>{if(connected)transport.stop().catch(()=>{});});
function selectTab(name){document.querySelectorAll('[data-tab]').forEach(b=>b.setAttribute('aria-selected',String(b.dataset.tab===name)));for(const n of ['generator','analyzer','analog','activity'])$(n+'Panel').hidden=n!==name;if(name==='analyzer')drawWaveform();if(name==='analog')drawAnalog();}
document.querySelectorAll('[data-tab]').forEach(b=>b.onclick=()=>selectTab(b.dataset.tab));
const wiring=SignalWiring.mount({getPreset:preset,dialog:$('wiringDialog'),boardHost:$('wiringBoard'),contentHost:$('wiringCircuit'),channelSelect:$('wiringChannel'),modeSelect:$('wiringMode'),settingsHost:$('wiringSettings'),titleHost:$('wiringPresetTitle')});
$('wiringButton').onclick=$('topWiringButton').onclick=()=>wiring.open();
document.querySelectorAll('[data-close]').forEach(b=>b.onclick=()=>$(b.dataset.close).close());
$('logSearch').oninput=$('logLevel').onchange=renderLog;$('clearLog').onclick=()=>{logs.length=0;$('logCount').textContent='0';renderLog();};$('exportLog').onclick=()=>download('signal-workbench-log.json',JSON.stringify(logs,null,2));

async function confirmAction(title,text){$('confirmTitle').textContent=title;$('confirmText').textContent=text;$('confirmDialog').showModal();return new Promise(resolve=>{const done=value=>{$('confirmDialog').close();resolve(value);};$('confirmCancel').onclick=()=>done(false);$('confirmOK').onclick=()=>done(true);$('confirmDialog').oncancel=()=>resolve(false);});}
async function openEditor(fresh=false){stopWatch();await stopAll('Preset editor opened; outputs stopped');editorDraft=fresh?M.newPreset():M.clone(preset());$('editorTitle').textContent=fresh?'Create preset':'Edit preset';$('editName').value=editorDraft.name;$('editDescription').value=editorDraft.description||'';$('editorError').textContent='';renderEditor();$('presetEditor').showModal();}
const wiringEditor=SignalWiringEditor.mount($('wiringEditor'),()=>editorDraft,()=>refreshJson());
$('editWiring').onclick=async()=>{const selection=wiring.selection();$('wiringDialog').close();await openEditor();wiringEditor.select(selection);$('wiringEditorSection').open=true;$('wiringEditorSection').scrollIntoView({block:'start'});};
$('editPreset').onclick=()=>openEditor();$('newPreset').onclick=()=>openEditor(true);
$('duplicatePreset').onclick=async()=>{const copy=M.clone(preset());copy.id=M.uid();copy.name+=' copy';copy.controls.forEach(c=>c.id=M.uid());presets.push(copy);persist();await changePreset(copy.id);toast('Preset duplicated.');};
$('deletePreset').onclick=async()=>{if(presets.length===1){toast('Keep at least one preset. Create another before deleting this one.');return;}if(await confirmAction('Delete preset?',`Delete “${preset().name}” from this browser? Export a copy first if you want to keep it.`)){const id=selectedId;await stopAll('Preset deleted; outputs stopped');presets=presets.filter(p=>p.id!==id);await changePreset(presets[0].id);toast('Preset deleted.');}};
const getPath=(o,path)=>path.split('.').reduce((a,k)=>a[k],o);
function setPath(o,path,value){const keys=path.split('.'),key=keys.pop();keys.reduce((a,k)=>a[k],o)[key]=value;}
function field(label,path,value,type='text',extra=''){return `<label>${label}<input data-field="${path}" type="${type}" value="${esc(value)}" ${extra}></label>`;}
function selectField(label,path,value,options){return `<label>${label}<select data-field="${path}">${options.map(([v,label])=>`<option value="${esc(v)}" ${String(v)===String(value)?'selected':''}>${esc(label)}</option>`).join('')}</select></label>`;}
function signalEditor(s,path,title,idle=false){
 const kinds=idle?['highz','digital','pwm','duty']:M.KINDS;
 const names={pwm:'PWM · pulse width (µs)',duty:'PWM · duty cycle (%)',digital:'Digital level / pulse',pulse:'Raw pulse sequence',uart:'UART · bytes / text / numbers',dshot:'DShot · CRC frame',highz:'High impedance',idle:'Use channel idle signal'};
 let params='';
 if(['pwm','duty'].includes(s.kind))params=field('Frequency (Hz)',path+'.hz',s.hz??C.defaults.signal.hz,'number','min="1" max="1000000"')+field(s.kind==='pwm'?'Width (µs)':'Duty (%)',path+'.value',s.value??C.defaults.signal.value,'number','min="0" step="any"');
 if(s.kind==='digital')params=selectField('Level',path+'.value',s.value??0,[[0,'LOW · 0'],[1,'HIGH · 1']])+(!idle?field('Once pulse duration (µs)',path+'.duration',s.duration??1000,'number','min="1" max="1000000"'):'');
 if(s.kind==='uart')params=field('Baud (8N1)',path+'.baud',s.baud??115200,'number')+`<div class="wide">${byteEditor(s,path)}</div>`;
 if(s.kind==='dshot')params=selectField('Bitrate (kbit/s)',path+'.rate',s.rate??150,[[150,'DShot150'],[300,'DShot300'],[600,'DShot600']])+field('Value · 0 stop / 1–47 commands',path+'.value',s.value??0,'number','min="0" max="2047"')+selectField('Telemetry request bit',path+'.telemetry',s.telemetry??0,[[0,'No'],[1,'Yes · no return decoding']]);
 if(s.kind==='pulse')params=`<label class="wide">[level, microseconds] segments<textarea data-field="${path}.pulses" data-json="true" spellcheck="false">${esc(JSON.stringify(s.pulses||[[1,1000],[0,1000]]))}</textarea></label>`+selectField('Level after once',path+'.idleLevel',s.idleLevel??0,[[0,'LOW'],[1,'HIGH']]);
 return `<fieldset class="signal-editor"><legend>${title}</legend>${selectField('Signal type',path+'.kind',s.kind,kinds.map(k=>[k,names[k]]))}<div class="signal-params">${params}</div></fieldset>`;
}
function renderEditor(){
 const d=editorDraft;editorByteErrors.clear();
 $('channelEditor').innerHTML=d.channels.map((c,i)=>{
  const pins=c.direction==='analog'?M.ADC_PINS:c.direction==='out'?M.OUTPUT_PINS:M.INPUT_PINS;
  let settings=c.direction==='out'?signalEditor(c.idle,`channels.${i}.idle`,'Idle / stop signal',true):c.direction==='analog'?`<div class="analog-channel-fields">${selectField('ADC attenuation',`channels.${i}.attenuation`,c.attenuation??12,Object.entries(M.ADC_RANGES).map(([db,range])=>[db,db+' dB · '+range[0]+'–'+range[1]+' V characterized']))}${field('Scale / divider ratio',`channels.${i}.scale`,c.scale??1,'number','step="any"')}${field('Offset',`channels.${i}.offset`,c.offset??0,'number','step="any"')}${field('Displayed unit',`channels.${i}.unit`,c.unit??'V','text','maxlength="12"')}<p class="help">Displayed = measured volts × scale + offset. Scaling does not protect the pin; use a physical divider. ADC1 is preferred.</p></div>`:'<p class="help">Passive digital probe. No output is driven.</p>';
  return `<div class="editor-channel">${field('CH'+(i+1)+' · name',`channels.${i}.name`,c.name)}${selectField('Direction',`channels.${i}.direction`,c.direction,[['out','Output'],['in','Digital input'],['analog','Analog input · ADC']])}${selectField('GPIO',`channels.${i}.pin`,c.pin,pins.map(p=>[p,'GPIO'+p+(c.direction==='analog'?' · ADC'+(M.ADC1_PINS.includes(p)?'1':'2'):p>=34?' · input only':'')]))}<div class="idle-fields">${settings}</div><button type="button" class="editor-remove" data-remove-channel="${i}" aria-label="Remove channel ${i+1}">×</button></div>`;
 }).join('');
 const outputs=d.channels.flatMap((c,i)=>c.direction==='out'?[[i,`CH${i+1} · ${c.name}`]]:[]);
 $('controlEditor').innerHTML=d.controls.map((c,i)=>{
  const base=`controls.${i}`,stop=c.mode==='stop',sequence=c.mode==='sequence';
  const fields=`<div class="control-fields">${field('Button name',base+'.name',c.name)}${selectField('Button behavior',base+'.mode',c.mode,Object.entries(M.MODES))}${selectField('Placement',base+'.placement',c.placement||'card',[['card','Large card button'],['toolbar','Compact toolbar button']])}${!stop&&!sequence?selectField('Target output',base+'.channel',c.channel,outputs):''}${!stop?selectField('Value source',base+'.source',c.source,[['slider','Slider + numeric input'],['fixed','Direct signal definition']]):''}${!stop&&!sequence?field('Repeat interval (ms) · bytes/pulses',base+'.repeatMs',c.repeatMs,'number','min="1" max="1000"'):''}${field('Button color',base+'.color',c.color||M.PALETTE.mint,'color')}${field('Active color',base+'.activeColor',c.activeColor||M.PALETTE.red,'color')}${field('Label when idle (optional)',base+'.labelOff',c.labelOff||'')}${field('Label when active (optional)',base+'.labelOn',c.labelOn||'')}<button type="button" class="editor-remove" data-remove-control="${i}">Remove control</button></div>`;
  const slider=!stop&&c.source==='slider'?`<div class="slider-fields">${field('Slider minimum',base+'.min',c.min,'number','step="any"')}${field('Slider maximum',base+'.max',c.max,'number','step="any"')}${field('Step',base+'.step',c.step,'number','min="0.001" step="any"')}${field('Initial value',base+'.value',c.value,'number','step="any"')}</div>`:'';
  let content=stop?'<p class="help">This preset button applies all output channels’ configured idle signals, cancels pending actions, and disables the internal output gate. It can be renamed, colored, moved, or removed.</p>':sequence?`<div class="sequence-editor">${c.steps.map((step,j)=>renderStep(step,`${base}.steps.${j}`,i,j,outputs)).join('')}<div class="step-add"><button type="button" data-add-step="${i}:signal">＋ Signal</button><button type="button" data-add-step="${i}:wait">＋ Wait</button><button type="button" data-add-step="${i}:idle">＋ Channel idle</button></div><p class="help">Steps execute in order. Waveforms run in hardware; waits are timed by the browser. A signal step starts the waveform—add a wait before the next step if it must finish first.</p></div>`:`<div class="signal-editors">${signalEditor(c.on,base+'.on',c.source==='slider'?'On signal · value from slider':'On / press signal')}${signalEditor(c.off,base+'.off','Off / release / alternate signal')}</div>`;
  return `<details class="editor-control" open><summary>${esc(c.name)} · ${esc(M.MODES[c.mode])}</summary>${fields}${slider}${content}</details>`;
 }).join('')||'<p class="help">No generator controls. This can be an input-only preset.</p>';
 wiringEditor.render();$('channelEditor').querySelectorAll('[data-field]').forEach(bindEditorField);$('controlEditor').querySelectorAll('[data-field]').forEach(bindEditorField);
 bindByteEditors($('controlEditor'),key=>getPath(editorDraft,key),(key,error)=>{if(error)editorByteErrors.set(key,error);else editorByteErrors.delete(key);refreshJson();$('editorError').textContent=error||'';});
 document.querySelectorAll('[data-remove-channel]').forEach(b=>b.onclick=()=>{const i=Number(b.dataset.removeChannel);d.channels.splice(i,1);for(const diagram of d.wiring.diagrams){if(diagram.channels)diagram.channels=diagram.channels.filter(ch=>ch!==i).map(ch=>ch>i?ch-1:ch);for(const param of diagram.parameters)if(param.type==='channel'){if(param.value===i)param.value=null;else if(param.value>i)param.value--;}}d.controls=d.controls.filter(c=>c.mode==='stop'||!M.controlChannels(c).includes(i));d.controls.forEach(c=>{if(c.channel>i)c.channel--;for(const step of c.steps||[])if(step.channel>i)step.channel--;});renderEditor();});
 document.querySelectorAll('[data-remove-control]').forEach(b=>b.onclick=()=>{d.controls.splice(Number(b.dataset.removeControl),1);renderEditor();});
 document.querySelectorAll('[data-add-step]').forEach(b=>b.onclick=()=>{const [i,action]=b.dataset.addStep.split(':');if(d.controls[i].steps.length>=32)return;d.controls[i].steps.push(newStep(action,outputs[0]?.[0]??0));renderEditor();});
 document.querySelectorAll('[data-step-op]').forEach(b=>b.onclick=()=>{const [i,j,op]=b.dataset.stepOp.split(':'),steps=d.controls[i].steps,index=Number(j);if(op==='remove')steps.splice(index,1);else{const other=index+(op==='up'?-1:1);if(other>=0&&other<steps.length)[steps[index],steps[other]]=[steps[other],steps[index]];}renderEditor();});refreshJson();
}
function newStep(action,ch=0){return action==='wait'?{action,ms:500}:action==='idle'||action==='stop'?{action,channel:ch}:{action:'signal',channel:ch,source:'fixed',mode:'once',repeatMs:100,signal:M.signal()};}
function renderStep(step,path,i,j,outputs){
 let fields=step.action==='wait'?field('Wait (ms)',path+'.ms',step.ms,'number','min="1" max="60000"'):step.action==='stop'?'<p class="help">Stops all outputs. Must be the last step.</p>':selectField('Output',path+'.channel',step.channel,outputs);
 if(step.action==='signal')fields+=selectField('Signal operation',path+'.mode',step.mode,[['once','Send once'],['continuous','Run continuously']])+selectField('Value',path+'.source',step.source,[['fixed','Direct signal fields'],['slider','Parent control slider']])+field('Repeat interval (ms)',path+'.repeatMs',step.repeatMs,'number','min="1" max="1000"')+signalEditor(step.signal,path+'.signal','Step signal');
 return `<div class="sequence-step"><div class="step-heading"><strong>Step ${j+1}</strong><div><button type="button" data-step-op="${i}:${j}:up" aria-label="Move step ${j+1} up">↑</button><button type="button" data-step-op="${i}:${j}:down" aria-label="Move step ${j+1} down">↓</button><button type="button" data-step-op="${i}:${j}:remove">Remove</button></div></div>${selectField('Action',path+'.action',step.action,[['signal','Send a signal'],['wait','Wait'],['idle','Channel idle'],['stop','Stop all outputs']])}<div class="step-fields">${fields}</div></div>`;
}

function bindEditorField(input){input.onchange=()=>{
 try{const path=input.dataset.field;let value=input.value;if(input.dataset.json)value=JSON.parse(value);else if(input.type==='number'||/\.(pin|channel|rate|value|telemetry|idleLevel|attenuation)$/.test(path))value=Number(value);setPath(editorDraft,path,value);
  if(path.endsWith('.kind')){const parent=path.slice(0,-5),old=getPath(editorDraft,parent),s=M.signal(value,value==='digital'?1:value==='duty'?50:value==='dshot'?0:1500);if(['idle','highz'].includes(value))setPath(editorDraft,parent,{kind:value});else setPath(editorDraft,parent,{...s,...old,kind:value,value:s.value});renderEditor();}
  else if(path.endsWith('.direction')){const ch=getPath(editorDraft,path.slice(0,-10)),pins=ch.direction==='analog'?M.ADC_PINS:ch.direction==='out'?M.OUTPUT_PINS:M.INPUT_PINS;if(!pins.includes(ch.pin))ch.pin=pins.find(p=>!editorDraft.channels.some(c=>c.pin===p))??pins[0];if(ch.direction==='analog'){ch.attenuation??=12;ch.scale??=1;ch.offset??=0;ch.unit??='V';}renderEditor();}
  else if(path.endsWith('.action')){const parent=path.slice(0,-7),old=getPath(editorDraft,parent);setPath(editorDraft,parent,newStep(value,old.channel??editorDraft.channels.findIndex(c=>c.direction==='out')));renderEditor();}
  else if(/^controls\.\d+\.mode$/.test(path)||path.endsWith('.source')||path.endsWith('.placement'))renderEditor();else refreshJson();$('editorError').textContent='';
 }catch(e){$('editorError').textContent=e.message;}
};}
function collectEditor(){editorDraft.name=$('editName').value.trim();editorDraft.description=$('editDescription').value.trim();return editorDraft;}
function refreshJson(){collectEditor();$('presetJson').value=JSON.stringify(editorDraft,null,2);}
$('editName').oninput=$('editDescription').oninput=refreshJson;$('refreshJson').onclick=refreshJson;
$('applyJson').onclick=()=>{try{const d=M.migratePreset(JSON.parse($('presetJson').value));M.validatePreset(d);d.id=editorDraft.id;editorDraft=d;$('editName').value=d.name;$('editDescription').value=d.description||'';renderEditor();$('editorError').textContent='';}catch(e){$('editorError').textContent=e.message;}};
$('addChannel').onclick=()=>{if(editorDraft.channels.length===8){$('editorError').textContent='The firmware supports eight simultaneous channels.';return;}const used=editorDraft.channels.map(c=>c.pin),pin=M.INPUT_PINS.find(p=>!used.includes(p));editorDraft.channels.push(M.channel('Channel '+(editorDraft.channels.length+1),pin,'in'));renderEditor();};
$('addControl').onclick=()=>{const ch=editorDraft.channels.findIndex(c=>c.direction==='out');if(ch<0){$('editorError').textContent='Add an output channel first.';return;}if(editorDraft.controls.length===24)return;editorDraft.controls.push(M.control('New control','continuous',ch));renderEditor();};
$('presetForm').onsubmit=async e=>{e.preventDefault();try{wiringEditor.check();if(editorByteErrors.size)throw Error([...editorByteErrors.values()][0]);const d=M.validatePreset(collectEditor()),index=presets.findIndex(p=>p.id===d.id);if(index<0)presets.push(M.clone(d));else presets[index]=M.clone(d);$('presetEditor').close();persist();await changePreset(d.id);toast('Preset saved.');log('connection','Saved preset: '+d.name);}catch(error){$('editorError').textContent=error.message;}};
$('exportPresets').onclick=()=>download('signal-workbench-presets.json',JSON.stringify({format:'signal-workbench/presets/3',presets},null,2));
$('importPresets').onclick=()=>$('importFile').click();$('importFile').onchange=async()=>{const file=$('importFile').files[0];if(!file)return;try{if(file.size>10000000)throw Error('Preset file must be smaller than 10 MB.');const json=JSON.parse(await file.text()),list=Array.isArray(json)?json:json.presets;if(!Array.isArray(list)||!list.length||list.length>100)throw Error('Import 1–100 presets.');const imported=list.map(p=>{const migrated=M.migratePreset(p);M.validatePreset(migrated);const copy=M.clone(migrated);copy.id=M.uid();copy.controls.forEach(c=>c.id=M.uid());return copy;});presets.push(...imported);persist();await changePreset(imported[0].id);toast(`Imported ${imported.length} presets.`);}catch(e){showError(e);}finally{$('importFile').value='';}};

async function takeCapture(){
 if(!ready()||busy)return;busy=true;drawState();
 try{if(demo)finishCapture(M.demoCapture(preset(),Number($('captureMs').value),$('captureSources').value==='all'));else await transport.command('capture',{ms:Number($('captureMs').value),triggerCh:Number($('trigger').value),triggerEdge:Number($('triggerEdge').value),includeOutputs:$('captureSources').value==='all'},20000);}
 catch(e){incoming=null;stopWatch();showError(e);}finally{busy=false;drawState();if(watching)watchTimer=setTimeout(takeCapture,150);}
}
$('capture').onclick=takeCapture;$('watch').onclick=()=>{if(watching)stopWatch();else{watching=true;takeCapture();drawState();}};
function finishCapture(cap){cap.key=++sequence;cap.name='Capture '+sequence;captures.unshift(cap);if(captures.length>10)captures.pop();currentCapture=cap;$('zoom').value=1;$('pan').value=0;cursors=[];renderCapture();log('capture',`${cap.demo?'[SIMULATED] ':''}${cap.name}: ${num(cap.duration/1000)} ms, ${cap.edges.length} changes, ${num(cap.sampleUs,3)} µs average sampling${cap.full?' · BUFFER FULL':''}${cap.triggered?'':' · TRIGGER TIMEOUT'}`);}
let cursors=[];
function renderCapture(){
 const cap=currentCapture;
 $('history').innerHTML=captures.length?captures.map(c=>`<option value="${c.key}" ${c===cap?'selected':''}>${esc(c.name)}${c.demo?' · demo':''}</option>`).join(''):'<option>No captures yet</option>';
 for(const id of ['exportCapture','exportCsv','captureToPreset'])$(id).disabled=!cap;
 $('scopeEmpty').hidden=!!cap;
 $('captureInfo').textContent=cap?`${cap.demo?'SIMULATED · ':''}${num(cap.duration/1000)} ms · ${cap.edges.length} changes · avg sample ${num(cap.sampleUs,3)} µs${cap.full?' · BUFFER FULL / TRUNCATED':''}${cap.triggered?'':' · TRIGGER NOT FOUND'}`:'No capture · connect probes, then capture';
 $('analysis').innerHTML=cap?(cap.analysis||[]).map(a=>{const ch=cap.channels[a.ch];return `<article class="analysis-card" style="--channel-color:${colors[a.ch]}"><h3>CH${a.ch+1} · ${esc(ch?.name||'Input')}</h3><div class="analysis-kind">${esc(a.kind)}</div><div class="analysis-numbers"><div><strong>${a.hz?num(a.hz):'—'}</strong><small>FREQUENCY / Hz</small></div><div><strong>${a.high?num(a.high):'—'}</strong><small>HIGH / µs</small></div><div><strong>${num(a.duty)}%</strong><small>DUTY CYCLE</small></div><div><strong>${a.edges}</strong><small>EDGES</small></div></div><details><summary>Analysis evidence</summary>${esc(a.description)}<br>Protocol labels are timing-based candidates. Sampling can miss faster edges.</details></article>`;}).join(''):'';
 updateCaptureRoles();drawWaveform();decode();
}
$('history').onchange=()=>{currentCapture=captures.find(c=>c.key===Number($('history').value));cursors=[];$('zoom').value=1;$('pan').value=0;renderCapture();};
function viewRange(){const duration=currentCapture?.duration||60000,span=duration/Number($('zoom').value),start=(duration-span)*Number($('pan').value)/1000;return{start,end:start+span,span};}
function drawWaveform(){
 const canvas=$('waveform'),wrap=$('waveformWrap'),cap=currentCapture,inputs=cap?cap.channels.flatMap((c,i)=>(c.direction!=='analog'&&(c.direction==='in'||cap.includeOutputs))?[i]:[]):[],width=wrap.clientWidth||800,height=Math.max(280,inputs.length*62+48),dpr=window.devicePixelRatio||1;
 canvas.style.height=height+'px';canvas.width=width*dpr;canvas.height=height*dpr;const ctx=canvas.getContext('2d');ctx.scale(dpr,dpr);ctx.fillStyle='#10161c';ctx.fillRect(0,0,width,height);if(!cap)return;
 const left=120,right=width-20,plot=right-left,{start,end,span}=viewRange();
 ctx.font='10px ui-monospace, monospace';
 for(let j=0;j<=8;j++){const x=left+j*plot/8;ctx.strokeStyle='#27313b';ctx.beginPath();ctx.moveTo(x,30);ctx.lineTo(x,height-18);ctx.stroke();ctx.fillStyle='#718293';ctx.fillText(num((start+j*span/8)/1000,3)+' ms',x-13,18);}
 inputs.forEach((ch,row)=>{
  const top=46+row*62,low=top+27,high=top,color=colors[ch];ctx.fillStyle=color;ctx.font='bold 11px ui-monospace, monospace';ctx.fillText('CH'+(ch+1),13,top+9);ctx.font='10px ui-monospace, monospace';ctx.fillStyle='#8192a1';ctx.fillText('GPIO'+cap.channels[ch].pin,13,top+25);
  ctx.save();ctx.beginPath();ctx.rect(left,top-3,plot,40);ctx.clip();ctx.strokeStyle=color;ctx.lineWidth=1.5;ctx.beginPath();let level=M.levelAt(cap,ch,start);ctx.moveTo(left,level?high:low);
  for(const [t,mask] of cap.edges){if(t<=start)continue;if(t>end)break;const next=(mask>>ch)&1;if(next===level)continue;const x=left+(t-start)/span*plot;ctx.lineTo(x,level?high:low);ctx.lineTo(x,next?high:low);level=next;}
  ctx.lineTo(right,level?high:low);ctx.stroke();ctx.restore();
 });
 cursors.forEach((time,i)=>{if(time<start||time>end)return;const x=left+(time-start)/span*plot;ctx.strokeStyle=i?'#ffc0ad':'#f8d788';ctx.setLineDash([4,4]);ctx.beginPath();ctx.moveTo(x,25);ctx.lineTo(x,height);ctx.stroke();ctx.setLineDash([]);});
 $('cursorReadout').textContent=cursors.length===2?`Δt ${num(Math.abs(cursors[1]-cursors[0]),3)} µs · ${num(1e6/Math.abs(cursors[1]-cursors[0]))} Hz`:cursors.length?`A ${num(cursors[0],3)} µs · click for B`:'Click two points to measure Δt';
}
$('waveform').onclick=e=>{if(!currentCapture)return;const rect=e.currentTarget.getBoundingClientRect(),x=e.clientX-rect.left;if(x<120)return;const {start,span}=viewRange(),t=start+(x-120)/(rect.width-140)*span;if(t<0||t>currentCapture.duration)return;if(cursors.length===2)cursors=[];cursors.push(t);drawWaveform();};
$('zoom').oninput=$('pan').oninput=drawWaveform;$('clearCursors').onclick=()=>{cursors=[];drawWaveform();};$('resetView').onclick=()=>{$('zoom').value=1;$('pan').value=0;cursors=[];drawWaveform();};new ResizeObserver(drawWaveform).observe($('waveformWrap'));
function decode(){
 const protocol=$('decoder').value,cap=currentCapture;$('roleALabel').hidden=false;$('roleBLabel').hidden=!['i2c','spi'].includes(protocol);$('roleCLabel').hidden=protocol!=='spi';$('baudLabel').hidden=protocol!=='uart';$('spiModeLabel').hidden=protocol!=='spi';for(const id of ['decodeFormatLabel','decodeEndianLabel','decodeOffsetLabel'])$(id).hidden=protocol!=='uart';
 if(!cap||protocol==='none'){$('decoded').innerHTML='<div class="decoded-empty">'+(!cap?'Capture a signal first.':'Inspect the timing above, or choose UART, I²C, or SPI decoding.')+'</div>';return;}
 const a=Number($('roleA').value),b=Number($('roleB').value),cs=Number($('roleC').value);let rows=[];
 if(protocol!=='uart'&&a===b){$('decoded').textContent='Data and clock need different input channels.';return;}
 if(protocol==='uart'){const baud=Number($('decodeBaud').value);if(baud<300||baud>2000000){$('decoded').textContent='Choose baud 300–2,000,000.';return;}rows=M.uartDecode(cap,a,baud);}
 if(protocol==='i2c')rows=M.i2cDecode(cap,b,a);
 if(protocol==='spi'){if(cs===a||cs===b){$('decoded').textContent='CS must be a separate channel or unused.';return;}rows=M.spiDecode(cap,b,a,cs,Number($('spiMode').value));}
 let interpreted='';
 if(protocol==='uart'&&rows.length){const good=rows.filter(r=>r.type==='UART byte'),hex=good.map(r=>r.data.slice(2)).join('');try{const offset=Number($('decodeOffset').value);if(!Number.isInteger(offset)||offset<0||offset>255)throw Error('Byte offset must be 0–255.');const view=B.inspect(hex,$('decodeFormat').value,$('decodeEndian').value,offset);interpreted=`<div class="received-values"><strong>${esc(B.FORMATS[$('decodeFormat').value])} · ${$('decodeEndian').value==='little'?'little':'big'} endian</strong><pre>${esc(view.text||'No complete values at this offset.')}</pre><p>${view.bytes} bytes after offset ${offset}${view.remainder?' · '+view.remainder+' trailing byte(s) do not form a complete value':''}${rows.length!==good.length?' · framing-error bytes excluded':''}. Groups consecutive bytes; message boundaries are not inferred.</p></div>`;}catch(e){interpreted=`<p class="error-message">${esc(e.message)}</p>`;}}
 $('decoded').innerHTML=rows.length?`${interpreted}<div class="decode-table-wrap"><table><thead><tr><th>Time / µs</th><th>Event</th><th>Value</th><th>Details</th></tr></thead><tbody>${rows.slice(0,1000).map(r=>`<tr><td>${num(r.time,3)}</td><td>${esc(r.type)}</td><td>${esc(r.data)}</td><td>${esc(r.detail)}</td></tr>`).join('')}</tbody></table></div><p class="help">${rows.length} events${rows.length>1000?' · showing first 1000':''}. Decoder settings must match the source. A captured frame may be partial.</p>`:'<div class="decoded-empty">No complete frames with these settings. Check channel roles, baud / mode, capture window, and sampling interval.</div>';
}
for(const id of ['decoder','roleA','roleB','roleC','decodeBaud','spiMode','decodeFormat','decodeEndian','decodeOffset'])$(id).onchange=decode;
$('exportCapture').onclick=()=>{if(currentCapture)download('capture-'+currentCapture.key+'.json',JSON.stringify(currentCapture,null,2));};
$('exportCsv').onclick=()=>{const c=currentCapture;if(!c)return;const header=['time_us',...c.channels.flatMap((ch,i)=>(ch.direction!=='analog'&&(ch.direction==='in'||c.includeOutputs))?['CH'+(i+1)+'_GPIO'+ch.pin]:[])];const masks=[[0,c.initial],...c.edges];const rows=masks.map(([t,v])=>[t,...c.channels.flatMap((ch,i)=>(ch.direction!=='analog'&&(ch.direction==='in'||c.includeOutputs))?[(v>>i)&1]:[])]);download('capture-'+c.key+'.csv',[header,...rows].map(r=>r.join(',')).join('\n'),'text/csv');};
$('captureToPreset').onclick=async()=>{
 const cap=currentCapture;if(!cap)return;const ch=Number($('roleA').value),edges=M.channelEdges(cap,ch),{start,end}=viewRange();const selected=edges.filter(e=>e[0]>=start&&e[0]<=end);if(selected.length<2){toast('Zoom to at least two edges on the selected data channel.');return;}
 if(selected.length>127){toast('More than 126 segments. Zoom into a shorter waveform first.');return;}
 const pulses=selected.slice(0,-1).map((e,i)=>[(e[1]>>ch)&1,Math.max(1,Math.round(selected[i+1][0]-e[0]))]);
 const d=M.migratePreset({id:M.uid(),name:'Captured pulse sequence',description:`${cap.demo?'Simulated':'Captured'} CH${ch+1} waveform · ${pulses.length} segments, rounded to 1 µs. Leading/trailing partial segments omitted.`,channels:[M.channel('Replay output',18)],controls:[{...M.control('Replay once','once'),source:'fixed',on:{...M.signal('pulse'),pulses}}]});
 M.validatePreset(d);presets.push(d);persist();await changePreset(d.id);await openEditor();toast('Pulse preset created. Review timing and pin before using it.');
};
async function takeAnalog(){
 if(!ready()||busy)return;
 const hz=Number($('analogHz').value),ms=Number($('analogMs').value);
 if(Math.ceil(hz*ms/1000)>1024){showError(Error('At most 1024 points per channel. Choose a shorter window or lower sample rate.'));return;}
 busy=true;drawState();
 try{
  if(demo){const signals=preset().channels.flatMap((c,i)=>c.direction==='analog'?[{ch:i,pin:c.pin,unit:M.ADC1_PINS.includes(c.pin)?1:2,attenuation:c.attenuation}]:[]),count=Math.ceil(hz*ms/1000),points=[];for(let i=0;i<count;i++){const row=[i*1e6/hz];for(let j=0;j<signals.length;j++){const mv=1000+400*Math.sin(i/hz*Math.PI*2+j);row.push(Math.round(mv/3300*4095),Math.round(mv));}points.push(row);}finishAnalog({signals,count,points,requestedHz:hz,duration:ms*1000,calibration:'Simulated values',demo:true,presetChannels:M.clone(preset().channels),date:new Date().toISOString()});}
  else await transport.command('analog',{hz,ms},ms+15000);
 }catch(e){incomingAnalog=null;analogWatching=false;showError(e);}finally{busy=false;drawState();if(analogWatching)analogTimer=setTimeout(takeAnalog,150);}
}
function finishAnalog(cap){cap.key=++sequence;cap.name='ADC '+sequence;analogCaptures.unshift(cap);if(analogCaptures.length>10)analogCaptures.pop();currentAnalog=cap;renderAnalog();log('capture',`${cap.demo?'[SIMULATED] ':''}${cap.name}: ${cap.count} points / channel · ${cap.signals.length} analog inputs · ${cap.calibration}${cap.cancelled?' · cancelled':''}`);}
function analogValue(cap,point,j){if($('analogDisplay').value==='raw')return point[1+j*2];const ch=cap.presetChannels[cap.signals[j].ch];return point[2+j*2]/1000*(ch.scale??1)+(ch.offset??0);}
function renderAnalog(){
 const cap=currentAnalog;$('analogHistory').innerHTML=analogCaptures.length?analogCaptures.map(c=>`<option value="${c.key}" ${c===cap?'selected':''}>${c.name}${c.demo?' · demo':''}</option>`).join(''):'<option>No readings yet</option>';
 $('analogExport').disabled=$('analogCsv').disabled=!cap;
 const measured=cap?.points.length>1?(cap.points.length-1)*1e6/(cap.points.at(-1)[0]-cap.points[0][0]):0;
 $('analogInfo').textContent=cap?`${cap.demo?'SIMULATED · ':''}${cap.count} points / channel · actual ${num(measured)} samples/s · ${cap.calibration}${cap.cancelled?' · cancelled':''}`:'Configure analog input channels in Edit preset, then read the ADC.';
 $('analogReadings').innerHTML=cap&&cap.points.length?cap.signals.map((signal,j)=>{const ch=cap.presetChannels[signal.ch],values=cap.points.map(p=>analogValue(cap,p,j)),last=values.at(-1),raw=cap.points.at(-1)[1+j*2],mv=cap.points.at(-1)[2+j*2],range=M.ADC_RANGES[signal.attenuation],warning=mv<range[0]*1000||mv>range[1]*1000;return `<article class="analysis-card" style="--channel-color:${colors[signal.ch]}"><h3>CH${signal.ch+1} · ${esc(ch.name)} · GPIO${signal.pin}</h3><div class="analog-value">${num(last,4)} <small>${$('analogDisplay').value==='raw'?'counts':esc(ch.unit||'V')}</small></div><div class="analysis-numbers"><div><strong>${num(Math.min(...values),4)}</strong><small>MIN</small></div><div><strong>${num(Math.max(...values),4)}</strong><small>MAX</small></div><div><strong>${num(values.reduce((a,b)=>a+b,0)/values.length,4)}</strong><small>MEAN</small></div><div><strong>${raw}</strong><small>LAST RAW / 4095</small></div></div><p class="help">ADC${signal.unit} · ${signal.attenuation} dB · ${mv} mV at pin${warning?' · Outside characterized range':''}${raw>=4090?' · ADC saturated':''}</p></article>`;}).join(''):'';
 drawAnalog();
}
function drawAnalog(){
 const canvas=$('analogWaveform'),cap=currentAnalog,width=canvas.parentElement.clientWidth||800,height=Math.max(250,(cap?.signals.length||1)*140),dpr=window.devicePixelRatio||1;canvas.style.height=height+'px';canvas.width=width*dpr;canvas.height=height*dpr;const ctx=canvas.getContext('2d');ctx.scale(dpr,dpr);ctx.fillStyle='#10161c';ctx.fillRect(0,0,width,height);ctx.font='11px ui-monospace, monospace';if(!cap?.points.length){ctx.fillStyle='#8f9baa';ctx.fillText('Analog traces appear after reading the ADC.',25,60);return;}
 const raw=$('analogDisplay').value==='raw',left=105,plot=width-left-20,lastTime=Math.max(1,cap.points.at(-1)[0]);
 cap.signals.forEach((signal,j)=>{const ch=cap.presetChannels[signal.ch],top=j*140+30,bottom=top+80,values=cap.points.map(p=>analogValue(cap,p,j)),range=M.ADC_RANGES[signal.attenuation],expected=range[1]*(ch.scale??1)+(ch.offset??0),lo=raw?0:Math.min(ch.offset??0,...values,expected),hi=raw?4095:Math.max(...values,expected,lo+.1);ctx.fillStyle=colors[signal.ch];ctx.fillText('CH'+(signal.ch+1),12,top+10);ctx.fillText('GPIO'+signal.pin,12,top+28);ctx.fillStyle='#718293';ctx.fillText(num(hi,3),left-55,top+5);ctx.fillText(num(lo,3),left-55,bottom+5);ctx.strokeStyle='#2b3642';for(let k=0;k<=4;k++){const x=left+k*plot/4;ctx.beginPath();ctx.moveTo(x,top);ctx.lineTo(x,bottom);ctx.stroke();ctx.fillText(num(lastTime*k/4/1000)+' ms',x-12,bottom+20);}ctx.strokeStyle=colors[signal.ch];ctx.lineWidth=1.5;ctx.beginPath();cap.points.forEach((point,i)=>{const x=left+point[0]/lastTime*plot,y=bottom-(values[i]-lo)/(hi-lo)*(bottom-top);if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y);});ctx.stroke();});
}
$('analogCapture').onclick=takeAnalog;
$('analogWatch').onclick=()=>{if(analogWatching){analogWatching=false;clearTimeout(analogTimer);if(connected&&busy)transport.command('cancelCapture').catch(showError);drawState();}else{analogWatching=true;takeAnalog();drawState();}};
$('analogDisplay').onchange=renderAnalog;
$('analogHistory').onchange=()=>{currentAnalog=analogCaptures.find(c=>c.key===Number($('analogHistory').value));renderAnalog();};
$('analogExport').onclick=()=>{if(currentAnalog)download('adc-'+currentAnalog.key+'.json',JSON.stringify(currentAnalog,null,2));};
$('analogCsv').onclick=()=>{const c=currentAnalog;if(!c)return;const header=['time_us',...c.signals.flatMap(s=>['CH'+(s.ch+1)+'_raw','CH'+(s.ch+1)+'_pin_mV','CH'+(s.ch+1)+'_scaled'])];const rows=c.points.map(p=>[p[0],...c.signals.flatMap((s,j)=>{const ch=c.presetChannels[s.ch];return[p[1+j*2],p[2+j*2],p[2+j*2]/1000*(ch.scale??1)+(ch.offset??0)];})]);download('adc-'+c.key+'.csv',[header,...rows].map(r=>r.join(',')).join('\n'),'text/csv');};
new ResizeObserver(drawAnalog).observe($('analogWaveform').parentElement);

$('decodeFormat').innerHTML=Object.entries(B.FORMATS).map(([v,n])=>`<option value="${v}">${n}</option>`).join('');
persist();render();renderLog();log('connection','Workbench ready. Saved presets stay in this browser; export for backup.');

function updateCaptureRoles(){
 const all=$('captureSources').value==='all',p=preset();
 const triggers=p.channels.flatMap((c,i)=>(c.direction!=='analog'&&(c.direction==='in'||all))?[[i,c]]:[]);
 const triggerValue=$('trigger').value;
 $('trigger').innerHTML='<option value="-1">Immediate</option>'+triggers.map(([i,c])=>`<option value="${i}">CH${i+1} · ${esc(c.name)}</option>`).join('');
 if(triggers.some(([i])=>String(i)===triggerValue))$('trigger').value=triggerValue;
 const channels=currentCapture?.channels||p.channels,monitor=currentCapture?.includeOutputs||all;
 const roles=channels.flatMap((c,i)=>(c.direction!=='analog'&&(c.direction==='in'||monitor))?[[i,c]]:[]),options=roles.map(([i,c])=>`<option value="${i}">CH${i+1} · ${esc(c.name)}</option>`).join('');
 for(const id of ['roleA','roleB','roleC']){const old=$(id).value;$(id).innerHTML=(id==='roleC'?'<option value="-1">Not used</option>':'')+options;if(roles.some(([i])=>String(i)===old)||old==='-1')$(id).value=old;}
 if($('roleA').value===$('roleB').value&&roles.length>1)$('roleB').value=roles[1][0];
}
$('captureSources').onchange=()=>{updateCaptureRoles();drawState();};
