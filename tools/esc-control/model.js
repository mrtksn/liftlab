/* Shared preset validation and capture decoders. No hardware or DOM access. */
(function(root,factory){const api=factory();if(typeof module==='object')module.exports=api;else root.SignalModel=api;})(globalThis,()=>{
'use strict';
const C=typeof module==='object'?require('./configuration.js'):globalThis.SignalConfig;
const W=typeof module==='object'?require('./wiring.js'):globalThis.SignalWiring;
const OUTPUT_PINS=C.board.capabilities.outputs,INPUT_PINS=C.board.capabilities.inputs;
const ADC1_PINS=C.board.capabilities.adc1,ADC2_PINS=C.board.capabilities.adc2,ADC_PINS=[...ADC1_PINS,...ADC2_PINS];
const ADC_RANGES=C.board.adcRanges;
const KINDS=['pwm','duty','digital','pulse','uart','dshot','highz','idle'];
const B=typeof module==='object'?require('./bytes.js'):globalThis.SignalBytes;
const PALETTE=C.defaults.palette;
const MODES={sequence:'Sequence · signals and delays',stop:'Stop · idle all outputs',continuous:'On/off · repeat while on',edges:'On/off · one signal on each edge',hold:'Hold · repeat until released',holdEdges:'Hold · signal on press and release',once:'Once · one signal per press',alternate:'Alternate · on/off signal each press'};
const clone=value=>JSON.parse(JSON.stringify(value));
const uid=()=>globalThis.crypto?.randomUUID?.()||'id-'+Date.now()+'-'+Math.random().toString(16).slice(2);
const signal=(kind=C.defaults.signal.kind,value=C.defaults.signal.value)=>({...clone(C.defaults.signal),kind,value});
const channel=(name,pin,direction='out',idle=C.defaults.channelIdle)=>({name,pin,direction,idle:clone(idle),...(direction==='analog'?clone(C.defaults.analog):{})});
const control=(name,mode=C.defaults.control.mode,ch=0)=>({...clone(C.defaults.control),id:uid(),name,mode,channel:ch,steps:[{action:'signal',channel:ch,mode:'once',source:'fixed',repeatMs:C.defaults.control.repeatMs,signal:signal()}],on:signal(),off:{kind:'idle'}});
function defaults(){return clone(C.presets).map(migratePreset);}
function stopControl(){return {...clone(C.defaults.stopControl),id:uid()};}
function newPreset(){const p={...clone(C.defaults.newPreset),id:uid(),schemaVersion:3,controls:[control(C.defaults.control.name)]};p.wiring=W.newWiring(p);return p;}
function migratePreset(input){
 const p=clone(input),version=p.schemaVersion||1,seed=C.presets.find(s=>s.id===p.id);
 if(seed?.previousDescriptions?.includes(p.description))p.description=seed.description;
 p.controls.forEach(c=>{c.placement??=C.defaults.control.placement;c.color??=C.defaults.control.color;c.activeColor??=C.defaults.control.activeColor;c.labelOff??='';c.labelOn??='';c.steps??=[{action:'signal',channel:c.channel,mode:'once',source:'fixed',repeatMs:c.repeatMs||C.defaults.control.repeatMs,signal:clone(c.on)}];});
 if(version<2&&p.channels.some(ch=>ch.direction==='out')){
  for(const item of C.defaults.legacy.presetControls[p.id]||[])if(p.controls.length<24)p.controls.unshift({...clone(item),id:uid()});
  for(const item of C.defaults.legacy.defaultControls)if(p.controls.length<24&&!p.controls.some(c=>c.mode===item.mode))p.controls.push({...clone(item),id:uid()});
 }
 if(!Object.hasOwn(p,'wiring'))p.wiring=seed?clone(seed.wiring):W.newWiring(p);
 p.schemaVersion=3;return p;
}
function controlChannels(c){return c.mode==='stop'?[]:c.mode==='sequence'?[...new Set(c.steps.filter(s=>['signal','idle'].includes(s.action)).map(s=>s.channel))]:[c.channel];}
function resolveStep(c,step){const s=clone(step.signal);if(step.source==='slider')s.value=c.value;return s;}
function validateSteps(c,p){
 if(!Array.isArray(c.steps)||!c.steps.length||c.steps.length>32)throw Error('A sequence needs 1–32 steps.');
 let duration=0;
 for(const [i,step] of c.steps.entries()){
  if(step.action==='wait'){if(!Number.isFinite(step.ms)||step.ms<1||step.ms>60000)throw Error('A wait must be 1–60000 ms.');duration+=step.ms;continue;}
  if(step.action==='stop'){if(i!==c.steps.length-1)throw Error('Stop all outputs must be the last sequence step.');continue;}
  if(!['signal','idle'].includes(step.action)||!Number.isInteger(step.channel)||p.channels[step.channel]?.direction!=='out')throw Error('Each signal / idle step needs an output channel.');
  if(step.action==='idle')continue;
  if(!['once','continuous'].includes(step.mode)||!['fixed','slider'].includes(step.source)||!Number.isFinite(step.repeatMs)||step.repeatMs<1||step.repeatMs>1000)throw Error('Check the sequence signal mode, value source, and repeat interval.');
  validateSignal(step.signal);
  if(step.source==='slider'){
   if(c.source!=='slider'||!['pwm','duty','digital','dshot'].includes(step.signal.kind))throw Error('Sequence slider steps need a numeric slider and numeric signal.');
   validateSignal({...step.signal,value:c.min});validateSignal({...step.signal,value:c.max});
  }
 }
 if(duration>120000)throw Error('Sequence waits may total at most 120000 ms.');
}
function validateSignal(s,idle=false){
 if(!s||!KINDS.includes(s.kind))throw Error('Choose a supported signal type.');
 if(idle&&!['pwm','duty','digital','highz'].includes(s.kind))throw Error('A channel idle signal must be PWM, duty PWM, a digital level, or high impedance.');
 const finite=x=>typeof x==='number'&&Number.isFinite(x);
 if(['pwm','duty'].includes(s.kind)){
  if(!Number.isInteger(s.hz)||s.hz<1||s.hz>1000000)throw Error('Frequency must be 1–1,000,000 whole Hz.');
  if(!finite(s.value)||s.value<0||(s.kind==='pwm'?s.value*s.hz>1000000:s.value>100))throw Error('Pulse width must fit its period; duty must be 0–100%.');
 }
 if(s.kind==='digital'&&![0,1].includes(s.value))throw Error('A digital level must be 0 or 1.');
 if(s.kind==='digital'&&s.duration!==undefined&&(!finite(s.duration)||s.duration<1||s.duration>1000000))throw Error('Pulse duration must be 1–1,000,000 µs.');
 if(s.kind==='uart'&&s.encoding!==undefined&&!B.FORMATS[s.encoding])throw Error('Choose a UART display format.');
 if(s.kind==='uart'&&s.endian!==undefined&&!['little','big'].includes(s.endian))throw Error('Choose little or big endian.');
 if(s.kind==='uart'&&(typeof s.data!=='string'||!/^(?:[a-f0-9]{2}){1,12}$/i.test(s.data)||!finite(s.baud)||s.baud<300||s.baud>1000000))throw Error('UART needs 1–12 hex bytes and baud 300–1,000,000.');
 if(s.kind==='dshot'&&(![150,300,600].includes(s.rate)||!Number.isInteger(s.value)||s.value<0||s.value>2047))throw Error('DShot needs rate 150/300/600 and value 0–2047.');
 if(s.kind==='pulse'&&(!Array.isArray(s.pulses)||s.pulses.length<1||s.pulses.length>126||s.pulses.some(p=>!Array.isArray(p)||p.length!==2||![0,1].includes(p[0])||!finite(p[1])||p[1]<1||p[1]>1000000)))throw Error('Pulse sequences need 1–126 [level, microseconds] segments.');
}
function validatePreset(p){
 if(!p||typeof p.name!=='string'||!p.name.trim()||p.name.length>100)throw Error('Give the preset a name (up to 100 characters).');
 if(!Array.isArray(p.channels)||p.channels.length<1||p.channels.length>8)throw Error('Use 1–8 channels per preset.');
 const pins=new Set();
 for(const ch of p.channels){
  if(!['in','out','analog'].includes(ch.direction))throw Error('Choose input or output for each channel.');
  if(!Number.isInteger(ch.pin)||!(ch.direction==='out'?OUTPUT_PINS:INPUT_PINS).includes(ch.pin))throw Error('GPIO '+ch.pin+' is reserved or cannot drive an output.');
  if(pins.has(Number(ch.pin)))throw Error('Each GPIO can only be assigned once.');pins.add(Number(ch.pin));
  if(typeof ch.name!=='string'||ch.name.length>80)throw Error('Channel names must be under 80 characters.');
  if(ch.direction==='out')validateSignal(ch.idle,true);
  if(ch.direction==='analog'&&(!ADC_PINS.includes(ch.pin)||!Object.hasOwn(ADC_RANGES,ch.attenuation)||!Number.isFinite(ch.scale)||ch.scale===0||!Number.isFinite(ch.offset)||typeof ch.unit!=='string'||ch.unit.length>12))throw Error('Analog inputs need an ADC pin, attenuation, finite scale / offset and a short unit label.');
 }
 if(!Array.isArray(p.controls)||p.controls.length>24)throw Error('A preset can have up to 24 controls.');
 const ids=new Set();
 for(const c of p.controls){
  if(typeof c.id!=='string'||ids.has(c.id))throw Error('Control IDs must be unique.');ids.add(c.id);
  if(typeof c.name!=='string'||!c.name.trim()||c.name.length>100)throw Error('Name each control (up to 100 characters).');
  if(!MODES[c.mode]||(!['stop','sequence'].includes(c.mode)&&(!Number.isInteger(c.channel)||p.channels[c.channel]?.direction!=='out')))throw Error('Each control needs a mode and output channel.');
  if(!['slider','fixed'].includes(c.source))throw Error('Choose a slider or direct value.');
  if(![c.min,c.max,c.step,c.value,c.repeatMs].every(x=>typeof x==='number'&&Number.isFinite(x))||c.min>=c.max||c.step<=0||c.value<c.min||c.value>c.max||c.repeatMs<1||c.repeatMs>1000)throw Error('Check slider bounds, value, step, and repeat interval (1–1000 ms).');
  if(!['card','toolbar'].includes(c.placement||'card')||![c.color||PALETTE.mint,c.activeColor||PALETTE.red].every(v=>/^#[a-f0-9]{6}$/i.test(v)))throw Error('Choose a card / toolbar placement and valid button colors.');
  if(c.mode==='sequence')validateSteps(c,p);
  if(c.mode==='stop')continue;
  validateSignal(c.on);validateSignal(c.off);
  if(c.source==='slider'&&c.mode!=='sequence'){
   if(!['pwm','duty','digital','dshot'].includes(c.on.kind))throw Error('Sliders control pulse width, duty, digital level, or DShot value. Use direct input for byte/pulse sequences.');
   validateSignal({...c.on,value:c.min});validateSignal({...c.on,value:c.max});
  }
 }
 W.validate(p.wiring);
 return p;
}
function resolveSignal(c,value){return {...clone(c.on),value:c.source==='slider'?Number(value):Number(c.on.value)};}
function levelAt(cap,ch,t){let lo=0,hi=cap.edges.length;while(lo<hi){const m=(lo+hi)>>1;if(cap.edges[m][0]<=t)lo=m+1;else hi=m;}return ((lo?cap.edges[lo-1][1]:cap.initial)>>ch)&1;}
function channelEdges(cap,ch){let prev=(cap.initial>>ch)&1;return cap.edges.filter(p=>{const next=(p[1]>>ch)&1;if(next===prev)return false;prev=next;return true;});}
function uartDecode(cap,ch,baud){
 const edges=channelEdges(cap,ch),bit=1e6/baud,rows=[];let end=-1;
 for(const [t,mask] of edges){if(t<end||((mask>>ch)&1)||t+bit*10>cap.duration)continue;
  let value=0;for(let k=0;k<8;k++)value|=levelAt(cap,ch,t+(1.5+k)*bit)<<k;
  const good=levelAt(cap,ch,t+9.5*bit)===1;
  rows.push({time:t,type:good?'UART byte':'Framing error',data:'0x'+value.toString(16).padStart(2,'0').toUpperCase(),detail:value>=32&&value<127?String.fromCharCode(value):'·'});end=t+bit*9.5;
 }return rows;
}
function i2cDecode(cap,scl,sda){
 const rows=[];let old=cap.initial,active=false,bits=0,value=0,address=true,start=0;
 for(const [t,v] of cap.edges){
  const a=(old>>scl)&1,b=(v>>scl)&1,x=(old>>sda)&1,y=(v>>sda)&1;
  if(a&&b&&x!==y){active=y===0;bits=0;value=0;address=true;rows.push({time:t,type:y?'STOP':'START',data:'',detail:''});}
  else if(active&&!a&&b){if(bits===0)start=t;if(bits<8)value=(value<<1)|y;bits++;
   if(bits===9){rows.push({time:start,type:address?'Address':'Data',data:'0x'+(address?value>>1:value).toString(16).padStart(2,'0').toUpperCase(),detail:(address?(value&1?'Read · ':'Write · '):'')+(y?'NACK':'ACK')});address=false;bits=0;value=0;}}
  old=v;
 }return rows;
}
function spiDecode(cap,clk,data,cs=-1,mode=0){
 const rows=[],cpol=(mode>>1)&1,cpha=mode&1;let old=cap.initial,bits=0,value=0,start=0;
 for(const [t,v] of cap.edges){
  if(cs>=0&&((v>>cs)&1)){bits=0;value=0;old=v;continue;}
  const a=(old>>clk)&1,b=(v>>clk)&1;
  if(a!==b&&b===(cpha?cpol:1-cpol)){
   if(!bits)start=t;value=(value<<1)|((v>>data)&1);bits++;
   if(bits===8){rows.push({time:start,type:'SPI byte',data:'0x'+value.toString(16).padStart(2,'0').toUpperCase(),detail:'MSB first · mode '+mode});bits=0;value=0;}
  }old=v;
 }return rows;
}
function analyze(cap,ch){
 const e=channelEdges(cap,ch),rise=[],high=[];let lastRise=null;
 for(const [t,v] of e){if((v>>ch)&1){rise.push(t);lastRise=t;}else if(lastRise!==null){high.push(t-lastRise);lastRise=null;}}
 const median=a=>a.length?[...a].sort((x,y)=>x-y)[a.length>>1]:0;
 const periods=rise.slice(1).map((t,i)=>t-rise[i]),period=median(periods),width=median(high);
 return {ch,kind:period?'Periodic digital':'Steady / sparse',description:period?'Timing estimate from captured edges':'Insufficient complete periods',hz:period?1e6/period:0,high:width,duty:period?100*width/period:0,edges:e.length};
}
function demoCapture(p,ms=60,includeOutputs=false){
 const duration=ms*1000;let initial=0;const changes=[];
 p.channels.forEach((c,i)=>{if(c.direction==='analog'||(c.direction!=='in'&&!includeOutputs))return;const period=i%2?2040.816:20000,high=i%2?1500:1000;
  for(let t=500;t<duration;t+=period){changes.push([t,i,1]);if(t+high<duration)changes.push([t+high,i,0]);}});
 changes.sort((a,b)=>a[0]-b[0]);let mask=initial;const edges=[];
 for(const [t,ch,v] of changes){mask=v?mask|(1<<ch):mask&~(1<<ch);if(edges.length&&edges.at(-1)[0]===t)edges.at(-1)[1]=mask;else edges.push([t,mask]);}
 const cap={initial,edges,duration,count:edges.length,sampleUs:.5,full:false,triggered:true,demo:true,includeOutputs,channels:clone(p.channels),date:new Date().toISOString()};
 cap.analysis=p.channels.flatMap((c,i)=>(c.direction!=='analog'&&(c.direction==='in'||includeOutputs))?[analyze(cap,i)]:[]);return cap;
}
return {newPreset,ADC1_PINS,ADC2_PINS,ADC_PINS,ADC_RANGES,PALETTE,migratePreset,stopControl,controlChannels,resolveStep,validateSteps,OUTPUT_PINS,INPUT_PINS,KINDS,MODES,clone,uid,signal,channel,control,defaults,validateSignal,validatePreset,resolveSignal,levelAt,channelEdges,uartDecode,i2cDecode,spiDecode,analyze,demoCapture};
});
