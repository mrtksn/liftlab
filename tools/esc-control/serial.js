(function(root,factory){const api=factory();if(typeof module==='object')module.exports=api;else root.SignalTransport=api;})(globalThis,()=>{
'use strict';
class Transport{
 constructor(onEvent,onLost){this.onEvent=onEvent;this.onLost=onLost;this.pending=new Map();this.nextId=1;this.epoch=0;this.queue=Promise.resolve();this.active=false;this.ready=false;this.closing=false;}
 async open(){
  if(!navigator.serial)throw Error('Open this page in Chrome or Edge: Web Serial is required.');
  this.port=await navigator.serial.requestPort();await this.port.open({baudRate:921600});
  this.writer=this.port.writable.getWriter();this.reader=this.port.readable.getReader();this.active=true;this.closing=false;this.queue=Promise.resolve();this.readLoop();
  try{let hello,lastError;for(let attempt=0;attempt<4;attempt++){try{hello=await this.command('hello',{},1500);break;}catch(error){lastError=error;if(!this.active)throw error;}}if(!hello)throw lastError;if(hello.protocol!==2)throw Error('This UI needs Signal Workbench firmware v2.');this.ready=true;this.heartbeat=setInterval(()=>this.write({cmd:'ping'}).catch(()=>{}),180);return hello;}
  catch(error){await this.close();throw Error('Signal Workbench firmware did not respond. '+error.message);}
 }
 write(message,guard=false,accept=()=>true){
  const writer=this.writer,epoch=this.epoch;
  const result=this.queue.catch(()=>{}).then(()=>{if(!this.active||writer!==this.writer)throw Error('Serial disconnected.');if(!accept()||(guard&&epoch!==this.epoch))throw Error('Cancelled');return writer.write(new TextEncoder().encode(JSON.stringify(message)+'\n'));});
  this.queue=result;return result;
 }
 command(cmd,data={},timeout=8000,accept=()=>true){
  const id=this.nextId++;
  return new Promise((resolve,reject)=>{
   const timer=setTimeout(()=>{this.pending.delete(id);reject(Error(cmd+' timed out.'));},timeout);
   this.pending.set(id,{resolve,reject,timer});
   this.write({...data,cmd,id},['signal','enable','idle'].includes(cmd),accept).catch(error=>{const p=this.pending.get(id);if(p){clearTimeout(p.timer);this.pending.delete(id);p.reject(error);}if(error.message!=='Cancelled')this.lost(error);});
  });
 }
 stop(){this.epoch++;return this.command('stop');}
 async readLoop(){
  const decoder=new TextDecoder();let carry='';
  try{while(this.active){const {value,done}=await this.reader.read();if(done)break;carry+=decoder.decode(value,{stream:true});if(carry.length>262144)throw Error('Device reply exceeded buffer limit.');const lines=carry.split(/\r?\n/);carry=lines.pop();for(const line of lines){const start=line.indexOf('@SB ');if(start<0)continue;let event;try{event=JSON.parse(line.slice(start+4));}catch{continue;}this.onEvent(event);
   const p=this.pending.get(event.id);if(p&&['hello','ack','error','captureEnd','analogEnd'].includes(event.type)){clearTimeout(p.timer);this.pending.delete(event.id);event.type==='error'?p.reject(Error(event.message)):p.resolve(event);}
  }}}catch(error){if(!this.closing)this.lost(error);}finally{if(this.active&&!this.closing)this.lost(Error('Device disconnected.'));}
 }
 lost(error){if(!this.active)return;this.active=false;this.ready=false;this.epoch++;clearInterval(this.heartbeat);for(const p of this.pending.values()){clearTimeout(p.timer);p.reject(error);}this.pending.clear();this.onLost(error);}
 async close(){
  this.closing=true;this.epoch++;this.active=false;this.ready=false;clearInterval(this.heartbeat);
  for(const p of this.pending.values()){clearTimeout(p.timer);p.reject(Error('Disconnected'));}this.pending.clear();
  try{await this.reader?.cancel();this.reader?.releaseLock();}catch{}
  try{await this.queue.catch(()=>{});this.writer?.releaseLock();}catch{}
  try{await this.port?.close();}catch{}
  this.reader=this.writer=this.port=null;
 }
}
return Transport;
});
