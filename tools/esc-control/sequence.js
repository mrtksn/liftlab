/* Cancellable host-timed sequences. Waveforms themselves run in ESP32 hardware. */
(function(root,factory){const api=factory();if(typeof module==='object')module.exports=api;else root.SignalSequence=api;})(globalThis,()=>{
'use strict';
const cancelled=()=>Error('Cancelled');
function check(signal){if(signal?.aborted)throw cancelled();}
function wait(ms,signal){return new Promise((resolve,reject)=>{check(signal);const abort=()=>{clearTimeout(timer);signal?.removeEventListener('abort',abort);reject(cancelled());};const timer=setTimeout(()=>{signal?.removeEventListener('abort',abort);resolve();},ms);signal?.addEventListener('abort',abort,{once:true});});}
async function execute(steps,{send,idle,stop,onStep=()=>{},resolveSignal=step=>step.signal,delay=wait},signal){
 for(let i=0;i<steps.length;i++){
  check(signal);const step=steps[i];onStep(i,steps.length);
  if(step.action==='wait')await delay(step.ms,signal);
  else if(step.action==='idle')await idle(step.channel,signal);
  else if(step.action==='stop'){await stop(signal);return;}
  else await send(step.channel,resolveSignal(step),step.mode||'once',step.repeatMs||100,signal);
  check(signal);
 }
}
return {execute,wait,check};
});
