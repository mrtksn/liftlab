/* Load declarative configuration before any instrument code. */
'use strict';
(async()=>{
 const read=async url=>{const r=await fetch(url,{cache:'no-cache'});if(!r.ok)throw Error('Cannot load '+url+' ('+r.status+')');return r.json();};
 try{
  const manifest=await read('config/manifest.json');
  window.SignalConfig=Object.fromEntries(await Promise.all(Object.entries(manifest.resources).map(async([key,file])=>[key,await read('config/'+file)])));
  for(const file of manifest.scripts)await new Promise((resolve,reject)=>{const script=document.createElement('script');script.src=file+'?v='+encodeURIComponent(manifest.revision);script.onload=resolve;script.onerror=()=>reject(Error('Cannot load '+file));document.body.append(script);});
 }catch(error){const notice=document.getElementById('notice');notice.textContent='Configuration could not be loaded. '+error.message;notice.className='notice error';}
})();
