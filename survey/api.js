'use strict';
// Where the survey's server (server.py, the companion on this computer) is. A page it serves sends its requests to
// its own address. A page served elsewhere (LiftLab's Room Survey dialog, GitHub Pages) names the companion with
// ?companion=http://127.0.0.1:PORT. Only this computer's addresses are accepted, since the page shows and runs what
// the companion sends.
const API=(()=>{
  const named=new URLSearchParams(location.search).get('companion');
  if(!named)return '';
  return companionOrigin(named)||'http://invalid.companion';
})();

// The origin of a companion address on this computer, else ''.
function companionOrigin(address){
  try{
    const url=new URL(address);
    return url.protocol==='http:'&&['127.0.0.1','localhost','[::1]'].includes(url.hostname)&&!url.username&&!url.password?
      url.origin:'';
  }catch{return '';}
}

// A path the companion serves ('/state', '/operator?…') as a URL this page can request; anything else as it is.
function api(path){return typeof path==='string'&&path.startsWith('/')&&!path.startsWith('//')?API+path:path;}
