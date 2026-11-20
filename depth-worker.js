// All inference runs in this worker on rectified images served by the Mac.
import * as ort from '/depth-assets/ort.wasm.min.mjs';

ort.env.wasm.numThreads=1;
ort.env.wasm.wasmPaths=new URL('/depth-assets/',self.location.origin).href;
const width=392, height=294;
const input=new OffscreenCanvas(width,height).getContext('2d',{willReadFrequently:true});
const mean=[.485,.456,.406], std=[.229,.224,.225];

async function predict(session,url){
  const response=await fetch(url);
  if(!response.ok)throw Error('Recorded image unavailable.');
  const bitmap=await createImageBitmap(await response.blob());
  input.drawImage(bitmap,0,0,width,height);
  bitmap.close();
  const rgba=input.getImageData(0,0,width,height).data;
  const pixels=width*height, tensor=new Float32Array(3*pixels);
  for(let c=0;c<3;c++){
    for(let i=0;i<pixels;i++)tensor[c*pixels+i]=(rgba[i*4+c]/255-mean[c])/std[c];
  }
  const image=new ort.Tensor('float32',tensor,[1,3,height,width]);
  const result=await session.run({[session.inputNames[0]]:image});
  const depth=result[session.outputNames[0]];
  const [h,w]=depth.dims.slice(-2), values=depth.data;
  const sampled=new Float32Array(96*72);
  for(let y=0;y<72;y++)for(let x=0;x<96;x++){
    sampled[y*96+x]=values[Math.floor((y+.5)*h/72)*w+Math.floor((x+.5)*w/96)];
  }
  image.dispose();depth.dispose();
  const lo=Math.min(...sampled), span=Math.max(...sampled)-lo;
  if(!Number.isFinite(span)||span<=0)throw Error('Model returned an empty depth image.');
  return Uint8Array.from(sampled,v=>Math.round(255*(v-lo)/span));
}

self.onmessage=async({data})=>{
  let session;
  try{
    const predictions={};
    const cache=await caches.open('room-survey-depth-anything-v2-v1');
    for(const [i,frame] of data.frames.entries()){
      const url='/depth-input.jpg?'+new URLSearchParams({run:data.run,index:data.index,frame});
      const entry=await cache.match(url);
      let raw;
      if(entry)raw=new Uint8Array(await entry.arrayBuffer());
      if(raw?.length!==96*72){
        if(!session){
          self.postMessage({status:'Loading Depth Anything V2 on this Mac…'});
          session=await ort.InferenceSession.create('/depth-assets/depth.onnx',{executionProviders:['wasm']});
        }
        self.postMessage({status:'Predicting depth · view '+(i+1)+' of '+data.frames.length});
        raw=await predict(session,url);
        await cache.put(url,new Response(raw));
      }
      predictions[frame]=btoa(String.fromCharCode(...raw));
      self.postMessage({frame,raw,url,status:'Depth ready · '+(i+1)+' of '+data.frames.length});
    }
    self.postMessage({predictions});
  }catch(error){
    self.postMessage({error:'Depth inference failed · '+(error.message||String(error))});
  }finally{if(session)await session.release();}
};
