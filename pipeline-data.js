// These presets are ordinary graphs. Their edits are saved with the graph.
const sourcePass = 'return input;';
const viewPass = 'return input; // connected layers, in display order';
const nativePass = 'return await runNative(settings);';
function recordingPreset(rangeChart=false){
  const nodes=[{id:'walk',type:'walk',settings:{program:sourcePass}}], inputs=[];
  if(rangeChart){
    nodes.push({id:'range',type:'code',input:'walk',settings:{
      name:'Range samples',inputType:'Recording tables',outputType:'JSON records',
      program:"return (input['range.csv'] || []).map(row => ({\n  seconds: row.pi_read_end_ns / 1e9,\n  raw: row.tof_raw,\n  metres: row.tof_raw > 0 && row.tof_raw < 250 ? row.tof_raw / 104 : null\n}));"
    }});
    inputs.push({source:'range',type:'JSON records',presentation:{mode:'plot',x:'seconds',y:'metres'}});
  }
  nodes.push({id:'viewer',type:'viewer',settings:{inputs,program:viewPass}});
  return {graph:{nodes,selected:rangeChart?'viewer':'walk',viewer:'viewer'}};
}

function isTableType(type){return ['Recording tables','Telemetry','JSON records'].includes(type);}
function isImageType(type){return type==='Recorded photos'||type==='Depth frames'||type==='Pixel frames';}

let recordedTablesRun, recordedTablesRequest;
function readTables(run){
  if(run!==recordedTablesRun||!recordedTablesRequest){
    recordedTablesRun=run;
    recordedTablesRequest=fetch('/recording-tables?'+new URLSearchParams({run})).then(async response=>{
      if(!response.ok)throw Error('Could not read the recording tables.');
      return response.json();
    }).catch(error=>{if(run===recordedTablesRun)recordedTablesRequest=null;throw error;});
  }
  return recordedTablesRequest;
}
function frameRefs(type,source){
  return timeline.map(f=>({frame:f.id,ms:f.ms,...(type==='Depth frames'?{method:source.settings.geometrySource}:{})}));
}
function pixelReader(refs,run,index){
  const permitted=new Set(refs.map(f=>JSON.stringify([f.frame,f.method||''])));
  return async ref=>{
    if(!ref||!permitted.has(JSON.stringify([ref.frame,ref.method||''])))throw Error('Choose a frame from a connected image port.');
    const query=new URLSearchParams({run,index,frame:ref.frame,method:ref.method||''});
    const response=await fetch((ref.method?'/frame-depth.bin?':'/recorded.jpg?')+query);
    if(!response.ok)throw Error('Frame unavailable: '+ref.frame);
    if(ref.method){
      const buffer=await response.arrayBuffer(),header=new DataView(buffer);
      return {width:header.getUint32(0,true),height:header.getUint32(4,true),pixels:new Float32Array(buffer,8),format:'depth'};
    }
    const image=await createImageBitmap(await response.blob());
    try{
      const canvas=document.createElement('canvas');canvas.width=image.width;canvas.height=image.height;
      const ctx=canvas.getContext('2d');ctx.drawImage(image,0,0);
      return {width:image.width,height:image.height,pixels:ctx.getImageData(0,0,image.width,image.height).data,format:'rgba'};
    }finally{image.close();}
  };
}

// Each execution gets an opaque frame and a worker. Only image refs on connected
// ports can be read by the host. Node source never executes in the editor.
function runProgram(program,input,inputs={},readFrame=null,native=null){
  return new Promise((resolve,reject)=>{
    const frame=document.createElement('iframe');frame.hidden=true;frame.sandbox='allow-scripts';
    let timer;
    const finish=(fn,v)=>{clearTimeout(timer);window.removeEventListener('message',receive);frame.remove();fn(v);};
    const renew=()=>{clearTimeout(timer);timer=setTimeout(()=>finish(reject,Error('Node code timed out.')),30000);};
    async function receive(e){
      if(e.source!==frame.contentWindow)return;
      if(e.data?.read||e.data?.compute){
        try{
          const id=e.data.read||e.data.compute;
          let value,transfer=[];
          if(e.data.compute){
            if(!native)throw Error('This node has no native operator.');
            clearTimeout(timer);
            value=await native.execute(e.data.settings);
          }else{
            if(!readFrame)throw Error('No connected image stream.');
            value=await readFrame(e.data.ref);transfer=[value.pixels.buffer];
          }
          if(!frame.isConnected)return;
          renew();frame.contentWindow.postMessage({reply:id,value},'*',transfer);
        }catch(err){
          if(frame.isConnected){renew();frame.contentWindow.postMessage({reply:e.data.read||e.data.compute,error:String(err.message)},'*');}
        }
        return;
      }
      if(e.data?.ok)finish(resolve,e.data.value);
      else finish(reject,Error(e.data?.error||'Node execution failed.'));
    }
    const workerMain = function(){
      let seq=0;const waiting=new Map();
      const readFrame=ref=>ref?.pixels?Promise.resolve(ref):new Promise((resolve,reject)=>{
        waiting.set(++seq,{resolve,reject});postMessage({read:seq,ref});
      });
      const runNative=settings=>new Promise((resolve,reject)=>{
        waiting.set(++seq,{resolve,reject});postMessage({compute:seq,settings});
      });
      onmessage=async({data})=>{
        if(data.reply){
          const p=waiting.get(data.reply);if(!p)return;waiting.delete(data.reply);
          if(data.error)p.reject(Error(data.error));else p.resolve(data.value);return;
        }
        try{postMessage({ok:true,value:await processNode(data.input,data.inputs,readFrame,runNative,data.settings)});}
        catch(e){postMessage({ok:false,error:String(e.message||e)});}
      };
    };
    function frameMain(){
      let worker;
      onmessage=({source,data})=>{
        if(source!==parent)return;
        if(data.reply){worker?.postMessage(data,data.value?.pixels?[data.value.pixels.buffer]:[]);return;}
        const url=URL.createObjectURL(new Blob([data.source],{type:'text/javascript'}));
        worker=new Worker(url);worker.onmessage=e=>parent.postMessage(e.data,'*');
        worker.onerror=e=>parent.postMessage({ok:false,error:e.message},'*');
        worker.postMessage({input:data.input,inputs:data.inputs,settings:data.settings});
      };
    }
    const source='async function processNode(input,inputs,readFrame,runNative,settings){\n'+program+'\n}\n('+workerMain.toString()+')();';
    window.addEventListener('message',receive);renew();
    frame.onload=()=>frame.contentWindow.postMessage({source,input,inputs,settings:native?.settings},'*');
    frame.srcdoc='<meta http-equiv="Content-Security-Policy" content="default-src \'none\'; script-src \'unsafe-inline\' blob:; worker-src blob:; connect-src \'none\'"><script>('+frameMain.toString()+')();<\/script>';
    document.body.append(frame);
  });
}

// Any array of records can use the same row inspector and line plot.
function recordDisplay(host,data,config={},save=()=>{}){
  const tables=Array.isArray(data)?{'':data}:data||{};
  const form=document.createElement('div'),content=document.createElement('div');
  form.className='controls record-controls';host.replaceChildren(form,content);
  function choice(title,items,value,change){
    const label=document.createElement('label'),select=document.createElement('select');
    for(const [key,text] of items)select.add(new Option(text,key));
    if(items.some(([key])=>key===value))select.value=value;
    label.append(title+' ',select);form.append(label);select.onchange=()=>change(select.value);return select.value;
  }
  function set(key,value){Object.assign(config,{[key]:value});save();render();}
  function render(){
    form.replaceChildren();content.replaceChildren();
    const names=Object.keys(tables);
    const table=names.length&&names[0]!==''?choice('Table',names.map(k=>[k,k]),config.table,v=>set('table',v)):'';
    const rows=tables[table]||[],count=document.createElement('small');
    count.textContent=rows.length.toLocaleString()+' rows';form.append(count);
    if(!rows.length)return;
    const mode=choice('Display',[['rows','Records'],['plot','Line plot']],config.mode,v=>set('mode',v));
    if(mode==='rows'){
      const row=document.createElement('input'),out=document.createElement('pre');
      row.type='number';row.min=1;row.max=rows.length;row.value=1;row.setAttribute('aria-label','Record number');
      row.oninput=()=>{out.textContent=JSON.stringify(rows[row.valueAsNumber-1]??null,null,2);};
      form.append(row);content.append(out);row.oninput();return;
    }
    const numeric=new Set();
    function visit(value,path=''){
      if(Number.isFinite(value))numeric.add(path);
      else if(value&&typeof value==='object')for(const [k,v] of Object.entries(value))visit(v,path?path+'.'+k:k);
    }
    for(const row of rows)visit(row);
    const fields=Array.from(numeric,k=>[k,k]);
    if(!fields.length){content.textContent='No numeric fields in this table.';return;}
    const x=choice('X',[['','Row number'],...fields],config.x,v=>set('x',v));
    const y=choice('Y',fields,config.y,v=>set('y',v));
    const at=(r,k)=>k.split('.').reduce((v,p)=>v?.[p],r);
    const xy=rows.map((r,i)=>[x?at(r,x):i+1,at(r,y)]);
    const plot=document.createElement('canvas');plot.width=800;plot.height=160;plot.className='record-plot';
    plot.setAttribute('aria-label',y+' by '+(x||'row number'));content.append(plot);
    let xmin=Infinity,xmax=-Infinity,ymin=Infinity,ymax=-Infinity;
    for(const [a,b] of xy)if(Number.isFinite(a)&&Number.isFinite(b)){
      xmin=Math.min(xmin,a);xmax=Math.max(xmax,a);ymin=Math.min(ymin,b);ymax=Math.max(ymax,b);
    }
    if(!Number.isFinite(ymin)){content.textContent='No paired numeric values.';return;}
    const ctx=plot.getContext('2d');ctx.strokeStyle='#a5efd1';ctx.lineWidth=2;ctx.beginPath();
    let connected=false;
    for(const [a,b] of xy){
      if(!Number.isFinite(a)||!Number.isFinite(b)){connected=false;continue;}
      const px=64+728*(a-xmin)/(xmax-xmin||1),py=135-125*(b-ymin)/(ymax-ymin||1);
      if(connected)ctx.lineTo(px,py);else ctx.moveTo(px,py);connected=true;
    }
    ctx.stroke();ctx.fillStyle='#bdd1d2';ctx.font='14px monospace';
    ctx.fillText(ymax.toPrecision(4),2,14);ctx.fillText(ymin.toPrecision(4),2,136);
    ctx.fillText(String(xmin),64,157);ctx.textAlign='right';ctx.fillText(String(xmax),792,157);
  }
  render();
}

function extraInputs(node){
  const host=$('extraInputs');host.replaceChildren();host.hidden=node.type!=='code';
  if(host.hidden)return;
  const list=node.settings.extra||[];
  function edit(value){node.settings.extra=value;markGraphEdited();renderGraph();draw();}
  for(const port of list){
    const row=document.createElement('div'),remove=document.createElement('button');
    row.textContent='inputs.'+port.name+' ← '+nodeLabel(nodeById(port.source))+' · '+port.type+' ';
    remove.textContent='Remove';remove.onclick=()=>edit(list.filter(p=>p!==port));row.append(remove);host.append(row);
  }
  const select=document.createElement('select'),name=document.createElement('input'),button=document.createElement('button');
  name.placeholder='Input name';name.setAttribute('aria-label','Additional input name');
  select.setAttribute('aria-label','Additional input source');
  for(const source of nodeGraph.nodes){
    if(dependsOn(source,node.id))continue;
    for(const type of produced(source).filter(t=>Object.hasOwn(codeTypes,t)))
      select.add(new Option(nodeLabel(source)+' · '+type,JSON.stringify({source:source.id,type})));
  }
  button.textContent='Add input';button.disabled=!select.length;button.onclick=()=>{
    const alias=name.value.trim();
    if(!/^[a-zA-Z_][a-zA-Z0-9_]*$/.test(alias)||list.some(p=>p.name===alias)){
      name.setCustomValidity('Enter a unique input name using letters, digits or underscores.');name.reportValidity();return;
    }
    edit([...list,{...JSON.parse(select.value),name:alias}]);
  };
  name.oninput=()=>name.setCustomValidity('');host.append(name,select,button);
}

function tofPixelPreset(){
  // This body is the program in the preset and can be edited in the node.
  async function correctPixels(input, inputs, readFrame){
    const {map,tables}=inputs;
    if(map.units!=='m')throw Error('This preset needs camera positions in metres.');
    // Approximate a centred, forward-facing beam. Correct only its local patch.
    const maxTimeMs=40, maxResidualM=0.20, pixelRadius=2;
    const poses=new Map(map.poses.map(p=>[p.frame,p]));
    const stamps=new Map((tables['timestamps.csv']||[]).map(r=>[r.frame,r]));
    const ranges=(tables['range.csv']||[]).filter(r=>Number.isFinite(r.pi_read_end_ns)).sort((a,b)=>a.pi_read_end_ns-b.pi_read_end_ns);
    const rows=[];
    let k=0;
    for(const ref of input){
      const pose=poses.get(ref.frame),stamp=stamps.get(ref.frame);
      if(!pose?.rotation||!stamp)continue;
      const t=stamp.pi_encoder_callback_ns;
      if(!Number.isFinite(t))continue;
      while(k+1<ranges.length&&Math.abs(ranges[k+1].pi_read_end_ns-t)<Math.abs(ranges[k].pi_read_end_ns-t))k++;
      const sample=ranges[k]||{pi_read_end_ns:stamp.pi_tof_read_end_ns,tof_raw:stamp.tof_raw};
      if(!Number.isFinite(sample.pi_read_end_ns)||Math.abs(sample.pi_read_end_ns-t)>maxTimeMs*1e6||!(sample.tof_raw>0&&sample.tof_raw<250))continue;
      const {width,height,pixels}=await readFrame(ref);
      const cx=Math.floor(width/2),cy=Math.floor(height/2),f=width*64/96;
      const centre=pixels[cy*width+cx],rangeM=sample.tof_raw/104;
      if(!(centre>0)||Math.abs(rangeM-centre)>maxResidualM)continue;
      for(let y=cy-pixelRadius;y<=cy+pixelRadius;y++)for(let x=cx-pixelRadius;x<=cx+pixelRadius;x++){
        const raw=pixels[y*width+x];
        if(!(raw>0)||Math.abs(raw-centre)>0.05)continue;
        const z=raw+rangeM-centre;
        if(z<=0)continue;
        const ray=[(x+.5-width/2)*z/f,(y+.5-height/2)*z/f,z];
        const xyz=pose.rotation.map((axis,i)=>pose.xyz[i]+axis.reduce((sum,v,j)=>sum+v*ray[j],0));
        rows.push({frame:ref.frame,x,y,depth_m:raw,range_m:rangeM,corrected_m:z,xyz});
      }
    }
    return rows;
  }
  const program='return ('+correctPixels.toString()+')(input, inputs, readFrame);';
  return {graph:{selected:'viewer',viewer:'viewer',nodes:[
    {id:'walk',type:'walk',settings:{}},
    {id:'pose',type:'pose',input:'walk',settings:{}},
    {id:'depth',type:'depth',input:'walk',settings:{geometrySource:'apple'}},
    {id:'correct',type:'code',input:'depth',settings:{name:'Correct beam pixels',inputType:'Depth frames',outputType:'JSON records',program,
      extra:[{name:'map',source:'pose',type:'Camera map'},{name:'tables',source:'walk',type:'Recording tables'}]}},
    {id:'dots',type:'code',input:'correct',settings:{name:'Corrected dots',inputType:'JSON records',outputType:'Colored points',program:'return input.map(row => [...row.xyz, 110, 226, 192]);'}},
    {id:'viewer',type:'viewer',settings:{inputs:[{source:'dots',type:'Colored points'}]}}
  ]}};
}

function sameData(left,right){
  if(Object.is(left,right))return true;
  if(!left||!right||typeof left!=='object'||typeof right!=='object'||left.constructor!==right.constructor)return false;
  if(ArrayBuffer.isView(left))return left.length===right.length&&left.every((v,i)=>Object.is(v,right[i]));
  const keys=Object.keys(left);
  return keys.length===Object.keys(right).length&&keys.every(k=>Object.hasOwn(right,k)&&sameData(left[k],right[k]));
}

function displayPixels(host,frames){
  if(!frames?.length){host.textContent='No pixel frames available.';return;}
  const selector=document.createElement('input'),canvas=document.createElement('canvas'),caption=document.createElement('small');
  selector.type='number';selector.min=1;selector.max=frames.length;selector.setAttribute('aria-label','Image number');
  selector.value=Math.max(0,frames.findIndex(f=>f.frame===timeline[cursor]?.id))+1;
  host.append(selector,caption,canvas);
  function render(){
    const f=frames[selector.valueAsNumber-1];if(!f)return;
    const {pixels,width,height,format}=f;
    canvas.width=width;canvas.height=height;canvas.className='pixel-preview';
    caption.textContent=' / '+frames.length+' · '+(f.frame||'Image')+' · '+width+' × '+height;
    const ctx=canvas.getContext('2d'),image=ctx.createImageData(width,height);
    if(format==='rgba')image.data.set(pixels);
    else{
      let lo=Infinity,hi=-Infinity;
      for(const n of pixels)if(Number.isFinite(n)){lo=Math.min(lo,n);hi=Math.max(hi,n);}
      for(let i=0;i<pixels.length;i++){
        const shade=255*(pixels[i]-lo)/(hi-lo||1);
        image.data.set([shade,shade,shade,Number.isFinite(pixels[i])?255:0],4*i);
      }
    }
    ctx.putImageData(image,0,0);
    canvas.onclick=e=>{
      const b=canvas.getBoundingClientRect(),x=Math.floor((e.clientX-b.left)*width/b.width),y=Math.floor((e.clientY-b.top)*height/b.height);
      const i=y*width+x;
      caption.textContent=' ('+x+', '+y+') · '+(format==='rgba'?Array.from(pixels.subarray(i*4,i*4+4)).join(', '):pixels[i]);
    };
  }
  selector.oninput=render;render();
}
