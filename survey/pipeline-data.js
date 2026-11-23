function isTableType(type){return ['table','tables'].includes(catalog.types[type]?.display);}
function isImageType(type){return ['frames','pixels'].includes(catalog.types[type]?.display);}

let recordedTablesRun, recordedTablesRequest;
function readTables(run){
  if(run!==recordedTablesRun||!recordedTablesRequest){
    recordedTablesRun=run;
    recordedTablesRequest=fetch(API+'/recording-tables?'+new URLSearchParams({run})).then(async response=>{
      if(!response.ok)throw Error('Could not read the recording tables.');
      return response.json();
    }).catch(error=>{if(run===recordedTablesRun)recordedTablesRequest=null;throw error;});
  }
  return recordedTablesRequest;
}
function pixelReader(refs,run,index){
  const permitted=new Set(refs.map(f=>JSON.stringify([f.frame,f.method||''])));
  return async ref=>{
    if(!ref||!permitted.has(JSON.stringify([ref.frame,ref.method||''])))throw Error('Choose a frame from a connected image port.');
    const query=new URLSearchParams({run,index,frame:ref.frame,method:ref.method||''});
    const response=await fetch((ref.method?API+'/frame-depth.bin?':API+'/recorded.jpg?')+query);
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

function viewerPlotTools(){
  return {
    isFramePort:port=>port.isFrame,
    replayFrame:port=>({draw:"replayFrames",ports:[port]}),
    compareFrames:(left,right)=>({draw:"replayFrames",ports:[left,right]}),
    points:rows=>({draw:'points',rows}),
    line:vertices=>({draw:'line',vertices}),
    path:port=>({...port,draw:'path'}),
    splats:values=>({draw:'splats',values}),
    polygons:faces=>({draw:'polygons',faces}),
    layer:port=>port
  };
}

// Each execution gets an opaque frame and a worker. Only image refs on connected
// ports can be read by the host. Node source never executes in the editor.
function runProgram(program,input,inputs={},readFrame=null,native=null,replay=null,onLog=null,settings={}){
  return new Promise((resolve,reject)=>{
    const frame=document.createElement('iframe');frame.hidden=true;frame.sandbox='allow-scripts';
    let timer;
    const finish=(fn,v)=>{clearTimeout(timer);window.removeEventListener('message',receive);frame.remove();fn(v);};
    const renew=()=>{clearTimeout(timer);timer=setTimeout(()=>finish(reject,Error('Node code timed out.')),30000);};
    async function receive(e){
      if(e.source!==frame.contentWindow)return;
      if(e.data?.consoleEntry){onLog?.(e.data.consoleEntry);return;}
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
        if(data.debug){
          for(const name of ['log','info','warn','error','dir','table','debug']){
            console[name]=(...values)=>postMessage({consoleEntry:[name,...values.map(v=>typeof v==="function"||typeof v==="symbol"?String(v):v)]});
          }
        }
        try{postMessage({ok:true,value:await processNode(data.input,data.inputs,readFrame,runNative,data.settings,data.replay)});}
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
        worker.postMessage({input:data.input,inputs:data.inputs,settings:data.settings,replay:data.replay,debug:data.debug});
      };
    }
    const source='const plot=('+viewerPlotTools.toString()+')();\nasync function processNode(input,inputs,readFrame,runNative,settings,replay){\n'+program+'\n}\n('+workerMain.toString()+')();';
    window.addEventListener('message',receive);renew();
    frame.onload=()=>frame.contentWindow.postMessage({source,input,inputs,replay,debug:!!onLog,settings:native?.settings??settings},'*');
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
    const mode=choice('Display',[['rows','Records'],['plot','Line plot'],['scatter','Scatter plot']],config.mode,v=>set('mode',v));
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
    const y=choice('Y',[...fields,['','Row number']],config.y,v=>set('y',v));
    const at=(r,k)=>k.split('.').reduce((v,p)=>v?.[p],r);
    const xy=rows.map((r,i)=>[x?at(r,x):i+1,y?at(r,y):i+1]);
    const plot=document.createElement('canvas');plot.width=800;plot.height=160;plot.className='record-plot';
    plot.setAttribute('aria-label',(y||'row number')+' by '+(x||'row number'));content.append(plot);
    let xmin=Infinity,xmax=-Infinity,ymin=Infinity,ymax=-Infinity;
    for(const [a,b] of xy)if(Number.isFinite(a)&&Number.isFinite(b)){
      xmin=Math.min(xmin,a);xmax=Math.max(xmax,a);ymin=Math.min(ymin,b);ymax=Math.max(ymax,b);
    }
    if(!Number.isFinite(ymin)){content.textContent='No paired numeric values.';return;}
    function bounds(axis,autoMin,autoMax){
      return ['min','max'].map((end,i)=>{
        const key=axis+end, label=document.createElement('label'),field=document.createElement('input');
        field.type='number';field.step='any';field.placeholder='Auto';field.style.width='6em';
        field.value=config[key]??'';label.append(axis.toUpperCase()+' '+end+' ',field);form.append(label);
        field.onchange=()=>set(key,field.value===''?null:field.valueAsNumber);
        return config[key]??(i?autoMax:autoMin);
      });
    }
    [xmin,xmax]=bounds('x',xmin,xmax);[ymin,ymax]=bounds('y',ymin,ymax);
    if(xmin>xmax||ymin>ymax){content.textContent='Each axis minimum must be below its maximum.';return;}
    const ctx=plot.getContext('2d');ctx.save();ctx.beginPath();ctx.rect(64,10,728,125);ctx.clip();
    ctx.strokeStyle=ctx.fillStyle='#a5efd1';ctx.lineWidth=2;ctx.beginPath();
    let connected=false;
    for(const [a,b] of xy){
      if(!Number.isFinite(a)||!Number.isFinite(b)){connected=false;continue;}
      const px=64+728*(a-xmin)/(xmax-xmin||1),py=135-125*(b-ymin)/(ymax-ymin||1);
      if(mode==='scatter'){ctx.fillRect(px-1,py-1,2,2);continue;}
      if(connected)ctx.lineTo(px,py);else ctx.moveTo(px,py);connected=true;
    }
    ctx.stroke();ctx.restore();ctx.fillStyle='#bdd1d2';ctx.font='14px monospace';
    ctx.fillText(ymax.toPrecision(4),2,14);ctx.fillText(ymin.toPrecision(4),2,136);
    ctx.fillText(xmin.toPrecision(4),64,157);ctx.textAlign='right';ctx.fillText(xmax.toPrecision(4),792,157);
  }
  render();
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
    paintFramePixels(canvas,f);
    caption.textContent=' / '+frames.length+' · '+(f.frame||'Image')+' · '+width+' × '+height;
    canvas.onclick=e=>{
      const b=canvas.getBoundingClientRect(),x=Math.floor((e.clientX-b.left)*width/b.width),y=Math.floor((e.clientY-b.top)*height/b.height);
      const i=y*width+x;
      caption.textContent=' ('+x+', '+y+') · '+(format==='rgba'?Array.from(pixels.subarray(i*4,i*4+4)).join(', '):pixels[i]);
    };
  }
  selector.oninput=render;render();
}

function paintFramePixels(canvas,{pixels,width,height,format}){
    canvas.width=width;canvas.height=height;canvas.className='pixel-preview';

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
}

// Outputs of code nodes must match their declared type.
// How a type is drawn or listed comes from the server's type catalogue.
function displayOf(type){return catalog.types[type]?.display;}
// Outputs of code nodes must match the shape their declared type displays as.
function validateTransform(type,value){
  const invalid=()=>{throw Error('Invalid '+type+' output. '+(catalog.types[type]?.description||''));};
  const vector=(v,n)=>Array.isArray(v)&&v.length===n&&v.every(Number.isFinite);
  const isRecord=v=>v!==null&&typeof v==='object'&&Object.getPrototypeOf(v)===Object.prototype;
  const json=v=>{
    const ancestors=new Set();
    (function check(v){
      if(v===null||typeof v==='string'||typeof v==='boolean'||Number.isFinite(v))return;
      if((!Array.isArray(v)&&!isRecord(v))||ancestors.has(v))invalid();
      ancestors.add(v);for(const child of Object.values(v))check(child);ancestors.delete(v);
    })(v);
  };
  const kind=displayOf(type);
  if(kind==='splats'){
    if(!(value instanceof Float32Array)||value.length%16!==0||!value.every(Number.isFinite))invalid();
    for(let i=0;i<value.length;i+=16)
      if(value[i+3]<0||value[i+3]>1||value.subarray(i+4,i+7).some(v=>v<0||v>1)||value.subarray(i+8,i+11).some(v=>v<0))invalid();
    return;
  }
  if(kind==='map'){
    if(!isRecord(value)||!Array.isArray(value.points)||!Array.isArray(value.poses))invalid();
    validateTransform('Colored points',value.points);
    if(!value.poses.every(p=>vector(p.xyz,3)&&typeof p.frame==='string'))invalid();
    return;
  }
  if(kind==='tables'){
    if(!isRecord(value)||!Object.values(value).every(rows=>Array.isArray(rows)&&rows.every(isRecord)))invalid();
    json(value);return;
  }
  if(kind==='report'){json(value);return;}
  if(!Array.isArray(value))invalid();
  if(kind==='pixels'&&!value.every(f=>f&&typeof f.frame==='string'&&Number.isSafeInteger(f.width)&&f.width>0&&Number.isSafeInteger(f.height)&&f.height>0&&
      (f.format==='depth'&&f.pixels instanceof Float32Array&&f.pixels.length===f.width*f.height||
       f.format==='rgba'&&f.pixels instanceof Uint8ClampedArray&&f.pixels.length===4*f.width*f.height)))invalid();
  if(kind==='points'&&!value.every(p=>vector(p,6)&&p.slice(3).every(v=>v>=0&&v<=255)))invalid();
  if(kind==='triangles'&&!value.every(t=>Array.isArray(t)&&t.length===3&&t.every(v=>vector(v,3))))invalid();
  if(kind==='polygons'&&!value.every(f=>f&&Array.isArray(f.vertices)&&f.vertices.length>=3&&f.vertices.every(v=>vector(v,3))))invalid();
  if(kind==='path'&&!value.every(p=>p&&vector(p.xyz,3)))invalid();
  if(kind==='frames'&&!value.every(f=>f&&typeof f.frame==='string'))invalid();
  if(kind==='table'){if(!value.every(isRecord))invalid();json(value);}
}

function inspectConsoleValue(label,value,depth=0,parents=new Set()){
  const simple=text=>{
    const row=document.createElement('div');
    row.textContent=label+'  '+text;
    return row;
  };
  if(value===null||typeof value!=='object')return simple(typeof value==='string'?JSON.stringify(value):String(value));
  if(parents.has(value))return simple('[Circular reference]');
  if(value instanceof Date||value instanceof RegExp||value instanceof Error)return simple(String(value));
  const identity=value;
  if(value instanceof Map||value instanceof Set)value=Array.from(value);
  if(value instanceof ArrayBuffer)value=new Uint8Array(value);
  if(value instanceof DataView)value=new Uint8Array(value.buffer,value.byteOffset,value.byteLength);

  const indexed=Array.isArray(value)||ArrayBuffer.isView(value);
  const keys=indexed?null:Object.keys(value),length=indexed?value.length:keys.length;
  const box=document.createElement('details'),header=document.createElement('summary'),children=document.createElement('div');
  header.textContent=label+'  '+(indexed?(Array.isArray(value)?'Array':value.constructor.name)+'('+length+')':'{'+length+' fields}');
  children.style.paddingLeft='1.2em';
  box.append(header,children);
  const pageSize=50;
  function populate(){
    if(children.childElementCount)return;
    const body=document.createElement('div'),ancestors=new Set(parents).add(identity);
    function page(n){
      body.replaceChildren();
      for(let i=n*pageSize;i<Math.min(length,(n+1)*pageSize);i++){
        const key=indexed?i:keys[i];
        body.append(inspectConsoleValue(String(key),value[key],depth+1,ancestors));
      }
    }
    if(length>pageSize){
      const control=document.createElement('label'),field=document.createElement('input');
      field.type='number';field.min=1;field.max=Math.ceil(length/pageSize);field.value=1;
      control.append('Page ',field,' of '+field.max+' · '+pageSize+' entries per page');
      field.oninput=()=>{if(field.value&&field.validity.valid)page(field.valueAsNumber-1);};
      children.append(control);
    }
    children.append(body);page(0);
  }
  box.ontoggle=()=>{if(box.open)populate();};
  if(depth===0){box.open=true;populate();}
  return box;
}
