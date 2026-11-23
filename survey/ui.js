'use strict';
const $ = id => document.getElementById(id);
// Links the browser follows itself: the map download on the companion, and the checkerboard page beside this one.
$('download').href=API+'/survey.json?download=1';
$('targetLink').href='target.html'+location.search;
let state, version = -1, model, active = false, yaw = -.38, pitch = .42, zoom = 1;
let replayMark=null, orbitPerspective=false;
let droneView=false, droneZoom=1, droneYaw=0, dronePitch=0;
let posesByFrame=new Map();
let lastSourceWalk=null;
let timeline=[], timelineRun, replay=false, cursor=0, animation=0, displayedFrame;
let routeSegments=[], replayEstimates=new Map(), replayDirections=new Map();
const routeStyles={
  mapped:{color:'#a5efd1',dash:[]},
  interpolated:{color:'#f7cc7f',dash:[7,5]},
  extrapolated:{color:'#cfb1f7',dash:[2,5]}
};
let gapRanges=[], gapRequest=0;
let estimate=null, tracking=false, estimating=false, generation=0;
let centre = [0,0,0], extent = 1, panX = 0, panY = 0, drag;
const canvas = $('map'), ctx = canvas.getContext('2d');
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const normalized=v=>v.map(x=>x/Math.hypot(...v));

const identityLevel=()=>[[1,0,0],[0,1,0],[0,0,1]];
const dot3=(a,b)=>a.reduce((sum,v,i)=>sum+v*b[i],0);
const columns=m=>m[0].map((_,i)=>m.map(row=>row[i]));
const multiply=(a,b)=>a.map(row=>columns(b).map(col=>dot3(row,col)));
let correction=identityLevel(),editingLevel=false,ringAxes=null,ringDrag=null;
const levelKey=m=>'room-survey-level:'+m.run+':'+(m.feature_index||'preview');
function loadCorrection(m){
  correction=identityLevel();
  try{
    const saved=m&&JSON.parse(localStorage.getItem(levelKey(m)));
    if(Array.isArray(saved)&&saved.length===3&&saved.every(r=>Array.isArray(r)&&r.length===3&&r.every(Number.isFinite)))correction=saved;
  }catch{}
}
function saveCorrection(){
  if(model)localStorage.setItem(levelKey(model),JSON.stringify(correction));
}
function editOrientation(on){
  editingLevel=on;
  $('levelRings').toggleAttribute('hidden',!on);$('levelTools').hidden=!on;
  $('editLevel').setAttribute('aria-pressed',on);
  if(on){droneView=false;updateViewControls();}
  draw();
}
function rotateLevel(axis,angle,base){
  const r=identityLevel(),j=(axis+1)%3,k=(axis+2)%3;
  r[j][j]=r[k][k]=Math.cos(angle);
  r[j][k]=-Math.sin(angle);r[k][j]=Math.sin(angle);
  correction=multiply(r,base);draw();
}
const rings=['#ed645b','#56bf74','#669cff'].map((color,axis)=>{
  const ring=document.createElementNS('http://www.w3.org/2000/svg','path');
  ring.setAttribute('stroke',color);ring.setAttribute('tabindex','0');ring.setAttribute('role','slider');
  ring.setAttribute('aria-label','Rotate around '+'XYZ'[axis]+' axis');
  const title=document.createElementNS(ring.namespaceURI,'title');
  title.textContent='Drag to rotate about '+'XYZ'[axis]+' · arrow keys adjust 1°';ring.append(title);
  $('levelRings').append(ring);
  ring.onkeydown=e=>{
    if(!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown'].includes(e.key))return;
    e.preventDefault();rotateLevel(axis,(['ArrowRight','ArrowUp'].includes(e.key)?1:-1)*Math.PI/180,correction);saveCorrection();
  };
  ring.onpointerdown=e=>{
    if(e.button!==0||!ringAxes)return;
    const box=$('levelRings').getBoundingClientRect(),scale=box.width/200;
    const x=(e.clientX-box.left-box.width/2)/scale,y=(e.clientY-box.top-box.height/2)/scale;
    const points=ringCurve(axis,ringAxes);
    const index=points.reduce((best,v,i)=>Math.hypot(v[0]-x,v[1]-y)<Math.hypot(points[best][0]-x,points[best][1]-y)?i:best,0);
    const a=points[(index+71)%72],b=points[(index+1)%72];
    let dx=b[0]-a[0],dy=b[1]-a[1],n=Math.hypot(dx,dy);
    if(n<.1){dx=1;dy=0;n=1;}
    ringDrag={id:e.pointerId,x:e.clientX,y:e.clientY,dx:dx/n,dy:dy/n,scale,base:correction};
    ring.setPointerCapture(e.pointerId);e.preventDefault();
  };
  ring.onpointermove=e=>{
    const d=ringDrag;if(!d||d.id!==e.pointerId)return;
    rotateLevel(axis,((e.clientX-d.x)*d.dx+(e.clientY-d.y)*d.dy)/(76*d.scale),d.base);
  };
  ring.onpointerup=ring.onpointercancel=ring.onlostpointercapture=()=>{if(ringDrag){ringDrag=null;saveCorrection();}};
  return ring;
});
function ringCurve(axis,axes){
  const j=(axis+1)%3,k=(axis+2)%3;
  return Array.from({length:72},(_,i)=>{
    const t=i*Math.PI/36;
    return axes.slice(0,2).map(row=>76*(row[j]*Math.cos(t)+row[k]*Math.sin(t)));
  });
}
$('editLevel').onclick=()=>editOrientation(!editingLevel);
$('levelDone').onclick=()=>editOrientation(false);
$('levelUndo').onclick=()=>{correction=identityLevel();saveCorrection();draw();};

function cameraAxes(pose){
  if(pose?.rotation)return pose.rotation[0].map((_,i)=>pose.rotation.map(row=>row[i]));
  if(!pose?.forward)return null;
  const forward=normalized(pose.forward);
  const right=normalized(cross(forward,Math.abs(forward[2])>.999?[0,1,0]:[0,0,1]));
  return [right,cross(forward,right),forward];
}

function orientationQuaternion([[xx,yx,zx],[xy,yy,zy],[xz,yz,zz]]){
  const diagonal=[1+xx+yy+zz,1+xx-yy-zz,1-xx+yy-zz,1-xx-yy+zz];
  const major=diagonal.indexOf(Math.max(...diagonal));
  const scale=2*Math.sqrt(diagonal[major]);
  const values=[
    [diagonal[0],zy-yz,xz-zx,yx-xy],
    [zy-yz,diagonal[1],xy+yx,xz+zx],
    [xz-zx,xy+yx,diagonal[2],yz+zy],
    [yx-xy,xz+zx,yz+zy,diagonal[3]]
  ];
  return normalized(values[major].map(v=>v/scale));
}

function quaternionAxes([w,x,y,z]){
  return [
    [1-2*(y*y+z*z),2*(x*y+w*z),2*(x*z-w*y)],
    [2*(x*y-w*z),1-2*(x*x+z*z),2*(y*z+w*x)],
    [2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)]
  ];
}

function blendDirection(a,b,t){
  if(!a||!b)return (a||b)?.axes;
  const sign=a.quat.reduce((sum,v,i)=>sum+v*b.quat[i],0)<0?-1:1;
  return quaternionAxes(normalized(a.quat.map((v,i)=>v*(1-t)+sign*b.quat[i]*t)));
}

function frameFraction(frames,index,left,right){
  const elapsed=frames[right].ms-frames[left].ms;
  return elapsed>0?(frames[index].ms-frames[left].ms)/elapsed:(index-left)/(right-left);
}

function buildReplayRoute(poses,frames,estimateEnds){
  const segments=[], positions=new Map(), directions=new Map();
  const order=new Map(frames.map((frame,i)=>[frame.id,i]));
  const knots=poses.map(p=>({pose:p,index:order.get(p.frame)}));
  for(let i=1;i<knots.length;i++){
    const left=knots[i-1],right=knots[i];
    const gap=(left.index!=null&&right.index!=null)
      ? right.index>left.index+1 : parseInt(right.pose.frame)>parseInt(left.pose.frame)+1;
    const kind=gap?'interpolated':'mapped';
    segments.push({kind,points:[left.pose.xyz,right.pose.xyz]});
    if(gap && left.index!=null && right.index!=null){
      for(let j=left.index+1;j<right.index;j++){
        const fraction=frameFraction(frames,j,left.index,right.index);
        positions.set(frames[j].id,{kind,xyz:left.pose.xyz.map((v,k)=>v+fraction*(right.pose.xyz[k]-v))});
      }
    }
  }

  const anchors=knots.filter(k=>k.index!=null);
  if(estimateEnds && anchors.length){
    for(const step of [1,-1]){
      const start=step===1;
      const anchorIndex=start?0:anchors.length-1;
      const anchor=anchors[anchorIndex];
      const end=start?0:frames.length-1;
      if(end===anchor.index)continue;
      // Use the span of up to four consecutive mapped frames for the end velocity.
      let reference=anchor;
      for(let offset=step;Math.abs(offset)<=3;offset+=step){
        const sample=anchors[anchorIndex+offset];
        if(!sample || sample.index!==anchor.index+offset)break;
        reference=sample;
      }
      // Fall back to the nearest distinct timestamp when the edge poses are sparse.
      if(frames[reference.index].ms===frames[anchor.index].ms){
        const neighbours=start?anchors:anchors.slice().reverse();
        reference=neighbours.find(sample=>frames[sample.index].ms!==frames[anchor.index].ms)||anchor;
      }
      const held=reference===anchor;
      for(let j=anchor.index-step;start?j>=end:j<=end;j-=step){
        const elapsed=frames[j].ms-frames[anchor.index].ms;
        const interval=frames[reference.index].ms-frames[anchor.index].ms;
        const xyz=anchor.pose.xyz.map((v,k)=>held?v:v+(reference.pose.xyz[k]-v)*elapsed/interval);
        positions.set(frames[j].id,{kind:'extrapolated',xyz,held});
      }
      if(!held)segments.push({kind:'extrapolated',points:[anchor.pose.xyz,positions.get(frames[end].id).xyz]});
    }
  }

  const oriented=anchors.flatMap(({pose,index})=>{
    const axes=cameraAxes(pose);
    return axes?[{index,axes,quat:orientationQuaternion(axes)}]:[];
  });
  if(oriented.length){
    let next=0;
    frames.forEach((frame,index)=>{
      while(next<oriented.length && oriented[next].index<index)next++;
      const left=oriented[next-1],right=oriented[next];
      if(right?.index===index)return;
      const fraction=left&&right?frameFraction(frames,index,left.index,right.index):0;
      directions.set(frame.id,blendDirection(left,right,fraction));
    });
  }
  return {segments,positions,directions};
}

// The path the viewer shows: a regenerated depth path, else the camera map's poses.
function displayedPoses(){
  const layers=scene.layers;
  return layers.find(l=>l.type==='Camera path')?.poses||layers.find(l=>l.type==='Camera map')?.poses||model?.poses||[];
}
function updateReplayRoute(){
  const recorded=model?.run===timelineRun && timeline.length>0;
  const frames=recorded?timeline:(model?.path_frames||model?.sampled_frames||[]).map((id,ms)=>({id,ms}));
  const route=buildReplayRoute(displayedPoses(),frames,recorded);
  routeSegments=route.segments;
  replayEstimates=route.positions;
  replayDirections=route.directions;
  updateRouteLegend();
}

function updateRouteLegend(){
  for(const kind of ['interpolated','extrapolated']){
    $('legend-'+kind).hidden=!$('route').checked || !routeSegments.some(segment=>segment.kind===kind);
  }
}

function droneCamera(){
  const id=timeline[cursor]?.id;
  const pose=posesByFrame.get(id);
  const estimated=replayEstimates.get(id);
  const xyz=pose?.xyz||estimated?.xyz;
  const savedAxes=cameraAxes(pose);
  const axes=savedAxes||replayDirections.get(id);
  if(!xyz||!axes)return null;
  const status=estimated
    ? estimated.held?'Estimated path · held at nearest mapped pose':
      estimated.kind==='interpolated'?'Estimated path · straight-line interpolation':'Estimated path · extrapolated end'
    : !savedAxes?'Camera direction estimated from nearby frames':!pose.rotation?'Camera roll estimated from map up':'';

  const [right,down,forward]=axes;
  const yawCos=Math.cos(droneYaw), yawSin=Math.sin(droneYaw);
  const turnRight=right.map((v,i)=>v*yawCos-forward[i]*yawSin);
  const turnForward=forward.map((v,i)=>v*yawCos+right[i]*yawSin);
  const pitchCos=Math.cos(dronePitch), pitchSin=Math.sin(dronePitch);
  return {xyz,status,axes:[
    turnRight,
    down.map((v,i)=>v*pitchCos-turnForward[i]*pitchSin),
    turnForward.map((v,i)=>v*pitchCos+down[i]*pitchSin)
  ]};
}

function clipNear(points, near, closed=true){
  return clipHeight(points,near,closed,true);
}

function clipHeight(points,height,closed,keepAbove,up=null){
  const elevation=p=>up?dot3(up,p):p[2];
  const inside=p=>keepAbove?elevation(p)>=height:elevation(p)<=height;
  const visible=[];
  if(!closed && inside(points[0]))visible.push(points[0]);
  for(let i=closed?0:1;i<points.length;i++){
    const a=points[(i+points.length-1)%points.length], b=points[i];
    if(inside(a)!==inside(b) && elevation(a)!==height && elevation(b)!==height){
      const t=(height-elevation(a))/(elevation(b)-elevation(a));
      visible.push(a.map((v,i)=>v+t*(b[i]-v)));
    }
    if(inside(b))visible.push(b);
  }
  return visible;
}

// Index the projected triangles by screen tile. Only faces covering a landmark's
// sightline can hide it. Smaller depth is closer in both projection modes.
class SurfaceVisibility {
  constructor(width,height){
    this.tile=32;
    this.cols=Math.ceil(width/this.tile);
    this.rows=Math.ceil(height/this.tile);
    this.bins=new Array(this.cols*this.rows);
    this.members=new Set();
  }

  addFace(screen,landmarks){
    for(const id of landmarks)this.members.add(id);
    // A triangle clipped at the drone camera's near plane can have four corners.
    for(let i=1;i+1<screen.length;i++){
      const [ax,ay,az]=screen[0], [bx,by,bz]=screen[i], [cx,cy,cz]=screen[i+1];
      const dx=bx-ax, dy=by-ay, ex=cx-ax, ey=cy-ay;
      const det=dx*ey-dy*ex;
      if(Math.abs(det)<1e-9)continue;
      const face={ax,ay,az,ux:ey/det,uy:-ex/det,vx:-dy/det,vy:dx/det,dz1:bz-az,dz2:cz-az};
      this.indexFace(face, [screen[0],screen[i],screen[i+1]]);
    }
  }

  indexFace(face, outline){
    let xmin=Infinity, xmax=-Infinity, ymin=Infinity, ymax=-Infinity;
    for(const [x,y] of outline){
      xmin=Math.min(xmin,x); xmax=Math.max(xmax,x);
      ymin=Math.min(ymin,y); ymax=Math.max(ymax,y);
    }
    const left=Math.max(0,Math.floor(xmin/this.tile)), right=Math.min(this.cols-1,Math.floor(xmax/this.tile));
    const top=Math.max(0,Math.floor(ymin/this.tile)), bottom=Math.min(this.rows-1,Math.floor(ymax/this.tile));
    for(let row=top;row<=bottom;row++){
      for(let col=left;col<=right;col++)(this.bins[row*this.cols+col]??=[]).push(face);
    }
  }

  addPolygon(rings,landmarks){
    for(const id of landmarks)this.members.add(id);
    const outer=rings[0], [ax,ay,az]=outer[0];
    // Use a wide projected triangle to determine the plane's screen-depth gradient.
    let b=outer[0], best=0;
    for(const v of outer){
      const d=(v[0]-ax)**2+(v[1]-ay)**2;
      if(d>best){best=d;b=v;}
    }
    const dx=b[0]-ax, dy=b[1]-ay, dz=b[2]-az;
    let det=0, c=b;
    for(const v of outer){
      const area=dx*(v[1]-ay)-dy*(v[0]-ax);
      if(Math.abs(area)>Math.abs(det)){det=area;c=v;}
    }
    if(Math.abs(det)<1e-9)return;
    const gx=(dz*(c[1]-ay)-dy*(c[2]-az))/det;
    const gy=(dx*(c[2]-az)-dz*(c[0]-ax))/det;
    this.indexFace({rings,ax,ay,az,gx,gy},outer);
  }

  hides([x,y,z]){
    const col=Math.floor(x/this.tile), row=Math.floor(y/this.tile);
    if(col<0 || row<0 || col>=this.cols || row>=this.rows)return false;
    const faces=this.bins[row*this.cols+col];
    if(!faces)return false;
    for(const f of faces){
      const dx=x-f.ax, dy=y-f.ay;
      if(f.rings){
        const depth=f.az+dx*f.gx+dy*f.gy;
        if(z-depth <= 1e-8*Math.max(1,Math.abs(z),Math.abs(depth)))continue;
        let inside=false;
        for(const ring of f.rings){
          for(let i=0,j=ring.length-1;i<ring.length;j=i++){
            const a=ring[i], b=ring[j];
            if((a[1]>y)!==(b[1]>y) && x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0])inside=!inside;
          }
        }
        if(inside)return true;
        continue;
      }
      const u=f.ux*dx+f.uy*dy, v=f.vx*dx+f.vy*dy;
      if(u < -1e-9 || v < -1e-9 || u+v > 1+1e-9)continue;
      const depth=f.az+u*f.dz1+v*f.dz2;
      // Retain landmarks lying on the face, including its own boundary vertices.
      if(z-depth > 1e-8*Math.max(1,Math.abs(z),Math.abs(depth)))return true;
    }
    return false;
  }
}

// Compare nearby visible depths to darken the receiving side of a crease.
// Fit depth across each face so tilting a flat wall does not shadow its triangles.
function paintContactShadows(context, patches, width, height) {
  if (!patches.length || !width || !height) return;
  const scale=Math.min(1,320/Math.max(width,height));
  const buffer=document.createElement('canvas');
  buffer.width=Math.max(1,Math.round(width*scale));
  buffer.height=Math.max(1,Math.round(height*scale));
  const raster=buffer.getContext('2d',{willReadFrequently:true});
  raster.scale(buffer.width/width,buffer.height/height);
  let lo=Infinity, hi=-Infinity;
  for(const {rings} of patches) for(const [, , z] of rings[0]){
    lo=Math.min(lo,z); hi=Math.max(hi,z);
  }
  if(!(hi>lo)) return;
  for(const {rings,path} of patches){
    const [a,b,...rest]=rings[0];
    const bx=b[0]-a[0], by=b[1]-a[1], bz=(b[2]-a[2])/(hi-lo);
    const level=(a[2]-lo)/(hi-lo);
    const gray=Math.round(level*255);
    raster.fillStyle=`rgb(${gray},${gray},${gray})`;
    for(const c of rest){
      const cx=c[0]-a[0], cy=c[1]-a[1], det=bx*cy-by*cx;
      if(Math.abs(det)<1e-5) continue;
      const cz=(c[2]-a[2])/(hi-lo);
      const gx=(bz*cy-by*cz)/det, gy=(bx*cz-bz*cx)/det;
      const norm=gx*gx+gy*gy;
      if(norm>1e-14){
        const dx=gx/norm, dy=gy/norm;
        const x=a[0]-dx*level, y=a[1]-dy*level;
        const ramp=raster.createLinearGradient(x,y,x+dx,y+dy);
        ramp.addColorStop(0,'black'); ramp.addColorStop(1,'white');
        raster.fillStyle=ramp;
      }
      break;
    }
    raster.fill(path,'evenodd');
  }
  const w=buffer.width,h=buffer.height;
  const pixels=raster.getImageData(0,0,w,h);
  const output=raster.createImageData(w,h);
  for(let y=0;y<h;y++) for(let x=0;x<w;x++){
    const i=4*(y*w+x);
    if(pixels.data[i+3]<245) continue;
    let shadow=0;
    for(let step=1;step<=4 && step<=x && step<=y;step++){
      const j=i-4*step*(w+1);
      if(pixels.data[j+3]<245) continue;
      const delta=pixels.data[i]-pixels.data[j];
      if(delta>3) shadow+=Math.min(1,(delta-3)/12)*(5-step);
    }
    output.data[i+3]=shadow*7;
  }
  raster.putImageData(output,0,0);
  context.drawImage(buffer,0,0,width,height);
}

function draw() {
  if(typeof viewerFrames!=='function')return;   // graph.js loads after this file
  const w=canvas.clientWidth, h=canvas.clientHeight, d=devicePixelRatio;
  canvas.width=w*d; canvas.height=h*d; ctx.setTransform(d,0,0,d,0,0);
  const status=$('droneStatus');
  status.hidden=true;
  $('vertexKey').hidden=true;
  $('recenterDrone').hidden=!droneView || (!droneYaw && !dronePitch);
  viewerFrames();
  if (!model&&!graphLayers().length) return;
  const cy=Math.cos(yaw), sy=Math.sin(yaw), cp=Math.cos(pitch), sp=Math.sin(pitch);
  const worldAxes=[[cy,-sy,0],[-sy*sp,-cy*sp,-cp],[sy*cp,cy*cp,-sp]];
  const orbitAxes=multiply(worldAxes,correction);
  if(editingLevel){
    ringAxes=worldAxes;
    rings.forEach((ring,i)=>ring.setAttribute('d','M'+ringCurve(i,worldAxes).map(p=>p.join(',')).join(' L')+' Z'));
  }
  const scale=Math.min(w,h)*zoom/extent;
  const focal=h*.5*(droneView?droneZoom:1)/Math.tan((droneView?75:45)*Math.PI/360);
  const origin=droneView?[w/2,h/2]:[w/2+panX,h/2+panY];
  const camera=droneView?droneCamera():orbitPerspective&&scale>0?
    {axes:orbitAxes,xyz:centre.map((v,i)=>v-orbitAxes[2][i]*focal/scale)}:null;
  const axes=camera?.axes||orbitAxes;
  updateAxisGizmo(droneView?multiply(axes,columns(correction)):worldAxes);
  if(droneView&&!viewBindings().some(p=>p.type==='Camera map')){droneView=false;updateViewControls();draw();return;}
  if(droneView && !camera){
    status.textContent='Camera pose unavailable for this frame';
    status.hidden=false;
    return;
  }
  if(camera?.status){
    status.textContent=camera.status;
    status.hidden=false;
  }
  const near=model?.units==='m'?.02:Math.max(1e-5,extent*.002);
  const cameraPoint=p=>camera.axes.map(([x,y,z])=>
    x*(p[0]-camera.xyz[0])+y*(p[1]-camera.xyz[1])+z*(p[2]-camera.xyz[2]));
  // Reciprocal depth varies linearly across a perspective triangle in screen space.
  const projectCamera=([x,y,z])=>[origin[0]+focal*x/z,origin[1]+focal*y/z,-1/z];
  const depth=p=>dot3(orbitAxes[2],p);
  const projectWithDepth=p=>{
    if(camera){
      const viewpoint=cameraPoint(p);
      return viewpoint[2]>=near?projectCamera(viewpoint):null;
    }
    const delta=p.map((v,i)=>v-centre[i]);
    return [origin[0]+dot3(orbitAxes[0],delta)*scale,origin[1]+dot3(orbitAxes[1],delta)*scale,depth(delta)];
  };
  const project=p=>projectWithDepth(p)?.slice(0,2);
  let visibility;
  const args=[ctx,w,h,camera?.xyz||centre,axes,origin,camera?focal:scale,!!camera,near];
  viewerFrames();
  const layers=graphLayers();
  $('mapempty').hidden=!!model||layers.length>0;
  const clouds=layers.filter(item=>item.points);
  const projected=[];
  for(const layer of layers){
    layer.splats?.paint(...args);
    if(layer.solid){
      layer.solid.paint(...args,!!layer.splats);
      if(clouds.length){
        visibility??=new SurfaceVisibility(w,h);
        layer.solid.indexVisibility(visibility,projectWithDepth,camera?cameraPoint:null,near,projectCamera);
      }
    }
    const mesh=layer.mesh;
    if(!mesh?.patches?.length)continue;
    if(clouds.length)visibility??=new SurfaceVisibility(w,h);
    const timed=mesh.patches.some(p=>p.first!=null);
    const frame=timed && replay && $('surfaceTime').checked && timeline[cursor]?.id;
    const visible=frame ? mesh.patches.filter(p=>p.first<=frame).concat(
      mesh.retired.filter(p=>p.first<=frame && frame<p.until)) : mesh.patches;
    const cutPercent=Number(layer.cut??0);
    if(mesh.heightRange && mesh.heightRange.up!==correction[2]){
      let min=Infinity,max=-Infinity;
      for(const face of mesh.patches)for(const v of face.vertices){
        const z=dot3(correction[2],v);min=Math.min(min,z);max=Math.max(max,z);
      }
      mesh.heightRange={min,max,up:correction[2]};
    }
    const cut=mesh.heightRange && cutPercent>0
      ? mesh.heightRange.max-(mesh.heightRange.max-mesh.heightRange.min)*cutPercent/100 : null;
    const patches=visible.flatMap(p=>{
      const outerRing=cut===null?p.vertices:clipHeight(p.vertices,cut,true,false,correction[2]);
      if(outerRing.length<3)return [];
      const rings=[outerRing,...(p.holes||[])];
      if(!camera)return [{p,rings:rings.map(r=>r.map(projectWithDepth)),
        z:outerRing.reduce((sum,v)=>sum+depth(v),0)/outerRing.length}];
      const clipped=rings.map(r=>clipNear(r.map(cameraPoint),near));
      const outer=clipped[0];
      if(outer.length<3)return [];
      return [{p,rings:clipped.filter(r=>r.length>=3).map(r=>r.map(projectCamera)),
        z:outer.reduce((sum,v)=>sum+v[2],0)/outer.length}];
    });
    for(const patch of patches)projected.push(patch);
  }
  if(projected.length){
    const patches=projected.sort((a,b)=>b.z-a.z);
    for(const patch of patches){
      const {p,rings}=patch;
      const [a,b,c]=p.vertices;
      const normal=p.normal||cross(b.map((v,i)=>v-a[i]),c.map((v,i)=>v-a[i]));
      const length=Math.hypot(...normal)||1;
      const [nx,ny,nz]=axes.map(axis=>axis.reduce((sum,v,i)=>sum+v*normal[i],0)/length);
      const eyeRay=camera?cameraPoint(a):[0,0,1];
      const sign=nx*eyeRay[0]+ny*eyeRay[1]+nz*eyeRay[2]>0?-1:1;
      const diffuse=Math.max(0,sign*(-.55*nx-.67*ny-.50*nz));
      const gray=Math.round(72+166*diffuse);
      ctx.fillStyle=`rgb(${gray},${gray},${gray})`;
      const path=new Path2D();
      for(const ring of rings){
        ring.forEach(([x,y],i)=>i?path.lineTo(x,y):path.moveTo(x,y));
        path.closePath();
      }
      ctx.fill(path,'evenodd');
      patch.path=path;
      if(p.holes)visibility?.addPolygon(rings,p.landmarks);
      else visibility?.addFace(rings[0],p.landmarks);
    }
    paintContactShadows(ctx,patches,w,h);
  }
  for(const layer of clouds){
    const vertices=[];
    layer.points.forEach((p,id)=>{
      const screen=projectWithDepth(p);
      if(!screen || visibility?.hides(screen))return;
      const [x,y]=screen;
      if(visibility?.members.has(id)){
        vertices.push(screen);
      }else{
        ctx.fillStyle='rgb('+p.slice(3).map(v=>Math.round(55+.78*v)).join(',')+')';
        ctx.fillRect(x-1,y-1,2,2);
      }
    });
    ctx.fillStyle='#ffffff';
    for(const [x,y] of vertices)ctx.fillRect(x-2,y-2,4,4);
    $('vertexKey').hidden=!visibility?.members.size;
  }
  for(const layer of layers)if(layer.line){
    ctx.beginPath();ctx.strokeStyle='#a5efd1';ctx.lineWidth=2;
    let connected=false;
    for(const point of layer.line){
      const xy=projectWithDepth(point);
      if(!xy){connected=false;continue;}
      if(connected)ctx.lineTo(xy[0],xy[1]);else ctx.moveTo(xy[0],xy[1]);
      connected=true;
    }
    ctx.stroke();
  }
  if(!camera && estimate?.xyz && viewBindings().some(p=>p.type==='Camera map')){
    const a=project(estimate.xyz), v=estimate.rotation.map(row=>row[2]);
    const b=project(estimate.xyz.map((x,i)=>x+v[i]*extent*.10));
    ctx.strokeStyle='#58baff';ctx.fillStyle='#58baff';ctx.lineWidth=3;
    ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();
    ctx.beginPath();ctx.arc(...a,6,0,Math.PI*2);ctx.fill();
  }
  const gaussian=!!activeGaussian;
  if(gaussian && $('tofEvidenceToggle').checked)drawTofEvidence(project,cameraPoint,projectCamera,camera,near);
  if(gaussian && !camera)drawClearanceEnvelope(project,scale);
  if(!$('route').checked||!viewBindings().some(p=>p.type==='Camera map')||!displayedPoses().length)return;
  ctx.lineWidth=2;
  for(const [kind,style] of Object.entries(routeStyles)){
    ctx.strokeStyle=style.color;
    ctx.setLineDash(style.dash);
    ctx.beginPath();
    for(const segment of routeSegments){
      if(segment.kind!==kind)continue;
      const screen=camera?clipNear(segment.points.map(cameraPoint),near,false).map(projectCamera):
        segment.points.map(project);
      if(screen.length===2){
        ctx.moveTo(screen[0][0],screen[0][1]);
        ctx.lineTo(screen[1][0],screen[1][1]);
      }
    }
    ctx.stroke();
  }
  ctx.setLineDash([]);
  for(const p of displayedPoses()){
    const screen=project(p.xyz);
    if(!screen)continue;
    const [x,y]=screen;
    const fraction=parseInt(p.frame)/Math.max(1,(state?.frames||1)-1);
    ctx.fillStyle='hsl('+(167-130*fraction)+' 78% 71%)';
    ctx.fillRect(x-2,y-2,4,4);
  }
  const poses=displayedPoses(), first=poses[0], last=poses.at(-1);
  const marker=(pose,label,color,below=false)=>{
    const screen=project(pose.xyz);
    if(!screen)return;
    const [x,y]=screen;
    ctx.fillStyle=color; ctx.beginPath(); ctx.arc(x,y,5,0,Math.PI*2); ctx.fill();
    ctx.font='600 12px system-ui';
    const tx=Math.min(w-ctx.measureText(label).width-8,Math.max(5,x+9));
    const ty=Math.min(h-5,Math.max(13,y+(below?17:-10)));
    ctx.strokeStyle='#0b1012';ctx.lineWidth=4;ctx.strokeText(label,tx,ty);ctx.fillText(label,tx,ty);
  };
  marker(first,parseInt(first.frame)===0?'Start':'First mapped pose','#a5efd1');
  marker(last,parseInt(last.frame)===state.frames-1?'Finish':state.recording?'Latest pose':'Last mapped pose','#ffcf80',true);
  const replayId=timeline[cursor]?.id, inferred=replayEstimates.get(replayId);
  const selected=!droneView&&replay&&(posesByFrame.get(replayId)||inferred);
  const selectedScreen=selected&&project(selected.xyz);
  if(selectedScreen){
    const [x,y]=selectedScreen;
    ctx.strokeStyle=inferred?routeStyles[inferred.kind].color:'#72caff';
    ctx.lineWidth=3;ctx.beginPath();ctx.arc(x,y,9,0,Math.PI*2);ctx.stroke();
  }
}
function fitMapView(){
  if(!canvas.clientWidth||!canvas.clientHeight)return;
  const samples=[];
  for(const layer of graphLayers()){
    if(layer.line)for(const point of layer.line)samples.push(point);
    if(layer.points?.length){
      for(let i=0;i<layer.points.length;i+=Math.max(1,Math.floor(layer.points.length/2000)))samples.push(layer.points[i]);
    }
    for(const face of layer.mesh?.patches||[])samples.push(...face.vertices);
    if(layer.solid)for(const v of layer.solid.vertices)samples.push(v);
    if(layer.splats)for(let i=0;i<layer.splats.xyz.length;i+=3*Math.max(1,Math.floor(layer.splats.order.length/2000)))samples.push(layer.splats.xyz.subarray(i,i+3));
  }
  if(viewBindings().some(p=>p.type==='Camera map'))for(const pose of displayedPoses())samples.push(pose.xyz);
  if(!samples.length) return;
  const min=[Infinity,Infinity,Infinity], max=min.map(n=>-n);
  for(const p of samples) for(let i=0;i<3;i++){min[i]=Math.min(min[i],p[i]);max[i]=Math.max(max[i],p[i]);}
  centre=min.map((n,i)=>(n+max[i])/2);
  let halfWidth=0,halfHeight=0;
  for(const p of samples){
    const x=p[0]-centre[0],y=p[1]-centre[1],z=p[2]-centre[2];
    halfWidth=Math.max(halfWidth,Math.abs(x*Math.cos(yaw)-y*Math.sin(yaw)));
    halfHeight=Math.max(halfHeight,Math.abs(z*Math.cos(pitch)+(x*Math.sin(yaw)+y*Math.cos(yaw))*Math.sin(pitch)));
  }
  const w=canvas.clientWidth,h=canvas.clientHeight;
  extent=Math.max(1e-5,2.2*Math.min(w,h)*Math.max(halfWidth/w,halfHeight/h));
  zoom=1;
  panX=panY=0;
}
function showMap(m) {
  const sameMap=m && model && m.run===model.run && m.units===model.units &&
    m.feature_index && m.feature_index===model.feature_index;
  if(!sameMap){loadCorrection(m);editingLevel=false;$('levelRings').setAttribute('hidden','');$('levelTools').hidden=true;$('editLevel').setAttribute('aria-pressed',false);}
  const walkChanged=m?.run!==model?.run;
  model=m;
  posesByFrame=new Map(displayedPoses().map(p=>[p.frame,p]));
  updateViewControls();
  updateGaps();
  updateReplayRoute();
  $('estimate').disabled=$('continuous').disabled=!m;
  const label=m ? 'Frames '+parseInt(m.poses[0].frame)+'–'+parseInt(m.poses.at(-1).frame)+' · '+state.frames+' saved' : '';
  $('coverage').textContent=label; $('mapempty').hidden=!!m;
  $('poses').textContent=m ? (m.registered_views??m.poses.length)+' / '+m.views : '—';
  $('points').textContent=m ? m.points.length.toLocaleString() : '—';
  $('residual').textContent=m ? m.mean_error_px+' px' : '—';
  $('units').textContent=m?.units==='m'?(m.reference?'Measured baseline · m':m.tof_scale?'ToF scale estimate · m':'Measured scale · m'):'Relative scale';
  if(m?.level_status)$('units').textContent+=' · '+m.level_status;
  if(m?.gravity_aligned) $('units').textContent+=m.up_fit?' · IMU levelled ±'+m.up_fit.uncertainty_deg+'°':' · IMU aligned';
  if(!sameMap)fitMapView();
  // Another walk opens its preset; a rebuilt map of the same walk keeps the graph being edited.
  if(m && walkChanged)restorePipeline(m.run);
  else if(m && !sameMap && !pipelineRunning)runQuiet(viewerNode());
  draw();
}
canvas.onpointerdown=e=>{
  if(e.button!==0 || drag || (droneView && !droneCamera()))return;
  drag={id:e.pointerId,x:e.clientX,y:e.clientY};
  canvas.setPointerCapture(e.pointerId);
};
canvas.onpointermove=e=>{
  if(drag?.id!==e.pointerId)return;
  const dx=e.clientX-drag.x, dy=e.clientY-drag.y;
  drag.x=e.clientX;
  drag.y=e.clientY;
  if(droneView){
    droneYaw=(droneYaw+dx*.008)%(2*Math.PI);
    dronePitch=Math.max(-Math.PI/2,Math.min(Math.PI/2,dronePitch+dy*.008));
  }else if(e.metaKey || e.altKey){
    panX+=dx;
    panY+=dy;
  }else{
    yaw+=dx*.008;
    pitch=Math.max(-Math.PI/2,Math.min(Math.PI/2,pitch+dy*.008));
  }
  draw();
};
canvas.onpointerup=canvas.onpointercancel=canvas.onlostpointercapture=e=>{
  if(drag?.id===e.pointerId)drag=null;
};
canvas.onwheel=e=>{
  e.preventDefault();
  const change=Math.exp(-e.deltaY*.002);
  if(droneView)droneZoom=Math.max(.65,Math.min(4,droneZoom*change));
  else zoom=Math.max(.15,Math.min(12,zoom*change));
  draw();
};
new ResizeObserver(draw).observe(canvas);
async function update() {
  const s=await (await fetch(API+'/state')).json(); state=s;
  $('connection').textContent=s.connection; $('connection').classList.toggle('live',s.fresh);
  const connected=['Live','Connecting'].includes(s.connection);
  $('connect').textContent=connected?'Disconnect':'Connect camera'; $('connect').disabled=s.recording;
  $('record').textContent=s.recording?'Finish walk':'Start walk'; $('record').disabled=!s.recording&&!s.fresh;
  $('record').classList.toggle('recording',s.recording);
  let recordStatus = 'Ready to record';
  if(s.recording) recordStatus = 'Recording in progress';
  else if(!s.fresh) recordStatus = connected?'Waiting for live camera frames':'Connect the camera to record';
  if(s.busy) recordStatus += s.fresh&&!s.recording
    ? ' · You can start a walk while the map rebuilds'
    : ' · Reconstruction running in the background';
  if($('recordStatus').textContent!==recordStatus) $('recordStatus').textContent=recordStatus;
  updateCamera();

  const size=s.camera?s.camera.join(' × '):'';
  if(!replay) $('rate').textContent=s.fresh?s.fps+' fps · '+size+' · '+(s.tofRaw==null?'ToF unavailable':'ToF '+s.tofRaw+' raw (~cm)')+(s.tofHz==null?'':' · '+s.tofHz+' ToF reads/s ('+s.tofValidPercent+'% valid)')+' · IMU '+(s.motionFresh?'live':'unavailable'):'OV5647 · 640 × 480';
  $('frames').textContent=s.frames.toLocaleString(); $('walk').textContent=s.run||'No walk selected';
  $('captureWarning').hidden=!s.fresh||s.motionFresh;
  $('message').textContent=s.message; $('samples').textContent=s.samples+' views'; $('lens').textContent=s.calibrated?'Calibrated':'Self-estimate available';
  $('download').hidden=!s.hasMap;
  updatePipelineButton();
  $('load').disabled=s.busy||s.recording; $('sample').disabled=!s.fresh||s.recording;
  $('calibrate').disabled=s.samples<12||s.recording;
  const replayPose=replay && !s.busy && model?.run===s.run && model.poses.some(p=>p.frame===timeline[cursor]?.id);
  if(replayMark?.run!==s.run || s.recording) replayMark=null;
  $('markA').disabled=!s.recording && !replayPose;
  $('markB').disabled=s.recording?!s.refs.A:(!replayPose || !replayMark || replayMark.frame===timeline[cursor]?.id);
  $('refs').textContent=[s.refs.A&&'A saved',s.refs.B&&'B saved'].filter(Boolean).join(' · ');
  if(replayMark) $('refs').textContent='A · frame '+parseInt(replayMark.frame);
  const run=s.recording?null:s.run;
  if(run!==timelineRun){
    pauseReplay();
    clearGapEvidence();
    const response=run&&await fetch(API+'/timeline?'+new URLSearchParams({run}));
    timeline=response?.ok?await response.json():[];
    timelineRun=run; displayedFrame=null; cursor=0;
    updateGaps();
    updateReplayRoute();
    $('timeline').max=timeline.at(-1)?.ms||0;
    $('timeline').value=0;
    replay=timeline.length>0;
    $('cameraMode').disabled=!timeline.length;
    refreshReplay();
  }
  if(s.fresh&&!replay&&!$('framesStep').dataset.live){$('framesStep').open=true;$('framesStep').dataset.live='1';}
  if(!s.fresh||replay)delete $('framesStep').dataset.live;
  if(s.version!==version) {
    const m=s.hasMap?await(await fetch(API+'/survey.json')).json():null;
    resetTracking(); showMap(m); version=s.version;
    if(!m&&run&&lastSourceWalk!==run)restorePipeline(run);
    lastSourceWalk=run;
    refreshReplay();
    const list=await(await fetch(API+'/runs')).json(), old=$('runs').value;
    $('runs').replaceChildren(...list.map(n=>new Option(n,n)));
    $('runs').value=list.includes(old)?old:(s.run||list[0]||'');
  }
  updateWalkActions();
}
async function act(key,body={}) {
  if(active)return; active=true;
  try{
    const r=await fetch(API+'/'+key,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    const v=await r.json(); if(!r.ok)throw Error(v.error);
    $('error').textContent=''; await update();
  }catch(e){$('error').textContent=e.message;}finally{active=false;}
}
$('connect').onclick=()=>act(['Live','Connecting'].includes(state.connection)?'disconnect':'connect');
$('levelStart').onchange=()=>{$('mountAngles').hidden=!$('levelStart').checked;};
for(const id of ['mountPitch','mountRoll']){
  const saved=localStorage.getItem('room-survey-'+id);
  if(saved!==null)$(id).value=saved;
}
$('record').onclick=()=>{
  if(state.recording)return act('stop');
  if(!$('levelStart').checked)return act('start');
  for(const id of ['mountPitch','mountRoll']){
    if(!$(id).reportValidity())return;
    localStorage.setItem('room-survey-'+id,$(id).value);
  }
  return act('start',{level:{pitch:$('mountPitch').valueAsNumber,roll:$('mountRoll').valueAsNumber}});
};
$('markA').onclick=()=>{
  if(state.recording) return act('A');
  replayMark={run:state.run,frame:timeline[cursor].id};
  $('refs').textContent='A · frame '+parseInt(replayMark.frame);
};
$('markB').onclick=()=>{
  const metres=$('metres').value;
  if(state.recording) return act('B',{metres});
  if(replayMark) return act('replay-baseline',{run:replayMark.run,A:replayMark.frame,B:timeline[cursor].id,metres});
};
$('load').onclick=()=>act('load',{name:$('runs').value});
function updateWalkActions(){
  const name=$('runs').value;
  const unfinished=(name===state?.run&&state.recording)||state?.buildingWalks?.includes(name);
  $('archive').disabled=$('deleteWalk').disabled=!name||unfinished;
  $('load').disabled=!name||state?.recording||state?.busy;
}
$('runs').onchange=updateWalkActions;
$('archive').onclick=()=>{
  const link=document.createElement('a');
  link.href=API+'/walk.zip?'+new URLSearchParams({run:$('runs').value});
  link.download=$('runs').value+'.zip';
  link.click();
};
$('deleteWalk').onclick=()=>{
  const name=$('runs').value;
  if(confirm('Move walk '+name+' and all its files to survey trash?')){
    act('delete',{name});
  }
};
for(const id of ['sample','calibrate']) $(id).onclick=()=>act(id);
$('reset').onclick=()=>act('reset-samples');
const preview=$('preview');
let previewPending=false;
preview.onload=preview.onerror=()=>{previewPending=false;};
setInterval(()=>{
  if(!replay && state?.fresh && !document.hidden && !previewPending) {
    previewPending=true; preview.src=API+'/preview.jpg?t='+Date.now();
  }
},125);
async function poll(){try{if(!active)await update();}catch(e){$('connection').textContent='Dashboard offline';$('record').disabled=true;$('recordStatus').textContent='Dashboard offline';}finally{setTimeout(poll,450);}}
function resetTracking(){
  generation++;tracking=false;estimate=null;$('continuous').textContent='Start tracking';
  $('matches').hidden=true;$('tracking').textContent='Choose live tracking or inspect a recorded frame.';draw();
}
async function locate(){
  if(estimating||!state||!model)return;
  estimating=true;const token=generation,source=$('source').value;
  try{
    const r=await fetch(API+'/locate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({
      run:state.run,source,frame:parseInt(timeline[cursor]?.id),minimum:+$('minimum').value})});
    const q=await r.json();if(token!==generation)return;
    if(!r.ok||q.version!==version)throw Error(q.error||'Map changed');
    const img=new Image();img.src='data:image/jpeg;base64,'+q.image;await img.decode();if(token!==generation)return;
    estimate=q;draw();const el=$('matches');el.width=img.naturalWidth;el.height=img.naturalHeight;
    const c=el.getContext('2d');el.hidden=false;c.drawImage(img,0,0);
    c.fillStyle='#38ff9e';for(const [x,y] of q.pixels||[])c.fillRect(x-2,y-2,4,4);
    $('tracking').textContent=q.status+' · '+q.inliers+' inliers / '+q.matches+' matches · '+(source==='live'?'camera '+(q.stamp/1e6).toFixed(2)+' s':'frame '+q.stamp);
  }catch(e){if(token===generation){resetTracking();$('tracking').textContent=e.message;}}
  finally{estimating=false;}
}
$('estimate').onclick=locate;$('continuous').onclick=()=>{tracking=!tracking;$('continuous').textContent=tracking?'Pause tracking':'Start tracking';if(tracking)locate();};
setInterval(()=>{if(tracking&&!document.hidden)locate();},750);
$('source').onchange=resetTracking;
// Files of the Gaussian map the viewer shows, including non-default training variants.
function gaussianUrl(file){
  return API+'/gaussian-artifact?'+new URLSearchParams({run:model.run,index:activeGaussian?.index||model.feature_index,variant:activeGaussian?.variant||'',file});
}
let clearanceReport=null, clearanceKey='';
const clearanceStates={
  fits:{label:'Fits observed space',color:'#89dfb2'},
  conflict:{label:'Obstacle conflict',color:'#ff8275'},
  unknown:{label:'Unknown space',color:'#ecbf75'},
  missing:{label:'No reliable pose',color:'#88969f'}
};
function envelopeRadius(){
  const body=$('bodyRadius'), margin=$('clearanceMargin');
  if(!body.value || !margin.value || !body.validity.valid || !margin.validity.valid)return null;
  return (body.valueAsNumber+margin.valueAsNumber)/100;
}
function clearanceState(row,radius){
  if(!row)return 'missing';
  if(row.obstacle_m!==null && row.obstacle_m<=radius)return 'conflict';
  return row.observed_m>radius?'fits':'unknown';
}
function syncClearance(){
  if(!clearanceReport || timelineRun!==model?.run)return;
  const frame=timeline[cursor], radius=envelopeRadius();
  if(!frame)return;
  $('clearanceSeek').value=frame.ms;
  const info=$('clearanceReading');
  if(radius===null){info.textContent='Enter a positive body radius and a nonnegative margin within the shown limits.';return;}
  const row=clearanceReport.frames[frame.id], result=clearanceStates[clearanceState(row,radius)];
  let detail='Frame '+parseInt(frame.id)+' · '+(frame.ms/1000).toFixed(2)+' s · '+result.label;
  if(row){
    detail+=' · '+row.pose+' pose · nearest occupied cell '+
      (row.obstacle_m===null?'unavailable':Math.round(row.obstacle_m*100)+' cm')+
      ' · observed free radius '+Math.round(100*Math.min(row.observed_m,row.obstacle_m??Infinity))+' cm';
  }
  info.textContent=detail;
}
function paintClearanceTrack(){
  const plot=$('clearancePlot'), width=plot.clientWidth;
  if(!width || !clearanceReport || !timeline.length || timelineRun!==model?.run)return;
  const radius=envelopeRadius(), d=devicePixelRatio, duration=timeline.at(-1).ms||1;
  plot.width=Math.ceil(width*d);plot.height=Math.ceil(plot.clientHeight*d);
  $('clearanceSeek').max=duration;
  const counts={fits:0,conflict:0,unknown:0,missing:0};
  const legend=$('clearanceCounts');
  legend.replaceChildren();
  if(radius===null){syncClearance();return;}
  const paint=plot.getContext('2d');
  for(let i=0;i<timeline.length;i++){
    const frame=timeline[i], state=clearanceState(clearanceReport.frames[frame.id],radius);
    const begin=i?(timeline[i-1].ms+frame.ms)/2:0;
    const end=i+1<timeline.length?(frame.ms+timeline[i+1].ms)/2:duration;
    paint.fillStyle=clearanceStates[state].color;
    paint.fillRect(begin/duration*plot.width,0,Math.max(1,(end-begin)/duration*plot.width),plot.height);
    counts[state]++;
  }
  for(const [state,count] of Object.entries(counts)){
    const item=document.createElement('span');
    item.style.color=clearanceStates[state].color;
    item.textContent=clearanceStates[state].label+' · '+count;
    legend.append(item);
  }
  syncClearance();
}
async function refreshClearance(){
  const open=model && activeGaussian && !$('clearancePanel').hidden && $('clearancePanel').open;
  const key=open?gaussianUrl('clearance-v1/report.json')+'&version='+state.version:'';
  if(clearanceKey===key)return;
  clearanceKey=key;clearanceReport=null;
  $('clearanceControls').hidden=true;
  if(!key)return;
  const status=$('clearanceStatus');
  status.textContent='Opening clearance map…';
  try{
    const response=await fetch(key);
    if(clearanceKey!==key)return;
    if(response.status===404){
      status.textContent='Run the Walk clearance node to build it.';return;
    }
    if(!response.ok)throw Error('Could not load clearance data.');
    const data=await response.json();
    if(clearanceKey!==key)return;
    clearanceReport=data;
    status.textContent=data.views+' camera views · '+data.placed_ranges.toLocaleString()+' / '+
      data.valid_ranges.toLocaleString()+' valid '+data.range_source+' readings placed · '+
      (100*data.voxel_m).toFixed(1)+' cm cells · '+data.scale_source+
      (data.scale_check_error==null?'':' ('+Math.round(data.scale_check_error*100)+'% median range discrepancy on scale check)')+
      ' · '+data.mount;
    $('clearanceGrid').href=gaussianUrl('clearance-v1/grid.npz');
    $('clearanceControls').hidden=false;
    paintClearanceTrack();draw();
  }catch(error){if(clearanceKey===key){status.textContent=error.message;clearanceKey='';}}
}
function drawClearanceEnvelope(project,screenScale){
  if(!clearanceReport || !replay || timelineRun!==model?.run)return;
  const radius=envelopeRadius(), row=clearanceReport.frames[timeline[cursor]?.id];
  if(!row || radius===null)return;
  const point=project(row.xyz);
  if(!point)return;
  const color=clearanceStates[clearanceState(row,radius)].color;
  ctx.save();ctx.strokeStyle=color;ctx.fillStyle=color+'40';ctx.lineWidth=3;
  ctx.beginPath();ctx.arc(point[0],point[1],Math.max(4,radius*screenScale/clearanceReport.scale),0,2*Math.PI);
  ctx.fill();ctx.stroke();ctx.restore();
}
$('clearancePanel').ontoggle=()=>{refreshClearance();draw();};
for(const id of ['bodyRadius','clearanceMargin'])$(id).oninput=()=>{paintClearanceTrack();draw();};
$('clearanceSeek').oninput=()=>{pauseReplay();replay=true;seek($('clearanceSeek').valueAsNumber);};
new ResizeObserver(paintClearanceTrack).observe($('clearancePlot'));

let evidenceKey='', evidenceTicket=0, evidenceRows=[], shownRays=[];

function selectTofRays(){
  const filter=$('tofRayFilter').value;
  shownRays=evidenceRows.filter(r=>filter==='all' || (r.validation && (filter!=='miss' || !r.hit)));
  $('tofRayFrame').replaceChildren(new Option('Choose a frame',''),...shownRays.map(r=>
    new Option('Frame '+parseInt(r.frame)+' · '+(r.hit?'hit':'miss')+' · '+r.measured_m.toFixed(2)+' m',r.frame)));
  for(const id of ['tofRayFrame','tofPrev','tofNext'])$(id).disabled=!shownRays.length;
  syncTofReading(); draw();
}

function syncTofReading(){
  if(!evidenceKey || $('tofEvidenceControls').hidden)return;
  const id=replay && timeline[cursor]?.id;
  const row=shownRays.find(r=>r.frame===id);
  $('tofRayFrame').value=row?.frame||'';
  const detail=$('tofRayDetail');
  if(!shownRays.length){detail.textContent='No readings match this filter.';return;}
  if(!row){detail.textContent='Choose a reading to see its recorded photo and beam.';return;}
  let info='Frame '+parseInt(id)+' · '+(row.validation?'validation':'comparison set')+' · measured '+row.measured_m.toFixed(2)+' m';
  if(!row.hit)info+=' · ray misses the mesh';
  else if(row.mesh_m!=null){
    const diff=row.mesh_m-row.measured_m;
    info+=' · first mesh hit '+row.mesh_m.toFixed(2)+' m · '+Math.abs(100*diff).toFixed(1)+' cm '+(diff>=0?'beyond':'before')+' the return';
  }else info+=' · ray intersects the mesh';
  detail.textContent=info;
}

async function refreshTofEvidence(){
  const enabled=model && activeGaussian && $('tofEvidenceToggle').checked;
  $('tofEvidencePanel').hidden=!enabled;
  const key=enabled?gaussianUrl('tof-evidence.json')+'&version='+state.version:'';
  if(evidenceKey===key)return;
  evidenceKey=key;
  const ticket=++evidenceTicket;
  evidenceRows=[]; shownRays=[];
  $('tofEvidenceControls').hidden=true; $('tofRayDetail').textContent='';
  if(!enabled)return;
  $('tofEvidenceStatus').textContent='Checking recorded ranges against the saved mesh…';
  try{
    const response=await fetch(key);
    if(!response.ok)throw Error(await response.text());
    const {summary,readings}=await response.json();
    if(ticket!==evidenceTicket)return;
    evidenceRows=readings;
    const held=readings.filter(r=>r.validation), misses=held.filter(r=>!r.hit);
    const scale=summary.units==='m'?'metric map':summary.candidate_metres_per_unit?
      'provisional scale '+summary.candidate_metres_per_unit.toFixed(3)+' m/unit':'unscaled map';
    $('tofEvidenceStatus').textContent=misses.length+' misses / '+held.length+' validation rays · '+readings.length+
      ' paired, registered readings · '+summary.beam+' · '+scale+'. '+
      (summary.reason||'Orange diamonds mark missed rays, teal dots mark returns on rays that hit. A white cross marks the selected mesh intersection.');
    $('tofEvidenceControls').hidden=!readings.length;
    selectTofRays();
  }catch(error){
    if(ticket!==evidenceTicket)return;
    $('tofEvidenceStatus').textContent=error.message;
    evidenceKey='';
  }
}

function drawTofEvidence(project,cameraPoint,projectCamera,camera,near){
  const current=replay && timeline[cursor]?.id;
  ctx.save();
  function marker(row,selected){
    const end=row.endpoint && project(row.endpoint);
    const color=row.hit?'#6ce3e7':'#ff9a49';
    if(selected && row.endpoint){
      const beam=camera?clipNear([row.origin,row.endpoint].map(cameraPoint),near,false).map(projectCamera):[project(row.origin),end];
      if(beam.length===2){
        ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(...beam[0].slice(0,2));ctx.lineTo(...beam[1].slice(0,2));ctx.stroke();
      }
    }
    if(end){
      const [x,y]=end, r=selected?7:row.hit?2.8:4.5;
      ctx.fillStyle=color;ctx.strokeStyle='#152025';ctx.lineWidth=1.3;ctx.beginPath();
      if(row.hit)ctx.arc(x,y,r,0,Math.PI*2);
      else{ctx.moveTo(x,y-r);ctx.lineTo(x+r,y);ctx.lineTo(x,y+r);ctx.lineTo(x-r,y);ctx.closePath();}
      ctx.fill();ctx.stroke();
      if(selected){ctx.strokeStyle='white';ctx.beginPath();ctx.arc(x,y,r+3,0,Math.PI*2);ctx.stroke();}
    }
    const hit=selected && row.intersection && project(row.intersection);
    if(hit){
      const [x,y]=hit;
      ctx.beginPath();ctx.moveTo(x-5,y);ctx.lineTo(x+5,y);ctx.moveTo(x,y-5);ctx.lineTo(x,y+5);
      ctx.strokeStyle='#162022';ctx.lineWidth=4;ctx.stroke();ctx.strokeStyle='white';ctx.lineWidth=2;ctx.stroke();
    }
  }
  // Draw misses and the selected frame last so they stay visible in dense clusters.
  for(const hit of [true,false])for(const row of shownRays)if(row.hit===hit && row.frame!==current)marker(row,false);
  const selected=shownRays.find(r=>r.frame===current);
  if(selected)marker(selected,true);
  ctx.restore();
}

function jumpToTofFrame(id){
  const frame=timeline.find(f=>f.id===id);
  if(!frame || timelineRun!==model?.run)return;
  replay=true;
  jumpToEndpoint(frame);
}
function stepTofReading(forward){
  const id=timeline[cursor]?.id||'';
  const row=forward?(shownRays.find(r=>r.frame>id)||shownRays[0]):
    (shownRays.findLast(r=>r.frame<id)||shownRays.at(-1));
  if(row)jumpToTofFrame(row.frame);
}
$('tofEvidenceToggle').onchange=()=>{refreshTofEvidence();draw();};
$('tofRayFilter').onchange=selectTofRays;
$('tofRayFrame').onchange=()=>jumpToTofFrame($('tofRayFrame').value);
$('tofPrev').onclick=()=>stepTofReading(false);
$('tofNext').onclick=()=>stepTofReading(true);

$('gaussianTestFrame').onchange=()=>gaussianPair();
function gaussianPair(){
  const frame=$('gaussianTestFrame').value;
  if(!frame)return;
  $('gaussianPhoto').src=gaussianUrl('reference/'+frame);
  $('gaussianRender').src=gaussianUrl('renders/'+frame.replace('.jpg','.png'));
}
function updateViewControls(){
  $('droneView').disabled=!model || !timeline.length;
  if($('droneView').disabled)droneView=false;
  $('droneView').setAttribute('aria-pressed',droneView);
  $('mapHelp').textContent=droneView?'Drag to look around · scrub the timeline to move · scroll to zoom':
    'Drag to orbit · ⌘ or ⌥ + drag to pan · scroll to zoom';
}

function setEnlarged(enlarged){
  const views=$('views');
  views.classList.toggle('enlarged',enlarged);
  draw();
  $('enlarge').textContent=enlarged?'Collapse':'Enlarge';
  $('enlarge').setAttribute('aria-expanded',enlarged);
}

$('enlarge').onclick=()=>setEnlarged(!$('views').classList.contains('enlarged'));
$('droneView').onclick=()=>{
  droneView=!droneView;
  if(droneView){
    if(editingLevel)editOrientation(false);
    replay=true;
    droneZoom=1;
    droneYaw=dronePitch=0;
  }
  refreshReplay();
};
$('recenterDrone').onclick=()=>{
  droneYaw=dronePitch=0;
  draw();
};
function setOverview(angle,pitchAngle){
  droneView=false;
  updateViewControls();
  yaw=angle;
  pitch=pitchAngle;
  fitMapView();
  draw();
}
const presets={top:[0,Math.PI/2],bottom:[0,-Math.PI/2],front:[Math.PI,0],
  back:[0,0],side:[-Math.PI/2,0],left:[Math.PI/2,0],iso:[-Math.PI/4,Math.asin(1/Math.sqrt(3))]};
for(const name of ['top','front','side','iso'])$(name).onclick=()=>setOverview(...presets[name]);
for(const name of ['persp','ortho'])$(name).onclick=()=>{
  droneView=false;orbitPerspective=name==='persp';updateViewControls();draw();
};
const gizmoNodes=[];
for(const [axis,sides,color] of [[0,['left','side'],'#eb6256'],[1,['back','front'],'#50ba72'],[2,['bottom','top'],'#6599ff']]){
  for(const sign of [-1,1]){
    const svg=tag=>document.createElementNS('http://www.w3.org/2000/svg',tag);
    const group=svg('g'),line=svg('line'),circle=svg('circle'),label=svg('text'),title=svg('title');
    const axisName=(sign>0?'+':'−')+'XYZ'[axis];
    title.textContent='Look from '+axisName;
    group.setAttribute('role','button');group.setAttribute('tabindex','0');group.setAttribute('aria-label',title.textContent);
    line.setAttribute('stroke',color);line.setAttribute('stroke-width','2');
    line.setAttribute('x1','42');line.setAttribute('y1','42');
    circle.setAttribute('r','7.5');circle.setAttribute('fill',sign>0?color:'#20292c');circle.setAttribute('stroke',color);
    label.textContent=sign>0?'XYZ'[axis]:'';
    group.append(title,line,circle,label);$('axisGizmo').append(group);
    group.onclick=()=>setOverview(...presets[sides[sign>0?1:0]]);
    group.onkeydown=e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();group.onclick();}};
    gizmoNodes.push({axis,sign,group,line,circle,label});
  }
}
function updateAxisGizmo(axes){
  $('persp').setAttribute('aria-pressed',droneView||orbitPerspective);
  $('ortho').setAttribute('aria-pressed',!droneView&&!orbitPerspective);
  const ordered=gizmoNodes.slice().sort((a,b)=>b.sign*axes[2][b.axis]-a.sign*axes[2][a.axis]);
  ordered.forEach(({axis,sign,group,line,circle,label},index)=>{
    const x=42+31*sign*axes[0][axis],y=42+31*sign*axes[1][axis];
    line.setAttribute('x2',x);line.setAttribute('y2',y);
    circle.setAttribute('cx',x);circle.setAttribute('cy',y);
    label.setAttribute('x',x);label.setAttribute('y',y);
    const svg=$('axisGizmo');
    if(svg.children[index]!==group)svg.insertBefore(group,svg.children[index]);
  });
}
$('range').onclick=async()=>{try{const r=await fetch(API+'/range',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({address:Number($('address').value)})});
const q=await r.json();$('reading').textContent=r.ok?'Raw reading '+q.raw:q.error;}catch(e){$('reading').textContent=e.message;}};
$('balloonCut').oninput=()=>{
  const percent=$('balloonCut').value;
  $('balloonCutValue').value=percent+'%';
  $('balloonCut').setAttribute('aria-valuetext',percent+' percent of balloon height');
  for(const layer of graphLayers())if(layer.cut!=null)layer.cut=+percent;
  draw();
};
let rightKey='';
function timingText(t){
  if(!t)return '';
  const stages=Object.entries(t.stages||{}).map(([name,seconds])=>name+' '+seconds.toFixed(2)+' s');
  return [t.seconds.toFixed(2)+' s total',t.frames?Math.round(1000*t.seconds/t.frames)+' ms/frame · '+(t.frames/t.seconds).toFixed(1)+' fps':'',...stages].filter(Boolean).join(' · ');
}
function refreshDisplayedPath(){
  posesByFrame=new Map(displayedPoses().map(p=>[p.frame,p]));
  updateReplayRoute();updateGaps();refreshReplay();
}
function comparisonUrl(method,frame){
  const query=depthQuery();
  const endpoint=method==='photo'?API+'/depth-input.jpg?':API+'/frame-depth.png?';
  return endpoint+new URLSearchParams({...query,method,frame});
}
function refreshComparison(frame){
  const enabled=replay&&$('compareFrames').checked;
  $('leftSourceLabel').textContent=enabled?'Left':'Frame view';
  for(const id of ['comparisonRight','wipe','wipeLine','rightChoice'])$(id).hidden=!enabled;
  if(!enabled||!frame||!depthQuery().index){rightKey='';return;}
  const url=comparisonUrl($('rightSource').value,frame.id);
  if(rightKey===url)return;
  rightKey=url;
  $('rightMissing').hidden=true;
  $('rightPhoto').style.visibility='hidden';
  $('rightPhoto').src=url;
}
$('rightPhoto').onload=()=>{$('rightPhoto').style.visibility='';$('rightMissing').hidden=true;};
$('rightPhoto').onerror=()=>{$('rightMissing').hidden=false;};
$('compareFrames').onchange=()=>{displayedFrame=null;refreshReplay();};
$('rightSource').onchange=()=>refreshComparison(timeline[cursor]);
$('wipe').oninput=()=>$('wipe').parentElement.style.setProperty('--split',$('wipe').value+'%');
let profileKey='', profileTick=0;
async function refreshProfile(){
  const panel=$('profilePanel'), query=depthQuery();
  if(!panel.open||!replay||!query.index)return;
  const key=query.run+'/'+query.index;
  if(key===profileKey&&Date.now()-profileTick<15000)return;
  const body=$('timingTable');
  if(key!==profileKey)body.textContent='Reading timings…';
  profileKey=key;profileTick=Date.now();
  const rows=await Promise.all(Array.from($('frameMethod').options).slice(1).map(async opt=>{
    try{
      const res=await fetch(API+'/frame-depth-status?'+new URLSearchParams({...query,method:opt.value}));
      const data=await res.json(), t=data.timing||data.benchmark;
      return {name:opt.text,t,kind:data.timing?'Build':'Benchmark'};
    }catch{return {name:opt.text};}
  }));
  if(profileKey!==key)return;
  const table=document.createElement('table');
  const header=table.createTHead().insertRow();
  for(const title of ['Method / stages','Frames','Total','ms/frame','fps']){
    const th=document.createElement('th');th.textContent=title;header.append(th);
  }
  const tbody=table.createTBody();
  for(const {name,t,kind} of rows){
    const row=tbody.insertRow();
    const cell=row.insertCell();
    if(!t?.frames){cell.textContent=name;const blank=row.insertCell();blank.colSpan=4;blank.textContent='Timing unavailable';continue;}
    const details=document.createElement('details'),summary=document.createElement('summary');
    details.open=Array.from(body.querySelectorAll('details[open] > summary')).some(item=>item.textContent===name);
    summary.textContent=name;details.append(summary);
    for(const [stage,seconds] of Object.entries(t.stages)){
      const line=document.createElement('div');
      line.textContent=stage+' · '+seconds.toFixed(2)+' s'+
        (['Infer frames and write arrays','Render and save'].includes(stage)?' · '+Math.round(seconds*1000/t.frames)+' ms/frame':'');
      details.append(line);
    }
    cell.append(details);
    for(const value of [kind+' · '+t.frames,t.seconds.toFixed(2)+' s',Math.round(t.seconds*1000/t.frames),(t.frames/t.seconds).toFixed(1)]){
      row.insertCell().textContent=value;
    }
  }
  body.replaceChildren(table);
}
$('profilePanel').ontoggle=refreshProfile;
let depthStatusKey='', depthPollTime=0, frameDepthStatus={};
function depthQuery(){
  return {run:timelineRun||'',index:model?.run===timelineRun ? model.feature_index||'' : '',method:$('frameMethod').value};
}
function describeFrameDepth(){
  const {method,index}=depthQuery(), info=$('frameDepthStatus'), build=$('buildFrameDepth');
  const timing=frameDepthStatus.timing||frameDepthStatus.benchmark;
  $('depthTiming').textContent=method==='photo'?'':timing?(frameDepthStatus.benchmark&&!frameDepthStatus.timing?'Benchmark · '+timing.frames+' frames · ':'Build · ')+timingText(timing):'Build timing unavailable for this archive';
  if(method==='photo'){
    const results=Object.values(frameDepthStatus);
    const available=results.filter(row=>row.available);
    const missing=available.some(row=>!row.complete&&!row.running);
    build.hidden=!index||!missing;
    build.disabled=!!state?.busy;
    build.textContent='Generate missing depth maps';
    const running=Object.keys(frameDepthStatus).filter(key=>frameDepthStatus[key].running).map(key=>
      Array.from($('frameMethod').options).find(option=>option.value===key)?.text);
    info.textContent=results.length ? available.filter(row=>row.complete).length+' / '+available.length+' depth methods ready'+
      (running.length?' · Processing '+running.join(', '):'')+
      (frameDepthStatus.gaussian?.available?'':' · Gaussian depth needs trained splats') : '';
    return;
  }
  build.textContent='Generate all frames';
  build.hidden=!index||frameDepthStatus.saved===timeline.length;
  build.disabled=!!state?.busy||frameDepthStatus.running||frameDepthStatus.available===false;
  const scale=method==='apple'?'Estimated metres · yellow near, purple 8 m+':'Relative depth · colours scaled per frame';
  const pose=frameDepthStatus.poses?.[timeline[cursor]?.id];
  info.textContent=method==='photo'?'':!index?'Build the camera map to enable frame depth':
    [scale,frameDepthStatus.saved==null?'':frameDepthStatus.saved+' / '+timeline.length+' ready',pose,frameDepthStatus.running?'Processing':'',frameDepthStatus.available===false?'Train the Gaussian map first':method==='gaussian'?'Black = no surface depth':''].filter(Boolean).join(' · ');
}
async function pollFrameDepth(){
  const query=depthQuery(), key=new URLSearchParams(query).toString();
  if(key!==depthStatusKey){depthStatusKey=key;depthPollTime=0;frameDepthStatus={};}
  describeFrameDepth();
  if(!query.index||(frameDepthStatus.total===timeline.length&&frameDepthStatus.saved===timeline.length)||Date.now()-depthPollTime<2500)return;
  depthPollTime=Date.now();
  try{
    const res=await fetch(API+'/frame-depth-status?'+new URLSearchParams({...query,method:query.method==='photo'?'all':query.method}));
    if(!res.ok)return;
    const next=await res.json();
    if(key!==depthStatusKey)return;
    const retry=next.saved!==frameDepthStatus.saved&&!$('depthMissing').hidden;
    const newlyComplete=next.saved===timeline.length && frameDepthStatus.saved!==next.saved;
    frameDepthStatus=next; describeFrameDepth();
    if(retry){displayedFrame=null;refreshReplay();}
  }catch{}
}
$('frameMethod').onchange=()=>{displayedFrame=null;refreshReplay();};
$('buildFrameDepth').onclick=()=>{const query=depthQuery();return act('build-frame-depth',{...query,method:query.method==='photo'?'all':query.method});};
$('replayImage').onerror=()=>{if(replay){$('replayImage').style.visibility='hidden';$('depthMissing').hidden=false;}};
$('replayImage').onload=()=>{$('replayImage').style.visibility='';$('depthMissing').hidden=true;};
function updateCamera(){
  
  $('cameraTitle').textContent=replay?($('pipeline').dataset.inspect==='depth'?'Depth preview':'Recorded frame'):'Live camera';
  const depthPreview=$('pipeline').dataset.inspect==='depth';
  $('frameControls').hidden=!replay||!depthPreview;
  refreshComparison(timeline[cursor]);
  if(replay&&depthPreview){pollFrameDepth();refreshProfile();}
  else $('depthMissing').hidden=true;
  $('cameraMode').textContent=replay?'Show live camera':'Review walk';
  $('replayPanel').hidden=!replay;
  $('replayImage').hidden=!replay;
  $('preview').hidden=replay||!state?.fresh;
  $('cameraempty').hidden=replay||!!state?.fresh;
}
function drawTimeline(){
  const plot=$('timelinePlot'), width=plot.clientWidth;
  if(!width||!timeline.length)return;
  const d=devicePixelRatio;
  plot.width=width*d;plot.height=13*d;
  const g=plot.getContext('2d');g.scale(d,d);
  const duration=Math.max(1,timeline.at(-1).ms);
  const x=ms=>2+ms/duration*(width-4);
  const registered=new Map(displayedPoses().map(p=>[p.frame,p.method]));
  for(const frame of timeline){
    const inferred=replayEstimates.get(frame.id);
    g.fillStyle=inferred?routeStyles[inferred.kind].color:registered.has(frame.id)?(registered.get(frame.id)?'#72caff':'#a5efd1'):'#556469';
    g.fillRect(x(frame.ms),3,2,7);
  }
  g.fillStyle='#72caff';g.fillRect(x(timeline[cursor].ms)-1,0,3,13);
}
new ResizeObserver(drawTimeline).observe($('timelinePlot'));
function replaySnapshot(){
  const frame=timeline[cursor];
  return frame?{frameId:frame.id,index:cursor,timeMs:frame.ms}:null;
}
function showReplayBinding(){
  const el=$('codeCursor');
  const frame=replaySnapshot();
  el.textContent=$('bindReplay').checked?(frame?'Current cursor · '+JSON.stringify(frame):'No replay frame is loaded'):'Cursor disabled for this node';
}
function refreshReplay(){
  showReplayBinding();
  syncClearance();
  syncTofReading();
  updateViewControls();
  updateCamera();
  const frame=timeline[cursor];
  if(!replay||!frame){draw();return;}
  const query=depthQuery();
  const compare=$('compareFrames').checked;
  const key=[timelineRun,frame.id,query.method,query.index,compare].join('/');
  if(displayedFrame!==key){
    displayedFrame=key;
    $('replayImage').style.visibility='hidden';
    $('depthMissing').hidden=true;
    if((query.method==='photo'&&!compare)||query.index){
      $('replayImage').src=compare?comparisonUrl(query.method,frame.id):(query.method==='photo'?API+'/recorded.jpg?':API+'/frame-depth.png?')+new URLSearchParams({...query,frame:frame.id});
    }else{
      $('replayImage').removeAttribute('src');
      $('depthMissing').hidden=false;
    }
  }
  const seconds=ms=>(ms/1000).toFixed(2)+' s';
  $('frameNo').textContent=seconds(frame.ms)+' / '+seconds(timeline.at(-1).ms)+' · Frame '+parseInt(frame.id);
  $('rate').textContent='Recorded · '+(frame.tof_m!=null?'ToF '+frame.tof_m.toFixed(2)+' m':'ToF unavailable')+
    ' · '+(frame.gyro?'IMU paired':'No paired IMU');
  const hasPose=posesByFrame.get(frame.id), inferred=replayEstimates.get(frame.id);
  $('timelineStatus').textContent=hasPose
    ? (hasPose.method==='depth_track'?'Pose estimated from depth-image matches':hasPose.method==='aligned_window'?'Pose aligned from overlapping views':hasPose.method?'Pose recovered from nearby frames':'Pose registered in the map')
    : inferred?.held?'Estimated path · held at nearest mapped pose'
    : inferred?.kind==='interpolated'?'Estimated path · straight line between mapped poses'
    : inferred?.kind==='extrapolated'?'Estimated path · extrapolated from mapped motion'
    : model?.sampled_frames?.includes(frame.id)
      ? 'Unmapped · sampled but pose unresolved in the main map'
      : 'Unmapped · outside the initial sample and no recovery passed';
  const visual=!!model;
  $('gapPicker').hidden=!visual||!gapRanges.length;
  $('inspectGap').hidden=!visual||!!hasPose||!model?.feature_index;
  $('inspectGap').disabled=!!state?.busy;
  if(!visual) $('timelineStatus').textContent='Sensor-only view';
  drawTimeline();draw();
}
function seek(ms){
  let left=0,right=timeline.length-1;
  while(left<right){
    const mid=Math.floor((left+right)/2);
    if(timeline[mid].ms<ms)left=mid+1;else right=mid;
  }
  const previous=timeline[left-1];
  const next=previous&&ms-previous.ms<timeline[left].ms-ms?left-1:left;
  if(cursor!==next){cursor=next;resetTracking();clearGapEvidence();}
  $('timeline').value=ms;
  refreshReplay();
}
function pauseReplay(){
  cancelAnimationFrame(animation);animation=0;$('play').textContent='Play';
}
$('timeline').oninput=()=>{pauseReplay();seek(+$('timeline').value);};
function jumpToEndpoint(frame){
  pauseReplay();
  seek(frame.ms);
}
$('first').onclick=()=>jumpToEndpoint(timeline[0]);
$('last').onclick=()=>jumpToEndpoint(timeline.at(-1));
$('play').onclick=()=>{
  if(animation){pauseReplay();return;}
  if(cursor===timeline.length-1)seek(0);
  const begin=performance.now()-timeline[cursor].ms;
  $('play').textContent='Pause';
  const tick=now=>{
    const end=timeline.at(-1).ms;
    const ms=Math.min(end,now-begin);
    if(ms>=end){seek(end);pauseReplay();return;}
    if(ms>=timeline[cursor].ms+40)seek(ms);
    animation=requestAnimationFrame(tick);
  };
  animation=requestAnimationFrame(tick);
};
$('cameraMode').onclick=()=>{
  $('framesStep').open=true;pauseReplay();replay=!replay;if(!replay)droneView=false;refreshReplay();};
function updateGaps(){
  const mapped=new Set(displayedPoses().map(p=>p.frame));
  gapRanges=[];
  if(model){
    let start=null;
    for(let i=0;i<=timeline.length;i++){
      if(i===timeline.length||mapped.has(timeline[i].id)){
        if(start!==null&&i-start>=2)
          gapRanges.push({first:timeline[start],last:timeline[i-1],count:i-start});
        start=null;
      }else if(start===null)start=i;
    }
  }
  $('gaps').replaceChildren(new Option('Jump to a gap',''),...gapRanges.map(gap=>
    new Option((gap.first.ms/1000).toFixed(1)+'–'+(gap.last.ms/1000).toFixed(1)+' s · '+gap.count+' frames',gap.first.ms)));
}
$('gaps').onchange=()=>{
  if($('gaps').value==='')return;
  pauseReplay();seek(+$('gaps').value);$('gaps').value='';
};
function clearGapEvidence(){
  gapRequest++;
  $('gapEvidence').hidden=true;
}
$('inspectGap').onclick=async()=>{
  const token=++gapRequest, frame=parseInt(timeline[cursor].id);
  const output=$('gapEvidence');output.hidden=false;output.textContent='Checking this frame against the saved map…';
  try{
    const response=await fetch(API+'/locate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({
      run:state.run,source:'replay',frame,minimum:12})});
    const data=await response.json();
    if(!response.ok)throw Error(data.error);
    if(token!==gapRequest)return;
    const detail=data.matches<12
      ? data.matches+' candidate matches · need 12 to try a pose'
      : data.inliers+' geometric inliers / '+data.matches+' candidate matches · '+
        (data.status==='Located'?'Independent pose check passed':'Pose check failed');
    output.textContent='Frame '+frame+' · '+detail;
  }catch(e){if(token===gapRequest)output.textContent=e.message;}
};
