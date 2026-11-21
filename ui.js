'use strict';
const $ = id => document.getElementById(id);
let state, version = -1, model, active = false, yaw = -.38, pitch = .42, zoom = 1;
let replayMark=null;
let droneView=false, droneZoom=1, droneYaw=0, dronePitch=0;
let posesByFrame=new Map();
let timeline=[], timelineRun, replay=false, cursor=0, animation=0, displayedFrame;
let routeSegments=[], replayEstimates=new Map(), replayDirections=new Map();
const routeStyles={
  mapped:{color:'#a5efd1',dash:[]},
  interpolated:{color:'#f7cc7f',dash:[7,5]},
  extrapolated:{color:'#cfb1f7',dash:[2,5]}
};
let mapRun, gapRanges=[], gapRequest=0;
let estimate=null, tracking=false, estimating=false, generation=0;
let centre = [0,0,0], extent = 1, panX = 0, panY = 0, drag;
const canvas = $('map'), ctx = canvas.getContext('2d');
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const normalized=v=>v.map(x=>x/Math.hypot(...v));

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

function updateReplayRoute(){
  const recorded=model?.run===timelineRun && timeline.length>0;
  const frames=recorded?timeline:(model?.path_frames||model?.sampled_frames||[]).map((id,ms)=>({id,ms}));
  const route=buildReplayRoute(model?.poses||[],frames,recorded);
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
  const visible=[];
  if(!closed && points[0][2]>=near)visible.push(points[0]);
  for(let i=closed?0:1;i<points.length;i++){
    const a=points[(i+points.length-1)%points.length], b=points[i];
    if((a[2]>=near)!==(b[2]>=near)){
      const t=(near-a[2])/(b[2]-a[2]);
      visible.push([a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]),near]);
    }
    if(b[2]>=near)visible.push(b);
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

function draw() {
  const w=canvas.clientWidth, h=canvas.clientHeight, d=devicePixelRatio;
  canvas.width=w*d; canvas.height=h*d; ctx.setTransform(d,0,0,d,0,0);
  const status=$('droneStatus');
  status.hidden=true;
  $('vertexKey').hidden=true;
  $('recenterDrone').hidden=!droneView || (!droneYaw && !dronePitch);
  if (!model) return;
  const camera=droneView?droneCamera():null;
  if(droneView && !camera){
    status.textContent='Camera pose unavailable for this frame';
    status.hidden=false;
    return;
  }
  if(camera?.status){
    status.textContent=camera.status;
    status.hidden=false;
  }
  const near=model.units==='m'?.02:Math.max(1e-5,extent*.002);
  const focal=h*.5*droneZoom/Math.tan(75*Math.PI/360);
  const cameraPoint=p=>camera.axes.map(([x,y,z])=>
    x*(p[0]-camera.xyz[0])+y*(p[1]-camera.xyz[1])+z*(p[2]-camera.xyz[2]));
  // Reciprocal depth varies linearly across a perspective triangle in screen space.
  const projectCamera=([x,y,z])=>[w/2+focal*x/z,h/2+focal*y/z,-1/z];
  const cy=Math.cos(yaw), sy=Math.sin(yaw), cp=Math.cos(pitch), sp=Math.sin(pitch);
  const depth=([x,y,z])=>(x*sy+y*cy)*cp-z*sp;
  const scale=Math.min(w,h)*zoom/extent;
  const projectWithDepth=p=>{
    if(camera){
      const viewpoint=cameraPoint(p);
      return viewpoint[2]>=near?projectCamera(viewpoint):null;
    }
    const x=p[0]-centre[0], y=p[1]-centre[1], z=p[2]-centre[2];
    const u=x*cy-y*sy, v=x*sy+y*cy;
    return [w/2+panX+u*scale,h/2+panY-(z*cp+v*sp)*scale,v*cp-z*sp];
  };
  const project=p=>projectWithDepth(p)?.slice(0,2);
  const depthGeometry=$('geometrySource').value!=='photo';
  const gaussian=!depthGeometry&&$('surfaceMethod').value==='gaussian-map';
  let visibility;
  if(gaussian && (gaussianReady || gaussianMeshReady)){
    const axes=camera?.axes||[[cy,-sy,0],[-sy*sp,-cy*sp,-cp],[sy*cp,cy*cp,-sp]];
    const args=[ctx,w,h,camera?.xyz||centre,axes,camera?[w/2,h/2]:[w/2+panX,h/2+panY],camera?focal:scale,!!camera,near];
    if(gaussianReady)gaussianViewer.paint(...args);
    if(gaussianMeshReady){
      gaussianMeshViewer.paint(...args,gaussianReady);
      if($('cloud').checked){
        visibility=new SurfaceVisibility(w,h);
        gaussianMeshViewer.indexVisibility(visibility,projectWithDepth,camera?cameraPoint:null,near,projectCamera);
      }
    }
  }
  const mesh=depthGeometry?depthSurfaceMesh:model.surfaces;
  if(!gaussian && $('surfaceLayer').checked&&mesh?.patches.length){
    if($('cloud').checked)visibility=new SurfaceVisibility(w,h);
    const frame=!depthGeometry && replay && $('surfaceTime').checked && timeline[cursor]?.id;
    const visible=frame ? mesh.patches.filter(p=>p.first<=frame).concat(
      mesh.retired.filter(p=>p.first<=frame && frame<p.until)) : mesh.patches;
    const patches=visible.flatMap(p=>{
      const rings=[p.vertices,...(p.holes||[])];
      if(!camera)return [{p,rings:rings.map(r=>r.map(projectWithDepth)),
        z:p.vertices.reduce((sum,v)=>sum+depth(v),0)/p.vertices.length}];
      const clipped=rings.map(r=>clipNear(r.map(cameraPoint),near));
      const outer=clipped[0];
      if(outer.length<3)return [];
      return [{p,rings:clipped.filter(r=>r.length>=3).map(r=>r.map(projectCamera)),
        z:outer.reduce((sum,v)=>sum+v[2],0)/outer.length}];
    });
    patches.sort((a,b)=>b.z-a.z);
    for(const {p,rings} of patches){
      const [a,b,c]=p.vertices;
      const n=p.normal||normalized(cross(b.map((v,i)=>v-a[i]),c.map((v,i)=>v-a[i])));
      const incidence=camera?camera.axes[2].reduce((sum,v,i)=>sum+v*n[i],0):depth(n);
      const shade=.55+.4*Math.abs(incidence);
      const rgb=[124,194,207].map(n=>Math.round(n*shade));
      ctx.fillStyle='rgb('+rgb.join(',')+')';
      ctx.beginPath();
      for(const ring of rings){
        ring.forEach(([x,y],i)=>i?ctx.lineTo(x,y):ctx.moveTo(x,y));
        ctx.closePath();
      }
      ctx.fill('evenodd');
      if(p.holes)visibility?.addPolygon(rings,p.landmarks);
      else visibility?.addFace(rings[0],p.landmarks);
    }
  }
  if($('cloud').checked){
    const vertices=[];
    (depthGeometry ? activeDepthCloud?.points||[] : model.points).forEach((p,id)=>{
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
  if(!camera && estimate?.xyz){
    const a=project(estimate.xyz), v=estimate.rotation.map(row=>row[2]);
    const b=project(estimate.xyz.map((x,i)=>x+v[i]*extent*.10));
    ctx.strokeStyle='#58baff';ctx.fillStyle='#58baff';ctx.lineWidth=3;
    ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();
    ctx.beginPath();ctx.arc(...a,6,0,Math.PI*2);ctx.fill();
  }
  if(gaussian && $('tofEvidenceToggle').checked)drawTofEvidence(project,cameraPoint,projectCamera,camera,near);
  if(gaussian && !camera)drawClearanceEnvelope(project,scale);
  if(!$('route').checked)return;
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
  for(const p of model.poses){
    const screen=project(p.xyz);
    if(!screen)continue;
    const [x,y]=screen;
    const fraction=parseInt(p.frame)/Math.max(1,(state?.frames||1)-1);
    ctx.fillStyle='hsl('+(167-130*fraction)+' 78% 71%)';
    ctx.fillRect(x-2,y-2,4,4);
  }
  const first=model.poses[0], last=model.poses.at(-1);
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
  const selected=!camera&&replay&&(posesByFrame.get(replayId)||inferred);
  if(selected){
    const [x,y]=project(selected.xyz);
    ctx.strokeStyle=inferred?routeStyles[inferred.kind].color:'#72caff';
    ctx.lineWidth=3;ctx.beginPath();ctx.arc(x,y,9,0,Math.PI*2);ctx.stroke();
  }
}
function fitMapView(){
  if(!model || !canvas.clientWidth || !canvas.clientHeight) return;
  const samples=[];
  if($('cloud').checked && model.points.length){
    const axes=[0,1,2].map(i=>model.points.map(p=>p[i]).sort((a,b)=>a-b));
    const limits=axes.map(v=>[v[Math.floor(v.length*.03)],v[Math.floor(v.length*.97)]]);
    for(const x of limits[0]) for(const y of limits[1]) for(const z of limits[2]) samples.push([x,y,z]);
  }
  if($('surfaceLayer').checked) for(const face of model.surfaces?.patches||[]) samples.push(...face.vertices);
  if($('route').checked || !samples.length) for(const pose of model.poses) samples.push(pose.xyz);
  if($('route').checked) for(const segment of routeSegments){
    if(segment.kind==='extrapolated')samples.push(...segment.points);
  }
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
const aiMethod='depth-anything-v2-v1';
let depthWorker=null, depthPreviewKey='', depthRequest=0, checkingDepth=false;

function depthPair(frame,raw){
  const params=new URLSearchParams({run:model.run,index:model.feature_index,frame});
  $('depthPhoto').src='/depth-input.jpg?'+params;
  $('depthPreview').hidden=false;
  $('depthCaption').textContent='Frame '+parseInt(frame)+ ' · brighter is nearer'+(raw?'':' · '+(model.surfaces?.ai?.verdicts?.[frame]||'Predicted depth'));
  const ctx=$('depthImage').getContext('2d');
  if(raw){
    const image=ctx.createImageData(96,72);
    raw.forEach((v,i)=>{image.data.set([v,v,v,255],i*4);});
    ctx.putImageData(image,0,0);
  }else{
    const key=depthPreviewKey, image=new Image();
    ctx.clearRect(0,0,96,72);
    image.onload=()=>{if(key===depthPreviewKey && !depthWorker)ctx.drawImage(image,0,0);};
    image.src='/depth-preview.png?'+params;
  }
}

function replayDepth(){
  if(depthWorker || $('depthPanel').hidden)return;
  const frames=model?.surfaces?.ai?.frames;
  if(!frames?.length){$('depthPreview').hidden=true;return;}
  const id=parseInt(timeline[cursor]?.id||frames[0]);
  const frame=frames.reduce((best,f)=>Math.abs(parseInt(f)-id)<Math.abs(parseInt(best)-id)?f:best);
  const key=model.run+model.feature_index+frame;
  if(depthPreviewKey!==key){depthPreviewKey=key;depthPair(frame);}
}

function stopDepth(message){
  depthRequest++;
  checkingDepth=false;
  depthWorker?.terminate();
  depthWorker=null;
  depthPreviewKey='';
  $('cancelDepth').hidden=true;
  $('depthProgress').textContent=message;
}
$('cancelDepth').onclick=()=>{stopDepth('Depth inference cancelled. Completed views are cached.');replayDepth();};

async function buildDepth(span){
  const candidates=model.poses.filter(p=>p.rotation && model.landmark_views?.[p.frame]?.length>=15);
  if(candidates.length<2){
    $('depthProgress').textContent='This walk needs two mapped camera views with landmark tracks.';
    return;
  }
  const count=Math.min(36,candidates.length), frames=Array.from({length:count},(_,i)=>
    candidates[Math.round(i*(candidates.length-1)/(count-1))].frame);
  const run=model.run,index=model.feature_index,mapVersion=version;
  $('depthProgress').textContent='Preparing local model · first use downloads about 38 MB';
  $('cancelDepth').hidden=false;
  $('rebuildSurfaces').disabled=$('surfaceMethod').disabled=$('surfaceEdge').disabled=true;
  const request=++depthRequest;
  checkingDepth=true;
  let cached=[];
  try{
    const response=await fetch('/depth-cache?'+new URLSearchParams({run,index}));
    if(response.ok)cached=await response.json();
  }catch{}
  if(request!==depthRequest)return;
  checkingDepth=false;
  if(model?.run!==run || version!==mapVersion || $('surfaceMethod').value!==aiMethod){stopDepth('Map changed.');return;}
  if(cached.length===frames.length && frames.every(f=>cached.includes(f))){
    stopDepth('Fitting surfaces from saved depth predictions…');
    await act('rebuild-surfaces',{run,index,version:mapVersion,method:aiMethod,growth_distance_cm:span});
    return;
  }
  const worker=new Worker('/depth-worker.js',{type:'module'});
  depthWorker=worker;
  worker.onerror=e=>stopDepth(e.message||'The local depth runtime could not load. Retry to download the model.');
  worker.onmessage=async({data})=>{
    if(worker!==depthWorker)return;
    if(model?.run!==run || state.version!==mapVersion){
      stopDepth('The selected map changed. Start depth inference again.');
      return;
    }
    if(data.error){stopDepth(data.error);return;}
    if(data.status)$('depthProgress').textContent=data.status;
    if(data.raw)depthPair(data.frame,data.raw);
    if(data.predictions){
      stopDepth('Checking predictions against mapped views and fitting planes…');
      await act('rebuild-surfaces',{run,index,version:mapVersion,method:aiMethod,growth_distance_cm:span,predictions:data.predictions});
      if($('error').textContent)$('depthProgress').textContent=$('error').textContent;
    }
  };
  worker.postMessage({run,index,frames});
}

function growthDistanceCm(mesh){
  return Number(((mesh?.growth_distance_m??mesh?.max_edge_m??.20)*100).toPrecision(12));
}
function showMap(m) {
  if(checkingDepth||depthWorker)stopDepth("The map changed. Start depth inference again.");
  depthPreviewKey="";
  $('depthProgress').textContent="";
  if(m?.surfaces && !['camera-ray-seeds-v1','first-return-cloth-v1','cloud-planes-v1',aiMethod].includes(m.surfaces.method)) delete m.surfaces;
  const sameMap=m && model && m.run===model.run && m.units===model.units &&
    m.feature_index && m.feature_index===model.feature_index;
  const keepLingbot=sameMap && $('surfaceMethod').value==='lingbot-map';
  const keepGaussian=sameMap && $('surfaceMethod').value==='gaussian-map';
  if(!sameMap){clearDepthSurface();$('geometrySource').value='photo';activeDepthCloud=null;++cloudTicket;}
  model=m;
  posesByFrame=new Map((m?.poses||[]).map(p=>[p.frame,p]));
  updateViewControls();
  $('surfaceEdge').value=growthDistanceCm(m?.surfaces);
  const hasTracks=!!Object.keys(m?.landmark_views||{}).length;
  for(const option of $('surfaceMethod').options)option.disabled=(!hasTracks && option.value!=='cloud-planes-v1') || (option.value===aiMethod && !m?.feature_index);
  for(const opt of $('surfaceMethod').options){
    if(['gaussian-map','lingbot-map'].includes(opt.value))opt.disabled=!m?.feature_index;
    else if(m?.units!=='m')opt.disabled=true;
  }
  const relativeGaussian=m?.feature_index && m.units!=='m';
  $('surfaceMethod').value=keepLingbot?'lingbot-map':(keepGaussian || relativeGaussian)?'gaussian-map':hasTracks?(m?.surfaces?.method||'camera-ray-seeds-v1'):'cloud-planes-v1';
  updateSpanLabel();
  updateSurfaceInfo();
  updateGaps();
  updateStreamControls();
  updateReplayRoute();
  $('estimate').disabled=$('continuous').disabled=!m;
  const label=m ? 'Frames '+parseInt(m.poses[0].frame)+'–'+parseInt(m.poses.at(-1).frame)+' · '+state.frames+' saved' : '';
  $('coverage').textContent=label; $('mapempty').hidden=!!m;
  $('poses').textContent=m ? (m.registered_views??m.poses.length)+' / '+m.views : '—';
  $('points').textContent=m ? m.points.length.toLocaleString() : '—';
  $('residual').textContent=m ? m.mean_error_px+' px' : '—';
  $('units').textContent=m?.units==='m'?(m.reference?'Measured baseline · m':m.tof_scale?'ToF scale estimate · m':'Measured scale · m'):'Relative scale';
  if(m?.gravity_aligned) $('units').textContent+=' · IMU aligned';
  if(!sameMap)fitMapView();
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
  const s=await (await fetch('/state')).json(); state=s;
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

  if(!replay) $('rate').textContent=s.fresh?s.fps+' fps · 640 × 480 · '+(s.tofRaw==null?'ToF unavailable':'ToF '+s.tofRaw+' raw (~cm)')+(s.tofHz==null?'':' · '+s.tofHz+' ToF reads/s ('+s.tofValidPercent+'% valid)')+' · IMU '+(s.motionFresh?'live':'unavailable'):'OV5647 · 640 × 480';
  $('frames').textContent=s.frames.toLocaleString(); $('walk').textContent=s.run||'No walk selected';
  $('captureWarning').hidden=!s.fresh||s.motionFresh;
  $('message').textContent=s.message; $('samples').textContent=s.samples+' views'; $('lens').textContent=s.calibrated?'Calibrated':'Self-estimate available';
  $('download').hidden=!s.hasMap;
  updateStreamControls();
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
    const response=run&&await fetch('/timeline?'+new URLSearchParams({run}));
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
  if(s.version!==version) {
    const m=s.hasMap?await(await fetch('/survey.json')).json():null;
    if(mapRun!==s.run){
      setStreams(m?.streams||streamChoices.map(choice=>choice.key));
      mapRun=s.run;
    }
    resetTracking(); showMap(m); version=s.version;
    refreshReplay();
    const list=await(await fetch('/runs')).json(), old=$('runs').value;
    $('runs').replaceChildren(...list.map(n=>new Option(n,n)));
    $('runs').value=list.includes(old)?old:(s.run||list[0]||'');
  }
  const surfaceBusy=checkingDepth || !!depthWorker || s.recording || s.busy || model?.run!==s.run;
  $('surfaceMethod').disabled=$('surfaceEdge').disabled=$('rebuildSurfaces').disabled=surfaceBusy;
  $('buildClearance').disabled=surfaceBusy;
  if(s.busy && $('clearancePanel').open && !$('buildClearance').hidden)$('clearanceStatus').textContent=s.message;
  $('buildGaussianMesh').disabled=$('buildLingbot').disabled=$('trainGaussian').disabled=surfaceBusy;
  if(s.busy && !$('buildGaussianMesh').hidden)$('gaussianStatus').textContent=s.message;
  updateWalkActions();
}
async function act(key,body={}) {
  if(active)return; active=true;
  try{
    const r=await fetch('/'+key,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    const v=await r.json(); if(!r.ok)throw Error(v.error);
    $('error').textContent=''; await update();
  }catch(e){$('error').textContent=e.message;}finally{active=false;}
}
$('connect').onclick=()=>act(['Live','Connecting'].includes(state.connection)?'disconnect':'connect');
$('record').onclick=()=>act(state.recording?'stop':'start');
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
  link.href='/walk.zip?'+new URLSearchParams({run:$('runs').value});
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
    previewPending=true; preview.src='/preview.jpg?t='+Date.now();
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
    const r=await fetch('/locate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({
      run:state.run,source,frame:parseInt(timeline[cursor]?.id),minimum:+$('minimum').value})});
    const q=await r.json();if(token!==generation)return;
    if(!r.ok||q.version!==version)throw Error(q.error||'Map changed');
    const img=new Image();img.src='data:image/jpeg;base64,'+q.image;await img.decode();if(token!==generation)return;
    estimate=q;draw();const el=$('matches'),c=el.getContext('2d');el.hidden=false;c.drawImage(img,0,0);
    c.fillStyle='#38ff9e';for(const [x,y] of q.pixels||[])c.fillRect(x-2,y-2,4,4);
    $('tracking').textContent=q.status+' · '+q.inliers+' inliers / '+q.matches+' matches · '+(source==='live'?'camera '+(q.stamp/1e6).toFixed(2)+' s':'frame '+q.stamp);
  }catch(e){if(token===generation){resetTracking();$('tracking').textContent=e.message;}}
  finally{estimating=false;}
}
$('estimate').onclick=locate;$('continuous').onclick=()=>{tracking=!tracking;$('continuous').textContent=tracking?'Pause tracking':'Start tracking';if(tracking)locate();};
setInterval(()=>{if(tracking&&!document.hidden)locate();},750);
$('source').onchange=resetTracking;
function updateSurfaceInfo(){
  const patches=model?.surfaces?.patches||[];
  const toggle=$('surfaceLayer'), info=$('surfaceInfo');
  toggle.disabled=!patches.length;
  const gaussian=$('surfaceMethod').value==='gaussian-map';
  $('surfaceSettings').hidden=!model?.points.length || (model.units!=='m' && !model.feature_index);
  const fromDepth=$('geometrySource').value!=='photo';
  $('surfaceMethod').hidden=$('methodTitle').hidden=fromDepth;
  $('geometryReport').hidden=!fromDepth;
  $('depthSurfaceControls').hidden=!fromDepth||!model;
  $('buildDepthSurfaces').disabled=!activeDepthCloud||surfacePending||!$('depthSurfaceMethod').value;
  $('depthSurfaceTof').disabled=model?.units!=='m';
  if(model?.units!=='m')$('depthSurfaceTof').checked=false;
  $('methodConfig').hidden=$('surfaceSettings').hidden||fromDepth;
  if(fromDepth){toggle.disabled=!depthSurfaceMesh?.patches.length;info.hidden=$('surfaceTimeControl').hidden=true;return;}
  if(gaussian || $('surfaceMethod').value==='lingbot-map'){
    toggle.disabled=true;
    $('surfaceTimeControl').hidden=info.hidden=true;
    return;
  }
  $('surfaceTimeControl').hidden=!patches.length || !toggle.checked;
  info.hidden=!(model&&(!patches.length||toggle.checked));
  const mesh=model?.surfaces;
  $('vertexKey').textContent=mesh?.method==='cloud-planes-v1'?'Plane support':'Surface vertices';
  if(!mesh){ info.textContent='Rebuild this walk to grow surfaces from its landmark tracks.';return; }
  if(mesh.status==='needs_scale'){
    info.textContent='Surface growth needs metric scale. Under Metric scale, mark two mapped replay frames and enter their measured separation.';
    return;
  }
  if(mesh.status==='needs_tracks'){info.textContent='Rebuild this walk to associate landmarks with the cameras that observed them.';return;}
  if(mesh.method===aiMethod){
    const a=mesh.ai;
    info.textContent=a?'Depth Anything V2 · '+patches.length+' inferred planes · '+a.aligned_views+'/'+a.frames.length+
      ' views passed depth checks · '+a.confirmed_points.toLocaleString()+' cross-view samples · ToF rejections '+a.tof_rejected:
      'Rebuild surfaces to predict depth for this camera map.';
    if(a && !patches.length)info.textContent+=' · Insufficient consistent planar support';
    return;
  }
  if(mesh.method==='cloud-planes-v1'){
    info.textContent='Cloud planes · '+patches.length+' polygons · '+mesh.support_count.toLocaleString()+
      ' supporting landmarks · '+Math.round(mesh.tolerance_m*1000)+' mm fitting tolerance';
    if(mesh.indoor){
      const e=mesh.indoor;
      info.textContent+=' · '+e.regularized+' aligned to room directions · '+e.range_matches+'/'+e.intersected+
        ' intersecting ToF rays agree · '+e.range_adjusted+' range-adjusted plates · '+e.attitude_samples+' paired attitudes · '+e.height_samples+
        ' barometric heights · Vertical direction '+e.up_source;
    }
    return;
  }
  if(mesh.method==='first-return-cloth-v1'){
    info.textContent='Ray cloth · '+patches.length+' triangles · '+growthDistanceCm(mesh)+' cm maximum span · '+
      mesh.unconfirmed+' landmarks lacked a second viewpoint · '+mesh.isolated+' lacked nearby support';
    return;
  }
  info.textContent='Seed growth · '+patches.length+' triangles · '+mesh.seeds+' single seeds · '+mesh.open_pairs+' open pairs · '+
    mesh.deferred_landmarks.length+' landmarks still occluded · '+growthDistanceCm(mesh)+(mesh.growth_distance_m==null?' cm maximum edge · rebuild to use growth distance':' cm growth distance');

}
let gaussianMeshViewer, gaussianMeshReady=false, loadedMeshUrl, loadedSplatUrl;
let gaussianViewer, gaussianReady=false, gaussianRequest=0;
function gaussianUrl(file){
  return '/gaussian-artifact?'+new URLSearchParams({run:model.run,index:model.feature_index,file});
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
  const open=model && $('surfaceMethod').value==='gaussian-map' && $('clearancePanel').open;
  const key=open?gaussianUrl('clearance-v1/report.json')+'&version='+state.version:'';
  if(clearanceKey===key)return;
  clearanceKey=key;clearanceReport=null;
  $('clearanceControls').hidden=$('buildClearance').hidden=true;
  if(!key)return;
  const status=$('clearanceStatus');
  status.textContent='Opening clearance map…';
  try{
    const response=await fetch(key);
    if(clearanceKey!==key)return;
    if(response.status===404){
      status.textContent=state.message?.startsWith('Walk clearance failed')?state.message:
        'Build from the trained Gaussian map and recorded ToF sightlines.';
      $('buildClearance').hidden=false;return;
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
$('buildClearance').onclick=()=>act('build-clearance',{run:model.run,index:model.feature_index});
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
  const enabled=model && $('surfaceMethod').value==='gaussian-map' && $('tofEvidenceToggle').checked;
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

function gaussianPair(){
  const frame=$('gaussianTestFrame').value;
  if(!frame)return;
  $('gaussianPhoto').src=gaussianUrl('reference/'+frame);
  $('gaussianRender').src=gaussianUrl('renders/'+frame.replace('.jpg','.png'));
}
async function loadGaussian(){
  const ticket=++gaussianRequest;
  refreshTofEvidence();
  gaussianMeshReady=gaussianReady=false;
  $('buildGaussianMesh').hidden=true;
  $('gaussianEval').hidden=$('gaussianDownload').hidden=$('trainGaussian').hidden=true;
  $('gaussianStatus').textContent='Opening Gaussian map…';
  draw();
  try{
    const response=await fetch(gaussianUrl('report.json'));
    if(ticket!==gaussianRequest)return;
    if(response.status===404){
      $('gaussianStatus').textContent='Build from the registered camera views on this Mac.';
      $('trainGaussian').hidden=false; return;
    }
    if(!response.ok)throw Error('Could not open the Gaussian map.');
    const report=await response.json();
    if(ticket!==gaussianRequest)return;
    const mode=$('gaussianMode').value, mesh=mode==='mesh' || mode==='overlay';
    const file=mesh?'mesh-v1/mesh.ply':mode, url=gaussianUrl(file);
    $('gaussianTestFrame').replaceChildren(...report.evaluation.map(e=>new Option(parseInt(e.frame)+' · '+e.psnr_db+' dB',e.frame)));
    gaussianPair(); $('gaussianEval').hidden=false;
    let message;
    if(mesh){
      const response=await fetch(gaussianUrl('mesh-v1/report.json'));
      if(ticket!==gaussianRequest)return;
      if(response.status===404){
        $('gaussianStatus').textContent='Extract surfaces from the trained splats on this Mac.';
        $('buildGaussianMesh').hidden=false;
        return;
      }
      if(!response.ok)throw Error('Could not open the mesh report.');
      const stats=await response.json();
      if(ticket!==gaussianRequest)return;
      if(loadedMeshUrl!==url){
        const result=await fetch(url);
        if(!result.ok)throw Error('Could not load the mesh.');
        const buffer=await result.arrayBuffer();
        if(ticket!==gaussianRequest)return;
        gaussianMeshViewer??=new GaussianMeshView();
        gaussianMeshViewer.load(buffer); loadedMeshUrl=url;
      }
      const tof=stats.tof;
      message=stats.triangles.toLocaleString()+' triangles · '+stats.planar_regions+' planar regions · '+stats.views+' registered views · '+
        (stats.units==='m'?'metric map':'relative scale')+' · ToF hits '+(tof.hits||0)+'/'+(tof.eligible||0)+
        (tof.median_error_m!=null?' · median difference '+(100*tof.median_error_m).toFixed(1)+' cm'+
          (stats.units==='m'?'':' after provisional scaling')+' · 80th percentile '+(100*tof.p80_error_m).toFixed(1)+' cm':'')+' · '+(tof.reason||tof.beam);
      gaussianMeshReady=true;
    }
    if(!mesh || mode==='overlay'){
      const splatUrl=gaussianUrl(mesh?'gaussians.ply':file), cloud=file==='cloud.ply';
      if(loadedSplatUrl!==splatUrl){
        const ply=await fetch(splatUrl);
        if(!ply.ok)throw Error('Could not load the PLY file.');
        const buffer=await ply.arrayBuffer();
        if(ticket!==gaussianRequest)return;
        gaussianViewer??=new GaussianView();
        gaussianViewer.load(buffer,cloud); loadedSplatUrl=splatUrl;
      }
      gaussianReady=true;
      if(!mesh)message=gaussianViewer.order.length.toLocaleString()+(cloud?' colored points':' trained splats')+' · '+report.train_views+' training views · '+report.test_views+' withheld · '+report.mean_psnr_db+' dB PSNR · saved camera poses';
    }
    $('gaussianStatus').textContent=message;
    $('gaussianDownload').hidden=false; $('gaussianDownload').href=url;
    $('gaussianDownload').download=file.split('/').at(-1);
    draw();
  }catch(error){if(ticket===gaussianRequest)$('gaussianStatus').textContent=error.message;}
}
$('gaussianMode').onchange=loadGaussian;
$('gaussianTestFrame').onchange=gaussianPair;
$('buildGaussianMesh').onclick=()=>act('build-gaussian-mesh',{run:model.run,index:model.feature_index});
$('trainGaussian').onclick=()=>act('build-gaussians',{run:model.run,index:model.feature_index});
let lingbotRequest=0;
$('buildLingbot').onclick=()=>act('build-lingbot',{run:model.run,index:model.feature_index});
async function updateLingbot(selected){
  const ticket=++lingbotRequest, viewer=$('lingbotView');
  $('lingbotPanel').hidden=!selected;
  $('mapHelp').closest('.legend').hidden=selected;
  $('route').closest('.map-controls').hidden=selected;
  viewer.hidden=true;
  if(!selected){viewer.removeAttribute('src');return;}
  $('lingbotStatus').textContent='Opening reconstruction…';
  $('buildLingbot').hidden=$('lingbotCloud').hidden=true;
  const url=file=>'/lingbot-artifact?'+new URLSearchParams({run:model.run,index:model.feature_index,file});
  try{
    const response=await fetch(url('trajectory.json'));
    if(ticket!==lingbotRequest)return;
    if(response.status===404){
      $('lingbotStatus').textContent='Sample every eighth image across this walk.';
      $('buildLingbot').hidden=false;return;
    }
    if(!response.ok)throw Error('Could not open the saved trajectory.');
    const report=await response.json();
    if(ticket!==lingbotRequest)return;
    $('lingbotStatus').textContent=report.views+' predicted poses · '+report.points.toLocaleString()+' depth samples · relative scale';
    viewer.src=url('view.html');viewer.hidden=false;
    $('lingbotCloud').href=url('cloud.ply');$('lingbotCloud').hidden=false;
  }catch(error){if(ticket===lingbotRequest)$('lingbotStatus').textContent=error.message;}
}
function updateSpanLabel(){
  const fromPhoto=$('geometrySource').value==='photo';
  $('cloudLabel').textContent=fromPhoto?'Landmarks':'Depth points';
  const lingbot=fromPhoto&&$('surfaceMethod').value==='lingbot-map';
  updateLingbot(lingbot);
  const gaussian=fromPhoto&&$('surfaceMethod').value==='gaussian-map';
  $('gaussianPanel').hidden=!gaussian;
  refreshClearance();
  for(const id of ['surfaceSpanLabel','surfaceEdge','rebuildSurfaces'])$(id).hidden=gaussian || lingbot;
  if(gaussian && model)loadGaussian();
  else{++gaussianRequest;gaussianMeshReady=gaussianReady=false;refreshTofEvidence();}
  $('depthPanel').hidden=$('surfaceMethod').value!==aiMethod;
  replayDepth();
  const labels={[aiMethod]:'Maximum landmark gap (cm)','cloud-planes-v1':'Maximum landmark gap (cm)','first-return-cloth-v1':'Maximum span (cm)'};
  $('surfaceSpanLabel').textContent=labels[$('surfaceMethod').value]||'Growth distance (cm)';
}
$('surfaceMethod').onchange=()=>{updateSpanLabel();updateSurfaceInfo();draw();};
$('rebuildSurfaces').onclick=()=>{
  const edge=$('surfaceEdge');
  edge.setCustomValidity(edge.valueAsNumber<=0?'Enter a positive growth distance.':'');
  if(!edge.reportValidity())return;
  $('surfaceLayer').checked=true;
  if($('surfaceMethod').value===aiMethod){buildDepth(edge.valueAsNumber);return;}
  act('rebuild-surfaces',{run:model.run,method:$('surfaceMethod').value,growth_distance_cm:edge.valueAsNumber});
};
$('surfaceEdge').oninput=()=>{$('surfaceEdge').setCustomValidity('');};
$('surfaceEdge').onkeydown=e=>{
  if(e.key==='Enter')$('rebuildSurfaces').click();
};
$('surfaceTime').onchange=draw;
$('surfaceLayer').onchange=()=>{updateSurfaceInfo();draw();};
$('cloud').onchange=draw;
$('route').onchange=()=>{updateRouteLegend();draw();};
function updateViewControls(){
  $('droneView').disabled=!model || !timeline.length;
  if($('droneView').disabled)droneView=false;
  $('droneView').setAttribute('aria-pressed',droneView);
  $('mapHelp').textContent=droneView?'Drag to look around · scrub the timeline to move · scroll to zoom':
    'Drag to orbit · ⌘ or ⌥ + drag to pan · scroll to zoom';
}

function setEnlarged(enlarged){
  const views=$('views'), camera=$('cameraPanel');
  views.classList.toggle('enlarged',enlarged);
  if(enlarged)$('mapViewport').append(camera);
  else views.prepend(camera);
  $('enlarge').textContent=enlarged?'Collapse':'Enlarge';
  $('enlarge').setAttribute('aria-expanded',enlarged);
}

$('enlarge').onclick=()=>setEnlarged(!$('views').classList.contains('enlarged'));
$('droneView').onclick=()=>{
  droneView=!droneView;
  if(droneView){
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
$('side').onclick=()=>setOverview(0,0);
$('top').onclick=()=>setOverview(0,Math.PI/2);
$('iso').onclick=()=>setOverview(-.38,.42);
$('range').onclick=async()=>{try{const r=await fetch('/range',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({address:Number($('address').value)})});
const q=await r.json();$('reading').textContent=r.ok?'Raw reading '+q.raw:q.error;}catch(e){$('reading').textContent=e.message;}};
const sensorTypes=[
  ['tof','ToF','m',true], ['gyro','Gyroscope','rad/s',true],
  ['accel','Acceleration','m/s²',true], ['height','Barometric height','m',true],
  ['attitude','Attitude quaternion','',false],
];
const sensorFields=sensorTypes.map(([key,name,unit,on])=>{
  const toggle=document.createElement('input'); toggle.type='checkbox';toggle.checked=on;
  const label=document.createElement('label');label.append(toggle,' '+name);$('sensorToggles').append(label);
  const field=document.createElement('div'), title=document.createElement('small'), output=document.createElement('output');
  title.textContent=name;field.append(title,output);$('sensorValues').append(field);
  toggle.onchange=()=>{field.hidden=!toggle.checked;};field.hidden=!on;
  return {key,unit,output};
});
let depthSurfaceMesh=null,surfaceTicket=0,surfacePending=false;
function clearDepthSurface(){
  ++surfaceTicket;surfacePending=false;depthSurfaceMesh=null;
  $('depthSurfaceStatus').textContent='';
}
function surfaceOptionsChanged(){
  clearDepthSurface();updateSurfaceInfo();draw();
}
$('depthSurfaceMethod').onchange=$('depthSurfaceTof').onchange=surfaceOptionsChanged;
fetch('/depth-surface-methods').then(r=>r.json()).then(methods=>{
  $('depthSurfaceMethod').replaceChildren(...Object.entries(methods).map(([id,label])=>new Option(label,id)));
  updateSurfaceInfo();
}).catch(()=>{$('depthSurfaceStatus').textContent='Surface methods unavailable. Reload the dashboard.';});
$('buildDepthSurfaces').onclick=async()=>{
  const ticket=++surfaceTicket, source=$('geometrySource').value;
  surfacePending=true;updateSurfaceInfo();
  $('depthSurfaceStatus').textContent='Generating surfaces…';
  try{
    const response=await fetch('/depth-surfaces',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({run:model.run,index:model.feature_index,source,method:$('depthSurfaceMethod').value,tof:$('depthSurfaceTof').checked})});
    const result=await response.json();
    if(ticket!==surfaceTicket)return;
    if(!response.ok)throw Error(result.error||'Surface generation failed');
    depthSurfaceMesh=result;
    $('surfaceLayer').checked=result.patches.length>0;
    $('depthSurfaceStatus').textContent=result.patches.length.toLocaleString()+' polygons · '+result.points.toLocaleString()+' input points · '+result.seconds.toFixed(2)+' s'+
      (result.tof?' · '+result.tof.used+'/'+result.tof.paired+' paired ToF returns used · '+result.tof.mount:'');
  }catch(e){if(ticket===surfaceTicket)$('depthSurfaceStatus').textContent=e.message;}
  finally{if(ticket===surfaceTicket){surfacePending=false;updateSurfaceInfo();draw();}}
};
let activeDepthCloud=null, cloudTicket=0, rightKey='';
for(const opt of $('frameMethod').options){
  $('rightSource').add(new Option(opt.text,opt.value));
  $('geometrySource').add(new Option(opt.value==='photo'?'Recorded photos':opt.text,opt.value));
}
$('rightSource').value='apple';
function timingText(t){
  if(!t)return '';
  const stages=Object.entries(t.stages||{}).map(([name,seconds])=>name+' '+seconds.toFixed(2)+' s');
  return [t.seconds.toFixed(2)+' s total',t.frames?Math.round(1000*t.seconds/t.frames)+' ms/frame · '+(t.frames/t.seconds).toFixed(1)+' fps':'',...stages].filter(Boolean).join(' · ');
}
$('geometrySource').onchange=async()=>{
  const method=$('geometrySource').value, ticket=++cloudTicket;
  activeDepthCloud=null;clearDepthSurface();
  updateSpanLabel();updateSurfaceInfo();draw();
  if(method==='photo'||!model)return;
  const report=$('geometryReport');
  report.textContent='Backprojecting depth with saved camera poses…';
  $('cloud').checked=true;
  try{
    const response=await fetch('/depth-cloud',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({run:model.run,index:model.feature_index,method})});
    const result=await response.json();
    if(ticket!==cloudTicket)return;
    if(!response.ok)throw Error(result.error||'Depth reconstruction unavailable');
    activeDepthCloud=result;updateSurfaceInfo();
    report.textContent='Depth point cloud · '+result.points.length.toLocaleString()+' displayed · '+result.used+'/'+result.poses+
      ' oriented frames contributed · '+timingText({...result,frames:result.poses});
    draw();
  }catch(e){if(ticket===cloudTicket)report.textContent=e.message;}
};
function comparisonUrl(method,frame){
  const query=depthQuery();
  const endpoint=method==='photo'?'/depth-input.jpg?':'/frame-depth.png?';
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
      const res=await fetch('/frame-depth-status?'+new URLSearchParams({...query,method:opt.value}));
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
    const res=await fetch('/frame-depth-status?'+new URLSearchParams({...query,method:query.method==='photo'?'all':query.method}));
    if(!res.ok)return;
    const next=await res.json();
    if(key!==depthStatusKey)return;
    const retry=next.saved!==frameDepthStatus.saved&&!$('depthMissing').hidden;
    frameDepthStatus=next; describeFrameDepth();
    if(retry){displayedFrame=null;refreshReplay();}
  }catch{}
}
$('frameMethod').onchange=()=>{displayedFrame=null;refreshReplay();};
$('buildFrameDepth').onclick=()=>{const query=depthQuery();return act('build-frame-depth',{...query,method:query.method==='photo'?'all':query.method});};
$('replayImage').onerror=()=>{if(replay){$('replayImage').style.visibility='hidden';$('depthMissing').hidden=false;}};
$('replayImage').onload=()=>{$('replayImage').style.visibility='';$('depthMissing').hidden=true;};
function updateCamera(){
  $('cameraTitle').textContent=replay?'Frame replay':'Live camera';
  $('frameControls').hidden=!replay;
  refreshComparison(timeline[cursor]);
  if(replay){pollFrameDepth();refreshProfile();}
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
  const registered=new Map(model?.poses.map(p=>[p.frame,p.method]));
  for(const frame of timeline){
    const inferred=selectedStreams().includes('camera')&&replayEstimates.get(frame.id);
    g.fillStyle=inferred?routeStyles[inferred.kind].color:registered.has(frame.id)?(registered.get(frame.id)?'#72caff':'#a5efd1'):'#556469';
    g.fillRect(x(frame.ms),3,2,7);
  }
  g.fillStyle='#72caff';g.fillRect(x(timeline[cursor].ms)-1,0,3,13);
}
new ResizeObserver(drawTimeline).observe($('timelinePlot'));
function refreshReplay(){
  syncClearance();
  syncTofReading();
  replayDepth();
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
      $('replayImage').src=compare?comparisonUrl(query.method,frame.id):(query.method==='photo'?'/recorded.jpg?':'/frame-depth.png?')+new URLSearchParams({...query,frame:frame.id});
    }else{
      $('replayImage').removeAttribute('src');
      $('depthMissing').hidden=false;
    }
  }
  const seconds=ms=>(ms/1000).toFixed(2)+' s';
  $('frameNo').textContent=seconds(frame.ms)+' / '+seconds(timeline.at(-1).ms)+' · Frame '+parseInt(frame.id);
  $('rate').textContent='Recorded · '+(frame.tof>0&&frame.tof<250?'ToF '+(frame.tof/104).toFixed(2)+' m':'ToF unavailable')+
    ' · '+(frame.gyro?'IMU paired':'No paired IMU');
  const hasPose=posesByFrame.get(frame.id), inferred=replayEstimates.get(frame.id);
  $('timelineStatus').textContent=hasPose
    ? (hasPose.method==='aligned_window'?'Pose aligned from overlapping views':hasPose.method?'Pose recovered from nearby frames':'Pose registered in the map')
    : inferred?.held?'Estimated path · held at nearest mapped pose'
    : inferred?.kind==='interpolated'?'Estimated path · straight line between mapped poses'
    : inferred?.kind==='extrapolated'?'Estimated path · extrapolated from mapped motion'
    : model?.sampled_frames?.includes(frame.id)
      ? 'Unmapped · sampled but pose unresolved in the main map'
      : 'Unmapped · outside the initial sample and no recovery passed';
  const visual=selectedStreams().includes('camera');
  $('gapPicker').hidden=!visual||!gapRanges.length;
  $('inspectGap').hidden=!visual||!!hasPose||!model?.feature_index;
  $('inspectGap').disabled=!!state?.busy;
  if(!visual) $('timelineStatus').textContent='Sensor-only view';
  drawTraces();
  for(const {key,unit,output} of sensorFields){
    const v=frame[key];
    if(key==='tof'){
      output.textContent=v==null?'Unavailable':v>0&&v<250?(v/104).toFixed(2)+' m · '+v+' raw':'Invalid · '+v+' raw';
    }else if(Array.isArray(v)){
      const axes=key==='attitude'?['w','x','y','z']:['x','y','z'];
      output.textContent=v.map((n,i)=>axes[i]+' '+(n==null?'—':n.toFixed(3))).join(' · ')+' '+unit;
    }else{
      output.textContent=v==null?'Unavailable':v.toFixed(2)+' '+unit;
    }
  }
  $('sampleTiming').textContent=frame.lag_ms==null?'No paired motion sample.':'Motion receipt offset '+frame.lag_ms+' ms from camera callback.';
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
$('cameraMode').onclick=()=>{pauseReplay();replay=!replay;if(!replay)droneView=false;refreshReplay();};
const streamNames={camera:'Camera',tof:'ToF',gyro:'Gyroscope',accel:'Accelerometer',barometer:'Barometer'};
const traceConfig={
  tof:['Range along beam · m', f=>f.tof>0&&f.tof<250?[f.tof/104]:[]],
  gyro:['Angular rate XYZ · rad/s', f=>f.gyro||[]],
  accel:['Specific force XYZ · m/s²', f=>f.accel||[]],
  barometer:['Estimated height · m', f=>[f.height]],
};
const streamChoices=Object.entries(streamNames).map(([key,name])=>{
  const input=document.createElement('input');
  input.type='checkbox';input.checked=true;
  const label=document.createElement('label');label.append(input,' '+name);
  $('inputStreams').append(label);
  input.onchange=()=>{clearGapEvidence();updateStreamControls();refreshReplay();};
  return {key,input};
});
function selectedStreams(){return streamChoices.filter(({input})=>input.checked).map(({key})=>key);}
function setStreams(keys){for(const {key,input} of streamChoices) input.checked=keys.includes(key);}
function updateStreamControls(){
  const selected=selectedStreams(), visual=selected.includes('camera');
  $('reconstructionInputs').hidden=!state?.run||!!state?.recording;
  $('rebuild').hidden=!visual;
  $('rebuild').disabled=!state?.run||state.busy||state.recording;
  $('sensorOnlyNote').hidden=visual;
  if(!visual && $('views').classList.contains('enlarged'))setEnlarged(false);
  $('mapPanel').hidden=!visual;
  $('posePanel').hidden=!visual;
  const used=model?.streams;
  const saved=used?'Saved map inputs · '+used.map(key=>streamNames[key]).join(', '):
    model?'Saved map inputs · legacy reconstruction':'No saved reconstruction';
  $('streamSummary').textContent=visual?((used||Object.keys(streamNames)).join(',')!==selected.join(',')?'Selection changed · rebuild to apply. ':'')+saved:
    selected.length?'Selected traces · '+selected.map(key=>streamNames[key]).join(', '):'Select a sensor to inspect its trace.';
  const traceKeys=selected.filter(key=>key!=='camera');
  $('sensorTraces').hidden=!traceKeys.length;
  if($('sensorTraces').dataset.keys!==traceKeys.join(',')){
    $('sensorTraces').dataset.keys=traceKeys.join(',');
    $('sensorTraces').replaceChildren(...traceKeys.map(key=>{
      const wrapper=document.createElement('div'), title=document.createElement('p'), plot=document.createElement('canvas');
      title.textContent=traceConfig[key][0];plot.dataset.sensor=key;
      plot.setAttribute('aria-label',title.textContent);
      wrapper.append(title,plot);return wrapper;
    }));
  }
}
$('rebuild').onclick=()=>act('rebuild',{streams:selectedStreams()});
function drawTraces(){
  if($('sensorTraces').hidden||!timeline.length)return;
  for(const plot of $('sensorTraces').querySelectorAll('canvas')){
    const width=plot.clientWidth, height=plot.clientHeight, d=devicePixelRatio;
    if(!width||!height)continue;
    const points=timeline.map(traceConfig[plot.dataset.sensor][1]);
    const values=points.flat().filter(Number.isFinite);
    plot.width=width*d;plot.height=height*d;
    const g=plot.getContext('2d');g.scale(d,d);
    if(!values.length){g.fillStyle='#a2b3b6';g.font='12px system-ui';g.fillText('No valid readings',12,20);continue;}
    const [lo,hi]=values.reduce(([a,b],v)=>[Math.min(a,v),Math.max(b,v)],[Infinity,-Infinity]);
    const x=ms=>54+(width-60)*ms/Math.max(1,timeline.at(-1).ms);
    const y=v=>height-9-(height-18)*(hi===lo?.5:(v-lo)/(hi-lo));
    g.fillStyle='#a2b3b6';g.font='11px monospace';g.fillText(hi.toFixed(2),5,12);g.fillText(lo.toFixed(2),5,height-5);
    const colors=['#a5efd1','#ffba7c','#c3a9ff'];
    colors.slice(0,points.reduce((n,p)=>Math.max(n,p.length),0)).forEach((color,axis)=>{
      g.strokeStyle=color;g.beginPath();
      let previous=false;
      points.forEach((values,i)=>{
        const value=values[axis];
        if(Number.isFinite(value)){
          const px=x(timeline[i].ms), py=y(value);
          if(previous)g.lineTo(px,py);else g.moveTo(px,py);
        }
        previous=Number.isFinite(value);
      });
      g.stroke();
    });
    g.strokeStyle='#72caff';g.beginPath();g.moveTo(x(timeline[cursor].ms),0);g.lineTo(x(timeline[cursor].ms),height);g.stroke();
  }
}
new ResizeObserver(drawTraces).observe($('sensorTraces'));
function updateGaps(){
  const mapped=new Set(model?.poses.map(p=>p.frame));
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
    const response=await fetch('/locate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({
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
poll();
