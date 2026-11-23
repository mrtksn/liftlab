// A sorted, alpha-composited projection of the trained 3D Gaussian ellipsoids.
// Splats fainter than one 8-bit step don't show; Send to World leaves out the same ones (graph.js).
const GAUSSIAN_MIN_ALPHA=1/255;
class GaussianView {
  constructor() {
    this.canvas = document.createElement('canvas');
    const gl = this.canvas.getContext('webgl2', {alpha:true, antialias:false});
    if (!gl) throw Error('The Gaussian view requires WebGL 2.');
    this.gl = gl;
    const vs = `#version 300 es
    precision highp float;
    precision highp int;
    in uint id;
    uniform highp sampler2D scene;
    uniform vec3 eye, rightAxis, downAxis, forwardAxis;
    uniform vec2 viewport, origin;
    uniform float focal, nearZ;
    uniform bool perspective, cloud;
    out vec2 offset;
    out vec4 color;
    vec4 field(int i) {
      int t=int(id)*4+i;
      return texelFetch(scene,ivec2(t%1024,t/1024),0);
    }
    void main() {
      vec4 mean=field(0);
      vec3 rel=mean.xyz-eye;
      float z=dot(rel,forwardAxis);
      vec2 xy=vec2(dot(rel,rightAxis),dot(rel,downAxis));
      if(perspective && z<nearZ) { gl_Position=vec4(2,2,0,1); return; }
      float scale=perspective?focal/z:focal;
      vec2 pixel=origin+xy*scale;
      vec3 jx=scale*rightAxis, jy=scale*downAxis;
      if(perspective){
        vec2 bounded=clamp(xy/z,(-0.15*viewport-origin)/focal,(1.15*viewport-origin)/focal);
        jx-=scale*bounded.x*forwardAxis; jy-=scale*bounded.y*forwardAxis;
      }
      vec4 q=field(3);
      vec3 v=q.yzw;
      // Each column is a scaled principal axis of the world-space ellipsoid.
      mat3 skew=mat3(0,v.z,-v.y,-v.z,0,v.x,v.y,-v.x,0);
      mat3 rotation=mat3(1)+2.*(q.x*skew+skew*skew);
      vec3 radii=field(2).xyz;
      vec3 u=(jx*rotation)*radii, w=(jy*rotation)*radii;
      float a=dot(u,u)+0.3, b=dot(u,w), c=dot(w,w)+0.3;
      if(cloud){a=0.55; b=0.; c=0.55;}
      // Cholesky factor of the projected covariance.
      float l=sqrt(a), m=b/l, n=sqrt(max(0.0001,c-m*m));
      offset=3.*vec2((gl_VertexID==1||gl_VertexID==3)?1.:-1.,gl_VertexID>=2?1.:-1.);
      pixel+=vec2(l*offset.x,m*offset.x+n*offset.y);
      gl_Position=vec4(pixel/viewport*vec2(2,-2)+vec2(-1,1),0,1);
      color=vec4(field(1).rgb,mean.w);
    }`;
    const fs = `#version 300 es
    precision highp float;
    in vec2 offset; in vec4 color; out vec4 pixel;
    void main(){
      float alpha=min(0.99,color.a*exp(-0.5*dot(offset,offset)));
      if(alpha<${GAUSSIAN_MIN_ALPHA.toFixed(6)}) discard;
      pixel=vec4(color.rgb*alpha,alpha);
    }`;
    this.program=gl.createProgram();
    for(const [type,source] of [[gl.VERTEX_SHADER,vs],[gl.FRAGMENT_SHADER,fs]]){
      const shader=gl.createShader(type);
      gl.shaderSource(shader,source); gl.compileShader(shader);
      if(!gl.getShaderParameter(shader,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(shader));
      gl.attachShader(this.program,shader); gl.deleteShader(shader);
    }
    gl.linkProgram(this.program);
    if(!gl.getProgramParameter(this.program,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(this.program));
    gl.useProgram(this.program);
    this.uniform=Object.fromEntries(['eye','rightAxis','downAxis','forwardAxis','origin','viewport','nearZ','focal','perspective','cloud'].map(k=>[k,gl.getUniformLocation(this.program,k)]));
    this.orderBuffer=gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER,this.orderBuffer);
    const id=gl.getAttribLocation(this.program,'id');
    gl.enableVertexAttribArray(id); gl.vertexAttribIPointer(id,1,gl.UNSIGNED_INT,0,0); gl.vertexAttribDivisor(id,1);
    this.texture=gl.createTexture(); gl.bindTexture(gl.TEXTURE_2D,this.texture);
    gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.NEAREST);
    gl.enable(gl.BLEND); gl.blendFunc(gl.ONE,gl.ONE_MINUS_SRC_ALPHA);
  }
  load(buffer,cloud){
    const bytes=new Uint8Array(buffer);
    const start=new TextDecoder().decode(bytes.subarray(0,4096));
    const header=start.match(/^ply\r?\n[\s\S]*?end_header\r?\n/);
    if(!header || !header[0].includes('format binary_little_endian 1.0'))throw Error('Unsupported map file.');
    const fields={}, lines=header[0].trim().split(/\r?\n/);
    let stride=0,count=0;
    for(const line of lines){
      const [kind,type,name]=line.split(' ');
      if(kind==='element' && type==='vertex')count=Number(name);
      if(kind==='property'){
        if(!['float','uchar'].includes(type))throw Error('Unsupported map properties.');
        fields[name]=stride; stride+=type==='float'?4:1;
      }
    }
    if(!count || header[0].length+stride*count!==bytes.length)throw Error('Incomplete map file.');
    const gl=this.gl, rows=Math.ceil(count/256);
    if(rows>gl.getParameter(gl.MAX_TEXTURE_SIZE))throw Error('Map exceeds this GPU’s texture limit.');
    const data=new DataView(buffer,header[0].length), texels=new Float32Array(rows*4096);
    this.xyz=new Float32Array(count*3);
    this.order=Uint32Array.from({length:count},(_,i)=>i);
    this.depth=new Float32Array(count);
    const read=(i,k)=>data.getFloat32(i*stride+fields[k],true);
    for(let i=0;i<count;i++){
      const base=i*16;
      for(const [j,k] of ['x','y','z'].entries())this.xyz[3*i+j]=texels[base+j]=read(i,k);
      texels[base+3]=cloud?1:1/(1+Math.exp(-read(i,'opacity')));
      for(const [j,k] of ['red','green','blue'].entries()){
        texels[base+4+j]=cloud?data.getUint8(i*stride+fields[k])/255:Math.max(0,Math.min(1,.5+.28209479*read(i,'f_dc_'+j)));
        texels[base+8+j]=cloud?0:Math.exp(read(i,'scale_'+j));
      }
      if(cloud){texels[base+12]=1;}else{
        const q=[0,1,2,3].map(j=>read(i,'rot_'+j)), n=Math.hypot(...q)||1;
        q.forEach((v,j)=>texels[base+12+j]=v/n);
      }
    }
    return this.upload(texels,cloud);
  }
  upload(texels,cloud){
    const gl=this.gl, rows=texels.length/4096;
    gl.texImage2D(gl.TEXTURE_2D,0,gl.RGBA32F,1024,rows,0,gl.RGBA,gl.FLOAT,texels);
    this.sorted=null; this.cloud=cloud; this.attributes=texels;
    return this.order.length;
  }
  loadAttributes(values){
    const count=values.length/16,rows=Math.ceil(count/256);
    if(!count||rows>this.gl.getParameter(this.gl.MAX_TEXTURE_SIZE))throw Error('Invalid Gaussian texture size.');
    const packed=new Float32Array(rows*4096);packed.set(values);
    this.xyz=new Float32Array(count*3);
    for(let i=0;i<count;i++)this.xyz.set(values.subarray(i*16,i*16+3),i*3);
    this.order=Uint32Array.from({length:count},(_,i)=>i);this.depth=new Float32Array(count);
    return this.upload(packed,false);
  }
  paint(ctx,width,height,eye,axes,origin,focal,perspective,near){
    if(!(width>0&&height>0))return;   // (a hidden viewer has no size to draw into)
    const gl=this.gl,u=this.uniform;
    if(this.canvas.width!==width || this.canvas.height!==height){this.canvas.width=width;this.canvas.height=height;}
    gl.viewport(0,0,width,height); gl.clear(gl.COLOR_BUFFER_BIT);
    const [right,down,forward]=axes;
    if(!this.sorted || forward.some((v,j)=>Math.abs(this.sorted[j]-v)>1e-5)){
      const [x,y,z]=forward;
      for(let i=0;i<this.order.length;i++)this.depth[i]=this.xyz[3*i]*x+this.xyz[3*i+1]*y+this.xyz[3*i+2]*z;
      this.order.sort((a,b)=>this.depth[b]-this.depth[a]);
      gl.bufferData(gl.ARRAY_BUFFER,this.order,gl.DYNAMIC_DRAW); this.sorted=forward.slice();
    }
    gl.uniform3fv(u.eye,eye); gl.uniform3fv(u.rightAxis,right); gl.uniform3fv(u.downAxis,down); gl.uniform3fv(u.forwardAxis,forward);
    gl.uniform2fv(u.origin,origin); gl.uniform2f(u.viewport,width,height);
    gl.uniform1f(u.focal,focal); gl.uniform1f(u.nearZ,near); gl.uniform1i(u.perspective,perspective); gl.uniform1i(u.cloud,this.cloud);
    gl.drawArraysInstanced(gl.TRIANGLE_STRIP,0,4,this.order.length);
    ctx.drawImage(this.canvas,0,0,width,height);
  }
}

// Gaussian splats fitted to points, in the splat layout of loadAttributes. s: the splat-fit node's settings
// (operators.py SPLAT_FIT_PARAMS); every number that shapes the result is one of them.
function fitGaussianAttributes(points,s){
    const count=points.length,data=new Float32Array(count*16);
    // Estimate a neighborhood size from the central extent to limit outlier influence.
    let span=0;
    for(let axis=0;axis<3;axis++){
      const values=points.map(p=>p[axis]).sort((a,b)=>a-b);
      const trim=s.extent_trim_pct/100;
      span=Math.max(span,values[Math.min(count-1,Math.floor((1-trim)*count))]-values[Math.floor(trim*count)]);
    }
    const cell=Math.max(1e-8,span/Math.max(s.min_cells,Math.sqrt(count)*s.cells_per_sqrt_point));
    const cells=new Map(),bins=points.map(p=>p.slice(0,3).map(v=>Math.floor(v/cell)));
    bins.forEach((bin,i)=>{
      const key=bin.join(',');
      if(!cells.has(key))cells.set(key,[]);
      cells.get(key).push(i);
    });
    points.forEach((p,i)=>{
      const base=i*16,near=[];
      const [bx,by,bz]=bins[i];
      for(let x=bx-1;x<=bx+1;x++)for(let y=by-1;y<=by+1;y++)for(let z=bz-1;z<=bz+1;z++){
        const bucket=cells.get(x+','+y+','+z);
        if(!bucket)continue;
        const stride=Math.max(1,Math.ceil(bucket.length/s.cell_samples));
        for(let j=0;j<bucket.length;j+=stride){
          const q=points[bucket[j]],dx=q[0]-p[0],dy=q[1]-p[1],dz=q[2]-p[2];
          const d=dx*dx+dy*dy+dz*dz;
          if(d>0)near.push([d,dx,dy,dz]);
        }
      }
      near.sort((a,b)=>a[0]-b[0]);near.length=Math.min(near.length,s.neighbours);
      let xx=0,xy=0,xz=0,yy=0,yz=0,zz=0;
      for(const [,x,y,z] of near){
        xx+=x*x;xy+=x*y;xz+=x*z;yy+=y*y;yz+=y*z;zz+=z*z;
      }
      // The longest covariance cofactor gives the locally fitted plane normal.
      const normals=[[yy*zz-yz*yz,xz*yz-xy*zz,xy*yz-xz*yy],
                     [xz*yz-xy*zz,xx*zz-xz*xz,xy*xz-xx*yz],
                     [xy*yz-xz*yy,xy*xz-xx*yz,xx*yy-xy*xy]];
      let normal=normals.reduce((a,b)=>Math.hypot(...a)>Math.hypot(...b)?a:b);
      const length=Math.hypot(...normal),planar=near.length>=3 && length>1e-8*(xx+yy+zz)**2;
      normal=planar?normal.map(v=>v*(normal[2]<0?-1:1)/length):[0,0,1];
      const radius=Math.max(s.min_radius*cell,Math.min(s.max_radius*cell,near.length?
        Math.sqrt(near[Math.min(s.radius_neighbour-1,near.length-1)][0])*s.radius_scale:s.lone_radius*cell));
      const residual=near.reduce((sum,[,x,y,z])=>sum+(normal[0]*x+normal[1]*y+normal[2]*z)**2,0);
      const thickness=planar?Math.min(radius,Math.max(radius*s.min_thickness,Math.sqrt(residual/near.length))):radius;
      const qw=Math.sqrt((1+normal[2])/2);
      for(let k=0;k<3;k++){
        data[base+k]=p[k];data[base+4+k]=p[k+3]/255;
      }
      data[base+3]=s.opacity;
      data.set([radius,radius,thickness],base+8);
      data.set([qw,-normal[1]/(2*qw),normal[0]/(2*qw),0],base+12);
    });
    return data;
}
