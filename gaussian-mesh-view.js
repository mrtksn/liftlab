// Draw the indexed mesh with a depth buffer in the same camera as the splats.
class GaussianMeshView {
  constructor() {
    this.canvas = document.createElement('canvas');
    const gl = this.canvas.getContext('webgl2', {alpha:true, antialias:true});
    if (!gl) throw Error('Mesh display requires WebGL 2.');
    this.gl = gl;
    const vertex = `#version 300 es
    in vec3 point;
    uniform mat3 camera;
    uniform vec3 eye;
    uniform vec2 viewport, origin;
    uniform float focal, nearZ, span;
    uniform bool perspective;
    out vec3 world;
    void main() {
      vec3 p = camera * (point-eye);
      float w = perspective ? p.z : 1.;
      vec2 xy = (origin/viewport*2.-1.)*w + 2.*focal*p.xy/viewport;
      float z = perspective ? (p.z - 2.*nearZ) : p.z/span;
      gl_Position = vec4(xy.x, -xy.y, z, w);
      world = point;
    }`;
    const fragment = `#version 300 es
    precision highp float;
    in vec3 world;
    out vec4 pixel;
    void main() {
      vec3 n = normalize(cross(dFdx(world), dFdy(world)));
      float light = .36 + .64*abs(dot(n,normalize(vec3(.25,-.45,.86))));
      pixel = vec4(vec3(.39,.79,.78)*light,1.);
    }`;
    this.program = gl.createProgram();
    for (const [type, source] of [[gl.VERTEX_SHADER,vertex],[gl.FRAGMENT_SHADER,fragment]]) {
      const shader=gl.createShader(type);
      gl.shaderSource(shader,source); gl.compileShader(shader);
      if (!gl.getShaderParameter(shader,gl.COMPILE_STATUS)) throw Error(gl.getShaderInfoLog(shader));
      gl.attachShader(this.program,shader); gl.deleteShader(shader);
    }
    gl.linkProgram(this.program);
    if (!gl.getProgramParameter(this.program,gl.LINK_STATUS)) throw Error(gl.getProgramInfoLog(this.program));
    gl.useProgram(this.program);
    this.uniform=Object.fromEntries(['eye','camera','origin','viewport','focal','nearZ','span','perspective']
      .map(key=>[key,gl.getUniformLocation(this.program,key)]));
    gl.bindBuffer(gl.ARRAY_BUFFER,gl.createBuffer());
    const loc=gl.getAttribLocation(this.program,'point');
    gl.enableVertexAttribArray(loc); gl.vertexAttribPointer(loc,3,gl.FLOAT,false,0,0);
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER,gl.createBuffer());
    gl.enable(gl.DEPTH_TEST);
  }

  load(buffer) {
    const start=new TextDecoder().decode(new Uint8Array(buffer,0,Math.min(buffer.byteLength,512)));
    const header=start.match(/^ply\nformat binary_little_endian 1\.0\nelement vertex (\d+)\nproperty float x\nproperty float y\nproperty float z\nelement face (\d+)\nproperty list uchar int vertex_indices\nend_header\n/);
    if (!header) throw Error('Unrecognized mesh format.');
    const vertices=Number(header[1]), faces=Number(header[2]), offset=header[0].length;
    if (offset+12*vertices+13*faces!==buffer.byteLength) throw Error('Incomplete mesh file.');
    const data=new DataView(buffer,offset), xyz=new Float32Array(vertices*3);
    this.vertices=[]; this.radius=0;
    for (let i=0;i<vertices;i++) {
      const p=[0,1,2].map(k=>data.getFloat32(12*i+4*k,true));
      this.vertices.push(p); xyz.set(p,3*i);
      this.radius=Math.max(this.radius,Math.hypot(...p));
    }
    this.indices=new Uint32Array(faces*3);
    for (let i=0;i<faces;i++) {
      const base=12*vertices+13*i;
      if(data.getUint8(base)!==3)throw Error('Expected triangular faces.');
      for(let k=0;k<3;k++){
        const index=data.getUint32(base+1+4*k,true);
        if(index>=vertices)throw Error('Invalid mesh index.');
        this.indices[3*i+k]=index;
      }
    }
    const gl=this.gl;
    gl.bufferData(gl.ARRAY_BUFFER,xyz,gl.STATIC_DRAW);
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER,this.indices,gl.STATIC_DRAW);
  }

  paint(ctx,w,h,eye,axes,origin,focal,perspective,near,overlay) {
    const gl=this.gl,u=this.uniform;
    if(this.canvas.width!==w || this.canvas.height!==h){this.canvas.width=w; this.canvas.height=h;}
    gl.viewport(0,0,w,h); gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
    gl.uniformMatrix3fv(u.camera,false,[0,1,2].flatMap(k=>axes.map(axis=>axis[k])));
    gl.uniform3fv(u.eye,eye); gl.uniform2fv(u.origin,origin); gl.uniform2f(u.viewport,w,h);
    gl.uniform1f(u.focal,focal); gl.uniform1f(u.nearZ,near);
    gl.uniform1f(u.span,this.radius+Math.hypot(...eye)+1); gl.uniform1i(u.perspective,perspective);
    gl.drawElements(gl.TRIANGLES,this.indices.length,gl.UNSIGNED_INT,0);
    ctx.save(); ctx.globalAlpha=overlay?.53:1;
    ctx.drawImage(this.canvas,0,0,w,h); ctx.restore();
  }

  indexVisibility(visibility,project,cameraPoint,near,projectCamera) {
    const screen=this.vertices.map(cameraPoint||project);
    for(let i=0;i<this.indices.length;i+=3){
      let face=[screen[this.indices[i]],screen[this.indices[i+1]],screen[this.indices[i+2]]];
      if(cameraPoint)face=clipNear(face,near).map(projectCamera);
      visibility.addFace(face,[]);
    }
  }
}
