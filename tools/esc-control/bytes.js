/* Lossless payload conversions. Canonical data always remains hexadecimal bytes. */
(function(root,factory){const api=factory();if(typeof module==='object')module.exports=api;else root.SignalBytes=api;})(globalThis,()=>{
'use strict';
const FORMATS={hex:'Hex bytes',text:'Text / UTF-8 + escapes',uint8:'Unsigned 8-bit integers',int8:'Signed 8-bit integers',uint16:'Unsigned 16-bit integers',int16:'Signed 16-bit integers',uint32:'Unsigned 32-bit integers',int32:'Signed 32-bit integers',float32:'32-bit floating point',float64:'64-bit floating point'};
const TYPES={uint8:[1,'Uint8',0,255],int8:[1,'Int8',-128,127],uint16:[2,'Uint16',0,65535],int16:[2,'Int16',-32768,32767],uint32:[4,'Uint32',0,4294967295],int32:[4,'Int32',-2147483648,2147483647],float32:[4,'Float32'],float64:[8,'Float64']};
function fromHex(text){if(typeof text!=='string')throw Error('Hex data must be text.');const hex=text.replace(/0x/gi,'').replace(/[\s,]/g,'');if(hex.length%2||!/^[a-f0-9]*$/i.test(hex))throw Error('Enter complete hex bytes, such as 31 32 33 or 0x31, 0x32.');return Uint8Array.from(hex.match(/../g)||[],b=>parseInt(b,16));}
const toHex=bytes=>Array.from(bytes,b=>b.toString(16).padStart(2,'0')).join('').toUpperCase();
function textBytes(text){
 const out=[],encoder=new TextEncoder();let literal='';const flush=()=>{out.push(...encoder.encode(literal));literal='';};
 for(let i=0;i<text.length;i++){
  if(text[i]!=='\\'){literal+=text[i];continue;}flush();const next=text[++i];
  const escapes={'n':10,'r':13,'t':9,'0':0,'\\':92};
  if(next in escapes){out.push(escapes[next]);continue;}
  if(next==='x'&&/^[a-f0-9]{2}$/i.test(text.slice(i+1,i+3))){out.push(parseInt(text.slice(i+1,i+3),16));i+=2;continue;}
  throw Error('Text escapes: \\n, \\r, \\t, \\0, \\\\ or \\xNN. Escape a literal backslash as \\\\.');
 }flush();return Uint8Array.from(out);
}
function displayText(bytes){
 let text;try{text=new TextDecoder('utf-8',{fatal:true,ignoreBOM:true}).decode(bytes);}catch{return Array.from(bytes,b=>b>=32&&b<127?(b===92?'\\\\':String.fromCharCode(b)):b===10?'\\n':b===13?'\\r':b===9?'\\t':b===0?'\\0':'\\x'+b.toString(16).padStart(2,'0').toUpperCase()).join('');}
 return Array.from(text,c=>c==='\\'?'\\\\':c==='\n'?'\\n':c==='\r'?'\\r':c==='\t'?'\\t':c==='\0'?'\\0':c.charCodeAt(0)<32||c.charCodeAt(0)===127?'\\x'+c.charCodeAt(0).toString(16).padStart(2,'0').toUpperCase():c).join('');
}
function encode(text,format='hex',endian='little',maxBytes=12){
 let bytes;
 if(format==='hex')bytes=fromHex(text);
 else if(format==='text')bytes=textBytes(text);
 else{
  const type=TYPES[format];if(!type)throw Error('Choose a byte format.');
  const tokens=text.trim().split(/[\s,]+/).filter(Boolean),[width,set,min,max]=type;bytes=new Uint8Array(tokens.length*width);const view=new DataView(bytes.buffer);
  for(let i=0;i<tokens.length;i++){
   const value=Number(tokens[i]);if(!Number.isFinite(value)||(min!==undefined&&(!Number.isInteger(value)||value<min||value>max)))throw Error(`Invalid ${FORMATS[format]} value: ${tokens[i]}`);
   view['set'+set](i*width,value,endian==='little');if(!Number.isFinite(view['get'+set](i*width,endian==='little')))throw Error('The number overflows the selected float format.');
  }
 }
 if(!bytes.length)throw Error('Enter at least one byte.');if(bytes.length>maxBytes)throw Error(`This signal supports at most ${maxBytes} bytes; this value uses ${bytes.length}.`);
 return toHex(bytes);
}
function decode(hex,format='hex',endian='little',allowPartial=false){
 const bytes=fromHex(hex);if(format==='hex')return Array.from(bytes,b=>b.toString(16).padStart(2,'0').toUpperCase()).join(' ');if(format==='text')return displayText(bytes);
 const type=TYPES[format];if(!type)throw Error('Choose a byte format.');const [width,get]=type;
 if(bytes.length%width&&!allowPartial)throw Error(`${FORMATS[format]} needs groups of ${width} bytes. Replace the value or use Hex / Text.`);
 const view=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength),values=[];for(let i=0;i+width<=bytes.length;i+=width)values.push(String(view['get'+get](i,endian==='little')));
 return values.join(', ');
}
function inspect(hex,format,endian,offset=0){const bytes=fromHex(hex).slice(offset),width=TYPES[format]?.[0]||1,remainder=bytes.length%width;return {text:decode(toHex(bytes),format,endian,true),count:Math.floor(bytes.length/width),remainder,bytes:bytes.length};}
return {FORMATS,TYPES,fromHex,toHex,encode,decode,inspect};
});
