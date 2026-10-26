import {STLLoader} from '../vendor/STLLoader.js?v=20261026-load-2';

const parser=new STLLoader();
self.onmessage=async({data:{id,path,compressed}})=>{
  try{
    let response;
    if(compressed&&typeof DecompressionStream==='function'){
      try{
        const packed=await fetch(compressed);
        if(!packed.ok)throw new Error(`HTTP ${packed.status}`);
        response=await new Response(packed.body.pipeThrough(new DecompressionStream('gzip'))).arrayBuffer();
      }catch{/* Original asset remains available during partial deployments. */}
    }
    if(!response){const raw=await fetch(path);if(!raw.ok)throw new Error(`模型资源加载失败（${raw.status}）`);response=await raw.arrayBuffer();}
    const geometry=parser.parse(response),attributes={},transfer=[];
    for(const [name,attribute]of Object.entries(geometry.attributes)){
      attributes[name]={array:attribute.array,itemSize:attribute.itemSize,normalized:attribute.normalized};transfer.push(attribute.array.buffer);
    }
    self.postMessage({id,attributes,groups:geometry.groups,hasColors:geometry.hasColors,alpha:geometry.alpha},transfer);geometry.dispose();
  }catch(error){self.postMessage({id,error:`${path.split('/').at(-1)}：${error.message}`});}
};
