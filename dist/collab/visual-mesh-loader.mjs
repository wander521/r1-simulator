import * as THREE from '../vendor/three.module.min.js';

/** Two decoders bound memory and main-thread work. Cache spans the three visual
 * robots because the collaboration view owns and disposes all their geometry. */
export function createVisualMeshLoader({onGeometry=()=>{},workerFactory=()=>new Worker(new URL('./visual-mesh-worker.mjs?v=20261026-load-2',import.meta.url),{type:'module'})}={}){
  const cache=new Map(),queue=[],slots=[];let sequence=0,closed=false;
  const manifest=fetch(new URL('./visual-assets.json?v=20261026-load-1',import.meta.url)).then(r=>r.ok?r.json():{}).catch(()=>({})).then(map=>new Map(Object.entries(map).map(([path,spec])=>[new URL(path,import.meta.url).href,new URL(spec.file,import.meta.url).href])));
  const stats={requests:0,loads:0,hits:0};
  function pump(){
    if(closed)return;
    while(queue.length){
      let slot=slots.find(s=>!s.job);
      if(!slot&&slots.length<2){
        const worker=workerFactory();slot={worker,job:null};slots.push(slot);
        worker.onmessage=({data})=>{
          const job=slot.job;if(!job||data.id!==job.id)return;slot.job=null;
          if(data.error)job.reject(new Error(data.error));
          else{
            const geometry=new THREE.BufferGeometry();
            for(const [name,a]of Object.entries(data.attributes))geometry.setAttribute(name,new THREE.BufferAttribute(a.array,a.itemSize,a.normalized));
            for(const g of data.groups??[])geometry.addGroup(g.start,g.count,g.materialIndex);
            if(data.hasColors){geometry.hasColors=true;geometry.alpha=data.alpha;}
            onGeometry(geometry);job.resolve(geometry);
          }
          pump();
        };
        worker.onerror=event=>{const error=new Error(event.message||'模型解码失败');slot.job?.reject(error);slot.job=null;worker.terminate();slots.splice(slots.indexOf(slot),1);pump();};
      }
      if(!slot)return;
      const job=queue.shift();slot.job=job;slot.worker.postMessage({id:job.id,path:job.path,compressed:job.compressed});
    }
  }
  function geometryFor(path){
    // URDF paths can contain ../ segments (C5 reuses the G1 Dex1 assets).
    path=new URL(path,import.meta.url).href;
    if(closed)return Promise.reject(new Error('模型加载器已关闭'));
    if(cache.has(path)){stats.hits++;return cache.get(path);}
    stats.loads++;
    const pending=manifest.then(map=>new Promise((resolve,reject)=>{
      if(closed){reject(new Error('模型加载器已关闭'));return;}
      queue.push({id:++sequence,path,compressed:map.get(path),resolve,reject});
      try{pump();}catch(error){reject(error);}
    }));
    cache.set(path,pending);pending.catch(()=>cache.delete(path));return pending;
  }
  return {stats,
    load(path,manager,done){
      if(!/\.stl(?:[?#]|$)/i.test(path))return false;
      stats.requests++;manager.itemStart(path);
      geometryFor(path).then(geometry=>done(new THREE.Mesh(geometry,new THREE.MeshPhongMaterial())),error=>{manager.itemError(path);done(null,error);}).finally(()=>manager.itemEnd(path));
      return true;
    },
    releaseWorkers(){for(const slot of slots)slot.worker.terminate();slots.length=0;},
    dispose(){closed=true;const error=new Error('模型加载器已关闭');for(const job of queue.splice(0))job.reject(error);for(const slot of slots){slot.job?.reject(error);slot.worker.terminate();}slots.length=0;cache.clear();},
  };
}
