// Per robot-load cache: repeated left/right finger or arm assets parse once.
// Mesh nodes and materials are cloned; immutable geometry is shared only inside
// this robot, so existing per-station Set-based disposal retains sole ownership.
export function cacheRobotMeshes(loader){
  const original=loader.loadMeshCb.bind(loader),pending=new Map();
  const stats={requests:0,loads:0,hits:0};
  loader.loadMeshCb=(path,manager,done)=>{
    stats.requests++;
    let promise=pending.get(path);
    // Only STL is a single immutable geometry; leave Collada/3DS material and
    // scene ownership with their existing loaders.
    if(!/\.stl(?:[?#]|$)/i.test(path)){stats.loads++;return original(path,manager,done);}
    manager.itemStart(path);
    if(promise)stats.hits++;
    else{
      stats.loads++;
      promise=new Promise((resolve,reject)=>{
        try{original(path,manager,(object,error)=>error||!object?reject(error??new Error(`零件为空：${path}`)):resolve(object));}
        catch(error){reject(error);}
      });
      pending.set(path,promise);promise.catch(()=>pending.delete(path));
    }
    promise.then(object=>{
      const clone=object.clone(true);
      clone.traverse(node=>{if(node.material)node.material=Array.isArray(node.material)?node.material.map(m=>m.clone()):node.material.clone();});
      done(clone);
    },error=>{manager.itemError(path);done(null,error);}).finally(()=>manager.itemEnd(path));
  };
  return stats;
}
