const DEFAULT_MAX_ENTRIES=512;
const DEFAULT_MAX_BYTES=2*1024*1024;

// Exact, length-delimited input encoding, not a rounded numeric fingerprint.
// Preserve object/array order, signed zero, missing properties and undefined.
// Unusual inputs bypass memoization and retain the original check's behavior.
function exactKey(value){
  const ancestors=new Set();
  const visit=item=>{
    if(item===null)return'n';
    if(item===undefined)return'u';
    if(typeof item==='boolean')return item?'t':'f';
    if(typeof item==='string')return`s${item.length}:${item}`;
    if(typeof item==='number'){
      if(!Number.isFinite(item))throw new TypeError('Nonfinite collision-check input');
      return`d${Object.is(item,-0)?'-0':String(item)};`;
    }
    if(typeof item!=='object'||ancestors.has(item))throw new TypeError('Unsupported collision-check input');
    const array=Array.isArray(item),prototype=Object.getPrototypeOf(item);
    if(array?prototype!==Array.prototype:prototype!==Object.prototype&&prototype!==null)throw new TypeError('Nonplain collision-check input');
    ancestors.add(item);
    const entries=[];
    for(const key of Reflect.ownKeys(item)){
      if(typeof key!=='string')throw new TypeError('Symbol collision-check input');
      const descriptor=Object.getOwnPropertyDescriptor(item,key);
      if(array&&key==='length')continue;
      if(!descriptor||!Object.hasOwn(descriptor,'value'))throw new TypeError('Accessor collision-check input');
      if(!descriptor.enumerable)throw new TypeError('Hidden collision-check input');
      entries.push(`${visit(key)}${visit(descriptor.value)}`);
    }
    const result=`${array?`a${item.length}`:prototype===null?'p':'o'}${entries.length}{${entries.join('')}}`;
    ancestors.delete(item);return result;
  };
  try{return visit(value);}catch{return null;}
}

/** A bounded memo for ONE isolated, immutable worker planning snapshot.
 *
 * G1 checkPose resets its private scratch data, copies live qpos/ctrl, then
 * derives all contacts from the full ordered arguments. Neither the scratch
 * state nor a cached result drives execution. Compiled model/safety metadata
 * are private to the worker and immutable for this job; changed identities or
 * live qpos/ctrl invalidate the memo permanently. Always dispose in the job's
 * finally block; never install this on the authoritative simulation runtime.
 */
export function createPlanningCheckCache(dynamics,{maxEntries=DEFAULT_MAX_ENTRIES,maxBytes=DEFAULT_MAX_BYTES}={}){
  if(!Number.isInteger(maxEntries)||maxEntries<1||!Number.isFinite(maxBytes)||maxBytes<1)throw new Error('Invalid planning-cache bounds');
  const original=dynamics.checkPose,model=dynamics.model,data=dynamics.data;
  if(typeof original!=='function'||!data?.qpos||!data?.ctrl)throw new Error('Missing planning collision-check context');
  const qpos=Array.from(data.qpos),ctrl=Array.from(data.ctrl),map=dynamics.map,robotBodies=dynamics.robotBodyIds,footBodies=dynamics.footBodyIds,floor=dynamics.floorGeom;
  const cache=new Map(),stats={requests:0,evaluations:0,hits:0,bypasses:0,evictions:0,invalidations:0,entries:0,bytes:0,peakEntries:0,peakBytes:0,disposed:false};
  let invalidated=false;
  const same=(actual,expected)=>{
    if(!actual||actual.length!==expected.length)return false;
    for(let i=0;i<expected.length;i++)if(!Object.is(actual[i],expected[i]))return false;
    return true;
  };
  const clear=()=>{cache.clear();stats.entries=0;stats.bytes=0;};
  const wrapped=function(...args){
    if(stats.disposed)return original.apply(this,args);
    stats.requests++;
    if(!invalidated&&(this!==dynamics||this.model!==model||this.data!==data||this.map!==map||this.robotBodyIds!==robotBodies||this.footBodyIds!==footBodies||this.floorGeom!==floor||!same(data.qpos,qpos)||!same(data.ctrl,ctrl))){
      clear();invalidated=true;stats.invalidations++;
    }
    const key=invalidated?null:exactKey(args);
    if(key!==null&&cache.has(key)){
      const row=cache.get(key);cache.delete(key);cache.set(key,row);stats.hits++;
      return structuredClone(row.result);
    }
    stats.evaluations++;
    const result=original.apply(this,args);
    if(key===null){stats.bypasses++;return result;}
    const resultKey=exactKey(result),bytes=resultKey===null?Infinity:2*(key.length+resultKey.length);
    if(bytes>maxBytes){stats.bypasses++;return result;}
    let stored;
    try{stored=structuredClone(result);}catch{stats.bypasses++;return result;}
    // Structured clone intentionally normalizes some unusual plain objects
    // (for example a null prototype). Never memoize a changed representation.
    if(exactKey(stored)!==resultKey){stats.bypasses++;return result;}
    while(cache.size&&(cache.size>=maxEntries||stats.bytes+bytes>maxBytes)){
      const [oldKey,row]=cache.entries().next().value;cache.delete(oldKey);stats.bytes-=row.bytes;stats.evictions++;
    }
    cache.set(key,{result:stored,bytes});stats.entries=cache.size;stats.bytes+=bytes;
    stats.peakEntries=Math.max(stats.peakEntries,stats.entries);stats.peakBytes=Math.max(stats.peakBytes,stats.bytes);
    return result;
  };
  dynamics.checkPose=wrapped;
  return{stats,dispose(){
    clear();stats.disposed=true;
    if(dynamics.checkPose===wrapped)dynamics.checkPose=original;
  }};
}
