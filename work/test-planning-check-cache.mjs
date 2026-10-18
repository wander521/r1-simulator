import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {createPlanningCheckCache} from '../dist/multi/planning-check-cache.mjs';

const root=new URL('../',import.meta.url),args=process.argv.slice(2);
const output=args.find(value=>value.startsWith('--report='))?.slice(9)??'work/validation/performance-v2/planning-check-cache-contracts.json';
const paths=['dist/multi/planning-check-cache.mjs','work/test-planning-check-cache.mjs'];
const sha=value=>createHash('sha256').update(value).digest('hex');
const hashes=async()=>Object.fromEntries(await Promise.all(paths.map(async path=>[path,sha(await readFile(new URL(path,root)))])));
const sourceHashes=await hashes(),checks=[];
const check=(name,run)=>{run();checks.push({name,pass:true});};
function fixture(evaluate=pose=>({safe:pose?.x!==2,contacts:[{dist:pose?.x??0}],limitViolations:[]})){
  const dynamics={model:{},data:{qpos:new Float64Array([0,1]),ctrl:new Float64Array([0])},map:new Map(),robotBodyIds:new Set(),footBodyIds:new Set(),floorGeom:0,calls:0,
    checkPose(...values){this.calls++;return evaluate(...values);}};
  return dynamics;
}
check('Repeated exact full inputs reuse actual results including unsafe contacts',()=>{
  const d=fixture(),memo=createPlanningCheckCache(d);
  try{for(const x of [1,2]){const first=d.checkPose({x},{opening:{left:.01,right:.02},ignoreObjectBodies:[1,2],objectPoses:[]});
    const second=d.checkPose({x},{opening:{left:.01,right:.02},ignoreObjectBodies:[1,2],objectPoses:[]});assert.deepEqual(first,second);}
    assert.equal(d.calls,2);assert.equal(memo.stats.hits,2);
  }finally{memo.dispose();}
});
check('Consumers cannot mutate either stored results or later returned clones',()=>{
  const d=fixture(),memo=createPlanningCheckCache(d);
  try{d.checkPose({x:1}).contacts[0].dist=99;const hit=d.checkPose({x:1});assert.equal(hit.contacts[0].dist,1);hit.contacts.push({dist:3});
    assert.equal(d.checkPose({x:1}).contacts.length,1);assert.equal(d.calls,1);
  }finally{memo.dispose();}
});
check('Signed zero is lossless, and property/array order and missing arguments remain distinct',()=>{
  const d=fixture(),memo=createPlanningCheckCache(d);
  try{const inputs=[[{x:0}],[{x:-0}],[{x:1}],[{x:1},undefined],[{x:1},{}],[{x:1},{opening:undefined}],
    [{a:1,b:2}],[{b:2,a:1}],[{x:1},{ignoreObjectBodies:[1,2]}],[{x:1},{ignoreObjectBodies:[2,1]}]];
    for(const values of inputs)d.checkPose(...values);assert.equal(d.calls,inputs.length);
    for(const values of inputs)d.checkPose(...values);assert.equal(d.calls,inputs.length);assert.equal(memo.stats.hits,inputs.length);
    assert(Object.is(d.checkPose({x:-0}).contacts[0].dist,-0));
  }finally{memo.dispose();}
});
check('Nonfinite, functions, cycles and nonplain inputs bypass without changing original validation',()=>{
  const d=fixture(()=>({safe:true,contacts:[],limitViolations:[]})),memo=createPlanningCheckCache(d),cycle={};cycle.self=cycle;
  try{const inputs=[{x:NaN},{x:Infinity},{x:()=>0},cycle,{x:new Date(0)},{x:new Float64Array([1])}];
    for(const value of inputs){d.checkPose(value);d.checkPose(value);}assert.equal(d.calls,inputs.length*2);assert.equal(memo.stats.hits,0);
  }finally{memo.dispose();}
});
check('Key construction never invokes accessors before the original implementation does',()=>{
  let reads=0;const pose={get x(){reads++;return 1;}},d=fixture(value=>({safe:true,contacts:[{dist:value.x}]})),memo=createPlanningCheckCache(d);
  try{d.checkPose(pose);d.checkPose(pose);assert.equal(reads,2);assert.equal(d.calls,2);assert.equal(memo.stats.hits,0);}
  finally{memo.dispose();}
});
check('Hidden, symbol and custom-array behavior cannot alias ordinary literal inputs',()=>{
  const d=fixture((pose,options={})=>({safe:true,contacts:[{dist:options.opening??0}]})),memo=createPlanningCheckCache(d);
  try{
    assert.equal(d.checkPose({x:1},{}).contacts[0].dist,0);
    const hidden=Object.defineProperty({},'opening',{value:.01});
    assert.equal(d.checkPose({x:1},hidden).contacts[0].dist,.01);assert.equal(d.checkPose({x:1},hidden).contacts[0].dist,.01);
    const symbol={opening:.02,[Symbol('extra')]:1},hiddenIndex=[1,2],custom=[1,2],iterator=[1,2];
    Object.defineProperty(hiddenIndex,'0',{value:1,enumerable:false});Object.setPrototypeOf(custom,Object.create(Array.prototype));
    iterator[Symbol.iterator]=function*(){yield 9;};
    for(const value of [symbol,{objectPoses:hiddenIndex},{objectPoses:custom},{objectPoses:iterator}]){d.checkPose({x:1},value);d.checkPose({x:1},value);}
    assert.equal(d.calls,11);assert.equal(memo.stats.hits,0);
  }finally{memo.dispose();}
});
for(const field of ['qpos','ctrl'])check(`Changing live ${field} clears the cache and permanently bypasses the job`,()=>{
  const d=fixture(()=>({safe:true,contacts:[{dist:d.data[field][0]}]})),memo=createPlanningCheckCache(d);
  try{d.checkPose({x:1});d.data[field][0]=5;assert.equal(d.checkPose({x:1}).contacts[0].dist,5);
    d.data[field][0]=0;assert.equal(d.checkPose({x:1}).contacts[0].dist,0);assert.equal(d.calls,3);assert.equal(memo.stats.invalidations,1);assert.equal(memo.stats.entries,0);
  }finally{memo.dispose();}
});
for(const field of ['model','data','map','robotBodyIds','footBodyIds','floorGeom'])check(`Replacing ${field} invalidates the private job epoch`,()=>{
  const d=fixture(),memo=createPlanningCheckCache(d);
  try{d.checkPose({x:1});d[field]=field==='data'?{qpos:new Float64Array([0,1]),ctrl:new Float64Array([0])}:field==='floorGeom'?1:{};
    d.checkPose({x:1});assert.equal(d.calls,2);assert.equal(memo.stats.invalidations,1);assert.equal(memo.stats.entries,0);
  }finally{memo.dispose();}
});
check('Bounded LRU entries retain hot checks and evict old results without changing them',()=>{
  const d=fixture(),memo=createPlanningCheckCache(d,{maxEntries:2});
  try{for(const x of [1,2,1,3,1,2])assert.deepEqual(d.checkPose({x}),{safe:x!==2,contacts:[{dist:x}],limitViolations:[]});
    assert.equal(d.calls,4);assert.equal(memo.stats.peakEntries,2);assert.equal(memo.stats.hits,2);assert.equal(memo.stats.evictions,2);
  }finally{memo.dispose();}
});
check('The byte bound evicts results, and oversized diagnostics are not retained',()=>{
  const d=fixture(pose=>({safe:false,contacts:[{dist:0,label:'x'.repeat(pose.x)}]})),memo=createPlanningCheckCache(d,{maxEntries:512,maxBytes:300});
  try{for(const x of [1,2,3,4])d.checkPose({x});assert(memo.stats.peakBytes<=300);assert(memo.stats.evictions>0);
    d.checkPose({x:1000});d.checkPose({x:1000});assert(memo.stats.bypasses>=2);assert(memo.stats.bytes<=300);
  }finally{memo.dispose();}
});
check('Unsupported return values bypass storage and preserve the original value',()=>{
  const result={safe:true,callback(){}},d=fixture(()=>result),memo=createPlanningCheckCache(d);
  try{assert.equal(d.checkPose({x:1}),result);assert.equal(d.checkPose({x:1}),result);assert.equal(d.calls,2);assert.equal(memo.stats.entries,0);}
  finally{memo.dispose();}
  const unusual=Object.assign(Object.create(null),{safe:true}),other=fixture(()=>unusual),guard=createPlanningCheckCache(other);
  try{assert.equal(other.checkPose({x:1}),unusual);assert.equal(other.checkPose({x:1}),unusual);assert.equal(other.calls,2);}
  finally{guard.dispose();}
});
check('A cancelled or failed job restores the exact original method and frees all entries',()=>{
  const d=fixture(),original=d.checkPose,memo=createPlanningCheckCache(d);
  assert.throws(()=>{try{d.checkPose({x:1});throw Error('cancelled');}finally{memo.dispose();}},/cancelled/);
  assert.equal(d.checkPose,original);assert.equal(memo.stats.disposed,true);assert.equal(memo.stats.entries,0);assert.equal(memo.stats.bytes,0);memo.dispose();
  const broken=fixture(()=>{throw Error('original error');}),failing=createPlanningCheckCache(broken);
  try{assert.throws(()=>broken.checkPose({x:1}),/original error/);assert.equal(failing.stats.entries,0);}finally{failing.dispose();}
});
check('Bounds reject invalid configuration before changing the original method',()=>{
  const d=fixture(),original=d.checkPose;for(const options of [{maxEntries:0},{maxEntries:1.5},{maxBytes:0},{maxBytes:Infinity}])assert.throws(()=>createPlanningCheckCache(d,options));
  assert.equal(d.checkPose,original);
});
const endSourceHashes=await hashes(),sourceFrozen=JSON.stringify(sourceHashes)===JSON.stringify(endSourceHashes);
const report={pass:sourceFrozen,sourceFrozen,sourceHashes,endSourceHashes,checks};
await mkdir(new URL('.',new URL(output,root)),{recursive:true});await writeFile(new URL(output,root),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({pass:report.pass,checks:checks.length,sourceFrozen,report:output}));
